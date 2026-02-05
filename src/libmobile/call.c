/* interface between mobile network/phone implementation and OsmoCC
 *
 * (C) 2016 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libtones/tones.h"
#include <osmocom/core/timer.h>
#include <osmocom/core/select.h>
#include <osmocom/cc/endpoint.h>
#include <osmocom/cc/helper.h>
#include <osmocom/cc/session.h>
#include <osmocom/cc/g711.h>
#include <osmocom/cc/rtp.h>
#include "cause.h"
#include "sender.h"
#include "call.h"
#include "console.h"
#include "../libecho/suppressor.h"
#include "../libdtmf/dtmf_decode.h"
#include "../libdtmf/dtmf_encode.h"
#include "../liboptions/options.h"

/* External configuration */
extern echo_suppressor_config_t echo_suppressor_config;

#define DISC_TIMEOUT	30, 0

//#define DEBUG_LEVEL

#ifdef DEBUG_LEVEL
static double level_of(double *samples, int count)
{
	double level = 0;
	int i;

	for (i = 0; i < count; i++) {
		if (samples[i] > level)
			level = samples[i];
	}

	return level;
}
#endif

static int connect_on_setup;		/* send patterns towards fixed network */
static int release_on_disconnect;	/* release towards mobile phone, if OSMO-CC call disconnects, don't send disconnect tone */

osmo_cc_endpoint_t endpoint, *ep;

static tones_data_t call_tones;

void encode_l16(uint8_t *src_data, int src_len, uint8_t **dst_data, int *dst_len, void __attribute__((unused)) *arg)
{
	uint16_t *src = (uint16_t *)src_data, *dst;
	int len = src_len / 2, i;

	dst = malloc(len * 2);
	if (!dst)
		return;
	for (i = 0; i < len; i++)
		dst[i] = htons(src[i]);
	*dst_data = (uint8_t *)dst;
	*dst_len = len * 2;
}

void decode_l16(uint8_t *src_data, int src_len, uint8_t **dst_data, int *dst_len, void __attribute__((unused)) *arg)
{
	uint16_t *src = (uint16_t *)src_data, *dst;
	int len = src_len / 2, i;

	dst = malloc(len * 2);
	if (!dst)
		return;
	for (i = 0; i < len; i++)
		dst[i] = ntohs(src[i]);
	*dst_data = (uint8_t *)dst;
	*dst_len = len * 2;
}

static struct osmo_cc_helper_audio_codecs codecs[] = {
	{ "L16", NULL, 8000, 1, encode_l16, decode_l16, NULL, NULL },
	{ "PCMA", NULL, 8000, 1, g711_encode_alaw, g711_decode_alaw, NULL, NULL },
	{ "PCMU", NULL, 8000, 1, g711_encode_ulaw, g711_decode_ulaw, NULL, NULL },
	{ "telephone-event", NULL, 8000, 1, NULL, NULL, NULL, NULL },  /* RFC 4733 DTMF/Flash */
	{ NULL, NULL, 0, 0, NULL, NULL, NULL, NULL},
};

static int no_l16 = 0;

enum process_state {
	PROCESS_IDLE = 0,	/* IDLE */
	PROCESS_SETUP_RO,	/* call from radio to OSMO-CC */
	PROCESS_SETUP_RT,	/* call from OSMO-CC to radio */
	PROCESS_ALERTING_RO,	/* call from radio to OSMO-CC */
	PROCESS_ALERTING_RT,	/* call from OSMO-CC to radio */
	PROCESS_CONNECT,
	PROCESS_DISCONNECT,
};

/* call process */
typedef struct process {
	struct process *next;
	int callref;
	enum process_state state;
	int audio_disconnected; /* if not associated with transceiver anymore */
	tones_t tones;
	uint8_t cause;
	struct osmo_timer_list timer;
	osmo_cc_session_t *session;
	osmo_cc_session_codec_t *codec; /* codec to send */
	osmo_cc_session_media_t *media; /* audio media (for RFC 4733 support) */
	
	/* Echo suppressor */
	int echo_suppressor_enabled;
	echo_suppressor_wrapper_t echo_suppressor;
	
	/* DTMF decoder (RX from mobile) */
	dtmf_dec_t dtmf_dec;
	int dtmf_dec_enabled;
	
	/* DTMF encoder (TX to mobile) */
	dtmf_enc_t dtmf_enc;
	char dtmf_tx_queue[33];
	double dtmf_tone_duration;
	double dtmf_gap_duration;
	int dtmf_tx_active;
	
	/* Flash/Hold state tracking
	 * Tracks whether the local mobile user has put the remote party on hold.
	 * Used to toggle between SUSPENDED and RESUMED on subsequent flash presses.
	 */
	int remote_on_hold;	/* 1 = remote party is on hold (we sent SUSPENDED) */
} process_t;

static process_t *process_head = NULL;

static void process_timeout(void *data);
static void indicate_disconnect_release(int callref, int cause, uint8_t msg_type);
static void call_echo_suppressor_cleanup(process_t *process);
static void call_dtmf_cleanup(process_t *process);

/* RFC 4733 event codes */
#define RFC4733_EVENT_FLASH 16

/* Local RFC 4733 RX callback - handles DTMF/Flash received via RTP directly.
 * This is called when RFC 4733 telephone-event packets are received from the network.
 * Instead of sending osmo-cc messages (which would go back to SIP endpoint),
 * we process the events locally: queue DTMF for TX to mobile, or handle flash.
 */
static void call_rfc4733_rx_cb(osmo_cc_session_media_t *media, uint8_t event, uint16_t __attribute__((unused)) duration)
{
	process_t *process;
	static const char dtmf_chars[] = "0123456789*#ABCD";

	if (!media || !media->session || !media->session->priv) {
		LOGP(DCALL, LOGL_ERROR, "RFC 4733 RX: no media/session/priv\n");
		return;
	}

	process = (process_t *)media->session->priv;

	if (event == RFC4733_EVENT_FLASH) {
		/* Flash from network - play recall tone to mobile */
		LOGP(DCALL, LOGL_INFO, "RFC 4733 Flash received from network (callref=%d)\n", process->callref);
		call_tone_recall(process->callref, 1);
	} else if (event <= 15) {
		/* DTMF from network - queue for TX to mobile */
		char digit = dtmf_chars[event];
		char digits[2] = { digit, '\0' };
		LOGP(DCALL, LOGL_INFO, "RFC 4733 DTMF '%c' received from network -> TX to mobile (callref=%d)\n",
		     digit, process->callref);
		call_dtmf_queue(process->callref, digits);
	} else {
		LOGP(DCALL, LOGL_DEBUG, "RFC 4733 unsupported event %d ignored (callref=%d)\n",
		     event, process->callref);
	}
}

/* Set up RFC 4733 telephone-event support for a process after codec negotiation.
 * This enables receiving DTMF/Flash via RTP telephone-event packets.
 * Must be called after process->codec is set and session is established.
 */
static void call_setup_rfc4733(process_t *process)
{
	osmo_cc_session_media_t *media;

	if (!process || !process->session || !process->codec)
		return;

	/* Get the media from the codec */
	media = process->codec->media;
	if (!media)
		return;

	/* Store media pointer for sending RFC 4733 events */
	process->media = media;

	/* Check if telephone-event was negotiated */
	if (!media->telephone_event_negotiated) {
		LOGP(DCALL, LOGL_NOTICE, "RFC 4733 not available - telephone-event not negotiated (callref=%d)\n",
		     process->callref);
		return;
	}

	/* Set up local RFC 4733 receive callback - handles DTMF/Flash directly
	 * instead of sending osmo-cc messages back to SIP endpoint */
	media->event_cb = call_rfc4733_rx_cb;
	media->event_cb_data = NULL;  /* We use media->session->priv instead */

	LOGP(DCALL, LOGL_INFO, "RFC 4733 enabled for DTMF/Flash signaling (callref=%d, pt=%d)\n",
	     process->callref, media->telephone_event_pt);
}

static process_t *create_process(int callref, enum process_state state)
{
	process_t *process;

	process = calloc(1, sizeof(*process));
	if (!process) {
		LOGP(DCALL, LOGL_ERROR, "No memory!\n");
		abort();
	}
	osmo_timer_setup(&process->timer, process_timeout, process);
	process->next = process_head;
	process_head = process;

	process->callref = callref;
	process->state = state;
	tones_set_tone(&call_tones, &process->tones, TONES_TONE_OFF);
	
	/* Initialize echo suppressor fields */
	process->echo_suppressor_enabled = 0;
	memset(&process->echo_suppressor, 0, sizeof(process->echo_suppressor));

	return process;
}

static void destroy_process(int callref)
{
	process_t *process = process_head;
	process_t **process_p = &process_head;

	while (process) {
		if (process->callref == callref) {
			*process_p = process->next;
			osmo_timer_del(&process->timer);
			if (process->session)
				osmo_cc_free_session(process->session);
			/* Cleanup DTMF */
			call_dtmf_cleanup(process);
			/* Cleanup echo suppressor */
			call_echo_suppressor_cleanup(process);
			free(process);
			return;
		}
		process_p = &process->next;
		process = process->next;
	}
	LOGP(DCALL, LOGL_ERROR, "Process with callref %d not found!\n", callref);
}

static process_t *get_process(int callref)
{
	process_t *process = process_head;

	while (process) {
		if (process->callref == callref)
			return process;
		process = process->next;
	}
	return NULL;
}

/* ===== Echo Suppressor Functions ===== */

/* Get echo suppressor wrapper for a callref */
echo_suppressor_wrapper_t *call_get_echo_suppressor_wrapper(int callref)
{
	process_t *process = get_process(callref);
	
	if (!process || !process->echo_suppressor_enabled || !process->echo_suppressor.suppressor_state)
		return NULL;
	
	return &process->echo_suppressor;
}

/* Initialize echo suppressor for a process */
static int call_echo_suppressor_init(process_t *process, int sample_rate, int frame_size)
{
	echo_suppressor_wrapper_t *supp;
	
	if (!process->echo_suppressor_enabled)
		return 0;
	
	supp = &process->echo_suppressor;
	
	supp->suppressor_state = echo_suppressor_init(sample_rate, frame_size, &echo_suppressor_config);
	if (!supp->suppressor_state) {
		LOGP(DECHO, LOGL_ERROR, "Failed to initialize echo suppressor\n");
		process->echo_suppressor_enabled = 0;
		return -1;
	}
	
	LOGP(DECHO, LOGL_INFO, "Echo suppressor initialized: threshold=%.1fdB atten=%.1fdB hangover=%dms delay=%dms\n",
	     echo_suppressor_config.threshold_db, echo_suppressor_config.attenuation_db,
	     echo_suppressor_config.hangover_ms, echo_suppressor_config.echo_delay_ms);
	
	return 0;
}

/* Process TX samples (far-end reference) through echo suppressor */
void call_echo_suppressor_tx(int callref, sample_t *samples, int count)
{
	echo_suppressor_wrapper_t *supp;
	int16_t tx_spl[count];
	
	supp = call_get_echo_suppressor_wrapper(callref);
	if (!supp)
		return;
	
	/* Convert to int16 */
	samples_to_int16_speech(tx_spl, samples, count);
	
	/* Process through suppressor */
	echo_suppressor_process_tx(supp->suppressor_state, tx_spl, count);
	
	/* Convert back to sample_t */
	int16_to_samples_speech(samples, tx_spl, count);
}

/* Process RX samples (near-end with echo) through echo suppressor */
void call_echo_suppressor_rx(int callref, sample_t *samples, int count)
{
	echo_suppressor_wrapper_t *supp;
	int16_t rx_spl[count];
	
	supp = call_get_echo_suppressor_wrapper(callref);
	if (!supp)
		return;
	
	/* Convert to int16 */
	samples_to_int16_speech(rx_spl, samples, count);
	
	/* Process through suppressor */
	echo_suppressor_process_rx(supp->suppressor_state, rx_spl, count);
	
	/* Convert back to sample_t */
	int16_to_samples_speech(samples, rx_spl, count);
}

/* Cleanup echo suppressor state */
static void call_echo_suppressor_cleanup(process_t *process)
{
	echo_suppressor_wrapper_t *supp;
	
	if (!process)
		return;
	
	supp = &process->echo_suppressor;
	
	if (supp->suppressor_state) {
		echo_suppressor_cleanup(supp->suppressor_state);
		supp->suppressor_state = NULL;
	}
}

/* ===== DTMF Functions ===== */

/* DTMF decoder callback - called when a digit is detected */
static void call_dtmf_rx_digit(void *priv, char digit, dtmf_meas_t __attribute__((unused)) *meas)
{
	process_t *process = priv;

	LOGP(DCALL, LOGL_INFO, "DTMF digit '%c' detected from mobile\n", digit);

	if (process && process->callref)
		call_up_dtmf(process->callref, digit);
}

/* Initialize DTMF processing for a process (called when call becomes active) */
static int call_dtmf_init(process_t *process)
{
	int rc;	if (process->dtmf_dec_enabled)
		return 0;  /* Already initialized */

	rc = dtmf_decode_init(&process->dtmf_dec, process, call_dtmf_rx_digit,
			      8000,  /* sample rate */
			      10.0,  /* max amplitude - allow for expanded audio that may exceed 1.0 */
			      0.1,   /* min amplitude (-20 dB) - higher threshold to avoid false triggers */
			      DTMF_FREQ_MARGIN_PERCENT_DEFAULT);
	if (rc < 0) {
		LOGP(DCALL, LOGL_ERROR, "Failed to init DTMF decoder!\n");
		return -1;
	}
	process->dtmf_dec_enabled = 1;

	dtmf_encode_init(&process->dtmf_enc, 8000, 1.0);  /* Amplitude */
	process->dtmf_tone_duration = 0.250;  /* 250ms tone */
	process->dtmf_gap_duration = 0.160;   /* 160ms gap */
	process->dtmf_tx_queue[0] = '\0';
	process->dtmf_tx_active = 0;

	LOGP(DCALL, LOGL_DEBUG, "DTMF initialized for call\n");

	return 0;
}

/* Cleanup DTMF processing for a process */
static void call_dtmf_cleanup(process_t *process)
{
	if (!process || !process->dtmf_dec_enabled)
		return;

	dtmf_decode_exit(&process->dtmf_dec);
	process->dtmf_dec_enabled = 0;
	process->dtmf_tx_queue[0] = '\0';
	process->dtmf_tx_active = 0;
	LOGP(DCALL, LOGL_DEBUG, "DTMF cleanup complete\n");
}

/* Process RX samples for DTMF detection (call AFTER echo suppressor) */
void call_dtmf_rx(int callref, sample_t *samples, int count)
{
	process_t *process = get_process(callref);

	if (!process || !process->dtmf_dec_enabled)
		return;

	dtmf_decode(&process->dtmf_dec, samples, count);
}

/* Process TX samples for DTMF generation (call BEFORE echo suppressor) */
void call_dtmf_tx(int callref, sample_t *samples, int count)
{
	process_t *process = get_process(callref);
	sample_t dtmf_samples[count];
	int generated;
	int i;

	if (!process || !process->dtmf_dec_enabled)
		return;

	/* Nothing to do if no digits queued and not currently active */
	if (!process->dtmf_tx_queue[0] && !process->dtmf_tx_active)
		return;

	/* Start new tone if queue has digits and not currently active */
	if (!process->dtmf_tx_active && process->dtmf_tx_queue[0]) {
		char digit = process->dtmf_tx_queue[0];
		memmove(process->dtmf_tx_queue, process->dtmf_tx_queue + 1,
			strlen(process->dtmf_tx_queue));
		dtmf_encode_set_tone(&process->dtmf_enc, digit,
				    process->dtmf_tone_duration,
				    process->dtmf_gap_duration);
		process->dtmf_tx_active = 1;
		LOGP(DCALL, LOGL_INFO, "DTMF TX: Sending '%c' to mobile\n", digit);
	}

	/* Generate DTMF samples */
	generated = dtmf_encode(&process->dtmf_enc, dtmf_samples, count);
	if (generated == 0 && process->dtmf_tx_active)
		process->dtmf_tx_active = 0;

	/* Mix DTMF with audio (additive) */
	for (i = 0; i < generated; i++)
		samples[i] += dtmf_samples[i];
}

/* Queue DTMF digits for TX to mobile */
void call_dtmf_queue(int callref, const char *digits)
{
	process_t *process = get_process(callref);
	size_t queue_len, digits_len, space;	if (!process || !digits || !digits[0])
		return;

	if (!process->dtmf_dec_enabled) {
		LOGP(DCALL, LOGL_NOTICE, "DTMF not enabled for callref %d\n", callref);
		return;
	}

	queue_len = strlen(process->dtmf_tx_queue);
	digits_len = strlen(digits);
	space = sizeof(process->dtmf_tx_queue) - 1 - queue_len;

	if (digits_len > space) {
		LOGP(DCALL, LOGL_NOTICE, "DTMF TX queue overflow, truncating '%s'\n", digits);
		digits_len = space;
	}

	if (digits_len > 0) {
		strncat(process->dtmf_tx_queue, digits, digits_len);
		LOGP(DCALL, LOGL_INFO, "Queued DTMF for TX: '%.*s' (queue now: '%s')\n",
			(int)digits_len, digits, process->dtmf_tx_queue);
	}
}

/* Print echo suppressor statistics */
static void call_echo_suppressor_print_stats(process_t *process)
{
	const echo_suppressor_state_t *stats;
	echo_suppressor_wrapper_t *supp;
	
	if (!process || !process->echo_suppressor_enabled)
		return;
	
	supp = &process->echo_suppressor;
	if (!supp->suppressor_state)
		return;
	
	stats = echo_suppressor_get_stats(supp->suppressor_state);
	if (!stats)
		return;
	
	LOGP(DECHO, LOGL_NOTICE, "SUPP: TX=%.1fdB RX=%.1fdB(gain=%.2f) delayTX=%.1fdB hang=%d delayF=%d wpos=%d | frames=%lu/%lu suppressed=%lu\n",
	     stats->tx_energy_smooth,
	     stats->rx_energy_smooth, stats->rx_gain,
	     stats->delayed_tx_energy,
	     stats->tx_hangover,
	     stats->delay_frames, stats->history_write_pos,
	     stats->tx_frames, stats->rx_frames,
	     stats->suppressed_frames);
}

static void new_state_process(int callref, enum process_state state)
{
	process_t *process = get_process(callref);

	if (!process) {
		LOGP(DCALL, LOGL_ERROR, "Process with callref %d not found!\n", callref);
		return;
	}
	LOGP(DCALL, LOGL_DEBUG, "Changing state for callref %d  %d->%d\n", callref, process->state, state);
	process->state = state;
}

static void set_tone_process(int callref, enum tones_tone tone)
{
	process_t *process = get_process(callref);

	if (!process) {
		LOGP(DCALL, LOGL_ERROR, "Process with callref %d not found!\n", callref);
		return;
	}
	tones_set_tone(&call_tones, &process->tones, tone);
}

static void disconnect_process(int callref, int cause)
{
	process_t *process = get_process(callref);

	if (!process) {
		LOGP(DCALL, LOGL_ERROR, "Process with callref %d not found!\n", callref);
		return;
	}
	tones_set_tone(&call_tones, &process->tones, cause);
	process->audio_disconnected = 1;
	process->cause = cause;
	osmo_timer_schedule(&process->timer, DISC_TIMEOUT);
}

static void process_timeout(void *data)
{
	process_t *process = data;

	{
		/* announcement timeout */
		if (process->state == PROCESS_DISCONNECT) {
			LOGP(DCALL, LOGL_INFO, "Call released toward mobile network (after timeout)\n");
			call_down_release(process->callref, process->cause);
		}
		indicate_disconnect_release(process->callref, process->cause, OSMO_CC_MSG_REL_IND);
		destroy_process(process->callref);
	}
}

static void down_audio(struct osmo_cc_session_codec *codec, uint8_t marker, uint16_t sequence_number, uint32_t timestamp, uint32_t ssrc, uint8_t *payload, int payload_len)
{
	process_t *process = codec->media->session->priv;
//	sample_t samples[len / 2];

	/* if we are disconnected or if a tone is played, ignore audio */
	if (!process || process->tones.tone != TONES_TONE_OFF)
		return;

#if 0
	int16_to_samples_speech(samples, (int16_t *)data, len / 2);
#ifdef DEBUG_LEVEL
	double lev = level_of(samples, len / 2);
	printf("festnetz-level: %s                  %.4f\n", debug_db(lev), (20 * log10(lev)));
#endif
#endif
	call_down_audio(codec->decoder, process, process->callref, sequence_number, marker, timestamp, ssrc, payload, payload_len);
}

static void indicate_setup(process_t *process, const char *callerid, const char *dialing, uint8_t network_type, const char *network_id)
{
	osmo_cc_msg_t *msg;

	msg = osmo_cc_new_msg(OSMO_CC_MSG_SETUP_IND);
	/* network type */
	if (network_type)
		osmo_cc_add_ie_calling_network(msg, network_type, network_id);
	/* calling number */
	if (callerid && callerid[0])
		osmo_cc_add_ie_calling(msg, OSMO_CC_TYPE_SUBSCRIBER, OSMO_CC_PLAN_TELEPHONY, OSMO_CC_PRESENT_ALLOWED, OSMO_CC_SCREEN_NETWORK, callerid);
	/* called number */
	if (dialing && dialing[0])
		osmo_cc_add_ie_called(msg, OSMO_CC_TYPE_UNKNOWN, OSMO_CC_PLAN_TELEPHONY, dialing);
	/* bearer capability */
	osmo_cc_add_ie_bearer(msg, OSMO_CC_CODING_ITU_T, OSMO_CC_CAPABILITY_AUDIO, OSMO_CC_MODE_CIRCUIT);
	/* sdp offer */
	process->session = osmo_cc_helper_audio_offer(&ep->session_config, process, codecs + no_l16, down_audio, msg, 1);

	LOGP(DCALL, LOGL_INFO, "Indicate OSMO-CC setup towards fixed network\n");
	osmo_cc_ll_msg(ep, process->callref, msg);
}

static void indicate_proceeding(int callref, const char *sdp)
{
	osmo_cc_msg_t *msg;

	msg = osmo_cc_new_msg(OSMO_CC_MSG_PROC_IND);

	/* sdp */
	osmo_cc_add_ie_sdp(msg, sdp);

	/* progress information */
	osmo_cc_add_ie_progress(msg, OSMO_CC_CODING_ITU_T, OSMO_CC_LOCATION_BEYOND_INTERWORKING, OSMO_CC_PROGRESS_INBAND_INFO_AVAILABLE);

	LOGP(DCALL, LOGL_INFO, "Indicate OSMO-CC call confirm towards fixed network\n");
	osmo_cc_ll_msg(ep, callref, msg);
}

static void indicate_alerting(int callref)
{
	osmo_cc_msg_t *msg;

	msg = osmo_cc_new_msg(OSMO_CC_MSG_ALERT_IND);

	LOGP(DCALL, LOGL_INFO, "Indicate OSMO-CC alerting towards fixed network\n");
	osmo_cc_ll_msg(ep, callref, msg);
}

static void indicate_answer(int callref, const char *sdp, const char *connectid)
{
	osmo_cc_msg_t *msg;

	msg = osmo_cc_new_msg(OSMO_CC_MSG_SETUP_CNF);
	/* calling number */
	if (connectid && connectid[0])
		osmo_cc_add_ie_calling(msg, OSMO_CC_TYPE_SUBSCRIBER, OSMO_CC_PLAN_TELEPHONY, OSMO_CC_PRESENT_ALLOWED, OSMO_CC_SCREEN_NETWORK, connectid);

	/* sdp */
	if (sdp)
		osmo_cc_add_ie_sdp(msg, sdp);

	LOGP(DCALL, LOGL_INFO, "Indicate OSMO-CC answer towards fixed network\n");
	osmo_cc_ll_msg(ep, callref, msg);
}

static void indicate_answer_ack(int callref)
{
	osmo_cc_msg_t *msg;

	msg = osmo_cc_new_msg(OSMO_CC_MSG_SETUP_COMP_IND);

	LOGP(DCALL, LOGL_INFO, "Indicate OSMO-CC setup complete towards fixed network\n");
	osmo_cc_ll_msg(ep, callref, msg);
}

static void indicate_disconnect_release(int callref, int cause, uint8_t msg_type)
{
	osmo_cc_msg_t *msg;

	msg = osmo_cc_new_msg(msg_type);

	/* cause */
	osmo_cc_add_ie_cause(msg, OSMO_CC_LOCATION_PRIV_SERV_LOC_USER, cause, 0, 0);

	/* progress information */
	if (msg_type == OSMO_CC_MSG_DISC_IND)
		osmo_cc_add_ie_progress(msg, OSMO_CC_CODING_ITU_T, OSMO_CC_LOCATION_BEYOND_INTERWORKING, OSMO_CC_PROGRESS_INBAND_INFO_AVAILABLE);

	LOGP(DCALL, LOGL_INFO, "%s OSMO-CC %s towards fixed network\n", (msg_type == OSMO_CC_MSG_REL_CNF) ? "Confirm" : "Indicated", (msg_type == OSMO_CC_MSG_DISC_IND) ? "disconnect" : "release");
	osmo_cc_ll_msg(ep, callref, msg);
}

/* Setup is received from transceiver. */
int call_up_setup(const char *callerid, const char *dialing, uint8_t network, const char *network_id)
{
	osmo_cc_call_t *call;
	process_t *process;

	LOGP(DCALL, LOGL_INFO, "Incoming call from '%s' to '%s'\n", callerid ? : "unknown", dialing);
	if (!strcmp(dialing, "010"))
		LOGP(DCALL, LOGL_INFO, " -> Call to Operator '%s'\n", dialing);

	call = osmo_cc_call_new(ep);

	process = create_process(call->callref, PROCESS_SETUP_RO);

	indicate_setup(process, callerid, dialing, network, network_id);

	return call->callref;
}

/* Transceiver indicates alerting. */
void call_up_alerting(int callref)
{
	if (!callref) {
		LOGP(DCALL, LOGL_DEBUG, "Ignoring alerting, because callref not set. (not for us)\n");
		return;
	}

	LOGP(DCALL, LOGL_INFO, "Call is alerting\n");

	if (!connect_on_setup)
		indicate_alerting(callref);
	set_tone_process(callref, TONES_TONE_RINGBACK);
	new_state_process(callref, PROCESS_ALERTING_RT);
}

/* Transceiver indicates early audio */
void call_up_early(int callref)
{
	set_tone_process(callref, TONES_TONE_OFF);
}

/* Transceiver indicates answer. */
void call_up_answer(int callref, const char *connect_id)
{
	process_t *process;
	
	if (!callref) {
		LOGP(DCALL, LOGL_DEBUG, "Ignoring answer, because callref not set. (not for us)\n");
		return;
	}

	LOGP(DCALL, LOGL_INFO, "*** Call has been answered by '%s' (callref=%d) ***\n", connect_id, callref);

	if (!connect_on_setup)
		indicate_answer(callref, NULL, connect_id);
	set_tone_process(callref, TONES_TONE_OFF);
	new_state_process(callref, PROCESS_CONNECT);
	
	/* Initialize echo suppressor */
	process = get_process(callref);
	if (process && echo_suppressor_config.enabled) {
		process->echo_suppressor_enabled = 1;
		call_echo_suppressor_init(process, 8000, 128);
	}
	/* Initialize DTMF */
	if (process)
		call_dtmf_init(process);
}

/*
 * Flash (Hook-Flash) Signal Handling - TX Direction (Mobile -> Network)
 *
 * This function is called when the LOCAL MOBILE USER presses the flash button.
 * We forward this event to the REMOTE PARTY (SIP side) via osmo-cc.
 *
 * In 1G AMPS cellular (TIA/EIA-553-A), flash is signaled by the mobile station
 * sending a 10 kHz Signaling Tone (ST) burst for 400ms on the voice channel.
 * The base station detects this as SAT/ST status change: (1,0) -> (1,1) for
 * 400ms -> (1,0).
 *
 * Historical use cases for flash in mobile telephony:
 *
 * 1. Three-Way Calling:
 *    - Mobile user talking to Party A (SIP)
 *    - Mobile user presses FLASH -> Party A on hold, mobile user hears dial tone
 *    - Mobile user dials Party B's number
 *    - Mobile user presses FLASH again -> All three parties conferenced
 *
 * 2. Call Waiting:
 *    - Mobile user talking to Party A, call waiting tone indicates Party B calling
 *    - Mobile user presses FLASH -> Party A on hold, connected to Party B
 *    - Mobile user presses FLASH -> Toggle back to Party A
 *
 * 3. Call Transfer:
 *    - Mobile user talking to Party A
 *    - Mobile user presses FLASH -> Dials transfer destination
 *    - Mobile user hangs up -> Party A transferred to destination
 *
 * 4. Feature Access:
 *    - After flash, mobile user dials feature codes for call forwarding, etc.
 *
 * Per TIA/EIA-553-A Section 3.6.4.4, after receiving flash the base station
 * may send Order 8 (Send Called-Address) to request digits dialed by user.
 * The mobile must respond within 10 seconds of the flash.
 *
 * osmo-cc Mapping:
 * We toggle between OSMO_CC_NOTIFY_USER_SUSPENDED and OSMO_CC_NOTIFY_USER_RESUMED
 * on each flash press. First flash puts remote on hold (SUSPENDED), second flash
 * brings them back (RESUMED), and so on.
 *
 * Note: The SIP endpoint may translate this to SIP re-INVITE with sendonly/recvonly,
 * SIP INFO with hookflash, or other hold signaling depending on configuration.
 */
void call_up_flash(int callref)
{
	process_t *process;
	osmo_cc_msg_t *msg;
	uint8_t notify_type;

	if (!callref) {
		LOGP(DCALL, LOGL_DEBUG, "Ignoring flash, because callref not set. (not for us)\n");
		return;
	}

	process = get_process(callref);
	if (!process) {
		LOGP(DCALL, LOGL_ERROR, "Flash received but no process for callref %d\n", callref);
		return;
	}

	LOGP(DCALL, LOGL_INFO, "*** Flash (Hook-Flash) received from mobile (callref=%d) ***\n", callref);

	/* Try RFC 4733 first if available (event 16 = Flash) */
	if (process->media && process->media->telephone_event_negotiated) {
		LOGP(DCALL, LOGL_INFO, "Flash from mobile -> RFC 4733 event 16 (callref=%d)\n", callref);
		osmo_cc_rtp_send_event(process->media, 16, 400);  /* 400ms duration per TIA/EIA-553-A */
		/* Still toggle hold state for local tracking */
		process->remote_on_hold = !process->remote_on_hold;
		return;
	}

	/* Fall back to osmo-cc NOTIFY message */
	LOGP(DCALL, LOGL_INFO, "Flash from mobile -> SIP INFO fallback (callref=%d)\n", callref);
	/* Toggle hold state: first flash puts remote on hold, second flash retrieves */
	if (!process->remote_on_hold) {
		/* Remote party is active -> put them on hold */
		notify_type = OSMO_CC_NOTIFY_USER_SUSPENDED;
		process->remote_on_hold = 1;
		LOGP(DCALL, LOGL_DEBUG, "Putting remote party on hold (SUSPENDED)\n");
	} else {
		/* Remote party is on hold -> retrieve them */
		notify_type = OSMO_CC_NOTIFY_USER_RESUMED;
		process->remote_on_hold = 0;
		LOGP(DCALL, LOGL_DEBUG, "Retrieving remote party from hold (RESUMED)\n");
	}

	/* Use NOTIFY_REQ to send towards SIP endpoint (REQ = request to lower layer) */
	msg = osmo_cc_new_msg(OSMO_CC_MSG_NOTIFY_REQ);
	osmo_cc_add_ie_notify(msg, notify_type);
	osmo_cc_ll_msg(ep, callref, msg);
}

/* Convert DTMF character to RFC 4733 event code.
 * Returns event code 0-15 for DTMF, -1 for invalid character.
 * Per RFC 4733 Section 3.2:
 *   0-9: DTMF digits 0-9
 *   10:  DTMF *
 *   11:  DTMF #
 *   12-15: DTMF A-D
 */
static int dtmf_char_to_rfc4733_event(char digit)
{
	if (digit >= '0' && digit <= '9')
		return digit - '0';
	if (digit == '*')
		return 10;
	if (digit == '#')
		return 11;
	if (digit >= 'A' && digit <= 'D')
		return 12 + (digit - 'A');
	if (digit >= 'a' && digit <= 'd')
		return 12 + (digit - 'a');
	return -1;
}

/* Transceiver indicates DTMF digit detected from mobile. */
void call_up_dtmf(int callref, char digit)
{
	process_t *process;
	osmo_cc_msg_t *msg;
	char dtmf_str[2] = { digit, '\0' };
	int event;

	if (!callref) {
		LOGP(DCALL, LOGL_DEBUG, "Ignoring DTMF, no callref\n");
		return;
	}

	process = get_process(callref);

	LOGP(DCALL, LOGL_NOTICE, "call_up_dtmf: digit='%c' callref=%d process=%p media=%p\n",
	     digit, callref, process, process ? process->media : NULL);
	if (process && process->media) {
		LOGP(DCALL, LOGL_NOTICE, "call_up_dtmf: media->telephone_event_negotiated=%d pt=%d\n",
		     process->media->telephone_event_negotiated, process->media->telephone_event_pt);
	}

	/* Try RFC 4733 first if available */
	if (process && process->media && process->media->telephone_event_negotiated) {
		event = dtmf_char_to_rfc4733_event(digit);
		if (event >= 0) {
			LOGP(DCALL, LOGL_INFO, "DTMF '%c' from mobile -> RFC 4733 (callref=%d)\n", digit, callref);
			osmo_cc_rtp_send_event(process->media, event, 160);  /* 160ms duration */
			return;
		}
	}

	/* Fall back to osmo-cc INFO message */
	LOGP(DCALL, LOGL_INFO, "DTMF '%c' from mobile -> SIP INFO fallback (callref=%d)\n", digit, callref);

	msg = osmo_cc_new_msg(OSMO_CC_MSG_INFO_IND);
	/* Use IE_DTMF (not IE_KEYPAD) - osmo-cc-sip-endpoint expects IE_DTMF for SIP INFO */
	osmo_cc_add_ie_dtmf(msg, 160, 160, OSMO_CC_DTMF_MODE_DIGITS, dtmf_str);
	osmo_cc_ll_msg(ep, callref, msg);
}

/* Transceiver indicates release. */
void call_up_release(int callref, int cause)
{
	process_t *process;

	if (!callref) {
		LOGP(DCALL, LOGL_DEBUG, "Ignoring release, because callref not set. (not for us)\n");
		return;
	}

	LOGP(DCALL, LOGL_INFO, "*** Call has been released with cause=%d (callref=%d) ***\n", cause, callref);

	process = get_process(callref);
	if (process) {
		LOGP(DCALL, LOGL_DEBUG, "Process state: %d, audio_disconnected: %d\n", process->state, process->audio_disconnected);
		/* just keep OSMO-CC connection if tones shall be sent.
		 * no tones while setting up / alerting the call.
		 * For active calls (PROCESS_CONNECT), mobile already hung up,
		 * so we can't play tones to it - release immediately. */
		if (connect_on_setup
		 && process->state != PROCESS_SETUP_RO
		 && process->state != PROCESS_ALERTING_RO
		 && process->state != PROCESS_CONNECT)
			disconnect_process(callref, cause);
		else
		/* if no tones shall be sent, release on disconnect
		 * or RO setup states, or active call (mobile side released) */
		if (process->state == PROCESS_DISCONNECT
		 || process->state == PROCESS_SETUP_RO
		 || process->state == PROCESS_ALERTING_RO
		 || process->state == PROCESS_CONNECT) {
			LOGP(DCALL, LOGL_DEBUG, "Destroying process and sending REL_IND to network\n");
			destroy_process(callref);
			indicate_disconnect_release(callref, cause, OSMO_CC_MSG_REL_IND);
		/* if no tones shall be sent, disconnect on all other states */
		} else {
			LOGP(DCALL, LOGL_DEBUG, "Disconnecting process and sending DISC_IND to network\n");
			disconnect_process(callref, cause);
			indicate_disconnect_release(callref, cause, OSMO_CC_MSG_DISC_IND);
		}
	} else {
		LOGP(DCALL, LOGL_DEBUG, "No process found for callref, sending REL_IND anyway\n");
		/* we don't know about the process, just send release to upper layer anyway */
		indicate_disconnect_release(callref, cause, OSMO_CC_MSG_REL_IND);
	}
}

/* turn recall tone on or off */
void call_tone_recall(int callref, int on)
{
	set_tone_process(callref, (on) ? TONES_TONE_RECALL : TONES_TONE_OFF);
}

/* forward audio to OSMO-CC or call instance */
void call_up_audio(int callref, sample_t *samples, int len)
{
	process_t *process;
	int16_t spl[len];
	uint8_t *payload;
	int payload_len;

	if (len != 160) {
		fprintf(stderr, "Samples must be 160, please fix!\n");
		abort();
	}
	if (!callref)
		return;

	/* if we are disconnected or if a tone is played, ignore audio */
	process = get_process(callref);
	if (!process || process->tones.tone != TONES_TONE_OFF || process->audio_disconnected)
		return;

	/* no codec negotiated (yet) */
	if (!process->codec)
		return;

	/* forward audio */
#ifdef DEBUG_LEVEL
	double lev = level_of(samples, len);
	printf("   mobil-level: %s%.4f\n", debug_db(lev), (20 * log10(lev)));
#endif
	/* real to integer */
	samples_to_int16_speech(spl, samples, len);
	
	/* DEBUG: RTP TX level diagnostics (disabled - uncomment to enable)
	 * Logs: RTP TX: avg/max int16 sample values and dB levels being sent to RTP
	 * Measures actual audio levels going out to SIP/RTP every second */
#if 0
	{
		static int rtp_dbg_count = 0;
		static int64_t rtp_dbg_sum = 0;
		static int16_t rtp_dbg_max = 0;
		int i;
		for (i = 0; i < len; i++) {
			int16_t abs_val = (spl[i] < 0) ? -spl[i] : spl[i];
			rtp_dbg_sum += abs_val;
			if (abs_val > rtp_dbg_max) rtp_dbg_max = abs_val;
		}
		rtp_dbg_count += len;
		if (rtp_dbg_count >= 8000) {
			double avg = (double)rtp_dbg_sum / rtp_dbg_count;
			double avg_db = (avg > 0) ? 20.0 * log10(avg / 32767.0) : -100.0;
			double max_db = (rtp_dbg_max > 0) ? 20.0 * log10((double)rtp_dbg_max / 32767.0) : -100.0;
			LOGP(DCALL, LOGL_DEBUG, "RTP TX: avg=%d (%.1fdB) max=%d (%.1fdB)\n",
			     (int)avg, avg_db, rtp_dbg_max, max_db);
			rtp_dbg_count = 0;
			rtp_dbg_sum = 0;
			rtp_dbg_max = 0;
		}
	}
#endif
	
	/* encode and send via RTP */
	process->codec->encoder((uint8_t *)spl, len * 2, &payload, &payload_len, process);
	osmo_cc_rtp_send(process->codec, payload, payload_len, 0, 1, len);
	free(payload);
	/* don't destroy process here in case of an error */
}

/* clock that is used to transmit patterns */
void call_clock(void)
{
	process_t *process = process_head;
	static int stats_counter = 0;

	call_down_clock();

	/* Print echo suppressor stats every 1 second (assuming call_clock is called every 100ms) */
	if (echo_suppressor_config.stats_enabled && ++stats_counter >= 10) {  /* 10 * 100ms = 1s */
		process_t *p = process_head;
		while (p) {
			if (p->echo_suppressor_enabled && p->state == PROCESS_CONNECT)
				call_echo_suppressor_print_stats(p);
			p = p->next;
		}
		stats_counter = 0;
	}

	while(process) {
		if (process->tones.tone != TONES_TONE_OFF) {
			int16_t spl[160];
			uint8_t *payload;
			int payload_len;
			/* Initialize buffer to silence in case tones_read_tone doesn't fill it */
			memset(spl, 0, sizeof(spl));
			/* try to get patterns, else copy the samples we got */
			tones_read_tone(&process->tones, spl, 160);
#ifdef DEBUG_LEVEL
			sample_t samples[160];
			int16_to_samples(samples, (int16_t *)spl->data, 160);
			double lev = level_of(samples, 160);
			printf("   mobil-level: %s%.4f\n", debug_db(lev), (20 * log10(lev)));
			samples_to_int16(spl, samples, 160);
#endif
			/* encode and send via RTP */
			process->codec->encoder((uint8_t *)spl, 160 * 2, &payload, &payload_len, process);
			osmo_cc_rtp_send(process->codec, payload, payload_len, 0, 1, 160);
			free(payload);
			/* don't destroy process here in case of an error */
		}
		process = process->next;
	}
}

/* messages received from fixed network */
static void ll_msg_cb(osmo_cc_endpoint_t __attribute__((unused)) *ep, uint32_t callref, osmo_cc_msg_t *msg)
{
	process_t *process;
	uint8_t coding, location, progress, isdn_cause, socket_cause;
	uint16_t sip_cause, metering_connect_units;
	uint8_t type, plan, present, screen, caller_type;
	char caller_id[33], number[33];
	struct timeval tv_meter = {};
	const char *suffix, *invalid;
	int rc;

	process = get_process(callref);
	if (!process) {
		if (msg->type == OSMO_CC_MSG_SETUP_REQ)
			process = create_process(callref, PROCESS_SETUP_RT);
		else {
			/* release collisions is not forbidden */
			if (msg->type != OSMO_CC_MSG_REL_REQ)
				LOGP(DCALL, LOGL_ERROR, "No process!\n");
			osmo_cc_free_msg(msg);
			return;
		}
	}

	if (process->audio_disconnected) {
		switch(msg->type) {
		case OSMO_CC_MSG_DISC_REQ:
			rc = osmo_cc_get_ie_cause(msg, 0, &location, &isdn_cause, &sip_cause, &socket_cause);
			if (rc < 0)
				isdn_cause = OSMO_CC_ISDN_CAUSE_NORM_CALL_CLEAR;
			LOGP(DCALL, LOGL_INFO, "Received OSMO-CC disconnect from fixed network with cause %d\n", isdn_cause);
			LOGP(DCALL, LOGL_INFO, "Call disconnected, releasing!\n");
			destroy_process(callref);
			indicate_disconnect_release(callref, isdn_cause, OSMO_CC_MSG_REL_IND);
		break;
		case OSMO_CC_MSG_REL_REQ:
			rc = osmo_cc_get_ie_cause(msg, 0, &location, &isdn_cause, &sip_cause, &socket_cause);
			if (rc < 0)
				isdn_cause = OSMO_CC_ISDN_CAUSE_NORM_CALL_CLEAR;
			LOGP(DCALL, LOGL_INFO, "Received OSMO-CC release from fixed network with cause %d\n", isdn_cause);
			LOGP(DCALL, LOGL_INFO, "Call released\n");
			destroy_process(callref);
			indicate_disconnect_release(callref, isdn_cause, OSMO_CC_MSG_REL_CNF);
			break;
		}
		osmo_cc_free_msg(msg);
		return;
	}

	/* get metering information, tv_meter elements are 0, if no metering info available */
	osmo_cc_get_ie_metering(msg, 0, &metering_connect_units, &tv_meter);

	switch(msg->type) {
	case OSMO_CC_MSG_SETUP_REQ:
	    {
		const char *sdp;

		/* sdp accept */
		sdp = osmo_cc_helper_audio_accept(&ep->session_config, process, codecs + no_l16, down_audio, msg, &process->session, &process->codec, 0);
		if (!sdp) {
			disconnect_process(callref, 47);
			indicate_disconnect_release(callref, 47, OSMO_CC_MSG_REJ_IND);
			break;
		}

		/* Set up RFC 4733 telephone-event support */
		call_setup_rfc4733(process);

		/* caller id */
		rc = osmo_cc_get_ie_calling(msg, 0, &type, &plan, &present, &screen, caller_id, sizeof(caller_id));
		if (rc < 0) {
			caller_type = TYPE_NOTAVAIL;
			caller_id[0] = '\0';
		} else {
			switch (type) {
			case OSMO_CC_TYPE_INTERNATIONAL:
				caller_type = TYPE_INTERNATIONAL;
				break;
			case OSMO_CC_TYPE_NATIONAL:
				caller_type = TYPE_NATIONAL;
				break;
			case OSMO_CC_TYPE_SUBSCRIBER:
				caller_type = TYPE_SUBSCRIBER;
				break;
			default:
				caller_type = TYPE_UNKNOWN;
			}
			if (present == OSMO_CC_PRESENT_RESTRICTED)
				caller_type = TYPE_ANONYMOUS;
		}

		/* dialing */
		rc = osmo_cc_get_ie_called(msg, 0, &type, &plan, number, sizeof(number));
		if (rc < 0) {
			number[0] = '\0';
			type = OSMO_CC_TYPE_UNKNOWN;
			plan = OSMO_CC_PLAN_TELEPHONY;
		}
		LOGP(DCALL, LOGL_INFO, "Received OSMO-CC call from fixed network '%s' to mobile '%s'\n", caller_id, number);
		if (!connect_on_setup)
			indicate_proceeding(callref, sdp);
		else {
			LOGP(DCALL, LOGL_DEBUG, "Early connecting after setup\n");
			indicate_answer(callref, sdp, number);
		}
		LOGP(DCALL, LOGL_INFO, "Outgoing call from '%s' to '%s'\n", caller_id, number);

		/* insert '+' for international dialing */
		if (type == OSMO_CC_TYPE_INTERNATIONAL && number[0] != '+') {
			memmove(number + 1, number, sizeof(number) - 2);
			number[0] = '+';
		}

		/* remove prefix, if any */
		suffix = mobile_number_remove_prefix(number);

		/* check suffix length */
		invalid = mobile_number_check_length(suffix);
		if (invalid) {
			LOGP(DCALL, LOGL_NOTICE, "Mobile number '%s' has invalid length: %s\n", suffix, invalid);
			disconnect_process(callref, OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT);
			if (!connect_on_setup) {
				LOGP(DCALL, LOGL_INFO, "Disconnecting OSMO-CC call towards fixed network (cause=%d)\n", OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT);
				indicate_disconnect_release(callref, OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT, OSMO_CC_MSG_DISC_IND);
			}
			break;
		}

		/* check suffix digits */
		invalid = mobile_number_check_digits(suffix);
		if (invalid) {
			LOGP(DCALL, LOGL_NOTICE, "Mobile number '%s' has invalid digit: %s.\n", suffix, invalid);
			disconnect_process(callref, OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT);
			if (!connect_on_setup) {
				LOGP(DCALL, LOGL_INFO, "Disconnecting OSMO-CC call towards fixed network (cause=%d)\n", OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT);
				indicate_disconnect_release(callref, OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT, OSMO_CC_MSG_DISC_IND);
			}
			break;
		}

		/* check if suffix is valid */
		if (mobile_number_check_valid) {
			invalid = mobile_number_check_valid(suffix);
			if (invalid) {
				LOGP(DCALL, LOGL_NOTICE, "Mobile number '%s' is invalid for this network: %s\n", suffix, invalid);
				disconnect_process(callref, OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT);
				if (!connect_on_setup) {
					LOGP(DCALL, LOGL_INFO, "Disconnecting OSMO-CC call towards fixed network (cause=%d)\n", OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT);
					indicate_disconnect_release(callref, OSMO_CC_ISDN_CAUSE_INV_NR_FORMAT, OSMO_CC_MSG_DISC_IND);
				}
				break;
			}
		}

		/* setup call */
		rc = call_down_setup(callref, caller_id, caller_type, suffix);
		if (rc < 0) {
			LOGP(DCALL, LOGL_NOTICE, "Call rejected, cause %d\n", -rc);
			if (!connect_on_setup) {
				LOGP(DCALL, LOGL_INFO, "Disconnecting OSMO-CC call towards fixed network (cause=%d)\n", -rc);
				indicate_disconnect_release(callref, -rc, OSMO_CC_MSG_DISC_IND);
			}
			disconnect_process(callref, -rc);
			break;
		}
		break;
	    }
	case OSMO_CC_MSG_SETUP_ACK_REQ:
		LOGP(DCALL, LOGL_INFO, "Received OSMO-CC overlap from fixed network\n");
		rc = osmo_cc_helper_audio_negotiate(msg, &process->session, &process->codec);
		if (rc < 0) {
			nego_failed:
			LOGP(DCALL, LOGL_INFO, "Releasing, because codec negotiation failed.\n");
			destroy_process(callref);
			indicate_disconnect_release(callref, 47, OSMO_CC_MSG_REL_IND);
			LOGP(DCALL, LOGL_INFO, "Call released toward mobile network\n");
			call_down_release(callref, 47);
			break;
		}
		break;
	case OSMO_CC_MSG_PROC_REQ:
		LOGP(DCALL, LOGL_INFO, "Received OSMO-CC proceeding from fixed network\n");
		rc = osmo_cc_helper_audio_negotiate(msg, &process->session, &process->codec);
		if (rc < 0)
			goto nego_failed;
		/* Notify AMPS layer - can now assign voice channel */
		call_down_proceeding(callref);
		break;
	case OSMO_CC_MSG_PROGRESS_REQ:
		LOGP(DCALL, LOGL_INFO, "Received OSMO-CC progress from fixed network\n");
		rc = osmo_cc_helper_audio_negotiate(msg, &process->session, &process->codec);
		if (rc < 0)
			goto nego_failed;
		break;
	case OSMO_CC_MSG_ALERT_REQ:
		LOGP(DCALL, LOGL_INFO, "Received OSMO-CC alerting from fixed network\n");
		rc = osmo_cc_helper_audio_negotiate(msg, &process->session, &process->codec);
		if (rc < 0)
			goto nego_failed;
		/* Also trigger channel assignment on ALERTING (in case PROCEEDING was skipped) */
		call_down_proceeding(callref);
		new_state_process(callref, PROCESS_ALERTING_RO);
		break;
	case OSMO_CC_MSG_SETUP_RSP:
		LOGP(DCALL, LOGL_INFO, "Received OSMO-CC answer from fixed network\n");
		rc = osmo_cc_helper_audio_negotiate(msg, &process->session, &process->codec);
		if (rc < 0)
			goto nego_failed;
		/* Set up RFC 4733 telephone-event support */
		call_setup_rfc4733(process);
		/* Trigger channel assignment if PROCEEDING was skipped (e.g., console auto-answer) */
		call_down_proceeding(callref);
		new_state_process(callref, PROCESS_CONNECT);
		LOGP(DCALL, LOGL_INFO, "Call answered\n");
		call_down_answer(callref, &tv_meter);
		indicate_answer_ack(callref);
		
		/* Initialize echo suppressor */
		if (echo_suppressor_config.enabled) {
			process->echo_suppressor_enabled = 1;
			call_echo_suppressor_init(process, 8000, 128);
		}
		/* Initialize DTMF */
		call_dtmf_init(process);
		break;
	case OSMO_CC_MSG_DISC_REQ:
		rc = osmo_cc_helper_audio_negotiate(msg, &process->session, &process->codec);
		if (rc < 0)
			goto nego_failed;
		rc = osmo_cc_get_ie_cause(msg, 0, &location, &isdn_cause, &sip_cause, &socket_cause);
		if (rc < 0)
			isdn_cause = OSMO_CC_ISDN_CAUSE_NORM_CALL_CLEAR;
		rc = osmo_cc_get_ie_progress(msg, 0, &coding, &location, &progress);
		if (rc < 0)
			progress = 0;
		LOGP(DCALL, LOGL_INFO, "Received OSMO-CC disconnect from fixed network with cause %d\n", isdn_cause);
		if (release_on_disconnect || (progress != 1 && progress != 8)) {
			LOGP(DCALL, LOGL_INFO, "Releasing, because we don't send disconnect tones to mobile phone\n");
			destroy_process(callref);
			indicate_disconnect_release(callref, isdn_cause, OSMO_CC_MSG_REL_IND);
			LOGP(DCALL, LOGL_INFO, "Call released toward mobile network\n");
			call_down_release(callref, isdn_cause);
			break;
		}
		new_state_process(callref, PROCESS_DISCONNECT);
		LOGP(DCALL, LOGL_INFO, "Call disconnected\n");
		call_down_disconnect(callref, isdn_cause);
		/* we might get released during disconnect handling!!! */
		process = get_process(callref);
		if (process && process->state == PROCESS_DISCONNECT)
			osmo_timer_schedule(&process->timer, DISC_TIMEOUT);
		break;
	case OSMO_CC_MSG_REJ_REQ:
	case OSMO_CC_MSG_REL_REQ:
		rc = osmo_cc_get_ie_cause(msg, 0, &location, &isdn_cause, &sip_cause, &socket_cause);
		if (rc < 0)
			isdn_cause = OSMO_CC_ISDN_CAUSE_NORM_CALL_CLEAR;
		destroy_process(callref);
		if (msg->type == OSMO_CC_MSG_REL_REQ) {
			LOGP(DCALL, LOGL_INFO, "Received OSMO-CC release from fixed network with cause %d\n", isdn_cause);
			indicate_disconnect_release(callref, isdn_cause, OSMO_CC_MSG_REL_CNF);
		} else
			LOGP(DCALL, LOGL_INFO, "Received OSMO-CC reject from fixed network with cause %d\n", isdn_cause);
		LOGP(DCALL, LOGL_INFO, "Call released toward mobile network\n");
		call_down_release(callref, isdn_cause);
		break;
	case OSMO_CC_MSG_INFO_REQ:
	    {
		char digits[33];
		uint8_t duration_ms, pause_ms, dtmf_mode;
		
		LOGP(DCALL, LOGL_NOTICE, "CALL-INFO-REQ: received from SIP side (callref=%d)\n", callref);
		
		/* Try IE_DTMF first (from SIP INFO or RFC 4733) */
		rc = osmo_cc_get_ie_dtmf(msg, 0, &duration_ms, &pause_ms, &dtmf_mode, digits, sizeof(digits));
		if (rc >= 0 && digits[0]) {
			LOGP(DCALL, LOGL_NOTICE, "CALL-INFO-REQ: DTMF '%s' from SIP -> queue for TX to mobile (callref=%d)\n", digits, callref);
			LOGP(DCALL, LOGL_INFO, "INFO_REQ: DTMF '%s' received via IE_DTMF (callref=%d)\n", digits, callref);
			call_dtmf_queue(callref, digits);
			break;
		}
		
		/* Fall back to IE_KEYPAD */
		rc = osmo_cc_get_ie_keypad(msg, 0, digits, sizeof(digits));
		if (rc >= 0 && digits[0]) {
			LOGP(DCALL, LOGL_NOTICE, "CALL-INFO-REQ: KEYPAD '%s' from SIP -> queue for TX to mobile (callref=%d)\n", digits, callref);
			LOGP(DCALL, LOGL_INFO, "INFO_REQ: DTMF '%s' received via IE_KEYPAD (callref=%d)\n", digits, callref);
			call_dtmf_queue(callref, digits);
		} else {
			LOGP(DCALL, LOGL_NOTICE, "CALL-INFO-REQ: no DTMF/KEYPAD IE found (callref=%d)\n", callref);
		}
		break;
	    }
	/*
	 * Handle NOTIFY messages from the network (e.g., remote party flash/hold).
	 *
	 * When the remote SIP endpoint sends a NOTIFY (e.g., via SIP NOTIFY or
	 * interpreted from SIP INFO with hookflash), we receive it here.
	 *
	 * Common scenarios:
	 * - Remote party pressed flash for three-way calling or call waiting
	 * - Remote party put us on hold (USER_SUSPENDED / REMOTE_HOLD)
	 * - Remote party retrieved us from hold (USER_RESUMED / REMOTE_RETRIEVAL)
	 *
	 * For 1G analog networks, we play a recall tone to indicate hold status
	 * to the mobile user, similar to how landline phones indicate hold.
	 *
	 * Note: We handle both NOTIFY_IND (from SIP endpoint) and NOTIFY_REQ
	 * for compatibility with different osmo-cc routing configurations.
	 */
	case OSMO_CC_MSG_NOTIFY_IND:  /* Flash/hold from SIP INFO (SIP endpoint -> call.c) */
	case OSMO_CC_MSG_NOTIFY_REQ:  /* Alternative routing path */
	    {
		uint8_t notify;
		
		rc = osmo_cc_get_ie_notify(msg, 0, &notify);
		if (rc >= 0) {
			LOGP(DCALL, LOGL_INFO, "Received NOTIFY from osmo-cc: notify=0x%02x (callref=%d)\n", notify, callref);
			/* Handle remote hold/retrieve notifications */
			switch (notify) {
			case OSMO_CC_NOTIFY_USER_SUSPENDED:
			case OSMO_CC_NOTIFY_REMOTE_HOLD:
				LOGP(DCALL, LOGL_INFO, "Remote party put call on hold\n");
				/* Play recall tone to indicate remote hold to mobile user */
				call_tone_recall(callref, 1);
				break;
			case OSMO_CC_NOTIFY_USER_RESUMED:
			case OSMO_CC_NOTIFY_REMOTE_RETRIEVAL:
				LOGP(DCALL, LOGL_INFO, "Remote party retrieved call from hold\n");
				/* Stop recall tone - call is active again */
				call_tone_recall(callref, 0);
				break;
			default:
				LOGP(DCALL, LOGL_DEBUG, "Unhandled notify type: 0x%02x\n", notify);
				break;
			}
		}
		break;
	    }
	}
	osmo_cc_free_msg(msg);
}

int call_init(const char *name, int _send_patterns, int _release_on_disconnect, int use_socket, int argc, const char *argv[], int _no_l16, const char *toneset)
{
	int rc;

	connect_on_setup = _send_patterns;
	release_on_disconnect = _release_on_disconnect;

	g711_init();
	rc = tones_init(&call_tones, toneset, TONES_TDATA_SLIN16HOST);
	if (rc < 0) {
		LOGP(DCALL, LOGL_ERROR, "Failed to initialize tone set '%s'. Please fix!\n", toneset);
		return -EINVAL;
	}

	no_l16 = !!_no_l16;
	ep = &endpoint;
	rc = osmo_cc_new(ep, OSMO_CC_VERSION, name, OSMO_CC_LOCATION_PRIV_SERV_LOC_USER, ll_msg_cb, (use_socket) ? NULL : console_msg, NULL, argc, argv);
	if (rc > 0)
		return -EINVAL;
	if (rc < 0)
		return rc;

	return 0;
}

void call_exit(void)
{
	tones_exit(&call_tones);
	if (ep) {
		osmo_cc_delete(ep);
		ep = NULL;
	}
}

int call_handle(void)
{
	return osmo_cc_handle();
}

