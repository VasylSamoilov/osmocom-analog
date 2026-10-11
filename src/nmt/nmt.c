/* NMT protocol handling
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

#define CHAN nmt->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/cause.h"
#include "../libmobile/get_time.h"
#include "../libmobile/console.h"
#include <osmocom/cc/message.h>
#include "nmt.h"
#include "transaction.h"
#include "dsp.h"
#include "frame.h"
#include "countries.h"

/* How does paging on all channels work:
 *
 * Paging is performed on all free CC channels. The transaction pointer
 * is set towards transaction. (nmt->trans points to trans)
 *
 * After timeout, all channels with that transaction pointer are released.
 *
 * When paging was replied, other channels with the transaction pointer are
 * released, but the channel with the reply is linked to transaction in both
 * directions. (trans->nmt points to nmt, nmt->trans points to trans)
 *
 */

static int sms_ref = 0;

/* Timers */
#define PAGING_TO	1,0	/* wait for paging response: fictive value */
#define RELEASE_TO	2,0	/* how long do we wait for release guard of the phone */
#define DIALING_TO	1,0	/* if we have a pause during dialing, we abort the call */
#define CHANNEL_TO	2,0	/* how long do we wait for phone to appear on assigned channel */
#define QUEUE_TO	60,0	/* how long a queued MT call waits on the CC for a free TC */
#define RINGING_TO	60,0	/* how long may the phone ring */
#define SUPERVISORY_TO1	3	/* 3 sec to detect after setup */
#define SUPERVISORY_TO2	20,0	/* 20 sec lost until abort */
#define RECALL_TO	30,0	/* register recall: max time to receive all digits (4.4.1.10) */
#define SCIP_TO		0,553000	/* T' = 553 ms (4 frames): MS re-identify window on the new TC (NMT 450-1 Scheme C2') */
#define DTMF_DURATION	0.1	/* 100ms */

/* Counters */
#define PAGE_TRIES	3	/* How many time do we try to page the phone */

static const char *nmt_state_name(enum nmt_state state)
{
	static char invalid[16];

	switch (state) {
	case STATE_NULL:
		return "(NULL)";
	case STATE_IDLE:
		return "IDLE";
	case STATE_ROAMING_IDENT:
		return "ROAMING IDENT";
	case STATE_ROAMING_CONFIRM:
		return "ROAMING CONFIRM";
	case STATE_MO_IDENT:
		return "MO CALL IDENT";
	case STATE_MO_CONFIRM:
		return "MO CALL CONFIRM";
	case STATE_MO_DIALING:
		return "MO CALL DIALING";
	case STATE_MO_COMPLETE:
		return "MO CALL COMPLETE";
	case STATE_MT_PAGING:
		return "MT CALL PAGING";
	case STATE_MT_QUEUE:
		return "MT CALL QUEUE";
	case STATE_MT_CHANNEL:
		return "MT CALL CHANNEL";
	case STATE_MT_IDENT:
		return "MT CALL IDENT";
	case STATE_MT_AUTOANSWER:
		return "MT CALL AUTOANSWER";
	case STATE_MT_RINGING:
		return "MT CALL RINGING";
	case STATE_MT_COMPLETE:
		return "MT CALL COMPLETE";
	case STATE_ACTIVE:
		return "ACTIVE";
	case STATE_SCIP_ORDER:
		return "SCIP ORDER";
	case STATE_SCIP_IDENT:
		return "SCIP IDENT";
	case STATE_SCIP_IDENT_REQ:
		return "SCIP IDENT REQ";
	case STATE_MO_RELEASE:
		return "RELEASE MTX->MS";
	case STATE_MT_RELEASE:
		return "RELEASE MTX->MS";
	}

	sprintf(invalid, "invalid(%d)", state);
	return invalid;
}

static void nmt_display_status(void)
{
	sender_t *sender;
	nmt_t *nmt;

	display_status_start();
	for (sender = sender_head; sender; sender = sender->next) {
		nmt = (nmt_t *) sender;
		display_status_channel(nmt->sender.kanal, chan_type_short_name(nmt->sysinfo.system, nmt->sysinfo.chan_type), nmt_state_name(nmt->state));
		if (nmt->trans)
			display_status_subscriber(nmt->trans->subscriber.number, NULL);
	}
	display_status_end();
}

static const char *nmt_active_state_name(enum nmt_active_state state)
{
	switch (state) {
	case ACTIVE_STATE_VOICE:		return "voice";
	case ACTIVE_STATE_MFT_IN:		return "MFT-in (ack)";
	case ACTIVE_STATE_MFT:			return "MFT (digit receive)";
	case ACTIVE_STATE_MFT_OUT:		return "MFT-out (ack)";
	case ACTIVE_STATE_MWI:			return "MWI (sending 5c)";
	case ACTIVE_STATE_MWI_CLEAR:		return "MWI-clear (5c+L15)";
	case ACTIVE_STATE_REGISTER_RECALL:	return "register-recall";
	case ACTIVE_STATE_COINBOX_ANSWER:	return "coin-box answer (5b)";
	case ACTIVE_STATE_POWER_CHANGE:		return "power-change (3a)";
	case ACTIVE_STATE_RECALL_PROCEED:	return "register-recall (L(3) proceed)";
	case ACTIVE_STATE_RECALL_DIGITS:	return "register-recall (digit collection)";
	}
	return "unknown";
}

static void nmt_new_state(nmt_t *nmt, enum nmt_state new_state)
{
	if (nmt->state == new_state)
		return;
	LOGP_CHAN(DNMT, LOGL_DEBUG, "State change: %s -> %s\n", nmt_state_name(nmt->state), nmt_state_name(new_state));
	nmt->state = new_state;
	nmt_display_status();
}

static struct nmt_channels {
	int system;
	enum nmt_chan_type chan_type;
	const char *short_name;
	const char *long_name;
} nmt_channels[] = {
	{ 0,	CHAN_TYPE_CC,		"CC",	"calling channel (calls to mobile)" },
	{ 900,	CHAN_TYPE_CCA,		"CCA",	"calling channel for group A mobiles with odd secret key (calls to mobile)" },
	{ 900,	CHAN_TYPE_CCB,		"CCB",	"calling channel for group B mobiles with even secret key (calls to mobile)" },
	{ 0,	CHAN_TYPE_TC,		"TC",	"traffic channel (calls from mobile)" },
	{ 900,	CHAN_TYPE_AC_TC,	"AC/TC","combined access & traffic channel (calls from mobile)" },
	{ 0,	CHAN_TYPE_CC_TC,	"CC/TC","combined calling & traffic channel (both way calls)" },
	{ 0,	CHAN_TYPE_TEST,	"TEST",	"test channel" },
	{ 0,	0, NULL, NULL }
};

void nmt_channel_list(int nmt_system)
{
	int i;

	printf("Type\tDescription\n");
	printf("------------------------------------------------------------------------\n");
	for (i = 0; nmt_channels[i].long_name; i++) {
		if (nmt_channels[i].system != 0 && nmt_channels[i].system != nmt_system)
			continue;
		printf("%s\t%s\n", nmt_channels[i].short_name, nmt_channels[i].long_name);
	}
}

int nmt_channel_by_short_name(int nmt_system, const char *short_name)
{
	int i;

	for (i = 0; nmt_channels[i].short_name; i++) {
		if (nmt_channels[i].system != 0 && nmt_channels[i].system != nmt_system)
			continue;
		if (!strcasecmp(nmt_channels[i].short_name, short_name))
			return nmt_channels[i].chan_type;
	}

	return -1;
}

const char *chan_type_short_name(int nmt_system, enum nmt_chan_type chan_type)
{
	int i;

	for (i = 0; nmt_channels[i].short_name; i++) {
		if (nmt_channels[i].system != 0 && nmt_channels[i].system != nmt_system)
			continue;
		if (nmt_channels[i].chan_type == chan_type)
			return nmt_channels[i].short_name;
	}

	return "invalid";
}

const char *chan_type_long_name(int nmt_system, enum nmt_chan_type chan_type)
{
	int i;

	for (i = 0; nmt_channels[i].long_name; i++) {
		if (nmt_channels[i].system != 0 && nmt_channels[i].system != nmt_system)
			continue;
		if (nmt_channels[i].chan_type == chan_type)
			return nmt_channels[i].long_name;
	}

	return "invalid";
}

const char *nmt_dir_name(enum nmt_direction dir)
{
	switch (dir) {
	case MTX_TO_MS:
		return "MTX->MS";
	case MTX_TO_BS:
		return "MTX->BS";
	case MTX_TO_XX:
		return "MTX->XX";
	case BS_TO_MTX:
		return "BS->MTX";
	case MS_TO_MTX:
		return "MS->MTX";
	case XX_TO_MTX:
		return "XX->MTX";
	}
	return "invalid";
}

/* Human-readable meaning of a line-signal number L(n), per direction
 * (NMT Doc 450-1 para 4.3.3.3, line signals in frames 5 and 13). Used for log
 * text so every line-signal scenario is self-describing rather than a bare
 * number. 'to_ms' selects the MTX->MS column (1) or the MS->MTX column (0). */
const char *nmt_linesignal_name(uint8_t signal, int to_ms)
{
	switch (signal & 0xf) {
	case 0:  return to_ms ? "answer to coin-box" : "spare";
	case 1:  return to_ms ? "spare" : "clearing, release guard";
	case 2:  return to_ms ? "spare" : "answer acknowledgement (coin-box)";
	case 3:  return to_ms ? "proceed to send unencrypted digits / roaming updating confirmation" : "spare";
	case 4:  return to_ms ? "acknowledge MFT converter in" : "spare";
	case 5:  return to_ms ? "switch compander in" : "register recall";
	case 6:  return to_ms ? "address complete" : "spare";
	case 7:  return to_ms ? "switch compander out" : "MFT converter out acknowledge / forced release";
	case 8:  return to_ms ? "spare" : "MFT converter in";
	case 9:  return to_ms ? "ringing order" : "spare";
	case 10: return to_ms ? "acknowledge MFT converter out / forced release" : "spare";
	case 11: return to_ms ? "proceed to send encrypted digits / roaming updating confirmation" : "spare";
	case 12: return to_ms ? "request to receive short message (SMS data)" : "acknowledge SMS request";
	case 13: return to_ms ? "clearing, call transfer activated" : "spare";
	case 14: return to_ms ? "spare" : "answer";
	case 15: return to_ms ? "clearing, call transfer not activated" : "spare";
	}
	return "invalid";
}

/* convert 7-digits dial string to NMT number */
static int dialstring2number(const char *dialstring, char *ms_country, char *ms_number)
{
	if (strlen(dialstring) != 7) {
		LOGP(DNMT, LOGL_NOTICE, "Wrong number of digits, use 7 digits: ZXXXXXX (Z=country, X=mobile number)\n");
		return -1;
	}
	if (dialstring[0] < '0' || dialstring[0] > '9') {
		LOGP(DNMT, LOGL_NOTICE, "Invalid country digit (first digit) of dial string\n");
		return -1;
	}
	*ms_country = dialstring[0];
	memcpy(ms_number, dialstring + 1, 6);
	return 0;
}

static inline int is_chan_class_cc(enum nmt_chan_type chan_type)
{
	if (chan_type == CHAN_TYPE_CC
	 || chan_type == CHAN_TYPE_CCA
	 || chan_type == CHAN_TYPE_CCB
	 || chan_type == CHAN_TYPE_CC_TC)
		return 1;

	return 0;
}

static inline int is_chan_class_tc(enum nmt_chan_type chan_type)
{
	if (chan_type == CHAN_TYPE_TC
	 || chan_type == CHAN_TYPE_AC_TC
	 || chan_type == CHAN_TYPE_CC_TC)
		return 1;

	return 0;
}

static void nmt_timeout(void *data);

/* Create transceiver instance and link to a list. */
int nmt_create(int nmt_system, const char *country, const char *kanal, enum nmt_chan_type chan_type, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int pre_emphasis, int de_emphasis, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, uint8_t ms_power, uint8_t traffic_area, uint8_t area_no, int compandor, int supervisory, const char *smsc_number, int send_callerid, int send_clock, int bs_period, int bs_group, int sis_demo, int coinbox_tariff, int loopback)
{
	nmt_t *nmt;
	int rc;
	double deviation_factor;
	int scandinavia;
	int tested;

	/* check channel matching and set deviation factor */
	if (nmt_channel2freq(nmt_system, country, atoi(kanal), 0, &deviation_factor, &scandinavia, &tested) == 0.0) {
		LOGP(DNMT, LOGL_NOTICE, "Channel number %s invalid, use '-Y list' to get a list of available channels.\n", kanal);
		return -EINVAL;
	}

	if (!tested) {
		LOGP(DNMT, LOGL_NOTICE, "*** The given NMT country has not been tested yet. Please tell the Author, if it works.\n");
	}

	if (scandinavia && atoi(kanal) >= 201) {
		LOGP(DNMT, LOGL_NOTICE, "*** Channels numbers above 200 have been specified, but never used. These 'interleaved channels are probably not supports by the phone.\n");
	}

	if (scandinavia && atoi(kanal) >= 181 && atoi(kanal) <= 200) {
		LOGP(DNMT, LOGL_NOTICE, "Extended channel numbers (181..200) have been specified, but never been supported for sure. There is no phone to test with, so don't use it!\n");
	}

	if (chan_type == CHAN_TYPE_TEST && !loopback) {
		LOGP(DNMT, LOGL_NOTICE, "*** Selected channel can be used for nothing but testing signal decoder.\n");
	}

	if (chan_type == CHAN_TYPE_CC_TC && send_clock) {
		LOGP(DNMT, LOGL_NOTICE, "*** Sending clock on combined CC + TC is not applicable.\n");
	}

	nmt = calloc(1, sizeof(nmt_t));
	if (!nmt) {
		LOGP(DNMT, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	LOGP(DNMT, LOGL_DEBUG, "Creating 'NMT' instance for channel = %s (sample rate %d).\n", kanal, samplerate);

	/* init general part of transceiver */
	rc = sender_create(&nmt->sender, kanal, nmt_channel2freq(nmt_system, country, atoi(kanal), 0, NULL, NULL, NULL), nmt_channel2freq(nmt_system, country, atoi(kanal), 1, NULL, NULL, NULL), device, use_sdr, samplerate, rx_gain, tx_gain, pre_emphasis, de_emphasis, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DNMT, LOGL_ERROR, "Failed to init transceiver process!\n");
		goto error;
	}

	osmo_timer_setup(&nmt->timer, nmt_timeout, nmt);
	nmt->sysinfo.system = nmt_system;
	nmt->sysinfo.chan_type = chan_type;
	nmt->sysinfo.ms_power = ms_power;
	nmt->sysinfo.traffic_area = traffic_area;
	nmt->sysinfo.area_no = area_no;
	nmt->compandor = compandor;
	nmt->supervisory = supervisory;
	nmt->send_callerid = send_callerid;
	nmt->send_clock = send_clock;
	/* optional, off by default: battery saving (period 0 = disabled) and the
	 * SIS authentication demonstrator */
	nmt->bs_period = (bs_period >= 0 && bs_period <= 9) ? bs_period : 0;
	nmt->bs_group = (bs_group >= 0 && bs_group <= 8) ? bs_group : 8;
	nmt->sis_demo = sis_demo ? 1 : 0;
	/* coin-box tariff class: no charging/PSTN source in this emulator, so it
	 * is disabled by default (-1); an operator may set 0..3 to exercise the
	 * frame-5b answer sequence. Any out-of-range value disables it. */
	nmt->coinbox_tariff = (coinbox_tariff >= 0 && coinbox_tariff <= 3) ? coinbox_tariff : -1;
	strncpy(nmt->smsc_number, smsc_number, sizeof(nmt->smsc_number) - 1);

	/* init audio processing */
	rc = dsp_init_sender(nmt, deviation_factor);
	if (rc < 0) {
		LOGP(DNMT, LOGL_ERROR, "Failed to init audio processing!\n");
		goto error;
	}

	/* init DMS processing */
	rc = dms_init_sender(nmt);
	if (rc < 0) {
		LOGP(DNMT, LOGL_ERROR, "Failed to init DMS processing!\n");
		goto error;
	}

	/* init SMS processing */
	rc = sms_init_sender(nmt);
	if (rc < 0) {
		LOGP(DNMT, LOGL_ERROR, "Failed to init SMS processing!\n");
		goto error;
	}

	/* go into idle state */
	nmt_go_idle(nmt);

	LOGP(DNMT, LOGL_NOTICE, "Created channel #%s of type '%s' = %s\n", kanal, chan_type_short_name(nmt_system, chan_type), chan_type_long_name(nmt_system, chan_type));
	if (nmt_long_name_by_short_name(nmt_system, country))
	LOGP(DNMT, LOGL_NOTICE, " -> Using country '%s'\n", nmt_long_name_by_short_name(nmt_system, country));
	LOGP(DNMT, LOGL_NOTICE, " -> Using traffic area %d,%d and area no %d\n", traffic_area >> 4, (nmt_system == 450) ? nmt_flip_ten((traffic_area & 0xf)) : (traffic_area & 0xf), area_no);
	if (nmt->supervisory)
		LOGP(DNMT, LOGL_NOTICE, " -> Using supervisory signal %d\n", supervisory);
	else
		LOGP(DNMT, LOGL_NOTICE, " -> Using no supervisory signal\n");

	return 0;

error:
	nmt_destroy(&nmt->sender);

	return rc;
}

void nmt_check_channels(int __attribute__((unused)) nmt_system)
{
	sender_t *sender;
	nmt_t *nmt;
	int cca = 0, ccb = 0, tc = 0;
	int note = 0;

	for (sender = sender_head; sender; sender = sender->next) {
		nmt = (nmt_t *) sender;
		if (nmt->sysinfo.chan_type == CHAN_TYPE_CC) {
			cca = 1;
			ccb = 1;
		}
		if (nmt->sysinfo.chan_type == CHAN_TYPE_CCA)
			cca = 1;
		if (nmt->sysinfo.chan_type == CHAN_TYPE_CCB)
			ccb = 1;
		if (nmt->sysinfo.chan_type == CHAN_TYPE_TC)
			tc = 1;
		if (nmt->sysinfo.chan_type == CHAN_TYPE_AC_TC)
			tc = 1;
		if (nmt->sysinfo.chan_type == CHAN_TYPE_CC_TC) {
			cca = 1;
			ccb = 1;
			tc = 1;
		}
	}
	if ((cca || ccb) && !tc) {
		LOGP(DNMT, LOGL_NOTICE, "\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Selected channel(s) can be used for control only.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** No registration and no call is possible.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Use at least one 'TC' or use combined 'CC/TC'!\n");
		note = 1;
	}
	if (tc && !(cca || ccb)) {
		LOGP(DNMT, LOGL_NOTICE, "\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Selected channel(s) can be used for traffic only.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** No call to the mobile phone is possible.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Use one 'CC' or use combined 'CC/TC'!\n");
		note = 1;
	}
	if (cca && !ccb) {
		LOGP(DNMT, LOGL_NOTICE, "\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Selected channel(s) can be used for control of MS type A only.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** No call to the MS type B phone is possible.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Use one 'CC' instead!\n");
		note = 1;
	}
	if (!cca && ccb) {
		LOGP(DNMT, LOGL_NOTICE, "\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Selected channel(s) can be used for control of MS type B only.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** No call to the MS type A phone is possible.\n");
		LOGP(DNMT, LOGL_NOTICE, "*** Use one 'CC' instead!\n");
		note = 1;
	}
	if (note)
		LOGP(DNMT, LOGL_NOTICE, "\n");
}

/* Destroy transceiver instance and unlink from list. */
void nmt_destroy(sender_t *sender)
{
	nmt_t *nmt = (nmt_t *) sender;

	LOGP(DNMT, LOGL_DEBUG, "Destroying 'NMT' instance for channel = %s.\n", sender->kanal);
	dsp_cleanup_sender(nmt);
	dms_cleanup_sender(nmt);
	sms_cleanup_sender(nmt);
	osmo_timer_del(&nmt->timer);
	sender_destroy(&nmt->sender);
	free(nmt);
}

/* Abort connection towards mobile station by sending idle digits. */
void nmt_go_idle(nmt_t *nmt)
{
	osmo_timer_del(&nmt->timer);
	dms_reset(nmt);
	sms_reset(nmt);

	LOGP_CHAN(DNMT, LOGL_INFO, "Entering IDLE state, sending idle frames on %s.\n", chan_type_long_name(nmt->sysinfo.system, nmt->sysinfo.chan_type));
	nmt->trans = NULL; /* remove transaction before state change, so status is shown correctly */
	nmt_new_state(nmt, STATE_IDLE);
	nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
	memset(&nmt->dialing, 0, sizeof(nmt->dialing));

#if 0
	/* go active for loopback tests */
	nmt_new_state(nmt, STATE_ACTIVE);
	nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
#endif
}

/* release an ongoing connection, this is used by roaming update and release initiated by MTX */
static void nmt_release(nmt_t *nmt)
{
	transaction_t *trans = nmt->trans;

	osmo_timer_del(&nmt->timer);

	LOGP_CHAN(DNMT, LOGL_INFO, "Releasing connection towards mobile station.\n");
	if (trans->callref) {
		LOGP_CHAN(DNMT, LOGL_ERROR, "Callref already set, please fix!\n");
		abort();
	}

	/* If MWI was set during call, use clearing sequence per NMT spec 4.4.1.16:
	 * 5c x2 (MWI data) then L(15) x4 (triggers phone to apply flags + release)
	 */
	if (trans->mwi_pending && nmt->state == STATE_ACTIVE) {
		LOGP_CHAN(DNMT, LOGL_INFO, "MWI pending (0x%02x), sending clearing sequence.\n", trans->mwi_pending);
		trans->mwi_flags = trans->mwi_pending;  /* Copy for clearing sequence */
		trans->mwi_pending = 0;  /* Clear to prevent restart */
		nmt->active_state = ACTIVE_STATE_MWI_CLEAR;
		nmt->tx_frame_count = 0;
		nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
		return;
	}

	nmt_new_state(nmt, STATE_MT_RELEASE);
	nmt->tx_frame_count = 0;
	nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
	osmo_timer_schedule(&nmt->timer, RELEASE_TO);
}

/* Battery-saving group membership (NMT Doc 450-1 para 4.3.3.12.3): does the
 * phone whose last subscriber digit is x6 belong to the group addressed by H7? */
static int nmt_bs_group_contains(int h7, int x6)
{
	switch (h7) {
	case 1: return (x6 & 1) == 1;		/* 1,3,5,7,9 */
	case 2: return (x6 & 1) == 0;		/* 2,4,6,8,0 */
	case 3: return x6 == 1 || x6 == 2;
	case 4: return x6 == 3 || x6 == 4;
	case 5: return x6 == 5 || x6 == 6;
	case 6: return x6 == 7 || x6 == 8;
	case 7: return x6 == 9 || x6 == 0;
	case 8: return 1;			/* all phones */
	default: return 0;			/* 0 and 9..15: no group */
	}
}

/* Battery-saving period for H3 code (NMT Doc 450-1 para 4.3.3.12.4), in frames
 * (1 frame = 138.33 ms). Returns 0 for H3=0 (disabled) and the spare codes. */
static int nmt_bs_period_frames(int h3)
{
	static const int tbl[10] = { 0, 24, 40, 56, 72, 88, 112, 168, 224, 280 };
	if (h3 < 1 || h3 > 9)
		return 0;
	return tbl[h3];
}

/* Enter paging state and transmit phone's number on calling channel.
 *
 * Battery-saving interaction (when enabled, bs_period>0): a phone asleep for its
 * group's H3 period will not hear an immediate page, so the first attempt is
 * DEFERRED by one H3 period (see below), which guarantees the phone has woken
 * before we page. The deferral is group-aware: only subscribers whose last
 * digit X6 falls in the addressed H7 group are held; others page immediately. */
static void nmt_page(transaction_t *trans, int try)
{
	sender_t *sender;
	nmt_t *nmt;

	/* Battery-saving page deferral (NMT Doc 450-1 para 4.3.3.12): on the first
	 * attempt, if any calling channel runs battery saving and this subscriber's
	 * group is addressed by it, the phone may be asleep. Hold the page for one
	 * H3 period, which guarantees the phone has woken (whatever its phase)
	 * before we page; timeout_mt_paging() then performs the real page. */
	if (try == 1 && !trans->page_deferred) {
		int x6 = trans->subscriber.number[5] - '0';
		if (x6 < 0 || x6 > 9)
			x6 = -1;	/* malformed number: matches no group, pages immediately */
		for (sender = sender_head; sender; sender = sender->next) {
			nmt = (nmt_t *)sender;
			if (!is_chan_class_cc(nmt->sysinfo.chan_type))
				continue;
			if (nmt->bs_period > 0 && x6 >= 0 && nmt_bs_group_contains(nmt->bs_group, x6)) {
				int frames = nmt_bs_period_frames(nmt->bs_period);
				/* 1 frame = 138.33 ms; round up to whole seconds for the timer */
				int secs = (frames * 13833 + 999999) / 1000000;
				if (secs < 1)
					secs = 1;
				LOGP(DNMT, LOGL_INFO, "Battery saving active for this subscriber's group (H3=%d, %d frames); deferring page ~%d s to the wake window.\n", nmt->bs_period, frames, secs);
				trans->page_deferred = 1;
				trans->page_try = try;
				osmo_timer_schedule(&trans->timer, secs, 0);
				return;
			}
		}
	}

	LOGP(DNMT, LOGL_INFO, "Entering paging state (try %d), sending call to '%c,%s'.\n", try, trans->subscriber.country, trans->subscriber.number);
	trans->page_try = try;
	osmo_timer_schedule(&trans->timer, PAGING_TO);
	/* page on all CC (CC/TC) */
	for (sender = sender_head; sender; sender = sender->next) {
		nmt = (nmt_t *)sender;
		if (!is_chan_class_cc(nmt->sysinfo.chan_type))
			continue;
		/* page on all idle channels and on channels we previously paged */
		if (nmt->state != STATE_IDLE && nmt->trans != trans)
			continue;
		LOGP(DNMT, LOGL_INFO, "Paging on channel %s.\n", sender->kanal);
		nmt->trans = trans; /* add transaction before state change, so status is shown correctly */
		nmt_new_state(nmt, STATE_MT_PAGING);
		nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
		nmt->tx_frame_count = 0;
	}
}

static nmt_t *search_free_tc(nmt_t *own)
{
	sender_t *sender;
	nmt_t *nmt, *cc_tc = NULL;

	for (sender = sender_head; sender; sender = sender->next) {
		nmt = (nmt_t *) sender;
		/* if our CC is used, we don't care about busy state,
		 * because it can be used, if it is CC/TC type */
		if (nmt != own && nmt->state != STATE_IDLE)
			continue;
		/* remember combined voice/control/paging channel as second alternative */
		if (nmt->sysinfo.chan_type == CHAN_TYPE_CC_TC)
			cc_tc = nmt;
		else if (is_chan_class_tc(nmt->sysinfo.chan_type))
			return nmt;
	}

	return cc_tc;
}


/*
 * frame matching functions to check if channels is accessed correctly
 */

/* check match channel no, area no and traffic area */
static int match_channel(nmt_t *nmt, frame_t *frame)
{
	int channel, power;
	int rc;

	/* check channel match */
	rc = nmt_decode_channel(nmt->sysinfo.system, frame->channel_no, &channel, &power);
	if (rc < 0) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Frame with illegal encoded channel received, ignoring.\n");
		return 0;
	}
	/* in case of interleaved channel, ignore the missing upper bit */
	if ((channel % 1024) != (atoi(nmt->sender.kanal) % 1024)) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Frame for different channel %d received, ignoring.\n", channel);
		return 0;
	}

	return 1;
}

static int match_area(nmt_t *nmt, frame_t *frame)
{
	uint8_t area_no, traffic_area;

	/* old phones do not support ZY digits */
	if (frame->area_info + frame->traffic_area == 0)
		goto skip_area;

	area_no = frame->area_info >> 2;
	if (area_no == 0 && nmt->sysinfo.area_no != 0)
		area_no = 4;
	if (area_no != nmt->sysinfo.area_no) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received area no (%d) does not match the base station's area no (%d), ignoring.\n", area_no, nmt->sysinfo.area_no);
		return 0;
	}

	traffic_area = ((frame->area_info & 0x3) << 4) | frame->traffic_area;
	if (nmt->sysinfo.traffic_area != 0 && (nmt->sysinfo.traffic_area & 0x3f) != traffic_area) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received 6 bits of traffic area (0x%02x) does not match the 6 bits of base station's traffic area (0x%02x), ignoring.\n", nmt->sysinfo.traffic_area & 0x3f, traffic_area);
		return 0;
	}
skip_area:

	return 1;
}

/* check match subscriber number */
static int match_subscriber(transaction_t *trans, frame_t *frame)
{
	if (nmt_digits2value(&trans->subscriber.country, 1) != frame->ms_country) {
		LOGP(DNMT, LOGL_NOTICE, "Received non matching subscriber counrtry, ignoring.\n");
		return 0;
	}
	if (nmt_digits2value(trans->subscriber.number, 6) != frame->ms_number) {
		LOGP(DNMT, LOGL_NOTICE, "Received non matching subscriber number, ignoring.\n");
		return 0;
	}

	return 1;
}

/*
 * helper functions to generate frames
 */

static void tx_ident(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	/* SIS authentication DEMONSTRATOR (optional, off by default): on the first
	 * identity frame, originate frame 7 (authentication request) carrying a
	 * RAND. This exercises the frame-7/frame-16 exchange only -- there is NO
	 * real SIS algorithm (the SAK is a secret operator key, not public), so the
	 * SRES cannot be checked. Labeled as a demo so it is never mistaken for
	 * working authentication. */
	if (nmt->sis_demo && nmt->tx_frame_count == 0) {
		frame->mt = NMT_MESSAGE_7;
		frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
		frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
		frame->rand = 0x1234567; /* fixed demo RAND (C1..C7); NOT cryptographic */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "SIS DEMO: sending authentication request (frame 7, RAND=0x%07x) -- demonstrator only, no SIS algorithm, SRES will not be verified.\n", (unsigned)frame->rand);
		return;
	}

	frame->mt = NMT_MESSAGE_3b;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	frame->additional_info = nmt_encode_area_no(nmt->sysinfo.area_no);
}

static void set_line_signal(nmt_t *nmt, frame_t *frame, uint8_t signal)
{
	transaction_t *trans = nmt->trans;

	frame->mt = NMT_MESSAGE_5a;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	frame->line_signal = (signal << 8) | (signal << 4) | signal;
	LOGP_CHAN(DNMT, LOGL_INFO, "Sending line signal L(%d) '%s'.\n",
		signal & 0xf, nmt_linesignal_name(signal, 1 /* MTX->MS */));
}

/* Build frame 5b, "Answer to coin-box" (NMT Doc 450-1 4.2.1/4.7.2, format
 * NNNPYYZXXXXXXLQQ): line signal L(0) = answer-to-coin-box carrying the tariff
 * class Q1Q2. The coin-box MS uses the tariff class to meter the call, and
 * echoes it back for control in frame 13b. The tariff value is operator-supplied
 * (nmt->coinbox_tariff); callers must only invoke this when a class is configured. */
static void set_coinbox_answer(nmt_t *nmt, frame_t *frame, uint8_t tariff_class)
{
	transaction_t *trans = nmt->trans;

	frame->mt = NMT_MESSAGE_5b;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	frame->line_signal = 0;			/* L(0) = answer to coin-box */
	frame->tariff_class = tariff_class & 0x3;
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Sending coin-box answer (frame 5b): L(0) answer-to-coin-box, tariff class 0x%x (operator-supplied).\n", frame->tariff_class);
}

/*
 * handle idle channel
 */

static void tx_idle(nmt_t *nmt, frame_t *frame)
{
	time_t time_sec;
	struct tm *tm;
	uint16_t clock;

	switch (nmt->sysinfo.chan_type) {
	case CHAN_TYPE_CC:
		frame->mt = NMT_MESSAGE_1a;
		break;
	case CHAN_TYPE_CCA:
		frame->mt = NMT_MESSAGE_1a_a;
		break;
	case CHAN_TYPE_CCB:
		frame->mt = NMT_MESSAGE_1a_b;
		break;
	case CHAN_TYPE_TC:
		frame->mt = NMT_MESSAGE_4;
		break;
	case CHAN_TYPE_AC_TC:
		frame->mt = NMT_MESSAGE_4b;
		break;
	case CHAN_TYPE_CC_TC:
		frame->mt = NMT_MESSAGE_1b;
		break;
	case CHAN_TYPE_TEST:
		frame->mt = NMT_MESSAGE_30;
		break;
	}

	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);

	/* additional info */
	frame->additional_info = 0;
	if (frame->mt == NMT_MESSAGE_1a || frame->mt == NMT_MESSAGE_1a_a || frame->mt == NMT_MESSAGE_1a_b || frame->mt == NMT_MESSAGE_1b) {
		/* Battery-saving additional info (NMT Doc 450-1 para 4.3.3.12):
		 * H1=14(E) H2=11(B) marker, H3 = period code, H7 = addressed group.
		 * Default bs_period=0 means "no saving" (phones never sleep) = the
		 * historic behaviour; a non-zero period enables battery saving. */
		frame->additional_info |= 0xeb00000000;			/* H1=E, H2=B marker */
		frame->additional_info |= ((uint64_t)(nmt->bs_period & 0xf)) << 28;	/* H3 period */
		frame->additional_info |= ((uint64_t)(nmt->bs_group & 0xf)) << 12;	/* H7 group */
		/* phone is allowed to send overdecadic dialing digits */
		frame->additional_info |= 0x0000020000;
		/* no clock on combined CC+TC */
		if (nmt->send_clock && frame->mt != NMT_MESSAGE_1b) {
			/* send battery saving message including clock */
			time_sec = get_time();
			tm = localtime(&time_sec);
			clock = (1 << 11) | (tm->tm_hour << 6) | tm->tm_min;
			/* add clock with flag */
			frame->additional_info |= clock;
		}
	}
	if (frame->mt == NMT_MESSAGE_1b || frame->mt == NMT_MESSAGE_4 || frame->mt == NMT_MESSAGE_4b || frame->mt == NMT_MESSAGE_30) {
		/* sent area info on traffic channels; it is always H8H9H10, because all IEs are aligned 'to the right' */
		frame->additional_info |= nmt_encode_area_no(nmt->sysinfo.area_no);
	}
	/* NMT 900 §4.3.3.13 channel band information (STANDARD POINT REACHED):
	 * On calling channels (1a/1a_a/1a_b/1b), H1H2 may signal the start/end of the
	 * CC band (H3H4..H5H6, §4.3.3.13.3) and TC/AC band (H7H8..H9H10, §4.3.3.13.4)
	 * to allow an MS to scan a wider or different band than its preprogrammed basic
	 * band. H1(14) H2(0) signals CC+TC band info present; H1(14) H2(12) signals
	 * CC+AC band info. If H3H4=H5H6=00 or H7H8=H9H10=00, the preprogrammed band
	 * is used. Currently this node always sends H1=E H2=B (battery saving marker)
	 * for NMT 450, and NMT 900 uses the same encoding -- the channel band fields
	 * (H3-H10) are always zero, so the MS falls back to its basic channel band.
	 * Full band advertisement would require --cc-band-start/--cc-band-end options
	 * and is not implemented. */
}

static void rx_idle(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans;
	nmt_subscriber_t subscr;

	switch (frame->mt) {
	case NMT_MESSAGE_11a: /* roaming update and seizure */
		if (!match_channel(nmt, frame))
			break;
		if (!match_area(nmt, frame))
			break;
		/* set subscriber */
		memset(&subscr, 0, sizeof(subscr));
		nmt_value2digits(frame->ms_country, &subscr.country, 1);
		nmt_value2digits(frame->ms_number, subscr.number, 6);

		LOGP_CHAN(DNMT, LOGL_INFO, "Received roaming seizure from subscriber %c,%s\n", subscr.country, subscr.number);

		/* create transaction */
		trans = create_transaction(&subscr);
		if (!trans) {
			LOGP(DNMT, LOGL_NOTICE, "Failed to create transaction!\n");
			break;
		}

		/* change state */
		nmt->trans = trans; /* add transaction before state change, so status is shown correctly */
		nmt_new_state(nmt, STATE_ROAMING_IDENT);
		trans->nmt = nmt;
		nmt->rx_frame_count = 0;
		nmt->tx_frame_count = 0;
		break;
	case NMT_MESSAGE_11b: /* priority MS (PMS) outgoing call (NMT Doc 450-1 4.4.1.7):
			       * the MS pressed the priority button to make an outgoing call.
			       * Frame 11b carries the subscriber number (NNNPZXXXXXXTJJJJ).
			       * We page the MS with frame 2a to verify its identity, then
			       * allocate a TC, and after "answer" (L=14) send L(3) to proceed
			       * to unencrypted digit transmission (states 14-16 without SIS). */
		if (!match_channel(nmt, frame))
			break;
		if (!is_chan_class_cc(nmt->sysinfo.chan_type)) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Received PMS priority seizure (11b) on non-CC channel, ignoring.\n");
			break;
		}
		if (!match_area(nmt, frame))
			break;
		/* extract subscriber from the 11b frame */
		memset(&subscr, 0, sizeof(subscr));
		nmt_value2digits(frame->ms_country, &subscr.country, 1);
		nmt_value2digits(frame->ms_number, subscr.number, 6);
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received PMS priority seizure (frame 11b) from subscriber %c,%s -- MS wants to place a priority outgoing call (4.4.1.7).\n",
			subscr.country, subscr.number);
		/* create a transaction for the PMS outgoing call; callref = 0
		 * (there is no incoming network call -- the MS is calling out) */
		trans = create_transaction(&subscr);
		if (!trans) {
			LOGP(DNMT, LOGL_NOTICE, "Failed to create transaction for PMS call!\n");
			break;
		}
		trans->priority = 1;		/* priority queueing (2c) if TC not immediately free */
		trans->priority_mo = 1;		/* MO flag: post-answer → L(3) proceed + digit collection */
		/* page the MS on this CC to verify its identity, exactly as the
		 * standard's state 2a specifies ("MTX checks identity by sending normal
		 * paging 2a to PMS"). nmt_page() handles the 3× paging retries. */
		nmt->trans = trans;
		trans->nmt = nmt;
		nmt_page(trans, 1);
		break;
	case NMT_MESSAGE_10b: /* seizure from ordinary MS */
	case NMT_MESSAGE_12: /* seizure from coinbox MS */
	case NMT_MESSAGE_10a: /* access signal */
		if (!match_channel(nmt, frame))
			break;
		if (!match_area(nmt, frame))
			break;
		/* set subscriber */
		memset(&subscr, 0, sizeof(subscr));
		nmt_value2digits(frame->ms_country, &subscr.country, 1);
		nmt_value2digits(frame->ms_number, subscr.number, 6);
		if (frame->mt == NMT_MESSAGE_12)
			subscr.coinbox = 1;

		LOGP_CHAN(DNMT, LOGL_INFO, "Received call from subscriber %c,%s%s\n", subscr.country, subscr.number, (subscr.coinbox) ? " (coinbox)" : "");

		/* create transaction */
		trans = create_transaction(&subscr);
		if (!trans) {
			LOGP(DNMT, LOGL_NOTICE, "Failed to create transaction!\n");
			break;
		}

		/* change state */
		nmt->trans = trans; /* add transaction before state change, so status is shown correctly */
		nmt_new_state(nmt, STATE_MO_IDENT);
		trans->nmt = nmt;
		nmt->rx_frame_count = 0;
		nmt->tx_frame_count = 0;
		break;
	case NMT_MESSAGE_10c: /* called-MS seizure on TC after blocking/congestion path (§4.4.1.2.2, B1;
			       * §4.4.1.14, B4). The MS found this TC independently after receiving the
			       * 2d blocking order and sends 10c as its seizure + identity.
			       * Find the MT transaction paging this subscriber and bind it to this TC. */
		if (!match_channel(nmt, frame))
			break;
		if (!is_chan_class_tc(nmt->sysinfo.chan_type)) {
			LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping frame 10c on non-TC channel.\n");
			break;
		}
		{
			nmt_subscriber_t tsubscr;
			memset(&tsubscr, 0, sizeof(tsubscr));
			nmt_value2digits(frame->ms_country, &tsubscr.country, 1);
			nmt_value2digits(frame->ms_number, tsubscr.number, 6);
			trans = get_transaction_by_number(&tsubscr);
			if (!trans) {
				LOGP_CHAN(DNMT, LOGL_NOTICE, "Received TC seizure (frame 10c) from %c,%s but no pending MT transaction; ignoring.\n",
					tsubscr.country, tsubscr.number);
				break;
			}
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Received TC seizure (frame 10c) from %c,%s matching pending MT call; binding TC and requesting identity.\n",
				tsubscr.country, tsubscr.number);
			/* bind transaction to this TC (was on CC during paging) */
			if (trans->nmt && trans->nmt != nmt)
				nmt_go_idle(trans->nmt);
			trans->nmt = nmt;
			nmt->trans = trans;
			osmo_timer_del(&trans->timer);
			nmt_new_state(nmt, STATE_MT_IDENT);
			nmt->tx_frame_count = 0;
			nmt->rx_frame_count = 0;
		}
		break;
	/* signals after release */
	case NMT_MESSAGE_13a: /* line signal */
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

/*
 * handle roaming
 */

static void tx_roaming_ident(nmt_t *nmt, frame_t *frame)
{
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Sending identity request.\n");
	tx_ident(nmt, frame);
	if (nmt->tx_frame_count == 8) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout waiting for identity reply\n");
		nmt_release(nmt);
	}
}

static void rx_roaming_ident(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_11a: /* roaming update */
		if (!match_channel(nmt, frame))
			break;
		if (!match_area(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		if (nmt->rx_frame_count < 2) {
			LOGP_CHAN(DNMT, LOGL_DEBUG, "Skipping second seizure frame\n");
			break;
		}
		nmt_value2digits(frame->ms_password, trans->subscriber.password, 3);
		LOGP_CHAN(DNMT, LOGL_INFO, "Received identity confirm (password %s).\n", trans->subscriber.password);
		nmt_new_state(nmt, STATE_ROAMING_CONFIRM);
		nmt->tx_frame_count = 0;
		console_inscription(&trans->subscriber.country);
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

static void tx_roaming_confirm(nmt_t *nmt, frame_t *frame)
{
	/* L(3) = "proceed to send unencrypted digits (roaming updating
	 * confirmation)" per NMT Doc 450-1 para 4.3.3.3. L(11) is the encrypted
	 * (SIS) variant, which is not used here (SIS is not implemented).
	 * §4.4.1.5 Scheme D (p.65) shows L(3) sent 4 times before the forced
	 * release (5a(L=13/15)). */
	set_line_signal(nmt, frame, 3);
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Send 'Roaming updating confirmation'.\n");
	if (nmt->tx_frame_count == 4)
		nmt_release(nmt); /* continue with this frame, then release */
}

static void rx_roaming_confirm(nmt_t *nmt, frame_t *frame)
{
	switch (frame->mt) {
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

/*
 * handle call MS -> MTX
 */

static void tx_mo_ident(nmt_t *nmt, frame_t *frame)
{
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Sending identity request.\n");
	tx_ident(nmt, frame);
	if (nmt->tx_frame_count == 8) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout waiting for identity reply\n");
		nmt_release(nmt);
	}
}

static void rx_mo_ident(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_10b: /* seizure */
	case NMT_MESSAGE_12: /* seizure */
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		if (nmt->rx_frame_count < 2) {
			LOGP_CHAN(DNMT, LOGL_DEBUG, "Skipping second seizure frame\n");
			break;
		}
		nmt_value2digits(frame->ms_password, trans->subscriber.password, 3);
		LOGP_CHAN(DNMT, LOGL_INFO, "Received identity confirm (password %s).\n", trans->subscriber.password);
		nmt_new_state(nmt, STATE_MO_CONFIRM);
		nmt->tx_frame_count = 0;
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

static void tx_mo_confirm(nmt_t *nmt, frame_t *frame)
{
	set_line_signal(nmt, frame, 3);
	if (++nmt->tx_frame_count <= 2) {
		if (nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Send 'Proceed to send'.\n");
	} else {
		if (nmt->tx_frame_count == 3) {
			LOGP_CHAN(DNMT, LOGL_INFO, "Send dial tone.\n");
			nmt_new_state(nmt, STATE_MO_DIALING);
			nmt_set_dsp_mode(nmt, DSP_MODE_DIALTONE);
			osmo_timer_schedule(&nmt->timer, DIALING_TO);
		}
	}
}

static void rx_mo_dialing(nmt_t *nmt, frame_t *frame)
{
	int len = strlen(nmt->dialing);
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_14a: /* digits */
		if (!match_channel(nmt, frame))
			break;
		if (!match_area(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		osmo_timer_schedule(&nmt->timer, DIALING_TO);
		/* max digits received */
		if (len + 1 == sizeof(nmt->dialing))
			break;
		if ((len & 1)) {
			/* received odd digit, but be already have odd number of digits */
			if (nmt->rx_frame_count > 1) /* we lost even digit */
				goto missing_digit;
			break;
		} else if (len) { /* complain only after first digit */
			/* received odd digit, and we have even number of digits */
			if (nmt->rx_frame_count > 3) /* we lost even digit */
				goto missing_digit;
		}
		if ((frame->digit >> 12) != 0x00) /* digit 0x0 0x0, x, x, x */
			goto not_right_position;
		if (((frame->digit >> 8) & 0xf) != ((frame->digit >> 4) & 0xf)
		 || ((frame->digit >> 4) & 0xf) != (frame->digit & 0xf))
			goto not_consistent_digit;
		nmt->dialing[len] = nmt_value2digit(frame->digit);
		nmt->dialing[len + 1] = '\0';
		LOGP_CHAN(DNMT, LOGL_INFO, "Received (odd)  digit %c.\n", nmt->dialing[len]);
		nmt->rx_frame_count = 0;
		/* finish dial tone after first digit */
		if (!len)
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
		break;
	case NMT_MESSAGE_14b: /* digits */
		if (!match_channel(nmt, frame))
			break;
		if (!match_area(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		osmo_timer_schedule(&nmt->timer, DIALING_TO);
		/* max digits received */
		if (len + 1 == sizeof(nmt->dialing))
			break;
		/* received even digit, but no digit yet, so we lost first odd digit */
		if (!len)
			goto missing_digit;
		if (!(len & 1)) {
			/* received even digit, but be already have even number of digits */
			if (nmt->rx_frame_count > 1) /* we lost odd digit */
				goto missing_digit;
			break;
		} else {
			/* received even digit, and we have odd number of digits */
			if (nmt->rx_frame_count > 3) /* we lost odd digit */
				goto missing_digit;
		}
		if ((frame->digit >> 12) != 0xff) /* digit 0xf 0xf, x, x, x */
			goto not_right_position;
		if (((frame->digit >> 8) & 0xf) != ((frame->digit >> 4) & 0xf)
		 || ((frame->digit >> 4) & 0xf) != (frame->digit & 0xf))
			goto not_consistent_digit;
		nmt->dialing[len] = nmt_value2digit(frame->digit);
		nmt->dialing[len + 1] = '\0';
		LOGP_CHAN(DNMT, LOGL_INFO, "Received (even) digit %c.\n", nmt->dialing[len]);
		nmt->rx_frame_count = 0;
		break;
	case NMT_MESSAGE_15: /* idle */
		if (!len)
			break;
		if (nmt->dialing[0] == 'A') {
			nmt->dialing[0] = '+';
			LOGP_CHAN(DNMT, LOGL_INFO, "Dialing includes international '+' sign at the beginning.\n");
		}
		if (nmt->dialing[0] == 'B') {
			const char *code = NULL;
			switch (nmt->dialing[1]) {
			case '1':
				code = "general emergency number";
				break;
			case '2':
				code = "fire alarm";
				break;
			case '3':
				code = "police";
				break;
			case '4':
				code = "ambulance";
				break;
			case '5':
				code = "gas emergency";
				break;
			case '6':
				code = "directory inquiry (national)";
				break;
			case '7':
				code = "directory inquiry (international)";
				break;
			case '8':
				code = "operator assisted service (to make outgoing calls)";
				break;
			case '9':
				code = "local customer care";
				break;
			case 'B':
				code = "road service";
				break;
			case 'C':
				code = "weather";
				break;
			}
			if (code)
				LOGP_CHAN(DNMT, LOGL_INFO, "Dialing includes service code: '%c%c' = '%s'\n", nmt->dialing[0], nmt->dialing[1], code);
		}
		LOGP_CHAN(DNMT, LOGL_INFO, "Dialing complete %s->%s, call established.\n", &trans->subscriber.country, nmt->dialing);
		if (nmt->dialing[0] == 'B')
			nmt->dialing[0] = '+';
		/* setup call */
		if (!strcmp(nmt->dialing, nmt->smsc_number)) {
			/* SMS */
			LOGP(DNMT, LOGL_INFO, "Setup call to SMSC.\n");
			trans->dms_call = 1;
		} else {
			LOGP(DNMT, LOGL_INFO, "Setup call to network.\n");
			trans->callref = call_up_setup(&trans->subscriber.country, nmt->dialing, OSMO_CC_NETWORK_NMT_NONE, "");
		}
		osmo_timer_del(&nmt->timer);
		nmt_new_state(nmt, STATE_MO_COMPLETE);
		nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
		nmt->tx_frame_count = 0;
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}

	return;

missing_digit:
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Missing digit, aborting.\n");
	nmt_release(nmt);
	return;

not_right_position:
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Position information of digit does not match, ignoring due to corrupt frame.\n");
	return;

not_consistent_digit:
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Digit repetition in frame does not match, ignoring due to corrupt frame.\n");
	return;
}

static void tx_mo_complete(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	if (++nmt->tx_frame_count <= 4) {
		set_line_signal(nmt, frame, 6);
		if (nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Send 'address complete'.\n");
	} else {
		if (nmt->compandor) {
			set_line_signal(nmt, frame, 5);
			if (nmt->tx_frame_count == 5)
				LOGP_CHAN(DNMT, LOGL_INFO, "Send 'compandor in'.\n");
		} else
			frame->mt = NMT_MESSAGE_6;
		if (nmt->tx_frame_count == 9) {
			LOGP_CHAN(DNMT, LOGL_INFO, "Connect audio.\n");
			nmt_new_state(nmt, STATE_ACTIVE);
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			if (nmt->supervisory && !trans->dms_call) {
				super_reset(nmt);
				osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO1,0);
			}
		}
	}
}

static void timeout_mo_dialing(nmt_t *nmt)
{
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout while receiving digits.\n");
	nmt_release(nmt);
	LOGP(DNMT, LOGL_INFO, "Release call towards network.\n");
}

/*
 * handle call MTX -> MS
 */
static void tx_mt_paging(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	frame->mt = NMT_MESSAGE_2a;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	frame->additional_info = nmt_encode_area_no(nmt->sysinfo.area_no);
	if (++nmt->tx_frame_count == 1) {
		LOGP_CHAN(DNMT, LOGL_INFO, "Send call to mobile.\n");
	} else
		tx_idle(nmt, frame);
}

void timeout_mt_paging(transaction_t *trans)
{
	/* battery saving: the deferral timer fired, the group's wake window has
	 * arrived -- now actually page (this was not a failed attempt). */
	if (trans->page_deferred) {
		trans->page_deferred = 0;
		LOGP(DNMT, LOGL_INFO, "Battery-saving wake window reached, paging now.\n");
		nmt_page(trans, 1);
		return;
	}
	LOGP(DNMT, LOGL_NOTICE, "No answer from mobile phone (try %d).\n", trans->page_try);
	if (trans->page_try == PAGE_TRIES) {
		LOGP(DNMT, LOGL_INFO, "Release call towards network.\n");
		if (trans->callref)
			call_up_release(trans->callref, CAUSE_OUTOFORDER);
		destroy_transaction(trans);
		return;
	}
	nmt_page(trans, trans->page_try + 1);
}

static void rx_mt_paging(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;
	sender_t *sender;
	nmt_t *other;

	switch (frame->mt) {
	case NMT_MESSAGE_11b: /* priority MS call acknowledgment (PMS) */
	case NMT_MESSAGE_10a: /* call acknowledgment */
	case NMT_MESSAGE_10d: /* call ack on alternate type */
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		if (frame->mt == NMT_MESSAGE_11b) {
			trans->priority = 1;
			LOGP_CHAN(DNMT, LOGL_INFO, "Call acknowledgment is from a priority MS (PMS).\n");
			/* NMT Doc 450-1 §4.4.1.7: after ringing+answer, the PMS continues as
			 * Scheme A (MO call) -- state 11a = authentication request (5a(L=3/11)),
			 * states 14-16 = PMS dials its own outgoing call (14a/14b digit pairs).
			 * This single-node emulator routes the PMS through the normal MT call
			 * path instead: priority queueing (2c) and identity (10b) are correct,
			 * but the post-answer MO-dialing path (the PMS's own outgoing call) is
			 * NOT implemented -- there is no mechanism to set up a second outgoing
			 * call from the answered-PMS context. STANDARD POINT: §4.4.1.7 states
			 * 11a/13a/14-16 are not reached. */
		}
		LOGP_CHAN(DNMT, LOGL_INFO, "Received call acknowledgment from subscriber %c,%s.\n", trans->subscriber.country, trans->subscriber.number);
		if (trans->sms_string[0])
			trans->dms_call = 1;
		osmo_timer_del(&trans->timer);
		nmt_new_state(nmt, STATE_MT_CHANNEL);
		trans->nmt = nmt;
		nmt->tx_frame_count = 0;
		/* release other channels */
		for (sender = sender_head; sender; sender = sender->next) {
			other = (nmt_t *)sender;
			if (other == nmt)
				continue;
			if (other->trans == trans)
				nmt_go_idle(other);
		}
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

/* Queued MT call: send queue-info (2c priority / 2f ordinary) on the calling channel.
 * Each frame we re-check for a free TC; as soon as one is available, go to the normal
 * channel-assignment path (STATE_MT_CHANNEL -> 2b). NMT Doc 450-1 4.4.1.6/4.4.1.7:
 * "2b is sent instead of 2c" once an idle TC is available.
 */
static void tx_mt_channel(nmt_t *nmt, frame_t *frame);

static void tx_mt_queue(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	/* a TC became free -> stop queueing, proceed to assignment */
	if (search_free_tc(nmt)) {
		osmo_timer_del(&nmt->timer);
		if (trans->priority_mo) {
			/* PMS MO call (4.4.1.7, states 4-5): when a TC becomes free the
			 * standard requires the MTX to send 2a again on the CC to confirm
			 * the PMS is still waiting, and wait for a T''' 11b acknowledgement
			 * BEFORE sending the 2b TC allocation (state 7). A direct jump to
			 * 2b (the normal path) would skip this re-confirmation handshake.
			 * nmt_page re-enters STATE_MT_PAGING and schedules the paging timer;
			 * rx_mt_paging handles the 11b ack and transitions to STATE_MT_CHANNEL
			 * for the 2b TC allocation as usual. */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "PMS: TC became free; re-paging MS before TC allocation (4.4.1.7 state 4).\n");
			nmt_page(trans, 1);
			tx_mt_paging(nmt, frame);	/* fill current frame with 2a */
			return;
		}
		LOGP_CHAN(DNMT, LOGL_INFO, "TC became free, assigning queued call.\n");
		nmt_new_state(nmt, STATE_MT_CHANNEL);
		nmt->tx_frame_count = 0;
		tx_mt_channel(nmt, frame);
		return;
	}

	/* otherwise keep telling the MS it is queued (2c priority, 2f ordinary) */
	/* Fictitious channel numbers for H8H9H10 in frames 2c, 2f, 2d.
	 *
	 * NMT Doc 450-1 §4.3.3.11 (p.51): raw 12-bit H8H9H10 values placed directly
	 * in additional_info -- these are NOT channel numbers in the BCD sense:
	 *   3F0 -> queuing info to ordinary MS (frame 2f)
	 *   3F1 -> queuing info to PMS (frame 2c)
	 *   3F2 -> TC scanning order (frame 2d)
	 *
	 * NMT Doc 900-1 §4.3.3.12 (p.51): same wire values expressed as fictitious
	 * channel numbers ch.1008/1009/1010. These must pass through nmt_encode_channel()
	 * (interleaved-channel encoding) to get the correct 12-bit H8H9H10 bit pattern:
	 *   ch.1008 -> nmt_encode_channel(900, 1008, 0) = 0x9F0 (ordinary 2f)
	 *   ch.1009 -> nmt_encode_channel(900, 1009, 0) = 0x9F1 (PMS 2c)
	 *   ch.1010 -> nmt_encode_channel(900, 1010, 0) = 0x9F2 (TC scanning 2d)
	 *
	 * NMT 450 ch.1008 is outside the valid BCD channel range (1-400), so the
	 * two specs cannot be unified through nmt_encode_channel(): system-specific
	 * raw constants are used instead.
	 */
#define NMT_QUEUE_H8H9H10_ORDINARY_450  0x3F0	/* NMT 450 §4.3.3.11: ordinary MS (2f) */
#define NMT_QUEUE_H8H9H10_PRIORITY_450  0x3F1	/* NMT 450 §4.3.3.11: PMS (2c) */
#define NMT_QUEUE_H8H9H10_SCANNING_450  0x3F2	/* NMT 450 §4.3.3.11: TC scanning (2d) */
#define NMT_QUEUE_H8H9H10_ORDINARY_900  0x9F0	/* NMT 900 §4.3.3.12: ch.1008 encoded */
#define NMT_QUEUE_H8H9H10_PRIORITY_900  0x9F1	/* NMT 900 §4.3.3.12: ch.1009 encoded */
#define NMT_QUEUE_H8H9H10_SCANNING_900  0x9F2	/* NMT 900 §4.3.3.12: ch.1010 encoded */

	frame->mt = trans->priority ? NMT_MESSAGE_2c : NMT_MESSAGE_2f;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	if (nmt->sysinfo.system == 450)
		frame->additional_info = trans->priority ? NMT_QUEUE_H8H9H10_PRIORITY_450 : NMT_QUEUE_H8H9H10_ORDINARY_450;
	else
		frame->additional_info = trans->priority ? NMT_QUEUE_H8H9H10_PRIORITY_900 : NMT_QUEUE_H8H9H10_ORDINARY_900;
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Send queue info to mobile (%s).\n", trans->priority ? "2c priority" : "2f ordinary");
}

static void timeout_mt_queue(nmt_t *nmt)
{
	transaction_t *trans = nmt->trans;

	LOGP_CHAN(DNMT, LOGL_NOTICE, "Queue timeout, no TC became free; releasing call.\n");
	LOGP(DNMT, LOGL_INFO, "Release call towards network.\n");
	if (trans->callref) {
		call_up_release(trans->callref, CAUSE_NOCHANNEL);
		trans->callref = 0;
	}
	nmt_release(nmt);
}

static void tx_mt_channel(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;
	nmt_t *tc;

	/* get free channel (after releasing all channels) */
	tc = search_free_tc(nmt);
	if (!tc) {
		/* No free TC: queue the call on the calling channel instead of dropping it. */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "No free TC, queueing call (%s).\n", trans->priority ? "priority" : "ordinary");
		nmt_new_state(nmt, STATE_MT_QUEUE);
		nmt->tx_frame_count = 0;
		osmo_timer_schedule(&nmt->timer, QUEUE_TO);
		/* send the queue-info frame now */
		tx_mt_queue(nmt, frame);
		return;
	}
	if (nmt != tc) {
		/* link trans and tc together, so we can continue with channel assignment */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Switching to TC channel #%s.\n", tc->sender.kanal);
		nmt_go_idle(nmt);
		tc->trans = trans;
		trans->nmt = tc;
	} else
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Staying on CC/TC channel #%s.\n", tc->sender.kanal);
	nmt_new_state(tc, STATE_MT_IDENT);
	tc->tx_frame_count = 0;

	/* assign channel on 'nmt' to 'tc' */
	frame->mt = NMT_MESSAGE_2b;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	frame->tc_no = nmt_encode_tc(tc->sysinfo.system, atoi(tc->sender.kanal), tc->sysinfo.ms_power);
	LOGP_CHAN(DNMT, LOGL_INFO, "Send channel activation to mobile.\n");
}

static void tx_mt_ident(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Sending identity request.\n");
	tx_ident(nmt, frame);
	if (nmt->tx_frame_count == 8) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout waiting for identity reply\n");
		if (trans->priority_mo) {
			LOGP_CHAN(DNMT, LOGL_INFO, "PMS priority MO call: no identity reply, releasing.\n");
		} else {
			LOGP_CHAN(DNMT, LOGL_INFO, "Release call towards network.\n");
			call_up_release(trans->callref, CAUSE_TEMPFAIL);
		}
		destroy_transaction(trans);
	}
}

static void rx_mt_ident(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_10b: /* seizure / identity */
	case NMT_MESSAGE_10c: /* seizure / identity after blocking-path TC scan (§4.4.1.2.2 Scheme B1, §4.4.1.14 Scheme B4) */
		if (!match_subscriber(trans, frame))
			break;
		nmt_value2digits(frame->ms_password, trans->subscriber.password, 3);
		LOGP_CHAN(DNMT, LOGL_INFO, "Received identity (password %s).\n", trans->subscriber.password);
		if (trans->dms_call || trans->mwi_call) {
			/* DMS (SMS) or MWI call - use autoanswer path */
			nmt_new_state(nmt, STATE_MT_AUTOANSWER);
			nmt->wait_autoanswer = 1;
			nmt->tx_frame_count = 0;
		} else if (trans->priority_mo) {
			/* PMS MO call (4.4.1.7): MS is calling OUT -- no incoming callref.
			 * Send ringing L(9) ×4 (state 11 in the diagram) so the MS knows
			 * the TC is ready and can lift the handset. No call_up_alerting(). */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "PMS priority MO call: identity received, sending ringing order (L=9) to notify MS the TC is ready.\n");
			nmt_new_state(nmt, STATE_MT_RINGING);
			nmt->tx_frame_count = 0;
			nmt->tx_callerid_count = 0;
			osmo_timer_schedule(&nmt->timer, RINGING_TO);
			/* do NOT call call_up_alerting() -- there is no incoming call */
		} else {
			nmt_new_state(nmt, STATE_MT_RINGING);
			/* start with caller ID before ringing */
			if (nmt->send_callerid) {
				nmt->tx_frame_count = 4;
				nmt->tx_callerid_count = 1;
			} else {
				nmt->tx_frame_count = 0;
				nmt->tx_callerid_count = 0;
			}
			osmo_timer_schedule(&nmt->timer, RINGING_TO);
			call_up_alerting(trans->callref);
		}
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

static void tx_mt_autoanswer(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	/* first we need to wait for autoanswer */
	if (nmt->wait_autoanswer) {
		frame->mt = NMT_MESSAGE_6;
		return;
	}
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Send 'autoanswer order'.\n");
	set_line_signal(nmt, frame, 12);
	if (nmt->tx_frame_count == 4) {
		LOGP_CHAN(DNMT, LOGL_INFO, "No reaction to autoanswer, proceed with ringing.\n");
		nmt_new_state(nmt, STATE_MT_RINGING);
		nmt->tx_frame_count = 0;
		nmt->tx_callerid_count = 0;
		osmo_timer_schedule(&nmt->timer, RINGING_TO);
		call_up_alerting(trans->callref);
	}
}

static void rx_mt_autoanswer(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_15: /* idle */
		nmt->wait_autoanswer = 0;
		break;
	case NMT_MESSAGE_13a: /* line signal */
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		if (nmt_line_signal_majority(frame->line_signal, 5, 3) != 12)
			break;
		LOGP_CHAN(DNMT, LOGL_INFO, "Received acknowledge to autoanswer (line signal L(12) '%s').\n",
			nmt_linesignal_name(12, 1));
		nmt_new_state(nmt, STATE_MT_COMPLETE);
		nmt->tx_frame_count = 0;
		call_up_answer(trans->callref, &trans->subscriber.country);
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

static void tx_mt_ringing(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	set_line_signal(nmt, frame, 9);
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Send 'ringing order'.\n");
	if (nmt->tx_frame_count >= 4) {
		if (nmt->tx_callerid_count) {
			if (nmt->tx_frame_count == 5)
				LOGP_CHAN(DNMT, LOGL_INFO, "Send 'A-number'.\n");
			nmt_encode_a_number(frame, nmt->tx_frame_count - 4, trans->caller_type, trans->caller_id, nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power, nmt->sysinfo.traffic_area);
		} else
			frame->mt = NMT_MESSAGE_6;
	}
	if (nmt->tx_callerid_count == 1) {
		/* start ringing after first caller ID of 6 frames */
		if (nmt->tx_frame_count == 10) {
			nmt->tx_frame_count = 0;
			nmt->tx_callerid_count++;
		}
	} else {
		/* repeat ringing after 5 seconds */
		if (nmt->tx_frame_count == 36) {
			nmt->tx_frame_count = 0;
		}
	}
}

static void rx_mt_ringing(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_13a: /* line signal */
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		if (nmt_line_signal_majority(frame->line_signal, 5, 3) != 14)
			break;
		LOGP_CHAN(DNMT, LOGL_INFO, "Received 'answer' from phone (line signal L(14) '%s').\n",
			nmt_linesignal_name(14, 0));
		osmo_timer_del(&nmt->timer);
		if (trans->priority_mo) {
			/* PMS MO call (4.4.1.7, state 14 without SIS): MS has lifted the
			 * handset and is ready to dial. Send L(3) proceed (tx_mo_confirm,
			 * 2 frames) then enter STATE_MO_DIALING to collect 14a/14b digits
			 * from the MS. call_up_setup() follows once dialing is complete. */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "PMS priority MO call: MS answered, entering MO dialing mode (proceed L(3) -> digit collection).\n");
			nmt_new_state(nmt, STATE_MO_CONFIRM);
			nmt->tx_frame_count = 0;
			/* no call_up_answer() -- there is no incoming call to answer */
		} else {
			nmt_new_state(nmt, STATE_MT_COMPLETE);
			nmt->tx_frame_count = 0;
			call_up_answer(trans->callref, &trans->subscriber.country);
		}
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

/* Send MWI frame 5c with message waiting indicators
 *
 * Frame 5c format: NNNPYYZXXXXXXFFF
 *   NNN = channel number
 *   P = 2 (prefix for frame 5c)
 *   YY = traffic area
 *   Z = subscriber country
 *   XXXXXX = subscriber number
 *   FFF = waiting indicators (12 bits, duplicated for error checking)
 *
 * FFF field (12 bits): i1 i2 i3 i4 | i5 i6 i'1 i'2 | i'3 i'4 i'5 i'6
 *
 * Bit positions (MSB first as per NMT spec):
 *   bit 11 = i1  (SMS)      bit 5 = i'1
 *   bit 10 = i2  (voice)    bit 4 = i'2
 *   bit  9 = i3  (fax)      bit 3 = i'3
 *   bit  8 = i4  (email)    bit 2 = i'4
 *   bit  7 = i5  (data)     bit 1 = i'5
 *   bit  6 = i6  (spare)    bit 0 = i'6
 */
static void set_mwi_frame(nmt_t *nmt, frame_t *frame, uint8_t flags)
{
	transaction_t *trans = nmt->trans;
	uint16_t info = 0;

	frame->mt = NMT_MESSAGE_5c;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);

	/* Encode flags into F1F2F3 field per NMT Doc 450-1 §4.3.3.11 (p.51):
	 *   F1F2F3 = i1 i2 i3 i4  i5 i6  i'1 i'2  i'3 i'4 i'5 i'6
	 *   bit:     11 10  9  8    7  6    5   4    3   2   1   0
	 * Each indicator is set in both the primary (in) and redundant (i'n) copy.
	 * IMPORTANT: "in = 0 carries no information on the corresponding indicator
	 * setting (does not reset the indicator)" -- only in=1 has meaning.
	 * There is no mechanism to reset an indicator via frame 5c alone. */
	if (flags & 0x01) info |= 0x820;  /* SMS:   i1(bit 11) + i'1(bit 5) */
	if (flags & 0x02) info |= 0x410;  /* voice: i2(bit 10) + i'2(bit 4) */
	if (flags & 0x04) info |= 0x208;  /* fax:   i3(bit 9)  + i'3(bit 3) */
	if (flags & 0x08) info |= 0x104;  /* email: i4(bit 8)  + i'4(bit 2) */
	if (flags & 0x10) info |= 0x082;  /* data:  i5(bit 7)  + i'5(bit 1) */

	frame->waiting_info = info;

	LOGP_CHAN(DNMT, LOGL_INFO, "Send MWI frame 5c: flags=0x%02x info=0x%03x (%s%s%s%s%s)\n",
		flags, info,
		(flags & 0x01) ? "sms " : "",
		(flags & 0x02) ? "voice " : "",
		(flags & 0x04) ? "fax " : "",
		(flags & 0x08) ? "email " : "",
		(flags & 0x10) ? "data " : "");
}

static void tx_mt_complete(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	++nmt->tx_frame_count;
	if (nmt->compandor && !trans->dms_call && !trans->mwi_call) {
		if (nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Send 'compandor in'.\n");
		set_line_signal(nmt, frame, 5);
	} else
		frame->mt = NMT_MESSAGE_6;
	/* §4.4.1.2 Scheme B (p.59) shows 4 frames of compander-in / idle before
	 * throughconnecting (state 14 → 14b). Transition on frame 4. */
	if (nmt->tx_frame_count == 4) {
		nmt_new_state(nmt, STATE_ACTIVE);
		if (trans->mwi_call) {
			/* Out-of-call MWI delivery per NMT spec 4.4.1.16:
			 * 5c x2 (MWI data), then L(15) x4 via nmt_release()
			 */
			LOGP_CHAN(DNMT, LOGL_INFO, "Starting MWI delivery (frame 5c).\n");
			trans->mwi_pending = 0;  /* Clear so release doesn't restart */
			nmt->tx_frame_count = 0;
			nmt->active_state = ACTIVE_STATE_MWI;
			/* Don't switch to audio mode - keep transmitting frames */
		} else if (trans->dms_call) {
			/* SMS delivery: handled by DMS layer */
			LOGP_CHAN(DNMT, LOGL_INFO, "Connect audio.\n");
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			time_t ti = time(NULL);
			sms_deliver(nmt, sms_ref, trans->caller_id, trans->caller_type, SMS_PLAN_ISDN_TEL, ti, 1, trans->sms_string, trans->sms_len);
		} else {
			/* Normal call: switch to audio mode */
			LOGP_CHAN(DNMT, LOGL_INFO, "Connect audio.\n");
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			if (nmt->supervisory) {
				super_reset(nmt);
				osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO1,0);
			}
		}
	}
}

static void timeout_mt_ringing(nmt_t *nmt)
{
	transaction_t *trans = nmt->trans;

	LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout while waiting for answer of the phone.\n");
	if (trans->priority_mo) {
		/* PMS MO call: MS didn't lift the handset -- just release the TC. */
		LOGP_CHAN(DNMT, LOGL_INFO, "PMS priority MO call: no answer (handset not lifted), releasing.\n");
	} else {
		LOGP(DNMT, LOGL_INFO, "Release call towards network.\n");
		call_up_release(trans->callref, CAUSE_NOANSWER);
		trans->callref = 0;
	}
	nmt_release(nmt);
}

/*
 * handle clearing towards MTX
 */

static void tx_mo_release(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	set_line_signal(nmt, frame, 15);
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Send release.\n");
	if (nmt->tx_frame_count == 4)
		destroy_transaction(trans); /* continue with this frame, then go idle */
}

/*
 * handle clearing towards MS
 */

static void tx_mt_release(nmt_t *nmt, frame_t *frame)
{
	/* L(15) = "clearing, call transfer not activated" per NMT Doc 450-1 para
	 * 4.3.3.3 -- the plain release. L(13) is the "call transfer activated"
	 * variant, used only with the call-transfer supplementary service (not
	 * implemented), so the diagrams' "L(13/15)" reduces to L(15) here. */
	set_line_signal(nmt, frame, 15);
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Send release.\n");
}

static void rx_mt_release(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_13a: /* line signal */
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		if (nmt_line_signal_majority(frame->line_signal, 5, 3) != 1)
			break;
		LOGP_CHAN(DNMT, LOGL_INFO, "Received release guard (line signal L(1) '%s').\n",
			nmt_linesignal_name(1, 0));
		if (trans->mwi_call) {
			/* MWI delivery confirmed - phone acknowledged clearing
			 * Note: This confirms phone received clearing signal,
			 * but doesn't confirm MWI was processed (phone may not support MWI)
			 */
			LOGP_CHAN(DNMT, LOGL_INFO, "MWI out-of-call delivery confirmed (release guard received).\n");
			printf("MWI delivery confirmed!\n");
		}
		osmo_timer_del(&nmt->timer);
		destroy_transaction(trans);
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

static void timeout_mt_release(nmt_t *nmt)
{
	transaction_t *trans = nmt->trans;

	if (trans->mwi_call) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "MWI: Timeout waiting for release guard - delivery status unknown.\n");
		printf("MWI delivery timeout (no ack)!\n");
	} else {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout while releasing.\n");
	}
	destroy_transaction(trans);
}

/*
 * SCIP — Switching Call In Progress (move active call to another TC)
 *
 * Operator-triggered intra-cell channel change, modelled on the AMPS handoff path
 * (src/amps/amps.c amps_handoff) but using the NMT frames:
 *   - frame 3a "Traffic channel allocation on traffic channel" (prefix 5), carrying the
 *     new TC number in NaNbNc, sent on the CURRENT TC (NMT Doc 450-1 4.3.2 msg 3a);
 *   - the MS stops TX, retunes to the new TC and re-identifies with frame 10b there
 *     (NMT Doc 450-3 5.4.5 "Switching call in progress"). On the new TC the base station
 *     is already radiating idle frames + supervisory, so the MS finds the channel live.
 * On timeout (MS did not re-appear) we fall back to the original TC and resume the call,
 * per 5.4.5 ("if the change of channel fails ... the MS shall go back to the previous TC").
 */

#define SCIP_ORDER_FRAMES	4	/* repeat frame 3c this many times before the move (min 4 per 5.4.5) */

static void tx_scip_order(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;
	nmt_t *tc = nmt->scip_target;

	/* Build frame 3c (channel change on traffic channel, short procedure --
	 * NMT Doc 450-1 Scheme C2', para 4.4.1.4.3 / 4.7.2.4b).  The short
	 * procedure combines "change to the new TC" and "send identity there"
	 * into one order, so the MS re-identifies with 10b on the new TC without
	 * a separate 3b request -- which is exactly the flow STATE_SCIP_IDENT
	 * implements.  Per 4.7.2.4b the channel-number field N1'N2'N3' must equal
	 * the ordered TC NaNbNc, so both carry the target channel. */
	frame->mt = NMT_MESSAGE_3c;
	frame->channel_no = nmt_encode_channel(tc->sysinfo.system, atoi(tc->sender.kanal), tc->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	frame->tc_no = nmt_encode_tc(tc->sysinfo.system, atoi(tc->sender.kanal), tc->sysinfo.ms_power);

	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "SCIP: sending channel change order (3c, short procedure) -> TC #%s.\n", tc->sender.kanal);

	if (nmt->tx_frame_count >= SCIP_ORDER_FRAMES) {
		/* Order delivered (step 6).  Per Scheme C2' the OLD TC keeps the speech
		 * path and stays supervised until the MS has re-identified on the new TC
		 * (step 8); only then is the old TC released (step 11), and on failure
		 * (step 9) the call simply stays on the old TC.  So the old TC KEEPS the
		 * transaction and returns to the active/supervised state; the TARGET is
		 * armed only to RECEIVE the identity (10b) there, carrying a copy of the
		 * expected subscriber for matching.  The transaction is handed over only
		 * once the MS actually appears on the new TC (rx_scip_ident). */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: order sent, awaiting identity on new TC #%s (old TC #%s keeps speech until re-identified).\n", tc->sender.kanal, nmt->sender.kanal);
		nmt->scip_target = tc;			/* remember target so we can give up on failure */
		/* arm the target to receive the identity, pointing back to the origin */
		tc->scip_source = nmt;
		tc->scip_country = trans->subscriber.country;	/* who to expect on the new TC */
		memcpy(tc->scip_number, trans->subscriber.number, sizeof(tc->scip_number));
		nmt_new_state(tc, STATE_SCIP_IDENT);
		tc->tx_frame_count = 0;
		tc->rx_frame_count = 0;
		nmt_set_dsp_mode(tc, DSP_MODE_FRAME);
		osmo_timer_schedule(&tc->timer, SCIP_TO);
		/* old TC: back to the supervised active state, still holding the call */
		nmt_new_state(nmt, STATE_ACTIVE);
		nmt->active_state = ACTIVE_STATE_VOICE;
		nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
		if (nmt->supervisory) {
			super_reset(nmt);
			osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO1, 0);
		}
	}
}

/* On the new TC: in STATE_SCIP_IDENT we radiate idle (free) frames while the
 * ø-signal is up (DSP mixes it in for this state); in STATE_SCIP_IDENT_REQ we
 * send the identity request 3b (Scheme C2' step 9). */
static void tx_scip_ident(nmt_t *nmt, frame_t *frame)
{
	if (nmt->state == STATE_SCIP_IDENT_REQ) {
		/* identity request on the new TC, addressed to the expected subscriber */
		frame->mt = NMT_MESSAGE_3b;
		frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
		frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
		frame->ms_country = nmt_digits2value(&nmt->scip_country, 1);
		frame->ms_number = nmt_digits2value(nmt->scip_number, 6);
		if (++nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "SCIP: no identity within T', re-requesting identity (3b) on new TC.\n");
		return;
	}
	tx_ident(nmt, frame);
}

/* Hand the live call over from the old TC to this (new) TC and throughconnect
 * (Scheme C2' step 8); then release the old TC (step 11, stop ø-signal). */
static void scip_complete_move(nmt_t *nmt)
{
	nmt_t *old = nmt->scip_source;
	transaction_t *trans = old ? old->trans : NULL;

	if (!old || !trans) {
		LOGP_CHAN(DNMT, LOGL_ERROR, "SCIP: lost origin channel on completion, releasing.\n");
		if (nmt->scip_source)
			nmt->scip_source = NULL;
		return;
	}

	/* move the transaction onto the new TC */
	old->trans = NULL;
	old->scip_target = NULL;
	nmt->trans = trans;
	trans->nmt = nmt;
	nmt->scip_source = NULL;
	memcpy(trans->subscriber.password, nmt->scip_password, sizeof(trans->subscriber.password));

	/* old TC: stop ø-signal, back to idle (step 11) */
	osmo_timer_del(&old->timer);
	nmt_go_idle(old);

	/* new TC: throughconnect speech + supervision */
	osmo_timer_del(&nmt->timer);
	nmt_new_state(nmt, STATE_ACTIVE);
	nmt->active_state = ACTIVE_STATE_VOICE;
	nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
	if (nmt->supervisory) {
		super_reset(nmt);
		osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO1, 0);
	}
	LOGP_CHAN(DNMT, LOGL_INFO, "SCIP: MS re-identified on new TC, speech throughconnected; old TC released.\n");
}

static void rx_scip_ident(nmt_t *nmt, frame_t *frame)
{
	switch (frame->mt) {
	case NMT_MESSAGE_10b: /* seizure / identity on the new TC */
		if (!match_channel(nmt, frame))
			break;
		if (nmt_digits2value(&nmt->scip_country, 1) != frame->ms_country
		 || nmt_digits2value(nmt->scip_number, 6) != frame->ms_number) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: identity on new TC does not match expected subscriber, ignoring.\n");
			break;
		}
		/* capture the password echoed in the identity for completeness */
		nmt_value2digits(frame->ms_password, nmt->scip_password, 3);
		scip_complete_move(nmt);
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

static void timeout_scip_ident(nmt_t *nmt)
{
	nmt_t *old = nmt->scip_source;

	if (nmt->state == STATE_SCIP_IDENT) {
		/* Step 9: no identity within T' after the 3c order -> request identity
		 * (3b) on the new TC and wait another T'. */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: no identity within T' on new TC, requesting identity (3b).\n");
		nmt_new_state(nmt, STATE_SCIP_IDENT_REQ);
		nmt->tx_frame_count = 0;
		nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
		osmo_timer_schedule(&nmt->timer, SCIP_TO);
		return;
	}

	/* STATE_SCIP_IDENT_REQ: identity still not received after the 3b request.
	 * Per Scheme C2' the MTX throughconnects speech on the PREVIOUS TC (the call
	 * was never torn down there) and sends a forced release on the NEW TC. The
	 * call survives on the old channel. */
	LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: MS did not re-identify on new TC; reverting call to old TC #%s, forced-releasing new TC.\n",
		old ? old->sender.kanal : "?");

	/* give up the target */
	nmt->scip_country = 0;
	nmt->scip_number[0] = '\0';
	nmt->scip_source = NULL;
	osmo_timer_del(&nmt->timer);
	nmt_go_idle(nmt);			/* forced release on the new TC */

	/* the old TC still owns the transaction and its ø-signal has been up the
	 * whole time (step: supervisory controls the previous TC) -- just clear its
	 * pointer to the abandoned target so it is a plain active call again. */
	if (old)
		old->scip_target = NULL;
}

/*
 * handle call
 */

/* True while the MFT converter is being set up, active, or torn down, OR while
 * a register-recall digit session is in progress. During these windows the MS is
 * sending digital frames instead of maintaining the ø-signal, so the supervision
 * alarm must be locally suppressed (NMT Doc 450-1 4.4.1.9/4.4.1.10). */
static int nmt_in_mft(nmt_t *nmt)
{
	return nmt->active_state == ACTIVE_STATE_MFT_IN
	    || nmt->active_state == ACTIVE_STATE_MFT
	    || nmt->active_state == ACTIVE_STATE_MFT_OUT
	    || nmt->active_state == ACTIVE_STATE_RECALL_PROCEED
	    || nmt->active_state == ACTIVE_STATE_RECALL_DIGITS;
}

void nmt_rx_super(nmt_t *nmt, int tone, double quality)
{
	if (tone)
		LOGP_CHAN(DNMT, LOGL_INFO, "Detected supervisory signal with quality=%.0f.\n", quality * 100.0);
	else
		LOGP_CHAN(DNMT, LOGL_INFO, "Lost supervisory signal\n");

	if (nmt->sender.loopback)
		return;

	/* only detect supervisory signal during active call */
	if (nmt->state != STATE_ACTIVE || !nmt->supervisory)
		return;

	/* ø-signal alarm suppressed during the MFT converter window (4.4.1.9): the
	 * MS sends DTMF, not the ø-signal, so a "lost" tone here is expected and must
	 * not arm the release timeout. */
	if (nmt_in_mft(nmt)) {
		osmo_timer_del(&nmt->timer);
		return;
	}

	if (tone)
		osmo_timer_del(&nmt->timer);
	else
		osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO2);
}

static void timeout_active(nmt_t *nmt, int duration)
{
	transaction_t *trans = nmt->trans;

	/* Register recall inactivity timeout (4.4.1.10, state 9): the MS stopped
	 * sending digits (or never started). Log the collected code, cancel the
	 * ø-signal alarm suppression, and return to voice. */
	if (nmt->active_state == ACTIVE_STATE_RECALL_DIGITS) {
		if (trans->recall_code_len > 0) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Register recall complete: collected code '%.*s' (%d digit%s). STANDARD POINT REACHED (4.4.1.10 state 9) -- no MTX register backend to process this code; returning to voice.\n",
				trans->recall_code_len, trans->recall_code, trans->recall_code_len,
				trans->recall_code_len == 1 ? "" : "s");
		} else {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Register recall timeout: no digits received from MS; returning to voice.\n");
		}
		/* cancel ø-signal suppression (local equivalent of frame 20 A=10, 4.4.1.10 state 10) */
		if (nmt->supervisory) {
			super_reset(nmt);
			osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO1, 0);
			LOGP_CHAN(DNMT, LOGL_INFO, "Resuming supervisory alarm after register recall (local equivalent of frame 20 A=10 to BS).\n");
		}
		nmt->active_state = ACTIVE_STATE_VOICE;
		nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
		return;
	}

	if (duration == SUPERVISORY_TO1)
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout after %d seconds not receiving supervisory signal.\n", duration);
	else
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Timeout after %d seconds loosing supervisory signal.\n", duration);
	LOGP_CHAN(DNMT, LOGL_INFO, "Release call towards network.\n");
	call_up_release(trans->callref, CAUSE_TEMPFAIL);
	trans->callref = 0;
	nmt_release(nmt);
}

static void rx_active(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;
	char digit;

	/* restart timer on every reception of frame (except during MFT, where the
	 * ø-signal alarm is suppressed per 4.4.1.9 -- see nmt_in_mft) */
	if (nmt->supervisory && !nmt_in_mft(nmt))
		osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO2);

	switch (frame->mt) {
	case NMT_MESSAGE_10b: /* identity with new power level (NMT Doc 450-1 4.4.1.8) */
		if (nmt->active_state != ACTIVE_STATE_POWER_CHANGE)
			break;
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		/* The MS re-identifies with 10b carrying the new power level in N1.
		 * Decode N1 to get the new power and update our sysinfo accordingly. */
		{
			int new_power = trans->pending_power;
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Received power-change ack (frame 10b): MS confirmed new power level %d (was %d).\n",
				new_power, nmt->sysinfo.ms_power);
			nmt->sysinfo.ms_power = new_power;
			trans->pending_power = -1;
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			if (nmt->supervisory)
				super_reset(nmt);
		}
		break;
	case NMT_MESSAGE_13b: /* coin-box answer acknowledgment with echoed tariff class */
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		/* The coin-box MS echoes the tariff class it received from the MTX (in frame 5b),
		 * for control purposes (NMT Doc 450-1 4.2.1 / 4.3.2, frame 13b). We do not drive a
		 * charging back-end, so we only decode and log what the phone reports. */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received coin-box answer acknowledgment (frame 13b): line signal L(%d), tariff class 0x%x.\n",
			frame->line_signal & 0xf, frame->tariff_class);
		break;
	case NMT_MESSAGE_13a: /* line signal */
	    {
		int ls;
		if (!match_channel(nmt, frame))
			break;
		if (!match_subscriber(trans, frame))
			break;
		ls = nmt_line_signal_majority(frame->line_signal, 5, 3);
		if (ls < 0) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Line signal not accepted: no majority of equal, meaningful characters (corrupt frame).\n");
			break;
		}
		switch (ls) {
		case 5:
			/* Register recall (NMT Doc 450-1 4.4.1.10 / 4.7.2.11): the MS requests
			 * that the MTX connect a register to the line for code sending.
			 * The MTX must acknowledge with L(3) "proceed to send unencrypted
			 * digits" (state 6 in the diagram), preceded by idle frames and the
			 * local equivalent of the ø-signal alarm suppression (frame 20 A=4/A=7
			 * to BS; no external BS, so suppressed locally via nmt_in_mft).
			 * Digit collection (14a/14b) follows in ACTIVE_STATE_RECALL_DIGITS. */
			if (nmt->active_state != ACTIVE_STATE_VOICE) {
				LOGP_CHAN(DNMT, LOGL_NOTICE, "Register recall request (L(5) '%s') received outside voice state (%s); ignoring.\n",
					nmt_linesignal_name(5, 0), nmt_active_state_name(nmt->active_state));
				break;
			}
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Register recall requested (L(5) '%s'): entering proceed + digit-collection mode (4.4.1.10).\n",
				nmt_linesignal_name(5, 0));
			/* suppress ø-signal alarm (same as MFT: delete timer, kept suppressed
			 * by nmt_in_mft while in recall states) */
			if (nmt->supervisory) {
				osmo_timer_del(&nmt->timer);
				LOGP_CHAN(DNMT, LOGL_INFO, "Suppressing supervisory alarm during register recall (local equivalent of frame 20 A=7 to BS).\n");
			}
			/* clear digit buffer */
			memset(trans->recall_code, 0, sizeof(trans->recall_code));
			trans->recall_code_len = 0;
			trans->recall_mft_num = 0;
			nmt->active_state = ACTIVE_STATE_RECALL_PROCEED;
			nmt->tx_frame_count = 0;
			nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
			break;
		case 8:
			if (nmt->active_state != ACTIVE_STATE_VOICE)
				break;
			LOGP_CHAN(DNMT, LOGL_INFO, "Received 'MFT in' request.\n");
			/* Suppress the ø-signal (supervisory) alarm for the MFT window
			 * (NMT Doc 450-1 4.4.1.9 states 4-5: frame 20 A=4/A=7 order to BS).
			 * No external BS in this combined node, so apply it locally: cancel
			 * the pending supervisory timeout so DTMF (not ø-signal) does not drop
			 * the call. nmt_in_mft() keeps it suppressed until MFT-out. */
			if (nmt->supervisory) {
				osmo_timer_del(&nmt->timer);
				LOGP_CHAN(DNMT, LOGL_INFO, "Suppressing supervisory alarm during MFT (local equivalent of frame 20 A=4/A=7 to BS).\n");
			}
			nmt->active_state = ACTIVE_STATE_MFT_IN;
			nmt->tx_frame_count = 0;
			nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
			nmt->mft_num = 0;
			break;
		case 7:
			if (nmt->active_state != ACTIVE_STATE_MFT)
				break;
			LOGP_CHAN(DNMT, LOGL_INFO, "Received 'MFT out' request.\n");
			nmt->active_state = ACTIVE_STATE_MFT_OUT;
			nmt->tx_frame_count = 0;
			nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
			break;
		}
		break;
	    }
	case NMT_MESSAGE_14a: /* digits */
		if (!match_channel(nmt, frame))
			break;
		if (!match_area(nmt, frame))
			break;
		/* register recall digit collection (4.4.1.10): 14a carries odd-position
		 * digits; collect into trans->recall_code and restart the inactivity timer. */
		if (nmt->active_state == ACTIVE_STATE_RECALL_DIGITS) {
			if (!(trans->recall_mft_num & 1)) {
				/* validate triple repetition */
				if (((frame->digit >> 8) & 0xf) != ((frame->digit >> 4) & 0xf)
				 || ((frame->digit >> 4) & 0xf) != (frame->digit & 0xf)) {
					LOGP_CHAN(DNMT, LOGL_NOTICE, "Register recall: digit repetition mismatch in 14a, ignoring corrupt frame.\n");
					break;
				}
				char d = nmt_value2digit(frame->digit);
				if (trans->recall_code_len < (int)sizeof(trans->recall_code) - 1)
					trans->recall_code[trans->recall_code_len++] = d;
				trans->recall_mft_num++;
				LOGP_CHAN(DNMT, LOGL_INFO, "Register recall digit (odd) received: '%c' (code so far: '%.*s').\n",
					d, trans->recall_code_len, trans->recall_code);
				/* restart inactivity timeout */
				osmo_timer_schedule(&nmt->timer, RECALL_TO);
			}
			break;
		}
		if (nmt->active_state != ACTIVE_STATE_MFT) {
			/* A keypress digit (frame 14a) arrived without, or before completing,
			 * the MFT-converter-in handshake (L(8) request -> L(4) ack, 4.4.1.9).
			 * Per spec it is only valid in MFT state; log the dropped keypress so
			 * it is visible rather than silently lost. */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Dropping digit frame 14a: received outside MFT converter state (%s); MFT-in handshake (L(8)->L(4)) not completed, keypress ignored.\n", nmt_active_state_name(nmt->active_state));
			break;
		}
		if ((nmt->mft_num & 1)) {
			/* 14a carries ODD-position digits; receiving one when an even digit was
			 * expected is a sequence error (lost/duplicated frame). Log it. */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Dropping digit frame 14a (odd): out of sequence, expected an even digit (mft_num=%d).\n", nmt->mft_num);
			break;
		}
		if ((frame->digit >> 12) != 0x00) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Position information of digit does not match, ignoring due to corrupt frame.\n");
			break;
		}
		if (((frame->digit >> 8) & 0xf) != ((frame->digit >> 4) & 0xf)
		 || ((frame->digit >> 4) & 0xf) != (frame->digit & 0xf)) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Digit repetition in frame does not match, ignoring due to corrupt frame.\n");
			break;
		}
		digit = nmt_value2digit(frame->digit);
		dtmf_encode_set_tone(&nmt->dtmf, digit, DTMF_DURATION, 0.0);
		LOGP_CHAN(DNMT, LOGL_INFO, "Received (odd)  digit %c.\n", digit);
		nmt->mft_num++;
		break;
	case NMT_MESSAGE_14b: /* digits */
		if (!match_channel(nmt, frame))
			break;
		if (!match_area(nmt, frame))
			break;
		/* register recall: 14b carries even-position digits */
		if (nmt->active_state == ACTIVE_STATE_RECALL_DIGITS) {
			if (trans->recall_mft_num & 1) {
				if (((frame->digit >> 8) & 0xf) != ((frame->digit >> 4) & 0xf)
				 || ((frame->digit >> 4) & 0xf) != (frame->digit & 0xf)) {
					LOGP_CHAN(DNMT, LOGL_NOTICE, "Register recall: digit repetition mismatch in 14b, ignoring corrupt frame.\n");
					break;
				}
				char d = nmt_value2digit(frame->digit);
				if (trans->recall_code_len < (int)sizeof(trans->recall_code) - 1)
					trans->recall_code[trans->recall_code_len++] = d;
				trans->recall_mft_num++;
				LOGP_CHAN(DNMT, LOGL_INFO, "Register recall digit (even) received: '%c' (code so far: '%.*s').\n",
					d, trans->recall_code_len, trans->recall_code);
				osmo_timer_schedule(&nmt->timer, RECALL_TO);
			}
			break;
		}
		if (nmt->active_state != ACTIVE_STATE_MFT) {
			/* Keypress digit (frame 14b) outside MFT state: see 14a above. */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Dropping digit frame 14b: received outside MFT converter state (%s); MFT-in handshake (L(8)->L(4)) not completed, keypress ignored.\n", nmt_active_state_name(nmt->active_state));
			break;
		}
		if (!(nmt->mft_num & 1)) {
			/* 14b carries EVEN-position digits; receiving one when an odd digit was
			 * expected is a sequence error (lost/duplicated frame). */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Dropping digit frame 14b (even): out of sequence, expected an odd digit (mft_num=%d).\n", nmt->mft_num);
			break;
		}
		if ((frame->digit >> 12) != 0xff) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Position information of digit does not match, ignoring due to corrupt frame.\n");
			break;
		}
		if (((frame->digit >> 8) & 0xf) != ((frame->digit >> 4) & 0xf)
		 || ((frame->digit >> 4) & 0xf) != (frame->digit & 0xf)) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Digit repetition in frame does not match, ignoring due to corrupt frame.\n");
			break;
		}
		digit = nmt_value2digit(frame->digit);
		dtmf_encode_set_tone(&nmt->dtmf, digit, DTMF_DURATION, 0.0);
		LOGP_CHAN(DNMT, LOGL_INFO, "Received (even) digit %c.\n", digit);
		nmt->mft_num++;
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame->mt), nmt_state_name(nmt->state));
	}
}

static void tx_active(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (nmt->active_state) {
	case ACTIVE_STATE_RECALL_PROCEED:
		/* Register recall proceed (NMT Doc 450-1 4.4.1.10, state 6): send L(3)
		 * "proceed to send unencrypted digits" 4 times, then switch to dialtone
		 * so the subscriber hears the register is ready. */
		set_line_signal(nmt, frame, 3);
		if (++nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Sending register recall proceed (L(3) '%s'): MS may now send digits.\n",
				nmt_linesignal_name(3, 1));
		if (nmt->tx_frame_count >= 4) {
			LOGP_CHAN(DNMT, LOGL_INFO, "Register recall: proceed sent; entering digit collection.\n");
			nmt->active_state = ACTIVE_STATE_RECALL_DIGITS;
			nmt->tx_frame_count = 0;
			nmt_set_dsp_mode(nmt, DSP_MODE_DIALTONE);
		}
		break;
	case ACTIVE_STATE_RECALL_DIGITS:
		/* Register recall digit collection (4.4.1.10, states 7-9a): send idle
		 * frames while the MS transmits 14a/14b digit pairs. The digit handler
		 * in rx_active collects them into trans->recall_code. */
		frame->mt = NMT_MESSAGE_6;
		break;
	case ACTIVE_STATE_POWER_CHANGE:
		/* MS output power change (NMT Doc 450-1 4.4.1.8): send frame 3a up to
		 * max T (8 frames) carrying the same TC channel number but with the
		 * new power level encoded in Na. The MS re-identifies with 10b carrying
		 * the new power level; that response is handled in rx_active. */
		if (nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Sending power change order (frame 3a, new power=%d).\n", trans->pending_power);
		frame->mt = NMT_MESSAGE_3a;
		frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
		frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
		frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
		frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
		/* Na = new TC with new power level */
		frame->tc_no = nmt_encode_tc(nmt->sysinfo.system, atoi(nmt->sender.kanal), trans->pending_power);
		if (++nmt->tx_frame_count >= 8) {
			/* T expired without 10b ack -- MS did not respond; stay on old power */
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Power change timeout: no 10b ack from MS, keeping power=%d.\n", nmt->sysinfo.ms_power);
			trans->pending_power = -1;
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			if (nmt->supervisory)
				super_reset(nmt);
		}
		break;
	case ACTIVE_STATE_COINBOX_ANSWER:
		/* Coin-box answer sequence (NMT Doc 450-1 4.4.1.6, state 13): send frame 5b
		 * carrying the operator-supplied tariff class 4 times, then return to
		 * the voice path. The coin-box MS echoes the tariff class in frame 13b
		 * (decoded/logged in rx_active). */
		set_coinbox_answer(nmt, frame, nmt->coinbox_tariff & 0x3);
		if (++nmt->tx_frame_count >= 4) {
			trans->coinbox_answer_pending = 0;
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			if (nmt->supervisory)
				super_reset(nmt);
		}
		break;
	case ACTIVE_STATE_MFT_IN:
		/* MFT converter in: acknowledge the MS request with L(4) exactly 4 times
		 * (NMT Doc 450-1 4.4.1.9 state 6), then switch the speech path to DTMF
		 * generation. The ø-signal supervision alarm was already suppressed on
		 * entry (L(8) handler), the local equivalent of the frame-20 A=4/A=7 order
		 * to the BS; it is kept suppressed by nmt_in_mft() and resumed at MFT-out. */
		set_line_signal(nmt, frame, 4);
		if (++nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Send 'MFT in acknowledge'.\n");
		if (nmt->tx_frame_count >= 4) {
			nmt->active_state = ACTIVE_STATE_MFT;
			nmt_set_dsp_mode(nmt, DSP_MODE_DTMF);
		}
		break;
	case ACTIVE_STATE_MFT_OUT:
		/* MFT converter out: acknowledge with L(10) (NMT Doc 450-1 4.4.1.9 state
		 * 10, "within T'"), then return to the voice path and resume supervision.
		 * (Same ack-count note as MFT_IN; T' timing is not enforced here.) */
		set_line_signal(nmt, frame, 10);
		if (++nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Send 'MFT out acknowledge'.\n");
		if (nmt->tx_frame_count >= 4) {
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			/* Cancel the ø-signal alarm suppression and resume supervision
			 * (NMT Doc 450-1 4.4.1.9 state 11: frame 20 A=10 to BS). nmt_in_mft()
			 * is now false, so re-establish the detector baseline and re-arm the
			 * loss timeout. */
			if (nmt->supervisory) {
				super_reset(nmt);
				osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO1, 0);
				LOGP_CHAN(DNMT, LOGL_INFO, "Resuming supervisory alarm after MFT (local equivalent of frame 20 A=10 to BS).\n");
			}
		}
		break;
	case ACTIVE_STATE_MWI:
		/* Send frame 5c with message waiting indicators
		 * Per NMT spec 4.4.1.16: 5c|5c (2 frames)
		 * For out-of-call: followed by normal release L(15) x4
		 * For in-call: return to voice, flags accumulated for call end
		 */
		set_mwi_frame(nmt, frame, trans->mwi_flags);
		++nmt->tx_frame_count;
		if (nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Sending MWI frame 5c.\n");
		if (nmt->tx_frame_count >= 2) {
			LOGP_CHAN(DNMT, LOGL_DEBUG, "MWI frame 5c sent 2 times.\n");
			if (trans->mwi_call) {
				/* Out-of-call MWI: release with normal L(15) clearing
				 * Per spec: 5c|5c|5a(L=15)|5a(L=15)|5a(L=15)|5a(L=15)
				 */
				LOGP_CHAN(DNMT, LOGL_INFO, "MWI delivery complete, releasing.\n");
				printf("MWI delivered!\n");
				nmt_release(nmt);
			} else {
				/* In-call MWI: return to voice, flags accumulated for call end */
				LOGP_CHAN(DNMT, LOGL_INFO, "MWI in-call complete, returning to voice.\n");
				printf("MWI sent (in-call)!\n");
				nmt->active_state = ACTIVE_STATE_VOICE;
				nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
				if (nmt->supervisory) {
					super_reset(nmt);
					osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO1, 0);
				}
			}
		}
		break;
	case ACTIVE_STATE_MWI_CLEAR:
		/* MWI during call release per NMT spec 4.4.1.16:
		 * 5c|5c|5a(L=15)|5a(L=15)|5a(L=15)|5a(L=15)
		 * Frame 5c delivers MWI data, L(15) triggers apply + release
		 * Used when call ends with pending MWI flags
		 */
		++nmt->tx_frame_count;
		if (nmt->tx_frame_count <= 2) {
			/* Frames 1-2: 5c with MWI flags */
			set_mwi_frame(nmt, frame, trans->mwi_flags);
			if (nmt->tx_frame_count == 1)
				LOGP_CHAN(DNMT, LOGL_INFO, "Sending MWI clearing: 5c x2 then L(15) x4 (flags=0x%02x).\n", trans->mwi_flags);
		} else {
			/* Frames 3-6: L(15) clearing */
			set_line_signal(nmt, frame, 15);
		}
		if (nmt->tx_frame_count >= 6) {
			LOGP_CHAN(DNMT, LOGL_DEBUG, "MWI clearing sequence sent.\n");
			/* Transition to normal release to wait for L(1) ack
			 * If phone doesn't respond, timeout will clean up
			 */
			LOGP_CHAN(DNMT, LOGL_INFO, "MWI clearing done, waiting for release ack.\n");
			nmt_new_state(nmt, STATE_MT_RELEASE);
			nmt->tx_frame_count = 0;
			osmo_timer_schedule(&nmt->timer, RELEASE_TO);
		}
		break;
	default:
		;
	}
}

/*
 * general handlers to call sub handling
 */

void nmt_receive_frame(nmt_t *nmt, const char *bits, double quality, double level, int frames_elapsed)
{
	frame_t frame;
	int rc;

	LOGP_CHAN(DDSP, LOGL_INFO, "RX Level: %.0f%% Quality=%.0f%%\n", level * 100.0, quality * 100.0);

	rc = decode_frame(nmt->sysinfo.system, &frame, bits, (nmt->sender.loopback) ? MTX_TO_XX : XX_TO_MTX, (nmt->state == STATE_MT_PAGING));
	if (rc < 0) {
		LOGP_CHAN(DNMT, (nmt->sender.loopback) ? LOGL_NOTICE : LOGL_DEBUG, "Received invalid frame.\n");
		return;
	}

	/* frame counter */
	nmt->rx_frame_count += frames_elapsed;

	LOGP_CHAN(DNMT, (nmt->sender.loopback) ? LOGL_NOTICE : LOGL_DEBUG, "Received frame %s\n", nmt_frame_name(frame.mt));

	if (nmt->sender.loopback)
		return;

	/* Observe-and-log frames that the MTX receives but this single-node implementation
	 * does not act on. We never originate the corresponding orders (frame 7 authentication,
	 * 21/22 measurement/management, 20 channel activation), so these are decode-only: report
	 * what the MS/BS reports, for diagnostics and capture analysis, then fall through. */
	switch (frame.mt) {
	case NMT_MESSAGE_16: /* Signed response (SRES) from MS — reply to an authentication request */
	    {
		/* Frame 16 carries R1R2R3R4 repeated three times (format
		 * NNNPRRRRRRRRRRRR). Recover the 16-bit SRES by majority across the
		 * three 4-digit copies, so a single corrupted copy does not spoil it
		 * (same tolerance the standard gives line signals). */
		uint16_t c0 = (frame.sres >> 32) & 0xffff;
		uint16_t c1 = (frame.sres >> 16) & 0xffff;
		uint16_t c2 = frame.sres & 0xffff;
		uint16_t sres;
		if (c0 == c1 || c0 == c2)
			sres = c0;
		else if (c1 == c2)
			sres = c1;
		else {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Received signed response (frame 16) with three differing copies (0x%04x/0x%04x/0x%04x), corrupt; authentication is not implemented, logging only.\n", c0, c1, c2);
			break;
		}
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received signed response (frame 16, SRES=0x%04x). STANDARD POINT REACHED (NMT Doc 450-1 4.7.2.1 authentication / SIS): the MTX would now compute the expected response XRES = SIS(RAND, SAK) and compare it to this SRES, accepting the MS only on a match. NOT IMPLEMENTED -- the SIS algorithm and the per-subscriber Subscriber Authentication Key (SAK) are a secret operator provision, not part of this emulator, so no comparison is made and the response is logged only.\n", sres);
		break;
	    }
	case NMT_MESSAGE_26: /* Signal strength measurement result (BS -> MTX) */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received signal strength measurement result (frame 26, TC=%d, result=0x%x); handover is not implemented, logging only.\n", frame.tc_no, frame.meas);
		break;
	case NMT_MESSAGE_25_1:
	case NMT_MESSAGE_25_2:
	case NMT_MESSAGE_25_3:
	case NMT_MESSAGE_25_4: /* Channel status information (SCC, BS -> MTX) */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received channel status information (frame 25, A=0x%x super=0x%x limit=0x%x c=0x%x); not acted on, logging only.\n", frame.chan_act, frame.supervisory, frame.limit_strength_eval, frame.c);
		break;
	case NMT_MESSAGE_27: /* Response on management/maintenance order (BS -> MTX) */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received management/maintenance response (frame 27, V=0x%x); management orders not implemented, logging only.\n", frame.meas_order);
		break;
	case NMT_MESSAGE_28: /* Other maintenance information from BS (BS -> MTX) */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received maintenance information from BS (frame 28, V=0x%x); not acted on, logging only.\n", frame.meas_order);
		break;
	case NMT_MESSAGE_10c: /* called-MS TC seizure (Scheme B1/B4 blocking path) */
		/* Handled in rx_idle when the TC is in STATE_IDLE. If we see it here
		 * the TC was not idle (call already set up, or wrong TC) -- log and ignore. */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received frame 10c (called-MS TC seizure) from subscriber %u,%06u outside idle state; ignoring.\n",
			frame.ms_country, frame.ms_number);
		break;
	default:
		break;
	}

	/* MS releases, but this is not the acknowledge of MTX release */
	if (frame.mt == NMT_MESSAGE_13a
	 && (frame.line_signal & 0xf) == 1
	 && nmt->state != STATE_MO_RELEASE
	 && nmt->state != STATE_MT_RELEASE) {
	 	/* drop packets after release */
	 	if (nmt->state == STATE_IDLE)
			return;
		if (!match_subscriber(nmt->trans, &frame))
			return;
		if (nmt_line_signal_majority(frame.line_signal, 5, 3) != 1) {
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Line signal not accepted: no majority of equal, meaningful characters (corrupt frame).\n");
			return;
		}
		LOGP_CHAN(DNMT, LOGL_INFO, "Received clearing by mobile phone (line signal L(1) '%s') in state %s.\n",
			nmt_linesignal_name(1, 0), nmt_state_name(nmt->state));
		if (nmt->trans->callref) {
			LOGP(DNMT, LOGL_INFO, "Release call towards network.\n");
			call_up_release(nmt->trans->callref, CAUSE_NORMAL);
			nmt->trans->callref = 0;
		}
		/* If already sending MWI clearing sequence, let it complete */
		if (nmt->state == STATE_ACTIVE && nmt->active_state == ACTIVE_STATE_MWI_CLEAR) {
			LOGP_CHAN(DNMT, LOGL_DEBUG, "Already in MWI clearing sequence, continuing.\n");
			return;
		}
		/* If MWI was set during call, send clearing sequence per NMT spec 4.4.1.16:
		 * 5c x2 (MWI data) then L(15) x4 (triggers phone to apply flags + release)
		 */
		if (nmt->trans->mwi_pending && nmt->state == STATE_ACTIVE) {
			LOGP_CHAN(DNMT, LOGL_INFO, "MWI pending (0x%02x), sending clearing sequence.\n", nmt->trans->mwi_pending);
			nmt->trans->mwi_flags = nmt->trans->mwi_pending;  /* Copy for clearing sequence */
			nmt->trans->mwi_pending = 0;  /* Clear to prevent restart */
			nmt->active_state = ACTIVE_STATE_MWI_CLEAR;
			nmt->tx_frame_count = 0;
			nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
			return;
		}
		nmt_new_state(nmt, STATE_MO_RELEASE);
		nmt->tx_frame_count = 0;
		nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
		return;
	}

	switch (nmt->state) {
	case STATE_IDLE:
		rx_idle(nmt, &frame);
		break;
	case STATE_ROAMING_IDENT:
		rx_roaming_ident(nmt, &frame);
		break;
	case STATE_ROAMING_CONFIRM:
		rx_roaming_confirm(nmt, &frame);
		break;
	case STATE_MO_IDENT:
		rx_mo_ident(nmt, &frame);
		break;
	case STATE_MO_CONFIRM:
	case STATE_MO_DIALING:
		rx_mo_dialing(nmt, &frame);
		break;
	case STATE_MO_RELEASE:
		break;
	case STATE_MT_PAGING:
		rx_mt_paging(nmt, &frame);
		break;
	case STATE_MT_IDENT:
		rx_mt_ident(nmt, &frame);
		break;
	case STATE_MT_AUTOANSWER:
		rx_mt_autoanswer(nmt, &frame);
		break;
	case STATE_MT_RINGING:
		rx_mt_ringing(nmt, &frame);
		break;
	case STATE_MT_RELEASE:
		rx_mt_release(nmt, &frame);
		break;
	case STATE_ACTIVE:
		rx_active(nmt, &frame);
		break;
	case STATE_SCIP_IDENT:
	case STATE_SCIP_IDENT_REQ:
		rx_scip_ident(nmt, &frame);
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Dropping message %s in state %s\n", nmt_frame_name(frame.mt), nmt_state_name(nmt->state));
	}
}

/* Timeout handling */
static void nmt_timeout(void *data)
{
	nmt_t *nmt = data;

	switch (nmt->state) {
	case STATE_MO_DIALING:
		timeout_mo_dialing(nmt);
		break;
	case STATE_MT_RINGING:
		timeout_mt_ringing(nmt);
		break;
	case STATE_MT_QUEUE:
		timeout_mt_queue(nmt);
		break;
	case STATE_MT_RELEASE:
		timeout_mt_release(nmt);
		break;
	case STATE_ACTIVE:
		timeout_active(nmt, nmt->timer.timeout.tv_sec);
		break;
	case STATE_SCIP_IDENT:
	case STATE_SCIP_IDENT_REQ:
		timeout_scip_ident(nmt);
		break;
	default:
		break;
	}
}

/* FSK processing requests next frame after transmission of previous
   frame has been finished. */
const char *nmt_get_frame(nmt_t *nmt)
{
	frame_t frame;
	const char *bits;
	int last_frame_idle, debug = 1;

	memset(&frame, 0, sizeof(frame));

	switch(nmt->state) {
	case STATE_IDLE:
		tx_idle(nmt, &frame);
		break;
	case STATE_ROAMING_IDENT:
		tx_roaming_ident(nmt, &frame);
		break;
	case STATE_ROAMING_CONFIRM:
		tx_roaming_confirm(nmt, &frame);
		break;
	case STATE_MO_IDENT:
		tx_mo_ident(nmt, &frame);
		break;
	case STATE_MO_CONFIRM:
		tx_mo_confirm(nmt, &frame);
		break;
	case STATE_MO_COMPLETE:
		tx_mo_complete(nmt, &frame);
		break;
	case STATE_MO_RELEASE:
		tx_mo_release(nmt, &frame);
		break;
	case STATE_MT_PAGING:
		tx_mt_paging(nmt, &frame);
		break;
	case STATE_MT_QUEUE:
		tx_mt_queue(nmt, &frame);
		break;
	case STATE_MT_CHANNEL:
		tx_mt_channel(nmt, &frame);
		break;
	case STATE_MT_IDENT:
		tx_mt_ident(nmt, &frame);
		break;
	case STATE_MT_AUTOANSWER:
		tx_mt_autoanswer(nmt, &frame);
		break;
	case STATE_MT_RINGING:
		tx_mt_ringing(nmt, &frame);
		break;
	case STATE_MT_COMPLETE:
		tx_mt_complete(nmt, &frame);
		break;
	case STATE_MT_RELEASE:
		tx_mt_release(nmt, &frame);
		break;
	case STATE_ACTIVE:
		tx_active(nmt, &frame);
		break;
	case STATE_SCIP_ORDER:
		tx_scip_order(nmt, &frame);
		break;
	case STATE_SCIP_IDENT:
	case STATE_SCIP_IDENT_REQ:
		tx_scip_ident(nmt, &frame);
		break;
	default:
		break;
	}

	last_frame_idle = nmt->tx_last_frame_idle;
	nmt->tx_last_frame_idle = 0;

	/* no encoding debug for certain (idle) frames */
	switch(frame.mt) {
	case NMT_MESSAGE_1a:
	case NMT_MESSAGE_1a_a:
	case NMT_MESSAGE_1a_b:
	case NMT_MESSAGE_4:
	case NMT_MESSAGE_1b:
	case NMT_MESSAGE_30:
		if (last_frame_idle)
			debug = 0;
		nmt->tx_last_frame_idle = 1;
		break;
	default:
		break;
	}

	/* frame sending aborted (e.g. due to audio) */
	if (nmt->dsp_mode != DSP_MODE_FRAME)
		return NULL;

	bits = encode_frame(nmt->sysinfo.system, &frame, debug);

	if (debug)
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Sending frame %s.\n", nmt_frame_name(frame.mt));
	if (debug && nmt->tx_last_frame_idle)
		LOGP_CHAN(DNMT, LOGL_DEBUG, "Subsequent IDLE frames are not shown, to prevent flooding the output.\n");
	return bits;
}

/*
 * call states received from call control
 */

/* Call control starts call towards mobile station. */
static int _out_setup(int callref, const char *caller_id, enum number_type caller_type, const char *dialing, const char *sms, int sms_len)
{
	sender_t *sender;
	nmt_t *nmt;
	nmt_subscriber_t subscr;
	transaction_t *trans;

	memset(&subscr, 0, sizeof(subscr));

	/* 1. split number into country and subscriber parts */
	if (dialstring2number(dialing, &subscr.country, subscr.number)) {
		LOGP(DNMT, LOGL_NOTICE, "Outgoing call to invalid number '%s', rejecting!\n", dialing);
		return -CAUSE_INVALNUMBER;
	}

	/* 2. check if given number is already in a call, return BUSY */
	trans = get_transaction_by_number(&subscr);
	if (trans) {
		LOGP(DNMT, LOGL_NOTICE, "Outgoing call to busy number, rejecting!\n");
		return -CAUSE_BUSY;
	}

	/* 3. check if all paging (calling) channels are busy, return NOCHANNEL */
	for (sender = sender_head; sender; sender = sender->next) {
		nmt = (nmt_t *) sender;
		if (!is_chan_class_cc(nmt->sysinfo.chan_type))
			continue;
		if (nmt->state == STATE_IDLE)
			break;
	}
	if (!sender) {
		LOGP(DNMT, LOGL_NOTICE, "Outgoing call, but no free calling channel, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}
	if (!search_free_tc(NULL)) {
		LOGP(DNMT, LOGL_NOTICE, "Outgoing call, but no free traffic channel, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}

	LOGP(DNMT, LOGL_INFO, "Call to mobile station, paging station id '%c%s'\n", subscr.country, subscr.number);

	/* 4. trying to page mobile station */
	trans = create_transaction(&subscr);
	if (!trans) {
		LOGP(DNMT, LOGL_NOTICE, "Failed to create transaction, rejecting!\n");
		return -CAUSE_TEMPFAIL;
	}
	trans->callref = callref;
	if (sms) {
		if (sms_len > (int)sizeof(trans->sms_string))
			sms_len = sizeof(trans->sms_string);
		memcpy(trans->sms_string, sms, sms_len);
		trans->sms_len = sms_len;
	}
	if (caller_type == TYPE_INTERNATIONAL) {
		trans->caller_id[0] = '+'; /* not done by phone */
		strncpy(trans->caller_id + 1, caller_id, sizeof(trans->caller_id) - 2);
	} else
		strncpy(trans->caller_id, caller_id, sizeof(trans->caller_id) - 1);
	trans->caller_type = caller_type;
	nmt_page(trans, 1);

	return 0;
}
int call_down_setup(int callref, const char *caller_id, enum number_type caller_type, const char *dialing)
{
	return _out_setup(callref, caller_id, caller_type, dialing, NULL, 0);
}
static int sms_out_setup(char *dialing, const char *caller_id, enum number_type caller_type, const char *sms, int sms_len)
{
	return _out_setup(0, caller_id, caller_type, dialing, sms, sms_len);
}

/* Call control signals that the far (network) end has answered.
 *
 * This is the standard-defined point for the COIN-BOX ANSWER SEQUENCE (NMT Doc
 * 450-1 4.2.1 / 4.7.2, frame 12 seizure -> special answer procedure): for a
 * coin-box MS the MTX must answer with frame 5b carrying the TARIFF CLASS Q1Q2,
 * which the coin-box uses to meter the call and echoes back in frame 13b.
 *
 * A real MTX obtains the tariff class from the charging/PSTN side at this moment.
 * This emulator has no such source, so:
 *   - if the operator configured a tariff class (nmt->coinbox_tariff >= 0), flag
 *     frame 5b to be sent on the active channel;
 *   - otherwise log that the standard point was reached but no tariff source
 *     exists, and continue as an ordinary answer (no invented value is sent). */
void call_down_answer(int callref, struct timeval __attribute__((unused)) *tv_meter)
{
	transaction_t *trans;
	nmt_t *nmt;

	trans = get_transaction_by_callref(callref);
	if (!trans)
		return;
	nmt = trans->nmt;
	if (!nmt || !trans->subscriber.coinbox)
		return;		/* not a coin-box call: ordinary answer, nothing special */

	if (nmt->coinbox_tariff < 0) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Coin-box call answered: STANDARD POINT REACHED (NMT Doc 450-1 4.7.2 coin-box answer sequence) -- frame 5b with a tariff class Q1Q2 is due here, but this emulator has NO charging/PSTN tariff source, so none is sent. Configure a tariff class to exercise the sequence. Continuing as a normal answer.\n");
		return;
	}
	LOGP_CHAN(DNMT, LOGL_INFO, "Coin-box call answered: scheduling frame 5b with configured tariff class 0x%x.\n", nmt->coinbox_tariff & 0x3);
	trans->coinbox_answer_pending = 1;
	/* Only inject the 5b sequence from an established voice call; outside it the
	 * frame cannot be placed safely, so just leave the flag logged. */
	if (nmt->state == STATE_ACTIVE && nmt->active_state == ACTIVE_STATE_VOICE) {
		nmt->active_state = ACTIVE_STATE_COINBOX_ANSWER;
		nmt->tx_frame_count = 0;
		nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
	}
}

void call_down_proceeding(int __attribute__((unused)) callref)
{
}

/* Call control sends disconnect (with tones).
 * An active call stays active, so tones and annoucements can be received
 * by mobile station.
 */
void call_down_disconnect(int callref, int cause)
{
	transaction_t *trans;
	nmt_t *nmt;

	LOGP(DNMT, LOGL_INFO, "Call has been disconnected by network.\n");

	trans = get_transaction_by_callref(callref);
	if (!trans) {
		LOGP(DNMT, LOGL_NOTICE, "Outgoing disconnect, but no callref!\n");
		call_up_release(callref, CAUSE_INVALCALLREF);
		return;
	}
	nmt = trans->nmt;

	if (!nmt) {
		call_up_release(callref, cause);
		trans->callref = 0;
		destroy_transaction(trans);
		return;
	}

	/* Release when not active and not waiting for answer */
	if (nmt->state == STATE_ACTIVE || nmt->state == STATE_MO_COMPLETE)
		return;
	switch (nmt->state) {
	case STATE_MT_RINGING:
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Outgoing disconnect, during ringing, releasing!\n");
		trans->callref = 0;
	 	nmt_release(nmt);
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Outgoing disconnect, when phone is in call setup, releasing!\n");
		trans->callref = 0;
	 	nmt_release(nmt);
		break;
	}

	call_up_release(callref, cause);
}

/* Call control releases call toward mobile station. */
void call_down_release(int callref, int __attribute__((unused)) cause)
{
	transaction_t *trans;
	nmt_t *nmt;

	LOGP(DNMT, LOGL_INFO, "Call has been released by network, releasing call.\n");

	trans = get_transaction_by_callref(callref);
	if (!trans) {
		LOGP(DNMT, LOGL_NOTICE, "Outgoing release, but no callref!\n");
		/* don't send release, because caller already released */
		return;
	}
	nmt = trans->nmt;

	trans->callref = 0;

	if (!nmt) {
		destroy_transaction(trans);
		return;
	}

	switch (nmt->state) {
	case STATE_ACTIVE:
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Outgoing release, during active call, releasing!\n");
	 	nmt_release(nmt);
		break;
	case STATE_MT_RINGING:
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Outgoing release, during ringing, releasing!\n");
	 	nmt_release(nmt);
		break;
	default:
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Outgoing release, when phone is in call setup, releasing!\n");
	 	nmt_release(nmt);
		break;
	}
}

/*
 * SMS layer messages
 */

/* SMS layer releases */
void sms_release(nmt_t *nmt)
{
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Outgoing release, by SMS layer!\n");
 	nmt_release(nmt);
}

/* SMS submit from mobile station
 * Writes to /tmp/nmt_sms_submit in format:
 *   <subscriber>,<rp_orig>,<dest>,<message>
 * Where:
 *   subscriber = Mobile station's NMT number (from call setup)
 *   rp_orig    = RP-Originator-Address from SMS (may be empty)
 *   dest       = Destination address
 *   message    = SMS text
 */
int sms_submit(nmt_t *nmt, uint8_t ref, const char *orig_address, uint8_t __attribute__((unused)) orig_type, uint8_t __attribute__((unused)) orig_plan, int __attribute__((unused)) msg_ref, const char *dest_address, uint8_t __attribute__((unused)) dest_type, uint8_t __attribute__((unused)) dest_plan, const char *message, int msg_len)
{
	char sms[1024];
	char subscriber[8];
	char text[768];
	int i, p = 0;

	/* Get mobile station's subscriber number (country + number) */
	snprintf(subscriber, sizeof(subscriber), "%c%s",
		nmt->trans->subscriber.country, nmt->trans->subscriber.number);

	/* Render the message for the text FIFO: printable ASCII verbatim, every
	 * other byte (incl. 0x00 and the 8-bit NMT national letters) as <0xNN>,
	 * so no information is lost and the operator sees exactly what arrived. */
	for (i = 0; i < msg_len && p < (int)sizeof(text) - 7; i++) {
		uint8_t c = (uint8_t)message[i];
		if (c >= 0x20 && c < 0x7f && c != '<')
			text[p++] = (char)c;
		else
			p += snprintf(text + p, sizeof(text) - p, "<0x%02x>", c);
	}
	text[p] = '\0';

	LOGP_CHAN(DNMT, LOGL_NOTICE, "Received SMS from subscriber '%s' (rp_orig='%s') to '%s' (ref=%d): %s\n",
		subscriber, orig_address, dest_address, ref, text);

	/* Format: subscriber,rp_orig,dest,message */
	snprintf(sms, sizeof(sms) - 1, "%s,%s,%s,%s", subscriber, orig_address, dest_address, text);
	sms[sizeof(sms) - 1] = '\0';

	return submit_sms(sms);
}

void sms_deliver_report(nmt_t *nmt, uint8_t ref, int error, uint8_t cause)
{
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Got SMS deliver report (ref=%d)\n", ref);
	if (error)
		printf("SMS failed! (cause=%d)\n", cause);
	else {
		sms_ref++;
		printf("SMS sent!\n");
	}
}

/* application sends ud a message, we need to deliver */
void deliver_sms(const char *sms)
{
	int rc;
	char buffer[strlen(sms) + 1], *p = buffer, *caller_id, *number, *message;
	enum number_type caller_type;

	strcpy(buffer, sms);
	caller_id = strsep(&p, ",");
	number = strsep(&p, ",");
	message = p;
	if (!caller_id || !number || !message) {
inval:
		LOGP(DNMT, LOGL_NOTICE, "Given SMS MUST be in the following format: [i|n|s|u]<caller ID>,<7 digits number>,<message with comma and spaces> (i, n, s, u indicate the type of number)\n");
		return;
	}
	if (strlen(number) != 7) {
		LOGP(DNMT, LOGL_NOTICE, "Given number must be 7 digits\n");
		goto inval;
	}

	switch(caller_id[0]) {
		case '\0':
			caller_type = TYPE_NOTAVAIL;
			break;
		case 'i':
			caller_type = TYPE_INTERNATIONAL;
			caller_id++;
			break;
		case 'n':
			caller_type = TYPE_NATIONAL;
			caller_id++;
			break;
		case 's':
			caller_type = TYPE_SUBSCRIBER;
			caller_id++;
			break;
		case 'u':
			caller_type = TYPE_UNKNOWN;
			caller_id++;
			break;
		default:
			caller_type = TYPE_UNKNOWN;
	}

	LOGP(DNMT, LOGL_INFO, "SMS from '%s' for subscriber '%s' with message '%s'\n", caller_id, number, message);
	printf("SMS sending '%s' -> '%s': %s\n", caller_id, number, message);

	/* Decode <0xNN> escapes so the operator can inject any byte, including
	 * non-printable ones and 0x00, through the text FIFO. Everything else is
	 * copied verbatim. The result is a byte buffer with an explicit length. */
	char msg_bytes[256];
	int msg_len = 0;
	{
		const char *s = message;
		while (*s && msg_len < (int)sizeof(msg_bytes)) {
			if (s[0] == '<' && (s[1] == '0') && (s[2] == 'x' || s[2] == 'X')
			 && isxdigit((unsigned char)s[3]) && isxdigit((unsigned char)s[4]) && s[5] == '>') {
				msg_bytes[msg_len++] = (uint8_t)strtol(s + 1, NULL, 16);
				s += 6;
			} else {
				msg_bytes[msg_len++] = *s++;
			}
		}
	}

	rc = sms_out_setup(number, caller_id, caller_type, msg_bytes, msg_len);
	if (rc < 0) {
		LOGP(DNMT, LOGL_INFO, "SMS delivery failed with cause '%d'\n", -rc);
		return;
	}
}

/*
 * MWI (Message Waiting Indicator) handling
 * See docs/NMT_MWI.md for details.
 */

/* MWI is delivered via frame 5c during calls
 *
 * SET command: Sends frame 5c to turn ON indicators
 *   - Out-of-call: Pages subscriber, sends 5c x2, then L(15) x4 to release
 *   - In-call: Sends 5c x2 immediately, accumulates flags for call end
 *
 * When any call ends with pending MWI, clearing sequence is sent:
 *   5c x2 (MWI data) then L(15) x4 (triggers phone to apply all flags)
 * Per NMT spec 4.4.1.16.
 */

/* Format MWI flags as human-readable string */
static const char *mwi_flags_string(uint8_t flags)
{
	static char buf[64];
	buf[0] = '\0';
	if (flags & 0x01) strcat(buf, "sms ");
	if (flags & 0x02) strcat(buf, "voice ");
	if (flags & 0x04) strcat(buf, "fax ");
	if (flags & 0x08) strcat(buf, "email ");
	if (flags & 0x10) strcat(buf, "data ");
	if (buf[0] == '\0') strcpy(buf, "(none)");
	return buf;
}

/* Send MWI in-call to subscriber with active connection
 *
 * Sends frame 5c x2 immediately and accumulates flags for clearing sequence.
 * The call continues after frame 5c is sent.
 * When call ends, clearing sequence (5c x2 then L(15) x4) will be sent.
 */
static int mwi_send_in_call(nmt_t *nmt, uint8_t flags)
{
	transaction_t *trans = nmt->trans;

	LOGP_CHAN(DNMT, LOGL_INFO, "MWI in-call SET to '%c%s': %s(0x%02x)\n",
		trans->subscriber.country, trans->subscriber.number,
		mwi_flags_string(flags), flags);

	/* Store flags for immediate frame 5c */
	trans->mwi_flags = flags;
	/* Accumulate flags for clearing sequence when call ends */
	trans->mwi_pending |= flags;

	/* Switch to frame mode to send MWI, then return to voice */
	nmt->tx_frame_count = 0;
	nmt->active_state = ACTIVE_STATE_MWI;
	nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);

	printf("MWI in-call: %s(0x%02x)\n", mwi_flags_string(flags), flags);
	return 0;
}

/* Set MWI indicators for subscriber
 *
 * If subscriber has active call: sends frame 5c x2, accumulates flags
 * If subscriber idle: initiates MWI delivery call (5c x2 then L(15) x4)
 *
 * When any call ends with pending MWI flags, clearing sequence is sent:
 * 5c x2 then L(15) x4 per NMT spec 4.4.1.16.
 */
static int mwi_set(const char *dialing, uint8_t flags)
{
	sender_t *sender;
	nmt_t *nmt;
	nmt_subscriber_t subscr;
	transaction_t *trans;

	memset(&subscr, 0, sizeof(subscr));

	/* 1. split number into country and subscriber parts */
	if (dialstring2number(dialing, &subscr.country, subscr.number)) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: invalid number '%s'\n", dialing);
		return -CAUSE_INVALNUMBER;
	}

	/* 2. check if subscriber has an active call - send in-call MWI */
	trans = get_transaction_by_number(&subscr);
	if (trans && trans->nmt && trans->nmt->state == STATE_ACTIVE) {
		LOGP(DNMT, LOGL_INFO, "MWI: subscriber '%s' has active call, sending in-call\n", dialing);
		return mwi_send_in_call(trans->nmt, flags);
	}

	/* 3. if subscriber is busy but not in active state, reject */
	if (trans) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: subscriber '%s' is busy (state: not active), rejecting\n", dialing);
		return -CAUSE_BUSY;
	}

	/* 4. check if all paging (calling) channels are busy */
	for (sender = sender_head; sender; sender = sender->next) {
		nmt = (nmt_t *) sender;
		if (!is_chan_class_cc(nmt->sysinfo.chan_type))
			continue;
		if (nmt->state == STATE_IDLE)
			break;
	}
	if (!sender) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: no free calling channel\n");
		return -CAUSE_NOCHANNEL;
	}
	if (!search_free_tc(NULL)) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: no free traffic channel\n");
		return -CAUSE_NOCHANNEL;
	}

	LOGP(DNMT, LOGL_INFO, "MWI out-of-call to '%c%s': %s(0x%02x)\n",
		subscr.country, subscr.number,
		mwi_flags_string(flags), flags);

	/* 5. create transaction and page mobile station */
	trans = create_transaction(&subscr);
	if (!trans) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: failed to create transaction\n");
		return -CAUSE_TEMPFAIL;
	}
	trans->callref = 0;
	trans->mwi_call = 1;
	trans->mwi_flags = flags;
	trans->mwi_pending = flags;  /* Accumulated flags for clearing */
	nmt_page(trans, 1);

	return 0;
}

/* Parse indicator flags from text or numeric value
 *
 * Indicators (space-separated text or numeric 0-31):
 *   sms   = bit 0 (0x01)
 *   voice = bit 1 (0x02)
 *   fax   = bit 2 (0x04)
 *   email = bit 3 (0x08)
 *   data  = bit 4 (0x10)
 */
static uint8_t parse_mwi_flags(const char *indicators)
{
	uint8_t flags = 0;

	if (!indicators || !indicators[0])
		return 0;

	/* Check if numeric */
	if (indicators[0] >= '0' && indicators[0] <= '9') {
		int raw = atoi(indicators);
		flags = raw & 0x1F;
		if (raw < 0 || raw > 0x1F)
			LOGP(DNMT, LOGL_NOTICE, "MWI numeric flags %d out of range for the 5 defined indicators (0..31), using 0x%02x.\n", raw, flags);
		LOGP(DNMT, LOGL_DEBUG, "MWI flags (numeric): 0x%02x\n", flags);
		return flags;
	}

	/* Parse space-separated text values (lowercase only) */
	if (strstr(indicators, "sms"))
		flags |= 0x01;
	if (strstr(indicators, "voice"))
		flags |= 0x02;
	if (strstr(indicators, "fax"))
		flags |= 0x04;
	if (strstr(indicators, "email"))
		flags |= 0x08;
	if (strstr(indicators, "data"))
		flags |= 0x10;

	LOGP(DNMT, LOGL_DEBUG, "MWI flags (text '%s'): 0x%02x\n", indicators, flags);
	return flags;
}

/* Deliver MWI update to subscriber
 *
 * Command format: <subscriber>,set,<indicators>
 *   subscriber: 7-digit NMT number
 *   indicators: space-separated text (sms voice fax email data) or numeric 0-31
 *
 * Behavior:
 *   Out-of-call: Sends frame 5c immediately to turn ON indicators
 *   In-call: Sends frame 5c immediately + accumulates flags for clearing
 *   When call ends: If MWI was set during call, sends L(15)+5c clearing sequence
 *
 * Examples:
 *   3735859,set,sms         - Set SMS indicator
 *   3735859,set,sms voice   - Set SMS and voice indicators
 *   3735859,set,3           - Set SMS + voice (numeric: bits 0+1 = 3)
 */
void deliver_mwi(const char *command)
{
	int rc;
	char buffer[256], *p, *number, *action, *indicators;
	uint8_t flags;

	if (!command || !command[0]) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: empty command\n");
		return;
	}

	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';
	p = buffer;

	/* Parse: number,action,indicators */
	number = strsep(&p, ",");
	action = strsep(&p, ",");
	indicators = p;  /* rest of string, may contain spaces */

	if (!number || !action) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: invalid format. Use: <number>,set,<indicators>\n");
		LOGP(DNMT, LOGL_NOTICE, "  number: 7-digit subscriber number\n");
		LOGP(DNMT, LOGL_NOTICE, "  indicators: sms voice fax email data (space-separated) or 0-31\n");
		return;
	}

	if (strlen(number) != 7) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: number must be 7 digits, got '%s'\n", number);
		return;
	}

	/* Only "set" action is supported */
	if (strcmp(action, "set") != 0) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: unknown action '%s'. Use: set\n", action);
		return;
	}

	flags = parse_mwi_flags(indicators);
	if (flags == 0) {
		LOGP(DNMT, LOGL_NOTICE, "MWI: no indicators specified\n");
		return;
	}

	LOGP(DNMT, LOGL_INFO, "MWI set '%s': %s(0x%02x)\n",
		number, mwi_flags_string(flags), flags);
	printf("MWI set '%s': %s(0x%02x)\n",
		number, mwi_flags_string(flags), flags);

	rc = mwi_set(number, flags);
	if (rc < 0) {
		LOGP(DNMT, LOGL_NOTICE, "MWI delivery failed: %s\n",
			(rc == -CAUSE_INVALNUMBER) ? "invalid number" :
			(rc == -CAUSE_BUSY) ? "subscriber busy" :
			(rc == -CAUSE_NOCHANNEL) ? "no channel available" :
			(rc == -CAUSE_TEMPFAIL) ? "temporary failure" : "unknown error");
		printf("MWI failed!\n");
		return;
	}
}

/*
 * SCIP (Switching Call In Progress): move an active call to another voice/traffic channel.
 * Modelled on AMPS amps_handoff(). Operator-triggered via the /tmp/nmt_move FIFO.
 */

/* Find a free TC other than 'own' (own stays busy with the call we are moving). */
static nmt_t *scip_find_target(nmt_t *own, int want_channel)
{
	sender_t *sender;
	nmt_t *nmt;

	for (sender = sender_head; sender; sender = sender->next) {
		nmt = (nmt_t *) sender;
		if (nmt == own)
			continue;
		if (nmt->state != STATE_IDLE)
			continue;
		if (!is_chan_class_tc(nmt->sysinfo.chan_type))
			continue;
		if (want_channel) {
			if (atoi(nmt->sender.kanal) == want_channel)
				return nmt;
			continue;
		}
		return nmt; /* auto-select: first free TC */
	}

	return NULL;
}

/* Initiate the move. want_channel == 0 -> auto-select a free TC. */
static int nmt_scip(const char *dialing, int want_channel)
{
	nmt_subscriber_t subscr;
	transaction_t *trans;
	nmt_t *nmt, *tc;

	memset(&subscr, 0, sizeof(subscr));
	if (dialstring2number(dialing, &subscr.country, subscr.number)) {
		LOGP(DNMT, LOGL_NOTICE, "SCIP: invalid number '%s'\n", dialing);
		return -CAUSE_INVALNUMBER;
	}

	trans = get_transaction_by_number(&subscr);
	if (!trans || !trans->nmt) {
		LOGP(DNMT, LOGL_NOTICE, "SCIP: subscriber '%s' not found in a call\n", dialing);
		return -CAUSE_OUTOFORDER;
	}
	nmt = trans->nmt;

	if (nmt->state != STATE_ACTIVE) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: subscriber '%s' not in active call (state %s)\n", dialing, nmt_state_name(nmt->state));
		return -CAUSE_BUSY;
	}
	/* only move a normal voice call; MFT/MWI sub-states and data calls are not movable */
	if (nmt->active_state != ACTIVE_STATE_VOICE || trans->dms_call || trans->mwi_call) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: call for '%s' is not in plain voice state, not moving\n", dialing);
		return -CAUSE_BUSY;
	}
	if (want_channel && want_channel == atoi(nmt->sender.kanal)) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: already on channel %d\n", want_channel);
		return -CAUSE_INVALNUMBER;
	}

	tc = scip_find_target(nmt, want_channel);
	if (!tc) {
		if (want_channel)
			LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: target channel %d is not a free TC\n", want_channel);
		else
			LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: no free traffic channel to move to\n");
		return -CAUSE_NOCHANNEL;
	}

	LOGP_CHAN(DNMT, LOGL_NOTICE, "SCIP: moving call '%c%s' from TC #%s to TC #%s\n",
		trans->subscriber.country, trans->subscriber.number, nmt->sender.kanal, tc->sender.kanal);

	/* arm the order on the current channel; the move happens in tx_scip_order() */
	nmt->scip_target = tc;
	nmt->tx_frame_count = 0;
	osmo_timer_del(&nmt->timer); /* stop supervisory timer while ordering */
	nmt_new_state(nmt, STATE_SCIP_ORDER);
	nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);

	return 0;
}

/* Deliver a move command from the control FIFO.
 * Format: <subscriber>[,<channel>]   e.g. "3735859" (auto) or "3735859,5" (to channel 5)
 */
void deliver_move(const char *command)
{
	char buffer[256], *p, *number, *chan_str;
	int want_channel = 0;
	int rc;

	if (!command || !command[0]) {
		LOGP(DNMT, LOGL_NOTICE, "MOVE: empty command\n");
		return;
	}

	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';
	p = buffer;

	number = strsep(&p, ",");
	chan_str = p; /* may be NULL */

	if (!number || strlen(number) != 7) {
		LOGP(DNMT, LOGL_NOTICE, "MOVE: number must be 7 digits. Use: <number>[,<channel>]\n");
		return;
	}
	if (chan_str && chan_str[0])
		want_channel = atoi(chan_str);

	LOGP(DNMT, LOGL_INFO, "MOVE '%s' to %s\n", number, want_channel ? chan_str : "free channel");
	printf("MOVE '%s' to %s\n", number, want_channel ? chan_str : "auto-selected free channel");

	rc = nmt_scip(number, want_channel);
	if (rc < 0) {
		LOGP(DNMT, LOGL_NOTICE, "MOVE failed: %s\n",
			(rc == -CAUSE_INVALNUMBER) ? "invalid number / same channel" :
			(rc == -CAUSE_OUTOFORDER) ? "subscriber not in a call" :
			(rc == -CAUSE_BUSY) ? "call not in a movable state" :
			(rc == -CAUSE_NOCHANNEL) ? "no free traffic channel" : "unknown error");
		printf("MOVE failed!\n");
		return;
	}
	printf("MOVE ordered.\n");
}

/* Change MS output power level during a call (NMT Doc 450-1 §4.4.1.8).
 * Command format: <7-digit-number>,<power-level 0-7>
 * The MTX sends frame 3a (same TC, new power encoded in Na) up to T (8 frames).
 * The MS re-identifies with 10b confirming the new power; nmt->sysinfo.ms_power
 * is then updated for all subsequent channel-number encodings.
 * Example: echo "3735859,2" > /tmp/nmt_power */
void deliver_power_change(const char *command)
{
	char buffer[64], *p, *number, *power_str;
	int new_power;
	nmt_subscriber_t subscr;
	transaction_t *trans;
	nmt_t *nmt;

	if (!command || !command[0]) {
		LOGP(DNMT, LOGL_NOTICE, "Power change: empty command.\n");
		return;
	}
	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';
	p = buffer;
	number = strsep(&p, ",");
	power_str = p;
	if (!number || !power_str || !power_str[0]) {
		LOGP(DNMT, LOGL_NOTICE, "Power change: invalid format. Use: <7-digit-number>,<power 0-7>\n");
		return;
	}
	if (strlen(number) != 7) {
		LOGP(DNMT, LOGL_NOTICE, "Power change: subscriber number must be 7 digits.\n");
		return;
	}
	new_power = atoi(power_str);
	if (new_power < 0 || new_power > 7) {
		LOGP(DNMT, LOGL_NOTICE, "Power change: power level %d out of range 0..7.\n", new_power);
		return;
	}
	memset(&subscr, 0, sizeof(subscr));
	if (dialstring2number(number, &subscr.country, subscr.number)) {
		LOGP(DNMT, LOGL_NOTICE, "Power change: invalid subscriber number '%s'.\n", number);
		return;
	}
	trans = get_transaction_by_number(&subscr);
	if (!trans || !trans->nmt || trans->nmt->state != STATE_ACTIVE) {
		LOGP(DNMT, LOGL_NOTICE, "Power change: subscriber '%s' not in an active call.\n", number);
		return;
	}
	nmt = trans->nmt;
	if (nmt->active_state != ACTIVE_STATE_VOICE) {
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Power change: subscriber '%s' is not in voice state (currently '%s'), cannot issue power change.\n", number, nmt_active_state_name(nmt->active_state));
		return;
	}
	LOGP_CHAN(DNMT, LOGL_NOTICE, "Power change ordered: subscriber '%s', new power level %d (current %d).\n",
		number, new_power, nmt->sysinfo.ms_power);
	trans->pending_power = new_power;
	nmt->active_state = ACTIVE_STATE_POWER_CHANGE;
	nmt->tx_frame_count = 0;
	nmt_set_dsp_mode(nmt, DSP_MODE_FRAME);
}

void dump_info(void) {}

