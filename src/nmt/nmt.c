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
	 * (SIS) variant, which is not used here (SIS is not implemented). */
	set_line_signal(nmt, frame, 3);
	if (++nmt->tx_frame_count == 1)
		LOGP_CHAN(DNMT, LOGL_INFO, "Send 'Roaming updating confirmation'.\n");
	if (nmt->tx_frame_count == 2)
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
		LOGP_CHAN(DNMT, LOGL_INFO, "TC became free, assigning queued call.\n");
		osmo_timer_del(&nmt->timer);
		nmt_new_state(nmt, STATE_MT_CHANNEL);
		nmt->tx_frame_count = 0;
		tx_mt_channel(nmt, frame);
		return;
	}

	/* otherwise keep telling the MS it is queued (2c priority, 2f ordinary) */
	frame->mt = trans->priority ? NMT_MESSAGE_2c : NMT_MESSAGE_2f;
	frame->channel_no = nmt_encode_channel(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.ms_power);
	frame->traffic_area = nmt_encode_traffic_area(nmt->sysinfo.system, atoi(nmt->sender.kanal), nmt->sysinfo.traffic_area);
	frame->ms_country = nmt_digits2value(&trans->subscriber.country, 1);
	frame->ms_number = nmt_digits2value(trans->subscriber.number, 6);
	/* H8H9H10 distinguishes the queue type: 3F1 = 2c (priority), 3F0 = 2f (ordinary) */
	frame->additional_info = trans->priority ? 0x3f1 : 0x3f0;
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
		LOGP_CHAN(DNMT, LOGL_INFO, "Release call towards network.\n");
		call_up_release(trans->callref, CAUSE_TEMPFAIL);
		destroy_transaction(trans);
	}
}

static void rx_mt_ident(nmt_t *nmt, frame_t *frame)
{
	transaction_t *trans = nmt->trans;

	switch (frame->mt) {
	case NMT_MESSAGE_10b: /* seizure */
		if (!match_subscriber(trans, frame))
			break;
		nmt_value2digits(frame->ms_password, trans->subscriber.password, 3);
		LOGP_CHAN(DNMT, LOGL_INFO, "Received identity (password %s).\n", trans->subscriber.password);
		if (trans->dms_call || trans->mwi_call) {
			/* DMS (SMS) or MWI call - use autoanswer path */
			nmt_new_state(nmt, STATE_MT_AUTOANSWER);
			nmt->wait_autoanswer = 1;
			nmt->tx_frame_count = 0;
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
		nmt_new_state(nmt, STATE_MT_COMPLETE);
		nmt->tx_frame_count = 0;
		osmo_timer_del(&nmt->timer);
		call_up_answer(trans->callref, &trans->subscriber.country);
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

	/* Encode flags into FFF field with correct bit positions
	 * Each indicator sets both i_n (bits 11-6) and i'_n (bits 5-0)
	 */
	if (flags & 0x01) info |= 0x820;  /* SMS:   i1 at bit 11, i'1 at bit 5 */
	if (flags & 0x02) info |= 0x410;  /* voice: i2 at bit 10, i'2 at bit 4 */
	if (flags & 0x04) info |= 0x208;  /* fax:   i3 at bit 9,  i'3 at bit 3 */
	if (flags & 0x08) info |= 0x104;  /* email: i4 at bit 8,  i'4 at bit 2 */
	if (flags & 0x10) info |= 0x082;  /* data:  i5 at bit 7,  i'5 at bit 1 */

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
	if (nmt->tx_frame_count == 5) {
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
	LOGP(DNMT, LOGL_INFO, "Release call towards network.\n");
	call_up_release(trans->callref, CAUSE_NOANSWER);
	trans->callref = 0;
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

	if (tone)
		osmo_timer_del(&nmt->timer);
	else
		osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO2);
}

static void timeout_active(nmt_t *nmt, int duration)
{
	transaction_t *trans = nmt->trans;

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

	/* restart timer on every reception of frame */
	if (nmt->supervisory)
		osmo_timer_schedule(&nmt->timer, SUPERVISORY_TO2);

	switch (frame->mt) {
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
			/* Register recall (NMT Doc 450-1 4.3.2 L(5) in frame 13a, procedure
			 * 4.7.2.11 "Subscriber service by register recall and code sending").
			 * In conversation state the subscriber requests that a REGISTER in the
			 * MTX be connected to the line; digits/codes are then sent MS->MTX using
			 * the authentication frame pair (frame 7 request / frame 16 response,
			 * per 4.7.2.11 which refers to 4.7.2.1). We decode the request and move
			 * to the register-recall state, but STOP here: there is no MTX register /
			 * digit-collection / supplementary-service back-end in this single-node
			 * emulator to connect the line to, so the standard's next step cannot be
			 * performed. The call stays up (voice) -- we do not drop or alter it. */
			if (nmt->active_state != ACTIVE_STATE_VOICE) {
				LOGP_CHAN(DNMT, LOGL_NOTICE, "Received register-recall request (L(5)) outside voice state; ignoring.\n");
				break;
			}
			nmt->active_state = ACTIVE_STATE_REGISTER_RECALL;
			LOGP_CHAN(DNMT, LOGL_NOTICE, "Register recall requested (L(5), NMT Doc 450-1 4.7.2.11): the MS asks to connect an MTX register to the line for code sending. STANDARD POINT REACHED but NOT IMPLEMENTED -- this emulator has no MTX register / digit-collection back-end, so no register is connected and no supplementary-service code can be processed. Call stays in voice; the MS will see no register response.\n");
			/* Remain in conversation: nothing to send back, so drop straight back to
			 * voice so the open call is not stuck in a dead sub-state. */
			nmt->active_state = ACTIVE_STATE_VOICE;
			break;
		case 8:
			if (nmt->active_state != ACTIVE_STATE_VOICE)
				break;
			LOGP_CHAN(DNMT, LOGL_INFO, "Received 'MFT in' request.\n");
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
		if (nmt->active_state != ACTIVE_STATE_MFT)
			break;
		if ((nmt->mft_num & 1))
			break;
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
		if (nmt->active_state != ACTIVE_STATE_MFT)
			break;
		if (!(nmt->mft_num & 1))
			break;
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
	case ACTIVE_STATE_COINBOX_ANSWER:
		/* Coin-box answer sequence (NMT Doc 450-1 4.7.2): send frame 5b carrying
		 * the operator-supplied tariff class, repeated like other line signals,
		 * then return to the voice path. The coin-box MS acknowledges by echoing
		 * the tariff class in frame 13b (decoded/logged in rx_active). */
		set_coinbox_answer(nmt, frame, nmt->coinbox_tariff & 0x3);
		if (++nmt->tx_frame_count > 4) {
			trans->coinbox_answer_pending = 0;
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			if (nmt->supervisory)
				super_reset(nmt);
		}
		break;
	case ACTIVE_STATE_MFT_IN:
		set_line_signal(nmt, frame, 4);
		if (++nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Send 'MFT in acknowledge'.\n");
		if (nmt->tx_frame_count > 4) {
			nmt->active_state = ACTIVE_STATE_MFT;
			nmt_set_dsp_mode(nmt, DSP_MODE_DTMF);
		}
		break;
	case ACTIVE_STATE_MFT_OUT:
		set_line_signal(nmt, frame, 10);
		if (++nmt->tx_frame_count == 1)
			LOGP_CHAN(DNMT, LOGL_INFO, "Send 'MFT out acknowledge'.\n");
		if (nmt->tx_frame_count > 4) {
			nmt->active_state = ACTIVE_STATE_VOICE;
			nmt_set_dsp_mode(nmt, DSP_MODE_AUDIO);
			if (nmt->supervisory)
				super_reset(nmt);
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
	case NMT_MESSAGE_10c: /* Seizure and identity from called MS after a traffic channel scanning order */
		LOGP_CHAN(DNMT, LOGL_NOTICE, "Received called-MS seizure on TC (frame 10c, subscriber %u,%06u); traffic channel scanning order not implemented, logging only.\n", frame.ms_country, frame.ms_number);
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
		flags = atoi(indicators) & 0x1F;
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

void dump_info(void) {}

