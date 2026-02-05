/* AMPS protocol handling
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

/*
 * Notes on frames and scheduling:
 *
 * If the amps->dsp_mode is set to transmit frames, fsk_frame() at dsp.c code
 * requests frames, modulates them and forwards them to sound device.  Whenever
 * the dsp.c code requests frame (if no frame exists or frame had been sent),
 * it calls amps_encode_frame_focc() or amps_encode_frame_fvc() of frame.c.
 * There it generates a sequence of frames (message train). If no sequence is
 * transmitted or a new sequence starts, amps_tx_frame_focc() or
 * amps_tx_frame_fvc() of amps.c is called.  There it sets message data and
 * other states according to the current trans->state.
 *
 * If a frame is received by dsp.c code, amps_decode_frame() at frame.c is
 * called. There the bits are decoded and messages are assembled from multiple
 * frames.  Then amps_rx_recc() at amps.c is called, so the received messages
 * are processed.
 */

#define CHAN amps->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../libmobile/get_time.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include "../libmobile/console.h"
#include <osmocom/cc/message.h>
#include "amps.h"
#include "dsp.h"
#include "frame.h"
#include "stations.h"
#include "esn.h"
#include "main.h"

/* Uncomment this to test SAT via loopback */
//#define DEBUG_VC

#define SAT_TO1		5,0		/* 5 sec to detect after setup */
#define SAT_TO2		5,0		/* 5 sec lost until abort (specs say 5) */
#define PAGE_TRIES	2		/* how many times to page the phone */
#define PAGE_TO1	8,0		/* max time to wait for paging reply */
#define PAGE_TO2	4,0		/* max time to wait for last paging reply */
#define ALERT_TRIES	3		/* how many times to alert the phone */
#define ALERT_TO	0,600000	/* max time to wait for alert confirm */
#define ANSWER_TO	60,0		/* max time to wait for answer */
#define RELEASE_TIMER	5,0		/* max time to send release messages */
#define ST_FLASH_MIN	0.2		/* min duration for flash (200ms) */
#define ST_FLASH_MAX	0.8		/* max duration for flash (800ms) */
#define ST_RELEASE_TIME	1,800000	/* duration for release (1.8s) */

/* Power Control Thresholds */
#define VMAC_ADJUST_INTERVAL        50     /* Measurements between adjustments */

/* Convert channel number to frequency number of base station.
   Set 'uplink' to 1 to get frequency of mobile station. */
double amps_channel2freq(int channel, int uplink)
{
	double freq;

	if (!tacs) {
		/* AMPS */
		if (uplink == 2)
			return -45.000 * 1e6;

		/* 832 channels, 990 not used, see TIA/EIA-136-110 */
		if (channel < 1 || channel > 1023 || (channel > 799 && channel < 991))
			return 0;

		if (channel >= 990) // 990 is not used
			channel -= 1023;

		freq = 870.030 + (channel - 1) * 0.030;

		if (uplink)
			freq -= 45.000;
	} else if (!jtacs) {
		/* TACS */
		if (uplink == 2)
			return -45.000 * 1e6;

		/* 600 channels */
		if (channel < 1 || channel > 600)
			return 0;

		freq = 935.0125 + (channel - 1) * 0.025;

		if (uplink)
			freq -= 45.000;
	} else {
		/* JTACS */
		/* see "ARIB_STD-T64-C.S0057-0v1.0.pdf" */
		if (uplink == 2)
			return 55.000 * 1e6;

		/* 799 channels */
		if (channel >= 1 && channel <= 799)
			freq = 860.0125 + (channel - 1) * 0.0125;
		else if (channel >= 801 && channel <= 1039)
			freq = 843.0125 + (channel - 801) * 0.0125;
		else if (channel >= 1041 && channel <= 1199)
			freq = 832.0125 + (channel - 1041) * 0.0125;
		else if (channel >= 1201 && channel <= 1600)
			freq = 838.0125 + (channel - 1201) * 0.0125;
		else
			return 0;


		if (uplink)
			freq += 55.000;
	}

	return freq * 1e6;
}

enum amps_chan_type amps_channel2type(int channel)
{
	if (!tacs) {
		/* AMPS */
		if (channel >= 313 && channel <= 354)
			return CHAN_TYPE_CC;
	} else if (!jtacs) {
		/* TACS */
		if (channel >= 23 && channel <= 43)
			return CHAN_TYPE_CC;
		if (channel >= 323 && channel <= 343)
			return CHAN_TYPE_CC;
	} else {
		/* JTACS */
		if (channel >= 418 && channel <= 456)
			return CHAN_TYPE_CC;
	}

	return CHAN_TYPE_VC;
}

const char *amps_channel2band(int channel)
{
	if (!tacs) {
		/* AMPS */
		if (channel >= 991 && channel <= 1023)
			return "A''";
		if (channel >= 1 && channel <= 333)
			return "A";
		if (channel >= 334 && channel <= 666)
			return "B";
		if (channel >= 667 && channel <= 716)
			return "A'";
		if (channel >= 717 && channel <= 799)
			return "B'";
	} else if (!jtacs) {
		/* TACS */
		if (channel >= 1 && channel <= 300)
			return "A";
		if (channel >= 301 && channel <= 600)
			return "B";
	} else {
		/* JTACS */
		return "A";
	}

	return "<invalid>";
}

static inline int digit2binary(int digit)
{
	if (digit == '0')
		return 10;
	return digit - '0';
}

static inline int binary2digit(int binary)
{
	if (binary == 10)
		return '0';
	return binary + '0';
}

/* AMPS: convert NPA-NXX-XXXX to MIN1 and MIN2
 * NPA = numbering plan area (MIN2)
 * NXX = mobile exchange code
 * XXXX = telephone number within the exchange
 */
/* TACS: convert AREA-XXXXXX to MIN1 and MIN2
 * AREA = 3 + 1 Digits
 * XXXXXX = telephone number
 */
int amps_number2min(const char *number, uint32_t *min1, uint16_t *min2)
{
	int nlen = strlen(number);
	int i;

	if (nlen != 10) {
		fprintf(stderr, "illegal length %d. Must be 10, returning error!\n", nlen);
		return -1;
	}

	for (i = 0; i < nlen; i++) {
		if (number[i] < '0' || number[i] > '9') {
			fprintf(stderr, "illegal number %s. Must consists only of digits 0..9, returning error!\n", number);
			return -1;
		}
	}

	/* MIN2 */
	if (nlen == 10) {
		*min2 = digit2binary(number[0]) * 100 + digit2binary(number[1]) * 10 + digit2binary(number[2]) - 111;
		number += 3;
		nlen -= 3;
	}

	if (!tacs) {
		/* MIN1 (amps) */
		*min1 = ((uint32_t)(digit2binary(number[0]) * 100 + digit2binary(number[1]) * 10 + digit2binary(number[2]) - 111)) << 14;
		*min1 |= digit2binary(number[3]) << 10;
		*min1 |= digit2binary(number[4]) * 100 + digit2binary(number[5]) * 10 + digit2binary(number[6]) - 111;
	} else {
		/* MIN1 (tacs/jtacs) */
		*min1 = digit2binary(number[0]) << 20;
		*min1 |= (digit2binary(number[1]) * 100 + digit2binary(number[2]) * 10 + digit2binary(number[3]) - 111) << 10;
		*min1 |= digit2binary(number[4]) * 100 + digit2binary(number[5]) * 10 + digit2binary(number[6]) - 111;
	}

	return 0;
}

/* AMPS: convert MIN1 and MIN2 to NPA-NXX-XXXX
 */
/* TACS: convert MIN1 and MIN2 to AREA-XXXXXXX
 */
/* JTACS: convert MIN1 and MIN2 to NET-XXXXXXX (NET = mobile network code, always 440)
 */
const char *amps_min22number(uint16_t min2)
{
	static char number[4];

	/* MIN2 */
	if (min2 > 999)
		strcpy(number, "???");
	else {
		number[0] = binary2digit((min2 / 100) + 1);
		number[1] = binary2digit(((min2 / 10) % 10) + 1);
		number[2] = binary2digit((min2 % 10) + 1);
	}
	number[3] = '\0';

	return number;
}

const char *amps_min12number(uint32_t min1)
{
	static char number[8];

	if (!tacs) {
		/* MIN1 (amps) */
		if ((min1 >> 14) > 999)
			strcpy(number, "???");
		else {
			number[0] = binary2digit(((min1 >> 14) / 100) + 1);
			number[1] = binary2digit((((min1 >> 14) / 10) % 10) + 1);
			number[2] = binary2digit(((min1 >> 14) % 10) + 1);
		}
		if (((min1 >> 10) & 0xf) < 1 || ((min1 >> 10) & 0xf) > 10)
			number[3] = '?';
		else
			number[3] = binary2digit((min1 >> 10) & 0xf);
		if ((min1 & 0x3ff) > 999)
			strcpy(number + 4, "???");
		else {
			number[4] = binary2digit(((min1 & 0x3ff) / 100) + 1);
			number[5] = binary2digit((((min1 & 0x3ff) / 10) % 10) + 1);
			number[6] = binary2digit(((min1 & 0x3ff) % 10) + 1);
		}
	} else {
		/* MIN1 (tacs/jtacs) */
		if ((min1 >> 20) < 1 || (min1 >> 20) > 10)
			number[0] = '?';
		else
			number[0] = binary2digit(min1 >> 20);
		if (((min1 >> 10) & 0x3ff) > 999)
			strcpy(number +  1, "???");
		else {
			number[1] = binary2digit((((min1 >> 10) & 0x3ff) / 100) + 1);
			number[2] = binary2digit(((((min1 >> 10) & 0x3ff) / 10) % 10) + 1);
			number[3] = binary2digit((((min1 >> 10) & 0x3ff) % 10) + 1);
		}
		if ((min1 & 0x3ff) > 999)
			strcpy(number + 4, "???");
		else {
			number[4] = binary2digit(((min1 & 0x3ff) / 100) + 1);
			number[5] = binary2digit((((min1 & 0x3ff) / 10) % 10) + 1);
			number[6] = binary2digit(((min1 & 0x3ff) % 10) + 1);
		}
	}
	number[7] = '\0';

	return number;
}

const char *amps_min2number(uint32_t min1, uint16_t min2)
{
	static char number[11];

	sprintf(number, "%s%s", amps_min22number(min2), amps_min12number(min1));

	return number;
}

/* encode ESN */
void amps_encode_esn(uint32_t *esn, uint8_t mfr, uint32_t serial)
{
	*esn = (((uint32_t)mfr) << 24) | (serial & 0xffffff);
}

/* decode ESN */
void amps_decode_esn(uint32_t esn, uint8_t *mfr, uint32_t *serial)
{
	*mfr = esn >> 24;
	*serial = esn & 0xffffff;
}

const char *amps_scm(uint8_t scm)
{
	static char text[64];

	sprintf(text, "Class %d / %sontinuous / %d MHz", ((scm & 16) >> 2) + (scm & 3) + 1, (scm & 4) ? "Disc" : "C", (scm & 8) ? 25 : 20);

	return text;
}

/* Power level names per TIA/EIA-553-A Table 2.1.2-1 (Class I mobile) */
static const char *power_level_names[] = {
	"4.0W (max)",	/* 0 */
	"1.6W",		/* 1 */
	"630mW",	/* 2 */
	"250mW",	/* 3 */
	"100mW",	/* 4 */
	"40mW",		/* 5 */
	"16mW",		/* 6 */
	"6.3mW (min)"	/* 7 */
};

const char *amps_power_level_name(int level)
{
	if (level < 0 || level > 7)
		return "invalid";
	return power_level_names[level];
}

static const char *amps_mpci(uint8_t mpci)
{
	switch (mpci) {
	case 0:
		return "TIA/EIA-553 or IS-54A mobile station";
	case 1:
		return "TIA/EIA-627 dual-mode mobile station";
	case 2:
		return "reserved (see TIA/EIA IS-95)";
	case 3:
		return "EIATIA/EIA-136 dual-mode mobile station";
	default:
		return "MPCI INVALID, PLEASE FIX!";
	}

}

static const char *amps_state_name(enum amps_state state)
{
	static char invalid[16];

	switch (state) {
	case STATE_NULL:
		return "(NULL)";
	case STATE_IDLE:
		return "IDLE";
	case STATE_BUSY:
		return "BUSY";
	}

	sprintf(invalid, "invalid(%d)", state);
	return invalid;
}

void amps_display_status(void)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans;

	display_status_start();
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		display_status_channel(amps->sender.kanal, chan_type_short_name(amps->chan_type), amps_state_name(amps->state));
		for (trans = amps->trans_list; trans; trans = trans->next)
			display_status_subscriber(amps_min2number(trans->min1, trans->min2), trans_short_state_name(trans->state));
	}
	display_status_end();
}

static void amps_new_state(amps_t *amps, enum amps_state new_state)
{
	if (amps->state == new_state)
		return;
	LOGP_CHAN(DAMPS, LOGL_DEBUG, "State change: %s -> %s\n", amps_state_name(amps->state), amps_state_name(new_state));
	amps->state = new_state;
	amps_display_status();
}

/* Forward declaration for flash timer callback */
static void flash_timer_callback(void *data);

static struct amps_channels {
	enum amps_chan_type chan_type;
	const char *short_name;
	const char *long_name;
} amps_channels[] = {
	{ CHAN_TYPE_CC,		"CC",	"control channel" },
	{ CHAN_TYPE_CC,		"PC",	"paging channel" },
	{ CHAN_TYPE_CC_PC,	"CC/PC","combined control & paging channel" },
	{ CHAN_TYPE_VC,		"VC",	"voice channel" },
	{ CHAN_TYPE_CC_PC_VC,	"CC/PC/VC","combined control & paging & voice channel" },
	{ 0, NULL, NULL }
};

void amps_channel_list(void)
{
	int i;

	printf("Type\t\tDescription\n");
	printf("------------------------------------------------------------------------\n");
	for (i = 0; amps_channels[i].long_name; i++)
		printf("%s%s\t%s\n", amps_channels[i].short_name, (strlen(amps_channels[i].short_name) >= 8) ? "" : "\t", amps_channels[i].long_name);
}

int amps_channel_by_short_name(const char *short_name)
{
	int i;

	for (i = 0; amps_channels[i].short_name; i++) {
		if (!strcasecmp(amps_channels[i].short_name, short_name)) {
			LOGP(DAMPS, LOGL_INFO, "Selecting channel '%s' = %s\n", amps_channels[i].short_name, amps_channels[i].long_name);
			return amps_channels[i].chan_type;
		}
	}

	return -1;
}

const char *chan_type_short_name(enum amps_chan_type chan_type)
{
	int i;

	for (i = 0; amps_channels[i].short_name; i++) {
		if (amps_channels[i].chan_type == chan_type)
			return amps_channels[i].short_name;
	}

	return "invalid";
}

const char *chan_type_long_name(enum amps_chan_type chan_type)
{
	int i;

	for (i = 0; amps_channels[i].long_name; i++) {
		if (amps_channels[i].chan_type == chan_type)
			return amps_channels[i].long_name;
	}

	return "invalid";
}

static amps_t *search_channel(int channel)
{
	sender_t *sender;
	amps_t *amps;

	for (sender = sender_head; sender; sender = sender->next) {
		if (atoi(sender->kanal) != channel)
			continue;
		amps = (amps_t *) sender;
		if (amps->state == STATE_IDLE)
			return amps;
	}

	return NULL;
}

static amps_t *search_free_vc(void)
{
	sender_t *sender;
	amps_t *amps, *cc_pc_vc = NULL;

	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		if (amps->state != STATE_IDLE)
			continue;
		/* return first free voice channel */
		if (amps->chan_type == CHAN_TYPE_VC)
			return amps;
		/* remember combined voice/control/paging channel as second alternative */
		if (amps->chan_type == CHAN_TYPE_CC_PC_VC)
			cc_pc_vc = amps;
	}

	return cc_pc_vc;
}

static amps_t *search_pc(void)
{
	sender_t *sender;
	amps_t *amps;

	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		if (amps->state != STATE_IDLE)
			continue;
		if (amps->chan_type == CHAN_TYPE_PC) {
			LOGP(DAMPS, LOGL_DEBUG, "DEBUG: search_pc() found PC channel %s\n", amps->sender.kanal);
			return amps;
		}
		if (amps->chan_type == CHAN_TYPE_CC_PC) {
			LOGP(DAMPS, LOGL_DEBUG, "DEBUG: search_pc() found CC_PC channel %s\n", amps->sender.kanal);
			return amps;
		}
		if (amps->chan_type == CHAN_TYPE_CC_PC_VC) {
			LOGP(DAMPS, LOGL_DEBUG, "DEBUG: search_pc() found CC_PC_VC channel %s\n", amps->sender.kanal);
			return amps;
		}
	}

	LOGP(DAMPS, LOGL_DEBUG, "DEBUG: search_pc() found NO available paging channel\n");
	return NULL;
}

/* Create transceiver instance and link to a list. */
int amps_create(const char *kanal, enum amps_chan_type chan_type, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int pre_emphasis, int de_emphasis, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, amps_si *si, uint16_t sid, uint8_t sat, int polarity, int send_callerid, int tolerant, int loopback)
{
	sender_t *sender;
	amps_t *amps;
	int rc;
	enum amps_chan_type ct;
	const char *band;

	/* check for channel number */
	if (amps_channel2freq(atoi(kanal), 0) == 0) {
		LOGP(DAMPS, LOGL_ERROR, "Channel number %s invalid.\n", kanal);
		if (jtacs)
			LOGP(DAMPS, LOGL_ERROR, "Try an even channel number, like 440.\n");
		return -EINVAL;
	}

	/* no paging channel (without control channel) support */
	if (chan_type == CHAN_TYPE_PC) {
		LOGP(DAMPS, LOGL_ERROR, "Dedicated paging channel currently not supported. Please select CC/PC or CC/PC/VC instead.\n");
		return -EINVAL;
	}

	/* check if there is only one paging channel */
	if (chan_type == CHAN_TYPE_PC || chan_type == CHAN_TYPE_CC_PC || chan_type == CHAN_TYPE_CC_PC_VC) {
		for (sender = sender_head; sender; sender = sender->next) {
			amps = (amps_t *)sender;
			if (amps->chan_type == CHAN_TYPE_PC || amps->chan_type == CHAN_TYPE_CC_PC || amps->chan_type == CHAN_TYPE_CC_PC_VC) {
				LOGP(DAMPS, LOGL_ERROR, "Only one paging channel is currently supported. Please check your channel types.\n");
				return -EINVAL;
			}
		}
	}

	/* check if channel type matches channel number */
	ct = amps_channel2type(atoi(kanal));
	if (ct == CHAN_TYPE_CC && chan_type != CHAN_TYPE_PC && chan_type != CHAN_TYPE_CC_PC && chan_type != CHAN_TYPE_CC_PC_VC) {
		LOGP(DAMPS, LOGL_NOTICE, "Channel number %s belongs to a control channel, but your channel type '%s' requires it to be on a voice channel number. Some phones may reject this, but all my phones don't.\n", kanal, chan_type_long_name(chan_type));
	}
	if (ct == CHAN_TYPE_VC && chan_type != CHAN_TYPE_VC) {
		LOGP(DAMPS, LOGL_ERROR, "Channel number %s belongs to a voice channel, but your channel type '%s' requires it to be on a control channel number. Please use correct channel.\n", kanal, chan_type_long_name(chan_type));
		return -EINVAL;
	}
	/* only even channels */
	if (jtacs && chan_type != CHAN_TYPE_VC && (atoi(kanal) & 1)) {
		LOGP(DAMPS, LOGL_ERROR, "Control channel on JTACS system seem not to work with odd channel numbers. Please use even channel number.\n");
		return -EINVAL;
	}

	/* check if sid machtes channel band */
	band = amps_channel2band(atoi(kanal));
	if (band[0] == 'A' && (sid & 1) == 0 && chan_type != CHAN_TYPE_VC) {
		LOGP(DAMPS, LOGL_ERROR, "Channel number %s belongs to system A, but your %s %d is even and belongs to system B. Please give odd %s.\n", kanal, (!tacs) ? "SID" : "AID", sid, (!tacs) ? "SID" : "AID");
		return -EINVAL;
	}
	if (band[0] == 'B' && (sid & 1) == 1 && chan_type != CHAN_TYPE_VC) {
		LOGP(DAMPS, LOGL_ERROR, "Channel number %s belongs to system B, but your %s %d is odd and belongs to system A. Please give even %s.\n", kanal, (!tacs) ? "SID" : "AID", sid, (!tacs) ? "SID" : "AID");
		return -EINVAL;
	}

	/* check if we use combined voice channel hack */
	if (chan_type == CHAN_TYPE_CC_PC_VC) {
		LOGP(DAMPS, LOGL_NOTICE, "You selected '%s'. This is a hack, but the only way to use control channel and voice channel on one transceiver. Some phones may reject this, but all my phones don't.\n", chan_type_long_name(chan_type));
	}

	/* check if we selected a voice channel that i outside 20 MHz band */
	if (chan_type == CHAN_TYPE_VC && atoi(kanal) > 666) {
		LOGP(DAMPS, LOGL_NOTICE, "You selected '%s' on channel #%s. Older phones do not support channels above #666.\n", chan_type_long_name(chan_type), kanal);
	}

	amps = calloc(1, sizeof(amps_t));
	if (!amps) {
		LOGP(DAMPS, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	LOGP(DAMPS, LOGL_DEBUG, "Creating 'AMPS' instance for channel = %s of band %s (sample rate %d).\n", kanal, band, samplerate);

	/* init general part of transceiver */
	/* We disable pre/de-emphasis in sender.c (passing 0, 0) because we handle it 
	 * carefully in dsp.c with correct ordering and limiting. */
	rc = sender_create(&amps->sender, kanal, amps_channel2freq(atoi(kanal), 0), amps_channel2freq(atoi(kanal), 1), device, use_sdr, samplerate, rx_gain, tx_gain, 0, 0, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DAMPS, LOGL_ERROR, "Failed to init transceiver process!\n");
		goto error;
	}

	amps->chan_type = chan_type;
	memcpy(&amps->si, si, sizeof(amps->si));
	amps->sat = sat;
	amps->send_callerid = send_callerid;
	if (polarity < 0)
		amps->flip_polarity = 1;
	amps->pre_emphasis = pre_emphasis;
	amps->de_emphasis = de_emphasis;

	/* the AMPS uses a frequency rage of 300..3000 Hz, but we still use the default low pass filter, which is not too far above */
	rc = init_emphasis(&amps->estate, samplerate, CUT_OFF_EMPHASIS_DEFAULT, CUT_OFF_HIGHPASS_DEFAULT, CUT_OFF_LOWPASS_DEFAULT);
	if (rc < 0)
		goto error;

	/* init separate RX de-emphasis state for 8000 Hz domain */
	rc = init_emphasis(&amps->estate_rx, 8000, CUT_OFF_EMPHASIS_DEFAULT, CUT_OFF_HIGHPASS_DEFAULT, CUT_OFF_LOWPASS_DEFAULT);
	if (rc < 0)
		goto error;

	/* init TX pre-emphasis using correct shelf filter (unity gain at DC, boost highs)
	 * Time constant tau = 1/(2*pi*300) = 530.5us for 300 Hz corner frequency
	 * High corner at 3000 Hz (AMPS audio bandwidth) */
	init_emphasis_fast(&amps->estate_tx_fast, 8000, 530.5e-6, 3000.0);

	/* init TX post-limiter low pass filter (cutoff 3000 Hz, 4th order approx) */
	iir_lowpass_init(&amps->tx_post_filter, 3000.0, samplerate, 4);

	/* RX LPF removed - downsampler already has 3400Hz anti-aliasing filter.
	 * The notch filter below removes SAT tones. Extra LPF was causing
	 * passband gain (+3dB) and distortion from cascaded biquads. */

	/* init RX notch filter for SAT rejection (5970/6000/6030 Hz)
	 * Q=10 gives ~600Hz bandwidth (5700-6300), covering all SAT frequencies.
	 * 2 iterations for deeper notch depth (~40dB rejection). */
	iir_notch_init(&amps->rx_notch_filter, 6000.0, samplerate, 2, 10.0);

	/* init RX HPF for DC and low freq noise (300 Hz) */
	iir_highpass_init(&amps->rx_hpf, 300.0, samplerate, 1);

	/* init voice band filters for expander (at 8000 Hz after downsample)
	 * These limit the bandwidth to 300-3400 Hz before the expander sees it,
	 * so the expander envelope tracks only voice band energy, not wideband noise */
	iir_highpass_init(&amps->rx_voice_hpf, 300.0, 8000, 2);
	iir_lowpass_init(&amps->rx_voice_lpf, 3400.0, 8000, 2);

	/* init audio processing */
	rc = dsp_init_sender(amps, tolerant);
	if (rc < 0) {
		LOGP(DAMPS, LOGL_ERROR, "Failed to init audio processing!\n");
		goto error;
	}

	/* go into idle state */
	amps_go_idle(amps);

#ifdef DEBUG_VC
	uint32_t min1;
	uint16_t min2;
	amps_number2min("1234567890", &min1, &min2);
	transaction_t __attribute__((__unused__)) *trans = create_transaction(amps, TRANS_CALL_MO_ASSIGN, min1, min2, 0, 0, 0, 0, amps->sender.kanal);
//	amps_new_state(amps, STATE_BUSY);
#endif

	LOGP(DAMPS, LOGL_NOTICE, "Created channel #%s (System %s) of type '%s' = %s\n", kanal, band, chan_type_short_name(chan_type), chan_type_long_name(chan_type));

	return 0;

error:
	amps_destroy(&amps->sender);

	return rc;
}

/* Destroy transceiver instance and unlink from list. */
void amps_destroy(sender_t *sender)
{
	amps_t *amps = (amps_t *) sender;
	transaction_t *trans;

	LOGP(DAMPS, LOGL_DEBUG, "Destroying 'AMPS' instance for channel = %s.\n", sender->kanal);

	while ((trans = amps->trans_list)) {
		const char *number = amps_min2number(trans->min1, trans->min2);
		LOGP(DAMPS, LOGL_NOTICE, "Removing pending transaction for subscriber '%s'\n", number);
		destroy_transaction(trans);
	}

	dsp_cleanup_sender(amps);
	sender_destroy(&amps->sender);
	free(amps);
}

/* Abort connection towards mobile station by sending FOCC/FVC pattern. */
void amps_go_idle(amps_t *amps)
{
	int frame_length;

	if (amps->state == STATE_IDLE)
		return;

	if (amps->trans_list) {
		LOGP(DAMPS, LOGL_ERROR, "Releasing but still having transaction, please fix!\n");
		if (amps->trans_list->callref)
			call_up_release(amps->trans_list->callref, CAUSE_NORMAL);
		destroy_transaction(amps->trans_list);
	}

	amps_new_state(amps, STATE_IDLE);

	if (amps->chan_type != CHAN_TYPE_VC) {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Entering IDLE state, sending Overhead/Filler frames on %s.\n", chan_type_long_name(amps->chan_type));
		if (amps->sender.loopback)
			frame_length = 441; /* bits after sync (FOCC) */
		else
			frame_length = 247; /* bits after sync (RECC) */
		amps_set_dsp_mode(amps, DSP_MODE_FRAME_RX_FRAME_TX, frame_length);
	} else {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Entering IDLE state (sending silence / no RF) on %s.\n", chan_type_long_name(amps->chan_type));
		amps_set_dsp_mode(amps, DSP_MODE_OFF, 0);
	}
}

/* Abort connection towards mobile station by sending FOCC/FVC pattern. */
static void amps_release(transaction_t *trans, uint8_t cause)
{
	amps_t *amps = trans->amps;
	const char *callerid = amps_min2number(trans->min1, trans->min2);

	LOGP_CHAN(DAMPS, LOGL_INFO, "Initiating Release (Order 3, ORDQ=0) to mobile %s (cause=%d)\n", callerid, cause);

	osmo_timer_del(&trans->timer);
	osmo_timer_schedule(&trans->timer, RELEASE_TIMER);
	trans_new_state(trans, TRANS_CALL_RELEASE);
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for standard Release */
	trans->order = 3;      /* Order 3 = Release */
	/* release towards call control */
	if (trans->callref) {
		call_up_release(trans->callref, cause);
		trans->callref = 0;
	}
	/* change DSP mode to transmit release */
	if (amps->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX || amps->dsp_mode == DSP_MODE_AUDIO_RX_SILENCE_TX || amps->dsp_mode == DSP_MODE_OFF)
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
}

/*
 * receive signaling
 */
void amps_rx_signaling_tone(amps_t *amps, int tone, double quality)
{
	transaction_t *trans = amps->trans_list;
	if (trans == NULL) {
		LOGP_CHAN(DAMPS, LOGL_ERROR, "Signaling Tone without transaction, please fix!\n");
		return;
	}
	
	if (tone)
		LOGP_CHAN(DAMPS, LOGL_INFO, "Detected Signaling Tone with quality=%.0f.\n", quality * 100.0);
	else
		LOGP_CHAN(DAMPS, LOGL_INFO, "Lost Signaling Tone signal\n");

	switch (trans->state) {
	case TRANS_CALL_MO_ASSIGN_CONFIRM: // should not happen
	case TRANS_CALL:
		if (tone) {
			/* ST detected during active call */
			if (trans->st_start_time == 0.0) {
				/* Start timing the ST duration */
				LOGP_CHAN(DAMPS, LOGL_DEBUG, "Signaling Tone detected in active call - starting timer\n");
				trans->st_start_time = get_time();
				/* Schedule timer for Release condition (1.8s) */
				if (trans->sat_detected)
					osmo_timer_schedule(&trans->timer, ST_RELEASE_TIME);
			}
		} else {
			/* ST Lost during active call */
			if (trans->st_start_time != 0.0) {
				double duration = get_time() - trans->st_start_time;
				
				LOGP_CHAN(DAMPS, LOGL_DEBUG, "Signaling Tone duration: %.4f s\n", duration);
				
				trans->st_start_time = 0.0;
				osmo_timer_del(&trans->timer);
				
				if (duration >= ST_FLASH_MIN && duration <= ST_FLASH_MAX) {
					LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** Hook-Flash Detected (duration %.4f s) ***\n", duration);
					if (trans->callref)
						call_up_flash(trans->callref);
					/* Start timer to auto-send Order 8 after ~1 second
					 * This gives user time to dial digits on the mobile
					 */
					trans->flash_time = get_time();
					osmo_timer_setup(&trans->flash_timer, flash_timer_callback, trans);
					osmo_timer_schedule(&trans->flash_timer, 1, 0);  /* 1 second delay */
					LOGP_CHAN(DAMPS, LOGL_INFO, "Flash timer started - will send Order 8 in 1 second\n");
				} else if (duration > ST_FLASH_MAX) {
					LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** ST Duration (%.4f s) > Flash Max - Interpreting as Release ***\n", duration);
					if (trans->callref)
						call_up_release(trans->callref, CAUSE_NORMAL);
					destroy_transaction(trans);
					amps_go_idle(amps);
				} else {
					LOGP_CHAN(DAMPS, LOGL_NOTICE, "ST Pulse too short (%.4f s) - ignored\n", duration);
				}
			}
		}
		break;
	case TRANS_CALL_RELEASE:
	case TRANS_CALL_RELEASE_SEND:
		/* also loosing singaling tone indicates release confirm (after alerting) */
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "ST detected during CALL/RELEASE state - releasing call (callref=%d)\n", trans->callref);
		osmo_timer_del(&trans->timer);
		if (trans->callref)
			call_up_release(trans->callref, CAUSE_NORMAL);
		destroy_transaction(trans);
		amps_go_idle(amps);
		break;
	case TRANS_CALL_MT_ASSIGN_CONFIRM: // should not happen
	case TRANS_CALL_MT_ALERT: // should not happen
	case TRANS_CALL_MT_ALERT_SEND: // should not happen
	case TRANS_CALL_MT_ALERT_CONFIRM:
		if (tone) {
			osmo_timer_del(&trans->timer);
			call_up_alerting(trans->callref);
			amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
			trans_new_state(trans, TRANS_CALL_MT_ANSWER_WAIT);
			osmo_timer_schedule(&trans->timer, ANSWER_TO);
		}
		break;
	case TRANS_CALL_MAINTENANCE_SEND:
		/* Maintenance order ST confirmation
		 * Per TIA/EIA-553-A: Mobile turns on ST for 500ms to confirm
		 * Unlike Alert, Maintenance doesn't ring - just confirms operation
		 */
		if (tone) {
			osmo_timer_del(&trans->timer);
			LOGP_CHAN(DAMPS, LOGL_INFO, "Maintenance order confirmed by ST - mobile station operational\n");
			trans_new_state(trans, TRANS_CALL);
			amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		}
		break;
	case TRANS_SILENT_PAGE_MAINTENANCE_SEND:
		/* Silent Page: Maintenance order ST confirmation
		 * Per TIA/EIA-553-A Section 2.6.4.4:
		 * - Mobile turns on ST for 500ms to confirm
		 * - Mobile then enters "Waiting for Answer Task" with 65-second timer
		 * - Mobile continues sending SAT
		 * 
		 * After ST confirmation, we send Release to cleanly terminate.
		 * This completes the silent page test successfully.
		 */
		if (tone) {
			osmo_timer_del(&trans->timer);
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** Silent Page SUCCESS: Mobile '%s' confirmed operational (ST received) ***\n",
				amps_min2number(trans->min1, trans->min2));
			LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: sending Release to terminate test\n");
			/* Send Release order to cleanly terminate */
			trans->chan = 0;
			trans->msg_type = 0;
			trans->ordq = 0;
			trans->order = 3;  /* Release */
			trans_new_state(trans, TRANS_CALL_RELEASE);
			amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
		}
		break;
	case TRANS_CALL_HANDOFF_SEND:
		/* Handoff ST confirmation
		 * Per TIA/EIA-553-A Section 2.6.4.4:
		 * - Mobile turns on ST for 50ms to confirm handoff
		 * - Mobile then turns off transmitter and tunes to new channel
		 * 
		 * After ST confirmation, we move the transaction to the new channel.
		 * Note: Target channel was already prepared (made BUSY, transmitting SAT)
		 * when the handoff command was issued.
		 */
		if (tone) {
			sender_t *sender;
			amps_t *target_vc = NULL;
			int new_channel = trans->handoff_channel;
			int old_channel = atoi(amps->sender.kanal);
			
			osmo_timer_del(&trans->timer);
			LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Handoff Step 3: ST confirmed, moving to channel %d\n", new_channel);
			
			/* Find the target voice channel (it's already BUSY, so search by number) */
			for (sender = sender_head; sender; sender = sender->next) {
				if (atoi(sender->kanal) == new_channel) {
					target_vc = (amps_t *) sender;
					break;
				}
			}
			
			if (!target_vc) {
				LOGP_CHAN(DAMPS, LOGL_ERROR, "Handoff FAILED: channel %d not found\n", new_channel);
				/* Return to call state on current channel */
				trans->chan = 0;
				trans->handoff_channel = 0;
				trans->handoff_scc = -1;
				trans_new_state(trans, TRANS_CALL);
				amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
				return;
			}
			
			LOGP_CHAN(DAMPS, LOGL_INFO, "    Freeing old channel %d\n", old_channel);
			
			/* Move transaction to new channel BEFORE making old channel idle
			 * This prevents the transaction from being destroyed when old channel goes idle
			 */
			unlink_transaction(trans);
			link_transaction(trans, target_vc);
			
			/* Now make old channel idle (transaction is already moved) */
			amps_go_idle(amps);
			
			LOGP(DAMPS, LOGL_INFO, "(chan %d) Handoff Step 4: Call active on new channel\n", new_channel);
			/* Target channel is already BUSY (prepared during handoff initiation) */
			
			/* Clear handoff fields */
			trans->chan = 0;
			trans->handoff_channel = 0;
			trans->handoff_scc = -1;
			
			/* Reset SAT detection state for new channel */
			trans->sat_detected = 0;
			
			/* Return to call state on new channel */
			trans_new_state(trans, TRANS_CALL);
			amps_set_dsp_mode(target_vc, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
			
			LOGP(DAMPS, LOGL_NOTICE, "(chan %d) <<< Handoff COMPLETE: %d -> %d\n", new_channel, old_channel, new_channel);
		}
		break;
	case TRANS_CALL_MT_ANSWER_WAIT:
		if (!tone) {
			/* Per TIA/EIA-553-A: Answer is indicated by (SAT=1, ST=0) transition.
			 * Only interpret ST loss as answer if SAT is present.
			 * This prevents false answer detection from signal glitches or
			 * non-compliant phones that briefly drop ST during alerting. */
			if (trans->sat_detected) {
				LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** ST LOST during MT_ANSWER_WAIT with SAT present - phone answered (callref=%d) ***\n", 
					trans->callref);
				osmo_timer_del(&trans->timer);
				if (!trans->sat_detected)
					osmo_timer_schedule(&trans->timer, SAT_TO1);
				call_up_answer(trans->callref, amps_min2number(trans->min1, trans->min2));
				trans_new_state(trans, TRANS_CALL);
			} else {
				/* ST lost but SAT not detected - likely signal issue or non-compliant phone.
				 * Keep waiting for answer. The ANSWER_TO timer will handle timeout. */
				LOGP_CHAN(DAMPS, LOGL_NOTICE, "ST lost during MT_ANSWER_WAIT but SAT not detected - ignoring (callref=%d)\n",
					trans->callref);
			}
		} else {
			LOGP_CHAN(DAMPS, LOGL_DEBUG, "ST still present during MT_ANSWER_WAIT - phone still ringing (callref=%d)\n", trans->callref);
		}
		break;
	default:
		LOGP_CHAN(DAMPS, LOGL_ERROR, "Signaling Tone without active call, please fix!\n");
	}
}

static void adjust_vmac(transaction_t *trans)
{
	amps_t *amps = trans->amps;
	if (trans->sat_level_avg > vmac_level_high) {
		if (trans->current_vmac < 7) { /* 7 is max attenuation */
			trans->current_vmac++;
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "SAT too strong (%.1f%%), reducing power (VMAC %d->%d)\n", 
				trans->sat_level_avg * 100.0, trans->current_vmac-1, trans->current_vmac);
			if (trans->state == TRANS_CALL) {
				trans_new_state(trans, TRANS_CALL_CHANGE_POWER);
				amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
			}
		}
	} else if (trans->sat_level_avg < vmac_level_low) {
		if (trans->current_vmac > trans->max_vmac) { /* Don't go below configured limit (allow more power) */
			trans->current_vmac--;
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "SAT too weak (%.1f%%), increasing power (VMAC %d->%d)\n", 
				trans->sat_level_avg * 100.0, trans->current_vmac+1, trans->current_vmac);
			if (trans->state == TRANS_CALL) {
				trans_new_state(trans, TRANS_CALL_CHANGE_POWER);
				amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
			}
		}
	}
}

void amps_rx_sat(amps_t *amps, int tone, double quality)
{
	transaction_t *trans = amps->trans_list;
	if (trans == NULL) {
		LOGP_CHAN(DAMPS, LOGL_ERROR, "SAT signal without transaction, please fix!\n");
		return;
	}

	/* irgnoring SAT loss on release */
	if (trans->state == TRANS_CALL_RELEASE
	 || trans->state == TRANS_CALL_RELEASE_SEND)
		return;

	/* only SAT with these states */
	if (trans->state != TRANS_CALL_MO_ASSIGN_CONFIRM
	 && trans->state != TRANS_CALL_MT_ASSIGN_CONFIRM
	 && trans->state != TRANS_CALL_MT_ALERT
	 && trans->state != TRANS_CALL_MT_ALERT_SEND
	 && trans->state != TRANS_CALL_MT_ALERT_CONFIRM
	 && trans->state != TRANS_CALL_MT_ANSWER_WAIT
	 && trans->state != TRANS_CALL_CHANGE_POWER
	 && trans->state != TRANS_CALL_CHANGE_POWER_SEND
	 && trans->state != TRANS_CALL_FLASH_INFO
	 && trans->state != TRANS_CALL_FLASH_INFO_SEND
	 && trans->state != TRANS_CALL_PCI_QUERY
	 && trans->state != TRANS_CALL_PCI_QUERY_SEND
	 && trans->state != TRANS_CALL_AUDIT
	 && trans->state != TRANS_CALL_AUDIT_SEND
	 && trans->state != TRANS_CALL_MWI
	 && trans->state != TRANS_CALL_MWI_SEND
	 && trans->state != TRANS_CALL_DIGITS_REQUEST
	 && trans->state != TRANS_CALL_DIGITS_REQUEST_SEND
	 && trans->state != TRANS_CALL_ESN_REQUEST
	 && trans->state != TRANS_CALL_ESN_REQUEST_SEND
	 && trans->state != TRANS_CALL_LOCAL_CONTROL
	 && trans->state != TRANS_CALL_LOCAL_CONTROL_SEND
	 && trans->state != TRANS_CALL_DISABLE_DTMF
	 && trans->state != TRANS_CALL_DISABLE_DTMF_SEND
	 && trans->state != TRANS_CALL_HANDOFF
	 && trans->state != TRANS_CALL_HANDOFF_SEND
	 && trans->state != TRANS_SILENT_PAGE_ASSIGN_CONFIRM
	 && trans->state != TRANS_SILENT_PAGE_MAINTENANCE
	 && trans->state != TRANS_SILENT_PAGE_MAINTENANCE_SEND
	 && trans->state != TRANS_CALL) {
		LOGP_CHAN(DAMPS, LOGL_ERROR, "SAT signal without active call, please fix!\n");
		return;
	}

	/* Check grace period - Ignore SAT status updates during power change / blank-and-burst */
	if (trans->vmac_grace_count > 0) {
		trans->vmac_grace_count--;
		/* Update average but with very low weight to avoid dragging it down */
		// if (tone) trans->sat_level_avg = (trans->sat_level_avg * 0.99) + (quality * 0.01);
		return;
	}

	if (tone) {
		if (!trans->sat_detected)
			LOGP(DAMPS, LOGL_INFO, "Detected SAT signal with quality=%.0f.\n", quality * 100.0);
		trans->sat_detected = 1;
		
		/* Power Control Logic */
		if (trans->state == TRANS_CALL && vmac_enable) {
			/* Initialize average on first sample */
			if (trans->sat_level_avg == 0.0)
				trans->sat_level_avg = quality;
			
			/* Update running average */
			trans->sat_level_avg = (trans->sat_level_avg * 0.9) + (quality * 0.1);
			trans->vmac_adjust_count++;

			/* Check if time to adjust */
			if (trans->vmac_adjust_count >= VMAC_ADJUST_INTERVAL) {
				adjust_vmac(trans);
				trans->vmac_adjust_count = 0;
			}
		}
	} else {
		LOGP(DAMPS, LOGL_INFO, "Lost SAT signal\n");
		trans->sat_detected = 0;
	}

	/* initial SAT received */
	if (tone && trans->state == TRANS_CALL_MO_ASSIGN_CONFIRM) {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Confirm from mobile (SAT) received\n");
		osmo_timer_del(&trans->timer);
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
	}
	if (tone && trans->state == TRANS_CALL_MT_ASSIGN_CONFIRM) {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Confirm from mobile (SAT) received\n");
		osmo_timer_del(&trans->timer);
		trans->alert_retry = 1;
		trans_new_state(trans, TRANS_CALL_MT_ALERT);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
	}
	/* Silent Page: SAT confirmation after channel assignment */
	if (tone && trans->state == TRANS_SILENT_PAGE_ASSIGN_CONFIRM) {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: SAT confirmed, sending Maintenance order\n");
		osmo_timer_del(&trans->timer);
		trans_new_state(trans, TRANS_SILENT_PAGE_MAINTENANCE);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
	}

	if (tone) {
		osmo_timer_del(&trans->timer);
	} else {
		if (!trans->dtx)
			osmo_timer_schedule(&trans->timer, SAT_TO2);
		else
			osmo_timer_del(&trans->timer);
	}

	if (amps->sender.loopback)
		return;
}

/*
 * SAT frequency mismatch callback - called when mobile transponds a different
 * SAT frequency than expected (potential handoff candidate).
 *
 * Per TIA/EIA-553-A, if SAT does not match SCCr (requested SCC), the base
 * station should enable fade timing. This callback provides the information
 * needed to initiate a handoff to the correct cell.
 */
void amps_rx_sat_mismatch(amps_t *amps, enum sat_state expected, enum sat_state detected)
{
	transaction_t *trans = amps->trans_list;

	if (trans == NULL) {
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "SAT mismatch without transaction, ignoring\n");
		return;
	}

	/* Only process mismatch during active call states */
	if (trans->state != TRANS_CALL_MO_ASSIGN_CONFIRM
	 && trans->state != TRANS_CALL_MT_ASSIGN_CONFIRM
	 && trans->state != TRANS_CALL_MT_ALERT
	 && trans->state != TRANS_CALL_MT_ALERT_SEND
	 && trans->state != TRANS_CALL_MT_ALERT_CONFIRM
	 && trans->state != TRANS_CALL_MT_ANSWER_WAIT
	 && trans->state != TRANS_CALL) {
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "SAT mismatch in non-call state, ignoring\n");
		return;
	}

	/* Ignore during release */
	if (trans->state == TRANS_CALL_RELEASE
	 || trans->state == TRANS_CALL_RELEASE_SEND)
		return;

	LOGP_CHAN(DAMPS, LOGL_NOTICE, "SAT mismatch detected: expected %s (SCC %d), got %s\n",
	          sat_state_name(expected),
	          amps->sat,
	          sat_state_name(detected));

	/*
	 * TODO: Implement handoff logic here
	 *
	 * Per TIA/EIA-553-A section 2.6.2.3:
	 * - If SAT does not match SCCr, enable fade timing
	 * - The mobile may have locked onto a different cell's SAT
	 * - This indicates potential handoff candidate
	 *
	 * Possible actions:
	 * 1. Start fade timer (if not already running from SAT loss)
	 * 2. Notify higher layer (e.g., MSC) for handoff decision
	 * 3. Log for inter-cell coordination
	 *
	 * For now, we just log the event. Full handoff implementation
	 * would require inter-cell coordination protocol.
	 */

	/* Start fade timer if not already set (treat as potential SAT loss) */
	if (!trans->dtx && trans->state == TRANS_CALL) {
		if (!osmo_timer_pending(&trans->timer)) {
			LOGP_CHAN(DAMPS, LOGL_INFO, "Starting fade timer due to SAT mismatch\n");
			osmo_timer_schedule(&trans->timer, SAT_TO2);
		}
	}
}


void amps_rx_recc(amps_t *amps, uint8_t t, uint8_t scm, uint8_t mpci, uint32_t esn, uint32_t min1, uint16_t min2, uint8_t msg_type, uint8_t ordq, uint8_t order, const char *dialing, uint8_t mspc, uint8_t mscap)
{
	amps_t *vc;
	transaction_t *trans;
	const char *callerid = amps_min2number(min1, min2);
	const char *carrier = NULL, *country = NULL, *national_number = NULL;

	/* Special handling for RVC Called Address response (Order 8 response)
	 * This is received on voice channel (STATE_BUSY) when we requested digits.
	 * T=0 on RVC means Called Address message.
	 * Note: We return to TRANS_CALL immediately after sending Order 8,
	 * so the response is handled asynchronously here.
	 */
	if (amps->state == STATE_BUSY && t == 0) {
		trans = amps->trans_list;
		if (trans && trans->state == TRANS_CALL) {
			/* This is the response to our Send Called-Address order */
			if (dialing && dialing[0]) {
				LOGP_CHAN(DAMPS, LOGL_INFO, "Received Called Address response: digits='%s'\n", dialing);
			} else {
				LOGP_CHAN(DAMPS, LOGL_INFO, "Received Called Address response: no digits (abbreviated response)\n");
			}
			osmo_timer_del(&trans->timer);  /* Cancel timeout if any */
			/* Already in TRANS_CALL, just log the digits */
			return;
		}
	}

	/* check if we are busy, so we ignore all signaling */
	if (amps->state == STATE_BUSY) {
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "Ignoring RECC messages from phone while using this channel for voice.\n");
		return;
	}

	/* Log Protocol Capability if received */
	if (mspc || mscap) {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Protocol Capability: MSPC=%d (%s), MSCAP=%d (%s)\n",
			mspc, ie_mspc(mspc), mscap, ie_mscap(mscap));
	}

	if (order == 13 && (ordq == 0 || ordq == 1 || ordq == 2 || ordq == 3) && msg_type == 0) {
		/* Per TIA/EIA-553-A Table 3.7.1-1:
		 * Order 13 (01101) = Registration without Authentication Word C
		 * ORDQ bits:
		 *   Bit 0: Make whereabouts known (1) or not (0)
		 *   Bit 1: Autonomous (1) or Non-autonomous (0)
		 * ORDQ=0 (000): Non-autonomous - Do not make whereabouts known
		 * ORDQ=1 (001): Non-autonomous - Make whereabouts known
		 * ORDQ=2 (010): Autonomous - Do not make whereabouts known
		 * ORDQ=3 (011): Autonomous - Make whereabouts known
		 * MSG_TYPE=0: Normal registration
		 * MSG_TYPE=1: Power-Down registration (only with ORDQ=3)
		 *
		 * Human-readable meanings:
		 * - Autonomous: Phone decided to register on its own (power-up, timer, location change)
		 * - Non-autonomous: Network requested the phone to register
		 * - Whereabouts known: Phone wants to receive incoming calls (normal)
		 * - Whereabouts hidden: Phone registers but doesn't want to be reachable (privacy)
		 * - ORDQ=3 is the normal power-up registration (phone-initiated, wants calls)
		 */
		const char *reg_type;
		const char *whereabouts = (ordq & 1) ? "whereabouts known" : "whereabouts hidden";
		const char *auto_type = (ordq & 2) ? "Autonomous" : "Non-autonomous";
		const char *human_readable = (ordq & 2) ? "Phone registering on its own (power-up, timer, location change)" : "Network requested the phone to register";
		const char *human_reachable = (ordq & 1) ? " Phone wants to receive incoming calls (normal)" : "Phone registers but doesn't want to be reachable (privacy)";
		reg_type = auto_type;
		LOGP_CHAN(DAMPS, LOGL_INFO, "Registration (%s, %s, ORDQ=%d) %s (ESN = %s, %s, %s)%s\n", 
			reg_type, whereabouts, ordq, callerid, esn_to_string(esn), amps_scm(scm), amps_mpci(mpci), (mspc || mscap) ? " +PCI" : "");
		LOGP_CHAN(DAMPS, LOGL_INFO, " -> %s, %s\n", human_readable, human_reachable);
_register:
		numbering(callerid, &carrier, &country, &national_number);
		if (carrier)
			LOGP_CHAN(DAMPS, LOGL_INFO, " -> Home carrier: %s\n", carrier);
		if (country)
			LOGP_CHAN(DAMPS, LOGL_INFO, " -> Home country: %s\n", country);
		if (national_number)
			LOGP_CHAN(DAMPS, LOGL_INFO, " -> Home number: %s\n", national_number);
		trans = create_transaction(amps, TRANS_REGISTER_ACK, min1, min2, esn, msg_type, ordq, order, 0);
		if (!trans) {
			LOGP(DAMPS, LOGL_ERROR, "Failed to create transaction\n");
			return;
		}
		console_inscription(callerid);
	} else
	if (order == 13 && ordq == 3 && msg_type == 1) {
		/* Per TIA/EIA-553-A Table 3.7.1-1:
		 * Order 13, ORDQ=3, MSG_TYPE=1 = Autonomous Registration - Power Down
		 * This is sent by the mobile just before it powers off.
		 * Phone is saying goodbye - it will no longer be reachable.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Registration (Power-Down, Autonomous, ORDQ=3, MSG_TYPE=1) %s (ESN = %s, %s, %s)\n", callerid, esn_to_string(esn), amps_scm(scm), amps_mpci(mpci));
		LOGP_CHAN(DAMPS, LOGL_INFO, " -> phone shutting down, no longer reachable\n");
		goto _register;
	} else
	if (order == 0 && ordq == 0 && msg_type == 0) {
		/* Check T field to distinguish Origination vs Paging Reply */
		if (t == 1 && dialing) {
			/* Mobile Origination (MO call) - T=1 with dialed digits */
			LOGP_CHAN(DAMPS, LOGL_INFO, "Call %s -> %s (ESN = %s, %s, %s)\n", callerid, dialing, esn_to_string(esn), amps_scm(scm), amps_mpci(mpci));
		} else if (t == 0) {
			/* Paging Reply (MT call) - T=0 */
			LOGP_CHAN(DAMPS, LOGL_INFO, "Paging reply %s (ESN = %s, %s, %s)\n", callerid, esn_to_string(esn), amps_scm(scm), amps_mpci(mpci));
		} else {
			/* Unexpected: T=1 but no dialing digits */
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "ORDER=0 with T=%d %s (ESN = %s, %s, %s)\n", t, dialing ? "with digits" : "without digits", esn_to_string(esn), amps_scm(scm), amps_mpci(mpci));
		}
		
		/* DEBUG: Log transaction search details */
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "DEBUG: Searching for transaction with MIN1=%u MIN2=%u\n", min1, min2);
		
		trans = search_transaction_number(amps, min1, min2);
		
		/* DEBUG: Log search result */
		if (trans) {
			LOGP_CHAN(DAMPS, LOGL_DEBUG, "DEBUG: Found transaction, state=%d MIN1=%u MIN2=%u\n", 
				trans->state, trans->min1, trans->min2);
		} else {
			LOGP_CHAN(DAMPS, LOGL_DEBUG, "DEBUG: Transaction NOT found! Listing all transactions on this channel:\n");
			transaction_t *t_iter = amps->trans_list;
			int count = 0;
			while (t_iter) {
				LOGP_CHAN(DAMPS, LOGL_DEBUG, "DEBUG:   Trans #%d: state=%d MIN1=%u MIN2=%u\n", 
					count++, t_iter->state, t_iter->min1, t_iter->min2);
				t_iter = t_iter->next;
			}
			if (count == 0) {
				LOGP_CHAN(DAMPS, LOGL_DEBUG, "DEBUG:   (no transactions on this channel)\n");
			}
		}
		
		/* For paging replies (T=0), transaction must exist */
		if (t == 0 && !trans && !dialing) {
			LOGP(DAMPS, LOGL_NOTICE, "Paging reply, but call is already gone, rejecting call\n");
			goto reject;
		}
		if (trans && dialing)
			LOGP(DAMPS, LOGL_NOTICE, "There is already a transaction for this phone. Cloning?\n");
		
		/* For MO calls (new transaction), use delayed channel assignment */
		if (!trans && dialing) {
			/* Delayed channel assignment - wait for network PROCEEDING before assigning voice channel.
			 * This allows us to send Reorder/Intercept on FOCC if call fails early.
			 */
			LOGP_CHAN(DAMPS, LOGL_INFO, "MO call to '%s' - waiting for network PROCEEDING\n", dialing);
			trans = create_transaction(amps, TRANS_CALL_MO_WAIT_PROCEED, min1, min2, esn, 0, 0, 0, 0);
			if (!trans) {
				LOGP(DAMPS, LOGL_ERROR, "Failed to create transaction\n");
				return;
			}
			strncpy(trans->dialing, dialing, sizeof(trans->dialing) - 1);
			
			/* Setup call to network immediately */
			char esn_text[16];
			sprintf(esn_text, "%u", esn);
			LOGP(DAMPS, LOGL_INFO, "Setup call to network (waiting for PROCEEDING).\n");
			trans->callref = call_up_setup(callerid, dialing, OSMO_CC_NETWORK_AMPS_ESN, esn_text);
			
			/* Start timeout timer (per TIA/EIA-553-A, max 5 seconds) */
			osmo_timer_schedule(&trans->timer, network_timeout, 0);
		} else if (!trans) {
			/* Paging reply or other - need voice channel */
			vc = search_free_vc();
			if (!vc) {
				LOGP(DAMPS, LOGL_NOTICE, "No free channel, rejecting call\n");
reject:
				if (!trans) {
					trans = create_transaction(amps, TRANS_CALL_REJECT, min1, min2, esn, 0, 0, 3, 0);
					if (!trans) {
						LOGP(DAMPS, LOGL_ERROR, "Failed to create transaction\n");
						return;
					}
				} else {
					trans_new_state(trans, TRANS_CALL_REJECT);
					trans->chan = 0;
					trans->msg_type = 0;
					trans->ordq = 0;
					trans->order = 3;
				}
				return;
			}
			/* This shouldn't happen - paging reply without transaction */
			LOGP(DAMPS, LOGL_ERROR, "Unexpected: no transaction for non-MO call\n");
			return;
		} else {
			/* Existing transaction - paging reply or silent page reply */
			/* Need to find a voice channel for these */
			vc = search_free_vc();
			if (!vc) {
				LOGP(DAMPS, LOGL_NOTICE, "No free channel for paging reply, rejecting\n");
				trans_new_state(trans, TRANS_CALL_REJECT);
				trans->chan = 0;
				trans->msg_type = 0;
				trans->ordq = 0;
				trans->order = 3;
				return;
			}
			/* Check if this is a silent page reply */
			if (trans->state == TRANS_SILENT_PAGE_REPLY) {
				LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: page reply received, assigning voice channel\n");
				trans_new_state(trans, TRANS_SILENT_PAGE_ASSIGN);
				trans->chan = atoi(vc->sender.kanal);
			} else {
				trans_new_state(trans, TRANS_CALL_MT_ASSIGN);
				trans->chan = atoi(vc->sender.kanal);
			}
		}
		/* if we support DTX and also the phone does, we set DTX state of transaction */
		if (amps->si.word2.dtx) {
			if ((scm & 4)) {
				LOGP(DAMPS, LOGL_INFO, " -> Use DTX for this call\n");
				trans->dtx = 1;
			} else
				LOGP(DAMPS, LOGL_INFO, " -> Requested DTX, but not supported by phone\n");
		}
	} else
	if (order == 7 && ordq == 0 && msg_type == 0) {
		/* Audit Order Confirmation - mobile confirms it's still present */
		LOGP_CHAN(DAMPS, LOGL_INFO, "%s from %s (ESN = %s)\n",
			amps_table4_name(msg_type, ordq, order), callerid, esn_to_string(esn));
	} else
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "Unsupported RECC messages: ORDER: %d ORDQ: %d MSG TYPE: %d (See Table 4 of specs.)\n", order, ordq, msg_type);
}

/*
 * call states received from call control
 */

/* Call control starts call towards mobile station. */
int call_down_setup(int callref, const char __attribute__((unused)) *caller_id, enum number_type __attribute__((unused)) caller_type, const char *dialing)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans;
	uint32_t min1;
	uint16_t min2;

	/* 1. split number into area code and number */
	int signal_pitch = mobile_amps_param_pitch;
	int signal_cadence = mobile_amps_param_cadence;
	int pi = mobile_amps_param_present;
	int si = mobile_amps_param_screen;

	if (amps_number2min(dialing, &min1, &min2) < 0) {
		LOGP(DAMPS, LOGL_ERROR, "Invalid dialing number '%s'\n", dialing);
		return -1;
	}

	/* 2. check if the subscriber is attached */
//	if (!find_db(min1, min2)) {
//		LOGP(DAMPS, LOGL_NOTICE, "Outgoing call to not attached subscriber, rejecting!\n");
//		return -CAUSE_OUTOFORDER;
//	}

	/* 3. check if given number is already in a call, return BUSY */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		/* search transaction for this number */
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}
	if (sender) {
		LOGP(DAMPS, LOGL_NOTICE, "Outgoing call to busy number, rejecting!\n");
		return -CAUSE_BUSY;
	}

	/* 4. check if all senders are busy, return NOCHANNEL */
	if (!search_free_vc()) {
		LOGP(DAMPS, LOGL_NOTICE, "Outgoing call, but no free channel, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}

	/* 5. check if we have (currently) no paging channel, return NOCHANNEL */
	amps = search_pc();
	if (!amps) {
		LOGP(DAMPS, LOGL_NOTICE, "Outgoing call, but paging channel (control channel) is currently busy, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Call to mobile station, paging station id '%s'\n", dialing);

	/* 6. trying to page mobile station */
	trans = create_transaction(amps, TRANS_PAGE, min1, min2, 0, 0, 0, 0, 0);
	if (!trans) {
		LOGP(DAMPS, LOGL_ERROR, "Failed to create transaction\n");
		return -CAUSE_TEMPFAIL;
	}
	/* store custom parameters if set */
	if (signal_pitch != -1)
		trans->signal_pitch = signal_pitch;
	if (signal_cadence != -1)
		trans->signal_cadence = signal_cadence;
	if (pi != -1)
		trans->presentation_indicator = pi;
	if (si != -1)
		trans->screening_indicator = si;

	trans->callref = callref;
	trans->page_retry = 1;
	if (caller_type == TYPE_INTERNATIONAL) {
		trans->caller_id[0] = '+';
		strncpy(trans->caller_id + 1, caller_id, sizeof(trans->caller_id) - 2);
	} else
		strncpy(trans->caller_id, caller_id, sizeof(trans->caller_id) - 1);

	return 0;
}

void call_down_answer(int __attribute__((unused)) callref, struct timeval __attribute__((unused)) *tv_meter)
{
}

/* Call control sends PROCEEDING - network accepted the call setup.
 * For delayed channel assignment: Now we can assign the voice channel.
 */
void call_down_proceeding(int callref)
{
	sender_t *sender;
	amps_t *amps;
	amps_t *vc;
	transaction_t *trans;

	LOGP(DAMPS, LOGL_INFO, "Network sent PROCEEDING for call.\n");

	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		/* search transaction for this callref */
		trans = search_transaction_callref(amps, callref);
		if (trans)
			break;
	}
	if (!sender) {
		LOGP(DAMPS, LOGL_DEBUG, "PROCEEDING received but no transaction (may be internal call)\n");
		return;
	}

	/* Only handle if we're in WAIT_PROCEED state (delayed assignment) */
	if (trans->state != TRANS_CALL_MO_WAIT_PROCEED) {
		LOGP(DAMPS, LOGL_DEBUG, "PROCEEDING received but not in WAIT_PROCEED state (state=%s)\n",
			trans_short_state_name(trans->state));
		return;
	}

	/* Cancel the timeout timer */
	osmo_timer_del(&trans->timer);

	/* Now assign voice channel */
	vc = search_free_vc();
	if (!vc) {
		LOGP(DAMPS, LOGL_NOTICE, "No free voice channel after PROCEEDING, sending Reorder\n");
		const char *number = amps_min2number(trans->min1, trans->min2);
		if (trans->callref) {
			call_up_release(trans->callref, CAUSE_NOCHANNEL);
			trans->callref = 0;
		}
		amps_reorder(number);
		return;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Network PROCEEDING received, assigning voice channel %s\n", vc->sender.kanal);

	/* Set channel and transition to assignment state */
	trans->chan = atoi(vc->sender.kanal);
	trans_new_state(trans, TRANS_CALL_MO_ASSIGN);
}

/* Call control sends disconnect (with tones).
 * An active call stays active, so tones and annoucements can be received
 * by mobile station.
 */
void call_down_disconnect(int callref, int cause)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans;

	LOGP(DAMPS, LOGL_INFO, "Call has been disconnected by network.\n");

	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		/* search transaction for this callref */
		trans = search_transaction_callref(amps, callref);
		if (trans)
			break;
	}
	if (!sender) {
		LOGP(DAMPS, LOGL_NOTICE, "Outgoing disconnect, but no callref!\n");
		call_up_release(callref, CAUSE_INVALCALLREF);
		return;
	}

	/* Release when not active */

	switch (amps->dsp_mode) {
	case DSP_MODE_AUDIO_RX_AUDIO_TX:
	case DSP_MODE_AUDIO_RX_FRAME_TX:
		if (trans->state == TRANS_CALL_MT_ASSIGN_CONFIRM
		 || trans->state == TRANS_CALL_MT_ALERT
		 || trans->state == TRANS_CALL_MT_ALERT_SEND
		 || trans->state == TRANS_CALL_MT_ALERT_CONFIRM
		 || trans->state == TRANS_CALL_MT_ANSWER_WAIT) {
			LOGP_CHAN(DAMPS, LOGL_INFO, "Call control disconnect on voice channel while alerting, releasing towards mobile station.\n");
			amps_release(trans, cause);
		}
		return;
	default:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Call control disconnects on control channel, removing transaction.\n");
		call_up_release(callref, cause);
		trans->callref = 0;
		destroy_transaction(trans);
		amps_go_idle(amps);
	}
}

/* Call control releases call toward mobile station. */
void call_down_release(int callref, int cause)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans;

	LOGP(DAMPS, LOGL_INFO, "Call has been released by network, releasing call.\n");

	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		/* search transaction for this callref */
		trans = search_transaction_callref(amps, callref);
		if (trans)
			break;
	}
	if (!sender) {
		LOGP(DAMPS, LOGL_NOTICE, "Outgoing release, but no callref!\n");
		/* don't send release, because caller already released */
		return;
	}

	trans->callref = 0;

	switch (amps->dsp_mode) {
	case DSP_MODE_AUDIO_RX_SILENCE_TX:
	case DSP_MODE_AUDIO_RX_AUDIO_TX:
	case DSP_MODE_AUDIO_RX_FRAME_TX:
		/* Phone is on voice channel - use Release order */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Call control releases on voice channel, releasing towards mobile station.\n");
		amps_release(trans, cause);
		break;
	default:
		/* Phone is on control channel - check if we can send Reorder/Intercept */
		if (trans->state == TRANS_CALL_MO_WAIT_PROCEED ||
		    trans->state == TRANS_CALL_MO_ASSIGN ||
		    trans->state == TRANS_CALL_MO_ASSIGN_SEND) {
			/*
			 * Delayed Channel Assignment: Network rejected before phone tuned to VC
			 * =====================================================================
			 *
			 * Phone is still on FOCC - either waiting for PROCEEDING, or we've
			 * started sending channel assignment but phone hasn't tuned yet.
			 * Per TIA/EIA-553-A Section 2.6.3.8, we can send Reorder or Intercept
			 * on FOCC and the mobile will return to idle automatically.
			 *
			 * CAUSE CODE MAPPING:
			 *   Reorder (Order 4) - "All circuits busy, try again later"
			 *     - CAUSE_NOCHANNEL (34): No circuit/channel available
			 *     - CAUSE_TEMPFAIL (41): Temporary failure
			 *
			 *   Intercept (Order 9) - "Number cannot be completed as dialed"
			 *     - CAUSE_UNASSIGNED (1): Unassigned/unallocated number
			 *     - CAUSE_INVALNUMBER (28): Invalid number format
			 *     - CAUSE_OUTOFORDER (27): Destination out of service
			 */
			const char *number = amps_min2number(trans->min1, trans->min2);
			
			/* Cancel any pending timer */
			osmo_timer_del(&trans->timer);
			
			switch (cause) {
			case CAUSE_UNASSIGNED:  /* 1 - Unassigned number (SIP 404) */
			case CAUSE_INVALNUMBER: /* 28 - Invalid number format */
			case CAUSE_OUTOFORDER:  /* 27 - Destination out of service */
				LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Intercept (cause=%d) to MO caller on FOCC\n", cause);
				amps_intercept(number);
				return; /* Transaction cleanup happens after order is sent */
			default:
				/* All other causes: send Reorder */
				LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Reorder (cause=%d) to MO caller on FOCC\n", cause);
				amps_reorder(number);
				return; /* Transaction cleanup happens after order is sent */
			}
		}
		/* Other control channel states: just destroy transaction */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Call control releases on control channel, removing transaction.\n");
		destroy_transaction(trans);
		amps_go_idle(amps);
	}
}

/* Call control sends DTMF digits toward mobile station.
 * Note: DTMF TX is now handled in call.c (per-call, not per-channel).
 */

/*
 * ============================================================================
 * COMMON ORDER DISPATCH PATTERN (Asynchronous)
 * ============================================================================
 *
 * The AMPS order system uses an ASYNCHRONOUS callback-driven architecture.
 * Order API functions set up state and return immediately - actual transmission
 * and confirmation happen via DSP/frame callbacks.
 *
 * ASYNC FLOW:
 * -----------
 * 1. Order API (e.g., amps_alert_order()) sets trans->state and DSP mode
 * 2. DSP layer (dsp.c) requests frames when ready via fsk_frame()
 * 3. Frame layer (frame.c) calls amps_tx_frame_fvc() to get order parameters
 * 4. Frame encoding happens in frame.c, transmission via sound/SDR
 * 5. Confirmation arrives asynchronously:
 *    - ST tone: detected by DSP, calls amps_rx_signaling_tone()
 *    - RVC message: decoded by frame.c, calls appropriate handler
 *
 * PATTERN OVERVIEW (Requirements 1.1, 1.2, 1.3):
 * ---------------------------------------------
 *
 * 1. COMMON ORDER DISPATCH PATTERN (Requirement 1.1):
 *    All orders that are sent to a mobile station follow this structure:
 *
 *    int amps_<order_name>(const char *number, [optional params])
 *    {
 *        sender_t *sender;
 *        amps_t *amps;
 *        transaction_t *trans = NULL;
 *        uint32_t min1;
 *        uint16_t min2;
 *
 *        // Step 1: Convert phone number to MIN (Mobile Identification Number)
 *        if (amps_number2min(number, &min1, &min2) < 0)
 *            return -1;
 *
 *        // Step 2: Find transaction for this subscriber
 *        for (sender = sender_head; sender; sender = sender->next) {
 *            amps = (amps_t *) sender;
 *            trans = search_transaction_number(amps, min1, min2);
 *            if (trans)
 *                break;
 *        }
 *
 *        // Step 3: Verify subscriber state
 *        if (!trans) {
 *            LOGP(DAMPS, LOGL_NOTICE, "<Order>: subscriber '%s' not found\n", number);
 *            return -CAUSE_OUTOFORDER;
 *        }
 *        if (trans->state != TRANS_CALL) {  // or other valid states
 *            LOGP(DAMPS, LOGL_NOTICE, "<Order>: subscriber '%s' not in valid state\n", number);
 *            return -CAUSE_BUSY;
 *        }
 *
 *        // Step 4: Log order transmission
 *        LOGP_CHAN(DAMPS, LOGL_INFO, "Sending <Order> to '%s'\n", number);
 *
 *        // Step 5: Set order parameters in transaction
 *        trans->msg_type = <msg_type>;  // LOCAL_MSG_TYPE field
 *        trans->ordq = <ordq>;          // Order Qualifier (0-7)
 *        trans->order = <order>;        // Order code (see TIA/EIA-553-A Table 3.7.1-1)
 *
 *        // Step 6: Transition state and switch DSP mode - RETURNS IMMEDIATELY
 *        //         Actual transmission happens asynchronously via DSP callbacks
 *        trans_new_state(trans, TRANS_CALL_<ORDER>);
 *        amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
 *
 *        return 0;  // Success - order queued for async transmission
 *    }
 *
 * 2. ST-CONFIRMED ORDER HANDLING (Requirement 1.2):
 *    Orders confirmed by Signaling Tone (10 kHz) use amps_rx_signaling_tone():
 *    - Alert (Order 1), Release (Order 3), Maintenance (Order 10), Alert With Info (Order 17)
 *    - Timeout: 5 seconds (configurable via ALERT_TO, etc.)
 *    - Retry logic: typically 3 attempts before failure
 *    - State transition: TRANS_CALL_<ORDER>_SEND -> wait for ST -> TRANS_CALL
 *
 * 3. RVC-CONFIRMED ORDER HANDLING (Requirement 1.3):
 *    Orders confirmed by RVC message use handlers in frame.c:
 *    - Change Power (Order 11), Flash With Info (Order 18), PCI Query (Order 26)
 *    - ESN Request (Order 15), Send Called-Address (Order 8)
 *    - Message parsing in amps_decode_frame_rvc() or dedicated handlers
 *    - Logging of response data
 *    - State transition: TRANS_CALL_<ORDER>_SEND -> response handled async -> TRANS_CALL
 *
 * 4. NO-CONFIRMATION ORDERS:
 *    Some orders don't require confirmation:
 *    - Reorder (Order 4), Message Waiting (Order 5), Stop Alert (Order 6)
 *    - Intercept (Order 9), Local Control (Order 30)
 *    - State transition: TRANS_CALL_<ORDER> -> TRANS_CALL_<ORDER>_SEND -> TRANS_CALL
 *
 * ORDER CODES (TIA/EIA-553-A Table 3.7.1-1):
 * ------------------------------------------
 *   0  - Page/Origination
 *   1  - Alert (ORDQ=0: standard, ORDQ=1: abbreviated)
 *   3  - Release
 *   4  - Reorder (fast busy)
 *   5  - Message Waiting (ORDQ=0: voice, 1: SMS, 2: fax)
 *   6  - Stop Alert
 *   7  - Audit
 *   8  - Send Called-Address
 *   9  - Intercept
 *   10 - Maintenance
 *   11 - Change Power (ORDQ=power level 0-7)
 *   12 - Directed Retry (FOCC only)
 *   13 - Registration
 *   15 - Serial Number Request
 *   17 - Alert With Info
 *   18 - Flash With Info
 *   26 - Protocol Capability Indicator (PCI)
 *   30 - Local Control
 *
 * FRAME ENCODING:
 * ---------------
 * Orders are encoded in amps_tx_frame_fvc() (for FVC orders) or
 * amps_tx_frame_focc() (for FOCC orders like Directed Retry).
 * Each order state has a corresponding case in the switch statement.
 *
 * CONTROL FIFO COMMANDS:
 * ----------------------
 * All orders are accessible via /tmp/amps_control named pipe.
 * Command format: "<command>,<MIN>[,<params>]"
 * Handler: amps_myhandler() in main_common.c
 *
 * ============================================================================
 */

/* Send Flash With Info to mobile station during active call
 *
 * This order delivers character information (e.g., second caller ID for call waiting)
 * to the mobile station. The mobile responds with an order confirmation.
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_flash_with_info(const char *number, const char *message, int pi, int si)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With Info: subscriber '%s' not found or not in call\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in active call state */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With Info: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Flash With Info to '%s': '%s' (PI=%d, SI=%d)\n",
		number, message, pi, si);

	/* Store Flash With Info parameters */
	strncpy(amps->tx_fvc_flashinfo, message, sizeof(amps->tx_fvc_flashinfo) - 1);
	amps->tx_fvc_flashinfo[sizeof(amps->tx_fvc_flashinfo) - 1] = '\0';
	amps->tx_fvc_flashinfo_pi = pi;
	amps->tx_fvc_flashinfo_si = si;

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_FLASH_INFO);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Parse CRI data string into BCD array
 * Format: "E1,E2,E3,..." where each element is up to 4 BCD digits
 * Returns number of elements parsed, or -1 on error
 *
 * WARNING: CRI (Charging Rate Indication) is NOT defined in TIA/EIA-553-A.
 * The data structure was provided by Andreas Eversberg in the original
 * osmocom-analog codebase (commit 4e669ec) but was never actually used.
 * No documentation reference is available to verify the value mapping or
 * how mobile stations interpret these BCD digits. This implementation is
 * based solely on the existing data structures - actual behavior with
 * real mobile equipment is unknown and untested.
 *
 * TESTING NOTE: Phones tested confirm the order (send Order Confirmation)
 * but do not display anything. Some show a generic "Call Waiting" message
 * as if it were a normal Flash With Info with no actual data displayed.
 */
static int parse_cri_data(const char *cri_str, uint8_t *cri_data, int max_elements)
{
	int elements = 0;
	const char *p = cri_str;
	
	while (*p && elements < max_elements) {
		int digit_count = 0;
		int base = elements * 4;
		
		/* Parse up to 4 digits for this element */
		while (*p && *p != ',' && digit_count < 4) {
			if (*p >= '0' && *p <= '9') {
				cri_data[base + digit_count] = *p - '0';
				digit_count++;
			}
			p++;
		}
		
		/* Pad remaining digits with 0 */
		while (digit_count < 4) {
			cri_data[base + digit_count] = 0;
			digit_count++;
		}
		
		elements++;
		
		/* Skip comma separator */
		if (*p == ',')
			p++;
	}
	
	return elements;
}

/*
 * Parse TCI data string into BCD array
 * Format: "R1,R2,R3,R4" where each row is up to 4 BCD digits
 * Returns number of rows parsed, or -1 on error
 *
 * WARNING: TCI (Total Charging Information) is NOT defined in TIA/EIA-553-A.
 * The data structure was provided by Andreas Eversberg in the original
 * osmocom-analog codebase (commit 4e669ec) but was never actually used.
 * No documentation reference is available to verify the value mapping or
 * how mobile stations interpret these BCD digits. This implementation is
 * based solely on the existing data structures - actual behavior with
 * real mobile equipment is unknown and untested.
 *
 * TESTING NOTE: Phones tested confirm the order (send Order Confirmation)
 * but do not display anything. Some show a generic "Call Waiting" message
 * as if it were a normal Flash With Info with no actual data displayed.
 */
static int parse_tci_data(const char *tci_str, uint8_t *tci_data, int max_rows)
{
	int rows = 0;
	const char *p = tci_str;
	
	while (*p && rows < max_rows) {
		int digit_count = 0;
		int base = rows * 4;
		
		/* Parse up to 4 digits for this row */
		while (*p && *p != ',' && digit_count < 4) {
			if (*p >= '0' && *p <= '9') {
				tci_data[base + digit_count] = *p - '0';
				digit_count++;
			}
			p++;
		}
		
		/* Pad remaining digits with 0 */
		while (digit_count < 4) {
			tci_data[base + digit_count] = 0;
			digit_count++;
		}
		
		rows++;
		
		/* Skip comma separator */
		if (*p == ',')
			p++;
	}
	
	return rows;
}

/*
 * Send Flash With CRI (Charging Rate Indication) to mobile station
 *
 * This order (Order 18, ORDQ=1) sends charging rate information to the mobile.
 * CRI displays call cost/rate information to the user.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   cri_data: CRI data string in format "E1,E2,E3,..." (up to 8 elements, 4 digits each)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_flash_with_cri(const char *number, const char *cri_data)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;
	int elements;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With CRI: subscriber '%s' not found or not in call\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in active call state */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With CRI: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* Parse CRI data */
	memset(amps->tx_fvc_cri, 0, sizeof(amps->tx_fvc_cri));
	elements = parse_cri_data(cri_data, amps->tx_fvc_cri, 8);
	if (elements <= 0) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With CRI: invalid CRI data '%s'\n", cri_data);
		return -EINVAL;
	}
	amps->tx_fvc_cri_elements = elements;

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Flash With CRI (ORDQ=1) to '%s': %d elements\n",
		number, elements);

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 1;       /* ORDQ=1 for CRI */
	trans->order = 18;     /* Order 18 = Flash With Info */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_FLASH_INFO);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Flash With TCI (Total Charging Information) to mobile station
 *
 * This order (Order 18, ORDQ=2) sends accumulated charge information to the mobile.
 * TCI shows total call charges to the user.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   tci_data: TCI data string in format "R1,R2,R3,R4" (up to 4 rows, 4 digits each)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_flash_with_tci(const char *number, const char *tci_data)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;
	int rows;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With TCI: subscriber '%s' not found or not in call\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in active call state */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With TCI: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* Parse TCI data */
	memset(amps->tx_fvc_tci, 0, sizeof(amps->tx_fvc_tci));
	rows = parse_tci_data(tci_data, amps->tx_fvc_tci, 4);
	if (rows <= 0) {
		LOGP(DAMPS, LOGL_NOTICE, "Flash With TCI: invalid TCI data '%s'\n", tci_data);
		return -EINVAL;
	}
	amps->tx_fvc_tci_rows = rows;

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Flash With TCI (ORDQ=2) to '%s': %d rows\n",
		number, rows);

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 2;       /* ORDQ=2 for TCI */
	trans->order = 18;     /* Order 18 = Flash With Info */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_FLASH_INFO);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Alert With CRI (Charging Rate Indication) to mobile station
 *
 * This order (Order 17, ORDQ=1) alerts the user and sends charging rate information.
 * Used during incoming call setup to show call cost/rate, or during active call
 * to re-alert with updated charging information.
 *
 * Valid states:
 *   - TRANS_CALL: Active call (re-alert with charging info)
 *   - TRANS_CALL_MT_ALERT*: During MT call alerting phase (normal use)
 *   - TRANS_CALL_MT_ANSWER_WAIT: While waiting for answer
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   cri_data: CRI data string in format "E1,E2,E3,..." (up to 8 elements, 4 digits each)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_alert_with_cri(const char *number, const char *cri_data)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;
	int elements;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert With CRI: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in a valid state for Alert With Info
	 * Alert With Info (Order 17) can be sent:
	 * - During active call (TRANS_CALL) to re-alert with charging info
	 * - During MT call alerting (TRANS_CALL_MT_ALERT*) - normal use case
	 * - While waiting for answer (TRANS_CALL_MT_ANSWER_WAIT)
	 */
	if (trans->state != TRANS_CALL &&
	    trans->state != TRANS_CALL_MT_ALERT &&
	    trans->state != TRANS_CALL_MT_ALERT_SEND &&
	    trans->state != TRANS_CALL_MT_ALERT_CONFIRM &&
	    trans->state != TRANS_CALL_MT_ANSWER_WAIT) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert With CRI: subscriber '%s' not in valid state (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* Parse CRI data */
	memset(amps->tx_fvc_cri, 0, sizeof(amps->tx_fvc_cri));
	elements = parse_cri_data(cri_data, amps->tx_fvc_cri, 8);
	if (elements <= 0) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert With CRI: invalid CRI data '%s'\n", cri_data);
		return -EINVAL;
	}
	amps->tx_fvc_cri_elements = elements;

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Alert With CRI (ORDQ=1) to '%s': %d elements\n",
		number, elements);

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 1;       /* ORDQ=1 for CRI */
	trans->order = 17;     /* Order 17 = Alert With Info */

	/* Use existing MT_ALERT states which handle ST confirmation */
	trans->alert_retry = 1;
	trans_new_state(trans, TRANS_CALL_MT_ALERT);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Alert With TCI (Total Charging Information) to mobile station
 *
 * This order (Order 17, ORDQ=2) alerts the user and sends accumulated charge information.
 * Used during incoming call setup to show total charges, or during active call
 * to re-alert with updated charging information.
 *
 * Valid states:
 *   - TRANS_CALL: Active call (re-alert with charging info)
 *   - TRANS_CALL_MT_ALERT*: During MT call alerting phase (normal use)
 *   - TRANS_CALL_MT_ANSWER_WAIT: While waiting for answer
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   tci_data: TCI data string in format "R1,R2,R3,R4" (up to 4 rows, 4 digits each)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_alert_with_tci(const char *number, const char *tci_data)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;
	int rows;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert With TCI: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in a valid state for Alert With Info
	 * Alert With Info (Order 17) can be sent:
	 * - During active call (TRANS_CALL) to re-alert with charging info
	 * - During MT call alerting (TRANS_CALL_MT_ALERT*) - normal use case
	 * - While waiting for answer (TRANS_CALL_MT_ANSWER_WAIT)
	 */
	if (trans->state != TRANS_CALL &&
	    trans->state != TRANS_CALL_MT_ALERT &&
	    trans->state != TRANS_CALL_MT_ALERT_SEND &&
	    trans->state != TRANS_CALL_MT_ALERT_CONFIRM &&
	    trans->state != TRANS_CALL_MT_ANSWER_WAIT) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert With TCI: subscriber '%s' not in valid state (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* Parse TCI data */
	memset(amps->tx_fvc_tci, 0, sizeof(amps->tx_fvc_tci));
	rows = parse_tci_data(tci_data, amps->tx_fvc_tci, 4);
	if (rows <= 0) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert With TCI: invalid TCI data '%s'\n", tci_data);
		return -EINVAL;
	}
	amps->tx_fvc_tci_rows = rows;

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Alert With TCI (ORDQ=2) to '%s': %d rows\n",
		number, rows);

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 2;       /* ORDQ=2 for TCI */
	trans->order = 17;     /* Order 17 = Alert With Info */

	/* Use existing MT_ALERT states which handle ST confirmation */
	trans->alert_retry = 1;
	trans_new_state(trans, TRANS_CALL_MT_ALERT);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Alert order (Order 1, ORDQ=0) to mobile station during active call
 *
 * This order causes the mobile station to ring/alert the user.
 * The mobile responds with ST (Signaling Tone) confirmation.
 *
 * Per TIA/EIA-553-A Section 2.6.4.4 (Conversation Task):
 * "Alert or Alert With Info: Turn on signaling tone, wait 500 ms,
 *  and then enter the Waiting for Answer Task"
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 3.1, 3.2, 3.4, 3.6
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_alert_order(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert Order: subscriber '%s' not found or not in call\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in active call state */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Alert Order: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Alert Order (Order 1, ORDQ=0) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Mobile will ring/alert the user\n",
		amps_table4_name(0, 0, 1));

	/* Set order parameters in transaction */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for standard alert */
	trans->order = 1;      /* Order 1 = Alert */

	/* Transition state and switch DSP mode */
	/* Use existing MT_ALERT states which handle ST confirmation */
	trans->alert_retry = 1;
	trans_new_state(trans, TRANS_CALL_MT_ALERT);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Abbreviated Alert order to mobile station during active call
 *
 * This order (Order 1, ORDQ=1) causes the mobile station to play a brief
 * feature reminder tone instead of a full ring. The mobile responds with
 * ST (Signaling Tone) confirmation.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 3.3, 3.5, 3.6
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_abbreviated_alert(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Abbreviated Alert: subscriber '%s' not found or not in call\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in active call state */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Abbreviated Alert: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Abbreviated Alert (Order 1, ORDQ=1) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Mobile will play brief feature reminder tone\n",
		amps_table4_name(0, 1, 1));

	/* Set order parameters in transaction */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 1;       /* ORDQ=1 for abbreviated alert */
	trans->order = 1;      /* Order 1 = Alert */

	/* Transition state and switch DSP mode */
	trans_new_state(trans, TRANS_CALL_ABB_ALERT);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Release Order (Order 3) to subscriber via control FIFO
 *
 * This function sends a Release order to force call termination.
 * The mobile station will confirm with ST (Signaling Tone).
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 4.3, 4.5
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_release_order(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Release Order: subscriber '%s' not found or not in call\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in an appropriate state for release */
	if (trans->state != TRANS_CALL &&
	    trans->state != TRANS_CALL_MT_ALERT &&
	    trans->state != TRANS_CALL_MT_ALERT_SEND &&
	    trans->state != TRANS_CALL_MT_ALERT_CONFIRM &&
	    trans->state != TRANS_CALL_MT_ANSWER_WAIT) {
		LOGP(DAMPS, LOGL_NOTICE, "Release Order: subscriber '%s' not in valid state for release (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Release Order (Order 3, ORDQ=0) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Mobile will terminate call and release channel\n",
		amps_table4_name(0, 0, 3));

	/* Set order parameters in transaction */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for standard Release */
	trans->order = 3;      /* Order 3 = Release */

	/* Cancel any existing timer and set release timer */
	osmo_timer_del(&trans->timer);
	osmo_timer_schedule(&trans->timer, RELEASE_TIMER);

	/* Transition state and switch DSP mode */
	trans_new_state(trans, TRANS_CALL_RELEASE);
	
	/* Change DSP mode to transmit release */
	if (amps->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX || 
	    amps->dsp_mode == DSP_MODE_AUDIO_RX_SILENCE_TX || 
	    amps->dsp_mode == DSP_MODE_OFF)
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Reorder Order (Order 4) to subscriber
 *
 * This order informs the user that all facilities (circuits/trunks) are busy
 * and the call should be placed again later. The mobile plays a "fast busy"
 * tone (reorder tone) - 480+620 Hz interrupted at 120 IPM.
 *
 * This is a MANDATORY order - all AMPS mobile stations must support it.
 * The reorder tone is generated by the mobile station itself.
 *
 * No confirmation is expected from the mobile station.
 *
 * IMPORTANT: This is a CALL SETUP order, not an in-call order!
 * It is sent after channel assignment but before the call is connected.
 * During an active call (TRANS_CALL), this order has no effect on the mobile.
 * The typical sequence is:
 *   1. Mobile originates call
 *   2. Base assigns voice channel
 *   3. Mobile tunes to VC, sends SAT (TRANS_CALL_MO_ASSIGN_CONFIRM)
 *   4. Base checks PSTN - if busy, sends Reorder then Release
 *
 * CC Cause Mapping:
 *   CAUSE_NOCHANNEL (34) -> Reorder (all circuits busy)
 *   CAUSE_TEMPFAIL (41)  -> Reorder (temporary network failure)
 *
 * Valid states (call setup only):
 *   - TRANS_CALL_MO_ASSIGN_CONFIRM: After MO channel assignment (most common)
 *   - TRANS_CALL_MT_ASSIGN_CONFIRM: After MT channel assignment (rare)
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 5.1, 5.2, 5.3
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_reorder(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	amps_t *cc_amps;  /* Control channel AMPS instance */
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Reorder: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Reorder is ONLY valid on FOCC during call setup, per TIA/EIA-553-A.
	 * Section 3.6.4.1 lists valid responses to origination: Initial Voice Channel,
	 * Directed Retry, Intercept, Reorder.
	 * Section 3.6.4.4 (Conversation on FVC) does NOT include Reorder.
	 *
	 * Valid states (phone still on FOCC):
	 *   - TRANS_CALL_MO_WAIT_PROCEED: Waiting for network PROCEEDING
	 *   - TRANS_CALL_MO_ASSIGN: PROCEEDING received, about to send assignment
	 *   - TRANS_CALL_MO_ASSIGN_SEND: Assignment being sent, phone hasn't tuned yet
	 *
	 * Invalid states:
	 *   - TRANS_CALL: Active call on FVC - use Release instead
	 *   - TRANS_CALL_MO_ASSIGN_CONFIRM: Phone already on FVC waiting for SAT
	 */
	if (trans->state == TRANS_CALL) {
		LOGP(DAMPS, LOGL_ERROR, "Reorder: Cannot send during active call (state=%s). "
			"Per TIA/EIA-553-A, Reorder is only valid on FOCC during call setup. "
			"Use Release (Order 3) instead.\n", trans_short_state_name(trans->state));
		return -EINVAL;
	}

	if (trans->state == TRANS_CALL_MO_ASSIGN_CONFIRM ||
	    trans->state == TRANS_CALL_MT_ASSIGN_CONFIRM) {
		LOGP(DAMPS, LOGL_ERROR, "Reorder: Phone already on voice channel (state=%s). "
			"Per TIA/EIA-553-A, Reorder is only valid on FOCC. "
			"Use Release (Order 3) instead.\n", trans_short_state_name(trans->state));
		return -EINVAL;
	}

	if (trans->state != TRANS_CALL_MO_WAIT_PROCEED &&
	    trans->state != TRANS_CALL_MO_ASSIGN &&
	    trans->state != TRANS_CALL_MO_ASSIGN_SEND) {
		LOGP(DAMPS, LOGL_NOTICE, "Reorder: Invalid state %s for subscriber '%s'. "
			"Reorder is only valid during MO call setup before phone tunes to VC.\n",
			trans_short_state_name(trans->state), number);
		return -EINVAL;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Reorder Order (Order 4, ORDQ=0) to '%s' on FOCC\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Mobile will play fast busy tone (all circuits busy)\n",
		amps_table4_name(0, 0, 4));

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for Reorder */
	trans->order = 4;      /* Order 4 = Reorder */

	/* Cancel any pending timer */
	osmo_timer_del(&trans->timer);

	/* Find control channel to send the order on FOCC.
	 * The phone is still listening on FOCC during call setup.
	 */
	cc_amps = search_pc();
	if (!cc_amps) {
		LOGP(DAMPS, LOGL_ERROR, "No control channel found for Reorder order\n");
		return -CAUSE_TEMPFAIL;
	}

	/* Move transaction to control channel if not already there */
	if (trans->amps != cc_amps) {
		unlink_transaction(trans);
		link_transaction(trans, cc_amps);
	}

	/* Set transaction state to trigger FOCC transmission */
	trans_new_state(trans, TRANS_REORDER);

	return 0;
}

/*
 * Send Message Waiting Order (Order 5) to subscriber
 *
 * This order notifies the mobile station that messages are waiting.
 * The MSG_TYPE field indicates the number of messages (0-30, or 31 for unknown).
 * The mobile typically displays an indicator icon.
 *
 * No confirmation is expected from the mobile station.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   count: Number of messages waiting (0-31, 31 = unknown count)
 *   type: Message type (0=voice, 1=SMS, 2=fax)
 *
 * Requirements: 6.1, 6.2, 6.3, 6.4, 6.5
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_mwi(const char *number, int count, int type)
{
	sender_t *sender;
	amps_t *amps = NULL;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Validate parameters */
	if (count < 0 || count > 31) {
		LOGP(DAMPS, LOGL_NOTICE, "MWI: invalid count %d (must be 0-31)\n", count);
		return -EINVAL;
	}
	if (type < 0 || type > 2) {
		LOGP(DAMPS, LOGL_NOTICE, "MWI: invalid type %d (must be 0-2)\n", type);
		return -EINVAL;
	}

	/* Find transaction for this subscriber (if any) */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	/* If transaction exists, check if it's in a valid state for FVC MWI
	 * Per TIA/EIA-553-A, MWI can be sent on FVC during:
	 * - Conversation Task (TRANS_CALL) - active call
	 * - Waiting for Answer Task (TRANS_CALL_MT_ANSWER_WAIT) - phone is ringing
	 * - Waiting for Order Task (TRANS_CALL_MT_ALERT_CONFIRM) - after alert sent
	 */
	if (trans) {
		if (trans->state == TRANS_CALL ||
		    trans->state == TRANS_CALL_MT_ANSWER_WAIT ||
		    trans->state == TRANS_CALL_MT_ALERT_CONFIRM) {
			/* Valid FVC state - will send on voice channel */
		} else {
			LOGP(DAMPS, LOGL_NOTICE, "MWI: subscriber '%s' has transaction but not in valid FVC state (state=%d). "
				"Will send on FOCC instead.\n", number, trans->state);
			/* Don't use this transaction, create new one for FOCC */
			trans = NULL;
		}
	}

	/* If no valid transaction, create one for FOCC MWI */
	if (!trans) {
		/* Find a paging channel to send the FOCC message */
		amps = search_pc();
		if (!amps) {
			LOGP(DAMPS, LOGL_ERROR, "MWI: No paging channel available for FOCC MWI\n");
			return -CAUSE_NOCHANNEL;
		}

		/* Create transaction for FOCC MWI */
		trans = create_transaction(amps, TRANS_MWI, min1, min2, 0, count, type, 5, 0);
		if (!trans) {
			LOGP(DAMPS, LOGL_ERROR, "MWI: Failed to create transaction\n");
			return -CAUSE_TEMPFAIL;
		}

		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Message Waiting (Order 5, ORDQ=%d) to '%s' on FOCC: count=%d\n",
			type, number, count);
		LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> MWI %s: %d message(s) waiting\n",
			(type == 0) ? "Voice" : (type == 1) ? "SMS" : "Fax", count);

		/* Transaction is already in TRANS_MWI state, FOCC handler will pick it up */
		return 0;
	}

	/* Subscriber is on voice channel - send MWI on FVC */
	if (trans->state == TRANS_CALL) {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Message Waiting (Order 5, ORDQ=%d) to '%s' on FVC (active call): count=%d\n",
			type, number, count);
	} else {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Message Waiting (Order 5, ORDQ=%d) to '%s' on FVC (alerting): count=%d\n",
			type, number, count);
	}
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Mobile will show message waiting indicator (%d messages)\n",
		amps_table4_name(count, type, 5), count);

	/* Save return state - we'll go back to this state after MWI is sent
	 * Per TIA/EIA-553-A, after MWI during alerting, mobile sends order confirmation
	 * and remains in the same task (Waiting for Answer Task).
	 */
	trans->mwi_return_state = trans->state;

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = count;  /* MSG_TYPE carries the message count */
	trans->ordq = type;       /* ORDQ carries the message type (0=voice, 1=SMS, 2=fax) */
	trans->order = 5;         /* Order 5 = Message Waiting */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_MWI);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Stop Alert Order (Order 6) to subscriber
 *
 * This order tells the mobile station to discontinue alerting (ringing) the user.
 * Used when the calling party hangs up before the mobile user answers.
 * The voice channel is NOT torn down - the mobile remains on channel.
 *
 * No confirmation is expected from the mobile station.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 7.1, 7.2, 7.3, 7.4, 7.5
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_stopalert(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Stop Alert: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in alerting state */
	if (trans->state != TRANS_CALL_MT_ALERT &&
	    trans->state != TRANS_CALL_MT_ALERT_SEND &&
	    trans->state != TRANS_CALL_MT_ALERT_CONFIRM &&
	    trans->state != TRANS_CALL_MT_ANSWER_WAIT) {
		LOGP(DAMPS, LOGL_NOTICE, "Stop Alert: subscriber '%s' not in alerting state (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Stop Alert (Order 6, ORDQ=0) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Mobile will stop ringing but stay on channel\n",
		amps_table4_name(0, 0, 6));

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for Stop Alert */
	trans->order = 6;      /* Order 6 = Stop Alert */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_STOP_ALERT);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Intercept Order (Order 9) to subscriber
 *
 * This order informs the user of a procedural error made in placing the call.
 * The mobile plays an intercept tone (alternating high-low tone) indicating
 * the number cannot be completed as dialed.
 *
 * This is a MANDATORY order - all AMPS mobile stations must support it.
 * The intercept tone is generated by the mobile station itself.
 *
 * No confirmation is expected from the mobile station.
 *
 * IMPORTANT: This is a CALL SETUP order, not an in-call order!
 * It is sent after channel assignment but before the call is connected.
 * During an active call (TRANS_CALL), this order has no effect on the mobile.
 * The typical sequence is:
 *   1. Mobile originates call, dials number
 *   2. Base assigns voice channel
 *   3. Mobile tunes to VC, sends SAT (TRANS_CALL_MO_ASSIGN_CONFIRM)
 *   4. Base checks dialed number - if invalid, sends Intercept then Release
 *
 * CC Cause Mapping:
 *   CAUSE_INVALNUMBER (28) -> Intercept (invalid number dialed)
 *   CAUSE_OUTOFORDER (27)  -> Intercept (number not in service)
 *
 * Valid states (call setup only):
 *   - TRANS_CALL_MO_ASSIGN_CONFIRM: After MO channel assignment
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 10.1, 10.2, 10.3
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_intercept(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	amps_t *cc_amps;  /* Control channel AMPS instance */
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Intercept: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Intercept is ONLY valid on FOCC during MO call setup, per TIA/EIA-553-A.
	 * Section 3.6.4.1 lists valid responses to origination: Initial Voice Channel,
	 * Directed Retry, Intercept, Reorder.
	 * Section 3.6.4.4 (Conversation on FVC) does NOT include Intercept.
	 *
	 * Valid states (phone still on FOCC):
	 *   - TRANS_CALL_MO_WAIT_PROCEED: Waiting for network PROCEEDING
	 *   - TRANS_CALL_MO_ASSIGN: PROCEEDING received, about to send assignment
	 *   - TRANS_CALL_MO_ASSIGN_SEND: Assignment being sent, phone hasn't tuned yet
	 *
	 * Invalid states:
	 *   - TRANS_CALL: Active call on FVC - use Release instead
	 *   - TRANS_CALL_MO_ASSIGN_CONFIRM: Phone already on FVC waiting for SAT
	 *   - MT calls: Intercept is only for MO calls (procedural error in placing call)
	 */
	if (trans->state == TRANS_CALL) {
		LOGP(DAMPS, LOGL_ERROR, "Intercept: Cannot send during active call (state=%s). "
			"Per TIA/EIA-553-A, Intercept is only valid on FOCC during call setup. "
			"Use Release (Order 3) instead.\n", trans_short_state_name(trans->state));
		return -EINVAL;
	}

	if (trans->state == TRANS_CALL_MO_ASSIGN_CONFIRM ||
	    trans->state == TRANS_CALL_MT_ASSIGN_CONFIRM) {
		LOGP(DAMPS, LOGL_ERROR, "Intercept: Phone already on voice channel (state=%s). "
			"Per TIA/EIA-553-A, Intercept is only valid on FOCC. "
			"Use Release (Order 3) instead.\n", trans_short_state_name(trans->state));
		return -EINVAL;
	}

	if (trans->state != TRANS_CALL_MO_WAIT_PROCEED &&
	    trans->state != TRANS_CALL_MO_ASSIGN &&
	    trans->state != TRANS_CALL_MO_ASSIGN_SEND) {
		LOGP(DAMPS, LOGL_NOTICE, "Intercept: Invalid state %s for subscriber '%s'. "
			"Intercept is only valid during MO call setup before phone tunes to VC.\n",
			trans_short_state_name(trans->state), number);
		return -EINVAL;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Intercept Order (Order 9, ORDQ=0) to '%s' on FOCC\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Mobile will play intercept tone (number cannot be completed)\n",
		amps_table4_name(0, 0, 9));

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for Intercept */
	trans->order = 9;      /* Order 9 = Intercept */

	/* Cancel any pending timer */
	osmo_timer_del(&trans->timer);

	/* Find control channel to send the order on FOCC.
	 * The phone is still listening on FOCC during call setup.
	 */
	cc_amps = search_pc();
	if (!cc_amps) {
		LOGP(DAMPS, LOGL_ERROR, "No control channel found for Intercept order\n");
		return -CAUSE_TEMPFAIL;
	}

	/* Move transaction to control channel if not already there */
	if (trans->amps != cc_amps) {
		unlink_transaction(trans);
		link_transaction(trans, cc_amps);
	}

	/* Set transaction state to trigger FOCC transmission */
	trans_new_state(trans, TRANS_INTERCEPT);

	return 0;
}

/*
 * Send Called-Address Request (Order 8)
 *
 * This order requests the mobile station to send a message containing
 * dialed-digit information. Used for call transfer and three-way calling.
 *
 * USAGE (per TIA/EIA-553-A Section 2.6.4.4):
 *   The mobile will ONLY respond if Order 8 is sent within 10 seconds
 *   of the completion of the last valid flash (hook-flash).
 *
 *   Typical workflow for call transfer / 3-way calling:
 *   1. User presses FLASH on mobile (sends 400ms ST burst)
 *   2. User dials digits on mobile keypad (e.g., transfer destination)
 *   3. User presses FLASH again (optional, to complete transfer)
 *   4. Base station sends Order 8 ("digits" command) within 10 seconds of step 1
 *   5. Mobile responds with Called-Address message containing the dialed digits
 *
 *   If Order 8 is sent outside the 10-second window, mobile ignores it.
 *
 * Console command:
 *   echo "digits,<MIN>" > /tmp/amps_control
 *
 * The mobile responds with a Called-Address message (T=0 on RVC) containing
 * the dialed digits. The response is decoded in frame.c and passed to
 * amps_rx_recc() for logging.
 *
 * Valid states:
 *   - TRANS_CALL: Active call (Conversation Task)
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 9.1, 9.2, 9.3
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_send_called_address(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Send Called-Address: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Send Called-Address is only valid during active call (Conversation Task) */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Send Called-Address: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Send Called-Address Order (Order 8, ORDQ=0) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Requesting dialed digits from mobile\n",
		amps_table4_name(0, 0, 8));
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> NOTE: Mobile will only respond if Order 8 is sent within 10 seconds of a flash!\n");

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for Send Called-Address */
	trans->order = 8;      /* Order 8 = Send Called-Address */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_DIGITS_REQUEST);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Maintenance Order (Order 10)
 *
 * This order is used to check the operation of a mobile station.
 * All functions are similar to Alert, but the alerting device (ringer)
 * is NOT activated. The mobile confirms with ST but doesn't ring.
 *
 * Per TIA/EIA-553-A Section 2.6.4.2 (Waiting for Order Task):
 * "Maintenance: Turn on signaling tone, wait 500 ms, and enter the
 *  Waiting for Answer Task"
 *
 * Per TIA/EIA-553-A Section 2.6.4.3.2 (Waiting for Answer Task):
 * "Maintenance: Remain in the Waiting for Answer Task, and reset
 *  the alert timer to 65 seconds."
 *
 * Valid states:
 *   - TRANS_CALL: Active call (Conversation Task)
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Requirements: 11.1, 11.2, 11.3, 11.4
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_maintenance(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Maintenance: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Maintenance is only valid during active call (Conversation Task) */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Maintenance: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Maintenance Order (Order 10, ORDQ=0) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> %s - Silent test (mobile confirms with ST, no ring)\n",
		amps_table4_name(0, 0, 10));

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for Maintenance */
	trans->order = 10;     /* Order 10 = Maintenance */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_MAINTENANCE);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Silent Page (Page + Maintenance Order)
 *
 * This function implements a "silent page" flow that pages an idle mobile
 * and sends a Maintenance order instead of Alert. This allows testing
 * the mobile's RF path without ringing the phone.
 *
 * Flow:
 * 1. Page the mobile on FOCC (same as MT call)
 * 2. Wait for page reply
 * 3. Assign voice channel
 * 4. Wait for SAT confirmation
 * 5. Send Maintenance order (Order 10) instead of Alert
 * 6. Wait for ST confirmation
 * 7. Release the channel
 *
 * Per TIA/EIA-553-A:
 * - Maintenance order can ONLY be sent on FVC (during call)
 * - This flow establishes a call just to send Maintenance
 * - Mobile confirms with ST (500ms) but doesn't ring
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_silent_page(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0) {
		LOGP(DAMPS, LOGL_ERROR, "Invalid number '%s'\n", number);
		return -1;
	}

	/* Check if subscriber is already in a call */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}
	if (trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Silent Page: subscriber '%s' already has transaction (state=%d)\n",
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* Check if we have a free voice channel */
	if (!search_free_vc()) {
		LOGP(DAMPS, LOGL_NOTICE, "Silent Page: no free voice channel\n");
		return -CAUSE_NOCHANNEL;
	}

	/* Find paging channel */
	amps = search_pc();
	if (!amps) {
		LOGP(DAMPS, LOGL_NOTICE, "Silent Page: paging channel busy\n");
		return -CAUSE_NOCHANNEL;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: paging '%s' for maintenance test\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Silent Page flow: Page -> Assign -> Maintenance -> Release\n");

	/* Create transaction for silent page */
	trans = create_transaction(amps, TRANS_SILENT_PAGE, min1, min2, 0, 0, 0, 0, 0);
	if (!trans) {
		LOGP(DAMPS, LOGL_ERROR, "Failed to create transaction\n");
		return -CAUSE_TEMPFAIL;
	}
	trans->page_retry = 1;
	/* No callref - this is not a real call */
	trans->callref = 0;

	return 0;
}

/*
 * Query Protocol Capability Indicator (PCI) from subscriber
 *
 * This order requests the mobile station to report its protocol capabilities
 * (MSPC and MSCAP fields). The response is logged.
 *
 * Can be sent on:
 * - FOCC (idle mode): Creates a new transaction, sends on control channel
 * - FVC (in-call): Uses existing transaction, sends on voice channel
 *
 * TESTING NOTE: None of the phones tested have ever replied to PCI query.
 * This may be because PCI support was added late to the standard (TIA/EIA-553-A)
 * and older phones do not implement it. The order is sent correctly but
 * no response is received from the mobile station.
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_pci_query(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	/* If transaction exists, it must be in a valid state */
	if (trans && trans->state != TRANS_CALL &&
	    trans->state != TRANS_CALL_PCI_QUERY &&
	    trans->state != TRANS_CALL_PCI_QUERY_SEND) {
		LOGP(DAMPS, LOGL_NOTICE, "PCI Query: subscriber '%s' busy (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* If no transaction, create one for FOCC PCI query */
	if (!trans) {
		/* Use the first AMPS instance to send the FOCC message */
		if (sender_head)
			amps = (amps_t *) sender_head;
		else {
			LOGP(DAMPS, LOGL_ERROR, "No AMPS instance found for PCI Query\n");
			return -CAUSE_TEMPFAIL;
		}

		trans = create_transaction(amps, TRANS_PCI, min1, min2, 0, 0, 4, 26, 0);
		if (!trans)
			return -CAUSE_TEMPFAIL;
		/* Note: create_transaction already links the transaction */
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending PCI Query (Order=26, ORDQ=4) to '%s'\n", number);

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 4;    /* ORDQ=4 for PCI */
	trans->order = 26;  /* Protocol Capability Indicator */

	/* Set transaction state to trigger transmission
	 * If in call -> FVC (TRANS_CALL_PCI_QUERY)
	 * If idle -> FOCC (TRANS_PCI - will be picked up by amps_tx_frame_focc)
	 */
	if (trans->state == TRANS_CALL) {
		trans_new_state(trans, TRANS_CALL_PCI_QUERY);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
	} else {
		/* Idle / FOCC - set to TRANS_PCI so amps_tx_frame_focc() picks it up */
		trans_new_state(trans, TRANS_PCI);
	}

	return 0;
}

/*
 * This order requests the mobile station to perform an audit.
 * The response (Order Confirmation) is logged by frame.c.
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_audit_order(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}



	/* Verify subscriber is in active call state */
	/* If transaction exists, it must be in a valid state */
	if (trans && trans->state != TRANS_CALL &&
	    trans->state != TRANS_CALL_AUDIT &&
	    trans->state != TRANS_CALL_AUDIT_SEND) {
		LOGP(DAMPS, LOGL_NOTICE, "Audit Order: subscriber '%s' busy (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* If no transaction, create one for FOCC audit */
	if (!trans) {
		/* Use the first AMPS instance to send the FOCC message */
		if (sender_head)
			amps = (amps_t *) sender_head;
		else {
			LOGP(DAMPS, LOGL_ERROR, "No AMPS instance found for Audit\n");
			return -CAUSE_TEMPFAIL;
		}

		trans = create_transaction(amps, TRANS_AUDIT, min1, min2, 0, 0, 0, 7, 0);
		if (!trans)
			return -CAUSE_TEMPFAIL;
		/* Note: create_transaction already links the transaction */
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Audit Order to '%s'\n", number);

	/* Set transaction state to trigger transmission
	 * If in call -> FVC (TRANS_CALL_AUDIT)
	 * If idle -> FOCC (TRANS_AUDIT - will be picked up by amps_tx_frame_focc)
	 */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;    /* ORDQ=0 for Audit */
	trans->order = 7;   /* Audit Order */

	if (trans->state == TRANS_CALL) {
		trans_new_state(trans, TRANS_CALL_AUDIT);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
	} else {
		/* Idle / FOCC - set to TRANS_AUDIT so amps_tx_frame_focc() picks it up */
		trans_new_state(trans, TRANS_AUDIT);
	}

	return 0;
}

/*
 * Send Change Power Order (Order 11) to subscriber via control FIFO
 *
 * This function sends a Change Power order to adjust the mobile station's
 * transmit power level. The mobile station will confirm via RVC.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   level: Power level 0-7 (0=max power, 7=min power)
 *
 * Requirements: 12.3, 12.4, 12.5
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_change_power_order(const char *number, int level)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Change Power Order: subscriber '%s' not found or not in call\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Verify subscriber is in active call state (Requirement 12.5) */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Change Power Order: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	/* Clamp power level to valid range 0-7 (Requirement 12.4) */
	if (level < 0) level = 0;
	if (level > 7) level = 7;

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Change Power Order (level=%d) to '%s'\n", level, number);

	/* Set the power level in transaction (used by TRANS_CALL_CHANGE_POWER handler) */
	trans->current_vmac = level;

	/* Transition state and switch DSP mode */
	trans_new_state(trans, TRANS_CALL_CHANGE_POWER);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Serial Number Request (Order 15)
 *
 * Requests the mobile station to send its Electronic Serial Number (ESN).
 * Used for verification and fraud detection during an active call.
 *
 * Per TIA/EIA-553-A Section 3.7.1.1:
 * - ORDER = 15 (01111)
 * - ORDQ = 0
 * - Mobile responds with Serial Number Response on RVC
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_serial_number_request(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "ESN Request: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* ESN Request is only valid during active call */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "ESN Request: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Serial Number Request (Order 15, ORDQ=1) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> ESN Request - Mobile will respond with ESN for verification\n");

	/* Set order parameters
	 * Per TIA/EIA-553-A Table 3.7.1-1:
	 * - Order 15 (01111) with ORDQ=001 = "Serial Number Request/Response"
	 * - Order 15 (01111) with ORDQ=000 = "Parameter Update Order/Confirmation"
	 */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 1;       /* ORDQ=1 for Serial Number Request (not 0!) */
	trans->order = 15;     /* Order 15 = Serial Number Request */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_ESN_REQUEST);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Local Control Order (Order 30)
 *
 * Initiates vendor-specific local control actions in the mobile station.
 * The MSG_TYPE field contains a 5-bit local control code (0-31) that is
 * manufacturer-defined.
 *
 * Per TIA/EIA-553-A Section 3.7.1.1:
 * - ORDER = 30 (11110)
 * - ORDQ = 0
 * - MSG_TYPE = local control code (0-31)
 * - No confirmation expected
 *
 * WARNING: This is highly vendor-specific. The meaning of each code
 * depends on the mobile station manufacturer.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   code: Local control code (0-31)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_local_control(const char *number, int code)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Validate code range */
	if (code < 0 || code > 31) {
		LOGP(DAMPS, LOGL_ERROR, "Local Control: invalid code %d (must be 0-31)\n", code);
		return -1;
	}

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Local Control: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Local Control is only valid during active call */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Local Control: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Local Control (Order 30, code=%d) to '%s'\n", code, number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Local Control Order: code=%d (vendor-specific)\n", code);

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = code;  /* MSG_TYPE carries the local control code */
	trans->ordq = 0;         /* ORDQ=0 for Local Control */
	trans->order = 30;       /* Order 30 = Local Control */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_LOCAL_CONTROL);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Disable DTMF Order on FVC
 *
 * Per TIA/EIA-553-A Section 2.6.4.4:
 * The Disable DTMF order tells the mobile station to disable its DTMF tone
 * generator. The mobile confirms with a digital Order Confirmation message.
 *
 * The DTMF generator remains disabled until the Called Address Message
 * (in response to the next Send Called-Address order) has been completely
 * transmitted. There is NO explicit "Enable DTMF" order - re-enable is automatic.
 *
 * Typical workflow:
 * 1. BS sends Disable DTMF (Order 22) -> Mobile disables DTMF tones
 * 2. User dials digits on keypad (no tones heard by far-end)
 * 3. BS sends Send Called-Address (Order 8) -> Mobile sends digits
 * 4. Mobile completes Called-Address transmission -> DTMF auto-enables
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_disable_dtmf(const char *number)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Find transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_NOTICE, "Disable DTMF: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Disable DTMF is only valid during active call */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_NOTICE, "Disable DTMF: subscriber '%s' not in active call (state=%d)\n", 
			number, trans->state);
		return -CAUSE_BUSY;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Disable DTMF Order (Order 22) to '%s'\n", number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Disable DTMF Order: Mobile will mute DTMF tones until Called-Address sent\n");

	/* Set order parameters */
	trans->chan = 0;
	trans->msg_type = 0;     /* MSG_TYPE=0 for Disable DTMF */
	trans->ordq = 0;         /* ORDQ=0 for Disable DTMF */
	trans->order = 22;       /* Order 22 = Disable DTMF */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_DISABLE_DTMF);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Handoff message on FVC
 *
 * Per TIA/EIA-553-A Section 2.6.4.4:
 * Handoff transfers a mobile station from one voice channel to another.
 * This is sent on the Forward Voice Channel (FVC) during an active call.
 *
 * Mobile station response:
 * - Turn on signaling tone for 50 ms
 * - Turn off signaling tone
 * - Turn off transmitter
 * - Adjust power level (VMAC)
 * - Tune to new channel (CHAN)
 * - Adjust to new SAT color code (SCC)
 * - Turn on transmitter
 * - Reset fade timer
 * - Remain in Conversation Task
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   new_channel: Target voice channel number
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_handoff(const char *number, int new_channel)
{
	sender_t *sender;
	amps_t *amps;
	amps_t *target_vc = NULL;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;
	const char *current_band;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0) {
		LOGP(DAMPS, LOGL_ERROR, "Handoff: invalid number '%s'\n", number);
		return -CAUSE_INVALNUMBER;
	}

	/* Find the transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans)
			break;
	}

	if (!trans) {
		LOGP(DAMPS, LOGL_ERROR, "Handoff: subscriber '%s' not found\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Handoff is only valid during active call */
	if (trans->state != TRANS_CALL) {
		LOGP(DAMPS, LOGL_ERROR, "Handoff: subscriber '%s' not in active call\n", number);
		LOGP(DAMPS, LOGL_ERROR, "  Current state: %s\n", trans_short_state_name(trans->state));
		return -CAUSE_BUSY;
	}

	amps = trans->amps;
	int current_channel = atoi(amps->sender.kanal);
	current_band = amps_channel2band(current_channel);

	/* Auto-select target channel if not specified */
	if (new_channel == 0) {
		LOGP_CHAN(DAMPS, LOGL_INFO, "Handoff: auto-selecting free voice channel (System %c)\n", current_band[0]);
		
		/* Find a free voice channel in the same system (A or B) */
		for (sender = sender_head; sender; sender = sender->next) {
			amps_t *check = (amps_t *) sender;
			int chan = atoi(check->sender.kanal);
			const char *chan_band = amps_channel2band(chan);
			
			/* Skip if different system */
			if (chan_band[0] != current_band[0])
				continue;
			
			/* Skip current channel (it's busy with our call) */
			if (chan == current_channel)
				continue;
			
			/* Check if it's a voice-capable channel and idle */
			if ((check->chan_type == CHAN_TYPE_VC || check->chan_type == CHAN_TYPE_CC_PC_VC) &&
			    check->state == STATE_IDLE) {
				target_vc = check;
				new_channel = chan;
				LOGP_CHAN(DAMPS, LOGL_NOTICE, "Handoff: auto-selected channel %d (System %c)\n", 
					new_channel, current_band[0]);
				break;
			}
		}
		
		if (!target_vc) {
			LOGP_CHAN(DAMPS, LOGL_ERROR, "Handoff: no free voice channel available in System %c\n", current_band[0]);
			return -CAUSE_NOCHANNEL;
		}
	} else {
		/* Manual channel selection */
		
		/* Check if trying to handoff to same channel */
		if (new_channel == current_channel) {
			LOGP_CHAN(DAMPS, LOGL_ERROR, "Handoff: already on channel %d\n", new_channel);
			return -CAUSE_INVALNUMBER;
		}

		/* Find the target voice channel */
		for (sender = sender_head; sender; sender = sender->next) {
			amps_t *check = (amps_t *) sender;
			if (atoi(check->sender.kanal) == new_channel) {
				/* Check if it's a voice-capable channel */
				if (check->chan_type == CHAN_TYPE_VC || 
				    check->chan_type == CHAN_TYPE_CC_PC_VC) {
					/* Check if it's available (idle) */
					if (check->state == STATE_IDLE) {
						target_vc = check;
					} else {
						LOGP_CHAN(DAMPS, LOGL_ERROR, "Handoff: channel %d is busy\n", new_channel);
						return -CAUSE_NOCHANNEL;
					}
				} else {
					LOGP_CHAN(DAMPS, LOGL_ERROR, "Handoff: channel %d is not a voice channel\n", new_channel);
					return -CAUSE_NOCHANNEL;
				}
				break;
			}
		}

		if (!target_vc) {
			LOGP_CHAN(DAMPS, LOGL_ERROR, "Handoff: channel %d not found\n", new_channel);
			return -CAUSE_NOCHANNEL;
		}
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Handoff to '%s': channel %d -> %d\n", 
		number, current_channel, new_channel);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Handoff: %d -> %d (SAT=%d, VMAC=%d)\n", 
		current_channel, new_channel, target_vc->sat, trans->current_vmac);

	/* Store handoff target info in transaction */
	trans->handoff_channel = new_channel;
	trans->handoff_scc = target_vc->sat;

	/* CRITICAL: Prepare target channel BEFORE sending handoff message!
	 * The mobile will tune to the new channel after sending ST confirmation.
	 * The target channel must already be transmitting SAT when the mobile arrives.
	 * 
	 * Per TIA/EIA-553-A Section 2.6.4.4:
	 * - Mobile receives handoff message on current channel
	 * - Mobile sends 50ms ST on current channel
	 * - Mobile turns off transmitter and tunes to new channel
	 * - Mobile expects to find SAT on new channel
	 */
	LOGP_CHAN(DAMPS, LOGL_INFO, "Preparing target channel %d for handoff\n", new_channel);
	amps_new_state(target_vc, STATE_BUSY);
	amps_set_dsp_mode(target_vc, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
	LOGP(DAMPS, LOGL_INFO, "(chan %d) Target channel now transmitting SAT %d\n", new_channel, target_vc->sat);

	/* Set channel field to trigger Word1_B format (handoff message) */
	trans->chan = new_channel;

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_HANDOFF);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);

	return 0;
}

/*
 * Send Directed Retry Order (Order 12) on FOCC
 *
 * Per TIA/EIA-553-A Section 3.7.1.1:
 * Directed Retry redirects a mobile station to attempt access on different
 * control channels. This is an FOCC order sent to mobiles that are currently
 * accessing the system (origination or page response).
 *
 * Message format:
 * - Word 1: Abbreviated Address Word (MIN1)
 * - Word 2: Extended Address Word A (MIN2, ORDER=12, ORDQ=0/1)
 * - Word 3: First Directed-Retry Word (up to 3 CHANPOS values)
 * - Word 4: Second Directed-Retry Word (up to 3 more CHANPOS values)
 *
 * CHANPOS is a 7-bit channel position value. The mobile calculates the
 * actual channel number based on FIRSTCHAs and serving-system status.
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *   channels: Array of channel positions (1-127, up to 6 values)
 *   num_channels: Number of channels in array (1-6)
 *   last_try: 1 = last try (ORDQ=1), 0 = not last try (ORDQ=0)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_directed_retry(const char *number, int *channels, int num_channels, int last_try)
{
	sender_t *sender;
	amps_t *amps;
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;
	int i;

	/* Convert number to MIN */
	if (amps_number2min(number, &min1, &min2) < 0)
		return -1;

	/* Validate channel count */
	if (num_channels < 1 || num_channels > 6) {
		LOGP(DAMPS, LOGL_ERROR, "Directed Retry: invalid channel count %d (must be 1-6)\n", num_channels);
		return -1;
	}

	/* Validate channel positions (7-bit values, 1-127) */
	for (i = 0; i < num_channels; i++) {
		if (channels[i] < 1 || channels[i] > 127) {
			LOGP(DAMPS, LOGL_ERROR, "Directed Retry: invalid channel position %d (must be 1-127)\n", channels[i]);
			return -1;
		}
	}

	/* Find a control channel to send on and search for existing transaction */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		if (amps->chan_type == CHAN_TYPE_CC || 
		    amps->chan_type == CHAN_TYPE_CC_PC ||
		    amps->chan_type == CHAN_TYPE_CC_PC_VC) {
			trans = search_transaction_number(amps, min1, min2);
			if (trans)
				break;
		}
	}

	/* If no transaction found, search all CCs for one to send from */
	if (!trans) {
		for (sender = sender_head; sender; sender = sender->next) {
			amps = (amps_t *) sender;
			if (amps->chan_type == CHAN_TYPE_CC || 
			    amps->chan_type == CHAN_TYPE_CC_PC ||
			    amps->chan_type == CHAN_TYPE_CC_PC_VC) {
				break;
			}
		}
	}

	if (!sender) {
		LOGP(DAMPS, LOGL_ERROR, "Directed Retry: no control channel available\n");
		return -CAUSE_NOCHANNEL;
	}

	/* Check if there's a transaction for this subscriber */
	if (trans) {
		/* Subscriber has a transaction - check if in valid access state
		 * Per TIA/EIA-553-A, Directed Retry is only valid during access:
		 * Mobile must be on FOCC (not yet tuned to voice channel)
		 */
		int valid = 0;
		switch (trans->state) {
		case TRANS_PAGE_REPLY:           /* MT: responded to page */
		case TRANS_CALL_MO_WAIT_PROCEED: /* MO: waiting for network PROCEEDING */
		case TRANS_CALL_MO_ASSIGN:       /* MO: about to send VC assignment */
		case TRANS_CALL_MO_ASSIGN_SEND:  /* MO: sending VC assignment */
		case TRANS_CALL_MT_ASSIGN:       /* MT: about to send VC assignment */
		case TRANS_CALL_MT_ASSIGN_SEND:  /* MT: sending VC assignment */
			valid = 1;
			break;
		default:
			break;
		}
		if (!valid) {
			LOGP_CHAN(DAMPS, LOGL_ERROR, "Directed Retry: mobile '%s' not in access state!\n", number);
			LOGP_CHAN(DAMPS, LOGL_ERROR, "  Current state: %s - mobile may already be on voice channel\n", 
				trans_short_state_name(trans->state));
			return -CAUSE_BUSY;
		}
		LOGP_CHAN(DAMPS, LOGL_INFO, "Directed Retry: mobile '%s' in %s state\n", 
			number, trans_short_state_name(trans->state));
		
		/* Cancel any pending timer (e.g., network timeout) */
		osmo_timer_del(&trans->timer);
	} else {
		/* No transaction - mobile is idle, Directed Retry won't work */
		LOGP(DAMPS, LOGL_ERROR, "Directed Retry: no active transaction for '%s' - mobile is idle\n", number);
		return -CAUSE_OUTOFORDER;
	}

	/* Store channel positions in transaction */
	for (i = 0; i < num_channels; i++) {
		trans->retry_channels[i] = channels[i];
	}
	trans->retry_num_channels = num_channels;
	trans->retry_last_try = last_try ? 1 : 0;

	/* Set order parameters */
	trans->msg_type = 0;
	trans->ordq = last_try ? 1 : 0;  /* ORDQ=0 = not last try, ORDQ=1 = last try */
	trans->order = 12;               /* Order 12 = Directed Retry */

	LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Directed Retry (Order 12, ORDQ=%d) to '%s'\n", 
		trans->ordq, number);
	LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Directed Retry: %d channel(s), %s\n", 
		num_channels, last_try ? "LAST TRY" : "not last try");
	for (i = 0; i < num_channels; i++) {
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "    Channel position %d: %d\n", i + 1, channels[i]);
	}

	/* Set transaction state to trigger FOCC transmission */
	trans_new_state(trans, TRANS_DIRECTED_RETRY);

	return 0;
}

/*
 * Send Rescan Order - Directed Retry with automatic CHANPOS calculation
 *
 * This is a convenience function that:
 * 1. Priority 1: Add all configured CC (pure control) channels
 * 2. Priority 2: Add all configured CC_PC and CC_PC_VC (combined) channels  
 * 3. Priority 3: Fill remaining slots with dedicated CC range channels
 *
 * Per TIA/EIA-553-A Section 2.6.3.9:
 * - If serving-system enabled (S=1):  Channel = FIRSTCHAs + 1 - CHANPOS
 *   So: CHANPOS = FIRSTCHAs + 1 - Channel
 * - If serving-system disabled (S=0): Channel = FIRSTCHAs - 1 + CHANPOS
 *   So: CHANPOS = Channel - FIRSTCHAs + 1
 *
 * Parameters:
 *   number: AMPS phone number (10 digits)
 *
 * Returns 0 on success, negative error code on failure.
 */
int amps_rescan(const char *number)
{
	sender_t *sender;
	amps_t *amps = NULL;
	amps_t *cc_amps = NULL;  /* The control channel we'll use for system params */
	transaction_t *trans = NULL;
	uint32_t min1;
	uint16_t min2;
	int s_bit = 0;  /* serving-system status */
	int firstcha = 0;
	int chanpos[6];
	int added_channels[6];  /* Track which channels we've added */
	int num_channels = 0;
	int i, pos, chan;
	const char *band;
	int cc_start, cc_end;
	int current_channel = 0;  /* Channel to exclude (either from transaction or sending CC) */

	/* Convert number to MIN to find the mobile's current channel */
	if (amps_number2min(number, &min1, &min2) < 0) {
		LOGP(DAMPS, LOGL_ERROR, "Rescan: invalid number '%s'\n", number);
		return -CAUSE_INVALNUMBER;
	}

	/* Find the transaction for this mobile to determine current channel */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		trans = search_transaction_number(amps, min1, min2);
		if (trans) {
			current_channel = atoi(trans->amps->sender.kanal);
			break;
		}
	}

	/* Find a control channel to get system parameters (and to send from) */
	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		if (amps->chan_type == CHAN_TYPE_CC || 
		    amps->chan_type == CHAN_TYPE_CC_PC ||
		    amps->chan_type == CHAN_TYPE_CC_PC_VC) {
			cc_amps = amps;
			s_bit = amps->si.word2.s;
			break;
		}
	}

	if (!cc_amps) {
		LOGP(DAMPS, LOGL_ERROR, "Rescan: no control channel available\n");
		return -CAUSE_NOCHANNEL;
	}

	/* If no active transaction, exclude the CC we'll be sending from
	 * (the mobile is likely monitoring this channel after registration) */
	if (!current_channel) {
		current_channel = atoi(cc_amps->sender.kanal);
	}

	/* Determine which band/system we're on from first CC */
	int first_cc_chan = atoi(cc_amps->sender.kanal);
	band = amps_channel2band(first_cc_chan);
	
	LOGP(DAMPS, LOGL_NOTICE, "=== Rescan Analysis for '%s' ===\n", number);
	LOGP(DAMPS, LOGL_NOTICE, "Excluding channel %d (current/sending CC)\n", current_channel);
	LOGP(DAMPS, LOGL_NOTICE, "Serving-system status (S): %d (%s)\n", s_bit, 
		s_bit ? "enabled/home" : "disabled/roaming");

	/* Determine FIRSTCHA based on band
	 * Per TIA/EIA-553-A:
	 * - System A: FIRSTCHA=333
	 * - System B: FIRSTCHA=334
	 */
	if (band[0] == 'A') {
		cc_start = 313;
		cc_end = 333;
		firstcha = 333;
		LOGP(DAMPS, LOGL_NOTICE, "System A detected, FIRSTCHA=%d\n", firstcha);
	} else if (band[0] == 'B') {
		cc_start = 334;
		cc_end = 354;
		firstcha = 334;
		LOGP(DAMPS, LOGL_NOTICE, "System B detected, FIRSTCHA=%d\n", firstcha);
	} else {
		LOGP(DAMPS, LOGL_ERROR, "Rescan: unknown band '%s' for channel %d\n", band, first_cc_chan);
		return -CAUSE_INVALNUMBER;
	}

	LOGP(DAMPS, LOGL_NOTICE, "CHANPOS formula: %s\n", 
		s_bit ? "CHANPOS = FIRSTCHA + 1 - Channel" : "CHANPOS = Channel - FIRSTCHA + 1");
	LOGP(DAMPS, LOGL_NOTICE, "---\n");

	/* Helper macro to calculate CHANPOS */
	#define CALC_CHANPOS(ch) (s_bit ? (firstcha + 1 - (ch)) : ((ch) - firstcha + 1))

	/* Priority 1: Add all configured CC (pure control) channels */
	LOGP(DAMPS, LOGL_NOTICE, "Priority 1: Configured CC channels:\n");
	for (sender = sender_head; sender && num_channels < 6; sender = sender->next) {
		amps = (amps_t *) sender;
		if (amps->chan_type == CHAN_TYPE_CC) {
			chan = atoi(amps->sender.kanal);
			/* Skip current channel */
			if (chan == current_channel) {
				LOGP(DAMPS, LOGL_NOTICE, "  Channel %d (CC) - SKIPPED (excluded)\n", chan);
				continue;
			}
			pos = CALC_CHANPOS(chan);
			if (pos >= 1 && pos <= 127) {
				added_channels[num_channels] = chan;
				chanpos[num_channels] = pos;
				LOGP(DAMPS, LOGL_NOTICE, "  [%d] Channel %d (CC) -> CHANPOS %d\n", 
					num_channels + 1, chan, pos);
				num_channels++;
			}
		}
	}
	if (num_channels == 0)
		LOGP(DAMPS, LOGL_NOTICE, "  (none configured)\n");

	/* Priority 2: Add all configured CC_PC and CC_PC_VC (combined) channels */
	LOGP(DAMPS, LOGL_NOTICE, "Priority 2: Configured combined CC+PC channels:\n");
	int added_combined = 0;
	for (sender = sender_head; sender && num_channels < 6; sender = sender->next) {
		amps = (amps_t *) sender;
		if (amps->chan_type == CHAN_TYPE_CC_PC || amps->chan_type == CHAN_TYPE_CC_PC_VC) {
			chan = atoi(amps->sender.kanal);
			/* Skip current channel */
			if (chan == current_channel) {
				LOGP(DAMPS, LOGL_NOTICE, "  Channel %d (%s) - SKIPPED (excluded)\n", 
					chan, amps->chan_type == CHAN_TYPE_CC_PC ? "CC+PC" : "CC+PC+VC");
				continue;
			}
			/* Check if already added */
			int already = 0;
			for (i = 0; i < num_channels; i++) {
				if (added_channels[i] == chan) {
					already = 1;
					break;
				}
			}
			if (already) continue;
			
			pos = CALC_CHANPOS(chan);
			if (pos >= 1 && pos <= 127) {
				added_channels[num_channels] = chan;
				chanpos[num_channels] = pos;
				LOGP(DAMPS, LOGL_NOTICE, "  [%d] Channel %d (%s) -> CHANPOS %d\n", 
					num_channels + 1, chan, 
					amps->chan_type == CHAN_TYPE_CC_PC ? "CC+PC" : "CC+PC+VC", pos);
				num_channels++;
				added_combined++;
			}
		}
	}
	if (added_combined == 0)
		LOGP(DAMPS, LOGL_NOTICE, "  (none configured)\n");

	/* Priority 3: Fill remaining slots with dedicated CC range */
	LOGP(DAMPS, LOGL_NOTICE, "Priority 3: Dedicated CC range (%d-%d):\n", cc_start, cc_end);
	int added_dedicated = 0;
	for (i = cc_start; i <= cc_end && num_channels < 6; i++) {
		/* Skip current channel */
		if (i == current_channel) {
			LOGP(DAMPS, LOGL_NOTICE, "  Channel %d (dedicated CC) - SKIPPED (excluded)\n", i);
			continue;
		}
		/* Check if already added */
		int already = 0;
		for (int j = 0; j < num_channels; j++) {
			if (added_channels[j] == i) {
				already = 1;
				break;
			}
		}
		if (already) continue;
		
		pos = CALC_CHANPOS(i);
		if (pos >= 1 && pos <= 127) {
			added_channels[num_channels] = i;
			chanpos[num_channels] = pos;
			LOGP(DAMPS, LOGL_NOTICE, "  [%d] Channel %d (dedicated CC) -> CHANPOS %d\n", 
				num_channels + 1, i, pos);
			num_channels++;
			added_dedicated++;
		}
	}
	if (added_dedicated == 0)
		LOGP(DAMPS, LOGL_NOTICE, "  (none added - slots full or out of range)\n");

	#undef CALC_CHANPOS

	if (num_channels == 0) {
		LOGP(DAMPS, LOGL_ERROR, "Rescan: no valid channels found for Directed Retry\n");
		return -CAUSE_NOCHANNEL;
	}

	LOGP(DAMPS, LOGL_NOTICE, "---\n");
	LOGP(DAMPS, LOGL_NOTICE, "Sending Directed Retry with %d channel(s)\n", num_channels);
	LOGP(DAMPS, LOGL_NOTICE, "=================================\n");

	/* Call amps_directed_retry with calculated CHANPOS values */
	return amps_directed_retry(number, chanpos, num_channels, 0);  /* not last try */
}

/*
 * Handle PCI Report from mobile station
 *
 * Called when we receive a Protocol Capability Indicator report message
 * from the mobile in response to our PCI query. This can arrive at any time
 * during an active call - we just log it and continue.
 */
void amps_rx_pci_report(amps_t *amps, uint8_t mspc, uint8_t mscap)
{
	transaction_t *trans = amps->trans_list;
	const char *callerid;

	if (!trans) {
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "PCI Report received without transaction (ignored)\n");
		return;
	}

	callerid = amps_min2number(trans->min1, trans->min2);

	/* Log the capabilities - this is informational only */
	LOGP_CHAN(DAMPS, LOGL_INFO, "PCI Report from %s: MSPC=%d (%s), MSCAP=%d (%s)\n",
		callerid, mspc, ie_mspc(mspc), mscap, ie_mscap(mscap));

	/* No state change needed - we're already in TRANS_CALL */
}

/*
 * Handle Release Order from mobile station (Order 3 on RVC)
 *
 * Called when we receive a Release order from the mobile station,
 * indicating the mobile user has hung up.
 *
 * Parameters:
 *   amps: AMPS instance
 *   ordq: Order Qualifier (0=standard release, 2=with DCCI, 3=release complete)
 */
void amps_rx_release_order(amps_t *amps, uint8_t ordq)
{
	transaction_t *trans = amps->trans_list;
	const char *callerid;

	if (!trans) {
		LOGP_CHAN(DAMPS, LOGL_ERROR, "Release Order from mobile without transaction!\n");
		return;
	}

	callerid = amps_min2number(trans->min1, trans->min2);

	/* Log the release with ORDQ variant */
	switch (ordq) {
	case 0:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Mobile %s initiated Release (Order 3, ORDQ=0: standard release)\n", callerid);
		break;
	case 2:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Mobile %s initiated Release (Order 3, ORDQ=2: with Digital Control Channel Info)\n", callerid);
		break;
	case 3:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Mobile %s sent Release Complete (Order 3, ORDQ=3)\n", callerid);
		break;
	default:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Mobile %s initiated Release (Order 3, ORDQ=%d: unknown variant)\n", callerid, ordq);
		break;
	}

	/* Release the call towards the network */
	osmo_timer_del(&trans->timer);
	if (trans->callref) {
		call_up_release(trans->callref, CAUSE_NORMAL);
		trans->callref = 0;
	}
	destroy_transaction(trans);
	amps_go_idle(amps);
}

/*
 * Handle Serial Number Response (Order 15, ORDQ=1) from mobile station
 *
 * This is called when we receive the ESN in response to a Serial Number Request.
 * We compare the received ESN with the stored ESN from call setup to detect
 * potential fraud (cloned phones).
 *
 * Per TIA/EIA-553-A Table 3.7.1-1:
 * - Order 15 (01111) with ORDQ=001 = "Serial Number Request/Response"
 *
 * Parameters:
 *   amps: AMPS instance
 *   esn: 32-bit Electronic Serial Number received from mobile
 */
void amps_rx_esn_response(amps_t *amps, uint32_t esn)
{
	transaction_t *trans = amps->trans_list;
	const char *callerid;
	uint8_t mfr_rx, mfr_stored;
	uint32_t serial_rx, serial_stored;

	if (!trans) {
		LOGP_CHAN(DAMPS, LOGL_ERROR, "ESN Response without transaction!\n");
		return;
	}

	callerid = amps_min2number(trans->min1, trans->min2);

	/* Decode ESN components */
	amps_decode_esn(esn, &mfr_rx, &serial_rx);
	amps_decode_esn(trans->esn, &mfr_stored, &serial_stored);

	LOGP_CHAN(DAMPS, LOGL_INFO, "Received Serial Number Response from '%s'\n", callerid);
	LOGP_CHAN(DAMPS, LOGL_INFO, "  Received ESN: %s (MFR=0x%02x, Serial=%d)\n",
		esn_to_string(esn), mfr_rx, serial_rx);
	LOGP_CHAN(DAMPS, LOGL_INFO, "  Stored ESN:   %s (MFR=0x%02x, Serial=%d)\n",
		esn_to_string(trans->esn), mfr_stored, serial_stored);

	/* Compare ESNs - but only if we have a stored ESN to compare against
	 * For MT calls (paging), trans->esn is 0 because phone doesn't send ESN in page response
	 */
	if (trans->esn == 0) {
		/* No stored ESN - this is normal for MT calls
		 * Store the received ESN for future reference
		 */
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "<<< ESN Received: Mobile '%s' ESN = %s (no prior ESN stored)\n",
			callerid, esn_to_string(esn));
		trans->esn = esn;  /* Store for future comparisons */
	} else if (esn == trans->esn) {
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "<<< ESN Verified: Mobile '%s' ESN matches stored value\n", callerid);
	} else {
		/* ESN mismatch - potential fraud! */
		LOGP_CHAN(DAMPS, LOGL_ERROR, "*** FRAUD ALERT: ESN MISMATCH for '%s' ***\n", callerid);
		LOGP_CHAN(DAMPS, LOGL_ERROR, "  Expected: %s (MFR=0x%02x, Serial=%d)\n",
			esn_to_string(trans->esn), mfr_stored, serial_stored);
		LOGP_CHAN(DAMPS, LOGL_ERROR, "  Received: %s (MFR=0x%02x, Serial=%d)\n",
			esn_to_string(esn), mfr_rx, serial_rx);
		LOGP_CHAN(DAMPS, LOGL_ERROR, "  This may indicate a cloned phone or SIM swap!\n");
	}

	/* No state change - call continues normally
	 * The operator can decide what action to take based on the fraud alert
	 */
}

/* Flash timer callback - automatically send Order 8 (Send Called-Address) after flash
 * Per TIA/EIA-553-A: Mobile only responds to Order 8 within 10 seconds of a flash.
 * We send it ~1 second after flash to give user time to dial digits.
 */
static void flash_timer_callback(void *data)
{
	transaction_t *trans = data;
	amps_t *amps = trans->amps;

	if (trans->state != TRANS_CALL) {
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "Flash timer fired but not in CALL state, ignoring\n");
		return;
	}

	LOGP_CHAN(DAMPS, LOGL_INFO, "Auto-sending Order 8 (Send Called-Address) after flash\n");
	
	/* Set order parameters for Order 8 */
	trans->chan = 0;
	trans->msg_type = 0;
	trans->ordq = 0;       /* ORDQ=0 for Send Called-Address */
	trans->order = 8;      /* Order 8 = Send Called-Address */

	/* Set transaction state to trigger FVC transmission */
	trans_new_state(trans, TRANS_CALL_DIGITS_REQUEST);
	amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
}

/* Timeout handling */
void transaction_timeout(void *data)
{
	transaction_t *trans = data;
	amps_t *amps = trans->amps;

	switch (trans->state) {
	case TRANS_CALL_MO_WAIT_PROCEED:
		/* Network didn't respond with PROCEEDING within timeout
		 * Per TIA/EIA-553-A §2.6.3.8: Mobile waits up to 5 seconds
		 * We use 4 seconds with safety margin
		 */
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "Network timeout waiting for PROCEEDING (4s), sending Reorder\n");
		{
			const char *number = amps_min2number(trans->min1, trans->min2);
			/* Release call towards network if callref exists */
			if (trans->callref) {
				call_up_release(trans->callref, CAUSE_TEMPFAIL);
				trans->callref = 0;
			}
			/* Send Reorder on FOCC - phone will play fast busy and return to idle */
			amps_reorder(number);
		}
		break;
	case TRANS_CALL_MO_ASSIGN_CONFIRM:
	case TRANS_CALL_MT_ASSIGN_CONFIRM:
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "Timeout after %ld seconds not receiving initial SAT signal.\n", trans->timer.timeout.tv_sec);
		LOGP_CHAN(DAMPS, LOGL_INFO, "Release call towards network.\n");
		amps_release(amps->trans_list, CAUSE_TEMPFAIL);
		break;
	case TRANS_CALL:
		/* Check if this is ST Release timeout */
		if (trans->st_start_time != 0.0) {
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** Signaling Tone held > 1.8s - Releasing call ***\n");
			trans->st_start_time = 0.0;
			/* Release call */
			if (trans->callref)
				call_up_release(trans->callref, CAUSE_NORMAL);
			destroy_transaction(trans);
			amps_go_idle(amps);
			break;
		}
		/* Otherwise this is SAT timeout */
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "Timeout after %ld seconds loosing SAT signal.\n", trans->timer.timeout.tv_sec);
		LOGP_CHAN(DAMPS, LOGL_INFO, "Release call towards network.\n");
		amps_release(amps->trans_list, CAUSE_TEMPFAIL);
		break;
	case TRANS_CALL_RELEASE:
	case TRANS_CALL_RELEASE_SEND:
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "Release timeout, destroying transaction\n");
		destroy_transaction(trans);
		amps_go_idle(amps);
		break;
	case TRANS_CALL_MT_ALERT_SEND:
	case TRANS_CALL_MT_ALERT_CONFIRM:
		if (trans->alert_retry++ == ALERT_TRIES) {
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "Phone does not respond to alert order, destroying transaction\n");
			amps_release(trans, CAUSE_TEMPFAIL);
		} else {
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "Phone does not respond to alert order, retrying\n");
			trans_new_state(trans, TRANS_CALL_MT_ALERT);
			amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
		}
		break;
	case TRANS_CALL_MT_ANSWER_WAIT:
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "Alerting timeout, destroying transaction\n");
		amps_release(trans, CAUSE_NOANSWER);
		break;
	case TRANS_PAGE_REPLY:
		if (trans->page_retry++ == PAGE_TRIES) {
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "Paging timeout, destroying transaction\n");
			/* Phone never answered page - no voice channel established.
			 * Just release call control and destroy transaction.
			 * Don't call amps_release() which would try to send Release
			 * order on voice channel that was never set up. */
			if (trans->callref) {
				call_up_release(trans->callref, CAUSE_OUTOFORDER);
				trans->callref = 0;
			}
			destroy_transaction(trans);
			amps_go_idle(amps);
		} else {
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "Paging timeout, retrying\n");
			trans_new_state(trans, TRANS_PAGE);
		}
		break;
	/* Silent Page timeout handling */
	case TRANS_SILENT_PAGE_REPLY:
		if (trans->page_retry++ == PAGE_TRIES) {
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** Silent Page FAILED: Mobile '%s' did not respond to page ***\n",
				amps_min2number(trans->min1, trans->min2));
			destroy_transaction(trans);
			amps_go_idle(amps);
		} else {
			LOGP_CHAN(DAMPS, LOGL_NOTICE, "Silent Page: paging timeout, retrying\n");
			trans_new_state(trans, TRANS_SILENT_PAGE);
		}
		break;
	case TRANS_SILENT_PAGE_ASSIGN_CONFIRM:
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** Silent Page FAILED: Mobile '%s' did not confirm SAT ***\n",
			amps_min2number(trans->min1, trans->min2));
		destroy_transaction(trans);
		amps_go_idle(amps);
		break;
	case TRANS_SILENT_PAGE_MAINTENANCE_SEND:
		/* Maintenance order timeout - mobile did not respond with ST
		 * Per TIA/EIA-553-A: Mobile should respond with ST within ~500ms
		 * If no ST, the mobile may be malfunctioning or out of range
		 */
		LOGP_CHAN(DAMPS, LOGL_NOTICE, "*** Silent Page FAILED: Mobile '%s' did not confirm Maintenance (no ST) ***\n",
			amps_min2number(trans->min1, trans->min2));
		LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: sending Release to terminate\n");
		/* Send Release order to cleanly terminate */
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;
		trans->order = 3;  /* Release */
		trans_new_state(trans, TRANS_CALL_RELEASE);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_FRAME_TX, 0);
		break;
	case TRANS_CALL_HANDOFF_SEND:
		/* Handoff timeout - mobile did not respond with ST
		 * Per TIA/EIA-553-A: Mobile should respond with 50ms ST
		 * If no ST, the mobile may have lost signal or failed to decode
		 */
		{
			amps_t *target_vc;
			int target_channel = trans->handoff_channel;
			
			LOGP_CHAN(DAMPS, LOGL_ERROR, "Handoff FAILED: Mobile did not confirm (no ST within 3s)\n");
			
			/* Release the target channel that we prepared */
			if (target_channel > 0) {
				target_vc = search_channel(target_channel);
				if (target_vc && target_vc->state == STATE_BUSY && target_vc != amps) {
					LOGP(DAMPS, LOGL_INFO, "(chan %d) Releasing prepared target channel\n", target_channel);
					amps_go_idle(target_vc);
				}
			}
			
			LOGP_CHAN(DAMPS, LOGL_INFO, "Returning to normal call state\n");
			/* Clear handoff fields and return to call state */
			trans->chan = 0;
			trans->handoff_channel = 0;
			trans->handoff_scc = -1;
			trans_new_state(trans, TRANS_CALL);
			amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		};
		break;
	default:
		LOGP_CHAN(DAMPS, LOGL_ERROR, "Timeout unhandled in state %d\n", trans->state);
	}
}

/* assigning voice channel and moving transaction+callref to that channel */
static amps_t *assign_voice_channel(transaction_t *trans)
{
	amps_t *amps = trans->amps, *vc;
	const char *callerid = amps_min2number(trans->min1, trans->min2);
	int is_silent_page = (trans->state == TRANS_SILENT_PAGE_ASSIGN_SEND);

	vc = search_channel(trans->chan);
	if (!vc) {
		LOGP(DAMPS, LOGL_NOTICE, "Channel %d is not free anymore, rejecting call\n", trans->chan);
		if (!is_silent_page)
			amps_release(trans, CAUSE_NOCHANNEL);
		else {
			/* Silent page: just destroy transaction and go idle */
			LOGP(DAMPS, LOGL_NOTICE, "*** Silent Page FAILED: No free channel ***\n");
			destroy_transaction(trans);
			amps_go_idle(amps);
		}
		return NULL;
	}

	if (vc == amps)
		LOGP(DAMPS, LOGL_INFO, "Staying on combined control + voice channel %s\n", vc->sender.kanal);
	else
		LOGP(DAMPS, LOGL_INFO, "Moving to voice channel %s\n", vc->sender.kanal);

	LOGP(DAMPS, LOGL_INFO, "Assigning Channel with Initial VMAC: %d (Max Allowed: %d)\n", trans->current_vmac, trans->max_vmac);

	/* switch channel... */
	osmo_timer_schedule(&trans->timer, SAT_TO1);
	/* make channel busy */
	amps_new_state(vc, STATE_BUSY);
	/* relink */
	unlink_transaction(trans);
	link_transaction(trans, vc);
	/* flush all other transactions, if any (in case of combined VC + CC) */
	amps_flush_other_transactions(vc, trans);

	/* Skip call setup for silent page - it's just a test, not a real call */
	if (!trans->callref && !is_silent_page) {
		char esn_text[16];
		sprintf(esn_text, "%u", trans->esn);
		/* setup call */
		LOGP(DAMPS, LOGL_INFO, "Setup call to network.\n");
		trans->callref = call_up_setup(callerid, trans->dialing, OSMO_CC_NETWORK_AMPS_ESN, esn_text);
	}

	return vc;
}

transaction_t *amps_tx_frame_focc(amps_t *amps)
{
	transaction_t *trans;
	amps_t *vc;
	
again:
	trans = amps->trans_list;
	if (!trans)
		return NULL;

	switch (trans->state) {
	case TRANS_REGISTER_ACK:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Register acknowledge\n");
		trans_new_state(trans, TRANS_REGISTER_ACK_SEND);
		return trans;
	case TRANS_REGISTER_ACK_SEND:
		destroy_transaction(trans);
		goto again;
	case TRANS_CALL_REJECT:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Rejecting call from mobile station\n");
		trans_new_state(trans, TRANS_CALL_REJECT_SEND);
		return trans;
	case TRANS_CALL_REJECT_SEND:
		destroy_transaction(trans);
		goto again;
	case TRANS_CALL_MO_ASSIGN:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Assigning channel to call from mobile station\n");
		trans_new_state(trans, TRANS_CALL_MO_ASSIGN_SEND);
		return trans;
	case TRANS_CALL_MO_ASSIGN_SEND:
		vc = assign_voice_channel(trans);
		if (vc) {
			LOGP_CHAN(DAMPS, LOGL_INFO, "Assignment complete, voice connected\n");
			/* timer and other things are processed at assign_voice_channel() */
			trans_new_state(trans, TRANS_CALL_MO_ASSIGN_CONFIRM);
			amps_set_dsp_mode(vc, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		}
		return NULL;
	case TRANS_CALL_MT_ASSIGN:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Assigning channel to call to mobile station\n");
		trans_new_state(trans, TRANS_CALL_MT_ASSIGN_SEND);
		return trans;
	case TRANS_CALL_MT_ASSIGN_SEND:
		vc = assign_voice_channel(trans);
		if (vc) {
			LOGP_CHAN(DAMPS, LOGL_INFO, "Assignment complete, waiting for SAT on VC\n");
			/* timer and other things are processed at assign_voice_channel() */
			trans_new_state(trans, TRANS_CALL_MT_ASSIGN_CONFIRM);
			amps_set_dsp_mode(vc, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		}
		return NULL;
	case TRANS_PAGE:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Paging the phone\n");
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "DEBUG: Creating PAGE transaction on channel %s, MIN1=%u MIN2=%u\n", 
			amps->sender.kanal, trans->min1, trans->min2);
		trans_new_state(trans, TRANS_PAGE_SEND);
		return trans;
	case TRANS_AUDIT:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Audit Order on FOCC\n");
		trans_new_state(trans, TRANS_AUDIT_SEND);
		return trans;
	case TRANS_AUDIT_SEND:
		/* Frame data copied to amps->tx_focc_*, frame.c handles repeat. */
		destroy_transaction(trans);
		goto again;
	case TRANS_PCI:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending PCI Query (Order=26, ORDQ=4) on FOCC\n");
		trans_new_state(trans, TRANS_PCI_SEND);
		return trans;
	case TRANS_PCI_SEND:
		/* Same pattern as TRANS_AUDIT_SEND - fire and forget */
		destroy_transaction(trans);
		goto again;
	case TRANS_MWI:
		/* Message Waiting Indicator on FOCC - send to idle phone
		 * Per TIA/EIA-553-A, MWI can be sent on FOCC after PCI order confirmation.
		 * We use this to send MWI to phones that are idle (not in a call).
		 * trans->msg_type = count (0-31)
		 * trans->ordq = type (0=voice, 1=SMS, 2=fax)
		 * trans->order = 5 (Message Waiting)
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Message Waiting (Order=5, ORDQ=%d, count=%d) on FOCC\n",
			trans->ordq, trans->msg_type);
		trans_new_state(trans, TRANS_MWI_SEND);
		return trans;
	case TRANS_MWI_SEND:
		/* MWI sent on FOCC - fire and forget, destroy transaction */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Message Waiting order sent on FOCC\n");
		destroy_transaction(trans);
		goto again;
	case TRANS_DIRECTED_RETRY:
		/* Directed Retry Order (Order 12) on FOCC
		 * This is a 4-word message:
		 * - Word 1: Abbreviated Address (MIN1)
		 * - Word 2: Extended Address Word A (MIN2, ORDER=12, ORDQ)
		 * - Word 3: First Directed-Retry Word (up to 3 CHANPOS)
		 * - Word 4: Second Directed-Retry Word (up to 3 more CHANPOS)
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Directed Retry (Order=12, ORDQ=%d) on FOCC\n",
			trans->ordq);
		trans_new_state(trans, TRANS_DIRECTED_RETRY_SEND);
		return trans;
	case TRANS_DIRECTED_RETRY_SEND:
		/* Directed Retry sent - fire and forget, destroy transaction */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Directed Retry order sent on FOCC\n");
		destroy_transaction(trans);
		goto again;
	case TRANS_REORDER:
		/* Reorder (Order 4) on FOCC during call setup
		 * Phone is still listening on FOCC, not yet tuned to voice channel.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Reorder order (Order 4, ORDQ=0) on FOCC\n");
		trans_new_state(trans, TRANS_REORDER_SEND);
		return trans;
	case TRANS_REORDER_SEND:
		/* Reorder sent on FOCC - mobile returns to idle automatically */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Reorder order sent on FOCC - mobile will return to idle\n");
		destroy_transaction(trans);
		goto again;
	case TRANS_INTERCEPT:
		/* Intercept (Order 9) on FOCC during call setup
		 * Phone is still listening on FOCC, not yet tuned to voice channel.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Intercept order (Order 9, ORDQ=0) on FOCC\n");
		trans_new_state(trans, TRANS_INTERCEPT_SEND);
		return trans;
	case TRANS_INTERCEPT_SEND:
		/* Intercept sent on FOCC - mobile returns to idle automatically */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Intercept order sent on FOCC - mobile will return to idle\n");
		destroy_transaction(trans);
		goto again;
	case TRANS_PAGE_SEND:
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "DEBUG: PAGE sent, moving to PAGE_REPLY state, MIN1=%u MIN2=%u\n", 
			trans->min1, trans->min2);
		trans_new_state(trans, TRANS_PAGE_REPLY);
		if (trans->page_retry == PAGE_TRIES)
			osmo_timer_schedule(&trans->timer, PAGE_TO2);
		else
			osmo_timer_schedule(&trans->timer, PAGE_TO1);
		return NULL;
	/* Silent Page states - page mobile for maintenance test */
	case TRANS_SILENT_PAGE:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: paging the phone for maintenance test\n");
		trans_new_state(trans, TRANS_SILENT_PAGE_SEND);
		return trans;
	case TRANS_SILENT_PAGE_SEND:
		LOGP_CHAN(DAMPS, LOGL_DEBUG, "Silent Page: page sent, waiting for reply\n");
		trans_new_state(trans, TRANS_SILENT_PAGE_REPLY);
		if (trans->page_retry == PAGE_TRIES)
			osmo_timer_schedule(&trans->timer, PAGE_TO2);
		else
			osmo_timer_schedule(&trans->timer, PAGE_TO1);
		return NULL;
	case TRANS_SILENT_PAGE_ASSIGN:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: assigning voice channel\n");
		trans_new_state(trans, TRANS_SILENT_PAGE_ASSIGN_SEND);
		return trans;
	case TRANS_SILENT_PAGE_ASSIGN_SEND:
		vc = assign_voice_channel(trans);
		if (vc) {
			LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: assignment complete, waiting for SAT\n");
			trans_new_state(trans, TRANS_SILENT_PAGE_ASSIGN_CONFIRM);
			amps_set_dsp_mode(vc, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		}
		return NULL;
	default:
		return NULL;
	}
}

transaction_t *amps_tx_frame_fvc(amps_t *amps)
{
	transaction_t *trans = amps->trans_list;

	trans = amps->trans_list;
	if (!trans)
		return NULL;

	switch (trans->state) {
	case TRANS_CALL_CHANGE_POWER:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Change Power to Level %d\n", trans->current_vmac);
		LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Change Power Order: Level %d -> %s\n", 
			trans->current_vmac, amps_power_level_name(trans->current_vmac));
		trans->chan = 0;  /* Use Word1_A format, not channel assignment */
		trans->msg_type = 0;  /* LOCAL_MSG_TYPE = 00000 */
		trans->ordq = trans->current_vmac; /* Power Level (0-7) */
		trans->order = 11; /* Order 11 = Change Power */
		trans_new_state(trans, TRANS_CALL_CHANGE_POWER_SEND);
		return trans;
	case TRANS_CALL_CHANGE_POWER_SEND:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Change Power sent\n");
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		/* Set grace period to ignore SAT drop caused by interruption (1/2 of SAT loss timer) */
		trans->vmac_grace_count = 75; /* ~2.5 seconds worth of samples */
		return NULL;
	case TRANS_CALL_FLASH_INFO:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Flash With Info order\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;
		trans->order = 18;  /* Flash With Info */
		trans_new_state(trans, TRANS_CALL_FLASH_INFO_SEND);
		return trans;
	case TRANS_CALL_FLASH_INFO_SEND:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Flash With Info sent\n");
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		return NULL;
	case TRANS_CALL_PCI_QUERY:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Protocol Capability Indicator order (Order=26, ORDQ=4)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 4;    /* ORDQ for PCI */
		trans->order = 26;  /* Protocol Capability Indicator */
		trans_new_state(trans, TRANS_CALL_PCI_QUERY_SEND);
		return trans;
	case TRANS_CALL_PCI_QUERY_SEND:
		/* PCI Query sent - return to call immediately, response handled asynchronously in frame.c */
		LOGP_CHAN(DAMPS, LOGL_INFO, "PCI Query sent, returning to call (response handled asynchronously)\n");
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		return NULL;
	case TRANS_CALL_AUDIT:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Audit order\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Audit */
		trans->order = 7;   /* Audit Order */
		trans_new_state(trans, TRANS_CALL_AUDIT_SEND);
		return trans;
	case TRANS_CALL_AUDIT_SEND:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Audit Order sent\n");
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		return NULL;
	case TRANS_CALL_ABB_ALERT:
		/* Abbreviated Alert (Order 1, ORDQ=1) - feature reminder tone
		 * Requirements: 3.3, 3.6
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Abbreviated Alert order (ORDQ=1)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 1;    /* ORDQ=1 for abbreviated alert */
		trans->order = 1;   /* Order 1 = Alert */
		trans_new_state(trans, TRANS_CALL_ABB_ALERT_SEND);
		return trans;
	case TRANS_CALL_ABB_ALERT_SEND:
		/* Abbreviated Alert sent, wait for ST confirmation */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Abbreviated Alert sent, waiting for ST confirmation\n");
		osmo_timer_schedule(&trans->timer, ALERT_TO);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		trans_new_state(trans, TRANS_CALL_MT_ALERT_CONFIRM);
		return NULL;
	case TRANS_CALL_RELEASE:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Release order (Order 3, ORDQ=%d) to mobile station\n", trans->ordq);
		trans_new_state(trans, TRANS_CALL_RELEASE_SEND);
		return trans;
	case TRANS_CALL_RELEASE_SEND:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Release order sent (Order 3, ORDQ=%d), continue sending\n", trans->ordq);
		return trans;
	case TRANS_CALL_REORDER:
		/* Reorder (Order 4, ORDQ=0) - fast busy tone
		 * Requirements: 5.1, 5.2, 5.3
		 * No confirmation expected from mobile
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Reorder order (Order 4, ORDQ=0)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Reorder */
		trans->order = 4;   /* Order 4 = Reorder */
		trans_new_state(trans, TRANS_CALL_REORDER_SEND);
		return trans;
	case TRANS_CALL_REORDER_SEND:
		/* Reorder sent - per TIA/EIA-553-A Section 2.6.3.8:
		 * "If the access is an origination: Reorder: The mobile station shall
		 * enter the Serving-System Determination Task"
		 *
		 * This means the mobile automatically returns to idle after receiving
		 * Reorder. NO RELEASE ORDER IS NEEDED. The base station should clean
		 * up the transaction and go idle.
		 *
		 * Note: Reorder is only valid during call setup (TRANS_CALL_MO_ASSIGN_CONFIRM).
		 * If sent during active call (TRANS_CALL), it has no effect on the mobile.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Reorder order sent - mobile will return to idle automatically (no Release needed)\n");
		destroy_transaction(trans);
		amps_go_idle(amps);
		return NULL;
	case TRANS_CALL_MWI:
		/* Message Waiting (Order 5) - voicemail/SMS/fax indicator
		 * Requirements: 6.1, 6.2, 6.3, 6.4
		 * No confirmation expected from mobile
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Message Waiting order (Order 5, ORDQ=%d, count=%d)\n",
			trans->ordq, trans->msg_type);
		/* trans->ordq already set (type), trans->msg_type already set (count) */
		trans->chan = 0;
		trans->order = 5;   /* Order 5 = Message Waiting */
		trans_new_state(trans, TRANS_CALL_MWI_SEND);
		return trans;
	case TRANS_CALL_MWI_SEND:
		/* MWI sent - return to the state we were in before MWI
		 * Per TIA/EIA-553-A:
		 * - During Conversation Task: return to TRANS_CALL, resume audio
		 * - During Waiting for Answer Task: return to alerting state, keep waiting
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Message Waiting order sent, returning to %s state\n",
			(trans->mwi_return_state == TRANS_CALL) ? "CALL" : "alerting");
		trans_new_state(trans, trans->mwi_return_state);
		if (trans->mwi_return_state == TRANS_CALL) {
			amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		} else {
			/* Alerting state - keep silence TX, waiting for answer */
			amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		}
		return NULL;
	case TRANS_CALL_STOP_ALERT:
		/* Stop Alert (Order 6, ORDQ=0) - stop ringing
		 * Requirements: 7.1, 7.2, 7.3, 7.4
		 * No confirmation expected from mobile
		 * Channel is NOT torn down - mobile remains on channel
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Stop Alert order (Order 6, ORDQ=0)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Stop Alert */
		trans->order = 6;   /* Order 6 = Stop Alert */
		trans_new_state(trans, TRANS_CALL_STOP_ALERT_SEND);
		return trans;
	case TRANS_CALL_STOP_ALERT_SEND:
		/* Stop Alert sent, return to call state (no confirmation expected)
		 * The mobile stops ringing but stays on the voice channel
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Stop Alert order sent\n");
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		return NULL;
	case TRANS_CALL_INTERCEPT:
		/* Intercept (Order 9, ORDQ=0) - number cannot be completed
		 * Requirements: 10.1, 10.2, 10.3
		 * No confirmation expected from mobile
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Intercept order (Order 9, ORDQ=0)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Intercept */
		trans->order = 9;   /* Order 9 = Intercept */
		trans_new_state(trans, TRANS_CALL_INTERCEPT_SEND);
		return trans;
	case TRANS_CALL_INTERCEPT_SEND:
		/* Intercept sent - per TIA/EIA-553-A Section 2.6.3.8:
		 * "If the access is an origination: Intercept: The mobile station shall
		 * enter the Serving-System Determination Task"
		 *
		 * This means the mobile automatically returns to idle after receiving
		 * Intercept. NO RELEASE ORDER IS NEEDED. The base station should clean
		 * up the transaction and go idle.
		 *
		 * Note: Intercept is only valid during MO call setup (TRANS_CALL_MO_ASSIGN_CONFIRM).
		 * It is NOT valid for MT calls or during active call (TRANS_CALL).
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Intercept order sent - mobile will return to idle automatically (no Release needed)\n");
		destroy_transaction(trans);
		amps_go_idle(amps);
		return NULL;
	case TRANS_CALL_DIGITS_REQUEST:
		/* Send Called-Address (Order 8, ORDQ=0) - request dialed digits
		 * Requirements: 9.1, 9.2, 9.3
		 * Mobile responds with Called Address message (T=0 on RVC)
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Send Called-Address order (Order 8, ORDQ=0)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Send Called-Address */
		trans->order = 8;   /* Order 8 = Send Called-Address */
		trans_new_state(trans, TRANS_CALL_DIGITS_REQUEST_SEND);
		return trans;
	case TRANS_CALL_DIGITS_REQUEST_SEND:
		/* Send Called-Address sent - wait for response
		 * Per TIA/EIA-553-A: Mobile responds with Called Address message
		 * containing dialed digits. Response handled in amps_rx_recc().
		 * Set a timeout in case mobile doesn't respond.
		 * Use AUDIO_RX mode to receive the RVC response while sending silence.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Send Called-Address order sent, waiting for digits response\n");
		osmo_timer_schedule(&trans->timer, 5, 0);  /* 5 second timeout */
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		trans_new_state(trans, TRANS_CALL);  /* Return to call state, response handled asynchronously */
		return NULL;
	case TRANS_CALL_MAINTENANCE:
		/* Maintenance (Order 10, ORDQ=0) - silent test
		 * Requirements: 11.1, 11.2, 11.3, 11.4
		 * Mobile confirms with ST but doesn't ring
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Maintenance order (Order 10, ORDQ=0)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Maintenance */
		trans->order = 10;  /* Order 10 = Maintenance */
		trans_new_state(trans, TRANS_CALL_MAINTENANCE_SEND);
		return trans;
	case TRANS_CALL_MAINTENANCE_SEND:
		/* Maintenance sent - wait for ST confirmation
		 * Per TIA/EIA-553-A: Mobile turns on ST for 500ms
		 * ST confirmation handled in amps_rx_signaling_tone()
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Maintenance order sent, waiting for ST confirmation\n");
		osmo_timer_schedule(&trans->timer, ALERT_TO);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		return NULL;
	case TRANS_CALL_ESN_REQUEST:
		/* Serial Number Request (Order 15, ORDQ=1) - request ESN
		 * Per TIA/EIA-553-A Table 3.7.1-1:
		 * - Order 15 (01111) with ORDQ=001 = "Serial Number Request/Response"
		 * - Order 15 (01111) with ORDQ=000 = "Parameter Update Order/Confirmation"
		 * Mobile responds with Serial Number Response on RVC containing ESN
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Serial Number Request order (Order 15, ORDQ=1)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		/* trans->ordq already set to 1 by amps_serial_number_request() */
		trans->order = 15;  /* Order 15 = Serial Number Request */
		trans_new_state(trans, TRANS_CALL_ESN_REQUEST_SEND);
		return trans;
	case TRANS_CALL_ESN_REQUEST_SEND:
		/* Serial Number Request sent - wait for response
		 * Per TIA/EIA-553-A: Mobile responds with Serial Number Response
		 * containing ESN. Response handled in frame.c.
		 * Set a timeout in case mobile doesn't respond.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Serial Number Request sent, waiting for ESN response\n");
		osmo_timer_schedule(&trans->timer, 5, 0);  /* 5 second timeout */
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		trans_new_state(trans, TRANS_CALL);  /* Return to call state, response handled asynchronously */
		return NULL;
	case TRANS_CALL_LOCAL_CONTROL:
		/* Local Control (Order 30, ORDQ=0) - vendor-specific action
		 * Requirements: 19.1, 19.2, 19.3
		 * No confirmation expected from mobile
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Local Control order (Order 30, code=%d)\n", trans->msg_type);
		trans->chan = 0;
		/* trans->msg_type already set to local control code */
		trans->ordq = 0;    /* ORDQ=0 for Local Control */
		trans->order = 30;  /* Order 30 = Local Control */
		trans_new_state(trans, TRANS_CALL_LOCAL_CONTROL_SEND);
		return trans;
	case TRANS_CALL_LOCAL_CONTROL_SEND:
		/* Local Control sent - return to call state (no confirmation expected)
		 * Per TIA/EIA-553-A: Local Control is vendor-specific, no standard response
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Local Control order sent\n");
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		return NULL;
	case TRANS_CALL_DISABLE_DTMF:
		/* Disable DTMF (Order 22, ORDQ=0) - mute mobile's DTMF generator
		 * Per TIA/EIA-553-A Section 2.6.4.4:
		 * Mobile confirms with digital Order Confirmation message.
		 * DTMF stays disabled until Called-Address Message is transmitted.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Disable DTMF order (Order 22, ORDQ=0)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Disable DTMF */
		trans->order = 22;  /* Order 22 = Disable DTMF */
		trans_new_state(trans, TRANS_CALL_DISABLE_DTMF_SEND);
		return trans;
	case TRANS_CALL_DISABLE_DTMF_SEND:
		/* Disable DTMF sent - wait for Order Confirmation
		 * Per TIA/EIA-553-A: Mobile sends digital Order Confirmation on RVC.
		 * Confirmation is logged in frame.c when received.
		 * Return to call state - confirmation handled asynchronously.
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Disable DTMF order sent, waiting for confirmation\n");
		trans_new_state(trans, TRANS_CALL);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_AUDIO_TX, 0);
		return NULL;
	case TRANS_CALL_HANDOFF:
		/* Handoff message - transfer mobile to new voice channel
		 * Per TIA/EIA-553-A Section 2.6.4.4:
		 * Mobile responds with 50ms ST, then tunes to new channel
		 */
		LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Handoff Step 1: Sending handoff message\n");
		LOGP_CHAN(DAMPS, LOGL_INFO, "    Target channel: %d\n", trans->handoff_channel);
		LOGP_CHAN(DAMPS, LOGL_INFO, "    PSCC (current SAT): %d\n", amps->sat);
		LOGP_CHAN(DAMPS, LOGL_INFO, "    SCC (target SAT): %d\n", trans->handoff_scc);
		LOGP_CHAN(DAMPS, LOGL_INFO, "    VMAC: %d\n", trans->current_vmac);
		/* trans->chan already set to target channel (triggers Word1_B format) */
		trans_new_state(trans, TRANS_CALL_HANDOFF_SEND);
		return trans;
	case TRANS_CALL_HANDOFF_SEND:
		/* Handoff message sent - wait for ST confirmation (50ms)
		 * Per TIA/EIA-553-A: Mobile sends 50ms ST before tuning to new channel
		 * Extended timeout to 3s to account for SDR TX/RX latency
		 */
		LOGP_CHAN(DAMPS, LOGL_NOTICE, ">>> Handoff Step 2: Message sent, waiting for ST (50ms)\n");
		LOGP_CHAN(DAMPS, LOGL_INFO, "    Fast ST detection enabled (20ms window)\n");
		LOGP_CHAN(DAMPS, LOGL_INFO, "    Timeout: 3 seconds (extended for SDR latency)\n");
		osmo_timer_schedule(&trans->timer, 3, 0);  /* 3 second timeout for ST (extended for SDR latency) */
		/* Reset fast ST detection state for clean detection */
		amps->fast_st_detected = 0;
		amps->fast_st_count = 0;
		amps->fast_st_pos = 0;
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		return NULL;
	case TRANS_CALL_MT_ALERT:
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;
		/* "Alert with caller ID" causes older phones to interrupt the connection for some reason,
		 * therefore we don't use order 17 when no caller ID is set.
		 * If the system does not receive a confirmation for an Alert With Info order, in addition,
		 * the system shall send an Alert order to provide backwards compatibility for
		 * ANSI EIA/TIA 553 (1989) mobile stations.
		 */
		if (amps->send_callerid && trans->alert_retry == 1 && trans->caller_id[0]) {
			LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Alert With Info (order 17) with caller ID\n");
			trans->order = 17;
		} else {
			if (amps->send_callerid && trans->alert_retry > 1 && trans->caller_id[0]) {
				LOGP_CHAN(DAMPS, LOGL_INFO, "No confirmation for Alert With Info order, sending Alert order for backwards compatibility with ANSI EIA/TIA 553 (1989) mobile stations\n");
			} else {
				LOGP_CHAN(DAMPS, LOGL_INFO, "Sending Alert order\n");
			}
			trans->order = 1;
		}
		trans_new_state(trans, TRANS_CALL_MT_ALERT_SEND);
		return trans;
	case TRANS_CALL_MT_ALERT_SEND:
		LOGP_CHAN(DAMPS, LOGL_INFO, "Alerting was sent, continue waiting for ST or timeout\n");
		osmo_timer_schedule(&trans->timer, ALERT_TO);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		trans_new_state(trans, TRANS_CALL_MT_ALERT_CONFIRM);
		return NULL;
	/* Silent Page: Maintenance order on FVC */
	case TRANS_SILENT_PAGE_MAINTENANCE:
		/* Silent Page: Send Maintenance order (Order 10) instead of Alert
		 * Per TIA/EIA-553-A Section 2.6.4.4:
		 * - Mobile turns on ST for 500ms to confirm
		 * - Mobile enters "Waiting for Answer Task" with 65-second timer
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: sending Maintenance order (Order 10, ORDQ=0)\n");
		trans->chan = 0;
		trans->msg_type = 0;
		trans->ordq = 0;    /* ORDQ=0 for Maintenance */
		trans->order = 10;  /* Order 10 = Maintenance */
		trans_new_state(trans, TRANS_SILENT_PAGE_MAINTENANCE_SEND);
		return trans;
	case TRANS_SILENT_PAGE_MAINTENANCE_SEND:
		/* Silent Page: Maintenance sent - wait for ST confirmation
		 * Per TIA/EIA-553-A: Mobile turns on ST for 500ms
		 * ST confirmation handled in amps_rx_signaling_tone()
		 * Use 5 second timeout (ALERT_TO) for ST confirmation
		 */
		LOGP_CHAN(DAMPS, LOGL_INFO, "Silent Page: Maintenance order sent, waiting for ST confirmation (5s timeout)\n");
		osmo_timer_schedule(&trans->timer, ALERT_TO);
		amps_set_dsp_mode(amps, DSP_MODE_AUDIO_RX_SILENCE_TX, 0);
		return NULL;
	default:
		return NULL;
	}
}

void dump_info(void) {}

