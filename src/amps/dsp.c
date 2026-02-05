/* AMPS audio processing
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

/* How does FSK decoding work:
 * ---------------------------
 *
 * AMPS modulates the carrier frequency. If it is 8 kHz above, it is high level,
 * if it is 8 kHz below, it is low level.  The bits are coded using Manchester
 * code. A 1 is coded by low level, followed by a high level. A 0 is coded by
 * a high level, followed by a low level. This will cause at least one level
 * change within each bit.  Also the level changes between equal bits, see
 * Manchester coding.  The bit rate is 10 KHz.
 *
 * In order to detect and demodulate a frame, the dotting sequnce is searched.
 * The dotting sequnece are alternate bits: 101010101...  The duration of a
 * level change within the dotting sequnene ist 100uS.  If all offsets of 8
 * level changes lay within +-50% of the expected time, the dotting sequence is
 * valid.  Now the next 12 bits will be searched for sync sequnece.  If better
 * dotting-offsets are found, the counter for searching the sync sequence is
 * reset, so the next 12 bits will be searched for sync too.  If no sync was
 * detected, the state changes to search for next dotting sequence.
 *
 * The average level change offsets of the dotting sequence is used to set the
 * window for the first bit.  When all samples for the window are received, a
 * raise in level is detected as 1, fall in level is detected as 0. This is done
 * by subtracting the average sample value of the left side of the window by
 * the average sample value of the right side.  After the bit has been detected,
 * the samples for the next window will be received and detected.
 *
 * +-----+-----+-----+-----+
 * |     |     |   __|__   |
 * |     |     |  /  |  \  |
 * |     |     | /   |   \ |
 * |     |     |/    |    \|
 * +-----+-----+-----+-----+
 * |\    |    /|     |     |
 * | \   |   / |     |     |
 * |  \__|__/  |     |     |
 * |     |     |     |     |
 * +-----+-----+-----+-----+
 *       End   Half  Begin
 *
 * The Rx window is depiced above. In this example there is a raising edge.
 * The window is analyzed in backward direction. The average level between
 * 'Half' position and 'Begin' position is calculated, also the average level
 * between 'End' position and 'Half' position. Because the right (second)
 * side of the average level is higher than the left (first) side, a raising
 * edge is detected.
 *
 * Tests showed that comparing half of the regions of the window will cause
 * more errors than only quarter regions of the regions. Especially this is
 * true with NBFM receivers that are normally not sufficient for AMPS signals.
 *
 * As soon as a sync pattern is detected, the polarity of the pattern is used
 * to decode the following frame bits with correct polarity.  During reception
 * of the frame bits, no sync and no dotting sequnece is searched or detected.
 *
 * After reception of the bit, the bits are re-assembled, parity checked and
 * decoded. Then the process hunts for next dotting sequence.  
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/get_time.h"
#include "amps.h"
#include "frame.h"
#include "dsp.h"
#include "../libclipper/clipper.h"
#include "main.h"

#define CHAN amps->sender.kanal

/* uncomment this to debug the encoding process */
//#define DEBUG_ENCODER

/* uncomment this to debug the decoding process */
//#define DEBUG_DECODER

#define PI			M_PI

#define AMPS_MAX_DEVIATION	8000.0
#define AMPS_MAX_MODULATION	10000.0
#define AMPS_SPEECH_DEVIATION	2900.0  /* deviation of speech at 1 kHz */
#define AMPS_FSK_DEVIATION	(8000.0 / AMPS_SPEECH_DEVIATION)	/* no emphasis */
#define AMPS_SAT_DEVIATION	(2000.0 / AMPS_SPEECH_DEVIATION)	/* no emphasis */
#define AMPS_MAX_DISPLAY	(10000.0 / AMPS_SPEECH_DEVIATION)	/* no emphasis */
#define AMPS_BITRATE		10000
/* TACS speech deviation: Per Panasonic manual, 2300 Hz at 1 kHz reference.
 * This is lower than AMPS (2900 Hz) because TACS uses narrower deviation.
 * Using 2900 Hz causes feedback loops due to excessive TX level. */
#define TACS_SPEECH_DEVIATION	2300.0
#define TACS_MAX_DEVIATION	9500.0	/* (according to wikipedia) */
#define TACS_MAX_MODULATION	9500.0	/* (according to panasonic manual) */
#define TACS_FSK_DEVIATION	(6400.0 / TACS_SPEECH_DEVIATION)	/* no emphasis */
#define TACS_SAT_DEVIATION	(1700.0 / TACS_SPEECH_DEVIATION)	/* no emphasis (panasonic / TI) */
#define TACS_MAX_DISPLAY	(8000.0 / TACS_SPEECH_DEVIATION)	/* no emphasis */
#define TACS_BITRATE		8000
#define SAT_BANDWIDTH		30.0	/* distance between two SAT tones, also bandwidth for goertzel filter */
#define SAT_QUALITY		0.85	/* quality needed to detect SAT signal */
#define SAT_PRINT		10	/* print sat measurement every 0.5 seconds */
#define DTX_LEVEL		0.50	/* SAT level needed to mute/unmute */
#define SIG_QUALITY		0.80	/* quality needed to detect Signaling Tone */
#define SIG_LEVEL		0.20	/* minimum level needed to detect Signaling Tone (relative to 8kHz deviation) */
#define SAT_DETECT_COUNT	5	/* number of measures to detect SAT signal (~250ms per spec) */
#define SAT_LOST_COUNT		5	/* number of measures to loose SAT signal (~250ms per spec) */
#define SAT_FREQ_CHANGE_COUNT	5	/* number of measures for SAT frequency change (~250ms per spec) */
#define SIG_DETECT_COUNT	6	/* number of measures to detect Signaling Tone */
#define SIG_LOST_COUNT		4	/* number of measures to loose Signaling Tone */
#define CUT_OFF_HIGHPASS	300.0   /* cut off frequency for high pass filter to remove dc level from sound card / sample */
#define BEST_QUALITY		0.68	/* Best possible RX quality */
#define COMFORT_NOISE		0.02	/* audio level of comfort noise (relative to speech level) */
/* SAT level thresholds with hysteresis (relative to nominal=1.0) */
#define SAT_LEVEL_HIGH		0.50	/* threshold to detect SAT (from no SAT state) */
#define SAT_LEVEL_LOW		0.35	/* threshold to lose SAT (hysteresis ~3 dB) */
/* Minimum ratio of dominant SAT frequency over others for valid classification */
#define SAT_FREQ_RATIO		2.0	/* dominant frequency must be 2x (6 dB) above others */

static sample_t ramp_up[256], ramp_down[256];

static double sat_freq[4] = {
	5970.0,
	6000.0,
	6030.0,
	5790.0, /* noise level to check against */
};

static sample_t dsp_sine_sat[65536];

static uint8_t dsp_sync_check[0x800];

/* global init for FSK */
void dsp_init(void)
{
	int i;
	double s;

	LOGP(DDSP, LOGL_DEBUG, "Generating sine table for SAT signal.\n");
	for (i = 0; i < 65536; i++) {
		s = sin((double)i / 65536.0 * 2.0 * PI);
		dsp_sine_sat[i] = s * ((!tacs) ? AMPS_SAT_DEVIATION : TACS_SAT_DEVIATION);
	}

	/* sync checker */
	for (i = 0; i < 0x800; i++) {
		dsp_sync_check[i] = 0xff; /* no sync */
	}
	for (i = 0; i < 11; i++) {
		dsp_sync_check[0x712 ^ (1 << i)] = 0x01; /* one bit error */
		dsp_sync_check[0x0ed ^ (1 << i)] = 0x81; /* one bit error */
	}
	dsp_sync_check[0x712] = 0x00; /* no bit error */
	dsp_sync_check[0x0ed] = 0x80; /* no bit error */

	compandor_init();
	clipper_init(0.95);
}

static void dsp_init_ramp(amps_t *amps)
{
	double c;
        int i;

	LOGP(DDSP, LOGL_DEBUG, "Generating smooth ramp table.\n");
	for (i = 0; i < 256; i++) {
		c = cos((double)i / 256.0 * PI);
#if 0
		if (c < 0)
			c = -sqrt(-c);
		else
			c = sqrt(c);
#endif
		ramp_down[i] = c * (double)amps->fsk_deviation;
		ramp_up[i] = -ramp_down[i];
	}
}

static void sat_reset(amps_t *amps, const char *reason);

/* Get human-readable name for SAT state */
const char *sat_state_name(enum sat_state state)
{
	switch (state) {
	case SAT_STATE_NONE: return "NONE";
	case SAT_STATE_5970: return "5970 Hz";
	case SAT_STATE_6000: return "6000 Hz";
	case SAT_STATE_6030: return "6030 Hz";
	default: return "UNKNOWN";
	}
}

/* Classify SAT frequency based on Goertzel filter outputs.
 * Returns the detected SAT frequency class, or SAT_FREQ_INVALID if
 * no clear dominant frequency is detected.
 */
static enum sat_freq_class sat_classify_frequency(double levels[3])
{
	int max_idx = 0;
	double max_level = levels[0];
	double second_max = 0;
	int i;

	/* Find strongest SAT frequency */
	for (i = 1; i < 3; i++) {
		if (levels[i] > max_level) {
			max_level = levels[i];
			max_idx = i;
		}
	}

	/* Find second strongest */
	for (i = 0; i < 3; i++) {
		if (i != max_idx && levels[i] > second_max)
			second_max = levels[i];
	}

	/* Require dominant frequency to be significantly above second (6 dB) */
	if (max_level < second_max * SAT_FREQ_RATIO)
		return SAT_FREQ_INVALID;

	switch (max_idx) {
	case 0: return SAT_FREQ_5970;
	case 1: return SAT_FREQ_6000;
	case 2: return SAT_FREQ_6030;
	default: return SAT_FREQ_INVALID;
	}
}

/* Convert SAT frequency class to SAT state */
static enum sat_state sat_freq_to_state(enum sat_freq_class freq_class)
{
	switch (freq_class) {
	case SAT_FREQ_5970: return SAT_STATE_5970;
	case SAT_FREQ_6000: return SAT_STATE_6000;
	case SAT_FREQ_6030: return SAT_STATE_6030;
	default: return SAT_STATE_NONE;
	}
}

/* Convert SCC (SAT Color Code) to expected SAT state */
static enum sat_state scc_to_sat_state(int scc)
{
	switch (scc) {
	case 0: return SAT_STATE_5970;
	case 1: return SAT_STATE_6000;
	case 2: return SAT_STATE_6030;
	default: return SAT_STATE_NONE;
	}
}

/* Init FSK of transceiver */
int dsp_init_sender(amps_t *amps, int tolerant)
{
	sample_t *spl;
	int i;
	int rc;
	int half;

	/* attack (3ms) and recovery time (13.5ms) according to amps specs */
	setup_compandor(&amps->cstate, 8000, 3.0, 13.5);

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Init DSP for transceiver.\n");

	/* set modulation parameters */
	sender_set_fm(&amps->sender,
		(!tacs) ? AMPS_MAX_DEVIATION : TACS_MAX_DEVIATION,
		(!tacs) ? AMPS_MAX_MODULATION : TACS_MAX_MODULATION,
		(!tacs) ? AMPS_SPEECH_DEVIATION : TACS_SPEECH_DEVIATION,
		(!tacs) ? AMPS_MAX_DISPLAY : TACS_MAX_DISPLAY);

	if (amps->sender.samplerate < 96000) {
		LOGP(DDSP, LOGL_ERROR, "Sample rate must be at least 96000 Hz to process FSK and SAT signals.\n");
		return -EINVAL;
	}

	amps->fsk_bitduration = (double)amps->sender.samplerate / (double)((!tacs) ? AMPS_BITRATE : TACS_BITRATE);
	amps->fsk_bitstep = 1.0 / amps->fsk_bitduration;
	LOGP(DDSP, LOGL_DEBUG, "Use %.4f samples for full bit duration @ %d.\n", amps->fsk_bitduration, amps->sender.samplerate);

	amps->fsk_tx_buffer_size = amps->fsk_bitduration + 10; /* 10 extra to avoid overflow due to rounding */
	spl = calloc(amps->fsk_tx_buffer_size, sizeof(*spl));
	if (!spl) {
		LOGP(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}
	amps->fsk_tx_buffer = spl;

	amps->fsk_rx_window_length = ceil(amps->fsk_bitduration); /* buffer holds one bit (rounded up) */
	half = amps->fsk_rx_window_length >> 1;
	amps->fsk_rx_window_begin = half >> 1;
	amps->fsk_rx_window_half = half;
	amps->fsk_rx_window_end = amps->fsk_rx_window_length - (half >> 1);
	LOGP(DDSP, LOGL_DEBUG, "Bit window length: %d\n", amps->fsk_rx_window_length);
	LOGP(DDSP, LOGL_DEBUG, " -> Samples in window to analyse level left of edge: %d..%d\n", amps->fsk_rx_window_begin, amps->fsk_rx_window_half - 1);
	LOGP(DDSP, LOGL_DEBUG, " -> Samples in window to analyse level right of edge: %d..%d\n", amps->fsk_rx_window_half, amps->fsk_rx_window_end - 1);
	spl = calloc(amps->fsk_rx_window_length, sizeof(*amps->fsk_rx_window));
	if (!spl) {
		LOGP(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}
	amps->fsk_rx_window = spl;

	/* create deviation and ramp */
	amps->fsk_deviation = (!tacs) ? AMPS_FSK_DEVIATION : TACS_FSK_DEVIATION;
	dsp_init_ramp(amps);

	/* allocate ring buffer for SAT signal detection
	 * the bandwidth of the Goertzel filter is the reciprocal of the duration
	 * we half our bandwidth, so that other supervisory signals will be canceled out completely by goertzel filter
	 */
	amps->sat_samples = (int)((double)amps->sender.samplerate * (1.0 / (SAT_BANDWIDTH / 2.0)) + 0.5);
	spl = calloc(amps->sat_samples, sizeof(*spl));
	if (!spl) {
		LOGP(DDSP, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}
	LOGP(DDSP, LOGL_DEBUG, "Sat detection interval is %d ms.\n", amps->sat_samples * 1000 / amps->sender.samplerate);
	amps->sat_filter_spl = spl;

	/* count SAT tones */
	for (i = 0; i < 4; i++) {
		audio_goertzel_init(&amps->sat_goertzel[i], sat_freq[i], amps->sender.samplerate);
		if (i < 3)
			amps->sat_phaseshift65536[i] = 65536.0 / ((double)amps->sender.samplerate / sat_freq[i]);
	}
	/* signaling tone */
	audio_goertzel_init(&amps->sat_goertzel[4], (!tacs) ? 10000.0 : 8000.0, amps->sender.samplerate);
	sat_reset(amps, "Initial state");

	/* Fast ST detection for handoff - 20ms window for catching 50ms ST
	 * Bandwidth = 1/0.020 = 50 Hz (wider than SAT, but sufficient for ST detection)
	 * This allows us to detect ST more quickly during handoff
	 */
	amps->fast_st_samples = (int)((double)amps->sender.samplerate * 0.020 + 0.5);  /* 20ms window */
	spl = calloc(amps->fast_st_samples, sizeof(*spl));
	if (!spl) {
		LOGP(DDSP, LOGL_ERROR, "No memory for fast ST buffer!\n");
		return -ENOMEM;
	}
	amps->fast_st_buffer = spl;
	amps->fast_st_pos = 0;
	amps->fast_st_detected = 0;
	amps->fast_st_count = 0;
	/* Initialize fast ST Goertzel filters */
	audio_goertzel_init(&amps->fast_st_goertzel[0], (!tacs) ? 10000.0 : 8000.0, amps->sender.samplerate);  /* ST */
	audio_goertzel_init(&amps->fast_st_goertzel[1], 5790.0, amps->sender.samplerate);  /* noise reference */
	LOGP(DDSP, LOGL_DEBUG, "Fast ST detection interval is %d ms.\n", amps->fast_st_samples * 1000 / amps->sender.samplerate);

	/* be more tolerant when syncing */
	amps->fsk_rx_sync_tolerant = tolerant;

	amps->dmp_frame_level = display_measurements_add(&amps->sender.dispmeas, "Frame Level", "%.1f %% (last)", DISPLAY_MEAS_LAST, DISPLAY_MEAS_LEFT, 0.0, 150.0, 100.0);
	amps->dmp_frame_quality = display_measurements_add(&amps->sender.dispmeas, "Frame Quality", "%.1f %% (last)", DISPLAY_MEAS_LAST, DISPLAY_MEAS_LEFT, 0.0, 100.0, 100.0);
	if (amps->chan_type == CHAN_TYPE_VC || amps->chan_type == CHAN_TYPE_CC_PC_VC) {
		amps->dmp_sat_level = display_measurements_add(&amps->sender.dispmeas, "SAT Level", "%.1f %%", DISPLAY_MEAS_AVG, DISPLAY_MEAS_LEFT, 0.0, 150.0, 100.0);
		amps->dmp_sat_quality = display_measurements_add(&amps->sender.dispmeas, "SAT Quality", "%.1f %%", DISPLAY_MEAS_AVG, DISPLAY_MEAS_LEFT, 0.0, 100.0, 100.0);
	}
	/* RF level display - shows signal strength from SDR (useful for diagnostics) */
	amps->dmp_rf_level = display_measurements_add(&amps->sender.dispmeas, "RF Level", "%.1f dB", DISPLAY_MEAS_AVG, DISPLAY_MEAS_LEFT, -100.0, 0.0, -INFINITY);

	/* Initialize RF level tracking */
	amps->rf_level_db = -INFINITY;
	amps->rf_level_sum = 0.0;
	amps->rf_level_count = 0;

	return 0;

error:
	dsp_cleanup_sender(amps);

	return rc;
}

/* Cleanup transceiver instance. */
void dsp_cleanup_sender(amps_t *amps)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Cleanup DSP for treansceiver.\n");

	if (amps->fsk_tx_buffer)
		free(amps->fsk_tx_buffer);
	if (amps->fsk_rx_window)
		free(amps->fsk_rx_window);
	if (amps->sat_filter_spl) {
		free(amps->sat_filter_spl);
		amps->sat_filter_spl = NULL;
	}
	if (amps->fast_st_buffer) {
		free(amps->fast_st_buffer);
		amps->fast_st_buffer = NULL;
	}
#if 0
	if (amps->frame_spl) {
		free(amps->frame_spl);
		amps->frame_spl = NULL;
	}
#endif
}

static int fsk_encode(amps_t *amps, char bit)
{
	sample_t *spl;
	double phase, bitstep, deviation;
	int count;
	char last;

	deviation = amps->fsk_deviation;
	spl = amps->fsk_tx_buffer;
	phase = amps->fsk_tx_phase;
	last = amps->fsk_tx_last_bit;
	bitstep = amps->fsk_bitstep * 256.0 * 2.0; /* half bit ramp */

//printf("%d %d\n", (bit) & 1, last & 1);
	if ((bit & 1)) {
		if ((last & 1)) {
			/* last bit was 1, this bit is 1, so we ramp down first */
			do {
				*spl++ = ramp_down[(uint8_t)phase];
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
		} else {
			/* last bit was 0, this bit is 1, so we stay down first */
			do {
				*spl++ = -deviation;
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
		}
		/* ramp up */
		do {
			*spl++ = ramp_up[(uint8_t)phase];
			phase += bitstep;
		} while (phase < 256.0);
		phase -= 256.0;
	} else {
		if ((last & 1)) {
			/* last bit was 1, this bit is 0, so we stay up first */
			do {
				*spl++ = deviation;
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
		} else {
			/* last bit was 0, this bit is 0, so we ramp up first */
			do {
				*spl++ = ramp_up[(uint8_t)phase];
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
		}
		/* ramp down */
		do {
			*spl++ = ramp_down[(uint8_t)phase];
			phase += bitstep;
		} while (phase < 256.0);
		phase -= 256.0;
	}
	last = bit;
	/* depending on the number of samples, return the number */
	count = ((uintptr_t)spl - (uintptr_t)amps->fsk_tx_buffer) / sizeof(*spl);

	amps->fsk_tx_last_bit = last;
	amps->fsk_tx_phase = phase;
	amps->fsk_tx_buffer_length = count;

	return count;
}

static int fsk_frame(amps_t *amps, sample_t *samples, int length)
{
	int count = 0, len, pos, copy, i;
	sample_t *spl;
	int rc;
	char c;

	len = amps->fsk_tx_buffer_length;
	pos = amps->fsk_tx_buffer_pos;
	spl = amps->fsk_tx_buffer;

again:
	/* there must be length, otherwise we would skip blocks */
	if (count == length)
		goto done;

	/* start of new bit, so generate buffer for one bit */
	if (pos == 0) {
		c = amps->fsk_tx_frame[amps->fsk_tx_frame_pos];
		/* start new frame, so we generate one */
		if (c == '\0') {
			if (amps->dsp_mode == DSP_MODE_AUDIO_RX_FRAME_TX)
				rc = amps_encode_frame_fvc(amps, amps->fsk_tx_frame);
			else
				rc = amps_encode_frame_focc(amps, amps->fsk_tx_frame);
			/* check if we have no bit string (change to tx audio / silence)
			 * we may not store fsk_tx_buffer_pos, because is was reset on a mode change */
			if (rc)
				return count;
			amps->fsk_tx_frame_pos = 0;
			c = amps->fsk_tx_frame[0];
		}
		if (c == 'i')
			c = (amps->channel_busy) ? '0' : '1';
		/* invert, if polarity of the cell is negative */
		if (amps->flip_polarity)
			c ^= 1;
		len = fsk_encode(amps, c);
		amps->fsk_tx_frame_pos++;
	}

	copy = len - pos;
	if (length - count < copy)
		copy = length - count;
//printf("pos=%d length=%d copy=%d\n", pos, length, copy);
	for (i = 0; i < copy; i++) {
#ifdef DEBUG_ENCODER
		puts(debug_amplitude((double)spl[pos]));
#endif
		*samples++ = spl[pos++];
	}
	count += copy;
	if (pos == len) {
		pos = 0;
		goto again;
	}

done:
	amps->fsk_tx_buffer_length = len;
	amps->fsk_tx_buffer_pos = pos;

	return count;
}

/* send comfort noise */
static void comfort_noise(sample_t *samples, int length)
{
	int i;
	int16_t r;

	for (i = 0; i < length; i++) {
		r = random();
		samples[i] = (double)r / 32768.0 * COMFORT_NOISE;
	}
}

/* Generate audio stream with SAT signal. Keep phase for next call of function. */
static void sat_encode(amps_t *amps, sample_t *samples, int length)
{
        double phaseshift, phase;
	int i;

	phaseshift = amps->sat_phaseshift65536[amps->sat];
	phase = amps->sat_phase65536;

	for (i = 0; i < length; i++) {
		*samples++ += dsp_sine_sat[(uint16_t)phase];
		phase += phaseshift;
		if (phase >= 65536)
			phase -= 65536;
	}

	amps->sat_phase65536 = phase;
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	amps_t *amps = (amps_t *) sender;
	int count, input_num;

again:
	switch (amps->dsp_mode) {
	case DSP_MODE_OFF:
		memset(power, 0, length);
		memset(samples, 0, sizeof(*samples) * length);
		break;
	case DSP_MODE_AUDIO_RX_AUDIO_TX:
		memset(power, 1, length);
		input_num = samplerate_upsample_input_num(&sender->srstate, length);

		{
			int16_t spl[input_num];
			jitter_load_samples(&sender->dejitter, (uint8_t *)spl, input_num, sizeof(*spl), jitter_conceal_s16, NULL);
			int16_to_samples_speech(samples, spl, input_num);
		}

#if 0 /* TX DEBUG: Uncomment to enable TX path level logging */
		/* TX DEBUG: Track levels at each stage */
		static int tx_dbg_count = 0;
		static double tx_dbg_input_peak = 0, tx_dbg_comp_peak = 0, tx_dbg_emph_peak = 0, tx_dbg_final_peak = 0;
		{
			int i;
			for (i = 0; i < input_num; i++) {
				double v = fabs(samples[i]);
				if (v > tx_dbg_input_peak) tx_dbg_input_peak = v;
			}
		}
#endif

		/* DTMF TX: Mix DTMF tones into audio (at 8000 Hz, before echo suppressor) */
		if (amps->trans_list && amps->trans_list->callref)
			call_dtmf_tx(amps->trans_list->callref, samples, input_num);

		/* Echo suppressor TX reference (far-end audio before DSP processing) */
		if (amps->trans_list && amps->trans_list->callref)
			call_echo_suppressor_tx(amps->trans_list->callref, samples, input_num);

		compress_audio(&amps->cstate, samples, input_num);

#if 0 /* TX DEBUG: After compressor */
		{
			int i;
			for (i = 0; i < input_num; i++) {
				double v = fabs(samples[i]);
				if (v > tx_dbg_comp_peak) tx_dbg_comp_peak = v;
			}
		}
#endif

		/* Pre-emphasis input scaling to prevent excessive peaks.
		 * 
		 * Pre-emphasis filter: 500 Hz low corner, 2000 Hz high corner, +6 dB/octave
		 * Gain at 1 kHz = 6 * log2(1000/500) = 6 * 1 = +6 dB = 2.0x
		 * Gain at 2 kHz = 6 * log2(2000/500) = 6 * 2 = +12 dB = 4.0x (max)
		 * 
		 * Scale = 1/gain_1kHz = 1/2.0 = 0.50 to maintain unity at 1 kHz reference */
		if (amps->pre_emphasis) {
			double preemph_scale = 0.50;
			int i;
			for (i = 0; i < input_num; i++)
				samples[i] *= preemph_scale;
		}

		/* pre-emphasis at 8 kHz (BEFORE upsample) - use correct shelf filter with unity gain at DC */
		if (amps->pre_emphasis)
			pre_emphasis_fast(&amps->estate_tx_fast, samples, input_num);

#if 0 /* TX DEBUG: After pre-emphasis */
		{
			int i;
			for (i = 0; i < input_num; i++) {
				double v = fabs(samples[i]);
				if (v > tx_dbg_emph_peak) tx_dbg_emph_peak = v;
			}
		}
#endif

		samplerate_upsample(&sender->srstate, samples, input_num, samples, length);

		/* Deviation limiter (per TIA/EIA-553 §2.1.3.1.3): scale down if peak exceeds limit.
		 * Applied AFTER upsample to limit actual FM deviation. Leave headroom for SAT. */
		{
			double sat_headroom = (!tacs) ? 0.69 : 0.59;  /* SAT amplitude in normalized units (1700/2900 for TACS) */
			double max_speech = sender->max_deviation / sender->speech_deviation - sat_headroom;
			double peak = 0;
			int i;
			for (i = 0; i < length; i++) {
				double v = fabs(samples[i]);
				if (v > peak) peak = v;
			}
			if (peak > max_speech) {
				double scale = max_speech / peak;
				for (i = 0; i < length; i++)
					samples[i] *= scale;
#if 0 /* TX DEBUG: Log when limiter activates */
				LOGP_CHAN(DDSP, LOGL_DEBUG, "TX LIMITER: peak=%.3f max=%.3f scale=%.3f\n", peak, max_speech, scale);
#endif
			}
#if 0 /* TX DEBUG */
			if (peak > tx_dbg_final_peak) tx_dbg_final_peak = peak;
#endif
		}

#if 0 /* TX DEBUG: Log levels once per second */
		if (++tx_dbg_count >= 50) {  /* ~1 sec at 20ms frames */
			LOGP_CHAN(DDSP, LOGL_INFO, "TX PATH: input=%.3f comp=%.3f emph=%.3f final=%.3f\n",
				tx_dbg_input_peak, tx_dbg_comp_peak, tx_dbg_emph_peak, tx_dbg_final_peak);
			tx_dbg_count = 0;
			tx_dbg_input_peak = tx_dbg_comp_peak = tx_dbg_emph_peak = tx_dbg_final_peak = 0;
		}
#endif

		/* encode SAT during call */
		sat_encode(amps, samples, length);
		break;
	case DSP_MODE_AUDIO_RX_SILENCE_TX:
		memset(power, 1, length);
		memset(samples, 0, sizeof(*samples) * length);
		/* encode SAT while waiting for alert response or answer */
		sat_encode(amps, samples, length);
		break;
	case DSP_MODE_AUDIO_RX_FRAME_TX:
	case DSP_MODE_FRAME_RX_FRAME_TX:
		/* Encode frame into audio stream. If frames have
		 * stopped, process again for rest of stream. */
		count = fsk_frame(amps, samples, length);
		memset(power, 1, count);
		// no SAT during frame transmission, according to specs
		samples += count;
		power += count;
		length -= count;
		if (length)
			goto again;
	}
}

static void fsk_rx_bit(amps_t *amps, sample_t *spl, int len, int pos, int begin, int half, int end)
{
	int i;
	double first, second;
	int bit;
	double max = 0, min = 0;

	/* decode one bit. subtract the first half from the second half.
	 * the result shows the direction of the bit change: 1 == positive.
	 */
	pos -= begin; /* possible wrap is handled below */
	second = first = 0;
	for (i = begin; i < half; i++) {
		if (--pos < 0)
			pos += len;
//printf("second %d: %d\n", pos, spl[pos]);
		second += spl[pos];
		if (i == 0 || spl[pos] > max)
			max = spl[pos];
		if (i == 0 || spl[pos] < min)
			min = spl[pos];
	}
	second /= (half - begin);
	for (i = half; i < end; i++) {
		if (--pos < 0)
			pos += len;
//printf("first %d: %d\n", pos, spl[pos]);
		first += spl[pos];
		if (spl[pos] > max)
			max = spl[pos];
		if (spl[pos] < min)
			min = spl[pos];
	}
	first /= (end - half);
//printf("first = %d second = %d\n", first, second);
	/* get bit */
	if (second > first)
		bit = 1;
	else
		bit = 0;
#ifdef DEBUG_DECODER
	if (amps->fsk_rx_sync != FSK_SYNC_POSITIVE && amps->fsk_rx_sync != FSK_SYNC_NEGATIVE)
		printf("Decoded bit as %d (dotting life = %d)\n", bit, amps->fsk_rx_dotting_life);
	else
		printf("Decoded bit as %d\n", bit);
#endif

	if (amps->fsk_rx_sync != FSK_SYNC_POSITIVE && amps->fsk_rx_sync != FSK_SYNC_NEGATIVE) {
		amps->fsk_rx_sync_register = (amps->fsk_rx_sync_register << 1) | bit;
		/* check if we received a sync */
		switch (dsp_sync_check[amps->fsk_rx_sync_register & 0x7ff]) {
		case 0x01:
			if (!amps->fsk_rx_sync_tolerant)
				break;
			/* FALLTHRU */
		case 0x00:
#ifdef DEBUG_DECODER
			printf("Sync word detected (positive)\n");
#endif
			amps->fsk_rx_sync = FSK_SYNC_POSITIVE;
prepare_frame:
			amps->fsk_rx_frame_count = 0;
			amps->fsk_rx_frame_quality = 0.0;
			amps->fsk_rx_frame_level = 0.0;
			amps->fsk_rx_sync_register = 0x555;
			amps->when_received = get_time() - (21.0 / (double)((!tacs) ? AMPS_BITRATE : TACS_BITRATE));
			return;
		case 0x81:
			if (!amps->fsk_rx_sync_tolerant)
				break;
			/* FALLTHRU */
		case 0x80:
#ifdef DEBUG_DECODER
			printf("Sync word detected (negative)\n");
#endif
			amps->fsk_rx_sync = FSK_SYNC_NEGATIVE;
			goto prepare_frame;
			return;
		}
		/* if no sync, count down the dotting life counter */
		if (--amps->fsk_rx_dotting_life == 0) {
#ifdef DEBUG_DECODER
			printf("No Sync detected after dotting\n");
#endif
			amps->fsk_rx_sync = FSK_SYNC_NONE;
			amps->channel_busy = 0;
			return;
		}
		return;
	}

	/* count level and quality */
	amps->fsk_rx_frame_level += (double)(max - min) / (double)((!tacs) ? AMPS_FSK_DEVIATION : TACS_FSK_DEVIATION) / 2.0;
	if (bit)
		amps->fsk_rx_frame_quality += (double)(second - first) / (double)((!tacs) ? AMPS_FSK_DEVIATION : TACS_FSK_DEVIATION) / 2.0 / BEST_QUALITY;
	else
		amps->fsk_rx_frame_quality += (double)(first - second) / (double)((!tacs) ? AMPS_FSK_DEVIATION : TACS_FSK_DEVIATION) / 2.0 / BEST_QUALITY;

	/* invert bit if negative sync was detected */
	if (amps->fsk_rx_sync == FSK_SYNC_NEGATIVE)
		bit = 1 - bit;

	/* read next bit. after all bits, we reset to FSK_SYNC_NONE */
	amps->fsk_rx_frame[amps->fsk_rx_frame_count++] = bit + '0';
	if (amps->fsk_rx_frame_count > FSK_MAX_BITS) {
		fprintf(stderr, "our fsk_tx_count (%d) is larger than our max bits we can handle, please fix!\n", amps->fsk_rx_frame_count);
		abort();
	}
	if (amps->fsk_rx_frame_count == amps->fsk_rx_frame_length) {
		int more;

		/* update measurements */
		display_measurements_update(amps->dmp_frame_level, amps->fsk_rx_frame_level / (double)amps->fsk_rx_frame_count * 100.0, 0.0);
		display_measurements_update(amps->dmp_frame_quality, amps->fsk_rx_frame_quality / (double)amps->fsk_rx_frame_count * 100.0, 0.0);

		/* a complete frame was received, so we process it */
		amps->fsk_rx_frame[amps->fsk_rx_frame_count] = '\0';
		more = amps_decode_frame(amps, amps->fsk_rx_frame, amps->fsk_rx_frame_count, amps->fsk_rx_frame_level / (double)amps->fsk_rx_frame_count, amps->fsk_rx_frame_quality / amps->fsk_rx_frame_level, (amps->fsk_rx_sync == FSK_SYNC_NEGATIVE));
		if (more) {
			/* switch to next word length without DCC included */
			amps->fsk_rx_frame_length = 240;
			goto prepare_frame;
		} else {
			/* switch back to first word length with DCC included */
			if (amps->fsk_rx_frame_length == 240)
				amps->fsk_rx_frame_length = 247;
			amps->fsk_rx_sync = FSK_SYNC_NONE;
			amps->channel_busy = 0;
		}
	}
}

static void fsk_rx_dotting(amps_t *amps, double _elapsed)
{
	uint8_t pos = amps->fsk_rx_dotting_pos++;
	double average, elapsed, offset;
	int i;

#ifdef DEBUG_DECODER
	printf("Level change detected\n");
#endif
	/* store into dotting list */
	amps->fsk_rx_dotting_elapsed[pos++] = _elapsed;

	/* check quality of dotting sequence.
	 * in case this is not a dotting sequence, noise or speech, the quality
	 * should be bad.
	 * count (only) 7 'elapsed' values between 8 zero-crossings.
	 * calculate the average relative to the current position.
	 */
	average = 0.0;
	elapsed = 0.0;
	for (i = 1; i < 8; i++) {
		elapsed += amps->fsk_rx_dotting_elapsed[--pos];
		offset = elapsed - (double)i;
		if (offset >= 0.5 || offset <= -0.5) {
#ifdef DEBUG_DECODER
//			printf("offset %.3f (last but %d) not within -0.5 .. 0.5 bit position, detecting no dotting.\n", offset, i - 1);
#endif
			return;
		}
		average += offset;
	}
	average /= (double)i;

	amps->fsk_rx_dotting_life = 12;

	/* if we are already found dotting, we detect better dotting.
	 * this happens, if dotting was falsely detected due to noise.
	 * then the real dotting causes a reastart of hunting for sync sequence.
	 */
	if (amps->fsk_rx_sync == FSK_SYNC_NONE || fabs(average) < amps->fsk_rx_dotting_average) {
#ifdef DEBUG_DECODER
		printf("Found (better) dotting sequence (average = %.3f)\n", average);
#endif
		amps->fsk_rx_sync = FSK_SYNC_DOTTING;
		amps->fsk_rx_dotting_average = fabs(average);
		amps->fsk_rx_bitcount = 0.5 + average;
		if (amps->si.acc_type.bis)
			amps->channel_busy = 1;
	}
}

/* decode frame */
/* Process one FSK sample (dotting, sync, bit decoding) */
static void fsk_rx_sample(amps_t *amps, sample_t sample)
{
#ifdef DEBUG_DECODER
	puts(debug_amplitude(sample / (double)FSK_DEVIATION));
#endif
	/* push sample to detection window and shift */
	amps->fsk_rx_window[amps->fsk_rx_window_pos++] = sample;
	if (amps->fsk_rx_window_pos == amps->fsk_rx_window_length)
		amps->fsk_rx_window_pos = 0;
	if (amps->fsk_rx_sync != FSK_SYNC_POSITIVE && amps->fsk_rx_sync != FSK_SYNC_NEGATIVE) {
		/* check for change in polarity */
		if (amps->fsk_rx_last_sample <= 0) {
			if (sample > 0) {
				fsk_rx_dotting(amps, amps->fsk_rx_elapsed);
				amps->fsk_rx_elapsed = 0.0;
			}
		} else {
			if (sample <= 0) {
				fsk_rx_dotting(amps, amps->fsk_rx_elapsed);
				amps->fsk_rx_elapsed = 0.0;
			}
		}
	}
	amps->fsk_rx_last_sample = sample;
	amps->fsk_rx_elapsed += amps->fsk_bitstep;
//	printf("%.4f\n", bitcount);
	if (amps->fsk_rx_sync != FSK_SYNC_NONE) {
		amps->fsk_rx_bitcount += amps->fsk_bitstep;
		if (amps->fsk_rx_bitcount >= 1.0) {
			amps->fsk_rx_bitcount -= 1.0;
			fsk_rx_bit(amps,
				amps->fsk_rx_window,
				amps->fsk_rx_window_length,
				amps->fsk_rx_window_pos,
				amps->fsk_rx_window_begin,
				amps->fsk_rx_window_half,
				amps->fsk_rx_window_end);
		}
	}
}

/* decode frame */
static void sender_receive_frame(amps_t *amps, sample_t *samples, int length)
{
	int i;

	for (i = 0; i < length; i++)
		fsk_rx_sample(amps, samples[i]);
}


/* decode SAT and signaling tone */
/* Enhanced SAT detection with frequency classification per TIA/EIA-553-A */
static void sat_decode(amps_t *amps, sample_t *samples, int length)
{
	double results[5], levels[3], sat_deviation;
	double sat_quality, sig_quality, sat_level, sig_level, noise_level;
	enum sat_freq_class freq_class;
	enum sat_state new_state, expected_state;
	double level_threshold;
	int required_count;
	int i;

	/* Run Goertzel filters for all 3 SAT frequencies + noise reference + signaling tone */
	for (i = 0; i < 5; i++) {
		audio_goertzel(&amps->sat_goertzel[i], samples, length, 0, &results[i], 1);
	}

	/* Normalize levels for all 3 SAT frequencies */
	sat_deviation = (!tacs) ? AMPS_SAT_DEVIATION : TACS_SAT_DEVIATION;
	for (i = 0; i < 3; i++) {
		levels[i] = results[i] / sat_deviation;
		amps->sat_goertzel_levels[i] = levels[i];
	}
	noise_level = results[3] / sat_deviation;
	sig_level = results[4] / ((!tacs) ? AMPS_FSK_DEVIATION : TACS_FSK_DEVIATION);

	/* Find maximum SAT level across all 3 frequencies */
	sat_level = levels[0];
	for (i = 1; i < 3; i++) {
		if (levels[i] > sat_level)
			sat_level = levels[i];
	}

	/* Calculate SAT level in dB (relative to nominal=1.0) */
	amps->sat_level_db = 20.0 * log10(sat_level + 0.001);

	/* Classify which SAT frequency is dominant */
	freq_class = sat_classify_frequency(levels);
	amps->sat_freq_detected = freq_class;

	/* Calculate quality (ratio of SAT to noise) */
	sat_quality = (sat_level > noise_level && sat_level > 0.001) ?
	              (sat_level - noise_level) / sat_level : 0.0;
	if (sat_quality < 0)
		sat_quality = 0;

	sig_quality = (results[4] > results[3] && results[4] > 0.001) ?
	              (results[4] - results[3]) / results[4] : 0.0;
	if (sig_quality < 0)
		sig_quality = 0;

	/* Debug SAT - enhanced with frequency info */
	if (++amps->sat_print == SAT_PRINT) {
		LOGP_CHAN(DDSP, LOGL_DEBUG, "SAT: state=%s level=%.1f dB (%.0f%%) quality=%.0f%% [5970:%.0f%% 6000:%.0f%% 6030:%.0f%%]\n",
		          sat_state_name(amps->sat_state),
		          amps->sat_level_db,
		          sat_level * 100.0, sat_quality * 100.0,
		          levels[0] * 100.0, levels[1] * 100.0, levels[2] * 100.0);
		amps->sat_print = 0;
	}

	/* Update measurements */
	display_measurements_update(amps->dmp_sat_level, sat_level * 100.0, 0.0);
	display_measurements_update(amps->dmp_sat_quality, sat_quality * 100.0, 0.0);

	/* Continuous SAT update to Upper Layer for Power Control */
	if (amps->sat_state != SAT_STATE_NONE) {

		/* Pass quality as before. RSSI could be passed too if we updated the API,
		 * but for now we logged it here as requested. */
		amps_rx_sat(amps, 1, sat_quality);
	}

	/* Debug signaling tone */
	if (amps->sender.loopback || loglevel == LOGL_DEBUG) {
		LOGP_CHAN(DDSP, loglevel, "Signaling Tone level %.2f%% quality %.0f%%\n", sig_level * 100.0, sig_quality * 100.0);
	}

	/* Update DTX state */
	if (sat_quality > SAT_QUALITY && sat_level > DTX_LEVEL)
		amps->dtx_state = 1;
	else
		amps->dtx_state = 0;

	/* === Enhanced SAT State Machine === */

	/* Determine target state based on level, quality, and frequency */
	if (freq_class != SAT_FREQ_INVALID && sat_quality > SAT_QUALITY) {
		/* Apply hysteresis based on current state */
		level_threshold = (amps->sat_state == SAT_STATE_NONE) ?
		                  SAT_LEVEL_HIGH : SAT_LEVEL_LOW;

		if (sat_level >= level_threshold) {
			new_state = sat_freq_to_state(freq_class);
		} else {
			new_state = SAT_STATE_NONE;
		}
	} else {
		new_state = SAT_STATE_NONE;
	}

	/* State machine persistence logic */
	if (new_state == amps->sat_pending_state) {
		amps->sat_state_count++;
	} else {
		amps->sat_pending_state = new_state;
		amps->sat_state_count = 1;
	}

	/* Determine required persistence count based on transition type */
	if (new_state == SAT_STATE_NONE && amps->sat_state != SAT_STATE_NONE) {
		/* SAT loss */
		required_count = SAT_LOST_COUNT;
	} else if (new_state != SAT_STATE_NONE && amps->sat_state == SAT_STATE_NONE) {
		/* SAT detection */
		required_count = SAT_DETECT_COUNT;
	} else if (new_state != amps->sat_state) {
		/* SAT frequency change */
		required_count = SAT_FREQ_CHANGE_COUNT;
	} else {
		/* No state change needed */
		required_count = 1;
	}

	/* Apply state transition if persistence requirement met */
	if (amps->sat_state_count >= required_count) {
		enum sat_state old_state = amps->sat_state;

		if (new_state != old_state) {
			amps->sat_state = new_state;

			/* Notify upper layer of state changes */
			if (old_state == SAT_STATE_NONE && new_state != SAT_STATE_NONE) {
				/* SAT detected */
				LOGP_CHAN(DDSP, LOGL_DEBUG, "SAT detected: freq=%s level=%.1f dB quality=%.0f%%\n",
				          sat_state_name(new_state), amps->sat_level_db, sat_quality * 100.0);
				amps->sat_detected = 1;
				amps->sat_detect_count = 0;
				amps_rx_sat(amps, 1, sat_quality);

				/* Check for SAT mismatch (mobile transponds different SCC) */
				expected_state = scc_to_sat_state(amps->sat);
				if (new_state != expected_state) {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "SAT mismatch: expected %s, detected %s - triggering handoff callback\n",
					          sat_state_name(expected_state), sat_state_name(new_state));
					amps_rx_sat_mismatch(amps, expected_state, new_state);
				}
			} else if (old_state != SAT_STATE_NONE && new_state == SAT_STATE_NONE) {
				/* SAT lost */
				LOGP_CHAN(DDSP, LOGL_DEBUG, "SAT lost\n");
				amps->sat_detected = 0;
				amps->sat_detect_count = 0;
				amps_rx_sat(amps, 0, 0.0);
			} else if (old_state != new_state) {
				/* SAT frequency changed (handoff candidate) */
				LOGP_CHAN(DDSP, LOGL_NOTICE, "SAT frequency changed: %s -> %s\n",
				          sat_state_name(old_state), sat_state_name(new_state));
				expected_state = scc_to_sat_state(amps->sat);
				amps_rx_sat_mismatch(amps, expected_state, new_state);
			}
		}
	}

	/* === Signaling Tone Detection === */
	/* For handoff, use fast detection (1 sample) since ST is only 50ms
	 * For other states, use normal detection (6 samples) to avoid false positives
	 */
	int sig_detect_threshold = SIG_DETECT_COUNT;
	transaction_t *trans = amps->trans_list;
	if (trans && trans->state == TRANS_CALL_HANDOFF_SEND) {
		/* Fast ST detection for handoff - 50ms ST needs immediate detection */
		sig_detect_threshold = 1;
	}
	
	/* Added SIG_LEVEL check to prevent spurious detection from low-level noise/spurs */
	if (sig_quality > SIG_QUALITY && sig_level > SIG_LEVEL) {
		if (amps->sig_detected == 0) {
			amps->sig_detect_count++;
			if (amps->sig_detect_count >= sig_detect_threshold) {
				amps->sig_detected = 1;
				amps->sig_detect_count = 0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Signaling Tone detected with level=%.0f%%, quality=%.0f%%.\n", sig_level * 100.0, sig_quality * 100.0);
				amps_rx_signaling_tone(amps, 1, sig_quality);
			}
		} else
			amps->sig_detect_count = 0;
	} else {
		if (amps->sig_detected == 1) {
			amps->sig_detect_count++;
			if (amps->sig_detect_count == SIG_LOST_COUNT) {
				amps->sig_detected = 0;
				amps->sig_detect_count = 0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Signaling Tone lost.\n");
				amps_rx_signaling_tone(amps, 0, 0.0);
			}
		} else
			amps->sig_detect_count = 0;
	}
}

/* Fast ST detection for handoff - runs every 20ms instead of 66ms
 * This is specifically designed to catch the 50ms ST during handoff
 */
static void fast_st_decode(amps_t *amps, sample_t *samples, int length)
{
	double results[2], sig_level, sig_quality;
	
	/* Run Goertzel filters for ST and noise reference */
	audio_goertzel(&amps->fast_st_goertzel[0], samples, length, 0, &results[0], 1);
	audio_goertzel(&amps->fast_st_goertzel[1], samples, length, 0, &results[1], 1);
	
	/* Normalize levels */
	sig_level = results[0] / ((!tacs) ? AMPS_FSK_DEVIATION : TACS_FSK_DEVIATION);
	
	/* Calculate quality (ratio of ST to noise) */
	sig_quality = (results[0] > results[1] && results[0] > 0.001) ?
	              (results[0] - results[1]) / results[0] : 0.0;
	if (sig_quality < 0)
		sig_quality = 0;
	
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Fast ST: level=%.0f%% quality=%.0f%%\n", 
	          sig_level * 100.0, sig_quality * 100.0);
	
	/* Fast ST detection - lower thresholds for quick detection */
	if (sig_quality > 0.70 && sig_level > 0.15) {
		if (amps->fast_st_detected == 0) {
			amps->fast_st_count++;
			/* Require 2 consecutive detections (40ms) to confirm */
			if (amps->fast_st_count >= 2) {
				amps->fast_st_detected = 1;
				amps->fast_st_count = 0;
				LOGP_CHAN(DDSP, LOGL_NOTICE, "Fast ST detected! level=%.0f%% quality=%.0f%%\n", 
				          sig_level * 100.0, sig_quality * 100.0);
				amps_rx_signaling_tone(amps, 1, sig_quality);
			}
		} else {
			amps->fast_st_count = 0;
		}
	} else {
		if (amps->fast_st_detected == 1) {
			amps->fast_st_count++;
			/* Require 2 consecutive losses (40ms) to confirm loss */
			if (amps->fast_st_count >= 2) {
				amps->fast_st_detected = 0;
				amps->fast_st_count = 0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Fast ST lost.\n");
				amps_rx_signaling_tone(amps, 0, 0.0);
			}
		} else {
			amps->fast_st_count = 0;
		}
	}
}

static void sender_receive_audio(amps_t *amps, sample_t *samples, int length)
{
	transaction_t *trans = amps->trans_list;
	sample_t *spl, s;
	int max, pos;
	int i;
	int use_fast_st = 0;
	
	/* Check if we should use fast ST detection (during handoff) */
	if (trans && trans->state == TRANS_CALL_HANDOFF_SEND) {
		use_fast_st = 1;
	}
	
	/* Fast ST detection during handoff - process raw samples BEFORE the delay buffer
	 * This gives us the most recent samples for quick ST detection
	 */
	if (use_fast_st && amps->fast_st_buffer) {
		sample_t *fast_spl = amps->fast_st_buffer;
		int fast_max = amps->fast_st_samples;
		int fast_pos = amps->fast_st_pos;
		
		/* Use the raw incoming samples for fast ST detection */
		for (i = 0; i < length; i++) {
			fast_spl[fast_pos++] = samples[i];
			if (fast_pos >= fast_max) {
				fast_pos = 0;
				fast_st_decode(amps, fast_spl, fast_max);
			}
		}
		amps->fast_st_pos = fast_pos;
	}

	/* SAT / signalling tone detection */
	max = amps->sat_samples;
	spl = amps->sat_filter_spl;
	pos = amps->sat_filter_pos;
	for (i = 0; i < length; i++) {
		/* unmute: use buffer, to delay audio, so we do not miss that chunk when SAT is detected */
		s = spl[pos];
		spl[pos++] = samples[i];
		samples[i] = s;
		if (pos == max) {
			pos = 0;
			sat_decode(amps, spl, max);
		}
	}
	amps->sat_filter_pos = pos;

	/* receive audio, but only if call established and SAT detected */

	if ((amps->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX || amps->dsp_mode == DSP_MODE_AUDIO_RX_FRAME_TX)
	 && trans && trans->callref) {
		int pos, count;
		int i;

		/* DEBUG: Log RAW input levels before any filtering (disabled - uncomment to enable)
		 * Logs: RAW-INPUT: avg/peak dB levels of FM demodulated signal */
#if 0
		{
			static int raw_count = 0;
			static double raw_sum = 0;
			static double raw_max = 0;
			int j;
			for (j = 0; j < length; j++) {
				double abs_val = fabs(samples[j]);
				raw_sum += abs_val;
				if (abs_val > raw_max) raw_max = abs_val;
			}
			raw_count += length;
			if (raw_count >= 8000 * (amps->sender.samplerate / 8000)) {
				double avg_db = (raw_sum > 0) ? 20.0 * log10(raw_sum / raw_count) : -100.0;
				double max_db = (raw_max > 0) ? 20.0 * log10(raw_max) : -100.0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "RAW-INPUT: avg=%.1fdB peak=%.1fdB\n", avg_db, max_db);
				raw_count = 0;
				raw_sum = 0;
				raw_max = 0;
			}
		}
#endif

		/* Parallel FSK detection (Blank-and-Burst support) */
		/* We process raw samples before filtering to catch dotting/sync */
		for (i = 0; i < length; i++) {
			/* Optimization: Only process full FSK if we are "hunting" (dotting detected) or "syncing" */
			/* Lightweight dotting check is cheap. Full decoding happens only if dotting triggers. */
			fsk_rx_sample(amps, samples[i]);
		}

		/* If we catch a digital frame during audio mode, we might want to mute audio (blanking) */
		/* For now, we let both run. If FSK decoder wins, it triggers a callback. */

		/* de-emphasis */
		/* downsample first (channel filter) */
		/* But first! Apply RX Pre-Filter to remove SAT tone (6kHz) and noise before downsampling */
		/* Apply HPF (300Hz) first to kill DC and LF noise */
		iir_process(&amps->rx_hpf, samples, length);

		/* DEBUG: Log levels AFTER HPF (disabled - uncomment to enable)
		 * Logs: AFTER-HPF: avg/peak dB levels after 300Hz highpass filter */
#if 0
		{
			static int hpf_count = 0;
			static double hpf_sum = 0;
			static double hpf_max = 0;
			int j;
			for (j = 0; j < length; j++) {
				double abs_val = fabs(samples[j]);
				hpf_sum += abs_val;
				if (abs_val > hpf_max) hpf_max = abs_val;
			}
			hpf_count += length;
			if (hpf_count >= amps->sender.samplerate) {
				double avg_db = (hpf_sum > 0) ? 20.0 * log10(hpf_sum / hpf_count) : -100.0;
				double max_db = (hpf_max > 0) ? 20.0 * log10(hpf_max) : -100.0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "AFTER-HPF: avg=%.1fdB peak=%.1fdB\n", avg_db, max_db);
				hpf_count = 0;
				hpf_sum = 0;
				hpf_max = 0;
			}
		}
#endif

		/* DEBUG: Spectrum analysis BEFORE notch filter (disabled - uncomment to enable)
		 * Logs: PRE-NOTCH-BANDS (voice/SAT band energy), PRE-NOTCH-SAT (5970/6000/6030 Hz levels)
		 * Uses Goertzel filters to measure frequency content before SAT notch removal */
#if 0
		{
			#define PRE_NOTCH_NUM_FREQS 15
			static const double pre_notch_freqs[PRE_NOTCH_NUM_FREQS] = {
				100.0, 200.0, 300.0,
				500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3400.0,
				4000.0, 5000.0,
				5970.0, 6000.0, 6030.0
			};
			static goertzel_t pre_notch_goertzel[PRE_NOTCH_NUM_FREQS];
			static int pre_notch_initialized = 0;
			static sample_t *pre_notch_buffer = NULL;
			static int pre_notch_pos = 0;
			static int pre_notch_size = 0;
			static int pre_notch_report_count = 0;
			
			if (!pre_notch_initialized) {
				int f;
				for (f = 0; f < PRE_NOTCH_NUM_FREQS; f++) {
					audio_goertzel_init(&pre_notch_goertzel[f], pre_notch_freqs[f], amps->sender.samplerate);
				}
				pre_notch_size = amps->sender.samplerate / 10;
				pre_notch_buffer = calloc(pre_notch_size, sizeof(sample_t));
				pre_notch_initialized = 1;
			}
			
			if (pre_notch_buffer) {
				int j;
				for (j = 0; j < length; j++) {
					pre_notch_buffer[pre_notch_pos++] = samples[j];
					if (pre_notch_pos >= pre_notch_size) {
						double results[PRE_NOTCH_NUM_FREQS];
						
						audio_goertzel(pre_notch_goertzel, pre_notch_buffer, pre_notch_size, 0, results, PRE_NOTCH_NUM_FREQS);
						
						double sat_band_energy = sqrt((results[12]*results[12] + results[13]*results[13] + results[14]*results[14]) / 3.0);
						double voice_band_energy = sqrt((results[3]*results[3] + results[4]*results[4] + results[5]*results[5] + 
						                                 results[6]*results[6] + results[7]*results[7] + results[8]*results[8] + 
						                                 results[9]*results[9]) / 7.0);
						
						double sat_band_db = (sat_band_energy > 0) ? 20.0 * log10(sat_band_energy) : -100.0;
						double voice_band_db = (voice_band_energy > 0) ? 20.0 * log10(voice_band_energy) : -100.0;
						
						pre_notch_report_count++;
						if (pre_notch_report_count >= 10) {
							double db_5970 = (results[12] > 0) ? 20.0 * log10(results[12]) : -100.0;
							double db_6000 = (results[13] > 0) ? 20.0 * log10(results[13]) : -100.0;
							double db_6030 = (results[14] > 0) ? 20.0 * log10(results[14]) : -100.0;
							LOGP_CHAN(DDSP, LOGL_DEBUG, "PRE-NOTCH-BANDS: Voice=%.1fdB SAT=%.1fdB\n",
								voice_band_db, sat_band_db);
							LOGP_CHAN(DDSP, LOGL_DEBUG, "PRE-NOTCH-SAT: 5970Hz=%.1fdB 6000Hz=%.1fdB 6030Hz=%.1fdB\n",
								db_5970, db_6000, db_6030);
							pre_notch_report_count = 0;
						}
						
						pre_notch_pos = 0;
					}
				}
			}
		}
#endif

		/* Apply Notch Filter (6000Hz) to kill the pilot tone */
		iir_process(&amps->rx_notch_filter, samples, length);

		/* DEBUG: Log levels AFTER NOTCH (disabled - uncomment to enable)
		 * Logs: AFTER-NOTCH: avg/peak dB levels after SAT notch filter */
#if 0
		{
			static int notch_count = 0;
			static double notch_sum = 0;
			static double notch_max = 0;
			int j;
			for (j = 0; j < length; j++) {
				double abs_val = fabs(samples[j]);
				notch_sum += abs_val;
				if (abs_val > notch_max) notch_max = abs_val;
			}
			notch_count += length;
			if (notch_count >= amps->sender.samplerate) {
				double avg_db = (notch_sum > 0) ? 20.0 * log10(notch_sum / notch_count) : -100.0;
				double max_db = (notch_max > 0) ? 20.0 * log10(notch_max) : -100.0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "AFTER-NOTCH: avg=%.1fdB peak=%.1fdB\n", avg_db, max_db);
				notch_count = 0;
				notch_sum = 0;
				notch_max = 0;
			}
		}
#endif

		/* FM Noise Gate with Makeup Gain
		 * 
		 * FM demodulation produces a noise floor (~0.09 normalized) even during silence.
		 * This gate removes signals below threshold and expands the remaining signal
		 * to restore full amplitude range.
		 *
		 * threshold = 0.10 (normalized) = ~290 Hz deviation
		 * makeup_gain = 1.0 / (1.0 - threshold) = 1.111
		 */
		{
			#define FM_NOISE_GATE_THRESHOLD 0.10
			#define FM_NOISE_GATE_MAKEUP (1.0 / (1.0 - FM_NOISE_GATE_THRESHOLD))
			int j;
			
			for (j = 0; j < length; j++) {
				double val = samples[j];
				double abs_val = fabs(val);
				
				if (abs_val < FM_NOISE_GATE_THRESHOLD) {
					/* Below threshold: gate to zero */
					samples[j] = 0.0;
				} else {
					/* Above threshold: subtract threshold and apply makeup gain */
					double sign = (val >= 0.0) ? 1.0 : -1.0;
					samples[j] = sign * (abs_val - FM_NOISE_GATE_THRESHOLD) * FM_NOISE_GATE_MAKEUP;
				}
			}
		}

		/* DEBUG: FM-GATE statistics (disabled - uncomment to enable)
		 * Logs: FM-GATE: input/output dB levels, percentage of samples gated */
#if 0
		{
			static int gate_dbg_count = 0;
			static int gate_dbg_gated = 0;
			static double gate_dbg_sum_in = 0;
			static double gate_dbg_sum_out = 0;
			static double gate_dbg_max_in = 0;
			static double gate_dbg_max_out = 0;
			int j;
			
			for (j = 0; j < length; j++) {
				/* Note: would need to track before/after gate separately */
			}
			
			gate_dbg_count += length;
			if (gate_dbg_count >= amps->sender.samplerate) {
				double gated_pct = 100.0 * gate_dbg_gated / gate_dbg_count;
				double in_avg = gate_dbg_sum_in / gate_dbg_count;
				double out_avg = gate_dbg_sum_out / gate_dbg_count;
				double in_db = (in_avg > 0) ? 20.0 * log10(in_avg) : -100.0;
				double out_db = (out_avg > 0) ? 20.0 * log10(out_avg) : -100.0;
				double in_pk_db = (gate_dbg_max_in > 0) ? 20.0 * log10(gate_dbg_max_in) : -100.0;
				double out_pk_db = (gate_dbg_max_out > 0) ? 20.0 * log10(gate_dbg_max_out) : -100.0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "FM-GATE: in=%.1fdB(pk%.1f) out=%.1fdB(pk%.1f) gated=%.1f%% thr=%.2f\n", 
					in_db, in_pk_db, out_db, out_pk_db, gated_pct, FM_NOISE_GATE_THRESHOLD);
				gate_dbg_count = 0;
				gate_dbg_gated = 0;
				gate_dbg_sum_in = 0;
				gate_dbg_sum_out = 0;
				gate_dbg_max_in = 0;
				gate_dbg_max_out = 0;
			}
		}
#endif

		/* DEBUG: Measure voice band (300-3400 Hz) broadband energy after notch (disabled - uncomment to enable)
		 * Logs: VOICEBAND-FILTERED: avg/peak dB in 300-3400Hz band using IIR bandpass */
#if 0
		{
			static iir_filter_t vb_hpf;  /* 300 Hz highpass */
			static iir_filter_t vb_lpf;  /* 3400 Hz lowpass */
			static int vb_initialized = 0;
			static int vb_count = 0;
			static double vb_sum = 0;
			static double vb_max = 0;
			static sample_t *vb_buffer = NULL;
			static int vb_buffer_size = 0;
			
			if (!vb_initialized) {
				iir_highpass_init(&vb_hpf, 300.0, amps->sender.samplerate, 2);
				iir_lowpass_init(&vb_lpf, 3400.0, amps->sender.samplerate, 2);
				vb_initialized = 1;
			}
			
			/* Allocate/reallocate buffer if needed */
			if (length > vb_buffer_size) {
				free(vb_buffer);
				vb_buffer = malloc(length * sizeof(sample_t));
				vb_buffer_size = length;
			}
			
			if (vb_buffer) {
				int j;
				/* Copy samples and apply bandpass filter */
				memcpy(vb_buffer, samples, length * sizeof(sample_t));
				iir_process(&vb_hpf, vb_buffer, length);
				iir_process(&vb_lpf, vb_buffer, length);
				
				/* Measure filtered signal */
				for (j = 0; j < length; j++) {
					double abs_val = fabs(vb_buffer[j]);
					vb_sum += abs_val;
					if (abs_val > vb_max) vb_max = abs_val;
				}
				vb_count += length;
				
				if (vb_count >= amps->sender.samplerate) {
					double avg_db = (vb_sum > 0) ? 20.0 * log10(vb_sum / vb_count) : -100.0;
					double max_db = (vb_max > 0) ? 20.0 * log10(vb_max) : -100.0;
					LOGP_CHAN(DDSP, LOGL_DEBUG, "VOICEBAND-FILTERED: avg=%.1fdB peak=%.1fdB (300-3400Hz)\n", avg_db, max_db);
					vb_count = 0;
					vb_sum = 0;
					vb_max = 0;
				}
			}
		}
#endif

		/* DEBUG: Comprehensive spectrum analysis after notch using Goertzel (disabled - uncomment to enable)
		 * Logs: SPECTRUM-BANDS (low/voice/above-voice/SAT/high freq band energies)
		 *       SPECTRUM-LOW (100/200/300 Hz), SPECTRUM-VOICE (500-3400 Hz)
		 *       SPECTRUM-HIGH (4k/5k Hz), SPECTRUM-SAT (5970/6000/6030 Hz)
		 *       SPECTRUM-ULTRAHIGH (7k-40k Hz)
		 * Uses 22 Goertzel filters with 100ms window for ~10Hz resolution */
#if 0
		{
			/* Frequency bins for analysis:
			 * - Low freq (HPF check): 100, 200, 300 Hz
			 * - Voice band: 500, 1000, 1500, 2000, 2500, 3000, 3400 Hz
			 * - Above voice: 4000, 5000 Hz
			 * - SAT band: 5970, 6000, 6030 Hz
			 * - High freq (above SAT): 7000, 8000, 10000, 15000, 20000, 30000, 40000 Hz
			 */
			#define DIAG_NUM_FREQS 22
			static const double diag_freqs[DIAG_NUM_FREQS] = {
				100.0, 200.0, 300.0,           /* Low freq - should be attenuated by HPF */
				500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3400.0,  /* Voice band */
				4000.0, 5000.0,                /* Above voice, below SAT */
				5970.0, 6000.0, 6030.0,        /* SAT band - should be attenuated by notch */
				7000.0, 8000.0, 10000.0, 15000.0, 20000.0, 30000.0, 40000.0  /* High freq above SAT */
			};
			static goertzel_t diag_goertzel[DIAG_NUM_FREQS];
			static int diag_initialized = 0;
			static sample_t *diag_buffer = NULL;
			static int diag_pos = 0;
			static int diag_size = 0;
			static int diag_report_count = 0;
			
			if (!diag_initialized) {
				int f;
				for (f = 0; f < DIAG_NUM_FREQS; f++) {
					audio_goertzel_init(&diag_goertzel[f], diag_freqs[f], amps->sender.samplerate);
				}
				/* Use 100ms window for ~10Hz frequency resolution */
				diag_size = amps->sender.samplerate / 10;
				diag_buffer = calloc(diag_size, sizeof(sample_t));
				if (diag_buffer) {
					LOGP_CHAN(DDSP, LOGL_DEBUG, "SPECTRUM-DIAG: initialized %d freq bins, %dms window @ %dHz\n",
						DIAG_NUM_FREQS, diag_size * 1000 / amps->sender.samplerate, amps->sender.samplerate);
				}
				diag_initialized = 1;
			}
			
			if (diag_buffer) {
				int j;
				for (j = 0; j < length; j++) {
					diag_buffer[diag_pos++] = samples[j];
					if (diag_pos >= diag_size) {
						double results[DIAG_NUM_FREQS];
						double results_db[DIAG_NUM_FREQS];
						int f;
						
						audio_goertzel(diag_goertzel, diag_buffer, diag_size, 0, results, DIAG_NUM_FREQS);
						
						for (f = 0; f < DIAG_NUM_FREQS; f++) {
							results_db[f] = (results[f] > 0) ? 20.0 * log10(results[f]) : -100.0;
						}
						
						/* Calculate band energies (RMS of linear values, then to dB) */
						double low_freq_energy = sqrt((results[0]*results[0] + results[1]*results[1] + results[2]*results[2]) / 3.0);
						double voice_band_energy = sqrt((results[3]*results[3] + results[4]*results[4] + results[5]*results[5] + 
						                                 results[6]*results[6] + results[7]*results[7] + results[8]*results[8] + 
						                                 results[9]*results[9]) / 7.0);
						double above_voice_energy = sqrt((results[10]*results[10] + results[11]*results[11]) / 2.0);
						double sat_band_energy = sqrt((results[12]*results[12] + results[13]*results[13] + results[14]*results[14]) / 3.0);
						double high_freq_energy = sqrt((results[15]*results[15] + results[16]*results[16] + results[17]*results[17] +
						                                results[18]*results[18] + results[19]*results[19] + results[20]*results[20] +
						                                results[21]*results[21]) / 7.0);
						
						double low_freq_db = (low_freq_energy > 0) ? 20.0 * log10(low_freq_energy) : -100.0;
						double voice_band_db = (voice_band_energy > 0) ? 20.0 * log10(voice_band_energy) : -100.0;
						double above_voice_db = (above_voice_energy > 0) ? 20.0 * log10(above_voice_energy) : -100.0;
						double sat_band_db = (sat_band_energy > 0) ? 20.0 * log10(sat_band_energy) : -100.0;
						double high_freq_db = (high_freq_energy > 0) ? 20.0 * log10(high_freq_energy) : -100.0;
						
						/* Report every 1 second (10 windows) */
						diag_report_count++;
						if (diag_report_count >= 10) {
							LOGP_CHAN(DDSP, LOGL_DEBUG, "SPECTRUM-BANDS: LowFreq=%.1fdB Voice=%.1fdB AboveVoice=%.1fdB SAT=%.1fdB HighFreq=%.1fdB\n",
								low_freq_db, voice_band_db, above_voice_db, sat_band_db, high_freq_db);
							LOGP_CHAN(DDSP, LOGL_DEBUG, "SPECTRUM-LOW: 100Hz=%.1fdB 200Hz=%.1fdB 300Hz=%.1fdB\n",
								results_db[0], results_db[1], results_db[2]);
							LOGP_CHAN(DDSP, LOGL_DEBUG, "SPECTRUM-VOICE: 500=%.1f 1k=%.1f 1.5k=%.1f 2k=%.1f 2.5k=%.1f 3k=%.1f 3.4k=%.1fdB\n",
								results_db[3], results_db[4], results_db[5], results_db[6], results_db[7], results_db[8], results_db[9]);
							LOGP_CHAN(DDSP, LOGL_DEBUG, "SPECTRUM-HIGH: 4kHz=%.1fdB 5kHz=%.1fdB\n",
								results_db[10], results_db[11]);
							LOGP_CHAN(DDSP, LOGL_DEBUG, "SPECTRUM-SAT: 5970Hz=%.1fdB 6000Hz=%.1fdB 6030Hz=%.1fdB\n",
								results_db[12], results_db[13], results_db[14]);
							LOGP_CHAN(DDSP, LOGL_DEBUG, "SPECTRUM-ULTRAHIGH: 7k=%.1f 8k=%.1f 10k=%.1f 15k=%.1f 20k=%.1f 30k=%.1f 40k=%.1fdB\n",
								results_db[15], results_db[16], results_db[17], results_db[18], results_db[19], results_db[20], results_db[21]);
							diag_report_count = 0;
						}
						
						diag_pos = 0;
					}
				}
			}
		}
#endif

		/* DEBUG: Sub-band energy measurement using time-domain (disabled - uncomment to enable)
		 * Logs: SUBBAND-AVG (total + 6 sub-band average dB), SUBBAND-PK (peak dB per band)
		 * Sub-bands: 0-500, 500-1k, 1k-2k, 2k-3k, 3k-4k, 4k-6k Hz
		 * Shows what the expander would see if we split the signal into bands */
#if 0
		{
			#define NUM_SUBBANDS 6
			/* Sub-bands: 0-500, 500-1000, 1000-2000, 2000-3000, 3000-4000, 4000-6000 Hz */
			static iir_filter_t subband_lpf[NUM_SUBBANDS];
			static iir_filter_t subband_hpf[NUM_SUBBANDS];
			static int subband_initialized = 0;
			static double subband_sum[NUM_SUBBANDS];
			static double subband_max[NUM_SUBBANDS];
			static int subband_count = 0;
			static int subband_report = 0;
			
			/* Band edges: [low, high] for each sub-band */
			static const double band_low[NUM_SUBBANDS] = {100, 500, 1000, 2000, 3000, 4000};
			static const double band_high[NUM_SUBBANDS] = {500, 1000, 2000, 3000, 4000, 6000};
			
			if (!subband_initialized) {
				int b;
				for (b = 0; b < NUM_SUBBANDS; b++) {
					iir_highpass_init(&subband_hpf[b], band_low[b], amps->sender.samplerate, 2);
					iir_lowpass_init(&subband_lpf[b], band_high[b], amps->sender.samplerate, 2);
					subband_sum[b] = 0;
					subband_max[b] = 0;
				}
				LOGP_CHAN(DDSP, LOGL_DEBUG, "SUBBAND: initialized %d bands for time-domain measurement\n", NUM_SUBBANDS);
				subband_initialized = 1;
			}
			
			/* Process each sub-band */
			int b;
			for (b = 0; b < NUM_SUBBANDS; b++) {
				/* Copy samples and apply bandpass */
				sample_t filtered[length];
				memcpy(filtered, samples, length * sizeof(sample_t));
				iir_process(&subband_hpf[b], filtered, length);
				iir_process(&subband_lpf[b], filtered, length);
				
				/* Measure like expander: sum of absolute values */
				int j;
				for (j = 0; j < length; j++) {
					double abs_val = fabs(filtered[j]);
					subband_sum[b] += abs_val;
					if (abs_val > subband_max[b])
						subband_max[b] = abs_val;
				}
			}
			subband_count += length;
			
			/* Report every second */
			if (subband_count >= amps->sender.samplerate) {
				subband_report++;
				if (subband_report >= 1) {
					double db[NUM_SUBBANDS], pk[NUM_SUBBANDS];
					double total_sum = 0;
					for (b = 0; b < NUM_SUBBANDS; b++) {
						double avg = subband_sum[b] / subband_count;
						db[b] = (avg > 0) ? 20.0 * log10(avg) : -100.0;
						pk[b] = (subband_max[b] > 0) ? 20.0 * log10(subband_max[b]) : -100.0;
						total_sum += subband_sum[b];
					}
					double total_avg = total_sum / subband_count;
					double total_db = (total_avg > 0) ? 20.0 * log10(total_avg) : -100.0;
					
					LOGP_CHAN(DDSP, LOGL_DEBUG, "SUBBAND-AVG: Total=%.1fdB | 0-500=%.1f | 500-1k=%.1f | 1k-2k=%.1f | 2k-3k=%.1f | 3k-4k=%.1f | 4k-6k=%.1fdB\n",
						total_db, db[0], db[1], db[2], db[3], db[4], db[5]);
					LOGP_CHAN(DDSP, LOGL_DEBUG, "SUBBAND-PK:  | 0-500=%.1f | 500-1k=%.1f | 1k-2k=%.1f | 2k-3k=%.1f | 3k-4k=%.1f | 4k-6k=%.1fdB\n",
						pk[0], pk[1], pk[2], pk[3], pk[4], pk[5]);
					subband_report = 0;
				}
				
				/* Reset accumulators */
				for (b = 0; b < NUM_SUBBANDS; b++) {
					subband_sum[b] = 0;
					subband_max[b] = 0;
				}
				subband_count = 0;
			}
		}
#endif

		/* No extra LPF - downsampler has built-in 3400Hz anti-aliasing filter.
		 * Notch filter above removes SAT tones (5970/6000/6030 Hz). */

		count = samplerate_downsample(&amps->sender.srstate, samples, length);

		/* DEBUG: Log levels AFTER DOWNSAMPLE (disabled - uncomment to enable)
		 * Logs: AFTER-DOWNSAMPLE: avg/peak dB levels after resampling to 8kHz */
#if 0
		{
			static int post_ds_count = 0;
			static double post_ds_sum = 0;
			static double post_ds_max = 0;
			int j;
			for (j = 0; j < count; j++) {
				double abs_val = fabs(samples[j]);
				post_ds_sum += abs_val;
				if (abs_val > post_ds_max) post_ds_max = abs_val;
			}
			post_ds_count += count;
			if (post_ds_count >= 8000) {
				double avg_db = (post_ds_sum > 0) ? 20.0 * log10(post_ds_sum / post_ds_count) : -100.0;
				double max_db = (post_ds_max > 0) ? 20.0 * log10(post_ds_max) : -100.0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "AFTER-DOWNSAMPLE: avg=%.1fdB peak=%.1fdB\n", avg_db, max_db);
				post_ds_count = 0;
				post_ds_sum = 0;
				post_ds_max = 0;
			}
		}
#endif

		/* de-emphasis (now at 8000 Hz) - use estate_rx which is initialized for 8000 Hz! */
		if (amps->de_emphasis) {
			/* DC filter MUST be applied before de-emphasis to prevent accumulation.
			 * The de-emphasis filter is y = x + factor*y_last, which accumulates DC.
			 * Without DC removal, the output grows over time (rising noise floor). */
			dc_filter(&amps->estate_rx, samples, count);
			de_emphasis(&amps->estate_rx, samples, count);
		}

		/* Voice band filter (300-3400 Hz) before expander
		 * Uses separate HPF + LPF since iir_bandpass_init() is a resonant filter
		 * for single-frequency detection, not a wide passband filter.
		 * This limits bandwidth so expander envelope tracks only voice band energy,
		 * not wideband receiver noise that would keep envelope elevated during silence */
		iir_process(&amps->rx_voice_hpf, samples, count);
		iir_process(&amps->rx_voice_lpf, samples, count);

		/* DEBUG: Log levels BEFORE expander (disabled - uncomment to enable)
		 * Logs: PRE-EXPANDER: avg/peak dB levels before 2:1 expander */
#if 0
		{
			static int pre_exp_count = 0;
			static double pre_exp_sum = 0;
			static double pre_exp_max = 0;
			int j;
			for (j = 0; j < count; j++) {
				double abs_val = fabs(samples[j]);
				pre_exp_sum += abs_val;
				if (abs_val > pre_exp_max) pre_exp_max = abs_val;
			}
			pre_exp_count += count;
			if (pre_exp_count >= 8000) {
				double avg_db = (pre_exp_sum > 0) ? 20.0 * log10(pre_exp_sum / pre_exp_count) : -100.0;
				double max_db = (pre_exp_max > 0) ? 20.0 * log10(pre_exp_max) : -100.0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "PRE-EXPANDER: avg=%.1fdB peak=%.1fdB\n", avg_db, max_db);
				pre_exp_count = 0;
				pre_exp_sum = 0;
				pre_exp_max = 0;
			}
		}
#endif

		/* removed redundant second downsample - other DSP files (cnetz, nmt, r2000) only have one */
		expand_audio(&amps->cstate, samples, count);

		/* Echo suppressor RX processing (near-end audio after DSP processing) */
		call_echo_suppressor_rx(trans->callref, samples, count);

		/* DTMF RX: Detect DTMF tones (after echo suppressor) */
		call_dtmf_rx(trans->callref, samples, count);

		spl = amps->sender.rxbuf;
		pos = amps->sender.rxbuf_pos;
		for (i = 0; i < count; i++) {
			spl[pos++] = samples[i];
			if (pos == 160) {
				if (amps->dtx_state == 0)
					comfort_noise(spl, 160);
				call_up_audio(trans->callref, spl, 160);
				pos = 0;
			}
		}
		amps->sender.rxbuf_pos = pos;
	} else
		amps->sender.rxbuf_pos = 0;
}

/* Process received audio stream from radio unit. */
void sender_receive(sender_t *sender, sample_t *samples, int length, double rf_level_db)
{
	amps_t *amps = (amps_t *) sender;

	/* Track RF level from SDR for diagnostics display
	 * Note: AMPS uses SAT for signal detection, not squelch.
	 * RF level is purely informational. */
	if (!isnan(rf_level_db)) {
		amps->rf_level_db = rf_level_db;
		amps->rf_level_sum += rf_level_db;
		amps->rf_level_count++;
		/* Update display every ~100 samples (about 10 times per second at 8kHz) */
		if (amps->rf_level_count >= 100) {
			double avg = amps->rf_level_sum / (double)amps->rf_level_count;
			if (amps->dmp_rf_level)
				display_measurements_update(amps->dmp_rf_level, avg, 0.0);
			/* Log RF level once per second (every 10th update = 1000 samples = ~1 sec at 8kHz) */
			static int log_counter = 0;
			if (++log_counter >= 10) {
				LOGP_CHAN(DDSP, LOGL_DEBUG, "RF Level: %.1f dB\n", avg);
				log_counter = 0;
			}
			amps->rf_level_sum = 0.0;
			amps->rf_level_count = 0;
		}
	}

	/* dc filter required for FSK decoding and tone detection */
	if (amps->de_emphasis)
		dc_filter(&amps->estate, samples, length);

	switch (amps->dsp_mode) {
	case DSP_MODE_OFF:
		break;
	case DSP_MODE_FRAME_RX_FRAME_TX:
		sender_receive_frame(amps, samples, length);
		break;
	case DSP_MODE_AUDIO_RX_AUDIO_TX:
	case DSP_MODE_AUDIO_RX_FRAME_TX:
	case DSP_MODE_AUDIO_RX_SILENCE_TX:
		sender_receive_audio(amps, samples, length);
		break;
	}
}

/* Reset SAT detection states, so ongoing tone will be detected again. */
static void sat_reset(amps_t *amps, const char *reason)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "SAT detector reset: %s.\n", reason);
	amps->sat_detected = 0;
	amps->sat_detect_count = 0;
	amps->sig_detected = 0;
	amps->sig_detect_count = 0;
	/* Reset enhanced SAT state machine */
	amps->sat_state = SAT_STATE_NONE;
	amps->sat_pending_state = SAT_STATE_NONE;
	amps->sat_state_count = 0;
	amps->sat_freq_detected = SAT_FREQ_INVALID;
	amps->sat_level_db = -100.0;
	/* Reset fast ST detection */
	amps->fast_st_detected = 0;
	amps->fast_st_count = 0;
	amps->fast_st_pos = 0;
}

void amps_set_dsp_mode(amps_t *amps, enum dsp_mode mode, int frame_length)
{
#if 0
	/* reset telegramm */
	if (mode == DSP_MODE_FRAME && amps->dsp_mode != mode)
		amps->frame = 0;
#endif
	if (mode == DSP_MODE_FRAME_RX_FRAME_TX) {
		/* reset SAT detection */
		sat_reset(amps, "Change to FOCC");
		LOGP_CHAN(DDSP, LOGL_INFO, "Change mode to FOCC\n");
		amps->tx_focc_debugged = 0;
	}
	if (amps->dsp_mode == DSP_MODE_FRAME_RX_FRAME_TX
	 && (mode == DSP_MODE_AUDIO_RX_AUDIO_TX || mode == DSP_MODE_AUDIO_RX_FRAME_TX || mode == DSP_MODE_AUDIO_RX_SILENCE_TX)) {
		/* reset SAT detection */
		sat_reset(amps, "Change from FOCC to FVC");
		LOGP_CHAN(DDSP, LOGL_INFO, "Change mode from FOCC to FVC\n");
	}
	if (amps->dsp_mode == DSP_MODE_OFF
	 && (mode == DSP_MODE_AUDIO_RX_AUDIO_TX || mode == DSP_MODE_AUDIO_RX_FRAME_TX || mode == DSP_MODE_AUDIO_RX_SILENCE_TX)) {
		/* reset SAT detection */
		sat_reset(amps, "Enable FVC");
		LOGP_CHAN(DDSP, LOGL_INFO, "Change mode from OFF to FVC\n");
	}
	if (mode == DSP_MODE_OFF) {
		/* reset SAT detection */
		sat_reset(amps, "Disable FVC");
		LOGP_CHAN(DDSP, LOGL_INFO, "Change mode from FVC to OFF\n");
	}
	if (mode == DSP_MODE_AUDIO_RX_AUDIO_TX && amps->dsp_mode != mode)
		jitter_reset(&amps->sender.dejitter);

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Reset FSK frame transmitter, due to setting DSP mode.\n");

	amps->dsp_mode = mode;
	if (frame_length)
		amps->fsk_rx_frame_length = frame_length;
	else if (mode == DSP_MODE_AUDIO_RX_AUDIO_TX || mode == DSP_MODE_AUDIO_RX_FRAME_TX)
		amps->fsk_rx_frame_length = 240;

	/* reset detection process */
	amps->fsk_rx_sync = FSK_SYNC_NONE;
	amps->channel_busy = 0;
	amps->fsk_rx_sync_register = 0x555;

	/* reset transmitter */
	amps->fsk_tx_buffer_pos = 0;
	amps->fsk_tx_frame[0] = '\0';
	amps->fsk_tx_frame_pos = 0;
}

/* Receive audio from call instance. */
void call_down_audio(void *decoder, void *decoder_priv, int callref, uint16_t sequence, uint8_t marker, uint32_t timestamp, uint32_t ssrc, uint8_t *payload, int payload_len)
{
	sender_t *sender;
	amps_t *amps;

	for (sender = sender_head; sender; sender = sender->next) {
		amps = (amps_t *) sender;
		if (amps->trans_list && amps->trans_list->callref == callref)
			break;
	}
	if (!sender)
		return;

	if (amps->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX) {
		jitter_frame_t *jf;
		jf = jitter_frame_alloc(decoder, decoder_priv, payload, payload_len, marker, sequence, timestamp, ssrc);
		if (jf)
			jitter_save(&amps->sender.dejitter, jf);
	}
}

void call_down_clock(void) {}

