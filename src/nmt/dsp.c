/* NMT audio processing
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "nmt.h"
#include "transaction.h"
#include "dsp.h"

#define PI			M_PI

/* Notes on TX_PEAK_FSK level:
 *
 * This deviation is -2.2db below the speech deviation.
 *
 * At 1800 Hz the deviation shall be 4.2 kHz, so with emphasis the deviation
 * at 1000 Hz would be theoretically 2.333 kHz. This is factor 0.777 below
 * 3 kHz deviation we want at speech.
 */

/* Notes on TX_PEAK_SUPER (supervisory signal) level:
 *
 * This level has 0.3 kHz deviation at 4015 Hz.
 *
 * Same calculation as above, but now we want 0.3 kHz deviation after emphasis,
 * so we calculate what we would need at 1000 Hz in relation to 3 kHz
 * deviation.
 */

/* gain of the speech toward the network, set by the --network-rx-gain option in main.c */
extern double network_rx_gain_db;

/* signaling */
#define MAX_DEVIATION		4700.0
#define MAX_MODULATION		4055.0
#define SPEECH_DEVIATION	3000.0	/* deviation of speech at 1 kHz */
#define TX_PEAK_FSK		(4200.0 / 1800.0 * 1000.0 / SPEECH_DEVIATION)
#define TX_PEAK_SUPER		(300.0 / 4015.0 * 1000.0 / SPEECH_DEVIATION)
/* transmitted speech: pre-emphasis flat above this frequency, gain before the compressor (-7 dB) */
#define TX_EMPHASIS_FLAT_HZ	3000.0
#define TX_SPEECH_GAIN		0.4467
/* noise gate of the received audio (normalized, 1.0 = reference), the threshold of AMPS */
#define RX_NOISE_GATE		0.10
/* highest envelope of the expander (1.0 = reference level): the expander restores up to the maximum
 * deviation the radio carries (4,7 kHz = +3,9 dB over the 3,0 kHz reference), see dsp_init_sender() */
#define EXPANDER_MAX_ENVELOPE	(MAX_DEVIATION / SPEECH_DEVIATION)
#define BIT_RATE		1200.0
#define BIT_ADJUST		0.1	/* how much do we adjust bit clock on frequency change */
#define F0			1800.0
#define F1			1200.0
#define MAX_DISPLAY		1.4	/* something above speech level */
#define DIALTONE_HZ		425.0	/* dial tone frequency */
#define TX_PEAK_DIALTONE	1.0	/* dial tone peak FIXME: Not found in the specs! */
#define SUPER_BANDWIDTH		30.0	/* distance between two SAT tones, also bandwidth for goertzel filter */
#define SUPER_PRINT		2	/* print supervisory signal measurement every 0.5 seconds */
#define SUPER_LOST_COUNT	4	/* number of measures to loose supervisory signal */
#define SUPER_DETECT_COUNT	6	/* number of measures to detect supervisory signal */
#define MUTE_DURATION		0.280	/* a tiny bit more than two frames */

/* four supervisory tones (NMT Doc 450-1 §4.5, p.83; NMT Doc 900-1 §4.5, p.75):
 * NMT-450: 3955, 3985, 4015, 4045 Hz (30 Hz uniform spacing)
 * NMT-900: identical set — 3955, 3985, 4015, 4045 Hz (same four tones)
 *
 * NOTE ON SPEC DISCREPANCY (now definitively resolved):
 * The 1997-08-08 NMT Doc 450-1 PDF (§4.5, p.83) lists the first tone as
 * "3055 Hz". This is a print/scan defect in that document: the 1995-01-25
 * NMT Doc 900-1 §4.5 (p.75) explicitly writes "3955 Hz" for the same first
 * tone. Cross-confirmed:
 *   1. NMT 900-1 §4.5 clearly shows 3955 Hz — same spec, different edition.
 *   2. 3955/3985/4015/4045 Hz = uniform 30 Hz spacing throughout.
 *   3. All four tones are above the speech passband (~3.5 kHz+); 3055 Hz is
 *      inside the speech band, a poor choice for a supervisory tone.
 *   4. The raw PDF text layer and OCR of the 450-1 both reproduce "3055" from
 *      the scan defect (a "9" printing with a broken bowl, reading as "0").
 * The code uses 3955 Hz. This is correct. */
static double super_freq[5] = {
	3955.0, /* ø-Signal 1 (confirmed 3955 Hz; 450-1 §4.5 PDF prints "3055" — print defect) */
	3985.0, /* ø-Signal 2 */
	4015.0, /* ø-Signal 3 */
	4045.0, /* ø-Signal 4 */
	3895.0, /* noise reference level (below signal band) */
};

/* table for fast sine generation */
static sample_t dsp_sine_super[65536];
static sample_t dsp_sine_dialtone[65536];

/* global init for dsp */
void dsp_init(void)
{
	int i;
	double s;

	LOGP(DDSP, LOGL_DEBUG, "Generating sine table for supervisory signal and dial tone.\n");
	for (i = 0; i < 65536; i++) {
		s = sin((double)i / 65536.0 * 2.0 * PI);
		/* supervisor sine */
		dsp_sine_super[i] = s * TX_PEAK_SUPER;
		/* dialtone sine */
		dsp_sine_dialtone[i] = s * TX_PEAK_DIALTONE;
	}

	compandor_init();
}

static int fsk_send_bit(void *inst);
static void fsk_receive_bit(void *inst, int bit, double quality, double level);
static int speech_limiter_init(nmt_t *nmt);

/* Gain to apply after the downsampler, so that the receive audio chain (high pass, notch and low
 * pass at the DSP rate, downsampler) has 0 dB at 1 kHz. A copy of the filters is run with a test
 * tone, so the states of the real ones are not touched. */
static double rx_chain_calibration(nmt_t *nmt)
{
	iir_filter_t hp[RX_HP_ORDER / 2], notch = nmt->rx_notch_filter, lp[RX_LP_ORDER / 2];
	samplerate_t sr;
	sample_t *spl;
	double c = 0.0, s = 0.0, amp;
	int num, out, i;

	/* 200 ms are enough for the filters to settle; at high rates stay below 100000 samples */
	num = nmt->sender.samplerate / 5;
	if (num > 100000)
		num = 100000;
	if (num < 4000)
		num = 4000;
	if (init_samplerate(&sr, 8000.0, (double)nmt->sender.samplerate, 3400.0) < 0)
		return 1.0;
	spl = calloc(num, sizeof(*spl));
	if (!spl)
		return 1.0;
	for (i = 0; i < num; i++)
		spl[i] = sin(2.0 * PI * 1000.0 * (double)i / (double)nmt->sender.samplerate);
	memcpy(hp, nmt->rx_hp_filter, sizeof(hp));
	memcpy(lp, nmt->rx_lp_filter, sizeof(lp));
	iir_butterworth_process(hp, RX_HP_ORDER, spl, num);
	if (nmt->supervisory)
		iir_process(&notch, spl, num);
	iir_butterworth_process(lp, RX_LP_ORDER, spl, num);
	out = samplerate_downsample(&sr, spl, num);
	/* amplitude of the 1 kHz component in the second half of the output */
	for (i = out / 2; i < out; i++) {
		c += spl[i] * cos(2.0 * PI * 1000.0 * (double)i / 8000.0);
		s += spl[i] * sin(2.0 * PI * 1000.0 * (double)i / 8000.0);
	}
	free(spl);
	if (out < 16)
		return 1.0;
	amp = 2.0 * sqrt(c * c + s * s) / (double)(out - out / 2);
	if (amp < 0.1 || amp > 10.0)
		return 1.0;
	return 1.0 / amp;
}

/* Scale of the supervisory tone. The tone is added to the audio at the DSP rate, before the pre-emphasis of
 * the sender (done in software, when on). TX_PEAK_SUPER assumes an ideal 6 dB/octave network (gain f/1 kHz),
 * but the pre-emphasis also has a low pass at 3400 Hz, so the tone would give less than the 0,3 kHz deviation
 * that NMT Doc 450-3 1.3.8.3 and NMT Doc 900-4 4.9 require. Measure the pre-emphasis at the frequency of the
 * tone with a copy of its state, and scale the tone to give exactly that deviation. */
static double super_tx_calibration(nmt_t *nmt)
{
	emphasis_t estate;
	sample_t *spl;
	double c = 0.0, s = 0.0, amp, freq;
	int num, i;

	if (!nmt->supervisory || !nmt->sender.pre_emphasis)
		return 1.0;
	freq = super_freq[nmt->supervisory - 1];
	num = nmt->sender.samplerate / 10;
	if (num > 100000)
		num = 100000;
	if (num < 4000)
		num = 4000;
	spl = calloc(num, sizeof(*spl));
	if (!spl)
		return 1.0;
	memcpy(&estate, &nmt->sender.estate, sizeof(estate));
	for (i = 0; i < num; i++)
		spl[i] = TX_PEAK_SUPER * sin(2.0 * PI * freq * (double)i / (double)nmt->sender.samplerate);
	pre_emphasis(&estate, spl, num);
	for (i = num / 2; i < num; i++) {
		c += spl[i] * cos(2.0 * PI * freq * (double)i / (double)nmt->sender.samplerate);
		s += spl[i] * sin(2.0 * PI * freq * (double)i / (double)nmt->sender.samplerate);
	}
	free(spl);
	amp = 2.0 * sqrt(c * c + s * s) / (double)(num - num / 2);
	if (amp <= 0.0)
		return 1.0;
	/* 0,3 kHz of the speech deviation (normalized 1.0 = SPEECH_DEVIATION) */
	return (300.0 / SPEECH_DEVIATION) / amp;
}

/* Init FSK of transceiver */
int dsp_init_sender(nmt_t *nmt, double deviation_factor)
{
	sample_t *spl;
	int i;

	/* Pre-emphasis as a first order shelf, +6 dB/octave from 300 Hz, flat above 3000 Hz, 0 dB at 1 kHz
	 * (NMT Doc 900-4 4.14: 6 dB/octave 300..3400 Hz, +1/-3 dB; the shelf is -0,5 dB at 3400 Hz). The
	 * default emphasis adds a 3400 Hz low pass of two peaking sections that raised the speech peaks: with
	 * recorded speech offline 0,374 % of the samples went over the limiter threshold, with the shelf
	 * 0,135 %. The limiter and the supervisory calibration use a copy of this state. */
	if (nmt->sender.pre_emphasis)
		emphasis_set_shelf(&nmt->sender.estate, nmt->sender.samplerate, 300.0, TX_EMPHASIS_FLAT_HZ);

	/* attack (3ms) and recovery time (13.5ms) according to NMT specs */
	setup_compandor(&nmt->cstate, 8000, 3.0, 13.5);
	/* The expander is linear (+-1 dB) from +3 dB to -55 dB re the reference level (NMT Doc 900-4
	 * 5.25.1). It restores up to what the radio carries (the maximum deviation) and no more: a higher
	 * ceiling (9.99 = +20 dB) gave gain that only fed acoustic loops between phones, on air. */
	compandor_set_expander_max(&nmt->cstate, EXPANDER_MAX_ENVELOPE);

	/* init filters */
	/* flat Butterworth cascades: iir_highpass_init()/iir_lowpass_init() with several iterations
	 * repeat one peaking section (8 iterations: +7.7 dB at 2,5 kHz, 4 iterations: +3 dB at 500 Hz) */
	iir_butterworth_highpass_init(nmt->rx_hp_filter, 300.0, nmt->sender.samplerate, RX_HP_ORDER);
	if (nmt->supervisory)
		iir_notch_init(&nmt->rx_notch_filter, super_freq[nmt->supervisory - 1], nmt->sender.samplerate, 2, 20.0);
	iir_butterworth_lowpass_init(nmt->rx_lp_filter, 3400.0, nmt->sender.samplerate, RX_LP_ORDER);

	/* The filters of the receive chain and the downsampler are cascades of biquads, which have a gain at
	 * 1 kHz (about +4.8 dB at 48 kHz and at 500 kHz). The expander is made for neutral level at
	 * 1 kHz, so measure the gain of the chain with a test tone and calibrate it away, as the emphasis
	 * does for itself. */
	nmt->rx_gain_cal = rx_chain_calibration(nmt);
	LOGP(DDSP, LOGL_DEBUG, "Receive chain has %.2f dB at 1 kHz, calibrated to 0 dB.\n", -20.0 * log10(nmt->rx_gain_cal));
	/* The level of the reference tone toward the network is a choice of the network (NMT Doc 450/900-2
	 * 7A.2), so it is set after the expander and does not change the level the expander works at. */
	nmt->network_rx_gain = pow(10.0, network_rx_gain_db / 20.0);
	nmt->super_tx_scale = super_tx_calibration(nmt);
	if (speech_limiter_init(nmt) < 0) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory for the speech limiter!\n");
		return -ENOMEM;
	}
	if (nmt->supervisory)
		LOGP(DDSP, LOGL_INFO, "Supervisory tone %.0f Hz scaled by %.2f dB for %.0f Hz deviation after the pre-emphasis.\n",
		     super_freq[nmt->supervisory - 1], 20.0 * log10(nmt->super_tx_scale), 300.0);

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Init DSP for Transceiver.\n");

	/* set modulation parameters */
	sender_set_fm(&nmt->sender, MAX_DEVIATION * deviation_factor, MAX_MODULATION * deviation_factor, SPEECH_DEVIATION * deviation_factor, MAX_DISPLAY);

	LOGP(DDSP, LOGL_DEBUG, "Using FSK level of %.3f (%.3f KHz deviation @ 1500 Hz)\n", TX_PEAK_FSK * deviation_factor, 3.5 * deviation_factor);
	LOGP(DDSP, LOGL_DEBUG, "Using Supervisory level of %.3f (%.3f KHz deviation @ 4015 Hz)\n", TX_PEAK_SUPER * deviation_factor, 0.3 * deviation_factor);

	/* init fsk */
	if (fsk_mod_init(&nmt->fsk_mod, nmt, fsk_send_bit, nmt->sender.samplerate, BIT_RATE, F0, F1, TX_PEAK_FSK, 1, 0) < 0) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "FSK init failed!\n");
		return -EINVAL;
	}
	if (fsk_demod_init(&nmt->fsk_demod, nmt, fsk_receive_bit, nmt->sender.samplerate, BIT_RATE, F0, F1, BIT_ADJUST) < 0) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "FSK init failed!\n");
		return -EINVAL;
	}

	/* allocate ring buffer for SAT signal detection
	 * the bandwidth of the Goertzel filter is the reciprocal of the duration
	 * we half our bandwidth, so that other supervisory signals will be canceled out completely by goertzel filter
	 */
	nmt->super_samples = (int)((double)nmt->sender.samplerate * (1.0 / (SUPER_BANDWIDTH / 2)) + 0.5);
	spl = calloc(1, nmt->super_samples * sizeof(*spl));
	if (!spl) {
		LOGP(DDSP, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}
	nmt->super_filter_spl = spl;

	/* count supervidory tones */
	for (i = 0; i < 5; i++) {
		audio_goertzel_init(&nmt->super_goertzel[i], super_freq[i], nmt->sender.samplerate);
		if (i < 4)
			nmt->super_phaseshift65536[i] = 65536.0 / ((double)nmt->sender.samplerate / super_freq[i]);
	}
	super_reset(nmt);

	/* dial tone */
	nmt->dial_phaseshift65536 = 65536.0 / ((double)nmt->sender.samplerate / DIALTONE_HZ);

	/* dtmf, generate tone relative to speech level */
	dtmf_encode_init(&nmt->dtmf, 8000, 1.0 / SPEECH_LEVEL);

	nmt->dmp_frame_level = display_measurements_add(&nmt->sender.dispmeas, "Frame Level", "%.1f %% (last)", DISPLAY_MEAS_LAST, DISPLAY_MEAS_LEFT, 0.0, 150.0, 100.0);
	nmt->dmp_frame_quality = display_measurements_add(&nmt->sender.dispmeas, "Frame Quality", "%.1f %% (last)", DISPLAY_MEAS_LAST, DISPLAY_MEAS_LEFT, 0.0, 100.0, 100.0);
	if (nmt->sysinfo.chan_type == CHAN_TYPE_TC || nmt->sysinfo.chan_type == CHAN_TYPE_AC_TC || nmt->sysinfo.chan_type == CHAN_TYPE_CC_TC) {
		nmt->dmp_super_level = display_measurements_add(&nmt->sender.dispmeas, "Super Level", "%.1f %%", DISPLAY_MEAS_AVG, DISPLAY_MEAS_LEFT, 0.0, 150.0, 100.0);
		nmt->dmp_super_quality = display_measurements_add(&nmt->sender.dispmeas, "Super Quality", "%.1f %%", DISPLAY_MEAS_AVG, DISPLAY_MEAS_LEFT, 0.0, 100.0, 100.0);
	}

	return 0;
}

/* Cleanup transceiver instance. */
void dsp_cleanup_sender(nmt_t *nmt)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Cleanup DSP for Transceiver.\n");

	fsk_mod_cleanup(&nmt->fsk_mod);
	fsk_demod_cleanup(&nmt->fsk_demod);
	tx_limiter_exit(&nmt->tx_limiter);

	if (nmt->super_filter_spl) {
		free(nmt->super_filter_spl);
		nmt->super_filter_spl = NULL;
	}
}

/* Check for SYNC bits, then collect data bits */
static void fsk_receive_bit(void *inst, int bit, double quality, double level)
{
	nmt_t *nmt = (nmt_t *)inst;
	uint64_t frames_elapsed;
	int i;

	/* normalize FSK level */
	level /= TX_PEAK_FSK;

	nmt->rx_bits_count++;

	if (nmt->trans && nmt->trans->dms_call)
		fsk_receive_bit_dms(nmt, bit, quality, level);

//	printf("bit=%d quality=%.4f\n", bit, quality);
	if (!nmt->rx_in_sync) {
		nmt->rx_sync = (nmt->rx_sync << 1) | bit;

		/* level and quality */
		nmt->rx_level[nmt->rx_count & 0xff] = level;
		nmt->rx_quality[nmt->rx_count & 0xff] = quality;
		nmt->rx_count++;

		/* check if pattern 1010111100010010 matches */
		if (nmt->rx_sync != 0xaf12)
			return;

		/* average level and quality */
		level = quality = 0;
		for (i = 0; i < 16; i++) {
			level += nmt->rx_level[(nmt->rx_count - 1 - i) & 0xff];
			quality += nmt->rx_quality[(nmt->rx_count - 1 - i) & 0xff];
		}
		level /= 16.0; quality /= 16.0;
//		printf("sync (level = %.2f, quality = %.2f\n", level, quality);

		/* do not accept garbage */
		if (quality < 0.65)
			return;

		/* sync time */
		nmt->rx_bits_count_last = nmt->rx_bits_count_current;
		nmt->rx_bits_count_current = nmt->rx_bits_count - 26.0;

		/* rest sync register */
		nmt->rx_sync = 0;
		nmt->rx_in_sync = 1;
		nmt->rx_count = 0;

		/* set muting of receive path */
		nmt->rx_mute = (int)((double)nmt->sender.samplerate * MUTE_DURATION);
		return;
	}

	/* read bits */
	nmt->rx_frame[nmt->rx_count] = bit + '0';
	nmt->rx_level[nmt->rx_count] = level;
	nmt->rx_quality[nmt->rx_count] = quality;
	if (++nmt->rx_count != 140)
		return;

	/* end of frame */
	nmt->rx_frame[140] = '\0';
	nmt->rx_in_sync = 0;

	/* average level and quality */
	level = quality = 0;
	for (i = 0; i < 140; i++) {
		level += nmt->rx_level[i];
		quality += nmt->rx_quality[i];
	}
	level /= 140.0; quality /= 140.0;

	/* update measurements */
	display_measurements_update(nmt->dmp_frame_level, level * 100.0, 0.0);
	display_measurements_update(nmt->dmp_frame_quality, quality * 100.0, 0.0);

	/* send telegramm */
	frames_elapsed = (nmt->rx_bits_count_current - nmt->rx_bits_count_last + 83) / 166; /* round to nearest frame */
	/* convert level so that received level at TX_PEAK_FSK results in 1.0 (100%) */
	nmt_receive_frame(nmt, nmt->rx_frame, quality, level, frames_elapsed);
}

/* compare supervisory signal against noise floor around 3895 Hz */
static void super_decode(nmt_t *nmt, sample_t *samples, int length)
{
	double result[2], level, quality;

	audio_goertzel(&nmt->super_goertzel[nmt->supervisory - 1], samples, length, 0, &result[0], 1);
	audio_goertzel(&nmt->super_goertzel[4], samples, length, 0, &result[1], 1); /* noise floor detection */

	/* normalize supervisory level */
	level = result[0] / TX_PEAK_SUPER;

	quality = (result[0] - result[1]) / result[0];
	if (quality < 0)
		quality = 0;

	if (nmt->state == STATE_ACTIVE) {
		if (++nmt->super_print == SUPER_PRINT) {
			nmt->super_print = 0;
			LOGP_CHAN(DDSP, LOGL_NOTICE, "Supervisory level %.0f%% quality %.0f%%\n", level * 100.0, quality * 100.0);
		}
		/* update measurements (if dmp_* params are NULL, we omit this) */
		display_measurements_update(nmt->dmp_super_level, level * 100.0, 0.0);
		display_measurements_update(nmt->dmp_super_quality, quality * 100.0, 0.0);
	}

	if (quality > 0.7) {
		if (nmt->super_detected == 0) {
			nmt->super_detect_count++;
			if (nmt->super_detect_count == SUPER_DETECT_COUNT) {
				nmt->super_detected = 1;
				nmt->super_detect_count = 0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Supervisory signal detected with level=%.0f%%, quality=%.0f%%.\n", result[0] / 0.63662 / TX_PEAK_SUPER * 100.0, quality * 100.0);
				nmt_rx_super(nmt, 1, quality);
			}
		} else
			nmt->super_detect_count = 0;
	} else {
		if (nmt->super_detected == 1) {
			nmt->super_detect_count++;
			if (nmt->super_detect_count == SUPER_LOST_COUNT) {
				nmt->super_detected = 0;
				nmt->super_detect_count = 0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Supervisory signal lost.\n");
				nmt_rx_super(nmt, 0, 0.0);
			}
		} else
			nmt->super_detect_count = 0;
	}
}

/* Reset supervisory detection states, so ongoing tone will be detected again. */
void super_reset(nmt_t *nmt)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Supervisory detector reset.\n");
	nmt->super_detected = 0;
	nmt->super_seen = 0;
	nmt->super_detect_count = 0;
}

/* LEVEL debug: levels of the speech in a call, once per second, at each stage of both directions. dB are
 * relative to the 1 kHz reference tone (normalized amplitude 1.0 = 3,0 kHz deviation): "pk" is the peak, "rms"
 * the RMS of a sine with the same power (0 dB = reference tone), "over" the samples beyond the given limit. */
/* At the 8 kHz stages the meter also measures the low band (below 800 Hz) and the high band (above 2 kHz)
 * of the speech (4th order Butterworth on a copy), "lo" and "hi" in the log: a colouration of the chain
 * shows as a change of hi-lo from stage to stage. */
#define METER_BAND_RATE	8000
#define METER_LO_HZ	800.0
#define METER_HI_HZ	2000.0

typedef struct level_meter {
	double peak, sumsq;
	long n, over;
	int band_init;
	iir_filter_t lo[2], hi[2];
	double lo_sumsq, hi_sumsq;
} level_meter_t;

static void meter_add(level_meter_t *m, const sample_t *s, int n, double limit)
{
	int i;

	for (i = 0; i < n; i++) {
		double a = fabs(s[i]);

		if (a > m->peak)
			m->peak = a;
		if (limit > 0.0 && a >= limit)
			m->over++;
		m->sumsq += s[i] * s[i];
	}
	m->n += n;
}

/* meter_add() plus low and high band, for 8 kHz stages */
static void meter_add_bands(level_meter_t *m, const sample_t *s, int n, double limit)
{
	sample_t lo[n], hi[n];
	int i;

	meter_add(m, s, n, limit);
	if (!m->band_init) {
		iir_butterworth_lowpass_init(m->lo, METER_LO_HZ, METER_BAND_RATE, 4);
		iir_butterworth_highpass_init(m->hi, METER_HI_HZ, METER_BAND_RATE, 4);
		m->band_init = 1;
	}
	memcpy(lo, s, sizeof(lo));
	memcpy(hi, s, sizeof(hi));
	iir_butterworth_process(m->lo, 4, lo, n);
	iir_butterworth_process(m->hi, 4, hi, n);
	for (i = 0; i < n; i++) {
		m->lo_sumsq += lo[i] * lo[i];
		m->hi_sumsq += hi[i] * hi[i];
	}
}

static double meter_pk_db(const level_meter_t *m)
{
	return 20.0 * log10(m->peak + 1e-12);
}

static double rms_db(double sumsq, long n)
{
	return n ? 20.0 * log10(sqrt(sumsq / (double)n) / M_SQRT1_2 + 1e-12) : -240.0;
}

static double meter_rms_db(const level_meter_t *m)
{
	return rms_db(m->sumsq, m->n);
}

/* one stage of the log: "name pk x rms y[ lo a hi b]" */
static int meter_print(char *line, int size, int first, const char *name, const level_meter_t *m)
{
	int pos = snprintf(line, size, "%s%s pk %+.1f rms %+.1f", first ? "" : " | ", name, meter_pk_db(m), meter_rms_db(m));

	if (m->band_init && pos < size)
		pos += snprintf(line + pos, size - pos, " lo %+.1f hi %+.1f", rms_db(m->lo_sumsq, m->n), rms_db(m->hi_sumsq, m->n));
	return pos;
}

/* stages of the receive chain (mobile->network) */
enum { RX_RADIO, RX_HIGHPASS, RX_NOTCH, RX_LOWPASS, RX_8KHZ, RX_CALIBRATED, RX_EXPANDER, RX_TO_NETWORK, LEVEL_RX_STAGES };
/* stages of the transmit chain (network->mobile) */
enum { TX_FROM_NETWORK, TX_COMPRESSOR, TX_UPSAMPLED, TX_LIMITED, TX_SUPERVISORY, LEVEL_TX_STAGES };

struct level_debug {
	nmt_t *nmt;
	level_meter_t rx[LEVEL_RX_STAGES], tx[LEVEL_TX_STAGES];
	long rx_samples, tx_samples;
	long blocks;			/* TX blocks through the speech limiter */
	long limited;			/* TX blocks scaled down by the speech limiter */
	long limited_samples;		/* samples in those blocks */
	double min_scale;		/* lowest scale of the limiter */
};
static struct level_debug level_debug[8];

static struct level_debug *level_debug_of(nmt_t *nmt)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (level_debug[i].nmt == nmt)
			return &level_debug[i];
		if (!level_debug[i].nmt) {
			level_debug[i].nmt = nmt;
			level_debug[i].min_scale = 1.0;
			return &level_debug[i];
		}
	}
	return NULL;
}

/* full scale of the 16 bit audio to and from the network, in normalized units */
#define INT16_FULL_SCALE (1.0 / (SPEECH_LEVEL * 0.7079))

static void level_debug_rx_log(nmt_t *nmt, struct level_debug *ld)
{
	static const char *name[LEVEL_RX_STAGES] = { "radio", "highpass", "notch", "lowpass", "8 kHz", "calibrated",
						     "expander", "to network" };
	char line[1024];
	int i, pos = 0;

	if (ld->rx_samples < nmt->sender.samplerate)
		return;
	for (i = 0; i < LEVEL_RX_STAGES; i++)
		pos += meter_print(line + pos, sizeof(line) - pos, !i, name[i], &ld->rx[i]);
	LOGP_CHAN(DDSP, LOGL_NOTICE, "LEVEL mobile->network dB: %s | clipped %ld (full scale %+.1f dB)\n", line,
		  ld->rx[RX_TO_NETWORK].over, 20.0 * log10(INT16_FULL_SCALE));
	memset(ld->rx, 0, sizeof(ld->rx));
	ld->rx_samples = 0;
}

static void level_debug_tx_log(nmt_t *nmt, struct level_debug *ld)
{
	static const char *name[LEVEL_TX_STAGES] = { "from network", "compressor", "upsampled", "limited",
						     "with supervisory" };
	char line[1024];
	int i, pos = 0;

	if (ld->tx_samples < nmt->sender.samplerate)
		return;
	for (i = 0; i < LEVEL_TX_STAGES; i++)
		pos += meter_print(line + pos, sizeof(line) - pos, !i, name[i], &ld->tx[i]);
	LOGP_CHAN(DDSP, LOGL_NOTICE, "LEVEL network->mobile dB: %s | clipped from network %ld | limiter: %ld of %ld blocks"
		  " (%.0f %% of the samples), down to %.1f dB\n",
		  line, ld->tx[TX_FROM_NETWORK].over, ld->limited, ld->blocks,
		  ld->tx_samples ? 100.0 * (double)ld->limited_samples / (double)ld->tx_samples : 0.0,
		  20.0 * log10(ld->min_scale));
	memset(ld->tx, 0, sizeof(ld->tx));
	ld->tx_samples = 0;
	ld->blocks = 0;
	ld->limited = 0;
	ld->limited_samples = 0;
	ld->min_scale = 1.0;
}

/* Process received audio stream from radio unit. */
void sender_receive(sender_t *sender, sample_t *samples, int length, double __attribute__((unused)) rf_level_db)
{
	nmt_t *nmt = (nmt_t *) sender;
	sample_t *spl;
	int max, pos;
	int i;
	int in_call = (nmt->dsp_mode == DSP_MODE_AUDIO || nmt->dsp_mode == DSP_MODE_DTMF) && nmt->trans && nmt->trans->callref;
	struct level_debug *ld = in_call ? level_debug_of(nmt) : NULL;

	if (ld) {
		meter_add(&ld->rx[RX_RADIO], samples, length, 0.0);
		ld->rx_samples += length;
	}

	/* write received samples to decode buffer */
	max = nmt->super_samples;
	spl = nmt->super_filter_spl;
	pos = nmt->super_filter_pos;
	for (i = 0; i < length; i++) {
		spl[pos++] = samples[i];
		if (pos == max) {
			pos = 0;
			if (nmt->supervisory)
				super_decode(nmt, spl, max);
		}
	}
	nmt->super_filter_pos = pos;

	/* fsk signal */
	fsk_demod_receive(&nmt->fsk_demod, samples, length);

	/* muting audio while receiving frame */
	for (i = 0; i < length; i++) {
		if (nmt->rx_mute && !nmt->sender.loopback) {
			samples[i] = 0;
			nmt->rx_mute--;
		}
	}

	/* filter audio */
	iir_butterworth_process(nmt->rx_hp_filter, RX_HP_ORDER, samples, length);
	if (ld)
		meter_add(&ld->rx[RX_HIGHPASS], samples, length, 0.0);
	if (nmt->supervisory)
		iir_process(&nmt->rx_notch_filter, samples, length);
	/* Noise gate of the received audio, as in AMPS (amps/dsp.c): what the phone's microphone picks up
	 * when nobody speaks (room, its own earpiece), lifted by the phone's compressor, stays below the
	 * threshold (0.10 = 300 Hz deviation, -20 dB re the reference) and is not sent to the network.
	 * Above the threshold, the threshold is subtracted and made up again. */
	if (in_call) {
		for (i = 0; i < length; i++) {
			double a = fabs(samples[i]);

			if (a < RX_NOISE_GATE)
				samples[i] = 0.0;
			else
				samples[i] = ((samples[i] >= 0.0) ? 1.0 : -1.0) * (a - RX_NOISE_GATE) / (1.0 - RX_NOISE_GATE);
		}
	}
	if (ld)
		meter_add(&ld->rx[RX_NOTCH], samples, length, 0.0);
	iir_butterworth_process(nmt->rx_lp_filter, RX_LP_ORDER, samples, length);
	if (ld)
		meter_add(&ld->rx[RX_LOWPASS], samples, length, 0.0);

	if (in_call) {
		int len, count;

		len = samplerate_downsample(&nmt->sender.srstate, samples, length);
		if (ld)
			meter_add_bands(&ld->rx[RX_8KHZ], samples, len, 0.0);
		/* the chain from the radio has a gain at 1 kHz, that is calibrated away, so the expander sees
		 * the level it is made for */
		if (nmt->rx_gain_cal != 1.0) {
			for (i = 0; i < len; i++)
				samples[i] *= nmt->rx_gain_cal;
		}
		if (ld)
			meter_add_bands(&ld->rx[RX_CALIBRATED], samples, len, 0.0);
		/* NMT Doc 900-4 5.25: the reference level (unaffected level) is the level of the reference tone
		 * (1 kHz, 3,0 kHz deviation), so the expander is neutral at 1.0, like the compressor of the
		 * transmitter (4.25). The 2,2 dB of 5.25 apply to the bypassed expander only (the FFSK signalling,
		 * 2,2 dB below the reference tone, then has the same level with and without the expander). */
		if (nmt->compandor)
			expand_audio(&nmt->cstate, samples, len);
		if (ld)
			meter_add_bands(&ld->rx[RX_EXPANDER], samples, len, 0.0);
		if (nmt->network_rx_gain != 1.0) {
			for (i = 0; i < len; i++)
				samples[i] *= nmt->network_rx_gain;
		}
		/* Echo suppressor RX processing (near-end audio after DSP processing) */
		call_echo_suppressor_rx(nmt->trans->callref, samples, len);
		if (nmt->dsp_mode == DSP_MODE_DTMF) {
			/* encode and fill with silence after finish */
			count = dtmf_encode(&nmt->dtmf, samples, len);
			if (count < len)
				memset(samples + count, 0, sizeof(*samples) * (len - count));
		}
		if (ld) {
			meter_add_bands(&ld->rx[RX_TO_NETWORK], samples, len, INT16_FULL_SCALE);
			level_debug_rx_log(nmt, ld);
		}
		spl = nmt->sender.rxbuf;
		pos = nmt->sender.rxbuf_pos;
		for (i = 0; i < len; i++) {
			spl[pos++] = samples[i];
			if (pos == 160) {
				call_up_audio(nmt->trans->callref, spl, 160);
				pos = 0;
			}
		}
		nmt->sender.rxbuf_pos = pos;
	} else
		nmt->sender.rxbuf_pos = 0;
}

static int fsk_send_bit(void *inst)
{
	nmt_t *nmt = (nmt_t *)inst;
	const char *frame;

	/* send frame bit (prio) */
	if (nmt->dsp_mode == DSP_MODE_FRAME) {
		if (!nmt->tx_frame_length || nmt->tx_frame_pos == nmt->tx_frame_length) {
			/* request frame */
			frame = nmt_get_frame(nmt);
			if (!frame) {
				nmt->tx_frame_length = 0;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Stop sending frames.\n");
				return -1;
			}
			memcpy(nmt->tx_frame, frame, 166);
			nmt->tx_frame_length = 166;
			nmt->tx_frame_pos = 0;
		}

		return nmt->tx_frame[nmt->tx_frame_pos++];
	}

	/* send dms bit */
	return dms_send_bit(nmt);
}

/* Speech limiter of the transmitter (NMT Doc 900-4 4.8.1 and 4.10: maximum permissible deviation 4,7 kHz;
 * 4.9: the supervisory signal does not pass through the limiter), see tx_limiter.c. The limit is the
 * maximum deviation less the deviation of the supervisory tone, after the TX gain, with a margin of 5 % for
 * what the smoothing of the gain does to the pre-emphasis. The speech is delayed by the lookahead of 2 ms,
 * the gain comes back by 1 dB per 10 ms. */
#define LIMITER_MARGIN		0.95
#define LIMITER_LOOKAHEAD_MS	2.0
#define LIMITER_RELEASE_DB_MS	(1.0 / 10.0)

static int speech_limiter_init(nmt_t *nmt)
{
	double limit;

	limit = MAX_DEVIATION / SPEECH_DEVIATION;
	if (nmt->supervisory)
		limit -= 300.0 / SPEECH_DEVIATION;
	if (nmt->sender.tx_gain > 0.0)
		limit /= nmt->sender.tx_gain;
	limit *= LIMITER_MARGIN;
	return tx_limiter_init(&nmt->tx_limiter, &nmt->sender.estate, nmt->sender.pre_emphasis, nmt->sender.samplerate,
			       limit, LIMITER_LOOKAHEAD_MS, LIMITER_RELEASE_DB_MS);
}

/* Generate audio stream with supervisory signal. Keep phase for next call of function. */
static void super_encode(nmt_t *nmt, sample_t *samples, int length)
{
        double phaseshift, phase;
	int i;

	phaseshift = nmt->super_phaseshift65536[nmt->supervisory - 1];
	phase = nmt->super_phase65536;

	for (i = 0; i < length; i++) {
		*samples++ += dsp_sine_super[(uint16_t)phase] * nmt->super_tx_scale;
		phase += phaseshift;
		if (phase >= 65536)
			phase -= 65536;
	}

	nmt->super_phase65536 = phase;
}

/* Generate audio stream from dial tone. Keep phase for next call of function. */
static void dial_tone(nmt_t *nmt, sample_t *samples, int length)
{
        double phaseshift, phase;
	int i;

	phaseshift = nmt->dial_phaseshift65536;
	phase = nmt->dial_phase65536;

	for (i = 0; i < length; i++) {
		*samples++ = dsp_sine_dialtone[(uint16_t)phase];
		phase += phaseshift;
		if (phase >= 65536)
			phase -= 65536;
	}

	nmt->dial_phase65536 = phase;
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	nmt_t *nmt = (nmt_t *) sender;
	int count, input_num;

	memset(power, 1, length);

again:
	switch (nmt->dsp_mode) {
	case DSP_MODE_AUDIO:
	case DSP_MODE_DTMF:
	{
		struct level_debug *ld = (nmt->trans && nmt->trans->callref) ? level_debug_of(nmt) : NULL;

		/* the sender adds the pre-emphasis and the deviation after this, and measures the final deviation */
		nmt->sender.level_debug = (ld != NULL);
		input_num = samplerate_upsample_input_num(&sender->srstate, length);
		{
			int16_t spl[input_num];
			jitter_load_samples(&sender->dejitter, (uint8_t *)spl, input_num, sizeof(*spl), jitter_conceal_s16, NULL);
			int16_to_samples_speech(samples, spl, input_num);
		}
		if (ld)
			meter_add_bands(&ld->tx[TX_FROM_NETWORK], samples, input_num, INT16_FULL_SCALE * 0.9999);
		/* Echo suppressor TX reference (far-end audio before DSP processing) */
		if (nmt->trans && nmt->trans->callref)
			call_echo_suppressor_tx(nmt->trans->callref, samples, input_num);
		/* NMT has 3,9 dB from the 3,0 kHz reference to the 4,7 kHz maximum deviation (AMPS 12,3 dB): normal
		 * speech from the network, compressed and pre-emphasized, reached the limiter 11 times per second
		 * and each limiting was audible. Speech is lowered before the compressor so it stays below: with
		 * the shelf above, on air with recorded speech, limiter interventions per second 11 -> 0 (median),
		 * peak deviation 4,3 -> 3,7 kHz. */
		{
			int k;

			for (k = 0; k < input_num; k++)
				samples[k] *= TX_SPEECH_GAIN;
		}
		if (nmt->compandor)
			compress_audio(&nmt->cstate, samples, input_num);
		if (ld)
			meter_add_bands(&ld->tx[TX_COMPRESSOR], samples, input_num, 0.0);
		samplerate_upsample(&sender->srstate, samples, input_num, samples, length);
		if (ld)
			meter_add(&ld->tx[TX_UPSAMPLED], samples, length, 0.0);
		/* send after dejitter, so audio is flushed */
		if (nmt->dms.tx_frame_valid) {
			fsk_mod_send(&nmt->fsk_mod, samples, length, 0);
			nmt->dms.tx_mute = (nmt->dms.tx_mute > length) ? nmt->dms.tx_mute - length : 0;
			nmt->tx_limiter_active = 0;
			break;
		}
		/* the speech stays closed for 200 ms from the start of a DMS
		 * frame (NMT Doc 450-3 Annex 21 para 6.6) */
		if (nmt->dms.tx_mute > 0) {
			int muted = (nmt->dms.tx_mute < length) ? nmt->dms.tx_mute : length;

			memset(samples, 0, muted * sizeof(*samples));
			nmt->dms.tx_mute -= muted;
		}
		{
			double scale;

			/* a new stretch of speech starts without the old speech in the lookahead */
			if (!nmt->tx_limiter_active) {
				tx_limiter_reset(&nmt->tx_limiter);
				nmt->tx_limiter_active = 1;
			}
			tx_limiter_process(&nmt->tx_limiter, samples, length);
			scale = tx_limiter_min_gain(&nmt->tx_limiter);
			/* a block counts as limited from -0.01 dB on */
			if (ld) {
				ld->blocks++;
				if (scale < 0.999) {
					ld->limited++;
					ld->limited_samples += length;
					if (scale < ld->min_scale)
						ld->min_scale = scale;
				}
				meter_add(&ld->tx[TX_LIMITED], samples, length, 0.0);
			}
		}
		if (nmt->supervisory)
			super_encode(nmt, samples, length);
		if (ld) {
			meter_add(&ld->tx[TX_SUPERVISORY], samples, length, 0.0);
			ld->tx_samples += length;
			level_debug_tx_log(nmt, ld);
		}
		break;
	}
	case DSP_MODE_DIALTONE:
		nmt->sender.level_debug = 0;
		nmt->tx_limiter_active = 0;
		dial_tone(nmt, samples, length);
		break;
	case DSP_MODE_SILENCE:
		nmt->sender.level_debug = 0;
		nmt->tx_limiter_active = 0;
		memset(samples, 0, length * sizeof(*samples));
		break;
	case DSP_MODE_FRAME:
		nmt->sender.level_debug = 0;
		nmt->tx_limiter_active = 0;
		/* Encode frame into audio stream. If frames have
		 * stopped, process again for rest of stream. */
		count = fsk_mod_send(&nmt->fsk_mod, samples, length, 0);
		/* Radiate the supervisory (ø) signal together with the frames while a
		 * SCIP move is in progress: the target TC must already be supervised
		 * when the MS retunes to it (NMT Doc 450-1 Scheme C2', steps 4-5), and
		 * at loop test for self-testing. */
		if (nmt->supervisory
		 && (nmt->sender.loopback || nmt->state == STATE_SCIP_IDENT || nmt->state == STATE_SCIP_IDENT_REQ))
			super_encode(nmt, samples, count);
		samples += count;
		length -= count;
		if (length)
			goto again;
		break;
	}
}

static const char *nmt_dsp_mode_name(enum dsp_mode mode)
{
        static char invalid[16];

	switch (mode) {
	case DSP_MODE_SILENCE:
		return "SILENCE";
	case DSP_MODE_DIALTONE:
		return "DIALTONE";
	case DSP_MODE_AUDIO:
		return "AUDIO";
	case DSP_MODE_FRAME:
		return "FRAME";
	case DSP_MODE_DTMF:
		return "DTMF";
	}

	sprintf(invalid, "invalid(%d)", mode);
	return invalid;
}

void nmt_set_dsp_mode(nmt_t *nmt, enum dsp_mode mode)
{
	/* reset frame */
	if (mode == DSP_MODE_FRAME && nmt->dsp_mode != mode) {
		fsk_mod_reset(&nmt->fsk_mod);
		nmt->tx_frame_length = 0;
	}

	LOGP_CHAN(DDSP, LOGL_DEBUG, "DSP mode %s -> %s\n", nmt_dsp_mode_name(nmt->dsp_mode), nmt_dsp_mode_name(mode));
	if ((mode == DSP_MODE_AUDIO || mode == DSP_MODE_DTMF) && (nmt->dsp_mode != DSP_MODE_AUDIO && nmt->dsp_mode != DSP_MODE_DTMF))
		jitter_reset(&nmt->sender.dejitter);

	nmt->dsp_mode = mode;
}

/* Receive audio from call instance. */
void call_down_audio(void *decoder, void *decoder_priv, int callref, uint16_t sequence, uint8_t marker, uint32_t timestamp, uint32_t ssrc, uint8_t *payload, int payload_len)
{
	transaction_t *trans;
	nmt_t *nmt;

	trans = get_transaction_by_callref(callref);
	if (!trans)
		return;
	nmt = trans->nmt;
	if (!nmt)
		return;

	if (nmt->dsp_mode == DSP_MODE_AUDIO || nmt->dsp_mode == DSP_MODE_DTMF) {
		jitter_frame_t *jf;
		jf = jitter_frame_alloc(decoder, decoder_priv, payload, payload_len, marker, sequence, timestamp, ssrc);
		if (jf)
			jitter_save(&nmt->sender.dejitter, jf);
	}
}

void call_down_clock(void) {}

