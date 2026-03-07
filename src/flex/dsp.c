/* FLEX signal processing
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
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

#define CHAN flex->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libfilter/iir_filter.h"
#include "../libmobile/main_mobile.h"
#include "flex.h"
#include "frame.h"
#include "dsp.h"

#define MAX_DISPLAY	1.4	/* something above speech level, no emphasis */

static void dsp_init_ramp(flex_t *flex)
{
	double c;
	int i;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Generating cosine shaped ramp table.\n");
	for (i = 0; i < 256; i++) {
		/* This is mathematically incorrect... */
		if (i < 64)
			c = 1.0;
		else if (i >= 192)
			c = -1.0;
		else
			c = cos((double)(i - 64) / 128.0 * M_PI);
		flex->fsk_ramp_down[i] = c * flex->fsk_deviation * flex->fsk_polarity;
		flex->fsk_ramp_up[i] = -flex->fsk_ramp_down[i];
	}
}

/* 4-FSK deviation levels indexed by symbol number (0-3).
 * +4800, +1600, -1600, -4800 Hz deviation levels.
 * With FM deviation = 4800 Hz, these map to +1.0, +1/3, -1/3, -1.0.
 *
 * The standard uses Gray coding for dibit-to-symbol mapping:
 *   dibit "00" -> sym 0 (-4800 Hz), dibit "01" -> sym 1 (-1600 Hz),
 *   dibit "11" -> sym 2 (+1600 Hz), dibit "10" -> sym 3 (+4800 Hz).
 * Gray encoding is applied in fsk4_block_encode(), not here;
 * this array is indexed by the gray-encoded symbol number. */
static const double fsk4_levels[4] = { -1.0, -1.0/3.0, 1.0/3.0, 1.0 };

/* Pre-compute 4-FSK ramp tables for all 16 level transitions.
 * Each ramp[from][to][phase] is a cosine-shaped transition. */
static void dsp_init_fsk4_ramps(flex_t *flex)
{
	int from, to, i;
	double lf, lt, mid, amp;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Generating 4-FSK ramp tables.\n");
	for (from = 0; from < 4; from++) {
		lf = fsk4_levels[from] * flex->fsk_deviation * flex->fsk_polarity;
		for (to = 0; to < 4; to++) {
			lt = fsk4_levels[to] * flex->fsk_deviation * flex->fsk_polarity;
			mid = (lf + lt) / 2.0;
			amp = (lf - lt) / 2.0;
			for (i = 0; i < 256; i++) {
				double c;
				if (i < 64)
					c = 1.0;
				else if (i >= 192)
					c = -1.0;
				else
					c = cos((double)(i - 64) / 128.0 * M_PI);
				flex->fsk4_ramps[from][to][i] = mid + amp * c;
			}
		}
	}
	flex->fsk4_tx_last_level = 0;
}

/* Switch speed for the next frame portion.
 * Updates fsk_bitduration and fsk_bitstep for the selected mode.
 *
 * FLEX terminology:
 *   bps    = bit rate (1600, 3200, or 6400 bits/second)
 *   baud   = symbol rate (symbols/second) — always 1600 or 3200
 *   symbol = one modulation event on the air:
 *            2FSK: 1 symbol = 1 bit  → baud = bps
 *            4FSK: 1 symbol = 2 bits (a dibit) → baud = bps / 2
 *
 * Mode table:
 *   1600bps/2FSK (A1): 1600 baud, 1 bit/symbol
 *   3200bps/2FSK (A2): 3200 baud, 1 bit/symbol
 *   3200bps/4FSK (A3): 1600 baud, 2 bits/symbol (dibit)
 *   6400bps/4FSK (A4): 3200 baud, 2 bits/symbol (dibit)
 *
 * The 'bitrate' parameter here is the BIT rate (bps), not the symbol
 * rate.  We derive the symbol rate (baud) from it. */
void dsp_set_speed(flex_t *flex, int bitrate, int modulation_type)
{
	int symbol_rate = bitrate;
	if (modulation_type == FLEX_MOD_4FSK)
		symbol_rate = bitrate / 2;
	flex->fsk_bitduration = (double)flex->sender.samplerate / (double)symbol_rate;
	flex->fsk_bitstep = 1.0 / flex->fsk_bitduration;
	flex->current_frame_speed = bitrate;
	flex->current_frame_mod_type = modulation_type;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "DSP speed: %d bps, %d baud (symbols/s), %s, %.4f samples/symbol.\n",
		  bitrate, symbol_rate,
		  (modulation_type == FLEX_MOD_4FSK) ? "4FSK" : "2FSK",
		  flex->fsk_bitduration);
}

/* Init transceiver instance. */
void dsp_set_polarity(flex_t *flex, double polarity)
{
	if (flex->fsk_polarity == polarity)
		return;
	flex->fsk_polarity = polarity;
	dsp_init_ramp(flex);
	dsp_init_fsk4_ramps(flex);
	LOGP_CHAN(DDSP, LOGL_DEBUG, "DSP polarity set to %s.\n",
		  polarity < 0 ? "neg" : "pos");
}

int dsp_init_sender(flex_t *flex, int samplerate, double deviation, double polarity, int enable_lpf)
{
	int rc;
	double max_bitduration;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Init DSP for transceiver.\n");

	/* Set FM modulation parameters for the SDR signal chain.
	 *
	 * sender_set_fm(sender, max_deviation, max_modulation, speech_deviation, max_display)
	 *
	 * max_deviation (4800 Hz):
	 *   Peak FM deviation.  ±4.8 kHz for
	 *   both 2-level and 4-level FSK (outer symbols).
	 *
	 * max_modulation (3200 Hz):
	 *   Highest baseband frequency component of the modulated signal.
	 *   The SDR layer uses this together with max_deviation to calculate
	 *   the channel bandwidth via Carson's rule:
	 *     bandwidth = 2 × (max_deviation + max_modulation)
	 *   This bandwidth determines:
	 *     - SDR channel filter / channelizer decimation
	 *     - RX FM demodulator IIR lowpass cutoff (bandwidth / 2)
	 *   The highest symbol rate across all FLEX modes is 3200 symbols/sec
	 *   (3200bps/2FSK and 6400bps/4FSK), so the baseband extends to
	 *   ~3200 Hz.  Carson's rule: 2 × (4800 + 3200) = 16 kHz, which
	 *   matches the occupied bandwidth limit (≤16 kHz).
	 *
	 * speech_deviation (4800 Hz):
	 *   Scaling factor applied by process_sender_audio() to convert
	 *   normalized baseband samples (±1.0) to frequency in Hz.
	 *   Since our FSK samples are already normalized to ±1.0 = ±4800 Hz,
	 *   speech_deviation equals max_deviation. */
	sender_set_fm(&flex->sender, deviation, 3200, deviation, MAX_DISPLAY);

	/* Size buffer for slowest speed (1600 baud = most samples per word) */
	max_bitduration = (double)samplerate / 1600.0;
	flex->fsk_tx_buffer_size = (int)(max_bitduration * 32.0) + 10;
	flex->fsk_tx_buffer = calloc(flex->fsk_tx_buffer_size, sizeof(sample_t));
	if (!flex->fsk_tx_buffer) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* create deviation and ramp */
	flex->fsk_deviation = 1.0; /* equals what we set at sender_set_fm() */
	flex->fsk_polarity = polarity;
	flex->lpf_enabled = enable_lpf;
	dsp_init_ramp(flex);
	dsp_init_fsk4_ramps(flex);

	/* Initialize baseband LPF: 3.2 kHz cutoff, 2 iterations (4th-order).
	 * Occupied bandwidth must not exceed 16 kHz.
	 * The IIR biquad is efficient and already used throughout the codebase. */
	if (flex->lpf_enabled) {
		iir_lowpass_init(&flex->lpf, 3200.0, samplerate, 2);
		LOGP_CHAN(DDSP, LOGL_DEBUG, "Baseband LPF enabled: 3.2 kHz cutoff, 2 iterations.\n");
	}

	/* Default to 1600 baud */
	dsp_set_speed(flex, 1600, FLEX_MOD_2FSK);

	return 0;

error:
	dsp_cleanup_sender(flex);

	return rc;
}

/* Cleanup transceiver instance. */
void dsp_cleanup_sender(flex_t *flex)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Cleanup DSP for transceiver.\n");

	if (flex->fsk_tx_buffer) {
		free(flex->fsk_tx_buffer);
		flex->fsk_tx_buffer = NULL;
	}
}

/* Encode one 2-FSK word into audio samples.
 * Reads bits MSB-first from 'word', generates cosine-ramped transitions.
 * Returns number of samples written to fsk_tx_buffer. */
static int fsk_block_encode(flex_t *flex, uint32_t word, int nbits)
{
	sample_t *spl;
	double phase, bitstep, devpol;
	int i, count;
	uint8_t lastbit;

	devpol = flex->fsk_deviation * flex->fsk_polarity;
	spl = flex->fsk_tx_buffer;
	phase = flex->fsk_tx_phase;
	lastbit = flex->fsk_tx_lastbit;
	bitstep = flex->fsk_bitstep * 256.0;

	for (i = 0; i < nbits; i++) {
		if (lastbit) {
			if ((word & 0x80000000)) {
				/* stay up */
				do {
					*spl++ = devpol;
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
			} else {
				/* ramp down */
				do {
					*spl++ = flex->fsk_ramp_down[(uint8_t)phase];
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
				lastbit = 0;
			}
		} else {
			if ((word & 0x80000000)) {
				/* ramp up */
				do {
					*spl++ = flex->fsk_ramp_up[(uint8_t)phase];
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
				lastbit = 1;
			} else {
				/* stay down */
				do {
					*spl++ = -devpol;
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
			}
		}
		word <<= 1;
	}

	/* depending on the number of samples, return the number */
	count = ((uintptr_t)spl - (uintptr_t)flex->fsk_tx_buffer) / sizeof(*spl);

	flex->fsk_tx_phase = phase;
	flex->fsk_tx_lastbit = lastbit;

	return count;
}

/* 4-FSK encoder for 4-level modes (3200bps/4FSK and 6400bps/4FSK).
 * Encodes 2 bits (one dibit) per symbol.
 *   3200bps/4FSK (A3): 1600 baud (symbols/s)
 *   6400bps/4FSK (A4): 3200 baud (symbols/s)
 * Uses pre-computed cosine ramp tables for smooth transitions. */
static int fsk4_block_encode(flex_t *flex, uint32_t word, int nsymbols)
{
	sample_t *spl;
	double phase, bitstep;
	int i, count;
	uint8_t last_level, sym;

	/* Gray code lookup: dibit → symbol index.
	 * 4-level FM uses Gray coding:
	 *   dibit "00" → -4800 Hz (sym 0)
	 *   dibit "01" → -1600 Hz (sym 1)
	 *   dibit "11" → +1600 Hz (sym 2)
	 *   dibit "10" → +4800 Hz (sym 3)
	 * Standard binary-to-gray: gray = val ^ (val >> 1)
	 *   0b00 → 0b00 (0), 0b01 → 0b01 (1),
	 *   0b10 → 0b11 (3), 0b11 → 0b10 (2) */
	static const uint8_t gray_encode[4] = { 0, 1, 3, 2 };

	spl = flex->fsk_tx_buffer;
	phase = flex->fsk_tx_phase;
	last_level = flex->fsk4_tx_last_level;
	/* Symbol rate is set by dsp_set_speed(): 1600 sym/s for 3200/4FSK (A3),
	 * 3200 sym/s for 6400/4FSK (A4). */
	bitstep = flex->fsk_bitstep * 256.0;

	for (i = 0; i < nsymbols; i++) {
		uint8_t dibit = (word >> 30) & 0x03;
		sym = gray_encode[dibit];
		word <<= 2;

		if (sym == last_level) {
			/* Stay at current level */
			double lev = fsk4_levels[sym] * flex->fsk_deviation * flex->fsk_polarity;
			do {
				*spl++ = lev;
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
		} else {
			/* Ramp from last_level to sym */
			do {
				*spl++ = flex->fsk4_ramps[last_level][sym][(uint8_t)phase];
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
			last_level = sym;
		}
	}

	count = ((uintptr_t)spl - (uintptr_t)flex->fsk_tx_buffer) / sizeof(*spl);
	flex->fsk_tx_phase = phase;
	flex->fsk4_tx_last_level = last_level;

	return count;
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	flex_t *flex = (flex_t *) sender;

again:
	/* get word */
	if (!flex->fsk_tx_buffer_length) {
		uint32_t word;
		int read_bytes;
		uint8_t *src_buf;
		int *src_pos, src_len;

		/* If both buffers exhausted, ask state machine for next frame */
		if (flex->sync_buffer_pos >= flex->sync_buffer_length &&
		    flex->frame_buffer_pos >= flex->frame_buffer_length) {
			if (!flex_get_next_frame(flex)) {
				/* No data — output silence */
				memset(samples, 0, sizeof(*samples) * length);
				memset(power, 0, length);
				return;
			}

			/* Reset per-frame debug counters */
			flex->dbg_frame_symbols = 0;
			flex->dbg_frame_samples = 0;

			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "TX frame start: sync=%d bytes (1600/2fsk), data=%d bytes (target=%d/%s)\n",
				  flex->sync_buffer_length,
				  flex->frame_buffer_length,
				  flex->frame_target_speed,
				  (flex->frame_target_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk");

			/* Start at 1600/2FSK for sync portion (if present) */
			if (flex->sync_buffer_length > 0)
				dsp_set_speed(flex, 1600, FLEX_MOD_2FSK);

			/* Start with phase accumulator at zero so the first
			 * symbol gets a full-length sample period. */
			flex->fsk_tx_phase = 0.0;
			flex->fsk_tx_lastbit = 0;
		}

		/* Select which buffer to consume.
		 * Sync buffer (S1+FIW) is consumed first at 1600/2FSK.
		 * When sync buffer is exhausted, switch to target speed
		 * and consume frame buffer (S2+DATA). */
		if (flex->sync_buffer_pos < flex->sync_buffer_length) {
			/* Still consuming sync portion at 1600/2FSK */
			src_buf = flex->sync_buffer;
			src_pos = &flex->sync_buffer_pos;
			src_len = flex->sync_buffer_length;
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "TX select: SYNC buf, pos=%d/%d\n",
				  flex->sync_buffer_pos, flex->sync_buffer_length);
		} else {
			/* Sync exhausted — switch to target speed if needed */
			if (flex->frame_target_speed > 0 &&
			    flex->current_frame_speed != flex->frame_target_speed) {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "TX speed switch: %d/%s -> %d/%s\n",
					  flex->current_frame_speed,
					  (flex->current_frame_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
					  flex->frame_target_speed,
					  (flex->frame_target_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk");
				dsp_set_speed(flex, flex->frame_target_speed,
					      flex->frame_target_mod_type);
				/* Reset phase accumulator at speed switch so the
				 * first symbol at the new symbol rate is full-length.
				 * Without this, the residual phase from the last
				 * 1600-baud symbol shortens the first higher-baud
				 * symbol, causing the demodulator PLL to miscount
				 * S2 symbols by one. */
				flex->fsk_tx_phase = 0.0;
				flex->fsk4_tx_last_level = 0;
			}
			src_buf = flex->frame_buffer;
			src_pos = &flex->frame_buffer_pos;
			src_len = flex->frame_buffer_length;
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "TX select: DATA buf, pos=%d/%d, speed=%d/%s\n",
				  flex->frame_buffer_pos, flex->frame_buffer_length,
				  flex->current_frame_speed,
				  (flex->current_frame_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk");
		}

		/* Read next word from the active buffer */
		{
			int remaining = src_len - *src_pos;
			read_bytes = (remaining >= 4) ? 4 : remaining;
		}

		if (read_bytes <= 0) {
			/* Shouldn't happen, but guard against it */
			goto again;
		}

		/* Read word (big-endian, MSB-justified) */
		word = 0;
		if (read_bytes >= 1)
			word |= (uint32_t)src_buf[*src_pos] << 24;
		if (read_bytes >= 2)
			word |= (uint32_t)src_buf[*src_pos + 1] << 16;
		if (read_bytes >= 3)
			word |= (uint32_t)src_buf[*src_pos + 2] << 8;
		if (read_bytes >= 4)
			word |= (uint32_t)src_buf[*src_pos + 3];
		*src_pos += read_bytes;

		LOGP_CHAN(DDSP, LOGL_DEBUG,
			  "TX word pos=%d bytes=%d word=0x%08x speed=%d/%s buf=%s\n",
			  *src_pos - read_bytes, read_bytes, word,
			  flex->current_frame_speed,
			  (flex->current_frame_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
			  (src_buf == flex->sync_buffer) ? "sync" : "data");

		/* Encode — dispatch based on modulation type */
		if (flex->current_frame_mod_type == FLEX_MOD_4FSK)
			flex->fsk_tx_buffer_length = fsk4_block_encode(flex, word, read_bytes * 8 / 2);
		else
			flex->fsk_tx_buffer_length = fsk_block_encode(flex, word, read_bytes * 8);

		flex->fsk_tx_buffer_pos = 0;
		flex->dbg_frame_symbols += (flex->current_frame_mod_type == FLEX_MOD_4FSK)
					   ? (read_bytes * 8 / 2) : (read_bytes * 8);
		flex->dbg_frame_samples += flex->fsk_tx_buffer_length;

		/* Log at frame end */
		if (flex->sync_buffer_pos >= flex->sync_buffer_length &&
		    flex->frame_buffer_pos >= flex->frame_buffer_length) {
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "TX frame end: total %d symbols, %d samples\n",
				  flex->dbg_frame_symbols, flex->dbg_frame_samples);

			/* If no more frames follow, the TX goes silent.
			 * The RX PLL drifts slightly forward over the frame,
			 * so the last symbol's vote window extends into silence,
			 * causing a BCH error.  Append one extra symbol at the
			 * last level as a guard.  In real operation with
			 * back-to-back frames, the next S1 provides this
			 * signal naturally. */
			if (!flex->msg_list &&
			    flex->scan_from >= flex->scan_to &&
			    !flex->sender.loopback) {
				int guard = (int)(flex->fsk_bitduration + 1.0);
				sample_t last_val = flex->fsk_tx_buffer[flex->fsk_tx_buffer_length - 1];
				int gi;
				for (gi = 0; gi < guard && flex->fsk_tx_buffer_length + gi < flex->fsk_tx_buffer_size; gi++)
					flex->fsk_tx_buffer[flex->fsk_tx_buffer_length + gi] = last_val;
				flex->fsk_tx_buffer_length += gi;
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "TX guard: appended %d samples (last_val=%.3f) — no more frames\n",
					  gi, last_val);
			}
		}

		/* Apply baseband LPF after modulation */
		if (flex->lpf_enabled && flex->fsk_tx_buffer_length > 0)
			iir_process(&flex->lpf, flex->fsk_tx_buffer, flex->fsk_tx_buffer_length);
	}

	/* send encoded word until end of source or destination buffer is reached */
	while (length) {
		*power++ = 1;
		*samples++ = flex->fsk_tx_buffer[flex->fsk_tx_buffer_pos++];
		length--;
		if (flex->fsk_tx_buffer_pos == flex->fsk_tx_buffer_length) {
			flex->fsk_tx_buffer_length = 0;
			break;
		}
	}

	/* do again, if destination buffer is not yet full */
	if (length)
		goto again;
}

/* ===== FLEX Receiver ===== */

/* RX state machine states */
enum {
	RX_STATE_SYNC1 = 0,	/* S1: sync detection + inv.A completion (1600 baud, 2FSK) */
	RX_STATE_FIW,		/* FIW: 32-bit BCH codeword (1600/2FSK) */
	RX_STATE_SYNC2,		/* S2: block sync at data symbol rate (25 ms, all modes) */
	RX_STATE_DATA,		/* reading interleaved phase data at data symbol rate */
};

/* PLL tuning constants (per multimon-ng, empirically validated) */
#define SLICE_THRESHOLD		0.667	/* 4-level quantization at 2/3 envelope */
#define DC_OFFSET_FILTER	0.010	/* DC removal IIR time constant (seconds) */
#define PHASE_LOCKED_RATE	0.045	/* PLL correction when locked */
#define PHASE_UNLOCKED_RATE	0.050	/* PLL correction when unlocked */
#define LOCK_LEN		24	/* symbols to check for lock pattern */
#define DEMOD_TIMEOUT		100	/* max periods with no zero crossings */

/* ===== BCH(31,21,2) Decoder — GF(2^5) Syndrome-Based =====
 *
 * BCH(31,21) + even parity = 32-bit codeword.
 * The code operates over GF(2^5) with primitive polynomial x^5+x^2+1.
 * Can correct up to t=2 bit errors using syndrome lookup tables.
 *
 * RX codeword layout (after LSB-first accumulation):
 *   Bits 0-20:  21 data bits
 *   Bits 21-30: 10 BCH parity bits
 *   Bit 31:     even parity bit
 *
 * The decoder works on the 31-bit codeword (bit 31 stripped).
 * Data bits 0-20, parity bits 21-30.
 */

/* GF(2^5) primitive polynomial: x^5 + x^2 + 1 = 0x25 */
#define BCH_GF_PRIM_POLY	0x25
#define BCH_CODE_LEN		31	/* 2^5 - 1 */

/* GF(2^5) field tables */
static unsigned char bch_exp_tbl[32];	/* alpha^i → polynomial representation */
static unsigned char bch_log_tbl[32];	/* polynomial → exponent */

/* Syndrome tables: S1[i] = alpha^i, S3[i] = alpha^(3i) for i=0..30 */
static unsigned char bch_s1_tbl[BCH_CODE_LEN];
static unsigned char bch_s3_tbl[BCH_CODE_LEN];

/* Syndrome key for each single-bit error position */
static unsigned int bch_bit_key[BCH_CODE_LEN];

/* Error correction lookup: (S1<<5)|S3 → error bit pattern (0 = uncorrectable) */
static unsigned int bch_err_tbl[1024];

static int bch_tables_initialized = 0;

/* Multiply two GF(2^5) elements using log/exp tables */
static unsigned char gf_mult(unsigned char a, unsigned char b)
{
	if (a == 0 || b == 0)
		return 0;
	return bch_exp_tbl[(bch_log_tbl[a] + bch_log_tbl[b]) % BCH_CODE_LEN];
}

/* Build all GF(2^5) tables and error correction lookup.
 * Called once at first use. */
static void flex_bch_init(void)
{
	unsigned int elem;
	int i, j;
	int seen[32] = {0};
	int roots[FLEX_BCH_ECC_BITS];
	int num_roots = 0;
	unsigned char gen_poly[FLEX_BCH_ECC_BITS + 1];

	if (bch_tables_initialized)
		return;

	/* Build GF(2^5) exp/log tables from primitive polynomial */
	elem = 1;
	for (i = 0; i < BCH_CODE_LEN; i++) {
		bch_exp_tbl[i] = elem;
		bch_log_tbl[elem] = i;
		elem <<= 1;
		if (elem & 0x20)
			elem ^= BCH_GF_PRIM_POLY;
	}
	bch_exp_tbl[BCH_CODE_LEN] = bch_exp_tbl[0];
	bch_log_tbl[0] = 0;

	/* Build syndrome tables */
	for (i = 0; i < BCH_CODE_LEN; i++) {
		bch_s1_tbl[i] = bch_exp_tbl[i];
		bch_s3_tbl[i] = bch_exp_tbl[(3 * i) % BCH_CODE_LEN];
	}

	/* Build generator polynomial g(x) from roots.
	 * BCH(31,21,2): roots are alpha^1..alpha^4 and their conjugates
	 * (cyclotomic cosets mod 31 under squaring). */
	for (i = 1; i <= 4; i++) {
		int val = i;
		while (!seen[val]) {
			seen[val] = 1;
			roots[num_roots++] = val;
			val = (val * 2) % BCH_CODE_LEN;
		}
	}

	/* g(x) = product of (x + alpha^root) for all roots */
	gen_poly[0] = 1;
	for (i = 1; i <= FLEX_BCH_ECC_BITS; i++)
		gen_poly[i] = 0;

	{
		int degree = 0;
		for (i = 0; i < num_roots; i++) {
			unsigned char alpha_root = bch_exp_tbl[roots[i]];
			for (j = degree + 1; j > 0; j--)
				gen_poly[j] = gen_poly[j - 1] ^
					       gf_mult(gen_poly[j], alpha_root);
			gen_poly[0] = gf_mult(gen_poly[0], alpha_root);
			degree++;
		}
	}
	(void)gen_poly; /* Used implicitly via syndrome tables */

	/* Build error correction lookup table.
	 * For each bit position, compute syndrome key = (S1<<5)|S3.
	 * Bit position in uint32: bit 0 = recv[30], bit 30 = recv[0]. */
	memset(bch_err_tbl, 0, sizeof(bch_err_tbl));

	for (i = 0; i < BCH_CODE_LEN; i++) {
		int recv_idx = 30 - i;
		unsigned int s1 = bch_s1_tbl[recv_idx];
		unsigned int s3 = bch_s3_tbl[recv_idx];
		unsigned int key = (s1 << 5) | s3;
		bch_bit_key[i] = key;
		bch_err_tbl[key] = 1U << i;
	}

	/* Two-bit error patterns */
	for (i = 0; i < BCH_CODE_LEN; i++) {
		for (j = i + 1; j < BCH_CODE_LEN; j++) {
			unsigned int key = bch_bit_key[i] ^ bch_bit_key[j];
			if (bch_err_tbl[key] == 0)
				bch_err_tbl[key] = (1U << i) | (1U << j);
		}
	}

	bch_tables_initialized = 1;
}

/* Compute syndrome key (S1<<5)|S3 for a 31-bit codeword.
 * Iterates over set bits for efficiency. */
static unsigned int flex_bch_syndrome(uint32_t codeword)
{
	unsigned int s1 = 0, s3 = 0;

	while (codeword) {
		int bit = __builtin_ctz(codeword);
		int recv_idx = 30 - bit;
		s1 ^= bch_s1_tbl[recv_idx];
		s3 ^= bch_s3_tbl[recv_idx];
		codeword &= codeword - 1; /* clear lowest set bit */
	}

	return (s1 << 5) | s3;
}

/* Count set bits (Hamming weight). */
static unsigned int count_bits(uint32_t data)
{
#ifdef __GNUC__
	return __builtin_popcount(data);
#else
	unsigned int n = (data >> 1) & 0x77777777;
	data = data - n;
	n = (n >> 1) & 0x77777777;
	data = data - n;
	n = (n >> 1) & 0x77777777;
	data = data - n;
	data = (data + (data >> 4)) & 0x0f0f0f0f;
	data = data * 0x01010101;
	return data >> 24;
#endif
}

/* BCH(31,21)+parity decoder.
 *
 * Input: 32-bit codeword as accumulated by RX:
 *   bits 0-20 = data (21 info bits)
 *   bits 21-30 = ECC (10 check bits)
 *   bit 31 = even parity
 *
 * The even parity bit extends the BCH(31,21) minimum distance from 5 to 6,
 * enabling simultaneous 2-bit correction + 3-bit detection (same as POCSAG).
 * This matters because we may be looking at random bits from a sliding
 * bitstream — parity halves the false-accept rate for syndrome=0 matches.
 *
 * Decision matrix:
 *   syndrome=0, parity OK  → clean (0 errors)
 *   syndrome=0, parity BAD → 1 error in parity bit only → correctable
 *   syndrome≠0, correction found, parity OK after correction → corrected
 *   syndrome≠0, correction found, parity BAD after correction → 3+ errors, reject
 *   syndrome≠0, no correction found → uncorrectable, reject
 *
 * Returns corrected 21-bit info (bits 0-20), or -1 if uncorrectable.
 *
 * Output parameters:
 *   *status:       0 = clean, 1 = corrected, -1 = uncorrectable.
 *   *corrected_p:  corrected full 32-bit word (code31 + recomputed parity).
 *                  Only written on success (return >= 0).  May be NULL if
 *                  the caller only needs the 21-bit info.
 *
 * Also updates flex->bch_stats counters. */
static int32_t flex_bch_decode(flex_t *flex, uint32_t codeword,
			       int *status, uint32_t *corrected_p)
{
	unsigned int key, error;
	uint32_t code31, parity;
	int parity_bad;

	flex_bch_init();

	code31 = codeword & 0x7FFFFFFFU;
	key = flex_bch_syndrome(code31);
	parity_bad = count_bits(codeword) & 1; /* odd popcount = parity error */

	flex->bch_stats.total++;

	if (key == 0) {
		if (!parity_bad) {
			/* syndrome=0, parity OK → clean */
			*status = 0;
			flex->bch_stats.clean++;
		} else {
			/* syndrome=0, parity BAD → single error in parity bit.
			 * code31 is a valid BCH codeword; only the parity bit
			 * was flipped.  Treat as corrected. */
			*status = 1;
			flex->bch_stats.corrected++;
		}
		goto success;
	}

	error = bch_err_tbl[key];
	if (error == 0) {
		/* syndrome≠0, no correction → uncorrectable */
		*status = -1;
		flex->bch_stats.uncorrectable++;
		return -1;
	}

	/* Apply BCH correction to code31, then recheck parity.
	 * Reassemble the full 32-bit word with the RECEIVED parity bit
	 * and the CORRECTED code31.  If parity is now OK, the correction
	 * was valid (1 or 2 bit errors in code31).  If parity is still
	 * BAD, there were 3+ total errors and BCH miscorrected — reject. */
	code31 ^= error;
	if (count_bits((codeword & 0x80000000U) | code31) & 1) {
		/* parity BAD after correction → 3+ errors, reject */
		*status = -1;
		flex->bch_stats.uncorrectable++;
		return -1;
	}

	*status = 1;
	flex->bch_stats.corrected++;

success:
	/* Recompute correct parity from (possibly corrected) code31.
	 * Even parity: set bit 31 so that popcount of all 32 bits is even. */
	parity = (count_bits(code31) & 1) ? 1U : 0U;

	if (corrected_p)
		*corrected_p = (parity << 31) | code31;

	return (int32_t)(code31 & FLEX_DATA_MASK);
}

/* De-interleave one block of 8 × 32-bit words.
 * Reverses the column-wise interleaving done by flex_interleave_block().
 *
 * NOTE: Not used in the current RX path because the idx formula in
 * flex_rx_read_data() performs de-interleaving inline during bit
 * accumulation. Kept for potential future use (e.g., raw bit buffer
 * approach).
 */
static void __attribute__((unused)) flex_deinterleave_block(uint32_t *words)
{
	uint8_t src[FLEX_CODEWORD_BITS]; /* 32 bytes of interleaved data */
	uint32_t out[FLEX_WORDS_PER_BLOCK];
	int i, w;

	memcpy(src, words, sizeof(src));
	memset(out, 0, sizeof(out));

	for (i = 0; i < (int)FLEX_CODEWORD_BITS; i++) {
		for (w = 0; w < (int)FLEX_WORDS_PER_BLOCK; w++) {
			if (src[i] & (1 << (7 - w)))
				out[w] |= (1U << (31 - i));
		}
	}

	memcpy(words, out, sizeof(out));
}

/* Check a 64-bit buffer against the FLEX sync pattern.
 * Returns the outer sync code (upper 16 bits) if matched, 0 otherwise.
 * The 64-bit sync word is: AAAA:BBBBBBBB:CCCC where
 *   BBBBBBBB = 0xA6C6AAAA (marker)
 *   AAAA ^ CCCC = 0xFFFF
 * We allow Hamming distance < 4 on both marker and outer code. */
static unsigned int flex_sync_check(uint64_t buf)
{
	uint32_t marker = (buf & 0x0000FFFFFFFF0000ULL) >> 16;
	uint16_t codehigh = (buf & 0xFFFF000000000000ULL) >> 48;
	uint16_t codelow = ~(uint16_t)(buf & 0x000000000000FFFFULL);

	if (count_bits(marker ^ FLEX_SYNC_MARKER) < 4 &&
	    count_bits((uint32_t)(codelow ^ codehigh)) < 4)
		return codehigh;

	return 0;
}

/* Try to detect a sync pattern from the 64-bit shift register.
 * Sets polarity (0=normal, 1=inverted) and returns the sync code, or 0. */
static unsigned int flex_rx_sync_detect(flex_t __attribute__((unused)) *flex, uint64_t syncbuf, int *polarity)
{
	unsigned int code;

	code = flex_sync_check(syncbuf);
	if (code != 0) {
		*polarity = 0;
		return code;
	}

	/* Try inverted */
	code = flex_sync_check(~syncbuf);
	if (code != 0) {
		*polarity = 1;
		return code;
	}

	return 0;
}


/* Extract the 32-bit A code from the sync shift register at detection time.
 *
 * Called when sync fires at bit 96 of S1.  At this point sync_buf_lo
 * contains the same 64-bit pattern as the old single sync_buf:
 *   [63:32] = A (32 bits), [31:16] = B (16 bits), [15:0] = inv.A_upper16.
 *
 * With negative polarity (polarity=0), the FSK mapping inverts all bits,
 * so the register contains ~A at [63:32].  Invert to recover A.
 * With positive polarity (polarity=1), the register contains A directly. */
static uint32_t flex_extract_a_code(uint64_t sync_buf_lo, int polarity)
{
	uint32_t raw = (uint32_t)(sync_buf_lo >> 32);
	return polarity ? raw : ~raw;
}

/* Extract inv.A from the 128-bit register after S1 completes.
 *
 * Called at s1_tail_count == 16, after the inv.A tail has been shifted in.
 *
 * Register layout (bit 0 = last shifted in):
 *   lo[31:0] = inv.A (32 bits)
 *
 * At sync detection, inv.A_first16 was at lo[15:0].  After 16 more
 * shifts: inv.A_first16 moved to lo[31:16], inv.A_last16 at lo[15:0].
 *
 * Raw bits (non-polarity-corrected, same as SYNC1):
 *   pol=1 (NEG TX): register = transmitted → inv.A directly.
 *   pol=0 (POS TX): register = ~transmitted → invert to recover. */
static uint32_t flex_extract_inv_a(uint64_t sync_buf_lo, int polarity)
{
	uint32_t raw = (uint32_t)(sync_buf_lo & 0xFFFFFFFF);
	return polarity ? raw : ~raw;
}

/* Combined A / inv.A error correction for S1 sync.
 *
 * A and ~inv.A are both BCH(31,21)+parity codewords encoding the same
 * 21-bit info.  By decoding both independently and cross-validating,
 * we can recover from more errors than BCH alone (which handles ≤2).
 *
 * Algorithm:
 *   1. BCH decode A → status_a, corrected_a
 *   2. BCH decode ~inv.A → status_b, corrected_b
 *   3. Both OK + match → high confidence, use corrected value
 *   4. Both OK + mismatch → not a sync frame, reject
 *   5. One OK, one failed → XOR raw_failed with corrected_good to find
 *      candidate error bits.  If ≤3 diffs, try flipping each one
 *      individually in the raw failed value (one at a time, always
 *      starting from the original raw) and re-attempt BCH.  If BCH
 *      succeeds and matches the good side → recovered.  If no single
 *      flip recovers it → uncorrectable, reject.
 *   6. Both failed → no sync
 *
 * Returns the corrected 32-bit A codeword, or 0 on failure.
 * The 16-bit sync code is derived as (~result >> 16) & 0xFFFF. */
static uint32_t flex_combined_a_correction(flex_t *flex,
					   uint32_t a_raw,
					   uint32_t inv_a_raw)
{
	uint32_t b_raw = ~inv_a_raw;  /* ~inv.A should equal A */
	uint32_t corr_a = 0, corr_b = 0;
	int status_a, status_b;
	int32_t info_a, info_b;

	info_a = flex_bch_decode(flex, a_raw, &status_a, &corr_a);
	info_b = flex_bch_decode(flex, b_raw, &status_b, &corr_b);

	/* Case 1: both decoded successfully */
	if (info_a >= 0 && info_b >= 0) {
		if (corr_a == corr_b) {
			/* Match — high confidence */
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "RX: A/inv.A both OK, match "
				  "(A:%s, ~inv.A:%s) → 0x%08X\n",
				  status_a == 0 ? "clean" : "corrected",
				  status_b == 0 ? "clean" : "corrected",
				  corr_a);
			return corr_a;
		}
		/* Both decoded OK but disagree — not a real sync frame.
		 * A and inv.A are redundant copies; if both pass BCH
		 * independently yet produce different results, the sync
		 * detection was a false positive. */
		LOGP_CHAN(DDSP, LOGL_NOTICE,
			  "RX: A/inv.A both %s but disagree "
			  "(A=0x%08X, ~inv.A=0x%08X) — not a sync frame\n",
			  (status_a == 0 && status_b == 0) ? "clean" : "decoded",
			  corr_a, corr_b);
		return 0;
	}

	/* Case 2: one OK, one failed — try to rescue the failed side */
	if (info_a >= 0 || info_b >= 0) {
		uint32_t good = (info_a >= 0) ? corr_a : corr_b;
		uint32_t bad_raw = (info_a >= 0) ? b_raw : a_raw;
		const char *good_name = (info_a >= 0) ? "A" : "~inv.A";
		const char *bad_name = (info_a >= 0) ? "~inv.A" : "A";
		uint32_t diff = good ^ bad_raw;
		int ndiff = (int)count_bits(diff);

		LOGP_CHAN(DDSP, LOGL_DEBUG,
			  "RX: %s OK (0x%08X), %s failed "
			  "(raw=0x%08X, %d bit diff)\n",
			  good_name, good, bad_name, bad_raw, ndiff);

		if (ndiff <= 3) {
			/* Try flipping each diff bit in bad_raw and
			 * re-attempt BCH.  If it succeeds and matches
			 * the good side, we've recovered.
			 *
			 * Save/restore bch_stats so rescue attempts
			 * don't inflate the per-frame counters. */
			typeof(flex->bch_stats) saved_stats = flex->bch_stats;
			uint32_t remaining = diff;
			while (remaining) {
				int bit = __builtin_ctz(remaining);
				uint32_t candidate = bad_raw ^ (1U << bit);
				int try_status;
				uint32_t try_corr = 0;
				int32_t try_info = flex_bch_decode(flex,
					candidate, &try_status, &try_corr);
				if (try_info >= 0 && try_corr == good) {
					flex->bch_stats = saved_stats;
					LOGP_CHAN(DDSP, LOGL_INFO,
						  "RX: %s recovered by "
						  "flipping bit %d "
						  "(raw 0x%08X → 0x%08X)\n",
						  bad_name, bit,
						  bad_raw, try_corr);
					return good;
				}
				remaining &= remaining - 1;
			}
			flex->bch_stats = saved_stats;
		}

		/* Rescue failed — too many errors, uncorrectable */
		LOGP_CHAN(DDSP, LOGL_NOTICE,
			  "RX: %s rescue failed (%d diffs), "
			  "uncorrectable\n",
			  bad_name, ndiff);
		return 0;
	}

	/* Case 3: both failed */
	LOGP_CHAN(DDSP, LOGL_NOTICE,
		  "RX: A/inv.A both BCH-failed "
		  "(A=0x%08X, ~inv.A=0x%08X)\n",
		  a_raw, b_raw);
	return 0;
}

/* Decode the sync code to determine symbol rate and FSK levels.
 * The outer code determines the mode.
 *
 * Uses BCH(31,21) error correction on the bit-reversed A code to
 * correct up to 2 bit errors before mode identification.
 *
 * Returns 1 if valid mode found, 0 otherwise. */
static int flex_rx_decode_mode(flex_t *flex, unsigned int sync_code,
			       int polarity)
{
	/* Mode table: sync code → symbol rate (baud) and FSK levels.
	 *
	 * Sync code to mode mapping:
	 *   A1: 1600bps/2FSK → 1600 baud, 2 levels (1 bit/symbol)
	 *   A2: 3200bps/2FSK → 3200 baud, 2 levels (1 bit/symbol)
	 *   A3: 3200bps/4FSK → 1600 baud, 4 levels (2 bits/symbol = dibit)
	 *   A4: 6400bps/4FSK → 3200 baud, 4 levels (2 bits/symbol = dibit)
	 *
	 * 'baud' is the SYMBOL rate, not the bit rate.
	 * Bit rate = baud × bits_per_symbol (1 for 2FSK, 2 for 4FSK). */
	static const struct {
		uint16_t code;
		int baud;	/* symbol rate (symbols/second) */
		int levels;	/* 2 = 2FSK (1 bit/sym), 4 = 4FSK (2 bits/sym) */
	} modes[] = {
		{ FLEX_SYNC_A1, 1600, 2 },	/* A1: 1600bps/2FSK, 1600 baud */
		{ FLEX_SYNC_A3, 1600, 4 },	/* A3: 3200bps/4FSK, 1600 baud */
		{ FLEX_SYNC_A2, 3200, 2 },	/* A2: 3200bps/2FSK, 3200 baud */
		{ FLEX_SYNC_A4, 3200, 4 },	/* A4: 6400bps/4FSK, 3200 baud */
		{ 0, 0, 0 }
	};
	int i;
	unsigned int corrected_code;

	/* BCH-correct the A code for reliable mode identification.
	 *
	 * The A-code from flex_extract_a_code() is in TX bit order (MSB-first),
	 * which is reverse_bits32(encoder_output).  This is the same layout as
	 * normal RX codewords (FIW, data words) accumulated LSB-first — so we
	 * pass it directly to flex_bch_decode(), no bit reversal needed.
	 * Verified: all 16 A-codes produce syndrome 0 in this layout.
	 *
	 * flex_bch_decode() returns the corrected full 32-bit word (code31 +
	 * recomputed parity) via corrected_word.  The 16-bit outer sync code
	 * is (~corrected_word >> 16) & 0xFFFF, matching FLEX_SYNC_A1..AR.
	 *
	 * This is a tentative decode — the authoritative BCH validation
	 * happens later in flex_combined_a_correction() once inv.A is
	 * available.  Save/restore bch_stats so this preliminary decode
	 * doesn't inflate the per-frame counters. */
	{
		typeof(flex->bch_stats) saved_stats = flex->bch_stats;
		uint32_t a_code = flex_extract_a_code(flex->rx.sync_buf_lo,
						      polarity);
		flex->rx.sync_a_code = a_code;
		uint32_t corrected_word;
		int bch_status;
		int32_t info = flex_bch_decode(flex, a_code, &bch_status,
					       &corrected_word);
		flex->bch_stats = saved_stats;
		if (info >= 0)
			corrected_code = (~corrected_word >> 16) & 0xFFFF;
		else
			corrected_code = 0;

		if (corrected_code != 0 && corrected_code != sync_code) {
			LOGP_CHAN(DDSP, LOGL_INFO,
				  "RX: BCH corrected sync code 0x%04X → 0x%04X.\n",
				  sync_code, corrected_code);
			sync_code = corrected_code;
		}
	}

	/* Check for Ar (ERS re-sync).
	 * When the receiver detects the Ar
	 * sync code, it must re-synchronize its frame timing.  ERS is not
	 * a data frame — no FIW/S2/DATA follows.  The receiver should reset
	 * to sync-hunting state so it can lock onto the next data frame's
	 * S1 sync after the ERS burst ends. */
	if (sync_code == FLEX_SYNC_AR) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "RX: Ar (ERS re-sync) detected — resetting sync.\n");
		return 0;
	}

	/* ReFLEX sync (0x4C7C): Motorola ReFLEX protocol extension.
	 * Same physical layer as A4 (6400bps/4FSK, 3200 baud) but
	 * uses a different framing format.  Decode what we can as
	 * standard FLEX, hex-dump the rest. */
	if (sync_code == FLEX_SYNC_REFLEX) {
		flex->rx.sync_baud = 3200;
		flex->rx.sync_levels = 4;
		flex->rx.reflex = 1;
		LOGP_CHAN(DDSP, LOGL_NOTICE,
			  "RX: ReFLEX sync detected — code=0x%04X, 6400bps/4FSK, 3200 baud, polarity=%s (stub).\n",
			  sync_code,
			  flex->rx.polarity ? "NEG" : "POS");
		return 1;
	}

	flex->rx.reflex = 0;

	for (i = 0; modes[i].code != 0; i++) {
		if (sync_code == modes[i].code) {
			flex->rx.sync_baud = modes[i].baud;
			flex->rx.sync_levels = modes[i].levels;
			LOGP_CHAN(DDSP, LOGL_INFO,
				  "RX: Sync detected — code=0x%04X, %dbps/%dFSK, %d baud, polarity=%s.\n",
				  sync_code,
				  modes[i].baud * (modes[i].levels == 4 ? 2 : 1),
				  modes[i].levels,
				  modes[i].baud,
				  flex->rx.polarity ? "NEG" : "POS");
			return 1;
		}
	}

	LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: Unknown sync code 0x%04X.\n", sync_code);
	return 0;
}

/* Decode a received FIW codeword to extract cycle and frame numbers.
 * Returns 0 on success, -1 on decode failure. */
static int flex_rx_decode_fiw(flex_t *flex, uint32_t fiw_raw)
{
	int32_t data;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: FIW raw accumulated = 0x%08X\n", fiw_raw);

	/* RX accumulated LSB-first: bits 0-20 = data, 21-30 = ECC, 31 = parity.
	 * This is the natural layout for the BCH decoder — no bit reversal needed. */
	int bch_status;
	data = flex_bch_decode(flex, fiw_raw, &bch_status, NULL);
	if (data < 0) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "RX: FIW BCH decode failed (uncorrectable, raw=0x%08X).\n",
			  fiw_raw);
		return -1;
	}
	if (bch_status == 1) {
		LOGP_CHAN(DDSP, LOGL_INFO,
			  "RX: FIW BCH corrected (raw=0x%08X → data=0x%05X).\n",
			  fiw_raw, (unsigned int)data);
	}

	/* FIW layout (21 data bits):
	 *   bits 0-3:   x (4-bit checksum)
	 *   bits 4-7:   c (cycle number, 0-14)
	 *   bits 8-14:  f (frame number, 0-127)
	 *   bit  15:    n (roaming: 1=allowed, 0=not allowed)
	 *   bit  16:    r (multiple transmission indication)
	 *   bits 17-20: t (depends on r — see below)
	 *
	 * When r=0: transmissions=1x, t3-t0 are Low Traffic Flags
	 *   per phase (d,c,b,a).  t=1 means address field ≤ block 0.
	 * When r=1: [t1,t0] = num transmissions (01=2x, 10=3x, 11=4x)
	 *           [t3,t2] = TD Collapse cycle override
	 *             (00=system, 01=value 6, 10=value 7, 11=value 5)
	 *
	 * Checksum: sum of all 4-bit nibbles + bit 20 = 0xF */
	{
		unsigned int checksum = (data & FLEX_FIW_CHECKSUM_MASK);
		checksum += ((data >> 4) & 0xF);
		checksum += ((data >> 8) & 0xF);
		checksum += ((data >> 12) & 0xF);
		checksum += ((data >> 16) & 0xF);
		checksum += ((data >> 20) & 0x01);
		checksum &= FLEX_FIW_CHECKSUM_MASK;

		if (checksum != FLEX_FIW_CHECKSUM_OK) {
			LOGP_CHAN(DDSP, LOGL_NOTICE,
				  "RX: FIW checksum failed (0x%X != 0xF).\n", checksum);
			return -1;
		}
	}

	flex->rx.fiw_cycle = (data >> FLEX_FIW_CYCLE_SHIFT) & FLEX_FIW_CYCLE_MASK;
	flex->rx.fiw_frame = (data >> FLEX_FIW_FRAME_SHIFT) & FLEX_FIW_FRAME_MASK;
	flex->rx.fiw_roaming = (data >> FLEX_FIW_ROAMING_SHIFT) & 0x01;
	flex->rx.fiw_repeat = (data >> FLEX_FIW_REPEAT_SHIFT) & 0x01;
	flex->rx.fiw_traffic = (data >> FLEX_FIW_TRAFFIC_SHIFT) & FLEX_FIW_TRAFFIC_MASK;

	/* Decode multiple transmission parameters.
	 *
	 * When r=1, [t1,t0] define the number of transmissions:
	 *   01=2x, 10=3x, 11=4x, 00=reserved.
	 * And [t3,t2] define the TD Collapse cycle override:
	 *   00=use dictated System Collapse cycle,
	 *   01=TD Collapse cycle value 6,
	 *   10=TD Collapse cycle value 7,
	 *   11=TD Collapse cycle value 5.
	 * When TD Collapse is used, it takes priority over System Collapse.
	 *
	 * When r=0, transmissions=1x and t3-t0 are Low Traffic Flags. */
	if (flex->rx.fiw_repeat) {
		uint32_t t10 = flex->rx.fiw_traffic & 0x03;
		uint32_t t32 = (flex->rx.fiw_traffic >> 2) & 0x03;

		switch (t10) {
		case 0x01: flex->rx.fiw_num_transmissions = 2; break;
		case 0x02: flex->rx.fiw_num_transmissions = 3; break;
		case 0x03: flex->rx.fiw_num_transmissions = 4; break;
		default:   flex->rx.fiw_num_transmissions = 1; break; /* 00=reserved */
		}

		switch (t32) {
		case 0x01: flex->rx.fiw_td_collapse = 6; break;
		case 0x02: flex->rx.fiw_td_collapse = 7; break;
		case 0x03: flex->rx.fiw_td_collapse = 5; break;
		default:   flex->rx.fiw_td_collapse = -1; break; /* 00=use system */
		}

		LOGP_CHAN(DDSP, LOGL_INFO,
			  "RX: FIW decoded — cycle=%u frame=%u roaming=%u "
			  "repeat=%ux td_collapse=%s\n",
			  flex->rx.fiw_cycle, flex->rx.fiw_frame,
			  flex->rx.fiw_roaming,
			  flex->rx.fiw_num_transmissions,
			  flex->rx.fiw_td_collapse >= 0 ?
			    (flex->rx.fiw_td_collapse == 5 ? "5" :
			     flex->rx.fiw_td_collapse == 6 ? "6" : "7") :
			    "system");
	} else {
		flex->rx.fiw_num_transmissions = 1;
		flex->rx.fiw_td_collapse = -1;

		LOGP_CHAN(DDSP, LOGL_INFO,
			  "RX: FIW decoded — cycle=%u frame=%u roaming=%u "
			  "traffic=0x%X (low_traffic flags)\n",
			  flex->rx.fiw_cycle, flex->rx.fiw_frame,
			  flex->rx.fiw_roaming,
			  flex->rx.fiw_traffic);
	}

	return 0;
}

/* ===== Fragment Reassembly Helpers ===== */

/* Find or allocate a reassembly slot for the given capcode + message number.
 * Returns slot index, or -1 if no slot available. */
static int reasm_find(flex_t *flex, uint64_t capcode, int msg_num)
{
	int i;
	for (i = 0; i < FLEX_REASM_SLOTS; i++) {
		if (flex->rx.reasm[i].active &&
		    flex->rx.reasm[i].capcode == capcode &&
		    flex->rx.reasm[i].msg_num == msg_num)
			return i;
	}
	return -1;
}

static int reasm_alloc(flex_t *flex, uint64_t capcode, int msg_num, int msg_type)
{
	int i;
	/* Find a free slot */
	for (i = 0; i < FLEX_REASM_SLOTS; i++) {
		if (!flex->rx.reasm[i].active)
			goto found;
	}
	/* Evict oldest slot */
	{
		int oldest = 0;
		uint32_t oldest_frame = flex->rx.reasm[0].last_frame;
		for (i = 1; i < FLEX_REASM_SLOTS; i++) {
			if (flex->rx.reasm[i].last_frame < oldest_frame) {
				oldest = i;
				oldest_frame = flex->rx.reasm[i].last_frame;
			}
		}
		i = oldest;
	}
found:
	flex->rx.reasm[i].active = 1;
	flex->rx.reasm[i].capcode = capcode;
	flex->rx.reasm[i].msg_num = msg_num;
	flex->rx.reasm[i].len = 0;
	flex->rx.reasm[i].msg_type = msg_type;
	flex->rx.reasm[i].kanji = 0;
	flex->rx.reasm[i].secure_subtype = -1;
	flex->rx.reasm[i].expected_f = 0; /* next after initial (F=11) is F=00 */
	flex->rx.reasm[i].last_frame = flex->rx.fiw_frame;
	flex->rx.reasm[i].last_cycle = flex->rx.fiw_cycle;
	return i;
}

/* Append text to a reassembly slot. Returns 0 on success, -1 if full. */
static int reasm_append(flex_t *flex, int slot, const char *text, int text_len)
{
	int avail = FLEX_REASM_MAX_LEN - 1 - flex->rx.reasm[slot].len;
	int copy = (text_len < avail) ? text_len : avail;
	if (copy <= 0)
		return -1;
	memcpy(flex->rx.reasm[slot].buf + flex->rx.reasm[slot].len, text, copy);
	flex->rx.reasm[slot].len += copy;
	flex->rx.reasm[slot].buf[flex->rx.reasm[slot].len] = '\0';
	flex->rx.reasm[slot].last_frame = flex->rx.fiw_frame;
	flex->rx.reasm[slot].last_cycle = flex->rx.fiw_cycle;
	return 0;
}

/* Expire stale reassembly slots (per spec: 32 frames max between fragments,
 * we use 64 as a generous timeout). Outputs partial result on expiry. */
static void reasm_expire(flex_t *flex)
{
	int i;
	uint32_t cur = flex->rx.fiw_cycle * 128 + flex->rx.fiw_frame;
	for (i = 0; i < FLEX_REASM_SLOTS; i++) {
		uint32_t last;
		int delta;
		if (!flex->rx.reasm[i].active)
			continue;
		last = flex->rx.reasm[i].last_cycle * 128 + flex->rx.reasm[i].last_frame;
		delta = (int)cur - (int)last;
		if (delta < 0)
			delta += 15 * 128; /* wrap around cycle boundary */
		if (delta > FLEX_REASM_TIMEOUT) {
			if (flex->rx.reasm[i].len > 0) {
				const char *type_tag =
					(flex->rx.reasm[i].msg_type == FLEX_VECTOR_TYPE_HEX_BINARY)
					? "HEX" :
					(flex->rx.reasm[i].msg_type == FLEX_VECTOR_TYPE_SECURE)
					? "SEC" : "ALN";
				if (flex->rx.reasm[i].msg_type == FLEX_VECTOR_TYPE_SECURE)
					LOGP(DDSP, LOGL_NOTICE,
					     "RX: reassembly timeout [%09" PRIu64 "] msgnum=%d (%d frames stale) partial %s t1t0=%d \"%s\"\n",
					     flex->rx.reasm[i].capcode,
					     flex->rx.reasm[i].msg_num, delta,
					     type_tag,
					     flex->rx.reasm[i].secure_subtype,
					     flex->rx.reasm[i].buf);
				else
					LOGP(DDSP, LOGL_NOTICE,
					     "RX: reassembly timeout [%09" PRIu64 "] msgnum=%d (%d frames stale) partial %s \"%s\"\n",
					     flex->rx.reasm[i].capcode,
					     flex->rx.reasm[i].msg_num, delta,
					     type_tag,
					     flex->rx.reasm[i].buf);
			} else {
				LOGP(DDSP, LOGL_DEBUG,
				     "RX: reassembly timeout for capcode %" PRIu64 " msgnum=%d (%d frames stale, empty).\n",
				     flex->rx.reasm[i].capcode, flex->rx.reasm[i].msg_num, delta);
			}
			flex->rx.reasm[i].active = 0;
		}
	}
}

/* Decode one phase of a received frame.
 * De-interleave blocks, BCH decode, parse BIW/addresses/vectors/messages.
 * phaseptr points to 88 words of raw interleaved data. */
static void flex_rx_decode_phase(flex_t *flex, flex_phase_data_t *ph, char phase_name)
{
	int i;
	int bitrate = flex->rx.sync_baud * (flex->rx.sync_levels == 4 ? 2 : 1);

	/* Expire stale reassembly slots at the start of each phase decode */
	reasm_expire(flex);

	/* Store reception context in the phase struct */
	ph->rx_phase = phase_name - 'A';	/* 'A'→0, 'B'→1, 'C'→2, 'D'→3 */
	ph->rx_cycle = flex->rx.fiw_cycle;
	ph->rx_frame = flex->rx.fiw_frame;
	ph->rx_baud = flex->rx.sync_baud;
	ph->rx_levels = flex->rx.sync_levels;
	ph->rx_polarity = flex->rx.polarity;

	LOGP_CHAN(DDSP, LOGL_INFO, "RX: Decoding phase %c (C%u/F%u, %dbps/%dFSK, %d baud, %ux tx).\n",
		  phase_name, flex->rx.fiw_cycle, flex->rx.fiw_frame,
		  bitrate, flex->rx.sync_levels, flex->rx.sync_baud,
		  flex->rx.fiw_num_transmissions);

	/* BCH(31,21) decode each word.
	 * "BCH (31, 21) codes and even parity can be detected and processed
	 * by the 2-bit error correction algorithm.  The error condition is
	 * checked for each word and the information bits are extracted."
	 *
	 * On success: ph->words[i] = 21-bit info, ph->status[i] = CLEAN/CORRECTED.
	 * On failure: ph->words[i] = raw 32-bit codeword, ph->status[i] = UNCORRECTABLE. */
	uint32_t raw_words[FLEX_WORDS_PER_FRAME];

	for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
		int bch_status;

		raw_words[i] = ph->words[i];
		int32_t result = flex_bch_decode(flex, ph->words[i], &bch_status, NULL);
		if (result < 0) {
			ph->status[i] = FLEX_WORD_UNCORRECTABLE;
		} else {
			ph->words[i] = (uint32_t)result;
			ph->status[i] = (bch_status == 0) ? FLEX_WORD_CLEAN
							   : FLEX_WORD_CORRECTED;
		}
	}

	/* Per-word summary: report corrected and uncorrectable words individually */
	{
		int ok = 0, fixed = 0, fail = 0;
		for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
			if (ph->status[i] == FLEX_WORD_CLEAN)
				ok++;
			else if (ph->status[i] == FLEX_WORD_CORRECTED)
				fixed++;
			else
				fail++;
		}
		LOGP_CHAN(DDSP, LOGL_INFO,
			  "RX: Phase %c BCH: %d/%d clean, %d corrected, %d uncorrectable.\n",
			  phase_name, ok, FLEX_WORDS_PER_FRAME, fixed, fail);

		for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
			if (ph->status[i] == FLEX_WORD_CORRECTED) {
				LOGP_CHAN(DDSP, LOGL_INFO,
					  "RX: Phase %c word %2d BCH corrected "
					  "(raw=0x%08X → data=0x%05X).\n",
					  phase_name, i, raw_words[i],
					  ph->words[i]);
			} else if (ph->status[i] == FLEX_WORD_UNCORRECTABLE) {
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "RX: Phase %c word %2d BCH uncorrectable "
					  "(raw=0x%08X).\n",
					  phase_name, i, raw_words[i]);
			}
		}
	}

	/* BIW1 (word 0) */
	if (ph->status[0] == FLEX_WORD_UNCORRECTABLE ||
	    ph->status[0] == FLEX_WORD_NOT_RECEIVED) {
		LOGP_CHAN(DDSP, LOGL_NOTICE,
			  "RX: Phase %c BIW1 uncorrectable, skipping.\n", phase_name);
		return;
	}

	{
		uint32_t biw = ph->words[0];

		/* Nothing to decode if BIW is idle.
		 * All-zeros or all-ones in the 21-bit data field means
		 * no addresses, vectors, or messages in this frame. */
		if (biw == FLEX_BIW_IDLE_ZEROS || biw == FLEX_BIW_IDLE_ONES) {
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "RX: Phase %c idle frame (BIW=0x%05X).\n",
				  phase_name, biw);
			return;
		}

		/* BIW1 layout (21 data bits):
		 * bits 0-3:   checksum (x)
		 * bits 4-7:   priority address word count (P, 0-15)
		 * bits 8-9:   address field start offset (a, +1 = word index)
		 * bits 10-15: vector field start offset (V, 0-63)
		 * bits 16-17: carry-on (C, 0-3 frames)
		 * bits 18-20: collapse cycle (m, 0-7) */
		int prio    = (biw >> FLEX_BIW1_PRIO_SHIFT) & FLEX_BIW1_PRIO_MASK;
		int voffset = (biw >> FLEX_BIW1_VSTART_SHIFT) & FLEX_BIW1_VSTART_MASK;
		int aoffset = ((biw >> FLEX_BIW1_ASTART_SHIFT) & FLEX_BIW1_ASTART_MASK) + 1;
		int carry   = (biw >> FLEX_BIW1_CARRY_SHIFT) & FLEX_BIW1_CARRY_MASK;
		int collapse = (biw >> FLEX_BIW1_COLLAPSE_SHIFT) & FLEX_BIW1_COLLAPSE_MASK;

		LOGP_CHAN(DDSP, LOGL_DEBUG,
			  "RX: Phase %c BIW1=0x%05X aoffset=%d voffset=%d prio=%d carry=%d collapse=%d (%d pages).\n",
			  phase_name, biw, aoffset, voffset, prio, carry, collapse,
			  voffset - aoffset);

		if (voffset < aoffset || voffset >= FLEX_WORDS_PER_FRAME) {
			LOGP_CHAN(DDSP, LOGL_NOTICE,
				  "RX: Phase %c invalid BIW offsets (aoffset=%d voffset=%d).\n",
				  phase_name, aoffset, voffset);
			return;
		}

		/* Parse BIW2/3/4 (words 1 through aoffset-1).
		 *
		 * BIW1 is always word 0.
		 * Words 1..(aoffset-1) are BIW2/3/4, identified by
		 * their type field (bits 4-6).  The transmission order
		 * is not regulated — we dispatch on type, not position.
		 *
		 * BIW1 aoffset_raw (bits 8-9) = number of extra BIW words.
		 * aoffset = aoffset_raw + 1 = first address word index.
		 * So extra BIW words are at indices 1..(aoffset-1). */
		{
			int bw;
			for (bw = 1; bw < aoffset && bw < FLEX_WORDS_PER_FRAME; bw++) {
				uint32_t bword;
				uint32_t btype;

				if (ph->status[bw] == FLEX_WORD_UNCORRECTABLE ||
				    ph->status[bw] == FLEX_WORD_NOT_RECEIVED) {
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c BIW[%d] uncorrectable.\n",
						  phase_name, bw);
					continue;
				}

				bword = ph->words[bw];
				btype = flex_biw_type(bword);

				switch (btype) {
				case FLEX_BIW_TYPE_SSID1: {
					uint32_t cov = (bword >> FLEX_BIW_SSID1_COVERAGE_SHIFT) & FLEX_BIW_SSID1_COVERAGE_MASK;
					uint32_t lid = (bword >> FLEX_BIW_SSID1_LOCALID_SHIFT) & FLEX_BIW_SSID1_LOCALID_MASK;
					int changed = !flex->rx.biw.seen ||
						      flex->rx.biw.local_id != lid ||
						      flex->rx.biw.coverage != cov;
					LOGP_CHAN(DDSP, changed ? LOGL_NOTICE : LOGL_DEBUG,
						  "RX: %dbps C%u/F%u phase=%c BIW SSID1 local_id=%u coverage=%u%s\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, lid, cov,
						  changed ? "" : " (unchanged)");
					flex->rx.biw.local_id = lid;
					flex->rx.biw.coverage = cov;
					flex->rx.biw.seen = 1;
					break;
				}
				case FLEX_BIW_TYPE_DATE: {
					uint32_t year_raw = (bword >> FLEX_BIW_DATE_YEAR_SHIFT) & FLEX_BIW_DATE_YEAR_MASK;
					uint32_t day  = (bword >> FLEX_BIW_DATE_DAY_SHIFT) & FLEX_BIW_DATE_DAY_MASK;
					uint32_t mon  = (bword >> FLEX_BIW_DATE_MONTH_SHIFT) & FLEX_BIW_DATE_MONTH_MASK;
					int biw_year = (int)year_raw + FLEX_BIW_DATE_YEAR_BASE;
					/* Reverse the calendar-equivalent mapping:
					 * if system year >2025, the TX may have mapped
					 * the real year to an equivalent in 1994-2025. */
					time_t now_t = time(NULL);
					struct tm now_tm;
					gmtime_r(&now_t, &now_tm);
					int sys_year = now_tm.tm_year + 1900;
					int real = flex_biw_real_year(biw_year, sys_year);
					if (real != biw_year) {
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: %dbps C%u/F%u phase=%c BIW DATE %04d-%02u-%02u (probably %d)\n",
							  bitrate,
							  flex->rx.fiw_cycle, flex->rx.fiw_frame,
							  phase_name,
							  biw_year, mon, day, real);
					} else {
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: %dbps C%u/F%u phase=%c BIW DATE %04d-%02u-%02u\n",
							  bitrate,
							  flex->rx.fiw_cycle, flex->rx.fiw_frame,
							  phase_name,
							  biw_year, mon, day);
					}
					flex->rx.biw.date_year = real;
					flex->rx.biw.date_month = mon;
					flex->rx.biw.date_day = day;
					flex->rx.biw.seen_date = 1;
					break;
				}
				case FLEX_BIW_TYPE_TIME: {
					uint32_t hour = (bword >> FLEX_BIW_TIME_HOUR_SHIFT) & FLEX_BIW_TIME_HOUR_MASK;
					uint32_t min  = (bword >> FLEX_BIW_TIME_MINUTE_SHIFT) & FLEX_BIW_TIME_MINUTE_MASK;
					uint32_t sec  = (bword >> FLEX_BIW_TIME_SECOND_SHIFT) & FLEX_BIW_TIME_SECOND_MASK;
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps C%u/F%u phase=%c BIW TIME %02u:%02u:%04.1f\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name,
						  hour, min, sec * FLEX_BIW_TIME_SECOND_STEP);
					flex->rx.biw.time_hour = hour;
					flex->rx.biw.time_minute = min;
					flex->rx.biw.time_second = sec;
					flex->rx.biw.seen_time = 1;
					break;
				}
				case FLEX_BIW_TYPE_SYSINFO: {
					uint32_t a_type = (bword >> FLEX_BIW_SYSINFO_A_SHIFT) & FLEX_BIW_SYSINFO_A_MASK;
					uint32_t info   = (bword >> FLEX_BIW_SYSINFO_I_SHIFT) & FLEX_BIW_SYSINFO_I_MASK;
					if (a_type == FLEX_BIW_SYSINFO_A_TIME ||
					    a_type == FLEX_BIW_SYSINFO_A_TIME_ADD) {
						/* Time-related: I field contains timezone,
						 * DST flag, and extended seconds.
						 * I4-I0: Z4-Z0 timezone zone code
						 * I5:    L0 DST flag (0=DST, 1=standard)
						 * I7-I9: S5-S3 extended seconds */
						uint32_t zone = info & FLEX_BIW_SYSINFO_TZ_MASK;
						uint32_t dst  = (info >> FLEX_BIW_SYSINFO_DST_SHIFT) & FLEX_BIW_SYSINFO_DST_MASK;
						uint32_t esec = (info >> FLEX_BIW_SYSINFO_EXTSEC_SHIFT) & FLEX_BIW_SYSINFO_EXTSEC_MASK;
						int tz_min = flex_tz_to_minutes(zone);
						char tzbuf[20];
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: %dbps C%u/F%u phase=%c BIW SYSINFO A=%u timezone zone=%u (%s) DST=%u extsec=%u\n",
							  bitrate,
							  flex->rx.fiw_cycle, flex->rx.fiw_frame,
							  phase_name, a_type, zone,
							  flex_tz_format(tz_min, tzbuf, sizeof(tzbuf)),
							  dst, esec);
						flex->rx.biw.timezone_zone = zone;
						flex->rx.biw.timezone_offset_min = tz_min;
						flex->rx.biw.timezone_dst = (int)dst;
						flex->rx.biw.timezone_extsec = esec;
						flex->rx.biw.seen_timezone = 1;
					} else if (a_type <= FLEX_BIW_SYSINFO_A_MSG_SSID) {
						/* System Message types A=0000~0011.
						 *
						 * When BIW101 carries A=0000~0100, the frame layout
						 * changes per the standard:
						 *   - System message vectors (except Secure type) are
						 *     placed at the END of the vector field
						 *   - System message body words are in the message field
						 *   - Normal pager addresses/vectors precede them
						 *
						 * The I-field (10 bits) is message-type-specific data.
						 * For A=0000~0011 it typically contains the system
						 * message word count or content identifier.
						 *
						 * Constraints:
						 *   - "Tone-Only Addresses cannot be transmitted in
						 *     Frames used for transmitting System Messages."
						 *   - "The transmission of a System Message by BIW 101
						 *     must be one time per each phase."
						 *   - "Even when data is frequently displayed or updated,
						 *     it can be transmitted within other Frames."
						 *
						 * A-type audience:
						 *   0000 = all pagers in system
						 *   0001 = home area pagers only
						 *   0010 = roaming pagers only
						 *   0011 = SSID-specific pagers
						 *
						 * Can also be transmitted via Operator Messaging Address
						 * per the operator msg decode below. */
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: %dbps C%u/F%u phase=%c BIW SYSINFO %s I=0x%03X"
							  " (vectors at end of VF, body in MF)\n",
							  bitrate,
							  flex->rx.fiw_cycle, flex->rx.fiw_frame,
							  phase_name,
							  flex_biw_sysinfo_a_name(a_type),
							  info);
					} else if (a_type == FLEX_BIW_SYSINFO_A_CHAN_SETUP) {
						/* Channel Set Up Instruction (A=0110).
						 *
						 * I-field layout (10 bits):
						 *   I5-I0 (6 bits): Frame Offset F5-F0 (1-63)
						 *     Tells roaming pagers which frame to monitor
						 *     on the new channel after handoff.
						 *   I7-I6 (2 bits): Max Carry On O1-O0 (0-3)
						 *     Max frames a roaming pager may carry on
						 *     decoding after its assigned frame.
						 *   I8 (1 bit): N0 — NID System Message Bit
						 *     1 = NID system message present in this frame.
						 *   I9 (1 bit): B0 — System Message Bit
						 *     1 = system message present in this frame.
						 *
						 * Example: frame_ofs=10 carry_on=2 nid=0 sysmsg=1
						 *   → roaming pager should tune to frame 10 on new
						 *     channel, may carry on 2 extra frames, and a
						 *     system message is present in this frame. */
						uint32_t frame_ofs = info & FLEX_BIW_SYSINFO_FRAME_OFS_MASK;
						uint32_t carry_on  = (info >> FLEX_BIW_SYSINFO_CARRY_ON_SHIFT) & FLEX_BIW_SYSINFO_CARRY_ON_MASK;
						int nid_bit = (info >> FLEX_BIW_SYSINFO_NID_BIT) & 1;
						int sys_bit = (info >> FLEX_BIW_SYSINFO_SYSMSG_BIT) & 1;
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: %dbps C%u/F%u phase=%c BIW SYSINFO ChanSetup frame_ofs=%u carry_on=%u nid=%d sysmsg=%d\n",
							  bitrate,
							  flex->rx.fiw_cycle, flex->rx.fiw_frame,
							  phase_name,
							  frame_ofs, carry_on, nid_bit, sys_bit);
					} else {
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: %dbps C%u/F%u phase=%c BIW SYSINFO %s info=0x%03X\n",
							  bitrate,
							  flex->rx.fiw_cycle, flex->rx.fiw_frame,
							  phase_name,
							  flex_biw_sysinfo_a_name(a_type),
							  info);
					}
					break;
				}
				case FLEX_BIW_TYPE_SSID2: {
					uint32_t tmf     = (bword >> FLEX_BIW_SSID2_TMF_SHIFT) & FLEX_BIW_SSID2_TMF_MASK;
					uint32_t country = (bword >> FLEX_BIW_SSID2_COUNTRY_SHIFT) & FLEX_BIW_SSID2_COUNTRY_MASK;
					const char *cname = flex_mcc_name(country);
					int changed = !flex->rx.biw.seen_ssid2 ||
						      flex->rx.biw.country != country ||
						      flex->rx.biw.tmf != tmf;
					LOGP_CHAN(DDSP, changed ? LOGL_NOTICE : LOGL_DEBUG,
						  "RX: %dbps C%u/F%u phase=%c BIW SSID2 country=%u (%s) tmf=0x%X%s\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, country,
						  cname ? cname : "unknown",
						  tmf,
						  changed ? "" : " (unchanged)");
					flex->rx.biw.country = country;
					flex->rx.biw.tmf = tmf;
					flex->rx.biw.seen_ssid2 = 1;
					break;
				}
				default:
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c BIW[%d] reserved type=%u data=0x%05X.\n",
						  phase_name, bw, btype, bword);
					break;
				}
			}
		}

		/* Walk address/vector pairs.
		 *
		 * Address/vector layout:
		 * - Short address (1 word): Ax pairs with Vx
		 * - Long address (2 words): Ax,Ay pair with Vx,Vy
		 *   (Ay is 2nd word of Ax; Vy is 2nd word of Vx)
		 * - Tone-only addresses at end of AF have no vector
		 *
		 * addr_idx tracks position in the address field,
		 * vec_count tracks how many vector slots consumed. */
		{
			int addr_idx = aoffset;
			int vec_count = 0;
			/* Per the spec: "Addresses as with higher priority
			 * are sent in the top of the address field... The number of
			 * address words with higher priority is specified by BIW1."
			 * Priority addresses occupy addr_idx [aoffset..aoffset+prio-1]. */
			int prio_end = aoffset + prio; /* first non-priority addr index */

		for (; addr_idx < voffset; addr_idx++) {
			int j = voffset + vec_count; /* vector index */
			uint64_t capcode;
			int is_long = 0;
			int vec_type, mw1, mw2, len;
			uint32_t aw_raw, aw_base;
			int addr_is_group = 0, addr_is_temp = 0;
			enum flex_addr_type aw_type;
			char grp_flag;
			/* Priority addresses are in
			 * addr_idx [aoffset..prio_end-1].  Show 'P' flag
			 * on message output lines for priority addresses. */
			char prio_flag = (addr_idx < prio_end) ? FLEX_RX_FLAG_PRIORITY : ' ';

			if (j >= FLEX_WORDS_PER_FRAME) break;

			if (ph->status[addr_idx] == FLEX_WORD_UNCORRECTABLE ||
			    ph->status[addr_idx] == FLEX_WORD_NOT_RECEIVED) {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c addr[%d] uncorrectable.\n",
					  phase_name, addr_idx);
				vec_count++;
				continue;
			}

			/* Address decode.
			 *
			 * First extract group/temporary flags from bit 20/19,
			 * then classify the base address word.
			 * Long address types (LA1-LA4) require a second word;
			 * all other types are single-word addresses. */
			aw_raw = ph->words[addr_idx];
			aw_base = flex_decode_addr_flags(aw_raw, &addr_is_group, &addr_is_temp);
			aw_type = flex_classify_addr_word(aw_base);
			grp_flag = flex_group_flag_char(addr_is_group, addr_is_temp);
			is_long = flex_addr_is_long(aw_base);

			if (is_long) {
				/* Long address: 2 address words, 2 vector words.
				 * Set detection + inverse formula.
				 * Group long addresses use only w1 (with group
				 * bit set), so they are single-word — the group
				 * bit extraction above already handled this. */
				if (addr_is_group) {
					/* Group address with long-range base:
					 * single word, decode base as short-style */
					capcode = flex_decode_short_address(aw_base);
					is_long = 0;

					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c addr[%d] %s[%c%c] aw=0x%05X(raw=0x%05X) → cap=%" PRIu64 ".\n",
						  phase_name, addr_idx,
						  flex_addr_type_name(aw_type),
						  flex_addr_type_flag(aw_type, 0),
						  grp_flag,
						  aw_base, aw_raw, capcode);

					if (ph->status[j] == FLEX_WORD_UNCORRECTABLE ||
					    ph->status[j] == FLEX_WORD_NOT_RECEIVED) {
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c vec[%d] uncorrectable.\n",
							  phase_name, j);
						vec_count++;
						continue;
					}
					vec_count++;
				} else {
					/* Individual long address: 2 words */
					if (addr_idx + 1 >= voffset) {
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: Phase %c long addr[%d] truncated.\n",
							  phase_name, addr_idx);
						break;
					}
					if (ph->status[addr_idx + 1] == FLEX_WORD_UNCORRECTABLE ||
					    ph->status[addr_idx + 1] == FLEX_WORD_NOT_RECEIVED) {
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c long addr[%d] word2 uncorrectable.\n",
							  phase_name, addr_idx);
						addr_idx++;
						vec_count += 2;
						continue;
					}
					uint32_t aw2 = ph->words[addr_idx + 1];
					capcode = flex_decode_long_address(aw_base, aw2);
					addr_idx++;

					if (capcode == 0) {
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  "RX: Phase %c long addr[%d,%d] aw=0x%05X,0x%05X invalid set.\n",
							  phase_name, addr_idx - 1, addr_idx, aw_base, aw2);
						vec_count += 2;
						continue;
					}

					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c long addr[%d,%d] %s[%c] aw=0x%05X(%s),0x%05X(%s) → cap=%" PRIu64 ".\n",
						  phase_name, addr_idx - 1, addr_idx,
						  flex_long_set_name(aw_base, aw2),
						  flex_addr_type_flag(aw_type, 1),
						  aw_base, flex_addr_type_name(aw_type),
						  aw2, flex_addr_type_name(flex_classify_addr_word(aw2)),
						  capcode);

					if (j + 1 >= FLEX_WORDS_PER_FRAME) break;
					if (ph->status[j] == FLEX_WORD_UNCORRECTABLE ||
					    ph->status[j] == FLEX_WORD_NOT_RECEIVED ||
					    ph->status[j + 1] == FLEX_WORD_UNCORRECTABLE ||
					    ph->status[j + 1] == FLEX_WORD_NOT_RECEIVED) {
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c long vec[%d,%d] uncorrectable.\n",
							  phase_name, j, j + 1);
						vec_count += 2;
						continue;
					}
					vec_count += 2;
				}
			} else {
				/* Single-word address.
				 * For group addresses, decode from the base word
				 * (flags already stripped). */
				capcode = flex_decode_short_address(aw_base);

				/* Enhanced logging for special address types */
				if (flex_addr_is_special(aw_base)) {
					if (aw_type == FLEX_ADDR_OPER_MSG) {
						LOGP_CHAN(DDSP, LOGL_INFO,
							  "RX: Phase %c addr[%d] %s/%s[%c] aw=0x%05X → cap=%" PRIu64 ".\n",
							  phase_name, addr_idx,
							  flex_addr_type_name(aw_type),
							  flex_oper_msg_subtype_name(aw_base),
							  flex_addr_type_flag(aw_type, 0),
							  aw_base, capcode);
					} else if (aw_type == FLEX_ADDR_TEMPORARY) {
						LOGP_CHAN(DDSP, LOGL_INFO,
							  "RX: Phase %c addr[%d] %s[%c] slot=%u aw=0x%05X → cap=%" PRIu64 " — %s.\n",
							  phase_name, addr_idx,
							  flex_addr_type_name(aw_type),
							  flex_addr_type_flag(aw_type, 0),
							  flex_temp_addr_slot(aw_base),
							  aw_base, capcode,
							  flex_special_addr_detail(aw_base));
					} else {
						LOGP_CHAN(DDSP, LOGL_INFO,
							  "RX: Phase %c addr[%d] %s[%c] aw=0x%05X → cap=%" PRIu64 " — %s.\n",
							  phase_name, addr_idx,
							  flex_addr_type_name(aw_type),
							  flex_addr_type_flag(aw_type, 0),
							  aw_base, capcode,
							  flex_special_addr_detail(aw_base));
					}
				} else if (addr_is_group) {
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c addr[%d] %s[%c%c] aw=0x%05X(raw=0x%05X) → cap=%" PRIu64 ".\n",
						  phase_name, addr_idx,
						  flex_addr_type_name(aw_type),
						  flex_addr_type_flag(aw_type, 0),
						  grp_flag,
						  aw_base, aw_raw, capcode);
				} else {
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c addr[%d] %s[%c] aw=0x%05X → cap=%" PRIu64 ".\n",
						  phase_name, addr_idx,
						  flex_addr_type_name(aw_type),
						  flex_addr_type_flag(aw_type, 0),
						  aw_base, capcode);
				}

				if (ph->status[j] == FLEX_WORD_UNCORRECTABLE ||
				    ph->status[j] == FLEX_WORD_NOT_RECEIVED) {
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c vec[%d] uncorrectable.\n",
						  phase_name, j);
					vec_count++;
					continue;
				}
				vec_count++;
			}

			/* Vector decode */
			{
				uint32_t viw = ph->words[j];
				vec_type = FLEX_VEC_TYPE(viw);
				mw1 = FLEX_VEC_START(viw);

				/* Numeric vectors
				 * (types 3, 4, 7) have a 3-bit n field (bits 14-16)
				 * and a 4-bit K checksum (bits 17-20).
				 * Alpha/hex/secure vectors have a 7-bit n field
				 * (bits 14-20). */
				if (vec_type == FLEX_VECTOR_TYPE_NUMERIC ||
				    vec_type == FLEX_VECTOR_TYPE_SPECIAL_NUM ||
				    vec_type == FLEX_VECTOR_TYPE_NUMBERED_NUM) {
					len = (viw >> FLEX_VEC_LEN_SHIFT) & FLEX_VEC_NUM_LEN_MASK;
				} else {
					len = FLEX_VEC_LEN(viw);
				}
				mw2 = mw1 + (len - 1);

				/* For long addresses, "the 1st word of the
				 * message is placed at the 2nd word of the
				 * vector" (Vy = body[0]).  The vector b field
				 * points to the Message Field start (body[1]),
				 * and n = total body words including body[0].
				 * Adjust mw2 down by 1 so the MF range covers
				 * only body[1..n-1].  Body[0] is read from Vy
				 * (ph->words[j+1]) in each message handler. */
				if (is_long && len > 0 &&
				    vec_type != FLEX_VECTOR_TYPE_SHORT_INSTR &&
				    vec_type != FLEX_VECTOR_TYPE_TONE) {
					mw2--;
				}

				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c cap=%" PRIu64 " type=%d mw1=%d len=%d%s%s.\n",
					  phase_name, capcode, vec_type, mw1, len,
					  is_long ? " (long)" : "",
					  addr_is_group ? (addr_is_temp ? " (tempgroup)" : " (group)") : "");
			}

			/* Short instruction vector.
			 * 14-bit instruction data encoded directly in the vector word.
			 * Types: 000=TempAddr assignment, 001=SysEvent notification,
			 * 010-111=reserved. */
			if (vec_type == FLEX_VECTOR_TYPE_SHORT_INSTR) {
				uint32_t viw = ph->words[j];
				uint32_t instr_data = FLEX_VEC_INSTR_DATA(viw);
				uint32_t itype = flex_instr_type(instr_data);

				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c short instruction: cap=%" PRIu64 " data=0x%04X type=%s(%u).\n",
					  phase_name, capcode, instr_data,
					  flex_instr_type_name(itype), itype);

				/* Temporary Address assignment:
				 * instruction type 000.
				 * bits 10-13 = a3-a0 (slot index 0-15)
				 * bits 3-9   = f6-f0 (target frame for group msg)
				 *
				 * Multiple capcodes can be assigned to the same
				 * slot — that's how temporary groups work.  Each
				 * pager's individual address + short instruction
				 * tells it "listen on temp slot X in frame Y." */
				if (itype == FLEX_INSTR_TYPE_TEMP_ADDR) {
					uint32_t slot = flex_instr_slot(instr_data);
					uint32_t tgt_frame = flex_instr_frame(instr_data);
					int phase_idx = (phase_name == 'A') ? 0 :
							(phase_name == 'B') ? 1 :
							(phase_name == 'C') ? 2 : 3;
					int cnt = flex->rx.temp_addr_map[phase_idx][slot].count;

					/* New SETUP on this slot: if slot was active
					 * from a previous assignment, overwrite it.
					 * The same temp address must not be reused
					 * until the previous group message ends —
					 * so a new SETUP implies the old one
					 * completed or was abandoned. */
					if (slot < FLEX_TEMP_ADDR_SLOTS) {
						if (!flex->rx.temp_addr_map[phase_idx][slot].active) {
							/* Fresh slot — initialize */
							flex->rx.temp_addr_map[phase_idx][slot].count = 0;
							flex->rx.temp_addr_map[phase_idx][slot].target_frame = tgt_frame;
							flex->rx.temp_addr_map[phase_idx][slot].setup_frame = flex->rx.fiw_frame;
							flex->rx.temp_addr_map[phase_idx][slot].setup_cycle = flex->rx.fiw_cycle;
							flex->rx.temp_addr_map[phase_idx][slot].active = 1;
							cnt = 0;
						}
						if (cnt < FLEX_TEMP_GROUP_MAX_MEMBERS) {
							flex->rx.temp_addr_map[phase_idx][slot].capcodes[cnt] = capcode;
							flex->rx.temp_addr_map[phase_idx][slot].count = cnt + 1;
						}
					}

					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  SETUP slot=%u target_frame=%u <- cap=%" PRIu64 " (member %d)\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_TEMPORARY,
						  slot, tgt_frame, capcode,
						  flex->rx.temp_addr_map[phase_idx][slot].count);
				} else if (aw_type == FLEX_ADDR_TEMPORARY) {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  INS temp_slot=%u type=%s data=0x%04X\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_TEMPORARY,
						  flex_temp_addr_slot(aw_base),
						  flex_instr_type_name(itype),
						  instr_data);
				} else if (aw_type == FLEX_ADDR_OPER_MSG &&
					   itype == FLEX_INSTR_TYPE_SYS_EVENT) {
					/* System Event Notification via short instruction
					 * on OperMsg address 0x1F781F.  Pre-alerts pagers
					 * about upcoming changes within 4 cycles.
					 * The 11-bit data field is infrastructure-defined. */
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  INS SysEvent pre-alert data=0x%04X (change within 4 cycles)\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_OPER_MSG,
						  instr_data);
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c%c%c INS type=%s data=0x%04X\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  flex_addr_type_flag(aw_type, is_long),
						  grp_flag,
						  prio_flag,
						  flex_instr_type_name(itype),
						  instr_data);
				}
				continue;
			}

			if (mw1 == 0 && mw2 == 0) continue;

			/* Network message payload parsing.
			 * Network addresses use Secure (type 0) vectors.
			 * First message word contains Service Area ID, Coverage
			 * Zone Count, and Traffic Management Flags. */
			if (aw_type == FLEX_ADDR_NETWORK &&
			    vec_type == FLEX_VECTOR_TYPE_SECURE &&
			    mw1 <= 87 && mw1 < FLEX_WORDS_PER_FRAME &&
			    (ph->status[mw1] == FLEX_WORD_CLEAN ||
			     ph->status[mw1] == FLEX_WORD_CORRECTED)) {
				uint32_t net_mw = ph->words[mw1];
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  NET area_id=%u zones=%u traffic=0x%02X%s\n",
					  bitrate,
					  flex->rx.fiw_cycle, flex->rx.fiw_frame,
					  phase_name, capcode,
					  FLEX_RX_FLAG_NETWORK,
					  flex_net_area_id(net_mw),
					  flex_net_coverage_zones(net_mw),
					  flex_net_traffic_flags(net_mw),
					  flex_net_is_overloaded(net_mw) ? " OVERLOAD" : "");
				continue;
			}

			/* Operator messaging payload parsing.
			 *
			 * System Messages can be transmitted in three ways:
			 *   (a) BIW101 only — vectors at end of VF, messages in MF.
			 *       No operator address needed; BIW A-type selects audience.
			 *   (b) BIW101 + Operator Messaging Address — both BIW101 and
			 *       operator address in AF, operator vector in VF, message in MF.
			 *       Allows system message + operator-specific content together.
			 *   (c) Operator Messaging Address only — no BIW101 needed.
			 *       Operator address in AF, vector in VF, message in MF.
			 *       Simplest path for operator-initiated system messages.
			 *
			 * Sub-type LSB (4 bits) selects function:
			 *   0x00-0x04: SysMsg (all/home/roaming/ssid/time)
			 *   0x0E: SSIDChange — TMF split or new coverage frequencies.
			 *         Mandatory on roaming frames, 5 cycles around change.
			 *   0x0F: SysEvent — pre-alert for changes within 4 cycles.
			 *         Uses Short Instruction Vector (type 001).
			 *
			 * FIFO: send via "sysmsg,<lsb>,,<payload>" command. */
			if (aw_type == FLEX_ADDR_OPER_MSG) {
				enum flex_oper_msg_category cat = flex_oper_msg_category(aw_base);
				const char *hint = "";
				if (cat == FLEX_OPER_CAT_SSID_CHANGE)
					hint = " (TMF split or new coverage frequencies)";
				else if (cat == FLEX_OPER_CAT_SYS_EVENT)
					hint = " (pre-alert: system change within 4 cycles)";
				if (mw1 <= 87 && mw1 < FLEX_WORDS_PER_FRAME &&
				    (ph->status[mw1] == FLEX_WORD_CLEAN ||
				     ph->status[mw1] == FLEX_WORD_CORRECTED)) {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  OPR %s/%s payload=0x%05X%s\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_OPER_MSG,
						  flex_oper_msg_subtype_name(aw_base),
						  flex_oper_msg_category_name(cat),
						  ph->words[mw1],
						  hint);
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  OPR %s/%s%s\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_OPER_MSG,
						  flex_oper_msg_subtype_name(aw_base),
						  flex_oper_msg_category_name(cat),
						  hint);
				}
				continue;
			}

			/* Info Service address (Spec Table 3.8.1-1, under study).
			 * Log raw payload for analysis — no defined protocol yet. */
			if (aw_type == FLEX_ADDR_INFO_SVC) {
				if (mw1 <= 87 && mw2 <= 87) {
					char hex[512];
					int hi = 0, w;
					for (w = mw1; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
						if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
						    ph->status[w] == FLEX_WORD_NOT_RECEIVED)
							continue;
						hi += snprintf(hex + hi, sizeof(hex) - hi,
							       "%05X ", ph->words[w]);
						if (hi >= (int)sizeof(hex) - 6)
							break;
					}
					if (hi > 0 && hex[hi - 1] == ' ')
						hex[--hi] = '\0';
					else
						hex[hi] = '\0';
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  ISV [%s]\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_INFO_SVC,
						  hex);
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  ISV\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_INFO_SVC);
				}
				continue;
			}

			/* Reserved Short address — log and skip (no defined behavior). */
			if (aw_type == FLEX_ADDR_RSVD_SHORT) {
				LOGP_CHAN(DDSP, LOGL_INFO,
					  "RX: Phase %c addr[%d] reserved short aw=0x%05X — ignoring.\n",
					  phase_name, addr_idx, aw_base);
				continue;
			}

			/* Temporary Address DELIVERY detection (§5.2).
			 *
			 * When a temp address (0x1F7800–0x1F780F) appears as an
			 * address word with a non-instruction vector, this is the
			 * group message delivery.  Log the member list from the
			 * tracking table, then determine if the message is complete.
			 *
			 * Per §5.2: "The Temporary Address is valid until the Group
			 * message ends."  And: "All dynamic group messages using
			 * temporary addresses are considered to be fragmented
			 * regardless of message length."
			 *
			 * Teardown: clear the slot when the message is complete:
			 *   - Alpha/Secure: C=0 in header (not continued)
			 *   - Tone/Numeric/HEX: always complete (single frame)
			 * If C=1 (continued), the slot stays active for the next
			 * frame's fragment. */
			if (aw_type == FLEX_ADDR_TEMPORARY) {
				uint32_t slot = flex_temp_addr_slot(aw_base);
				int phase_idx = (phase_name == 'A') ? 0 :
						(phase_name == 'B') ? 1 :
						(phase_name == 'C') ? 2 : 3;
				int is_complete = 1; /* assume complete unless alpha says otherwise */

				/* Check alpha header for continuation flag.
				 * For long addresses, body[0] (header) is at
				 * Vy (j+1); for short, it's at mw1. */
				if ((vec_type == FLEX_VECTOR_TYPE_ALPHA ||
				     vec_type == FLEX_VECTOR_TYPE_SECURE)) {
					int hi = is_long ? (j + 1) : mw1;
					if (hi <= 87 && hi < FLEX_WORDS_PER_FRAME &&
					    (ph->status[hi] == FLEX_WORD_CLEAN ||
					     ph->status[hi] == FLEX_WORD_CORRECTED)) {
						uint32_t hdr = ph->words[hi];
						int hdr_c = (hdr & FLEX_ALPHA_HDR_C_MASK) >> FLEX_ALPHA_HDR_C_SHIFT;
						if (hdr_c)
							is_complete = 0; /* continued — keep slot active */
					}
				}

				/* Log DELIVERY with member list */
				if (slot < FLEX_TEMP_ADDR_SLOTS &&
				    flex->rx.temp_addr_map[phase_idx][slot].active) {
					int cnt = flex->rx.temp_addr_map[phase_idx][slot].count;
					int m;
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  DELIVERY slot=%u members=%d%s:",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_TEMPORARY,
						  slot, cnt,
						  is_complete ? " COMPLETE" : " CONTINUED");
					for (m = 0; m < cnt; m++)
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  " %" PRIu64,
							  flex->rx.temp_addr_map[phase_idx][slot].capcodes[m]);
					LOGP_CHAN(DDSP, LOGL_NOTICE, "\n");

					/* Teardown on completion */
					if (is_complete) {
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c temp slot=%u TEARDOWN — message complete.\n",
							  phase_name, slot);
						memset(&flex->rx.temp_addr_map[phase_idx][slot], 0,
						       sizeof(flex->rx.temp_addr_map[phase_idx][slot]));
					}
				} else {
					/* No tracked SETUP for this slot — log anyway */
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c [%09" PRIu64 "] %c  DELIVERY slot=%u (no tracked SETUP)%s\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name, capcode,
						  FLEX_RX_FLAG_TEMPORARY,
						  slot,
						  is_complete ? " COMPLETE" : " CONTINUED");
				}
				/* Fall through to normal message decode below */
			}

			/* Alpha message */
			if (vec_type == FLEX_VECTOR_TYPE_ALPHA || vec_type == FLEX_VECTOR_TYPE_SECURE) {
				if (mw1 > 87 || mw2 > 87) continue;

				/* Decode header word flags (frame.h has layout).
				 * Per Sections 3.9.3/3.9.4: for long addresses,
				 * body[0] (header) is at Vy (j+1), and mw1
				 * points to body[1] in the Message Field.
				 * For short addresses, body[0] is at mw1. */
				int hdr_k = 0, hdr_c = 0, hdr_f = 0;
				int hdr_n = 0, hdr_r = 0, hdr_m = 0;
				int hdr_u = 0, hdr_v = 0;
				/* Secure message type field (Fig. 3.10.1.4-1):
				 *   t1t0=00: 7-bit alphanumeric (JIS X 0201)
				 *   t1t0=10: binary data
				 *   t1t0=01: data defined separately
				 *   t1t0=11: reserved
				 * Present on ALL secure fragments (bits 19-20). */
				int sec_t = -1; /* -1 = not secure */
				const char *msg_tag = "ALN"; /* default for alpha */
				char frag_flag = '?';
				int hdr_idx = is_long ? (j + 1) : mw1;
				if (hdr_idx < FLEX_WORDS_PER_FRAME &&
				    (ph->status[hdr_idx] == FLEX_WORD_CLEAN ||
				     ph->status[hdr_idx] == FLEX_WORD_CORRECTED)) {
					uint32_t hdr = ph->words[hdr_idx];
					hdr_k = hdr & FLEX_ALPHA_HDR_K_MASK;
					hdr_c = (hdr & FLEX_ALPHA_HDR_C_MASK) >> FLEX_ALPHA_HDR_C_SHIFT;
					hdr_f = (hdr & FLEX_ALPHA_HDR_F_MASK) >> FLEX_ALPHA_HDR_F_SHIFT;
					hdr_n = (hdr & FLEX_ALPHA_HDR_N_MASK) >> FLEX_ALPHA_HDR_N_SHIFT;

					if (hdr_c == 0 && hdr_f == 3) frag_flag = 'K';
					else if (hdr_c == 0) frag_flag = 'C';
					else frag_flag = 'F';

					/* Bits 19-20 differ by vector type and fragment:
					 *
					 * Secure (V=000, Fig. 3.10.1.4-1):
					 *   bit 19 = t0, bit 20 = t1 (ALL fragments)
					 *   t1t0: 00=alpha, 10=binary, 01=separate, 11=rsvd
					 *
					 * Alpha (V=101):
					 *   First fragment (F=11, Fig. 3.10.1.3-1):
					 *     bit 19 = R (retrieval), bit 20 = M (mail drop)
					 *   Continuation/final (Fig. 3.10.1.3-2):
					 *     bit 19 = U₀, bit 20 = V₀ (reserved) */
					if (vec_type == FLEX_VECTOR_TYPE_SECURE) {
						sec_t = (hdr >> FLEX_SEC_TYPE_SHIFT) & FLEX_SEC_TYPE_MASK;
						msg_tag = "SEC";
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c secure hdr[%d]=0x%05X: "
							  "K=0x%03X C=%d F=%d N=%d t=%d (%s)\n",
							  phase_name, hdr_idx, hdr,
							  hdr_k, hdr_c, hdr_f, hdr_n, sec_t,
							  sec_t == FLEX_SEC_TYPE_ALPHA ? "alpha/JIS" :
							  sec_t == FLEX_SEC_TYPE_BINARY ? "binary" :
							  sec_t == FLEX_SEC_TYPE_SEPARATE ? "separate" :
							  "reserved");
					} else if (hdr_f == 3) {
						hdr_r = (hdr & FLEX_ALPHA_HDR_R_MASK) >> FLEX_ALPHA_HDR_R_SHIFT;
						hdr_m = (hdr & FLEX_ALPHA_HDR_M_MASK) >> FLEX_ALPHA_HDR_M_SHIFT;
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c alpha hdr[%d]=0x%05X: "
							  "K=0x%03X C=%d F=%d "
							  "N=%d R=%d M=%d "
							  "(%s)\n",
							  phase_name, hdr_idx, hdr,
							  hdr_k, hdr_c, hdr_f,
							  hdr_n, hdr_r, hdr_m,
							  (frag_flag == 'K') ? "complete" :
							  (frag_flag == 'F') ? "continuation" :
							  "final");
					} else {
						hdr_u = (hdr & FLEX_ALPHA_HDR_U_MASK) >> FLEX_ALPHA_HDR_U_SHIFT;
						hdr_v = (hdr & FLEX_ALPHA_HDR_V_MASK) >> FLEX_ALPHA_HDR_V_SHIFT;
						/* TODO: Enhanced Fragmentation (ARIB STD-43A
						 * §3.10.1.3, Fig. 3.10.1.3-2) — when
						 * U₀V₀ ≠ 00, handle SI/SO character mode
						 * switching during character extraction:
						 *   10 = starts in default char mode
						 *   11 = starts in alternative char mode
						 *   01 = reserved (2nd alternative mode)
						 * Also: pad chars are NUL not ETX, and
						 * signature excludes ETX/NUL.  See frame.h
						 * FLEX_ALPHA_HDR_U/V defines for full spec
						 * reference. */
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c alpha hdr[%d]=0x%05X: "
							  "K=0x%03X C=%d F=%d "
							  "N=%d U=%d V=%d "
							  "(%s)\n",
							  phase_name, hdr_idx, hdr,
							  hdr_k, hdr_c, hdr_f,
							  hdr_n, hdr_u, hdr_v,
							  (frag_flag == 'F') ? "continuation" :
							  "final");
					}
				}

				/* Extract 7-bit alphanumeric characters and
				 * verify the message signature.
				 *
				 * First data word after header on initial
				 * fragment: bits 0-6 = Signature (S), then
				 * characters at bits 7-13 and 14-20.
				 * On continuation fragments, all three 7-bit
				 * slots are message characters.
				 *
				 * ETX ($03) is the fill/termination character
				 * per ARIB STD-43A Section 3.8.8.3.
				 *
				 * TODO: Enhanced Fragmentation (ARIB STD-43A
				 * §3.10.1.3) — when U₀V₀ ≠ 00 on continuation
				 * fragments, the character extraction must:
				 *   (1) Track current char mode (default vs.
				 *       alternative) starting from U₀V₀ value.
				 *   (2) Interpret SI ($0F) as switch to default
				 *       mode, SO ($0E) as switch to alternative.
				 *   (3) Strip NUL ($00) fill chars (not ETX).
				 *   (4) Exclude ETX and NUL from signature sum.
				 * See frame.h FLEX_ALPHA_HDR_U/V for full spec
				 * reference and U₀V₀ value table. */
				{
					char text[512];
					int ti = 0, w;
					/* For short addresses: mw1 = body[0] (header),
					 * data starts at mw1+1.
					 * For long addresses: header is at Vy (j+1),
					 * mw1 = body[1] in MF, data starts at mw1. */
					int start_word = is_long ? mw1 : (mw1 + 1);
					uint32_t rx_sig = 0;
					uint32_t sig_sum = 0;
					int is_initial = (hdr_f == 3); /* F=11 = first/only */
					const char *alpha_sig_status = ""; /* signature validation result */

					/* Kanji mode: 16-bit Shift-JIS extraction
					 * (1 char per word, bits 0-15) instead of
					 * 7-bit ASCII (3 chars per word).
					 * Only applies to alpha (V=101), not secure.
					 *
					 * For initial fragments: use global flag.
					 * For continuation/final: use the reassembly
					 * slot's kanji flag for consistency (Req 15.6),
					 * so all fragments in a stream use the same
					 * extraction mode even if the flag changes. */
					int is_kanji = 0;
					if (vec_type == FLEX_VECTOR_TYPE_ALPHA) {
						if (is_initial) {
							is_kanji = rx_kanji_enabled;
						} else {
							/* Check reassembly slot for this stream */
							int kslot = reasm_find(flex, capcode, hdr_n);
							if (kslot >= 0)
								is_kanji = flex->rx.reasm[kslot].kanji;
							else
								is_kanji = rx_kanji_enabled;
						}
					}
					if (is_kanji)
						msg_tag = "ALN:KNJ";

					if (is_kanji) {
						/* Kanji/Shift-JIS: extract 16-bit chars,
						 * 1 per word (bits 0-15).  Output as
						 * 2-byte big-endian per character.
						 * Signature field handling is the same:
						 * first word on initial fragment has sig
						 * in bits 0-6, but in kanji mode we skip
						 * the signature word entirely for char
						 * extraction (it doesn't carry a valid
						 * 16-bit character). */
						for (w = start_word; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
							uint16_t ch16;

							if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
							    ph->status[w] == FLEX_WORD_NOT_RECEIVED)
								continue;

							if (w == start_word && is_initial) {
								/* Signature word — extract sig,
								 * skip for character data */
								rx_sig = ph->words[w] & FLEX_ALPHA_SIG_MASK;
								continue;
							}

							ch16 = ph->words[w] & 0xFFFF;
							if (ch16 == 0)
								continue; /* skip NUL padding */
							/* Output as 2-byte big-endian */
							if (ti < (int)sizeof(text) - 2) {
								text[ti++] = (ch16 >> 8) & 0xFF;
								text[ti++] = ch16 & 0xFF;
							}
						}
					} else {
						/* Standard 7-bit ASCII extraction:
						 * 3 characters per word */
						for (w = start_word; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
							uint32_t dw;
							unsigned char ch;

							if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
							    ph->status[w] == FLEX_WORD_NOT_RECEIVED)
								continue;
							dw = ph->words[w];

							/* First slot: signature on initial
							 * fragment, character otherwise */
							if (w == start_word && is_initial) {
								rx_sig = dw & FLEX_ALPHA_SIG_MASK;
							} else {
								ch = (dw >> FLEX_ALPHA_CHAR1_SHIFT) & FLEX_ALPHA_CHAR_MASK;
								sig_sum += ch;
								if (ch != FLEX_ALPHA_ETX && ch != '\0' && ti < (int)sizeof(text) - 1)
									text[ti++] = ch;
							}
							ch = (dw >> FLEX_ALPHA_CHAR2_SHIFT) & FLEX_ALPHA_CHAR_MASK;
							sig_sum += ch;
							if (ch != FLEX_ALPHA_ETX && ch != '\0' && ti < (int)sizeof(text) - 1)
								text[ti++] = ch;
							ch = (dw >> FLEX_ALPHA_CHAR3_SHIFT) & FLEX_ALPHA_CHAR_MASK;
							sig_sum += ch;
							if (ch != FLEX_ALPHA_ETX && ch != '\0' && ti < (int)sizeof(text) - 1)
								text[ti++] = ch;
						}
					}
					text[ti] = '\0';

					/* S: Verify 7-bit message signature on initial fragment.
					 *
					 * Per Spec §3.8.8.3 / §3.10.1.3: "Signature is defined
					 * as the 1's complement of binary sum for the entire
					 * message (including all fragments) for every 7 bits,
					 * starting from the first 7 bits that directly follow
					 * the Signature Field.  The 7 LSB's of the result is
					 * transmitted as the Message Signature."
					 *
					 * Standard Fragmentation (U₀,V₀ = 0,0):
					 *   ETX padding IS included in the sum (TX/RX agree).
					 * Enhanced Fragmentation (U₀,V₀ ≠ 0,0 — not impl.):
					 *   ETX ($03) and NUL ($00) are NOT included.
					 *
					 * TODO: Enhanced Fragmentation — when U₀V₀ ≠ 00,
					 * the sig_sum loop above must skip characters that
					 * are ETX ($03) or NUL ($00).  See frame.h
					 * FLEX_ALPHA_HDR_U/V for full spec reference.
					 *
					 * For a complete (single-fragment) message, we validate
					 * immediately.  For multi-fragment messages, the
					 * signature covers ALL fragments — validation must
					 * wait until reassembly is complete. */
					if (is_initial) {
						uint32_t expected_sig = (~sig_sum) & FLEX_ALPHA_SIG_MASK;
						if (hdr_c == 0) {
							/* Complete message — validate now */
							if (rx_sig == expected_sig) {
								alpha_sig_status = ",sig=OK";
								LOGP_CHAN(DDSP, LOGL_DEBUG,
									  "RX: Phase %c alpha signature OK "
									  "(S=0x%02X, sum=0x%02X)\n",
									  phase_name, rx_sig,
									  sig_sum & 0x7F);
							} else {
								alpha_sig_status = ",sig=FAIL";
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: Phase %c alpha signature MISMATCH: "
									  "received=0x%02X computed=0x%02X "
									  "(sum=0x%02X)\n",
									  phase_name, rx_sig, expected_sig,
									  sig_sum & 0x7F);
							}
						} else {
							/* First fragment of multi-fragment message.
							 * Signature covers all fragments — log
							 * partial sum for debugging, defer final
							 * validation to reassembly completion. */
							alpha_sig_status = ",sig=partial";
							LOGP_CHAN(DDSP, LOGL_DEBUG,
								  "RX: Phase %c alpha signature deferred "
								  "(multi-fragment): S=0x%02X, partial_sum=0x%02X\n",
								  phase_name, rx_sig,
								  sig_sum & 0x7F);
						}
					}

					/* K: Verify 10-bit fragment checksum (Spec §3.10.1.3).
					 *
					 * Recompute K over all message words in this fragment
					 * using the 3-group method: bits 0-7, 8-15, 16-20.
					 * The header word (mw1) participates with K₀-K₉ = 0.
					 * Compare 1's complement of lower 10 bits against
					 * the received K value.
					 *
					 * Unlike S (which spans all fragments), K is per-
					 * fragment and can always be validated immediately. */
					const char *alpha_k_status = "";
					/* For long addresses, body[0] (header) is at
					 * Vy (hdr_idx), not mw1.  K checksum covers
					 * all body words including body[0]. */
					if (ph->status[hdr_idx] == FLEX_WORD_CLEAN ||
					    ph->status[hdr_idx] == FLEX_WORD_CORRECTED) {
						uint32_t k_sum = 0;
						int all_words_ok = 1;

						/* Include body[0] (header) from Vy for
						 * long addresses, or from mw1 for short */
						if (is_long) {
							uint32_t dw = ph->words[hdr_idx];
							dw &= ~FLEX_ALPHA_HDR_K_MASK;
							k_sum += dw & FLEX_ALPHA_K_GRP1_MASK;
							k_sum += (dw >> FLEX_ALPHA_K_GRP2_SHIFT) & FLEX_ALPHA_K_GRP2_MASK;
							k_sum += (dw >> FLEX_ALPHA_K_GRP3_SHIFT) & FLEX_ALPHA_K_GRP3_MASK;
						}

						/* Sum remaining body words from MF.
						 * Short: mw1=body[0] (header), mw1+1..mw2=data
						 * Long:  mw1=body[1], mw1..mw2=data */
						{
							int k_start = is_long ? mw1 : mw1;
							for (w = k_start; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
								uint32_t dw;
								if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
								    ph->status[w] == FLEX_WORD_NOT_RECEIVED) {
									all_words_ok = 0;
									break;
								}
								dw = ph->words[w];
								if (!is_long && w == mw1) {
									/* Short addr: header at mw1,
									 * zero K field before summing */
									dw &= ~FLEX_ALPHA_HDR_K_MASK;
								}
								k_sum += dw & FLEX_ALPHA_K_GRP1_MASK;
								k_sum += (dw >> FLEX_ALPHA_K_GRP2_SHIFT) & FLEX_ALPHA_K_GRP2_MASK;
								k_sum += (dw >> FLEX_ALPHA_K_GRP3_SHIFT) & FLEX_ALPHA_K_GRP3_MASK;
							}
						}

						if (all_words_ok) {
							uint32_t expected_k = (~k_sum) & FLEX_ALPHA_HDR_K_MASK;
							if ((uint32_t)hdr_k == expected_k) {
								alpha_k_status = ",K=OK";
								LOGP_CHAN(DDSP, LOGL_DEBUG,
									  "RX: Phase %c alpha K checksum OK "
									  "(K=0x%03X)\n",
									  phase_name, hdr_k);
							} else {
								alpha_k_status = ",K=FAIL";
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: Phase %c alpha K checksum MISMATCH: "
									  "received=0x%03X computed=0x%03X\n",
									  phase_name, hdr_k, expected_k);
							}
						} else {
							alpha_k_status = ",K=incomplete";
						}
					}

					/* Fragment reassembly (Spec §3.10.1.3 / §4.2).
					 *
					 * frag_flag: 'K' = complete (F=11, C=0)
					 *            'F' = continuation (C=1)
					 *            'C' = final fragment (C=0, F≠11)
					 *
					 * Every fragment is always output at NOTICE level.
					 * Additionally, fragments are buffered for reassembly.
					 * When the final fragment arrives, the full reassembled
					 * message is output as a separate "reassembled" line. */

					/* Always output this fragment independently */
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s,frag=%s%s%s [%09" PRIu64 "] %c%c%c %s \"%s\"\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name,
						  flex->rx.sync_baud,
						  flex->rx.sync_levels,
						  flex->rx.polarity ? "neg" : "pos",
						  (frag_flag == 'K') ? "complete" :
						  (frag_flag == 'F') ? (is_initial ? "frag_start" : "frag_cont") :
						  "frag_end",
						  alpha_sig_status,
						  alpha_k_status,
						  capcode,
						  flex_addr_type_flag(aw_type, is_long),
						  grp_flag,
						  prio_flag,
						  msg_tag,
						  text);

					/* Reassembly logic for fragmented messages.
					 *
					 * F sequence per spec (§3.10.1.2/3):
					 *   11(initial), 00, 01, 10, 00, 01, 10, ...
					 * We validate expected_f on continuation/final
					 * fragments to detect missing or out-of-order
					 * fragments.  On mismatch we log a warning but
					 * still append (best-effort reassembly). */
					if (frag_flag == 'F' && is_initial) {
						/* First fragment (F=11, C=1) — start reassembly */
						int reasm_type = (vec_type == FLEX_VECTOR_TYPE_SECURE)
							? FLEX_VECTOR_TYPE_SECURE : FLEX_VECTOR_TYPE_ALPHA;
						int slot = reasm_alloc(flex, capcode, hdr_n, reasm_type);
						if (vec_type == FLEX_VECTOR_TYPE_SECURE)
							flex->rx.reasm[slot].secure_subtype = sec_t;
						if (is_kanji)
							flex->rx.reasm[slot].kanji = 1;
						reasm_append(flex, slot, text, ti);
					} else if (frag_flag == 'F') {
						/* Continuation fragment (C=1, F≠11) — append */
						int slot = reasm_find(flex, capcode, hdr_n);
						if (slot >= 0) {
							if (hdr_f != flex->rx.reasm[slot].expected_f)
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: Phase %c reassembly F mismatch for [%09" PRIu64 "] msgnum=%d: expected=%d got=%d (missing fragment?)\n",
									  phase_name, capcode, hdr_n,
									  flex->rx.reasm[slot].expected_f, hdr_f);
							reasm_append(flex, slot, text, ti);
							flex->rx.reasm[slot].expected_f = (hdr_f + 1) % 3;
						}
					} else if (frag_flag == 'C') {
						/* Final fragment (C=0, F≠11) — append and emit reassembled */
						int slot = reasm_find(flex, capcode, hdr_n);
						if (slot >= 0) {
							const char *reasm_tag;
							if (flex->rx.reasm[slot].kanji)
								reasm_tag = "ALN:KNJ";
							else if (flex->rx.reasm[slot].msg_type == FLEX_VECTOR_TYPE_SECURE)
								reasm_tag = "SEC";
							else
								reasm_tag = msg_tag;
							if (hdr_f != flex->rx.reasm[slot].expected_f)
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: Phase %c reassembly F mismatch for [%09" PRIu64 "] msgnum=%d: expected=%d got=%d (missing fragment?)\n",
									  phase_name, capcode, hdr_n,
									  flex->rx.reasm[slot].expected_f, hdr_f);
							reasm_append(flex, slot, text, ti);
							if (flex->rx.reasm[slot].msg_type == FLEX_VECTOR_TYPE_SECURE)
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s,frag=%s [%09" PRIu64 "] %c%c%c %s t1t0=%d \"%s\"\n",
									  bitrate,
									  flex->rx.fiw_cycle, flex->rx.fiw_frame,
									  phase_name,
									  flex->rx.sync_baud,
									  flex->rx.sync_levels,
									  flex->rx.polarity ? "neg" : "pos",
									  "reassembled",
									  capcode,
									  flex_addr_type_flag(aw_type, is_long),
									  grp_flag,
									  prio_flag,
									  reasm_tag,
									  flex->rx.reasm[slot].secure_subtype,
									  flex->rx.reasm[slot].buf);
							else
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s,frag=%s [%09" PRIu64 "] %c%c%c %s \"%s\"\n",
									  bitrate,
									  flex->rx.fiw_cycle, flex->rx.fiw_frame,
									  phase_name,
									  flex->rx.sync_baud,
									  flex->rx.sync_levels,
									  flex->rx.polarity ? "neg" : "pos",
									  "reassembled",
									  capcode,
									  flex_addr_type_flag(aw_type, is_long),
									  grp_flag,
									  prio_flag,
									  reasm_tag,
									  flex->rx.reasm[slot].buf);
							flex->rx.reasm[slot].active = 0;
						}
					}
					/* frag_flag == 'K': complete message, no reassembly needed */
				}
			} else if (vec_type == FLEX_VECTOR_TYPE_NUMERIC ||
				   vec_type == FLEX_VECTOR_TYPE_SPECIAL_NUM ||
				   vec_type == FLEX_VECTOR_TYPE_NUMBERED_NUM) {
				/* Numeric message decode (Spec Section 3.10.1.1, Table 3.10-1).
				 *
				 * Message types per vector V field:
				 *   V=011: Standard Numeric — 4-bit BCD, 5 chars/word
				 *   V=100: Special Format Numeric — same encoding,
				 *          display format per pager ID-ROM
				 *   V=111: Numbered Numeric — 10-bit header (N5-N0
				 *          message number + R retrieval + S special
				 *          format + 2 reserved), then BCD digits
				 *
				 * Word layout (Fig. 3.10.1.1.1-1, Standard/Special):
				 *   1st word: K5 K4 a0 a1 a2 a3 b0 b1 b2 b3 c0...
				 *   Bits 1-2 = K5,K4 (checksum overflow)
				 *   Bits 3+ = BCD nibbles packed LSB-first
				 *
				 * Numbered format (Fig. 3.10.1.1.2-1):
				 *   1st word: K5 K4 N0..N5 R0 S0 a0 a1 a2 a3...
				 *   N = message number (0-63, displayed as N+1)
				 *   R = retrieval flag (1=check sequence, 0=skip)
				 *   S = special format (1=ID-ROM display format)
				 *
				 * BCD table (Table 3.10.2.1-1):
				 *   0-9=digits, A=spare, B=U(urgency),
				 *   C=space, D=hyphen, E=], F=[ */
				if (mw1 <= 87 && mw2 <= 87) {
					static const char bcd_table[16] = FLEX_NUM_BCD_TABLE;
					char digits[256];
					int di = 0, w, bit_count;
					int skip_bits;
					const char *num_tag;

					/* Numbered numeric header extraction
					 * (Section 3.10.1.1.2, Fig. 3.10.1.1.2-1) */
					int num_n = -1, num_r = 0, num_s = 0;

					if (vec_type == FLEX_VECTOR_TYPE_NUMBERED_NUM)
						skip_bits = FLEX_NUM_NUMBERED_SKIP_BITS;
					else
						skip_bits = FLEX_NUM_OVERHEAD_BITS;

					/* Extract N/R/S from numbered numeric header.
					 * Header is in body[0]: for long addr at Vy (j+1),
					 * for short addr at mw1. */
					if (vec_type == FLEX_VECTOR_TYPE_NUMBERED_NUM) {
						int num_hdr_idx = is_long ? (j + 1) : mw1;
						if (num_hdr_idx < FLEX_WORDS_PER_FRAME &&
						    (ph->status[num_hdr_idx] == FLEX_WORD_CLEAN ||
						     ph->status[num_hdr_idx] == FLEX_WORD_CORRECTED)) {
							uint32_t nhdr = ph->words[num_hdr_idx];
							/* bits 2-7 = N5-N0 (after K5K4 at bits 0-1) */
							num_n = (nhdr >> 2) & 0x3F;
							num_r = (nhdr >> 8) & 1;
							num_s = (nhdr >> 9) & 1;
							LOGP_CHAN(DDSP, LOGL_DEBUG,
								  "RX: Phase %c V=111 hdr[%d]=0x%05X: "
								  "N=%d R=%d S=%d\n",
								  phase_name, num_hdr_idx, nhdr,
								  num_n, num_r, num_s);
						}
					}

					/* Subtype tag for output:
					 *   NUM   = standard numeric (V=011)
					 *   SNUM  = special format numeric (V=100)
					 *   NNUM  = numbered numeric (V=111 S=0, like NUM with sequencing)
					 *   NSNUM = numbered special (V=111 S=1, like SNUM with sequencing) */
					num_tag =
						(vec_type == FLEX_VECTOR_TYPE_SPECIAL_NUM) ? "SNUM" :
						(vec_type == FLEX_VECTOR_TYPE_NUMBERED_NUM) ?
							(num_s ? "NSNUM" : "NNUM") : "NUM";

					bit_count = 0;
					uint8_t nibble_acc = 0;

					/* For long addresses, body[0] is at Vy (j+1)
					 * per Section 3.9.1. Process it first. */
					if (is_long && j + 1 < FLEX_WORDS_PER_FRAME &&
					    ph->status[j + 1] != FLEX_WORD_UNCORRECTABLE &&
					    ph->status[j + 1] != FLEX_WORD_NOT_RECEIVED) {
						uint32_t dw = ph->words[j + 1];
						int b;
						for (b = 0; b < FLEX_BCH_DATA_BITS; b++) {
							if (bit_count < skip_bits) {
								bit_count++;
								continue;
							}
							{
								int nibble_pos = (bit_count - skip_bits) % 4;
								if (nibble_pos == 0)
									nibble_acc = 0;
								nibble_acc |= ((dw >> b) & 1) << nibble_pos;
								if (nibble_pos == 3 && di < (int)sizeof(digits) - 1)
									digits[di++] = bcd_table[nibble_acc & 0x0F];
							}
							bit_count++;
						}
					}

					/* Process remaining body words from MF */
					for (w = mw1; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
						int b;
						uint32_t dw;

						if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
						    ph->status[w] == FLEX_WORD_NOT_RECEIVED)
							break;
						dw = ph->words[w];

						for (b = 0; b < FLEX_BCH_DATA_BITS; b++) {
							if (bit_count < skip_bits) {
								bit_count++;
								continue;
							}
							{
								int nibble_pos = (bit_count - skip_bits) % 4;
								if (nibble_pos == 0)
									nibble_acc = 0;
								nibble_acc |= ((dw >> b) & 1) << nibble_pos;
								if (nibble_pos == 3 && di < (int)sizeof(digits) - 1)
									digits[di++] = bcd_table[nibble_acc & 0x0F];
							}
							bit_count++;
						}
					}
					digits[di] = '\0';

					/* K checksum verification (Section 3.10.1.1.1).
					 * K3-K0 from vector word, K5K4 from msg word 1 bits 0-1.
					 * Recompute: sum 3 groups per word (bits 1-8, 9-16, 17-21),
					 * fold 2 MSB into 6 LSB, 1's complement. */
					{
						uint32_t viw = ph->words[j];
						uint32_t vec_k30 = (viw >> FLEX_VEC_NUM_KBIT_SHIFT) & FLEX_VEC_NUM_KBIT_MASK;
						int body0_idx = is_long ? (j + 1) : mw1;
						uint32_t k54 = 0;
						const char *num_k_status = "";

						if (body0_idx < FLEX_WORDS_PER_FRAME &&
						    (ph->status[body0_idx] == FLEX_WORD_CLEAN ||
						     ph->status[body0_idx] == FLEX_WORD_CORRECTED)) {
							k54 = ph->words[body0_idx] & 0x3;
						}
						uint32_t rx_k = (k54 << 4) | vec_k30;

						/* Recompute K over all body words */
						uint32_t k_sum = 0;
						int all_ok = 1;
						if (is_long && body0_idx < FLEX_WORDS_PER_FRAME &&
						    (ph->status[body0_idx] == FLEX_WORD_CLEAN ||
						     ph->status[body0_idx] == FLEX_WORD_CORRECTED)) {
							uint32_t dw = ph->words[body0_idx] & ~0x3U; /* zero K5K4 */
							k_sum += dw & 0xFF;
							k_sum += (dw >> 8) & 0xFF;
							k_sum += (dw >> 16) & 0x1F;
						}
						for (w = mw1; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
							uint32_t dw;
							if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
							    ph->status[w] == FLEX_WORD_NOT_RECEIVED) {
								all_ok = 0;
								break;
							}
							dw = ph->words[w];
							if (!is_long && w == mw1)
								dw &= ~0x3U; /* zero K5K4 */
							k_sum += dw & 0xFF;
							k_sum += (dw >> 8) & 0xFF;
							k_sum += (dw >> 16) & 0x1F;
						}

						if (all_ok) {
							k_sum &= 0xFF;
							k_sum = (k_sum & 0x3F) + (k_sum >> 6);
							uint32_t expected_k = (~k_sum) & 0x3F;
							if (rx_k == expected_k) {
								num_k_status = ",K=OK";
								LOGP_CHAN(DDSP, LOGL_DEBUG,
									  "RX: Phase %c numeric K OK (K=0x%02X)\n",
									  phase_name, rx_k);
							} else {
								num_k_status = ",K=FAIL";
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: Phase %c numeric K MISMATCH: "
									  "received=0x%02X computed=0x%02X\n",
									  phase_name, rx_k, expected_k);
							}
						} else {
							num_k_status = ",K=incomplete";
						}

						/* Output with subtype tag and optional sequencing fields */
						if (vec_type == FLEX_VECTOR_TYPE_NUMBERED_NUM && num_n >= 0) {
							LOGP_CHAN(DDSP, LOGL_NOTICE,
								  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s%s [%09" PRIu64 "] %c%c%c %s N=%d,R=%d,S=%d \"%s\"\n",
								  bitrate,
								  flex->rx.fiw_cycle, flex->rx.fiw_frame,
								  phase_name,
								  flex->rx.sync_baud,
								  flex->rx.sync_levels,
								  flex->rx.polarity ? "neg" : "pos",
								  num_k_status,
								  capcode,
								  flex_addr_type_flag(aw_type, is_long),
								  grp_flag,
								  prio_flag,
								  num_tag, num_n, num_r, num_s,
								  digits);
						} else {
							LOGP_CHAN(DDSP, LOGL_NOTICE,
								  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s%s [%09" PRIu64 "] %c%c%c %s \"%s\"\n",
								  bitrate,
								  flex->rx.fiw_cycle, flex->rx.fiw_frame,
								  phase_name,
								  flex->rx.sync_baud,
								  flex->rx.sync_levels,
								  flex->rx.polarity ? "neg" : "pos",
								  num_k_status,
								  capcode,
								  flex_addr_type_flag(aw_type, is_long),
								  grp_flag,
								  prio_flag,
								  num_tag,
								  digits);
						}
					}
				} else {
					/* V=111 without decoded header: can't determine S,
					 * fall back to NNUM */
					const char *num_tag =
						(vec_type == FLEX_VECTOR_TYPE_SPECIAL_NUM) ? "SNUM" :
						(vec_type == FLEX_VECTOR_TYPE_NUMBERED_NUM) ? "NNUM" : "NUM";
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s [%09" PRIu64 "] %c%c%c %s\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name,
						  flex->rx.sync_baud,
						  flex->rx.sync_levels,
						  flex->rx.polarity ? "neg" : "pos",
						  capcode,
						  flex_addr_type_flag(aw_type, is_long),
						  grp_flag,
						  prio_flag,
						  num_tag);
				}
			} else if (vec_type == FLEX_VECTOR_TYPE_TONE) {
				/* Tone-only / Short message (Section 8.7).
				 * Bits 9-15 of the vector word carry a 7-bit
				 * short message index.  Index 0 = pure tone-only
				 * alert; index 1-127 = predefined short message
				 * stored in the pager's ID-ROM. */
				uint32_t viw = ph->words[j];
				uint32_t smsg_idx = (viw >> 9) & 0x7F;
				if (smsg_idx == 0) {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s [%09" PRIu64 "] %c%c%c TON\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name,
						  flex->rx.sync_baud,
						  flex->rx.sync_levels,
						  flex->rx.polarity ? "neg" : "pos",
						  capcode,
						  flex_addr_type_flag(aw_type, is_long),
						  grp_flag,
						  prio_flag);
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s [%09" PRIu64 "] %c%c%c SMSG idx=%u\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name,
						  flex->rx.sync_baud,
						  flex->rx.sync_levels,
						  flex->rx.polarity ? "neg" : "pos",
						  capcode,
						  flex_addr_type_flag(aw_type, is_long),
						  grp_flag,
						  prio_flag,
						  smsg_idx);
				}
			} else if (vec_type == FLEX_VECTOR_TYPE_HEX_BINARY) {
				/* HEX/Binary message (Spec §3.10.1.2).
				 *
				 * First message word = header: K(12), C(1), F(2), N(6).
				 * First fragment also has a 2nd header word with
				 * R, M, D, H, B, I, s, S fields.
				 * Data starts at 3rd word (first frag) or 2nd word
				 * (continuation frags).
				 *
				 * We parse C/F/N for fragment reassembly, then dump
				 * raw hex from the data words. */
				if (mw1 <= 87 && mw2 <= 87) {
					char hex[512];
					int hi = 0, w;
					int hex_c = 0, hex_f = 0, hex_n = 0;
					int hex_is_initial = 0;
					int hex_blocking = 0; /* B field from hdr2 (0=16 bits/char) */
					uint32_t rx_hex_sig = 0; /* S field from hdr2 (8-bit signature) */
					int hex_has_sig = 0; /* 1 if we extracted S from hdr2 */
					const char *hex_sig_status = ""; /* signature validation result */
					char hex_frag_flag = '?';
					int data_start = mw1; /* default: all words */

					/* Parse header word.
					 * Per Sections 3.9.3: for long addresses,
					 * body[0] (hdr1) is at Vy (j+1), and mw1
					 * points to body[1] in the Message Field.
					 * For short addresses, body[0] is at mw1. */
					int hex_hdr_idx = is_long ? (j + 1) : mw1;
					if (hex_hdr_idx < FLEX_WORDS_PER_FRAME &&
					    (ph->status[hex_hdr_idx] == FLEX_WORD_CLEAN ||
					     ph->status[hex_hdr_idx] == FLEX_WORD_CORRECTED)) {
						uint32_t hdr = ph->words[hex_hdr_idx];
						hex_c = (hdr & FLEX_HEX_HDR_C_MASK) >> FLEX_HEX_HDR_C_SHIFT;
						hex_f = (hdr & FLEX_HEX_HDR_F_MASK) >> FLEX_HEX_HDR_F_SHIFT;
						hex_n = (hdr & FLEX_HEX_HDR_N_MASK) >> FLEX_HEX_HDR_N_SHIFT;
						hex_is_initial = (hex_f == 3);

						if (hex_c == 0 && hex_f == 3) hex_frag_flag = 'K';
						else if (hex_c == 0) hex_frag_flag = 'C';
						else hex_frag_flag = 'F';

						/* Data starts after header words.
						 * Short address layout:
						 *   mw1=hdr1, mw1+1=hdr2 (initial), data from mw1+2
						 *   mw1=hdr1, data from mw1+1 (continuation)
						 * Long address layout (hdr1 at Vy):
						 *   mw1=hdr2 (initial), data from mw1+1
						 *   mw1=data (continuation), data from mw1 */
						if (is_long) {
							if (hex_is_initial) {
								data_start = mw1 + 1;
								/* Parse header2 from mw1 (body[1]) */
								if (mw1 <= mw2 &&
								    (ph->status[mw1] == FLEX_WORD_CLEAN ||
								     ph->status[mw1] == FLEX_WORD_CORRECTED)) {
									uint32_t hdr2 = ph->words[mw1];
									hex_blocking = (hdr2 >> FLEX_HEX_HDR2_B_SHIFT) & 0xF;
									rx_hex_sig = (hdr2 >> FLEX_HEX_HDR2_S_SHIFT) & 0xFF;
									hex_has_sig = 1;
									LOGP_CHAN(DDSP, LOGL_DEBUG,
										  "RX: Phase %c hex hdr2[%d]=0x%05X: "
										  "B=%d (%d bits/char) S=0x%02X\n",
										  phase_name, mw1, hdr2,
										  hex_blocking,
										  hex_blocking ? hex_blocking : 16,
										  rx_hex_sig);
								}
							} else {
								data_start = mw1;
							}
						} else {
							if (hex_is_initial) {
								data_start = mw1 + 2;
								/* Parse header2 for B and S fields */
								if ((mw1 + 1) <= mw2 &&
								    (ph->status[mw1 + 1] == FLEX_WORD_CLEAN ||
								     ph->status[mw1 + 1] == FLEX_WORD_CORRECTED)) {
									uint32_t hdr2 = ph->words[mw1 + 1];
									hex_blocking = (hdr2 >> FLEX_HEX_HDR2_B_SHIFT) & 0xF;
									rx_hex_sig = (hdr2 >> FLEX_HEX_HDR2_S_SHIFT) & 0xFF;
									hex_has_sig = 1;
									LOGP_CHAN(DDSP, LOGL_DEBUG,
										  "RX: Phase %c hex hdr2[%d]=0x%05X: "
										  "B=%d (%d bits/char) S=0x%02X\n",
										  phase_name, mw1 + 1, hdr2,
										  hex_blocking,
										  hex_blocking ? hex_blocking : 16,
										  rx_hex_sig);
								}
							} else {
								data_start = mw1 + 1;
							}
						}

						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c hex hdr[%d]=0x%05X: "
							  "checksum=0x%03X continued=%d frag=%d "
							  "msgnum=%d (%s)\n",
							  phase_name, hex_hdr_idx, hdr,
							  (int)(hdr & FLEX_HEX_HDR_K_MASK),
							  hex_c, hex_f, hex_n,
							  (hex_frag_flag == 'K') ? "complete" :
							  (hex_frag_flag == 'F') ? "continuation" :
							  "final");
					}

					/* Unpack nibbles from data words to flat hex string.
					 * Each 21-bit word holds 5 × 4-bit nibbles (bits 0-19).
					 * Output format: continuous uppercase hex chars like
					 * "DEADBEEF01020304" (no spaces).
					 *
					 * Termination detection (Spec §3.10.1.2):
					 * The last data word's unused bits are filled with the
					 * inverse of the last valid data bit.  We detect this
					 * fill pattern in the last word to find the exact data
					 * boundary and strip padding nibbles.
					 *
					 * For non-last fragments (C=1), the spec says data must
					 * end at character boundaries and continue from bit 1
					 * of the next fragment's 2nd word — so no partial-word
					 * termination fill is expected (all 21 bits are data). */
					{
						int last_data_word = -1;
						for (w = data_start; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
							uint32_t dw;
							int n;
							if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
							    ph->status[w] == FLEX_WORD_NOT_RECEIVED)
								continue;
							dw = ph->words[w];
							last_data_word = w;
							for (n = 0; n < 5; n++) {
								uint8_t nibble = (dw >> (n * 4)) & 0xF;
								if (hi < (int)sizeof(hex) - 1)
									hex[hi++] = "0123456789ABCDEF"[nibble];
							}
						}

						/* Strip termination fill from last word of
						 * last fragment (C=0) or complete message.
						 * The fill is inverse-of-last-data-bit from
						 * the end of real data to bit 20.
						 *
						 * Strategy: scan backwards from bit 20 of the
						 * last word.  All consecutive identical bits
						 * from the top are fill.  The transition point
						 * marks the last real data bit.  We then round
						 * up to the next nibble boundary to find how
						 * many complete nibbles are real data.
						 *
						 * Per spec rule (3): if the last character is
						 * all-0 or all-1, an extra fill word is appended.
						 * We must strip that extra word first, then strip
						 * partial fill from the actual last data word. */
						if (last_data_word >= 0 && hex_c == 0) {
							/* Walk backwards through data words,
							 * stripping fill from the end */
							int strip_word = last_data_word;
							int word_offset = 0;
							int ww;

							/* Count nibbles from all data words */
							for (ww = data_start; ww <= last_data_word; ww++) {
								if (ph->status[ww] == FLEX_WORD_UNCORRECTABLE ||
								    ph->status[ww] == FLEX_WORD_NOT_RECEIVED)
									continue;
								word_offset += 5;
							}

							/* Strip from the last word backwards */
							while (strip_word >= data_start) {
								uint32_t lw;
								uint32_t top_bit;
								int fill_start, b, real_nibbles;
								int this_word_offset;

								if (ph->status[strip_word] == FLEX_WORD_UNCORRECTABLE ||
								    ph->status[strip_word] == FLEX_WORD_NOT_RECEIVED)
									break;

								lw = ph->words[strip_word];
								top_bit = (lw >> 20) & 1;
								fill_start = 21;

								for (b = 20; b >= 0; b--) {
									if (((lw >> b) & 1) != top_bit)
										break;
									fill_start = b;
								}

								if (fill_start >= 21)
									break; /* no fill in this word */

								real_nibbles = (fill_start + 3) / 4;

								/* Compute offset of this word's nibbles */
								this_word_offset = 0;
								for (ww = data_start; ww < strip_word; ww++) {
									if (ph->status[ww] == FLEX_WORD_UNCORRECTABLE ||
									    ph->status[ww] == FLEX_WORD_NOT_RECEIVED)
										continue;
									this_word_offset += 5;
								}

								if (real_nibbles == 0) {
									/* Entire word is fill — remove it
									 * and check the previous word */
									hi = this_word_offset;
									strip_word--;
									continue;
								}

								/* Partial fill — trim to real nibbles */
								if (this_word_offset + real_nibbles < hi)
									hi = this_word_offset + real_nibbles;
								break;
							}
						}
					}
					hex[hi] = '\0';

					/* S: Verify 8-bit message signature on initial fragment.
					 *
					 * Per Spec §3.10.1.2: "Signature is defined as the
					 * 1's complement of binary sum for the entire message
					 * (including all fragments) for every 8 bits, beginning
					 * with the first 8 bits which follow directly after the
					 * Signature Field.  The 8 LSB of the result is
					 * transmitted as the Message Signature."
					 *
					 * Note: Termination bits are NOT included.
					 *
					 * For a complete (single-fragment) message, we can
					 * validate immediately.  For multi-fragment messages,
					 * the signature covers ALL fragments — validation
					 * must wait until reassembly is complete.
					 *
					 * We compute the sum over the raw data bits from the
					 * data words, stopping at hi*4 bits (the actual data
					 * length after termination stripping). */
					if (hex_is_initial && hex_has_sig) {
						uint32_t sig_sum = 0;
						int total_data_bits = hi * 4;
						int bit_pos = 0;
						uint8_t accum = 0;
						int accum_bits = 0;

						for (w = data_start; w <= mw2 && w < FLEX_WORDS_PER_FRAME && bit_pos < total_data_bits; w++) {
							int b;
							uint32_t dw;
							if (ph->status[w] == FLEX_WORD_UNCORRECTABLE ||
							    ph->status[w] == FLEX_WORD_NOT_RECEIVED)
								continue;
							dw = ph->words[w];
							for (b = 0; b < 21 && bit_pos < total_data_bits; b++) {
								accum |= (uint8_t)(((dw >> b) & 1) << accum_bits);
								accum_bits++;
								bit_pos++;
								if (accum_bits == 8) {
									sig_sum += accum;
									accum = 0;
									accum_bits = 0;
								}
							}
						}
						/* Include any remaining partial byte */
						if (accum_bits > 0)
							sig_sum += accum;

						{
							uint32_t expected_sig = (~sig_sum) & 0xFF;
							if (hex_c == 0) {
								/* Complete message — validate now */
								if (rx_hex_sig == expected_sig) {
									hex_sig_status = ",sig=OK";
									LOGP_CHAN(DDSP, LOGL_DEBUG,
										  "RX: Phase %c HEX signature OK "
										  "(S=0x%02X, sum=0x%02X, %d data bits)\n",
										  phase_name, rx_hex_sig,
										  sig_sum & 0xFF, total_data_bits);
								} else {
									hex_sig_status = ",sig=FAIL";
									LOGP_CHAN(DDSP, LOGL_NOTICE,
										  "RX: Phase %c HEX signature MISMATCH: "
										  "received=0x%02X computed=0x%02X "
										  "(sum=0x%02X, %d data bits)\n",
										  phase_name, rx_hex_sig, expected_sig,
										  sig_sum & 0xFF, total_data_bits);
								}
							} else {
								/* First fragment of multi-fragment message.
								 * Signature covers all fragments — log
								 * partial sum for debugging, defer final
								 * validation to reassembly completion. */
								hex_sig_status = ",sig=partial";
								LOGP_CHAN(DDSP, LOGL_DEBUG,
									  "RX: Phase %c HEX signature deferred "
									  "(multi-fragment): S=0x%02X, partial_sum=0x%02X "
									  "(%d data bits in this fragment)\n",
									  phase_name, rx_hex_sig,
									  sig_sum & 0xFF, total_data_bits);
							}
						}
					}

					/* For continuation/final fragments, recover B
					 * from the reassembly slot (only the initial
					 * fragment carries header2 with the B field). */
					if (!hex_is_initial && hex_blocking == 0) {
						int slot = reasm_find(flex, capcode, hex_n);
						if (slot >= 0)
							hex_blocking = flex->rx.reasm[slot].blocking;
					}

					/* Pad output to full byte boundary for parseable
					 * hex display, using the spec's termination fill
					 * convention (ARIB STD-43A §3.10.1.2):
					 * inverse of the last data bit.
					 *
					 * If data already ends on a byte boundary, append
					 * a full byte of fill so consumers can always
					 * detect the fill/data boundary by scanning
					 * identical trailing bits.
					 *
					 * This means the hex string is always a whole
					 * number of bytes, and the real data length can
					 * be recovered by stripping trailing fill bits
					 * (same algorithm as the spec's termination
					 * detection). */
					if (hi > 0) {
						/* Last data nibble → last data bit */
						uint8_t last_nib = (uint8_t)((hex[hi - 1] >= '0' && hex[hi - 1] <= '9')
							? (hex[hi - 1] - '0')
							: (hex[hi - 1] - 'A' + 10));
						uint8_t last_bit = (last_nib >> 3) & 1;
						uint8_t fill_nib = last_bit ? 0x0 : 0xF;
						char fill_char = "0123456789ABCDEF"[fill_nib];

						if ((hi & 1) == 0) {
							/* Already on byte boundary — add full
							 * fill byte (2 nibbles) */
							hex[hi++] = fill_char;
							hex[hi++] = fill_char;
						} else {
							/* Odd nibble count — pad to byte */
							hex[hi++] = fill_char;
						}
						hex[hi] = '\0';
					}

					/* Log blocking length and decoded block count */
					{
						int bl = hex_blocking ? hex_blocking : 16;
						int data_bits = hi * 4;
						int blocks = (bl > 0) ? data_bits / bl : 0;
						LOGP_CHAN(DDSP, LOGL_DEBUG,
							  "RX: Phase %c HEX/Binary: B=%d (%d bits/char), "
							  "%d data bits, %d blocks\n",
							  phase_name, hex_blocking, bl,
							  data_bits, blocks);
					}

					/* Always output this fragment independently */
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s,frag=%s,B=%d%s [%09" PRIu64 "] %c%c%c HEX [%s]\n",
						  bitrate,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  phase_name,
						  flex->rx.sync_baud,
						  flex->rx.sync_levels,
						  flex->rx.polarity ? "neg" : "pos",
						  (hex_frag_flag == 'K') ? "complete" :
						  (hex_frag_flag == 'F') ? (hex_is_initial ? "frag_start" : "frag_cont") :
						  (hex_frag_flag == 'C') ? "frag_end" : "unknown",
						  hex_blocking,
						  hex_sig_status,
						  capcode,
						  flex_addr_type_flag(aw_type, is_long),
						  grp_flag,
						  prio_flag,
						  hex);

					/* HEX fragment reassembly (same pattern as alpha).
					 * We reassemble the hex string representation.
					 * No separator between fragments — hex is a
					 * continuous nibble stream, not text. */
					if (hex_frag_flag == 'F' && hex_is_initial) {
						int slot = reasm_alloc(flex, capcode, hex_n, FLEX_VECTOR_TYPE_HEX_BINARY);
						flex->rx.reasm[slot].blocking = hex_blocking;
						reasm_append(flex, slot, hex, hi);
					} else if (hex_frag_flag == 'F') {
						int slot = reasm_find(flex, capcode, hex_n);
						if (slot >= 0) {
							hex_blocking = flex->rx.reasm[slot].blocking;
							if (hex_f != flex->rx.reasm[slot].expected_f)
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: Phase %c HEX reassembly F mismatch for [%09" PRIu64 "] msgnum=%d: expected=%d got=%d (missing fragment?)\n",
									  phase_name, capcode, hex_n,
									  flex->rx.reasm[slot].expected_f, hex_f);
							reasm_append(flex, slot, hex, hi);
							flex->rx.reasm[slot].expected_f = (hex_f + 1) % 3;
						}
					} else if (hex_frag_flag == 'C') {
						int slot = reasm_find(flex, capcode, hex_n);
						if (slot >= 0) {
							hex_blocking = flex->rx.reasm[slot].blocking;
							if (hex_f != flex->rx.reasm[slot].expected_f)
								LOGP_CHAN(DDSP, LOGL_NOTICE,
									  "RX: Phase %c HEX reassembly F mismatch for [%09" PRIu64 "] msgnum=%d: expected=%d got=%d (missing fragment?)\n",
									  phase_name, capcode, hex_n,
									  flex->rx.reasm[slot].expected_f, hex_f);
							reasm_append(flex, slot, hex, hi);
							{
								int rbl = hex_blocking ? hex_blocking : 16;
								int r_bits = flex->rx.reasm[slot].len * 4;
								int r_blocks = (rbl > 0) ? r_bits / rbl : 0;
								LOGP_CHAN(DDSP, LOGL_DEBUG,
									  "RX: Phase %c HEX/Binary reassembled: B=%d (%d bits/char), "
									  "%d data bits, %d blocks\n",
									  phase_name, hex_blocking, rbl,
									  r_bits, r_blocks);
							}
							LOGP_CHAN(DDSP, LOGL_NOTICE,
								  "RX: %dbps cycle=%u,frame=%u,phase=%c,baud=%d,fsk=%d,polarity=%s,frag=%s,B=%d [%09" PRIu64 "] %c%c%c HEX [%s]\n",
								  bitrate,
								  flex->rx.fiw_cycle, flex->rx.fiw_frame,
								  phase_name,
								  flex->rx.sync_baud,
								  flex->rx.sync_levels,
								  flex->rx.polarity ? "neg" : "pos",
								  "reassembled",
								  hex_blocking,
								  capcode,
								  flex_addr_type_flag(aw_type, is_long),
								  grp_flag,
								  prio_flag,
								  flex->rx.reasm[slot].buf);
							flex->rx.reasm[slot].active = 0;
						}
					}
					/* hex_frag_flag == 'K': complete message, no reassembly needed */
				}
			} else {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c unknown type %d from %" PRIu64 ".\n",
					  phase_name, vec_type, capcode);
			}
		}
		} /* end block scope for addr_idx/vec_count */
	}
}

/* Clear all phase data buffers before reading a new frame. */
static void flex_rx_clear_phase_data(flex_t *flex)
{
	int i;

	for (i = 0; i < FLEX_MAX_PHASES; i++) {
		memset(&flex->rx.phase[i], 0, sizeof(flex->rx.phase[i]));
		/* status[] is zeroed by memset — FLEX_WORD_NOT_RECEIVED == 0 */
		flex->rx.phase[i].rx_phase = -1;	/* not received */
	}
	flex->rx.phase_toggle = 0;
	flex->rx.data_bit_counter = 0;
}

/* Decode all active phases after data reading is complete.
 * Which phases are active depends on baud/levels:
 *   1600/2: A only
 *   1600/4: A, C (simultaneous via 4-level dibit, per Section 3.3.2)
 *   3200/2: A, C (interleaved via toggle)
 *   3200/4: A, B, C, D (interleaved + simultaneous) */
static void flex_rx_decode_data(flex_t *flex)
{
	/* Expire stale temporary group assignments (§5.2 timeout).
	 * Per standard: "The first transmission of the Group message must
	 * be started within 128 Frames from the first transmission of the
	 * Short Instruction Vector."  If 128+ frames have elapsed since
	 * SETUP without delivery, the assignment is stale — clear it.
	 *
	 * Frame numbers wrap at 128 (0-127), cycle at 15 (0-14).
	 * Total frame count = cycle * 128 + frame. */
	{
		int p, s;
		uint32_t cur_total = flex->rx.fiw_cycle * 128 + flex->rx.fiw_frame;

		for (p = 0; p < FLEX_MAX_PHASES; p++) {
			for (s = 0; s < FLEX_TEMP_ADDR_SLOTS; s++) {
				if (!flex->rx.temp_addr_map[p][s].active)
					continue;
				uint32_t setup_total = flex->rx.temp_addr_map[p][s].setup_cycle * 128
						     + flex->rx.temp_addr_map[p][s].setup_frame;
				/* Handle wraparound: 15 cycles * 128 frames = 1920 total */
				uint32_t elapsed = (cur_total >= setup_total)
						 ? (cur_total - setup_total)
						 : (15 * 128 - setup_total + cur_total);
				if (elapsed >= 128) {
					static const char pn[FLEX_MAX_PHASES] = { 'A', 'B', 'C', 'D' };
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c temp slot=%d TIMEOUT — %u frames since SETUP, clearing.\n",
						  pn[p], s, elapsed);
					memset(&flex->rx.temp_addr_map[p][s], 0,
					       sizeof(flex->rx.temp_addr_map[p][s]));
				}
			}
		}
	}

	/* ReFLEX stub: attempt standard FLEX decode (may partially work),
	 * then hex-dump raw phase words for analysis — both per-phase
	 * and as one contiguous block (in case ReFLEX uses a different
	 * phase layout than standard FLEX A4). */
	if (flex->rx.reflex) {
		LOGP_CHAN(DDSP, LOGL_NOTICE,
			  "RX: ReFLEX frame C%u/F%u — attempting FLEX decode (may fail), then hex dump.\n",
			  flex->rx.fiw_cycle, flex->rx.fiw_frame);

		/* Try standard FLEX phase decode — ReFLEX shares the same
		 * physical layer as A4 (6400/4FSK, 4 phases) so BCH and
		 * address/vector parsing may partially succeed. */
		flex_rx_decode_phase(flex, &flex->rx.phase[0], 'A');
		flex_rx_decode_phase(flex, &flex->rx.phase[1], 'B');
		flex_rx_decode_phase(flex, &flex->rx.phase[2], 'C');
		flex_rx_decode_phase(flex, &flex->rx.phase[3], 'D');

		/* Hex dump: per-phase, then full frame as contiguous block */
		{
			uint32_t *pptrs[4] = {
				flex->rx.phase[0].words, flex->rx.phase[1].words,
				flex->rx.phase[2].words, flex->rx.phase[3].words,
			};
			static const char pnames[4] = { 'A', 'B', 'C', 'D' };
			/* 4 phases × 88 words, 9 chars per word ("XXXXXXXX ") */
			char frame_hex[4 * FLEX_WORDS_PER_FRAME * 9 + 1];
			int frame_pos = 0;
			int frame_nonzero = 0;
			int p, w;

			for (p = 0; p < 4; p++) {
				char hex[FLEX_WORDS_PER_FRAME * 9 + 1];
				int pos = 0;
				int all_zero = 1;

				for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
					if (pptrs[p][w] != 0) {
						all_zero = 0;
						frame_nonzero = 1;
					}
					pos += snprintf(hex + pos, sizeof(hex) - pos,
							"%08X ", pptrs[p][w]);
					frame_pos += snprintf(frame_hex + frame_pos,
							      sizeof(frame_hex) - frame_pos,
							      "%08X ", pptrs[p][w]);
				}
				if (!all_zero) {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: ReFLEX phase %c raw: %s\n",
						  pnames[p], hex);
				}
			}

			if (frame_nonzero) {
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "RX: ReFLEX frame raw (A|B|C|D): %s\n",
					  frame_hex);
			}
		}
		return;
	}

	/* Decode phases based on mode (ARIB STD-43A Section 3.3.4):
	 *   A1 (1600bps/2FSK): 1600 baud, 2 levels → phase A only
	 *   A3 (3200bps/4FSK): 1600 baud, 4 levels → phases A, C
	 *     (4FSK dibit: MSB=phase A, LSB=phase C per Section 3.3.2)
	 *   A2 (3200bps/2FSK): 3200 baud, 2 levels → phases A, C
	 *     (alternating symbols: even=A, odd=C)
	 *   A4 (6400bps/4FSK): 3200 baud, 4 levels → phases A, B, C, D
	 *     (alternating dibit pairs: even sym MSB=A/LSB=B, odd sym MSB=C/LSB=D) */
	if (flex->rx.sync_baud == 1600) {
		if (flex->rx.sync_levels == 2) {
			flex_rx_decode_phase(flex, &flex->rx.phase[0], 'A');
		} else {
			flex_rx_decode_phase(flex, &flex->rx.phase[0], 'A');
			flex_rx_decode_phase(flex, &flex->rx.phase[1], 'C');
		}
	} else {
		if (flex->rx.sync_levels == 2) {
			flex_rx_decode_phase(flex, &flex->rx.phase[0], 'A');
			flex_rx_decode_phase(flex, &flex->rx.phase[2], 'C');
		} else {
			flex_rx_decode_phase(flex, &flex->rx.phase[0], 'A');
			flex_rx_decode_phase(flex, &flex->rx.phase[1], 'B');
			flex_rx_decode_phase(flex, &flex->rx.phase[2], 'C');
			flex_rx_decode_phase(flex, &flex->rx.phase[3], 'D');
		}
	}

	/* Log active temporary group assignments.
	 * These persist across frames until delivery completes or timeout.
	 * Per §5.2: "The Temporary Address is valid until the Group
	 * message ends." */
	{
		static const char pnames[FLEX_MAX_PHASES] = { 'A', 'B', 'C', 'D' };
		int p, s, m;

		for (p = 0; p < FLEX_MAX_PHASES; p++) {
			for (s = 0; s < FLEX_TEMP_ADDR_SLOTS; s++) {
				if (!flex->rx.temp_addr_map[p][s].active)
					continue;
				int cnt = flex->rx.temp_addr_map[p][s].count;
				if (cnt > 0) {
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: C%u/F%u phase=%c temp_group=%d ACTIVE members=%d target_frame=%u setup=C%u/F%u:",
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  pnames[p], s, cnt,
						  flex->rx.temp_addr_map[p][s].target_frame,
						  flex->rx.temp_addr_map[p][s].setup_cycle,
						  flex->rx.temp_addr_map[p][s].setup_frame);
					for (m = 0; m < cnt; m++)
						LOGP_CHAN(DDSP, LOGL_NOTICE,
							  " %" PRIu64,
							  flex->rx.temp_addr_map[p][s].capcodes[m]);
					LOGP_CHAN(DDSP, LOGL_NOTICE, "\n");
				}
			}
		}
	}

	/* Frame-level BCH summary (S1 + FIW + all data phases) */
	LOGP_CHAN(DDSP, LOGL_INFO,
		  "RX: Frame C%u/F%u BCH totals: %u codewords — "
		  "%u clean, %u corrected, %u uncorrectable.\n",
		  flex->rx.fiw_cycle, flex->rx.fiw_frame,
		  flex->bch_stats.total,
		  flex->bch_stats.clean,
		  flex->bch_stats.corrected,
		  flex->bch_stats.uncorrectable);
}

/* Read one data symbol into the appropriate phase buffers.
 * Per ARIB STD-43A Section 3.3 and multimon-ng read_data():
 *
 * 4-level slicer produces bit_a (MSB) and bit_b (LSB) from symbol:
 *   sym 0 → bit_a=0, bit_b=0  (-4800 Hz)
 *   sym 1 → bit_a=0, bit_b=1  (-1600 Hz)
 *   sym 2 → bit_a=1, bit_b=1  (+1600 Hz)  [Gray coded]
 *   sym 3 → bit_a=1, bit_b=0  (+4800 Hz)
 *
 * Phase assignment:
 *   toggle=0: bit_a → phase[0], bit_b → phase[1]
 *   toggle=1: bit_a → phase[2], bit_b → phase[3]
 *
 * At 1600 baud (A3): toggle forced to 0, no interleaving.
 *   Per Section 3.3.2: MSB=phase A, LSB=phase C.
 *   So phase[0]=A, phase[1]=C.
 * At 3200 baud (A4): toggle alternates 0/1 per symbol.
 *   Per Section 3.3.2: even sym MSB=A/LSB=B, odd sym MSB=C/LSB=D.
 *   So phase[0]=A, phase[1]=B, phase[2]=C, phase[3]=D.
 *
 * De-interleave index formula (block de-interleave):
 *   idx = ((data_bit_counter >> 5) & 0xFFF8) | (data_bit_counter & 0x0007)
 * This maps bits 0-2 straight through and shifts bits 5+ down by 2,
 * creating the 8-word block interleave pattern. */
static void flex_rx_read_data(flex_t *flex, unsigned char sym)
{
	int bit_a, bit_b = 0;
	unsigned int idx;

	/* Extract bits from the demodulated symbol.
	 * 2FSK (2 levels): 1 symbol = 1 bit.
	 *   bit_a = MSB of symbol (sym > 1 → 1, else 0).
	 * 4FSK (4 levels): 1 symbol = 2 bits (dibit).
	 *   bit_a = MSB of dibit, bit_b = LSB of dibit.
	 *   Per ARIB STD-43A Section 3.3.2: MSB is phase a/c, LSB is phase b/d.
	 *   Symbol levels 0-3 map to dibits via Gray decode:
	 *     sym 0 (-4800Hz) → "00", sym 1 (-1600Hz) → "01",
	 *     sym 2 (+1600Hz) → "11", sym 3 (+4800Hz) → "10".
	 *   bit_a (MSB) = (sym > 1), bit_b (LSB) = (sym == 1 || sym == 2). */
	bit_a = (sym > 1);
	if (flex->rx.sync_levels == 4)
		bit_b = (sym == 1) || (sym == 2);

	if (flex->rx.sync_baud == 1600)
		flex->rx.phase_toggle = 0;

	/* De-interleave index */
	idx = ((flex->rx.data_bit_counter >> 5) & 0xFFF8) |
	      (flex->rx.data_bit_counter & 0x0007);

	if (idx >= FLEX_WORDS_PER_FRAME) {
		/* Safety: don't overflow phase buffers */
		return;
	}

	if (flex->rx.phase_toggle == 0) {
		flex->rx.phase[0].words[idx] = (flex->rx.phase[0].words[idx] >> 1) |
					       (bit_a ? 0x80000000U : 0);
		flex->rx.phase[1].words[idx] = (flex->rx.phase[1].words[idx] >> 1) |
					       (bit_b ? 0x80000000U : 0);
		flex->rx.phase_toggle = 1;

		/* Track idle words for early termination (Fig. 3.4.1-3).
		 * Frames can be shortened when unused blocks contain only
		 * idle fill (alternating all-1s/all-0s words). */
		if ((flex->rx.data_bit_counter & 0xFF) == 0xFF) {
			if (flex->rx.phase[0].words[idx] == 0x00000000 ||
			    flex->rx.phase[0].words[idx] == 0xFFFFFFFF)
				flex->rx.phase[0].idle_count++;
			if (flex->rx.phase[1].words[idx] == 0x00000000 ||
			    flex->rx.phase[1].words[idx] == 0xFFFFFFFF)
				flex->rx.phase[1].idle_count++;
		}
	} else {
		flex->rx.phase[2].words[idx] = (flex->rx.phase[2].words[idx] >> 1) |
					       (bit_a ? 0x80000000U : 0);
		flex->rx.phase[3].words[idx] = (flex->rx.phase[3].words[idx] >> 1) |
					       (bit_b ? 0x80000000U : 0);
		flex->rx.phase_toggle = 0;

		if ((flex->rx.data_bit_counter & 0xFF) == 0xFF) {
			if (flex->rx.phase[2].words[idx] == 0x00000000 ||
			    flex->rx.phase[2].words[idx] == 0xFFFFFFFF)
				flex->rx.phase[2].idle_count++;
			if (flex->rx.phase[3].words[idx] == 0x00000000 ||
			    flex->rx.phase[3].words[idx] == 0xFFFFFFFF)
				flex->rx.phase[3].idle_count++;
		}
	}

	/* Advance data_bit_counter once per complete bit-pair cycle.
	 * At 1600 baud (A1, A3): every symbol advances the counter
	 *   (A1 has only phase a; A3 has phases a+c in one 4FSK symbol).
	 * At 3200 baud (A2, A4): two consecutive symbols form one bit-pair
	 *   (first symbol → phases a,b; second → phases c,d).
	 *   Counter advances only after the second symbol (phase_toggle==0). */
	if (flex->rx.sync_baud == 1600 || flex->rx.phase_toggle == 0)
		flex->rx.data_bit_counter++;
}

/* Process one demodulated symbol through the RX state machine.
 * Called once per symbol period from the PLL.
 * sym is the 4-level symbol value (0-3), already rectified for polarity. */
static void flex_rx_sym(flex_t *flex, unsigned char sym)
{
	/* Rectify symbol for polarity */
	unsigned char sym_rect;
	if (flex->rx.polarity)
		sym_rect = 3 - sym;
	else
		sym_rect = sym;

	switch (flex->rx.rx_state) {
	case RX_STATE_SYNC1:
	{
		/* S1 = BS1(32) + A(32) + B(16) + inv.A(32) = 112 bits.
		 * Always 1600 baud / 2FSK.
		 *
		 * The 128-bit shift register accumulates raw (non-polarity-
		 * corrected) bits throughout.  Sync detection fires at bit 96
		 * when the marker appears in sync_buf_lo.  After that, 16 more
		 * symbols complete inv.A.  s1_tail_count tracks progress:
		 *   0 = hunting (sync not yet detected)
		 *   1..16 = counting inv.A tail after sync */
		int bit = (sym < 2) ? 1 : 0;
		flex->rx.sync_buf_hi = (flex->rx.sync_buf_hi << 1) |
				       (flex->rx.sync_buf_lo >> 63);
		flex->rx.sync_buf_lo = (flex->rx.sync_buf_lo << 1) | bit;

		if (flex->rx.s1_tail_count == 0) {
			/* Hunting: check for sync marker */
			unsigned int sync_code;
			int polarity;
			sync_code = flex_rx_sync_detect(flex,
				flex->rx.sync_buf_lo, &polarity);
			if (sync_code != 0) {
				flex->rx.polarity = polarity;
				if (flex_rx_decode_mode(flex, sync_code,
							polarity))
					flex->rx.s1_tail_count = 1;
			}
		} else if (++flex->rx.s1_tail_count > 16) {
			/* S1 complete — extract inv.A, run combined
			 * A/inv.A error correction.  This is the final
			 * sync validation: if correction fails, the
			 * sync detection was a false positive. */
			uint32_t inv_a = flex_extract_inv_a(
				flex->rx.sync_buf_lo,
				flex->rx.polarity);

			/* Reset BCH stats — this is the start of a new
			 * frame.  The combined correction's BCH decodes
			 * of A and ~inv.A are the authoritative counts. */
			memset(&flex->bch_stats, 0, sizeof(flex->bch_stats));

			uint32_t corrected = flex_combined_a_correction(
				flex, flex->rx.sync_a_code, inv_a);

			if (corrected == 0) {
				/* Rejected — back to hunting */
				flex->rx.s1_tail_count = 0;
				break;
			}

			/* Validate mode from the combined-corrected A-code.
			 * We cannot call flex_rx_decode_mode() here because
			 * it re-extracts A from sync_buf_lo, which has shifted
			 * 16 bits since sync detection.  Instead, derive the
			 * sync code and compare against the already-established
			 * mode directly. */
			flex->rx.sync_a_code = corrected;
			{
				uint32_t final_sync = (~corrected >> 16) & 0xFFFF;
				int prev_baud = flex->rx.sync_baud;
				int prev_levels = flex->rx.sync_levels;

				/* Look up mode from corrected sync code */
				static const struct {
					uint16_t code;
					int baud;
					int levels;
				} modes[] = {
					{ FLEX_SYNC_A1, 1600, 2 },
					{ FLEX_SYNC_A3, 1600, 4 },
					{ FLEX_SYNC_A2, 3200, 2 },
					{ FLEX_SYNC_A4, 3200, 4 },
					{ FLEX_SYNC_REFLEX, 3200, 4 },
					{ 0, 0, 0 }
				};
				int found = 0, mi;
				for (mi = 0; modes[mi].code != 0; mi++) {
					if (final_sync == modes[mi].code) {
						flex->rx.sync_baud = modes[mi].baud;
						flex->rx.sync_levels = modes[mi].levels;
						found = 1;
						break;
					}
				}
				if (!found) {
					/* Corrected code doesn't map to a
					 * valid mode — reject */
					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: combined correction "
						  "produced unknown sync "
						  "0x%04X, rejecting.\n",
						  final_sync);
					flex->rx.s1_tail_count = 0;
					break;
				}
				if (final_sync == FLEX_SYNC_AR) {
					flex->rx.s1_tail_count = 0;
					break;
				}
				if (flex->rx.sync_baud != prev_baud ||
				    flex->rx.sync_levels != prev_levels) {
					LOGP_CHAN(DDSP, LOGL_INFO,
						  "RX: combined correction "
						  "changed mode: %d/%d → %d/%d.\n",
						  prev_baud, prev_levels,
						  flex->rx.sync_baud,
						  flex->rx.sync_levels);
				}
			}

			flex->rx.rx_state = RX_STATE_FIW;
			flex->rx.fiw_count = 0;
			flex->rx.fiw_rawdata = 0;
		}
		break;
	}

	case RX_STATE_FIW:
	{
		/* FIW: 32-bit BCH codeword, always 1600/2FSK. */
		flex->rx.fiw_rawdata = (flex->rx.fiw_rawdata >> 1) |
				       ((sym_rect > 1) ? 0x80000000U : 0);

		if (++flex->rx.fiw_count == 32) {
			if (flex_rx_decode_fiw(flex, flex->rx.fiw_rawdata) == 0) {
				flex->rx.baud = flex->rx.sync_baud;
				flex->rx.sync2_count = 0;
				flex->rx.sync2_shiftreg = 0;
				flex->rx.sync2_c_found = 0;
				flex->rx.sync2_c_pos = 0;
				flex->rx.sync2_c_errs = 0;
				flex->rx.sync2_cinv_found = 0;
				flex->rx.sync2_cinv_pos = 0;
				flex->rx.sync2_cinv_errs = 0;
				flex->rx.sync2_sym_buf_count = 0;
				flex->rx.sync2_sym_buf_start = 0;
				flex->rx.rx_state = RX_STATE_SYNC2;
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: FIW→SYNC2, symbol rate switch to %d baud.\n",
					  flex->rx.baud);
			} else {
				flex->rx.rx_state = RX_STATE_SYNC1;
				flex->rx.s1_tail_count = 0;
			}
		}
		break;
	}

	case RX_STATE_SYNC2:
	{
		/* S2: 25 ms at the data symbol rate (ARIB STD-43A Section 3.2).
		 * S2 = BS2 + C(0xED84) + inv.BS2 + inv.C(0x127B).
		 *
		 * By this point the PLL is already well-synchronized: it locked
		 * during S1 (112 symbols of BS1+A+B+inv.A at 1600 baud), then
		 * tracked 48 more symbols of FIW.  The only disruption is the
		 * baud rate switch (1600 → data rate), and BS2's alternating
		 * pattern re-trains the PLL at the new rate before C arrives.
		 * Corrections here should be extremely rare.
		 *
		 * S2 is symmetric: first half = BS2 + C, second half = inv.BS2 + inv.C.
		 * Distance from C-end to inv.C-end is always s2_symbols/2.
		 * In a correctly-synced signal, inv.C ends at exactly s2_symbols —
		 * the same position as the blind symbol count.  The normal outcome
		 * is that detection confirms the blind count, or falls back to it.
		 *
		 * We scan the full window (nominal + 2 symbols), record where
		 * C and inv.C are found, and only correct if the evidence is
		 * unambiguous: both patterns perfect (0 errors), at the exact
		 * expected distance, with a small offset from nominal.
		 *
		 * S2 component breakdown (in decoded bits / symbols):
		 *   Mode        BS2   C    inv.BS2  inv.C  Total sym
		 *   A1 (1600/2) :  4 + 16 +   4   + 16   = 40
		 *   A2 (3200/2) : 24 + 16 +  24   + 16   = 80
		 *   A3 (3200/4) : 12 +  8 +  12   +  8   = 40
		 *   A4 (6400/4) : 32 +  8 +  32   +  8   = 80 */
		int s2_symbols = flex->rx.sync_baud * 25 / 1000;
		int s2_half = s2_symbols / 2;

		/* Scan window: nominal + 2 symbols.  We are already synced
		 * from S1, so this margin is a safety net, not an expectation. */
		int s2_window = s2_symbols + 2;

		/* Extract decoded bit(s) from this symbol and shift into
		 * the 16-bit C pattern shift register.
		 * 2FSK: 1 bit per symbol (bit_a only).
		 * 4FSK: 2 bits per symbol (bit_a=MSB, bit_b=LSB via Gray). */
		int bit_a = (sym_rect > 1);

		if (flex->rx.sync_levels == 4) {
			int bit_b = (sym_rect == 1) || (sym_rect == 2);
			flex->rx.sync2_shiftreg = (uint16_t)(
				(flex->rx.sync2_shiftreg << 2) |
				(bit_a << 1) | bit_b);
		} else {
			flex->rx.sync2_shiftreg = (uint16_t)(
				(flex->rx.sync2_shiftreg << 1) | bit_a);
		}

		/* Buffer symbols around the nominal boundary [nominal-2 .. nominal+2).
		 * These may be data symbols if the boundary is earlier than nominal,
		 * or S2 tail if the boundary is later.  We decide at the end. */
		if (flex->rx.sync2_count >= s2_symbols - 2 &&
		    flex->rx.sync2_sym_buf_count < 4) {
			if (flex->rx.sync2_sym_buf_count == 0)
				flex->rx.sync2_sym_buf_start = flex->rx.sync2_count;
			flex->rx.sync2_sym_buf[flex->rx.sync2_sym_buf_count++] = sym_rect;
		}

		flex->rx.sync2_count++;

		/* Correlate against both C and inv.C patterns for diagnostics.
		 * We record positions but do NOT transition yet — we always
		 * scan the full window before making a decision.
		 * Detection threshold ≤2 errors is for logging only;
		 * correction requires 0 errors (see decision logic below). */
		if (flex->rx.sync2_count >= 16) {
			/* Check for C (first half) — take first match only */
			if (!flex->rx.sync2_c_found) {
				unsigned int c_errs = count_bits(
					flex->rx.sync2_shiftreg ^ FLEX_S2_C);
				if (c_errs <= 2) {
					flex->rx.sync2_c_found = 1;
					flex->rx.sync2_c_pos = flex->rx.sync2_count;
					flex->rx.sync2_c_errs = (int)c_errs;
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: S2 C detected at symbol %d/%d "
						  "(%u bit error%s, reg=0x%04X).\n",
						  flex->rx.sync2_count, s2_symbols,
						  c_errs, c_errs == 1 ? "" : "s",
						  flex->rx.sync2_shiftreg);
				}
			}

			/* Check for inv.C (second half) — take first match only */
			if (!flex->rx.sync2_cinv_found) {
				unsigned int errs = count_bits(
					flex->rx.sync2_shiftreg ^ FLEX_S2_C_INV);
				if (errs <= 2) {
					flex->rx.sync2_cinv_found = 1;
					flex->rx.sync2_cinv_pos = flex->rx.sync2_count;
					flex->rx.sync2_cinv_errs = (int)errs;
					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: S2 inv.C detected at symbol %d/%d "
						  "(%u bit error%s, reg=0x%04X).\n",
						  flex->rx.sync2_count, s2_symbols,
						  errs, errs == 1 ? "" : "s",
						  flex->rx.sync2_shiftreg);
				}
			}
		}

		/* End of scan window — make the boundary decision. */
		if (flex->rx.sync2_count >= s2_window) {
			int boundary = s2_symbols;  /* default: nominal (blind count) */
			int correction = 0;
			const char *reason = "blind";

			/* Decision logic: we are already synced from S1 and do not
			 * expect corrections here.  The blind symbol count is the
			 * correct baseline.  We only override it when the evidence
			 * is strong — anything less falls back to nominal.
			 *
			 * C and inv.C are raw 16-bit correlation patterns, NOT
			 * BCH-protected.  We use them as redundant copies (like
			 * A/inv.A): if at least one is clean (0 errors), we
			 * trust its timing position.
			 *
			 * Correction criteria (ALL must hold):
			 *   1. Both C and inv.C detected (≤2 errors each).
			 *   2. At least one has 0 bit errors.
			 *   3. Gap between them = exactly s2_half.
			 *   4. Offset from nominal is ±1 (not ±0, not ±2). */
			if (flex->rx.sync2_c_found && flex->rx.sync2_cinv_found) {
				int gap = flex->rx.sync2_cinv_pos - flex->rx.sync2_c_pos;
				int offset = flex->rx.sync2_cinv_pos - s2_symbols;
				int best_errs = (flex->rx.sync2_c_errs < flex->rx.sync2_cinv_errs)
					? flex->rx.sync2_c_errs : flex->rx.sync2_cinv_errs;

				if (best_errs == 0 &&
				    gap == s2_half &&
				    offset != 0 &&
				    offset >= -1 && offset <= 1) {
					boundary = flex->rx.sync2_cinv_pos;
					correction = offset;
					reason = (flex->rx.sync2_c_errs == 0 &&
						  flex->rx.sync2_cinv_errs == 0)
						? "C+inv.C perfect"
						: "C/inv.C one clean";
				}
				/* else: detected but not clean enough to correct */
			}
			/* else: neither found → nominal */

			/* Transition to DATA */
			flex->rx.data_count = 0;
			flex_rx_clear_phase_data(flex);
			flex->rx.rx_state = RX_STATE_DATA;

			/* Log only when something noteworthy happens.
			 * Normal case (no correction) is silent — blind
			 * count is the expected baseline. */
			if (correction != 0) {
				LOGP_CHAN(DDSP, LOGL_INFO,
					  "RX: SYNC2→DATA (%s), boundary corrected "
					  "by %+d symbol%s, "
					  "C@%d(%de), inv.C@%d(%de), "
					  "gap=%d (expected %d), "
					  "%d baud/%dFSK.\n",
					  reason, correction,
					  (correction == 1 || correction == -1) ? "" : "s",
					  flex->rx.sync2_c_pos,
					  flex->rx.sync2_c_errs,
					  flex->rx.sync2_cinv_pos,
					  flex->rx.sync2_cinv_errs,
					  flex->rx.sync2_c_found && flex->rx.sync2_cinv_found ?
						flex->rx.sync2_cinv_pos - flex->rx.sync2_c_pos : 0,
					  s2_half,
					  flex->rx.sync_baud,
					  flex->rx.sync_levels);
			} else if (!flex->rx.sync2_c_found && !flex->rx.sync2_cinv_found) {
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "RX: SYNC2→DATA, neither C nor inv.C detected, "
					  "using blind count (%d symbols, %d baud/%dFSK).\n",
					  s2_symbols,
					  flex->rx.sync_baud,
					  flex->rx.sync_levels);
			}

			/* Replay buffered symbols that fall AFTER the decided
			 * boundary.  The buffer starts at sync2_sym_buf_start
			 * (which is s2_symbols - 2).  Symbols at positions
			 * >= boundary are data; symbols < boundary are S2 tail. */
			{
				int k;
				int replay_count = 0;
				for (k = 0; k < flex->rx.sync2_sym_buf_count; k++) {
					int sym_pos = flex->rx.sync2_sym_buf_start + k;
					if (sym_pos >= boundary) {
						flex_rx_read_data(flex,
							flex->rx.sync2_sym_buf[k]);
						flex->rx.data_count++;
						replay_count++;
					}
				}
				if (replay_count > 0) {
					LOGP_CHAN(DDSP, LOGL_INFO,
						  "RX: replayed %d buffered symbol%s "
						  "into data (boundary=%d, buf_start=%d).\n",
						  replay_count,
						  replay_count == 1 ? "" : "s",
						  boundary,
						  flex->rx.sync2_sym_buf_start);
				}
			}
		}
		break;
	}

	case RX_STATE_DATA:
	{
		/* Data: 1760 ms at the data symbol rate (ARIB STD-43A Section 3.3).
		 * sync_baud is the SYMBOL rate, so:
		 *   sync_baud * 1760 / 1000 = number of SYMBOLS in data portion.
		 *
		 * Per standard:
		 *   A1 (1600bps/2FSK): 1600 baud → 2816 symbols (= 2816 bits)
		 *   A2 (3200bps/2FSK): 3200 baud → 5632 symbols (= 5632 bits)
		 *   A3 (3200bps/4FSK): 1600 baud → 2816 symbols (= 5632 bits)
		 *   A4 (6400bps/4FSK): 3200 baud → 5632 symbols (= 11264 bits)
		 *
		 * Each symbol produces 1 bit (2FSK) or 2 bits (4FSK).
		 * flex_rx_read_data() handles the bit extraction per symbol. */
		flex_rx_read_data(flex, sym_rect);

		if (++flex->rx.data_count == flex->rx.sync_baud * 1760 / 1000) {
			int bps = flex->rx.sync_baud * (flex->rx.sync_levels == 4 ? 2 : 1);
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "RX: DATA complete, %d symbols (%d baud, %dbps/%dFSK).\n",
				  flex->rx.data_count, flex->rx.sync_baud,
				  bps, flex->rx.sync_levels);
			flex_rx_decode_data(flex);
			/* Return to sync hunting at 1600 baud / 2FSK */
			flex->rx.baud = 1600;
			flex->rx.rx_state = RX_STATE_SYNC1;
			flex->rx.s1_tail_count = 0;
		}
		break;
	}
	}
}

/* PLL-based symbol timing recovery.
 * Processes one audio sample and returns 1 when a symbol boundary is reached.
 * Uses zero-crossing PLL to track symbol timing, and a 4-level slicer
 * to determine the modal symbol value during each symbol period.
 *
 * This is a standard-compliant implementation following the same algorithm
 * as multimon-ng's buildSymbol(), adapted for our codebase. */
static int flex_rx_build_symbol(flex_t *flex, double sample)
{
	const int64_t phase_max = 100 * (int64_t)flex->rx.sample_freq;
	const int64_t phase_rate = phase_max * (int64_t)flex->rx.baud /
				   (int64_t)flex->rx.sample_freq;
	const double phasepercent = 100.0 * (double)flex->rx.pll_phase /
				    (double)phase_max;

	flex->rx.pll_sample_count++;

	/* DC offset removal (IIR filter, only during sync hunting) */
	if (flex->rx.rx_state == RX_STATE_SYNC1) {
		flex->rx.pll_zero = (flex->rx.pll_zero *
				     (flex->rx.sample_freq * DC_OFFSET_FILTER) +
				     sample) /
				    ((flex->rx.sample_freq * DC_OFFSET_FILTER) + 1);
	}
	sample -= flex->rx.pll_zero;

	if (flex->rx.pll_locked) {
		/* Establish signal envelope during sync hunting */
		if (flex->rx.rx_state == RX_STATE_SYNC1) {
			flex->rx.pll_envelope_sum += fabs(sample);
			flex->rx.pll_envelope_count++;
			flex->rx.pll_envelope =
				flex->rx.pll_envelope_sum /
				flex->rx.pll_envelope_count;
		}
	} else {
		/* Not locked: reset envelope and hold in S1 hunting */
		flex->rx.pll_envelope = 0;
		flex->rx.pll_envelope_sum = 0;
		flex->rx.pll_envelope_count = 0;
		flex->rx.baud = 1600;
		flex->rx.pll_timeout = 0;
		flex->rx.pll_nonconsec = 0;
		flex->rx.rx_state = RX_STATE_SYNC1;
		flex->rx.s1_tail_count = 0;
	}

	/* Mid 80% of symbol period: vote on symbol level */
	if (phasepercent > 10.0 && phasepercent < 90.0) {
		if (sample > 0) {
			if (sample > flex->rx.pll_envelope * SLICE_THRESHOLD)
				flex->rx.pll_symcount[3]++;
			else
				flex->rx.pll_symcount[2]++;
		} else {
			if (sample < -flex->rx.pll_envelope * SLICE_THRESHOLD)
				flex->rx.pll_symcount[0]++;
			else
				flex->rx.pll_symcount[1]++;
		}
	}

	/* Zero crossing detection → PLL phase correction */
	if ((flex->rx.pll_last_sample < 0 && sample >= 0) ||
	    (flex->rx.pll_last_sample >= 0 && sample < 0)) {
		double phase_error;

		if (phasepercent < 50.0)
			phase_error = (double)flex->rx.pll_phase;
		else
			phase_error = (double)flex->rx.pll_phase - (double)phase_max;

		if (flex->rx.pll_locked)
			flex->rx.pll_phase -= (int64_t)(phase_error * PHASE_LOCKED_RATE);
		else
			flex->rx.pll_phase -= (int64_t)(phase_error * PHASE_UNLOCKED_RATE);

		/* Too many zero crossings in mid-symbol → lost lock */
		if (phasepercent > 10.0 && phasepercent < 90.0) {
			flex->rx.pll_nonconsec++;
			if (flex->rx.pll_nonconsec > 20 && flex->rx.pll_locked) {
				LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: PLL lock lost.\n");
				flex->rx.pll_locked = 0;
			}
		} else {
			flex->rx.pll_nonconsec = 0;
		}

		flex->rx.pll_timeout = 0;
	}
	flex->rx.pll_last_sample = sample;

	/* Advance phase accumulator */
	flex->rx.pll_phase += phase_rate;

	/* Symbol boundary reached? */
	if (flex->rx.pll_phase > phase_max) {
		flex->rx.pll_phase -= phase_max;
		return 1;
	}

	return 0;
}

/* Main demodulator: process one audio sample.
 * Calls buildSymbol to track timing, then dispatches the modal symbol
 * to the state machine when a symbol boundary is reached. */
static void flex_rx_demodulate(flex_t *flex, double sample)
{
	if (flex_rx_build_symbol(flex, sample) != 1)
		return;

	/* Symbol boundary reached */
	flex->rx.pll_nonconsec = 0;
	flex->rx.pll_symbol_count++;

	/* Determine modal symbol (most votes wins) */
	int j, decmax = 0, modal_symbol = 0;
	for (j = 0; j < 4; j++) {
		if (flex->rx.pll_symcount[j] > decmax) {
			modal_symbol = j;
			decmax = flex->rx.pll_symcount[j];
		}
	}
	flex->rx.pll_symcount[0] = 0;
	flex->rx.pll_symcount[1] = 0;
	flex->rx.pll_symcount[2] = 0;
	flex->rx.pll_symcount[3] = 0;

	if (flex->rx.pll_locked) {
		/* Process symbol through state machine */
		flex_rx_sym(flex, (unsigned char)modal_symbol);
	} else {
		/* Check for lock pattern (alternating 0/3 symbols = BS1 1,0 pattern).
		 * Shift symbols into buffer; XOR with 0x1 maps sym 0→1, 3→2
		 * (each containing a single set bit).
		 * Lock pattern: 0x6666... (alternating 01 10 01 10...) */
		flex->rx.pll_lock_buf = (flex->rx.pll_lock_buf << 2) |
					(modal_symbol ^ 0x1);
		uint64_t lock_pattern = flex->rx.pll_lock_buf ^ 0x6666666666666666ULL;
		uint64_t lock_mask = (1ULL << (2 * LOCK_LEN)) - 1;
		if ((lock_pattern & lock_mask) == 0 ||
		    ((~lock_pattern) & lock_mask) == 0) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: PLL locked.\n");
			flex->rx.pll_locked = 1;
			flex->rx.pll_lock_buf = 0;
			flex->rx.pll_symbol_count = 0;
			flex->rx.pll_sample_count = 0;
		}
	}

	/* Timeout: no zero crossings for too long → unlock */
	flex->rx.pll_timeout++;
	if (flex->rx.pll_timeout > DEMOD_TIMEOUT) {
		if (flex->rx.pll_locked) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: PLL timeout, unlocking.\n");
		}
		flex->rx.pll_locked = 0;
	}
}

/* Process received audio stream from radio unit.
 * Entry point called by the SDR/WAV layer for each buffer of samples. */
void sender_receive(sender_t *sender, sample_t *samples, int length,
		    double __attribute__((unused)) rf_level_db)
{
	flex_t *flex = (flex_t *)sender;
	int i;

	if (!flex->rx.enabled)
		return;

	for (i = 0; i < length; i++)
		flex_rx_demodulate(flex, (double)samples[i]);

	/* Exit after RX WAV file is fully consumed (--wav-test mode) */
	if (flex->wav_test_mode && sender->wave_rx_play.fp &&
	    sender->wave_rx_play.left == 0) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "RX WAV file finished, exiting.\n");
		quit = 1;
	}
}
