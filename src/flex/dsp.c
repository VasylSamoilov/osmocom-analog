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
 * ARIB STD-43A Section 2: +4800, +1600, -1600, -4800 Hz.
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
 * ARIB STD-43A terminology:
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
	 *   Peak FM deviation.  ARIB STD-43A §2.1.3 specifies ±4.8 kHz for
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
	 *   matches the standard's occupied bandwidth limit (§2.1.7: ≤16 kHz).
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
	 * ARIB STD-43A Chapter 2: occupied bandwidth must not exceed 16 kHz.
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
	 * Per ARIB STD-43A Section 2.1.3, 4-level FM uses Gray coding:
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
		}

		/* Apply baseband LPF after modulation (ARIB STD-43A Chapter 2) */
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

/* ===== FLEX Receiver (ARIB STD-43A compliant) ===== */

/* RX state machine states */
enum {
	RX_STATE_SYNC1 = 0,	/* hunting for S1 sync pattern (always 1600 baud, 2FSK) */
	RX_STATE_FIW,		/* reading FIW (16 bits dotting + 32 bits data, 1600/2FSK) */
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
 * ARIB STD-43A Section 3.5.2: BCH(31,21) + even parity = 32-bit codeword.
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

/* BCH(31,21,2) decoder.
 *
 * Input: 32-bit word as accumulated by RX (LSB-first):
 *   bits 0-20 = data, bits 21-30 = ECC, bit 31 = even parity.
 *
 * Corrects up to 2 bit errors in the 31-bit codeword.
 * Returns corrected 21-bit data (bits 0-20), or -1 if uncorrectable. */
static int32_t flex_bch_decode(uint32_t codeword)
{
	unsigned int key, error;
	uint32_t code31;

	flex_bch_init();

	/* Strip even parity bit (bit 31), work on 31-bit codeword */
	code31 = codeword & 0x7FFFFFFFU;

	/* Compute syndrome */
	key = flex_bch_syndrome(code31);

	if (key == 0) {
		/* No errors in BCH code (parity bit error is harmless) */
		return (int32_t)(code31 & 0x1FFFFF);
	}

	/* Look up error pattern */
	error = bch_err_tbl[key];
	if (error == 0)
		return -1; /* uncorrectable (>2 errors) */

	/* Apply correction */
	code31 ^= error;
	return (int32_t)(code31 & 0x1FFFFF);
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

/* Decode the sync code to determine symbol rate and FSK levels.
 * Per ARIB STD-43A Table 3.2-5, the outer code determines the mode.
 * Returns 1 if valid mode found, 0 otherwise. */
static int flex_rx_decode_mode(flex_t *flex, unsigned int sync_code)
{
	/* Mode table: sync code → symbol rate (baud) and FSK levels.
	 *
	 * ARIB STD-43A Section 3.2, Table 3.2-5:
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

	/* Check for Ar (ERS re-sync).
	 * Per ARIB STD-43A Section 3.2.1: when the receiver detects the Ar
	 * sync code, it must re-synchronize its frame timing.  ERS is not
	 * a data frame — no FIW/S2/DATA follows.  The receiver should reset
	 * to sync-hunting state so it can lock onto the next data frame's
	 * S1 sync after the ERS burst ends. */
	if (count_bits(sync_code ^ FLEX_SYNC_AR) < 4) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "RX: Ar (ERS re-sync) detected — resetting sync.\n");
		return 0;
	}

	/* ReFLEX sync (0x4C7C): Motorola ReFLEX protocol extension.
	 * Same physical layer as A4 (6400bps/4FSK, 3200 baud) but
	 * uses a different framing format.  Decode what we can as
	 * standard FLEX, hex-dump the rest. */
	if (count_bits((uint32_t)(FLEX_SYNC_REFLEX ^ sync_code)) < 4) {
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
		if (count_bits((uint32_t)(modes[i].code ^ sync_code)) < 4) {
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
	data = flex_bch_decode(fiw_raw);
	if (data < 0) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "RX: FIW BCH decode failed (uncorrectable).\n");
		return -1;
	}

	/* FIW layout per multimon-ng decode_fiw():
	 * bits 3-0:   checksum
	 * bits 7-4:   cycle number (0-14)
	 * bits 14-8:  frame number (0-127)
	 * bits 20-15: fix3 (reserved)
	 * Checksum: sum of all 4-bit nibbles + bit 20 = 0xF */
	{
		unsigned int checksum = (data & 0xF);
		checksum += ((data >> 4) & 0xF);
		checksum += ((data >> 8) & 0xF);
		checksum += ((data >> 12) & 0xF);
		checksum += ((data >> 16) & 0xF);
		checksum += ((data >> 20) & 0x01);
		checksum &= 0xF;

		if (checksum != 0xF) {
			LOGP_CHAN(DDSP, LOGL_NOTICE,
				  "RX: FIW checksum failed (0x%X != 0xF).\n", checksum);
			return -1;
		}
	}

	flex->rx.fiw_cycle = (data >> 4) & 0xF;
	flex->rx.fiw_frame = (data >> 8) & 0x7F;

	LOGP_CHAN(DDSP, LOGL_INFO, "RX: FIW decoded — cycle=%u frame=%u.\n",
		  flex->rx.fiw_cycle, flex->rx.fiw_frame);

	return 0;
}

/* Decode one phase of a received frame.
 * De-interleave blocks, BCH decode, parse BIW/addresses/vectors/messages.
 * phaseptr points to 88 words of raw interleaved data. */
static void flex_rx_decode_phase(flex_t *flex, uint32_t *phaseptr, char phase_name)
{
	int32_t decoded[FLEX_WORDS_PER_FRAME];
	int i;

	LOGP_CHAN(DDSP, LOGL_INFO, "RX: Decoding phase %c (C%u/F%u, %d/%d).\n",
		  phase_name, flex->rx.fiw_cycle, flex->rx.fiw_frame,
		  flex->rx.sync_baud, flex->rx.sync_levels);

	/* No separate de-interleave step needed here.
	 * The idx formula in flex_rx_read_data() already de-interleaves
	 * bits into the correct word positions during accumulation
	 * (same approach as multimon-ng read_data).
	 *
	 * Each word has: bits 0-20 = data, bits 21-30 = ECC, bit 31 = parity.
	 * BCH decode strips parity and returns 21-bit data. */
	for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
		int decode_error = flex_bch_decode(phaseptr[i]);
		if (decode_error < 0) {
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "RX: Phase %c word %d BCH uncorrectable (0x%08X).\n",
				  phase_name, i, phaseptr[i]);
			decoded[i] = -1;
		} else {
			decoded[i] = decode_error;
			phaseptr[i] = (uint32_t)decode_error; /* store corrected 21-bit data */
		}
	}

	/* Summary */
	{
		int ok = 0, fail = 0;
		for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
			if (decoded[i] >= 0) ok++; else fail++;
		}
		LOGP_CHAN(DDSP, LOGL_INFO,
			  "RX: Phase %c BCH: %d/%d OK, %d uncorrectable.\n",
			  phase_name, ok, FLEX_WORDS_PER_FRAME, fail);
	}

	/* BIW1 (word 0) */
	if (decoded[0] < 0) {
		LOGP_CHAN(DDSP, LOGL_NOTICE,
			  "RX: Phase %c BIW1 uncorrectable, skipping.\n", phase_name);
		return;
	}

	{
		uint32_t biw = phaseptr[0];

		/* Nothing to decode if BIW is idle */
		if (biw == 0 || biw == 0x001FFFFF) {
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "RX: Phase %c idle frame (BIW=0x%05X).\n",
				  phase_name, biw);
			return;
		}

		/* BIW1 layout (21 bits, per multimon-ng decode_phase):
		 * bits 15-10: voffset (start of vector field)
		 * bits 9-8:   aoffset_raw (+1 = start of address field)
		 * Other bits: priority, carry, collapse */
		int voffset = (biw >> 10) & 0x3F;
		int aoffset = ((biw >> 8) & 0x03) + 1;

		LOGP_CHAN(DDSP, LOGL_DEBUG,
			  "RX: Phase %c BIW=0x%05X aoffset=%d voffset=%d (%d pages).\n",
			  phase_name, biw, aoffset, voffset, voffset - aoffset);

		if (voffset <= aoffset || voffset >= FLEX_WORDS_PER_FRAME) {
			LOGP_CHAN(DDSP, LOGL_NOTICE,
				  "RX: Phase %c invalid BIW offsets.\n", phase_name);
			return;
		}

		/* Walk address/vector pairs */
		for (i = aoffset; i < voffset; i++) {
			int j = voffset + i - aoffset; /* vector index */
			uint64_t capcode;
			int is_long = 0;
			int vec_type, mw1, mw2, len;

			if (j >= FLEX_WORDS_PER_FRAME) break;

			if (decoded[i] < 0 || decoded[j] < 0) {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c addr[%d] or vec[%d] uncorrectable.\n",
					  phase_name, i, j);
				continue;
			}

			/* Address decode (per multimon-ng parse_capcode) */
			{
				uint32_t aw = phaseptr[i];
				is_long = (aw < 0x008001U) ||
					  (aw > 0x1E0000U) ||
					  (aw > 0x1E7FFEU);
				capcode = aw - 0x8000;
			}

			/* Vector decode */
			{
				uint32_t viw = phaseptr[j];
				vec_type = (viw >> 4) & 0x7;
				mw1 = (viw >> 7) & 0x7F;
				len = (viw >> 14) & 0x7F;
				mw2 = mw1 + (len - 1);

				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c cap=%" PRIu64 " type=%d mw1=%d len=%d%s.\n",
					  phase_name, capcode, vec_type, mw1, len,
					  is_long ? " (long)" : "");
			}

			/* Short instruction (group message setup) — skip */
			if (vec_type == FLEX_VECTOR_TYPE_SHORT_INSTR) {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c short instruction, skipping.\n",
					  phase_name);
				continue;
			}

			if (mw1 == 0 && mw2 == 0) continue;

			/* Alpha message */
			if (vec_type == FLEX_VECTOR_TYPE_ALPHA || vec_type == FLEX_VECTOR_TYPE_SECURE) {
				if (mw1 > 87 || mw2 > 87) continue;

				/* Decode header word flags (frame.h has layout) */
				int hdr_k = 0, hdr_c = 0, hdr_f = 0;
				int hdr_n = 0, hdr_r = 0, hdr_m = 0;
				char frag_flag = '?';
				if (decoded[mw1] >= 0) {
					uint32_t hdr = phaseptr[mw1];
					hdr_k = hdr & FLEX_ALPHA_HDR_K_MASK;
					hdr_c = (hdr & FLEX_ALPHA_HDR_C_MASK) >> FLEX_ALPHA_HDR_C_SHIFT;
					hdr_f = (hdr & FLEX_ALPHA_HDR_F_MASK) >> FLEX_ALPHA_HDR_F_SHIFT;
					hdr_n = (hdr & FLEX_ALPHA_HDR_N_MASK) >> FLEX_ALPHA_HDR_N_SHIFT;
					hdr_r = (hdr & FLEX_ALPHA_HDR_R_MASK) >> FLEX_ALPHA_HDR_R_SHIFT;
					hdr_m = (hdr & FLEX_ALPHA_HDR_M_MASK) >> FLEX_ALPHA_HDR_M_SHIFT;

					if (hdr_c == 0 && hdr_f == 3) frag_flag = 'K';
					else if (hdr_c == 0) frag_flag = 'C';
					else frag_flag = 'F';

					LOGP_CHAN(DDSP, LOGL_DEBUG,
						  "RX: Phase %c alpha hdr[%d]=0x%05X: "
						  "checksum=0x%03X continued=%d frag=%d "
						  "msgnum=%d retrieval=%d maildrop=%d "
						  "(%s)\n",
						  phase_name, mw1, hdr,
						  hdr_k, hdr_c, hdr_f,
						  hdr_n, hdr_r, hdr_m,
						  (frag_flag == 'K') ? "complete" :
						  (frag_flag == 'F') ? "continuation" :
						  "final");
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
				 * per ARIB STD-43A Section 3.8.8.3. */
				{
					char text[512];
					int ti = 0, w;
					int start_word = mw1 + 1;
					uint32_t rx_sig = 0;
					uint32_t sig_sum = 0;

					for (w = start_word; w <= mw2 && w < FLEX_WORDS_PER_FRAME; w++) {
						uint32_t dw;
						unsigned char ch;

						if (decoded[w] < 0) continue;
						dw = phaseptr[w];

						/* First slot: signature on initial
						 * fragment, character otherwise */
						if (w == start_word && frag_flag == 'K') {
							rx_sig = dw & FLEX_ALPHA_SIG_MASK;
						} else {
							ch = (dw >> FLEX_ALPHA_CHAR1_SHIFT) & FLEX_ALPHA_CHAR_MASK;
							sig_sum += ch;
							if (ch != FLEX_ALPHA_ETX && ti < (int)sizeof(text) - 1)
								text[ti++] = ch;
						}
						ch = (dw >> FLEX_ALPHA_CHAR2_SHIFT) & FLEX_ALPHA_CHAR_MASK;
						sig_sum += ch;
						if (ch != FLEX_ALPHA_ETX && ti < (int)sizeof(text) - 1)
							text[ti++] = ch;
						ch = (dw >> FLEX_ALPHA_CHAR3_SHIFT) & FLEX_ALPHA_CHAR_MASK;
						sig_sum += ch;
						if (ch != FLEX_ALPHA_ETX && ti < (int)sizeof(text) - 1)
							text[ti++] = ch;
					}
					text[ti] = '\0';

					/* Verify signature on initial fragment */
					if (frag_flag == 'K') {
						uint32_t expected_sig = (~sig_sum) & FLEX_ALPHA_SIG_MASK;
						if (rx_sig == expected_sig) {
							LOGP_CHAN(DDSP, LOGL_DEBUG,
								  "RX: Phase %c signature OK (0x%02X).\n",
								  phase_name, rx_sig);
						} else {
							LOGP_CHAN(DDSP, LOGL_NOTICE,
								  "RX: Phase %c signature MISMATCH: "
								  "received=0x%02X computed=0x%02X.\n",
								  phase_name, rx_sig, expected_sig);
						}
					}

					LOGP_CHAN(DDSP, LOGL_NOTICE,
						  "RX: %d/%d/%c/%c %02u.%03u [%09" PRIu64 "] ALN %s\n",
						  flex->rx.sync_baud, flex->rx.sync_levels,
						  frag_flag, phase_name,
						  flex->rx.fiw_cycle, flex->rx.fiw_frame,
						  capcode, text);
				}
			} else if (vec_type == FLEX_VECTOR_TYPE_NUMERIC) {
				/* Numeric */
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "RX: %d/%d/%c %02u.%03u [%09" PRIu64 "] NUM\n",
					  flex->rx.sync_baud, flex->rx.sync_levels,
					  phase_name,
					  flex->rx.fiw_cycle, flex->rx.fiw_frame,
					  capcode);
			} else if (vec_type == FLEX_VECTOR_TYPE_TONE) {
				/* Tone-only */
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "RX: %d/%d/%c %02u.%03u [%09" PRIu64 "] TON\n",
					  flex->rx.sync_baud, flex->rx.sync_levels,
					  phase_name,
					  flex->rx.fiw_cycle, flex->rx.fiw_frame,
					  capcode);
			} else {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Phase %c unknown type %d from %" PRIu64 ".\n",
					  phase_name, vec_type, capcode);
			}
		}
	}
}

/* Clear all phase data buffers before reading a new frame. */
static void flex_rx_clear_phase_data(flex_t *flex)
{
	memset(flex->rx.phase_a, 0, sizeof(flex->rx.phase_a));
	memset(flex->rx.phase_b, 0, sizeof(flex->rx.phase_b));
	memset(flex->rx.phase_c, 0, sizeof(flex->rx.phase_c));
	memset(flex->rx.phase_d, 0, sizeof(flex->rx.phase_d));
	flex->rx.phase_a_idle = 0;
	flex->rx.phase_b_idle = 0;
	flex->rx.phase_c_idle = 0;
	flex->rx.phase_d_idle = 0;
	flex->rx.phase_toggle = 0;
	flex->rx.data_bit_counter = 0;
}

/* Decode all active phases after data reading is complete.
 * Which phases are active depends on baud/levels:
 *   1600/2: A only
 *   1600/4: A, B (simultaneous via 4-level)
 *   3200/2: A, C (interleaved via toggle)
 *   3200/4: A, B, C, D (interleaved + simultaneous) */
static void flex_rx_decode_data(flex_t *flex)
{
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
		flex_rx_decode_phase(flex, flex->rx.phase_a, 'A');
		flex_rx_decode_phase(flex, flex->rx.phase_b, 'B');
		flex_rx_decode_phase(flex, flex->rx.phase_c, 'C');
		flex_rx_decode_phase(flex, flex->rx.phase_d, 'D');

		/* Hex dump: per-phase, then full frame as contiguous block */
		{
			uint32_t *pptrs[4] = {
				flex->rx.phase_a, flex->rx.phase_b,
				flex->rx.phase_c, flex->rx.phase_d,
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
	 *   A3 (3200bps/4FSK): 1600 baud, 4 levels → phases A, B
	 *     (4FSK dibit: MSB=phase A, LSB=phase B)
	 *   A2 (3200bps/2FSK): 3200 baud, 2 levels → phases A, C
	 *     (alternating symbols: even=A, odd=C)
	 *   A4 (6400bps/4FSK): 3200 baud, 4 levels → phases A, B, C, D
	 *     (alternating dibit pairs: even sym MSB=A/LSB=B, odd sym MSB=C/LSB=D) */
	if (flex->rx.sync_baud == 1600) {
		if (flex->rx.sync_levels == 2) {
			flex_rx_decode_phase(flex, flex->rx.phase_a, 'A');
		} else {
			flex_rx_decode_phase(flex, flex->rx.phase_a, 'A');
			flex_rx_decode_phase(flex, flex->rx.phase_b, 'B');
		}
	} else {
		if (flex->rx.sync_levels == 2) {
			flex_rx_decode_phase(flex, flex->rx.phase_a, 'A');
			flex_rx_decode_phase(flex, flex->rx.phase_c, 'C');
		} else {
			flex_rx_decode_phase(flex, flex->rx.phase_a, 'A');
			flex_rx_decode_phase(flex, flex->rx.phase_b, 'B');
			flex_rx_decode_phase(flex, flex->rx.phase_c, 'C');
			flex_rx_decode_phase(flex, flex->rx.phase_d, 'D');
		}
	}
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
 *   toggle=0: bit_a → Phase A, bit_b → Phase B
 *   toggle=1: bit_a → Phase C, bit_b → Phase D
 *
 * At 1600 baud: toggle is always 0 (no interleaving).
 * At 3200 baud: toggle alternates 0/1 on each symbol.
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
		flex->rx.phase_a[idx] = (flex->rx.phase_a[idx] >> 1) |
					(bit_a ? 0x80000000U : 0);
		flex->rx.phase_b[idx] = (flex->rx.phase_b[idx] >> 1) |
					(bit_b ? 0x80000000U : 0);
		flex->rx.phase_toggle = 1;

		/* Track idle words for early termination */
		if ((flex->rx.data_bit_counter & 0xFF) == 0xFF) {
			if (flex->rx.phase_a[idx] == 0x00000000 ||
			    flex->rx.phase_a[idx] == 0xFFFFFFFF)
				flex->rx.phase_a_idle++;
			if (flex->rx.phase_b[idx] == 0x00000000 ||
			    flex->rx.phase_b[idx] == 0xFFFFFFFF)
				flex->rx.phase_b_idle++;
		}
	} else {
		flex->rx.phase_c[idx] = (flex->rx.phase_c[idx] >> 1) |
					(bit_a ? 0x80000000U : 0);
		flex->rx.phase_d[idx] = (flex->rx.phase_d[idx] >> 1) |
					(bit_b ? 0x80000000U : 0);
		flex->rx.phase_toggle = 0;

		if ((flex->rx.data_bit_counter & 0xFF) == 0xFF) {
			if (flex->rx.phase_c[idx] == 0x00000000 ||
			    flex->rx.phase_c[idx] == 0xFFFFFFFF)
				flex->rx.phase_c_idle++;
			if (flex->rx.phase_d[idx] == 0x00000000 ||
			    flex->rx.phase_d[idx] == 0xFFFFFFFF)
				flex->rx.phase_d_idle++;
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
		/* Feed 2-level bit into 64-bit sync shift register.
		 * S1 is always at 1600 baud / 2FSK (1 bit per symbol).
		 * sym < 2 → bit 1, sym >= 2 → bit 0 (per multimon-ng flex_sync) */
		flex->rx.sync_buf = (flex->rx.sync_buf << 1) |
				    ((sym < 2) ? 1 : 0);

		unsigned int sync_code;
		int polarity;
		sync_code = flex_rx_sync_detect(flex, flex->rx.sync_buf, &polarity);
		if (sync_code != 0) {
			flex->rx.polarity = polarity;
			if (flex_rx_decode_mode(flex, sync_code)) {
				flex->rx.rx_state = RX_STATE_FIW;
				flex->rx.fiw_count = 0;
				flex->rx.fiw_rawdata = 0;
			}
		}
		break;
	}

	case RX_STATE_FIW:
	{
		/* FIW: always at 1600 baud / 2FSK (1 bit per symbol).
		 * 16 bits of dotting (PLL settling), then 32 bits of FIW data.
		 * Total = 48 bits = 48 symbols at 1600/2FSK. */
		flex->rx.fiw_count++;
		if (flex->rx.fiw_count > 16) {
			/* Accumulate FIW data bits LSB-first (sym > 1 → bit 1).
			 * FIW is a 32-bit BCH codeword at 1600/2FSK. */
			flex->rx.fiw_rawdata = (flex->rx.fiw_rawdata >> 1) |
					       ((sym_rect > 1) ? 0x80000000U : 0);
		}

		if (flex->rx.fiw_count == 48) {
			if (flex_rx_decode_fiw(flex, flex->rx.fiw_rawdata) == 0) {
				/* FIW OK — switch to data symbol rate and enter S2.
				 * sync_baud is the SYMBOL rate (baud), not bit rate.
				 * PLL now tracks symbols at the data rate. */
				flex->rx.baud = flex->rx.sync_baud;
				flex->rx.sync2_count = 0;
				flex->rx.rx_state = RX_STATE_SYNC2;
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: FIW→SYNC2, symbol rate switch to %d baud.\n",
					  flex->rx.baud);
			} else {
				flex->rx.rx_state = RX_STATE_SYNC1;
			}
		}
		break;
	}

	case RX_STATE_SYNC2:
	{
		/* S2: 25 ms at the data symbol rate (ARIB STD-43A Section 3.2).
		 * S2 = BS2 + C(0xED84) + inv.BS2 + inv.C(0x127B).
		 *
		 * sync_baud is the SYMBOL rate (baud), so:
		 *   sync_baud * 25 / 1000 = number of SYMBOLS in S2.
		 *
		 * Per standard Section 3.2 (S2 description):
		 *   1600bps/2FSK (A1): 1600 baud → 40 symbols (= 40 bits)
		 *   3200bps/2FSK (A2): 3200 baud → 80 symbols (= 80 bits)
		 *   3200bps/4FSK (A3): 1600 baud → 40 symbols (= 80 bits)
		 *   6400bps/4FSK (A4): 3200 baud → 80 symbols (= 160 bits)
		 *
		 * S2 component breakdown (in symbols):
		 *   Mode        BS2   C    inv.BS2  inv.C  Total
		 *   A1 (1600/2) :  4 + 16 +   4   + 16   = 40 sym
		 *   A2 (3200/2) : 24 + 16 +  24   + 16   = 80 sym
		 *   A3 (3200/4) :  6 +  8 +   6   +  8   = 28 sym (*)
		 *   A4 (6400/4) : 32 +  8 +  32   +  8   = 80 sym
		 *
		 * (*) A3 note: standard Table 3.2-3 shows BS2 as 12 symbols
		 *     of alternating comma pattern "101010101010" which is
		 *     12 4-level symbols. C is 16 decoded bits = 8 symbols.
		 *     Total = 12 + 8 + 12 + 8 = 40 symbols.
		 *
		 * TODO: Replace blind skip with C pattern detection for
		 * precise block boundary timing (see BCH_REWRITE_PLAN.md). */
		int s2_symbols = flex->rx.sync_baud * 25 / 1000;
		if (++flex->rx.sync2_count == s2_symbols) {
			flex->rx.data_count = 0;
			flex_rx_clear_phase_data(flex);
			flex->rx.rx_state = RX_STATE_DATA;
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "RX: SYNC2→DATA, skipped %d S2 symbols (%d baud, %dFSK).\n",
				  flex->rx.sync2_count,
				  flex->rx.sync_baud,
				  flex->rx.sync_levels);
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
		/* Not locked: reset envelope and hold in SYNC1 */
		flex->rx.pll_envelope = 0;
		flex->rx.pll_envelope_sum = 0;
		flex->rx.pll_envelope_count = 0;
		flex->rx.baud = 1600;
		flex->rx.pll_timeout = 0;
		flex->rx.pll_nonconsec = 0;
		flex->rx.rx_state = RX_STATE_SYNC1;
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
		/* Check for lock pattern (alternating 0/3 symbols = dotting).
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
