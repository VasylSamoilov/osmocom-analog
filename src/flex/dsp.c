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

/* Switch baud rate for the next frame.
 * Updates fsk_bitduration and fsk_bitstep for the selected speed.
 * Valid speeds: 1600, 3200 (2-FSK), 6400 (4-FSK, handled separately). */
void dsp_set_speed(flex_t *flex, int baud_rate, int modulation_type)
{
	/* FLEX baud rate = bit rate.
	 * 2FSK: 1 bit/symbol → symbol rate = baud rate.
	 * 4FSK: 2 bits/symbol → symbol rate = baud rate / 2.
	 * e.g. 3200/4FSK = 1600 symbols/sec at 4 levels. */
	int symbol_rate = baud_rate;
	if (modulation_type == FLEX_MOD_4FSK)
		symbol_rate = baud_rate / 2;
	flex->fsk_bitduration = (double)flex->sender.samplerate / (double)symbol_rate;
	flex->fsk_bitstep = 1.0 / flex->fsk_bitduration;
	flex->current_frame_speed = baud_rate;
	flex->current_frame_mod_type = modulation_type;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "DSP speed set to %d baud (symbol rate %d, %.4f samples/symbol).\n",
		  baud_rate, symbol_rate, flex->fsk_bitduration);
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

/* 4-FSK encoder for 6400 bps mode.
 * Encodes 2 bits per symbol at 3200 baud (16 symbols per 32-bit word).
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
				 * first symbol at the new baud rate is full-length.
				 * Without this, the residual phase from the last
				 * 1600-baud symbol shortens the first higher-baud
				 * symbol, causing the demodulator PLL to miscount
				 * S2 symbols by one. */
				flex->fsk_tx_phase = 0.0;
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

/* ===== FLEX Receiver ===== */

/* RX state machine states */
enum flex_rx_state {
	RX_HUNT_SYNC = 0,	/* hunting for A1 sync pattern */
	RX_SKIP_S1_TAIL,	/* skipping B(16) + A1_inv(32) after A1 detected */
	RX_READ_FIW,		/* reading 32-bit FIW codeword */
	RX_SKIP_S2,		/* skipping C block (40 bits) */
	RX_READ_FRAME,		/* reading 88 × 32-bit data words */
};

/* A sync patterns as 32-bit values (MSB first), from Table 3.2-5 */
#define FLEX_SYNC_A1		0x78F35939U	/* 1600/2FSK */
#define FLEX_SYNC_A1_INV	0x870CA6C6U
#define FLEX_SYNC_A2		0x84E75939U	/* 3200/2FSK */
#define FLEX_SYNC_A2_INV	0x7B18A6C6U
#define FLEX_SYNC_A3		0x4F975939U	/* 3200/4FSK */
#define FLEX_SYNC_A3_INV	0xB068A6C6U
#define FLEX_SYNC_A4		0x215F5939U	/* 6400/4FSK */
#define FLEX_SYNC_A4_INV	0xDEA0A6C6U
#define FLEX_SYNC_A5		0xDD4B5939U	/* Reserved */
#define FLEX_SYNC_A6		0x163B5939U	/* Reserved */
#define FLEX_SYNC_A7		0xB3835939U	/* Reserved */
#define FLEX_SYNC_A8		0x63415939U	/* Reserved */
#define FLEX_SYNC_A9		0x1BE25939U	/* Reserved */
#define FLEX_SYNC_A10		0x2C865939U	/* Reserved */
#define FLEX_SYNC_A11		0xA5E85939U	/* Reserved */
#define FLEX_SYNC_A12		0x928C5939U	/* Reserved */
#define FLEX_SYNC_A13		0x6E985939U	/* Reserved */
#define FLEX_SYNC_A14		0xBE5A5939U	/* Reserved */
#define FLEX_SYNC_A15		0xF19D5939U	/* Reserved */
#define FLEX_SYNC_AR		0xCB205939U	/* Re-synchronization */
#define FLEX_SYNC_AR_INV	0x34DFA6C6U

/* RX detected mode */
enum flex_rx_mode {
	RX_MODE_A1 = 0,
	RX_MODE_A3 = 1,
};

/* S1 tail after A1: B(16 bits) + A1_inv(32 bits) = 48 bits */
#define RX_S1_TAIL_BITS		48
/* S2: C block = 40 bits */
#define RX_S2_BITS		40
/* FIW: 1 codeword = 32 bits */
#define RX_FIW_BITS		32

/* BCH(31,21) syndrome-based decoder.
 * Returns corrected 21-bit data word, or -1 if uncorrectable (>2 errors). */
static int32_t flex_bch_decode(uint32_t codeword)
{
	uint32_t data21, ecc10, syndrome;
	uint32_t gen = FLEX_BCH_POLY; /* 0x769 */
	int i, parity;

	/* Check overall parity (bit 0) */
	parity = 0;
	{
		uint32_t tmp = codeword;
		while (tmp) {
			parity ^= (tmp & 1);
			tmp >>= 1;
		}
	}

	/* Extract data (bits 31-11) and received ECC (bits 10-1) */
	data21 = (codeword >> 11) & 0x1FFFFF;
	ecc10 = (codeword >> 1) & 0x3FF;

	/* Compute syndrome: divide data by generator, XOR remainder with received ECC */
	{
		uint32_t dividend = data21;
		for (i = 20; i >= 10; i--) {
			if ((dividend >> i) & 1)
				dividend ^= gen << (i - 10);
		}
		syndrome = (dividend & 0x3FF) ^ ecc10;
	}

	if (syndrome == 0 && parity == 0)
		return (int32_t)data21; /* no errors */

	if (syndrome == 0 && parity == 1)
		return (int32_t)data21; /* parity bit error only, data correct */

	/* Try single-bit error correction in data field */
	for (i = 0; i < 21; i++) {
		uint32_t test = data21 ^ (1U << (20 - i));
		uint32_t div = test;
		int j;
		for (j = 20; j >= 10; j--) {
			if ((div >> j) & 1)
				div ^= gen << (j - 10);
		}
		if ((div & 0x3FF) == ecc10)
			return (int32_t)test;
	}

	/* Try single-bit error in ECC field */
	for (i = 0; i < 10; i++) {
		uint32_t test_ecc = ecc10 ^ (1U << (9 - i));
		uint32_t div = data21;
		int j;
		for (j = 20; j >= 10; j--) {
			if ((div >> j) & 1)
				div ^= gen << (j - 10);
		}
		if ((div & 0x3FF) == test_ecc)
			return (int32_t)data21; /* error was in ECC, data is fine */
	}

	/* Try double-bit error correction (brute force over data bits) */
	{
		int a, b;
		for (a = 0; a < 21; a++) {
			for (b = a + 1; b < 21; b++) {
				uint32_t test = data21 ^ (1U << (20 - a)) ^ (1U << (20 - b));
				uint32_t div = test;
				int j;
				for (j = 20; j >= 10; j--) {
					if ((div >> j) & 1)
						div ^= gen << (j - 10);
				}
				if ((div & 0x3FF) == ecc10)
					return (int32_t)test;
			}
		}
	}

	/* Uncorrectable */
	return -1;
}

/* De-interleave one block of 8 × 32-bit words.
 * Reverses the column-wise interleaving done by flex_interleave_block().
 *
 * The TX interleaver maps:
 *   dst_byte[i] bit (7-w) = src_word[w] bit (31-i)
 *   for i=0..31, w=0..7
 *
 * So de-interleave is:
 *   word[w] bit (31-i) = interleaved_byte[i] bit (7-w) */
static void flex_deinterleave_block(uint32_t *words)
{
	uint8_t src[FLEX_CODEWORD_BITS]; /* 32 bytes of interleaved data */
	uint32_t out[FLEX_WORDS_PER_BLOCK];
	int i, w;

	/* The interleaved data is stored as raw bytes over the word array */
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

/* Decode a received FIW codeword to extract cycle and frame numbers.
 * Returns 0 on success, -1 on decode failure. */
static int flex_rx_decode_fiw(flex_t *flex, uint32_t fiw_raw)
{
	int32_t data;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: FIW raw codeword = 0x%08X\n", fiw_raw);

	data = flex_bch_decode(fiw_raw);
	if (data < 0) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "RX: FIW BCH decode failed (uncorrectable).\n");
		return -1;
	}

	/* FIW bit layout (21 data bits, MSB first):
	 * bits 20-17: cycle (4 bits, 0-14)
	 * bits 16-10: frame (7 bits, 0-127)
	 * bits 9-2:   reserved/flags
	 * bits 1-0:   n (roaming), r (repeat) */
	flex->rx.fiw_cycle = (data >> 17) & 0xF;
	flex->rx.fiw_frame = (data >> 10) & 0x7F;

	LOGP_CHAN(DDSP, LOGL_INFO, "RX: FIW decoded — cycle=%u frame=%u.\n",
		  flex->rx.fiw_cycle, flex->rx.fiw_frame);

	return 0;
}

/* Process a complete received frame (88 words).
 * De-interleave, BCH decode, extract addresses/vectors/messages. */
static void flex_rx_process_frame(flex_t *flex)
{
	uint32_t *words = flex->rx.frame_words;
	int32_t decoded[FLEX_WORDS_PER_FRAME];
	int i, biw1_data, e_biw, s_vfield;
	int addr_start, vec_start;

	LOGP_CHAN(DDSP, LOGL_INFO, "RX: Processing frame C%u/F%u (%d words).\n",
		  flex->rx.fiw_cycle, flex->rx.fiw_frame, flex->rx.word_count);

	/* Dump raw frame words (pre-deinterleave) for bitstream debugging.
	 * This lets you compare TX output against what the RX actually received. */
	{
		int d;
		LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: === Raw frame dump (pre-deinterleave) ===\n");
		for (d = 0; d < flex->rx.word_count; d++) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: word[%02d] = 0x%08X\n",
				  d, words[d]);
		}
		LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: === End raw frame dump ===\n");
	}

	/* De-interleave each block of 8 words */
	{
		int b;
		for (b = 0; b < FLEX_BLOCKS_PER_FRAME; b++)
			flex_deinterleave_block(words + b * FLEX_WORDS_PER_BLOCK);
	}

	/* BCH decode all 88 words */
	for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
		decoded[i] = flex_bch_decode(words[i]);
		if (decoded[i] < 0) {
			LOGP_CHAN(DDSP, LOGL_DEBUG,
				  "RX: Word %d BCH uncorrectable (0x%08X).\n",
				  i, words[i]);
		}
	}

	/* Post-BCH decode summary for debugging transmission issues */
	{
		int ok = 0, fail = 0;
		for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
			if (decoded[i] >= 0)
				ok++;
			else
				fail++;
		}
		LOGP_CHAN(DDSP, LOGL_INFO,
			  "RX: BCH decode: %d/%d words OK, %d uncorrectable.\n",
			  ok, FLEX_WORDS_PER_FRAME, fail);
	}

	/* Decode BIW1 (word 0) */
	if (decoded[0] < 0) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "RX: BIW1 uncorrectable, skipping frame.\n");
		return;
	}

	biw1_data = decoded[0];
	/* BIW1 layout (21 bits):
	 * bits 20-17: priority address count (prio)
	 * bits 16-13: end of BIW field (e_biw)
	 * bits 12-6:  start of vector field (s_vfield)
	 * bits 5-3:   carry
	 * bits 2-0:   collapse */
	e_biw = (biw1_data >> 13) & 0xF;
	s_vfield = (biw1_data >> 6) & 0x7F;

	addr_start = e_biw + 1;
	vec_start = s_vfield;

	if (vec_start <= addr_start || vec_start >= FLEX_WORDS_PER_FRAME) {
		LOGP_CHAN(DDSP, LOGL_NOTICE,
			  "RX: Invalid BIW1 offsets (e_biw=%d, s_vfield=%d).\n",
			  e_biw, s_vfield);
		return;
	}

	LOGP_CHAN(DDSP, LOGL_DEBUG,
		  "RX: BIW1: e_biw=%d s_vfield=%d, addresses at %d-%d, vectors at %d+.\n",
		  e_biw, s_vfield, addr_start, vec_start - 1, vec_start);

	/* Walk address/vector pairs and extract messages */
	{
		int addr_idx = addr_start;
		int vec_idx = vec_start;

		while (addr_idx < vec_start && vec_idx < FLEX_WORDS_PER_FRAME) {
			uint32_t addr_word, vec_word;
			uint64_t capcode;
			int vec_type, msg_word_start, msg_word_count;
			int is_long = 0;

			if (decoded[addr_idx] < 0) {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Address word %d uncorrectable.\n", addr_idx);
				addr_idx++;
				vec_idx++;
				continue;
			}
			if (decoded[vec_idx] < 0) {
				LOGP_CHAN(DDSP, LOGL_DEBUG,
					  "RX: Vector word %d uncorrectable.\n", vec_idx);
				addr_idx++;
				vec_idx++;
				continue;
			}

			addr_word = (uint32_t)decoded[addr_idx];
			vec_word = (uint32_t)decoded[vec_idx];

			/* Decode capcode from address word */
			if (addr_word < FLEX_SHORT_ADDR_OFFSET) {
				/* Could be a long address — check next word */
				if (addr_idx + 1 < vec_start && decoded[addr_idx + 1] >= 0) {
					is_long = 1;
					/* Long address decode (simplified) */
					capcode = ((uint64_t)(addr_word) << 21) |
						  (uint64_t)(uint32_t)decoded[addr_idx + 1];
					addr_idx += 2;
				} else {
					capcode = addr_word;
					addr_idx++;
				}
			} else {
				/* Short address: subtract offset */
				capcode = addr_word - FLEX_SHORT_ADDR_OFFSET;
				addr_idx++;
			}

			/* Decode vector word */
			vec_type = (vec_word >> 4) & 0x7; /* bits 6-4: type */

			/* Extract message start and word count from vector */
			msg_word_start = (vec_word >> 7) & 0x7F; /* bits 13-7 */
			msg_word_count = vec_word & 0xF;          /* bits 3-0 (approx) */

			LOGP_CHAN(DDSP, LOGL_INFO,
				  "RX: Message — capcode=%" PRIu64 " type=%d start=%d words=%d%s.\n",
				  capcode, vec_type, msg_word_start, msg_word_count,
				  is_long ? " (long)" : "");

			/* Decode message content based on vector type */
			if (vec_type == 0x5) {
				/* Alpha message: 3 chars per 21-bit word */
				char text[256];
				int ti = 0, w;

				for (w = msg_word_start;
				     w < msg_word_start + msg_word_count && w < FLEX_WORDS_PER_FRAME;
				     w++) {
					if (decoded[w] < 0)
						continue;
					uint32_t d = (uint32_t)decoded[w];
					/* 3 × 7-bit chars packed into 21 bits */
					if (ti < (int)sizeof(text) - 1)
						text[ti++] = (d >> 14) & 0x7F;
					if (ti < (int)sizeof(text) - 1)
						text[ti++] = (d >> 7) & 0x7F;
					if (ti < (int)sizeof(text) - 1)
						text[ti++] = d & 0x7F;
				}
				text[ti] = '\0';

				LOGP_CHAN(DDSP, LOGL_INFO,
					  "RX: Alpha message from %" PRIu64 ": \"%s\"\n",
					  capcode, text);
			} else if (vec_type == 0x3) {
				/* Numeric message */
				LOGP_CHAN(DDSP, LOGL_INFO,
					  "RX: Numeric message from %" PRIu64 " (%d words).\n",
					  capcode, msg_word_count);
			} else if (vec_type == 0x2) {
				/* Tone-only */
				LOGP_CHAN(DDSP, LOGL_INFO,
					  "RX: Tone-only alert for %" PRIu64 ".\n",
					  capcode);
			} else {
				LOGP_CHAN(DDSP, LOGL_INFO,
					  "RX: Unknown vector type %d from %" PRIu64 ".\n",
					  vec_type, capcode);
			}

			vec_idx++;
		}
	}
}

/* Feed one demodulated bit into the RX state machine. */
static void flex_rx_bit(flex_t *flex, uint8_t bit)
{
	flex->rx.shift_reg = (flex->rx.shift_reg << 1) | (bit & 1);

	/* Bitstream trace: log every bit during active frame reception.
	 * Use LOGL_DEBUG so it only appears with -v -v or higher. */
	if (flex->rx.rx_state == RX_READ_FRAME || flex->rx.rx_state == RX_READ_FIW) {
		LOGP_CHAN(DDSP, LOGL_DEBUG, "RX BIT: state=%d bit=%u shift=0x%08X bc=%d wc=%d\n",
			  flex->rx.rx_state, bit, flex->rx.shift_reg,
			  flex->rx.bit_count, flex->rx.word_count);
	}

	switch (flex->rx.rx_state) {
	case RX_HUNT_SYNC:
		/* Look for A sync patterns (32 bits) — Table 3.2-5 */
		if (flex->rx.shift_reg == FLEX_SYNC_A1) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: A1 sync detected (1600/2FSK).\n");
			flex->rx.rx_mode = RX_MODE_A1;
			flex->rx.rx_state = RX_SKIP_S1_TAIL;
			flex->rx.bit_count = 0;
		} else if (flex->rx.shift_reg == FLEX_SYNC_A1_INV) {
			LOGP_CHAN(DDSP, LOGL_NOTICE,
				  "RX: Inverted A1 sync detected — wrong polarity?\n");
		} else if (flex->rx.shift_reg == FLEX_SYNC_A2) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: A2 sync detected (3200/2FSK).\n");
			flex->rx.rx_mode = RX_MODE_A1; /* TODO: add RX_MODE_A2 */
			flex->rx.rx_state = RX_SKIP_S1_TAIL;
			flex->rx.bit_count = 0;
		} else if (flex->rx.shift_reg == FLEX_SYNC_A3) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: A3 sync detected (3200/4FSK).\n");
			flex->rx.rx_mode = RX_MODE_A3;
			flex->rx.rx_state = RX_SKIP_S1_TAIL;
			flex->rx.bit_count = 0;
		} else if (flex->rx.shift_reg == FLEX_SYNC_A3_INV) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: Inverted A3 sync detected (3200/4FSK).\n");
			flex->rx.rx_mode = RX_MODE_A3;
			flex->rx.rx_state = RX_SKIP_S1_TAIL;
			flex->rx.bit_count = 0;
		} else if (flex->rx.shift_reg == FLEX_SYNC_A4) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: A4 sync detected (6400/4FSK).\n");
			flex->rx.rx_mode = RX_MODE_A3; /* TODO: add RX_MODE_A4 */
			flex->rx.rx_state = RX_SKIP_S1_TAIL;
			flex->rx.bit_count = 0;
		} else if (flex->rx.shift_reg == FLEX_SYNC_A5
			|| flex->rx.shift_reg == FLEX_SYNC_A6
			|| flex->rx.shift_reg == FLEX_SYNC_A7
			|| flex->rx.shift_reg == FLEX_SYNC_A8
			|| flex->rx.shift_reg == FLEX_SYNC_A9
			|| flex->rx.shift_reg == FLEX_SYNC_A10
			|| flex->rx.shift_reg == FLEX_SYNC_A11
			|| flex->rx.shift_reg == FLEX_SYNC_A12
			|| flex->rx.shift_reg == FLEX_SYNC_A13
			|| flex->rx.shift_reg == FLEX_SYNC_A14
			|| flex->rx.shift_reg == FLEX_SYNC_A15) {
			LOGP_CHAN(DDSP, LOGL_NOTICE,
				  "RX: Reserved A sync detected (0x%08X) — unsupported frame speed.\n",
				  flex->rx.shift_reg);
		} else if (flex->rx.shift_reg == FLEX_SYNC_AR) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: Ar sync detected (re-synchronization).\n");
		} else if (flex->rx.shift_reg == FLEX_SYNC_AR_INV) {
			LOGP_CHAN(DDSP, LOGL_DEBUG, "RX: Inverted Ar sync detected (re-synchronization).\n");
		}
		break;

	case RX_SKIP_S1_TAIL:
		/* Skip B(16) + A1_inv(32) = 48 bits */
		if (++flex->rx.bit_count >= RX_S1_TAIL_BITS) {
			flex->rx.rx_state = RX_READ_FIW;
			flex->rx.bit_count = 0;
			flex->rx.shift_reg = 0;
		}
		break;

	case RX_READ_FIW:
		/* Accumulate 32 bits for FIW */
		if (++flex->rx.bit_count >= RX_FIW_BITS) {
			if (flex_rx_decode_fiw(flex, flex->rx.shift_reg) < 0) {
				/* FIW decode failed — go back to hunting */
				flex->rx.rx_state = RX_HUNT_SYNC;
				break;
			}
			flex->rx.rx_state = RX_SKIP_S2;
			flex->rx.bit_count = 0;
		}
		break;

	case RX_SKIP_S2:
		/* Skip C block (40 bits) */
		if (++flex->rx.bit_count >= RX_S2_BITS) {
			flex->rx.rx_state = RX_READ_FRAME;
			flex->rx.bit_count = 0;
			flex->rx.word_count = 0;
			flex->rx.shift_reg = 0;
		}
		break;

	case RX_READ_FRAME:
		/* Accumulate 32 bits per word */
		flex->rx.bit_count++;
		if (flex->rx.bit_count >= 32) {
			flex->rx.frame_words[flex->rx.word_count] = flex->rx.shift_reg;
			flex->rx.word_count++;
			flex->rx.bit_count = 0;
			flex->rx.shift_reg = 0;

			if (flex->rx.rx_mode == RX_MODE_A3) {
				/* A3 (3200/4FSK): 2 phases (A, B) packed
				 * into 4-level symbols.  The demodulator
				 * delivers dibits as 2-bit values; at the
				 * bit level we receive 88 words of phase A
				 * interleaved with 88 words of phase B.
				 * Total: 176 words (5632 bits = 2816 dibits).
				 */
				if (flex->rx.word_count >= FLEX_WORDS_PER_FRAME * 2) {
					/* De-interleave: even words → phase A, odd → phase B */
					uint32_t phase_a[FLEX_WORDS_PER_FRAME];
					uint32_t phase_b[FLEX_WORDS_PER_FRAME];
					int i;

					for (i = 0; i < FLEX_WORDS_PER_FRAME; i++) {
						phase_a[i] = flex->rx.frame_words[2 * i];
						phase_b[i] = flex->rx.frame_words[2 * i + 1];
					}

					/* Process phase A */
					memcpy(flex->rx.frame_words, phase_a,
					       FLEX_WORDS_PER_FRAME * sizeof(uint32_t));
					flex->rx.word_count = FLEX_WORDS_PER_FRAME;
					flex_rx_process_frame(flex);

					/* Process phase B */
					memcpy(flex->rx.frame_words, phase_b,
					       FLEX_WORDS_PER_FRAME * sizeof(uint32_t));
					flex->rx.word_count = FLEX_WORDS_PER_FRAME;
					flex_rx_process_frame(flex);

					/* Return to hunting for next frame */
					flex->rx.rx_state = RX_HUNT_SYNC;
				}
			} else {
				/* A1/A2: 88 words total */
				if (flex->rx.word_count >= FLEX_WORDS_PER_FRAME) {
					/* Complete frame received — process it */
					flex_rx_process_frame(flex);
					/* Return to hunting for next frame */
					flex->rx.rx_state = RX_HUNT_SYNC;
				}
			}
		}
		break;
	}
}

/* 2-FSK demodulator: convert audio samples to bits.
 * Same zero-crossing approach as the POCSAG receiver. */
static void flex_fsk_demod(flex_t *flex, sample_t *samples, int length)
{
	double phase, bitstep, polarity;
	uint8_t lastbit;
	int i;

	polarity = flex->fsk_polarity;
	phase = flex->rx.fsk_rx_phase;
	lastbit = flex->rx.fsk_rx_lastbit;
	bitstep = flex->fsk_bitstep;

	for (i = 0; i < length; i++) {
		if (samples[i] * polarity > 0.0) {
			if (lastbit) {
				/* stay high */
				phase += bitstep;
				if (phase >= 1.0) {
					phase -= 1.0;
					flex_rx_bit(flex, 1);
				}
			} else {
				/* transition low→high */
				phase = -0.5;
				flex_rx_bit(flex, 1);
				lastbit = 1;
			}
		} else {
			if (lastbit) {
				/* transition high→low */
				phase = -0.5;
				flex_rx_bit(flex, 0);
				lastbit = 0;
			} else {
				/* stay low */
				phase += bitstep;
				if (phase >= 1.0) {
					phase -= 1.0;
					flex_rx_bit(flex, 0);
				}
			}
		}
	}

	flex->rx.fsk_rx_phase = phase;
	flex->rx.fsk_rx_lastbit = lastbit;
}

/* Process received audio stream from radio unit. */
void sender_receive(sender_t *sender, sample_t *samples, int length, double __attribute__((unused)) rf_level_db)
{
	flex_t *flex = (flex_t *)sender;

	if (flex->rx.enabled)
		flex_fsk_demod(flex, samples, length);
}
