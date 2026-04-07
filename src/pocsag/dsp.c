/* POCSAG signal processing
 *
 * (C) 2019 by Andreas Eversberg <jolly@eversberg.eu>
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

#define CHAN pocsag->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "pocsag.h"
#include "frame.h"
#include "dsp.h"

#define CODEWORD_SYNC	0x7cd215d8

#define MAX_DISPLAY	1.4	/* something above speech level, no emphasis */

/*
 * Count number of differing bits (hamming distance) between two 32-bit words.
 */
static inline int hamming32(uint32_t a, uint32_t b)
{
	uint32_t x = a ^ b;
	/* popcount via bit manipulation */
	x = x - ((x >> 1) & 0x55555555u);
	x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
	x = (x + (x >> 4)) & 0x0f0f0f0fu;
	return (int)((x * 0x01010101u) >> 24);
}

/*
 * Maximum hamming distance for sync word detection.
 * BCH(31,21) can correct up to 2-bit errors, so we accept sync words
 * with up to 2 bit differences. This matches the correction capability
 * without introducing false positives (probability of a random 32-bit
 * word being within hamming distance 2 of SYNC is ~529/2^32 ≈ 1.2e-7).
 */
#define SYNC_MAX_HAMMING	2

/*
 * PLL-based clock recovery constants.
 *
 * The demodulator uses a phase accumulator that advances at the expected
 * baud rate, with proportional correction on zero crossings. This is the
 * same approach used by multimon-ng (since 1996) and our FLEX demodulator.
 */

/* Phase accumulator range.
 * Use 24 bits for high precision. At 48000 Hz / 1200 baud:
 *   pll_inc = 16777216 * 1200 / 48000 = 419430.4 → 419430
 *   Error per sample: 0.4 / 419430 = 0.000000954 = 0.0001%
 *   Over one batch (544 bits × 40 samples = 21760 samples):
 *   drift = 0.4 × 21760 = 8704 phase units out of 419430/bit = 0.02 bits
 * This is negligible — no bit drift over any realistic transmission. */
#define PLL_PHASE_MAX		0x1000000u

/* PLL correction gain: pll_inc >> 3 = 12.5% of one bit period per
 * zero crossing, matching multimon-ng's SPHASEINC/8. Works correctly
 * because subsampling normalizes samples-per-bit to ~9 at all rates. */

/* Mid-symbol crossing threshold for sync acceptance in scanner.
 * If the PLL has more than this many consecutive mid-symbol crossings,
 * reject the sync candidate — the PLL is not tracking valid FSK. */
#define PLL_NONCONSEC_MAX_SYNC	10

/* Phase boundaries for mid-symbol crossing detection.
 * In the PLL, phase 0 = just emitted a bit (symbol center).
 * Phase ~50% = expected FSK transition point.
 * Phase ~100% = about to emit next bit.
 *
 * Crossings in the middle range (10%-90%) are near the expected
 * transition point — PLL is tracking. Crossings outside this range
 * (near 0% or 100%, i.e., near symbol centers) indicate noise. */
#define PLL_PHASE_10PCT		(PLL_PHASE_MAX / 10)
#define PLL_PHASE_90PCT		(PLL_PHASE_MAX * 9 / 10)
#define PLL_PHASE_50PCT		(PLL_PHASE_MAX / 2)

/*
 * PLL-based FSK demodulator: process one audio sample.
 *
 * Phase accumulator advances at the expected baud rate. Zero crossings
 * nudge the phase toward the nearest symbol boundary by a fraction
 * (12.5%, matching multimon-ng). Bits are emitted at phase wrap, not
 * at zero crossings — so noise crossings don't produce extra bits.
 *
 * Returns 1 if a symbol boundary was reached (bit ready in *bit_out).
 * Returns 0 if no bit yet.
 *
 * Also tracks signal quality:
 *   nonconsec: consecutive mid-symbol crossings (noise/wrong speed)
 *   timeout: symbol periods with no crossings (silence/carrier)
 */
static inline int pll_process_sample(
	sample_t sample, double polarity,
	uint32_t pll_inc,
	uint32_t *pll_phase, uint8_t *lastsign,
	uint8_t *nonconsec, uint8_t *timeout,
	uint8_t *bit_out)
{
	uint8_t sign = (sample * polarity > 0.0) ? 1 : 0;

	if (sign != *lastsign) {
		uint32_t correction = pll_inc >> 3;
		uint32_t threshold = PLL_PHASE_50PCT - (pll_inc >> 1);

		if (*pll_phase < threshold)
			*pll_phase += correction;
		else
			*pll_phase -= correction;

		/* Mid-symbol crossing classification.
		 * Crossings near the expected transition point (around 50% of
		 * phase) are normal for FSK. Crossings far from it (near 0%
		 * or near 100%, i.e., near the symbol center) indicate noise
		 * or wrong baud rate. */
		if (*pll_phase >= PLL_PHASE_10PCT && *pll_phase <= PLL_PHASE_90PCT) {
			/* Crossing near expected transition point — good */
			*nonconsec = 0;
		} else {
			/* Crossing near symbol center — bad */
			if (*nonconsec < 255)
				(*nonconsec)++;
		}

		*timeout = 0;
		*lastsign = sign;
	}

	/* Advance phase accumulator */
	*pll_phase += pll_inc;

	/* Symbol boundary? */
	if (*pll_phase >= PLL_PHASE_MAX) {
		*pll_phase &= (PLL_PHASE_MAX - 1);
		*bit_out = sign;

		if (*timeout < 255)
			(*timeout)++;

		return 1;
	}

	return 0;
}

void dsp_init_ramp(pocsag_t *pocsag)
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
                pocsag->fsk_ramp_down[i] = c * pocsag->fsk_deviation * pocsag->fsk_tx_polarity;
                pocsag->fsk_ramp_up[i] = -pocsag->fsk_ramp_down[i];
        }
}

/* Init transceiver instance. */
int dsp_init_sender(pocsag_t *pocsag, int samplerate, int baudrate, double deviation, double polarity, int auto_baud, int auto_polarity)
{
	static const int rates[] = { 512, 1200, 2400 };
	int rc;
	int i, n;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Init DSP for transceiver.\n");

	/* set modulation parameters */
	// NOTE: baudrate equals modulation, because we have a raised cosine ramp of beta = 0.5
	sender_set_fm(&pocsag->sender, deviation, baudrate, deviation, MAX_DISPLAY);

	pocsag->fsk_bitduration = (double)samplerate / (double)baudrate;
	pocsag->fsk_tx_bitduration = pocsag->fsk_bitduration;
	pocsag->fsk_tx_bitstep = 1.0 / pocsag->fsk_bitduration;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Use %.4f samples for one bit duration @ %d.\n", pocsag->fsk_bitduration, pocsag->sender.samplerate);

	pocsag->fsk_tx_buffer_size = pocsag->fsk_tx_bitduration * 32.0 + 10; /* 32 bit, add some extra to prevent short buffer due to rounding */
	pocsag->fsk_tx_buffer = calloc(pocsag->fsk_tx_buffer_size, sizeof(sample_t));
	if (!pocsag->fsk_tx_buffer) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* create deviation and ramp */
	pocsag->fsk_deviation = 1.0; // equals what we st at sender_set_fm()
	pocsag->fsk_polarity = polarity;
	pocsag->fsk_tx_polarity = polarity;
	dsp_init_ramp(pocsag);

	/* store auto-detection config */
	pocsag->rx_auto_baud = auto_baud;
	pocsag->rx_auto_polarity = auto_polarity;
	pocsag->rx_baud_locked = baudrate;
	pocsag->rx_polarity_locked = polarity;
	pocsag->samplerate = samplerate;

	/* init multi-rate RX decoders */
	if (auto_baud) {
		n = 3;
		for (i = 0; i < 3; i++) {
			int spb = samplerate / rates[i];
			int subsamp = spb / 9;
			if (subsamp < 1) subsamp = 1;
			pocsag->rx_baud[i].baudrate = rates[i];
			pocsag->rx_baud[i].subsamp = subsamp;
			pocsag->rx_baud[i].subsamp_cnt = 0;
			pocsag->rx_baud[i].pll_inc = (uint32_t)((uint64_t)PLL_PHASE_MAX * rates[i] * subsamp / samplerate);
			pocsag->rx_baud[i].pll_phase = 0;
			pocsag->rx_baud[i].lastsign = 0;
			pocsag->rx_baud[i].nonconsec = 0;
			pocsag->rx_baud[i].timeout = 0;
			pocsag->rx_baud[i].word = 0;
			pocsag->rx_baud[i].word_inv = 0;
		}
		LOGP_CHAN(DDSP, LOGL_INFO, "RX baud rate: auto-detect (512/1200/2400).\n");
	} else {
		n = 1;
		{
			int spb = samplerate / baudrate;
			int subsamp = spb / 9;
			if (subsamp < 1) subsamp = 1;
			pocsag->rx_baud[0].baudrate = baudrate;
			pocsag->rx_baud[0].subsamp = subsamp;
			pocsag->rx_baud[0].subsamp_cnt = 0;
			pocsag->rx_baud[0].pll_inc = (uint32_t)((uint64_t)PLL_PHASE_MAX * baudrate * subsamp / samplerate);
			pocsag->rx_baud[0].pll_phase = 0;
			pocsag->rx_baud[0].lastsign = 0;
			pocsag->rx_baud[0].nonconsec = 0;
			pocsag->rx_baud[0].timeout = 0;
			pocsag->rx_baud[0].word = 0;
			pocsag->rx_baud[0].word_inv = 0;
		}
		LOGP_CHAN(DDSP, LOGL_INFO, "RX baud rate: locked to %d.\n", baudrate);
	}
	pocsag->rx_baud_count = n;

	if (auto_polarity)
		LOGP_CHAN(DDSP, LOGL_INFO, "RX polarity: auto-detect.\n");
	else
		LOGP_CHAN(DDSP, LOGL_INFO, "RX polarity: locked to %s.\n", (polarity < 0) ? "normal" : "inverted");

	return 0;

error:
        dsp_cleanup_sender(pocsag);

        return -rc;

}

/* Cleanup transceiver instance. */
void dsp_cleanup_sender(pocsag_t *pocsag)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Cleanup DSP for transceiver.\n");

	if (pocsag->fsk_tx_buffer) {
		free(pocsag->fsk_tx_buffer);
		pocsag->fsk_tx_buffer = NULL;
	}
}


/* encode one codeward into samples
 * input: 32 data bits
 * output: samples
 * return number of samples */
static int fsk_block_encode(pocsag_t *pocsag, uint32_t word)
{
	/* alloc samples, add 1 in case there is a rest */
	sample_t *spl;
	double phase, bitstep, devpol;
	int i, count;
	uint8_t lastbit;

	devpol = pocsag->fsk_deviation * pocsag->fsk_tx_polarity;
	spl = pocsag->fsk_tx_buffer;
	phase = pocsag->fsk_tx_phase;
	lastbit = pocsag->fsk_tx_lastbit;
	bitstep = pocsag->fsk_tx_bitstep * 256.0;

	/* add 32 bits */
	for (i = 0; i < 32; i++) {
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
					*spl++ = pocsag->fsk_ramp_down[(uint8_t)phase];
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
				lastbit = 0;
			}
		} else {
			if ((word & 0x80000000)) {
				/* ramp up */
				do {
					*spl++ = pocsag->fsk_ramp_up[(uint8_t)phase];
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
	count = ((uintptr_t)spl - (uintptr_t)pocsag->fsk_tx_buffer) / sizeof(*spl);

	pocsag->fsk_tx_phase = phase;
	pocsag->fsk_tx_lastbit = lastbit;

	return count;
}

/*
 * Lock the receiver to a specific baud rate and polarity after sync detection.
 * Initializes the locked-decoder PLL at the detected rate.
 */
static void dsp_rx_lock(pocsag_t *pocsag, int baudrate, double polarity)
{
	pocsag->rx_baud_locked = baudrate;
	pocsag->rx_polarity_locked = polarity;
	pocsag->fsk_polarity = polarity;
	pocsag->rx_rate_locked = 1;
	pocsag->rx_resync_countdown = 0;

	/* Initialize locked decoder PLL.
	 * Phase, lastsign, and subsamp_cnt are set by the caller from the
	 * scanner slot that found sync, so the PLL continues seamlessly. */
	{
		int spb = pocsag->samplerate / baudrate;
		int subsamp = spb / 9;
		if (subsamp < 1) subsamp = 1;
		pocsag->fsk_rx_subsamp = subsamp;
		pocsag->fsk_rx_pll_inc = (uint32_t)((uint64_t)PLL_PHASE_MAX * baudrate * subsamp / pocsag->samplerate);
	}
}

/*
 * Reset multi-rate search state. Called when sync is lost (batch ends)
 * and we need to go back to scanning for preamble/sync.
 */
static void dsp_rx_unlock(pocsag_t *pocsag)
{
	int i;

	pocsag->rx_rate_locked = 0;
	pocsag->rx_resync_countdown = 0;

	for (i = 0; i < pocsag->rx_baud_count; i++) {
		pocsag->rx_baud[i].pll_phase = 0;
		pocsag->rx_baud[i].lastsign = 0;
		pocsag->rx_baud[i].nonconsec = 0;
		pocsag->rx_baud[i].timeout = 0;
		pocsag->rx_baud[i].subsamp_cnt = 0;
		pocsag->rx_baud[i].word = 0;
		pocsag->rx_baud[i].word_inv = 0;
	}
}

/*
 * Maximum number of bits to search for the next FSC after a batch ends.
 *
 * Per POCSAG spec, consecutive batches are at the same rate with the FSC
 * immediately following the last codeword (zero gap). We only need tolerance
 * for minor bit slip / clock drift.
 *
 * Keep this small: while we're locked at one rate, the locked decoder is
 * consuming samples. If a new transmission starts at a different rate,
 * those samples are demodulated at the wrong rate and lost. At 512 baud,
 * each bit is ~2ms, so 32 bits = ~62ms — short enough to not miss a
 * new preamble at any rate (576 bits at 2400 baud = 240ms).
 */
#define RESYNC_TIMEOUT_BITS	48

/* forward declaration — fsk_decode_resync calls fsk_decode on unlock */
static void fsk_decode(pocsag_t *pocsag, sample_t *spl, int length);

/* Minimum alternating bits to qualify as preamble (used for
 * BCH-corrected sync gating). */
#define MIN_PREAMBLE_BITS_BCH	8

/* Count preamble bits preceding a sync word in a 64-bit shift register. */
static int count_preamble_bits(uint64_t word);

static void fsk_block_decode(pocsag_t *pocsag, uint8_t bit)
{
	if (!pocsag->fsk_rx_sync) {
		pocsag->fsk_rx_word = (pocsag->fsk_rx_word << 1) | bit;
		pocsag->fsk_rx_bit_count++;
		uint32_t w = (uint32_t)pocsag->fsk_rx_word;

		/*
		 * Check for sync word BEFORE countdown — the FSC is 32 bits,
		 * same as the timeout. If we check countdown first, we'd
		 * unlock on the last bit of the FSC before detecting it.
		 */
		if (hamming32(w, CODEWORD_SYNC) <= SYNC_MAX_HAMMING) {
			uint32_t trial = w;
			if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
				if (pocsag->rx_resync_countdown > 0) {
					/* Between batches — no preamble expected */
					if (w != CODEWORD_SYNC)
						LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected FSC (0x%08x) at %d baud, %s polarity.\n",
							  w, pocsag->rx_baud_locked, (pocsag->fsk_polarity < 0) ? "normal" : "inverted");
					else
						LOGP_CHAN(DDSP, LOGL_INFO, "FSC found at %d baud, %s polarity.\n",
							  pocsag->rx_baud_locked, (pocsag->fsk_polarity < 0) ? "normal" : "inverted");
				} else {
					/* First batch — report preamble quality */
					int preamble = count_preamble_bits(pocsag->fsk_rx_word);
					if (w != CODEWORD_SYNC)
						LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected FSC (0x%08x) at %d baud, %s polarity (%d%s preamble bits).\n",
							  w, pocsag->rx_baud_locked, (pocsag->fsk_polarity < 0) ? "normal" : "inverted",
							  preamble, (preamble >= 31) ? "+" : "");
					else
						LOGP_CHAN(DDSP, LOGL_INFO, "FSC found at %d baud, %s polarity (%d%s preamble bits).\n",
							  pocsag->rx_baud_locked, (pocsag->fsk_polarity < 0) ? "normal" : "inverted",
							  preamble, (preamble >= 31) ? "+" : "");
				}
				put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
				pocsag->fsk_rx_sync = 16;
				pocsag->fsk_rx_index = 0;
				pocsag->rx_resync_countdown = 0;
				pocsag->fsk_rx_bit_count = 0;
			}
		} else if (hamming32(w, (uint32_t)(~CODEWORD_SYNC)) <= SYNC_MAX_HAMMING) {
			uint32_t trial = ~w;
			if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
				if (pocsag->rx_auto_polarity) {
					double new_pol = -pocsag->fsk_polarity;
					int preamble = count_preamble_bits(~pocsag->fsk_rx_word);
					LOGP_CHAN(DDSP, LOGL_INFO, "Inverted FSC in locked mode — flipping to %s polarity (%d%s preamble bits).\n",
						  (new_pol < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, pocsag->rx_baud_locked, new_pol);
					pocsag->fsk_rx_lastsign = !pocsag->fsk_rx_lastsign;
					put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
					pocsag->fsk_rx_sync = 16;
					pocsag->fsk_rx_index = 0;
					pocsag->rx_resync_countdown = 0;
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Inverted FSC in locked mode (polarity locked, ignoring).\n");
				}
			}
		}

		/* count down resync timeout AFTER FSC checks */
		if (pocsag->rx_resync_countdown > 0) {
			/*
			 * Frame-level sanity check: during the between-batch
			 * FSC search, the shift register should only contain
			 * FSC or noise. If a non-FSC word BCH-corrects to a
			 * valid codeword, the signal has changed (e.g., new
			 * transmission at different speed). Immediately unlock
			 * to prevent phantom messages from garbage data.
			 */
			if (pocsag->fsk_rx_bit_count >= 32) {
				uint32_t trial = w;
				if (pocsag_bch_correct(&trial, NULL) >= 0 && trial != CODEWORD_SYNC) {
					LOGP_CHAN(DDSP, LOGL_INFO, "Non-FSC codeword (0x%08x) during resync — signal changed, unlocking.\n", trial);
					pocsag->rx_resync_countdown = 0;
					if (pocsag->rx_auto_baud || pocsag->rx_auto_polarity)
						dsp_rx_unlock(pocsag);
					return;
				}
			}
			if (--pocsag->rx_resync_countdown == 0) {
				LOGP_CHAN(DDSP, LOGL_DEBUG, "No FSC found within %d bits after batch end, unlocking (last word=0x%08x, phase=%u, bits_since_fsc=%d).\n",
					  RESYNC_TIMEOUT_BITS, w, pocsag->fsk_rx_pll_phase, pocsag->fsk_rx_bit_count);
				if (pocsag->rx_auto_baud || pocsag->rx_auto_polarity)
					dsp_rx_unlock(pocsag);
			}
		}
	} else {
		pocsag->fsk_rx_word = (pocsag->fsk_rx_word << 1) | bit;
		if (++pocsag->fsk_rx_index == 32) {
			pocsag->fsk_rx_index = 0;
			put_codeword(pocsag, (uint32_t)pocsag->fsk_rx_word, (16 - pocsag->fsk_rx_sync) >> 1, pocsag->fsk_rx_sync & 1);
			if (--pocsag->fsk_rx_sync == 0) {
				/*
				 * Batch complete. Per POCSAG spec, all batches after
				 * the preamble are at the same data rate. Stay locked
				 * and search for the next FSC at the same rate.
				 * Start a countdown — if no FSC is found within
				 * RESYNC_TIMEOUT_BITS, the transmission has ended
				 * and we unlock for the next preamble.
				 */
				pocsag->rx_resync_countdown = RESYNC_TIMEOUT_BITS;
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Batch complete, searching for next FSC (phase=%u, last_cw=0x%08x).\n",
					  pocsag->fsk_rx_pll_phase, (uint32_t)pocsag->fsk_rx_word);
				pocsag->fsk_rx_bit_count = 0;
			}
		}
	}
}

/*
 * Minimum preamble bits for sync acceptance.
 *
 * Exact sync match (0 bit errors): no preamble required.
 *   False positive rate: 2400 baud × 6 scanners × 2^-32 ≈ 3e-6/sec.
 *   Essentially zero — the 32-bit sync word is sufficient on its own.
 *
 * BCH-corrected sync (1-2 bit errors): require MIN_PREAMBLE_BITS_BCH.
 *   529 words within hamming ≤ 2 of SYNC, so false positive rate without
 *   preamble: 2400 × 6 × 529/2^32 ≈ 1.8e-3/sec (one every ~10 min).
 *   Even 8 alternating preamble bits (probability ~2^-8 for random data)
 *   reduces this to ~7e-6/sec — negligible.
 */

/*
 * Count preamble bits preceding a sync word in a 64-bit shift register.
 * The lower 32 bits hold the sync word, the upper 32 hold the preceding bits.
 * Preamble is alternating 1-0, so we count backward from bit 32 (first bit
 * above the sync word) checking that each bit differs from the previous one.
 * Returns 0-32. If all 32 upper bits alternate, returns 32 (meaning "32+").
 */
static int count_preamble_bits(uint64_t word)
{
	int count = 0;
	int i;
	uint8_t prev_bit = (word >> 32) & 1;  /* bit 32: first bit above sync */
	
	for (i = 33; i < 64; i++) {
		uint8_t bit = (word >> i) & 1;
		if (bit != prev_bit) {
			count++;
			prev_bit = bit;
		} else {
			break;
		}
	}
	return count;
}

/*
 * Multi-rate preamble/sync scanner.
 *
 * Runs all configured baud rate slots in parallel. Each slot maintains its
 * own phase accumulator and shift register. We check both normal and inverted
 * polarity shift registers for the sync word.
 *
 * Sync acceptance policy:
 *   - Exact sync match: accepted immediately (no preamble needed).
 *   - BCH-corrected sync (1-2 bit errors): requires MIN_PREAMBLE_BITS_BCH
 *     alternating preamble bits to reduce false positive risk.
 *
 * When sync is found: lock to that rate/polarity, configure the main decoder,
 * and hand off to fsk_decode_locked() for the rest of the batch.
 *
 * Returns 1 if sync was found (and we locked), 0 otherwise.
 * *consumed is set to the number of samples processed before lock.
 */
static int fsk_decode_scan(pocsag_t *pocsag, sample_t *spl, int length, int *consumed)
{
	double polarity = pocsag->fsk_polarity;
	int i, s;

	for (i = 0; i < length; i++) {
		sample_t sample = spl[i];

		for (s = 0; s < pocsag->rx_baud_count; s++) {
			struct rx_baud_state *st = &pocsag->rx_baud[s];
			uint8_t bit_val;

			/* PLL-based FSK demod per slot (with subsampling) */
			if (++st->subsamp_cnt < st->subsamp)
				continue;
			st->subsamp_cnt = 0;

			if (!pll_process_sample(sample, polarity, st->pll_inc,
						&st->pll_phase, &st->lastsign,
						&st->nonconsec, &st->timeout,
						&bit_val))
				continue;

			/* shift bit into both normal and inverted 64-bit registers */
			st->word = (st->word << 1) | bit_val;
			st->word_inv = (st->word_inv << 1) | (bit_val ^ 1);

			/*
			 * Sync detection with tiered acceptance:
			 *   - Exact match: accept immediately (no preamble needed)
			 *   - BCH-corrected (hamming ≤ 2): require preamble
			 *   - Both: require PLL tracking (low nonconsec)
			 *
			 * Compare lower 32 bits against sync word.
			 * Upper 32 bits hold preceding data for preamble counting.
			 * Hamming pre-filter avoids expensive BCH on every bit.
			 */
			{
			uint32_t w = (uint32_t)st->word;
			uint32_t w_inv = (uint32_t)st->word_inv;

			/* check normal polarity */
			if (w == CODEWORD_SYNC) {
				if (st->nonconsec >= PLL_NONCONSEC_MAX_SYNC) {
					LOGP_CHAN(DDSP, LOGL_DEBUG, "Sync candidate rejected at %d baud, %s polarity (nonconsec=%d).\n",
						  st->baudrate, (polarity < 0) ? "normal" : "inverted", st->nonconsec);
					goto check_inverted;
				}
				int preamble = count_preamble_bits(st->word);
				LOGP_CHAN(DDSP, LOGL_INFO, "Sync detected at %d baud, %s polarity (%d%s preamble bits).\n",
					  st->baudrate, (polarity < 0) ? "normal" : "inverted",
					  preamble, (preamble >= 31) ? "+" : "");
				dsp_rx_lock(pocsag, st->baudrate, polarity);
				pocsag->fsk_rx_pll_phase = st->pll_phase;
				pocsag->fsk_rx_lastsign = st->lastsign;
				pocsag->fsk_rx_subsamp_cnt = st->subsamp_cnt;
				pocsag->fsk_rx_word = CODEWORD_SYNC;
				put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
				pocsag->fsk_rx_sync = 16;
				pocsag->fsk_rx_index = 0;
				*consumed = i + 1;
				return 1;
			}
			if (st->nonconsec < PLL_NONCONSEC_MAX_SYNC
			    && count_preamble_bits(st->word) >= MIN_PREAMBLE_BITS_BCH
			    && hamming32(w, CODEWORD_SYNC) <= SYNC_MAX_HAMMING) {
				uint32_t trial = w;
				if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
					int preamble = count_preamble_bits(st->word);
					LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected sync at %d baud, %s polarity (%d%s preamble bits).\n",
						  st->baudrate, (polarity < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, st->baudrate, polarity);
					pocsag->fsk_rx_pll_phase = st->pll_phase;
					pocsag->fsk_rx_lastsign = st->lastsign;
				pocsag->fsk_rx_subsamp_cnt = st->subsamp_cnt;
					pocsag->fsk_rx_word = CODEWORD_SYNC;
					put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
					pocsag->fsk_rx_sync = 16;
					pocsag->fsk_rx_index = 0;
					*consumed = i + 1;
					return 1;
				}
			}
check_inverted:
			/* check inverted polarity (only if auto-detecting polarity) */
			if (pocsag->rx_auto_polarity) {
				if (w_inv == CODEWORD_SYNC) {
					if (st->nonconsec >= PLL_NONCONSEC_MAX_SYNC)
						continue;
					double inv_pol = -polarity;
					int preamble = count_preamble_bits(st->word_inv);
					LOGP_CHAN(DDSP, LOGL_INFO, "Sync detected at %d baud, %s polarity (inverted, %d%s preamble bits).\n",
						  st->baudrate, (inv_pol < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, st->baudrate, inv_pol);
					pocsag->fsk_rx_pll_phase = st->pll_phase;
					pocsag->fsk_rx_lastsign = !st->lastsign;
					pocsag->fsk_rx_subsamp_cnt = st->subsamp_cnt;
					pocsag->fsk_rx_word = CODEWORD_SYNC;
					put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
					pocsag->fsk_rx_sync = 16;
					pocsag->fsk_rx_index = 0;
					*consumed = i + 1;
					return 1;
				}
				if (st->nonconsec < PLL_NONCONSEC_MAX_SYNC
				    && count_preamble_bits(st->word_inv) >= MIN_PREAMBLE_BITS_BCH
				    && hamming32(w_inv, CODEWORD_SYNC) <= SYNC_MAX_HAMMING) {
					uint32_t trial = w_inv;
					if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
						double inv_pol = -polarity;
						int preamble = count_preamble_bits(st->word_inv);
						LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected sync at %d baud, %s polarity (inverted, %d%s preamble bits).\n",
							  st->baudrate, (inv_pol < 0) ? "normal" : "inverted",
							  preamble, (preamble >= 31) ? "+" : "");
						dsp_rx_lock(pocsag, st->baudrate, inv_pol);
						pocsag->fsk_rx_pll_phase = st->pll_phase;
						pocsag->fsk_rx_lastsign = !st->lastsign;
					pocsag->fsk_rx_subsamp_cnt = st->subsamp_cnt;
						pocsag->fsk_rx_word = CODEWORD_SYNC;
						put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
						pocsag->fsk_rx_sync = 16;
						pocsag->fsk_rx_index = 0;
						*consumed = i + 1;
						return 1;
					}
				}
			}
			}
		}
	}

	*consumed = length;
	return 0;
}

/*
 * Locked-mode FSK decoder. Runs at the locked baud rate and polarity.
 * This is the original fsk_decode logic, used after sync has been found.
 */
static void fsk_decode_locked(pocsag_t *pocsag, sample_t *spl, int length)
{
	double polarity;
	uint32_t pll_phase, pll_inc;
	uint8_t lastsign;
	uint8_t nonconsec = 0, timeout = 0; /* unused, required by pll_process_sample */
	int subsamp, subsamp_cnt;
	int i;

	polarity = pocsag->fsk_polarity;
	pll_phase = pocsag->fsk_rx_pll_phase;
	pll_inc = pocsag->fsk_rx_pll_inc;
	lastsign = pocsag->fsk_rx_lastsign;
	subsamp = pocsag->fsk_rx_subsamp;
	subsamp_cnt = pocsag->fsk_rx_subsamp_cnt;

	for (i = 0; i < length; i++) {
		uint8_t bit_val;

		if (++subsamp_cnt < subsamp)
			continue;
		subsamp_cnt = 0;

		if (!pll_process_sample(spl[i], polarity, pll_inc,
					&pll_phase, &lastsign,
					&nonconsec, &timeout,
					&bit_val))
			continue;

		fsk_block_decode(pocsag, bit_val);
	}

	pocsag->fsk_rx_pll_phase = pll_phase;
	pocsag->fsk_rx_lastsign = lastsign;
	pocsag->fsk_rx_subsamp_cnt = subsamp_cnt;
}

/*
 * Between-batches decoder: runs the locked decoder PLL searching for
 * the next FSC at the current rate. The scanner is NOT run here — it
 * only runs when fully unlocked (transmission ended).
 */
static void fsk_decode_resync(pocsag_t *pocsag, sample_t *spl, int length)
{
	double polarity;
	uint32_t pll_phase, pll_inc;
	uint8_t lastsign;
	uint8_t nonconsec = 0, timeout = 0; /* unused, required by pll_process_sample */
	int subsamp, subsamp_cnt;
	int i;

	polarity = pocsag->fsk_polarity;
	pll_phase = pocsag->fsk_rx_pll_phase;
	pll_inc = pocsag->fsk_rx_pll_inc;
	lastsign = pocsag->fsk_rx_lastsign;
	subsamp = pocsag->fsk_rx_subsamp;
	subsamp_cnt = pocsag->fsk_rx_subsamp_cnt;

	for (i = 0; i < length; i++) {
		uint8_t bit_val;

		if (++subsamp_cnt < subsamp)
			continue;
		subsamp_cnt = 0;

		if (pll_process_sample(spl[i], polarity, pll_inc,
				       &pll_phase, &lastsign,
				       &nonconsec, &timeout,
				       &bit_val)) {
			fsk_block_decode(pocsag, bit_val);
		}

		/* Did the locked decoder find the next FSC? */
		if (pocsag->fsk_rx_sync) {
			pocsag->fsk_rx_pll_phase = pll_phase;
			pocsag->fsk_rx_lastsign = lastsign;
			pocsag->fsk_rx_subsamp_cnt = subsamp_cnt;
			if (i + 1 < length)
				fsk_decode_locked(pocsag, spl + i + 1, length - i - 1);
			return;
		}

		/* Did the resync timeout expire? Unlock and let scanner take over. */
		if (!pocsag->rx_rate_locked) {
			pocsag->fsk_rx_pll_phase = pll_phase;
			pocsag->fsk_rx_lastsign = lastsign;
			pocsag->fsk_rx_subsamp_cnt = subsamp_cnt;
			if (i + 1 < length)
				fsk_decode(pocsag, spl + i + 1, length - i - 1);
			return;
		}
	}

	pocsag->fsk_rx_pll_phase = pll_phase;
	pocsag->fsk_rx_lastsign = lastsign;
	pocsag->fsk_rx_subsamp_cnt = subsamp_cnt;
}

/*
 * Main FSK decode entry point.
 *
 * Three modes of operation:
 *
 * 1. Synced (fsk_rx_sync > 0): Actively decoding a batch at locked rate.
 *
 * 2. Rate-locked, between batches (rx_rate_locked && !fsk_rx_sync):
 *    Run locked decoder PLL only, searching for next FSC at same rate.
 *    Per POCSAG spec, all batches in a transmission use the same rate.
 *
 * 3. Unlocked (!rx_rate_locked && !fsk_rx_sync): No active transmission.
 *    Run the multi-rate preamble+sync scanner only.
 */
static void fsk_decode(pocsag_t *pocsag, sample_t *spl, int length)
{
	int consumed;

	while (length > 0) {
		if (pocsag->fsk_rx_sync) {
			/* Actively decoding a batch — locked decoder only */
			fsk_decode_locked(pocsag, spl, length);
			return;
		}

		if (pocsag->rx_rate_locked) {
			/* Between batches — run both paths in parallel */
			fsk_decode_resync(pocsag, spl, length);
			return;
		}

		if (pocsag->rx_auto_baud || pocsag->rx_auto_polarity) {
			/* Fully unlocked — scan for preamble/sync */
			if (fsk_decode_scan(pocsag, spl, length, &consumed)) {
				spl += consumed;
				length -= consumed;
				continue;
			}
			return;
		}

		/* Not auto-detecting — original single-rate behavior */
		fsk_decode_locked(pocsag, spl, length);
		return;
	}
}

/* Process received audio stream from radio unit. */
void sender_receive(sender_t *sender, sample_t *samples, int length, double __attribute__((unused)) rf_level_db)
{
	pocsag_t *pocsag = (pocsag_t *) sender;

	if (pocsag->rx)
		fsk_decode(pocsag, samples, length);
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	pocsag_t *pocsag = (pocsag_t *) sender;

again:
	/* get word */
	if (!pocsag->fsk_tx_buffer_length) {
		int64_t word = get_codeword(pocsag);

		/* no message, power is off */
		if (word < 0) {
			memset(samples, 0, sizeof(samples) * length);
			memset(power, 0, length);
			return;
		}

		/* encode */
		pocsag->fsk_tx_buffer_length = fsk_block_encode(pocsag, word);
		pocsag->fsk_tx_buffer_pos = 0;
	}

	/* send encoded word until end of source or destination buffer is reaced */
	while (length) {
		*power++ = 1;
		*samples++ = pocsag->fsk_tx_buffer[pocsag->fsk_tx_buffer_pos++];
		length--;
		if (pocsag->fsk_tx_buffer_pos == pocsag->fsk_tx_buffer_length) {
			pocsag->fsk_tx_buffer_length = 0;
			break;
		}
	}

	/* do again, if destination buffer is not yet full */
	if (length)
		goto again;
}


