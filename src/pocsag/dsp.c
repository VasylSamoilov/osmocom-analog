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
                pocsag->fsk_ramp_down[i] = c * pocsag->fsk_deviation * pocsag->fsk_polarity;
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
	pocsag->fsk_bitstep = 1.0 / pocsag->fsk_bitduration;
	pocsag->fsk_tx_bitduration = pocsag->fsk_bitduration;
	pocsag->fsk_tx_bitstep = pocsag->fsk_bitstep;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Use %.4f samples for one bit duration @ %d.\n", pocsag->fsk_bitduration, pocsag->sender.samplerate);

	pocsag->fsk_tx_buffer_size = pocsag->fsk_bitduration * 32.0 + 10; /* 32 bit, add some extra to prevent short buffer due to rounding */
	pocsag->fsk_tx_buffer = calloc(pocsag->fsk_tx_buffer_size, sizeof(sample_t));
	if (!pocsag->fsk_tx_buffer) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* create deviation and ramp */
	pocsag->fsk_deviation = 1.0; // equals what we st at sender_set_fm()
	pocsag->fsk_polarity = polarity;
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
			pocsag->rx_baud[i].baudrate = rates[i];
			pocsag->rx_baud[i].bitstep = 1.0 / ((double)samplerate / (double)rates[i]);
			pocsag->rx_baud[i].phase = 0.0;
			pocsag->rx_baud[i].lastbit = 0;
			pocsag->rx_baud[i].word = 0;
			pocsag->rx_baud[i].word_inv = 0;
		}
		LOGP_CHAN(DDSP, LOGL_INFO, "RX baud rate: auto-detect (512/1200/2400).\n");
	} else {
		n = 1;
		pocsag->rx_baud[0].baudrate = baudrate;
		pocsag->rx_baud[0].bitstep = 1.0 / ((double)samplerate / (double)baudrate);
		pocsag->rx_baud[0].phase = 0.0;
		pocsag->rx_baud[0].lastbit = 0;
		pocsag->rx_baud[0].word = 0;
		pocsag->rx_baud[0].word_inv = 0;
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

	devpol = pocsag->fsk_deviation * pocsag->fsk_polarity;
	spl = pocsag->fsk_tx_buffer;
	phase = pocsag->fsk_tx_phase;
	lastbit = pocsag->fsk_tx_lastbit;
	bitstep = pocsag->fsk_bitstep * 256.0;

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
 * Reconfigures the main fsk_bitstep/phase so the existing locked-mode decoder
 * (fsk_block_decode / fsk_decode) works at the detected rate.
 */
static void dsp_rx_lock(pocsag_t *pocsag, int baudrate, double polarity)
{
	pocsag->rx_baud_locked = baudrate;
	pocsag->rx_polarity_locked = polarity;
	pocsag->fsk_bitstep = 1.0 / ((double)pocsag->samplerate / (double)baudrate);
	pocsag->fsk_polarity = polarity;
	pocsag->rx_rate_locked = 1;
	pocsag->rx_resync_countdown = 0;
	pocsag->fsk_rx_phase = 0.0;
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
		pocsag->rx_baud[i].phase = 0.0;
		pocsag->rx_baud[i].lastbit = 0;
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
#define RESYNC_TIMEOUT_BITS	32

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
		uint32_t w = (uint32_t)pocsag->fsk_rx_word;

		/*
		 * Check for sync word BEFORE countdown — the FSC is 32 bits,
		 * same as the timeout. If we check countdown first, we'd
		 * unlock on the last bit of the FSC before detecting it.
		 */
		if (hamming32(w, CODEWORD_SYNC) <= SYNC_MAX_HAMMING) {
			uint32_t trial = w;
			if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
				int preamble = count_preamble_bits(pocsag->fsk_rx_word);
				if (w != CODEWORD_SYNC)
					LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected FSC in locked mode (0x%08x -> 0x%08x) at %d baud, %s polarity (%d%s preamble bits).\n",
						  w, CODEWORD_SYNC,
						  pocsag->rx_baud_locked, (pocsag->fsk_polarity < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
				else
					LOGP_CHAN(DDSP, LOGL_INFO, "FSC found in locked mode at %d baud, %s polarity (%d%s preamble bits).\n",
						  pocsag->rx_baud_locked, (pocsag->fsk_polarity < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
				put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
				pocsag->fsk_rx_sync = 16;
				pocsag->fsk_rx_index = 0;
				pocsag->rx_resync_countdown = 0;
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
					pocsag->fsk_rx_lastbit = !pocsag->fsk_rx_lastbit;
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
			if (--pocsag->rx_resync_countdown == 0) {
				LOGP_CHAN(DDSP, LOGL_INFO, "No FSC found within %d bits after batch end, unlocking.\n", RESYNC_TIMEOUT_BITS);
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
		double sample = spl[i];

		for (s = 0; s < pocsag->rx_baud_count; s++) {
			struct rx_baud_state *st = &pocsag->rx_baud[s];
			uint8_t got_bit = 0;
			uint8_t bit_val = 0;

			/* simple zero-crossing FSK demod per slot */
			if (sample * polarity > 0.0) {
				if (!st->lastbit) {
					st->phase = -0.5;
					st->lastbit = 1;
					got_bit = 1; bit_val = 1;
				} else {
					st->phase += st->bitstep;
					if (st->phase >= 1.0) {
						st->phase -= 1.0;
						got_bit = 1; bit_val = 1;
					}
				}
			} else {
				if (st->lastbit) {
					st->phase = -0.5;
					st->lastbit = 0;
					got_bit = 1; bit_val = 0;
				} else {
					st->phase += st->bitstep;
					if (st->phase >= 1.0) {
						st->phase -= 1.0;
						got_bit = 1; bit_val = 0;
					}
				}
			}

			if (!got_bit)
				continue;

			/* shift bit into both normal and inverted 64-bit registers */
			st->word = (st->word << 1) | bit_val;
			st->word_inv = (st->word_inv << 1) | (bit_val ^ 1);

			/*
			 * Sync detection with tiered acceptance:
			 *   - Exact match: accept immediately (no preamble needed)
			 *   - BCH-corrected (hamming ≤ 2): require preamble
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
				int preamble = count_preamble_bits(st->word);
				LOGP_CHAN(DDSP, LOGL_INFO, "Sync detected at %d baud, %s polarity (%d%s preamble bits).\n",
					  st->baudrate, (polarity < 0) ? "normal" : "inverted",
					  preamble, (preamble >= 31) ? "+" : "");
				dsp_rx_lock(pocsag, st->baudrate, polarity);
				pocsag->fsk_rx_lastbit = st->lastbit;
				pocsag->fsk_rx_word = CODEWORD_SYNC;
				put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
				pocsag->fsk_rx_sync = 16;
				pocsag->fsk_rx_index = 0;
				*consumed = i + 1;
				return 1;
			}
			if (count_preamble_bits(st->word) >= MIN_PREAMBLE_BITS_BCH
			    && hamming32(w, CODEWORD_SYNC) <= SYNC_MAX_HAMMING) {
				uint32_t trial = w;
				if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
					int preamble = count_preamble_bits(st->word);
					LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected sync at %d baud, %s polarity (%d%s preamble bits).\n",
						  st->baudrate, (polarity < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, st->baudrate, polarity);
					pocsag->fsk_rx_lastbit = st->lastbit;
					pocsag->fsk_rx_word = CODEWORD_SYNC;
					put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
					pocsag->fsk_rx_sync = 16;
					pocsag->fsk_rx_index = 0;
					*consumed = i + 1;
					return 1;
				}
			}
			/* check inverted polarity (only if auto-detecting polarity) */
			if (pocsag->rx_auto_polarity) {
				if (w_inv == CODEWORD_SYNC) {
					double inv_pol = -polarity;
					int preamble = count_preamble_bits(st->word_inv);
					LOGP_CHAN(DDSP, LOGL_INFO, "Sync detected at %d baud, %s polarity (inverted, %d%s preamble bits).\n",
						  st->baudrate, (inv_pol < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, st->baudrate, inv_pol);
					pocsag->fsk_rx_lastbit = !st->lastbit;
					pocsag->fsk_rx_word = CODEWORD_SYNC;
					put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
					pocsag->fsk_rx_sync = 16;
					pocsag->fsk_rx_index = 0;
					*consumed = i + 1;
					return 1;
				}
				if (count_preamble_bits(st->word_inv) >= MIN_PREAMBLE_BITS_BCH
				    && hamming32(w_inv, CODEWORD_SYNC) <= SYNC_MAX_HAMMING) {
					uint32_t trial = w_inv;
					if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
						double inv_pol = -polarity;
						int preamble = count_preamble_bits(st->word_inv);
						LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected sync at %d baud, %s polarity (inverted, %d%s preamble bits).\n",
							  st->baudrate, (inv_pol < 0) ? "normal" : "inverted",
							  preamble, (preamble >= 31) ? "+" : "");
						dsp_rx_lock(pocsag, st->baudrate, inv_pol);
						pocsag->fsk_rx_lastbit = !st->lastbit;
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
	double phase, bitstep, polarity;
	int i;
	uint8_t lastbit;

	polarity = pocsag->fsk_polarity;
	phase = pocsag->fsk_rx_phase;
	lastbit = pocsag->fsk_rx_lastbit;
	bitstep = pocsag->fsk_bitstep;

	for (i = 0; i < length; i++) {
		if (*spl++ * polarity > 0.0) {
			if (lastbit) {
				phase += bitstep;
				if (phase >= 1.0) {
					phase -= 1.0;
					fsk_block_decode(pocsag, 1);
				}
			} else {
				phase = -0.5;
				fsk_block_decode(pocsag, 1);
				lastbit = 1;
			}
		} else {
			if (lastbit) {
				phase = -0.5;
				fsk_block_decode(pocsag, 0);
				lastbit = 0;
			} else {
				phase += bitstep;
				if (phase >= 1.0) {
					phase -= 1.0;
					fsk_block_decode(pocsag, 0);
				}
			}
		}
	}

	pocsag->fsk_rx_phase = phase;
	pocsag->fsk_rx_lastbit = lastbit;
}

/*
 * Between-batches decoder: runs the locked decoder AND the multi-rate
 * scanner in parallel on the same samples.
 *
 * The locked decoder searches for the next FSC at the current rate
 * (same transmission continuing). The scanner searches for a new
 * preamble+sync at any rate (new transmission starting).
 *
 * Whoever finds a match first wins:
 *   - Locked decoder finds FSC → continue at same rate (fsk_rx_sync set)
 *   - Scanner finds preamble+sync → switch to new rate (dsp_rx_lock called)
 *   - Resync timeout expires → unlock, scanner takes over
 *
 * We process sample-by-sample so both paths see every sample.
 */
static void fsk_decode_resync(pocsag_t *pocsag, sample_t *spl, int length)
{
	double phase, bitstep, polarity;
	int i, scan_consumed;
	uint8_t lastbit;

	polarity = pocsag->fsk_polarity;
	phase = pocsag->fsk_rx_phase;
	lastbit = pocsag->fsk_rx_lastbit;
	bitstep = pocsag->fsk_bitstep;

	for (i = 0; i < length; i++) {
		/* Feed one sample to the locked decoder (searching for next FSC) */
		if (spl[i] * polarity > 0.0) {
			if (lastbit) {
				phase += bitstep;
				if (phase >= 1.0) {
					phase -= 1.0;
					fsk_block_decode(pocsag, 1);
				}
			} else {
				phase = -0.5;
				fsk_block_decode(pocsag, 1);
				lastbit = 1;
			}
		} else {
			if (lastbit) {
				phase = -0.5;
				fsk_block_decode(pocsag, 0);
				lastbit = 0;
			} else {
				phase += bitstep;
				if (phase >= 1.0) {
					phase -= 1.0;
					fsk_block_decode(pocsag, 0);
				}
			}
		}

		/* Did the locked decoder find the next FSC? */
		if (pocsag->fsk_rx_sync) {
			/* Yes — save state and hand remaining samples to locked decoder */
			pocsag->fsk_rx_phase = phase;
			pocsag->fsk_rx_lastbit = lastbit;
			if (i + 1 < length)
				fsk_decode_locked(pocsag, spl + i + 1, length - i - 1);
			return;
		}

		/* Did the resync timeout expire (unlock happened)? */
		if (!pocsag->rx_rate_locked) {
			/* Yes — hand remaining samples to the scanner */
			pocsag->fsk_rx_phase = phase;
			pocsag->fsk_rx_lastbit = lastbit;
			if (i + 1 < length)
				fsk_decode(pocsag, spl + i + 1, length - i - 1);
			return;
		}

		/* Feed the same sample to the multi-rate scanner in parallel */
		if (pocsag->rx_auto_baud || pocsag->rx_auto_polarity) {
			if (fsk_decode_scan(pocsag, spl + i, 1, &scan_consumed)) {
				/*
				 * Scanner found preamble+sync at a different rate.
				 * dsp_rx_lock() was called, which reconfigured
				 * fsk_bitstep/fsk_polarity. Hand remaining samples
				 * to the locked decoder at the new rate.
				 */
				if (i + 1 < length)
					fsk_decode_locked(pocsag, spl + i + 1, length - i - 1);
				return;
			}
		}
	}

	pocsag->fsk_rx_phase = phase;
	pocsag->fsk_rx_lastbit = lastbit;
}

/*
 * Main FSK decode entry point.
 *
 * Three modes of operation:
 *
 * 1. Synced (fsk_rx_sync > 0): Actively decoding a batch at locked rate.
 *
 * 2. Rate-locked, between batches (rx_rate_locked && !fsk_rx_sync):
 *    Run locked decoder AND multi-rate scanner in parallel. Per POCSAG spec,
 *    all batches in a transmission use the same rate, so the locked decoder
 *    searches for the next FSC. But a new transmission at a different rate
 *    could start, so the scanner runs simultaneously on the same samples.
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


