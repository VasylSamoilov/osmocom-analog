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
	/* AFC: a sample value of 1.0 is the deviation (Hz) */
	afc_init(&pocsag->afc, &pocsag->sender, pocsag_afc_range, deviation);

	pocsag->fsk_bitduration = (double)samplerate / (double)baudrate;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Use %.4f samples for one bit duration @ %d.\n", pocsag->fsk_bitduration, pocsag->sender.samplerate);

	pocsag->fsk_tx_buffer_size = pocsag->fsk_bitduration * 32.0 + 10; /* 32 bit, add some extra to prevent short buffer due to rounding */
	pocsag->fsk_tx_buffer = calloc(pocsag->fsk_tx_buffer_size, sizeof(sample_t));
	if (!pocsag->fsk_tx_buffer) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* the modulator: deviation 1.0 is what we set at sender_set_fm() */
	pocsag->fsk_polarity = polarity;
	fsk_nrz_mod_init(&pocsag->fsk_mod, samplerate, (double)baudrate, 1.0, polarity);

	/* store auto-detection config */
	pocsag->rx_auto_baud = auto_baud;
	pocsag->rx_auto_polarity = auto_polarity;
	pocsag->rx_baud_locked = baudrate;
	pocsag->rx_polarity_locked = polarity;
	pocsag->samplerate = samplerate;

	/* init locked decoder PLL */
	fsk_pll_init(&pocsag->fsk_rx_pll, samplerate, baudrate);

	/* init multi-rate RX decoders */
	if (auto_baud) {
		n = 3;
		for (i = 0; i < 3; i++) {
			pocsag->rx_baud[i].baudrate = rates[i];
			fsk_pll_init(&pocsag->rx_baud[i].pll, samplerate, rates[i]);
			pocsag->rx_baud[i].word = 0;
			pocsag->rx_baud[i].word_inv = 0;
		}
		LOGP_CHAN(DDSP, LOGL_INFO, "RX baud rate: auto-detect (512/1200/2400).\n");
	} else {
		n = 1;
		{
			pocsag->rx_baud[0].baudrate = baudrate;
			fsk_pll_init(&pocsag->rx_baud[0].pll, samplerate, baudrate);
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

/* The speed and polarity of the group to send (a transmission starts with
 * its preamble): what the scheduler takes the messages for, and what the
 * modulator sends them at, with room for a codeword at that speed */
void dsp_set_tx_group(pocsag_t *pocsag, int speed, double polarity)
{
	int size;

	pocsag->tx_speed = speed;
	pocsag->tx_polarity = polarity;
	fsk_nrz_mod_set_rate(&pocsag->fsk_mod, (double)speed);
	fsk_nrz_mod_set_polarity(&pocsag->fsk_mod, polarity);
	size = (int)(pocsag->fsk_mod.symbol_samples * 32.0 + 10);
	if (size > pocsag->fsk_tx_buffer_size) {
		sample_t *nb = realloc(pocsag->fsk_tx_buffer, size * sizeof(sample_t));
		if (nb) {
			pocsag->fsk_tx_buffer = nb;
			pocsag->fsk_tx_buffer_size = size;
		}
	}
}


/* encode one codeward into samples
 * input: 32 data bits
 * output: samples
 * return number of samples */
static int fsk_block_encode(pocsag_t *pocsag, uint32_t word)
{
	return fsk_nrz_mod_bits(&pocsag->fsk_mod, word, 32, pocsag->fsk_tx_buffer);
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

	/* Initialize locked decoder PLL at the detected rate.
	 * Phase and state are copied from the scanner slot by the caller
	 * via fsk_pll_copy_state(), so the PLL continues seamlessly. */
	fsk_pll_init(&pocsag->fsk_rx_pll, pocsag->samplerate, baudrate);
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

	/* the transmission ended: the AFC's reception is confirmed by a
	 * message (or two sync words) */
	afc_release(&pocsag->afc, pocsag->afc_decoded);
	pocsag->afc_decoded = 0;

	for (i = 0; i < pocsag->rx_baud_count; i++) {
		fsk_pll_reset(&pocsag->rx_baud[i].pll);
		pocsag->rx_baud[i].word = 0;
		pocsag->rx_baud[i].word_inv = 0;
	}
}

/*
 * AFC (libmobile/afc.c, as SDRangel): at each sync codeword the levels of its
 * 32 symbols are fitted by least squares; the DC found is held for the batch
 * and moves the channel's demodulator onto the signal.
 *
 * pll: the PLL that decoded the sync word (its last 32 symbol values), pol:
 * its polarity (its values are sample * pol), inverted: its bits were ~SYNC.
 * Returns 1 and the DC in sample units (relative to the demodulator now).
 */
double pocsag_afc_range = 5000.0;	/* --afc-range (Hz), 0 = off */

static int pocsag_afc_sync(pocsag_t *pocsag, const fsk_pll_t *pll, double pol, int inverted, double *dc)
{
	uint32_t pat = inverted ? ~(uint32_t)CODEWORD_SYNC : (uint32_t)CODEWORD_SYNC;
	double levels[32], dev, expected;
	int i;

	if (pocsag->afc.range <= 0.0 || pll->sym_hist_n < 32)
		return 0;
	/* the deviation hunted (SDRangel: mean distance from the mean of the
	 * last symbols): the 32 before the sync word (the preamble or the last
	 * codeword), if there */
	expected = 0.0;
	if (pll->sym_hist_n >= 64) {
		double m = 0.0;

		for (i = 0; i < 32; i++)
			m += pll->sym_hist[(pll->sym_hist_pos + FSK_PLL_SYM_HIST - 64 + i) % FSK_PLL_SYM_HIST];
		m /= 32.0;
		for (i = 0; i < 32; i++)
			expected += fabs(pll->sym_hist[(pll->sym_hist_pos + FSK_PLL_SYM_HIST - 64 + i) % FSK_PLL_SYM_HIST] - m);
		expected /= 32.0;
	}
	/* the 32 symbols of the sync word into the AFC, oldest first, in sample
	 * units; a bit 1 was a positive value of sample * pol */
	for (i = 0; i < 32; i++) {
		afc_symbol(&pocsag->afc, pll->sym_hist[(pll->sym_hist_pos + FSK_PLL_SYM_HIST - 32 + i) % FSK_PLL_SYM_HIST] * pol);
		levels[i] = (((pat >> (31 - i)) & 1) ? 1.0 : -1.0) * pol;
	}
	if (!afc_fit(&pocsag->afc, levels, 32, 0, expected, dc, &dev)) {
		LOGP_CHAN(DDSP, LOGL_INFO, "AFC: sync word levels do not fit (level %.2f), not used.\n", expected);
		return 0;
	}
	LOGP_CHAN(DDSP, LOGL_INFO, "AFC: carrier %+.0f Hz off the demodulator, trim %+.0f Hz -> %+.0f Hz (deviation %.2f).\n",
		  *dc * pocsag->afc.dev_hz, pocsag->afc.trim_block, pocsag->afc.trim_block + *dc * pocsag->afc.dev_hz, fabs(dev));
	afc_measured(&pocsag->afc, *dc);
	return 1;
}

/* after a lock by the scanner: fit the sync word of the scanner's PLL, the
 * locked decoder takes the fitted DC (in its polarity) */
static void pocsag_afc_lock(pocsag_t *pocsag, const fsk_pll_t *scan_pll, double scan_pol, int inverted)
{
	double dc;

	if (pocsag_afc_sync(pocsag, scan_pll, scan_pol, inverted, &dc))
		pocsag->fsk_rx_pll.dc_offset = dc * pocsag->fsk_polarity;
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
				/* AFC: the levels of this sync word */
				{
					double dc;

					if (pocsag_afc_sync(pocsag, &pocsag->fsk_rx_pll, pocsag->fsk_polarity, 0, &dc))
						pocsag->fsk_rx_pll.dc_offset = dc * pocsag->fsk_polarity;
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
					double new_pol = -pocsag->fsk_polarity, afc_dc;
					int preamble = count_preamble_bits(~pocsag->fsk_rx_word);
					/* AFC: fit before the lock clears the PLL */
					int afc_ok = pocsag_afc_sync(pocsag, &pocsag->fsk_rx_pll, pocsag->fsk_polarity, 1, &afc_dc);
					LOGP_CHAN(DDSP, LOGL_INFO, "Inverted FSC in locked mode — flipping to %s polarity (%d%s preamble bits).\n",
						  (new_pol < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, pocsag->rx_baud_locked, new_pol);
					pocsag->fsk_rx_pll.last_sample = -pocsag->fsk_rx_pll.last_sample;
					pocsag->fsk_rx_pll.dc_offset = -pocsag->fsk_rx_pll.dc_offset;
					if (afc_ok)
						pocsag->fsk_rx_pll.dc_offset = afc_dc * new_pol;
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
				LOGP_CHAN(DDSP, LOGL_DEBUG, "No FSC found within %d bits after batch end, unlocking (last word=0x%08x, bits_since_fsc=%d).\n",
					  RESYNC_TIMEOUT_BITS, w, pocsag->fsk_rx_bit_count);
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
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Batch complete, searching for next FSC (last_cw=0x%08x).\n",
					  (uint32_t)pocsag->fsk_rx_word);
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

			/* PLL-based FSK demod per slot */
			if (!fsk_pll_process(&st->pll, sample, polarity, &bit_val))
				continue;

			/* Debug: log scanner state periodically for slot 1 (1200 baud) */
			{
				static int dbg_cnt = 0;
				if (s == 1 && (dbg_cnt++ % 1000) == 0)
					LOGP_CHAN(DDSP, LOGL_INFO, "SCAN[%d]: bit=%d nc=%d phase=%lld/%lld sample=%.4f dc=%.4f level=%.4f\n",
						  st->baudrate, bit_val,
						  fsk_pll_get_nonconsec(&st->pll),
						  (long long)st->pll.phase, (long long)st->pll.phase_max,
						  sample, st->pll.dc_offset,
						  fsk_pll_get_level(&st->pll));
			}

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
			int nc = fsk_pll_get_nonconsec(&st->pll);

			/* check normal polarity */
			if (w == CODEWORD_SYNC) {
				if (nc >= FSK_PLL_NONCONSEC_MAX_SYNC) {
					LOGP_CHAN(DDSP, LOGL_DEBUG, "Sync candidate rejected at %d baud, %s polarity (nonconsec=%d).\n",
						  st->baudrate, (polarity < 0) ? "normal" : "inverted", nc);
					goto check_inverted;
				}
				int preamble = count_preamble_bits(st->word);
				LOGP_CHAN(DDSP, LOGL_INFO, "Sync detected at %d baud, %s polarity (%d%s preamble bits).\n",
					  st->baudrate, (polarity < 0) ? "normal" : "inverted",
					  preamble, (preamble >= 31) ? "+" : "");
				dsp_rx_lock(pocsag, st->baudrate, polarity);
				fsk_pll_copy_state(&pocsag->fsk_rx_pll, &st->pll);
				pocsag_afc_lock(pocsag, &st->pll, polarity, 0);
				pocsag->fsk_rx_word = CODEWORD_SYNC;
				put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
				pocsag->fsk_rx_sync = 16;
				pocsag->fsk_rx_index = 0;
				*consumed = i + 1;
				return 1;
			}
			if (nc < FSK_PLL_NONCONSEC_MAX_SYNC
			    && count_preamble_bits(st->word) >= MIN_PREAMBLE_BITS_BCH
			    && hamming32(w, CODEWORD_SYNC) <= SYNC_MAX_HAMMING) {
				uint32_t trial = w;
				if (pocsag_bch_correct(&trial, NULL) == 0 && trial == CODEWORD_SYNC) {
					int preamble = count_preamble_bits(st->word);
					LOGP_CHAN(DDSP, LOGL_INFO, "BCH-corrected sync at %d baud, %s polarity (%d%s preamble bits).\n",
						  st->baudrate, (polarity < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, st->baudrate, polarity);
					fsk_pll_copy_state(&pocsag->fsk_rx_pll, &st->pll);
					pocsag_afc_lock(pocsag, &st->pll, polarity, 0);
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
					if (nc >= FSK_PLL_NONCONSEC_MAX_SYNC)
						continue;
					double inv_pol = -polarity;
					int preamble = count_preamble_bits(st->word_inv);
					LOGP_CHAN(DDSP, LOGL_INFO, "Sync detected at %d baud, %s polarity (inverted, %d%s preamble bits).\n",
						  st->baudrate, (inv_pol < 0) ? "normal" : "inverted",
						  preamble, (preamble >= 31) ? "+" : "");
					dsp_rx_lock(pocsag, st->baudrate, inv_pol);
					fsk_pll_copy_state(&pocsag->fsk_rx_pll, &st->pll);
					/* Flip last_sample and DC offset sign for inverted polarity */
					pocsag->fsk_rx_pll.last_sample = -pocsag->fsk_rx_pll.last_sample;
					pocsag->fsk_rx_pll.dc_offset = -pocsag->fsk_rx_pll.dc_offset;
					pocsag_afc_lock(pocsag, &st->pll, polarity, 1);
					pocsag->fsk_rx_word = CODEWORD_SYNC;
					put_codeword(pocsag, CODEWORD_SYNC, -1, -1);
					pocsag->fsk_rx_sync = 16;
					pocsag->fsk_rx_index = 0;
					*consumed = i + 1;
					return 1;
				}
				if (nc < FSK_PLL_NONCONSEC_MAX_SYNC
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
						fsk_pll_copy_state(&pocsag->fsk_rx_pll, &st->pll);
						pocsag->fsk_rx_pll.last_sample = -pocsag->fsk_rx_pll.last_sample;
						pocsag->fsk_rx_pll.dc_offset = -pocsag->fsk_rx_pll.dc_offset;
						pocsag_afc_lock(pocsag, &st->pll, polarity, 1);
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
	double polarity = pocsag->fsk_polarity;
	int i;

	fsk_pll_set_locked(&pocsag->fsk_rx_pll, 1);
	fsk_pll_set_dc_removal(&pocsag->fsk_rx_pll, 0); /* freeze DC during data decode */

	for (i = 0; i < length; i++) {
		uint8_t bit_val;

		if (!fsk_pll_process(&pocsag->fsk_rx_pll, spl[i], polarity, &bit_val))
			continue;

		fsk_block_decode(pocsag, bit_val);
	}
}

/*
 * Between-batches decoder: runs the locked decoder PLL searching for
 * the next FSC at the current rate. The scanner is NOT run here — it
 * only runs when fully unlocked (transmission ended).
 */
static void fsk_decode_resync(pocsag_t *pocsag, sample_t *spl, int length)
{
	double polarity = pocsag->fsk_polarity;
	int i;

	fsk_pll_set_locked(&pocsag->fsk_rx_pll, 1);

	for (i = 0; i < length; i++) {
		uint8_t bit_val;

		if (fsk_pll_process(&pocsag->fsk_rx_pll, spl[i], polarity, &bit_val))
			fsk_block_decode(pocsag, bit_val);

		/* Did the locked decoder find the next FSC? */
		if (pocsag->fsk_rx_sync) {
			if (i + 1 < length)
				fsk_decode_locked(pocsag, spl + i + 1, length - i - 1);
			return;
		}

		/* Did the resync timeout expire? Unlock and let scanner take over. */
		if (!pocsag->rx_rate_locked) {
			if (i + 1 < length)
				fsk_decode(pocsag, spl + i + 1, length - i - 1);
			return;
		}
	}
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
	double p;
	int i;

	if (!pocsag->rx)
		return;
	/* AFC: a trim set while the last block was decoded turns the
	 * demodulator from this block on: the DC estimates follow it here
	 * (sample units, the PLLs work on sample * polarity) */
	p = afc_block(&pocsag->afc, length, pocsag->samplerate);
	if (p != 0.0) {
		pocsag->fsk_rx_pll.dc_offset += p * pocsag->fsk_polarity;
		for (i = 0; i < FSK_PLL_SYM_HIST; i++)
			pocsag->fsk_rx_pll.sym_hist[i] += p * pocsag->fsk_polarity;
		for (i = 0; i < pocsag->rx_baud_count; i++) {
			int h;

			pocsag->rx_baud[i].pll.dc_offset += p * pocsag->fsk_polarity;
			for (h = 0; h < FSK_PLL_SYM_HIST; h++)
				pocsag->rx_baud[i].pll.sym_hist[h] += p * pocsag->fsk_polarity;
		}
	}
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
			/* Append one extra bit duration of the last FSK level
			 * as a guard before going silent.  This prevents the
			 * receiver PLL from drifting into silence during the
			 * last symbol's vote window, which can cause CRC
			 * errors in one-shot operation.
			 *
			 * fsk_tx_buffer_length is already 0 (cleared by the
			 * output loop), but fsk_tx_buffer_pos still equals the
			 * old length and the sample data is still in the buffer. */
			if (pocsag->fsk_tx_buffer_pos > 0 &&
			    !pocsag->network_mode &&
			    pocsag->scan_from >= pocsag->scan_to &&
			    !pocsag->sender.loopback) {
				int gi = fsk_nrz_mod_guard(&pocsag->fsk_mod, samples, length);
				LOGP_CHAN(DDSP, LOGL_NOTICE,
					  "TX guard: %d samples (last_val=%.3f) -- no more codewords\n",
					  gi, pocsag->fsk_mod.last);
				memset(power, 1, gi);
				samples += gi;
				power += gi;
				length -= gi;
			}
			pocsag->fsk_tx_buffer_pos = 0;
			if (length > 0) {
				memset(samples, 0, sizeof(*samples) * length);
				memset(power, 0, length);
			}
			/* the next codeword starts a new transmission (also after
			 * the gap of a speed or polarity switch) */
			fsk_nrz_mod_silence(&pocsag->fsk_mod);
			return;
		}

		/* encode */
		pocsag->fsk_tx_buffer_length = fsk_block_encode(pocsag, word);
		pocsag->fsk_tx_buffer_pos = 0;
	}

	/* send encoded word until end of source or destination buffer is reached */
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


