/* FSK Symbol Timing PLL Implementation
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * Ported from the FLEX demodulator's flex_rx_build_symbol(), which is
 * itself derived from multimon-ng's FLEX decoder (1996, Thomas Sailer).
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

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "fsk_pll.h"

void fsk_pll_init(fsk_pll_t *pll, int samplerate, int baudrate)
{
	memset(pll, 0, sizeof(*pll));

	pll->samplerate = samplerate;
	pll->baudrate = baudrate;

	/* Phase accumulator: range = 100 * samplerate (matching FLEX).
	 * This gives integer arithmetic with no truncation error:
	 *   phase_rate = phase_max * baudrate / samplerate
	 *             = 100 * samplerate * baudrate / samplerate
	 *             = 100 * baudrate (exact integer) */
	pll->phase_max = 100LL * (int64_t)samplerate;
	pll->phase_rate = 100LL * (int64_t)baudrate;
	pll->phase = 0;

	/* Zero crossing detection */
	pll->last_sample = 0.0;

	/* DC offset */
	pll->dc_offset = 0.0;
	pll->dc_enabled = 1;

	/* Majority voting */
	pll->vote_high = 0;
	pll->vote_low = 0;

	/* Signal quality */
	pll->nonconsec = 0;
	pll->timeout = 0;

	/* Signal level */
	pll->level_sum = 0.0;
	pll->level_count = 0;

	/* Lock state */
	pll->locked = 0;
}

void fsk_pll_reset(fsk_pll_t *pll)
{
	pll->phase = 0;
	pll->last_sample = 0.0;
	pll->dc_offset = 0.0;

	pll->vote_high = 0;
	pll->vote_low = 0;

	pll->nonconsec = 0;
	pll->timeout = 0;

	pll->level_sum = 0.0;
	pll->level_count = 0;

	pll->locked = 0;
}

void fsk_pll_copy_state(fsk_pll_t *dst, const fsk_pll_t *src)
{
	/* Copy timing state */
	dst->phase = src->phase;
	dst->last_sample = src->last_sample;

	/* Copy DC offset */
	dst->dc_offset = src->dc_offset;

	/* Copy votes (in-progress symbol) */
	dst->vote_high = src->vote_high;
	dst->vote_low = src->vote_low;

	/* Copy quality state */
	dst->nonconsec = src->nonconsec;
	dst->timeout = src->timeout;

	/* Copy level tracking */
	dst->level_sum = src->level_sum;
	dst->level_count = src->level_count;
}

int fsk_pll_process(fsk_pll_t *pll, sample_t sample, double polarity, uint8_t *bit_out)
{
	double val = sample * polarity;
	double phasepercent;

	/* Step 1: DC offset removal (IIR, matching FLEX).
	 * dc = dc * (sr * tc) / (sr * tc + 1) + sample / (sr * tc + 1)
	 * Only active when enabled. */
	if (pll->dc_enabled) {
		double tc_samples = (double)pll->samplerate * FSK_PLL_DC_OFFSET_FILTER;
		pll->dc_offset = (pll->dc_offset * tc_samples + val) / (tc_samples + 1.0);
	}
	val -= pll->dc_offset;

	/* Track signal level (mean of |sample|) */
	pll->level_sum += fabs(val);
	pll->level_count++;

	/* Step 2: Phase percentage for voting window and crossing classification */
	phasepercent = 100.0 * (double)pll->phase / (double)pll->phase_max;

	/* Step 3: Majority voting in the middle 80% of the symbol period. */
	if (phasepercent > 10.0 && phasepercent < 90.0) {
		if (val > 0.0)
			pll->vote_high++;
		else
			pll->vote_low++;
	}

	/* Step 4: Zero crossing detection on RAW sample values.
	 * Crossings are detected on the actual analog sample (after DC removal),
	 * not on a quantized sign. This gives the PLL cleaner timing information.
	 * Matches FLEX: (last_sample < 0 && sample >= 0) || (last_sample >= 0 && sample < 0) */
	if ((pll->last_sample < 0.0 && val >= 0.0) ||
	    (pll->last_sample >= 0.0 && val < 0.0)) {
		double phase_error;

		/* Proportional correction: compute actual phase error.
		 * If crossing is in first half of symbol (phase < 50%),
		 * the error is the current phase (we're late).
		 * If in second half (phase > 50%), the error is
		 * phase - phase_max (we're early, negative error). */
		if (phasepercent < 50.0)
			phase_error = (double)pll->phase;
		else
			phase_error = (double)pll->phase - (double)pll->phase_max;

		/* Apply correction with rate depending on lock state */
		if (pll->locked)
			pll->phase -= (int64_t)(phase_error * FSK_PLL_PHASE_LOCKED_RATE);
		else
			pll->phase -= (int64_t)(phase_error * FSK_PLL_PHASE_UNLOCKED_RATE);

		/* Mid-symbol crossing classification.
		 * Crossings in the middle 80% (10%-90%) are near the expected
		 * transition point — PLL is tracking. Crossings outside this
		 * range (near symbol centers) indicate noise or wrong baud rate. */
		if (phasepercent > 10.0 && phasepercent < 90.0) {
			pll->nonconsec++;
		} else {
			pll->nonconsec = 0;
		}

		pll->timeout = 0;
	}
	pll->last_sample = val;

	/* Step 5: Advance phase accumulator */
	pll->phase += pll->phase_rate;

	/* Step 6: Symbol boundary — decide bit by majority vote */
	if (pll->phase >= pll->phase_max) {
		pll->phase -= pll->phase_max;

		/* Reset nonconsec at symbol boundary (matching FLEX).
		 * Nonconsec only counts bad crossings within one symbol period,
		 * not across symbols. */
		pll->nonconsec = 0;

		/* Decide bit by majority vote.
		 * If no votes (e.g., first symbol), fall back to instantaneous. */
		if (pll->vote_high + pll->vote_low > 0)
			*bit_out = (pll->vote_high >= pll->vote_low) ? 1 : 0;
		else
			*bit_out = (val > 0.0) ? 1 : 0;

		pll->vote_high = 0;
		pll->vote_low = 0;

		/* Timeout tracking */
		pll->timeout++;
		if (pll->timeout > FSK_PLL_TIMEOUT_MAX) {
			pll->locked = 0;
		}

		return 1;
	}

	return 0;
}
