/* FSK of NRZ levels: shaped baseband levels for the FM deviation
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

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "fsk_nrz.h"

/* The levels, in the deviation: 2 levels use 0 and 3 */
static const double fsk_nrz_levels[FSK_NRZ_LEVELS] = { -1.0, -1.0/3.0, 1.0/3.0, 1.0 };

/* A symbol ends when the phase reaches 256. The step is samples per symbol
 * in 1/256 symbol, rounded in floating point: 30 steps of 256/30 can sum to
 * just under 256 and give a symbol one sample too long. The tolerance is far
 * below a sample, so symbols of a whole number of samples are exact. */
#ifndef FSK_NRZ_PHASE_END
#define FSK_NRZ_PHASE_END	(256.0 - 1e-6)
#endif

/* The ramps for all level changes in the polarity */
static void fsk_nrz_mod_ramps(fsk_nrz_mod_t *mod)
{
	int from, to, i;
	double lf, lt, mid, amp, c;

	for (from = 0; from < FSK_NRZ_LEVELS; from++) {
		lf = fsk_nrz_levels[from] * mod->deviation * mod->polarity;
		for (to = 0; to < FSK_NRZ_LEVELS; to++) {
			lt = fsk_nrz_levels[to] * mod->deviation * mod->polarity;
			mid = (lf + lt) / 2.0;
			amp = (lf - lt) / 2.0;
			for (i = 0; i < 256; i++) {
				/* the old level for a quarter, the change in the middle half */
				if (i < 64)
					c = 1.0;
				else if (i >= 192)
					c = -1.0;
				else
					c = cos((double)(i - 64) / 128.0 * M_PI);
				mod->ramps[from][to][i] = mid + amp * c;
			}
		}
	}
}

void fsk_nrz_mod_init(fsk_nrz_mod_t *mod, int samplerate, double symbol_rate, double deviation, double polarity)
{
	memset(mod, 0, sizeof(*mod));
	mod->samplerate = samplerate;
	mod->deviation = deviation;
	mod->polarity = polarity;
	fsk_nrz_mod_ramps(mod);
	fsk_nrz_mod_set_rate(mod, symbol_rate);
	fsk_nrz_mod_restart(mod);
}

/* A change of symbol rate keeps the time the phase stands for: the phase is
 * a fraction of the old symbol length, the next sample is as far into the
 * next symbol at the new length (no sample added or lost, the first symbol
 * at the new rate is full length). */
void fsk_nrz_mod_set_rate(fsk_nrz_mod_t *mod, double symbol_rate)
{
	double old_symbol_samples = mod->symbol_samples;

	mod->symbol_samples = (double)mod->samplerate / symbol_rate;
	mod->step = 1.0 / mod->symbol_samples * 256.0;
	if (old_symbol_samples > 0.0 && old_symbol_samples != mod->symbol_samples)
		mod->phase *= old_symbol_samples / mod->symbol_samples;
}

/* The levels are numbered in the polarity: the level on the air (the
 * deviation the last symbol ended at) has the opposite number in the other
 * polarity, so the next symbol starts from where the signal is. */
void fsk_nrz_mod_set_polarity(fsk_nrz_mod_t *mod, double polarity)
{
	if (mod->polarity == polarity)
		return;
	mod->polarity = polarity;
	fsk_nrz_mod_ramps(mod);
	mod->level = FSK_NRZ_LEVELS - 1 - mod->level;
}

/* A transmission after silence: a full first symbol from the lowest level */
void fsk_nrz_mod_restart(fsk_nrz_mod_t *mod)
{
	mod->phase = 0.0;
	mod->level = 0;
	mod->silence = 0;
}

/* Silence (or other audio) was sent: the next symbol starts a transmission */
void fsk_nrz_mod_silence(fsk_nrz_mod_t *mod)
{
	mod->silence = 1;
}

/* One symbol of the level (0..3) into spl; returns the number of samples */
int fsk_nrz_mod_symbol(fsk_nrz_mod_t *mod, int level, sample_t *spl)
{
	sample_t *start = spl;
	double phase, step;

	if (mod->silence)
		fsk_nrz_mod_restart(mod);
	phase = mod->phase;
	step = mod->step;

	if (level == mod->level) {
		/* stay */
		sample_t lev = fsk_nrz_levels[level] * mod->deviation * mod->polarity;
		do {
			*spl++ = lev;
			phase += step;
		} while (phase < FSK_NRZ_PHASE_END);
	} else {
		/* ramp from the level on the air */
		const sample_t *ramp = mod->ramps[mod->level][level];
		do {
			*spl++ = ramp[(uint8_t)phase];
			phase += step;
		} while (phase < FSK_NRZ_PHASE_END);
		mod->level = level;
	}
	mod->phase = phase - 256.0;
	mod->last = spl[-1];

	return spl - start;
}

/* The end of a transmission: the last sample held for a symbol and one sample
 * more (at most max), so a receiver still has the last symbol where its clock
 * may have drifted to, not silence */
int fsk_nrz_mod_guard(fsk_nrz_mod_t *mod, sample_t *spl, int max)
{
	int guard = (int)(mod->symbol_samples + 1.0);
	int i;

	for (i = 0; i < guard && i < max; i++)
		spl[i] = mod->last;

	return i;
}

/* 2 levels: nbits bits of word, MSB first, a 1 is the upper level */
int fsk_nrz_mod_bits(fsk_nrz_mod_t *mod, uint32_t word, int nbits, sample_t *spl)
{
	int i, count = 0;

	for (i = 0; i < nbits; i++) {
		count += fsk_nrz_mod_symbol(mod, (word & 0x80000000) ? FSK_NRZ_LEVELS - 1 : 0, spl + count);
		word <<= 1;
	}

	return count;
}

/* Buffer size for a number of symbols at a rate (with the sample a symbol
 * can have more from rounding, and some room) */
int fsk_nrz_mod_max_samples(double samplerate, double symbol_rate, int symbols)
{
	return (int)(samplerate / symbol_rate * (double)symbols) + symbols + 10;
}
