/* ERMES receiver: signal part (symbol clock, levels)
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

#include <string.h>
#include <math.h>
#include "dsp.h"

#define TIMING_GAIN	0.02	/* Gardner loop: phase correction per symbol (of the error) */
#define TIMING_LIMIT	0.05	/* at most this part of a symbol per symbol */
#define DC_GAIN		0.01	/* decision-directed offset, per symbol */
#define LEVEL_GAIN	0.005	/* decision-directed outer level, per symbol */

/* levels lowest to highest, as fractions of the outer deviation, and their dibits (§8.3, the
 * modulator's table) */
static const double level_value[4] = { -1.0, -1.0 / 3.0, 1.0 / 3.0, 1.0 };
static const int level_dibit[4] = { 0, 1, 3, 2 };

void ermes_dsp_rx_init(ermes_dsp_rx_t *dsp, ermes_rx_t *rx, int samplerate, double polarity)
{
	memset(dsp, 0, sizeof(*dsp));
	dsp->rx = rx;
	dsp->step = (double)ERMES_SYMBOL_RATE / samplerate;
	dsp->gain = 1.0;
	dsp->polarity = (polarity < 0) ? -1.0 : 1.0;
}

/* the level nearest to v (DC removed, gain applied) */
static int slice(double v)
{
	if (v < -2.0 / 3.0)
		return 0;
	if (v < 0.0)
		return 1;
	if (v < 2.0 / 3.0)
		return 2;
	return 3;
}

static void symbol(ermes_dsp_rx_t *dsp, double y)
{
	double v = (y - dsp->dc) / dsp->gain, e, err;
	int l = slice(v);

	/* Gardner: the value between two symbols points to which side the clock is off */
	if (dsp->have_mid) {
		e = (dsp->mid - dsp->dc) * ((dsp->prev - dsp->dc) - (y - dsp->dc)) / (dsp->gain * dsp->gain);
		e *= TIMING_GAIN;
		if (e > TIMING_LIMIT)
			e = TIMING_LIMIT;
		if (e < -TIMING_LIMIT)
			e = -TIMING_LIMIT;
		dsp->phase -= e;
	}
	dsp->prev = y;
	dsp->have_mid = 0;

	/* levels: offset and gain follow the decisions */
	err = v - level_value[l];
	dsp->dc += DC_GAIN * err * dsp->gain;
	if (l == 0 || l == 3)
		dsp->gain += LEVEL_GAIN * (fabs(v) - 1.0) * dsp->gain;
	if (dsp->gain < 0.2)
		dsp->gain = 0.2;
	if (dsp->gain > 5.0)
		dsp->gain = 5.0;

	dsp->symbols++;
	ermes_rx_dibit(dsp->rx, level_dibit[l]);
}

void ermes_dsp_rx_samples(ermes_dsp_rx_t *dsp, const double *samples, int length)
{
	double s, frac, y;
	int i;

	for (i = 0; i < length; i++) {
		s = samples[i] * dsp->polarity;
		dsp->phase += dsp->step;
		/* half way: the value between the symbols */
		if (!dsp->have_mid && dsp->phase >= 0.5 && dsp->phase - dsp->step < 0.5) {
			frac = (dsp->phase - 0.5) / dsp->step;	/* how far back from this sample */
			dsp->mid = s - frac * (s - dsp->last);
			dsp->have_mid = 1;
		}
		/* the centre of a symbol */
		if (dsp->phase >= 1.0) {
			dsp->phase -= 1.0;
			frac = dsp->phase / dsp->step;
			y = s - frac * (s - dsp->last);
			symbol(dsp, y);
		}
		dsp->last = s;
	}
}
