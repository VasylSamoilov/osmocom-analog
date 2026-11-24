/* AFC for the FSK paging receivers (FLEX, POCSAG, GSC)
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "sender.h"
#include "afc.h"

void afc_init(afc_t *afc, struct sender *sender, double range_hz, double dev_hz)
{
	memset(afc, 0, sizeof(*afc));
	afc->sender = sender;
	afc->range = (range_hz > 0.0) ? range_hz : 0.0;
	afc->dev_hz = (dev_hz > 0.0) ? dev_hz : 1.0;
}

void afc_symbol(afc_t *afc, double raw)
{
	afc->hist[afc->hist_pos] = raw;
	afc->hist_pos = (afc->hist_pos + 1) % AFC_HIST;
	if (afc->hist_n < AFC_HIST)
		afc->hist_n++;
}

/* the symbol i of the last n (i = 0 oldest), symbols_ago before the newest */
static double hist_at(const afc_t *afc, int n, int symbols_ago, int i)
{
	return afc->hist[(afc->hist_pos + 2 * AFC_HIST - symbols_ago - n + i) % AFC_HIST];
}

int afc_hunt(const afc_t *afc, int n, double *dc, double *dev)
{
	double s = 0.0, d = 0.0;
	int i;

	if (n > afc->hist_n)
		n = afc->hist_n;
	if (n <= 0)
		return 0;
	for (i = 0; i < n; i++)
		s += hist_at(afc, n, 0, i);
	s /= n;
	for (i = 0; i < n; i++)
		d += fabs(hist_at(afc, n, 0, i) - s);
	*dc = s;
	*dev = d / n;
	return n;
}

int afc_fit(const afc_t *afc, const double *levels, int n, int symbols_ago, double dev_expected, double *dc, double *dev)
{
	double sx = 0.0, sl = 0.0, sxl = 0.0, sll = 0.0, ml, mx, var, d;
	int i;

	if (n < 2 || symbols_ago < 0 || symbols_ago + n > afc->hist_n)
		return 0;
	for (i = 0; i < n; i++) {
		double x = hist_at(afc, n, symbols_ago, i), l = levels[i];

		sx += x;
		sl += l;
		sxl += x * l;
		sll += l * l;
	}
	ml = sl / n;
	mx = sx / n;
	var = sll / n - ml * ml;
	if (var < 1e-6)
		return 0;
	d = (sxl / n - mx * ml) / var;
	/* false sync guard (SDRangel: 0.6 .. 1.6 of the deviation it hunted) */
	if (fabs(d) < 0.05 || (dev_expected > 0.0 && (fabs(d) < 0.6 * dev_expected || fabs(d) > 1.6 * dev_expected)))
		return 0;
	*dc = mx - d * ml;
	*dev = d;
	return 1;
}

/* Set the trim. The DC seen by the decoder moves by -delta from the next
 * block on (the demodulator turns then). */
static void afc_set_trim(afc_t *afc, double hz)
{
	double applied, delta;

	if (hz > afc->range)
		hz = afc->range;
	if (hz < -afc->range)
		hz = -afc->range;
	applied = sender_set_rx_trim(afc->sender, hz);
	delta = applied - afc->trim;
	if (delta == 0.0)
		return;
	afc->trim = applied;
	afc->pending -= delta / afc->dev_hz;
}

void afc_measured(afc_t *afc, double dc)
{
	if (afc->range <= 0.0)
		return;
	if (afc->fits == 0)
		afc->confirmed = 0;
	afc->fits++;
	afc->last_fit_hz = dc * afc->dev_hz;
	afc_set_trim(afc, afc->trim + afc->last_fit_hz);
}

void afc_release(afc_t *afc, int decoded)
{
	if (afc->range <= 0.0 || afc->fits == 0)
		return;
	if (afc->fits >= 2 || decoded)
		afc->confirmed = 1;
	if (afc->confirmed)
		afc->hunt_centre = afc->trim;
	afc->fits = 0;
	afc_set_trim(afc, afc->hunt_centre);
}

double afc_block(afc_t *afc, int length, int samplerate)
{
	double p;
	int i;

	if (afc->range > 0.0 && samplerate > 0) {
		afc->samples += length;
		if (afc->samples >= samplerate) {
			afc->samples -= samplerate;
			if (afc->fits == 0 && afc->hunt_centre != 0.0) {
				afc->hunt_centre *= 0.967216;	/* exp(-1/30) */
				if (fabs(afc->hunt_centre) < 10.0)
					afc->hunt_centre = 0.0;
				afc_set_trim(afc, afc->hunt_centre);
			}
		}
	}
	p = afc->pending;
	if (p != 0.0) {
		for (i = 0; i < AFC_HIST; i++)
			afc->hist[i] += p;
		afc->pending = 0.0;
	}
	return p;
}
