/* The device clock's drift against the PC's clocks: history, fits, the carrier trim decision
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
#include "clock_drift.h"

/* Between two points the PC's UTC may move against the raw clock by the PC crystal's error plus
 * NTP's slew (the kernel slews at most 500 ppm): more is a step of the clock. The device against
 * the raw clock: its error plus the PC crystal's, some 100 ppm at most; more is a jump of the
 * device time. Each plus 1 ms of read jitter. */
#define UTC_STEP_PPM	600e-6
#define DEVICE_JUMP_PPM	200e-6
#define READ_JITTER_S	1e-3

void clock_drift_init(clock_drift_t *cd)
{
	memset(cd, 0, sizeof(*cd));
}

static int idx(const clock_drift_t *cd, int back)	/* back 0: the newest */
{
	return (cd->head - 1 - back + 2 * CLOCK_DRIFT_POINTS) % CLOCK_DRIFT_POINTS;
}

int clock_drift_add(clock_drift_t *cd, double raw, double device, double utc, int ntp)
{
	int rc = 0, last;

	if (cd->n) {
		double dt;

		last = idx(cd, 0);
		dt = raw - cd->t[last];
		if (dt <= 0.0)
			return 0;
		if (fabs((device - raw) - cd->d_raw[last]) > DEVICE_JUMP_PPM * dt + READ_JITTER_S) {
			cd->n = cd->head = cd->utc_n = 0;
			cd->device_restarts++;
			rc = 2;
		} else if (cd->utc_n && fabs((device - utc) - cd->d_utc[last] - ((device - raw) - cd->d_raw[last])) > UTC_STEP_PPM * dt + READ_JITTER_S) {
			cd->utc_n = 0;
			cd->utc_restarts++;
			rc = 1;
		}
	}
	if (!ntp && cd->utc_n) {
		cd->utc_n = 0;
		cd->utc_restarts++;
		rc = rc ? rc : 1;
	}
	cd->t[cd->head] = raw;
	cd->d_raw[cd->head] = device - raw;
	cd->d_utc[cd->head] = device - utc;
	cd->head = (cd->head + 1) % CLOCK_DRIFT_POINTS;
	if (cd->n < CLOCK_DRIFT_POINTS)
		cd->n++;
	if (ntp && cd->utc_n < cd->n)
		cd->utc_n++;
	return rc;
}

/* least squares through the newest n points */
static int fit(const clock_drift_t *cd, const double *d, int n, clock_fit_t *f)
{
	double t0, d0, mx = 0.0, my = 0.0, sxx = 0.0, sxy = 0.0, res = 0.0, x, y;
	int i, k;

	memset(f, 0, sizeof(*f));
	if (n < 3)
		return -1;
	k = idx(cd, n - 1);
	t0 = cd->t[k];
	d0 = d[k];
	for (i = 0; i < n; i++) {
		k = idx(cd, i);
		mx += cd->t[k] - t0;
		my += d[k] - d0;
	}
	mx /= n;
	my /= n;
	for (i = 0; i < n; i++) {
		k = idx(cd, i);
		x = cd->t[k] - t0 - mx;
		y = d[k] - d0 - my;
		sxx += x * x;
		sxy += x * y;
	}
	if (sxx <= 0.0)
		return -1;
	f->slope = sxy / sxx;
	for (i = 0; i < n; i++) {
		k = idx(cd, i);
		x = cd->t[k] - t0 - mx;
		y = d[k] - d0 - my;
		res += (y - f->slope * x) * (y - f->slope * x);
	}
	f->n = n;
	f->span = cd->t[idx(cd, 0)] - t0;
	f->scatter = sqrt(res / (n - 2));
	f->se = f->scatter / sqrt(sxx);
	return 0;
}

int clock_drift_fit_raw(const clock_drift_t *cd, clock_fit_t *f)
{
	return fit(cd, cd->d_raw, cd->n, f);
}

int clock_drift_fit_utc(const clock_drift_t *cd, clock_fit_t *f)
{
	return fit(cd, cd->d_utc, cd->utc_n, f);
}

double clock_drift_utc_walk(const clock_drift_t *cd)
{
	if (cd->utc_n < 2)
		return 0.0;
	return cd->d_utc[idx(cd, 0)] - cd->d_utc[idx(cd, cd->utc_n - 1)];
}

double clock_fit_utc_bound(const clock_fit_t *f, double eps)
{
	if (f->n < 3 || f->span <= 0.0)
		return INFINITY;
	return (3.0 * f->se + 2.0 * eps / f->span) * 1e6;
}

void clock_trim_init(clock_trim_t *tr)
{
	memset(tr, 0, sizeof(*tr));
	tr->min_span = 1800.0;
	tr->max_step = 0.1;
	tr->min_interval = 600.0;
}

double clock_trim_decide(clock_trim_t *tr, const clock_fit_t *utc, double eps, double static_ppm, double now)
{
	double r, bound, step;
	int sign;

	if (utc->n < 3 || utc->span < tr->min_span)
		return 0.0;
	r = utc->slope * 1e6 + static_ppm + tr->applied;
	bound = clock_fit_utc_bound(utc, eps);
	tr->residual = r;
	tr->bound = bound;
	if (tr->steps && now - tr->last_t < tr->min_interval)
		return 0.0;
	step = -r;
	if (step > tr->max_step)
		step = tr->max_step;
	if (step < -tr->max_step)
		step = -tr->max_step;
	/* certain to help: the true error is more than half a step on r's side */
	if (fabs(r) - bound <= fabs(step) / 2.0)
		return 0.0;
	sign = (step > 0.0) ? 1 : -1;
	if (tr->last_sign && sign != tr->last_sign) {
		if (fabs(r) - 2.0 * bound <= fabs(step))
			return 0.0;
		tr->reversals++;
	}
	tr->applied += step;
	tr->last_t = now;
	tr->last_sign = sign;
	tr->steps++;
	return step;
}
