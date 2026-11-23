/* Halfband 2x decimation / interpolation filter
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "halfband.h"

/* zeroth order modified Bessel function of the first kind */
static double bessel_i0(double x)
{
	double sum = 1.0, term = 1.0;
	int k;

	for (k = 1; k < 100; k++) {
		term *= (x / (2.0 * k)) * (x / (2.0 * k));
		sum += term;
		if (term < sum * 1e-17)
			break;
	}
	return sum;
}

/* H(f) of the halfband (f in cycles per sample of the higher rate):
 * 0.5 + 2 * sum c_j cos(2 pi f d_j), d_j = 2 (n - j) - 1 */
static double response(const double *c, int n, double f)
{
	double h = 0.5;
	int j;

	for (j = 0; j < n; j++)
		h += 2.0 * c[j] * cos(2.0 * M_PI * f * (2 * (n - j) - 1));
	return h;
}

/* Kaiser windowed ideal halfband (cutoff 0.25), n taps per side; the odd taps
 * are scaled to sum to 0.5 (DC gain 1, centre 0.5: halfband property) */
static void kaiser_halfband(double *c, int n, double beta)
{
	double sum = 0.0, len = 2.0 * n;
	int j;

	for (j = 0; j < n; j++) {
		int d = 2 * (n - j) - 1;
		double r = d / len, w = bessel_i0(beta * sqrt(1.0 - r * r)) / bessel_i0(beta);

		c[j] = sin(M_PI * d / 2.0) / (M_PI * d) * w;
		sum += c[j];
	}
	for (j = 0; j < n; j++)
		c[j] *= 0.25 / sum;
}

/* worst stopband attenuation (dB, positive) and passband deviation (dB) */
static void measure(const double *c, int n, double pass, double *stop_db, double *ripple_db)
{
	double worst_stop = 0.0, worst_pass = 0.0, f;
	int k, steps = 400;

	for (k = 0; k <= steps; k++) {
		f = pass * k / steps;
		double dev = fabs(20.0 * log10(fabs(response(c, n, f))));

		if (dev > worst_pass)
			worst_pass = dev;
		f = 0.5 - pass + pass * k / steps;
		double h = fabs(response(c, n, f));

		if (h > worst_stop)
			worst_stop = h;
	}
	*stop_db = -20.0 * log10(worst_stop + 1e-300);
	*ripple_db = worst_pass;
}

/* designs made so far: every channel with the same rates needs the same
 * stages (init runs in one thread) */
#define DESIGN_CACHE	32
static struct {
	double pass, atten;
	int n;
	double coeffs[HALFBAND_MAX_COEFFS], stop_db, ripple_db;
} cache[DESIGN_CACHE];
static int cache_len;

int halfband_design(halfband_t *hb, double pass, double atten_db)
{
	double c[HALFBAND_MAX_COEFFS], stop, ripple, beta0;
	int n, n0, i;

	memset(hb, 0, sizeof(*hb));
	if (pass <= 0.0 || pass >= 0.25)
		return -EINVAL;
	for (i = 0; i < cache_len; i++) {
		if (cache[i].pass == pass && cache[i].atten == atten_db) {
			hb->num_coeffs = cache[i].n;
			memcpy(hb->coeffs, cache[i].coeffs, sizeof(hb->coeffs));
			hb->stop_db = cache[i].stop_db;
			hb->ripple_db = cache[i].ripple_db;
			goto found;
		}
	}
	/* shortest filter that reaches the attenuation (best Kaiser beta for each
	 * length), passband within 0.001 dB. The search starts a little below
	 * the Kaiser estimate of the length (taps 4n - 1, transition 0.5 - 2
	 * pass); short filters have their best beta far from the Kaiser formula,
	 * so all of 0 .. 16 is tried. */
	n0 = (int)floor(((atten_db - 7.95) / (2.285 * 2.0 * M_PI * (0.5 - 2.0 * pass)) + 1.0) / 4.0) - 2;
	if (n0 < 1)
		n0 = 1;
	beta0 = 0.1102 * (atten_db - 8.7);
	for (n = n0; n <= HALFBAND_MAX_COEFFS; n++) {
		double best_stop = -1.0, best_ripple = 0.0, best_beta = 0.0, beta;

		for (beta = 0.0; beta <= fmax(16.0, beta0 + 3.0); beta += 0.1) {
			kaiser_halfband(c, n, beta);
			measure(c, n, pass, &stop, &ripple);
			if (ripple <= 0.001 && stop > best_stop) {
				best_stop = stop;
				best_ripple = ripple;
				best_beta = beta;
			}
		}
		if (best_stop >= atten_db) {
			kaiser_halfband(hb->coeffs, n, best_beta);
			hb->num_coeffs = n;
			hb->stop_db = best_stop;
			hb->ripple_db = best_ripple;
			break;
		}
	}
	if (!hb->num_coeffs)
		return -EINVAL;
	if (cache_len < DESIGN_CACHE) {
		cache[cache_len].pass = pass;
		cache[cache_len].atten = atten_db;
		cache[cache_len].n = hb->num_coeffs;
		memcpy(cache[cache_len].coeffs, hb->coeffs, sizeof(hb->coeffs));
		cache[cache_len].stop_db = hb->stop_db;
		cache[cache_len].ripple_db = hb->ripple_db;
		cache_len++;
	}
found:
	hb->order = 4 * hb->num_coeffs;
	hb->pass = pass;

	/* the window is stored twice (at ptr and ptr + length), so a read of
	 * length samples from ptr never wraps */
	hb->buf_i = calloc(2 * hb->order, sizeof(*hb->buf_i));
	hb->buf_q = calloc(2 * hb->order, sizeof(*hb->buf_q));
	if (!hb->buf_i || !hb->buf_q) {
		halfband_exit(hb);
		return -ENOMEM;
	}
	return 0;
}

void halfband_exit(halfband_t *hb)
{
	free(hb->buf_i);
	free(hb->buf_q);
	memset(hb, 0, sizeof(*hb));
}

/* Window of order samples, bi[0] oldest. The non-zero taps sit at odd
 * distances 1, 3, ... from the centre bi[order/2]; coeffs[0] (outermost) at
 * bi[1] and bi[order-1]; centre tap 0.5. */
int halfband_decimate_block(halfband_t *hb, sample_t *in_i, sample_t *in_q, int n)
{
	const int nc = hb->num_coeffs, order = hb->order, half = order / 2;
	const double *c = hb->coeffs;
	double *buf_i = hb->buf_i, *buf_q = hb->buf_q;
	int ptr = hb->ptr, phase = hb->phase, s, j, out = 0;

	for (s = 0; s < n; s++) {
		buf_i[ptr] = buf_i[ptr + order] = in_i[s];
		buf_q[ptr] = buf_q[ptr + order] = in_q[s];
		if (++ptr == order)
			ptr = 0;
		if (!phase) {
			phase = 1;
			continue;
		}
		phase = 0;

		const double *bi = buf_i + ptr, *bq = buf_q + ptr;
		double acc_i = bi[half] * 0.5, acc_q = bq[half] * 0.5;

		for (j = 0; j < nc; j++) {
			int l = 1 + 2 * j, r = order - 1 - 2 * j;

			acc_i += (bi[l] + bi[r]) * c[j];
			acc_q += (bq[l] + bq[r]) * c[j];
		}
		in_i[out] = acc_i;
		in_q[out] = acc_q;
		out++;
	}
	hb->ptr = ptr;
	hb->phase = phase;
	return out;
}

/* Zero-stuffing the input and filtering with the halfband h (gain 2, centre
 * at a delay of L-1 output samples, L = order/2) gives two outputs per input
 * x[m], in time order:
 *   out[0] = 2 * sum c_j * (x[m - (L-1-d_j)/2] + x[m - (L-1+d_j)/2])
 *   out[1] = x[m - (L-2)/2]                     (centre tap 0.5 * 2)
 * with d_j = 2*(n-j)-1. The window of L input samples is stored twice. */
void halfband_interpolate_block(halfband_t *hb, const sample_t *in_i, const sample_t *in_q, int n, sample_t *out_i, sample_t *out_q)
{
	const int L = hb->order / 2, nc = hb->num_coeffs;
	const double *c = hb->coeffs;
	double *buf_i = hb->buf_i, *buf_q = hb->buf_q;
	const int centre = L - 1 - (L - 2) / 2;
	int ptr = hb->ptr, s, j, ia[HALFBAND_MAX_COEFFS], ib[HALFBAND_MAX_COEFFS];

	/* window positions of the tap pairs: x[m - k] = w[L-1-k] */
	for (j = 0; j < nc; j++) {
		int d = 2 * (nc - j) - 1;

		ia[j] = L - 1 - (L - 1 - d) / 2;
		ib[j] = L - 1 - (L - 1 + d) / 2;
	}
	for (s = 0; s < n; s++) {
		buf_i[ptr] = buf_i[ptr + L] = in_i[s];
		buf_q[ptr] = buf_q[ptr + L] = in_q[s];
		if (++ptr == L)
			ptr = 0;
		const double *wi = buf_i + ptr, *wq = buf_q + ptr;
		double acc_i = 0.0, acc_q = 0.0;

		for (j = 0; j < nc; j++) {
			acc_i += (wi[ia[j]] + wi[ib[j]]) * c[j];
			acc_q += (wq[ia[j]] + wq[ib[j]]) * c[j];
		}
		out_i[2 * s] = 2.0 * acc_i;
		out_q[2 * s] = 2.0 * acc_q;
		out_i[2 * s + 1] = wi[centre];
		out_q[2 * s + 1] = wq[centre];
	}
	hb->ptr = ptr;
}
