/* Rational polyphase resampler P/Q (complex)
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Upsampling by P (zero stuffing), filtering with h at P * fin, keeping every
 * Q-th sample: with t = n * P + phase the output at t is
 *   v[t] = sum_j h[j * P + phase] * x[n - j]
 * so only the taps of one phase are computed per output.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "resamp.h"

static int gcd(int a, int b)
{
	while (b) {
		int t = a % b;

		a = b;
		b = t;
	}
	return a;
}

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

int resamp_init(resamp_t *r, int in_rate, int out_rate)
{
	double low = (in_rate < out_rate) ? in_rate : out_rate;

	return resamp_init_band(r, in_rate, out_rate, RESAMP_PASS * low, RESAMP_STOP * low, RESAMP_ATTEN_DB);
}

int resamp_init_band(resamp_t *r, int in_rate, int out_rate, double pass_hz, double stop_hz, double atten)
{
	double beta, fc, dw, sum = 0.0, hi, *h;
	int g, n, k, taps, ph, j;

	memset(r, 0, sizeof(*r));
	if (in_rate <= 0 || out_rate <= 0 || in_rate == out_rate || pass_hz <= 0.0 || stop_hz <= pass_hz)
		return -EINVAL;
	g = gcd(in_rate, out_rate);
	r->p = out_rate / g;
	r->q = in_rate / g;

	/* design at the upsampled rate hi = P * fin */
	hi = (double)in_rate * r->p;
	fc = (pass_hz + stop_hz) / 2.0 / hi;
	dw = 2.0 * M_PI * (stop_hz - pass_hz) / hi;
	beta = 0.1102 * (atten - 8.7);
	n = (int)ceil((atten - 7.95) / (2.285 * dw)) + 1;
	if (!(n & 1))
		n++;
	/* a multiple of P: equal phases; the extra taps are zero */
	r->len = (n + r->p - 1) / r->p;
	taps = r->len * r->p;

	h = calloc(taps, sizeof(*h));
	r->h = calloc(taps, sizeof(*r->h));
	if (!h || !r->h) {
		free(h);
		free(r->h);
		r->h = NULL;
		return -ENOMEM;
	}
	for (k = 0; k < n; k++) {
		double t = k - (n - 1) / 2.0;
		double sinc = (t == 0.0) ? 2.0 * fc : sin(2.0 * M_PI * fc * t) / (M_PI * t);
		double x = (n > 1) ? 2.0 * k / (n - 1) - 1.0 : 0.0;
		double w = bessel_i0(beta * sqrt(1.0 - x * x)) / bessel_i0(beta);

		h[k] = sinc * w;
		sum += h[k];
	}
	/* unity gain at 0 Hz: zero stuffing by P loses P */
	for (k = 0; k < n; k++)
		h[k] *= r->p / sum;
	/* one contiguous row per phase, in the order of the window (oldest
	 * first): row[ph][len-1-j] = h[j P + ph] */
	for (ph = 0; ph < r->p; ph++)
		for (j = 0; j < r->len; j++)
			r->h[ph * r->len + (r->len - 1 - j)] = h[j * r->p + ph];
	free(h);

	r->buf_i = calloc(2 * r->len, sizeof(*r->buf_i));
	r->buf_q = calloc(2 * r->len, sizeof(*r->buf_q));
	if (!r->buf_i || !r->buf_q) {
		resamp_exit(r);
		return -ENOMEM;
	}
	return 0;
}

void resamp_exit(resamp_t *r)
{
	free(r->h);
	free(r->buf_i);
	free(r->buf_q);
	memset(r, 0, sizeof(*r));
}

int resamp_max_out(const resamp_t *r)
{
	return (r->p + r->q - 1) / r->q;
}

int resamp_process(resamp_t *r, sample_t in_i, sample_t in_q, sample_t *out_i, sample_t *out_q)
{
	int len = r->len, p = r->p, count = 0, j;
	const double *bi, *bq;

	r->buf_i[r->ptr] = r->buf_i[r->ptr + len] = in_i;
	r->buf_q[r->ptr] = r->buf_q[r->ptr + len] = in_q;
	if (++r->ptr == len)
		r->ptr = 0;
	/* window from ptr: oldest first, newest x[n] at len - 1 */
	bi = r->buf_i + r->ptr;
	bq = r->buf_q + r->ptr;
	while (r->phase < p) {
		const double *h = r->h + r->phase * len;
		double acc_i = 0.0, acc_q = 0.0;

		for (j = 0; j < len; j++) {
			acc_i += h[j] * bi[j];
			acc_q += h[j] * bq[j];
		}
		out_i[count] = acc_i;
		out_q[count] = acc_q;
		count++;
		r->phase += r->q;
	}
	r->phase -= p;
	return count;
}
