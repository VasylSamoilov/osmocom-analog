/* cut-off filter (biquad) based on Nigel Redmon (www.earlevel.com)
 *
 * (C) 2016 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../libsample/sample.h"
#include "iir_filter.h"

//#define DEBUG_NAN

#define PI M_PI

void iir_lowpass_init(iir_filter_t *filter, double frequency, int samplerate, int iterations)
{
	double Fc, Q, K, norm;

	if (iterations > 64) {
		fprintf(stderr, "%s failed: too many iterations, please fix!\n", __func__);
		abort();
	}

	memset(filter, 0, sizeof(*filter));
	filter->iter = iterations;
	Q = pow(sqrt(0.5), 1.0 / (double)iterations); /* 0.7071 @ 1 iteration */
	Fc = frequency / (double)samplerate;
	K = tan(PI * Fc);
	norm = 1 / (1 + K / Q + K * K);
	filter->a0 = K * K * norm;
	filter->a1 = 2 * filter->a0;
	filter->a2 = filter->a0;
	filter->b1 = 2 * (K * K - 1) * norm;
	filter->b2 = (1 - K / Q + K * K) * norm;
#ifdef DEBUG_NAN
	printf("%p\n", filter);
#endif
}

void iir_highpass_init(iir_filter_t *filter, double frequency, int samplerate, int iterations)
{
	double Fc, Q, K, norm;

	memset(filter, 0, sizeof(*filter));
	filter->iter = iterations;
	Q = pow(sqrt(0.5), 1.0 / (double)iterations); /* 0.7071 @ 1 iteration */
	Fc = frequency / (double)samplerate;
	K = tan(PI * Fc);
	norm = 1 / (1 + K / Q + K * K);
	filter->a0 = 1 * norm;
	filter->a1 = -2 * filter->a0;
	filter->a2 = filter->a0;
	filter->b1 = 2 * (K * K - 1) * norm;
	filter->b2 = (1 - K / Q + K * K) * norm;
}

/* Butterworth cascades of 'order' (even, 2..16): one biquad per pole pair, each
 * with its own Q = 1 / (2 cos((2k-1) pi / (2 order))). The passband is flat
 * (-3 dB at 'frequency'). iir_lowpass_init() with iterations > 1 instead
 * repeats one section with Q = 0.7071^(1/iterations), which peaks (8
 * iterations: +7.7 dB below the cut-off). The sections are 'filter[order/2]'. */
static int butterworth_sections(int order)
{
	if (order < 2 || order > 16 || (order & 1)) {
		fprintf(stderr, "%s failed: order %d must be even, 2..16, please fix!\n", __func__, order);
		abort();
	}
	return order / 2;
}

static double butterworth_q(int order, int k)	/* k = 1 .. order/2 */
{
	return 1.0 / (2.0 * cos((2.0 * k - 1.0) * PI / (2.0 * order)));
}

void iir_butterworth_lowpass_init(iir_filter_t *filter, double frequency, int samplerate, int order)
{
	int n = butterworth_sections(order), k;

	for (k = 0; k < n; k++) {
		double Q = butterworth_q(order, k + 1), K = tan(PI * frequency / (double)samplerate);
		double norm = 1 / (1 + K / Q + K * K);

		memset(&filter[k], 0, sizeof(filter[k]));
		filter[k].iter = 1;
		filter[k].a0 = K * K * norm;
		filter[k].a1 = 2 * filter[k].a0;
		filter[k].a2 = filter[k].a0;
		filter[k].b1 = 2 * (K * K - 1) * norm;
		filter[k].b2 = (1 - K / Q + K * K) * norm;
	}
}

void iir_butterworth_highpass_init(iir_filter_t *filter, double frequency, int samplerate, int order)
{
	int n = butterworth_sections(order), k;

	for (k = 0; k < n; k++) {
		double Q = butterworth_q(order, k + 1), K = tan(PI * frequency / (double)samplerate);
		double norm = 1 / (1 + K / Q + K * K);

		memset(&filter[k], 0, sizeof(filter[k]));
		filter[k].iter = 1;
		filter[k].a0 = 1 * norm;
		filter[k].a1 = -2 * filter[k].a0;
		filter[k].a2 = filter[k].a0;
		filter[k].b1 = 2 * (K * K - 1) * norm;
		filter[k].b2 = (1 - K / Q + K * K) * norm;
	}
}

void iir_butterworth_process(iir_filter_t *filter, int order, sample_t *samples, int length)
{
	int n = butterworth_sections(order), k;

	for (k = 0; k < n; k++)
		iir_process(&filter[k], samples, length);
}

void iir_bandpass_init(iir_filter_t *filter, double frequency, int samplerate, int iterations)
{
	double Fc, Q, K, norm;

	memset(filter, 0, sizeof(*filter));
	filter->iter = iterations;
	Q = pow(sqrt(0.5), 1.0 / (double)iterations); /* 0.7071 @ 1 iteration */
	Fc = frequency / (double)samplerate;
	K = tan(PI * Fc);
	norm = 1 / (1 + K / Q + K * K);
	filter->a0 = K / Q * norm;
	filter->a1 = 0;
	filter->a2 = -filter->a0;
	filter->b1 = 2 * (K * K - 1) * norm;
	filter->b2 = (1 - K / Q + K * K) * norm;
}

void iir_notch_init(iir_filter_t *filter, double frequency, int samplerate, int iterations, double Q)
{
	double Fc, K, norm;

	memset(filter, 0, sizeof(*filter));
	filter->iter = iterations;
	Fc = frequency / (double)samplerate;
	K = tan(PI * Fc);
	norm = 1 / (1 + K / Q + K * K);
	filter->a0 = (1 + K * K) * norm;
	filter->a1 = 2 * (K * K - 1) * norm;
	filter->a2 = filter->a0;
	filter->b1 = filter->a1;
	filter->b2 = (1 - K / Q + K * K) * norm;
}

/* Filter states of a filter that gets no input decay exponentially. When they
 * become subnormal, arithmetic on them is about 100 times slower. So states that
 * are below this limit (about -600 dB of full scale) are set to zero, which
 * does not change the output. This is done every IIR_FLUSH_CHUNK samples, so
 * states never get to the subnormal range, even with the longest block. */
#define IIR_FLUSH_LIMIT	1e-30
#define IIR_FLUSH_CHUNK	256

static inline void iir_flush_states(double *z1, double *z2, int iterations)
{
	int j;

	for (j = 0; j < iterations; j++) {
		if (fabs(z1[j]) < IIR_FLUSH_LIMIT)
			z1[j] = 0.0;
		if (fabs(z2[j]) < IIR_FLUSH_LIMIT)
			z2[j] = 0.0;
	}
}

void iir_process(iir_filter_t *filter, sample_t *samples, int length)
{
	double a0, a1, a2, b1, b2;
	double *z1, *z2;
	double in, out;
	int iterations = filter->iter;
	int i, j, end;

	/* get states */
	a0 = filter->a0;
	a1 = filter->a1;
	a2 = filter->a2;
	b1 = filter->b1;
	b2 = filter->b2;

	/* these are state pointers, so no need to write back */
	z1 = filter->z1;
	z2 = filter->z2;

	/* process filter */
	for (i = 0; i < length; ) {
		end = i + IIR_FLUSH_CHUNK;
		if (end > length)
			end = length;
		for (; i < end; i++) {
			in = *samples;
			for (j = 0; j < iterations; j++) {
				out = in * a0 + z1[j];
				z1[j] = in * a1 + z2[j] - b1 * out;
				z2[j] = in * a2 - b2 * out;
				in = out;
			}
			*samples++ = in;
		}
		iir_flush_states(z1, z2, iterations);
	}
}

#ifdef DEBUG_NAN
#pragma GCC push_options
//#pragma GCC optimize ("O0")
#endif

void iir_process_baseband(iir_filter_t *filter, float *baseband, int length)
{
	double a0, a1, a2, b1, b2;
	double *z1, *z2;
	double in, out;
	int iterations = filter->iter;
	int i, j;

	/* get states */
	a0 = filter->a0;
	a1 = filter->a1;
	a2 = filter->a2;
	b1 = filter->b1;
	b2 = filter->b2;

	/* these are state pointers, so no need to write back */
	z1 = filter->z1;
	z2 = filter->z2;

	/* process filter */
	for (i = 0; i < length; i++) {
		/* add a small value, otherwise this loop will perform really bad on my 'nuedel' machine!!! */
		in = *baseband + 0.000000001;
		for (j = 0; j < iterations; j++) {
			out = in * a0 + z1[j];
#ifdef DEBUG_NAN
			if (!(out > -100 && out < 100)) {
				printf("%p\n", filter);
				printf("1. i=%d j=%d z=%.5f in=%.5f a0=%.5f out=%.5f\n", i, j, z1[j], in, a0, out);
				abort();
			}
#endif
			z1[j] = in * a1 + z2[j] - b1 * out;
#ifdef DEBUG_NAN
			if (!(z1[j] > -100 && z1[j] < 100)) {
				printf("%p\n", filter);
				printf("2. i=%d j=%d z1=%.5f z2=%.5f in=%.5f a1=%.5f out=%.5f b1=%.5f\n", i, j, z1[j], z2[j], in, a1, out, b1);
				abort();
			}
#endif
			z2[j] = in * a2 - b2 * out;
#ifdef DEBUG_NAN
			if (!(z2[j] > -100 && z2[j] < 100)) {
				printf("%p\n", filter);
				printf("%.5f\n", (in * a2) - (b2 * out));
				printf("3. i=%d j=%d z2=%.5f in=%.5f a2=%.5f b2=%.5f out=%.5f\n", i, j, z2[j], in, a2, b2, out);
				abort();
			}
#endif
			in = out;
		}
		*baseband = in;
		baseband += 2;
	}
}

#ifdef DEBUG_NAN
#pragma GCC pop_options
#endif
