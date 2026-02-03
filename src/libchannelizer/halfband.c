/* Halfband decimation filter implementation
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Efficient 2x decimation using halfband FIR filter.
 * Reference: SDRangel implementation was studied to understand the algorithm.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "halfband.h"
#include "halfband_coeffs.h"

/* Initialize halfband filter */
int halfband_init(halfband_t *hb, int order, halfband_mode_t mode, int fast_math)
{
	memset(hb, 0, sizeof(*hb));

	/* Validate and set order */
	switch (order) {
	case HALFBAND_ORDER_32:
		hb->num_coeffs = 8;
		hb->coeffs_d = halfband_coeffs_32_d;
		hb->coeffs_f = halfband_coeffs_32_f;
		hb->shift = HALFBAND_SHIFT_32;
		break;
	case HALFBAND_ORDER_48:
		hb->num_coeffs = 12;
		hb->coeffs_d = halfband_coeffs_48_d;
		hb->coeffs_f = halfband_coeffs_48_f;
		hb->shift = HALFBAND_SHIFT_48;
		break;
	case HALFBAND_ORDER_64:
		hb->num_coeffs = 16;
		hb->coeffs_d = halfband_coeffs_64_d;
		hb->coeffs_f = halfband_coeffs_64_f;
		hb->shift = HALFBAND_SHIFT_64;
		break;
	default:
		fprintf(stderr, "Halfband: unsupported order %d\n", order);
		return -EINVAL;
	}

	hb->order = order;
	hb->mode = mode;
	hb->fast_math = fast_math;
	hb->state = 0;
	hb->ptr = 0;

	/* Allocate ring buffers - size is order/2 for double-buffer technique */
	int buf_size = order;

	if (fast_math) {
		hb->buf_i_f = calloc(buf_size, sizeof(*hb->buf_i_f));
		hb->buf_q_f = calloc(buf_size, sizeof(*hb->buf_q_f));
		if (!hb->buf_i_f || !hb->buf_q_f) {
			fprintf(stderr, "Halfband: no memory for buffers\n");
			halfband_exit(hb);
			return -ENOMEM;
		}
	} else {
		hb->buf_i_d = calloc(buf_size, sizeof(*hb->buf_i_d));
		hb->buf_q_d = calloc(buf_size, sizeof(*hb->buf_q_d));
		if (!hb->buf_i_d || !hb->buf_q_d) {
			fprintf(stderr, "Halfband: no memory for buffers\n");
			halfband_exit(hb);
			return -ENOMEM;
		}
	}

	return 0;
}

/* Free halfband filter resources */
void halfband_exit(halfband_t *hb)
{
	free(hb->buf_i_d);
	free(hb->buf_q_d);
	free(hb->buf_i_f);
	free(hb->buf_q_f);
	memset(hb, 0, sizeof(*hb));
}

/*
 * Double precision FIR convolution
 * Halfband property: only compute non-zero taps + center tap
 */
static void do_fir_double(halfband_t *hb, double *out_i, double *out_q)
{
	double acc_i = 0.0, acc_q = 0.0;
	int n = hb->num_coeffs;
	int half = hb->order / 2;
	int j, idx;

	/* Symmetric filter - compute both sides together */
	for (j = 0; j < n; j++) {
		/* Left side of filter */
		idx = (hb->ptr + j * 2) % hb->order;
		acc_i += hb->buf_i_d[idx] * hb->coeffs_d[j];
		acc_q += hb->buf_q_d[idx] * hb->coeffs_d[j];

		/* Right side (symmetric) */
		idx = (hb->ptr + hb->order - 1 - j * 2) % hb->order;
		acc_i += hb->buf_i_d[idx] * hb->coeffs_d[j];
		acc_q += hb->buf_q_d[idx] * hb->coeffs_d[j];
	}

	/* Center tap (always 0.5 for halfband) */
	idx = (hb->ptr + half) % hb->order;
	acc_i += hb->buf_i_d[idx] * 0.5;
	acc_q += hb->buf_q_d[idx] * 0.5;

	*out_i = acc_i;
	*out_q = acc_q;
}

/*
 * Fixed-point FIR convolution
 */
static void do_fir_fixed(halfband_t *hb, int32_t *out_i, int32_t *out_q)
{
	int64_t acc_i = 0, acc_q = 0;
	int n = hb->num_coeffs;
	int half = hb->order / 2;
	int j, idx;

	/* Symmetric filter - compute both sides together */
	for (j = 0; j < n; j++) {
		/* Left side */
		idx = (hb->ptr + j * 2) % hb->order;
		acc_i += (int64_t)hb->buf_i_f[idx] * hb->coeffs_f[j];
		acc_q += (int64_t)hb->buf_q_f[idx] * hb->coeffs_f[j];

		/* Right side */
		idx = (hb->ptr + hb->order - 1 - j * 2) % hb->order;
		acc_i += (int64_t)hb->buf_i_f[idx] * hb->coeffs_f[j];
		acc_q += (int64_t)hb->buf_q_f[idx] * hb->coeffs_f[j];
	}

	/* Center tap (0.5 = 2048 in Q12) */
	idx = (hb->ptr + half) % hb->order;
	acc_i += (int64_t)hb->buf_i_f[idx] << (hb->shift - 1);
	acc_q += (int64_t)hb->buf_q_f[idx] << (hb->shift - 1);

	*out_i = (int32_t)(acc_i >> hb->shift);
	*out_q = (int32_t)(acc_q >> hb->shift);
}

/*
 * Process one IQ sample - CENTER mode (no frequency shift)
 * Returns 1 every other sample when decimated output is ready
 */
static int process_center(halfband_t *hb, sample_t *i, sample_t *q)
{
	if (hb->fast_math) {
		/* Store sample */
		hb->buf_i_f[hb->ptr] = (int32_t)(*i * 4096.0);
		hb->buf_q_f[hb->ptr] = (int32_t)(*q * 4096.0);
	} else {
		hb->buf_i_d[hb->ptr] = *i;
		hb->buf_q_d[hb->ptr] = *q;
	}

	hb->ptr = (hb->ptr + 1) % hb->order;

	if (hb->state == 0) {
		hb->state = 1;
		return 0;  /* No output yet */
	}

	/* Compute FIR and output */
	hb->state = 0;

	if (hb->fast_math) {
		int32_t out_i, out_q;
		do_fir_fixed(hb, &out_i, &out_q);
		*i = (sample_t)out_i / 4096.0;
		*q = (sample_t)out_q / 4096.0;
	} else {
		double out_i, out_q;
		do_fir_double(hb, &out_i, &out_q);
		*i = out_i;
		*q = out_q;
	}

	return 1;  /* Output ready */
}

/*
 * Process one IQ sample - LOWER mode (extract lower half, shift up)
 * Uses 4-state machine to apply frequency shift by Fs/4
 */
static int process_lower(halfband_t *hb, sample_t *i, sample_t *q)
{
	sample_t ti = *i, tq = *q;

	/* Rotate by -Fs/4 (multiply by e^(-j*pi/2*n)) */
	switch (hb->state) {
	case 0:  /* x1 */
		break;
	case 1:  /* x(-j) = swap and negate I */
		*i = *q;
		*q = -ti;
		break;
	case 2:  /* x(-1) */
		*i = -ti;
		*q = -tq;
		break;
	case 3:  /* x(j) = swap and negate Q */
		*i = -tq;
		*q = ti;
		break;
	}

	/* Store rotated sample */
	if (hb->fast_math) {
		hb->buf_i_f[hb->ptr] = (int32_t)(*i * 4096.0);
		hb->buf_q_f[hb->ptr] = (int32_t)(*q * 4096.0);
	} else {
		hb->buf_i_d[hb->ptr] = *i;
		hb->buf_q_d[hb->ptr] = *q;
	}

	hb->ptr = (hb->ptr + 1) % hb->order;

	/* Advance state, output every other sample */
	int output = (hb->state & 1);
	hb->state = (hb->state + 1) & 3;

	if (!output)
		return 0;

	/* Compute FIR */
	if (hb->fast_math) {
		int32_t out_i, out_q;
		do_fir_fixed(hb, &out_i, &out_q);
		*i = (sample_t)out_i / 4096.0;
		*q = (sample_t)out_q / 4096.0;
	} else {
		double out_i, out_q;
		do_fir_double(hb, &out_i, &out_q);
		*i = out_i;
		*q = out_q;
	}

	return 1;
}

/*
 * Process one IQ sample - UPPER mode (extract upper half, shift down)
 * Uses 4-state machine to apply frequency shift by -Fs/4
 */
static int process_upper(halfband_t *hb, sample_t *i, sample_t *q)
{
	sample_t ti = *i, tq = *q;

	/* Rotate by +Fs/4 (multiply by e^(j*pi/2*n)) */
	switch (hb->state) {
	case 0:  /* x1 */
		break;
	case 1:  /* x(j) = swap and negate Q */
		*i = -tq;
		*q = ti;
		break;
	case 2:  /* x(-1) */
		*i = -ti;
		*q = -tq;
		break;
	case 3:  /* x(-j) = swap and negate I */
		*i = tq;
		*q = -ti;
		break;
	}

	/* Store rotated sample */
	if (hb->fast_math) {
		hb->buf_i_f[hb->ptr] = (int32_t)(*i * 4096.0);
		hb->buf_q_f[hb->ptr] = (int32_t)(*q * 4096.0);
	} else {
		hb->buf_i_d[hb->ptr] = *i;
		hb->buf_q_d[hb->ptr] = *q;
	}

	hb->ptr = (hb->ptr + 1) % hb->order;

	/* Advance state, output every other sample */
	int output = (hb->state & 1);
	hb->state = (hb->state + 1) & 3;

	if (!output)
		return 0;

	/* Compute FIR */
	if (hb->fast_math) {
		int32_t out_i, out_q;
		do_fir_fixed(hb, &out_i, &out_q);
		*i = (sample_t)out_i / 4096.0;
		*q = (sample_t)out_q / 4096.0;
	} else {
		double out_i, out_q;
		do_fir_double(hb, &out_i, &out_q);
		*i = out_i;
		*q = out_q;
	}

	return 1;
}

/* Process one IQ sample pair */
int halfband_process(halfband_t *hb, sample_t *i, sample_t *q)
{
	switch (hb->mode) {
	case HALFBAND_CENTER:
		return process_center(hb, i, q);
	case HALFBAND_LOWER:
		return process_lower(hb, i, q);
	case HALFBAND_UPPER:
		return process_upper(hb, i, q);
	default:
		return 0;
	}
}

/*
 * ==================== INTERPOLATION (2x upsample) ====================
 *
 * Interpolation is the reverse of decimation:
 * - For each input sample, produce 2 output samples
 * - Insert zeros between samples, then filter
 * - Optimized: only compute FIR for non-zero taps
 */

/*
 * Interpolate CENTER mode (no frequency shift)
 * Returns 1 when need new input, 0 when same input produces more output
 */
static int interpolate_center(halfband_t *hb, sample_t in_i, sample_t in_q,
                              sample_t *out_i, sample_t *out_q)
{
	if (hb->state == 0) {
		/* First output: FIR result */
		if (hb->fast_math) {
			hb->buf_i_f[hb->ptr] = (int32_t)(in_i * 4096.0);
			hb->buf_q_f[hb->ptr] = (int32_t)(in_q * 4096.0);
			int32_t o_i, o_q;
			do_fir_fixed(hb, &o_i, &o_q);
			*out_i = (sample_t)o_i / 4096.0 * 2.0;  /* 2x gain for interpolation */
			*out_q = (sample_t)o_q / 4096.0 * 2.0;
		} else {
			hb->buf_i_d[hb->ptr] = in_i;
			hb->buf_q_d[hb->ptr] = in_q;
			double o_i, o_q;
			do_fir_double(hb, &o_i, &o_q);
			*out_i = o_i * 2.0;  /* 2x gain for interpolation */
			*out_q = o_q * 2.0;
		}
		hb->state = 1;
		return 0;  /* Need same input for second output */
	} else {
		/* Second output: center tap only (input sample passthrough) */
		*out_i = in_i;
		*out_q = in_q;
		hb->ptr = (hb->ptr + 1) % hb->order;
		hb->state = 0;
		return 1;  /* Need new input */
	}
}

/*
 * Interpolate LOWER mode (shift to lower half of spectrum)
 * Applies Fs/4 frequency shift after interpolation
 */
static int interpolate_lower(halfband_t *hb, sample_t in_i, sample_t in_q,
                             sample_t *out_i, sample_t *out_q)
{
	sample_t ti, tq;
	int need_new = interpolate_center(hb, in_i, in_q, &ti, &tq);
	
	/* Rotate by +Fs/4 (multiply by e^(j*pi/2*n)) - opposite of decimate */
	int phase = (hb->ptr * 2 + (hb->state ? 0 : 1)) & 3;
	switch (phase) {
	case 0:  /* x1 */
		*out_i = ti;
		*out_q = tq;
		break;
	case 1:  /* x(j) = swap and negate Q */
		*out_i = -tq;
		*out_q = ti;
		break;
	case 2:  /* x(-1) */
		*out_i = -ti;
		*out_q = -tq;
		break;
	case 3:  /* x(-j) = swap and negate I */
		*out_i = tq;
		*out_q = -ti;
		break;
	}
	
	return need_new;
}

/*
 * Interpolate UPPER mode (shift to upper half of spectrum)
 * Applies -Fs/4 frequency shift after interpolation
 */
static int interpolate_upper(halfband_t *hb, sample_t in_i, sample_t in_q,
                             sample_t *out_i, sample_t *out_q)
{
	sample_t ti, tq;
	int need_new = interpolate_center(hb, in_i, in_q, &ti, &tq);
	
	/* Rotate by -Fs/4 (multiply by e^(-j*pi/2*n)) - opposite of decimate */
	int phase = (hb->ptr * 2 + (hb->state ? 0 : 1)) & 3;
	switch (phase) {
	case 0:  /* x1 */
		*out_i = ti;
		*out_q = tq;
		break;
	case 1:  /* x(-j) = swap and negate I */
		*out_i = tq;
		*out_q = -ti;
		break;
	case 2:  /* x(-1) */
		*out_i = -ti;
		*out_q = -tq;
		break;
	case 3:  /* x(j) = swap and negate Q */
		*out_i = -tq;
		*out_q = ti;
		break;
	}
	
	return need_new;
}

/* Interpolate one IQ sample pair (2x upsample) */
int halfband_interpolate(halfband_t *hb, sample_t in_i, sample_t in_q,
                         sample_t *out_i, sample_t *out_q)
{
	switch (hb->mode) {
	case HALFBAND_CENTER:
		return interpolate_center(hb, in_i, in_q, out_i, out_q);
	case HALFBAND_LOWER:
		return interpolate_lower(hb, in_i, in_q, out_i, out_q);
	case HALFBAND_UPPER:
		return interpolate_upper(hb, in_i, in_q, out_i, out_q);
	default:
		*out_i = in_i;
		*out_q = in_q;
		return 1;
	}
}
