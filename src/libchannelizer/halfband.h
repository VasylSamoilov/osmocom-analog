/* Halfband decimation/interpolation filter
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Halfband filter for efficient 2x decimation and interpolation.
 * Supports three modes:
 * - CENTER: Extract/insert at center of spectrum (no frequency shift)
 * - LOWER: Extract lower half / insert to lower half (shift by Fs/4)
 * - UPPER: Extract upper half / insert to upper half (shift by Fs/4)
 *
 * Reference: SDRangel implementation was studied to understand the algorithm.
 */

#ifndef _HALFBAND_H
#define _HALFBAND_H

#include <stdint.h>
#include "../libsample/sample.h"

/* Supported filter orders */
#define HALFBAND_ORDER_32  32
#define HALFBAND_ORDER_48  48   /* Default - good balance */
#define HALFBAND_ORDER_64  64

/* Decimation/interpolation modes */
typedef enum {
	HALFBAND_CENTER,      /* Extract/insert at center spectrum (DC) */
	HALFBAND_LOWER,       /* Extract lower half / insert to lower half */
	HALFBAND_UPPER,       /* Extract upper half / insert to upper half */
} halfband_mode_t;

/* Halfband filter state */
typedef struct halfband {
	int order;                  /* Filter order (32, 48, or 64) */
	int num_coeffs;             /* Number of non-zero coefficients */
	halfband_mode_t mode;       /* Decimation/interpolation mode */
	int fast_math;              /* 0=double precision, 1=fixed-point */
	int state;                  /* State machine for modes (0-3) */
	int ptr;                    /* Ring buffer write position */

	/* Double precision (default) */
	double *buf_i_d;            /* I component ring buffer */
	double *buf_q_d;            /* Q component ring buffer */
	const double *coeffs_d;     /* Filter coefficients */

	/* Fixed-point (fast_math mode) */
	int32_t *buf_i_f;           /* I component ring buffer */
	int32_t *buf_q_f;           /* Q component ring buffer */
	const int32_t *coeffs_f;    /* Filter coefficients */
	int shift;                  /* Right shift for fixed-point scaling */
} halfband_t;

/* Initialize halfband filter
 * order: filter order (32, 48, or 64)
 * mode: decimation/interpolation mode (CENTER, LOWER, UPPER)
 * fast_math: 0 for double precision, 1 for fixed-point
 * Returns 0 on success, <0 on error */
int halfband_init(halfband_t *hb, int order, halfband_mode_t mode, int fast_math);

/* Free halfband filter resources */
void halfband_exit(halfband_t *hb);

/* Process one IQ sample pair (DECIMATION - 2x downsample)
 * Performs 2x decimation - returns 1 every other sample when output ready
 * i, q: input/output sample (modified in place when output ready)
 * Returns: 1 if output sample ready, 0 if no output yet */
int halfband_process(halfband_t *hb, sample_t *i, sample_t *q);

/* Interpolate one IQ sample pair (INTERPOLATION - 2x upsample)
 * For each input sample, produces 2 output samples.
 * Call twice per input: once with input, once with 0,0.
 * in_i, in_q: input sample (use 0,0 for second call)
 * out_i, out_q: output sample
 * Returns: 1 if need new input sample, 0 if same input produces more output */
int halfband_interpolate(halfband_t *hb, sample_t in_i, sample_t in_q,
                         sample_t *out_i, sample_t *out_q);

#endif /* _HALFBAND_H */

