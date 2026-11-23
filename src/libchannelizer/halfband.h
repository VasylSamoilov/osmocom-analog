/* Halfband 2x decimation / interpolation filter
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Linear phase halfband FIR, double precision, centred at 0 Hz (the
 * channelizer shifts each channel to 0 Hz with an NCO first).
 *
 * The filter is designed for its stage (halfband_design): a stage whose
 * channel band reaches 'pass' (fraction of its higher rate) must pass
 * 0 .. pass and stop 0.5 - pass .. 0.5 (what folds onto the channel band).
 * Early stages of a cascade see the channel in a tiny part of their band and
 * get by with a few taps; only the stage next to the DSP rate needs many.
 * The design is checked numerically on a fine grid: the shortest filter
 * that reaches the attenuation is taken.
 */

#ifndef _HALFBAND_H
#define _HALFBAND_H

#include "../libsample/sample.h"

#define HALFBAND_MAX_COEFFS	32	/* non-zero coefficients on one side */

typedef struct halfband {
	int		order;		/* 4 * num_coeffs: taps at odd distances 1 .. order/2 - 1 */
	int		num_coeffs;	/* non-zero coefficients on one side */
	double		coeffs[HALFBAND_MAX_COEFFS];	/* coeffs[0] outermost .. [n-1] next to the centre */
	double		pass;		/* passband edge it was designed for (fraction of the higher rate) */
	double		stop_db;	/* worst measured attenuation in the stopband */
	double		ripple_db;	/* worst measured deviation in the passband */
	int		phase;		/* decimation: 0 = store only, 1 = store and output */
	int		ptr;		/* ring position */
	double		*buf_i, *buf_q;	/* window stored twice, reads never wrap */
} halfband_t;

/* Design for a passband 0 .. pass (fraction of the higher rate, < 0.25) and
 * a stopband 0.5 - pass .. 0.5 with at least atten_db; returns 0 or <0 if no
 * filter up to HALFBAND_MAX_COEFFS reaches it. */
int halfband_design(halfband_t *hb, double pass, double atten_db);
void halfband_exit(halfband_t *hb);

/* 2x decimation of a block: n input samples (in place, I and Q separately),
 * returns the number of output samples (written from the start). */
int halfband_decimate_block(halfband_t *hb, sample_t *i, sample_t *q, int n);

/* 2x interpolation of a block: n input samples, 2n output samples in time
 * order. Unity gain in the passband. */
void halfband_interpolate_block(halfband_t *hb, const sample_t *in_i, const sample_t *in_q, int n, sample_t *out_i, sample_t *out_q);

#endif /* _HALFBAND_H */
