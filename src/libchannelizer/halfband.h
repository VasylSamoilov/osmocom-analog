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
 * channelizer shifts each channel to 0 Hz with an NCO first). Passband
 * 0 .. 0.2, stopband 0.3 .. 0.5 of the higher rate.
 * Measured (docs/CHANNELIZER.md): order 48 rejects aliases / images by
 * 81 dB or more, order 64 by 104 dB or more.
 */

#ifndef _HALFBAND_H
#define _HALFBAND_H

#include "../libsample/sample.h"

#define HALFBAND_ORDER_32	32
#define HALFBAND_ORDER_48	48
#define HALFBAND_ORDER_64	64

typedef struct halfband {
	int		order;		/* filter order (32, 48, 64) */
	int		num_coeffs;	/* non-zero coefficients on one side */
	const double	*coeffs;	/* coeffs[0] outermost .. coeffs[n-1] next to the centre */
	int		phase;		/* decimation: 0 = store only, 1 = store and output */
	int		ptr;		/* ring position */
	double		*buf_i, *buf_q;	/* window stored twice, reads never wrap */
} halfband_t;

int halfband_init(halfband_t *hb, int order);
void halfband_exit(halfband_t *hb);

/* 2x decimation: one input sample, returns 1 and the output in *i, *q every
 * second call, else 0. Group delay order/2 input samples. */
int halfband_decimate(halfband_t *hb, sample_t *i, sample_t *q);

/* 2x interpolation, polyphase form: one input sample gives two output
 * samples, in time order. Unity gain in the passband. */
void halfband_interpolate(halfband_t *hb, sample_t in_i, sample_t in_q, sample_t out_i[2], sample_t out_q[2]);

#endif /* _HALFBAND_H */
