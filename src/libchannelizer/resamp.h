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
 * Output rate = input rate * P / Q (P, Q coprime, either direction).
 * Linear phase lowpass, Kaiser window, designed at the upsampled rate P * fin:
 * passband 0 .. RESAMP_PASS, stopband from RESAMP_STOP of the lower of the
 * two rates, attenuation RESAMP_ATTEN_DB (as the order 64 halfband).
 */

#ifndef _RESAMP_H
#define _RESAMP_H

#include "../libsample/sample.h"

#define RESAMP_ATTEN_DB	100.0
#define RESAMP_PASS	0.4	/* passband edge, fraction of the lower rate */
#define RESAMP_STOP	0.5	/* stopband edge, fraction of the lower rate: nothing folds into its band */

typedef struct resamp {
	int		p, q;		/* up, down */
	int		len;		/* taps per phase = input samples in the window */
	double		*h;		/* len * p taps, h[j * p + phase] */
	int		ptr;		/* ring position */
	double		*buf_i, *buf_q;	/* ring of len input samples, stored twice */
	int		phase;		/* output position within the current input, in 1/p input samples */
} resamp_t;

int resamp_init(resamp_t *r, int in_rate, int out_rate);
void resamp_exit(resamp_t *r);

/* One input sample; writes 0 or more output samples (at most
 * resamp_max_out()) and returns their number. */
int resamp_process(resamp_t *r, sample_t in_i, sample_t in_q, sample_t *out_i, sample_t *out_q);
int resamp_max_out(const resamp_t *r);

#endif /* _RESAMP_H */
