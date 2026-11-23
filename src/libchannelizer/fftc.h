/* Complex FFT of a fixed power-of-two size with precomputed twiddles
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Stockham autosort, radix 4 (one radix 2 stage at the end for odd powers),
 * separate real and imaginary arrays, no scaling. No bit reversal; every
 * stage reads and writes contiguous runs. For the filter banks of the
 * channelizer: one size, many calls, so the twiddles of every stage are made
 * once.
 */

#ifndef _FFTC_H
#define _FFTC_H

#define FFTC_MAX_STAGES	16

typedef struct fftc {
	int		n;
	int		stages;			/* radix 4 stages */
	int		radix2;			/* one radix 2 stage at the end */
	double		*tw[FFTC_MAX_STAGES];	/* per stage: w1, w2, w3 (re, im) for p < m */
	double		*work_re, *work_im;
	int		sign;
} fftc_t;

/* sign = -1: forward (e^{-j..}), +1: inverse without 1/n */
int fftc_init(fftc_t *f, int n, int sign);
void fftc_exit(fftc_t *f);
/* in place */
void fftc_process(fftc_t *f, double *re, double *im);

#endif /* _FFTC_H */
