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
 * Stockham radix 4: a stage of length L = 4m with stride s (s m 4 = n) takes
 * a = x[q + s p], b = x[q + s (p + m)], c = x[q + s (p + 2m)],
 * d = x[q + s (p + 3m)] for p < m, q < s, and writes
 *   y[q + s 4p]       = (a + c) + (b + d)
 *   y[q + s (4p + 1)] = w^p  ((a - c) - sign j (b - d))   (sign -1: + j (b - d)... see below)
 *   y[q + s (4p + 2)] = w^2p ((a + c) - (b + d))
 *   y[q + s (4p + 3)] = w^3p ((a - c) + sign j (b - d))
 * with w = e^{sign j 2 pi / L}; then L /= 4, s *= 4. A radix 2 stage
 * (L = 2) ends odd powers. Decimation in frequency, natural order out.
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "fftc.h"

/* GCC/clang vector extension: 2 doubles, unaligned access allowed */
typedef double v2d __attribute__((vector_size(16), aligned(8), may_alias));

int fftc_init(fftc_t *f, int n, int sign)
{
	int len, st, p;

	memset(f, 0, sizeof(*f));
	if (n < 2 || (n & (n - 1)))
		return -EINVAL;
	f->n = n;
	f->sign = (sign < 0) ? -1 : 1;
	for (len = n, st = 0; len >= 4; len /= 4, st++) {
		int m = len / 4;

		if (st >= FFTC_MAX_STAGES)
			return -EINVAL;
		f->tw[st] = calloc(6 * m, sizeof(double));
		if (!f->tw[st]) {
			fftc_exit(f);
			return -ENOMEM;
		}
		/* planar: w1r[p], w1i[p], w2r[p], ... each m long */
		for (p = 0; p < m; p++) {
			double a = f->sign * 2.0 * M_PI * p / len;

			f->tw[st][0 * m + p] = cos(a);
			f->tw[st][1 * m + p] = sin(a);
			f->tw[st][2 * m + p] = cos(2.0 * a);
			f->tw[st][3 * m + p] = sin(2.0 * a);
			f->tw[st][4 * m + p] = cos(3.0 * a);
			f->tw[st][5 * m + p] = sin(3.0 * a);
		}
	}
	f->stages = st;
	f->radix2 = (len == 2);
	f->work_re = calloc(n, sizeof(double));
	f->work_im = calloc(n, sizeof(double));
	if (!f->work_re || !f->work_im) {
		fftc_exit(f);
		return -ENOMEM;
	}
	return 0;
}

void fftc_exit(fftc_t *f)
{
	int st;

	for (st = 0; st < FFTC_MAX_STAGES; st++)
		free(f->tw[st]);
	free(f->work_re);
	free(f->work_im);
	memset(f, 0, sizeof(*f));
}

/* sign as a constant: the compiler makes one loop per direction */
static inline __attribute__((always_inline)) void process(fftc_t *f, double *re, double *im, const int sign)
{
	const int n = f->n;
	double *xr = re, *xi = im, *yr = f->work_re, *yi = f->work_im, *t;
	int len = n, s = 1, st, p, q;

	for (st = 0; st < f->stages; st++, len /= 4, s *= 4) {
		const int m = len / 4;
		const double *tw = f->tw[st];

		if (s == 1 && m >= 2) {
			/* first stage: two butterflies (p, p + 1) at a time; the
			 * outputs of one butterfly lie together (4p .. 4p + 3) */
			for (p = 0; p < m; p += 2) {
				v2d a_r = *(const v2d *)(xr + p), a_i = *(const v2d *)(xi + p);
				v2d b_r = *(const v2d *)(xr + p + m), b_i = *(const v2d *)(xi + p + m);
				v2d c_r = *(const v2d *)(xr + p + 2 * m), c_i = *(const v2d *)(xi + p + 2 * m);
				v2d d_r = *(const v2d *)(xr + p + 3 * m), d_i = *(const v2d *)(xi + p + 3 * m);
				v2d w1r = *(const v2d *)(tw + p), w1i = *(const v2d *)(tw + m + p);
				v2d w2r = *(const v2d *)(tw + 2 * m + p), w2i = *(const v2d *)(tw + 3 * m + p);
				v2d w3r = *(const v2d *)(tw + 4 * m + p), w3i = *(const v2d *)(tw + 5 * m + p);
				v2d apcr = a_r + c_r, apci = a_i + c_i;
				v2d amcr = a_r - c_r, amci = a_i - c_i;
				v2d bpdr = b_r + d_r, bpdi = b_i + d_i;
				v2d jr = (b_i - d_i) * (double)-sign, ji = (b_r - d_r) * (double)sign;
				v2d u1r = amcr + jr, u1i = amci + ji;
				v2d u2r = apcr - bpdr, u2i = apci - bpdi;
				v2d u3r = amcr - jr, u3i = amci - ji;
				v2d o0r = apcr + bpdr, o0i = apci + bpdi;
				v2d o1r = u1r * w1r - u1i * w1i, o1i = u1r * w1i + u1i * w1r;
				v2d o2r = u2r * w2r - u2i * w2i, o2i = u2r * w2i + u2i * w2r;
				v2d o3r = u3r * w3r - u3i * w3i, o3i = u3r * w3i + u3i * w3r;
				double *y = yr + 4 * p, *z = yi + 4 * p;

				y[0] = o0r[0]; y[1] = o1r[0]; y[2] = o2r[0]; y[3] = o3r[0];
				y[4] = o0r[1]; y[5] = o1r[1]; y[6] = o2r[1]; y[7] = o3r[1];
				z[0] = o0i[0]; z[1] = o1i[0]; z[2] = o2i[0]; z[3] = o3i[0];
				z[4] = o0i[1]; z[5] = o1i[1]; z[6] = o2i[1]; z[7] = o3i[1];
			}
			t = xr; xr = yr; yr = t;
			t = xi; xi = yi; yi = t;
			continue;
		}
		for (p = 0; p < m; p++) {
			const double w1r = tw[p], w1i = tw[m + p];
			const double w2r = tw[2 * m + p], w2i = tw[3 * m + p];
			const double w3r = tw[4 * m + p], w3i = tw[5 * m + p];
			const double *restrict ar = xr + s * p, *restrict ai = xi + s * p;
			const double *restrict br = xr + s * (p + m), *restrict bi = xi + s * (p + m);
			const double *restrict cr = xr + s * (p + 2 * m), *restrict ci = xi + s * (p + 2 * m);
			const double *restrict dr = xr + s * (p + 3 * m), *restrict di = xi + s * (p + 3 * m);
			double *restrict y0r = yr + s * 4 * p, *restrict y0i = yi + s * 4 * p;
			double *restrict y1r = y0r + s, *restrict y1i = y0i + s;
			double *restrict y2r = y0r + 2 * s, *restrict y2i = y0i + 2 * s;
			double *restrict y3r = y0r + 3 * s, *restrict y3i = y0i + 3 * s;

			/* two at a time (128 bit vectors: SSE2, NEON) */
			for (q = 0; q + 1 < s; q += 2) {
				v2d a_r = *(const v2d *)(ar + q), a_i = *(const v2d *)(ai + q);
				v2d b_r = *(const v2d *)(br + q), b_i = *(const v2d *)(bi + q);
				v2d c_r = *(const v2d *)(cr + q), c_i = *(const v2d *)(ci + q);
				v2d d_r = *(const v2d *)(dr + q), d_i = *(const v2d *)(di + q);
				v2d apcr = a_r + c_r, apci = a_i + c_i;
				v2d amcr = a_r - c_r, amci = a_i - c_i;
				v2d bpdr = b_r + d_r, bpdi = b_i + d_i;
				v2d jr = (b_i - d_i) * (double)-sign, ji = (b_r - d_r) * (double)sign;
				v2d u1r = amcr + jr, u1i = amci + ji;
				v2d u2r = apcr - bpdr, u2i = apci - bpdi;
				v2d u3r = amcr - jr, u3i = amci - ji;

				*(v2d *)(y0r + q) = apcr + bpdr;
				*(v2d *)(y0i + q) = apci + bpdi;
				*(v2d *)(y1r + q) = u1r * w1r - u1i * w1i;
				*(v2d *)(y1i + q) = u1r * w1i + u1i * w1r;
				*(v2d *)(y2r + q) = u2r * w2r - u2i * w2i;
				*(v2d *)(y2i + q) = u2r * w2i + u2i * w2r;
				*(v2d *)(y3r + q) = u3r * w3r - u3i * w3i;
				*(v2d *)(y3i + q) = u3r * w3i + u3i * w3r;
			}
			for (; q < s; q++) {
				double apcr = ar[q] + cr[q], apci = ai[q] + ci[q];
				double amcr = ar[q] - cr[q], amci = ai[q] - ci[q];
				double bpdr = br[q] + dr[q], bpdi = bi[q] + di[q];
				/* sign j (b - d) */
				double jr = -sign * (bi[q] - di[q]), ji = sign * (br[q] - dr[q]);
				double u1r = amcr + jr, u1i = amci + ji;
				double u2r = apcr - bpdr, u2i = apci - bpdi;
				double u3r = amcr - jr, u3i = amci - ji;

				y0r[q] = apcr + bpdr;
				y0i[q] = apci + bpdi;
				y1r[q] = u1r * w1r - u1i * w1i;
				y1i[q] = u1r * w1i + u1i * w1r;
				y2r[q] = u2r * w2r - u2i * w2i;
				y2i[q] = u2r * w2i + u2i * w2r;
				y3r[q] = u3r * w3r - u3i * w3i;
				y3i[q] = u3r * w3i + u3i * w3r;
			}
		}
		t = xr; xr = yr; yr = t;
		t = xi; xi = yi; yi = t;
	}
	if (f->radix2) {
		/* len 2, s = n/2: y[q] = x[q] + x[q + s], y[q + s] = x[q] - x[q + s] */
		for (q = 0; q + 1 < s; q += 2) {
			v2d a_r = *(const v2d *)(xr + q), a_i = *(const v2d *)(xi + q);
			v2d b_r = *(const v2d *)(xr + q + s), b_i = *(const v2d *)(xi + q + s);

			*(v2d *)(yr + q) = a_r + b_r;
			*(v2d *)(yi + q) = a_i + b_i;
			*(v2d *)(yr + q + s) = a_r - b_r;
			*(v2d *)(yi + q + s) = a_i - b_i;
		}
		for (; q < s; q++) {
			double ar = xr[q], ai = xi[q], br = xr[q + s], bi = xi[q + s];

			yr[q] = ar + br;
			yi[q] = ai + bi;
			yr[q + s] = ar - br;
			yi[q + s] = ai - bi;
		}
		t = xr; xr = yr; yr = t;
		t = xi; xi = yi; yi = t;
	}
	if (xr != re) {
		memcpy(re, xr, n * sizeof(*re));
		memcpy(im, xi, n * sizeof(*im));
	}
}

#if defined(__x86_64__) && defined(__GNUC__) && !defined(__clang__)
__attribute__((target_clones("arch=x86-64-v3", "default")))
#endif
void fftc_process(fftc_t *f, double *re, double *im)
{
	if (f->sign < 0)
		process(f, re, im, -1);
	else
		process(f, re, im, 1);
}
