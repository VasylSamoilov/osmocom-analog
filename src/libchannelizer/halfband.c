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
#include "halfband.h"
#include "halfband_coeffs.h"

int halfband_init(halfband_t *hb, int order)
{
	memset(hb, 0, sizeof(*hb));

	switch (order) {
	case HALFBAND_ORDER_32:
		hb->num_coeffs = 8;
		hb->coeffs = halfband_coeffs_32;
		break;
	case HALFBAND_ORDER_48:
		hb->num_coeffs = 12;
		hb->coeffs = halfband_coeffs_48;
		break;
	case HALFBAND_ORDER_64:
		hb->num_coeffs = 16;
		hb->coeffs = halfband_coeffs_64;
		break;
	default:
		fprintf(stderr, "Halfband: unsupported order %d\n", order);
		return -EINVAL;
	}
	hb->order = order;

	/* the window is stored twice (at ptr and ptr + length), so a read of
	 * length samples from ptr never wraps */
	hb->buf_i = calloc(2 * order, sizeof(*hb->buf_i));
	hb->buf_q = calloc(2 * order, sizeof(*hb->buf_q));
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
int halfband_decimate(halfband_t *hb, sample_t *i, sample_t *q)
{
	int n = hb->num_coeffs, order = hb->order, j;
	double acc_i = 0.0, acc_q = 0.0;
	const double *bi, *bq;

	hb->buf_i[hb->ptr] = hb->buf_i[hb->ptr + order] = *i;
	hb->buf_q[hb->ptr] = hb->buf_q[hb->ptr + order] = *q;
	if (++hb->ptr == order)
		hb->ptr = 0;

	if (!hb->phase) {
		hb->phase = 1;
		return 0;
	}
	hb->phase = 0;

	bi = hb->buf_i + hb->ptr;
	bq = hb->buf_q + hb->ptr;
	for (j = 0; j < n; j++) {
		int l = 1 + 2 * j, r = order - 1 - 2 * j;

		acc_i += (bi[l] + bi[r]) * hb->coeffs[j];
		acc_q += (bq[l] + bq[r]) * hb->coeffs[j];
	}
	acc_i += bi[order / 2] * 0.5;
	acc_q += bq[order / 2] * 0.5;
	*i = acc_i;
	*q = acc_q;
	return 1;
}

/* Zero-stuffing the input and filtering with the halfband h (gain 2, centre
 * at a delay of L-1 output samples, L = order/2) gives two outputs per input
 * x[m], in time order:
 *   out[0] = 2 * sum c_j * (x[m - (L-1-d_j)/2] + x[m - (L-1+d_j)/2])
 *   out[1] = x[m - (L-2)/2]                     (centre tap 0.5 * 2)
 * with d_j = 2*(n-j)-1. The window of L input samples is stored twice. */
void halfband_interpolate(halfband_t *hb, sample_t in_i, sample_t in_q, sample_t out_i[2], sample_t out_q[2])
{
	int L = hb->order / 2, n = hb->num_coeffs, j;
	double acc_i = 0.0, acc_q = 0.0;
	const double *wi, *wq;

	hb->buf_i[hb->ptr] = hb->buf_i[hb->ptr + L] = in_i;
	hb->buf_q[hb->ptr] = hb->buf_q[hb->ptr + L] = in_q;
	if (++hb->ptr == L)
		hb->ptr = 0;
	/* x[m - k] = w[L-1-k] */
	wi = hb->buf_i + hb->ptr;
	wq = hb->buf_q + hb->ptr;
	for (j = 0; j < n; j++) {
		int d = 2 * (n - j) - 1;
		int a = L - 1 - (L - 1 - d) / 2;
		int b = L - 1 - (L - 1 + d) / 2;

		acc_i += (wi[a] + wi[b]) * hb->coeffs[j];
		acc_q += (wq[a] + wq[b]) * hb->coeffs[j];
	}
	out_i[0] = 2.0 * acc_i;
	out_q[0] = 2.0 * acc_q;
	out_i[1] = wi[L - 1 - (L - 2) / 2];
	out_q[1] = wq[L - 1 - (L - 2) / 2];
}
