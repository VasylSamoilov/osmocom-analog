/* Polyphase FFT filter banks (RX analysis, TX synthesis) and their channels
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RX, bin k, frame m (newest input n = mD + D - 1):
 *   y_k[m] = sum_l h[l] x[n - l] e^{-j 2 pi k (n - l) / M}
 *          = e^{-j 2 pi k n / M} sum_r u[r] e^{+j 2 pi k r / M}
 *   u[r]   = sum_p h[r + pM] x[n - r - pM]
 * the sum over r is an inverse FFT; e^{-j 2 pi k n / M} = (-1)^(k m)
 * e^{-j 2 pi k (D - 1) / M}.
 *
 * TX: x[n] = sum_m g[n - mD] V_m[n mod M], V_m the inverse FFT of frame m,
 * g = D h. For n = m0 D + j (j < D):
 *   x[n] = sum_p g[j + pD] V_{m0 - p}[j + (m0 odd ? D : 0)]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "pfb.h"

/* GCC/clang vector extension: 2 doubles (SSE2, NEON), unaligned */
typedef double v2d __attribute__((vector_size(16), aligned(8), may_alias));

static char error_text[256];

const char *pfb_error(void)
{
	return error_text;
}

static double bessel_i0(double x)
{
	double sum = 1.0, term = 1.0;
	int k;

	for (k = 1; k < 200; k++) {
		term *= (x / (2.0 * k)) * (x / (2.0 * k));
		sum += term;
		if (term < sum * 1e-17)
			break;
	}
	return sum;
}

static int init_common(pfb_t *pfb, int fs, int m, double clean, int tx)
{
	double spacing, fb, beta, cutoff, sum = 0.0, c;
	int len, l, t, p, rc;

	memset(pfb, 0, sizeof(*pfb));
	error_text[0] = '\0';
	if (m < 8 || (m & (m - 1))) {
		snprintf(error_text, sizeof(error_text), "%d bins: must be a power of two, at least 8", m);
		return -EINVAL;
	}
	if (fs <= 0 || fs % (m / 2)) {
		snprintf(error_text, sizeof(error_text), "SDR rate %d is not a multiple of %d (bins / 2)", fs, m / 2);
		return -EINVAL;
	}
	spacing = (double)fs / m;
	fb = 2.0 * spacing;
	pfb->pass = spacing / 2.0 + clean;
	pfb->stop = fb - pfb->pass;
	if (clean <= 0.0 || pfb->stop <= pfb->pass) {
		snprintf(error_text, sizeof(error_text), "a clean band of +-%.0f Hz needs a sub-band spacing above %.0f Hz (have %.0f)", clean, 2.0 * clean, spacing);
		return -EINVAL;
	}
	/* Kaiser: length for the attenuation over the transition, a multiple of M */
	len = (int)ceil((PFB_ATTEN_DB - 7.95) / (2.285 * 2.0 * M_PI * (pfb->stop - pfb->pass) / fs)) + 1;
	t = (len + m - 1) / m;
	len = t * m;
	beta = 0.1102 * (PFB_ATTEN_DB - 8.7);
	cutoff = (pfb->pass + pfb->stop) / 2.0 / fs;

	pfb->fs = fs;
	pfb->m = m;
	pfb->d = m / 2;
	pfb->t = t;
	pfb->len = len;
	pfb->tx = tx;
	pfb->clean = clean;
	{
		double *h = calloc(len, sizeof(*h));

		pfb->h = calloc(len, sizeof(*pfb->h));
		if (!h || !pfb->h) {
			free(h);
			goto nomem;
		}
		c = (len - 1) / 2.0;
		for (l = 0; l < len; l++) {
			double x = l - c, r = x / c;
			double s = (x == 0.0) ? 2.0 * cutoff : sin(2.0 * M_PI * cutoff * x) / (M_PI * x);

			h[l] = s * bessel_i0(beta * sqrt(fmax(0.0, 1.0 - r * r))) / bessel_i0(beta);
			sum += h[l];
		}
		for (l = 0; l < len; l++)
			h[l] /= sum;
		if (tx) {
			/* g = D h: unity gain through the interpolation by D */
			for (l = 0; l < len; l++)
				pfb->h[l] = h[l] * pfb->d;
		} else {
			/* per branch reversed: hr[pM + q] = h[pM + M-1-q] */
			for (p = 0; p < t; p++)
				for (l = 0; l < m; l++)
					pfb->h[p * m + l] = h[p * m + m - 1 - l];
		}
		free(h);
	}
	rc = fftc_init(&pfb->fft, m, +1);
	if (rc < 0)
		goto nomem;
	pfb->fr = calloc(m, sizeof(double));
	pfb->fi = calloc(m, sizeof(double));
	if (!pfb->fr || !pfb->fi)
		goto nomem;
	if (tx) {
		int np = len / pfb->d;

		pfb->ring_r = calloc((size_t)2 * np * m, sizeof(double));
		pfb->ring_i = calloc((size_t)2 * np * m, sizeof(double));
		if (!pfb->ring_r || !pfb->ring_i)
			goto nomem;
	} else {
		pfb->hist_i = calloc(2 * len, sizeof(double));
		pfb->hist_q = calloc(2 * len, sizeof(double));
		pfb->acc_i = calloc(m, sizeof(double));
		pfb->acc_q = calloc(m, sizeof(double));
		pfb->rot_r = calloc(m, sizeof(double));
		pfb->rot_i = calloc(m, sizeof(double));
		if (!pfb->hist_i || !pfb->hist_q || !pfb->acc_i || !pfb->acc_q || !pfb->rot_r || !pfb->rot_i)
			goto nomem;
		for (l = 0; l < m; l++) {
			pfb->rot_r[l] = cos(-2.0 * M_PI * (double)l * (pfb->d - 1) / m);
			pfb->rot_i[l] = sin(-2.0 * M_PI * (double)l * (pfb->d - 1) / m);
		}
		pfb->pos = len;	/* zeros before the start */
	}
	return 0;

nomem:
	snprintf(error_text, sizeof(error_text), "no memory");
	pfb_exit(pfb);
	return -ENOMEM;
}

int pfb_rx_init(pfb_t *pfb, int fs, int m, double clean)
{
	return init_common(pfb, fs, m, clean, 0);
}

int pfb_tx_init(pfb_t *pfb, int fs, int m, double clean)
{
	return init_common(pfb, fs, m, clean, 1);
}

void pfb_exit(pfb_t *pfb)
{
	if (pfb->fft.n)
		fftc_exit(&pfb->fft);
	free(pfb->h);
	free(pfb->fr);
	free(pfb->fi);
	free(pfb->rot_r);
	free(pfb->rot_i);
	free(pfb->hist_i);
	free(pfb->hist_q);
	free(pfb->acc_i);
	free(pfb->acc_q);
	free(pfb->ring_r);
	free(pfb->ring_i);
	memset(pfb, 0, sizeof(*pfb));
}

int pfb_bin_rate(const pfb_t *pfb)
{
	return pfb->fs / pfb->d;
}

int pfb_rx_frames(const pfb_t *pfb, int n)
{
	return (pfb->fill + n) / pfb->d;
}

int pfb_rx_need(const pfb_t *pfb, int f)
{
	if (f <= 0)
		return 0;
	return (pfb->d - pfb->fill) + (f - 1) * pfb->d;
}

int pfb_rx_process(pfb_t *pfb, const float *iq, int n, int nbins, const int *bins, sample_t **out_i, sample_t **out_q)
{
	const int len = pfb->len, m = pfb->m, d = pfb->d, t = pfb->t;
	int s = 0, frames = 0, r, p, c;

	while (s < n) {
		int take = d - pfb->fill;

		if (take > n - s)
			take = n - s;
		/* the history of len samples sits at [pos - len, pos) */
		if (pfb->pos + take > 2 * len) {
			memmove(pfb->hist_i, pfb->hist_i + pfb->pos - len, len * sizeof(double));
			memmove(pfb->hist_q, pfb->hist_q + pfb->pos - len, len * sizeof(double));
			pfb->pos = len;
		}
		for (r = 0; r < take; r++) {
			pfb->hist_i[pfb->pos + r] = iq[2 * (s + r)];
			pfb->hist_q[pfb->pos + r] = iq[2 * (s + r) + 1];
		}
		pfb->pos += take;
		pfb->fill += take;
		s += take;
		if (pfb->fill < d)
			break;
		pfb->fill = 0;

		/* newest x[n] at hist[pos - 1]; branch p, element q = M-1-r:
		 * hr[pM + q] x = hist[pos - pM - M + q]; 4 sums of I and Q in
		 * registers over all branches */
		const double *x0i = pfb->hist_i + pfb->pos - m;
		const double *x0q = pfb->hist_q + pfb->pos - m;
		double *ai = pfb->acc_i, *aq = pfb->acc_q;

		for (r = 0; r < m; r += 4) {
			v2d i01 = { 0, 0 }, i23 = { 0, 0 }, q01 = { 0, 0 }, q23 = { 0, 0 };

			for (p = 0; p < t; p++) {
				const double *hp = pfb->h + p * m + r;
				const double *xi = x0i - p * m + r, *xq = x0q - p * m + r;
				v2d h01 = *(const v2d *)hp, h23 = *(const v2d *)(hp + 2);

				i01 += h01 * *(const v2d *)xi;
				i23 += h23 * *(const v2d *)(xi + 2);
				q01 += h01 * *(const v2d *)xq;
				q23 += h23 * *(const v2d *)(xq + 2);
			}
			ai[r] = i01[0]; ai[r + 1] = i01[1]; ai[r + 2] = i23[0]; ai[r + 3] = i23[1];
			aq[r] = q01[0]; aq[r + 1] = q01[1]; aq[r + 2] = q23[0]; aq[r + 3] = q23[1];
		}
		for (r = 0; r < m; r++) {
			pfb->fr[r] = ai[m - 1 - r];
			pfb->fi[r] = aq[m - 1 - r];
		}
		fftc_process(&pfb->fft, pfb->fr, pfb->fi);
		for (c = 0; c < nbins; c++) {
			int k = bins[c];
			double vr = pfb->fr[k] * pfb->rot_r[k] - pfb->fi[k] * pfb->rot_i[k];
			double vi = pfb->fr[k] * pfb->rot_i[k] + pfb->fi[k] * pfb->rot_r[k];

			if ((k & 1) && (pfb->frame & 1)) {
				vr = -vr;
				vi = -vi;
			}
			out_i[c][frames] = vr;
			out_q[c][frames] = vi;
		}
		pfb->frame++;
		frames++;
	}
	return frames;
}

int pfb_tx_process(pfb_t *pfb, int nbins, const int *bins, const sample_t *const *in_i, const sample_t *const *in_q, int frames, float *iq)
{
	const int m = pfb->m, d = pfb->d, np = pfb->len / d;
	int f, out = 0, c, j, q;

	for (f = 0; f < frames; f++) {
		double *vr = pfb->ring_r + pfb->rpos * m, *vi = pfb->ring_i + pfb->rpos * m;
		int off = (pfb->frame & 1) ? d : 0;

		memset(vr, 0, m * sizeof(*vr));
		memset(vi, 0, m * sizeof(*vi));
		for (c = 0; c < nbins; c++) {
			vr[bins[c]] += in_i[c][f];
			vi[bins[c]] += in_q[c][f];
		}
		fftc_process(&pfb->fft, vr, vi);
		memcpy(pfb->ring_r + (pfb->rpos + np) * m, vr, m * sizeof(*vr));
		memcpy(pfb->ring_i + (pfb->rpos + np) * m, vi, m * sizeof(*vi));
		/* newest frame at ring[rpos + np], frame m0 - p at ring[rpos + np - p] */
		const double *br = pfb->ring_r + (pfb->rpos + np) * m + off;
		const double *bi = pfb->ring_i + (pfb->rpos + np) * m + off;
		const double *g = pfb->h;

		for (j = 0; j < d; j += 4) {
			v2d i01 = { 0, 0 }, i23 = { 0, 0 }, q01 = { 0, 0 }, q23 = { 0, 0 };

			for (q = 0; q < np; q++) {
				const double *gq = g + j + q * d;
				const double *xr = br - q * m + j, *xi = bi - q * m + j;
				v2d g01 = *(const v2d *)gq, g23 = *(const v2d *)(gq + 2);

				i01 += g01 * *(const v2d *)xr;
				i23 += g23 * *(const v2d *)(xr + 2);
				q01 += g01 * *(const v2d *)xi;
				q23 += g23 * *(const v2d *)(xi + 2);
			}
			float *o = iq + 2 * (out + j);
			o[0] += i01[0]; o[1] += q01[0]; o[2] += i01[1]; o[3] += q01[1];
			o[4] += i23[0]; o[5] += q23[0]; o[6] += i23[1]; o[7] += q23[1];
		}
		if (++pfb->rpos == np)
			pfb->rpos = 0;
		out += d;
		pfb->frame++;
	}
	return out;
}

/*
 * channels
 */

static int chan_init(pfb_chan_t *ch, const pfb_t *pfb, double offset, int dsp_rate, int max, int tx)
{
	double spacing = (double)pfb->fs / pfb->m, resid, low;
	int b, fb = pfb_bin_rate(pfb), mid = 2 * dsp_rate, rc, mid_max, out_max;

	memset(ch, 0, sizeof(*ch));
	error_text[0] = '\0';
	if (fabs(offset) + dsp_rate / 2.0 > pfb->fs / 2.0) {
		snprintf(error_text, sizeof(error_text), "the channel at %+.0f Hz (DSP rate %d) is outside the SDR band of +-%.0f Hz", offset, dsp_rate, pfb->fs / 2.0);
		return -EINVAL;
	}
	if (dsp_rate / 2.0 > pfb->clean + 1e-6) {
		snprintf(error_text, sizeof(error_text), "DSP rate %d: the bank keeps only +-%.0f Hz clean (DSP rate up to %.0f)", dsp_rate, pfb->clean, 2.0 * pfb->clean);
		return -EINVAL;
	}
	b = (int)lround(offset / spacing);
	resid = offset - b * spacing;
	ch->tx = tx;
	ch->bin = ((b % pfb->m) + pfb->m) % pfb->m;
	ch->offset = offset;
	ch->dsp_rate = dsp_rate;
	ch->bin_rate = fb;
	ch->nco_i = 1.0;
	ch->nco_q = 0.0;
	/* RX: the rest down to 0 Hz; TX: from 0 Hz up */
	ch->rot_i = cos(2.0 * M_PI * resid / fb);
	ch->rot_q = (tx ? 1.0 : -1.0) * sin(2.0 * M_PI * resid / fb);
	ch->max_in = max;

	/* the wide resampler between the bin rate and 2 dsp: keeps +-dsp/2,
	 * stops what would fold or image onto it */
	low = (fb < mid) ? fb : mid;
	ch->use_rs1 = (fb != mid);
	if (ch->use_rs1) {
		rc = tx ? resamp_init_band(&ch->rs1, mid, fb, dsp_rate / 2.0, low - dsp_rate / 2.0, PFB_ATTEN_DB)
			: resamp_init_band(&ch->rs1, fb, mid, dsp_rate / 2.0, low - dsp_rate / 2.0, PFB_ATTEN_DB);
		if (rc < 0)
			goto fail;
	}
	/* the sharp 2:1 (1:2): flat to 0.4, stop from 0.5 of the DSP rate */
	rc = tx ? resamp_init_band(&ch->rs2, dsp_rate, mid, 0.4 * dsp_rate, 0.5 * dsp_rate, PFB_ATTEN_DB)
		: resamp_init_band(&ch->rs2, mid, dsp_rate, 0.4 * dsp_rate, 0.5 * dsp_rate, PFB_ATTEN_DB);
	if (rc < 0)
		goto fail;

	if (tx) {
		/* max DSP samples in -> 2 max at 2 dsp -> frames */
		mid_max = 2 * max + 2;
		out_max = ch->use_rs1 ? mid_max * resamp_max_out(&ch->rs1) + 2 : mid_max;
	} else {
		/* max frames in -> at 2 dsp; the DSP samples kept for the next call */
		mid_max = ch->use_rs1 ? max * resamp_max_out(&ch->rs1) + 2 : max;
		out_max = mid_max / 2 + 2;
	}
	ch->mid_i = calloc(mid_max, sizeof(sample_t));
	ch->mid_q = calloc(mid_max, sizeof(sample_t));
	ch->out_i = calloc(out_max, sizeof(sample_t));
	ch->out_q = calloc(out_max, sizeof(sample_t));
	if (!ch->mid_i || !ch->mid_q || !ch->out_i || !ch->out_q) {
		rc = -ENOMEM;
		goto fail;
	}
	return 0;

fail:
	if (!error_text[0])
		snprintf(error_text, sizeof(error_text), "channel at %+.0f Hz: %s", offset, (rc == -ENOMEM) ? "no memory" : "no resampler for these rates");
	pfb_chan_exit(ch);
	return rc;
}

int pfb_chan_rx_init(pfb_chan_t *ch, const pfb_t *pfb, double offset, int dsp_rate, int max_frames)
{
	return chan_init(ch, pfb, offset, dsp_rate, max_frames, 0);
}

int pfb_chan_tx_init(pfb_chan_t *ch, const pfb_t *pfb, double offset, int dsp_rate, int max_in)
{
	return chan_init(ch, pfb, offset, dsp_rate, max_in, 1);
}

void pfb_chan_exit(pfb_chan_t *ch)
{
	if (ch->use_rs1 && ch->rs1.h)
		resamp_exit(&ch->rs1);
	if (ch->rs2.h)
		resamp_exit(&ch->rs2);
	free(ch->mid_i);
	free(ch->mid_q);
	free(ch->out_i);
	free(ch->out_q);
	memset(ch, 0, sizeof(*ch));
}

/* RX: the DSP samples beyond 'want' of a call are kept (out_i/out_q,
 * kept of them) for the next call */
int pfb_chan_rx_need(const pfb_chan_t *ch, int want)
{
	int m = want - ch->kept, n;

	if (m <= 0)
		return 0;
	n = resamp_need(&ch->rs2, m);
	if (ch->use_rs1)
		n = resamp_need(&ch->rs1, n);
	return n;
}

int pfb_chan_rx_process(pfb_chan_t *ch, const sample_t *bin_i, const sample_t *bin_q, int frames, int want, sample_t *out_i, sample_t *out_q)
{
	double pi = ch->nco_i, pq = ch->nco_q, ri = ch->rot_i, rq = ch->rot_q, mag;
	int f, k = 0, s, give, n = 0;
	sample_t ti[4], tq[4];

	if (frames > ch->max_in) {
		fprintf(stderr, "pfb_chan_rx_process: %d frames, more than %d, please fix!\n", frames, ch->max_in);
		abort();
	}
	/* kept from the last call first */
	give = (ch->kept < want) ? ch->kept : want;
	memcpy(out_i, ch->out_i, give * sizeof(*out_i));
	memcpy(out_q, ch->out_q, give * sizeof(*out_q));
	ch->kept -= give;
	memmove(ch->out_i, ch->out_i + give, ch->kept * sizeof(*ch->out_i));
	memmove(ch->out_q, ch->out_q + give, ch->kept * sizeof(*ch->out_q));

	/* fine NCO, wide resampler to 2 dsp */
	for (f = 0; f < frames; f++) {
		double xi = bin_i[f], xq = bin_q[f], t;
		double vi = xi * pi - xq * pq, vq = xi * pq + xq * pi;

		t = pi * ri - pq * rq;
		pq = pi * rq + pq * ri;
		pi = t;
		if (ch->use_rs1)
			k += resamp_process(&ch->rs1, vi, vq, ch->mid_i + k, ch->mid_q + k);
		else {
			ch->mid_i[k] = vi;
			ch->mid_q[k] = vq;
			k++;
		}
	}
	mag = sqrt(pi * pi + pq * pq);
	ch->nco_i = pi / mag;
	ch->nco_q = pq / mag;
	/* 2:1 to the DSP rate: up to want to the caller, the rest kept */
	for (s = 0; s < k; s++) {
		n = resamp_process(&ch->rs2, ch->mid_i[s], ch->mid_q[s], ti, tq);
		if (n && give < want) {
			out_i[give] = ti[0];
			out_q[give] = tq[0];
			give++;
		} else if (n) {
			ch->out_i[ch->kept] = ti[0];
			ch->out_q[ch->kept] = tq[0];
			ch->kept++;
		}
	}
	return give;
}

int pfb_chan_tx_max(const pfb_chan_t *ch, int n)
{
	int mid = 2 * n + 2;

	return ch->use_rs1 ? mid * resamp_max_out(&ch->rs1) + 2 : mid;
}

int pfb_chan_tx_process(pfb_chan_t *ch, const sample_t *in_i, const sample_t *in_q, int n)
{
	double pi = ch->nco_i, pq = ch->nco_q, ri = ch->rot_i, rq = ch->rot_q, mag;
	int s, k = 0, c = 0;

	if (n > ch->max_in) {
		fprintf(stderr, "pfb_chan_tx_process: %d samples, more than %d, please fix!\n", n, ch->max_in);
		abort();
	}
	/* 1:2 (nothing beyond +-dsp/2), wide resampler to the bin rate */
	for (s = 0; s < n; s++)
		k += resamp_process(&ch->rs2, in_i[s], in_q[s], ch->mid_i + k, ch->mid_q + k);
	if (ch->use_rs1) {
		for (s = 0; s < k; s++)
			c += resamp_process(&ch->rs1, ch->mid_i[s], ch->mid_q[s], ch->out_i + c, ch->out_q + c);
	} else {
		memcpy(ch->out_i, ch->mid_i, k * sizeof(*ch->out_i));
		memcpy(ch->out_q, ch->mid_q, k * sizeof(*ch->out_q));
		c = k;
	}
	/* fine NCO up from 0 Hz */
	for (s = 0; s < c; s++) {
		double xi = ch->out_i[s], xq = ch->out_q[s], t;

		ch->out_i[s] = xi * pi - xq * pq;
		ch->out_q[s] = xi * pq + xq * pi;
		t = pi * ri - pq * rq;
		pq = pi * rq + pq * ri;
		pi = t;
	}
	mag = sqrt(pi * pi + pq * pq);
	ch->nco_i = pi / mag;
	ch->nco_q = pq / mag;
	return c;
}
