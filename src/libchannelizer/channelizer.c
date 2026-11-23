/* Channelizer (RX) and synthesizer (TX) of one channel
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
#include <math.h>
#include "channelizer.h"

static char error_text[256];

const char *channelizer_error(void)
{
	return error_text;
}

static int init_common(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth, int max_block, int tx)
{
	int k = 0, s, rc = -ENOMEM, work;

	memset(ch, 0, sizeof(*ch));
	error_text[0] = '\0';
	if (sdr_rate <= 0 || dsp_rate <= 0 || sdr_rate < dsp_rate) {
		snprintf(error_text, sizeof(error_text), "the SDR rate %d must be at least the DSP rate %d", sdr_rate, dsp_rate);
		return -EINVAL;
	}
	if (bandwidth / 2.0 > CHANNELIZER_PASS * dsp_rate) {
		snprintf(error_text, sizeof(error_text), "the channel bandwidth %.0f Hz needs a DSP rate of at least %.0f Hz (half of it within %.0f%% of the DSP rate)",
			 bandwidth, ceil(bandwidth / 2.0 / CHANNELIZER_PASS), CHANNELIZER_PASS * 100.0);
		return -EINVAL;
	}
	if (fabs(offset) + bandwidth / 2.0 > sdr_rate / 2.0) {
		snprintf(error_text, sizeof(error_text), "the channel at %+.0f Hz (bandwidth %.0f Hz) is outside the SDR band of +-%.0f Hz", offset, bandwidth, sdr_rate / 2.0);
		return -EINVAL;
	}
	if (max_block < 1) {
		snprintf(error_text, sizeof(error_text), "no block size");
		return -EINVAL;
	}

	ch->sdr_rate = sdr_rate;
	ch->dsp_rate = dsp_rate;
	ch->offset = offset;
	ch->tx = tx;
	ch->max_block = max_block;

	/* As many halfbands as keep an integer rate of at least 1.25 times the
	 * DSP rate (or reach it exactly): the last one folds onto 0.4 .. 0.5 of
	 * its output rate, above 0.5 of the DSP rate, where the resampler
	 * stops. */
	while (k < CHANNELIZER_MAX_HB && !(sdr_rate % (2 << k))
	    && (sdr_rate / (2 << k) == dsp_rate || sdr_rate / (2 << k) * 4 >= dsp_rate * 5))
		k++;
	ch->mid_rate = sdr_rate >> k;
	ch->use_resamp = (ch->mid_rate != dsp_rate);

	/* Each halfband for its stage (channel at 0 Hz). Stage s (RX: from the
	 * SDR side) has the higher rate sdr_rate / 2^s; it must keep 0 .. W
	 * flat and not fold anything onto it, W = where the stages after it
	 * stop (reach the full attenuation):
	 * - with the resampler: 0.5 of the DSP rate (RESAMP_STOP)
	 * - else the last halfband delivers the DSP rate: it passes 0.4 of it
	 *   (its own W) and stops from 0.6 of it, so W = 0.6 before it.
	 * TX: the same stages in reverse, the images fall where the aliases
	 * would. */
	for (s = 0; s < k; s++) {
		double rate = (double)(sdr_rate >> s), pass;
		int hs = tx ? k - 1 - s : s;	/* index in hb[]: TX hb[k-1] at the SDR rate */

		if (ch->use_resamp)
			pass = RESAMP_STOP * dsp_rate / rate;
		else if (s == k - 1)
			pass = CHANNELIZER_PASS * dsp_rate / rate;
		else
			pass = (1.0 - CHANNELIZER_PASS) * dsp_rate / rate;
		rc = halfband_design(&ch->hb[hs], pass, CHANNELIZER_ATTEN_DB);
		if (rc < 0) {
			snprintf(error_text, sizeof(error_text), "no halfband for stage %d (passband %.4f)", s, pass);
			goto fail_text;
		}
		ch->hb_stages++;
	}
	if (ch->use_resamp) {
		rc = tx ? resamp_init(&ch->resamp, dsp_rate, ch->mid_rate) : resamp_init(&ch->resamp, ch->mid_rate, dsp_rate);
		if (rc < 0)
			goto fail;
	}

	/* block buffers: the largest number of samples at any stage */
	if (tx)
		work = channelizer_tx_max(ch, max_block);
	else
		work = max_block;
	ch->work_i = calloc(work, sizeof(*ch->work_i));
	ch->work_q = calloc(work, sizeof(*ch->work_q));
	ch->work2_i = calloc(work, sizeof(*ch->work2_i));
	ch->work2_q = calloc(work, sizeof(*ch->work2_q));
	if (!tx) {
		ch->fifo_size = (int)((double)max_block * dsp_rate / sdr_rate) + 64;
		ch->fifo_i = calloc(ch->fifo_size, sizeof(*ch->fifo_i));
		ch->fifo_q = calloc(ch->fifo_size, sizeof(*ch->fifo_q));
	}
	if (!ch->work_i || !ch->work_q || !ch->work2_i || !ch->work2_q || (!tx && (!ch->fifo_i || !ch->fifo_q))) {
		rc = -ENOMEM;
		goto fail;
	}

	/* RX shifts the channel down to 0 Hz, TX up from 0 Hz */
	ch->nco_i = 1.0;
	ch->nco_q = 0.0;
	ch->rot_i = cos(2.0 * M_PI * offset / sdr_rate);
	ch->rot_q = (tx ? 1.0 : -1.0) * sin(2.0 * M_PI * offset / sdr_rate);
	return 0;

fail:
	snprintf(error_text, sizeof(error_text), "no memory");
fail_text:
	channelizer_exit(ch);
	return rc;
}

int channelizer_rx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth, int max_block)
{
	return init_common(ch, sdr_rate, dsp_rate, offset, bandwidth, max_block, 0);
}

int channelizer_tx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth, int max_block)
{
	return init_common(ch, sdr_rate, dsp_rate, offset, bandwidth, max_block, 1);
}

void channelizer_exit(channelizer_t *ch)
{
	int s;

	for (s = 0; s < CHANNELIZER_MAX_HB; s++)
		if (ch->hb[s].order)
			halfband_exit(&ch->hb[s]);
	if (ch->use_resamp)
		resamp_exit(&ch->resamp);
	free(ch->work_i);
	free(ch->work_q);
	free(ch->work2_i);
	free(ch->work2_q);
	free(ch->fifo_i);
	free(ch->fifo_q);
	memset(ch, 0, sizeof(*ch));
}

/* mix a block with the NCO (in place), renormalising the phasor at the end */
static void nco_block(channelizer_t *ch, sample_t *i, sample_t *q, int n)
{
	double pi = ch->nco_i, pq = ch->nco_q, ri = ch->rot_i, rq = ch->rot_q, mag;
	int s;

	for (s = 0; s < n; s++) {
		double xi = i[s], xq = q[s], t;

		i[s] = xi * pi - xq * pq;
		q[s] = xi * pq + xq * pi;
		t = pi * ri - pq * rq;
		pq = pi * rq + pq * ri;
		pi = t;
	}
	mag = sqrt(pi * pi + pq * pq);
	ch->nco_i = pi / mag;
	ch->nco_q = pq / mag;
}

int channelizer_rx_need(const channelizer_t *ch, int want)
{
	int m = want - ch->fifo_len, s, n;

	if (m <= 0)
		return 0;
	/* resampler: output k (from 0) comes at phase + k q (in 1/p input
	 * samples), with input floor((phase + k q) / p) + 1 */
	if (ch->use_resamp) {
		long long phase = ch->resamp.phase, p = ch->resamp.p, q = ch->resamp.q;

		n = (int)((phase + (long long)(m - 1) * q) / p) + 1;
		m = n;
	}
	/* halfbands from the DSP side: m outputs need 2m inputs, one less if
	 * one is stored already */
	for (s = ch->hb_stages - 1; s >= 0; s--)
		m = 2 * m - ch->hb[s].phase;
	return m;
}

int channelizer_rx_process(channelizer_t *ch, const float *iq, int n, int want, sample_t *out_i, sample_t *out_q)
{
	sample_t *wi = ch->work_i, *wq = ch->work_q;
	int s, st, count, give = 0;

	/* output kept from the last call first */
	if (ch->fifo_len) {
		give = (ch->fifo_len < want) ? ch->fifo_len : want;
		memcpy(out_i, ch->fifo_i, give * sizeof(*out_i));
		memcpy(out_q, ch->fifo_q, give * sizeof(*out_q));
		ch->fifo_len -= give;
		memmove(ch->fifo_i, ch->fifo_i + give, ch->fifo_len * sizeof(*ch->fifo_i));
		memmove(ch->fifo_q, ch->fifo_q + give, ch->fifo_len * sizeof(*ch->fifo_q));
	}
	if (n > ch->max_block)
		n = ch->max_block;
	if (n <= 0)
		return give;

	for (s = 0; s < n; s++) {
		wi[s] = iq[2 * s];
		wq[s] = iq[2 * s + 1];
	}
	nco_block(ch, wi, wq, n);
	count = n;
	for (st = 0; st < ch->hb_stages; st++)
		count = halfband_decimate_block(&ch->hb[st], wi, wq, count);
	if (ch->use_resamp) {
		int r = 0;

		for (s = 0; s < count; s++) {
			sample_t ri, rq;

			/* decimating: 0 or 1 output per input */
			if (resamp_process(&ch->resamp, wi[s], wq[s], &ri, &rq)) {
				ch->work2_i[r] = ri;
				ch->work2_q[r] = rq;
				r++;
			}
		}
		wi = ch->work2_i;
		wq = ch->work2_q;
		count = r;
	}
	/* to the caller up to want, the rest kept */
	for (s = 0; s < count; s++) {
		if (give < want) {
			out_i[give] = wi[s];
			out_q[give] = wq[s];
			give++;
		} else if (ch->fifo_len < ch->fifo_size) {
			ch->fifo_i[ch->fifo_len] = wi[s];
			ch->fifo_q[ch->fifo_len] = wq[s];
			ch->fifo_len++;
		}
	}
	return give;
}

int channelizer_tx_max(const channelizer_t *ch, int n)
{
	int rs = ch->use_resamp ? (n * resamp_max_out(&ch->resamp)) : n;

	return rs << ch->hb_stages;
}

int channelizer_tx_process(channelizer_t *ch, const sample_t *in_i, const sample_t *in_q, int n, float *iq)
{
	sample_t *ai = ch->work_i, *aq = ch->work_q, *bi = ch->work2_i, *bq = ch->work2_q, *t;
	int s, st, count;

	if (n > ch->max_block)
		n = ch->max_block;
	if (ch->use_resamp) {
		count = 0;
		for (s = 0; s < n; s++)
			count += resamp_process(&ch->resamp, in_i[s], in_q[s], ai + count, aq + count);
	} else {
		memcpy(ai, in_i, n * sizeof(*ai));
		memcpy(aq, in_q, n * sizeof(*aq));
		count = n;
	}
	/* halfbands from the DSP side: hb[0] next to the resampler */
	for (st = 0; st < ch->hb_stages; st++) {
		halfband_interpolate_block(&ch->hb[st], ai, aq, count, bi, bq);
		count *= 2;
		t = ai; ai = bi; bi = t;
		t = aq; aq = bq; bq = t;
	}
	nco_block(ch, ai, aq, count);
	for (s = 0; s < count; s++) {
		iq[2 * s] += ai[s];
		iq[2 * s + 1] += aq[s];
	}
	return count;
}
