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

/* renormalise the NCO phasor this often (rounding drift of its magnitude) */
#define NCO_RENORM	1024

static int init_common(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth, int tx)
{
	int k = 0, s, rc = -ENOMEM;

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

	ch->sdr_rate = sdr_rate;
	ch->dsp_rate = dsp_rate;
	ch->offset = offset;
	ch->tx = tx;

	/* As many halfbands as keep an integer rate of at least 1.25 times the
	 * DSP rate: the last one folds its transition band (0.2 .. 0.3 of its
	 * input rate) onto 0.4 .. 0.5 of its output rate, that is above 0.5 of
	 * the DSP rate, where the resampler stops. Without a resampler (the rate
	 * after the halfbands is the DSP rate) the halfbands alone pass 0.4 and
	 * stop from 0.6 (folded onto 0.4 .. 0.5) of the DSP rate. */
	while (k < CHANNELIZER_MAX_HB && !(sdr_rate % (2 << k))
	    && (sdr_rate / (2 << k) == dsp_rate || sdr_rate / (2 << k) * 4 >= dsp_rate * 5))
		k++;
	ch->mid_rate = sdr_rate >> k;
	for (s = 0; s < k; s++) {
		rc = halfband_init(&ch->hb[s], CHANNELIZER_HB_ORDER);
		if (rc < 0)
			goto fail;
		ch->hb_stages = s + 1;
	}
	if (ch->mid_rate != dsp_rate) {
		rc = tx ? resamp_init(&ch->resamp, dsp_rate, ch->mid_rate) : resamp_init(&ch->resamp, ch->mid_rate, dsp_rate);
		if (rc < 0)
			goto fail;
		ch->use_resamp = 1;
	}
	if (tx) {
		int rs_max = ch->use_resamp ? resamp_max_out(&ch->resamp) : 1;

		ch->max_expand = rs_max << k;
		ch->tmp_i = calloc(ch->max_expand, sizeof(*ch->tmp_i));
		ch->tmp_q = calloc(ch->max_expand, sizeof(*ch->tmp_q));
		ch->rs_i = calloc(rs_max, sizeof(*ch->rs_i));
		ch->rs_q = calloc(rs_max, sizeof(*ch->rs_q));
		if (!ch->tmp_i || !ch->tmp_q || !ch->rs_i || !ch->rs_q) {
			rc = -ENOMEM;
			goto fail;
		}
	}

	/* RX shifts the channel down to 0 Hz, TX up from 0 Hz */
	ch->nco_i = 1.0;
	ch->nco_q = 0.0;
	ch->rot_i = cos(2.0 * M_PI * offset / sdr_rate);
	ch->rot_q = (tx ? 1.0 : -1.0) * sin(2.0 * M_PI * offset / sdr_rate);
	return 0;

fail:
	snprintf(error_text, sizeof(error_text), "no memory");
	channelizer_exit(ch);
	return rc;
}

int channelizer_rx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth)
{
	return init_common(ch, sdr_rate, dsp_rate, offset, bandwidth, 0);
}

int channelizer_tx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth)
{
	return init_common(ch, sdr_rate, dsp_rate, offset, bandwidth, 1);
}

void channelizer_exit(channelizer_t *ch)
{
	int s;

	for (s = 0; s < ch->hb_stages; s++)
		halfband_exit(&ch->hb[s]);
	if (ch->use_resamp)
		resamp_exit(&ch->resamp);
	free(ch->tmp_i);
	free(ch->tmp_q);
	free(ch->rs_i);
	free(ch->rs_q);
	memset(ch, 0, sizeof(*ch));
}

static inline void nco_step(channelizer_t *ch)
{
	double i = ch->nco_i * ch->rot_i - ch->nco_q * ch->rot_q;
	double q = ch->nco_i * ch->rot_q + ch->nco_q * ch->rot_i;

	if (++ch->nco_count == NCO_RENORM) {
		double mag = sqrt(i * i + q * q);

		i /= mag;
		q /= mag;
		ch->nco_count = 0;
	}
	ch->nco_i = i;
	ch->nco_q = q;
}

int channelizer_rx_process(channelizer_t *ch, const float *iq, int n, int want, sample_t *out_i, sample_t *out_q, int *used)
{
	int s, st, count = 0;

	for (s = 0; s < n && count < want; s++) {
		double x_i = iq[2 * s], x_q = iq[2 * s + 1];
		sample_t i, q;

		/* mix down */
		i = x_i * ch->nco_i - x_q * ch->nco_q;
		q = x_i * ch->nco_q + x_q * ch->nco_i;
		nco_step(ch);

		for (st = 0; st < ch->hb_stages; st++)
			if (!halfband_decimate(&ch->hb[st], &i, &q))
				break;
		if (st < ch->hb_stages)
			continue;
		if (ch->use_resamp) {
			/* decimating: 0 or 1 output per input */
			sample_t ri, rq;

			if (!resamp_process(&ch->resamp, i, q, &ri, &rq))
				continue;
			i = ri;
			q = rq;
		}
		out_i[count] = i;
		out_q[count] = q;
		count++;
	}
	*used = s;
	return count;
}

int channelizer_tx_max(const channelizer_t *ch, int n)
{
	return n * ch->max_expand;
}

/* one sample before stage st -> 2^(hb_stages - st) samples; hb[0] next to the
 * resampler, hb[hb_stages - 1] at the SDR rate */
static void tx_halfbands(channelizer_t *ch, int st, sample_t i, sample_t q, sample_t **oi, sample_t **oq)
{
	sample_t ti[2], tq[2];

	if (st == ch->hb_stages) {
		*(*oi)++ = i;
		*(*oq)++ = q;
		return;
	}
	halfband_interpolate(&ch->hb[st], i, q, ti, tq);
	tx_halfbands(ch, st + 1, ti[0], tq[0], oi, oq);
	tx_halfbands(ch, st + 1, ti[1], tq[1], oi, oq);
}

int channelizer_tx_process(channelizer_t *ch, const sample_t *in_i, const sample_t *in_q, int n, float *iq)
{
	int s, p, k, rs, total = 0;

	for (s = 0; s < n; s++) {
		sample_t *oi = ch->tmp_i, *oq = ch->tmp_q;
		int count;

		if (ch->use_resamp)
			rs = resamp_process(&ch->resamp, in_i[s], in_q[s], ch->rs_i, ch->rs_q);
		else {
			ch->rs_i[0] = in_i[s];
			ch->rs_q[0] = in_q[s];
			rs = 1;
		}
		for (p = 0; p < rs; p++)
			tx_halfbands(ch, 0, ch->rs_i[p], ch->rs_q[p], &oi, &oq);
		count = oi - ch->tmp_i;
		/* mix up and add */
		for (k = 0; k < count; k++) {
			double i = ch->tmp_i[k], q = ch->tmp_q[k];

			iq[0] += i * ch->nco_i - q * ch->nco_q;
			iq[1] += i * ch->nco_q + q * ch->nco_i;
			iq += 2;
			nco_step(ch);
		}
		total += count;
	}
	return total;
}
