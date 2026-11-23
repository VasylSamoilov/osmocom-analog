/* Channelizer (RX) and synthesizer (TX) of one channel
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * One channel at an offset from the SDR's centre frequency, converted
 * between the SDR rate and the DSP rate:
 *
 *   RX: NCO (offset to 0 Hz) at the SDR rate -> k halfband 2x decimators
 *       -> rational resampler P/Q (if the rate is not reached yet)
 *       -> channel at 0 Hz at the DSP rate
 *   TX: channel at 0 Hz at the DSP rate -> rational resampler
 *       -> k halfband 2x interpolators -> NCO (0 Hz to offset)
 *       -> added to the SDR rate IQ
 *
 * k is as large as possible (halfbands are the cheapest stage): the rate
 * after them is at least the DSP rate. An integer ratio of the rates is
 * preferred (small P/Q); any ratio works.
 *
 * The channel may be anywhere within the SDR band; its band (bandwidth / 2)
 * must be within CHANNELIZER_PASS of the DSP rate (passband of the filters).
 * See docs/CHANNELIZER.md.
 */

#ifndef _CHANNELIZER_H
#define _CHANNELIZER_H

#include "../libsample/sample.h"
#include "halfband.h"
#include "resamp.h"

#define CHANNELIZER_HB_ORDER	HALFBAND_ORDER_64
#define CHANNELIZER_MAX_HB	16
#define CHANNELIZER_PASS	0.4	/* usable part of the DSP rate (each side) */

typedef struct channelizer {
	int		sdr_rate, dsp_rate;
	int		hb_stages;		/* k */
	int		mid_rate;		/* rate after the halfbands (RX) / before them (TX) */
	int		use_resamp;		/* mid_rate != dsp_rate */
	double		offset;			/* channel frequency - SDR centre, Hz */
	halfband_t	hb[CHANNELIZER_MAX_HB];	/* RX: hb[0] at the SDR rate; TX: hb[k-1] at the SDR rate */
	resamp_t	resamp;
	double		nco_i, nco_q;		/* NCO phasor */
	double		rot_i, rot_q;		/* NCO step per SDR sample */
	int		nco_count;		/* samples since the phasor was renormalised */
	int		tx;
	sample_t	*tmp_i, *tmp_q;		/* TX: one DSP sample expanded to SDR samples */
	sample_t	*rs_i, *rs_q;		/* TX: output of the resampler for one DSP sample */
	int		max_expand;		/* TX: most SDR samples from one DSP sample */
} channelizer_t;

/* Returns 0, or <0 with a reason in channelizer_error(). */
int channelizer_rx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth);
int channelizer_tx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth);
const char *channelizer_error(void);
void channelizer_exit(channelizer_t *ch);

/* RX: takes SDR rate samples (interleaved float I/Q) from iq, at most n, until
 * want DSP rate samples are out (or the input is used up). Returns the number
 * of output samples; *used gets the number of input samples taken. Channels
 * with the same rates take the same number of input samples. */
int channelizer_rx_process(channelizer_t *ch, const float *iq, int n, int want, sample_t *out_i, sample_t *out_q, int *used);

/* TX: n DSP rate samples in; the SDR rate samples are ADDED to iq
 * (interleaved float I/Q). Returns their number (n * sdr_rate / dsp_rate on
 * average; equal for all channels with the same rates). iq must have room
 * for channelizer_tx_max(ch, n). */
int channelizer_tx_process(channelizer_t *ch, const sample_t *in_i, const sample_t *in_q, int n, float *iq);
int channelizer_tx_max(const channelizer_t *ch, int n);

#endif /* _CHANNELIZER_H */
