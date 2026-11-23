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
 * k is as large as possible (halfbands are the cheapest stage). Each
 * halfband is designed for its stage: with the channel at 0 Hz a stage at
 * rate f only protects the DSP band, a part of f that halves with every stage
 * towards the SDR rate, so the early stages need few taps (halfband.h). An
 * integer ratio of the rates is preferred (small P/Q); any ratio works.
 *
 * Processing is block-wise, stage by stage. See docs/CHANNELIZER.md.
 */

#ifndef _CHANNELIZER_H
#define _CHANNELIZER_H

#include "../libsample/sample.h"
#include "halfband.h"
#include "resamp.h"

#define CHANNELIZER_MAX_HB	16
#define CHANNELIZER_PASS	0.4	/* usable part of the DSP rate (each side) */
#define CHANNELIZER_ATTEN_DB	105.0	/* every halfband stage */

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
	int		tx;
	int		max_block;		/* RX: most SDR samples per call; TX: most DSP samples per call */
	sample_t	*work_i, *work_q;	/* block buffers */
	sample_t	*work2_i, *work2_q;
	sample_t	*fifo_i, *fifo_q;	/* RX: output samples kept for the next call */
	int		fifo_len, fifo_size;
} channelizer_t;

/* max_block: RX most SDR rate samples per call, TX most DSP rate samples per
 * call. Returns 0, or <0 with a reason in channelizer_error(). */
int channelizer_rx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth, int max_block);
int channelizer_tx_init(channelizer_t *ch, int sdr_rate, int dsp_rate, double offset, double bandwidth, int max_block);
const char *channelizer_error(void);
void channelizer_exit(channelizer_t *ch);

/* RX: n SDR rate samples (interleaved float I/Q, n <= max_block) in, at most
 * want DSP rate samples out (output samples beyond want are kept for the next
 * call). Returns the number of output samples. Takes all of the input:
 * channelizer_rx_need() says how much input gives want samples. */
int channelizer_rx_process(channelizer_t *ch, const float *iq, int n, int want, sample_t *out_i, sample_t *out_q);
/* SDR rate samples to give for want more output samples (equal for all
 * channels with the same rates) */
int channelizer_rx_need(const channelizer_t *ch, int want);

/* TX: n DSP rate samples in (n <= max_block); the SDR rate samples are ADDED
 * to iq (interleaved float I/Q). Returns their number (n * sdr_rate /
 * dsp_rate on average; equal for all channels with the same rates). iq must
 * have room for channelizer_tx_max(ch, n). */
int channelizer_tx_process(channelizer_t *ch, const sample_t *in_i, const sample_t *in_q, int n, float *iq);
int channelizer_tx_max(const channelizer_t *ch, int n);

#endif /* _CHANNELIZER_H */
