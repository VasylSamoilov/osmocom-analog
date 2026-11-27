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
 * Many channels of one SDR stream at a cost that does not grow with their
 * number: the band is split into M sub-bands ("bins") of spacing fs / M, each
 * at the bin rate fb = 2 fs / M (2x oversampled, decimation M/2). One
 * prototype lowpass, one FFT per M/2 SDR samples, for all bins.
 *
 *   RX: SDR IQ -> analysis bank -> bin k = round(offset M / fs)
 *       -> channel: fine NCO (the rest of the offset, within +-fs/2M)
 *       -> wide resampler fb -> 2 dsp (keeps +-dsp/2)
 *       -> 2:1 (flat to 0.4, stop from 0.5 of dsp) -> DSP rate, 0 Hz
 *   TX: the mirror; the bank adds all bins into the SDR IQ.
 *
 * The bank keeps +-clean around every channel free of aliases and images:
 * its prototype passes to spacing/2 + clean and stops from fb - that. With
 * clean = dsp/2 the channel sees the same as from the per-channel
 * channelizer: flat to 0.4 of the DSP rate, nothing else in the whole DSP
 * band (>= 105 dB). The spacing must be more than 2 clean.
 *
 * Frame m of the RX bank is complete after SDR sample mD + D - 1 (D = M/2);
 * every channel of a bank gets the same frames, so channels with the same
 * DSP rate give the same number of samples.
 */

#ifndef _PFB_H
#define _PFB_H

#include "../libsample/sample.h"
#include "fftc.h"
#include "resamp.h"
#include "halfband.h"

#define PFB_ATTEN_DB	105.0

typedef struct pfb {
	int		fs, m, d, t, len;	/* SDR rate, bins, M/2, taps per branch, prototype length M t */
	int		tx;
	double		clean;			/* clean band each side of a channel, Hz */
	double		pass, stop;		/* prototype band edges, Hz */
	double		*h;			/* prototype; RX: per branch reversed (hr), TX: scaled by D (g) */
	fftc_t		fft;
	double		*fr, *fi;		/* FFT work */
	long		frame;
	double		*rot_r, *rot_i;		/* RX: time reference of each bin */
	/* RX: input history, linear, refilled to the front */
	double		*hist_i, *hist_q;
	int		pos, fill;
	double		*acc_i, *acc_q;
	/* TX: the last L/D frames of the inverse FFT, stored twice */
	double		*ring_r, *ring_i;
	int		rpos;
} pfb_t;

/* bank for SDR rate fs, m bins (a power of two, >= 8, fs a multiple of m/2),
 * clean band each side of a channel. Returns 0 or <0 with a reason in
 * pfb_error(). */
int pfb_rx_init(pfb_t *pfb, int fs, int m, double clean);
int pfb_tx_init(pfb_t *pfb, int fs, int m, double clean);
const char *pfb_error(void);
void pfb_exit(pfb_t *pfb);
/* bin rate (exact: fs a multiple of m/2) */
int pfb_bin_rate(const pfb_t *pfb);

/* RX: n SDR samples (interleaved float I/Q) -> frames; frame f of bin
 * bins[c] to out_i[c][f], out_q[c][f]. Returns the number of frames (at most
 * pfb_rx_frames(pfb, n)). */
int pfb_rx_process(pfb_t *pfb, const float *iq, int n, int nbins, const int *bins, sample_t **out_i, sample_t **out_q);
int pfb_rx_frames(const pfb_t *pfb, int n);
/* SDR samples needed for f more frames */
int pfb_rx_need(const pfb_t *pfb, int f);

/* TX: frames of nbins channels (channel c's frame f in in_i[c][f] goes into
 * bin bins[c], channels in one bin add up); the SDR samples (D per frame) are
 * ADDED to iq (interleaved float). Returns their number. */
int pfb_tx_process(pfb_t *pfb, int nbins, const int *bins, const sample_t *const *in_i, const sample_t *const *in_q, int frames, float *iq);

/* One channel of a bank */
typedef struct pfb_chan {
	int		tx;
	int		bin;
	double		offset;			/* Hz from the SDR centre */
	int		dsp_rate, bin_rate;
	double		nco_i, nco_q, rot_i, rot_q;
	int		use_rs1;
	resamp_t	rs1, rs2;		/* RX: rs1 fb -> 2 dsp, rs2 2 dsp -> dsp; TX: rs2 dsp -> 2 dsp, rs1 2 dsp -> fb */
	int		use_hb1;		/* RX, fb = 4 dsp: a halfband instead of rs1 (same band edges) */
	halfband_t	hb1;
	int		max_in;			/* RX: most frames per call; TX: most DSP samples per call */
	sample_t	*mid_i, *mid_q;		/* at 2 dsp */
	sample_t	*out_i, *out_q;		/* TX: frames for the bank; RX: DSP samples kept */
	int		kept;			/* RX */
	/* TX pull: surplus kept between calls */
	int		mid_kept;		/* samples at 2 dsp not yet resampled (at the start of mid_i/q) */
	int		fr_len, fr_given;	/* frames in out_i/q, of them given out by the last pull */
} pfb_chan_t;

/* RX: max_frames = most frames per call; TX: max_in = most DSP samples per
 * call. The channel must lie in the band, its DSP rate fit the bank. */
int pfb_chan_rx_init(pfb_chan_t *ch, const pfb_t *pfb, double offset, int dsp_rate, int max_frames);
int pfb_chan_tx_init(pfb_chan_t *ch, const pfb_t *pfb, double offset, int dsp_rate, int max_in);
void pfb_chan_exit(pfb_chan_t *ch);

/* RX: frames of the channel's bin -> at most want DSP samples (returns their
 * number; more are kept for the next call) */
int pfb_chan_rx_process(pfb_chan_t *ch, const sample_t *bin_i, const sample_t *bin_q, int frames, int want, sample_t *out_i, sample_t *out_q);
/* RX: frames needed for want more DSP samples (equal for all channels of
 * the same DSP rate) */
int pfb_chan_rx_need(const pfb_chan_t *ch, int want);
/* TX: n DSP samples -> frames in ch->out_i/out_q (returns their number) */
int pfb_chan_tx_process(pfb_chan_t *ch, const sample_t *in_i, const sample_t *in_q, int n);
/* TX: most frames from n DSP samples */
int pfb_chan_tx_max(const pfb_chan_t *ch, int n);

/* TX, pulled by the bank (channels that started at different times or have
 * different DSP rates): DSP samples needed for exactly 'frames' frames;
 * then pull with that many: ch->out_i/out_q hold exactly 'frames' frames
 * (the surplus is kept for the next pull). Do not mix with
 * pfb_chan_tx_process() on one channel. */
int pfb_chan_tx_need(const pfb_chan_t *ch, int frames);
void pfb_chan_tx_pull(pfb_chan_t *ch, const sample_t *in_i, const sample_t *in_q, int n, int frames);

#endif /* _PFB_H */
