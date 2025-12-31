/* TX Polyphase channelizer implementation
 *
 * (C) 2024 Osmocom-analog contributors
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Multi-stage interpolation channelizer using halfband filters.
 * Mirrors the RX channelizer (decimation) for TX path.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "channelizer_tx.h"
#include "../libsdr/sdr.h"

/*
 * Calculate minimum "fit space" for a channel within a signal band
 */
static double channel_min_space(double sig_start, double sig_end,
                                double chan_start, double chan_end)
{
	double left_space = chan_start - sig_start;
	double right_space = sig_end - chan_end;
	return (left_space < right_space) ? left_space : right_space;
}

/*
 * Create filter chain for TX (interpolation)
 * Mirrors RX filter chain creation but in reverse order
 * Note: For TX, stages are built in reverse order (first stage = closest to output)
 */
static double create_tx_filter_chain(channelizer_tx_t *ch,
                                     double sig_start, double sig_end,
                                     double chan_start, double chan_end,
                                     int fast_math, int filter_order)
{
	double sig_bw = sig_end - sig_start;
	double chan_bw = chan_end - chan_start;
	double rot = sig_bw / 4.0;

	/* Calculate fit space for each mode */
	double space_lower = channel_min_space(sig_start, sig_start + sig_bw / 2.0,
	                                       chan_start, chan_end);
	double space_center = channel_min_space(sig_start + rot, sig_end - rot,
	                                        chan_start, chan_end);
	double space_upper = channel_min_space(sig_end - sig_bw / 2.0, sig_end,
	                                       chan_start, chan_end);

	/* Find best mode */
	int best_mode;
	double best_space;

	if (space_lower >= space_center && space_lower >= space_upper) {
		best_mode = 0;  /* Lower */
		best_space = space_lower;
	} else if (space_center >= space_upper) {
		best_mode = 1;  /* Center */
		best_space = space_center;
	} else {
		best_mode = 2;  /* Upper */
		best_space = space_upper;
	}

	/* Check if we should add another stage */
	if (ch->num_stages < CHANNELIZER_TX_MAX_STAGES &&
	    sig_start < sig_end && chan_start < chan_end &&
	    best_space >= chan_bw / 8.0) {

		halfband_mode_t mode;
		double new_sig_start, new_sig_end;

		switch (best_mode) {
		case 0:  /* Lower half */
			mode = HALFBAND_LOWER;
			new_sig_start = sig_start;
			new_sig_end = sig_start + sig_bw / 2.0;
			break;
		case 1:  /* Center */
			mode = HALFBAND_CENTER;
			new_sig_start = sig_start + rot;
			new_sig_end = sig_end - rot;
			break;
		case 2:  /* Upper half */
		default:
			mode = HALFBAND_UPPER;
			new_sig_start = sig_end - sig_bw / 2.0;
			new_sig_end = sig_end;
			break;
		}

		/* Initialize this stage */
		int rc = halfband_init(&ch->stages[ch->num_stages], filter_order,
		                       mode, fast_math);
		if (rc < 0)
			return 0;  /* Error */

		ch->num_stages++;

		/* Recurse with narrower band */
		return create_tx_filter_chain(ch, new_sig_start, new_sig_end,
		                              chan_start, chan_end,
		                              fast_math, filter_order);
	}

	/* No more stages needed - return final offset */
	double chan_center = (chan_end + chan_start) / 2.0;
	double sig_center = (sig_end + sig_start) / 2.0;
	return chan_center - sig_center;
}

/* Initialize TX channelizer with auto-calculated input rate */
int channelizer_tx_init(channelizer_tx_t *ch, int sdr_rate,
                        double bandwidth, int center_freq, int fast_math)
{
	return channelizer_tx_init_order(ch, sdr_rate, bandwidth, center_freq,
	                                 fast_math, CHANNELIZER_TX_DEFAULT_ORDER);
}

/* Initialize TX channelizer with specific filter order */
int channelizer_tx_init_order(channelizer_tx_t *ch, int sdr_rate,
                              double bandwidth, int center_freq,
                              int fast_math, int filter_order)
{
	double half_rate;
	double chan_bw;
	double offset;

	memset(ch, 0, sizeof(*ch));

	if (sdr_rate <= 0 || bandwidth <= 0) {
		fprintf(stderr, "Channelizer TX: invalid parameters\n");
		return -EINVAL;
	}

	ch->output_rate = sdr_rate;
	ch->fast_math = fast_math;
	ch->filter_order = filter_order;
	ch->num_stages = 0;

	/* Auto-calculate optimal input rate */
	ch->input_rate = sdr_calculate_optimal_rate(sdr_rate, bandwidth);

	/* Calculate channel bandwidth */
	chan_bw = bandwidth;

	/* Create filter chain */
	half_rate = (double)sdr_rate / 2.0;
	offset = create_tx_filter_chain(ch,
	                                -half_rate, half_rate,
	                                (double)center_freq - chan_bw / 2.0,
	                                (double)center_freq + chan_bw / 2.0,
	                                fast_math, filter_order);

	/* Calculate actual interpolation */
	ch->interpolation = 1 << ch->num_stages;
	ch->input_rate = sdr_rate / ch->interpolation;
	ch->center_offset = (int)round(offset);

	return 0;
}

/* Free TX channelizer resources */
void channelizer_tx_exit(channelizer_tx_t *ch)
{
	int i;

	for (i = 0; i < ch->num_stages; i++)
		halfband_exit(&ch->stages[i]);

	memset(ch, 0, sizeof(*ch));
}

/* Process block of samples: interpolate and ADD to output buffer */
int channelizer_tx_process(channelizer_tx_t *ch,
                           sample_t *i_in, sample_t *q_in, int in_count,
                           float *iq_out)
{
	int i, s;
	int out_count = 0;
	
	if (ch->num_stages == 0) {
		/* No interpolation needed - just copy */
		for (i = 0; i < in_count; i++) {
			iq_out[i * 2] += i_in[i];
			iq_out[i * 2 + 1] += q_in[i];
		}
		return in_count;
	}

	/* Process each input sample */
	for (i = 0; i < in_count; i++) {
		sample_t si = i_in[i];
		sample_t sq = q_in[i];
		
		/* Buffer for intermediate samples during upsampling */
		sample_t buf_i[256], buf_q[256];  /* Max 2^8 = 256 outputs per input */
		int buf_count = 1;
		buf_i[0] = si;
		buf_q[0] = sq;
		
		/* Process through stages in REVERSE order (innermost to outermost)
		 * TX stages were built the same as RX, but we apply them in reverse
		 * and use interpolation instead of decimation */
		for (s = ch->num_stages - 1; s >= 0; s--) {
			int new_count = 0;
			int j;
			
			for (j = 0; j < buf_count; j++) {
				sample_t out_i, out_q;
				
				/* Each input produces 2 outputs */
				halfband_interpolate(&ch->stages[s], buf_i[j], buf_q[j],
				                     &out_i, &out_q);
				buf_i[new_count] = out_i;
				buf_q[new_count] = out_q;
				new_count++;
				
				/* Second output (same input, different phase) */
				halfband_interpolate(&ch->stages[s], buf_i[j], buf_q[j],
				                     &out_i, &out_q);
				buf_i[new_count] = out_i;
				buf_q[new_count] = out_q;
				new_count++;
			}
			buf_count = new_count;
		}
		
		/* Add interpolated samples to output buffer */
		for (s = 0; s < buf_count; s++) {
			iq_out[out_count * 2] += buf_i[s];
			iq_out[out_count * 2 + 1] += buf_q[s];
			out_count++;
		}
	}

	return out_count;
}
