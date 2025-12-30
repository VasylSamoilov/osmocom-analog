/* Polyphase channelizer implementation
 *
 * (C) 2024 Osmocom-analog contributors
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Multi-stage decimation channelizer using halfband filters.
 * Automatically selects optimal filter chain to reach target sample rate
 * and center frequency.
 *
 * Based on the 'DownChannelizer' C++ implementation from SDRangel (f4exb/sdrangel),
 * which is used extensively for Narrow FM, AM, Broadcast FM, and digital modes.
 *
 * This component is designed to efficiently extract a specific channel from a
 * wideband signal by recursively applying halfband decimation filters and
 * selecting the sub-band (lower, center, or upper) that best contains the
 * target frequency.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "channelizer.h"

/*
 * Calculate minimum "fit space" for a channel within a signal band
 * Used to determine which halfband mode gives best channel isolation
 */
static double channel_min_space(double sig_start, double sig_end,
                                double chan_start, double chan_end)
{
	double left_space = chan_start - sig_start;
	double right_space = sig_end - chan_end;
	return (left_space < right_space) ? left_space : right_space;
}

/*
 * Recursively create filter chain to extract channel from signal band
 *
 * This recursive process naturally simplifies multi-mode support:
 * - Narrower bandwidths (e.g., NFM, AM) trigger more decimation stages.
 * - Wider bandwidths (e.g., Broadcast FM) stop after fewer stages.
 *
 * Returns the resulting frequency offset
 *
 * sig_start, sig_end: Current signal band edges
 * chan_start, chan_end: Desired channel band edges
 * stages: Array to store created filter stages
 * num_stages: Pointer to stage count
 */
static double create_filter_chain(channelizer_t *ch,
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
	if (ch->num_stages < CHANNELIZER_MAX_STAGES &&
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
		return create_filter_chain(ch, new_sig_start, new_sig_end,
		                           chan_start, chan_end,
		                           fast_math, filter_order);
	}

	/* No more stages needed - return final offset */
	double chan_center = (chan_end + chan_start) / 2.0;
	double sig_center = (sig_end + sig_start) / 2.0;
	return chan_center - sig_center;
}

/* Initialize channelizer with default filter order */
int channelizer_init(channelizer_t *ch, int input_rate,
                     int output_rate, int center_freq, int fast_math)
{
	return channelizer_init_order(ch, input_rate, output_rate, center_freq,
	                              fast_math, CHANNELIZER_DEFAULT_ORDER);
}

/* Initialize channelizer with specific filter order */
int channelizer_init_order(channelizer_t *ch, int input_rate,
                           int output_rate, int center_freq,
                           int fast_math, int filter_order)
{
	double half_rate;
	double chan_bw;
	double offset;

	memset(ch, 0, sizeof(*ch));

	if (input_rate <= 0 || output_rate <= 0) {
		fprintf(stderr, "Channelizer: invalid sample rates\n");
		return -EINVAL;
	}

	if (output_rate > input_rate) {
		fprintf(stderr, "Channelizer: output rate cannot exceed input rate\n");
		return -EINVAL;
	}

	ch->input_rate = input_rate;
	ch->fast_math = fast_math;
	ch->filter_order = filter_order;
	ch->num_stages = 0;

	/* Calculate channel bandwidth (same as output rate for baseband) */
	chan_bw = (double)output_rate;

	/* Create filter chain */
	half_rate = (double)input_rate / 2.0;
	offset = create_filter_chain(ch,
	                             -half_rate, half_rate,
	                             (double)center_freq - chan_bw / 2.0,
	                             (double)center_freq + chan_bw / 2.0,
	                             fast_math, filter_order);

	/* Calculate actual decimation and output rate */
	ch->decimation = 1 << ch->num_stages;
	ch->output_rate = input_rate / ch->decimation;
	ch->center_offset = (int)round(offset);

	return 0;
}

/* Free channelizer resources */
void channelizer_exit(channelizer_t *ch)
{
	int i;

	for (i = 0; i < ch->num_stages; i++)
		halfband_exit(&ch->stages[i]);

	memset(ch, 0, sizeof(*ch));
}

/* Process block of IQ samples through filter chain */
int channelizer_process(channelizer_t *ch, float *iq_in, int in_count,
                        sample_t *i_out, sample_t *q_out)
{
	int i, s;
	int out_count = 0;
	sample_t si, sq;

	/* Process each input sample */
	for (i = 0; i < in_count; i++) {
		si = iq_in[i * 2];
		sq = iq_in[i * 2 + 1];

		/* Pass through each stage */
		int output = 1;
		for (s = 0; s < ch->num_stages && output; s++) {
			output = halfband_process(&ch->stages[s], &si, &sq);
		}

		/* If all stages produced output, store result */
		if (output) {
			i_out[out_count] = si;
			q_out[out_count] = sq;
			out_count++;
		}
	}

	return out_count;
}
