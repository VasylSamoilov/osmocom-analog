/* Polyphase channelizer
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Filter chain management for multi-stage decimation channelizer.
 * Reference: SDRangel implementation was studied to understand the algorithm.
 */

#ifndef _CHANNELIZER_H
#define _CHANNELIZER_H

#include "halfband.h"

/* Maximum number of 2x decimation stages (up to 2^8 = 256x decimation) */
#define CHANNELIZER_MAX_STAGES 8

/* Default filter order */
#define CHANNELIZER_DEFAULT_ORDER HALFBAND_ORDER_32

/* Channelizer state */
typedef struct channelizer {
	int input_rate;             /* Input sample rate (Hz) */
	int output_rate;            /* Actual output sample rate after decimation */
	int decimation;             /* Total decimation factor (power of 2) */
	int center_offset;          /* Resulting center frequency offset (Hz) */
	int num_stages;             /* Number of halfband stages */
	int fast_math;              /* 0=double precision, 1=fixed-point */
	int filter_order;           /* Filter order for all stages */
	halfband_t stages[CHANNELIZER_MAX_STAGES];
} channelizer_t;

/* Initialize channelizer
 * input_rate: Input sample rate in Hz
 * output_rate: Requested output sample rate (actual may be higher)
 * center_freq: Requested channel center frequency offset from DC
 * fast_math: 0 for double precision, 1 for fixed-point
 * Returns 0 on success, <0 on error */
int channelizer_init(channelizer_t *ch, int input_rate,
                     int output_rate, int center_freq, int fast_math);

/* Initialize with specific filter order */
int channelizer_init_order(channelizer_t *ch, int input_rate,
                           int output_rate, int center_freq,
                           int fast_math, int filter_order);

/* Free channelizer resources */
void channelizer_exit(channelizer_t *ch);

/* Process block of interleaved IQ samples
 * iq_in: Interleaved float I,Q,I,Q... input array
 * in_count: Number of IQ sample pairs
 * i_out, q_out: Output sample arrays (must be sized for in_count/decimation)
 * Returns: Number of output samples produced */
int channelizer_process(channelizer_t *ch, float *iq_in, int in_count,
                        sample_t *i_out, sample_t *q_out);

/* Get actual output rate (may be higher than requested) */
static inline int channelizer_get_output_rate(channelizer_t *ch)
{
	return ch->output_rate;
}

/* Get actual center frequency offset */
static inline int channelizer_get_center_offset(channelizer_t *ch)
{
	return ch->center_offset;
}

/* Get decimation factor */
static inline int channelizer_get_decimation(channelizer_t *ch)
{
	return ch->decimation;
}

#endif /* _CHANNELIZER_H */
