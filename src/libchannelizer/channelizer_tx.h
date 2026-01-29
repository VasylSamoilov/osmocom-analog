/* TX Polyphase channelizer (synthesis/interpolation)
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
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

#ifndef _CHANNELIZER_TX_H
#define _CHANNELIZER_TX_H

#include "halfband.h"

/* Maximum number of 2x interpolation stages (up to 2^8 = 256x) */
#define CHANNELIZER_TX_MAX_STAGES 8

/* Default filter order */
#define CHANNELIZER_TX_DEFAULT_ORDER HALFBAND_ORDER_48

/* TX Channelizer state */
typedef struct channelizer_tx {
	int input_rate;             /* Channel sample rate (Hz) */
	int output_rate;            /* SDR sample rate (Hz) */
	int interpolation;          /* Total factor (power of 2) */
	int center_offset;          /* Frequency offset (Hz) */
	int num_stages;             /* Number of halfband stages */
	int fast_math;              /* 0=double precision, 1=fixed-point */
	int filter_order;           /* Filter order for all stages */
	halfband_t stages[CHANNELIZER_TX_MAX_STAGES];
} channelizer_tx_t;

/* Initialize TX channelizer
 * sdr_rate: SDR output sample rate (Hz)
 * bandwidth: Required signal bandwidth (Hz)
 * center_freq: Channel center frequency offset from DC (Hz)
 * fast_math: 0 for double precision, 1 for fixed-point
 * Returns 0 on success, <0 on error */
int channelizer_tx_init(channelizer_tx_t *ch, int sdr_rate,
                        double bandwidth, int center_freq, int fast_math);

/* Initialize with specific filter order */
int channelizer_tx_init_order(channelizer_tx_t *ch, int sdr_rate,
                              double bandwidth, int center_freq,
                              int fast_math, int filter_order);

/* Free TX channelizer resources */
void channelizer_tx_exit(channelizer_tx_t *ch);

/* Process samples: interpolate and add to output buffer
 * i_in, q_in: Input I/Q sample arrays at channel rate
 * in_count: Number of input samples
 * iq_out: Interleaved I/Q output (ADDS to existing buffer!)
 * Returns: Number of output samples produced */
int channelizer_tx_process(channelizer_tx_t *ch,
                           sample_t *i_in, sample_t *q_in, int in_count,
                           float *iq_out);

/* Get the calculated channel input rate */
static inline int channelizer_tx_get_input_rate(channelizer_tx_t *ch)
{
	return ch->input_rate;
}

/* Get the interpolation factor */
static inline int channelizer_tx_get_interpolation(channelizer_tx_t *ch)
{
	return ch->interpolation;
}

#endif /* _CHANNELIZER_TX_H */
