/* Rational polyphase resampler for exact sample rate conversion
 *
 * Converts between rates using rational P/Q ratio found by GCD reduction.
 * Uses standard polyphase FIR decomposition with Kaiser-windowed sinc filter.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef POLYPHASE_H
#define POLYPHASE_H

#include "../libsample/sample.h"

typedef struct polyphase_state {
	double		input_rate;
	double		output_rate;

	/* Rational ratio: output_rate/input_rate = up/down */
	int		up;		/* P: upsample factor */
	int		down;		/* Q: downsample factor */

	/* Filter */
	int		taps_per_phase;	/* taps per polyphase branch (design param) */
	int		branch_len;	/* actual taps per branch after padding */
	int		total_taps;	/* total filter length = branch_len * up */
	float		*branches;	/* polyphase branches [up * branch_len] */

	/* Delay line */
	sample_t	*samples;	/* I delay line [branch_len] */
	sample_t	*samples_q;	/* Q delay line [branch_len] (IQ mode) */
	int		ptr;		/* circular buffer write pointer */

	/* State tracking across calls */
	long long	out_count;	/* total output samples produced */
	long long	in_count;	/* total input samples consumed */

	/* Legacy fields for API compatibility with sdr.c */
	double		distance;	/* input_rate / output_rate */
	double		gain;		/* always 1.0 (gain baked into filter) */
	int		phase_steps;	/* = up */
	int		num_taps;	/* = taps_per_phase */
	float		*taps;		/* unused, kept for free() compat */
	float		*aligned_taps;	/* unused */
	sample_t	last_sample;	/* unused */
} polyphase_t;

/* Initialize polyphase resampler (legacy API, uses 16 taps/phase) */
int polyphase_init(polyphase_t *state, double input_rate, double output_rate,
                   double cutoff, int phase_steps);

/* Initialize with explicit tap count per phase */
int polyphase_init_taps(polyphase_t *state, double input_rate, double output_rate,
                        double cutoff, int phase_steps, int num_taps);

/* Resample mono audio samples */
int polyphase_resample(polyphase_t *state, const sample_t *input, int input_num,
                       sample_t *output, int output_max);

/* Calculate expected output count for given input count */
int polyphase_output_num(polyphase_t *state, int input_num);

/* Calculate required input count for desired output count */
int polyphase_input_num(polyphase_t *state, int output_num);

/* Resample IQ (complex) samples.
 * I and Q are processed with identical timing. */
int polyphase_resample_iq(polyphase_t *state, const sample_t *input_i,
                          const sample_t *input_q, int input_num,
                          sample_t *output_i, sample_t *output_q,
                          int output_max);

/* Free resampler resources */
void polyphase_free(polyphase_t *state);

#endif /* POLYPHASE_H */
