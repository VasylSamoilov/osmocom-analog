/* Polyphase resampler for arbitrary sample rate conversion
 *
 * Based on SDRangel's polyphase interpolator design by Christian Daniel
 * and Edouard Griffiths (F4EXB). Adapted for osmocom-analog.
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
#include <math.h>
#include "polyphase.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Create polyphase lowpass filter coefficients */
static void create_polyphase_filter(float *taps, int num_taps, int phase_steps,
                                    double sample_rate, double cutoff)
{
	int total_taps = num_taps * phase_steps;
	double *window;
	double *raw_taps;
	int i, phase, n, M;
	double fwT0, max, sum;

	/* Allocate temporary buffers */
	window = calloc(total_taps, sizeof(double));
	raw_taps = calloc(total_taps, sizeof(double));
	if (!window || !raw_taps) {
		free(window);
		free(raw_taps);
		return;
	}

	/* Hamming window */
	for (n = 0; n < total_taps; n++) {
		window[n] = 0.54 - 0.46 * cos(2.0 * M_PI * n / (total_taps - 1));
	}

	/* Sinc lowpass filter */
	M = (total_taps - 1) / 2;
	fwT0 = 2.0 * M_PI * cutoff / sample_rate;

	for (n = -M; n <= M; n++) {
		if (n == 0) {
			raw_taps[n + M] = fwT0 / M_PI * window[n + M];
		} else {
			raw_taps[n + M] = sin(n * fwT0) / (n * M_PI) * window[n + M];
		}
	}

	/* Normalize */
	max = raw_taps[M];
	for (n = 1; n <= M; n++) {
		max += 2.0 * raw_taps[n + M];
	}
	for (i = 0; i < total_taps; i++) {
		raw_taps[i] /= max;
	}

	/* Reorder into polyphase decomposition */
	for (phase = 0; phase < phase_steps; phase++) {
		for (i = 0; i < num_taps; i++) {
			taps[phase * num_taps + i] = (float)raw_taps[i * phase_steps + phase];
		}
	}

	/* Normalize each phase filter */
	for (phase = 0; phase < phase_steps; phase++) {
		sum = 0.0;
		for (i = 0; i < num_taps; i++) {
			sum += taps[phase * num_taps + i];
		}
		if (sum != 0.0) {
			for (i = 0; i < num_taps; i++) {
				taps[phase * num_taps + i] /= (float)sum;
			}
		}
	}

	free(window);
	free(raw_taps);
}

/* Apply polyphase filter at given phase */
static sample_t apply_filter(polyphase_t *state, int phase)
{
	int i, idx;
	float *coeff;
	sample_t acc = 0.0;

	if (phase < 0) phase = 0;
	if (phase >= state->phase_steps) phase = state->phase_steps - 1;

	coeff = &state->taps[phase * state->num_taps];
	idx = state->ptr;

	for (i = 0; i < state->num_taps; i++) {
		acc += coeff[i] * state->samples[idx];
		idx = (idx + 1) % state->num_taps;
	}

	return acc;
}

/* Advance delay line with new sample */
static void advance_filter(polyphase_t *state, sample_t sample)
{
	state->ptr--;
	if (state->ptr < 0) {
		state->ptr = state->num_taps - 1;
	}
	state->samples[state->ptr] = sample;
}

int polyphase_init(polyphase_t *state, double input_rate, double output_rate,
                   double cutoff, int phase_steps)
{
	int num_taps;
	double filter_rate;

	memset(state, 0, sizeof(*state));

	state->input_rate = input_rate;
	state->output_rate = output_rate;
	state->distance = input_rate / output_rate;
	state->distance_remain = 0.0;
	state->phase_steps = phase_steps;

	/* Number of taps per phase - higher for better stopband */
	num_taps = 8;  /* 8 taps * 16 phases = 128 total taps */
	state->num_taps = num_taps;

	/* Use higher of the two rates for filter design */
	filter_rate = (input_rate > output_rate) ? input_rate : output_rate;
	filter_rate *= phase_steps;

	/* Allocate filter taps */
	state->taps = calloc(num_taps * phase_steps, sizeof(float));
	if (!state->taps) {
		return -1;
	}

	/* Allocate delay line */
	state->samples = calloc(num_taps, sizeof(sample_t));
	if (!state->samples) {
		free(state->taps);
		state->taps = NULL;
		return -1;
	}

	state->ptr = 0;
	state->last_sample = 0.0;

	/* Create polyphase filter */
	create_polyphase_filter(state->taps, num_taps, phase_steps, 
	                        filter_rate, cutoff);

	return 0;
}

int polyphase_output_num(polyphase_t *state, int input_num)
{
	/* Estimate output samples for given input */
	return (int)((double)input_num / state->distance + 0.5) + 1;
}

int polyphase_input_num(polyphase_t *state, int output_num)
{
	/* Estimate input samples needed for desired output */
	return (int)((double)output_num * state->distance + 0.5) + 1;
}

int polyphase_resample(polyphase_t *state, const sample_t *input, int input_num,
                       sample_t *output, int output_max)
{
	int in_idx = 0;
	int out_idx = 0;
	int phase;

	if (state->distance < 1.0) {
		/* Interpolation: output_rate > input_rate (produce more samples) */
		while (out_idx < output_max) {
			/* Consume input when distance exceeds 1.0 */
			if (state->distance_remain >= 1.0) {
				if (in_idx >= input_num)
					break;
				advance_filter(state, input[in_idx++]);
				state->distance_remain -= 1.0;
			}

			/* Produce output sample */
			phase = (int)(state->distance_remain * state->phase_steps);
			output[out_idx++] = apply_filter(state, phase);
			state->distance_remain += state->distance;
		}
	} else {
		/* Decimation: output_rate < input_rate (produce fewer samples) */
		while (in_idx < input_num && out_idx < output_max) {
			/* Advance filter with input */
			advance_filter(state, input[in_idx++]);
			state->distance_remain -= 1.0;

			/* Produce output when distance goes below 1.0 */
			if (state->distance_remain < 1.0) {
				phase = (int)(state->distance_remain * state->phase_steps);
				if (phase < 0) phase = 0;
				output[out_idx++] = apply_filter(state, phase);
				state->distance_remain += state->distance;
			}
		}
	}





	return out_idx;
}

void polyphase_free(polyphase_t *state)
{
	if (state->taps) {
		free(state->taps);
		state->taps = NULL;
	}
	if (state->samples) {
		free(state->samples);
		state->samples = NULL;
	}
	state->aligned_taps = NULL;
}
