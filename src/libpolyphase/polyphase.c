/* Rational polyphase resampler for exact sample rate conversion
 *
 * Converts between rates using rational P/Q ratio:
 *   output_rate / input_rate = P / Q
 * where P and Q are found by GCD reduction.
 *
 * This is mathematically exact (no drift, no approximation error)
 * and uses the standard polyphase decomposition:
 *   - Design FIR lowpass of length taps_per_phase * P
 *   - Decompose into P polyphase branches
 *   - For each output sample, select branch and compute dot product
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

/* Compute GCD of two positive integers */
static int compute_gcd(int a, int b)
{
	while (b != 0) {
		int t = b;
		b = a % b;
		a = t;
	}
	return a;
}

/* Modified Bessel function I0 for Kaiser window */
static double bessel_i0(double x)
{
	double sum = 1.0;
	double term = 1.0;
	double x_half = x / 2.0;
	int k;

	for (k = 1; k < 30; k++) {
		term *= (x_half / k) * (x_half / k);
		sum += term;
		if (term < sum * 1e-15)
			break;
	}
	return sum;
}

/* Design and decompose the polyphase filter.
 *
 * The filter is designed at the upsampled rate (input_rate * P) with
 * cutoff at min(pi/P, pi/Q) to prevent aliasing in both directions.
 * It is then decomposed into P polyphase branches.
 *
 * The filter gain is scaled by P to compensate for the zero-stuffing
 * in the conceptual upsample-by-P step.
 */
static int design_filter(polyphase_t *state)
{
	int total_taps, pad;
	int max_factor;
	double cutoff_norm, beta;
	double *raw_taps = NULL;
	double *window = NULL;
	double sum;
	int n, M, p, k, idx;

	max_factor = (state->up > state->down) ? state->up : state->down;

	/* Total filter length: taps_per_phase * max(P, Q)
	 * This ensures enough taps for the harder direction */
	total_taps = state->taps_per_phase * max_factor;
	/* Make odd for symmetric filter */
	if (total_taps % 2 == 0)
		total_taps++;

	/* Pad to multiple of P for clean polyphase decomposition */
	pad = (state->up - (total_taps % state->up)) % state->up;
	total_taps += pad;

	state->branch_len = total_taps / state->up;
	state->total_taps = total_taps;

	/* Allocate temporary buffers */
	raw_taps = calloc(total_taps, sizeof(double));
	window = calloc(total_taps, sizeof(double));
	if (!raw_taps || !window)
		goto fail;

	/* Kaiser window with beta=7.0 for ~70dB stopband rejection */
	beta = 7.0;
	{
		double denom = bessel_i0(beta);
		for (n = 0; n < total_taps; n++) {
			double x = 2.0 * n / (total_taps - 1) - 1.0;
			window[n] = bessel_i0(beta * sqrt(1.0 - x * x)) / denom;
		}
	}

	/* Sinc lowpass filter.
	 * Cutoff at 0.5/max(P,Q) of the upsampled sampling rate.
	 * This equals the Nyquist frequency of the slower rate.
	 * (scipy.firwin uses cutoff normalized to Nyquist, so firwin(N, 1/P)
	 *  corresponds to cutoff_norm = 0.5/P in our convention.) */
	cutoff_norm = 0.5 / max_factor;
	M = (total_taps - 1) / 2;

	for (n = 0; n < total_taps; n++) {
		double t = n - M;
		if (fabs(t) < 1e-10) {
			raw_taps[n] = 2.0 * cutoff_norm * window[n];
		} else {
			raw_taps[n] = sin(2.0 * M_PI * cutoff_norm * t) / (M_PI * t) * window[n];
		}
	}

	/* Normalize total filter to unity, then scale by P for interpolation gain */
	sum = 0.0;
	for (n = 0; n < total_taps; n++)
		sum += raw_taps[n];
	if (fabs(sum) > 1e-15) {
		double scale = (double)state->up / sum;
		for (n = 0; n < total_taps; n++)
			raw_taps[n] *= scale;
	}

	/* Allocate polyphase branch storage: P branches of branch_len taps each */
	state->branches = calloc(state->up * state->branch_len, sizeof(float));
	if (!state->branches)
		goto fail;

	/* Polyphase decomposition:
	 * Branch p gets taps: h[p], h[p+P], h[p+2P], ... */
	for (p = 0; p < state->up; p++) {
		for (k = 0; k < state->branch_len; k++) {
			idx = p + k * state->up;
			if (idx < total_taps)
				state->branches[p * state->branch_len + k] = (float)raw_taps[idx];
		}
	}

	free(raw_taps);
	free(window);
	return 0;

fail:
	free(raw_taps);
	free(window);
	return -1;
}

int polyphase_init(polyphase_t *state, double input_rate, double output_rate,
                   double cutoff __attribute__((unused)), int phase_steps __attribute__((unused)))
{
	return polyphase_init_taps(state, input_rate, output_rate, cutoff,
	                           phase_steps, 16);
}

int polyphase_init_taps(polyphase_t *state, double input_rate, double output_rate,
                        double cutoff __attribute__((unused)),
                        int phase_steps __attribute__((unused)),
                        int taps_per_phase)
{
	int ir, otr, g;

	memset(state, 0, sizeof(*state));

	state->input_rate = input_rate;
	state->output_rate = output_rate;

	/* Find rational P/Q ratio via GCD */
	ir = (int)round(input_rate);
	otr = (int)round(output_rate);
	g = compute_gcd(ir, otr);
	state->down = ir / g;   /* Q: downsample factor */
	state->up = otr / g;    /* P: upsample factor */
	state->taps_per_phase = taps_per_phase;

	/* For the old API compatibility */
	state->distance = input_rate / output_rate;
	state->phase_steps = state->up;
	state->num_taps = taps_per_phase;
	state->gain = 1.0;

	/* Design and decompose the filter */
	if (design_filter(state) < 0)
		return -1;

	/* Allocate delay line for I channel */
	state->samples = calloc(state->branch_len, sizeof(sample_t));
	if (!state->samples)
		return -1;

	state->ptr = 0;
	state->out_count = 0;

	/* Log the resampler configuration (uses stderr since we don't have
	 * access to the logging framework from this library) */
	/* fprintf(stderr, "Polyphase resampler: %d Hz -> %d Hz (P=%d, Q=%d, "
	        "branch_len=%d, total_taps=%d)\n",
	        ir, otr, state->up, state->down,
	        state->branch_len, state->total_taps); */

	return 0;
}

int polyphase_output_num(polyphase_t *state, int input_num)
{
	/* Exact: input_num * P / Q, rounded up + margin */
	return (int)((long long)input_num * state->up / state->down) + 2;
}

int polyphase_input_num(polyphase_t *state, int output_num)
{
	/* Exact: output_num * Q / P, rounded up + margin */
	return (int)((long long)output_num * state->down / state->up) + 2;
}

/* Resample mono audio using rational polyphase.
 *
 * For absolute output sample n:
 *   pos = n * Q          (position in upsampled stream)
 *   phase = pos % P      (which polyphase branch)
 *   base = pos / P       (which input sample is the most recent needed)
 *   output[n] = dot(branch[phase], delay_line[base-k])
 *
 * State tracks absolute output/input counts across calls for streaming.
 */
int polyphase_resample(polyphase_t *state, const sample_t *input, int input_num,
                       sample_t *output, int output_max)
{
	int out_idx = 0;
	int in_consumed = 0;
	long long pos;
	int phase;
	long long base_idx;
	int k;
	float *branch;
	sample_t acc;
	int idx;

	/* Prevent counter overflow: subtract full rational cycles periodically.
	 * out_count * Q and in_count track the same position in the upsampled
	 * domain. Subtracting (out -= N*P, in -= N*Q) shifts both by exactly
	 * N complete P/Q cycles, preserving phase and delay line state. */
	if (state->out_count > 1000000000LL) {
		long long cycles = state->out_count / state->up;
		state->out_count -= cycles * state->up;
		state->in_count -= cycles * state->down;
	}

	while (out_idx < output_max) {
		pos = state->out_count * state->down;
		phase = (int)(pos % state->up);
		base_idx = pos / state->up;

		/* Advance delay line until we have input up to base_idx */
		while (state->in_count <= base_idx) {
			if (in_consumed >= input_num)
				return out_idx;  /* Need more input */
			state->samples[state->ptr] = input[in_consumed++];
			state->ptr = (state->ptr + 1) % state->branch_len;
			state->in_count++;
		}

		/* Compute output: dot product of branch[phase] with delay line.
		 * ptr points to the NEXT write position, so ptr-1 is the most recent sample. */
		branch = &state->branches[phase * state->branch_len];
		acc = 0.0;
		idx = (state->ptr - 1 + state->branch_len) % state->branch_len;
		for (k = 0; k < state->branch_len; k++) {
			acc += branch[k] * state->samples[idx];
			idx = (idx - 1 + state->branch_len) % state->branch_len;
		}

		output[out_idx++] = acc;
		state->out_count++;
	}

	return out_idx;
}

/* Resample IQ (complex) audio using rational polyphase.
 * Both I and Q share the same timing/phase selection. */
int polyphase_resample_iq(polyphase_t *state, const sample_t *input_i,
                          const sample_t *input_q, int input_num,
                          sample_t *output_i, sample_t *output_q,
                          int output_max)
{
	int out_idx = 0;
	int in_consumed = 0;
	long long pos;
	int phase;
	long long base_idx;
	int k;
	float *branch;
	sample_t acc_i, acc_q;
	int idx;

	/* Allocate Q delay line on first use */
	if (!state->samples_q) {
		state->samples_q = calloc(state->branch_len, sizeof(sample_t));
		if (!state->samples_q)
			return -1;
	}

	/* Prevent counter overflow — see polyphase_resample() for explanation */
	if (state->out_count > 1000000000LL) {
		long long cycles = state->out_count / state->up;
		state->out_count -= cycles * state->up;
		state->in_count -= cycles * state->down;
	}

	while (out_idx < output_max) {
		pos = state->out_count * state->down;
		phase = (int)(pos % state->up);
		base_idx = pos / state->up;

		/* Advance delay line */
		while (state->in_count <= base_idx) {
			if (in_consumed >= input_num)
				return out_idx;
			state->samples[state->ptr] = input_i[in_consumed];
			state->samples_q[state->ptr] = input_q[in_consumed];
			in_consumed++;
			state->ptr = (state->ptr + 1) % state->branch_len;
			state->in_count++;
		}

		/* Compute output */
		branch = &state->branches[phase * state->branch_len];
		acc_i = 0.0;
		acc_q = 0.0;
		idx = (state->ptr - 1 + state->branch_len) % state->branch_len;
		for (k = 0; k < state->branch_len; k++) {
			acc_i += branch[k] * state->samples[idx];
			acc_q += branch[k] * state->samples_q[idx];
			idx = (idx - 1 + state->branch_len) % state->branch_len;
		}

		output_i[out_idx] = acc_i;
		output_q[out_idx] = acc_q;
		out_idx++;
		state->out_count++;
	}

	return out_idx;
}

void polyphase_free(polyphase_t *state)
{
	if (state->branches) {
		free(state->branches);
		state->branches = NULL;
	}
	if (state->taps) {
		free(state->taps);
		state->taps = NULL;
	}
	if (state->samples) {
		free(state->samples);
		state->samples = NULL;
	}
	if (state->samples_q) {
		free(state->samples_q);
		state->samples_q = NULL;
	}
}
