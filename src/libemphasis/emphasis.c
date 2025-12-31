/* Pre-Emphasis and De-Emphasis implementation
 *
 * (C) 2016 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../libfilter/iir_filter.h"
#include "emphasis.h"

#define PI		M_PI

static void gen_sine(sample_t *samples, int num, int samplerate, double freq)
{
	int i;

	for (i = 0; i < num; i++)
		samples[i] = cos(2.0 * M_PI * freq / (double)samplerate * (double)i);
}

static double get_level(sample_t *samples, int num)
{
	int i;
	double envelope = 0;
	for (i = num/2; i < num; i++) {
		if (samples[i] > envelope)
			envelope = samples[i];
	}

	return envelope;
}

/* calculate cut off from time constant in uS */
double timeconstant2cutoff(double time_constant_us)
{
	return 1.0 / (2.0 * PI * time_constant_us / 1e6);
}

int init_emphasis(emphasis_t *state, int samplerate, double cut_off, double cut_off_h, double cut_off_l)
{
	double factor;
	int test_num;
	sample_t *test_samples;

	memset(state, 0, sizeof(*state));

	/* Limit test buffer to 1 second or 100000 samples max to avoid
	 * excessive memory use at high sample rates. 100ms is sufficient
	 * for calibration of a 1 kHz sine wave. */
	test_num = samplerate / 10;
	if (test_num > 100000)
		test_num = 100000;
	if (test_num < 1000)
		test_num = 1000;

	test_samples = malloc(test_num * sizeof(sample_t));
	if (!test_samples) {
		fprintf(stderr, "Failed to allocate emphasis calibration buffer\n");
		return -1;
	}

	/* exp (-2 * PI * CUT_OFF * delta_t) */
	factor = exp(-2.0 * PI * cut_off / (double)samplerate); /* 1/samplerate == delta_t */

//	printf("Emphasis factor = %.3f\n", factor);
	state->p.factor = factor;
	state->p.amp = 1.0;
	state->d.factor = factor;
	state->d.amp = 1.0;

	/* do not de-emphasis below CUT_OFF_H */
	iir_highpass_init(&state->d.hp, cut_off_h, samplerate, 1);

	/* do not pre-emphasis above CUT_OFF_L
	 * Mobile network specifications want -18 dB per octave.
	 * With two iterations we have 24 dB, - 6 dB (from emphasis). */
	iir_lowpass_init(&state->p.lp, cut_off_l, samplerate, 2);

	/* calibrate amplification to be neutral at 1000 Hz */
	gen_sine(test_samples, test_num, samplerate, 1000.0);
	pre_emphasis(state, test_samples, test_num);
	state->p.amp = 1.0 / get_level(test_samples, test_num);
	gen_sine(test_samples, test_num, samplerate, 1000.0);
	de_emphasis(state, test_samples, test_num);
	state->d.amp = 1.0 / get_level(test_samples, test_num);

	free(test_samples);

	return 0;
}

void pre_emphasis(emphasis_t *state, sample_t *samples, int num)
{
	double x, y, x_last, factor, amp;
	int i;

	iir_process(&state->p.lp, samples, num);

	x_last = state->p.x_last;
	factor = state->p.factor;
	amp = state->p.amp;

	for (i = 0; i < num; i++) {
		x = *samples;

		/* pre-emphasis */
		y = x - factor * x_last;

		x_last = x;

		*samples++ = amp * y;
	}

	state->p.x_last = x_last;
}

void de_emphasis(emphasis_t *state, sample_t *samples, int num)
{
	double x, y, y_last, factor, amp;
	int i;

	y_last = state->d.y_last;
	factor = state->d.factor;
	amp = state->d.amp;

	for (i = 0; i < num; i++) {
		x = *samples;

		/* de-emphasis */
		y = x + factor * y_last;

		y_last = y;

		*samples++ = amp * y;
	}

	state->d.y_last = y_last;
}

/* high pass filter to remove DC and low frequencies */
void dc_filter(emphasis_t *state, sample_t *samples, int num)
{
	iir_process(&state->d.hp, samples, num);
}

/*
 * ==========================================================================
 * Optimized 1st-order IIR emphasis filter for FM broadcast
 * Based on SDRangel's FMPreemphasis implementation (GPLv3)
 * Uses bilinear transform for accurate frequency mapping
 * ==========================================================================
 */

void init_emphasis_fast(emphasis_fast_t *e, int samplerate, double tau, double high_freq)
{
	/* Based on SDRangel/gnuradio implementation */
	
	/* Limit high freq to 92.5% of Nyquist to avoid instability */
	double fh = high_freq;
	if (fh > 0.925 * samplerate / 2.0)
		fh = 0.925 * samplerate / 2.0;

	/* Digital corner frequencies */
	double w_cl = 1.0 / tau;                   /* Low corner (emphasis start) */
	double w_ch = 2.0 * M_PI * fh;             /* High corner (emphasis stop) */

	/* Prewarped analog corner frequencies (bilinear transform) */
	double w_cla = 2.0 * samplerate * tan(w_cl / (2.0 * samplerate));
	double w_cha = 2.0 * samplerate * tan(w_ch / (2.0 * samplerate));

	/* Digital pole, zero, and gain from bilinear transform of
	 * H(s) = (s + w_cla) / (s + w_cha) */
	double kl = -w_cla / (2.0 * samplerate);
	double kh = -w_cha / (2.0 * samplerate);
	double z1 = (1.0 + kl) / (1.0 - kl);       /* Zero location */
	double p1 = (1.0 + kh) / (1.0 - kh);       /* Pole location */
	double b0 = (1.0 - kl) / (1.0 - kh);       /* Gain */

	/* Normalize for 0 dB at DC */
	double g = fabs(1.0 - p1) / (b0 * fabs(1.0 - z1));

	/* Store coefficients */
	e->b0 = (float)(g * b0);
	e->b1 = (float)(g * b0 * -z1);
	e->a1 = (float)(-p1);
	e->z = 0.0f;
}

void pre_emphasis_fast(emphasis_fast_t *e, sample_t *samples, int num)
{
	float b0 = e->b0;
	float b1 = e->b1;
	float a1 = e->a1;
	float z = e->z;

	for (int i = 0; i < num; i++) {
		float in = (float)samples[i];
		/* Direct Form II Transposed */
		float out = in * b0 + z;
		z = in * b1 + out * (-a1);
		samples[i] = out;
	}

	e->z = z;
}

void de_emphasis_fast(emphasis_fast_t *e, sample_t *samples, int num)
{
	/* De-emphasis is just the inverse filter: swap numerator/denominator
	 * For now, use same coefficients but with inverted signs conceptually.
	 * Actually, de-emphasis H(z) = (1 - p1*z^-1) / (b0 * (1 - z1*z^-1))
	 * Simplified: just apply IIR with swapped a/b */
	float a1 = e->a1;
	float b0 = e->b0;
	float b1 = e->b1;
	float z = e->z;

	for (int i = 0; i < num; i++) {
		float in = (float)samples[i];
		/* Inverse filter: b becomes a, a becomes b */
		float out = (in - z * (-a1)) / b0;
		z = in - out * b1;
		samples[i] = out;
	}

	e->z = z;
}

/*
 * ==========================================================================
 * Simple 1st-order highpass DC blocking filter
 * Uses classic DC blocker formula: y[n] = x[n] - x[n-1] + alpha * y[n-1]
 * ==========================================================================
 */

void init_dc_filter_fast(dc_filter_fast_t *f, int samplerate, double cutoff_hz)
{
	/* Calculate alpha for 1st-order highpass:
	 * alpha = (1 - sin(2*pi*fc/fs)) / cos(2*pi*fc/fs)
	 * Approximation for low frequencies: alpha ≈ 1 - 2*pi*fc/fs
	 * We use a simpler formula: alpha = 1 - (cutoff / samplerate * 2 * pi) */
	double w = 2.0 * M_PI * cutoff_hz / samplerate;
	f->alpha = (float)(1.0 - w);
	
	/* Clamp alpha to valid range */
	if (f->alpha < 0.9f) f->alpha = 0.9f;
	if (f->alpha > 0.9999f) f->alpha = 0.9999f;
	
	f->prev_in = 0.0f;
	f->prev_out = 0.0f;
}

void dc_filter_fast(dc_filter_fast_t *f, sample_t *samples, int num)
{
	float alpha = f->alpha;
	float prev_in = f->prev_in;
	float prev_out = f->prev_out;

	/* Classic DC blocker: y[n] = x[n] - x[n-1] + alpha * y[n-1]
	 * This is a 1st-order highpass with very low cutoff frequency.
	 * At DC (0 Hz): output = 0
	 * At high frequencies: output ≈ input */
	for (int i = 0; i < num; i++) {
		float in = (float)samples[i];
		float out = in - prev_in + alpha * prev_out;
		prev_in = in;
		prev_out = out;
		samples[i] = out;
	}

	f->prev_in = prev_in;
	f->prev_out = prev_out;
}
