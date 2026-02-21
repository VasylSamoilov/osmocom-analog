/* Phase-Locked Loop Implementation
 *
 * Type-2, 4th order PLL for carrier/pilot tracking.
 * Based on SDRangel's phaselock.cpp by Edouard Griffiths, F4EXB.
 *
 * Open-loop transfer function:
 *   G(z) = K * (z - q1) / ((z - p1) * (z - p2) * (z - 1) * (z - 1))
 *   K  = 3.788 * (bandwidth * 2 * Pi)^3
 *   q1 = exp(-0.1153 * bandwidth * 2*Pi)
 *   p1 = exp(-1.146 * bandwidth * 2*Pi)
 *   p2 = exp(-5.331 * bandwidth * 2*Pi)
 *
 * (C) 2024 - Adapted for osmocom-analog
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <math.h>
#include <string.h>
#include "pll.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void pll_init(pll_t *pll, double freq, double bandwidth, double min_signal)
{
	memset(pll, 0, sizeof(*pll));
	pll_configure(pll, freq, bandwidth, min_signal);
}

void pll_configure(pll_t *pll, double freq, double bandwidth, double min_signal)
{
	pll->center_freq = freq;
	pll->bandwidth = bandwidth;
	pll->min_signal = min_signal;
	
	/* Set min/max locking frequencies (allow +/- bandwidth around center) */
	pll->min_freq = (freq - bandwidth) * 2.0 * M_PI;
	pll->max_freq = (freq + bandwidth) * 2.0 * M_PI;
	
	/* Lock detection delay: ~20 cycles at loop bandwidth */
	pll->lock_delay = (int)(20.0 / bandwidth);
	if (pll->lock_delay < 100)
		pll->lock_delay = 100;
	pll->lock_cnt = 0;
	pll->pilot_level = 0;
	
	/* Create 2nd order filter for I/Q phasor smoothing.
	 * Two poles at p1 and p2, unit DC gain. */
	double p1 = exp(-1.146 * bandwidth * 2.0 * M_PI);
	double p2 = exp(-5.331 * bandwidth * 2.0 * M_PI);
	pll->phasor_a1 = -(p1 + p2);
	pll->phasor_a2 = p1 * p2;
	pll->phasor_b0 = 1.0 + pll->phasor_a1 + pll->phasor_a2;
	
	/* Create loop filter to stabilize the loop.
	 * This is a 1st order filter with zero at q1. */
	double q1 = exp(-0.1153 * bandwidth * 2.0 * M_PI);
	pll->loopfilter_b0 = 0.62 * bandwidth * 2.0 * M_PI;
	pll->loopfilter_b1 = -pll->loopfilter_b0 * q1;
	
	/* Initialize frequency to center */
	pll->freq = freq * 2.0 * M_PI;
	pll->phase = 0;
	pll->psin = 0;
	pll->pcos = 1.0;
	
	/* Reset filter states */
	pll->phasor_i1 = pll->phasor_i2 = 0;
	pll->phasor_q1 = pll->phasor_q2 = 0;
	pll->loopfilter_x1 = 0;
	
	pll->sample_cnt = 0;
	pll->pilot_periods = 0;
}

void pll_reset(pll_t *pll)
{
	pll->freq = pll->center_freq * 2.0 * M_PI;
	pll->phase = 0;
	pll->psin = 0;
	pll->pcos = 1.0;
	pll->lock_cnt = 0;
	pll->pilot_level = 0;
	
	pll->phasor_i1 = pll->phasor_i2 = 0;
	pll->phasor_q1 = pll->phasor_q2 = 0;
	pll->loopfilter_x1 = 0;
	
	pll->sample_cnt = 0;
	pll->pilot_periods = 0;
}

/* Internal: process phasor through filters and update frequency/phase */
static void pll_process_phasor(pll_t *pll, double phasor_i, double phasor_q)
{
	/* Run I/Q phase error through 2nd order low-pass filter */
	double filtered_i = pll->phasor_b0 * phasor_i
	                  - pll->phasor_a1 * pll->phasor_i1
	                  - pll->phasor_a2 * pll->phasor_i2;
	double filtered_q = pll->phasor_b0 * phasor_q
	                  - pll->phasor_a1 * pll->phasor_q1
	                  - pll->phasor_a2 * pll->phasor_q2;
	
	pll->phasor_i2 = pll->phasor_i1;
	pll->phasor_i1 = filtered_i;
	pll->phasor_q2 = pll->phasor_q1;
	pll->phasor_q1 = filtered_q;
	
	/* Convert I/Q to phase error estimate.
	 * When locked, I is large positive and Q is near zero.
	 * Phase error = atan2(Q, I), but we use linear approx when close. */
	double phase_err;
	double abs_q = fabs(filtered_q);
	
	if (filtered_i > abs_q) {
		/* Within +/- 45 degrees of lock: use linear approximation */
		phase_err = filtered_q / filtered_i;
	} else if (filtered_q > 0) {
		/* Lagging more than 45 degrees */
		phase_err = 1.0;
	} else {
		/* Leading more than 45 degrees */
		phase_err = -1.0;
	}
	
	/* Update pilot level estimate (I component when locked) */
	pll->pilot_level = filtered_i;
	
	/* Run phase error through loop filter and update frequency */
	pll->freq += pll->loopfilter_b0 * phase_err
	           + pll->loopfilter_b1 * pll->loopfilter_x1;
	pll->loopfilter_x1 = phase_err;
	
	/* Limit frequency to allowable range */
	if (pll->freq < pll->min_freq)
		pll->freq = pll->min_freq;
	else if (pll->freq > pll->max_freq)
		pll->freq = pll->max_freq;
	
	/* Update phase */
	pll->phase += pll->freq;
	if (pll->phase >= 2.0 * M_PI) {
		pll->phase -= 2.0 * M_PI;
		pll->pilot_periods++;
	} else if (pll->phase < 0) {
		pll->phase += 2.0 * M_PI;
	}
	
	/* Update lock status */
	if (2.0 * pll->pilot_level > pll->min_signal) {
		if (pll->lock_cnt < pll->lock_delay)
			pll->lock_cnt++;
	} else {
		pll->lock_cnt = 0;
	}
	
	pll->sample_cnt++;
}

double pll_process(pll_t *pll, double sample_in)
{
	/* Generate local oscillator */
	pll->psin = sin(pll->phase);
	pll->pcos = cos(pll->phase);
	
	/* Multiply input with local oscillator to get I/Q phasor */
	double phasor_i = pll->psin * sample_in;
	double phasor_q = pll->pcos * sample_in;
	
	/* Process through PLL */
	pll_process_phasor(pll, phasor_i, phasor_q);
	
	return pll->phase;
}

double pll_process_iq(pll_t *pll, double real_in, double imag_in)
{
	/* Generate local oscillator */
	pll->psin = sin(pll->phase);
	pll->pcos = cos(pll->phase);
	
	/* Complex multiply: (real + j*imag) * (cos - j*sin) for downconversion */
	double phasor_i = pll->psin * real_in - pll->pcos * imag_in;
	double phasor_q = pll->pcos * real_in + pll->psin * imag_in;
	
	/* Process through PLL */
	pll_process_phasor(pll, phasor_i, phasor_q);
	
	return pll->phase;
}
