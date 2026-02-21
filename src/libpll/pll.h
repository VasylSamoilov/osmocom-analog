/* Phase-Locked Loop Library
 *
 * Type-2, 4th order PLL for carrier/pilot tracking.
 * Based on SDRangel's phaselock.cpp by Edouard Griffiths, F4EXB.
 *
 * (C) 2024 - Adapted for osmocom-analog
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef PLL_H
#define PLL_H

#include <stdint.h>

typedef struct pll {
	/* Configuration */
	double center_freq;      /* Center frequency (normalized: freq/samplerate) */
	double bandwidth;        /* Loop bandwidth (normalized) */
	double min_signal;       /* Minimum signal level for lock */
	
	/* Frequency limits */
	double min_freq;         /* Minimum frequency (rad/sample) */
	double max_freq;         /* Maximum frequency (rad/sample) */
	
	/* Phasor filter (2nd order IIR for I/Q smoothing) */
	double phasor_b0;
	double phasor_a1, phasor_a2;
	double phasor_i1, phasor_i2;  /* I filter state */
	double phasor_q1, phasor_q2;  /* Q filter state */
	
	/* Loop filter (1st order) */
	double loopfilter_b0, loopfilter_b1;
	double loopfilter_x1;
	
	/* PLL state */
	double freq;             /* Current frequency estimate (rad/sample) */
	double phase;            /* Current phase (0 to 2*PI) */
	double psin, pcos;       /* sin/cos of current phase */
	
	/* Lock detection */
	double pilot_level;      /* Detected signal level */
	int lock_delay;          /* Samples needed for lock */
	int lock_cnt;            /* Current lock counter */
	
	/* Statistics */
	uint64_t sample_cnt;
	int pilot_periods;       /* Cycles since last PPS */
} pll_t;

/* Initialize PLL
 * freq: center frequency normalized to sample rate (e.g., 19000/500000 = 0.038)
 * bandwidth: loop bandwidth normalized (e.g., 50/500000 = 0.0001)
 * min_signal: minimum signal level for lock detection (e.g., 0.01)
 */
void pll_init(pll_t *pll, double freq, double bandwidth, double min_signal);

/* Reconfigure PLL (can be called while running) */
void pll_configure(pll_t *pll, double freq, double bandwidth, double min_signal);

/* Reset PLL state */
void pll_reset(pll_t *pll);

/* Process one sample
 * sample_in: input signal (e.g., FM baseband)
 * Returns: current phase (0 to 2*PI)
 */
double pll_process(pll_t *pll, double sample_in);

/* Process one complex sample (I/Q input)
 * real_in, imag_in: complex input
 * Returns: current phase
 */
double pll_process_iq(pll_t *pll, double real_in, double imag_in);

/* Get current sin/cos of phase (for mixing) */
static inline void pll_get_sincos(pll_t *pll, double *psin, double *pcos)
{
	*psin = pll->psin;
	*pcos = pll->pcos;
}

/* Get current phase */
static inline double pll_get_phase(pll_t *pll)
{
	return pll->phase;
}

/* Get current frequency (rad/sample) */
static inline double pll_get_freq(pll_t *pll)
{
	return pll->freq;
}

/* Get frequency in Hz given sample rate */
static inline double pll_get_freq_hz(pll_t *pll, double samplerate)
{
	return pll->freq * samplerate / (2.0 * 3.14159265358979323846);
}

/* Get frequency error from center (Hz) */
static inline double pll_get_freq_error_hz(pll_t *pll, double samplerate)
{
	return (pll->freq - pll->center_freq * 2.0 * 3.14159265358979323846) * samplerate / (2.0 * 3.14159265358979323846);
}

/* Check if PLL is locked */
static inline int pll_is_locked(pll_t *pll)
{
	return pll->lock_cnt >= pll->lock_delay;
}

/* Get detected signal level */
static inline double pll_get_level(pll_t *pll)
{
	return 2.0 * pll->pilot_level;  /* Scale to match input amplitude */
}

/* Get stereo demodulation outputs (like sdrangel's RDSPhaseLock)
 * out[0] = sin(phase)      - 19 kHz pilot
 * out[1] = sin(2*phase)    - 38 kHz stereo carrier
 * out[2] = cos(2*phase)    - 38 kHz quadrature
 * out[3] = phase           - raw phase for RDS (57 kHz = 3x)
 */
static inline void pll_get_stereo_outputs(pll_t *pll, double *out)
{
	out[0] = pll->psin;                           /* sin(phase) - 19 kHz */
	out[1] = 2.0 * pll->psin * pll->pcos;         /* sin(2*phase) - 38 kHz */
	out[2] = 2.0 * pll->pcos * pll->pcos - 1.0;   /* cos(2*phase) - 38 kHz quad */
	out[3] = pll->phase;                          /* raw phase */
}

#endif /* PLL_H */
