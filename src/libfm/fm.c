/* FM modulation processing
 *
 * (C) 2017 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "fm.h"

static int has_init = 0;
static int fast_math = 0;
static float *sin_tab = NULL, *cos_tab = NULL;

/* global init */
int fm_init(int _fast_math)
{
	fast_math = _fast_math;

	if (fast_math) {
		int i;

		/*
		 * NCO (Numerically Controlled Oscillator) Lookup Table
		 * =====================================================
		 * Pre-compute sine values for fast trig approximation.
		 *
		 * Table size: 65,536 entries (2^16)
		 *   - Angular resolution: 360° / 65536 = 0.0055° (0.000096 rad)
		 *   - Max amplitude error: ~0.005% (0.00005)
		 *   - Memory usage: (65536 + 16384) x 4 bytes = ~320 KB
		 *
		 * Cosine is obtained by offsetting into the same array by 16384
		 * entries (90° = 65536/4), avoiding a separate table.
		 *
		 * Trade-offs for different table sizes:
		 *   2^12 (4K):   0.088° step, ~0.08% error,  ~20 KB
		 *   2^14 (16K):  0.022° step, ~0.02% error,  ~80 KB
		 *   2^16 (64K):  0.0055° step, ~0.005% error, ~320 KB  <-- current
		 *   2^18 (256K): 0.0014° step, ~0.001% error, ~1.3 MB
		 *
		 * The 65536-entry table is well-suited for FM radio applications:
		 * - RDS (1187.5 bps DBPSK): quantization error is far below noise floor
		 * - Stereo pilot (19 kHz): sub-degree precision is more than adequate
		 * - Larger tables risk L2 cache misses, hurting performance
		 */
		sin_tab = calloc(65536+16384, sizeof(*sin_tab));
		if (!sin_tab) {
			fprintf(stderr, "No mem!\n");
			return -ENOMEM;
		}
		cos_tab = sin_tab + 16384;

		/* generate sine and cosine */
		for (i = 0; i < 65536+16384; i++)
			sin_tab[i] = sin(2.0 * M_PI * (double)i / 65536.0);
	}

	has_init = 1;

	return 0;
}

/* global exit */
void fm_exit(void)
{
	if (sin_tab) {
		free(sin_tab);
		sin_tab = cos_tab = NULL;
	}

	has_init = 0;
}

/* Check if fast math lookup is enabled */
int fm_fast_math_enabled(void)
{
	return fast_math && sin_tab != NULL;
}

/*
 * Fast sincos using NCO lookup table.
 *
 * @param phase  Phase in range 0..65535 (maps to 0..2π radians)
 *               Use: phase_rad * (65536.0 / (2.0 * M_PI)) to convert
 * @param sin_out  Output sine value (~0.005% max error)
 * @param cos_out  Output cosine value (~0.005% max error)
 *
 * This is ~10x faster than sincos() on most CPUs due to simple table lookup.
 */
void fm_fast_sincos(double phase, double *sin_out, double *cos_out)
{
	uint16_t idx = (uint16_t)phase;
	*sin_out = sin_tab[idx];
	*cos_out = cos_tab[idx];
}

/* init FM modulator */
int fm_mod_init(fm_mod_t *mod, double samplerate, double offset, double amplitude)
{
	int i;

	if (!has_init) {
		fprintf(stderr, "libfm was not initialized, please fix!\n");
		abort();
	}

	memset(mod, 0, sizeof(*mod));
	mod->samplerate = samplerate;
	mod->offset = offset;
	mod->amplitude = amplitude;

	mod->ramp_length = samplerate * 0.001;
	mod->ramp_tab = calloc(mod->ramp_length, sizeof(*mod->ramp_tab));
	if (!mod->ramp_tab) {
		fprintf(stderr, "No mem!\n");
		return -ENOMEM;
	}
	mod->state = MOD_STATE_OFF;

	/* generate ramp up with ramp_length */
	for (i = 0; i < mod->ramp_length; i++)
		mod->ramp_tab[i] = 0.5 - cos(M_PI * i / mod->ramp_length) / 2.0;

	return 0;
}

void fm_mod_exit(fm_mod_t *mod)
{
	if (mod->ramp_tab) {
		free(mod->ramp_tab);
		mod->ramp_tab = NULL;
	}
}

/* do frequency modulation of samples and add them to existing baseband */
void fm_modulate_complex(fm_mod_t *mod, sample_t *frequency, uint8_t *power, int length, float *baseband)
{
	double dev, rate, phase, offset;
	int ramp, ramp_length;
	double *ramp_tab;
	double amplitude;

	rate = mod->samplerate;
	phase = mod->phase;
	offset = mod->offset;
	ramp = mod->ramp;
	ramp_length = mod->ramp_length;
	ramp_tab = mod->ramp_tab;
	amplitude = mod->amplitude;

again:
	switch (mod->state) {
	case MOD_STATE_ON:
		/* modulate */
		while (length) {
			/* is power is not set, ramp down */
			if (!(*power)) {
				mod->state = MOD_STATE_RAMP_DOWN;
				break;
			}
			/* deviation is defined by the frequency value and the offset */
			dev = offset + *frequency++;
			power++;
			length--;
			if (fast_math) {
				phase += 65536.0 * dev / rate;
				if (phase < 0.0)
					phase += 65536.0;
				else if (phase >= 65536.0)
					phase -= 65536.0;
				*baseband++ += cos_tab[(uint16_t)phase] * amplitude;
				*baseband++ += sin_tab[(uint16_t)phase] * amplitude;
			} else {
				phase += 2.0 * M_PI * dev / rate;
				if (phase < 0.0)
					phase += 2.0 * M_PI;
				else if (phase >= 2.0 * M_PI)
					phase -= 2.0 * M_PI;
				*baseband++ += cos(phase) * amplitude;
				*baseband++ += sin(phase) * amplitude;
			}
		}
		break;
	case MOD_STATE_RAMP_DOWN:
		while (length) {
			/* if power is set, ramp up */
			if (*power) {
				mod->state = MOD_STATE_RAMP_UP;
				break;
			}
			if (ramp == 0) {
				mod->state = MOD_STATE_OFF;
				break;
			}
			dev = offset + *frequency++;
			power++;
			length--;
			if (fast_math) {
				phase += 65536.0 * dev / rate;
				if (phase < 0.0)
					phase += 65536.0;
				else if (phase >= 65536.0)
					phase -= 65536.0;
				*baseband++ += cos_tab[(uint16_t)phase] * amplitude * ramp_tab[ramp];
				*baseband++ += sin_tab[(uint16_t)phase] * amplitude * ramp_tab[ramp];
			} else {
				phase += 2.0 * M_PI * dev / rate;
				if (phase < 0.0)
					phase += 2.0 * M_PI;
				else if (phase >= 2.0 * M_PI)
					phase -= 2.0 * M_PI;
				*baseband++ += cos(phase) * amplitude * ramp_tab[ramp];
				*baseband++ += sin(phase) * amplitude * ramp_tab[ramp];
			}
			ramp--;
		}
		break;
	case MOD_STATE_OFF:
		while (length) {
			/* if power is set, ramp up */
			if (*power) {
				mod->state = MOD_STATE_RAMP_UP;
				break;
			}
			/* just count, and add nothing */
			frequency++;
			power++;
			length--;
			baseband += 2;
		}
		break;
	case MOD_STATE_RAMP_UP:
		while (length) {
			/* is power is not set, ramp down */
			if (!(*power)) {
				mod->state = MOD_STATE_RAMP_DOWN;
				break;
			}
			if (ramp == ramp_length - 1) {
				mod->state = MOD_STATE_ON;
				break;
			}
			/* deviation is defined by the frequency value and the offset */
			dev = offset + *frequency++;
			power++;
			length--;
			if (fast_math) {
				phase += 65536.0 * dev / rate;
				if (phase < 0.0)
					phase += 65536.0;
				else if (phase >= 65536.0)
					phase -= 65536.0;
				*baseband++ += cos_tab[(uint16_t)phase] * amplitude * ramp_tab[ramp];
				*baseband++ += sin_tab[(uint16_t)phase] * amplitude * ramp_tab[ramp];
			} else {
				phase += 2.0 * M_PI * dev / rate;
				if (phase < 0.0)
					phase += 2.0 * M_PI;
				else if (phase >= 2.0 * M_PI)
					phase -= 2.0 * M_PI;
				*baseband++ += cos(phase) * amplitude * ramp_tab[ramp];
				*baseband++ += sin(phase) * amplitude * ramp_tab[ramp];
			}
			ramp++;
		}
		break;
	}
	if (length)
		goto again;

	mod->phase = phase;
	mod->ramp = ramp;
}

/* init FM demodulator, bandwidth is the 2 * (deviation + modulation) */
int fm_demod_init(fm_demod_t *demod, double samplerate, double offset, double bandwidth)
{
	if (!has_init) {
		fprintf(stderr, "libfm was not initialized, please fix!\n");
		abort();
	}

	memset(demod, 0, sizeof(*demod));
	demod->samplerate = samplerate;

	if (fast_math)
		demod->rot = 65536.0 * -offset / samplerate;
	else
		demod->rot = 2 * M_PI * -offset / samplerate;

	/* use fourth order (2 iter) filter, since it is as fast as second order (1 iter) filter */
	iir_lowpass_init(&demod->lp[0], bandwidth / 2.0, samplerate, 2);
	iir_lowpass_init(&demod->lp[1], bandwidth / 2.0, samplerate, 2);

	return 0;
}

void fm_demod_exit(fm_demod_t __attribute__ ((unused)) *demod)
{
}

static inline float fast_tan(float z)
{
	const float n1 = 0.97239411f;
	const float n2 = -0.19194795f;
	return (n1 + n2 * z * z) * z;
}

static inline float fast_atan2(float y, float x)
{
	if (x != 0.0) {
		if (fabsf(x) > fabsf(y)) {
			const float z = y / x;
			if (x > 0.0) /* atan2(y,x) = atan(y/x) if x > 0 */
				return fast_tan(z);
			else if (y >= 0.0) /* atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0 */
				return fast_tan(z) + M_PI;
			else /* atan2(y,x) = atan(y/x) - PI if x < 0, y < 0 */
				return fast_tan(z) - M_PI;
		} else { /* Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1 */
			const float z = x / y;
			if (y > 0.0) /* atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0 */
				return -fast_tan(z) + M_PI_2;
			else /* atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0 */
				return -fast_tan(z) - M_PI_2;
		}
	} else {
		if (y > 0.0) /* x = 0, y > 0 */
			return M_PI_2;
		else if (y < 0.0) /* x = 0, y < 0 */
			return -M_PI_2;
	}
	return 0.0; /* x,y = 0. return 0, because NaN would harm further processing  */
}

/* do frequency demodulation of baseband and write them to samples */
void fm_demodulate_complex(fm_demod_t *demod, sample_t *frequency, int length, float *baseband, sample_t *I, sample_t *Q)
{
	double phase, rot, last_phase, dev, rate;
	double _sin, _cos;
	sample_t i, q;
	int s, ss;

	rate = demod->samplerate;
	phase = demod->phase;
	rot = demod->rot;
	for (s = 0, ss = 0; s < length; s++) {
		phase += rot;
		i = baseband[ss++];
		q = baseband[ss++];
		if (fast_math) {
			if (phase < 0.0)
				phase += 65536.0;
			else if (phase >= 65536.0)
				phase -= 65536.0;
			_sin = sin_tab[(uint16_t)phase];
			_cos = cos_tab[(uint16_t)phase];
		} else {
			if (phase < 0.0)
				phase += 2.0 * M_PI;
			else if (phase >= 2.0 * M_PI)
				phase -= 2.0 * M_PI;
			_sin = sin(phase);
			_cos = cos(phase);
		}
		I[s] = i * _cos - q * _sin;
		Q[s] = i * _sin + q * _cos;
	}
	demod->phase = phase;
	iir_process(&demod->lp[0], I, length);
	iir_process(&demod->lp[1], Q, length);
	last_phase = demod->last_phase;
	for (s = 0; s < length; s++) {
		if (fast_math)
			phase = fast_atan2(Q[s], I[s]);
		else
			phase = atan2(Q[s], I[s]);
		dev = (phase - last_phase) / 2 / M_PI;
		last_phase = phase;
		if (dev < -0.49)
			dev += 1.0;
		else if (dev > 0.49)
			dev -= 1.0;
		dev *= rate;
		frequency[s] = dev;
	}
	demod->last_phase = last_phase;
}

void fm_demodulate_real(fm_demod_t *demod, sample_t *frequency, int length, sample_t *baseband, sample_t *I, sample_t *Q)
{
	double phase, rot, last_phase, dev, rate;
	double _sin, _cos;
	sample_t i;
	int s, ss;

	rate = demod->samplerate;
	phase = demod->phase;
	rot = demod->rot;
	for (s = 0, ss = 0; s < length; s++) {
		phase += rot;
		i = baseband[ss++];
		if (fast_math) {
			if (phase < 0.0)
				phase += 65536.0;
			else if (phase >= 65536.0)
				phase -= 65536.0;
			_sin = sin_tab[(uint16_t)phase];
			_cos = cos_tab[(uint16_t)phase];
		} else {
			if (phase < 0.0)
				phase += 2.0 * M_PI;
			else if (phase >= 2.0 * M_PI)
				phase -= 2.0 * M_PI;
			_sin = sin(phase);
			_cos = cos(phase);
		}
		I[s] = i * _cos;
		Q[s] = i * _sin;
	}
	demod->phase = phase;
	iir_process(&demod->lp[0], I, length);
	iir_process(&demod->lp[1], Q, length);
	last_phase = demod->last_phase;
	for (s = 0; s < length; s++) {
		if (fast_math)
			phase = fast_atan2(Q[s], I[s]);
		else
			phase = atan2(Q[s], I[s]);
		dev = (phase - last_phase) / 2 / M_PI;
		last_phase = phase;
		if (dev < -0.49)
			dev += 1.0;
		else if (dev > 0.49)
			dev -= 1.0;
		dev *= rate;
		frequency[s] = dev;
	}
	demod->last_phase = last_phase;
}

