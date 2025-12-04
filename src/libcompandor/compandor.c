/* Compandor to use various networks like C-Netz / NMT / AMPS / TACS
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
#include "compandor.h"

//#define db2level(db)                 pow(10, (double)db / 20.0)

/*
 * Compandor Attack/Recovery Time Constants
 * =========================================
 * Per TIA/EIA-553 Section 2.1.3.1.1 and 2.2.2.1.2, referencing ITU-T G.162:
 *   - Attack time:   3 ms (nominal)
 *   - Recovery time: 13.5 ms (nominal)
 *
 * ITU-T G.162 defines these times as the time for the output to reach
 * within 2 dB of its final value after a step change in input level.
 *
 * For a 2:1 compressor with 12 dB input step:
 *   - Final output change = 6 dB
 *   - "Within 2 dB" means output has changed by at least 4 dB
 *   - This is 4/6 = 66.7% of the final value
 *
 * For exponential response: 1 - e^(-t/tau) = 0.667
 *   - e^(-t/tau) = 0.333
 *   - tau = t / ln(3) = t / 1.099
 *
 * Effective time constants:
 *   - tau_attack = 3.0 ms / 1.099 = 2.73 ms
 *   - tau_recovery = 13.5 ms / 1.099 = 12.3 ms
 *
 * Per-sample step values at sample rate fs:
 *   - step_up = e^(1 / (tau_attack * fs))
 *   - step_down = e^(-1 / (tau_recovery * fs))
 *
 * At 8000 Hz:
 *   - step_up = e^(1 / (0.00273 * 8000)) = e^0.0458 = 1.0469
 *   - step_down = e^(-1 / (0.0123 * 8000)) = e^-0.0102 = 0.9899
 *
 * FACTOR values are chosen so that:
 *   step = pow(FACTOR, 1000.0 / time_ms / samplerate)
 *
 * After attack_ms at 8000 Hz (24 samples):
 *   ATTACK_FACTOR = step_up^24 = 1.0469^24 = 3.0
 *
 * After recovery_ms at 8000 Hz (108 samples):
 *   RECOVERY_FACTOR = step_down^108 = 0.9899^108 = 0.33
 */
#define COMPANDOR_ATTACK_FACTOR		3.0	/* Envelope multiplier after attack time (3ms) */
#define COMPANDOR_RECOVERY_FACTOR	0.33	/* Envelope multiplier after recovery time (13.5ms) */

/* Minimum level value to keep state (-60 dB) */
#define ENVELOPE_MIN	0.001

/* Maximum level, to prevent sqrt_tab to overflow */
#define ENVELOPE_MAX	9.990

static double sqrt_tab[10000];
static int compandor_initalized = 0;

/*
 * Init compandor according to ITU-T G.162 specification
 *
 * Hopefully this is correct
 *
 */

void compandor_init(void)
{
	int i;

	// FIXME: make global, not at instance
	for (i = 0; i < 10000; i++)
		sqrt_tab[i] = sqrt(i * 0.001);
	compandor_initalized = 1;
}

void setup_compandor(compandor_t *state, double samplerate, double attack_ms, double recovery_ms)
{
	if (!compandor_initalized) {
		fprintf(stderr, "Compandor nicht initialized.\n");
		abort();
	}

	memset(state, 0, sizeof(*state));

	state->c.peak = 1.0;
	state->c.envelope = 1.0;
	state->e.peak = 1.0;
	state->e.envelope = 1.0;
	/* Both compressor and expander use same attack/recovery per TIA/EIA-553 */
	state->c.step_up = pow(COMPANDOR_ATTACK_FACTOR, 1000.0 / attack_ms / samplerate);
	state->c.step_down = pow(COMPANDOR_RECOVERY_FACTOR, 1000.0 / recovery_ms / samplerate);
	state->e.step_up = pow(COMPANDOR_ATTACK_FACTOR, 1000.0 / attack_ms / samplerate);
	state->e.step_down = pow(COMPANDOR_RECOVERY_FACTOR, 1000.0 / recovery_ms / samplerate);
}

void compress_audio(compandor_t *state, sample_t *samples, int num)
{
	double value, peak, envelope, step_up, step_down;
	int i;

	step_up = state->c.step_up;
	step_down = state->c.step_down;
	peak = state->c.peak;
	envelope = state->c.envelope;

//	printf("envelope=%.4f\n", envelope);
	for (i = 0; i < num; i++) {
		value = *samples;

		/* 'peak' is the level that rises instantly with the signal
		 * level, but falls with specified recovery rate. */
		if (fabs(value) > peak)
			peak = fabs(value);
		else
			peak *= step_down;

		/* 'envelope' follows peak with attack/recovery timing:
		 * - Attack (3ms): envelope rises slowly toward peak
		 * - Recovery (13.5ms): envelope falls slowly toward peak
		 * Per TIA/EIA-553 and ITU-T G.162 specifications.
		 */
		if (peak > envelope)
			envelope *= step_up;    /* Attack: slow rise */
		else
			envelope *= step_down;  /* Recovery: slow fall (not instant!) */

		/* Clamp envelope to valid range */
		if (envelope < ENVELOPE_MIN)
			envelope = ENVELOPE_MIN;
		if (envelope > ENVELOPE_MAX)
			envelope = ENVELOPE_MAX;

		*samples++ = value / sqrt_tab[(int)(envelope / 0.001)];
//if (i > 47000.0 && i < 48144)
//printf("time=%.4f envelope=%.4fdb, value=%.4f\n", (double)i/48000.0, 20*log10(envelope), value);
	}
//exit(0);

	state->c.envelope = envelope;
	state->c.peak = peak;
}

void expand_audio(compandor_t *state, sample_t *samples, int num)
{
	double value, peak, envelope, step_up, step_down;
	int i;

	step_up = state->e.step_up;
	step_down = state->e.step_down;
	peak = state->e.peak;
	envelope = state->e.envelope;

	for (i = 0; i < num; i++) {
		value = *samples;

		/* for comments: see compress_audio() */
		/* 'peak' is the level that rises instantly with the signal
		 * level, but falls with specified recovery rate. */
		if (fabs(value) > peak)
			peak = fabs(value);
		else
			peak *= step_down;

		/* 'envelope' follows peak with attack/recovery timing:
		 * - Attack (3ms): envelope rises slowly toward peak
		 * - Recovery (13.5ms): envelope falls slowly toward peak
		 * Per TIA/EIA-553 and ITU-T G.162 specifications.
		 */
		if (peak > envelope)
			envelope *= step_up;    /* Attack: slow rise */
		else
			envelope *= step_down;  /* Recovery: slow fall (not instant!) */

		/* Clamp envelope to valid range */
		if (envelope < ENVELOPE_MIN)
			envelope = ENVELOPE_MIN;

		/* Expansion uses sqrt(envelope) to undo 2:1 compression */
		/* Compression divides by sqrt(envelope), so expansion multiplies by sqrt(envelope) */
		*samples++ = value * sqrt(envelope);
	}

	state->e.envelope = envelope;
	state->e.peak = peak;
}

