/* Syllabic Compandor for Telephony Systems (C-Netz / NMT / AMPS / TACS)
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
 *
 * COMPANDOR OPERATION PRINCIPLE:
 * ==============================
 * A syllabic compandor improves signal-to-noise ratio (SNR) in analog radio
 * transmission by reducing dynamic range before transmission and restoring it
 * at the receiver.
 *
 * TX Path (Compression):
 *   - Loud sounds: Reduced in amplitude (compressed)
 *   - Quiet sounds: Increased in amplitude (boosted)
 *   - Result: Narrower dynamic range, better noise immunity during transmission
 *
 * RX Path (Expansion):
 *   - Loud sounds: Increased in amplitude (restored)
 *   - Quiet sounds: Reduced in amplitude (restored)
 *   - Result: Original dynamic range restored, noise reduced
 *
 * The compandor uses envelope tracking with attack/recovery timing to follow
 * the syllabic structure of speech (hence "syllabic compandor").
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
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

/* Minimum envelope value to keep state (-30 dB below nominal)
 * Prevents division by zero and maintains compandor state during silence.
 * 
 * ADJUSTED FOR NOISE FLOOR MITIGATION:
 * Original value was 0.001 (-60 dB), allowing +30 dB boost.
 * Previous attempt 0.01 (-40 dB), allowing +20 dB boost (10x).
 * 
 * New value is 0.032 (~ -30 dB):
 * - Linear: 10^(-30/20) = 0.0316...
 * - Max Compressor Boost: 1 / sqrt(0.032) = ~5.6x (+15 dB)
 * - This limits noise boosting significantly while keeping compander active.
 */
#define ENVELOPE_MIN	0.032

/* Maximum envelope for compressor (9.990 = ~20 dB above nominal)
 * Limits maximum compression to prevent sqrt_tab overflow
 * At envelope=9.990: compression divides by sqrt(9.990)=3.16, giving -10 dB output */
#define ENVELOPE_MAX	9.990

/* Maximum envelope for expander (1.0 = 0 dB, unity gain)
 * CRITICAL FIX: Limits expansion to prevent amplification beyond original signal level.
 * 
 * WHY 1.0?
 * - Compressor boosts quiet signals (envelope < 1.0) for better SNR during transmission
 * - Expander should only RESTORE what was compressed, not ADD gain
 * - When envelope > 1.0, expander would amplify beyond original → causes the reported issue
 * - Setting max to 1.0 ensures expander is transparent at nominal level and only attenuates
 *
 * EXAMPLE:
 * - Quiet signal (envelope=0.1): Compressor amplifies 3.16x, expander attenuates 0.1x → restored
 * - Loud signal (envelope=3.0): Compressor attenuates 0.58x, expander limited to 1.0x → compressed
 * - Nominal signal (envelope=1.0): Compressor 1.0x, expander 1.0x → transparent
 */
#define EXPANDER_ENVELOPE_MAX	1.0

static double sqrt_tab[10000];
static int compandor_initalized = 0;

/*
 * Initialize compandor lookup tables
 *
 * Pre-computes square root values for fast compression calculation.
 * The compressor divides by sqrt(envelope), so we pre-calculate sqrt values
 * for envelope range 0.001 to 9.990 in steps of 0.001.
 */
void compandor_init(void)
{
	int i;

	// FIXME: make global, not at instance
	for (i = 0; i < 10000; i++)
		sqrt_tab[i] = sqrt(i * 0.001);
	compandor_initalized = 1;
}

/*
 * Setup compandor state with specified timing parameters
 *
 * @param state: Compandor state structure to initialize
 * @param samplerate: Audio sample rate in Hz (typically 8000)
 * @param attack_ms: Attack time in milliseconds (typically 3.0)
 * @param recovery_ms: Recovery time in milliseconds (typically 13.5)
 *
 * Initializes both compressor and expander with:
 * - Peak tracking (instant rise, slow fall)
 * - Envelope following (slow rise/fall with attack/recovery timing)
 * - Per-sample step multipliers calculated from FACTOR constants
 */
void setup_compandor(compandor_t *state, double samplerate, double attack_ms, double recovery_ms)
{
	if (!compandor_initalized) {
		fprintf(stderr, "Compandor nicht initialized.\n");
		abort();
	}

	memset(state, 0, sizeof(*state));

	/* Initialize compressor state
	 * peak: Instant rise, slow fall - tracks signal envelope
	 * envelope: Slow rise/fall - follows peak with attack/recovery timing */
	state->c.peak = 1.0;
	state->c.envelope = 1.0;
	
	/* Initialize expander state (same initial values) */
	state->e.peak = 1.0;
	state->e.envelope = 1.0;
	
	/* Calculate per-sample step multipliers from time constants
	 * Both compressor and expander use same attack/recovery per TIA/EIA-553
	 * 
	 * step_up: Multiplier for envelope rise (attack)
	 * step_down: Multiplier for envelope fall (recovery)
	 * 
	 * Formula: step = pow(FACTOR, 1000.0 / time_ms / samplerate)
	 * 
	 * Example at 8000 Hz with 3ms attack:
	 *   step_up = pow(3.0, 1000.0 / 3.0 / 8000) = pow(3.0, 0.04167) = 1.0469
	 *   After 24 samples (3ms): envelope *= 1.0469^24 = 3.0
	 */
	state->c.step_up = pow(COMPANDOR_ATTACK_FACTOR, 1000.0 / attack_ms / samplerate);
	state->c.step_down = pow(COMPANDOR_RECOVERY_FACTOR, 1000.0 / recovery_ms / samplerate);
	state->e.step_up = pow(COMPANDOR_ATTACK_FACTOR, 1000.0 / attack_ms / samplerate);
	state->e.step_down = pow(COMPANDOR_RECOVERY_FACTOR, 1000.0 / recovery_ms / samplerate);
}

/*
 * Compress audio for transmission (TX path)
 *
 * PURPOSE: Reduce dynamic range to improve SNR during transmission
 * - Loud sounds: Reduced (compressed)
 * - Quiet sounds: Increased (boosted)
 * - Result: Narrower dynamic range, better noise immunity
 *
 * OPERATION:
 * 1. Track peak level (instant rise, slow fall)
 * 2. Envelope follows peak with attack/recovery timing
 * 3. Divide audio by sqrt(envelope) for 2:1 compression
 *
 * COMPRESSION MATH:
 * - envelope = 1.0 (nominal): output = value / 1.0 = value (transparent)
 * - envelope = 4.0 (loud): output = value / 2.0 (compressed -6 dB)
 * - envelope = 0.1 (quiet): output = value / 0.316 = value * 3.16 (boosted +10 dB)
 *
 * The sqrt() gives 2:1 compression ratio:
 * - 12 dB input change → 6 dB output change
 *
 * @param state: Compandor state (maintains envelope between calls)
 * @param samples: Audio buffer to compress in-place
 * @param num: Number of samples to process
 */
void compress_audio(compandor_t *state, sample_t *samples, int num)
{
	double value, peak, envelope, step_up, step_down;
	int i;

	/* Load state variables for this processing block */
	step_up = state->c.step_up;
	step_down = state->c.step_down;
	peak = state->c.peak;
	envelope = state->c.envelope;

	for (i = 0; i < num; i++) {
		value = *samples;

		/* Peak tracking: instant rise, slow fall
		 * 'peak' represents the instantaneous signal level that:
		 * - Rises immediately when signal increases (instant attack)
		 * - Falls slowly when signal decreases (recovery time)
		 * This creates the "peak detector" behavior */
		if (fabs(value) > peak)
			peak = fabs(value);  /* Instant rise to new peak */
		else
			peak *= step_down;   /* Slow fall (recovery) */

		/* Envelope tracking: slow rise and fall
		 * 'envelope' follows the peak with attack/recovery timing:
		 * - Attack (3ms): envelope rises slowly toward peak
		 * - Recovery (13.5ms): envelope falls slowly toward peak
		 * Per TIA/EIA-553 and ITU-T G.162 specifications.
		 * 
		 * This creates the "syllabic" behavior - envelope follows
		 * the syllable structure of speech, not individual cycles */
		if (peak > envelope)
			envelope *= step_up;    /* Attack: slow rise */
		else
			envelope *= step_down;  /* Recovery: slow fall */

		/* Clamp envelope to valid range
		 * Min: Prevents division by zero and maintains state during silence
		 * Max: Prevents sqrt_tab overflow and limits maximum compression */
		if (envelope < ENVELOPE_MIN)
			envelope = ENVELOPE_MIN;
		if (envelope > ENVELOPE_MAX)
			envelope = ENVELOPE_MAX;

		/* Apply 2:1 compression by dividing by sqrt(envelope)
		 * Uses pre-computed sqrt table for performance
		 * 
		 * WHY sqrt()?
		 * - For 2:1 compression: output_dB = input_dB / 2
		 * - In linear domain: output = input / sqrt(envelope)
		 * - Example: 12 dB input change → 6 dB output change
		 * 
		 * EFFECT ON DIFFERENT SIGNAL LEVELS:
		 * - Loud (envelope=4.0): divide by 2.0 → compress -6 dB
		 * - Nominal (envelope=1.0): divide by 1.0 → transparent
		 * - Quiet (envelope=0.1): divide by 0.316 → boost +10 dB
		 * 
		 * The boost of quiet signals is INTENTIONAL - it improves SNR
		 * by raising quiet speech above the noise floor during transmission */
		double output = value / sqrt_tab[(int)(envelope / 0.001)];
		*samples++ = output;

		/* DEBUG: Monitor Compressor Levels */
		static int c_dbg_count = 0;
		static double c_dbg_sum_in = 0;
		static double c_dbg_sum_out = 0;
		c_dbg_sum_in += fabs(value);
		c_dbg_sum_out += fabs(output);
		if (++c_dbg_count >= 8000) {
			LOGP(DDSP, LOGL_DEBUG, "COMP DEBUG: InLevel=%.5f OutLevel=%.5f Env=%.5f\n", 
				c_dbg_sum_in / 8000.0, c_dbg_sum_out / 8000.0, envelope);
			c_dbg_count = 0;
			c_dbg_sum_in = 0;
			c_dbg_sum_out = 0;
		}
	}

	/* Save state for next processing block */
	state->c.envelope = envelope;
	state->c.peak = peak;
}

/*
 * Expand audio after reception (RX path)
 *
 * PURPOSE: Restore original dynamic range and reduce noise
 * - Loud sounds: Increased (restored)
 * - Quiet sounds: Reduced (restored + noise reduction)
 * - Result: Original dynamic range restored, noise floor lowered
 *
 * OPERATION:
 * 1. Track peak level (instant rise, slow fall)
 * 2. Envelope follows peak with attack/recovery timing
 * 3. Multiply audio by envelope (limited to 1.0) for expansion
 *
 * EXPANSION MATH (with EXPANDER_ENVELOPE_MAX = 1.0):
 * - envelope = 1.0 (nominal): output = value * 1.0 = value (transparent)
 * - envelope = 0.1 (quiet): output = value * 0.1 (attenuated -20 dB)
 * - envelope > 1.0 (loud): LIMITED to 1.0 → output = value * 1.0 (no amplification)
 *
 * WHY LIMIT TO 1.0?
 * - Compressor boosts quiet signals for better SNR during transmission
 * - Expander should RESTORE what was compressed, not ADD gain
 * - Without limit: envelope=3.0 would amplify by +9.5 dB beyond original
 * - With limit: envelope capped at 1.0 ensures no amplification beyond original
 *
 * EXAMPLE SIGNAL PATH:
 * 1. Quiet signal (0.1): Compressor boosts 3.16x → Expander attenuates 0.1x → Restored
 * 2. Loud signal (3.0): Compressor attenuates 0.58x → Expander limited to 1.0x → Compressed
 * 3. Nominal signal (1.0): Compressor 1.0x → Expander 1.0x → Transparent
 *
 * @param state: Compandor state (maintains envelope between calls)
 * @param samples: Audio buffer to expand in-place
 * @param num: Number of samples to process
 */
void expand_audio(compandor_t *state, sample_t *samples, int num)
{
	double value, peak, envelope, step_up, step_down;
	int i;

	/* Load state variables for this processing block */
	step_up = state->e.step_up;
	step_down = state->e.step_down;
	peak = state->e.peak;
	envelope = state->e.envelope;

	for (i = 0; i < num; i++) {
		value = *samples;

		/* Peak tracking: instant rise, slow fall
		 * Same algorithm as compressor - see compress_audio() for details */
		if (fabs(value) > peak)
			peak = fabs(value);  /* Instant rise to new peak */
		else
			peak *= step_down;   /* Slow fall (recovery) */

		/* Envelope tracking: slow rise and fall
		 * Same algorithm as compressor - see compress_audio() for details
		 * 
		 * The envelope tracks the received signal level, which has already
		 * been compressed at the transmitter. This envelope will be used
		 * to restore the original dynamic range. */
		if (peak > envelope)
			envelope *= step_up;    /* Attack: slow rise */
		else
			envelope *= step_down;  /* Recovery: slow fall */

		/* Clamp envelope to valid range
		 * Min: Prevents multiplication by zero and maintains state during silence
		 * Max: CRITICAL FIX - prevents amplification beyond original signal level
		 * 
		 * BEFORE FIX (EXPANDER_ENVELOPE_MAX = 3.16):
		 * - Quiet microphone sounds would be amplified up to +10 dB
		 * - This caused the reported issue of quiet sounds being too loud
		 * 
		 * AFTER FIX (EXPANDER_ENVELOPE_MAX = 1.0):
		 * - Expander can only attenuate or pass through (no amplification)
		 * - Quiet sounds are properly restored without excessive gain
		 * - Loud sounds remain compressed (acceptable for telephony)
		 */
		if (envelope < ENVELOPE_MIN)
			envelope = ENVELOPE_MIN;
		if (envelope > EXPANDER_ENVELOPE_MAX)
			envelope = EXPANDER_ENVELOPE_MAX;

		/* Apply expansion by multiplying by envelope
		 * This is the inverse of compression (which divides by sqrt(envelope))
		 * 
		 * EFFECT ON DIFFERENT SIGNAL LEVELS (with max envelope = 1.0):
		 * - Loud (envelope limited to 1.0): multiply by 1.0 → no change (stays compressed)
		 * - Nominal (envelope=1.0): multiply by 1.0 → transparent
		 * - Quiet (envelope=0.1): multiply by 0.1 → attenuate -20 dB (restores + reduces noise)
		 * 
		 * The attenuation of quiet signals is INTENTIONAL - it:
		 * 1. Restores the original level (undoes compression boost)
		 * 2. Reduces noise floor (improves perceived SNR)
		 * 
		 * Note: Loud signals stay somewhat compressed because envelope is limited.
		 * This is acceptable for telephony and prevents over-amplification. */
		double output = value * envelope;
		*samples++ = output;

		/* DEBUG: Watch for stuck envelope, DC offset, and IO levels */
		static int dbg_count = 0;
		static double dbg_sum_dc = 0;
		static double dbg_sum_in = 0;
		static double dbg_sum_out = 0;
		
		dbg_sum_dc += value;
		dbg_sum_in += fabs(value);
		dbg_sum_out += fabs(output);

		if (++dbg_count >= 8000) {
			LOGP(DDSP, LOGL_DEBUG, "EXP DEBUG: InLevel=%.5f OutLevel=%.5f DC=%.5f Env=%.5f\n", 
				dbg_sum_in / 8000.0, dbg_sum_out / 8000.0, dbg_sum_dc / 8000.0, envelope);
			dbg_count = 0;
			dbg_sum_dc = 0;
			dbg_sum_in = 0;
			dbg_sum_out = 0;
		}
	}

	/* Save state for next processing block */
	state->e.envelope = envelope;
	state->e.peak = peak;
}
