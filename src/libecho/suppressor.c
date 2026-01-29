/* Echo Suppressor for Analog Telephony
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
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
 * SDR ECHO DELAY COMPENSATION
 * ===========================
 * In SDR environments, there is significant roundtrip delay between TX and
 * the echo arriving on RX:
 *
 *   TX audio → SDR TX buffer → RF → phone → phone mic → RF → SDR RX → RX audio
 *
 * This delay can be 50-200ms+ depending on:
 *   - SDR TX/RX buffer sizes
 *   - RF propagation time
 *   - Phone audio processing latency
 *
 * Without delay compensation, the suppressor would see:
 *   Time:  T0          T1          T2          T3
 *   TX:    [SPEECH]    [silence]   [silence]   [silence]
 *   RX:    [silence]   [silence]   [ECHO!]     [silence]
 *
 * By T2, TX energy has decayed, so the echo is not suppressed.
 *
 * SOLUTION: TX Energy Delay Buffer
 * We store TX energy history in a circular buffer and compare RX energy
 * against the TX energy from echo_delay_ms ago. This allows proper echo
 * detection even with significant roundtrip delay.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "suppressor.h"
#include "../liblogging/logging.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Maximum echo delay we support (500ms should cover most SDR setups) */
#define MAX_ECHO_DELAY_MS 500

/* Frame duration assumption for history buffer sizing (16ms typical) */
#define FRAME_DURATION_MS 16

/* Calculate RMS energy of audio samples in dB
 * samples: Audio samples (int16_t format)
 * count: Number of samples
 * Returns: Energy in dB (relative to full scale)
 */
static double calculate_energy_db(const int16_t *samples, int count)
{
	double sum = 0.0;
	int i;
	
	if (count <= 0)
		return -100.0;
	
	for (i = 0; i < count; i++) {
		double s = (double)samples[i] / 32768.0;
		sum += s * s;
	}
	
	double rms = sqrt(sum / count);
	if (rms < 1e-10)
		return -100.0;
	
	return 20.0 * log10(rms);
}

/* Apply exponential smoothing to energy value
 * current: Current energy value
 * previous: Previous smoothed value
 * alpha: Smoothing factor (0.0-1.0, higher = more current)
 * Returns: Smoothed energy value
 */
static double smooth_energy(double current, double previous, double alpha)
{
	return alpha * current + (1.0 - alpha) * previous;
}

/* Determine direction based on energy levels
 * tx_energy_db: TX energy in dB
 * rx_energy_db: RX energy in dB
 * threshold_db: Threshold for direction decision
 * doubletalk_threshold_db: Threshold for double-talk detection
 * Returns: Direction state
 */
static suppressor_direction_t determine_direction(double tx_energy_db, double rx_energy_db,
                                                   double threshold_db, double doubletalk_threshold_db)
{
	double energy_diff_db = tx_energy_db - rx_energy_db;
	
	if (energy_diff_db > threshold_db) {
		/* TX dominant - attenuate RX */
		return SUPP_DIR_TX;
	} else if (energy_diff_db < -threshold_db) {
		/* RX dominant - attenuate TX */
		return SUPP_DIR_RX;
	} else if (fabs(energy_diff_db) < doubletalk_threshold_db) {
		/* Both directions active */
		return SUPP_DIR_DOUBLETALK;
	} else {
		/* No clear dominant direction */
		return SUPP_DIR_NONE;
	}
}

/* Update hangover timers based on direction
 * state: Echo suppressor state
 * direction: Current direction
 * frame_size: Number of samples in current frame
 */
static void update_hangover(echo_suppressor_state_t *state, suppressor_direction_t direction, int frame_size)
{
	if (direction == SUPP_DIR_TX) {
		/* TX dominant - reset TX hangover, decay RX hangover */
		state->tx_hangover = state->hangover_samples;
		state->rx_hangover = MAX(0, state->rx_hangover - frame_size);
	} else if (direction == SUPP_DIR_RX) {
		/* RX dominant - reset RX hangover, decay TX hangover */
		state->rx_hangover = state->hangover_samples;
		state->tx_hangover = MAX(0, state->tx_hangover - frame_size);
	} else {
		/* No dominant direction - decay both hangover timers */
		state->tx_hangover = MAX(0, state->tx_hangover - frame_size);
		state->rx_hangover = MAX(0, state->rx_hangover - frame_size);
	}
}

/* Calculate target gains based on direction and hangover state
 * state: Echo suppressor state
 * direction: Current direction
 */
static void calculate_target_gains(echo_suppressor_state_t *state, suppressor_direction_t direction)
{
	if (state->tx_hangover > 0) {
		/* TX direction active - pass TX, attenuate RX */
		state->tx_gain_target = 1.0;
		state->rx_gain_target = state->attenuation_linear;
	} else if (state->rx_hangover > 0) {
		/* RX direction active - attenuate TX, pass RX */
		state->tx_gain_target = state->attenuation_linear;
		state->rx_gain_target = 1.0;
	} else if (direction == SUPP_DIR_DOUBLETALK) {
		/* Double-talk - partial attenuation both directions */
		state->tx_gain_target = 0.5;
		state->rx_gain_target = 0.5;
	} else {
		/* No direction - pass both */
		state->tx_gain_target = 1.0;
		state->rx_gain_target = 1.0;
	}
}

/* Apply smooth gain ramping to current gain
 * current_gain: Current gain value
 * target_gain: Target gain value
 * ramp_alpha: Ramping smoothing factor
 * Returns: New gain value
 */
static double ramp_gain(double current_gain, double target_gain, double ramp_alpha)
{
	return ramp_alpha * target_gain + (1.0 - ramp_alpha) * current_gain;
}

/* Apply gain to audio samples
 * samples: Audio samples (int16_t format)
 * count: Number of samples
 * gain: Gain to apply (0.0-1.0)
 */
static void apply_gain(int16_t *samples, int count, double gain)
{
	int i;
	
	/* Clamp gain to valid range */
	if (gain < 0.0)
		gain = 0.0;
	if (gain > 1.0)
		gain = 1.0;
	
	for (i = 0; i < count; i++) {
		double s = (double)samples[i] * gain;
		
		/* Clamp to int16 range */
		if (s > 32767.0)
			s = 32767.0;
		if (s < -32768.0)
			s = -32768.0;
		
		samples[i] = (int16_t)s;
	}
}

/* Initialize echo suppressor */
echo_suppressor_state_t *echo_suppressor_init(int sample_rate, int frame_size,
                                               const echo_suppressor_config_t *config)
{
	echo_suppressor_state_t *state;
	int echo_delay_ms;
	
	if (!config || sample_rate <= 0 || frame_size <= 0)
		return NULL;
	
	state = calloc(1, sizeof(*state));
	if (!state) {
		LOGP(DECHO, LOGL_ERROR, "Failed to allocate echo suppressor state\n");
		return NULL;
	}
	
	/* Initialize energy values */
	state->tx_energy = -100.0;
	state->rx_energy = -100.0;
	state->tx_energy_smooth = -100.0;
	state->rx_energy_smooth = -100.0;
	
	/* Initialize TX energy delay buffer for SDR roundtrip compensation
	 * Buffer size = max_delay / frame_duration + 1 (for safety margin) */
	echo_delay_ms = config->echo_delay_ms;
	if (echo_delay_ms < 0)
		echo_delay_ms = 0;
	if (echo_delay_ms > MAX_ECHO_DELAY_MS)
		echo_delay_ms = MAX_ECHO_DELAY_MS;
	
	/* Calculate frame duration from sample_rate and frame_size */
	int frame_duration_ms = (frame_size * 1000) / sample_rate;
	if (frame_duration_ms < 1)
		frame_duration_ms = 1;
	
	/* Allocate history buffer (always allocate, even if delay=0, for future adjustment) */
	state->history_size = (MAX_ECHO_DELAY_MS / frame_duration_ms) + 2;
	state->tx_energy_history = calloc(state->history_size, sizeof(double));
	if (!state->tx_energy_history) {
		LOGP(DECHO, LOGL_ERROR, "Failed to allocate echo suppressor history buffer\n");
		free(state);
		return NULL;
	}
	
	/* Initialize history buffer to silence */
	for (int i = 0; i < state->history_size; i++)
		state->tx_energy_history[i] = -100.0;
	
	state->history_write_pos = 0;
	state->delay_frames = echo_delay_ms / frame_duration_ms;
	state->delayed_tx_energy = -100.0;
	
	/* Initialize direction */
	state->direction = SUPP_DIR_NONE;
	state->prev_direction = SUPP_DIR_NONE;
	
	/* Initialize hangover timers */
	state->tx_hangover = 0;
	state->rx_hangover = 0;
	
	/* Initialize gains */
	state->tx_gain = 1.0;
	state->rx_gain = 1.0;
	state->tx_gain_target = 1.0;
	state->rx_gain_target = 1.0;
	
	/* Set configuration */
	state->threshold_db = config->threshold_db;
	state->attenuation_db = config->attenuation_db;
	state->attenuation_linear = pow(10.0, -config->attenuation_db / 20.0);
	state->hangover_samples = (config->hangover_ms * sample_rate) / 1000;
	state->doubletalk_threshold_db = config->doubletalk_threshold_db;
	
	/* Calculate smoothing factors */
	/* Energy smoothing: alpha = 0.3 (30% current, 70% previous) */
	state->energy_alpha = 0.3;
	
	/* Gain ramping: ramp over ramp_ms duration */
	int ramp_samples = (config->ramp_ms * sample_rate) / 1000;
	if (ramp_samples > 0)
		state->ramp_alpha = 1.0 / (double)ramp_samples;
	else
		state->ramp_alpha = 1.0;
	
	/* Initialize statistics */
	state->tx_frames = 0;
	state->rx_frames = 0;
	state->direction_changes = 0;
	state->doubletalk_frames = 0;
	
	LOGP(DECHO, LOGL_INFO, "Echo suppressor initialized: delay=%d frames (%d ms), history=%d frames, thresh=%.1f dB, att=%.1f dB\n",
	     state->delay_frames, echo_delay_ms, state->history_size, state->threshold_db, state->attenuation_db);

	return state;
}

/* Process TX (far-end) audio samples */
void echo_suppressor_process_tx(echo_suppressor_state_t *state, int16_t *samples, int count)
{
	if (!state || !samples || count <= 0)
		return;
	
	/* Calculate energy */
	state->tx_energy = calculate_energy_db(samples, count);
	
	/* Store TX energy in history buffer for delayed comparison
	 * This allows RX processing to compare against TX energy from
	 * echo_delay_ms ago, compensating for SDR roundtrip delay */
	if (state->tx_energy_history) {
		state->tx_energy_history[state->history_write_pos] = state->tx_energy;
		state->history_write_pos = (state->history_write_pos + 1) % state->history_size;
	}
	
	/* Smooth energy */
	state->tx_energy_smooth = smooth_energy(state->tx_energy, state->tx_energy_smooth, state->energy_alpha);
	
	/* For TX processing, we use current (non-delayed) energy comparison
	 * because we want to detect when the local user is speaking */
	suppressor_direction_t direction = determine_direction(state->tx_energy_smooth, state->rx_energy_smooth,
	                                                        state->threshold_db, state->doubletalk_threshold_db);
	
	/* Update hangover timers */
	update_hangover(state, direction, count);
	
	/* Calculate target gains */
	calculate_target_gains(state, direction);
	
	/* Ramp current gain toward target */
	state->tx_gain = ramp_gain(state->tx_gain, state->tx_gain_target, state->ramp_alpha);
	
	/* Apply gain to samples */
	apply_gain(samples, count, state->tx_gain);
	
	/* Update direction state */
	if (direction != state->direction) {
		state->prev_direction = state->direction;
		state->direction = direction;
		state->direction_changes++;
	}
	
	/* Update statistics */
	state->tx_frames++;
	if (direction == SUPP_DIR_DOUBLETALK)
		state->doubletalk_frames++;
}

/* Process RX (near-end) audio samples */
void echo_suppressor_process_rx(echo_suppressor_state_t *state, int16_t *samples, int count)
{
	double tx_energy_for_comparison;
	
	if (!state || !samples || count <= 0)
		return;
	
	/* Calculate energy */
	state->rx_energy = calculate_energy_db(samples, count);
	
	/* Smooth energy */
	state->rx_energy_smooth = smooth_energy(state->rx_energy, state->rx_energy_smooth, state->energy_alpha);
	
	/* Get delayed TX energy for echo detection
	 * In SDR environments, echo arrives delayed by roundtrip time.
	 * We compare RX against TX energy from delay_frames ago.
	 *
	 * Example with 100ms delay (6 frames at 16ms/frame):
	 *   Time:  T0    T1    T2    T3    T4    T5    T6
	 *   TX:    [HI]  [lo]  [lo]  [lo]  [lo]  [lo]  [lo]
	 *   RX:    [lo]  [lo]  [lo]  [lo]  [lo]  [lo]  [ECHO!]
	 *
	 * At T6, we compare RX against TX from T0 (6 frames ago) = [HI]
	 * This correctly identifies the echo and suppresses it.
	 */
	if (state->tx_energy_history && state->delay_frames > 0) {
		/* Read from delay_frames positions behind write position */
		int read_pos = state->history_write_pos - state->delay_frames;
		if (read_pos < 0)
			read_pos += state->history_size;
		state->delayed_tx_energy = state->tx_energy_history[read_pos];
		tx_energy_for_comparison = state->delayed_tx_energy;
	} else {
		/* No delay configured - use current TX energy (original behavior) */
		tx_energy_for_comparison = state->tx_energy_smooth;
	}
	
	/* Determine direction using delayed TX energy
	 * This allows proper echo detection even with SDR roundtrip delay */
	suppressor_direction_t direction = determine_direction(tx_energy_for_comparison, state->rx_energy_smooth,
	                                                        state->threshold_db, state->doubletalk_threshold_db);
	
	/* Update hangover timers */
	update_hangover(state, direction, count);
	
	/* Calculate target gains */
	calculate_target_gains(state, direction);
	
	/* Ramp current gain toward target */
	state->rx_gain = ramp_gain(state->rx_gain, state->rx_gain_target, state->ramp_alpha);
	
	/* Apply gain to samples */
	apply_gain(samples, count, state->rx_gain);
	
	/* Update direction state */
	if (direction != state->direction) {
		state->prev_direction = state->direction;
		state->direction = direction;
		state->direction_changes++;
	}
	
	/* Update statistics */
	state->rx_frames++;
	if (direction == SUPP_DIR_DOUBLETALK)
		state->doubletalk_frames++;
}

/* Get statistics from echo suppressor */
const echo_suppressor_state_t *echo_suppressor_get_stats(echo_suppressor_state_t *state)
{
	return state;
}

/* Cleanup echo suppressor */
void echo_suppressor_cleanup(echo_suppressor_state_t *state)
{
	if (state) {
		if (state->tx_energy_history)
			free(state->tx_energy_history);
		free(state);
	}
}

/* Get human-readable direction name */
const char *echo_suppressor_direction_name(suppressor_direction_t direction)
{
	switch (direction) {
	case SUPP_DIR_NONE:
		return "NONE";
	case SUPP_DIR_TX:
		return "TX";
	case SUPP_DIR_RX:
		return "RX";
	case SUPP_DIR_DOUBLETALK:
		return "DOUBLETALK";
	default:
		return "UNKNOWN";
	}
}
