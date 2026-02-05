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
 * UNIDIRECTIONAL ECHO SUPPRESSOR
 * ==============================
 * This is a classical unidirectional echo suppressor:
 *   - TX = far-end signal (from SIP/network) - REFERENCE, never modified
 *   - RX = near-end signal (from mobile) - may contain echo, suppressed when TX active
 *
 * Operation:
 *   1. Monitor TX energy level
 *   2. When TX has speech, suppress RX (attenuate potential echo)
 *   3. Hangover timer keeps suppression active briefly after TX stops
 *   4. TX always passes through unmodified
 *
 * SDR ECHO DELAY COMPENSATION
 * ===========================
 * In SDR environments, echo arrives delayed (50-200ms+). We store TX energy
 * history and compare RX against delayed TX energy for proper echo detection.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "suppressor.h"
#include "../liblogging/logging.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Maximum echo delay we support (500ms should cover most SDR setups) */
#define MAX_ECHO_DELAY_MS 500

/* Calculate RMS energy of audio samples in dB */
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

/* Apply exponential smoothing to energy value */
static double smooth_energy(double current, double previous, double alpha)
{
	return alpha * current + (1.0 - alpha) * previous;
}

/* Apply smooth gain ramping with separate attack/release rates */
static double ramp_gain(double current_gain, double target_gain, double attack_alpha, double release_alpha)
{
	double alpha;
	/* Use fast attack when suppressing, slow release when opening */
	if (target_gain < current_gain)
		alpha = attack_alpha;   /* Suppressing - fast */
	else
		alpha = release_alpha;  /* Releasing - slow */
	
	return alpha * target_gain + (1.0 - alpha) * current_gain;
}

/* Apply gain to audio samples */
static void apply_gain(int16_t *samples, int count, double gain)
{
	int i;
	
	if (gain >= 0.999)
		return;  /* No change needed */
	
	if (gain < 0.0)
		gain = 0.0;
	
	for (i = 0; i < count; i++) {
		double s = (double)samples[i] * gain;
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
	
	/* Initialize TX energy delay buffer for SDR roundtrip compensation */
	echo_delay_ms = config->echo_delay_ms;
	if (echo_delay_ms < 0)
		echo_delay_ms = 0;
	if (echo_delay_ms > MAX_ECHO_DELAY_MS)
		echo_delay_ms = MAX_ECHO_DELAY_MS;
	
	int frame_duration_ms = (frame_size * 1000) / sample_rate;
	if (frame_duration_ms < 1)
		frame_duration_ms = 1;
	
	state->history_size = (MAX_ECHO_DELAY_MS / frame_duration_ms) + 2;
	state->tx_energy_history = calloc(state->history_size, sizeof(double));
	if (!state->tx_energy_history) {
		LOGP(DECHO, LOGL_ERROR, "Failed to allocate echo suppressor history buffer\n");
		free(state);
		return NULL;
	}
	
	for (int i = 0; i < state->history_size; i++)
		state->tx_energy_history[i] = -100.0;
	
	state->history_write_pos = 0;
	state->delayed_tx_energy = -100.0;
	
	/* Initialize state */
	state->tx_hangover = 0;
	state->rx_gain = 1.0;
	state->rx_gain_target = 1.0;
	
	/* Set configuration */
	state->threshold_db = config->threshold_db;
	state->attenuation_db = config->attenuation_db;
	state->attenuation_linear = pow(10.0, -config->attenuation_db / 20.0);
	state->hangover_samples = (config->hangover_ms * sample_rate) / 1000;
	
	/* Energy smoothing factor */
	state->energy_alpha = 0.3;
	
	/* Attack time (Bell/CCITT: 1-5ms) - fast to catch plosives */
	int attack_samples = (config->ramp_ms * sample_rate) / 1000;
	if (attack_samples > 0)
		state->attack_alpha = 1.0 / (double)attack_samples;
	else
		state->attack_alpha = 1.0;
	
	/* Release time (Bell/CCITT: 200-500ms) - slow to prevent pumping */
	int release_ms = config->release_ms;
	if (release_ms <= 0)
		release_ms = 300;  /* Default 300ms if not specified */
	int release_samples = (release_ms * sample_rate) / 1000;
	if (release_samples > 0)
		state->release_alpha = 1.0 / (double)release_samples;
	else
		state->release_alpha = 0.01;  /* Very slow default */
	
	/* Calculate delay_frames with attack compensation:
	 * Start suppression early so gain ramp reaches full attenuation when echo arrives.
	 * effective_delay = echo_delay - attack_time */
	int delay_frames = echo_delay_ms / frame_duration_ms;
	int attack_frames = config->ramp_ms / frame_duration_ms;
	int effective_delay = delay_frames - attack_frames;
	if (effective_delay < 0)
		effective_delay = 0;
	state->delay_frames = effective_delay;
	
	/* Statistics */
	state->tx_frames = 0;
	state->rx_frames = 0;
	state->suppressed_frames = 0;
	
	LOGP(DECHO, LOGL_INFO, "Echo suppressor initialized (Bell/CCITT params): delay=%dms thresh=%.1fdB att=%.1fdB hold=%dms attack=%dms release=%dms\n",
	     echo_delay_ms, state->threshold_db, state->attenuation_db, config->hangover_ms, config->ramp_ms, release_ms);

	return state;
}

/* Process TX (far-end) audio - reference signal, never modified */
void echo_suppressor_process_tx(echo_suppressor_state_t *state, int16_t *samples, int count)
{
	if (!state || !samples || count <= 0)
		return;
	
	/* Calculate and store TX energy */
	state->tx_energy = calculate_energy_db(samples, count);
	state->tx_energy_smooth = smooth_energy(state->tx_energy, state->tx_energy_smooth, state->energy_alpha);
	
	/* Store in history buffer for delayed comparison */
	if (state->tx_energy_history) {
		state->tx_energy_history[state->history_write_pos] = state->tx_energy;
		state->history_write_pos = (state->history_write_pos + 1) % state->history_size;
	}
	
	/* TX passes through unmodified - it's the reference signal */
	state->tx_frames++;
}

/* Process RX (near-end) audio - suppress when TX is active */
void echo_suppressor_process_rx(echo_suppressor_state_t *state, int16_t *samples, int count)
{
	double tx_ref_energy;
	
	if (!state || !samples || count <= 0)
		return;
	
	/* Calculate RX energy */
	state->rx_energy = calculate_energy_db(samples, count);
	state->rx_energy_smooth = smooth_energy(state->rx_energy, state->rx_energy_smooth, state->energy_alpha);
	
	/* Get TX reference energy (delayed if configured) */
	if (state->tx_energy_history && state->delay_frames > 0) {
		int read_pos = state->history_write_pos - state->delay_frames;
		if (read_pos < 0)
			read_pos += state->history_size;
		state->delayed_tx_energy = state->tx_energy_history[read_pos];
		tx_ref_energy = state->delayed_tx_energy;
	} else {
		tx_ref_energy = state->tx_energy_smooth;
	}
	
	/* Update hangover: if TX has significant energy, set hangover */
	if (tx_ref_energy > state->threshold_db) {
		state->tx_hangover = state->hangover_samples;
	}
	
	/* Decay hangover */
	state->tx_hangover = MAX(0, state->tx_hangover - count);
	
	/* Determine RX gain target */
	if (state->tx_hangover > 0) {
		/* TX active - suppress RX (potential echo) */
		state->rx_gain_target = state->attenuation_linear;
		state->suppressed_frames++;
	} else {
		/* TX silent - pass RX through */
		state->rx_gain_target = 1.0;
	}
	
	/* Smooth gain transition (fast attack, slow release per Bell/CCITT) */
	state->rx_gain = ramp_gain(state->rx_gain, state->rx_gain_target, 
	                           state->attack_alpha, state->release_alpha);
	
	/* Apply gain to RX samples */
	apply_gain(samples, count, state->rx_gain);
	
	state->rx_frames++;
}

/* Get statistics */
const echo_suppressor_state_t *echo_suppressor_get_stats(echo_suppressor_state_t *state)
{
	return state;
}

/* Cleanup */
void echo_suppressor_cleanup(echo_suppressor_state_t *state)
{
	if (state) {
		if (state->tx_energy_history)
			free(state->tx_energy_history);
		free(state);
	}
}

/* Direction name (simplified - only TX active or not) */
const char *echo_suppressor_direction_name(suppressor_direction_t direction)
{
	switch (direction) {
	case SUPP_DIR_NONE:
		return "IDLE";
	case SUPP_DIR_TX:
		return "TX_ACTIVE";
	default:
		return "UNKNOWN";
	}
}
