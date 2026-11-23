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
#define MAX_ECHO_DELAY_MS 300	/* round trip of a usable setup: 50..250 ms */

/* The TX energy is kept in slots of this duration. The blocks that are given to process_tx() and
 * process_rx() have any size (the main loop gives what the sound device or SDR needs), and
 * all times are counted in samples. */
#define ENERGY_SLOT_MS 4

/* Longest time TX may be ahead of RX or behind them, because of the sizes of the blocks */
#define MAX_BLOCK_SKEW_MS 250

/* Measurement of the echo delay: the envelopes (RMS per slot) of TX and of RX before the suppression are
 * compared over the last EST_WINDOW_MS, for lags 0..MAX_ECHO_DELAY_MS, every EST_INTERVAL_MS. The lag with the
 * highest correlation is the delay, if the correlation is at least EST_MIN_CORR and TX had speech. */
#define EST_SLOTS 1024
#define EST_WINDOW_MS 2000
#define EST_INTERVAL_MS 1000
#define EST_MIN_CORR 0.7
#define EST_MIN_TX_DB -45.0

static void set_delay(echo_suppressor_state_t *state, int echo_delay_ms, int ramp_ms)
{
	long long delay_samples = ((long long)echo_delay_ms * state->sample_rate) / 1000;
	int attack_samples = (ramp_ms * state->sample_rate) / 1000;

	/* Start suppression early so the gain ramp has reached the full attenuation when the echo
	 * arrives. The gain falls exponentially with the attack time as time constant, so it takes
	 * ln(1 / attenuation) time constants. */
	if (attack_samples > 0 && state->attenuation_linear > 0.0 && state->attenuation_linear < 1.0)
		delay_samples -= (long long)((double)attack_samples * log(1.0 / state->attenuation_linear));
	if (delay_samples < 0)
		delay_samples = 0;
	state->delay_samples = delay_samples;
	state->delay_frames = (int)(state->delay_samples / state->slot_samples);
}

/* the lag (in slots) of the best match of the envelopes, returns the correlation, or -1 if TX had no speech */
static double estimate_delay(echo_suppressor_state_t *state, int *lag_slots)
{
	int window = EST_WINDOW_MS / ENERGY_SLOT_MS, max_lag = MAX_ECHO_DELAY_MS / ENERGY_SLOT_MS, lag, k;
	long long last = state->rx_slots - 1, first = state->rx_slots - window;
	double best = -1.0, tx_peak = 0.0;

	if (first - max_lag < 0 || state->slots_written < state->rx_slots - max_lag)
		return -1.0;
	for (k = 0; k < window + max_lag; k++) {
		long long s = last - k;

		if (s >= 0 && s < state->slots_written && state->est_tx[s % EST_SLOTS] > tx_peak)
			tx_peak = state->est_tx[s % EST_SLOTS];
	}
	if (20.0 * log10(tx_peak + 1e-12) < EST_MIN_TX_DB)
		return -1.0;
	for (lag = 0; lag <= max_lag; lag++) {
		double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, cov, var;
		int n = 0;
		long long s;

		for (s = first; s <= last; s++) {
			long long t = s - lag;
			double x, y;

			if (t < 0 || t >= state->slots_written)
				continue;
			x = state->est_tx[t % EST_SLOTS];
			y = state->est_rx[s % EST_SLOTS];
			sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
			n++;
		}
		if (n < window / 2)
			continue;
		cov = sxy - sx * sy / n;
		var = (sxx - sx * sx / n) * (syy - sy * sy / n);
		if (var <= 0.0)
			continue;
		if (cov / sqrt(var) > best) {
			best = cov / sqrt(var);
			*lag_slots = lag;
		}
	}
	return best;
}

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

/* Energy of the sum of squares of normalized samples in dB */
static double slot_energy_db(double sum, int count)
{
	double rms;

	if (count <= 0)
		return -100.0;
	rms = sqrt(sum / count);
	if (rms < 1e-10)
		return -100.0;
	return 20.0 * log10(rms);
}

/* Apply exponential smoothing to energy value */
static double smooth_energy(double current, double previous, double alpha)
{
	return alpha * current + (1.0 - alpha) * previous;
}

/* Apply the gain to the samples, moving it to the target for every sample. The rates are per
 * sample: fast attack when suppressing, slow release when opening. */
static double apply_gain_ramp(int16_t *samples, int count, double gain, double target, double attack_alpha, double release_alpha)
{
	double alpha = (target < gain) ? attack_alpha : release_alpha;
	int i;

	for (i = 0; i < count; i++) {
		double s;

		gain += alpha * (target - gain);
		s = rint((double)samples[i] * gain);
		if (s > 32767.0)
			s = 32767.0;
		if (s < -32768.0)
			s = -32768.0;
		samples[i] = (int16_t)s;
	}
	return gain;
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
	
	/* Initialize TX energy history for SDR roundtrip compensation: slots of a fixed duration, long
	 * enough for the longest delay and the skew between the TX and the RX blocks */
	echo_delay_ms = config->echo_delay_ms;
	if (echo_delay_ms < 0)
		echo_delay_ms = 0;
	if (echo_delay_ms > MAX_ECHO_DELAY_MS)
		echo_delay_ms = MAX_ECHO_DELAY_MS;
	
	state->slot_samples = (ENERGY_SLOT_MS * sample_rate) / 1000;
	if (state->slot_samples < 1)
		state->slot_samples = 1;
	state->history_size = ((MAX_ECHO_DELAY_MS + MAX_BLOCK_SKEW_MS) / ENERGY_SLOT_MS) + 2;
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
	state->doubletalk_db = config->doubletalk_threshold_db;
	state->near_hangover = 0;
	state->near_slot = -1000000;
	state->attenuation_db = config->attenuation_db;
	state->attenuation_linear = pow(10.0, -config->attenuation_db / 20.0);
	state->hangover_samples = (config->hangover_ms * sample_rate) / 1000;
	
	/* Energy smoothing factor */
	state->energy_alpha = 0.3;
	
	/* Attack time (Bell/CCITT: 1-5ms) - fast to catch plosives. The rates are per sample. */
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
	
	/* The delay with attack compensation, measured during the call unless it was given */
	state->sample_rate = sample_rate;
	state->ramp_ms = config->ramp_ms;
	set_delay(state, echo_delay_ms, config->ramp_ms);
	state->auto_delay = !config->echo_delay_given;
	state->est_delay_ms = -1;
	state->est_next = EST_INTERVAL_MS / ENERGY_SLOT_MS;
	state->est_tx = calloc(EST_SLOTS, sizeof(double));
	state->est_rx = calloc(EST_SLOTS, sizeof(double));
	if (!state->est_tx || !state->est_rx) {
		LOGP(DECHO, LOGL_ERROR, "Failed to allocate echo suppressor delay measurement\n");
		free(state->est_tx);
		free(state->est_rx);
		free(state->tx_energy_history);
		free(state);
		return NULL;
	}
	
	/* Statistics */
	state->tx_frames = 0;
	state->rx_frames = 0;
	state->suppressed_frames = 0;
	
	LOGP(DECHO, LOGL_INFO, "Echo suppressor initialized (Bell/CCITT params): delay=%dms%s thresh=%.1fdB att=%.1fdB hold=%dms attack=%dms release=%dms\n",
	     echo_delay_ms, state->auto_delay ? " (until measured)" : "", state->threshold_db, state->attenuation_db,
	     config->hangover_ms, config->ramp_ms, release_ms);

	return state;
}

/* Process TX (far-end) audio - reference signal, never modified */
void echo_suppressor_process_tx(echo_suppressor_state_t *state, int16_t *samples, int count)
{
	int i;

	if (!state || !samples || count <= 0)
		return;
	
	/* Energy of the block (for the statistics) */
	state->tx_energy = calculate_energy_db(samples, count);
	state->tx_energy_smooth = smooth_energy(state->tx_energy, state->tx_energy_smooth, state->energy_alpha);
	
	/* Store the energy in slots of a fixed duration, for the delayed comparison */
	for (i = 0; i < count; i++) {
		double s = (double)samples[i] / 32768.0;

		state->slot_acc += s * s;
		if (++state->slot_count == state->slot_samples) {
			state->tx_energy_history[state->history_write_pos] = slot_energy_db(state->slot_acc, state->slot_count);
			state->history_write_pos = (state->history_write_pos + 1) % state->history_size;
			state->est_tx[state->slots_written % EST_SLOTS] = sqrt(state->slot_acc / state->slot_count);
			state->slots_written++;
			state->slot_acc = 0.0;
			state->slot_count = 0;
		}
	}
	state->tx_total += count;
	
	/* TX passes through unmodified - it's the reference signal */
	state->tx_frames++;
}

/* The highest TX energy of the slots that this block of RX audio is compared with: the slots
 * of the time span of the block, the echo delay ago. */
static double tx_reference_energy(echo_suppressor_state_t *state, int count)
{
	long long first, last, slot;
	double energy = -100.0;

	/* the time span of the block on the TX time line, in slots; while the delay is measured but not yet
	 * known, any delay up to MAX_ECHO_DELAY_MS is possible, so all of them are taken */
	first = (state->rx_total - state->delay_samples) / state->slot_samples;
	if (state->auto_delay && state->est_delay_ms < 0) {
		long long d = ((long long)MAX_ECHO_DELAY_MS * state->sample_rate) / 1000;

		first = (state->rx_total > d) ? (state->rx_total - d) / state->slot_samples : 0;
		last = (state->rx_total + count - 1) / state->slot_samples;
		if (last > state->slots_written - 1)
			last = state->slots_written - 1;
		if (first < state->slots_written - state->history_size + 1)
			first = state->slots_written - state->history_size + 1;
		if (first < 0)
			first = 0;
		for (slot = first; slot <= last; slot++) {
			double e = state->tx_energy_history[slot % state->history_size];

			if (e > energy)
				energy = e;
		}
		return energy;
	}
	last = (state->rx_total + count - 1 - state->delay_samples) / state->slot_samples;
	if (state->rx_total < state->delay_samples)
		return -100.0;
	/* TX may not have got that far yet, or too far to remember it */
	if (last > state->slots_written - 1)
		last = state->slots_written - 1;
	if (first < state->slots_written - state->history_size + 1)
		first = state->slots_written - state->history_size + 1;
	if (first < 0)
		first = 0;
	for (slot = first; slot <= last; slot++) {
		double e = state->tx_energy_history[slot % state->history_size];

		if (e > energy)
			energy = e;
	}
	return energy;
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

	/* Envelope of RX before the suppression, for the measurement of the echo delay */
	{
		int i;

		for (i = 0; i < count; i++) {
			double s = (double)samples[i] / 32768.0;

			state->est_rx_acc += s * s;
			if (++state->est_rx_count == state->slot_samples) {
				state->est_rx[state->rx_slots % EST_SLOTS] = sqrt(state->est_rx_acc / state->est_rx_count);
				state->rx_slots++;
				state->est_rx_acc = 0.0;
				state->est_rx_count = 0;
			}
		}
		if (state->rx_slots >= state->est_next && state->rx_slots - state->near_slot > EST_WINDOW_MS / ENERGY_SLOT_MS) {
			int lag = 0, ms;
			double corr;

			state->est_next = state->rx_slots + EST_INTERVAL_MS / ENERGY_SLOT_MS;
			corr = estimate_delay(state, &lag);
			ms = lag * ENERGY_SLOT_MS;
			if (corr >= EST_MIN_CORR) {
				if (state->est_delay_ms < 0 || abs(ms - state->est_delay_ms) > ENERGY_SLOT_MS) {
					LOGP(DECHO, LOGL_NOTICE, "Echo delay measured: %d ms (correlation %.2f)%s\n", ms, corr,
					     state->auto_delay ? ", using it" : ", not used, because the delay was given");
					if (state->auto_delay)
						set_delay(state, ms, state->ramp_ms);
				}
				state->est_delay_ms = ms;
				state->est_corr = corr;
			} else if (corr >= 0.0)
				LOGP(DECHO, LOGL_DEBUG, "Echo delay not clear: best %d ms with correlation %.2f\n", ms, corr);
		}
	}
	
	/* TX reference energy, the echo delay ago */
	tx_ref_energy = tx_reference_energy(state, count);
	state->delayed_tx_energy = tx_ref_energy;
	state->rx_total += count;
	
	/* Update hangover: if TX has significant energy, set hangover */
	if (tx_ref_energy > state->threshold_db) {
		state->tx_hangover = state->hangover_samples;
	}
	
	/* Decay hangover */
	state->tx_hangover = MAX(0, state->tx_hangover - count);
	
	/* Double talk: the echo of the phone returns at least 28 dB below the far end (measured on air, NMT and
	 * AMPS phones on the table); RX louder than the far end minus doubletalk_db is the near end talking.
	 * Then RX passes (break-in) for the hangover time, and the delay measurement pauses. */
	if (state->doubletalk_db > 0.0 && state->rx_energy > -60.0 && state->rx_energy > tx_ref_energy - state->doubletalk_db) {
		state->near_hangover = state->hangover_samples;
		state->near_slot = state->rx_slots;
	}
	state->near_hangover = MAX(0, state->near_hangover - count);

	/* Determine RX gain target */
	if (state->near_hangover > 0) {
		/* near end talks - pass RX through */
		state->rx_gain_target = 1.0;
	} else if (state->tx_hangover > 0) {
		/* TX active - suppress RX (potential echo) */
		state->rx_gain_target = state->attenuation_linear;
		state->suppressed_frames++;
	} else {
		/* TX silent - pass RX through */
		state->rx_gain_target = 1.0;
	}
	
	/* Smooth gain transition per sample (fast attack, slow release per Bell/CCITT) */
	/* the near end breaking in opens as fast as the suppression closes, so its first syllable is not lost */
	state->rx_gain = apply_gain_ramp(samples, count, state->rx_gain, state->rx_gain_target,
	                                 state->attack_alpha, (state->near_hangover > 0) ? state->attack_alpha : state->release_alpha);
	
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
		free(state->est_tx);
		free(state->est_rx);
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
