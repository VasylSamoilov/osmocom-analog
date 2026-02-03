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
 */

#ifndef _ECHO_SUPPRESSOR_H
#define _ECHO_SUPPRESSOR_H

#include <stdint.h>

/* Direction state */
typedef enum {
	SUPP_DIR_NONE = 0,      /* No dominant direction */
	SUPP_DIR_TX,            /* TX (far-end) dominant */
	SUPP_DIR_RX,            /* RX (near-end) dominant */
	SUPP_DIR_DOUBLETALK     /* Both directions active */
} suppressor_direction_t;

/* Echo suppressor state */
typedef struct {
	/* Energy detection */
	double tx_energy;           /* Current TX energy (RMS) */
	double rx_energy;           /* Current RX energy (RMS) */
	double tx_energy_smooth;    /* Smoothed TX energy */
	double rx_energy_smooth;    /* Smoothed RX energy */
	
	/* TX energy delay buffer for SDR roundtrip compensation
	 * In SDR environments, echo arrives delayed (50-200ms+ roundtrip).
	 * We store TX energy history and compare RX against delayed TX energy
	 * to properly detect echo that arrives after the original TX. */
	double *tx_energy_history;  /* Circular buffer of TX energy values */
	int history_size;           /* Buffer size in frames */
	int history_write_pos;      /* Current write position */
	int delay_frames;           /* Echo delay in frames (from config) */
	double delayed_tx_energy;   /* TX energy from delay_frames ago */
	
	/* Direction state */
	suppressor_direction_t direction;
	suppressor_direction_t prev_direction;
	
	/* Hangover timers (in samples) */
	int tx_hangover;            /* TX direction hangover counter */
	int rx_hangover;            /* RX direction hangover counter */
	
	/* Gain control */
	double tx_gain;             /* Current TX path gain (0.0-1.0) */
	double rx_gain;             /* Current RX path gain (0.0-1.0) */
	double tx_gain_target;      /* Target TX gain */
	double rx_gain_target;      /* Target RX gain */
	
	/* Configuration */
	double threshold_db;        /* Energy threshold for direction decision (dB) */
	double attenuation_db;      /* Attenuation depth (40-60 dB) */
	double attenuation_linear;  /* Attenuation as linear gain */
	int hangover_samples;       /* Hangover duration in samples */
	double ramp_alpha;          /* Gain ramp smoothing factor */
	double energy_alpha;        /* Energy smoothing factor */
	double doubletalk_threshold_db; /* Threshold for double-talk detection */
	
	/* Statistics */
	unsigned long tx_frames;
	unsigned long rx_frames;
	unsigned long direction_changes;
	unsigned long doubletalk_frames;
} echo_suppressor_state_t;

/* Configuration structure */
typedef struct {
	int enabled;                    /* Enable echo suppressor */
	double threshold_db;            /* Direction threshold (default: 6.0) */
	double attenuation_db;          /* Attenuation depth (default: 50.0) */
	int hangover_ms;                /* Hangover time (default: 100) */
	int ramp_ms;                    /* Gain ramp time (default: 5) */
	double doubletalk_threshold_db; /* Double-talk threshold (default: 3.0) */
	int stats_enabled;              /* Enable statistics logging */
	int echo_delay_ms;              /* SDR echo roundtrip delay (default: 0, max: 500) */
} echo_suppressor_config_t;

/* Initialize echo suppressor
 * sample_rate: Audio sample rate (typically 8000 Hz)
 * frame_size: Frame size in samples (typically 128 for 16ms @ 8kHz)
 * config: Configuration parameters
 * Returns: Pointer to initialized state, or NULL on error
 */
echo_suppressor_state_t *echo_suppressor_init(int sample_rate, int frame_size, 
                                               const echo_suppressor_config_t *config);

/* Process TX (far-end) audio samples
 * state: Echo suppressor state
 * samples: Audio samples (int16_t format)
 * count: Number of samples
 */
void echo_suppressor_process_tx(echo_suppressor_state_t *state, int16_t *samples, int count);

/* Process RX (near-end) audio samples
 * state: Echo suppressor state
 * samples: Audio samples (int16_t format)
 * count: Number of samples
 */
void echo_suppressor_process_rx(echo_suppressor_state_t *state, int16_t *samples, int count);

/* Get statistics from echo suppressor
 * state: Echo suppressor state
 * Returns: Pointer to state structure (read-only)
 */
const echo_suppressor_state_t *echo_suppressor_get_stats(echo_suppressor_state_t *state);

/* Cleanup echo suppressor
 * state: Echo suppressor state to free
 */
void echo_suppressor_cleanup(echo_suppressor_state_t *state);

/* Get human-readable direction name */
const char *echo_suppressor_direction_name(suppressor_direction_t direction);

#endif /* _ECHO_SUPPRESSOR_H */
