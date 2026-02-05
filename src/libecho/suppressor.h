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
 * TX = far-end signal (reference) - passes through unmodified
 * RX = near-end signal - suppressed when TX is active (echo removal)
 */

#ifndef _ECHO_SUPPRESSOR_H
#define _ECHO_SUPPRESSOR_H

#include <stdint.h>

/* Suppressor state (simplified) */
typedef enum {
	SUPP_DIR_NONE = 0,      /* TX silent, RX passes */
	SUPP_DIR_TX,            /* TX active, RX suppressed */
} suppressor_direction_t;

/* Echo suppressor state */
typedef struct {
	/* Energy detection */
	double tx_energy;           /* Current TX energy (dB) */
	double rx_energy;           /* Current RX energy (dB) */
	double tx_energy_smooth;    /* Smoothed TX energy */
	double rx_energy_smooth;    /* Smoothed RX energy */
	
	/* TX energy delay buffer for SDR roundtrip compensation */
	double *tx_energy_history;  /* Circular buffer of TX energy values */
	int history_size;           /* Buffer size in frames */
	int history_write_pos;      /* Current write position */
	int delay_frames;           /* Echo delay in frames */
	double delayed_tx_energy;   /* TX energy from delay_frames ago */
	
	/* Suppression state */
	int tx_hangover;            /* Hangover counter (samples) */
	double rx_gain;             /* Current RX gain (0.0-1.0) */
	double rx_gain_target;      /* Target RX gain */
	
	/* Configuration */
	double threshold_db;        /* TX energy threshold for suppression */
	double attenuation_db;      /* RX attenuation depth */
	double attenuation_linear;  /* Attenuation as linear gain */
	int hangover_samples;       /* Hangover duration in samples */
	double attack_alpha;        /* Attack ramp smoothing factor (fast) */
	double release_alpha;       /* Release ramp smoothing factor (slow) */
	double energy_alpha;        /* Energy smoothing factor */
	
	/* Statistics */
	unsigned long tx_frames;
	unsigned long rx_frames;
	unsigned long suppressed_frames;
} echo_suppressor_state_t;

/* Configuration structure
 * Based on Bell/CCITT standard parameters:
 * - Attenuation: 35-45 dB (typical ~40 dB)
 * - Attack time: 1-5 ms
 * - Hold time: 150-300 ms
 * - Release time: 200-500 ms
 * - Detection threshold: -30 to -36 dBm0
 */
typedef struct {
	int enabled;                /* Enable echo suppressor */
	double threshold_db;        /* TX threshold for suppression (Bell/CCITT: -30 to -36 dBm0) */
	double attenuation_db;      /* RX attenuation depth (Bell/CCITT: 35-45 dB) */
	int hangover_ms;            /* Hold time after TX stops (Bell/CCITT: 150-300 ms) */
	int ramp_ms;                /* Attack time (Bell/CCITT: 1-5 ms) */
	int release_ms;             /* Release time (Bell/CCITT: 200-500 ms) */
	double doubletalk_threshold_db; /* Unused - kept for compatibility */
	int stats_enabled;          /* Enable statistics logging */
	int echo_delay_ms;          /* SDR echo delay (default: 0, max: 500) */
} echo_suppressor_config_t;

/* Initialize echo suppressor */
echo_suppressor_state_t *echo_suppressor_init(int sample_rate, int frame_size, 
                                               const echo_suppressor_config_t *config);

/* Process TX (far-end) audio - reference signal, passes through unmodified */
void echo_suppressor_process_tx(echo_suppressor_state_t *state, int16_t *samples, int count);

/* Process RX (near-end) audio - suppressed when TX is active */
void echo_suppressor_process_rx(echo_suppressor_state_t *state, int16_t *samples, int count);

/* Get statistics */
const echo_suppressor_state_t *echo_suppressor_get_stats(echo_suppressor_state_t *state);

/* Cleanup */
void echo_suppressor_cleanup(echo_suppressor_state_t *state);

/* Get state name */
const char *echo_suppressor_direction_name(suppressor_direction_t direction);

#endif /* _ECHO_SUPPRESSOR_H */
