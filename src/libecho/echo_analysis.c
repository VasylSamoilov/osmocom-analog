/* Echo Delay Analysis for Analog Telephony
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "echo_analysis.h"
#include "../libgoertzel/goertzel.h"
#include "../libsample/sample.h"
#include "../liblogging/logging.h"

/* Test frequency grid: 600Hz to 2850Hz, 150Hz spacing, omitting 900, 1200, 1800, 2400 Hz */
/* Note: High frequencies (3000-3750 Hz) and low frequencies (150-450 Hz) excluded - 
 * not detected with sufficient SNR on typical analog telephony channels */
#define ECHO_ANALYSIS_NUM_FREQS 21
#define ECHO_ANALYSIS_SAMPLE_RATE 8000
#define ECHO_ANALYSIS_FRAME_SIZE 160  /* 20ms at 8kHz */
#define SILENCE_DURATION_SECONDS 3
#define SILENCE_FRAMES (SILENCE_DURATION_SECONDS * ECHO_ANALYSIS_SAMPLE_RATE / ECHO_ANALYSIS_FRAME_SIZE)

/* Chirp configuration */
#define CHIRP_DURATION_MS 50          /* 50ms chirp duration */
#define CHIRP_DURATION_SAMPLES (CHIRP_DURATION_MS * ECHO_ANALYSIS_SAMPLE_RATE / 1000)
#define CHIRP_START_FREQ 600          /* Start frequency in Hz */
#define CHIRP_END_FREQ 2850           /* End frequency in Hz */
#define NUM_CHIRPS 10                 /* Number of chirps to send */
#define CHIRP_INTERVAL_MS 500         /* 500ms between chirps */
#define CHIRP_INTERVAL_FRAMES (CHIRP_INTERVAL_MS * ECHO_ANALYSIS_SAMPLE_RATE / 1000 / ECHO_ANALYSIS_FRAME_SIZE)  /* Frames between chirps */
#define MAX_DELAY_SAMPLES 4000        /* Max delay to search: 500ms at 8kHz */

typedef enum {
    PHASE_SILENCE,                    /* Initial silence period */
    PHASE_CHIRP,                      /* Chirp-based delay measurement */
    PHASE_TONE,                       /* Tone-based frequency response */
    PHASE_COMPLETE                    /* Analysis complete */
} analysis_phase_t;

/* Full frequency grid (Hz) - for reference:
 * 150, 300, 450, 600, 750, 1050, 1350, 1500, 1650, 1950,
 * 2100, 2250, 2550, 2700, 2850, 3000, 3150, 3300, 3450, 3600, 3750
 */

/* Active frequency grid (Hz) */
static const int echo_analysis_freqs[ECHO_ANALYSIS_NUM_FREQS] = {
    150, 300, 450, 600, 750, 1050, 1350, 1500, 1650, 1950,
    2100, 2250, 2550, 2700, 2850, 3000, 3150, 3300, 3450, 3600, 3750
};

/* Pre-computed amplitude table for speech-equivalent spectrum */
/* Based on -6dB/octave rolloff above 1kHz with formant boost */
static const double speech_amplitude[ECHO_ANALYSIS_NUM_FREQS] = {
    /* 150 Hz */ 0.120,
    /* 300 Hz */ 0.120,
    /* 450 Hz */ 0.120,
    /* 600 Hz */ 0.120,  /* F1 region boost */
    /* 750 Hz */ 0.120,  /* F1 region boost */
    /* 1050 Hz */ 0.095,
    /* 1350 Hz */ 0.078,
    /* 1500 Hz */ 0.067,
    /* 1650 Hz */ 0.061,
    /* 1950 Hz */ 0.052,
    /* 2100 Hz */ 0.048,
    /* 2250 Hz */ 0.045,
    /* 2550 Hz */ 0.039,
    /* 2700 Hz */ 0.037,
    /* 2850 Hz */ 0.035,
    /* 3000 Hz */ 0.033,
    /* 3150 Hz */ 0.031,
    /* 3300 Hz */ 0.029,
    /* 3450 Hz */ 0.027,
    /* 3600 Hz */ 0.025,
    /* 3750 Hz */ 0.023
};

/* Tone combination for a single test */
typedef struct {
    int sequence;                           /* Sequence number for correlation */
    int num_tones;                          /* Number of tones in combination (3-5) */
    int freq_indices[5];                    /* Indices into echo_analysis_freqs */
    uint64_t tx_timestamp_ms;               /* TX timestamp in milliseconds */
    uint64_t rx_timestamp_ms;               /* RX timestamp (0 if not received) */
    double tx_levels_dbm[5];                /* TX levels per tone */
    double rx_levels_dbm[5];                /* RX levels per tone */
    double noise_floor_dbm;                 /* Noise from non-transmitted frequencies */
    int detected;                           /* 1 if echo detected */
} echo_tone_combination_t;

/* Per-frequency measurement results */
typedef struct {
    int freq_hz;                            /* Frequency in Hz */
    int num_measurements;                   /* Number of successful measurements */
    double sum_tx_level_dbm;                /* Sum of TX levels */
    double sum_rx_level_dbm;                /* Sum of RX levels */
    double sum_delay_ms;                    /* Sum of delays */
    double sum_delay_sq_ms;                 /* Sum of squared delays (for stddev) */
} echo_freq_result_t;

/* Chirp delay measurement result */
typedef struct {
    int chirp_number;                       /* Chirp sequence number (1-5) */
    uint64_t tx_timestamp_ms;               /* TX timestamp */
    uint64_t rx_timestamp_ms;               /* RX timestamp (0 if not detected) */
    double delay_ms;                        /* Measured delay */
    int detected;                           /* 1 if chirp was detected */
} chirp_result_t;

/* Main echo analysis state */
struct echo_analysis_state {
    /* Configuration */
    int enabled;                            /* Analysis mode enabled */
    int duration_sec;                       /* Test duration in seconds */
    int tones_per_combination;              /* 3-5 tones per combination */
    
    /* Analysis phase */
    analysis_phase_t phase;                 /* Current analysis phase */
    
    /* Goertzel filters for all frequencies */
    goertzel_t goertzel[ECHO_ANALYSIS_NUM_FREQS];
    
    /* Tone generation state */
    int current_combination;                /* Current combination index */
    int combination_frame_count;            /* Frames in current combination */
    int frames_per_combination;             /* Frames to hold each combination */
    double tone_phase[5];                   /* Phase accumulators for tone generation */
    
    /* Initial silence period */
    int silence_frames;                     /* Frames of silence at start */
    int silence_frame_count;                /* Current silence frame count */
    int silence_complete;                   /* 1 if silence period is done */
    
    /* Chirp state */
    int current_chirp;                      /* Current chirp number (0-4) */
    int chirp_frame_count;                  /* Frames since chirp start */
    sample_t *chirp_reference;              /* Reference chirp signal */
    sample_t *rx_buffer;                    /* RX buffer for correlation */
    int rx_buffer_pos;                      /* Current position in RX buffer */
    int rx_buffer_size;                     /* Size of RX buffer */
    chirp_result_t chirp_results[NUM_CHIRPS]; /* Chirp measurement results */
    int chirp_tx_active;                    /* 1 if currently transmitting chirp */
    uint64_t chirp_tx_start_ms;             /* Chirp TX start timestamp */
    int chirp_tx_sample[NUM_CHIRPS];        /* Sample number when each chirp TX started */
    int total_samples_processed;            /* Total samples processed in chirp phase */
    
    /* Current test combination */
    echo_tone_combination_t current;
    
    /* Results storage */
    echo_tone_combination_t *results;       /* Array of all combinations */
    int num_results;                        /* Number of results stored */
    int max_results;                        /* Maximum results capacity */
    
    /* Per-frequency aggregated results */
    echo_freq_result_t freq_results[ECHO_ANALYSIS_NUM_FREQS];
    
    /* Overall statistics */
    double total_tx_level_dbm;
    double total_rx_level_dbm;
    double total_noise_dbm;
    int total_measurements;
    double sum_delay_ms;
    double sum_delay_sq_ms;
    
    /* Timing */
    uint64_t start_time_ms;
    uint64_t last_tx_time_ms;
};

/* Get current time in milliseconds */
static uint64_t get_time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

/* Calculate amplitude for frequency to match speech spectrum */
static double get_speech_equivalent_amplitude(int freq_idx)
{
    if (freq_idx < 0 || freq_idx >= ECHO_ANALYSIS_NUM_FREQS)
        return 0.0;
    return speech_amplitude[freq_idx];
}

/* Generate a chirp signal (frequency sweep) */
static void generate_chirp(sample_t *samples, int count)
{
    int i;
    double phase = 0.0;
    double amplitude = 0.5;  /* Moderate amplitude to avoid clipping */
    
    for (i = 0; i < count; i++) {
        /* Linear frequency sweep from CHIRP_START_FREQ to CHIRP_END_FREQ */
        double t = (double)i / count;  /* 0.0 to 1.0 */
        double freq = CHIRP_START_FREQ + t * (CHIRP_END_FREQ - CHIRP_START_FREQ);
        double phase_inc = 2.0 * M_PI * freq / ECHO_ANALYSIS_SAMPLE_RATE;
        
        samples[i] = amplitude * sin(phase);
        phase += phase_inc;
        while (phase > 2.0 * M_PI)
            phase -= 2.0 * M_PI;
    }
}

/* Cross-correlate received signal with reference chirp to find delay */
static int correlate_chirp(sample_t *reference, sample_t *received, int chirp_len, int search_len)
{
    int delay;
    int best_delay = 0;
    double best_correlation = 0.0;
    double ref_energy = 0.0;
    int i;
    
    /* Calculate reference signal energy for normalization */
    for (i = 0; i < chirp_len; i++) {
        ref_energy += reference[i] * reference[i];
    }
    
    if (ref_energy < 1e-10) {
        return -1;  /* Reference signal has no energy */
    }
    
    /* Search for best correlation across possible delays */
    for (delay = 0; delay < search_len - chirp_len; delay++) {
        double correlation = 0.0;
        double rx_energy = 0.0;
        
        /* Calculate correlation and RX energy at this delay */
        for (i = 0; i < chirp_len; i++) {
            correlation += reference[i] * received[delay + i];
            rx_energy += received[delay + i] * received[delay + i];
        }
        
        /* Normalize correlation by geometric mean of energies */
        if (rx_energy > 1e-10) {
            correlation = correlation / sqrt(ref_energy * rx_energy);
        } else {
            correlation = 0.0;
        }
        
        /* Track best correlation */
        if (correlation > best_correlation) {
            best_correlation = correlation;
            best_delay = delay;
        }
    }
    
    /* Return delay in samples, or -1 if correlation is too weak */
    /* Normalized correlation ranges from -1 to 1, threshold at 0.3 (30% similarity) */
    if (best_correlation < 0.3) {
        return -1;  /* Weak correlation, chirp not detected */
    }
    
    return best_delay;
}

/* Select next tone combination */
static void select_next_combination(echo_analysis_state_t *state)
{
    /* Fixed 3 tones per combination */
    /* Total 21 frequencies. 7 unique combinations. */
    /* Pattern: Interleaved with stride 7 (1050 Hz spacing) */
    /* Round 1: Combos 0-6. Round 2: Combos 0-6. */
    
    int combo_idx = state->current_combination;
    int t;
    int unique_combo_idx = combo_idx % 7;
    
    state->current.sequence = combo_idx + 1;
    state->current.num_tones = 3; /* Always 3 tones */
    state->current.detected = 0;
    state->current.tx_timestamp_ms = 0;
    state->current.rx_timestamp_ms = 0;
    state->current.noise_floor_dbm = -100.0;
    
    /* Select frequencies for this combination */
    /* Tone 0: index 0..6 (150-1050 Hz) */
    state->current.freq_indices[0] = unique_combo_idx;
    /* Tone 1: index 7..13 (1200-2100 Hz) -> actually 7+0..7+6 */
    state->current.freq_indices[1] = unique_combo_idx + 7;
    /* Tone 2: index 14..20 (2250-3150 Hz) -> actually 14+0..14+6 */
    state->current.freq_indices[2] = unique_combo_idx + 14;
    
    for (t = 0; t < 3; t++) {
        state->current.tx_levels_dbm[t] = 0.0;
        state->current.rx_levels_dbm[t] = -100.0;
    }
    
    /* Reset phase accumulators */
    for (t = 0; t < 5; t++) {
        state->tone_phase[t] = 0.0;
    }
}

/* Generate multi-tone signal with speech-equivalent spectrum */
static void generate_tone_combination(echo_analysis_state_t *state, 
                                       sample_t *samples, int count)
{
    int i, t;
    
    /* Calculate total amplitude to check for clipping */
    double total_amplitude = 0.0;
    for (t = 0; t < state->current.num_tones; t++) {
        int freq_idx = state->current.freq_indices[t];
        total_amplitude += get_speech_equivalent_amplitude(freq_idx);
    }
    
    /* Scale factor to prevent clipping (keep peak < 0.9) */
    double scale = (total_amplitude > 0.9) ? (0.9 / total_amplitude) : 1.0;
    
    /* Generate samples */
    for (i = 0; i < count; i++) {
        double sample = 0.0;
        for (t = 0; t < state->current.num_tones; t++) {
            int freq_idx = state->current.freq_indices[t];
            int freq_hz = echo_analysis_freqs[freq_idx];
            double amplitude = get_speech_equivalent_amplitude(freq_idx) * scale;
            double phase_inc = 2.0 * M_PI * freq_hz / ECHO_ANALYSIS_SAMPLE_RATE;
            
            sample += amplitude * sin(state->tone_phase[t]);
            state->tone_phase[t] += phase_inc;
            if (state->tone_phase[t] > 2.0 * M_PI)
                state->tone_phase[t] -= 2.0 * M_PI;
        }
        samples[i] = sample;
    }
}

/* Initialize echo analysis */
echo_analysis_state_t *echo_analysis_init(int duration_sec, int tones_per_combo)
{
    echo_analysis_state_t *state;
    int i;
    
    /* Validate parameters */
    if (duration_sec <= 0)
        duration_sec = 20;  /* Default 20 seconds */
        
    /* Force 3 tones per combination (hardcoded pattern) */
    tones_per_combo = 3;
    
    /* We need exactly 14 combinations (7 unique * 2 rounds) */
    /* Duration checked/adjusted later if needed, but fixed sequence overrides "duration" logic mostly */
    
    /* Allocate state */
    state = calloc(1, sizeof(echo_analysis_state_t));
    if (!state) {
        LOGP(DECHO, LOGL_ERROR, "Failed to allocate echo analysis state\n");
        return NULL;
    }
    
    /* Initialize configuration */
    state->enabled = 1;
    state->duration_sec = duration_sec;
    state->tones_per_combination = tones_per_combo;
    state->frames_per_combination = 25;  /* 500ms per combination (160 samples * 25 = 4000 samples = 500ms) */
    
    /* Initialize phase */
    state->phase = PHASE_SILENCE;
    
    /* Initialize silence period - 3 seconds to allow audio path to establish */
    state->silence_frames = SILENCE_FRAMES;
    state->silence_frame_count = 0;
    state->silence_complete = 0;
    
    /* Initialize chirp state */
    state->current_chirp = 0;
    state->chirp_frame_count = 0;
    state->chirp_tx_active = 0;
    state->chirp_tx_start_ms = 0;
    state->total_samples_processed = 0;
    
    /* Initialize chirp TX sample tracking */
    for (i = 0; i < NUM_CHIRPS; i++) {
        state->chirp_tx_sample[i] = -1;  /* -1 means not yet transmitted */
    }
    
    /* Allocate chirp reference buffer */
    state->chirp_reference = calloc(CHIRP_DURATION_SAMPLES, sizeof(sample_t));
    if (!state->chirp_reference) {
        LOGP(DECHO, LOGL_ERROR, "Failed to allocate chirp reference buffer\n");
        free(state);
        return NULL;
    }
    
    /* Generate reference chirp */
    generate_chirp(state->chirp_reference, CHIRP_DURATION_SAMPLES);
    
    /* Allocate RX buffer for correlation (needs to hold max delay + chirp duration) */
    state->rx_buffer_size = MAX_DELAY_SAMPLES + CHIRP_DURATION_SAMPLES;
    state->rx_buffer = calloc(state->rx_buffer_size, sizeof(sample_t));
    if (!state->rx_buffer) {
        LOGP(DECHO, LOGL_ERROR, "Failed to allocate RX buffer\n");
        free(state->chirp_reference);
        free(state);
        return NULL;
    }
    state->rx_buffer_pos = 0;
    
    /* Initialize chirp results */
    for (i = 0; i < NUM_CHIRPS; i++) {
        state->chirp_results[i].chirp_number = i + 1;
        state->chirp_results[i].tx_timestamp_ms = 0;
        state->chirp_results[i].rx_timestamp_ms = 0;
        state->chirp_results[i].delay_ms = 0.0;
        state->chirp_results[i].detected = 0;
    }
    
    /* Initialize Goertzel filters for all frequencies */
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
        audio_goertzel_init(&state->goertzel[i], 
                           (double)echo_analysis_freqs[i], 
                           ECHO_ANALYSIS_SAMPLE_RATE);
    }
    
    /* Allocate results storage */
    /* Estimate: duration_sec * 2 combinations per second */
    state->max_results = duration_sec * 2 + 10;
    state->results = calloc(state->max_results, sizeof(echo_tone_combination_t));
    if (!state->results) {
        LOGP(DECHO, LOGL_ERROR, "Failed to allocate results storage\n");
        free(state->rx_buffer);
        free(state->chirp_reference);
        free(state);
        return NULL;
    }
    
    /* Initialize per-frequency results */
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
        state->freq_results[i].freq_hz = echo_analysis_freqs[i];
        state->freq_results[i].num_measurements = 0;
        state->freq_results[i].sum_tx_level_dbm = 0.0;
        state->freq_results[i].sum_rx_level_dbm = 0.0;
        state->freq_results[i].sum_delay_ms = 0.0;
        state->freq_results[i].sum_delay_sq_ms = 0.0;
    }
    
    /* Initialize timing - start_time_ms will be set on first TX frame */
    state->start_time_ms = 0;
    state->last_tx_time_ms = 0;
    
    /* Initialize first combination (for tone phase) */
    state->current_combination = 0;
    state->combination_frame_count = 0;  /* Will be incremented to 1 on first call */
    select_next_combination(state);
    
    return state;
}

/* Process TX audio - generates test tones */
void echo_analysis_process_tx(echo_analysis_state_t *state, sample_t *samples, int count)
{
    int t, i;
    
    if (!state || !state->enabled)
        return;
    
    /* Set start time on first TX frame */
    if (state->start_time_ms == 0) {
        state->start_time_ms = get_time_ms();
        LOGP(DECHO, LOGL_NOTICE, "[%llu] Echo analysis: 5 second silence to establish audio path...\n", 
               (unsigned long long)state->start_time_ms);
    }
    
    /* Handle initial silence period */
    if (state->phase == PHASE_SILENCE) {
        uint64_t now = get_time_ms();
        uint64_t elapsed = now - state->start_time_ms;
        
        state->silence_frame_count++;
        
        /* Calculate expected time based on samples processed */
        int total_samples = state->silence_frame_count * count;
        uint64_t expected_ms = (uint64_t)total_samples * 1000 / ECHO_ANALYSIS_SAMPLE_RATE;
        
        /* Debug: Log every 50 calls */
        if (state->silence_frame_count % 50 == 0) {
            LOGP(DECHO, LOGL_DEBUG, "[%llu] SILENCE: call=%d samples=%d elapsed=%llums expected=%llums\n",
                   (unsigned long long)now, state->silence_frame_count, total_samples,
                   (unsigned long long)elapsed, (unsigned long long)expected_ms);
        }
        
        /* Check if silence period is complete */
        /* Check if silence period is complete */
        if (state->silence_frame_count >= state->silence_frames) {
            state->phase = PHASE_CHIRP;
            LOGP(DECHO, LOGL_NOTICE, "[%llu] Echo analysis: Starting chirp delay measurement (10 chirps)... (elapsed=%llums, samples=%d)\n",
                   (unsigned long long)now, (unsigned long long)elapsed, total_samples);
            
            /* Initialize first chirp state - transmission will start on next call */
            state->chirp_tx_active = 1;
            state->chirp_frame_count = 0;
            state->current_chirp = 0;
            state->chirp_tx_start_ms = now;
            state->chirp_results[0].tx_timestamp_ms = now;
            state->chirp_tx_sample[0] = state->total_samples_processed;
            
            LOGP(DECHO, LOGL_DEBUG, "Starting chirp 1 (initial)\n");
            
            /* Generate comfort noise for this frame, chirp starts on NEXT call */
            for (i = 0; i < count; i++) {
                double noise = ((double)rand() / RAND_MAX * 2.0 - 1.0) * 0.001;
                samples[i] = noise;
            }
            return;
        } else {
            /* Still in silence period - generate comfort noise to keep VAD active
             * Use very low amplitude white noise (-60 dBFS) to prevent phone from
             * stopping RTP transmission due to silence detection */
            for (i = 0; i < count; i++) {
                /* Generate white noise at -60 dBFS (amplitude 0.001) */
                double noise = ((double)rand() / RAND_MAX * 2.0 - 1.0) * 0.001;
                samples[i] = noise;
            }
            return;
        }
    }
    
    /* Handle chirp phase */
    if (state->phase == PHASE_CHIRP) {
        uint64_t now = get_time_ms();
        
        /* Check if all chirps are done (transmitted AND either detected or timed out) */
        int all_chirps_done = (state->current_chirp >= NUM_CHIRPS);
        
        /* If all chirps transmitted, wait for detection or timeout before transitioning */
        if (all_chirps_done) {
            /* Check if we've waited long enough for the last chirp (625ms) */
            uint64_t last_chirp_time = state->chirp_results[NUM_CHIRPS - 1].tx_timestamp_ms;
            uint64_t elapsed_since_last = now - last_chirp_time;
            
            /* Wait at least 625ms after last chirp for detection */
            if (elapsed_since_last < 625) {
                /* Still waiting for last chirp detection - generate comfort noise */
                for (i = 0; i < count; i++) {
                    /* Generate white noise at -60 dBFS (amplitude 0.001) */
                    double noise = ((double)rand() / RAND_MAX * 2.0 - 1.0) * 0.001;
                    samples[i] = noise;
                }
                state->total_samples_processed += count;
                return;
            }
            
            /* Print chirp results and transition to tone phase */
            LOGP(DECHO, LOGL_NOTICE, "\nChirp Delay Measurement Results:\n");
            
            int detected_count = 0;
            double sum_delay = 0.0;
            double sum_delay_sq = 0.0;
            double min_delay = 1e9;
            double max_delay = 0.0;
            
            /* Print all chirps but only use chirps 5-9 (indices 4-8) for statistics */
            for (i = 0; i < NUM_CHIRPS; i++) {
                if (state->chirp_results[i].detected) {
                    double delay = state->chirp_results[i].delay_ms;
                    LOGP(DECHO, LOGL_INFO, "  Chirp %d: %.1f ms%s\n", i + 1, delay, 
                           (i < 4 || i >= 9) ? " (ignored)" : "");
                    
                    /* Only include chirps 5-9 (indices 4-8) in statistics */
                    if (i >= 4 && i < 9) {
                        detected_count++;
                        sum_delay += delay;
                        sum_delay_sq += delay * delay;
                        if (delay < min_delay) min_delay = delay;
                        if (delay > max_delay) max_delay = delay;
                    }
                } else {
                    LOGP(DECHO, LOGL_INFO, "  Chirp %d: Not detected%s\n", i + 1,
                           (i < 4 || i >= 9) ? " (ignored)" : "");
                }
            }
            

            if (detected_count > 0) {
                double mean_delay = sum_delay / detected_count;
                double variance = (sum_delay_sq / detected_count) - (mean_delay * mean_delay);
                double stddev = sqrt(variance > 0 ? variance : 0);
                
                LOGP(DECHO, LOGL_NOTICE, "\nChirp Statistics (chirps 5-9 only):\n");
                LOGP(DECHO, LOGL_NOTICE, "  Mean Delay: %.1f ms\n", mean_delay);
                LOGP(DECHO, LOGL_NOTICE, "  Jitter (stddev): %.1f ms\n", stddev);
                LOGP(DECHO, LOGL_NOTICE, "  Min Delay: %.1f ms\n", min_delay);
                LOGP(DECHO, LOGL_NOTICE, "  Max Delay: %.1f ms\n", max_delay);
                LOGP(DECHO, LOGL_NOTICE, "  Detected: %d/5 chirps (ignoring first 4 and last 1)\n", detected_count);
                
                if (stddev < mean_delay * 0.1) {
                    LOGP(DECHO, LOGL_NOTICE, "  Assessment: Low jitter - stable path delay\n");
                } else if (stddev < mean_delay * 0.2) {
                    LOGP(DECHO, LOGL_NOTICE, "  Assessment: Moderate jitter - acceptable\n");
                } else {
                    LOGP(DECHO, LOGL_NOTICE, "  Assessment: High jitter - unstable path\n");
                }
            } else {
                LOGP(DECHO, LOGL_NOTICE, "\nNo chirps detected in measurement range (chirps 5-9)\n");
            }
            
            LOGP(DECHO, LOGL_NOTICE, "\nStarting tone-based frequency response analysis...\n");
            state->phase = PHASE_TONE;
            state->silence_complete = 1;  /* Mark as ready for tone phase */
            return;
        }
        
        /* Transmitting a chirp */
        if (state->chirp_tx_active) {
            /* Calculate how many samples of the chirp we've sent */
            int chirp_samples_sent = state->chirp_frame_count * count;
            int samples_remaining = CHIRP_DURATION_SAMPLES - chirp_samples_sent;
            
            if (samples_remaining > 0) {
                /* Still have chirp data to send */
                int samples_to_send = (samples_remaining < count) ? samples_remaining : count;
                
                /* Copy from reference chirp */
                for (i = 0; i < samples_to_send; i++) {
                    samples[i] = state->chirp_reference[chirp_samples_sent + i];
                }
                
                /* Fill rest with silence if needed */
                for (i = samples_to_send; i < count; i++) {
                    samples[i] = 0.0;
                }
                
                state->chirp_frame_count++;
            } else {
                /* Chirp transmission complete - start interval */
                LOGP(DECHO, LOGL_DEBUG, "Chirp %d transmission complete\n", state->current_chirp + 1);
                for (i = 0; i < count; i++) {
                    samples[i] = 0.0;
                }
                
                state->chirp_tx_active = 0;
                state->current_chirp++;
            }
            
            /* Increment sample counter */
            state->total_samples_processed += count;
            return;
        }
        
        /* Waiting in interval between chirps - send comfort noise to keep VAD active */
        for (i = 0; i < count; i++) {
            /* Generate white noise at -60 dBFS (amplitude 0.001) */
            double noise = ((double)rand() / RAND_MAX * 2.0 - 1.0) * 0.001;
            samples[i] = noise;
        }
        
        /* Check if interval is complete using wall-clock time (like tones do) */
        if (state->current_chirp < NUM_CHIRPS) {
            /* Get time since last chirp was transmitted */
            uint64_t last_chirp_time = (state->current_chirp > 0) ? 
                state->chirp_results[state->current_chirp - 1].tx_timestamp_ms : 
                state->chirp_tx_start_ms;
            
            uint64_t elapsed = now - last_chirp_time;
            
            /* Start next chirp if 500ms has elapsed */
            if (elapsed >= CHIRP_INTERVAL_MS) {
                LOGP(DECHO, LOGL_DEBUG, "Starting chirp %d after %llums interval\n", 
                       state->current_chirp + 1, (unsigned long long)elapsed);
                state->chirp_tx_active = 1;
                state->chirp_frame_count = 0;  /* Reset to 0, will be used by transmission block */
                state->chirp_tx_start_ms = now;
                state->chirp_results[state->current_chirp].tx_timestamp_ms = now;
                state->chirp_tx_sample[state->current_chirp] = state->total_samples_processed;
                /* Don't transmit here - let the transmission block handle it on next call */
            }
        }
        
        /* Increment sample counter */
        state->total_samples_processed += count;
        return;
    }
    
    /* Handle tone phase (existing code) */
    if (state->phase == PHASE_TONE) {
        /* Increment frame count at START of function */
        state->combination_frame_count++;
        
        /* Check if we need to advance to next combination */
        if (state->combination_frame_count > state->frames_per_combination) {
            /* Save current combination to results if it was transmitted */
            if (state->current.tx_timestamp_ms > 0 && state->num_results < state->max_results) {
                state->results[state->num_results] = state->current;
                state->num_results++;
            }
            
            /* Advance to next combination */
            state->current_combination++;
            
            /* Stop after 14 combinations (2 rounds of 7) */
            if (state->current_combination >= 14) {
                 LOGP(DECHO, LOGL_NOTICE, "Echo analysis: Tone generation complete (14 combinations)\n");
                 state->phase = PHASE_COMPLETE;
                 return;
            }
            
            select_next_combination(state);
            state->combination_frame_count = 1;  /* This is frame 1 of new combination */
        }
        
        /* Record TX timestamp on first frame of combination */
        if (state->combination_frame_count == 1 && state->current.tx_timestamp_ms == 0) {
            state->current.tx_timestamp_ms = get_time_ms();
            state->last_tx_time_ms = state->current.tx_timestamp_ms;
            
            /* Calculate TX levels for each tone */
            for (t = 0; t < state->current.num_tones; t++) {
                int freq_idx = state->current.freq_indices[t];
                double amplitude = get_speech_equivalent_amplitude(freq_idx);
                /* Convert amplitude to dBm (relative to 1.0 = 0 dBm) */
                state->current.tx_levels_dbm[t] = 20.0 * log10(amplitude + 1e-10);
            }
            
            /* Log TX event */
            LOGP(DECHO, LOGL_DEBUG, "TX seq=%d time=%llu freqs=", 
                   state->current.sequence,
                   (unsigned long long)state->current.tx_timestamp_ms);
            for (t = 0; t < state->current.num_tones; t++) {
                int freq_idx = state->current.freq_indices[t];
                LOGPC(DECHO, LOGL_DEBUG, "%d%s", echo_analysis_freqs[freq_idx],
                       (t < state->current.num_tones - 1) ? "," : "");
            }
            LOGPC(DECHO, LOGL_DEBUG, " Hz\n");
        }
        
        /* Generate tones for current combination */
        generate_tone_combination(state, samples, count);
        
        /* Increment sample counter (for chirp delay calculation) */
        state->total_samples_processed += count;
    }
}



/* Check if a frequency index is in the current combination */
static int is_frequency_in_combination(echo_analysis_state_t *state, int freq_idx)
{
    int t;
    for (t = 0; t < state->current.num_tones; t++) {
        if (state->current.freq_indices[t] == freq_idx)
            return 1;
    }
    return 0;
}

/* Get the tone index (0-4) for a frequency index in the current combination */
static int get_tone_index_in_combination(echo_analysis_state_t *state, int freq_idx)
{
    int t;
    for (t = 0; t < state->current.num_tones; t++) {
        if (state->current.freq_indices[t] == freq_idx)
            return t;
    }
    return -1;
}

/* Minimum SNR required for confident tone detection (dB) */
#define MIN_SNR_DB 6.0

/* Detect tones in received audio using Goertzel filters */
static void detect_tones(echo_analysis_state_t *state, sample_t *samples, int count)
{
    double results[ECHO_ANALYSIS_NUM_FREQS];
    int i;
    
    /* Run Goertzel on all frequencies */
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
        audio_goertzel(&state->goertzel[i], samples, count, 0, &results[i], 1);
    }
    
    /* First pass: calculate noise floor from non-expected frequencies */
    double noise_sum = 0.0;
    int noise_count = 0;
    
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
        if (!is_frequency_in_combination(state, i)) {
            noise_sum += results[i];
            noise_count++;
        }
    }
    
    /* Calculate noise floor */
    double noise_floor_linear = (noise_count > 0) ? (noise_sum / noise_count) : 1e-10;
    state->current.noise_floor_dbm = 20.0 * log10(noise_floor_linear + 1e-10);
    
    /* Second pass: check each expected tone against noise floor with SNR requirement */
    int detected_count = 0;
    
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
        if (is_frequency_in_combination(state, i)) {
            int tone_idx = get_tone_index_in_combination(state, i);
            if (tone_idx >= 0) {
                double level_dbm = 20.0 * log10(results[i] + 1e-10);
                state->current.rx_levels_dbm[tone_idx] = level_dbm;
                
                /* Check SNR: tone must be MIN_SNR_DB above noise floor */
                double snr = level_dbm - state->current.noise_floor_dbm;
                
                
                if (snr >= MIN_SNR_DB) {
                    detected_count++;
                }
            }
        }
    }
    
    /* Require at least ONE tone to be detected */
    if (detected_count >= 1) {
        state->current.detected = 1;
    }
}

/* Process RX audio - detects echoed tones */
void echo_analysis_process_rx(echo_analysis_state_t *state, sample_t *samples, int count)
{
    int t, i;
    
    if (!state || !state->enabled)
        return;
    
    /* Skip during silence period */
    if (state->phase == PHASE_SILENCE)
        return;
    
    /* Handle chirp phase - accumulate RX samples and correlate */
    if (state->phase == PHASE_CHIRP) {
        static int rx_call_count = 0;
        uint64_t now = get_time_ms();
        rx_call_count++;
        
        /* Calculate RMS of received samples to check if we're getting audio */
        double rms = 0.0;
        for (i = 0; i < count; i++) {
            rms += samples[i] * samples[i];
        }
        rms = sqrt(rms / count);
        
        /* Print debug info every 10 calls to see what's happening */
        if (rx_call_count % 10 == 0) {
            printf("DEBUG RX: [%llu] call=%d, rx_buffer_pos=%d, rms=%.6f\n", 
                   (unsigned long long)now, rx_call_count, state->rx_buffer_pos, rms);
        }
        
        /* Add samples to RX buffer (circular buffer) */
        for (i = 0; i < count; i++) {
            state->rx_buffer[state->rx_buffer_pos] = samples[i];
            state->rx_buffer_pos = (state->rx_buffer_pos + 1) % state->rx_buffer_size;
        }
        
        /* Check if we should try to detect a chirp */
        /* Only try detection ONCE after we have collected enough samples to cover the max delay */
        for (i = 0; i < NUM_CHIRPS; i++) {
            chirp_result_t *result = &state->chirp_results[i];
            
            /* Skip if already detected or not yet transmitted */
            if (result->detected || state->chirp_tx_sample[i] < 0)
                continue;
            
            /* Check if we have already tried detection for this chirp (and failed) */
             /* We use result->rx_timestamp_ms == 1 to indicate "tried but failed" to avoid re-running */
            if (result->rx_timestamp_ms == 1)
                continue;

            /* Calculate how many samples have passed since this chirp was transmitted */
            int samples_since_tx = state->total_samples_processed - state->chirp_tx_sample[i];
            
            /* Wait until we have slightly more than MAX_DELAY_SAMPLES + CHIRP_DURATION_SAMPLES */
            /* This ensures the full window is available */
            int needed_samples = MAX_DELAY_SAMPLES + CHIRP_DURATION_SAMPLES + 100;
            
            if (samples_since_tx < needed_samples)
                continue;
            
            /* Perform correlation ONCE */
            
            /* Create linear buffer from circular RX buffer */
            /* We only need the last 'needed_samples' from the buffer, but for simplicity
             * lets just unwrap the whole thing or enough of it.
             * Actually, we need to extract the window corresponding to:
             * [tx_time ... tx_time + MAX_DELAY + CHIRP_LEN]
             * But since we don't track absolute sample index in rx_buffer easily (it overwrites),
             * we rely on the fact that rx_buffer_size > MAX_DELAY + CHIRP_LEN.
             * 
             * state->rx_buffer_size is MAX_DELAY_SAMPLES + CHIRP_DURATION_SAMPLES.
             * So the buffer holds EXACTLY the window we need (plus maybe a tiny bit if aligned).
             * 
             * Wait, if samples_since_tx is large, the start of the chirp (at delay=0) 
             * might have been overwritten if the buffer is too small?
             * 
             * rx_buffer_size = MAX_DELAY + CHIRP_DURATION.
             * If we wait for 'needed_samples' (which is approx rx_buffer_size),
             * then the OLDEST sample in the buffer should be roughly the TX start time.
             * 
             * Let's construct a linear buffer of the last 'rx_buffer_size' samples.
             */
            
            sample_t *linear_buffer = calloc(state->rx_buffer_size, sizeof(sample_t));
            if (!linear_buffer) continue;

            int pos = state->rx_buffer_pos; /* Oldest sample is here? No, rx_buffer_pos is where NEXT write goes. */
            /* So rx_buffer_pos is effectively the oldest sample index in the circular buffer */
            
            for (int j = 0; j < state->rx_buffer_size; j++) {
                linear_buffer[j] = state->rx_buffer[pos];
                pos = (pos + 1) % state->rx_buffer_size;
            }
            
            printf("DEBUG: Correlating chirp %d (samples_since_tx=%d)...\n", i+1, samples_since_tx);
            
            /* Correlate to find delay */
            int delay_samples = correlate_chirp(state->chirp_reference, linear_buffer, 
                                               CHIRP_DURATION_SAMPLES, state->rx_buffer_size);
            
            free(linear_buffer);

            if (delay_samples >= 0) {
                /* Chirp detected! */
                uint64_t current_time = get_time_ms();
                result->detected = 1;
                result->rx_timestamp_ms = current_time; /* This is just "when we found it" */
                /* Real delay is delay_samples relative to the start of the window we passed.
                 * The window we passed roughly represents [now - rx_buffer_size ... now]
                 * Wait, correlation finds delay relative to the START of 'linear_buffer'.
                 * 'linear_buffer' oldest sample corresponds to 'now - rx_buffer_size' (approx).
                 * So real delay relative to TX?
                 * 
                 * TX happened at 'state->chirp_tx_sample[i]'.
                 * Current sample count is 'state->total_samples_processed'.
                 * We captured the last 'state->rx_buffer_size' samples.
                 * The oldest sample in linear_buffer is at absolute index: 
                 *    total_samples_processed - state->rx_buffer_size.
                 * 
                 * The chirp was found at offset 'delay_samples' into linear_buffer.
                 * So the chirp started at absolute index:
                 *    start_index = (total_samples_processed - rx_buffer_size) + delay_samples.
                 * 
                 * The propagation delay in samples is:
                 *    prop_delay = start_index - state->chirp_tx_sample[i].
                 */
                 
                int buffer_start_sample_idx = state->total_samples_processed - state->rx_buffer_size;
                int chirp_found_at_idx = buffer_start_sample_idx + delay_samples;
                double calc_delay_samples = chirp_found_at_idx - state->chirp_tx_sample[i];
                
                result->delay_ms = calc_delay_samples * 1000.0 / ECHO_ANALYSIS_SAMPLE_RATE;
                
                printf("RX chirp %d detected! delay_samples=%.1f (offset=%d) delay=%.1f ms\n",
                       i + 1,
                       calc_delay_samples,
                       delay_samples,
                       result->delay_ms);
            } else {
                printf("RX chirp %d NOT detected.\n", i + 1);
                /* Mark as processed so we don't try again */
                result->rx_timestamp_ms = 1; 
            }
        }
        
        return;
    }
    
    /* Handle tone phase (existing code) */
    if (state->phase == PHASE_TONE) {
        /* Skip if no active TX combination */
        if (state->current.tx_timestamp_ms == 0)
            return;
        
        /* Detect tones in received audio */
        detect_tones(state, samples, count);
        
        /* If tones were detected, record RX timestamp and log event */
        if (state->current.detected && state->current.rx_timestamp_ms == 0) {
            state->current.rx_timestamp_ms = get_time_ms();
            
            /* Calculate delay */
            double delay_ms = (double)(state->current.rx_timestamp_ms - state->current.tx_timestamp_ms);
            
            /* Calculate average SNR across all tones */
            double avg_snr = 0.0;
            for (t = 0; t < state->current.num_tones; t++) {
                avg_snr += state->current.rx_levels_dbm[t] - state->current.noise_floor_dbm;
            }
            avg_snr /= state->current.num_tones;
            
            /* Log RX event on a single line */
            printf("RX seq=%d time=%llu freqs=", 
                   state->current.sequence,
                   (unsigned long long)state->current.rx_timestamp_ms);
            
            /* Print frequencies */
            for (t = 0; t < state->current.num_tones; t++) {
                int freq_idx = state->current.freq_indices[t];
                printf("%d%s", echo_analysis_freqs[freq_idx],
                       (t < state->current.num_tones - 1) ? "," : "");
            }
            printf(" Hz levels=");
            
            /* Print RX levels */
            for (t = 0; t < state->current.num_tones; t++) {
                printf("%.1f%s", state->current.rx_levels_dbm[t],
                       (t < state->current.num_tones - 1) ? "," : "");
            }
            
            /* Print noise floor, SNR, and latency */
            printf(" dBm noise=%.1f dBm SNR=%.1f dB latency=%.1f ms\n",
                   state->current.noise_floor_dbm, avg_snr, delay_ms);
            
            /* Update per-frequency statistics */
            for (t = 0; t < state->current.num_tones; t++) {
                int freq_idx = state->current.freq_indices[t];
                echo_freq_result_t *freq_result = &state->freq_results[freq_idx];
                
                freq_result->num_measurements++;
                freq_result->sum_tx_level_dbm += state->current.tx_levels_dbm[t];
                freq_result->sum_rx_level_dbm += state->current.rx_levels_dbm[t];
                freq_result->sum_delay_ms += delay_ms;
                freq_result->sum_delay_sq_ms += delay_ms * delay_ms;
            }
            
            /* Update overall statistics */
            state->total_measurements++;
            state->sum_delay_ms += delay_ms;
            state->sum_delay_sq_ms += delay_ms * delay_ms;
            
            /* Accumulate levels for averaging */
            for (t = 0; t < state->current.num_tones; t++) {
                state->total_tx_level_dbm += state->current.tx_levels_dbm[t];
                state->total_rx_level_dbm += state->current.rx_levels_dbm[t];
            }
            state->total_noise_dbm += state->current.noise_floor_dbm;
        }
    }
}

/* Check if analysis is complete */
int echo_analysis_is_complete(echo_analysis_state_t *state)
{
    if (!state || !state->enabled)
        return 1;
    
    /* If test hasn't started yet (start_time_ms == 0), it's not complete */
    if (state->start_time_ms == 0)
        return 0;

    /* Check if phase is COMPLETE */
    if (state->phase == PHASE_COMPLETE)
        return 1;
    
    /* Wait forever until phase is complete. 
     * The state machine is responsible for transitioning to PHASE_COMPLETE. 
     */
    return 0;
}

/* Print ASCII frequency response chart */
static void print_frequency_response_chart(echo_analysis_state_t *state)
{
    int i, row;
    double rx_levels[ECHO_ANALYSIS_NUM_FREQS];
    double max_level_dbm = -100.0;
    double min_level_dbm = 100.0;
    double level_step = 2.0;
    int has_data = 0;
    double avg_noise_dbm = -90.0;
    
    if (state->total_measurements > 0) {
        avg_noise_dbm = state->total_noise_dbm / state->total_measurements;
    }
    
    /* Calculate metrics for each frequency */
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
        echo_freq_result_t *freq = &state->freq_results[i];
        
        if (freq->num_measurements > 0) {
            double avg_rx = freq->sum_rx_level_dbm / freq->num_measurements;
            
            rx_levels[i] = avg_rx;
            has_data = 1;
            
            if (avg_rx > max_level_dbm) max_level_dbm = avg_rx;
        } else {
            rx_levels[i] = -100.0; /* No signal */
        }
    }
    
    if (!has_data) {
        printf("Frequency Response: No data available\n");
        return;
    }
    
    /* Set chart range based on noise and signal */
    /* Bottom aligns with noise floor */
    min_level_dbm = floor(avg_noise_dbm / level_step) * level_step;
    /* Top aligns with max signal + headroom */
    max_level_dbm = ceil(max_level_dbm / level_step) * level_step;
    if (max_level_dbm < min_level_dbm + 10.0) max_level_dbm = min_level_dbm + 10.0;
    
    /* Print header */
    /* Level --------------------Frequency-------------------- Attn */
    printf("Level ");
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) printf("-");
    printf("Frequency");
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) printf("-");
    printf(" Attn\n");
    
    /* Print rows */
    for (double level = max_level_dbm; level >= min_level_dbm; level -= level_step) {
        /* Left Y-axis label (Level) */
        printf(" %3.0f  ", level);
        
        /* Bar chart */
        /* Start with dot, then X, then x at top? 
         * User example:
         * -18  . . x x ...
         * -34  x X X ...
         * -44  X=X=...
         * 
         * Interpretation: 
         * If RX Level >= current_row_level, print char.
         * The 'height' of the bar represents RX level.
         */
        
        printf("."); /* Left margin dot in user example? No, looks like just space or dot grid */
        
        for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
            /* Check if this frequency has signal at this level */
            if (state->freq_results[i].num_measurements > 0 && rx_levels[i] >= level) {
                if (level <= min_level_dbm + 0.001) {
                     /* Bottom row (noise floor): Alternate X and = */
                     printf("%c", (i % 2 == 0) ? 'X' : '=');
                } else if (rx_levels[i] < level + level_step) {
                     /* Top-most row for this frequency: x */
                     printf("x");
                } else {
                     /* Body: X */
                     printf("X");
                }
            } else {
                printf(".");
            }
            printf(" "); /* Spacing */
        }
        
        /* Right Y-axis label (Attenuation: 0, 2, 4...) matching levels -18, -20... */
        /* Ref level -18 corresponds to Attn 0 */
        double attn_label = -(level - max_level_dbm);
        printf("  %2.0f\n", attn_label);
    }
    
    /* Print Frequency Labels (Vertical) */
    /* Rows: thousands, hundreds, tens, units */
    int digits[4][ECHO_ANALYSIS_NUM_FREQS];
    for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
        int freq = echo_analysis_freqs[i];
        digits[0][i] = (freq / 1000) % 10;
        digits[1][i] = (freq / 100) % 10;
        digits[2][i] = (freq / 10) % 10;
        digits[3][i] = freq % 10;
    }
    
    for (row = 0; row < 4; row++) {
        printf("      "); /* Indent to match chart */
        printf(" ");    /* Extra space for dot column */
        for (i = 0; i < ECHO_ANALYSIS_NUM_FREQS; i++) {
            printf("%d ", digits[row][i]);
        }
        printf("\n");
    }
    printf("\n");
}

/* Generate and print final report */
void echo_analysis_print_report(echo_analysis_state_t *state)
{
    int i;
    double avg_tx_level_dbm, avg_rx_level_dbm, avg_noise_dbm;
    double snr_db, near_echo_loss_db;
    double delay_mean_ms, delay_stddev_ms;
    int is_incomplete = 0;
    
    if (!state)
        return;
    
    /* Check if test was interrupted (not complete) */
    if (!echo_analysis_is_complete(state)) {
        is_incomplete = 1;
    }
    
    printf("\n");
    printf("========================================\n");
    if (is_incomplete) {
        printf("Echo Analysis Report (INCOMPLETE)\n");
    } else {
        printf("Echo Analysis Report\n");
    }
    printf("========================================\n");
    
    /* Check if we have any measurements */
    if (state->total_measurements == 0) {
        printf("No echo detected\n");
        printf("========================================\n");
        return;
    }
    
    /* Calculate average levels */
    int total_tone_measurements = 0;
    for (i = 0; i < state->current.num_tones; i++) {
        total_tone_measurements += state->total_measurements;
    }
    
    avg_tx_level_dbm = state->total_tx_level_dbm / (state->total_measurements * state->tones_per_combination);
    avg_rx_level_dbm = state->total_rx_level_dbm / (state->total_measurements * state->tones_per_combination);
    avg_noise_dbm = state->total_noise_dbm / state->total_measurements;
    
    /* Calculate SNR (signal to noise ratio) */
    /* SNR = RX signal level - noise floor */
    snr_db = avg_rx_level_dbm - avg_noise_dbm;
    
    /* Calculate echo loss */
    /* Echo loss = TX level - RX level (positive value means attenuation) */
    near_echo_loss_db = avg_tx_level_dbm - avg_rx_level_dbm;
    /* Echo loss = TX level - RX level (positive value means attenuation) */
    near_echo_loss_db = avg_tx_level_dbm - avg_rx_level_dbm;
    
    /* Calculate delay statistics */
    delay_mean_ms = state->sum_delay_ms / state->total_measurements;
    
    /* Calculate standard deviation: sqrt(E[X^2] - E[X]^2) */
    double mean_sq = state->sum_delay_sq_ms / state->total_measurements;
    double sq_mean = delay_mean_ms * delay_mean_ms;
    delay_stddev_ms = sqrt(mean_sq - sq_mean);
    
    /* Print report */
    printf("\n");
    printf("Level Measurements:\n");
    printf("  RX Level: %.1f dBm\n", avg_rx_level_dbm);
    printf("  TX Level: %.1f dBm\n", avg_tx_level_dbm);
    printf("  Noise Floor: %.1f dBm\n", avg_noise_dbm);
    printf("  SNR: %.1f dB\n", snr_db);
    printf("\n");
    
    printf("Echo Loss:\n");
    printf("  Echo Return Loss: %.1f dB\n", near_echo_loss_db);
    printf("\n");
    
    printf("Delay Measurements:\n");
    printf("  Roundtrip Delay: %.1f ms (± %.1f ms)\n", delay_mean_ms, delay_stddev_ms);
    printf("  Total Measurements: %d\n", state->total_measurements);
    printf("\n");
    
    /* Check for warnings */
    if (avg_rx_level_dbm < -50.0) {
        printf("WARNING: Low signal level detected (< -50 dBm)\n");
        printf("         Results may be unreliable\n");
        printf("\n");
    }
    
    if (delay_stddev_ms > delay_mean_ms * 0.2) {
        printf("WARNING: High delay variation detected\n");
        printf("         Possible causes: jitter, interference, multipath\n");
        printf("\n");
    }
    
    /* Print frequency response chart */
    print_frequency_response_chart(state);
    
    printf("========================================\n");
}

/* Cleanup echo analysis */
void echo_analysis_cleanup(echo_analysis_state_t *state)
{
    if (!state)
        return;
    
    if (state->results)
        free(state->results);
    
    if (state->chirp_reference)
        free(state->chirp_reference);
    
    if (state->rx_buffer)
        free(state->rx_buffer);
    
    free(state);
}
