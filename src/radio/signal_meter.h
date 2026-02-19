/* Signal Meter - Signal level measurement for FM radio
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef SIGNAL_METER_H
#define SIGNAL_METER_H

#include <stdint.h>
#include <sys/types.h>

/* Sentinel value indicating no valid measurement yet */
#define SIGNAL_METER_NO_VALUE  (-200.0)

/* Maximum registered threads for CPU monitoring */
#define SIGNAL_METER_MAX_THREADS  8

/* Signal meter state (opaque to callers, defined in .c) */
typedef struct signal_meter signal_meter_t;

/* Thread CPU info (returned by query functions) */
typedef struct {
    char        name[32];       /* Thread name */
    pid_t       tid;            /* Thread ID */
    double      cpu_pct;        /* Current CPU % of one core (0-100) */
    double      peak_cpu_pct;   /* Peak CPU % since last reset */
    int         capacity_warn;  /* 1 if cpu_pct > 90% */
} signal_meter_thread_info_t;

/*
 * Initialize signal meter.
 *   sample_rate: DSP sample rate in Hz (e.g. 100000)
 *   dbf_offset:  offset added to dBFS for xdr-gtk (default 120.0)
 * Returns allocated signal_meter_t, or NULL on failure.
 */
signal_meter_t *signal_meter_init(int sample_rate, double dbf_offset);

/*
 * Destroy signal meter and free resources.
 */
void signal_meter_free(signal_meter_t *sm);

/*
 * Feed IQ samples. Call from main loop after sdr_read().
 *   iq_buf: interleaved float I/Q pairs
 *   count:  number of I/Q pairs (NOT number of floats)
 */
void signal_meter_feed_iq(signal_meter_t *sm,
                          const float *iq_buf, int count);

/*
 * Reset all measurement state. Call on retune.
 */
void signal_meter_reset(signal_meter_t *sm);

/*
 * Get smoothed signal level in dBFS.
 * Returns SIGNAL_METER_NO_VALUE if no valid measurement yet.
 */
double signal_meter_get_level_dbfs(const signal_meter_t *sm);

/*
 * Get smoothed signal level + dBf offset (for xdr-gtk).
 * Returns SIGNAL_METER_NO_VALUE if no valid measurement yet.
 */
double signal_meter_get_level_dbf(const signal_meter_t *sm);

/*
 * Get noise floor estimate in dBFS.
 * Returns SIGNAL_METER_NO_VALUE if no valid measurement yet.
 */
double signal_meter_get_noise_floor(const signal_meter_t *sm);

/*
 * Get SNR estimate (smoothed_dBFS - noise_floor) in dB.
 * Returns 0.0 if no valid measurement yet.
 */
double signal_meter_get_snr(const signal_meter_t *sm);

/*
 * Get peak hold value in dBFS.
 * Returns SIGNAL_METER_NO_VALUE if no valid measurement yet.
 */
double signal_meter_get_peak(const signal_meter_t *sm);

/*
 * Get raw time-domain RMS in dBFS (always from IQ samples, never FFT-overridden).
 * Useful for debug comparison between time-domain and FFT measurements.
 */
double signal_meter_get_raw_dbfs(const signal_meter_t *sm);

/*
 * Set dBf offset at runtime.
 */
void signal_meter_set_dbf_offset(signal_meter_t *sm, double offset);

/*
 * Set external noise floor from SDR layer (sdr_status.rx.noise_floor_db).
 * Call periodically from main loop after sdr_status_snapshot().
 * This gives accurate noise floor from wideband SDR input where
 * signal and noise are separable.
 */
void signal_meter_set_noise_floor(signal_meter_t *sm, double nf_db);

/*
 * Set FFT-based signal power from spectral measurement.
 * When set, this overrides the time-domain RMS for SNR/dBf calculation,
 * giving values that match the scan engine.
 * Set to SIGNAL_METER_NO_VALUE to revert to time-domain RMS.
 */
void signal_meter_set_signal_power(signal_meter_t *sm, double power_dbfs);

/*
 * Update 19 kHz FM stereo pilot info. Call after radio_rx().
 * The "stereo pilot" is the 19 kHz subcarrier tone in FM broadcast
 * that indicates stereo presence — NOT the RF carrier.
 * pilot_mag comes from radio.rx_pilot_mag_avg (IIR-smoothed magnitude).
 */
void signal_meter_set_stereo_pilot(signal_meter_t *sm,
                            double pilot_mag, int pilot_locked);

/*
 * Get 19 kHz stereo pilot magnitude (0.0 = no pilot, ~0.1 = locked).
 */
double signal_meter_get_stereo_pilot_mag(const signal_meter_t *sm);

/*
 * Get stereo pilot lock state (1 = locked/stereo active, 0 = unlocked/mono).
 */
int signal_meter_get_stereo_pilot_locked(const signal_meter_t *sm);

/*
 * Register a thread for CPU monitoring.
 *   name: human-readable thread name (e.g. "main", "sdr_rx")
 *   tid:  thread ID from gettid()
 * Returns 0 on success, -1 if max threads reached.
 */
int signal_meter_register_thread(signal_meter_t *sm,
                                 const char *name, pid_t tid);

/*
 * Update CPU load measurements. Call periodically (e.g. once per second).
 * Reads /proc/self/stat and /proc/self/task/<tid>/stat.
 */
void signal_meter_update_cpu(signal_meter_t *sm);

/*
 * Get whole-process CPU load (% of all cores, 0-100*ncores).
 */
double signal_meter_get_process_cpu(const signal_meter_t *sm);

/*
 * Get peak whole-process CPU load since last reset.
 */
double signal_meter_get_process_cpu_peak(const signal_meter_t *sm);

/*
 * Get per-thread CPU info.
 *   out:     array to fill with thread info
 *   max_out: size of out array
 * Returns number of threads written.
 */
int signal_meter_get_thread_cpu(const signal_meter_t *sm,
                                signal_meter_thread_info_t *out,
                                int max_out);

/*
 * Check if any capacity warning is active.
 * Returns 1 if any thread > 90% or process > 90% of available cores.
 */
int signal_meter_cpu_warning(const signal_meter_t *sm);

#endif /* SIGNAL_METER_H */
