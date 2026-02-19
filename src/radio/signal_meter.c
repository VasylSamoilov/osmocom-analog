/* Signal Meter - Signal level measurement for FM radio
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FFT-based measurement (primary):
 *   - Signal power: FFT at tuned frequency (±20 kHz integration)
 *   - Noise floor: minimum power across offset frequencies
 *   - Output: SNR in dB (signal - noise)
 *
 * Time-domain RMS (fallback when FFT not available):
 *   - mean_power = Σ(I² + Q²) / N → dBFS
 *   - Uses dbf_offset for approximate dBf conversion
 *
 * Also tracks: stereo pilot lock, CPU usage per thread.
 */

#include "signal_meter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

struct signal_meter {
    int         sample_rate;
    int         window_samples;
    double      dbf_offset;         /* fallback offset when no FFT noise floor */
    double      power_sum;
    int         count;
    double      raw_dbfs;
    double      smoothed_dbfs;      /* time-domain RMS (fallback) */
    double      peak_dbfs;
    int         valid;
    double      ext_nf_db;          /* FFT noise floor from sdr_status */
    int         ext_nf_valid;
    double      ext_signal_dbfs;    /* FFT signal power at tuned freq */
    int         ext_signal_valid;
    double      attack, decay, peak_decay;
    double      pilot_mag;
    int         pilot_locked;
    int         num_threads;
    struct {
        char name[32];
        pid_t tid;
        unsigned long long prev_utime, prev_stime;
        double cpu_pct, peak_cpu_pct;
    } threads[SIGNAL_METER_MAX_THREADS];
    unsigned long long proc_prev_utime, proc_prev_stime;
    double      proc_cpu_pct, proc_peak_cpu_pct;
    int         num_cores;
    int64_t     cpu_last_us;
    long        clk_tck;
};

/* ---- init / free / reset ---- */

signal_meter_t *signal_meter_init(int sample_rate, double dbf_offset)
{
    if (sample_rate <= 0) return NULL;
    signal_meter_t *sm = calloc(1, sizeof(*sm));
    if (!sm) return NULL;
    sm->sample_rate = sample_rate;
    sm->window_samples = sample_rate / 10;
    if (sm->window_samples < 1) sm->window_samples = 1;
    sm->dbf_offset = dbf_offset;
    sm->attack = 0.7;
    sm->decay = 0.25;
    sm->peak_decay = 0.05;
    sm->raw_dbfs = sm->smoothed_dbfs = sm->peak_dbfs = sm->ext_nf_db = SIGNAL_METER_NO_VALUE;
    sm->ext_signal_dbfs = SIGNAL_METER_NO_VALUE;
    sm->clk_tck = sysconf(_SC_CLK_TCK);
    sm->num_cores = (int)sysconf(_SC_NPROCESSORS_ONLN);
    if (sm->num_cores < 1) sm->num_cores = 1;
    return sm;
}

void signal_meter_free(signal_meter_t *sm) { free(sm); }

void signal_meter_reset(signal_meter_t *sm)
{
    if (!sm) return;
    sm->power_sum = 0.0;
    sm->count = 0;
    sm->valid = 0;
    sm->raw_dbfs = sm->smoothed_dbfs = sm->peak_dbfs = SIGNAL_METER_NO_VALUE;
    sm->ext_signal_dbfs = SIGNAL_METER_NO_VALUE;
    sm->ext_signal_valid = 0;
}

/* ---- IQ feed (time-domain RMS fallback) ---- */

void signal_meter_feed_iq(signal_meter_t *sm, const float *iq, int n)
{
    if (!sm || !iq || n <= 0) return;
    double sum = sm->power_sum;
    int cnt = sm->count;
    const int win = sm->window_samples;

    for (int i = 0; i < n; i++) {
        float I = iq[i * 2], Q = iq[i * 2 + 1];
        sum += (double)I * I + (double)Q * Q;
        if (++cnt >= win) {
            double dbfs = (sum > 1e-20 * cnt) ? 10.0 * log10(sum / cnt) : -200.0;
            sm->raw_dbfs = dbfs;
            sm->valid = 1;
            if (sm->smoothed_dbfs <= SIGNAL_METER_NO_VALUE + 1.0)
                sm->smoothed_dbfs = dbfs;
            else {
                double a = (dbfs > sm->smoothed_dbfs) ? sm->attack : sm->decay;
                sm->smoothed_dbfs += a * (dbfs - sm->smoothed_dbfs);
            }
            if (sm->smoothed_dbfs > sm->peak_dbfs || sm->peak_dbfs <= SIGNAL_METER_NO_VALUE + 1.0)
                sm->peak_dbfs = sm->smoothed_dbfs;
            else
                sm->peak_dbfs -= sm->peak_decay;
            sum = 0.0;
            cnt = 0;
        }
    }
    sm->power_sum = sum;
    sm->count = cnt;
}

/* ---- FFT-based measurements from sdr_status ---- */

void signal_meter_set_noise_floor(signal_meter_t *sm, double nf_db)
{
    if (sm) { sm->ext_nf_db = nf_db; sm->ext_nf_valid = 1; }
}

void signal_meter_set_signal_power(signal_meter_t *sm, double power_dbfs)
{
    if (sm) {
        sm->ext_signal_dbfs = power_dbfs;
        sm->ext_signal_valid = (power_dbfs > SIGNAL_METER_NO_VALUE + 1.0) ? 1 : 0;
    }
}

/* ---- getters ---- */

double signal_meter_get_level_dbfs(const signal_meter_t *sm)
{
    if (!sm || !sm->valid) return SIGNAL_METER_NO_VALUE;
    /* Prefer FFT-based signal power when available */
    if (sm->ext_signal_valid)
        return sm->ext_signal_dbfs;
    return sm->smoothed_dbfs;
}

double signal_meter_get_level_dbf(const signal_meter_t *sm)
{
    if (!sm || !sm->valid) return SIGNAL_METER_NO_VALUE;
    double signal_dbfs;
    /* Prefer FFT-based signal power when available */
    if (sm->ext_signal_valid)
        signal_dbfs = sm->ext_signal_dbfs;
    else
        signal_dbfs = sm->smoothed_dbfs;
    if (sm->ext_nf_valid && sm->ext_nf_db > SIGNAL_METER_NO_VALUE + 1.0) {
        double snr = signal_dbfs - sm->ext_nf_db;
        return snr;
    }
    double dbf = signal_dbfs + sm->dbf_offset;
    return dbf > 0.0 ? dbf : 0.0;
}

double signal_meter_get_noise_floor(const signal_meter_t *sm)
{
    return (sm && sm->ext_nf_valid) ? sm->ext_nf_db : SIGNAL_METER_NO_VALUE;
}

double signal_meter_get_snr(const signal_meter_t *sm)
{
    if (!sm || !sm->valid || !sm->ext_nf_valid) return 0.0;
    double signal_dbfs;
    if (sm->ext_signal_valid)
        signal_dbfs = sm->ext_signal_dbfs;
    else
        signal_dbfs = sm->smoothed_dbfs;
    double snr = signal_dbfs - sm->ext_nf_db;
    return snr > 0.0 ? snr : 0.0;
}

double signal_meter_get_peak(const signal_meter_t *sm)
{
    return (sm && sm->valid) ? sm->peak_dbfs : SIGNAL_METER_NO_VALUE;
}

double signal_meter_get_raw_dbfs(const signal_meter_t *sm)
{
    return (sm && sm->valid) ? sm->smoothed_dbfs : SIGNAL_METER_NO_VALUE;
}

void signal_meter_set_dbf_offset(signal_meter_t *sm, double offset)
{
    if (sm) sm->dbf_offset = offset;
}

/* ---- stereo pilot passthrough ---- */

void signal_meter_set_stereo_pilot(signal_meter_t *sm, double mag, int locked)
{
    if (sm) { sm->pilot_mag = mag; sm->pilot_locked = locked; }
}

double signal_meter_get_stereo_pilot_mag(const signal_meter_t *sm)
{
    return sm ? sm->pilot_mag : 0.0;
}

int signal_meter_get_stereo_pilot_locked(const signal_meter_t *sm)
{
    return sm ? sm->pilot_locked : 0;
}

/* ---- CPU monitoring ---- */

static void parse_proc_stat(const char *path,
                            unsigned long long *utime,
                            unsigned long long *stime)
{
    *utime = *stime = 0;
    FILE *f = fopen(path, "r");
    if (!f) return;
    char buf[512];
    if (fgets(buf, sizeof(buf), f)) {
        char *p = strrchr(buf, ')');
        if (p) {
            unsigned long long ut, st;
            if (sscanf(p + 2,
                       "%*c %*d %*d %*d %*d %*d %*u "
                       "%*u %*u %*u %*u %llu %llu",
                       &ut, &st) == 2) {
                *utime = ut;
                *stime = st;
            }
        }
    }
    fclose(f);
}

static int64_t get_wall_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

int signal_meter_register_thread(signal_meter_t *sm,
                                 const char *name, pid_t tid)
{
    if (!sm || sm->num_threads >= SIGNAL_METER_MAX_THREADS) return -1;
    int idx = sm->num_threads++;
    strncpy(sm->threads[idx].name, name, sizeof(sm->threads[idx].name) - 1);
    sm->threads[idx].name[sizeof(sm->threads[idx].name) - 1] = '\0';
    sm->threads[idx].tid = tid;
    char path[64];
    snprintf(path, sizeof(path), "/proc/self/task/%d/stat", (int)tid);
    parse_proc_stat(path, &sm->threads[idx].prev_utime,
                    &sm->threads[idx].prev_stime);
    return 0;
}

void signal_meter_update_cpu(signal_meter_t *sm)
{
    if (!sm) return;
    int64_t now = get_wall_us();
    if (sm->cpu_last_us && (now - sm->cpu_last_us) < 900000)
        return;
    int64_t elapsed_us = sm->cpu_last_us ? (now - sm->cpu_last_us) : 0;
    sm->cpu_last_us = now;
    if (elapsed_us <= 0) return;

    double elapsed_ticks = (double)elapsed_us / 1e6 * sm->clk_tck;
    if (elapsed_ticks < 1.0) return;

    unsigned long long ut, st;
    parse_proc_stat("/proc/self/stat", &ut, &st);
    double dticks = (double)((ut + st) - (sm->proc_prev_utime + sm->proc_prev_stime));
    sm->proc_cpu_pct = 100.0 * dticks / elapsed_ticks;
    sm->proc_prev_utime = ut;
    sm->proc_prev_stime = st;
    if (sm->proc_cpu_pct > sm->proc_peak_cpu_pct)
        sm->proc_peak_cpu_pct = sm->proc_cpu_pct;

    for (int i = 0; i < sm->num_threads; i++) {
        char path[64];
        snprintf(path, sizeof(path), "/proc/self/task/%d/stat",
                 (int)sm->threads[i].tid);
        parse_proc_stat(path, &ut, &st);
        dticks = (double)((ut + st) -
                 (sm->threads[i].prev_utime + sm->threads[i].prev_stime));
        sm->threads[i].cpu_pct = 100.0 * dticks / elapsed_ticks;
        sm->threads[i].prev_utime = ut;
        sm->threads[i].prev_stime = st;
        if (sm->threads[i].cpu_pct > sm->threads[i].peak_cpu_pct)
            sm->threads[i].peak_cpu_pct = sm->threads[i].cpu_pct;
    }
}

double signal_meter_get_process_cpu(const signal_meter_t *sm)
{
    return sm ? sm->proc_cpu_pct : 0.0;
}

double signal_meter_get_process_cpu_peak(const signal_meter_t *sm)
{
    return sm ? sm->proc_peak_cpu_pct : 0.0;
}

int signal_meter_get_thread_cpu(const signal_meter_t *sm,
                                signal_meter_thread_info_t *out,
                                int max_out)
{
    if (!sm || !out || max_out <= 0) return 0;
    int n = sm->num_threads < max_out ? sm->num_threads : max_out;
    for (int i = 0; i < n; i++) {
        strncpy(out[i].name, sm->threads[i].name, sizeof(out[i].name) - 1);
        out[i].name[sizeof(out[i].name) - 1] = '\0';
        out[i].tid = sm->threads[i].tid;
        out[i].cpu_pct = sm->threads[i].cpu_pct;
        out[i].peak_cpu_pct = sm->threads[i].peak_cpu_pct;
        out[i].capacity_warn = (sm->threads[i].cpu_pct > 90.0) ? 1 : 0;
    }
    return n;
}

int signal_meter_cpu_warning(const signal_meter_t *sm)
{
    if (!sm) return 0;
    if (sm->proc_cpu_pct > 90.0 * sm->num_cores) return 1;
    for (int i = 0; i < sm->num_threads; i++)
        if (sm->threads[i].cpu_pct > 90.0) return 1;
    return 0;
}
