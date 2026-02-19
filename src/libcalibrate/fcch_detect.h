/* FCCH Burst Detector - adaptive filter + burst scan
 *
 * Detects GSM FCCH bursts using a kalibrate-style adaptive filter
 * error metric (Varma et al.) to find pure-tone neighborhoods, then
 * estimates tone frequency from that neighborhood.
 *
 * This is closer to GSM MS behavior than pure "strongest FFT peak".
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef FCCH_DETECT_H
#define FCCH_DETECT_H

#include <stddef.h>

/* GSM constants */
#define GSM_RATE            (1625000.0 / 6.0)   /* 270833.33 Hz */
#define FCCH_FREQ           (GSM_RATE / 4.0)    /* 67708.33 Hz */

/* Detection thresholds */
#define FCCH_MIN_SNR_DB     6.0     /* Min peak-to-mean SNR on detected burst */
#define FCCH_OFFSET_MAX     40000.0 /* Max allowable offset Hz */
#define FCCH_SEARCH_HZ      20000.0 /* ±20 kHz search window (covers ~10 ppm at 1900 MHz) */
#define FCCH_SCAN_BUF_SAMPLES 16384 /* Complex samples in detector scan buffer */

typedef struct fcch_detect {
    double sample_rate;
    double bin_hz;

    /* Adaptive detector state (kalibrate-style) */
    int adapt_delay;
    int adapt_w_len;
    double adapt_p;
    double adapt_g;
    double adapt_err_ema;
    double *w_real;
    double *w_imag;

    /* Rolling scan buffer (interleaved IQ float) */
    float *scan_iq;
    int scan_count;
    int last_burst_len;         /* accepted burst neighborhood length (samples) */
    int total_frames;           /* processed analysis windows */

    /* Statistics */
    int bursts_found;
    int bursts_rejected;
    double last_snr_db;
    double last_peak_hz;
    int last_peak_bin;
    int detect_fail_count;      /* reserved */
} fcch_detect_t;

int fcch_detect_init(fcch_detect_t *det, double sample_rate);

/* Returns 1 if FCCH detected, 0 if not yet.
 * offset: frequency error from expected FCCH (Hz)
 * snr_db: measured SNR (may be NULL) */
int fcch_detect_process(fcch_detect_t *det, const float *iq_in, int num_samples,
                        double *offset, double *snr_db);

void fcch_detect_reset(fcch_detect_t *det);
void fcch_detect_exit(fcch_detect_t *det);

#endif
