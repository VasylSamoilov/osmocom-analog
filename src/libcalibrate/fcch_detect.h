/* FCCH Burst Detector
 *
 * Adaptive filter-based FCCH (Frequency Correction Channel) burst detector.
 * Ported from kalibrate by Joshua Lackey.
 *
 * The FCCH is a pure sine wave at GSM_RATE/4 (67.7 kHz). This detector
 * uses an adaptive filter to track the signal and identify low-error
 * "neighborhoods" where a pure tone exists.
 *
 * (C) 2010 Joshua Lackey (original kalibrate)
 * (C) 2026 Osmocom-analog contributors (C port)
 * GPLv3
 */

#ifndef FCCH_DETECT_H
#define FCCH_DETECT_H

#include <stddef.h>

/* GSM constants */
#define GSM_RATE            (1625000.0 / 6.0)   /* 270833.33 Hz */
#define FCCH_FREQ           (GSM_RATE / 4.0)    /* 67708.33 Hz */

/* Detection parameters (from kalibrate) */
#define FCCH_FFT_SIZE       1024
#define FCCH_MIN_PM         50.0    /* Minimum peak/mean for valid detection */
#define FCCH_OFFSET_MAX     40000.0 /* Maximum allowable offset in Hz */

/* Adaptive filter parameters */
#define FCCH_FILTER_LEN     21      /* Adaptive filter length (odd) */
#define FCCH_DELAY          4       /* Prediction delay */

/* Circular buffer for samples */
typedef struct fcch_buffer {
    float *data;        /* Interleaved I/Q */
    int size;           /* Buffer size in samples */
    int head;           /* Write position */
    int count;          /* Samples in buffer */
} fcch_buffer_t;

/* FCCH detector state */
typedef struct fcch_detect {
    double sample_rate;         /* Input sample rate */
    double sps;                 /* Samples per GSM symbol */
    int min_burst_len;          /* Minimum burst length in samples */
    int fcch_burst_len;         /* FCCH burst length in samples */
    
    /* Adaptive filter state */
    float w_real[FCCH_FILTER_LEN];  /* Filter weights (real) */
    float w_imag[FCCH_FILTER_LEN];  /* Filter weights (imag) */
    double G;                   /* Adaptive gain */
    double p;                   /* Smoothing factor */
    double e_avg;               /* Average error power */
    
    /* Sample buffers */
    fcch_buffer_t x_buf;        /* Input sample buffer */
    fcch_buffer_t e_buf;        /* Error buffer */
    
    /* FFT for frequency detection */
    double *fft_real;
    double *fft_imag;
    
    /* Statistics */
    int bursts_found;
    int bursts_rejected;
} fcch_detect_t;

/* Initialize FCCH detector
 * sample_rate: Input sample rate in Hz (should be ~270833 for GSM)
 * Returns 0 on success, <0 on error
 */
int fcch_detect_init(fcch_detect_t *det, double sample_rate);

/* Process samples and detect FCCH burst
 * iq_in: Interleaved I/Q samples
 * num_samples: Number of sample pairs
 * offset: Output frequency offset from expected FCCH (if detected)
 * Returns: 1 if FCCH burst detected, 0 if not, <0 on error
 */
int fcch_detect_process(fcch_detect_t *det, const float *iq_in, int num_samples,
                        double *offset);

/* Reset detector state (keep parameters) */
void fcch_detect_reset(fcch_detect_t *det);

/* Free detector resources */
void fcch_detect_exit(fcch_detect_t *det);

#endif /* FCCH_DETECT_H */
