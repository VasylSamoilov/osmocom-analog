/* FCCH Burst Detector Implementation
 *
 * Adaptive filter-based FCCH detection, ported from kalibrate.
 * Reference: "Robust Frequency Burst Detection Algorithm for GSM/GPRS"
 *            by Varma, Sahu, and Charan.
 *
 * (C) 2010 Joshua Lackey (original kalibrate)
 * (C) 2026 Osmocom-analog contributors (C port)
 * GPLv3
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "fcch_detect.h"
#include "../libfft/fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Buffer operations */
static int buffer_init(fcch_buffer_t *buf, int size)
{
    buf->data = calloc(size * 2, sizeof(float));  /* I/Q interleaved */
    if (!buf->data)
        return -ENOMEM;
    buf->size = size;
    buf->head = 0;
    buf->count = 0;
    return 0;
}

static void buffer_exit(fcch_buffer_t *buf)
{
    free(buf->data);
    memset(buf, 0, sizeof(*buf));
}

static void buffer_flush(fcch_buffer_t *buf)
{
    buf->head = 0;
    buf->count = 0;
}

static int buffer_write(fcch_buffer_t *buf, float i, float q)
{
    if (buf->count >= buf->size)
        return 0;
    int idx = (buf->head + buf->count) % buf->size;
    buf->data[idx * 2] = i;
    buf->data[idx * 2 + 1] = q;
    buf->count++;
    return 1;
}

static void buffer_get(fcch_buffer_t *buf, int offset, float *i, float *q)
{
    int idx = (buf->head + offset) % buf->size;
    *i = buf->data[idx * 2];
    *q = buf->data[idx * 2 + 1];
}

static void buffer_purge(fcch_buffer_t *buf, int count)
{
    if (count > buf->count)
        count = buf->count;
    buf->head = (buf->head + count) % buf->size;
    buf->count -= count;
}

/* Vector norm squared */
static double vectornorm2(fcch_buffer_t *buf, int start, int len)
{
    double sum = 0;
    for (int i = 0; i < len; i++) {
        float re, im;
        buffer_get(buf, start + i, &re, &im);
        sum += re * re + im * im;
    }
    return sum;
}

/* Compute next normalized error sample */
static int next_norm_error(fcch_detect_t *det, float *error)
{
    int n = FCCH_FILTER_LEN - 1;
    int required = n + FCCH_DELAY + 1;
    
    if (det->x_buf.count < required)
        return -1;  /* Not enough samples */
    
    /* Calculate energy for gain adaptation */
    double E = vectornorm2(&det->x_buf, 0, FCCH_FILTER_LEN);
    if (det->G >= 2.0 / E)
        det->G = 1.0 / E;
    
    /* Calculate filtered value: y = sum(conj(w[i]) * x[n-i]) */
    double y_re = 0, y_im = 0;
    for (int i = 0; i < FCCH_FILTER_LEN; i++) {
        float x_re, x_im;
        buffer_get(&det->x_buf, n - i, &x_re, &x_im);
        /* conj(w) * x = (w_re - j*w_im) * (x_re + j*x_im) */
        y_re += det->w_real[i] * x_re + det->w_imag[i] * x_im;
        y_im += det->w_real[i] * x_im - det->w_imag[i] * x_re;
    }
    
    /* Get desired signal (delayed input) */
    float d_re, d_im;
    buffer_get(&det->x_buf, n + FCCH_DELAY, &d_re, &d_im);
    
    /* Calculate error */
    double e_re = d_re - y_re;
    double e_im = d_im - y_im;
    
    /* Update filter weights with opposite gradient */
    for (int i = 0; i < FCCH_FILTER_LEN; i++) {
        float x_re, x_im;
        buffer_get(&det->x_buf, n - i, &x_re, &x_im);
        /* w += G * conj(e) * x */
        det->w_real[i] += det->G * (e_re * x_re + e_im * x_im);
        det->w_imag[i] += det->G * (e_re * x_im - e_im * x_re);
    }
    
    /* Update average error power */
    double e_power = e_re * e_re + e_im * e_im;
    E /= FCCH_FILTER_LEN;
    det->e_avg = (1.0 - det->p) * det->e_avg + det->p * e_power;
    
    /* Return normalized error */
    if (error)
        *error = (E > 0) ? (float)(det->e_avg / E) : 0;
    
    /* Remove processed sample */
    buffer_purge(&det->x_buf, 1);
    
    return 0;
}

/* Detect frequency using FFT */
static double freq_detect(fcch_detect_t *det, const float *iq, int len, double *pm)
{
    int fft_len = (len < FCCH_FFT_SIZE) ? len : FCCH_FFT_SIZE;
    int m = 0, n = FCCH_FFT_SIZE;
    
    /* Calculate log2 for FFT */
    while (n > 1) { n >>= 1; m++; }
    
    /* Copy samples to FFT buffers */
    for (int i = 0; i < fft_len; i++) {
        det->fft_real[i] = iq[i * 2];
        det->fft_imag[i] = iq[i * 2 + 1];
    }
    for (int i = fft_len; i < FCCH_FFT_SIZE; i++) {
        det->fft_real[i] = 0;
        det->fft_imag[i] = 0;
    }
    
    /* Execute FFT */
    fft_process(1, m, det->fft_real, det->fft_imag);
    
    /* Find peak and calculate mean */
    double peak_power = 0;
    double sum_power = 0;
    int peak_bin = 0;
    
    for (int i = 0; i < FCCH_FFT_SIZE; i++) {
        double power = det->fft_real[i] * det->fft_real[i] + 
                       det->fft_imag[i] * det->fft_imag[i];
        sum_power += power;
        if (power > peak_power) {
            peak_power = power;
            peak_bin = i;
        }
    }
    
    double mean_power = (sum_power - peak_power) / (FCCH_FFT_SIZE - 1);
    if (pm)
        *pm = (mean_power > 0) ? (peak_power / mean_power) : 0;
    
    /* Parabolic interpolation for sub-bin accuracy */
    int i0 = (peak_bin - 1 + FCCH_FFT_SIZE) % FCCH_FFT_SIZE;
    int i2 = (peak_bin + 1) % FCCH_FFT_SIZE;
    double y0 = det->fft_real[i0] * det->fft_real[i0] + det->fft_imag[i0] * det->fft_imag[i0];
    double y1 = peak_power;
    double y2 = det->fft_real[i2] * det->fft_real[i2] + det->fft_imag[i2] * det->fft_imag[i2];
    
    double delta = 0;
    double denom = y0 - 2.0 * y1 + y2;
    if (denom != 0)
        delta = 0.5 * (y0 - y2) / denom;
    
    double peak_index = peak_bin + delta;
    
    /* Convert to frequency */
    double freq = peak_index * (det->sample_rate / FCCH_FFT_SIZE);
    if (peak_index > FCCH_FFT_SIZE / 2)
        freq -= det->sample_rate;
    
    return freq;
}

int fcch_detect_init(fcch_detect_t *det, double sample_rate)
{
    int rc;
    
    memset(det, 0, sizeof(*det));
    det->sample_rate = sample_rate;
    det->sps = sample_rate / GSM_RATE;
    det->min_burst_len = (int)(100 * det->sps);  /* 100 symbols minimum */
    det->fcch_burst_len = (int)(148 * det->sps); /* Full FCCH burst */
    
    /* Initialize adaptive filter */
    det->G = 0.01;
    det->p = 0.01;
    det->e_avg = 0;
    memset(det->w_real, 0, sizeof(det->w_real));
    memset(det->w_imag, 0, sizeof(det->w_imag));
    
    /* Allocate buffers - enough for 12 frames + 1 burst */
    int buf_size = (int)ceil((12 * 8 * 156.25 + 156.25) * det->sps);
    
    rc = buffer_init(&det->x_buf, buf_size);
    if (rc < 0) return rc;
    
    rc = buffer_init(&det->e_buf, buf_size);
    if (rc < 0) {
        buffer_exit(&det->x_buf);
        return rc;
    }
    
    /* Allocate FFT buffers */
    det->fft_real = calloc(FCCH_FFT_SIZE, sizeof(double));
    det->fft_imag = calloc(FCCH_FFT_SIZE, sizeof(double));
    if (!det->fft_real || !det->fft_imag) {
        fcch_detect_exit(det);
        return -ENOMEM;
    }
    
    return 0;
}

int fcch_detect_process(fcch_detect_t *det, const float *iq_in, int num_samples,
                        double *offset)
{
    float errors[8192];  /* Error values for this batch */
    int e_count = 0;
    double e_sum = 0;
    
    /* Add samples to buffer and compute errors */
    for (int i = 0; i < num_samples && e_count < 8192; i++) {
        buffer_write(&det->x_buf, iq_in[i * 2], iq_in[i * 2 + 1]);
        
        float e;
        if (next_norm_error(det, &e) == 0) {
            errors[e_count++] = e;
            e_sum += e;
        }
    }
    
    if (e_count < det->min_burst_len)
        return 0;  /* Not enough samples */
    
    /* Calculate average error and threshold */
    double avg_error = e_sum / e_count;
    double limit = 0.7 * avg_error;
    
    /* Find low-error neighborhoods */
    int low_count = 0;
    int best_start = -1;
    int best_len = 0;
    
    for (int i = 0; i < e_count; i++) {
        if (errors[i] < limit) {
            low_count++;
        } else {
            if (low_count > best_len) {
                best_len = low_count;
                best_start = i - low_count;
            }
            low_count = 0;
        }
    }
    if (low_count > best_len) {
        best_len = low_count;
        best_start = e_count - low_count;
    }
    
    /* Check if we found a long enough low-error region */
    if (best_len < det->min_burst_len || best_start < 0)
        return 0;
    
    /* Get the samples for this region from input */
    int fft_len = (best_len < det->fcch_burst_len) ? best_len : det->fcch_burst_len;
    if (best_start + fft_len > num_samples)
        fft_len = num_samples - best_start;
    
    if (fft_len < det->min_burst_len)
        return 0;
    
    /* Detect frequency */
    double pm;
    double freq = freq_detect(det, iq_in + best_start * 2, fft_len, &pm);
    
    /* Check peak/mean threshold */
    if (pm < FCCH_MIN_PM) {
        det->bursts_rejected++;
        return 0;
    }
    
    /* Valid FCCH detected! */
    det->bursts_found++;
    
    /* Calculate offset from expected FCCH frequency */
    double fcch_offset = freq - FCCH_FREQ;
    
    /* Sanity check */
    if (fabs(fcch_offset) > FCCH_OFFSET_MAX)
        return 0;
    
    if (offset)
        *offset = fcch_offset;
    
    return 1;
}

void fcch_detect_reset(fcch_detect_t *det)
{
    buffer_flush(&det->x_buf);
    buffer_flush(&det->e_buf);
    det->G = 0.01;
    det->e_avg = 0;
    memset(det->w_real, 0, sizeof(det->w_real));
    memset(det->w_imag, 0, sizeof(det->w_imag));
}

void fcch_detect_exit(fcch_detect_t *det)
{
    buffer_exit(&det->x_buf);
    buffer_exit(&det->e_buf);
    free(det->fft_real);
    free(det->fft_imag);
    memset(det, 0, sizeof(*det));
}
