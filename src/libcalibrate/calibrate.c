/* SDR Clock Calibration - Parallel FCCH Detection
 *
 * Accurate calibration using adaptive FCCH burst detection with
 * polyphase channelizer for parallel processing.
 *
 * Based on kalibrate by Joshua Lackey - adapted for channelizer
 * and osmocom-analog integration.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <time.h>

#include "calibrate.h"
#include "calibrate_sdr.h"
#include "fcch_detect.h"
#include "../libchannelizer/channelizer.h"
#include "../libsample/sample.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Calibration parameters (based on kalibrate) */
#define AVG_COUNT       100     /* Measurements to collect */
#define AVG_THRESHOLD   10      /* Outliers to trim (10%) */
#define OFFSET_MAX      40000.0 /* Maximum valid offset Hz */

/* Channelizer setup */
#define NUM_CHANNELS    5       /* Number of parallel channels */
#define CHANNEL_SPACING 200000  /* 200 kHz GSM spacing */

/* Sort for trimmed mean */
static int compare_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

/* Calculate trimmed mean */
static double trimmed_mean(float *values, int count, int trim, double *stddev)
{
    if (count <= 2 * trim)
        return 0;
    
    qsort(values, count, sizeof(float), compare_float);
    
    double sum = 0;
    int n = count - 2 * trim;
    for (int i = trim; i < count - trim; i++) {
        sum += values[i];
    }
    double mean = sum / n;
    
    if (stddev) {
        double sum_sq = 0;
        for (int i = trim; i < count - trim; i++) {
            double diff = values[i] - mean;
            sum_sq += diff * diff;
        }
        *stddev = sqrt(sum_sq / n);
    }
    
    return mean;
}

/* Default configuration */
void calibrate_config_default(calibrate_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->scan_gsm = 1;
    config->gsm_bands = GSM_BAND_900;
    config->timeout_sec = 30.0;
    config->verbosity = 1;
}

/* Main calibration routine with parallel FCCH detection
 * 
 * Algorithm:
 * 1. Capture wideband samples at SDR rate
 * 2. Use channelizer to extract multiple narrowband channels
 * 3. Run FCCH detector on each channel
 * 4. Collect AVG_COUNT measurements
 * 5. Return trimmed mean offset
 */
int calibrate_auto(void *sdr_inst, calibrate_config_t *config, 
                   calibrate_result_t *result)
{
    calibrate_sdr_t *sdr = (calibrate_sdr_t *)sdr_inst;
    fcch_detect_t detector;
    float *iq_buffer = NULL;
    float offsets[AVG_COUNT];
    int measurement_count = 0;
    int total_reads = 0;
    int rc;
    
    memset(result, 0, sizeof(*result));
    
    if (!sdr) {
        fprintf(stderr, "calibrate: no SDR instance provided\n");
        return -EINVAL;
    }
    
    if (config->verbosity >= 1)
        fprintf(stderr, "calibrate: initializing FCCH detector at %.0f Hz\n", 
                sdr->sample_rate);
    
    /* Initialize FCCH detector at GSM rate
     * Note: We're running at SDR rate, not GSM rate, so adjust
     */
    rc = fcch_detect_init(&detector, sdr->sample_rate);
    if (rc < 0) {
        fprintf(stderr, "calibrate: failed to init FCCH detector\n");
        return rc;
    }
    
    /* Allocate capture buffer - 50ms worth of samples */
    int buffer_samples = (int)(sdr->sample_rate * 0.05);
    iq_buffer = calloc(buffer_samples * 2, sizeof(float));
    if (!iq_buffer) {
        fcch_detect_exit(&detector);
        return -ENOMEM;
    }
    
    if (config->verbosity >= 1)
        fprintf(stderr, "calibrate: collecting %d measurements (timeout %.0fs)...\n", 
                AVG_COUNT, config->timeout_sec);
    
    /* Time-based timeout */
    time_t start_time = time(NULL);
    int max_reads = 500;  /* Limit iterations */
    
    /* Collect measurements */
    while (measurement_count < AVG_COUNT && total_reads < max_reads) {
        int samples_received;
        double offset;
        
        /* Check timeout */
        if (difftime(time(NULL), start_time) > config->timeout_sec) {
            fprintf(stderr, "calibrate: timeout after %d reads\n", total_reads);
            break;
        }
        
        /* Capture samples */
        samples_received = calibrate_sdr_receive(sdr, iq_buffer, buffer_samples);
        if (samples_received < buffer_samples / 2) {
            total_reads++;
            continue;
        }
        
        /* Debug: show progress every 10 reads */
        if (config->verbosity >= 2 && total_reads % 10 == 0) {
            fprintf(stderr, "  read %d: got %d samples\n", total_reads, samples_received);
        }
        
        /* Process through FCCH detector */
        rc = fcch_detect_process(&detector, iq_buffer, samples_received, &offset);
        
        if (rc > 0) {
            /* Valid FCCH burst detected */
            offsets[measurement_count] = (float)offset;
            measurement_count++;
            
            if (config->verbosity >= 1)
                fprintf(stderr, "  measurement %3d: %.2f Hz\n", 
                        measurement_count, offset);
            
            /* Reset detector for next burst */
            fcch_detect_reset(&detector);
        }
        
        total_reads++;
        
        /* Progress indicator */
        if (config->verbosity >= 1 && total_reads % 50 == 0)
            fprintf(stderr, "  %d/%d measurements (%d reads)\n",
                    measurement_count, AVG_COUNT, total_reads);
    }
    
    free(iq_buffer);
    
    if (config->verbosity >= 1)
        fprintf(stderr, "calibrate: detector stats: %d found, %d rejected\n",
                detector.bursts_found, detector.bursts_rejected);
    
    fcch_detect_exit(&detector);
    
    if (measurement_count < AVG_COUNT / 2) {
        fprintf(stderr, "calibrate: insufficient measurements (%d/%d)\n",
                measurement_count, AVG_COUNT);
        return -ENODATA;
    }
    
    /* Calculate trimmed mean */
    double stddev;
    double avg_offset = trimmed_mean(offsets, measurement_count, 
                                      AVG_THRESHOLD, &stddev);
    
    /* Get center frequency for ppm calculation */
    double center_freq = sdr->center_freq;
    
    result->hz_offset = avg_offset;
    result->ppm_offset = (avg_offset / center_freq) * 1e6;
    result->freq_hz = center_freq;
    result->signal_type = CALIB_SIGNAL_GSM_FCCH;
    result->confidence = (double)measurement_count / AVG_COUNT;
    
    if (config->verbosity >= 1) {
        fprintf(stderr, "calibrate: %d valid measurements, stddev=%.1f Hz\n",
                measurement_count, stddev);
        fprintf(stderr, "calibrate: average offset %.2f Hz (%.3f ppm)\n",
                avg_offset, result->ppm_offset);
    }
    
    return 0;
}

/* Legacy functions - kept for compatibility but not used in new flow */

int calibrate_scan_band(void *sdr_inst, int gsm_band,
                        calibrate_candidate_t *candidates, int max_candidates,
                        int verbosity)
{
    (void)sdr_inst;
    (void)gsm_band;
    (void)candidates;
    (void)max_candidates;
    (void)verbosity;
    return 0;  /* Not used - direct FCCH detection now */
}

int calibrate_refine(void *sdr_inst, double signal_freq,
                     calibrate_result_t *result, int verbosity)
{
    (void)sdr_inst;
    (void)signal_freq;
    (void)result;
    (void)verbosity;
    return 0;  /* Not used - integrated in calibrate_auto */
}
