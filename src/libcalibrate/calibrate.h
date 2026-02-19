/* SDR Clock Calibration Library
 *
 * Fast automatic calibration using polyphase channelizer for parallel
 * GSM FCCH tone detection.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <stdint.h>

/* Calibration signal types */
#define CALIB_SIGNAL_GSM_FCCH   1
#define CALIB_SIGNAL_LTE_PSS    2

/* GSM band identifiers (single values, not bitmask) */
#define GSM_BAND_900    1
#define GSM_BAND_1800   2
#define GSM_BAND_1900   3
#define GSM_BAND_850    4
#define GSM_BAND_ALL    0       /* Scan all supported bands */

/* Maximum parallel channels for channelizer */
#define CALIB_MAX_CHANNELS      5

/* Maximum bands we can scan */
#define CALIB_MAX_BANDS         4

/* Calibration result structure (per-band) */
typedef struct {
	double ppm_offset;          /* Calculated ppm error */
	double hz_offset;           /* Frequency offset in Hz */
	double confidence;          /* 0.0 - 1.0 quality metric */
	double freq_hz;             /* Calibration signal frequency */
	double stddev_hz;           /* Standard deviation of measurements */
	int signal_type;            /* CALIB_SIGNAL_* */
	double signal_strength_db;  /* Relative signal power */
	int gsm_band;               /* GSM_BAND_* if GSM signal */
	int arfcn;                  /* Best ARFCN if GSM signal */
	int num_measurements;       /* Number of measurements collected */
	int valid;                  /* 1 if this result is valid */
} calibrate_result_t;

/* Calibration configuration */
typedef struct {
	int scan_gsm;               /* Scan GSM bands */
	int gsm_bands;              /* GSM_BAND_* or GSM_BAND_ALL */
	double timeout_sec;         /* Maximum calibration time per band */
	int verbosity;              /* 0=quiet, 1=normal, 2=debug */
	int num_channels;           /* Number of parallel channels (1..CALIB_MAX_CHANNELS) */
	int fast_math;              /* 0=double precision, 1=fixed-point (lower CPU) */
	int use_channelizer;        /* 1=use polyphase resampler path (lower CPU) */
	int forced_arfcn;           /* >0: force this ARFCN for FCCH detection */
	int probe_sequential;       /* 1=probe ARFCNs one-by-one; stop on clear FCCH */
} calibrate_config_t;

/* Initialize default configuration */
void calibrate_config_default(calibrate_config_t *config);

/* Calibrate a single band. Returns 0 on success, <0 on error. */
int calibrate_band(double center_freq, int samplerate,
                   int gsm_band, calibrate_config_t *config,
                   calibrate_result_t *result);

/* Full automatic calibration — scans one or all bands.
 * results: array of CALIB_MAX_BANDS results (one per band)
 * num_results: number of valid results written
 * Returns 0 on success (at least one band), <0 on total failure. */
int calibrate_auto(double center_freq, int samplerate,
                   calibrate_config_t *config,
                   calibrate_result_t *results, int *num_results);

/* Utility: Convert ARFCN to downlink frequency (Hz) */
double arfcn_to_freq(int arfcn, int gsm_band);

/* Utility: Get band center frequency and bandwidth for wideband scan */
void gsm_band_params(int gsm_band, double *center_freq, double *bandwidth,
                     int *arfcn_start, int *arfcn_end);

/* Utility: band name string */
const char *gsm_band_name(int gsm_band);

#endif /* CALIBRATE_H */
