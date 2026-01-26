/* SDR Clock Calibration Library
 *
 * Fast automatic calibration using polyphase channelizer for parallel
 * GSM FCCH tone detection.
 *
 * (C) 2026 Osmocom-analog contributors
 * GPLv3
 */

#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <stdint.h>

/* Calibration signal types */
#define CALIB_SIGNAL_GSM_FCCH   1
#define CALIB_SIGNAL_LTE_PSS    2

/* GSM band identifiers */
#define GSM_BAND_900    1
#define GSM_BAND_1800   2
#define GSM_BAND_1900   3
#define GSM_BAND_850    4

/* Algorithm parameters */
#define CALIB_FFT_SIZE          1024
#define CALIB_AVG_COUNT         100
#define CALIB_AVG_THRESHOLD     10      /* Trim 10% from each end */
#define CALIB_OFFSET_MAX        40000.0 /* Max 40kHz offset */
#define CALIB_MIN_PEAK_MEAN     50.0    /* Minimum peak/mean ratio for valid FCCH */

/* GSM rate constant: 1625000/6 Hz */
#define GSM_RATE                (1625000.0 / 6.0)

/* Calibration result structure */
typedef struct {
	double ppm_offset;          /* Calculated ppm error */
	double hz_offset;           /* Frequency offset in Hz */
	double confidence;          /* 0.0 - 1.0 quality metric */
	double freq_hz;             /* Calibration signal frequency */
	int signal_type;            /* CALIB_SIGNAL_* */
	double signal_strength_db;  /* Relative signal power */
	int gsm_band;               /* GSM_BAND_* if GSM signal */
	int arfcn;                  /* ARFCN if GSM signal */
} calibrate_result_t;

/* Calibration configuration */
typedef struct {
	int scan_gsm;               /* Scan GSM bands */
	int gsm_bands;              /* Bitmask of GSM_BAND_* to scan */
	double timeout_sec;         /* Maximum calibration time */
	int verbosity;              /* 0=quiet, 1=normal, 2=debug */
} calibrate_config_t;

/* Candidate signal during scanning */
typedef struct {
	double center_freq;         /* Channel center frequency */
	double tone_offset;         /* Detected tone offset from center */
	double power;               /* Signal power (linear) */
	double peak_mean_ratio;     /* FFT peak to mean ratio */
	int valid;                  /* Non-zero if likely FCCH */
} calibrate_candidate_t;

/* Initialize default configuration */
void calibrate_config_default(calibrate_config_t *config);

/* Full automatic calibration
 * Returns 0 on success, <0 on error
 */
int calibrate_auto(void *sdr_inst, calibrate_config_t *config, 
                   calibrate_result_t *result);

/* Wideband scan phase - find calibration signal candidates
 * Returns number of candidates found, <0 on error
 */
int calibrate_scan_band(void *sdr_inst, int gsm_band,
                        calibrate_candidate_t *candidates, int max_candidates,
                        int verbosity);

/* Fine offset measurement on single frequency
 * Returns 0 on success, <0 on error
 */
int calibrate_refine(void *sdr_inst, double signal_freq,
                     calibrate_result_t *result, int verbosity);

/* Utility: Convert ARFCN to downlink frequency (Hz) */
double arfcn_to_freq(int arfcn, int gsm_band);

/* Utility: Get band center frequency and bandwidth for wideband scan */
void gsm_band_params(int gsm_band, double *center_freq, double *bandwidth,
                     int *arfcn_start, int *arfcn_end);

#endif /* CALIBRATE_H */
