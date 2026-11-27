/* SDR Clock Calibration Library
 *
 * GSM base stations as the frequency reference: the band's power scan,
 * then windows of channels (as many as the CPU budget allows) measured in
 * parallel: FCCH offset per channel, SCH/BCCH identity as the proof of a
 * real cell. Stops after enough confirmed towers agree.
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

#include "gsm_sch.h"

/* GSM band identifiers (single values, not bitmask) */
#define GSM_BAND_900    1
#define GSM_BAND_1800   2
#define GSM_BAND_1900   3
#define GSM_BAND_850    4
#define GSM_BAND_ALL    0       /* Scan all supported bands */

/* Maximum bands we can scan */
#define CALIB_MAX_BANDS         4

/* carriers measured in one run (all of them with --full) */
#define CALIB_MAX_CELLS         256

/* towers agree if their offsets are within this (GSM base stations: +-0.05
 * ppm each, 3GPP TS 45.010) */
#define CALIB_AGREE_PPM         0.1

/* One measured carrier: its offset and its identity */
typedef struct {
	int arfcn;
	int gsm_band;
	double freq_hz;
	double hz_offset;           /* FCCH offset (trimmed mean) */
	double ppm_offset;
	double stddev_hz;
	int num_measurements;
	double level_dbfs;          /* channel level (at the RX gain in use) */
	double snr_db;              /* FCCH SNR, mean of the measurements */
	int decoded;                /* identity decoding was run */
	int confirmed;              /* SCH decodes consistent: a base station */
	double confirm_sec;         /* decoding time until confirmed */
	int agrees;                 /* within CALIB_AGREE_PPM of the result */
	gsm_cell_ident_t ident;     /* BSIC, FN, SI counts, MCC/MNC/LAC/CI */
} calibrate_cell_t;

/* Result of a run */
typedef struct {
	int valid;                  /* a result (enough agreeing towers, or what there is) */
	double ppm_offset;          /* mean of the agreeing towers */
	int num_towers;             /* confirmed towers measured */
	int num_agree;              /* of them within CALIB_AGREE_PPM of the result */
	int disagree;               /* confirmed towers outside it */
	double spread_ppm;          /* max - min of the confirmed towers */
	double time_to_result;      /* s from the start until the result was known */
	/* how it ran */
	int samplerate;
	int channels_per_window;
	double cpu_per_channel;     /* share of one core per channel, measured */
	double cpu_transport;       /* share of one core for the reading threads */
	int windows;
	int stopped_weak;           /* the weaker carriers left out (none confirmed in a row) */
	calibrate_cell_t cells[CALIB_MAX_CELLS]; /* carriers measured, in order */
	int num_cells;
} calibrate_result_t;

/* Calibration configuration */
typedef struct {
	int gsm_bands;              /* GSM_BAND_* or GSM_BAND_ALL */
	double timeout_sec;         /* most time per window */
	int verbosity;              /* 0=quiet, 1=normal, 2=debug */
	int num_channels;           /* channels per window, 0 = as the CPU budget allows */
	double cpu_percent;         /* CPU budget for the channels, % of one core */
	int towers;                 /* stop after this many agreeing confirmed towers */
	int full;                   /* measure every carrier found */
	int forced_arfcn;           /* >0: only this ARFCN */
	int confirm;                /* 1=use only carriers whose SCH identity decodes */
	double decode_timeout_sec;  /* time allowed for the identity decoding */
	int decode_bcch;            /* also decode BCCH System Information */
	/* the device's usable part of the stream each side of the centre:
	 * this fraction of the rate, at most usable_max_hz (0: no limit);
	 * channels and scan steps stay inside it (docs/SDR-FRONTENDS.md) */
	double usable_fraction;
	double usable_max_hz;
	/* reopen the SDR at another rate (NULL: the rate stays): after the
	 * cost is measured at the widest, the rate with the most channels per
	 * window in the budget */
	int (*reopen)(int samplerate);
} calibrate_config_t;

/* Initialize default configuration */
void calibrate_config_default(calibrate_config_t *config);

/* The SDR to receive from: a libsdr instance (sdr_open() with channels = 0,
 * started), read in chunks of at most buffer_size samples. */
void calibrate_set_sdr(void *sdr, int buffer_size);

/* The whole run: scan the band(s), measure windows of carriers until
 * config->towers confirmed towers agree (or all are measured with --full).
 * Returns 0 if there is a result, <0 if not (result->cells still lists
 * what was measured). start_info: the start of the line logged once the
 * cost per channel is measured (the rate, the bus load). */
int calibrate_auto(int samplerate, calibrate_config_t *config, calibrate_result_t *result,
		   const char *start_info);

/* Utility: Convert ARFCN to downlink frequency (Hz) */
double arfcn_to_freq(int arfcn, int gsm_band);

/* Utility: Get band center frequency and bandwidth for wideband scan */
void gsm_band_params(int gsm_band, double *center_freq, double *bandwidth,
                     int *arfcn_start, int *arfcn_end);

/* Utility: band name string */
const char *gsm_band_name(int gsm_band);

#endif /* CALIBRATE_H */
