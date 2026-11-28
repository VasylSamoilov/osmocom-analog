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

/* GSM band identifiers (gsm_bands.c, 3GPP TS 45.005 clause 2) */
#define GSM_BAND_900    1	/* P-GSM 900 */
#define GSM_BAND_1800   2
#define GSM_BAND_1900   3
#define GSM_BAND_850    4
#define GSM_BAND_450    5
#define GSM_BAND_480    6
#define GSM_BAND_750    7
#define GSM_BAND_E900   8
#define GSM_BAND_R900   9

typedef struct gsm_band_def {
	int		id;
	const char	*name;		/* GSM-900, DCS-1800, ... */
	const char	*arg;		/* --band name */
	struct {
		int	first, last;	/* ARFCNs (last < first: none) */
		double	f_ref;		/* downlink of n_ref (Hz) */
		int	n_ref;
	} run[2];
	int		dynamic;	/* numbering not fixed by 45.005 (GSM 750) */
} gsm_band_def_t;

/* the table, in order (i from 0; NULL after the last) */
const gsm_band_def_t *gsm_band_at(int i);
const gsm_band_def_t *gsm_band_def(int id);
/* id of a --band name, -1 if none */
int gsm_band_parse(const char *arg);
int gsm_freq_to_arfcn(int gsm_band, double freq_hz);
int gsm_band_dl_range(int gsm_band, double *f_lo, double *f_hi);

/* systems */
#define CALIB_GSM	0
#define CALIB_LTE	1

/* the bands of a run (GSM and LTE; LTE: all of 36.101) */
#define CALIB_MAX_BANDS         128

/* one band to measure: its carriers within [f_lo, f_hi] (the part in reach) */
typedef struct calib_band {
	int		system;		/* CALIB_GSM, CALIB_LTE */
	int		id;		/* GSM_BAND_* or the LTE band number */
	double		f_lo, f_hi;	/* downlink carrier frequencies (Hz) */
	int		scan;		/* 0: inside another band of the run (only names carriers) */
} calib_band_t;

/* carriers measured in one run (all of them with --full) */
#define CALIB_MAX_CELLS         256

/* towers agree if their offsets are within this (GSM base stations: +-0.05
 * ppm each, 3GPP TS 45.010) */
#define CALIB_AGREE_PPM         0.1

/* One measured carrier: its offset and its identity */
typedef struct {
	int system;                 /* CALIB_GSM, CALIB_LTE */
	int arfcn;                  /* LTE: the EARFCN */
	int gsm_band;               /* LTE: the band number */
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
	double weight;              /* in the result: 1, less when uncertain */
	double carrier_ppm;         /* the carrier's offset (the prior for LTE searches) */
	gsm_cell_ident_t ident;     /* BSIC, FN, SI counts, MCC/MNC/LAC/CI */
	struct {
		int pci, n_rb, ports, tdd;
		double rsrp_dbfs, rsrq_db, sinr_db;
		double alias_margin;    /* cfo_alias_margin (standard errors) */
		double timing_ppm, timing_std_ppm;
		int timing;             /* ppm_offset is the timing ppm (RTL) */
	} lte;
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
	double lte_idle_sec;        /* LTE: idle between windows to stay within --cpu */
	calibrate_cell_t cells[CALIB_MAX_CELLS]; /* carriers measured, in order */
	int num_cells;
} calibrate_result_t;

/* Calibration configuration */
typedef struct {
	calib_band_t bands[CALIB_MAX_BANDS]; /* in the order measured */
	int num_bands;
	int lte_timing;             /* LTE: the frame timing ppm, not the carrier's
	                             * (a tuner whose LO is off its setting: RTL) */
	double timeout_sec;        /* most time per window */
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
	/* another RX rate without a reopen (NULL: not possible), at most the
	 * rate opened; window_stage: CALIB_STAGE_* what a window may use */
	int (*set_rate)(int samplerate);
	int window_stage;
} calibrate_config_t;

/* a window at the narrowest rate that holds its carriers (set_rate) */
#define CALIB_STAGE_RATE	1
/* the window's channels on one shared first stage (libsdr's engine) */
#define CALIB_STAGE_SHARED	2

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

/* Utility: band name string */
const char *gsm_band_name(int gsm_band);

/* the half width of the stream an LTE carrier needs (its centre 6 RB and
 * the offset): reach and rate checks */
#define CALIB_LTE_HALF_HZ	600e3

#endif /* CALIBRATE_H */
