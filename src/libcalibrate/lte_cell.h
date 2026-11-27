/* LTE cell search, identity proof (MIB) and frequency offset from one carrier
 *
 * Input is the complex baseband of one LTE downlink carrier, centred on its
 * EARFCN frequency, at 1.92 MS/s (only the centre 6 resource blocks are
 * used, so the carrier's bandwidth does not matter). The samples come from
 * the shared SDR path (sdr_open + sdr_engine / libchannelizer); this module
 * has no radio access and no resampler.
 *
 * Chain (all in lte_cell.c / lte_phy.c, no external library):
 *   search: PSS correlation (3 N_ID2 x carrier offset hypotheses) over 4
 *     half-frames, coarse offset from the best hypothesis, then the phase
 *     between the two PSS halves;
 *   SSS: N_ID1, subframe 0/5 (frame timing), FDD/TDD and normal/extended CP
 *     from the SSS position relative to the PSS;
 *   tracking per frame: PSS timing every half-frame (sample clock drift),
 *     CRS channel estimate, carrier offset from the CRS phase (slot to slot,
 *     then frame to frame), RSRP / RSRQ / SINR from the CRS, PBCH
 *     (transmit diversity 1/2/4 ports), BCH decode and CRC with the
 *     antenna-port mask -> MIB (bandwidth, PHICH, SFN).
 * A cell is confirmed by two or more MIB decodes whose SFN advances as the
 * time between them says.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef LTE_CELL_H
#define LTE_CELL_H

#include <stdint.h>
#include <complex.h>

#define LTE_CELL_MAX	6	/* cells tracked per carrier */

/* what is known about one cell */
typedef struct lte_cell_info {
	int	pci;		/* physical cell identity 0..503 */
	int	tdd;		/* frame structure type 2 */
	int	ext_cp;		/* extended cyclic prefix */
	double	pss_metric;	/* normalized PSS correlation at the search (0..1) */
	double	sss_score;	/* best SSS correlation / rms of the other hypotheses */
	int	frames;		/* frames processed */
	int	lost;		/* tracking given up */
	/* MIB */
	int	mib_good;	/* decodes with valid CRC and re-encode agreement */
	int	mib_bad;	/* frames that did not decode */
	int	mib_consistent;	/* good decodes agreeing with the reference decode */
	int	mib_inconsistent;
	int	ports;		/* antenna ports from the CRC mask, 0 if unknown */
	int	n_rb;		/* downlink bandwidth in resource blocks */
	int	phich_ext;
	int	phich_res;
	int	spare;
	int	sfn;		/* SFN of the last good decode, -1 if none */
	double	sfn_pos;	/* stream position (samples at 1.92 MS/s) where
				 * that frame starts */
	double	mib_agree;	/* re-encode agreement of the last good decode (0..1) */
	/* frequency and time; positive offset = the carrier is received above
	 * the expected centre = the SDR clock is slow */
	double	cfo_hz;		/* carrier frequency offset */
	double	cfo_std_hz;	/* standard error of cfo_hz */
	int	cfo_points;
	double	cfo_alias_margin; /* the frame-to-frame estimate is ambiguous by
				 * 100 Hz, resolved by the slot-to-slot one: its
				 * distance from the half-way point, in standard
				 * errors (below ~3: the 100 Hz step is uncertain) */
	double	cfo_ppm;	/* cfo_hz / carrier, if the carrier is known */
	double	timing_ppm;	/* from the frame timing drift (sample clock) */
	double	timing_std_ppm;	/* its standard error */
	int	timing_points;
	/* quality from the CRS of port 0, centre 6 RB, subframe 0 */
	double	rsrp_dbfs;	/* power per CRS resource element, dB full scale */
	double	rssi_dbfs;	/* power of the 6 RB in CRS symbols */
	double	rsrq_db;	/* 6 * RSRP / RSSI */
	double	sinr_db;	/* CRS signal to noise and interference */
} lte_cell_info_t;

struct lte_track;

typedef struct lte_cell {
	int		debug;		/* print search and offset stages to stderr */
	/* search parameters */
	double		carrier_hz;	/* carrier frequency (0: unknown) */
	double		cfo_center, cfo_max;
	/* sample buffer at 1.92 MS/s */
	float complex	*buf;
	int		len, cap;
	int64_t		buf_pos;	/* stream position of buf[0] */
	/* search */
	int		searches;	/* searches done */
	int64_t		search_at;	/* stream position of the next search */
	float		*metric;	/* PSS search metric [3][hyps][9600] */
	double		*swork;		/* search FFT work arrays */
	int		hyps;
	/* cells */
	struct lte_track *track[LTE_CELL_MAX];
	int		ntrack;
	int		new_mib;	/* good MIB decodes in this process call */
} lte_cell_t;

/* rate must be 1.92e6 (LTE_RATE): the channel comes from the SDR layer at
 * that DSP rate (sdr_engine group or libchannelizer, bandwidth >= 1.08 MHz;
 * their pass band of 0.4 x DSP rate keeps the centre 6 RB).
 * carrier_hz: the EARFCN's frequency (0 if unknown): gives cfo_ppm and lets
 * the timing drift help to resolve the carrier offset's 200 Hz ambiguity.
 * The carrier offset is searched in cfo_center +- cfo_max Hz; cfo_max should
 * stay below 50 kHz (the channel raster is 100 kHz). */
int lte_cell_init(lte_cell_t *lc, double rate, double carrier_hz, double cfo_center, double cfo_max);
void lte_cell_exit(lte_cell_t *lc);

/* Feed interleaved IQ samples (full scale = 1.0). Returns the number of new
 * good MIB decodes. */
int lte_cell_process(lte_cell_t *lc, const float *iq, int num);

/* cells found so far (also unconfirmed ones) */
int lte_cell_count(const lte_cell_t *lc);
const lte_cell_info_t *lte_cell_get(const lte_cell_t *lc, int i);

/* confirmed: two or more MIB decodes whose SFN advance as the time says */
int lte_cell_confirmed(const lte_cell_info_t *ci);

#endif /* LTE_CELL_H */
