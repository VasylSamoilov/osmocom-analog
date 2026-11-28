/* GSM cell identity from one channel: SCH (BSIC, frame number) and BCCH
 *
 * Proves that a carrier measured by its FCCH is a real base station: the SCH
 * one frame after each FCCH is demodulated and decoded (parity checked), the
 * frame numbers of several SCH must advance as their distance in time says.
 * BCCH blocks (FIRE code checked) give the System Information messages; SI3
 * carries MCC, MNC, LAC and the cell identity.
 *
 * Input is one GSM channel's complex baseband (centred on the carrier, at any
 * rate above the symbol rate) and the carrier frequency offset measured from
 * the FCCH. How the samples arrive does not matter.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef GSM_SCH_H
#define GSM_SCH_H

#include <stdint.h>

/* the decoded identity of a cell (what is known so far) */
typedef struct gsm_cell_ident {
	int		sch_good;	/* SCH decodes with valid parity */
	int		sch_bad;	/* SCH candidates that failed parity */
	int		sch_consistent;	/* good SCH whose BSIC and frame number agree with the first */
	int		bsic;		/* -1 if unknown */
	uint32_t	fn;		/* frame number of the last good SCH */
	double		snr_db;		/* SNR of the last good SCH (training residual) */
	int		bcch_good;	/* BCCH blocks with valid FIRE code */
	int		bcch_bad;	/* BCCH blocks that failed */
	int		si_count[8];	/* good SI1, SI2, SI3, SI4 ... by index below */
	int		have_lai;	/* MCC/MNC/LAC known (SI3 or SI4) */
	int		mcc, mnc, mnc_3digit, lac;
	int		have_ci;	/* cell identity known (SI3) */
	int		ci;
	/* the sample clock from the frame timing (consistent SCH: position
	 * against frame number, a regression): the crystal alone, no tuning
	 * step in it. positive = the SDR clock is slow (as the carrier ppm) */
	double		timing_ppm, timing_std_ppm;
	int		timing_points;
} gsm_cell_ident_t;

/* si_count[] index */
#define GSM_SI_1	0
#define GSM_SI_2	1
#define GSM_SI_3	2
#define GSM_SI_4	3
#define GSM_SI_OTHER	4

typedef struct gsm_sch {
	double		rate;		/* input sample rate */
	double		sps;		/* samples per symbol */
	int		decode_bcch;	/* also decode BCCH (SI) blocks */
	float		*buf;		/* interleaved IQ, as received */
	int		len, cap, keep;	/* samples in buf, capacity, kept overlap */
	int64_t		buf_pos;	/* stream position of buf[0] */
	double		*zs_re, *zs_im, *ps; /* FCCH metric prefix sums */
	int64_t		last_sch_pos;	/* stream position of the last SCH tried */
	int64_t		first_pos;	/* first good SCH: stream position, */
	uint32_t	first_fn;	/* frame number */
	int64_t		bcch_done_fn;	/* last BCCH block FN decoded */
	/* frame timing regression: k frames since (t_fn, t_pos), y samples */
	int		t_n;
	uint32_t	t_fn;
	double		t_pos;
	double		t_sk, t_sy, t_skk, t_sky, t_syy;
	gsm_cell_ident_t ident;
} gsm_sch_t;

int gsm_sch_init(gsm_sch_t *sch, double rate, int decode_bcch);
void gsm_sch_exit(gsm_sch_t *sch);

/* Feed samples (interleaved IQ). freq_offset_hz is the carrier offset as
 * measured from the FCCH (FCCH tone at +67.7 kHz + offset). Returns the
 * number of new good SCH decodes in this call. */
int gsm_sch_process(gsm_sch_t *sch, const float *iq, int num, double freq_offset_hz);

/* The carrier is a base station: two or more SCH decodes with the same BSIC
 * whose frame numbers advance with time. */
int gsm_sch_confirmed(const gsm_sch_t *sch);

/* "NCC-BCC" style helpers */
static inline int gsm_bsic_ncc(int bsic) { return (bsic >> 3) & 7; }
static inline int gsm_bsic_bcc(int bsic) { return bsic & 7; }

#endif /* GSM_SCH_H */
