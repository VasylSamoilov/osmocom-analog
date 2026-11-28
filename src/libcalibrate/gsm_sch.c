/* GSM cell identity from one channel: SCH (BSIC, frame number) and BCCH
 *
 * Receive chain (all in this file, no external library):
 *   FCCH position: the carrier offset is removed, the FCCH tone (+1/4 symbol
 *     rate) is mixed to DC, a sliding window of 140 symbols measures how much
 *     of the power is that tone.
 *   Burst demodulation: GMSK in its linear (Laurent) approximation. After
 *     derotation by j^-k the received symbol is a 5 tap channel over +-1
 *     symbols; the taps are a least squares estimate from the training
 *     sequence, an MLSE (16 state Viterbi) decides the symbols, a flip cost
 *     gives soft values for the channel decoder.
 *   Channel decoding per 3GPP TS 45.003: rate 1/2 K=5 convolutional code
 *     (G0 = 1+D^3+D^4, G1 = 1+D+D^3+D^4), soft Viterbi; SCH (4.7): 25 bits
 *     with 10 inverted parity bits g(D) = D^10+D^8+D^6+D^5+D^4+D^2+1; BCCH
 *     (4.1): 184 bits, FIRE code g(D) = (D^23+1)(D^17+D^3+1) inverted, block
 *     diagonal interleaving over 4 normal bursts.
 *   SCH information per TS 44.018 9.1.30 / 45.002 3.3.2.2: BSIC, T1, T2, T3'.
 *   System Information 3 / 4 per TS 44.018 9.1.35 / 9.1.36, LAI per TS 24.008
 *     10.5.1.3.
 * The bit orders follow libosmocore (gsm0503_coding.c: SCH unpacking,
 * xCCH de-interleaving and burst unmapping; crcXXgen.c: parity register) and
 * osmocom-bb trxcon (sched_lchan_sch.c: SCH field layout), GPL-2.0-or-later,
 * written anew here for this tree.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "gsm_sch.h"
#include "fcch_detect.h"
#include "../liblogging/logging.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FRAME_SYMBOLS	1250		/* 8 timeslots of 156.25 symbols */
#define FN_MODULUS	2715648		/* 26 * 51 * 2048 */
#define FCCH_WIN_SYM	140		/* FCCH metric window (FCCH is 148) */
#define FCCH_MIN_METRIC	0.70		/* tone share needed in the window */
#define FCCH_MAX_PLATEAU 80		/* longer runs are not an FCCH (CW): the
					 * run above the threshold is ~55 symbols */
#define SCH_SEARCH_SYM	8		/* SCH timing search +- symbols */
#define NB_SEARCH_SYM	2		/* normal burst search after a SCH */
#define BUF_SEC		0.25		/* processing block */

/* interpolator: windowed sinc, 16 taps, 64 fractional phases */
#define INT_TAPS	16
#define INT_PHASES	64
static float int_table[INT_PHASES + 1][INT_TAPS];
static int int_table_ready;

/* SCH extended training sequence (TS 45.002 5.2.5) */
static const uint8_t sch_train[64] = {
	1,0,1,1,1,0,0,1, 0,1,1,0,0,0,1,0, 0,0,0,0,0,1,0,0, 0,0,0,0,1,1,1,1,
	0,0,1,0,1,1,0,1, 0,1,0,0,0,1,0,1, 0,1,1,1,0,1,1,0, 0,0,0,1,1,0,1,1,
};

/* normal burst training sequences, set 1 (TS 45.002 5.2.3) */
static const uint8_t nb_tsc[8][26] = {
	{0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1},
	{0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1},
	{0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0},
	{0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0},
	{0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1},
	{0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0},
	{1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1},
	{1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0},
};

static void int_table_init(void)
{
	if (int_table_ready)
		return;
	for (int p = 0; p <= INT_PHASES; p++) {
		double frac = (double)p / INT_PHASES;
		for (int j = 0; j < INT_TAPS; j++) {
			/* tap j is input sample floor(t) - 7 + j */
			double x = (double)(j - 7) - frac;
			double s = (fabs(x) < 1e-9) ? 1.0 : sin(M_PI * x) / (M_PI * x);
			double w = 0.42 + 0.5 * cos(M_PI * x / 8.0) + 0.08 * cos(2.0 * M_PI * x / 8.0);
			int_table[p][j] = (float)(s * w);
		}
	}
	int_table_ready = 1;
}

int gsm_sch_init(gsm_sch_t *sch, double rate, int decode_bcch)
{
	memset(sch, 0, sizeof(*sch));
	if (rate < GSM_RATE * 0.999)
		return -EINVAL;
	int_table_init();
	sch->rate = rate;
	sch->sps = rate / GSM_RATE;
	sch->decode_bcch = decode_bcch;
	sch->cap = (int)ceil(BUF_SEC * rate);
	/* after an FCCH: SCH one frame later, four BCCH frames after that */
	sch->keep = (int)ceil((5.0 * FRAME_SYMBOLS + 160.0 + FCCH_WIN_SYM + 2 * SCH_SEARCH_SYM + 40.0) * sch->sps);
	if (sch->cap < 3 * sch->keep)
		sch->cap = 3 * sch->keep;
	sch->buf = malloc(sch->cap * 2 * sizeof(float));
	sch->zs_re = malloc((sch->cap + 1) * sizeof(double));
	sch->zs_im = malloc((sch->cap + 1) * sizeof(double));
	sch->ps = malloc((sch->cap + 1) * sizeof(double));
	if (!sch->buf || !sch->zs_re || !sch->zs_im || !sch->ps) {
		gsm_sch_exit(sch);
		return -ENOMEM;
	}
	sch->last_sch_pos = -1000000000;
	sch->bcch_done_fn = -1;
	sch->ident.bsic = -1;
	return 0;
}

void gsm_sch_exit(gsm_sch_t *sch)
{
	free(sch->buf);
	free(sch->zs_re);
	free(sch->zs_im);
	free(sch->ps);
	sch->buf = NULL;
	sch->zs_re = sch->zs_im = sch->ps = NULL;
}

int gsm_sch_confirmed(const gsm_sch_t *sch)
{
	return sch->ident.sch_consistent >= 2;
}

/* ---- sample access ---- */

/* sample at fractional buffer position t, carrier offset removed */
static int sample_at(const gsm_sch_t *sch, double t, double w, double *re, double *im)
{
	int i0 = (int)floor(t);
	double frac = t - i0;
	int p = (int)lround(frac * INT_PHASES);
	int first = i0 - 7;

	if (first < 0 || first + INT_TAPS > sch->len)
		return -1;
	const float *k = int_table[p];
	const float *x = sch->buf + first * 2;
	float sr = 0.0f, si = 0.0f;
	for (int j = 0; j < INT_TAPS; j++) {
		sr += k[j] * x[j * 2];
		si += k[j] * x[j * 2 + 1];
	}
	/* remove carrier offset: w = -2 pi f / rate (phase relative to buf[0]) */
	double c = cos(w * t), s = sin(w * t);
	*re = sr * c - si * s;
	*im = sr * s + si * c;
	return 0;
}

/* ---- burst demodulation ---- */

/* rotate by j^-k */
static void derotate(int k, double re, double im, double *ore, double *oim)
{
	switch (k & 3) {
	case 0: *ore = re;  *oim = im;  break;
	case 1: *ore = im;  *oim = -re; break;
	case 2: *ore = -re; *oim = -im; break;
	default: *ore = -im; *oim = re; break;
	}
}

/* solve 5x5 real system a*x = b for complex b (b_re, b_im), in place */
static int solve5(double a[5][5], double b_re[5], double b_im[5])
{
	for (int c = 0; c < 5; c++) {
		int piv = c;
		for (int r = c + 1; r < 5; r++)
			if (fabs(a[r][c]) > fabs(a[piv][c]))
				piv = r;
		if (fabs(a[piv][c]) < 1e-9)
			return -1;
		if (piv != c) {
			for (int j = 0; j < 5; j++) {
				double t = a[c][j]; a[c][j] = a[piv][j]; a[piv][j] = t;
			}
			double t = b_re[c]; b_re[c] = b_re[piv]; b_re[piv] = t;
			t = b_im[c]; b_im[c] = b_im[piv]; b_im[piv] = t;
		}
		for (int r = 0; r < 5; r++) {
			if (r == c)
				continue;
			double f = a[r][c] / a[c][c];
			for (int j = c; j < 5; j++)
				a[r][j] -= f * a[c][j];
			b_re[r] -= f * b_re[c];
			b_im[r] -= f * b_im[c];
		}
	}
	for (int c = 0; c < 5; c++) {
		b_re[c] /= a[c][c];
		b_im[c] /= a[c][c];
	}
	return 0;
}

#define BURST_OBS	152	/* y_k for k = -2 .. 149 */

/* Demodulate a 148 symbol burst near buffer position 'guess' (position of
 * symbol 0), searching +-search symbols with the training sequence 'train'
 * (bits, at symbols tstart .. tstart+tlen-1). soft[148]: > 0 means bit 0.
 * Returns 0 and the refined position / SNR, or -1. */
static int demod_burst(const gsm_sch_t *sch, double guess, int search, double w,
		       const uint8_t *train, int tstart, int tlen,
		       float *soft, double *pos_out, double *snr_out)
{
	double sps = sch->sps;
	double best_pos = guess, best_c = -1.0;
	int steps = search * 4;

	/* timing: single tap correlation with the training symbols */
	for (int st = -steps; st <= steps; st++) {
		double tau = guess + st * sps / 4.0;
		double cr = 0.0, ci = 0.0;
		int ok = 1;
		for (int k = 0; k < tlen && ok; k++) {
			double re, im, dr, di;
			if (sample_at(sch, tau + (tstart + k) * sps, w, &re, &im) < 0) {
				ok = 0;
				break;
			}
			derotate(tstart + k, re, im, &dr, &di);
			double s = train[k] ? -1.0 : 1.0;
			cr += s * dr;
			ci += s * di;
		}
		if (!ok)
			continue;
		double c = cr * cr + ci * ci;
		if (c > best_c) {
			best_c = c;
			best_pos = tau;
		}
	}
	if (best_c < 0.0)
		return -1;

	/* observations y_k, k = -2 .. 149, at yr[k + 2] */
	double yr[BURST_OBS], yi[BURST_OBS];
	for (int k = -2; k < BURST_OBS - 2; k++) {
		double re, im;
		if (sample_at(sch, best_pos + k * sps, w, &re, &im) < 0)
			return -1;
		derotate(k, re, im, &yr[k + 2], &yi[k + 2]);
	}

	/* least squares channel: y_k = sum_i g_i s_(k+2-i), rows where all five
	 * symbols are training symbols */
	double a[5][5] = {{0}}, br[5] = {0}, bi[5] = {0};
	for (int k = tstart + 2; k <= tstart + tlen - 3; k++) {
		double s[5];
		for (int i = 0; i < 5; i++)
			s[i] = train[k + 2 - i - tstart] ? -1.0 : 1.0;
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 5; j++)
				a[i][j] += s[i] * s[j];
			br[i] += s[i] * yr[k + 2];
			bi[i] += s[i] * yi[k + 2];
		}
	}
	if (solve5(a, br, bi) < 0)
		return -1;
	double *gr = br, *gi = bi;

	/* noise from the training residual */
	double noise = 0.0, gpow = 0.0;
	int nrows = 0;
	for (int k = tstart + 2; k <= tstart + tlen - 3; k++) {
		double er = yr[k + 2], ei = yi[k + 2];
		for (int i = 0; i < 5; i++) {
			double s = train[k + 2 - i - tstart] ? -1.0 : 1.0;
			er -= gr[i] * s;
			ei -= gi[i] * s;
		}
		noise += er * er + ei * ei;
		nrows++;
	}
	noise /= nrows;
	for (int i = 0; i < 5; i++)
		gpow += gr[i] * gr[i] + gi[i] * gi[i];
	if (noise < gpow * 1e-6)
		noise = gpow * 1e-6;
	if (gpow <= 0.0)
		return -1;

	/* expected outputs for the 32 symbol combinations: bit i of c is the
	 * bit of s_(m-i) */
	double er_tab[32], ei_tab[32];
	for (int c = 0; c < 32; c++) {
		double r = 0.0, im = 0.0;
		for (int i = 0; i < 5; i++) {
			double s = ((c >> i) & 1) ? -1.0 : 1.0;
			r += gr[i] * s;
			im += gi[i] * s;
		}
		er_tab[c] = r;
		ei_tab[c] = im;
	}

	/* MLSE: state = bits of s_(m-1) .. s_(m-4) (bit 0 = s_(m-1)) */
	double metric[16], nmetric[16];
	static uint8_t from[BURST_OBS][16];	/* single threaded use */
	for (int s = 0; s < 16; s++)
		metric[s] = 0.0;
	for (int m = 0; m < BURST_OBS; m++) {
		for (int ns = 0; ns < 16; ns++) {
			int b = ns & 1;
			double best = 1e300;
			int bp = 0;
			for (int hi = 0; hi < 2; hi++) {
				int ps = (ns >> 1) | (hi << 3);
				int c = b | (ps << 1);
				double dr = yr[m] - er_tab[c], di = yi[m] - ei_tab[c];
				double mt = metric[ps] + dr * dr + di * di;
				if (mt < best) {
					best = mt;
					bp = ps;
				}
			}
			nmetric[ns] = best;
			from[m][ns] = (uint8_t)bp;
		}
		memcpy(metric, nmetric, sizeof(metric));
	}
	int st = 0;
	for (int s = 1; s < 16; s++)
		if (metric[s] < metric[st])
			st = s;
	double shat[BURST_OBS + 4];
	for (int m = BURST_OBS - 1; m >= 0; m--) {
		shat[m + 4] = (st & 1) ? -1.0 : 1.0;	/* s_m */
		st = from[m][st];
	}
	/* s_-1 .. s_-4 from the start state */
	for (int i = 0; i < 4; i++)
		shat[3 - i] = ((st >> i) & 1) ? -1.0 : 1.0;
#define SHAT(k) shat[(k) + 4]

	/* residuals and flip costs as soft values */
	double rr[BURST_OBS], ri[BURST_OBS];
	for (int k = -2; k < BURST_OBS - 2; k++) {
		double r = yr[k + 2], im = yi[k + 2];
		for (int i = 0; i < 5; i++) {
			int idx = k + 2 - i;
			double s = (idx >= -4 && idx < BURST_OBS) ? SHAT(idx) : 1.0;
			r -= gr[i] * s;
			im -= gi[i] * s;
		}
		rr[k + 2] = r;
		ri[k + 2] = im;
	}
	for (int k = 0; k < 148; k++) {
		double d = 0.0;
		for (int kk = k - 2; kk <= k + 2; kk++) {
			int i = kk + 2 - k;
			double fr = rr[kk + 2] + 2.0 * gr[i] * SHAT(k);
			double fi = ri[kk + 2] + 2.0 * gi[i] * SHAT(k);
			d += fr * fr + fi * fi - rr[kk + 2] * rr[kk + 2] - ri[kk + 2] * ri[kk + 2];
		}
		if (d < 0.0)
			d = 0.0;
		soft[k] = (float)(SHAT(k) * d / noise);
	}
#undef SHAT

	*pos_out = best_pos;
	*snr_out = 10.0 * log10(gpow / noise);
	return 0;
}

/* ---- channel decoding ---- */

/* soft Viterbi for the K=5 rate 1/2 code, n input bits (tail included),
 * terminated in state 0. in[2n] soft (> 0 means 0), out[n] bits. */
static void conv_decode(const float *in, int n, uint8_t *out)
{
	float metric[16], nmetric[16];
	uint8_t *from = malloc((size_t)n * 16);
	if (!from) {
		memset(out, 0, n);
		return;
	}
	for (int s = 0; s < 16; s++)
		metric[s] = (s == 0) ? 0.0f : -1e30f;
	for (int k = 0; k < n; k++) {
		float s0 = in[2 * k], s1 = in[2 * k + 1];
		for (int ns = 0; ns < 16; ns++) {
			int u = ns & 1;
			float best = -1e38f;
			int bp = 0;
			for (int hi = 0; hi < 2; hi++) {
				int ps = (ns >> 1) | (hi << 3);
				/* ps bit0 = u(k-1), bit1 = u(k-2), bit2 = u(k-3), bit3 = u(k-4) */
				int c0 = u ^ ((ps >> 2) & 1) ^ ((ps >> 3) & 1);
				int c1 = u ^ (ps & 1) ^ ((ps >> 2) & 1) ^ ((ps >> 3) & 1);
				float m = metric[ps] + (c0 ? -s0 : s0) + (c1 ? -s1 : s1);
				if (m > best) {
					best = m;
					bp = ps;
				}
			}
			nmetric[ns] = best;
			from[k * 16 + ns] = (uint8_t)bp;
		}
		memcpy(metric, nmetric, sizeof(metric));
	}
	int st = 0;
	for (int k = n - 1; k >= 0; k--) {
		out[k] = st & 1;
		st = from[k * 16 + st];
	}
	free(from);
}

/* parity check: register as libosmocore crcXXgen, remainder inverted */
static int crc_ok(const uint8_t *bits, int n, uint64_t poly, int nbits, const uint8_t *parity)
{
	uint64_t crc = 0, top = (uint64_t)1 << (nbits - 1);
	uint64_t mask = (nbits == 64) ? ~(uint64_t)0 : (((uint64_t)1 << nbits) - 1);

	for (int i = 0; i < n; i++) {
		crc ^= (uint64_t)(bits[i] & 1) << (nbits - 1);
		if (crc & top)
			crc = (crc << 1) ^ poly;
		else
			crc <<= 1;
		crc &= mask;
	}
	crc ^= mask;
	for (int i = 0; i < nbits; i++)
		if (((crc >> (nbits - 1 - i)) & 1) != parity[i])
			return 0;
	return 1;
}

/* ---- System Information ---- */

static void parse_lai(gsm_cell_ident_t *id, const uint8_t *l)
{
	int mcc1 = l[0] & 0xf, mcc2 = l[0] >> 4, mcc3 = l[1] & 0xf;
	int mnc3 = l[1] >> 4, mnc1 = l[2] & 0xf, mnc2 = l[2] >> 4;

	if (mcc1 > 9 || mcc2 > 9 || mcc3 > 9 || mnc1 > 9 || mnc2 > 9 || (mnc3 > 9 && mnc3 != 0xf))
		return;
	id->mcc = mcc1 * 100 + mcc2 * 10 + mcc3;
	if (mnc3 == 0xf) {
		id->mnc = mnc1 * 10 + mnc2;
		id->mnc_3digit = 0;
	} else {
		id->mnc = mnc1 * 100 + mnc2 * 10 + mnc3;
		id->mnc_3digit = 1;
	}
	id->lac = (l[3] << 8) | l[4];
	id->have_lai = 1;
}

static void parse_si(gsm_cell_ident_t *id, const uint8_t *d)
{
	/* d[0] L2 pseudo length, d[1] RR protocol discriminator, d[2] type */
	if ((d[1] & 0x0f) != 0x06) {
		id->si_count[GSM_SI_OTHER]++;
		return;
	}
	switch (d[2]) {
	case 0x19:
		id->si_count[GSM_SI_1]++;
		break;
	case 0x1a:
		id->si_count[GSM_SI_2]++;
		break;
	case 0x1b:	/* SI3: cell identity, LAI */
		id->si_count[GSM_SI_3]++;
		id->ci = (d[3] << 8) | d[4];
		id->have_ci = 1;
		parse_lai(id, d + 5);
		break;
	case 0x1c:	/* SI4: LAI */
		id->si_count[GSM_SI_4]++;
		parse_lai(id, d + 3);
		break;
	default:
		id->si_count[GSM_SI_OTHER]++;
		break;
	}
}

/* ---- SCH and BCCH ---- */

static void decode_bcch(gsm_sch_t *sch, double sch_pos, uint32_t fn_sch, double w)
{
	gsm_cell_ident_t *id = &sch->ident;
	int bcc = gsm_bsic_bcc(id->bsic);
	float soft[148], ib[4 * 114], cb[456];
	uint8_t u[228], l2[23];

	for (int b = 0; b < 4; b++) {
		double pos, snr;
		double guess = sch_pos + (b + 1) * FRAME_SYMBOLS * sch->sps;
		if (demod_burst(sch, guess, NB_SEARCH_SYM, w, nb_tsc[bcc], 61, 26,
				soft, &pos, &snr) < 0) {
			id->bcch_bad++;
			return;
		}
		memcpy(ib + b * 114, soft + 3, 57 * sizeof(float));
		memcpy(ib + b * 114 + 57, soft + 88, 57 * sizeof(float));
	}
	for (int k = 0; k < 456; k++)
		cb[k] = ib[(k & 3) * 114 + 2 * ((49 * k) % 57) + ((k & 7) >> 2)];
	conv_decode(cb, 228, u);
	if (!crc_ok(u, 184, 0x0004820009ULL, 40, u + 184)) {
		id->bcch_bad++;
		LOGP(DSDR, LOGL_DEBUG, "  BCCH FN %u: FIRE code failed\n", fn_sch + 1);
		return;
	}
	for (int i = 0; i < 23; i++) {
		l2[i] = 0;
		for (int j = 0; j < 8; j++)
			l2[i] |= u[i * 8 + j] << j;
	}
	id->bcch_good++;
	parse_si(id, l2);
	LOGP(DSDR, LOGL_DEBUG, "  BCCH FN %u TC %u: message type 0x%02x\n",
	     fn_sch + 1, ((fn_sch + 1) / 51) % 8, l2[2]);
}

static int try_sch(gsm_sch_t *sch, double guess, double w)
{
	gsm_cell_ident_t *id = &sch->ident;
	float soft[148], conv[78];
	uint8_t u[39];
	double pos, snr;

	if (demod_burst(sch, guess, SCH_SEARCH_SYM, w, sch_train, 42, 64, soft, &pos, &snr) < 0)
		return 0;
	memcpy(conv, soft + 3, 39 * sizeof(float));
	memcpy(conv + 39, soft + 106, 39 * sizeof(float));
	conv_decode(conv, 39, u);
	if (!crc_ok(u, 25, 0x175, 10, u + 25)) {
		id->sch_bad++;
		LOGP(DSDR, LOGL_DEBUG, "  SCH parity failed (SNR %.1f dB)\n", snr);
		return 0;
	}

	uint32_t sb = 0;
	for (int i = 0; i < 25; i++)
		sb |= (uint32_t)u[i] << i;
	int bsic = (sb >> 2) & 0x3f;
	int t1 = ((sb >> 23) & 0x01) | ((sb >> 7) & 0x1fe) | ((sb << 9) & 0x600);
	int t2 = (sb >> 18) & 0x1f;
	int t3p = ((sb >> 24) & 0x01) | ((sb >> 15) & 0x06);
	int t3 = t3p * 10 + 1;
	if (t2 > 25 || t3p > 4) {
		id->sch_bad++;
		return 0;
	}
	uint32_t fn = 51 * ((t3 - t2 + 26) % 26) + t3 + 51 * 26 * t1;
	int64_t abs_pos = sch->buf_pos + (int64_t)llround(pos);

	id->sch_good++;
	id->snr_db = snr;
	id->fn = fn;

	/* consistency with the previous good SCH: same BSIC, frame number
	 * advanced by the frames elapsed */
	int consistent = 0;
	if (id->sch_consistent > 0 && bsic == id->bsic) {
		double frames = (double)(abs_pos - sch->first_pos) / (FRAME_SYMBOLS * sch->sps);
		long expect = lround(frames);
		long got = ((long)fn - (long)sch->first_fn + FN_MODULUS) % FN_MODULUS;
		if (fabs(frames - expect) < 0.2 && got == expect % FN_MODULUS)
			consistent = 1;
	}
	if (consistent) {
		id->sch_consistent++;
	} else if (id->sch_consistent <= 1) {
		/* (re)start the reference: a single decode may be a false parity */
		id->bsic = bsic;
		id->sch_consistent = 1;
	}
	if (consistent || id->sch_consistent == 1) {
		sch->first_pos = abs_pos;
		sch->first_fn = fn;
	}
	/* frame timing: the burst position (fractional) against the frame
	 * number, from the first decode of the consistent run */
	if (id->sch_consistent == 1 && !consistent)
		sch->t_n = 0;
	if (consistent || id->sch_consistent == 1) {
		double posf = (double)sch->buf_pos + pos;

		if (!sch->t_n) {
			sch->t_fn = fn;
			sch->t_pos = posf;
			sch->t_sk = sch->t_sy = sch->t_skk = sch->t_sky = sch->t_syy = 0.0;
		}
		double k = (double)(((long)fn - (long)sch->t_fn + FN_MODULUS) % FN_MODULUS);
		/* the deviation from the nominal samples per frame */
		double y = posf - sch->t_pos - k * FRAME_SYMBOLS * sch->sps;

		sch->t_n++;
		sch->t_sk += k;
		sch->t_sy += y;
		sch->t_skk += k * k;
		sch->t_sky += k * y;
		sch->t_syy += y * y;
		if (sch->t_n >= 4) {
			double n = sch->t_n, sxx = sch->t_skk - sch->t_sk * sch->t_sk / n;
			double sxy = sch->t_sky - sch->t_sk * sch->t_sy / n, syy = sch->t_syy - sch->t_sy * sch->t_sy / n;

			if (sxx > 0.0) {
				double b = sxy / sxx, ssr = syy - b * sxy, per = FRAME_SYMBOLS * sch->sps;

				/* slow clock: fewer samples per frame */
				id->timing_ppm = -b / per * 1e6;
				id->timing_std_ppm = (ssr > 0.0 && n > 2) ? sqrt(ssr / (n - 2) / sxx) / per * 1e6 : 0.0;
				id->timing_points = sch->t_n;
			}
		}
	}
	LOGP(DSDR, LOGL_DEBUG, "  SCH: BSIC %d (NCC %d BCC %d) FN %u (T1 %d T2 %d T3 %d) "
	     "SNR %.1f dB%s\n", bsic, bsic >> 3, bsic & 7, fn, t1, t2, t3, snr,
	     consistent ? ", consistent" : "");

	/* BCCH Norm: frames 2..5 of the 51-multiframe, right after the SCH of
	 * frame 1 */
	if (sch->decode_bcch && t3 == 1 && bsic == id->bsic && (int64_t)fn != sch->bcch_done_fn) {
		sch->bcch_done_fn = fn;
		decode_bcch(sch, pos, fn, w);
	}
	return 1;
}

static int process_block(gsm_sch_t *sch, int limit, double freq_offset_hz)
{
	int n = sch->len;
	int win = (int)lround(FCCH_WIN_SYM * sch->sps);
	double w_tone = -2.0 * M_PI * (freq_offset_hz + FCCH_FREQ) / sch->rate;
	double w_car = -2.0 * M_PI * freq_offset_hz / sch->rate;
	int found = 0;

	if (n < win + 1)
		return 0;

	/* prefix sums of the tone-mixed signal and of the power */
	double pr = 1.0, pi = 0.0, cr = cos(w_tone), ci = sin(w_tone);
	double zr = 0.0, zi = 0.0, pw = 0.0;
	sch->zs_re[0] = sch->zs_im[0] = sch->ps[0] = 0.0;
	for (int i = 0; i < n; i++) {
		double xr = sch->buf[i * 2], xi = sch->buf[i * 2 + 1];
		zr += xr * pr - xi * pi;
		zi += xr * pi + xi * pr;
		pw += xr * xr + xi * xi;
		sch->zs_re[i + 1] = zr;
		sch->zs_im[i + 1] = zi;
		sch->ps[i + 1] = pw;
		double t = pr * cr - pi * ci;
		pi = pr * ci + pi * cr;
		pr = t;
		if ((i & 1023) == 1023) {
			double g = 1.0 / sqrt(pr * pr + pi * pi);
			pr *= g;
			pi *= g;
		}
	}

	/* runs of high tone share; the plateau centre is 4 symbols into the
	 * FCCH (window 140 in a burst of 148) */
	int i = 0, last = n - win;
	while (i <= last) {
		double p = sch->ps[i + win] - sch->ps[i];
		double m = 0.0;
		if (p > 0.0) {
			double sr = sch->zs_re[i + win] - sch->zs_re[i];
			double si = sch->zs_im[i + win] - sch->zs_im[i];
			m = (sr * sr + si * si) / (win * p);
		}
		if (m < FCCH_MIN_METRIC) {
			i++;
			continue;
		}
		/* run: find its extent and maximum */
		int a = i;
		while (i <= last) {
			p = sch->ps[i + win] - sch->ps[i];
			m = 0.0;
			if (p > 0.0) {
				double sr = sch->zs_re[i + win] - sch->zs_re[i];
				double si = sch->zs_im[i + win] - sch->zs_im[i];
				m = (sr * sr + si * si) / (win * p);
			}
			if (m < FCCH_MIN_METRIC)
				break;
			i++;
		}
		int b = i - 1;
		if ((b - a) > FCCH_MAX_PLATEAU * sch->sps)
			continue;	/* steady tone, not a burst */
		double fcch_start = 0.5 * (a + b) - 4.0 * sch->sps;
		if (fcch_start >= limit)
			break;		/* next block (kept overlap) */
		int64_t abs_pos = sch->buf_pos + (int64_t)fcch_start;
		if (abs_pos < sch->last_sch_pos + (int64_t)(600 * sch->sps))
			continue;	/* same FCCH seen in the overlap */
		sch->last_sch_pos = abs_pos;
		found += try_sch(sch, fcch_start + FRAME_SYMBOLS * sch->sps, w_car);
	}
	return found;
}

int gsm_sch_process(gsm_sch_t *sch, const float *iq, int num, double freq_offset_hz)
{
	int found = 0;

	while (num > 0) {
		int space = sch->cap - sch->len;
		int n = (num < space) ? num : space;
		memcpy(sch->buf + sch->len * 2, iq, n * 2 * sizeof(float));
		sch->len += n;
		iq += n * 2;
		num -= n;
		if (sch->len < sch->cap)
			break;
		/* FCCH may start up to len - keep; the rest is kept for the
		 * next block */
		found += process_block(sch, sch->len - sch->keep, freq_offset_hz);
		memmove(sch->buf, sch->buf + (sch->len - sch->keep) * 2, sch->keep * 2 * sizeof(float));
		sch->buf_pos += sch->len - sch->keep;
		sch->len = sch->keep;
	}
	return found;
}
