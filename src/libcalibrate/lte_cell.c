/* LTE cell search, identity proof (MIB) and frequency offset from one carrier
 *
 * See lte_cell.h for the chain. Frame layout (36.211 4, 6.12), at 1.92 MS/s
 * (128 point FFT): a slot is 960 samples, normal CP symbols are 10 + 128
 * and 9 + 128 samples, extended CP 32 + 128. The data part of symbol l of
 * slot s starts at 960 s + 10 + 137 l (normal) or 960 s + 32 + 160 l
 * (extended) after the frame start.
 *   FDD: PSS in the last symbol of slots 0 and 10 (at 832), SSS one symbol
 *        before (137 or 160 samples earlier).
 *   TDD: PSS in symbol 2 of subframes 1 and 6 (at 2204 / 2272), SSS in the
 *        last symbol of slots 1 and 11 (412 / 480 samples earlier).
 *   PBCH: slot 1, symbols 0..3, centre 72 subcarriers, CRS positions of
 *        four ports left out.
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

#include "lte_cell.h"
#include "lte_phy.h"
#include "../libfft/fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS		LTE_RATE
#define HF		9600		/* half-frame */
#define HF2		(HF / 2)	/* search timings (every second sample) */
#define FRAME		19200
#define SLOT		960
#define SEARCH_HF	4		/* half-frames combined in the search */
#define SEARCH_LOG	14
#define SEARCH_N	(1 << SEARCH_LOG)
#define SEARCH_PRE	600		/* kept before the search window (SSS before PSS) */
#define SEARCH_SPAN	((SEARCH_HF - 1) * HF + SEARCH_N)
#define SEARCH_MAX	2		/* searches without a cell before giving up */
#define CFO_STEP	7500.0		/* search grid of the carrier offset */
#define PSS_MIN_METRIC	0.05		/* noise: mean 1/128 per window */
#define CAND_PER_NID2	2
#define SSS_MIN_SCORE	6.0		/* best / rms of the other hypotheses */
#define BACKOFF		3		/* FFT window starts this far into the CP */
#define TRACK_WIN	3		/* PSS tracking search +- samples */
#define TRACK_MIN	0.03		/* PSS tracking metric needed */
#define REG_MIN		6		/* timing points before the regression is used */
#define REFINE_FRAMES	3		/* frames of slot-to-slot offset refinement */
#define SETTLE_FRAMES	8		/* then frames averaged before frame-to-frame */
#define SPUR_FRAMES	3		/* frames before CRS positions are judged */
#define SPUR_RATIO	2.0		/* CRS power above the median: a receiver spur */
#define MIB_MIN_AGREE	0.35		/* re-encode agreement of a good decode */
#define LOST_NO_MIB	50		/* frames without any good MIB */
#define LOST_PSS	20		/* half-frames without the PSS */
#define FRAME_NEED	12400		/* samples after the frame start needed */
#define BUF_CAP		(2 * (SEARCH_PRE + SEARCH_SPAN) + 4 * FRAME)

typedef struct lte_track {
	lte_cell_info_t	info;
	int		nid1, nid2;
	int		pss_off;	/* PSS data start after the frame start */
	int		nsymb;		/* symbols per slot */
	double		t0;		/* stream position of the next frame start */
	double		spf;		/* samples per frame */
	double		cfo;		/* offset used for derotation */
	int		refine;		/* refinement frames left */
	double		settle_s;
	int		settle_n;
	/* receiver spurs on the port 0 CRS (symbol 0 / lc1, m) */
	double		spow[2][12];
	int		spow_n;
	uint8_t		smask[2][12];
	float complex	pss[128];	/* PSS, time domain */
	double		pss_energy;
	float complex	crs[2][7][12];	/* CRS of slots 0 and 1 */
	uint8_t		scr[LTE_PBCH_BITS_N];
	/* timing regression of the PSS positions over the half-frame index */
	int64_t		hf;		/* half-frame index of the next frame */
	double		k0, y0, sk, sy, skk, sky, syy;
	int		reg_n;
	int		pss_miss;
	/* carrier offset estimates (Hz, residual to cfo) */
	double		intra_s, intra_s2;
	int		intra_n;
	float complex	prev_h[48];
	double		prev_t0;
	int		have_prev;
	double complex	zi, ui;		/* frame to frame: sum, sum of unit vectors */
	int		inter_n;
	float complex	crs5[2][7][12];	/* CRS of slots 10 and 11 */
	double complex	z5, u5;		/* subframe 0 to 5 */
	int		n5;
	double		cfo_fixed;	/* cfo when the residuals started */
	/* quality */
	double		q_crs, q_noise, q_rssi;
	int		q_n, q_nn, q_rn;
	/* MIB reference */
	int		have_ref;
	int		ref_sfn, ref_ports;
	double		ref_t0;
	lte_mib_t	ref_mib;
	int		since_good;
} lte_track_t;

/*
 * small helpers
 */

static inline int bin_idx(int bin)
{
	return (bin + LTE_NFFT) & (LTE_NFFT - 1);
}

/* PSS n (0..61) -> FFT bin */
static inline int pss_bin(int n)
{
	return (n < 31) ? n - 31 : n - 30;
}

static float complex *bufp(lte_cell_t *lc, int64_t pos)
{
	return lc->buf + (pos - lc->buf_pos);
}

/* phase rotation of the derotation at stream position pos */
static double derot_phase(double cfo, int64_t pos)
{
	double cyc = cfo * (double)pos / FS;
	return -2.0 * M_PI * (cyc - floor(cyc));
}

/* FFT of the symbol whose data part starts at stream position t: derotated
 * by cfo, compensated so that it is as if the window began exactly at t */
static void symbol_fft(lte_cell_t *lc, double t, double cfo, float complex *out)
{
	double re[LTE_NFFT], im[LTE_NFFT];
	int64_t w = (int64_t)floor(t + 0.5) - BACKOFF;
	double d = t - (double)w;
	float complex *x = bufp(lc, w);
	double complex rot = cexp(I * derot_phase(cfo, w));
	double complex step = cexp(-I * 2.0 * M_PI * cfo / FS);
	int n;

	for (n = 0; n < LTE_NFFT; n++) {
		double complex v = x[n] * rot;
		re[n] = creal(v);
		im[n] = cimag(v);
		rot *= step;
	}
	fft_process(1, 7, re, im);
	for (n = 0; n < LTE_NFFT; n++) {
		int bin = (n < LTE_NFFT / 2) ? n : n - LTE_NFFT;
		double a = 2.0 * M_PI * bin * d / LTE_NFFT;
		out[n] = (re[n] + I * im[n]) * (cos(a) + I * sin(a));
	}
}

/* data start of symbol l in slot s after the frame start */
static int sym_start(int ext_cp, int s, int l)
{
	return SLOT * s + (ext_cp ? 32 + 160 * l : 10 + 137 * l);
}

/*
 * search: PSS over 4 half-frames, then SSS
 */

/* PSS in the time domain (128 samples, no CP) */
static void pss_time(int nid2, float complex *p, double *energy)
{
	float complex d[62];
	double re[LTE_NFFT], im[LTE_NFFT], e = 0;
	int n;

	lte_pss_seq(nid2, d);
	memset(re, 0, sizeof(re));
	memset(im, 0, sizeof(im));
	for (n = 0; n < 62; n++) {
		int b = bin_idx(pss_bin(n));
		re[b] = crealf(d[n]);
		im[b] = cimagf(d[n]);
	}
	fft_process(-1, 7, re, im);
	for (n = 0; n < LTE_NFFT; n++) {
		p[n] = re[n] + I * im[n];
		e += re[n] * re[n] + im[n] * im[n];
	}
	*energy = e;
}

typedef struct cand {
	int	nid2, tau, hyp;
	double	metric;
} cand_t;

static void track_new(lte_cell_t *lc, int nid1, int nid2, int tdd, int ext_cp, double frame_start, double cfo,
		      double pss_metric, double sss_score)
{
	lte_track_t *tr;
	int pci = 3 * nid1 + nid2, i, s, l;

	for (i = 0; i < lc->ntrack; i++) {
		if (lc->track[i]->info.pci == pci)
			return;
	}
	if (lc->ntrack >= LTE_CELL_MAX)
		return;
	tr = calloc(1, sizeof(*tr));
	if (!tr)
		return;
	tr->info.pci = pci;
	tr->info.tdd = tdd;
	tr->info.ext_cp = ext_cp;
	tr->info.pss_metric = pss_metric;
	tr->info.sss_score = sss_score;
	tr->info.sfn = -1;
	tr->nid1 = nid1;
	tr->nid2 = nid2;
	tr->nsymb = ext_cp ? 6 : 7;
	tr->pss_off = tdd ? (ext_cp ? 2272 : 2204) : 832;
	tr->spf = FRAME;
	tr->cfo = cfo;
	tr->info.cfo_hz = cfo;
	tr->refine = REFINE_FRAMES + SETTLE_FRAMES;
	pss_time(nid2, tr->pss, &tr->pss_energy);
	for (s = 0; s < 2; s++) {
		for (l = 0; l < tr->nsymb; l++)
		{
			lte_crs_seq(pci, s, l, ext_cp, tr->crs[s][l]);
			lte_crs_seq(pci, 10 + s, l, ext_cp, tr->crs5[s][l]);
		}
	}
	lte_gold(pci, tr->scr, LTE_PBCH_BITS_N);
	/* first whole frame inside the buffer */
	while (frame_start < (double)lc->buf_pos + SEARCH_PRE)
		frame_start += FRAME;
	tr->t0 = frame_start;
	lc->track[lc->ntrack++] = tr;
}

/* SSS: N_ID1, subframe, duplex and CP of a PSS candidate */
static void search_sss(lte_cell_t *lc, int64_t s0, const cand_t *cd, double cfo, double pss_metric)
{
	static const int dist[4] = { 137, 160, 412, 480 };	/* FDD N, FDD E, TDD N, TDD E */
	float complex d_pss[62], Y[LTE_NFFT], H[62], z[4][SEARCH_HF][62];
	float (*dsss)[2][62];
	double best = 0, best_score = 0, cfo1;
	double complex acc = 0;
	float complex p[128];
	double pe;
	int h, q, n, nid1, sf, best_q = -1, best_nid1 = -1, best_sf = 0;

	pss_time(cd->nid2, p, &pe);
	/* fine offset from the phase between the two PSS halves (+-15 kHz) */
	for (h = 0; h < SEARCH_HF; h++) {
		int64_t pos = s0 + (int64_t)h * HF + cd->tau;
		float complex *x = bufp(lc, pos);
		double complex c1 = 0, c2 = 0, step = cexp(-I * 2.0 * M_PI * cfo / FS), rot = 1;
		for (n = 0; n < LTE_NFFT; n++) {
			double complex v = x[n] * rot * conjf(p[n]);
			if (n < LTE_NFFT / 2)
				c1 += v;
			else
				c2 += v;
			rot *= step;
		}
		acc += c2 * conj(c1);
	}
	cfo1 = cfo + carg(acc) * FS / (2.0 * M_PI * (LTE_NFFT / 2));

	lte_pss_seq(cd->nid2, d_pss);
	for (h = 0; h < SEARCH_HF; h++) {
		double tp = (double)(s0 + (int64_t)h * HF + cd->tau);
		symbol_fft(lc, tp, cfo1, Y);
		for (n = 0; n < 62; n++)
			H[n] = Y[bin_idx(pss_bin(n))] * conjf(d_pss[n]);
		for (q = 0; q < 4; q++) {
			symbol_fft(lc, tp - dist[q], cfo1, Y);
			for (n = 0; n < 62; n++)
				z[q][h][n] = Y[bin_idx(pss_bin(n))] * conjf(H[n]);
		}
	}
	dsss = malloc(168 * sizeof(*dsss));
	if (!dsss)
		return;
	for (nid1 = 0; nid1 < 168; nid1++) {
		lte_sss_seq(nid1, cd->nid2, 0, dsss[nid1][0]);
		lte_sss_seq(nid1, cd->nid2, 1, dsss[nid1][1]);
	}
	for (q = 0; q < 4; q++) {
		double sc[168][2], bq = -1e30, s2 = 0, rms, score;
		int bn = 0, bs = 0, cnt = 0;
		for (nid1 = 0; nid1 < 168; nid1++) {
			for (sf = 0; sf < 2; sf++) {
				double v = 0;
				for (h = 0; h < SEARCH_HF; h++) {
					const float *d = dsss[nid1][sf ^ (h & 1)];
					for (n = 0; n < 62; n++)
						v += crealf(z[q][h][n]) * d[n];
				}
				sc[nid1][sf] = v;
				if (v > bq) {
					bq = v;
					bn = nid1;
					bs = sf;
				}
			}
		}
		for (nid1 = 0; nid1 < 168; nid1++) {
			for (sf = 0; sf < 2; sf++) {
				if (nid1 == bn && sf == bs)
					continue;
				s2 += sc[nid1][sf] * sc[nid1][sf];
				cnt++;
			}
		}
		rms = sqrt(s2 / cnt);
		score = (rms > 0) ? bq / rms : 0;
		if (score > best_score) {
			best_score = score;
			best = bq;
			best_q = q;
			best_nid1 = bn;
			best_sf = bs;
		}
	}
	free(dsss);
	(void)best;
	if (lc->debug)
		fprintf(stderr, "cand nid2 %d tau %d cfo %.0f/%.0f metric %.3f: SSS q %d nid1 %d sf %d score %.1f\n",
			cd->nid2, cd->tau, cfo, cfo1, pss_metric, best_q, best_nid1, best_sf, best_score);
	if (best_q < 0 || best_score < SSS_MIN_SCORE)
		return;
	{
		int tdd = best_q >= 2, ext_cp = best_q & 1;
		int pss_off = tdd ? (ext_cp ? 2272 : 2204) : 832;
		double fs0 = (double)(s0 + cd->tau) - pss_off - (best_sf ? HF : 0);
		track_new(lc, best_nid1, cd->nid2, tdd, ext_cp, fs0, cfo1, pss_metric, best_score);
	}
}

/* the search sees every second sample: the best timing within +-2 samples,
 * PSS correlation summed over the search half-frames */
static int refine_tau(lte_cell_t *lc, int64_t s0, int nid2, int tau, double cfo)
{
	float complex p[128];
	double pe, best = -1;
	int o, h, n, bt = tau;

	pss_time(nid2, p, &pe);
	for (o = -2; o <= 2; o++) {
		double sum = 0;
		if (tau + o < 0 || tau + o >= HF)
			continue;
		for (h = 0; h < SEARCH_HF; h++) {
			int64_t pos = s0 + (int64_t)h * HF + tau + o;
			float complex *x = bufp(lc, pos);
			double complex acc = 0, rot = 1, step = cexp(-I * 2.0 * M_PI * cfo / FS);
			for (n = 0; n < LTE_NFFT; n++) {
				acc += x[n] * rot * conjf(p[n]);
				rot *= step;
			}
			sum += creal(acc * conj(acc));
		}
		if (sum > best) {
			best = sum;
			bt = tau + o;
		}
	}
	return bt;
}

static void search(lte_cell_t *lc)
{
	int64_t s0 = lc->search_at;
	double *xr, *xi, *wr, *wi, *pr[3], *pi[3], *energy, pe[3], *cr_tab, *ci_tab;
	float *best;
	short *besth;
	cand_t cand[3 * CAND_PER_NID2];
	int ncand = 0, h, nid2, hy, n, i;

	lc->searches++;
	lc->search_at += SEARCH_HF * HF;
	if (!lc->swork) {
		lc->swork = calloc(12 * SEARCH_N + HF, sizeof(double));
		if (!lc->swork)
			return;
	}
	xr = lc->swork;
	xi = xr + SEARCH_N;
	wr = xi + SEARCH_N;
	wi = wr + SEARCH_N;
	energy = wi + SEARCH_N;
	cr_tab = energy + HF + 6 * SEARCH_N;
	ci_tab = cr_tab + SEARCH_N;
	for (nid2 = 0; nid2 < 3; nid2++) {
		float complex p[128];
		pr[nid2] = energy + HF + (2 * nid2) * SEARCH_N;
		pi[nid2] = pr[nid2] + SEARCH_N;
		memset(pr[nid2], 0, SEARCH_N * sizeof(double));
		memset(pi[nid2], 0, SEARCH_N * sizeof(double));
		pss_time(nid2, p, &pe[nid2]);
		for (n = 0; n < LTE_NFFT; n++) {
			pr[nid2][n] = crealf(p[n]);
			pi[nid2][n] = cimagf(p[n]);
		}
		fft_process(1, SEARCH_LOG, pr[nid2], pi[nid2]);
	}
	for (n = 0; n < SEARCH_N; n++) {
		int k = (n < SEARCH_N / 2) ? n : n - SEARCH_N;
		cr_tab[n] = cos(2.0 * M_PI * k / SEARCH_N);
		ci_tab[n] = sin(2.0 * M_PI * k / SEARCH_N);
	}
	memset(lc->metric, 0, 3 * lc->hyps * HF * sizeof(float));
	for (h = 0; h < SEARCH_HF; h++) {
		float complex *x = bufp(lc, s0 + (int64_t)h * HF);
		double e = 0;
		for (n = 0; n < SEARCH_N; n++) {
			xr[n] = crealf(x[n]);
			xi[n] = cimagf(x[n]);
		}
		/* window energies */
		for (n = 0; n < LTE_NFFT; n++)
			e += xr[n] * xr[n] + xi[n] * xi[n];
		for (n = 0; n < HF; n++) {
			energy[n] = e;
			e += xr[n + LTE_NFFT] * xr[n + LTE_NFFT] + xi[n + LTE_NFFT] * xi[n + LTE_NFFT];
			e -= xr[n] * xr[n] + xi[n] * xi[n];
			if (e < 0)
				e = 0;
		}
		fft_process(1, SEARCH_LOG, xr, xi);
		for (nid2 = 0; nid2 < 3; nid2++) {
			for (hy = 0; hy < lc->hyps; hy++) {
				double f = lc->cfo_center + (hy - (lc->hyps - 1) / 2) * CFO_STEP;
				int b = (int)floor(f * SEARCH_N / FS + 0.5);
				float *m = lc->metric + (nid2 * lc->hyps + hy) * HF;
				int odd;
				/* X * conj(P), only the half of the bins around the
				 * (shifted) PSS: an inverse FFT of half size gives
				 * the correlation at every second sample; a second
				 * one, shifted by one sample (e^(j 2 pi k / N)),
				 * the odd samples */
				for (odd = 0; odd < 2; odd++) {
					for (n = 0; n < SEARCH_N / 2; n++) {
						int kx = (b - SEARCH_N / 4 + n) & (SEARCH_N - 1);
						int kp = (n - SEARCH_N / 4) & (SEARCH_N - 1);
						double ar = xr[kx] * pr[nid2][kp] + xi[kx] * pi[nid2][kp];
						double ai = xi[kx] * pr[nid2][kp] - xr[kx] * pi[nid2][kp];
						if (odd) {
							double c = cr_tab[kx], s = ci_tab[kx];
							wr[n] = ar * c - ai * s;
							wi[n] = ar * s + ai * c;
						} else {
							wr[n] = ar;
							wi[n] = ai;
						}
					}
					fft_process(-1, SEARCH_LOG - 1, wr, wi);
					for (n = 0; n < HF2; n++) {
						double cr = wr[n] * SEARCH_N, ci = wi[n] * SEARCH_N;
						int t = 2 * n + odd;
						if (energy[t] > 0)
							m[t] += (cr * cr + ci * ci) / (energy[t] * pe[nid2]) / SEARCH_HF;
					}
				}
			}
		}
	}
	/* candidates: best offset hypothesis per timing, a few peaks per N_ID2
	 * (the work arrays are free now) */
	best = (float *)wr;
	besth = (short *)wi;
	for (nid2 = 0; nid2 < 3; nid2++) {
		for (n = 0; n < HF; n++) {
			best[n] = 0;
			besth[n] = 0;
			for (hy = 0; hy < lc->hyps; hy++) {
				float v = lc->metric[(nid2 * lc->hyps + hy) * HF + n];
				if (v > best[n]) {
					best[n] = v;
					besth[n] = hy;
				}
			}
		}
		for (i = 0; i < CAND_PER_NID2; i++) {
			int bn = -1;
			float bv = PSS_MIN_METRIC;
			for (n = 0; n < HF; n++) {
				if (best[n] > bv) {
					bv = best[n];
					bn = n;
				}
			}
			if (bn < 0)
				break;
			cand[ncand].nid2 = nid2;
			cand[ncand].tau = bn;
			cand[ncand].hyp = besth[bn];
			cand[ncand].metric = bv;
			ncand++;
			for (n = bn - 16; n <= bn + 16; n++)
				best[(n + HF) % HF] = 0;
		}
	}
	/* strongest first */
	for (i = 0; i < ncand; i++) {
		int j;
		for (j = i + 1; j < ncand; j++) {
			if (cand[j].metric > cand[i].metric) {
				cand_t t = cand[i];
				cand[i] = cand[j];
				cand[j] = t;
			}
		}
	}
	for (i = 0; i < ncand && lc->ntrack < LTE_CELL_MAX; i++) {
		const float *m = lc->metric + cand[i].nid2 * lc->hyps * HF;
		int hb = cand[i].hyp, t = cand[i].tau;
		double f = lc->cfo_center + (hb - (lc->hyps - 1) / 2) * CFO_STEP;
		/* parabola over the offset hypotheses */
		if (hb > 0 && hb < lc->hyps - 1) {
			double a = m[(hb - 1) * HF + t], b = m[hb * HF + t], c = m[(hb + 1) * HF + t];
			double den = a - 2 * b + c;
			if (den < 0)
				f += 0.5 * (a - c) / den * CFO_STEP;
		}
		cand[i].tau = refine_tau(lc, s0, cand[i].nid2, t, f);
		search_sss(lc, s0, &cand[i], f, cand[i].metric);
	}
}

/*
 * tracking
 */

/* PSS correlation around the expected position: fractional position and metric */
static int pss_track(lte_cell_t *lc, lte_track_t *tr, double expect, double *pos, double *metric)
{
	int64_t c = (int64_t)floor(expect + 0.5);
	int64_t w = c - TRACK_WIN;
	int len = LTE_NFFT + 2 * TRACK_WIN, n, o, bo = 0;
	float complex x[LTE_NFFT + 2 * TRACK_WIN];
	float complex *src = bufp(lc, w);
	double complex rot = cexp(I * derot_phase(tr->cfo, w)), step = cexp(-I * 2.0 * M_PI * tr->cfo / FS);
	double mag[2 * TRACK_WIN + 1], e[2 * TRACK_WIN + 1], d;

	for (n = 0; n < len; n++) {
		x[n] = src[n] * rot;
		rot *= step;
	}
	for (o = 0; o <= 2 * TRACK_WIN; o++) {
		float complex acc = 0;
		float en = 0;
		for (n = 0; n < LTE_NFFT; n++) {
			acc += x[o + n] * conjf(tr->pss[n]);
			en += crealf(x[o + n]) * crealf(x[o + n]) + cimagf(x[o + n]) * cimagf(x[o + n]);
		}
		mag[o] = cabsf(acc);
		e[o] = en;
		if (mag[o] > mag[bo])
			bo = o;
	}
	*metric = (e[bo] > 0) ? mag[bo] * mag[bo] / (e[bo] * tr->pss_energy) : 0;
	if (bo == 0 || bo == 2 * TRACK_WIN)
		return -1;
	d = mag[bo - 1] - 2 * mag[bo] + mag[bo + 1];
	*pos = (double)(w + bo) + ((d < 0) ? 0.5 * (mag[bo - 1] - mag[bo + 1]) / d : 0.0);
	return 0;
}

static void reg_add(lte_track_t *tr, double k, double y)
{
	if (tr->reg_n == 0) {
		tr->k0 = k;
		tr->y0 = y;
	}
	k -= tr->k0;
	/* the deviation from the nominal half-frame: small numbers, so the
	 * sums of squares keep their precision */
	y -= tr->y0 + k * HF;
	tr->sk += k;
	tr->sy += y;
	tr->skk += k * k;
	tr->sky += k * y;
	tr->syy += y * y;
	tr->reg_n++;
}

/* deviation from the nominal half-frame (slope - 9600) and intercept */
static int reg_fit_dev(const lte_track_t *tr, double *dslope, double *icpt)
{
	double n = tr->reg_n, den = n * tr->skk - tr->sk * tr->sk;

	if (tr->reg_n < 2 || den <= 0)
		return -1;
	*dslope = (n * tr->sky - tr->sk * tr->sy) / den;
	*icpt = (tr->sy - *dslope * tr->sk) / n;
	return 0;
}

/* samples per half-frame and the predicted PSS position of half-frame k */
static int reg_fit(const lte_track_t *tr, double *slope, double *icpt)
{
	if (reg_fit_dev(tr, slope, icpt) < 0)
		return -1;
	*slope += HF;
	return 0;
}

static double reg_predict(const lte_track_t *tr, double k)
{
	double b, a;

	if (reg_fit(tr, &b, &a) < 0)
		return 0;
	return tr->y0 + a + b * (k - tr->k0);
}

/* linear interpolation of a channel estimate on a grid of every third
 * subcarrier to all 72 subcarriers (in FFT bin positions) */
static void interp72(const float complex *sum, const int *cnt, float complex *h)
{
	int kp, pos[LTE_SC], np = 0, i;
	float complex val[LTE_SC];

	for (kp = 0; kp < LTE_SC; kp++) {
		if (cnt[kp]) {
			pos[np] = lte_kp_bin(kp);
			val[np] = sum[kp] / cnt[kp];
			np++;
		}
	}
	if (np == 0) {
		for (kp = 0; kp < LTE_SC; kp++)
			h[kp] = 0;
		return;
	}
	i = 0;
	for (kp = 0; kp < LTE_SC; kp++) {
		int b = lte_kp_bin(kp);
		while (i < np - 1 && pos[i + 1] <= b)
			i++;
		if (b <= pos[0])
			h[kp] = val[0];
		else if (i >= np - 1)
			h[kp] = val[np - 1];
		else {
			float a = (float)(b - pos[i]) / (pos[i + 1] - pos[i]);
			h[kp] = val[i] * (1 - a) + val[i + 1] * a;
		}
	}
}

/* soft bits of the PBCH REs of one frame for 1, 2 or 4 antenna ports */
static void pbch_soft(const float complex *y, float complex (*H)[240], int nre, int ports, float *soft)
{
	int i;

	if (ports == 1) {
		for (i = 0; i < nre; i++) {
			float complex z = conjf(H[0][i]) * y[i];
			soft[2 * i] = crealf(z);
			soft[2 * i + 1] = cimagf(z);
		}
		return;
	}
	for (i = 0; i + 1 < nre; i += 2) {
		int pa, pb;
		float complex x0, x1;
		int d;	/* symbol index of x0 */
		if (ports == 2) {
			pa = 0;
			pb = 1;
			d = i;
		} else {
			/* groups of 4: REs 0,1 ports 0,2 carry d(4i), d(4i+1);
			 * REs 2,3 ports 1,3 carry d(4i+2), d(4i+3) */
			if ((i & 2) == 0) {
				pa = 0;
				pb = 2;
			} else {
				pa = 1;
				pb = 3;
			}
			d = i;
		}
		/* r_a = ha x0 - hb x1*, r_b = ha x1 + hb x0* */
		x0 = conjf(H[pa][i]) * y[i] + H[pb][i + 1] * conjf(y[i + 1]);
		x1 = -H[pb][i] * conjf(y[i]) + conjf(H[pa][i + 1]) * y[i + 1];
		soft[2 * d] = crealf(x0);
		soft[2 * d + 1] = cimagf(x0);
		soft[2 * d + 2] = crealf(x1);
		soft[2 * d + 3] = cimagf(x1);
	}
}

/* try to decode the BCH; ports 0 = try all. Returns ports on success. */
static int bch_try(lte_track_t *tr, const float *soft, int nbits, int ports, int j, uint8_t *a, double *agree)
{
	float llr[LTE_BCH_CODED];
	uint8_t c[LTE_BCH_K], e[LTE_PBCH_BITS_N];
	double s = 0, sa = 0;
	int i;

	memset(llr, 0, sizeof(llr));
	for (i = 0; i < nbits; i++) {
		int bi = j * nbits + i;
		float v = tr->scr[bi] ? -soft[i] : soft[i];
		llr[lte_bch_coded_index(bi)] += v;
	}
	lte_bch_viterbi(llr, c);
	if (!lte_bch_check(c, ports))
		return 0;
	/* the decode must explain the received bits (rejects e.g. an all
	 * zero block from no signal, which passes the 1 port CRC) */
	lte_bch_encode(c, ports, e, 4 * nbits);
	for (i = 0; i < nbits; i++) {
		int bi = j * nbits + i;
		float v = tr->scr[bi] ? -soft[i] : soft[i];
		s += e[bi] ? -v : v;
		sa += fabsf(v);
	}
	*agree = (sa > 0) ? s / sa : 0;
	if (*agree < MIB_MIN_AGREE)
		return 0;
	memcpy(a, c, LTE_BCH_A);
	return ports;
}

static void mib_result(lte_cell_t *lc, lte_track_t *tr, const uint8_t *a, int ports, int j, double agree, double t0)
{
	lte_cell_info_t *ci = &tr->info;
	lte_mib_t mib;
	int sfn;

	lte_mib_parse(a, &mib);
	sfn = (mib.sfn8 << 2) | j;
	ci->mib_good++;
	lc->new_mib++;
	tr->since_good = 0;
	ci->sfn = sfn;
	ci->sfn_pos = t0;
	ci->mib_agree = agree;
	if (tr->have_ref) {
		int df = (int)floor((t0 - tr->ref_t0) / tr->spf + 0.5);
		int expect = (tr->ref_sfn + df) & 1023;
		if (sfn == expect && ports == tr->ref_ports && mib.n_rb == tr->ref_mib.n_rb
		 && mib.phich_ext == tr->ref_mib.phich_ext && mib.phich_res == tr->ref_mib.phich_res) {
			ci->mib_consistent++;
			return;
		}
		ci->mib_inconsistent++;
		if (ci->mib_inconsistent <= ci->mib_consistent)
			return;
		/* the reference was the odd one: start over from this decode */
		ci->mib_inconsistent = 0;
	}
	tr->have_ref = 1;
	tr->ref_sfn = sfn;
	tr->ref_t0 = t0;
	tr->ref_ports = ports;
	tr->ref_mib = mib;
	ci->mib_consistent = 1;
	ci->ports = ports;
	ci->n_rb = mib.n_rb;
	ci->phich_ext = mib.phich_ext;
	ci->phich_res = mib.phich_res;
	ci->spare = mib.spare;
}

static void update_info(lte_cell_t *lc, lte_track_t *tr)
{
	lte_cell_info_t *ci = &tr->info;
	double b, a, im = 0, isd = 0;

	ci->cfo_hz = tr->cfo;
	if (tr->intra_n >= 1) {
		double v;
		im = tr->intra_s / tr->intra_n;
		v = tr->intra_s2 / tr->intra_n - im * im;
		isd = sqrt((v > 0 ? v : 0) / tr->intra_n);
		ci->cfo_hz = tr->cfo_fixed + im;
		ci->cfo_std_hz = isd;
		ci->cfo_points = tr->intra_n;
	}
	/* timing drift of the frames: the sample clock's offset */
	if (tr->reg_n >= REG_MIN && reg_fit_dev(tr, &b, &a) == 0) {
		double n = tr->reg_n, sxx = tr->skk - tr->sk * tr->sk / n;
		double ssr = tr->syy - a * tr->sy - b * tr->sky;	/* residual sum of squares */
		ci->timing_ppm = -b / HF * 1e6;
		ci->timing_points = tr->reg_n;
		ci->timing_std_ppm = (sxx > 0 && n > 2 && ssr > 0) ? sqrt(ssr / (n - 2) / sxx) / HF * 1e6 : 0;
	}
	/* Three baselines of the CRS phase (port 0), each resolving the
	 * ambiguity of the next: slot to slot 0.5 ms (+-1 kHz, coarse),
	 * subframe 0 to 5 (5 ms, +-100 Hz), frame to frame (10 ms, +-50 Hz,
	 * the finest). The sums are circular means (no unwrapping of single
	 * steps). The slot to slot mean reads 10 - 20 Hz off on weak cells and
	 * up to 80 Hz when a receiver spur falls on the CRS; with the carrier
	 * frequency known, the timing drift (same oscillator as the LO) joins
	 * it to pick the 5 ms alias. */
	if (tr->n5 >= 3 && tr->inter_n >= 3) {
		double t5 = tr->spf / 2 / FS, t10 = tr->spf / FS;
		double a5 = 1.0 / t5, a10 = 1.0 / t10;
		double m5 = carg(tr->z5) / (2.0 * M_PI * t5);
		double m10 = carg(tr->zi) / (2.0 * M_PI * t10);
		double r5 = cabs(tr->u5) / tr->n5, r10 = cabs(tr->ui) / tr->inter_n;
		double sd5 = sqrt(-2.0 * log(r5 > 1e-9 ? (r5 < 1 ? r5 : 1) : 1e-9)) / (2.0 * M_PI * t5);
		double sd10 = sqrt(-2.0 * log(r10 > 1e-9 ? (r10 < 1 ? r10 : 1) : 1e-9)) / (2.0 * M_PI * t10);
		double f5, f10, sem5, ma, mb, coarse, csd;

		/* coarse: slot to slot (with its bias as an error) */
		coarse = im;
		csd = sqrt(isd * isd + 15.0 * 15.0);
		if (lc->carrier_hz > 0 && ci->timing_points >= REG_MIN) {
			/* timing: allow 0.02 ppm between the cell's clock and the frame timing */
			double tsd = sqrt(ci->timing_std_ppm * ci->timing_std_ppm + 0.02 * 0.02) * 1e-6 * lc->carrier_hz;
			double tres = ci->timing_ppm * 1e-6 * lc->carrier_hz - tr->cfo_fixed;
			double w1 = 1.0 / (csd * csd), w2 = 1.0 / (tsd * tsd);
			coarse = (coarse * w1 + tres * w2) / (w1 + w2);
			csd = sqrt(1.0 / (w1 + w2));
		}
		f5 = m5 + floor((coarse - m5) / a5 + 0.5) * a5;
		f10 = m10 + floor((f5 - m10) / a10 + 0.5) * a10;
		sem5 = sd5 / sqrt(tr->n5);
		ma = (a5 / 2 - fabs(coarse - f5)) / csd;
		mb = (sem5 > 0) ? (a10 / 2 - fabs(f5 - f10)) / sem5 : 99;
		if (lc->debug && ci->frames % 50 == 0)
			fprintf(stderr, "PCI %d frame %d slot %.1f +- %.1f, coarse %.1f +- %.1f, 5 ms %.2f (sd %.1f), 10 ms %.2f (sd %.1f), margins %.1f %.1f\n",
				ci->pci, ci->frames, im, isd, coarse, csd, f5, sd5, f10, sd10, ma, mb);
		ci->cfo_hz = tr->cfo_fixed + f10;
		/* consecutive frame steps telescope: the error of the mean is
		 * about the error of one step divided by the steps */
		ci->cfo_std_hz = sd10 / tr->inter_n;
		ci->cfo_points = tr->inter_n;
		ci->cfo_alias_margin = (ma < mb) ? ma : mb;
	}
	if (lc->carrier_hz > 0)
		ci->cfo_ppm = ci->cfo_hz / lc->carrier_hz * 1e6;
	if (tr->q_n) {
		double crs = tr->q_crs / tr->q_n;
		double noise = tr->q_nn ? tr->q_noise / tr->q_nn : 0;
		double rssi = tr->q_rn ? tr->q_rssi / tr->q_rn : 0;
		double sig = crs - noise;
		ci->rsrp_dbfs = 10 * log10(crs + 1e-30);
		ci->rssi_dbfs = 10 * log10(rssi + 1e-30);
		ci->rsrq_db = 10 * log10(6.0 * crs / (rssi + 1e-30) + 1e-30);
		ci->sinr_db = (noise > 0 && sig > 0) ? 10 * log10(sig / noise) : (sig > 0 ? 99.0 : -99.0);
	}
}

static void process_frame(lte_cell_t *lc, lte_track_t *tr)
{
	lte_cell_info_t *ci = &tr->info;
	static float complex Y[2][7][LTE_NFFT];
	float complex hraw[2][7][4][12];
	float complex Hs[4][LTE_SC], hsum[LTE_SC];
	float complex y[240], H[4][240];
	float soft[480];
	int hcnt[LTE_SC];
	double t0, meas[2], met[2], slope, icpt;
	int ok[2], h, s, l, p, m, kp, nre, nbits, lc1 = tr->nsymb - 3, vs = ci->pci % 6;
	int need[2][7];

	ci->frames++;
	/* PSS timing, both half-frames */
	for (h = 0; h < 2; h++) {
		double k = (double)(tr->hf + h);
		double expect = (tr->reg_n >= REG_MIN) ? reg_predict(tr, k) : tr->t0 + tr->pss_off + h * tr->spf / 2;
		ok[h] = (pss_track(lc, tr, expect, &meas[h], &met[h]) == 0 && met[h] >= TRACK_MIN);
		if (ok[h] && tr->reg_n >= REG_MIN && fabs(meas[h] - expect) > 1.5)
			ok[h] = 0;
		if (ok[h]) {
			reg_add(tr, k, meas[h]);
			tr->pss_miss = 0;
		} else
			tr->pss_miss++;
	}
	if (tr->reg_n >= REG_MIN)
		t0 = reg_predict(tr, (double)tr->hf) - tr->pss_off;
	else if (ok[0])
		t0 = meas[0] - tr->pss_off;
	else
		t0 = tr->t0;
	tr->hf += 2;
	if (tr->reg_n >= REG_MIN && reg_fit(tr, &slope, &icpt) == 0)
		tr->spf = 2 * slope;
	tr->t0 = t0 + tr->spf;
	if (tr->pss_miss > LOST_PSS) {
		ci->lost = 1;
		return;
	}

	/* symbols of subframe 0 */
	memset(need, 0, sizeof(need));
	need[0][0] = need[0][1] = need[0][lc1] = 1;
	need[1][0] = need[1][1] = need[1][2] = need[1][3] = need[1][lc1] = 1;
	for (s = 0; s < 2; s++) {
		for (l = 0; l < tr->nsymb; l++) {
			if (need[s][l])
				symbol_fft(lc, t0 + sym_start(ci->ext_cp, s, l), tr->cfo, Y[s][l]);
		}
	}
	/* raw CRS estimates */
	for (s = 0; s < 2; s++) {
		for (p = 0; p < 4; p++) {
			int ls = (p < 2) ? 0 : 1;
			int le = (p < 2) ? lc1 : 1;
			for (l = ls; l <= le; l += (le - ls) ? (le - ls) : 1) {
				int off = lte_crs_offset(ci->pci, p, s, l);
				for (m = 0; m < 12; m++) {
					kp = 6 * m + off;
					hraw[s][l][p][m] = Y[s][l][bin_idx(lte_kp_bin(kp))] * conjf(tr->crs[s][l][m]);
				}
			}
		}
	}

	/* spurs of the receiver (narrow tones, fixed in the SDR's frequency)
	 * pull the phase of the CRS they hit: CRS positions whose long term
	 * power is far above the median of all are left out */
	{
		double sorted[24], med;
		int li, a, b;
		for (li = 0; li < 2; li++) {
			l = li ? lc1 : 0;
			for (m = 0; m < 12; m++) {
				float complex x0 = hraw[0][l][0][m], x1 = hraw[1][l][0][m];
				tr->spow[li][m] += crealf(x0 * conjf(x0)) + crealf(x1 * conjf(x1));
			}
		}
		tr->spow_n++;
		for (a = 0; a < 24; a++)
			sorted[a] = tr->spow[a / 12][a % 12];
		for (a = 1; a < 24; a++) {
			double t = sorted[a];
			for (b = a - 1; b >= 0 && sorted[b] > t; b--)
				sorted[b + 1] = sorted[b];
			sorted[b + 1] = t;
		}
		med = 0.5 * (sorted[11] + sorted[12]);
		for (a = 0; a < 24; a++)
			tr->smask[a / 12][a % 12] = (tr->spow_n >= SPUR_FRAMES && tr->spow[a / 12][a % 12] > SPUR_RATIO * med);
	}

	/* quality and offset from port 0 (symbols 0 and lc1 of both slots) */
	{
		double complex zs = 0;
		float complex v[48];
		int idx = 0, li;
		for (li = 0; li < 2; li++) {
			double complex z = 0;
			double rot_noise = 0, pw = 0;
			float complex ph;
			int used = 0;
			l = li ? lc1 : 0;
			for (m = 0; m < 12; m++) {
				if (!tr->smask[li][m])
					z += hraw[1][l][0][m] * conjf(hraw[0][l][0][m]);
			}
			zs += z;
			ph = (cabs(z) > 0) ? z / cabs(z) : 1;
			for (m = 0; m < 12; m++) {
				float complex a = hraw[0][l][0][m], b = hraw[1][l][0][m];
				float complex dlt = b - a * ph;
				if (tr->smask[li][m]) {
					v[idx++] = 0;
					v[idx++] = 0;
					continue;
				}
				rot_noise += crealf(dlt * conjf(dlt)) / 2;
				pw += crealf(a * conjf(a)) + crealf(b * conjf(b));
				v[idx++] = a;
				v[idx++] = b;
				used++;
			}
			if (!used)
				continue;
			tr->q_crs += pw / (2 * used);
			tr->q_n++;
			tr->q_noise += rot_noise / used;
			tr->q_nn++;
			for (s = 0; s < 2; s++) {
				double r = 0;
				for (kp = 0; kp < LTE_SC; kp++) {
					float complex yy = Y[s][l][bin_idx(lte_kp_bin(kp))];
					r += crealf(yy * conjf(yy));
				}
				tr->q_rssi += r;
				tr->q_rn++;
			}
		}
		/* slot to slot (0.5 ms): +-1 kHz */
		if (cabs(zs) > 0) {
			double resid = carg(zs) * FS / (2.0 * M_PI * SLOT);
			if (tr->refine > SETTLE_FRAMES) {
				/* first frames: follow each estimate */
				tr->cfo += resid;
				tr->refine--;
			} else if (tr->refine > 0) {
				/* then the mean of several, which must be well
				 * inside the +-50 Hz of the frame to frame step */
				tr->settle_s += resid;
				tr->settle_n++;
				if (--tr->refine == 0) {
					tr->cfo += tr->settle_s / tr->settle_n;
					tr->cfo_fixed = tr->cfo;
					if (lc->debug)
						fprintf(stderr, "PCI %d settled cfo %.1f (mean resid %.1f)\n", ci->pci, tr->cfo,
							tr->settle_s / tr->settle_n);
				}
			} else {
				double complex z5 = 0;
				int s5;
				tr->intra_s += resid;
				tr->intra_s2 += resid * resid;
				tr->intra_n++;
				/* subframe 0 to subframe 5 (5 ms): +-100 Hz */
				for (s5 = 0; s5 < 2; s5++) {
					int li;
					for (li = 0; li < 2; li++) {
						float complex Y5[LTE_NFFT];
						int off;
						l = li ? lc1 : 0;
						off = lte_crs_offset(ci->pci, 0, s5, l);
						symbol_fft(lc, t0 + tr->spf / 2 + sym_start(ci->ext_cp, s5, l), tr->cfo, Y5);
						for (m = 0; m < 12; m++) {
							float complex h5;
							if (tr->smask[li][m])
								continue;
							h5 = Y5[bin_idx(lte_kp_bin(6 * m + off))] * conjf(tr->crs5[s5][l][m]);
							z5 += h5 * conjf(hraw[s5][l][0][m]);
						}
					}
				}
				if (cabs(z5) > 0) {
					tr->z5 += z5;
					tr->u5 += z5 / cabs(z5);
					tr->n5++;
				}
				/* frame to frame (10 ms): +-50 Hz, consecutive frames only */
				if (tr->have_prev && fabs(t0 - tr->prev_t0 - tr->spf) < 4) {
					double complex zf = 0;
					for (m = 0; m < 48; m++)
						zf += v[m] * conjf(tr->prev_h[m]);
					if (cabs(zf) > 0) {
						tr->zi += zf;
						tr->ui += zf / cabs(zf);
						tr->inter_n++;
					}
				}
				memcpy(tr->prev_h, v, sizeof(v));
				tr->prev_t0 = t0;
				tr->have_prev = 1;
			}
		}
	}

	/* channel per port for the PBCH (slot 1) */
	for (p = 0; p < 4; p++) {
		int src[3][2], ns = 0, i;
		memset(hsum, 0, sizeof(hsum));
		memset(hcnt, 0, sizeof(hcnt));
		if (p < 2) {
			src[0][0] = 0; src[0][1] = lc1;
			src[1][0] = 1; src[1][1] = 0;
			src[2][0] = 1; src[2][1] = lc1;
			ns = 3;
		} else {
			src[0][0] = 0; src[0][1] = 1;
			src[1][0] = 1; src[1][1] = 1;
			ns = 2;
		}
		for (i = 0; i < ns; i++) {
			int off;
			s = src[i][0];
			l = src[i][1];
			off = lte_crs_offset(ci->pci, p, s, l);
			for (m = 0; m < 12; m++) {
				kp = 6 * m + off;
				hsum[kp] += hraw[s][l][p][m];
				hcnt[kp]++;
			}
		}
		interp72(hsum, hcnt, Hs[p]);
	}
	/* PBCH resource elements */
	nre = 0;
	for (l = 0; l < 4; l++) {
		int crs_sym = (l == 0 || l == 1 || (ci->ext_cp && l == 3));
		for (kp = 0; kp < LTE_SC; kp++) {
			if (crs_sym && kp % 3 == vs % 3)
				continue;
			y[nre] = Y[1][l][bin_idx(lte_kp_bin(kp))];
			for (p = 0; p < 4; p++)
				H[p][nre] = Hs[p][kp];
			nre++;
		}
	}
	nbits = 2 * nre;

	/* BCH */
	{
		static const int try_ports[3] = { 1, 2, 4 };
		uint8_t a[LTE_BCH_A];
		double agree = 0;
		int done = 0, i, j;
		if (tr->have_ref) {
			int df = (int)floor((t0 - tr->ref_t0) / tr->spf + 0.5);
			j = (tr->ref_sfn + df) & 3;
			pbch_soft(y, H, nre, tr->ref_ports, soft);
			if (bch_try(tr, soft, nbits, tr->ref_ports, j, a, &agree)) {
				mib_result(lc, tr, a, tr->ref_ports, j, agree, t0);
				done = 1;
			}
		}
		if (!done && !lte_cell_confirmed(ci)) {
			for (i = 0; i < 3 && !done; i++) {
				pbch_soft(y, H, nre, try_ports[i], soft);
				for (j = 0; j < 4 && !done; j++) {
					if (bch_try(tr, soft, nbits, try_ports[i], j, a, &agree)) {
						mib_result(lc, tr, a, try_ports[i], j, agree, t0);
						done = 1;
					}
				}
			}
		}
		if (!done) {
			ci->mib_bad++;
			tr->since_good++;
			if (!ci->mib_good && tr->since_good > LOST_NO_MIB)
				ci->lost = 1;
		}
	}
	update_info(lc, tr);
}

/*
 * stream handling
 */

static void run(lte_cell_t *lc)
{
	int64_t end;
	int progress, i;

	do {
		progress = 0;
		end = lc->buf_pos + lc->len;
		if (lc->ntrack == 0 && lc->searches < SEARCH_MAX && end >= lc->search_at + SEARCH_SPAN) {
			search(lc);
			progress = 1;
		}
		for (i = 0; i < lc->ntrack; i++) {
			lte_track_t *tr = lc->track[i];
			if (tr->info.lost)
				continue;
			if ((double)end >= tr->t0 + FRAME_NEED) {
				process_frame(lc, tr);
				progress = 1;
			}
		}
	} while (progress);
}

static void trim(lte_cell_t *lc)
{
	int64_t keep = lc->buf_pos + lc->len;
	int i, drop;

	if (lc->ntrack == 0 && lc->searches < SEARCH_MAX && lc->search_at - SEARCH_PRE < keep)
		keep = lc->search_at - SEARCH_PRE;
	for (i = 0; i < lc->ntrack; i++) {
		lte_track_t *tr = lc->track[i];
		int64_t k;
		if (tr->info.lost)
			continue;
		k = (int64_t)floor(tr->t0) - 2 * SEARCH_PRE;
		if (k < keep)
			keep = k;
	}
	drop = (int)(keep - lc->buf_pos);
	if (drop <= 0)
		return;
	if (drop > lc->len)
		drop = lc->len;
	memmove(lc->buf, lc->buf + drop, (lc->len - drop) * sizeof(float complex));
	lc->len -= drop;
	lc->buf_pos += drop;
}

int lte_cell_init(lte_cell_t *lc, double rate, double carrier_hz, double cfo_center, double cfo_max)
{
	memset(lc, 0, sizeof(*lc));
	/* the channel comes at the DSP rate from the SDR layer (sdr_engine or
	 * libchannelizer): no resampling here */
	if (fabs(rate - FS) > 1e-3)
		return -EINVAL;
	lc->carrier_hz = carrier_hz;
	lc->cfo_center = cfo_center;
	lc->cfo_max = cfo_max;
	lc->hyps = 2 * (int)ceil(cfo_max / CFO_STEP) + 1;
	lc->cap = BUF_CAP;
	lc->buf = calloc(lc->cap, sizeof(float complex));
	lc->metric = calloc(3 * lc->hyps * HF, sizeof(float));
	if (!lc->buf || !lc->metric) {
		lte_cell_exit(lc);
		return -ENOMEM;
	}
	lc->search_at = SEARCH_PRE;
	return 0;
}

void lte_cell_exit(lte_cell_t *lc)
{
	int i;

	for (i = 0; i < lc->ntrack; i++)
		free(lc->track[i]);
	free(lc->buf);
	free(lc->metric);
	free(lc->swork);
	memset(lc, 0, sizeof(*lc));
}

int lte_cell_process(lte_cell_t *lc, const float *iq, int num)
{
	lc->new_mib = 0;
	if (!lc->buf)
		return 0;
	while (num > 0) {
		int space, n, i;
		run(lc);
		trim(lc);
		space = lc->cap - lc->len;
		if (space <= 0) {
			/* nothing can be dropped: give up the oldest work */
			lc->len = 0;
			lc->buf_pos += lc->cap;
			space = lc->cap;
		}
		n = (num < space) ? num : space;
		for (i = 0; i < n; i++)
			lc->buf[lc->len + i] = iq[2 * i] + I * iq[2 * i + 1];
		lc->len += n;
		iq += 2 * n;
		num -= n;
	}
	run(lc);
	trim(lc);
	return lc->new_mib;
}

int lte_cell_count(const lte_cell_t *lc)
{
	return lc->ntrack;
}

const lte_cell_info_t *lte_cell_get(const lte_cell_t *lc, int i)
{
	if (i < 0 || i >= lc->ntrack)
		return NULL;
	return &lc->track[i]->info;
}

int lte_cell_confirmed(const lte_cell_info_t *ci)
{
	return ci->mib_consistent >= 2;
}
