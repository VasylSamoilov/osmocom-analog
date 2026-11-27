/* LTE cell search test: self test on synthetic signals, and IQ captures
 *
 * test_lte_cell selftest
 * test_lte_cell file <capture> [options] -e <earfcn>[:<last>] ...
 *   -t sc16|cf32|cu8   sample format (default sc16)
 *   -r <rate>          sample rate of the capture (Hz)
 *   -c <freq>          centre frequency of the capture (Hz)
 *   -e <n>[:<m>]       EARFCN(s) to try (all in the capture's span)
 *   -b <band>          all EARFCNs of a band inside the capture's span
 *   -m <hz>            carrier offset search range (+-, default 40000)
 *   -p <ppm>           expected SDR offset (centre of the search)
 *   -s <sec>           use at most this much of the capture
 *   -k <sec>           skip this much at the start (default 0.05)
 *   -q                 only print cells
 * The capture's EARFCN is cut out with libchannelizer (1.92 MS/s), as the
 * SDR layer will give it. Environment: LTE_DEBUG=1 prints the search and
 * offset stages, LTE_DUMP=<file> writes the channel (cf32, last EARFCN).
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
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <complex.h>

#include "../libcalibrate/lte_phy.h"
#include "../libcalibrate/lte_cell.h"
#include "../libcalibrate/lte_bands.h"
#include "../libfft/fft.h"
#include "../libchannelizer/channelizer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		failures++; \
		printf("FAIL: "); \
		printf(__VA_ARGS__); \
		printf("\n"); \
	} \
} while (0)

static double cpu_now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static double gauss(void)
{
	double u1 = (rand() + 1.0) / (RAND_MAX + 2.0), u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
	return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/*
 * synthetic LTE downlink (centre 6 RB at 1.92 MS/s)
 */

typedef struct synth {
	int	pci, ports, tdd, ext_cp, n_rb_code, phich_ext, phich_res;
	int	sfn;
	double	load;		/* power of the random data REs */
	float complex gain[4];	/* flat channel per port */
} synth_t;

static void synth_frame(const synth_t *sy, float complex *out)
{
	int nsymb = sy->ext_cp ? 6 : 7, lc1 = nsymb - 3, vs = sy->pci % 6;
	int nid1 = sy->pci / 3, nid2 = sy->pci % 3;
	static float complex grid[4][20][7][LTE_SC];
	static uint8_t used[20][7][LTE_SC];
	float complex pss[62];
	float sss0[62], sss5[62];
	uint8_t mib[24], e[LTE_PBCH_BITS_N], scr[LTE_PBCH_BITS_N];
	int p, s, l, k, m, n, i, pos = 0;
	int nbits = sy->ext_cp ? LTE_PBCH_BITS_E / 4 : LTE_PBCH_BITS_N / 4;
	int j = sy->sfn & 3;

	memset(grid, 0, sizeof(grid));
	memset(used, 0, sizeof(used));
	/* CRS (positions of all ports of the cell) */
	for (s = 0; s < 20; s++) {
		for (p = 0; p < sy->ports; p++) {
			int ls[2], nl = (p < 2) ? 2 : 1;
			ls[0] = (p < 2) ? 0 : 1;
			ls[1] = lc1;
			for (i = 0; i < nl; i++) {
				float complex r[12];
				int off;
				l = ls[i];
				off = lte_crs_offset(sy->pci, p, s, l);
				lte_crs_seq(sy->pci, s, l, sy->ext_cp, r);
				for (m = 0; m < 12; m++) {
					grid[p][s][l][6 * m + off] = r[m];
					used[s][l][6 * m + off] = 1;
				}
			}
		}
	}
	/* PSS / SSS */
	lte_pss_seq(nid2, pss);
	lte_sss_seq(nid1, nid2, 0, sss0);
	lte_sss_seq(nid1, nid2, 1, sss5);
	for (i = 0; i < 2; i++) {
		int ps, pl, ss, sl;
		if (!sy->tdd) {
			ps = 10 * i;
			pl = nsymb - 1;
			ss = ps;
			sl = nsymb - 2;
		} else {
			ps = 2 + 10 * i;
			pl = 2;
			ss = 1 + 10 * i;
			sl = nsymb - 1;
		}
		for (k = 0; k < LTE_SC; k++) {
			used[ps][pl][k] = 1;
			used[ss][sl][k] = 1;
		}
		for (n = 0; n < 62; n++) {
			grid[0][ps][pl][n + 5] = pss[n];
			grid[0][ss][sl][n + 5] = i ? sss5[n] : sss0[n];
		}
	}
	/* PBCH */
	memset(mib, 0, sizeof(mib));
	for (i = 0; i < 3; i++)
		mib[i] = (sy->n_rb_code >> (2 - i)) & 1;
	mib[3] = sy->phich_ext;
	mib[4] = (sy->phich_res >> 1) & 1;
	mib[5] = sy->phich_res & 1;
	for (i = 0; i < 8; i++)
		mib[6 + i] = ((sy->sfn >> 2) >> (7 - i)) & 1;
	lte_bch_encode(mib, sy->ports, e, 4 * nbits);
	lte_gold(sy->pci, scr, LTE_PBCH_BITS_N);
	{
		float complex d[240], y[4][240];
		int nre = nbits / 2;
		for (i = 0; i < nre; i++) {
			int b0 = e[j * nbits + 2 * i] ^ scr[j * nbits + 2 * i];
			int b1 = e[j * nbits + 2 * i + 1] ^ scr[j * nbits + 2 * i + 1];
			d[i] = ((1 - 2 * b0) + I * (1 - 2 * b1)) * (float)M_SQRT1_2;
		}
		memset(y, 0, sizeof(y));
		for (i = 0; i < nre; i++) {
			if (sy->ports == 1)
				y[0][i] = d[i];
		}
		if (sy->ports == 2) {
			for (i = 0; i < nre; i += 2) {
				y[0][i] = d[i];
				y[1][i] = -conjf(d[i + 1]);
				y[0][i + 1] = d[i + 1];
				y[1][i + 1] = conjf(d[i]);
			}
		}
		if (sy->ports == 4) {
			for (i = 0; i < nre; i += 4) {
				y[0][i] = d[i];
				y[2][i] = -conjf(d[i + 1]);
				y[0][i + 1] = d[i + 1];
				y[2][i + 1] = conjf(d[i]);
				y[1][i + 2] = d[i + 2];
				y[3][i + 2] = -conjf(d[i + 3]);
				y[1][i + 3] = d[i + 3];
				y[3][i + 3] = conjf(d[i + 2]);
			}
		}
		i = 0;
		for (l = 0; l < 4; l++) {
			int crs_sym = (l == 0 || l == 1 || (sy->ext_cp && l == 3));
			for (k = 0; k < LTE_SC; k++) {
				if (crs_sym && k % 3 == vs % 3) {
					used[1][l][k] = 1;
					continue;
				}
				for (p = 0; p < sy->ports; p++)
					grid[p][1][l][k] = y[p][i];
				used[1][l][k] = 1;
				i++;
			}
		}
	}
	/* random data (port 0) */
	for (s = 0; s < 20; s++) {
		for (l = 0; l < nsymb; l++) {
			for (k = 0; k < LTE_SC; k++) {
				if (used[s][l][k])
					continue;
				grid[0][s][l][k] = ((rand() & 1 ? 1 : -1) + I * (rand() & 1 ? 1 : -1)) * (float)(M_SQRT1_2 * sqrt(sy->load));
			}
		}
	}
	/* OFDM modulation */
	for (s = 0; s < 20; s++) {
		for (l = 0; l < nsymb; l++) {
			double re[LTE_NFFT], im[LTE_NFFT];
			int cp = sy->ext_cp ? 32 : (l == 0 ? 10 : 9);
			memset(re, 0, sizeof(re));
			memset(im, 0, sizeof(im));
			for (k = 0; k < LTE_SC; k++) {
				int b = (lte_kp_bin(k) + LTE_NFFT) & (LTE_NFFT - 1);
				float complex v = 0;
				for (p = 0; p < sy->ports; p++)
					v += grid[p][s][l][k] * sy->gain[p];
				re[b] = crealf(v);
				im[b] = cimagf(v);
			}
			fft_process(-1, 7, re, im);
			for (n = 0; n < cp; n++)
				out[pos++] = (re[LTE_NFFT - cp + n] + I * im[LTE_NFFT - cp + n]) / LTE_NFFT;
			for (n = 0; n < LTE_NFFT; n++)
				out[pos++] = (re[n] + I * im[n]) / LTE_NFFT;
		}
	}
}

/* run a synthetic cell through lte_cell; drift_ppm makes the sample clock
 * differ (the module is told a slightly wrong input rate) */
static void synth_run(const char *name, synth_t *sy, int frames, double cfo, double drift_ppm, double snr_db,
		      int expect_ok)
{
	int total = frames * 19200, nout, f, n, i, found = -1, pos;
	float complex *sig = malloc(total * sizeof(float complex));
	float *iq = malloc(2 * total * sizeof(float));
	lte_cell_t lc;
	double noise, pw = 0, phase = 0, t, step = 1.0 + drift_ppm * 1e-6;
	const lte_cell_info_t *ci = NULL;
	int start_sfn = sy->sfn;

	for (f = 0; f < frames; f++) {
		synth_frame(sy, sig + f * 19200);
		sy->sfn = (sy->sfn + 1) & 1023;
	}
	/* noise relative to the mean signal power */
	for (n = 0; n < total; n++)
		pw += crealf(sig[n] * conjf(sig[n]));
	pw /= total;
	noise = sqrt(pw / pow(10, snr_db / 10) / 2);
	/* a slow SDR clock: sample k is taken at transmitter sample k * step
	 * (windowed sinc interpolation, test only; the signal fills +-540 kHz
	 * of +-960 kHz), carrier offset, noise; start in the middle of a frame */
	nout = 0;
	for (;;) {
		double tt = 7000 + nout * step;
		int i0 = (int)floor(tt), k;
		float complex v = 0;
		if (i0 + 16 >= total)
			break;
		for (k = -15; k <= 16; k++) {
			double x = i0 + k - tt, a = 0.8 * x;
			double w = 0.42 + 0.5 * cos(M_PI * x / 16.5) + 0.08 * cos(2 * M_PI * x / 16.5);
			double h = (fabs(a) < 1e-9) ? 0.8 : sin(M_PI * a) / (M_PI * x);
			if (i0 + k >= 0)
				v += sig[i0 + k] * (float)(h * w);
		}
		v *= (float complex)cexp(I * phase);
		phase += 2.0 * M_PI * cfo / 1.92e6;
		if (phase > M_PI)
			phase -= 2 * M_PI;
		iq[2 * nout] = crealf(v) + noise * gauss();
		iq[2 * nout + 1] = cimagf(v) + noise * gauss();
		nout++;
	}
	/* the carrier frequency is known where offset and drift belong to one
	 * clock */
	lte_cell_init(&lc, 1.92e6, (drift_ppm != 0 && fabs(cfo / drift_ppm) > 1e8) ? cfo / (drift_ppm * 1e-6) : 0, 0, 40000);
	t = cpu_now();
	for (pos = 0; pos < nout; pos += 19200)
		lte_cell_process(&lc, iq + 2 * pos, (nout - pos < 19200) ? nout - pos : 19200);
	t = cpu_now() - t;
	for (i = 0; i < lte_cell_count(&lc); i++) {
		if (lte_cell_get(&lc, i)->pci == sy->pci)
			found = i;
	}
	if (found >= 0)
		ci = lte_cell_get(&lc, found);
	if (ci) {
		printf("%-26s PCI %3d %s %s MIB %d/%d cons %d ports %d BW %d SFN %d CFO %+.2f Hz (%.2f) timing %+.2f ppm "
		       "RSRP %.1f RSRQ %.1f SINR %.1f cpu %.1f%%\n",
		       name, ci->pci, ci->tdd ? "TDD" : "FDD", ci->ext_cp ? "ext" : "norm", ci->mib_good, ci->mib_bad,
		       ci->mib_consistent, ci->ports, ci->n_rb, ci->sfn, ci->cfo_hz, ci->cfo_std_hz, ci->timing_ppm,
		       ci->rsrp_dbfs, ci->rsrq_db, ci->sinr_db, 100.0 * t / (frames * 0.01));
	} else
		printf("%-26s not found (%d cells)\n", name, lte_cell_count(&lc));
	if (expect_ok) {
		static const int bw[6] = { 6, 15, 25, 50, 75, 100 };
		CHECK(ci && lte_cell_confirmed(ci), "%s: not confirmed", name);
		if (ci && lte_cell_confirmed(ci)) {
			CHECK(ci->tdd == sy->tdd && ci->ext_cp == sy->ext_cp, "%s: frame type", name);
			CHECK(ci->ports == sy->ports, "%s: ports %d", name, ci->ports);
			CHECK(ci->n_rb == bw[sy->n_rb_code], "%s: n_rb %d", name, ci->n_rb);
			/* the last frame or the one before (the capture starts
			 * mid-frame and drifts) */
			CHECK(ci->sfn == ((start_sfn + frames - 1) & 1023)
			      || ci->sfn == ((start_sfn + frames - 2) & 1023), "%s: sfn %d", name, ci->sfn);
			CHECK(fabs(ci->cfo_hz - cfo) < 2.0, "%s: cfo %.2f", name, ci->cfo_hz);
			if (drift_ppm != 0)
				CHECK(fabs(ci->timing_ppm - drift_ppm) < 0.3, "%s: timing %.3f ppm", name, ci->timing_ppm);
		}
	} else {
		for (i = 0; i < lte_cell_count(&lc); i++)
			CHECK(!lte_cell_confirmed(lte_cell_get(&lc, i)), "%s: false cell PCI %d", name,
			      lte_cell_get(&lc, i)->pci);
	}
	lte_cell_exit(&lc);
	free(sig);
	free(iq);
}

static void selftest(void)
{
	int m0, m1, i, k, err;
	uint8_t mib[24], e[LTE_PBCH_BITS_N], c[40];
	float soft[LTE_BCH_CODED];
	synth_t sy;

	/* SSS table spot checks (Table 6.11.2.1-1) */
	lte_sss_m(0, &m0, &m1);
	CHECK(m0 == 0 && m1 == 1, "sss m 0");
	lte_sss_m(30, &m0, &m1);
	CHECK(m0 == 0 && m1 == 2, "sss m 30");
	lte_sss_m(87, &m0, &m1);
	CHECK(m0 == 0 && m1 == 4, "sss m 87");
	lte_sss_m(140, &m0, &m1);
	CHECK(m0 == 0 && m1 == 6, "sss m 140");
	lte_sss_m(167, &m0, &m1);
	CHECK(m0 == 2 && m1 == 9, "sss m 167");
	lte_sss_m(114, &m0, &m1);
	CHECK(m0 == 0 && m1 == 5, "sss m 114");

	/* BCH round trip with noise */
	for (k = 0; k < 200; k++) {
		int ports = (k % 3 == 0) ? 1 : (k % 3 == 1) ? 2 : 4;
		for (i = 0; i < 24; i++)
			mib[i] = rand() & 1;
		lte_bch_encode(mib, ports, e, LTE_PBCH_BITS_N);
		memset(soft, 0, sizeof(soft));
		/* one frame's worth: 480 bits */
		for (i = 0; i < 480; i++)
			soft[lte_bch_coded_index(i)] += (e[i] ? -1.0 : 1.0) + 1.0 * gauss();
		lte_bch_viterbi(soft, c);
		err = memcmp(c, mib, 24) != 0 || !lte_bch_check(c, ports);
		CHECK(!err, "bch round trip %d ports %d", k, ports);
		if (ports != 2)
			CHECK(!lte_bch_check(c, 2), "bch mask 2 accepted for %d ports", ports);
	}

	/* bands */
	{
		int b, first, last;
		double f0, f1;
		CHECK(fabs(lte_earfcn_to_freq(1300, &b) - 1815.0e6) < 1 && b == 3, "earfcn 1300");
		CHECK(fabs(lte_earfcn_to_freq(6300, &b) - 806.0e6) < 1 && b == 20, "earfcn 6300");
		CHECK(fabs(lte_earfcn_to_freq(3100, &b) - 2655.0e6) < 1 && b == 7, "earfcn 3100");
		CHECK(fabs(lte_earfcn_to_freq(38950, &b) - 2330.0e6) < 1 && b == 40, "earfcn 38950");
		CHECK(fabs(lte_earfcn_to_freq(3900, &b) - 1854.9e6) < 1 && b == 9, "earfcn 3900");
		CHECK(lte_freq_to_earfcn(3, 1815.0e6) == 1300, "freq 1815");
		CHECK(lte_freq_to_earfcn(3, 1815.05e6) == -1, "freq off raster");
		lte_band_dl_range(1, &first, &last, &f0, &f1);
		CHECK(first == 0 && last == 599 && fabs(f0 - 2110e6) < 1 && fabs(f1 - 2169.9e6) < 1, "band 1 range");
		lte_band_dl_range(66, &first, &last, &f0, &f1);
		CHECK(first == 66436 && last == 67335 && fabs(f1 - 2199.9e6) < 1, "band 66 range");
		for (i = 0; lte_bands[i].band; i++) {
			const lte_band_t *bd = &lte_bands[i];
			CHECK(bd->n_dl_last >= bd->n_offs_dl, "band %d order", bd->band);
			if (lte_bands[i + 1].band)
				CHECK(lte_bands[i + 1].n_offs_dl > bd->n_dl_last, "band %d overlaps next", bd->band);
		}
	}

	/* end to end, synthetic */
	memset(&sy, 0, sizeof(sy));
	sy.pci = 301;
	sy.ports = 2;
	sy.n_rb_code = 3;
	sy.phich_res = 1;
	sy.sfn = 1021;
	sy.load = 1.0;
	sy.gain[0] = 0.8 + 0.3 * I;
	sy.gain[1] = -0.4 + 0.6 * I;
	sy.gain[2] = 0.5 - 0.2 * I;
	sy.gain[3] = 0.1 + 0.7 * I;
	synth_run("FDD 2 ports 10 dB", &sy, 30, 7971.3, 4.39, 10, 1);
	sy.ports = 1;
	sy.pci = 0;
	synth_run("FDD 1 port -2 dB", &sy, 30, -12345.6, -0.33, -2, 1);
	sy.ports = 4;
	sy.pci = 503;
	synth_run("FDD 4 ports 5 dB", &sy, 30, 39000, 14.76, 5, 1);
	sy.ports = 2;
	sy.pci = 77;
	sy.tdd = 1;
	synth_run("TDD 2 ports 10 dB", &sy, 30, 3000, 0, 10, 1);
	sy.ext_cp = 1;
	sy.pci = 200;
	synth_run("TDD ext CP 2 ports 10 dB", &sy, 30, -3000, 0, 10, 1);
	sy.tdd = 0;
	sy.pci = 151;
	synth_run("FDD ext CP 2 ports 10 dB", &sy, 30, 100, 2, 10, 1);
	sy.ext_cp = 0;
	sy.pci = 302;
	synth_run("FDD -8 dB (weak)", &sy, 50, 5000, 0, -8, 0);
	/* noise only */
	{
		lte_cell_t lc;
		float *iq = malloc(2 * 19200 * sizeof(float));
		int f, n;
		lte_cell_init(&lc, 1.92e6, 0, 0, 40000);
		for (f = 0; f < 60; f++) {
			for (n = 0; n < 2 * 19200; n++)
				iq[n] = 0.01 * gauss();
			lte_cell_process(&lc, iq, 19200);
		}
		printf("noise only: %d candidates", lte_cell_count(&lc));
		for (i = 0; i < lte_cell_count(&lc); i++) {
			const lte_cell_info_t *ci = lte_cell_get(&lc, i);
			printf(" [PCI %d MIB %d]", ci->pci, ci->mib_good);
			CHECK(!lte_cell_confirmed(ci), "noise: confirmed PCI %d", ci->pci);
		}
		printf("\n");
		lte_cell_exit(&lc);
		free(iq);
	}
	printf("selftest: %s (%d failures)\n", failures ? "FAILED" : "ok", failures);
}

/*
 * captures
 */

typedef struct capture {
	float complex	*x;
	int64_t		n;
	double		rate, center;
} capture_t;

static int load_capture(capture_t *cap, const char *file, const char *fmt, double skip, double secs)
{
	FILE *fp = fopen(file, "rb");
	int64_t bytes, ns, i, s0;
	int bps;

	if (!fp) {
		perror(file);
		return -1;
	}
	bps = !strcmp(fmt, "cf32") ? 8 : !strcmp(fmt, "cu8") ? 2 : 4;
	fseek(fp, 0, SEEK_END);
	bytes = ftell(fp);
	ns = bytes / bps;
	s0 = (int64_t)(skip * cap->rate);
	if (s0 > ns)
		s0 = ns;
	ns -= s0;
	if (secs > 0 && ns > (int64_t)(secs * cap->rate))
		ns = (int64_t)(secs * cap->rate);
	fseek(fp, s0 * bps, SEEK_SET);
	cap->x = malloc(ns * sizeof(float complex));
	cap->n = ns;
	if (!strcmp(fmt, "cf32")) {
		float *v = malloc(ns * 8);
		ns = fread(v, 8, ns, fp);
		for (i = 0; i < ns; i++)
			cap->x[i] = v[2 * i] + I * v[2 * i + 1];
		free(v);
	} else if (!strcmp(fmt, "cu8")) {
		uint8_t *v = malloc(ns * 2);
		ns = fread(v, 2, ns, fp);
		for (i = 0; i < ns; i++)
			cap->x[i] = (v[2 * i] - 127.4f) / 128.0f + I * (v[2 * i + 1] - 127.4f) / 128.0f;
		free(v);
	} else {
		int16_t *v = malloc(ns * 4);
		ns = fread(v, 4, ns, fp);
		for (i = 0; i < ns; i++)
			cap->x[i] = v[2 * i] / 32768.0f + I * v[2 * i + 1] / 32768.0f;
		free(v);
	}
	cap->n = ns;
	fclose(fp);
	return 0;
}

/* the carrier at 0 Hz and 1.92 MS/s, through the shared channelizer
 * (libchannelizer), as the SDR layer gives it to the module */
#define DDC_BLOCK 16384
static float *ddc(const capture_t *cap, double offset, int64_t *nout, double *rate_out)
{
	channelizer_t ch;
	sample_t *oi, *oq;
	float *out;
	int64_t n, o = 0, cap_out = (int64_t)((double)cap->n * LTE_RATE / cap->rate) + 64;
	int k;

	*nout = 0;
	*rate_out = LTE_RATE;
	if (channelizer_rx_init(&ch, (int)cap->rate, (int)LTE_RATE, offset, 1.2e6, DDC_BLOCK) < 0) {
		printf("channelizer: %s\n", channelizer_error());
		return NULL;
	}
	out = malloc(2 * cap_out * sizeof(float));
	oi = malloc(2 * DDC_BLOCK * sizeof(sample_t));
	oq = malloc(2 * DDC_BLOCK * sizeof(sample_t));
	for (n = 0; n < cap->n; n += DDC_BLOCK) {
		int num = (cap->n - n < DDC_BLOCK) ? (int)(cap->n - n) : DDC_BLOCK;
		int got = channelizer_rx_process(&ch, (const float *)(cap->x + n), num, 2 * DDC_BLOCK, oi, oq);
		for (k = 0; k < got && o < cap_out; k++, o++) {
			out[2 * o] = oi[k];
			out[2 * o + 1] = oq[k];
		}
	}
	channelizer_exit(&ch);
	free(oi);
	free(oq);
	*nout = o;
	return out;
}

static void run_earfcn(const capture_t *cap, int earfcn, double cfo_max, double ppm, int quiet)
{
	int band, i;
	double f = lte_earfcn_to_freq(earfcn, &band), rate, t_ddc, t_lte;
	int64_t n, pos;
	float *iq;
	lte_cell_t lc;
	int chunk;

	if (f == 0) {
		printf("EARFCN %d: unknown\n", earfcn);
		return;
	}
	t_ddc = cpu_now();
	iq = ddc(cap, f - cap->center, &n, &rate);
	t_ddc = cpu_now() - t_ddc;
	if (!iq)
		return;
	if (getenv("LTE_DUMP")) {
		FILE *fp = fopen(getenv("LTE_DUMP"), "wb");
		if (fp) {
			fwrite(iq, 8, n, fp);
			fclose(fp);
		}
	}
	t_lte = cpu_now();
	lte_cell_init(&lc, rate, f, ppm * 1e-6 * f, cfo_max);
	lc.debug = (getenv("LTE_DEBUG") != NULL);
	chunk = (int)(rate * 0.01);
	for (pos = 0; pos < n; pos += chunk) {
		int num = (n - pos < chunk) ? (int)(n - pos) : chunk;
		lte_cell_process(&lc, iq + 2 * pos, num);
	}
	t_lte = cpu_now() - t_lte;
	if (!quiet || lte_cell_count(&lc))
		printf("EARFCN %d (band %d, %.1f MHz): %d cell(s), %.2f s of signal, cpu %.3f s (%.1f%% of real time), ddc %.3f s\n",
		       earfcn, band, f / 1e6, lte_cell_count(&lc), n / rate, t_lte, 100.0 * t_lte / (n / rate), t_ddc);
	for (i = 0; i < lte_cell_count(&lc); i++) {
		const lte_cell_info_t *ci = lte_cell_get(&lc, i);
		printf("  PCI %3d %s %s CP  PSS %.3f SSS %.1f  frames %d%s\n", ci->pci, ci->tdd ? "TDD" : "FDD",
		       ci->ext_cp ? "ext" : "normal", ci->pss_metric, ci->sss_score, ci->frames, ci->lost ? " (lost)" : "");
		printf("    MIB good %d bad %d consistent %d inconsistent %d", ci->mib_good, ci->mib_bad,
		       ci->mib_consistent, ci->mib_inconsistent);
		if (ci->mib_good)
			printf("  (frame start %.2f us into the capture mod 10 ms: SFN %d at %.3f ms)\n   ",
			       fmod(ci->sfn_pos / 1.92, 10000.0), ci->sfn, ci->sfn_pos / 1920.0);
		if (ci->mib_good)
			printf("  ports %d BW %d RB PHICH %s %s SFN %d spare 0x%03x agree %.2f", ci->ports, ci->n_rb,
			       ci->phich_ext ? "extended" : "normal", lte_phich_res_name(ci->phich_res), ci->sfn,
			       ci->spare, ci->mib_agree);
		printf("\n");
		printf("    CFO %+.2f Hz (+-%.2f, %d, alias margin %.1f) = %+.4f ppm   timing %+.3f ppm (+-%.3f, %d)\n",
		       ci->cfo_hz, ci->cfo_std_hz, ci->cfo_points, ci->cfo_alias_margin, ci->cfo_ppm, ci->timing_ppm,
		       ci->timing_std_ppm, ci->timing_points);
		printf("    RSRP %.1f dBFS  RSSI %.1f dBFS  RSRQ %.1f dB  SINR %.1f dB  %s\n", ci->rsrp_dbfs,
		       ci->rssi_dbfs, ci->rsrq_db, ci->sinr_db, lte_cell_confirmed(ci) ? "CONFIRMED" : "not confirmed");
	}
	lte_cell_exit(&lc);
	free(iq);
}

int main(int argc, char *argv[])
{
	capture_t cap;
	const char *fmt = "sc16", *file;
	double cfo_max = 40000, ppm = 0, secs = 0, skip = 0.05;
	int earfcn[4096], ne = 0, quiet = 0, i, band = 0;

	setvbuf(stdout, NULL, _IOLBF, 0);
	if (argc < 2 || !strcmp(argv[1], "selftest")) {
		selftest();
		return failures ? 1 : 0;
	}
	if (strcmp(argv[1], "file") || argc < 3) {
		fprintf(stderr, "usage: %s selftest | file <capture> [-t fmt] -r rate -c centre -e earfcn[:last] "
			"[-b band] [-m cfo_max] [-p ppm] [-s sec] [-k sec] [-q]\n", argv[0]);
		return 2;
	}
	memset(&cap, 0, sizeof(cap));
	file = argv[2];
	for (i = 3; i < argc; i++) {
		const char *o = argv[i], *v = (i + 1 < argc) ? argv[i + 1] : "";
		if (!strcmp(o, "-q")) {
			quiet = 1;
			continue;
		}
		i++;
		if (!strcmp(o, "-t"))
			fmt = v;
		else if (!strcmp(o, "-r"))
			cap.rate = atof(v);
		else if (!strcmp(o, "-c"))
			cap.center = atof(v);
		else if (!strcmp(o, "-m"))
			cfo_max = atof(v);
		else if (!strcmp(o, "-p"))
			ppm = atof(v);
		else if (!strcmp(o, "-s"))
			secs = atof(v);
		else if (!strcmp(o, "-k"))
			skip = atof(v);
		else if (!strcmp(o, "-b"))
			band = atoi(v);
		else if (!strcmp(o, "-e")) {
			int a, b;
			if (sscanf(v, "%d:%d", &a, &b) == 2) {
				for (; a <= b && ne < 4096; a++)
					earfcn[ne++] = a;
			} else
				earfcn[ne++] = atoi(v);
		}
	}
	if (cap.rate <= 0 || cap.center <= 0) {
		fprintf(stderr, "need -r and -c\n");
		return 2;
	}
	if (band) {
		int first, last, e;
		lte_band_dl_range(band, &first, &last, NULL, NULL);
		for (e = first; e <= last && ne < 4096; e++) {
			double f = lte_earfcn_to_freq(e, NULL);
			/* the centre 6 RB (+-0.6 MHz) inside 80 % of the span */
			if (fabs(f - cap.center) + 0.6e6 < 0.4 * cap.rate)
				earfcn[ne++] = e;
		}
	}
	if (load_capture(&cap, file, fmt, skip, secs) < 0)
		return 1;
	printf("%s: %.3f s at %.3f MS/s, centre %.3f MHz, %d EARFCN(s)\n", file, cap.n / cap.rate, cap.rate / 1e6,
	       cap.center / 1e6, ne);
	for (i = 0; i < ne; i++)
		run_earfcn(&cap, earfcn[i], cfo_max, ppm, quiet);
	free(cap.x);
	return 0;
}
