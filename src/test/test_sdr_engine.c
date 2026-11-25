/* Tests of the SDR engine (libsdr/sdr_engine.c) with a null device: the TX
 * output is looped to the RX input at the SDR rate. See docs/SDR-BACKEND.md.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <complex.h>
#include <pthread.h>
#include "../libsample/sample.h"
#include "../libsdr/sdr_engine.h"

static int failures = 0;

static void check(int ok, const char *what, double value, const char *unit)
{
	printf("  %-68s %10.3f %-3s %s\n", what, value, unit, ok ? "ok" : "FAIL");
	if (!ok)
		failures++;
}

static double level(const double complex *x, int n, double f)
{
	double complex acc = 0;
	double wsum = 0;
	int k;

	for (k = 0; k < n; k++) {
		double w = 0.5 - 0.5 * cos(2 * M_PI * k / (n - 1));

		acc += w * x[k] * cexp(-2 * M_PI * I * f * k);
		wsum += w;
	}
	return 20 * log10(cabs(acc) / wsum + 1e-300);
}

#define FS	1920000
#define DSP	48000
#define BLOCK	1920	/* 1 ms */
#define NMAX	8

/* removing a group waits for the device threads; in this single threaded
 * test the remove runs in a thread of its own while blocks go on */
typedef struct { sdr_engine_t *e; sdr_group_t *g; } remove_arg_t;

static void *remove_thread(void *arg)
{
	remove_arg_t *a = arg;

	sdr_engine_group_remove(a->e, a->g);
	return NULL;
}

/* 1. one program, 2 channels, TX looped to RX */
static void test_loopback(void)
{
	sdr_engine_t e;
	sdr_group_t *g;
	double off[2] = { -25000.0, 25000.0 }, scale;
	int blocks = 500, b, s, nrx = 0, cap = DSP;
	float *iq = malloc(sizeof(float) * 2 * BLOCK);
	sample_t ti[2][2000], tq[2][2000], ri[2][2000], rq[2][2000];
	sample_t *tip[2] = { ti[0], ti[1] }, *tqp[2] = { tq[0], tq[1] }, *rip[2] = { ri[0], ri[1] }, *rqp[2] = { rq[0], rq[1] };
	double complex *y0 = malloc(sizeof(*y0) * cap), *y1 = malloc(sizeof(*y1) * cap);
	long t = 0;

	printf("Engine %d S/s, DSP %d, one group of 2 RX + 2 TX channels, TX looped to RX:\n", FS, DSP);
	if (sdr_engine_init(&e, FS, FS, DSP, NMAX, BLOCK) < 0) {
		printf("  %s\n", sdr_engine_error());
		failures++;
		return;
	}
	printf("  RX bank %d sub-bands, TX bank %d sub-bands, TX step %d\n", e.bank_rx.m, e.bank_tx.m, sdr_engine_tx_step(&e));
	scale = sdr_engine_tx_scale(&e);
	g = sdr_engine_group_add(&e, DSP, 2, off, 2, off, 200, DSP / 50);	/* ring 200 ms, keep 20 ms */
	if (!g) {
		printf("  %s\n", sdr_engine_error());
		failures++;
		return;
	}
	for (b = 0; b < blocks; b++) {
		int n = sdr_engine_group_tosend(g), got;

		if (n > 2000)
			n = 2000;
		for (s = 0; s < n; s++, t++) {
			double complex z0 = scale * cexp(2 * M_PI * I * 1000.0 / DSP * t), z1 = scale * cexp(-2 * M_PI * I * 2000.0 / DSP * t);

			ti[0][s] = creal(z0); tq[0][s] = cimag(z0);
			ti[1][s] = creal(z1); tq[1][s] = cimag(z1);
		}
		sdr_engine_group_write(g, tip, tqp, n);
		sdr_engine_tx_produce(&e, iq, BLOCK);
		sdr_engine_rx_feed(&e, iq, BLOCK);
		got = sdr_engine_group_read(g, rip, rqp, 2000);
		for (s = 0; s < got && nrx < cap; s++, nrx++) {
			y0[nrx] = (ri[0][s] + I * rq[0][s]) / scale;
			y1[nrx] = (ri[1][s] + I * rq[1][s]) / scale;
		}
	}
	{
		int skip = nrx / 4, len = nrx - skip;
		double l0 = level(y0 + skip, len, 1000.0 / DSP), l1 = level(y1 + skip, len, -2000.0 / DSP);
		double x0 = level(y0 + skip, len, -2000.0 / DSP), x1 = level(y1 + skip, len, 1000.0 / DSP);

		check(fabs(l0) < 0.01 && fabs(l1) < 0.01, "level of each channel's tone back (worst error)", fmax(fabs(l0), fabs(l1)), "dB");
		check(fmax(x0, x1) < -100.0, "the other channel's tone in a channel", fmax(x0, x1), "dB");
		check(g->view->tx_missing == 0 && g->view->rx_dropped == 0, "TX rows missing + RX rows dropped", (double)(g->view->tx_missing + g->view->rx_dropped), "");
		printf("  %ld rows sent, %ld received, TX scale 1/%d\n", (long)g->view->tx_rows, (long)g->view->rx_rows, NMAX);
	}
	sdr_engine_exit(&e);
	free(iq);
	free(y0);
	free(y1);
}

/* 2. RX isolation: group A's samples are the same whether B joins and
 * leaves during the run */
static int rx_run(int with_b, sample_t *out_a, int cap)
{
	sdr_engine_t e;
	sdr_group_t *a, *bg = NULL;
	double offa[1] = { 123456.0 }, offb[2] = { -300000.0, 123456.0 + 25000.0 };
	float *iq = malloc(sizeof(float) * 2 * BLOCK);
	sample_t ri[2][2000], rq[2][2000];
	sample_t *rip[2] = { ri[0], ri[1] }, *rqp[2] = { rq[0], rq[1] };
	int b, s, n = 0;
	long t = 0;
	pthread_t tid;
	remove_arg_t ra;

	sdr_engine_init(&e, FS, 0, DSP, NMAX, BLOCK);
	a = sdr_engine_group_add(&e, DSP, 1, offa, 0, NULL, 200, 0);
	srand(5);
	for (b = 0; b < 400; b++) {
		for (s = 0; s < BLOCK; s++, t++) {
			double complex z = 0.2 * cexp(2 * M_PI * I * (123456.0 + 700.0) / FS * t) + 0.2 * cexp(2 * M_PI * I * -300000.0 / FS * t);

			iq[2 * s] = creal(z) + 0.01 * (rand() / (double)RAND_MAX - 0.5);
			iq[2 * s + 1] = cimag(z) + 0.01 * (rand() / (double)RAND_MAX - 0.5);
		}
		if (with_b && b == 100)
			bg = sdr_engine_group_add(&e, DSP, 2, offb, 0, NULL, 200, 0);
		if (with_b && b == 300) {
			ra.e = &e;
			ra.g = bg;
			pthread_create(&tid, NULL, remove_thread, &ra);
		}
		sdr_engine_rx_feed(&e, iq, BLOCK);
		if (with_b && bg && b >= 100 && b < 300)
			sdr_engine_group_read(bg, rip, rqp, 2000);
		int got = sdr_engine_group_read(a, rip, rqp, 2000);
		for (s = 0; s < got && n < cap; s++, n++) {
			out_a[2 * n] = ri[0][s];
			out_a[2 * n + 1] = rq[0][s];
		}
		if (with_b && b == 305)
			pthread_join(tid, NULL);
	}
	sdr_engine_exit(&e);
	free(iq);
	return n;
}

static void test_rx_isolation(void)
{
	int cap = 400 * DSP / 1000 + 100, n1, n2, s;
	sample_t *a1 = malloc(sizeof(sample_t) * 2 * cap), *a2 = malloc(sizeof(sample_t) * 2 * cap);
	double worst = 0;

	printf("RX isolation: group A alone, then with group B joining at 100 ms and leaving at 300 ms:\n");
	n1 = rx_run(0, a1, cap);
	n2 = rx_run(1, a2, cap);
	for (s = 0; s < 2 * n1 && s < 2 * n2; s++)
		worst = fmax(worst, fabs(a1[s] - a2[s]));
	check(n1 == n2 && worst == 0.0, "A's samples: count equal, largest difference", worst, "");
}

/* 3. TX isolation: A transmits a tone; B joins, stalls (writes nothing),
 * transmits, leaves. A's channel received by C must stay the same. */
static int tx_run(int with_b, double complex *out_c, int cap, long *b_missing, long *a_missing)
{
	sdr_engine_t e;
	sdr_group_t *a, *bg = NULL, *c;
	double offa[1] = { 200000.0 }, offb[1] = { -400000.0 }, scale;
	float *iq = malloc(sizeof(float) * 2 * BLOCK);
	sample_t ti[1][2000], tq[1][2000], ri[1][2000], rq[1][2000];
	sample_t *tip[1] = { ti[0] }, *tqp[1] = { tq[0] }, *rip[1] = { ri[0] }, *rqp[1] = { rq[0] };
	int b, s, n = 0;
	long ta = 0, tb = 0;
	pthread_t tid;
	remove_arg_t ra;

	sdr_engine_init(&e, FS, FS, DSP, NMAX, BLOCK);
	scale = sdr_engine_tx_scale(&e);
	a = sdr_engine_group_add(&e, DSP, 0, NULL, 1, offa, 200, DSP / 50);
	c = sdr_engine_group_add(&e, DSP, 1, offa, 0, NULL, 200, 0);
	for (b = 0; b < 400; b++) {
		int k = sdr_engine_group_tosend(a);

		for (s = 0; s < k && s < 2000; s++, ta++) {
			double complex z = scale * cexp(2 * M_PI * I * 1500.0 / DSP * ta);

			ti[0][s] = creal(z);
			tq[0][s] = cimag(z);
		}
		sdr_engine_group_write(a, tip, tqp, s);
		if (with_b && b == 100)
			bg = sdr_engine_group_add(&e, DSP, 0, NULL, 1, offb, 200, DSP / 50);
		if (with_b && bg && b < 300 && (b < 150 || b >= 250)) {
			/* B transmits 100..150 ms, stalls (writes nothing) 150..250 ms,
			 * transmits again until it leaves at 300 ms; a group that has
			 * not written yet is not counted as missing (start) */
			k = sdr_engine_group_tosend(bg);
			for (s = 0; s < k && s < 2000; s++, tb++) {
				double complex z = scale * cexp(2 * M_PI * I * -3000.0 / DSP * tb);

				ti[0][s] = creal(z);
				tq[0][s] = cimag(z);
			}
			sdr_engine_group_write(bg, tip, tqp, s);
		}
		if (with_b && b == 300) {
			*b_missing = bg->view->tx_missing;
			ra.e = &e;
			ra.g = bg;
			pthread_create(&tid, NULL, remove_thread, &ra);
		}
		sdr_engine_tx_produce(&e, iq, BLOCK);
		sdr_engine_rx_feed(&e, iq, BLOCK);
		int got = sdr_engine_group_read(c, rip, rqp, 2000);
		for (s = 0; s < got && n < cap; s++, n++)
			out_c[n] = ri[0][s] + I * rq[0][s];
		if (with_b && b == 305)
			pthread_join(tid, NULL);
	}
	*a_missing = a->view->tx_missing;
	sdr_engine_exit(&e);
	free(iq);
	return n;
}

static void test_tx_isolation(void)
{
	int cap = 400 * DSP / 1000 + 100, n1, n2, s, skip;
	double complex *c1 = malloc(sizeof(*c1) * cap), *c2 = malloc(sizeof(*c2) * cap);
	double worst = 0, ref = 0;
	long bm = 0, am1 = 0, am2 = 0;

	printf("TX isolation: A transmits; B joins at 100 ms, stalls 150..250 ms, leaves at 300 ms:\n");
	n1 = tx_run(0, c1, cap, &bm, &am1);
	n2 = tx_run(1, c2, cap, &bm, &am2);
	skip = DSP / 20;
	for (s = skip; s < n1 && s < n2; s++) {
		worst = fmax(worst, cabs(c1[s] - c2[s]));
		ref = fmax(ref, cabs(c1[s]));
	}
	check(n1 == n2 && 20 * log10(worst / ref + 1e-300) < -100.0, "A's channel received: difference with B (re A's level)", 20 * log10(worst / ref + 1e-300), "dB");
	check(am1 == 0 && am2 == 0, "A's TX rows missing", (double)(am1 + am2), "");
	check(bm > 0, "B's missing rows while it stalled (counted on B only)", (double)bm, "");
}

/* 4. a group above the bank's DSP rate (broadcast FM next to NFM): the
 * per-channel chain beside the bank, TX looped to RX */
#define WIDE	240000
static void test_wide(void)
{
	sdr_engine_t e;
	sdr_group_t *gn, *gw;
	double offn[2] = { -400000.0, -375000.0 }, offw = 400000.0, scale;
	int blocks = 500, b, s, nn = 0, nw = 0, capn = DSP, capw = WIDE;
	float *iq = malloc(sizeof(float) * 2 * BLOCK);
	static sample_t ti[2][8000], tq[2][8000], ri[2][8000], rq[2][8000];
	sample_t *tip[2] = { ti[0], ti[1] }, *tqp[2] = { tq[0], tq[1] }, *rip[2] = { ri[0], ri[1] }, *rqp[2] = { rq[0], rq[1] };
	double complex *y0 = malloc(sizeof(*y0) * capn), *y1 = malloc(sizeof(*y1) * capn), *yw = malloc(sizeof(*yw) * capw);
	long tn = 0, tw = 0;

	printf("Engine %d S/s, bank for DSP %d, an NFM group (2 ch, %d) and a wide group (1 ch, %d, chain), TX looped to RX:\n", FS, DSP, DSP, WIDE);
	if (sdr_engine_init(&e, FS, FS, DSP, NMAX, BLOCK) < 0) {
		printf("  %s\n", sdr_engine_error());
		failures++;
		return;
	}
	scale = sdr_engine_tx_scale(&e);
	gn = sdr_engine_group_add(&e, DSP, 2, offn, 2, offn, 200, DSP / 50);
	gw = sdr_engine_group_add(&e, WIDE, 1, &offw, 1, &offw, 200, WIDE / 50);
	if (!gn || !gw) {
		printf("  %s\n", sdr_engine_error());
		failures++;
		return;
	}
	check(gw->chain && !gn->chain, "the wide group on the chain, the NFM group on the bank", (double)gw->chain, "");
	for (b = 0; b < blocks; b++) {
		int n = sdr_engine_group_tosend(gn), got;

		if (n > 8000)
			n = 8000;
		for (s = 0; s < n; s++, tn++) {
			double complex z0 = scale * cexp(2 * M_PI * I * 1000.0 / DSP * tn), z1 = scale * cexp(-2 * M_PI * I * 2000.0 / DSP * tn);

			ti[0][s] = creal(z0); tq[0][s] = cimag(z0);
			ti[1][s] = creal(z1); tq[1][s] = cimag(z1);
		}
		sdr_engine_group_write(gn, tip, tqp, n);
		n = sdr_engine_group_tosend(gw);
		if (n > 8000)
			n = 8000;
		for (s = 0; s < n; s++, tw++) {
			double complex z = scale * cexp(2 * M_PI * I * 20000.0 / WIDE * tw);

			ti[0][s] = creal(z); tq[0][s] = cimag(z);
		}
		sdr_engine_group_write(gw, tip, tqp, n);
		sdr_engine_tx_produce(&e, iq, BLOCK);
		sdr_engine_rx_feed(&e, iq, BLOCK);
		got = sdr_engine_group_read(gn, rip, rqp, 8000);
		for (s = 0; s < got && nn < capn; s++, nn++) {
			y0[nn] = (ri[0][s] + I * rq[0][s]) / scale;
			y1[nn] = (ri[1][s] + I * rq[1][s]) / scale;
		}
		got = sdr_engine_group_read(gw, rip, rqp, 8000);
		for (s = 0; s < got && nw < capw; s++, nw++)
			yw[nw] = (ri[0][s] + I * rq[0][s]) / scale;
	}
	{
		int skn = nn / 4, ln = nn - skn, skw = nw / 4, lw = nw - skw;
		double l0 = level(y0 + skn, ln, 1000.0 / DSP), l1 = level(y1 + skn, ln, -2000.0 / DSP), lw0 = level(yw + skw, lw, 20000.0 / WIDE);
		double xw = fmax(level(y0 + skn, ln, (offw + 20000.0 - offn[0]) / DSP), level(y1 + skn, ln, (offw + 20000.0 - offn[1]) / DSP));
		double xn = fmax(level(yw + skw, lw, (offn[0] + 1000.0 - offw) / WIDE), level(yw + skw, lw, (offn[1] - 2000.0 - offw) / WIDE));
		long expect = (long)blocks * BLOCK / (FS / WIDE);

		check(fabs(l0) < 0.01 && fabs(l1) < 0.01, "level of each NFM channel's tone back (worst error)", fmax(fabs(l0), fabs(l1)), "dB");
		check(fabs(lw0) < 0.05, "level of the wide channel's tone back (error)", fabs(lw0), "dB");
		check(xw < -100.0 && xn < -100.0, "the other group's tones in a channel (worst)", fmax(xw, xn), "dB");
		check(gn->view->tx_missing == 0 && gn->view->rx_dropped == 0 && gw->view->tx_missing == 0 && gw->view->rx_dropped == 0,
		      "TX rows missing + RX rows dropped, both groups", (double)(gn->view->tx_missing + gn->view->rx_dropped + gw->view->tx_missing + gw->view->rx_dropped), "");
		check(labs((long)gw->view->rx_rows - expect) <= 8 && labs((long)gw->view->tx_rows - expect) <= 8 + WIDE / 50,
		      "wide rows received / sent vs the rate (worst difference)", (double)labs((long)gw->view->rx_rows - expect), "");
		printf("  wide: %ld rows sent, %ld received (%ld expected); NFM: %ld / %ld\n", (long)gw->view->tx_rows, (long)gw->view->rx_rows, expect,
		       (long)gn->view->tx_rows, (long)gn->view->rx_rows);
	}
	sdr_engine_exit(&e);
	free(iq);
	free(y0);
	free(y1);
	free(yw);
}

int main(void)
{
	printf("=== SDR engine tests ===\n\n");
	test_loopback();
	test_rx_isolation();
	test_tx_isolation();
	test_wide();
	printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
