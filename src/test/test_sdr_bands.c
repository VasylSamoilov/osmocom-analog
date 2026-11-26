/* test of the SDR bands (sdr_bands.c): four bands of one program at
 * different IQ rates (AM and NFM on the filter bank, FM mono 240 k and FM
 * stereo + RDS 480 k on the per-channel chain), serviced by one loop in
 * turn as osmoradio does; the same body on
 * the engine in-process and on the backend (one registration per band).
 * Null device, TX looped to RX, in real time. */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include "../libsample/sample.h"
#include "../libsdr/sdr_engine.h"
#include "../libsdr/sdr_backend.h"
#include "../libsdr/sdr_bands.h"
#include <complex.h>

#define FS	1920000
#define DSP	48000	/* the filter bank's DSP rate */
#define NB	4	/* bands */
#define NMAX	NB	/* TX channels: the bands */
#define BLOCK	1920	/* 1 ms */
#define MS	800	/* received per band */

static int failures;

static void check(int ok, const char *what, double value, const char *unit)
{
	printf("  %-66s %10.3f %-3s %s\n", what, value, unit, ok ? "ok" : "FAILED");
	if (!ok)
		failures++;
}

static double now(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

/* level (dB) of the tone at f (cycles per sample) in y */
static double level(const double complex *y, int n, double f)
{
	double complex a = 0;
	int i;

	for (i = 0; i < n; i++)
		a += y[i] * cexp(-2 * M_PI * I * f * i);
	return 20 * log10(cabs(a) / n + 1e-300);
}

/* the device: TX looped to RX, 1 ms blocks in real time */
static volatile int device_run;
static sdr_engine_t engine;
static volatile double device_late;	/* the device's block was due this long ago (most) */

static void *device(void __attribute__((unused)) *arg)
{
	float *iq = malloc(sizeof(float) * 2 * BLOCK);
	double next = now();

	while (device_run) {
		if (now() - next > device_late)
			device_late = now() - next;
		sdr_engine_tx_produce(&engine, iq, BLOCK);
		sdr_engine_rx_feed(&engine, iq, BLOCK);
		next += (double)BLOCK / FS;
		while (now() < next)
			usleep(200);
	}
	free(iq);
	return NULL;
}

/* the three bands: offset from the centre, IQ rate, the tone each sends */
static const struct {
	const char *name;
	double freq;
	int rate;
	double tone;
} B[NB] = {
	{ "AM 48k", -300000.0, 48000, 1000.0 },
	{ "NFM 24k", 250000.0, 24000, -2000.0 },
	{ "FM 240k", 600000.0, 240000, 30000.0 },
	{ "FM 480k", -650000.0, 480000, 100000.0 },	/* stereo + RDS */
};

/* the program: the three bands, one loop over them, each sends its tone
 * and receives MS of its band */
static void run(sdr_bands_t *bs, const char *how)
{
	double complex *y[NB];
	float *buf;
	int n[NB] = { 0 }, cap[NB], idx[NB], b, s, rc, bufmax = 0, more;
	long t[NB] = { 0 }, missing[NB], dropped[NB];
	char err[256];
	double start, pass, longest = 0.0;

	printf("%s: %d bands, one loop:\n", how, NB);
	for (b = 0; b < NB; b++) {
		sdr_band_def_t def = { B[b].name, 1, 1, B[b].freq, B[b].freq, B[b].rate, B[b].rate / 50 };	/* 20 ms */

		idx[b] = sdr_band_add(bs, &def, err, sizeof(err));
		check(idx[b] == b, B[b].name, idx[b], "idx");
		if (idx[b] < 0) {
			printf("  %s\n", err);
			return;
		}
		cap[b] = B[b].rate * MS / 1000;
		y[b] = calloc(cap[b], sizeof(**y));
		if (def.buffer_size > bufmax)
			bufmax = def.buffer_size;
	}
	buf = malloc(sizeof(float) * 2 * bufmax);
	start = pass = now();
	for (more = 1; more && now() - start < MS / 1000.0 * 3; ) {
		for (more = 0, b = 0; b < NB; b++)
			if (n[b] < cap[b])
				more = 1;
		for (b = 0; b < NB; b++) {
			int k = sdr_band_tosend(bs, b), got;

			for (s = 0; s < k; s++, t[b]++) {
				double complex z = cexp(2 * M_PI * I * B[b].tone / B[b].rate * t[b]);

				buf[2 * s] = creal(z);
				buf[2 * s + 1] = cimag(z);
			}
			rc = sdr_band_write(bs, b, buf, k);
			if (rc != k)
				printf("  %s: %d of %d written\n", B[b].name, rc, k);
			got = sdr_band_read(bs, b, buf, bufmax < B[b].rate / 50 ? bufmax : B[b].rate / 50);
			for (s = 0; s < got && n[b] < cap[b]; s++, n[b]++)
				y[b][n[b]] = (buf[2 * s] + I * buf[2 * s + 1]) / sdr_band_tx_scale(bs, b);
		}
		usleep(2000);
		if (now() - pass > longest)
			longest = now() - pass;
		pass = now();
	}
	/* the counters now: TX runs dry while the levels are worked out */
	for (b = 0; b < NB; b++) {
		missing[b] = bs->band[b].port->view->tx_missing;
		dropped[b] = bs->band[b].port->view->rx_dropped;
	}
	printf("  the longest pass of the loop: %.1f ms, the device at most %.1f ms late\n", longest * 1e3, device_late * 1e3);
	device_late = 0.0;
	for (b = 0; b < NB; b++) {
		int skip = n[b] / 2, len = n[b] - skip, o;
		double own = level(y[b] + skip, len, B[b].tone / B[b].rate), mirror = level(y[b] + skip, len, -B[b].tone / B[b].rate);
		double other = -300.0;
		char what[100];

		/* the others' tones, if they fall in this band's rate */
		for (o = 0; o < NB; o++)
			if (o != b && fabs(B[o].tone) < B[b].rate / 2.0) {
				double l = level(y[b] + skip, len, B[o].tone / B[b].rate);

				if (l > other)
					other = l;
			}
		snprintf(what, sizeof(what), "%s: %d rows of %d at %d S/s", B[b].name, n[b], cap[b], B[b].rate);
		check(n[b] == cap[b], what, n[b], "");
		snprintf(what, sizeof(what), "%s: own tone (%+.0f Hz)", B[b].name, B[b].tone);
		check(fabs(own) < 0.05, what, own, "dB");
		snprintf(what, sizeof(what), "%s: its mirror (I / Q order)", B[b].name);
		check(mirror < -100.0, what, mirror, "dB");
		snprintf(what, sizeof(what), "%s: the other bands' tones", B[b].name);
		check(other < -100.0, what, other, "dB");
		snprintf(what, sizeof(what), "%s: TX rows missing", B[b].name);
		check(missing[b] == 0, what, (double)missing[b], "");
		snprintf(what, sizeof(what), "%s: RX rows dropped", B[b].name);
		check(dropped[b] == 0, what, (double)dropped[b], "");
		free(y[b]);
	}
	free(buf);
}

/* refusals on an engine that is not running (nothing adopted) */
static void refusals(void)
{
	sdr_engine_t e;
	sdr_bands_t bs;
	sdr_band_def_t def;
	char err[256];
	int i, rc;

	printf("Refusals:\n");
	if (sdr_engine_init(&e, FS, FS, DSP, 2, BLOCK) < 0) {
		printf("  %s\n", sdr_engine_error());
		failures++;
		return;
	}
	sdr_bands_engine(&bs, &e, 0.0, 0.0);
	def = (sdr_band_def_t){ "outside", 1, 1, 5e6, 5e6, DSP, DSP / 50 };
	rc = sdr_band_add(&bs, &def, err, sizeof(err));
	printf("    %s\n", err);
	check(rc < 0, "a band outside the SDR's band refused", rc, "");
	def = (sdr_band_def_t){ "none", 0, 0, 0.0, 0.0, DSP, DSP / 50 };
	rc = sdr_band_add(&bs, &def, err, sizeof(err));
	check(rc < 0, "a band without direction refused", rc, "");
	/* TX: the engine was made for 2 */
	for (i = 0; i < 3; i++) {
		def = (sdr_band_def_t){ "tx", 0, 1, 0.0, -400000.0 + i * 200000.0, DSP, DSP / 50 };
		rc = sdr_band_add(&bs, &def, err, sizeof(err));
	}
	printf("    %s\n", err);
	check(rc < 0 && bs.n == 2, "the third TX band on an engine for 2 refused", bs.n, "bands");
	/* chain: RX only, at most SDR_ENGINE_MAX_CHAIN */
	for (i = 0; i <= SDR_ENGINE_MAX_CHAIN; i++) {
		def = (sdr_band_def_t){ "chain", 1, 0, -700000.0 + i * 300000.0, 0.0, 240000, 4800 };
		rc = sdr_band_add(&bs, &def, err, sizeof(err));
	}
	printf("    %s\n", err);
	check(rc < 0 && bs.n == 2 + SDR_ENGINE_MAX_CHAIN, "a chain band over the cap refused", bs.n, "bands");
	def = (sdr_band_def_t){ "big call", 1, 0, 0.0, 0.0, DSP, 100 };
	rc = sdr_band_add(&bs, &def, err, sizeof(err));
	check(sdr_band_read(&bs, rc, (float[2 * 101]){ 0 }, 101) == -EINVAL, "a read above the band's buffer size refused", 101, "");
	check(sdr_band_read(&bs, 99, (float[2]){ 0 }, 1) == -EINVAL, "a read of a band that is not there refused", 99, "");
	sdr_bands_close(&bs);
	check(e.tx_used == 0 && e.rx_chain == 0, "nothing left on the engine after close", e.tx_used + e.rx_chain, "");
	sdr_engine_exit(&e);
}

static int groups_left(void)
{
	int i, n = 0;

	for (i = 0; i < SDR_ENGINE_MAX_GROUPS; i++)
		if (engine.groups[i])
			n++;
	return n;
}

int main(void)
{
	static sdr_bands_t bs;
	sdr_backend_t *be;
	pthread_t tid;
	char sock[64];
	int i, rc;

	setvbuf(stdout, NULL, _IONBF, 0);
	printf("=== SDR bands tests (%d S/s, bank DSP %d) ===\n\n", FS, DSP);

	refusals();

	if (sdr_engine_init(&engine, FS, FS, DSP, NMAX, BLOCK) < 0) {
		printf("  %s\n", sdr_engine_error());
		return 1;
	}
	device_run = 1;
	pthread_create(&tid, NULL, device, NULL);

	/* in-process: on the engine */
	sdr_bands_engine(&bs, &engine, 0.0, 0.0);
	run(&bs, "In-process (engine)");
	sdr_bands_close(&bs);
	check(groups_left() == 0 && engine.tx_used == 0, "in-process: nothing left on the engine after close", groups_left(), "groups");

	/* the same on the backend: one registration per band */
	snprintf(sock, sizeof(sock), "@test_sdr_bands_%d", (int)getpid());
	be = sdr_backend_start(&engine, sock, 0.0, 0.0);
	if (!be) {
		printf("  %s\n", sdr_backend_error());
		return 1;
	}
	sdr_bands_backend(&bs, sock);
	run(&bs, "On the backend (one registration per band)");
	check(sdr_backend_groups(be) == NB, "backend: a registration per band", sdr_backend_groups(be), "");
	sdr_bands_close(&bs);
	for (i = 0; i < 300 && sdr_backend_groups(be); i++)
		usleep(10000);
	check(sdr_backend_groups(be) == 0 && groups_left() == 0, "backend: nothing left after close", sdr_backend_groups(be), "groups");

	/* the backend goes while bands are registered: a read says so */
	printf("Backend stops while bands are registered:\n");
	{
		sdr_band_def_t def = { "gone", 1, 1, -300000.0, -300000.0, DSP, DSP / 50 };
		char err[256];
		float iq[2 * 960];
		double start;

		sdr_bands_backend(&bs, sock);
		rc = sdr_band_add(&bs, &def, err, sizeof(err));
		check(rc == 0, "a band registered", rc, "");
		sdr_backend_stop(be);
		for (start = now(); now() - start < 3.0; usleep(10000))
			if ((rc = sdr_band_read(&bs, 0, iq, 960)) == -EPERM)
				break;
		check(rc == -EPERM, "read: the backend is gone (-EPERM)", rc, "");
		sdr_bands_close(&bs);
	}

	device_run = 0;
	pthread_join(tid, NULL);
	sdr_engine_exit(&engine);

	printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
