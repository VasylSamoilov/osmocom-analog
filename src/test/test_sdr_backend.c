/* test of the SDR backend: programs (processes) register channels on one
 * engine over the socket and shared memory; null device, TX looped to RX,
 * in real time */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <sys/wait.h>
#include "../libsample/sample.h"
#include "../libsdr/sdr_engine.h"
#include "../libsdr/sdr_backend.h"
#include <complex.h>

#define FS	1920000
#define DSP	48000
#define NMAX	8
#define BLOCK	1920	/* 1 ms */

static char sock_name[64];
static int failures;
static int prog_dsp = DSP;	/* the DSP rate of the next program forked */

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

/*
 * a program: one RX and one TX channel at freq, sends a tone of tone_hz,
 * receives its channel for ms; exit status: 0 ok, 1 failed. mode:
 * 0 normal, 1 sleeps (to be killed), 2 writes a bad ring index once,
 * 3 waits until the backend is gone, 4 asks for a channel outside the band
 */
static int program(const char *name, double freq, double tone_hz, double other_hz, int ms, int mode)
{
	sdr_client_t c;
	char err[256];
	double complex *y = NULL;
	sample_t ti[1][2000], tq[1][2000], ri[1][2000], rq[1][2000];
	sample_t *tip[1] = { ti[0] }, *tqp[1] = { tq[0] }, *rip[1] = { ri[0] }, *rqp[1] = { rq[0] };
	int dsp = prog_dsp, cap = dsp * ms / 1000, n = 0, s, rc, tries, wrote_bad = 0;
	long t = 0;
	double start;

	usleep(200000);	/* the backend starts */
	for (tries = 0; tries < 20; tries++) {
		rc = sdr_client_open(&c, sock_name, name, dsp, 1, &freq, 1, &freq, 200, dsp / 50, err, sizeof(err));
		if (rc == 0 || mode == 4 || !strstr(err, "Connection refused"))
			break;
		usleep(50000);
	}
	if (mode == 4) {
		printf("  [%s] refused as it should: %s\n", name, (rc < 0) ? err : "(accepted!)");
		return (rc < 0) ? 0 : 1;
	}
	if (rc < 0) {
		printf("  [%s] %s\n", name, err);
		return 1;
	}
	if (mode == 3) {
		for (start = now(); now() - start < 5.0; usleep(10000))
			if (!sdr_client_alive(&c)) {
				printf("  [%s] noticed the backend is gone\n", name);
				sdr_client_close(&c);
				return 0;
			}
		printf("  [%s] backend still there after 5 s\n", name);
		return 1;
	}
	y = calloc(cap + 1, sizeof(*y));
	if (!y)
		return 1;
	start = now();
	while (n < cap && now() - start < ms / 1000.0 * 3) {
		int k = sdr_port_tosend(&c.port), got;

		if (k > 2000)
			k = 2000;
		for (s = 0; s < k; s++, t++) {
			double complex z = c.tx_scale * cexp(2 * M_PI * I * tone_hz / dsp * t);

			ti[0][s] = creal(z);
			tq[0][s] = cimag(z);
		}
		sdr_port_write(&c.port, tip, tqp, k);
		if (mode == 2 && n > cap / 4) {
			/* garbage in all four indices, every pass (in and out
			 * of range): the device threads must neither write nor
			 * read outside the rings, nobody else be affected */
			int rows = c.port.view->rows;

			c.port.view->rx_ctl.in = rand() % (4 * rows) - 2 * rows;
			c.port.view->rx_ctl.out = rand() % (4 * rows) - 2 * rows;
			c.port.view->tx_ctl.in = rand() % (4 * rows) - 2 * rows;
			c.port.view->tx_ctl.out = rand() % (4 * rows) - 2 * rows;
			wrote_bad = 1;
		}
		got = sdr_port_read(&c.port, rip, rqp, 2000);
		for (s = 0; s < got && n < cap; s++, n++)
			y[n] = (ri[0][s] + I * rq[0][s]) / c.tx_scale;
		if (mode == 1 && n > cap / 4)
			pause();	/* killed here */
		usleep(2000);
	}
	{
		int skip = n / 2, len = n - skip;
		double own = level(y + skip, len, tone_hz / dsp), other = level(y + skip, len, other_hz / dsp);
		long bad = (long)c.port.view->bad_index, missing = (long)c.port.view->tx_missing, dropped = (long)c.port.view->rx_dropped;
		/* a program that wrote a bad index loses its TX (its writes are
		 * refused too): only the counting is checked */
		int ok = wrote_bad ? (bad > 0) : ((n == cap) && fabs(own) < 0.05 && other < -100.0);

		printf("  [%s] %d rows, own tone %.3f dB, the other's %.1f dB, missing %ld, dropped %ld, bad index %ld: %s\n",
		       name, n, own, other, missing, dropped, bad, ok ? "ok" : "FAILED");
		free(y);
		sdr_client_close(&c);
		return ok ? 0 : 1;
	}
}

/* the device: TX looped to RX, 1 ms blocks in real time */
static volatile int device_run = 1;
static sdr_engine_t engine;

static double block_max;	/* longest block (TX + RX), s */

static void *device(void __attribute__((unused)) *arg)
{
	float *iq = malloc(sizeof(float) * 2 * BLOCK);
	double next = now();

	while (device_run) {
		double t0 = now(), d;

		sdr_engine_tx_produce(&engine, iq, BLOCK);
		sdr_engine_rx_feed(&engine, iq, BLOCK);
		d = now() - t0;
		if (d > block_max)
			block_max = d;
		next += (double)BLOCK / FS;
		while (now() < next)
			usleep(200);
	}
	free(iq);
	return NULL;
}

static pid_t spawn(const char *name, double freq, double tone, double other, int ms, int mode)
{
	pid_t p = fork();

	if (p == 0) {
		int rc = program(name, freq, tone, other, ms, mode);

		fflush(stdout);
		_exit(rc);
	}
	return p;
}

static int status_of(pid_t p)
{
	int st = 0;

	waitpid(p, &st, 0);
	return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
}

int main(void)
{
	sdr_backend_t *b;
	pthread_t tid;
	pid_t pa, pb, pk, pbad, pref, phalf, pgone;
	int i;

	/* nothing buffered when the programs fork */
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("=== SDR backend tests ===\n\n");
	snprintf(sock_name, sizeof(sock_name), "@test_sdr_backend_%d", (int)getpid());

	/* the programs fork before any thread runs */
	printf("Two programs at -300 / +250 kHz, one killed, one writing a bad index, one refused (%d S/s, DSP %d):\n", FS, DSP);
	pa = spawn("A", -300000.0, 1000.0, -2000.0, 600, 0);
	pb = spawn("B", 250000.0, -2000.0, 1000.0, 600, 0);
	pk = spawn("K", 25000.0, 3000.0, 1000.0, 600, 1);
	pbad = spawn("X", -100000.0, 500.0, 1000.0, 600, 2);
	pref = spawn("R", 5e6, 0.0, 0.0, 100, 4);
	/* a program at half the DSP rate on the same engine */
	prog_dsp = DSP / 2;
	phalf = spawn("H", 500000.0, 700.0, -2000.0, 600, 0);
	prog_dsp = DSP;

	if (sdr_engine_init(&engine, FS, FS, DSP, NMAX, BLOCK) < 0) {
		printf("  %s\n", sdr_engine_error());
		return 1;
	}
	b = sdr_backend_start(&engine, sock_name, 0.0, 0.0);
	if (!b) {
		printf("  %s\n", sdr_backend_error());
		return 1;
	}
	pthread_create(&tid, NULL, device, NULL);

	/* K is killed while A and B run */
	usleep(500000);
	kill(pk, SIGKILL);
	check(status_of(pa) == 0, "A: own channel back, nothing of the others (see above)", 0, "");
	check(status_of(pb) == 0, "B: the same", 0, "");
	check(status_of(pk) == -1, "K: killed", 0, "");
	check(status_of(pbad) == 0, "X: wrote a bad index, counted by the device threads", 0, "");
	check(status_of(pref) == 0, "R: outside the band, refused with the reason", 0, "");
	check(status_of(phalf) == 0, "H: at half the DSP rate on the same engine, own channel back", 0, "");
	for (i = 0; i < 300 && sdr_backend_groups(b); i++)
		usleep(10000);
	check(sdr_backend_groups(b) == 0, "groups left after all programs ended (K removed)", sdr_backend_groups(b), "");
	check(engine.tx_used == 0, "TX channels in use after all ended", engine.tx_used, "");
	/* programs join and leave while the device runs: no block may take
	 * long (the device would fall behind, UHD time stamps late) */
	check(block_max < 0.002, "longest device block (1 ms of samples) while programs joined and left", block_max * 1e3, "ms");

	/* the backend goes while a program is registered */
	printf("Backend stops while a program is registered:\n");
	device_run = 0;
	pthread_join(tid, NULL);
	sdr_backend_stop(b);
	sdr_engine_exit(&engine);
	/* a fresh backend, before threads: fork first again */
	pgone = spawn("G", 0.0, 0.0, 0.0, 0, 3);
	if (sdr_engine_init(&engine, FS, FS, DSP, NMAX, BLOCK) < 0)
		return 1;
	b = sdr_backend_start(&engine, sock_name, 0.0, 0.0);
	device_run = 1;
	pthread_create(&tid, NULL, device, NULL);
	for (i = 0; i < 300 && !sdr_backend_groups(b); i++)
		usleep(10000);
	sdr_backend_stop(b);
	check(status_of(pgone) == 0, "G: noticed the backend is gone", 0, "");
	device_run = 0;
	pthread_join(tid, NULL);
	sdr_engine_exit(&engine);

	printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
