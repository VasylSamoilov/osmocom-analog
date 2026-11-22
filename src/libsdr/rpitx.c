/* rpitx backend - Raspberry Pi GPIO RF transmitter via librpitx
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * rpitx is not an SDR. It moves the frequency of one square wave clock on
 * GPIO4 once per sample, and it can switch the pin off or set one of 8 drive
 * levels. It cannot make a linear IQ signal.
 *
 * So we take the phase of the IQ signal from sdr.c and send its frequency:
 *
 *     f[n] = arg(s[n] * conj(s[n-1])) * samplerate / 2pi
 *
 * The carrier is on at a constant drive level (--sdr-tx-gain 0..7) while
 * |s| is above a threshold, and off (pin switched to input) below it.
 *
 * One FM channel: the phase is the channel's phase, the conversion is exact.
 *
 * Two FM channels: s = e^(j th1) + r * e^(j (dw t + th2)), with r < 1. Its
 * phase is th1 + arg(1 + r e^(j psi)), psi = dw t + th2 - th1. Expanding
 * e^(j arg(1 + r e^(j psi))) into its Fourier series in psi gives
 *     c0 e^(j th1) + c1 e^(j (dw t + th2)) + c-1 e^(j (2 th1 - th2 - dw t)) + ...
 * the two channels are exact copies, the other terms land at -dw, +2dw, ...
 * For r = 0.6: channel 1 at -0.9 dB, channel 2 at -10 dB, the strongest
 * other term at -12 dB one spacing below channel 1. r must stay below 1,
 * at r = 1 the sum passes through zero and the phase jumps. sdr.c sets the
 * levels, see RPITX_DEFAULT_RATIO and --sdr-rpitx-ratio.
 *
 * AM channels only get the carrier keyed on and off.
 *
 * Pacing: rpitx_dma_write() blocks until the DMA ring has room, so it is the
 * sample clock (the backend is "device paced", see sdr_get_tosend()). The ring
 * is circular: if we are late, the DMA sends old samples again, without any
 * error. So we check the time between writes against the fill level we left
 * and report it.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include "rpitx.h"
#include "rpitx_dma.h"
#include "../liblogging/logging.h"

#define RPITX_MIN_RATE		10000
#define RPITX_MAX_RATE		250000	/* 2 control blocks per sample as ngfmdmasync, pifmrds runs 228000 */
#define RPITX_MIN_FREQUENCY	5e3
#define RPITX_MAX_FREQUENCY	1500e6
#define RPITX_FIFO_TIME		0.08	/* sendiq: 16000 samples at 200 kS/s */
#define RPITX_MIN_FIFO		4000
#define RPITX_LEVEL_ON		0.10	/* |IQ| that keys the carrier on */
#define RPITX_LEVEL_OFF		0.05	/* |IQ| that keys it off (hysteresis) */
#define RPITX_REPORT_INTERVAL	10.0	/* seconds */

static struct rpitx {
	int		open;
	int		running;
	int		samplerate;
	int		fifo;
	int		pad;

	/* conversion */
	int		carrier_on;
	int		prev_valid;
	float		prev_i, prev_q;
	float		last_freq;
	float		*freq;
	float		*level;
	int		buffer_size;

	/* pacing */
	int		have_last;
	double		t_last_return;
	int		fill_last_return;
	double		t_report;
	long long	samples_report;
	int		min_fill;
	double		max_block;
	long		underruns;
	double		underrun_time;
	long		underruns_total;
	long		dry_reported;
} rt;

static double get_time(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/*
 * The DMA does not stop with the process: it would go on reading control
 * blocks from memory we no longer own, with the carrier on. So we stop it
 * on exit() and on every signal that ends the process without the
 * application's own clean up. The applications trap SIGINT, SIGHUP, SIGTERM
 * and SIGPIPE and close the SDR. Only SIGKILL cannot be caught.
 */
static const struct {
	int	sig;
	int	always;	/* 1: crash, chain to any old handler; 0: only if untrapped */
} stop_signals[] = {
	{ SIGSEGV, 1 }, { SIGBUS, 1 }, { SIGFPE, 1 }, { SIGILL, 1 }, { SIGABRT, 1 },
	{ SIGQUIT, 0 }, { SIGUSR1, 0 }, { SIGUSR2, 0 }, { SIGALRM, 0 },
	{ SIGXCPU, 0 }, { SIGXFSZ, 0 }, { SIGVTALRM, 0 }, { SIGPROF, 0 },
};
#define NUM_STOP (int)(sizeof(stop_signals) / sizeof(stop_signals[0]))
static struct sigaction old_action[NUM_STOP];
static int installed[NUM_STOP];

static void stop_handler(int sig)
{
	int i;

	rpitx_dma_stop();
	for (i = 0; i < NUM_STOP; i++) {
		if (stop_signals[i].sig == sig)
			sigaction(sig, &old_action[i], NULL);
	}
	raise(sig);
}

static void stop_at_exit(void)
{
	rpitx_dma_stop();
}

static void install_stop_handlers(void)
{
	static int atexit_done = 0;
	struct sigaction sa, old;
	int i;

	if (!atexit_done) {
		atexit(stop_at_exit);
		atexit_done = 1;
	}
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = stop_handler;
	sigemptyset(&sa.sa_mask);
	for (i = 0; i < NUM_STOP; i++) {
		installed[i] = 0;
		sigaction(stop_signals[i].sig, NULL, &old);
		if (!stop_signals[i].always && old.sa_handler != SIG_DFL)
			continue;
		sigaction(stop_signals[i].sig, &sa, &old_action[i]);
		installed[i] = 1;
	}
}

static void remove_stop_handlers(void)
{
	int i;

	for (i = 0; i < NUM_STOP; i++) {
		if (installed[i])
			sigaction(stop_signals[i].sig, &old_action[i], NULL);
		installed[i] = 0;
	}
}

/* BCM2711 (Pi 4) or an earlier SoC? */
static int is_bcm2711(void)
{
	char buf[256];
	size_t n;
	FILE *fp;

	fp = fopen("/proc/device-tree/compatible", "r");
	if (!fp)
		return 0;
	n = fread(buf, 1, sizeof(buf) - 1, fp);
	fclose(fp);
	/* NUL separated strings */
	for (size_t i = 0; i < n; i++) {
		if (buf[i] == '\0')
			buf[i] = ' ';
	}
	buf[n] = '\0';
	return strstr(buf, "bcm2711") != NULL;
}

/*
 * The PLL setting librpitx will choose, before it programs it: the same
 * search as clkgpio::ComputeBestLO(), called by SetCenterFrequency() with
 * the carrier (twice the carrier on BCM2711) and the sample rate as band.
 * librpitx accepts PLL rates down to 200 MHz, but below 600 MHz the board
 * freezes (found by rpitx-ui). Returns the PLL rate, 0 if there is none.
 */
static double pll_rate(double carrier, int samplerate, int bcm2711)
{
	double xosc = bcm2711 ? 54e6 : 19.2e6;
	double f = carrier * (bcm2711 ? 2.0 : 1.0);
	int divider, best = 0;

	if (f * 2.0 > 4e9 || f < 200e6 / 4095.0)
		return 0.0;
	if (f * 2.0 > 1500e6)
		return f * 2.0;	/* PLL pre-divider */
	for (divider = 4095; divider > 1; divider--) {
		if (f * divider < 200e6 || f * divider > 1500e6)
			continue;
		best = divider;
		if ((long)((f + samplerate) * divider / xosc) == (long)((f - samplerate) * divider / xosc))
			break;
	}
	return f * best;
}

int rpitx_open(double tx_frequency, double rate, double tx_gain)
{
	int samplerate = (int)rate;
	double boundary;
	long pad;

	if (rt.open) {
		LOGP(DSDR, LOGL_ERROR, "rpitx is already open\n");
		return -1;
	}

	if (geteuid() != 0) {
		LOGP(DSDR, LOGL_ERROR, "rpitx needs root: it programs the DMA, the PLL and the GPIO through /dev/mem and the VideoCore mailbox.\n");
		return -1;
	}

	if (tx_frequency < RPITX_MIN_FREQUENCY || tx_frequency > RPITX_MAX_FREQUENCY) {
		LOGP(DSDR, LOGL_ERROR, "rpitx can transmit from %.0f kHz to %.0f MHz, not at %.6f MHz\n",
		     RPITX_MIN_FREQUENCY / 1e3, RPITX_MAX_FREQUENCY / 1e6, tx_frequency / 1e6);
		return -1;
	}

	if (pll_rate(tx_frequency, samplerate, is_bcm2711()) < 600e6) {
		LOGP(DSDR, LOGL_ERROR, "rpitx has no PLL setting for %.6f MHz on this board (the PLL would run below 600 MHz, which freezes the board, or above its limit; on the Pi 4 the limit is 1 GHz)\n",
		     tx_frequency / 1e6);
		return -1;
	}

	if (samplerate < RPITX_MIN_RATE || samplerate > RPITX_MAX_RATE) {
		LOGP(DSDR, LOGL_ERROR, "rpitx sample rate %d is out of range: %d .. %d S/s\n",
		     samplerate, RPITX_MIN_RATE, RPITX_MAX_RATE);
		return -1;
	}

	/* --sdr-tx-gain is the GPIO drive level 0..7 (2..16 mA) */
	pad = lround(tx_gain);
	if (pad < 0 || pad > 7) {
		LOGP(DSDR, LOGL_NOTICE, "rpitx: --sdr-tx-gain is the GPIO drive level 0..7, using %d instead of %.1f\n",
		     (pad < 0) ? 0 : 7, tx_gain);
		pad = (pad < 0) ? 0 : 7;
	}

	rt.samplerate = samplerate;
	rt.pad = (int)pad;
	rt.fifo = (int)(samplerate * RPITX_FIFO_TIME);
	if (rt.fifo < RPITX_MIN_FIFO)
		rt.fifo = RPITX_MIN_FIFO;

	LOGP(DSDR, LOGL_NOTICE, "rpitx: %.6f MHz on GPIO4, %d S/s, frequency + keying, drive level %d, FIFO %d samples (%.0f ms)\n",
	     tx_frequency / 1e6, samplerate, rt.pad, rt.fifo, 1000.0 * rt.fifo / samplerate);

	install_stop_handlers();
	if (rpitx_dma_open(tx_frequency, samplerate, rt.fifo, rt.pad)) {
		remove_stop_handlers();
		LOGP(DSDR, LOGL_ERROR, "rpitx: failed to open the transmitter\n");
		return -1;
	}
	/* librpitx may round the FIFO size */
	rt.fifo = rpitx_dma_fifo_size();

	/* our frequency samples reach up to +-samplerate/2 */
	if (rpitx_dma_crosses_boundary(tx_frequency, samplerate / 2.0, &boundary)) {
		LOGP(DSDR, LOGL_ERROR, "rpitx cannot transmit around %.6f MHz: the PLL multiplier steps at %.6f MHz, within +-%.0f kHz of the carrier, where the carrier would jump. Use other channels or a lower sample rate.\n",
		     tx_frequency / 1e6, boundary / 1e6, samplerate / 2e3);
		rpitx_dma_close();
		remove_stop_handlers();
		return -1;
	}

	rt.open = 1;
	return 0;
}

int rpitx_start(void)
{
	if (!rt.open) {
		LOGP(DSDR, LOGL_ERROR, "rpitx is not open\n");
		return -1;
	}

	rt.carrier_on = 0;
	rt.prev_valid = 0;
	rt.last_freq = 0.0;
	rt.have_last = 0;
	rt.t_report = 0.0;
	rt.samples_report = 0;
	rt.min_fill = rt.fifo;
	rt.max_block = 0.0;
	rt.underruns = 0;
	rt.underrun_time = 0.0;
	rt.underruns_total = 0;
	rt.dry_reported = rpitx_dma_ran_dry();
	rt.running = 1;

	/* the DMA itself starts when the FIFO is filled by a quarter */
	return 0;
}

/* turn IQ into frequency and drive level */
static void convert(const float *buff, int num)
{
	const float scale = (float)rt.samplerate / (2.0f * (float)M_PI);
	const float on2 = RPITX_LEVEL_ON * RPITX_LEVEL_ON;
	const float off2 = RPITX_LEVEL_OFF * RPITX_LEVEL_OFF;
	const float on_level = 1.0f;
	float i, q, m2, re, im;
	int s;

	for (s = 0; s < num; s++) {
		i = buff[s * 2];
		q = buff[s * 2 + 1];
		m2 = i * i + q * q;
		if (rt.carrier_on) {
			if (m2 < off2)
				rt.carrier_on = 0;
		} else {
			if (m2 > on2) {
				rt.carrier_on = 1;
				rt.prev_valid = 0;
			}
		}
		if (rt.carrier_on) {
			if (rt.prev_valid) {
				/* s[n] * conj(s[n-1]) */
				re = i * rt.prev_i + q * rt.prev_q;
				im = q * rt.prev_i - i * rt.prev_q;
				rt.last_freq = atan2f(im, re) * scale;
			}
			rt.prev_i = i;
			rt.prev_q = q;
			rt.prev_valid = 1;
			rt.level[s] = on_level;
		} else {
			rt.prev_valid = 0;
			rt.level[s] = 0.0f;
		}
		/* while off, hold the frequency, so the next key up does not jump */
		rt.freq[s] = rt.last_freq;
	}
}

static void report(double now)
{
	double elapsed = now - rt.t_report;
	int loglevel_ = (rt.underruns) ? LOGL_NOTICE : LOGL_INFO;

	long dry = rpitx_dma_ran_dry();

	if (dry != rt.dry_reported)
		loglevel_ = LOGL_NOTICE;
	/* an underrun is a hold now: the DMA repeats the last sample (an
	 * unmodulated carrier) until we are back, it never plays old signal */
	LOGP(DSDR, loglevel_, "rpitx: %.0f S/s (nominal %d), lowest FIFO fill %.1f ms, longest write %.1f ms, late %ld times (%.1f ms held), total %ld, DMA ran dry %ld times\n",
	     (double)rt.samples_report / elapsed, rt.samplerate,
	     1000.0 * rt.min_fill / rt.samplerate, 1000.0 * rt.max_block,
	     rt.underruns, 1000.0 * rt.underrun_time, rt.underruns_total,
	     dry - rt.dry_reported);
	rt.dry_reported = dry;
	rt.t_report = now;
	rt.samples_report = 0;
	rt.min_fill = rt.fifo;
	rt.max_block = 0.0;
	rt.underruns = 0;
	rt.underrun_time = 0.0;
}

static void count_underrun(double missing)
{
	rt.underruns++;
	rt.underruns_total++;
	rt.underrun_time += missing / rt.samplerate;
}

int rpitx_send(float *buff, int num)
{
	double t0, t1, drained, expected;
	int fill = -1;

	if (!rt.open || !rt.running)
		return -1;
	if (num <= 0)
		return 0;

	if (num > rt.buffer_size) {
		free(rt.freq);
		free(rt.level);
		rt.freq = malloc(num * sizeof(*rt.freq));
		rt.level = malloc(num * sizeof(*rt.level));
		if (!rt.freq || !rt.level) {
			LOGP(DSDR, LOGL_ERROR, "rpitx: no memory\n");
			rt.buffer_size = 0;
			return -1;
		}
		rt.buffer_size = num;
	}

	convert(buff, num);

	t0 = get_time();
	if (rpitx_dma_started()) {
		/* fill level now, the lowest point between our writes */
		fill = rt.fifo - rpitx_dma_free();
		if (fill < rt.min_fill)
			rt.min_fill = fill;
		/* if more time passed than the samples we left in the FIFO
		 * last time last, the DMA has run over and sent old samples
		 * again */
		if (rt.have_last) {
			drained = (t0 - rt.t_last_return) * rt.samplerate;
			if (drained > rt.fill_last_return)
				count_underrun(drained - rt.fill_last_return);
		}
	}

	rpitx_dma_write(rt.freq, rt.level, num);

	t1 = get_time();
	if (t1 - t0 > rt.max_block)
		rt.max_block = t1 - t0;
	/* the same inside the write: what was in the FIFO plus what we wrote
	 * must cover the time the write took (we may be stopped in between) */
	if (fill >= 0) {
		expected = fill + num - (t1 - t0) * rt.samplerate;
		if (expected < 0)
			count_underrun(-expected);
	}
	if (rpitx_dma_started()) {
		rt.fill_last_return = rt.fifo - rpitx_dma_free();
		rt.t_last_return = t1;
		rt.have_last = 1;
		if (rt.t_report == 0.0) {
			/* start counting when the DMA runs */
			rt.t_report = t1;
			rt.samples_report = 0;
		} else
			rt.samples_report += num;
	}
	if (rt.t_report != 0.0 && t1 - rt.t_report >= RPITX_REPORT_INTERVAL)
		report(t1);

	return num;
}

void rpitx_close(void)
{
	if (!rt.open)
		return;
	LOGP(DSDR, LOGL_INFO, "rpitx: stopping, %ld underruns\n", rt.underruns_total);
	rpitx_dma_close();
	remove_stop_handlers();
	free(rt.freq);
	free(rt.level);
	memset(&rt, 0, sizeof(rt));
}

/* not used: the backend is device paced, see rpitx_tx_device_paced() */
int rpitx_get_tosend(int buffer_size)
{
	return buffer_size;
}

int rpitx_tx_device_paced(void)
{
	return rt.open;
}
