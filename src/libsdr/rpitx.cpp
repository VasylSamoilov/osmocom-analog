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
 * This is a thin C++ shim that wraps librpitx's C++ classes and exposes
 * a plain C interface for the SDR backend in sdr.c.
 *
 * librpitx uses the Raspberry Pi's DMA engine and PLL to generate RF
 * signals on GPIO4. It supports IQ modulation (iqdmasync) and pure FM
 * (ngfmdmasync). We use iqdmasync here since sdr_write() produces IQ
 * samples.
 *
 * TX only — librpitx has no receive capability.
 */

#include <librpitx/librpitx.h>
#include <complex>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <time.h>

extern "C" {
#include "rpitx.h"
}

/* IQ DMA sync object — the core transmitter */
static iqdmasync *iqsender = NULL;

/* Configuration state */
static double rpitx_samplerate = 0;
static double rpitx_frequency = 0;
static int rpitx_running = 0;

/* Software clock for get_tosend timing */
static struct timespec rpitx_base_ts;
static long long rpitx_tx_time_ns = 0;
static int rpitx_clock_valid = 0;

/* Buffer for IQ conversion */
#define RPITX_BURST_SIZE 4000
static std::complex<float> iq_buffer[RPITX_BURST_SIZE];

static long long get_time_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (ts.tv_sec * 1000000000LL + ts.tv_nsec) -
	       (rpitx_base_ts.tv_sec * 1000000000LL + rpitx_base_ts.tv_nsec);
}

extern "C" int rpitx_open(double tx_frequency, double rate, double tx_gain)
{
	int fifo_size;
	int pad_level;

	if (iqsender) {
		fprintf(stderr, "rpitx already open!\n");
		return -1;
	}

	rpitx_samplerate = rate;
	rpitx_frequency = tx_frequency;

	/* Map gain (0-7) to PAD drive level */
	pad_level = (int)tx_gain;
	if (pad_level < 0) pad_level = 0;
	if (pad_level > 7) pad_level = 7;

	/* FIFO size: 4x burst for comfortable buffering */
	fifo_size = RPITX_BURST_SIZE * 4;

	fprintf(stderr, "rpitx: Opening at %.6f MHz, rate %.0f Hz, gain %d\n",
		tx_frequency / 1e6, rate, pad_level);

	try {
		iqsender = new iqdmasync(
			(uint64_t)tx_frequency,
			(uint32_t)rate,
			14,		/* DMA channel (auto-adjusted by librpitx) */
			fifo_size,
			MODE_IQ
		);
		iqsender->SetPLLMasterLoop(3, 4, 0);
	} catch (...) {
		fprintf(stderr, "rpitx: Failed to initialize librpitx!\n");
		fprintf(stderr, "rpitx: Are you running on a Raspberry Pi with root privileges?\n");
		iqsender = NULL;
		return -1;
	}

	fprintf(stderr, "rpitx: Initialized successfully\n");
	return 0;
}

extern "C" int rpitx_start(void)
{
	if (!iqsender) {
		fprintf(stderr, "rpitx: not open!\n");
		return -1;
	}

	/* Initialize software clock */
	clock_gettime(CLOCK_MONOTONIC_RAW, &rpitx_base_ts);
	rpitx_tx_time_ns = 0;
	rpitx_clock_valid = 1;
	rpitx_running = 1;

	fprintf(stderr, "rpitx: Started TX on GPIO4\n");
	return 0;
}

extern "C" int rpitx_send(float *buff, int num)
{
	int i, sent = 0;

	if (!iqsender || !rpitx_running)
		return -1;

	/* Convert interleaved float I/Q pairs to std::complex<float> and
	 * send in bursts. librpitx's SetIQSamples handles DMA buffer
	 * management and pacing internally. */
	while (sent < num) {
		int chunk = num - sent;
		if (chunk > RPITX_BURST_SIZE)
			chunk = RPITX_BURST_SIZE;

		for (i = 0; i < chunk; i++) {
			iq_buffer[i] = std::complex<float>(
				buff[(sent + i) * 2],
				buff[(sent + i) * 2 + 1]
			);
		}

		iqsender->SetIQSamples(iq_buffer, chunk, 1);
		sent += chunk;
	}

	/* Advance software TX clock */
	if (rpitx_clock_valid) {
		rpitx_tx_time_ns += (long long)((double)num / rpitx_samplerate * 1e9);
	}

	return num;
}

extern "C" void rpitx_close(void)
{
	if (iqsender) {
		fprintf(stderr, "rpitx: Closing\n");
		iqsender->stop();
		delete iqsender;
		iqsender = NULL;
	}
	rpitx_running = 0;
	rpitx_clock_valid = 0;
}

extern "C" int rpitx_get_tosend(int buffer_size)
{
	long long now_ns, target_ns;
	int tosend;

	if (!rpitx_clock_valid || !rpitx_running)
		return buffer_size;

	/* Use software clock to determine how many samples to send.
	 * Target: keep TX time ahead of wall clock by buffer_size samples. */
	now_ns = get_time_ns();
	target_ns = now_ns + (long long)((double)buffer_size / rpitx_samplerate * 1e9);

	if (rpitx_tx_time_ns >= target_ns)
		return 0;

	tosend = (int)((double)(target_ns - rpitx_tx_time_ns) * rpitx_samplerate / 1e9);
	if (tosend > buffer_size)
		tosend = buffer_size;

	return tosend;
}
