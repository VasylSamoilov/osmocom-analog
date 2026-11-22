/* rpitx DMA wrapper - plain C interface to librpitx
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
 * The only C++ in the rpitx backend: the DMA chain on librpitx's classes,
 * nothing else. The signal processing is in rpitx.c.
 *
 * librpitx moves the fractional multiplier of PLLC once per sample by DMA
 * and puts the clock out on GPIO4 (GPCLK0). The DMA is paced by the PCM
 * clock at the sample rate. The FIFO is a ring of DMA control blocks; the
 * writes sleep until the ring has room, so they are the sample clock for
 * whoever writes.
 *
 * Our chain (keyed_fm_dma) has two control blocks per sample, as librpitx's
 * ngfmdmasync: the first writes the PLLC fraction (the frequency), the second
 * paces at the PCM FIFO. Only on a sample where the carrier switches on or
 * off, the first block is pointed at the function register of GPIO4 instead
 * (clock out or input) and the frequency holds for that sample.
 *
 * Why not iqdmasync MODE_FREQ_A, which keys too: it writes 4 blocks per
 * sample (pad drive, fraction, function, pacing) and that makes the moment
 * of the frequency update jitter. Measured on a Pi 4 at 192 kS/s with test
 * tones (3 kHz deviation): one channel 21..27 dB SINAD with iqdmasync, 44 dB
 * with this chain; two channels on one carrier 17 / 8 dB with iqdmasync,
 * 25 / 24 dB with this chain (the same as ngfmdmasync, which cannot key).
 * A 3 block chain (fraction, function, pacing in every sample) gave 24 / 18.
 */

#include <librpitx/librpitx.h>
#include <cstdio>
#include <unistd.h>

#include "rpitx_dma.h"

static double mono_time(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

/*
 * Fill level. librpitx's GetBufferAvailable() is (DMA position - last written
 * slot) mod ring size. When we are late and the DMA overtakes the last
 * written slot, that reads "nearly full", the write would sleep up to 3/4 of
 * the ring "for room" while the DMA plays old signal, and one short delay
 * turns into a whole ring of old signal. So we keep the fill ourselves: the
 * DMA's advance between two polls (the wall clock tells how many whole rings
 * it went round), minus what it played, plus what we wrote. When it has run
 * past what we wrote, we jump the write position a short lead ahead of the DMA
 * (a resync) instead of waiting.
 */
#define RESYNC_LEAD_DIV 8	/* lead after a resync: 1/8 ring (10 ms at 80 ms) */

class keyed_fm_dma : public bufferdma, public clkgpio, public pcmgpio
{
	uint32_t fsel_on, fsel_off;
	uint32_t samplerate;
	bool last_on;
	uint32_t wr_last;	/* the next write goes to wr_last + 1 */
	uint32_t pushed;	/* slot last given to PushSample (librpitx's last_sample) */
	long fill;		/* samples written and not played yet */
	bool tracking;
	uint32_t prev_pos;
	double prev_t;
	bool rekey;		/* write the pin function on the next sample */

public:
	long resyncs;

	keyed_fm_dma(uint64_t frequency, uint32_t rate, int channel, uint32_t fifo_size)
		: bufferdma(channel, fifo_size, 2, 1), samplerate(rate), last_on(false),
		  wr_last(0), pushed(0), fill(0), tracking(false), prev_pos(0), prev_t(0.0), rekey(false), resyncs(0)
	{
		uint32_t fsel;

		/* the same set up as ngfmdmasync */
		clkgpio::SetAdvancedPllMode(true);
		clkgpio::SetCenterFrequency(frequency, samplerate);
		clkgpio::SetFrequency(0);
		clkgpio::enableclk(4);
		pcmgpio::SetPllNumber(clk_plld, 1);
		pcmgpio::SetFrequency(samplerate);
		/* GPIO4 function: alt0 = GPCLK0 = carrier on, input = off. The
		 * constructor puts the clock out already, unmodulated at the
		 * center frequency: switch it off until the first sample */
		fsel = clkgpio::gengpio.gpioreg[GPFSEL0];
		fsel_on = (fsel & ~(7 << 12)) | (4 << 12);
		fsel_off = fsel & ~(7 << 12);
		clkgpio::gengpio.gpioreg[GPFSEL0] = fsel_off;
		SetDmaAlgo();
	}

	~keyed_fm_dma()
	{
		clkgpio::gengpio.gpioreg[GPFSEL0] = fsel_off;
		clkgpio::disableclk(4);
	}

	void SetDmaAlgo()
	{
		dma_cb_t *cbp = cbarray;

		for (uint32_t s = 0; s < buffersize; s++) {
			SetEasyCB(cbp++, s, dma_pllc_frac, 1);
			SetEasyCB(cbp++, s, dma_pcm, 1);
		}
		cbp--;
		cbp->next = mem_virt_to_phys(cbarray);
	}

	/* one sample into a ring slot the DMA has played */
	void set_sample(uint32_t index, float frequency, bool on)
	{
		dma_cb_t *cb;

		index %= buffersize;
		cb = &cbarray[index * 2];
		if (on != last_on || rekey) {
			/* after a resync the slots just played may have switched the
			 * pin: set it again */
			rekey = false;
			cb->dst = dma_fsel;
			sampletab[index] = on ? fsel_on : fsel_off;
			last_on = on;
		} else {
			cb->dst = dma_pllc_frac;
			sampletab[index] = (0x5A << 24) | GetMasterFrac(frequency);
		}
		PushSample(index);
		wr_last = index;
		pushed = index;
	}

	/* free slots, see "Fill level" above; resyncs if the DMA ran past us */
	int free_slots(void)
	{
		int raw = GetBufferAvailable();
		long n = (long)buffersize, adv;
		uint32_t pos;
		double now, est;

		if (!Started) {
			tracking = false;
			return raw;
		}
		pos = (uint32_t)((pushed + (uint32_t)raw) % buffersize);
		now = mono_time();
		if (!tracking) {
			tracking = true;
			fill = n - raw;
		} else {
			adv = ((long)pos - (long)prev_pos + n) % n;
			/* more than half a ring more than the position says: it
			 * went round whole rings in between */
			est = (now - prev_t) * samplerate;
			while ((double)adv + (double)n / 2.0 < est)
				adv += n;
			fill -= adv;
		}
		prev_pos = pos;
		prev_t = now;
		if (fill <= 0) {
			long lead = n / RESYNC_LEAD_DIV;

			/* the next write goes a lead ahead of the DMA */
			wr_last = (uint32_t)(((long)pos + lead - 1 + n) % n);
			fill = lead;
			rekey = true;
			resyncs++;
		}
		return (int)(n - fill - 1);
	}

	/* blocks until all samples are in the ring, with ngfmdmasync's rule:
	 * sleep until 3/4 of the ring is free, then write what fits */
	void set_samples(const float *freq, const float *level, int num)
	{
		int done = 0, avail, n, i, sleep_us;

		while (done < num) {
			avail = free_slots();
			sleep_us = (int)(1e6 * ((int)buffersize * 3 / 4 - avail) / samplerate) - 200;
			if (sleep_us > 0)
				usleep(sleep_us);
			avail = free_slots();
			n = (num - done < avail) ? num - done : avail;
			for (i = 0; i < n; i++, done++)
				set_sample(wr_last + 1, freq[done], level[done] > 0.0f);
			if (Started)
				fill += n;
		}
	}
};

static keyed_fm_dma *tx = NULL;

/*
 * Start and end of a transmission, as rpitx-ui does it
 * (https://github.com/IgrikXD/rpitx-ui, utils/dma/dma_transmission.cpp):
 *
 * The PCM FIFO that paces the DMA is empty at the start, so the DMA plays
 * its first samples back to back until the FIFO is full (64 samples),
 * starting with ring slot 0, which librpitx never writes before the start.
 * So slot 0 and a lead of 64 samples get "carrier off".
 *
 * The writes return when the samples are in the ring, not when they are
 * played. Stopping right away loses up to a quarter ring plus a block. So at
 * the end one ring of "carrier off" is queued, in two halves (a single write
 * of a whole ring would wrap before the DMA has started).
 */
#define PCM_FIFO_DEPTH 64
#define IDLE_CHUNK 1024

static void write_idle(int num)
{
	static float zero[IDLE_CHUNK];
	int n;

	while (num > 0) {
		n = (num > IDLE_CHUNK) ? IDLE_CHUNK : num;
		tx->set_samples(zero, zero, n);
		num -= n;
	}
}

extern "C" int rpitx_dma_open(double frequency, int samplerate, int fifo_size, int pad_level)
{
	if (tx)
		return -1;

	tx = new keyed_fm_dma((uint64_t)frequency, (uint32_t)samplerate, 14, (uint32_t)fifo_size);
	/* constant drive */
	{
		padgpio pad;
		pad.setlevel(pad_level);
	}
	tx->set_sample(0, 0.0f, false);
	write_idle(PCM_FIFO_DEPTH);

	return 0;
}

extern "C" int rpitx_dma_crosses_boundary(double frequency, double span, double *boundary)
{
	double k, lo, hi;

	if (!tx)
		return 0;
	/* PLL multiplier per Hz, as in clkgpio::GetMasterFrac() */
	clkgpio *clk = tx;
	double xosc = (double)clk->XOSC_FREQUENCY;

	if (clk->PllFixDivider == 1) {
		/* PLL pre-divider (above 750 MHz), no Pi 4 pre-divider */
		if (clk->pi_is_2711)
			k = 2.0 / xosc;
		else
			k = 1.0 / (xosc * 2.0);
	} else {
		if (clk->pi_is_2711)
			k = (double)clk->PllFixDivider * 2.0 / xosc;
		else
			k = (double)clk->PllFixDivider / xosc;
	}
	lo = (frequency - span) * k;
	hi = (frequency + span) * k;
	if ((long)lo == (long)hi)
		return 0;
	*boundary = (double)(long)hi / k;
	return 1;
}

extern "C" void rpitx_dma_write(const float *freq, const float *level, int num)
{
	if (tx)
		tx->set_samples(freq, level, num);
}

extern "C" int rpitx_dma_free(void)
{
	if (!tx)
		return 0;
	return tx->free_slots();
}

extern "C" long rpitx_dma_resyncs(void)
{
	if (!tx)
		return 0;
	return tx->resyncs;
}

extern "C" int rpitx_dma_fifo_size(void)
{
	if (!tx)
		return 0;
	return (int)tx->buffersize;
}

extern "C" int rpitx_dma_started(void)
{
	if (!tx)
		return 0;
	return tx->Started;
}

extern "C" void rpitx_dma_stop(void)
{
	if (!tx)
		return;
	tx->stop();
	tx->disableclk(4);
}

extern "C" void rpitx_dma_close(void)
{
	if (!tx)
		return;
	/* play what is still queued, see above; this also starts a DMA that
	 * never got a quarter of the ring, so a short transmission is sent */
	write_idle(tx->buffersize / 2);
	write_idle(tx->buffersize / 2);
	tx->stop();
	/* the destructor switches the pin to input and the clock off */
	delete tx;
	tx = NULL;
}
