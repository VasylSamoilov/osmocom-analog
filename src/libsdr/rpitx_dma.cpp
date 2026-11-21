/* rpitx DMA wrapper - plain C interface to the librpitx C++ classes
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
 * The only C++ in the rpitx backend: it drives the librpitx classes and
 * nothing else. The signal processing is in rpitx.c.
 *
 * librpitx moves the fractional multiplier of PLLC once per sample by DMA
 * and puts the clock out on GPIO4 (GPCLK0). The DMA is paced by the PCM
 * clock at the sample rate. The FIFO is a ring of DMA control blocks; the
 * Set...Samples() calls sleep until the ring has room, so they are the
 * sample clock for whoever writes.
 *
 * iqdmasync in MODE_FREQ_A takes (frequency, level) per sample, the level
 * 0 switches the pin to input (carrier off), 1..8 is PAD drive 0..7. This
 * is what sendiq -d and spectrumpaint use. 4 control blocks per sample,
 * sendiq limits it to 200 kS/s.
 *
 * ngfmdmasync takes the frequency only (2 control blocks per sample), the
 * carrier cannot be switched off. This is what pifmrds uses at 228 kS/s.
 */

#include <librpitx/librpitx.h>
#include <complex>
#include <cstdio>

#include "rpitx_dma.h"

static iqdmasync *iq_tx = NULL;
static ngfmdmasync *fm_tx = NULL;
static bufferdma *dma_buf = NULL;	/* the one that is open */
static clkgpio *dma_clk = NULL;

#define WRITE_CHUNK 1024
static std::complex<float> fa_buffer[WRITE_CHUNK];

/*
 * Start and end of a transmission, as rpitx-ui does it
 * (https://github.com/IgrikXD/rpitx-ui, utils/dma/dma_transmission.cpp):
 *
 * The PCM FIFO that paces the DMA is empty at the start, so the DMA plays
 * its first samples back to back until the FIFO is full (64 samples),
 * starting with ring slot 0, which librpitx never writes before the start.
 * So slot 0 and a lead of 64 samples get an idle sample.
 *
 * The Set...Samples() calls return when the samples are in the ring, not
 * when they are played. Stopping right away loses up to a quarter ring plus
 * a block. So at the end one ring of idle samples is queued, in two halves
 * (a single write of a whole ring would wrap before the DMA has started).
 *
 * The idle sample is "carrier off" in frequency+level mode and 0 Hz (carrier
 * on, unmodulated) in frequency only mode.
 */
#define PCM_FIFO_DEPTH 64

static void write_idle(int num)
{
	int i, n;

	if (fm_tx) {
		static float zero[WRITE_CHUNK];
		while (num > 0) {
			n = (num > WRITE_CHUNK) ? WRITE_CHUNK : num;
			fm_tx->SetFrequencySamples(zero, n);
			num -= n;
		}
		return;
	}
	while (num > 0) {
		n = (num > WRITE_CHUNK) ? WRITE_CHUNK : num;
		for (i = 0; i < n; i++)
			fa_buffer[i] = std::complex<float>(0.0f, 0.0f);
		iq_tx->SetIQSamples(fa_buffer, n, 1);
		num -= n;
	}
}

extern "C" int rpitx_dma_open(enum rpitx_dma_mode mode, double frequency, int samplerate, int fifo_size, int pad_level)
{
	if (dma_buf)
		return -1;

	if (mode == RPITX_DMA_FREQ_AMP) {
		iq_tx = new iqdmasync((uint64_t)frequency, (uint32_t)samplerate, 14, (uint32_t)fifo_size, MODE_FREQ_A);
		/* same PLL loop as sendiq */
		iq_tx->SetPLLMasterLoop(3, 4, 0);
		/* The constructor puts the clock out on GPIO4 already, unmodulated
		 * at the center frequency, until the DMA starts. If the samples
		 * never come (a stalled process), that carrier would stay on air.
		 * Switch the pin to input: the DMA sets its function with every
		 * sample, so it comes on with the first one. */
		iq_tx->disableclk(4);
		iq_tx->SetFreqAmplitudeSample(0, std::complex<float>(0.0f, 0.0f), 1);
		dma_buf = iq_tx;
		dma_clk = iq_tx;
		write_idle(PCM_FIFO_DEPTH);
	} else {
		fm_tx = new ngfmdmasync((uint64_t)frequency, (uint32_t)samplerate, 14, (uint32_t)fifo_size);
		/* constant drive, as there is no level per sample */
		padgpio pad;
		pad.setlevel(pad_level);
		fm_tx->SetFrequencySample(0, 0.0f);
		dma_buf = fm_tx;
		dma_clk = fm_tx;
		write_idle(PCM_FIFO_DEPTH);
	}

	return 0;
}

extern "C" int rpitx_dma_crosses_boundary(double frequency, double span, double *boundary)
{
	double k, lo, hi;

	if (!dma_clk)
		return 0;
	/* PLL multiplier per Hz, as in clkgpio::GetMasterFrac() */
	if (dma_clk->PllFixDivider == 1) {
		/* PLL pre-divider (above 750 MHz), no Pi 4 pre-divider */
		if (dma_clk->pi_is_2711)
			k = 2.0 / (double)dma_clk->XOSC_FREQUENCY;
		else
			k = 1.0 / ((double)dma_clk->XOSC_FREQUENCY * 2.0);
	} else {
		if (dma_clk->pi_is_2711)
			k = (double)dma_clk->PllFixDivider * 2.0 / (double)dma_clk->XOSC_FREQUENCY;
		else
			k = (double)dma_clk->PllFixDivider / (double)dma_clk->XOSC_FREQUENCY;
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
	int i, n;

	if (fm_tx) {
		/* SetFrequencySamples() takes a non-const pointer, but only reads */
		fm_tx->SetFrequencySamples((float *)freq, num);
		return;
	}
	if (!iq_tx)
		return;

	while (num > 0) {
		n = (num > WRITE_CHUNK) ? WRITE_CHUNK : num;
		for (i = 0; i < n; i++)
			fa_buffer[i] = std::complex<float>(freq[i], level[i]);
		iq_tx->SetIQSamples(fa_buffer, n, 1);
		freq += n;
		level += n;
		num -= n;
	}
}

extern "C" int rpitx_dma_free(void)
{
	if (!dma_buf)
		return 0;
	return dma_buf->GetBufferAvailable();
}

extern "C" int rpitx_dma_fifo_size(void)
{
	if (!dma_buf)
		return 0;
	return (int)dma_buf->buffersize;
}

extern "C" int rpitx_dma_started(void)
{
	if (!dma_buf)
		return 0;
	return dma_buf->Started;
}

extern "C" void rpitx_dma_stop(void)
{
	if (!dma_buf)
		return;
	dma_buf->stop();
	dma_clk->disableclk(4);
}

extern "C" void rpitx_dma_close(void)
{
	if (!dma_buf)
		return;
	/* play what is still queued, see above; this also starts a DMA that
	 * never got a quarter of the ring, so a short transmission is sent */
	write_idle(dma_buf->buffersize / 2);
	write_idle(dma_buf->buffersize / 2);
	dma_buf->stop();
	/* the destructors switch the clock off and give the GPIO back */
	delete iq_tx;
	delete fm_tx;
	iq_tx = NULL;
	fm_tx = NULL;
	dma_buf = NULL;
	dma_clk = NULL;
}
