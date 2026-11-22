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

#ifndef RPITX_DMA_H
#define RPITX_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Open the transmitter on GPIO4 at the given carrier frequency (Hz), sample rate
 * (S/s), FIFO size (samples) and drive level (0..7). Returns 0 on success. */
int rpitx_dma_open(double frequency, int samplerate, int fifo_size, int pad_level);

/* After open: does the PLL multiplier cross an integer between
 * frequency - span and frequency + span? librpitx keeps the integer part
 * fixed and only moves the 20 bit fraction, which then wraps: the carrier
 * jumps by XOSC / divider. Returns 1 and the frequency of the boundary. */
int rpitx_dma_crosses_boundary(double frequency, double span, double *boundary);

/* Write samples, block until all are in the DMA FIFO. The blocking follows the
 * DMA read position, so this call is the sample clock of the transmitter.
 * freq: frequency offset from the carrier in Hz per sample.
 * level: 0 = carrier off, above 0 = carrier on (at the drive level of open). */
void rpitx_dma_write(const float *freq, const float *level, int num);

/* Free FIFO space (samples), from our own fill count (see rpitx_dma.cpp).
 * Call it from the writing thread only: it may resync. */
int rpitx_dma_free(void);

/* How often the DMA ran past what we wrote and we jumped ahead of it. */
long rpitx_dma_resyncs(void);

/* Size of the FIFO (samples). */
int rpitx_dma_fifo_size(void);

/* Has the DMA started (it starts after the FIFO is filled by a quarter)? */
int rpitx_dma_started(void);

/* Stop the DMA and switch the GPIO back. Safe to call from a signal handler. */
void rpitx_dma_stop(void);

/* Stop and free everything. */
void rpitx_dma_close(void);

#ifdef __cplusplus
}
#endif

#endif /* RPITX_DMA_H */
