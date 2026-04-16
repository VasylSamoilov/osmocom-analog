/* IQ WAV virtual audio device
 *
 * Provides a sender_t-compatible audio backend that reads/writes IQ WAV files
 * with built-in FM/AM modulation and demodulation.  This allows any protocol
 * that uses the sender_t framework to decode IQ captures or generate IQ output
 * without requiring SDR hardware or a sound card.
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

#ifndef _IQ_WAVE_H
#define _IQ_WAVE_H

#include "../libsample/sample.h"
#include "../libfm/fm.h"

enum paging_signal;

void *iq_wave_open(int direction, const char *audiodev, double *tx_frequency,
		   double *rx_frequency, int *am, int channels,
		   double paging_frequency, int samplerate, int buffer_size,
		   double interval, double max_deviation, double max_modulation,
		   double modulation_index);
int iq_wave_start(void *inst);
void iq_wave_close(void *inst);
int iq_wave_write(void *inst, sample_t **samples, uint8_t **power, int num,
		  enum paging_signal *paging_signal, int *on, int channels);
int iq_wave_read(void *inst, sample_t **samples, int num, int channels,
		 double *rf_level_db);
int iq_wave_get_tosend(void *inst, int buffer_size);

/* Return pointer to the FM demodulator for a given channel.
 * Returns NULL if channel is invalid or uses AM.
 * Caller can use this to enable AFC via fm_demod_afc_enable(). */
fm_demod_t *iq_wave_get_fm_demod(void *inst, int channel);

/* Global config — set from main_mobile option parsing */
extern const char *iq_read_rx_wave;
extern const char *iq_write_rx_wave;
extern const char *iq_read_tx_wave;
extern const char *iq_write_tx_wave;

/* Returns 1 if any IQ wave option is set */
int iq_wave_is_active(void);

#endif /* _IQ_WAVE_H */
