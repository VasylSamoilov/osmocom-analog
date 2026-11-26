/* No audio device (-a none): the audio wave files are the only input and
 * output (--read-rx-wave / --write-tx-wave and the others, the taps in
 * sender.c); the run is a batch, time follows the samples.
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

#ifndef _AUDIO_NONE_H
#define _AUDIO_NONE_H

#include "../libsample/sample.h"

enum paging_signal;

/* the device name that selects it (-a none) */
#define AUDIO_NONE_DEVICE "none"

void *audio_none_open(int direction, const char *audiodev, double *tx_frequency,
		      double *rx_frequency, int *am, int channels,
		      double paging_frequency, int samplerate, int buffer_size,
		      double interval, double max_deviation, double max_modulation,
		      double modulation_index);
int audio_none_start(void *inst);
void audio_none_close(void *inst);
int audio_none_write(void *inst, sample_t **samples, uint8_t **power, int num,
		     enum paging_signal *paging_signal, int *on, int channels);
int audio_none_read(void *inst, sample_t **samples, int num, int channels,
		    double *rf_level_db);
int audio_none_get_tosend(void *inst, int buffer_size);

#endif /* _AUDIO_NONE_H */
