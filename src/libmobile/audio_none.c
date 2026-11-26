/* No audio device (-a none): the audio wave files are the only input and
 * output; the run is a batch, time follows the samples.
 *
 * As the IQ wave device (iq_wave.c) without the modem: a read gives silence
 * (the --read-rx-wave tap in sender.c puts the file's samples in), a write
 * goes nowhere (the --write-tx-wave tap has recorded them), every buffer is
 * accepted, and the first instance's writes advance the sample clock.
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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "get_time.h"
#include "audio_none.h"

typedef struct audio_none {
	int	samplerate;
	int	buffer_size;
} audio_none_t;

/* the instance whose samples advance the sample clock (the first opened):
 * channels of one run share the same time */
static void *audio_none_clock_owner = NULL;

void *audio_none_open(int direction, const char *audiodev, double *tx_frequency,
		      double *rx_frequency, int *am, int channels,
		      double paging_frequency, int samplerate, int buffer_size,
		      double interval, double max_deviation, double max_modulation,
		      double modulation_index)
{
	audio_none_t *an;

	(void)direction;
	(void)audiodev;
	(void)tx_frequency;
	(void)rx_frequency;
	(void)am;
	(void)channels;
	(void)paging_frequency;
	(void)interval;
	(void)max_deviation;
	(void)max_modulation;
	(void)modulation_index;

	an = calloc(1, sizeof(*an));
	if (!an) {
		LOGP(DSENDER, LOGL_ERROR, "No memory!\n");
		return NULL;
	}
	an->samplerate = samplerate;
	an->buffer_size = buffer_size;
	if (!audio_none_clock_owner)
		audio_none_clock_owner = an;
	LOGP(DSENDER, LOGL_INFO, "No audio device: the audio wave files only, samplerate=%d\n", samplerate);
	return an;
}

int audio_none_start(void *inst)
{
	(void)inst;
	return 0;
}

void audio_none_close(void *inst)
{
	if (audio_none_clock_owner == inst)
		audio_none_clock_owner = NULL;
	free(inst);
}

int audio_none_write(void *inst, sample_t **samples, uint8_t **power, int num,
		     enum paging_signal *paging_signal, int *on, int channels)
{
	audio_none_t *an = inst;

	(void)samples;
	(void)power;
	(void)paging_signal;
	(void)on;
	(void)channels;

	/* time is the samples processed: each pass writes one buffer */
	if (an == audio_none_clock_owner)
		sample_clock_advance(num, an->samplerate);
	return num;
}

int audio_none_read(void *inst, sample_t **samples, int num, int channels,
		    double *rf_level_db)
{
	audio_none_t *an = inst;
	int c;

	if (num > an->buffer_size)
		num = an->buffer_size;
	for (c = 0; c < channels; c++) {
		memset(samples[c], 0, num * sizeof(**samples));
		if (rf_level_db)
			rf_level_db[c] = NAN;
	}
	return num;
}

int audio_none_get_tosend(void *inst, int buffer_size)
{
	(void)inst;
	/* a virtual device takes a whole buffer every pass */
	return buffer_size;
}
