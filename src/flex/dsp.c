/* FLEX signal processing
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

#define CHAN flex->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "flex.h"
#include "frame.h"
#include "dsp.h"

#define MAX_DISPLAY	1.4	/* something above speech level, no emphasis */

static void dsp_init_ramp(flex_t *flex)
{
	double c;
	int i;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Generating cosine shaped ramp table.\n");
	for (i = 0; i < 256; i++) {
		/* This is mathematically incorrect... */
		if (i < 64)
			c = 1.0;
		else if (i >= 192)
			c = -1.0;
		else
			c = cos((double)(i - 64) / 128.0 * M_PI);
		flex->fsk_ramp_down[i] = c * flex->fsk_deviation * flex->fsk_polarity;
		flex->fsk_ramp_up[i] = -flex->fsk_ramp_down[i];
	}
}

/* Init transceiver instance. */
int dsp_init_sender(flex_t *flex, int samplerate, double deviation, double polarity)
{
	int rc;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Init DSP for transceiver.\n");

	/* set modulation parameters
	 * NOTE: baudrate equals modulation, because we have a raised cosine ramp of beta = 0.5 */
	sender_set_fm(&flex->sender, deviation, 1600, deviation, MAX_DISPLAY);

	flex->fsk_bitduration = (double)samplerate / 1600.0;
	flex->fsk_bitstep = 1.0 / flex->fsk_bitduration;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Use %.4f samples for one bit duration @ %d.\n", flex->fsk_bitduration, flex->sender.samplerate);

	flex->fsk_tx_buffer_size = flex->fsk_bitduration * 32.0 + 10; /* 32 bit, add some extra to prevent short buffer due to rounding */
	flex->fsk_tx_buffer = calloc(flex->fsk_tx_buffer_size, sizeof(sample_t));
	if (!flex->fsk_tx_buffer) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* create deviation and ramp */
	flex->fsk_deviation = 1.0; /* equals what we set at sender_set_fm() */
	flex->fsk_polarity = polarity;
	dsp_init_ramp(flex);

	return 0;

error:
	dsp_cleanup_sender(flex);

	return rc;
}

/* Cleanup transceiver instance. */
void dsp_cleanup_sender(flex_t *flex)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Cleanup DSP for transceiver.\n");

	if (flex->fsk_tx_buffer) {
		free(flex->fsk_tx_buffer);
		flex->fsk_tx_buffer = NULL;
	}
}

/* Encode one codeword into samples.
 * input: 32 data bits
 * output: samples
 * return number of samples */
static int fsk_block_encode(flex_t *flex, uint32_t word)
{
	/* alloc samples, add 1 in case there is a rest */
	sample_t *spl;
	double phase, bitstep, devpol;
	int i, count;
	uint8_t lastbit;

	devpol = flex->fsk_deviation * flex->fsk_polarity;
	spl = flex->fsk_tx_buffer;
	phase = flex->fsk_tx_phase;
	lastbit = flex->fsk_tx_lastbit;
	bitstep = flex->fsk_bitstep * 256.0;

	/* add 32 bits */
	for (i = 0; i < 32; i++) {
		if (lastbit) {
			if ((word & 0x80000000)) {
				/* stay up */
				do {
					*spl++ = devpol;
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
			} else {
				/* ramp down */
				do {
					*spl++ = flex->fsk_ramp_down[(uint8_t)phase];
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
				lastbit = 0;
			}
		} else {
			if ((word & 0x80000000)) {
				/* ramp up */
				do {
					*spl++ = flex->fsk_ramp_up[(uint8_t)phase];
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
				lastbit = 1;
			} else {
				/* stay down */
				do {
					*spl++ = -devpol;
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
			}
		}
		word <<= 1;
	}

	/* depending on the number of samples, return the number */
	count = ((uintptr_t)spl - (uintptr_t)flex->fsk_tx_buffer) / sizeof(*spl);

	flex->fsk_tx_phase = phase;
	flex->fsk_tx_lastbit = lastbit;

	return count;
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	flex_t *flex = (flex_t *) sender;

again:
	/* get word */
	if (!flex->fsk_tx_buffer_length) {
		uint32_t word;

		/* If no frame data buffered, ask state machine for next frame */
		if (flex->frame_buffer_pos >= flex->frame_buffer_length) {
			if (!flex_get_next_frame(flex)) {
				/* No data — output silence */
				memset(samples, 0, sizeof(*samples) * length);
				memset(power, 0, length);
				return;
			}
		}

		/* Read next 32-bit word from frame buffer (big-endian).
		 * Handle partial last word: FLEX_BUFFER_SIZE (795) is not
		 * divisible by 4, so the last chunk may be 1-3 bytes. */
		{
			int remaining = flex->frame_buffer_length - flex->frame_buffer_pos;
			if (remaining >= 4) {
				word = ((uint32_t)flex->frame_buffer[flex->frame_buffer_pos] << 24)
				     | ((uint32_t)flex->frame_buffer[flex->frame_buffer_pos + 1] << 16)
				     | ((uint32_t)flex->frame_buffer[flex->frame_buffer_pos + 2] << 8)
				     |  (uint32_t)flex->frame_buffer[flex->frame_buffer_pos + 3];
				flex->frame_buffer_pos += 4;
			} else {
				/* Partial word: pad with zeros */
				word = 0;
				if (remaining >= 1)
					word |= (uint32_t)flex->frame_buffer[flex->frame_buffer_pos] << 24;
				if (remaining >= 2)
					word |= (uint32_t)flex->frame_buffer[flex->frame_buffer_pos + 1] << 16;
				if (remaining >= 3)
					word |= (uint32_t)flex->frame_buffer[flex->frame_buffer_pos + 2] << 8;
				flex->frame_buffer_pos = flex->frame_buffer_length;
			}
		}

		/* encode */
		flex->fsk_tx_buffer_length = fsk_block_encode(flex, word);
		flex->fsk_tx_buffer_pos = 0;
	}

	/* send encoded word until end of source or destination buffer is reached */
	while (length) {
		*power++ = 1;
		*samples++ = flex->fsk_tx_buffer[flex->fsk_tx_buffer_pos++];
		length--;
		if (flex->fsk_tx_buffer_pos == flex->fsk_tx_buffer_length) {
			flex->fsk_tx_buffer_length = 0;
			break;
		}
	}

	/* do again, if destination buffer is not yet full */
	if (length)
		goto again;
}

/* Process received audio stream from radio unit (no-op, TX only) */
void sender_receive(sender_t *sender, sample_t *samples, int length, double __attribute__((unused)) rf_level_db)
{
	(void)sender; (void)samples; (void)length;
}
