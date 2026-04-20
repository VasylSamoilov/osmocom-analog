/* IQ WAV virtual audio device
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libwave/wave.h"
#include "../libfm/fm.h"
#include "../libam/am.h"
#include "../libsound/sound.h"
#include "iq_wave.h"

/* Global config — set from main_mobile option parsing */
const char *iq_read_rx_wave = NULL;
const char *iq_write_rx_wave = NULL;
const char *iq_read_tx_wave = NULL;
const char *iq_write_tx_wave = NULL;

int iq_wave_is_active(void)
{
	return (iq_read_rx_wave || iq_write_rx_wave ||
		iq_read_tx_wave || iq_write_tx_wave);
}

/* Per-channel state for FM/AM demod/mod */
struct iq_wave_chan {
	int		am;		/* 0 = FM, 1 = AM */
	fm_demod_t	fm_demod;
	fm_mod_t	fm_mod;
	am_demod_t	am_demod;
	am_mod_t	am_mod;
};

/* Instance state */
typedef struct iq_wave {
	int		samplerate;
	int		channels;
	int		buffer_size;
	double		amplitude;	/* TX amplitude per channel */
	double		max_deviation;
	double		max_modulation;
	double		modulation_index;

	/* per-channel demod/mod */
	struct iq_wave_chan *chan;

	/* IQ wave file I/O */
	wave_play_t	wave_rx_play;	/* IQ RX playback */
	wave_rec_t	wave_rx_rec;	/* IQ RX recording */
	wave_play_t	wave_tx_play;	/* IQ TX playback */
	wave_rec_t	wave_tx_rec;	/* IQ TX recording */

	/* scratch buffers for IQ ↔ baseband conversion */
	float		*iq_buff;	/* interleaved I/Q float buffer */
	sample_t	*mod_I;		/* demod scratch I */
	sample_t	*mod_Q;		/* demod scratch Q */
	sample_t	*mod_carrier;	/* AM carrier scratch */
	sample_t	*wavespl0;	/* wave I channel */
	sample_t	*wavespl1;	/* wave Q channel */

	/* TX timing: track how many samples have been written vs read */
	int		tx_write_pos;
	int		tx_read_pos;

	/* RX EOF flag and padding */
	int		rx_eof;
	int		rx_pad_remaining;	/* padding samples after EOF */
} iq_wave_t;

void *iq_wave_open(int direction, const char *audiodev, double *tx_frequency,
		   double *rx_frequency, int *am, int channels,
		   double paging_frequency, int samplerate, int buffer_size,
		   double interval, double max_deviation, double max_modulation,
		   double modulation_index)
{
	iq_wave_t *iq;
	int rc;
	int c;
	double bandwidth;

	(void)direction;
	(void)audiodev;
	(void)tx_frequency;
	(void)rx_frequency;
	(void)paging_frequency;
	(void)interval;

	LOGP(DSDR, LOGL_INFO, "Opening IQ wave virtual device: samplerate=%d channels=%d\n",
	     samplerate, channels);

	iq = calloc(1, sizeof(*iq));
	if (!iq) {
		LOGP(DSDR, LOGL_ERROR, "No memory!\n");
		return NULL;
	}

	iq->samplerate = samplerate;
	iq->channels = channels;
	iq->buffer_size = buffer_size;
	iq->amplitude = (channels > 0) ? 1.0 / (double)channels : 1.0;
	iq->max_deviation = max_deviation;
	iq->max_modulation = max_modulation;
	iq->modulation_index = modulation_index;

	bandwidth = 2.0 * (max_deviation + max_modulation);
	if (bandwidth > 0.0)
		LOGP(DSDR, LOGL_INFO, "Channel bandwidth: 2 * (%.1f + %.1f) = %.1f Hz\n",
		     max_deviation, max_modulation, bandwidth);

	/* allocate scratch buffers */
	iq->iq_buff = calloc(buffer_size * 2, sizeof(float));
	iq->mod_I = calloc(buffer_size, sizeof(sample_t));
	iq->mod_Q = calloc(buffer_size, sizeof(sample_t));
	iq->mod_carrier = calloc(buffer_size, sizeof(sample_t));
	iq->wavespl0 = calloc(buffer_size, sizeof(sample_t));
	iq->wavespl1 = calloc(buffer_size, sizeof(sample_t));
	if (!iq->iq_buff || !iq->mod_I || !iq->mod_Q || !iq->mod_carrier ||
	    !iq->wavespl0 || !iq->wavespl1) {
		LOGP(DSDR, LOGL_ERROR, "No memory!\n");
		goto error;
	}

	/* allocate per-channel state */
	if (channels > 0) {
		iq->chan = calloc(channels, sizeof(*iq->chan));
		if (!iq->chan) {
			LOGP(DSDR, LOGL_ERROR, "No memory!\n");
			goto error;
		}
	}

	/* initialize per-channel FM/AM demod and mod
	 * offset = 0 because IQ WAV is centered on the signal.
	 * Unlike live SDR (which shifts center to avoid DC), captured IQ files
	 * typically have DC removal already applied by the recording software. */
	for (c = 0; c < channels; c++) {
		iq->chan[c].am = am[c];
		if (am[c]) {
			double gain = modulation_index / 2.0;
			double bias = 1.0 - gain;
			rc = am_demod_init(&iq->chan[c].am_demod, samplerate,
					   0.0, bandwidth / 2.0,
					   1.0 / modulation_index);
			if (rc < 0)
				goto error;
			rc = am_mod_init(&iq->chan[c].am_mod, samplerate,
					 0.0, iq->amplitude * gain,
					 iq->amplitude * bias);
			if (rc < 0)
				goto error;
			LOGP(DSDR, LOGL_INFO, "Channel %d: AM demod/mod (index=%.2f)\n",
			     c, modulation_index);
		} else {
			rc = fm_demod_init(&iq->chan[c].fm_demod, samplerate,
					   0.0, bandwidth);
			if (rc < 0)
				goto error;
			rc = fm_mod_init(&iq->chan[c].fm_mod, samplerate,
					 0.0, iq->amplitude);
			if (rc < 0)
				goto error;
			LOGP(DSDR, LOGL_INFO, "Channel %d: FM demod/mod (bw=%.0f Hz)\n",
			     c, bandwidth);
		}
	}

	/* open IQ wave files */
	if (iq_write_rx_wave) {
		rc = wave_create_record(&iq->wave_rx_rec, iq_write_rx_wave,
					samplerate, 2, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create IQ RX wave recording!\n");
			goto error;
		}
	}
	if (iq_read_rx_wave) {
		int iq_samplerate = samplerate;
		int iq_channels = 2;
		rc = wave_create_playback(&iq->wave_rx_play, iq_read_rx_wave,
					  &iq_samplerate, &iq_channels, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to open IQ RX wave file!\n");
			goto error;
		}
		if (iq_samplerate != samplerate) {
			LOGP(DSDR, LOGL_ERROR, "IQ RX wave sample rate %d does not match DSP rate %d!\n",
			     iq_samplerate, samplerate);
			goto error;
		}
		LOGP(DSDR, LOGL_NOTICE, "Reading IQ RX from: %s (rate=%d)\n",
		     iq_read_rx_wave, iq_samplerate);
	}
	if (iq_write_tx_wave) {
		rc = wave_create_record(&iq->wave_tx_rec, iq_write_tx_wave,
					samplerate, 2, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create IQ TX wave recording!\n");
			goto error;
		}
	}
	if (iq_read_tx_wave) {
		int iq_samplerate = samplerate;
		int iq_channels = 2;
		rc = wave_create_playback(&iq->wave_tx_play, iq_read_tx_wave,
					  &iq_samplerate, &iq_channels, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to open IQ TX wave file!\n");
			goto error;
		}
		LOGP(DSDR, LOGL_NOTICE, "Reading IQ TX from: %s (rate=%d)\n",
		     iq_read_tx_wave, iq_samplerate);
	}

	return iq;

error:
	iq_wave_close(iq);
	return NULL;
}

int iq_wave_start(void *inst)
{
	(void)inst;
	return 0;
}

void iq_wave_close(void *inst)
{
	iq_wave_t *iq = (iq_wave_t *)inst;
	int c;

	if (!iq)
		return;

	/* destroy wave files */
	wave_destroy_playback(&iq->wave_rx_play);
	wave_destroy_record(&iq->wave_rx_rec);
	wave_destroy_playback(&iq->wave_tx_play);
	wave_destroy_record(&iq->wave_tx_rec);

	/* destroy per-channel demod/mod */
	if (iq->chan) {
		for (c = 0; c < iq->channels; c++) {
			if (iq->chan[c].am) {
				am_demod_exit(&iq->chan[c].am_demod);
				am_mod_exit(&iq->chan[c].am_mod);
			} else {
				fm_demod_exit(&iq->chan[c].fm_demod);
				fm_mod_exit(&iq->chan[c].fm_mod);
			}
		}
		free(iq->chan);
	}

	/* free scratch buffers */
	free(iq->iq_buff);
	free(iq->mod_I);
	free(iq->mod_Q);
	free(iq->mod_carrier);
	free(iq->wavespl0);
	free(iq->wavespl1);

	free(iq);
}

int iq_wave_read(void *inst, sample_t **samples, int num, int channels,
		 double *rf_level_db)
{
	iq_wave_t *iq = (iq_wave_t *)inst;
	float *buff = iq->iq_buff;
	sample_t *spl_list[2] = { iq->wavespl0, iq->wavespl1 };
	int count, c, s, ss;

	if (num > iq->buffer_size)
		num = iq->buffer_size;

	/* read IQ samples from wave file */
	if (iq->wave_rx_play.fp || iq->rx_eof) {
		if (iq->rx_eof) {
			/* EOF already hit - feed silence padding so decoder
			 * can finish processing the last frame */
			if (iq->rx_pad_remaining <= 0) {
				LOGP(DSDR, LOGL_NOTICE, "IQ RX padding exhausted, quitting.\n");
				return -EPERM;
			}
			count = (num < iq->rx_pad_remaining) ? num : iq->rx_pad_remaining;
			memset(spl_list[0], 0, count * sizeof(sample_t));
			memset(spl_list[1], 0, count * sizeof(sample_t));
			iq->rx_pad_remaining -= count;
			num = count;
			goto demod;
		}
		count = wave_read(&iq->wave_rx_play, spl_list, num);
		if (count == 0 && iq->wave_rx_play.left == 0) {
			/* File exhausted - start padding with silence for
			 * one FLEX frame (1.875s) so the decoder can finish
			 * processing any partially-received frame */
			iq->rx_eof = 1;
			iq->rx_pad_remaining = iq->samplerate * 2;
			LOGP(DSDR, LOGL_NOTICE, "IQ RX wave EOF - padding %d samples for decoder flush.\n",
			     iq->rx_pad_remaining);
			count = (num < iq->rx_pad_remaining) ? num : iq->rx_pad_remaining;
			memset(spl_list[0], 0, count * sizeof(sample_t));
			memset(spl_list[1], 0, count * sizeof(sample_t));
			iq->rx_pad_remaining -= count;
			num = count;
			goto demod;
		}
		if (count == 0) {
			/* buffer temporarily empty, return silence */
			return 0;
		}
		num = count;
		/* interleave into float IQ buffer */
demod:
		for (s = 0, ss = 0; s < num; s++) {
			buff[ss++] = (float)spl_list[0][s];
			buff[ss++] = (float)spl_list[1][s];
		}
	} else {
		/* no RX wave file — return silence */
		memset(buff, 0, num * 2 * sizeof(float));
	}

	/* record raw IQ if requested */
	if (iq->wave_rx_rec.fp) {
		for (s = 0, ss = 0; s < num; s++) {
			spl_list[0][s] = buff[ss++];
			spl_list[1][s] = buff[ss++];
		}
		wave_write(&iq->wave_rx_rec, spl_list, num);
	}

	/* demodulate IQ → baseband for each channel */
	for (c = 0; c < channels && c < iq->channels; c++) {
		if (rf_level_db)
			rf_level_db[c] = NAN;
		if (iq->chan[c].am)
			am_demodulate_complex(&iq->chan[c].am_demod, samples[c],
					      num, buff, iq->mod_I, iq->mod_Q,
					      iq->mod_carrier);
		else
			fm_demodulate_complex(&iq->chan[c].fm_demod, samples[c],
					      num, buff, iq->mod_I, iq->mod_Q);

		/* compute RF level from IQ magnitude */
		if (rf_level_db) {
			double avg = 0.0;
			for (s = 0; s < num; s++)
				avg += iq->mod_I[s] * iq->mod_I[s]
				     + iq->mod_Q[s] * iq->mod_Q[s];
			avg = sqrt(avg / (double)num);
			rf_level_db[c] = log10(avg + 1e-20) * 20.0;
		}
	}

	return num;
}

int iq_wave_write(void *inst, sample_t **samples, uint8_t **power, int num,
		  enum paging_signal *paging_signal, int *on, int channels)
{
	iq_wave_t *iq = (iq_wave_t *)inst;
	float *buff = iq->iq_buff;
	int c, s, ss;

	(void)paging_signal;
	(void)on;

	if (num > iq->buffer_size)
		num = iq->buffer_size;

	/* modulate baseband → IQ for each channel */
	memset(buff, 0, num * 2 * sizeof(float));
	for (c = 0; c < channels && c < iq->channels; c++) {
		if (iq->chan[c].am)
			am_modulate_complex(&iq->chan[c].am_mod, samples[c],
					    power[c], num, buff);
		else
			fm_modulate_complex(&iq->chan[c].fm_mod, samples[c],
					    power[c], num, buff);
	}

	/* record IQ if requested */
	if (iq->wave_tx_rec.fp) {
		sample_t *spl_list[2] = { iq->wavespl0, iq->wavespl1 };
		for (s = 0, ss = 0; s < num; s++) {
			spl_list[0][s] = buff[ss++];
			spl_list[1][s] = buff[ss++];
		}
		wave_write(&iq->wave_tx_rec, spl_list, num);
	}

	/* replace IQ from playback file if requested */
	if (iq->wave_tx_play.fp) {
		sample_t *spl_list[2] = { iq->wavespl0, iq->wavespl1 };
		wave_read(&iq->wave_tx_play, spl_list, num);
		for (s = 0, ss = 0; s < num; s++) {
			buff[ss++] = (float)spl_list[0][s];
			buff[ss++] = (float)spl_list[1][s];
		}
	}

	iq->tx_write_pos += num;

	return num;
}

int iq_wave_get_tosend(void *inst, int buffer_size)
{
	iq_wave_t *iq = (iq_wave_t *)inst;

	/* For a virtual device, we always accept up to buffer_size.
	 * Pace TX to match RX: return buffer_size worth of samples
	 * so the main loop processes at a steady rate. */
	(void)iq;
	return buffer_size;
}


