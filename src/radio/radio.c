/* main function
 *
 * (C) 2018 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libclipper/clipper.h"
#include "radio.h"

#define CLIP_POINT	0.85
#define DC_CUTOFF	30.0 // Wikipedia: UKW-Rundfunk
#define STEREO_BW	15000.0
#define PILOT_FREQ	19000.0
#define PILOT_BW	5.0
#define PHASE_ERROR_TOLERANCE	3.0	/* ITU-R BS.450-4 S2.2.2.5: +/-3deg */
#define PHASE_ERROR_AVG_SAMPLES	10000	/* samples to average for phase error */

static char freq_name[2][64];

int radio_init(radio_t *radio, int buffer_size, int samplerate, double frequency, const char *tx_wave_file, const char *rx_wave_file, const char *tx_audiodev, const char *rx_audiodev, enum modulation modulation, double bandwidth, double deviation, double modulation_index, double time_constant_us, double volume, int stereo, int rds, int rds2, int sca_67k, int sca_92k, int rds_debug, int rds_verbose)
{
	int rc = -EINVAL;
	double safe_scaler = 1.0;


	/* 
	 * GAIN SCALING & CLIPPER SETUP
	 * ----------------------------
	 * FM broadcast uses pre-emphasis to boost high frequencies before transmission,
	 * which are then de-emphasized on receive. This creates a headroom problem:
	 *
	 * Pre-emphasis boost at 50us (European) / 75us (US):
	 *   500 Hz: +3 dB,  1 kHz: +6 dB,  5 kHz: +14 dB,  15 kHz: +17 dB
	 *
	 * IMPORTANT: Input levels should be reduced to avoid clipping!
	 * - Use -V 0.5 or lower for full-scale input (e.g., test tones)
	 * - Music/speech with typical dynamics usually works at -V 0.8
	 * - The soft clipper will activate on peaks, creating odd harmonics
	 *
	 * 1. Headroom for Pilot/RDS subcarriers
	 */
	if (stereo)
		safe_scaler -= 0.10; /* Reserve 10% for 19 kHz Pilot Tone */
	if (rds || rds2)
		safe_scaler -= 0.05; /* Reserve 5% for 57 kHz RDS Subcarrier */

	/* 
	 * 2. Pre-emphasis Gain Strategy (Standard Broadcast Practice):
	 *    - Normalize for unity gain at 1 kHz reference tone
	 *    - High-frequency peaks (>5 kHz) will hit the soft clipper
	 *    - This maximizes loudness while clipper prevents over-deviation
	 */

	if (safe_scaler < 1.0) {
		LOGP(DRADIO, LOGL_NOTICE, "Auto-scaling input volume by %.3f to reserve headroom for Pilot/RDS.\n", safe_scaler);
		volume *= safe_scaler;
	}

	/* Soft clipper at 1.0 (maximum deviation). Peaks exceeding this are limited. */
	clipper_init(1.0);

	memset(radio, 0, sizeof(*radio));
	radio->buffer_size = buffer_size;
	radio->volume = volume;
	radio->stereo = stereo;
	radio->rds = rds;
	radio->rds2 = rds2;
	radio->sca_67k = sca_67k;
	radio->sca_92k = sca_92k;
	radio->tx_wave_file = tx_wave_file;
	radio->modulation = modulation;
	radio->signal_samplerate = samplerate;
	radio->audio_bandwidth = bandwidth;

	switch (radio->modulation) {
	case MODULATION_FM:
		radio->fm_deviation = deviation;
		radio->signal_bandwidth = deviation + bandwidth;
		if (radio->stereo) {
			radio->signal_bandwidth = deviation + 53000.0;
			radio->audio_bandwidth = STEREO_BW;
		}
		if (radio->rds)
			radio->signal_bandwidth = deviation + 60000.0;
		if (radio->rds2)
			radio->signal_bandwidth = deviation + 80000.0;
		/* SCA extends bandwidth further */
		if (radio->sca_67k)
			radio->signal_bandwidth = deviation + 75000.0;
		if (radio->sca_92k)
			radio->signal_bandwidth = deviation + 100000.0;
		break;
	case MODULATION_AM_DSB:
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		/* level is 1.0, which is full amplitude */
		radio->signal_bandwidth = bandwidth;
		break;
	case MODULATION_NONE:
		LOGP(DRADIO, LOGL_ERROR, "Wrong modulation, please fix!\n");
		goto error;
	}

	if (tx_wave_file) {
		/* open wave file */
		int _samplerate = 0;
		radio->tx_audio_channels = 0;
		rc = wave_create_playback(&radio->wave_tx_play, tx_wave_file, &_samplerate, &radio->tx_audio_channels, 1.0);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to create WAVE playback instance!\n");
			goto error;
		}
		if (radio->tx_audio_channels != 1 && radio->tx_audio_channels != 2)
		{
			LOGP(DRADIO, LOGL_ERROR, "WAVE file must have one or two channels!\n");
			goto error;
		}
		radio->tx_audio_samplerate = _samplerate;
		radio->tx_audio_mode = AUDIO_MODE_WAVEFILE;
	} else if (tx_audiodev) {
#ifdef HAVE_ALSA
		/* open audio device */
		radio->tx_audio_samplerate = 48000;
		radio->tx_audio_channels = (stereo) ? 2 : 1;
		radio->tx_sound = sound_open(SOUND_DIR_PLAY, tx_audiodev, NULL, NULL, NULL, radio->tx_audio_channels, 0.0, radio->tx_audio_samplerate, radio->buffer_size, 1.0, 1.0, 0.0, 2.0);
		if (!radio->tx_sound) {
			rc = -EIO;
			LOGP(DRADIO, LOGL_ERROR, "Failed to open sound device!\n");
			goto error;
		}
		jitter_create(&radio->tx_dejitter[0], "left", radio->tx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		jitter_create(&radio->tx_dejitter[1], "right", radio->tx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		radio->tx_audio_mode = AUDIO_MODE_AUDIODEV;
#else
		rc = -ENOTSUP;
		LOGP(DRADIO, LOGL_ERROR, "No sound card support compiled in!\n");
		goto error;
#endif
	} else {
		int i;
		double phase;
		/* use built-in sample sound */
		radio->tx_audio_samplerate = samplerate;
		radio->tx_audio_channels = (radio->stereo) ? 2 : 1;
		radio->testtone_length = radio->tx_audio_samplerate;
		radio->testtone[0] = calloc(radio->testtone_length * 2, sizeof(sample_t));
		if (!radio->testtone[0]) {
			rc = -ENOMEM;
			LOGP(DRADIO, LOGL_ERROR, "Failed to allocate test sound buffer!\n");
			goto error;
		}
		radio->testtone[1] = radio->testtone[0] + radio->testtone_length;
		/* generate tone */
		phase = 2.0 * M_PI * 1000.0 / radio->tx_audio_samplerate;
		if (radio->stereo) {
			for (i = 0; i < radio->testtone_length / 2; i++) {
				radio->testtone[0][i] = sin(i * phase);
				radio->testtone[1][i] = 0.0;
			}
			for (; i < radio->testtone_length; i++) {
				radio->testtone[0][i] = 0.0;
				radio->testtone[1][i] = sin(i * phase);
			}
		} else {
			for (i = 0; i < radio->testtone_length; i++) {
				radio->testtone[0][i] = sin(i * phase);
			}
		}
		radio->tx_audio_mode = AUDIO_MODE_TESTTONE;
	}

	if (rx_wave_file) {
		/* open wave file */
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (radio->stereo) ? 2 : 1;
		rc = wave_create_record(&radio->wave_rx_rec, rx_wave_file, radio->rx_audio_samplerate, radio->rx_audio_channels, 1.0);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to create WAVE record instance!\n");
			goto error;
		}
		radio->rx_audio_mode |= AUDIO_MODE_WAVEFILE;
	}
	if (rx_audiodev) {
#ifdef HAVE_ALSA
		/* open audio device */
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (stereo) ? 2 : 1;
		/* check if we use same device */
		radio->rx_sound = sound_open(SOUND_DIR_REC, rx_audiodev, NULL, NULL, NULL, radio->rx_audio_channels, 0.0, radio->rx_audio_samplerate, radio->buffer_size, 1.0, 1.0, 0.0, 2.0);
		if (!radio->rx_sound) {
			rc = -EIO;
			LOGP(DRADIO, LOGL_ERROR, "Failed to open sound device!\n");
			goto error;
		}
		jitter_create(&radio->rx_dejitter[0], "left", radio->rx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		jitter_create(&radio->rx_dejitter[1], "right", radio->rx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		radio->rx_audio_mode |= AUDIO_MODE_AUDIODEV;
#else
		rc = -ENOTSUP;
		LOGP(DRADIO, LOGL_ERROR, "No sound card support compiled in!\n");
		goto error;
#endif
	}
	/* if no sink was selected, we use dummy settings */
	if (!rx_wave_file && !rx_audiodev) {
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (stereo) ? 2 : 1;
	}

	/* check if sample rate is too low */
	if (radio->tx_audio_samplerate > radio->signal_samplerate) {
		rc = -EINVAL;
		LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your audio sample rate is %.0f.\n", radio->signal_samplerate, radio->tx_audio_samplerate);
		LOGP(DRADIO, LOGL_ERROR, "Please select a sample rate that is higher or equal the audio sample rate!\n");
		goto error;
	}
	if (radio->rx_audio_samplerate > radio->signal_samplerate) {
		rc = -EINVAL;
		LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your audio sample rate is %.0f.\n", radio->signal_samplerate, radio->rx_audio_samplerate);
		LOGP(DRADIO, LOGL_ERROR, "Please select a sample rate that is higher or equal the audio sample rate!\n");
		goto error;
	}
	if (radio->signal_samplerate < radio->signal_bandwidth * 2 / 0.75) {
		rc = -EINVAL;
		LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your signal's bandwidth %.0f.\n", radio->signal_samplerate, radio->signal_bandwidth);
		LOGP(DRADIO, LOGL_ERROR, "Your signal processing sample rate must be at least one third greater than the signal's double bandwidth. Use at least %.0f.\n", radio->signal_bandwidth * 2.0 / 0.75);
		goto error;
	}

	iir_highpass_init(&radio->tx_dc_removal[0], DC_CUTOFF, radio->tx_audio_samplerate, 1);
	iir_highpass_init(&radio->tx_dc_removal[1], DC_CUTOFF, radio->tx_audio_samplerate, 1);

	/* init DC blocker state */
	radio->tx_dc_prev_x[0] = 0.0;
	radio->tx_dc_prev_x[1] = 0.0;
	radio->tx_dc_prev_y[0] = 0.0;
	radio->tx_dc_prev_y[1] = 0.0;

	/* stereo pilot tone phase */
	radio->pilot_phasestep = 2.0 * M_PI * PILOT_FREQ / radio->signal_samplerate;

	/* stere decoding filters */
	iir_lowpass_init(&radio->rx_lp_pilot_I, PILOT_BW, radio->signal_samplerate, 2);
        iir_lowpass_init(&radio->rx_lp_pilot_Q, PILOT_BW, radio->signal_samplerate, 2);
        iir_lowpass_init(&radio->rx_lp_sum, STEREO_BW, radio->signal_samplerate, 2);
        iir_lowpass_init(&radio->rx_lp_diff, STEREO_BW, radio->signal_samplerate, 2);

	/* init sample rate conversion, use complete bandwidth for resample filter */
	/* 
	 * RECONSTRUCTION/ANTI-ALIASING FILTER
	 * -----------------------------------
	 * We use a strict 15kHz cutoff (Standard FM Bandwidth) for the upsampler.
	 * 
	 * JUSTIFICATION:
	 * 1. The previous default (audio_samplerate / 2) allowed ultrasonic images to pass 
	 *    through the 2nd order filter.
	 * 2. These ultrasonic images (e.g. at 24kHz+) folded back into the audible band 
	 *    during SDR modulation, creating "8-bit like" hiss and intermodulation noise.
	 * 3. 15kHz creates a clean, hard stop before the 19kHz stereo pilot, protecting
	 *    the pilot from interference and the audio from aliasing.
	 * 
	 * Note: Filter order was also increased to 4 in libsamplerate/samplerate.c
	 */
	rc = init_samplerate(&radio->tx_resampler[0], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;
	rc = init_samplerate(&radio->tx_resampler[1], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;
	rc = init_samplerate(&radio->rx_resampler[0], radio->rx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;
	rc = init_samplerate(&radio->rx_resampler[1], radio->rx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;

	/* init display of wave form */
	sprintf(freq_name[0], "%.4f MHz", frequency / 1e6);
	display_wave_init(&radio->dispwav[0], radio->rx_audio_samplerate, freq_name[0]);

	/* init filters (using signal sample rate) */
	switch (radio->modulation) {
	case MODULATION_FM:
		if (time_constant_us > 0.0) {
			radio->emphasis = 1;
			/* time constant */
			LOGP(DRADIO, LOGL_INFO, "Using emphasis cut-off at %.0f Hz.\n", timeconstant2cutoff(time_constant_us));
			rc = init_emphasis(&radio->fm_emphasis[0], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
			if (rc < 0)
				goto error;
			rc = init_emphasis(&radio->fm_emphasis[1], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
			if (rc < 0)
				goto error;
		}
		rc = fm_mod_init(&radio->fm_mod, radio->signal_samplerate, 0.0, 1.0);
		if (rc < 0)
			goto error;
		rc = fm_demod_init(&radio->fm_demod, radio->signal_samplerate, 0.0, 2 * radio->signal_bandwidth);
		if (rc < 0)
			goto error;
		if (stereo) {
			sprintf(freq_name[0], "%.4f MHz left", frequency / 1e6);
			sprintf(freq_name[1], "%.4f MHz right", frequency / 1e6);
			display_wave_init(&radio->dispwav[1], samplerate, freq_name[1]);
		}
		/* Initialize RDS encoder if enabled */
		if (rds || rds2) {
			const char *ptyn = "OsmoPtyn"; /* PTYN override */
			rds_encoder_init(&radio->rds_enc, radio->signal_samplerate,
				0x1234, "OSMO RDS", "osmocom-analog FM Radio", 0, ptyn);
			rds_decoder_init(&radio->rds_dec, radio->signal_samplerate, rds_debug, rds_verbose);
			
			/*
			 * TODO: RDS2 Encoder Initialization (IEC 62106-2:2021)
			 * ----------------------------------------------------
			 * If rds2 flag is set, initialize additional encoder for streams 2-4:
			 *
			 *   if (rds2) {
			 *       rds2_encoder_init(&radio->rds2_enc, radio->signal_samplerate);
			 *   }
			 *
			 * The rds2_encoder_t would handle:
			 * - Stream 2:  66.5 kHz subcarrier (independent NCO)
			 * - Stream 3:  71.25 kHz subcarrier (independent NCO)
			 * - Stream 4:  76 kHz subcarrier (4 x pilot, can be locked)
			 * - Group Type C encoding for extended data
			 * - UTF-8 Extended RadioText (128 bytes)
			 * - RDS2 File Transfer (RFT) protocol
			 *
			 * Note: The radio_t struct would need: rds2_encoder_t rds2_enc;
			 * See rds.c for stub implementation.
			 */
		}
		/* Initialize SCA encoder/decoder if enabled */
		if (sca_67k || sca_92k) {
			sca_encoder_init(&radio->sca_enc, radio->signal_samplerate, sca_67k, sca_92k);
			sca_decoder_init(&radio->sca_dec, radio->signal_samplerate, sca_67k, sca_92k);
		}
		break;
	case MODULATION_AM_DSB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		/* modulation index 0.0 = no envelope, bias 1.0
		 * modulation index 1.0 = envelope +-0.5, bias 0.5
		 * modulation index 0.5 = envelope +-0.25, bias 0.75
		 */
		double gain = modulation_index / 2.0;
		double bias = 1.0 - gain;
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, gain, bias);
		if (rc < 0)
			goto error;
		rc = am_demod_init(&radio->am_demod, radio->signal_samplerate, 0.0, radio->signal_bandwidth, 1.0 / modulation_index);
		if (rc < 0)
			goto error;
		break;
	case MODULATION_AM_USB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, 1.0, 0.0);
		if (rc < 0)
			goto error;
		break;
	case MODULATION_AM_LSB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, 1.0, 0.0);
		if (rc < 0)
			goto error;
		break;
	default:
		break;
	}
	
	if (radio->tx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio source is %.0f Hz.\n", radio->tx_audio_samplerate / 2.0);
	if (radio->rx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio sink is %.0f Hz.\n", radio->rx_audio_samplerate / 2.0);
	LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio signal is %.0f Hz.\n", radio->audio_bandwidth);
	LOGP(DRADIO, LOGL_INFO, "Bandwidth of modulated signal is %.0f Hz.\n", radio->signal_bandwidth);
	if (radio->tx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Sample rate of audio source is %.0f Hz.\n", radio->tx_audio_samplerate);
	if (radio->rx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Sample rate of audio sink is %.0f Hz.\n", radio->rx_audio_samplerate);
	LOGP(DRADIO, LOGL_INFO, "Sample rate of signal is %.0f Hz.\n", radio->signal_samplerate);

	/* one or two audio channels */
	if (radio->tx_audio_channels != 1 && radio->tx_audio_channels != 2)
	{
		LOGP(DRADIO, LOGL_ERROR, "Wrong number of audio channels, please fix!\n");
		goto error;
	}

	/* audio buffers: how many sample for audio (rounded down) */
	int tx_size = (int)((double)buffer_size / radio->tx_resampler[0].factor);
	int rx_size = (int)((double)buffer_size / radio->rx_resampler[0].factor);
	if (tx_size > rx_size)
		radio->audio_buffer_size = tx_size;
	else
		radio->audio_buffer_size = rx_size;
	radio->audio_buffer = calloc(radio->audio_buffer_size * 2, sizeof(*radio->audio_buffer));
	if (!radio->audio_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* signal buffers */
	radio->signal_buffer_size = buffer_size;
	radio->signal_buffer = calloc(radio->signal_buffer_size * 3, sizeof(*radio->signal_buffer));
	radio->signal_power_buffer = calloc(radio->signal_buffer_size, sizeof(*radio->signal_power_buffer));
	if (!radio->signal_buffer || !radio->signal_power_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* temporary I/Q/carrier buffers, used while demodulating */
	radio->I_buffer = calloc(buffer_size, sizeof(*radio->I_buffer));
	radio->Q_buffer = calloc(buffer_size, sizeof(*radio->Q_buffer));
	radio->carrier_buffer = calloc(buffer_size, sizeof(*radio->carrier_buffer));
	if (!radio->I_buffer || !radio->Q_buffer || !radio->carrier_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	return 0;

error:
	radio_exit(radio);
	return rc;
}

void radio_exit(radio_t *radio)
{
	if (radio->audio_buffer) {
		free(radio->audio_buffer);
		radio->audio_buffer = NULL;
	}
	if (radio->signal_buffer) {
		free(radio->signal_buffer);
		radio->signal_buffer = NULL;
	}
	if (radio->signal_power_buffer) {
		free(radio->signal_power_buffer);
		radio->signal_power_buffer = NULL;
	}
	if (radio->I_buffer) {
		free(radio->I_buffer);
		radio->I_buffer = NULL;
	}
	if (radio->Q_buffer) {
		free(radio->Q_buffer);
		radio->Q_buffer = NULL;
	}
	if (radio->carrier_buffer) {
		free(radio->carrier_buffer);
		radio->carrier_buffer = NULL;
	}
	if (radio->tx_audio_mode == AUDIO_MODE_WAVEFILE) {
		wave_destroy_playback(&radio->wave_tx_play);
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if ((radio->rx_audio_mode & AUDIO_MODE_WAVEFILE)) {
		wave_destroy_record(&radio->wave_rx_rec);
		radio->rx_audio_mode = AUDIO_MODE_NONE;
	}
#ifdef HAVE_ALSA
	if (radio->tx_sound) {
		sound_close(radio->tx_sound);
		/* if same device was used */
		if (radio->tx_sound == radio->rx_sound)
			radio->rx_sound = NULL;
		radio->tx_sound = NULL;
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if (radio->rx_sound) {
		sound_close(radio->rx_sound);
		radio->rx_sound = NULL;
		radio->rx_audio_mode = AUDIO_MODE_NONE;
	}
#endif
	jitter_destroy(&radio->tx_dejitter[0]);
	jitter_destroy(&radio->tx_dejitter[1]);
	jitter_destroy(&radio->rx_dejitter[0]);
	jitter_destroy(&radio->rx_dejitter[1]);
	if (radio->tx_audio_mode == AUDIO_MODE_TESTTONE) {
		free(radio->testtone[0]);
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if (radio->modulation == MODULATION_FM)
		fm_mod_exit(&radio->fm_mod);
	else
		am_mod_exit(&radio->am_mod);
}

int radio_start(radio_t __attribute__((unused)) *radio)
{
#ifdef HAVE_ALSA
	int rc;

	/* start rx sound */
	if (radio->rx_sound) {
		rc = sound_start(radio->rx_sound);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to start receiving from audio device..\n");
			return rc;
		}
	}

	/* start tx sound, if different device */
	if (radio->tx_sound && radio->tx_sound != radio->rx_sound)  {
		rc = sound_start(radio->tx_sound);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to start transmitting to audio device..\n");
			return rc;
		}
	}
#endif

	return 0;
}

int radio_tx(radio_t *radio, float *baseband, int signal_num)
{
	int i;
	int __attribute__((unused)) rc;
	int audio_num;
	sample_t *audio_samples[2];
	sample_t *signal_samples[3];
	uint8_t *signal_power;
#ifdef HAVE_ALSA
	jitter_frame_t *jf;
#endif

	if (signal_num > radio->buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > buffer_size, please fix!.\n");
		abort();
	}

	/* audio buffers: how many sample for audio (rounded down) */
	audio_num = (int)((double)signal_num / radio->tx_resampler[0].factor);
	if (audio_num > radio->audio_buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "audio_num > audio_buffer_size, please fix!.\n");
		abort();
	}
	audio_samples[0] = radio->audio_buffer;
	audio_samples[1] = radio->audio_buffer + radio->audio_buffer_size;

	/* signal buffers: a bit more samples to be safe */
	signal_num = (int)((double)audio_num * radio->tx_resampler[0].factor + 0.5) + 10;
	if (signal_num > radio->signal_buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > signal_buffer_size, please fix!.\n");
		abort();
	}
	signal_samples[0] = radio->signal_buffer;
	signal_samples[1] = radio->signal_buffer + radio->signal_buffer_size;
	signal_samples[2] = radio->signal_buffer + radio->signal_buffer_size * 2;
	signal_power = radio->signal_power_buffer;

	/* get audio to be sent */
	switch (radio->tx_audio_mode) {
	case AUDIO_MODE_WAVEFILE:
		wave_read(&radio->wave_tx_play, audio_samples, audio_num);
		
		if (!radio->wave_tx_play.left) {
			int rc;
			int _samplerate = 0;
			wave_destroy_playback(&radio->wave_tx_play);
			rc = wave_create_playback(&radio->wave_tx_play, radio->tx_wave_file, &_samplerate, &radio->tx_audio_channels, 1.0);
			if (rc < 0) {
				LOGP(DRADIO, LOGL_ERROR, "Failed to re-open wave file.\n");
				return rc;
			}
		}
		break;
#ifdef HAVE_ALSA
	case AUDIO_MODE_AUDIODEV:
		rc = sound_read(radio->tx_sound, audio_samples, radio->audio_buffer_size, radio->tx_audio_channels, NULL);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to read from sound device (rc = %d)!\n", audio_num);
			if (rc == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
		jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)audio_samples[0], rc * sizeof(*(audio_samples[0])), 0, radio->tx_sequence[0], radio->tx_timestamp[0], 123);
		if (jf)
			jitter_save(&radio->tx_dejitter[0], jf);
		radio->tx_sequence[0] += 1;
		radio->tx_timestamp[0] += rc;
		jitter_load_samples(&radio->tx_dejitter[0], (uint8_t *)audio_samples[0], audio_num, sizeof(*(audio_samples[0])), NULL, NULL);
		if (radio->tx_audio_channels == 2) {
			jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)audio_samples[1], rc * sizeof(*(audio_samples[1])), 0, radio->tx_sequence[1], radio->tx_timestamp[1], 123);
			if (jf)
				jitter_save(&radio->tx_dejitter[1], jf);
			radio->tx_sequence[1] += 1;
			radio->tx_timestamp[1] += rc;
			jitter_load_samples(&radio->tx_dejitter[1], (uint8_t *)audio_samples[1], audio_num, sizeof(*(audio_samples[1])), NULL, NULL);
		}
		break;
#endif
	case AUDIO_MODE_TESTTONE:
		for (i = 0; i < audio_num; i++) {
			audio_samples[0][i] = radio->testtone[0][radio->testtone_pos];
			audio_samples[1][i] = radio->testtone[1][radio->testtone_pos];
			radio->testtone_pos = (radio->testtone_pos + 1) % radio->testtone_length;
		}
		break;
	default:
		LOGP(DRADIO, LOGL_ERROR, "Wrong audio mode, please fix!\n");
		return -EINVAL;
	}





	/* convert mono/stereo, generate differential signal */
	/* (Skip this if we want pure clean signal, but let's keep it to test stereo proc) */
	if (radio->stereo && radio->tx_audio_channels == 1) {
		/* mono to stereo: scale sum to 90%, differential signal is 0 */
		for (i = 0; i < audio_num; i++) {
			audio_samples[0][i] *= 0.9;
			audio_samples[1][i] = 0.0;
		}
	}
	if (radio->stereo && radio->tx_audio_channels == 2) {
		/* stereo: sum is 90%, diffential is 90% */
		double left, right;
		for (i = 0; i < audio_num; i++) {
			left = audio_samples[0][i];
			right = audio_samples[1][i];
			audio_samples[0][i] = (left + right) * 0.45;
			audio_samples[1][i] = (left - right) * 0.45;
		}
	}
	if (!radio->stereo && radio->tx_audio_channels == 2) {
		/* stereo to mono: sum both channel */
		for (i = 0; i < audio_num; i++)
			audio_samples[0][i] = (audio_samples[0][i] + audio_samples[1][i]) / 2.0;
	}



	/* remove DC */
	// iir_process(&radio->tx_dc_removal[0], audio_samples[0], audio_num);
	// if (radio->stereo)
	// 	iir_process(&radio->tx_dc_removal[1], audio_samples[1], audio_num);
	
	/* 
	 * DC OFFSET REMOVAL
	 * -----------------
	 * We use a recursive DC blocker filter: y[n] = x[n] - x[n-1] + R * y[n-1]
	 * R = 0.9995 corresponds to a cutoff of approx 10Hz at 48kHz.
	 * 
	 * JUSTIFICATION:
	 * 1. The previous IIR highpass filter from libfilter was ineffective, leaving a DC 
	 *    offset (up to 0.02) in the signal.
	 * 2. This DC offset caused asymmetric clipping and generated a strong 2nd harmonic 
	 *    distortion (2 kHz tone from a 1 kHz fundamental).
	 * 3. This manual implementation ensures the signal is centered at 0.0 before modulation.
	 */
	{
		double R = 0.9995;
		double x, y;
		int i;
		
		/* Channel 0 (Left/Mono) */
		for (i = 0; i < audio_num; i++) {
			x = audio_samples[0][i];
			y = x - radio->tx_dc_prev_x[0] + R * radio->tx_dc_prev_y[0];
			radio->tx_dc_prev_x[0] = x;
			radio->tx_dc_prev_y[0] = y;
			audio_samples[0][i] = y;
		}

		/* Channel 1 (Right) */
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++) {
				x = audio_samples[1][i];
				y = x - radio->tx_dc_prev_x[1] + R * radio->tx_dc_prev_y[1];
				radio->tx_dc_prev_x[1] = x;
				radio->tx_dc_prev_y[1] = y;
				audio_samples[1][i] = y;
			}
		}
	}



	/* gain volume */
	if (radio->volume != 1.0) {
		for (i = 0; i < audio_num; i++)
			audio_samples[0][i] *= radio->volume;
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++)
				audio_samples[1][i] *= radio->volume;
		}
	}

	/* upsample */
	signal_num = samplerate_upsample_output_num(&radio->tx_resampler[0], audio_num);
	samplerate_upsample(&radio->tx_resampler[0], audio_samples[0], audio_num, signal_samples[0], signal_num);
	if (radio->stereo)
		samplerate_upsample(&radio->tx_resampler[1], audio_samples[1], audio_num, signal_samples[1], signal_num);

	/* prepare baseband */
	memset(baseband, 0, sizeof(float) * 2 * signal_num);
	memset(signal_power, 1, signal_num);

	/* filter audio (remove DC, remove high frequencies, pre-emphasis)
	 * and modulate */
	switch (radio->modulation) {
	case MODULATION_FM:
		if (radio->emphasis)
			pre_emphasis(&radio->fm_emphasis[0], signal_samples[0], signal_num);


		
		clipper_process(signal_samples[0], signal_num);
		if (radio->stereo) {
			if (radio->emphasis)
				pre_emphasis(&radio->fm_emphasis[1], signal_samples[1], signal_num);
			clipper_process(signal_samples[1], signal_num);
		}
		
		/* Advance pilot phase if Stereo OR RDS is enabled */
		if (radio->stereo || radio->rds || radio->rds2) {
			double phasestep = radio->pilot_phasestep;
			double phase = radio->tx_pilot_phase;
			double start_phase = phase; /* Capture start phase for RDS */
			
			for (i = 0; i < signal_num; i++) {
				/* Add pilot tone only if Stereo */
				if (radio->stereo) {
					/* Add pilot (19 kHz) */
					signal_samples[0][i] += sin(phase) * 0.1;
					/* Add stereo diff (mixed with 38 kHz) */
					signal_samples[0][i] += signal_samples[1][i] * sin(phase * 2);
				}
				
				phase += phasestep;
				if (phase >= 2.0 * M_PI)
					phase -= 2.0 * M_PI;
			}
			radio->tx_pilot_phase = phase;

			/* Add RDS subcarrier if enabled (phase-locked to pilot at 3x frequency) */
			if (radio->rds || radio->rds2) {
				rds_encoder_process(&radio->rds_enc, signal_samples[0], signal_num,
						    start_phase, radio->pilot_phasestep);
				
				/*
				 * TODO: RDS2 Additional Subcarriers (IEC 62106-2:2021)
				 * ---------------------------------------------------
				 * If rds2 flag is set, we should add 3 more BPSK streams:
				 *
				 *   Stream 2:  66.5 kHz   (free-running, NOT pilot harmonic)
				 *   Stream 3:  71.25 kHz  (free-running, NOT pilot harmonic)
				 *   Stream 4:  76.0 kHz   (4 x pilot, CAN be phase-locked)
				 *
				 * Implementation would look like:
				 *   if (radio->rds2) {
				 *       rds2_encoder_process(&radio->rds2_enc, signal_samples[0],
				 *                            signal_num, start_phase, 
				 *                            radio->pilot_phasestep);
				 *   }
				 *
				 * Note: Streams 2 & 3 require independent NCOs because:
				 *   - 66.5 kHz = 19 kHz x 3.5   (not integer harmonic)
				 *   - 71.25 kHz = 19 kHz x 3.75 (not integer harmonic)
				 *
				 * Stream 4 can use: sin(pilot_phase * 4) for 76 kHz
				 *
				 * Injection level: ~2-5% per stream (same as original RDS)
				 * Total RDS2 injection: up to 4 x 5% = 20% (aggressive)
				 *
				 * See rds.c for rds2_encoder_t stub structure.
				 */
			}
		}
		for (i = 0; i < signal_num; i++)
			signal_samples[0][i] *= radio->fm_deviation;
		fm_modulate_complex(&radio->fm_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	case MODULATION_AM_DSB:
		/* also clip to prevent overshooting after audio filtering */
		clipper_process(signal_samples[0], signal_num);
		iir_process(&radio->tx_am_bw_limit, signal_samples[0], signal_num);
		am_modulate_complex(&radio->am_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		/* also clip to prevent overshooting after audio filtering */
		clipper_process(signal_samples[0], signal_num);
		iir_process(&radio->tx_am_bw_limit, signal_samples[0], signal_num);
		am_modulate_complex(&radio->am_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	default:
		break;
	}

	return signal_num;
}

int radio_rx(radio_t *radio, float *baseband, int signal_num)
{
	int i;
	int audio_num;
	sample_t *samples[3];
	double p;
#ifdef HAVE_ALSA
	jitter_frame_t *jf;
#endif

	if (signal_num > radio->buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > buffer_size, please fix!.\n");
		abort();
	}

	if (signal_num > radio->signal_buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > signal_buffer_size, please fix!.\n");
		abort();
	}
	samples[0] = radio->signal_buffer;
	samples[1] = radio->signal_buffer + radio->signal_buffer_size;
	samples[2] = radio->signal_buffer + radio->signal_buffer_size * 2;

	switch (radio->modulation) {
	case MODULATION_FM:
		fm_demodulate_complex(&radio->fm_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer);
		for (i = 0; i < signal_num; i++)
			samples[0][i] /= radio->fm_deviation;
		/* Decode RDS from FM baseband if enabled */
		if (radio->rds || radio->rds2) {
			/* Pass pilot phase WITH phase offset compensation (same as stereo decoder)
			 * The rx_pll_freq_offset compensates for IIR filter group delay and
			 * TX/RX frequency offset. RDS at 57 kHz = 3 x pilot needs 3x the offset.
			 */
			double rds_pilot_phase = radio->rx_pilot_phase - radio->rx_pll_freq_offset;
			rds_decoder_process(&radio->rds_dec, samples[0], signal_num,
			                    rds_pilot_phase, radio->pilot_phasestep);
		}
		if (radio->stereo) {
			/*
			 * FM STEREO DEMODULATION (Pilot-Tone System)
			 * ==========================================
			 * FM stereo uses DSB-SC modulation at 38 kHz (2x the 19 kHz pilot).
			 * The stereo difference signal (L-R) is modulated onto this subcarrier.
			 *
			 * Challenge: The narrow-band IIR pilot filter (5 Hz BW) needed to extract
			 * the 19 kHz pilot from the FM baseband introduces significant group delay.
			 * This makes per-sample phase tracking unreliable (the measured phase drifts
			 * continuously as our local oscillator advances).
			 *
			 * Solution: Block-level phase offset tracking
			 * 1. Demodulate using local 38 kHz oscillator with a fixed phase offset
			 * 2. Measure the actual phase offset once per block (from filtered I/Q)
			 * 3. Slowly track the average offset (~1 second time constant)
			 *
			 * The tracked offset compensates for:
			 * - IIR filter group delay (~13deg at 5 Hz BW)
			 * - Any transmitter/receiver frequency offset
			 * - SDR sample rate inaccuracies
			 */
			
			/* Step 1: Stereo demodulation using local 38 kHz oscillator */
			/* Apply tracked phase offset compensation */
			double phase_offset = radio->rx_pll_freq_offset;
			p = radio->rx_pilot_phase;
			for (i = 0; i < signal_num; i++) {
				/* 38 kHz carrier = 2x pilot phase, minus compensation offset */
				double carrier_38k = (p - phase_offset) * 2.0;
				samples[1][i] = samples[0][i] * sin(carrier_38k) * 2.0;
				
				p += radio->pilot_phasestep;
				if (p >= 2.0 * M_PI)
					p -= 2.0 * M_PI;
			}
			
			/* Step 2: Measure phase offset using I/Q mixing at end of block */
			/* We only need to compute I and Q for phase detection - reuse samples[2] */
			/* First compute cos (I) component across block */
			p = radio->rx_pilot_phase;
			for (i = 0; i < signal_num; i++) {
				samples[2][i] = samples[0][i] * cos(p);
				p += radio->pilot_phasestep;
				if (p >= 2.0 * M_PI)
					p -= 2.0 * M_PI;
			}
			iir_process(&radio->rx_lp_pilot_I, samples[2], signal_num);
			double I_end = samples[2][signal_num - 1];
			
			/* Then compute sin (Q) component across block */
			p = radio->rx_pilot_phase;
			for (i = 0; i < signal_num; i++) {
				samples[2][i] = samples[0][i] * sin(p);
				p += radio->pilot_phasestep;
				if (p >= 2.0 * M_PI)
					p -= 2.0 * M_PI;
			}
			iir_process(&radio->rx_lp_pilot_Q, samples[2], signal_num);
			double Q_end = samples[2][signal_num - 1];
			
			double pilot_mag = sqrt(I_end * I_end + Q_end * Q_end);
			
			if (pilot_mag > 1e-9) {
				/* Measured phase = offset between our oscillator and received pilot */
				double measured_offset = atan2(Q_end, I_end);
				
				/* Normalize to -45..+45 degrees for 90deg periodicity of sin(2x) */
				while (measured_offset > M_PI/4) measured_offset -= M_PI/2;
				while (measured_offset < -M_PI/4) measured_offset += M_PI/2;
				
				/* Track average offset with slow IIR (time constant ~1 second) */
				double alpha = 1.0 / (radio->signal_samplerate * 1.0);  /* 1 second TC */
				radio->rx_pll_freq_offset += alpha * signal_num * (measured_offset - radio->rx_pll_freq_offset);
			}
			
			/* Update pilot phase for next block */
			radio->rx_pilot_phase = p;
			
			/* Diagnostics (every ~1 second) */
			{
				static int diag_count = 0;
				diag_count += signal_num;
				if (diag_count >= 1000000) {
					diag_count = 0;
					double offset_deg = radio->rx_pll_freq_offset * (180.0 / M_PI);
					// LOGP(DRADIO, LOGL_DEBUG, "Stereo: offset=%.1fdeg pilot=%.6f\n", offset_deg, pilot_mag);
				}
			}
			
			/* Filter stereo channels to match bandwidth */
			iir_process(&radio->rx_lp_sum, samples[0], signal_num);
			iir_process(&radio->rx_lp_diff, samples[1], signal_num);
		}
		if (radio->emphasis) {
			dc_filter(&radio->fm_emphasis[0], samples[0], signal_num);
			de_emphasis(&radio->fm_emphasis[0], samples[0], signal_num);
			if (radio->stereo) {
				dc_filter(&radio->fm_emphasis[1], samples[1], signal_num);
				de_emphasis(&radio->fm_emphasis[1], samples[1], signal_num);
			}
		}
		break;
	case MODULATION_AM_DSB:
		am_demodulate_complex(&radio->am_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer, radio->carrier_buffer);
		break;
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		am_demodulate_complex(&radio->am_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer, radio->carrier_buffer);
		break;
	default:
		break;
	}

	/* downsample */
	audio_num = samplerate_downsample(&radio->rx_resampler[0], samples[0], signal_num);
	if (radio->stereo)
		samplerate_downsample(&radio->rx_resampler[1], samples[1], signal_num);

	/* dampen volume */
	if (radio->volume != 1.0) {
		for (i = 0; i < audio_num; i++)
			samples[0][i] /= radio->volume;
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++)
				samples[1][i] /= radio->volume;
		}
	}

	/* convert mono/stereo, (from differential signal) */
	if (radio->stereo && radio->rx_audio_channels == 1) {
		/* stereo to mono */
		for (i = 0; i < audio_num; i++) {
			samples[0][i] = (samples[0][i] + samples[1][i]) / 2.0;
		}
	}
	if (radio->stereo && radio->rx_audio_channels == 2) {
		/* stereo from differential */
		double sum, diff;
		for (i = 0; i < audio_num; i++) {
			sum = samples[0][i];
			diff = samples[1][i];
			samples[0][i] = sum + diff / 2.0;
			samples[1][i] = sum - diff / 2.0;
		}
	}
	if (!radio->stereo && radio->rx_audio_channels == 2) {
		/* mono to stereo: clone channel */
		for (i = 0; i < audio_num; i++)
			samples[1][i] = samples[0][i];
	}

	/* display wave */
	display_wave(&radio->dispwav[0], samples[0], audio_num, 1.0);
	if (radio->stereo && radio->rx_audio_channels == 2)
		display_wave(&radio->dispwav[1], samples[1], audio_num, 1.0);

	/* store received audio */
	if ((radio->rx_audio_mode & AUDIO_MODE_WAVEFILE))
		wave_write(&radio->wave_rx_rec, samples, audio_num);
#ifdef HAVE_ALSA
	if ((radio->rx_audio_mode & AUDIO_MODE_AUDIODEV)) {
		jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)samples[0], audio_num * sizeof(*(samples[0])), 0, radio->rx_sequence[0], radio->rx_timestamp[0], 123);
		if (jf)
			jitter_save(&radio->rx_dejitter[0], jf);
		radio->rx_sequence[0] += 1;
		radio->rx_timestamp[0] += audio_num;
		if (radio->rx_audio_channels == 2) {
			jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)samples[1], audio_num * sizeof(*(samples[1])), 0, radio->rx_sequence[1], radio->rx_timestamp[1], 123);
			if (jf)
				jitter_save(&radio->rx_dejitter[1], jf);
			radio->rx_sequence[1] += 1;
			radio->rx_timestamp[1] += audio_num;
		}
		audio_num = sound_get_tosend(radio->rx_sound, radio->signal_buffer_size);
		if (audio_num < 0) {
			LOGP(DDSP, LOGL_ERROR, "Failed to get number of samples in buffer (rc = %d)!\n", audio_num);
			if (audio_num == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
		jitter_load_samples(&radio->rx_dejitter[0], (uint8_t *)samples[0], audio_num, sizeof(*samples), NULL, NULL);
		if (radio->rx_audio_channels == 2)
			jitter_load_samples(&radio->rx_dejitter[1], (uint8_t *)samples[1], audio_num, sizeof(*samples), NULL, NULL);
		printf("channels=%d num=%d\n", radio->rx_audio_channels, audio_num);
		audio_num = sound_write(radio->rx_sound, samples, NULL, audio_num, NULL, NULL, radio->rx_audio_channels);
		if (audio_num < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to write to sound device (rc = %d)!\n", audio_num);
			if (audio_num == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
	}
#endif

	return signal_num;
}

