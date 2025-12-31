/*
 * SCA (Subsidiary Communications Authorization) encoder/decoder
 *
 * (C) 2024 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * Implements FM subcarrier encoding/decoding per FCC 47 CFR 73.319
 * - Standard frequencies: 67 kHz, 92 kHz
 * - FM modulation with +/-7.5 kHz deviation
 * - Used for: reading services, background music, data services
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sca.h"
#include "../liblogging/logging.h"
#include "../libfm/fm.h"

/* Initialize a single SCA channel */
static int sca_channel_init(sca_channel_t *ch, double samplerate, 
			    double frequency, double injection, int enabled)
{
	memset(ch, 0, sizeof(*ch));
	
	ch->samplerate = samplerate;
	ch->frequency = frequency;
	ch->deviation = SCA_DEVIATION;
	ch->injection = injection;
	ch->enabled = enabled;
	
	ch->phasestep = 2.0 * M_PI * frequency / samplerate;
	
	/* Low-pass filter for I/Q: cutoff ~10 kHz (wider than RDS) */
	ch->lpf_alpha = 10000.0 / samplerate;
	
	return 0;
}

/* ==================== SCA ENCODER ==================== */

int sca_encoder_init(sca_encoder_t *sca, double samplerate, int enable_67k, int enable_92k)
{
	memset(sca, 0, sizeof(*sca));
	sca->samplerate = samplerate;
	
	if (enable_67k) {
		sca_channel_init(&sca->ch67, samplerate, SCA_FREQ_67K, 
				 SCA_INJECTION_67K, 1);
		LOGP(DRADIO, LOGL_INFO, "SCA encoder: 67 kHz channel enabled\n");
	}
	
	if (enable_92k) {
		sca_channel_init(&sca->ch92, samplerate, SCA_FREQ_92K,
				 SCA_INJECTION_92K, 1);
		LOGP(DRADIO, LOGL_INFO, "SCA encoder: 92 kHz channel enabled\n");
	}
	
	return 0;
}

/* FM modulate audio onto subcarrier and add to baseband */
static void sca_modulate_channel(sca_channel_t *ch, sample_t *audio, 
				 sample_t *baseband, int num)
{
	int i;
	
	if (!ch->enabled || !audio)
		return;
	
	for (i = 0; i < num; i++) {
		/* FM modulation: frequency = fc + deviation * audio */
		double freq_offset = ch->deviation * audio[i] / ch->samplerate;
		double instantaneous_phase = ch->phase + 2.0 * M_PI * freq_offset;
		
		/* Generate subcarrier */
		double subcarrier;
		if (fm_fast_math_enabled()) {
			double sc_sin, sc_cos;
			fm_fast_sincos(instantaneous_phase * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
			subcarrier = sc_sin;
		} else {
			subcarrier = sin(instantaneous_phase);
		}
		
		/* Add to baseband with injection level */
		baseband[i] += subcarrier * ch->injection;
		
		/* Advance phase */
		ch->phase += ch->phasestep;
		if (ch->phase >= 2.0 * M_PI)
			ch->phase -= 2.0 * M_PI;
	}
}

void sca_encoder_process(sca_encoder_t *sca, sample_t *audio_67k, sample_t *audio_92k,
			 sample_t *baseband, int num)
{
	sca_modulate_channel(&sca->ch67, audio_67k, baseband, num);
	sca_modulate_channel(&sca->ch92, audio_92k, baseband, num);
}

void sca_encoder_exit(sca_encoder_t *sca)
{
	(void)sca;
}

/* ==================== SCA DECODER ==================== */

int sca_decoder_init(sca_decoder_t *sca, double samplerate, int enable_67k, int enable_92k)
{
	memset(sca, 0, sizeof(*sca));
	sca->samplerate = samplerate;
	
	if (enable_67k) {
		sca_channel_init(&sca->ch67, samplerate, SCA_FREQ_67K, 0, 1);
		LOGP(DRADIO, LOGL_INFO, "SCA decoder: 67 kHz channel enabled\n");
	}
	
	if (enable_92k) {
		sca_channel_init(&sca->ch92, samplerate, SCA_FREQ_92K, 0, 1);
		LOGP(DRADIO, LOGL_INFO, "SCA decoder: 92 kHz channel enabled\n");
	}
	
	return 0;
}

/* FM demodulate subcarrier from baseband */
static void sca_demodulate_channel(sca_channel_t *ch, sample_t *baseband,
				   sample_t *audio, int num)
{
	int i;
	
	if (!ch->enabled) {
		if (audio)
			memset(audio, 0, sizeof(sample_t) * num);
		return;
	}
	
	for (i = 0; i < num; i++) {
		double sample = baseband[i];
		
		/* Mix with subcarrier to get I and Q */
		double sc_sin, sc_cos;
		if (fm_fast_math_enabled()) {
			fm_fast_sincos(ch->phase * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
		} else {
			sincos(ch->phase, &sc_sin, &sc_cos);
		}
		double i_mix = sample * sc_cos;
		double q_mix = sample * sc_sin;
		
		/* Low-pass filter I and Q */
		ch->lpf_i += ch->lpf_alpha * (i_mix - ch->lpf_i);
		ch->lpf_q += ch->lpf_alpha * (q_mix - ch->lpf_q);
		
		/* FM demodulation: differentiate phase */
		double current_phase = atan2(ch->lpf_q, ch->lpf_i);
		double phase_diff = current_phase - ch->last_phase;
		
		/* Unwrap phase */
		if (phase_diff > M_PI) phase_diff -= 2.0 * M_PI;
		if (phase_diff < -M_PI) phase_diff += 2.0 * M_PI;
		
		ch->last_phase = current_phase;
		
		/* Convert to audio: phase_diff * samplerate / (2pi * deviation) */
		if (audio)
			audio[i] = phase_diff * ch->samplerate / (2.0 * M_PI * ch->deviation);
		
		/* Advance phase */
		ch->phase += ch->phasestep;
		if (ch->phase >= 2.0 * M_PI)
			ch->phase -= 2.0 * M_PI;
	}
}

void sca_decoder_process(sca_decoder_t *sca, sample_t *baseband,
			 sample_t *audio_67k, sample_t *audio_92k, int num)
{
	sca_demodulate_channel(&sca->ch67, baseband, audio_67k, num);
	sca_demodulate_channel(&sca->ch92, baseband, audio_92k, num);
}

void sca_decoder_exit(sca_decoder_t *sca)
{
	(void)sca;
}
