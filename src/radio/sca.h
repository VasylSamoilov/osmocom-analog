/*
 * SCA (Subsidiary Communications Authorization) encoder/decoder
 *
 * (C) 2024 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * Implements FM subcarrier encoding/decoding per FCC 47 CFR 73.319
 * - Standard frequencies: 67 kHz, 92 kHz
 * - FM modulation with +/-7.5 kHz deviation (typical)
 * - Max 10% injection for subcarriers above 75 kHz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef _SCA_H
#define _SCA_H

#include <stdint.h>
#include "../libsample/sample.h"

/* SCA constants per FCC 73.319 */
#define SCA_FREQ_67K		67000.0		/* 67 kHz subcarrier */
#define SCA_FREQ_92K		92000.0		/* 92 kHz subcarrier */
#define SCA_DEVIATION		7500.0		/* +/-7.5 kHz typical deviation */
#define SCA_INJECTION_67K	0.10		/* 10% max injection for 67 kHz */
#define SCA_INJECTION_92K	0.10		/* 10% max injection for 92 kHz */
#define SCA_AUDIO_BW		5000.0		/* 5 kHz audio bandwidth */

typedef struct sca_channel {
	/* Configuration */
	double		frequency;	/* subcarrier frequency (67k or 92k) */
	double		deviation;	/* FM deviation */
	double		injection;	/* injection level (0.0-1.0) */
	int		enabled;	/* channel enabled */
	
	/* Modulator state */
	double		samplerate;
	double		phase;		/* subcarrier phase */
	double		phasestep;	/* phase increment per sample */
	
	/* Demodulator state */
	double		lpf_i;		/* low-pass filtered I */
	double		lpf_q;		/* low-pass filtered Q */
	double		lpf_alpha;	/* filter coefficient */
	double		last_phase;	/* for FM demod */
} sca_channel_t;

typedef struct sca_encoder {
	sca_channel_t	ch67;		/* 67 kHz channel */
	sca_channel_t	ch92;		/* 92 kHz channel */
	double		samplerate;
} sca_encoder_t;

typedef struct sca_decoder {
	sca_channel_t	ch67;		/* 67 kHz channel */
	sca_channel_t	ch92;		/* 92 kHz channel */
	double		samplerate;
} sca_decoder_t;

/* Encoder functions */
int sca_encoder_init(sca_encoder_t *sca, double samplerate, int enable_67k, int enable_92k);
void sca_encoder_process(sca_encoder_t *sca, sample_t *audio_67k, sample_t *audio_92k,
			 sample_t *baseband, int num);
void sca_encoder_exit(sca_encoder_t *sca);

/* Decoder functions */
int sca_decoder_init(sca_decoder_t *sca, double samplerate, int enable_67k, int enable_92k);
void sca_decoder_process(sca_decoder_t *sca, sample_t *baseband, 
			 sample_t *audio_67k, sample_t *audio_92k, int num);
void sca_decoder_exit(sca_decoder_t *sca);

#endif /* _SCA_H */
