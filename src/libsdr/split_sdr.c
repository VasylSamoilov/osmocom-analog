/* Split SDR backend - separate TX and RX devices
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This backend handles separate TX and RX SDR devices using software clock
 * for timing synchronization.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "sdr_config.h"
#include "split_sdr.h"

#ifdef HAVE_SOAPY
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#endif

#define DSPLITSDR DSDR

typedef struct split_sdr {
	int channels;
	int samplerate;
	int buffer_size;
	
	/* TX device */
	void *tx_device;
	void *tx_stream;
	int tx_samplerate;
	
	/* RX device */
	void *rx_device;
	void *rx_stream;
	int rx_samplerate;
	
	/* Software clock for timing */
	struct timespec base_time;
	long long tx_time_ns;
	int clock_valid;
	
	/* Buffers */
	float *tx_buffer;
	float *rx_buffer;
	int tx_buffer_size;
	int rx_buffer_size;
} split_sdr_t;

static split_sdr_t *split_sdr_inst = NULL;

/* Initialize software clock */
static void init_clock(split_sdr_t *s)
{
	clock_gettime(CLOCK_MONOTONIC_RAW, &s->base_time);
	s->tx_time_ns = 0;
	s->clock_valid = 1;
	LOGP(DSPLITSDR, LOGL_NOTICE, "Split SDR: Using software clock (may drift relative to SDR sample clocks)\n");
}

/* Get elapsed time in nanoseconds */
static long long get_time_ns(split_sdr_t *s)
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC_RAW, &now);
	return (now.tv_sec - s->base_time.tv_sec) * 1000000000LL +
	       (now.tv_nsec - s->base_time.tv_nsec);
}

void *split_sdr_open(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index)
{
	split_sdr_t *s;
	
	(void)direction;
	(void)audiodev;
	(void)am;
	(void)paging_frequency;
	(void)interval;
	(void)max_deviation;
	(void)max_modulation;
	(void)modulation_index;
	
	LOGP(DSPLITSDR, LOGL_INFO, "Opening split SDR backend\n");
	
	if (!sdr_config->split_mode) {
		LOGP(DSPLITSDR, LOGL_ERROR, "Split SDR backend requires split mode configuration\n");
		return NULL;
	}
	
	s = calloc(1, sizeof(*s));
	if (!s) {
		LOGP(DSPLITSDR, LOGL_ERROR, "Failed to allocate split SDR state\n");
		return NULL;
	}
	
	s->channels = channels;
	s->samplerate = samplerate;
	s->buffer_size = buffer_size;
	s->tx_samplerate = sdr_config->tx_samplerate ? sdr_config->tx_samplerate : samplerate;
	s->rx_samplerate = sdr_config->rx_samplerate ? sdr_config->rx_samplerate : samplerate;
	
#ifdef HAVE_SOAPY
	/* Open TX device if configured */
	if (sdr_config->tx_device_args && tx_frequency && *tx_frequency != 0.0) {
		LOGP(DSPLITSDR, LOGL_INFO, "Opening TX device: %s\n", sdr_config->tx_device_args);
		s->tx_device = SoapySDRDevice_makeStrArgs(sdr_config->tx_device_args);
		if (!s->tx_device) {
			LOGP(DSPLITSDR, LOGL_ERROR, "Failed to open TX SDR device: %s\n", 
			     SoapySDRDevice_lastError());
			goto error;
		}
		
		/* Configure TX */
		SoapySDRDevice_setSampleRate(s->tx_device, SOAPY_SDR_TX, 0, s->tx_samplerate);
		SoapySDRDevice_setFrequency(s->tx_device, SOAPY_SDR_TX, 0, *tx_frequency, NULL);
		SoapySDRDevice_setGain(s->tx_device, SOAPY_SDR_TX, 0, sdr_config->tx_gain);
		
		/* Setup TX stream */
#ifdef SOAPY_0_8_0_OR_HIGHER
		s->tx_stream = SoapySDRDevice_setupStream(s->tx_device, SOAPY_SDR_TX,
		                                          SOAPY_SDR_CF32, NULL, 0, NULL);
		if (!s->tx_stream) {
#else
		if (SoapySDRDevice_setupStream(s->tx_device, (SoapySDRStream **)&s->tx_stream, SOAPY_SDR_TX,
		                               SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
#endif
			LOGP(DSPLITSDR, LOGL_ERROR, "Failed to setup TX stream\n");
			goto error;
		}
		
		LOGP(DSPLITSDR, LOGL_INFO, "TX: freq=%.6f MHz, rate=%d Hz, gain=%.1f dB\n",
		     *tx_frequency / 1e6, s->tx_samplerate, sdr_config->tx_gain);
	}
	
	/* Open RX device if configured */
	if (sdr_config->rx_device_args && rx_frequency && *rx_frequency != 0.0) {
		LOGP(DSPLITSDR, LOGL_INFO, "Opening RX device: %s\n", sdr_config->rx_device_args);
		s->rx_device = SoapySDRDevice_makeStrArgs(sdr_config->rx_device_args);
		if (!s->rx_device) {
			LOGP(DSPLITSDR, LOGL_ERROR, "Failed to open RX SDR device: %s\n",
			     SoapySDRDevice_lastError());
			goto error;
		}
		
		/* Configure RX */
		SoapySDRDevice_setSampleRate(s->rx_device, SOAPY_SDR_RX, 0, s->rx_samplerate);
		SoapySDRDevice_setFrequency(s->rx_device, SOAPY_SDR_RX, 0, *rx_frequency, NULL);
		SoapySDRDevice_setGain(s->rx_device, SOAPY_SDR_RX, 0, sdr_config->rx_gain);
		
		/* Setup RX stream */
#ifdef SOAPY_0_8_0_OR_HIGHER
		s->rx_stream = SoapySDRDevice_setupStream(s->rx_device, SOAPY_SDR_RX,
		                                          SOAPY_SDR_CF32, NULL, 0, NULL);
		if (!s->rx_stream) {
#else
		if (SoapySDRDevice_setupStream(s->rx_device, (SoapySDRStream **)&s->rx_stream, SOAPY_SDR_RX,
		                               SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
#endif
			LOGP(DSPLITSDR, LOGL_ERROR, "Failed to setup RX stream\n");
			goto error;
		}
		
		LOGP(DSPLITSDR, LOGL_INFO, "RX: freq=%.6f MHz, rate=%d Hz, gain=%.1f dB\n",
		     *rx_frequency / 1e6, s->rx_samplerate, sdr_config->rx_gain);
	}
#else
	LOGP(DSPLITSDR, LOGL_ERROR, "SoapySDR support not compiled in\n");
	goto error;
#endif
	
	/* Allocate buffers */
	s->tx_buffer_size = buffer_size * 2;  /* I/Q pairs */
	s->rx_buffer_size = buffer_size * 2;
	s->tx_buffer = calloc(s->tx_buffer_size, sizeof(float));
	s->rx_buffer = calloc(s->rx_buffer_size, sizeof(float));
	if (!s->tx_buffer || !s->rx_buffer) {
		LOGP(DSPLITSDR, LOGL_ERROR, "Failed to allocate buffers\n");
		goto error;
	}
	
	split_sdr_inst = s;
	return s;

error:
	split_sdr_close(s);
	return NULL;
}

int split_sdr_start(void *inst)
{
	split_sdr_t *s = inst;
	
#ifdef HAVE_SOAPY
	if (s->tx_stream) {
		if (SoapySDRDevice_activateStream(s->tx_device, s->tx_stream, 0, 0, 0) != 0) {
			LOGP(DSPLITSDR, LOGL_ERROR, "Failed to activate TX stream\n");
			return -EIO;
		}
		LOGP(DSPLITSDR, LOGL_INFO, "TX stream activated\n");
	}
	
	if (s->rx_stream) {
		if (SoapySDRDevice_activateStream(s->rx_device, s->rx_stream, 0, 0, 0) != 0) {
			LOGP(DSPLITSDR, LOGL_ERROR, "Failed to activate RX stream\n");
			return -EIO;
		}
		LOGP(DSPLITSDR, LOGL_INFO, "RX stream activated\n");
	}
#endif
	
	/* Initialize software clock */
	init_clock(s);
	
	return 0;
}

void split_sdr_close(void *inst)
{
	split_sdr_t *s = inst;
	
	if (!s)
		return;
	
	LOGP(DSPLITSDR, LOGL_DEBUG, "Closing split SDR backend\n");
	
#ifdef HAVE_SOAPY
	if (s->tx_stream) {
		SoapySDRDevice_deactivateStream(s->tx_device, s->tx_stream, 0, 0);
		SoapySDRDevice_closeStream(s->tx_device, s->tx_stream);
	}
	if (s->tx_device)
		SoapySDRDevice_unmake(s->tx_device);
	
	if (s->rx_stream) {
		SoapySDRDevice_deactivateStream(s->rx_device, s->rx_stream, 0, 0);
		SoapySDRDevice_closeStream(s->rx_device, s->rx_stream);
	}
	if (s->rx_device)
		SoapySDRDevice_unmake(s->rx_device);
#endif
	
	free(s->tx_buffer);
	free(s->rx_buffer);
	free(s);
	
	if (split_sdr_inst == s)
		split_sdr_inst = NULL;
}

int split_sdr_write(void *inst, sample_t **samples, uint8_t **power, int num, enum paging_signal *paging_signal, int *on, int channels)
{
	split_sdr_t *s = inst;
	int sent = 0;
	
	(void)power;
	(void)paging_signal;
	(void)on;
	
	if (!s->tx_stream)
		return num;  /* No TX device, pretend we sent everything */
	
#ifdef HAVE_SOAPY
	/* Convert samples to IQ (simple FM modulation placeholder) */
	/* For now, just send silence - full modulation requires integration */
	void *buffs[1] = { s->tx_buffer };
	int flags = 0;
	
	memset(s->tx_buffer, 0, num * 2 * sizeof(float));
	
	sent = SoapySDRDevice_writeStream(s->tx_device, s->tx_stream, 
	                                  (const void * const *)buffs, num, &flags, 0, 0);
	if (sent < 0) {
		LOGP(DSPLITSDR, LOGL_ERROR, "TX write error: %d\n", sent);
		return 0;
	}
#endif
	
	(void)samples;
	(void)channels;
	
	return sent;
}

int split_sdr_read(void *inst, sample_t **samples, int num, int channels, double *rf_level_db)
{
	split_sdr_t *s = inst;
	int count = 0;
	
	if (!s->rx_stream)
		return 0;  /* No RX device */
	
#ifdef HAVE_SOAPY
	void *buffs[1] = { s->rx_buffer };
	int flags = 0;
	long long timeNs;
	
	count = SoapySDRDevice_readStream(s->rx_device, s->rx_stream,
	                                  buffs, num, &flags, &timeNs, 0);
	if (count < 0) {
		/* TIMEOUT (-1) is normal for non-blocking reads, silently return 0 */
		if (count == SOAPY_SDR_TIMEOUT)
			return 0;
		LOGP(DSPLITSDR, LOGL_ERROR, "RX read error: %d\n", count);
		return 0;
	}
	
	/* Convert IQ to samples (simple demodulation placeholder) */
	/* For now, just return zeros - full demodulation requires integration */
	for (int c = 0; c < channels; c++) {
		memset(samples[c], 0, count * sizeof(sample_t));
	}
#endif
	
	if (rf_level_db)
		*rf_level_db = 0.0;
	
	return count;
}

int split_sdr_get_tosend(void *inst, int buffer_size)
{
	split_sdr_t *s = inst;
	long long now_ns, elapsed_ns;
	long long ns_per_sample;
	int tosend;
	
	if (!s->clock_valid)
		return 0;
	
	ns_per_sample = 1000000000LL / s->tx_samplerate;
	now_ns = get_time_ns(s);
	
	/* How many samples should have been sent by now */
	elapsed_ns = now_ns - s->tx_time_ns;
	tosend = elapsed_ns / ns_per_sample;
	
	/* Clamp to buffer size */
	if (tosend > buffer_size)
		tosend = buffer_size;
	if (tosend < 0)
		tosend = 0;
	
	/* Advance our TX time */
	if (tosend > 0)
		s->tx_time_ns += tosend * ns_per_sample;
	
	return tosend;
}
