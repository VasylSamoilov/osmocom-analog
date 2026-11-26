/* SDR bands: several IQ channels (bands) of one program on one SDR
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * A band is one IQ channel at its frequency and its own IQ rate: the
 * program's IQ (interleaved float) goes through as it is, it makes its own
 * modulation (osmoradio: one band per station, docs/RADIO-STATIONS.md).
 * Each band is one group with one RX and / or one TX channel:
 *
 *  - in-process: on the engine of an SDR opened in server mode
 *    (sdr_server_mode(), sdr_server_engine()), the device threads run the
 *    filter banks as for a backend;
 *  - on an SDR backend: one registration (sdr_client_open()) per band.
 *
 * The same calls for both. Bands are added at start; the engine was made
 * for their number of TX channels (each at 1 / N of full scale).
 */

#ifndef _SDR_BANDS_H
#define _SDR_BANDS_H

#include "sdr_engine.h"
#include "sdr_backend.h"

#define SDR_BANDS_MAX	SDR_ENGINE_MAX_GROUPS

typedef struct sdr_band_def {
	const char	*name;		/* on the backend: shown in its log */
	int		rx, tx;		/* directions used */
	double		rx_frequency, tx_frequency;	/* Hz */
	int		iq_rate;	/* the band's IQ rate (S/s) */
	int		buffer_size;	/* most IQ samples per read / write; the TX ring is kept at it */
} sdr_band_def_t;

typedef struct sdr_band {
	sdr_band_def_t	def;
	sdr_group_t	*group;		/* in-process */
	sdr_client_t	client;		/* on a backend */
	int		client_open;
	sdr_port_t	*port;
	double		tx_scale;	/* TX level of the band (1 / N max of full scale) */
	sample_t	*i, *q;		/* buffer_size, the port's rows */
	double		alive_check;	/* backend: time it was last checked */
} sdr_band_t;

typedef struct sdr_bands {
	sdr_engine_t	*engine;	/* in-process, or NULL */
	const char	*backend;	/* backend socket, or NULL */
	double		rx_centre, tx_centre;	/* in-process: the SDR's frequencies (Hz) */
	int		n;
	sdr_band_t	band[SDR_BANDS_MAX];
} sdr_bands_t;

/* in-process, on an engine and the SDR's centre frequencies */
void sdr_bands_engine(sdr_bands_t *b, sdr_engine_t *e, double rx_centre, double tx_centre);
/* on the SDR backend at socket_name */
void sdr_bands_backend(sdr_bands_t *b, const char *socket_name);
/* add a band; returns its index, or < 0 with the reason in error */
int sdr_band_add(sdr_bands_t *b, const sdr_band_def_t *def, char *error, int error_len);
/* up to num IQ samples of the band (num <= buffer_size); returns the
 * number, -EPERM if the backend is gone, -EINVAL for a bad call */
int sdr_band_read(sdr_bands_t *b, int band, float *iq, int num);
/* write num IQ samples (full scale 1.0; sent at the band's TX level);
 * returns how many fit, -EINVAL for a bad call */
int sdr_band_write(sdr_bands_t *b, int band, const float *iq, int num);
/* how many to write to keep the band's TX ring at buffer_size */
int sdr_band_tosend(sdr_bands_t *b, int band);
/* the band's TX level (1 / N max of full scale) */
double sdr_band_tx_scale(const sdr_bands_t *b, int band);
/* remove all bands. In-process: before the engine goes (before sdr_close()):
 * the engine frees its groups at exit. */
void sdr_bands_close(sdr_bands_t *b);

#endif /* _SDR_BANDS_H */
