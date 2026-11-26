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
 * See sdr_bands.h. The IQ path is the one of sdr.c's band client (a program
 * with channels = 0 on the backend): rows of one channel, TX scaled to the
 * band's level, the same ring length and TX target.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include "sdr_bands.h"

static double now(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

void sdr_bands_engine(sdr_bands_t *b, sdr_engine_t *e, double rx_centre, double tx_centre)
{
	memset(b, 0, sizeof(*b));
	b->engine = e;
	b->rx_centre = rx_centre;
	b->tx_centre = tx_centre;
}

void sdr_bands_backend(sdr_bands_t *b, const char *socket_name)
{
	memset(b, 0, sizeof(*b));
	b->backend = socket_name;
}

static sdr_band_t *band_of(sdr_bands_t *b, int band, int num)
{
	sdr_band_t *d;

	if (band < 0 || band >= b->n)
		return NULL;
	d = &b->band[band];
	if (num < 0 || num > d->def.buffer_size)
		return NULL;
	return d;
}

static void band_free(sdr_bands_t *b, sdr_band_t *d)
{
	if (d->group)
		sdr_engine_group_remove(b->engine, d->group);
	d->group = NULL;
	if (d->client_open)
		sdr_client_close(&d->client);
	d->client_open = 0;
	free(d->i);
	free(d->q);
	d->i = d->q = NULL;
	d->port = NULL;
}

int sdr_band_add(sdr_bands_t *b, const sdr_band_def_t *def, char *error, int error_len)
{
	sdr_band_t *d;
	int n_rx = def->rx ? 1 : 0, n_tx = def->tx ? 1 : 0, ring_ms, rc;

	error[0] = '\0';
	if (b->n >= SDR_BANDS_MAX) {
		snprintf(error, error_len, "at most %d bands", SDR_BANDS_MAX);
		return -EINVAL;
	}
	if ((!n_rx && !n_tx) || def->iq_rate <= 0 || def->buffer_size <= 0) {
		snprintf(error, error_len, "band '%s': no direction, IQ rate %d, buffer %d", def->name ? def->name : "", def->iq_rate, def->buffer_size);
		return -EINVAL;
	}
	if (!b->engine && !b->backend) {
		snprintf(error, error_len, "no engine and no backend");
		return -EINVAL;
	}
	d = &b->band[b->n];
	memset(d, 0, sizeof(*d));
	d->def = *def;
	d->i = calloc(def->buffer_size, sizeof(*d->i));
	d->q = calloc(def->buffer_size, sizeof(*d->q));
	if (!d->i || !d->q) {
		snprintf(error, error_len, "no memory");
		band_free(b, d);
		return -ENOMEM;
	}
	/* rings of 4 buffers, TX kept at one buffer: as sdr.c's band client */
	ring_ms = (int)ceil(4.0 * def->buffer_size * 1000.0 / def->iq_rate) + 10;
	if (b->engine) {
		double rx_off = def->rx_frequency - b->rx_centre, tx_off = def->tx_frequency - b->tx_centre;

		d->group = sdr_engine_group_add(b->engine, def->iq_rate, n_rx, &rx_off, n_tx, &tx_off, ring_ms, def->buffer_size);
		if (!d->group) {
			snprintf(error, error_len, "band '%s': %s", def->name ? def->name : "", sdr_engine_error());
			band_free(b, d);
			return -EINVAL;
		}
		d->port = &d->group->port;
		d->tx_scale = sdr_engine_tx_scale(b->engine);
	} else {
		char err[256];

		rc = sdr_client_open(&d->client, b->backend, def->name ? def->name : "osmocom-analog band", def->iq_rate,
				     n_rx, &def->rx_frequency, n_tx, &def->tx_frequency, ring_ms, def->buffer_size, err, sizeof(err));
		if (rc < 0) {
			snprintf(error, error_len, "band '%s': %s", def->name ? def->name : "", err);
			band_free(b, d);
			return rc;
		}
		d->client_open = 1;
		d->port = &d->client.port;
		d->tx_scale = d->client.tx_scale;
		d->alive_check = now();
	}
	return b->n++;
}

int sdr_band_read(sdr_bands_t *b, int band, float *iq, int num)
{
	sdr_band_t *d = band_of(b, band, num);
	sample_t *pi[1], *pq[1];
	int count, s;

	if (!d)
		return -EINVAL;
	/* on a backend: once a second, is it still there? */
	if (d->client_open) {
		double t = now();

		if (t - d->alive_check >= 1.0) {
			d->alive_check = t;
			if (!sdr_client_alive(&d->client))
				return -EPERM;
		}
	}
	if (!d->port->n_rx)
		return 0;
	pi[0] = d->i;
	pq[0] = d->q;
	count = sdr_port_read(d->port, pi, pq, num);
	for (s = 0; s < count; s++) {
		iq[2 * s] = d->i[s];
		iq[2 * s + 1] = d->q[s];
	}
	return count;
}

int sdr_band_write(sdr_bands_t *b, int band, const float *iq, int num)
{
	sdr_band_t *d = band_of(b, band, num);
	sample_t *pi[1], *pq[1];
	int s;

	if (!d)
		return -EINVAL;
	if (!d->port->n_tx)
		return 0;
	/* the band at its level (1 / N max of full scale, as every channel) */
	for (s = 0; s < num; s++) {
		d->i[s] = iq[2 * s] * d->tx_scale;
		d->q[s] = iq[2 * s + 1] * d->tx_scale;
	}
	pi[0] = d->i;
	pq[0] = d->q;
	return sdr_port_write(d->port, pi, pq, num);
}

int sdr_band_tosend(sdr_bands_t *b, int band)
{
	sdr_band_t *d = band_of(b, band, 0);
	int n;

	if (!d)
		return -EINVAL;
	n = sdr_port_tosend(d->port);
	return (n > d->def.buffer_size) ? d->def.buffer_size : n;
}

double sdr_band_tx_scale(const sdr_bands_t *b, int band)
{
	if (band < 0 || band >= b->n)
		return 0.0;
	return b->band[band].tx_scale;
}

void sdr_bands_close(sdr_bands_t *b)
{
	int i;

	for (i = 0; i < b->n; i++)
		band_free(b, &b->band[i]);
	b->n = 0;
}
