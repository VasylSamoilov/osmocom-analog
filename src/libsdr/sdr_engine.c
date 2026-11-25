/* SDR engine: filter banks, registered channel groups, DSP rate rings
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include "sdr_engine.h"

static char error_text[256];

const char *sdr_engine_error(void)
{
	return error_text;
}

/*
 * ring of rows, single producer / single consumer
 */

void sdr_ring_set(sdr_ring_t *r, float *buf, int width, int rows, sdr_ring_ctl_t *ctl)
{
	r->buf = buf;
	r->width = width;
	r->rows = rows;
	r->ctl = ctl;
	r->pos = 0;	/* the view starts zeroed */
}

int sdr_ring_fill(const sdr_ring_t *r)
{
	int in = __atomic_load_n(&r->ctl->in, __ATOMIC_ACQUIRE), out = __atomic_load_n(&r->ctl->out, __ATOMIC_ACQUIRE);

	/* the other side may write anything (shared memory) */
	if (in < 0 || in >= r->rows || out < 0 || out >= r->rows)
		return -1;
	return (in - out + r->rows) % r->rows;
}

int sdr_ring_space(const sdr_ring_t *r)
{
	int fill = sdr_ring_fill(r);

	return (fill < 0) ? -1 : r->rows - 1 - fill;
}

/* the device threads: their own index is r->pos, the program's is read
 * once and checked (it may write anything at any time) */
static int ring_space_dev(const sdr_ring_t *r)	/* RX: the device writes */
{
	int out = __atomic_load_n(&r->ctl->out, __ATOMIC_ACQUIRE);

	if (out < 0 || out >= r->rows)
		return -1;
	return r->rows - 1 - (r->pos - out + r->rows) % r->rows;
}

static int ring_fill_dev(const sdr_ring_t *r)	/* TX: the device reads */
{
	int in = __atomic_load_n(&r->ctl->in, __ATOMIC_ACQUIRE);

	if (in < 0 || in >= r->rows)
		return -1;
	return (in - r->pos + r->rows) % r->rows;
}

/*
 * views
 */

int sdr_engine_rows(int dsp_rate, int ring_ms)
{
	return ring_ms * dsp_rate / 1000 + 2;
}

/* the header rounded up, so the rows are aligned */
static size_t view_header(void)
{
	return (sizeof(sdr_view_t) + 63) & ~(size_t)63;
}

size_t sdr_view_size(int n_rx, int n_tx, int rows)
{
	return view_header() + (size_t)rows * 2 * (n_rx + n_tx) * sizeof(float);
}

/* the rings on a view whose sizes are known (checked) */
static void view_rings(sdr_view_t *v, int n_rx, int n_tx, int rows, sdr_ring_t *rx, sdr_ring_t *tx)
{
	float *rows_rx = (float *)((char *)v + view_header());
	float *rows_tx = rows_rx + (size_t)rows * 2 * n_rx;

	sdr_ring_set(rx, rows_rx, 2 * (n_rx ? n_rx : 1), rows, &v->rx_ctl);
	sdr_ring_set(tx, rows_tx, 2 * (n_tx ? n_tx : 1), rows, &v->tx_ctl);
}

/*
 * bins
 */

/* Number of bins of a filter bank for these rates, or 0 if none fits: the
 * spacing (SDR rate / bins) must be more than the DSP rate (the bank keeps
 * +-DSP/2 clean around every channel), the SDR rate a multiple of bins / 2.
 * Of those the cheapest by an estimate of the work per SDR sample:
 * - the bank: the prototype (4 per tap per branch, the length from the
 *   Kaiser formula for the transition spacing - DSP rate) and the FFT
 *   (10 log2 bins);
 * - each channel's tail: the fine NCO at the bin rate (8) and the wide
 *   resampler between the bin rate fb and 2 DSP rate (4 per tap, 2 DSP
 *   outputs per second, 6.75 fb / (min(fb, 2 dsp) - dsp) taps each).
 * Measured at 10 MS/s, DSP 48 k, 2 channels (both ways, this PC): 8 bins
 * 28.6 %, 16: 22.4 %, 32: 21.1 %, 64: 21.3 %, 128: 36.0 % - the estimate
 * ranks them the same. */
int sdr_engine_bins(int fs, int dsp, int nchan)
{
	int m, best = 0;
	double best_cost = 0.0;

	for (m = 8; m <= 4096; m *= 2) {
		double spacing = (double)fs / m, trans, len, t, cost, fb, low, tail;

		if (fs % (m / 2))
			continue;
		trans = spacing - dsp;
		if (trans <= 0.05 * spacing)
			break;
		len = (PFB_ATTEN_DB - 7.95) / (2.285 * 2.0 * M_PI * trans / fs) + 1.0;
		t = ceil(len / m);
		fb = 2.0 * spacing;
		low = (fb < 2.0 * dsp) ? fb : 2.0 * dsp;
		tail = 8.0 * fb / fs + 4.0 * (2.0 * dsp / fs) * 6.75 * fb / (low - dsp);
		cost = 4.0 * t + 10.0 * log2(m) + nchan * tail;
		if (!best || cost < best_cost) {
			best = m;
			best_cost = cost;
		}
	}
	return best;
}

/*
 * engine
 */

int sdr_engine_init(sdr_engine_t *e, int rx_rate, int tx_rate, int max_dsp, int tx_nmax, int max_block)
{
	int m, rc;

	memset(e, 0, sizeof(*e));
	error_text[0] = '\0';
	pthread_mutex_init(&e->lock, NULL);
	e->rx_rate = rx_rate;
	e->tx_rate = tx_rate;
	e->max_dsp = max_dsp;
	e->tx_nmax = (tx_nmax > 0) ? tx_nmax : 1;
	e->max_block = max_block;
	e->next_id = 1;
	/* the sub-bands for the channels expected (tx_nmax): the more channels,
	 * the more the tails weigh against the bank */
	if (rx_rate) {
		m = sdr_engine_bins(rx_rate, max_dsp, e->tx_nmax);
		if (!m) {
			snprintf(error_text, sizeof(error_text), "no filter bank fits SDR rate %d and DSP rate %d (sub-bands must be wider than the DSP rate)", rx_rate, max_dsp);
			return -EINVAL;
		}
		rc = pfb_rx_init(&e->bank_rx, rx_rate, m, max_dsp / 2.0);
		if (rc < 0) {
			snprintf(error_text, sizeof(error_text), "RX bank: %s", pfb_error());
			return rc;
		}
		e->rx_frames_max = max_block / e->bank_rx.d + 2;
		e->rx_bins = calloc(SDR_ENGINE_MAX_GROUPS * SDR_ENGINE_MAX_CHAN, sizeof(*e->rx_bins));
		e->rx_bin_i = calloc(SDR_ENGINE_MAX_GROUPS * SDR_ENGINE_MAX_CHAN, sizeof(*e->rx_bin_i));
		e->rx_bin_q = calloc(SDR_ENGINE_MAX_GROUPS * SDR_ENGINE_MAX_CHAN, sizeof(*e->rx_bin_q));
		if (!e->rx_bins || !e->rx_bin_i || !e->rx_bin_q)
			goto nomem;
	}
	if (tx_rate) {
		m = sdr_engine_bins(tx_rate, max_dsp, e->tx_nmax);
		if (!m) {
			snprintf(error_text, sizeof(error_text), "no filter bank fits SDR rate %d and DSP rate %d (sub-bands must be wider than the DSP rate)", tx_rate, max_dsp);
			sdr_engine_exit(e);
			return -EINVAL;
		}
		rc = pfb_tx_init(&e->bank_tx, tx_rate, m, max_dsp / 2.0);
		if (rc < 0) {
			snprintf(error_text, sizeof(error_text), "TX bank: %s", pfb_error());
			sdr_engine_exit(e);
			return rc;
		}
		e->tx_frames_max = max_block / e->bank_tx.d + 2;
		e->tx_bins = calloc(SDR_ENGINE_MAX_GROUPS * SDR_ENGINE_MAX_CHAN, sizeof(*e->tx_bins));
		e->tx_ci = calloc(SDR_ENGINE_MAX_GROUPS * SDR_ENGINE_MAX_CHAN, sizeof(*e->tx_ci));
		e->tx_cq = calloc(SDR_ENGINE_MAX_GROUPS * SDR_ENGINE_MAX_CHAN, sizeof(*e->tx_cq));
		if (!e->tx_bins || !e->tx_ci || !e->tx_cq)
			goto nomem;
	}
	return 0;

nomem:
	snprintf(error_text, sizeof(error_text), "no memory");
	sdr_engine_exit(e);
	return -ENOMEM;
}

static void group_free(sdr_group_t *g)
{
	int c;

	if (g->rx) {
		for (c = 0; c < g->n_rx; c++)
			pfb_chan_exit(&g->rx[c]);
		free(g->rx);
	}
	if (g->tx) {
		for (c = 0; c < g->n_tx; c++)
			pfb_chan_exit(&g->tx[c]);
		free(g->tx);
	}
	if (g->crx) {
		for (c = 0; c < g->n_rx; c++)
			channelizer_exit(&g->crx[c]);
		free(g->crx);
	}
	if (g->ctx) {
		for (c = 0; c < g->n_tx; c++)
			channelizer_exit(&g->ctx[c]);
		free(g->ctx);
	}
	free(g->tx_fifo);
	if (g->view_own)
		free(g->view);
	free(g->tx_in_i);
	free(g->tx_in_q);
	free(g->rx_out_i);
	free(g->rx_out_q);
	free(g);
}

void sdr_engine_exit(sdr_engine_t *e)
{
	int i;

	for (i = 0; i < SDR_ENGINE_MAX_GROUPS; i++)
		if (e->groups[i])
			group_free(e->groups[i]);
	if (e->rx_bin_i) {
		for (i = 0; i < SDR_ENGINE_MAX_GROUPS * SDR_ENGINE_MAX_CHAN; i++) {
			free(e->rx_bin_i[i]);
			free(e->rx_bin_q[i]);
		}
	}
	if (e->bank_rx.m)
		pfb_exit(&e->bank_rx);
	if (e->bank_tx.m)
		pfb_exit(&e->bank_tx);
	free(e->rx_bins);
	free(e->rx_bin_i);
	free(e->rx_bin_q);
	free(e->tx_bins);
	free(e->tx_ci);
	free(e->tx_cq);
	pthread_mutex_destroy(&e->lock);
	memset(e, 0, sizeof(*e));
}

double sdr_engine_tx_scale(const sdr_engine_t *e)
{
	return 1.0 / e->tx_nmax;
}

int sdr_engine_tx_step(const sdr_engine_t *e)
{
	return e->bank_tx.d;
}

sdr_group_t *sdr_engine_group_add(sdr_engine_t *e, int dsp_rate, int n_rx, const double *rx_offset, int n_tx, const double *tx_offset, int ring_ms, int tx_target)
{
	return sdr_engine_group_add_mem(e, dsp_rate, n_rx, rx_offset, n_tx, tx_offset, ring_ms, tx_target, NULL, 0);
}

sdr_group_t *sdr_engine_group_add_mem(sdr_engine_t *e, int dsp_rate, int n_rx, const double *rx_offset, int n_tx, const double *tx_offset, int ring_ms, int tx_target, void *mem, size_t size)
{
	sdr_group_t *g = NULL;
	int c, i, rc, rows;
	size_t vsize;

	error_text[0] = '\0';
	if (n_rx < 0 || n_rx > SDR_ENGINE_MAX_CHAN || n_tx < 0 || n_tx > SDR_ENGINE_MAX_CHAN || (n_rx && !e->rx_rate) || (n_tx && !e->tx_rate)) {
		snprintf(error_text, sizeof(error_text), "%d RX / %d TX channels: not possible (at most %d each, RX %s, TX %s)",
			 n_rx, n_tx, SDR_ENGINE_MAX_CHAN, e->rx_rate ? "on" : "off", e->tx_rate ? "on" : "off");
		return NULL;
	}
	if (dsp_rate <= 0) {
		snprintf(error_text, sizeof(error_text), "DSP rate %d", dsp_rate);
		return NULL;
	}
	pthread_mutex_lock(&e->lock);
	if (e->tx_used + n_tx > e->tx_nmax) {
		pthread_mutex_unlock(&e->lock);
		snprintf(error_text, sizeof(error_text), "%d TX channels, %d of %d in use", n_tx, e->tx_used, e->tx_nmax);
		return NULL;
	}
	if (dsp_rate > e->max_dsp && (e->rx_chain + n_rx > SDR_ENGINE_MAX_CHAIN || e->tx_chain + n_tx > SDR_ENGINE_MAX_CHAIN)) {
		pthread_mutex_unlock(&e->lock);
		snprintf(error_text, sizeof(error_text), "DSP rate %d is above the %d of the filter banks: per-channel chain, %d RX / %d TX of at most %d each in use",
			 dsp_rate, e->max_dsp, e->rx_chain, e->tx_chain, SDR_ENGINE_MAX_CHAIN);
		return NULL;
	}
	e->tx_used += n_tx;	/* reserved, given back on failure */
	if (dsp_rate > e->max_dsp) {
		e->rx_chain += n_rx;
		e->tx_chain += n_tx;
	}
	pthread_mutex_unlock(&e->lock);

	g = calloc(1, sizeof(*g));
	if (!g)
		goto nomem;
	g->dsp_rate = dsp_rate;
	g->n_rx = n_rx;
	g->n_tx = n_tx;
	g->chain = (dsp_rate > e->max_dsp);
	rows = sdr_engine_rows(dsp_rate, ring_ms);
	/* the view: rings and counters */
	vsize = sdr_view_size(n_rx, n_tx, rows);
	if (mem) {
		if (size < vsize) {
			snprintf(error_text, sizeof(error_text), "view of %zu bytes, %zu needed", size, vsize);
			goto fail;
		}
		g->view = mem;
		memset(g->view, 0, vsize);
	} else {
		g->view = calloc(1, vsize);
		if (!g->view)
			goto nomem;
		g->view_own = 1;
	}
	g->view->size = vsize;
	g->view->dsp_rate = dsp_rate;
	g->view->n_rx = n_rx;
	g->view->n_tx = n_tx;
	g->view->rows = rows;
	g->view->tx_scale = sdr_engine_tx_scale(e);
	view_rings(g->view, n_rx, n_tx, rows, &g->rx_ring, &g->tx_ring);
	if (n_rx && g->chain) {
		/* the chain takes every SDR block whole: its output per block */
		g->crx = calloc(n_rx, sizeof(*g->crx));
		g->rx_max = (int)ceil((double)e->max_block * dsp_rate / e->rx_rate) + 8;
		g->rx_out_i = calloc((size_t)n_rx * g->rx_max, sizeof(sample_t));
		g->rx_out_q = calloc((size_t)n_rx * g->rx_max, sizeof(sample_t));
		if (!g->crx || !g->rx_out_i || !g->rx_out_q)
			goto nomem;
		for (c = 0; c < n_rx; c++) {
			g->rx_offset[c] = rx_offset[c];
			/* the same guarantee as the tails: flat to 0.4 of the DSP rate */
			rc = channelizer_rx_init(&g->crx[c], e->rx_rate, dsp_rate, rx_offset[c], 2.0 * CHANNELIZER_PASS * dsp_rate, e->max_block);
			if (rc < 0) {
				snprintf(error_text, sizeof(error_text), "RX channel at %+.0f Hz, DSP rate %d: %s", rx_offset[c], dsp_rate, channelizer_error());
				g->n_rx = c;
				goto fail;
			}
		}
	} else if (n_rx) {
		int fb = pfb_bin_rate(&e->bank_rx);

		g->rx = calloc(n_rx, sizeof(*g->rx));
		g->rx_max = (int)ceil((double)e->rx_frames_max * dsp_rate / fb) + 8;
		g->rx_out_i = calloc((size_t)n_rx * g->rx_max, sizeof(sample_t));
		g->rx_out_q = calloc((size_t)n_rx * g->rx_max, sizeof(sample_t));
		if (!g->rx || !g->rx_out_i || !g->rx_out_q)
			goto nomem;
		for (c = 0; c < n_rx; c++) {
			g->rx_offset[c] = rx_offset[c];
			rc = pfb_chan_rx_init(&g->rx[c], &e->bank_rx, rx_offset[c], dsp_rate, e->rx_frames_max);
			if (rc < 0) {
				snprintf(error_text, sizeof(error_text), "RX channel at %+.0f Hz: %s", rx_offset[c], pfb_error());
				g->n_rx = c;
				goto fail;
			}
		}
	}
	if (n_tx && g->chain) {
		g->ctx = calloc(n_tx, sizeof(*g->ctx));
		g->tx_max = (int)ceil((double)e->max_block * dsp_rate / e->tx_rate) + 8;
		g->tx_in_i = calloc((size_t)n_tx * g->tx_max, sizeof(sample_t));
		g->tx_in_q = calloc((size_t)n_tx * g->tx_max, sizeof(sample_t));
		if (!g->ctx || !g->tx_in_i || !g->tx_in_q)
			goto nomem;
		for (c = 0; c < n_tx; c++) {
			g->tx_offset[c] = tx_offset[c];
			rc = channelizer_tx_init(&g->ctx[c], e->tx_rate, dsp_rate, tx_offset[c], 2.0 * CHANNELIZER_PASS * dsp_rate, g->tx_max);
			if (rc < 0) {
				snprintf(error_text, sizeof(error_text), "TX channel at %+.0f Hz, DSP rate %d: %s", tx_offset[c], dsp_rate, channelizer_error());
				g->n_tx = c;
				goto fail;
			}
		}
		/* what one pull makes on top of a block still to give */
		g->tx_fifo_size = e->max_block + channelizer_tx_max(&g->ctx[0], g->tx_max) + 16;
		g->tx_fifo = calloc((size_t)2 * g->tx_fifo_size, sizeof(float));
		if (!g->tx_fifo)
			goto nomem;
		g->tx_target = (tx_target < rows - 1) ? tx_target : rows - 1;
	} else if (n_tx) {
		int fb = pfb_bin_rate(&e->bank_tx);

		g->tx = calloc(n_tx, sizeof(*g->tx));
		g->tx_max = (int)ceil((double)e->tx_frames_max * dsp_rate / fb) + 8;
		g->tx_in_i = calloc((size_t)n_tx * g->tx_max, sizeof(sample_t));
		g->tx_in_q = calloc((size_t)n_tx * g->tx_max, sizeof(sample_t));
		if (!g->tx || !g->tx_in_i || !g->tx_in_q)
			goto nomem;
		for (c = 0; c < n_tx; c++) {
			g->tx_offset[c] = tx_offset[c];
			rc = pfb_chan_tx_init(&g->tx[c], &e->bank_tx, tx_offset[c], dsp_rate, g->tx_max);
			if (rc < 0) {
				snprintf(error_text, sizeof(error_text), "TX channel at %+.0f Hz: %s", tx_offset[c], pfb_error());
				g->n_tx = c;
				goto fail;
			}
		}
		g->tx_target = (tx_target < rows - 1) ? tx_target : rows - 1;
	}
	g->view->tx_target = g->tx_target;
	/* complete: the program side may attach (the magic last) */
	g->view->version = SDR_VIEW_VERSION;
	__atomic_store_n(&g->view->magic, SDR_VIEW_MAGIC, __ATOMIC_RELEASE);
	if (sdr_port_attach(&g->port, g->view, vsize) < 0) {
		snprintf(error_text, sizeof(error_text), "view not valid, please fix");
		goto fail;
	}

	/* into the registry: the device threads adopt it at their next block */
	pthread_mutex_lock(&e->lock);
	for (i = 0; i < SDR_ENGINE_MAX_GROUPS; i++)
		if (!e->groups[i])
			break;
	if (i == SDR_ENGINE_MAX_GROUPS) {
		pthread_mutex_unlock(&e->lock);
		snprintf(error_text, sizeof(error_text), "all %d groups in use", SDR_ENGINE_MAX_GROUPS);
		goto fail;
	}
	g->id = e->next_id++;
	e->groups[i] = g;
	pthread_mutex_unlock(&e->lock);
	return g;

nomem:
	snprintf(error_text, sizeof(error_text), "no memory");
fail:
	pthread_mutex_lock(&e->lock);
	e->tx_used -= n_tx;
	if (dsp_rate > e->max_dsp) {
		e->rx_chain -= n_rx;
		e->tx_chain -= n_tx;
	}
	pthread_mutex_unlock(&e->lock);
	if (g)
		group_free(g);
	return NULL;
}

void sdr_engine_group_remove(sdr_engine_t *e, sdr_group_t *g)
{
	int i, wait;

	pthread_mutex_lock(&e->lock);
	g->leaving = 1;
	pthread_mutex_unlock(&e->lock);
	/* the device threads drop it at their next block; if they do not run
	 * (stopped), they will not touch it again either */
	for (wait = 0; wait < 2000 && (__atomic_load_n(&g->rx_on, __ATOMIC_ACQUIRE) || __atomic_load_n(&g->tx_on, __ATOMIC_ACQUIRE)); wait++)
		usleep(1000);
	pthread_mutex_lock(&e->lock);
	for (i = 0; i < SDR_ENGINE_MAX_GROUPS; i++)
		if (e->groups[i] == g)
			e->groups[i] = NULL;
	e->tx_used -= g->n_tx;
	if (g->chain) {
		e->rx_chain -= g->n_rx;
		e->tx_chain -= g->n_tx;
	}
	pthread_mutex_unlock(&e->lock);
	if (__atomic_load_n(&g->rx_on, __ATOMIC_ACQUIRE) || __atomic_load_n(&g->tx_on, __ATOMIC_ACQUIRE)) {
		fprintf(stderr, "sdr_engine_group_remove: a device thread still uses the group after 2 s, not freed\n");
		return;
	}
	group_free(g);
}

/* a device thread takes the registry's changes, if it is free: adopted and
 * dropped groups, its list of bins rebuilt */
static void adopt(sdr_engine_t *e, int tx)
{
	sdr_group_t **list = tx ? e->tx_list : e->rx_list;
	int *n = tx ? &e->tx_n : &e->rx_n;
	int i, j, c, changed = 0;

	if (pthread_mutex_trylock(&e->lock))
		return;
	/* drop the ones leaving */
	for (j = 0; j < *n; ) {
		if (list[j]->leaving) {
			if (tx)
				__atomic_store_n(&list[j]->tx_on, 0, __ATOMIC_RELEASE);
			else
				__atomic_store_n(&list[j]->rx_on, 0, __ATOMIC_RELEASE);
			list[j] = list[--(*n)];
			changed = 1;
		} else
			j++;
	}
	/* adopt the new ones */
	for (i = 0; i < SDR_ENGINE_MAX_GROUPS; i++) {
		sdr_group_t *g = e->groups[i];

		if (!g || g->leaving || (tx ? !g->n_tx : !g->n_rx) || (tx ? g->tx_on : g->rx_on))
			continue;
		list[(*n)++] = g;
		if (tx)
			__atomic_store_n(&g->tx_on, 1, __ATOMIC_RELEASE);
		else
			__atomic_store_n(&g->rx_on, 1, __ATOMIC_RELEASE);
		changed = 1;
	}
	pthread_mutex_unlock(&e->lock);
	if (!changed)
		return;
	/* bins of all channels on the bank, in list order (chain groups have
	 * none) */
	c = 0;
	for (j = 0; j < *n; j++) {
		sdr_group_t *g = list[j];
		int k;

		if (g->chain)
			continue;
		if (tx) {
			for (k = 0; k < g->n_tx; k++)
				e->tx_bins[c++] = g->tx[k].bin;
		} else {
			for (k = 0; k < g->n_rx; k++) {
				e->rx_bins[c] = g->rx[k].bin;
				/* the bin buffers of channel k of this group */
				if (!e->rx_bin_i[c]) {
					e->rx_bin_i[c] = calloc(e->rx_frames_max, sizeof(sample_t));
					e->rx_bin_q[c] = calloc(e->rx_frames_max, sizeof(sample_t));
				}
				c++;
			}
		}
	}
	if (!tx)
		e->rx_bins_n = c;
}

void sdr_engine_rx_feed(sdr_engine_t *e, const float *iq, int n)
{
	int frames, j, k, c, s, out;

	adopt(e, 0);
	if (n > e->max_block) {
		fprintf(stderr, "sdr_engine_rx_feed: %d samples, more than %d, please fix!\n", n, e->max_block);
		abort();
	}
	frames = pfb_rx_process(&e->bank_rx, iq, n, e->rx_bins_n, e->rx_bins, e->rx_bin_i, e->rx_bin_q);
	c = 0;
	for (j = 0; j < e->rx_n; j++) {
		sdr_group_t *g = e->rx_list[j];
		sdr_ring_t *r = &g->rx_ring;
		int in, space;

		out = 0;
		for (k = 0; k < g->n_rx; k++) {
			int got;

			if (g->chain)
				got = channelizer_rx_process(&g->crx[k], iq, n, g->rx_max,
							     g->rx_out_i + k * g->rx_max, g->rx_out_q + k * g->rx_max);
			else {
				got = pfb_chan_rx_process(&g->rx[k], e->rx_bin_i[c], e->rx_bin_q[c], frames, g->rx_max,
							  g->rx_out_i + k * g->rx_max, g->rx_out_q + k * g->rx_max);
				c++;
			}
			if (k == 0)
				out = got;
			else if (got != out)
				fprintf(stderr, "sdr_engine_rx_feed: channels of a group out of step (%d / %d), please fix!\n", got, out);
		}
		/* rows into the ring, what does not fit is dropped */
		space = ring_space_dev(r);
		if (space < 0) {
			/* the program wrote an index out of range: nothing goes in */
			g->view->bad_index++;
			g->view->rx_dropped += out;
			continue;
		}
		in = r->pos;
		for (s = 0; s < out && s < space; s++) {
			float *row = r->buf + (size_t)in * r->width;

			for (k = 0; k < g->n_rx; k++) {
				row[2 * k] = g->rx_out_i[k * g->rx_max + s];
				row[2 * k + 1] = g->rx_out_q[k * g->rx_max + s];
			}
			if (++in == r->rows)
				in = 0;
		}
		r->pos = in;
		__atomic_store_n(&r->ctl->in, in, __ATOMIC_RELEASE);
		g->view->rx_rows += s;
		if (s < out)
			g->view->rx_dropped += out - s;
	}
}

/* need rows of the group's TX ring into tx_in_i / _q (channel k at
 * k * tx_max); the program fell behind: silence for the missing rows, so
 * the tails stay continuous (counted) */
static void tx_take(sdr_group_t *g, int need)
{
	sdr_ring_t *r = &g->tx_ring;
	int have = ring_fill_dev(r), take, out, s, k;

	if (need > g->tx_max) {
		fprintf(stderr, "sdr_engine_tx_produce: %d DSP samples needed, more than %d, please fix!\n", need, g->tx_max);
		abort();
	}
	if (have < 0) {
		/* the program wrote an index out of range: silence */
		g->view->bad_index++;
		have = 0;
	}
	take = (have < need) ? have : need;
	out = r->pos;
	for (s = 0; s < take; s++) {
		const float *row = r->buf + (size_t)out * r->width;

		for (k = 0; k < g->n_tx; k++) {
			g->tx_in_i[k * g->tx_max + s] = row[2 * k];
			g->tx_in_q[k * g->tx_max + s] = row[2 * k + 1];
		}
		if (++out == r->rows)
			out = 0;
	}
	r->pos = out;
	__atomic_store_n(&r->ctl->out, out, __ATOMIC_RELEASE);
	for (k = 0; k < g->n_tx; k++) {
		memset(g->tx_in_i + k * g->tx_max + take, 0, (need - take) * sizeof(sample_t));
		memset(g->tx_in_q + k * g->tx_max + take, 0, (need - take) * sizeof(sample_t));
	}
	g->view->tx_rows += take;
	if (take < need && g->view->tx_rows)
		g->view->tx_missing += need - take;
}

/* a chain group: exactly n SDR samples added to iq; its channelizers make
 * what the DSP samples give (not a fixed count per call), the surplus waits
 * in the group's FIFO for the next block */
static void tx_chain(sdr_engine_t *e, sdr_group_t *g, float *iq, int n)
{
	int s, k;

	while (g->tx_fifo_len < n) {
		/* DSP samples for what is missing, at least one */
		int need = (int)ceil((double)(n - g->tx_fifo_len) * g->dsp_rate / e->tx_rate), made = 0;

		if (need < 1)
			need = 1;
		if (need > g->tx_max)
			need = g->tx_max;
		if (g->tx_fifo_len + channelizer_tx_max(&g->ctx[0], need) > g->tx_fifo_size) {
			fprintf(stderr, "sdr_engine_tx_produce: chain FIFO of %d too small, please fix!\n", g->tx_fifo_size);
			abort();
		}
		tx_take(g, need);
		memset(g->tx_fifo + 2 * g->tx_fifo_len, 0, sizeof(float) * 2 * channelizer_tx_max(&g->ctx[0], need));
		for (k = 0; k < g->n_tx; k++) {
			int got = channelizer_tx_process(&g->ctx[k], g->tx_in_i + k * g->tx_max, g->tx_in_q + k * g->tx_max, need,
							 g->tx_fifo + 2 * g->tx_fifo_len);

			if (k == 0)
				made = got;
			else if (got != made)
				fprintf(stderr, "sdr_engine_tx_produce: chain channels of a group out of step (%d / %d), please fix!\n", got, made);
		}
		g->tx_fifo_len += made;
	}
	for (s = 0; s < 2 * n; s++)
		iq[s] += g->tx_fifo[s];
	g->tx_fifo_len -= n;
	memmove(g->tx_fifo, g->tx_fifo + 2 * n, sizeof(float) * 2 * g->tx_fifo_len);
}

void sdr_engine_tx_produce(sdr_engine_t *e, float *iq, int n)
{
	int frames = n / e->bank_tx.d, j, k, c, need;

	adopt(e, 1);
	memset(iq, 0, sizeof(*iq) * 2 * n);
	if (n % e->bank_tx.d || frames > e->tx_frames_max) {
		fprintf(stderr, "sdr_engine_tx_produce: %d samples, not a multiple of %d or more than %d, please fix!\n", n, e->bank_tx.d, e->max_block);
		abort();
	}
	c = 0;
	for (j = 0; j < e->tx_n; j++) {
		sdr_group_t *g = e->tx_list[j];

		if (g->chain) {
			tx_chain(e, g, iq, n);
			continue;
		}
		need = pfb_chan_tx_need(&g->tx[0], frames);
		tx_take(g, need);
		for (k = 0; k < g->n_tx; k++, c++) {
			pfb_chan_tx_pull(&g->tx[k], g->tx_in_i + k * g->tx_max, g->tx_in_q + k * g->tx_max, need, frames);
			e->tx_ci[c] = g->tx[k].out_i;
			e->tx_cq[c] = g->tx[k].out_q;
		}
	}
	if (c)
		pfb_tx_process(&e->bank_tx, c, e->tx_bins, e->tx_ci, e->tx_cq, frames, iq);
}

/*
 * program side
 */

int sdr_port_attach(sdr_port_t *p, void *mem, size_t size)
{
	sdr_view_t *v = mem;

	memset(p, 0, sizeof(*p));
	if (!v || size < sizeof(*v) || __atomic_load_n(&v->magic, __ATOMIC_ACQUIRE) != SDR_VIEW_MAGIC || v->version != SDR_VIEW_VERSION)
		return -EINVAL;
	if (v->n_rx < 0 || v->n_rx > SDR_ENGINE_MAX_CHAN || v->n_tx < 0 || v->n_tx > SDR_ENGINE_MAX_CHAN || v->rows < 2
	 || v->tx_target < 0 || v->tx_target >= v->rows || v->size > size
	 || sdr_view_size(v->n_rx, v->n_tx, v->rows) > v->size)
		return -EINVAL;
	p->view = v;
	p->n_rx = v->n_rx;
	p->n_tx = v->n_tx;
	p->tx_target = v->tx_target;
	view_rings(v, p->n_rx, p->n_tx, v->rows, &p->rx, &p->tx);
	return 0;
}

int sdr_port_read(sdr_port_t *p, sample_t **out_i, sample_t **out_q, int num)
{
	sdr_ring_t *r = &p->rx;
	int n, out, s, k;

	if (!p->n_rx)
		return 0;
	n = sdr_ring_fill(r);
	if (n < 0)
		return 0;
	out = r->ctl->out;
	if (n > num)
		n = num;
	for (s = 0; s < n; s++) {
		const float *row = r->buf + (size_t)out * r->width;

		for (k = 0; k < p->n_rx; k++) {
			out_i[k][s] = row[2 * k];
			out_q[k][s] = row[2 * k + 1];
		}
		if (++out == r->rows)
			out = 0;
	}
	__atomic_store_n(&r->ctl->out, out, __ATOMIC_RELEASE);
	return n;
}

int sdr_port_write(sdr_port_t *p, sample_t **in_i, sample_t **in_q, int num)
{
	sdr_ring_t *r = &p->tx;
	int n, in, s, k;

	if (!p->n_tx)
		return 0;
	n = sdr_ring_space(r);
	if (n < 0)
		return 0;
	in = r->ctl->in;
	if (n > num)
		n = num;
	for (s = 0; s < n; s++) {
		float *row = r->buf + (size_t)in * r->width;

		for (k = 0; k < p->n_tx; k++) {
			row[2 * k] = in_i[k][s];
			row[2 * k + 1] = in_q[k][s];
		}
		if (++in == r->rows)
			in = 0;
	}
	__atomic_store_n(&r->ctl->in, in, __ATOMIC_RELEASE);
	return n;
}

int sdr_port_tx_fill(sdr_port_t *p)
{
	int fill = p->n_tx ? sdr_ring_fill(&p->tx) : 0;

	return (fill < 0) ? 0 : fill;
}

int sdr_port_tosend(sdr_port_t *p)
{
	int n = p->tx_target - sdr_port_tx_fill(p);

	return (p->n_tx && n > 0) ? n : 0;
}

int sdr_engine_group_read(sdr_group_t *g, sample_t **out_i, sample_t **out_q, int num)
{
	return sdr_port_read(&g->port, out_i, out_q, num);
}

int sdr_engine_group_write(sdr_group_t *g, sample_t **in_i, sample_t **in_q, int num)
{
	return sdr_port_write(&g->port, in_i, in_q, num);
}

int sdr_engine_group_tosend(sdr_group_t *g)
{
	return sdr_port_tosend(&g->port);
}
