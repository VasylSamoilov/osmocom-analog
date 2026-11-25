/* SDR engine: filter banks, registered channel groups, DSP rate rings
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The engine sits between the device and the programs (docs/SDR-BACKEND.md):
 *
 *   device RX thread -> sdr_engine_rx_feed(): RX bank, every group's channel
 *       tails, one row per DSP sample into the group's RX ring
 *   device TX thread -> sdr_engine_tx_produce(): every group's TX tails pull
 *       exactly the frames of the block from the group's TX ring (rows), the
 *       TX bank adds them up
 *   program -> sdr_engine_group_read() / _write() / _tosend() at its DSP rate
 *
 * A group is one program's channels (RX and TX) at one DSP rate. A row of a
 * ring holds one complex sample of every channel of the group, so all
 * channels of a group always have the same number of samples.
 *
 * The device threads never wait: groups are built completely by the caller
 * of sdr_engine_group_add() (filter design takes time) and adopted or
 * dropped by each device thread at the start of a block, if the registry is
 * free (trylock); a group a program does not keep up with gets silence on TX
 * (counted) and loses rows on RX (counted).
 */

#ifndef _SDR_ENGINE_H
#define _SDR_ENGINE_H

#include <pthread.h>
#include <stdint.h>
#include <stddef.h>
#include "../libsample/sample.h"
#include "../libchannelizer/pfb.h"
#include "../libchannelizer/channelizer.h"

#define SDR_ENGINE_MAX_GROUPS	64
#define SDR_ENGINE_MAX_CHAN	128	/* channels in one group, per direction */

/* the indices of a ring, where both sides see them (a view) */
typedef struct sdr_ring_ctl {
	volatile int32_t in, out;	/* row indices */
} sdr_ring_ctl_t;

/* single producer, single consumer ring of rows (complex float per channel):
 * each side has its own descriptor (its size is its own, never read from
 * the other side), buffer and indices may be shared memory */
typedef struct sdr_ring {
	float		*buf;		/* rows * width floats */
	int		width;		/* floats per row (2 per channel) */
	int		rows;		/* capacity (one row stays free) */
	sdr_ring_ctl_t	*ctl;
	int		pos;		/* the device threads' own index (in for RX, out for
					 * TX): kept here, only published to ctl, never read back */
} sdr_ring_t;

/* A group's view: everything the program side uses, in one block without
 * pointers, so it can be shared memory (backend process, docs/SDR-BACKEND.md):
 * the header, then the RX ring's rows, then the TX ring's rows. */
#define SDR_VIEW_MAGIC		0x56524453	/* "SDRV" */
#define SDR_VIEW_VERSION	1

typedef struct sdr_view {
	uint32_t	magic, version;
	uint64_t	size;		/* bytes of the whole block */
	int32_t		dsp_rate, n_rx, n_tx, rows;
	int32_t		tx_target;	/* rows the program keeps in the TX ring */
	double		tx_scale;	/* TX amplitude of every channel (1 / N max) */
	/* counters, written by the device threads */
	volatile int64_t rx_rows, rx_dropped;	/* rows written / dropped (ring full) */
	volatile int64_t tx_rows, tx_missing;	/* rows taken / missing (ring empty: silence) */
	volatile int64_t bad_index;		/* indices out of range seen by the device threads */
	sdr_ring_ctl_t	rx_ctl, tx_ctl;
} sdr_view_t;

/* the program side of a group */
typedef struct sdr_port {
	sdr_view_t	*view;
	int		n_rx, n_tx, tx_target;
	sdr_ring_t	rx, tx;
} sdr_port_t;

typedef struct sdr_engine sdr_engine_t;

typedef struct sdr_group {
	int		id;
	int		dsp_rate;
	int		n_rx, n_tx;
	double		rx_offset[SDR_ENGINE_MAX_CHAN], tx_offset[SDR_ENGINE_MAX_CHAN];
	pfb_chan_t	*rx, *tx;	/* channel tails (on the bank) */
	/* a DSP rate above the bank's (broadcast FM next to NFM): every channel
	 * a channelizer on the SDR stream instead of a tail on the bank */
	int		chain;
	channelizer_t	*crx, *ctx;
	float		*tx_fifo;	/* SDR rate IQ made, not yet given (all TX channels added) */
	int		tx_fifo_len, tx_fifo_size;
	sdr_view_t	*view;		/* rings and counters (own memory or the caller's) */
	int		view_own;
	sdr_ring_t	rx_ring, tx_ring;	/* the device threads' descriptors */
	int		tx_target;
	sdr_port_t	port;		/* the program side, in-process */
	/* adoption by the device threads */
	volatile int	rx_on, tx_on;		/* adopted by the RX / TX thread */
	volatile int	leaving;		/* remove requested */
	/* work */
	sample_t	*tx_in_i, *tx_in_q;	/* one channel's DSP samples for a pull */
	sample_t	*rx_out_i, *rx_out_q;	/* one channel's DSP samples of a block */
	int		rx_max, tx_max;
} sdr_group_t;

struct sdr_engine {
	int		rx_rate, tx_rate;	/* SDR rates (0: direction not used) */
	int		max_dsp;		/* highest DSP rate the banks carry */
	int		tx_nmax;		/* most TX channels: each gets 1 / tx_nmax of full scale */
	int		tx_used;		/* TX channels registered */
	pfb_t		bank_rx, bank_tx;
	int		max_block;		/* most SDR samples per feed / produce */
	/* registry: groups[] is changed under 'lock' only; each device thread
	 * keeps its own list of adopted groups */
	pthread_mutex_t	lock;
	sdr_group_t	*groups[SDR_ENGINE_MAX_GROUPS];
	int		next_id;
	/* RX thread */
	sdr_group_t	*rx_list[SDR_ENGINE_MAX_GROUPS];
	int		rx_n;
	int		*rx_bins;		/* bins of all adopted RX channels */
	sample_t	**rx_bin_i, **rx_bin_q;
	int		rx_bins_n, rx_frames_max;
	/* TX thread */
	sdr_group_t	*tx_list[SDR_ENGINE_MAX_GROUPS];
	int		tx_n;
	int		*tx_bins;
	const sample_t	**tx_ci, **tx_cq;
	int		tx_frames_max;
};

/* rx_rate / tx_rate: SDR rates (0 for a direction not used), max_dsp: the
 * highest DSP rate of a group, tx_nmax: most TX channels at once, max_block:
 * most SDR samples per sdr_engine_rx_feed() / _tx_produce() call. The
 * number of sub-bands is chosen for the rates (cheapest by estimate). */
int sdr_engine_init(sdr_engine_t *e, int rx_rate, int tx_rate, int max_dsp, int tx_nmax, int max_block);
const char *sdr_engine_error(void);
void sdr_engine_exit(sdr_engine_t *e);
/* bins chosen for these rates (0 = no bank fits), also used by sdr.c */
int sdr_engine_bins(int fs, int dsp, int nchan);

/* Build and register a group (the caller's thread; takes the filter design
 * time). Offsets in Hz from the SDR centre. ring_ms: ring length in ms of
 * DSP samples, tx_target: TX ring fill the program keeps (rows). A DSP
 * rate above max_dsp (wider than the bank's sub-bands, e.g. broadcast FM)
 * gets the per-channel chain on the SDR stream (docs/SDR-BACKEND.md,
 * measured cheaper than a bank for it). Returns the group or NULL with a
 * reason in sdr_engine_error(). */
sdr_group_t *sdr_engine_group_add(sdr_engine_t *e, int dsp_rate, int n_rx, const double *rx_offset, int n_tx, const double *tx_offset, int ring_ms, int tx_target);
/* the same with the view in the caller's memory (mem, size; zeroed, at
 * least sdr_view_size() of the group: shared memory for the backend) */
sdr_group_t *sdr_engine_group_add_mem(sdr_engine_t *e, int dsp_rate, int n_rx, const double *rx_offset, int n_tx, const double *tx_offset, int ring_ms, int tx_target, void *mem, size_t size);
/* ring rows for a ring of ring_ms, bytes of a view */
int sdr_engine_rows(int dsp_rate, int ring_ms);
size_t sdr_view_size(int n_rx, int n_tx, int rows);
/* ask the device threads to drop it, wait until they did, free it */
void sdr_engine_group_remove(sdr_engine_t *e, sdr_group_t *g);
/* TX amplitude of every channel (1 / tx_nmax of full scale) */
double sdr_engine_tx_scale(const sdr_engine_t *e);

/* device threads: n SDR samples (interleaved float) in / out (out is
 * written, not added; n a multiple of the TX bank's M/2) */
void sdr_engine_rx_feed(sdr_engine_t *e, const float *iq, int n);
void sdr_engine_tx_produce(sdr_engine_t *e, float *iq, int n);
/* SDR samples per TX bank frame (sdr_engine_tx_produce() wants multiples) */
int sdr_engine_tx_step(const sdr_engine_t *e);

/* program side: up to num DSP samples of every RX channel (out_i[c][s]);
 * returns the number. Write num of every TX channel; returns how many fit.
 * tosend: how many to write to keep the TX ring at its target. */
int sdr_engine_group_read(sdr_group_t *g, sample_t **out_i, sample_t **out_q, int num);
int sdr_engine_group_write(sdr_group_t *g, sample_t **in_i, sample_t **in_q, int num);
int sdr_engine_group_tosend(sdr_group_t *g);

/* program side on a view (in-process: the group's port; a client of the
 * backend: its mapped view). attach checks the view (magic, version, sizes
 * within size); returns 0 or -EINVAL. */
int sdr_port_attach(sdr_port_t *p, void *mem, size_t size);
int sdr_port_read(sdr_port_t *p, sample_t **out_i, sample_t **out_q, int num);
int sdr_port_write(sdr_port_t *p, sample_t **in_i, sample_t **in_q, int num);
int sdr_port_tosend(sdr_port_t *p);
int sdr_port_tx_fill(sdr_port_t *p);	/* rows in the TX ring */

/* ring on a buffer and indices (shared or not); fill: rows in it, -1 if an
 * index is out of range (the other side wrote it) */
void sdr_ring_set(sdr_ring_t *r, float *buf, int width, int rows, sdr_ring_ctl_t *ctl);
int sdr_ring_fill(const sdr_ring_t *r);
int sdr_ring_space(const sdr_ring_t *r);

#endif /* _SDR_ENGINE_H */
