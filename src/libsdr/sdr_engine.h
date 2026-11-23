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
#include "../libsample/sample.h"
#include "../libchannelizer/pfb.h"

#define SDR_ENGINE_MAX_GROUPS	64
#define SDR_ENGINE_MAX_CHAN	128	/* channels in one group, per direction */

/* single producer, single consumer ring of rows (complex float per channel) */
typedef struct sdr_ring {
	float		*buf;		/* rows * width floats */
	int		width;		/* floats per row (2 per channel) */
	int		rows;		/* capacity (one row stays free) */
	volatile int	in, out;	/* row indices */
} sdr_ring_t;

typedef struct sdr_engine sdr_engine_t;

typedef struct sdr_group {
	int		id;
	int		dsp_rate;
	int		n_rx, n_tx;
	double		rx_offset[SDR_ENGINE_MAX_CHAN], tx_offset[SDR_ENGINE_MAX_CHAN];
	pfb_chan_t	*rx, *tx;	/* channel tails */
	sdr_ring_t	rx_ring, tx_ring;
	int		tx_target;	/* rows the program keeps in the TX ring */
	/* counters (read by anyone, written by the device threads) */
	volatile long	rx_rows, rx_dropped;	/* rows written / dropped (ring full) */
	volatile long	tx_rows, tx_missing;	/* rows taken / missing (ring empty: silence) */
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
 * DSP samples, tx_target: TX ring fill the program keeps (rows). Returns
 * the group or NULL with a reason in sdr_engine_error(). */
sdr_group_t *sdr_engine_group_add(sdr_engine_t *e, int dsp_rate, int n_rx, const double *rx_offset, int n_tx, const double *tx_offset, int ring_ms, int tx_target);
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

/* ring (also for the backend's shared memory) */
int sdr_ring_init(sdr_ring_t *r, int width, int rows);
void sdr_ring_exit(sdr_ring_t *r);
int sdr_ring_fill(const sdr_ring_t *r);
int sdr_ring_space(const sdr_ring_t *r);

#endif /* _SDR_ENGINE_H */
