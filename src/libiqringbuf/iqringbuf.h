/* IQ Ring Buffer — threaded SDR sample reader
 *
 * Decouples SDR USB/network receive timing from signal processing.
 * A background thread continuously reads IQ samples into a circular
 * buffer; the consumer reads at its own pace without blocking the SDR.
 *
 * This prevents USB overflows (the "OOOO" problem) at high sample
 * rates where processing takes longer than the buffer fill time.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef IQRINGBUF_H
#define IQRINGBUF_H

#include <pthread.h>

/* Receive callback: reads IQ samples from SDR hardware.
 * Must fill buffer with interleaved float IQ pairs.
 * Returns number of IQ samples read, or <=0 on error. */
typedef int (*iqring_recv_fn)(float *buffer, int max_samples);

typedef struct iqringbuf {
	float		*buf;		/* Ring buffer (interleaved IQ) */
	int		capacity;	/* Total IQ samples in ring */
	int		rd;		/* Read position (consumer) */
	int		wr;		/* Write position (producer) */

	pthread_t	thread;		/* Reader thread */
	pthread_mutex_t	mutex;
	pthread_cond_t	cond;		/* Signals consumer when data available */
	int		running;	/* 1 while thread should run */

	iqring_recv_fn	recv_fn;	/* SDR receive callback */
	int		chunk_size;	/* Samples per read call */

	/* Statistics */
	int		overflows;	/* Times writer caught up to reader */
	long long	total_read;	/* Total samples read from SDR */
	long long	total_consumed;	/* Total samples consumed */
} iqringbuf_t;

/* Initialize ring buffer.
 *   duration_sec: buffer duration in seconds (e.g. 1.0)
 *   samplerate:   IQ sample rate in Hz
 *   chunk_size:   samples per SDR read call (e.g. 8192)
 *   recv_fn:      SDR receive function pointer
 * Returns 0 on success, <0 on error. */
int iqringbuf_init(iqringbuf_t *rb, double duration_sec, int samplerate,
                   int chunk_size, iqring_recv_fn recv_fn);

/* Start the reader thread. Call after SDR is opened and streaming. */
int iqringbuf_start(iqringbuf_t *rb);

/* Read IQ samples from the ring buffer (blocking).
 * Waits until requested samples are available.
 * Returns number of IQ samples copied to output. */
int iqringbuf_read(iqringbuf_t *rb, float *output, int num_samples);

/* Number of samples currently available for reading */
int iqringbuf_available(iqringbuf_t *rb);

/* Stop the reader thread and free resources */
void iqringbuf_stop(iqringbuf_t *rb);

/* Free all allocated memory (call after stop) */
void iqringbuf_free(iqringbuf_t *rb);

#endif /* IQRINGBUF_H */
