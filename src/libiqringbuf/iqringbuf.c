/* IQ Ring Buffer — threaded SDR sample reader
 *
 * Background thread reads IQ samples from SDR into a circular buffer.
 * Consumer thread reads from the buffer at its own pace.
 * Mutex + condvar for synchronization; overflow detection.
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
#include <unistd.h>
#include "iqringbuf.h"

int iqringbuf_init(iqringbuf_t *rb, double duration_sec, int samplerate,
                   int chunk_size, iqring_recv_fn recv_fn)
{
	memset(rb, 0, sizeof(*rb));

	rb->capacity = (int)(duration_sec * samplerate);
	if (rb->capacity < chunk_size * 4)
		rb->capacity = chunk_size * 4;

	/* Round up to multiple of chunk_size for clean wrapping */
	rb->capacity = ((rb->capacity + chunk_size - 1) / chunk_size) * chunk_size;

	rb->buf = calloc(rb->capacity * 2, sizeof(float));  /* ×2 for IQ */
	if (!rb->buf)
		return -1;

	rb->rd = 0;
	rb->wr = 0;
	rb->recv_fn = recv_fn;
	rb->chunk_size = chunk_size;
	rb->running = 0;
	rb->overflows = 0;
	rb->total_read = 0;
	rb->total_consumed = 0;

	pthread_mutex_init(&rb->mutex, NULL);
	pthread_cond_init(&rb->cond, NULL);

	return 0;
}

/* Reader thread: continuously reads from SDR into ring buffer */
static void *reader_thread(void *arg)
{
	iqringbuf_t *rb = (iqringbuf_t *)arg;
	float *tmp = malloc(rb->chunk_size * 2 * sizeof(float));
	if (!tmp) {
		fprintf(stderr, "iqringbuf: reader thread malloc failed\n");
		return NULL;
	}

	while (rb->running) {
		int got = rb->recv_fn(tmp, rb->chunk_size);
		if (got <= 0) {
			usleep(100);
			continue;
		}

		pthread_mutex_lock(&rb->mutex);

		/* Copy samples into ring buffer */
		int wr = rb->wr;
		for (int i = 0; i < got; i++) {
			int next_wr = (wr + 1) % rb->capacity;
			if (next_wr == rb->rd) {
				/* Overflow: drop oldest sample by advancing reader */
				rb->rd = (rb->rd + 1) % rb->capacity;
				rb->overflows++;
			}
			rb->buf[wr * 2]     = tmp[i * 2];
			rb->buf[wr * 2 + 1] = tmp[i * 2 + 1];
			wr = next_wr;
		}
		rb->wr = wr;
		rb->total_read += got;

		/* Wake up consumer */
		pthread_cond_signal(&rb->cond);
		pthread_mutex_unlock(&rb->mutex);
	}

	free(tmp);
	return NULL;
}

int iqringbuf_start(iqringbuf_t *rb)
{
	if (rb->running)
		return 0;

	rb->running = 1;
	int rc = pthread_create(&rb->thread, NULL, reader_thread, rb);
	if (rc != 0) {
		rb->running = 0;
		fprintf(stderr, "iqringbuf: failed to create reader thread\n");
		return -1;
	}

	return 0;
}

static int available_locked(iqringbuf_t *rb)
{
	int avail = rb->wr - rb->rd;
	if (avail < 0)
		avail += rb->capacity;
	return avail;
}

int iqringbuf_available(iqringbuf_t *rb)
{
	pthread_mutex_lock(&rb->mutex);
	int avail = available_locked(rb);
	pthread_mutex_unlock(&rb->mutex);
	return avail;
}

int iqringbuf_read(iqringbuf_t *rb, float *output, int num_samples)
{
	int copied = 0;

	pthread_mutex_lock(&rb->mutex);

	while (copied < num_samples) {
		/* Wait for data */
		while (available_locked(rb) == 0 && rb->running)
			pthread_cond_wait(&rb->cond, &rb->mutex);

		if (!rb->running && available_locked(rb) == 0)
			break;

		/* Copy available samples */
		int avail = available_locked(rb);
		int want = num_samples - copied;
		if (want > avail)
			want = avail;

		for (int i = 0; i < want; i++) {
			output[copied * 2]     = rb->buf[rb->rd * 2];
			output[copied * 2 + 1] = rb->buf[rb->rd * 2 + 1];
			rb->rd = (rb->rd + 1) % rb->capacity;
			copied++;
		}
		rb->total_consumed += want;
	}

	pthread_mutex_unlock(&rb->mutex);
	return copied;
}

void iqringbuf_stop(iqringbuf_t *rb)
{
	if (!rb->running)
		return;

	rb->running = 0;

	/* Wake consumer in case it's waiting */
	pthread_mutex_lock(&rb->mutex);
	pthread_cond_signal(&rb->cond);
	pthread_mutex_unlock(&rb->mutex);

	pthread_join(rb->thread, NULL);
}

void iqringbuf_free(iqringbuf_t *rb)
{
	if (rb->buf) {
		free(rb->buf);
		rb->buf = NULL;
	}
	pthread_mutex_destroy(&rb->mutex);
	pthread_cond_destroy(&rb->cond);
}
