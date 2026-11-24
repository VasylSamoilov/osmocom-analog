/* SDR backend: programs register their channels on one SDR engine
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * docs/SDR-BACKEND.md, transport: local only. The backend listens on a Unix
 * socket (SOCK_SEQPACKET; a name starting with '@' is in the abstract
 * namespace). A program connects and sends one request (its channels, DSP
 * rate, ring length); the answer carries the result and, if accepted, a
 * memfd with the group's view (sdr_view_t: rings and counters), sealed
 * against resizing, which both map. The connection is the registration:
 * closed (the program ended or crashed), the backend removes the group.
 */

#ifndef _SDR_BACKEND_H
#define _SDR_BACKEND_H

#include <stdint.h>
#include "sdr_engine.h"

#define SDR_BACKEND_MAGIC	0x4b424453	/* "SDBK" */
#define SDR_BACKEND_PROTO	1
#define SDR_BACKEND_MAX_RING_MS	2000

/* program -> backend */
typedef struct sdr_backend_req {
	uint32_t	magic, proto;
	char		name[32];
	int32_t		dsp_rate;
	int32_t		n_rx, n_tx;
	int32_t		ring_ms;	/* ring length (ms of DSP samples) */
	int32_t		tx_target;	/* rows the program keeps in the TX ring */
	double		rx_freq[SDR_ENGINE_MAX_CHAN];	/* Hz */
	double		tx_freq[SDR_ENGINE_MAX_CHAN];
} sdr_backend_req_t;

/* backend -> program (with the memfd if ok) */
typedef struct sdr_backend_ans {
	uint32_t	magic, proto;
	int32_t		ok;
	char		error[200];
	uint64_t	view_size;
	double		tx_scale;	/* TX amplitude of every channel */
	double		rx_centre, tx_centre;	/* the SDR's frequencies (Hz) */
} sdr_backend_ans_t;

/* the socket address of a name ('@name': abstract namespace) */
struct sockaddr_un;
int sdr_backend_addr(const char *name, struct sockaddr_un *a, unsigned int *len);

/* backend side: a control thread accepts programs for the engine e, whose
 * device threads the caller runs; centre frequencies to make the offsets */
typedef struct sdr_backend sdr_backend_t;
sdr_backend_t *sdr_backend_start(sdr_engine_t *e, const char *socket_name, double rx_centre, double tx_centre);
void sdr_backend_stop(sdr_backend_t *b);
const char *sdr_backend_error(void);
int sdr_backend_groups(sdr_backend_t *b);	/* programs registered */

/* program side */
typedef struct sdr_client {
	int		sock;
	void		*mem;
	size_t		size;
	sdr_port_t	port;
	double		tx_scale;
	double		rx_centre, tx_centre;
} sdr_client_t;

/* register; returns 0, or < 0 with the reason in error */
int sdr_client_open(sdr_client_t *c, const char *socket_name, const char *name, int dsp_rate,
		    int n_rx, const double *rx_freq, int n_tx, const double *tx_freq,
		    int ring_ms, int tx_target, char *error, int error_len);
void sdr_client_close(sdr_client_t *c);
/* 0 if the backend is gone (the connection closed) */
int sdr_client_alive(sdr_client_t *c);

#endif /* _SDR_BACKEND_H */
