/* SDR backend: the program side (register, map the view)
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "sdr_backend.h"

int sdr_client_open(sdr_client_t *c, const char *socket_name, const char *name, int dsp_rate,
		    int n_rx, const double *rx_freq, int n_tx, const double *tx_freq,
		    int ring_ms, int tx_target, char *error, int error_len)
{
	sdr_backend_req_t req;
	sdr_backend_ans_t ans;
	struct sockaddr_un a;
	socklen_t len;
	struct msghdr m;
	struct iovec iov = { &ans, sizeof(ans) };
	char cbuf[CMSG_SPACE(sizeof(int))];
	struct cmsghdr *cm;
	struct stat st;
	int fd = -1, i;
	ssize_t n;

	memset(c, 0, sizeof(*c));
	c->sock = -1;
	if (n_rx < 0 || n_rx > SDR_ENGINE_MAX_CHAN || n_tx < 0 || n_tx > SDR_ENGINE_MAX_CHAN) {
		snprintf(error, error_len, "%d RX / %d TX channels, at most %d each", n_rx, n_tx, SDR_ENGINE_MAX_CHAN);
		return -EINVAL;
	}
	if (sdr_backend_addr(socket_name, &a, &len) < 0) {
		snprintf(error, error_len, "socket name '%s' empty or too long", socket_name);
		return -EINVAL;
	}
	c->sock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	if (c->sock < 0 || connect(c->sock, (struct sockaddr *)&a, len) < 0) {
		snprintf(error, error_len, "SDR backend '%s': %s", socket_name, strerror(errno));
		goto fail;
	}
	memset(&req, 0, sizeof(req));
	req.magic = SDR_BACKEND_MAGIC;
	req.proto = SDR_BACKEND_PROTO;
	snprintf(req.name, sizeof(req.name), "%s", name);
	req.dsp_rate = dsp_rate;
	req.n_rx = n_rx;
	req.n_tx = n_tx;
	req.ring_ms = ring_ms;
	req.tx_target = tx_target;
	for (i = 0; i < n_rx; i++)
		req.rx_freq[i] = rx_freq[i];
	for (i = 0; i < n_tx; i++)
		req.tx_freq[i] = tx_freq[i];
	if (send(c->sock, &req, sizeof(req), MSG_NOSIGNAL) != sizeof(req)) {
		snprintf(error, error_len, "SDR backend: request not sent: %s", strerror(errno));
		goto fail;
	}
	memset(&m, 0, sizeof(m));
	m.msg_iov = &iov;
	m.msg_iovlen = 1;
	m.msg_control = cbuf;
	m.msg_controllen = sizeof(cbuf);
	n = recvmsg(c->sock, &m, MSG_CMSG_CLOEXEC);
	if (n != sizeof(ans) || ans.magic != SDR_BACKEND_MAGIC || ans.proto != SDR_BACKEND_PROTO) {
		snprintf(error, error_len, "SDR backend: no valid answer (protocol %d)", SDR_BACKEND_PROTO);
		goto fail;
	}
	for (cm = CMSG_FIRSTHDR(&m); cm; cm = CMSG_NXTHDR(&m, cm))
		if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS)
			memcpy(&fd, CMSG_DATA(cm), sizeof(int));
	ans.error[sizeof(ans.error) - 1] = '\0';
	if (!ans.ok) {
		snprintf(error, error_len, "SDR backend refused: %s", ans.error);
		goto fail;
	}
	/* the view: its size from the memory itself, not from the answer only */
	if (fd < 0 || fstat(fd, &st) < 0 || (uint64_t)st.st_size < ans.view_size || ans.view_size < sizeof(sdr_view_t)) {
		snprintf(error, error_len, "SDR backend: no valid shared memory");
		goto fail;
	}
	c->size = ans.view_size;
	c->mem = mmap(NULL, c->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	fd = -1;
	if (c->mem == MAP_FAILED) {
		c->mem = NULL;
		snprintf(error, error_len, "SDR backend: shared memory: %s", strerror(errno));
		goto fail;
	}
	if (sdr_port_attach(&c->port, c->mem, c->size) < 0 || c->port.n_rx != n_rx || c->port.n_tx != n_tx) {
		snprintf(error, error_len, "SDR backend: the shared view does not match the request");
		goto fail;
	}
	c->tx_scale = ans.tx_scale;
	c->rx_centre = ans.rx_centre;
	c->tx_centre = ans.tx_centre;
	return 0;

fail:
	if (fd >= 0)
		close(fd);
	sdr_client_close(c);
	return -EIO;
}

void sdr_client_close(sdr_client_t *c)
{
	if (c->mem)
		munmap(c->mem, c->size);
	if (c->sock >= 0)
		close(c->sock);
	memset(c, 0, sizeof(*c));
	c->sock = -1;
}

int sdr_client_alive(sdr_client_t *c)
{
	struct pollfd p = { c->sock, POLLIN, 0 };
	char b;

	if (c->sock < 0)
		return 0;
	if (poll(&p, 1, 0) <= 0)
		return 1;
	/* readable: closed (0) or an error; the backend sends nothing else */
	if (p.revents & (POLLHUP | POLLERR))
		return 0;
	return recv(c->sock, &b, 1, MSG_PEEK | MSG_DONTWAIT) != 0;
}
