/* SDR backend: the control thread that registers the programs' channels
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
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/mman.h>
#include "sdr_backend.h"

#define MAX_PROGRAMS	SDR_ENGINE_MAX_GROUPS

typedef struct program {
	int		sock;		/* -1: free */
	sdr_group_t	*group;		/* NULL: not registered (yet) */
	int		memfd;
	void		*mem;
	size_t		size;
	char		name[32];
} program_t;

struct sdr_backend {
	sdr_engine_t	*e;
	double		rx_centre, tx_centre;
	int		listen_sock;
	int		wake[2];	/* stop */
	pthread_t	thread;
	program_t	prog[MAX_PROGRAMS];
	volatile int	n_groups;
};

static char error_text[256];

const char *sdr_backend_error(void)
{
	return error_text;
}

int sdr_backend_addr(const char *name, struct sockaddr_un *a, unsigned int *len)
{
	size_t n = strlen(name);

	memset(a, 0, sizeof(*a));
	a->sun_family = AF_UNIX;
	if (n == 0 || n >= sizeof(a->sun_path))
		return -EINVAL;
	memcpy(a->sun_path, name, n);
	if (name[0] == '@')
		a->sun_path[0] = '\0';
	*len = (unsigned int)(offsetof(struct sockaddr_un, sun_path) + n);
	return 0;
}

static void program_drop(sdr_backend_t *b, program_t *p)
{
	if (p->group) {
		sdr_engine_group_remove(b->e, p->group);
		p->group = NULL;
		b->n_groups--;
		fprintf(stderr, "SDR backend: '%s' left, its channels removed\n", p->name);
	}
	if (p->mem)
		munmap(p->mem, p->size);
	if (p->memfd >= 0)
		close(p->memfd);
	if (p->sock >= 0)
		close(p->sock);
	memset(p, 0, sizeof(*p));
	p->sock = -1;
	p->memfd = -1;
}

static void answer(int sock, sdr_backend_ans_t *ans, int fd)
{
	struct msghdr m;
	struct iovec iov = { ans, sizeof(*ans) };
	char cbuf[CMSG_SPACE(sizeof(int))];

	ans->magic = SDR_BACKEND_MAGIC;
	ans->proto = SDR_BACKEND_PROTO;
	memset(&m, 0, sizeof(m));
	m.msg_iov = &iov;
	m.msg_iovlen = 1;
	if (fd >= 0) {
		struct cmsghdr *c;

		memset(cbuf, 0, sizeof(cbuf));
		m.msg_control = cbuf;
		m.msg_controllen = sizeof(cbuf);
		c = CMSG_FIRSTHDR(&m);
		c->cmsg_level = SOL_SOCKET;
		c->cmsg_type = SCM_RIGHTS;
		c->cmsg_len = CMSG_LEN(sizeof(int));
		memcpy(CMSG_DATA(c), &fd, sizeof(int));
	}
	if (sendmsg(sock, &m, MSG_NOSIGNAL) < 0)
		fprintf(stderr, "SDR backend: answer not sent: %s\n", strerror(errno));
}

/* one request of a program */
static void program_request(sdr_backend_t *b, program_t *p)
{
	sdr_backend_req_t req;
	sdr_backend_ans_t ans;
	double rx_off[SDR_ENGINE_MAX_CHAN], tx_off[SDR_ENGINE_MAX_CHAN];
	ssize_t n;
	int c, rows;

	n = recv(p->sock, &req, sizeof(req), 0);
	if (n <= 0) {
		/* closed (or an error): the program is gone */
		program_drop(b, p);
		return;
	}
	memset(&ans, 0, sizeof(ans));
	if (n != sizeof(req) || req.magic != SDR_BACKEND_MAGIC || req.proto != SDR_BACKEND_PROTO) {
		snprintf(ans.error, sizeof(ans.error), "not a request of protocol %d", SDR_BACKEND_PROTO);
		goto refuse;
	}
	if (p->group) {
		snprintf(ans.error, sizeof(ans.error), "already registered (one registration per connection)");
		goto refuse;
	}
	req.name[sizeof(req.name) - 1] = '\0';
	if (req.n_rx < 0 || req.n_rx > SDR_ENGINE_MAX_CHAN || req.n_tx < 0 || req.n_tx > SDR_ENGINE_MAX_CHAN || req.n_rx + req.n_tx == 0
	 || req.dsp_rate <= 0 || req.ring_ms <= 0 || req.ring_ms > SDR_BACKEND_MAX_RING_MS || req.tx_target < 0) {
		snprintf(ans.error, sizeof(ans.error), "request out of range (channels 1..%d each, ring 1..%d ms)", SDR_ENGINE_MAX_CHAN, SDR_BACKEND_MAX_RING_MS);
		goto refuse;
	}
	for (c = 0; c < req.n_rx; c++)
		rx_off[c] = req.rx_freq[c] - b->rx_centre;
	for (c = 0; c < req.n_tx; c++)
		tx_off[c] = req.tx_freq[c] - b->tx_centre;
	/* the view in sealed shared memory */
	rows = sdr_engine_rows(req.dsp_rate, req.ring_ms);
	p->size = sdr_view_size(req.n_rx, req.n_tx, rows);
	p->memfd = memfd_create("sdr-backend-view", MFD_CLOEXEC | MFD_ALLOW_SEALING);
	if (p->memfd < 0 || ftruncate(p->memfd, (off_t)p->size) < 0
	 || fcntl(p->memfd, F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_SEAL) < 0) {
		snprintf(ans.error, sizeof(ans.error), "shared memory: %s", strerror(errno));
		goto refuse;
	}
	p->mem = mmap(NULL, p->size, PROT_READ | PROT_WRITE, MAP_SHARED, p->memfd, 0);
	if (p->mem == MAP_FAILED) {
		p->mem = NULL;
		snprintf(ans.error, sizeof(ans.error), "shared memory: %s", strerror(errno));
		goto refuse;
	}
	p->group = sdr_engine_group_add_mem(b->e, req.dsp_rate, req.n_rx, rx_off, req.n_tx, tx_off, req.ring_ms, req.tx_target, p->mem, p->size);
	if (!p->group) {
		snprintf(ans.error, sizeof(ans.error), "%s", sdr_engine_error());
		goto refuse;
	}
	b->n_groups++;
	memcpy(p->name, req.name, sizeof(p->name));
	fprintf(stderr, "SDR backend: '%s' registered %d RX / %d TX channels at DSP rate %d\n", p->name, req.n_rx, req.n_tx, req.dsp_rate);
	ans.ok = 1;
	ans.view_size = p->size;
	ans.tx_scale = sdr_engine_tx_scale(b->e);
	ans.rx_centre = b->rx_centre;
	ans.tx_centre = b->tx_centre;
	answer(p->sock, &ans, p->memfd);
	return;

refuse:
	fprintf(stderr, "SDR backend: request refused: %s\n", ans.error);
	answer(p->sock, &ans, -1);
	/* the connection stays: the program may close it or ask again */
	if (p->mem)
		munmap(p->mem, p->size);
	if (p->memfd >= 0)
		close(p->memfd);
	p->mem = NULL;
	p->memfd = -1;
}

static void *control_thread(void *arg)
{
	sdr_backend_t *b = arg;
	struct pollfd pfd[MAX_PROGRAMS + 2];
	int idx[MAX_PROGRAMS + 2];

	for (;;) {
		int n = 0, i;

		pfd[n].fd = b->wake[0];
		pfd[n].events = POLLIN;
		idx[n++] = -1;
		pfd[n].fd = b->listen_sock;
		pfd[n].events = POLLIN;
		idx[n++] = -2;
		for (i = 0; i < MAX_PROGRAMS; i++) {
			if (b->prog[i].sock < 0)
				continue;
			pfd[n].fd = b->prog[i].sock;
			pfd[n].events = POLLIN;
			idx[n++] = i;
		}
		if (poll(pfd, n, -1) < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (pfd[0].revents)
			break;
		for (i = 2; i < n; i++)
			if (pfd[i].revents)
				program_request(b, &b->prog[idx[i]]);
		if (pfd[1].revents & POLLIN) {
			int s = accept4(b->listen_sock, NULL, NULL, SOCK_CLOEXEC);

			if (s >= 0) {
				for (i = 0; i < MAX_PROGRAMS; i++)
					if (b->prog[i].sock < 0)
						break;
				if (i == MAX_PROGRAMS) {
					fprintf(stderr, "SDR backend: %d programs, no more\n", MAX_PROGRAMS);
					close(s);
				} else
					b->prog[i].sock = s;
			}
		}
	}
	return NULL;
}

sdr_backend_t *sdr_backend_start(sdr_engine_t *e, const char *socket_name, double rx_centre, double tx_centre)
{
	sdr_backend_t *b;
	struct sockaddr_un a;
	socklen_t len;
	int i;

	error_text[0] = '\0';
	b = calloc(1, sizeof(*b));
	if (!b) {
		snprintf(error_text, sizeof(error_text), "no memory");
		return NULL;
	}
	b->e = e;
	b->rx_centre = rx_centre;
	b->tx_centre = tx_centre;
	b->listen_sock = -1;
	b->wake[0] = b->wake[1] = -1;
	for (i = 0; i < MAX_PROGRAMS; i++) {
		b->prog[i].sock = -1;
		b->prog[i].memfd = -1;
	}
	if (sdr_backend_addr(socket_name, &a, &len) < 0) {
		snprintf(error_text, sizeof(error_text), "socket name '%s' empty or too long", socket_name);
		goto fail;
	}
	b->listen_sock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	if (b->listen_sock < 0 || bind(b->listen_sock, (struct sockaddr *)&a, len) < 0 || listen(b->listen_sock, 8) < 0) {
		snprintf(error_text, sizeof(error_text), "socket '%s': %s", socket_name, strerror(errno));
		goto fail;
	}
	if (pipe2(b->wake, O_CLOEXEC) < 0) {
		snprintf(error_text, sizeof(error_text), "pipe: %s", strerror(errno));
		goto fail;
	}
	if (pthread_create(&b->thread, NULL, control_thread, b)) {
		snprintf(error_text, sizeof(error_text), "control thread not started");
		goto fail;
	}
	return b;

fail:
	if (b->listen_sock >= 0)
		close(b->listen_sock);
	if (b->wake[0] >= 0) {
		close(b->wake[0]);
		close(b->wake[1]);
	}
	free(b);
	return NULL;
}

void sdr_backend_stop(sdr_backend_t *b)
{
	int i;

	if (!b)
		return;
	if (write(b->wake[1], "x", 1) != 1)
		fprintf(stderr, "SDR backend: control thread not woken\n");
	pthread_join(b->thread, NULL);
	for (i = 0; i < MAX_PROGRAMS; i++)
		if (b->prog[i].sock >= 0)
			program_drop(b, &b->prog[i]);
	close(b->listen_sock);
	close(b->wake[0]);
	close(b->wake[1]);
	free(b);
}

int sdr_backend_groups(sdr_backend_t *b)
{
	return b->n_groups;
}
