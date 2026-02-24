/* RDS Protocol Server Implementation
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ============================================================
 * SUPPORTED CLIENTS
 * ============================================================
 * 
 * XDR-GTK Protocol (RDS_PROTO_XDR_GTK):
 *   - XDR-GTK desktop application (tested)
 *   - fm-dx-webserver (tested, supported)
 *     Uses librdsparser for RDS decoding. Supports PI, PS, RT, PTY, AF,
 *     signal level, stereo indicator, and spectral scan.
 *   - Any client implementing XDR-GTK protocol
 *
 * RDS-Spy Protocol (RDS_PROTO_RDSSPY):
 *   - RDS Spy desktop application
 *   - Compatible RDS logging software
 *
 * UECP Protocol (RDS_PROTO_UECP):
 *   - Professional RDS encoder control (TX mode)
 *   - EN 50067 / IEC 62106 compliant
 *
 * ASCII-G Protocol (RDS_PROTO_ASCII_G):
 *   - Simple text-based RDS encoder control (TX mode)
 *   - SETSPY command for RDS Spy output
 *
 * ============================================================
 * DESIGN
 * ============================================================
 * - TX: radio writes to ring buffer, poll drains to socket
 * - RX: socket fills ring buffer, poll processes ONE command per call
 * - Never blocks main thread
 * - If buffers full, data dropped (real-time, stale data useless)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <termios.h>

#include <math.h>
#include "../liblogging/logging.h"
#include "../libfft/fft.h"
#include "rds_protocol.h"
#include "rds.h"
#include "rds_tables.h"

/* ============================================================
 * Constants
 * ============================================================ */

#define TCP_BACKLOG			1
#define XDR_SIGNAL_INTERVAL_DEFAULT_MS	100
#define XDR_SIGNAL_INTERVAL_MIN_MS	10
#define XDR_SIGNAL_INTERVAL_MAX_MS	10000

/* Spectral scan FFT constants */
#define SCAN_FFT_SIZE		4096
#define SCAN_FFT_M		12	/* log2(4096) */
#define SCAN_FFT_FRAMES		8	/* average this many FFT frames per window */
#define SCAN_SETTLE_FRAMES	3	/* discard this many FFT frames after retune */

/* After login, suppress the first T (tune) command from XDR-GTK for this
 * many microseconds.  XDR-GTK's connection_dialog_callback fires within
 * ~100ms of tuner_ready and unconditionally sends T<conf.initial_freq>.
 * 500ms gives plenty of margin. */
#define XDR_LOGIN_TUNE_GRACE_US		500000

/* TCP keepalive: detect dead clients at OS level.
 * After TCP_KEEPALIVE_IDLE_S seconds of inactivity, send probes every
 * TCP_KEEPALIVE_INTVL_S seconds, up to TCP_KEEPALIVE_CNT times.
 * Total detection time: IDLE + CNT * INTVL seconds. */
#define TCP_KEEPALIVE_IDLE_S	60	/* Start probing after 60s idle */
#define TCP_KEEPALIVE_INTVL_S	10	/* Probe every 10s */
#define TCP_KEEPALIVE_CNT	3	/* 3 failed probes = dead (30s) */

/* Max bytes to read/write per poll (bound I/O time) */
#define IO_CHUNK_MAX			256

/* ============================================================
 * Helpers
 * ============================================================ */

static int64_t time_us(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

static int set_nonblock(int fd)
{
	int flags = fcntl(fd, F_GETFL, 0);
	if (flags < 0)
		return -1;
	return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

/* Format connection duration from microseconds into "Xh Xm Xs" */
static void fmt_duration(int64_t us, char *buf, int size)
{
	int64_t secs = us / 1000000LL;
	if (secs < 60)
		snprintf(buf, size, "%llds", (long long)secs);
	else if (secs < 3600)
		snprintf(buf, size, "%lldm %llds", (long long)(secs / 60), (long long)(secs % 60));
	else
		snprintf(buf, size, "%lldh %lldm", (long long)(secs / 3600), (long long)((secs % 3600) / 60));
}

/* ============================================================
 * SHA1 (RFC 3174) — self-contained, no external deps
 * Used for XDR-GTK TCP password authentication.
 * ============================================================ */

#define SHA1_ROTL(v, n) (((v) << (n)) | ((v) >> (32 - (n))))

static void sha1(const uint8_t *data, size_t len, uint8_t out[20])
{
	uint32_t h0 = 0x67452301u;
	uint32_t h1 = 0xEFCDAB89u;
	uint32_t h2 = 0x98BADCFEu;
	uint32_t h3 = 0x10325476u;
	uint32_t h4 = 0xC3D2E1F0u;

	/* Pre-processing: build padded message in chunks */
	uint64_t bit_len = (uint64_t)len * 8;
	size_t padded = ((len + 8) / 64 + 1) * 64;
	uint8_t *msg = calloc(padded, 1);
	if (!msg) return;
	memcpy(msg, data, len);
	msg[len] = 0x80;
	/* Append bit length big-endian at end */
	for (int i = 0; i < 8; i++)
		msg[padded - 8 + i] = (uint8_t)(bit_len >> (56 - i * 8));

	for (size_t off = 0; off < padded; off += 64) {
		uint32_t w[80];
		int i;
		for (i = 0; i < 16; i++) {
			const uint8_t *b = msg + off + i * 4;
			w[i] = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
			       ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
		}
		for (i = 16; i < 80; i++)
			w[i] = SHA1_ROTL(w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16], 1);

		uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
		for (i = 0; i < 80; i++) {
			uint32_t f, k;
			if (i < 20)      { f = (b & c) | (~b & d); k = 0x5A827999u; }
			else if (i < 40) { f = b ^ c ^ d;           k = 0x6ED9EBA1u; }
			else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
			else             { f = b ^ c ^ d;           k = 0xCA62C1D6u; }
			uint32_t tmp = SHA1_ROTL(a, 5) + f + e + k + w[i];
			e = d; d = c; c = SHA1_ROTL(b, 30); b = a; a = tmp;
		}
		h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
	}
	free(msg);

	uint32_t h[5] = { h0, h1, h2, h3, h4 };
	for (int i = 0; i < 5; i++) {
		out[i*4+0] = (h[i] >> 24) & 0xFF;
		out[i*4+1] = (h[i] >> 16) & 0xFF;
		out[i*4+2] = (h[i] >>  8) & 0xFF;
		out[i*4+3] =  h[i]        & 0xFF;
	}
}

/* Compute XDR-GTK auth hash: SHA1(salt[16] + password) -> 40-char hex string */
static void xdr_auth_hash(const uint8_t salt[XDR_AUTH_SALT_LEN],
			  const char *password, char out[XDR_AUTH_HASH_LEN + 1])
{
	size_t pwlen = password ? strlen(password) : 0;
	size_t total = XDR_AUTH_SALT_LEN + pwlen;
	uint8_t *buf = malloc(total);
	uint8_t digest[20];
	int i;

	if (!buf) { out[0] = '\0'; return; }
	memcpy(buf, salt, XDR_AUTH_SALT_LEN);
	if (pwlen)
		memcpy(buf + XDR_AUTH_SALT_LEN, password, pwlen);
	sha1(buf, total, digest);
	free(buf);

	for (i = 0; i < 20; i++)
		snprintf(out + i * 2, 3, "%02x", digest[i]);
	out[XDR_AUTH_HASH_LEN] = '\0';
}

/* ============================================================
 * Protocol Names
 * ============================================================ */

const char *rds_proto_name(rds_proto_t proto)
{
	switch (proto) {
	case RDS_PROTO_XDR_GTK:	return "XDR-GTK";
	case RDS_PROTO_RDSSPY:	return "RDS-Spy";
	case RDS_PROTO_UECP:	return "UECP";
	case RDS_PROTO_ASCII_G:	return "ASCII-G";
	default:		return "Unknown";
	}
}

/* ============================================================
 * XDR-GTK Command Decoders (for logging)
 * ============================================================ */

/* Command name lookup */
static const char *xdr_cmd_name(char cmd)
{
	switch (cmd) {
	case 'A': return "AGC";
	case 'B': return "stereo mode";
	case 'D': return "de-emphasis";
	case 'F': return "IF filter";
	case 'G': return "gain/iMS+cEQ";
	case 'I': return "signal interval";
	case 'M': return "FM/AM mode";
	case 'N': return "pilot test";
	case 'Q': return "squelch";
	case 'T': return "tune";
	case 'V': return "DAA voltage";
	case 'W': return "bandwidth";
	case 'Y': return "volume";
	case 'Z': return "antenna";
	case 'x': return "init";
	case 'X': return "shutdown";
	default:  return NULL;
	}
}

/* Decode AGC value */
static const char *xdr_agc_str(int val)
{
	switch (val) {
	case 0: return "off";
	case 1: return "low";
	case 2: return "mid";
	case 3: return "high";
	default: return "?";
	}
}

/* Decode stereo mode */
static const char *xdr_stereo_str(int val)
{
	switch (val) {
	case 0: return "stereo";
	case 1: return "mono";
	case 2: return "forced stereo";
	default: return "?";
	}
}

/* Decode de-emphasis */
static const char *xdr_deemph_str(int val)
{
	switch (val) {
	case 0: return "50µs";
	case 1: return "75µs";
	default: return "?";
	}
}

/* Decode FM/AM mode */
static const char *xdr_mode_str(int val)
{
	switch (val) {
	case 0: return "FM";
	case 1: return "AM";
	default: return "?";
	}
}

/* Format command value with human-readable description.
 * Returns static buffer - not thread safe, use immediately. */
static const char *xdr_cmd_desc(char cmd, int val, const char *arg)
{
	static char buf[64];
	const char *name = xdr_cmd_name(cmd);
	
	if (!name) {
		snprintf(buf, sizeof(buf), "%c%d", cmd, val);
		return buf;
	}
	
	switch (cmd) {
	case 'A':
		snprintf(buf, sizeof(buf), "%s=%s", name, xdr_agc_str(val));
		break;
	case 'B':
		snprintf(buf, sizeof(buf), "%s=%s", name, xdr_stereo_str(val));
		break;
	case 'D':
		snprintf(buf, sizeof(buf), "%s=%s", name, xdr_deemph_str(val));
		break;
	case 'F':
		if (val < 0)
			snprintf(buf, sizeof(buf), "%s=auto", name);
		else
			snprintf(buf, sizeof(buf), "%s=%d", name, val);
		break;
	case 'G':
		/* Detect TEF6686 format (2-digit: 00, 01, 10, 11) vs XDR-GTK format */
		if (arg && strlen(arg) == 2 && arg[0] >= '0' && arg[0] <= '1' && arg[1] >= '0' && arg[1] <= '1') {
			snprintf(buf, sizeof(buf), "iMS=%c cEQ=%c", arg[0], arg[1]);
		} else {
			snprintf(buf, sizeof(buf), "RF=%d IF=%d", val / 10, val % 10);
		}
		break;
	case 'I':
		snprintf(buf, sizeof(buf), "%s=%dms", name, val);
		break;
	case 'M':
		snprintf(buf, sizeof(buf), "%s=%s", name, xdr_mode_str(val));
		break;
	case 'Q':
		snprintf(buf, sizeof(buf), "%s=%d", name, val);
		break;
	case 'T':
		snprintf(buf, sizeof(buf), "%s=%.3fMHz", name, val / 1000.0);
		break;
	case 'W':
		snprintf(buf, sizeof(buf), "%s=%dkHz", name, val);
		break;
	case 'Y':
		snprintf(buf, sizeof(buf), "%s=%d%%", name, val);
		break;
	case 'Z':
		snprintf(buf, sizeof(buf), "%s=%d", name, val);
		break;
	default:
		snprintf(buf, sizeof(buf), "%s=%d", name, val);
		break;
	}
	return buf;
}

/* Format command as "CMD (decoded)" - uses two static buffers to allow
 * two calls in the same printf. Alternates between buffers. */
static const char *xdr_cmd_fmt(char cmd, int val, const char *arg)
{
	static char buf[2][80];
	static int idx = 0;
	char *b = buf[idx];
	idx = (idx + 1) % 2;
	
	/* Format protocol part */
	int len;
	if (cmd == 'G' && arg && strlen(arg) == 2 && arg[0] >= '0' && arg[0] <= '1' && arg[1] >= '0' && arg[1] <= '1') {
		len = snprintf(b, sizeof(buf[0]), "G%02d", val);
	} else if (cmd == 'G') {
		len = snprintf(b, sizeof(buf[0]), "G%d", val);
	} else {
		len = snprintf(b, sizeof(buf[0]), "%c%d", cmd, val);
	}
	
	/* Add decoded part */
	snprintf(b + len, sizeof(buf[0]) - len, " (%s)", xdr_cmd_desc(cmd, val, arg));
	return b;
}

/* ============================================================
 * Endpoint Parsing
 * ============================================================ */

int rds_ep_parse(const char *str, rds_endpoint_t *ep)
{
	char *copy, *p, *tok;
	
	memset(ep, 0, sizeof(*ep));
	ep->type = RDS_EP_NONE;
	
	if (!str || !*str)
		return 0;  /* Empty = disabled */
	
	copy = strdup(str);
	if (!copy)
		return -1;
	
	/* Serial: starts with / */
	if (str[0] == '/') {
		ep->type = RDS_EP_SERIAL;
		
		/* device,speed,8N1[,flow] */
		tok = strtok_r(copy, ",", &p);
		if (!tok) goto err;
		strncpy(ep->device, tok, RDS_EP_DEV_LEN - 1);
		
		tok = strtok_r(NULL, ",", &p);
		if (!tok) goto err;
		ep->speed = atoi(tok);
		if (ep->speed <= 0) goto err;
		
		tok = strtok_r(NULL, ",", &p);
		if (!tok || strlen(tok) < 3) goto err;
		ep->bits = tok[0] - '0';
		ep->parity = toupper((unsigned char)tok[1]);
		ep->stopbits = tok[2] - '0';
		
		if (ep->bits != 7 && ep->bits != 8) goto err;
		if (ep->parity != 'N' && ep->parity != 'E' && ep->parity != 'O') goto err;
		if (ep->stopbits != 1 && ep->stopbits != 2) goto err;
		
		/* Optional flow control */
		tok = strtok_r(NULL, ",", &p);
		if (tok) {
			if (strcasecmp(tok, "xonxoff") == 0)
				ep->flow = RDS_FLOW_XONXOFF;
			else if (strcasecmp(tok, "rtscts") == 0)
				ep->flow = RDS_FLOW_RTSCTS;
			else if (strcasecmp(tok, "none") != 0)
				goto err;
		}
	} else {
		/* TCP: ip:port */
		ep->type = RDS_EP_TCP;
		
		char *colon = strchr(copy, ':');
		if (!colon) goto err;
		
		*colon = '\0';
		strncpy(ep->ip, copy, RDS_EP_IP_LEN - 1);
		ep->port = atoi(colon + 1);
		
		if (ep->port <= 0 || ep->port > 65535) goto err;
	}
	
	free(copy);
	return 0;
	
err:
	free(copy);
	ep->type = RDS_EP_NONE;
	return -1;
}

/* ============================================================
 * TCP Socket Setup
 * ============================================================ */

static int tcp_listen(const char *ip, int port)
{
	struct sockaddr_in addr;
	int fd, opt = 1;
	
	fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
	if (fd < 0) {
		LOGP(DRADIO, LOGL_ERROR, "socket(): %s\n", strerror(errno));
		return -1;
	}
	
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) {
		LOGP(DRADIO, LOGL_ERROR, "Invalid IP: %s\n", ip);
		close(fd);
		return -1;
	}
	
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "bind(%s:%d): %s\n", ip, port, strerror(errno));
		close(fd);
		return -1;
	}
	
	if (listen(fd, TCP_BACKLOG) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "listen(): %s\n", strerror(errno));
		close(fd);
		return -1;
	}
	
	return fd;
}

static int tcp_accept(int listen_fd, struct sockaddr_in *out_addr)
{
	socklen_t len = sizeof(*out_addr);
	int fd, opt = 1;
	
	fd = accept(listen_fd, (struct sockaddr *)out_addr, &len);
	if (fd < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return RDS_FD_NONE;  /* No connection pending */
		LOGP(DRADIO, LOGL_ERROR, "accept(): %s\n", strerror(errno));
		return -1;
	}
	
	set_nonblock(fd);
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

	/* Enable TCP keepalive to detect dead clients */
	setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE,   &opt, sizeof(opt));
	opt = TCP_KEEPALIVE_IDLE_S;
	setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,   &opt, sizeof(opt));
	opt = TCP_KEEPALIVE_INTVL_S;
	setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL,  &opt, sizeof(opt));
	opt = TCP_KEEPALIVE_CNT;
	setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,    &opt, sizeof(opt));

	return fd;
}

/* ============================================================
 * Serial Port Setup
 * ============================================================ */

static speed_t baud_to_speed(int baud)
{
	switch (baud) {
	case 300:	return B300;
	case 1200:	return B1200;
	case 2400:	return B2400;
	case 4800:	return B4800;
	case 9600:	return B9600;
	case 19200:	return B19200;
	case 38400:	return B38400;
	case 57600:	return B57600;
	case 115200:	return B115200;
	case 230400:	return B230400;
	default:	return B0;
	}
}

static int serial_open(const rds_endpoint_t *ep)
{
	struct termios tio;
	speed_t speed;
	int fd;
	
	fd = open(ep->device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		LOGP(DRADIO, LOGL_ERROR, "open(%s): %s\n", ep->device, strerror(errno));
		return -1;
	}
	
	memset(&tio, 0, sizeof(tio));
	
	/* Control modes */
	tio.c_cflag = CREAD | CLOCAL;
	
	/* Data bits */
	if (ep->bits == 7)
		tio.c_cflag |= CS7;
	else
		tio.c_cflag |= CS8;
	
	/* Parity */
	if (ep->parity == 'E') {
		tio.c_cflag |= PARENB;
	} else if (ep->parity == 'O') {
		tio.c_cflag |= PARENB | PARODD;
	}
	
	/* Stop bits */
	if (ep->stopbits == 2)
		tio.c_cflag |= CSTOPB;
	
	/* Flow control */
	if (ep->flow == RDS_FLOW_RTSCTS)
		tio.c_cflag |= CRTSCTS;
	
	/* Input modes */
	if (ep->flow == RDS_FLOW_XONXOFF)
		tio.c_iflag = IXON | IXOFF;
	
	/* Non-canonical, no echo */
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;
	
	/* Baud rate */
	speed = baud_to_speed(ep->speed);
	if (speed == B0) {
		LOGP(DRADIO, LOGL_ERROR, "Unsupported baud rate: %d\n", ep->speed);
		close(fd);
		return -1;
	}
	cfsetispeed(&tio, speed);
	cfsetospeed(&tio, speed);
	
	if (tcsetattr(fd, TCSANOW, &tio) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "tcsetattr(%s): %s\n", ep->device, strerror(errno));
		close(fd);
		return -1;
	}
	
	tcflush(fd, TCIOFLUSH);
	
	LOGP(DRADIO, LOGL_NOTICE, "Serial port %s opened at %d baud\n",
	     ep->device, ep->speed);
	
	return fd;
}

/* ============================================================
 * Server Init/Cleanup
 * ============================================================ */

int rds_server_init(rds_server_t *srv, const char *endpoint,
		    rds_proto_t proto, struct rds_encoder *enc)
{
	memset(srv, 0, sizeof(*srv));
	srv->listen_fd = RDS_FD_NONE;
	srv->client_fd = RDS_FD_NONE;
	srv->proto = proto;
	srv->encoder = enc;
	srv->signal_interval_ms = XDR_SIGNAL_INTERVAL_DEFAULT_MS;
	srv->deemphasis = -1;  /* Unknown until set by caller */
	
	rds_ring_init(&srv->tx_ring);
	rds_ring_init(&srv->rx_ring);
	
	if (rds_ep_parse(endpoint, &srv->ep) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Invalid endpoint: %s\n", endpoint);
		return -1;
	}
	
	if (srv->ep.type == RDS_EP_NONE)
		return 0;  /* Disabled */
	
	if (srv->ep.type == RDS_EP_TCP) {
		srv->listen_fd = tcp_listen(srv->ep.ip, srv->ep.port);
		if (srv->listen_fd < 0)
			return -1;
		LOGP(DRADIO, LOGL_NOTICE, "%s server listening on %s:%d\n",
		     rds_proto_name(proto), srv->ep.ip, srv->ep.port);
	} else {
		srv->client_fd = serial_open(&srv->ep);
		if (srv->client_fd < 0)
			return -1;
	}
	
	return 0;
}

void rds_server_cleanup(rds_server_t *srv)
{
	if (srv->client_fd != RDS_FD_NONE) {
		close(srv->client_fd);
		srv->client_fd = RDS_FD_NONE;
	}
	if (srv->listen_fd != RDS_FD_NONE) {
		close(srv->listen_fd);
		srv->listen_fd = RDS_FD_NONE;
	}
}

int rds_server_connected(const rds_server_t *srv)
{
	return srv->client_fd != RDS_FD_NONE;
}

void rds_server_set_password(rds_server_t *srv, const char *password)
{
	if (password && *password)
		strncpy(srv->password, password, sizeof(srv->password) - 1);
	else
		srv->password[0] = '\0';
}

void rds_server_set_deemphasis(rds_server_t *srv, int val)
{
	srv->deemphasis = val;
}

void rds_server_set_callbacks(rds_server_t *srv,
			      void (*tune)(int freq_khz, void *arg),
			      void (*setting)(const char *name, int val, void *arg),
			      void *arg)
{
	srv->tune_cb = tune;
	srv->setting_cb = setting;
	srv->cb_arg = arg;
}

void rds_server_update_signal(rds_server_t *srv, double dbm, int stereo, double pilot_mag)
{
	srv->signal_dbm = dbm;
	srv->stereo = stereo;
	srv->pilot_mag = pilot_mag;
}

/* ============================================================
 * TX Ring Buffer Output Formatting
 * ============================================================ */

/* Write string to TX ring (drops if full) */
static int tx_write_str(rds_server_t *srv, const char *str)
{
	int len = strlen(str);
	int written = rds_ring_write(&srv->tx_ring, (const uint8_t *)str, len);
	if (written < len)
		srv->groups_dropped++;
	return written;
}

/* Write formatted string to TX ring */
static int tx_printf(rds_server_t *srv, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));

static int tx_printf(rds_server_t *srv, const char *fmt, ...)
{
	char buf[256];
	va_list ap;
	int len;
	
	va_start(ap, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	
	if (len < 0 || len >= (int)sizeof(buf))
		return 0;
	
	return rds_ring_write(&srv->tx_ring, (const uint8_t *)buf, len);
}

/* Write raw bytes to TX ring */
static int tx_write_raw(rds_server_t *srv, const uint8_t *data, int len)
{
	int written = rds_ring_write(&srv->tx_ring, data, len);
	if (written < len)
		srv->groups_dropped++;
	return written;
}

/* ============================================================
 * Protocol-Specific TX Formatting
 * ============================================================ */

void rds_server_send_group(rds_server_t *srv,
			   const uint16_t blocks[4],
			   const uint8_t errors[4])
{
	if (srv->client_fd == RDS_FD_NONE)
		return;
	/* Don't queue data until auth is complete */
	if (srv->proto == RDS_PROTO_XDR_GTK && srv->auth_state == XDR_AUTH_WAIT)
		return;
	
	switch (srv->proto) {
	case RDS_PROTO_XDR_GTK: {
		/* Format: R<AAAA><BBBB><CCCC><DDDD><EE>\n
		 * EE = error byte: A[7:6], B[5:4], C[3:2], D[1:0]
		 * 
		 * librdsparser (used by XDR-GTK and fm-dx-webserver) expects:
		 *   0 = RDSPARSER_BLOCK_ERROR_NONE (valid block)
		 *   1 = RDSPARSER_BLOCK_ERROR_SMALL (corrected)
		 *   2 = RDSPARSER_BLOCK_ERROR_LARGE
		 *   3 = RDSPARSER_BLOCK_ERROR_UNCORRECTABLE
		 * 
		 * fm-dx-webserver sets correction threshold to LARGE (2), so it
		 * accepts blocks with error levels 0, 1, or 2. This enables AF
		 * decoding and other features that require error-free Block C.
		 * 
		 * Our RDS_STATUS_* values need mapping:
		 *   0 = NONE (no data) -> 3 (uncorrectable)
		 *   1 = VALID -> 0 (no error)
		 *   2 = CORRECTED -> 1 (small error)
		 *   3 = ERROR -> 3 (uncorrectable) */
		uint8_t mapped[4];
		for (int i = 0; i < 4; i++) {
			switch (errors[i]) {
			case 1:  mapped[i] = 0; break;  /* VALID -> no error */
			case 2:  mapped[i] = 1; break;  /* CORRECTED -> small error */
			default: mapped[i] = 3; break;  /* NONE/ERROR -> uncorrectable */
			}
		}
		uint8_t err_byte = ((mapped[0] & 0x03) << 6) |
				   ((mapped[1] & 0x03) << 4) |
				   ((mapped[2] & 0x03) << 2) |
				   (mapped[3] & 0x03);
		tx_printf(srv, "R%04X%04X%04X%04X%02X\n",
			  blocks[0], blocks[1], blocks[2], blocks[3], err_byte);
		break;
	}
	case RDS_PROTO_RDSSPY: {
		/* Format: G:\r\n<AAAA><BBBB><CCCC><DDDD>\r\n\r\n
		 * Bad blocks shown as "----" */
		char blk[4][5];
		int i;
		for (i = 0; i < 4; i++) {
			if (errors[i] >= RDS_ERR_UNCORR)
				strcpy(blk[i], "----");
			else
				snprintf(blk[i], sizeof(blk[i]), "%04X", blocks[i]);
		}
		LOGP(DRADIO, LOGL_DEBUG, "RDS-Spy: TX group %s%s%s%s (err %d%d%d%d)\n",
		     blk[0], blk[1], blk[2], blk[3],
		     errors[0], errors[1], errors[2], errors[3]);
		tx_printf(srv, "G:\r\n%s%s%s%s\r\n\r\n",
			  blk[0], blk[1], blk[2], blk[3]);
		break;
	}
	case RDS_PROTO_UECP:
		/* UECP is encoder control, not decoder output */
		break;
	case RDS_PROTO_ASCII_G:
		/* ASCII-G is encoder control, but SETSPY enables RDS Spy output */
		if (srv->setspy_remaining > 0) {
			char blk[4][5];
			int i;
			for (i = 0; i < 4; i++) {
				if (errors[i] >= RDS_ERR_UNCORR)
					strcpy(blk[i], "----");
				else
					snprintf(blk[i], sizeof(blk[i]), "%04X", blocks[i]);
			}
			LOGP(DRADIO, LOGL_DEBUG, "ASCII-G SETSPY: TX group %s%s%s%s (err %d%d%d%d, remaining %d)\n",
			     blk[0], blk[1], blk[2], blk[3],
			     errors[0], errors[1], errors[2], errors[3],
			     srv->setspy_remaining);
			tx_printf(srv, "G:\r\n%s%s%s%s\r\n\r\n",
				  blk[0], blk[1], blk[2], blk[3]);
			srv->setspy_remaining--;
		}
		break;
	}
	
	srv->groups_sent++;
}

void rds_server_send_pi(rds_server_t *srv, uint16_t pi, int err)
{
	if (srv->client_fd == RDS_FD_NONE || srv->proto != RDS_PROTO_XDR_GTK)
		return;
	
	/* Format: P<XXXX>[?...]\n - one '?' per error level */
	char errs[4] = "";
	if (err > 0 && err <= 3)
		memset(errs, '?', err);
	tx_printf(srv, "P%04X%s\n", pi, errs);
}

void rds_server_send_signal(rds_server_t *srv, double dbm,
			    int stereo, int forced_mono)
{
	if (srv->client_fd == RDS_FD_NONE || srv->proto != RDS_PROTO_XDR_GTK)
		return;
	
	/* Format: S<s/m><signal>\n
	 * s/S = stereo (s=auto, S=forced mono)
	 * m/M = mono (m=auto, M=forced mono) */
	char mode;
	if (stereo)
		mode = forced_mono ? 'S' : 's';
	else
		mode = forced_mono ? 'M' : 'm';
	
	tx_printf(srv, "S%c%.1f\n", mode, dbm);
}

void rds_server_send_freq(rds_server_t *srv, int freq_khz)
{
	if (srv->client_fd == RDS_FD_NONE || srv->proto != RDS_PROTO_XDR_GTK)
		return;
	
	tx_printf(srv, "T%d\n", freq_khz);
	srv->freq_khz = freq_khz;
}

void rds_server_send_reset(rds_server_t *srv)
{
	if (srv->client_fd == RDS_FD_NONE || srv->proto != RDS_PROTO_RDSSPY)
		return;
	
	tx_write_str(srv, "G:\r\nRESET\r\n\r\n");
}

/* ============================================================
 * XDR-GTK Command Processing
 * ============================================================ */

/* Process single XDR-GTK command line */
static void xdr_process_cmd(rds_server_t *srv, const char *cmd, int len)
{
	/* Empty line (len == 0) is used by XDR-GTK to stop an active scan.
	 * See xdr-gtk/src/scan.c line 477: tuner_write(tuner.thread, "") */
	if (len == 0) {
		if (srv->scan_active) {
			LOGP(DRADIO, LOGL_NOTICE, "XDR-GTK: Scan stop requested (empty line)\n");
			srv->scan_active = 0;
			/* Restore original frequency */
			if (srv->scan_cb && srv->scan_orig_freq_khz > 0) {
				srv->scan_cb(srv->scan_orig_freq_khz, srv->scan_cb_arg);
				tx_printf(srv, "T%d\n", srv->scan_orig_freq_khz);
				srv->freq_khz = srv->scan_orig_freq_khz;
			}
		}
		return;
	}
	
	char c = cmd[0];
	const char *arg = (len > 1) ? cmd + 1 : "";
	
	switch (c) {
	case 'x':
		/* Status/init request - respond with OK and current frequency */
		LOGP(DRADIO, LOGL_INFO, "XDR-GTK: Received 'x' (init), reply: OK + T%d%s\n",
		     srv->freq_khz, srv->deemphasis >= 0 ? " + D" : "");
		tx_write_str(srv, "OK\n");
		tx_printf(srv, "T%d\n", srv->freq_khz);
		if (srv->deemphasis >= 0)
			tx_printf(srv, "D%d\n", srv->deemphasis);
		/* Start post-login sync window: clients send their saved settings
		 * (T, A, Y, D, etc.) immediately after connection. During this window
		 * we ignore those and echo back our current values so the client
		 * syncs to our state instead of overriding it. */
		srv->login_tune_grace_us = time_us() + XDR_LOGIN_TUNE_GRACE_US;
		break;

	case 'X':
		/* Client shutdown notification - close connection */
		LOGP(DRADIO, LOGL_NOTICE, "XDR-GTK: Client %s:%d sent shutdown, disconnecting\n",
		     srv->client_ip, srv->client_port);
		close(srv->client_fd);
		srv->client_fd = RDS_FD_NONE;
		srv->rx_bytes = 0;
		srv->tx_bytes = 0;
		break;
		
	case 'T':
		/* Tune: T<freq_khz> */
		if (*arg) {
			int freq = atoi(arg);
			/* During post-login sync window: client sends its saved settings,
			 * but we want it to adopt our current state instead. */
			if (srv->login_tune_grace_us && time_us() < srv->login_tune_grace_us) {
				LOGP(DRADIO, LOGL_INFO, "XDR-GTK: Ignoring T%d (tune=%.3fMHz), reply: T%d (tune=%.3fMHz)\n",
				     freq, freq / 1000.0, srv->freq_khz, srv->freq_khz / 1000.0);
				/* Echo back our current frequency so client UI syncs to us */
				tx_printf(srv, "T%d\n", srv->freq_khz);
				break;
			}
			srv->login_tune_grace_us = 0;
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: T%d (tune=%.3fMHz)\n", freq, freq / 1000.0);
			if (srv->tune_cb)
				srv->tune_cb(freq, srv->cb_arg);
			tx_printf(srv, "T%d\n", freq);
			srv->freq_khz = freq;
		}
		break;
		
	case 'B':  /* Stereo/Mono mode: B0=auto, B1=forced mono */
		if (*arg) {
			int val = atoi(arg);
			srv->forced_mono = (val != 0);
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: %s\n", xdr_cmd_fmt(c, val, arg));
			tx_printf(srv, "B%d\n", val);
			
			/* Notify callback */
			if (srv->setting_cb) {
				srv->setting_cb("B", val, srv->cb_arg);
			}
		}
		break;
		
	case 'A':  /* AGC */
	case 'D':  /* De-emphasis */
	case 'F':  /* Bandwidth filter */
	case 'G':  /* RF/IF gain (XDR-GTK) or iMS/cEQ (TEF6686/fm-dx-webserver) */
	case 'Y':  /* Volume */
	case 'Z':  /* Antenna */
	case 'Q':  /* Squelch */
		if (*arg) {
			int val = atoi(arg);
			
			/* During post-login sync window: client sends its saved settings,
			 * but we want it to adopt our current state instead.
			 * Echo back acknowledgment without acting on the command. */
			if (srv->login_tune_grace_us && time_us() < srv->login_tune_grace_us) {
				if (c == 'D' && srv->deemphasis >= 0) {
					/* For de-emphasis, send our actual value */
					LOGP(DRADIO, LOGL_INFO, "XDR-GTK: Ignoring %s, reply: %s\n",
					     xdr_cmd_fmt(c, val, arg), xdr_cmd_fmt(c, srv->deemphasis, NULL));
					tx_printf(srv, "D%d\n", srv->deemphasis);
				} else {
					/* Echo back same value to acknowledge */
					LOGP(DRADIO, LOGL_INFO, "XDR-GTK: Ignoring %s, reply: %s\n",
					     xdr_cmd_fmt(c, val, arg), xdr_cmd_fmt(c, val, arg));
					if (c == 'G' && strlen(arg) == 2 && arg[0] >= '0' && arg[0] <= '1' && arg[1] >= '0' && arg[1] <= '1')
						tx_printf(srv, "G%02d\n", val);
					else if (c == 'G')
						tx_printf(srv, "G%d\n", val);
					else
						tx_printf(srv, "%c%d\n", c, val);
				}
				break;
			}
			
			/* Normal command processing */
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: %s [not wired]\n", xdr_cmd_fmt(c, val, arg));
			
			/* Echo back in appropriate format */
			if (c == 'G') {
				if (strlen(arg) == 2 && arg[0] >= '0' && arg[0] <= '1' && arg[1] >= '0' && arg[1] <= '1')
					tx_printf(srv, "G%02d\n", val);
				else
					tx_printf(srv, "G%d\n", val);
			} else {
				tx_printf(srv, "%c%d\n", c, val);
			}
			
			/* Notify callback */
			if (srv->setting_cb) {
				char name[2] = { c, '\0' };
				srv->setting_cb(name, val, srv->cb_arg);
			}
		}
		break;
		
	case 'I':
		/* Signal sampling interval: I<ms>[,<mode>] */
		if (*arg) {
			int ms = atoi(arg);
			if (ms >= XDR_SIGNAL_INTERVAL_MIN_MS &&
			    ms <= XDR_SIGNAL_INTERVAL_MAX_MS) {
				srv->signal_interval_ms = ms;
			}
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: %s, reply: %s\n",
			     xdr_cmd_fmt('I', ms, arg), xdr_cmd_fmt('I', srv->signal_interval_ms, NULL));
			tx_printf(srv, "I%d\n", srv->signal_interval_ms);
		}
		break;

	case 'M':
		/* FM/AM mode: M<mode> (0=FM, 1=AM, ...) — echo back */
		if (*arg) {
			int mode = atoi(arg);
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: %s\n", xdr_cmd_fmt('M', mode, arg));
			tx_printf(srv, "M%d\n", mode);
		}
		break;

	case 'W':
		/* Bandwidth kHz (TEF6686): W<bw> — echo back */
		if (*arg) {
			int bw = atoi(arg);
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: %s\n", xdr_cmd_fmt('W', bw, arg));
			tx_printf(srv, "W%d\n", bw);
		}
		break;

	case 'V':
		/* DAA tuning voltage alignment: V<val> — echo back */
		if (*arg) {
			int val = atoi(arg);
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: V%d (DAA voltage)\n", val);
			tx_printf(srv, "V%d\n", val);
		}
		break;

	case 'C':
		/* Rotator control: C<state> — echo back */
		if (*arg) {
			int state = atoi(arg);
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: C%d (rotator)\n", state);
			tx_printf(srv, "C%d\n", state);
		}
		break;

	case 'N':
		/* Stereo pilot test request — reply N<level> in 0.1 kHz units */
		{
			/* pilot_mag is true injection level (0.1 = 10% = 7.5 kHz).
			 * Convert to 0.1 kHz units: mag * 75 kHz * 10 */
			int pilot_level = (int)(srv->pilot_mag * 75.0 * 10.0 + 0.5);
			LOGP(DRADIO, LOGL_INFO, "XDR-GTK: N (pilot test), reply: N%d (%.1f kHz)\n",
			     pilot_level, pilot_level / 10.0);
			tx_printf(srv, "N%d\n", pilot_level);
		}
		break;

	case 'S':
		/* Spectral scan commands (XDR-GTK scan dialog):
		 *   Sa<freq>   - scan start frequency (kHz)
		 *   Sb<freq>   - scan stop frequency (kHz)
		 *   Sc<step>   - scan step (kHz)
		 *   Sf<filter> - filter index (XDR mode)
		 *   Sw<bw>     - bandwidth (kHz)
		 *   Sz<ant>    - antenna index
		 *   S          - start single-pass scan
		 *   Sm         - start continuous scan */
		if (!*arg) {
			/* Bare 'S': start single-pass scan */
			if (srv->scan_cb && srv->scan_step_khz > 0 &&
			    srv->scan_start_khz < srv->scan_stop_khz) {
				int npts = (srv->scan_stop_khz - srv->scan_start_khz) / srv->scan_step_khz + 1;
				if (npts > SCAN_MAX_POINTS) npts = SCAN_MAX_POINTS;
				LOGP(DRADIO, LOGL_NOTICE,
				     "XDR-GTK: Scan start %d-%d kHz step %d kHz (%d points, FFT-based)\n",
				     srv->scan_start_khz, srv->scan_stop_khz, srv->scan_step_khz, npts);
				srv->scan_active = 1;
				srv->scan_continuous = 0;
				srv->scan_orig_freq_khz = srv->freq_khz;
				srv->scan_points_total = npts;
				srv->scan_points_done = 0;
				srv->scan_result_len = 0;
				srv->scan_result[0] = '\0';
				srv->scan_fft_fill = 0;
				srv->scan_settle_frames = SCAN_SETTLE_FRAMES;
				memset(srv->scan_measured, 0, npts * sizeof(srv->scan_measured[0]));
				memset(srv->scan_power_acc, 0, npts * sizeof(srv->scan_power_acc[0]));
				memset(srv->scan_power_cnt, 0, npts * sizeof(srv->scan_power_cnt[0]));
				/* Retune immediately to the start of the scan range so the
				 * first window is measured with a proper settle delay, not
				 * whatever frequency the radio happens to be on. */
				if (srv->scan_cb) {
					int first_center = srv->scan_start_khz + (srv->scan_step_khz * 2);
					srv->scan_cb(first_center, srv->scan_cb_arg);
				}
			} else {
				LOGP(DRADIO, LOGL_INFO, "XDR-GTK: Scan trigger but no scan support, sending empty U\n");
				tx_write_str(srv, "U\n");
			}
		} else if (*arg == 'm') {
			/* 'Sm': start continuous scan */
			if (srv->scan_cb && srv->scan_step_khz > 0 &&
			    srv->scan_start_khz < srv->scan_stop_khz) {
				int npts = (srv->scan_stop_khz - srv->scan_start_khz) / srv->scan_step_khz + 1;
				if (npts > SCAN_MAX_POINTS) npts = SCAN_MAX_POINTS;
				LOGP(DRADIO, LOGL_NOTICE,
				     "XDR-GTK: Continuous scan start %d-%d kHz step %d kHz (%d points, FFT-based)\n",
				     srv->scan_start_khz, srv->scan_stop_khz, srv->scan_step_khz, npts);
				srv->scan_active = 1;
				srv->scan_continuous = 1;
				srv->scan_orig_freq_khz = srv->freq_khz;
				srv->scan_points_total = npts;
				srv->scan_points_done = 0;
				srv->scan_result_len = 0;
				srv->scan_result[0] = '\0';
				srv->scan_fft_fill = 0;
				srv->scan_settle_frames = SCAN_SETTLE_FRAMES;
				memset(srv->scan_measured, 0, npts * sizeof(srv->scan_measured[0]));
				memset(srv->scan_power_acc, 0, npts * sizeof(srv->scan_power_acc[0]));
				memset(srv->scan_power_cnt, 0, npts * sizeof(srv->scan_power_cnt[0]));
				/* Retune immediately to the start of the scan range */
				if (srv->scan_cb) {
					int first_center = srv->scan_start_khz + (srv->scan_step_khz * 2);
					srv->scan_cb(first_center, srv->scan_cb_arg);
				}
			} else {
				tx_write_str(srv, "U\n");
			}
		} else {
			/* Scan parameter subcommands */
			int val = atoi(arg + 1);
			switch (*arg) {
			case 'a':
				srv->scan_start_khz = val;
				LOGP(DRADIO, LOGL_DEBUG, "XDR-GTK: Scan start=%d kHz\n", val);
				break;
			case 'b':
				srv->scan_stop_khz = val;
				LOGP(DRADIO, LOGL_DEBUG, "XDR-GTK: Scan stop=%d kHz\n", val);
				break;
			case 'c':
				srv->scan_step_khz = val;
				LOGP(DRADIO, LOGL_DEBUG, "XDR-GTK: Scan step=%d kHz\n", val);
				break;
			case 'f':
				srv->scan_filter = val;
				LOGP(DRADIO, LOGL_DEBUG, "XDR-GTK: Scan filter=%d\n", val);
				break;
			case 'w':
				srv->scan_bw_khz = val;
				LOGP(DRADIO, LOGL_DEBUG, "XDR-GTK: Scan bw=%d kHz\n", val);
				break;
			case 'z':
				srv->scan_antenna = val;
				LOGP(DRADIO, LOGL_DEBUG, "XDR-GTK: Scan antenna=%d\n", val);
				break;
			default:
				LOGP(DRADIO, LOGL_DEBUG, "XDR-GTK: Unknown scan param 'S%s'\n", arg);
				break;
			}
		}
		break;

	default:
		LOGP(DRADIO, LOGL_ERROR, "XDR-GTK: Unknown command '%c' (0x%02X): \"%.*s\"\n",
		     isprint((unsigned char)c) ? c : '?', (unsigned char)c, len, cmd);
		break;
	}
}

/* ============================================================
 * UECP Frame Processing
 * ============================================================ */

/* UECP MEC name lookup table for human-readable logging */
static const struct {
	uint8_t		mec;
	const char	*name;
} uecp_mec_names[] = {
	/* RDS data (IEC 62106-10 Table A.1) */
	{ UECP_MEC_PI,		"PI" },
	{ UECP_MEC_PS,		"PS" },
	{ UECP_MEC_TP_TA,	"TP/TA" },
	{ UECP_MEC_DI_PTYI,	"DI/PTYI" },
	{ UECP_MEC_MS,		"M/S" },
	{ UECP_MEC_PTY,	"PTY" },
	{ UECP_MEC_RTC_CORR,	"RTC Correction" },
	{ UECP_MEC_RT,		"RT" },
	{ UECP_MEC_PSN_ENABLE,	"PSN Enable" },
	{ UECP_MEC_RTC,	"RTC" },
	{ UECP_MEC_RDS_LEVEL,	"RDS Level" },
	{ UECP_MEC_AF,		"AF" },
	{ UECP_MEC_EON_AF,	"EON-AF" },
	{ UECP_MEC_EON_TA_CTRL,"EON-TA Ctrl" },
	{ UECP_MEC_GROUP_SEQ,	"Group Seq" },
	{ UECP_MEC_REQUEST,	"Request" },
	{ UECP_MEC_ACK,	"ACK" },
	{ UECP_MEC_CT_ONOFF,	"CT On/Off" },
	{ UECP_MEC_ECC,	"ECC/SLC" },
	{ UECP_MEC_DATA_SET_SEL,"Data Set Sel" },
	{ UECP_MEC_REF_INPUT,	"Ref Input" },
	{ UECP_MEC_RDS_ONOFF,	"RDS On/Off" },
	{ UECP_MEC_LPS,	"LPS" },
	{ UECP_MEC_RDS_PHASE,	"RDS Phase" },
	{ UECP_MEC_SITE_ADDR,	"Site Addr" },
	{ UECP_MEC_FREE_FORMAT,	"Free Format" },
	{ UECP_MEC_TDC,	"TDC" },
	{ UECP_MEC_ENC_ADDR,	"Enc Addr" },
	{ UECP_MEC_MAKE_PSN_LIST,"Make PSN List" },
	{ UECP_MEC_GROUP_VARIANT,"Group Variant" },
	{ UECP_MEC_TA_CTRL,	"TA Ctrl" },
	{ UECP_MEC_EWS,	"EWS" },
	{ UECP_MEC_COMM_MODE,	"Comm Mode" },
	{ UECP_MEC_MANUFACTURER,"Manufacturer" },
	{ UECP_MEC_LINKAGE,	"Linkage" },
	{ UECP_MEC_TMC,	"TMC" },
	{ UECP_MEC_EXT_GROUP_SEQ,"Ext Group Seq" },
	{ UECP_MEC_ACCESS_RIGHT,"Access Right" },
	{ UECP_MEC_PORT_MODE,	"Port Mode" },
	{ UECP_MEC_PORT_SPEED,	"Port Speed" },
	{ UECP_MEC_PORT_TIMEOUT,"Port Timeout" },
	{ UECP_MEC_PTYN,	"PTYN" },
	{ UECP_MEC_EON_ENABLE,	"EON Enable" },
	/* ODA commands */
	{ UECP_MEC_ODA_CONFIG,	"ODA Config" },
	{ UECP_MEC_ODA_IDENT,	"ODA Ident" },
	{ UECP_MEC_ODA_FREE,	"ODA Free" },
	{ UECP_MEC_ODA_PRIORITY,"ODA Priority" },
	{ UECP_MEC_ODA_BURST,	"ODA Burst" },
	{ UECP_MEC_ODA_SPIN,	"ODA Spin" },
	{ UECP_MEC_ODA_DATA,	"ODA Data" },
	{ UECP_MEC_ODA_ACCESS,	"ODA Access" },
	{ UECP_MEC_DAB_DL_CMD,	"DAB DL Cmd" },
	/* RDS2 / type C commands */
	{ UECP_MEC_ODA_AID_C1,	"ODA-AID C1" },
	{ UECP_MEC_ODA_AID_C2,	"ODA-AID C2" },
	{ UECP_MEC_PRIORITY_C,	"Priority C" },
	{ UECP_MEC_BURST_C,	"Burst C" },
	{ UECP_MEC_RFT_ALT2,	"RFT Alt2" },
	{ UECP_MEC_ODA_DATA_C,	"ODA Data C" },
	{ UECP_MEC_RFT_ALT1,	"RFT Alt1" },
	{ UECP_MEC_RFT_VARIANTS,"RFT Variants" },
	{ UECP_MEC_FILE_SEQ,	"File Seq" },
	{ UECP_MEC_GROUP_SEQ_C,	"Group Seq C" },
	{ UECP_MEC_EXT_GS_C,	"Ext GS C" },
	{ UECP_MEC_DAB_DL_MSG,	"DAB DL Msg" },
	{ UECP_MEC_UPPER_LEVEL,	"Upper Level" },
	{ UECP_MEC_UPPER_ONOFF,	"Upper On/Off" },
	{ 0, NULL }
};

const char *uecp_mec_name(uint8_t mec)
{
	int i;
	for (i = 0; uecp_mec_names[i].name; i++) {
		if (uecp_mec_names[i].mec == mec)
			return uecp_mec_names[i].name;
	}
	return "Unknown";
}

/* SLC variant names (IEC 62106-2 Table 9) — shared by request and group variant handlers */
static const char *slc_variant_names[RDS_1A_VARIANT_MAX] = {
	[RDS_1A_VARIANT_ECC]      = "ECC",
	[RDS_1A_VARIANT_TMC_ID]   = "TMC",
	[RDS_1A_VARIANT_PAGER]    = "Paging",
	[RDS_1A_VARIANT_LANGUAGE] = "Language",
	[4]                       = "Rsvd4",
	[5]                       = "Rsvd5",
	[RDS_1A_VARIANT_BCAST]    = "Broadcaster",
	[RDS_1A_VARIANT_EWS]      = "EWS",
};

/* CRC-16 (ITU-T polynomial x^16 + x^12 + x^5 + 1) */
static uint16_t uecp_crc16(const uint8_t *data, int len)
{
	uint16_t crc = 0xFFFF;
	int i, j;
	
	for (i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (j = 0; j < 8; j++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}
	return crc ^ 0xFFFF;
}

/* Send UECP acknowledgement */
static void uecp_send_ack(rds_server_t *srv, uint8_t code, uint8_t seq)
{
	uint8_t frame[16];
	uint8_t msg[3];
	uint16_t crc;
	int len = 0;
	const char *code_str;
	
	/* Build message: MEC_ACK, code, [seq if error] */
	msg[0] = UECP_MEC_ACK;
	msg[1] = code;
	int msg_len = (code == UECP_ACK_OK) ? 2 : 3;
	if (msg_len == 3)
		msg[2] = seq;
	
	/* Build frame */
	frame[len++] = UECP_STA;
	
	/* Address: global (all sites/encoders) */
	frame[len++] = UECP_ADDR_GLOBAL;
	frame[len++] = UECP_ADDR_GLOBAL;
	
	/* Sequence counter */
	frame[len++] = srv->uecp_seq++;
	if (srv->uecp_seq == 0)
		srv->uecp_seq = 1;
	
	/* Message length */
	frame[len++] = msg_len;
	
	/* Message */
	memcpy(frame + len, msg, msg_len);
	len += msg_len;
	
	/* CRC (over ADD to MSG) */
	crc = uecp_crc16(frame + 1, len - 1);
	frame[len++] = (crc >> 8) & 0xFF;
	frame[len++] = crc & 0xFF;
	
	frame[len++] = UECP_STP;
	
	/* Log the ACK reply */
	switch (code) {
	case UECP_ACK_OK:       code_str = "OK"; break;
	case UECP_ACK_CRC_ERR:  code_str = "CRC error"; break;
	case UECP_ACK_UNKNOWN:  code_str = "unknown command"; break;
	case UECP_ACK_PARAM_ERR: code_str = "parameter error"; break;
	default:                code_str = "unknown"; break;
	}
	LOGP(DRADIO, LOGL_INFO, "UECP: Sending ACK reply: code=%s (0x%02X), seq=%d\n",
	     code_str, code, seq);
	
	/* Note: Should byte-stuff, but ACK frames rarely need it */
	tx_write_raw(srv, frame, len);
}

/* Send a UECP frame with arbitrary message content.
 * msg/msg_len: complete message field (MEC + data).
 * Handles STA/STP delimiters, addressing, sequence counter, and CRC. */
static void uecp_send_frame(rds_server_t *srv, const uint8_t *msg, int msg_len)
{
	uint8_t frame[UECP_FRAME_MAX + 4]; /* room for STA + max frame + CRC + STP */
	uint16_t crc;
	int len = 0;

	if (msg_len > 255)
		return;

	frame[len++] = UECP_STA;

	/* Address: global (all sites/encoders) */
	frame[len++] = UECP_ADDR_GLOBAL;
	frame[len++] = UECP_ADDR_GLOBAL;

	/* Sequence counter */
	frame[len++] = srv->uecp_seq++;
	if (srv->uecp_seq == 0)
		srv->uecp_seq = 1;

	/* Message length */
	frame[len++] = (uint8_t)msg_len;

	/* Message */
	memcpy(frame + len, msg, msg_len);
	len += msg_len;

	/* CRC (over ADD to MSG) */
	crc = uecp_crc16(frame + 1, len - 1);
	frame[len++] = (crc >> 8) & 0xFF;
	frame[len++] = crc & 0xFF;

	frame[len++] = UECP_STP;

	tx_write_raw(srv, frame, len);
}

/* Build and send a UECP response for a single MEC query.
 * requested_mec: the MEC code being queried
 * resp_data/resp_len: the MEC-specific response payload (DSN+PSN+value) */
static void uecp_send_mec_response(rds_server_t *srv, uint8_t requested_mec,
				   const uint8_t *resp_data, int resp_len)
{
	uint8_t msg[260];
	int mlen = 0;

	msg[mlen++] = requested_mec;
	if (resp_len > 0 && resp_len < 256) {
		memcpy(msg + mlen, resp_data, resp_len);
		mlen += resp_len;
	}

	LOGP(DRADIO, LOGL_DEBUG, "UECP: Sending %s (0x%02X) response, %d bytes\n",
	     uecp_mec_name(requested_mec), requested_mec, resp_len);

	uecp_send_frame(srv, msg, mlen);
}

/* Build AF code list from encoder state (Method A or B).
 * Returns number of AF codes written to buf, 0 if no AFs.
 * UECP AF memory is a flat array of code pairs (IEC 62106-10 A.2.9):
 * "No distinction is made between Methods A or B. The AF list(s)
 *  have to be structured in pairs as specified in IEC 62106-2."
 * Method A: count_code + AF codes (pre-computed in af_codes[])
 * Method B: per list: [count_code, tuning] + [F1,F2] pairs
 *   count_code = 0xE0 + (1 + 2*af_count)
 *   F1<F2 = same programme, F1>F2 = regional variant
 * Terminated with 0x00. */
static int uecp_build_af_codes(const rds_encoder_t *enc, uint8_t *buf, int max,
			       const char **method_out, int *n_freqs)
{
	int n = 0;

	*n_freqs = 0;

	if (enc->use_method_b && enc->af_method_b.list_count > 0) {
		int i, j;
		*method_out = "B";
		for (i = 0; i < enc->af_method_b.list_count && n + 2 < max; i++) {
			const rds_af_method_b_list_t *list = &enc->af_method_b.lists[i];
			uint8_t tuning_code = (uint8_t)(list->tuning_freq - RDS_AF_FM_BASE);
			uint8_t count = 1 + 2 * list->af_count;
			/* Header pair: [count_code, tuning_freq] */
			buf[n++] = (uint8_t)(RDS_AF_NO_AF + count);
			buf[n++] = tuning_code;
			/* AF pairs: [F1, F2] with ascending/descending order */
			for (j = 0; j < list->af_count && n + 1 < max; j++) {
				uint8_t af_code = (uint8_t)(list->af_freq[j] - RDS_AF_FM_BASE);
				if (list->af_is_regional[j]) {
					/* Regional: F1 > F2 (descending) */
					if (tuning_code > af_code) {
						buf[n++] = tuning_code;
						buf[n++] = af_code;
					} else {
						buf[n++] = af_code;
						buf[n++] = tuning_code;
					}
				} else {
					/* Same programme: F1 < F2 (ascending) */
					if (tuning_code < af_code) {
						buf[n++] = tuning_code;
						buf[n++] = af_code;
					} else {
						buf[n++] = af_code;
						buf[n++] = tuning_code;
					}
				}
				(*n_freqs)++;
			}
		}
		/* Terminator */
		if (n < max)
			buf[n++] = 0x00;
	} else if (enc->af_code_count > 0) {
		*method_out = "A";
		n = (enc->af_code_count <= max - 1) ? enc->af_code_count : max - 1;
		memcpy(buf, enc->af_codes, n);
		*n_freqs = enc->af_method_a.vhf_count + enc->af_method_a.lf_mf_count;
		/* Terminator */
		buf[n++] = 0x00;
	} else {
		*method_out = "none";
	}

	return n;
}

/* Handle UECP Request MEC (0x17): query encoder state and send response.
 * data points to the bytes after the MEC byte, len is their count.
 * Format: MEL(1) + requested_MEC(1) + [DSN(1) + PSN(1) + ...] */
static void uecp_handle_request(rds_server_t *srv, const uint8_t *data, int len)
{
	rds_encoder_t *enc = srv->encoder;
	uint8_t resp[72];
	int rlen;
	uint8_t mel, req_mec, req_psn;
	const char *req_name;
	const rds_eon_entry_t *eon;

	if (!enc) {
		LOGP(DRADIO, LOGL_DEBUG, "UECP: Request ignored, no encoder\n");
		return;
	}

	if (len < 1) return;

	mel = data[0];
	if (mel < 1 || len < 1 + mel) {
		LOGP(DRADIO, LOGL_DEBUG, "UECP: Request truncated (mel=%d len=%d)\n", mel, len);
		return;
	}

	req_mec = data[1];
	req_name = uecp_mec_name(req_mec);

	/* Extract PSN from request: data[2]=DSN, data[3]=PSN (when mel >= 3)
	 * Exceptions — MECs with non-standard request format:
	 * MEC 0x1A: no PSN, data[3] = variant code (IEC 62106-10 A.2.11)
	 * MEC 0x3A: no DSN/PSN, data[2] = target MEC, data[3] = port (A.6.14) */
	if (req_mec == UECP_MEC_ECC || req_mec == UECP_MEC_ACCESS_RIGHT)
		req_psn = UECP_PSN_MAIN;
	else
		req_psn = (mel >= 3) ? data[3] : UECP_PSN_MAIN;

	/* PSN >= 2 = EON entry (index = PSN - 2) */
	eon = NULL;
	if (req_psn >= 2)
		eon = rds_enc_eon_get_by_index(enc, req_psn - 2);

	if (req_mec == UECP_MEC_ECC) {
		uint8_t rv = (mel >= 3) ? (data[3] & 0x07) : 0;
		LOGP(DRADIO, LOGL_INFO, "UECP: Request for %s (0x%02X) variant=%d (%s)\n",
		     req_name, req_mec, rv,
		     slc_variant_names[rv] ? slc_variant_names[rv] : "?");
	} else if (req_mec == UECP_MEC_ACCESS_RIGHT) {
		uint8_t target_mec = (mel >= 2) ? data[2] : 0;
		uint8_t port = (mel >= 3) ? data[3] : 0;
		LOGP(DRADIO, LOGL_INFO, "UECP: Request for %s (0x%02X) target=%s (0x%02X) port=%d\n",
		     req_name, req_mec, uecp_mec_name(target_mec), target_mec, port);
	} else {
		LOGP(DRADIO, LOGL_INFO, "UECP: Request for %s (0x%02X) PSN=%d%s\n",
		     req_name, req_mec, req_psn,
		     (req_psn >= 2) ? (eon ? " (EON)" : " (EON, not found)") : "");
	}

	rlen = 0;

	switch (req_mec) {
	case UECP_MEC_PI: {
		uint16_t pi;
		if (eon) {
			pi = eon->pi;
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = req_psn;
		} else {
			pi = rds_enc_get_pi(enc);
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = UECP_PSN_MAIN;
		}
		resp[rlen++] = (pi >> 8) & 0xFF;
		resp[rlen++] = pi & 0xFF;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): PI=0x%04X%s\n",
		     req_name, req_mec, pi, eon ? " (EON)" : "");
		break;
	}
	case UECP_MEC_PS: {
		char ps[9];
		if (eon) {
			memcpy(ps, eon->ps, 8);
			ps[8] = '\0';
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = req_psn;
		} else {
			rds_enc_get_ps(enc, ps, sizeof(ps));
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = UECP_PSN_MAIN;
		}
		memcpy(resp + rlen, ps, 8);
		rlen += 8;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): \"%s\"%s\n",
		     req_name, req_mec, ps, eon ? " (EON)" : "");
		break;
	}
	case UECP_MEC_TP_TA: {
		int tp, ta;
		uint8_t flags;
		if (eon) {
			tp = eon->tp;
			ta = eon->ta;
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = req_psn;
		} else {
			tp = rds_enc_get_tp(enc);
			ta = rds_enc_get_ta(enc);
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = UECP_PSN_MAIN;
		}
		flags = (uint8_t)((tp << 1) | ta);
		resp[rlen++] = flags;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): flags=0x%02X TP=%d TA=%d (%s)%s\n",
		     req_name, req_mec, flags, tp, ta,
		     rds_get_tp_ta_description(tp, ta), eon ? " (EON)" : "");
		break;
	}
	case UECP_MEC_DI_PTYI: {
		int stereo, artificial, compressed, dynamic_pty;
		uint8_t flags;
		rds_enc_get_di(enc, &stereo, &artificial, &compressed, &dynamic_pty);
		flags = (uint8_t)((dynamic_pty << 3) | (compressed << 2) |
				  (artificial << 1) | stereo);
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = UECP_PSN_MAIN;
		resp[rlen++] = flags;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): flags=0x%02X "
		     "stereo=%d art=%d comp=%d dpty=%d\n",
		     req_name, req_mec, flags, stereo, artificial, compressed, dynamic_pty);
		break;
	}
	case UECP_MEC_PTY: {
		uint8_t pty;
		if (eon) {
			pty = eon->pty;
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = req_psn;
		} else {
			pty = rds_enc_get_pty(enc);
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = UECP_PSN_MAIN;
		}
		resp[rlen++] = pty;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): PTY=%d (%s)%s\n",
		     req_name, req_mec, pty,
		     rds_get_pty_name(pty, RDS_IS_RBDS(enc->ecc)),
		     eon ? " (EON)" : "");
		break;
	}
	case UECP_MEC_RT: {
		char rt[65];
		int i, text_len;
		rds_enc_get_radiotext(enc, rt, sizeof(rt));
		/* Find actual text length (before CR terminators) */
		text_len = 64;
		for (i = 0; i < 64; i++) {
			if (rt[i] == '\r') { text_len = i; break; }
		}
		/* UECP RT response: config(1) + text only (no CR padding).
		 * MEL delimits the field — controller knows length from MEL. */
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = UECP_PSN_MAIN;
		resp[rlen++] = (uint8_t)(text_len + 1); /* MEL: config + text */
		resp[rlen++] = (uint8_t)(enc->rt_ab & 0x01);
		if (text_len > 0) {
			memcpy(resp + rlen, rt, text_len);
			rlen += text_len;
		}
		rt[text_len] = '\0';
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): A/B=%c \"%s\" (%d chars)\n",
		     req_name, req_mec, enc->rt_ab ? 'B' : 'A', rt, text_len);
		break;
	}
	case UECP_MEC_PTYN: {
		char ptyn[9];
		rds_enc_get_ptyn(enc, ptyn, sizeof(ptyn));
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = UECP_PSN_MAIN;
		memcpy(resp + rlen, ptyn, 8);
		rlen += 8;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): \"%s\"\n",
		     req_name, req_mec, ptyn);
		break;
	}
	case UECP_MEC_ECC: {
		/* IEC 62106-10 A.2.11: request includes variant code (data[3] when mel>=3)
		 * Reply format matches set: DSN + variant_byte(variant<<4 | data_msb) + data_lsb
		 * For 8-bit variants (ECC, Language): data_msb=0, data_lsb=value
		 * For 12-bit variants (TMC, Bcast, EWS): data_msb=high nibble, data_lsb=low byte */
		uint8_t req_variant = (mel >= 3) ? (data[3] & 0x07) : RDS_1A_VARIANT_ECC;
		const char *vname = slc_variant_names[req_variant];
		resp[rlen++] = UECP_DSN_CURRENT;
		switch (req_variant) {
		case RDS_1A_VARIANT_ECC: {
			uint8_t ecc = rds_enc_get_ecc(enc);
			resp[rlen++] = (RDS_1A_VARIANT_ECC << 4);
			resp[rlen++] = ecc;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): variant=%d (%s) ECC=0x%02X\n",
			     req_name, req_mec, req_variant, vname, ecc);
			break;
		}
		case RDS_1A_VARIANT_TMC_ID: {
			uint16_t tmc = rds_enc_get_tmc_id(enc);
			resp[rlen++] = (RDS_1A_VARIANT_TMC_ID << 4) | ((tmc >> 8) & 0x0F);
			resp[rlen++] = tmc & 0xFF;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): variant=%d (%s) TMC=0x%03X\n",
			     req_name, req_mec, req_variant, vname, tmc);
			break;
		}
		case RDS_1A_VARIANT_LANGUAGE: {
			uint8_t lang = rds_enc_get_language(enc);
			resp[rlen++] = (RDS_1A_VARIANT_LANGUAGE << 4);
			resp[rlen++] = lang;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): variant=%d (%s) LIC=%d\n",
			     req_name, req_mec, req_variant, vname, lang);
			break;
		}
		case RDS_1A_VARIANT_BCAST: {
			uint16_t bc = rds_enc_get_slc_broadcaster(enc);
			resp[rlen++] = (RDS_1A_VARIANT_BCAST << 4) | ((bc >> 8) & 0x0F);
			resp[rlen++] = bc & 0xFF;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): variant=%d (%s) Bcast=0x%03X\n",
			     req_name, req_mec, req_variant, vname, bc);
			break;
		}
		case RDS_1A_VARIANT_EWS: {
			uint16_t ews = rds_enc_get_ews_channel(enc);
			resp[rlen++] = (RDS_1A_VARIANT_EWS << 4) | ((ews >> 8) & 0x0F);
			resp[rlen++] = ews & 0xFF;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): variant=%d (%s) EWS=%d\n",
			     req_name, req_mec, req_variant, vname, ews);
			break;
		}
		default:
			resp[rlen++] = (req_variant << 4);
			resp[rlen++] = 0x00;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): variant=%d (%s) data=0\n",
			     req_name, req_mec, req_variant, vname);
			break;
		}
		break;
	}
	case UECP_MEC_RDS_ONOFF:
		resp[rlen++] = UECP_FLAG_ON; /* always on */
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): on=%d (always on)\n",
		     req_name, req_mec, UECP_FLAG_ON);
		break;
	case UECP_MEC_RDS_PHASE: {
		uint8_t ref = (mel >= 2) ? (data[2] & 0x07) : 0;
		resp[rlen++] = (ref << 5); /* ref in bits 7-5, phase MSB = 0 */
		resp[rlen++] = UECP_RDS_PHASE_DEFAULT & 0xFF;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): ref=%d phase=%d.%d°\n",
		     req_name, req_mec, ref,
		     UECP_RDS_PHASE_DEFAULT / 10, UECP_RDS_PHASE_DEFAULT % 10);
		break;
	}
	case UECP_MEC_RDS_LEVEL: {
		uint8_t ref = (mel >= 2) ? (data[2] & 0x07) : 0;
		resp[rlen++] = (ref << 5) | ((UECP_RDS_LEVEL_DEFAULT >> 8) & 0x1F);
		resp[rlen++] = UECP_RDS_LEVEL_DEFAULT & 0xFF;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): ref=%d level=%d mVpp\n",
		     req_name, req_mec, ref, UECP_RDS_LEVEL_DEFAULT);
		break;
	}
	case UECP_MEC_SITE_ADDR:
		resp[rlen++] = UECP_ADDR_CTRL_ADD;
		resp[rlen++] = (srv->site_addr >> 8) & 0x03;
		resp[rlen++] = srv->site_addr & 0xFF;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): addr=0x%03X\n",
		     req_name, req_mec, srv->site_addr);
		break;
	case UECP_MEC_ENC_ADDR:
		resp[rlen++] = UECP_ADDR_CTRL_ADD;
		resp[rlen++] = srv->enc_addr & UECP_ENC_ADDR_MASK;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): addr=0x%02X\n",
		     req_name, req_mec, srv->enc_addr);
		break;
	case UECP_MEC_ACCESS_RIGHT: {
		/* IEC 62106-10 A.6.14: reply format = target_MEC + port + enable_bit
		 * Request: data[2]=target_MEC, data[3]=port */
		uint8_t target_mec = (mel >= 2) ? data[2] : 0;
		uint8_t port = (mel >= 3) ? data[3] : 0;
		resp[rlen++] = target_mec;
		resp[rlen++] = port;
		resp[rlen++] = UECP_ACCESS_ENABLED;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): target=%s (0x%02X) port=%d enabled\n",
		     req_name, req_mec, uecp_mec_name(target_mec), target_mec, port);
		break;
	}
	case UECP_MEC_MS: {
		int ms = rds_enc_get_ms(enc);
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = UECP_PSN_MAIN;
		resp[rlen++] = (uint8_t)ms;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): M/S=%d (%s)\n",
		     req_name, req_mec, ms, rds_get_ms_name(ms));
		break;
	}
	case UECP_MEC_CT_ONOFF:
		resp[rlen++] = (uint8_t)(enc->ct_enabled ? UECP_FLAG_ON : UECP_FLAG_OFF);
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): ct=%s\n",
		     req_name, req_mec, enc->ct_enabled ? "on" : "off");
		break;
	case UECP_MEC_AF: {
		/* Return actual AF list from encoder (Method A or B) */
		uint8_t af_buf[64];
		const char *af_method;
		int af_freqs;
		int af_n = uecp_build_af_codes(enc, af_buf, (int)sizeof(af_buf),
						&af_method, &af_freqs);
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = UECP_PSN_MAIN;
		if (af_n > 0) {
			resp[rlen++] = (uint8_t)(af_n + 2); /* MEL: start_loc(2) + codes */
			resp[rlen++] = 0x00; /* start location high */
			resp[rlen++] = 0x00; /* start location low */
			memcpy(resp + rlen, af_buf, af_n);
			rlen += af_n;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): Method %s, "
			     "%d codes, %d freqs\n",
			     req_name, req_mec, af_method, af_n, af_freqs);
		} else {
			resp[rlen++] = 0x02; /* MEL: start_loc(2) only */
			resp[rlen++] = 0x00;
			resp[rlen++] = 0x00;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): empty list (no AFs)\n",
			     req_name, req_mec);
		}
		break;
	}
	case UECP_MEC_EON_AF:
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = req_psn;
		if (eon && eon->af_count > 0) {
			/* Build AF code list from EON entry frequencies */
			int af_n = 0;
			uint8_t af_buf[52]; /* max 25 AFs + count + terminator */
			for (int i = 0; i < eon->af_count && af_n < (int)sizeof(af_buf); i++) {
				uint8_t code = (uint8_t)(eon->af[i] - RDS_AF_FM_BASE);
				if (code >= 1 && code <= 204)
					af_buf[af_n++] = code;
			}
			resp[rlen++] = (uint8_t)(af_n + 2); /* MEL: start_loc(2) + codes */
			resp[rlen++] = 0x00; /* start location high */
			resp[rlen++] = 0x00; /* start location low */
			memcpy(resp + rlen, af_buf, af_n);
			rlen += af_n;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): %d EON-AFs for PI=0x%04X\n",
			     req_name, req_mec, af_n, eon->pi);
		} else {
			resp[rlen++] = 0x02; /* MEL: start_loc(2) only */
			resp[rlen++] = 0x00;
			resp[rlen++] = 0x00;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): empty list (0 EON-AFs)%s\n",
			     req_name, req_mec,
			     (req_psn >= 2 && !eon) ? " (no EON entry)" : "");
		}
		break;
	case UECP_MEC_MAKE_PSN_LIST:
		/* Return current PSN list - single main service */
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = 0x01; /* MEL: 1 PSN */
		resp[rlen++] = UECP_PSN_MAIN;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): 1 service (PSN=%d)\n",
		     req_name, req_mec, UECP_PSN_MAIN);
		break;
	case UECP_MEC_PSN_ENABLE:
		/* Return PSN enable status - main service always enabled */
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = 0x02; /* MEL: 1 pair */
		resp[rlen++] = UECP_FLAG_ON; /* enabled */
		resp[rlen++] = UECP_PSN_MAIN;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): PSN=%d enabled\n",
		     req_name, req_mec, UECP_PSN_MAIN);
		break;
	case UECP_MEC_GROUP_SEQ: {
		/* Return current group sequence from scheduler */
		int i;
		resp[rlen++] = UECP_DSN_CURRENT;
		if (enc->group_sched_len > 0) {
			resp[rlen++] = (uint8_t)enc->group_sched_len;
			for (i = 0; i < enc->group_sched_len && rlen < (int)sizeof(resp) - 1; i++)
				resp[rlen++] = (uint8_t)enc->group_sched_buffer[i];
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): %d groups in sequence\n",
			     req_name, req_mec, enc->group_sched_len);
		} else {
			resp[rlen++] = 0x01; /* MEL: 1 entry */
			resp[rlen++] = 0x00; /* Group 0A */
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): default (0A only)\n",
			     req_name, req_mec);
		}
		break;
	}
	case UECP_MEC_GROUP_VARIANT: {
		/* Return actual SLC variant sequence for requested group type.
		 * Group type byte follows DSN+PSN when present:
		 *   mel=2: data[2]=group_type (no DSN/PSN)
		 *   mel=3: DSN+PSN only, no group_type → default 1A
		 *   mel>=4: data[4]=group_type (after DSN+PSN) */
		uint8_t gt;
		if (mel >= 4)
			gt = data[4];		/* after MEC+DSN+PSN */
		else if (mel >= 3)
			gt = 0x02;		/* DSN+PSN but no group type → default 1A */
		else if (mel >= 2)
			gt = data[2];		/* no DSN/PSN, group type directly */
		else
			gt = 0x02;		/* nothing → default 1A */
		int group_num = (gt >> 1) & 0x0F;
		int group_ver = gt & 0x01;

		/* SLC variants only apply to Group 1A */
		if (group_num == 1 && group_ver == 0) {
			int variants[RDS_1A_VARIANT_MAX];
			int nv = rds_enc_get_slc_variants(enc, variants, RDS_1A_VARIANT_MAX);

			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = req_psn;
			resp[rlen++] = gt;
			for (int i = 0; i < nv; i++)
				resp[rlen++] = (uint8_t)variants[i];

			/* Build human-readable log */
			char vbuf[64];
			int vpos = 0;
			for (int i = 0; i < nv && vpos < (int)sizeof(vbuf) - 12; i++) {
				int v = variants[i];
				const char *vname = (v >= 0 && v <= 7) ? slc_variant_names[v] : "?";
				vpos += snprintf(vbuf + vpos, sizeof(vbuf) - vpos,
						 "%s%d=%s", i ? "," : "", v, vname);
			}
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): group=1A, %d variants [%s]\n",
			     req_name, req_mec, nv, vbuf);
		} else {
			/* Non-1A groups: no variant sequence */
			resp[rlen++] = UECP_DSN_CURRENT;
			resp[rlen++] = req_psn;
			resp[rlen++] = gt;
			LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): group=%d%c (no variants)\n",
			     req_name, req_mec, group_num, group_ver ? 'B' : 'A');
		}
		break;
	}
	case UECP_MEC_EON_ENABLE: {
		/* Build enable flags from actual EON entry content */
		uint8_t eon_flags = 0;
		if (eon) {
			/* Per-entry: use stored flags if set, otherwise build from content */
			eon_flags = eon->enable_flags ? eon->enable_flags
						      : rds_enc_eon_build_enable_flags(eon);
		} else if (req_psn <= 1 && enc->eon_tx_count > 0 && enc->eon_enabled) {
			/* Main PSN: OR all entry flags together */
			for (int i = 0; i < enc->eon_tx_count; i++) {
				uint8_t f = enc->eon_tx[i].enable_flags
					  ? enc->eon_tx[i].enable_flags
					  : rds_enc_eon_build_enable_flags(&enc->eon_tx[i]);
				eon_flags |= f;
			}
		}
		resp[rlen++] = UECP_DSN_CURRENT;
		resp[rlen++] = req_psn;
		resp[rlen++] = eon_flags;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): flags=0x%02X "
		     "(PS=%d AF=%d Link=%d PTY=%d PIN=%d Bcast=%d TA=%d)\n",
		     req_name, req_mec, eon_flags,
		     !!(eon_flags & RDS_EON_FLAG_PS), !!(eon_flags & RDS_EON_FLAG_AF),
		     !!(eon_flags & RDS_EON_FLAG_LINK), !!(eon_flags & RDS_EON_FLAG_PTY),
		     !!(eon_flags & RDS_EON_FLAG_PIN), !!(eon_flags & RDS_EON_FLAG_BCAST),
		     !!(eon_flags & RDS_EON_FLAG_TA));
		break;
	}
	case UECP_MEC_MANUFACTURER: {
		/* Return manufacturer ID + model string */
		int slen = (int)strlen(UECP_MFR_MODEL);
		resp[rlen++] = UECP_MFR_ID_HI;
		resp[rlen++] = UECP_MFR_ID_LO;
		memcpy(resp + rlen, UECP_MFR_MODEL, slen);
		rlen += slen;
		LOGP(DRADIO, LOGL_INFO, "UECP: Reply %s (0x%02X): \"%c%c\" \"%s\"\n",
		     req_name, req_mec, UECP_MFR_ID_HI, UECP_MFR_ID_LO, UECP_MFR_MODEL);
		break;
	}
	default:
		LOGP(DRADIO, LOGL_NOTICE, "UECP: Request for unsupported %s (0x%02X)\n", req_name, req_mec);
		return;
	}

	uecp_send_mec_response(srv, req_mec, resp, rlen);
}

/* Process single UECP message element */
static void uecp_process_mec(rds_server_t *srv, uint8_t mec,
			     const uint8_t *data, int len)
{
	rds_encoder_t *enc = srv->encoder;
	const char *name = uecp_mec_name(mec);

	if (!enc) {
		LOGP(DRADIO, LOGL_DEBUG, "UECP: No encoder attached\n");
		return;
	}

	/* Most MECs have DSN(1) + PSN(1) prefix.
	 * PSN 0-1 = main programme, PSN >= 2 = EON entry (index PSN-2) */
	uint8_t psn = UECP_PSN_MAIN;
	int is_eon = 0;

	switch (mec) {
	/* MECs with DSN + PSN prefix */
	case UECP_MEC_PI:
	case UECP_MEC_PS:
	case UECP_MEC_TP_TA:
	case UECP_MEC_DI_PTYI:
	case UECP_MEC_PTY:
	case UECP_MEC_MS:
	case UECP_MEC_PTYN:
	case UECP_MEC_AF:
	case UECP_MEC_EON_AF:
	case UECP_MEC_EON_ENABLE:
		if (len >= 2)
			psn = data[1];
		is_eon = (psn >= 2);
		break;
	default:
		break;
	}

	switch (mec) {
	case UECP_MEC_PI:
		if (len >= 4) {
			uint16_t pi = ((uint16_t)data[2] << 8) | data[3];
			if (is_eon) {
				rds_eon_entry_t *eon = rds_enc_eon_ensure(enc, pi);
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) PSN=%d: EON PI=0x%04X%s\n",
				     name, mec, psn, pi, eon ? "" : " (FAILED)");
			} else {
				rds_enc_set_pi(enc, pi);
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): PI=0x%04X\n", name, mec, pi);
			}
		}
		break;

	case UECP_MEC_PS:
		if (len >= 10) {
			char ps[9];
			memcpy(ps, data + 2, 8);
			ps[8] = '\0';
			if (is_eon) {
				/* Need PI to find EON entry — get from eon_tx by PSN index */
				const rds_eon_entry_t *eon = rds_enc_eon_get_by_index(enc, psn - 2);
				if (eon) {
					rds_enc_eon_set_ps(enc, eon->pi, ps);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) PSN=%d: EON PS=\"%s\" (PI=0x%04X)\n",
					     name, mec, psn, ps, eon->pi);
				} else {
					LOGP(DRADIO, LOGL_NOTICE, "UECP: Set %s (0x%02X) PSN=%d: no EON entry at index %d\n",
					     name, mec, psn, psn - 2);
				}
			} else {
				rds_enc_set_ps(enc, ps);
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): \"%s\"\n", name, mec, ps);
			}
		}
		break;

	case UECP_MEC_TP_TA:
		if (len >= 3) {
			int ta = data[2] & 0x01;
			int tp = (data[2] >> 1) & 0x01;
			if (is_eon) {
				const rds_eon_entry_t *eon = rds_enc_eon_get_by_index(enc, psn - 2);
				if (eon) {
					rds_enc_eon_set_tp(enc, eon->pi, tp);
					rds_enc_eon_set_ta(enc, eon->pi, ta);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) PSN=%d: EON TP=%d TA=%d (PI=0x%04X)\n",
					     name, mec, psn, tp, ta, eon->pi);
				} else {
					LOGP(DRADIO, LOGL_NOTICE, "UECP: Set %s (0x%02X) PSN=%d: no EON entry at index %d\n",
					     name, mec, psn, psn - 2);
				}
			} else {
				rds_enc_set_tp(enc, tp);
				rds_enc_set_ta(enc, ta);
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): TP=%d TA=%d (%s)\n",
				     name, mec, tp, ta, rds_get_tp_ta_description(tp, ta));
			}
		}
		break;

	case UECP_MEC_DI_PTYI:
		if (len >= 3) {
			int stereo = data[2] & 0x01;
			int artificial = (data[2] >> 1) & 0x01;
			int compressed = (data[2] >> 2) & 0x01;
			int dynamic_pty = (data[2] >> 3) & 0x01;
			rds_enc_set_di(enc, stereo, artificial, compressed, dynamic_pty);
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): stereo=%d art=%d comp=%d dpty=%d\n",
			     name, mec, stereo, artificial, compressed, dynamic_pty);
		}
		break;

	case UECP_MEC_PTY:
		if (len >= 3) {
			uint8_t pty = data[2] & 0x1F;
			if (is_eon) {
				const rds_eon_entry_t *eon = rds_enc_eon_get_by_index(enc, psn - 2);
				if (eon) {
					rds_enc_eon_set_pty(enc, eon->pi, pty);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) PSN=%d: EON PTY=%d (PI=0x%04X)\n",
					     name, mec, psn, pty, eon->pi);
				} else {
					LOGP(DRADIO, LOGL_NOTICE, "UECP: Set %s (0x%02X) PSN=%d: no EON entry at index %d\n",
					     name, mec, psn, psn - 2);
				}
			} else {
				rds_enc_set_pty(enc, pty);
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): PTY=%d (%s)\n",
				     name, mec, pty,
				     rds_get_pty_name(pty, RDS_IS_RBDS(enc->ecc)));
			}
		}
		break;

	case UECP_MEC_RT:
		if (len >= 4) {
			int mel = data[2];
			int text_len = mel - 1;
			if (text_len > 0 && text_len <= 64 && len >= 4 + text_len) {
				char rt[65];
				memcpy(rt, data + 4, text_len);
				rt[text_len] = '\0';
				rds_enc_set_radiotext(enc, rt);
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): \"%s\"\n", name, mec, rt);
			}
		}
		break;

	case UECP_MEC_PTYN:
		if (len >= 10) {
			char ptyn[9];
			memcpy(ptyn, data + 2, 8);
			ptyn[8] = '\0';
			rds_enc_set_ptyn(enc, ptyn);
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): \"%s\"\n", name, mec, ptyn);
		}
		break;

	case UECP_MEC_ECC:
		if (len >= 2) {
			uint8_t variant = (data[1] >> 4) & 0x07;
			const char *vname = slc_variant_names[variant];
			switch (variant) {
			case RDS_1A_VARIANT_ECC:
				if (len >= 3) {
					rds_enc_set_ecc(enc, data[2]);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) variant=%d (%s): ECC=0x%02X\n",
					     name, mec, variant, vname, data[2]);
				}
				break;
			case RDS_1A_VARIANT_TMC_ID:
				if (len >= 4) {
					uint16_t tmc = ((uint16_t)(data[2] & 0x0F) << 8) | data[3];
					rds_enc_set_tmc_id(enc, tmc);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) variant=%d (%s): TMC=0x%03X\n",
					     name, mec, variant, vname, tmc);
				}
				break;
			case RDS_1A_VARIANT_LANGUAGE:
				if (len >= 3) {
					rds_enc_set_language(enc, data[2]);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) variant=%d (%s): LIC=%d\n",
					     name, mec, variant, vname, data[2]);
				}
				break;
			case RDS_1A_VARIANT_BCAST:
				if (len >= 4) {
					uint16_t bc = ((uint16_t)(data[2] & 0x0F) << 8) | data[3];
					rds_enc_set_slc_broadcaster(enc, bc);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) variant=%d (%s): Bcast=0x%03X\n",
					     name, mec, variant, vname, bc);
				}
				break;
			case RDS_1A_VARIANT_EWS:
				if (len >= 4) {
					uint16_t ews = ((uint16_t)(data[2] & 0x0F) << 8) | data[3];
					rds_enc_set_ews_channel(enc, ews);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) variant=%d (%s): EWS=%d\n",
					     name, mec, variant, vname, ews);
				}
				break;
			default:
				LOGP(DRADIO, LOGL_NOTICE, "UECP: Set %s (0x%02X) variant=%d (%s): unsupported\n",
				     name, mec, variant, vname);
				break;
			}
		}
		break;

	case UECP_MEC_REQUEST:
		uecp_handle_request(srv, data, len);
		break;

	case UECP_MEC_CT_ONOFF:
		if (len >= 1) {
			int ct_on = data[0] & UECP_FLAG_ON;
			if (enc)
				enc->ct_enabled = ct_on;
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): ct=%s\n",
			     name, mec, ct_on ? "on" : "off");
		}
		break;

	case UECP_MEC_MS:
		if (len >= 3) {
			int ms = data[2] & 0x01;
			rds_enc_set_ms(enc, ms);
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): M/S=%d (%s)\n",
			     name, mec, ms, ms ? "Music" : "Speech");
		}
		break;

	case UECP_MEC_RTC_CORR:
		if (len >= 2) {
			int16_t corr_ms = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
			enc->ct_time_offset = corr_ms / 1000;  /* Store as whole seconds */
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): correction=%d ms\n",
			     name, mec, corr_ms);
		}
		break;

	case UECP_MEC_PSN_ENABLE:
		/* PSN enable/disable - we only have a single programme service,
		 * so just acknowledge and log */
		if (len >= 2) {
			int mel_val = data[1];
			int n_pairs = mel_val / 2;
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): %d PSN entries (single-service encoder, ignored)\n",
			     name, mec, n_pairs);
		}
		break;

	case UECP_MEC_MAKE_PSN_LIST:
		/* Make PSN list - we only have a single programme service,
		 * so just acknowledge and log */
		if (len >= 2) {
			int mel_val = data[1];
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): %d services (single-service encoder, ignored)\n",
			     name, mec, mel_val);
		}
		break;

	case UECP_MEC_GROUP_SEQ:
		/* Group sequence - log the requested sequence */
		if (len >= 2) {
			int mel_val = data[1];
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): %d groups in sequence (scheduler handles)\n",
			     name, mec, mel_val);
		}
		break;

	case UECP_MEC_GROUP_VARIANT:
		/* Group variant code sequence - log */
		if (len >= 3) {
			int mel_val = data[1];
			uint8_t group_type = data[2];
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): group=%d%c, %d variants (scheduler handles)\n",
			     name, mec, (group_type >> 1) & 0x0F, (group_type & 0x01) ? 'B' : 'A',
			     mel_val - 1);
		}
		break;

	case UECP_MEC_EON_ENABLE:
		/* EON elements enable/disable */
		if (len >= 3) {
			uint8_t flags = data[2];
			if (is_eon) {
				const rds_eon_entry_t *eon = rds_enc_eon_get_by_index(enc, psn - 2);
				if (eon) {
					rds_enc_eon_set_enable_flags(enc, eon->pi, flags);
					LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) PSN=%d: EON flags=0x%02X "
					     "(PS=%d AF=%d Link=%d PTY=%d PIN=%d Bcast=%d TA=%d) PI=0x%04X\n",
					     name, mec, psn, flags,
					     !!(flags & RDS_EON_FLAG_PS), !!(flags & RDS_EON_FLAG_AF),
					     !!(flags & RDS_EON_FLAG_LINK), !!(flags & RDS_EON_FLAG_PTY),
					     !!(flags & RDS_EON_FLAG_PIN), !!(flags & RDS_EON_FLAG_BCAST),
					     !!(flags & RDS_EON_FLAG_TA), eon->pi);
				} else {
					LOGP(DRADIO, LOGL_NOTICE, "UECP: Set %s (0x%02X) PSN=%d: no EON entry at index %d\n",
					     name, mec, psn, psn - 2);
				}
			} else {
				/* Main PSN: enable/disable EON globally */
				enc->eon_enabled = (flags != 0) ? 1 : 0;
				rds_scheduler_update(enc);
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): flags=0x%02X %s "
				     "(PS=%d AF=%d Link=%d PTY=%d PIN=%d Bcast=%d TA=%d)\n",
				     name, mec, flags, enc->eon_enabled ? "enabled" : "disabled",
				     !!(flags & RDS_EON_FLAG_PS), !!(flags & RDS_EON_FLAG_AF),
				     !!(flags & RDS_EON_FLAG_LINK), !!(flags & RDS_EON_FLAG_PTY),
				     !!(flags & RDS_EON_FLAG_PIN), !!(flags & RDS_EON_FLAG_BCAST),
				     !!(flags & RDS_EON_FLAG_TA));
			}
		}
		break;

	case UECP_MEC_RDS_ONOFF:
		if (len >= 1) {
			int rds_on = data[0] & UECP_FLAG_ON;
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): %s (always on)\n",
			     name, mec, rds_on ? "on" : "off");
		}
		break;

	case UECP_MEC_RDS_PHASE:
		if (len >= 2) {
			uint8_t ref = (data[0] >> 5) & 0x07;
			uint16_t phase = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): ref=%d phase=%d.%d° (DSP handles)\n",
			     name, mec, ref, phase / 10, phase % 10);
		}
		break;

	case UECP_MEC_RDS_LEVEL:
		if (len >= 2) {
			uint8_t ref = (data[0] >> 5) & 0x07;
			uint16_t level = ((uint16_t)(data[0] & 0x1F) << 8) | data[1];
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): ref=%d level=%d mVpp (DSP handles)\n",
			     name, mec, ref, level);
		}
		break;

	case UECP_MEC_SITE_ADDR:
		if (len >= 3) {
			uint8_t ctrl = data[0] & 0x03;
			uint16_t addr = ((uint16_t)(data[1] & 0x03) << 8) | data[2];
			if (ctrl == UECP_ADDR_CTRL_ADD) {
				srv->site_addr = addr;
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): add 0x%03X\n", name, mec, addr);
			} else if (ctrl == UECP_ADDR_CTRL_REMOVE) {
				if (srv->site_addr == addr)
					srv->site_addr = UECP_ADDR_GLOBAL;
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): remove 0x%03X\n", name, mec, addr);
			} else if (ctrl == UECP_ADDR_CTRL_CLEAR) {
				srv->site_addr = UECP_ADDR_GLOBAL;
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): clear all\n", name, mec);
			}
		}
		break;

	case UECP_MEC_ENC_ADDR:
		if (len >= 2) {
			uint8_t ctrl = data[0] & 0x03;
			uint8_t addr = data[1] & UECP_ENC_ADDR_MASK;
			if (ctrl == UECP_ADDR_CTRL_ADD) {
				srv->enc_addr = addr;
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): add 0x%02X\n", name, mec, addr);
			} else if (ctrl == UECP_ADDR_CTRL_REMOVE) {
				if (srv->enc_addr == addr)
					srv->enc_addr = UECP_ADDR_GLOBAL;
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): remove 0x%02X\n", name, mec, addr);
			} else if (ctrl == UECP_ADDR_CTRL_CLEAR) {
				srv->enc_addr = UECP_ADDR_GLOBAL;
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): clear all\n", name, mec);
			}
		}
		break;

	case UECP_MEC_ACCESS_RIGHT:
		if (len >= 3) {
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): target=%s (0x%02X) port=%d enable=%d\n",
			     name, mec, uecp_mec_name(data[0]), data[0], data[1], data[2] & 0x01);
		}
		break;

	case UECP_MEC_MANUFACTURER:
		/* Manufacturer-specific command — log and ignore */
		if (len >= 2) {
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): mfr=\"%c%c\" %d data bytes\n",
			     name, mec, data[0], data[1], len - 2);
		}
		break;

	case UECP_MEC_ACK:
		LOGP(DRADIO, LOGL_INFO, "UECP: Received %s (0x%02X) from client\n", name, mec);
		break;

	case UECP_MEC_RTC:
		if (len >= 8) {
			uint8_t year = data[0];
			uint8_t month = data[1];
			uint8_t day = data[2];
			uint8_t hour = data[3];
			uint8_t minute = data[4];
			uint8_t second = data[5];
			uint8_t centisec = data[6];
			uint8_t lto = data[7];
			/* We use system time for CT, but log the received time
			 * and update local_offset if provided */
			if (lto != 0xFF) {
				/* Bits 5-0: magnitude in half-hours, bit 5: sign (1=negative) */
				int8_t offset = (int8_t)(lto & 0x3F);
				if (lto & 0x20)
					offset = -offset;
				enc->local_offset = offset;
			}
			LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X): 20%02d-%02d-%02d %02d:%02d:%02d.%02d UTC, LTO=%s%d.%dh\n",
			     name, mec, year, month, day, hour, minute, second, centisec,
			     (enc->local_offset < 0) ? "-" : "+",
			     abs(enc->local_offset) / 2, (abs(enc->local_offset) % 2) * 5);
		}
		break;

	case UECP_MEC_AF:
		if (len >= 5) {
			/* DSN(1) + PSN(1) + MEL(1) + start_loc(2) + AF data
			 * AF data is the on-air code sequence: either Method A
			 * (individual codes) or Method B (count+tuning+pairs per list).
			 * Method B: count = 1 + 2*N (always odd, ≥3), pairs contain
			 * tuning freq with ascending=same, descending=regional. */
			int af_mel = data[2];
			int af_data_len = af_mel - 2; /* subtract start_loc(2) */
			uint16_t af_offset = ((uint16_t)data[3] << 8) | data[4];
			if (af_data_len > 0 && len >= 5 + af_data_len) {
				const uint8_t *af = data + 5;
				int i = 0;

				/* Detect Method B: first byte is count code with odd
				 * count ≥ 3, second byte is valid VHF tuning code,
				 * and pairs reference the tuning frequency */
				int is_method_b = 0;
				if (af_data_len >= 4 && af[0] >= RDS_AF_COUNT_MIN &&
				    af[0] <= RDS_AF_COUNT_MAX) {
					int cnt = af[0] - RDS_AF_NO_AF;
					uint8_t tuning = af[1];
					/* Method B: odd count ≥ 3, valid tuning code,
					 * and first pair references tuning */
					if (cnt >= 3 && (cnt & 1) && tuning >= 1 &&
					    tuning <= 204 && af_data_len >= 4 &&
					    (af[2] == tuning || af[3] == tuning))
						is_method_b = 1;
				}

				if (is_method_b) {
					/* Method B: parse [count, tuning, pairs...] lists */
					rds_enc_af_clear(enc);
					int list_num = 0;

					while (i < af_data_len) {
						uint8_t code = af[i];

						/* Terminator */
						if (code == RDS_AF_NOT_USED)
							break;

						/* Expect count code */
						if (code < RDS_AF_COUNT_MIN || code > RDS_AF_COUNT_MAX) {
							i++;
							continue;
						}

						int cnt = code - RDS_AF_NO_AF;
						int n_pairs = (cnt - 1) / 2;
						i++; /* skip count */

						if (i >= af_data_len)
							break;

						uint8_t tuning_code = af[i++];
						uint16_t tuning_freq = tuning_code + RDS_AF_FM_BASE;

						uint16_t af_freqs[RDS_AF_METHOD_B_MAX_AFS];
						uint8_t af_regional[RDS_AF_METHOD_B_MAX_AFS];
						int af_count = 0;

						for (int p = 0; p < n_pairs && i + 1 < af_data_len; p++) {
							uint8_t f1 = af[i++];
							uint8_t f2 = af[i++];

							/* Skip filler pairs */
							if (f1 == RDS_AF_FILLER || f2 == RDS_AF_FILLER)
								continue;

							/* Identify which is the AF (not tuning) */
							uint8_t af_code;
							if (f1 == tuning_code)
								af_code = f2;
							else if (f2 == tuning_code)
								af_code = f1;
							else {
								/* Neither is tuning — take the non-tuning one */
								af_code = (f1 != tuning_code) ? f1 : f2;
							}

							if (af_code < 1 || af_code > 204)
								continue;

							if (af_count < RDS_AF_METHOD_B_MAX_AFS) {
								af_freqs[af_count] = af_code + RDS_AF_FM_BASE;
								/* Descending (F1 > F2) = regional */
								af_regional[af_count] = (f1 > f2) ? 1 : 0;
								af_count++;
							}
						}

						if (af_count > 0) {
							rds_enc_af_method_b_add_list(enc, tuning_freq,
								af_freqs, af_regional, af_count);
							LOGP(DRADIO, LOGL_INFO,
							     "UECP: Set %s (0x%02X): Method B list[%d] tuning=%.1f, %d AFs\n",
							     name, mec, list_num,
							     tuning_freq / 10.0, af_count);
							list_num++;
						}
					}

					if (list_num == 0) {
						LOGP(DRADIO, LOGL_INFO,
						     "UECP: Set %s (0x%02X): cleared (no valid Method B lists)\n",
						     name, mec);
					}
				} else {
					/* Method A: individual AF codes */
					char af_str[256];
					int pos = 0;
					int freq_count = 0;
					int skip_next = 0;

					for (i = 0; i < af_data_len; i++) {
						uint8_t code = af[i];

						if (skip_next) {
							skip_next = 0;
							continue;
						}
						if (code == RDS_AF_LF_MF_FOLLOWS) {
							skip_next = 1;
							continue;
						}
						if (code >= RDS_AF_COUNT_MIN || code == RDS_AF_FILLER ||
						    code == RDS_AF_NOT_USED)
							continue;

						int freq_tenths = code + RDS_AF_FM_BASE;
						int n = snprintf(af_str + pos, sizeof(af_str) - pos,
								 "%s%d.%d", freq_count ? "," : "",
								 freq_tenths / 10, freq_tenths % 10);
						if (n < 0 || pos + n >= (int)sizeof(af_str))
							break;
						pos += n;
						freq_count++;
					}
					af_str[pos] = '\0';

					if (freq_count > 0) {
						rds_enc_af_set_method_a(enc, af_str);
						LOGP(DRADIO, LOGL_INFO,
						     "UECP: Set %s (0x%02X): Method A, %d freqs at offset %d: %s\n",
						     name, mec, freq_count, af_offset, af_str);
					} else {
						rds_enc_af_set_method_a(enc, NULL);
						LOGP(DRADIO, LOGL_INFO,
						     "UECP: Set %s (0x%02X): cleared (no valid AF codes)\n",
						     name, mec);
					}
				}
			}
		}
		break;

	case UECP_MEC_EON_AF:
		if (is_eon && len >= 5) {
			int eon_mel = data[2];
			int eon_data_len = eon_mel - 2;
			uint16_t eon_offset = ((uint16_t)data[3] << 8) | data[4];
			const rds_eon_entry_t *eon = rds_enc_eon_get_by_index(enc, psn - 2);
			if (eon && eon_data_len > 0 && len >= 5 + eon_data_len) {
				const uint8_t *af = data + 5;
				/* Clear existing AFs and parse new ones */
				rds_enc_eon_af_clear(enc, eon->pi);
				int af_count = 0;
				for (int i = 0; i < eon_data_len; i++) {
					uint8_t code = af[i];
					if (code == RDS_AF_NOT_USED || code == RDS_AF_FILLER)
						continue;
					if (code >= RDS_AF_COUNT_MIN)
						continue;
					if (code >= 1 && code <= 204) {
						uint16_t freq = code + RDS_AF_FM_BASE;
						rds_enc_eon_af_add(enc, eon->pi, freq);
						af_count++;
					}
				}
				LOGP(DRADIO, LOGL_INFO, "UECP: Set %s (0x%02X) PSN=%d: %d EON-AFs at offset %d (PI=0x%04X)\n",
				     name, mec, psn, af_count, eon_offset, eon->pi);
			} else if (!eon) {
				LOGP(DRADIO, LOGL_NOTICE, "UECP: Set %s (0x%02X) PSN=%d: no EON entry at index %d\n",
				     name, mec, psn, psn - 2);
			}
		} else if (!is_eon && len >= 5) {
			LOGP(DRADIO, LOGL_DEBUG, "UECP: Set %s (0x%02X): EON-AF for main programme (ignored)\n",
			     name, mec);
		}
		break;

	default:
		LOGP(DRADIO, LOGL_ERROR, "UECP: Unknown %s (0x%02X) (len=%d)\n", uecp_mec_name(mec), mec, len);
		break;
	}
}

/* Process complete UECP frame (after unstuffing, CRC validated) */
static void uecp_process_frame(rds_server_t *srv, const uint8_t *frame, int len)
{
	uint16_t site_addr;
	uint8_t enc_addr;
	uint8_t seq;
	uint8_t msg_len;
	const uint8_t *msg;
	int pos;
	
	/* Minimum frame: ADD(2) + SQC(1) + MFL(1) = 4 bytes */
	if (len < 4) {
		LOGP(DRADIO, LOGL_DEBUG, "UECP: Frame too short (%d)\n", len);
		return;
	}
	
	/* Parse header */
	site_addr = ((frame[0] & 0x03) << 8) | frame[1];  /* 10 bits */
	enc_addr = (frame[0] >> 2) & 0x3F;                /* 6 bits */
	seq = frame[2];
	msg_len = frame[3];
	msg = frame + 4;
	
	/* Verify message length */
	if (4 + msg_len > len) {
		LOGP(DRADIO, LOGL_DEBUG, "UECP: Message length mismatch\n");
		uecp_send_ack(srv, UECP_ACK_PARAM_ERR, seq);
		return;
	}
	
	/* Check addressing (UECP_ADDR_GLOBAL = matches all) */
	if (site_addr != UECP_ADDR_GLOBAL && site_addr != srv->site_addr) {
		LOGP(DRADIO, LOGL_DEBUG, "UECP: Site address mismatch\n");
		return;  /* Not for us, ignore silently */
	}
	if (enc_addr != UECP_ADDR_GLOBAL && enc_addr != srv->enc_addr) {
		LOGP(DRADIO, LOGL_DEBUG, "UECP: Encoder address mismatch\n");
		return;  /* Not for us, ignore silently */
	}
	
	/* Process message elements */
	pos = 0;
	while (pos < msg_len) {
		uint8_t mec = msg[pos];
		int mel;
		
		/* Determine message element length based on MEC */
		switch (mec) {
		case UECP_MEC_PI:
			mel = 4;  /* DSN + PSN + PI(2) */
			break;
		case UECP_MEC_PS:
			mel = 10; /* DSN + PSN + PS(8) */
			break;
		case UECP_MEC_TP_TA:
		case UECP_MEC_PTY:
			mel = 3;  /* DSN + PSN + data(1) */
			break;
		case UECP_MEC_RT:
			/* Variable length: DSN + PSN + MEL + config + text */
			if (pos + 4 > msg_len) goto frame_err;
			mel = 3 + msg[pos + 3];  /* DSN + PSN + MEL + data[MEL] */
			break;
		case UECP_MEC_PTYN:
			mel = 10; /* DSN + PSN + PTYN(8) */
			break;
		case UECP_MEC_ECC:
			mel = 3;  /* DSN + variant + data */
			break;
		case UECP_MEC_DI_PTYI:
			mel = 3;  /* DSN + PSN + flags */
			break;
		case UECP_MEC_MS:
			mel = 3;  /* DSN + PSN + M/S flag */
			break;
		case UECP_MEC_RTC:
			mel = 8;  /* year + month + date + hours + minutes + seconds + centisec + local_offset */
			break;
		case UECP_MEC_RTC_CORR:
			mel = 2;  /* RTCC high + RTCC low (16-bit signed ms) */
			break;
		case UECP_MEC_AF:
			/* Variable length: DSN + PSN + MEL + data */
			if (pos + 3 > msg_len) goto frame_err;
			mel = 3 + msg[pos + 3]; /* DSN + PSN + MEL + data */
			break;
		case UECP_MEC_EON_AF:
			/* Variable length: DSN + PSN + MEL + data */
			if (pos + 3 > msg_len) goto frame_err;
			mel = 3 + msg[pos + 3]; /* DSN + PSN + MEL + data */
			break;
		case UECP_MEC_PSN_ENABLE:
			/* Variable length: DSN + MEL + pairs */
			if (pos + 2 > msg_len) goto frame_err;
			mel = 2 + msg[pos + 2]; /* DSN + MEL + data */
			break;
		case UECP_MEC_MAKE_PSN_LIST:
			/* Variable length: DSN + MEL + PSN list */
			if (pos + 2 > msg_len) goto frame_err;
			mel = 2 + msg[pos + 2]; /* DSN + MEL + data */
			break;
		case UECP_MEC_GROUP_SEQ:
			/* Variable length: DSN + MEL + group types */
			if (pos + 2 > msg_len) goto frame_err;
			mel = 2 + msg[pos + 2]; /* DSN + MEL + data */
			break;
		case UECP_MEC_GROUP_VARIANT:
			/* Variable length: DSN + MEL + group type + variants */
			if (pos + 2 > msg_len) goto frame_err;
			mel = 2 + msg[pos + 2]; /* DSN + MEL + data */
			break;
		case UECP_MEC_EON_ENABLE:
			mel = 3;  /* DSN + PSN + flags */
			break;
		case UECP_MEC_REQUEST:
			/* Variable length: MEL + requested_MEC + [DSN + PSN + ...] */
			if (pos + 1 > msg_len) goto frame_err;
			mel = 1 + msg[pos + 1]; /* MEL byte + MEL bytes of data */
			break;
		case UECP_MEC_CT_ONOFF:
			mel = 1;  /* on/off (no DSN per IEC 62106-10 A.4.3) */
			break;
		case UECP_MEC_RDS_ONOFF:
			mel = 1;  /* on/off */
			break;
		case UECP_MEC_RDS_PHASE:
			mel = 2;  /* ref_table_entry + phase(2 bytes packed) */
			break;
		case UECP_MEC_RDS_LEVEL:
			mel = 2;  /* ref_table_entry + level(2 bytes packed) */
			break;
		case UECP_MEC_SITE_ADDR:
			mel = 3;  /* control + addr_hi + addr_lo */
			break;
		case UECP_MEC_ENC_ADDR:
			mel = 2;  /* control + addr */
			break;
		case UECP_MEC_ACCESS_RIGHT:
			mel = 3;  /* target_mec + port + enable */
			break;
		case UECP_MEC_MANUFACTURER:
			/* Variable length: MEL + mfr_id(2) + data */
			if (pos + 1 > msg_len) goto frame_err;
			mel = 1 + msg[pos + 1]; /* MEL byte + MEL bytes of data */
			break;
		case UECP_MEC_ACK:
			mel = 2;  /* code + [seq] */
			if (pos + 1 < msg_len && msg[pos + 1] != UECP_ACK_OK)
				mel = 3;
			break;
		default:
			/* Unknown MEC mid-frame - log and skip rest of frame */
			LOGP(DRADIO, LOGL_ERROR, "UECP: Unsupported %s (0x%02X) in frame, skipping remainder\n",
			     uecp_mec_name(mec), mec);
			goto frame_done;
		}
		
		if (pos + 1 + mel > msg_len) {
			LOGP(DRADIO, LOGL_DEBUG, "UECP: %s (0x%02X) truncated\n", uecp_mec_name(mec), mec);
			goto frame_err;
		}
		
		uecp_process_mec(srv, mec, msg + pos + 1, mel);
		pos += 1 + mel;
	}
	
frame_done:
	/* Send ACK if sequence counter enabled */
	if (seq != 0)
		uecp_send_ack(srv, UECP_ACK_OK, seq);
	return;
	
frame_err:
	uecp_send_ack(srv, UECP_ACK_PARAM_ERR, seq);
}

/* ============================================================
 * UECP Frame Parser (Byte Unstuffing)
 * ============================================================ */

/* Feed byte to UECP frame parser. Returns 1 if complete frame ready. */
static int uecp_feed_byte(rds_server_t *srv, uint8_t byte)
{
	/* Handle byte stuffing */
	if (srv->uecp_escaped) {
		srv->uecp_escaped = 0;
		switch (byte) {
		case 0x00: byte = UECP_STUFF; break;  /* 0xFD 0x00 -> 0xFD */
		case 0x01: byte = UECP_STA; break;    /* 0xFD 0x01 -> 0xFE */
		case 0x02: byte = UECP_STP; break;    /* 0xFD 0x02 -> 0xFF */
		default:
			/* Invalid escape sequence - reset */
			srv->uecp_in_frame = 0;
			srv->uecp_len = 0;
			return 0;
		}
		if (srv->uecp_len < UECP_FRAME_MAX)
			srv->uecp_frame[srv->uecp_len++] = byte;
		return 0;
	}
	
	/* Check for special bytes */
	switch (byte) {
	case UECP_STA:
		/* Start of frame */
		srv->uecp_in_frame = 1;
		srv->uecp_len = 0;
		return 0;
		
	case UECP_STP:
		/* End of frame - validate and process */
		if (srv->uecp_in_frame && srv->uecp_len >= 6) {
			/* Verify CRC (last 2 bytes) */
			uint16_t rx_crc = ((uint16_t)srv->uecp_frame[srv->uecp_len - 2] << 8) |
					  srv->uecp_frame[srv->uecp_len - 1];
			uint16_t calc_crc = uecp_crc16(srv->uecp_frame, srv->uecp_len - 2);
			
			if (rx_crc == calc_crc) {
				/* Valid frame - process (exclude CRC) */
				uecp_process_frame(srv, srv->uecp_frame, srv->uecp_len - 2);
			} else {
				LOGP(DRADIO, LOGL_DEBUG, "UECP: CRC error (rx=%04X calc=%04X)\n",
				     rx_crc, calc_crc);
				uecp_send_ack(srv, UECP_ACK_CRC_ERR, 0);
			}
		}
		srv->uecp_in_frame = 0;
		srv->uecp_len = 0;
		return 1;
		
	case UECP_STUFF:
		/* Escape sequence start */
		if (srv->uecp_in_frame)
			srv->uecp_escaped = 1;
		return 0;
		
	default:
		/* Regular data byte */
		if (srv->uecp_in_frame && srv->uecp_len < UECP_FRAME_MAX)
			srv->uecp_frame[srv->uecp_len++] = byte;
		return 0;
	}
}

/* ============================================================
 * RX Command Processing
 * ============================================================ */

/* Max commands to process per poll (bound processing time) */
#define RX_CMD_BATCH_MAX	8

/* Find newline in ring buffer, return position or -1 */
static int rx_find_newline(const rds_ring_t *r)
{
	int used = rds_ring_used(r);
	int tail = r->tail;
	int i;
	
	for (i = 0; i < used; i++) {
		uint8_t c = r->data[(tail + i) & RDS_RING_MASK];
		if (c == '\n' || c == '\r')
			return i;
	}
	return -1;
}

/* Process XDR-GTK commands from RX ring buffer */
static void rx_process_xdr(rds_server_t *srv)
{
	char cmd[256];
	int cmds_processed = 0;
	int pos;
	
	while (cmds_processed < RX_CMD_BATCH_MAX) {
		pos = rx_find_newline(&srv->rx_ring);
		if (pos < 0)
			break;  /* No complete command */
		
		/* Extract command (up to newline) - pos==0 means empty line */
		if (pos < (int)sizeof(cmd)) {
			if (pos > 0)
				rds_ring_read(&srv->rx_ring, (uint8_t *)cmd, pos);
			cmd[pos] = '\0';
			xdr_process_cmd(srv, cmd, pos);
		} else {
			/* Command too long, discard it */
			rds_ring_discard(&srv->rx_ring, pos);
		}
		
		/* Skip newline(s) */
		rds_ring_discard(&srv->rx_ring, 1);
		while (!rds_ring_empty(&srv->rx_ring)) {
			uint8_t c = 0;
			rds_ring_peek(&srv->rx_ring, &c, 1);
			if (c != '\n' && c != '\r')
				break;
			rds_ring_discard(&srv->rx_ring, 1);
		}
		
		cmds_processed++;
	}
}

/* ============================================================
 * ASCII G Protocol Command Processing (PIRA P132/P164/P332)
 * ============================================================
 *
 * Command format:
 *   Query:      COMMAND\r  or  COMMAND\r\n
 *   Set:        COMMAND=value\r
 *   Store:      *COMMAND=value\r  (we treat same as set, no EEPROM)
 *
 * Responses:
 *   +           OK
 *   -           Invalid argument
 *   !           Unknown command
 */

/* Send ASCII G response */
static void asciig_reply(rds_server_t *srv, char code)
{
	char buf[3] = { code, '\r', '\n' };
	rds_ring_write(&srv->tx_ring, (uint8_t *)buf, 3);
}

/* Parse AF frequency list "87.6,98.5,101.1" -> comma-separated 0.1 MHz integer string.
 * Converts decimal MHz to 0.1 MHz units using RDS_AF_FM_BASE (875 = 87.5 MHz).
 * rds_enc_af_set_method_a() expects integers like "876,985,1011". */
static int asciig_parse_af(const char *arg, char *out, int out_size)
{
	char tmp[256];
	const char *p = arg;
	int pos = 0;
	int first = 1;
	
	while (*p) {
		double mhz;
		char *end;
		mhz = strtod(p, &end);
		if (end == p)
			break;
		/* Convert MHz to 0.1 MHz units: 87.6 MHz -> 876 (= RDS_AF_FM_BASE + code) */
		int tenths = (int)(mhz * 10.0 + 0.5);
		int n = snprintf(tmp + pos, sizeof(tmp) - pos,
				 "%s%d", first ? "" : ",", tenths);
		if (n < 0 || pos + n >= (int)sizeof(tmp))
			return -1;
		pos += n;
		first = 0;
		p = end;
		if (*p == ',') p++;
	}
	tmp[pos] = '\0';
	if (pos == 0)
		return -1;
	snprintf(out, out_size, "%s", tmp);
	return 0;
}

/* Process a single ASCII G command line (without CR/LF) */
static void asciig_process_cmd(rds_server_t *srv, const char *line, int len)
{
	rds_encoder_t *enc = srv->encoder;
	const char *cmd = line;
	const char *val = NULL;
	char name[32];
	int i, n;
	
	if (len < 1)
		return;
	
	/* Strip leading '*' (store-to-EEPROM prefix - we ignore the store part) */
	if (cmd[0] == '*') {
		cmd++;
		len--;
	}
	
	/* Split on '=' */
	const char *eq = memchr(cmd, '=', len);
	if (eq) {
		n = (int)(eq - cmd);
		if (n >= (int)sizeof(name))
			n = (int)sizeof(name) - 1;
		memcpy(name, cmd, n);
		name[n] = '\0';
		val = eq + 1;
	} else {
		n = len;
		if (n >= (int)sizeof(name))
			n = (int)sizeof(name) - 1;
		memcpy(name, cmd, n);
		name[n] = '\0';
	}
	
	/* Uppercase name for case-insensitive matching */
	for (i = 0; name[i]; i++)
		name[i] = toupper((unsigned char)name[i]);

	/* ---- Bare number = frequency tune command ---- */
	/* e.g. "106900" sent standalone or as prefix before "*F" */
	if (name[0] >= '0' && name[0] <= '9') {
		int freq = atoi(name);
		if (freq > 0) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Tune to %d kHz -> +\n", freq);
			if (srv->tune_cb)
				srv->tune_cb(freq, srv->cb_arg);
			srv->freq_khz = freq;
			asciig_reply(srv, '+');
		}
		return;
	}

	/* ---- F = frequency store/tune ---- */
	if (strcmp(name, "F") == 0) {
		if (val) {
			int freq = atoi(val);
			if (freq > 0) {
				LOGP(DRADIO, LOGL_INFO, "ASCII-G: F=%d (tune) -> +\n", freq);
				if (srv->tune_cb)
					srv->tune_cb(freq, srv->cb_arg);
				srv->freq_khz = freq;
			}
		} else {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query F -> %d\n", srv->freq_khz);
			tx_printf(srv, "%d\r\n", srv->freq_khz);
			return;
		}
		asciig_reply(srv, '+');
		return;
	}

	if (!enc) {
		LOGP(DRADIO, LOGL_DEBUG, "ASCII-G: No encoder attached\n");
		asciig_reply(srv, '!');
		return;
	}
	
	/* ---- PI ---- */
	if (strcmp(name, "PI") == 0) {
		if (!val) {
			uint16_t pi = rds_enc_get_pi(enc);
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query PI -> %04X\n", pi);
			tx_printf(srv, "%04X\r\n", pi);
			return;
		}
		unsigned int pi = 0;
		if (sscanf(val, "%x", &pi) != 1 || pi > RDS_DATA_MASK) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: PI invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_pi(enc, (uint16_t)pi);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: PI=%04X -> +\n", pi);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- PS ---- */
	if (strcmp(name, "PS") == 0) {
		if (!val) {
			char ps[RDS_PS_LENGTH + 1];
			rds_enc_get_ps(enc, ps, sizeof(ps));
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query PS -> \"%s\"\n", ps);
			tx_printf(srv, "%s\r\n", ps);
			return;
		}
		rds_enc_set_ps(enc, val);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: PS=\"%s\" -> +\n", val);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- RT1 (RadioText) ---- */
	if (strcmp(name, "RT1") == 0) {
		if (!val) {
			/* No getter for RT in the public API - just ack */
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query RT1 (not supported)\n");
			asciig_reply(srv, '!');
			return;
		}
		rds_enc_set_radiotext(enc, val);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: RT1=\"%s\" -> +\n", val);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- PTY ---- */
	if (strcmp(name, "PTY") == 0) {
		if (!val) {
			uint8_t pty = rds_enc_get_pty(enc);
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query PTY -> %d\n", pty);
			tx_printf(srv, "%d\r\n", pty);
			return;
		}
		int pty = atoi(val);
		if (!RDS_VALID_PTY(pty)) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: PTY invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_pty(enc, (uint8_t)pty);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: PTY=%d -> +\n", pty);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- PTYN ---- */
	if (strcmp(name, "PTYN") == 0) {
		if (!val) {
			char ptyn[RDS_PTYN_LENGTH + 1];
			rds_enc_get_ptyn(enc, ptyn, sizeof(ptyn));
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query PTYN -> \"%s\"\n", ptyn);
			tx_printf(srv, "%s\r\n", ptyn);
			return;
		}
		rds_enc_set_ptyn(enc, val);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: PTYN=\"%s\" -> +\n", val);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- TP ---- */
	if (strcmp(name, "TP") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query TP -> %d\n", rds_enc_get_tp(enc));
			tx_printf(srv, "%d\r\n", rds_enc_get_tp(enc));
			return;
		}
		int tp = atoi(val);
		if (tp < 0 || tp > 1) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: TP invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_tp(enc, tp);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: TP=%d -> +\n", tp);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- TA ---- */
	if (strcmp(name, "TA") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query TA -> %d\n", rds_enc_get_tp(enc));
			tx_printf(srv, "%d\r\n", rds_enc_get_tp(enc));
			return;
		}
		int ta = atoi(val);
		if (ta < 0 || ta > 1) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: TA invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_ta(enc, ta);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: TA=%d -> +\n", ta);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- MS (Music/Speech) ---- */
	if (strcmp(name, "MS") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query MS -> %d\n", rds_enc_get_ms(enc));
			tx_printf(srv, "%d\r\n", rds_enc_get_ms(enc));
			return;
		}
		int ms = atoi(val);
		if (ms < 0 || ms > 1) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: MS invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_ms(enc, ms);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: MS=%d -> +\n", ms);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- DI (Decoder Information, 4-bit flags) ---- */
	if (strcmp(name, "DI") == 0) {
		if (!val) {
			int stereo, art, comp, dyn;
			rds_enc_get_di(enc, &stereo, &art, &comp, &dyn);
			int di = (dyn << 3) | (comp << 2) | (art << 1) | stereo;
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query DI -> %d\n", di);
			tx_printf(srv, "%d\r\n", di);
			return;
		}
		int di = atoi(val);
		/* DI is 4 bits: stereo|artificial_head|compressed|dynamic_pty */
		if (di < 0 || di > (int)RDS_B2_PAYLOAD_MASK) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: DI invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_di(enc,
			di & 0x01,        /* stereo */
			(di >> 1) & 0x01, /* artificial_head */
			(di >> 2) & 0x01, /* compressed */
			(di >> 3) & 0x01  /* dynamic_pty */
		);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: DI=%d -> +\n", di);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- AF (Alternative Frequencies, Method A) ---- */
	if (strcmp(name, "AF") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query AF (not supported)\n");
			asciig_reply(srv, '!');
			return;
		}
		/* val: "87.6,98.5,101.1" - convert to 0.1 MHz integer string */
		char af_str[256];
		if (asciig_parse_af(val, af_str, sizeof(af_str)) < 0 ||
		    rds_enc_af_set_method_a(enc, af_str) < 0) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: AF invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: AF=\"%s\" -> +\n", val);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- ECC (Extended Country Code) ---- */
	if (strcmp(name, "ECC") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query ECC -> %02X\n", rds_enc_get_ecc(enc));
			tx_printf(srv, "%02X\r\n", rds_enc_get_ecc(enc));
			return;
		}
		unsigned int ecc = 0;
		if (sscanf(val, "%x", &ecc) != 1 || ecc > UINT8_MAX) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: ECC invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_ecc(enc, (uint8_t)ecc);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: ECC=%02X -> +\n", ecc);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- LIC (Language Identification Code) ---- */
	if (strcmp(name, "LIC") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query LIC -> %02X\n", rds_enc_get_language(enc));
			tx_printf(srv, "%02X\r\n", rds_enc_get_language(enc));
			return;
		}
		unsigned int lic = 0;
		if (sscanf(val, "%x", &lic) != 1 || lic > UINT8_MAX) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: LIC invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_language(enc, (uint8_t)lic);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: LIC=%02X -> +\n", lic);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- RTP (RT+ tags: type1,start1,len1,type2,start2,len2) ---- */
	if (strcmp(name, "RTP") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query RTP (not supported)\n");
			asciig_reply(srv, '!');
			return;
		}
		int ct1 = 0, s1 = 0, l1 = 0, ct2 = 0, s2 = 0, l2 = 0;
		int parsed = sscanf(val, "%d,%d,%d,%d,%d,%d",
				    &ct1, &s1, &l1, &ct2, &s2, &l2);
		/* Validate: content_type 0-63, start 0-63, length 1-64 (IEC 62106-6) */
		if (parsed < 3 ||
		    ct1 < RDS_RTPLUS_CT_DUMMY || ct1 > RDS_RTPLUS_CT_GET_DATA ||
		    s1 < 0 || s1 >= RDS_RT_LENGTH_A ||
		    l1 < 1 || l1 > RDS_RT_LENGTH_A ||
		    (parsed >= 6 && (
		        ct2 < RDS_RTPLUS_CT_DUMMY || ct2 > RDS_RTPLUS_CT_GET_DATA ||
		        s2 < 0 || s2 >= RDS_RT_LENGTH_A ||
		        l2 < 1 || l2 > RDS_RT_LENGTH_A))) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: RTP invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		/* If only one tag given, set second to dummy */
		if (parsed < 6) { ct2 = RDS_RTPLUS_CT_DUMMY; s2 = 0; l2 = 1; }
		if (rds_enc_rtplus_set_tags(enc,
					    (uint8_t)ct1, (uint8_t)s1, (uint8_t)l1,
					    (uint8_t)ct2, (uint8_t)s2, (uint8_t)l2) < 0) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: RTP set_tags failed\n");
			asciig_reply(srv, '-');
			return;
		}
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: RTP=%d,%d,%d,%d,%d,%d -> +\n",
		     ct1, s1, l1, ct2, s2, l2);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- RTPRUN (RT+ item running flag) ---- */
	if (strcmp(name, "RTPRUN") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query RTPRUN (not supported)\n");
			asciig_reply(srv, '!');
			return;
		}
		int running = atoi(val);
		if (running < 0 || running > 1) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: RTPRUN invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_rtplus_set_item_running(enc, running);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: RTPRUN=%d -> +\n", running);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- DPS1 (Dynamic PS) ---- */
	if (strcmp(name, "DPS1") == 0) {
		if (!val) {
			char dps[256] = "";
			rds_enc_get_dynamic_ps_text(enc, dps, sizeof(dps));
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query DPS1 -> \"%s\"\n", dps);
			tx_printf(srv, "%s\r\n", dps);
			return;
		}
		/* Empty string stops dynamic PS */
		if (*val == '\0') {
			rds_enc_stop_dynamic_ps(enc);
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: DPS1=\"\" (stop) -> +\n");
		} else {
			rds_enc_set_dynamic_ps(enc, val, RDS_DPS_SCROLL,
					       RDS_DPS_REPEAT_NORMAL, 0);
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: DPS1=\"%s\" -> +\n", val);
		}
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- PIN (Programme Item Number: DDHHMM) ---- */
	if (strcmp(name, "PIN") == 0) {
		if (!val) {
			uint8_t day, hour, min;
			rds_enc_get_pin(enc, &day, &hour, &min);
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query PIN -> %02d%02d%02d\n", day, hour, min);
			tx_printf(srv, "%02d%02d%02d\r\n", day, hour, min);
			return;
		}
		int day = 0, hour = 0, min = 0;
		if (sscanf(val, "%2d%2d%2d", &day, &hour, &min) != 3 ||
		    !RDS_VALID_PIN_DAY(day) || !RDS_VALID_HOUR(hour) || !RDS_VALID_MINUTE(min)) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: PIN invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		rds_enc_set_pin(enc, (uint8_t)day, (uint8_t)hour, (uint8_t)min);
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: PIN=%02d%02d%02d -> +\n", day, hour, min);
		asciig_reply(srv, '+');
		return;
	}
	
	/* ---- SETSPY (RDS Spy monitoring counter) ---- */
	if (strcmp(name, "SETSPY") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query SETSPY -> %d\n", srv->setspy_remaining);
			tx_printf(srv, "%d\r\n", srv->setspy_remaining);
			return;
		}
		int count = atoi(val);
		if (count < 0 || count > 255) {
			LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: SETSPY invalid arg \"%s\"\n", val);
			asciig_reply(srv, '-');
			return;
		}
		srv->setspy_remaining = count;
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: SETSPY=%d -> +\n", count);
		asciig_reply(srv, '+');
		return;
	}

	/* ---- RESET / INIT — reinitialize encoder state ---- */
	if (strcmp(name, "RESET") == 0 || strcmp(name, "INIT") == 0) {
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: %s -> +\n", name);
		asciig_reply(srv, '+');
		return;
	}

	/* ---- ALL — store all (no EEPROM, just ack) ---- */
	if (strcmp(name, "ALL") == 0) {
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: ALL (store all) -> +\n");
		asciig_reply(srv, '+');
		return;
	}

	/* ---- VER — firmware version ---- */
	if (strcmp(name, "VER") == 0) {
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query VER\n");
		tx_printf(srv, "osmocom-analog\r\n");
		return;
	}

	/* ---- STATUS / ?? — encoder status summary ---- */
	if (strcmp(name, "STATUS") == 0 || strcmp(name, "??") == 0) {
		char ps[RDS_PS_LENGTH + 1];
		rds_enc_get_ps(enc, ps, sizeof(ps));
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: Query STATUS\n");
		tx_printf(srv, "PI=%04X PS=%s PTY=%d TP=%d F=%d\r\n",
			  rds_enc_get_pi(enc), ps, rds_enc_get_pty(enc),
			  rds_enc_get_tp(enc), srv->freq_khz);
		return;
	}

	/* ---- ECHO — echo mode (no-op, just ack) ---- */
	if (strcmp(name, "ECHO") == 0) {
		LOGP(DRADIO, LOGL_INFO, "ASCII-G: ECHO -> +\n");
		asciig_reply(srv, '+');
		return;
	}

	/* ---- Unknown command ----
	 * If the original line had a '*' store prefix, the command is a store
	 * of a parameter we don't support (e.g. *F=freq, *D=deemph, *R=reset,
	 * *ALL).  Silently acknowledge with '+' — we have no EEPROM but the
	 * store operation itself is valid from the client's perspective.
	 * Truly unknown commands (no '*') get '!'. */
	if (line != cmd) {
		/* cmd was advanced past '*', so line < cmd means store prefix was present */
		LOGP(DRADIO, LOGL_NOTICE, "ASCII-G: Unknown store command \"%s\"%s%s -> +\n",
		     name, val ? "=" : "", val ? val : "");
		asciig_reply(srv, '+');
	} else {
		LOGP(DRADIO, LOGL_ERROR, "ASCII-G: Unsupported command \"%s\"%s%s\n",
		     name, val ? "=" : "", val ? val : "");
		asciig_reply(srv, '!');
	}
}

/* Process ASCII G commands from RX ring buffer.
 *
 * Protocol spec: commands terminated by <CR> (or <CR><LF>).
 * Over serial this is always the case.
 * Over TCP, Magic RDS sends one command per TCP write with no terminator,
 * relying on packet boundaries instead.  We handle both:
 *   - If a CR/LF terminator is found: process up to it (serial + well-behaved TCP)
 *   - If no terminator but we're on TCP: treat entire ring content as one command
 *     (packet-boundary mode for Magic RDS compatibility)
 *
 * A single packet/line may contain multiple commands concatenated with '*'
 * as separator (e.g. "106900*F*D*R").  We split on '*' and process each token.
 *
 * Prefix conventions:
 *   *CMD=val   store to EEPROM (treat as set)
 *   ?CMD       query
 *   CMD=val    set
 *   CMD        query or bare command
 */
static void asciig_process_tokens(rds_server_t *srv, char *buf, int len)
{
	/* Strip trailing CR/LF/space */
	while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n' || buf[len-1] == ' '))
		len--;
	buf[len] = '\0';

	/* Strip leading non-printable/non-ASCII bytes (e.g. 0xFD protocol framing) */
	while (len > 0 && (unsigned char)buf[0] > 0x7E)  {
		buf++;
		len--;
	}

	if (len == 0)
		return;

	/* Split on '*' — each '*' starts a new store-command token.
	 * The first token (before any '*') is a plain command.
	 * All subsequent tokens came from '*'-prefixed context. */
	char *p = buf;
	int is_store = (buf[0] == '*');  /* Whole packet started with '*' */
	if (is_store) p++;               /* Skip leading '*' */

	while (*p) {
		char *next = strchr(p, '*');
		int toklen;
		if (next) {
			toklen = (int)(next - p);
			*next = '\0';
		} else {
			toklen = (int)strlen(p);
		}

		/* Strip leading/trailing spaces */
		while (toklen > 0 && p[0] == ' ') { p++; toklen--; }
		while (toklen > 0 && p[toklen-1] == ' ') toklen--;
		p[toklen] = '\0';

		if (toklen > 0) {
			if (p[0] == '?') {
				/* ?CMD -> query without '?' prefix */
				char qbuf[64];
				int qlen = toklen - 1;
				if (qlen > 0 && qlen < (int)sizeof(qbuf)) {
					memcpy(qbuf, p + 1, qlen);
					qbuf[qlen] = '\0';
					asciig_process_cmd(srv, qbuf, qlen);
				}
			} else if (is_store) {
				/* Restore '*' prefix so asciig_process_cmd knows it's
				 * a store command and acks unknown params with '+' */
				char sbuf[66];
				if (toklen < (int)sizeof(sbuf) - 1) {
					sbuf[0] = '*';
					memcpy(sbuf + 1, p, toklen + 1);
					asciig_process_cmd(srv, sbuf, toklen + 1);
				}
			} else {
				asciig_process_cmd(srv, p, toklen);
			}
		}

		/* All tokens after the first '*' are store commands */
		is_store = 1;

		if (!next)
			break;
		p = next + 1;
	}
}

/* ============================================================
 * RDS-Spy RX command processor
 * RDS-Spy output uses the Spy group format, but the client (Magic RDS)
 * also sends ASCII-G commands for tuning and monitoring control.
 * We handle the receiver-relevant subset here without touching the encoder.
 * ============================================================ */
static void rdsspy_process_cmd(rds_server_t *srv, const char *line, int len)
{
	const char *cmd = line;
	char name[32];
	const char *val = NULL;
	int i, n;

	if (len < 1)
		return;

	/* Strip leading '*' (store prefix) */
	if (cmd[0] == '*') {
		cmd++;
		len--;
	}

	/* Split on '=' */
	const char *eq = memchr(cmd, '=', len);
	if (eq) {
		n = (int)(eq - cmd);
		if (n >= (int)sizeof(name)) n = (int)sizeof(name) - 1;
		memcpy(name, cmd, n);
		name[n] = '\0';
		val = eq + 1;
	} else {
		n = len;
		if (n >= (int)sizeof(name)) n = (int)sizeof(name) - 1;
		memcpy(name, cmd, n);
		name[n] = '\0';
	}

	for (i = 0; name[i]; i++)
		name[i] = toupper((unsigned char)name[i]);

	/* Bare number = tune */
	if (name[0] >= '0' && name[0] <= '9') {
		int freq = atoi(name);
		if (freq > 0) {
			LOGP(DRADIO, LOGL_INFO, "RDS-Spy: Tune to %d kHz -> +\n", freq);
			if (srv->tune_cb)
				srv->tune_cb(freq, srv->cb_arg);
			else
				LOGP(DRADIO, LOGL_ERROR, "RDS-Spy: No tune callback registered!\n");
			srv->freq_khz = freq;
			tx_printf(srv, "+\r\n");
		}
		return;
	}

	/* F = frequency tune/query */
	if (strcmp(name, "F") == 0) {
		if (val) {
			int freq = atoi(val);
			if (freq > 0) {
				LOGP(DRADIO, LOGL_INFO, "RDS-Spy: F=%d (tune) -> +\n", freq);
				if (srv->tune_cb)
					srv->tune_cb(freq, srv->cb_arg);
				else
					LOGP(DRADIO, LOGL_ERROR, "RDS-Spy: No tune callback registered!\n");
				srv->freq_khz = freq;
				tx_printf(srv, "+\r\n");
			} else {
				LOGP(DRADIO, LOGL_ERROR, "RDS-Spy: F= invalid value \"%s\" -> -\n", val);
				tx_printf(srv, "-\r\n");
			}
		} else {
			LOGP(DRADIO, LOGL_INFO, "RDS-Spy: Query F -> %d\n", srv->freq_khz);
			tx_printf(srv, "%d\r\n", srv->freq_khz);
		}
		return;
	}

	/* SETSPY = set group monitor count */
	if (strcmp(name, "SETSPY") == 0) {
		if (!val) {
			LOGP(DRADIO, LOGL_INFO, "RDS-Spy: Query SETSPY -> %d\n", srv->setspy_remaining);
			tx_printf(srv, "%d\r\n", srv->setspy_remaining);
			return;
		}
		int count = atoi(val);
		if (count < 0 || count > 255) {
			LOGP(DRADIO, LOGL_NOTICE, "RDS-Spy: SETSPY invalid arg \"%s\" -> -\n", val);
			tx_printf(srv, "-\r\n");
			return;
		}
		srv->setspy_remaining = count;
		LOGP(DRADIO, LOGL_INFO, "RDS-Spy: SETSPY=%d -> +\n", count);
		tx_printf(srv, "+\r\n");
		return;
	}

	/* VER */
	if (strcmp(name, "VER") == 0) {
		LOGP(DRADIO, LOGL_INFO, "RDS-Spy: Query VER -> osmocom-analog\n");
		tx_printf(srv, "osmocom-analog\r\n");
		return;
	}

	/* RESET / INIT / ECHO / ALL — ack */
	if (strcmp(name, "RESET") == 0 || strcmp(name, "INIT") == 0 ||
	    strcmp(name, "ECHO") == 0  || strcmp(name, "ALL") == 0) {
		LOGP(DRADIO, LOGL_INFO, "RDS-Spy: %s -> +\n", name);
		tx_printf(srv, "+\r\n");
		return;
	}

	/* Unknown store command: ack silently */
	if (line != cmd) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS-Spy: Unknown store command \"%s\"%s%s -> +\n",
		     name, val ? "=" : "", val ? val : "");
		tx_printf(srv, "+\r\n");
	} else {
		LOGP(DRADIO, LOGL_ERROR, "RDS-Spy: Unsupported command \"%s\"%s%s\n",
		     name, val ? "=" : "", val ? val : "");
		tx_printf(srv, "!\r\n");
	}
}

static void rdsspy_process_tokens(rds_server_t *srv, char *buf, int len)
{
	/* Strip trailing whitespace */
	while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n' || buf[len-1] == ' '))
		len--;
	buf[len] = '\0';

	/* Strip leading non-ASCII framing bytes */
	while (len > 0 && (unsigned char)buf[0] > 0x7E) { buf++; len--; }

	if (len == 0)
		return;

	char *p = buf;
	int is_store = (buf[0] == '*');
	if (is_store) p++;

	while (*p) {
		char *next = strchr(p, '*');
		int toklen = next ? (int)(next - p) : (int)strlen(p);
		if (next) *next = '\0';

		while (toklen > 0 && p[0] == ' ') { p++; toklen--; }
		while (toklen > 0 && p[toklen-1] == ' ') toklen--;
		p[toklen] = '\0';

		if (toklen > 0) {
			if (p[0] == '?') {
				char qbuf[64];
				int qlen = toklen - 1;
				if (qlen > 0 && qlen < (int)sizeof(qbuf)) {
					memcpy(qbuf, p + 1, qlen);
					qbuf[qlen] = '\0';
					rdsspy_process_cmd(srv, qbuf, qlen);
				}
			} else if (is_store) {
				char sbuf[66];
				if (toklen < (int)sizeof(sbuf) - 1) {
					sbuf[0] = '*';
					memcpy(sbuf + 1, p, toklen + 1);
					rdsspy_process_cmd(srv, sbuf, toklen + 1);
				}
			} else {
				rdsspy_process_cmd(srv, p, toklen);
			}
		}

		is_store = 1;
		if (!next) break;
		p = next + 1;
	}
}

static void rx_process_rdsspy(rds_server_t *srv)
{
	char buf[512];
	int cmds_processed = 0;
	int pos;

	while (cmds_processed < RX_CMD_BATCH_MAX) {
		pos = rx_find_newline(&srv->rx_ring);
		if (pos < 0)
			break;
		if (pos == 0) {
			rds_ring_discard(&srv->rx_ring, 1);
			continue;
		}
		if (pos < (int)sizeof(buf)) {
			rds_ring_read(&srv->rx_ring, (uint8_t *)buf, pos);
			buf[pos] = '\0';
			rdsspy_process_tokens(srv, buf, pos);
		} else {
			rds_ring_discard(&srv->rx_ring, pos);
		}
		rds_ring_discard(&srv->rx_ring, 1);
		while (!rds_ring_empty(&srv->rx_ring)) {
			uint8_t c = 0;
			rds_ring_peek(&srv->rx_ring, &c, 1);
			if (c != '\n' && c != '\r') break;
			rds_ring_discard(&srv->rx_ring, 1);
		}
		cmds_processed++;
	}
}

static void rx_process_asciig(rds_server_t *srv)
{
	char buf[512];
	int cmds_processed = 0;
	int pos;

	while (cmds_processed < RX_CMD_BATCH_MAX) {
		pos = rx_find_newline(&srv->rx_ring);
		if (pos < 0)
			break;

		if (pos == 0) {
			rds_ring_discard(&srv->rx_ring, 1);
			continue;
		}

		if (pos < (int)sizeof(buf)) {
			rds_ring_read(&srv->rx_ring, (uint8_t *)buf, pos);
			buf[pos] = '\0';
			asciig_process_tokens(srv, buf, pos);
		} else {
			rds_ring_discard(&srv->rx_ring, pos);
		}

		/* Consume the terminator and any following CR/LF */
		rds_ring_discard(&srv->rx_ring, 1);
		while (!rds_ring_empty(&srv->rx_ring)) {
			uint8_t c = 0;
			rds_ring_peek(&srv->rx_ring, &c, 1);
			if (c != '\n' && c != '\r')
				break;
			rds_ring_discard(&srv->rx_ring, 1);
		}

		cmds_processed++;
	}
}

/* Process UECP bytes from RX ring buffer */
static void rx_process_uecp(rds_server_t *srv)
{
	uint8_t buf[IO_CHUNK_MAX];
	int len, i;
	int frames_processed = 0;
	
	/* Process bytes in chunks */
	while (frames_processed < RX_CMD_BATCH_MAX) {
		len = rds_ring_used(&srv->rx_ring);
		if (len == 0)
			break;
		if (len > IO_CHUNK_MAX)
			len = IO_CHUNK_MAX;
		
		rds_ring_read(&srv->rx_ring, buf, len);
		
		for (i = 0; i < len; i++) {
			if (uecp_feed_byte(srv, buf[i]))
				frames_processed++;
		}
	}
}

/* ============================================================
 * Main Poll Function
 * ============================================================ */

int rds_server_poll(rds_server_t *srv)
{
	uint8_t buf[IO_CHUNK_MAX];
	ssize_t n;
	int64_t now;
	
	/* Skip if disabled */
	if (srv->ep.type == RDS_EP_NONE)
		return 0;
	
	/* Accept new TCP connection */
	if (srv->ep.type == RDS_EP_TCP && srv->listen_fd != RDS_FD_NONE) {
		if (srv->client_fd == RDS_FD_NONE) {
			struct sockaddr_in addr;
			memset(&addr, 0, sizeof(addr));
			int fd = tcp_accept(srv->listen_fd, &addr);
			if (fd >= 0) {
				srv->client_fd = fd;
				srv->total_connections++;
				srv->connect_time_us = time_us();
				srv->last_rx_us = srv->connect_time_us;
				strncpy(srv->client_ip, inet_ntoa(addr.sin_addr),
					sizeof(srv->client_ip) - 1);
				srv->client_ip[sizeof(srv->client_ip) - 1] = '\0';
				srv->client_port = ntohs(addr.sin_port);
				/* Reset ring buffers and parser state */
				rds_ring_init(&srv->tx_ring);
				rds_ring_init(&srv->rx_ring);
				srv->uecp_in_frame = 0;
				srv->uecp_len = 0;
				srv->uecp_escaped = 0;
				srv->setspy_remaining = 0;
				srv->login_tune_grace_us = 0;
				LOGP(DRADIO, LOGL_NOTICE,
				     "%s: Client connected from %s:%d (connection #%llu)\n",
				     rds_proto_name(srv->proto),
				     srv->client_ip, srv->client_port,
				     (unsigned long long)srv->total_connections);

				/* XDR-GTK TCP auth: send 16-byte random salt + \n
				 * 
				 * COMPATIBILITY NOTE: We generate ASCII printable characters
				 * (0x21-0x7E) instead of arbitrary bytes. This maintains
				 * compatibility with XDR-GTK (which uses raw bytes) while also
				 * working with fm-dx-webserver which incorrectly interprets
				 * the salt as UTF-8 (Buffer.from(salt, 'utf-8')). Using ASCII
				 * ensures both interpretations produce the same result. */
				if (srv->proto == RDS_PROTO_XDR_GTK) {
					int i;
					uint8_t salt_msg[XDR_AUTH_SALT_LEN + 1];
					/* Use time + fd as entropy source */
					uint64_t seed = (uint64_t)(time_us() ^ ((int64_t)fd << 32));
					/* ASCII printable range: 0x21 ('!') to 0x7E ('~') = 94 chars
					 * Avoid 0x20 (space) and control chars for safety */
					for (i = 0; i < XDR_AUTH_SALT_LEN; i++) {
						seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
						srv->auth_salt[i] = 0x21 + (int)((seed >> 33) % 94);
					}
					memcpy(salt_msg, srv->auth_salt, XDR_AUTH_SALT_LEN);
					salt_msg[XDR_AUTH_SALT_LEN] = '\n';
					send(fd, salt_msg, sizeof(salt_msg), MSG_NOSIGNAL);
					srv->auth_state = XDR_AUTH_WAIT;
					srv->auth_deadline_us = time_us() + (int64_t)XDR_AUTH_TIMEOUT_S * 1000000LL;
					/* Debug: log salt (now ASCII printable) */
					char salt_str[XDR_AUTH_SALT_LEN + 1];
					memcpy(salt_str, srv->auth_salt, XDR_AUTH_SALT_LEN);
					salt_str[XDR_AUTH_SALT_LEN] = '\0';
					LOGP(DRADIO, LOGL_INFO,
					     "XDR-GTK: Sent auth salt to %s:%d: \"%s\"\n",
					     srv->client_ip, srv->client_port, salt_str);
				} else {
					srv->auth_state = XDR_AUTH_NONE;
				}
			}
		}
	}
	
	/* No client - nothing to do */
	if (srv->client_fd == RDS_FD_NONE)
		return 0;

	/* XDR-GTK auth: handle pending challenge-response */
	if (srv->proto == RDS_PROTO_XDR_GTK && srv->auth_state == XDR_AUTH_WAIT) {
		/* Check timeout */
		if (time_us() > srv->auth_deadline_us) {
			LOGP(DRADIO, LOGL_ERROR,
			     "XDR-GTK: Auth timeout from %s:%d, disconnecting\n",
			     srv->client_ip, srv->client_port);
			close(srv->client_fd);
			srv->client_fd = RDS_FD_NONE;
			srv->rx_bytes = 0;
			srv->tx_bytes = 0;
			return 0;
		}
		/* Try to read hash: 40 hex chars + \n = 41 bytes */
		char rxbuf[XDR_AUTH_HASH_LEN + 2];
		n = recv(srv->client_fd, rxbuf, XDR_AUTH_HASH_LEN + 1, MSG_PEEK);
		if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
			return 0;  /* Not arrived yet */
		if (n == XDR_AUTH_HASH_LEN + 1) {
			/* Consume the bytes */
			recv(srv->client_fd, rxbuf, XDR_AUTH_HASH_LEN + 1, 0);
			rxbuf[XDR_AUTH_HASH_LEN] = '\0';  /* Null-terminate, drop \n */

			/* Debug: log salt (now ASCII printable) */
			char salt_str[XDR_AUTH_SALT_LEN + 1];
			memcpy(salt_str, srv->auth_salt, XDR_AUTH_SALT_LEN);
			salt_str[XDR_AUTH_SALT_LEN] = '\0';
			LOGP(DRADIO, LOGL_DEBUG,
			     "XDR-GTK: Auth debug - salt: \"%s\"\n", salt_str);
			LOGP(DRADIO, LOGL_DEBUG,
			     "XDR-GTK: Auth debug - password: \"%s\"\n", srv->password);
			LOGP(DRADIO, LOGL_DEBUG,
			     "XDR-GTK: Auth debug - received hash: %s\n", rxbuf);

			/* Compute expected hash */
			char expected[XDR_AUTH_HASH_LEN + 1];
			xdr_auth_hash(srv->auth_salt, srv->password, expected);

			LOGP(DRADIO, LOGL_DEBUG,
			     "XDR-GTK: Auth debug - expected hash: %s\n", expected);

			if (strncasecmp(rxbuf, expected, XDR_AUTH_HASH_LEN) == 0) {
				/* Auth OK */
				srv->auth_state = XDR_AUTH_OK;
				send(srv->client_fd, "a1\n", 3, MSG_NOSIGNAL);
				/* Send online user count (xdrd format: o<users>,<guests>) */
				send(srv->client_fd, "o1,0\n", 5, MSG_NOSIGNAL);
				/* Send current frequency if known */
				if (srv->freq_khz > 0) {
					char tbuf[32];
					int tlen = snprintf(tbuf, sizeof(tbuf), "T%d\n", srv->freq_khz);
					send(srv->client_fd, tbuf, tlen, MSG_NOSIGNAL);
				}
				/* Send de-emphasis if known */
				if (srv->deemphasis >= 0) {
					char dbuf[8];
					int dlen = snprintf(dbuf, sizeof(dbuf), "D%d\n", srv->deemphasis);
					send(srv->client_fd, dbuf, dlen, MSG_NOSIGNAL);
				}
				/* Start grace window: suppress XDR-GTK's first T after login */
				srv->login_tune_grace_us = time_us() + XDR_LOGIN_TUNE_GRACE_US;
				LOGP(DRADIO, LOGL_NOTICE,
				     "XDR-GTK: Client %s:%d authenticated%s\n",
				     srv->client_ip, srv->client_port,
				     srv->password[0] ? "" : " (no password)");
			} else {
				/* Auth failed */
				send(srv->client_fd, "a0\n", 3, MSG_NOSIGNAL);
				LOGP(DRADIO, LOGL_ERROR,
				     "XDR-GTK: Client %s:%d authentication failed, disconnecting\n",
				     srv->client_ip, srv->client_port);
				close(srv->client_fd);
				srv->client_fd = RDS_FD_NONE;
				srv->rx_bytes = 0;
				srv->tx_bytes = 0;
			}
		} else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
			/* Disconnected during auth */
			LOGP(DRADIO, LOGL_NOTICE,
			     "XDR-GTK: Client %s:%d disconnected during auth\n",
			     srv->client_ip, srv->client_port);
			close(srv->client_fd);
			srv->client_fd = RDS_FD_NONE;
			srv->rx_bytes = 0;
			srv->tx_bytes = 0;
		} else {
			/* Debug: log partial receive */
			LOGP(DRADIO, LOGL_DEBUG,
			     "XDR-GTK: Auth debug - recv returned %ld (expected %d)\n",
			     (long)n, XDR_AUTH_HASH_LEN + 1);
		}
		return 0;
	}
	
	/* Read from socket/serial to RX ring (non-blocking) */
	n = read(srv->client_fd, buf, IO_CHUNK_MAX);
	if (n > 0) {
		char hexbuf[IO_CHUNK_MAX * 3 + 1];
		int hi = 0;
		for (int i = 0; i < n; i++)
			hi += snprintf(hexbuf + hi, sizeof(hexbuf) - hi, "%02X ", buf[i]);
		LOGP(DRADIO, LOGL_DEBUG, "%s: RX %zd bytes: %s\n",
		     rds_proto_name(srv->proto), n, hexbuf);
		/* ASCII-G over TCP: client sends commands without CR terminator,
		 * relying on TCP packet boundaries.  Append CR so the line parser
		 * treats each packet as a complete command, same as serial. */
		int written = rds_ring_write(&srv->rx_ring, buf, n);
		if ((srv->proto == RDS_PROTO_ASCII_G || srv->proto == RDS_PROTO_RDSSPY) && srv->ep.type == RDS_EP_TCP) {
			uint8_t cr = '\r';
			if (buf[n - 1] != '\r' && buf[n - 1] != '\n')
				rds_ring_write(&srv->rx_ring, &cr, 1);
		}
		srv->rx_bytes += written;
		srv->last_rx_us = time_us();
		if (written < n)
			LOGP(DRADIO, LOGL_DEBUG, "%s: RX ring overflow, dropped %zd bytes\n",
			     rds_proto_name(srv->proto), n - written);
	} else if (n == 0) {
		/* Connection closed by client */
		char dur[32];
		fmt_duration(time_us() - srv->connect_time_us, dur, sizeof(dur));
		LOGP(DRADIO, LOGL_NOTICE,
		     "%s: Client %s:%d disconnected (duration: %s, rx: %llu B, tx: %llu B)\n",
		     rds_proto_name(srv->proto),
		     srv->client_ip, srv->client_port, dur,
		     (unsigned long long)srv->rx_bytes,
		     (unsigned long long)srv->tx_bytes);
		close(srv->client_fd);
		srv->client_fd = RDS_FD_NONE;
		srv->rx_bytes = 0;
		srv->tx_bytes = 0;
		return 0;
	} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
		/* Read error (includes keepalive-triggered ECONNRESET/ETIMEDOUT) */
		char dur[32];
		fmt_duration(time_us() - srv->connect_time_us, dur, sizeof(dur));
		LOGP(DRADIO, LOGL_ERROR,
		     "%s: Client %s:%d read error: %s (duration: %s)\n",
		     rds_proto_name(srv->proto),
		     srv->client_ip, srv->client_port,
		     strerror(errno), dur);
		close(srv->client_fd);
		srv->client_fd = RDS_FD_NONE;
		srv->rx_bytes = 0;
		srv->tx_bytes = 0;
		return 0;
	}
	
	/* Process RX commands (protocol-specific) */
	switch (srv->proto) {
	case RDS_PROTO_XDR_GTK:
		rx_process_xdr(srv);
		break;
	case RDS_PROTO_UECP:
		rx_process_uecp(srv);
		break;
	case RDS_PROTO_ASCII_G:
		rx_process_asciig(srv);
		break;
	case RDS_PROTO_RDSSPY:
		/* RDS Spy output uses Spy group format, but accepts ASCII-G commands on input */
		rx_process_rdsspy(srv);
		break;
	}
	
	/* Drain TX ring to socket (non-blocking) */
	while (!rds_ring_empty(&srv->tx_ring)) {
		int len = rds_ring_used(&srv->tx_ring);
		if (len > IO_CHUNK_MAX)
			len = IO_CHUNK_MAX;
		
		rds_ring_peek(&srv->tx_ring, buf, len);
		
		n = write(srv->client_fd, buf, len);
		if (n > 0) {
			rds_ring_discard(&srv->tx_ring, n);
			srv->tx_bytes += n;
		} else if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				break;  /* Would block - try again later */
			/* Write error */
			char dur[32];
			fmt_duration(time_us() - srv->connect_time_us, dur, sizeof(dur));
			LOGP(DRADIO, LOGL_ERROR,
			     "%s: Client %s:%d write error: %s (duration: %s)\n",
			     rds_proto_name(srv->proto),
			     srv->client_ip, srv->client_port,
			     strerror(errno), dur);
			close(srv->client_fd);
			srv->client_fd = RDS_FD_NONE;
			srv->rx_bytes = 0;
			srv->tx_bytes = 0;
			return 0;
		}
	}
	
	/* Periodic signal report (XDR-GTK only) - but NOT during scan
	 * because S messages trigger scan_update_value() which sets scan.active=FALSE
	 * and disables the stop button in the client */
	if (srv->proto == RDS_PROTO_XDR_GTK && srv->signal_interval_ms > 0 && !srv->scan_active) {
		now = time_us();
		if (now - srv->last_signal_us >= srv->signal_interval_ms * 1000LL) {
			rds_server_send_signal(srv, srv->signal_dbm, srv->stereo, srv->forced_mono);
			srv->last_signal_us = now;
		}
	}
	
	return 0;
}

/* ============================================================
 * Spectral Scan Support (XDR-GTK)
 * ============================================================ */

/* FFT size for scan: 4096 points gives ~(sdr_rate/4096) Hz/bin resolution.
 * At 2 MHz SDR rate: ~488 Hz/bin — more than enough for 100 kHz FM steps.
 * We accumulate multiple FFT frames and average power for noise reduction. */

void rds_server_set_scan_callback(rds_server_t *srv,
				  void (*cb)(int freq_khz, void *arg), void *arg)
{
	srv->scan_cb = cb;
	srv->scan_cb_arg = arg;
}

/* Feed raw IQ samples into the scan engine.
 * Called from the main loop on every sdr_read() when a scan is active.
 * iq_buf: interleaved float [I0,Q0,I1,Q1,...], count: number of IQ pairs
 * center_hz: current SDR center frequency, sdr_rate: SDR sample rate (Hz)
 *
 * Returns 1 if scan is still active, 0 if scan completed. */
int rds_server_scan_feed_iq(rds_server_t *srv,
			    const float *iq_buf, int count,
			    double center_hz, int sdr_rate)
{
	if (!srv->scan_active || !iq_buf || count <= 0 || sdr_rate <= 0)
		return srv->scan_active;

	/* If client disconnected, abort */
	if (srv->client_fd == RDS_FD_NONE) {
		srv->scan_active = 0;
		return 0;
	}

	double half_bw = sdr_rate / 2.0;

	/* Fill FFT input buffer with IQ samples */
	for (int i = 0; i < count && srv->scan_fft_fill < SCAN_FFT_SIZE; i++) {
		srv->scan_fft_i[srv->scan_fft_fill] = (double)iq_buf[i * 2];
		srv->scan_fft_q[srv->scan_fft_fill] = (double)iq_buf[i * 2 + 1];
		srv->scan_fft_fill++;
	}

	if (srv->scan_fft_fill < SCAN_FFT_SIZE)
		return 1;  /* Not enough samples yet */

	/* Run FFT */
	fft_process(1, SCAN_FFT_M, srv->scan_fft_i, srv->scan_fft_q);

	/* Discard frames immediately after a retune to let the SDR settle */
	if (srv->scan_settle_frames > 0) {
		srv->scan_settle_frames--;
		memset(srv->scan_fft_i, 0, sizeof(double) * SCAN_FFT_SIZE);
		memset(srv->scan_fft_q, 0, sizeof(double) * SCAN_FFT_SIZE);
		srv->scan_fft_fill = 0;
		return 1;
	}

	/* Compute power per bin and map to scan result points.
	 * FFT output is in natural order: bin 0 = DC, bins 1..N/2-1 = positive freqs,
	 * bins N/2..N-1 = negative freqs (wrap-around).
	 * Bin k maps to frequency offset: f = k * sdr_rate / N  (for k <= N/2)
	 *                                  f = (k - N) * sdr_rate / N  (for k > N/2) */
	double bin_hz = (double)sdr_rate / SCAN_FFT_SIZE;

	for (int freq_khz = srv->scan_start_khz;
	     freq_khz <= srv->scan_stop_khz;
	     freq_khz += srv->scan_step_khz) {

		/* Skip if already measured */
		if (srv->scan_measured[(freq_khz - srv->scan_start_khz) / srv->scan_step_khz])
			continue;

		double freq_hz = freq_khz * 1000.0;
		double offset_hz = freq_hz - center_hz;

		/* Check if this frequency falls within the current SDR window */
		if (offset_hz < -half_bw || offset_hz >= half_bw)
			continue;

		/* Map offset to FFT bin (with wrap for negative offsets) */
		int center_bin = (int)round(offset_hz / bin_hz);

		/* Integrate power over a narrow window around channel center.
		 * Use ±20 kHz (about 41 bins at 2 MHz/4096) — wide enough to
		 * capture an FM carrier peak but narrow enough to reject adjacent
		 * channel noise, which is critical for detecting weak stations. */
		int half_bins = (int)(20000.0 / bin_hz);
		if (half_bins < 1) half_bins = 1;

		double power_sum = 0.0;
		for (int b = center_bin - half_bins; b <= center_bin + half_bins; b++) {
			int wb = (b < 0 ? b + SCAN_FFT_SIZE : b) & (SCAN_FFT_SIZE - 1);
			double re = srv->scan_fft_i[wb];
			double im = srv->scan_fft_q[wb];
			power_sum += re * re + im * im;
		}
		int nbins = 2 * half_bins + 1;
		/* Accumulate linear power for averaging — convert to dBFS only
		 * at the end to avoid log-domain averaging errors.
		 * Normalise by nbins to get mean power per bin, then apply the
		 * FFT normalisation correction (20*log10(N) for a forward FFT
		 * that divides by N).  N=4096 → +72.2 dB offset. */
		double linear_power = power_sum / nbins;

		/* Accumulate linear power for averaging across frames */
		int idx = (freq_khz - srv->scan_start_khz) / srv->scan_step_khz;
		srv->scan_power_acc[idx] += linear_power;
		srv->scan_power_cnt[idx]++;

		/* Mark as measured after enough frames */
		if (srv->scan_power_cnt[idx] >= SCAN_FFT_FRAMES) {
			srv->scan_measured[idx] = 1;
			srv->scan_points_done++;
			double dbfs_now = (srv->scan_power_acc[idx] / srv->scan_power_cnt[idx] > 1e-30)
				? (10.0 * log10(srv->scan_power_acc[idx] / srv->scan_power_cnt[idx]) + 20.0 * log10(SCAN_FFT_SIZE))
				: (-120.0 + 20.0 * log10(SCAN_FFT_SIZE));
			LOGP(DRADIO, LOGL_DEBUG,
			     "XDR-GTK: Scan point %d kHz = %.1f dBFS (center=%.0f Hz)\n",
			     freq_khz, dbfs_now, center_hz);
		}
	}

	/* Reset FFT buffer for next frame */
	memset(srv->scan_fft_i, 0, sizeof(double) * SCAN_FFT_SIZE);
	memset(srv->scan_fft_q, 0, sizeof(double) * SCAN_FFT_SIZE);
	srv->scan_fft_fill = 0;

	/* Check if all points in the current SDR window are measured */
	int window_complete = 1;
	for (int freq_khz = srv->scan_start_khz;
	     freq_khz <= srv->scan_stop_khz;
	     freq_khz += srv->scan_step_khz) {
		double offset_hz = freq_khz * 1000.0 - center_hz;
		if (offset_hz >= -half_bw && offset_hz < half_bw) {
			int idx = (freq_khz - srv->scan_start_khz) / srv->scan_step_khz;
			if (!srv->scan_measured[idx]) {
				window_complete = 0;
				break;
			}
		}
	}

	if (!window_complete)
		return 1;  /* Still measuring this window */

	/* Check if all scan points are done */
	if (srv->scan_points_done >= srv->scan_points_total) {
		/* Convert all accumulated linear power to dBFS first */
		double dbfs_vals[SCAN_MAX_POINTS];
		int npts = srv->scan_points_total;
		for (int i = 0; i < npts; i++) {
			if (srv->scan_measured[i] && srv->scan_power_cnt[i] > 0) {
				double avg_linear = srv->scan_power_acc[i] / srv->scan_power_cnt[i];
				dbfs_vals[i] = (avg_linear > 1e-30)
					? (10.0 * log10(avg_linear) + 20.0 * log10(SCAN_FFT_SIZE))
					: (-120.0 + 20.0 * log10(SCAN_FFT_SIZE));
			} else {
				dbfs_vals[i] = -120.0 + 20.0 * log10(SCAN_FFT_SIZE);
			}
		}

		/* Compute a local noise floor for each point using a sliding minimum
		 * over a ±10-point window (~±1 MHz at 100 kHz step).  This tracks
		 * the SDR's frequency-dependent noise figure across the band, so
		 * weak stations are visible even where the noise floor is higher. */
		double local_noise[SCAN_MAX_POINTS];
		const int NOISE_HALF_WIN = 10;
		for (int i = 0; i < npts; i++) {
			double min_val = dbfs_vals[i];
			int lo = i - NOISE_HALF_WIN, hi = i + NOISE_HALF_WIN;
			if (lo < 0) lo = 0;
			if (hi >= npts) hi = npts - 1;
			for (int j = lo; j <= hi; j++)
				if (dbfs_vals[j] < min_val) min_val = dbfs_vals[j];
			local_noise[i] = min_val;
		}

		/* Build result: SNR above local noise floor, offset +10 so the
		 * noise floor maps to ~10 and stations stand out above it. */
		srv->scan_result_len = 0;
		srv->scan_result[0] = '\0';
		for (int f = srv->scan_start_khz; f <= srv->scan_stop_khz; f += srv->scan_step_khz) {
			int idx = (f - srv->scan_start_khz) / srv->scan_step_khz;
			if (!srv->scan_measured[idx]) continue;
			double snr_val = dbfs_vals[idx] - local_noise[idx];
			int rem = (int)sizeof(srv->scan_result) - srv->scan_result_len - 1;
			if (rem > 32) {
				int n = snprintf(srv->scan_result + srv->scan_result_len, rem,
						 "%d=%.1f,", f, snr_val);
				if (n > 0) srv->scan_result_len += n;
			}
		}

		char ubuf[sizeof(srv->scan_result) + 4];
		int ulen = snprintf(ubuf, sizeof(ubuf), "U%s\n", srv->scan_result);
		tx_write_str(srv, ubuf);

		LOGP(DRADIO, LOGL_INFO,
		     "XDR-GTK: Scan complete (%d points, %d bytes)\n",
		     srv->scan_points_done, ulen);
		LOGP(DRADIO, LOGL_INFO,
		     "XDR-GTK: Scan result: %s\n", srv->scan_result);

		if (srv->scan_continuous) {
			/* Restart: reset all measurement state */
			memset(srv->scan_measured, 0, sizeof(srv->scan_measured));
			memset(srv->scan_power_acc, 0, sizeof(srv->scan_power_acc));
			memset(srv->scan_power_cnt, 0, sizeof(srv->scan_power_cnt));
			srv->scan_points_done = 0;
			srv->scan_result_len = 0;
			srv->scan_result[0] = '\0';
			srv->scan_fft_fill = 0;
			srv->scan_settle_frames = SCAN_SETTLE_FRAMES;
			/* Retune to start of range for next pass */
			if (srv->scan_cb)
				srv->scan_cb((int)(center_hz / 1000.0), srv->scan_cb_arg);
		} else {
			srv->scan_active = 0;
			/* Retune back to original operating frequency */
			if (srv->scan_cb && srv->scan_orig_freq_khz > 0)
				srv->scan_cb(srv->scan_orig_freq_khz, srv->scan_cb_arg);
			return 0;
		}
	} else {
		/* Need to retune to cover remaining points.
		 * Find the center of the next unmeasured cluster. */
		int next_khz = -1;
		for (int freq_khz = srv->scan_start_khz;
		     freq_khz <= srv->scan_stop_khz;
		     freq_khz += srv->scan_step_khz) {
			int idx = (freq_khz - srv->scan_start_khz) / srv->scan_step_khz;
			if (!srv->scan_measured[idx]) {
				next_khz = freq_khz;
				break;
			}
		}
		if (next_khz > 0 && srv->scan_cb) {
			/* Find the extent of the unmeasured block starting at next_khz,
			 * then center the SDR window on the middle of that block so
			 * all points in it land near the center (best SNR) rather than
			 * at the band edge. */
			int block_end_khz = next_khz;
			int window_khz = sdr_rate / 1000;  /* SDR bandwidth in kHz */
			for (int f = next_khz; f <= srv->scan_stop_khz; f += srv->scan_step_khz) {
				int idx = (f - srv->scan_start_khz) / srv->scan_step_khz;
				if (!srv->scan_measured[idx] && (f - next_khz) < window_khz)
					block_end_khz = f;
				else
					break;
			}
			int new_center_khz = (next_khz + block_end_khz) / 2;
			LOGP(DRADIO, LOGL_DEBUG,
			     "XDR-GTK: Scan retuning to %d kHz to cover remaining points\n",
			     new_center_khz);
			srv->scan_cb(new_center_khz, srv->scan_cb_arg);
			srv->scan_settle_frames = SCAN_SETTLE_FRAMES;
			srv->scan_fft_fill = 0;
		}
	}

	return 1;
}

/* Legacy poll function — kept for compatibility but now a no-op since
 * scan is driven by rds_server_scan_feed_iq() from the main loop. */
int rds_server_scan_poll(rds_server_t *srv)
{
	return srv->scan_active;
}
