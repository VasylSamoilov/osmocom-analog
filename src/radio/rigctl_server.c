/* Rigctl Server - Hamlib-compatible remote control protocol
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * TESTING: Use "rigctl -m 2 -r localhost:PORT" to connect.
 *          The -m 2 selects NET rigctl model (required for remote connection).
 *          Without -m 2, rigctl uses the Dummy model which ignores -r.
 *          See rigctl_server.h for full documentation.
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <termios.h>

#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "rigctl_server.h"
#include "radio.h"
#include "signal_meter.h"

#define DEFAULT_PORT		7356
#define VERSION_STRING		"osmoradio 1.0"

/* Logging category */
#define DRIGCTL DRADIO

/* Time helper */
static int64_t time_us(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

/* ============================================================
 * Endpoint Parsing
 * ============================================================ */

int rigctl_ep_parse(const char *str, rigctl_endpoint_t *ep)
{
	char buf[512];
	char *p, *tok;

	if (!str || !ep)
		return -1;

	memset(ep, 0, sizeof(*ep));
	strncpy(buf, str, sizeof(buf) - 1);

	/* Check for serial: contains comma */
	if (strchr(buf, ',')) {
		ep->type = RIGCTL_EP_SERIAL;

		/* device,speed,8N1[,flow] */
		tok = strtok(buf, ",");
		if (!tok) return -1;
		strncpy(ep->device, tok, sizeof(ep->device) - 1);

		tok = strtok(NULL, ",");
		if (!tok) return -1;
		ep->speed = atoi(tok);

		tok = strtok(NULL, ",");
		if (!tok) return -1;
		/* Parse 8N1 format */
		if (strlen(tok) >= 3) {
			ep->bits = tok[0] - '0';
			ep->parity = tok[1];
			ep->stopbits = tok[2] - '0';
		} else {
			ep->bits = 8;
			ep->parity = 'N';
			ep->stopbits = 1;
		}

		tok = strtok(NULL, ",");
		if (tok) {
			if (strcasecmp(tok, "xonxoff") == 0)
				ep->flow = RIGCTL_FLOW_XONXOFF;
			else if (strcasecmp(tok, "rtscts") == 0)
				ep->flow = RIGCTL_FLOW_RTSCTS;
		}
		return 0;
	}

	/* TCP: ip:port or just port */
	ep->type = RIGCTL_EP_TCP;
	p = strchr(buf, ':');
	if (p) {
		*p = '\0';
		/* Copy IP, ensuring null termination */
		memset(ep->ip, 0, sizeof(ep->ip));
		memcpy(ep->ip, buf, sizeof(ep->ip) - 1);
		ep->port = atoi(p + 1);
	} else {
		/* Just port number */
		memcpy(ep->ip, "0.0.0.0", 8);
		ep->port = atoi(buf);
	}

	if (ep->port <= 0)
		ep->port = DEFAULT_PORT;

	return 0;
}

/* ============================================================
 * Serial Port Setup
 * ============================================================ */

static int serial_open(const rigctl_endpoint_t *ep)
{
	int fd;
	struct termios tio;
	speed_t speed;

	fd = open(ep->device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		LOGP(DRIGCTL, LOGL_ERROR, "Failed to open %s: %s\n",
		     ep->device, strerror(errno));
		return -1;
	}

	memset(&tio, 0, sizeof(tio));

	/* Speed */
	switch (ep->speed) {
	case 9600:   speed = B9600; break;
	case 19200:  speed = B19200; break;
	case 38400:  speed = B38400; break;
	case 57600:  speed = B57600; break;
	case 115200: speed = B115200; break;
	default:     speed = B9600; break;
	}
	cfsetispeed(&tio, speed);
	cfsetospeed(&tio, speed);

	/* 8N1 default */
	tio.c_cflag |= CS8 | CLOCAL | CREAD;

	/* Parity */
	if (ep->parity == 'E')
		tio.c_cflag |= PARENB;
	else if (ep->parity == 'O')
		tio.c_cflag |= PARENB | PARODD;

	/* Stop bits */
	if (ep->stopbits == 2)
		tio.c_cflag |= CSTOPB;

	/* Flow control */
	if (ep->flow == RIGCTL_FLOW_RTSCTS)
		tio.c_cflag |= CRTSCTS;
	else if (ep->flow == RIGCTL_FLOW_XONXOFF)
		tio.c_iflag |= IXON | IXOFF;

	/* Raw mode */
	tio.c_lflag = 0;
	tio.c_oflag = 0;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;

	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &tio);

	LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl serial opened: %s @ %d\n",
	     ep->device, ep->speed);

	return fd;
}

/* ============================================================
 * TCP Server Setup
 * ============================================================ */

static int tcp_listen(const rigctl_endpoint_t *ep)
{
	int fd, rc, opt = 1;
	struct sockaddr_in addr;

	fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
	if (fd < 0) {
		LOGP(DRIGCTL, LOGL_ERROR, "socket() failed: %s\n", strerror(errno));
		return -1;
	}

	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(ep->port);
	if (ep->ip[0])
		inet_pton(AF_INET, ep->ip, &addr.sin_addr);
	else
		addr.sin_addr.s_addr = INADDR_ANY;

	rc = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
	if (rc < 0) {
		LOGP(DRIGCTL, LOGL_ERROR, "bind() failed on port %d: %s\n",
		     ep->port, strerror(errno));
		close(fd);
		return -1;
	}

	rc = listen(fd, 4);
	if (rc < 0) {
		LOGP(DRIGCTL, LOGL_ERROR, "listen() failed: %s\n", strerror(errno));
		close(fd);
		return -1;
	}

	LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl TCP server listening on %s:%d (fd=%d)\n",
	     ep->ip[0] ? ep->ip : "0.0.0.0", ep->port, fd);

	return fd;
}

static int is_host_allowed(const rigctl_server_t *srv, const char *ip)
{
	char buf[512];
	char *tok, *saveptr;

	if (!srv->allowed_hosts[0])
		return 1;  /* No restriction */

	snprintf(buf, sizeof(buf), "%s", srv->allowed_hosts);
	tok = strtok_r(buf, ",", &saveptr);
	while (tok) {
		while (*tok == ' ') tok++;
		if (strcmp(tok, ip) == 0)
			return 1;
		tok = strtok_r(NULL, ",", &saveptr);
	}
	return 0;
}

/* ============================================================
 * Client Management
 * ============================================================ */

static void client_init(rigctl_client_t *c)
{
	memset(c, 0, sizeof(*c));
	c->fd = RIGCTL_FD_NONE;
	rigctl_ring_init(&c->tx_ring);
	rigctl_ring_init(&c->rx_ring);
}

static void client_close(rigctl_client_t *c)
{
	if (c->fd != RIGCTL_FD_NONE) {
		close(c->fd);
		c->fd = RIGCTL_FD_NONE;
	}
}

static int tx_write(rigctl_client_t *c, const char *data, int len)
{
	return rigctl_ring_write(&c->tx_ring, (const uint8_t *)data, len);
}

static int tx_write_str(rigctl_client_t *c, const char *str)
{
	return tx_write(c, str, strlen(str));
}

static int tx_printf(rigctl_client_t *c, const char *fmt, ...)
{
	char buf[512];
	va_list ap;
	int len;

	va_start(ap, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (len > 0)
		return tx_write(c, buf, len);
	return 0;
}

/* ============================================================
 * Mode String Conversion
 * ============================================================ */

/* Convert internal mode to Hamlib mode string.
 * For FM, distinguishes between narrowband FM and wideband FM based on bandwidth.
 * Wideband FM (>50kHz) returns WFM. Stereo is indicated separately via flags.
 * Note: Hamlib doesn't have a standard WFM_ST mode - use WFM for both mono/stereo. */
static const char *mode_to_str(int mode, int stereo, int bandwidth)
{
	(void)stereo;  /* Stereo is not part of Hamlib mode string */

	switch (mode) {
	case MODULATION_NONE:   return "OFF";
	case MODULATION_FM:
		/* Distinguish narrowband FM from wideband FM based on bandwidth
		 * Typical narrowband FM: 10-15 kHz
		 * Typical wideband FM: 150-200 kHz
		 * Use 50 kHz as threshold */
		if (bandwidth > 50000)
			return "WFM";
		return "FM";
	case MODULATION_AM_DSB: return "AM";
	case MODULATION_AM_USB: return "USB";
	case MODULATION_AM_LSB: return "LSB";
	default:                return "ERR";
	}
}

static int str_to_mode(const char *str, int *stereo)
{
	*stereo = 0;

	if (strcasecmp(str, "OFF") == 0)
		return MODULATION_NONE;
	if (strcasecmp(str, "FM") == 0)
		return MODULATION_FM;
	if (strcasecmp(str, "WFM") == 0)
		return MODULATION_FM;
	if (strcasecmp(str, "WFM_ST") == 0) {
		*stereo = 1;
		return MODULATION_FM;
	}
	if (strcasecmp(str, "WFM_ST_OIRT") == 0) {
		*stereo = 1;
		return MODULATION_FM;
	}
	if (strcasecmp(str, "AM") == 0)
		return MODULATION_AM_DSB;

	/* TODO: AMS (AM Synchronous) mode
	 * Currently mapped to regular AM as placeholder.
	 * Full implementation requires:
	 * - PLL-based carrier recovery for sync demodulation
	 * - Selectable sideband (USB/LSB/DSB) after sync detection
	 * - Carrier lock indicator
	 * - Adjustable PLL bandwidth
	 * See: radio.c for demodulator implementation
	 */
	if (strcasecmp(str, "AMS") == 0)
		return MODULATION_AM_DSB;  /* TODO: implement sync AM */

	if (strcasecmp(str, "USB") == 0)
		return MODULATION_AM_USB;
	if (strcasecmp(str, "LSB") == 0)
		return MODULATION_AM_LSB;

	/* TODO: CW modes (CW, CWU, CWL, CWR)
	 * Currently mapped to USB/LSB as placeholders.
	 * Full implementation requires:
	 * - BFO (Beat Frequency Oscillator) offset (~700Hz typical)
	 * - Narrow CW filter (200-500Hz bandwidth)
	 * - CW sidetone generator for TX
	 * - Optional: CW decoder, keyer interface
	 * CWU = CW upper sideband (BFO below signal)
	 * CWL/CWR = CW lower sideband (BFO above signal)
	 * See: radio.c for demodulator implementation
	 */
	if (strcasecmp(str, "CW") == 0 || strcasecmp(str, "CWU") == 0)
		return MODULATION_AM_USB;  /* TODO: implement CW with BFO */
	if (strcasecmp(str, "CWL") == 0 || strcasecmp(str, "CWR") == 0)
		return MODULATION_AM_LSB;  /* TODO: implement CW with BFO */

	return -1;
}

/* ============================================================
 * Frequency Validation Helper
 * ============================================================ */

/* Validate frequency against SDR capabilities.
 * Returns 0 if valid, -1 if out of range.
 * Applies upconverter offset if configured.
 * user_freq: frequency as seen by user (with upconverter offset)
 * sdr_freq: actual SDR frequency (without upconverter offset) */
static int validate_freq(rigctl_server_t *srv, double user_freq, int is_tx, double *sdr_freq)
{
	double min_freq, max_freq;
	double upconv;

	/* Get upconverter offset for this direction */
	upconv = is_tx ? srv->sdr_caps.tx_upconverter : srv->sdr_caps.rx_upconverter;

	/* Convert user frequency to SDR frequency
	 * User sees: 7.1 MHz, upconverter adds 125 MHz -> SDR tunes 132.1 MHz
	 * So: sdr_freq = user_freq + upconverter */
	*sdr_freq = user_freq + upconv;

	/* Get limits based on direction */
	if (is_tx) {
		min_freq = srv->sdr_caps.tx_freq_min;
		max_freq = srv->sdr_caps.tx_freq_max;
	} else {
		min_freq = srv->sdr_caps.rx_freq_min;
		max_freq = srv->sdr_caps.rx_freq_max;
	}

	/* Check limits (0 means unknown/unlimited) */
	if (min_freq > 0 && *sdr_freq < min_freq) {
		LOGP(DRIGCTL, LOGL_INFO, "Frequency %.0f Hz (SDR: %.0f Hz) below minimum %.0f Hz\n",
		     user_freq, *sdr_freq, min_freq);
		return -1;
	}
	if (max_freq > 0 && *sdr_freq > max_freq) {
		LOGP(DRIGCTL, LOGL_INFO, "Frequency %.0f Hz (SDR: %.0f Hz) above maximum %.0f Hz\n",
		     user_freq, *sdr_freq, max_freq);
		return -1;
	}

	return 0;
}

/* Convert SDR frequency to user-visible frequency (add upconverter offset) */
static double sdr_to_user_freq(rigctl_server_t *srv, double sdr_freq, int is_tx)
{
	double upconv = is_tx ? srv->sdr_caps.tx_upconverter : srv->sdr_caps.rx_upconverter;
	/* User sees frequency WITHOUT upconverter offset
	 * SDR tunes 132.1 MHz, upconverter is 125 MHz -> User sees 7.1 MHz
	 * So: user_freq = sdr_freq - upconverter */
	return sdr_freq - upconv;
}

/* ============================================================
 * Command Handlers
 * ============================================================ */

/* f - Get frequency
 * Returns RX frequency if RX enabled, TX frequency if TX-only.
 * For split mode, returns the primary (RX) frequency.
 * Frequency is returned as user-visible (with upconverter offset applied). */
static void cmd_get_freq(rigctl_server_t *srv, rigctl_client_t *c)
{
	double freq;

	/* Return RX freq if RX enabled, otherwise TX freq */
	if (srv->rx_enabled) {
		freq = sdr_to_user_freq(srv, srv->rx_freq, 0);
	} else {
		freq = sdr_to_user_freq(srv, srv->tx_freq, 1);
	}

	tx_printf(c, "%.0f\n", freq);
}

/* F <freq> - Set frequency
 * Sets frequency for both TX and RX if both enabled (simplex mode).
 * In split mode, only sets RX frequency (use I command for TX).
 * Frequency is expected as user-visible (upconverter offset will be applied). */
static void cmd_set_freq(rigctl_server_t *srv, rigctl_client_t *c, const char *arg)
{
	double user_freq, sdr_freq;
	int rc_rx = 0, rc_tx = 0;

	if (!arg || !*arg) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	user_freq = atof(arg);
	if (user_freq <= 0) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	/* Set RX frequency if RX enabled */
	if (srv->rx_enabled) {
		if (validate_freq(srv, user_freq, 0, &sdr_freq) < 0) {
			tx_write_str(c, "RPRT 1\n");
			return;
		}
		if (srv->cb.set_rx_freq)
			rc_rx = srv->cb.set_rx_freq(sdr_freq, srv->cb.arg);
		if (rc_rx == 0)
			srv->rx_freq = sdr_freq;
	}

	/* Set TX frequency if TX enabled and not in split mode */
	if (srv->tx_enabled && !srv->split) {
		if (validate_freq(srv, user_freq, 1, &sdr_freq) < 0) {
			/* TX freq out of range is not fatal if RX succeeded */
			if (!srv->rx_enabled) {
				tx_write_str(c, "RPRT 1\n");
				return;
			}
		} else {
			if (srv->cb.set_tx_freq)
				rc_tx = srv->cb.set_tx_freq(sdr_freq, srv->cb.arg);
			if (rc_tx == 0)
				srv->tx_freq = sdr_freq;
		}
	}

	/* Return success if at least one direction succeeded */
	if ((srv->rx_enabled && rc_rx == 0) || (srv->tx_enabled && rc_tx == 0))
		tx_write_str(c, "RPRT 0\n");
	else
		tx_write_str(c, "RPRT 1\n");
}

/* m - Get mode */
static void cmd_get_mode(rigctl_server_t *srv, rigctl_client_t *c)
{
	tx_printf(c, "%s\n%d\n", mode_to_str(srv->mode, srv->stereo, srv->bandwidth), srv->bandwidth);
}

/* M <mode> [bw] - Set mode */
static void cmd_set_mode(rigctl_server_t *srv, rigctl_client_t *c,
			 const char *arg1, const char *arg2)
{
	int mode, stereo, bw = 0;

	if (!arg1 || !*arg1) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	/* Query supported modes */
	if (strcmp(arg1, "?") == 0) {
		tx_write_str(c, "OFF FM WFM WFM_ST AM AMS USB LSB CW CWU CWL CWR\n");
		return;
	}

	mode = str_to_mode(arg1, &stereo);
	if (mode < 0) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	if (arg2 && *arg2)
		bw = atoi(arg2);

	/* Mode change requires restart - just acknowledge for now */
	srv->mode = mode;
	srv->stereo = stereo;
	if (bw > 0)
		srv->bandwidth = bw;

	tx_write_str(c, "RPRT 0\n");
}

/* l - Get level */
static void cmd_get_level(rigctl_server_t *srv, rigctl_client_t *c, const char *arg)
{
	if (!arg || !*arg || strcmp(arg, "?") == 0) {
		tx_write_str(c, "SQL STRENGTH AF");
		if (srv->tx_enabled)
			tx_write_str(c, " TX_RMS TX_PEAK");
		if (srv->mode == MODULATION_FM)
			tx_write_str(c, " DEVIATION");
		else
			tx_write_str(c, " MOD_INDEX");
		tx_write_str(c, "\n");
		return;
	}

	if (strcasecmp(arg, "STRENGTH") == 0) {
		tx_printf(c, "%.1f\n", srv->signal_level);
	} else if (strcasecmp(arg, "SQL") == 0) {
		tx_printf(c, "%.1f\n", srv->squelch_level);
	} else if (strcasecmp(arg, "AF") == 0) {
		tx_printf(c, "%.1f\n", srv->audio_gain);
	} else if (strcasecmp(arg, "TX_RMS") == 0) {
		tx_printf(c, "%.1f\n", srv->tx_rms);
	} else if (strcasecmp(arg, "TX_PEAK") == 0) {
		double peak = (srv->tx_peak_i > srv->tx_peak_q) ? srv->tx_peak_i : srv->tx_peak_q;
		double peak_db = (peak > 0) ? 20.0 * log10(peak) : -100.0;
		tx_printf(c, "%.1f\n", peak_db);
	} else if (strcasecmp(arg, "DEVIATION") == 0) {
		tx_printf(c, "%.0f\n", srv->deviation);
	} else if (strcasecmp(arg, "MOD_INDEX") == 0) {
		tx_printf(c, "%.2f\n", srv->mod_index);
	} else {
		/* Check for hardware gain */
		/* TODO: implement hardware gain lookup */
		tx_write_str(c, "RPRT 1\n");
	}
}

/* L - Set level */
static void cmd_set_level(rigctl_server_t *srv, rigctl_client_t *c,
			  const char *arg1, const char *arg2)
{
	double val;

	if (!arg1 || !*arg1 || strcmp(arg1, "?") == 0) {
		tx_write_str(c, "SQL AF");
		if (srv->mode == MODULATION_FM)
			tx_write_str(c, " DEVIATION");
		else
			tx_write_str(c, " MOD_INDEX");
		tx_write_str(c, "\n");
		return;
	}

	if (!arg2 || !*arg2) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	val = atof(arg2);

	if (strcasecmp(arg1, "SQL") == 0) {
		if (val < -150) val = -150;
		if (val > 0) val = 0;
		srv->squelch_level = val;
		tx_write_str(c, "RPRT 0\n");
	} else if (strcasecmp(arg1, "AF") == 0) {
		if (val < -80) val = -80;
		if (val > 50) val = 50;
		srv->audio_gain = val;
		tx_write_str(c, "RPRT 0\n");
	} else if (strcasecmp(arg1, "DEVIATION") == 0) {
		srv->deviation = val;
		tx_write_str(c, "RPRT 0\n");
	} else if (strcasecmp(arg1, "MOD_INDEX") == 0) {
		if (val < 0) val = 0;
		if (val > 1) val = 1;
		srv->mod_index = val;
		tx_write_str(c, "RPRT 0\n");
	} else {
		tx_write_str(c, "RPRT 1\n");
	}
}

/* u - Get function */
static void cmd_get_func(rigctl_server_t *srv, rigctl_client_t *c, const char *arg)
{
	if (!arg || !*arg || strcmp(arg, "?") == 0) {
		tx_write_str(c, "DSP RECORD RDS STEREO\n");
		return;
	}

	if (strcasecmp(arg, "DSP") == 0) {
		tx_printf(c, "%d\n", srv->dsp_running);
	} else if (strcasecmp(arg, "RECORD") == 0) {
		int rec = 0;
		if (srv->cb.is_recording)
			rec = srv->cb.is_recording(srv->cb.arg);
		tx_printf(c, "%d\n", rec);
	} else if (strcasecmp(arg, "RDS") == 0) {
		tx_printf(c, "%d\n", srv->rds_enabled);
	} else if (strcasecmp(arg, "STEREO") == 0) {
		tx_printf(c, "%d\n", srv->stereo);
	} else {
		tx_write_str(c, "RPRT 1\n");
	}
}

/* U - Set function */
static void cmd_set_func(rigctl_server_t *srv, rigctl_client_t *c,
			 const char *arg1, const char *arg2)
{
	int val;

	if (!arg1 || !*arg1 || strcmp(arg1, "?") == 0) {
		tx_write_str(c, "DSP RECORD RDS\n");
		return;
	}

	if (!arg2 || !*arg2) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	val = atoi(arg2);

	if (strcasecmp(arg1, "DSP") == 0) {
		srv->dsp_running = val ? 1 : 0;
		tx_write_str(c, "RPRT 0\n");
	} else if (strcasecmp(arg1, "RECORD") == 0) {
		int rc = -1;
		if (val && srv->cb.start_recording)
			rc = srv->cb.start_recording(srv->cb.arg);
		else if (!val && srv->cb.stop_recording)
			rc = srv->cb.stop_recording(srv->cb.arg);
		tx_printf(c, "RPRT %d\n", rc < 0 ? 1 : 0);
	} else if (strcasecmp(arg1, "RDS") == 0) {
		srv->rds_enabled = val ? 1 : 0;
		tx_write_str(c, "RPRT 0\n");
	} else {
		tx_write_str(c, "RPRT 1\n");
	}
}

/* p - Get parameter (RDS) */
static void cmd_get_param(rigctl_server_t *srv, rigctl_client_t *c, const char *arg)
{
	if (!arg || !*arg || strcmp(arg, "?") == 0) {
		tx_write_str(c, "RDS_PI RDS_PS_NAME RDS_RADIOTEXT\n");
		return;
	}

	if (strcasecmp(arg, "RDS_PI") == 0) {
		tx_printf(c, "%04X\n", srv->rds_pi);
	} else if (strcasecmp(arg, "RDS_PS_NAME") == 0) {
		tx_printf(c, "%s\n", srv->rds_ps);
	} else if (strcasecmp(arg, "RDS_RADIOTEXT") == 0) {
		tx_printf(c, "%s\n", srv->rds_rt);
	} else {
		tx_write_str(c, "RPRT 1\n");
	}
}

/* \dump_state - Hamlib state dump */
static void cmd_dump_state(rigctl_server_t *srv, rigctl_client_t *c)
{
	double rx_min, rx_max, tx_min, tx_max;

	/* Get frequency limits from SDR caps
	 * Convert to user-visible range by subtracting upconverter offset
	 *
	 * TODO: These values should come from actual SDR hardware queries
	 *       (soapy_query_freq_range / uhd_query_freq_range) performed
	 *       at SDR startup/retune and stored in sdr_caps. Currently
	 *       falls back to 0-10 GHz when not configured. RX and TX can
	 *       have different ranges and upconverter offsets (transverter).
	 *       IMPORTANT: Never query hardware here! Use stored values only. */
	if (srv->sdr_caps.rx_freq_min > 0 || srv->sdr_caps.rx_freq_max > 0) {
		rx_min = srv->sdr_caps.rx_freq_min - srv->sdr_caps.rx_upconverter;
		rx_max = srv->sdr_caps.rx_freq_max - srv->sdr_caps.rx_upconverter;
		if (rx_min < 0) rx_min = 0;
	} else {
		/* Unknown range - report wide range (fallback) */
		rx_min = 0.0;
		rx_max = 10000000000.0;
	}

	if (srv->sdr_caps.tx_freq_min > 0 || srv->sdr_caps.tx_freq_max > 0) {
		tx_min = srv->sdr_caps.tx_freq_min - srv->sdr_caps.tx_upconverter;
		tx_max = srv->sdr_caps.tx_freq_max - srv->sdr_caps.tx_upconverter;
		if (tx_min < 0) tx_min = 0;
	} else {
		/* Unknown range - report wide range (fallback) */
		tx_min = 0.0;
		tx_max = 10000000000.0;
	}

	/* Protocol version */
	tx_write_str(c, "0\n");
	/* Model: NET rigctl */
	tx_write_str(c, "2\n");
	/* ITU region */
	tx_write_str(c, "1\n");

	/* RX range (user-visible frequencies) */
	if (srv->rx_enabled || srv->sdr_caps.has_rx) {
		tx_printf(c, "%.0f %.0f 0x2ef -1 -1 0x1 0x0\n", rx_min, rx_max);
	}
	tx_write_str(c, "0 0 0 0 0 0 0\n");

	/* TX range (if TX enabled) */
	if (srv->tx_enabled || srv->sdr_caps.has_tx) {
		tx_printf(c, "%.0f %.0f 0x2ef 1 1000000 0x1 0x0\n", tx_min, tx_max);
	}
	tx_write_str(c, "0 0 0 0 0 0 0\n");

	/* Tuning steps - terminated by 0 0 */
	tx_write_str(c, "0xef 1\n");
	tx_write_str(c, "0 0\n");

	/* Filter presets - report actual bandwidth for current mode */
	tx_write_str(c, "0x82 500\n");   /* CW normal */
	tx_write_str(c, "0x82 200\n");   /* CW narrow */
	tx_write_str(c, "0x82 2000\n");  /* CW wide */
	tx_write_str(c, "0x221 10000\n"); /* AM/FM normal */
	tx_write_str(c, "0x221 5000\n");  /* AM/FM narrow */
	tx_write_str(c, "0x221 20000\n"); /* AM/FM wide */
	tx_write_str(c, "0x0c 2700\n");  /* SSB normal */
	tx_write_str(c, "0x0c 1400\n");  /* SSB narrow */
	tx_write_str(c, "0x0c 3900\n");  /* SSB wide */
	/* WFM: report actual rf_bandwidth if available, else typical values */
	if (srv->radio && srv->radio->rf_bandwidth > 0) {
		tx_printf(c, "0x40 %d\n", (int)srv->radio->rf_bandwidth);
	} else {
		tx_write_str(c, "0x40 200000\n"); /* WFM normal */
		tx_write_str(c, "0x40 150000\n"); /* WFM narrow */
		tx_write_str(c, "0x40 270000\n"); /* WFM wide (stereo+RDS) */
	}
	tx_write_str(c, "0 0\n");

	/* max_rit, max_xit, max_ifshift */
	tx_write_str(c, "0\n0\n0\n");
	/* Announces */
	tx_write_str(c, "0\n");
	/* Preamp (empty = none), Attenuator (empty = none) */
	tx_write_str(c, "\n\n");
	/* Get/Set functions */
	tx_write_str(c, "0\n0\n");
	/* Get levels: SQL | STRENGTH */
	tx_write_str(c, "0x40000020\n");
	/* Set levels: SQL */
	tx_write_str(c, "0x20\n");
	/* Get/Set parm */
	tx_write_str(c, "0\n0\n");
}

/* v - Get VFO */
static void cmd_get_vfo(rigctl_client_t *c)
{
	tx_write_str(c, "VFOA\n");
}

/* V - Set VFO */
static void cmd_set_vfo(rigctl_client_t *c, const char *arg)
{
	if (!arg || !*arg) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}
	if (strcmp(arg, "?") == 0) {
		tx_write_str(c, "VFOA\n");
		return;
	}
	if (strcasecmp(arg, "VFOA") == 0) {
		tx_write_str(c, "RPRT 0\n");
	} else {
		tx_write_str(c, "RPRT 1\n");
	}
}

/* s - Get split status
 * Returns split mode (0=off, 1=on) and TX VFO. */
static void cmd_get_split(rigctl_server_t *srv, rigctl_client_t *c)
{
	tx_printf(c, "%d\n", srv->split);
	tx_write_str(c, "VFOA\n");
}

/* S - Set split mode
 * Enables/disables split operation (separate TX/RX frequencies).
 * When split is enabled, use 'I' command to set TX frequency. */
static void cmd_set_split(rigctl_server_t *srv, rigctl_client_t *c,
			  const char *arg1, const char *arg2)
{
	int split;

	if (!arg1 || !*arg1) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	split = atoi(arg1);
	(void)arg2;  /* TX VFO is always VFOA */

	srv->split = split ? 1 : 0;

	/* If disabling split, sync TX freq to RX freq */
	if (!srv->split && srv->tx_enabled && srv->rx_enabled) {
		srv->tx_freq = srv->rx_freq;
		if (srv->cb.set_tx_freq)
			srv->cb.set_tx_freq(srv->tx_freq, srv->cb.arg);
	}

	tx_write_str(c, "RPRT 0\n");
}

/* _ - Get version/info */
static void cmd_get_info(rigctl_client_t *c)
{
	tx_printf(c, "%s\n", VERSION_STRING);
}

/* t - Get PTT status
 * Returns current PTT state: 0=RX, 1=TX
 * For full-duplex operation, returns 1 if TX is active. */
static void cmd_get_ptt(rigctl_server_t *srv, rigctl_client_t *c)
{
	int ptt = srv->ptt;

	/* Use callback if available */
	if (srv->cb.get_ptt)
		ptt = srv->cb.get_ptt(srv->cb.arg);

	tx_printf(c, "%d\n", ptt);
}

/* T - Set PTT
 * TODO: Implement proper PTT control
 * Currently returns error as PTT switching is not implemented.
 * Full implementation requires:
 * - Callback to radio/SDR to switch TX/RX
 * - GPIO control for hardware PTT line (if applicable)
 * - Proper sequencing (antenna relay before TX power)
 * - TX timeout timer for safety
 */
static void cmd_set_ptt(rigctl_server_t *srv, rigctl_client_t *c, const char *arg)
{
	int ptt;

	if (!arg || !*arg) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	ptt = atoi(arg);

	/* Use callback if available */
	if (srv->cb.set_ptt) {
		int rc = srv->cb.set_ptt(ptt, srv->cb.arg);
		if (rc == 0) {
			srv->ptt = ptt;
			tx_write_str(c, "RPRT 0\n");
			return;
		}
	}

	/* TODO: PTT control not implemented
	 * Need to wire up set_ptt callback to radio/SDR control */
	LOGP(DRIGCTL, LOGL_INFO, "PTT control not implemented (requested: %d)\n", ptt);
	tx_write_str(c, "RPRT 1\n");
}

/* i - Get split TX frequency
 * Returns the TX frequency (may differ from RX in split mode).
 * Frequency is returned as user-visible (with upconverter offset applied). */
static void cmd_get_split_freq(rigctl_server_t *srv, rigctl_client_t *c)
{
	double freq = sdr_to_user_freq(srv, srv->tx_freq, 1);
	tx_printf(c, "%.0f\n", freq);
}

/* I - Set split TX frequency
 * Sets the TX frequency independently of RX (enables split mode).
 * Frequency is expected as user-visible (upconverter offset will be applied). */
static void cmd_set_split_freq(rigctl_server_t *srv, rigctl_client_t *c, const char *arg)
{
	double user_freq, sdr_freq;
	int rc = -1;

	if (!arg || !*arg) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	user_freq = atof(arg);
	if (user_freq <= 0) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	/* Validate TX frequency */
	if (validate_freq(srv, user_freq, 1, &sdr_freq) < 0) {
		tx_write_str(c, "RPRT 1\n");
		return;
	}

	/* Set TX frequency via callback */
	if (srv->cb.set_tx_freq)
		rc = srv->cb.set_tx_freq(sdr_freq, srv->cb.arg);

	if (rc == 0) {
		srv->tx_freq = sdr_freq;
		srv->split = 1;  /* Enable split mode */
		tx_write_str(c, "RPRT 0\n");
	} else {
		tx_write_str(c, "RPRT 1\n");
	}
}

/* 0x8b - Get DCD (squelch/signal detect) */
static void cmd_get_dcd(rigctl_server_t *srv, rigctl_client_t *c)
{
	/* Return 1 if signal above squelch, 0 otherwise */
	int dcd = (srv->signal_level > srv->squelch_level) ? 1 : 0;
	tx_printf(c, "%d\n", dcd);
}

/* \get_vfo_info - Get VFO info (freq, mode, width, split, satmode)
 * Returns info for the current VFO (VFOA).
 * In split mode, returns RX frequency; use 'i' for TX frequency. */
static void cmd_get_vfo_info(rigctl_server_t *srv, rigctl_client_t *c, const char *arg)
{
	double freq;
	const char *mode_str;
	int bandwidth;
	
	(void)arg;  /* We only have VFOA */
	
	/* Get frequency as user-visible */
	if (srv->rx_enabled) {
		freq = sdr_to_user_freq(srv, srv->rx_freq, 0);
	} else {
		freq = sdr_to_user_freq(srv, srv->tx_freq, 1);
	}

	/* Get mode and bandwidth for the active direction */
	if (srv->rx_enabled) {
		bandwidth = srv->rx_bandwidth ? srv->rx_bandwidth : srv->bandwidth;
		mode_str = mode_to_str(srv->rx_mode ? srv->rx_mode : srv->mode, srv->stereo, bandwidth);
	} else {
		bandwidth = srv->tx_bandwidth ? srv->tx_bandwidth : srv->bandwidth;
		mode_str = mode_to_str(srv->tx_mode ? srv->tx_mode : srv->mode, srv->stereo, bandwidth);
	}
	
	tx_printf(c, "%.0f\n", freq);
	tx_printf(c, "%s\n", mode_str);
	tx_printf(c, "%d\n", bandwidth);
	tx_printf(c, "%d\n", srv->split);  /* Split status */
	tx_write_str(c, "0\n");  /* Satmode off */
}

/* \get_vfo_list - Get list of available VFOs */
static void cmd_get_vfo_list(rigctl_client_t *c)
{
	tx_write_str(c, "VFOA\n");
}

/* \get_modes - Get supported modes with bandwidths */
static void cmd_get_modes(rigctl_server_t *srv, rigctl_client_t *c)
{
	/* Report filter bandwidths for each mode.
	 * For WFM, report actual rf_bandwidth if available. */
	tx_write_str(c, "AM 10000 6000 3000\n");
	tx_write_str(c, "FM 15000 10000 7000\n");
	if (srv->radio && srv->radio->rf_bandwidth > 0) {
		tx_printf(c, "WFM %d\n", (int)srv->radio->rf_bandwidth);
	} else {
		tx_write_str(c, "WFM 270000 200000 150000\n");
	}
	tx_write_str(c, "USB 3000 2400 1800\n");
	tx_write_str(c, "LSB 3000 2400 1800\n");
}

/* 1 - dump_caps */
static void cmd_dump_caps(rigctl_server_t *srv, rigctl_client_t *c)
{
	tx_write_str(c, "Caps dump for model: 2\n");
	tx_write_str(c, "Model name:\tosmoradio\n");
	tx_write_str(c, "Mfg name:\tosmocom\n");
	tx_write_str(c, "Backend version:\t1.0\n");
	tx_printf(c, "Rig type:\t%s\n", srv->tx_enabled ? "Transceiver" : "Receiver");
	tx_write_str(c, "PTT type:\tNone\n");
	tx_write_str(c, "DCD type:\tRig capable\n");
	tx_write_str(c, "Port type:\tNetwork link\n");
}

/* ============================================================
 * Command Parser
 * ============================================================ */

static void process_command(rigctl_server_t *srv, rigctl_client_t *c,
			    const char *line)
{
	char cmd[RIGCTL_CMD_MAX];
	char *args[8];
	int nargs = 0;
	char *p, *saveptr;

	if (!line || !*line)
		return;

	strncpy(cmd, line, sizeof(cmd) - 1);
	cmd[sizeof(cmd) - 1] = '\0';

	/* Trim trailing whitespace */
	p = cmd + strlen(cmd) - 1;
	while (p >= cmd && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n'))
		*p-- = '\0';

	if (!cmd[0])
		return;

	LOGP(DRIGCTL, LOGL_DEBUG, "Rigctl cmd from %s: '%s'\n", c->client_ip, cmd);

	/* Parse into tokens */
	p = strtok_r(cmd, " \t", &saveptr);
	while (p && nargs < 8) {
		args[nargs++] = p;
		p = strtok_r(NULL, " \t", &saveptr);
	}

	if (nargs == 0)
		return;

	/* Single-char commands */
	if (strlen(args[0]) == 1) {
		switch (args[0][0]) {
		case 'f':
			cmd_get_freq(srv, c);
			return;
		case 'F':
			cmd_set_freq(srv, c, nargs > 1 ? args[1] : NULL);
			return;
		case 'm':
			cmd_get_mode(srv, c);
			return;
		case 'M':
			cmd_set_mode(srv, c, nargs > 1 ? args[1] : NULL,
				     nargs > 2 ? args[2] : NULL);
			return;
		case 'l':
			cmd_get_level(srv, c, nargs > 1 ? args[1] : NULL);
			return;
		case 'L':
			cmd_set_level(srv, c, nargs > 1 ? args[1] : NULL,
				      nargs > 2 ? args[2] : NULL);
			return;
		case 'u':
			cmd_get_func(srv, c, nargs > 1 ? args[1] : NULL);
			return;
		case 'U':
			cmd_set_func(srv, c, nargs > 1 ? args[1] : NULL,
				     nargs > 2 ? args[2] : NULL);
			return;
		case 'p':
			cmd_get_param(srv, c, nargs > 1 ? args[1] : NULL);
			return;
		case 'v':
			cmd_get_vfo(c);
			return;
		case 'V':
			cmd_set_vfo(c, nargs > 1 ? args[1] : NULL);
			return;
		case 's':
			cmd_get_split(srv, c);
			return;
		case 'S':
			cmd_set_split(srv, c, nargs > 1 ? args[1] : NULL,
				      nargs > 2 ? args[2] : NULL);
			return;
		case 't':
			cmd_get_ptt(srv, c);
			return;
		case 'T':
			cmd_set_ptt(srv, c, nargs > 1 ? args[1] : NULL);
			return;
		case 'i':
			cmd_get_split_freq(srv, c);
			return;
		case 'I':
			cmd_set_split_freq(srv, c, nargs > 1 ? args[1] : NULL);
			return;
		case '_':
			cmd_get_info(c);
			return;
		case '1':
			cmd_dump_caps(srv, c);
			return;
		case 'd':
			/* Get DCS code - we don't use tone squelch, return 0
			 * TODO: DCS (Digital Coded Squelch) implementation
			 * - DCS uses 23-bit digital codes for selective calling
			 * - Common codes: 023, 025, 026, 031, 032, etc.
			 * - Requires: DCS encoder/decoder in audio path
			 * - Used for: repeater access, group calling
			 * - Implementation: Add DCS detector to FM demodulator */
			tx_write_str(c, "0\n");
			return;
		case 'D':
			/* Set DCS code - accept but ignore (see TODO above) */
			tx_write_str(c, "RPRT 0\n");
			return;
		case 'c':
			/* Get CTCSS tone - we don't use tone squelch, return 0
			 * TODO: CTCSS (Continuous Tone-Coded Squelch System) implementation
			 * - CTCSS uses sub-audible tones (67.0-254.1 Hz) for selective calling
			 * - Also known as PL (Private Line) or tone squelch
			 * - Requires: CTCSS tone generator and detector
			 * - Used for: repeater access, interference rejection
			 * - Implementation: Add Goertzel filter or FFT-based tone detector
			 *   to FM demodulator, tone generator to FM modulator */
			tx_write_str(c, "0\n");
			return;
		case 'C':
			/* Set CTCSS tone - accept but ignore (see TODO above) */
			tx_write_str(c, "RPRT 0\n");
			return;
		case 'q':
		case 'Q':
			/* Send success response and mark for close after flush */
			tx_write_str(c, "RPRT 0\n");
			c->close_pending = 1;
			return;
		}
	}

	/* Extended commands */
	if (strcmp(args[0], "\\chk_vfo") == 0) {
		tx_write_str(c, "0\n");
		return;
	}
	if (strcmp(args[0], "\\dump_state") == 0) {
		cmd_dump_state(srv, c);
		return;
	}
	if (strcmp(args[0], "\\get_powerstat") == 0) {
		tx_write_str(c, "1\n");
		return;
	}
	if (strcmp(args[0], "\\set_powerstat") == 0) {
		tx_write_str(c, "RPRT 0\n");
		return;
	}
	if (strcmp(args[0], "\\get_vfo_info") == 0) {
		cmd_get_vfo_info(srv, c, nargs > 1 ? args[1] : NULL);
		return;
	}
	if (strcmp(args[0], "\\get_vfo_list") == 0) {
		cmd_get_vfo_list(c);
		return;
	}
	if (strcmp(args[0], "\\get_modes") == 0) {
		cmd_get_modes(srv, c);
		return;
	}
	if (strcmp(args[0], "\\get_dcd") == 0) {
		cmd_get_dcd(srv, c);
		return;
	}
	if (strcmp(args[0], "\\dump_caps") == 0) {
		cmd_dump_caps(srv, c);
		return;
	}
	if (strcmp(args[0], "\\get_rig_info") == 0) {
		/* Return basic rig info in key=value format */
		double freq;
		if (srv->rx_enabled)
			freq = sdr_to_user_freq(srv, srv->rx_freq, 0);
		else
			freq = sdr_to_user_freq(srv, srv->tx_freq, 1);
		tx_printf(c, "VFO=VFOA\n");
		tx_printf(c, "Freq=%.0f\n", freq);
		tx_printf(c, "Mode=%s\n", mode_to_str(srv->mode, srv->stereo, srv->bandwidth));
		tx_printf(c, "Width=%d\n", srv->bandwidth);
		tx_printf(c, "Split=%d\n", srv->split);
		tx_printf(c, "SatMode=0\n");
		tx_printf(c, "PTT=%d\n", srv->ptt);
		if (srv->tx_enabled && srv->split) {
			double tx_freq = sdr_to_user_freq(srv, srv->tx_freq, 1);
			tx_printf(c, "TXFreq=%.0f\n", tx_freq);
		}
		return;
	}
	if (strcmp(args[0], "\\halt") == 0) {
		/* Send success response and mark for close after flush */
		tx_write_str(c, "RPRT 0\n");
		c->close_pending = 1;
		return;
	}
	if (strcmp(args[0], "\\set_vfo_opt") == 0) {
		/* VFO mode option - we always use simple mode */
		tx_write_str(c, "RPRT 0\n");
		return;
	}
	if (strcmp(args[0], "\\hamlib_version") == 0) {
		tx_write_str(c, "Hamlib 4.5 compatible\n");
		return;
	}

	/* Unknown command */
	LOGP(DRIGCTL, LOGL_INFO, "Rigctl unknown command: '%s'\n", args[0]);
	tx_write_str(c, "RPRT 1\n");
}

/* ============================================================
 * I/O Handling
 * ============================================================ */

static void client_read(rigctl_server_t *srv, rigctl_client_t *c)
{
	uint8_t buf[256];
	int rc, i;

	rc = read(c->fd, buf, sizeof(buf));
	if (rc <= 0) {
		if (rc == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
			LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl client disconnected: %s:%d\n",
			     c->client_ip, c->client_port);
			client_close(c);
		}
		return;
	}

	srv->rx_bytes += rc;

	/* Process bytes, looking for newlines */
	for (i = 0; i < rc; i++) {
		if (buf[i] == '\n' || buf[i] == '\r') {
			if (c->cmd_len > 0) {
				c->cmd_buf[c->cmd_len] = '\0';
				process_command(srv, c, c->cmd_buf);
				c->cmd_len = 0;

				/* If close is pending, stop processing more commands */
				if (c->close_pending)
					return;
			}
		} else if (c->cmd_len < RIGCTL_CMD_MAX - 1) {
			c->cmd_buf[c->cmd_len++] = buf[i];
		}
	}
}

static void client_write(rigctl_server_t *srv, rigctl_client_t *c)
{
	uint8_t buf[256];
	int len, rc;

	len = rigctl_ring_used(&c->tx_ring);
	if (len == 0)
		return;

	if (len > (int)sizeof(buf))
		len = sizeof(buf);

	len = rigctl_ring_read(&c->tx_ring, buf, len);
	if (len <= 0)
		return;

	/* Log response (truncate for readability) */
	if (len > 0) {
		char logbuf[128];
		int loglen = (len < 80) ? len : 80;
		int i, j;
		for (i = 0, j = 0; i < loglen && j < (int)sizeof(logbuf) - 4; i++) {
			if (buf[i] == '\n') {
				logbuf[j++] = '\\';
				logbuf[j++] = 'n';
			} else if (buf[i] == '\r') {
				logbuf[j++] = '\\';
				logbuf[j++] = 'r';
			} else if (buf[i] >= 32 && buf[i] < 127) {
				logbuf[j++] = buf[i];
			} else {
				logbuf[j++] = '?';
			}
		}
		if (len > 80) {
			logbuf[j++] = '.';
			logbuf[j++] = '.';
			logbuf[j++] = '.';
		}
		logbuf[j] = '\0';
		LOGP(DRIGCTL, LOGL_DEBUG, "Rigctl resp to %s: '%s'\n", c->client_ip, logbuf);
	}

	rc = write(c->fd, buf, len);
	if (rc > 0)
		srv->tx_bytes += rc;
}

static void accept_client(rigctl_server_t *srv)
{
	struct sockaddr_in addr;
	socklen_t addrlen = sizeof(addr);
	int fd, i, slot = -1;
	char ip[RIGCTL_EP_IP_LEN];

	fd = accept4(srv->listen_fd, (struct sockaddr *)&addr, &addrlen, SOCK_NONBLOCK);
	if (fd < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK)
			LOGP(DRIGCTL, LOGL_ERROR, "Rigctl accept() failed: %s\n", strerror(errno));
		return;
	}

	inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));

	/* Check allowed hosts */
	if (!is_host_allowed(srv, ip)) {
		LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl connection rejected from %s (not allowed)\n", ip);
		close(fd);
		return;
	}

	/* Find free slot */
	for (i = 0; i < RIGCTL_MAX_CLIENTS; i++) {
		if (srv->clients[i].fd == RIGCTL_FD_NONE) {
			slot = i;
			break;
		}
	}

	if (slot < 0) {
		LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl connection rejected from %s (max clients)\n", ip);
		close(fd);
		return;
	}

	/* Disable Nagle */
	int opt = 1;
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

	/* Initialize client */
	client_init(&srv->clients[slot]);
	srv->clients[slot].fd = fd;
	snprintf(srv->clients[slot].client_ip, sizeof(srv->clients[slot].client_ip), "%s", ip);
	srv->clients[slot].client_port = ntohs(addr.sin_port);
	srv->clients[slot].connect_time_us = time_us();
	srv->num_clients++;
	srv->total_connections++;

	LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl client connected: %s:%d\n",
	     ip, ntohs(addr.sin_port));
}

/* ============================================================
 * Public API
 * ============================================================ */

int rigctl_server_init(rigctl_server_t *srv, const char *endpoint,
		       const char *allowed_hosts)
{
	int i;

	if (!srv || !endpoint)
		return -1;

	memset(srv, 0, sizeof(*srv));

	/* Parse endpoint */
	if (rigctl_ep_parse(endpoint, &srv->ep) < 0) {
		LOGP(DRIGCTL, LOGL_ERROR, "Failed to parse rigctl endpoint: %s\n", endpoint);
		return -1;
	}

	/* Allowed hosts (empty = allow all) */
	if (allowed_hosts)
		strncpy(srv->allowed_hosts, allowed_hosts, sizeof(srv->allowed_hosts) - 1);
	/* else: leave empty to allow all hosts */

	/* Initialize clients */
	srv->listen_fd = RIGCTL_FD_NONE;
	srv->serial_fd = RIGCTL_FD_NONE;
	for (i = 0; i < RIGCTL_MAX_CLIENTS; i++)
		client_init(&srv->clients[i]);

	/* Default values */
	srv->signal_level = -100.0;
	srv->squelch_level = -150.0;
	srv->audio_gain = 0.0;
	srv->deviation = 75000.0;
	srv->mod_index = 1.0;
	srv->dsp_running = 1;

	/* Open endpoint */
	if (srv->ep.type == RIGCTL_EP_TCP) {
		srv->listen_fd = tcp_listen(&srv->ep);
		if (srv->listen_fd < 0)
			return -1;
	} else if (srv->ep.type == RIGCTL_EP_SERIAL) {
		srv->serial_fd = serial_open(&srv->ep);
		if (srv->serial_fd < 0)
			return -1;
		/* Serial acts as single client */
		srv->clients[0].fd = srv->serial_fd;
		strcpy(srv->clients[0].client_ip, "serial");
		srv->num_clients = 1;
	}

	return 0;
}

void rigctl_server_cleanup(rigctl_server_t *srv)
{
	int i;

	if (!srv)
		return;

	/* Close clients */
	for (i = 0; i < RIGCTL_MAX_CLIENTS; i++) {
		if (srv->clients[i].fd != RIGCTL_FD_NONE &&
		    srv->clients[i].fd != srv->serial_fd)
			client_close(&srv->clients[i]);
	}

	/* Close listen socket */
	if (srv->listen_fd != RIGCTL_FD_NONE) {
		close(srv->listen_fd);
		srv->listen_fd = RIGCTL_FD_NONE;
	}

	/* Close serial */
	if (srv->serial_fd != RIGCTL_FD_NONE) {
		close(srv->serial_fd);
		srv->serial_fd = RIGCTL_FD_NONE;
	}

	LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl server stopped (rx=%lu tx=%lu conns=%lu)\n",
	     (unsigned long)srv->rx_bytes, (unsigned long)srv->tx_bytes,
	     (unsigned long)srv->total_connections);
}

void rigctl_server_set_callbacks(rigctl_server_t *srv, const rigctl_callbacks_t *cb)
{
	if (srv && cb)
		srv->cb = *cb;
}

void rigctl_server_set_radio(rigctl_server_t *srv, struct radio *radio)
{
	if (srv)
		srv->radio = radio;
}

void rigctl_server_set_meter(rigctl_server_t *srv, struct signal_meter *meter)
{
	if (srv)
		srv->meter = meter;
}

void rigctl_server_set_mode_flags(rigctl_server_t *srv, int tx, int rx)
{
	if (srv) {
		srv->tx_enabled = tx;
		srv->rx_enabled = rx;
	}
}

void rigctl_server_set_sdr_caps(rigctl_server_t *srv, const rigctl_sdr_caps_t *caps)
{
	if (srv && caps) {
		srv->sdr_caps = *caps;
		LOGP(DRIGCTL, LOGL_INFO, "SDR caps set:\n");
		LOGP(DRIGCTL, LOGL_INFO, "  RX: %.0f-%.0f Hz (upconv=%.0f Hz)\n",
		     caps->rx_freq_min, caps->rx_freq_max, caps->rx_upconverter);
		LOGP(DRIGCTL, LOGL_INFO, "  TX: %.0f-%.0f Hz (upconv=%.0f Hz)\n",
		     caps->tx_freq_min, caps->tx_freq_max, caps->tx_upconverter);
		if (caps->rx_gain_names[0])
			LOGP(DRIGCTL, LOGL_INFO, "  RX gains: %s (%.1f-%.1f dB)\n",
			     caps->rx_gain_names, caps->rx_gain_min, caps->rx_gain_max);
		if (caps->tx_gain_names[0])
			LOGP(DRIGCTL, LOGL_INFO, "  TX gains: %s (%.1f-%.1f dB)\n",
			     caps->tx_gain_names, caps->tx_gain_min, caps->tx_gain_max);
		if (caps->is_split)
			LOGP(DRIGCTL, LOGL_INFO, "  Split mode: separate TX/RX devices\n");
	}
}

int rigctl_server_poll(rigctl_server_t *srv)
{
	int i;

	if (!srv)
		return -1;

	/* Accept new TCP connections */
	if (srv->listen_fd != RIGCTL_FD_NONE)
		accept_client(srv);

	/* Process clients */
	for (i = 0; i < RIGCTL_MAX_CLIENTS; i++) {
		rigctl_client_t *c = &srv->clients[i];
		if (c->fd == RIGCTL_FD_NONE)
			continue;

		/* Read incoming data (skip if close pending) */
		if (!c->close_pending)
			client_read(srv, c);

		/* Write outgoing data */
		if (c->fd != RIGCTL_FD_NONE)
			client_write(srv, c);

		/* Handle pending close - close after tx_ring is empty */
		if (c->close_pending && rigctl_ring_used(&c->tx_ring) == 0) {
			LOGP(DRIGCTL, LOGL_NOTICE, "Rigctl client disconnected (quit): %s:%d\n",
			     c->client_ip, c->client_port);
			client_close(c);
			if (srv->ep.type == RIGCTL_EP_TCP)
				srv->num_clients--;
			continue;
		}

		/* Update client count for unexpected disconnects */
		if (c->fd == RIGCTL_FD_NONE && srv->ep.type == RIGCTL_EP_TCP)
			srv->num_clients--;
	}

	return 0;
}

void rigctl_server_set_signal(rigctl_server_t *srv, double level_dbfs)
{
	if (srv)
		srv->signal_level = level_dbfs;
}

void rigctl_server_set_tx_status(rigctl_server_t *srv,
				 double rms, double peak_i, double peak_q)
{
	if (srv) {
		srv->tx_rms = rms;
		srv->tx_peak_i = peak_i;
		srv->tx_peak_q = peak_q;
	}
}

void rigctl_server_set_frequency(rigctl_server_t *srv, double rx_freq, double tx_freq)
{
	if (srv) {
		srv->rx_freq = rx_freq;
		srv->tx_freq = tx_freq;
		/* If not in split mode, keep TX synced to RX */
		if (!srv->split && srv->tx_enabled && srv->rx_enabled)
			srv->tx_freq = rx_freq;
	}
}

void rigctl_server_set_modulation(rigctl_server_t *srv, int mode, int bandwidth,
				  double deviation, double mod_index)
{
	if (srv) {
		srv->mode = mode;
		srv->rx_mode = mode;
		srv->tx_mode = mode;
		srv->bandwidth = bandwidth;
		srv->rx_bandwidth = bandwidth;
		srv->tx_bandwidth = bandwidth;
		srv->deviation = deviation;
		srv->mod_index = mod_index;
	}
}

void rigctl_server_set_rds(rigctl_server_t *srv, uint16_t pi,
			   const char *ps, const char *rt)
{
	if (!srv)
		return;

	srv->rds_pi = pi;
	if (ps)
		strncpy(srv->rds_ps, ps, sizeof(srv->rds_ps) - 1);
	if (rt)
		strncpy(srv->rds_rt, rt, sizeof(srv->rds_rt) - 1);
}

void rigctl_server_set_flags(rigctl_server_t *srv, int stereo, int rds)
{
	if (srv) {
		srv->stereo = stereo;
		srv->rds_enabled = rds;
	}
}

int rigctl_server_connected(const rigctl_server_t *srv)
{
	int i;

	if (!srv)
		return 0;

	for (i = 0; i < RIGCTL_MAX_CLIENTS; i++) {
		if (srv->clients[i].fd != RIGCTL_FD_NONE)
			return 1;
	}
	return 0;
}
