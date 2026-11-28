/* The SDR device's clock against the PC's clocks: status, protocol needs, a slow carrier trim
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * A lab tool, best effort: libsdr's control thread reads the device time every 10 s (and the RX
 * stream's time stamps give it for free); here a point every 10 s goes into a history, and least
 * squares give the device clock's rate against CLOCK_MONOTONIC_RAW (the PC's crystal, steady but
 * some 45 ppm off) and against CLOCK_REALTIME while NTP keeps it synchronised (UTC, the truth in
 * the long run, but only as good as NTP: its error at both ends of the span bounds the slope).
 *
 * Each protocol declares what it needs (the table). A warning only when a need is certainly not
 * met - the error minus its whole uncertainty beyond the need - with what to do. Everything else
 * on demand (SIGUSR1) and one line at the end; at most one line of its own per 10 minutes.
 *
 * The carrier trim (--sdr-clock-trim, off by default): see clock_drift.h. dry logs what it would
 * do; on moves the TX carrier through the NCO (as --sdr-ppm, the symbol rate stays).
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/timex.h>
#include <stdint.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "clock_monitor.h"
#include "clock_drift.h"
#ifdef HAVE_SDR
#include "../libsdr/sdr.h"
#include "../libsdr/sdr_config.h"
#endif

int clock_ntp_state(double *esterror, const char **how)
{
	struct timex tx;
	int state;

	*esterror = -1.0;
	*how = "NTP";
	memset(&tx, 0, sizeof(tx));
	state = ntp_adjtime(&tx);
	if (state < 0)
		return -1;
	if (state == TIME_ERROR || (tx.status & STA_UNSYNC)) {
		/* systemd-timesyncd (SNTP) sets the clock without the kernel's error estimate: it
		 * marks a synchronised clock with this file */
		*how = "systemd-timesyncd";
		return (access("/run/systemd/timesync/synchronized", F_OK) == 0) ? 1 : 0;
	}
	*esterror = tx.esterror / 1e6;
	return 1;
}

#ifdef HAVE_SDR

/* What a protocol needs of the clock. From min_freq on (0: any carrier). */
static const struct clock_need {
	const char	*protocol;
	const char	*title;
	double		min_freq;
	double		utc_ms;		/* its frames within ±ms of UTC (0: none) */
	const char	*utc_ref;
	double		carrier_hz;	/* the carrier within ±Hz (0: none) */
	double		carrier_ppm;	/* or ±ppm (0: none) */
} needs[] = {
	{ "ermes", "ERMES", 0.0, 2.0, "ETS 300 133-4 §12.4", 200.0, 0.0 },
	{ "nmt", "NMT-900", 800e6, 0.0, NULL, 0.0, 0.26 },
	{ NULL, NULL, 0.0, 0.0, NULL, 0.0, 0.0 }
};

/* NTP's error when it gives none (timesyncd): assumed, s */
#define NTP_EPS_ASSUMED		5e-3
#define NTP_EPS_MIN		0.5e-3
#define POINT_EVERY_S		10.0
#define OWN_LINE_EVERY_S	600.0

static struct {
	int		on;
	const struct clock_need *need;
	double		carrier, need_ppm;
	double		static_ppm;
	int		trim_mode;	/* 0 off, 1 dry, 2 on */
	FILE		*log;
	int		started, src;
	unsigned long	last_seq;
	double		last_point, last_check, last_own;
	int		warned_carrier, warned_utc, trim_failed;
	int		ntp;
	double		eps;
	const char	*ntp_how;
	sdr_clock_status_t st;
	clock_drift_t	cd;
	clock_trim_t	tr;
} cm;

static double mono_now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void ntp_now(void)
{
	double est;

	cm.ntp = (clock_ntp_state(&est, &cm.ntp_how) == 1);
	cm.eps = (est < 0.0) ? NTP_EPS_ASSUMED : (est < NTP_EPS_MIN) ? NTP_EPS_MIN : est;
}

void clock_monitor_start(const char *protocol, double carrier)
{
	int i;

	memset(&cm, 0, sizeof(cm));
	cm.on = 1;
	cm.src = -1;
	cm.carrier = carrier;
	cm.static_ppm = sdr_get_tx_ppm();
	cm.trim_mode = sdr_config ? sdr_config->clock_trim : 0;
	clock_drift_init(&cm.cd);
	clock_trim_init(&cm.tr);
	for (i = 0; protocol && needs[i].protocol; i++) {
		if (strcasecmp(needs[i].protocol, protocol) || carrier < needs[i].min_freq)
			continue;
		cm.need = &needs[i];
		if (needs[i].carrier_hz > 0.0 && carrier > 0.0)
			cm.need_ppm = needs[i].carrier_hz / carrier * 1e6;
		else
			cm.need_ppm = needs[i].carrier_ppm;
		break;
	}
	if (sdr_config && sdr_config->clock_log) {
		cm.log = fopen(sdr_config->clock_log, "a");
		if (!cm.log)
			LOGP(DSDR, LOGL_NOTICE, "Clock log '%s' can't be opened.\n", sdr_config->clock_log);
		else
			fprintf(cm.log, "# raw_s device-raw_s device-utc_s ntp src(1 rx 0 poll) poll:device-raw_s poll:device-utc_s poll:uncertainty_s temp_C ref_locked poll:raw_s\n");
	}
}

static const char *lock_text(int locked, const char *yes, const char *no)
{
	return (locked < 0) ? "" : locked ? yes : no;
}

static void start_line(void)
{
	char needs_text[256] = "", hint[256] = "", temp[32] = "";
	const struct clock_need *n = cm.need;

	if (!isnan(cm.st.temperature))
		snprintf(temp, sizeof(temp), ", %.1f °C", cm.st.temperature);
	if (n) {
		int len = snprintf(needs_text, sizeof(needs_text), "; %s needs", n->title);

		if (cm.need_ppm > 0.0 && cm.carrier > 0.0) {
			if (n->carrier_hz > 0.0)
				len += snprintf(needs_text + len, sizeof(needs_text) - len, " the carrier within ±%.0f Hz (±%.2f ppm at %.3f MHz)",
						n->carrier_hz, cm.need_ppm, cm.carrier / 1e6);
			else
				len += snprintf(needs_text + len, sizeof(needs_text) - len, " the carrier within ±%.2f ppm", cm.need_ppm);
		}
		if (n->utc_ms > 0.0)
			snprintf(needs_text + len, sizeof(needs_text) - len, "%s its cycles within ±%.0f ms of UTC (%s)",
				 (cm.need_ppm > 0.0) ? " and" : "", n->utc_ms, n->utc_ref);
		/* what NTP can never confirm (in the history's 5.7 h): say what can */
		if (cm.need_ppm > 0.0 && 2.0 * cm.eps / (CLOCK_DRIFT_POINTS * POINT_EVERY_S) * 1e6 > cm.need_ppm)
			snprintf(hint, sizeof(hint), " NTP (±%.0f ms) can't confirm ±%.2f ppm: calibrate against towers (osmocalibrate gives the --sdr-ppm).",
				 cm.eps * 1e3, cm.need_ppm);
		else if (cm.need_ppm > 0.0 && cm.static_ppm == 0.0 && strcasecmp(cm.st.clock_source, "gpsdo"))
			snprintf(hint, sizeof(hint), " No --sdr-ppm: a free-running reference can be ppm off; osmocalibrate gives the correction.");
	}
	LOGP(DSDR, LOGL_INFO, "Clock: reference %s%s, time source %s%s%s; the PC's clock %s%s%s%s; --sdr-ppm %+.3f, trim %s.%s The drift is measured from now (the status: kill -USR1 %d).\n",
	     cm.st.clock_source, lock_text(cm.st.ref_locked, " locked", " NOT LOCKED"),
	     cm.st.time_source, lock_text(cm.st.gps_locked, ", GPS locked", ", GPS NOT locked"), temp,
	     cm.ntp ? "synchronised by " : "NOT synchronised (", cm.ntp ? cm.ntp_how : "no NTP: the drift against UTC can't be measured)",
	     (cm.ntp && cm.eps == NTP_EPS_ASSUMED) ? " (no error estimate, 5 ms assumed)" : "",
	     needs_text, cm.static_ppm, cm.trim_mode == 2 ? "on" : cm.trim_mode == 1 ? "dry run" : "off", hint, (int)getpid());
}

/* the status as one line */
static void status_line(int level, const char *why)
{
	clock_fit_t raw, utc;
	char temp[32] = "", utc_text[160] = "", carrier[160] = "", grid[128] = "";
	double bound = INFINITY, r = 0.0;
	int have_utc;

	if (!isnan(cm.st.temperature))
		snprintf(temp, sizeof(temp), ", %.1f °C", cm.st.temperature);
	if (clock_drift_fit_raw(&cm.cd, &raw) < 0) {
		LOGP(DSDR, level, "Clock%s: reference %s%s%s (lock changes %lu); too few points yet.\n", why,
		     cm.st.clock_source, lock_text(cm.st.ref_locked, " locked", " NOT LOCKED"), temp, cm.st.ref_changes);
		return;
	}
	have_utc = (clock_drift_fit_utc(&cm.cd, &utc) == 0);
	if (have_utc) {
		bound = clock_fit_utc_bound(&utc, cm.eps);
		r = utc.slope * 1e6 + cm.static_ppm + cm.tr.applied;
		snprintf(utc_text, sizeof(utc_text), ", against NTP's UTC %+.3f ppm ± %.3f over %.0f s",
			 utc.slope * 1e6, bound, utc.span);
		if (cm.carrier > 0.0)
			snprintf(carrier, sizeof(carrier), "; the carrier %+.0f Hz ± %.0f (%+.3f ppm with --sdr-ppm %+.3f%s)%s",
				 r * cm.carrier / 1e6, bound * cm.carrier / 1e6, r, cm.static_ppm,
				 cm.trim_mode ? (cm.trim_mode == 2 ? ", trimmed" : ", dry trim") : "",
				 (!cm.need || cm.need_ppm <= 0.0) ? ""
				 : (fabs(r) - bound > cm.need_ppm) ? ": NOT within the need"
				 : (fabs(r) + bound <= cm.need_ppm) ? ": within the need" : ": not known yet");
		if (cm.need && cm.need->utc_ms > 0.0)
			snprintf(grid, sizeof(grid), "; the cycles %+.2f ms from UTC since %.0f s (need ±%.0f, NTP's own ±%.0f)",
				 clock_drift_utc_walk(&cm.cd) * 1e3, utc.span, cm.need->utc_ms, cm.eps * 1e3);
	} else
		snprintf(utc_text, sizeof(utc_text), ", against UTC not measured (NTP %s)", cm.ntp ? "just synchronised" : "not synchronised");
	if (cm.tr.steps)
		snprintf(carrier + strlen(carrier), sizeof(carrier) - strlen(carrier), "; %d trim steps (%+.3f ppm)", cm.tr.steps, cm.tr.applied);
	LOGP(DSDR, level, "Clock%s: reference %s%s%s (lock changes %lu); the device against the PC's raw clock %+.3f ppm ± %.3f (scatter %.3f ms)%s, %d points (%s)%s%s; control calls at most %.1f ms.\n",
	     why, cm.st.clock_source, lock_text(cm.st.ref_locked, " locked", " NOT LOCKED"), temp, cm.st.ref_changes,
	     raw.slope * 1e6, 3.0 * raw.se * 1e6, raw.scatter * 1e3, utc_text, raw.n,
	     cm.src == 1 ? "RX time stamps" : "device time reads", carrier, grid, cm.st.ctl_ms_max);
}

static int own_line_allowed(double now)
{
	if (cm.last_own && now - cm.last_own < OWN_LINE_EVERY_S)
		return 0;
	cm.last_own = now;
	return 1;
}

static void evaluate(double now)
{
	clock_fit_t utc;
	double bound, r, lower, step, minutes;

	if (clock_drift_fit_utc(&cm.cd, &utc) < 0)
		return;
	bound = clock_fit_utc_bound(&utc, cm.eps);
	r = utc.slope * 1e6 + cm.static_ppm + cm.tr.applied;
	if (cm.need && cm.need_ppm > 0.0 && cm.carrier > 0.0 && !cm.warned_carrier && fabs(r) - bound > cm.need_ppm) {
		if (!own_line_allowed(now))
			return;
		cm.warned_carrier = 1;
		LOGP(DSDR, LOGL_NOTICE, "Clock: the %s carrier is %+.0f Hz off (%+.2f ppm ± %.2f against NTP's UTC over %.0f min), beyond the ±%.0f Hz it needs. Calibrate (osmocalibrate gives the --sdr-ppm) or use a disciplined reference (GPSDO, a good 10 MHz)%s.\n",
		     cm.need->title, r * cm.carrier / 1e6, r, bound, utc.span / 60.0, cm.need_ppm * cm.carrier / 1e6,
		     cm.trim_mode ? "; the trim steers it slowly" : "");
		return;
	}
	lower = fabs(utc.slope) * 1e6 - bound;
	if (cm.need && cm.need->utc_ms > 0.0 && !cm.warned_utc && lower > 0.0) {
		if (!own_line_allowed(now))
			return;
		cm.warned_utc = 1;
		minutes = cm.need->utc_ms * 1e-3 / (lower * 1e-6) / 60.0;
		LOGP(DSDR, LOGL_NOTICE, "Clock: the device runs %+.2f ppm ± %.2f against NTP's UTC: %s's cycles, set from UTC once at the start, pass ±%.0f ms of UTC (%s) after %.0f min at most. --sdr-ppm corrects the carrier only; a reference disciplined to GPS (10 MHz and PPS) keeps the cycles on UTC.\n",
		     utc.slope * 1e6, bound, cm.need->title, cm.need->utc_ms, cm.need->utc_ref, minutes);
		return;
	}
	if (!cm.trim_mode || cm.trim_failed || (cm.last_own && now - cm.last_own < OWN_LINE_EVERY_S))
		return;
	step = clock_trim_decide(&cm.tr, &utc, cm.eps, cm.static_ppm, cm.cd.t[(cm.cd.head + CLOCK_DRIFT_POINTS - 1) % CLOCK_DRIFT_POINTS]);
	if (step == 0.0)
		return;
	cm.last_own = now;
	if (cm.trim_mode == 1) {
		LOGP(DSDR, LOGL_INFO, "Clock: would trim the carrier by %+.3f ppm (%+.0f Hz) to %+.3f ppm: it is %+.3f ppm ± %.3f off against NTP's UTC over %.0f min (dry run, %d steps).\n",
		     step, step * cm.carrier / 1e6, cm.static_ppm + cm.tr.applied, cm.tr.residual, cm.tr.bound, utc.span / 60.0, cm.tr.steps);
		return;
	}
	if (sdr_set_tx_ppm(cm.static_ppm + cm.tr.applied) < 0) {
		cm.trim_failed = 1;
		cm.tr.applied -= step;
		LOGP(DSDR, LOGL_NOTICE, "Clock: the carrier can't be trimmed on this device; the trim is off.\n");
		return;
	}
	LOGP(DSDR, LOGL_INFO, "Clock: carrier trimmed by %+.3f ppm (%+.0f Hz) to %+.3f ppm: it was %+.3f ppm ± %.3f off against NTP's UTC over %.0f min (%d steps).\n",
	     step, step * cm.carrier / 1e6, cm.static_ppm + cm.tr.applied, cm.tr.residual, cm.tr.bound, utc.span / 60.0, cm.tr.steps);
}

void clock_monitor_poll(void)
{
	const sdr_clock_pair_t *p;
	struct timespec raw_now;
	double now = mono_now();
	int src, rc;

	if (!cm.on || now - cm.last_check < 1.0)
		return;
	cm.last_check = now;
	if (sdr_clock_status(&cm.st) < 0)
		return;
	if (!cm.started) {
		cm.started = 1;
		ntp_now();
		start_line();
	}
	/* the RX time stamps while they come (an RX stream stopped: the reads) */
	clock_gettime(CLOCK_MONOTONIC_RAW, &raw_now);
	if (cm.st.rx_time.seq && raw_now.tv_sec + raw_now.tv_nsec / 1e9 - cm.st.rx_time.raw < 5.0) {
		p = &cm.st.rx_time;
		src = 1;
	} else if (cm.st.poll_time.seq) {
		p = &cm.st.poll_time;
		src = 0;
	} else
		return;
	if (src != cm.src) {
		/* one yardstick for the whole history */
		clock_drift_init(&cm.cd);
		cm.src = src;
		cm.last_seq = 0;
	}
	if (p->seq == cm.last_seq || (cm.cd.n && p->raw - cm.last_point < POINT_EVERY_S - 0.5))
		return;
	cm.last_seq = p->seq;
	cm.last_point = p->raw;
	ntp_now();
	rc = clock_drift_add(&cm.cd, p->raw, p->device, p->system, cm.ntp);
	if (rc == 2)
		LOGP(DSDR, LOGL_NOTICE, "Clock: the device time jumped; its drift is measured again from now.\n");
	else if (rc == 1)
		LOGP(DSDR, LOGL_DEBUG, "Clock: NTP %s; the drift against UTC is measured again.\n", cm.ntp ? "stepped the PC's clock" : "lost its synchronisation");
	if (cm.log) {
		fprintf(cm.log, "%.6f %.9f %.9f %d %d %.9f %.9f %.6f %.1f %d %.6f\n", p->raw, p->device - p->raw, p->device - p->system,
			cm.ntp, src, cm.st.poll_time.seq ? cm.st.poll_time.device - cm.st.poll_time.raw : 0.0,
			cm.st.poll_time.seq ? cm.st.poll_time.device - cm.st.poll_time.system : 0.0,
			cm.st.poll_time.uncertainty, cm.st.temperature, cm.st.ref_locked, cm.st.poll_time.raw);
		fflush(cm.log);
	}
	evaluate(now);
}

void clock_monitor_report(void)
{
	if (!cm.on || !cm.started)
		return;
	status_line(LOGL_NOTICE, "");
}

void clock_monitor_stop(void)
{
	if (!cm.on)
		return;
	if (cm.started)
		status_line(LOGL_INFO, " at the end");
	if (cm.log)
		fclose(cm.log);
	cm.on = 0;
}

#endif
