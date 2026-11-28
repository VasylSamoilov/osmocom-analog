#ifndef CLOCK_MONITOR_H
#define CLOCK_MONITOR_H

/* The NTP state of the system clock (ntp_adjtime; systemd-timesyncd marks a synchronised clock
 * with a file but sets no error estimate): 1 synchronised, 0 not, -1 unknown. *esterror: the
 * kernel's estimate (s), or -1 when there is none (timesyncd); how: "NTP", "systemd-timesyncd" */
int clock_ntp_state(double *esterror, const char **how);

/* The SDR device's clock against the PC's raw clock and NTP's UTC, for all programs: one status
 * line at the start, a warning only when a need the protocol declares can't be met (with what to
 * do), the rest on demand (SIGUSR1) and at the end. protocol: main_mobile_loop()'s name, which
 * picks the needs; carrier: the TX frequency (Hz, 0: none). */
void clock_monitor_start(const char *protocol, double carrier);
/* from the main loop (returns at once unless a second passed) */
void clock_monitor_poll(void);
/* the status line now (on demand) */
void clock_monitor_report(void);
void clock_monitor_stop(void);

#endif
