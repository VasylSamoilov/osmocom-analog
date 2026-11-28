#ifndef CLOCK_DRIFT_H
#define CLOCK_DRIFT_H

/* The device clock's drift against the PC's clocks: a history of points and least-squares fits,
 * and the decision of a carrier trim. No I/O (the clock monitor feeds it; a test can too). */

#define CLOCK_DRIFT_POINTS	2048	/* at 10 s: 5.7 h */

typedef struct clock_drift {
	int		n, head;		/* points in the ring, the next slot */
	double		t[CLOCK_DRIFT_POINTS];	/* CLOCK_MONOTONIC_RAW, s */
	double		d_raw[CLOCK_DRIFT_POINTS];	/* device - raw */
	double		d_utc[CLOCK_DRIFT_POINTS];	/* device - CLOCK_REALTIME (NTP's UTC) */
	int		utc_n;			/* the newest utc_n points: NTP synchronised, no step */
	unsigned long	utc_restarts;		/* sync lost or a step of the PC's clock */
	unsigned long	device_restarts;	/* the device time jumped: all points dropped */
} clock_drift_t;

typedef struct clock_fit {
	int		n;
	double		span;		/* s, the first to the last point */
	double		slope;		/* the device's rate against the yardstick: 1e-6 = +1 ppm fast */
	double		se;		/* the slope's standard error (white residuals) */
	double		scatter;	/* the points around the line, s (rms) */
} clock_fit_t;

void clock_drift_init(clock_drift_t *cd);
/* a point: the device time and the two system clocks at the same moment; ntp: CLOCK_REALTIME is
 * synchronised now. Returns 1 when the UTC run restarted (sync lost, a step), 2 when the device
 * time jumped (history dropped), else 0. */
int clock_drift_add(clock_drift_t *cd, double raw, double device, double utc, int ntp);
/* least squares over all points (raw) / the UTC run; -1: fewer than 3 points */
int clock_drift_fit_raw(const clock_drift_t *cd, clock_fit_t *f);
int clock_drift_fit_utc(const clock_drift_t *cd, clock_fit_t *f);
/* the UTC run's change of device - UTC since its first point (s): how far a grid set from UTC at
 * that moment and counted in device samples is off UTC now */
double clock_drift_utc_walk(const clock_drift_t *cd);

/* The uncertainty of a UTC fit as a bound, ppm: 3 standard errors plus what NTP's own error
 * (eps, s: at both ends of the span) can do to the slope */
double clock_fit_utc_bound(const clock_fit_t *f, double eps);

/* The carrier trim: open loop - a trim changes the carrier, not the device clock that is measured,
 * so the residual is the measured offset plus all the corrections. Very slow: steps of at most
 * max_step, at least min_interval apart, only after min_span of NTP synchronised points, only
 * when the residual is certain beyond the step (no step can make it worse), and against the last
 * step's direction only with twice the margin (no oscillation). */
typedef struct clock_trim {
	double		min_span;	/* s (1800) */
	double		max_step;	/* ppm (0.1) */
	double		min_interval;	/* s (600) */
	double		applied;	/* ppm, the steps so far (on top of --sdr-ppm) */
	double		last_t;		/* when the last step was, raw s */
	int		last_sign;	/* its direction, 0: none yet */
	int		steps, reversals;
	double		residual, bound;	/* of the last decision, ppm */
} clock_trim_t;

void clock_trim_init(clock_trim_t *tr);
/* the step to take now (ppm, 0: none). utc: the fit against UTC; eps: NTP's error (s);
 * static_ppm: --sdr-ppm; now: raw s. The carrier's error is device rate + static + applied
 * (the frequency asked for is f * (1 + ppm), the device makes it (1 + rate) of that). */
double clock_trim_decide(clock_trim_t *tr, const clock_fit_t *utc, double eps, double static_ppm, double now);

#endif
