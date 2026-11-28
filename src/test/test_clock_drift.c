/* The clock monitor's drift fits and carrier trim decision, offline: synthetic series and
 * recorded ones (the NTP yardstick of a PC, the monitor's --sdr-clock-log).
 *
 * test_clock_drift                       synthetic cases (exit 1 on a failed check)
 * test_clock_drift --ntp <file>          the same with a recorded NTP series as UTC's error
 *                                        (columns: raw, realtime - raw, ...; ntp_rec.py)
 * test_clock_drift --replay <file> [static_ppm]   a --sdr-clock-log through the decision
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../libmobile/clock_drift.h"

static int failed;

static void check(int ok, const char *what)
{
	printf("  %s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok)
		failed++;
}

/* deterministic noise */
static unsigned long long rng = 88172645463325252ULL;

static double uniform(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 7;
	rng ^= rng << 17;
	return (rng >> 11) * (1.0 / 9007199254740992.0);
}

static double gauss(void)
{
	double u = uniform(), v = uniform();

	if (u < 1e-300)
		u = 1e-300;
	return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
}

/* a recorded NTP error: realtime - raw with its line taken out (what is left is NTP's wander and
 * the PC crystal's: more than NTP's alone, so a harder test) */
static int ntp_n;
static double *ntp_t, *ntp_e;

static int load_ntp(const char *path)
{
	FILE *f = fopen(path, "r");
	char line[512];
	double t, d, mx = 0, my = 0, sxx = 0, sxy = 0, a;
	int i, cap = 0, synced;

	if (!f)
		return -1;
	while (fgets(line, sizeof(line), f)) {
		int st, status, est;
		double freq, unc;

		if (sscanf(line, "%lf %lf %d %d %d %lf %d %lf", &t, &d, &st, &status, &est, &freq, &synced, &unc) < 7 || !synced)
			continue;
		if (ntp_n == cap) {
			cap = cap ? cap * 2 : 1024;
			ntp_t = realloc(ntp_t, cap * sizeof(double));
			ntp_e = realloc(ntp_e, cap * sizeof(double));
		}
		ntp_t[ntp_n] = t;
		ntp_e[ntp_n++] = d;
	}
	fclose(f);
	if (ntp_n < 10)
		return -1;
	for (i = 0; i < ntp_n; i++) {
		mx += ntp_t[i] - ntp_t[0];
		my += ntp_e[i] - ntp_e[0];
	}
	mx /= ntp_n;
	my /= ntp_n;
	for (i = 0; i < ntp_n; i++) {
		sxx += (ntp_t[i] - ntp_t[0] - mx) * (ntp_t[i] - ntp_t[0] - mx);
		sxy += (ntp_t[i] - ntp_t[0] - mx) * (ntp_e[i] - ntp_e[0] - my);
	}
	a = sxy / sxx;
	d = ntp_e[0];
	for (i = 0; i < ntp_n; i++)
		ntp_e[i] = (ntp_e[i] - d - my) - a * (ntp_t[i] - ntp_t[0] - mx);
	printf("NTP series: %d points over %.1f h, line %+.3f ppm (the PC crystal against NTP), wander after it: ", ntp_n, (ntp_t[ntp_n - 1] - ntp_t[0]) / 3600.0, a * 1e6);
	for (i = 0, mx = 0, my = 0; i < ntp_n; i++) {
		mx += ntp_e[i] * ntp_e[i];
		if (fabs(ntp_e[i]) > my)
			my = fabs(ntp_e[i]);
	}
	printf("rms %.3f ms, peak %.3f ms\n", sqrt(mx / ntp_n) * 1e3, my * 1e3);
	return 0;
}

/* the recorded error at t (s from the start), the series repeated (each lap shifted to join) */
static double ntp_recorded(double t)
{
	double span = ntp_t[ntp_n - 1] - ntp_t[0], x;
	int lap, i;

	lap = (int)floor(t / span);
	x = t - lap * span + ntp_t[0];
	for (i = 1; i < ntp_n - 1 && ntp_t[i] < x; i++);
	return ntp_e[i] - ((lap & 1) ? ntp_e[ntp_n - 1] - ntp_e[0] : 0.0);
}

typedef struct sim {
	const char	*name;
	double		hours;
	double		rate_ppm;	/* the device clock against true time (-4.4: 4.4 ppm slow) */
	double		rate_change_ppm;/* plus this, linear over the run (temperature) */
	double		kink_h;		/* from then on rate_kink_ppm instead (0: none) */
	double		rate_kink_ppm;
	double		static_ppm;	/* --sdr-ppm */
	double		ntp_amp_ms;	/* synthetic NTP wander: a 34 min swing of this size ... */
	double		ntp_white_ms;	/* ... and white noise */
	double		ntp_ramp_ms;	/* NTP's error from -this to +this over the run (the worst
					 * for the bound: a false rate of 2 eps / span) */
	int		recorded_ntp;	/* the recorded series instead */
	double		unsync_from_h, unsync_to_h;	/* NTP lost in between */
	double		step_at_h;	/* NTP steps the clock by 100 ms */
	double		jump_at_h;	/* the device time jumps by 1 s */
	double		eps;		/* NTP's error as the monitor assumes it, s */
	/* results */
	int		steps, reversals;
	double		first_step_h, applied, worst_overshoot, final_error;
	double		max_step;
	unsigned long	utc_restarts, device_restarts;
} sim_t;

static clock_drift_t cd;

static void run(sim_t *s)
{
	clock_trim_t tr;
	clock_fit_t utc;
	double t, dev = 0.0, last = 0.0, rate = s->rate_ppm, err, step, truth;
	int ntp;

	clock_drift_init(&cd);
	clock_trim_init(&tr);
	s->steps = s->reversals = 0;
	s->first_step_h = -1.0;
	s->worst_overshoot = 0.0;
	s->max_step = 0.0;
	for (t = 0.0; t <= s->hours * 3600.0; t += 10.0) {
		rate = s->rate_ppm + s->rate_change_ppm * t / (s->hours * 3600.0);
		if (s->kink_h > 0.0 && t >= s->kink_h * 3600.0)
			rate = s->rate_kink_ppm;
		dev += (t - last) * (1.0 + rate * 1e-6);
		last = t;
		if (s->jump_at_h > 0.0 && fabs(t - s->jump_at_h * 3600.0) < 5.0)
			dev += 1.0;
		/* the raw clock: the PC crystal 45 ppm fast; UTC: true time plus NTP's error */
		if (s->recorded_ntp)
			err = ntp_recorded(t);
		else
			err = s->ntp_amp_ms * 1e-3 * sin(2.0 * M_PI * t / 2048.0) + s->ntp_white_ms * 1e-3 * gauss();
		err += s->ntp_ramp_ms * 1e-3 * (2.0 * t / (s->hours * 3600.0) - 1.0);
		if (s->step_at_h > 0.0 && t >= s->step_at_h * 3600.0)
			err += 0.1;
		ntp = !(t >= s->unsync_from_h * 3600.0 && t < s->unsync_to_h * 3600.0);
		clock_drift_add(&cd, 1000.0 + t * (1.0 + 45e-6), 5000.0 + dev + 17e-6 * gauss(), 1.8e9 + t + err, ntp);
		if (clock_drift_fit_utc(&cd, &utc) < 0)
			continue;
		step = clock_trim_decide(&tr, &utc, s->eps, s->static_ppm, t);
		if (step == 0.0)
			continue;
		if (s->first_step_h < 0.0)
			s->first_step_h = t / 3600.0;
		if (fabs(step) > s->max_step)
			s->max_step = fabs(step);
		/* the carrier's true error after the step, and how far past 0 it went */
		truth = rate + s->static_ppm + tr.applied;
		if ((step > 0.0 && truth > s->worst_overshoot) || (step < 0.0 && -truth > s->worst_overshoot))
			s->worst_overshoot = fabs(truth);
	}
	s->steps = tr.steps;
	s->reversals = tr.reversals;
	s->applied = tr.applied;
	s->final_error = rate + s->static_ppm + tr.applied;
	s->utc_restarts = cd.utc_restarts;
	s->device_restarts = cd.device_restarts;
	printf("%s: %.1f h, device %+.2f ppm, --sdr-ppm %+.2f: %d steps (%+.3f ppm, the largest %.3f), first after %.2f h, %d reversals, carrier error at the end %+.3f ppm, the worst past 0 %.3f; UTC restarts %lu, device %lu\n",
	       s->name, s->hours, s->rate_ppm, s->static_ppm, s->steps, s->applied, s->max_step, s->first_step_h,
	       s->reversals, s->final_error, s->worst_overshoot, s->utc_restarts, s->device_restarts);
}

static void synthetic(int recorded)
{
	const char *tag = recorded ? " (recorded NTP)" : "";
	char name[128];
	sim_t s;

#define BASE(n) do { memset(&s, 0, sizeof(s)); snprintf(name, sizeof(name), "%s%s", n, tag); s.name = name; \
	s.hours = 5.5; s.eps = 5e-3; s.ntp_amp_ms = 2.0; s.ntp_white_ms = 0.5; s.recorded_ntp = recorded; } while (0)

	/* the B210 internal reference, not calibrated: 4.4 ppm slow */
	BASE("uncalibrated");
	s.rate_ppm = -4.4;
	run(&s);
	check(s.steps > 0, "an uncalibrated 4.4 ppm gets trimmed");
	check(s.first_step_h >= 0.5, "not before 30 min");
	check(s.max_step <= 0.1 + 1e-12, "steps of at most 0.1 ppm");
	check(s.reversals == 0, "no reversal");
	check(s.worst_overshoot < 0.05, "never past 0");
	check(s.applied > 0.0, "toward the right side (the clock is slow: +ppm)");

	/* calibrated (osmocalibrate): 4.4 ppm slow, --sdr-ppm 4.4 */
	BASE("calibrated");
	s.rate_ppm = -4.4;
	s.static_ppm = 4.4;
	run(&s);
	check(s.steps == 0, "calibrated: no trim");

	/* a small residual: NTP can't confirm it */
	BASE("residual 0.3 ppm");
	s.rate_ppm = -4.4;
	s.static_ppm = 4.1;
	run(&s);
	check(s.steps == 0, "0.3 ppm off: no trim (within NTP's bound)");

	/* NTP lost for an hour: the UTC run starts again */
	BASE("NTP lost 1-2 h");
	s.rate_ppm = -4.4;
	s.unsync_from_h = 1.0;
	s.unsync_to_h = 2.0;
	run(&s);
	check(s.utc_restarts >= 1, "the UTC run restarted");
	check(s.first_step_h < 0.0 || s.first_step_h < 1.0 || s.first_step_h >= 2.5, "no step within 30 min after NTP came back");
	check(s.reversals == 0, "no reversal");

	/* NTP steps the PC clock by 100 ms */
	BASE("NTP step");
	s.rate_ppm = -4.4;
	s.static_ppm = 4.4;
	s.step_at_h = 2.0;
	run(&s);
	check(s.utc_restarts == 1, "the step seen once");
	check(s.steps == 0, "a step of the PC clock makes no trim");

	/* the device time jumps */
	BASE("device jump");
	s.rate_ppm = -4.4;
	s.static_ppm = 4.4;
	s.jump_at_h = 1.0;
	run(&s);
	check(s.device_restarts == 1, "the jump seen once");
	check(s.steps == 0, "no trim");

	/* warming up: the rate moves by 1 ppm over the run */
	BASE("ramp");
	s.rate_ppm = -4.4;
	s.rate_change_ppm = 1.0;
	run(&s);
	check(s.reversals == 0, "a ramp: no reversal");
	check(s.worst_overshoot < 0.05, "never past 0");

	/* a kink: once trimmed toward 4.4, the clock jumps to 2.0 ppm slow */
	BASE("kink");
	s.hours = 5.5;
	s.rate_ppm = -4.4;
	s.static_ppm = 4.0;
	s.kink_h = 2.0;
	s.rate_kink_ppm = -1.0;
	run(&s);
	check(s.reversals <= 1, "a kink: at most one reversal");

	/* the B210 on its bad external reference (14.8 ppm slow) */
	BASE("external 14.8");
	s.rate_ppm = -14.8;
	run(&s);
	check(s.steps > 0 && s.reversals == 0 && s.max_step <= 0.1 + 1e-12, "14.8 ppm: slow steps, no reversal");

	/* a fast clock: steps the other way */
	/* NTP's error as a ramp over the run, the whole eps at each end, nothing off: the worst
	 * case of the bound */
	BASE("NTP ramp");
	s.rate_ppm = -4.4;
	s.static_ppm = 4.4;
	s.ntp_ramp_ms = 5.0;
	run(&s);
	check(s.steps == 0, "NTP's error as a ramp of ±eps: no trim");

	/* 8 h: the ring (5.7 h) wraps */
	BASE("8 h");
	s.hours = 8.0;
	s.rate_ppm = -4.4;
	run(&s);
	check(s.steps > 0 && s.reversals == 0 && s.worst_overshoot < 0.05 && s.max_step <= 0.1 + 1e-12, "8 h, the history wraps: steps, no reversal, never past 0");
	BASE("8 h calibrated");
	s.hours = 8.0;
	s.rate_ppm = -4.4;
	s.static_ppm = 4.4;
	run(&s);
	check(s.steps == 0, "8 h calibrated: no trim");

	BASE("fast 3 ppm");
	s.rate_ppm = 3.0;
	run(&s);
	check(s.steps > 0 && s.applied < 0.0 && s.reversals == 0, "a fast clock: -ppm steps, no reversal");
}

/* a --sdr-clock-log through the fits and the decision */
static int replay(const char *path, double static_ppm)
{
	FILE *f = fopen(path, "r");
	char line[512];
	clock_trim_t tr;
	clock_fit_t raw, utc, praw;
	clock_drift_t *pc = calloc(1, sizeof(*pc));
	double t, dr, du, pr, pu, unc, temp, step, praw_t;
	int ntp, src, ref, n = 0;

	if (!f || !pc)
		return -1;
	clock_drift_init(&cd);
	clock_drift_init(pc);
	clock_trim_init(&tr);
	while (fgets(line, sizeof(line), f)) {
		if (line[0] == '#' || sscanf(line, "%lf %lf %lf %d %d %lf %lf %lf %lf %d", &t, &dr, &du, &ntp, &src, &pr, &pu, &unc, &temp, &ref) < 10)
			continue;
		n++;
		clock_drift_add(&cd, t, t + dr, t + dr - du, ntp);
		/* the device time reads beside the RX time stamps, at their own raw time (older logs
		 * have none: skipped) */
		if (pr != 0.0 && sscanf(line, "%*f %*f %*f %*d %*d %*f %*f %*f %*f %*d %lf", &praw_t) == 1)
			clock_drift_add(pc, praw_t, praw_t + pr, praw_t + pr - pu, ntp);
		if (clock_drift_fit_utc(&cd, &utc) < 0)
			continue;
		step = clock_trim_decide(&tr, &utc, 5e-3, static_ppm, t);
		if (step != 0.0)
			printf("  %.0f s: would trim by %+.3f ppm (residual %+.3f ± %.3f)\n", t - cd.t[0], step, tr.residual, tr.bound);
	}
	fclose(f);
	if (clock_drift_fit_raw(&cd, &raw) < 0 || clock_drift_fit_utc(&cd, &utc) < 0) {
		printf("%s: too few points (%d)\n", path, n);
		return 0;
	}
	printf("%s: %d points over %.0f s; against raw %+.3f ppm ± %.3f (scatter %.4f ms), against UTC %+.3f ppm ± %.3f (scatter %.4f ms, NTP bound ± %.3f); %d steps\n",
	       path, raw.n, raw.span, raw.slope * 1e6, 3 * raw.se * 1e6, raw.scatter * 1e3, utc.slope * 1e6, 3 * utc.se * 1e6,
	       utc.scatter * 1e3, clock_fit_utc_bound(&utc, 5e-3), tr.steps);
	if (clock_drift_fit_raw(pc, &praw) == 0)
		printf("  the device time reads beside them: against raw %+.3f ppm ± %.3f (scatter %.4f ms)\n",
		       praw.slope * 1e6, 3 * praw.se * 1e6, praw.scatter * 1e3);
	free(pc);
	return 0;
}

int main(int argc, char *argv[])
{
	if (argc >= 3 && !strcmp(argv[1], "--replay"))
		return replay(argv[2], argc >= 4 ? atof(argv[3]) : 0.0) < 0;
	printf("Synthetic NTP (a 2 ms swing of 34 min, 0.5 ms white), 5 ms assumed:\n");
	synthetic(0);
	if (argc >= 3 && !strcmp(argv[1], "--ntp")) {
		if (load_ntp(argv[2]) < 0) {
			fprintf(stderr, "%s: no NTP series\n", argv[2]);
			return 1;
		}
		synthetic(1);
	}
	printf("%s\n", failed ? "FAILED" : "all ok");
	return failed ? 1 : 0;
}
