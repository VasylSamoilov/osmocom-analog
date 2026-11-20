#include <time.h>
#include <stdbool.h>
#include <sys/time.h>
#include <osmocom/core/timer.h>

#include "get_time.h"

/* Sample clock state: the real time when it started, and the samples
 * processed since then (as seconds, exact for any sample rate). */
static int sample_clock_on = 0;
static struct timespec sample_clock_start_real;
static struct timespec sample_clock_start_mono;
static double sample_clock_elapsed = 0.0;

static void timespec_add(struct timespec *out, const struct timespec *base, double seconds)
{
	long long nsec = (long long)base->tv_nsec + (long long)((seconds - (double)(long long)seconds) * 1e9);

	out->tv_sec = base->tv_sec + (time_t)(long long)seconds;
	while (nsec >= 1000000000LL) {
		nsec -= 1000000000LL;
		out->tv_sec++;
	}
	out->tv_nsec = (long)nsec;
}

/* Keep the libosmocore clocks (its timers) on the sample clock */
static void sample_clock_update_osmo(void)
{
	struct timespec *mono = osmo_clock_override_gettimespec(CLOCK_MONOTONIC);
	struct timespec *real = osmo_clock_override_gettimespec(CLOCK_REALTIME);
	struct timespec now;

	if (mono)
		timespec_add(mono, &sample_clock_start_mono, sample_clock_elapsed);
	if (real)
		timespec_add(real, &sample_clock_start_real, sample_clock_elapsed);
	timespec_add(&now, &sample_clock_start_real, sample_clock_elapsed);
	osmo_gettimeofday_override_time.tv_sec = now.tv_sec;
	osmo_gettimeofday_override_time.tv_usec = now.tv_nsec / 1000;
}

void sample_clock_start(void)
{
	if (sample_clock_on)
		return;
	clock_gettime(CLOCK_REALTIME, &sample_clock_start_real);
	clock_gettime(CLOCK_MONOTONIC, &sample_clock_start_mono);
	sample_clock_elapsed = 0.0;
	osmo_clock_override_enable(CLOCK_MONOTONIC, true);
	osmo_clock_override_enable(CLOCK_REALTIME, true);
	osmo_gettimeofday_override = true;
	sample_clock_on = 1;
	sample_clock_update_osmo();
}

void sample_clock_advance(int samples, int samplerate)
{
	if (!sample_clock_on || samples <= 0 || samplerate <= 0)
		return;
	sample_clock_elapsed += (double)samples / (double)samplerate;
	sample_clock_update_osmo();
}

int sample_clock_active(void)
{
	return sample_clock_on;
}

/* Real time (UTC), or the sample clock's time of day while it runs */
void get_realtime(struct timespec *ts)
{
	if (sample_clock_on) {
		timespec_add(ts, &sample_clock_start_real, sample_clock_elapsed);
		return;
	}
	clock_gettime(CLOCK_REALTIME, ts);
}

double get_time(void)
{
	static struct timespec tv;

	if (sample_clock_on) {
		return (double)sample_clock_start_real.tv_sec + (double)sample_clock_start_real.tv_nsec / 1000000000.0
			+ sample_clock_elapsed;
	}

	clock_gettime(CLOCK_REALTIME, &tv);

	return (double)tv.tv_sec + (double)tv.tv_nsec / 1000000000.0;
}
