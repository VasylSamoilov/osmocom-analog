#include <time.h>

double get_time(void);
void get_realtime(struct timespec *ts);

/* Sample clock: when every sample comes from and goes to a file, time is the
 * samples processed, not the wall clock, so a run is limited by the CPU and
 * gives the same result at any speed. */
void sample_clock_set_start(double seconds);
void sample_clock_start(void);
void sample_clock_advance(int samples, int samplerate);
int sample_clock_active(void);
