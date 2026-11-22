#include "../libfilter/iir_filter.h"

typedef struct emphasis {
	struct {
		iir_filter_t lp;
		double x_last;
		double factor;
		double amp;
	} p;
	struct {
		iir_filter_t hp;
		double y_last;
		double factor;
		double amp;
	} d;
} emphasis_t;

/* refers to NMT specs, cnetz uses different emphasis cutoff */
#define CUT_OFF_EMPHASIS_DEFAULT 300.0
#define CUT_OFF_HIGHPASS_DEFAULT 300.0
#define CUT_OFF_LOWPASS_DEFAULT 3400.0

double timeconstant2cutoff(double time_constant_us);
int init_emphasis(emphasis_t *state, int samplerate, double cut_off, double cut_off_h, double cut_off_l);
void pre_emphasis(emphasis_t *state, sample_t *samples, int num);
void de_emphasis(emphasis_t *state, sample_t *samples, int num);
void dc_filter(emphasis_t *state, sample_t *samples, int num);

/*
 * Optimized 1st-order IIR emphasis filter for FM broadcast.
 * Based on SDRangel's FMPreemphasis (bilinear transform, Direct Form II).
 * ~3x more efficient than emphasis_t for broadcast FM applications.
 *
 * Note: This is for FM broadcast only. Mobile networks (NMT, AMPS, etc.)
 * use the original emphasis_t which has different rolloff characteristics.
 */

#define EMPHASIS_TAU_EU  50e-6   /* Europe: 50µs */
#define EMPHASIS_TAU_US  75e-6   /* USA/Japan: 75µs */

typedef struct emphasis_fast {
	float b0, b1, a1;   /* IIR coefficients */
	float z;            /* Single delay state */
} emphasis_fast_t;

/* Initialize FM broadcast emphasis filter
 * samplerate: Audio sample rate (Hz)
 * tau: Time constant (use EMPHASIS_TAU_EU or EMPHASIS_TAU_US)
 * high_freq: High frequency corner for flattening (typically 12000-15000 Hz) */
void init_emphasis_fast(emphasis_fast_t *e, int samplerate, double tau, double high_freq);

/* Apply pre-emphasis (TX) - boosts high frequencies before transmission */
void pre_emphasis_fast(emphasis_fast_t *e, sample_t *samples, int num);

/* Apply de-emphasis (RX) - restores flat frequency response after reception */
void de_emphasis_fast(emphasis_fast_t *e, sample_t *samples, int num);

/*
 * Simple 1st-order highpass DC filter for RX audio.
 * Removes DC offset from FM demodulator output.
 * Cutoff typically 20-30 Hz (very low to avoid affecting bass).
 */
typedef struct dc_filter_fast {
	float alpha;        /* Filter coefficient (0 < alpha < 1) */
	float prev_in;      /* Previous input sample */
	float prev_out;     /* Previous output sample */
} dc_filter_fast_t;

/* Initialize DC blocking filter
 * samplerate: Audio sample rate (Hz)
 * cutoff_hz: Cutoff frequency in Hz (typically 20-30 Hz) */
void init_dc_filter_fast(dc_filter_fast_t *f, int samplerate, double cutoff_hz);

/* Apply DC blocking filter - removes DC offset from samples */
void dc_filter_fast(dc_filter_fast_t *f, sample_t *samples, int num);
