/* Audio Quality Debug - Track levels, clipping, resampling, and sample drops
 *
 * RX-focused: traces signal through entire demodulation chain
 * Enable with: -DAUDIO_DEBUG in CFLAGS
 */

#ifndef AUDIO_DEBUG_H
#define AUDIO_DEBUG_H

#include <stdint.h>

#ifdef AUDIO_DEBUG

typedef enum {
RX_STAGE_SDR_RAW,
RX_STAGE_CHANNELIZER,
RX_STAGE_FM_DEMOD,
RX_STAGE_STEREO_DECODE,
RX_STAGE_DEEMPHASIS,
RX_STAGE_RESAMPLE,
RX_STAGE_AUDIO_OUT,
RX_STAGE_COUNT
} rx_stage_t;

typedef struct stage_stats {
uint64_t sample_count;
double peak_pos, peak_neg;
double sum_squares;
uint64_t clip_count;
double last_sample;
uint64_t discontinuities;
} stage_stats_t;

typedef struct resample_stats {
uint64_t input_samples, output_samples;
double ratio_expected, ratio_actual;
uint64_t ratio_errors;
} resample_stats_t;

typedef struct fm_hf_stats {
double pre_peak, pre_rms, pre_hf;
double post_peak, post_rms, post_hf;
double out_peak, out_rms, out_hf;
double signal_samplerate, audio_samplerate;
double stereo_blend, stereo_hf_gain;
int stereo, valid;
} fm_hf_stats_t;

typedef struct afc_stats {
double freq_error_hz, correction_hz;
double pilot_accuracy_hz;
int using_pll_method, pll_locked;
double peak_error_hz;
uint64_t update_count;
} afc_stats_t;

typedef struct audio_debug {
stage_stats_t stages[RX_STAGE_COUNT];
resample_stats_t resample;
afc_stats_t afc;
fm_hf_stats_t fm_hf;
uint64_t expected_samples, received_samples;
uint64_t gap_events, gap_samples_total;
uint64_t jitter_underruns, jitter_overruns;
double last_report_time, report_interval_sec;
int enabled;
} audio_debug_t;

extern audio_debug_t g_rx_debug;

void audio_debug_init(audio_debug_t *dbg, double report_interval);
void audio_debug_enable(audio_debug_t *dbg, int enable);
void audio_debug_stage(audio_debug_t *dbg, rx_stage_t stage, const double *samples, int num);
void audio_debug_resample(audio_debug_t *dbg, int in_samples, int out_samples, double expected_ratio);
void audio_debug_gap(audio_debug_t *dbg, int expected, int actual);
void audio_debug_jitter_underrun(audio_debug_t *dbg);
void audio_debug_jitter_overrun(audio_debug_t *dbg);
void audio_debug_afc(audio_debug_t *dbg, double freq_error_hz, double correction_hz,
                     double pilot_accuracy_hz, int using_pll_method, int pll_locked);
void audio_debug_fm_hf(audio_debug_t *dbg,
                       double pre_peak, double pre_rms, double pre_hf,
                       double post_peak, double post_rms, double post_hf,
                       double out_peak, double out_rms, double out_hf,
                       double signal_samplerate, double audio_samplerate,
                       double stereo_blend, double stereo_hf_gain, int stereo);
void audio_debug_report(audio_debug_t *dbg);
void audio_debug_report_now(audio_debug_t *dbg);
void audio_debug_reset(audio_debug_t *dbg);

#else /* !AUDIO_DEBUG */

typedef struct audio_debug { int _dummy; } audio_debug_t;
typedef enum { RX_STAGE_SDR_RAW=0, RX_STAGE_CHANNELIZER, RX_STAGE_FM_DEMOD,
               RX_STAGE_STEREO_DECODE, RX_STAGE_DEEMPHASIS, RX_STAGE_RESAMPLE,
               RX_STAGE_AUDIO_OUT, RX_STAGE_COUNT } rx_stage_t;
extern audio_debug_t g_rx_debug;
static inline void audio_debug_init(audio_debug_t *d, double i) { (void)d; (void)i; }
static inline void audio_debug_enable(audio_debug_t *d, int e) { (void)d; (void)e; }
static inline void audio_debug_stage(audio_debug_t *d, rx_stage_t s, const double *b, int n) { (void)d; (void)s; (void)b; (void)n; }
static inline void audio_debug_resample(audio_debug_t *d, int i, int o, double r) { (void)d; (void)i; (void)o; (void)r; }
static inline void audio_debug_gap(audio_debug_t *d, int e, int a) { (void)d; (void)e; (void)a; }
static inline void audio_debug_jitter_underrun(audio_debug_t *d) { (void)d; }
static inline void audio_debug_jitter_overrun(audio_debug_t *d) { (void)d; }
static inline void audio_debug_afc(audio_debug_t *d, double a, double b, double c, int e, int f) { (void)d; (void)a; (void)b; (void)c; (void)e; (void)f; }
static inline void audio_debug_fm_hf(audio_debug_t *d, double a, double b, double c, double e, double f, double g, double h, double ii, double j, double k, double l, double m, double n, int o) { (void)d; (void)a; (void)b; (void)c; (void)e; (void)f; (void)g; (void)h; (void)ii; (void)j; (void)k; (void)l; (void)m; (void)n; (void)o; }
static inline void audio_debug_report(audio_debug_t *d) { (void)d; }
static inline void audio_debug_report_now(audio_debug_t *d) { (void)d; }
static inline void audio_debug_reset(audio_debug_t *d) { (void)d; }

#endif /* AUDIO_DEBUG */

#endif /* AUDIO_DEBUG_H */
