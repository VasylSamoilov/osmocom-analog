/* Audio Quality Debug - Track levels, clipping, resampling, and sample drops
 *
 * RX-focused: traces signal through entire demodulation chain
 * Enable with: -d DRADIO:DEBUG
 */

#ifndef AUDIO_DEBUG_H
#define AUDIO_DEBUG_H

#include <stdint.h>
#include <stdbool.h>

/* Stage identifiers for RX pipeline */
typedef enum {
	RX_STAGE_SDR_RAW,       /* Raw IQ from SDR */
	RX_STAGE_CHANNELIZER,   /* After channelizer/downconvert */
	RX_STAGE_FM_DEMOD,      /* After FM demodulation */
	RX_STAGE_STEREO_DECODE, /* After stereo decoding */
	RX_STAGE_DEEMPHASIS,    /* After de-emphasis */
	RX_STAGE_RESAMPLE,      /* After resampling to audio rate */
	RX_STAGE_AUDIO_OUT,     /* Final audio output */
	RX_STAGE_COUNT
} rx_stage_t;

/* Per-stage statistics */
typedef struct stage_stats {
	uint64_t sample_count;
	double peak_pos;           /* max positive value */
	double peak_neg;           /* max negative value (abs) */
	double sum_squares;        /* for RMS calculation */
	uint64_t clip_count;       /* samples >= 1.0 (overmod for FM_DEMOD, clip for audio) */
	double last_sample;        /* for discontinuity detection */
	uint64_t discontinuities;  /* sudden jumps */
} stage_stats_t;

/* Resampler statistics */
typedef struct resample_stats {
	uint64_t input_samples;
	uint64_t output_samples;
	double ratio_expected;
	double ratio_actual;       /* computed from in/out counts */
	uint64_t ratio_errors;     /* times ratio deviated significantly */
} resample_stats_t;

/* AFC (Automatic Frequency Control) statistics
 * 
 * FLL (Frequency-Locked Loop): Measures carrier offset by tracking average
 * phase rotation rate of IQ samples. Works for mono and stereo FM.
 * 
 * DC method: Legacy approach measuring DC offset after FM demod. Unreliable
 * because FM audio content creates DC bias unrelated to carrier offset.
 * 
 * Pilot accuracy: Measures how accurate the station's 19 kHz pilot is.
 * NOT useful for AFC - the pilot is always at 19 kHz in baseband regardless
 * of carrier offset. Useful for diagnosing station transmitter issues.
 */
typedef struct afc_stats {
	double dc_offset;          /* Current DC offset (normalized) - debug only */
	double freq_error_hz;      /* FLL frequency error = carrier offset (Hz) */
	double correction_hz;      /* Applied NCO correction (Hz) */
	double dc_freq_error_hz;   /* DC method error (debug only, unreliable) */
	double pilot_accuracy_hz;  /* 19 kHz pilot deviation from nominal (station TX accuracy) */
	int using_pll_method;      /* Unused, kept for compatibility */
	int pll_locked;            /* Stereo pilot PLL lock status */
	double peak_error_hz;      /* Peak FLL error in reporting period */
	uint64_t update_count;     /* Number of AFC updates */
} afc_stats_t;

/* Main debug structure */
typedef struct audio_debug {
	/* Per-stage tracking */
	stage_stats_t stages[RX_STAGE_COUNT];
	
	/* Resampling tracking */
	resample_stats_t resample;
	
	/* AFC tracking */
	afc_stats_t afc;
	
	/* Sample continuity */
	uint64_t expected_samples;
	uint64_t received_samples;
	uint64_t gap_events;
	uint64_t gap_samples_total;
	
	/* Jitter buffer */
	uint64_t jitter_underruns;
	uint64_t jitter_overruns;
	
	/* Timing */
	double last_report_time;
	double report_interval_sec;
	
	/* Enable flag */
	int enabled;
} audio_debug_t;

/* Global debug instance (for easy access) */
extern audio_debug_t g_rx_debug;

/* Initialize */
void audio_debug_init(audio_debug_t *dbg, double report_interval);

/* Track samples at a specific stage */
void audio_debug_stage(audio_debug_t *dbg, rx_stage_t stage, 
                       const double *samples, int num);

/* Track resampler input/output */
void audio_debug_resample(audio_debug_t *dbg, int in_samples, int out_samples,
                          double expected_ratio);

/* Track sample gap */
void audio_debug_gap(audio_debug_t *dbg, int expected, int actual);

/* Jitter events */
void audio_debug_jitter_underrun(audio_debug_t *dbg);
void audio_debug_jitter_overrun(audio_debug_t *dbg);

/* AFC tracking
 * dc_offset: DC offset in FM demod output (debug only)
 * freq_error_hz: FLL carrier offset measurement (primary AFC source)
 * correction_hz: NCO correction being applied
 * dc_freq_error_hz: DC method estimate (unreliable, debug only)
 * pilot_accuracy_hz: 19 kHz pilot deviation from nominal (station TX quality)
 * using_pll_method: unused, kept for API compatibility
 * pll_locked: stereo pilot lock status
 */
void audio_debug_afc(audio_debug_t *dbg, double dc_offset, double freq_error_hz, 
                     double correction_hz, double dc_freq_error_hz,
                     double pilot_accuracy_hz, int using_pll_method, int pll_locked);

/* Periodic report - call every block, rate-limited internally */
void audio_debug_report(audio_debug_t *dbg);

/* Force immediate report */
void audio_debug_report_now(audio_debug_t *dbg);

/* Reset all stats */
void audio_debug_reset(audio_debug_t *dbg);

/* Enable/disable */
void audio_debug_enable(audio_debug_t *dbg, int enable);

#endif /* AUDIO_DEBUG_H */
