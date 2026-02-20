/* Audio Quality Debug Implementation
 *
 * RX-focused: traces signal levels through entire demodulation chain
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "../liblogging/logging.h"
#include "audio_debug.h"

/* Global instance */
audio_debug_t g_rx_debug;

static const char *stage_names[] = {
	"SDR_RAW",
	"CHANNELIZER", 
	"FM_DEMOD",
	"STEREO_DEC",
	"DEEMPH",
	"RESAMPLE",
	"AUDIO_OUT"
};

static double get_time_sec(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

void audio_debug_init(audio_debug_t *dbg, double report_interval)
{
	memset(dbg, 0, sizeof(*dbg));
	dbg->report_interval_sec = report_interval > 0 ? report_interval : 5.0;
	dbg->last_report_time = get_time_sec();
	dbg->enabled = 1;
}

void audio_debug_enable(audio_debug_t *dbg, int enable)
{
	dbg->enabled = enable;
}

void audio_debug_stage(audio_debug_t *dbg, rx_stage_t stage, 
                       const double *samples, int num)
{
	if (!dbg->enabled || stage >= RX_STAGE_COUNT || num <= 0)
		return;
	
	stage_stats_t *s = &dbg->stages[stage];
	
	for (int i = 0; i < num; i++) {
		double val = samples[i];
		double absval = fabs(val);
		
		/* Track peaks */
		if (val > s->peak_pos)
			s->peak_pos = val;
		if (val < 0 && absval > s->peak_neg)
			s->peak_neg = absval;
		
		/* RMS accumulator */
		s->sum_squares += val * val;
		
		/* Clipping (>= 1.0) */
		if (absval >= 1.0)
			s->clip_count++;
		
		/* Discontinuity detection: jump > 0.5 between consecutive samples */
		if (s->sample_count > 0) {
			double delta = fabs(val - s->last_sample);
			if (delta > 0.5)
				s->discontinuities++;
		}
		s->last_sample = val;
	}
	
	s->sample_count += num;
}

void audio_debug_resample(audio_debug_t *dbg, int in_samples, int out_samples,
                          double expected_ratio)
{
	if (!dbg->enabled)
		return;
	
	dbg->resample.input_samples += in_samples;
	dbg->resample.output_samples += out_samples;
	dbg->resample.ratio_expected = expected_ratio;
	
	if (in_samples > 0) {
		double actual = (double)out_samples / in_samples;
		dbg->resample.ratio_actual = actual;
		
		/* Check for significant ratio deviation (> 5%) */
		if (expected_ratio > 0) {
			double error = fabs(actual - expected_ratio) / expected_ratio;
			if (error > 0.05) {
				dbg->resample.ratio_errors++;
				LOGP(DRADIO, LOGL_DEBUG, 
				     "RESAMPLE: ratio error %.3f vs expected %.3f (%.1f%% off)\n",
				     actual, expected_ratio, error * 100);
			}
		}
	}
}

void audio_debug_gap(audio_debug_t *dbg, int expected, int actual)
{
	if (!dbg->enabled)
		return;
	
	dbg->expected_samples += expected;
	dbg->received_samples += actual;
	
	if (actual < expected) {
		int gap = expected - actual;
		dbg->gap_events++;
		dbg->gap_samples_total += gap;
		
		LOGP(DRADIO, LOGL_NOTICE, 
		     "RX GAP: expected %d, got %d (lost %d, total gaps: %lu)\n",
		     expected, actual, gap, (unsigned long)dbg->gap_events);
	}
}

void audio_debug_jitter_underrun(audio_debug_t *dbg)
{
	if (!dbg->enabled)
		return;
	dbg->jitter_underruns++;
	LOGP(DRADIO, LOGL_NOTICE, "RX JITTER UNDERRUN #%lu\n", 
	     (unsigned long)dbg->jitter_underruns);
}

void audio_debug_jitter_overrun(audio_debug_t *dbg)
{
	if (!dbg->enabled)
		return;
	dbg->jitter_overruns++;
	LOGP(DRADIO, LOGL_NOTICE, "RX JITTER OVERRUN #%lu\n",
	     (unsigned long)dbg->jitter_overruns);
}

void audio_debug_report(audio_debug_t *dbg)
{
	if (!dbg->enabled)
		return;
	
	double now = get_time_sec();
	if (now - dbg->last_report_time < dbg->report_interval_sec)
		return;
	
	audio_debug_report_now(dbg);
	dbg->last_report_time = now;
}

void audio_debug_report_now(audio_debug_t *dbg)
{
	if (!dbg->enabled)
		return;
	
	LOGP(DRADIO, LOGL_INFO, "=== RX AUDIO DEBUG REPORT ===\n");
	
	/* Per-stage stats */
	for (int i = 0; i < RX_STAGE_COUNT; i++) {
		stage_stats_t *s = &dbg->stages[i];
		if (s->sample_count == 0)
			continue;
		
		double rms = sqrt(s->sum_squares / s->sample_count);
		double peak = (s->peak_pos > s->peak_neg) ? s->peak_pos : s->peak_neg;
		double peak_db = (peak > 0) ? 20.0 * log10(peak) : -999;
		double rms_db = (rms > 0) ? 20.0 * log10(rms) : -999;
		
		LOGP(DRADIO, LOGL_INFO,
		     "  %-12s: peak=%.4f (%+.1fdB) rms=%.4f (%+.1fdB) clip=%lu disc=%lu n=%.1fM\n",
		     stage_names[i],
		     peak, peak_db,
		     rms, rms_db,
		     (unsigned long)s->clip_count,
		     (unsigned long)s->discontinuities,
		     s->sample_count / 1e6);
	}
	
	/* Resampler stats */
	if (dbg->resample.input_samples > 0) {
		double actual_ratio = (double)dbg->resample.output_samples / 
		                      dbg->resample.input_samples;
		LOGP(DRADIO, LOGL_INFO,
		     "  RESAMPLE: in=%luM out=%luM ratio=%.6f (expect %.6f) errors=%lu\n",
		     (unsigned long)(dbg->resample.input_samples / 1000000),
		     (unsigned long)(dbg->resample.output_samples / 1000000),
		     actual_ratio,
		     dbg->resample.ratio_expected,
		     (unsigned long)dbg->resample.ratio_errors);
	}
	
	/* Continuity stats */
	if (dbg->expected_samples > 0) {
		double loss_pct = 100.0 * dbg->gap_samples_total / dbg->expected_samples;
		LOGP(DRADIO, LOGL_INFO,
		     "  CONTINUITY: gaps=%lu lost=%lu (%.4f%%) jit_under=%lu jit_over=%lu\n",
		     (unsigned long)dbg->gap_events,
		     (unsigned long)dbg->gap_samples_total,
		     loss_pct,
		     (unsigned long)dbg->jitter_underruns,
		     (unsigned long)dbg->jitter_overruns);
	}
	
	LOGP(DRADIO, LOGL_INFO, "=============================\n");
}

void audio_debug_reset(audio_debug_t *dbg)
{
	double interval = dbg->report_interval_sec;
	int enabled = dbg->enabled;
	memset(dbg, 0, sizeof(*dbg));
	dbg->report_interval_sec = interval;
	dbg->enabled = enabled;
	dbg->last_report_time = get_time_sec();
}
