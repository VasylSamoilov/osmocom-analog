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

#ifdef AUDIO_DEBUG

/* Global instance */
audio_debug_t g_rx_debug;

static const char *stage_names[] = {
	"SDR_RAW",
	"CHANNELIZER", 
	"FM_DEMOD",      /* >1.0 = over-modulation (>75kHz dev), common in broadcast */
	"STEREO_DEC",
	"DEEMPH",
	"RESAMPLE",
	"AUDIO_OUT"      /* >1.0 = actual audio clipping */
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

void audio_debug_fm_hf(audio_debug_t *dbg,
                       double pre_peak, double pre_rms, double pre_hf,
                       double post_peak, double post_rms, double post_hf,
                       double out_peak, double out_rms, double out_hf,
                       double signal_samplerate, double audio_samplerate,
                       double stereo_blend, double stereo_hf_gain, int stereo)
{
	if (!dbg->enabled)
		return;
	fm_hf_stats_t *f = &dbg->fm_hf;
	/* Keep last-seen values; report prints them once per interval */
	f->pre_peak = pre_peak; f->pre_rms = pre_rms; f->pre_hf = pre_hf;
	f->post_peak = post_peak; f->post_rms = post_rms; f->post_hf = post_hf;
	f->out_peak = out_peak; f->out_rms = out_rms; f->out_hf = out_hf;
	f->signal_samplerate = signal_samplerate;
	f->audio_samplerate = audio_samplerate;
	f->stereo_blend = stereo_blend;
	f->stereo_hf_gain = stereo_hf_gain;
	f->stereo = stereo;
	f->valid = 1;
}

void audio_debug_afc(audio_debug_t *dbg, double freq_error_hz,
                     double correction_hz,
                     double pilot_accuracy_hz, int using_pll_method, int pll_locked)
{
	if (!dbg->enabled)
		return;
	
	dbg->afc.freq_error_hz = freq_error_hz;
	dbg->afc.correction_hz = correction_hz;
	dbg->afc.pilot_accuracy_hz = pilot_accuracy_hz;
	dbg->afc.using_pll_method = using_pll_method;
	dbg->afc.pll_locked = pll_locked;
	dbg->afc.update_count++;
	
	/* Track peak error */
	double abs_err = fabs(freq_error_hz);
	if (abs_err > dbg->afc.peak_error_hz)
		dbg->afc.peak_error_hz = abs_err;
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
		
		/* For FM_DEMOD, >1.0 means over-modulation (>75kHz), not clipping.
		 * For intermediate stages (STEREO_DEC, DEEMPH, RESAMPLE), >1.0 is just
		 * "over" nominal level - no actual clipping since we use float/double.
		 * Only AUDIO_OUT has real clipping when written to 16-bit PCM. */
		const char *exceed_label;
		if (i == RX_STAGE_FM_DEMOD)
			exceed_label = "ovmod";
		else if (i == RX_STAGE_AUDIO_OUT)
			exceed_label = "clip";
		else
			exceed_label = "over";
		
		LOGP(DRADIO, LOGL_INFO,
		     "  %-12s: peak=%.4f (%+.1fdB) rms=%.4f (%+.1fdB) %s=%lu disc=%lu n=%.1fM\n",
		     stage_names[i],
		     peak, peak_db,
		     rms, rms_db,
		     exceed_label,
		     (unsigned long)s->clip_count,
		     (unsigned long)s->discontinuities,
		     s->sample_count / 1e6);
	}
	
	/* FM HF diagnostics */
	if (dbg->fm_hf.valid) {
		fm_hf_stats_t *f = &dbg->fm_hf;
		double sr = f->signal_samplerate / 1000.0;
		double ar = f->audio_samplerate / 1000.0;
		double pre_crest  = f->pre_peak  / (f->pre_rms  + 1e-12);
		double post_crest = f->post_peak / (f->post_rms + 1e-12);
		double out_crest  = f->out_peak  / (f->out_rms  + 1e-12);
		double pre_hfk    = (f->pre_hf  / (f->pre_rms  + 1e-12)) * sr;
		double post_hfk   = (f->post_hf / (f->post_rms + 1e-12)) * sr;
		double out_hfk    = (f->out_hf  / (f->out_rms  + 1e-12)) * ar;
		const char *mode  = f->stereo ? "stereo" : "mono";
		LOGP(DRADIO, LOGL_INFO,
		     "  FM HF (%s): pre{crest=%.2f hfk=%.1f} deemph{crest=%.2f hfk=%.1f} out{crest=%.2f hfk=%.1f} blend=%.0f%% hf_gain=%.0f%%\n",
		     mode,
		     pre_crest, pre_hfk,
		     post_crest, post_hfk,
		     out_crest, out_hfk,
		     f->stereo_blend * 100.0,
		     f->stereo_hf_gain * 100.0);
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
	
	/* AFC stats */
	if (dbg->afc.update_count > 0) {
		/* FLL_err: carrier offset from FLL (used for AFC correction)
		 * DC_err: legacy DC method (unreliable, debug only)
		 * pilot: 19 kHz pilot accuracy (station TX quality, not tuning error)
		 * corr: NCO correction being applied
		 * peak: max FLL error this period
		 * pilot_lock: stereo pilot PLL lock status (not same as stereo mode!) */
		LOGP(DRADIO, LOGL_INFO,
		     "  AFC: FLL_err=%+.1fHz pilot=%+.1fHz corr=%+.1fHz peak=%.1fHz n=%lu pilot_lock=%d\n",
		     dbg->afc.freq_error_hz,
		     dbg->afc.pilot_accuracy_hz,
		     dbg->afc.correction_hz,
		     dbg->afc.peak_error_hz,
		     (unsigned long)dbg->afc.update_count,
		     dbg->afc.pll_locked);
		/* Reset peak for next period */
		dbg->afc.peak_error_hz = 0;
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

#else /* !AUDIO_DEBUG */
/* Global stub instance */
audio_debug_t g_rx_debug;
#endif /* AUDIO_DEBUG */
