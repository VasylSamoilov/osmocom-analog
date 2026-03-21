/* SDR Clock Calibration - Scan + Detect
 *
 * Two-phase calibration:
 *   Phase 1 (Scan): Step across the GSM band in samplerate-wide chunks,
 *     measure per-ARFCN power via FFT to find active base stations.
 *   Phase 2 (Detect): Retune to the strongest ARFCNs, run parallel
 *     channelizer + FFT-averaging FCCH detectors for precise offset.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

#include "calibrate.h"
#include "fcch_detect.h"
#include "../libchannelizer/channelizer.h"
#include "../libpolyphase/polyphase.h"
#include "../libfastmath/fastmath.h"
#include "../libiqringbuf/iqringbuf.h"
#include "../libsample/sample.h"
#include "../libsdr/sdr_config.h"
#include "../liblogging/logging.h"
#include "../libfft/fft.h"

#ifdef HAVE_SOAPY
#include "../libsdr/soapy.h"
#endif
#ifdef HAVE_UHD
#include "../libsdr/uhd.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Calibration parameters */
#define AVG_COUNT       50      /* Measurements to collect per band */
#define AVG_THRESHOLD   8       /* Outliers to trim from each end */

/* Online outlier rejection: after MIN_FOR_REJECT measurements, reject
 * any new measurement that deviates more than OUTLIER_MAX_HZ from the
 * running median.  This catches spurious detections (e.g. adjacent
 * channel SCH bursts) without waiting for post-hoc trimming.
 *
 * Bootstrap phase: the first BOOTSTRAP_COUNT measurements are collected
 * without rejection, then split into clusters.  The tightest cluster
 * sets the initial median, preventing lock-on to a persistent spur. */
#define MIN_FOR_REJECT  8
#define BOOTSTRAP_COUNT 8       /* collect this many before first rejection */
#define OUTLIER_MAX_HZ  2000.0  /* max deviation from median */
/* Kalibrate-style behavior: collect first, trim/cluster at the end.
 * Keep online reject available for debugging, but disabled by default
 * to avoid median-lock starvation on bimodal channels. */
#define ENABLE_ONLINE_REJECT 0

/* Dead channel detection: deactivate channels with no FCCH after this many
 * total FFT frames.  At 500 kHz channelizer rate with 1024-pt FFT, each
 * frame is ~2 ms, so 5000 frames ≈ 10 seconds of data. */
#define DEAD_CHANNEL_FRAMES     5000

/* GSM channel spacing */
#define CHANNEL_SPACING 200000  /* 200 kHz */

/* Scan parameters */
#define SCAN_FFT_SIZE   4096    /* FFT size for power scan */
#define SCAN_FFT_LOG2   12
#define SCAN_AVG_FRAMES 32      /* FFT frames to average during scan */
#define SCAN_SETTLE_MS  50      /* ms to discard after retune (B210 needs ~30ms) */
#define DEBUG_MAX_DETECTIONS 4096
#define PREVALIDATE_SEC   0.45  /* Seconds to probe each candidate ARFCN */
#define PREVALIDATE_BATCH_TARGET_SEC 3.6 /* target wall-time per probe batch */
#define PREVALIDATE_BATCH_MIN 4
#define PREVALIDATE_BATCH_MAX 12
#define PREVALIDATE_MIN_FOUND     6
#define PREVALIDATE_MIN_DOM_COUNT 5
#define PREVALIDATE_MIN_DOM_RATIO 0.55
#define PREVALIDATE_RELAXED_MIN_SNR_DB 12.0
#define PREVALIDATE_INCONSISTENT_SPAN_HZ 1200.0
#define PREVALIDATE_CONSENSUS_HZ 600.0
#define PREVALIDATE_FALLBACK_MIN_FOUND 4
#define PREVALIDATE_FALLBACK_MIN_DOM_COUNT 2
#define PREVALIDATE_FALLBACK_MIN_DOM_RATIO 0.33
#define PREVALIDATE_FALLBACK_MIN_SNR_DB 6.5
#define PREVALIDATE_BEST_AVAILABLE_MIN_DOM_COUNT 4
#define PREVALIDATE_BEST_AVAILABLE_MIN_RATIO 0.75
#define PREVALIDATE_BEST_AVAILABLE_MIN_SNR_DB 9.0
#define PREVALIDATE_WEAK_MIN_FOUND 1
#define PREVALIDATE_WEAK_MIN_RATIO 0.60
#define PREVALIDATE_WEAK_MIN_SNR_DB 6.0
#define PREVALIDATE_WEAK_FAMILY_HZ 1500.0
#define PREVALIDATE_WEAK_FAMILY_MIN 2
#define PREVALIDATE_CLEAR_MIN_FOUND 8
#define PREVALIDATE_CLEAR_MIN_DOM_RATIO 0.80
#define PREVALIDATE_CLEAR_MIN_SNR_DB 12.0
#define PREVALIDATE_CLEAR_MAX_STD_HZ 250.0

/* Final confidence should reflect measurement quality, not only count. */
#define RESULT_MIN_ACCEPTED_SNR_DB 8.0
#define RESULT_FULL_ACCEPTED_SNR_DB 14.0
#define RESULT_REJECT_RATIO_GOOD 0.40
#define RESULT_REJECT_RATIO_BAD 1.80

/* Per-channel state for parallel detection (phase 2) */
typedef struct {
	channelizer_t   channelizer;
	/* Polyphase resampler path (alternative to channelizer) */
	int             use_polyphase;
	polyphase_t     poly_iq;        /* Single resampler for I+Q (shared timing) */
	double          mixer_phase;    /* NCO phase for freq shift */
	double          mixer_step;     /* NCO phase increment per sample */
	fcch_detect_t   detector;
	sample_t        *i_out;
	sample_t        *q_out;
	int             max_out;
	double          freq_offset;    /* Offset from SDR center */
	double          abs_freq;       /* Absolute frequency */
	int             arfcn;
	int             active;
	int             poly_debug_blocks;
} calib_channel_t;

/* ARFCN scan result (phase 1) */
typedef struct {
	int    arfcn;
	double freq;        /* Absolute frequency Hz */
	double power_db;    /* Measured power dBFS */
} arfcn_power_t;

typedef struct {
	float offset;
	float snr;
} offset_snr_t;

typedef struct {
	float offset;
	float snr;
	int accepted;
	int peak_bin;
} debug_sample_t;

typedef struct {
	double off;
	double snr;
} probe_pair_t;

/* All supported bands for --band all */
static const int all_bands[] = {
	GSM_BAND_900, GSM_BAND_1800, GSM_BAND_1900, GSM_BAND_850
};
#define NUM_ALL_BANDS 4

const char *gsm_band_name(int gsm_band)
{
	switch (gsm_band) {
	case GSM_BAND_900:  return "GSM-900";
	case GSM_BAND_850:  return "GSM-850";
	case GSM_BAND_1800: return "DCS-1800";
	case GSM_BAND_1900: return "PCS-1900";
	default:            return "Unknown";
	}
}

static int compare_float(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;
	return (fa > fb) - (fa < fb);
}

static int compare_double(const void *a, const void *b)
{
	double da = *(const double *)a;
	double db = *(const double *)b;
	return (da > db) - (da < db);
}

static double clamp01(double v)
{
	if (v < 0.0)
		return 0.0;
	if (v > 1.0)
		return 1.0;
	return v;
}

static int compare_power_desc(const void *a, const void *b)
{
	const arfcn_power_t *pa = a;
	const arfcn_power_t *pb = b;
	return (pa->power_db < pb->power_db) - (pa->power_db > pb->power_db);
}

static int compare_offset_snr(const void *a, const void *b)
{
	const offset_snr_t *pa = a;
	const offset_snr_t *pb = b;
	return (pa->offset > pb->offset) - (pa->offset < pb->offset);
}

static int compare_debug_sample_offset(const void *a, const void *b)
{
	const debug_sample_t *pa = a;
	const debug_sample_t *pb = b;
	return (pa->offset > pb->offset) - (pa->offset < pb->offset);
}

static int compare_probe_pair(const void *a, const void *b)
{
	const probe_pair_t *pa = a;
	const probe_pair_t *pb = b;
	return (pa->off > pb->off) - (pa->off < pb->off);
}

static void debug_dump_offset_families(const debug_sample_t *samples, int count)
{
	if (count < 4)
		return;

	debug_sample_t *sorted = malloc(count * sizeof(*sorted));
	if (!sorted)
		return;
	memcpy(sorted, samples, count * sizeof(*sorted));
	qsort(sorted, count, sizeof(*sorted), compare_debug_sample_offset);

	LOGP(DSDR, LOGL_INFO,
	     "Offset family debug: %d detections (accepted+rejected)\n", count);

	const float max_gap_hz = 900.0f;
	int start = 0;
	int family = 0;
	while (start < count && family < 12) {
		int end = start + 1;
		while (end < count &&
		       fabsf(sorted[end].offset - sorted[end - 1].offset) <= max_gap_hz)
			end++;

		int n = end - start;
		if (n >= 2) {
			double sum_off = 0.0, sum_snr = 0.0, sum_bin = 0.0;
			int n_acc = 0;
			for (int i = start; i < end; i++) {
				sum_off += sorted[i].offset;
				sum_snr += sorted[i].snr;
				sum_bin += sorted[i].peak_bin;
				if (sorted[i].accepted)
					n_acc++;
			}
			LOGP(DSDR, LOGL_INFO,
			     "  Family %d: n=%d accepted=%d rejected=%d "
			     "mean_off=%+.1f Hz mean_snr=%.1f dB mean_bin=%.1f "
			     "range=[%+.1f,%+.1f] Hz\n",
			     family + 1, n, n_acc, n - n_acc,
			     sum_off / n, sum_snr / n, sum_bin / n,
			     sorted[start].offset, sorted[end - 1].offset);
			family++;
		}
		start = end;
	}

	free(sorted);
}

static double trimmed_mean(float *values, int count, int trim, double *stddev)
{
	if (count <= 2 * trim)
		return 0;
	qsort(values, count, sizeof(float), compare_float);
	double sum = 0;
	int n = count - 2 * trim;
	for (int i = trim; i < count - trim; i++)
		sum += values[i];
	double mean = sum / n;
	if (stddev) {
		double sum_sq = 0;
		for (int i = trim; i < count - trim; i++) {
			double diff = values[i] - mean;
			sum_sq += diff * diff;
		}
		*stddev = sqrt(sum_sq / n);
	}
	return mean;
}

/* Estimate dominant offset cluster among high-SNR measurements.
 * Returns 1 on success, 0 if insufficient structure in data. */
static int estimate_high_snr_cluster(const float *offsets, const float *snrs,
				     int count, double *best_offset, double *best_stddev)
{
	if (count < 12)
		return 0;

	float *snr_sorted = malloc(count * sizeof(float));
	offset_snr_t *pairs = malloc(count * sizeof(offset_snr_t));
	float *cluster_vals = malloc(count * sizeof(float));
	if (!snr_sorted || !pairs || !cluster_vals) {
		free(snr_sorted);
		free(pairs);
		free(cluster_vals);
		return 0;
	}

	memcpy(snr_sorted, snrs, count * sizeof(float));
	qsort(snr_sorted, count, sizeof(float), compare_float);
	float snr_q3 = snr_sorted[(count * 3) / 4];

	int n = 0;
	for (int i = 0; i < count; i++) {
		if (snrs[i] >= snr_q3) {
			pairs[n].offset = offsets[i];
			pairs[n].snr = snrs[i];
			n++;
		}
	}
	if (n < 6) {
		free(snr_sorted);
		free(pairs);
		free(cluster_vals);
		return 0;
	}

	qsort(pairs, n, sizeof(offset_snr_t), compare_offset_snr);

	const float max_gap_hz = 1200.0f;
	double best_score = -1e9;
	double chosen_mean = 0.0;
	double chosen_stddev = 0.0;
	int found = 0;

	int start = 0;
	while (start < n) {
		int end = start + 1;
		while (end < n && fabsf(pairs[end].offset - pairs[end - 1].offset) <= max_gap_hz)
			end++;

		int c = end - start;
		if (c >= 3) {
			double snr_sum = 0.0;
			for (int i = start; i < end; i++) {
				snr_sum += pairs[i].snr;
				cluster_vals[i - start] = pairs[i].offset;
			}
			double cluster_stddev = 0.0;
			double cluster_mean = trimmed_mean(cluster_vals, c, (c > 10) ? 2 : 0, &cluster_stddev);
			double score = (snr_sum / c) + 0.15 * c;

			if (score > best_score) {
				best_score = score;
				chosen_mean = cluster_mean;
				chosen_stddev = cluster_stddev;
				found = 1;
			}
		}
		start = end;
	}

	free(snr_sorted);
	free(pairs);
	free(cluster_vals);

	if (!found)
		return 0;

	*best_offset = chosen_mean;
	*best_stddev = chosen_stddev;
	return 1;
}

/* Guard against stable wrong-family locks: if dominant detections sit far from
 * expected FCCH but there is a credible near-center family, prefer that family.
 * This keeps true large offsets unless near-center evidence is also coherent. */
static int select_near_center_family(const float *offsets, const float *snrs,
				     int count, double *sel_offset, double *sel_stddev)
{
	if (count < 6)
		return 0;

	offset_snr_t *pairs = malloc(count * sizeof(*pairs));
	float *cluster_vals = malloc(count * sizeof(*cluster_vals));
	if (!pairs || !cluster_vals) {
		free(pairs);
		free(cluster_vals);
		return 0;
	}
	for (int i = 0; i < count; i++) {
		pairs[i].offset = offsets[i];
		pairs[i].snr = snrs[i];
	}
	qsort(pairs, count, sizeof(*pairs), compare_offset_snr);

	const double gap_hz = 1200.0;
	const double far_guard_hz = 6000.0;
	const double near_guard_hz = 2000.0;
	const int near_min_n = 4;
	const double near_snr_slack_db = 4.0;

	int dominant_n = 0;
	double dominant_mean = 0.0;
	double dominant_snr = -1e9;

	int near_n = 0;
	double near_mean = 0.0;
	double near_std = 1e9;
	double near_snr = -1e9;

	int start = 0;
	while (start < count) {
		int end = start + 1;
		while (end < count &&
		       fabs(pairs[end].offset - pairs[end - 1].offset) <= gap_hz)
			end++;
		int n = end - start;
		if (n >= 2) {
			double sum_o = 0.0, sum_s = 0.0;
			for (int i = start; i < end; i++) {
				sum_o += pairs[i].offset;
				sum_s += pairs[i].snr;
				cluster_vals[i - start] = pairs[i].offset;
			}
			double mean_o = sum_o / n;
			double mean_s = sum_s / n;
			double std_o = 0.0;
			trimmed_mean(cluster_vals, n, 0, &std_o);

			if (n > dominant_n ||
			    (n == dominant_n && mean_s > dominant_snr)) {
				dominant_n = n;
				dominant_mean = mean_o;
				dominant_snr = mean_s;
			}
			if (fabs(mean_o) <= near_guard_hz &&
			    n >= near_min_n &&
			    (n > near_n || (n == near_n && mean_s > near_snr))) {
				near_n = n;
				near_mean = mean_o;
				near_std = std_o;
				near_snr = mean_s;
			}
		}
		start = end;
	}

	free(pairs);
	free(cluster_vals);

	if (dominant_n >= near_min_n &&
	    fabs(dominant_mean) >= far_guard_hz &&
	    near_n >= near_min_n &&
	    near_snr >= (dominant_snr - near_snr_slack_db)) {
		*sel_offset = near_mean;
		*sel_stddev = near_std;
		return 1;
	}

	return 0;
}

/* Running median of float array (non-destructive — copies before sorting) */
static double running_median(const float *values, int count)
{
	if (count <= 0) return 0.0;
	float *tmp = malloc(count * sizeof(float));
	if (!tmp) return values[0];
	memcpy(tmp, values, count * sizeof(float));
	qsort(tmp, count, sizeof(float), compare_float);
	double med = (count % 2 == 0)
		? ((double)tmp[count/2 - 1] + tmp[count/2]) / 2.0
		: tmp[count/2];
	free(tmp);
	return med;
}

static int receive_iq(float *buffer, int max_samples)
{
	int got = 0;
#ifdef HAVE_SOAPY
	if (sdr_config->soapy)
		got = soapy_receive(buffer, max_samples);
#endif
#ifdef HAVE_UHD
	if (sdr_config->uhd)
		got = uhd_receive(buffer, max_samples);
#endif
	return got;
}

/* Blocking receive: retries until buffer is full (used by scan phase) */
static int receive_iq_blocking(float *buffer, int max_samples)
{
	int total = 0, retries = 0;
	const int max_retries = 100;
	while (total < max_samples && retries < max_retries) {
		int got = receive_iq(buffer + total * 2, max_samples - total);
		if (got > 0) {
			total += got;
			retries = 0;
		} else {
			usleep(1000);
			retries++;
		}
	}
	return total;
}

/* Retune SDR RX to new center frequency */
static int retune_rx(double frequency)
{
	/* Apply upconverter offset (mirrors sdr_open_internal in sdr.c) */
	double actual_freq = frequency + sdr_config->rx_upconverter;

#ifdef HAVE_UHD
	if (sdr_config->uhd)
		return uhd_set_rx_frequency(actual_freq);
#endif
#ifdef HAVE_SOAPY
	if (sdr_config->soapy)
		return soapy_set_rx_frequency(actual_freq);
#endif
	return -ENODEV;
}

/* Flush RX buffer after retune (discard stale samples) */
static void flush_rx(float *buf, int buf_samples, int samplerate)
{
	int total_discard = (int)(samplerate * SCAN_SETTLE_MS / 1000.0);
	if (total_discard < 4096) total_discard = 4096;

	int chunk = buf_samples;
	if (chunk > total_discard) chunk = total_discard;
	if (chunk < 1024) chunk = 1024;

	int discarded = 0;
	while (discarded < total_discard) {
		int want = total_discard - discarded;
		if (want > chunk) want = chunk;
		int got = receive_iq_blocking(buf, want);
		if (got <= 0) break;
		discarded += got;
	}
}

void calibrate_config_default(calibrate_config_t *config)
{
	memset(config, 0, sizeof(*config));
	config->scan_gsm = 1;
	config->gsm_bands = GSM_BAND_900;
	config->timeout_sec = 90.0;
	config->verbosity = 1;
	config->num_channels = CALIB_MAX_CHANNELS;
	config->fast_math = 0;
	config->use_channelizer = 0;
	config->forced_arfcn = -1;
	config->probe_sequential = 0;
}

/* ================================================================
 * Phase 1: Scan band for active ARFCNs
 * ================================================================ */

static int scan_band(int samplerate, int gsm_band,
                     int arfcn_start, int arfcn_end,
                     arfcn_power_t *results, int max_results,
                     int verbosity)
{
	int num_results = 0;
	double band_lo = arfcn_to_freq(arfcn_start, gsm_band);
	double band_hi = arfcn_to_freq(arfcn_end, gsm_band);

	if (band_lo == 0.0 || band_hi == 0.0)
		return 0;

	double usable_bw = (double)samplerate * 0.75;
	double step_hz = usable_bw;

	double scan_start = band_lo - CHANNEL_SPACING;
	double scan_end = band_hi + CHANNEL_SPACING;
	int num_steps = (int)ceil((scan_end - scan_start) / step_hz);
	if (num_steps < 1) num_steps = 1;

	LOGP(DSDR, LOGL_INFO, "Scanning %s: %.1f-%.1f MHz in %d steps of %.1f MHz\n",
	     gsm_band_name(gsm_band),
	     band_lo / 1e6, band_hi / 1e6,
	     num_steps, step_hz / 1e6);

	int fft_size = SCAN_FFT_SIZE;
	double *fft_re = calloc(fft_size, sizeof(double));
	double *fft_im = calloc(fft_size, sizeof(double));
	double *mag_accum = calloc(fft_size, sizeof(double));
	double *window = calloc(fft_size, sizeof(double));
	int capture_samples = fft_size * (SCAN_AVG_FRAMES + 2);
	float *iq_buf = calloc(capture_samples * 2, sizeof(float));

	if (!fft_re || !fft_im || !mag_accum || !window || !iq_buf) {
		free(fft_re); free(fft_im); free(mag_accum); free(window); free(iq_buf);
		return -ENOMEM;
	}

	for (int i = 0; i < fft_size; i++)
		window[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (fft_size - 1)));

	double bin_hz = (double)samplerate / fft_size;

	for (int s = 0; s < num_steps; s++) {
		double center = scan_start + step_hz / 2.0 + s * step_hz;

		if (center < band_lo - samplerate / 2.0)
			center = band_lo;
		if (center > band_hi + samplerate / 2.0)
			center = band_hi;

		int rc = retune_rx(center);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to retune to %.3f MHz\n",
			     center / 1e6);
			continue;
		}

		flush_rx(iq_buf, capture_samples, samplerate);

		memset(mag_accum, 0, fft_size * sizeof(double));
		int frames_done = 0;

		while (frames_done < SCAN_AVG_FRAMES) {
			int want = fft_size * (SCAN_AVG_FRAMES - frames_done + 1);
			if (want > capture_samples) want = capture_samples;
			int got = receive_iq_blocking(iq_buf, want);
			if (got < fft_size) break;

			int offset = 0;
			while (offset + fft_size <= got && frames_done < SCAN_AVG_FRAMES) {
				for (int i = 0; i < fft_size; i++) {
					fft_re[i] = iq_buf[(offset + i) * 2]     * window[i];
					fft_im[i] = iq_buf[(offset + i) * 2 + 1] * window[i];
				}
				fft_process(1, SCAN_FFT_LOG2, fft_re, fft_im);
				for (int i = 0; i < fft_size; i++)
					mag_accum[i] += fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i];
				frames_done++;
				offset += fft_size;
			}
		}

		if (frames_done == 0)
			continue;

		double half_bw = usable_bw / 2.0;

		for (int a = arfcn_start; a <= arfcn_end && num_results < max_results; a++) {
			double freq = arfcn_to_freq(a, gsm_band);
			if (freq == 0.0) continue;
			double diff = freq - center;
			if (fabs(diff) > half_bw) continue;

			int bin = (int)round(diff / bin_hz);
			if (bin < 0) bin += fft_size;
			if (bin < 0 || bin >= fft_size) continue;

			int ch_bins = (int)(CHANNEL_SPACING / bin_hz);
			if (ch_bins < 3) ch_bins = 3;
			int start_bin = bin - ch_bins / 2;
			double ch_power = 0;
			for (int b = 0; b < ch_bins; b++) {
				int idx = (start_bin + b + fft_size) % fft_size;
				ch_power += mag_accum[idx];
			}
			ch_power /= (frames_done * ch_bins);

			double power_db = 10.0 * log10(ch_power > 0 ? ch_power : 1e-30);

			int found = 0;
			for (int r = 0; r < num_results; r++) {
				if (results[r].arfcn == a) {
					if (power_db > results[r].power_db)
						results[r].power_db = power_db;
					found = 1;
					break;
				}
			}
			if (!found) {
				results[num_results].arfcn = a;
				results[num_results].freq = freq;
				results[num_results].power_db = power_db;
				num_results++;
			}
		}

		if (verbosity >= 1)
			LOGP(DSDR, LOGL_INFO, "  Step %d/%d: %.3f MHz, %d frames averaged\n",
			     s + 1, num_steps, center / 1e6, frames_done);
	}

	free(fft_re); free(fft_im); free(mag_accum); free(window); free(iq_buf);

	if (num_results > 0)
		qsort(results, num_results, sizeof(arfcn_power_t), compare_power_desc);

	return num_results;
}

/* Kalibrate-style FCCH pre-validation:
 * probe top power ARFCNs briefly and choose the one with strongest FCCH evidence.
 * Returns index into 'scan_results', or -1 on failure/no evidence. */
static int choose_fcch_reference_arfcn(int samplerate,
				       const arfcn_power_t *scan_results,
				       int num_active,
				       int verbosity,
				       int probe_sequential)
{
	int top_n = num_active; /* Probe all active ARFCNs, but in batches. */
	if (top_n <= 0)
		return -1;

	int probe_samples = (int)(samplerate * PREVALIDATE_SEC);
	if (probe_samples < samplerate / 8)
		probe_samples = samplerate / 8;

	int batch_size = (int)lround(PREVALIDATE_BATCH_TARGET_SEC / PREVALIDATE_SEC);
	if (batch_size < PREVALIDATE_BATCH_MIN)
		batch_size = PREVALIDATE_BATCH_MIN;
	if (batch_size > PREVALIDATE_BATCH_MAX)
		batch_size = PREVALIDATE_BATCH_MAX;
	if (probe_sequential)
		batch_size = 1;

	int chunk_samples = 8192;
	if (chunk_samples > probe_samples)
		chunk_samples = probe_samples;

	float *iq = calloc(chunk_samples * 2, sizeof(float));
	if (!iq)
		return -1;

	int best_idx = -1;
	double best_score = -1e9;
	double best_dom_snr = -1e9;
	double best_dom_off = 0.0;
	int best_dom_count = 0;
	int best_consensus_hits = 0;
	double best_power = -1e9;
	int evidence_idx = -1;
	double evidence_score = -1e9;
	double evidence_off = 0.0;
	double evidence_snr = -1e9;
	double strict_min_off = 1e9, strict_max_off = -1e9;
	int strict_count = 0;
	typedef struct {
		int idx;
		double off;
		double snr;
		double std;
		double power;
		int found;
		int dom_count;
		double dom_ratio;
	} probe_candidate_t;
	probe_candidate_t *fallback = calloc(top_n, sizeof(*fallback));
	int fallback_count = 0;
	probe_candidate_t *weak = calloc(top_n, sizeof(*weak));
	int weak_count = 0;
	double *family_seen_offsets = calloc(top_n, sizeof(double));
	int family_seen_count = 0;
	if (!fallback || !weak || !family_seen_offsets) {
		free(iq);
		free(fallback);
		free(weak);
		free(family_seen_offsets);
		return -ENOMEM;
	}

	if (verbosity >= 1) {
		if (probe_sequential) {
			LOGP(DSDR, LOGL_INFO,
			     "FCCH pre-validation: probing %d active ARFCNs channel-by-channel for %.0f ms each\n",
			     top_n, PREVALIDATE_SEC * 1000.0);
		} else {
			LOGP(DSDR, LOGL_INFO,
			     "FCCH pre-validation: probing %d active ARFCNs in batches of %d for %.0f ms each\n",
			     top_n, batch_size, PREVALIDATE_SEC * 1000.0);
		}
	}

	for (int i = 0; i < top_n; i++) {
		if (verbosity >= 1 && (i % batch_size) == 0) {
			int batch_end = i + batch_size;
			if (batch_end > top_n)
				batch_end = top_n;
			LOGP(DSDR, LOGL_INFO,
			     "  FCCH pre-validation batch %d: candidates %d..%d\n",
			     (i / batch_size) + 1, i + 1, batch_end);
		}
		fcch_detect_t det;
		channelizer_t chz;
		sample_t *i_out = NULL, *q_out = NULL;
		float *ch_iq = NULL;
		int found = 0;
		double snr_sum = 0.0;
		double snr_peak = -1e9;
		double last_offset = 0.0;
		probe_pair_t pairs[64];
		int pair_count = 0;
		int sampled = 0;

		if (retune_rx(scan_results[i].freq) < 0)
			continue;

		flush_rx(iq, chunk_samples, samplerate);

		int rc_ch = channelizer_init(&chz, samplerate, (int)(GSM_RATE + 0.5), 0, 0);
		if (rc_ch < 0)
			continue;
		int out_rate = channelizer_get_output_rate(&chz);
		int max_out = chunk_samples + 16;
		i_out = calloc(max_out, sizeof(sample_t));
		q_out = calloc(max_out, sizeof(sample_t));
		ch_iq = calloc(max_out * 2, sizeof(float));
		if (!i_out || !q_out || !ch_iq) {
			free(i_out);
			free(q_out);
			free(ch_iq);
			channelizer_exit(&chz);
			continue;
		}
		if (fcch_detect_init(&det, out_rate) < 0) {
			free(i_out);
			free(q_out);
			free(ch_iq);
			channelizer_exit(&chz);
			continue;
		}

		while (sampled < probe_samples) {
			int want = probe_samples - sampled;
			if (want > chunk_samples)
				want = chunk_samples;

			int got = receive_iq_blocking(iq, want);
			if (got <= 0)
				break;
			sampled += got;
			int out_count = channelizer_process(&chz, iq, got, i_out, q_out);
			if (out_count <= 0)
				continue;
			for (int s = 0; s < out_count; s++) {
				ch_iq[s * 2] = (float)i_out[s];
				ch_iq[s * 2 + 1] = (float)q_out[s];
			}

			double offset = 0.0;
			double snr_db = 0.0;
			int rc = fcch_detect_process(&det, ch_iq, out_count, &offset, &snr_db);
			if (rc > 0) {
				found++;
				snr_sum += snr_db;
				if (snr_db > snr_peak)
					snr_peak = snr_db;
				last_offset = offset;
				if (pair_count < (int)(sizeof(pairs) / sizeof(pairs[0]))) {
					pairs[pair_count].off = offset;
					pairs[pair_count].snr = snr_db;
					pair_count++;
				}
				fcch_detect_reset(&det);
			}
		}

		/* Dominant-family analysis: prefer candidates with stable FCCH family
		 * and strong SNR, not just raw detection count. */
		double dom_mean = 0.0, dom_std = 1e9, dom_snr = -1e9;
		int dom_count = 0;
		if (pair_count > 0) {
			qsort(pairs, pair_count, sizeof(pairs[0]), compare_probe_pair);
			const double gap_hz = 1200.0;
			int start = 0;
			while (start < pair_count) {
				int end = start + 1;
				while (end < pair_count &&
				       fabs(pairs[end].off - pairs[end - 1].off) <= gap_hz)
					end++;
				int n = end - start;
				if (n >= 2) {
					double sum_o = 0.0, sum_s = 0.0;
					for (int k = start; k < end; k++) {
						sum_o += pairs[k].off;
						sum_s += pairs[k].snr;
					}
					double mean_o = sum_o / n;
					double mean_s = sum_s / n;
					double var = 0.0;
					for (int k = start; k < end; k++) {
						double d = pairs[k].off - mean_o;
						var += d * d;
					}
					double std_o = sqrt(var / n);
					/* Cluster score: high SNR, more members, lower spread. */
					double cscore = mean_s + 0.25 * n - 0.002 * std_o;
					double best_local = dom_snr + 0.25 * dom_count - 0.002 * dom_std;
					if (cscore > best_local) {
						dom_mean = mean_o;
						dom_std = std_o;
						dom_snr = mean_s;
						dom_count = n;
					}
				}
				start = end;
			}
		}
		if (dom_count == 0 && found > 0) {
			dom_mean = last_offset;
			dom_snr = snr_sum / found;
			dom_count = 1;
			dom_std = 5000.0;
		}
		double dom_ratio = (found > 0) ? ((double)dom_count / found) : 0.0;
		int fallback_quality = (found >= PREVALIDATE_FALLBACK_MIN_FOUND &&
				      dom_count >= PREVALIDATE_FALLBACK_MIN_DOM_COUNT &&
				      dom_ratio >= PREVALIDATE_FALLBACK_MIN_DOM_RATIO &&
				      dom_snr >= PREVALIDATE_FALLBACK_MIN_SNR_DB);
		int strict_quality = (found >= PREVALIDATE_MIN_FOUND &&
				    dom_count >= PREVALIDATE_MIN_DOM_COUNT &&
				    dom_ratio >= PREVALIDATE_MIN_DOM_RATIO &&
				    dom_snr >= 10.0);
		int consensus_hits = 0;
		if (found > 0) {
			consensus_hits = 1; /* include current candidate */
			for (int si = 0; si < family_seen_count; si++) {
				if (fabs(dom_mean - family_seen_offsets[si]) <= PREVALIDATE_CONSENSUS_HZ)
					consensus_hits++;
			}
		}
		/* Track any weakly plausible family so consensus is not dominated by
		 * only very-strong channels. */
		if (found > 0 &&
		    dom_ratio >= PREVALIDATE_WEAK_MIN_RATIO &&
		    dom_snr >= PREVALIDATE_WEAK_MIN_SNR_DB &&
		    family_seen_count < top_n)
			family_seen_offsets[family_seen_count++] = dom_mean;

		if (verbosity >= 1) {
			LOGP(DSDR, LOGL_INFO,
			     "  Probe ARFCN %d %.3f MHz: FCCH found=%d snr_peak=%.1f dB "
			     "snr_avg=%.1f dB dom_n=%d dom_ratio=%.2f dom_snr=%.1f dom_std=%.0fHz cons=%d "
			     "dom_off=%+.1f Hz last_offset=%+.1f Hz power=%+.1f dBFS\n",
			     scan_results[i].arfcn, scan_results[i].freq / 1e6,
			     found, (found > 0) ? snr_peak : -99.0,
			     (found > 0) ? (snr_sum / found) : -99.0,
			     dom_count, dom_ratio, dom_snr, dom_std, consensus_hits, dom_mean,
			     last_offset, scan_results[i].power_db);
		}

		/* Prefer consistency over raw hit count:
		 * a high-SNR channel with mixed offset families is unsafe. */
		double candidate_score = dom_snr
				       + 0.20 * dom_count
				       + 8.0 * dom_ratio
				       - 0.002 * dom_std;
		/* Penalize isolated families; they are often stable but wrong locks. */
		if (consensus_hits < 2)
			candidate_score -= 8.0;
		if (!strict_quality)
			candidate_score = -1e9;
		if (strict_quality && (candidate_score > best_score ||
		    (fabs(candidate_score - best_score) < 1e-6 &&
		     dom_snr > best_dom_snr) ||
		    (fabs(candidate_score - best_score) < 1e-6 &&
		     fabs(dom_snr - best_dom_snr) < 1e-6 &&
		     scan_results[i].power_db > best_power))) {
			best_idx = i;
			best_score = candidate_score;
			best_dom_snr = dom_snr;
			best_dom_off = dom_mean;
			best_dom_count = dom_count;
			best_consensus_hits = consensus_hits;
			best_power = scan_results[i].power_db;
		}
		if (strict_quality) {
			if (dom_mean < strict_min_off)
				strict_min_off = dom_mean;
			if (dom_mean > strict_max_off)
				strict_max_off = dom_mean;
			strict_count++;
		}
		if (fallback_quality && fallback_count < top_n) {
			fallback[fallback_count].idx = i;
			fallback[fallback_count].off = dom_mean;
			fallback[fallback_count].snr = dom_snr;
			fallback[fallback_count].std = dom_std;
			fallback[fallback_count].power = scan_results[i].power_db;
			fallback[fallback_count].found = found;
			fallback[fallback_count].dom_count = dom_count;
			fallback[fallback_count].dom_ratio = dom_ratio;
			fallback_count++;
		}
		if (found > 0 && dom_snr >= PREVALIDATE_WEAK_MIN_SNR_DB) {
			double escore = dom_snr * found * fmax(dom_ratio, 0.25);
			if (escore > evidence_score) {
				evidence_score = escore;
				evidence_idx = i;
				evidence_off = dom_mean;
				evidence_snr = dom_snr;
			}
		}
		if (found >= PREVALIDATE_WEAK_MIN_FOUND &&
		    dom_ratio >= PREVALIDATE_WEAK_MIN_RATIO &&
		    dom_snr >= PREVALIDATE_WEAK_MIN_SNR_DB &&
		    weak_count < top_n) {
			weak[weak_count].idx = i;
			weak[weak_count].off = dom_mean;
			weak[weak_count].snr = dom_snr;
			weak[weak_count].std = dom_std;
			weak[weak_count].power = scan_results[i].power_db;
			weak[weak_count].found = found;
			weak[weak_count].dom_count = dom_count;
			weak[weak_count].dom_ratio = dom_ratio;
			weak_count++;
		}

		/* Optional kalibrate-like mode: stop as soon as we hit one clearly
		 * clean FCCH channel instead of waiting for cross-ARFCN consensus. */
		if (probe_sequential &&
		    strict_quality &&
		    found >= PREVALIDATE_CLEAR_MIN_FOUND &&
		    dom_ratio >= PREVALIDATE_CLEAR_MIN_DOM_RATIO &&
		    dom_snr >= PREVALIDATE_CLEAR_MIN_SNR_DB &&
		    dom_std <= PREVALIDATE_CLEAR_MAX_STD_HZ) {
			if (verbosity >= 1) {
				LOGP(DSDR, LOGL_NOTICE,
				     "FCCH pre-validation: sequential clear winner ARFCN %d %.3f MHz "
				     "(found=%d dom_n=%d dom_ratio=%.2f dom_snr=%.1f dB dom_std=%.0fHz dom_off=%+.1f Hz)\n",
				     scan_results[i].arfcn, scan_results[i].freq / 1e6,
				     found, dom_count, dom_ratio, dom_snr, dom_std, dom_mean);
			}
			fcch_detect_exit(&det);
			channelizer_exit(&chz);
			free(i_out);
			free(q_out);
			free(ch_iq);
			free(iq);
			free(family_seen_offsets);
			free(fallback);
			free(weak);
			return i;
		}

		fcch_detect_exit(&det);
		channelizer_exit(&chz);
		free(i_out);
		free(q_out);
		free(ch_iq);

		/* Early-exit at batch boundaries only when winner is strong and
		 * candidate families seen so far are not contradictory. */
		if (((i + 1) % batch_size) == 0 || (i + 1) == top_n) {
			double span = (strict_count >= 2) ? (strict_max_off - strict_min_off) : 0.0;
			if (best_idx >= 0 &&
			    best_dom_snr >= 14.0 &&
			    best_dom_count >= 7 &&
			    best_consensus_hits >= 2 &&
			    (strict_count < 2 || span <= PREVALIDATE_INCONSISTENT_SPAN_HZ)) {
				if (verbosity >= 1) {
					LOGP(DSDR, LOGL_INFO,
					     "  FCCH pre-validation early winner after batch %d: ARFCN %d (dom_snr %.1f dB)\n",
					     (i / batch_size) + 1,
					     scan_results[best_idx].arfcn, best_dom_snr);
				}
				break;
			}
		}
	}

	free(iq);
	free(family_seen_offsets);

	/* If strict winner is an isolated family, but weak evidence shows
	 * a different multi-ARFCN family, trust the family consensus instead. */
	if (best_idx >= 0 && best_consensus_hits < 2 &&
	    weak_count >= PREVALIDATE_WEAK_FAMILY_MIN) {
		int fam_start = 0;
		int best_fam_n = 0;
		double best_fam_w = -1e9;
		double best_fam_off = 0.0;
		int best_w_idx = -1;
		for (int a = 0; a < weak_count - 1; a++) {
			for (int b = a + 1; b < weak_count; b++) {
				if (weak[b].off < weak[a].off) {
					probe_candidate_t tmp = weak[a];
					weak[a] = weak[b];
					weak[b] = tmp;
				}
			}
		}
		while (fam_start < weak_count) {
			int fam_end = fam_start + 1;
			double fam_w = weak[fam_start].snr * weak[fam_start].found;
			while (fam_end < weak_count &&
			       fabs(weak[fam_end].off - weak[fam_end - 1].off) <= PREVALIDATE_WEAK_FAMILY_HZ) {
				fam_w += weak[fam_end].snr * weak[fam_end].found;
				fam_end++;
			}
			int fam_n = fam_end - fam_start;
			if (fam_w > best_fam_w ||
			    (fabs(fam_w - best_fam_w) < 1e-6 && fam_n > best_fam_n)) {
				best_fam_w = fam_w;
				best_fam_n = fam_n;
				best_fam_off = weak[fam_start].off;
			}
			fam_start = fam_end;
		}
		for (int i = 0; i < weak_count; i++) {
			if (fabs(weak[i].off - best_fam_off) > PREVALIDATE_WEAK_FAMILY_HZ)
				continue;
			if (best_w_idx < 0 ||
			    weak[i].snr > weak[best_w_idx].snr ||
			    (fabs(weak[i].snr - weak[best_w_idx].snr) < 1e-6 &&
			     weak[i].found > weak[best_w_idx].found) ||
			    (fabs(weak[i].snr - weak[best_w_idx].snr) < 1e-6 &&
			     weak[i].found == weak[best_w_idx].found &&
			     weak[i].power > weak[best_w_idx].power)) {
				best_w_idx = i;
			}
		}
		if (best_w_idx >= 0 &&
		    best_fam_n >= PREVALIDATE_WEAK_FAMILY_MIN &&
		    fabs(best_fam_off - best_dom_off) > PREVALIDATE_CONSENSUS_HZ) {
			if (verbosity >= 1) {
				LOGP(DSDR, LOGL_NOTICE,
				     "FCCH pre-validation: overriding isolated strict winner ARFCN %d "
				     "(dom_off=%+.1f Hz, cons=%d) with weak-family consensus ARFCN %d "
				     "(family_n=%d dom_off=%+.1f Hz)\n",
				     scan_results[best_idx].arfcn, best_dom_off, best_consensus_hits,
				     scan_results[weak[best_w_idx].idx].arfcn, best_fam_n, best_fam_off);
			}
			int ret = weak[best_w_idx].idx;
			free(fallback);
			free(weak);
			return ret;
		}
	}

	if (best_idx < 0 || best_dom_count <= 0) {
		/* Best-effort fallback: choose from the dominant offset family among
		 * weaker candidates, rather than strongest-power ARFCN. */
		if (fallback_count > 0) {
			int fam_start = 0;
			int best_fam_n = 0;
			double best_fam_w = -1e9;
			double best_fam_off = 0.0;
			int best_fb_idx = -1;
			/* Sort fallback candidates by dominant offset. */
			for (int a = 0; a < fallback_count - 1; a++) {
				for (int b = a + 1; b < fallback_count; b++) {
					if (fallback[b].off < fallback[a].off) {
						probe_candidate_t tmp = fallback[a];
						fallback[a] = fallback[b];
						fallback[b] = tmp;
					}
				}
			}
			while (fam_start < fallback_count) {
				int fam_end = fam_start + 1;
				double fam_w = fallback[fam_start].snr * fallback[fam_start].dom_count;
				while (fam_end < fallback_count &&
				       fabs(fallback[fam_end].off - fallback[fam_end - 1].off) <= PREVALIDATE_CONSENSUS_HZ) {
					fam_w += fallback[fam_end].snr * fallback[fam_end].dom_count;
					fam_end++;
				}
				int fam_n = fam_end - fam_start;
				if (fam_w > best_fam_w ||
				    (fabs(fam_w - best_fam_w) < 1e-6 && fam_n > best_fam_n)) {
					best_fam_w = fam_w;
					best_fam_n = fam_n;
					best_fam_off = fallback[fam_start].off;
				}
				fam_start = fam_end;
			}
			for (int i = 0; i < fallback_count; i++) {
				if (fabs(fallback[i].off - best_fam_off) > PREVALIDATE_CONSENSUS_HZ)
					continue;
				if (best_fb_idx < 0 ||
				    fallback[i].snr > fallback[best_fb_idx].snr ||
				    (fabs(fallback[i].snr - fallback[best_fb_idx].snr) < 1e-6 &&
				     fallback[i].dom_count > fallback[best_fb_idx].dom_count) ||
				    (fabs(fallback[i].snr - fallback[best_fb_idx].snr) < 1e-6 &&
				     fallback[i].dom_count == fallback[best_fb_idx].dom_count &&
				     fallback[i].power > fallback[best_fb_idx].power)) {
					best_fb_idx = i;
				}
			}
			if (best_fb_idx >= 0 &&
			    (best_fam_n >= 2 ||
			     (fallback[best_fb_idx].found >= PREVALIDATE_FALLBACK_MIN_FOUND &&
			      fallback[best_fb_idx].dom_count >= PREVALIDATE_BEST_AVAILABLE_MIN_DOM_COUNT &&
			      fallback[best_fb_idx].dom_ratio >= PREVALIDATE_BEST_AVAILABLE_MIN_RATIO &&
			      fallback[best_fb_idx].snr >= PREVALIDATE_BEST_AVAILABLE_MIN_SNR_DB))) {
				if (verbosity >= 1) {
					LOGP(DSDR, LOGL_NOTICE,
					     "FCCH pre-validation: no strict winner, using %s fallback ARFCN %d %.3f MHz "
					     "(family_n=%d dom_snr %.1f dB dom_off=%+.1f Hz)\n",
					     (best_fam_n >= 2) ? "family-consensus" : "best-available",
					     scan_results[fallback[best_fb_idx].idx].arfcn,
					     scan_results[fallback[best_fb_idx].idx].freq / 1e6,
					     best_fam_n, fallback[best_fb_idx].snr,
					     fallback[best_fb_idx].off);
				}
				int ret = fallback[best_fb_idx].idx;
				free(fallback);
				free(weak);
				return ret;
			}
		}
		/* Weak-evidence family fallback: when bursts are sparse, accept the
		 * strongest consistent weak family instead of hard failing. */
		if (weak_count > 0) {
			int fam_start = 0;
			int best_fam_n = 0;
			double best_fam_w = -1e9;
			double best_fam_off = 0.0;
			int best_w_idx = -1;
			for (int a = 0; a < weak_count - 1; a++) {
				for (int b = a + 1; b < weak_count; b++) {
					if (weak[b].off < weak[a].off) {
						probe_candidate_t tmp = weak[a];
						weak[a] = weak[b];
						weak[b] = tmp;
					}
				}
			}
			while (fam_start < weak_count) {
				int fam_end = fam_start + 1;
				double fam_w = weak[fam_start].snr * weak[fam_start].found;
				while (fam_end < weak_count &&
				       fabs(weak[fam_end].off - weak[fam_end - 1].off) <= PREVALIDATE_WEAK_FAMILY_HZ) {
					fam_w += weak[fam_end].snr * weak[fam_end].found;
					fam_end++;
				}
				int fam_n = fam_end - fam_start;
				if (fam_w > best_fam_w ||
				    (fabs(fam_w - best_fam_w) < 1e-6 && fam_n > best_fam_n)) {
					best_fam_w = fam_w;
					best_fam_n = fam_n;
					best_fam_off = weak[fam_start].off;
				}
				fam_start = fam_end;
			}
			for (int i = 0; i < weak_count; i++) {
				if (fabs(weak[i].off - best_fam_off) > PREVALIDATE_WEAK_FAMILY_HZ)
					continue;
				if (best_w_idx < 0 ||
				    weak[i].snr > weak[best_w_idx].snr ||
				    (fabs(weak[i].snr - weak[best_w_idx].snr) < 1e-6 &&
				     weak[i].found > weak[best_w_idx].found) ||
				    (fabs(weak[i].snr - weak[best_w_idx].snr) < 1e-6 &&
				     weak[i].found == weak[best_w_idx].found &&
				     weak[i].power > weak[best_w_idx].power)) {
					best_w_idx = i;
				}
			}
			if (best_w_idx >= 0 && best_fam_n >= PREVALIDATE_WEAK_FAMILY_MIN) {
				if (verbosity >= 1) {
					LOGP(DSDR, LOGL_NOTICE,
					     "FCCH pre-validation: using weak-family fallback ARFCN %d %.3f MHz "
					     "(family_n=%d dom_snr %.1f dB dom_off=%+.1f Hz)\n",
					     scan_results[weak[best_w_idx].idx].arfcn,
					     scan_results[weak[best_w_idx].idx].freq / 1e6,
					     best_fam_n, weak[best_w_idx].snr,
					     weak[best_w_idx].off);
				}
				int ret = weak[best_w_idx].idx;
				free(fallback);
				free(weak);
				return ret;
			}
		}
		if (evidence_idx >= 0) {
			if (verbosity >= 1) {
				LOGP(DSDR, LOGL_NOTICE,
				     "FCCH pre-validation: using last-resort evidence fallback ARFCN %d %.3f MHz "
				     "(score %.1f dom_snr %.1f dB dom_off=%+.1f Hz)\n",
				     scan_results[evidence_idx].arfcn,
				     scan_results[evidence_idx].freq / 1e6,
				     evidence_score, evidence_snr, evidence_off);
			}
			int ret = evidence_idx;
			free(fallback);
			free(weak);
			return ret;
		}
		free(fallback);
		free(weak);
		return -1;
	}
	free(fallback);
	free(weak);

	if (verbosity >= 1) {
		LOGP(DSDR, LOGL_INFO,
		     "FCCH pre-validation winner: ARFCN %d %.3f MHz "
		     "(score=%.2f dom_n=%d dom_snr=%.1f dB)\n",
		     scan_results[best_idx].arfcn, scan_results[best_idx].freq / 1e6,
		     best_score, best_dom_count, best_dom_snr);
	}

	return best_idx;
}

/* ================================================================
 * Phase 2: Set up channelizer + detector on selected ARFCNs
 * ================================================================ */

static int setup_channels_for_arfcns(calib_channel_t *channels, int max_channels,
                                     int samplerate, double center_freq,
                                     const arfcn_power_t *arfcns, int num_arfcns,
                                     int fast_math, int use_polyphase,
                                     int verbosity)
{
	int num = 0;
	double half_bw = (double)samplerate / 2.0 * 0.75;

	/* Track selected ARFCNs to skip adjacent channels (±2 ARFCNs).
	 * Adjacent channels show power from spectral leakage but don't
	 * carry their own BCCH/FCCH, wasting detection time. */
	int selected_arfcns[CALIB_MAX_CHANNELS];
	int num_selected = 0;

	for (int i = 0; i < num_arfcns && num < max_channels; i++) {
		double offset = arfcns[i].freq - center_freq;
		if (fabs(offset) + CHANNEL_SPACING / 2.0 > half_bw)
			continue;

		/* Skip ARFCNs within ±2 of an already-selected stronger ARFCN.
		 * The scan results are sorted by power (strongest first), so
		 * any already-selected ARFCN is stronger than this candidate. */
		int too_close = 0;
		for (int j = 0; j < num_selected; j++) {
			if (abs(arfcns[i].arfcn - selected_arfcns[j]) <= 2) {
				too_close = 1;
				break;
			}
		}
		if (too_close) {
			if (verbosity >= 2)
				LOGP(DSDR, LOGL_INFO,
				     "  Skipping ARFCN %d (adjacent to stronger selected ARFCN)\n",
				     arfcns[i].arfcn);
			continue;
		}

		int target_rate = (int)(GSM_RATE + 0.5);
		int decimation, out_rate;

		if (use_polyphase) {
			/* Polyphase resampler path — single resampler for I+Q
			 * 32 taps/phase × 32 phases = 1024 total taps
			 * gives ~75 dB stopband for strong adjacent channel rejection.
			 * Using polyphase_resample_iq() keeps I/Q timing locked. */
			double cutoff = (double)target_rate * 0.45;
			int rc = polyphase_init_taps(&channels[num].poly_iq,
			                        (double)samplerate, (double)target_rate,
			                        cutoff, 32, 32);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "Failed to init polyphase for ARFCN %d\n",
				     arfcns[i].arfcn);
				continue;
			}
			channels[num].use_polyphase = 1;
			channels[num].mixer_phase = 0.0;
			if (fastmath_enabled())
				channels[num].mixer_step = fastmath_hz_to_step(-offset, (double)samplerate);
			else
				channels[num].mixer_step = -2.0 * M_PI * offset / (double)samplerate;
			out_rate = target_rate;
			decimation = samplerate / target_rate;
			if (verbosity >= 1) {
				LOGP(DSDR, LOGL_INFO,
				     "  Polyphase dbg ARFCN %d: in=%d out=%d distance=%.6f "
				     "cutoff=%.0fHz mixer_step=%.9f\n",
				     arfcns[i].arfcn, samplerate, out_rate,
				     channels[num].poly_iq.distance, cutoff,
				     channels[num].mixer_step);
			}
		} else {
			/* Halfband channelizer path (original) */
			int rc = channelizer_init(&channels[num].channelizer,
			                          samplerate, target_rate,
			                          (int)offset, fast_math);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "Failed to init channelizer for ARFCN %d\n",
				     arfcns[i].arfcn);
				continue;
			}
			channels[num].use_polyphase = 0;
			out_rate = channelizer_get_output_rate(&channels[num].channelizer);
			decimation = channelizer_get_decimation(&channels[num].channelizer);
		}

		int rc2 = fcch_detect_init(&channels[num].detector, (double)out_rate);
		if (rc2 < 0) {
			if (use_polyphase) {
				polyphase_free(&channels[num].poly_iq);
			} else {
				channelizer_exit(&channels[num].channelizer);
			}
			continue;
		}

		channels[num].max_out = samplerate / decimation + 16;
		channels[num].i_out = calloc(channels[num].max_out, sizeof(sample_t));
		channels[num].q_out = calloc(channels[num].max_out, sizeof(sample_t));
		if (!channels[num].i_out || !channels[num].q_out) {
			fcch_detect_exit(&channels[num].detector);
			if (use_polyphase) {
				polyphase_free(&channels[num].poly_iq);
			} else {
				channelizer_exit(&channels[num].channelizer);
			}
			free(channels[num].i_out);
			free(channels[num].q_out);
			continue;
		}

		channels[num].freq_offset = offset;
		channels[num].abs_freq = arfcns[i].freq;
		channels[num].arfcn = arfcns[i].arfcn;
		channels[num].active = 1;
		channels[num].poly_debug_blocks = 0;

		selected_arfcns[num_selected++] = arfcns[i].arfcn;

		if (verbosity >= 1)
			LOGP(DSDR, LOGL_INFO,
			     "  Ch %d: ARFCN %d  %.3f MHz  %.1f dBFS  "
			     "offset %+.0f Hz  decim x%d -> %d Hz%s\n",
			     num, arfcns[i].arfcn, arfcns[i].freq / 1e6,
			     arfcns[i].power_db, offset,
			     decimation, out_rate,
			     use_polyphase ? " [polyphase]" : "");
		num++;
	}

	return num;
}

static void cleanup_channels(calib_channel_t *channels, int num)
{
	for (int i = 0; i < num; i++) {
		if (!channels[i].active) continue;
		if (channels[i].use_polyphase) {
			polyphase_free(&channels[i].poly_iq);
		} else {
			channelizer_exit(&channels[i].channelizer);
		}
		fcch_detect_exit(&channels[i].detector);
		free(channels[i].i_out);
		free(channels[i].q_out);
	}
}

static double channel_power_db(const float *iq, int count)
{
	if (count <= 0) return -999.0;
	double sum = 0;
	for (int i = 0; i < count; i++) {
		float re = iq[i * 2];
		float im = iq[i * 2 + 1];
		sum += re * re + im * im;
	}
	double rms = sqrt(sum / count);
	return 20.0 * log10(rms > 0 ? rms : 1e-30);
}

/* ================================================================
 * Calibrate a single band: scan + detect
 * ================================================================ */

int calibrate_band(double center_freq, int samplerate,
                   int gsm_band, calibrate_config_t *config,
                   calibrate_result_t *result)
{
	calib_channel_t channels[CALIB_MAX_CHANNELS];
	float *iq_buffer = NULL;
	float *ch_iq = NULL;
	float offsets[AVG_COUNT];
	float snrs[AVG_COUNT];
	debug_sample_t debug_samples[DEBUG_MAX_DETECTIONS];
	int debug_sample_count = 0;
	int measurement_count = 0;
	int rejected_count = 0;
	int total_reads = 0;
	int num_channels;
	int detect_arfcn = -1;
	int arfcn_start, arfcn_end;
	double band_center, band_bw;

	memset(result, 0, sizeof(*result));
	memset(channels, 0, sizeof(channels));
	result->gsm_band = gsm_band;

	gsm_band_params(gsm_band, &band_center, &band_bw,
	                &arfcn_start, &arfcn_end);

	int max_ch = config->num_channels;
	if (max_ch < 1) max_ch = 1;
	if (max_ch > CALIB_MAX_CHANNELS) max_ch = CALIB_MAX_CHANNELS;

	LOGP(DSDR, LOGL_INFO,
	     "\n=== Phase 1: Scanning %s band for active channels ===\n",
	     gsm_band_name(gsm_band));

	/* Phase 1: Scan entire band */
	int max_arfcns = arfcn_end - arfcn_start + 1;
	arfcn_power_t *scan_results = calloc(max_arfcns, sizeof(arfcn_power_t));
	if (!scan_results)
		return -ENOMEM;

	int num_scanned = scan_band(samplerate, gsm_band,
	                            arfcn_start, arfcn_end,
	                            scan_results, max_arfcns,
	                            config->verbosity);
	if (num_scanned <= 0) {
		LOGP(DSDR, LOGL_ERROR, "%s: band scan failed or found no ARFCNs\n",
		     gsm_band_name(gsm_band));
		free(scan_results);
		return -ENODATA;
	}

	int show = num_scanned < 15 ? num_scanned : 15;
	LOGP(DSDR, LOGL_INFO, "%s scan complete: %d ARFCNs measured. Strongest:\n",
	     gsm_band_name(gsm_band), num_scanned);
	for (int i = 0; i < show; i++)
		LOGP(DSDR, LOGL_INFO, "  ARFCN %3d  %.3f MHz  %+.1f dBFS\n",
		     scan_results[i].arfcn, scan_results[i].freq / 1e6,
		     scan_results[i].power_db);

	/* Noise floor as median */
	double *all_powers = calloc(num_scanned, sizeof(double));
	for (int i = 0; i < num_scanned; i++)
		all_powers[i] = scan_results[i].power_db;
	qsort(all_powers, num_scanned, sizeof(double), compare_double);
	double noise_floor = all_powers[num_scanned / 2];
	free(all_powers);

	LOGP(DSDR, LOGL_INFO, "Noise floor estimate: %.1f dBFS\n", noise_floor);

	double threshold = noise_floor + 6.0;
	int num_active = 0;
	for (int i = 0; i < num_scanned; i++) {
		if (scan_results[i].power_db >= threshold)
			num_active++;
		else
			break;
	}

	if (num_active == 0) {
		LOGP(DSDR, LOGL_NOTICE,
		     "%s: no ARFCNs above threshold (%.1f dBFS). "
		     "Strongest: ARFCN %d at %.1f dBFS\n",
		     gsm_band_name(gsm_band), threshold,
		     scan_results[0].arfcn, scan_results[0].power_db);
		free(scan_results);
		return -ENODATA;
	}

	LOGP(DSDR, LOGL_INFO, "%d ARFCNs above threshold (%.1f dBFS)\n",
	     num_active, threshold);

	(void)center_freq;
	double detect_center;
	arfcn_power_t forced_entry;
	arfcn_power_t winner_entry;
	arfcn_power_t *detect_list = scan_results;
	int detect_list_count = num_active;

	if (config->forced_arfcn > 0) {
		int found = -1;
		for (int i = 0; i < num_scanned; i++) {
			if (scan_results[i].arfcn == config->forced_arfcn) {
				found = i;
				break;
			}
		}
		if (found < 0) {
			LOGP(DSDR, LOGL_ERROR,
			     "Forced ARFCN %d not found in scan results\n",
			     config->forced_arfcn);
			free(scan_results);
			return -ENODATA;
		}
		forced_entry = scan_results[found];
		detect_center = forced_entry.freq;
		detect_arfcn = forced_entry.arfcn;
		detect_list = &forced_entry;
		detect_list_count = 1;
		LOGP(DSDR, LOGL_NOTICE,
		     "Using forced ARFCN %d at %.3f MHz for FCCH detection\n",
		     detect_arfcn, detect_center / 1e6);
	} else {
		int best_fcch_idx = choose_fcch_reference_arfcn(samplerate,
							       scan_results,
							       num_active,
							       config->verbosity,
							       config->probe_sequential);
		if (best_fcch_idx >= 0) {
			detect_center = scan_results[best_fcch_idx].freq;
			detect_arfcn = scan_results[best_fcch_idx].arfcn;
			/* Use the winning ARFCN directly in phase 2. If we only retune
			 * center while keeping the full power-sorted list, adjacent
			 * stronger channels can replace the winner during setup. */
			winner_entry = scan_results[best_fcch_idx];
			detect_list = &winner_entry;
			detect_list_count = 1;
			if (config->verbosity >= 1) {
				LOGP(DSDR, LOGL_INFO,
				     "Using FCCH pre-validation winner ARFCN %d at %.3f MHz for phase 2\n",
				     detect_arfcn, detect_center / 1e6);
			}
		} else {
			LOGP(DSDR, LOGL_NOTICE,
			     "FCCH pre-validation found no reliable winner. "
			     "Use --arfcn <n> or improve SNR/antenna to avoid wrong auto-lock\n");
			free(scan_results);
			return -ENODATA;
		}
	}

	/* Phase 2: Retune and detect */
	LOGP(DSDR, LOGL_INFO,
	     "\n=== Phase 2: %s FCCH detection at %.3f MHz ===\n",
	     gsm_band_name(gsm_band), detect_center / 1e6);

	int rc = retune_rx(detect_center);
	if (rc < 0) {
		LOGP(DSDR, LOGL_ERROR, "Failed to retune to %.3f MHz\n",
		     detect_center / 1e6);
		free(scan_results);
		return rc;
	}

	num_channels = setup_channels_for_arfcns(channels,
	                                         (config->forced_arfcn > 0) ? 1 : max_ch,
	                                         samplerate, detect_center,
	                                         detect_list, detect_list_count,
	                                         config->fast_math,
	                                         config->use_channelizer,
	                                         config->verbosity);
	free(scan_results);

	if (num_channels == 0) {
		LOGP(DSDR, LOGL_ERROR, "No channels could be set up for detection\n");
		return -EINVAL;
	}

	LOGP(DSDR, LOGL_INFO,
	     "Monitoring %d channels, need %d measurements (timeout %.0fs)\n",
	     num_channels, AVG_COUNT, config->timeout_sec);

	int buffer_samples = (int)(samplerate * 0.05);
	if (buffer_samples > 100000)
		buffer_samples = 100000;
	iq_buffer = calloc(buffer_samples * 2, sizeof(float));
	int max_out_samples = buffer_samples + 16;
	ch_iq = calloc(max_out_samples * 2, sizeof(float));
	/* Mixer buffers for polyphase path (reused across iterations) */
	sample_t *mix_i = NULL, *mix_q = NULL;
	if (config->use_channelizer) {
		mix_i = calloc(buffer_samples, sizeof(sample_t));
		mix_q = calloc(buffer_samples, sizeof(sample_t));
	}
	if (!iq_buffer || !ch_iq || (config->use_channelizer && (!mix_i || !mix_q))) {
		free(iq_buffer); free(ch_iq); free(mix_i); free(mix_q);
		cleanup_channels(channels, num_channels);
		return -ENOMEM;
	}

	/* Flush stale samples from SDR (direct read, before ring buffer) */
	flush_rx(iq_buffer, buffer_samples, samplerate);

	/* Start threaded ring buffer reader.
	 * 1 second of buffering absorbs processing jitter at any sample rate.
	 * Chunk size 8192 balances USB transfer efficiency vs latency. */
	iqringbuf_t rb;
	rc = iqringbuf_init(&rb, 1.0, samplerate, 8192, receive_iq);
	if (rc < 0) {
		free(iq_buffer); free(ch_iq); free(mix_i); free(mix_q);
		cleanup_channels(channels, num_channels);
		return -ENOMEM;
	}
	rc = iqringbuf_start(&rb);
	if (rc < 0) {
		iqringbuf_free(&rb);
		free(iq_buffer); free(ch_iq); free(mix_i); free(mix_q);
		cleanup_channels(channels, num_channels);
		return -ENOMEM;
	}

	time_t start_time = time(NULL);
	int max_reads = 5000;
	int logged_channels = 0;

	/* EMA-smoothed offset for convergence tracking
	 * (inspired by SDRangel freqtracker alphaEMA approach) */
	double ema_offset = 0.0;
	int ema_initialized = 0;
	const double ema_alpha = 0.15;  /* smoothing factor */

	while (measurement_count < AVG_COUNT && total_reads < max_reads) {
		int samples_received;

		if (difftime(time(NULL), start_time) > config->timeout_sec) {
			LOGP(DSDR, LOGL_NOTICE,
			     "Timeout after %.0fs (%d/%d measurements)\n",
			     config->timeout_sec,
			     measurement_count, AVG_COUNT);
			break;
		}

		/* Read from ring buffer — never blocks the SDR */
		samples_received = iqringbuf_read(&rb, iq_buffer, buffer_samples);
		if (samples_received < buffer_samples / 2) {
			total_reads++;
			continue;
		}

		for (int c = 0; c < num_channels && measurement_count < AVG_COUNT; c++) {
			if (!channels[c].active) continue;

			int out_count;

			if (channels[c].use_polyphase) {
				/* Polyphase path: mix down + resample I/Q together */

				/* NCO mixer: shift channel to baseband */
				double phase = channels[c].mixer_phase;
				double step = channels[c].mixer_step;
				if (fastmath_enabled()) {
					for (int s = 0; s < samples_received; s++) {
						double cos_p, sin_p;
						fastmath_sincos(phase, &sin_p, &cos_p);
						float si = iq_buffer[s * 2];
						float sq = iq_buffer[s * 2 + 1];
						mix_i[s] = si * cos_p - sq * sin_p;
						mix_q[s] = si * sin_p + sq * cos_p;
						phase += step;
						if (phase >= 65536.0) phase -= 65536.0;
						else if (phase < 0.0) phase += 65536.0;
					}
				} else {
					for (int s = 0; s < samples_received; s++) {
						double cos_p = cos(phase);
						double sin_p = sin(phase);
						float si = iq_buffer[s * 2];
						float sq = iq_buffer[s * 2 + 1];
						mix_i[s] = si * cos_p - sq * sin_p;
						mix_q[s] = si * sin_p + sq * cos_p;
						phase += step;
					}
					phase = fmod(phase, 2.0 * M_PI);
				}
				channels[c].mixer_phase = phase;

				/* Resample I and Q together — single resampler
				 * keeps fractional timing locked between I and Q */
				out_count = polyphase_resample_iq(&channels[c].poly_iq,
				                                  mix_i, mix_q,
				                                  samples_received,
				                                  channels[c].i_out,
				                                  channels[c].q_out,
				                                  channels[c].max_out);
				if (channels[c].poly_debug_blocks < 12 && out_count > 0) {
					double eff_rate = ((double)out_count * samplerate) / samples_received;
					LOGP(DSDR, LOGL_INFO,
					     "  Polyphase block dbg ch=%d in=%d out=%d eff_rate=%.1fHz "
					     "P=%d Q=%d\n",
					     c, samples_received, out_count, eff_rate,
					     channels[c].poly_iq.up,
					     channels[c].poly_iq.down);
					channels[c].poly_debug_blocks++;
				}
			} else {
				out_count = channelizer_process(&channels[c].channelizer,
				                                iq_buffer, samples_received,
				                                channels[c].i_out,
				                                channels[c].q_out);
			}
			if (out_count <= 0) continue;

			for (int s = 0; s < out_count; s++) {
				ch_iq[s * 2]     = (float)channels[c].i_out[s];
				ch_iq[s * 2 + 1] = (float)channels[c].q_out[s];
			}

			if (!logged_channels) {
				double pwr = channel_power_db(ch_iq, out_count);
				LOGP(DSDR, LOGL_INFO,
				     "  Ch %d ARFCN %d (%.3f MHz): RX power %.1f dBFS\n",
				     c, channels[c].arfcn,
				     channels[c].abs_freq / 1e6, pwr);
			}

			double offset, snr_db;
			rc = fcch_detect_process(&channels[c].detector,
			                         ch_iq, out_count,
			                         &offset, &snr_db);
			if (rc > 0) {
				double total_offset;
				int dbg_idx = -1;

				if (channels[c].use_polyphase) {
					/* Polyphase path: NCO mixer shifts channel
					 * exactly to baseband, so detector output
					 * is the pure clock error directly. */
					total_offset = offset;
				} else {
					/* Halfband channelizer: discrete frequency
					 * steps may leave a residual offset that the
					 * channelizer couldn't remove. Add it back. */
					total_offset = offset
					    + channels[c].channelizer.center_offset;
				}

				if (debug_sample_count < DEBUG_MAX_DETECTIONS) {
					dbg_idx = debug_sample_count++;
					debug_samples[dbg_idx].offset = (float)total_offset;
					debug_samples[dbg_idx].snr = (float)snr_db;
					debug_samples[dbg_idx].accepted = 1;
					debug_samples[dbg_idx].peak_bin = channels[c].detector.last_peak_bin;
				}

				/* Online outlier rejection: once we have enough
				 * measurements, reject values far from the median.
				 * This catches spurious peaks (adjacent channel SCH,
				 * DC spurs) that would otherwise corrupt the average. */
				if (ENABLE_ONLINE_REJECT && measurement_count >= MIN_FOR_REJECT) {
					double med = running_median(offsets, measurement_count);
					if (fabs(total_offset - med) > OUTLIER_MAX_HZ) {
						rejected_count++;
						if (dbg_idx >= 0)
							debug_samples[dbg_idx].accepted = 0;
						LOGP(DSDR, LOGL_INFO,
						     "  [rejected] ARFCN %d: %+.1f Hz "
						     "(median %+.1f Hz, delta %.0f Hz)\n",
						     channels[c].arfcn,
						     total_offset, med,
						     fabs(total_offset - med));
						fcch_detect_reset(&channels[c].detector);
						continue;
					}
				}

				/* EMA smoothing */
				if (!ema_initialized) {
					ema_offset = total_offset;
					ema_initialized = 1;
				} else {
					ema_offset = ema_alpha * total_offset
					           + (1.0 - ema_alpha) * ema_offset;
				}

				offsets[measurement_count] = (float)total_offset;
				snrs[measurement_count] = (float)snr_db;
				measurement_count++;

				LOGP(DSDR, LOGL_INFO,
				     "  #%d ARFCN %d (%.3f MHz): offset %+.1f Hz "
				     "(EMA %+.1f Hz), SNR %.1f dB, burst_len %d samples\n",
				     measurement_count, channels[c].arfcn,
				     channels[c].abs_freq / 1e6,
				     total_offset, ema_offset, snr_db,
				     channels[c].detector.last_burst_len);

				fcch_detect_reset(&channels[c].detector);
			}
		}

		if (!logged_channels) logged_channels = 1;
		total_reads++;

		if (config->verbosity >= 1 && total_reads % 200 == 0) {
			LOGP(DSDR, LOGL_INFO,
			     "--- %d/%d measurements, %.0fs elapsed ---\n",
			     measurement_count, AVG_COUNT,
			     difftime(time(NULL), start_time));
			for (int c = 0; c < num_channels; c++) {
				if (!channels[c].active) continue;
				LOGP(DSDR, LOGL_INFO,
				     "  Ch %d ARFCN %d: last_burst %d samp / %d scan frames, "
				     "SNR %.1f dB, found %d, rejected %d\n",
				     c, channels[c].arfcn,
				     channels[c].detector.last_burst_len,
				     channels[c].detector.total_frames,
				     channels[c].detector.last_snr_db,
				     channels[c].detector.bursts_found,
				     channels[c].detector.bursts_rejected);
			}

			/* Deactivate dead channels: if a channel has processed
			 * many frames but found nothing, stop wasting CPU on it */
			int active_remaining = 0;
			for (int c = 0; c < num_channels; c++) {
				if (!channels[c].active) continue;
				if (channels[c].detector.total_frames >= DEAD_CHANNEL_FRAMES
				    && channels[c].detector.bursts_found == 0) {
					LOGP(DSDR, LOGL_INFO,
					     "  Ch %d ARFCN %d: no detections after %d frames, deactivating\n",
					     c, channels[c].arfcn,
					     channels[c].detector.total_frames);
					channels[c].active = 0;
				} else {
					active_remaining++;
				}
			}
			if (active_remaining == 0) {
				LOGP(DSDR, LOGL_NOTICE,
				     "All channels deactivated — no FCCH signal found in %s\n",
				     gsm_band_name(gsm_band));
				break;
			}
		}
	}

	free(iq_buffer);
	free(ch_iq);
	free(mix_i);
	free(mix_q);

	/* Stop ring buffer reader thread */
	iqringbuf_stop(&rb);
	if (rb.overflows > 0)
		LOGP(DSDR, LOGL_NOTICE,
		     "Ring buffer: %d overflows (%.1f%% data loss)\n",
		     rb.overflows,
		     100.0 * rb.overflows / (rb.total_read > 0 ? rb.total_read : 1));
	iqringbuf_free(&rb);

	/* Final stats */
	int detector_found_total = 0;
	int detector_rejected_total = 0;
	LOGP(DSDR, LOGL_INFO, "%s channel results:\n", gsm_band_name(gsm_band));
	for (int c = 0; c < num_channels; c++) {
		if (!channels[c].active) continue;
		detector_found_total += channels[c].detector.bursts_found;
		detector_rejected_total += channels[c].detector.bursts_rejected;
		LOGP(DSDR, LOGL_INFO,
		     "  Ch %d ARFCN %d (%.3f MHz): %d detected, %d rejected, "
		     "SNR %.1f dB\n",
		     c, channels[c].arfcn, channels[c].abs_freq / 1e6,
		     channels[c].detector.bursts_found,
		     channels[c].detector.bursts_rejected,
		     channels[c].detector.last_snr_db);
	}

	if (debug_sample_count > 0)
		debug_dump_offset_families(debug_samples, debug_sample_count);

	cleanup_channels(channels, num_channels);

	if (measurement_count < 3) {
		LOGP(DSDR, LOGL_ERROR,
		     "%s: insufficient measurements (%d/%d)\n",
		     gsm_band_name(gsm_band), measurement_count, AVG_COUNT);
		return -ENODATA;
	}

	int trim = (measurement_count > 2 * AVG_THRESHOLD) ? AVG_THRESHOLD : 0;
	double stddev;
	double avg_offset = trimmed_mean(offsets, measurement_count, trim, &stddev);
	double guarded_offset = 0.0, guarded_stddev = 0.0;
	if (select_near_center_family(offsets, snrs, measurement_count,
				      &guarded_offset, &guarded_stddev)) {
		LOGP(DSDR, LOGL_NOTICE,
		     "%s family guard: overriding far dominant family with near-center family "
		     "(%+.1f Hz, stddev %.1f Hz)\n",
		     gsm_band_name(gsm_band), guarded_offset, guarded_stddev);
		avg_offset = guarded_offset;
		stddev = guarded_stddev;
	}
	double cluster_offset, cluster_stddev;
	if (estimate_high_snr_cluster(offsets, snrs, measurement_count,
	                              &cluster_offset, &cluster_stddev)) {
		/* Prefer dominant high-SNR cluster when it is clearly tighter. */
		if (cluster_stddev > 0.0 && (stddev <= 0.0 || cluster_stddev < stddev * 0.8)) {
			avg_offset = cluster_offset;
			stddev = cluster_stddev;
			LOGP(DSDR, LOGL_INFO,
			     "%s using high-SNR cluster estimate: %+.1f Hz, stddev %.1f Hz\n",
			     gsm_band_name(gsm_band), avg_offset, stddev);
		}
	}

	double accepted_snr_sum = 0.0;
	for (int i = 0; i < measurement_count; i++)
		accepted_snr_sum += snrs[i];
	double accepted_snr_avg = accepted_snr_sum / measurement_count;
	double reject_ratio = (double)detector_rejected_total /
			      (double)((detector_found_total > 0) ? detector_found_total : 1);
	double count_conf = clamp01((double)measurement_count / AVG_COUNT);
	double snr_conf = clamp01((accepted_snr_avg - RESULT_MIN_ACCEPTED_SNR_DB) /
				  (RESULT_FULL_ACCEPTED_SNR_DB - RESULT_MIN_ACCEPTED_SNR_DB));
	double reject_conf = clamp01((RESULT_REJECT_RATIO_BAD - reject_ratio) /
				     (RESULT_REJECT_RATIO_BAD - RESULT_REJECT_RATIO_GOOD));
	double quality_conf = (0.2 + 0.8 * snr_conf) * (0.2 + 0.8 * reject_conf);

	result->hz_offset = avg_offset;
	result->ppm_offset = (avg_offset / detect_center) * 1e6;
	result->freq_hz = detect_center;
	result->arfcn = detect_arfcn;
	result->signal_type = CALIB_SIGNAL_GSM_FCCH;
	result->confidence = count_conf * quality_conf;
	result->gsm_band = gsm_band;
	result->stddev_hz = stddev;
	result->num_measurements = measurement_count;
	result->valid = 1;

	if (accepted_snr_avg < RESULT_MIN_ACCEPTED_SNR_DB ||
	    reject_ratio > RESULT_REJECT_RATIO_BAD) {
		LOGP(DSDR, LOGL_NOTICE,
		     "%s low-quality lock warning: accepted_snr=%.1f dB reject_ratio=%.2f "
		     "(confidence %.0f%%)\n",
		     gsm_band_name(gsm_band), accepted_snr_avg, reject_ratio,
		     result->confidence * 100.0);
	}

	/* Polyphase path sanity check:
	 * If it converges to a very large offset, this is likely a spur/alias
	 * lock (observed around ~-19.6 kHz). Signal caller to retry with the
	 * halfband channelizer path for a safer result. */
	if (config->use_channelizer && fabs(avg_offset) > 10000.0) {
		LOGP(DSDR, LOGL_NOTICE,
		     "%s polyphase result looks implausible (%+.1f Hz), "
		     "requesting halfband retry\n",
		     gsm_band_name(gsm_band), avg_offset);
		return -ERANGE;
	}

	LOGP(DSDR, LOGL_INFO,
	     "%s result: %d measurements, stddev %.1f Hz, "
	     "rejected %d online, trimmed %d post-hoc, accepted_snr %.1f dB, "
	     "reject_ratio %.2f, confidence %.0f%%\n",
	     gsm_band_name(gsm_band), measurement_count, stddev,
	     rejected_count, trim * 2,
	     accepted_snr_avg, reject_ratio, result->confidence * 100.0);
	LOGP(DSDR, LOGL_INFO,
	     "%s offset: %+.2f Hz (%.3f ppm) [EMA: %+.1f Hz]\n",
	     gsm_band_name(gsm_band), avg_offset, result->ppm_offset,
	     ema_offset);

	return 0;
}

/* ================================================================
 * Main entry point: scan one or all bands
 * ================================================================ */

int calibrate_auto(double center_freq, int samplerate,
                   calibrate_config_t *config,
                   calibrate_result_t *results, int *num_results)
{
	(void)center_freq;  /* each band determines its own center */
	int bands_to_scan[CALIB_MAX_BANDS];
	int num_bands = 0;
	int valid_count = 0;

	*num_results = 0;

	if (config->gsm_bands == GSM_BAND_ALL) {
		for (int i = 0; i < NUM_ALL_BANDS && i < CALIB_MAX_BANDS; i++)
			bands_to_scan[num_bands++] = all_bands[i];
	} else {
		bands_to_scan[num_bands++] = config->gsm_bands;
	}

	for (int b = 0; b < num_bands; b++) {
		int band = bands_to_scan[b];
		double bc, bw;
		int as, ae;

		gsm_band_params(band, &bc, &bw, &as, &ae);

		int rc = calibrate_band(bc, samplerate, band, config,
		                        &results[valid_count]);
		if (rc == -ERANGE && config->use_channelizer) {
			calibrate_config_t fallback = *config;
			fallback.use_channelizer = 0;
			LOGP(DSDR, LOGL_NOTICE,
			     "%s: retrying with halfband channelizer path\n",
			     gsm_band_name(band));
			rc = calibrate_band(bc, samplerate, band, &fallback,
			                    &results[valid_count]);
		}
		if (rc == 0) {
			valid_count++;
		} else {
			LOGP(DSDR, LOGL_NOTICE,
			     "%s: no result (rc=%d), skipping\n",
			     gsm_band_name(band), rc);
		}
	}

	*num_results = valid_count;

	if (valid_count == 0)
		return -ENODATA;

	/* If multiple bands succeeded, compute weighted average */
	if (valid_count > 1) {
		double weighted_ppm = 0.0;
		double total_weight = 0.0;

		LOGP(DSDR, LOGL_INFO, "\n=== Multi-band summary ===\n");
		for (int i = 0; i < valid_count; i++) {
			/* Weight by confidence × (1/stddev) */
			double w = results[i].confidence;
			if (results[i].stddev_hz > 0)
				w /= results[i].stddev_hz;
			else
				w *= 10.0;  /* very good stddev */

			weighted_ppm += results[i].ppm_offset * w;
			total_weight += w;

			LOGP(DSDR, LOGL_INFO,
			     "  %s: %+.2f Hz (%+.3f ppm), stddev %.1f Hz, "
			     "%d measurements, confidence %.0f%%\n",
			     gsm_band_name(results[i].gsm_band),
			     results[i].hz_offset, results[i].ppm_offset,
			     results[i].stddev_hz, results[i].num_measurements,
			     results[i].confidence * 100);
		}

		if (total_weight > 0) {
			double avg_ppm = weighted_ppm / total_weight;
			LOGP(DSDR, LOGL_INFO,
			     "  Weighted average: %.3f ppm\n", avg_ppm);
		}
	}

	return 0;
}
