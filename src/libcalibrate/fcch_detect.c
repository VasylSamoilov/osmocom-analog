/* FCCH Burst Detector - adaptive error detector with tone estimation
 *
 * This implementation is inspired by the kalibrate BSD-licensed FCCH
 * detector (Joshua Lackey), itself based on:
 *   Varma, Sahu, Charan - "Robust Frequency Burst Detection Algorithm
 *   for GSM / GPRS".
 *
 * We use adaptive-filter normalized error to find pure-tone neighborhoods,
 * then estimate tone frequency from that neighborhood.
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

#include "fcch_detect.h"
#include "../libfft/fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DETECT_FFT_SIZE 1024
#define DETECT_FFT_LOG2 10

/* Tuned to match kalibrate-like behavior while keeping false locks down. */
#define ADAPT_DEFAULT_D 8
#define ADAPT_DEFAULT_P (1.0 / 32.0)
#define ADAPT_DEFAULT_G (1.0 / 12.5)
#define ADAPT_FILTER_DELAY 8
#define PRECOND_TARGET_RMS 0.20
#define PRECOND_MIN_RMS 1e-6

static inline double c_norm2(double re, double im)
{
	return re * re + im * im;
}

static inline void iq_at(const float *iq, int idx, double *re, double *im)
{
	*re = iq[idx * 2];
	*im = iq[idx * 2 + 1];
}

/* Instantaneous-frequency variance (Hz^2): pure tones are low variance,
 * modulated bursts (like SCH) are much higher. */
static double inst_freq_var_hz2(const float *iq, int n, double sample_rate)
{
	if (n < 8)
		return 0.0;
	double sum = 0.0, sum2 = 0.0;
	int m = 0;
	for (int i = 1; i < n; i++) {
		double r0 = iq[(i - 1) * 2], i0 = iq[(i - 1) * 2 + 1];
		double r1 = iq[i * 2],       i1 = iq[i * 2 + 1];
		double pr = r1 * r0 + i1 * i0;
		double pi = i1 * r0 - r1 * i0;
		double dphi = atan2(pi, pr);
		double hz = dphi * sample_rate / (2.0 * M_PI);
		sum += hz;
		sum2 += hz * hz;
		m++;
	}
	if (m < 4)
		return 0.0;
	double mean = sum / m;
	double var = (sum2 / m) - mean * mean;
	return (var > 0.0) ? var : 0.0;
}

static int detect_tone_from_segment(fcch_detect_t *det, const float *iq, int n,
				    double *offset_hz, double *snr_db,
				    int *peak_bin, double *peak_hz)
{
	if (n < 64)
		return 0;

	double fft_r[DETECT_FFT_SIZE];
	double fft_i[DETECT_FFT_SIZE];
	memset(fft_r, 0, sizeof(fft_r));
	memset(fft_i, 0, sizeof(fft_i));

	int copy_n = (n < DETECT_FFT_SIZE) ? n : DETECT_FFT_SIZE;
	for (int i = 0; i < copy_n; i++) {
		fft_r[i] = iq[i * 2];
		fft_i[i] = iq[i * 2 + 1];
	}

	fft_process(1, DETECT_FFT_LOG2, fft_r, fft_i);

	double bin_hz = det->sample_rate / (double)DETECT_FFT_SIZE;
	int tone_bin = (int)lround(FCCH_FREQ / bin_hz);
	int search_bins = (int)ceil(FCCH_SEARCH_HZ / bin_hz);
	int lo = tone_bin - search_bins;
	int hi = tone_bin + search_bins;
	if (lo < 2) lo = 2;
	if (hi > DETECT_FFT_SIZE / 2 - 2) hi = DETECT_FFT_SIZE / 2 - 2;
	if (hi <= lo)
		return 0;

	int best = lo;
	double best_mag = c_norm2(fft_r[lo], fft_i[lo]);
	double sum = 0.0;
	int cnt = 0;
	for (int b = lo; b <= hi; b++) {
		double mag = c_norm2(fft_r[b], fft_i[b]);
		if (mag > best_mag) {
			best_mag = mag;
			best = b;
		}
		sum += mag;
		cnt++;
	}
	if (cnt < 4 || best_mag <= 0.0)
		return 0;

	double avg = (sum - best_mag) / (double)(cnt - 1);
	if (avg <= 0.0)
		return 0;
	double snr = 10.0 * log10(best_mag / avg);
	if (snr < FCCH_MIN_SNR_DB)
		return 0;

	double alpha = c_norm2(fft_r[best - 1], fft_i[best - 1]);
	double beta  = best_mag;
	double gamma = c_norm2(fft_r[best + 1], fft_i[best + 1]);
	double denom = alpha - 2.0 * beta + gamma;
	double delta = 0.0;
	if (fabs(denom) > 1e-30)
		delta = 0.5 * (alpha - gamma) / denom;

	double freq = ((double)best + delta) * bin_hz;
	double off = freq - FCCH_FREQ;
	if (fabs(off) > FCCH_OFFSET_MAX)
		return 0;

	if (offset_hz) *offset_hz = off;
	if (snr_db) *snr_db = snr;
	if (peak_bin) *peak_bin = best;
	if (peak_hz) *peak_hz = freq;
	return 1;
}

static int scan_for_fcch(fcch_detect_t *det, double *offset, double *snr_db,
			 int *consumed_samples)
{
	const float *s = det->scan_iq;
	int s_len = det->scan_count;
	if (s_len < 256)
		return 0;

	double sps = det->sample_rate / GSM_RATE;
	int min_fb_len = (int)lround(100.0 * sps);
	int fcch_len = (int)lround(148.0 * sps);
	int frame_len = (int)lround(156.25 * sps);
	if (min_fb_len < 64) min_fb_len = 64;
	if (fcch_len < min_fb_len) fcch_len = min_fb_len;

	int max_err = s_len;
	double *errs = malloc(max_err * sizeof(double));
	int *err_pos = malloc(max_err * sizeof(int));
	if (!errs || !err_pos) {
		free(errs);
		free(err_pos);
		return 0;
	}

	int err_count = 0;
	double err_sum = 0.0;
	int n0 = det->adapt_w_len - 1;
	for (int n = n0; n + det->adapt_delay < s_len; n++) {
		double e_in = 0.0;
		double y_re = 0.0, y_im = 0.0;
		for (int i = 0; i < det->adapt_w_len; i++) {
			double xr, xi;
			iq_at(s, n - i, &xr, &xi);
			e_in += c_norm2(xr, xi);
			y_re += det->w_real[i] * xr + det->w_imag[i] * xi;
			y_im += det->w_real[i] * xi - det->w_imag[i] * xr;
		}
		e_in /= det->adapt_w_len;
		if (e_in <= 1e-12)
			continue;

		double dr, di;
		iq_at(s, n + det->adapt_delay, &dr, &di);
		double er = dr - y_re;
		double ei = di - y_im;
		double en = c_norm2(er, ei);

		for (int i = 0; i < det->adapt_w_len; i++) {
			double xr, xi;
			iq_at(s, n - i, &xr, &xi);
			det->w_real[i] += det->adapt_g * (er * xr + ei * xi);
			det->w_imag[i] += det->adapt_g * (er * xi - ei * xr);
		}

		det->adapt_err_ema = (1.0 - det->adapt_p) * det->adapt_err_ema + det->adapt_p * en;
		double ratio = det->adapt_err_ema / e_in;
		errs[err_count] = ratio;
		err_pos[err_count] = n;
		err_sum += ratio;
		err_count++;
	}

	if (err_count < 64) {
		free(errs);
		free(err_pos);
		return 0;
	}

	double avg = err_sum / err_count;
	double limit = 0.7 * avg;
	int low = 0, run_start = 0;

	for (int i = 0; i < err_count; i++) {
		int is_low = (errs[i] <= limit);
		if (!low && is_low) {
			low = 1;
			run_start = i;
		} else if (low && !is_low) {
			int sidx = err_pos[run_start];
			int eidx = err_pos[i - 1];
			int run_len = eidx - sidx + 1;
			low = 0;

			if (run_len >= min_fb_len) {
				int y_len = (run_len < fcch_len) ? run_len : fcch_len;
				double off = 0.0, snr = 0.0, phz = 0.0;
				int pbin = 0;
				if (detect_tone_from_segment(det, s + sidx * 2, y_len,
							     &off, &snr, &pbin, &phz)) {
					/* SCH confirmation guard:
					 * In GSM TS0, SCH follows FCCH by one frame. FCCH is CW-like
					 * (low phase-variance), while SCH is data-modulated (higher
					 * phase-variance). Reject tone families that do not show this. */
					int sch_ok = 1;
					int sch_start = sidx + frame_len;
					/* Guard is for far-offset spur families. Do not penalize
					 * near-center candidates; those are handled by clustering. */
					if (fabs(off) >= 3000.0 && sch_start + fcch_len < s_len) {
						double var_fcch = inst_freq_var_hz2(s + sidx * 2, y_len, det->sample_rate);
						double var_sch  = inst_freq_var_hz2(s + sch_start * 2, fcch_len, det->sample_rate);
						/* Reject only when SCH frame looks too similar to FCCH
						 * (continuous-tone behavior) with a stricter ratio. */
						if (var_fcch > 0.0 && var_sch <= var_fcch * 1.30) {
							sch_ok = 0;
							det->bursts_rejected++;
						}
					}
					if (!sch_ok)
						continue;

					if (offset) *offset = off;
					if (snr_db) *snr_db = snr;
					det->last_snr_db = snr;
					det->last_peak_bin = pbin;
					det->last_peak_hz = phz;
					det->last_burst_len = y_len;
					det->bursts_found++;
					if (consumed_samples)
						*consumed_samples = sidx + y_len;
					free(errs);
					free(err_pos);
					return 1;
				}
			}
		}
	}

	/* If buffer gets crowded, consume old samples to keep runtime bounded. */
	if (consumed_samples && s_len > (FCCH_SCAN_BUF_SAMPLES * 3) / 4)
		*consumed_samples = s_len / 2;

	free(errs);
	free(err_pos);
	return 0;
}

int fcch_detect_init(fcch_detect_t *det, double sample_rate)
{
	memset(det, 0, sizeof(*det));
	det->sample_rate = sample_rate;
	det->bin_hz = sample_rate / (double)DETECT_FFT_SIZE;

	det->adapt_delay = ADAPT_DEFAULT_D;
	det->adapt_w_len = 2 * ADAPT_FILTER_DELAY + 1;
	det->adapt_p = ADAPT_DEFAULT_P;
	det->adapt_g = ADAPT_DEFAULT_G;
	det->adapt_err_ema = 0.0;

	det->w_real = calloc(det->adapt_w_len, sizeof(double));
	det->w_imag = calloc(det->adapt_w_len, sizeof(double));
	det->scan_iq = calloc(FCCH_SCAN_BUF_SAMPLES * 2, sizeof(float));
	if (!det->w_real || !det->w_imag || !det->scan_iq) {
		fcch_detect_exit(det);
		return -ENOMEM;
	}

	return 0;
}

int fcch_detect_process(fcch_detect_t *det, const float *iq_in, int num_samples,
			double *offset, double *snr_db)
{
	if (num_samples <= 0)
		return 0;

	det->total_frames += (num_samples >= 1024) ? (num_samples / 1024) : 1;

	/* Append input; if not enough room, drop oldest half. */
	if (det->scan_count + num_samples > FCCH_SCAN_BUF_SAMPLES) {
		int keep = det->scan_count / 2;
		memmove(det->scan_iq, det->scan_iq + (det->scan_count - keep) * 2,
			keep * 2 * sizeof(float));
		det->scan_count = keep;
	}
	int copy = num_samples;
	if (copy > FCCH_SCAN_BUF_SAMPLES - det->scan_count)
		copy = FCCH_SCAN_BUF_SAMPLES - det->scan_count;
	float *dst = det->scan_iq + det->scan_count * 2;
	memcpy(dst, iq_in, copy * 2 * sizeof(float));

	/* Quick front-end conditioning:
	 * - remove per-block DC (I/Q mean)
	 * - normalize RMS to a gentle target
	 * This improves detector stability when RF gain/device levels vary. */
	double mean_i = 0.0, mean_q = 0.0;
	for (int i = 0; i < copy; i++) {
		mean_i += dst[i * 2];
		mean_q += dst[i * 2 + 1];
	}
	mean_i /= copy;
	mean_q /= copy;
	double pwr = 0.0;
	for (int i = 0; i < copy; i++) {
		double re = dst[i * 2] - mean_i;
		double im = dst[i * 2 + 1] - mean_q;
		dst[i * 2] = (float)re;
		dst[i * 2 + 1] = (float)im;
		pwr += re * re + im * im;
	}
	double rms = sqrt(pwr / (double)copy);
	if (rms > PRECOND_MIN_RMS) {
		double g = PRECOND_TARGET_RMS / rms;
		if (g > 4.0) g = 4.0;
		if (g < 0.25) g = 0.25;
		for (int i = 0; i < copy; i++) {
			dst[i * 2] *= (float)g;
			dst[i * 2 + 1] *= (float)g;
		}
	}

	det->scan_count += copy;

	int consumed = 0;
	int rc = scan_for_fcch(det, offset, snr_db, &consumed);
	if (consumed > 0 && consumed <= det->scan_count) {
		memmove(det->scan_iq, det->scan_iq + consumed * 2,
			(det->scan_count - consumed) * 2 * sizeof(float));
		det->scan_count -= consumed;
	}
	if (rc > 0)
		return 1;

	/* Count non-detections as rejected attempts for diagnostics parity. */
	det->bursts_rejected++;
	return 0;
}

void fcch_detect_reset(fcch_detect_t *det)
{
	det->scan_count = 0;
	det->last_burst_len = 0;
	det->total_frames = 0;
	det->adapt_err_ema = 0.0;
	if (det->w_real)
		memset(det->w_real, 0, det->adapt_w_len * sizeof(double));
	if (det->w_imag)
		memset(det->w_imag, 0, det->adapt_w_len * sizeof(double));
}

void fcch_detect_exit(fcch_detect_t *det)
{
	free(det->w_real);
	free(det->w_imag);
	free(det->scan_iq);
	det->w_real = NULL;
	det->w_imag = NULL;
	det->scan_iq = NULL;
}

