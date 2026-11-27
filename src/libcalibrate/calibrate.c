/* SDR Clock Calibration - scan, then windows of channels
 *
 *   Scan: step across the GSM band, FFT power per ARFCN; the carriers above
 *     the noise floor, strongest first.
 *   Windows: tune so that the strongest carrier not measured yet and as many
 *     others as fit (and as the CPU budget allows) are in the band; separate
 *     them with libsdr's engine (the filter bank, or the per-channel chain
 *     for one or two), measure the FCCH offset of each, decode the SCH (and
 *     the BCCH SI) as the proof of a real cell. Next window, until enough
 *     confirmed towers agree.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

#include "calibrate.h"
#include "fcch_detect.h"
#include "../libsample/sample.h"
#include "../libsdr/sdr_config.h"
#include "../libsdr/sdr_engine.h"
#include "../liblogging/logging.h"
#include "../libfft/fft.h"

#include "../libsdr/sdr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Measurements per carrier */
#define AVG_COUNT       50      /* FCCH offsets to collect per carrier */
#define AVG_THRESHOLD   8       /* Outliers to trim from each end */

/* a carrier without an FCCH this long is no BCCH carrier (FCCH every 46 ms) */
#define NO_FCCH_SEC     2.0
/* nor one with fewer than 3 in this time */
#define FEW_FCCH_SEC    4.0

/* GSM channel spacing */
#define CHANNEL_SPACING 200000  /* 200 kHz */
/* no carrier centre closer to the DC (the LO sits in the middle of the
 * stream: its DC strip, tens of kHz, stays outside the carrier's +-90 kHz):
 * the window centre on the half-channel grid puts the nearest at 100 kHz */
#define DC_GUARD_HZ     (CHANNEL_SPACING / 2.0 - 1.0)

/* Scan parameters */
#define SCAN_FFT_SIZE   4096    /* FFT size for power scan */
#define SCAN_FFT_LOG2   12
#define SCAN_AVG_FRAMES 32      /* FFT frames to average during scan */
#define SCAN_SETTLE_MS  50      /* ms after a retune returned to discard (B210: 5.5 ms measured; other devices buffer more) */

/* channels measured in the first window, before the cost per channel is known */
#define FIRST_WINDOW_CHANNELS 3
/* the CPU measurement: after this long in the first window */
#define CPU_MEASURE_SEC 1.0

/* ARFCN scan result */
typedef struct {
	int    arfcn;
	double freq;        /* Absolute frequency Hz */
	double power_db;    /* Measured power dBFS */
	int    done;        /* measured (or skipped) */
} arfcn_power_t;

typedef struct {
	float offset;
	float snr;
} offset_snr_t;

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

static void *calib_sdr;		/* libsdr instance, the whole band (channels = 0) */
static int calib_sdr_chunk;	/* its buffer_size: most samples per sdr_read() */
static int calib_rate;		/* its rate (sleeps while the ring fills) */

void calibrate_set_sdr(void *sdr, int buffer_size)
{
	calib_sdr = sdr;
	calib_sdr_chunk = buffer_size;
	calib_rate = sdr ? sdr_get_samplerate(sdr) : 0;
}

/* IQ from libsdr's RX ring (interleaved float at the DSP rate): what is
 * there, at most max_samples; 0 if nothing yet */
static int receive_iq(float *buffer, int max_samples)
{
	int got;

	if (!calib_sdr)
		return -ENODEV;
	if (max_samples > calib_sdr_chunk)
		max_samples = calib_sdr_chunk;
	if (max_samples <= 0)
		return 0;
	got = sdr_read(calib_sdr, (sample_t **)(void *)buffer, max_samples, 0, NULL);
	return (got < 0) ? 0 : got;
}

/* Rate of a GSM channel taken out of the band: the SDR rate halved as long
 * as it stays at least the GSM rate (exact). The FCCH detector works at
 * that rate. */
static int gsm_channel_rate(int samplerate)
{
	int rate = samplerate, gsm = (int)ceil(GSM_RATE);

	while (!(rate % 2) && rate / 2 >= gsm)
		rate /= 2;
	return rate;
}

/* IQ samples per read in the measurement loop (50 ms) */
static int calib_block_samples(int samplerate)
{
	return (int)(samplerate * 0.05);
}

/* the usable part of the stream each side of the centre (Hz): the device's
 * flat, alias-free part (calibrate_config_t.usable_fraction, _max_hz;
 * docs/SDR-FRONTENDS.md) */
static double calib_usable_fraction = 0.35, calib_usable_max_hz = 0.0;

static double usable_half(int samplerate)
{
	double h = calib_usable_fraction * samplerate;

	if (calib_usable_max_hz > 0.0 && h > calib_usable_max_hz)
		h = calib_usable_max_hz;
	/* never beyond what the channel separation takes (0.4 each side) */
	if (h > 0.4 * samplerate)
		h = 0.4 * samplerate;
	return h;
}

/* the farthest a channel's centre may be from the SDR centre: its 200 kHz
 * within the usable part */
static double window_reach(int samplerate)
{
	return usable_half(samplerate) - CHANNEL_SPACING / 2.0;
}

static int window_capacity(int samplerate)
{
	/* the window centre on the half-channel grid: channels at +-100, +-300 kHz ... */
	double reach = window_reach(samplerate);

	if (reach < CHANNEL_SPACING / 2.0)
		return 0;
	return 2 * ((int)floor((reach - CHANNEL_SPACING / 2.0) / CHANNEL_SPACING) + 1);
}

/* Blocking receive: until the buffer is full, or 1 s without a sample (an
 * RTL2832 delivers in large chunks, the first ones late after a start or a
 * retune). Sleeps for the time the missing samples take (at most 20 ms):
 * few large reads, no polling of the ring. */
static int receive_iq_blocking(float *buffer, int max_samples)
{
	int total = 0;
	double idle = 0.0;

	while (total < max_samples && idle < 1.0) {
		int got = receive_iq(buffer + total * 2, max_samples - total);
		if (got > 0) {
			total += got;
			idle = 0.0;
			continue;
		}
		double wait = (calib_rate > 0) ? (double)(max_samples - total) / calib_rate : 0.001;
		if (wait > 0.02)
			wait = 0.02;
		if (wait < 0.001)
			wait = 0.001;
		usleep((useconds_t)(wait * 1e6));
		idle += wait;
	}
	return total;
}

/* Retune SDR RX to new center frequency */
static int retune_rx(double frequency)
{
	/* sdr_open() applies the upconverter, sdr_set_rx_frequency() does not
	 * (it only moves the device by the band's shift off the DC): add it */
	return sdr_set_rx_frequency(frequency + sdr_config->rx_upconverter);
}

/* After a retune: libsdr drops what came before it, until SCAN_SETTLE_MS
 * after the retune returned (the ring's backlog, the zeros while the
 * retune runs, the settling) */
static void settle_rx(void)
{
	if (calib_sdr)
		sdr_rx_settle(calib_sdr, SCAN_SETTLE_MS / 1000.0);
}

static double now_sec(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static double cpu_sec(clockid_t id)
{
	struct timespec ts;
	clock_gettime(id, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

void calibrate_config_default(calibrate_config_t *config)
{
	memset(config, 0, sizeof(*config));
	config->gsm_bands = GSM_BAND_900;
	config->timeout_sec = 30.0;
	config->verbosity = 1;
	config->num_channels = 0;
	config->cpu_percent = 50.0;
	config->towers = 3;
	config->full = 0;
	config->forced_arfcn = -1;
	config->confirm = 1;
	config->decode_timeout_sec = 5.0;
	config->decode_bcch = 1;
	config->usable_fraction = 0.35;	/* an unknown device */
	config->usable_max_hz = 0.0;
}

/* ================================================================
 * Scan: FFT power per ARFCN over the band
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

	double usable_bw = 2.0 * usable_half(samplerate);
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

		settle_rx();

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
				results[num_results].done = 0;
				num_results++;
			}
		}

		if (verbosity >= 2)
			LOGP(DSDR, LOGL_INFO, "  Step %d/%d: %.3f MHz, %d frames averaged\n",
			     s + 1, num_steps, center / 1e6, frames_done);
	}

	free(fft_re); free(fft_im); free(mag_accum); free(window); free(iq_buf);

	if (num_results > 0)
		qsort(results, num_results, sizeof(arfcn_power_t), compare_power_desc);

	return num_results;
}

/* The band's carriers to measure, strongest first: above the noise floor
 * (median + 6 dB), none next to a stronger one (its 200 kHz neighbours see
 * its spectrum). Returns their number, <0 on error. */
static int band_candidates(int samplerate, int gsm_band, calibrate_config_t *config,
			   arfcn_power_t **list_out)
{
	int arfcn_start, arfcn_end;
	double band_center, band_bw;

	*list_out = NULL;
	gsm_band_params(gsm_band, &band_center, &band_bw, &arfcn_start, &arfcn_end);

	/* one ARFCN asked for: no scan */
	if (config->forced_arfcn > 0) {
		arfcn_power_t *one = calloc(1, sizeof(*one));

		if (!one)
			return -ENOMEM;
		one->arfcn = config->forced_arfcn;
		one->freq = arfcn_to_freq(config->forced_arfcn, gsm_band);
		one->power_db = NAN;
		*list_out = one;
		LOGP(DSDR, LOGL_INFO, "%s: ARFCN %d (forced, no scan)\n", gsm_band_name(gsm_band), one->arfcn);
		return 1;
	}

	int max_arfcns = arfcn_end - arfcn_start + 1;
	arfcn_power_t *scan = calloc(max_arfcns, sizeof(*scan));
	arfcn_power_t *list = calloc(max_arfcns, sizeof(*list));
	if (!scan || !list) {
		free(scan);
		free(list);
		return -ENOMEM;
	}
	int num_scanned = scan_band(samplerate, gsm_band, arfcn_start, arfcn_end,
				    scan, max_arfcns, config->verbosity);
	if (num_scanned <= 0) {
		LOGP(DSDR, LOGL_ERROR, "%s: band scan failed or found no ARFCNs\n",
		     gsm_band_name(gsm_band));
		free(scan);
		free(list);
		return -ENODATA;
	}

	double *all_powers = calloc(num_scanned, sizeof(double));
	if (!all_powers) {
		free(scan);
		free(list);
		return -ENOMEM;
	}
	for (int i = 0; i < num_scanned; i++)
		all_powers[i] = scan[i].power_db;
	qsort(all_powers, num_scanned, sizeof(double), compare_double);
	double noise_floor = all_powers[num_scanned / 2];
	free(all_powers);
	double threshold = noise_floor + 6.0;

	int n = 0;
	for (int i = 0; i < num_scanned && scan[i].power_db >= threshold; i++) {
		int near = 0;
		for (int j = 0; j < n; j++)
			if (abs(scan[i].arfcn - list[j].arfcn) <= 1)
				near = 1;
		if (!near)
			list[n++] = scan[i];
	}
	LOGP(DSDR, LOGL_INFO, "%s: %d ARFCNs scanned, noise floor %.1f dBFS, %d carriers to measure\n",
	     gsm_band_name(gsm_band), num_scanned, noise_floor, n);
	for (int i = 0; i < n && i < 15; i++)
		LOGP(DSDR, LOGL_INFO, "  ARFCN %4d  %.3f MHz  %+.1f dBFS\n",
		     list[i].arfcn, list[i].freq / 1e6, list[i].power_db);
	free(scan);
	*list_out = list;
	return n;
}

/* ================================================================
 * A window: the carriers in the band at one tuning, measured together
 * ================================================================ */

/* one carrier in a window */
typedef struct {
	arfcn_power_t	*entry;
	double		offset;		/* from the SDR centre */
	fcch_detect_t	det;
	int		det_ok;
	gsm_sch_t	sch;
	int		sch_ok;
	float		offsets[AVG_COUNT], snrs[AVG_COUNT];
	int		nmeas;
	enum { DEC_WAIT, DEC_RUN, DEC_DONE } dec;
	double		dec_start;
	int		finished;
	double		level_sum;
	long		level_n;
	calibrate_cell_t *cell;
} wchan_t;

/* the CPU measurement of the first window */
typedef struct {
	int		done;
	double		per_channel;	/* share of a core per channel (main thread) */
	double		transport;	/* share of a core of the other threads */
	int		budget_channels;
} cpu_meter_t;

static void cell_result(wchan_t *w, int gsm_band)
{
	calibrate_cell_t *cell = w->cell;

	cell->ident = w->sch.ident;
	cell->confirmed = w->sch_ok ? gsm_sch_confirmed(&w->sch) : 0;
	cell->level_dbfs = (w->level_n > 0) ? 10.0 * log10(w->level_sum / w->level_n + 1e-30) : -200.0;
	cell->num_measurements = w->nmeas;
	if (w->nmeas < 3) {
		LOGP(DSDR, LOGL_INFO, "  %s ARFCN %d: %d FCCH, no result\n", gsm_band_name(gsm_band), cell->arfcn, w->nmeas);
		return;
	}

	int trim = (w->nmeas > 2 * AVG_THRESHOLD) ? AVG_THRESHOLD : 0;
	double stddev;
	float sorted[AVG_COUNT];
	memcpy(sorted, w->offsets, w->nmeas * sizeof(float));
	double avg = trimmed_mean(sorted, w->nmeas, trim, &stddev);
	double cluster_offset, cluster_stddev;
	if (estimate_high_snr_cluster(w->offsets, w->snrs, w->nmeas, &cluster_offset, &cluster_stddev)) {
		/* the dominant high-SNR cluster when it is clearly tighter */
		if (cluster_stddev > 0.0 && (stddev <= 0.0 || cluster_stddev < stddev * 0.8)) {
			avg = cluster_offset;
			stddev = cluster_stddev;
		}
	}
	double snr_sum = 0.0;
	for (int i = 0; i < w->nmeas; i++)
		snr_sum += w->snrs[i];
	cell->hz_offset = avg;
	cell->ppm_offset = avg / cell->freq_hz * 1e6;
	cell->stddev_hz = stddev;
	cell->snr_db = snr_sum / w->nmeas;
	LOGP(DSDR, LOGL_INFO, "  %s ARFCN %d: %d measurements, %+.2f Hz (%.3f ppm), stddev %.1f Hz, SNR %.1f dB, %s\n",
	     gsm_band_name(gsm_band), cell->arfcn, w->nmeas, avg, cell->ppm_offset, stddev,
	     cell->snr_db, cell->confirmed ? "confirmed" : (cell->decoded ? "NOT confirmed" : "identity not decoded"));
}

/* Measure the carriers ents[0..n-1] (all within the window of 'centre').
 * Fills their cells. Returns 0, <0 on error. */
static int measure_window(int samplerate, int gsm_band, calibrate_config_t *config,
			  double centre, arfcn_power_t **ents, int n,
			  calibrate_cell_t *cells, cpu_meter_t *meter, double t_run_start,
			  const char *start_info)
{
	int dsp = gsm_channel_rate(samplerate);
	int block = calib_block_samples(samplerate);
	/* the bank from 4 channels: measured at 8 MS/s, DSP 500 k (one
	 * recorded GSM-900 capture), a chain channel 7.3 % of a core, the bank
	 * 10.5 % + 4.5 % per channel; at 16 MS/s 13.4 % vs 15 % + 6.5 % */
	int use_bank = (n >= 4 && sdr_engine_bins(samplerate, dsp, n) > 0);
	sdr_engine_t eng;
	sdr_group_t *grp = NULL;
	double offs[SDR_ENGINE_MAX_CHAN];
	wchan_t *w = calloc(n, sizeof(*w));
	float *band = malloc((size_t)block * 2 * sizeof(float));
	int rows_max = (int)ceil((double)block * dsp / samplerate) + 64;
	sample_t **oi = calloc(n, sizeof(*oi)), **oq = calloc(n, sizeof(*oq));
	float *ciq = malloc((size_t)rows_max * 2 * sizeof(float));
	int rc = 0, c;

	if (!w || !band || !oi || !oq || !ciq) {
		rc = -ENOMEM;
		goto out_free;
	}
	for (c = 0; c < n; c++) {
		oi[c] = malloc(rows_max * sizeof(sample_t));
		oq[c] = malloc(rows_max * sizeof(sample_t));
		if (!oi[c] || !oq[c]) {
			rc = -ENOMEM;
			goto out_free;
		}
		offs[c] = ents[c]->freq - centre;
	}

	/* the separation first (filter design takes time), then the retune:
	 * the ring does not fill meanwhile */
	double t_design = now_sec();
	if (sdr_engine_init(&eng, samplerate, 0, use_bank ? dsp : 0, use_bank ? n : 0, block) < 0) {
		LOGP(DSDR, LOGL_ERROR, "Channel separation: %s\n", sdr_engine_error());
		rc = -EINVAL;
		goto out_free;
	}
	grp = sdr_engine_group_add(&eng, dsp, n, offs, 0, NULL, 1000, 0);
	if (!grp) {
		LOGP(DSDR, LOGL_ERROR, "Channel separation: %s\n", sdr_engine_error());
		sdr_engine_exit(&eng);
		rc = -EINVAL;
		goto out_free;
	}
	t_design = now_sec() - t_design;
	for (c = 0; c < n; c++) {
		w[c].entry = ents[c];
		w[c].offset = offs[c];
		w[c].cell = &cells[c];
		memset(&cells[c], 0, sizeof(cells[c]));
		cells[c].arfcn = ents[c]->arfcn;
		cells[c].gsm_band = gsm_band;
		cells[c].freq_hz = ents[c]->freq;
		cells[c].ident.bsic = -1;
		cells[c].level_dbfs = -200.0;
		if (fcch_detect_init(&w[c].det, dsp) < 0) {
			rc = -ENOMEM;
			goto out_chan;
		}
		w[c].det_ok = 1;
		w[c].sch_ok = (gsm_sch_init(&w[c].sch, dsp, config->decode_bcch) == 0);
		if (!w[c].sch_ok)
			LOGP(DSDR, LOGL_ERROR, "Cannot decode the cell identity at %d Hz\n", dsp);
	}

	LOGP(DSDR, LOGL_INFO, "Window at %.3f MHz: %d carriers (%s, %d Hz each, set up in %.0f ms)\n",
	     centre / 1e6, n, use_bank ? "filter bank" : "per-channel chain", dsp, t_design * 1e3);
	for (c = 0; c < n; c++)
		LOGP(DSDR, LOGL_INFO, "  ARFCN %d %.3f MHz %+.0f kHz, scan %.1f dBFS\n",
		     ents[c]->arfcn, ents[c]->freq / 1e6, offs[c] / 1e3, ents[c]->power_db);

	rc = retune_rx(centre);
	if (rc < 0) {
		LOGP(DSDR, LOGL_ERROR, "Failed to retune to %.3f MHz\n", centre / 1e6);
		goto out_chan;
	}
	rc = 0;
	settle_rx();

	double t0 = now_sec(), thr0 = cpu_sec(CLOCK_THREAD_CPUTIME_ID), proc0 = cpu_sec(CLOCK_PROCESS_CPUTIME_ID);
	int left = n;

	while (left > 0) {
		double t = now_sec() - t0;

		if (t > config->timeout_sec) {
			LOGP(DSDR, LOGL_NOTICE, "Window at %.3f MHz: timeout after %.0f s\n",
			     centre / 1e6, config->timeout_sec);
			break;
		}
		/* the budget is the whole program's (the reading too), with 15 %
		 * margin; the cost per channel measured here (on the chain it is
		 * not less than on the bank from 4 channels, by the bench) */
		if (meter && !meter->done && t >= CPU_MEASURE_SEC) {
			double thr = cpu_sec(CLOCK_THREAD_CPUTIME_ID) - thr0;
			double proc = cpu_sec(CLOCK_PROCESS_CPUTIME_ID) - proc0;

			meter->per_channel = thr / t / n;
			meter->transport = (proc - thr) / t;
			meter->budget_channels = (int)floor((0.85 * config->cpu_percent / 100.0 - meter->transport) / meter->per_channel);
			if (meter->budget_channels < 1)
				meter->budget_channels = 1;
			meter->done = 1;
			LOGP(DSDR, LOGL_NOTICE, "%s, %d channels per window (%s%.0f %% of a core with the reading, 15 %% kept free: %.1f %% per channel measured), reading %.1f %% of a core\n",
			     start_info ? start_info : "", config->num_channels ? config->num_channels : meter->budget_channels,
			     config->num_channels ? "forced; budget " : "budget ", config->cpu_percent,
			     100.0 * meter->per_channel, 100.0 * meter->transport);
			if (config->num_channels && config->num_channels * meter->per_channel + meter->transport > config->cpu_percent / 100.0)
				LOGP(DSDR, LOGL_NOTICE, "--channels %d needs about %.0f %% of a core with the reading, more than the budget of %.0f %%\n",
				     config->num_channels, 100.0 * (config->num_channels * meter->per_channel + meter->transport), config->cpu_percent);
		}

		int got = receive_iq_blocking(band, block);
		if (got <= 0) {
			LOGP(DSDR, LOGL_ERROR, "No samples from the SDR\n");
			rc = -EIO;
			break;
		}
		sdr_engine_rx_feed(&eng, band, got);
		int rows = sdr_engine_group_read(grp, oi, oq, rows_max);
		if (rows <= 0)
			continue;

		for (c = 0; c < n; c++) {
			wchan_t *ch = &w[c];
			double el;

			if (ch->finished)
				continue;
			for (int s = 0; s < rows; s++) {
				ciq[2 * s] = (float)oi[c][s];
				ciq[2 * s + 1] = (float)oq[c][s];
				ch->level_sum += oi[c][s] * oi[c][s] + oq[c][s] * oq[c][s];
			}
			ch->level_n += rows;

			/* identity decoding: starts when a few offsets are known,
			 * ends when confirmed (and SI3 seen if BCCH is decoded) or
			 * after the decode timeout */
			if (ch->dec == DEC_WAIT && ch->nmeas >= 3 && ch->sch_ok) {
				ch->dec = DEC_RUN;
				ch->dec_start = now_sec();
				ch->cell->decoded = 1;
			}
			if (ch->dec == DEC_RUN) {
				gsm_sch_process(&ch->sch, ciq, rows, running_median(ch->offsets, ch->nmeas));
				el = now_sec() - ch->dec_start;
				if (gsm_sch_confirmed(&ch->sch) && !ch->cell->confirmed) {
					ch->cell->confirmed = 1;
					ch->cell->confirm_sec = el;
				}
				if ((gsm_sch_confirmed(&ch->sch) && (!config->decode_bcch || ch->sch.ident.have_ci))
				 || el > config->decode_timeout_sec) {
					ch->dec = DEC_DONE;
					LOGP(DSDR, LOGL_INFO, "  ARFCN %d identity decoding ended after %.2f s: %s\n",
					     ch->entry->arfcn, el, gsm_sch_confirmed(&ch->sch) ? "confirmed" : "NOT confirmed");
					/* an unconfirmed carrier is not measured further */
					if (config->confirm && !gsm_sch_confirmed(&ch->sch))
						ch->finished = 1;
				}
			}
			if (!ch->finished && ch->nmeas < AVG_COUNT) {
				double offset, snr_db;

				if (fcch_detect_process(&ch->det, ciq, rows, &offset, &snr_db) > 0) {
					ch->offsets[ch->nmeas] = (float)offset;
					ch->snrs[ch->nmeas] = (float)snr_db;
					ch->nmeas++;
					if (config->verbosity >= 2)
						LOGP(DSDR, LOGL_INFO, "  #%d ARFCN %d: offset %+.1f Hz, SNR %.1f dB\n",
						     ch->nmeas, ch->entry->arfcn, offset, snr_db);
					fcch_detect_reset(&ch->det);
				}
			}
			if (!ch->finished && ch->nmeas >= AVG_COUNT && (ch->dec == DEC_DONE || !ch->sch_ok))
				ch->finished = 1;
			/* no BCCH carrier */
			if (!ch->finished && ((t > NO_FCCH_SEC && ch->nmeas == 0) || (t > FEW_FCCH_SEC && ch->nmeas < 3))) {
				LOGP(DSDR, LOGL_INFO, "  ARFCN %d: %d FCCH in %.1f s, no BCCH carrier\n",
				     ch->entry->arfcn, ch->nmeas, t);
				ch->finished = 1;
			}
			if (ch->finished) {
				cell_result(ch, gsm_band);
				left--;
			}
		}
	}
	/* what did not finish (timeout, error) */
	for (c = 0; c < n; c++)
		if (!w[c].finished)
			cell_result(&w[c], gsm_band);
	LOGP(DSDR, LOGL_INFO, "Window at %.3f MHz done after %.1f s (%.1f s into the run)\n",
	     centre / 1e6, now_sec() - t0, now_sec() - t_run_start);

out_chan:
	for (c = 0; c < n; c++) {
		if (w[c].det_ok)
			fcch_detect_exit(&w[c].det);
		if (w[c].sch_ok)
			gsm_sch_exit(&w[c].sch);
	}
	/* the feeding thread is this one: no device thread to wait for,
	 * the exit frees the group with the engine */
	sdr_engine_exit(&eng);
out_free:
	if (oi && oq)
		for (c = 0; c < n; c++) {
			free(oi[c]);
			free(oq[c]);
		}
	free(oi);
	free(oq);
	free(ciq);
	free(band);
	free(w);
	return rc;
}

/* ================================================================
 * The result: confirmed towers that agree
 * ================================================================ */

static int cell_usable(const calibrate_cell_t *cell, const calibrate_config_t *config)
{
	if (cell->num_measurements < 3)
		return 0;
	return config->confirm ? cell->confirmed : 1;
}

/* the result from the cells so far; returns the number of agreeing towers */
static int evaluate(calibrate_result_t *r, const calibrate_config_t *config)
{
	double ppm[CALIB_MAX_CELLS], med, sum = 0.0, lo = 1e9, hi = -1e9;
	int n = 0, agree = 0, i;

	for (i = 0; i < r->num_cells; i++) {
		r->cells[i].agrees = 0;
		if (cell_usable(&r->cells[i], config)) {
			ppm[n++] = r->cells[i].ppm_offset;
			if (r->cells[i].ppm_offset < lo)
				lo = r->cells[i].ppm_offset;
			if (r->cells[i].ppm_offset > hi)
				hi = r->cells[i].ppm_offset;
		}
	}
	r->num_towers = n;
	r->valid = 0;
	r->num_agree = 0;
	r->disagree = 0;
	r->spread_ppm = (n > 1) ? hi - lo : 0.0;
	if (!n)
		return 0;
	qsort(ppm, n, sizeof(double), compare_double);
	med = (n % 2) ? ppm[n / 2] : 0.5 * (ppm[n / 2 - 1] + ppm[n / 2]);
	for (i = 0; i < r->num_cells; i++) {
		if (!cell_usable(&r->cells[i], config))
			continue;
		if (fabs(r->cells[i].ppm_offset - med) <= CALIB_AGREE_PPM) {
			r->cells[i].agrees = 1;
			sum += r->cells[i].ppm_offset;
			agree++;
		} else
			r->disagree++;
	}
	r->num_agree = agree;
	/* none near the median (two towers apart): no result, both listed */
	r->ppm_offset = agree ? sum / agree : med;
	r->valid = (agree > 0);
	return agree;
}

/* the window centre for the strongest carrier not measured yet (index
 * first): on the half-channel grid (no carrier on the DC), with the
 * strongest in reach, covering as many other carriers not measured yet as
 * possible (ties: the strongest's side) */
static double window_centre(const arfcn_power_t *list, int n, int first, double reach, int per_window)
{
	double best_centre = list[first].freq + CHANNEL_SPACING / 2.0;
	int best = -1;

	if (per_window <= 1)
		return best_centre;
	for (int j = -(int)(reach / CHANNEL_SPACING) - 1; j <= (int)(reach / CHANNEL_SPACING); j++) {
		double c = list[first].freq + CHANNEL_SPACING / 2.0 + j * CHANNEL_SPACING;
		int count = 0;

		if (fabs(list[first].freq - c) > reach)
			continue;
		for (int i = 0; i < n; i++)
			if (!list[i].done && fabs(list[i].freq - c) <= reach)
				count++;
		if (count > best || (count == best && fabs(c - list[first].freq) < fabs(best_centre - list[first].freq))) {
			best = count;
			best_centre = c;
		}
	}
	return best_centre;
}

/* ================================================================
 * Main entry point
 * ================================================================ */

int calibrate_auto(int samplerate, calibrate_config_t *config, calibrate_result_t *result,
		   const char *start_info)
{
	int bands_to_scan[CALIB_MAX_BANDS];
	int num_bands = 0, stop = 0;
	int capacity, per_window;
	double reach;

	if (config->usable_fraction > 0.0)
		calib_usable_fraction = config->usable_fraction;
	calib_usable_max_hz = config->usable_max_hz;
	capacity = window_capacity(samplerate);
	reach = window_reach(samplerate);
	double t_start = now_sec();
	cpu_meter_t meter = { 0 };

	memset(result, 0, sizeof(*result));
	result->samplerate = samplerate;
	if (capacity > SDR_ENGINE_MAX_CHAN)
		capacity = SDR_ENGINE_MAX_CHAN;
	/* without a bank (rate too low for one) the chain carries at most 4 */
	if (sdr_engine_bins(samplerate, gsm_channel_rate(samplerate), 8) == 0 && capacity > SDR_ENGINE_MAX_CHAIN)
		capacity = SDR_ENGINE_MAX_CHAIN;
	if (capacity < 1) {
		LOGP(DSDR, LOGL_ERROR, "%d S/s is too narrow for a GSM channel\n", samplerate);
		return -EINVAL;
	}
	if (config->num_channels > capacity) {
		LOGP(DSDR, LOGL_ERROR, "--channels %d does not fit: a window at %d S/s holds %d channels (+-%.0f kHz around the centre%s)\n",
		     config->num_channels, samplerate, capacity, reach / 1e3,
		     (capacity == SDR_ENGINE_MAX_CHAIN) ? "; no filter bank fits this rate: at most 4 on the per-channel chain" : "");
		return -EINVAL;
	}
	per_window = config->num_channels ? config->num_channels : FIRST_WINDOW_CHANNELS;
	if (per_window > capacity)
		per_window = capacity;

	if (config->gsm_bands == GSM_BAND_ALL) {
		for (int i = 0; i < NUM_ALL_BANDS && i < CALIB_MAX_BANDS; i++)
			bands_to_scan[num_bands++] = all_bands[i];
	} else {
		bands_to_scan[num_bands++] = config->gsm_bands;
	}

	for (int b = 0; b < num_bands && !stop; b++) {
		int band = bands_to_scan[b];
		arfcn_power_t *list;
		int n = band_candidates(samplerate, band, config, &list);

		if (n <= 0) {
			free(list);
			continue;
		}
		while (!stop) {
			arfcn_power_t *ents[SDR_ENGINE_MAX_CHAN];
			int k = 0, i, first = -1;

			for (i = 0; i < n; i++)
				if (!list[i].done) {
					first = i;
					break;
				}
			if (first < 0 || result->num_cells >= CALIB_MAX_CELLS)
				break;
			double centre = window_centre(list, n, first, reach, per_window);			/* the strongest first, then the next strongest in reach */
			for (i = first; i < n && k < per_window && result->num_cells + k < CALIB_MAX_CELLS; i++) {
				if (list[i].done || fabs(list[i].freq - centre) > reach
				 || fabs(list[i].freq - centre) < DC_GUARD_HZ)
					continue;
				ents[k++] = &list[i];
				list[i].done = 1;
			}
			int rc = measure_window(samplerate, band, config, centre, ents, k,
						&result->cells[result->num_cells],
						meter.done ? NULL : &meter, t_start, start_info);
			if (rc < 0 && rc != -EIO)
				break;
			result->num_cells += k;
			result->windows++;
			if (rc == -EIO) {
				stop = 1;
				break;
			}
			/* the channels per window from the measured cost */
			if (meter.done && !config->num_channels) {
				per_window = meter.budget_channels;
				if (per_window > capacity)
					per_window = capacity;
			}
			int agree = evaluate(result, config);
			if (!config->full && agree >= ((config->forced_arfcn > 0) ? 1 : config->towers)) {
				result->time_to_result = now_sec() - t_start;
				stop = 1;
			}
		}
		free(list);
	}

	result->channels_per_window = config->num_channels ? config->num_channels : per_window;
	result->cpu_per_channel = meter.per_channel;
	result->cpu_transport = meter.transport;
	evaluate(result, config);
	if (!result->time_to_result)
		result->time_to_result = now_sec() - t_start;
	return result->valid ? 0 : -ENODATA;
}
