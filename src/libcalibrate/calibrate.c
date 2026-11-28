/* SDR Clock Calibration - scan, then windows of channels
 *
 *   Scan: step across the band, the averaged spectrum in 25 kHz cells (kept
 *     for the run: bands that overlap, GSM and LTE, are scanned once). GSM:
 *     power per ARFCN, the carriers above the noise floor, strongest first.
 *     LTE: power of the centre 6 RB (PSS, SSS, PBCH) per EARFCN, its local
 *     peaks, strongest first, each with its two raster neighbours.
 *   Windows: tune so that the strongest carrier not measured yet and as many
 *     others as fit (and as the CPU budget allows) are in the band; separate
 *     them with libsdr's engine (the filter bank, or the per-channel chain
 *     for one or two), measure the FCCH offset of each, decode the SCH (and
 *     the BCCH SI) as the proof of a real cell. LTE: the EARFCNs of a peak
 *     at 1.92 MS/s each into lte_cell (PSS/SSS search, MIB as the proof,
 *     the carrier offset from the CRS). Next window, until enough confirmed
 *     towers (GSM and LTE together) agree.
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
#include "lte_bands.h"
#include "lte_cell.h"
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
/* nor one whose first FCCH are this weak (mean SNR); the run stops after
 * this many carriers in a row (strongest first) without a confirmed one */
#define WEAK_FCCH_COUNT 5
#define WEAK_FCCH_SNR_DB 10.0
#define WEAK_IN_A_ROW   8
/* a carrier with this many offsets after this time is done (trimmed mean
 * from 17 on) */
#define SLOW_SEC        6.0
#define SLOW_MIN        20

/* GSM channel spacing */
#define CHANNEL_SPACING 200000  /* 200 kHz */
/* no carrier centre closer to the DC (the LO sits in the middle of the
 * stream: its DC strip, tens of kHz, stays outside the carrier's +-90 kHz):
 * the window centre on the half-channel grid puts the nearest at 100 kHz */
#define DC_GUARD_HZ     (CHANNEL_SPACING / 2.0 - 1.0)

/* Scan parameters */
#define SCAN_FFT_SIZE   4096    /* FFT size for power scan */
#define SCAN_FFT_LOG2   12
#define SCAN_AVG_FRAMES 32      /* FFT frames to average during scan, at least */
#define SCAN_AVG_SEC    0.02    /* and at least this long (LTE: PSS/SSS/PBCH every 5 ms) */
#define SCAN_SETTLE_MS  50      /* ms after a retune returned to discard (B210: 5.5 ms measured; other devices buffer more) */
#define SCAN_RES_HZ     25e3    /* cells of the spectrum kept */
#define SCAN_DC_HZ      10e3    /* the receiver's DC strip, left out */
#define SCAN_MARGIN_HZ  0.7e6   /* scanned beyond the band's outer carriers */
/* the scan centres 50 kHz off the 100 kHz raster (GSM, LTE): no carrier
 * centre on the DC strip */
#define SCAN_GRID_HZ    100e3
#define SCAN_GRID_OFF   50e3

/* LTE */
#define LTE_DSP         1920000 /* lte_cell's input rate */
#define LTE_6RB_HZ      540e3   /* half of the centre 6 RB */
#define LTE_PEAK_REACH  5       /* a peak: the most within +- this many EARFCNs */
#define LTE_PEAK_MIN_DB 2.0     /* above the floor (10th percentile of 100 kHz cells) */
#define LTE_FLOOR_PCT   10
#define LTE_WEAK_IN_A_ROW 6     /* peaks in a row without a confirmed cell: the band ends */
#define LTE_STAGGER_SEC 0.03    /* channel c starts c x this late: their searches do not coincide */
#define LTE_NO_MIB_SEC  2.0     /* a cell without a good MIB this long: no cell */
#define LTE_MIN_POINTS  50      /* frame-to-frame offset points (0.5 s) */
#define LTE_MAX_SEC     4.0     /* most time per carrier */
#define LTE_TIMING_STD  0.02    /* timing ppm wanted (lte_timing) */
#define LTE_TIMING_MAX_SEC 12.0
#define LTE_ALIAS_MIN   3.0     /* cfo_alias_margin below: weight LTE_LOW_WEIGHT */
#define LTE_LOW_WEIGHT  0.25
#define LTE_LO_OFF_HZ   750e3   /* the LO this far from the carrier (outside its 6 RB) where the rate allows */
#define LTE_RATE_MAX    8200000 /* LTE windows at most this SDR rate: the chain's cost grows with it */
#define LTE_CFO_MAX_HZ  40000.0 /* without a prior (lte_cell: below 50 kHz) */
#define LTE_CFO_PPM     20.0    /* without a prior, but at most this */
#define LTE_PRIOR_PPM   1.5     /* with a prior: +- this plus LTE_PRIOR_HZ */
#define LTE_PRIOR_HZ    2000.0

/* channels measured in the first window, before the cost per channel is known */
#define FIRST_WINDOW_CHANNELS 3
/* the CPU measurement: after this long in the first window */
#define CPU_MEASURE_SEC 1.0

/* ARFCN scan result */
typedef struct {
	int    arfcn;       /* LTE: EARFCN */
	double freq;        /* Absolute frequency Hz */
	double power_db;    /* Measured power dBFS */
	int    done;        /* measured (or skipped) */
	int    band;        /* the band that names it */
	int    peak;        /* LTE: the peak it belongs to (0: the strongest) */
} arfcn_power_t;

typedef struct {
	float offset;
	float snr;
} offset_snr_t;

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
	config->bands[0].system = CALIB_GSM;
	config->bands[0].id = GSM_BAND_900;
	gsm_band_dl_range(GSM_BAND_900, &config->bands[0].f_lo, &config->bands[0].f_hi);
	config->bands[0].scan = 1;
	config->num_bands = 1;
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

/* the spectrum of the run: 25 kHz cells, power density (per Hz, full scale
 * 1.0) from the scan step whose centre is nearest */
static struct {
	double	f0;
	int	n;
	float	*pow;		/* <0: not measured */
	float	*dist;		/* the cell from its step's centre (Hz) */
} spec;

static void spec_exit(void)
{
	free(spec.pow);
	free(spec.dist);
	memset(&spec, 0, sizeof(spec));
}

static int spec_init(const calibrate_config_t *config)
{
	double lo = 0.0, hi = 0.0;

	spec_exit();
	for (int b = 0; b < config->num_bands; b++) {
		if (lo == 0.0 || config->bands[b].f_lo < lo)
			lo = config->bands[b].f_lo;
		if (config->bands[b].f_hi > hi)
			hi = config->bands[b].f_hi;
	}
	if (hi <= 0.0)
		return -EINVAL;
	spec.f0 = floor((lo - 2.0 * SCAN_MARGIN_HZ) / SCAN_RES_HZ) * SCAN_RES_HZ;
	spec.n = (int)ceil((hi + 2.0 * SCAN_MARGIN_HZ - spec.f0) / SCAN_RES_HZ) + 1;
	spec.pow = malloc(spec.n * sizeof(float));
	spec.dist = malloc(spec.n * sizeof(float));
	if (!spec.pow || !spec.dist)
		return -ENOMEM;
	for (int i = 0; i < spec.n; i++) {
		spec.pow[i] = -1.0f;
		spec.dist[i] = 1e30f;
	}
	return 0;
}

static int spec_cell(double f)
{
	int i = (int)floor((f - spec.f0) / SCAN_RES_HZ);

	return (i < 0 || i >= spec.n) ? -1 : i;
}

/* the measured share of the cells in [a, b] */
static double spec_covered(double a, double b)
{
	int i0 = spec_cell(a), i1 = spec_cell(b), n = 0, m = 0;

	for (int i = i0; i >= 0 && i <= i1; i++, n++)
		if (spec.pow[i] >= 0.0f)
			m++;
	return n ? (double)m / n : 0.0;
}

/* power in [a, b] (dBFS: the density times the width), NAN if not measured */
static double spec_db(double a, double b)
{
	int i0 = spec_cell(a), i1 = spec_cell(b), n = 0;
	double s = 0.0;

	for (int i = i0; i >= 0 && i <= i1; i++)
		if (spec.pow[i] >= 0.0f) {
			s += spec.pow[i];
			n++;
		}
	if (!n)
		return NAN;
	return 10.0 * log10(s / n * (b - a) + 1e-30);
}

/* the median cell in [a, b] as the power of the whole width (dBFS), NAN if
 * not measured: an LTE carrier's centre 6 RB fill it, a GSM carrier (200
 * kHz of 1080) does not move it */
static double spec_median_db(double a, double b)
{
	int i0 = spec_cell(a), i1 = spec_cell(b), n = 0;
	float v[128];

	for (int i = i0; i >= 0 && i <= i1 && n < 128; i++)
		if (spec.pow[i] >= 0.0f)
			v[n++] = spec.pow[i];
	if (!n)
		return NAN;
	qsort(v, n, sizeof(float), compare_float);
	return 10.0 * log10(v[n / 2] * (b - a) + 1e-30);
}

/* scan [band_lo, band_hi] (and the margin) where the spectrum is not
 * measured yet */
static int scan_range(int samplerate, const char *name, double band_lo, double band_hi, int verbosity)
{
	double usable_bw = 2.0 * usable_half(samplerate);
	double step_hz = usable_bw;

	double scan_start = band_lo - SCAN_MARGIN_HZ;
	double scan_end = band_hi + SCAN_MARGIN_HZ;
	int num_steps = (int)ceil((scan_end - scan_start) / step_hz);
	if (num_steps < 1) num_steps = 1;

	int fft_size = SCAN_FFT_SIZE;
	int frames_want = (int)ceil(SCAN_AVG_SEC * samplerate / fft_size);
	if (frames_want < SCAN_AVG_FRAMES)
		frames_want = SCAN_AVG_FRAMES;
	double *fft_re = calloc(fft_size, sizeof(double));
	double *fft_im = calloc(fft_size, sizeof(double));
	double *mag_accum = calloc(fft_size, sizeof(double));
	double *window = calloc(fft_size, sizeof(double));
	int capture_samples = fft_size * (SCAN_AVG_FRAMES + 2);
	float *iq_buf = calloc(capture_samples * 2, sizeof(float));
	int cells_max = (int)ceil(usable_bw / SCAN_RES_HZ) + 4;
	double *csum = calloc(cells_max, sizeof(double));
	int *ccnt = calloc(cells_max, sizeof(int));
	int done_steps = 0;

	if (!fft_re || !fft_im || !mag_accum || !window || !iq_buf || !csum || !ccnt) {
		free(fft_re); free(fft_im); free(mag_accum); free(window); free(iq_buf); free(csum); free(ccnt);
		return -ENOMEM;
	}

	double wsum = 0.0;
	for (int i = 0; i < fft_size; i++) {
		window[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (fft_size - 1)));
		wsum += window[i] * window[i];
	}

	double bin_hz = (double)samplerate / fft_size;

	for (int s = 0; s < num_steps; s++) {
		double center = scan_start + step_hz / 2.0 + s * step_hz;

		center = floor(center / SCAN_GRID_HZ) * SCAN_GRID_HZ + SCAN_GRID_OFF;
		if (spec_covered(center - usable_bw / 2.0, center + usable_bw / 2.0) > 0.9)
			continue;
		done_steps++;

		int rc = retune_rx(center);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to retune to %.3f MHz\n",
			     center / 1e6);
			continue;
		}

		settle_rx();

		memset(mag_accum, 0, fft_size * sizeof(double));
		int frames_done = 0;

		while (frames_done < frames_want) {
			int want = fft_size * (frames_want - frames_done + 1);
			if (want > capture_samples) want = capture_samples;
			int got = receive_iq_blocking(iq_buf, want);
			if (got < fft_size) break;

			int offset = 0;
			while (offset + fft_size <= got && frames_done < frames_want) {
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

		/* the step's bins into the cells: density per Hz (a full scale
		 * tone in one bin: 1 / bin_hz) */
		double half_bw = usable_bw / 2.0;
		int c0 = spec_cell(center - half_bw);
		/* fft_process() scales the forward transform by 1 / N */
		double scale = (double)fft_size * fft_size / (frames_done * wsum * fft_size * bin_hz);

		memset(csum, 0, cells_max * sizeof(double));
		memset(ccnt, 0, cells_max * sizeof(int));
		for (int k = 0; k < fft_size; k++) {
			double off = ((k < fft_size / 2) ? k : k - fft_size) * bin_hz;
			int ci;

			if (fabs(off) > half_bw || fabs(off) < SCAN_DC_HZ)
				continue;
			ci = spec_cell(center + off) - c0;
			if (c0 < 0 || ci < 0 || ci >= cells_max)
				continue;
			csum[ci] += mag_accum[k] * scale;
			ccnt[ci]++;
		}
		for (int ci = 0; ci < cells_max; ci++) {
			int i = c0 + ci;
			double d;

			if (!ccnt[ci] || i < 0 || i >= spec.n)
				continue;
			d = fabs(spec.f0 + (i + 0.5) * SCAN_RES_HZ - center);
			if (d < spec.dist[i]) {
				spec.pow[i] = (float)(csum[ci] / ccnt[ci]);
				spec.dist[i] = (float)d;
			}
		}

		if (verbosity >= 2)
			LOGP(DSDR, LOGL_INFO, "  Step %d/%d: %.3f MHz, %d frames averaged\n",
			     s + 1, num_steps, center / 1e6, frames_done);
	}

	LOGP(DSDR, LOGL_INFO, "Scanned %s: %.1f-%.1f MHz, %d of %d steps of %.1f MHz (the rest scanned before)\n",
	     name, band_lo / 1e6, band_hi / 1e6, done_steps, num_steps, step_hz / 1e6);
	free(fft_re); free(fft_im); free(mag_accum); free(window); free(iq_buf); free(csum); free(ccnt);
	return 0;
}

/* a carrier of this system at this frequency measured already in the run */
static int measured_before(const calibrate_result_t *r, int system, double freq)
{
	for (int i = 0; i < r->num_cells; i++)
		if (r->cells[i].system == system && fabs(r->cells[i].freq_hz - freq) < 1.0)
			return 1;
	return 0;
}

/* the band of the run that names a GSM carrier: the first (P-, E-, R-GSM
 * in that order) with this frequency; its ARFCN in *arfcn */
static int gsm_name(const calibrate_config_t *config, double freq, int *arfcn)
{
	for (int b = 0; b < config->num_bands; b++) {
		const calib_band_t *cb = &config->bands[b];
		int n;

		if (cb->system != CALIB_GSM || freq < cb->f_lo - 1.0 || freq > cb->f_hi + 1.0)
			continue;
		n = gsm_freq_to_arfcn(cb->id, freq);
		if (n >= 0) {
			*arfcn = n;
			return cb->id;
		}
	}
	return -1;
}

/* the same for LTE: the lowest band number of the run with this EARFCN
 * frequency */
static int lte_name(const calibrate_config_t *config, double freq, int *earfcn)
{
	int best = -1;

	for (int b = 0; b < config->num_bands; b++) {
		const calib_band_t *cb = &config->bands[b];
		int n;

		if (cb->system != CALIB_LTE || freq < cb->f_lo - 1.0 || freq > cb->f_hi + 1.0)
			continue;
		if (best >= 0 && cb->id > best)
			continue;
		n = lte_freq_to_earfcn(cb->id, freq);
		if (n >= 0) {
			*earfcn = n;
			best = cb->id;
		}
	}
	return best;
}

/* The band's carriers to measure, strongest first: above the noise floor
 * (median + 6 dB), none next to a stronger one (its 200 kHz neighbours see
 * its spectrum). Returns their number, <0 on error. */
static int band_candidates(int samplerate, const calib_band_t *band, calibrate_config_t *config,
			   const calibrate_result_t *result, arfcn_power_t **list_out)
{
	int gsm_band = band->id;
	const gsm_band_def_t *def = gsm_band_def(gsm_band);

	*list_out = NULL;
	if (!def)
		return -EINVAL;

	/* one ARFCN asked for: no scan */
	if (config->forced_arfcn > 0) {
		arfcn_power_t *one = calloc(1, sizeof(*one));

		if (!one)
			return -ENOMEM;
		one->arfcn = config->forced_arfcn;
		one->freq = arfcn_to_freq(config->forced_arfcn, gsm_band);
		one->power_db = NAN;
		one->band = gsm_band;
		*list_out = one;
		LOGP(DSDR, LOGL_INFO, "%s: ARFCN %d (forced, no scan)\n", gsm_band_name(gsm_band), one->arfcn);
		return 1;
	}

	int max_arfcns = 0;
	for (int r = 0; r < 2; r++)
		if (def->run[r].last >= def->run[r].first)
			max_arfcns += def->run[r].last - def->run[r].first + 1;
	arfcn_power_t *scan = calloc(max_arfcns, sizeof(*scan));
	arfcn_power_t *list = calloc(max_arfcns, sizeof(*list));
	if (!scan || !list) {
		free(scan);
		free(list);
		return -ENOMEM;
	}
	int rc = scan_range(samplerate, gsm_band_name(gsm_band), band->f_lo - CHANNEL_SPACING, band->f_hi + CHANNEL_SPACING,
			    config->verbosity);
	int num_scanned = 0, before = 0;
	for (int r = 0; rc == 0 && r < 2; r++) {
		for (int a = def->run[r].first; a <= def->run[r].last; a++) {
			double freq = arfcn_to_freq(a, gsm_band), p;
			int na;

			if (freq < band->f_lo - 1.0 || freq > band->f_hi + 1.0)
				continue;
			p = spec_db(freq - CHANNEL_SPACING / 2.0, freq + CHANNEL_SPACING / 2.0);
			if (isnan(p))
				continue;
			if (measured_before(result, CALIB_GSM, freq)) {
				before++;
				continue;
			}
			scan[num_scanned].band = gsm_name(config, freq, &na);
			if (scan[num_scanned].band < 0) {
				scan[num_scanned].band = gsm_band;
				na = a;
			}
			scan[num_scanned].arfcn = na;
			scan[num_scanned].freq = freq;
			scan[num_scanned].power_db = p;
			num_scanned++;
		}
	}
	if (num_scanned > 0)
		qsort(scan, num_scanned, sizeof(arfcn_power_t), compare_power_desc);
	if (num_scanned <= 0) {
		if (before)
			LOGP(DSDR, LOGL_INFO, "%s: its %d carriers measured before\n", gsm_band_name(gsm_band), before);
		else
			LOGP(DSDR, LOGL_ERROR, "%s: band scan failed or found no ARFCNs\n",
			     gsm_band_name(gsm_band));
		free(scan);
		free(list);
		return before ? 0 : -ENODATA;
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
			if (fabs(scan[i].freq - list[j].freq) <= CHANNEL_SPACING + 1.0)
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

/* EARFCN frequencies searched in the run (bands overlap: band 1 in 65,
 * 7 in 41, ...): each once */
#define LTE_TESTED_MAX 8192
static double lte_tested[LTE_TESTED_MAX];
static int lte_tested_n;

static int lte_tested_before(double f)
{
	for (int i = 0; i < lte_tested_n; i++)
		if (fabs(lte_tested[i] - f) < 1.0)
			return 1;
	return 0;
}

static void lte_tested_add(double f)
{
	if (lte_tested_n < LTE_TESTED_MAX && !lte_tested_before(f))
		lte_tested[lte_tested_n++] = f;
}

static int compare_peak(const void *a, const void *b)
{
	const arfcn_power_t *pa = a, *pb = b;

	if (pa->peak != pb->peak)
		return (pa->peak > pb->peak) - (pa->peak < pb->peak);
	return (pa->freq > pb->freq) - (pa->freq < pb->freq);
}

/* LTE: the EARFCNs whose centre 6 RB power is the most within +-500 kHz and
 * above the floor, strongest first, each with its raster neighbours (the
 * peak of an evenly loaded carrier is not always on its centre: measured on
 * 9 cells, 6 on the peak, 3 one EARFCN off). list[].peak: the rank. */
static int lte_candidates(int samplerate, const calib_band_t *band, calibrate_config_t *config,
			  const calibrate_result_t *result, arfcn_power_t **list_out)
{
	int first, last, n = 0, np = 0, nr, i, j, rc;
	double ff, fl, *p, *cellp, floor_db;
	arfcn_power_t *list;
	char name[16];

	*list_out = NULL;
	snprintf(name, sizeof(name), "LTE band %d", band->id);
	if (lte_band_dl_range(band->id, &first, &last, &ff, &fl) < 0)
		return -EINVAL;
	nr = last - first + 1;
	rc = scan_range(samplerate, name, band->f_lo - LTE_6RB_HZ, band->f_hi + LTE_6RB_HZ, config->verbosity);
	if (rc < 0)
		return rc;
	p = calloc(nr, sizeof(double));
	cellp = calloc(nr, sizeof(double));
	list = calloc(4 * nr, sizeof(*list));
	if (!p || !cellp || !list) {
		free(p);
		free(cellp);
		free(list);
		return -ENOMEM;
	}
	int m = 0;
	for (i = 0; i < nr; i++) {
		double f = ff + 100e3 * i;

		p[i] = NAN;
		if (f < band->f_lo - 1.0 || f > band->f_hi + 1.0)
			continue;
		p[i] = spec_median_db(f - LTE_6RB_HZ, f + LTE_6RB_HZ);
		/* the floor in 100 kHz, as a density per 1.08 MHz */
		double c = spec_db(f - 50e3, f + 50e3);
		if (!isnan(c))
			cellp[m++] = c + 10.0 * log10(2.0 * LTE_6RB_HZ / 100e3);
	}
	if (!m) {
		free(p);
		free(cellp);
		free(list);
		return -ENODATA;
	}
	qsort(cellp, m, sizeof(double), compare_double);
	floor_db = cellp[m * LTE_FLOOR_PCT / 100];
	for (i = 0; i < nr; i++) {
		int top = 1;

		if (isnan(p[i]) || p[i] - floor_db < LTE_PEAK_MIN_DB)
			continue;
		for (j = i - LTE_PEAK_REACH; j <= i + LTE_PEAK_REACH && top; j++)
			if (j >= 0 && j < nr && j != i && !isnan(p[j]) && p[j] > p[i])
				top = 0;
		if (!top)
			continue;
		/* a flat top (the median of the same cells): its middle */
		for (j = i; j + 1 < nr && p[j + 1] == p[i]; j++)
			;
		if (j > i) {
			int mid = (i + j) / 2;

			p[mid] = p[i];
			i = j;	/* the run is one peak */
			list[np].arfcn = first + mid;
			list[np].freq = ff + 100e3 * mid;
			list[np].power_db = p[mid];
			np++;
			continue;
		}
		list[np].arfcn = first + i;	/* the peaks first, ranked below */
		list[np].freq = ff + 100e3 * i;
		list[np].power_db = p[i];
		np++;
	}
	qsort(list, np, sizeof(*list), compare_power_desc);
	/* a loaded carrier is flat over its width, its centre 6 RB no higher:
	 * a peak on a wide flat top (within 3 dB over more than 2 MHz) gets
	 * the top's middle right after it */
	for (i = 0; i < np && np < nr; i++) {
		int k = list[i].arfcn - first, lo = k, hi = k, mid, dup = 0;

		if (list[i].peak < 0)
			continue;	/* a middle added */
		while (lo > 0 && !isnan(p[lo - 1]) && p[lo - 1] >= list[i].power_db - 3.0)
			lo--;
		while (hi + 1 < nr && !isnan(p[hi + 1]) && p[hi + 1] >= list[i].power_db - 3.0)
			hi++;
		if (hi - lo < 20)
			continue;
		mid = (lo + hi) / 2;
		for (j = 0; j < np; j++)
			if (abs(list[j].arfcn - (first + mid)) <= 1)
				dup = 1;
		if (dup)
			continue;
		memmove(list + i + 2, list + i + 1, (np - i - 1) * sizeof(*list));
		list[i + 1].arfcn = first + mid;
		list[i + 1].freq = ff + 100e3 * mid;
		list[i + 1].power_db = list[i].power_db;
		list[i + 1].peak = -1;
		np++;
	}
	LOGP(DSDR, LOGL_INFO, "%s: %d EARFCNs, floor %.1f dBFS per 1.08 MHz, %d peaks%s\n", name, m, floor_db, np,
	     np ? " (strongest first):" : "");
	for (i = 0; i < np && i < 15; i++)
		LOGP(DSDR, LOGL_INFO, "  EARFCN %5d  %.1f MHz  %+.1f dB\n", list[i].arfcn, list[i].freq / 1e6, list[i].power_db - floor_db);
	/* each peak with its neighbours (a stronger peak takes a shared one) */
	for (i = 0; i < np; i++) {
		arfcn_power_t pk = list[i];

		for (j = -1; j <= 1; j++) {
			int k = pk.arfcn - first + j, e;
			double f = ff + 100e3 * k;
			int dup = 0;

			if (k < 0 || k >= nr || f < band->f_lo - 1.0 || f > band->f_hi + 1.0)
				continue;
			if (measured_before(result, CALIB_LTE, f) || lte_tested_before(f))
				continue;
			for (int q = np; q < np + n; q++)
				if (fabs(list[q].freq - f) < 1.0)
					dup = 1;
			if (dup)
				continue;
			list[np + n].band = lte_name(config, f, &e);
			if (list[np + n].band < 0) {
				list[np + n].band = band->id;
				e = first + k;
			}
			list[np + n].arfcn = e;
			list[np + n].freq = f;
			list[np + n].power_db = pk.power_db;
			list[np + n].peak = i;
			n++;
		}
	}
	memmove(list, list + np, n * sizeof(*list));
	qsort(list, n, sizeof(*list), compare_peak);
	free(p);
	free(cellp);
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

/* the CPU measurement of the first window, shares of one core, split into
 * what grows with the SDR rate and what does not */
typedef struct {
	int		done;
	int		rate;		/* SDR rate measured at */
	double		per_channel;	/* main thread per channel: separation + detection */
	double		transport;	/* the reading: other threads + the main thread's reads */
	double		sep;		/* per channel: separation (grows with the rate) */
	double		det;		/* per channel: FCCH, SCH, BCCH (does not) */
	int		budget_channels;
	int		applied;	/* the rate chosen from it */
} cpu_meter_t;

/* channels that fit the budget at another rate, from the measurement:
 * the reading and the separation scale with the rate */
static int budget_at(const cpu_meter_t *m, double cpu_percent, int rate)
{
	double x = (double)rate / m->rate;
	double c = m->sep * x + m->det;
	double free = 0.85 * cpu_percent / 100.0 - m->transport * x;

	if (c <= 0.0 || free <= 0.0)
		return 0;
	return (int)floor(free / c);
}

static void cell_result(wchan_t *w)
{
	calibrate_cell_t *cell = w->cell;
	int gsm_band = cell->gsm_band;

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
	cell->carrier_ppm = cell->ppm_offset;
	cell->weight = 1.0;
	cell->stddev_hz = stddev;
	cell->snr_db = snr_sum / w->nmeas;
	LOGP(DSDR, LOGL_INFO, "  %s ARFCN %d: %d measurements, %+.2f Hz (%.3f ppm), stddev %.1f Hz, SNR %.1f dB, %s\n",
	     gsm_band_name(gsm_band), cell->arfcn, w->nmeas, avg, cell->ppm_offset, stddev,
	     cell->snr_db, cell->confirmed ? "confirmed" : (cell->decoded ? "NOT confirmed" : "identity not decoded"));
}

/* Measure the carriers ents[0..n-1] (all within the window of 'centre').
 * Fills their cells. Returns 0, <0 on error. */
static int measure_window(int samplerate, calibrate_config_t *config,
			  double centre, arfcn_power_t **ents, int n,
			  calibrate_cell_t *cells, cpu_meter_t *meter, double t_run_start,
			  const char *start_info)
{
	(void)start_info;
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
		cells[c].system = CALIB_GSM;
		cells[c].arfcn = ents[c]->arfcn;
		cells[c].gsm_band = ents[c]->band;
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
	double tc_read = 0.0, tc_sep = 0.0;	/* main thread CPU: reading, separation */
	int left = n, probe_end = 0;

	while (left > 0) {
		double t = now_sec() - t0;

		if (t > config->timeout_sec) {
			LOGP(DSDR, LOGL_NOTICE, "Window at %.3f MHz: timeout after %.0f s\n",
			     centre / 1e6, config->timeout_sec);
			break;
		}
		/* the budget is the whole program's (the reading too), with 15 %
		 * margin: what each part costs, measured here */
		if (meter && !meter->done && t >= CPU_MEASURE_SEC) {
			double thr = cpu_sec(CLOCK_THREAD_CPUTIME_ID) - thr0;
			double proc = cpu_sec(CLOCK_PROCESS_CPUTIME_ID) - proc0;

			meter->rate = samplerate;
			meter->transport = (proc - thr + tc_read) / t;
			meter->sep = tc_sep / t / n;
			meter->det = (thr - tc_read - tc_sep) / t / n;
			meter->per_channel = meter->sep + meter->det;
			meter->budget_channels = budget_at(meter, config->cpu_percent, samplerate);
			if (meter->budget_channels < 1)
				meter->budget_channels = 1;
			meter->done = 1;
			/* the rate may change: this window was the probe, its
			 * carriers are measured again at the rate chosen */
			if (config->reopen) {
				probe_end = 1;
				break;
			}
		}

		double tc = cpu_sec(CLOCK_THREAD_CPUTIME_ID);
		int got = receive_iq_blocking(band, block);
		if (got <= 0) {
			LOGP(DSDR, LOGL_ERROR, "No samples from the SDR\n");
			rc = -EIO;
			break;
		}
		double tc2 = cpu_sec(CLOCK_THREAD_CPUTIME_ID);
		tc_read += tc2 - tc;
		sdr_engine_rx_feed(&eng, band, got);
		int rows = sdr_engine_group_read(grp, oi, oq, rows_max);
		tc_sep += cpu_sec(CLOCK_THREAD_CPUTIME_ID) - tc2;
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
			/* a slow one (weak, at the window's edge: 50 FCCH once took
			 * 22 s) does not hold the window: what it has after SLOW_SEC */
			if (!ch->finished && t > SLOW_SEC && ch->nmeas >= SLOW_MIN && (ch->dec == DEC_DONE || !ch->sch_ok)) {
				LOGP(DSDR, LOGL_INFO, "  ARFCN %d: %d FCCH in %.1f s, enough\n", ch->entry->arfcn, ch->nmeas, t);
				ch->finished = 1;
			}
			/* no BCCH carrier */
			if (!ch->finished && ((t > NO_FCCH_SEC && ch->nmeas == 0) || (t > FEW_FCCH_SEC && ch->nmeas < 3))) {
				LOGP(DSDR, LOGL_INFO, "  ARFCN %d: %d FCCH in %.1f s, no BCCH carrier\n",
				     ch->entry->arfcn, ch->nmeas, t);
				ch->finished = 1;
			}
			/* tones too weak for a BCCH carrier that can confirm: the
			 * confirmed ones gave a mean FCCH SNR of 12.7-14 dB, the
			 * carriers without a BCCH 6.5-8.4 dB (B210, GSM-900/1800) */
			if (!ch->finished && ch->nmeas >= WEAK_FCCH_COUNT && !(config->confirm && ch->cell->confirmed)) {
				double s = 0.0;

				for (int i = 0; i < ch->nmeas; i++)
					s += ch->snrs[i];
				if (s / ch->nmeas < WEAK_FCCH_SNR_DB && !config->full) {
					LOGP(DSDR, LOGL_INFO, "  ARFCN %d: FCCH SNR %.1f dB after %d, too weak to confirm\n",
					     ch->entry->arfcn, s / ch->nmeas, ch->nmeas);
					ch->finished = 1;
				}
			}
			if (ch->finished) {
				cell_result(ch);
				left--;
			}
		}
	}
	if (probe_end) {
		LOGP(DSDR, LOGL_INFO, "Window at %.3f MHz: the cost measured in %.1f s\n", centre / 1e6, now_sec() - t0);
		rc = -EAGAIN;
		goto out_chan;
	}
	/* what did not finish (timeout, error) */
	for (c = 0; c < n; c++)
		if (!w[c].finished)
			cell_result(&w[c]);
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
 * An LTE window: the EARFCNs of a peak, one lte_cell each
 * ================================================================ */

/* a carrier is done: no cell found, or its cells measured */
static int lte_done(const lte_cell_t *lc, const calibrate_config_t *config, double t)
{
	int i, all = 1;

	if (t > (config->lte_timing ? LTE_TIMING_MAX_SEC : LTE_MAX_SEC))
		return 1;
	if (lte_cell_count(lc) == 0)
		return lc->searches >= 2;	/* lte_cell gives up after 2 */
	for (i = 0; i < lte_cell_count(lc); i++) {
		const lte_cell_info_t *ci = lte_cell_get(lc, i);

		if (ci->lost)
			continue;
		if (!lte_cell_confirmed(ci)) {
			if (ci->mib_good == 0 && t > LTE_NO_MIB_SEC)
				continue;
			all = 0;
			continue;
		}
		if (ci->cfo_points < LTE_MIN_POINTS || (ci->cfo_alias_margin < LTE_ALIAS_MIN && t < LTE_MAX_SEC / 2))
			all = 0;
		if (config->lte_timing && (ci->timing_points < 6 || ci->timing_std_ppm > LTE_TIMING_STD))
			all = 0;
	}
	return all;
}

/* the cells of a carrier into the result */
static void lte_result(const lte_cell_t *lc, const arfcn_power_t *ent, const calibrate_config_t *config,
		       calibrate_result_t *r, double t)
{
	for (int i = 0; i < lte_cell_count(lc) && r->num_cells < CALIB_MAX_CELLS; i++) {
		const lte_cell_info_t *ci = lte_cell_get(lc, i);
		calibrate_cell_t *cell = &r->cells[r->num_cells];
		static const int bw_rb[6] = { 6, 15, 25, 50, 75, 100 };
		int rb_ok = 0;

		/* SSS without a MIB: not listed (lte_cell's candidates) */
		if (ci->mib_good == 0)
			continue;
		/* the same cell from a raster neighbour: the confirmed one */
		int dup = -1;
		for (int k = 0; k < r->num_cells; k++)
			if (r->cells[k].system == CALIB_LTE && r->cells[k].lte.pci == ci->pci
			 && fabs(r->cells[k].freq_hz - ent->freq) < 150e3)
				dup = k;
		if (dup >= 0) {
			if (r->cells[dup].confirmed || !lte_cell_confirmed(ci))
				continue;
			cell = &r->cells[dup];
			r->num_cells--;	/* replaced, counted again below */
		}
		for (int k = 0; k < 6; k++)
			if (ci->n_rb == bw_rb[k])
				rb_ok = 1;
		memset(cell, 0, sizeof(*cell));
		cell->system = CALIB_LTE;
		cell->arfcn = ent->arfcn;
		cell->gsm_band = ent->band;
		cell->freq_hz = ent->freq;
		cell->ident.bsic = -1;
		cell->decoded = 1;
		cell->confirmed = lte_cell_confirmed(ci) && rb_ok;
		cell->confirm_sec = t;
		cell->hz_offset = ci->cfo_hz;
		cell->stddev_hz = ci->cfo_std_hz;
		cell->num_measurements = ci->cfo_points;
		cell->carrier_ppm = ci->cfo_hz / ent->freq * 1e6;
		cell->lte.timing = config->lte_timing;
		/* the timing is the crystal's whole error (--sdr-ppm corrects the
		 * carrier only): as a residual like the carrier's */
		cell->ppm_offset = config->lte_timing ? ci->timing_ppm - sdr_config->rx_ppm : cell->carrier_ppm;
		cell->level_dbfs = ci->rsrp_dbfs;
		cell->snr_db = ci->sinr_db;
		cell->lte.pci = ci->pci;
		cell->lte.n_rb = ci->n_rb;
		cell->lte.ports = ci->ports;
		cell->lte.tdd = ci->tdd;
		cell->lte.rsrp_dbfs = ci->rsrp_dbfs;
		cell->lte.rsrq_db = ci->rsrq_db;
		cell->lte.sinr_db = ci->sinr_db;
		cell->lte.alias_margin = ci->cfo_alias_margin;
		cell->lte.timing_ppm = ci->timing_ppm;
		cell->lte.timing_std_ppm = ci->timing_std_ppm;
		/* the 100 Hz alias choice uncertain (carrier), or the timing
		 * not settled (RTL): it counts less */
		if (config->lte_timing)
			cell->weight = (ci->timing_points >= 6 && ci->timing_std_ppm <= 2.5 * LTE_TIMING_STD) ? 1.0 : LTE_LOW_WEIGHT;
		else
			cell->weight = (ci->cfo_alias_margin >= LTE_ALIAS_MIN) ? 1.0 : LTE_LOW_WEIGHT;
		LOGP(DSDR, LOGL_INFO, "  LTE band %d EARFCN %d PCI %d: %s, %d RB, %d ports, %+.2f Hz +- %.2f (%.4f ppm, alias margin %.1f), timing %.4f +- %.4f ppm, RSRP %.1f dBFS, SINR %.1f dB\n",
		     ent->band, ent->arfcn, ci->pci, cell->confirmed ? "confirmed" : "NOT confirmed", ci->n_rb, ci->ports,
		     ci->cfo_hz, ci->cfo_std_hz, cell->carrier_ppm, ci->cfo_alias_margin, ci->timing_ppm, ci->timing_std_ppm,
		     ci->rsrp_dbfs, ci->sinr_db);
		r->num_cells++;
	}
}

/* Measure the LTE carriers ents[0..n-1] in the window of 'centre'; the cells
 * found go into the result. prior: the carrier ppm of the towers so far (or
 * NAN). Returns the number of confirmed cells, <0 on error. */
static int measure_window_lte(int samplerate, calibrate_config_t *config, double centre, arfcn_power_t **ents, int n,
			      calibrate_result_t *result, double prior)
{
	int block = calib_block_samples(samplerate);
	int use_bank = (n >= 4 && sdr_engine_bins(samplerate, LTE_DSP, n) > 0);
	sdr_engine_t eng;
	sdr_group_t *grp = NULL;
	double offs[SDR_ENGINE_MAX_CHAN];
	lte_cell_t *lc = calloc(n, sizeof(*lc));
	int *done = calloc(n, sizeof(int)), *lc_ok = calloc(n, sizeof(int));
	float *band = malloc((size_t)block * 2 * sizeof(float));
	int rows_max = (int)ceil((double)block * LTE_DSP / samplerate) + 64;
	sample_t **oi = calloc(n, sizeof(*oi)), **oq = calloc(n, sizeof(*oq));
	float *ciq = malloc((size_t)rows_max * 2 * sizeof(float));
	int rc = 0, c, left = n, confirmed = 0, eng_ok = 0;
	int64_t rows_total = 0;

	if (!lc || !done || !lc_ok || !band || !oi || !oq || !ciq) {
		rc = -ENOMEM;
		goto out;
	}
	for (c = 0; c < n; c++) {
		oi[c] = malloc(rows_max * sizeof(sample_t));
		oq[c] = malloc(rows_max * sizeof(sample_t));
		if (!oi[c] || !oq[c]) {
			rc = -ENOMEM;
			goto out;
		}
		offs[c] = ents[c]->freq - centre;
	}
	if (sdr_engine_init(&eng, samplerate, 0, use_bank ? LTE_DSP : 0, use_bank ? n : 0, block) < 0) {
		LOGP(DSDR, LOGL_ERROR, "Channel separation: %s\n", sdr_engine_error());
		rc = -EINVAL;
		goto out;
	}
	eng_ok = 1;
	grp = sdr_engine_group_add(&eng, LTE_DSP, n, offs, 0, NULL, 1000, 0);
	if (!grp) {
		LOGP(DSDR, LOGL_ERROR, "Channel separation: %s\n", sdr_engine_error());
		rc = -EINVAL;
		goto out;
	}
	for (c = 0; c < n; c++) {
		double f = ents[c]->freq, cfo_c, cfo_m;

		if (!isnan(prior)) {
			cfo_c = prior * 1e-6 * f;
			cfo_m = LTE_PRIOR_PPM * 1e-6 * f + LTE_PRIOR_HZ;
		} else {
			cfo_c = 0.0;
			cfo_m = LTE_CFO_PPM * 1e-6 * f;
		}
		if (cfo_m > LTE_CFO_MAX_HZ)
			cfo_m = LTE_CFO_MAX_HZ;
		if (lte_cell_init(&lc[c], LTE_DSP, f, cfo_c, cfo_m) < 0) {
			rc = -ENOMEM;
			goto out;
		}
		lc_ok[c] = 1;
	}
	LOGP(DSDR, LOGL_INFO, "LTE window at %.3f MHz: EARFCN %d..%d (band %d, %s, %d Hz each), offset searched %s\n",
	     centre / 1e6, ents[0]->arfcn, ents[n - 1]->arfcn, ents[0]->band, use_bank ? "filter bank" : "per-channel chain",
	     LTE_DSP, isnan(prior) ? "without a prior" : "around the towers' ppm");

	rc = retune_rx(centre);
	if (rc < 0) {
		LOGP(DSDR, LOGL_ERROR, "Failed to retune to %.3f MHz\n", centre / 1e6);
		goto out;
	}
	rc = 0;
	settle_rx();

	double t0 = now_sec();
	while (left > 0) {
		double t = now_sec() - t0;

		if (t > config->timeout_sec)
			break;
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
		rows_total += rows;
		for (c = 0; c < n; c++) {
			/* stream time of this channel, from its late start */
			double tc = (double)rows_total / LTE_DSP - c * LTE_STAGGER_SEC;

			if (done[c] || tc <= 0.0)
				continue;
			for (int s = 0; s < rows; s++) {
				ciq[2 * s] = (float)oi[c][s];
				ciq[2 * s + 1] = (float)oq[c][s];
			}
			lte_cell_process(&lc[c], ciq, rows);
			if (lte_done(&lc[c], config, tc)) {
				int before = result->num_cells;

				done[c] = 1;
				left--;
				lte_result(&lc[c], ents[c], config, result, tc);
				for (int k = before; k < result->num_cells; k++)
					if (result->cells[k].confirmed)
						confirmed++;
				if (result->num_cells == before && config->verbosity >= 2)
					LOGP(DSDR, LOGL_INFO, "  EARFCN %d: no cell (%.2f s)\n", ents[c]->arfcn, tc);
			}
		}
	}
	for (c = 0; c < n; c++)
		if (!done[c])
			lte_result(&lc[c], ents[c], config, result, (double)rows_total / LTE_DSP);
	LOGP(DSDR, LOGL_INFO, "LTE window at %.3f MHz done after %.1f s, %d confirmed\n", centre / 1e6, now_sec() - t0, confirmed);
	rc = (rc < 0) ? rc : confirmed;
out:
	for (c = 0; lc && c < n; c++)
		if (lc_ok[c])
			lte_cell_exit(&lc[c]);
	if (eng_ok)
		sdr_engine_exit(&eng);
	if (oi && oq)
		for (c = 0; c < n; c++) {
			free(oi[c]);
			free(oq[c]);
		}
	free(oi);
	free(oq);
	free(ciq);
	free(band);
	free(lc);
	free(done);
	free(lc_ok);
	return rc;
}

/* ================================================================
 * The result: confirmed towers that agree
 * ================================================================ */

static int cell_usable(const calibrate_cell_t *cell, const calibrate_config_t *config)
{
	if (cell->system == CALIB_LTE)
		return cell->confirmed && (cell->lte.timing ? cell->lte.timing_std_ppm > 0.0 : cell->num_measurements >= 3);
	if (cell->num_measurements < 3)
		return 0;
	return config->confirm ? cell->confirmed : 1;
}

/* the carrier ppm of the towers so far (LTE searches around it), NAN if none */
static double prior_ppm(const calibrate_result_t *r, const calibrate_config_t *config)
{
	double ppm[CALIB_MAX_CELLS];
	int n = 0;

	for (int i = 0; i < r->num_cells; i++)
		if (cell_usable(&r->cells[i], config) && r->cells[i].confirmed && r->cells[i].weight >= 1.0)
			ppm[n++] = r->cells[i].carrier_ppm;
	if (!n)
		return NAN;
	qsort(ppm, n, sizeof(double), compare_double);
	return (n % 2) ? ppm[n / 2] : 0.5 * (ppm[n / 2 - 1] + ppm[n / 2]);
}

/* the result from the cells so far; returns the number of agreeing towers
 * (of full weight: an LTE cell with an uncertain alias counts in the mean
 * with less weight, not as a tower) */
static int evaluate(calibrate_result_t *r, const calibrate_config_t *config)
{
	double ppm[CALIB_MAX_CELLS], med, sum = 0.0, wsum = 0.0, lo = 1e9, hi = -1e9;
	int n = 0, agree = 0, i, full = 0;

	for (i = 0; i < r->num_cells; i++)
		if (cell_usable(&r->cells[i], config) && r->cells[i].weight >= 1.0)
			full++;
	for (i = 0; i < r->num_cells; i++) {
		r->cells[i].agrees = 0;
		if (cell_usable(&r->cells[i], config)) {
			/* the median from the full-weight ones if any */
			if (!full || r->cells[i].weight >= 1.0)
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
			double w = r->cells[i].weight > 0.0 ? r->cells[i].weight : 1.0;

			r->cells[i].agrees = 1;
			sum += w * r->cells[i].ppm_offset;
			wsum += w;
			if (w >= 1.0)
				agree++;
		} else
			r->disagree++;
	}
	r->num_agree = agree;
	/* none near the median (two towers apart): no result, both listed */
	r->ppm_offset = (wsum > 0.0) ? sum / wsum : med;
	r->valid = (wsum > 0.0);
	return agree;
}

/* the window centre for the strongest carrier not measured yet (index
 * first): on the half-channel grid (no carrier on the DC), with the
 * strongest in reach, covering as many other carriers not measured yet as
 * possible (ties: the strongest's side) */
static double window_centre(const arfcn_power_t *list, int n, int first, double reach, int per_window)
{
	double best_centre = list[first].freq + CHANNEL_SPACING / 2.0;
	double best = -1.0;

	if (per_window <= 1)
		return best_centre;
	for (int j = -(int)(reach / CHANNEL_SPACING) - 1; j <= (int)(reach / CHANNEL_SPACING); j++) {
		double c = list[first].freq + CHANNEL_SPACING / 2.0 + j * CHANNEL_SPACING;
		double count = 0.0;
		int rank = 0;

		if (fabs(list[first].freq - c) > reach)
			continue;
		/* the strong ones count (the 2 x per_window strongest not
		 * measured yet), the weaker only to break ties: they rarely
		 * confirm */
		for (int i = 0; i < n; i++) {
			if (list[i].done)
				continue;
			if (fabs(list[i].freq - c) <= reach && fabs(list[i].freq - c) >= DC_GUARD_HZ)
				count += (rank < 2 * per_window) ? 1.0 : 0.001;
			rank++;
		}
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

/* channels one window can hold at this rate (the usable part; without a
 * bank, at most the chain's) */
static int rate_capacity(int samplerate)
{
	int capacity = window_capacity(samplerate);

	if (capacity > SDR_ENGINE_MAX_CHAN)
		capacity = SDR_ENGINE_MAX_CHAN;
	if (sdr_engine_bins(samplerate, gsm_channel_rate(samplerate), 8) == 0 && capacity > SDR_ENGINE_MAX_CHAIN)
		capacity = SDR_ENGINE_MAX_CHAIN;
	return capacity;
}

/* After the measurement: the rate (from the one opened, the widest, down
 * by halves to 1 MS/s) with the most channels per window in the budget
 * (ties: the wider, no reopen), from what the reading, the separation and
 * the detection cost at the rate opened; reopens the SDR if another one.
 * Logs the start line. Returns 0, <0 if the reopen failed. */
/* the most carriers not measured yet that one window at this rate takes */
static int rate_cover(const arfcn_power_t *list, int n, int samplerate)
{
	double reach = window_reach(samplerate);
	int best = 0;

	for (int f = 0; f < n; f++) {
		if (list[f].done)
			continue;
		double c = window_centre(list, n, f, reach, 2);
		int k = 0;

		for (int i = 0; i < n; i++)
			if (!list[i].done && fabs(list[i].freq - c) <= reach && fabs(list[i].freq - c) >= DC_GUARD_HZ)
				k++;
		if (k > best)
			best = k;
	}
	return best;
}

/* channels a window at this rate measures: the budget's, the capacity's,
 * and the carriers left in reach */
static int rate_channels(const cpu_meter_t *m, double cpu_percent, const arfcn_power_t *list, int n, int r)
{
	int k = budget_at(m, cpu_percent, r), c = rate_capacity(r), v = rate_cover(list, n, r);

	if (k > c)
		k = c;
	if (k > v)
		k = v;
	return k;
}

static int choose_rate(calibrate_config_t *config, cpu_meter_t *m, int *samplerate, const char *start_info,
		       const arfcn_power_t *list, int nlist)
{
	int best = *samplerate, best_n, r, n, max_n, i, nr = 0;
	int rates[16], chans[16];

	/* the rates from the one open down by halves, and what each measures;
	 * the widest that gets at least 3/4 of the most (a wider window takes
	 * the strong carriers together, a narrower one fills up with weak
	 * ones nearby) */
	rates[nr] = best;
	chans[nr++] = rate_channels(m, config->cpu_percent, list, nlist, best);
	if (config->reopen && !config->num_channels) {
		for (r = *samplerate / 2; r >= 1000000 && !(r % 2) && nr < 16; r /= 2) {
			if (gsm_channel_rate(r) < GSM_RATE)
				break;
			rates[nr] = r;
			chans[nr++] = rate_channels(m, config->cpu_percent, list, nlist, r);
		}
	}
	for (max_n = 0, i = 0; i < nr; i++)
		if (chans[i] > max_n)
			max_n = chans[i];
	best_n = chans[0];
	for (i = 0; i < nr; i++) {
		if (4 * chans[i] >= 3 * max_n) {
			best = rates[i];
			best_n = chans[i];
			break;
		}
	}
	n = 0;
	LOGP(DSDR, LOGL_NOTICE, "%s; measured at %d S/s: reading %.1f %% of a core, a channel %.1f %% (separation %.1f %%, detection %.1f %%)\n",
	     start_info ? start_info : "", m->rate, 100.0 * m->transport, 100.0 * m->per_channel, 100.0 * m->sep, 100.0 * m->det);
	if (best != *samplerate) {
		int rc;

		LOGP(DSDR, LOGL_NOTICE, "SDR rate %d S/s instead of %d: %d carriers per window (budget, capacity, carriers in reach) instead of %d; reopening the SDR\n",
		     best, *samplerate, best_n, rate_channels(m, config->cpu_percent, list, nlist, *samplerate));
		rc = config->reopen(best);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Reopening the SDR at %d S/s failed\n", best);
			return rc;
		}
		*samplerate = best;
	}
	n = budget_at(m, config->cpu_percent, *samplerate);
	if (n > rate_capacity(*samplerate))
		n = rate_capacity(*samplerate);
	m->budget_channels = (n < 1) ? 1 : n;
	LOGP(DSDR, LOGL_NOTICE, "SDR rate %d S/s, %d channels per window (%s%.0f %% of a core with the reading, 15 %% kept free)\n",
	     *samplerate, config->num_channels ? config->num_channels : m->budget_channels,
	     config->num_channels ? "forced; budget " : "budget ", config->cpu_percent);
	if (config->num_channels) {
		double x = (double)*samplerate / m->rate;
		double need = config->num_channels * (m->sep * x + m->det) + m->transport * x;

		if (need > config->cpu_percent / 100.0)
			LOGP(DSDR, LOGL_NOTICE, "--channels %d needs about %.0f %% of a core with the reading, more than the budget of %.0f %%\n",
			     config->num_channels, 100.0 * need, config->cpu_percent);
	}
	return 0;
}

/* An LTE band: its peaks, strongest first, a window each (the peak's
 * EARFCN and its raster neighbours), until enough towers agree or too many
 * peaks in a row without a confirmed cell. Returns 0, <0 to stop. */
static int calibrate_lte_band(int *samplerate, calibrate_config_t *config, const calib_band_t *cb,
			      calibrate_result_t *result, double t_start, int *stop)
{
	arfcn_power_t *list;
	int n = lte_candidates(*samplerate, cb, config, result, &list), since = 0, i;
	double lreach;

	if (n <= 0) {
		free(list);
		return 0;
	}
	/* the windows' rate: the chain costs at the SDR rate, the scan was
	 * done at the widest */
	if (*samplerate > LTE_RATE_MAX && config->reopen) {
		int r = *samplerate;

		while (r > LTE_RATE_MAX && !(r % 2) && r / 2 >= LTE_DSP)
			r /= 2;
		if (r != *samplerate) {
			LOGP(DSDR, LOGL_NOTICE, "LTE: SDR rate %d S/s instead of %d for the carriers (the separation costs at the SDR rate); reopening the SDR\n",
			     r, *samplerate);
			if (config->reopen(r) < 0) {
				LOGP(DSDR, LOGL_ERROR, "Reopening the SDR at %d S/s failed\n", r);
				free(list);
				*stop = 1;
				return -EIO;
			}
			*samplerate = r;
		}
	}
	lreach = usable_half(*samplerate) - CALIB_LTE_HALF_HZ;
	if (lreach < 0.0) {
		LOGP(DSDR, LOGL_ERROR, "LTE band %d: %d S/s holds no LTE carrier (+-%.0f kHz usable, +-%.0f kHz needed)\n",
		     cb->id, *samplerate, usable_half(*samplerate) / 1e3, CALIB_LTE_HALF_HZ / 1e3);
		free(list);
		return 0;
	}
	while (!*stop) {
		arfcn_power_t *ents[3];
		int first = -1, k = 0, pk;
		double centre = 0.0;

		for (i = 0; i < n; i++)
			if (!list[i].done) {
				first = i;
				break;
			}
		if (first < 0 || result->num_cells >= CALIB_MAX_CELLS)
			break;
		pk = list[first].peak;
		for (i = first; i < n && list[i].peak == pk && k < 3; i++)
			if (!list[i].done)
				ents[k++] = &list[i];
		for (i = 0; i < k; i++)
			centre += ents[i]->freq / k;
		/* the LO outside the carrier's 6 RB where the rate allows, else
		 * on its (empty) DC subcarrier */
		if (LTE_LO_OFF_HZ + 100e3 <= lreach)
			centre += LTE_LO_OFF_HZ;
		{
			int kk = 0;

			for (i = 0; i < k; i++) {
				ents[i]->done = 1;
				if (fabs(ents[i]->freq - centre) <= lreach + 1.0) {
					lte_tested_add(ents[i]->freq);
					ents[kk++] = ents[i];
				}
			}
			k = kk;
		}
		if (!k)
			continue;
		int rc = measure_window_lte(*samplerate, config, centre, ents, k, result, prior_ppm(result, config));
		if (rc == -EIO) {
			*stop = 1;
			break;
		}
		if (rc < 0)
			break;
		result->windows++;
		since = (rc > 0) ? 0 : since + 1;
		int agree = evaluate(result, config);
		if (!config->full && agree >= config->towers) {
			result->time_to_result = now_sec() - t_start;
			*stop = 1;
		} else if (!config->full && since >= LTE_WEAK_IN_A_ROW) {
			LOGP(DSDR, LOGL_NOTICE, "LTE band %d: %d peaks in a row without a confirmed cell: the weaker rest is not measured (--full to measure all)\n",
			     cb->id, since);
			result->stopped_weak = 1;
			break;
		}
	}
	free(list);
	return 0;
}

int calibrate_auto(int samplerate, calibrate_config_t *config, calibrate_result_t *result,
		   const char *start_info)
{
	int stop = 0, since_confirmed = 0, any_gsm = 0;
	int capacity, per_window;
	double reach;

	for (int b = 0; b < config->num_bands; b++)
		if (config->bands[b].system == CALIB_GSM && config->bands[b].scan)
			any_gsm = 1;

	if (config->usable_fraction > 0.0)
		calib_usable_fraction = config->usable_fraction;
	calib_usable_max_hz = config->usable_max_hz;
	capacity = rate_capacity(samplerate);
	reach = window_reach(samplerate);
	double t_start = now_sec();
	cpu_meter_t meter = { 0 };

	memset(result, 0, sizeof(*result));
	result->samplerate = samplerate;
	if (capacity < 1 && any_gsm) {
		LOGP(DSDR, LOGL_ERROR, "%d S/s is too narrow for a GSM channel\n", samplerate);
		return -EINVAL;
	}
	if (any_gsm && config->num_channels > capacity) {
		LOGP(DSDR, LOGL_ERROR, "--channels %d does not fit: a window at %d S/s holds %d channels (+-%.0f kHz around the centre%s)\n",
		     config->num_channels, samplerate, capacity, reach / 1e3,
		     (capacity == SDR_ENGINE_MAX_CHAIN) ? "; no filter bank fits this rate: at most 4 on the per-channel chain" : "");
		return -EINVAL;
	}
	per_window = config->num_channels ? config->num_channels : FIRST_WINDOW_CHANNELS;
	if (per_window > capacity)
		per_window = capacity;

	lte_tested_n = 0;
	if (spec_init(config) < 0) {
		LOGP(DSDR, LOGL_ERROR, "No band to measure\n");
		return -EINVAL;
	}

	for (int b = 0; b < config->num_bands && !stop; b++) {
		const calib_band_t *cb = &config->bands[b];
		int band = cb->id;
		arfcn_power_t *list;
		int n;

		if (!cb->scan)
			continue;
		if (cb->system == CALIB_LTE) {
			if (calibrate_lte_band(&samplerate, config, cb, result, t_start, &stop) < 0)
				break;
			capacity = rate_capacity(samplerate);
			reach = window_reach(samplerate);
			result->samplerate = samplerate;
			if (per_window > capacity)
				per_window = capacity;
			continue;
		}
		n = band_candidates(samplerate, cb, config, result, &list);
		if (n <= 0) {
			free(list);
			continue;
		}
		since_confirmed = 0;
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
			double centre = window_centre(list, n, first, reach, per_window);
			/* the strongest first, then the next strongest in reach */
			for (i = first; i < n && k < per_window && result->num_cells + k < CALIB_MAX_CELLS; i++) {
				if (list[i].done || fabs(list[i].freq - centre) > reach
				 || fabs(list[i].freq - centre) < DC_GUARD_HZ)
					continue;
				ents[k++] = &list[i];
				list[i].done = 1;
			}
			int rc = measure_window(samplerate, config, centre, ents, k,
						&result->cells[result->num_cells],
						meter.done ? NULL : &meter, t_start, start_info);
			if (rc == -EAGAIN) {
				/* the probe: measured again at the rate chosen */
				for (i = 0; i < k; i++)
					ents[i]->done = 0;
				k = 0;
			} else if (rc < 0 && rc != -EIO)
				break;
			result->num_cells += k;
			if (k)
				result->windows++;
			if (rc == -EIO) {
				stop = 1;
				break;
			}
			/* measured: the rate and the channels per window */
			if (meter.done && !meter.applied) {
				meter.applied = 1;
				if (choose_rate(config, &meter, &samplerate, start_info, list, n) < 0) {
					stop = 1;
					break;
				}
				capacity = rate_capacity(samplerate);
				reach = window_reach(samplerate);
				result->samplerate = samplerate;
				if (!config->num_channels)
					per_window = meter.budget_channels;
				if (per_window > capacity)
					per_window = capacity;
			}
			/* strongest first: after this many in a row without a
			 * confirmed one, the rest (weaker) are not worth it */
			{
				int any = 0;

				for (i = result->num_cells - k; i < result->num_cells; i++)
					if (result->cells[i].confirmed)
						any = 1;
				since_confirmed = any ? 0 : since_confirmed + k;
			}
			int agree = evaluate(result, config);
			if (!config->full && agree >= ((config->forced_arfcn > 0) ? 1 : config->towers)) {
				result->time_to_result = now_sec() - t_start;
				stop = 1;
			} else if (!config->full && since_confirmed >= WEAK_IN_A_ROW) {
				LOGP(DSDR, LOGL_NOTICE, "%s: %d carriers in a row without a confirmed one: the weaker rest is not measured (--full to measure all)\n",
				     gsm_band_name(band), since_confirmed);
				result->stopped_weak = 1;
				break;
			}
		}
		free(list);
	}

	result->channels_per_window = config->num_channels ? config->num_channels : per_window;
	result->cpu_per_channel = meter.per_channel;
	result->cpu_transport = meter.transport;
	spec_exit();
	evaluate(result, config);
	if (!result->time_to_result)
		result->time_to_result = now_sec() - t_start;
	return result->valid ? 0 : -ENODATA;
}
