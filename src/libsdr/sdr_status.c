/*
 * SDR Status Monitoring Implementation
 *
 * Collects TX/RX IQ statistics into structured status objects.
 * Computes derived metrics (noise floor, SNR, crest factor, signal quality)
 * on each snapshot interval.
 *
 * FFT-based spectral measurement (same algorithm as scan):
 *   - Signal power: FFT at registered frequencies (±20 kHz integration)
 *   - Noise floor: minimum power across offset frequencies (skip ±100 kHz DC)
 *   - Output: dBFS values, caller computes SNR = signal - noise
 *
 * CPU cost: ~0.05ms per FFT, negligible even at 20 Msps.
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
#include <string.h>
#include <math.h>
#include "../liblogging/logging.h"
#include "../libfft/fft.h"
#include "sdr_status.h"

/* Global status instance */
sdr_status_t sdr_status;

void sdr_status_init(int num_channels, double report_interval)
{
	memset(&sdr_status, 0, sizeof(sdr_status));
	sdr_status.report_interval = report_interval > 0.0 ? report_interval : 1.0;
	sdr_status.spectral_interval = 0.1;  /* 10Hz default for fast signal meter updates */
	sdr_status.rx.num_channels = num_channels;
	if (num_channels > SDR_STATUS_MAX_CHANNELS)
		sdr_status.rx.num_channels = SDR_STATUS_MAX_CHANNELS;
}

void sdr_status_reset(void)
{
	double interval = sdr_status.report_interval;
	double spectral_interval = sdr_status.spectral_interval;
	int num_ch = sdr_status.rx.num_channels;
	sdr_spectral_t spectral_save = sdr_status.spectral;
	memset(&sdr_status, 0, sizeof(sdr_status));
	sdr_status.report_interval = interval;
	sdr_status.spectral_interval = spectral_interval;
	sdr_status.rx.num_channels = num_ch;
	/* Preserve spectral config (frequencies, rates) but reset accumulators */
	sdr_status.spectral = spectral_save;
	sdr_status.spectral._fft_fill = 0;
	sdr_status.spectral._fft_count = 0;
	memset(sdr_status.spectral._power_acc, 0, sizeof(sdr_status.spectral._power_acc));
	memset(sdr_status.spectral._power_cnt, 0, sizeof(sdr_status.spectral._power_cnt));
	sdr_status.spectral._noise_acc = 0.0;
	sdr_status.spectral._noise_cnt = 0;
}

/*
 * TX accumulation — called from sdr_write after FM/AM modulation
 */
void sdr_status_tx_accumulate(float *buff, int num, uint8_t **power, int channels,
			      double amplitude, double *chan_tx_freq)
{
	sdr_tx_status_t *tx = &sdr_status.tx;
	int c, s;

	tx->amplitude = amplitude;
	tx->total_channels = channels;

	/* Per-channel power state */
	int active = 0;
	for (c = 0; c < channels && c < SDR_STATUS_MAX_CHANNELS; c++) {
		tx->chan[c].frequency = chan_tx_freq ? chan_tx_freq[c] : 0.0;
		int ch_on = 0;
		for (s = 0; s < num; s++) {
			if (power[c][s]) { ch_on = 1; break; }
		}
		if (ch_on) {
			active++;
			tx->chan[c].power_on = 1;
			tx->chan[c].power_on_samples += num;
		}
		tx->chan[c].total_samples += num;
	}
	tx->active_channels = active;

	/* Combined IQ levels */
	for (s = 0; s < num; s++) {
		double vi = fabs(buff[s * 2]);
		double vq = fabs(buff[s * 2 + 1]);
		if (vi > tx->peak_i) tx->peak_i = vi;
		if (vq > tx->peak_q) tx->peak_q = vq;
		tx->sum_i += vi;
		tx->sum_q += vq;
		tx->sum_sq += vi * vi + vq * vq;
	}
	tx->sample_count += num;
}

/*
 * RX IQ accumulation — called from sdr_read on raw IQ buffer
 */
void sdr_status_rx_accumulate(float *buff, int num)
{
	sdr_rx_status_t *rx = &sdr_status.rx;
	int s;

	for (s = 0; s < num; s++) {
		double ri = buff[s * 2];
		double rq = buff[s * 2 + 1];
		double vi = fabs(ri);
		double vq = fabs(rq);

		if (vi > rx->_peak_i) rx->_peak_i = vi;
		if (vq > rx->_peak_q) rx->_peak_q = vq;
		rx->_sum_i += vi;
		rx->_sum_q += vq;
		rx->_raw_sum_i += ri;
		rx->_raw_sum_q += rq;
		rx->_sum_sq += vi * vi + vq * vq;

		/* Histogram bin */
		double vmax = vi > vq ? vi : vq;
		int bin = (int)(vmax * SDR_STATUS_HIST_BINS);
		if (bin >= SDR_STATUS_HIST_BINS) bin = SDR_STATUS_HIST_BINS - 1;
		rx->_hist[bin]++;

		/* Clipping detection */
		if (vi > 0.98 || vq > 0.98)
			rx->_clip_soft++;
		if (vi >= 0.999 || vq >= 0.999)
			rx->_clip_hard++;
	}
	rx->_sample_count += num;
}

/*
 * Per-channel RX update — called after FM/AM demod
 *
 * Accumulates per-channel RF level for noise floor tracking.
 * Called many times per second (once per DSP buffer).
 */
void sdr_status_rx_chan_update(int ch, double rf_level_db, double freq_offset_hz,
			       double deviation_hz, double frequency)
{
	if (ch < 0 || ch >= SDR_STATUS_MAX_CHANNELS)
		return;
	sdr_rx_chan_status_t *c = &sdr_status.rx.chan[ch];
	c->frequency = frequency;
	c->rf_level_db = rf_level_db;
	c->freq_offset_hz = freq_offset_hz;
	c->deviation_hz = deviation_hz;
	c->valid = 1;

	/* Accumulate for per-channel noise floor / SNR */
	c->_rf_sum_db += rf_level_db;
	if (c->_rf_count == 0 || rf_level_db < c->_rf_min_db)
		c->_rf_min_db = rf_level_db;
	if (c->_rf_count == 0 || rf_level_db > c->_rf_max_db)
		c->_rf_max_db = rf_level_db;
	c->_rf_count++;
}

/*
 * HW TX accumulation — called from soapy_send
 */
void sdr_status_hw_tx_accumulate(float *buff, int num, double sdr_rate)
{
	sdr_hw_status_t *hw = &sdr_status.hw;
	int i;

	hw->tx_sdr_rate = sdr_rate;
	for (i = 0; i < num * 2; i++) {
		double v = fabs(buff[i]);
		if (v > hw->_tx_peak) hw->_tx_peak = v;
		hw->_tx_sum += v;
		if (v > 0.95) hw->_tx_clip++;
	}
	hw->_tx_count += num * 2;
}

/*
 * HW RX accumulation — called from soapy_receive
 */
void sdr_status_hw_rx_accumulate(float *buff, int num)
{
	sdr_hw_status_t *hw = &sdr_status.hw;
	int i;

	for (i = 0; i < num * 2; i++) {
		double v = fabs(buff[i]);
		if (v > hw->_rx_peak) hw->_rx_peak = v;
		hw->_rx_sum += v;
		if (v >= 0.999) hw->_rx_rail++;
	}
	hw->_rx_count += num * 2;
}

/*
 * Estimate noise floor from histogram (kept for backward compat / diagnostics).
 */
static double estimate_noise_floor(sdr_rx_status_t *rx)
{
	long total = rx->_sample_count;
	if (total == 0)
		return 0.0;

	int peak_bin = 0;
	long peak_count = 0;
	int i;
	for (i = 0; i < SDR_STATUS_HIST_BINS; i++) {
		if (rx->_hist[i] > peak_count) {
			peak_count = rx->_hist[i];
			peak_bin = i;
		}
	}

	if (peak_bin == 0)
		return total ? (rx->_sum_i + rx->_sum_q) / (2.0 * total) : 0.0;

	double noise_sum = 0.0;
	long noise_count = 0;
	for (i = 0; i < peak_bin && i < SDR_STATUS_HIST_BINS; i++) {
		if (rx->_hist[i] > 0) {
			double bin_center = ((double)i + 0.5) / SDR_STATUS_HIST_BINS;
			noise_sum += bin_center * rx->_hist[i];
			noise_count += rx->_hist[i];
		}
	}

	if (noise_count > 0)
		return noise_sum / noise_count;

	for (i = 0; i < SDR_STATUS_HIST_BINS; i++) {
		if (rx->_hist[i] > 0)
			return ((double)i + 0.5) / SDR_STATUS_HIST_BINS;
	}

	return 0.0;
}

static enum sdr_signal_quality assess_quality(sdr_rx_status_t *rx)
{
	if (rx->sample_count == 0)
		return SDR_SIGNAL_NONE;
	if (rx->clip_ratio > 0.80)
		return SDR_SIGNAL_SATURATED;
	if (rx->clip_ratio > 0.05)
		return SDR_SIGNAL_CLIPPING;

	double peak = rx->peak_i > rx->peak_q ? rx->peak_i : rx->peak_q;
	if (peak < 0.01)
		return SDR_SIGNAL_NONE;
	if (peak < 0.05)
		return SDR_SIGNAL_WEAK;
	if (peak > 0.8)
		return SDR_SIGNAL_STRONG;
	return SDR_SIGNAL_GOOD;
}

#if 0  /* Unused - was for verbose RX IQ debug */
static const char *quality_str(enum sdr_signal_quality q)
{
	switch (q) {
	case SDR_SIGNAL_NONE:		return "NONE";
	case SDR_SIGNAL_WEAK:		return "WEAK";
	case SDR_SIGNAL_GOOD:		return "GOOD";
	case SDR_SIGNAL_STRONG:		return "STRONG";
	case SDR_SIGNAL_CLIPPING:	return "CLIPPING";
	case SDR_SIGNAL_SATURATED:	return "SATURATED";
	}
	return "?";
}
#endif

/* ================================================================
 * FFT-based spectral measurement (same algorithm as scan)
 *
 * Signal power: FFT at registered frequencies with ±20 kHz integration
 * Noise floor: minimum power across offset frequencies (skip ±100 kHz DC)
 *
 * This matches the scan algorithm exactly, so signal meter output
 * will be consistent with scan results (SNR = signal - noise).
 * ================================================================ */

void sdr_spectral_configure(int sdr_rate, double center_hz)
{
	sdr_spectral_t *sp = &sdr_status.spectral;

	sp->sdr_rate = sdr_rate;
	sp->center_hz = center_hz;
	sp->_fft_fill = 0;
	sp->_fft_count = 0;

	/* Calculate optimal FFTs per update based on sample rate and update interval.
	 * FFT size = 4096, so max FFTs/sec = sdr_rate / 4096.
	 * With 10Hz updates (0.1 sec), we want enough FFTs to average but not too many.
	 * Target: 4 FFTs per update for good averaging without blocking. */
	int max_fft_per_sec = sdr_rate / SDR_SPECTRAL_FFT_SIZE;
	int fft_per_update = (int)(max_fft_per_sec * sdr_status.spectral_interval);
	if (fft_per_update < 1) fft_per_update = 1;
	if (fft_per_update > 4) fft_per_update = 4;  /* cap at 4 for fast updates */

	sp->_fft_target = fft_per_update;
	sp->noise_floor_dbfs = -120.0;
	sp->noise_floor_valid = 0;
	sp->enabled = 1;

	/* Reset accumulators */
	memset(sp->_power_acc, 0, sizeof(sp->_power_acc));
	memset(sp->_power_cnt, 0, sizeof(sp->_power_cnt));
	sp->_noise_acc = 0.0;
	sp->_noise_cnt = 0;

	LOGP(DSDR, LOGL_INFO, "Spectral measurement: rate=%d center=%.0f Hz fft_per_update=%d (max_fft/sec=%d, interval=%.2fs)\n",
	     sdr_rate, center_hz, sp->_fft_target, max_fft_per_sec, sdr_status.spectral_interval);
}

void sdr_spectral_set_center(double center_hz)
{
	sdr_spectral_t *sp = &sdr_status.spectral;
	if (sp->center_hz == center_hz)
		return;
	sp->center_hz = center_hz;
	/* Reset FFT buffer on retune — partial data is from old frequency */
	sp->_fft_fill = 0;
}

int sdr_spectral_register_freq(double freq_hz)
{
	sdr_spectral_t *sp = &sdr_status.spectral;

	if (sp->num_freq >= SDR_SPECTRAL_MAX_FREQ)
		return -1;

	/* Check if within SDR bandwidth */
	if (sp->sdr_rate > 0) {
		double offset = freq_hz - sp->center_hz;
		double half_bw = sp->sdr_rate / 2.0;
		if (offset < -half_bw || offset >= half_bw) {
			LOGP(DSDR, LOGL_NOTICE,
			     "Spectral: freq %.0f Hz outside SDR bandwidth (center=%.0f ±%.0f)\n",
			     freq_hz, sp->center_hz, half_bw);
			/* Register anyway — will be measured when/if center changes */
		}
	}

	int idx = sp->num_freq++;
	sp->freq[idx].frequency = freq_hz;
	sp->freq[idx].power_dbfs = -120.0;
	sp->freq[idx].valid = 0;
	sp->_power_acc[idx] = 0.0;
	sp->_power_cnt[idx] = 0;

	LOGP(DSDR, LOGL_INFO, "Spectral: registered freq[%d] = %.0f Hz\n", idx, freq_hz);
	return idx;
}

void sdr_spectral_unregister_freq(int idx)
{
	sdr_spectral_t *sp = &sdr_status.spectral;
	if (idx < 0 || idx >= sp->num_freq)
		return;

	/* Shift remaining entries down */
	for (int i = idx; i < sp->num_freq - 1; i++) {
		sp->freq[i] = sp->freq[i + 1];
		sp->_power_acc[i] = sp->_power_acc[i + 1];
		sp->_power_cnt[i] = sp->_power_cnt[i + 1];
	}
	sp->num_freq--;
}

void sdr_spectral_clear_freq(void)
{
	sdr_spectral_t *sp = &sdr_status.spectral;
	sp->num_freq = 0;
	memset(sp->_power_acc, 0, sizeof(sp->_power_acc));
	memset(sp->_power_cnt, 0, sizeof(sp->_power_cnt));
}

/*
 * Process one completed FFT frame.
 * 1. Measures power at registered frequencies (including DC/tuned frequency)
 * 2. Measures power at offset frequencies, takes minimum as noise floor
 * Same approach as scan: ±20 kHz integration window, minimum of offsets = noise floor.
 */
static void spectral_process_fft(sdr_spectral_t *sp)
{
	const int N = SDR_SPECTRAL_FFT_SIZE;
	double bin_hz = (double)sp->sdr_rate / N;
	int half_bins = (int)(20000.0 / bin_hz);  /* ±20 kHz integration window */
	if (half_bins < 1) half_bins = 1;

	/* Run FFT (in-place, divides by N) */
	fft_process(1, SDR_SPECTRAL_FFT_M, sp->_fft_i, sp->_fft_q);

	/* 1. Measure power at each registered frequency (same method as scan) */
	for (int i = 0; i < sp->num_freq; i++) {
		double offset_hz = sp->freq[i].frequency - sp->center_hz;
		double half_bw = sp->sdr_rate / 2.0;

		/* Skip if outside current SDR bandwidth */
		if (offset_hz < -half_bw || offset_hz >= half_bw)
			continue;

		/* Map offset to FFT bin (with wrap for negative offsets) */
		int center_bin = (int)round(offset_hz / bin_hz);

		/* Integrate power over ±20 kHz window around channel center */
		double power_sum = 0.0;
		for (int b = center_bin - half_bins; b <= center_bin + half_bins; b++) {
			int wb = (b < 0 ? b + N : b) & (N - 1);
			double re = sp->_fft_i[wb];
			double im = sp->_fft_q[wb];
			power_sum += re * re + im * im;
		}
		double avg_power = power_sum / (2 * half_bins + 1);
		sp->_power_acc[i] += avg_power;
		sp->_power_cnt[i]++;
	}

	/* 2. Measure noise floor: power at offset frequencies (skip ±100 kHz around DC).
	 * Take minimum as noise floor estimate. */
	double min_power = 1e30;
	int step_bins = (int)(100000.0 / bin_hz);  /* 100 kHz steps */
	int dc_skip_bins = (int)(100000.0 / bin_hz);  /* skip ±100 kHz around DC */

	/* Measure positive frequencies */
	for (int center = dc_skip_bins; center < N/2 - half_bins; center += step_bins) {
		double power_sum = 0.0;
		for (int b = center - half_bins; b <= center + half_bins; b++) {
			double re = sp->_fft_i[b];
			double im = sp->_fft_q[b];
			power_sum += re * re + im * im;
		}
		double avg_power = power_sum / (2 * half_bins + 1);
		if (avg_power < min_power)
			min_power = avg_power;
	}

	/* Measure negative frequencies (bins N/2+1 to N-1) */
	for (int center = N/2 + dc_skip_bins; center < N - half_bins; center += step_bins) {
		double power_sum = 0.0;
		for (int b = center - half_bins; b <= center + half_bins; b++) {
			double re = sp->_fft_i[b];
			double im = sp->_fft_q[b];
			power_sum += re * re + im * im;
		}
		double avg_power = power_sum / (2 * half_bins + 1);
		if (avg_power < min_power)
			min_power = avg_power;
	}

	if (min_power < 1e29) {
		sp->_noise_acc += min_power;
		sp->_noise_cnt++;
	}

	sp->_fft_count++;
}

void sdr_spectral_feed_iq(const float *iq_buf, int count)
{
	sdr_spectral_t *sp = &sdr_status.spectral;

	if (!sp->enabled || !iq_buf || count <= 0)
		return;

	/* Already have enough FFTs for this snapshot period — skip until reset */
	if (sp->_fft_count >= sp->_fft_target)
		return;

	/* Fill FFT buffer from IQ stream */
	const int N = SDR_SPECTRAL_FFT_SIZE;
	int i = 0;

	while (i < count && sp->_fft_count < sp->_fft_target) {
		/* Copy samples into FFT buffer */
		int remaining = N - sp->_fft_fill;
		int available = count - i;
		int to_copy = remaining < available ? remaining : available;

		for (int j = 0; j < to_copy; j++) {
			sp->_fft_i[sp->_fft_fill] = (double)iq_buf[(i + j) * 2];
			sp->_fft_q[sp->_fft_fill] = (double)iq_buf[(i + j) * 2 + 1];
			sp->_fft_fill++;
		}
		i += to_copy;

		/* FFT buffer full — process it */
		if (sp->_fft_fill >= N) {
			spectral_process_fft(sp);
			sp->_fft_fill = 0;
		}
	}
}

/*
 * Finalize spectral measurements for this snapshot period.
 * Converts accumulated linear power to dBFS and resets accumulators.
 */
static void spectral_snapshot(sdr_spectral_t *sp)
{
	const int N = SDR_SPECTRAL_FFT_SIZE;
	/* FFT normalization: the C fft_process divides by N, so to get
	 * proper dBFS we add 20*log10(N) back. */
	double fft_norm = 20.0 * log10((double)N);
	int i;

	if (!sp->enabled || sp->_fft_count == 0)
		return;

	/* Noise floor */
	if (sp->_noise_cnt > 0) {
		double avg_noise = sp->_noise_acc / sp->_noise_cnt;
		sp->noise_floor_dbfs = (avg_noise > 1e-30)
			? 10.0 * log10(avg_noise) + fft_norm : -120.0;
		sp->noise_floor_valid = 1;
	}

	/* Per-frequency power */
	for (i = 0; i < sp->num_freq; i++) {
		if (sp->_power_cnt[i] > 0) {
			double avg_power = sp->_power_acc[i] / sp->_power_cnt[i];
			sp->freq[i].power_dbfs = (avg_power > 1e-30)
				? 10.0 * log10(avg_power) + fft_norm : -120.0;
			sp->freq[i].valid = 1;
		}
	}

	/* Reset accumulators for next period */
	sp->_fft_fill = 0;
	sp->_fft_count = 0;
	memset(sp->_power_acc, 0, sizeof(sp->_power_acc));
	memset(sp->_power_cnt, 0, sizeof(sp->_power_cnt));
	sp->_noise_acc = 0.0;
	sp->_noise_cnt = 0;
}

/* Query functions */

double sdr_spectral_get_noise_floor(void)
{
	sdr_spectral_t *sp = &sdr_status.spectral;
	return sp->noise_floor_valid ? sp->noise_floor_dbfs : -120.0;
}

double sdr_spectral_get_power(int idx)
{
	sdr_spectral_t *sp = &sdr_status.spectral;
	if (idx < 0 || idx >= sp->num_freq || !sp->freq[idx].valid)
		return -120.0;
	return sp->freq[idx].power_dbfs;
}

double sdr_spectral_get_snr(int idx)
{
	sdr_spectral_t *sp = &sdr_status.spectral;
	if (idx < 0 || idx >= sp->num_freq || !sp->freq[idx].valid || !sp->noise_floor_valid)
		return 0.0;
	double snr = sp->freq[idx].power_dbfs - sp->noise_floor_dbfs;
	return snr > 0.0 ? snr : 0.0;
}

int sdr_spectral_is_valid(void)
{
	return sdr_status.spectral.noise_floor_valid;
}

/*
 * Snapshot: compute derived values from accumulators, log, and reset.
 * Returns 1 if snapshot was taken.
 */
int sdr_status_snapshot(double now)
{
	sdr_tx_status_t *tx = &sdr_status.tx;
	sdr_rx_status_t *rx = &sdr_status.rx;
	sdr_hw_status_t *hw = &sdr_status.hw;
	double interval = sdr_status.report_interval;
	int i;

	/* Initialize timers on first call */
	if (tx->report_timer == 0.0) tx->report_timer = now;
	if (rx->_report_timer == 0.0) rx->_report_timer = now;
	if (hw->_tx_timer == 0.0) hw->_tx_timer = now;
	if (hw->_rx_timer == 0.0) hw->_rx_timer = now;

	if (now - tx->report_timer < interval)
		return 0;

	/* === TX snapshot === */
	if (tx->sample_count > 0) {
		tx->avg_i = tx->sum_i / tx->sample_count;
		tx->avg_q = tx->sum_q / tx->sample_count;
		tx->rms = sqrt(tx->sum_sq / tx->sample_count);

		for (i = 0; i < tx->total_channels && i < SDR_STATUS_MAX_CHANNELS; i++) {
			sdr_tx_chan_status_t *c = &tx->chan[i];
			c->duty_cycle = c->total_samples > 0
				? (double)c->power_on_samples / c->total_samples : 0.0;
		}
		tx->valid = 1;

		/* Log TX status */
		LOGP(DSDR, LOGL_DEBUG, "TX IQ LEVEL: peak I=%.4f Q=%.4f | avg I=%.4f Q=%.4f | rms=%.4f | active_ch=%d/%d | amplitude=%.4f\n",
		     tx->peak_i, tx->peak_q, tx->avg_i, tx->avg_q, tx->rms,
		     tx->active_channels, tx->total_channels, tx->amplitude);
		for (i = 0; i < tx->total_channels && i < SDR_STATUS_MAX_CHANNELS; i++) {
			LOGP(DSDR, LOGL_DEBUG, "TX CH%d: power_on=%s duty=%.1f%% freq=%.6f MHz\n",
			     i, tx->chan[i].power_on ? "YES" : "NO",
			     tx->chan[i].duty_cycle * 100.0,
			     tx->chan[i].frequency / 1e6);
		}
	}

	/* === RX snapshot === */
	if (rx->_sample_count > 0) {
		long n = rx->_sample_count;

		/* Copy peaks and counts */
		rx->peak_i = rx->_peak_i;
		rx->peak_q = rx->_peak_q;
		rx->avg_i = rx->_sum_i / n;
		rx->avg_q = rx->_sum_q / n;
		rx->dc_offset_i = rx->_raw_sum_i / n;
		rx->dc_offset_q = rx->_raw_sum_q / n;
		rx->rms = sqrt(rx->_sum_sq / n);
		rx->clip_soft = rx->_clip_soft;
		rx->clip_hard = rx->_clip_hard;
		rx->sample_count = n;
		rx->clip_ratio = (double)rx->_clip_soft / n;

		/* Histogram percentages */
		for (i = 0; i < SDR_STATUS_HIST_BINS; i++) {
			rx->hist[i] = rx->_hist[i];
			rx->hist_pct[i] = 100.0 * rx->_hist[i] / n;
		}

		/* Noise floor estimation (histogram-based, kept for diagnostics) */
		rx->noise_floor = estimate_noise_floor(rx);
		rx->noise_floor_db = rx->noise_floor > 0.0
			? 20.0 * log10(rx->noise_floor) : -120.0;

		/* Signal power (RMS-based) */
		rx->signal_power = rx->rms;
		rx->signal_power_db = rx->rms > 0.0
			? 20.0 * log10(rx->rms) : -120.0;

		/* Dynamic range */
		rx->dynamic_range_db = rx->signal_power_db - rx->noise_floor_db;

		/* Crest factor: peak / RMS */
		double peak_max = rx->peak_i > rx->peak_q ? rx->peak_i : rx->peak_q;
		rx->crest_factor_db = (rx->rms > 0.0 && peak_max > 0.0)
			? 20.0 * log10(peak_max / rx->rms) : 0.0;

		/* Quality assessment */
		rx->quality = assess_quality(rx);
		rx->valid = 1;

#if 0  /* Verbose RX IQ debug - disabled */
		/* Log RX status */
		LOGP(DSDR, LOGL_DEBUG, "RX IQ: peak=%.4f/%.4f avg=%.4f/%.4f rms=%.4f DC=%.4f/%.4f | %s\n",
		     rx->peak_i, rx->peak_q, rx->avg_i, rx->avg_q, rx->rms,
		     rx->dc_offset_i, rx->dc_offset_q, quality_str(rx->quality));
		if (rx->clip_soft > 0) {
			LOGP(DSDR, LOGL_NOTICE, "RX CLIP: soft=%d hard=%d / %ld (%.2f%%/%.2f%%)\n",
			     rx->clip_soft, rx->clip_hard, n,
			     100.0 * rx->clip_soft / n,
			     100.0 * rx->clip_hard / n);
		}
		LOGP(DSDR, LOGL_DEBUG, "RX HIST: [0-0.1]=%.1f%% [.1-.2]=%.1f%% [.2-.3]=%.1f%% [.3-.4]=%.1f%% [.4-.5]=%.1f%% [.5-.6]=%.1f%% [.6-.7]=%.1f%% [.7-.8]=%.1f%% [.8-.9]=%.1f%% [.9-1]=%.1f%%\n",
		     rx->hist_pct[0], rx->hist_pct[1], rx->hist_pct[2],
		     rx->hist_pct[3], rx->hist_pct[4], rx->hist_pct[5],
		     rx->hist_pct[6], rx->hist_pct[7], rx->hist_pct[8],
		     rx->hist_pct[9]);
		LOGP(DSDR, LOGL_DEBUG, "RX ANALYSIS: noise_floor=%.1f dBFS signal=%.1f dBFS dynamic_range=%.1f dB crest=%.1f dB\n",
		     rx->noise_floor_db, rx->signal_power_db,
		     rx->dynamic_range_db, rx->crest_factor_db);
#endif

		/* Per-channel RX status (after FM demod) */
		for (i = 0; i < rx->num_channels && i < SDR_STATUS_MAX_CHANNELS; i++) {
			sdr_rx_chan_status_t *c = &rx->chan[i];
			if (!c->valid)
				continue;

			double avg_rf_db = c->_rf_count > 0
				? c->_rf_sum_db / c->_rf_count : c->rf_level_db;

			if (c->_rf_count > 0) {
				double min_this_period = c->_rf_min_db;
				if (!c->noise_floor_valid) {
					c->noise_floor_db = min_this_period;
					c->noise_floor_valid = 1;
				} else if (!c->has_signal) {
					double alpha;
					if (min_this_period < c->noise_floor_db)
						alpha = 0.5;
					else
						alpha = 0.1;
					c->noise_floor_db = c->noise_floor_db
						+ alpha * (min_this_period - c->noise_floor_db);
				}
			}

			if (c->noise_floor_valid)
				c->snr_db = avg_rf_db - c->noise_floor_db;
			else
				c->snr_db = 0.0;

			c->has_signal = (c->snr_db > 6.0) ? 1 : 0;

			LOGP(DSDR, LOGL_DEBUG, "RX CH%d: rf=%.1f dB nf=%.1f dB snr=%.1f dB %s | dev=%.0f Hz offset=%.0f Hz freq=%.6f MHz\n",
			     i, avg_rf_db, c->noise_floor_db, c->snr_db,
			     c->has_signal ? "SIGNAL" : "noise",
			     c->deviation_hz, c->freq_offset_hz,
			     c->frequency / 1e6);

			/* Reset per-period accumulators (keep noise_floor) */
			c->_rf_sum_db = 0.0;
			c->_rf_min_db = 0.0;
			c->_rf_max_db = 0.0;
			c->_rf_count = 0;
		}
	}

	/* === HW snapshot === */
	if (hw->_tx_count > 0) {
		hw->tx_peak = hw->_tx_peak;
		hw->tx_avg = hw->_tx_sum / hw->_tx_count;
		hw->tx_clip_count = hw->_tx_clip;
		hw->tx_sample_count = hw->_tx_count;
		hw->valid = 1;
	}
	if (hw->_rx_count > 0) {
		hw->rx_peak = hw->_rx_peak;
		hw->rx_avg = hw->_rx_sum / hw->_rx_count;
		hw->rx_rail_count = hw->_rx_rail;
		hw->rx_sample_count = hw->_rx_count;
		hw->valid = 1;

		LOGP(DSDR, LOGL_DEBUG, "HW: TX peak=%.4f avg=%.4f clip=%d | RX peak=%.6f avg=%.4f rail=%d\n",
		     hw->tx_peak, hw->tx_avg, hw->tx_clip_count,
		     hw->rx_peak, hw->rx_avg, hw->rx_rail_count);
	}

	/* === Reset accumulators === */
	/* TX */
	tx->peak_i = tx->peak_q = 0.0;
	tx->sum_i = tx->sum_q = tx->sum_sq = 0.0;
	tx->sample_count = 0;
	for (i = 0; i < SDR_STATUS_MAX_CHANNELS; i++) {
		tx->chan[i].power_on = 0;
		tx->chan[i].power_on_samples = 0;
		tx->chan[i].total_samples = 0;
	}
	tx->report_timer = now;

	/* RX */
	rx->_peak_i = rx->_peak_q = 0.0;
	rx->_sum_i = rx->_sum_q = 0.0;
	rx->_raw_sum_i = rx->_raw_sum_q = 0.0;
	rx->_sum_sq = 0.0;
	rx->_clip_soft = rx->_clip_hard = 0;
	rx->_sample_count = 0;
	for (i = 0; i < SDR_STATUS_HIST_BINS; i++)
		rx->_hist[i] = 0;
	rx->_report_timer = now;

	/* HW */
	hw->_tx_peak = 0.0;
	hw->_tx_sum = 0.0;
	hw->_tx_clip = 0;
	hw->_tx_count = 0;
	hw->_tx_timer = now;
	hw->_rx_peak = 0.0;
	hw->_rx_sum = 0.0;
	hw->_rx_rail = 0;
	hw->_rx_count = 0;
	hw->_rx_timer = now;

	return 1;
}

/*
 * Spectral-only snapshot: update FFT measurements at higher rate (10Hz default).
 * This allows the signal meter to update faster than the main status logging.
 * Returns 1 if spectral data was updated, 0 if not yet time.
 */
int sdr_spectral_snapshot(double now)
{
	double interval = sdr_status.spectral_interval;

	/* Initialize timer on first call */
	if (sdr_status._spectral_timer == 0.0)
		sdr_status._spectral_timer = now;

	if (now - sdr_status._spectral_timer < interval)
		return 0;

	/* Update spectral measurements */
	spectral_snapshot(&sdr_status.spectral);
	sdr_status._spectral_timer = now;

	return 1;
}

/*
 * Set spectral update interval.
 * Default is 0.1 seconds (10Hz). Range: 0.1 to 1.0 seconds (1-10 Hz).
 */
void sdr_spectral_set_interval(double interval_sec)
{
	if (interval_sec < 0.1)
		interval_sec = 0.1;  /* max 10 Hz */
	if (interval_sec > 1.0)
		interval_sec = 1.0;  /* min 1 Hz */
	sdr_status.spectral_interval = interval_sec;
	LOGP(DSDR, LOGL_INFO, "Spectral update interval set to %.3f sec (%.1f Hz)\n",
	     interval_sec, 1.0 / interval_sec);
}

const sdr_status_t *sdr_status_get(void)
{
	return &sdr_status;
}
