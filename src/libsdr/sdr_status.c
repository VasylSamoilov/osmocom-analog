/*
 * SDR Status Monitoring Implementation
 *
 * Collects TX/RX IQ statistics into structured status objects.
 * Computes derived metrics (noise floor, SNR, crest factor, signal quality)
 * on each snapshot interval.
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
#include "sdr_status.h"

/* Global status instance */
sdr_status_t sdr_status;

void sdr_status_init(int num_channels, double report_interval)
{
	memset(&sdr_status, 0, sizeof(sdr_status));
	sdr_status.report_interval = report_interval > 0.0 ? report_interval : 1.0;
	sdr_status.rx.num_channels = num_channels;
	if (num_channels > SDR_STATUS_MAX_CHANNELS)
		sdr_status.rx.num_channels = SDR_STATUS_MAX_CHANNELS;
}

void sdr_status_reset(void)
{
	double interval = sdr_status.report_interval;
	int num_ch = sdr_status.rx.num_channels;
	memset(&sdr_status, 0, sizeof(sdr_status));
	sdr_status.report_interval = interval;
	sdr_status.rx.num_channels = num_ch;
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
 * Estimate noise floor from histogram.
 *
 * Strategy: The noise floor is the average amplitude of the lowest-energy bins.
 * For a signal-free spectrum, all energy is noise and the histogram is concentrated
 * in the low bins. When a signal is present, the histogram becomes bimodal —
 * noise in low bins, signal in higher bins.
 *
 * We find the noise floor by looking at the lowest bins that contain samples
 * but are clearly below the signal peak.
 */
static double estimate_noise_floor(sdr_rx_status_t *rx)
{
	long total = rx->_sample_count;
	if (total == 0)
		return 0.0;

	/* Find the bin with the most samples (dominant energy) */
	int peak_bin = 0;
	long peak_count = 0;
	int i;
	for (i = 0; i < SDR_STATUS_HIST_BINS; i++) {
		if (rx->_hist[i] > peak_count) {
			peak_count = rx->_hist[i];
			peak_bin = i;
		}
	}

	/* If peak is in bin 0 (most energy near zero), the noise floor IS the signal.
	 * This means either no signal or very weak signal. */
	if (peak_bin == 0) {
		/* Noise floor = average amplitude */
		return total ? (rx->_sum_i + rx->_sum_q) / (2.0 * total) : 0.0;
	}

	/* Signal is in higher bins. Noise floor = weighted average of bins below the peak.
	 * Use bins 0 through (peak_bin - 1) as noise estimate. */
	double noise_sum = 0.0;
	long noise_count = 0;
	for (i = 0; i < peak_bin && i < SDR_STATUS_HIST_BINS; i++) {
		if (rx->_hist[i] > 0) {
			/* Center of bin i is (i + 0.5) / BINS */
			double bin_center = ((double)i + 0.5) / SDR_STATUS_HIST_BINS;
			noise_sum += bin_center * rx->_hist[i];
			noise_count += rx->_hist[i];
		}
	}

	if (noise_count > 0)
		return noise_sum / noise_count;

	/* No samples in lower bins — signal fills entire range.
	 * Use the lowest non-empty bin as noise estimate. */
	for (i = 0; i < SDR_STATUS_HIST_BINS; i++) {
		if (rx->_hist[i] > 0)
			return ((double)i + 0.5) / SDR_STATUS_HIST_BINS;
	}

	return 0.0;
}

/*
 * Determine signal quality assessment
 */
static enum sdr_signal_quality assess_quality(sdr_rx_status_t *rx)
{
	if (rx->sample_count == 0)
		return SDR_SIGNAL_NONE;

	/* Saturated: >80% of samples clipping */
	if (rx->clip_ratio > 0.80)
		return SDR_SIGNAL_SATURATED;

	/* Clipping: >5% of samples clipping */
	if (rx->clip_ratio > 0.05)
		return SDR_SIGNAL_CLIPPING;

	/* Check if there's meaningful signal above noise */
	double peak = rx->peak_i > rx->peak_q ? rx->peak_i : rx->peak_q;
	if (peak < 0.01)
		return SDR_SIGNAL_NONE;

	if (peak < 0.05)
		return SDR_SIGNAL_WEAK;

	if (peak > 0.8)
		return SDR_SIGNAL_STRONG;

	return SDR_SIGNAL_GOOD;
}

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
		LOGP(DSDR, LOGL_NOTICE, "TX IQ LEVEL: peak I=%.4f Q=%.4f | avg I=%.4f Q=%.4f | rms=%.4f | active_ch=%d/%d | amplitude=%.4f\n",
		     tx->peak_i, tx->peak_q, tx->avg_i, tx->avg_q, tx->rms,
		     tx->active_channels, tx->total_channels, tx->amplitude);
		for (i = 0; i < tx->total_channels && i < SDR_STATUS_MAX_CHANNELS; i++) {
			LOGP(DSDR, LOGL_NOTICE, "TX CH%d: power_on=%s duty=%.1f%% freq=%.6f MHz\n",
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

		/* Noise floor estimation */
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

		/* Per-channel RX status (after FM demod) */
		for (i = 0; i < rx->num_channels && i < SDR_STATUS_MAX_CHANNELS; i++) {
			sdr_rx_chan_status_t *c = &rx->chan[i];
			if (!c->valid)
				continue;

			/* Compute average rf_level this period */
			double avg_rf_db = c->_rf_count > 0
				? c->_rf_sum_db / c->_rf_count : c->rf_level_db;

			/*
			 * In-channel noise floor tracking:
			 *
			 * The minimum rf_level seen during this 1-second window
			 * approximates the in-channel noise floor (the quietest
			 * moment on this channel). We smooth it with exponential
			 * moving average to avoid jumps.
			 *
			 * Key insight: only update the noise floor when the
			 * channel appears idle. When a signal is present
			 * (has_signal from previous period), the minimum
			 * rf_level is still the signal — not noise. Updating
			 * upward during active signal would contaminate the
			 * noise floor estimate.
			 *
			 * When signal disappears, the noise floor quickly
			 * re-acquires the true idle level (alpha=0.5 downward).
			 */
			if (c->_rf_count > 0) {
				double min_this_period = c->_rf_min_db;
				if (!c->noise_floor_valid) {
					/* First measurement — seed the noise floor */
					c->noise_floor_db = min_this_period;
					c->noise_floor_valid = 1;
				} else if (!c->has_signal) {
					/* Channel is idle — safe to update noise floor.
					 * Drop fast, rise slow. */
					double alpha;
					if (min_this_period < c->noise_floor_db)
						alpha = 0.5;  /* drop fast */
					else
						alpha = 0.1;  /* rise slow */
					c->noise_floor_db = c->noise_floor_db
						+ alpha * (min_this_period - c->noise_floor_db);
				}
				/* else: signal present — freeze noise floor */
			}

			/* SNR: current rf_level above in-channel noise floor */
			if (c->noise_floor_valid)
				c->snr_db = avg_rf_db - c->noise_floor_db;
			else
				c->snr_db = 0.0;

			/* Signal detection: SNR > 6 dB is a reasonable threshold
			 * for FM signal presence above in-channel noise */
			c->has_signal = (c->snr_db > 6.0) ? 1 : 0;

			LOGP(DSDR, LOGL_NOTICE, "RX CH%d: rf=%.1f dB nf=%.1f dB snr=%.1f dB %s | dev=%.0f Hz offset=%.0f Hz freq=%.6f MHz\n",
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

		LOGP(DSDR, LOGL_NOTICE, "HW: TX peak=%.4f avg=%.4f clip=%d | RX peak=%.6f avg=%.4f rail=%d\n",
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

const sdr_status_t *sdr_status_get(void)
{
	return &sdr_status;
}
