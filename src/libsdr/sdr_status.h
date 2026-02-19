/*
 * SDR Status Monitoring
 *
 * Structured status objects for TX/RX IQ signal monitoring.
 * Replaces scattered static variables with proper per-channel
 * and global status that can be queried by other subsystems
 * (VMAC power control, display, diagnostics).
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef SDR_STATUS_H
#define SDR_STATUS_H

#include <stdint.h>

#define SDR_STATUS_MAX_CHANNELS	16
#define SDR_STATUS_HIST_BINS	10

/* Signal quality assessment */
enum sdr_signal_quality {
	SDR_SIGNAL_NONE = 0,	/* no signal detected */
	SDR_SIGNAL_WEAK,	/* signal below noise floor threshold */
	SDR_SIGNAL_GOOD,	/* signal in usable range */
	SDR_SIGNAL_STRONG,	/* signal strong but not clipping */
	SDR_SIGNAL_CLIPPING,	/* ADC clipping detected */
	SDR_SIGNAL_SATURATED,	/* ADC fully saturated */
};

/* Per-channel TX status */
typedef struct sdr_tx_chan_status {
	double		frequency;	/* TX frequency in Hz */
	int		power_on;	/* carrier active this period */
	double		duty_cycle;	/* fraction of time power was on (0.0-1.0) */
	long		power_on_samples;
	long		total_samples;
} sdr_tx_chan_status_t;

/* Global TX status (combined IQ after modulation) */
typedef struct sdr_tx_status {
	/* IQ levels (pre-LIMIT_IQ_LEVEL scaling, at DSP rate) */
	double		peak_i, peak_q;
	double		avg_i, avg_q;
	double		rms;		/* RMS of combined IQ magnitude */

	/* Channel info */
	int		active_channels;
	int		total_channels;
	double		amplitude;	/* per-channel amplitude (1/channels) */
	sdr_tx_chan_status_t chan[SDR_STATUS_MAX_CHANNELS];

	/* Accumulation state */
	double		sum_i, sum_q;
	double		sum_sq;		/* sum of (I^2 + Q^2) for RMS */
	long		sample_count;
	double		report_timer;
	int		valid;		/* set after first report */
} sdr_tx_status_t;

/* Per-channel RX status (after FM/AM demod) */
typedef struct sdr_rx_chan_status {
	double		frequency;	/* RX frequency in Hz */
	double		rf_level_db;	/* RF level in dB (from IQ magnitude) */
	double		freq_offset_hz;	/* frequency offset from center */
	double		deviation_hz;	/* peak-to-peak FM deviation */
	double		snr_db;		/* estimated in-channel SNR in dB */
	int		has_signal;	/* 1 if signal detected above noise */
	int		valid;

	/* In-channel noise floor tracking (internal accumulation) */
	double		_rf_sum_db;	/* sum of rf_level_db samples */
	double		_rf_min_db;	/* minimum rf_level_db this period */
	double		_rf_max_db;	/* maximum rf_level_db this period */
	long		_rf_count;	/* number of rf_level updates */
	double		noise_floor_db;	/* tracked in-channel noise floor (dB) */
	int		noise_floor_valid; /* set once we have a baseline */
} sdr_rx_chan_status_t;

/* Global RX status (raw IQ from SDR hardware) */
typedef struct sdr_rx_status {
	/* IQ levels */
	double		peak_i, peak_q;
	double		avg_i, avg_q;
	double		rms;		/* RMS of IQ magnitude */

	/* DC offset (signed mean — nonzero indicates bias) */
	double		dc_offset_i, dc_offset_q;

	/* Clipping */
	int		clip_soft;	/* samples > 0.98 */
	int		clip_hard;	/* samples > 0.999 (ADC rail) */
	long		sample_count;
	double		clip_ratio;	/* fraction of samples clipping (0.0-1.0) */

	/* Amplitude histogram (10 bins: [0,0.1), [0.1,0.2), ... [0.9,1.0]) */
	long		hist[SDR_STATUS_HIST_BINS];
	double		hist_pct[SDR_STATUS_HIST_BINS]; /* normalized percentages */

	/* Noise floor estimation
	 * Estimated from the lowest-energy histogram bins when signal is present.
	 * When no signal: noise_floor ≈ avg amplitude.
	 * When signal present: noise_floor from bins below signal peak. */
	double		noise_floor;	/* estimated noise floor (linear, 0.0-1.0) */
	double		noise_floor_db;	/* noise floor in dB relative to full scale */

	/* Signal presence detection */
	double		signal_power;	/* estimated signal power (linear) */
	double		signal_power_db;/* signal power in dBFS */
	double		dynamic_range_db;/* signal_power_db - noise_floor_db */
	enum sdr_signal_quality quality;

	/* Crest factor: peak/RMS ratio — high values indicate impulsive signal or clipping */
	double		crest_factor_db;

	/* Per-channel (after demod) */
	sdr_rx_chan_status_t chan[SDR_STATUS_MAX_CHANNELS];
	int		num_channels;

	/* Accumulation state (internal) */
	double		_sum_i, _sum_q;
	double		_raw_sum_i, _raw_sum_q;	/* signed sums for DC offset */
	double		_sum_sq;		/* sum of (I^2 + Q^2) for RMS */
	double		_peak_i, _peak_q;
	int		_clip_soft, _clip_hard;
	long		_hist[SDR_STATUS_HIST_BINS];
	long		_sample_count;
	double		_report_timer;
	int		valid;		/* set after first report */
} sdr_rx_status_t;

/* Soapy/driver-level status (raw hardware interface) */
typedef struct sdr_hw_status {
	/* TX output (after LIMIT_IQ_LEVEL scaling + upsample, at SDR rate) */
	double		tx_peak;
	double		tx_avg;
	int		tx_clip_count;
	long		tx_sample_count;
	double		tx_sdr_rate;

	/* RX input (raw from SoapySDR, before any processing) */
	double		rx_peak;
	double		rx_avg;
	int		rx_rail_count;	/* samples at ADC maximum */
	long		rx_sample_count;

	/* TX accumulation */
	double		_tx_peak;
	double		_tx_sum;
	int		_tx_clip;
	long		_tx_count;
	double		_tx_timer;

	/* RX accumulation */
	double		_rx_peak;
	double		_rx_sum;
	int		_rx_rail;
	long		_rx_count;
	double		_rx_timer;

	int		valid;
} sdr_hw_status_t;

/* Top-level SDR status — one global instance */
typedef struct sdr_status {
	sdr_tx_status_t	tx;
	sdr_rx_status_t	rx;
	sdr_hw_status_t	hw;
	double		report_interval;	/* seconds between snapshots (default 1.0) */
} sdr_status_t;

/* Global status instance */
extern sdr_status_t sdr_status;

/* API */
void sdr_status_init(int num_channels, double report_interval);
void sdr_status_reset(void);

/* Called from sdr_write to accumulate TX samples */
void sdr_status_tx_accumulate(float *buff, int num, uint8_t **power, int channels,
			      double amplitude, double *chan_tx_freq);

/* Called from sdr_read to accumulate RX IQ samples */
void sdr_status_rx_accumulate(float *buff, int num);

/* Called from sdr_read per-channel after FM demod */
void sdr_status_rx_chan_update(int ch, double rf_level_db, double freq_offset_hz,
			       double deviation_hz, double frequency);

/* Called from soapy_send to accumulate HW TX output */
void sdr_status_hw_tx_accumulate(float *buff, int num, double sdr_rate);

/* Called from soapy_receive to accumulate HW RX input */
void sdr_status_hw_rx_accumulate(float *buff, int num);

/* Snapshot: compute derived values, log, and reset accumulators.
 * Returns 1 if a new snapshot was taken, 0 if not yet time. */
int sdr_status_snapshot(double now);

/* Query current status (returns pointer to last snapshot) */
const sdr_status_t *sdr_status_get(void);

#endif /* SDR_STATUS_H */
