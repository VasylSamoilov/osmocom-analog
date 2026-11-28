#ifndef SDR_H
#define SDR_H

enum paging_signal;

int sdr_start(void *inst);
void *sdr_open(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index);
void sdr_close(void *inst);
int sdr_write(void *inst, sample_t **samples, uint8_t **power, int num, enum paging_signal *paging_signal, int *on, int channels);
int sdr_read(void *inst, sample_t **samples, int num, int channels, double *rf_level_db);
int sdr_get_tosend(void *inst, int buffer_size);
/* the SDR cannot go on (TX transport broken): the program's exit code, 0: works */
int sdr_fatal(void);

/* The TX device's clock: reference, time source, lock sensors, device time. Read in the main loop
 * only (control-path calls that can block for milliseconds: never in the read/write threads). */
typedef struct sdr_clock_status {
	int		valid;			/* read at least once */
	char		clock_source[32];	/* "internal", "external", "gpsdo" ... */
	char		time_source[32];
	int		ref_locked;		/* 1 locked, 0 not, -1 no such sensor */
	int		gps_locked;		/* 1 locked, 0 not, -1 no such sensor (no GPSDO) */
	unsigned long	ref_changes, gps_changes; /* lock changes seen since the start */
	int		have_time;
	/* the device time and the system clock at (nearly) the same moment. Without PPS the
	 * device time is not UTC: it counts from when UHD set it at the start, so the two give the
	 * device clock's drift against the system clock, not an offset to UTC. system_time is the
	 * middle of the system clock read before and after the device's; time_uncertainty half the
	 * span (the USB control round trip): reject wide ones. */
	double		device_time, system_time, time_uncertainty;
	double		system_raw;	/* the same moment on CLOCK_MONOTONIC_RAW: no NTP slews, the
					 * fair yardstick for the device clock's rate */
	double		poll_ms, poll_ms_max;	/* how long the last / longest poll blocked the main loop */
} sdr_clock_status_t;
/* poll the clock status (rate-limited to once a second; logs changes only): from the main loop */
void sdr_clock_poll(void);
/* the last status; 0, or -1 if the device has none (no UHD TX) */
int sdr_clock_status(sdr_clock_status_t *status);
double sdr_set_rx_trim(void *inst, int channel, double hz);
void sdr_set_rx_bandwidth(void *inst, int channel, double hz);
void calibrate_bias(void);
int sdr_get_samplerate(void *inst);
int sdr_set_rx_frequency(double frequency);
/* RX after sdr_set_rx_frequency(): drop what the ring holds and receives
 * until settle_sec after the retune returned, then what it holds at that
 * moment, so the next sdr_read() gives samples from the new frequency only,
 * settled (the ring keeps what came before the retune; while a retune runs
 * the B210 delivers zeros). Blocks until then. Returns the
 * number of SDR rate samples dropped. */
long sdr_rx_settle(void *inst, double settle_sec);
/* Another RX rate without reopening the device: for a band receiver whose
 * band passes unchanged (channels = 0, DSP rate = SDR rate, no LO offset or
 * shift), RX only, at most the rate it was opened at (the buffers and the
 * device's receive frames are sized for that). The read thread is stopped,
 * the device's stream stopped and started again at the new rate (analog
 * bandwidth = the rate unless --sdr-bandwidth was given), the ring
 * emptied; sdr_read() then gives the new rate (sdr_get_samplerate()), and
 * sdr_rx_settle() drops what came before the change settled. Returns 0,
 * -ENOTSUP for a configuration it does not cover (with the reason logged),
 * <0 when the device refused (the stream is then stopped). */
int sdr_set_rx_rate(void *inst, int rate);
int sdr_set_tx_frequency(double frequency);
int sdr_calculate_optimal_rate(int master_rate, double bandwidth);
/* the device of an SDR backend: call before sdr_open() (no channels, the
 * frequencies are the centres, the DSP rate the highest of the programs);
 * nmax: most TX channels. The engine for sdr_backend_start(). */
struct sdr_engine;
void sdr_server_mode(int nmax);
struct sdr_engine *sdr_server_engine(void *inst);
void sdr_server_poll(void *inst);

/* SDR capability information */
typedef struct sdr_caps {
	/* Frequency ranges (Hz) - 0 means unknown/unlimited */
	double		rx_freq_min;
	double		rx_freq_max;
	double		tx_freq_min;
	double		tx_freq_max;

	/* Gain ranges (dB) */
	double		rx_gain_min;
	double		rx_gain_max;
	double		tx_gain_min;
	double		tx_gain_max;

	/* Gain element names (space-separated) */
	char		rx_gain_names[256];
	char		tx_gain_names[256];

	/* Upconverter offsets from config (Hz) */
	double		rx_upconverter;
	double		tx_upconverter;

	/* Sample rate ranges */
	double		sample_rate_min;
	double		sample_rate_max;

	/* Flags */
	int		has_rx;
	int		has_tx;
	int		is_split;	/* Separate TX/RX devices */
} sdr_caps_t;

/* Query SDR capabilities from configured device(s)
 * Call after sdr_configure() but before sdr_open()
 * Returns 0 on success, -1 on error */
int sdr_query_caps(sdr_caps_t *caps);

#endif /* SDR_H */