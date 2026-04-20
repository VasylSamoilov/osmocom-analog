#ifndef SDR_H
#define SDR_H

enum paging_signal;

int sdr_start(void *inst);
void *sdr_open(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index);
void sdr_close(void *inst);
int sdr_write(void *inst, sample_t **samples, uint8_t **power, int num, enum paging_signal *paging_signal, int *on, int channels);
int sdr_read(void *inst, sample_t **samples, int num, int channels, double *rf_level_db);
int sdr_get_tosend(void *inst, int buffer_size);
void calibrate_bias(void);
void *sdr_open_channelizer(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index, int use_channelizer, int fast_math);
int sdr_get_samplerate(void *inst);
int sdr_set_rx_frequency(double frequency);
int sdr_set_tx_frequency(double frequency);
int sdr_calculate_optimal_rate(int master_rate, double bandwidth);

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