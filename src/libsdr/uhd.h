#ifndef UHD_H
#define UHD_H

#include "sdr_config.h"

int uhd_open(size_t channel, const char *_device_args, const char *_stream_args, const char *_tune_args, const char *tx_antenna, const char *rx_antenna, const char *clock_source, double tx_frequency, double rx_frequency, double lo_offset, double rate, double tx_gain, double rx_gain, double bandwidth, int timestamps);
int uhd_start(void);
void uhd_close(void);
int uhd_send(float *buff, int num);
int uhd_receive(float *buff, int max);
int uhd_get_tosend(int buffer_size);
int uhd_get_tx_mtu(void);

int uhd_set_rx_frequency(double frequency);
int uhd_set_tx_frequency(double frequency);

/* Query supported sample rates from device */
int uhd_query_sample_rates(const char *device_args, int direction, size_t channel, sdr_rate_info_t *info);

/* Query frequency range from device */
int uhd_query_freq_range(const char *device_args, int direction, size_t channel,
			 double *min_freq, double *max_freq);

/* Query gain range from device */
int uhd_query_gain_info(const char *device_args, int direction, size_t channel,
			double *min_gain, double *max_gain,
			char *gain_names, int gain_names_len);

#endif /* UHD_H */
