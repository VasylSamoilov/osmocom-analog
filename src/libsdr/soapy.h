#ifndef SOAPY_H
#define SOAPY_H

#include "sdr_config.h"

int soapy_open(size_t channel, const char *_device_args, const char *_stream_args, const char *_tune_args, const char *tx_antenna, const char *rx_antenna, const char *clock_source, double tx_frequency, double rx_frequency, double lo_offset, double rate, double tx_gain, double rx_gain, double bandwidth, int timestamps);
int soapy_start(void);
void soapy_close(void);
int soapy_send(float *buff, int num);
int soapy_receive(float *buff, int max);
int soapy_get_tosend(int buffer_size);

int soapy_set_rx_frequency(double frequency);
int soapy_set_tx_frequency(double frequency);

/* Query supported sample rates and IF bandwidths from device (single open) */
int soapy_query_device_info(const char *device_args, int direction, size_t channel, sdr_rate_info_t *rates, sdr_rate_info_t *bandwidths);

/* Select optimal IF bandwidth from supported values */
int soapy_select_bandwidth(double min_required, const sdr_rate_info_t *info, double *out_bw);

#endif /* SOAPY_H */
