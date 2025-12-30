/* Split SDR backend - separate TX and RX devices
 *
 * (C) 2024 by osmocom-analog contributors
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "../libsample/sample.h"

enum paging_signal;

void *split_sdr_open(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index);
int split_sdr_start(void *inst);
void split_sdr_close(void *inst);
int split_sdr_write(void *inst, sample_t **samples, uint8_t **power, int num, enum paging_signal *paging_signal, int *on, int channels);
int split_sdr_read(void *inst, sample_t **samples, int num, int channels, double *rf_level_db);
int split_sdr_get_tosend(void *inst, int buffer_size);
