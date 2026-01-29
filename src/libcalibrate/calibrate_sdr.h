/* SDR interface for calibration
 *
 * Provides simplified SDR access for raw IQ capture during calibration.
 * Uses the existing soapy/uhd infrastructure.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef CALIBRATE_SDR_H
#define CALIBRATE_SDR_H

#include <stddef.h>

/* Calibration SDR state */
typedef struct calibrate_sdr {
	int is_open;
	double center_freq;
	double sample_rate;
	int use_soapy;
	int use_uhd;
} calibrate_sdr_t;

/* Open SDR for calibration
 * Returns 0 on success, <0 on error
 */
int calibrate_sdr_open(calibrate_sdr_t *sdr, const char *device_args,
                        double center_freq, double sample_rate,
                        double rx_gain, double bandwidth);

/* Receive IQ samples
 * Returns number of samples received, <0 on error
 */
int calibrate_sdr_receive(calibrate_sdr_t *sdr, float *iq_buffer, int max_samples);

/* Set center frequency (for tuning during scan)
 * Returns 0 on success, <0 on error
 */
int calibrate_sdr_set_freq(calibrate_sdr_t *sdr, double center_freq);

/* Close SDR
 */
void calibrate_sdr_close(calibrate_sdr_t *sdr);

#endif /* CALIBRATE_SDR_H */
