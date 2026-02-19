/* SoapySDR device access
 *
 * (C) 2017 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* how time stamp process works:
 *
 * TX and RX time stamps are not valid in the beginning.
 *
 * If a first chunk is received from SDR, RX time becomes valid. The duration
 * of the received chunk is added to the RX time stamp, so it becomes the time
 * of the next expected chunk.
 *
 * If a RX time stamp is valid and first chunk is to be transmitted (tosend()
 * is called), TX time stamp becomes valid and is set to RX time stamp, but
 * advanced by the duration of the buffer size. tosend() always returns
 * the number of samples that are needed, to make TX time stamp advance RX time
 * stamp by given buffer size.
 *
 * If chunk is transmitted to SDR, the TX time stamp is advanced by the
 * duration of the transmitted chunk.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Errors.h>
#include "soapy.h"
#include "sdr_config.h"
#include "sdr_status.h"
#include "../liblogging/logging.h"
#include "../liboptions/options.h"

extern int sdr_rx_overflow;

/* Per-device instance holding all state that was previously module-level static */
typedef struct soapy_instance {
	SoapySDRDevice		*sdr;
	SoapySDRStream		*txStream;
	SoapySDRStream		*rxStream;
	double			samplerate;
	long long		Ns_per_sample;
	long long		rx_timeNs;
	long long		tx_timeNs;
	int			use_time_stamps;
	int			rx_valid;
	int			tx_valid;
	int			software_clock;
	struct timespec		software_base_ts;
	int			tx_samps_per_buff;
	int			rx_samps_per_buff;
	pthread_mutex_t		timestamp_mutex;
} soapy_instance_t;

/* In single-device mode, both point to the same instance.
 * In split mode (future), they point to separate instances. */
static soapy_instance_t *soapy_tx_inst = NULL;
static soapy_instance_t *soapy_rx_inst = NULL;

static long long get_software_time_ns(soapy_instance_t *inst)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (ts.tv_sec * 1000000000LL + ts.tv_nsec) -
	       (inst->software_base_ts.tv_sec * 1000000000LL + inst->software_base_ts.tv_nsec);
}

static void init_software_clock(soapy_instance_t *inst)
{
	clock_gettime(CLOCK_MONOTONIC_RAW, &inst->software_base_ts);
	inst->rx_timeNs = 0;
	inst->rx_valid = 1;
	inst->software_clock = 1;
	LOGP(DSOAPY, LOGL_NOTICE, "Using software clock (may drift relative to SDR sample clock)\n");
}

static int parse_args(SoapySDRKwargs *args, const char *_args_string)
{
	char *args_string = options_strdup(_args_string), *key, *val;

	memset(args, 0, sizeof(*args));
	while (args_string && *args_string) {
		key = args_string;
		val = strchr(key, '=');
		if (!val) {
			LOGP(DSOAPY, LOGL_ERROR, "Error parsing SDR args: No '=' after key\n");
			soapy_close();
			return -EIO;
		}
		*val++ = '\0';
		args_string = strchr(val, ',');
		if (args_string)
			*args_string++ = '\0';
		LOGP(DSOAPY, LOGL_DEBUG, "SDR device args: key='%s' value='%s'\n", key, val);
		SoapySDRKwargs_set(args, key, val);
	}

	return 0;
}

/* Helper to assign instance for cleanup on error during soapy_open.
 * In split mode, only assign to the appropriate pointer based on what's being opened. */
static void assign_instance_for_cleanup(soapy_instance_t *inst, double tx_frequency, double rx_frequency)
{
	if (tx_frequency && !rx_frequency) {
		/* TX-only: assign to TX instance */
		soapy_tx_inst = inst;
	} else if (rx_frequency && !tx_frequency) {
		/* RX-only: assign to RX instance */
		soapy_rx_inst = inst;
	} else {
		/* Both or neither: assign to both */
		soapy_tx_inst = soapy_rx_inst = inst;
	}
}

int soapy_open(size_t channel, const char *_device_args, const char *_stream_args, const char *_tune_args, const char *tx_antenna, const char *rx_antenna, const char *clock_source, double tx_frequency, double rx_frequency, double lo_offset, double rate, double tx_gain, double rx_gain, double bandwidth, int timestamps)
{
	double got_frequency, got_rate, got_gain, got_bandwidth;
	const char *got_antenna, *got_clock;
	size_t num_channels;
	SoapySDRKwargs device_args;
	SoapySDRKwargs stream_args;
	SoapySDRKwargs tune_args;
	int rc;
	soapy_instance_t *inst;

	/* Allocate a new instance and zero-initialize */
	inst = calloc(1, sizeof(soapy_instance_t));
	if (!inst) {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to allocate SoapySDR instance\n");
		return -ENOMEM;
	}

	inst->use_time_stamps = timestamps;
	if (inst->use_time_stamps && (1000000000LL % (long long)rate)) {
		LOGP(DSOAPY, LOGL_ERROR, "The given sample duration is not a multiple of a nano second. I.e. we can't divide 10^9 by sample rate of %.0f. Please choose a different sample rate for time stamp support!\n", rate);
		inst->use_time_stamps = 0;
	}
	inst->Ns_per_sample = 1000000000LL / (long long)rate;
	inst->samplerate = rate;

	/* parsing ARGS */
	LOGP(DSOAPY, LOGL_INFO, "Using device args \"%s\"\n", _device_args);
	rc = parse_args(&device_args, _device_args);
	if (rc < 0) {
		free(inst);
		return rc;
	}
	LOGP(DSOAPY, LOGL_INFO, "Using stream args \"%s\"\n", _stream_args);
	rc = parse_args(&stream_args, _stream_args);
	if (rc < 0) {
		free(inst);
		return rc;
	}
	LOGP(DSOAPY, LOGL_INFO, "Using tune args \"%s\"\n", _tune_args);
	rc = parse_args(&tune_args, _tune_args);
	if (rc < 0) {
		free(inst);
		return rc;
	}

	if (lo_offset) {
		char val[32];
		snprintf(val, sizeof(val), "%.0f", lo_offset);
		val[sizeof(val) - 1] = '\0';
		SoapySDRKwargs_set(&tune_args, "OFFSET", val);
	}

	/* create SoapySDR device */
	inst->sdr = SoapySDRDevice_make(&device_args);
	if (!inst->sdr) {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to create SoapySDR\n");
		assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
		soapy_close();
		return -EIO;
	}

	/* clock source */
	if (clock_source && clock_source[0]) {
		if (!strcasecmp(clock_source, "list")) {
			char **clocks;
			size_t clocks_length;
			int i;
			clocks = SoapySDRDevice_listClockSources(inst->sdr, &clocks_length);
			if (!clocks) {
				LOGP(DSOAPY, LOGL_ERROR, "Failed to request list of clock sources!\n");
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return -EIO;
			}
			if (clocks_length) {
				for (i = 0; i < (int)clocks_length; i++)
					LOGP(DSOAPY, LOGL_NOTICE, "Clock source: '%s'\n", clocks[i]);
				got_clock = SoapySDRDevice_getClockSource(inst->sdr);
				LOGP(DSOAPY, LOGL_NOTICE, "Default clock source: '%s'\n", got_clock);
			} else
				LOGP(DSOAPY, LOGL_NOTICE, "There are no clock sources configurable for this device.\n");
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return 1;
		}

		if (SoapySDRDevice_setClockSource(inst->sdr, clock_source) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set clock source to '%s'\n", clock_source);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}
		got_clock = SoapySDRDevice_getClockSource(inst->sdr);
		if (!!strcasecmp(clock_source, got_clock)) {
			LOGP(DSOAPY, LOGL_NOTICE, "Given clock source '%s' was accepted, but driver claims to use '%s'\n", clock_source, got_clock);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EINVAL;
		}
	}

	if (rx_frequency) {
		/* get number of channels and check if requested channel is in range */
		num_channels = SoapySDRDevice_getNumChannels(inst->sdr, SOAPY_SDR_RX);
		LOGP(DSOAPY, LOGL_DEBUG, "We have %d RX channel, selecting channel #%d\n", (int)num_channels, (int)channel);
		if (channel >= num_channels) {
			LOGP(DSOAPY, LOGL_ERROR, "Requested channel #%d (capable of RX) does not exist. Please select channel %d..%d!\n", (int)channel, 0, (int)num_channels - 1);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* antenna */
		if (rx_antenna && rx_antenna[0]) {
			if (!strcasecmp(rx_antenna, "list")) {
				char **antennas;
				size_t antennas_length;
				int i;
				antennas = SoapySDRDevice_listAntennas(inst->sdr, SOAPY_SDR_RX, channel, &antennas_length);
				if (!antennas) {
					LOGP(DSOAPY, LOGL_ERROR, "Failed to request list of RX antennas!\n");
					assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
					soapy_close();
					return -EIO;
				}
				for (i = 0; i < (int)antennas_length; i++)
					LOGP(DSOAPY, LOGL_NOTICE, "RX Antenna: '%s'\n", antennas[i]);
				got_antenna = SoapySDRDevice_getAntenna(inst->sdr, SOAPY_SDR_RX, channel);
				LOGP(DSOAPY, LOGL_NOTICE, "Default RX Antenna: '%s'\n", got_antenna);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return 1;
			}

			if (SoapySDRDevice_setAntenna(inst->sdr, SOAPY_SDR_RX, channel, rx_antenna) != 0) {
				LOGP(DSOAPY, LOGL_ERROR, "Failed to set RX antenna to '%s'\n", rx_antenna);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return -EIO;
			}
			got_antenna = SoapySDRDevice_getAntenna(inst->sdr, SOAPY_SDR_RX, channel);
			if (!!strcasecmp(rx_antenna, got_antenna)) {
				LOGP(DSOAPY, LOGL_NOTICE, "Given RX antenna '%s' was accepted, but driver claims to use '%s'\n", rx_antenna, got_antenna);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return -EINVAL;
			}
		}

		/* set rate */
		if (SoapySDRDevice_setSampleRate(inst->sdr, SOAPY_SDR_RX, channel, rate) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set RX rate to %.0f Hz\n", rate);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* see what rate actually is */
		got_rate = SoapySDRDevice_getSampleRate(inst->sdr, SOAPY_SDR_RX, channel);
		if (fabs(got_rate - rate) > 1.0) {
			LOGP(DSOAPY, LOGL_ERROR, "Given RX rate %.3f Hz is not supported, try %.3f Hz\n", rate, got_rate);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EINVAL;
		}

		if (rx_gain) {
			/* set gain */
			if (SoapySDRDevice_setGain(inst->sdr, SOAPY_SDR_RX, channel, rx_gain) != 0) {
				LOGP(DSOAPY, LOGL_ERROR, "Failed to set RX gain to %.0f\n", rx_gain);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return -EIO;
			}

			/* see what gain actually is */
			got_gain = SoapySDRDevice_getGain(inst->sdr, SOAPY_SDR_RX, channel);
			if (fabs(got_gain - rx_gain) > 0.001) {
				LOGP(DSOAPY, LOGL_NOTICE, "Given RX gain %.3f is not supported, we use %.3f\n", rx_gain, got_gain);
				rx_gain = got_gain;
			}
		}

		/* hack to make limesdr tune rx to tx */
		if (tx_frequency == rx_frequency)
			rx_frequency += 1.0;

		/* set frequency */
		if (SoapySDRDevice_setFrequency(inst->sdr, SOAPY_SDR_RX, channel, rx_frequency, &tune_args) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set RX frequency to %.0f Hz\n", rx_frequency);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* see what frequency actually is */
		got_frequency = SoapySDRDevice_getFrequency(inst->sdr, SOAPY_SDR_RX, channel);
		if (fabs(got_frequency - rx_frequency) > 100.0) {
			LOGP(DSOAPY, LOGL_ERROR, "Given RX frequency %.0f Hz is not supported, try %.0f Hz\n", rx_frequency, got_frequency);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EINVAL;
		}

		/* set bandwidth */
		if (bandwidth > 0.0) {
		if (SoapySDRDevice_setBandwidth(inst->sdr, SOAPY_SDR_RX, channel, bandwidth) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set RX bandwidth to %.0f Hz\n", bandwidth);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* see what bandwidth actually is */
		got_bandwidth = SoapySDRDevice_getBandwidth(inst->sdr, SOAPY_SDR_RX, channel);
		if (fabs(got_bandwidth - bandwidth) > 100.0) {
			LOGP(DSOAPY, LOGL_NOTICE, "Given RX bandwidth %.0f Hz is not supported by device (got %.0f Hz), continuing anyway\n", bandwidth, got_bandwidth);
		}
		}

		/* set up streamer */
#ifdef SOAPY_0_8_0_OR_HIGHER
		if (!(inst->rxStream = SoapySDRDevice_setupStream(inst->sdr, SOAPY_SDR_RX, SOAPY_SDR_CF32, &channel, 1, &stream_args)))
#else
		if (SoapySDRDevice_setupStream(inst->sdr, &inst->rxStream, SOAPY_SDR_RX, SOAPY_SDR_CF32, &channel, 1, &stream_args) != 0)
#endif
		{
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set RX streamer args\n");
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* get buffer sizes */
		inst->rx_samps_per_buff = SoapySDRDevice_getStreamMTU(inst->sdr, inst->rxStream);
		if (inst->rx_samps_per_buff == 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to get RX streamer sample buffer\n");
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}
	}

	if (tx_frequency) {
		/* get number of channels and check if requested channel is in range */
		num_channels = SoapySDRDevice_getNumChannels(inst->sdr, SOAPY_SDR_TX);
		LOGP(DSOAPY, LOGL_DEBUG, "We have %d TX channel, selecting channel #%d\n", (int)num_channels, (int)channel);
		if (channel >= num_channels) {
			LOGP(DSOAPY, LOGL_ERROR, "Requested channel #%d (capable of TX) does not exist. Please select channel %d..%d!\n", (int)channel, 0, (int)num_channels - 1);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* antenna */
		if (tx_antenna && tx_antenna[0]) {
			if (!strcasecmp(tx_antenna, "list")) {
				char **antennas;
				size_t antennas_length;
				int i;
				antennas = SoapySDRDevice_listAntennas(inst->sdr, SOAPY_SDR_TX, channel, &antennas_length);
				if (!antennas) {
					LOGP(DSOAPY, LOGL_ERROR, "Failed to request list of TX antennas!\n");
					assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
					soapy_close();
					return -EIO;
				}
				for (i = 0; i < (int)antennas_length; i++)
					LOGP(DSOAPY, LOGL_NOTICE, "TX Antenna: '%s'\n", antennas[i]);
				got_antenna = SoapySDRDevice_getAntenna(inst->sdr, SOAPY_SDR_TX, channel);
				LOGP(DSOAPY, LOGL_NOTICE, "Default TX Antenna: '%s'\n", got_antenna);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return 1;
			}

			if (SoapySDRDevice_setAntenna(inst->sdr, SOAPY_SDR_TX, channel, tx_antenna) != 0) {
				LOGP(DSOAPY, LOGL_ERROR, "Failed to set TX antenna to '%s'\n", tx_antenna);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return -EIO;
			}
			got_antenna = SoapySDRDevice_getAntenna(inst->sdr, SOAPY_SDR_TX, channel);
			if (!!strcasecmp(tx_antenna, got_antenna)) {
				LOGP(DSOAPY, LOGL_NOTICE, "Given TX antenna '%s' was accepted, but driver claims to use '%s'\n", tx_antenna, got_antenna);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return -EINVAL;
			}
		}

		/* set rate */
		if (SoapySDRDevice_setSampleRate(inst->sdr, SOAPY_SDR_TX, channel, rate) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set TX rate to %.0f Hz\n", rate);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* see what rate actually is */
		got_rate = SoapySDRDevice_getSampleRate(inst->sdr, SOAPY_SDR_TX, channel);
		if (fabs(got_rate - rate) > 1.0) {
			LOGP(DSOAPY, LOGL_ERROR, "Given TX rate %.3f Hz is not supported, try %.3f Hz\n", rate, got_rate);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EINVAL;
		}

		if (tx_gain) {
			/* set gain */
			if (SoapySDRDevice_setGain(inst->sdr, SOAPY_SDR_TX, channel, tx_gain) != 0) {
				LOGP(DSOAPY, LOGL_ERROR, "Failed to set TX gain to %.0f\n", tx_gain);
				assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
				soapy_close();
				return -EIO;
			}

			/* see what gain actually is */
			got_gain = SoapySDRDevice_getGain(inst->sdr, SOAPY_SDR_TX, channel);
			if (fabs(got_gain - tx_gain) > 0.001) {
				LOGP(DSOAPY, LOGL_NOTICE, "Given TX gain %.3f is not supported, we use %.3f\n", tx_gain, got_gain);
				tx_gain = got_gain;
			}
		}

		/* set frequency */
		if (SoapySDRDevice_setFrequency(inst->sdr, SOAPY_SDR_TX, channel, tx_frequency, &tune_args) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set TX frequency to %.0f Hz\n", tx_frequency);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* see what frequency actually is */
		got_frequency = SoapySDRDevice_getFrequency(inst->sdr, SOAPY_SDR_TX, channel);
		if (fabs(got_frequency - tx_frequency) > 100.0) {
			LOGP(DSOAPY, LOGL_ERROR, "Given TX frequency %.0f Hz is not supported, try %.0f Hz\n", tx_frequency, got_frequency);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EINVAL;
		}

		/* set bandwidth */
		if (bandwidth > 0.0) {
		if (SoapySDRDevice_setBandwidth(inst->sdr, SOAPY_SDR_TX, channel, bandwidth) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set TX bandwidth to %.0f Hz\n", bandwidth);
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* see what bandwidth actually is */
		got_bandwidth = SoapySDRDevice_getBandwidth(inst->sdr, SOAPY_SDR_TX, channel);
		if (fabs(got_bandwidth - bandwidth) > 100.0) {
			LOGP(DSOAPY, LOGL_NOTICE, "Given TX bandwidth %.0f Hz is not supported by device (got %.0f Hz), continuing anyway\n", bandwidth, got_bandwidth);
		}
		}

		/* set up streamer */
#ifdef SOAPY_0_8_0_OR_HIGHER
		if (!(inst->txStream = SoapySDRDevice_setupStream(inst->sdr, SOAPY_SDR_TX, SOAPY_SDR_CF32, &channel, 1, &stream_args)))
#else
		if (SoapySDRDevice_setupStream(inst->sdr, &inst->txStream, SOAPY_SDR_TX, SOAPY_SDR_CF32, &channel, 1, &stream_args) != 0)
#endif
		{
			LOGP(DSOAPY, LOGL_ERROR, "Failed to set TX streamer args\n");
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}

		/* get buffer sizes */
		inst->tx_samps_per_buff = SoapySDRDevice_getStreamMTU(inst->sdr, inst->txStream);
		if (inst->tx_samps_per_buff == 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to get TX streamer sample buffer\n");
			assign_instance_for_cleanup(inst, tx_frequency, rx_frequency);
			soapy_close();
			return -EIO;
		}
	}

	/* create mutex for time stamp protection */
	rc = pthread_mutex_init(&inst->timestamp_mutex, NULL);
	if (rc < 0) {
		LOGP(DSOAPY, LOGL_ERROR, "Mutex init failed!\n");
		free(inst);
		return rc;
	}

	/* Assign instance based on what was configured (TX, RX, or both) */
	if (tx_frequency && !rx_frequency) {
		/* TX-only call (split mode TX device) */
		soapy_tx_inst = inst;
		LOGP(DSOAPY, LOGL_DEBUG, "Assigned TX instance (split mode)\n");
	} else if (rx_frequency && !tx_frequency) {
		/* RX-only call (split mode RX device) */
		soapy_rx_inst = inst;
		LOGP(DSOAPY, LOGL_DEBUG, "Assigned RX instance (split mode)\n");
	} else {
		/* Single-device mode: both TX and RX point to the same instance */
		soapy_tx_inst = soapy_rx_inst = inst;
	}

	return 0;
}

/* start streaming */
int soapy_start(void)
{
	/* enable rx stream if configured */
	if (soapy_rx_inst && soapy_rx_inst->rxStream) {
		if (SoapySDRDevice_activateStream(soapy_rx_inst->sdr, soapy_rx_inst->rxStream, 0, 0, 0) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to issue RX stream command\n");
			return -EIO;
		}
	}

	/* enable tx stream if configured */
	if (soapy_tx_inst && soapy_tx_inst->txStream) {
		if (SoapySDRDevice_activateStream(soapy_tx_inst->sdr, soapy_tx_inst->txStream, 0, 0, 0) != 0) {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to issue TX stream command\n");
			return -EIO;
		}
	}

	/* TX-only mode: no RX stream to provide clock, use software clock */
	if (soapy_tx_inst && soapy_tx_inst->txStream && !(soapy_rx_inst && soapy_rx_inst->rxStream)) {
		init_software_clock(soapy_tx_inst);
	}

	/* Split mode: TX and RX on different devices, initialize software clock on TX instance */
	if (sdr_config && sdr_config->split_mode && soapy_tx_inst && soapy_tx_inst->txStream) {
		init_software_clock(soapy_tx_inst);
	}

	return 0;
}

void soapy_close(void)
{
	LOGP(DSOAPY, LOGL_DEBUG, "Clean up SoapySDR\n");
	if (soapy_tx_inst) {
		if (soapy_tx_inst->txStream) {
			SoapySDRDevice_deactivateStream(soapy_tx_inst->sdr, soapy_tx_inst->txStream, 0, 0);
			SoapySDRDevice_closeStream(soapy_tx_inst->sdr, soapy_tx_inst->txStream);
			soapy_tx_inst->txStream = NULL;
		}
		if (soapy_tx_inst == soapy_rx_inst) {
			/* single-device mode: also free RX resources from same instance */
			if (soapy_tx_inst->rxStream) {
				SoapySDRDevice_deactivateStream(soapy_tx_inst->sdr, soapy_tx_inst->rxStream, 0, 0);
				SoapySDRDevice_closeStream(soapy_tx_inst->sdr, soapy_tx_inst->rxStream);
				soapy_tx_inst->rxStream = NULL;
			}
		}
		if (soapy_tx_inst->sdr) {
			SoapySDRDevice_unmake(soapy_tx_inst->sdr);
			soapy_tx_inst->sdr = NULL;
			pthread_mutex_destroy(&soapy_tx_inst->timestamp_mutex);
		}
		free(soapy_tx_inst);
	}
	if (soapy_rx_inst && soapy_rx_inst != soapy_tx_inst) {
		/* split mode: free RX instance separately */
		if (soapy_rx_inst->rxStream) {
			SoapySDRDevice_deactivateStream(soapy_rx_inst->sdr, soapy_rx_inst->rxStream, 0, 0);
			SoapySDRDevice_closeStream(soapy_rx_inst->sdr, soapy_rx_inst->rxStream);
			soapy_rx_inst->rxStream = NULL;
		}
		if (soapy_rx_inst->sdr) {
			SoapySDRDevice_unmake(soapy_rx_inst->sdr);
			soapy_rx_inst->sdr = NULL;
			pthread_mutex_destroy(&soapy_rx_inst->timestamp_mutex);
		}
		free(soapy_rx_inst);
	}
	soapy_tx_inst = soapy_rx_inst = NULL;
}
int soapy_set_rx_frequency(double frequency)
{
	soapy_instance_t *inst = soapy_rx_inst;
	SoapySDRKwargs tune_args;
	double got_frequency;

	if (!inst || !inst->sdr) {
		LOGP(DSOAPY, LOGL_ERROR, "Cannot set frequency: no device open\n");
		return -ENODEV;
	}

	memset(&tune_args, 0, sizeof(tune_args));

	if (SoapySDRDevice_setFrequency(inst->sdr, SOAPY_SDR_RX, 0, frequency, &tune_args) != 0) {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to set RX frequency to %.0f Hz\n", frequency);
		return -EIO;
	}

	got_frequency = SoapySDRDevice_getFrequency(inst->sdr, SOAPY_SDR_RX, 0);
	if (fabs(got_frequency - frequency) > 100.0) {
		LOGP(DSOAPY, LOGL_ERROR, "Given RX frequency %.0f Hz is not supported, got %.0f Hz\n", frequency, got_frequency);
		return -EINVAL;
	}

	LOGP(DSOAPY, LOGL_INFO, "RX frequency set to %.0f Hz\n", got_frequency);
	return 0;
}

int soapy_set_tx_frequency(double frequency)
{
	soapy_instance_t *inst = soapy_tx_inst;
	SoapySDRKwargs tune_args;
	double got_frequency;

	if (!inst || !inst->sdr) {
		LOGP(DSOAPY, LOGL_ERROR, "Cannot set TX frequency: no device open\n");
		return -ENODEV;
	}

	memset(&tune_args, 0, sizeof(tune_args));

	if (SoapySDRDevice_setFrequency(inst->sdr, SOAPY_SDR_TX, 0, frequency, &tune_args) != 0) {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to set TX frequency to %.0f Hz\n", frequency);
		return -EIO;
	}

	got_frequency = SoapySDRDevice_getFrequency(inst->sdr, SOAPY_SDR_TX, 0);
	if (fabs(got_frequency - frequency) > 100.0) {
		LOGP(DSOAPY, LOGL_ERROR, "Given TX frequency %.0f Hz is not supported, got %.0f Hz\n", frequency, got_frequency);
		return -EINVAL;
	}

	LOGP(DSOAPY, LOGL_INFO, "TX frequency set to %.0f Hz\n", got_frequency);
	return 0;
}


int soapy_send(float *buff, int num)
{
    	const void *buffs_ptr[1];
	int chunk;
	int sent = 0, count;
	int flags = 0;
	int split_mode = sdr_config && sdr_config->split_mode;
	static int send_logged = 0;

	if (!soapy_tx_inst || !soapy_tx_inst->txStream)
		return 0;

	if (!send_logged) {
		LOGP(DSOAPY, LOGL_NOTICE, "SOAPY_SEND: num=%d samplerate=%.0f Ns_per_sample=%lld\n",
		     num, soapy_tx_inst->samplerate, soapy_tx_inst->Ns_per_sample);
		send_logged = 1;
	}

	while (num) {
		chunk = num;
		if (chunk > soapy_tx_inst->tx_samps_per_buff)
			chunk = soapy_tx_inst->tx_samps_per_buff;

		/* TX output level diagnostics via status objects */
		sdr_status_hw_tx_accumulate(buff, chunk, soapy_tx_inst->samplerate);

		/* write TX stream */
		buffs_ptr[0] = buff;
		/* In split mode, don't use hardware timestamps - we use software clock */
		if (soapy_tx_inst->use_time_stamps && !split_mode)
			flags |= SOAPY_SDR_HAS_TIME;
		count = SoapySDRDevice_writeStream(soapy_tx_inst->sdr, soapy_tx_inst->txStream, buffs_ptr, chunk, &flags, soapy_tx_inst->tx_timeNs, 1000000);
		if (count <= 0) {
			LOGP(DUHD, LOGL_ERROR, "Failed to write to TX streamer (error=%d)\n", count);
			break;
		}
		/* Update TX time for samples actually sent */
		if (!soapy_tx_inst->tx_valid)
			LOGP(DSOAPY, LOGL_ERROR, "SDR TX: tosend() was not called before, please fix!\n");
		else {
			pthread_mutex_lock(&soapy_tx_inst->timestamp_mutex);
			soapy_tx_inst->tx_timeNs += count * soapy_tx_inst->Ns_per_sample;
			pthread_mutex_unlock(&soapy_tx_inst->timestamp_mutex);
		}
		/* increment transmit counters */
		sent += count;
		buff += count * 2;
		num -= count;
	}

	return sent;
}

/* read what we got, return 0, if buffer is empty, otherwise return the number of samples */
int soapy_receive(float *buff, int max)
{
    	void *buffs_ptr[1];
	int count;
	long long timeNs;
	int flags = 0;
	int num_to_read;
	int split_mode = sdr_config && sdr_config->split_mode;

	if (!soapy_rx_inst || !soapy_rx_inst->rxStream)
		return 0;

	if (max <= 0)
		return 0;
	/* read up to MTU, but no more than available buffer space */
	num_to_read = (max < soapy_rx_inst->rx_samps_per_buff) ? max : soapy_rx_inst->rx_samps_per_buff;
	/* read RX stream */
	buffs_ptr[0] = buff;
	count = SoapySDRDevice_readStream(soapy_rx_inst->sdr, soapy_rx_inst->rxStream, buffs_ptr, num_to_read, &flags, &timeNs, 0);
	if (count == SOAPY_SDR_OVERFLOW) {
		sdr_rx_overflow = 1;
		return 0;
	}
	if (count > 0) {
		static int rx_debug_count = 0;
		
		/* Check if hardware timestamps are available */
		int has_hw_time = (flags & SOAPY_SDR_HAS_TIME) != 0;
		
		/* In split mode, use software clock for overflow detection.
		 * Hardware timestamps from different SDR devices are unreliable
		 * (RTL-SDR reports buffer position, not samples read).
		 * Instead, compare wall-clock time vs expected processing time. */
		if (split_mode) {
			/* First read: initialize software clock for RX */
			if (!soapy_rx_inst->rx_valid) {
				init_software_clock(soapy_rx_inst);
				LOGP(DSOAPY, LOGL_NOTICE, "RX INIT (split mode): count=%d Ns_per_sample=%lld samplerate=%.0f\n",
				     count, soapy_rx_inst->Ns_per_sample, soapy_rx_inst->samplerate);
			}
			
			/* Get wall-clock time and compare with expected processing time.
			 * rx_timeNs tracks how much time worth of samples we've processed.
			 * If wall clock is ahead of rx_timeNs, we're falling behind. */
			long long wall_time_ns = get_software_time_ns(soapy_rx_inst);
			long long samples_time_ns = soapy_rx_inst->rx_timeNs + count * soapy_rx_inst->Ns_per_sample;
			
			/* Allow some tolerance (e.g., 100ms) for jitter and buffering */
			long long tolerance_ns = 100000000LL; /* 100ms */
			if (wall_time_ns > samples_time_ns + tolerance_ns) {
				if (rx_debug_count < 20) {
					LOGP(DSOAPY, LOGL_ERROR, "SDR RX overflow (split mode): wall=%lldms processed=%lldms behind=%lldms\n",
					     wall_time_ns / 1000000LL, samples_time_ns / 1000000LL,
					     (wall_time_ns - samples_time_ns) / 1000000LL);
					rx_debug_count++;
				}
				sdr_rx_overflow = 1;
			}
			
			soapy_rx_inst->rx_timeNs = samples_time_ns;

			/* Raw RX sample diagnostics via status objects */
			sdr_status_hw_rx_accumulate(buff, count);

			return count;
		}
		
		if (!soapy_rx_inst->use_time_stamps || !has_hw_time) {
			/* No hardware timestamps - use software clock */
			if (soapy_rx_inst->use_time_stamps && !soapy_rx_inst->software_clock) {
				LOGP(DSOAPY, LOGL_NOTICE, "SDR RX: No hardware timestamps available, falling back to software clock.\n");
				init_software_clock(soapy_rx_inst);
				soapy_rx_inst->use_time_stamps = 0;
			}
			/* Get time from software clock */
			if (soapy_rx_inst->software_clock)
				timeNs = get_software_time_ns(soapy_rx_inst);
			else
				timeNs = soapy_rx_inst->rx_timeNs;
		}
		/* else: use hardware timeNs from device */
		
		/* process RX time stamp */
		if (!soapy_rx_inst->rx_valid) {
			soapy_rx_inst->rx_timeNs = timeNs;
			soapy_rx_inst->rx_valid = 1;
			LOGP(DSOAPY, LOGL_NOTICE, "RX INIT: first_timeNs=%lld count=%d Ns_per_sample=%lld samplerate=%.0f\n",
			     timeNs, count, soapy_rx_inst->Ns_per_sample, soapy_rx_inst->samplerate);
		}
		pthread_mutex_lock(&soapy_rx_inst->timestamp_mutex);
		long long expected_advance = count * soapy_rx_inst->Ns_per_sample;
		if (soapy_rx_inst->rx_timeNs != timeNs) {
			if (rx_debug_count < 20) {
				LOGP(DSOAPY, LOGL_ERROR, "SDR RX overflow: expected=%lld actual=%lld diff=%lld count=%d expected_adv=%lld Ns_per_sample=%lld\n",
				     soapy_rx_inst->rx_timeNs, timeNs, timeNs - soapy_rx_inst->rx_timeNs, count, 
				     expected_advance, soapy_rx_inst->Ns_per_sample);
				rx_debug_count++;
			}
			sdr_rx_overflow = 1;
		}
		soapy_rx_inst->rx_timeNs = timeNs + count * soapy_rx_inst->Ns_per_sample;
		pthread_mutex_unlock(&soapy_rx_inst->timestamp_mutex);

		/* Raw RX sample diagnostics via status objects */
		sdr_status_hw_rx_accumulate(buff, count);
	}

	return count;
}

/* estimate number of samples that can be sent */
int soapy_get_tosend(int buffer_size)
{
	int tosend;
	static int debug_count = 0;

	if (!soapy_tx_inst)
		return 0;

	/* Split mode with software clock: independent timing based on wall clock.
	 * 
	 * tx_time tracks how much time we've "sent" - when we send N samples,
	 * tx_time advances by N/Ns_per_sample nanoseconds.
	 * 
	 * The goal is to keep tx_time ahead of now by buffer_duration.
	 * tosend = how many samples needed to reach that goal.
	 */
	if (sdr_config && sdr_config->split_mode && soapy_tx_inst->software_clock) {
		long long now_ns = get_software_time_ns(soapy_tx_inst);
		long long buffer_duration_ns = (long long)buffer_size * soapy_tx_inst->Ns_per_sample;
		long long target_tx_time_ns;
		static int underrun_count = 0;
		
		/* Initialize TX time on first call.
		 * Set tx_timeNs = now, target is now + buffer_duration.
		 * This means we need buffer_size samples to reach target. */
		if (!soapy_tx_inst->tx_valid) {
			soapy_tx_inst->tx_timeNs = now_ns;
			soapy_tx_inst->tx_valid = 1;
			LOGP(DSOAPY, LOGL_INFO, "SPLIT TX init: now_ns=%lld tx_timeNs=%lld target_headroom=%.3fms\n",
			     now_ns, now_ns, (double)buffer_duration_ns / 1000000.0);
			/* Return buffer_size on first call to build up headroom */
			return buffer_size;
		}
		
		/* Target: tx_timeNs should be (now + buffer_duration) ahead.
		 * tosend = (target - tx_time) / Ns_per_sample
		 */
		target_tx_time_ns = now_ns + buffer_duration_ns;
		tosend = (int)((target_tx_time_ns - soapy_tx_inst->tx_timeNs) / soapy_tx_inst->Ns_per_sample);
		
		if (debug_count++ < 50 || (debug_count % 1000) == 0) {
			double ahead_ms = (double)(soapy_tx_inst->tx_timeNs - now_ns) / 1000000.0;
			LOGP(DSOAPY, LOGL_DEBUG, "SPLIT: now=%lld tx_time=%lld ahead=%.1fms tosend=%d\n",
			     now_ns, soapy_tx_inst->tx_timeNs, ahead_ms, tosend);
		}
		
		/* Cap tosend to buffer_size - we can only produce this many per iteration.
		 * If tosend > buffer_size, we're behind but will catch up over time. */
		if (tosend > buffer_size) {
			/* Only log if significantly behind */
			double behind_ms = (double)(target_tx_time_ns - soapy_tx_inst->tx_timeNs - buffer_duration_ns) / 1000000.0;
			if (behind_ms > (double)buffer_duration_ns / 1000000.0) {
				underrun_count++;
				if (underrun_count <= 10 || (underrun_count % 100) == 0) {
					LOGP(DSOAPY, LOGL_NOTICE, "SPLIT TX catching up #%d: %.1f ms behind target\n",
					     underrun_count, behind_ms);
				}
			}
			tosend = buffer_size;
		}
		if (tosend < 0)
			tosend = 0;
		
		return tosend;
	}

	/* Update software clock if in use (TX-only or no HW timestamps) */
	if (soapy_rx_inst && soapy_rx_inst->software_clock) {
		soapy_rx_inst->rx_timeNs = get_software_time_ns(soapy_rx_inst);
	}

	/* if no RX time stamp is set, we must wait until we receive a valid time stamp */
	if (!soapy_rx_inst || !soapy_rx_inst->rx_valid)
		return 0;

	/* RX time stamp is valid the first time, set the TX time stamp in advance */
	if (!soapy_tx_inst->tx_valid) {
		soapy_tx_inst->tx_timeNs = soapy_rx_inst->rx_timeNs + buffer_size * soapy_tx_inst->Ns_per_sample;
		soapy_tx_inst->tx_valid = 1;
		LOGP(DSOAPY, LOGL_DEBUG, "TX init: rx_timeNs=%lld tx_timeNs=%lld Ns_per_sample=%lld buffer_size=%d\n",
		     soapy_rx_inst->rx_timeNs, soapy_tx_inst->tx_timeNs, soapy_tx_inst->Ns_per_sample, buffer_size);
		return 0;
	}

	/* we check how advance our transmitted time stamp is */
	pthread_mutex_lock(&soapy_tx_inst->timestamp_mutex);
	tosend = buffer_size - (soapy_tx_inst->tx_timeNs - soapy_rx_inst->rx_timeNs) / soapy_tx_inst->Ns_per_sample;

	/* in case of underrun, resync TX timestamp */
	if (tosend > buffer_size) {
		LOGP(DSOAPY, LOGL_ERROR, "SDR TX underrun, seems we are too slow. Use lower SDR sample rate.\n");
		if (!soapy_tx_inst->use_time_stamps) {
			/* When TX timestamps are disabled, we must resync to recover.
			 * This causes a slip in the transmit stream. */
			soapy_tx_inst->tx_timeNs = soapy_rx_inst->rx_timeNs;
		}
		/* When TX timestamps are enabled, the driver drops late packets.
		 * The TX timestamp naturally catches up without causing a slip.
		 * We just cap tosend to buffer_size and let recovery happen. */
		tosend = buffer_size;
	}
	pthread_mutex_unlock(&soapy_tx_inst->timestamp_mutex);

	/* race condition and routing errors may cause TX time stamps to be in advance of slightly more than buffer_size */
	if (tosend < 0)
		tosend = 0;

	return tosend;
}

/* Query supported sample rates and IF bandwidths from device in a single open.
 *
 * @param device_args   SoapySDR device arguments string
 * @param direction     SOAPY_SDR_TX (0) or SOAPY_SDR_RX (1)
 * @param channel       Channel number
 * @param rates_out     Output: supported sample rates (caller must free with sdr_rate_info_free)
 * @param bw_out        Output: supported IF bandwidths (caller must free), may be NULL
 * @return 0 on success (at least rates obtained), -1 on failure
 */
int soapy_query_device_info(const char *device_args, int direction, size_t channel, sdr_rate_info_t *rates_out, sdr_rate_info_t *bw_out)
{
	SoapySDRDevice *dev = NULL;
	SoapySDRKwargs args;
	size_t length = 0;
	double *rate_list = NULL;
	double *bw_list = NULL;
	SoapySDRRange *ranges = NULL;
	int got_rates = 0;
	size_t i;

	if (!rates_out)
		return -1;

	memset(rates_out, 0, sizeof(*rates_out));
	if (bw_out)
		memset(bw_out, 0, sizeof(*bw_out));

	LOGP(DSOAPY, LOGL_INFO, "Querying device '%s' (direction=%s, channel=%zu)\n",
	     device_args ? device_args : "(default)",
	     direction == SOAPY_SDR_TX ? "TX" : "RX", channel);

	/* Parse device args */
	memset(&args, 0, sizeof(args));
	if (device_args && device_args[0]) {
		char *args_copy = strdup(device_args);
		char *key, *val, *p = args_copy;
		while (p && *p) {
			key = p;
			val = strchr(key, '=');
			if (!val) break;
			*val++ = '\0';
			p = strchr(val, ',');
			if (p) *p++ = '\0';
			SoapySDRKwargs_set(&args, key, val);
		}
		free(args_copy);
	}

	/* Open device once for all queries */
	dev = SoapySDRDevice_make(&args);
	if (!dev) {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to open SoapySDR device for query\n");
		SoapySDRKwargs_clear(&args);
		return -1;
	}

	/* --- Query IF bandwidths --- */
	if (bw_out) {
		length = 0;
		bw_list = SoapySDRDevice_listBandwidths(dev, direction, channel, &length);
		if (bw_list && length > 0) {
			LOGP(DSOAPY, LOGL_INFO, "Device %s supports %zu discrete bandwidths:\n",
			     direction == SOAPY_SDR_TX ? "TX" : "RX", length);

			bw_out->rates = malloc(length * sizeof(double));
			if (bw_out->rates) {
				memcpy(bw_out->rates, bw_list, length * sizeof(double));
				bw_out->num_rates = length;
				bw_out->is_continuous = 0;
				bw_out->min_rate = bw_out->max_rate = bw_list[0];
				for (i = 0; i < length; i++) {
					LOGP(DSOAPY, LOGL_INFO, "  BW %zu: %.0f Hz (%.3f MHz)\n",
					     i, bw_list[i], bw_list[i] / 1e6);
					if (bw_list[i] < bw_out->min_rate) bw_out->min_rate = bw_list[i];
					if (bw_list[i] > bw_out->max_rate) bw_out->max_rate = bw_list[i];
				}
			}
		} else {
			ranges = SoapySDRDevice_getBandwidthRange(dev, direction, channel, &length);
			if (ranges && length > 0) {
				bw_out->is_continuous = 1;
				bw_out->min_rate = ranges[0].minimum;
				bw_out->max_rate = ranges[0].maximum;
				bw_out->rates = NULL;
				bw_out->num_rates = 0;
				LOGP(DSOAPY, LOGL_INFO, "Device %s supports continuous bandwidth range: %.0f - %.0f Hz\n",
				     direction == SOAPY_SDR_TX ? "TX" : "RX",
				     bw_out->min_rate, bw_out->max_rate);
			}
			ranges = NULL;
		}
	}

	/* --- Query sample rates --- */
	length = 0;
	rate_list = SoapySDRDevice_listSampleRates(dev, direction, channel, &length);
	if (rate_list && length > 0) {
		LOGP(DSOAPY, LOGL_INFO, "Device supports %zu discrete sample rates:\n", length);

		rates_out->rates = malloc(length * sizeof(double));
		if (rates_out->rates) {
			memcpy(rates_out->rates, rate_list, length * sizeof(double));
			rates_out->num_rates = length;
			rates_out->is_continuous = 0;
			rates_out->min_rate = rates_out->max_rate = rate_list[0];
			for (i = 0; i < length; i++) {
				LOGP(DSOAPY, LOGL_INFO, "  Rate %zu: %.0f Hz (%.3f MHz)\n",
				     i, rate_list[i], rate_list[i] / 1e6);
				if (rate_list[i] < rates_out->min_rate) rates_out->min_rate = rate_list[i];
				if (rate_list[i] > rates_out->max_rate) rates_out->max_rate = rate_list[i];
			}
			LOGP(DSOAPY, LOGL_INFO, "Rate range: %.0f - %.0f Hz\n",
			     rates_out->min_rate, rates_out->max_rate);
			got_rates = 1;
		}
	} else {
		ranges = SoapySDRDevice_getSampleRateRange(dev, direction, channel, &length);
		if (ranges && length > 0) {
			rates_out->is_continuous = 1;
			rates_out->min_rate = ranges[0].minimum;
			rates_out->max_rate = ranges[0].maximum;
			rates_out->rates = NULL;
			rates_out->num_rates = 0;
			LOGP(DSOAPY, LOGL_INFO, "Device supports continuous sample rate range:\n");
			for (i = 0; i < length; i++) {
				LOGP(DSOAPY, LOGL_INFO, "  Range %zu: %.0f - %.0f Hz (%.3f - %.3f MHz)\n",
				     i, ranges[i].minimum, ranges[i].maximum,
				     ranges[i].minimum / 1e6, ranges[i].maximum / 1e6);
			}
			got_rates = 1;
		} else {
			LOGP(DSOAPY, LOGL_ERROR, "Failed to get sample rates from device\n");
		}
	}

	SoapySDRDevice_unmake(dev);
	SoapySDRKwargs_clear(&args);

	return got_rates ? 0 : -1;
}

/**
 * Select optimal IF bandwidth from supported values
 *
 * @param min_required  Minimum required bandwidth (typically sample rate)
 * @param info          Supported bandwidth info from device query
 * @param out_bw        Output: selected bandwidth
 * @return 0 on success, -1 on failure
 */
int soapy_select_bandwidth(double min_required, const sdr_rate_info_t *info, double *out_bw)
{
        int i;
        double best = 0;

        if (!info || !out_bw)
                return -1;

        if (info->is_continuous) {
                /* Continuous range: use min_required if within range, else use min */
                if (min_required >= info->min_rate && min_required <= info->max_rate) {
                        *out_bw = min_required;
                } else if (min_required < info->min_rate) {
                        *out_bw = info->min_rate;
                } else {
                        LOGP(DSOAPY, LOGL_ERROR, "Required bandwidth %.0f Hz exceeds device max %.0f Hz\n",
                             min_required, info->max_rate);
                        return -1;
                }
                return 0;
        }

        /* Discrete bandwidths: find smallest >= min_required */
        for (i = 0; i < info->num_rates; i++) {
                if (info->rates[i] >= min_required) {
                        if (best == 0 || info->rates[i] < best)
                                best = info->rates[i];
                }
        }

        if (best == 0) {
                LOGP(DSOAPY, LOGL_ERROR, "No supported bandwidth >= %.0f Hz (max supported: %.0f Hz)\n",
                     min_required, info->max_rate);
                return -1;
        }

        *out_bw = best;
        return 0;
}

/**
 * Query frequency range from SoapySDR device
 *
 * @param device_args   Device arguments string
 * @param direction     SOAPY_SDR_TX or SOAPY_SDR_RX
 * @param channel       Channel number
 * @param min_freq      Output: minimum frequency (Hz)
 * @param max_freq      Output: maximum frequency (Hz)
 * @return 0 on success, -1 on failure
 */
int soapy_query_freq_range(const char *device_args, int direction, size_t channel,
			   double *min_freq, double *max_freq)
{
	SoapySDRDevice *dev = NULL;
	SoapySDRKwargs args;
	SoapySDRRange *ranges = NULL;
	size_t length = 0;
	int rc = -1;

	if (!min_freq || !max_freq)
		return -1;

	*min_freq = 0;
	*max_freq = 0;

	/* Parse device args */
	memset(&args, 0, sizeof(args));
	if (device_args && device_args[0]) {
		char *args_copy = strdup(device_args);
		char *key, *val, *p = args_copy;
		while (p && *p) {
			key = p;
			val = strchr(key, '=');
			if (!val) break;
			*val++ = '\0';
			p = strchr(val, ',');
			if (p) *p++ = '\0';
			SoapySDRKwargs_set(&args, key, val);
		}
		free(args_copy);
	}

	/* Open device */
	dev = SoapySDRDevice_make(&args);
	if (!dev) {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to open SoapySDR device for freq range query\n");
		SoapySDRKwargs_clear(&args);
		return -1;
	}

	/* Get frequency range */
	ranges = SoapySDRDevice_getFrequencyRange(dev, direction, channel, &length);
	if (ranges && length > 0) {
		/* Use first range (usually the main tunable range) */
		*min_freq = ranges[0].minimum;
		*max_freq = ranges[0].maximum;

		LOGP(DSOAPY, LOGL_INFO, "Device %s frequency range: %.0f - %.0f Hz (%.3f - %.3f MHz)\n",
		     direction == SOAPY_SDR_TX ? "TX" : "RX",
		     *min_freq, *max_freq, *min_freq / 1e6, *max_freq / 1e6);

		/* If multiple ranges, find overall min/max */
		for (size_t i = 1; i < length; i++) {
			if (ranges[i].minimum < *min_freq)
				*min_freq = ranges[i].minimum;
			if (ranges[i].maximum > *max_freq)
				*max_freq = ranges[i].maximum;
		}

		rc = 0;
	} else {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to get frequency range from device\n");
	}

	SoapySDRDevice_unmake(dev);
	SoapySDRKwargs_clear(&args);

	return rc;
}

/**
 * Query gain range and available gain elements from SoapySDR device
 *
 * @param device_args   Device arguments string
 * @param direction     SOAPY_SDR_TX or SOAPY_SDR_RX
 * @param channel       Channel number
 * @param min_gain      Output: minimum overall gain (dB)
 * @param max_gain      Output: maximum overall gain (dB)
 * @param gain_names    Output: space-separated list of gain element names
 * @param gain_names_len Size of gain_names buffer
 * @return 0 on success, -1 on failure
 */
int soapy_query_gain_info(const char *device_args, int direction, size_t channel,
			  double *min_gain, double *max_gain,
			  char *gain_names, int gain_names_len)
{
	SoapySDRDevice *dev = NULL;
	SoapySDRKwargs args;
	SoapySDRRange range;
	char **names = NULL;
	size_t num_names = 0;
	int rc = -1;
	int pos = 0;

	if (!min_gain || !max_gain)
		return -1;

	*min_gain = 0;
	*max_gain = 0;
	if (gain_names && gain_names_len > 0)
		gain_names[0] = '\0';

	/* Parse device args */
	memset(&args, 0, sizeof(args));
	if (device_args && device_args[0]) {
		char *args_copy = strdup(device_args);
		char *key, *val, *p = args_copy;
		while (p && *p) {
			key = p;
			val = strchr(key, '=');
			if (!val) break;
			*val++ = '\0';
			p = strchr(val, ',');
			if (p) *p++ = '\0';
			SoapySDRKwargs_set(&args, key, val);
		}
		free(args_copy);
	}

	/* Open device */
	dev = SoapySDRDevice_make(&args);
	if (!dev) {
		LOGP(DSOAPY, LOGL_ERROR, "Failed to open SoapySDR device for gain query\n");
		SoapySDRKwargs_clear(&args);
		return -1;
	}

	/* Get overall gain range */
	range = SoapySDRDevice_getGainRange(dev, direction, channel);
	*min_gain = range.minimum;
	*max_gain = range.maximum;

	LOGP(DSOAPY, LOGL_INFO, "Device %s overall gain range: %.1f - %.1f dB\n",
	     direction == SOAPY_SDR_TX ? "TX" : "RX", *min_gain, *max_gain);

	/* Get list of gain elements */
	names = SoapySDRDevice_listGains(dev, direction, channel, &num_names);
	if (names && num_names > 0 && gain_names && gain_names_len > 0) {
		for (size_t i = 0; i < num_names; i++) {
			SoapySDRRange elem_range = SoapySDRDevice_getGainElementRange(dev, direction, channel, names[i]);
			int len = snprintf(gain_names + pos, gain_names_len - pos,
					   "%s%s", (i > 0) ? " " : "", names[i]);
			if (len > 0 && pos + len < gain_names_len)
				pos += len;

			LOGP(DSOAPY, LOGL_INFO, "  Gain element '%s': %.1f - %.1f dB\n",
			     names[i], elem_range.minimum, elem_range.maximum);
		}
	}

	rc = 0;

	SoapySDRDevice_unmake(dev);
	SoapySDRKwargs_clear(&args);

	return rc;
}
