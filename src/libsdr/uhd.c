/* UHD device access
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/usbdevice_fs.h>
#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include "uhd.h"
#include "../libsample/sample.h"
#include "sdr.h"
#include "sdr_config.h"
#include "../liblogging/logging.h"
#include "../liboptions/options.h"

/* HACK: Set to 1 to trigger a one-time TX underrun after 5 seconds for testing recovery */
#ifndef TEST_UNDERRUN_RECOVERY
#define TEST_UNDERRUN_RECOVERY 0
#endif
/* test: every TX write fails from 5 s to 8 s after the first write (wall
 * time; as a broken USB transfer does: at once, nothing sent) */
#ifndef TEST_TX_FAIL
#define TEST_TX_FAIL 0
#endif

extern int sdr_rx_overflow;

/* Per-device instance holding all state that was previously module-level static */
typedef struct uhd_instance {
	uhd_usrp_handle		usrp;
	uhd_tx_streamer_handle	tx_streamer;
	uhd_rx_streamer_handle	rx_streamer;
	uhd_tx_metadata_handle	tx_metadata;
	uhd_rx_metadata_handle	rx_metadata;
	double			samplerate;
	time_t			rx_time_secs;
	double			rx_time_fract_sec;
	time_t			tx_time_secs;
	double			tx_time_fract_sec;
	int			tx_timestamps;
	int			software_clock;
	struct timespec		software_base_ts;
	double			ppm_tx;		/* PPM correction to apply to TX frequencies */
	double			ppm_rx;		/* PPM correction to apply to RX frequencies */
	uhd_async_metadata_handle tx_async_md;	/* TX async event (underflow, seq error) */
	int			rx_dev_valid;	/* rx_dev_next is set */
	double			rx_dev_next;	/* device time of the next expected RX sample */
	int			rx_dev_warned;	/* overflow without device time logged */
	long long		rx_gap;		/* samples lost before the last packet, not yet filled */
	long long		tx_dropped;	/* TX samples not sent (failed writes), their time passed */
	double			tx_fail_since;	/* monotonic time of the first failed write, 0: writes work */
	double			tx_fail_logged;	/* last 'still failing' line */
	long long		tx_fail_dropped;/* tx_dropped when the failure began */
	unsigned long		tx_fail_writes;	/* failed writes of this failure */
	char			serial[64];	/* the device's serial (a USB reset of this one only) */
	int			tx_dead;	/* writes failed too long: the program stops */
} uhd_instance_t;

/* TX writes failing this long while running: the transport is broken (a
 * USB transfer error stays until the device is reset, also across
 * processes): a hard failure, the program stops with UHD_EXIT_TX_DEAD */
#define UHD_TX_DEAD_S	2.0

static double uhd_mono_time(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* TX write state, logged when it changes (a failed write repeats some
 * hundred times per second while a transport is broken) */
static void uhd_tx_write_failed(uhd_instance_t *inst, uhd_error error, const char *what)
{
	double now = uhd_mono_time();

	inst->tx_fail_writes++;
	if (!inst->tx_fail_since) {
		inst->tx_fail_since = now;
		inst->tx_fail_logged = now;
		inst->tx_fail_dropped = inst->tx_dropped;
		LOGP(DUHD, LOGL_ERROR, "TX writes fail (error=%d): %s - samples are dropped until they work again\n", error, what);
		return;
	}
	if (!inst->tx_dead && now - inst->tx_fail_since >= UHD_TX_DEAD_S) {
		inst->tx_dead = 1;
		LOGP(DUHD, LOGL_ERROR, "TX writes failed for %.0f s (%s): the device is broken, the program stops with exit code %d (a new start checks the device and resets it if TX does not respond)\n",
		     UHD_TX_DEAD_S, what, UHD_EXIT_TX_DEAD);
	}
	if (now - inst->tx_fail_logged >= 5.0) {
		inst->tx_fail_logged = now;
		LOGP(DUHD, LOGL_ERROR, "TX writes still fail after %.0f s (%lu failed, %.1f s of samples dropped): %s\n",
		     now - inst->tx_fail_since, inst->tx_fail_writes,
		     (inst->tx_dropped - inst->tx_fail_dropped) / inst->samplerate, what);
	}
}

static void uhd_tx_write_ok(uhd_instance_t *inst)
{
	if (!inst->tx_fail_since)
		return;
	LOGP(DUHD, LOGL_NOTICE, "TX writes work again after %.1f s (%lu failed, %.3f s of samples dropped)\n",
	     uhd_mono_time() - inst->tx_fail_since, inst->tx_fail_writes,
	     (inst->tx_dropped - inst->tx_fail_dropped) / inst->samplerate);
	inst->tx_fail_since = 0;
	inst->tx_fail_writes = 0;
}

/* In single-device mode, both point to the same instance.
 * In split mode (future), they point to separate instances. */
static uhd_instance_t *uhd_tx_inst = NULL;
static uhd_instance_t *uhd_rx_inst = NULL;

/* TX writes failed too long (the transport is broken): the program should
 * stop */
int uhd_tx_dead(void)
{
	return uhd_tx_inst && uhd_tx_inst->tx_dead;
}

/* a lock sensor of the motherboard: 1 locked, 0 not, -1 no such sensor (a missing sensor is not
 * "unlocked": a B210 without GPSDO has no gps_locked) */
static int clock_sensor(uhd_usrp_handle usrp, uhd_string_vector_handle names, size_t n, const char *name)
{
	uhd_sensor_value_handle value;
	char got[64];
	bool locked = false;
	size_t i;
	int rc = -1;

	for (i = 0; i < n; i++) {
		if (uhd_string_vector_at(names, i, got, sizeof(got)) == UHD_ERROR_NONE && !strcmp(got, name))
			break;
	}
	if (i == n)
		return -1;
	if (uhd_sensor_value_make(&value) != UHD_ERROR_NONE)
		return -1;
	if (uhd_usrp_get_mboard_sensor(usrp, name, 0, &value) == UHD_ERROR_NONE
	 && uhd_sensor_value_to_bool(value, &locked) == UHD_ERROR_NONE)
		rc = locked ? 1 : 0;
	uhd_sensor_value_free(&value);
	return rc;
}

/* Read the TX device's clock into *st and log what changed (UHD category, only on change). From
 * the main loop only: these are control-path calls that can block for milliseconds. */
void uhd_clock_poll(sdr_clock_status_t *st)
{
	uhd_instance_t *inst = uhd_tx_inst;
	uhd_string_vector_handle names;
	sdr_clock_status_t now;
	struct timeval tv;
	int64_t secs;
	double frac;
	size_t n = 0;

	if (!inst || !inst->usrp)
		return;
	memset(&now, 0, sizeof(now));
	now.valid = 1;
	now.ref_locked = now.gps_locked = -1;
	if (uhd_usrp_get_clock_source(inst->usrp, 0, now.clock_source, sizeof(now.clock_source)) != UHD_ERROR_NONE)
		strcpy(now.clock_source, "?");
	if (uhd_usrp_get_time_source(inst->usrp, 0, now.time_source, sizeof(now.time_source)) != UHD_ERROR_NONE)
		strcpy(now.time_source, "?");
	if (uhd_string_vector_make(&names) == UHD_ERROR_NONE) {
		if (uhd_usrp_get_mboard_sensor_names(inst->usrp, 0, &names) == UHD_ERROR_NONE)
			uhd_string_vector_size(names, &n);
		now.ref_locked = clock_sensor(inst->usrp, names, n, "ref_locked");
		now.gps_locked = clock_sensor(inst->usrp, names, n, "gps_locked");
		uhd_string_vector_free(&names);
	}
	if (uhd_usrp_get_time_now(inst->usrp, 0, &secs, &frac) == UHD_ERROR_NONE) {
		gettimeofday(&tv, NULL);
		now.have_time = 1;
		now.device_time = (double)secs + frac;
		now.system_time = tv.tv_sec + tv.tv_usec / 1e6;
	}
	now.ref_changes = st->ref_changes + (st->valid && now.ref_locked != st->ref_locked);
	now.gps_changes = st->gps_changes + (st->valid && now.gps_locked != st->gps_locked);
	if (!st->valid || strcmp(now.clock_source, st->clock_source) || strcmp(now.time_source, st->time_source)
	 || now.ref_locked != st->ref_locked || now.gps_locked != st->gps_locked) {
		LOGP(DUHD, st->valid ? LOGL_NOTICE : LOGL_INFO,
		     "Clock: reference %s%s, time source %s%s (lock changes so far: reference %lu, GPS %lu)\n",
		     now.clock_source,
		     now.ref_locked < 0 ? "" : now.ref_locked ? " locked" : " NOT LOCKED",
		     now.time_source,
		     now.gps_locked < 0 ? ", no GPS" : now.gps_locked ? ", GPS locked" : ", GPS NOT LOCKED",
		     now.ref_changes, now.gps_changes);
	}
	*st = now;
}

static double get_software_time(uhd_instance_t *inst)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (double)(ts.tv_sec - inst->software_base_ts.tv_sec) +
	       (double)(ts.tv_nsec - inst->software_base_ts.tv_nsec) / 1000000000.0;
}

static void init_software_clock(uhd_instance_t *inst)
{
	clock_gettime(CLOCK_MONOTONIC_RAW, &inst->software_base_ts);
	inst->rx_time_secs = 0;
	inst->rx_time_fract_sec = 0.0;
	inst->software_clock = 1;
	/* TX timestamps don't work with software clock - they would all be "Late" */
	if (inst->tx_timestamps) {
		LOGP(DUHD, LOGL_NOTICE, "Disabling TX timestamps for software clock mode\n");
		inst->tx_timestamps = 0;
	}
	LOGP(DUHD, LOGL_NOTICE, "Using software clock for TX-only mode (may drift relative to SDR sample clock)\n");
}

static int uhd_open_device(size_t channel, const char *_device_args, const char *_stream_args, const char *_tune_args, const char *tx_antenna, const char *rx_antenna, const char *clock_source, double tx_frequency, double rx_frequency, double lo_offset, double rate, double tx_gain, double rx_gain, double bandwidth, int timestamps)
{
	uhd_error error;
	double got_frequency, got_rate, got_gain, got_bandwidth;
	char got_antenna[64], got_clock[64];
	uhd_tune_request_t	tune_request;
	uhd_tune_result_t	tune_result;
	uhd_stream_args_t	stream_args;
	size_t tx_samps_per_buff, rx_samps_per_buff;
	uhd_instance_t *inst;

	/* Allocate a new instance and zero-initialize */
	inst = calloc(1, sizeof(uhd_instance_t));
	if (!inst) {
		LOGP(DUHD, LOGL_ERROR, "Failed to allocate UHD instance\n");
		return -ENOMEM;
	}

	inst->samplerate = rate;
	inst->tx_timestamps = timestamps;

	LOGP(DUHD, LOGL_INFO, "Using device args \"%s\"\n", _device_args);
	LOGP(DUHD, LOGL_INFO, "Using stream args \"%s\"\n", _stream_args);
	LOGP(DUHD, LOGL_INFO, "Using tune args \"%s\"\n", _tune_args);

	/* create USRP */
	LOGP(DUHD, LOGL_INFO, "Creating USRP with args \"%s\"...\n", _device_args);
	error = uhd_usrp_make(&inst->usrp, _device_args);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create USRP\n");
		uhd_tx_inst = uhd_rx_inst = inst;
		uhd_close();
		return -EIO;
	}

	/* clock source */
	if (clock_source && clock_source[0]) {
		if (!strcasecmp(clock_source, "list")) {
			uhd_string_vector_handle clocks;
			size_t clocks_length;
			int i;
			error = uhd_string_vector_make(&clocks);
			if (error) {
				clock_vector_error:
				LOGP(DUHD, LOGL_ERROR, "Failed to handle UHD vector, please fix!\n");
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EIO;
			}
			error = uhd_usrp_get_clock_sources(inst->usrp, 0, &clocks);
			if (error) {
				LOGP(DUHD, LOGL_ERROR, "Failed to request list of clock sources!\n");
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EIO;
			}
			error = uhd_string_vector_size(clocks, &clocks_length);
			if (error)
				goto clock_vector_error;
			for (i = 0; i < (int)clocks_length; i++) {
				error = uhd_string_vector_at(clocks, i, got_clock, sizeof(got_clock));
				if (error)
					goto clock_vector_error;
				LOGP(DUHD, LOGL_NOTICE, "Clock source: '%s'\n", got_clock);
			}
			uhd_string_vector_free(&clocks);
			error = uhd_usrp_get_clock_source(inst->usrp, 0, got_clock, sizeof(got_clock));
			if (error) {
				LOGP(DUHD, LOGL_ERROR, "Failed to get clock source\n");
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EINVAL;
			}
			LOGP(DUHD, LOGL_NOTICE, "Default clock source: '%s'\n", got_clock);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return 1;
		}
		error = uhd_usrp_set_clock_source(inst->usrp, clock_source, 0);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set clock source to '%s'\n", clock_source);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		error = uhd_usrp_get_clock_source(inst->usrp, 0, got_clock, sizeof(got_clock));
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get clock source\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}
		if (!!strcasecmp(clock_source, got_clock)) {
			LOGP(DUHD, LOGL_NOTICE, "Given clock source '%s' was accepted, but driver claims to use '%s'\n", clock_source, got_clock);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}
	}

	/* apply PPM frequency correction (UHD has no native PPM API, adjust frequencies manually) */
	if (sdr_config) {
		if (sdr_config->tx_ppm != 0.0) {
			inst->ppm_tx = sdr_config->tx_ppm;
			if (tx_frequency) {
				double corrected = tx_frequency * (1.0 + sdr_config->tx_ppm / 1e6);
				LOGP(DUHD, LOGL_INFO, "Applying TX PPM correction %.3f: %.0f Hz -> %.0f Hz\n", sdr_config->tx_ppm, tx_frequency, corrected);
				tx_frequency = corrected;
			}
		}
		if (sdr_config->rx_ppm != 0.0) {
			inst->ppm_rx = sdr_config->rx_ppm;
			if (rx_frequency) {
				double corrected = rx_frequency * (1.0 + sdr_config->rx_ppm / 1e6);
				LOGP(DUHD, LOGL_INFO, "Applying RX PPM correction %.3f: %.0f Hz -> %.0f Hz\n", sdr_config->rx_ppm, rx_frequency, corrected);
				rx_frequency = corrected;
			}
		}
	}

	if (tx_frequency) {
		/* antenna */
		if (tx_antenna && tx_antenna[0]) {
			if (!strcasecmp(tx_antenna, "list")) {
				uhd_string_vector_handle antennas;
				size_t antennas_length;
				int i;
				error = uhd_string_vector_make(&antennas);
				if (error) {
					tx_vector_error:
					LOGP(DUHD, LOGL_ERROR, "Failed to handle UHD vector, please fix!\n");
					uhd_tx_inst = uhd_rx_inst = inst;
					uhd_close();
					return -EIO;
				}
				error = uhd_usrp_get_tx_antennas(inst->usrp, channel, &antennas);
				if (error) {
					LOGP(DUHD, LOGL_ERROR, "Failed to request list of TX antennas!\n");
					uhd_tx_inst = uhd_rx_inst = inst;
					uhd_close();
					return -EIO;
				}
				error = uhd_string_vector_size(antennas, &antennas_length);
				if (error)
					goto tx_vector_error;
				for (i = 0; i < (int)antennas_length; i++) {
					error = uhd_string_vector_at(antennas, i, got_antenna, sizeof(got_antenna));
					if (error)
						goto tx_vector_error;
					LOGP(DUHD, LOGL_NOTICE, "TX Antenna: '%s'\n", got_antenna);
				}
				uhd_string_vector_free(&antennas);
				error = uhd_usrp_get_tx_antenna(inst->usrp, channel, got_antenna, sizeof(got_antenna));
				if (error) {
					LOGP(DUHD, LOGL_ERROR, "Failed to get TX antenna\n");
					uhd_tx_inst = uhd_rx_inst = inst;
					uhd_close();
					return -EINVAL;
				}
				LOGP(DUHD, LOGL_NOTICE, "Default TX Antenna: '%s'\n", got_antenna);
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return 1;
			}
			error = uhd_usrp_set_tx_antenna(inst->usrp, tx_antenna, channel);
			if (error) {
				LOGP(DUHD, LOGL_ERROR, "Failed to set TX antenna to '%s'\n", tx_antenna);
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EIO;
			}
			error = uhd_usrp_get_tx_antenna(inst->usrp, channel, got_antenna, sizeof(got_antenna));
			if (error) {
				LOGP(DUHD, LOGL_ERROR, "Failed to get TX antenna\n");
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EINVAL;
			}
			if (!!strcasecmp(tx_antenna, got_antenna)) {
				LOGP(DUHD, LOGL_NOTICE, "Given TX antenna '%s' was accepted, but driver claims to use '%s'\n", tx_antenna, got_antenna);
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EINVAL;
			}
		}

		/* create streamers */
		error = uhd_tx_streamer_make(&inst->tx_streamer);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to create TX streamer\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* set rate */
		error = uhd_usrp_set_tx_rate(inst->usrp, rate, channel);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set TX rate to %.0f Hz\n", rate);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what rate actually is */
		error = uhd_usrp_get_tx_rate(inst->usrp, channel, &got_rate);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get TX rate\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_rate - rate) > 1.0) {
			LOGP(DUHD, LOGL_ERROR, "Given TX rate %.0f Hz is not supported, try %.0f Hz\n", rate, got_rate);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}

		/* set gain */
		error = uhd_usrp_set_tx_gain(inst->usrp, tx_gain, channel, "");
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set TX gain to %.0f\n", tx_gain);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what gain actually is */
		error = uhd_usrp_get_tx_gain(inst->usrp, channel, "", &got_gain);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get TX gain\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_gain - tx_gain) > 0.001) {
			LOGP(DUHD, LOGL_NOTICE, "Given TX gain %.0f is not supported, we use %.0f\n", tx_gain, got_gain);
			tx_gain = got_gain;
		}

		/* set frequency */
		memset(&tune_request, 0, sizeof(tune_request));
		tune_request.target_freq = tx_frequency;
		if (lo_offset) {
			tune_request.rf_freq_policy = UHD_TUNE_REQUEST_POLICY_MANUAL;
			tune_request.rf_freq = tx_frequency + lo_offset;
		} else
			tune_request.rf_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
		tune_request.dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
		tune_request.args = options_strdup(_tune_args);
		error = uhd_usrp_set_tx_freq(inst->usrp, &tune_request, channel, &tune_result);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set TX frequency to %.0f Hz\n", tx_frequency);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what frequency actually is */
		error = uhd_usrp_get_tx_freq(inst->usrp, channel, &got_frequency);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get TX frequency\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_frequency - tx_frequency) > 100.0) {
			LOGP(DUHD, LOGL_ERROR, "Given TX frequency %.0f Hz is not supported, try %.0f Hz\n", tx_frequency, got_frequency);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}

		/* set bandwidth */
		if (uhd_usrp_set_tx_bandwidth(inst->usrp, bandwidth, channel) != 0) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set TX bandwidth to %.0f Hz\n", bandwidth);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what bandwidth actually is */
		error = uhd_usrp_get_tx_bandwidth(inst->usrp, channel, &got_bandwidth);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get TX bandwidth\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_bandwidth - bandwidth) > 100.0) {
			LOGP(DUHD, LOGL_ERROR, "Given TX bandwidth %.0f Hz is not supported, try %.0f Hz\n", bandwidth, got_bandwidth);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}

		/* set up streamer */
		memset(&stream_args, 0, sizeof(stream_args));
		stream_args.cpu_format = "fc32";
		stream_args.otw_format = "sc16";
		stream_args.args = options_strdup(_stream_args);
		stream_args.channel_list = &channel;
		stream_args.n_channels = 1;
		error = uhd_usrp_get_tx_stream(inst->usrp, &stream_args, inst->tx_streamer);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set TX streamer args\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* the device's serial: a USB reset of this one if TX does not
		 * respond at start */
		{
			uhd_usrp_tx_info_t info;

			memset(&info, 0, sizeof(info));
			if (!uhd_usrp_get_tx_info(inst->usrp, channel, &info)) {
				if (info.mboard_serial)
					snprintf(inst->serial, sizeof(inst->serial), "%s", info.mboard_serial);
				uhd_usrp_tx_info_free(&info);
			}
		}

		/* get buffer sizes */
		error = uhd_tx_streamer_max_num_samps(inst->tx_streamer, &tx_samps_per_buff);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get TX streamer sample buffer\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
	}

	if (rx_frequency) {
		/* antenna */
		if (rx_antenna && rx_antenna[0]) {
			if (!strcasecmp(rx_antenna, "list")) {
				uhd_string_vector_handle antennas;
				size_t antennas_length;
				int i;
				error = uhd_string_vector_make(&antennas);
				if (error) {
					rx_vector_error:
					LOGP(DUHD, LOGL_ERROR, "Failed to handle UHD vector, please fix!\n");
					uhd_tx_inst = uhd_rx_inst = inst;
					uhd_close();
					return -EIO;
				}
				error = uhd_usrp_get_rx_antennas(inst->usrp, channel, &antennas);
				if (error) {
					LOGP(DUHD, LOGL_ERROR, "Failed to request list of RX antennas!\n");
					uhd_tx_inst = uhd_rx_inst = inst;
					uhd_close();
					return -EIO;
				}
				error = uhd_string_vector_size(antennas, &antennas_length);
				if (error)
					goto rx_vector_error;
				for (i = 0; i < (int)antennas_length; i++) {
					error = uhd_string_vector_at(antennas, i, got_antenna, sizeof(got_antenna));
					if (error)
						goto rx_vector_error;
					LOGP(DUHD, LOGL_NOTICE, "RX Antenna: '%s'\n", got_antenna);
				}
				uhd_string_vector_free(&antennas);
				error = uhd_usrp_get_rx_antenna(inst->usrp, channel, got_antenna, sizeof(got_antenna));
				if (error) {
					LOGP(DUHD, LOGL_ERROR, "Failed to get RX antenna\n");
					uhd_tx_inst = uhd_rx_inst = inst;
					uhd_close();
					return -EINVAL;
				}
				LOGP(DUHD, LOGL_NOTICE, "Default RX Antenna: '%s'\n", got_antenna);
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return 1;
			}
			error = uhd_usrp_set_rx_antenna(inst->usrp, rx_antenna, channel);
			if (error) {
				LOGP(DUHD, LOGL_ERROR, "Failed to set RX antenna to '%s'\n", rx_antenna);
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EIO;
			}
			error = uhd_usrp_get_rx_antenna(inst->usrp, channel, got_antenna, sizeof(got_antenna));
			if (error) {
				LOGP(DUHD, LOGL_ERROR, "Failed to get RX antenna\n");
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EINVAL;
			}
			if (!!strcasecmp(rx_antenna, got_antenna)) {
				LOGP(DUHD, LOGL_NOTICE, "Given RX antenna '%s' was accepted, but driver claims to use '%s'\n", rx_antenna, got_antenna);
				uhd_tx_inst = uhd_rx_inst = inst;
				uhd_close();
				return -EINVAL;
			}
		}
		/* create streamers */
		error = uhd_rx_streamer_make(&inst->rx_streamer);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to create RX streamer\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* create metadata */
		error = uhd_rx_metadata_make(&inst->rx_metadata);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to create RX metadata\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* set rate */
		error = uhd_usrp_set_rx_rate(inst->usrp, rate, channel);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set RX rate to %.0f Hz\n", rate);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what rate actually is */
		error = uhd_usrp_get_rx_rate(inst->usrp, channel, &got_rate);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get RX rate\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_rate - rate) > 1.0) {
			LOGP(DUHD, LOGL_ERROR, "Given RX rate %.0f Hz is not supported, try %.0f Hz\n", rate, got_rate);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}

		/* set gain */
		error = uhd_usrp_set_rx_gain(inst->usrp, rx_gain, channel, "");
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set RX gain to %.0f\n", rx_gain);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what gain actually is */
		error = uhd_usrp_get_rx_gain(inst->usrp, channel, "", &got_gain);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get RX gain\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_gain - rx_gain) > 0.001) {
			LOGP(DUHD, LOGL_NOTICE, "Given RX gain %.3f is not supported, we use %.3f\n", rx_gain, got_gain);
			rx_gain = got_gain;
		}

		/* set frequency */
		memset(&tune_request, 0, sizeof(tune_request));
		tune_request.target_freq = rx_frequency;
		if (lo_offset) {
			tune_request.rf_freq_policy = UHD_TUNE_REQUEST_POLICY_MANUAL;
			tune_request.rf_freq = rx_frequency + lo_offset;
		} else
			tune_request.rf_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
		tune_request.dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
		tune_request.args = options_strdup(_tune_args);
		error = uhd_usrp_set_rx_freq(inst->usrp, &tune_request, channel, &tune_result);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set RX frequency to %.0f Hz\n", rx_frequency);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what frequency actually is */
		error = uhd_usrp_get_rx_freq(inst->usrp, channel, &got_frequency);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get RX frequency\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_frequency - rx_frequency) > 100.0) {
			LOGP(DUHD, LOGL_ERROR, "Given RX frequency %.0f Hz is not supported, try %.0f Hz\n", rx_frequency, got_frequency);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}

		/* set bandwidth */
		if (uhd_usrp_set_rx_bandwidth(inst->usrp, bandwidth, channel) != 0) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set RX bandwidth to %.0f Hz\n", bandwidth);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* see what bandwidth actually is */
		error = uhd_usrp_get_rx_bandwidth(inst->usrp, channel, &got_bandwidth);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get RX bandwidth\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
		if (fabs(got_bandwidth - bandwidth) > 100.0) {
			LOGP(DUHD, LOGL_ERROR, "Given RX bandwidth %.0f Hz is not supported, try %.0f Hz\n", bandwidth, got_bandwidth);
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EINVAL;
		}

		/* set up streamer */
		memset(&stream_args, 0, sizeof(stream_args));
		stream_args.cpu_format = "fc32";
		stream_args.otw_format = "sc16";
		stream_args.args = options_strdup(_stream_args);
		stream_args.channel_list = &channel;
		stream_args.n_channels = 1;
		error = uhd_usrp_get_rx_stream(inst->usrp, &stream_args, inst->rx_streamer);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to set RX streamer args\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}

		/* get buffer sizes */
		error = uhd_rx_streamer_max_num_samps(inst->rx_streamer, &rx_samps_per_buff);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to get RX streamer sample buffer\n");
			uhd_tx_inst = uhd_rx_inst = inst;
			uhd_close();
			return -EIO;
		}
	}

	/* Single-device mode: both TX and RX point to the same instance */
	uhd_tx_inst = uhd_rx_inst = inst;

	return 0;
}

/* USB reset of the Ettus USB device (vendor 2500) with this serial; waits
 * until it is back. A network device has no USB entry: -ENODEV. */
static int uhd_usb_reset(const char *serial)
{
	char path[512], val[64], dev[64] = "";
	unsigned busnum = 0, devnum = 0;
	struct dirent *de;
	DIR *dir;
	FILE *fp;
	int fd, rc, i;

	if (!serial[0])
		return -ENODEV;
	dir = opendir("/sys/bus/usb/devices");
	if (!dir)
		return -errno;
	while ((de = readdir(dir)) && !dev[0]) {
		int ok;

		if (de->d_name[0] == '.')
			continue;
		snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s/idVendor", de->d_name);
		if (!(fp = fopen(path, "r")))
			continue;
		ok = fgets(val, sizeof(val), fp) && !strncmp(val, "2500", 4);
		fclose(fp);
		if (!ok)
			continue;
		snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s/serial", de->d_name);
		if (!(fp = fopen(path, "r")))
			continue;
		ok = fgets(val, sizeof(val), fp) != NULL;
		fclose(fp);
		val[strcspn(val, "\r\n")] = '\0';
		if (!ok || strcmp(val, serial))
			continue;
		snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s/busnum", de->d_name);
		if ((fp = fopen(path, "r"))) {
			ok = fscanf(fp, "%u", &busnum) == 1;
			fclose(fp);
		} else
			ok = 0;
		snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s/devnum", de->d_name);
		if (ok && (fp = fopen(path, "r"))) {
			if (fscanf(fp, "%u", &devnum) == 1)
				snprintf(dev, sizeof(dev), "/dev/bus/usb/%03u/%03u", busnum, devnum);
			fclose(fp);
		}
	}
	closedir(dir);
	if (!dev[0])
		return -ENODEV;

	fd = open(dev, O_WRONLY);
	if (fd < 0) {
		rc = -errno;
		LOGP(DUHD, LOGL_ERROR, "USB reset of the device %s (%s): %s\n", serial, dev, strerror(errno));
		return rc;
	}
	rc = ioctl(fd, USBDEVFS_RESET, 0) < 0 ? -errno : 0;
	close(fd);
	if (rc) {
		LOGP(DUHD, LOGL_ERROR, "USB reset of the device %s (%s): %s\n", serial, dev, strerror(-rc));
		return rc;
	}
	/* back when its node can be opened again (up to 10 s) */
	for (i = 0; i < 100; i++) {
		usleep(100000);
		fd = open(dev, O_WRONLY);
		if (fd >= 0) {
			close(fd);
			break;
		}
	}
	LOGP(DUHD, LOGL_NOTICE, "USB reset of the device %s (%s) done\n", serial, dev);
	return 0;
}

#ifndef TEST_TX_PROBE_FAIL
#define TEST_TX_PROBE_FAIL 0	/* test: the first start's TX probe fails */
#endif

/* Does TX work end to end? A burst of zeros (no time stamp, start and end
 * of burst) of 100 ms, at least 64 packets - far more than UHD's USB send
 * frames, so the transfers really go out - then the device's end of burst
 * ACK is awaited (1 s). A send only queues into the send frames and returns
 * at once: a broken USB transfer (a device that kept it from an earlier
 * run) shows as an error at a later send or as a missing ACK. Before
 * streaming starts. 0: it works; what: the reason. */
static int uhd_tx_probe(char *what, size_t len)
{
	const void *buffs_ptr[1];
	uhd_tx_metadata_handle md = NULL;
	uhd_async_metadata_handle amd = NULL;
	size_t mtu = 0, count = 0, total;
	uhd_error error;
	float *zero;
	int rc = 0, acked = 0, events = 0;
	double t0;

	what[0] = '\0';
	if (!uhd_tx_inst || !uhd_tx_inst->tx_streamer)
		return 0;
	if (uhd_tx_streamer_max_num_samps(uhd_tx_inst->tx_streamer, &mtu) || !mtu)
		return 0;
	total = (size_t)(uhd_tx_inst->samplerate * 0.1);
	if (total < 64 * mtu)
		total = 64 * mtu;
	zero = calloc(total * 2, sizeof(float));
	if (!zero)
		return 0;
	if (uhd_tx_metadata_make(&md, false, 0, 0.0, true, true)) {
		free(zero);
		return 0;
	}
	buffs_ptr[0] = zero;
	error = uhd_tx_streamer_send(uhd_tx_inst->tx_streamer, buffs_ptr, total, &md, 1.0, &count);
	/* the ACK of the burst's end (any other event, e.g. an underflow of
	 * this burst, is not a broken transport) */
	if (!error && count == total && !uhd_async_metadata_make(&amd)) {
		t0 = uhd_mono_time();
		while (!acked && uhd_mono_time() - t0 < 1.0) {
			uhd_async_metadata_event_code_t code;
			bool valid = false;

			if (uhd_tx_streamer_recv_async_msg(uhd_tx_inst->tx_streamer, &amd, 0.1, &valid) || !valid)
				continue;
			events++;
			if (!uhd_async_metadata_event_code(amd, &code) && code == UHD_ASYNC_METADATA_EVENT_CODE_BURST_ACK)
				acked = 1;
		}
		uhd_async_metadata_free(&amd);
	}
#if TEST_TX_PROBE_FAIL
	{
		static int failed;

		if (!failed++) {
			error = UHD_ERROR_IO;
			snprintf(what, len, "TEST_TX_PROBE_FAIL");
			rc = -1;
		}
	}
#endif
	if (rc) {
		/* the test's failure */
	} else if (error) {
		uhd_tx_streamer_last_error(uhd_tx_inst->tx_streamer, what, len);
		rc = -1;
	} else if (count < total) {
		snprintf(what, len, "%zu of %zu probe samples sent within 1 s", count, total);
		rc = -1;
	} else if (!acked) {
		snprintf(what, len, "no end of burst ACK within 1 s (%d other events)", events);
		rc = -1;
	} else
		LOGP(DUHD, LOGL_INFO, "TX probe: %zu samples sent, end of burst ACK received\n", total);
	uhd_tx_metadata_free(&md);
	free(zero);
	return rc;
}

/* Open the device. With TX, one probe write: a device whose TX does not
 * respond (its USB transfer broken, a state it keeps across processes
 * until it is reset) is reset on USB and opened again, once. */
int uhd_open(size_t channel, const char *_device_args, const char *_stream_args, const char *_tune_args, const char *tx_antenna, const char *rx_antenna, const char *clock_source, double tx_frequency, double rx_frequency, double lo_offset, double rate, double tx_gain, double rx_gain, double bandwidth, int timestamps)
{
	char what[256], serial[64];
	int rc;

	rc = uhd_open_device(channel, _device_args, _stream_args, _tune_args, tx_antenna, rx_antenna, clock_source, tx_frequency, rx_frequency, lo_offset, rate, tx_gain, rx_gain, bandwidth, timestamps);
	if (rc < 0 || tx_frequency == 0.0 || !uhd_tx_probe(what, sizeof(what)))
		return rc;

	snprintf(serial, sizeof(serial), "%s", uhd_tx_inst->serial);
	LOGP(DUHD, LOGL_ERROR, "TX of the device %s does not respond (%s): USB reset and open again\n",
	     serial[0] ? serial : "(no serial)", what);
	uhd_close();
	if (uhd_usb_reset(serial) < 0) {
		LOGP(DUHD, LOGL_ERROR, "TX does not respond and the device cannot be reset (not a USB device?)\n");
		return -EIO;
	}
	rc = uhd_open_device(channel, _device_args, _stream_args, _tune_args, tx_antenna, rx_antenna, clock_source, tx_frequency, rx_frequency, lo_offset, rate, tx_gain, rx_gain, bandwidth, timestamps);
	if (rc < 0)
		return rc;
	if (uhd_tx_probe(what, sizeof(what))) {
		LOGP(DUHD, LOGL_ERROR, "TX still does not respond after the USB reset (%s)\n", what);
		uhd_close();
		return -EIO;
	}
	LOGP(DUHD, LOGL_NOTICE, "TX responds after the USB reset\n");
	return 0;
}

/* start streaming */
int uhd_start(void)
{
	uhd_error error;
	int need_software_clock = 0;

	/* enable rx stream if configured */
	if (uhd_rx_inst && uhd_rx_inst->rx_streamer) {
		uhd_stream_cmd_t stream_cmd;
		memset(&stream_cmd, 0, sizeof(stream_cmd));
		stream_cmd.stream_mode = UHD_STREAM_MODE_START_CONTINUOUS;
		stream_cmd.stream_now = true;
		error = uhd_rx_streamer_issue_stream_cmd(uhd_rx_inst->rx_streamer, &stream_cmd);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to issue RX stream command\n");
			return -EIO;
		}
	}

	/* Determine if we need software clock:
	 * - TX-only mode (no RX stream on this UHD instance)
	 * - Split mode (RX is on a different device) */
	if (uhd_tx_inst && uhd_tx_inst->tx_streamer) {
		if (!(uhd_rx_inst && uhd_rx_inst->rx_streamer)) {
			/* No RX stream on UHD - TX-only or split mode */
			need_software_clock = 1;
		}
		if (sdr_config && sdr_config->split_mode) {
			/* Split mode - RX is on different device */
			need_software_clock = 1;
		}
	}

	if (need_software_clock && uhd_tx_inst && uhd_tx_inst->tx_streamer) {
		init_software_clock(uhd_tx_inst);

		/* NOTE: Pre-fill removed - it was causing UHD_ERROR_IO on subsequent sends.
		 * The timing algorithm should handle the initial ramp-up. */
	}

	return 0;
}

void uhd_close(void)
{
	LOGP(DUHD, LOGL_DEBUG, "Clean up UHD\n");
	if (uhd_tx_inst) {
		if (uhd_tx_inst->tx_streamer)
			uhd_tx_streamer_free(&uhd_tx_inst->tx_streamer);
		if (uhd_tx_inst->tx_metadata)
			uhd_tx_metadata_free(&uhd_tx_inst->tx_metadata);
		if (uhd_tx_inst->tx_async_md)
			uhd_async_metadata_free(&uhd_tx_inst->tx_async_md);
		if (uhd_tx_inst == uhd_rx_inst) {
			/* single-device mode: also free RX resources from same instance */
			if (uhd_tx_inst->rx_streamer)
				uhd_rx_streamer_free(&uhd_tx_inst->rx_streamer);
			if (uhd_tx_inst->rx_metadata)
				uhd_rx_metadata_free(&uhd_tx_inst->rx_metadata);
		}
		if (uhd_tx_inst->usrp)
			uhd_usrp_free(&uhd_tx_inst->usrp);
		free(uhd_tx_inst);
	}
	if (uhd_rx_inst && uhd_rx_inst != uhd_tx_inst) {
		/* split mode: free RX instance separately */
		if (uhd_rx_inst->rx_streamer)
			uhd_rx_streamer_free(&uhd_rx_inst->rx_streamer);
		if (uhd_rx_inst->rx_metadata)
			uhd_rx_metadata_free(&uhd_rx_inst->rx_metadata);
		if (uhd_rx_inst->usrp)
			uhd_usrp_free(&uhd_rx_inst->usrp);
		free(uhd_rx_inst);
	}
	uhd_tx_inst = uhd_rx_inst = NULL;
}
int uhd_set_rx_frequency(double frequency)
{
	uhd_instance_t *inst = uhd_rx_inst;
	uhd_tune_request_t tune_request;
	uhd_tune_result_t tune_result;
	uhd_error error;
	double got_frequency;
	double corrected;

	if (!inst || !inst->usrp) {
		LOGP(DUHD, LOGL_ERROR, "Cannot set frequency: no device open\n");
		return -ENODEV;
	}

	/* apply manual PPM correction */
	corrected = frequency;
	if (inst->ppm_rx != 0.0)
		corrected = frequency * (1.0 + inst->ppm_rx / 1e6);

	memset(&tune_request, 0, sizeof(tune_request));
	tune_request.target_freq = corrected;
	tune_request.rf_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
	tune_request.dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
	tune_request.args = "";

	error = uhd_usrp_set_rx_freq(inst->usrp, &tune_request, 0, &tune_result);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to set RX frequency to %.0f Hz\n", frequency);
		return -EIO;
	}

	error = uhd_usrp_get_rx_freq(inst->usrp, 0, &got_frequency);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get RX frequency\n");
		return -EIO;
	}

	if (fabs(got_frequency - corrected) > 100.0) {
		LOGP(DUHD, LOGL_ERROR, "Given RX frequency %.0f Hz is not supported, got %.0f Hz\n", frequency, got_frequency);
		return -EINVAL;
	}

	LOGP(DUHD, LOGL_INFO, "RX frequency set to %.0f Hz\n", got_frequency);
	return 0;
}

int uhd_set_tx_frequency(double frequency)
{
	uhd_instance_t *inst = uhd_tx_inst;
	uhd_tune_request_t tune_request;
	uhd_tune_result_t tune_result;
	uhd_error error;
	double got_frequency;
	double corrected;

	if (!inst || !inst->usrp) {
		LOGP(DUHD, LOGL_ERROR, "Cannot set TX frequency: no device open\n");
		return -ENODEV;
	}

	/* apply manual PPM correction */
	corrected = frequency;
	if (inst->ppm_tx != 0.0)
		corrected = frequency * (1.0 + inst->ppm_tx / 1e6);

	memset(&tune_request, 0, sizeof(tune_request));
	tune_request.target_freq = corrected;
	tune_request.rf_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
	tune_request.dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO;
	tune_request.args = "";

	error = uhd_usrp_set_tx_freq(inst->usrp, &tune_request, 0, &tune_result);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to set TX frequency to %.0f Hz\n", frequency);
		return -EIO;
	}

	error = uhd_usrp_get_tx_freq(inst->usrp, 0, &got_frequency);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get TX frequency\n");
		return -EIO;
	}

	if (fabs(got_frequency - corrected) > 100.0) {
		LOGP(DUHD, LOGL_ERROR, "Given TX frequency %.0f Hz is not supported, got %.0f Hz\n", frequency, got_frequency);
		return -EINVAL;
	}

	LOGP(DUHD, LOGL_INFO, "TX frequency set to %.0f Hz\n", got_frequency);
	return 0;
}


int uhd_send(float *buff, int num)
{
    	const void *buffs_ptr[1];
	int chunk;
	size_t sent = 0, count;
	size_t tx_samps_per_buff;
	uhd_error error;

	if (!uhd_tx_inst || !uhd_tx_inst->tx_streamer)
		return 0;

	error = uhd_tx_streamer_max_num_samps(uhd_tx_inst->tx_streamer, &tx_samps_per_buff);
	if (error)
		return 0;

	while (num) {
		chunk = num;
		if (chunk > (int)tx_samps_per_buff)
			chunk = (int)tx_samps_per_buff;
		/* Create TX metadata for this chunk.
		 * uhd_tx_metadata_make() allocates a new C++ object each call,
		 * so free the previous handle first to avoid leaking memory. */
		if (uhd_tx_inst->tx_metadata) {
			uhd_tx_metadata_free(&uhd_tx_inst->tx_metadata);
			uhd_tx_inst->tx_metadata = NULL;
		}
		if (uhd_tx_inst->tx_timestamps)
			error = uhd_tx_metadata_make(&uhd_tx_inst->tx_metadata, true, uhd_tx_inst->tx_time_secs, uhd_tx_inst->tx_time_fract_sec, false, false);
		else
			error = uhd_tx_metadata_make(&uhd_tx_inst->tx_metadata, false, 0, 0.0, false, false);
		if (error) {
			LOGP(DUHD, LOGL_ERROR, "Failed to create TX metadata (error=%d)\n", error);
			break;
		}
		buffs_ptr[0] = buff;
		count = 0;
		double t_before = get_software_time(uhd_tx_inst);
#if TEST_TX_FAIL
		static struct timespec fail_t0;
		struct timespec fail_now;

		clock_gettime(CLOCK_MONOTONIC, &fail_now);
		if (!fail_t0.tv_sec)
			fail_t0 = fail_now;
		if (fail_now.tv_sec - fail_t0.tv_sec >= 5 && fail_now.tv_sec - fail_t0.tv_sec < 8) {
			error = UHD_ERROR_IO;
			count = 0;
		} else
#endif
		error = uhd_tx_streamer_send(uhd_tx_inst->tx_streamer, buffs_ptr, chunk, &uhd_tx_inst->tx_metadata, 1.0, &count);
		double t_after = get_software_time(uhd_tx_inst);
		if (t_after - t_before > 0.01) {
			LOGP(DUHD, LOGL_ERROR, "UHD SEND BLOCKED %.1fms (chunk=%d count=%zu ts=%d)\n",
			     (t_after - t_before) * 1000.0, chunk, count,
			     uhd_tx_inst->tx_timestamps);
		}
		if (error) {
			char what[256] = "";

			uhd_tx_streamer_last_error(uhd_tx_inst->tx_streamer, what, sizeof(what));
			LOGP(DUHD, LOGL_DEBUG, "Failed to write to TX streamer (error=%d, chunk=%d, count=%zu): %s\n", error, chunk, count, what);
			uhd_tx_write_failed(uhd_tx_inst, error, what);
			break;
		}
		if (count == 0) {
			LOGP(DUHD, LOGL_DEBUG, "TX streamer returned 0 samples (chunk=%d)\n", chunk);
			uhd_tx_write_failed(uhd_tx_inst, UHD_ERROR_NONE, "send() timed out, nothing sent");
			break;
		}
		uhd_tx_write_ok(uhd_tx_inst);

		/* increment time stamp */
		uhd_tx_inst->tx_time_fract_sec += (double)count / uhd_tx_inst->samplerate;
		if (uhd_tx_inst->tx_time_fract_sec >= 1.0) {
			uhd_tx_inst->tx_time_secs++;
			uhd_tx_inst->tx_time_fract_sec -= 1.0;
		}

		sent += count;
		buff += count * 2;
		num -= count;
	}

	/* Not sent (a failed write or a timeout): the samples are dropped, but
	 * their time passes as if sent - TX time moves on, so the caller is not
	 * asked again for the same time (it would make samples far faster than
	 * real time, and the application's time, counted in samples, would run
	 * ahead); where a blocking send paces TX, wait their duration instead. */
	if (num > 0) {
		uhd_tx_inst->tx_time_fract_sec += (double)num / uhd_tx_inst->samplerate;
		while (uhd_tx_inst->tx_time_fract_sec >= 1.0) {
			uhd_tx_inst->tx_time_secs++;
			uhd_tx_inst->tx_time_fract_sec -= 1.0;
		}
		uhd_tx_inst->tx_dropped += num;
		if (uhd_tx_inst->software_clock)
			usleep((useconds_t)((double)num / uhd_tx_inst->samplerate * 1e6));
	}

	return sent;
}

/* TX has no device time reference (TX-only or split mode). The host clock
 * must not pace TX then: it drifts against the USRP sample clock and drains
 * the device FIFO. Pacing is done by send() backpressure instead. */
int uhd_tx_device_paced(void)
{
	return uhd_tx_inst && uhd_tx_inst->tx_streamer && uhd_tx_inst->software_clock;
}

/* Collect pending TX async events without blocking, add them to the counters. */
void uhd_tx_poll_events(unsigned long *underflow, unsigned long *seq_error, unsigned long *time_error)
{
	uhd_async_metadata_event_code_t code;
	bool valid;
	int i;

	if (!uhd_tx_inst || !uhd_tx_inst->tx_streamer)
		return;
	if (!uhd_tx_inst->tx_async_md && uhd_async_metadata_make(&uhd_tx_inst->tx_async_md))
		return;

	for (i = 0; i < 256; i++) {
		valid = false;
		if (uhd_tx_streamer_recv_async_msg(uhd_tx_inst->tx_streamer, &uhd_tx_inst->tx_async_md, 0.0, &valid) || !valid)
			break;
		if (uhd_async_metadata_event_code(uhd_tx_inst->tx_async_md, &code))
			break;
		if (code & (UHD_ASYNC_METADATA_EVENT_CODE_UNDERFLOW | UHD_ASYNC_METADATA_EVENT_CODE_UNDERFLOW_IN_PACKET))
			(*underflow)++;
		if (code & (UHD_ASYNC_METADATA_EVENT_CODE_SEQ_ERROR | UHD_ASYNC_METADATA_EVENT_CODE_SEQ_ERROR_IN_BURST))
			(*seq_error)++;
		if (code & UHD_ASYNC_METADATA_EVENT_CODE_TIME_ERROR)
			(*time_error)++;
	}
}

/* Return the TX streamer MTU (max samples per send call) */
int uhd_get_tx_mtu(void)
{
	size_t mtu;
	uhd_error error;

	if (!uhd_tx_inst || !uhd_tx_inst->tx_streamer)
		return 0;

	error = uhd_tx_streamer_max_num_samps(uhd_tx_inst->tx_streamer, &mtu);
	if (error)
		return 0;

	return (int)mtu;
}

/* read what we got, return 0, if buffer is empty, otherwise return the number of samples */
int uhd_receive(float *buff, int max)
{
    	void *buffs_ptr[1];
	size_t count;
	size_t num_to_read;
	size_t rx_samps_per_buff;
	uhd_error error;
	bool has_time_spec;
	int rc;

	/* TX-only mode or no RX instance: no RX streamer available */
	if (!uhd_rx_inst || !uhd_rx_inst->rx_streamer)
		return 0;

	if (max <= 0)
		return 0;

	error = uhd_rx_streamer_max_num_samps(uhd_rx_inst->rx_streamer, &rx_samps_per_buff);
	if (error)
		return 0;

	/* read up to MTU, but no more than available buffer space */
	num_to_read = ((size_t)max < rx_samps_per_buff) ? (size_t)max : rx_samps_per_buff;
	/* read RX stream */
	buffs_ptr[0] = buff;
	count = 0;
	error = uhd_rx_streamer_recv(uhd_rx_inst->rx_streamer, buffs_ptr, num_to_read, &uhd_rx_inst->rx_metadata, 0.0, false, &count);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to read from UHD device.\n");
		return -EIO;
	}
	{
		/* Check for overflow (lost samples). UHD reports it on a recv()
		 * that returns no samples, so check before testing count. */
		uhd_rx_metadata_error_code_t error_code;
		uhd_rx_metadata_error_code(uhd_rx_inst->rx_metadata, &error_code);
		if (error_code == UHD_RX_METADATA_ERROR_CODE_OVERFLOW) {
			sdr_rx_overflow = 1;
			static int overflow_count = 0;
			overflow_count++;
			if (overflow_count <= 10 || (overflow_count % 100) == 0)
				LOGP(DUHD, LOGL_ERROR, "RX overflow #%d (lost samples)\n", overflow_count);
		}
	}
	if (count) {
		/* Measure samples lost before this packet from the device time
		 * stamp: compare the packet time with the expected time of the
		 * next sample. The gap is zero-filled by the caller and, without
		 * TX timestamps, added to the counted RX time below. */
		long long lost = 0;
		{
			bool hts = false;
			int64_t pkt_secs;
			double pkt_frac;

			if (uhd_rx_metadata_has_time_spec(uhd_rx_inst->rx_metadata, &hts) == 0 && hts &&
			    uhd_rx_metadata_time_spec(uhd_rx_inst->rx_metadata, &pkt_secs, &pkt_frac) == 0) {
				double pkt_time = (double)pkt_secs + pkt_frac;

				if (uhd_rx_inst->rx_dev_valid) {
					lost = llround((pkt_time - uhd_rx_inst->rx_dev_next) * uhd_rx_inst->samplerate);
					if (lost < 0 || lost > (long long)uhd_rx_inst->samplerate * 10) {
						/* time went backwards or jumped (device time reset): resync only */
						LOGP(DUHD, LOGL_ERROR, "RX time stamp jumped by %lld samples, resync without fill\n", lost);
						lost = 0;
					} else if (lost > 0) {
						LOGP(DUHD, LOGL_ERROR, "RX gap of %lld samples (%.3f ms) before packet\n",
						     lost, 1000.0 * (double)lost / uhd_rx_inst->samplerate);
						uhd_rx_inst->rx_gap += lost;
					}
				}
				uhd_rx_inst->rx_dev_next = pkt_time + (double)count / uhd_rx_inst->samplerate;
				uhd_rx_inst->rx_dev_valid = 1;
			} else if (sdr_rx_overflow && !uhd_rx_inst->rx_dev_warned) {
				LOGP(DUHD, LOGL_ERROR, "RX overflow without device time stamps: lost samples cannot be measured or filled\n");
				uhd_rx_inst->rx_dev_warned = 1;
			}
		}

		if (uhd_rx_inst->tx_timestamps) {
			/* get time stamp of received RX packet */
			rc = uhd_rx_metadata_has_time_spec(uhd_rx_inst->rx_metadata, &has_time_spec);
			if (rc == 0 && has_time_spec)
				rc = uhd_rx_metadata_time_spec(uhd_rx_inst->rx_metadata, &uhd_rx_inst->rx_time_secs, &uhd_rx_inst->rx_time_fract_sec);
			if (rc < 0 || !has_time_spec) {
				LOGP(DSOAPY, LOGL_ERROR, "SDR RX: No time stamps available. This may cuse little gaps and problems with time slot based networks, like C-Netz.\n");
				uhd_rx_inst->tx_timestamps = 0;
			}
		}
		if (!uhd_rx_inst->tx_timestamps) {
			/* RX time is a count of received samples. Samples lost in an
			 * overflow are not counted, so RX time would fall behind the
			 * device and TX headroom (tx_time - rx_time) would shrink by
			 * the lost amount, permanently. Add the measured gap. */
			if (lost > 0) {
				uhd_rx_inst->rx_time_fract_sec += (double)lost / uhd_rx_inst->samplerate;
				while (uhd_rx_inst->rx_time_fract_sec >= 1.0) {
					uhd_rx_inst->rx_time_secs++;
					uhd_rx_inst->rx_time_fract_sec -= 1.0;
				}
			}

			/* increment time stamp */
			uhd_rx_inst->rx_time_fract_sec += (double)count / uhd_rx_inst->samplerate;
			if (uhd_rx_inst->rx_time_fract_sec >= 1.0) {
				uhd_rx_inst->rx_time_secs++;
				uhd_rx_inst->rx_time_fract_sec -= 1.0;
			}
		}
	}

	return count;
}

/* Return and clear the number of RX samples lost before the last packet. */
long uhd_rx_take_gap(void)
{
	long long gap;

	if (!uhd_rx_inst)
		return 0;
	gap = uhd_rx_inst->rx_gap;
	uhd_rx_inst->rx_gap = 0;
	return (long)gap;
}

/* estimate number of samples that can be sent */
int uhd_get_tosend(int buffer_size)
{
	double advance;
	int tosend;
	static int call_count = 0;
	static int underrun_count = 0;
	static double last_now = 0.0;
	static double max_gap = 0.0;
	int split_mode = sdr_config && sdr_config->split_mode;
#if TEST_UNDERRUN_RECOVERY
	static int underrun_triggered = 0;
#endif

	if (!uhd_tx_inst)
		return 0;

	call_count++;

	/* Split mode with software clock: independent timing based on wall clock.
	 * 
	 * Unlike full-duplex mode where we sync TX to RX timestamps, in split mode
	 * we track wall-clock time and try to stay ahead by buffer_size samples.
	 * 
	 * tx_time tracks how much time we've "sent" - when we send N samples,
	 * tx_time advances by N/samplerate seconds.
	 * 
	 * The goal is to keep tx_time ahead of now by buffer_duration.
	 * tosend = how many samples needed to reach that goal.
	 */
	if (split_mode && uhd_tx_inst->software_clock) {
		double now = get_software_time(uhd_tx_inst);
		double tx_time = (double)uhd_tx_inst->tx_time_secs + uhd_tx_inst->tx_time_fract_sec;
		double buffer_duration = (double)buffer_size / uhd_tx_inst->samplerate;
		double target_tx_time;
		double gap = now - last_now;
		
		/* Track maximum gap between calls */
		if (last_now > 0.0 && gap > max_gap) {
			max_gap = gap;
		}
		last_now = now;
		
		/* Initialize TX time on first call.
		 * Set tx_time = now, target is now + buffer_duration.
		 * This means we need buffer_size samples to reach target. */
		if (uhd_tx_inst->tx_time_secs == 0 && uhd_tx_inst->tx_time_fract_sec == 0.0) {
			uhd_tx_inst->tx_time_secs = (time_t)now;
			uhd_tx_inst->tx_time_fract_sec = now - (double)uhd_tx_inst->tx_time_secs;
			LOGP(DUHD, LOGL_INFO, "SPLIT TX init: now=%.6f tx_time=%.6f target_headroom=%.3fms samplerate=%.0f\n",
			     now, now, buffer_duration * 1000.0, uhd_tx_inst->samplerate);
			/* Return buffer_size on first call to build up headroom */
			return buffer_size;
		}
		
		/* Target: tx_time should be (now + buffer_duration) ahead.
		 * tosend = (target - tx_time) * samplerate
		 *        = (now + buffer_duration - tx_time) * samplerate
		 */
		target_tx_time = now + buffer_duration;
		tosend = (int)((target_tx_time - tx_time) * uhd_tx_inst->samplerate);
		
		/* Debug: log periodically */
		if (call_count < 100 || (call_count % 5000) == 0) {
			double ahead_ms = (tx_time - now) * 1000.0;
			LOGP(DUHD, LOGL_DEBUG, "SPLIT[%d]: now=%.6f tx_time=%.6f ahead=%.1fms tosend=%d buf=%d\n",
			     call_count, now, tx_time, ahead_ms, tosend, buffer_size);
		}
		
		/* Allow up to 2x buffer_size for catch-up.
		 * The comment "will catch up over time" was wrong — capping to
		 * buffer_size means we can never produce MORE than real-time,
		 * so the deficit only grows. Allow 2x so we can actually recover. */
		if (tosend > buffer_size * 2) {
			double behind_ms = (target_tx_time - tx_time - buffer_duration) * 1000.0;
			underrun_count++;
			if (underrun_count <= 10 || (underrun_count % 100) == 0) {
				LOGP(DUHD, LOGL_NOTICE, "SPLIT TX catching up #%d: %.1f ms behind target (capping to 2x buf)\n",
				     underrun_count, behind_ms);
			}
			tosend = buffer_size * 2;
		}
		if (tosend < 0)
			tosend = 0;
		
		return tosend;
	}

	/* Non-split mode: Update software clock if in use (TX-only mode) */
	if (uhd_tx_inst->software_clock && uhd_rx_inst) {
		double sw_time = get_software_time(uhd_tx_inst);
		uhd_rx_inst->rx_time_secs = (time_t)sw_time;
		uhd_rx_inst->rx_time_fract_sec = sw_time - (double)uhd_rx_inst->rx_time_secs;
	}

	/* we need the rx time stamp to determine how much data is already sent in advance */
	if (!uhd_rx_inst || (uhd_rx_inst->rx_time_secs == 0 && uhd_rx_inst->rx_time_fract_sec == 0.0)) {
		return 0;
	}

#if TEST_UNDERRUN_RECOVERY
	/* HACK: Trigger underrun after 5 seconds of operation, only once */
	if (!underrun_triggered && uhd_rx_inst->rx_time_secs >= 5) {
		underrun_triggered = 1;
		LOGP(DUHD, LOGL_NOTICE, "HACK: Triggering artificial TX underrun by sleeping 200ms...\n");
		usleep(200000); /* 200ms delay to cause underrun */
	}
#endif

	/* if we have not yet sent any data, we set initial tx time stamp */
	if (uhd_tx_inst->tx_time_secs == 0 && uhd_tx_inst->tx_time_fract_sec == 0.0) {
		uhd_tx_inst->tx_time_secs = uhd_rx_inst->rx_time_secs;
		uhd_tx_inst->tx_time_fract_sec = uhd_rx_inst->rx_time_fract_sec;
		/* Always start TX time ahead by buffer_size samples.
		 * This is critical for TX-only mode where the software clock
		 * starts immediately but sample generation takes time.
		 * For RX+TX mode with timestamps, the SDR uses these as actual
		 * transmission times. For TX-only mode without timestamps,
		 * this provides the necessary head start to prevent underrun. */
		uhd_tx_inst->tx_time_fract_sec += (double)buffer_size / uhd_tx_inst->samplerate;
		if (uhd_tx_inst->tx_time_fract_sec >= 1.0) {
			uhd_tx_inst->tx_time_fract_sec -= 1.0;
			uhd_tx_inst->tx_time_secs++;
		}
	}

	/* we check how advance our transmitted time stamp is */
	advance = ((double)uhd_tx_inst->tx_time_secs + uhd_tx_inst->tx_time_fract_sec) - ((double)uhd_rx_inst->rx_time_secs + uhd_rx_inst->rx_time_fract_sec);
	tosend = buffer_size - (int)(advance * uhd_tx_inst->samplerate);

	/* Periodic timing debug */
	{
		static double last_dbg = 0;
		double now_dbg = (double)uhd_rx_inst->rx_time_secs + uhd_rx_inst->rx_time_fract_sec;
		if (now_dbg - last_dbg >= 1.0 || tosend > buffer_size) {
			double tx_t = (double)uhd_tx_inst->tx_time_secs + uhd_tx_inst->tx_time_fract_sec;
			double rx_t = (double)uhd_rx_inst->rx_time_secs + uhd_rx_inst->rx_time_fract_sec;
			LOGP(DUHD, LOGL_NOTICE,
			     "TX_TIMING: tx=%.3f rx=%.3f advance=%.1fms tosend=%d buf=%d ts=%d swclk=%d\n",
			     tx_t, rx_t, advance * 1000.0, tosend, buffer_size,
			     uhd_tx_inst->tx_timestamps, uhd_tx_inst->software_clock);
			last_dbg = now_dbg;
		}
	}

	/* With TX time stamps, a packet stamped in the past is late and the
	 * device drops it: producing more samples cannot catch up, because
	 * tx_time only moves on by the samples sent. Any deficit (advance < 0)
	 * needs a resync of tx_time to rx_time + buffer.
	 * Without time stamps, allow producing up to 2x buffer_size so the
	 * caller can catch up from small deficits; resync only when hopelessly
	 * behind (> 2x buffer = can't recover). */
	if ((uhd_tx_inst->tx_timestamps && advance < 0.0) || tosend > buffer_size * 2) {
		LOGP(DUHD, LOGL_ERROR, "SDR TX underrun (%.1f ms behind), resync.\n",
			-advance * 1000.0);
		/* Always resync tx_time regardless of timestamp mode.
		 * With timestamps enabled, late packets are dropped by the driver,
		 * but tx_time must still be reset or we'll be stuck sending
		 * late-timestamped packets indefinitely. */
		uhd_tx_inst->tx_time_secs = uhd_rx_inst->rx_time_secs;
		uhd_tx_inst->tx_time_fract_sec = uhd_rx_inst->rx_time_fract_sec;
		uhd_tx_inst->tx_time_fract_sec += (double)buffer_size / uhd_tx_inst->samplerate;
		if (uhd_tx_inst->tx_time_fract_sec >= 1.0) {
			uhd_tx_inst->tx_time_fract_sec -= 1.0;
			uhd_tx_inst->tx_time_secs++;
		}
		tosend = buffer_size;
	} else if (tosend > buffer_size) {
		static int catchup_count = 0;
		catchup_count++;
		if (catchup_count <= 10 || (catchup_count % 100) == 0)
			LOGP(DUHD, LOGL_NOTICE, "TX catching up #%d: %.1f ms behind, producing %d samples (buf=%d)\n",
				catchup_count, -advance * 1000.0, tosend, buffer_size);
	}
	if (tosend < 0)
		tosend = 0;

	return tosend;
}

/* Query supported sample rates from device */
int uhd_query_sample_rates(const char *device_args, int direction, size_t channel, sdr_rate_info_t *info)
{
	uhd_usrp_handle usrp = NULL;
	uhd_meta_range_handle range = NULL;
	uhd_error error;
	double start, stop;
	int rc = -1;

	if (!info)
		return -EINVAL;

	memset(info, 0, sizeof(*info));

	LOGP(DUHD, LOGL_INFO, "Querying sample rates from device '%s' (direction=%s, channel=%zu)\n",
	     device_args ? device_args : "(default)",
	     direction == 1 ? "TX" : "RX", channel);

	/* Create USRP device */
	error = uhd_usrp_make(&usrp, device_args ? device_args : "");
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create USRP for rate query\n");
		goto out;
	}

	/* Create meta range handle */
	error = uhd_meta_range_make(&range);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create meta range\n");
		goto out;
	}

	/* Get sample rate range */
	if (direction == 1) {
		/* TX direction */
		error = uhd_usrp_get_tx_rates(usrp, channel, range);
	} else {
		/* RX direction */
		error = uhd_usrp_get_rx_rates(usrp, channel, range);
	}

	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get %s sample rates\n", direction == 1 ? "TX" : "RX");
		goto out;
	}

	/* Get range start and stop */
	error = uhd_meta_range_start(range, &start);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get rate range start\n");
		goto out;
	}

	error = uhd_meta_range_stop(range, &stop);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get rate range stop\n");
		goto out;
	}

	/* UHD typically returns continuous range */
	info->is_continuous = 1;
	info->min_rate = start;
	info->max_rate = stop;
	info->rates = NULL;
	info->num_rates = 0;

	LOGP(DUHD, LOGL_INFO, "Device supports continuous sample rate range:\n");
	LOGP(DUHD, LOGL_INFO, "  Range: %.0f - %.0f Hz (%.3f - %.3f MHz)\n",
	     start, stop, start / 1e6, stop / 1e6);

	rc = 0;

out:
	if (range)
		uhd_meta_range_free(&range);
	if (usrp)
		uhd_usrp_free(&usrp);

	return rc;
}


/**
 * Query frequency range from UHD device
 *
 * @param device_args   Device arguments string
 * @param direction     1=TX, 0=RX
 * @param channel       Channel number
 * @param min_freq      Output: minimum frequency (Hz)
 * @param max_freq      Output: maximum frequency (Hz)
 * @return 0 on success, -1 on failure
 */
int uhd_query_freq_range(const char *device_args, int direction, size_t channel,
			 double *min_freq, double *max_freq)
{
	uhd_usrp_handle usrp = NULL;
	uhd_meta_range_handle range = NULL;
	uhd_error error;
	double start, stop;
	int rc = -1;

	if (!min_freq || !max_freq)
		return -EINVAL;

	*min_freq = 0;
	*max_freq = 0;

	LOGP(DUHD, LOGL_INFO, "Querying frequency range from device '%s' (direction=%s, channel=%zu)\n",
	     device_args ? device_args : "(default)",
	     direction == 1 ? "TX" : "RX", channel);

	/* Create USRP device */
	error = uhd_usrp_make(&usrp, device_args ? device_args : "");
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create USRP for freq range query\n");
		goto out;
	}

	/* Create meta range handle */
	error = uhd_meta_range_make(&range);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create meta range\n");
		goto out;
	}

	/* Get frequency range */
	if (direction == 1) {
		error = uhd_usrp_get_tx_freq_range(usrp, channel, range);
	} else {
		error = uhd_usrp_get_rx_freq_range(usrp, channel, range);
	}

	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get %s frequency range\n", direction == 1 ? "TX" : "RX");
		goto out;
	}

	/* Get range start and stop */
	error = uhd_meta_range_start(range, &start);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get freq range start\n");
		goto out;
	}

	error = uhd_meta_range_stop(range, &stop);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get freq range stop\n");
		goto out;
	}

	*min_freq = start;
	*max_freq = stop;

	LOGP(DUHD, LOGL_INFO, "Device %s frequency range: %.0f - %.0f Hz (%.3f - %.3f MHz)\n",
	     direction == 1 ? "TX" : "RX", start, stop, start / 1e6, stop / 1e6);

	rc = 0;

out:
	if (range)
		uhd_meta_range_free(&range);
	if (usrp)
		uhd_usrp_free(&usrp);

	return rc;
}

/**
 * Query gain range from UHD device
 *
 * @param device_args   Device arguments string
 * @param direction     1=TX, 0=RX
 * @param channel       Channel number
 * @param min_gain      Output: minimum gain (dB)
 * @param max_gain      Output: maximum gain (dB)
 * @param gain_names    Output: space-separated list of gain element names
 * @param gain_names_len Size of gain_names buffer
 * @return 0 on success, -1 on failure
 */
int uhd_query_gain_info(const char *device_args, int direction, size_t channel,
			double *min_gain, double *max_gain,
			char *gain_names, int gain_names_len)
{
	uhd_usrp_handle usrp = NULL;
	uhd_meta_range_handle range = NULL;
	uhd_string_vector_handle names = NULL;
	uhd_error error;
	double start, stop;
	size_t num_names = 0;
	int rc = -1;
	int pos = 0;

	if (!min_gain || !max_gain)
		return -EINVAL;

	*min_gain = 0;
	*max_gain = 0;
	if (gain_names && gain_names_len > 0)
		gain_names[0] = '\0';

	LOGP(DUHD, LOGL_INFO, "Querying gain info from device '%s' (direction=%s, channel=%zu)\n",
	     device_args ? device_args : "(default)",
	     direction == 1 ? "TX" : "RX", channel);

	/* Create USRP device */
	error = uhd_usrp_make(&usrp, device_args ? device_args : "");
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create USRP for gain query\n");
		goto out;
	}

	/* Create meta range handle */
	error = uhd_meta_range_make(&range);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create meta range\n");
		goto out;
	}

	/* Get overall gain range */
	if (direction == 1) {
		error = uhd_usrp_get_tx_gain_range(usrp, "", channel, range);
	} else {
		error = uhd_usrp_get_rx_gain_range(usrp, "", channel, range);
	}

	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get %s gain range\n", direction == 1 ? "TX" : "RX");
		goto out;
	}

	error = uhd_meta_range_start(range, &start);
	if (!error)
		error = uhd_meta_range_stop(range, &stop);

	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to get gain range values\n");
		goto out;
	}

	*min_gain = start;
	*max_gain = stop;

	LOGP(DUHD, LOGL_INFO, "Device %s overall gain range: %.1f - %.1f dB\n",
	     direction == 1 ? "TX" : "RX", start, stop);

	/* Get list of gain elements */
	error = uhd_string_vector_make(&names);
	if (error) {
		LOGP(DUHD, LOGL_ERROR, "Failed to create string vector\n");
		goto out;
	}

	if (direction == 1) {
		error = uhd_usrp_get_tx_gain_names(usrp, channel, &names);
	} else {
		error = uhd_usrp_get_rx_gain_names(usrp, channel, &names);
	}

	if (!error) {
		uhd_string_vector_size(names, &num_names);
		for (size_t i = 0; i < num_names && gain_names && gain_names_len > 0; i++) {
			char name[64];
			uhd_string_vector_at(names, i, name, sizeof(name));
			int len = snprintf(gain_names + pos, gain_names_len - pos,
					   "%s%s", (i > 0) ? " " : "", name);
			if (len > 0 && pos + len < gain_names_len)
				pos += len;

			LOGP(DUHD, LOGL_INFO, "  Gain element: '%s'\n", name);
		}
	}

	rc = 0;

out:
	if (names)
		uhd_string_vector_free(&names);
	if (range)
		uhd_meta_range_free(&range);
	if (usrp)
		uhd_usrp_free(&usrp);

	return rc;
}
