/* osmocalibrate - Fast automatic SDR clock calibration
 *
 * Scans GSM bands to find FCCH tones and measure crystal frequency offset.
 * Uses the common sdr_config option infrastructure shared with all other
 * osmocom-analog tools.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

enum paging_signal;
typedef struct sender sender_t;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

#include "../libcalibrate/calibrate.h"
#include "../libfastmath/fastmath.h"
#include "../libsdr/sdr_config.h"
#include "../liblogging/logging.h"
#include "../liboptions/options.h"

#ifdef HAVE_SOAPY
#include "../libsdr/soapy.h"
#endif
#ifdef HAVE_UHD
#include "../libsdr/uhd.h"
#endif

/* Globals required by linked libraries */
sender_t *sender_head = NULL;
int use_sdr = 0;

/* Calibration-specific options */
static calibrate_config_t config;
static int json_output = 0;

/* Custom option IDs (above 512 to avoid collision with sdr_config) */
enum {
	OPT_BAND = 600,
	OPT_TIMEOUT,
	OPT_JSON,
	OPT_NUM_CHANNELS,
	OPT_FAST_MATH,
	OPT_CHANNELIZER,
	OPT_ARFCN,
	OPT_PROBE_SEQUENTIAL,
};

static void print_help(const char *arg0)
{
	printf("Usage: %s [options]\n", arg0);
	printf("\nFast automatic SDR clock calibration using GSM FCCH signals.\n");
	printf("Uses parallel channelizer-based detection across multiple ARFCNs.\n\n");
	printf("Calibration options:\n");
	printf("  -h, --help              Show this help\n");
	printf("  -v, --verbose <level>   Set verbosity (default: 5)\n");
	printf("  -q, --quiet             Quiet mode (only output result)\n");
	printf("      --band <band>       GSM band: 900, 850, 1800, 1900, all\n");
	printf("      --timeout <sec>     Timeout per band in seconds (default: 60)\n");
	printf("      --channels <n>      Number of parallel channels (default: %d)\n",
	       CALIB_MAX_CHANNELS);
	printf("      --fast-math         Use fixed-point math (lower CPU usage)\n");
	printf("      --channelizer       Use polyphase resampler (much lower CPU)\n");
	printf("      --arfcn <n>         Force FCCH detection on a specific ARFCN\n");
	printf("      --probe-sequential  Probe ARFCNs one-by-one; stop on clear FCCH\n");
	printf("  -j, --json              Output in JSON format\n");
	sdr_config_print_help();
	printf("\nExamples:\n");
	printf("  %s --sdr-uhd --sdr-rx-gain 40 --sdr-samplerate 2000000\n", arg0);
	printf("  %s --sdr-uhd --band all          Scan all GSM bands\n", arg0);
	printf("  %s --sdr-soapy --band 1800 -v 7  Calibrate on DCS-1800, verbose\n", arg0);
	printf("  %s --sdr-soapy -j                Output JSON for scripting\n", arg0);
}

static void add_options(void)
{
	option_add('h', "help", 0);
	option_add('v', "verbose", 1);
	option_add('q', "quiet", 0);
	option_add(OPT_BAND, "band", 1);
	option_add(OPT_TIMEOUT, "timeout", 1);
	option_add(OPT_NUM_CHANNELS, "channels", 1);
	option_add(OPT_FAST_MATH, "fast-math", 0);
	option_add(OPT_CHANNELIZER, "channelizer", 0);
	option_add(OPT_ARFCN, "arfcn", 1);
	option_add(OPT_PROBE_SEQUENTIAL, "probe-sequential", 0);
	option_add('j', "json", 0);
	sdr_config_add_options();
}

static int parse_band(const char *str)
{
	if (strcasecmp(str, "all") == 0)
		return GSM_BAND_ALL;
	int band = atoi(str);
	switch (band) {
	case 900:  return GSM_BAND_900;
	case 850:  return GSM_BAND_850;
	case 1800: return GSM_BAND_1800;
	case 1900: return GSM_BAND_1900;
	default:
		fprintf(stderr, "Unknown band: %s (use 900, 850, 1800, 1900, or all)\n", str);
		return -1;
	}
}

static int handle_options(int short_option, int argi, char **argv)
{
	switch (short_option) {
	case 'h':
		print_help(argv[0]);
		return 0;
	case 'v':
		parse_logging_opt(argv[argi]);
		config.verbosity = 2;
		break;
	case 'q':
		config.verbosity = 0;
		break;
	case OPT_BAND:
		config.gsm_bands = parse_band(argv[argi]);
		if (config.gsm_bands < 0)
			return -EINVAL;
		break;
	case OPT_TIMEOUT:
		config.timeout_sec = atof(argv[argi]);
		break;
	case OPT_NUM_CHANNELS:
		config.num_channels = atoi(argv[argi]);
		if (config.num_channels < 1)
			config.num_channels = 1;
		if (config.num_channels > CALIB_MAX_CHANNELS)
			config.num_channels = CALIB_MAX_CHANNELS;
		break;
	case OPT_FAST_MATH:
		config.fast_math = 1;
		break;
	case OPT_CHANNELIZER:
		config.use_channelizer = 1;
		break;
	case OPT_ARFCN:
		config.forced_arfcn = atoi(argv[argi]);
		if (config.forced_arfcn <= 0) {
			fprintf(stderr, "Invalid ARFCN: %s\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_PROBE_SEQUENTIAL:
		config.probe_sequential = 1;
		break;
	case 'j':
		json_output = 1;
		break;
	default:
		return sdr_config_handle_options(short_option, argi, argv);
	}

	return 1;
}

/* Override: calibration is RX-only, no split device needed */
int sdr_check_separate_device_support(int tx_only, int rx_only, int split_mode)
{
	(void)rx_only;
	if (tx_only) {
		fprintf(stderr, "Calibration requires RX, --sdr-tx-only is not supported\n");
		return -1;
	}
	if (split_mode) {
		fprintf(stderr, "Calibration does not support split device mode\n");
		return -1;
	}
	return 0;
}

static int open_sdr_rx(double center_freq, int samplerate)
{
	/* Apply upconverter offset (mirrors sdr_open_internal in sdr.c) */
	double actual_freq = center_freq + sdr_config->rx_upconverter;

	if (sdr_config->rx_upconverter != 0.0)
		LOGP(DSDR, LOGL_INFO, "Calibrate upconverter RX: %.6f MHz + %.6f MHz = %.6f MHz\n",
		     center_freq / 1e6, sdr_config->rx_upconverter / 1e6, actual_freq / 1e6);

#ifdef HAVE_UHD
	if (sdr_config->uhd) {
		int rc;
		rc = uhd_open(sdr_config->channel,
		              sdr_config->device_args,
		              sdr_config->stream_args,
		              sdr_config->tune_args,
		              "",
		              sdr_config->rx_antenna,
		              sdr_config->clock_source,
		              0.0,
		              actual_freq,
		              sdr_config->lo_offset,
		              (double)samplerate,
		              0.0,
		              sdr_config->rx_gain,
		              sdr_config->bandwidth ? sdr_config->bandwidth : (double)samplerate,
		              0);
		if (rc) return rc;
		rc = uhd_start();
		if (rc < 0) { uhd_close(); return rc; }
		return 0;
	}
#endif
#ifdef HAVE_SOAPY
	if (sdr_config->soapy) {
		int rc;
		rc = soapy_open(sdr_config->channel,
		                sdr_config->device_args,
		                sdr_config->stream_args,
		                sdr_config->tune_args,
		                "",
		                sdr_config->rx_antenna,
		                sdr_config->clock_source,
		                0.0,
		                actual_freq,
		                sdr_config->lo_offset,
		                (double)samplerate,
		                0.0,
		                sdr_config->rx_gain,
		                sdr_config->bandwidth ? sdr_config->bandwidth : (double)samplerate,
		                0);
		if (rc) return rc;
		rc = soapy_start();
		if (rc < 0) { soapy_close(); return rc; }
		return 0;
	}
#endif
	fprintf(stderr, "No SDR backend selected. Use --sdr-soapy or --sdr-uhd.\n");
	return -ENODEV;
}

static void close_sdr(void)
{
#ifdef HAVE_SOAPY
	if (sdr_config->soapy)
		soapy_close();
#endif
#ifdef HAVE_UHD
	if (sdr_config->uhd)
		uhd_close();
#endif
}

int main(int argc, char *argv[])
{
	calibrate_result_t results[CALIB_MAX_BANDS];
	int num_results = 0;
	double center_freq, bandwidth;
	int arfcn_start, arfcn_end;
	int samplerate;
	int rc;

	logging_init();

	sdr_config_init(0.0);
	calibrate_config_default(&config);

	add_options();
	rc = options_command_line(argc, argv, handle_options);
	if (rc <= 0)
		return (rc < 0) ? 1 : 0;

	if (config.forced_arfcn > 0 && config.gsm_bands == GSM_BAND_ALL) {
		fprintf(stderr, "--arfcn requires a specific --band (not 'all')\n");
		return 1;
	}
	if (config.forced_arfcn > 0 && config.gsm_bands != GSM_BAND_ALL) {
		double ff = arfcn_to_freq(config.forced_arfcn, config.gsm_bands);
		if (ff == 0.0) {
			fprintf(stderr, "ARFCN %d is invalid for selected band %s\n",
			        config.forced_arfcn, gsm_band_name(config.gsm_bands));
			return 1;
		}
	}

	samplerate = sdr_config->samplerate;
	if (samplerate == 0)
		samplerate = 1000000;

	rc = sdr_configure(samplerate);
	if (rc < 0)
		return 1;
	if (rc == 0) {
		fprintf(stderr, "No SDR selected. Use --sdr-soapy or --sdr-uhd.\n");
		return 1;
	}

	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);

	/* Init fast math lookup tables if requested */
	if (config.fast_math) {
		rc = fastmath_init();
		if (rc < 0) {
			fprintf(stderr, "Failed to init fast math tables\n");
			return 1;
		}
	}

	/* Use GSM-900 center for initial SDR open (will retune during scan) */
	gsm_band_params(GSM_BAND_900, &center_freq, &bandwidth,
	                &arfcn_start, &arfcn_end);

	if (config.verbosity >= 1 && !json_output) {
		printf("osmocalibrate - SDR clock calibration\n");
		if (config.gsm_bands == GSM_BAND_ALL)
			printf("Scanning all GSM bands with %d parallel channels...\n",
			       config.num_channels);
		else
			printf("Scanning %s with %d parallel channels...\n",
			       gsm_band_name(config.gsm_bands),
			       config.num_channels);
	}

	rc = open_sdr_rx(center_freq, samplerate);
	if (rc < 0) {
		if (!json_output) {
			fprintf(stderr, "Failed to open SDR: %s\n", strerror(-rc));
		} else {
			printf("{\"success\": false, \"error\": \"SDR open failed: %s\"}\n",
			       strerror(-rc));
		}
		return 1;
	}

	if (config.verbosity >= 1 && !json_output)
		printf("SDR opened at %.3f MHz, %d Hz sample rate\n",
		       center_freq / 1e6, samplerate);

	rc = calibrate_auto(center_freq, samplerate, &config,
	                     results, &num_results);

	close_sdr();

	if (rc < 0 || num_results == 0) {
		if (!json_output)
			fprintf(stderr, "Calibration failed: %s\n",
			        strerror(rc < 0 ? -rc : ENODATA));
		else
			printf("{\"success\": false, \"error\": \"%s\"}\n",
			       strerror(rc < 0 ? -rc : ENODATA));
		return 1;
	}

	/* Output results */
	if (json_output) {
		printf("{\"success\": true, \"bands\": [");
		for (int i = 0; i < num_results; i++) {
			if (i > 0) printf(", ");
			printf("{\"band\": \"%s\", ", gsm_band_name(results[i].gsm_band));
			printf("\"ppm\": %.6f, ", results[i].ppm_offset);
			printf("\"hz\": %.2f, ", results[i].hz_offset);
			printf("\"stddev_hz\": %.2f, ", results[i].stddev_hz);
			printf("\"frequency\": %.0f, ", results[i].freq_hz);
			printf("\"measurements\": %d, ", results[i].num_measurements);
			printf("\"confidence\": %.2f}", results[i].confidence);
		}
		printf("]}\n");
	} else {
		printf("\n=== Calibration Results ===\n");
		for (int i = 0; i < num_results; i++) {
			printf("\n  %s (%.3f MHz):\n",
			       gsm_band_name(results[i].gsm_band),
			       results[i].freq_hz / 1e6);
			printf("    Offset:       %+.2f Hz (%.3f ppm)\n",
			       results[i].hz_offset, results[i].ppm_offset);
			printf("    Std dev:      %.1f Hz\n", results[i].stddev_hz);
			printf("    Measurements: %d (%.0f%% confidence)\n",
			       results[i].num_measurements,
			       results[i].confidence * 100);
		}

		/* Best result (lowest stddev) */
		int best = 0;
		for (int i = 1; i < num_results; i++) {
			if (results[i].stddev_hz < results[best].stddev_hz)
				best = i;
		}

		printf("\nBest result from %s:\n", gsm_band_name(results[best].gsm_band));
		printf("  Frequency offset: %+.2f Hz\n", results[best].hz_offset);
		printf("  PPM offset:       %.3f ppm\n", results[best].ppm_offset);
		printf("  Std deviation:    %.1f Hz\n", results[best].stddev_hz);
		printf("\nTo apply this correction:\n");
		printf("  --ppm %.1f\n", -results[best].ppm_offset);
	}

	fflush(stdout);
	fflush(stderr);
	_exit(0);
}

/* Stub functions required by linked libraries */
sender_t *get_sender_by_empfangsfrequenz(double __attribute__((unused)) freq) { return NULL; }
void osmo_cc_set_log_cat(int __attribute__((unused)) cc_log_cat) {}
