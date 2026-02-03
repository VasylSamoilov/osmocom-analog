/* osmocalibrate - Fast automatic SDR clock calibration
 *
 * Scans GSM bands to find FCCH tones and measure crystal frequency offset.
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
#include <getopt.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

#include "../libcalibrate/calibrate.h"
#include "../libcalibrate/calibrate_sdr.h"
#include "../liblogging/logging.h"

/* Global variables required by linked libraries */
sender_t *sender_head = NULL;
int use_sdr = 0;

/* Default SDR parameters */
#define DEFAULT_SAMPLE_RATE   1000000  /* 1 MHz */
#define DEFAULT_RX_GAIN       30.0
#define DEFAULT_BANDWIDTH     1000000  /* 1 MHz */

static int quit = 0;

static void sighandler(int signum)
{
	(void)signum;
	quit = 1;
}

static void print_usage(const char *name)
{
	printf("Usage: %s [options]\n", name);
	printf("\nFast automatic SDR clock calibration using GSM FCCH signals.\n\n");
	printf("Options:\n");
	printf("  -h, --help              Show this help\n");
	printf("  -v, --verbose           Increase verbosity\n");
	printf("  -q, --quiet             Quiet mode (only output result)\n");
	printf("  -b, --band <band>       GSM band to scan (900, 850, 1800, 1900)\n");
	printf("  -t, --timeout <sec>     Timeout in seconds (default: 10)\n");
	printf("  -d, --device <dev>      SDR device string (e.g., 'driver=lime')\n");
	printf("  -g, --gain <dB>         RX gain in dB (default: %.0f)\n", DEFAULT_RX_GAIN);
	printf("  -j, --json              Output in JSON format\n");
	printf("\nExamples:\n");
	printf("  %s                              Auto-calibrate on GSM-900\n", name);
	printf("  %s -b 1800 -v                   Calibrate on GSM-1800, verbose\n", name);
	printf("  %s -d 'driver=rtlsdr' -g 40     Use RTL-SDR with 40dB gain\n", name);
	printf("  %s -j                           Output JSON for scripting\n", name);
}

static int parse_band(const char *str)
{
	int band = atoi(str);
	switch (band) {
	case 900:
		return GSM_BAND_900;
	case 850:
		return GSM_BAND_850;
	case 1800:
		return GSM_BAND_1800;
	case 1900:
		return GSM_BAND_1900;
	default:
		fprintf(stderr, "Unknown band: %s (use 900, 850, 1800, or 1900)\n", str);
		return -1;
	}
}

int main(int argc, char *argv[])
{
	calibrate_config_t config;
	calibrate_result_t result;
	calibrate_sdr_t sdr;
	const char *device = "";
	double rx_gain = DEFAULT_RX_GAIN;
	int json_output = 0;
	double center_freq, bandwidth;
	int arfcn_start, arfcn_end;
	int rc;

	static const struct option long_options[] = {
		{"help",    no_argument,       0, 'h'},
		{"verbose", no_argument,       0, 'v'},
		{"quiet",   no_argument,       0, 'q'},
		{"band",    required_argument, 0, 'b'},
		{"timeout", required_argument, 0, 't'},
		{"device",  required_argument, 0, 'd'},
		{"gain",    required_argument, 0, 'g'},
		{"json",    no_argument,       0, 'j'},
		{0, 0, 0, 0}
	};

	/* Default config */
	calibrate_config_default(&config);

	/* Parse command line */
	while (1) {
		int c = getopt_long(argc, argv, "hvqb:t:d:g:j", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_usage(argv[0]);
			return 0;
		case 'v':
			config.verbosity++;
			break;
		case 'q':
			config.verbosity = 0;
			break;
		case 'b':
			config.gsm_bands = parse_band(optarg);
			if (config.gsm_bands < 0)
				return 1;
			break;
		case 't':
			config.timeout_sec = atof(optarg);
			break;
		case 'd':
			device = optarg;
			break;
		case 'g':
			rx_gain = atof(optarg);
			break;
		case 'j':
			json_output = 1;
			break;
		default:
			print_usage(argv[0]);
			return 1;
		}
	}

	/* Set up signal handlers */
	signal(SIGINT, sighandler);
	signal(SIGTERM, sighandler);

	if (config.verbosity >= 1 && !json_output) {
		printf("osmocalibrate - SDR clock calibration\n");
		printf("Scanning GSM band %d...\n", 
		       config.gsm_bands == GSM_BAND_900 ? 900 :
		       config.gsm_bands == GSM_BAND_850 ? 850 :
		       config.gsm_bands == GSM_BAND_1800 ? 1800 : 1900);
	}

	/* Get band parameters for SDR tuning */
	gsm_band_params(config.gsm_bands, &center_freq, &bandwidth,
	                &arfcn_start, &arfcn_end);

	if (config.verbosity >= 2)
		fprintf(stderr, "Opening SDR: device='%s', freq=%.1f MHz, rate=%.0f, gain=%.0f dB\n",
		        device, center_freq / 1e6, (double)DEFAULT_SAMPLE_RATE, rx_gain);

	/* Open SDR */
	rc = calibrate_sdr_open(&sdr, device, center_freq, 
	                         DEFAULT_SAMPLE_RATE, rx_gain, DEFAULT_BANDWIDTH);
	if (rc < 0) {
		if (!json_output) {
			fprintf(stderr, "Failed to open SDR: %s\n", strerror(-rc));
			fprintf(stderr, "\nMake sure:\n");
			fprintf(stderr, "  1. SDR device is connected\n");
			fprintf(stderr, "  2. SoapySDR or UHD is installed\n");
			fprintf(stderr, "  3. Device string is correct (e.g., -d 'driver=rtlsdr')\n");
		} else {
			printf("{\"success\": false, \"error\": \"SDR open failed: %s\"}\n", 
			       strerror(-rc));
		}
		return 1;
	}

	if (config.verbosity >= 1 && !json_output)
		printf("SDR opened successfully\n");

	/* Run calibration */
	rc = calibrate_auto(&sdr, &config, &result);

	/* Close SDR */
	calibrate_sdr_close(&sdr);

	if (rc < 0) {
		if (!json_output) {
			fprintf(stderr, "Calibration failed: %s\n", strerror(-rc));
		} else {
			printf("{\"success\": false, \"error\": \"%s\"}\n", strerror(-rc));
		}
		return 1;
	}

	/* Output result */
	if (json_output) {
		printf("{\"success\": true, ");
		printf("\"ppm\": %.6f, ", result.ppm_offset);
		printf("\"hz\": %.2f, ", result.hz_offset);
		printf("\"frequency\": %.0f, ", result.freq_hz);
		printf("\"confidence\": %.2f", result.confidence);
		printf("}\n");
	} else {
		printf("\n=== Calibration Result ===\n");
		printf("Frequency offset: %.2f Hz\n", result.hz_offset);
		printf("PPM offset:       %.3f ppm\n", result.ppm_offset);
		printf("Signal frequency: %.3f MHz\n", result.freq_hz / 1e6);
		printf("Confidence:       %.0f%%\n", result.confidence * 100);
		printf("\nTo apply this correction:\n");
		printf("  --ppm %.1f\n", -result.ppm_offset);
	}

	fflush(stdout);
	fflush(stderr);
	
	/* Force quick exit to avoid hanging on soapy cleanup threads */
	_exit(0);
}

/* Stub functions required by linked libraries */
sender_t *get_sender_by_empfangsfrequenz(double __attribute__((unused)) freq) { return NULL; }
void osmo_cc_set_log_cat(int __attribute__((unused)) cc_log_cat) {}
