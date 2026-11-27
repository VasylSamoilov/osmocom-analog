/* osmocalibrate - Fast automatic SDR clock calibration
 *
 * GSM base stations as the reference: the band's carriers, measured in
 * windows of as many channels as the CPU budget allows, each confirmed as a
 * cell by its decoded identity; the result when enough towers agree.
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
#include <time.h>
#include <sys/resource.h>

#include "../libcalibrate/calibrate.h"
#include "../libsample/sample.h"
#include "../libsdr/sdr_config.h"
#include "../libsdr/sdr.h"
#include "../liblogging/logging.h"
#include "../liboptions/options.h"

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
	OPT_CPU,
	OPT_TOWERS,
	OPT_FULL,
	OPT_ARFCN,
	OPT_CONFIRM,
	OPT_NO_CONFIRM,
	OPT_DECODE_TIMEOUT,
	OPT_NO_BCCH,
};

static void print_help(const char *arg0)
{
	printf("Usage: %s [options]\n", arg0);
	printf("\nSDR clock calibration against GSM base stations: their FCCH gives the\n");
	printf("offset, their decoded identity (SCH, BCCH) proves they are cells.\n\n");
	printf("Calibration options:\n");
	printf("  -h, --help              Show this help\n");
	printf("  -v, --verbose <level>   Set verbosity (default: 5)\n");
	printf("  -q, --quiet             Quiet mode (only output result)\n");
	printf("      --band <band>       GSM band: 900, 850, 1800, 1900, all (default 900)\n");
	printf("      --arfcn <n>         Measure only this ARFCN\n");
	printf("      --towers <n>        Stop after n confirmed towers agree (default %d)\n", config.towers);
	printf("      --full              Measure every carrier found, list all towers\n");
	printf("      --channels auto|<n> Channels per window: as many as the CPU budget\n");
	printf("                          allows (default), or n\n");
	printf("      --cpu <percent>     CPU budget for the channels, %% of one core\n");
	printf("                          (default %.0f)\n", config.cpu_percent);
	printf("      --timeout <sec>     Most time per window (default %.0f)\n", config.timeout_sec);
	printf("      --confirm           Use only carriers confirmed as base stations by\n");
	printf("                          decoding their SCH (BSIC, frame number) (default)\n");
	printf("      --no-confirm        Use the measured carriers even if their identity\n");
	printf("                          does not decode\n");
	printf("      --decode-timeout <s> Time allowed for the identity decoding\n");
	printf("                          (default: %.0f)\n", config.decode_timeout_sec);
	printf("      --no-bcch           Decode only the SCH, not the System Information\n");
	printf("                          (MCC, MNC, LAC, cell identity)\n");
	printf("  -j, --json              Output in JSON format\n");
	printf("\nThe SDR rate is the widest the device and its USB link carry (with a\n");
	printf("margin), no wider than the band; --sdr-samplerate sets it.\n");
	sdr_config_print_help();
	printf("\nExamples:\n");
	printf("  %s --sdr-uhd --sdr-rx-gain 50 --sdr-rx-antenna RX2\n", arg0);
	printf("  %s --sdr-uhd --band all          Scan all GSM bands\n", arg0);
	printf("  %s --sdr-soapy driver=rtlsdr --band 900 --arfcn 76\n", arg0);
	printf("  %s --sdr-uhd -j                  Output JSON for scripting\n", arg0);
}

static void add_options(void)
{
	option_add('h', "help", 0);
	option_add('v', "verbose", 1);
	option_add('q', "quiet", 0);
	option_add(OPT_BAND, "band", 1);
	option_add(OPT_TIMEOUT, "timeout", 1);
	option_add(OPT_NUM_CHANNELS, "channels", 1);
	option_add(OPT_CPU, "cpu", 1);
	option_add(OPT_TOWERS, "towers", 1);
	option_add(OPT_FULL, "full", 0);
	option_add(OPT_ARFCN, "arfcn", 1);
	option_add(OPT_CONFIRM, "confirm", 0);
	option_add(OPT_NO_CONFIRM, "no-confirm", 0);
	option_add(OPT_DECODE_TIMEOUT, "decode-timeout", 1);
	option_add(OPT_NO_BCCH, "no-bcch", 0);
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
		if (!strcasecmp(argv[argi], "auto")) {
			config.num_channels = 0;
			break;
		}
		config.num_channels = atoi(argv[argi]);
		if (config.num_channels < 1) {
			fprintf(stderr, "--channels: 'auto' or a number from 1, not '%s'\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_CPU:
		config.cpu_percent = atof(argv[argi]);
		if (config.cpu_percent <= 0.0) {
			fprintf(stderr, "--cpu: a percentage of one core above 0, not '%s'\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_TOWERS:
		config.towers = atoi(argv[argi]);
		if (config.towers < 1) {
			fprintf(stderr, "--towers: a number from 1, not '%s'\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_FULL:
		config.full = 1;
		break;
	case OPT_ARFCN:
		config.forced_arfcn = atoi(argv[argi]);
		if (config.forced_arfcn <= 0) {
			fprintf(stderr, "Invalid ARFCN: %s\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_CONFIRM:
		config.confirm = 1;
		break;
	case OPT_NO_CONFIRM:
		config.confirm = 0;
		break;
	case OPT_DECODE_TIMEOUT:
		config.decode_timeout_sec = atof(argv[argi]);
		if (config.decode_timeout_sec <= 0.0) {
			fprintf(stderr, "Invalid decode timeout: %s\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_NO_BCCH:
		config.decode_bcch = 0;
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

static void *sdr;

/* RX only, the whole band at the DSP rate (channels = 0): libsdr opens the
 * device (upconverter, PPM, clock source as every program), its read thread
 * fills a ring that calibrate.c reads with sdr_read(). The SDR rate is the
 * DSP rate here, so the band passes unchanged (no channelizer stage). */
static int open_sdr_rx(double center_freq, int samplerate)
{
	/* 100 ms per read; the ring holds 4 x that (0.4 s): the gaps between
	 * the reads (scan processing, retune) stay far below it */
	int buffer_size = samplerate / 10;
	double rx_frequency = center_freq;
	/* the read thread sleeps this long (ms) when the device has nothing:
	 * half a UHD packet (about 2000 samples), at most 1 ms (at 16 MS/s and
	 * more, 1 ms is half of UHD's receive buffer: it overflowed) */
	double interval = 1000.0 * 1000.0 / samplerate;
	int rc;

	if (interval > 1.0)
		interval = 1.0;
	if (interval < 0.02)
		interval = 0.02;
	sdr = sdr_open(0, NULL, NULL, &rx_frequency, NULL, 0, 0.0, samplerate, buffer_size, interval, 0.0, 0.0, 0.0);
	if (!sdr)
		return -EIO;
	rc = sdr_start(sdr);
	if (rc < 0) {
		sdr_close(sdr);
		sdr = NULL;
		return rc;
	}
	calibrate_set_sdr(sdr, buffer_size);
	return 0;
}

static void close_sdr(void)
{
	calibrate_set_sdr(NULL, 0);
	if (sdr)
		sdr_close(sdr);
	sdr = NULL;
}

/* the SDR at another rate (calibrate.c, from the measured cost) */
static int reopen_sdr(int samplerate)
{
	double c, bw;
	int s, e;

	close_sdr();
	sdr_config->samplerate = samplerate;
	if (!sdr_config->bandwidth_given)
		sdr_config->bandwidth = samplerate;
	gsm_band_params(GSM_BAND_900, &c, &bw, &s, &e);
	return open_sdr_rx(c, samplerate);
}

static void print_cell_text(const calibrate_cell_t *cell, double applied_ppm)
{
	const gsm_cell_ident_t *id = &cell->ident;
	char ident[96] = "", mccmnc[16] = "-", laci[40] = "-";

	if (cell->confirmed)
		snprintf(ident, sizeof(ident), "BSIC %2d", id->bsic);
	else if (cell->decoded)
		snprintf(ident, sizeof(ident), "not confirmed");
	else
		snprintf(ident, sizeof(ident), "no identity");
	if (id->have_lai) {
		snprintf(mccmnc, sizeof(mccmnc), "%03d-%0*d", id->mcc, id->mnc_3digit ? 3 : 2, id->mnc);
		if (id->have_ci)
			snprintf(laci, sizeof(laci), "LAC %d CI %d", id->lac, id->ci);
		else
			snprintf(laci, sizeof(laci), "LAC %d", id->lac);
	}
	printf("  %-8s ARFCN %4d %9.3f MHz  %-13s %-7s %-20s", gsm_band_name(cell->gsm_band), cell->arfcn,
	       cell->freq_hz / 1e6, ident, mccmnc, laci);
	if (cell->num_measurements >= 3)
		printf(" %+8.1f Hz %7.3f ppm %6.1f dBFS  SNR %4.1f dB%s\n", cell->hz_offset,
		       cell->ppm_offset + applied_ppm, cell->level_dbfs, cell->snr_db,
		       (cell->confirmed && !cell->agrees) ? "  disagrees" : "");
	else
		printf(" no FCCH         %6.1f dBFS\n", cell->level_dbfs);
}

static void print_cell_json(const calibrate_cell_t *cell, double applied_ppm, int comma)
{
	const gsm_cell_ident_t *id = &cell->ident;

	printf("%s{\"band\": \"%s\", \"arfcn\": %d, \"frequency\": %.0f, \"measurements\": %d, "
	       "\"level_dbfs\": %.1f, \"confirmed\": %s",
	       comma ? ", " : "", gsm_band_name(cell->gsm_band), cell->arfcn, cell->freq_hz,
	       cell->num_measurements, cell->level_dbfs, cell->confirmed ? "true" : "false");
	if (cell->num_measurements >= 3)
		printf(", \"hz\": %.2f, \"ppm\": %.6f, \"stddev_hz\": %.2f, \"snr_db\": %.1f, \"agrees\": %s",
		       cell->hz_offset, cell->ppm_offset + applied_ppm, cell->stddev_hz, cell->snr_db,
		       cell->agrees ? "true" : "false");
	if (cell->confirmed)
		printf(", \"bsic\": %d", id->bsic);
	if (id->have_lai)
		printf(", \"mcc\": \"%03d\", \"mnc\": \"%0*d\", \"lac\": %d", id->mcc,
		       id->mnc_3digit ? 3 : 2, id->mnc, id->lac);
	if (id->have_ci)
		printf(", \"ci\": %d", id->ci);
	printf("}");
}

/* Per device: the usable part of the stream each side of the centre and the
 * rates (docs/SDR-FRONTENDS.md). One entry per device; the first that
 * matches. max_rate: refused above (0: no limit), max_auto_rate: the widest
 * chosen on its own (0: what the device and the bus carry; the CPU budget
 * then picks the rate, calibrate.c). */
typedef struct device_rule {
	const char	*name;
	int		uhd;		/* matches --sdr-uhd */
	const char	*args;		/* matches a Soapy device args substring (NULL: any) */
	double		usable_fraction, usable_max_hz;
	int		max_rate, max_auto_rate;
	const char	*why;		/* of max_rate */
} device_rule_t;

static const device_rule_t device_rules[] = {
	/* B200/B210: +-0.40 (power-of-2 decimation >= 4 of the master clock;
	 * above 15.36 MS/s the master clock is the rate, the AD9361 alone,
	 * +-0.40 with the LO centred, as here: no LO offset) */
	{ "UHD (B200/B210)", 1, NULL, 0.40, 0.0, 0, 0, NULL },
	/* RTL2832U: its FIR is flat to +-0.9 MHz at any rate */
	{ "RTL-SDR", 0, "rtlsdr", 0.40, 0.85e6, 0, 0, NULL },
	/* LMS7002M: clean to +-0.30 */
	{ "LimeSDR", 0, "lime", 0.30, 0.0, 0, 0, NULL },
	/* unknown */
	{ "this device", 0, NULL, 0.35, 0.0, 0, 0, NULL },
};

static const device_rule_t *device_rule(void)
{
	const char *args = sdr_config->device_args ? sdr_config->device_args : "";
	int n = sizeof(device_rules) / sizeof(device_rules[0]);

	for (int i = 0; i < n - 1; i++) {
		if (device_rules[i].uhd) {
			if (sdr_config->uhd)
				return &device_rules[i];
		} else if (sdr_config->soapy && device_rules[i].args && strstr(args, device_rules[i].args))
			return &device_rules[i];
	}
	return &device_rules[n - 1];
}

/* the band(s) asked for: the widest span */
static double wanted_span(void)
{
	double c, bw, span = 0.0;
	int s, e, b;

	for (b = GSM_BAND_900; b <= GSM_BAND_850; b++) {
		if (config.gsm_bands != GSM_BAND_ALL && config.gsm_bands != b)
			continue;
		gsm_band_params(b, &c, &bw, &s, &e);
		if (bw > span)
			span = bw;
	}
	return span;
}

int main(int argc, char *argv[])
{
	static calibrate_result_t result;
	double center_freq, bandwidth;
	int arfcn_start, arfcn_end;
	int samplerate;
	char start_info[256];
	int rc;

	logging_init();

	sdr_config_init();
	/* no LO offset (as before): calibration looks at the device itself */
	sdr_config->lo_offset = 0.0;
	sdr_config->lo_offset_given = 1;
	/* no IQ level counters (nobody shows them here, they cost a pass over
	 * every sample); --sdr-iq-status on to have them */
	sdr_config->iq_status = 0;
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

	int rate_given = sdr_config->samplerate != 0;
	samplerate = rate_given ? sdr_config->samplerate : 1000000;

	rc = sdr_configure(samplerate);
	if (rc < 0)
		return 1;
	if (rc == 0) {
		fprintf(stderr, "No SDR selected. Use --sdr-soapy or --sdr-uhd.\n");
		return 1;
	}

	const device_rule_t *dev = device_rule();
	int max_auto = dev->max_auto_rate;

	config.usable_fraction = dev->usable_fraction;
	config.usable_max_hz = dev->usable_max_hz;
	if (rate_given && dev->max_rate && samplerate > dev->max_rate) {
		fprintf(stderr, "--sdr-samplerate %d: above %.2f MS/s on %s: %s. Use %.2f MS/s or less.\n",
			samplerate, dev->max_rate / 1e6, dev->name, dev->why, dev->max_rate / 1e6);
		return 1;
	}
	if (rate_given) {
		snprintf(start_info, sizeof(start_info), "SDR rate %d S/s (given)", samplerate);
	} else {
		/* the widest the device and the bus carry; one ARFCN needs no more
		 * than 1 MS/s, a band no more than its span */
		sdr_rate_widest_t w;
		double want = (config.forced_arfcn > 0) ? 1e6 : wanted_span() / 0.8;

		/* the device's own rule, if any; else the widest, and the CPU
		 * budget picks a narrower one once the cost is measured */
		if (max_auto && want > max_auto)
			want = max_auto;
		if (config.forced_arfcn <= 0)
			config.reopen = reopen_sdr;

		rc = sdr_rx_widest_rate(500000, (int)want, &w);
		if (rc < 0) {
			fprintf(stderr, "No SDR rate: %s. Give one with --sdr-samplerate.\n", w.why);
			return 1;
		}
		samplerate = rc;
		sdr_config->samplerate = samplerate;
		if (!sdr_config->bandwidth_given)
			sdr_config->bandwidth = samplerate;
		snprintf(start_info, sizeof(start_info), "SDR opened at %d S/s, the widest (device up to %.1f MS/s, bus up to %.1f MS/s on USB %d Mbit/s: load %.0f %%)",
			 samplerate, w.device_max / 1e6, w.bus_max / 1e6, w.usb_speed, 100.0 * w.bus_load);
	}

	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);

	/* Use GSM-900 center for initial SDR open (will retune during scan) */
	gsm_band_params(GSM_BAND_900, &center_freq, &bandwidth,
	                &arfcn_start, &arfcn_end);

	if (config.verbosity >= 1 && !json_output)
		printf("osmocalibrate - SDR clock calibration, %s, %s\n",
		       (config.gsm_bands == GSM_BAND_ALL) ? "all GSM bands" : gsm_band_name(config.gsm_bands),
		       start_info);

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

	struct timespec t0, t1;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	rc = calibrate_auto(samplerate, &config, &result, start_info);
	clock_gettime(CLOCK_MONOTONIC, &t1);

	close_sdr();

	{
		struct rusage ru;
		getrusage(RUSAGE_SELF, &ru);
		double wall = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
		double cpu = ru.ru_utime.tv_sec + ru.ru_utime.tv_usec * 1e-6 +
			     ru.ru_stime.tv_sec + ru.ru_stime.tv_usec * 1e-6;
		LOGP(DSDR, LOGL_NOTICE, "Calibration took %.1f s, CPU %.2f s (%.1f%% of one core), result after %.1f s\n",
		     wall, cpu, (wall > 0.0) ? 100.0 * cpu / wall : 0.0, result.time_to_result);
	}

	double applied_ppm = sdr_config->rx_ppm;

	if (rc < 0 || !result.valid) {
		if (!json_output) {
			fprintf(stderr, "Calibration failed: %s\n", (rc == -EINVAL) ? "see above" :
				result.num_towers ? "the confirmed towers disagree, none agrees with another" : "no confirmed tower");
			for (int c = 0; c < result.num_cells; c++)
				print_cell_text(&result.cells[c], applied_ppm);
		} else {
			printf("{\"success\": false, \"error\": \"%s\", \"carriers\": [",
			       (rc == -EINVAL) ? "invalid setting" : result.num_towers ? "towers disagree" : "no confirmed tower");
			for (int c = 0; c < result.num_cells; c++)
				print_cell_json(&result.cells[c], applied_ppm, c > 0);
			printf("]}\n");
		}
		fflush(stdout);
		fflush(stderr);
		_exit(1);
	}

	double total_ppm = result.ppm_offset + applied_ppm;

	if (json_output) {
		printf("{\"success\": true, \"ppm\": %.6f, ", total_ppm);
		if (applied_ppm != 0.0)
			printf("\"applied_ppm\": %.6f, \"residual_ppm\": %.6f, ", applied_ppm, result.ppm_offset);
		printf("\"towers\": %d, \"agree\": %d, \"disagree\": %s, \"spread_ppm\": %.6f, ",
		       result.num_towers, result.num_agree, result.disagree ? "true" : "false", result.spread_ppm);
		printf("\"rx_gain_db\": %.1f, \"samplerate\": %d, \"channels_per_window\": %d, \"time_s\": %.1f, ",
		       sdr_config->rx_gain, result.samplerate, result.channels_per_window, result.time_to_result);
		printf("\"carriers\": [");
		for (int c = 0; c < result.num_cells; c++)
			print_cell_json(&result.cells[c], applied_ppm, c > 0);
		printf("]}\n");
	} else {
		printf("\n=== Calibration result ===\n");
		if (result.disagree)
			printf("  %.3f ppm from %d agreeing towers; %d tower(s) DISAGREE: spread %.3f ppm\n",
			       total_ppm, result.num_agree, result.disagree, result.spread_ppm);
		else
			printf("  %.3f ppm: %d tower%s agree%s (spread %.3f ppm)\n", total_ppm, result.num_agree,
			       (result.num_agree == 1) ? "" : "s", (result.num_agree == 1) ? "s" : "", result.spread_ppm);
		if (!config.full && config.forced_arfcn <= 0 && result.num_agree < config.towers)
			printf("  only %d of the %d agreeing towers asked for (--towers): %s\n",
			       result.num_agree, config.towers,
			       result.stopped_weak ? "the weaker carriers left out after a row without one (--full measures all)" : "no more confirmed carriers found");
		if (applied_ppm != 0.0)
			printf("  (residual %+.3f with --sdr-ppm %.3f applied)\n", result.ppm_offset, applied_ppm);
		for (int c = 0; c < result.num_cells; c++)
			print_cell_text(&result.cells[c], applied_ppm);
		printf("  Levels in dBFS at RX gain %.1f dB.\n", sdr_config->rx_gain);
		printf("\nTo apply this correction:\n");
		printf("  --sdr-ppm %.1f\n", total_ppm);
	}

	fflush(stdout);
	fflush(stderr);
	_exit(0);
}

/* Stub functions required by linked libraries */
sender_t *get_sender_by_empfangsfrequenz(double __attribute__((unused)) freq) { return NULL; }
void osmo_cc_set_log_cat(int __attribute__((unused)) cc_log_cat) {}
