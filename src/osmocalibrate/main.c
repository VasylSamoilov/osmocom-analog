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
#include "../libcalibrate/lte_bands.h"
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
	printf("\nSDR clock calibration against GSM and LTE base stations: the GSM FCCH\n");
	printf("or the LTE reference signals give the offset, their decoded identity\n");
	printf("(GSM SCH and BCCH, LTE MIB) proves they are cells.\n\n");
	printf("Calibration options:\n");
	printf("  -h, --help              Show this help\n");
	printf("  -v, --verbose <level>   Set verbosity (default: 5)\n");
	printf("  -q, --quiet             Quiet mode (only output result)\n");
	printf("      --band <list>       Bands, comma separated (default 900):\n");
	printf("                          GSM (3GPP TS 45.005): 450, 480, 750, 850, 900 (P-GSM),\n");
	printf("                          e900, r900, 1800, 1900; gsm = all of them;\n");
	printf("                          LTE (3GPP TS 36.101): lte1 .. lte88; lte = all;\n");
	printf("                          all = every GSM and LTE band the SDR reaches.\n");
	printf("                          A band named that the SDR (with --sdr-rx-upconverter)\n");
	printf("                          does not reach is refused.\n");
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
	printf("  %s --sdr-uhd --band all          Every GSM and LTE band in reach\n", arg0);
	printf("  %s --sdr-uhd --band 900,lte3,lte8\n", arg0);
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

/* the bands asked for: explicit ones are refused when out of reach, the
 * ones of 'all' / 'gsm' / 'lte' are skipped */
typedef struct band_req {
	int	system, id, explicit;
} band_req_t;
static band_req_t req[CALIB_MAX_BANDS];
static int num_req = 0, band_given = 0;

static void req_add(int system, int id, int explicit)
{
	for (int i = 0; i < num_req; i++)
		if (req[i].system == system && req[i].id == id) {
			req[i].explicit |= explicit;
			return;
		}
	if (num_req < CALIB_MAX_BANDS) {
		req[num_req].system = system;
		req[num_req].id = id;
		req[num_req].explicit = explicit;
		num_req++;
	}
}

static void req_all(int system)
{
	if (system == CALIB_GSM)
		for (int i = 0; gsm_band_at(i); i++)
			req_add(CALIB_GSM, gsm_band_at(i)->id, 0);
	else
		for (int i = 0; lte_bands[i].band; i++)
			req_add(CALIB_LTE, lte_bands[i].band, 0);
}

/* --band: a list (comma separated) of 450 480 750 850 900 e900 r900 1800
 * 1900, lte<n>, gsm (every GSM band), lte (every LTE band), all */
static int parse_bands(const char *str)
{
	char buf[512], *tok, *save = NULL;

	if (!band_given)
		num_req = 0;
	band_given = 1;
	snprintf(buf, sizeof(buf), "%s", str);
	for (tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
		int id;

		if (!strcasecmp(tok, "all")) {
			req_all(CALIB_GSM);
			req_all(CALIB_LTE);
		} else if (!strcasecmp(tok, "gsm"))
			req_all(CALIB_GSM);
		else if (!strcasecmp(tok, "lte"))
			req_all(CALIB_LTE);
		else if (!strncasecmp(tok, "lte", 3) && tok[3]) {
			id = atoi(tok + 3);
			if (!lte_band_get(id)) {
				fprintf(stderr, "Unknown LTE band: %s (3GPP TS 36.101 Table 5.5-1)\n", tok);
				return -1;
			}
			req_add(CALIB_LTE, id, 1);
		} else if ((id = gsm_band_parse(tok)) > 0)
			req_add(CALIB_GSM, id, 1);
		else {
			fprintf(stderr, "Unknown band: %s (use 450, 480, 750, 850, 900, e900, r900, 1800, 1900, gsm, lte<n>, lte, all; a list with commas)\n", tok);
			return -1;
		}
	}
	return 0;
}

static int compare_req(const void *a, const void *b)
{
	const band_req_t *x = a, *y = b;
	int ix = 0, iy = 0;

	if (x->system != y->system)
		return x->system - y->system;	/* GSM first: its towers give LTE the prior */
	if (x->system == CALIB_LTE)
		return x->id - y->id;
	for (int i = 0; gsm_band_at(i); i++) {
		if (gsm_band_at(i)->id == x->id)
			ix = i;
		if (gsm_band_at(i)->id == y->id)
			iy = i;
	}
	return ix - iy;
}

static const char *band_label(int system, int id)
{
	static char s[32];

	if (system == CALIB_GSM)
		return gsm_band_name(id);
	snprintf(s, sizeof(s), "LTE band %d", id);
	return s;
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
		if (parse_bands(argv[argi]) < 0)
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
#define LTE_RATE_SDR 1920000

/* where the SDR opens: the first band's centre (the windows retune) */
static double open_centre;

static int reopen_sdr(int samplerate)
{
	close_sdr();
	sdr_config->samplerate = samplerate;
	if (!sdr_config->bandwidth_given)
		sdr_config->bandwidth = samplerate;
	return open_sdr_rx(open_centre, samplerate);
}

static double lte_bw_mhz(int n_rb)
{
	switch (n_rb) {
	case 6: return 1.4;
	case 15: return 3;
	case 25: return 5;
	case 50: return 10;
	case 75: return 15;
	case 100: return 20;
	}
	return 0;
}

static void print_lte_text(const calibrate_cell_t *cell, double applied_ppm)
{
	char name[16];

	snprintf(name, sizeof(name), "LTE-%d", cell->gsm_band);
	printf("  %-8s EARFCN %5d %9.3f MHz  PCI %3d %s %4.1f MHz %d port%s %-12s", name, cell->arfcn, cell->freq_hz / 1e6,
	       cell->lte.pci, cell->lte.tdd ? "TDD" : "FDD", lte_bw_mhz(cell->lte.n_rb), cell->lte.ports,
	       (cell->lte.ports == 1) ? " " : "s", cell->confirmed ? "MIB" : "not confirmed");
	printf(" %+8.1f Hz %7.3f ppm%s  RSRP %6.1f dBFS RSRQ %5.1f dB SINR %5.1f dB%s%s\n", cell->hz_offset,
	       cell->ppm_offset + applied_ppm, cell->lte.timing ? " (timing)" : "", cell->lte.rsrp_dbfs, cell->lte.rsrq_db,
	       cell->lte.sinr_db, (cell->confirmed && cell->weight < 1.0) ? "  low weight" : "",
	       (cell->confirmed && !cell->agrees) ? "  disagrees" : "");
}

static void print_cell_text(const calibrate_cell_t *cell, double applied_ppm)
{
	const gsm_cell_ident_t *id = &cell->ident;
	char ident[96] = "", mccmnc[16] = "-", laci[40] = "-";

	if (cell->system == CALIB_LTE) {
		print_lte_text(cell, applied_ppm);
		return;
	}

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

	if (cell->system == CALIB_LTE) {
		printf("%s{\"system\": \"LTE\", \"band\": %d, \"earfcn\": %d, \"frequency\": %.0f, \"pci\": %d, "
		       "\"duplex\": \"%s\", \"bandwidth_mhz\": %.1f, \"ports\": %d, \"confirmed\": %s, "
		       "\"hz\": %.2f, \"stddev_hz\": %.3f, \"ppm\": %.6f, \"ppm_from\": \"%s\", \"carrier_ppm\": %.6f, "
		       "\"timing_ppm\": %.6f, \"timing_std_ppm\": %.6f, \"alias_margin\": %.1f, \"weight\": %.2f, "
		       "\"rsrp_dbfs\": %.1f, \"rsrq_db\": %.1f, \"sinr_db\": %.1f, \"agrees\": %s}",
		       comma ? ", " : "", cell->gsm_band, cell->arfcn, cell->freq_hz, cell->lte.pci, cell->lte.tdd ? "TDD" : "FDD",
		       lte_bw_mhz(cell->lte.n_rb), cell->lte.ports, cell->confirmed ? "true" : "false",
		       cell->hz_offset, cell->stddev_hz, cell->ppm_offset + applied_ppm, cell->lte.timing ? "timing" : "carrier",
		       cell->carrier_ppm + applied_ppm, cell->lte.timing_ppm, cell->lte.timing_std_ppm, cell->lte.alias_margin,
		       cell->weight, cell->lte.rsrp_dbfs, cell->lte.rsrq_db, cell->lte.sinr_db, cell->agrees ? "true" : "false");
		return;
	}
	printf("%s{\"system\": \"GSM\", \"band\": \"%s\", \"arfcn\": %d, \"frequency\": %.0f, \"measurements\": %d, "
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
	int		lte_timing;	/* LTE: the frame timing ppm (the LO is off its setting) */
} device_rule_t;

static const device_rule_t device_rules[] = {
	/* B200/B210: +-0.40 (power-of-2 decimation >= 4 of the master clock;
	 * above 15.36 MS/s the master clock is the rate, the AD9361 alone,
	 * +-0.40 with the LO centred, as here: no LO offset) */
	{ "UHD (B200/B210)", 1, NULL, 0.40, 0.0, 0, 0, NULL, 0 },
	/* RTL2832U: its FIR is flat to +-0.9 MHz at any rate; the R820T's PLL
	 * step is in the carrier offset (docs/SDR-FRONTENDS.md) */
	{ "RTL-SDR", 0, "rtlsdr", 0.40, 0.85e6, 0, 0, NULL, 1 },
	/* LMS7002M: clean to +-0.30 */
	{ "LimeSDR", 0, "lime", 0.30, 0.0, 0, 0, NULL, 0 },
	/* unknown */
	{ "this device", 0, NULL, 0.35, 0.0, 0, 0, NULL, 0 },
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

/* the band(s) to scan: the widest span */
static double wanted_span(void)
{
	double span = 0.0;

	for (int b = 0; b < config.num_bands; b++)
		if (config.bands[b].scan && config.bands[b].f_hi - config.bands[b].f_lo > span)
			span = config.bands[b].f_hi - config.bands[b].f_lo;
	return span + 0.4e6;
}

/* the bands asked for, in reach of the SDR: config.bands. Returns 0, <0 if
 * a band named is out of reach or none is in reach. */
static int bands_in_reach(void)
{
	sdr_caps_t caps;
	double dmin = 0.0, dmax = 0.0, up = sdr_config->rx_upconverter;
	int rx_only = sdr_config->rx_only, known, skipped = 0;

	/* the RX range only (the device opens once more for it) */
	sdr_config->rx_only = 1;
	if (sdr_query_caps(&caps) == 0) {
		dmin = caps.rx_freq_min;
		dmax = caps.rx_freq_max;
	}
	sdr_config->rx_only = rx_only;
	known = (dmax > 0.0);
	if (!known)
		LOGP(DSDR, LOGL_NOTICE, "The device's frequency range is not known: every band taken as in reach\n");
	else
		LOGP(DSDR, LOGL_INFO, "Device RX %.3f-%.3f MHz%s: carriers %.3f-%.3f MHz in reach\n", dmin / 1e6, dmax / 1e6,
		     up ? " with the upconverter" : "", (dmin - up) / 1e6, (dmax - up) / 1e6);
	qsort(req, num_req, sizeof(req[0]), compare_req);
	config.num_bands = 0;
	for (int i = 0; i < num_req; i++) {
		calib_band_t *cb = &config.bands[config.num_bands];
		double lo, hi, a, z;
		int f1, f2;

		if (req[i].system == CALIB_GSM)
			gsm_band_dl_range(req[i].id, &lo, &hi);
		else
			lte_band_dl_range(req[i].id, &f1, &f2, &lo, &hi);
		a = lo;
		z = hi;
		if (known) {
			/* the device tunes to the carrier + the upconverter */
			if (a + up < dmin)
				a = dmin - up;
			if (z + up > dmax)
				z = dmax - up;
		}
		if (a > z) {
			if (req[i].explicit) {
				fprintf(stderr, "%s (%.1f-%.1f MHz) is out of reach: the device receives %.1f-%.1f MHz%s.\n",
					band_label(req[i].system, req[i].id), lo / 1e6, hi / 1e6, dmin / 1e6, dmax / 1e6,
					up ? " (with --sdr-rx-upconverter: carriers in reach shifted by it)" : "");
				return -EINVAL;
			}
			LOGP(DSDR, LOGL_INFO, "%s (%.1f-%.1f MHz) skipped: out of the device's %.1f-%.1f MHz\n",
			     band_label(req[i].system, req[i].id), lo / 1e6, hi / 1e6, (dmin - up) / 1e6, (dmax - up) / 1e6);
			skipped++;
			continue;
		}
		if (a > lo || z < hi)
			LOGP(DSDR, LOGL_NOTICE, "%s (%.1f-%.1f MHz): only %.1f-%.1f MHz in the device's reach\n",
			     band_label(req[i].system, req[i].id), lo / 1e6, hi / 1e6, a / 1e6, z / 1e6);
		cb->system = req[i].system;
		cb->id = req[i].id;
		cb->f_lo = a;
		cb->f_hi = z;
		cb->scan = 1;
		config.num_bands++;
	}
	if (skipped)
		LOGP(DSDR, LOGL_NOTICE, "%d bands out of the device's reach (%.1f-%.1f MHz) skipped\n", skipped,
		     (dmin - up) / 1e6, (dmax - up) / 1e6);
	if (!config.num_bands) {
		fprintf(stderr, "No band asked for is in the device's reach (%.1f-%.1f MHz).\n", (dmin - up) / 1e6, (dmax - up) / 1e6);
		return -EINVAL;
	}
	/* a GSM band inside another of the run is not measured again; it still
	 * names its carriers (P-GSM in E-GSM). LTE bands each get their own
	 * peaks (band 7 inside 41 would drown in the wider band's), the
	 * spectrum is scanned once and an EARFCN searched once anyway. */
	for (int i = 0; i < config.num_bands; i++)
		for (int j = 0; j < config.num_bands; j++) {
			calib_band_t *x = &config.bands[i], *y = &config.bands[j];

			if (i == j || !y->scan || x->system != y->system || x->system != CALIB_GSM)
				continue;
			if (x->f_lo >= y->f_lo && x->f_hi <= y->f_hi && (x->f_lo != y->f_lo || x->f_hi != y->f_hi || j < i)) {
				x->scan = 0;
				break;
			}
		}
	return 0;
}

/* the rate holds one carrier of every system asked for */
static int rate_fits(int samplerate, const device_rule_t *dev)
{
	double half = dev->usable_fraction * samplerate;
	int gsm = 0, lte = 0;

	if (dev->usable_max_hz > 0.0 && half > dev->usable_max_hz)
		half = dev->usable_max_hz;
	if (half > 0.4 * samplerate)
		half = 0.4 * samplerate;
	for (int b = 0; b < config.num_bands; b++) {
		if (config.bands[b].system == CALIB_GSM)
			gsm = 1;
		else
			lte = 1;
	}
	/* GSM: a 200 kHz channel off the DC (calibrate.c window_capacity) */
	if (gsm && half < 300e3) {
		fprintf(stderr, "%d S/s on %s: +-%.0f kHz usable, a GSM channel needs +-300 kHz. Use a wider --sdr-samplerate.\n",
			samplerate, dev->name, half / 1e3);
		return -EINVAL;
	}
	if (lte && half < CALIB_LTE_HALF_HZ) {
		fprintf(stderr, "%d S/s on %s: +-%.0f kHz usable, an LTE carrier's centre 6 RB need +-%.0f kHz. Use a wider --sdr-samplerate.\n",
			samplerate, dev->name, half / 1e3, CALIB_LTE_HALF_HZ / 1e3);
		return -EINVAL;
	}
	return 0;
}

int main(int argc, char *argv[])
{
	static calibrate_result_t result;
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

	if (!band_given)
		req_add(CALIB_GSM, GSM_BAND_900, 1);
	if (config.forced_arfcn > 0 && (num_req != 1 || req[0].system != CALIB_GSM)) {
		fprintf(stderr, "--arfcn requires one GSM --band\n");
		return 1;
	}
	if (config.forced_arfcn > 0) {
		double ff = arfcn_to_freq(config.forced_arfcn, req[0].id);
		if (ff == 0.0) {
			fprintf(stderr, "ARFCN %d is invalid for selected band %s\n",
			        config.forced_arfcn, gsm_band_name(req[0].id));
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
	/* the RTL's R820T lands off the LO set (PLL step, up to +-0.5 ppm at
	 * 900 MHz, per frequency): its carrier offsets carry that; the LTE
	 * frame timing is the crystal only */
	config.lte_timing = dev->lte_timing;
	if (bands_in_reach() < 0)
		return 1;
	if (config.forced_arfcn > 0) {
		double ff = arfcn_to_freq(config.forced_arfcn, req[0].id);

		if (ff < config.bands[0].f_lo - 1.0 || ff > config.bands[0].f_hi + 1.0) {
			fprintf(stderr, "ARFCN %d (%.1f MHz) is out of the device's reach\n", config.forced_arfcn, ff / 1e6);
			return 1;
		}
		config.bands[0].f_lo = config.bands[0].f_hi = ff;
	}
	for (int b = 0; b < config.num_bands; b++)
		if (config.bands[b].scan) {
			open_centre = 0.5 * (config.bands[b].f_lo + config.bands[b].f_hi);
			break;
		}
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

		/* with LTE: rates of 1.92 MS/s x 2^k (LTE's rate: no resampler
		 * for its carriers, measured 26 % less separation than at
		 * 8 MS/s; GSM then at 480 kS/s per channel) */
		int has_lte = 0;

		for (int b = 0; b < config.num_bands; b++)
			if (config.bands[b].system == CALIB_LTE)
				has_lte = 1;
		rc = sdr_rx_widest_rate(has_lte ? 480000 : 500000, (int)want, &w);
		if (rc < 0) {
			fprintf(stderr, "No SDR rate: %s. Give one with --sdr-samplerate.\n", w.why);
			return 1;
		}
		/* a device whose usable part is fixed in Hz (RTL: +-0.85 MHz)
		 * gains nothing above it: LTE's rate itself */
		if (has_lte && dev->usable_max_hz > 0.0 && rc > LTE_RATE_SDR && w.device_max >= LTE_RATE_SDR)
			rc = LTE_RATE_SDR;
		samplerate = rc;
		sdr_config->samplerate = samplerate;
		if (!sdr_config->bandwidth_given)
			sdr_config->bandwidth = samplerate;
		snprintf(start_info, sizeof(start_info), "SDR opened at %d S/s, the widest (device up to %.1f MS/s, bus up to %.1f MS/s on USB %d Mbit/s: load %.0f %%)",
			 samplerate, w.device_max / 1e6, w.bus_max / 1e6, w.usb_speed, 100.0 * w.bus_load);
	}

	if (rate_fits(samplerate, dev) < 0)
		return 1;

	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);

	if (config.verbosity >= 1 && !json_output) {
		char list[512] = "";
		int nb = 0;

		for (int b = 0; b < config.num_bands; b++) {
			if (nb++ < 12)
				snprintf(list + strlen(list), sizeof(list) - strlen(list), "%s%s", nb > 1 ? ", " : "",
					 band_label(config.bands[b].system, config.bands[b].id));
		}
		if (nb > 12)
			snprintf(list + strlen(list), sizeof(list) - strlen(list), " and %d more", nb - 12);
		printf("osmocalibrate - SDR clock calibration, %s, %s\n", list, start_info);
		if (config.lte_timing)
			printf("  (%s: LTE ppm from the frame timing, the crystal alone; its carrier offset carries the tuner's PLL step)\n",
			       dev->name);
	}

	rc = open_sdr_rx(open_centre, samplerate);
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
