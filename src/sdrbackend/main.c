/* osmo-sdr-backend: one radio, the programs register their channels on it
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * docs/SDR-BACKEND.md: the backend owns the device (the usual --sdr-...
 * options), tuned to the centre frequencies given; its device threads run
 * the filter banks. Programs started with --sdr-backend <socket> register
 * their channels on it instead of opening a device.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include "../liblogging/logging.h"
#include "../liboptions/options.h"
#include "../libsample/sample.h"
#include <osmocom/cc/misc.h>
#include "../libsdr/sdr_config.h"
#include "../libsdr/sdr.h"
#include "../libsdr/sdr_engine.h"
#include "../libsdr/sdr_backend.h"
#include "../libmobile/sender.h"

#define DEFAULT_LO_OFFSET	-10000.0

int use_sdr = 0;
static const char *socket_name = "@osmocom-sdr-backend";
static double tx_centre = 0.0, rx_centre = 0.0;
static int dsp_rate = 48000;
static int nmax = 8;
static int buffer_ms = 50;
static volatile int quit = 0;

static void print_help(const char *arg0)
{
	printf("Usage: %s --tx <MHz> --rx <MHz> --sdr-uhd | --sdr-soapy ... [options]\n", arg0);
	printf("\nOne radio for several programs: they register their channels with\n");
	printf("--sdr-backend <socket> instead of opening a device (docs/SDR-BACKEND.md).\n\n");
	printf(" -h --help\n");
	printf("        This help\n");
	logging_print_help();
	printf("    --tx <MHz>\n");
	printf("        TX centre frequency (the programs' TX channels around it)\n");
	printf("    --rx <MHz>\n");
	printf("        RX centre frequency (the programs' RX channels around it)\n");
	printf("    --socket <name>\n");
	printf("        Socket the programs connect to, '@name' in the abstract namespace\n");
	printf("        (default = %s)\n", socket_name);
	printf("    --dsp-rate <Hz>\n");
	printf("        Highest DSP rate (-s) of the programs (default = %d)\n", dsp_rate);
	printf("    --channels <n>\n");
	printf("        Most TX channels at once; each gets 1/n of full scale, so the sum\n");
	printf("        never clips (default = %d)\n", nmax);
	printf("    --buffer <ms>\n");
	printf("        Device block budget (default = %d)\n", buffer_ms);
	sdr_config_print_help();
}

#define OPT_TX		256
#define OPT_RX		257
#define OPT_SOCKET	258
#define OPT_DSP		259
#define OPT_CHANNELS	260
#define OPT_BUFFER	261

static void add_options(void)
{
	option_add('h', "help", 0);
	option_add('v', "verbose", 1);
	option_add(OPT_TX, "tx", 1);
	option_add(OPT_RX, "rx", 1);
	option_add(OPT_SOCKET, "socket", 1);
	option_add(OPT_DSP, "dsp-rate", 1);
	option_add(OPT_CHANNELS, "channels", 1);
	option_add(OPT_BUFFER, "buffer", 1);
	sdr_config_add_options();
}

static int handle_options(int short_option, int argi, char **argv)
{
	int rc;

	switch (short_option) {
	case 'h':
		print_help(argv[0]);
		return 0;
	case 'v':
		rc = parse_logging_opt(argv[argi]);
		if (rc > 0)
			return 0;
		if (rc < 0) {
			fprintf(stderr, "Failed to parse logging option, please use -h for help.\n");
			return rc;
		}
		break;
	case OPT_TX:
		tx_centre = atof(argv[argi]) * 1e6;
		break;
	case OPT_RX:
		rx_centre = atof(argv[argi]) * 1e6;
		break;
	case OPT_SOCKET:
		socket_name = options_strdup(argv[argi]);
		break;
	case OPT_DSP:
		dsp_rate = atoi(argv[argi]);
		break;
	case OPT_CHANNELS:
		nmax = atoi(argv[argi]);
		break;
	case OPT_BUFFER:
		buffer_ms = atoi(argv[argi]);
		break;
	default:
		return sdr_config_handle_options(short_option, argi, argv);
	}
	return 1;
}

static void sighandler(int sigset)
{
	if (sigset == SIGPIPE)
		return;
	quit = 1;
}

int main(int argc, char *argv[])
{
	void *sdr;
	sdr_engine_t *e;
	sdr_backend_t *b;
	int rc, argi, groups = -1;

	logging_init();
	sdr_config_init(DEFAULT_LO_OFFSET);
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/sdrbackend/sdrbackend.conf", handle_options);
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;
	if (sdr_config->backend) {
		fprintf(stderr, "--sdr-backend is for the programs, the backend opens the device itself.\n");
		return 1;
	}
	rc = sdr_configure(dsp_rate);
	if (rc <= 0) {
		fprintf(stderr, "Select the device (--sdr-uhd or --sdr-soapy ...), use '-h' for help.\n");
		return 1;
	}
	if (tx_centre == 0.0 && rx_centre == 0.0) {
		fprintf(stderr, "Give the centre frequencies (--tx and / or --rx), use '-h' for help.\n");
		return 1;
	}
	if (dsp_rate <= 0 || nmax < 1 || nmax > SDR_ENGINE_MAX_CHAN || buffer_ms < 5) {
		fprintf(stderr, "--dsp-rate, --channels (1..%d) or --buffer (>= 5 ms) out of range.\n", SDR_ENGINE_MAX_CHAN);
		return 1;
	}

	sdr_server_mode(nmax);
	sdr = sdr_open(0, NULL, (tx_centre != 0.0) ? &tx_centre : NULL, (rx_centre != 0.0) ? &rx_centre : NULL, NULL, 0, 0.0,
		       dsp_rate, dsp_rate * buffer_ms / 1000, 1.0, 0.0, 0.0, 1.0);
	if (!sdr) {
		fprintf(stderr, "Failed to open the SDR, use '-h' for help.\n");
		return 1;
	}
	e = sdr_server_engine(sdr);
	b = e ? sdr_backend_start(e, socket_name, rx_centre, tx_centre) : NULL;
	if (!b) {
		fprintf(stderr, "SDR backend not started: %s\n", e ? sdr_backend_error() : "no engine");
		sdr_close(sdr);
		return 1;
	}
	if (sdr_start(sdr) < 0) {
		fprintf(stderr, "Failed to start the SDR.\n");
		sdr_backend_stop(b);
		sdr_close(sdr);
		return 1;
	}
	signal(SIGINT, sighandler);
	signal(SIGHUP, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGPIPE, sighandler);
	LOGP(DSDR, LOGL_NOTICE, "SDR backend on '%s': up to %d TX channels at DSP rates up to %d. CTRL+c to end.\n", socket_name, nmax, dsp_rate);

	while (!quit) {
		int g = sdr_backend_groups(b);

		if (g != groups) {
			LOGP(DSDR, LOGL_NOTICE, "%d program(s) registered.\n", g);
			groups = g;
		}
		usleep(100000);
	}

	LOGP(DSDR, LOGL_NOTICE, "Ending: the programs lose their channels.\n");
	sdr_backend_stop(b);
	sdr_close(sdr);
	options_free();
	return 0;
}

/* used by libsdr, there are no senders here */
void osmo_cc_set_log_cat(int __attribute__((unused)) cc_log_cat) {}
sender_t *get_sender_by_empfangsfrequenz(double __attribute__((unused)) freq) { return NULL; }
