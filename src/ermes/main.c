/* ERMES (European Radio Message System) main
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
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

#define _GNU_SOURCE	/* strptime */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/main_mobile.h"
#include "../liboptions/options.h"
#include "../libfm/fm.h"
#include "../libmobile/msg_fifo.h"
#include "ermes.h"

#define MSG_SEND_DEFAULT "/tmp/ermes_msg_send"
#define MAX_START_PAGES 16

extern const char *ermes_default_text;
static const char *book_file = NULL;

static const char *msg_send_path = MSG_SEND_DEFAULT;
static const char *msg_send_file = NULL;
static msg_fifo_t msg_fifo;

/* per channel, in the order of -k: polarity (-P) and frequency subset indicator (--fsi) */
static double polarity[MAX_SENDER];
static int num_polarity = 0;
static int fsi[MAX_SENDER];
static int num_fsi = 0;
/* Default network: the factory setting of NEC ERMES pagers (ERM1BV2 ROM map: zone 2, country 1,
 * operator 2, paging area 0), so a pager as delivered answers without options. */
#define DEFAULT_ZONE		2
#define DEFAULT_COUNTRY		1
#define DEFAULT_OPERATOR	2
#define DEFAULT_PA		0
static ermes_network_t net = { .zone = DEFAULT_ZONE, .country = DEFAULT_COUNTRY, .operator = DEFAULT_OPERATOR,
			       .pa = DEFAULT_PA, .bai = 0, .fsi = ERMES_FSI_ALL };
static const char *start_pages[MAX_START_PAGES];
static int num_start_pages = 0;
static double page_delay = 30.0;
static double start_time;

#define OPT_ZONE	256
#define OPT_COUNTRY	257
#define OPT_OPERATOR	258
#define OPT_PA		259
#define OPT_FSI		260
#define OPT_FIFO	261
#define OPT_FIFO_FILE	262
#define OPT_PAGE	263
#define OPT_PAGE_DELAY	264
#define OPT_BOOK	265
#define OPT_CHARSET	266
#define OPT_CHARSET_PRINT 267
#define OPT_SSI		268
#define OPT_CLOCK	269
#define OPT_SSI_HOLD	270
#define OPT_URGENT	271
#define OPT_ALERT	272
#define OPT_TX_LATENCY	273
#define OPT_YEAR_FROM	274
#define OPT_PRIORITY	275
#define OPT_RETRANSMIT	276
#define OPT_RETRANSMIT_INTERVAL 277
#define OPT_RECEIVER_BATCH 278
#define OPT_GROUP_FIG16	279

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "-k 1..16 | -k list ");
	/*      -                                                                             - */
	printf(" -k --kanal 1..%d\n", ERMES_CHANNELS);
	printf("        ERMES channel n: %.3f + n * %.3f MHz (ETS 300 133-4 ed.2 8.2).\n",
	       ERMES_CHANNEL_BASE_HZ / 1e6, ERMES_CHANNEL_STEP_HZ / 1e6);
	printf("        Edition 1 (1992) and early pagers count from 0 at %.3f MHz: their\n",
	       (ERMES_CHANNEL_BASE_HZ + ERMES_CHANNEL_STEP_HZ) / 1e6);
	printf("        channel 8 is -k 9 here (a NEC PA3E6-1A home channel 8 = 169.625 MHz).\n");
	printf("        Several -k: one network on several channels, one clock, one queue; each\n");
	printf("        channel carries the pages of the receivers of its frequency subset (--fsi).\n");
	printf(" -P --polarity normal | inverted\n");
	printf("        Normal: dibit 10 = +%.1f Hz (ETS 300 133-4 Table 17). Once per -k, in\n", ERMES_DEVIATION_HZ);
	printf("        the same order (one: all channels). The standard has normal only; inverted\n");
	printf("        is for a radio path that inverts. (default normal)\n");
	printf("    --zone 0..%d --country 0..%d --operator 0..%d\n",
	       (1 << ERMES_ZONE_BITS) - 1, (1 << ERMES_COUNTRY_BITS) - 1, (1 << ERMES_OPERATOR_BITS) - 1);
	printf("        The network, sent in the system information: the receivers' home\n");
	printf("        network. (default zone %d, country %d, operator %d: the factory\n", net.zone, net.country, net.operator);
	printf("        setting of NEC ERMES pagers)\n");
	printf("    --pa 0..%d\n", (1 << ERMES_PA_BITS) - 1);
	printf("        Paging area. (default %d)\n", net.pa);
	printf("    --fsi 0..%d\n", ERMES_FSI_MAX);
	printf("        Frequency subset indicator of a channel (ETS 300 133-4 9.2 Fig.18): the\n");
	printf("        receivers' frequency subset numbers (FSN, fsn= in the book) it carries:\n");
	printf("        0..15 one FSN, 16..23 two, 24..27 four, 28..29 eight, %d all. Once per -k,\n", ERMES_FSI_ALL);
	printf("        in the same order; the channels' subsets must not overlap. (default: one\n");
	printf("        channel %d, two channels 29 and 28 as the standard says; more: give them)\n", ERMES_FSI_ALL);
	printf("    --ssi hold | <test mode>\n");
	printf("        Supplementary system information (time and date).\n");
	printf("        hold (default): the network schedule. The hour starts with zone/hour/date\n");
	printf("        in every batch of its first subsequence, day/month/year in the next, and\n");
	printf("        so on by turns; continuous over hours and restarts. This is what the Swiss\n");
	printf("        (1997) and Dutch (2014) networks sent.\n");
	printf("        Test modes, to study receivers (not for a network):\n");
	printf("          alternate       zone/hour/date and day/month/year by turns every batch\n");
	printf("          strict, pairs   0000 0001 ... / 0000 0000 0001 0001 ... every batch\n");
	printf("          cycle           the 13 types of edition 2 5.3.2.1, one a batch (network\n");
	printf("                          version 02.01, reserved ones zero): edition 1 pagers\n");
	printf("                          cannot use it\n");
	printf("          zone-hour-date, day-month-year   only that type in every batch\n");
	printf("    --ssi-hold <subsequences>\n");
	printf("        With --ssi hold: subsequences each type is held. (default 1, as the real\n");
	printf("        networks; %d = one minute, for tests)\n", ERMES_SUBSEQUENCES);
	printf("    --year-from <year>\n");
	printf("        Send the year (SSI day/month/year) moved by whole 28-year steps into\n");
	printf("        <year> .. <year>+27, for pagers whose calendar ends early: the weekday\n");
	printf("        and the date stay right (1901-2099). E.g. 1995: 2026 is sent as 1998.\n");
	printf("        (default: the real year)\n");
	printf("    --tx-latency <ms>\n");
	printf("        Time from making a sample to sending it, so the batches are locked to\n");
	printf("        UTC on the air. (default: the DSP buffer, -b)\n");
	printf("    --clock \"YYYY-MM-DD HH:MM:SS\"\n");
	printf("        Run the network clock from this local time at start instead of the\n");
	printf("        system clock: the minute (cycle), subsequences and the time, date and\n");
	printf("        year in the SSI follow it. (default: the system clock)\n");
	printf(" -y --type tone | numeric | alpha\n");
	printf("        Type of pages without a type. (default %s)\n",
	       ermes_category_name(ermes_default_opts.category));
	printf("    --urgent\n");
	printf("        Pages are urgent (UMI, Table 9) unless their line says urgent=0.\n");
	printf("    --alert 0..%d\n", (1 << ERMES_ALERT_BITS) - 1);
	printf("        Alert type of pages whose line gives none (alert=). (default %d)\n", ermes_default_opts.alert);
	printf("    --priority %d..%d\n", ERMES_PRIORITY_HIGH, ERMES_PRIORITY_LOW);
	printf("        Priority of pages without one (ETS 300 133-2 4.3.2): %d goes before the\n", ERMES_PRIORITY_HIGH);
	printf("        others, %d is normal, %d after them (a batch takes them in this order).\n", ERMES_PRIORITY_NORMAL, ERMES_PRIORITY_LOW);
	printf("        Only the queue order, nothing on the air. (default %d)\n", ermes_default_opts.priority);
	printf("    --retransmit 0..%d\n", ERMES_RETRANSMIT_MAX);
	printf("        Times a page without one is sent again after the first, with the same\n");
	printf("        message number (ETS 300 133-2 4.3.1.3.1). (default %d)\n", ermes_default_opts.retransmit);
	printf("    --retransmit-interval 1..%d\n", ERMES_RETRANSMIT_INTERVAL_MAX_S);
	printf("        Seconds between the transmissions. (default %d)\n", ermes_default_opts.retransmit_interval);
	printf("    --receiver-batch 1..%d\n", ERMES_RECEIVER_BATCH_MAX);
	printf("        Messages to one receiver in one batch at most; more go in its next batch,\n");
	printf("        in order. The standard allows any number. (default %d)\n", ERMES_RECEIVER_BATCH_DEFAULT);
	printf("    --group-fig16\n");
	printf("        A group call's pointer goes with the member's initial address in its own\n");
	printf("        batch (ETS 300 133-5 7.8 Fig.16), not alone in the batch after.\n");
	printf(" -M --message \"...\"\n");
	printf("        Text of pages without a text (console, FIFO lines without text).\n");
	printf("    --charset <vendor-model-charset> | list\n");
	printf("        Codepage of alphanumeric messages. (default %s-%s-%s)\n",
	       ermes_charset->vendor, ermes_charset->model, ermes_charset->charset);
	printf("    --charset-print <vendor-model-charset>\n");
	printf("        Show the table of a codepage.\n");
	printf("    --page <address>,<type>,<options>,<message>\n");
	printf("        Queue this page --page-delay seconds after start (may be repeated).\n");
	printf("        The address is the %d-bit local address, decimal or 0x hex, or\n", ERMES_LA_BITS);
	printf("        nec-<code> as NEC shows it (\"Tokyo hex\", the local address * %d).\n", 1 << ERMES_NEC_SHIFT);
	printf("    --page-delay <seconds>\n");
	printf("        Let the receiver find the channel first. (default %.0f)\n", page_delay);
	printf("    --book <file>\n");
	printf("        Subscriber book: one receiver per line, <local address> key=value ...\n");
	printf("        number=1..31: the last message number, kept across runs (a receiver\n");
	printf("        reports missing messages when the numbers start again); fsn=0..15: the\n");
	printf("        receiver's frequency subset number (see --fsi). The program rewrites it.\n");
	printf("        Without it, the numbers of an earlier ermes_numbers file are taken over.\n");
	printf("        (default ~/.osmocom/analog/ermes_book)\n");
	printf("    --fifo <path>\n");
	printf("        Named pipe for pages, one per line as --page. (default %s)\n", MSG_SEND_DEFAULT);
	printf("    FIFO protocol (as flex and pocsag):\n");
	printf("        Format: address,type,options,message\n");
	printf("          address  local address (decimal or 0x hex) or nec-<label code>;\n");
	printf("                   tempgroup:<address> <address> ... : a group call, the\n");
	printf("                   message sent once through a common temporary address\n");
	printf("          type     tone|numeric|alpha (empty: -y)\n");
	printf("          options  space-separated key=value pairs:\n");
	printf("                   priority=1-3  urgent=0|1  alert=0-7\n");
	printf("                   charset=<vendor-model-charset>\n");
	printf("                   retransmit=0-%d  retransmit_interval=1-%d (seconds)\n",
	       ERMES_RETRANSMIT_MAX, ERMES_RETRANSMIT_INTERVAL_MAX_S);
	printf("          message  the rest of the line, commas included\n");
	printf("        Example: nec-C63900,alpha,priority=1 urgent=1 alert=3,Call home, now\n");
	printf("        A page gets its message number when it is sent, so the numbers of an\n");
	printf("        address go up on the air whatever the priorities.\n");
	printf("    --fifo-file <file>\n");
	printf("        Read the page lines from this file instead, then end when all are sent.\n");
	main_mobile_print_station_id();
	main_mobile_print_hotkeys();
}

static void add_options(void)
{
	main_mobile_add_options();
	option_add('P', "polarity", 1);
	option_add(OPT_ZONE, "zone", 1);
	option_add(OPT_COUNTRY, "country", 1);
	option_add(OPT_OPERATOR, "operator", 1);
	option_add(OPT_PA, "pa", 1);
	option_add(OPT_FSI, "fsi", 1);
	option_add('y', "type", 1);
	option_add(OPT_URGENT, "urgent", 0);
	option_add(OPT_ALERT, "alert", 1);
	option_add(OPT_PRIORITY, "priority", 1);
	option_add(OPT_RETRANSMIT, "retransmit", 1);
	option_add(OPT_RETRANSMIT_INTERVAL, "retransmit-interval", 1);
	option_add(OPT_RECEIVER_BATCH, "receiver-batch", 1);
	option_add(OPT_GROUP_FIG16, "group-fig16", 0);
	option_add(OPT_TX_LATENCY, "tx-latency", 1);
	option_add(OPT_YEAR_FROM, "year-from", 1);
	option_add('M', "message", 1);
	option_add(OPT_PAGE, "page", 1);
	option_add(OPT_PAGE_DELAY, "page-delay", 1);
	option_add(OPT_BOOK, "book", 1);
	option_add(OPT_CHARSET, "charset", 1);
	option_add(OPT_CHARSET_PRINT, "charset-print", 1);
	option_add(OPT_SSI, "ssi", 1);
	option_add(OPT_CLOCK, "clock", 1);
	option_add(OPT_SSI_HOLD, "ssi-hold", 1);
	option_add(OPT_FIFO, "fifo", 1);
	option_add(OPT_FIFO_FILE, "fifo-file", 1);
}

/* a field value of the given width: 0 .. 2^bits - 1 */
static int field_option(const char *name, const char *arg, int bits, int *value)
{
	char *end;
	long v = strtol(arg, &end, 0);
	long max = (1L << bits) - 1;

	if (end == arg || *end || v < 0 || v > max) {
		fprintf(stderr, "Given %s '%s' is not 0..%ld.\n", name, arg, max);
		return -EINVAL;
	}
	*value = v;
	return 1;
}

#define PROTOCOL	"ermes"

static int handle_options(int short_option, int argi, char **argv)
{
	switch (short_option) {
	case 'P':
		if (num_polarity == MAX_SENDER) {
			fprintf(stderr, "More than %d -P given.\n", MAX_SENDER);
			return -EINVAL;
		}
		if (argv[argi][0] == 'n' || argv[argi][0] == 'N')
			polarity[num_polarity++] = 1.0;
		else if (argv[argi][0] == 'i' || argv[argi][0] == 'I')
			polarity[num_polarity++] = -1.0;
		else {
			fprintf(stderr, "Given polarity is not 'normal' or 'inverted'.\n");
			return -EINVAL;
		}
		break;
	case OPT_ZONE:
		return field_option("zone", argv[argi], ERMES_ZONE_BITS, &net.zone);
	case OPT_COUNTRY:
		return field_option("country", argv[argi], ERMES_COUNTRY_BITS, &net.country);
	case OPT_OPERATOR:
		return field_option("operator", argv[argi], ERMES_OPERATOR_BITS, &net.operator);
	case OPT_PA:
		return field_option("PA", argv[argi], ERMES_PA_BITS, &net.pa);
	case OPT_FSI:
	{
		int count;

		if (num_fsi == MAX_SENDER) {
			fprintf(stderr, "More than %d --fsi given.\n", MAX_SENDER);
			return -EINVAL;
		}
		if (field_option("FSI", argv[argi], ERMES_FSI_BITS, &fsi[num_fsi]) < 0
		 || ermes_fsi_subset(fsi[num_fsi], &count) < 0) {
			fprintf(stderr, "Given FSI '%s' is not 0..%d.\n", argv[argi], ERMES_FSI_MAX);
			return -EINVAL;
		}
		num_fsi++;
		break;
	}
	case 'y':
		if (ermes_category_from_name(argv[argi], &ermes_default_opts.category) < 0
		 || ermes_default_opts.category == ERMES_CAT_TRANSPARENT) {
			fprintf(stderr, "Given type is not tone, numeric or alpha.\n");
			return -EINVAL;
		}
		break;
	case OPT_YEAR_FROM:
		ermes_year_from = atoi(argv[argi]);
		if (ermes_year_from < ERMES_YEAR_BASE || ermes_year_from + 27 > ERMES_YEAR_BASE + (1 << ERMES_YEAR_BITS) - 1) {
			fprintf(stderr, "Given year-from is not %d..%d.\n", ERMES_YEAR_BASE, ERMES_YEAR_BASE + (1 << ERMES_YEAR_BITS) - 28);
			return -EINVAL;
		}
		break;
	case OPT_TX_LATENCY:
		ermes_tx_latency = atof(argv[argi]) / 1000.0;
		break;
	case OPT_URGENT:
		ermes_default_opts.urgent = 1;
		break;
	case OPT_ALERT:
		return field_option("alert", argv[argi], ERMES_ALERT_BITS, &ermes_default_opts.alert);
	case OPT_PRIORITY:
	case OPT_RETRANSMIT:
	case OPT_RETRANSMIT_INTERVAL:
	{
		const char *key = (short_option == OPT_PRIORITY) ? "priority"
				: (short_option == OPT_RETRANSMIT) ? "retransmit" : "retransmit_interval";
		char opt[64];

		/* the same check as the FIFO option */
		snprintf(opt, sizeof(opt), "%s=%s", key, argv[argi]);
		if (ermes_parse_page_opts(opt, &ermes_default_opts) < 0)
			return -EINVAL;
		break;
	}
	case OPT_RECEIVER_BATCH:
		ermes_receiver_batch = atoi(argv[argi]);
		if (ermes_receiver_batch < 1 || ermes_receiver_batch > ERMES_RECEIVER_BATCH_MAX) {
			fprintf(stderr, "Given receiver-batch is not 1..%d.\n", ERMES_RECEIVER_BATCH_MAX);
			return -EINVAL;
		}
		break;
	case OPT_GROUP_FIG16:
		ermes_group_fig16 = 1;
		break;
	case OPT_SSI_HOLD:
		ermes_ssi_hold = atoi(argv[argi]);
		if (ermes_ssi_hold < 1) {
			fprintf(stderr, "Given SSI hold is not 1 or more subsequences.\n");
			return -EINVAL;
		}
		break;
	case OPT_CLOCK:
		{
			struct tm tm;
			const char *end;
			time_t t;

			memset(&tm, 0, sizeof(tm));
			end = strptime(argv[argi], "%Y-%m-%d %H:%M:%S", &tm);
			if (!end || *end) {
				fprintf(stderr, "Given clock '%s' is not \"YYYY-MM-DD HH:MM:SS\".\n", argv[argi]);
				return -EINVAL;
			}
			tm.tm_isdst = -1;
			t = mktime(&tm);
			ermes_clock_offset = (double)t - (double)time(NULL);
		}
		break;
	case OPT_SSI:
		if (!strcasecmp(argv[argi], "alternate"))
			ermes_ssi_mode = ERMES_SSI_ALTERNATE;
		else if (!strcasecmp(argv[argi], "cycle"))
			ermes_ssi_mode = ERMES_SSI_CYCLE;
		else if (!strcasecmp(argv[argi], "hold"))
			ermes_ssi_mode = ERMES_SSI_HOLD;
		else if (!strcasecmp(argv[argi], "strict"))
			ermes_ssi_mode = ERMES_SSI_STRICT;
		else if (!strcasecmp(argv[argi], "pairs"))
			ermes_ssi_mode = ERMES_SSI_PAIRS;
		else if (!strcasecmp(argv[argi], "zone-hour-date"))
			ermes_ssi_mode = ERMES_SSIT_ZONE_HOUR_DATE;
		else if (!strcasecmp(argv[argi], "day-month-year"))
			ermes_ssi_mode = ERMES_SSIT_DAY_MONTH_YEAR;
		else {
			fprintf(stderr, "Given SSI is not hold, or a test mode: alternate, cycle, strict, pairs, zone-hour-date, day-month-year.\n");
			return -EINVAL;
		}
		break;
	case OPT_CHARSET:
		if (!strcasecmp(argv[argi], "list")) {
			printf("ERMES codepages:\n");
			pager_charset_list_protocol(PROTOCOL);
			return 0;
		}
		ermes_charset = pager_charset_lookup(PROTOCOL, argv[argi]);
		if (!ermes_charset) {
			fprintf(stderr, "'%s' is not an ERMES codepage, use '--charset list'.\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_CHARSET_PRINT:
		{
			const struct pager_charset *cs = pager_charset_lookup(PROTOCOL, argv[argi]);

			if (!cs) {
				fprintf(stderr, "'%s' is not an ERMES codepage, use '--charset list'.\n", argv[argi]);
				return -EINVAL;
			}
			pager_charset_print(cs);
			return 0;
		}
	case 'M':
		ermes_default_text = options_strdup(argv[argi]);
		break;
	case OPT_PAGE:
		if (num_start_pages == MAX_START_PAGES) {
			fprintf(stderr, "Too many --page.\n");
			return -EINVAL;
		}
		start_pages[num_start_pages++] = options_strdup(argv[argi]);
		break;
	case OPT_PAGE_DELAY:
		page_delay = atof(argv[argi]);
		break;
	case OPT_BOOK:
		book_file = options_strdup(argv[argi]);
		break;
	case OPT_FIFO:
		msg_send_path = options_strdup(argv[argi]);
		break;
	case OPT_FIFO_FILE:
		msg_send_file = options_strdup(argv[argi]);
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}

static double now_s(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return tv.tv_sec + tv.tv_usec / 1e6;
}

static void myhandler(void)
{
	char *line;
	int i;

	if (num_start_pages && now_s() - start_time >= page_delay) {
		for (i = 0; i < num_start_pages; i++)
			ermes_msg_send(start_pages[i]);
		num_start_pages = 0;
	}

	msg_fifo_read(&msg_fifo);
	while ((line = msg_fifo_line(&msg_fifo)))
		ermes_msg_send(line);

	if (msg_fifo_batch_end(&msg_fifo, ermes_idle() && !num_start_pages, 2.0))
		quit = 1;
}

static const struct number_lengths number_lengths[] = {
	{ 7, "local address (decimal, 22 bits)" },
	{ 0, NULL }
};

int main(int argc, char *argv[])
{
	int rc, argi, i, j, channel[MAX_SENDER], chan_fsi[MAX_SENDER];
	double chan_polarity[MAX_SENDER];
	const char *station_id = "";
	char *end;

	/* no emphasis on a data channel */
	uses_emphasis = 0;

	main_mobile_init("0123456789", number_lengths, NULL, ermes_number_valid, "german");
	main_mobile_set_no_calls();

	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/ermes.conf", handle_options);
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;

	if (argi < argc) {
		station_id = argv[argi];
		rc = main_mobile_number_ask(station_id, "station ID (local address)");
		if (rc)
			return rc;
	}

	if (!num_kanal) {
		printf("Give a channel (-k 1..%d), use '-k list' to list them.\n\n", ERMES_CHANNELS);
		print_help(argv[0]);
		return 0;
	}
	for (i = 0; i < num_kanal; i++) {
		if (!strcasecmp(kanal[i], "list")) {
			printf("Channel  Frequency\n");
			for (j = 1; j <= ERMES_CHANNELS; j++)
				printf("%4d     %.4f MHz\n", j, ermes_channel_frequency(j) / 1e6);
			return 0;
		}
		channel[i] = strtol(kanal[i], &end, 10);
		if (*end || channel[i] < 1 || channel[i] > ERMES_CHANNELS) {
			fprintf(stderr, "Invalid channel '%s', use 1..%d.\n", kanal[i], ERMES_CHANNELS);
			return 0;
		}
	}
	/* per channel: one value for all, or one per -k */
	if (num_polarity > 1 && num_polarity != num_kanal) {
		fprintf(stderr, "Give -P once, or once per channel (%d).\n", num_kanal);
		return 0;
	}
	for (i = 0; i < num_kanal; i++)
		chan_polarity[i] = num_polarity ? polarity[(num_polarity == 1) ? 0 : i] : 1.0;
	if (num_fsi) {
		if (num_fsi != num_kanal) {
			fprintf(stderr, "Give --fsi once per channel (%d).\n", num_kanal);
			return 0;
		}
		memcpy(chan_fsi, fsi, num_kanal * sizeof(int));
	} else if (num_kanal == 1)
		chan_fsi[0] = ERMES_FSI_ALL;
	else if (num_kanal == 2) {
		/* §9.2 p.29: "In two channel networks the transmitted FSI shall be set to 28 and 29",
		 * used in descending order */
		chan_fsi[0] = ERMES_FSI_ALL - 1;
		chan_fsi[1] = ERMES_FSI_ALL - 2;
	} else {
		fprintf(stderr, "With %d channels give --fsi for each: the standard says 30 for one, 28 and 29 for two.\n", num_kanal);
		return 0;
	}
	for (i = 0; i < num_kanal; i++) {
		for (j = i + 1; j < num_kanal; j++) {
			/* one transmitter cannot send two signals on one frequency */
			if (channel[i] == channel[j]) {
				fprintf(stderr, "Channel %d given twice: two signals on one frequency collide on the air.\n", channel[i]);
				return 0;
			}
			/* a page could go on both */
			if (ermes_fsi_overlap(chan_fsi[i], chan_fsi[j])) {
				fprintf(stderr, "Channels %d (FSI %d) and %d (FSI %d) have receivers (FSN) in common: give FSIs that do not overlap.\n",
					channel[i], chan_fsi[i], channel[j], chan_fsi[j]);
				return 0;
			}
		}
	}
	if (use_sdr) {
		for (i = 0; i < num_kanal; i++)
			dsp_device[i] = "sdr";
		num_device = num_kanal;
	}
	if (num_kanal == 1 && num_device == 0)
		num_device = 1;
	if (num_kanal != num_device) {
		fprintf(stderr, "You need to specify as many sound devices as you have channels.\n");
		return 0;
	}

	if (msg_fifo_open(&msg_fifo, msg_send_path, msg_send_file) < 0)
		goto fail;

	if (book_file)
		ermes_book_file = book_file;
	else if (getenv("HOME")) {
		static char path[512], old[512];

		snprintf(path, sizeof(path), "%s/.osmocom", getenv("HOME"));
		mkdir(path, 0755);
		snprintf(path, sizeof(path), "%s/.osmocom/analog", getenv("HOME"));
		mkdir(path, 0755);
		snprintf(path, sizeof(path), "%s/.osmocom/analog/ermes_book", getenv("HOME"));
		ermes_book_file = path;
		/* the message numbers file of earlier versions, taken over when there is no book yet */
		snprintf(old, sizeof(old), "%s/.osmocom/analog/ermes_numbers", getenv("HOME"));
		ermes_book_old_numbers = old;
	}

	fm_init(fast_math);

	for (i = 0; i < num_kanal; i++) {
		ermes_network_t chan_net = net;

		chan_net.fsi = chan_fsi[i];
		rc = ermes_create(kanal[i], channel[i], dsp_device[i], use_sdr, dsp_samplerate, rx_gain, tx_gain, chan_polarity[i], &chan_net, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback);
		if (rc < 0) {
			fprintf(stderr, "Failed to create \"Sender\" instance. Quitting!\n");
			goto fail;
		}
		printf("Base station ready on channel %d, %.4f MHz, FSI %d, polarity %s\n", channel[i],
		       ermes_channel_frequency(channel[i]) / 1e6, chan_fsi[i], (chan_polarity[i] > 0) ? "normal" : "inverted");
	}

	start_time = now_s();
	main_mobile_loop("ermes", &quit, myhandler, station_id);

fail:
	msg_fifo_close(&msg_fifo);
	while (sender_head)
		ermes_destroy(sender_head);
	main_mobile_exit();
	fm_exit();
	options_free();

	return 0;
}
