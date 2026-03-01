/* FLEX pager transmitter main (stub)
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/main_mobile.h"
#include "../liboptions/options.h"
#include "../libfm/fm.h"
#include "flex.h"
#include "frame.h"
#include "dsp.h"

#define MSG_SEND "/tmp/flex_msg_send"
static int msg_send_fd = -1;

static int tx = 0;
static double deviation = 4800;
static double polarity = -1;
static enum flex_msg_type msg_type = FLEX_MSG_TYPE_AUTO;
static const char *message = "1234";
static uint64_t scan_from = 0;
static uint64_t scan_to = 0;

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "-k <frequency>");
	printf(" -T --tx\n");
	printf("        Transmit FLEX signal on given channel. (default)\n");
	printf(" -D --deviation <KHz>\n");
	printf("        Choose deviation of FSK signal (default %.0f Hz).\n", deviation);
	printf(" -P --polarity -1 | negative | 1 | positive\n");
	printf("        Choose polarity of FSK signal. (default %s).\n", (polarity < 0) ? "negative" : "positive");
	printf(" -y --type auto | tone | numeric | alpha\n");
	printf("        Set message type. (default auto)\n");
	printf(" -M --message \"...\"\n");
	printf("        Default message text. (default \"%s\").\n", message);
	printf(" -S --scan <from> <to>\n");
	printf("        Scan through given capcode range.\n");
	main_mobile_print_hotkeys();
}

static void add_options(void)
{
	main_mobile_add_options();
	option_add('T', "tx", 0);
	option_add('D', "deviation", 1);
	option_add('P', "polarity", 1);
	option_add('y', "type", 1);
	option_add('M', "message", 1);
	option_add('S', "scan", 2);
}

static int handle_options(int short_option, int argi, char **argv)
{
	switch (short_option) {
	case 'T':
		tx = 1;
		break;
	case 'D':
		deviation = atof(argv[argi]) * 1000.0;
		break;
	case 'P':
		if (argv[argi][0] == 'n' || argv[argi][0] == 'N')
			polarity = -1.0;
		else if (argv[argi][0] == 'p' || argv[argi][0] == 'P')
			polarity = 1.0;
		else if (atoi(argv[argi]) == -1)
			polarity = -1.0;
		else if (atoi(argv[argi]) == 1)
			polarity = 1.0;
		else {
			fprintf(stderr, "Given polarity is not positive nor negative, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case 'y':
		if (!strcasecmp(argv[argi], "auto") || !strcmp(argv[argi], "0"))
			msg_type = FLEX_MSG_TYPE_AUTO;
		else if (!strcasecmp(argv[argi], "tone") || !strcmp(argv[argi], "1"))
			msg_type = FLEX_MSG_TYPE_TONE;
		else if (!strcasecmp(argv[argi], "numeric") || !strcasecmp(argv[argi], "num") || !strcmp(argv[argi], "2"))
			msg_type = FLEX_MSG_TYPE_NUMERIC;
		else if (!strcasecmp(argv[argi], "alpha") || !strcasecmp(argv[argi], "alphanumeric") || !strcmp(argv[argi], "3"))
			msg_type = FLEX_MSG_TYPE_ALPHA;
		else {
			fprintf(stderr, "Given type is invalid. Use auto/tone/numeric/alpha.\n");
			return -EINVAL;
		}
		break;
	case 'M':
		message = options_strdup(argv[argi]);
		break;
	case 'S':
		scan_from = atoll(argv[argi++]);
		scan_to = atoll(argv[argi++]) + 1;
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}

static void myhandler(void)
{
	static char buffer[256];
	static int pos = 0;
	int rc, i;
	int space = sizeof(buffer) - pos;

	rc = read(msg_send_fd, buffer + pos, space);
	if (rc > 0) {
		pos += rc;
		if (pos == space) {
			fprintf(stderr, "Message buffer overflow!\n");
			pos = 0;
		}
		/* check for end of line */
		for (i = 0; i < pos; i++) {
			if (buffer[i] == '\r' || buffer[i] == '\n')
				break;
		}
		/* parse and send msg */
		if (i < pos) {
			int text_length = i;
			const char *text = buffer;
			char capcode_string[text_length + 1];
			char type_string[text_length + 1];
			char message[text_length + 1];
			uint64_t capcode;
			enum flex_msg_type mtype;
			int message_length = 0;
			int j;

			pos = 0;

			if (!tx) {
				LOGP(DFLEX, LOGL_ERROR, "Failed to send message, transmitter is not enabled!\n");
				return;
			}

			/* Parse capcode */
			for (j = 0; j < text_length; j++) {
				if (text[j] == ',')
					break;
				capcode_string[j] = text[j];
			}
			capcode_string[j] = '\0';
			if (j >= text_length) {
				LOGP(DFLEX, LOGL_NOTICE, "Given message MUST be in the format: capcode,type,message\n");
				return;
			}
			j++; /* skip comma */

			/* Parse type */
			{
				int k = 0;
				for (; j < text_length; j++, k++) {
					if (text[j] == ',')
						break;
					type_string[k] = text[j];
				}
				type_string[k] = '\0';
			}

			/* Parse optional message */
			if (j < text_length) {
				j++; /* skip comma */
				message_length = flex_scan_message(text + j, text_length - j, message, sizeof(message));
			}

			/* Validate capcode */
			capcode = strtoull(capcode_string, NULL, 10);
			if (!flex_capcode_valid(capcode)) {
				LOGP(DFLEX, LOGL_NOTICE, "Invalid capcode '%" PRIu64 "'.\n", capcode);
				return;
			}

			/* Validate message type */
			if (!strcasecmp(type_string, "auto") || !strcmp(type_string, "0"))
				mtype = FLEX_MSG_TYPE_AUTO;
			else if (!strcasecmp(type_string, "tone") || !strcmp(type_string, "1"))
				mtype = FLEX_MSG_TYPE_TONE;
			else if (!strcasecmp(type_string, "numeric") || !strcasecmp(type_string, "num") || !strcmp(type_string, "2"))
				mtype = FLEX_MSG_TYPE_NUMERIC;
			else if (!strcasecmp(type_string, "alpha") || !strcasecmp(type_string, "alphanumeric") || !strcmp(type_string, "3"))
				mtype = FLEX_MSG_TYPE_ALPHA;
			else {
				LOGP(DFLEX, LOGL_NOTICE, "Invalid type '%s'. Use auto/tone/numeric/alpha.\n", type_string);
				return;
			}

			/* Auto-detect message type if AUTO */
			if (mtype == FLEX_MSG_TYPE_AUTO) {
				if (message_length == 0) {
					mtype = FLEX_MSG_TYPE_TONE;
				} else {
					mtype = flex_detect_msg_type(message, message_length);
				}
			}

			/* Enqueue message on first transmitter instance */
			{
				flex_t *flex = (flex_t *)sender_head;
				if (flex)
					flex_msg_create(flex, capcode, mtype, message, message_length);
			}
		}
	}
}

int main(int argc, char *argv[])
{
	int rc, argi;
	const char *station_id = "";
	int i;

	/* FLEX does not use emphasis */
	uses_emphasis = 0;

	/* init mobile interface */
	static const struct number_lengths number_lengths[] = {
		{ 7, "short capcode (1-1933312)" },
		{ 10, "long capcode (2101249-4297068542)" },
		{ 0, NULL }
	};
	main_mobile_init("0123456789", number_lengths, NULL, flex_number_valid, NULL);

	/* handle options / config file */
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/flex.conf", handle_options);
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;

	if (argi < argc)
		station_id = argv[argi];

	if (!num_kanal) {
		printf("No channel is specified, use '-k <frequency>'.\n\n");
		print_help(argv[0]);
		return 0;
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
		goto fail;
	}

	/* TX is default */
	if (!tx)
		tx = 1;

	/* no TX, no scanning */
	if (!tx && scan_to > scan_from) {
		fprintf(stderr, "You need to enable TX, in order to scan.\n");
		goto fail;
	}

	/* create pipe for message send */
	unlink(MSG_SEND);
	rc = mkfifo(MSG_SEND, 0666);
	if (rc < 0) {
		fprintf(stderr, "Failed to create message send FIFO '%s'!\n", MSG_SEND);
		goto fail;
	} else {
		msg_send_fd = open(MSG_SEND, O_RDONLY | O_NONBLOCK);
		if (msg_send_fd < 0) {
			fprintf(stderr, "Failed to open message send FIFO '%s'!\n", MSG_SEND);
			goto fail;
		}
	}

	/* inits */
	fm_init(fast_math);

	/* create transceiver instance */
	for (i = 0; i < num_kanal; i++) {
		double frequency = atof(kanal[i]) * 1e6;
		rc = flex_create(kanal[i], frequency, dsp_device[i], use_sdr, dsp_samplerate, rx_gain, tx_gain, tx, deviation, polarity, msg_type, message, scan_from, scan_to, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback);
		if (rc < 0) {
			fprintf(stderr, "Failed to create \"Sender\" instance. Quitting!\n");
			goto fail;
		}
		printf("Base station ready, please tune transmitter to %.4f MHz\n", frequency / 1e6);
	}

	main_mobile_loop("flex", &quit, myhandler, station_id);

fail:
	/* pipe cleanup */
	if (msg_send_fd > 0)
		close(msg_send_fd);
	unlink(MSG_SEND);

	/* destroy transceiver instance */
	while (sender_head)
		flex_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();

	options_free();

	return 0;
}
