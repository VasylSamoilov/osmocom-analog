/* Golay/GSC pager main
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/main_mobile.h"
#include "../liboptions/options.h"
#include "../libfm/fm.h"
#include "golay.h"

#define MSG_SEND_DEFAULT "/tmp/golay_msg_send"
#define MSG_RECEIVED "/tmp/golay_msg_received"
#define FIFO_BUFFER_SIZE 4096
static const char *msg_send_path = MSG_SEND_DEFAULT;
static int msg_send_fd = -1;

static int tx = 0;		/* we transmit */
static int rx = 0;		/* we receive */
static double deviation = 4500;	/* WB confirmed by an email: POCSAG and GSC have same deviation of +-4.5 kHz. */
static int deviation_given = 0;
static double polarity = 1;
static int polarity_given = 0;
static const char *message = "1234";
static const char *voice_dir = NULL;	/* voice recording output folder */
static int voice_monitor = 0;		/* voice monitor mode (play to audio output) */
static int batching_mode = BATCHING_OFF;
static int protocol_dump = 0;
static int nbs_mode = 0;
static uint32_t scan_from = 0;
static uint32_t scan_to = 0;
static enum gsc_msg_type scan_type = TYPE_NUMERIC;

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "| -k 462.900 | -k <MHz> ");
	/*      -                                                                             - */
	printf(" -T --tx\n");
	printf("        Transmit GSC signal on given channel, to page a receiver. (default)\n");
	printf(" -R --rx\n");
	printf("        Receive GSC signal on given channel, so we are the receiver.\n");
	printf("        If none of the options -T nor -R is given, only transmitter is enabled.\n");
	printf(" -D --deviation wide | 4.5 | narrow | 1.0 | <other KHz>\n"); /* NB confirmed by IQ data from signal-id-wiki */
	printf("        Choose deviation of FFSK signal (default %.0f KHz).\n", deviation / 1000.0);
	printf(" -P --polarity normal | inverted | positive | negative | 1 | -1\n");
	printf("        Choose polarity of FFSK signal. 'normal' (or 'positive' or '1') means\n");
	printf("        binary 1 = positive deviation per GSC standard. 'inverted' (or\n");
	printf("        'negative' or '-1') reverses the mapping. (default normal).\n");
	printf("        For RX, locks polarity to this value. If not given, RX auto-detects\n");
	printf("        polarity from the preamble.\n");
	printf(" -B --batching off | normal | extended\n");
	printf("        Set batch encoding mode. 'off' transmits each message individually\n");
	printf("        (default). 'normal' groups up to 16 addresses per preamble. 'extended'\n");
	printf("        groups up to 32 addresses per preamble.\n");
	printf(" -M --message \"...\"\n");
	printf("        Send this message, if no caller ID was given or if built-in console\n");
	printf("        is used. (default \"%s\").\n", message);
	printf(" -V --voice-dir <path>\n");
	printf("        Record received voice pages to WAV files in the given directory.\n");
	printf("        Filenames: golay_voice_page_<timestamp>_<address>.wav\n");
	printf("    --voice-monitor\n");
	printf("        Also play received voice pages to the audio output device.\n");
	printf("    --fifo <path>\n");
	printf("        Path for the message send FIFO (default %s).\n", MSG_SEND_DEFAULT);
	printf("    --protocol-dump\n");
	printf("        Dump TX bit buffer to the log for protocol analysis.\n");
	printf("    --nbs\n");
	printf("        Non-battery-saver mode: use 75 Hz preamble without coded preamble\n");
	printf("        or start code. Higher throughput, but no battery saving groups.\n");
	printf(" -S --scan <from> <to>\n");
	printf("        Scan through given 7-digit functional address range.\n");
	printf("        Messages are batch-packed by preamble group for efficiency.\n");
	printf("        Use -y to select message type: numeric (default), alpha, or tone.\n");
	printf("\n");
	printf("File: %s\n", msg_send_path);
	printf("        Write \"<address>[,message]\" to it, to send a default message.\n");
	printf("        Write \"<address>,n,message\" to it, to send a numeric message.\n");
	printf("        Write \"<address>,a,message\" to it, to send an alphanumeric message.\n");
	printf("        Write \"<address>,v,<wave file name>\" to it, to send a voice message.\n");
	printf("\n");
	printf("By default, an alphanumic message is sent, if last digit of the functional\n");
	printf("address is 5..8. Otherwise a tone only message is sent.\n");
	printf("\n");
	printf("A numeric message can have up to 24 digits, they are: 0123456789U-* and space\n");
	printf("Also 'shifted' digits can be sent using two digits, they are: ABCDEFGHJLNPR\n");
	printf("\n");
	printf("An aplhanumeric message can have up to 80 digits, sent upper case only.\n");
	main_mobile_print_station_id();
	main_mobile_print_hotkeys();
}

static void add_options(void)
{
	main_mobile_add_options();
	option_add('T', "tx", 0);
	option_add('R', "rx", 0);
	option_add('D', "deviation", 1);
	option_add('P', "polarity", 1);
	option_add('M', "message", 1);
	option_add('V', "voice-dir", 1);
	option_add('B', "batching", 1);
	option_add(0x100, "voice-monitor", 0);
	option_add(0x101, "fifo", 1);
	option_add(0x102, "protocol-dump", 0);
	option_add(0x103, "nbs", 0);
	option_add('S', "scan", 2);
	option_add('y', "type", 1);
}

static int handle_options(int short_option, int argi, char **argv)
{
	switch (short_option) {
	case 'T':
		tx = 1;
		break;
	case 'R':
		rx = 1;
		break;
	case 'D':
		if (argv[argi][0] == 'n' || argv[argi][0] == 'N')
			deviation = 1000.0;
		else if (argv[argi][0] == 'w' || argv[argi][0] == 'W')
			deviation = 4500.0;
		else
			deviation = atof(argv[argi]) * 1000.0;
		if (deviation < 1000.0) {
			fprintf(stderr, "Given deviation is too low, use higher deviation.\n");
			return -EINVAL;
		}
		if (deviation > 10000.0) {
			fprintf(stderr, "Given deviation is too high, use lower deviation.\n");
			return -EINVAL;
		}
		deviation_given = 1;
		break;
	case 'P':
		if (!strcmp(argv[argi], "normal") || !strcmp(argv[argi], "positive"))
			polarity = 1.0;
		else if (!strcmp(argv[argi], "inverted") || !strcmp(argv[argi], "negative"))
			polarity = -1.0;
		else if (atoi(argv[argi]) == 1)
			polarity = 1.0;
		else if (atoi(argv[argi]) == -1)
			polarity = -1.0;
		else {
			fprintf(stderr, "Given polarity '%s' is invalid, use 'normal', 'inverted', 'positive', 'negative', '1', or '-1'.\n", argv[argi]);
			return -EINVAL;
		}
		polarity_given = 1;
		break;
	case 'M':
		message = options_strdup(argv[argi++]);
		break;
	case 'V':
		voice_dir = options_strdup(argv[argi++]);
		break;
	case 0x100: /* --voice-monitor */
		voice_monitor = 1;
		break;
	case 0x101: /* --fifo */
		msg_send_path = options_strdup(argv[argi++]);
		break;
	case 'B':
		if (!strcmp(argv[argi], "off"))
			batching_mode = BATCHING_OFF;
		else if (!strcmp(argv[argi], "normal"))
			batching_mode = BATCHING_NORMAL;
		else if (!strcmp(argv[argi], "extended"))
			batching_mode = BATCHING_EXTENDED;
		else {
			fprintf(stderr, "Invalid batching mode '%s', use 'off', 'normal', or 'extended'.\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case 0x102: /* --protocol-dump */
		protocol_dump = 1;
		break;
	case 0x103: /* --nbs */
		nbs_mode = 1;
		break;
	case 'S':
		scan_from = atoi(argv[argi++]);
		if (scan_from > 9999999) {
			fprintf(stderr, "Given address to scan from is out of range (max 9999999)!\n");
			return -EINVAL;
		}
		scan_to = atoi(argv[argi++]) + 1;
		if (scan_to > 9999999 + 1) {
			fprintf(stderr, "Given address to scan to is out of range (max 9999999)!\n");
			return -EINVAL;
		}
		break;
	case 'y':
		if (!strcmp(argv[argi], "numeric") || !strcmp(argv[argi], "n"))
			scan_type = TYPE_NUMERIC;
		else if (!strcmp(argv[argi], "alpha") || !strcmp(argv[argi], "a"))
			scan_type = TYPE_ALPHA;
		else if (!strcmp(argv[argi], "tone") || !strcmp(argv[argi], "t"))
			scan_type = TYPE_TONE;
		else {
			fprintf(stderr, "Invalid type '%s', use numeric, alpha, or tone.\n", argv[argi]);
			return -EINVAL;
		}
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}

static void myhandler(void)
{
	static char buffer[FIFO_BUFFER_SIZE];
	static int buf_len = 0;
	int rc, i, start;
	int space = sizeof(buffer) - buf_len;

	rc = read(msg_send_fd, buffer + buf_len, space);
	if (rc > 0) {
		LOGP(DGOLAY, LOGL_DEBUG, "FIFO: read %d bytes, buf_len=%d\n", rc, buf_len + rc);
		buf_len += rc;

		/* Overflow handling: if buffer is full with no complete message,
		 * log warning and discard all data */
		if (buf_len == (int)sizeof(buffer)) {
			int has_newline = 0;
			for (i = 0; i < buf_len; i++) {
				if (buffer[i] == '\r' || buffer[i] == '\n') {
					has_newline = 1;
					break;
				}
			}
			if (!has_newline) {
				LOGP(DGOLAY, LOGL_ERROR, "FIFO buffer overflow (%d bytes) with no complete message, discarding data!\n", buf_len);
				buf_len = 0;
				return;
			}
		}

		/* Process up to 16 complete messages per call to avoid starving
		 * the audio loop when the FIFO is flooded. */
		start = 0;
		int msg_count = 0;
		for (i = 0; i < buf_len; i++) {
			if (buffer[i] == '\r' || buffer[i] == '\n') {
				buffer[i] = '\0';
				/* Only process non-empty lines */
				if (i > start) {
					LOGP(DGOLAY, LOGL_DEBUG, "FIFO: processing msg %d: '%s'\n", msg_count + 1, buffer + start);
					if (tx)
						golay_msg_send(buffer + start);
					else
						LOGP(DGOLAY, LOGL_ERROR, "Failed to send message, transmitter is not enabled!\n");
					if (++msg_count >= 16)  {
						start = i + 1;
						break;
					}
				}
				start = i + 1;
			}
		}

		/* Retain any partial message (no trailing newline) via memmove */
		LOGP(DGOLAY, LOGL_DEBUG, "FIFO: processed %d msgs, start=%d buf_len=%d\n", msg_count, start, buf_len);
		if (start > 0 && start < buf_len) {
			memmove(buffer, buffer + start, buf_len - start);
			buf_len -= start;
		} else if (start >= buf_len) {
			/* All data was consumed */
			buf_len = 0;
		}
		/* If start == 0, no newline was found yet — keep accumulating */
	}
}

static const struct number_lengths number_lengths[] = {
	{ 7, "functional address" },
	{ 0, NULL }
};

int main(int argc, char *argv[])
{
	int rc, argi;
	const char *station_id = "";
	int i;
	double frequency;

	/* GSC does not use emphasis, so disable it */
	uses_emphasis = 0;

	/* init coding tables */
	init_golay();
	init_bch();

	/* init mobile interface */
	main_mobile_init("0123456789", number_lengths, NULL, NULL, "american");

	/* handle options / config file */
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/golay.conf", handle_options);
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;

	if (argi < argc) {
		station_id = argv[argi];
		rc = main_mobile_number_ask(station_id, "functional address");
		if (rc)
			return rc;
	}

	if (!num_kanal) {
		printf("No channel is specified, Use '-k <MHz>' to define frequency.\n\n");
		print_help(argv[0]);
		return 0;
	}
	if (use_sdr) {
		/* set device */
		for (i = 0; i < num_kanal; i++)
			dsp_device[i] = "sdr";
		num_device = num_kanal;
	}
	if (num_kanal == 1 && num_device == 0)
		num_device = 1; /* use default */
	if (num_kanal != num_device) {
		fprintf(stderr, "You need to specify as many sound devices as you have channels.\n");
		goto fail;
	}

	/* TX is default */
	if (!tx && !rx)
		tx = 1;

	/* no TX, no scanning */
	if (!tx && scan_to > scan_from) {
		fprintf(stderr, "You need to enable TX, in order to scan.\n");
		goto fail;
	}

	/* TX & RX if loopback */
	if (loopback)
		tx = rx = 1;

	/* create pipe for message sendy */
	unlink(msg_send_path);
	rc = mkfifo(msg_send_path, 0666);
	if (rc < 0) {
		fprintf(stderr, "Failed to create message send FIFO '%s'!\n", msg_send_path);
		goto fail;
	} else {
		msg_send_fd = open(msg_send_path, O_RDWR | O_NONBLOCK);
		if (msg_send_fd < 0) {
			fprintf(stderr, "Failed to open message send FIFO '%s'!\n", msg_send_path);
			goto fail;
		}
	}

	/* inits */
	fm_init(fast_math);

	/* create transceiver instance */
	for (i = 0; i < num_kanal; i++) {
		frequency = atof(kanal[i]) * 1e6;
		rc = golay_create(kanal[i], frequency, dsp_device[i], use_sdr, dsp_samplerate, rx_gain, tx_gain, deviation, polarity, tx, rx, !polarity_given, message, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, voice_dir, voice_monitor);
		if (rc < 0) {
			fprintf(stderr, "Failed to create \"Sender\" instance. Quitting!\n");
			goto fail;
		}
		{
			gsc_t *gsc = (gsc_t *)sender_head;
			gsc->batching_mode = batching_mode;
			gsc->protocol_dump = protocol_dump;
			gsc->nbs = nbs_mode;
			/* Set up scan mode */
			if (scan_to > scan_from) {
				gsc->scan_from = scan_from;
				gsc->scan_to = scan_to;
				gsc->scan_type = scan_type;
				/* Auto-enable normal batching for scan */
				if (gsc->batching_mode == BATCHING_OFF)
					gsc->batching_mode = BATCHING_NORMAL;
				/* Enqueue initial batch */
				golay_scan_enqueue(gsc, &gsc->scan_from, gsc->scan_to, gsc->scan_type, 16);
			}
		}
		printf("Base station ready, please tune transmitter (or receiver) to %.4f MHz\n", frequency / 1e6);
	}

	main_mobile_loop("golay", &quit, myhandler, station_id);

fail:
	/* pipe */
	if (msg_send_fd > 0)
		close(msg_send_fd);
	unlink(msg_send_path);

	/* destroy transceiver instance */
	while(sender_head)
		golay_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();

	options_free();

	return 0;
}

