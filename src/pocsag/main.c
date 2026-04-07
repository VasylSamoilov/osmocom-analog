/* POCSAG (Radio-Paging Code #1) main
 *
 * (C) 2021 by Andreas Eversberg <jolly@eversberg.eu>
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
#include "pocsag.h"
#include "dsp.h"

#define MSG_SEND_DEFAULT "/tmp/pocsag_msg_send"
#define MSG_RECEIVED "/tmp/pocsag_msg_received"
static const char *msg_send_path = MSG_SEND_DEFAULT;
static int msg_send_fd = -1;

static int tx = 0;		/* we transmit */
static int rx = 0;		/* we receive */
static int baudrate = 1200;
static int baudrate_given = 0;
static double deviation = 4500;
static int deviation_given = 0;
static double polarity = -1;
static int polarity_given = 0;
/* Default function (sub-address) and message type */
static enum pocsag_function function = POCSAG_FUNCTION_A;
static enum pocsag_msg_type msg_type = POCSAG_MSG_TYPE_AUTO;
static const char *message = "1234";
/*
 * Default padding for alphanumeric messages.
 *
 * Per POCSAG spec (AN142, CCIR Rec. 584): "Alphanumeric messages should be
 * padded with null." The spec also notes: "The last codeword is filled with
 * unprintable characters such as end of message, end of text, or null.
 * Null is the only character which can be incomplete."
 *
 * NULL (0x00) is the correct default per specification. Some legacy pagers
 * may expect EOT (0x04), which can be selected with --padding 4.
 */
static char padding = 0;
static enum pocsag_language language = LANGUAGE_DEFAULT;
static uint32_t scan_from = 0;
static uint32_t scan_to = 0;
static double dedup_window = 0.0;	/* RX dedup window in seconds (0=disabled) */

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "-k 466.230 | -k list ");
	/*      -                                                                             - */
	printf(" -T --tx\n");
	printf("        Transmit POCSAG signal on given channel, to page a receiver. (default)\n");
	printf(" -R --rx\n");
	printf("        Receive POCSAG signal on given channel, so we are the receiver.\n");
	printf("        If none of the options -T nor -R is given, only transmitter is enabled.\n");
	printf(" -B --baud-rate 512 | 1200 | 2400\n");
	printf("        Choose baud rate of transmitter. For RX, locks auto-detection to this\n");
	printf("        rate. If not given, RX auto-detects from preamble timing.\n");
	printf(" -D --deviation wide | 4.5 | narrow | 2.5 | <other KHz>\n");
	printf("        Choose deviation of FFSK signal (default %.0f KHz).\n", deviation / 1000.0);
	printf(" -P --polarity normal | inverted | -1 | negative | 1 | positive\n");
	printf("        Choose polarity of FFSK signal. (default %s).\n", (polarity < 0) ? "normal" : "inverted");
	printf("        'normal' (CCIR Rec. 584 standard): binary 1 = negative deviation,\n");
	printf("        binary 0 = positive deviation. This is the default for all known\n");
	printf("        POCSAG networks (Scall, Quix, TeLMI, Skyper, Cityruf, DAPNET).\n");
	printf("        'inverted': opposite mapping. Rarely used.\n");
	printf("        Legacy names 'negative'/'positive' are accepted for compatibility.\n");
	printf("        For RX, locks auto-detection to this polarity. If not given, RX\n");
	printf("        auto-detects by trying both polarities during sync search.\n");
	printf(" -F --function 0..3 | A..D\n");
	printf("        Set default sub-address for 7-digit RICs. (default %s)\n", pocsag_function_name[function]);
	printf(" -y --type auto | tone | numeric | alpha\n");
	printf("        Set message encoding type. (default %s)\n", pocsag_msg_type_name(msg_type));
	printf("          auto    - Auto-detect based on message content\n");
	printf("          tone    - Tone-only (no message content)\n");
	printf("          numeric - BCD-encoded numeric message\n");
	printf("                    Valid chars: 0-9 R U <space> - [ ]\n");
	printf("                    (R=return/received, U=urgent, 5 chars per codeword)\n");
	printf("          alpha   - 7-bit ASCII alphanumeric message\n");
	printf(" -M --message \"...\"\n");
	printf("        Send this message, if no caller ID was given or of built-in console\n");
	printf("        is used. (default \"%s\").\n", message);
	printf(" -L --language <maker-model-codepage>\n");
	printf("        Select character encoding for TX/RX translation.\n");
	printf("          unknown-unknown-german                - German Umlauts from/to UTF-8\n");
	printf("          nec-skyper-categories                 - NEC Skyper ROT-1 Caesar cipher\n");
	printf("          motorola-advisor_linguist-cyrillic    - Motorola Advisor Linguist Cyrillic\n");
	printf(" -S --scan <from> <to>\n");
	printf("        Scan through given IDs once (no repetition). This can be useful to find\n");
	printf("        the RIC of a vintage pager. Note that scanning all RICs from 0 through\n");
	printf("        2097151 would take about 16.5 Hours at 1200 Baud and known sub RIC.\n");
	printf("        Use -F to select sub-address and -y to select message type.\n");
	printf("        Short messages with 5 numeric or 2 alpha chars are sent without\n");
	printf("        increase in scanning time.\n");
	printf("    --padding 0 | 3 | 4 | ...\n");
	printf("        Padding character for alphanumeric messages (7-bit ASCII, 0-127).\n");
	printf("        Per POCSAG spec: 'The last codeword is filled with unprintable\n");
	printf("        characters such as end of message, end of text, or null.'\n");
	printf("        Recommended: 0 (NUL, default per spec), 3 (ETX), 4 (EOT).\n");
	printf("        Only NULL can be incomplete (partial bits filled with zeros).\n");
	printf("    --dedup <seconds>\n");
	printf("        RX deduplication window in seconds (default 0 = disabled).\n");
	printf("        When set, duplicate messages to the same RIC+function within this\n");
	printf("        window are suppressed. Uncorrectable codewords from the new copy\n");
	printf("        are recovered from the previous copy if possible.\n");
	printf("        Typical pager dedup windows: 15, 30, 60, 120 seconds.\n");
	printf("\n");
	printf("RIC (Radio Identity Code) Structure:\n");
	printf("      The RIC is a 21-bit pager address (0 to 2097151), formed as follows:\n");
	printf("        - 18 address bits are transmitted in the address codeword (bits 2-19)\n");
	printf("        - 3 frame bits are derived from the frame position within the batch\n");
	printf("        - Frame bits are the 3 LSBs (RIC & 7), address bits are upper 18 bits\n");
	printf("      This allows pagers to power-save by only listening to their frame.\n");
	printf("      Reserved/Invalid RICs:\n");
	printf("        - 2097152+     : Out of range (exceeds 21 bits)\n");
	printf("        - 2007664-2007671: Reserved (address bits match idle codeword pattern)\n");
	printf("\n");
	printf("NOTE: Function bits (0-3/A-D) are sub-addresses, NOT message types.\n");
	printf("      Message encoding (tone/numeric/alpha) is set separately with -y.\n");
	printf("\n");
	printf("    --fifo <path>\n");
	printf("        Path for the message send FIFO (default %s).\n", MSG_SEND_DEFAULT);
	printf("File: %s\n", msg_send_path);
	printf("        Write \"<capcode>,<type>,<options>,<message>\" to send a message.\n");
	printf("        Format: capcode,type,options,message (always 4 comma-separated fields)\n");
	printf("          capcode: RIC with optional function suffix, e.g. 1234567A\n");
	printf("            RIC: 0-2097151, function: A-D suffix (default A if omitted)\n");
	printf("          type: auto|tone|numeric|alpha\n");
	printf("          options: space-separated key=value pairs (can be empty):\n");
	printf("            charset=default|unknown-unknown-german|nec-skyper-categories|motorola-advisor_linguist-cyrillic\n");
	printf("            repeat=N          retransmissions after initial TX (0-10, default 0)\n");
	printf("            delay=N           defer initial TX by N seconds (default 0)\n");
	printf("            interval=N        seconds between retransmissions (default 10)\n");
	printf("          message: text content\n");
	printf("        Examples:\n");
	printf("          1234567A,alpha,,Hello World\n");
	printf("          1234567,numeric,,12345\n");
	printf("          1234567B,alpha,charset=motorola-advisor_linguist-cyrillic,Привіт\n");
	printf("          1234567C,alpha,charset=nec-skyper-categories,Hello\n");
	printf("          1234567,tone,,\n");
	printf("        Numeric messages: only 0-9 R U <space> - [ ] are valid.\n");
	printf("        Alphanumeric messages may contain any 7-bit character (0-127).\n");
	printf("        Use escape sequences for control characters (LF/CR terminate input):\n");
	printf("          '<NUL>' '<SOH>' '<STX>' '<ETX>' '<EOT>' '<ENQ>' '<ACK>' '<BEL>'\n");
	printf("          '<BS>'  '<HT>'  '<LF>'  '<VT>'  '<FF>'  '<CR>'  '<SO>'  '<SI>'\n");
	printf("          '<DLE>  '<DC1>' '<DC2'  '<DC3>' '<DC4>' '<NAK>' '<SYN>' '<ETB>'\n");
	printf("          '<CAN>' '<EM>'  '<SUB>' '<ESC>' '<FS>'  '<GS>'  '<RS>'  '<US>'\n");
	printf("          '<DEL>  Example: Hello,<LF><CR>World!'\n");

	printf("File: %s\n", MSG_RECEIVED);
	printf("        Read from it to see received messages.\n");
	main_mobile_print_station_id();
	main_mobile_print_hotkeys();
}

#define OPT_PADDING	256
#define OPT_DEDUP	257
#define OPT_FIFO	258

static void add_options(void)
{
	main_mobile_add_options();
	option_add('T', "tx", 0);
	option_add('R', "rx", 0);
	option_add('B', "baud-rate", 1);
	option_add('D', "deviation", 1);
	option_add('F', "function", 1);
	option_add('y', "type", 1);
	option_add('P', "polarity", 1);
	option_add('M', "message", 1);
	option_add('L', "language", 1);
	option_add('S', "scan", 2);
	option_add(OPT_PADDING, "padding", 1);
	option_add(OPT_DEDUP, "dedup", 1);
	option_add(OPT_FIFO, "fifo", 1);
}

static int handle_options(int short_option, int argi, char **argv)
{
	int rc;

	switch (short_option) {
	case 'T':
		tx = 1;
		break;
	case 'R':
		rx = 1;
		break;
	case 'B':
		baudrate = atoi(argv[argi]);
		if (baudrate != 512 && baudrate != 1200 && baudrate != 2400) {
			fprintf(stderr, "Given baud-rate is not 512, 1200 nor 2400, use '-h' for help.\n");
			return -EINVAL;
		}
		baudrate_given = 1;
		break;
	case 'D':
		if (argv[argi][0] == 'n' || argv[argi][0] == 'N')
			deviation = 2500.0;
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
		if (argv[argi][0] == 'n' || argv[argi][0] == 'N')
			polarity = -1.0;  /* "normal" or legacy "negative" */
		else if (argv[argi][0] == 'p' || argv[argi][0] == 'P')
			polarity = 1.0;   /* legacy "positive" */
		else if (argv[argi][0] == 'i' || argv[argi][0] == 'I')
			polarity = 1.0;   /* "inverted" */
		else if (atoi(argv[argi]) == -1)
			polarity = -1.0;
		else if (atoi(argv[argi]) == 1)
			polarity = 1.0;
		else {
			fprintf(stderr, "Given polarity is invalid, use 'normal', 'inverted', or '-h' for help.\n");
			return -EINVAL;
		}
		polarity_given = 1;
		break;
	case 'F':
		rc = pocsag_function_name2value(argv[argi]);
		if (rc < 0) {
			fprintf(stderr, "Given function is invalid. Use A/B/C/D or 0/1/2/3.\n");
			return rc;
		}
		function = rc;
		break;
	case 'y':
		rc = pocsag_msg_type_name2value(argv[argi]);
		if (rc < 0) {
			fprintf(stderr, "Given type is invalid. Use auto/tone/numeric/alpha.\n");
			return rc;
		}
		msg_type = rc;
		break;
	case 'M':
		message = options_strdup(argv[argi++]);
		break;
	case 'L':
		if (!strcasecmp(argv[argi], "unknown-unknown-german"))
			language = LANGUAGE_GERMAN;
		else if (!strcasecmp(argv[argi], "nec-skyper-categories"))
			language = LANGUAGE_SKYPER;
		else if (!strcasecmp(argv[argi], "motorola-advisor_linguist-cyrillic"))
			language = LANGUAGE_CYRILLIC;
		else {
			fprintf(stderr, "Unknown language '%s'. Use unknown-unknown-german, nec-skyper-categories, or motorola-advisor_linguist-cyrillic.\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case 'S':
		scan_from = atoi(argv[argi++]);
		if (scan_from > 2097151) {
			fprintf(stderr, "Given RIC to scan from is out of range!\n");
			return -EINVAL;
		}
		scan_to = atoi(argv[argi++]) + 1;
		if (scan_to > 2097151 + 1) {
			fprintf(stderr, "Given RIC to scan to is out of range!\n");
			return -EINVAL;
		}
		break;
	case OPT_PADDING:
		/*
		 * Validate padding character per POCSAG spec (AN142, CCIR Rec. 584):
		 * "The last codeword is filled with unprintable characters such as
		 * end of message, end of text, or null."
		 *
		 * Must be a 7-bit ASCII value. Recommended values are control
		 * characters: NUL (0), ETX (3), EOT (4). Printable characters
		 * (32-126) are discouraged as they may confuse pager displays.
		 */
		{
			int pad_value = atoi(argv[argi++]);
			if (pad_value < 0 || pad_value > 127) {
				fprintf(stderr, "Padding must be a 7-bit ASCII value (0-127).\n");
				return -EINVAL;
			}
			if (pad_value >= 32 && pad_value < 127) {
				fprintf(stderr, "Warning: Padding with printable character '%c' (0x%02X).\n", pad_value, pad_value);
				fprintf(stderr, "         Per POCSAG spec, padding should be unprintable (NUL, ETX, EOT, etc.).\n");
			}
			padding = (char)pad_value;
		}
		break;
	case OPT_DEDUP:
		dedup_window = atof(argv[argi++]);
		if (dedup_window < 0.0) {
			fprintf(stderr, "Dedup window must be >= 0 seconds.\n");
			return -EINVAL;
		}
		break;
	case OPT_FIFO:
		msg_send_path = options_strdup(argv[argi++]);
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}

static void myhandler(void)
{
	static char buffer[4096];
	static int buf_len = 0;
	int rc, i, start;
	int space = sizeof(buffer) - buf_len;

	rc = read(msg_send_fd, buffer + buf_len, space);
	if (rc > 0) {
		buf_len += rc;

		/* Overflow handling: if buffer is full with no complete message,
		 * discard all data */
		if (buf_len == (int)sizeof(buffer)) {
			int has_newline = 0;
			for (i = 0; i < buf_len; i++) {
				if (buffer[i] == '\r' || buffer[i] == '\n') {
					has_newline = 1;
					break;
				}
			}
			if (!has_newline) {
				fprintf(stderr, "Message buffer overflow, discarding!\n");
				buf_len = 0;
				return;
			}
		}

		/* Process all complete lines */
		start = 0;
		for (i = 0; i < buf_len; i++) {
			if (buffer[i] == '\r' || buffer[i] == '\n') {
				buffer[i] = '\0';
				if (i > start) {
					if (tx)
						pocsag_msg_send(language, buffer + start, i - start);
					else
						LOGP(DPOCSAG, LOGL_ERROR, "Failed to send message, transmitter is not enabled!\n");
				}
				start = i + 1;
			}
		}

		/* Retain any partial line */
		if (start > 0 && start < buf_len) {
			memmove(buffer, buffer + start, buf_len - start);
			buf_len -= start;
		} else if (start >= buf_len) {
			buf_len = 0;
		}
	}
}

int msg_receive(const char *text)
{
	FILE *fp;

	fp = fopen(MSG_RECEIVED, "a");
	if (!fp) {
		fprintf(stderr, "Failed to open MSG receive file '%s'!\n", MSG_RECEIVED);
		return -1;
	}

	fprintf(fp, "%s\n", text);

	fclose(fp);

	return 0;
}

static const struct number_lengths number_lengths[] = {
	{ 7, "RIC with default function" },
	{ 8, "RIC with function (append 0..3 or A..D)" },
	{ 0, NULL }
};

int main(int argc, char *argv[])
{
	int rc, argi;
	const char *station_id = "";
	int i;
	double frequency;

	/* pocsag does not use emphasis, so disable it */
	uses_emphasis = 0;

	/* init mobile interface */
	main_mobile_init("0123456789ABCD", number_lengths, NULL, pocsag_number_valid, "german");

	/* handle options / config file */
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/pocsag.conf", handle_options);
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;

	if (argi < argc) {
		station_id = argv[argi];
		rc = main_mobile_number_ask(station_id, "station ID (RIC)");
		if (rc)
			return rc;
	}

	if (!num_kanal) {
		printf("No channel is specified, Use '-k list' to get a list of all channels.\n\n");
		print_help(argv[0]);
		return 0;
	}
	for (i = 0; i < num_kanal; i++) {
		if (!strcasecmp(kanal[i], "list")) {
			pocsag_list_channels();
			goto fail;
		}
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

	/* TX & RX if loopback */
	if (loopback)
		tx = rx = 1;

	/* no TX, no scanning */
	if (!tx && scan_to > scan_from) {
		fprintf(stderr, "You need to enable TX, in order to scan.\n");
		goto fail;
	}

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
	pocsag_init();

	/* create transceiver instance */
	for (i = 0; i < num_kanal; i++) {
		frequency = pocsag_channel2freq(kanal[i], (deviation_given) ? NULL : &deviation, (polarity_given) ? NULL : &polarity, (baudrate_given) ? NULL : &baudrate);
		if (frequency == 0.0) {
			printf("Invalid channel '%s', Use '-k list' to get a list of all channels.\n\n", kanal[i]);
			goto fail;
		}
		rc = pocsag_create(kanal[i], frequency, dsp_device[i], use_sdr, dsp_samplerate, rx_gain, tx_gain, tx, rx, language, baudrate, deviation, polarity, function, msg_type, message, padding, scan_from, scan_to, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, !baudrate_given, !polarity_given, dedup_window);
		if (rc < 0) {
			fprintf(stderr, "Failed to create \"Sender\" instance. Quitting!\n");
			goto fail;
		}
		printf("Base station ready, please tune transmitter (or receiver) to %.4f MHz\n", frequency / 1e6);
	}

	main_mobile_loop("pocsag", &quit, myhandler, station_id);

fail:
	/* pipe */
	if (msg_send_fd > 0)
		close(msg_send_fd);
	unlink(msg_send_path);

	/* destroy transceiver instance */
	while(sender_head)
		pocsag_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();
	pocsag_exit();

	options_free();

	return 0;
}

