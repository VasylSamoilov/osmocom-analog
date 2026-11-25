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
#include "../libmobile/msg_fifo.h"
#include "golay.h"

#define MSG_SEND_DEFAULT "/tmp/golay_msg_send"
#define MSG_RECEIVED "/tmp/golay_msg_received"
static const char *msg_send_path = MSG_SEND_DEFAULT;
/* The FIFO's command lines read from a file instead (batch runs) */
static const char *msg_send_file = NULL;
static msg_fifo_t msg_fifo;

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
static double dedup_window = 60.0;	/* --dedup: RX duplicate window in seconds (0 = off) */
static int oneshot = 0;
static uint32_t scan_from = 0;
static uint32_t scan_to = 0;
static enum gsc_msg_type scan_type = TYPE_NUMERIC;
static enum gsc_msg_type default_msg_type = TYPE_AUTO;

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
	printf("        A batch has 16 slots: every address and every data block of a\n");
	printf("        message takes one. When they are used, the start code is sent again\n");
	printf("        and 16 more slots follow, without a new preamble.\n");
	printf(" -M --message \"...\"\n");
	printf("        Send this message, if no caller ID was given or if built-in console\n");
	printf("        is used. For voice pages (-y voice), this is the WAV file path.\n");
	printf("        (default \"%s\").\n", message);
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
	printf("    --oneshot\n");
	printf("        Automatic single message TX: enqueue message from CLI args\n");
	printf("        (address, -y type, -M message), transmit, then exit.\n");
	printf("        Implies -T. No interactive console needed.\n");
	printf("    --dedup <seconds>\n");
	printf("        RX deduplication window in seconds (default 60, 0 = disabled).\n");
	printf("        The receiver keeps the last 16 messages of each address (with its\n");
	printf("        address number) and polarity; a data message that is one of them\n");
	printf("        again within this window is suppressed. Tone and voice pages are\n");
	printf("        always reported.\n");
	printf("    --charset <vendor-model-charset> | list\n");
	printf("        Codepage of alphanumeric messages (Rep. 900-2 Table VII): a\n");
	printf("        character without a code refuses the message. 'list' shows the\n");
	printf("        codepages of GSC. (default %s-%s-%s)\n",
	       golay_charset->vendor, golay_charset->model, golay_charset->charset);
	printf("    --charset-print <vendor-model-charset>\n");
	printf("        Show the table of a codepage: each code, its glyph and the other\n");
	printf("        characters accepted for it on TX.\n");
	printf(" -y --type auto | tone | voice | numeric | alpha\n");
	printf("        Message type for console and scan modes (default auto).\n");
	printf("        'auto' is tone-only without a message, alpha with one.\n");
	printf("        The address number does not set the type: what a pager does\n");
	printf("        with each address number depends on the pager.\n");
	printf("        'tone' sends tone-only alert (no data, no voice).\n");
	printf("        'voice' sends voice page.\n");
	printf("        'numeric' sends BCD-encoded numeric message.\n");
	printf("        'alpha' sends alphanumeric message.\n");
	printf(" -S --scan <from> <to>\n");
	printf("        Scan through given range of 7 digit cap codes (the suffix gives\n");
	printf("        the address number: 1-4 and 5-8 are 1-4, 9 and 0 are 1 and 2).\n");
	printf("        Messages are batch-packed by preamble group for efficiency.\n");
	printf("        Use -y to select message type: numeric (default), alpha, or tone.\n");
	printf("    --fifo-file <file>\n");
	printf("        Read the command lines of the FIFO from a file instead (no FIFO is\n");
	printf("        made): for batch runs with IQ files, where time follows the samples.\n");
	printf("    --afc-range <Hz>\n");
	printf("        RX: how far the receiver follows the carrier offset (AFC): the slicer\n");
	printf("        level follows the signal, at each preamble codeword the offset is\n");
	printf("        measured and the channel's demodulator moved onto the signal.\n");
	printf("        0 = off. Correct larger offsets of the SDR with --sdr-ppm.\n");
	printf("        (default = %.0f)\n", golay_afc_range);
	printf("\n");
	printf("File: %s\n", msg_send_path);
	printf("        Write \"<address>\" to send a tone-only message.\n");
	printf("        Write \"<address>,n,message\" to send a numeric message.\n");
	printf("        Write \"<address>,a,message\" to send an alphanumeric message.\n");
	printf("        Write \"<address>,v,<wave file name>\" to send a voice message.\n");
	printf("        Write \"<address>,t,\" to send a tone-only message.\n");
	printf("        Optional polarity suffix: append ',+' or ',-' as last field.\n");
	printf("        Optional priority prefix: prepend '!' before address.\n");
	printf("        Examples:\n");
	printf("          123456/2             (tone-only, address number 2)\n");
	printf("          123456/1,a,HELLO     (alpha message)\n");
	printf("          123456/3,n,5551234   (numeric message)\n");
	printf("          123456/4,v,alert.wav (voice page)\n");
	printf("          1234567,a,HELLO      (7 digits: suffix 7 is address number 3)\n");
	printf("          !123456/1,a,URGENT   (priority alpha message)\n");
	printf("\n");
	printf("The address is 6 digits (I G1 G0 A2 A1 A0) and the address number 1-4\n");
	printf("(function A-D), written 123456/1. Also accepted: 7 digits (the suffix gives\n");
	printf("the address number: 1-4 and 5-8 are 1-4, 9 and 0 are 1 and 2), 6 digits\n");
	printf("(address number 1) and 5 digits (index 0 in front, address number 1).\n");
	printf("\n");
	printf("A numeric message can have up to 24 digits, they are: 0123456789U-* and space\n");
	printf("Also 'shifted' digits can be sent using two digits, they are: ABCDEFGHJLNPR\n");
	printf("\n");
	printf("An alphanumeric message can have up to 80 digits, sent upper case only.\n");
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
	option_add(0x104, "oneshot", 0);
	option_add(0x108, "fifo-file", 1);
	option_add('S', "scan", 2);
	option_add('y', "type", 1);
	option_add(0x105, "dedup", 1);
	option_add(0x106, "charset", 1);
	option_add(0x107, "charset-print", 1);
	option_add(0x109, "afc-range", 1);
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
	case 0x106:
		if (!strcasecmp(argv[argi], "list")) {
			printf("GSC codepages:\n");
			pager_charset_list_protocol("golay");
			return 0;
		}
		golay_charset = pager_charset_lookup("golay", argv[argi]);
		if (!golay_charset) {
			fprintf(stderr, "'%s' is not a GSC codepage, use '--charset list'.\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case 0x107:
		{
			const struct pager_charset *cs = pager_charset_lookup("golay", argv[argi]);

			if (!cs) {
				fprintf(stderr, "'%s' is not a GSC codepage, use '--charset list'.\n", argv[argi]);
				return -EINVAL;
			}
			pager_charset_print(cs);
		}
		return 0;
	case 'V':
		voice_dir = options_strdup(argv[argi++]);
		break;
	case 0x109: /* --afc-range */
		golay_afc_range = atof(argv[argi]);
		if (golay_afc_range < 0.0) {
			fprintf(stderr, "--afc-range must be 0 (off) or more Hz.\n");
			return -EINVAL;
		}
		break;
	case 0x100: /* --voice-monitor */
		voice_monitor = 1;
		break;
	case 0x101: /* --fifo */
		msg_send_path = options_strdup(argv[argi++]);
		break;
	case 0x108: /* --fifo-file */
		msg_send_file = options_strdup(argv[argi]);
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
	case 0x104: /* --oneshot */
		oneshot = 1;
		tx = 1;
		break;
	case 0x105: /* --dedup */
		dedup_window = atof(argv[argi++]);
		if (dedup_window < 0.0) {
			fprintf(stderr, "Dedup window must be >= 0 seconds.\n");
			return -EINVAL;
		}
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
		if (!strcmp(argv[argi], "numeric") || !strcmp(argv[argi], "n")) {
			scan_type = TYPE_NUMERIC;
			default_msg_type = TYPE_NUMERIC;
		} else if (!strcmp(argv[argi], "alpha") || !strcmp(argv[argi], "a")) {
			scan_type = TYPE_ALPHA;
			default_msg_type = TYPE_ALPHA;
		} else if (!strcmp(argv[argi], "tone") || !strcmp(argv[argi], "t")) {
			scan_type = TYPE_TONE;
			default_msg_type = TYPE_TONE;
		} else if (!strcmp(argv[argi], "voice") || !strcmp(argv[argi], "v")) {
			scan_type = TYPE_VOICE;
			default_msg_type = TYPE_VOICE;
		} else if (!strcmp(argv[argi], "auto")) {
			scan_type = TYPE_NUMERIC;
			default_msg_type = TYPE_AUTO;
		} else {
			fprintf(stderr, "Invalid type '%s', use auto, tone, voice, numeric, or alpha.\n", argv[argi]);
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
	char *line;
	int msg_count = 0;

	/* Process up to 16 complete messages per call to avoid starving
	 * the audio loop when the FIFO is flooded. */
	msg_fifo_read(&msg_fifo);
	while (msg_count < 16 && (line = msg_fifo_line(&msg_fifo))) {
		LOGP(DGOLAY, LOGL_DEBUG, "FIFO: processing msg %d: '%s'\n", msg_count + 1, line);
		if (tx)
			golay_msg_send(line);
		else
			LOGP(DGOLAY, LOGL_ERROR, "Failed to send message, transmitter is not enabled!\n");
		msg_count++;
	}

	/* --oneshot: quit after transmission completes */
	if (oneshot) {
		gsc_t *gsc = (gsc_t *)sender_head;
		if (gsc && gsc->bit_num == 0 &&
		    !gsc->msg_list &&
		    gsc->priority_count == 0 &&
		    gsc->normal_count == 0)
			quit = 1;
	}

	/* Batch with a command file: once every line was taken and nothing is
	 * left to send, end a second later so a receiver of the same run can
	 * finish. */
	{
		sender_t *s;
		int idle = 1;

		for (s = sender_head; s; s = s->next) {
			gsc_t *g = (gsc_t *)s;

			if (g->bit_num != 0 || g->msg_list || g->priority_count != 0 || g->normal_count != 0)
				idle = 0;
		}
		if (msg_fifo_batch_end(&msg_fifo, idle, 1.0))
			quit = 1;
	}
}

static const struct number_lengths number_lengths[] = {
	{ 8, "address 123456/1 (6 digits, / and the address number 1-4)" },
	{ 7, "7 digit cap code (the suffix gives the address number)" },
	{ 6, "6 digit address (address number 1)" },
	{ 5, "5 digit address (index 0, address number 1)" },
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
	main_mobile_init("0123456789/", number_lengths, NULL, NULL, "american");
	main_mobile_set_no_calls();

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

	/* --oneshot conflicts with scan */
	if (oneshot && scan_to > scan_from) {
		fprintf(stderr, "--oneshot and --scan are mutually exclusive.\n");
		goto fail;
	}

	/* TX & RX if loopback */
	if (loopback)
		tx = rx = 1;

	/* the pipe for message send, or the command lines from a file */
	if (msg_fifo_open(&msg_fifo, msg_send_path, msg_send_file) < 0)
		goto fail;

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
			gsc->rx_dedup_window = dedup_window;
			gsc->nbs = nbs_mode;
			gsc->default_msg_type = default_msg_type;
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

	/* --oneshot: enqueue message from CLI args and quit when TX done */
	if (oneshot) {
		if (!station_id[0]) {
			fprintf(stderr, "--oneshot requires an address as positional argument.\n");
			goto fail;
		}
		{
			char fifo_line[4096];
			char type_char = 'a'; /* default auto */
			if (default_msg_type == TYPE_NUMERIC) type_char = 'n';
			else if (default_msg_type == TYPE_ALPHA) type_char = 'a';
			else if (default_msg_type == TYPE_TONE) type_char = 't';
			else if (default_msg_type == TYPE_VOICE) type_char = 'v';
			snprintf(fifo_line, sizeof(fifo_line),
				 "%s,%c,%s", station_id, type_char, message);
			golay_msg_send(fifo_line);
		}
	}

	main_mobile_loop("golay", &quit, myhandler, station_id);

fail:
	/* pipe */
	msg_fifo_close(&msg_fifo);

	/* destroy transceiver instance */
	while(sender_head)
		golay_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();

	options_free();

	return 0;
}

