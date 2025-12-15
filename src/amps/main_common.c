/* AMPS main
 *
 * (C) 2016 by Andreas Eversberg <jolly@eversberg.eu>
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
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "../libsample/sample.h"
#include "../libmobile/main_mobile.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../liboptions/options.h"
#include "../libfm/fm.h"
#include "amps.h"
#include "dsp.h"
#include "frame.h"
#include "stations.h"
#include "main.h"

/* Flash With Info FIFO
 *
 * Format: flash,<number>,<message>[,<pi>,<si>]
 *   number: AMPS phone number (10 digits)
 *   message: Text to send (max 32 chars)
 *   pi: Presentation Indicator 0-2 (optional, default=0)
 *   si: Screening Indicator 0-3 (optional, default=3)
 *
 * Example: echo "flash,1234567890,Hello,0,3" > /tmp/amps_flash
 */
#define AMPS_FLASH_FIFO "/tmp/amps_flash"
static int flash_fd = -1;

/* settings */
int num_chan_type = 0;
enum amps_chan_type chan_type[MAX_SENDER] = { CHAN_TYPE_CC_PC_VC };
const char *flip_polarity = "";
int ms_power = 4;
int dtx = 0;
int send_callerid = 0;
int dcc = 0, scc = 0, sid = 0, regh = 1, regr = 1, pureg = 0, pdreg = 0, locaid = -1, regincr = 300, bis = 0;
int tolerant = 0;
int vmac_enable = 0;
double vmac_level_low = 0.95;
double vmac_level_high = 1.01;

static void print_location_area_note(void)
{
	/*      -                                                                             - */
	printf("	Warning: Older phones may not like this and show 'No Service'!\n");
	printf("	Note: This feature was added 1995 to the standard, it might not work.\n");
}

void print_help(const char *arg0)
{
	if (!tacs)
		main_mobile_print_help(arg0, "-p -d -F yes | no [-S sid=<sid>] ");
	else
		main_mobile_print_help(arg0, "-p -d -F yes | no [-S aid=<aid>] ");
	/*      -                                                                             - */
	printf(" -T --channel-type <channel type> | list\n");
	printf("        Give channel type, use 'list' to get a list. (default = '%s')\n", chan_type_short_name(chan_type[0]));
	printf(" -F --flip-polarity no | yes\n");
	printf("        Flip polarity of transmitted FSK signal. If yes, the sound card\n");
	printf("        generates a negative signal rather than a positive one. Be sure that\n");
	printf("        a positive signal causes a positive deviation on your transmitter.\n");
	printf("        If the phone shows 'NoSrv', try the other way.\n");
	printf(" -P --ms-power <power level>\n");
	printf("        Give maximum power level of the mobile station 0..7. (default = '%d')\n", ms_power);
   if (!tacs) {
	printf("        0 = 4 W;     1 = 1.6 W;   2 = 630 mW;  3 = 250 mW;\n");
	printf("	4 = 100 mW;  5 = 40 mW;   6 = 16 mW;   7 = 6.3 mW\n");
	printf("        (Setting this limits the maximum power, but allows further attenuation with VMAC option)\n");
   } else {
	/* tacs, not jtacs: https://www.academia.edu/8265916/Total_Access_Communication_System?email_work_card=view-paper */
	printf("        0 = 2.28 W;  1 = 1.12 W;  2 = 447 mW;  3 = 178 mW;\n");
	printf("	4 = 70.8 mW; 5 = 28.2 mW; 6 = 11.2 mW; 7 = 4.5 mW\n");
   }
	printf(" -V --vmac-levels <low,high> | default\n");
	printf("        Enable Dynamic Power Control (disabled by default).\n");
	printf("        Specify low,high quality levels (e.g. 95,101) or use 'default'.\n");
	printf(" -D --dtx <parameter>\n");
	printf("        Give DTX parameter for Discontinuous Transmission. (default = '%d')\n", dtx);
	printf("        0 = disable DTX;                     1 = reserved;\n");
	printf("	2 = 8 dB attenuation in low state;   3 = transmitter off\n");
	printf(" -I --caller-id 1 | 0\n");
	printf("        If set, the caller ID is sent while ringing the phone. (default = '%d')\n", send_callerid);
	printf("        Note that this does not work as documented in the specs. If the phone\n");
	printf("        does not support caller ID, it will abort connection on receiving\n");
	printf("        caller ID for some unknown reason. Therefore use caller ID only with\n");
	printf("        phones that support it.\n");
    if (!tacs) {
	printf(" -S --sysinfo sid=<System ID> | sid=list\n");
	printf("        Give system ID of cell broadcast\n");
	printf("        If it changes, phone re-registers. Use 'sid=list' to get a full list.\n");
    } else {
	printf(" -S --sysinfo aid=<System ID> | aid=list\n");
	printf("        Give area ID of cell broadcast (default = '%d')\n", sid);
	printf("        If it changes, phone re-registers. Use 'aid=list' to get a full list.\n");
    }
	printf(" -S --sysinfo dcc=<digital color code>\n");
	printf("        Give digital color code 0..3 (default = '%d')\n", dcc);
	printf(" -S --sysinfo scc=<SAT color code>\n");
	printf("        Give supervisor tone color code 0..2 (default = '%d')\n", scc);
	printf(" -S --sysinfo regincr\n");
	printf("        Amount to add to REGID after successful registration (default = '%d')\n", regincr);
	printf("        Since REGID is incremented every second, this value define after how\n");
	printf("        many second the phone waits before it re-registers.\n");
	printf(" -S --sysinfo pureg=0 | pureg=1\n");
	printf("        If 1, phone registers on every power on (default = '%d')\n", pureg);
	print_location_area_note();
	printf(" -S --sysinfo pdreg=0 | pdreg=1\n");
	printf("        If 1, phone de-registers on every power down (default = '%d')\n", pureg);
	print_location_area_note();
	printf(" -S --sysinfo locaid=<location area ID > | locaid=-1 to disable\n");
	printf("        (default = '%d')\n", locaid);
	printf("        If it changes, phone re-registers.\n");
	print_location_area_note();
	printf(" -S --sysinfo regh=0 | regh=1\n");
	printf("        If 1, phone registers only if System ID matches (default = '%d')\n", regh);
	printf(" -S --sysinfo regr=0 | regr=1\n");
	printf("        If 1, phone registers only if System ID is different (default = '%d')\n", regr);
	printf(" -S --sysinfo bis=0 | bis=1\n");
	printf("        If 0, phone ignores BUSY/IDLE bit on FOCC (default = '%d')\n", bis);
	printf("        If 1, be sure to have a round-trip delay (latency) not more than 5 ms\n");
	printf(" -O --tolerant\n");
	printf("        Be more tolerant when hunting for sync sequence\n");
	printf("\nDialing Prefixes:\n");
	printf("        You can configure alerting parameters per call by prefixing the number:\n");
	printf("        +%s[PI][SI][Pitch][Cadence]xxxxxxxxxx  (or %s...)\n", mobile_amps_param_prefix, mobile_amps_param_prefix);
	printf("        PI: 0=Allowed(Default), 1=Restricted, 2=NotAvail, 3=Reserved\n");
	printf("        SI: 0=Unscreened, 1=Passed, 2=Failed, 3=Network(Default)\n");
	printf("        Pitch: 0=Medium(Default), 1=High, 2=Low, 3=Reserved\n");
	printf("        Cadence: 00=NoTone, 01=Long(Default), 02=ShortShort, 03=ShortShortLong,\n");
	printf("                 04-06=OtherPatterns, 07-11=PBXPatterns\n");
	printf("    --amps-prefix <prefix>\n");
	printf("        Give prefix for alerting parameters. (default = '%s')\n", mobile_amps_param_prefix);
	printf("\nFlash With Info (In-Call Caller ID):\n");
	printf("    To send caller ID during an active call (e.g., call waiting):\n");
	printf("        echo \"flash,<number>,<message>[,<pi>,<si>]\" > /tmp/amps_flash\n");
	printf("    Example: echo \"flash,1234567890,John Doe\" > /tmp/amps_flash\n");
	printf("    PI: 0=Allowed(Default), 1=Restricted, 2=NotAvail\n");
	printf("    SI: 0=Unscreened, 1=Passed, 2=Failed, 3=Network(Default)\n");
	main_mobile_print_station_id();
	main_mobile_print_hotkeys();
}

/* Handler for Flash With Info FIFO commands */
static void amps_myhandler(void)
{
	static char buffer[256];
	static int pos = 0;
	int rc, i, space;
	char *p, *cmd, *number, *message, *pi_str, *si_str;
	int pi = 0, si = 3;  /* defaults: Allowed, Network-provided */

	if (flash_fd < 0)
		return;

	space = sizeof(buffer) - pos;
	rc = read(flash_fd, buffer + pos, space);
	if (rc > 0) {
		pos += rc;
		if (pos == space) {
			fprintf(stderr, "Flash buffer overflow!\n");
			pos = 0;
		}
		/* check for end of line */
		for (i = 0; i < pos; i++) {
			if (buffer[i] == '\r' || buffer[i] == '\n')
				break;
		}
		/* process command */
		if (i < pos) {
			buffer[i] = '\0';
			pos = 0;

			/* Parse: flash,number,message[,pi,si] */
			p = buffer;
			cmd = strsep(&p, ",");
			if (!cmd || strcasecmp(cmd, "flash") != 0) {
				fprintf(stderr, "Invalid command '%s', expected 'flash'\n", cmd ? cmd : "(null)");
				return;
			}
			number = strsep(&p, ",");
			message = strsep(&p, ",");
			if (!number || !message) {
				fprintf(stderr, "Usage: flash,<number>,<message>[,<pi>,<si>]\n");
				return;
			}
			/* optional PI and SI */
			pi_str = strsep(&p, ",");
			si_str = strsep(&p, ",");
			if (pi_str && pi_str[0])
				pi = atoi(pi_str);
			if (si_str && si_str[0])
				si = atoi(si_str);
			/* clamp values */
			if (pi < 0 || pi > 2) pi = 0;
			if (si < 0 || si > 3) si = 3;

			rc = amps_flash_with_info(number, message, pi, si);
			if (rc < 0)
				fprintf(stderr, "Flash With Info failed: %d\n", rc);
		}
	}
}

#define OPT_PREFIX 256

static void add_options(void)
{
	main_mobile_add_options();
	option_add('T', "channel-type", 1);
	option_add('F', "flip-polarity", 1);
	option_add('P', "ms-power", 1);
	option_add('D', "dtx", 1);
	option_add('I', "caller-id", 1);
	option_add('S', "sysinfo", 1);
	option_add('O', "tolerant", 0);
	option_add(OPT_PREFIX, "amps-prefix", 1);
}

static int handle_options(int short_option, int argi, char **argv)
{
	const char *p;
	int rc;

	switch (short_option) {
	case 'T':
		if (!strcmp(argv[argi], "list")) {
			amps_channel_list();
			return 0;
		}
		rc = amps_channel_by_short_name(argv[argi]);
		if (rc < 0) {
			fprintf(stderr, "Error, channel type '%s' unknown. Please use '-t list' to get a list. I suggest to use the default.\n", argv[argi]);
			return -EINVAL;
		}
		OPT_ARRAY(num_chan_type, chan_type, rc)
		break;
	case 'F':
		if (!strcasecmp(argv[argi], "no"))
			flip_polarity = "no";
		else if (!strcasecmp(argv[argi], "yes"))
			flip_polarity = "yes";
		else {
			fprintf(stderr, "Given polarity '%s' is illegal, use '-h' for help!\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case 'P':
		ms_power = atoi(argv[argi]);
		if (ms_power > 7)
			ms_power = 7;
		if (ms_power < 0)
			ms_power = 0;
		break;
	case 'D':
		dtx = atoi(argv[argi]);
		if (dtx > 3)
			dtx = 3;
		if (dtx < 0)
			dtx = 0;
		break;
	case 'I':
		send_callerid = atoi(argv[argi]);
		break;
	case 'S':
		p = strchr(argv[argi], '=');
		if (!p) {
			fprintf(stderr, "Given sysinfo parameter '%s' requires '=' character to set value, use '-h' for help!\n", argv[argi]);
			return -EINVAL;
		}
		p++;
		if (!strncasecmp(argv[argi], "sid=", p - argv[argi])
		 || !strncasecmp(argv[argi], "aid=", p - argv[argi])) {
			if (!strcasecmp(p, "list")) {
				list_stations();
				return 0;
			}
			sid = atoi(p);
			if (sid > 32767)
				sid = 32767;
			if (sid < 0)
				sid = 0;
		} else
		if (!strncasecmp(argv[argi], "dcc=", p - argv[argi])) {
			dcc = atoi(p);
			if (dcc > 3)
				dcc = 3;
			if (dcc < 0)
				dcc = 0;
		} else
		if (!strncasecmp(argv[argi], "scc=", p - argv[argi])) {
			scc = atoi(p);
			if (scc > 2)
				scc = 2;
			if (scc < 0)
				scc = 0;
		} else
		if (!strncasecmp(argv[argi], "regincr=", p - argv[argi])) {
			regincr = atoi(p);
		} else
		if (!strncasecmp(argv[argi], "pureg=", p - argv[argi])) {
			pureg = atoi(p) & 1;
		} else
		if (!strncasecmp(argv[argi], "pdreg=", p - argv[argi])) {
			pdreg = atoi(p) & 1;
		} else
		if (!strncasecmp(argv[argi], "locaid=", p - argv[argi])) {
			locaid = atoi(p);
			if (locaid > 4095)
				locaid = 4095;
		} else
		if (!strncasecmp(argv[argi], "regh=", p - argv[argi])) {
			regh = atoi(p) & 1;
		} else
		if (!strncasecmp(argv[argi], "regr=", p - argv[argi])) {
			regr = atoi(p) & 1;
		} else
		if (!strncasecmp(argv[argi], "bis=", p - argv[argi])) {
			bis = atoi(p) & 1;
		} else {
			fprintf(stderr, "Given sysinfo parameter '%s' unknown, use '-h' for help!\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case 'O':
		tolerant = 1;
		break;
	case 'V':
		vmac_enable = 1;
		if (!strcasecmp(argv[argi], "default")) {
			vmac_level_low = 0.95;
			vmac_level_high = 1.01;
		} else {
			if (sscanf(argv[argi], "%lf,%lf", &vmac_level_low, &vmac_level_high) != 2) {
				fprintf(stderr, "Error parsing VMAC levels '%s'. Format: low,high (e.g. 95,101)\n", argv[argi]);
				return -EINVAL;
			}
			/* Convert integer percentages to ratio if needed */
			if (vmac_level_low > 2.0) vmac_level_low /= 100.0;
			if (vmac_level_high > 2.0) vmac_level_high /= 100.0;
		}
		break;
	case OPT_PREFIX:
		mobile_amps_param_prefix = argv[argi];
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}

extern const struct number_lengths number_lengths[];

int main_amps_tacs(const char *name, int argc, char *argv[], const char *toneset)
{
	int rc, argi;
	const char *station_id = "";
	int polarity;
	int i;

	/* jtacs has only system A, so there are only odd AIDs */
	if (jtacs)
		sid = 1;

	/* override default */
	dsp_samplerate = 96000;

	main_mobile_init("0123456789", number_lengths, number_prefixes, NULL, toneset);

	/* handle options / config file */
	add_options();
    if (!tacs) {
	rc = options_config_file(argc, argv, "~/.osmocom/analog/amps.conf", handle_options);
    } else if (!jtacs) {
	rc = options_config_file(argc, argv, "~/.osmocom/analog/tacs.conf", handle_options);
    } else {
	rc = options_config_file(argc, argv, "~/.osmocom/analog/jtacs.conf", handle_options);
    }
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;


	if (argi < argc) {
		station_id = argv[argi];
		rc = main_mobile_number_ask(station_id, "station ID");
		if (rc)
			return rc;
	}

	if (!num_kanal) {
		printf("No channel (\"Kanal\") is specified, I suggest channel %d.\n\n", (!tacs) ? 333 : ((!jtacs) ? 323 : 418));
		print_help(argv[0]);
		return 0;
	}
	if (use_sdr) {
		/* set device */
		for (i = 0; i < num_kanal; i++)
			dsp_device[i] = "sdr";
		num_device = num_kanal;
		/* set channel types for more than 1 channel */
		if (num_kanal > 1 && num_chan_type == 0) {
			chan_type[0] = CHAN_TYPE_CC_PC;
			for (i = 1; i < num_kanal; i++)
				chan_type[i] = CHAN_TYPE_VC;
			num_chan_type = num_kanal;
		}
	}
	if (num_kanal == 1 && num_device == 0)
		num_device = 1; /* use default */
	if (num_kanal != num_device) {
		fprintf(stderr, "You need to specify as many sound devices as you have channels.\n");
		exit(0);
	}
	if (num_kanal == 1 && num_chan_type == 0)
		num_chan_type = 1; /* use default */
	if (num_kanal != num_chan_type) {
		fprintf(stderr, "You need to specify as many channel types as you have channels.\n");
		exit(0);
	}

	/* check for mandatory CC */
	for (i = 0; i < num_kanal; i++) {
		if (chan_type[i] == CHAN_TYPE_CC || chan_type[i] == CHAN_TYPE_CC_PC || chan_type[i] == CHAN_TYPE_CC_PC_VC)
			break;
	}
	if (i == num_kanal) {
		fprintf(stderr, "You must define at least one CC (control) or combined channel type. Quitting!\n");
		goto fail;
	}
	// NOTE: Variable 'i' from above is used here:
	/* default SID/AID, depending on system */
	if (!sid) {
		if (amps_channel2band(atoi(kanal[i]))[0] == 'A') {
			if (!tacs)
				sid = 1; /* Chicago */
			else
				sid = 2051; /* UK Vodafone */
		} else {
			if (!tacs)
				sid = 40; /* Frisco */
			else
				sid = 3600; /* UK Cellnet */
		}

	}

	if (bis && dsp_buffer > 5) {
		fprintf(stderr, "If you use BUSY/IDLE bit, you need to lower the round-trip delay to 5 ms (--buffer 5).\n");
		exit(0);
	}

	sid_stations(sid);

	/* inits */
	fm_init(fast_math);
	dsp_init();
	init_frame();

	/* check for mandatory PC */
	for (i = 0; i < num_kanal; i++) {
		if (chan_type[i] == CHAN_TYPE_CC_PC || chan_type[i] == CHAN_TYPE_CC_PC_VC)
			break;
	}
	if (i == num_kanal) {
		fprintf(stderr, "You must define at least one PC (paging) or combined channel type. Quitting!\n");
		goto fail;
	}

	/* check for mandatory VC */
	for (i = 0; i < num_kanal; i++) {
		if (chan_type[i] == CHAN_TYPE_VC || chan_type[i] == CHAN_TYPE_CC_PC_VC)
			break;
	}
	if (i == num_kanal)
		fprintf(stderr, "You did not define any VC (voice) channel. You will not be able to make any call.\n");

	/* SDR always requires emphasis */
	if (use_sdr) {
		do_pre_emphasis = 1;
		do_de_emphasis = 1;
	}

	if (!do_pre_emphasis || !do_de_emphasis) {
		fprintf(stderr, "*******************************************************************************\n");
		fprintf(stderr, "I strongly suggest to let me do pre- and de-emphasis (options -p -d)!\n");
		fprintf(stderr, "Use a transmitter/receiver without emphasis and let me do that!\n");
		fprintf(stderr, "Because carrier FSK signaling does not use emphasis, I like to control\n");
		fprintf(stderr, "emphasis by myself for best results.\n");
		fprintf(stderr, "*******************************************************************************\n");
	}

	if (!strcmp(flip_polarity, "no"))
		polarity = 1; /* positive */
	else if (!strcmp(flip_polarity, "yes"))
		polarity = -1; /* negative */
	else if (use_sdr)
		polarity = 1; /* SDR is always positive */
	else {
		fprintf(stderr, "You must define, if the the TX deviation polarity has to be flipped. (-F yes | no) use '-h' for help.\n");
		exit(0);
	}

	/* create transceiver instance */
	for (i = 0; i < num_kanal; i++) {
		amps_si si;

		init_sysinfo(&si, ms_power, ms_power, dtx, dcc, sid >> 1, regh, regr, pureg, pdreg, locaid, regincr, bis);
		rc = amps_create(kanal[i], chan_type[i], dsp_device[i], use_sdr, dsp_samplerate, rx_gain, tx_gain, do_pre_emphasis, do_de_emphasis, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, &si, sid, scc, polarity, send_callerid, tolerant, loopback);
		if (rc < 0) {
			fprintf(stderr, "Failed to create \"Sender\" instance. Quitting!\n");
			goto fail;
		}
		if (!tacs)
			printf("Base station on channel %s ready (%s), please tune transmitter to %.3f MHz and receiver to %.3f MHz. (%.3f MHz offset)\n", kanal[i], chan_type_long_name(chan_type[i]), amps_channel2freq(atoi(kanal[i]), 0) / 1e6, amps_channel2freq(atoi(kanal[i]), 1) / 1e6, amps_channel2freq(atoi(kanal[i]), 2) / 1e6);
		else
			printf("Base station on channel %s ready (%s), please tune transmitter to %.4f MHz and receiver to %.4f MHz. (%.3f MHz offset)\n", kanal[i], chan_type_long_name(chan_type[i]), amps_channel2freq(atoi(kanal[i]), 0) / 1e6, amps_channel2freq(atoi(kanal[i]), 1) / 1e6, amps_channel2freq(atoi(kanal[i]), 2) / 1e6);
	}

	/* Create Flash With Info FIFO */
	unlink(AMPS_FLASH_FIFO);
	rc = mkfifo(AMPS_FLASH_FIFO, 0666);
	if (rc < 0) {
		fprintf(stderr, "Failed to create Flash FIFO '%s'!\n", AMPS_FLASH_FIFO);
		goto fail;
	} else {
		flash_fd = open(AMPS_FLASH_FIFO, O_RDONLY | O_NONBLOCK);
		if (flash_fd < 0) {
			fprintf(stderr, "Failed to open Flash FIFO '%s'!\n", AMPS_FLASH_FIFO);
			goto fail;
		}
	}

	main_mobile_loop(name, &quit, amps_myhandler, station_id);

fail:
	/* FIFO cleanup */
	if (flash_fd > 0)
		close(flash_fd);
	unlink(AMPS_FLASH_FIFO);

	/* destroy transceiver instance */
	while (sender_head)
		amps_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();

	options_free();

	return 0;
}

