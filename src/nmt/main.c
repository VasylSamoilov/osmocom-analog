/* NMT main
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
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "../libsample/sample.h"
#include "../libmobile/main_mobile.h"
#include "../liblogging/logging.h"
#include "../liboptions/options.h"
#include "../libsdr/sdr_config.h"
#include "nmt.h"
#include "frame.h"
#include "dsp.h"
#include "countries.h"

/* SMS FIFOs
 *
 * SMS_DELIVER: FIFO to send SMS to mobile stations
 *   Format: <dest>,<orig>,<message>
 *   Example: echo "3704021,3735859,Hello" > /tmp/nmt_sms_deliver
 *   The coding is chosen automatically: GSM 7-bit for plain ASCII, the NMT
 *   8-bit alphabet (TP-DCS 0x30) as soon as the message contains a byte >=0x80.
 *   Any byte that is not printable ASCII -- including 0x00 and the 8-bit
 *   national letters -- is written in the message as the escape <0xNN>, e.g.
 *     echo "3704021,3735859,caf<0xe9>" > /tmp/nmt_sms_deliver
 *
 * SMS_SUBMIT: File where received SMS from mobile stations are written
 *   Format: <subscriber>,<rp_orig>,<dest>,<message>
 *   Where:
 *     subscriber = Mobile station's NMT number (from call setup)
 *     rp_orig    = RP-Originator-Address from SMS (may be empty)
 *     dest       = Destination address
 *     message    = SMS text; non-printable bytes are rendered as <0xNN>
 */
#define SMS_DELIVER "/tmp/nmt_sms_deliver"
#define SMS_SUBMIT "/tmp/nmt_sms_submit"
static int sms_deliver_fd = -1;

/* MWI (Message Waiting Indicator) control FIFO
 * See docs/NMT_MWI.md for full documentation.
 *
 * Command format: <subscriber>,set,<indicators>
 *   subscriber: 7-digit NMT number
 *   indicators: space-separated text (sms voice fax email data) or numeric 0-31
 *
 * Examples:
 *   echo "3735859,set,sms" > /tmp/nmt_cmd
 *   echo "3735859,set,sms voice" > /tmp/nmt_cmd
 *   echo "3735859,set,3" > /tmp/nmt_cmd
 *
 * Clearing sequence (5c x2 then L(15) x4) is sent per NMT spec 4.4.1.16.
 */
/* UNIFIED COMMAND FIFO — operator control of mid-call and subscriber features.
 * One command per line. Commands:
 *
 *   mwi <subscriber>,set,<indicators>
 *       Set message-waiting indicators on the MS (NMT Doc 450-1 §4.3.3.11).
 *       indicators: space-separated text (sms voice fax email data) or numeric 0-31
 *       Examples:
 *         echo "mwi 3735859,set,sms"       > /tmp/nmt_cmd
 *         echo "mwi 3735859,set,sms voice" > /tmp/nmt_cmd
 *         echo "mwi 3735859,set,3"         > /tmp/nmt_cmd   (bits 0+1 = SMS + voice)
 *
 *   move <subscriber>[,<channel>]
 *       Move an active call to another traffic channel via SCIP (NMT Doc 450-1
 *       §4.4.1.4). Channel omitted = auto-select a free TC.
 *       Examples:
 *         echo "move 3735859"   > /tmp/nmt_cmd   (move to any free TC)
 *         echo "move 3735859,5" > /tmp/nmt_cmd   (move to channel 5)
 *       See docs/NMT_SCIP.md for full detail.
 *
 *   power <subscriber>,<level>
 *       Change MS output power level during a call (NMT Doc 450-1 §4.4.1.8).
 *       level: 0..7
 *       Example:
 *         echo "power 3735859,2" > /tmp/nmt_cmd
 */
#define NMT_CMD_FIFO "/tmp/nmt_cmd"
static int nmt_cmd_fd = -1;

/* settings */
int nmt_system = 450;
int num_chan_type = 0;
enum nmt_chan_type chan_type[MAX_SENDER] = { CHAN_TYPE_CC_TC };
int ms_power = 1; /* 0..3 */
char country[16] = "";
uint8_t traffic_area;
int traffic_area_given = 0;
char area_no = 0;
int compandor = 1;
int num_supervisory = 0;
int supervisory[MAX_SENDER] = { 1 };
const char *smsc_number = "767";
int send_callerid = 0;
int send_clock = 0;
int bs_period = 0;	/* battery saving: H3 period code, 0 = disabled (default) */
int bs_group = 8;	/* battery saving: H7 group code (default 8 = all) */
int sis_demo = 0;	/* SIS authentication demonstrator, off by default */
int coinbox_tariff = -1;	/* coin-box answer tariff class, -1 = disabled (default) */

/* Long-only option ids (>=256 so they never match a single-char '-x' flag) for
 * rare operator-only options that do not warrant a short flag. */
#define OPT_BATTERY_SAVING	256
#define OPT_SIS_DEMO		257
#define OPT_COINBOX_TARIFF	258

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "[-N 900] -Y <traffic area> | list  [-I 1] [-0 1] ");
	/*      -                                                                             - */
	printf(" -N --nmt-system 450/900\n");
	printf("        Give NMT type as first parameter. (default = '%d')\n", nmt_system);
	printf("        Note: This option must be given at first!\n");
	printf(" -T --channel-type <channel type> | list\n");
	printf("        Give channel type, use 'list' to get a list. (default = '%s')\n", chan_type_short_name(nmt_system, chan_type[0]));
	printf(" -P --ms-power <power level>\n");
	printf("        Give power level of the mobile station 0..3. (default = '%d')\n", ms_power);
    if (nmt_system == 450) {
	printf("        NMT-450: 3 = 15 W / 7 W (handheld), 2 = 1.5 W, 1/0 = 150 mW\n");
    } else {
	printf("        NMT-900: 3/2 = 6 W, 1 = 1.5 W, 0 = 150 mW\n");
    }
	printf("        Note: Power level < 2 causes mobile to search longer for a cell.\n");
	printf(" -Y --traffic-area <traffic area> | list\n");
	printf("        NOTE: MUST MATCH WITH YOUR ROAMING SETTINGS IN THE PHONE!\n");
	printf("              Your phone will not connect, if country code is different!\n");
	printf("        Give short country code and traffic area separated by comma.\n");
	printf("        (Example: Give 'SE,1' for Sweden, traffic area 1)\n");
	printf("        Add '!' to force traffic area that is not supported by country.\n");
	printf("        (Example: Give 'B,12!' for Belgium, traffic area 12)\n");
	printf("        Use 'list' to get a list of available country code names\n");
	printf(" -A --area-number <area no> | 0\n");
	printf("        Give area number 1..4 or 0 for no area number. (default = '%d')\n", area_no);
	printf(" -C --compandor 1 | 0\n");
	printf("        Make use of the compandor to reduce noise during call. (default = '%d')\n", compandor);
	printf(" -0 --supervisory 1..4 | 0\n");
	printf("        Use supervisory signal 1..4 to detect loss of signal from mobile\n");
	printf("        station, use 0 to disable. (default = '%d')\n", supervisory[0]);
	printf(" -S --smsc-number <digits>\n");
	printf("        If this number is dialed, the mobile is connected to the SMSC (Short\n");
	printf("        Message Service Center). (default = '%s')\n", smsc_number);
	printf(" -I --caller-id 1 | 0\n");
	printf("        If set, the caller ID is sent while ringing the phone. (default = '%d')\n", send_callerid);
	printf(" -U --clock 1 | 0\n");
	printf("        If set, the current time is transmitted with CC. (default = '%d')\n", send_clock);
	printf("        Note that this works only with pure CC, not with combined CC+TC.\n");
	printf("    --battery-saving <H3 period 0..9>\n");
	printf("        Enable battery saving on the calling channel (NMT Doc 450-1 4.3.3.12).\n");
	printf("        0 = disabled (phones never sleep), 1..9 = sleep period code\n");
	printf("        (24/40/56/72/88/112/168/224/280 frames). (default = '%d')\n", bs_period);
	printf("    --sis-demo 1 | 0\n");
	printf("        Enable the SIS authentication DEMONSTRATOR (frame 7/16 exchange only;\n");
	printf("        there is NO real SIS algorithm, it cannot authenticate). (default = '%d')\n", sis_demo);
	printf("    --coinbox-tariff <class 0..3>\n");
	printf("        Tariff class sent to a coin-box MS at answer (frame 5b, NMT Doc 450-1 4.7.2).\n");
	printf("        Default -1 = disabled (no charging source; the answer sequence is logged\n");
	printf("        but no frame 5b is sent). 0..3 enables the frame-5b answer sequence.\n");
	printf("\nNMT command FIFO: %s\n", NMT_CMD_FIFO);
	printf("  One command per line. Commands:\n");
	printf("  mwi <subscriber>,set,<indicators>\n");
	printf("    Set message-waiting indicators. indicators: 'sms', 'voice', 'fax',\n");
	printf("    'email', 'data' (space-separated) or numeric 0-31.\n");
	printf("    Example: echo \"mwi 3735859,set,sms voice\" > %s\n", NMT_CMD_FIFO);
	printf("  move <subscriber>[,<channel>]\n");
	printf("    Move active call to another TC via SCIP (see docs/NMT_SCIP.md).\n");
	printf("    Example: echo \"move 3735859\"   > %s\n", NMT_CMD_FIFO);
	printf("             echo \"move 3735859,5\" > %s  (specific channel)\n", NMT_CMD_FIFO);
	printf("  power <subscriber>,<level>\n");
	printf("    Change MS output power level (0-7) during a call.\n");
	printf("    Example: echo \"power 3735859,2\" > %s\n", NMT_CMD_FIFO);
	main_mobile_print_station_id();
	main_mobile_print_hotkeys();
}

static void add_options(void)
{
	main_mobile_add_options();
	option_add('N', "nmt-system", 1);
	option_add('T', "channel-type", 1);
	option_add('P', "ms-power", 1);
	option_add('Y', "traffic-area", 1);
	option_add('A', "area-number", 1);
	option_add('C', "compandor", 1);
	option_add('0', "supervisory", 1);
	option_add('S', "smsc-number", 1);
	option_add('I', "caller-id", 1);
	option_add('U', "clock", 1);
	option_add(OPT_BATTERY_SAVING, "battery-saving", 1);
	option_add(OPT_SIS_DEMO, "sis-demo", 1);
	option_add(OPT_COINBOX_TARIFF, "coinbox-tariff", 1);
}

static int handle_options(int short_option, int argi, char **argv)
{
	int rc;
	char *p;
	int super;

	switch (short_option) {
	case 'N':
		nmt_system = atoi(argv[argi]);
		if (nmt_system != 450 && nmt_system != 900) {
			fprintf(stderr, "Error, NMT system type '%s' unknown. Please use '-N 450' for NMT-450 or '-N 900' for NMT-900.\n", argv[argi]);
			return -EINVAL;
		}
		if (nmt_system == 900)
			ms_power = 0;
		if (!option_is_first()) {
			fprintf(stderr, "Please specify the NMT system (-N) as first command line option!\n");
			return -EINVAL;
		}
		break;
	case 'T':
		if (!strcmp(argv[argi], "list")) {
			nmt_channel_list(nmt_system);
			return 0;
		}
		rc = nmt_channel_by_short_name(nmt_system, argv[argi]);
		if (rc < 0) {
			fprintf(stderr, "Error, channel type '%s' unknown. Please use '-t list' to get a list. I suggest to use the default.\n", argv[argi]);
			return -EINVAL;
		}
		OPT_ARRAY(num_chan_type, chan_type, rc)
		break;
	case 'P':
		ms_power = atoi(argv[argi]);
		if (ms_power > 3)
			ms_power = 3;
		if (ms_power < 0)
			ms_power = 0;
		break;
	case 'Y':

		if (!strcmp(argv[argi], "list")) {
			nmt_country_list(nmt_system);
			return 0;
		}
		/* digits */
		strncpy(country, argv[argi], sizeof(country) - 1);
		country[sizeof(country) - 1] = '\0';
		p = strchr(country, ',');
		if (!p) {
			fprintf(stderr, "Illegal traffic area '%s', see '-h' for help\n", argv[argi]);
			return -EINVAL;
		}
		*p++ = '\0';
		rc = nmt_country_by_short_name(nmt_system, country);
		if (rc < 0) {
error_ta:
			fprintf(stderr, "Invalid traffic area '%s', use '-Y list' for a list of valid areas\n", argv[argi]);
			return -EINVAL;
		}
		traffic_area = rc << 4;
		if (p[strlen(p) - 1] != '!') {
			rc = nmt_ta_by_short_name(nmt_system, country, atoi(p));
			if (rc < 0)
				goto error_ta;
		}
		if (atoi(p) < 0 || atoi(p) > 15)
			goto error_ta;
		traffic_area |= (nmt_system == 450) ? nmt_flip_ten(atoi(p)) : atoi(p);
		traffic_area_given = 1;
		break;
	case 'A':
		area_no = argv[argi][0] - '0';
		if (area_no > 4) {
			fprintf(stderr, "Area number '%s' out of range, please use 1..4 or 0 for no area\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case 'C':
		compandor = atoi(argv[argi]);
		break;
	case '0':
		super = atoi(argv[argi]);
		if (super < 0 || super > 4) {
			fprintf(stderr, "Given supervisory signal is wrong, use '-h' for help!\n");
			return -EINVAL;
		}
		OPT_ARRAY(num_supervisory, supervisory, super)
		break;
	case 'S':
		smsc_number = options_strdup(argv[argi]);
		break;
	case 'I':
		send_callerid = atoi(argv[argi]);
		break;
	case 'U':
		send_clock = atoi(argv[argi]);
		break;
	case OPT_BATTERY_SAVING:
		bs_period = atoi(argv[argi]);
		if (bs_period < 0 || bs_period > 9) {
			fprintf(stderr, "Battery-saving period '%s' out of range, use H3 code 0..9 (0 = disabled)\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_SIS_DEMO:
		sis_demo = atoi(argv[argi]);
		break;
	case OPT_COINBOX_TARIFF:
		coinbox_tariff = atoi(argv[argi]);
		if (coinbox_tariff < -1 || coinbox_tariff > 3) {
			fprintf(stderr, "Coin-box tariff class '%s' out of range, use 0..3 (or -1 to disable)\n", argv[argi]);
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
	static char sms_buffer[256];
	static int sms_pos = 0;
	int rc, i, space;

	/* Handle SMS deliver FIFO */
	space = sizeof(sms_buffer) - sms_pos;
	rc = read(sms_deliver_fd, sms_buffer + sms_pos, space);
	if (rc > 0) {
		sms_pos += rc;
		if (sms_pos == space) {
			fprintf(stderr, "SMS buffer overflow!\n");
			sms_pos = 0;
		}
		/* check for end of line */
		for (i = 0; i < sms_pos; i++) {
			if (sms_buffer[i] == '\r' || sms_buffer[i] == '\n')
				break;
		}
		/* send sms */
		if (i < sms_pos) {
			sms_buffer[i] = '\0';
			sms_pos = 0;
			deliver_sms(sms_buffer);
		}
	}

	/* Handle unified NMT command FIFO (/tmp/nmt_cmd).
	 * Commands: "mwi ...", "move ...", "power ..."
	 * One command per line; arguments follow the command name separated by a space.
	 * See the NMT_CMD_FIFO definition at the top of this file for full syntax. */
	if (nmt_cmd_fd >= 0) {
		static char cmd_buffer[512];
		static int  cmd_pos = 0;
		char *cmd, *args;
		space = sizeof(cmd_buffer) - cmd_pos - 1;
		rc = read(nmt_cmd_fd, cmd_buffer + cmd_pos, space);
		if (rc > 0) {
			cmd_pos += rc;
			cmd_buffer[cmd_pos] = '\0';
			/* process all complete lines */
			for (;;) {
				char *nl = NULL;
				int j;
				for (j = 0; j < cmd_pos; j++) {
					if (cmd_buffer[j] == '\r' || cmd_buffer[j] == '\n') {
						nl = cmd_buffer + j;
						break;
					}
				}
				if (!nl) break;
				*nl = '\0';
				cmd = cmd_buffer;
				/* split on first space: command + arguments */
				args = strchr(cmd, ' ');
				if (args) {
					*args++ = '\0';
					while (*args == ' ') args++;
				} else {
					args = cmd + strlen(cmd); /* empty args */
				}
				if (!strcasecmp(cmd, "mwi"))
					deliver_mwi(args);
				else if (!strcasecmp(cmd, "move"))
					deliver_move(args);
				else if (!strcasecmp(cmd, "power"))
					deliver_power_change(args);
				else
					fprintf(stderr, "NMT command FIFO: unknown command '%s' (known: mwi, move, power)\n", cmd);
				/* shift the remaining buffer */
				cmd_pos -= (int)(nl + 1 - cmd_buffer);
				memmove(cmd_buffer, nl + 1, cmd_pos);
			}
			if (cmd_pos >= (int)sizeof(cmd_buffer) - 1) {
				fprintf(stderr, "NMT command FIFO: line too long, discarding buffer\n");
				cmd_pos = 0;
			}
		}
	}
}

/* Write received SMS to file
 * Format: <subscriber>,<rp_orig>,<dest>,<message>
 * See SMS_SUBMIT comment at top of file for field descriptions.
 */
int submit_sms(const char *sms)
{
	FILE *fp;
	int rc;

	fp = fopen(SMS_SUBMIT, "a");
	if (!fp) {
		fprintf(stderr, "Failed to open SMS submit file '%s': %s\n", SMS_SUBMIT, strerror(errno));
		return -1;
	}

	rc = fprintf(fp, "%s\n", sms);
	if (rc < 0) {
		fprintf(stderr, "Failed to write to SMS submit file '%s': %s\n", SMS_SUBMIT, strerror(errno));
		fclose(fp);
		return -1;
	}

	fclose(fp);

	return 0;
}

static const struct number_lengths number_lengths[] = {
	{ 7, "NMT number (1st digit country code)" },
	{ 0, NULL }
};

int main(int argc, char *argv[])
{
	int rc, argi;
	const char *station_id = "";
	int mandatory = 0;
	int i;

	/* init mobile interface */
	main_mobile_init("0123456789", number_lengths, NULL, NULL, "denmark");

	/* handle options / config file */
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/nmt.conf", handle_options);
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
		printf("No channel (\"Kanal\") is specified, I suggest channel 1 (-k 1).\n\n");
		mandatory = 1;
	}
	if (use_sdr) {
		/* set device */
		for (i = 0; i < num_kanal; i++)
			dsp_device[i] = "sdr";
		num_device = num_kanal;
		/* set channel types for more than 1 channel */
		if (num_kanal > 1 && num_chan_type == 0) {
			if (loopback)
				chan_type[0] = CHAN_TYPE_TEST;
			else
				chan_type[0] = CHAN_TYPE_CC;
			for (i = 1; i < num_kanal; i++) {
				if (loopback)
					chan_type[i] = CHAN_TYPE_TEST;
				else
					chan_type[i] = CHAN_TYPE_TC;
			}
			num_chan_type = num_kanal;
		}
		if (num_supervisory == 0) {
			/* set supervisory signal */
			for (i = 0; i < num_kanal; i++)
				supervisory[i] = (i % 4) + 1;
			num_supervisory = num_kanal;
		}
	}
	if (num_kanal == 1 && num_device == 0)
		num_device = 1; /* use default */
	if (num_kanal != num_device) {
		fprintf(stderr, "You need to specify as many sound devices as you have channels.\n");
		return -EINVAL;
	}
	if (num_kanal == 1 && num_chan_type == 0) {
		num_chan_type = 1; /* use default */
		if (loopback)
			chan_type[0] = CHAN_TYPE_TEST;
	}
	if (num_kanal != num_chan_type) {
		fprintf(stderr, "You need to specify as many channel types as you have channels.\n");
		return -EINVAL;
	}
	if (num_kanal == 1 && num_supervisory == 0)
		num_supervisory = 1; /* use default */
	if (num_kanal != num_supervisory) {
		fprintf(stderr, "You need to specify as many supervisory signals as you have channels.\n");
		fprintf(stderr, "They shall be different at channels that are close to each other.\n");
		return -EINVAL;
	}
	if (num_kanal) {
		uint8_t super[5] = { 0, 0, 0, 0, 0 };

		for (i = 0; i < num_kanal; i++) {
			if (supervisory[i] == 0) {
				fprintf(stderr, "No supervisory signal given for channel %s. This is ok, but signal loss dannot be detected. \n", kanal[i]);
				continue;
			}
			if (super[supervisory[i]]) {
				fprintf(stderr, "Supervisory signal %d is selected for both cannels #%s and #%d. I advice to use different signal, to avoid co-channel interferences.\n", supervisory[i], kanal[i], super[supervisory[i]]);
				continue;
			}
			super[supervisory[i]] = atoi(kanal[i]);
		}
	}

	if (!traffic_area_given) {
		printf("No traffic area is specified, I suggest to use Sweden (-Y SE,1) and set the phone's roaming to 'SE' also.\n\n");
		mandatory = 1;
	}

	if (mandatory) {
		print_help(argv[0]);
		return 0;
	}

	/* create pipe for SMS delivery */
	unlink(SMS_DELIVER);
	rc = mkfifo(SMS_DELIVER, 0666);
	if (rc < 0) {
		fprintf(stderr, "Failed to create SMS deliver FIFO '%s'!\n", SMS_DELIVER);
		goto fail;
	} else {
		sms_deliver_fd = open(SMS_DELIVER, O_RDONLY | O_NONBLOCK);
		if (sms_deliver_fd < 0) {
			fprintf(stderr, "Failed to open SMS deliver FIFO! '%s'\n", SMS_DELIVER);
			goto fail;
		}
	}

	/* create unified NMT command FIFO (see NMT_CMD_FIFO definition for command syntax) */
	unlink(NMT_CMD_FIFO);
	rc = mkfifo(NMT_CMD_FIFO, 0666);
	if (rc < 0) {
		fprintf(stderr, "Failed to create NMT command FIFO '%s'!\n", NMT_CMD_FIFO);
		goto fail;
	} else {
		nmt_cmd_fd = open(NMT_CMD_FIFO, O_RDONLY | O_NONBLOCK);
		if (nmt_cmd_fd < 0) {
			fprintf(stderr, "Failed to open NMT command FIFO '%s'!\n", NMT_CMD_FIFO);
			goto fail;
		}
	}

	/* create unified NMT command FIFO (see NMT_CMD_FIFO for command syntax) */
	unlink(NMT_CMD_FIFO);
	rc = mkfifo(NMT_CMD_FIFO, 0666);
	if (rc < 0) {
		fprintf(stderr, "Failed to create NMT command FIFO '%s'!\n", NMT_CMD_FIFO);
		goto fail;
	} else {
		nmt_cmd_fd = open(NMT_CMD_FIFO, O_RDONLY | O_NONBLOCK);
		if (nmt_cmd_fd < 0) {
			fprintf(stderr, "Failed to open NMT command FIFO '%s'!\n", NMT_CMD_FIFO);
			goto fail;
		}
	}

	/* inits */
	fm_init(fast_math);
	rc = init_frame();
	if (rc < 0) {
		fprintf(stderr, "Failed to setup frames. Quitting!\n");
		return -1;
	}
	dsp_init();

	/* SDR always requires emphasis */
	if (use_sdr) {
		do_pre_emphasis = 1;
		do_de_emphasis = 1;
	}

	/* create transceiver instance */
	for (i = 0; i < num_kanal; i++) {
		rc = nmt_create(nmt_system, country, kanal[i], chan_type[i], dsp_device[i], use_sdr, dsp_samplerate, rx_gain, tx_gain, do_pre_emphasis, do_de_emphasis, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, ms_power, traffic_area, area_no, compandor, supervisory[i], smsc_number, send_callerid, send_clock, bs_period, bs_group, sis_demo, coinbox_tariff, loopback);
		if (rc < 0) {
			fprintf(stderr, "Failed to create transceiver instance. Quitting!\n");
			goto fail;
		}
		printf("base station on channel %s ready, please tune transmitter to %.4f MHz and receiver to %.4f MHz. (%.3f MHz offset)\n", kanal[i], nmt_channel2freq(nmt_system, country, atoi(kanal[i]), 0, NULL, NULL, NULL) / 1e6, nmt_channel2freq(nmt_system, country, atoi(kanal[i]), 1, NULL, NULL, NULL) / 1e6, nmt_channel2freq(nmt_system, country, atoi(kanal[i]), 2, NULL, NULL, NULL) / 1e6);
	}

	nmt_check_channels(nmt_system);

	main_mobile_loop("nmt", &quit, myhandler, station_id);

fail:
	/* fifo cleanup */
	if (sms_deliver_fd > 0)
		close(sms_deliver_fd);
	unlink(SMS_DELIVER);
	if (nmt_cmd_fd > 0)
		close(nmt_cmd_fd);
	unlink(NMT_CMD_FIFO);

	/* destroy transceiver instance */
	while (sender_head)
		nmt_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();

	options_free();

	return 0;
}


/* Full-duplex application: reject TX-only or RX-only modes, but allow split devices */
int sdr_check_separate_device_support(int tx_only, int rx_only, int split_mode)
{
	(void)split_mode; /* split mode is fine - we just need both TX and RX */
	if (tx_only || rx_only)
		return -1;
	return 0;
}
