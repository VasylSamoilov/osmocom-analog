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
#include "scheduler.h"

#define MSG_SEND "/tmp/flex_msg_send"
static int msg_send_fd = -1;

static int tx = 0;
static double deviation = 4800;
static double polarity = -1;
static enum flex_msg_type msg_type = FLEX_MSG_TYPE_AUTO;
static const char *message = "1234";
static uint64_t scan_from = 0;
static uint64_t scan_to = 0;

/* STD-43A compliance options */
static int network_mode = 0;
static int collapse = 0;
static int fixed_speed = -1;		/* -1 = not fixed */
static int fixed_mod_type = FLEX_MOD_2FSK;	/* default 2FSK */
static double fixed_polarity = 0.0;	/* 0.0 = not fixed */
static int lpf_enabled = 1;
static int biw_time_enabled = -1;	/* -1 = auto (on in network mode) */
static int ers_cycles_override = -1;
static int default_charset = 0;
static uint32_t ssid = 0;
static uint32_t nid = 0;
static uint32_t country_code = 0;
static uint32_t tmf_flags = 0;
static int timezone_code = -1;		/* -1 = none, 0-31 = 5-bit zone code (Z4..Z0) */
static const char *pocsag_mix = NULL;
static const char *temp_addr = NULL;
static int no_ers = 0;
static int default_phase = -1;		/* -1=auto (scheduler), 0=A, 1=B, 2=C, 3=D */
static int default_blocking_length = 1;	/* HEX/Binary B field: 1-15 bits/char, 0=16.
					 * Default 1 (raw bits). */
static int wav_test = 0;		/* --wav-test: exit after TX completes */
static int num_transmissions = 1;	/* --num-transmissions: 1/2/3/4 */
static int td_collapse = -1;		/* --td-collapse: -1=system, 5/6/7 */
static int chan_setup_enabled = 0;	/* --chan-setup: enable BIW channel setup */
int rx_kanji_enabled = 0;		/* --rx-kanji: enable Kanji/Shift-JIS decode */

/* Long-only option IDs (3000+ range to avoid conflicts with main_mobile) */
#define OPT_NETWORK		3000
#define OPT_COLLAPSE		3001
#define OPT_SPEED		3002
#define OPT_FIXED_POLARITY	3003
#define OPT_LPF		3004
#define OPT_NO_LPF		3005
#define OPT_BIW_TIME		3006
#define OPT_NO_BIW_TIME	3007
#define OPT_ERS_CYCLES		3008
#define OPT_CHARSET		3009
#define OPT_SSID		3010
#define OPT_NID			3011
#define OPT_POCSAG_MIX		3012
#define OPT_TEMP_ADDR		3013
#define OPT_NO_ERS		3014
#define OPT_PHASE		3015
#define OPT_WAV_TEST		3016
#define OPT_BLOCKING		3017
#define OPT_COUNTRY_CODE	3018
#define OPT_TMF			3019
#define OPT_TIMEZONE		3020
#define OPT_NUM_TX		3021
#define OPT_TD_COLLAPSE		3022
#define OPT_CHAN_SETUP		3023
#define OPT_RX_KANJI		3024

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "-k <frequency>");
	printf(" -T --tx\n");
	printf("        Transmit FLEX signal on given channel. (default)\n");
	printf(" -D --deviation <KHz>\n");
	printf("        Choose deviation of FSK signal (default %.0f Hz).\n", deviation);
	printf(" -P --polarity -1 | negative | 1 | positive\n");
	printf("        Choose polarity of FSK signal. (default %s).\n", (polarity < 0) ? "negative" : "positive");
	printf(" -y --type auto | tone | numeric | alpha | hex | instruction | short | secure | special | snum | nnumeric | nnum | nspecial | nsnum\n");
	printf("        Set message type. (default auto)\n");
	printf("        Message types (V2V1V0 vector type field):\n");
	printf("          tone        — V=010: alert only, no message body\n");
	printf("          numeric     — V=011: 4-bit BCD digits, 5 chars/word\n");
	printf("          alpha       — V=101: 7-bit alphanumeric (3 chars/word)\n");
	printf("          hex         — V=110: raw hex/binary data\n");
	printf("          instruction — V=001: 14-bit short instruction word\n");
	printf("          short       — V=010: short message index (0-127)\n");
	printf("          secure      — V=000: operator-controlled secure message (alpha or binary body)\n");
	printf("          special     — V=100: special format numeric (BCD, ID-ROM display) [alias: snum]\n");
	printf("          nnumeric    — V=111 S=0: like numeric, with duplicate detection/sequencing [alias: nnum]\n");
	printf("          nspecial    — V=111 S=1: like special, with duplicate detection/sequencing [alias: nsnum]\n");
	printf("          auto        — detect from message content\n");
	printf("        RX output tags: ALN=alpha, SEC=secure(V=000),\n");
	printf("          NUM=standard numeric, SNUM=special format,\n");
	printf("          NNUM=numbered numeric, NSNUM=numbered special,\n");
	printf("          HEX=hex/binary, TON=tone, INS=instruction\n");
	printf(" -M --message \"...\"\n");
	printf("        Default message text. (default \"%s\").\n", message);
	printf(" -S --scan <from> <to>\n");
	printf("        Scan through given capcode range.\n");
	printf("    --network\n");
	printf("        Enable network mode (continuous operation).\n");
	printf("    --collapse <0-7>\n");
	printf("        Set collapse cycle value (default %d, network mode only).\n", collapse);
	printf("    --speed <1600|3200|3200-4fsk|6400>\n");
	printf("        Lock transmitter to fixed baud rate (fixed-mode).\n");
	printf("    --fixed-polarity <neg|pos>\n");
	printf("        Lock transmitter to fixed FSK polarity (fixed-mode).\n");
	printf("    --lpf\n");
	printf("        Enable baseband low-pass filter (default).\n");
	printf("    --no-lpf\n");
	printf("        Disable baseband low-pass filter.\n");
	printf("    --biw-time\n");
	printf("        Enable BIW3/BIW4 time broadcast.\n");
	printf("    --no-biw-time\n");
	printf("        Disable BIW3/BIW4 time broadcast.\n");
	printf("    --no-ers\n");
	printf("        Skip ERS burst in single-shot mode (workaround for decoders\n");
	printf("        that choke on ERS sync patterns, e.g. PDW).\n");
	printf("    --ers-cycles <N>\n");
	printf("        Override ERS cycle count (default auto).\n");
	printf("    --charset <ascii|kanji>\n");
	printf("        Set default character set (default ascii).\n");
	printf("    --ssid <N>\n");
	printf("        Set System Sub-ID for roaming (default 0).\n");
	printf("    --nid <N>\n");
	printf("        Set Network ID for roaming (default 0).\n");
	printf("    --country-code <N>\n");
	printf("        Set SSID2 country code (ITU-T E.212, 0-1023, e.g. 440=Japan).\n");
	printf("    --tmf <N>\n");
	printf("        Set SSID2 traffic management flags (0-15, 4-bit bitmask).\n");
	printf("    --timezone <N>\n");
	printf("        Set timezone zone code (0-31, 5-bit Z4..Z0).\n");
	printf("        Emitted as BIW SysInfo type 101 (A=4). Use 'timezone,0,,' FIFO\n");
	printf("        command to list all zone codes. Common: 9=JST, 24=PST, 27=EST.\n");
	printf("    --pocsag-mix <frames>\n");
	printf("        Enable POCSAG frame slot allocation.\n");
	printf("    --temp-addr <cap:temp>\n");
	printf("        Assign temporary address (format capcode:temp_addr).\n");
	printf("    --phase <A|B|C|D|auto>\n");
	printf("        Force message onto a specific phase (channel). (default auto)\n");
	printf("        In FLEX, a 'phase' is an independent data channel within a frame.\n");
	printf("        Multi-phase modes transmit several channels simultaneously:\n");
	printf("          1600/2FSK: 1 phase  (A only, --phase ignored)\n");
	printf("          3200/2FSK: 2 phases (A, C)\n");
	printf("          3200/4FSK: 2 phases (A, C)\n");
	printf("          6400/4FSK: 4 phases (A, B, C, D)\n");
	printf("        With 'auto', the scheduler assigns phase from capcode.\n");
	printf("        FIFO option: phase=A|B|C|D|auto\n");
	printf("    --wav-test\n");
	printf("        Exit after TX completes (use with --write-tx-wave).\n");
	printf("    --blocking <0-16>\n");
	printf("        HEX/Binary blocking length: bits per character.\n");
	printf("        1-15 = that many bits/char, 0 or 16 = 16 bits/char.\n");
	printf("        Default 1 (raw bits). Stored as B field in header word 2.\n");
	printf("    --num-transmissions <1-4>\n");
	printf("        Multiple transmission count (subframe repeat).\n");
	printf("        1=single (default), 2/3/4=subframe repeat.\n");
	printf("        Subframe words: 2x=44, 3x=29, 4x=22. Requires --network.\n");
	printf("    --td-collapse <5|6|7>\n");
	printf("        TD Collapse cycle override for multiple transmission.\n");
	printf("        Overrides system --collapse for repeat interval calculation.\n");
	printf("    --chan-setup\n");
	printf("        Enable BIW Channel Setup instruction (A-type 0x06) emission.\n");
	printf("        Emits frame offset, carry-on, NID, and system message bits.\n");
	printf("    --rx-kanji\n");
	printf("        Enable Kanji/Shift-JIS 16-bit character decode for RX alpha messages.\n");
	printf("        Output tagged ALN:KNJ instead of ALN.\n");
	printf("\n");
	printf("    FIFO protocol (write to %s):\n", MSG_SEND);
	printf("        Format: capcode,type,options,message\n");
	printf("        Types:  auto|tone|numeric|alpha|hex|instruction|short|secure|special|nnumeric|nspecial (or 0-10)\n");
	printf("        Options: space-separated key=value pairs:\n");
	printf("          speed=1600|3200|3200-4fsk|6400  polarity=neg|pos\n");
	printf("          priority=0|1  charset=ascii|kanji\n");
	printf("          group=0|1  tempgroup=0|1  source=<id>\n");
	printf("          phase=A|B|C|D|auto\n");
	printf("          blocking=0-16  (hex bits/char: 0 or 16=16bit, 1=raw, default 1)\n");
	printf("          maildrop=0|1  (alpha/hex: separate handling from ordinary msgs)\n");
	printf("          sectype=alpha|binary  (secure: wire encoding of body, default alpha)\n");
	printf("          sectag=0|1|2|3  (secure: pager type tag, 0=alpha 1=vendor 2=binary 3=rsvd, default=sectype)\n");
	printf("          msgnum=0-63  (nnumeric/nspecial: sequence number for dedup, default auto)\n");
	printf("          chan_setup=0|1  (enable/disable BIW channel setup emission)\n");
	printf("        Special: ers,0,,  — trigger ERS re-sync burst\n");
	printf("        Special: status,0,,  — dump system config + temp group assignments\n");
	printf("        Special: timezone,0,,  — dump timezone table (32 entries)\n");
	printf("        Special: timezone,<code>,,  — show offset for zone code 0-31\n");
	printf("        Special: sysmsg,<lsb>,,<payload>  — send system message via\n");
	printf("          Operator Messaging Address (capcode 0x1F7800-0x1F780F).\n");
	printf("          LSB: 0=all 1=home 2=roaming 3=ssid 4=time 14=SSIDChange 15=SysEvent\n");
	printf("        Examples:\n");
	printf("          1234567,alpha,,Hello World\n");
	printf("          1234567,alpha,speed=3200 priority=1,Hello World\n");
	printf("          1234567,numeric,,31415926\n");
	printf("          1234567,hex,blocking=8,DEADBEEF\n");
	printf("          1234567,tone,,\n");
	printf("          1234567,instruction,,42\n");
	printf("          1234567,short,,5\n");
	printf("          1234567,secure,,Secure alpha message\n");
	printf("          1234567,secure,sectype=binary sectag=2,DEADBEEF\n");
	printf("          1234567,special,,31415926\n");
	printf("          1234567,nnumeric,msgnum=7,31415926\n");
	printf("          1234567,nspecial,,31415926\n");
	printf("          group:1234567,alpha,group=1,Group message\n");
	printf("          group:1234567,alpha,group=1 tempgroup=1,Temp group msg\n");
	printf("          sysmsg,0,,System maintenance at 03:00 UTC\n");
	printf("          sysmsg,14,,\n");
	printf("\n");
	printf("    Valid capcode ranges:\n");
	printf("        Short:  1–1933312 (1 address word, 21-bit d0-d20)\n");
	printf("        Long:   2101249–4297068542 (2 address words, d0-d20 + e0-e20)\n");
	printf("          Set 1-2: 2101249–1075843072     (LA1+LA2)\n");
	printf("          Set 1-3/1-4: 1075843073–3223326720 (LA1+LA3 or LA1+LA4)\n");
	printf("          Set 2-3/2-4: 3223326721–4297068542 (LA2+LA3 or LA2+LA4)\n");
	printf("        Special addresses (gap 1933313–2101248):\n");
	printf("          Network:  NID system info (4096 addrs, area_id/zones/traffic)\n");
	printf("          Temporary: 16 group slots (assigned via short instruction)\n");
	printf("          Operator:  system messages (all/home/roaming/ssid/time),\n");
	printf("                     SSID change (TMF split, new coverage frequencies),\n");
	printf("                     system event pre-alert (change within 4 cycles)\n");
	printf("          Info Svc:  reserved (under investigation)\n");
	printf("        Special addresses are allowed but will log a warning.\n");
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
	option_add(OPT_NETWORK, "network", 0);
	option_add(OPT_COLLAPSE, "collapse", 1);
	option_add(OPT_SPEED, "speed", 1);
	option_add(OPT_FIXED_POLARITY, "fixed-polarity", 1);
	option_add(OPT_LPF, "lpf", 0);
	option_add(OPT_NO_LPF, "no-lpf", 0);
	option_add(OPT_BIW_TIME, "biw-time", 0);
	option_add(OPT_NO_BIW_TIME, "no-biw-time", 0);
	option_add(OPT_ERS_CYCLES, "ers-cycles", 1);
	option_add(OPT_CHARSET, "charset", 1);
	option_add(OPT_SSID, "ssid", 1);
	option_add(OPT_NID, "nid", 1);
	option_add(OPT_COUNTRY_CODE, "country-code", 1);
	option_add(OPT_TMF, "tmf", 1);
	option_add(OPT_TIMEZONE, "timezone", 1);
	option_add(OPT_POCSAG_MIX, "pocsag-mix", 1);
	option_add(OPT_TEMP_ADDR, "temp-addr", 1);
	option_add(OPT_NO_ERS, "no-ers", 0);
	option_add(OPT_PHASE, "phase", 1);
	option_add(OPT_WAV_TEST, "wav-test", 0);
	option_add(OPT_BLOCKING, "blocking", 1);
	option_add(OPT_NUM_TX, "num-transmissions", 1);
	option_add(OPT_TD_COLLAPSE, "td-collapse", 1);
	option_add(OPT_CHAN_SETUP, "chan-setup", 0);
	option_add(OPT_RX_KANJI, "rx-kanji", 0);
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
		else if (!strcasecmp(argv[argi], "hex") || !strcmp(argv[argi], "4"))
			msg_type = FLEX_MSG_TYPE_HEX;
		else if (!strcasecmp(argv[argi], "instruction") || !strcmp(argv[argi], "5"))
			msg_type = FLEX_MSG_TYPE_INSTRUCTION;
		else if (!strcasecmp(argv[argi], "short") || !strcmp(argv[argi], "6"))
			msg_type = FLEX_MSG_TYPE_SHORT;
		else if (!strcasecmp(argv[argi], "secure") || !strcmp(argv[argi], "7"))
			msg_type = FLEX_MSG_TYPE_SECURE;
		else if (!strcasecmp(argv[argi], "special") || !strcasecmp(argv[argi], "snum") || !strcmp(argv[argi], "8"))
			msg_type = FLEX_MSG_TYPE_SPECIAL_NUM;
		else if (!strcasecmp(argv[argi], "nnumeric") || !strcasecmp(argv[argi], "nnum") || !strcmp(argv[argi], "9"))
			msg_type = FLEX_MSG_TYPE_NUMBERED_NUM;
		else if (!strcasecmp(argv[argi], "nspecial") || !strcasecmp(argv[argi], "nsnum") || !strcmp(argv[argi], "10"))
			msg_type = FLEX_MSG_TYPE_NUMBERED_SPECIAL;
		else {
			fprintf(stderr, "Given type is invalid. Use auto/tone/numeric/alpha/hex/instruction/short/secure/special/nnumeric/nspecial.\n");
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
	case OPT_NETWORK:
		network_mode = 1;
		break;
	case OPT_COLLAPSE:
		collapse = atoi(argv[argi]);
		if (collapse < 0 || collapse > 7) {
			fprintf(stderr, "Collapse value must be 0-7, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_SPEED:
		if (!strcmp(argv[argi], "1600")) {
			fixed_speed = 1600;
			fixed_mod_type = FLEX_MOD_2FSK;
		} else if (!strcmp(argv[argi], "3200")) {
			fixed_speed = 3200;
			fixed_mod_type = FLEX_MOD_2FSK;
		} else if (!strcmp(argv[argi], "3200-4fsk")) {
			fixed_speed = 3200;
			fixed_mod_type = FLEX_MOD_4FSK;
		} else if (!strcmp(argv[argi], "6400")) {
			fixed_speed = 6400;
			fixed_mod_type = FLEX_MOD_4FSK;
		} else {
			fprintf(stderr, "Speed must be 1600, 3200, 3200-4fsk, or 6400, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_FIXED_POLARITY:
		if (argv[argi][0] == 'n' || argv[argi][0] == 'N')
			fixed_polarity = -1.0;
		else if (argv[argi][0] == 'p' || argv[argi][0] == 'P')
			fixed_polarity = 1.0;
		else {
			fprintf(stderr, "Polarity must be neg or pos, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_LPF:
		lpf_enabled = 1;
		break;
	case OPT_NO_LPF:
		lpf_enabled = 0;
		break;
	case OPT_BIW_TIME:
		biw_time_enabled = 1;
		break;
	case OPT_NO_BIW_TIME:
		biw_time_enabled = 0;
		break;
	case OPT_ERS_CYCLES:
		ers_cycles_override = atoi(argv[argi]);
		if (ers_cycles_override < 1) {
			fprintf(stderr, "ERS cycles must be >= 1, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_CHARSET:
		if (!strcasecmp(argv[argi], "ascii"))
			default_charset = 0;
		else if (!strcasecmp(argv[argi], "kanji"))
			default_charset = 1;
		else {
			fprintf(stderr, "Charset must be ascii or kanji, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_SSID:
		ssid = (uint32_t)strtoul(argv[argi], NULL, 10);
		break;
	case OPT_NID:
		nid = (uint32_t)strtoul(argv[argi], NULL, 10);
		break;
	case OPT_COUNTRY_CODE:
		country_code = (uint32_t)strtoul(argv[argi], NULL, 10);
		if (country_code > 1023) {
			fprintf(stderr, "Country code must be 0-1023 (ITU-T E.212), use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_TMF:
		tmf_flags = (uint32_t)strtoul(argv[argi], NULL, 10);
		if (tmf_flags > 15) {
			fprintf(stderr, "TMF must be 0-15 (4-bit bitmask), use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_TIMEZONE:
		timezone_code = atoi(argv[argi]);
		if (timezone_code < 0 || timezone_code >= (int)FLEX_TZ_ENTRIES) {
			fprintf(stderr, "Timezone code must be 0-31 (5-bit Z4..Z0), use '-h' for help.\n");
			return -EINVAL;
		}
		if (timezone_code == (int)FLEX_TZ_RESERVED) {
			fprintf(stderr, "Warning: timezone zone code 16 is reserved in the standard.\n");
		}
		break;
	case OPT_POCSAG_MIX:
		pocsag_mix = options_strdup(argv[argi]);
		break;
	case OPT_TEMP_ADDR:
		temp_addr = options_strdup(argv[argi]);
		break;
	case OPT_NO_ERS:
		no_ers = 1;
		break;
	case OPT_PHASE:
		if (!strcasecmp(argv[argi], "auto") || !strcmp(argv[argi], "-1"))
			default_phase = -1;
		else if (!strcasecmp(argv[argi], "a") || !strcmp(argv[argi], "0"))
			default_phase = 0;
		else if (!strcasecmp(argv[argi], "b") || !strcmp(argv[argi], "1"))
			default_phase = 1;
		else if (!strcasecmp(argv[argi], "c") || !strcmp(argv[argi], "2"))
			default_phase = 2;
		else if (!strcasecmp(argv[argi], "d") || !strcmp(argv[argi], "3"))
			default_phase = 3;
		else {
			fprintf(stderr, "Phase must be A, B, C, D, or auto, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_WAV_TEST:
		wav_test = 1;
		break;
	case OPT_BLOCKING:
		default_blocking_length = atoi(argv[argi]);
		if (default_blocking_length < 0 || default_blocking_length > 16) {
			fprintf(stderr, "Blocking length must be 0-16 (0 and 16 both mean 16 bits/char), use '-h' for help.\n");
			return -EINVAL;
		}
		/* Normalize: 16 maps to B=0000 (stored as 0 internally) */
		if (default_blocking_length == 16)
			default_blocking_length = 0;
		break;
	case OPT_NUM_TX:
		num_transmissions = atoi(argv[argi]);
		if (num_transmissions < 1 || num_transmissions > 4) {
			fprintf(stderr, "Number of transmissions must be 1-4, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_TD_COLLAPSE:
		td_collapse = atoi(argv[argi]);
		if (td_collapse != 5 && td_collapse != 6 && td_collapse != 7) {
			fprintf(stderr, "TD collapse must be 5, 6, or 7, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_CHAN_SETUP:
		chan_setup_enabled = 1;
		break;
	case OPT_RX_KANJI:
		rx_kanji_enabled = 1;
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}


/* Parse FIFO options field: space-separated key=value pairs.
 * Sets per-message parameters from parsed values; unset keys keep defaults. */
static void parse_fifo_options(const char *opts, int opts_len,
			       int *speed, int *modulation_type,
			       double *polarity_out, int *priority,
			       int *charset, int *is_group, int *is_temp_group,
			       char *source_id, int *phase, int *blocking_length,
			       int *mail_drop,
			       int *secure_encoding, int *secure_subtype,
			       int *numbered_msgnum,
			       int *msg_chan_setup)
{
	char buf[256];
	char *p, *key, *val;
	int len;

	/* defaults */
	*speed = 1600;
	*modulation_type = FLEX_MOD_2FSK;
	*polarity_out = -1.0;
	*priority = 0;
	*charset = 0;
	/* is_group already set from group: prefix */
	*is_temp_group = 0;
	source_id[0] = '\0';
	*phase = -1;
	*blocking_length = default_blocking_length;
	*mail_drop = 0;
	*secure_encoding = 0;	/* wire encoding: 0=7-bit alpha, 1=raw binary */
	*secure_subtype = -1;	/* pager-side type tag; -1 = derive from encoding */
	*numbered_msgnum = -1;	/* sequence number for dedup; -1 = auto from counter */
	*msg_chan_setup = -1;	/* -1 = not set (use global default) */

	if (opts_len <= 0)
		return;

	len = opts_len;
	if (len >= (int)sizeof(buf))
		len = sizeof(buf) - 1;
	memcpy(buf, opts, len);
	buf[len] = '\0';

	p = buf;
	while (*p) {
		/* skip spaces */
		while (*p == ' ')
			p++;
		if (!*p)
			break;

		key = p;
		val = NULL;
		while (*p && *p != '=' && *p != ' ')
			p++;
		if (*p == '=') {
			*p++ = '\0';
			val = p;
			while (*p && *p != ' ')
				p++;
			if (*p)
				*p++ = '\0';
		} else {
			if (*p)
				*p++ = '\0';
		}

		if (!val)
			continue;

		if (!strcmp(key, "speed")) {
			if (!strcmp(val, "3200-4fsk")) {
				*speed = 3200;
				*modulation_type = FLEX_MOD_4FSK;
			} else if (!strcmp(val, "6400")) {
				*speed = 6400;
				*modulation_type = FLEX_MOD_4FSK;
			} else if (!strcmp(val, "3200")) {
				*speed = 3200;
				*modulation_type = FLEX_MOD_2FSK;
			} else {
				*speed = atoi(val);
				*modulation_type = FLEX_MOD_2FSK;
			}
		}
		else if (!strcmp(key, "polarity")) {
			if (val[0] == 'n' || val[0] == 'N')
				*polarity_out = -1.0;
			else if (val[0] == 'p' || val[0] == 'P')
				*polarity_out = 1.0;
		}
		else if (!strcmp(key, "priority"))
			*priority = atoi(val);
		else if (!strcmp(key, "charset")) {
			if (!strcasecmp(val, "kanji"))
				*charset = 1;
			else
				*charset = 0;
		}
		else if (!strcmp(key, "group"))
			*is_group = atoi(val);
		else if (!strcmp(key, "tempgroup"))
			*is_temp_group = atoi(val);
		else if (!strcmp(key, "source")) {
			strncpy(source_id, val, 63);
			source_id[63] = '\0';
		}
		else if (!strcmp(key, "phase")) {
			if (!strcasecmp(val, "auto") || !strcmp(val, "-1"))
				*phase = -1;
			else if (!strcasecmp(val, "a") || !strcmp(val, "0"))
				*phase = 0;
			else if (!strcasecmp(val, "b") || !strcmp(val, "1"))
				*phase = 1;
			else if (!strcasecmp(val, "c") || !strcmp(val, "2"))
				*phase = 2;
			else if (!strcasecmp(val, "d") || !strcmp(val, "3"))
				*phase = 3;
		}
		else if (!strcmp(key, "blocking")) {
			int bv = atoi(val);
			if (bv == 16) bv = 0; /* normalize: 16 → B=0000 */
			if (bv >= 0 && bv <= 15)
				*blocking_length = bv;
		}
		else if (!strcmp(key, "maildrop"))
			*mail_drop = atoi(val) ? 1 : 0;
		else if (!strcmp(key, "sectype")) {
			if (!strcasecmp(val, "alpha"))
				*secure_encoding = 0;
			else if (!strcasecmp(val, "binary"))
				*secure_encoding = 1;
			else {
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: invalid sectype '%s', must be alpha or binary.\n", val);
				*secure_encoding = -1; /* signal error */
			}
		}
		else if (!strcmp(key, "sectag")) {
			int st = atoi(val);
			if (st >= 0 && st <= 3)
				*secure_subtype = st;
			else
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: sectag %d out of range (0-3), using default.\n", st);
		}
		else if (!strcmp(key, "msgnum")) {
			int mn = atoi(val);
			if (mn >= 0 && mn <= 63)
				*numbered_msgnum = mn;
			else
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: msgnum %d out of range (0-63), using auto-assign.\n", mn);
		}
		else if (!strcmp(key, "chan_setup"))
			*msg_chan_setup = atoi(val) ? 1 : 0;
	}

	/* Derive sectag from sectype when operator didn't set it explicitly:
	 * sectype=alpha → sectag=0 (t1t0=00), sectype=binary → sectag=2 (t1t0=10) */
	if (*secure_subtype == -1) {
		if (*secure_encoding == 1)
			*secure_subtype = 2; /* binary default: t1t0=10 */
		else
			*secure_subtype = 0; /* alpha default: t1t0=00 */
	}
}

/* Process a single FIFO line: capcode,type,options,message */
static void fifo_process_line(const char *text, int text_length)
{
	char capcode_string[text_length + 1];
	char type_string[text_length + 1];
	char msg_buf[text_length + 1];
	uint64_t capcode;
	enum flex_msg_type mtype;
	int message_length = 0;
	int j;
	int is_group = 0;
	int is_temp_group = 0;
	int comma_count = 0;
	int comma1 = -1, comma2 = -1, comma3 = -1;
	int msg_speed;
	int msg_mod_type;
	double msg_polarity;
	int msg_priority;
	int msg_charset;
	int msg_phase;
	int msg_blocking_length;
	int msg_mail_drop;
	int msg_secure_encoding;
	int msg_secure_subtype;
	int msg_numbered_msgnum;
	int msg_chan_setup;
	char msg_source[64];
	const char *opts_start;
	int opts_len;

	if (!tx) {
		LOGP(DFLEX, LOGL_ERROR, "Failed to send message, transmitter is not enabled!\n");
		return;
	}

	/* Count commas and record positions */
	for (j = 0; j < text_length; j++) {
		if (text[j] == ',') {
			comma_count++;
			if (comma_count == 1)
				comma1 = j;
			else if (comma_count == 2)
				comma2 = j;
			else if (comma_count == 3) {
				comma3 = j;
				break; /* stop at third comma */
			}
		}
	}

	if (comma_count < 3) {
		LOGP(DFLEX, LOGL_NOTICE, "Given message MUST be in the format: capcode,type,options,message\n");
		return;
	}

	/* Extract capcode field (before first comma) */
	memcpy(capcode_string, text, comma1);
	capcode_string[comma1] = '\0';

	/* === ERS command: "ers,0,," ===
	 * Triggers an Emergency Re-Synchronization burst.
	 * Format: ers,0,,
	 * Duration is calculated from collapse value (ERS cycles).
	 *
	 * Polarity is irrelevant for ERS: each cycle is BS+Ar+BS_inv+Ar_inv,
	 * so inverting all bits (= flipping polarity) just shifts the
	 * continuous stream by half a cycle.  Pagers of either polarity
	 * will detect the re-sync pattern. */
	if (!strcasecmp(capcode_string, "ers")) {
		flex_t *flex = (flex_t *)sender_head;

		if (!flex) {
			LOGP(DFLEX, LOGL_ERROR, "No transmitter instance for ERS command.\n");
			return;
		}

		LOGP(DFLEX, LOGL_INFO, "FIFO: ERS command.\n");
		flex_trigger_ers(flex);
		return;
	}

	/* === Status command: "status,0,," ===
	 * Dumps current temporary group assignments (persistent across frames).
	 * Assignments persist until group message delivery completes or
	 * 128-frame timeout (~2 minutes).
	 * 16 group slots per phase, each can have multiple members. */
	if (!strcasecmp(capcode_string, "status")) {
		flex_t *flex = (flex_t *)sender_head;
		static const char pnames[FLEX_MAX_PHASES] = { 'A', 'B', 'C', 'D' };
		int p, s, m, any = 0;

		if (!flex) {
			LOGP(DFLEX, LOGL_ERROR, "No instance for status command.\n");
			return;
		}

		/* System config summary */
		{
			flex_msg_t *qm;
			int q_total = 0;
			for (qm = flex->msg_list; qm; qm = qm->next)
				q_total++;
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: System status: collapse=%d num_tx=%d td_collapse=%s queue=%d last=C%u/F%u\n",
			     flex->collapse,
			     flex->num_transmissions,
			     flex->td_collapse >= 0 ?
			       (flex->td_collapse == 5 ? "5" :
			        flex->td_collapse == 6 ? "6" : "7") :
			       "system",
			     q_total,
			     flex->sched_last_cycle, flex->sched_last_frame);
		}

		LOGP(DFLEX, LOGL_NOTICE, "FIFO: Temp group assignments (16 groups/phase):\n");
		for (p = 0; p < FLEX_MAX_PHASES; p++) {
			for (s = 0; s < FLEX_TEMP_ADDR_SLOTS; s++) {
				if (!flex->rx.temp_addr_map[p][s].active)
					continue;
				int cnt = flex->rx.temp_addr_map[p][s].count;
				if (cnt > 0) {
					LOGP(DFLEX, LOGL_NOTICE,
					     "  phase=%c group=%d members=%d target_frame=%u setup=C%u/F%u:",
					     pnames[p], s, cnt,
					     flex->rx.temp_addr_map[p][s].target_frame,
					     flex->rx.temp_addr_map[p][s].setup_cycle,
					     flex->rx.temp_addr_map[p][s].setup_frame);
					for (m = 0; m < cnt; m++)
						LOGP(DFLEX, LOGL_NOTICE,
						     " %" PRIu64,
						     flex->rx.temp_addr_map[p][s].capcodes[m]);
					LOGP(DFLEX, LOGL_NOTICE, "\n");
					any = 1;
				}
			}
		}
		if (!any)
			LOGP(DFLEX, LOGL_NOTICE, "  (no active assignments)\n");
		return;
	}

	/* === Timezone command: "timezone,0,," or "timezone,<code>,," ===
	 * Dumps the 32-entry timezone conversion table, or shows
	 * the offset for a specific 5-bit zone code (0-31). */
	if (!strcasecmp(capcode_string, "timezone")) {
		int tlen = comma2 - comma1 - 1;
		char tbuf[16];
		memcpy(tbuf, text + comma1 + 1, tlen < (int)sizeof(tbuf) - 1 ? tlen : (int)sizeof(tbuf) - 1);
		tbuf[tlen < (int)sizeof(tbuf) - 1 ? tlen : (int)sizeof(tbuf) - 1] = '\0';

		if (tbuf[0] == '\0' || !strcmp(tbuf, "0") || !strcasecmp(tbuf, "list")) {
			/* Dump full table */
			char fmtbuf[20];
			uint32_t i;
			LOGP(DFLEX, LOGL_NOTICE, "FIFO: Timezone conversion table (32 entries, 5-bit Z4..Z0):\n");
			for (i = 0; i < FLEX_TZ_ENTRIES; i++) {
				if (i == FLEX_TZ_RESERVED) {
					LOGP(DFLEX, LOGL_NOTICE, "  zone=%2u (reserved)\n", i);
				} else {
					LOGP(DFLEX, LOGL_NOTICE, "  zone=%2u %s\n",
					     i, flex_tz_format(flex_tz_table[i], fmtbuf, sizeof(fmtbuf)));
				}
			}
		} else {
			/* Show specific zone code */
			int code = atoi(tbuf);
			if (code < 0 || code >= (int)FLEX_TZ_ENTRIES) {
				LOGP(DFLEX, LOGL_ERROR, "FIFO: timezone code %d out of range (0-31).\n", code);
			} else if (code == (int)FLEX_TZ_RESERVED) {
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: timezone zone=%d (reserved).\n", code);
			} else {
				char fmtbuf[20];
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: timezone zone=%d %s (%+d min)\n",
				     code,
				     flex_tz_format(flex_tz_table[code], fmtbuf, sizeof(fmtbuf)),
				     flex_tz_table[code]);
			}
		}
		return;
	}

	/* === System Message command: "sysmsg,<lsb>,,<payload>" ===
	 * Sends a system message via Operator Messaging Address
	 * (capcode 0x1F7800 + LSB, range 0x1F7800-0x1F780F).
	 *
	 * The <lsb> field (0-15) selects the operator address sub-type:
	 *   0 = all pagers    1 = home area    2 = roaming    3 = SSID
	 *   4 = time related  5-13 = reserved  14 = SSIDChange  15 = SysEvent
	 *
	 * The <payload> is sent as an alpha message body on the operator
	 * messaging address.  The pager infrastructure interprets the
	 * content based on the sub-type.
	 *
	 * Examples:
	 *   sysmsg,0,,System maintenance at 03:00 UTC
	 *   sysmsg,14,,                (SSIDChange, empty payload = TMF update)
	 *   sysmsg,4,,                 (time-related system message)
	 *
	 * Note: Tone-Only Addresses cannot be transmitted in frames
	 * used for transmitting System Messages. */
	if (!strcasecmp(capcode_string, "sysmsg")) {
		flex_t *flex = (flex_t *)sender_head;
		int lsb;
		int tlen;
		char tbuf[16];

		if (!flex) {
			LOGP(DFLEX, LOGL_ERROR, "No transmitter instance for sysmsg command.\n");
			return;
		}

		/* Extract LSB from type field */
		tlen = comma2 - comma1 - 1;
		if (tlen <= 0 || tlen >= (int)sizeof(tbuf)) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: sysmsg requires LSB (0-15). Format: sysmsg,<lsb>,,<payload>\n"
			     "  LSB: 0=all 1=home 2=roaming 3=ssid 4=time 14=SSIDChange 15=SysEvent\n");
			return;
		}
		memcpy(tbuf, text + comma1 + 1, tlen);
		tbuf[tlen] = '\0';
		lsb = atoi(tbuf);

		if (lsb < 0 || lsb > 15) {
			LOGP(DFLEX, LOGL_NOTICE, "FIFO: sysmsg LSB %d out of range (0-15).\n", lsb);
			return;
		}

		/* Compute the operator messaging capcode.
		 * Base address 0x1F7810 + LSB (range 0-15). */
		{
			uint64_t oper_capcode = (uint64_t)(FLEX_ADDR_OPER_MSG_MIN + (uint32_t)lsb);
			flex_msg_t *msg;

			/* Extract message payload (after third comma) */
			if (comma3 + 1 < text_length) {
				message_length = flex_scan_message(text + comma3 + 1,
								   text_length - comma3 - 1,
								   msg_buf, sizeof(msg_buf));
			}

			msg = flex_msg_create(flex, oper_capcode,
					      message_length > 0 ? FLEX_MSG_TYPE_ALPHA : FLEX_MSG_TYPE_TONE,
					      msg_buf, message_length);
			if (msg) {
				msg->speed = 1600;
				msg->modulation_type = FLEX_MOD_2FSK;
				msg->polarity = -1.0;
				msg->priority = 1; /* system messages are priority */
				LOGP(DFLEX, LOGL_INFO,
				     "FIFO: sysmsg enqueued LSB=%d (%s) capcode=%" PRIu64 " len=%d\n",
				     lsb, flex_oper_msg_subtype_name((uint32_t)(FLEX_ADDR_OPER_MSG_MIN + lsb)),
				     oper_capcode, message_length);
			}
		}
		return;
	}

	/* Handle group:capcode prefix */
	if (strncmp(capcode_string, "group:", 6) == 0) {
		is_group = 1;
		memmove(capcode_string, capcode_string + 6, strlen(capcode_string + 6) + 1);
	}

	/* Extract type field (between first and second comma) */
	{
		int tlen = comma2 - comma1 - 1;
		memcpy(type_string, text + comma1 + 1, tlen);
		type_string[tlen] = '\0';
	}

	/* Extract options field (between second and third comma) */
	opts_start = text + comma2 + 1;
	opts_len = comma3 - comma2 - 1;

	/* Extract message payload (everything after third comma) */
	if (comma3 + 1 < text_length) {
		message_length = flex_scan_message(text + comma3 + 1, text_length - comma3 - 1, msg_buf, sizeof(msg_buf));
	}

	/* Parse options field */
	parse_fifo_options(opts_start, opts_len,
			   &msg_speed, &msg_mod_type,
			   &msg_polarity, &msg_priority,
			   &msg_charset, &is_group, &is_temp_group,
			   msg_source, &msg_phase, &msg_blocking_length,
			   &msg_mail_drop,
			   &msg_secure_encoding, &msg_secure_subtype,
			   &msg_numbered_msgnum,
			   &msg_chan_setup);

	/* Discard message if sectype was invalid */
	if (msg_secure_encoding == -1)
		return;

	/* Validate capcode */
	capcode = strtoull(capcode_string, NULL, 10);
	if (!flex_capcode_valid(capcode)) {
		LOGP(DFLEX, LOGL_NOTICE, "Invalid capcode '%" PRIu64 "'.\n", capcode);
		return;
	}

	/* Validate message type.
	 * Accept names or numeric codes:
	 *   0=auto, 1=tone, 2=numeric, 3=alpha, 4=hex, 5=instruction, 6=short
	 * Also accept vector type numbers directly for advanced use:
	 *   vtype1=instruction, vtype2=tone, vtype3=numeric, vtype5=alpha, vtype6=hex */
	if (!strcasecmp(type_string, "auto") || !strcmp(type_string, "0"))
		mtype = FLEX_MSG_TYPE_AUTO;
	else if (!strcasecmp(type_string, "tone") || !strcmp(type_string, "1"))
		mtype = FLEX_MSG_TYPE_TONE;
	else if (!strcasecmp(type_string, "numeric") || !strcasecmp(type_string, "num") || !strcmp(type_string, "2"))
		mtype = FLEX_MSG_TYPE_NUMERIC;
	else if (!strcasecmp(type_string, "alpha") || !strcasecmp(type_string, "alphanumeric") || !strcmp(type_string, "3"))
		mtype = FLEX_MSG_TYPE_ALPHA;
	else if (!strcasecmp(type_string, "hex") || !strcasecmp(type_string, "binary") || !strcmp(type_string, "4"))
		mtype = FLEX_MSG_TYPE_HEX;
	else if (!strcasecmp(type_string, "instruction") || !strcasecmp(type_string, "instr") || !strcmp(type_string, "5"))
		mtype = FLEX_MSG_TYPE_INSTRUCTION;
	else if (!strcasecmp(type_string, "short") || !strcmp(type_string, "6"))
		mtype = FLEX_MSG_TYPE_SHORT;
	else if (!strcasecmp(type_string, "secure") || !strcasecmp(type_string, "sec") || !strcmp(type_string, "7"))
		mtype = FLEX_MSG_TYPE_SECURE;
	else if (!strcasecmp(type_string, "special") || !strcasecmp(type_string, "snum") || !strcmp(type_string, "8"))
		mtype = FLEX_MSG_TYPE_SPECIAL_NUM;
	else if (!strcasecmp(type_string, "nnumeric") || !strcasecmp(type_string, "nnum") || !strcmp(type_string, "9"))
		mtype = FLEX_MSG_TYPE_NUMBERED_NUM;
	else if (!strcasecmp(type_string, "nspecial") || !strcasecmp(type_string, "nsnum") || !strcmp(type_string, "10"))
		mtype = FLEX_MSG_TYPE_NUMBERED_SPECIAL;
	else {
		LOGP(DFLEX, LOGL_NOTICE, "FIFO: invalid type '%s'. Use auto/tone/numeric/alpha/hex/instruction/short/secure/special/nnumeric/nspecial.\n", type_string);
		return;
	}

	/* Auto-detect message type if AUTO */
	if (mtype == FLEX_MSG_TYPE_AUTO) {
		if (message_length == 0) {
			mtype = FLEX_MSG_TYPE_TONE;
		} else {
			mtype = flex_detect_msg_type(msg_buf, message_length);
		}
	}

	/* Fixed-mode rejection: check speed, modulation type, and polarity locks */
	{
		flex_t *flex = (flex_t *)sender_head;
		if (flex) {
			if (flex->fixed_speed != -1 &&
			    (msg_speed != flex->fixed_speed || msg_mod_type != flex->fixed_mod_type)) {
				LOGP(DFLEX, LOGL_NOTICE, "fixed-mode: speed locked to %d/%s, discarding message with speed=%d/%s\n",
				     flex->fixed_speed,
				     (flex->fixed_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
				     msg_speed,
				     (msg_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk");
				return;
			}
			if (flex->fixed_polarity != 0.0 && msg_polarity != flex->fixed_polarity) {
				LOGP(DFLEX, LOGL_NOTICE, "fixed-mode: polarity locked to %s, discarding message with polarity=%s\n",
				     (flex->fixed_polarity < 0) ? "negative" : "positive",
				     (msg_polarity < 0) ? "negative" : "positive");
				return;
			}
		}
	}

	/* Enqueue message on first transmitter instance */
	{
		flex_t *flex = (flex_t *)sender_head;
		if (flex) {
			flex_msg_t *msg;
			msg = flex_msg_create(flex, capcode, mtype, msg_buf, message_length);
			if (msg) {
				msg->speed = msg_speed;
				msg->modulation_type = msg_mod_type;
				msg->polarity = msg_polarity;
				msg->priority = msg_priority;
				msg->charset = msg_charset;
				msg->is_group = is_group;
				msg->is_temp_group = is_temp_group;
				msg->phase = msg_phase;
				msg->blocking_length = msg_blocking_length;
				msg->mail_drop = msg_mail_drop;
				msg->secure_encoding = msg_secure_encoding;
				msg->secure_subtype = msg_secure_subtype;
				msg->numbered_msgnum = msg_numbered_msgnum;
				if (msg_chan_setup >= 0)
					flex->chan_setup_enabled = msg_chan_setup;
				if (msg_source[0] != '\0') {
					strncpy(msg->source_id, msg_source, sizeof(msg->source_id) - 1);
					msg->source_id[sizeof(msg->source_id) - 1] = '\0';
				}
				LOGP(DFLEX, LOGL_INFO,
				     "FIFO: enqueued capcode=%" PRIu64 " type=%s speed=%d/%s polarity=%s priority=%d charset=%s group=%d tempgroup=%d phase=%s len=%d\n",
				     capcode,
				     flex_msg_type_name(mtype),
				     msg_speed,
				     (msg_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
				     (msg_polarity < 0) ? "neg" : "pos",
				     msg_priority,
				     msg_charset ? "kanji" : "ascii",
				     is_group,
				     is_temp_group,
				     (msg_phase < 0) ? "auto" :
				     (msg_phase == 0) ? "A" :
				     (msg_phase == 1) ? "B" :
				     (msg_phase == 2) ? "C" : "D",
				     message_length);
				if (mtype == FLEX_MSG_TYPE_SECURE)
					LOGP(DFLEX, LOGL_INFO,
					     "FIFO:   secure options: sectype=%s sectag=%d\n",
					     msg_secure_encoding ? "binary" : "alpha",
					     msg_secure_subtype);
				if (mtype == FLEX_MSG_TYPE_NUMBERED_NUM ||
				    mtype == FLEX_MSG_TYPE_NUMBERED_SPECIAL)
					LOGP(DFLEX, LOGL_INFO,
					     "FIFO:   numbered options: msgnum=%d\n",
					     msg_numbered_msgnum);
				if (mtype == FLEX_MSG_TYPE_HEX)
					LOGP(DFLEX, LOGL_INFO,
					     "FIFO:   hex options: blocking=%d\n",
					     msg_blocking_length);
			}
		}
	}
}

static void myhandler(void)
{
	static char buffer[4096 + 256];
	static int pos = 0;
	int rc, i, line_start;
	int space;

	/* Read whatever is available from the FIFO */
	space = (int)sizeof(buffer) - pos;
	if (space <= 0) {
		fprintf(stderr, "Message buffer overflow, discarding.\n");
		pos = 0;
		space = (int)sizeof(buffer);
	}

	rc = read(msg_send_fd, buffer + pos, space);
	if (rc > 0)
		pos += rc;

	/* Process all complete lines in the buffer */
	line_start = 0;
	while (line_start < pos) {
		/* Find next line terminator */
		for (i = line_start; i < pos; i++) {
			if (buffer[i] == '\r' || buffer[i] == '\n')
				break;
		}

		if (i >= pos)
			break;  /* no complete line yet — keep remainder */

		/* Process this line if non-empty */
		if (i > line_start)
			fifo_process_line(buffer + line_start, i - line_start);

		/* Skip past the line terminator(s) */
		i++;
		while (i < pos && (buffer[i] == '\r' || buffer[i] == '\n'))
			i++;
		line_start = i;
	}

	/* Shift any remaining partial line to the front of the buffer */
	if (line_start > 0) {
		int remaining = pos - line_start;
		if (remaining > 0)
			memmove(buffer, buffer + line_start, remaining);
		pos = remaining;
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

	/* STD-43A option validation */
	if (collapse > 0 && !network_mode) {
		fprintf(stderr, "Warning: --collapse is only meaningful in network mode, setting collapse=0.\n");
		collapse = 0;
	}
	if (num_transmissions > 1 && !network_mode) {
		fprintf(stderr, "Warning: --num-transmissions > 1 requires --network, setting to 1.\n");
		num_transmissions = 1;
	}
	if (num_transmissions > 1 && collapse <= 0) {
		fprintf(stderr, "Warning: --num-transmissions > 1 requires --collapse > 0, setting to 1.\n");
		num_transmissions = 1;
	}
	if (biw_time_enabled == -1)
		biw_time_enabled = network_mode;

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
		/* Set STD-43A compliance config on the just-created instance.
		 * sender_create() appends to the list, so walk to the tail. */
		{
			sender_t *s;
			flex_t *f;
			for (s = sender_head; s->next; s = s->next)
				;
			f = (flex_t *)s;
			f->network_mode = network_mode;
			f->collapse = collapse;
			f->fixed_speed = fixed_speed;
			f->fixed_mod_type = fixed_mod_type;
			f->fixed_polarity = fixed_polarity;
			f->lpf_enabled = lpf_enabled;
			f->biw_time_enabled = biw_time_enabled;
			f->ers_cycles_override = ers_cycles_override;
			f->no_ers = no_ers;
			f->default_charset = default_charset;
			f->default_polarity = polarity;
			f->default_phase = default_phase;
			f->default_blocking_length = default_blocking_length;
			if (wav_test)
				f->wav_test_mode = 1;
			f->ssid = ssid;
			f->nid = nid;
			f->country_code = country_code;
			f->tmf = tmf_flags;
			f->timezone_code = timezone_code;
			f->roaming_active = (ssid != 0 || nid != 0) ? 1 : 0;
			f->num_transmissions = num_transmissions;
			f->td_collapse = td_collapse;
			f->chan_setup_enabled = chan_setup_enabled;

			/* Parse POCSAG mixing frame slots if specified */
			if (pocsag_mix) {
				rc = flex_scheduler_parse_pocsag_slots(f, pocsag_mix);
				if (rc < 0) {
					fprintf(stderr, "Failed to parse --pocsag-mix '%s'.\n", pocsag_mix);
					goto fail;
				}
			}
		}
		printf("Base station ready, please tune transmitter to %.4f MHz\n", frequency / 1e6);
	}

	/* Start scanning/loopback after all config is applied */
	{
		sender_t *s;
		for (s = sender_head; s; s = s->next)
			flex_scan_or_loopback((flex_t *)s);
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
