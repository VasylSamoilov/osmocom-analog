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
#include "../libmobile/get_time.h"
#include "flex.h"
#include "frame.h"
#include "dsp.h"
#include "scheduler.h"

#define MSG_SEND_DEFAULT "/tmp/flex_msg_send"
static const char *msg_send_path = MSG_SEND_DEFAULT;
static int msg_send_fd = -1;

static int tx = 0;
static double deviation = 4800;
static double polarity = FLEX_DEFAULT_POLARITY;
static enum flex_msg_type msg_type = FLEX_MSG_TYPE_AUTO;
static int msg_type_given = 0;
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
static int default_retransmit = 0;		/* --retransmit: 0-15, default 0 (no retransmission) */
static int default_retransmit_interval = 128;	/* --retransmit-interval: 1-1920 frames, default 128 (~4 min) */
static int default_send_delay = 0;		/* --send-delay: 0-1920 frames, default 0 (immediate) */
static int hack_nonstandard_decoders = 0;	/* --hack-for-non-standard-decoders: block-boundary fixup */
static int roaming_enabled = 0;			/* --roaming: FIW n=1, default 0 */

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
#define OPT_RETRANSMIT		3025
#define OPT_RETRANSMIT_INTERVAL	3026
#define OPT_SEND_DELAY		3027
#define OPT_HACK_NONSTANDARD	3028
#define OPT_FIFO		3029
#define OPT_ROAMING		3030
#define OPT_SSID1		3031
#define OPT_SSID2		3032

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "-k <frequency>");
	printf(" -T --tx\n");
	printf("        Transmit FLEX signal on given channel. (default)\n");
	printf(" -D --deviation <KHz>\n");
	printf("        Choose deviation of FSK signal (default %.0f Hz).\n", deviation);
	printf(" -P --polarity normal | inverted | positive | negative | 1 | -1\n");
	printf("        Choose polarity of FSK signal. (default %s).\n", (polarity < 0) ? "inverted" : "normal");
	printf("        normal=+4800Hz for '1' (spec-compliant), inverted=-4800Hz for '1'.\n");
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
	printf("        Scan through given capcode range. Default type is 'short' (short\n");
	printf("        numeric message in vector word only — no body overhead). Short\n");
	printf("        addresses carry 3 BCD digits, long addresses carry 6 digits.\n");
	printf("        Use -y to override: numeric, alpha, tone, short.\n");
	printf("    --network\n");
	printf("        Enable network mode (continuous operation).\n");
	printf("    --collapse <0-7>\n");
	printf("        Set collapse cycle value (default %d, network mode only).\n", collapse);
	printf("    --speed <1600|3200|3200-4fsk|6400>\n");
	printf("        Lock transmitter to fixed baud rate (fixed-mode).\n");
	printf("    --fixed-polarity <normal|inverted|pos|neg>\n");
	printf("        Lock transmitter to fixed FSK polarity (fixed-mode).\n");
	printf("    --lpf\n");
	printf("        Enable baseband low-pass filter (default).\n");
	printf("    --no-lpf\n");
	printf("        Disable baseband low-pass filter.\n");
	printf("    --biw-time <auto|offset>\n");
	printf("        Enable BIW date/time/timezone broadcast.\n");
	printf("        auto    — detect timezone from system clock.\n");
	printf("        offset  — UTC offset, e.g. +9, -5, +5:30, +0.\n");
	printf("        Transmits BIW Date (001), Time (010), and Timezone (101).\n");
	printf("    --no-biw-time\n");
	printf("        Disable BIW date/time/timezone broadcast.\n");
	printf("    --no-ers\n");
	printf("        Skip ERS burst in single-shot mode (workaround for decoders\n");
	printf("        that choke on ERS sync patterns, e.g. PDW).\n");
	printf("    --ers-cycles <N>\n");
	printf("        Override ERS cycle count (default auto).\n");
	printf("    --charset <ascii|kanji>\n");
	printf("        Set default character set (default ascii).\n");
	printf("    --ssid1 <LID>,<CZ>\n");
	printf("        Set SSID1 (BIW000): Local channel ID and Coverage Zone.\n");
	printf("        LID: 0-511 (9 bits). Unique per operator across all frequencies.\n");
	printf("        CZ:  0-31 (5 bits). 32 systems per LID.\n");
	printf("        Example: --ssid1 10,1\n");
	printf("    --ssid2 <CC>,<TMF>\n");
	printf("        Set SSID2 (BIW111): Country Code and Traffic Management Flag.\n");
	printf("        CC:  0-1023 (10 bits, ITU-T E.212). Example: 440=Japan.\n");
	printf("        TMF: 0-15 (4 bits). Identifies up to 4 channels per SSID1+CC.\n");
	printf("        Example: --ssid2 440,1\n");
	printf("    --roaming\n");
	printf("        Set FIW roaming flag n=1. Default n=0.\n");
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
	printf("    --retransmit <0-15>\n");
	printf("        Default retransmission count after initial TX (default 0 = none).\n");
	printf("        Each retransmission re-sends the message with R=0 and same N.\n");
	printf("        Only applies to message types with R/N flags (alpha, hex, secure,\n");
	printf("        numbered numeric, numbered special). Requires --network.\n");
	printf("    --retransmit-interval <1-1920>\n");
	printf("        Default frames between retransmissions (default 128 = ~4 minutes).\n");
	printf("        1 frame = 1.875 seconds. Range: 1.875s to 1 hour.\n");
	printf("    --send-delay <0-1920>\n");
	printf("        Default frames to defer initial TX (default 0 = immediate).\n");
	printf("        Delays first transmission by N frames after enqueue.\n");
	printf("    --hack-for-non-standard-decoders\n");
	printf("        Enable block-boundary fixup for non-standard decoders.\n");
	printf("        PDW and multimon-ng use idle-word detection (not in the\n");
	printf("        standard) that causes early frame termination when a\n");
	printf("        legitimate data word is all-zeros or all-ones at a block\n");
	printf("        boundary (word 7, 15, 23, ...).  This flag flips bit 0\n");
	printf("        of such words; BCH(31,21) corrects the 1-bit error on RX.\n");
	printf("        Default OFF (standard-compliant behavior).\n");
	printf("    --fifo <path>\n");
	printf("        Path for the message send FIFO (default %s).\n", MSG_SEND_DEFAULT);
	printf("\n");
	printf("    FIFO protocol (write to %s):\n", msg_send_path);
	printf("        Format: capcode,type,options,message\n");
	printf("\n");
	printf("        Capcode field (ARIB STD-43A Appendix A Extended CAPCODE):\n");
	printf("          Plain numeric:  1234567 (short) or 007005031 (long)\n");
	printf("          Extended form:  [R][fff][b]<alpha><digits>\n");
	printf("            R     = roaming prefix: P(none) Q(TMF) R(FrameOfs) S(both)\n");
	printf("            fff   = explicit frame 0-127 (optional)\n");
	printf("            b     = collapse cycle 0-7 (optional, wake every 2^b frames)\n");
	printf("            alpha = pager type prefix (required for extended form):\n");
	printf("              Standard rule (frame/phase derived from address digits):\n");
	printf("                A-D = Single Phase (subtract 0-3 for multi-addr pagers)\n");
	printf("                E-H = Any Phase    (subtract 0-3)\n");
	printf("                I-L = All Phase    (subtract 0-3)\n");
	printf("              Non-standard (explicit phase, not derived from address):\n");
	printf("                U=Phase A  V=Phase B  W=Phase C  X=Phase D\n");
	printf("                Y=Any Phase  Z=All Phase\n");
	printf("            digits = 7-digit short or 9-digit long address\n");
	printf("          Examples:\n");
	printf("            3E007005031  → collapse=3, Any Phase, addr=007005031 (Long)\n");
	printf("            A1234567     → Single Phase, subtract 0, addr=1234567 (Short)\n");
	printf("            5U0000100    → collapse=5, Single Phase A, addr=0000100\n");
	printf("            R3E007005031 → roaming=R(FrameOfs), collapse=3, Any Phase\n");
	printf("          Phase/frame from standard rule (A-L):\n");
	printf("            phase = (address / 4) mod 4    → 0=A 1=B 2=C 3=D\n");
	printf("            frame = (address / 16) mod 128 → 0-127\n");
	printf("          Non-standard U/V/W/X set phase= automatically (if not overridden).\n");
	printf("\n");
	printf("        Types:  auto|tone|numeric|alpha|hex|instruction|short|secure|special|nnumeric|nspecial (or 0-10)\n");
	printf("        Options: space-separated key=value pairs:\n");
	printf("          speed=1600|3200|3200-4fsk|6400  polarity=normal|inverted\n");
	printf("          priority=0|1  charset=ascii|kanji\n");
	printf("          tempgroup=0|1  source=<id>\n");
	printf("          phase=A|B|C|D|auto\n");
	printf("          blocking=0-16  (hex bits/char: 0 or 16=16bit, 1=raw, default 1)\n");
	printf("          maildrop=0|1  (alpha/hex: separate handling from ordinary msgs)\n");
	printf("          stype=numeric|source|numbered  (short msg sub-type, §3.9.2)\n");
	printf("            numeric: message = up to 3 BCD digits (short addr)\n");
	printf("                     or 8 BCD digits (long addr). Default.\n");
	printf("            source:  message = source code (0-7)\n");
	printf("            numbered: message = message number N (0-63)\n");
	printf("                      ssource=0-7 and sr=0|1 in options\n");
	printf("          ssource=0-7  (short msg source code S, for stype=source/numbered)\n");
	printf("          sr=0|1  (short msg retrieval flag R, for stype=numbered)\n");
	printf("          itype=tempaddr|sysevent  (instruction sub-type, §3.9.6)\n");
	printf("            tempaddr: islot=0-15 iframe=0-127 (message field ignored)\n");
	printf("            sysevent: message = space-separated flag names:\n");
	printf("              SSID_TMF NID_TMF CHAN_SETUP NID_FREQ SSID_FREQ\n");
	printf("            (omit itype for raw decimal instruction data in message)\n");
	printf("          islot=0-15  (temp address slot, for itype=tempaddr)\n");
	printf("          iframe=0-127  (target frame, for itype=tempaddr)\n");
	printf("          sectype=alpha|binary|regack  (secure: wire encoding, default alpha)\n");
	printf("            regack: Registration Acknowledgment (opcode '=' in 2nd word)\n");
	printf("          sectag=0|1|2|3  (secure: pager type tag, 0=alpha 1=vendor 2=binary 3=rsvd, default=sectype)\n");
	printf("          msgnum=0-63  (nnumeric/nspecial: sequence number for dedup, default auto)\n");
	printf("          chan_setup=0|1  (enable/disable BIW channel setup emission)\n");
	printf("          retransmit=0-15  (retransmissions after initial TX, overrides --retransmit)\n");
	printf("          retransmit_interval=1-1920  (frames between retransmissions, overrides --retransmit-interval)\n");
	printf("          send_delay=0-1920  (frames to defer initial TX, overrides --send-delay)\n");
	printf("        Special: ers,0,,  — trigger ERS re-sync burst\n");
	printf("        Special: status,0,,  — dump system config + temp group assignments\n");
	printf("        Special: timezone,0,,  — dump timezone table (32 entries)\n");
	printf("        Special: timezone,<code>,,  — show offset for zone code 0-31\n");
	printf("        Special: sysmsg,<type>,lsb=<target> method=<m>,<message>  — send system message via\n");
	printf("          Operator Messaging Address (§3.8.2.4).\n");
	printf("          type: alpha, numeric, special, hex, tone (not secure per §3.9.2)\n");
	printf("          lsb= target audience (name or 0-15, default all):\n");
	printf("            all(0) home(1) roaming(2) ssid(3) time(4)\n");
	printf("            ssidchange(14) sysevent(15)\n");
	printf("          method= transmission method (§3.9.2, Fig. 3.7.2-2, default b):\n");
	printf("            a = BIW101 only (implicit address, vector at end of VF, no operator addr)\n");
	printf("            b = BIW101 + Operator Messaging Address (both, default)\n");
	printf("            c = Operator Messaging Address only (no BIW101)\n");
	printf("        Examples:\n");
	printf("          1234567,alpha,,Hello World\n");
	printf("          1234567,alpha,speed=3200 priority=1,Hello World\n");
	printf("          1234567,numeric,,31415926\n");
	printf("          1234567,hex,blocking=8,DEADBEEF\n");
	printf("          1234567,tone,,\n");
	printf("          1234567,instruction,,42\n");
	printf("          1234567,instruction,itype=tempaddr islot=5 iframe=42,\n");
	printf("          1234567,instruction,itype=sysevent,SSID_TMF NID_FREQ\n");
	printf("          1234567,short,,123              (t=00 numeric: 3 BCD digits)\n");
	printf("          1234567,short,stype=source,3    (t=01 source code 3)\n");
	printf("          1234567,short,stype=numbered ssource=2 sr=1,7  (t=10 S=2 N=7 R=1)\n");
	printf("          1234567,secure,,Secure alpha message\n");
	printf("          1234567,secure,sectype=binary sectag=2,DEADBEEF\n");
	printf("          1234567,secure,sectype=regack,  (Registration Acknowledgment)\n");
	printf("          1234567,special,,31415926\n");
	printf("          1234567,nnumeric,msgnum=7,31415926\n");
	printf("          1234567,nspecial,,31415926\n");
	printf("          1234567,alpha,,Hello World\n");
	printf("          1234567,alpha,tempgroup=1,Temp group msg\n");
	printf("          tempgroup:1234567 2345678 3456789,alpha,,Hello everyone\n");
	printf("          3E007005031,alpha,,Extended CAPCODE (Long, Any Phase, collapse=3)\n");
	printf("          A1234567,numeric,,Standard rule Short CAPCODE\n");
	printf("          5U0000100,tone,,Non-standard, collapse=5, Phase A\n");
	printf("          sysmsg,alpha,,System maintenance at 03:00 UTC\n");
	printf("          sysmsg,alpha,lsb=home,Home area update\n");
	printf("          sysmsg,numeric,,31415926\n");
	printf("          sysmsg,tone,lsb=ssidchange,\n");
	printf("          sysmsg,tone,,\n");
	printf("\n");
	printf("    Valid capcode ranges:\n");
	printf("        Short:  %" PRIu64 "–%" PRIu64 " (1 address word, 21-bit d0-d20)\n",
	       (uint64_t)FLEX_SHORT_ADDR_MIN, (uint64_t)FLEX_SHORT_ADDR_MAX);
	printf("        Long:   %" PRIu64 "–%" PRIu64 " (2 address words, d0-d20 + e0-e20)\n",
	       (uint64_t)FLEX_LONG_ADDR_MIN, (uint64_t)FLEX_LONG_ADDR_MAX);
	printf("          Set 1-2: %" PRIu64 "–%" PRIu64 "     (LA1+LA2)\n",
	       (uint64_t)FLEX_LONG_SET12_MIN, (uint64_t)FLEX_LONG_SET12_MAX);
	printf("          Set 1-3/1-4: %" PRIu64 "–%" PRIu64 " (LA1+LA3 or LA1+LA4)\n",
	       (uint64_t)FLEX_LONG_SET34_MIN, (uint64_t)FLEX_LONG_SET34_MAX);
	printf("          Set 2-3: %" PRIu64 "–%" PRIu64 " (LA2+LA3)\n",
	       (uint64_t)FLEX_LONG_SET23_MIN, (uint64_t)FLEX_LONG_SET23_MAX);
	printf("        Special addresses (gap %" PRIu64 "–%" PRIu64 "):\n",
	       (uint64_t)FLEX_SHORT_ADDR_MAX + 1, (uint64_t)FLEX_LONG_ADDR_MIN - 1);
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
	option_add(OPT_BIW_TIME, "biw-time", 1);
	option_add(OPT_NO_BIW_TIME, "no-biw-time", 0);
	option_add(OPT_ERS_CYCLES, "ers-cycles", 1);
	option_add(OPT_CHARSET, "charset", 1);
	option_add(OPT_SSID1, "ssid1", 1);
	option_add(OPT_SSID2, "ssid2", 1);
	option_add(OPT_ROAMING, "roaming", 0);
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
	option_add(OPT_RETRANSMIT, "retransmit", 1);
	option_add(OPT_RETRANSMIT_INTERVAL, "retransmit-interval", 1);
	option_add(OPT_SEND_DELAY, "send-delay", 1);
	option_add(OPT_HACK_NONSTANDARD, "hack-for-non-standard-decoders", 0);
	option_add(OPT_FIFO, "fifo", 1);
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
		if (!strcasecmp(argv[argi], "normal") || !strcasecmp(argv[argi], "positive") ||
		    !strcasecmp(argv[argi], "pos") || !strcmp(argv[argi], "1"))
			polarity = 1.0;
		else if (!strcasecmp(argv[argi], "inverted") || !strcasecmp(argv[argi], "negative") ||
			 !strcasecmp(argv[argi], "neg") || !strcmp(argv[argi], "-1"))
			polarity = -1.0;
		else {
			fprintf(stderr, "Invalid polarity '%s'. Use normal|inverted (or positive|negative|1|-1).\n", argv[argi]);
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
		msg_type_given = 1;
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
		if (!strcasecmp(argv[argi], "normal") || !strcasecmp(argv[argi], "positive") ||
		    !strcasecmp(argv[argi], "pos"))
			fixed_polarity = 1.0;
		else if (!strcasecmp(argv[argi], "inverted") || !strcasecmp(argv[argi], "negative") ||
			 !strcasecmp(argv[argi], "neg"))
			fixed_polarity = -1.0;
		else {
			fprintf(stderr, "Invalid polarity '%s'. Use normal|inverted (or pos|neg).\n", argv[argi]);
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
		if (!strcasecmp(argv[argi], "auto")) {
			/* Auto-detect timezone from system */
			timezone_code = flex_tz_auto_detect();
			if (timezone_code < 0) {
				fprintf(stderr, "Warning: --biw-time auto: could not map system timezone to FLEX zone code.\n");
				fprintf(stderr, "         Use --biw-time <offset> (e.g. +9, -5, +5:30) or --timezone <0-31>.\n");
				timezone_code = -1;
			} else {
				char tzbuf[20];
				int tz_min = flex_tz_to_minutes((uint32_t)timezone_code);
				fprintf(stderr, "BIW time: auto-detected timezone zone=%d (%s)\n",
					timezone_code,
					flex_tz_format(tz_min, tzbuf, sizeof(tzbuf)));
			}
		} else {
			/* Parse UTC offset string like +9, -5, +5:30 */
			int offset_min = flex_tz_parse_offset(argv[argi]);
			if (offset_min == INT_MIN) {
				fprintf(stderr, "Invalid --biw-time offset '%s'. Use 'auto' or UTC offset like +9, -5, +5:30.\n", argv[argi]);
				return -EINVAL;
			}
			uint32_t code = flex_tz_from_minutes(offset_min);
			if (code == FLEX_TZ_RESERVED) {
				char tzbuf[20];
				fprintf(stderr, "UTC offset '%s' (%s) does not match any FLEX timezone zone code.\n",
					argv[argi],
					flex_tz_format(offset_min, tzbuf, sizeof(tzbuf)));
				fprintf(stderr, "Valid offsets: ");
				{
					uint32_t i;
					for (i = 0; i < FLEX_TZ_ENTRIES; i++) {
						if (i == FLEX_TZ_RESERVED) continue;
						fprintf(stderr, "%s%s",
							i > 0 ? ", " : "",
							flex_tz_format(flex_tz_table[i], tzbuf, sizeof(tzbuf)));
					}
				}
				fprintf(stderr, "\n");
				return -EINVAL;
			}
			timezone_code = (int)code;
			{
				char tzbuf[20];
				int tz_min = flex_tz_to_minutes((uint32_t)timezone_code);
				fprintf(stderr, "BIW time: timezone zone=%d (%s)\n",
					timezone_code,
					flex_tz_format(tz_min, tzbuf, sizeof(tzbuf)));
			}
		}
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
	case OPT_SSID1: {
		/* --ssid1 LID,CZ */
		char *comma = strchr(argv[argi], ',');
		if (!comma) {
			fprintf(stderr, "--ssid1 requires LID,CZ (e.g. --ssid1 10,1)\n");
			return -EINVAL;
		}
		ssid = (uint32_t)strtoul(argv[argi], NULL, 10);
		nid = (uint32_t)strtoul(comma + 1, NULL, 10);
		if (ssid > 511) {
			fprintf(stderr, "SSID1 LID must be 0-511.\n");
			return -EINVAL;
		}
		if (nid > 31) {
			fprintf(stderr, "SSID1 CZ must be 0-31.\n");
			return -EINVAL;
		}
		break;
	}
	case OPT_SSID2: {
		/* --ssid2 CC,TMF */
		char *comma = strchr(argv[argi], ',');
		if (!comma) {
			fprintf(stderr, "--ssid2 requires CC,TMF (e.g. --ssid2 440,1)\n");
			return -EINVAL;
		}
		country_code = (uint32_t)strtoul(argv[argi], NULL, 10);
		tmf_flags = (uint32_t)strtoul(comma + 1, NULL, 10);
		if (country_code > 1023) {
			fprintf(stderr, "SSID2 CC must be 0-1023.\n");
			return -EINVAL;
		}
		if (tmf_flags > 15) {
			fprintf(stderr, "SSID2 TMF must be 0-15.\n");
			return -EINVAL;
		}
		break;
	}
	case OPT_ROAMING:
		roaming_enabled = 1;
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
	case OPT_RETRANSMIT:
		default_retransmit = atoi(argv[argi]);
		if (default_retransmit < 0 || default_retransmit > 15) {
			fprintf(stderr, "Retransmit count must be 0-15, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_RETRANSMIT_INTERVAL:
		default_retransmit_interval = atoi(argv[argi]);
		if (default_retransmit_interval < 1 || default_retransmit_interval > 1920) {
			fprintf(stderr, "Retransmit interval must be 1-1920 frames, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_SEND_DELAY:
		default_send_delay = atoi(argv[argi]);
		if (default_send_delay < 0 || default_send_delay > 1920) {
			fprintf(stderr, "Send delay must be 0-1920 frames, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_HACK_NONSTANDARD:
		hack_nonstandard_decoders = 1;
		break;
	case OPT_FIFO:
		msg_send_path = options_strdup(argv[argi++]);
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
			       int *charset, int *is_temp_group,
			       char *source_id, int *phase, int *blocking_length,
			       int *mail_drop,
			       int *secure_encoding, int *secure_subtype,
			       int *numbered_msgnum,
			       int *msg_chan_setup,
			       int *retransmit, int *retransmit_interval,
			       int *send_delay,
			       int *short_msg_type, int *short_msg_source,
			       int *short_msg_number, int *short_msg_r,
			       int *instr_type, int *instr_slot, int *instr_frame)
{
	char buf[256];
	char *p, *key, *val;
	int len;

	/* defaults — use fixed speed/modulation if set, else 1600/2FSK */
	*speed = (fixed_speed > 0) ? fixed_speed : 1600;
	*modulation_type = (fixed_speed > 0) ? fixed_mod_type : FLEX_MOD_2FSK;
	*polarity_out = FLEX_DEFAULT_POLARITY;
	*priority = 0;
	*charset = 0;
	*is_temp_group = 0;
	source_id[0] = '\0';
	*phase = -1;
	*blocking_length = default_blocking_length;
	*mail_drop = 0;
	*secure_encoding = 0;	/* wire encoding: 0=7-bit alpha, 1=raw binary */
	*secure_subtype = -1;	/* pager-side type tag; -1 = derive from encoding */
	*numbered_msgnum = -1;	/* sequence number for dedup; -1 = auto from counter */
	*msg_chan_setup = -1;	/* -1 = not set (use global default) */
	*retransmit = default_retransmit;
	*retransmit_interval = default_retransmit_interval;
	*send_delay = default_send_delay;
	*short_msg_type = FLEX_SMSG_TYPE_NUMERIC;
	*short_msg_source = 0;
	*short_msg_number = 0;
	*short_msg_r = 0;
	*instr_type = -1;	/* -1 = not set (raw decimal in message field) */
	*instr_slot = 0;
	*instr_frame = 0;

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
			if (!strcasecmp(val, "normal") || !strcasecmp(val, "positive") ||
			    !strcasecmp(val, "pos"))
				*polarity_out = 1.0;
			else if (!strcasecmp(val, "inverted") || !strcasecmp(val, "negative") ||
				 !strcasecmp(val, "neg"))
				*polarity_out = -1.0;
		}
		else if (!strcmp(key, "priority"))
			*priority = atoi(val);
		else if (!strcmp(key, "charset")) {
			if (!strcasecmp(val, "kanji"))
				*charset = 1;
			else
				*charset = 0;
		}
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
				*secure_encoding = FLEX_SEC_ENC_ALPHA;
			else if (!strcasecmp(val, "binary"))
				*secure_encoding = FLEX_SEC_ENC_BINARY;
			else if (!strcasecmp(val, "regack"))
				*secure_encoding = FLEX_SEC_ENC_REGACK;
			else {
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: invalid sectype '%s', must be alpha, binary, or regack.\n", val);
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
		else if (!strcmp(key, "retransmit")) {
			int rv = atoi(val);
			if (rv < 0) rv = 0;
			if (rv > 15) rv = 15;
			*retransmit = rv;
		}
		else if (!strcmp(key, "retransmit_interval")) {
			int ri = atoi(val);
			if (ri < 1) {
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: retransmit_interval %d clamped to 1.\n", ri);
				ri = 1;
			}
			if (ri > 1920) {
				LOGP(DFLEX, LOGL_NOTICE, "FIFO: retransmit_interval %d clamped to 1920.\n", ri);
				ri = 1920;
			}
			*retransmit_interval = ri;
		}
		else if (!strcmp(key, "send_delay")) {
			int sd = atoi(val);
			if (sd < 0) sd = 0;
			if (sd > 1920) sd = 1920;
			*send_delay = sd;
		}
		else if (!strcmp(key, "stype")) {
			/* Short message sub-type (§3.9.2 Table 3.9.2-1):
			 *   numeric (0): 3-digit BCD (short) / 8-digit (long)
			 *   source  (1): source codes S (0-7)
			 *   numbered(2): source + message number N + R flag
			 * Default: numeric */
			if (!strcasecmp(val, "numeric") || !strcmp(val, "0"))
				*short_msg_type = FLEX_SMSG_TYPE_NUMERIC;
			else if (!strcasecmp(val, "source") || !strcmp(val, "1"))
				*short_msg_type = FLEX_SMSG_TYPE_SOURCE;
			else if (!strcasecmp(val, "numbered") || !strcmp(val, "2"))
				*short_msg_type = FLEX_SMSG_TYPE_NUMBERED;
			else
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: invalid stype '%s' — use numeric, source, or numbered.\n", val);
		}
		else if (!strcmp(key, "ssource")) {
			int sv = atoi(val);
			if (sv >= 0 && sv <= 7)
				*short_msg_source = sv;
			else
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: ssource %d out of range (0-7).\n", sv);
		}
		else if (!strcmp(key, "sr")) {
			*short_msg_r = atoi(val) ? 1 : 0;
		}
		else if (!strcmp(key, "itype")) {
			/* Short instruction sub-type (§3.9.6 Table 3.9.6-1):
			 *   tempaddr (0): temp address assignment (slot + frame)
			 *   sysevent (1): system event notification (event flags)
			 * When set, the message field is interpreted per type
			 * instead of as a raw decimal integer. */
			if (!strcasecmp(val, "tempaddr") || !strcmp(val, "0"))
				*instr_type = FLEX_INSTR_TYPE_TEMP_ADDR;
			else if (!strcasecmp(val, "sysevent") || !strcmp(val, "1"))
				*instr_type = FLEX_INSTR_TYPE_SYS_EVENT;
			else
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: invalid itype '%s' — use tempaddr or sysevent.\n", val);
		}
		else if (!strcmp(key, "islot")) {
			int sv = atoi(val);
			if (sv >= 0 && sv <= 15)
				*instr_slot = sv;
			else
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: islot %d out of range (0-15).\n", sv);
		}
		else if (!strcmp(key, "iframe")) {
			int fv = atoi(val);
			if (fv >= 0 && fv <= 127)
				*instr_frame = fv;
			else
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: iframe %d out of range (0-127).\n", fv);
		}
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

/* --- FIFO logging helpers --- */

static const char *flex_mod_name(int mod_type)
{
	return (mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk";
}

static const char *flex_phase_name(int phase)
{
	static const char *names[] = { "A", "B", "C", "D" };
	if (phase < 0) return "auto";
	if (phase >= 0 && phase <= 3) return names[phase];
	return "?";
}

/*
 * ARIB STD-43A Appendix A Extended CAPCODE parser.
 *
 * CAPCODE is the unified notation for FLEX pager addressing that bundles
 * the numeric address with frame/phase/collapse metadata and pager type
 * information into a single string.
 *
 * Formats (Appendix A Section 1):
 *
 *   Plain numeric:
 *     1234567              — 7-digit short address (1 to 1,933,312)
 *     123456789            — 9-digit long address  (2,101,249 to 4,297,068,542)
 *
 *   With alpha prefix (standard or non-standard rule):
 *     [b]<alpha><digits>   — alpha prefix encodes pager type + phase rule
 *
 *   Form 1: fffbU1234567   — frame(fff) + collapse(b) + alpha + 7-digit short
 *   Form 2: U1234567       — alpha + 7-digit short (frame/collapse from system)
 *   Form 3: bA1234567      — collapse(b) + alpha + 7-digit short
 *   Form 4: A1234567       — alpha + 7-digit short
 *   (Same patterns with 9-digit for long addresses, 10-digit for extended)
 *
 * Alpha prefix meanings (Section 4):
 *   Standard rule (frame/phase embedded in address digits):
 *     A = Single Phase, subtract 0 (1st address)
 *     B = Single Phase, subtract 1 (2nd address)
 *     C = Single Phase, subtract 2 (3rd address)
 *     D = Single Phase, subtract 3 (4th address)
 *     E = Any Phase, subtract 0
 *     F = Any Phase, subtract 1
 *     G = Any Phase, subtract 2
 *     H = Any Phase, subtract 3
 *     I = All Phase, subtract 0
 *     J = All Phase, subtract 1
 *     K = All Phase, subtract 2
 *     L = All Phase, subtract 3
 *
 *   Non-standard (frame/phase NOT embedded, explicit assignment):
 *     U = Single Phase, Phase 0 (A)
 *     V = Single Phase, Phase 1 (B)
 *     W = Single Phase, Phase 2 (C)
 *     X = Single Phase, Phase 3 (D)
 *     Y = Any Phase
 *     Z = All Phase
 *
 *   2nd alpha (roaming capability, prepended as P/Q/R/S):
 *     P = no Frame Offset, no TMF
 *     Q = no Frame Offset, TMF capable
 *     R = Frame Offset capable, no TMF
 *     S = Frame Offset + TMF capable
 *
 * Collapse cycle 'b' (0-7):
 *   Pager wakes every 2^b frames.
 *   b=0: every frame, b=1: every 2, ..., b=7: every 128 frames.
 *
 * Frame/phase extraction (Section 3, standard rule A-L):
 *   phase = (address / 4) mod 4       → 0=A, 1=B, 2=C, 3=D
 *   frame = (address / 16) mod 128    → 0-127
 *
 * This function parses the string, extracts the numeric capcode,
 * and optionally populates metadata (collapse, phase override, etc.)
 * for human-readable logging.  The numeric capcode is what gets
 * passed to the encoder — the alpha prefix is informational.
 *
 * Returns 0 on success, -1 on parse error.
 */

/* Parsed CAPCODE metadata for logging */
typedef struct capcode_parsed {
	uint64_t	capcode;	/* numeric address for encoder */
	int		has_alpha;	/* 1 if alpha prefix was present */
	char		alpha;		/* primary alpha char (A-Z) */
	int		has_roaming;	/* 1 if 2nd alpha (P/Q/R/S) present */
	char		roaming;	/* 2nd alpha char */
	int		has_collapse;	/* 1 if collapse digit present */
	int		collapse;	/* collapse value 0-7 */
	int		has_frame;	/* 1 if fff prefix present */
	int		frame;		/* explicit frame 0-127 */
	/* Derived from standard rule (A-L) + numeric address */
	int		std_rule;	/* 1 if alpha is A-L (standard rule) */
	int		computed_phase;	/* phase from address: (addr/4)%4 */
	int		computed_frame;	/* frame from address: (addr/16)%128 */
	const char	*phase_type;	/* "Single", "Any", or "All" */
	int		subtract;	/* address offset (0-3) for multi-addr pagers */
} capcode_parsed_t;

/* Classify the alpha prefix per ARIB STD-43A Appendix A Section 4 */
static void classify_capcode_alpha(char alpha, const char **phase_type,
				   int *subtract, int *std_rule,
				   int *explicit_phase)
{
	*std_rule = 0;
	*explicit_phase = -1;
	*subtract = 0;

	switch (alpha) {
	/* Standard rule: frame/phase embedded in address */
	case 'A': *phase_type = "Single"; *subtract = 0; *std_rule = 1; break;
	case 'B': *phase_type = "Single"; *subtract = 1; *std_rule = 1; break;
	case 'C': *phase_type = "Single"; *subtract = 2; *std_rule = 1; break;
	case 'D': *phase_type = "Single"; *subtract = 3; *std_rule = 1; break;
	case 'E': *phase_type = "Any";    *subtract = 0; *std_rule = 1; break;
	case 'F': *phase_type = "Any";    *subtract = 1; *std_rule = 1; break;
	case 'G': *phase_type = "Any";    *subtract = 2; *std_rule = 1; break;
	case 'H': *phase_type = "Any";    *subtract = 3; *std_rule = 1; break;
	case 'I': *phase_type = "All";    *subtract = 0; *std_rule = 1; break;
	case 'J': *phase_type = "All";    *subtract = 1; *std_rule = 1; break;
	case 'K': *phase_type = "All";    *subtract = 2; *std_rule = 1; break;
	case 'L': *phase_type = "All";    *subtract = 3; *std_rule = 1; break;
	/* Non-standard: explicit phase assignment */
	case 'U': *phase_type = "Single"; *explicit_phase = 0; break;
	case 'V': *phase_type = "Single"; *explicit_phase = 1; break;
	case 'W': *phase_type = "Single"; *explicit_phase = 2; break;
	case 'X': *phase_type = "Single"; *explicit_phase = 3; break;
	case 'Y': *phase_type = "Any";    break;
	case 'Z': *phase_type = "All";    break;
	default:  *phase_type = "Unknown"; break;
	}
}

static int parse_capcode_string(const char *str, capcode_parsed_t *out)
{
	const char *p = str;
	char *endptr;

	memset(out, 0, sizeof(*out));
	out->collapse = -1;
	out->frame = -1;
	out->computed_phase = -1;
	out->computed_frame = -1;
	out->phase_type = "Unknown";

	if (!str || !*str)
		return -1;

	/*
	 * Detect whether this is a plain numeric capcode or an Extended
	 * CAPCODE with alpha prefix.  Plain numeric strings contain only
	 * digits 0-9.  Extended CAPCODEs contain at least one letter A-Z.
	 */
	{
		int has_alpha = 0;
		const char *c;
		for (c = str; *c; c++) {
			char ch = *c;
			if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z')) {
				has_alpha = 1;
				break;
			}
		}

		if (!has_alpha) {
			/* Plain numeric capcode — pass through directly */
			out->capcode = strtoull(str, &endptr, 10);
			if (*endptr != '\0')
				return -1;
			/* Compute standard frame/phase for logging */
			out->computed_frame = (int)((out->capcode / 16) % 128);
			out->computed_phase = (int)((out->capcode / 4) % 4);
			return 0;
		}
	}

	/*
	 * Extended CAPCODE parsing.
	 *
	 * Scan left-to-right, consuming optional components:
	 *   [R|S|P|Q]          — optional 2nd alpha (roaming), if followed by
	 *                        another alpha or digit+alpha
	 *   [fff]              — optional 1-3 digit frame number (0-127)
	 *   [b]                — optional 1 digit collapse (0-7)
	 *   <A-Z>              — required primary alpha prefix
	 *   <digits>           — required numeric address (7, 9, or 10 digits)
	 *
	 * The tricky part: a leading digit could be frame prefix OR collapse.
	 * We use a right-to-left approach: find the alpha, then parse what's
	 * before it as optional frame+collapse, and what's after as the address.
	 */

	/* Find the rightmost alpha character — that's the primary alpha prefix.
	 * Everything after it must be digits (the address).
	 * Everything before it is optional frame/collapse/roaming. */
	{
		const char *alpha_pos = NULL;
		const char *c;
		int explicit_phase = -1;

		/* Scan for the last alpha character before the digit run */
		for (c = p; *c; c++) {
			char ch = *c;
			if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'))
				alpha_pos = c;
		}

		if (!alpha_pos)
			return -1; /* no alpha found — shouldn't happen */

		/* Verify everything after alpha is digits */
		{
			const char *d;
			for (d = alpha_pos + 1; *d; d++) {
				if (*d < '0' || *d > '9')
					return -1;
			}
		}

		/* Extract the numeric address (after alpha) */
		out->capcode = strtoull(alpha_pos + 1, &endptr, 10);
		if (*endptr != '\0')
			return -1;

		/* Extract primary alpha (uppercase) */
		out->alpha = (*alpha_pos >= 'a' && *alpha_pos <= 'z')
			     ? (*alpha_pos - 'a' + 'A') : *alpha_pos;
		out->has_alpha = 1;

		/* Classify the alpha prefix */
		classify_capcode_alpha(out->alpha, &out->phase_type,
				       &out->subtract, &out->std_rule,
				       &explicit_phase);

		/* Parse what's before the alpha: [roaming][frame][collapse] */
		{
			int prefix_len = (int)(alpha_pos - p);
			char prefix[16];
			int pi = 0;

			if (prefix_len > 0 && prefix_len < (int)sizeof(prefix)) {
				memcpy(prefix, p, prefix_len);
				prefix[prefix_len] = '\0';

				/* Check for leading roaming alpha (P/Q/R/S) */
				{
					char ch = prefix[pi];
					if (ch == 'P' || ch == 'p' ||
					    ch == 'Q' || ch == 'q' ||
					    ch == 'R' || ch == 'r' ||
					    ch == 'S' || ch == 's') {
						out->has_roaming = 1;
						out->roaming = (ch >= 'a') ? (ch - 'a' + 'A') : ch;
						pi++;
					}
				}

				/* Remaining chars before alpha are digits: [fff][b]
				 * The last digit is collapse (0-7).
				 * Preceding digits (if any) are frame number (0-127). */
				{
					int remaining = prefix_len - pi;
					if (remaining > 0) {
						/* Last digit = collapse */
						char collapse_ch = prefix[prefix_len - 1];
						if (collapse_ch >= '0' && collapse_ch <= '7') {
							out->has_collapse = 1;
							out->collapse = collapse_ch - '0';
						}

						/* Digits before collapse = frame */
						if (remaining > 1) {
							char frame_buf[8];
							int flen = remaining - 1;
							memcpy(frame_buf, prefix + pi, flen);
							frame_buf[flen] = '\0';
							out->has_frame = 1;
							out->frame = atoi(frame_buf);
							if (out->frame < 0 || out->frame > 127) {
								LOGP(DFLEX, LOGL_NOTICE,
								     "CAPCODE: frame %d out of range (0-127), ignoring.\n",
								     out->frame);
								out->has_frame = 0;
								out->frame = -1;
							}
						}
					}
				}
			}
		}

		/* Compute standard-rule frame/phase from the numeric address.
		 * Per Appendix A Section 3:
		 *   phase = (address / 4) mod 4
		 *   frame = (address / 16) mod 128
		 * For multi-address pagers (subtract > 0), the standard rule
		 * says subtract N from the capcode before computing, so all
		 * addresses for one pager land on the same frame. */
		{
			uint64_t adj = out->capcode;
			if (out->std_rule && out->subtract > 0 && adj >= (uint64_t)out->subtract)
				adj -= out->subtract;
			out->computed_frame = (int)((adj / 16) % 128);
			out->computed_phase = (int)((adj / 4) % 4);
		}
	}

	return 0;
}

/* Log human-readable Extended CAPCODE breakdown.
 * Called after successful parse to explain what the CAPCODE means. */
static void log_capcode_parsed(const capcode_parsed_t *cp, const char *raw_str)
{
	static const char *phase_names[] = { "A", "B", "C", "D" };
	static const char *roaming_desc[] = {
		[0] = "no FrameOffset, no TMF",		/* P */
		[1] = "no FrameOffset, TMF capable",	/* Q */
		[2] = "FrameOffset capable, no TMF",	/* R */
		[3] = "FrameOffset + TMF capable",	/* S */
	};

	if (!cp->has_alpha) {
		/* Plain numeric — just log computed frame/phase */
		LOGP(DFLEX, LOGL_INFO,
		     "CAPCODE: plain numeric %" PRIu64 " → frame=%d phase=%s (%s)\n",
		     cp->capcode,
		     cp->computed_frame,
		     (cp->computed_phase >= 0 && cp->computed_phase <= 3)
		       ? phase_names[cp->computed_phase] : "?",
		     flex_capcode_type_name(cp->capcode));
		return;
	}

	/* Extended CAPCODE — full breakdown */
	LOGP(DFLEX, LOGL_INFO,
	     "CAPCODE: parsed '%s' → address=%" PRIu64 " alpha=%c (%s Phase, subtract %d%s)\n",
	     raw_str, cp->capcode, cp->alpha,
	     cp->phase_type, cp->subtract,
	     cp->std_rule ? ", standard rule" : ", non-standard");

	if (cp->has_collapse)
		LOGP(DFLEX, LOGL_INFO,
		     "CAPCODE:   collapse=%d (wake every %d frames = %.1fs interval)\n",
		     cp->collapse, 1 << cp->collapse,
		     (1 << cp->collapse) * 1.875);

	if (cp->has_frame)
		LOGP(DFLEX, LOGL_INFO,
		     "CAPCODE:   explicit frame=%d (from CAPCODE prefix)\n",
		     cp->frame);

	if (cp->has_roaming) {
		int ri = cp->roaming - 'P'; /* P=0, Q=1, R=2, S=3 */
		LOGP(DFLEX, LOGL_INFO,
		     "CAPCODE:   roaming=%c (%s)\n",
		     cp->roaming,
		     (ri >= 0 && ri <= 3) ? roaming_desc[ri] : "unknown");
	}

	if (cp->std_rule) {
		LOGP(DFLEX, LOGL_INFO,
		     "CAPCODE:   standard rule → computed frame=%d phase=%s "
		     "(from address: frame=(%" PRIu64 "/16)%%128=%d, phase=(%" PRIu64 "/4)%%4=%d)\n",
		     cp->computed_frame,
		     (cp->computed_phase >= 0 && cp->computed_phase <= 3)
		       ? phase_names[cp->computed_phase] : "?",
		     cp->capcode, cp->computed_frame,
		     cp->capcode, cp->computed_phase);
	}

	LOGP(DFLEX, LOGL_INFO,
	     "CAPCODE:   address type: %s\n",
	     flex_capcode_type_name(cp->capcode));
}

/* Build a human-readable flags string like "[TGRP MAILDROP RETX]".
 * Returns pointer to static buffer. */
static const char *flex_flags_str(int is_temp_group,
				  int mail_drop, int secure_encoding,
				  int retransmit)
{
	static char buf[64];
	int pos = 0;

	buf[pos++] = '[';
	if (is_temp_group)   pos += sprintf(buf + pos, "TGRP ");
	if (mail_drop)       pos += sprintf(buf + pos, "MAILDROP ");
	if (secure_encoding) pos += sprintf(buf + pos, "SECBIN ");
	if (retransmit > 0)  pos += sprintf(buf + pos, "RETX ");
	/* trim trailing space */
	if (pos > 1 && buf[pos - 1] == ' ')
		pos--;
	buf[pos++] = ']';
	buf[pos] = '\0';
	return buf;
}

/* Log message payload: text preview for alpha/numeric, hex dump for hex type. */
static void flex_log_payload(enum flex_msg_type mtype,
			     const char *data, int len)
{
	if (len <= 0)
		return;

	if (mtype == FLEX_MSG_TYPE_HEX) {
		char hex_preview[1280]; /* 420 bytes * 3 chars + 1 */
		int i;
		for (i = 0; i < len && i < 420; i++)
			sprintf(hex_preview + i * 3, "%02X ", (unsigned char)data[i]);
		if (len > 0)
			hex_preview[len * 3 - 1] = '\0';
		LOGP(DFLEX, LOGL_INFO,
		     "FIFO:   payload (hex): '%s'\n", hex_preview);
	} else {
		LOGP(DFLEX, LOGL_INFO,
		     "FIFO:   payload: '%.*s'\n", len, data);
	}
}

/* Log type-specific options for secure, numbered, and hex messages. */
static void flex_log_type_options(enum flex_msg_type mtype,
				  int secure_encoding, int secure_subtype,
				  int numbered_msgnum, int blocking_length)
{
	switch (mtype) {
	case FLEX_MSG_TYPE_SECURE:
		LOGP(DFLEX, LOGL_INFO,
		     "FIFO:   secure options: sectype=%s sectag=%d\n",
		     secure_encoding ? "binary" : "alpha", secure_subtype);
		break;
	case FLEX_MSG_TYPE_NUMBERED_NUM:
	case FLEX_MSG_TYPE_NUMBERED_SPECIAL:
		LOGP(DFLEX, LOGL_INFO,
		     "FIFO:   numbered options: msgnum=%d\n",
		     numbered_msgnum);
		break;
	case FLEX_MSG_TYPE_HEX:
		LOGP(DFLEX, LOGL_INFO,
		     "FIFO:   hex options: blocking=%d\n",
		     blocking_length);
		break;
	default:
		break;
	}
}

/* Parse message type string. Returns FLEX_MSG_TYPE_AUTO for unknown types. */
static enum flex_msg_type flex_parse_msg_type(const char *s)
{
	if (!strcasecmp(s, "auto") || !strcmp(s, "0"))
		return FLEX_MSG_TYPE_AUTO;
	if (!strcasecmp(s, "tone") || !strcmp(s, "1"))
		return FLEX_MSG_TYPE_TONE;
	if (!strcasecmp(s, "numeric") || !strcasecmp(s, "num") || !strcmp(s, "2"))
		return FLEX_MSG_TYPE_NUMERIC;
	if (!strcasecmp(s, "alpha") || !strcasecmp(s, "alphanumeric") || !strcmp(s, "3"))
		return FLEX_MSG_TYPE_ALPHA;
	if (!strcasecmp(s, "hex") || !strcasecmp(s, "binary") || !strcmp(s, "4"))
		return FLEX_MSG_TYPE_HEX;
	if (!strcasecmp(s, "instruction") || !strcasecmp(s, "instr") || !strcmp(s, "5"))
		return FLEX_MSG_TYPE_INSTRUCTION;
	if (!strcasecmp(s, "short") || !strcmp(s, "6"))
		return FLEX_MSG_TYPE_SHORT;
	if (!strcasecmp(s, "secure") || !strcasecmp(s, "sec") || !strcmp(s, "7"))
		return FLEX_MSG_TYPE_SECURE;
	if (!strcasecmp(s, "special") || !strcasecmp(s, "snum") || !strcmp(s, "8"))
		return FLEX_MSG_TYPE_SPECIAL_NUM;
	if (!strcasecmp(s, "nnumeric") || !strcasecmp(s, "nnum") || !strcmp(s, "9"))
		return FLEX_MSG_TYPE_NUMBERED_NUM;
	if (!strcasecmp(s, "nspecial") || !strcasecmp(s, "nsnum") || !strcmp(s, "10"))
		return FLEX_MSG_TYPE_NUMBERED_SPECIAL;
	return FLEX_MSG_TYPE_AUTO; /* unknown */
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
	int msg_retransmit;
	int msg_retransmit_interval;
	int msg_send_delay;
	int msg_short_type;
	int msg_short_source;
	int msg_short_number;
	int msg_short_r;
	int msg_instr_type;
	int msg_instr_slot;
	int msg_instr_frame;
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
		int pol, p, s, m, any = 0;

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
		for (pol = 0; pol < FLEX_RX_POLARITIES; pol++) {
		for (p = 0; p < FLEX_MAX_PHASES; p++) {
			for (s = 0; s < FLEX_TEMP_ADDR_SLOTS; s++) {
				if (!flex->rx.temp_addr_map[pol][p][s].active)
					continue;
				int cnt = flex->rx.temp_addr_map[pol][p][s].count;
				if (cnt > 0) {
					LOGP(DFLEX, LOGL_NOTICE,
					     "  phase=%c group=%d members=%d target_frame=%u setup=C%u/F%u:",
					     pnames[p], s, cnt,
					     flex->rx.temp_addr_map[pol][p][s].target_frame,
					     flex->rx.temp_addr_map[pol][p][s].setup_cycle,
					     flex->rx.temp_addr_map[pol][p][s].setup_frame);
					for (m = 0; m < cnt; m++)
						LOGP(DFLEX, LOGL_NOTICE,
						     " %" PRIu64,
						     flex->rx.temp_addr_map[pol][p][s].capcodes[m]);
					LOGP(DFLEX, LOGL_NOTICE, "\n");
					any = 1;
				}
			}
		}
		}
		if (!any)
			LOGP(DFLEX, LOGL_NOTICE, "  (no active RX assignments)\n");

		/* TX temp group slots */
		any = 0;
		LOGP(DFLEX, LOGL_NOTICE, "FIFO: TX temp group slots (16 slots):\n");
		for (s = 0; s < FLEX_TEMP_ADDR_SLOTS; s++) {
			if (flex->tx_temp[s].state == FLEX_TG_FREE)
				continue;
			LOGP(DFLEX, LOGL_NOTICE,
			     "  slot=%d state=%s frame=%u members=%d:",
			     s,
			     flex->tx_temp[s].state == FLEX_TG_SETUP ? "SETUP" : "DELIVERY",
			     flex->tx_temp[s].target_frame,
			     flex->tx_temp[s].count);
			for (m = 0; m < flex->tx_temp[s].count; m++)
				LOGP(DFLEX, LOGL_NOTICE,
				     " %" PRIu64, flex->tx_temp[s].capcodes[m]);
			LOGP(DFLEX, LOGL_NOTICE, "\n");
			any = 1;
		}
		if (!any)
			LOGP(DFLEX, LOGL_NOTICE, "  (no active TX slots)\n");
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

	/* === System Message command: "sysmsg,<type>,lsb=<N> method=<m>,<message>" ===
	 * Sends a system message via Operator Messaging Address
	 * (capcode derived from FLEX_ADDR_OPER_MSG_MIN + LSB).
	 *
	 * Follows standard FIFO convention: capcode,type,options,message.
	 * "sysmsg" replaces the capcode field; the type field selects the
	 * message encoding (alpha, numeric, hex, etc.); lsb= in options
	 * selects the operator address sub-type (0-15); method= selects
	 * the transmission method per §3.9.2 Fig. 3.7.2-2.
	 *
	 * LSB values (§3.8.2.4 Table 3.8.2.4-1):
	 *   0 = all pagers    1 = home area    2 = roaming    3 = SSID
	 *   4 = time related  5-13 = reserved  14 = SSIDChange  15 = SysEvent
	 *
	 * Method (§3.9.2, Fig. 3.7.2-2):
	 *   a = BIW101 only — no operator address, vector at end of VF
	 *   b = BIW101 + Operator Messaging Address (default)
	 *   c = Operator Messaging Address only — no BIW101
	 *
	 * Secure (V=000) is rejected per §3.9.2: "all Vector types are
	 * valid except the Secure Vector."
	 *
	 * Examples:
	 *   sysmsg,alpha,,System maintenance at 03:00 UTC
	 *   sysmsg,alpha,lsb=home,Home area update
	 *   sysmsg,alpha,lsb=all method=c,Legacy mode test
	 *   sysmsg,numeric,,31415926
	 *   sysmsg,tone,lsb=ssidchange,
	 *
	 * Note: Tone-Only Addresses cannot be transmitted in frames
	 * used for transmitting System Messages. */
	if (!strcasecmp(capcode_string, "sysmsg")) {
		flex_t *flex = (flex_t *)sender_head;
		int lsb = 0; /* default: all pagers */
		char method = 'b'; /* default: BIW101 + Operator Messaging Address */
		enum flex_msg_type smtype;

		if (!flex) {
			LOGP(DFLEX, LOGL_ERROR, "No transmitter instance for sysmsg command.\n");
			return;
		}

		/* Parse type from the type field (between 1st and 2nd comma) */
		{
			int tlen = comma2 - comma1 - 1;
			char tbuf[32];
			if (tlen <= 0 || tlen >= (int)sizeof(tbuf)) {
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: sysmsg requires type. Format: sysmsg,<type>,lsb=<target>,<message>\n"
				     "  Types: alpha, numeric, special, hex, tone\n"
				     "  lsb: all(0) home(1) roaming(2) ssid(3) time(4) ssidchange(14) sysevent(15)\n");
				return;
			}
			memcpy(tbuf, text + comma1 + 1, tlen);
			tbuf[tlen] = '\0';

			if (!strcasecmp(tbuf, "alpha"))
				smtype = FLEX_MSG_TYPE_ALPHA;
			else if (!strcasecmp(tbuf, "numeric"))
				smtype = FLEX_MSG_TYPE_NUMERIC;
			else if (!strcasecmp(tbuf, "special"))
				smtype = FLEX_MSG_TYPE_SPECIAL_NUM;
			else if (!strcasecmp(tbuf, "hex"))
				smtype = FLEX_MSG_TYPE_HEX;
			else if (!strcasecmp(tbuf, "tone"))
				smtype = FLEX_MSG_TYPE_TONE;
			else if (!strcasecmp(tbuf, "secure")) {
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: sysmsg type=secure rejected — "
				     "Secure vectors not allowed for system messages (§3.9.2).\n");
				return;
			} else {
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: sysmsg invalid type '%s' — "
				     "use alpha, numeric, special, hex, or tone.\n", tbuf);
				return;
			}
		}

		/* Parse lsb= from options field (between 2nd and 3rd comma).
		 * Accepts numeric (0-15) or human-readable names:
		 *   all, home, roaming, ssid, time, ssidchange, sysevent */
		{
			int olen = comma3 - comma2 - 1;
			if (olen > 0) {
				char obuf[64];
				int ol = olen < (int)sizeof(obuf) - 1 ? olen : (int)sizeof(obuf) - 1;
				memcpy(obuf, text + comma2 + 1, ol);
				obuf[ol] = '\0';
				char *lp = strstr(obuf, "lsb=");
				if (lp) {
					char *lv = lp + 4;
					char *le = lv;
					while (*le && *le != ' ') le++;
					char saved = *le;
					*le = '\0';
					if (!strcasecmp(lv, "all"))
						lsb = FLEX_OPER_MSG_ALL_PAGERS;
					else if (!strcasecmp(lv, "home"))
						lsb = FLEX_OPER_MSG_HOME;
					else if (!strcasecmp(lv, "roaming"))
						lsb = FLEX_OPER_MSG_ROAMING;
					else if (!strcasecmp(lv, "ssid"))
						lsb = FLEX_OPER_MSG_SSID;
					else if (!strcasecmp(lv, "time"))
						lsb = FLEX_OPER_MSG_TIME;
					else if (!strcasecmp(lv, "ssidchange"))
						lsb = FLEX_OPER_MSG_SSID_CHANGE;
					else if (!strcasecmp(lv, "sysevent"))
						lsb = FLEX_OPER_MSG_SYS_EVENT;
					else
						lsb = atoi(lv);
					*le = saved;
				}
				/* method= transmission method (a/b/c) */
				char *mp = strstr(obuf, "method=");
				if (mp) {
					char mv = mp[7];
					if (mv == 'a' || mv == 'A')
						method = 'a';
					else if (mv == 'b' || mv == 'B')
						method = 'b';
					else if (mv == 'c' || mv == 'C')
						method = 'c';
					else {
						LOGP(DFLEX, LOGL_NOTICE,
						     "FIFO: sysmsg invalid method='%c'"
						     " — use a, b, or c.\n", mv);
						return;
					}
				}
			}
		}

		if (lsb < 0 || lsb > 15) {
			LOGP(DFLEX, LOGL_NOTICE, "FIFO: sysmsg lsb=%d out of range (0-15).\n", lsb);
			return;
		}

		if (method == 'a' && lsb > 3) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: sysmsg method=a only valid for"
			     " lsb=0-3 (audience types), not %d."
			     " Using method=b instead.\n", lsb);
			method = 'b';
		}

		/* Extract message payload (after third comma) */
		if (comma3 + 1 < text_length) {
			message_length = flex_scan_message(text + comma3 + 1,
							   text_length - comma3 - 1,
							   msg_buf, sizeof(msg_buf));
		}

		/* Compute capcode: aw = OPER_MSG_MIN + LSB, cap = aw - OFFSET */
		{
			uint64_t oper_capcode = (uint64_t)(FLEX_ADDR_OPER_MSG_MIN
						- FLEX_SHORT_ADDR_OFFSET
						+ (uint32_t)lsb);
			flex_msg_t *msg;

			msg = flex_msg_create(flex, oper_capcode, smtype,
					      msg_buf, message_length);
			if (msg) {
				msg->speed = 1600;
				msg->modulation_type = FLEX_MOD_2FSK;
				msg->polarity = FLEX_DEFAULT_POLARITY;
				msg->priority = 1;
				msg->numbered_r = 0; /* R=0 per §3.9.2 */
				msg->sysmsg_method = method;
				LOGP(DFLEX, LOGL_INFO,
				     "FIFO: sysmsg enqueued LSB=%d (%s) type=%s method=%c capcode=%" PRIu64 " len=%d R=0\n",
				     lsb, flex_oper_msg_subtype_name((uint32_t)(FLEX_ADDR_OPER_MSG_MIN + lsb)),
				     flex_msg_type_name(smtype),
				     method,
				     oper_capcode, message_length);
			}
		}
		return;
	}

	/* Handle tempgroup:cap1 cap2 cap3 prefix (§3.8.2.3).
	 * Format: "tempgroup:cap1 cap2 cap3,type,options,message"
	 * Automatically handles SETUP → DELIVERY → TEARDOWN. */
	if (strncmp(capcode_string, "tempgroup:", 10) == 0) {
		flex_t *flex = (flex_t *)sender_head;
		char *tg_str = capcode_string + 10;
		uint64_t tg_caps[FLEX_TEMP_GROUP_MAX_MEMBERS];
		int tg_count = 0;
		char *p = tg_str;
		char type_str_tg[64];
		int tlen_tg;
		enum flex_msg_type mtype_tg;
		int tg_speed = 1600, tg_mod = FLEX_MOD_2FSK;
		double tg_pol = FLEX_DEFAULT_POLARITY;
		int tg_prio = 0, tg_phase = -1;

		if (!flex) {
			LOGP(DFLEX, LOGL_ERROR, "No transmitter for tempgroup.\n");
			return;
		}

		/* Parse space-separated capcodes */
		while (*p && tg_count < FLEX_TEMP_GROUP_MAX_MEMBERS) {
			while (*p == ' ') p++;
			if (!*p) break;
			tg_caps[tg_count++] = strtoull(p, &p, 10);
		}
		if (tg_count == 0) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: tempgroup requires at least 1 capcode.\n");
			return;
		}

		/* Extract type field */
		tlen_tg = comma2 - comma1 - 1;
		if (tlen_tg <= 0 || tlen_tg >= (int)sizeof(type_str_tg)) {
			LOGP(DFLEX, LOGL_NOTICE, "FIFO: tempgroup missing type field.\n");
			return;
		}
		memcpy(type_str_tg, text + comma1 + 1, tlen_tg);
		type_str_tg[tlen_tg] = '\0';

		/* Parse type */
		mtype_tg = flex_parse_msg_type(type_str_tg);
		if (mtype_tg == FLEX_MSG_TYPE_AUTO) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: tempgroup unknown type '%s'.\n", type_str_tg);
			return;
		}

		/* Extract and parse options (speed, polarity, phase, priority) */
		{
			int dummy_charset, dummy_tg;
			char dummy_src[64];
			int dummy_bl, dummy_md, dummy_se, dummy_ss, dummy_nm;
			int dummy_cs, dummy_rt, dummy_ri, dummy_sd;
			int dummy_st, dummy_ssrc, dummy_sn, dummy_sr;
			int dummy_it, dummy_is, dummy_if;
			parse_fifo_options(text + comma2 + 1, comma3 - comma2 - 1,
					   &tg_speed, &tg_mod,
					   &tg_pol, &tg_prio,
					   &dummy_charset, &dummy_tg,
					   dummy_src, &tg_phase, &dummy_bl,
					   &dummy_md,
					   &dummy_se, &dummy_ss,
					   &dummy_nm,
					   &dummy_cs,
					   &dummy_rt, &dummy_ri,
					   &dummy_sd,
					   &dummy_st, &dummy_ssrc,
					   &dummy_sn, &dummy_sr,
					   &dummy_it, &dummy_is, &dummy_if);
		}

		/* Extract message payload */
		if (comma3 + 1 < text_length)
			message_length = flex_scan_message(text + comma3 + 1,
							   text_length - comma3 - 1,
							   msg_buf, sizeof(msg_buf));

		/* Enqueue */
		{
			int rc = flex_tempgroup_enqueue(flex, tg_caps, tg_count,
						       mtype_tg, msg_buf, message_length,
						       tg_speed, tg_mod, tg_pol,
						       tg_prio, tg_phase);
			if (rc < 0)
				LOGP(DFLEX, LOGL_NOTICE,
				     "FIFO: tempgroup enqueue failed.\n");
		}
		return;
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
			   &msg_charset, &is_temp_group,
			   msg_source, &msg_phase, &msg_blocking_length,
			   &msg_mail_drop,
			   &msg_secure_encoding, &msg_secure_subtype,
			   &msg_numbered_msgnum,
			   &msg_chan_setup,
			   &msg_retransmit, &msg_retransmit_interval,
			   &msg_send_delay,
			   &msg_short_type, &msg_short_source,
			   &msg_short_number, &msg_short_r,
			   &msg_instr_type, &msg_instr_slot, &msg_instr_frame);

	/* Discard message if sectype was invalid */
	if (msg_secure_encoding == -1)
		return;

	/* Parse capcode — supports both plain numeric and Extended CAPCODE
	 * format per ARIB STD-43A Appendix A.
	 *
	 * Plain:    1234567 or 007005031
	 * Extended: 3E007005031  (collapse=3, alpha=E, address=007005031)
	 *           A1234567     (alpha=A, address=1234567)
	 *           0073U123456  (frame=007, collapse=3, alpha=U, address=123456)
	 *
	 * The parser extracts the numeric address and optional metadata
	 * (collapse, phase type, roaming capability) for logging.
	 * Only the numeric address is passed to the encoder. */
	{
		capcode_parsed_t cp;
		if (parse_capcode_string(capcode_string, &cp) < 0) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: cannot parse capcode '%s' — expected numeric or Extended CAPCODE format.\n",
			     capcode_string);
			return;
		}
		capcode = cp.capcode;
		if (!flex_capcode_valid(capcode)) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: invalid capcode %" PRIu64 " from '%s' "
			     "(valid: short %" PRIu64 "–%" PRIu64
			     ", long %" PRIu64 "–%" PRIu64 ").\n",
			     capcode, capcode_string,
			     (uint64_t)FLEX_SHORT_ADDR_MIN,
			     (uint64_t)FLEX_SHORT_ADDR_MAX,
			     (uint64_t)FLEX_LONG_ADDR_MIN,
			     (uint64_t)FLEX_LONG_ADDR_MAX);
			return;
		}
		log_capcode_parsed(&cp, capcode_string);

		/* Apply phase from Extended CAPCODE alpha prefix if not
		 * overridden by explicit phase= option.
		 * Non-standard U/V/W/X specify an explicit phase. */
		if (cp.has_alpha && !cp.std_rule) {
			int explicit_phase = -1;
			switch (cp.alpha) {
			case 'U': explicit_phase = 0; break;
			case 'V': explicit_phase = 1; break;
			case 'W': explicit_phase = 2; break;
			case 'X': explicit_phase = 3; break;
			}
			if (explicit_phase >= 0 && msg_phase < 0) {
				msg_phase = explicit_phase;
				LOGP(DFLEX, LOGL_INFO,
				     "CAPCODE: applying phase=%s from non-standard alpha '%c'.\n",
				     flex_phase_name(msg_phase), cp.alpha);
			}
		}
	}

	/* Validate message type.
	 * Accept names or numeric codes:
	 *   0=auto, 1=tone, 2=numeric, 3=alpha, 4=hex, 5=instruction, 6=short
	 * Also accept vector type numbers directly for advanced use:
	 *   vtype1=instruction, vtype2=tone, vtype3=numeric, vtype5=alpha, vtype6=hex */
	mtype = flex_parse_msg_type(type_string);
	if (mtype == FLEX_MSG_TYPE_AUTO && strcasecmp(type_string, "auto") && strcmp(type_string, "0")) {
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
				     flex_mod_name(flex->fixed_mod_type),
				     msg_speed,
				     flex_mod_name(msg_mod_type));
				return;
			}
			if (flex->fixed_polarity != 0.0 && msg_polarity != flex->fixed_polarity) {
				LOGP(DFLEX, LOGL_NOTICE, "fixed-mode: polarity locked to %s, rejecting message with polarity=%s\n",
				     (flex->fixed_polarity < 0) ? "inverted" : "normal",
				     (msg_polarity < 0) ? "inverted" : "normal");
				return;
			}
		}
	}

	/* Enqueue message on first transmitter instance */
	{
		flex_t *flex = (flex_t *)sender_head;
		if (flex) {
			flex_msg_t *msg;

			/* Pre-enqueue validation for secure binary: the
			 * secure_encoding option is parsed from the FIFO line
			 * but only set on the msg struct after create.
			 * Validate here so bad payloads never enter the queue. */
			if (mtype == FLEX_MSG_TYPE_SECURE && msg_secure_encoding == 1) {
				int verr = flex_msg_validate(FLEX_MSG_TYPE_SECURE,
							     msg_buf, message_length,
							     msg_secure_encoding);
				if (verr) {
					LOGP(DFLEX, LOGL_NOTICE,
					     "FIFO: rejecting secure binary capcode=%" PRIu64 ": invalid hex payload.\n",
					     capcode);
					return;
				}
			}

			/* Construct instruction data from structured options (§3.9.6).
			 * When itype= is set and type is instruction, build the
			 * 14-bit instruction data and replace the message field. */
			if (mtype == FLEX_MSG_TYPE_INSTRUCTION && msg_instr_type >= 0) {
				static char instr_buf[16];
				uint32_t idata = 0;

				if (msg_instr_type == FLEX_INSTR_TYPE_TEMP_ADDR) {
					idata = ((uint32_t)FLEX_INSTR_TYPE_TEMP_ADDR & FLEX_INSTR_TYPE_MASK);
					idata |= ((uint32_t)msg_instr_frame & FLEX_INSTR_FRAME_MASK)
						 << FLEX_INSTR_FRAME_SHIFT;
					idata |= ((uint32_t)msg_instr_slot & FLEX_INSTR_SLOT_MASK)
						 << FLEX_INSTR_SLOT_SHIFT;
				} else if (msg_instr_type == FLEX_INSTR_TYPE_SYS_EVENT) {
					uint32_t eflags = 0;
					if (message_length > 0) {
						char fbuf[256];
						int flen = message_length;
						if (flen >= (int)sizeof(fbuf)) flen = sizeof(fbuf) - 1;
						memcpy(fbuf, msg_buf, flen);
						fbuf[flen] = '\0';
						char *fp = fbuf;
						while (*fp) {
							while (*fp == ' ') fp++;
							if (!*fp) break;
							char *end = fp;
							while (*end && *end != ' ') end++;
							char saved = *end;
							*end = '\0';
							if (!strcasecmp(fp, "SSID_TMF"))
								eflags |= FLEX_SYSEVENT_FLAG_SSID_TMF;
							else if (!strcasecmp(fp, "NID_TMF"))
								eflags |= FLEX_SYSEVENT_FLAG_NID_TMF;
							else if (!strcasecmp(fp, "CHAN_SETUP"))
								eflags |= FLEX_SYSEVENT_FLAG_CHAN_SETUP;
							else if (!strcasecmp(fp, "NID_FREQ"))
								eflags |= FLEX_SYSEVENT_FLAG_NID_FREQ;
							else if (!strcasecmp(fp, "SSID_FREQ"))
								eflags |= FLEX_SYSEVENT_FLAG_SSID_FREQ;
							else
								LOGP(DFLEX, LOGL_NOTICE,
								     "FIFO: unknown sysevent flag '%s'\n", fp);
							*end = saved;
							fp = end;
						}
					}
					idata = ((uint32_t)FLEX_INSTR_TYPE_SYS_EVENT & FLEX_INSTR_TYPE_MASK);
					idata |= (eflags & 0x7FF) << FLEX_SYSEVENT_DATA_SHIFT;
				}

				snprintf(instr_buf, sizeof(instr_buf), "%u", idata);
				memcpy(msg_buf, instr_buf, strlen(instr_buf) + 1);
				message_length = (int)strlen(instr_buf);
			}

			msg = flex_msg_create(flex, capcode, mtype, msg_buf, message_length);
			if (msg) {
				msg->speed = msg_speed;
				msg->modulation_type = msg_mod_type;
				msg->polarity = msg_polarity;
				msg->priority = msg_priority;
				msg->charset = msg_charset;
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
				/* Wire retransmission parameters */
				msg->retransmit_max = msg_retransmit;
				msg->retransmit_interval = msg_retransmit_interval;
				msg->send_delay = msg_send_delay;
				msg->short_msg_type = msg_short_type;
				msg->short_msg_source = msg_short_source;
				msg->short_msg_number = msg_short_number;
				msg->short_msg_r = msg_short_r;
				if (msg->send_delay > 0) {
					flex_t *fl = (flex_t *)sender_head;
					uint32_t current_abs = fl->sched_last_cycle * 128 + fl->sched_last_frame;
					msg->next_send_frame = (current_abs + msg->send_delay) % 1920;
					LOGP(DFLEX, LOGL_DEBUG,
					     "TX_DEFERRED: capcode=%" PRIu64 " send_delay=%d target_frame=%u\n",
					     capcode, msg_send_delay, msg->next_send_frame);
				}
				/* Check retransmit on non-R/N types */
				if (msg->retransmit_max > 0 &&
				    mtype != FLEX_MSG_TYPE_ALPHA &&
				    mtype != FLEX_MSG_TYPE_HEX &&
				    mtype != FLEX_MSG_TYPE_SECURE &&
				    mtype != FLEX_MSG_TYPE_NUMBERED_NUM &&
				    mtype != FLEX_MSG_TYPE_NUMBERED_SPECIAL) {
					LOGP(DFLEX, LOGL_NOTICE,
					     "FIFO: retransmit=%d ignored for type %s (no R/N flag support).\n",
					     msg->retransmit_max, flex_msg_type_name(mtype));
					msg->retransmit_max = 0;
				}
				LOGP(DFLEX, LOGL_INFO,
				     "FIFO: enqueued capcode=%" PRIu64 " addr=%s type=%s speed=%d/%s polarity=%s priority=%d charset=%s tempgroup=%d phase=%s len=%d flags=%s\n",
				     capcode,
				     flex_capcode_type_name(capcode),
				     flex_msg_type_name(mtype),
				     msg_speed,
				     flex_mod_name(msg_mod_type),
				     (msg_polarity < 0) ? "inverted" : "normal",
				     msg_priority,
				     msg_charset ? "kanji" : "ascii",
				     is_temp_group,
				     flex_phase_name(msg_phase),
				     message_length,
				     flex_flags_str(is_temp_group,
						   msg_mail_drop, msg_secure_encoding,
						   msg_retransmit));
			flex_log_payload(mtype, msg_buf, message_length);
				flex_log_type_options(mtype, msg_secure_encoding,
						     msg_secure_subtype,
						     msg_numbered_msgnum,
						     msg_blocking_length);
			}
		}
	}
}

/* Delayed quit: after one-shot TX completes, keep running for
 * a short time so the RX path can finish decoding the last frame
 * (loopback or off-air).  0 = not pending, >0 = quit time. */
double quit_after_time = 0;
#define QUIT_DELAY_SEC	0.25

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

	/* Delayed quit: wait for RX to finish processing */
	if (quit_after_time > 0) {
		double now = get_time();
		if (now >= quit_after_time) {
			quit = 1;
			quit_after_time = 0;
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
	static char short_desc[64], long_desc[64];
	snprintf(short_desc, sizeof(short_desc),
		 "short capcode (%" PRIu64 "-%" PRIu64 ")",
		 (uint64_t)FLEX_SHORT_ADDR_MIN, (uint64_t)FLEX_SHORT_ADDR_MAX);
	snprintf(long_desc, sizeof(long_desc),
		 "long capcode (%" PRIu64 "-%" PRIu64 ")",
		 (uint64_t)FLEX_LONG_ADDR_MIN, (uint64_t)FLEX_LONG_ADDR_MAX);
	const struct number_lengths number_lengths[] = {
		{ 7, short_desc },
		{ 10, long_desc },
		{ 0, NULL }
	};
	main_mobile_init("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz", number_lengths, NULL, flex_number_valid, NULL);

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

	/* scan mode: default to short numeric if user didn't specify -y */
	if (scan_to > scan_from && !msg_type_given)
		msg_type = FLEX_MSG_TYPE_SHORT;

	/* scan mode: imply --network for continuous operation */
	if (scan_to > scan_from)
		network_mode = 1;

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
	if (td_collapse >= 0 && (td_collapse < 5 || td_collapse > 7)) {
		fprintf(stderr, "Error: --td-collapse must be 5, 6, or 7.\n");
		return -EINVAL;
	}
	if (td_collapse >= 0 && td_collapse <= collapse) {
		fprintf(stderr, "Error: --td-collapse (%d) must be > --collapse (%d).\n",
			td_collapse, collapse);
		return -EINVAL;
	}
	if (roaming_enabled && !ssid && !nid) {
		fprintf(stderr, "Warning: --roaming requires --ssid1 and/or --ssid2 to be useful.\n");
	}
	if (biw_time_enabled == -1)
		biw_time_enabled = 0;

	/* When biw_time is enabled (explicitly or via network mode) and no
	 * timezone was set, auto-detect from system timezone. */
	if (biw_time_enabled > 0 && timezone_code < 0) {
		timezone_code = flex_tz_auto_detect();
		if (timezone_code >= 0) {
			char tzbuf[20];
			int tz_min = flex_tz_to_minutes((uint32_t)timezone_code);
			fprintf(stderr, "BIW time: auto-detected timezone zone=%d (%s)\n",
				timezone_code,
				flex_tz_format(tz_min, tzbuf, sizeof(tzbuf)));
		} else {
			fprintf(stderr, "Warning: BIW time enabled but could not auto-detect timezone.\n");
			fprintf(stderr, "         Time will be transmitted as UTC. Use --biw-time <offset> or --timezone <0-31>.\n");
		}
	}

	/* create pipe for message send */
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
		double frequency = atof(kanal[i]) * 1e6;
		rc = flex_create(kanal[i], frequency, dsp_device[i], use_sdr, dsp_samplerate, rx_gain, tx_gain, tx, deviation, polarity, msg_type, (scan_to > scan_from) ? "" : message, scan_from, scan_to, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback);
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
			f->roaming_active = roaming_enabled;
			f->num_transmissions = num_transmissions;
			f->td_collapse = td_collapse;
			f->chan_setup_enabled = chan_setup_enabled;
			f->hack_nonstandard_decoders = hack_nonstandard_decoders;

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

	/* One-shot auto-enqueue: if station_id is provided in
	 * "capcode,type,,message" format and we're not in network mode,
	 * parse it as a FIFO line to enqueue the message and trigger
	 * transmission immediately (no interactive console needed). */
	if (station_id[0] && !network_mode) {
		fifo_process_line(station_id, (int)strlen(station_id));
		{
			sender_t *s;
			for (s = sender_head; s; s = s->next) {
				flex_t *f = (flex_t *)s;
				if (f->tx && f->msg_list) {
					flex_trigger_ers(f);
					LOGP(DFLEX, LOGL_INFO,
					     "One-shot: auto-enqueued from CLI, TX triggered.\n");
				}
			}
		}
	}

	main_mobile_loop("flex", &quit, myhandler, station_id);

fail:
	/* pipe cleanup */
	if (msg_send_fd > 0)
		close(msg_send_fd);
	unlink(msg_send_path);

	/* destroy transceiver instance */
	while (sender_head)
		flex_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();

	options_free();

	return 0;
}
