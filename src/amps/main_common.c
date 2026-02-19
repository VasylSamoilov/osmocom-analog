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
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "../libsample/sample.h"
#include "../libmobile/main_mobile.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../liboptions/options.h"
#include "../libfm/fm.h"
#ifdef HAVE_SDR
#include "../libsdr/sdr_config.h"
#endif
#include "amps.h"
#include "dsp.h"
#include "frame.h"
#include "stations.h"
#include "main.h"

/* ============================================================
 * AMPS Control FIFO - Runtime Command Interface
 * ============================================================
 * Named pipe at /tmp/amps_control for sending orders to active calls.
 * Commands are case-insensitive, comma-separated.
 *
 * STANDARD ORDERS (TIA/EIA-553-A):
 *   alert,<number>              Ring mobile (Order 1, ORDQ=0)
 *   abbalert,<number>           Feature reminder tone (Order 1, ORDQ=1)
 *   release,<number>            Terminate call (Order 3)
 *   reorder,<number>            Fast busy tone (Order 4)
 *   mwi,<number>,<count>[,type] Voicemail indicator (Order 5)
 *                               count: 0-31, type: 0=voice,1=SMS,2=fax
 *   stopalert,<number>          Stop ringing (Order 6)
 *   digits,<number>             Request dialed digits (Order 8)
 *   intercept,<number>          Number cannot be completed (Order 9)
 *   maintenance,<number>        Silent test (Order 10)
 *   power,<number>,<level>      Adjust TX power (Order 11), level: 0-7
 *   retry,<number>,<ch1>[,...]  Redirect to other CC (Order 12)
 *   esn,<number>                Request ESN (Order 15)
 *   local,<number>,<code>       Vendor-specific (Order 30), code: 0-31
 *
 * EXTENDED FEATURES:
 *   flash,<number>,<msg>[,pi,si]  In-call Caller ID (Order 18)
 *   pci,<number>                  Query Protocol Capability
 *   audit,<number>                Audit mobile (Order 2)
 *   handoff,<number>[,channel]    Transfer to different VC
 *   rescan,<number>               Auto-calculated Directed Retry
 *   silentpage,<number>           Page + Maintenance (no ring)
 *
 * EXPERIMENTAL (non-standard):
 *   flashcri,<number>,<data>    Flash with Charging Rate (Order 18, ORDQ=1)
 *   flashtci,<number>,<data>    Flash with Total Charges (Order 18, ORDQ=2)
 *   alertcri,<number>,<data>    Alert with Charging Rate (Order 17, ORDQ=1)
 *   alerttci,<number>,<data>    Alert with Total Charges (Order 17, ORDQ=2)
 *
 * Examples:
 *   echo "alert,5551234567" > /tmp/amps_control
 *   echo "power,5551234567,3" > /tmp/amps_control
 *   echo "flash,5551234567,Hello World,0,3" > /tmp/amps_control
 * ============================================================ */
#define AMPS_CONTROL_FIFO "/tmp/amps_control"
static int control_fd = -1;

/* settings */
int num_chan_type = 0;
enum amps_chan_type chan_type[MAX_SENDER] = { CHAN_TYPE_CC_PC_VC };
const char *flip_polarity = "";
int ms_power = 4;
int dtx = 0;
int send_callerid = 0;

/* System parameters broadcast in overhead messages (per TIA/EIA-553-A):
 * dcc      - Digital Color Code (0-3): Identifies control channel, prevents interference from adjacent cells
 * scc      - SAT Color Code (0-2): Supervisory Audio Tone frequency on voice channels (5970/6000/6030 Hz)
 * sid      - System ID (15-bit): Unique identifier for cellular system, phone uses to detect home vs roaming
 * regh     - Registration Home: If 1, home phones must register
 * regr     - Registration Roaming: If 1, roaming phones must register
 * pureg    - Power-Up Registration: If 1, phones send registration immediately after power-on
 * pdreg    - Power-Down Registration: If 1, phones send registration just before power-off
 * locaid   - Location Area ID (0-4095, -1=disabled): Phone re-registers when entering new location area
 * regincr  - Registration Increment (seconds): Timer-based registration interval
 * bis      - Busy/Idle Status: If 1, phone checks B/I bit before TX. Set to 0 (osmocom can't respond fast enough)
 */
int dcc = 0, scc = 0, sid = 0, regh = 1, regr = 1, pureg = 1, pdreg = 1, locaid = -1, regincr = 300, bis = 0;

int tolerant = 0;
int vmac_enable = 0;
double vmac_level_low = 0.95;
double vmac_level_high = 1.01;
int network_timeout = 4;  /* seconds to wait for network PROCEEDING (1-5 per standard) */

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
	printf("        If 1, phone de-registers on every power down (default = '%d')\n", pdreg);
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
	printf("    --network-timeout <seconds>\n");
	printf("        Timeout waiting for network PROCEEDING before sending Reorder.\n");
	printf("        Valid range: 1-5 seconds (default = %d). Per TIA/EIA-553-A max is 5s.\n", network_timeout);
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
	printf("\nAMPS Control FIFO (/tmp/amps_control):\n");
	printf("    Flash With Info (In-Call Caller ID):\n");
	printf("        echo \"flash,<number>,<message>[,<pi>,<si>]\" > /tmp/amps_control\n");
	printf("        Example: echo \"flash,1234567890,John Doe\" > /tmp/amps_control\n");
	printf("        PI: 0=Allowed(Default), 1=Restricted, 2=NotAvail\n");
	printf("        SI: 0=Unscreened, 1=Passed, 2=Failed, 3=Network(Default)\n");
	printf("    Protocol Capability Indicator (PCI) Query:\n");
	printf("        echo \"pci,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"pci,1234567890\" > /tmp/amps_control\n");
	printf("        Response shows MSPC (protocol version) and MSCAP (analog capability)\n");
	printf("        NOTE: Most phones tested do not reply to PCI query\n");
	printf("    Audit Order:\n");
	printf("        echo \"audit,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"audit,1234567890\" > /tmp/amps_control\n");
	printf("    Alert Order (Ring Mobile Station):\n");
	printf("        echo \"alert,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"alert,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 1 (ORDQ=0) to ring the mobile station during active call\n");
	printf("    Abbreviated Alert Order (Feature Reminder Tone):\n");
	printf("        echo \"abbalert,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"abbalert,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 1 (ORDQ=1) for feature reminder tones\n");
	printf("    Release Order (Terminate Call):\n");
	printf("        echo \"release,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"release,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 3 to force call termination\n");
	printf("    Reorder Order (Fast Busy Tone):\n");
	printf("        echo \"reorder,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"reorder,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 4 - mobile plays fast busy (all circuits busy)\n");
	printf("    Message Waiting Indicator:\n");
	printf("        echo \"mwi,<number>,<count>[,<type>]\" > /tmp/amps_control\n");
	printf("        Example: echo \"mwi,1234567890,3,0\" > /tmp/amps_control\n");
	printf("        Sends Order 5 - count: 0-31, type: 0=voice, 1=SMS, 2=fax\n");
	printf("    Stop Alert Order (Stop Ringing):\n");
	printf("        echo \"stopalert,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"stopalert,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 6 - stops ringing but keeps channel\n");
	printf("    Intercept Order (Number Cannot Be Completed):\n");
	printf("        echo \"intercept,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"intercept,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 9 - mobile plays intercept tone (invalid number)\n");
	printf("    Send Called-Address Order (Request Dialed Digits):\n");
	printf("        echo \"digits,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"digits,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 8 - requests mobile to send dialed digits\n");
	printf("    Maintenance Order (Silent Test):\n");
	printf("        echo \"maintenance,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"maintenance,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 10 - mobile confirms with ST but doesn't ring\n");
	printf("        NOTE: Requires active call - use 'silentpage' for idle mobiles\n");
	printf("    Silent Page (Page + Maintenance):\n");
	printf("        echo \"silentpage,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"silentpage,1234567890\" > /tmp/amps_control\n");
	printf("        Pages idle mobile, assigns channel, sends Maintenance, releases\n");
	printf("        Tests mobile RF path without ringing - logs SUCCESS or FAILED\n");
	printf("    Change Power Order:\n");
	printf("        echo \"power,<number>,<level>\" > /tmp/amps_control\n");
	printf("        Example: echo \"power,1234567890,3\" > /tmp/amps_control\n");
	printf("        Sends Order 11 - level: 0-7 (0=max power, 7=min power)\n");
	printf("    Serial Number Request (ESN Verification):\n");
	printf("        echo \"esn,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"esn,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 15 - requests mobile to send ESN for verification\n");
	printf("    Disable DTMF Order:\n");
	printf("        echo \"disabledtmf,<number>\" > /tmp/amps_control\n");
	printf("        Example: echo \"disabledtmf,1234567890\" > /tmp/amps_control\n");
	printf("        Sends Order 22 - mutes mobile's DTMF tones until Called-Address sent\n");
	printf("        Use before Order 8 (digits) to prevent far-end hearing DTMF\n");
	printf("        DTMF auto-enables after mobile sends Called-Address response\n");
	printf("    Local Control Order (Vendor-Specific):\n");
	printf("        echo \"local,<number>,<code>\" > /tmp/amps_control\n");
	printf("        Example: echo \"local,1234567890,5\" > /tmp/amps_control\n");
	printf("        Sends Order 30 - code: 0-31 (manufacturer-defined)\n");
	printf("        WARNING: Highly vendor-specific, meaning depends on mobile manufacturer\n");
	printf("    Handoff (Voice Channel Transfer):\n");
	printf("        echo \"handoff,<number>,<channel>\" > /tmp/amps_control\n");
	printf("        Example: echo \"handoff,1234567890,335\" > /tmp/amps_control\n");
	printf("        Transfers active call to different voice channel\n");
	printf("        Mobile sends 50ms ST, tunes to new channel, continues call\n");
	/* CRI/TCI commands are experimental - value mapping is unknown.
	 * Data structures were provided by Andreas Eversberg but no documentation
	 * reference is available to verify how mobile stations interpret the BCD values.
	 *
	 * TESTING NOTE: Phones tested confirm the order (send Order Confirmation)
	 * but do not display anything. Some show a generic "Call Waiting" message
	 * as if it were a normal Flash With Info with no actual data displayed.
	 *
	 * Uncomment below if you want to experiment with these commands.
	 *
	printf("    Flash With CRI (Charging Rate Indication) - EXPERIMENTAL:\n");
	printf("        echo \"flashcri,<number>,<cri_data>\" > /tmp/amps_control\n");
	printf("        cri_data: E1,E2,E3,... (up to 8 elements, 4 digits each)\n");
	printf("        WARNING: Value mapping unknown, may not work with real mobiles\n");
	printf("    Flash With TCI (Total Charging Information) - EXPERIMENTAL:\n");
	printf("        echo \"flashtci,<number>,<tci_data>\" > /tmp/amps_control\n");
	printf("        tci_data: R1,R2,R3,R4 (up to 4 rows, 4 digits each)\n");
	printf("        WARNING: Value mapping unknown, may not work with real mobiles\n");
	printf("    Alert With CRI/TCI - EXPERIMENTAL:\n");
	printf("        echo \"alertcri,<number>,<cri_data>\" > /tmp/amps_control\n");
	printf("        echo \"alerttci,<number>,<tci_data>\" > /tmp/amps_control\n");
	printf("        WARNING: Value mapping unknown, may not work with real mobiles\n");
	*/
	main_mobile_print_station_id();
	main_mobile_print_hotkeys();
}

/* Handler for AMPS Control FIFO commands */
static void amps_myhandler(void)
{
	static char buffer[256];
	static int pos = 0;
	int rc, i, space;
	char *p, *cmd, *number, *message, *pi_str, *si_str;
	int pi = 0, si = 3;  /* defaults: Allowed, Network-provided */

	if (control_fd < 0)
		return;

	space = sizeof(buffer) - pos;
	rc = read(control_fd, buffer + pos, space);
	if (rc > 0) {
		pos += rc;
		if (pos == space) {
			fprintf(stderr, "Control buffer overflow!\n");
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

			p = buffer;
			cmd = strsep(&p, ",");
			if (!cmd) {
				fprintf(stderr, "Empty command\n");
				return;
			}

			if (strcasecmp(cmd, "flash") == 0) {
				/* Parse: flash,number,message[,pi,si] */
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
			} else if (strcasecmp(cmd, "pci") == 0) {
				/* Parse: pci,number */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: pci,<number>\n");
					return;
				}
				rc = amps_pci_query(number);
				if (rc < 0)
					fprintf(stderr, "PCI query failed: %d\n", rc);
			} else if (strcasecmp(cmd, "audit") == 0) {
				/* Parse: audit,number */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: audit,<number>\n");
					return;
				}
				rc = amps_audit_order(number);
				if (rc < 0)
					fprintf(stderr, "Audit order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "alert") == 0) {
				/* Parse: alert,number
				 * Send Alert Order (Order 1, ORDQ=0) to ring the mobile station
				 * Requirements: 3.1, 3.2, 3.4, 3.6
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: alert,<number>\n");
					return;
				}
				rc = amps_alert_order(number);
				if (rc < 0)
					fprintf(stderr, "Alert order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "release") == 0) {
				/* Parse: release,number
				 * Send Release Order (Order 3) to terminate the call
				 * Requirements: 4.3, 4.5
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: release,<number>\n");
					return;
				}
				rc = amps_release_order(number);
				if (rc < 0)
					fprintf(stderr, "Release order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "power") == 0) {
				/* Parse: power,number,level
				 * Send Change Power Order (Order 11) to adjust mobile TX power
				 * level: 0-7 (0=max power, 7=min power)
				 * Requirements: 12.3, 12.4, 12.5
				 */
				char *level_str;
				int level;
				number = strsep(&p, ",");
				level_str = strsep(&p, ",");
				if (!number || !level_str) {
					fprintf(stderr, "Usage: power,<number>,<level>\n");
					fprintf(stderr, "  level: 0-7 (0=max power, 7=min power)\n");
					return;
				}
				level = atoi(level_str);
				/* Clamp to valid range (Requirement 12.4) */
				if (level < 0) level = 0;
				if (level > 7) level = 7;
				rc = amps_change_power_order(number, level);
				if (rc < 0)
					fprintf(stderr, "Change Power order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "abbalert") == 0) {
				/* Parse: abbalert,number
				 * Send Abbreviated Alert Order (Order 1, ORDQ=1) for feature reminder tones
				 * Requirements: 3.5
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: abbalert,<number>\n");
					return;
				}
				rc = amps_abbreviated_alert(number);
				if (rc < 0)
					fprintf(stderr, "Abbreviated Alert order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "reorder") == 0) {
				/* Parse: reorder,number
				 * Send Reorder Order (Order 4, ORDQ=0) - fast busy tone
				 * Requirements: 5.1, 5.2, 5.3
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: reorder,<number>\n");
					return;
				}
				rc = amps_reorder(number);
				if (rc < 0)
					fprintf(stderr, "Reorder order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "mwi") == 0) {
				/* Parse: mwi,number,count[,type]
				 * Send Message Waiting Order (Order 5) - voicemail indicator
				 * count: 0-31 (31 = unknown count)
				 * type: 0=voice (default), 1=SMS, 2=fax
				 * Requirements: 6.1, 6.2, 6.3, 6.4, 6.5
				 */
				char *count_str, *type_str;
				int count, type = 0;
				number = strsep(&p, ",");
				count_str = strsep(&p, ",");
				type_str = strsep(&p, ",");
				if (!number || !count_str) {
					fprintf(stderr, "Usage: mwi,<number>,<count>[,<type>]\n");
					fprintf(stderr, "  count: 0-31 (31 = unknown count)\n");
					fprintf(stderr, "  type: 0=voice (default), 1=SMS, 2=fax\n");
					return;
				}
				count = atoi(count_str);
				if (type_str && type_str[0])
					type = atoi(type_str);
				rc = amps_mwi(number, count, type);
				if (rc < 0)
					fprintf(stderr, "Message Waiting order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "stopalert") == 0) {
				/* Parse: stopalert,number
				 * Send Stop Alert Order (Order 6, ORDQ=0) - stop ringing
				 * Requirements: 7.1, 7.2, 7.3, 7.4, 7.5
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: stopalert,<number>\n");
					return;
				}
				rc = amps_stopalert(number);
				if (rc < 0)
					fprintf(stderr, "Stop Alert order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "intercept") == 0) {
				/* Parse: intercept,number
				 * Send Intercept Order (Order 9, ORDQ=0) - number cannot be completed
				 * Mobile plays intercept tone (alternating high-low)
				 * This is a MANDATORY order - all AMPS phones must support it
				 * Requirements: 10.1, 10.2, 10.3
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: intercept,<number>\n");
					return;
				}
				rc = amps_intercept(number);
				if (rc < 0)
					fprintf(stderr, "Intercept order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "digits") == 0) {
				/* Parse: digits,number
				 * Send Called-Address Request (Order 8, ORDQ=0)
				 * Requests mobile to send dialed digits
				 * Requirements: 9.1, 9.2, 9.3
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: digits,<number>\n");
					return;
				}
				rc = amps_send_called_address(number);
				if (rc < 0)
					fprintf(stderr, "Send Called-Address order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "maintenance") == 0) {
				/* Parse: maintenance,number
				 * Send Maintenance Order (Order 10, ORDQ=0)
				 * Silent test - mobile confirms with ST but doesn't ring
				 * Requirements: 11.1, 11.2, 11.3, 11.4
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: maintenance,<number>\n");
					return;
				}
				rc = amps_maintenance(number);
				if (rc < 0)
					fprintf(stderr, "Maintenance order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "silentpage") == 0) {
				/* Parse: silentpage,number
				 * Silent Page - page mobile and send Maintenance instead of Alert
				 * This tests the mobile's RF path without ringing the phone.
				 * Flow: Page -> Assign Channel -> Maintenance -> Release
				 * Per TIA/EIA-553-A: Maintenance can only be sent on FVC,
				 * so we must establish a call first.
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: silentpage,<number>\n");
					return;
				}
				rc = amps_silent_page(number);
				if (rc < 0)
					fprintf(stderr, "Silent Page failed: %d\n", rc);
			} else if (strcasecmp(cmd, "flashcri") == 0) {
				/* Parse: flashcri,number,cri_data
				 * Send Flash With CRI (Order 18, ORDQ=1) - Charging Rate Indication
				 * cri_data: "E1,E2,E3,..." (up to 8 elements, 4 digits each)
				 * Example: flashcri,0570000000,0025,0100
				 *
				 * WARNING: EXPERIMENTAL - CRI is NOT defined in TIA/EIA-553-A.
				 * Data structures from Andreas Eversberg, value mapping unknown.
				 */
				char *cri_data;
				number = strsep(&p, ",");
				cri_data = p;  /* Rest of string is CRI data */
				if (!number || !cri_data || !cri_data[0]) {
					fprintf(stderr, "Usage: flashcri,<number>,<cri_data>\n");
					fprintf(stderr, "  cri_data: E1,E2,E3,... (up to 8 elements, 4 digits each)\n");
					fprintf(stderr, "  Example: flashcri,0570000000,0025,0100\n");
					return;
				}
				rc = amps_flash_with_cri(number, cri_data);
				if (rc < 0)
					fprintf(stderr, "Flash With CRI failed: %d\n", rc);
			} else if (strcasecmp(cmd, "flashtci") == 0) {
				/* Parse: flashtci,number,tci_data
				 * Send Flash With TCI (Order 18, ORDQ=2) - Total Charging Information
				 * tci_data: "R1,R2,R3,R4" (up to 4 rows, 4 digits each)
				 * Example: flashtci,0570000000,0123,4567
				 *
				 * WARNING: EXPERIMENTAL - TCI is NOT defined in TIA/EIA-553-A.
				 * Data structures from Andreas Eversberg, value mapping unknown.
				 */
				char *tci_data;
				number = strsep(&p, ",");
				tci_data = p;  /* Rest of string is TCI data */
				if (!number || !tci_data || !tci_data[0]) {
					fprintf(stderr, "Usage: flashtci,<number>,<tci_data>\n");
					fprintf(stderr, "  tci_data: R1,R2,R3,R4 (up to 4 rows, 4 digits each)\n");
					fprintf(stderr, "  Example: flashtci,0570000000,0123,4567\n");
					return;
				}
				rc = amps_flash_with_tci(number, tci_data);
				if (rc < 0)
					fprintf(stderr, "Flash With TCI failed: %d\n", rc);
			} else if (strcasecmp(cmd, "alertcri") == 0) {
				/* Parse: alertcri,number,cri_data
				 * Send Alert With CRI (Order 17, ORDQ=1) - Alert with Charging Rate
				 * cri_data: "E1,E2,E3,..." (up to 8 elements, 4 digits each)
				 * Example: alertcri,0570000000,0025,0100
				 *
				 * WARNING: EXPERIMENTAL - CRI is NOT defined in TIA/EIA-553-A.
				 * Data structures from Andreas Eversberg, value mapping unknown.
				 */
				char *cri_data;
				number = strsep(&p, ",");
				cri_data = p;  /* Rest of string is CRI data */
				if (!number || !cri_data || !cri_data[0]) {
					fprintf(stderr, "Usage: alertcri,<number>,<cri_data>\n");
					fprintf(stderr, "  cri_data: E1,E2,E3,... (up to 8 elements, 4 digits each)\n");
					fprintf(stderr, "  Example: alertcri,0570000000,0025,0100\n");
					return;
				}
				rc = amps_alert_with_cri(number, cri_data);
				if (rc < 0)
					fprintf(stderr, "Alert With CRI failed: %d\n", rc);
			} else if (strcasecmp(cmd, "alerttci") == 0) {
				/* Parse: alerttci,number,tci_data
				 * Send Alert With TCI (Order 17, ORDQ=2) - Alert with Total Charges
				 * tci_data: "R1,R2,R3,R4" (up to 4 rows, 4 digits each)
				 * Example: alerttci,0570000000,0123,4567
				 *
				 * WARNING: EXPERIMENTAL - TCI is NOT defined in TIA/EIA-553-A.
				 * Data structures from Andreas Eversberg, value mapping unknown.
				 */
				char *tci_data;
				number = strsep(&p, ",");
				tci_data = p;  /* Rest of string is TCI data */
				if (!number || !tci_data || !tci_data[0]) {
					fprintf(stderr, "Usage: alerttci,<number>,<tci_data>\n");
					fprintf(stderr, "  tci_data: R1,R2,R3,R4 (up to 4 rows, 4 digits each)\n");
					fprintf(stderr, "  Example: alerttci,0570000000,0123,4567\n");
					return;
				}
				rc = amps_alert_with_tci(number, tci_data);
				if (rc < 0)
					fprintf(stderr, "Alert With TCI failed: %d\n", rc);
			} else if (strcasecmp(cmd, "esn") == 0) {
				/* Parse: esn,number
				 * Send Serial Number Request (Order 15, ORDQ=0)
				 * Requests mobile to send its ESN for verification
				 * Requirements: 15.1, 15.2, 15.3, 15.4
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: esn,<number>\n");
					return;
				}
				rc = amps_serial_number_request(number);
				if (rc < 0)
					fprintf(stderr, "Serial Number Request failed: %d\n", rc);
			} else if (strcasecmp(cmd, "disabledtmf") == 0) {
				/* Parse: disabledtmf,number
				 * Send Disable DTMF Order (Order 22, ORDQ=0)
				 * Mutes mobile's DTMF tone generator until Called-Address sent
				 *
				 * Per TIA/EIA-553-A Section 2.6.4.4:
				 * - Mobile confirms with digital Order Confirmation
				 * - DTMF stays disabled until Called-Address Message transmitted
				 * - No explicit "Enable DTMF" order - re-enable is automatic
				 *
				 * Typical workflow:
				 * 1. Send disabledtmf -> Mobile mutes DTMF
				 * 2. User dials digits (no tones heard by far-end)
				 * 3. Send digits (Order 8) -> Mobile sends Called-Address
				 * 4. DTMF auto-enables after Called-Address transmission
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: disabledtmf,<number>\n");
					return;
				}
				rc = amps_disable_dtmf(number);
				if (rc < 0)
					fprintf(stderr, "Disable DTMF order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "local") == 0) {
				/* Parse: local,number,code
				 * Send Local Control Order (Order 30, ORDQ=0)
				 * Vendor-specific local control action
				 * code: 0-31 (manufacturer-defined)
				 * Requirements: 19.1, 19.2, 19.3, 19.4
				 *
				 * WARNING: This is highly vendor-specific. The meaning of each
				 * code depends on the mobile station manufacturer.
				 */
				char *code_str;
				int code;
				number = strsep(&p, ",");
				code_str = strsep(&p, ",");
				if (!number || !code_str) {
					fprintf(stderr, "Usage: local,<number>,<code>\n");
					fprintf(stderr, "  code: 0-31 (vendor-specific)\n");
					return;
				}
				code = atoi(code_str);
				rc = amps_local_control(number, code);
				if (rc < 0)
					fprintf(stderr, "Local Control order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "handoff") == 0) {
				/* Parse: handoff,number[,channel]
				 * Send Handoff message on FVC
				 * Transfers active call to different voice channel
				 *
				 * Per TIA/EIA-553-A Section 2.6.4.4:
				 * - Mobile sends 50ms ST to confirm
				 * - Mobile tunes to new channel
				 * - Call continues on new channel
				 *
				 * If channel is omitted, auto-selects a free VC in same system
				 *
				 * Example: handoff,5203495579        (auto-select)
				 * Example: handoff,5203495579,335   (specific channel)
				 */
				char *chan_str;
				int channel = 0;  /* 0 = auto-select */
				
				number = strsep(&p, ",");
				chan_str = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: handoff,<number>[,<channel>]\n");
					fprintf(stderr, "  channel: Target voice channel (optional, auto-selects if omitted)\n");
					return;
				}
				if (chan_str && chan_str[0])
					channel = atoi(chan_str);
				rc = amps_handoff(number, channel);
				if (rc < 0)
					fprintf(stderr, "Handoff failed: %d\n", rc);
			} else if (strcasecmp(cmd, "retry") == 0) {
				/* Parse: retry,number,chan1[,chan2,chan3,chan4,chan5,chan6][,last]
				 * Send Directed Retry Order (Order 12) on FOCC
				 * Redirects mobile to try access on different control channels
				 * chan1-chan6: Channel positions (1-127, up to 6 channels)
				 * last: Optional "last" keyword to indicate last try (ORDQ=1)
				 *
				 * Per TIA/EIA-553-A Section 3.7.1.1:
				 * - ORDQ=0: Not last try (mobile should try again if this fails)
				 * - ORDQ=1: Last try (mobile should give up if this fails)
				 *
				 * Example: retry,5203495579,10,20,30
				 * Example: retry,5203495579,10,20,30,last
				 */
				char *chan_str;
				int channels[6];
				int num_channels = 0;
				int last_try = 0;
				
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: retry,<number>,<chan1>[,<chan2>,...][,last]\n");
					fprintf(stderr, "  chan1-chan6: Channel positions (1-127)\n");
					fprintf(stderr, "  last: Optional - indicates last try (ORDQ=1)\n");
					return;
				}
				
				/* Parse channel positions */
				while ((chan_str = strsep(&p, ",")) != NULL && num_channels < 6) {
					if (strcasecmp(chan_str, "last") == 0) {
						last_try = 1;
						break;
					}
					int chan = atoi(chan_str);
					if (chan < 1 || chan > 127) {
						fprintf(stderr, "Invalid channel position %d (must be 1-127)\n", chan);
						return;
					}
					channels[num_channels++] = chan;
				}
				
				/* Check for "last" keyword after channels */
				if (chan_str && strcasecmp(chan_str, "last") == 0) {
					last_try = 1;
				}
				
				if (num_channels < 1) {
					fprintf(stderr, "Usage: retry,<number>,<chan1>[,<chan2>,...][,last]\n");
					fprintf(stderr, "  At least one channel position required\n");
					return;
				}
				
				rc = amps_directed_retry(number, channels, num_channels, last_try);
				if (rc < 0)
					fprintf(stderr, "Directed Retry order failed: %d\n", rc);
			} else if (strcasecmp(cmd, "rescan") == 0) {
				/* Parse: rescan,number
				 * Send Directed Retry with automatic CHANPOS calculation
				 * Determines current system (A/B), calculates valid control
				 * channel positions, and sends Directed Retry.
				 *
				 * This is a convenience command that handles the CHANPOS
				 * calculation automatically based on the current control
				 * channel and serving-system status.
				 */
				number = strsep(&p, ",");
				if (!number) {
					fprintf(stderr, "Usage: rescan,<number>\n");
					return;
				}
				rc = amps_rescan(number);
				if (rc < 0)
					fprintf(stderr, "Rescan order failed: %d\n", rc);
			} else {
				fprintf(stderr, "Unknown command '%s', use 'flash', 'pci', 'audit', 'alert', 'abbalert', 'release', 'reorder', 'mwi', 'stopalert', 'intercept', 'power', 'esn', 'local', 'handoff', 'retry' or 'rescan'\n", cmd);
			}
		}
	}
}

#define OPT_PREFIX 256
#define OPT_NETWORK_TIMEOUT 257

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
	option_add(OPT_NETWORK_TIMEOUT, "network-timeout", 1);
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
	case OPT_NETWORK_TIMEOUT:
		network_timeout = atoi(argv[argi]);
		if (network_timeout < 1) network_timeout = 1;
		if (network_timeout > 5) network_timeout = 5;
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}

extern const struct number_lengths number_lengths[];

#ifdef HAVE_SDR
/**
 * Calculate required SDR bandwidth for given channels.
 * 
 * This replicates the range calculation from sdr.c to ensure rate selection
 * picks a sample rate that will actually work. The calculation accounts for:
 * 1. DC avoidance - center frequency is moved between channels
 * 2. Asymmetric spectrum - max(low_side, high_side) * 2
 * 
 * @param kanal       Array of channel number strings
 * @param num_kanal   Number of channels
 * @param channel_bw  Per-channel bandwidth (2 * (deviation + modulation))
 * @return Required bandwidth in Hz, or 0 on error
 */
static double amps_calculate_sdr_bandwidth(const char **kanal, int num_kanal, double channel_bw)
{
	double *frequencies;
	double low_freq = 0, high_freq = 0, center_freq;
	double low_side, high_side, range;
	int i;

	if (num_kanal <= 0)
		return channel_bw;

	/* Get frequencies for all channels */
	frequencies = calloc(num_kanal, sizeof(double));
	if (!frequencies)
		return 0;

	for (i = 0; i < num_kanal; i++) {
		frequencies[i] = amps_channel2freq(atoi(kanal[i]), 1); /* RX freq */
		if (frequencies[i] <= 0) {
			free(frequencies);
			return 0;
		}
		if (i == 0 || frequencies[i] < low_freq)
			low_freq = frequencies[i];
		if (i == 0 || frequencies[i] > high_freq)
			high_freq = frequencies[i];
	}

	/* Initial center = midpoint */
	center_freq = (high_freq + low_freq) / 2.0;

	/* DC avoidance: move center between two closest channels */
	if (num_kanal == 1) {
		/* Single channel: shift center down by bandwidth to avoid DC */
		center_freq -= channel_bw;
		/* low_freq stays at original, so low_side becomes 0 */
	} else {
		/* Find two channels closest to center, one on each side */
		double low_dist = 0, high_dist = 0, dist;
		int low_c = -1, high_c = -1;
		
		for (i = 0; i < num_kanal; i++) {
			dist = fabs(center_freq - frequencies[i]);
			if (round(frequencies[i]) >= round(center_freq)) {
				if (high_c < 0 || dist < high_dist) {
					high_dist = dist;
					high_c = i;
				}
			} else {
				if (low_c < 0 || dist < low_dist) {
					low_dist = dist;
					low_c = i;
				}
			}
		}
		
		/* New center = midpoint of the two channels aside old center */
		if (low_c >= 0 && high_c >= 0) {
			center_freq = (frequencies[low_c] + frequencies[high_c]) / 2.0;
		}
	}

	/* Calculate range: max(low_side, high_side) * 2 */
	low_side = (center_freq - low_freq) + channel_bw / 2.0;
	high_side = (high_freq - center_freq) + channel_bw / 2.0;
	range = ((low_side > high_side) ? low_side : high_side) * 2.0;

	free(frequencies);
	return range;
}
#endif

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

#ifdef HAVE_SDR
	/* Early SDR rate selection - BEFORE creating senders */
	if (use_sdr) {
		double max_dev, max_mod, channel_bw, required_bw;

		/* Get protocol-specific bandwidth parameters */
		amps_get_bandwidth(tacs, &max_dev, &max_mod);
		channel_bw = 2.0 * (max_dev + max_mod);

		/* Calculate actual required bandwidth using same algorithm as sdr.c */
		required_bw = amps_calculate_sdr_bandwidth((const char **)kanal, num_kanal, channel_bw);
		if (required_bw <= 0) {
			fprintf(stderr, "Failed to calculate SDR bandwidth!\n");
			goto fail;
		}

		/* Select optimal SDR rate (may update dsp_samplerate) */
		rc = sdr_select_rate(required_bw, &dsp_samplerate);
		if (rc < 0)
			goto fail;
	}
#endif

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

	/* Create AMPS Control FIFO */
	unlink(AMPS_CONTROL_FIFO);
	rc = mkfifo(AMPS_CONTROL_FIFO, 0666);
	if (rc < 0) {
		fprintf(stderr, "Failed to create Control FIFO '%s'!\n", AMPS_CONTROL_FIFO);
		goto fail;
	} else {
		control_fd = open(AMPS_CONTROL_FIFO, O_RDONLY | O_NONBLOCK);
		if (control_fd < 0) {
			fprintf(stderr, "Failed to open Control FIFO '%s'!\n", AMPS_CONTROL_FIFO);
			goto fail;
		}
	}

	main_mobile_loop(name, &quit, amps_myhandler, station_id);

fail:
	/* FIFO cleanup */
	if (control_fd > 0)
		close(control_fd);
	unlink(AMPS_CONTROL_FIFO);

	/* destroy transceiver instance */
	while (sender_head)
		amps_destroy(sender_head);

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
