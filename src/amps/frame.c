/* AMPS frame transcoding
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

#define CHAN amps->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <inttypes.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/get_time.h"
#include "amps.h"
#include "dsp.h"
#include "frame.h"
#include "main.h"

/* uncomment this to debug bits */
//#define BIT_DEBUGGING

/* uncomment this to debug all messages (control filler / global action messages) */
//#define DEBUG_ALL_MESSAGES

/*
 * parity
 */

static uint64_t cut_bits[37] = {
	0x0,
	0x1, 0x3, 0x7, 0xf, 0x1f, 0x3f, 0x7f, 0xff,
	0x1ff, 0x3ff, 0x7ff, 0xfff, 0x1fff, 0x3fff, 0x7fff, 0xffff,
	0x1ffff, 0x3ffff, 0x7ffff, 0xfffff, 0x1fffff, 0x3fffff, 0x7fffff, 0xffffff,
	0x1ffffff, 0x3ffffff, 0x7ffffff, 0xfffffff, 0x1fffffff, 0x3fffffff, 0x7fffffff, 0xffffffff,
	0x1ffffffff, 0x3ffffffff, 0x7ffffffff, 0xfffffffff,
};

static char gp[12] = { 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1 };

/* do BCH(length+12,length,5) encoding:
 * given data and length, return 12 bits redundancy
 * all arrays are MSB first.
 */
static const char *encode_bch(const char *data, int length)
{
	static char redun[13];
	int i, j, feedback;

	for (i = 0; i < 12; i++)
		redun[i] = 0;

	for (i = 0; i < length; i++) {
		feedback = (data[i] & 1) ^ redun[0];
		if (feedback) {
			for (j = 11; j > 0; j--) {
				if (gp[11 - j])
					redun[11 - j] = redun[12 - j] ^ feedback;
				else
					redun[11 - j] = redun[12 - j];
			}
			redun[11] = gp[11];
		} else {
			for (j = 11; j > 0; j--)
				redun[11 - j] = redun[12 - j];
			redun[11] = 0;
		}
	}
	for (i = 0; i < 12; i++)
		redun[i] += '0';
	redun[12] = '\0';

	return redun;
}

/* same as above, but with binary data (without parity space holder) */
static uint16_t encode_bch_binary(uint64_t value, int length)
{
	char data[length + 1];
	const char *redun;
	uint16_t p = 0;
	int i;

	for (i = 0; i < length; i++)
		data[i] = '0' + ((value >> (length - 1 - i)) & 1);
	data[i] = '\0';

	redun = encode_bch(data, length);

	for (i = 0; i < 12; i++)
		p = (p << 1) | ((*redun++) & 1);

	return p;
}

/*
 * helper
 */

/* convert amps digits to number digits */
static char digit2number[16] = {
	'\0',
	'1',
	'2',
	'3',
	'4',
	'5',
	'6',
	'7',
	'8',
	'9',
	'0',
	'*',
	'#',
	'+',
	'?',
	'?',
};


/*
 * Word definitions
 */

struct def_ie {
	const char		*name;
	int			bits;
	enum amps_ie		ie;
};

struct def_word {
	const char		*name;
	struct def_ie		ie[];
};

struct def_message_set {
	const char		*name;
	int			num_bits;
	struct def_word		*word[];
};


/* FOCC - Mobile Station Control Message */

static struct def_word word1_abbreviated_address_word = {
	"Word 1 - Abbreviated Address Word",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "MIN1",			24,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_extended_address_word_a = {
	"Word 2 - Extended Address Word (SCC == 11)",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "MIN2",			10,	0 },
		{ "EF",				1,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_extended_address_word_b = {
	"Word 2 - Extended Address Word (SCC != 11)",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "MIN2",			10,	0 },
		{ "VMAC",			3,	0 },
		{ "CHAN",			11,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_first_analog_channel_assignment_word = {
	"Word 3 - First Analog Channel Assignment Word",
	{
		{ "T1T2",			2,	0 },
		{ "PVI",			1,	0 },
		{ "MEM",			1,	0 },
		{ "DTX Support",		2,	0 },
		{ "RSVD",			6,	0 },
		{ "SCC",			2,	0 },
		{ "VMAC",			3,	0 },
		{ "CHAN",			11,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_first_digital_channel_assignment_word = {
	"Word 3 - First Digital Channel Assignment Word",
	{
		{ "T1T2",			2,	0 },
		{ "PVI",			1,	0 },
		{ "MEM",			1,	0 },
		{ "DVCC",			8,	0 },
		{ "PM",				1,	0 },
		{ "DMAC",			4,	0 },
		{ "CHAN",			11,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_first_directed_retry_word = {
	"Word 3 - First Directed-Retry Word",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "CHANPOS",			7,	0 },
		{ "CHANPOS",			7,	0 },
		{ "CHANPOS",			7,	0 },
		{ "RSVD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_base_station_challenge_order_confirmation_word = {
	"Word 3 - Base Station Challenge Order Confirmation Word",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "AUTHBS",			18,	0 },
		{ "RSVD",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_unique_challenge_order_word = {
	"Word 3 - Unique Challenge Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "RANDU",			24,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_first_ssd_update_order_word = {
	"Word 3 - First SSD Update Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "RANDSSD_1",			24,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_second_directed_retry_word = {
	"Word 4 - Second Directed-Retry Word",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "CHANPOS",			7,	0 },
		{ "CHANPOS",			7,	0 },
		{ "CHANPOS",			7,	0 },
		{ "RSVD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_second_ssd_update_order_word = {
	"Word 4 - Second SSD Update Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "RANDSSD_2",			24,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word5_third_ssd_update_order_word = {
	"Word 5 - Third SSD Update Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "RSVD",			12,	0 },
		{ "RANDSSD_3",			8,	0 },
		{ "RSVD",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* FOCC - System Parameter Overhead Message */

static struct def_word amps_word1_system_parameter_overhead = {
	"Word 1 - System Parameter Overhead",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "SID1",			14,	0 },
		{ "EP",				1,	0 },
		{ "AUTH",			1,	0 },
		{ "PCI",			1,	0 },
		{ "NAWC",			4,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word tacs_word1_system_parameter_overhead = {
	"Word 1 - System Parameter Overhead",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "AID1",			14,	0 },
		{ "EP",				1,	0 },
		{ "AUTH",			1,	0 },
		{ "PCI",			1,	0 },
		{ "NAWC",			4,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_system_parameter_overhead = {
	"Word 2 - System Parameter Overhead",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "S",				1,	0 },
		{ "E",				1,	0 },
		{ "REGH",			1,	0 },
		{ "REGR",			1,	0 },
		{ "DTX Support",		2,	0 },
		{ "N-1",			5,	0 },
		{ "RCF",			1,	0 },
		{ "CPA",			1,	0 },
		{ "CMAX-1",			7,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* FOCC - Global action Overhead Message */

static struct def_word rescan_global_action = {
	"Rescan Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "RSVD",			16,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word registration_increment_global_action = {
	"Registration Increment Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "REGINCR",			12,	0 },
		{ "RSVD",			4,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word location_area_global_action = {
	"Location Area Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "PUREG",			1,	0 },
		{ "PDREG",			1,	0 },
		{ "LREG",			1,	0 },
		{ "RSVD",			1,	0 },
		{ "LOCAID",			12,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word new_access_channel_set_global_action = {
	"New Access Channel Set Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "NEWACC",			11,	0 },
		{ "RSVD",			5,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word overload_control_global_action = {
	"Overload Control Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "OLC 0",			1,	0 },
		{ "OLC 1",			1,	0 },
		{ "OLC 2",			1,	0 },
		{ "OLC 3",			1,	0 },
		{ "OLC 4",			1,	0 },
		{ "OLC 5",			1,	0 },
		{ "OLC 6",			1,	0 },
		{ "OLC 7",			1,	0 },
		{ "OLC 8",			1,	0 },
		{ "OLC 9",			1,	0 },
		{ "OLC 10",			1,	0 },
		{ "OLC 11",			1,	0 },
		{ "OLC 12",			1,	0 },
		{ "OLC 13",			1,	0 },
		{ "OLC 14",			1,	0 },
		{ "OLC 15",			1,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word access_type_parameters_global_action = {
	"Access Type Parameters Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "BIS",			1,	0 },
		{ "PCI_HOME",			1,	0 },
		{ "PCI_ROAM",			1,	0 },
		{ "BSPC",			4,	0 },
		{ "BSCAP",			3,	0 },
		{ "RSVD",			6,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word access_attempt_parameters_global_action = {
	"Access Attempt Parameters Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "MAXBUSY-PGR",		4,	0 },
		{ "MAXSZTR-PGR",		4,	0 },
		{ "MAXBUSY-OTHER",		4,	0 },
		{ "MAXSZTR-OTHER",		4,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word random_challenge_a_global_action = {
	"Random Challenge A Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "RAND1_A",			16,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word random_challenge_b_global_action = {
	"Random Challenge B Global Action Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "RAND1_B",			16,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word local_control_1 = {
	"Local Control 1 Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "LOCAL CONTROL",		16,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word local_control_2 = {
	"Local Control 2 Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "ACT",			4,	0 },
		{ "LOCAL CONTROL",		16,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* FOCC - Registration ID Message */

static struct def_word registration_id = {
	"Registration ID Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "REGID",			20,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* FOCC - Control Filler Message */

static struct def_word control_filler = {
	"Control-Filler Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "010111",			6,	0 },
		{ "CMAC",			3,	0 },
		{ "SDCC1",			2,	0 },
		{ "11",				2,	0 },
		{ "SDCC2",			2,	0 },
		{ "1",				1,	0 },
		{ "WFOM",			1,	0 },
		{ "1111",			4,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* FOCC - Control Channel Information Message */

static struct def_word control_channel_information = {
	"Control Channel Information Message",
	{
		{ "T1T2",			2,	0 },
		{ "DCC",			2,	0 },
		{ "CHAN",			11,	0 },
		{ "Async Data",			1,	0 },
		{ "G3 Fax",			1,	0 },
		{ "Data Privacy",		1,	0 },
		{ "HDVCC",			4,	0 },
		{ "Hyperband",			2,	0 },
		{ "END",			1,	0 },
		{ "OHD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_message_set focc_words = {
	"FOCC Messages", 40,
	{
		&word1_abbreviated_address_word,
		&word2_extended_address_word_a,
		&word2_extended_address_word_b,
		&word3_first_analog_channel_assignment_word,
		&word3_first_digital_channel_assignment_word,
		&word3_first_directed_retry_word,
		&word3_base_station_challenge_order_confirmation_word,
		&word3_unique_challenge_order_word,
		&word3_first_ssd_update_order_word,
		&word4_second_directed_retry_word,
		&word4_second_ssd_update_order_word,
		&word5_third_ssd_update_order_word,

		&amps_word1_system_parameter_overhead,
		&tacs_word1_system_parameter_overhead,
		&word2_system_parameter_overhead,
		&rescan_global_action,
		&registration_increment_global_action,
		&location_area_global_action,
		&new_access_channel_set_global_action,
		&overload_control_global_action,
		&access_type_parameters_global_action,
		&access_attempt_parameters_global_action,
		&random_challenge_a_global_action,
		&random_challenge_b_global_action,
		&local_control_1,
		&local_control_2,
		&registration_id,
		&control_filler,
		&control_channel_information,
		NULL
	}
};

/* RECC - Words */

static struct def_word abbreviated_address_word = {
	"Word A - Abbreviated Address Word",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "T",				1,	0 },
		{ "S",				1,	0 },
		{ "E",				1,	0 },
		{ "ER",				1,	0 },
		{ "SCM",			4,	0 },
		{ "MIN1",			24,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word extended_address_word = {
	"Word B - Extended Address Word",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "LT",				1,	0 },
		{ "EP",				1,	0 },
		{ "SCM",			1,	0 },
		{ "MPCI",			2,	0 },
		{ "SDCC1",			2,	0 },
		{ "SDCC2",			2,	0 },
		{ "MIN2",			10,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word serial_number_word = {
	"Word C - Serial Number Word",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "ESN",			32,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word authentication_word = {
	"Word C - Authentication Word",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "COUNT",			6,	0 },
		{ "RANDC",			8,	0 },
		{ "AUTHR",			18,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word unique_challenge_order_confirmation_word = {
	"Word C - Unique Challenge Order Confirmation Word",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "RSVD",			14,	0 },
		{ "AUTHU",			18,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word base_station_challenge_word = {
	"Word C - Base Station Challenge Word",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "RANDBS",			32,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word pci_report_registration_word = {
	"Word C - PCI Report/Registration Word",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "MSPC",			4,	0 },
		{ "MSCAP",			3,	0 },
		{ "RSVD",			25,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word order_confirmation_rvc_word = {
	"Word 1 - Order Confirmation (RVC)",
	{
		{ "F",                1,    0 },
		{ "NAWC",             2,    0 },
		{ "T",                1,    0 },
		{ "LOCAL/MSG TYPE",   5,    AMPS_IE_LOCAL_MSG_TYPE }, /* Fix name to match TIA */
		{ "ORDQ",             3,    AMPS_IE_ORDQ },
		{ "ORDER",            5,    AMPS_IE_ORDER },
		{ "MSPC",             4,    AMPS_IE_MSPC },
		{ "MSCAP",            3,    AMPS_IE_MSCAP },
		{ "RSVD",             12,   0 },
		{ "P",                12,   0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word first_word_of_the_called_address = {
	"Word D - First Word of the Called-Address (Origination - Voice Service)",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "DIGIT 1",			4,	0 },
		{ "DIGIT 2",			4,	0 },
		{ "DIGIT 3",			4,	0 },
		{ "DIGIT 4",			4,	0 },
		{ "DIGIT 5",			4,	0 },
		{ "DIGIT 6",			4,	0 },
		{ "DIGIT 7",			4,	0 },
		{ "DIGIT 8",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word service_code_word = {
	"Word D - Service Code Word (Origination with Service and Page Response with 2 Service)",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "Service Code",		4,	0 },
		{ "PM_D",			3,	0 },
		{ "SAP",			1,	0 },
		{ "Acked Data",			1,	0 },
		{ "CRC",			2,	0 },
		{ "Data Part",			3,	0 },
		{ "RLP",			2,	0 },
		{ "RSVD",			16,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word second_word_of_the_called_address = {
	"Word E - Second Word of the Called-Address (Origination - Voice Service)",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "DIGIT 9",			4,	0 },
		{ "DIGIT 10",			4,	0 },
		{ "DIGIT 11",			4,	0 },
		{ "DIGIT 12",			4,	0 },
		{ "DIGIT 13",			4,	0 },
		{ "DIGIT 14",			4,	0 },
		{ "DIGIT 15",			4,	0 },
		{ "DIGIT 16",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word third_word_of_the_called_address = {
	"Word F - Third Word of the Called-Address (Origination - Voice Service)",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "DIGIT 17",			4,	0 },
		{ "DIGIT 18",			4,	0 },
		{ "DIGIT 19",			4,	0 },
		{ "DIGIT 20",			4,	0 },
		{ "DIGIT 21",			4,	0 },
		{ "DIGIT 22",			4,	0 },
		{ "DIGIT 23",			4,	0 },
		{ "DIGIT 24",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word fourth_word_of_the_called_address = {
	"Word G - Fourth Word of the Called-Address (Origination - Voice Service)",
	{
		{ "F",				1,	0 },
		{ "NAWC",			3,	0 },
		{ "DIGIT 25",			4,	0 },
		{ "DIGIT 26",			4,	0 },
		{ "DIGIT 27",			4,	0 },
		{ "DIGIT 28",			4,	0 },
		{ "DIGIT 29",			4,	0 },
		{ "DIGIT 30",			4,	0 },
		{ "DIGIT 31",			4,	0 },
		{ "DIGIT 32",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_message_set recc_words = {
	"RECC Words", 48,
	{
		&abbreviated_address_word,
		&extended_address_word,
		&serial_number_word,
		&authentication_word,
		&unique_challenge_order_confirmation_word,
		&base_station_challenge_word,
		&pci_report_registration_word,
		&first_word_of_the_called_address,
		&service_code_word,
		&second_word_of_the_called_address,
		&third_word_of_the_called_address,
		&fourth_word_of_the_called_address,
		NULL
	}
};

/* FVC - Mobile Station Control Message */

static struct def_word mobile_station_control_message_word1_a = {
	"Mobile Station Control Message Word 1 (SCC == 11)",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "PSCC",			2,	0 },
		{ "EF",				1,	0 },
		{ "DVCC",			8,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word mobile_station_control_message_word1_b = {
	"Mobile Station Control Message Word 1 (SCC != 11)",
	{
		{ "T1T2",			2,	0 },
		{ "SCC",			2,	0 },
		{ "PSCC",			2,	0 },
		{ "EF",				1,	0 },
		{ "RSVD",			4,	0 },
		{ "DTX",			1,	0 },
		{ "PVI",			1,	0 },
		{ "MEM",			1,	0 },
		{ "VMAC",			3,	0 },
		{ "CHAN",			11,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_digital_channel_assignment = {
	"Word 2 - Digital Channel Assignment",
	{
		{ "T1T2",			2,	0 },
		{ "MEM",			1,	0 },
		{ "PM",				1,	0 },
		{ "PSCC",			2,	0 },
		{ "SBI",			2,	0 },
		{ "TA",				5,	0 },
		{ "DMAC",			4,	0 },
		{ "CHAN",			11,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_digital_control_channel_information_word = {
	"Word 2 - Digital Control Channel Information Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			5,	0 },
		{ "Hyperband",			2,	0 },
		{ "DVCC",			8,	0 },
		{ "CHAN",			11,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_base_station_challenge_order_confirmation = {
	"Word 2 - Base Station Challenge Order Confirmation",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			4,	0 },
		{ "AUTHBS",			18,	0 },
		{ "RSVD",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_unique_challenge_order_word = {
	"Word 2 - Unique Challenge Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "RANDU",			24,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_first_ssd_update_order_word = {
	"Word 2 - First SSD Update Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "RANDSSD_1",			24,	0 },
		{ "RSVD",			2,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_first_alert_with_info_word = {
	"Word 2 - First Alert With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RL_W",			5,	0 },
		{ "SIGNAL",			8,	0 },
		{ "CPN_RL",			6,	0 },
		{ "PI",				2,	0 },
		{ "SI",				2,	0 },
		{ "RSVD",			3,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_first_flash_with_info_word = {
	"Word 2 - First Flash With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RL_W",			5,	0 },
		{ "CPN_RL",			6,	0 },
		{ "PI",				2,	0 },
		{ "SI",				2,	0 },
		{ "RSVD",			11,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_second_ssd_update_oder_word = {
	"Word 3 - Second SSD Update Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "RANDSSD_2",			24,	0 },
		{ "RSVD",			2,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_second_alert_with_info_word = {
	"Word 3 - Second Alert With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_second_alert_with_info_cri_message_word = {
	"Word 3 - Second Alert With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E14",			4,	0 },
		{ "CRI E13",			4,	0 },
		{ "CRI E12",			4,	0 },
		{ "CRI E11",			4,	0 },
		{ "CRI E24",			4,	0 },
		{ "CRI E23",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_second_alert_with_info_tci_message_word = {
	"Word 3 - Second Alert With Info TCI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "TCI1",			7,	0 },
		{ "TCI5",			1,	0 },
		{ "TCI24",			4,	0 },
		{ "TCI23",			4,	0 },
		{ "TCI22",			4,	0 },
		{ "TCI21",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_second_flash_with_info_word = {
	"Word 3 - Second Flash With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_second_flash_with_info_cri_message_word = {
	"Word 3 - Second Flash With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E14",			4,	0 },
		{ "CRI E13",			4,	0 },
		{ "CRI E12",			4,	0 },
		{ "CRI E11",			4,	0 },
		{ "CRI E24",			4,	0 },
		{ "CRI E23",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_second_flash_with_info_tci_message_word = {
	"Word 3 - Second Flash With Info TCI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "TCI1",			7,	0 },
		{ "TCI5",			1,	0 },
		{ "TCI24",			4,	0 },
		{ "TCI23",			4,	0 },
		{ "TCI22",			4,	0 },
		{ "TCI21",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_third_ssd_update_order_word = {
	"Word 4 - Third SSD Update Order Word",
	{
		{ "T1T2",			2,	0 },
		{ "RANDSSD_3",			8,	0 },
		{ "RSVD",			18,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_third_alert_with_info_word = {
	"Word 4 - Third Alert With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_third_alert_with_info_cri_message_word = {
	"Word 4 - Third Alert With Info CRI Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E22",			4,	0 },
		{ "CRI E21",			4,	0 },
		{ "CRI E34",			4,	0 },
		{ "CRI E33",			4,	0 },
		{ "CRI E32",			4,	0 },
		{ "CRI E31",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_third_alert_with_info_tci_message_word = {
	"Word 4 - Third Alert With Info TCI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "TCI34",			7,	0 },
		{ "TCI33",			1,	0 },
		{ "TCI32",			4,	0 },
		{ "TCI31",			4,	0 },
		{ "TCI44",			4,	0 },
		{ "TCI43",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_third_flash_with_info_word = {
	"Word 4 - Third Flash With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_third_flash_with_info_cri_message_word = {
	"Word 4 - Third Flash With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E22",			4,	0 },
		{ "CRI E21",			4,	0 },
		{ "CRI E34",			4,	0 },
		{ "CRI E33",			4,	0 },
		{ "CRI E32",			4,	0 },
		{ "CRI E31",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_third_flash_with_info_tci_message_word = {
	"Word 4 - Third Flash With Info TCI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "TCI34",			4,	0 },
		{ "TCI33",			4,	0 },
		{ "TCI32",			4,	0 },
		{ "TCI31",			4,	0 },
		{ "TCI44",			4,	0 },
		{ "TCI43",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word5_alert_with_info_word = {
	"Word 5 - Fourth Alert With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word5_alert_with_info_cri_message_word = {
	"Word 5 - Fourth Alert With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E44",			4,	0 },
		{ "CRI E43",			4,	0 },
		{ "CRI E42",			4,	0 },
		{ "CRI E41",			4,	0 },
		{ "CRI E54",			4,	0 },
		{ "CRI E53",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word5_alert_with_info_tci_message_word = {
	"Word 5 - Fourth Alert With Info TCI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "TCI42",			4,	0 },
		{ "TCI41",			4,	0 },
		{ "NULL",			8,	0 },
		{ "NULL",			8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word5_flash_with_info_word = {
	"Word 5 - Fourth Flash With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word5_flash_with_info_cri_message_word = {
	"Word 5 - Fourth Flash With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E44",			4,	0 },
		{ "CRI E43",			4,	0 },
		{ "CRI E42",			4,	0 },
		{ "CRI E41",			4,	0 },
		{ "CRI E54",			4,	0 },
		{ "CRI E53",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word5_flash_with_info_tci_message_word = {
	"Word 5 - Fourth Flash With Info TCI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "TCI42",			4,	0 },
		{ "TCI41",			4,	0 },
		{ "NULL",			8,	0 },
		{ "NULL",			8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word wordn_n_minus_1th_alert_with_info_word = {
	"Word N - (N-1)th Alert With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word6_fifth_alert_with_info_cri_mesage_word = {
	"Word 6 - Fifth Alert With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E52",			4,	0 },
		{ "CRI E51",			4,	0 },
		{ "CRI E64",			4,	0 },
		{ "CRI E63",			4,	0 },
		{ "CRI E62",			4,	0 },
		{ "CRI E61",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word7_sixth_alert_with_info_cri_mesage_word = {
	"Word 7 - Sixth Alert With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E74",			4,	0 },
		{ "CRI E73",			4,	0 },
		{ "CRI E72",			4,	0 },
		{ "CRI E71",			4,	0 },
		{ "CRI E84",			4,	0 },
		{ "CRI E83",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word8_seventh_alert_with_info_cri_mesage_word = {
	"Word 8 - Seventh Alert With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E82",			4,	0 },
		{ "CRI E81",			4,	0 },
		{ "NULL",			8,	0 },
		{ "NULL",			8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word wordn_n_minus_1th_flash_with_info_word = {
	"Word N - (N-1)th Flash With Info Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CHARACTER 1",		8,	0 },
		{ "CHARACTER 2",		8,	0 },
		{ "CHARACTER 3",		8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word6_fith_flash_with_info_cri_message_word = {
	"Word 6 - Fifth Flash With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E52",			4,	0 },
		{ "CRI E51",			4,	0 },
		{ "CRI E64",			4,	0 },
		{ "CRI E63",			4,	0 },
		{ "CRI E62",			4,	0 },
		{ "CRI E61",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word7_sixth_flash_with_info_cri_message_word = {
	"Word 7 - Sixth Flash With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E74",			4,	0 },
		{ "CRI E73",			4,	0 },
		{ "CRI E72",			4,	0 },
		{ "CRI E71",			4,	0 },
		{ "CRI E84",			4,	0 },
		{ "CRI E83",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word8_seventh_flash_with_info_cri_message_word = {
	"Word 8 - Seventh Flash With Info CRI Message Word",
	{
		{ "T1T2",			2,	0 },
		{ "RSVD",			2,	0 },
		{ "CRI E82",			4,	0 },
		{ "CRI E81",			4,	0 },
		{ "NULL",			8,	0 },
		{ "NULL",			8,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_message_set fvc_words = {
	"FVC Words", 40,
	{
		&mobile_station_control_message_word1_a,
		&mobile_station_control_message_word1_b,
		&word2_digital_channel_assignment,
		&word2_digital_control_channel_information_word,
		&word2_base_station_challenge_order_confirmation,
		&word2_unique_challenge_order_word,
		&word2_first_ssd_update_order_word,
		&word2_first_alert_with_info_word,
		&word2_first_flash_with_info_word,
		&word3_second_ssd_update_oder_word,
		&word3_second_alert_with_info_word,
		&word3_second_alert_with_info_cri_message_word,
		&word3_second_alert_with_info_tci_message_word,
		&word3_second_flash_with_info_word,
		&word3_second_flash_with_info_cri_message_word,
		&word3_second_flash_with_info_tci_message_word,
		&word4_third_ssd_update_order_word,
		&word4_third_alert_with_info_word,
		&word4_third_alert_with_info_cri_message_word,
		&word4_third_alert_with_info_tci_message_word,
		&word4_third_flash_with_info_word,
		&word4_third_flash_with_info_cri_message_word,
		&word4_third_flash_with_info_tci_message_word,
		&word5_alert_with_info_word,
		&word5_alert_with_info_cri_message_word,
		&word5_alert_with_info_tci_message_word,
		&word5_flash_with_info_word,
		&word5_flash_with_info_cri_message_word,
		&word5_flash_with_info_tci_message_word,
		&wordn_n_minus_1th_alert_with_info_word,
		&word6_fifth_alert_with_info_cri_mesage_word,
		&word7_sixth_alert_with_info_cri_mesage_word,
		&word8_seventh_alert_with_info_cri_mesage_word,
		&wordn_n_minus_1th_flash_with_info_word,
		&word6_fith_flash_with_info_cri_message_word,
		&word7_sixth_flash_with_info_cri_message_word,
		&word8_seventh_flash_with_info_cri_message_word,
		NULL
	}
};

/* RVC - Order Confirmation Message */

static struct def_word order_confirmation_message = {
	"Order/Order Confirmation Message",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "RSVD",			19,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* RVC - Called-Address Message */

static struct def_word word1_called_address = {
	"Word 1 - First Word of the Called-Address",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "DIGIT 1",			4,	0 },
		{ "DIGIT 2",			4,	0 },
		{ "DIGIT 3",			4,	0 },
		{ "DIGIT 4",			4,	0 },
		{ "DIGIT 5",			4,	0 },
		{ "DIGIT 6",			4,	0 },
		{ "DIGIT 7",			4,	0 },
		{ "DIGIT 8",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_called_address = {
	"Word 2 - Second Word of the Called-Address",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "DIGIT 9",			4,	0 },
		{ "DIGIT 10",			4,	0 },
		{ "DIGIT 11",			4,	0 },
		{ "DIGIT 12",			4,	0 },
		{ "DIGIT 13",			4,	0 },
		{ "DIGIT 14",			4,	0 },
		{ "DIGIT 15",			4,	0 },
		{ "DIGIT 16",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word3_called_address = {
	"Word 3 - Third Word of the Called-Address",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "DIGIT 17",			4,	0 },
		{ "DIGIT 18",			4,	0 },
		{ "DIGIT 19",			4,	0 },
		{ "DIGIT 20",			4,	0 },
		{ "DIGIT 21",			4,	0 },
		{ "DIGIT 22",			4,	0 },
		{ "DIGIT 23",			4,	0 },
		{ "DIGIT 24",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word4_called_address = {
	"Word 4 - Fourth Word of the Called-Address",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "DIGIT 25",			4,	0 },
		{ "DIGIT 26",			4,	0 },
		{ "DIGIT 27",			4,	0 },
		{ "DIGIT 28",			4,	0 },
		{ "DIGIT 29",			4,	0 },
		{ "DIGIT 30",			4,	0 },
		{ "DIGIT 31",			4,	0 },
		{ "DIGIT 32",			4,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* RVC - Serial Number Response Message */

static struct def_word word1_serial_number_response_message = {
	"Word 1 of Serial Number Response Message",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "RSVD",			19,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_serial_number_response_message = {
	"Word 2 of Serial Number response message",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "ESN",			32,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word page_response = {
	"Page Response",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "RSVD",			19,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word challenge_order_confirmation_message = {
	"Unique Challenge Order Confirmation Message",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "AUTHU",			18,	0 },
		{ "RSVD",			1,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

/* RVC - Base Station Challenge Order Message */

static struct def_word word1_base_station_challenge_order_message = {
	"Word 1 of Base Station Challenge Order Message",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "LOCAL/MSG TYPE",		5,	0 },
		{ "ORDQ",			3,	0 },
		{ "ORDER",			5,	0 },
		{ "RSVD",			19,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_word word2_base_station_challenge_order_message = {
	"Word 2 of Base Station Challenge Order Message",
	{
		{ "F",				1,	0 },
		{ "NAWC",			2,	0 },
		{ "T",				1,	0 },
		{ "RANDBS",			32,	0 },
		{ "P",				12,	0 },
		{ NULL, 0, 0 }
	}
};

static struct def_message_set rvc_words = {
	"RVC Words", 48,
	{
		&order_confirmation_message,
		&word1_called_address,
		&word2_called_address,
		&word3_called_address,
		&word4_called_address,
		&word1_serial_number_response_message,
		&word2_serial_number_response_message,
		&page_response,
		&challenge_order_confirmation_message,
		&word1_base_station_challenge_order_message,
		&word2_base_station_challenge_order_message,
		NULL
	}
};

static struct def_message_set *amps_message_sets[] = {
	&focc_words,
	&recc_words,
	&fvc_words,
	&rvc_words,
	NULL
};

static const char *amps_act[16] = {
	"Reserved",
	"Rescan paging channels",
	"Registration increment",
	"Location Area",
	"Reserved",
	"Reserved",
	"New access channel set",
	"Random Challenge A",
	"Overload control",
	"Access type parameters",
	"Access attempt parameters",
	"Random Challenge B",
	"Reserved",
	"Reserved",
	"Local Control 1",
	"Local Control 2",
};

static const char *ie_hex(uint64_t value)
{
	static char string[64];
	
	sprintf(string, "0x%" PRIx64, value);
	return string;
}

static const char *ie_act(uint64_t value)
{
	return amps_act[value & 0xf];
}

static const char *ie_yes(uint64_t value)
{
	if (value)
		return "Yes";
	return "No";
}

static const char *ie_bis(uint64_t value)
{
	if (value)
		return "Wait for Idle-Busy transition";
	return "Ignore Idle-Busy after access";
}

static const char *ie_bscap(uint64_t value)
{
	switch (value) {
	case 0:
		return "Reserved for backward compatibility";
	case 1:
		return "ANSI TIA/EIA-553-A";
	}
	return "Reserved";
}

static const char *ie_bspc(uint64_t value)
{
	switch (value) {
	case 0:
		return "Reserved for backward compatibility";
	case 2:
		return "IS-91A or TIA/EIA-691";
	case 3:
		return "TIA/EIA-136-B";
	case 4:
		return "IS-95B or TIA/EIA-95";
	}
	return "Reserved";
}

static const char *ie_chan(uint64_t value)
{
	static char string[64];
	
	if (value == 0)
		return "No channel";
	sprintf(string, "%" PRIu64 " = %.3f MHz", value, amps_channel2freq(value, 0) / 1e6);
	return string;
}

static const char *ie_cmac(uint64_t value)
{
	switch (value) {
	case 0:
		if (!tacs)
			return "6 dbW (4 Watts)";
		else
			return "10 dbW (10 Watts)";
	case 1:
		return "2 dbW (1.6 Watts)";
	case 2:
		return "-2 dbW (630 Milliwatts)";
	case 3:
		return "-6 dbW (250 Milliwatts)";
	case 4:
		return "-10 dbW (100 Milliwatts)";
	case 5:
		return "-14 dbW (40 Milliwatts)";
	case 6:
		return "-18 dbW (16 Milliwatts)";
	}
	return "-22 dbW (6.3 Milliwatts)";
}

static const char *ie_cmax(uint64_t value)
{
	static char string[32];
	
	sprintf(string, "%" PRIu64, value + 1);
	return string;
}

static const char *ie_n(uint64_t value)
{
	static char string[32];
	
	sprintf(string, "%" PRIu64, value + 1);
	return string;
}

static const char *ie_dtx_support(uint64_t value)
{
	switch (value) {
	case 0:
		return "DTX Not Supported";
	case 2:
		return "DTX Supported up to 8 dB attenuation";
	case 3:
		return "DTX Supported with no limit on attenuation";
	}
	return "Reserved";
}

static const char *ie_hyperband(uint64_t value)
{
	switch (value) {
	case 0:
		return "800 MHz";
	case 1:
		return "1900 MHz";
	}
	return "Reserved";
}

static const char *ie_enabled(uint64_t value)
{
	if (value)
		return "Enabled";
	return "Disabled";
}

static const char *amps_ohd[8] = {
	"Registration ID",
	"Control-Filler",
	"Control Channel Information",
	"Reserved",
	"Global action",
	"Reserved",
	"Word 1 of system parameter message",
	"Word 2 of system parameter message",
};

static const char *ie_ohd(uint64_t value)
{
	return amps_ohd[value & 0x7];
}

static const char *amps_t1t2[4] = {
	"Only Word 1",
	"First Word (FOCC) Next Word (FVC)",
	"Next Word (FOCC) First Word (FVC)",
	"Overhead Message",
};

static const char *ie_t1t2(uint64_t value)
{
	return amps_t1t2[value & 0x3];
}

static const char *ie_digit(uint64_t value)
{
	static char string[32];

	switch (value) {
	case 0:
		return "NULL";
	case 10:
		return "0";
	case 11:
		return "*";
	case 12:
		return "#";
	case 13:
		return "+";
	case 14:
		return "reserved 15";
	case 15:
		return "reserved 16";
	}
	sprintf(string, "%" PRIu64, value);
	return string;
}

const char *ie_mscap(uint64_t value)
{
	switch (value) {
	case 0:
		return "Reserved for backward compatibility";
	case 1:
		return "TIA/EIA-553-A";
	}
	return "Reserved";
}

const char *ie_mspc(uint64_t value)
{
	switch (value) {
	case 0:
		return "Reserved for backward compatibility";
	case 1:
		return "TIA/EIA-553-A";
	case 2:
		return "IS-91A";
	case 3:
		return "TIA/EIA-136-B";
	case 4:
		return "IS-95B";
	}
	return "Reserved";
}

static const char *ie_service_code(uint64_t value)
{
	switch (value) {
	case 4:
		return "Async Data";
	case 5:
		return "G3 Fax";
	case 6:
		return "Service Rejected";
	case 8:
		return "Direct Async Data Service";
	}
	return "Reserved";
}

static const char *amps_scc[4] = {
	"5970 Hz",
	"6000 Hz",
	"6030 Hz",
	"This word includes message",
};

static const char *ie_scc(uint64_t value)
{
	return amps_scc[value & 0x3];
}

static const char *amps_acked_data[2] = {
	"Acknowledged data, unacknowledged data, or both",
	"Unacknowledged data only",
};

static const char *ie_acked_data(uint64_t value)
{
	return amps_acked_data[value & 0x1];
}

static const char *ie_ascii(uint64_t value)
{
	static char string[32];

	if (value >= 32 && value <= 126)
		sprintf(string, "'%c'", (char)value);
	else
		sprintf(string, "0x%02x", (unsigned char)value);

	return string;
}

static const char *amps_crc[4] = {
	"16-bit CRC",
	"24-bit CRC",
	"No CRC",
	"Reserved",
};

static const char *ie_crc(uint64_t value)
{
	return amps_crc[value & 0x3];
}

static const char *ie_data_part(uint64_t value)
{
	switch (value) {
	case 0:
		return "See TIA/EIA-136-350";
	case 1:
		return "STU-II (Standard FSVS211)";
	}
	return "Reserved";
}

static const char *amps_mpci[4] = {
	"indicates TIA/EIA-553 or IS-54A mobile station",
	"indicates TIA/EIA-627 dual-mode mobile station",
	"reserved (see TIA/EIA IS-95)",
	"indicates EIATIA/EIA-136 dual-mode mobile station",
};

const char *ie_mpci(uint64_t value)
{
	return amps_mpci[value & 0x3];
}

static const char *amps_pi[4] = {
	"Presentation Allowed",
	"Presentation Restricted",
	"Number Not Available",
	"Reserved",
};

static const char *ie_pi(uint64_t value)
{
	return amps_pi[value & 0x3];
}

static const char *ie_pm_d(uint64_t value)
{
	switch (value) {
	case 0:
		return "No Data Privacy";
	case 1:
		return "Data Privacy Algorithm A (ORYX)";
	case 2:
		return "Data Privacy Algorithm B (SCEMA)";
	}
	return "Reserved";
}

static const char *amps_pvi[2] = {
	"TIA/EIA 627",
	"TIA/EIA-136",
};

static const char *ie_pvi(uint64_t value)
{
	return amps_pvi[value & 0x1];
}

static const char *ie_rlp(uint64_t value)
{
	switch (value) {
	case 0:
		return "RLP1";
	case 1:
		return "RLP2";
	}
	return "Reserved";
}

static const char *amps_sap[2] = {
	"SAP 0 only",
	"SAP 0 and SAP 1",
};

static const char *ie_sap(uint64_t value)
{
	return amps_sap[value & 0x1];
}

static const char *ie_sbi(uint64_t value)
{
	switch (value) {
	case 0:
		return "Transmit normal burst after cell to cell handoff";
	case 1:
		return "Transmit normal burst after handoff within cell";
	case 2:
		return "Transmit shortened burst after cell to cell handoff";
	}
	return "Reserved";
}

static const char *amps_ie_si[4] = {
	"User-provided, not screened",
	"User-provided, verified and passed",
	"User-provided, verified and failed",
	"Network-provided",
};

static const char *ie_si(uint64_t value)
{
	return amps_ie_si[value & 0x3];
}

static const char *ie_signal(uint64_t value)
{
	static char string[256];
	const char *pitch, *cadence;

	switch ((value >> 6) & 0x3) {
	case 0:
		pitch = "Medium pitch";
		break;
	case 1:
		pitch = "High pitch";
		break;
	case 2:
		pitch = "Low pitch";
		break;
	default:
		pitch = "Reserved";
	}
	switch (value & 0x3f) {
	case 0:
		cadence = "No Tone";
		break;
	case 1:
		cadence = "Long";
		break;
	case 2:
		cadence = "Short-Short";
		break;
	case 3:
		cadence = "Short-Short-Long";
		break;
	case 4:
		cadence = "Short-Short-2";
		break;
	case 5:
		cadence = "Short-Long-Short";
		break;
	case 6:
		cadence = "Short-Short-Short-Short";
		break;
	case 7:
		cadence = "PBX Long";
		break;
	case 8:
		cadence = "PBX Short-Short";
		break;
	case 9:
		cadence = "PBX Short-Short-Long";
		break;
	case 10:
		cadence = "PBX Short-Long-Short";
		break;
	case 11:
		cadence = "PBX Short-Short-Short-Short";
		break;
	default:
		cadence = "Reserved";
	}
	sprintf(string, "Pitch=%s, Cadence=%s", pitch, cadence);

	return string;
}

static const char *ie_min1(uint64_t value)
{
	return amps_min12number(value);
}

static const char *ie_min2(uint64_t value)
{
	return amps_min22number(value);
}

static const char *ie_scm(uint64_t value)
{
	return amps_scm(value);
}

struct amps_ie_desc {
	enum amps_ie	ie;
	const char	*name;
	const char	*desc;
	const char	*(*decoder)(uint64_t value);
};

struct amps_ie_desc amps_ie_desc[] = {
	{ AMPS_IE_010111,		"010111",		"bit combination 23", NULL },
	{ AMPS_IE_1,			"1",			"bit combination 1", NULL },
	{ AMPS_IE_11,			"11",			"bit combination 3", NULL },
	{ AMPS_IE_1111,			"1111",			"bit combination 15", NULL },
	{ AMPS_IE_ACT,			"ACT",			"Global action field", ie_act },
	{ AMPS_IE_AID1,			"AID1",			"First part of the area identification field", NULL },
	{ AMPS_IE_AUTH,			"AUTH",			"Support of authentication procedures described in TIA/EIA-136-510", ie_yes },
	{ AMPS_IE_AUTHBS,		"AUTHBS",		"Output response of the authentication algorithm initiated by the Base Station Challenge Order", ie_hex },
	{ AMPS_IE_AUTHR,		"AUTHR",		"Output response of the authentication algorithm", ie_hex },
	{ AMPS_IE_AUTHU,		"AUTHU",		"Output of the authentication algorithm when responding to a Unique Challenge Order", ie_hex },
	{ AMPS_IE_Acked_Data,		"Acked Data",		"Used to identidy the selected privacy mode for a data/fax call", ie_acked_data },
	{ AMPS_IE_Async_Data,		"Async Data",		"Async Data is supported on the current Analog Control Channel", ie_yes },
	{ AMPS_IE_BIS,			"BIS",			"Busy-Idle status field", ie_bis },
	{ AMPS_IE_BSCAP,		"BSCAP",		"Base Station Core Analog Protocol field", ie_bscap },
	{ AMPS_IE_BSPC,			"BSPC",			"Base Station Protocol Capability field", ie_bspc },
	{ AMPS_IE_CHAN,			"CHAN",			"Channel number field", ie_chan },
	{ AMPS_IE_CHANPOS,		"CHANPOS",		"Channel position field (relative to FIRSTCHA)", NULL },
	{ AMPS_IE_CHARACTER_1,		"CHARACTER 1",		"ASCII Character", ie_ascii },
	{ AMPS_IE_CHARACTER_2,		"CHARACTER 2",		"ASCII Character", ie_ascii },
	{ AMPS_IE_CHARACTER_3,		"CHARACTER 3",		"ASCII Character", ie_ascii },
	{ AMPS_IE_CMAC,			"CMAC",			"Control mobile attenuation field", ie_cmac },
	{ AMPS_IE_CMAX_1,		"CMAX-1",		"CMAX is the number of access channels in the system", ie_cmax },
	{ AMPS_IE_COUNT,		"COUNT",		"A modulo-64 count for authentication", NULL },
	{ AMPS_IE_CPA,			"CPA",			"Combined paging/access field", ie_yes },
	{ AMPS_IE_CPN_RL,		"CPN_RL",		"Number of Characters in Calling Party Number", NULL },
	{ AMPS_IE_CRC,			"CRC",			"Identifies used CRC", ie_crc },
	{ AMPS_IE_CRI_E11,		"CRI E11",		"Charing Rate Indication Element 1 Digit 1", NULL },
	{ AMPS_IE_CRI_E12,		"CRI E12",		"Charing Rate Indication Element 1 Digit 2", NULL },
	{ AMPS_IE_CRI_E13,		"CRI E13",		"Charing Rate Indication Element 1 Digit 3", NULL },
	{ AMPS_IE_CRI_E14,		"CRI E14",		"Charing Rate Indication Element 1 Digit 4", NULL },
	{ AMPS_IE_CRI_E21,		"CRI E21",		"Charing Rate Indication Element 2 Digit 1", NULL },
	{ AMPS_IE_CRI_E22,		"CRI E22",		"Charing Rate Indication Element 2 Digit 2", NULL },
	{ AMPS_IE_CRI_E23,		"CRI E23",		"Charing Rate Indication Element 2 Digit 3", NULL },
	{ AMPS_IE_CRI_E24,		"CRI E24",		"Charing Rate Indication Element 2 Digit 4", NULL },
	{ AMPS_IE_CRI_E31,		"CRI E31",		"Charing Rate Indication Element 3 Digit 1", NULL },
	{ AMPS_IE_CRI_E32,		"CRI E32",		"Charing Rate Indication Element 3 Digit 2", NULL },
	{ AMPS_IE_CRI_E33,		"CRI E33",		"Charing Rate Indication Element 3 Digit 3", NULL },
	{ AMPS_IE_CRI_E34,		"CRI E34",		"Charing Rate Indication Element 3 Digit 4", NULL },
	{ AMPS_IE_CRI_E41,		"CRI E41",		"Charing Rate Indication Element 4 Digit 1", NULL },
	{ AMPS_IE_CRI_E42,		"CRI E42",		"Charing Rate Indication Element 4 Digit 2", NULL },
	{ AMPS_IE_CRI_E43,		"CRI E43",		"Charing Rate Indication Element 4 Digit 3", NULL },
	{ AMPS_IE_CRI_E44,		"CRI E44",		"Charing Rate Indication Element 4 Digit 4", NULL },
	{ AMPS_IE_CRI_E51,		"CRI E51",		"Charing Rate Indication Element 5 Digit 1", NULL },
	{ AMPS_IE_CRI_E52,		"CRI E52",		"Charing Rate Indication Element 5 Digit 2", NULL },
	{ AMPS_IE_CRI_E53,		"CRI E53",		"Charing Rate Indication Element 5 Digit 3", NULL },
	{ AMPS_IE_CRI_E54,		"CRI E54",		"Charing Rate Indication Element 5 Digit 4", NULL },
	{ AMPS_IE_CRI_E61,		"CRI E61",		"Charing Rate Indication Element 6 Digit 1", NULL },
	{ AMPS_IE_CRI_E62,		"CRI E62",		"Charing Rate Indication Element 6 Digit 2", NULL },
	{ AMPS_IE_CRI_E63,		"CRI E63",		"Charing Rate Indication Element 6 Digit 3", NULL },
	{ AMPS_IE_CRI_E64,		"CRI E64",		"Charing Rate Indication Element 6 Digit 4", NULL },
	{ AMPS_IE_CRI_E71,		"CRI E71",		"Charing Rate Indication Element 7 Digit 1", NULL },
	{ AMPS_IE_CRI_E72,		"CRI E72",		"Charing Rate Indication Element 7 Digit 2", NULL },
	{ AMPS_IE_CRI_E73,		"CRI E73",		"Charing Rate Indication Element 7 Digit 3", NULL },
	{ AMPS_IE_CRI_E74,		"CRI E74",		"Charing Rate Indication Element 7 Digit 4", NULL },
	{ AMPS_IE_CRI_E81,		"CRI E81",		"Charing Rate Indication Element 8 Digit 1", NULL },
	{ AMPS_IE_CRI_E82,		"CRI E82",		"Charing Rate Indication Element 8 Digit 2", NULL },
	{ AMPS_IE_CRI_E83,		"CRI E83",		"Charing Rate Indication Element 8 Digit 3", NULL },
	{ AMPS_IE_CRI_E84,		"CRI E84",		"Charing Rate Indication Element 8 Digit 4", NULL },
	{ AMPS_IE_DCC,			"DCC",			"Digital color code field", NULL },
	{ AMPS_IE_DIGIT_1,		"DIGIT 1",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_10,		"DIGIT 10",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_11,		"DIGIT 11",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_12,		"DIGIT 12",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_13,		"DIGIT 13",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_14,		"DIGIT 14",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_15,		"DIGIT 15",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_16,		"DIGIT 16",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_17,		"DIGIT 17",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_18,		"DIGIT 18",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_19,		"DIGIT 19",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_2,		"DIGIT 2",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_20,		"DIGIT 20",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_21,		"DIGIT 21",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_22,		"DIGIT 22",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_23,		"DIGIT 23",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_24,		"DIGIT 24",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_25,		"DIGIT 25",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_26,		"DIGIT 26",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_27,		"DIGIT 27",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_28,		"DIGIT 28",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_29,		"DIGIT 29",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_3,		"DIGIT 3",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_30,		"DIGIT 30",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_31,		"DIGIT 31",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_32,		"DIGIT 32",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_4,		"DIGIT 4",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_5,		"DIGIT 5",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_6,		"DIGIT 6",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_7,		"DIGIT 7",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_8,		"DIGIT 8",		"Digit field", ie_digit },
	{ AMPS_IE_DIGIT_9,		"DIGIT 9",		"Digit field", ie_digit },
	{ AMPS_IE_DMAC,			"DMAC",			"Digital mobile attenuation code field", ie_cmac },
	{ AMPS_IE_DTX,			"DTX",			"Discontinuous-Transmission field", ie_yes },
	{ AMPS_IE_DTX_Support,		"DTX Support",		"Indicates the nature of DTX supported on an analog voice", ie_dtx_support },
	{ AMPS_IE_DVCC,			"DVCC",			"Digital Verification Color Code", NULL},
	{ AMPS_IE_Data_Part,		"Data Part",		"Identifies the Data Port associated with a data/fax call", ie_data_part },
	{ AMPS_IE_Data_Privacy,		"Data Privacy",		"This field indicates whether or not Data Privacy is supported", ie_yes },
	{ AMPS_IE_E,			"E",			"Extended address field", ie_yes },
	{ AMPS_IE_EC,			"EC",			"Extended Protocol Reverse Channel", ie_yes },
	{ AMPS_IE_EF,			"EF",			"Extended Protocol Forward Channel Indicator", ie_yes },
	{ AMPS_IE_END,			"END",			"End indication field", ie_yes },
	{ AMPS_IE_EP,			"EP",			"Extended Protocol Capability Indicator", ie_yes },
	{ AMPS_IE_ER,			"ER",			"Extended Protocol Reverse Channel", ie_yes },
	{ AMPS_IE_ESN,			"ESN",			"Electronic Serial Number field", ie_hex },
	{ AMPS_IE_F,			"F",			"First word indication Field", ie_yes },
	{ AMPS_IE_G3_Fax,		"G3 Fax",		"This field indicates whether or not G3 Fax is supported", ie_yes },
	{ AMPS_IE_HDVCC,		"HDVCC",		"Half Digital Verification Color Code", NULL },
	{ AMPS_IE_Hyperband,		"Hyperband",		"Designates Hyperband associated with the Digital Control Channel specified by the CHAN field", ie_hyperband },
	{ AMPS_IE_LOCAID,		"LOCAID",		"Location area identity field", NULL },
	{ AMPS_IE_LOCAL_CONTROL,	"LOCAL CONTROL",	"May be set to any bit pattern", NULL },
	{ AMPS_IE_LOCAL_MSG_TYPE,	"LOCAL/MSG TYPE",	"Message Type field", NULL },
	{ AMPS_IE_LREG,			"LREG",			"Location area ID registration status field", ie_enabled },
	{ AMPS_IE_LT,			"LT",			"Last-try code field", ie_yes },
	{ AMPS_IE_MAXBUSY_OTHER,	"MAXBUSY-OTHER",	"Maximum busy occurrences field (other accesses)", NULL },
	{ AMPS_IE_MAXBUSY_PGR,		"MAXBUSY-PGR",		"Maximum busy occurrences field (page response)", NULL },
	{ AMPS_IE_MAXSZTR_OTHER,	"MAXSZTR-OTHER",	"Maximum seizure tries field (other accesses)", NULL },
	{ AMPS_IE_MAXSZTR_PGR,		"MAXSZTR-PGR",		"Maximum seizure tries field (page response)", NULL },
	{ AMPS_IE_MEM,			"MEM",			"Message Encryption Mode", ie_yes },
	{ AMPS_IE_MIN1,			"MIN1",			"First part of the mobile identification number field", ie_min1 },
	{ AMPS_IE_MIN2,			"MIN2",			"Second part of the mobile identification number field", ie_min2 },
	{ AMPS_IE_MPCI,			"MPCI",			"Mobile station Protocol Indicator", ie_mpci },
	{ AMPS_IE_MSCAP,		"MSCAP",		"Mobile Station Core Analog Protocol field", ie_mscap },
	{ AMPS_IE_MSPC,			"MSPC",			"Mobile Station Protocol Capability field", ie_mspc },
	{ AMPS_IE_N_1,			"N-1",			"N is the number of paging channels in the system", ie_n },
	{ AMPS_IE_NAWC	,		"NAWC",			"Number of additional words coming field", NULL },
	{ AMPS_IE_NEWACC,		"NEWACC",		"New access channel starting point field", ie_chan },
	{ AMPS_IE_NULL,			"NULL",			"Null character", NULL },
	{ AMPS_IE_OHD,			"OHD",			"Overhead Message Type field", ie_ohd },
	{ AMPS_IE_OLC_0,		"OLC 0",		"Overload class field 0", NULL },
	{ AMPS_IE_OLC_1,		"OLC 1",		"Overload class field 1", NULL },
	{ AMPS_IE_OLC_10,		"OLC 10",		"Overload class field 10", NULL },
	{ AMPS_IE_OLC_11,		"OLC 11",		"Overload class field 11", NULL },
	{ AMPS_IE_OLC_12,		"OLC 12",		"Overload class field 12", NULL },
	{ AMPS_IE_OLC_13,		"OLC 13",		"Overload class field 13", NULL },
	{ AMPS_IE_OLC_14,		"OLC 14",		"Overload class field 14", NULL },
	{ AMPS_IE_OLC_15,		"OLC 15",		"Overload class field 15", NULL },
	{ AMPS_IE_OLC_2,		"OLC 2",		"Overload class field 2", NULL },
	{ AMPS_IE_OLC_3,		"OLC 3",		"Overload class field 3", NULL },
	{ AMPS_IE_OLC_4,		"OLC 4",		"Overload class field 4", NULL },
	{ AMPS_IE_OLC_5,		"OLC 5",		"Overload class field 5", NULL },
	{ AMPS_IE_OLC_6,		"OLC 6",		"Overload class field 6", NULL },
	{ AMPS_IE_OLC_7,		"OLC 7",		"Overload class field 7", NULL },
	{ AMPS_IE_OLC_8,		"OLC 8",		"Overload class field 8", NULL },
	{ AMPS_IE_OLC_9,		"OLC 9",		"Overload class field 9", NULL },
	{ AMPS_IE_ORDER,		"ORDER",		"Order field", NULL },
	{ AMPS_IE_ORDQ,			"ORDQ",			"Order qualifier field", NULL },
	{ AMPS_IE_P,			"P",			"Parity field", NULL },
	{ AMPS_IE_PCI,			"PCI",			"Set to 1 if Control Channel can assign digital traffic channels", ie_yes },
	{ AMPS_IE_PCI_HOME,		"PCI_HOME",		"Home Protocol Capability Indicator", ie_yes },
	{ AMPS_IE_PCI_ROAM,		"PCI_ROAM",		"Roam Protocol Capability Indicator", ie_yes },
	{ AMPS_IE_PDREG,		"PDREG",		"Power Down Registration status field", ie_enabled },
	{ AMPS_IE_PI,			"PI",			"Presentation Indicator", ie_pi },
	{ AMPS_IE_PM,			"PM",			"Privacy Mode indicator", ie_yes },
	{ AMPS_IE_PM_D,			"PM_D",			"Privacy Mode indicator for Fax/Data", ie_pm_d },
	{ AMPS_IE_PSCC,			"PSCC",			"Present SAT color code", ie_scc },
	{ AMPS_IE_PUREG,		"PUREG",		"Power Up Registration status field", ie_enabled },
	{ AMPS_IE_PVI,			"PVI",			"Protocol Version Indicator", ie_pvi },
	{ AMPS_IE_RAND1_A,		"RAND1_A",		"The 16 most significant bits of the 32-bit RAND variable stored by a mobile station for use in the authentication process", ie_hex },
	{ AMPS_IE_RAND1_B,		"RAND1_B",		"The 16 least significant bits of the 32-bit RAND variable stored by a mobile station for use in the authentication process", ie_hex },
	{ AMPS_IE_RANDBS,		"RANDBS",		"Random number used in the SSD Update procedure", ie_hex },
	{ AMPS_IE_RANDC,		"RANDC",		"Confirm the last RAND received by the mobile station", ie_hex },
	{ AMPS_IE_RANDSSD_1,		"RANDSSD_1",		"The most significant 24 bits of the random number issued by the base station in the SSD Update Order", ie_hex },
	{ AMPS_IE_RANDSSD_2,		"RANDSSD_2",		"The subsequent 24 bits (following RANDSSD_1) of the random number issued by the base station in the SSD Update Order", ie_hex },
	{ AMPS_IE_RANDSSD_3,		"RANDSSD_3",		"The least significant 8 bits of the random number issued by the base station in the SSD Update Order", ie_hex },
	{ AMPS_IE_RANDU,		"RANDU",		"The 24-bit random number issued by the base station in the Unique Challenge Order", ie_hex },
	{ AMPS_IE_RCF,			"RCF",			"Read-control-filler field", ie_yes },
	{ AMPS_IE_REGH,			"REGH",			"Registration field for home stations", ie_yes },
	{ AMPS_IE_REGID,		"REGID",		"Registration ID field", NULL },
	{ AMPS_IE_REGINCR,		"REGINCR",		"Registration increment field", NULL },
	{ AMPS_IE_REGR,			"REGR",			"Registration field for roaming stations", ie_yes },
	{ AMPS_IE_RLP,			"RLP",			"Identifies the layer 2 radio link protocol used for a data/fax call", ie_rlp },
	{ AMPS_IE_RL_W,			"RL_W",			"The remaining length, in `Words' of the Alert With Info or Flash With Info order", NULL },
	{ AMPS_IE_RSVD,			"RSVD",			"Reserved for future use", NULL },
	{ AMPS_IE_S,			"S",			"Serial number field", ie_yes },
	{ AMPS_IE_SAP,			"SAP",			"Service Access Point(s) used for data/fax call", ie_sap },
	{ AMPS_IE_SBI,			"SBI",			"Short Burst Indication", ie_sbi },
	{ AMPS_IE_SCC,			"SCC",			"SAT color code", ie_scc },
	{ AMPS_IE_SCM,			"SCM",			"The station class mark field", ie_scm },
	{ AMPS_IE_SDCC1,		"SDCC1",		"Supplementary Digital Color Codes", NULL },
	{ AMPS_IE_SDCC2,		"SDCC2",		"Supplementary Digital Color Codes", NULL },
	{ AMPS_IE_SI,			"SI",			"Screening Indicator", ie_si },
	{ AMPS_IE_SID1,			"SID1",			"First part of the system identification field", NULL },
	{ AMPS_IE_SIGNAL,		"SIGNAL",		"An 8-bit IE to cause MS to generate tones", ie_signal },
	{ AMPS_IE_Service_Code,		"Service Code",		"Service Indicator", ie_service_code },
	{ AMPS_IE_T,			"T",			"T field. 1 = Orig/Order 0 = (paging) response", NULL },
	{ AMPS_IE_T1T2,			"T1T2",			"Type field", ie_t1t2 },
	{ AMPS_IE_TA,			"TA",			"Time Alignment Offset", NULL },
	{ AMPS_IE_TCI1,			"TCI1",			"Total Charing component", NULL },
	{ AMPS_IE_TCI21,		"TCI21",		"Total Charing component", NULL },
	{ AMPS_IE_TCI22,		"TCI22",		"Total Charing component", NULL },
	{ AMPS_IE_TCI23,		"TCI23",		"Total Charing component", NULL },
	{ AMPS_IE_TCI24,		"TCI24",		"Total Charing component", NULL },
	{ AMPS_IE_TCI31,		"TCI31",		"Total Charing component", NULL },
	{ AMPS_IE_TCI32,		"TCI32",		"Total Charing component", NULL },
	{ AMPS_IE_TCI33,		"TCI33",		"Total Charing component", NULL },
	{ AMPS_IE_TCI34,		"TCI34",		"Total Charing component", NULL },
	{ AMPS_IE_TCI41,		"TCI41",		"Total Charing component", NULL },
	{ AMPS_IE_TCI42,		"TCI42",		"Total Charing component", NULL },
	{ AMPS_IE_TCI43,		"TCI43",		"Total Charing component", NULL },
	{ AMPS_IE_TCI44,		"TCI44",		"Total Charing component", NULL },
	{ AMPS_IE_TCI5,			"TCI5",			"Total Charing component", NULL },
	{ AMPS_IE_VMAC,			"VMAC",			"Voice mobile attenuation code field", ie_cmac },
	{ AMPS_IE_WFOM,			"WFOM",			"Wait-for-overhead-message field", ie_yes },
	{ AMPS_IE_NUM,			NULL,			NULL, NULL }
};

static int ie_desc_max_len;

/* decode 7 bit sequence to DCC code
 * return -1 if failed */
static int8_t dcc_decode[128];

/* encode DCC code to 7 bit sequence */
static uint8_t dcc_encode[4] = {
	0x00, 0x1f, 0x63, 0x7c
};

struct amps_table4_def {
	const char *order;
	const char *ordq;
	const char *msg_type;
	const char *function;
} amps_table4_def[] = {
	{ "00001", "000", "00000", "Alert" },
	{ "00001", "001", "00000", "Abbreviated Alert" },
	{ "10001", "000", "00000", "Alert With Info" },
	{ "10001", "001", "00000", "Alert with Info CRI Message" },
	{ "10001", "010", "00000", "Alert with Info TCI Message" },
	{ "10010", "000", "00000", "Flash With Info" },
	{ "10010", "001", "00000", "Flash with Info CRI Message" },
	{ "10010", "010", "00000", "Flash with Info TCI Message" },
	{ "00011", "000", "00000", "Release" },
	{ "00011", "010", "00000", "Release with Digital Control Channel Information" },
	{ "00011", "011", "00000", "Release Complete" },
	{ "00100", "000", "00000", "Reorder" },
	{ "00101", "000", "XXXXX", "Voice Message Waiting" },
	{ "00101", "001", "XXXXX", "SMS Message Waiting" },
	{ "00101", "010", "XXXXX", "G3-Fax Message Waiting" },
	{ "00110", "000", "00000", "Stop Alert" },
	{ "00111", "000", "00000", "Audit" },
	{ "01000", "000", "00000", "Send Called-address" },
	{ "01001", "000", "00000", "Intercept" },
	{ "01010", "000", "00000", "Maintenance" },
	{ "01011", "000", "00000", "Change Power to Power Level 0 (see TIA/EIA-136-270)" },
	{ "01011", "001", "00000", "Change Power to Power Level 1" },
	{ "01011", "010", "00000", "Change Power to Power Level 2" },
	{ "01011", "011", "00000", "Change Power to Power Level 3" },
	{ "01011", "100", "00000", "Change Power to Power Level 4" },
	{ "01011", "101", "00000", "Change Power to Power Level 5" },
	{ "01011", "110", "00000", "Change Power to Power Level 6" },
	{ "01011", "111", "00000", "Change Power to Power Level 7" },
	{ "01100", "000", "00000", "Directed Retry - not last try" },
	{ "01100", "000", "00001", "Directed Retry to Primary Dedicated Control Channels - analog channels only, Authentication disabled, not last try" },
	{ "01100", "000", "00010", "Directed Retry to Primary Dedicated Control Channels - analog channels only, Authentication enabled, not last try" },
	{ "01100", "001", "00000", "Directed Retry - last try" },
	{ "01100", "001", "00001", "Directed Retry to Primary Dedicated Control Channels - analog channels only, Authentication disabled, last try" },
	{ "01111", "001", "00000", "Serial Number Request / Response" },
	{ "01100", "001", "00010", "Directed Retry to Primary Dedicated Control Channels - analog channels only, Authentication enabled, last try" },
	{ "01101", "010", "00000", "Autonomous Registration - Do not make whereabouts known, Authentication Word C not included" },
	{ "01101", "011", "00000", "Autonomous Registration - Make whereabouts known, Authentication Word C not included" },
	{ "01101", "011", "00001", "Autonomous Registration - Power Down, Authentication Word C not included" },
	{ "11000", "010", "00000", "Autonomous Registration - Do not make whereabouts known, Authentication Word C included" },
	{ "11000", "011", "00000", "Autonomous Registration - Make whereabouts known, Authentication Word C included" },
	{ "11000", "011", "00001", "Autonomous Registration - Power Down, Authentication Word C included" },
	{ "11010", "100", "00000", "PCI Query (report) Order/Order Confirmation - Authentication Word C not included" },
	{ "11010", "100", "00001", "PCI Query (report) Order/Order Confirmation - Authentication Word C included" },
	{ "11110", "000", "XXXXX", "local control" },
	/* (Base station initiated messages only - Page and Call Mode Ack messages) */
	{ "00000", "000", "00000", "Page Message (Voice Service)" },
	{ "00000", "000", "00001", "Page Message (Async Data)" },
	{ "00000", "000", "00010", "Page Message (Group 3 Fax)" },
	{ "10000", "000", "XXXX0", "Call Mode Ack: Analog Voice channel permissible" },
	{ "10000", "000", "XXXX1", "Call Mode Ack: Analog Voice channel not permissible" },
	{ "10000", "000", "XXX0X", "Call Mode Ack: Full-rate digital traffic channel not permissible (VSELP)" },
	{ "10000", "000", "XXX1X", "Call Mode Ack: Full -rate digital traffic channel permissible, voice privacy off (VSELP)" },
	{ "10000", "100", "XXX1X", "Call Mode Ack: Full -rate digital traffic channel permissible, voice privacy on (VSELP)" },
	{ "10000", "000", "XX0XX", "Call Mode Ack: Half-rate digital traffic channel not permissible" },
	{ "10000", "000", "XX1XX", "Call Mode Ack: Half-rate digital traffic channel permissible, voice privacy off" },
	{ "10000", "100", "XX1XX", "Call Mode Ack: Half-rate digital traffic channel permissible, voice privacy on" },
	{ "10000", "000", "X0XXX", "Call Mode Ack: Other DQPSK channel not permissible" },
	{ "10000", "000", "X1XXX", "Call Mode Ack: Other DQPSK channel permissible" },
	{ "10000", "000", "0XXXX", "Call Mode Ack: Other voice coding not permissible (see TIA/EIA-136-410)" },
	{ "10000", "000", "1XXXX", "Call Mode Ack: Other voice coding permissible (see TIA/EIA-136-410), voice privacy off" },
	{ "10000", "100", "1XXXX", "Call Mode Ack: Other voice coding permissible (see TIA/EIA-136-410), voice privacy on" },
	{ "10000", "001", "XXXXX", "Call Mode Ack: Extended modulation and framing permissible" },
	/* (Mobile station initiated messages only - Origination and Page Response messages) */
	{ "00000", "000", "XXXX0", "Analog Voice channel acceptable, Authentication Word C not included" },
	{ "00000", "000", "XXXX1", "Analog Voice channel not acceptable, Authentication Word C not included" },
	{ "00000", "000", "XXX0X", "Full-rate digital traffic channel (VSELP) not acceptable, Authentication Word C not included" },
	{ "00000", "000", "XXX1X", "Full-rate digital traffic channel (VSELP) acceptable (voice privacy off), Authentication Word C not included" },
	{ "00000", "100", "XXX1X", "Full-rate digital traffic channel (VSELP) acceptable (voice privacy on), Authentication Word C not included" },
	{ "00000", "000", "XX0XX", "Half-rate digital traffic channel not acceptable, Authentication Word C not included" },
	{ "00000", "000", "XX1XX", "Half-rate digital traffic channel acceptable (voice privacy off), Authentication Word C not included" },
	{ "00000", "100", "XX1XX", "Half-rate digital traffic channel acceptable (voice privacy on), Authentication Word C not included" },
	{ "00000", "000", "X0XXX", "Other DQPSK channel not acceptable, Authentication Word C not included" },
	{ "00000", "000", "X1XXX", "Other DQPSK channel acceptable, Authentication Word C not included" },
	{ "00000", "000", "0XXXX", "Other voice coding not acceptable, (see TIA/EIA-136-410), Authentication Word C not included" },
	{ "00000", "000", "1XXXX", "Other voice coding acceptable (see TIA/EIA-136-410), (voice privacy off), Authentication Word C not included" },
	{ "00000", "100", "1XXXX", "Other voice coding acceptable (see TIA/EIA-136-410), (voice privacy on), Authentication Word C not included" },
	{ "00000", "001", "XXXXX", "Extended Modulation and Framing, Authentication Word C not included" },
	{ "00010", "000", "XXXX0", "Analog Voice Channel (AVC) acceptable, Authentication Word C included" },
	{ "00010", "000", "XXXX1", "AVC not acceptable, Auth. Word C included" },
	{ "00010", "000", "XXX0X", "Full-rate digital traffic channel (VSELP) not acceptable, Authentication Word C included" },
	{ "00010", "000", "XXX1X", "Full-rate digital traffic channel (VSELP) acceptable (voice privacy off), Authentication Word C included" },
	{ "00010", "100", "XXX1X", "Full-rate digital traffic channel (VSELP) acceptable (voice privacy on), Authentication Word C included" },
	{ "00010", "000", "XX0XX", "Half-rate digital traffic channel not acceptable, Authentication Word C included" },
	{ "00010", "000", "XX1XX", "Half-rate digital traffic channel acceptable (voice privacy off), Authentication Word C included" },
	{ "00010", "100", "XX1XX", "Half-rate digital traffic channel acceptable (voice privacy on), Authentication Word C included" },
	/* (Mobile station initiated messages only - Origination and Page Response messages) */
	{ "00010", "000", "X0XXX", "Other DQPSK channel not acceptable, Authentication Word C included" },
	{ "00010", "000", "X1XXX", "Other DQPSK channel acceptable, Authentication Word C included" },
	{ "00010", "000", "0XXXX", "Other voice coding not acceptable, (see TIA/EIA-136-410), Authentication Word C included" },
	{ "00010", "000", "1XXXX", "Other voice coding acceptable (see TIA/EIA-136-410), (voice privacy off), Authentication Word C included" },
	{ "00010", "100", "1XXXX", "Other voice coding acceptable (see TIA/EIA-136-410), (voice privacy on), Authentication Word C included" },
	{ "00010", "001", "XXXXX", "Extended Modulation and Framing, Authentication Word C included" },
	/* (Mobile station initiated messages only - Origination with Service and Page Response with Service messages) */
	{ "11101", "000", "XXX0X", "Full-rate DTC not acceptable, Authentication Word C not included" },
	{ "11101", "000", "XXX1X", "Full-rate DTC acceptable, Authentication Word C not included" },
	{ "11101", "000", "XX0XX", "Half-rate DTC not acceptable, Authentication Word C not included" },
	{ "11101", "000", "XX1XX", "Half-rate DTC acceptable, Authentication Word C not included" },
	{ "11101", "000", "X0XXX", "Other DQPSK channel not acceptable, Authentication Word C not included" },
	{ "11101", "000", "X1XXX", "Other DQPSK channel acceptable, Authentication Word C not included" },
	{ "11101", "000", "0XXXX", "Other voice coding not acceptable, Authentication Word C not included" },
	{ "11101", "000", "1XXXX", "Other voice coding acceptable, Authentication Word C not included" },
	{ "11101", "001", "XXXXX", "Extended Modulation and Framing, Authentication Word C not included" },
	{ "11101", "010", "XXX0X", "Double Full-Rate or Full-Rate Digital Traffic Channel not acceptable, Authentication Word C not included" },
	{ "11101", "010", "XXX1X", "Double Full-Rate or Full-Rate Digital Traffic Channel acceptable - Double-Rate Preferred, Authentication Word C not included" },
	{ "11101", "010", "XX0XX", "Triple Full-Rate, Double Full-Rate, or Full-Rate Digital Traffic Channel not acceptable, Authentication Word C not included" },
	{ "11101", "010", "XX1XX", "Triple Full-Rate, Double Full-Rate, or Full-Rate Digital Traffic Channel acceptable - Triple Rate Preferred, Authentication Word C not included" },
	{ "11101", "010", "X0XXX", "Double Full-Rate Digital Traffic Channel not acceptable, Authentication Word C not included" },
	{ "11101", "010", "X1XXX", "Double Full-Rate Digital Traffic Channel acceptable, Authentication Word C not included" },
	{ "11101", "010", "0XXXX", "Triple Full-Rate Digital Traffic Channel not acceptable, Authentication Word C not included" },
	{ "11101", "010", "1XXXX", "Triple Full-Rate Digital Traffic Channel acceptable, Authentication Word C not included" },
	{ "11111", "000", "XXX0X", "Full-rate DTC not acceptable, Authentication Word C included" },
	/* (Mobile station initiated messages only - Origination with Service and Page Response with Service messages) */
	{ "11111", "000", "XXX1X", "Full-rate DTC acceptable, Authentication Word C included" },
	{ "11111", "000", "XX0XX", "Half-rate DTC not acceptable, Authentication Word C included" },
	{ "11111", "000", "XX1XX", "Half-rate DTC acceptable, Authentication Word C included" },
	{ "11111", "000", "X0XXX", "Other DQPSK channel not acceptable, Authentication Word C included" },
	{ "11111", "000", "X1XXX", "Other DQPSK channel acceptable, Authentication Word C included" },
	{ "11111", "000", "0XXXX", "Other voice coding not acceptable, Authentication Word C included" },
	{ "11111", "000", "1XXXX", "Other voice coding acceptable, Authentication Word C included" },
	{ "11111", "001", "XXXXX", "Extended Modulation and Framing, Authentication Word C included" },
	{ "11111", "010", "XXX0X", "Double Full-Rate or Full-Rate Digital Traffic Channel not acceptable, Authentication Word C included" },
	{ "11111", "010", "XXX1X", "Double Full-Rate Digital Traffic Channel acceptable - Double-Rate Preferred, Authentication Word C included" },
	{ "11111", "010", "XX0XX", "Triple Full-Rate, Double Full-Rate, or Full-Rate Digital Traffic Channel not acceptable, Authentication Word C included" },
	{ "11111", "010", "XX1XX", "Triple Full-Rate, Double Full-Rate, or Full-Rate Digital Traffic Channel acceptable - Triple Rate Preferred, Authentication Word C included" },
	{ "11111", "010", "X0XXX", "Double Full-Rate Digital Traffic Channel not acceptable, Authentication Word C included" },
	{ "11111", "010", "X1XXX", "Double Full-Rate Digital Traffic Channel acceptable, Authentication Word C included" },
	{ "11111", "010", "0XXXX", "Triple Full-Rate Digital Traffic Channel not acceptable, Authentication Word C included" },
	{ "11111", "010", "1XXXX", "Triple Full-Rate Digital Traffic Channel acceptable, Authentication Word C included" },
	/* (Base station initiated messages only - Initial Traffic Channel Designation message) */
	{ "01110", "000", "00001", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 1, full-rate (VSELP)" },
	{ "01110", "000", "01001", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 1, half-rate" },
	{ "01110", "000", "00010", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 2, full-rate (VSELP)" },
	{ "01110", "000", "01010", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 2 , half-rate" },
	{ "01110", "000", "00011", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 3, full-rate (VSELP)" },
	{ "01110", "000", "01011", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 3, half-rate" },
	{ "01110", "000", "01100", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 4, half-rate" },
	{ "01110", "000", "01101", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 5, half-rate" },
	{ "01110", "000", "01110", "DTC Assignment for TIA/EIA 627 Minimum Dual-Mode: Assigned to timeslot 6, half-rate" },
	{ "01110", "010", "00001", "DTC Assignment for IS-136: Assigned to timeslot 1, full-rate (VSELP)" },
	{ "01110", "010", "01001", "DTC Assignment for IS-136: Assigned to timeslot 1, half-rate" },
	{ "01110", "010", "00010", "DTC Assignment for IS-136: Assigned to timeslot 2, full-rate (VSELP)" },
	{ "01110", "010", "01010", "DTC Assignment for IS-136: Assigned to timeslot 2 , half-rate" },
	{ "01110", "010", "00011", "DTC Assignment for IS-136: Assigned to timeslot 3, full-rate (VSELP)" },
	{ "01110", "010", "01011", "DTC Assignment for IS-136: Assigned to timeslot 3, half-rate" },
	{ "01110", "010", "01100", "DTC Assignment for IS-136: Assigned to timeslot 4, half-rate" },
	{ "01110", "010", "01101", "DTC Assignment for IS-136: Assigned to timeslot 5, half-rate" },
	{ "01110", "010", "01110", "DTC Assignment for IS-136: Assigned to timeslot 6, half-rate" },
	/* (Base station initiated messages only - Initial Traffic Channel Designation message) */
	{ "01110", "010", "10001", "DTC Assignment for TIA/EIA-136: Assigned to timeslot 1, full-rate (see TIA/EIA-136-410)" },
	{ "01110", "010", "10010", "DTC Assignment for TIA/EIA-136: Assigned to timeslot 2, full-rate (see TIA/EIA-136-410)" },
	{ "01110", "010", "10011", "DTC Assignment for TIA/EIA-136: Assigned to timeslot 3, full-rate (see TIA/EIA-136-410)" },
	{ "01110", "100", "00001", "DTC Assignment for TIA/EIA-136: Assigned to timeslot 1, full-rate (Fax/Data)" },
	{ "01110", "100", "00010", "DTC Assignment for TIA/EIA-136: Assigned to timeslot 2, full-rate (Fax/Data)" },
	{ "01110", "100", "00011", "DTC Assignment for TIA/EIA-136: Assigned to timeslot 3, full-rate (Fax/Data)" },
	{ "01110", "100", "00100", "DTC Assignment for TIA/EIA-136: Assigned to timeslots 1 & 2, double rate (Fax/Data)" },
	{ "01110", "100", "00101", "DTC Assignment for TIA/EIA-136: Assigned to timeslots 1 & 3, double rate (Fax/Data)" },
	{ "01110", "100", "00110", "DTC Assignment for TIA/EIA-136: Assigned to timeslots 2 & 3, double rate (Fax/Data)" },
	{ "01110", "100", "00111", "DTC Assignment for TIA/EIA-136: Assigned to timeslots 1, 2 & 3, triple rate (Fax/Data)" },
	{ "01110", "001", "XXXXX", "Digital Traffic Channel Assignment with Extended Modulation and Framing" },
	{ "11010", "000", "00000", "Analog Voice Channel Assignment" },
	/* (Base station initiated messages only - Mobile Station Authentication and Privacy) */
	{ "01111", "000", "00000", "Parameter Update Order" },
	{ "10011", "000", "00000", "Base Station Challenge Order Confirmation" },
	{ "10100", "000", "00000", "Unique Challenge Order" },
	{ "10101", "000", "00000", "SSD Update Order" },
	{ "10110", "000", "00000", "Disable DTMF Order" },
	{ "10111", "000", "00000", "Message Encryption Mode Order with disable indication" },
	{ "10111", "001", "00000", "Message Encryption Mode Order with enable indication" },
	/* (Mobile station initiated messages only - Mobile Station Authentication and Privacy) */
	{ "01111", "000", "00000", "Parameter Update Order/Confirmation" },
	{ "10011", "000", "00000", "Base Station Challenge Order" },
	{ "10100", "000", "00000", "Unique Challenge Order Confirmation" },
	{ "10101", "000", "00000", "SSD Update Order Confirmation with failure indication" },
	{ "10101", "001", "00000", "SSD Update Order Confirmation with success indication" },
	{ "10111", "000", "00000", "Message Encryption Mode Order Confirmation with disable indication" },
	{ "10111", "001", "00000", "Message Encryption Mode Order Confirmation with enable indication" },
	{ NULL, NULL, NULL, NULL }
};

struct amps_table4 {
	uint8_t order;
	uint8_t ordq;
	uint8_t msg_type;
	uint8_t msg_type_mask;
	const char *function;
} *amps_table4;

static void gen_table4(void)
{
	uint8_t value, mask;
	int i, j;

	/* count entries including last one */
	for (i = 0; amps_table4_def[i].function; i++)
		;
	amps_table4 = calloc(i + 1, sizeof(struct amps_table4));
	if (!amps_table4) {
		fprintf(stderr, "No mem!\n");
		abort();
	}


	for (i = 0; amps_table4_def[i].function; i++) {
		if (strlen(amps_table4_def[i].order) != 5
		 || strlen(amps_table4_def[i].ordq) != 3
		 || strlen(amps_table4_def[i].msg_type) != 5) {
			fprintf(stderr, "Error in table definition entry %d: Wrong length!\n", i);
			abort();
		}
		value = 0;
		for (j = 0; j < 5; j++) {
			if (amps_table4_def[i].order[j] == '1')
				value = (value << 1) | 1;
			else if (amps_table4_def[i].order[j] != '0') {
				fprintf(stderr, "Error in table definition entry %d: Wrong digit!\n", i);
				abort();
			} else
				value = (value << 1);
		}
		amps_table4[i].order = value;
		value = 0;
		for (j = 0; j < 3; j++) {
			if (amps_table4_def[i].ordq[j] == '1')
				value = (value << 1) | 1;
			else if (amps_table4_def[i].ordq[j] != '0') {
				fprintf(stderr, "Error in table definition entry %d: Wrong digit!\n", i);
				abort();
			} else
				value = (value << 1);
		}
		amps_table4[i].ordq = value;
		value = 0;
		mask = 0;
		for (j = 0; j < 5; j++) {
			if (amps_table4_def[i].msg_type[j] == '1') {
				value = (value << 1) | 1;
				mask = (mask << 1) | 1;
			} else if (amps_table4_def[i].msg_type[j] == '0') {
				value = (value << 1);
				mask = (mask << 1) | 1;
			} else if (amps_table4_def[i].msg_type[j] == 'X') {
				value = (value << 1);
				mask = (mask << 1);
			} else {
				fprintf(stderr, "Error in table definition entry %d: Wrong digit!\n", i);
				abort();
			}
		}
		amps_table4[i].msg_type = value;
		amps_table4[i].msg_type_mask = mask;
		amps_table4[i].function = amps_table4_def[i].function;
	}
}

const char *amps_table4_name(uint8_t msg_type, uint8_t ordq, uint8_t order)
{
	static char mwi_buffer[80];
	int i;

	/* Special handling for Message Waiting order (Order 5) to show actual count */
	if (order == 5 && ordq <= 2) {
		const char *type_name;
		switch (ordq) {
		case 0: type_name = "Voice"; break;
		case 1: type_name = "SMS"; break;
		case 2: type_name = "G3-Fax"; break;
		default: type_name = "Unknown"; break;
		}
		if (msg_type == 0)
			snprintf(mwi_buffer, sizeof(mwi_buffer), "%s Message Waiting: No messages (cleared)", type_name);
		else if (msg_type == 31)
			snprintf(mwi_buffer, sizeof(mwi_buffer), "%s Message Waiting: Unknown number of messages", type_name);
		else
			snprintf(mwi_buffer, sizeof(mwi_buffer), "%s Message Waiting: %d message%s", type_name, msg_type, msg_type == 1 ? "" : "s");
		return mwi_buffer;
	}

	for (i = 0; amps_table4[i].function; i++) {
//printf("c %d %d %d with %d %d %d\n", msg_type, ordq, order, amps_table4[i].msg_type, amps_table4[i].ordq, amps_table4[i].order);
		if (amps_table4[i].order == order
		 && amps_table4[i].ordq == ordq
		 && amps_table4[i].msg_type == (msg_type & amps_table4[i].msg_type_mask))
		 	return amps_table4[i].function;
	}
	return ("Unknown message type");
}

void init_frame(void)
{
	struct def_message_set **ms;
	struct def_word **w;
	struct def_ie *ie;
	struct amps_ie_desc *ied;
	int num_bits, bits;
	int i, j;
	uint8_t dcc;

	ie_desc_max_len = 0;
	for (i = 0; amps_ie_desc[i].name; i++) {
		if ((int)strlen(amps_ie_desc[i].name) > ie_desc_max_len)
			ie_desc_max_len = strlen(amps_ie_desc[i].name);
		if (i != (int)amps_ie_desc[i].ie) {
			fprintf(stderr, "IEs #%d in amps_ie_desc is different from definitions AMPS_IE_xxx (%d), please fix!\n", i, amps_ie_desc[i].ie);
			abort();
		}
		if (amps_ie_desc[i + 1].name) {
			if (strcmp(amps_ie_desc[i + 1].name, amps_ie_desc[i].name) <= 0) {
				fprintf(stderr, "IE '%s' in amps_ie_desc list is not greater (unsorted) or is equal to '%s', please fix!\n", amps_ie_desc[i + 1].name, amps_ie_desc[i].name);
				abort();
			}
		}
	}
	if (i != AMPS_IE_NUM) {
		fprintf(stderr, "number of IEs in amps_ie_desc (%d) is different from number of definitions AMPS_IE_xxx (%d), please fix!\n", i, AMPS_IE_NUM);
		abort();
	}

	/* check message words */
	for (ms = amps_message_sets; *ms; ms++) {
//		printf("Checking message set '%s'\n", (*ms)->name);
		num_bits = (*ms)->num_bits;
		for (w = (*ms)->word; *w; w++) {
//			printf("   Checking message word '%s'\n", (*w)->name);
			bits = 0;
			for (ie = (*w)->ie; ie->name; ie++) {
//				printf("      Checking ie '%s'\n", ie->name);
				bits += ie->bits;
				if (strchr(ie->name, '=')) {
					fprintf(stderr, "IE name '%s' in '%s' has '=' character, please fix!\n", ie->name, (*w)->name);
					abort();
				}
				for (i = 0, ied = amps_ie_desc; ied->name; i++, ied++) {
					if (!strcmp(ied->name, ie->name))
						break;
				}
				if (!ied->name) {
					fprintf(stderr, "IE name '%s' not found in amps_ie_desc list, please fix!\n", ie->name);
					abort();
				}
				ie->ie = i;
			}
			if (bits != num_bits) {
				fprintf(stderr, "Bits in '%s' is not %d, please fix!\n", (*w)->name, num_bits);
				abort();
			}
		}
	}

	/* generate DCC decoding table */
	for (i = 0; i < 128; i++)
		dcc_decode[i] = -1;
	for (i = 0; i < 4; i++) {
		dcc = dcc_encode[i];
		dcc_decode[dcc] = i;
		/* one bit errors */
		for (j = 0; j < 7; j++)
			dcc_decode[dcc ^ (1 << j)] = i;
	}

	/* generate table 4 */
	gen_table4();
}


/*
 * encode and decode words
 */

static uint64_t amps_encode_word(frame_t *frame, struct def_word *w, int debug)
{
	uint64_t word, value;
	char spaces[ie_desc_max_len + 1];
	int sum_bits, bits;
	int i, t4 = 0;

#ifdef DEBUG_ALL_MESSAGES
	debug = 1;
#endif

	memset(spaces, ' ', ie_desc_max_len);
	spaces[ie_desc_max_len] = '\0';

	/* sum of bits */
	sum_bits = 0;
	for (i = 0; w->ie[i].name; i++)
		sum_bits += w->ie[i].bits;

	if (debug)
		LOGP(DFRAME, LOGL_INFO, "Transmit: %s\n", w->name);
	word = 0;
	for (i = 0; w->ie[i].name; i++) {
		bits = w->ie[i].bits;
		if (w->ie[i].name[0] == 'P' && w->ie[i].name[1] == '\0')
			value = encode_bch_binary(word, sum_bits - bits);
		else
			value = frame->ie[w->ie[i].ie];
		word = (word << bits) | (value & cut_bits[bits]);
		if (debug) {
			if (amps_ie_desc[w->ie[i].ie].decoder)
				LOGP(DFRAME, LOGL_DEBUG, " %s%s: %" PRIu64 " = %s  (%s)\n", spaces + strlen(w->ie[i].name), w->ie[i].name, value, amps_ie_desc[w->ie[i].ie].decoder(value), amps_ie_desc[w->ie[i].ie].desc);
			else
				LOGP(DFRAME, LOGL_DEBUG, " %s%s: %" PRIu64 "  (%s)\n", spaces + strlen(w->ie[i].name), w->ie[i].name, value, amps_ie_desc[w->ie[i].ie].desc);
		}
		/* show result for 3 IEs of table 4 */
		if (w->ie[i].ie == AMPS_IE_LOCAL_MSG_TYPE || w->ie[i].ie == AMPS_IE_ORDQ || w->ie[i].ie == AMPS_IE_ORDER)
			t4++;
		if (t4 == 3) {
			t4 = 0;
			if (debug)
				LOGP(DFRAME, LOGL_DEBUG, " %s--> %s\n", spaces, amps_table4_name(frame->ie[AMPS_IE_LOCAL_MSG_TYPE], frame->ie[AMPS_IE_ORDQ], frame->ie[AMPS_IE_ORDER]));
		}
	}

	return word;
}

static uint64_t amps_encode_control_filler(amps_t *amps, uint8_t dcc, uint8_t cmac, uint8_t sdcc1, uint8_t sdcc2, uint8_t wfom, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_010111] = 23;
	frame.ie[AMPS_IE_CMAC] = cmac;
	frame.ie[AMPS_IE_SDCC1] = sdcc1;
	frame.ie[AMPS_IE_11] = 3;
	frame.ie[AMPS_IE_SDCC2] = sdcc2;
	frame.ie[AMPS_IE_1] = 1;
	frame.ie[AMPS_IE_WFOM] = wfom;
	if (amps->sender.loopback) {
		frame.ie[AMPS_IE_1111] = amps->when_count;
		amps->when_transmitted[amps->when_count] = get_time();
		amps->when_count = (amps->when_count + 1) & 0xf;
	} else
		frame.ie[AMPS_IE_1111] = 15;
	frame.ie[AMPS_IE_OHD] = 1;
	return amps_encode_word(&frame, &control_filler, debug);
}

uint64_t amps_encode_word1_system(uint8_t dcc, uint16_t sid1, uint8_t ep, uint8_t auth, uint8_t pci, uint8_t nawc, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_SID1] = sid1;
	frame.ie[AMPS_IE_EP] = ep;
	frame.ie[AMPS_IE_AUTH] = auth;
	frame.ie[AMPS_IE_PCI] = pci;
	frame.ie[AMPS_IE_NAWC] = nawc;
	frame.ie[AMPS_IE_OHD] = 6;
	return amps_encode_word(&frame, &amps_word1_system_parameter_overhead, debug);
}

uint64_t tacs_encode_word1_system(uint8_t dcc, uint16_t aid1, uint8_t ep, uint8_t auth, uint8_t pci, uint8_t nawc, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_AID1] = aid1;
	frame.ie[AMPS_IE_EP] = ep;
	frame.ie[AMPS_IE_AUTH] = auth;
	frame.ie[AMPS_IE_PCI] = pci;
	frame.ie[AMPS_IE_NAWC] = nawc;
	frame.ie[AMPS_IE_OHD] = 6;
	return amps_encode_word(&frame, &tacs_word1_system_parameter_overhead, debug);
}

uint64_t amps_encode_word2_system(uint8_t dcc, uint8_t s, uint8_t e, uint8_t regh, uint8_t regr, uint8_t dtx, uint8_t n_1, uint8_t rcf, uint8_t cpa, uint8_t cmax_1, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_S] = s;
	frame.ie[AMPS_IE_E] = e;
	frame.ie[AMPS_IE_REGH] = regh;
	frame.ie[AMPS_IE_REGR] = regr;
	frame.ie[AMPS_IE_DTX_Support] = dtx;
	frame.ie[AMPS_IE_N_1] = n_1;
	frame.ie[AMPS_IE_RCF] = rcf;
	frame.ie[AMPS_IE_CPA] = cpa;
	frame.ie[AMPS_IE_CMAX_1] = cmax_1;
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 7;
	return amps_encode_word(&frame, &word2_system_parameter_overhead, debug);
}

uint64_t amps_encode_registration_id(uint8_t dcc, uint32_t regid, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_REGID] = regid;
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 0;
	return amps_encode_word(&frame, &registration_id, debug);
}

uint64_t amps_encode_registration_increment(uint8_t dcc, uint16_t regincr, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_ACT] = 2;
	frame.ie[AMPS_IE_REGINCR] = regincr;
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 4;
	return amps_encode_word(&frame, &registration_increment_global_action, debug);
}

uint64_t amps_encode_location_area(uint8_t dcc, uint8_t pureg, uint8_t pdreg, uint8_t lreg, uint16_t locaid, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_ACT] = 3;
	frame.ie[AMPS_IE_PUREG] = pureg;
	frame.ie[AMPS_IE_PDREG] = pdreg;
	frame.ie[AMPS_IE_LREG] = lreg;
	frame.ie[AMPS_IE_LOCAID] = locaid;
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 4;
	return amps_encode_word(&frame, &location_area_global_action, debug);
}

uint64_t amps_encode_new_access_channel_set(uint8_t dcc, uint16_t newacc, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_ACT] = 6;
	frame.ie[AMPS_IE_NEWACC] = newacc;
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 4;
	return amps_encode_word(&frame, &new_access_channel_set_global_action, debug);
}

uint64_t amps_encode_overload_control(uint8_t dcc, uint8_t *olc, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_ACT] = 8;
	frame.ie[AMPS_IE_OLC_0] = olc[0];
	frame.ie[AMPS_IE_OLC_1] = olc[1];
	frame.ie[AMPS_IE_OLC_2] = olc[2];
	frame.ie[AMPS_IE_OLC_3] = olc[3];
	frame.ie[AMPS_IE_OLC_4] = olc[4];
	frame.ie[AMPS_IE_OLC_5] = olc[5];
	frame.ie[AMPS_IE_OLC_6] = olc[6];
	frame.ie[AMPS_IE_OLC_7] = olc[7];
	frame.ie[AMPS_IE_OLC_8] = olc[8];
	frame.ie[AMPS_IE_OLC_9] = olc[9];
	frame.ie[AMPS_IE_OLC_10] = olc[10];
	frame.ie[AMPS_IE_OLC_11] = olc[11];
	frame.ie[AMPS_IE_OLC_12] = olc[12];
	frame.ie[AMPS_IE_OLC_13] = olc[13];
	frame.ie[AMPS_IE_OLC_14] = olc[14];
	frame.ie[AMPS_IE_OLC_15] = olc[15];
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 4;
	return amps_encode_word(&frame, &overload_control_global_action, debug);
}

uint64_t amps_encode_access_type(uint8_t dcc, uint8_t bis, uint8_t pci_home, uint8_t pci_roam, uint8_t bspc, uint8_t bscap, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_ACT] = 9;
	frame.ie[AMPS_IE_BIS] = bis;
	frame.ie[AMPS_IE_PCI_HOME] = pci_home;
	frame.ie[AMPS_IE_PCI_ROAM] = pci_roam;
	frame.ie[AMPS_IE_BSPC] = bspc;
	frame.ie[AMPS_IE_BSCAP] = bscap;
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 4;
	return amps_encode_word(&frame, &access_type_parameters_global_action, debug);
}

uint64_t amps_encode_access_attempt(uint8_t dcc, uint8_t maxbusy_pgr, uint8_t maxsztr_pgr, uint8_t maxbusy_other, uint8_t maxsztr_other, uint8_t end, int debug)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 3;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_ACT] = 10;
	frame.ie[AMPS_IE_MAXBUSY_PGR] = maxbusy_pgr;
	frame.ie[AMPS_IE_MAXSZTR_PGR] = maxsztr_pgr;
	frame.ie[AMPS_IE_MAXBUSY_OTHER] = maxbusy_other;
	frame.ie[AMPS_IE_MAXSZTR_OTHER] = maxsztr_other;
	frame.ie[AMPS_IE_END] = end;
	frame.ie[AMPS_IE_OHD] = 4;
	return amps_encode_word(&frame, &access_attempt_parameters_global_action, debug);
}

static uint64_t amps_encode_word1_abbreviated_address_word(uint8_t dcc, uint32_t min1, int multiple)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	if (multiple)
		frame.ie[AMPS_IE_T1T2] = 1;
	else
		frame.ie[AMPS_IE_T1T2] = 0;
	frame.ie[AMPS_IE_DCC] = dcc;
	frame.ie[AMPS_IE_MIN1] = min1;
	return amps_encode_word(&frame, &word1_abbreviated_address_word, 1);
}

static uint64_t amps_encode_word2_extended_address_word_a(uint16_t min2, uint8_t msg_type, uint8_t ordq, uint8_t order)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 2;
	frame.ie[AMPS_IE_SCC] = 3;
	frame.ie[AMPS_IE_MIN2] = min2;
	frame.ie[AMPS_IE_EF] = 0;
	frame.ie[AMPS_IE_LOCAL_MSG_TYPE] = msg_type;
	frame.ie[AMPS_IE_ORDQ] = ordq;
	frame.ie[AMPS_IE_ORDER] = order;
	return amps_encode_word(&frame, &word2_extended_address_word_a, 1);
}

static uint64_t amps_encode_word2_extended_address_word_b(uint8_t scc, uint16_t min2, uint8_t vmac, uint16_t chan)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 2;
	frame.ie[AMPS_IE_SCC] = scc;
	frame.ie[AMPS_IE_MIN2] = min2;
	frame.ie[AMPS_IE_VMAC] = vmac;
	frame.ie[AMPS_IE_CHAN] = chan;
	return amps_encode_word(&frame, &word2_extended_address_word_b, 1);
}

/*
 * Encode Word 3 - First Directed-Retry Word
 * Per TIA/EIA-553-A Section 3.7.1.1:
 * T1T2=10 (2 bits), SCC=11 (2 bits), CHANPOS(7), CHANPOS(7), CHANPOS(7), RSVD=000 (3 bits), P(12)
 *
 * Manual encoding because the word definition has multiple CHANPOS fields
 * that can't be handled by the generic encoder.
 *
 * Parameters:
 *   scc: SAT Color Code (0-3)
 *   chanpos1, chanpos2, chanpos3: Channel positions (0-127, 0 = unused)
 */
static uint64_t amps_encode_word3_directed_retry(uint8_t scc, uint8_t chanpos1, uint8_t chanpos2, uint8_t chanpos3)
{
	uint64_t word;
	
	/* Build word manually: T1T2(2) + SCC(2) + CHANPOS(7) + CHANPOS(7) + CHANPOS(7) + RSVD(3) = 28 bits */
	word = 0;
	word = (word << 2) | (2 & 0x3);           /* T1T2 = 10 */
	word = (word << 2) | (scc & 0x3);         /* SCC */
	word = (word << 7) | (chanpos1 & 0x7f);   /* First CHANPOS */
	word = (word << 7) | (chanpos2 & 0x7f);   /* Second CHANPOS */
	word = (word << 7) | (chanpos3 & 0x7f);   /* Third CHANPOS */
	word = (word << 3) | 0;                   /* RSVD = 000 */
	
	/* Add BCH parity (12 bits) */
	word = (word << 12) | encode_bch_binary(word, 28);
	
	LOGP(DFRAME, LOGL_INFO, "Transmit: Word 3 - First Directed-Retry Word\n");
	LOGP(DFRAME, LOGL_DEBUG, "          T1T2: 2 (multiple word)\n");
	LOGP(DFRAME, LOGL_DEBUG, "           SCC: %d\n", scc);
	LOGP(DFRAME, LOGL_DEBUG, "       CHANPOS: %d, %d, %d\n", chanpos1, chanpos2, chanpos3);
	
	return word;
}

/*
 * Encode Word 4 - Second Directed-Retry Word
 * Per TIA/EIA-553-A Section 3.7.1.1:
 * T1T2=10 (2 bits), SCC=11 (2 bits), CHANPOS(7), CHANPOS(7), CHANPOS(7), RSVD=000 (3 bits), P(12)
 *
 * Parameters:
 *   scc: SAT Color Code (0-3)
 *   chanpos4, chanpos5, chanpos6: Channel positions (0-127, 0 = unused)
 */
static uint64_t amps_encode_word4_directed_retry(uint8_t scc, uint8_t chanpos4, uint8_t chanpos5, uint8_t chanpos6)
{
	uint64_t word;
	
	/* Build word manually: T1T2(2) + SCC(2) + CHANPOS(7) + CHANPOS(7) + CHANPOS(7) + RSVD(3) = 28 bits */
	word = 0;
	word = (word << 2) | (2 & 0x3);           /* T1T2 = 10 */
	word = (word << 2) | (scc & 0x3);         /* SCC */
	word = (word << 7) | (chanpos4 & 0x7f);   /* Fourth CHANPOS */
	word = (word << 7) | (chanpos5 & 0x7f);   /* Fifth CHANPOS */
	word = (word << 7) | (chanpos6 & 0x7f);   /* Sixth CHANPOS */
	word = (word << 3) | 0;                   /* RSVD = 000 */
	
	/* Add BCH parity (12 bits) */
	word = (word << 12) | encode_bch_binary(word, 28);
	
	LOGP(DFRAME, LOGL_INFO, "Transmit: Word 4 - Second Directed-Retry Word\n");
	LOGP(DFRAME, LOGL_DEBUG, "          T1T2: 2 (multiple word)\n");
	LOGP(DFRAME, LOGL_DEBUG, "           SCC: %d\n", scc);
	LOGP(DFRAME, LOGL_DEBUG, "       CHANPOS: %d, %d, %d\n", chanpos4, chanpos5, chanpos6);
	
	return word;
}

static uint64_t amps_encode_mobile_station_control_message_word1_a(uint8_t pscc, uint8_t msg_type, uint8_t ordq, uint8_t order)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 2;
	frame.ie[AMPS_IE_SCC] = 3;
	frame.ie[AMPS_IE_PSCC] = pscc;
	frame.ie[AMPS_IE_EF] = 0;
	frame.ie[AMPS_IE_LOCAL_MSG_TYPE] = msg_type;
	frame.ie[AMPS_IE_ORDQ] = ordq;
	frame.ie[AMPS_IE_ORDER] = order;
	return amps_encode_word(&frame, &mobile_station_control_message_word1_a, 1);
}

static uint64_t amps_encode_mobile_station_control_message_word1_b(uint8_t scc, uint8_t pscc, uint8_t dtx, uint8_t pvi, uint8_t mem, uint8_t vmac, uint16_t chan)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 2;
	frame.ie[AMPS_IE_SCC] = scc;
	frame.ie[AMPS_IE_PSCC] = pscc;
	frame.ie[AMPS_IE_EF] = 0;
	frame.ie[AMPS_IE_DTX] = dtx;
	frame.ie[AMPS_IE_PVI] = pvi;
	frame.ie[AMPS_IE_MEM] = mem;
	frame.ie[AMPS_IE_VMAC] = vmac;
	frame.ie[AMPS_IE_CHAN] = chan;
	return amps_encode_word(&frame, &mobile_station_control_message_word1_b, 1);
}

static uint64_t amps_encode_word2_first_alert_with_info_word(uint8_t rl_w, uint8_t signal, uint8_t cpn_rl, uint8_t pi, uint8_t si)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_RL_W] = rl_w;
	frame.ie[AMPS_IE_SIGNAL] = signal;
	frame.ie[AMPS_IE_CPN_RL] = cpn_rl;
	frame.ie[AMPS_IE_PI] = pi;
	frame.ie[AMPS_IE_SI] = si;
	return amps_encode_word(&frame, &word2_first_alert_with_info_word, 1);
}

static uint64_t amps_encode_wordn_n_minus_1th_alert_with_info_word(const char *character)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	if (character[0]) {
		frame.ie[AMPS_IE_CHARACTER_1] = character[0];
		if (character[1]) {
			frame.ie[AMPS_IE_CHARACTER_2] = character[1];
			if (character[2])
				frame.ie[AMPS_IE_CHARACTER_3] = character[2];
		}
	}
	return amps_encode_word(&frame, &wordn_n_minus_1th_alert_with_info_word, 1);
}

static uint64_t amps_encode_word2_first_flash_with_info_word(uint8_t rl_w, uint8_t cpn_rl, uint8_t pi, uint8_t si)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_RL_W] = rl_w;
	frame.ie[AMPS_IE_CPN_RL] = cpn_rl;
	frame.ie[AMPS_IE_PI] = pi;
	frame.ie[AMPS_IE_SI] = si;
	return amps_encode_word(&frame, &word2_first_flash_with_info_word, 1);
}

static uint64_t amps_encode_wordn_n_minus_1th_flash_with_info_word(const char *character)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	if (character[0]) {
		frame.ie[AMPS_IE_CHARACTER_1] = character[0];
		if (character[1]) {
			frame.ie[AMPS_IE_CHARACTER_2] = character[1];
			if (character[2])
				frame.ie[AMPS_IE_CHARACTER_3] = character[2];
		}
	}
	return amps_encode_word(&frame, &wordn_n_minus_1th_flash_with_info_word, 1);
}

/* CRI (Charging Rate Indication) encoding functions for ORDQ=1 */

/* Word 2 for CRI - same structure as character message but with RL_W indicating CRI word count */
static uint64_t amps_encode_word2_alert_with_info_cri(uint8_t rl_w, uint8_t signal)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_RL_W] = rl_w;      /* Number of CRI words to follow */
	frame.ie[AMPS_IE_SIGNAL] = signal;  /* Alert signal pitch/cadence */
	frame.ie[AMPS_IE_CPN_RL] = 0;       /* Not used for CRI */
	frame.ie[AMPS_IE_PI] = 0;           /* Not used for CRI */
	frame.ie[AMPS_IE_SI] = 0;           /* Not used for CRI */
	return amps_encode_word(&frame, &word2_first_alert_with_info_word, 1);
}

static uint64_t amps_encode_word2_flash_with_info_cri(uint8_t rl_w)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_RL_W] = rl_w;      /* Number of CRI words to follow */
	frame.ie[AMPS_IE_CPN_RL] = 0;       /* Not used for CRI */
	frame.ie[AMPS_IE_PI] = 0;           /* Not used for CRI */
	frame.ie[AMPS_IE_SI] = 0;           /* Not used for CRI */
	return amps_encode_word(&frame, &word2_first_flash_with_info_word, 1);
}

/* CRI Word 3 (Second CRI Word) - Elements E1 and E2 (partial) */
static uint64_t amps_encode_word3_cri(uint8_t e14, uint8_t e13, uint8_t e12, uint8_t e11,
                                       uint8_t e24, uint8_t e23)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_CRI_E14] = e14;
	frame.ie[AMPS_IE_CRI_E13] = e13;
	frame.ie[AMPS_IE_CRI_E12] = e12;
	frame.ie[AMPS_IE_CRI_E11] = e11;
	frame.ie[AMPS_IE_CRI_E24] = e24;
	frame.ie[AMPS_IE_CRI_E23] = e23;
	return amps_encode_word(&frame, &word3_second_alert_with_info_cri_message_word, 1);
}

/* CRI Word 4 (Third CRI Word) - Elements E2 (partial) and E3 */
static uint64_t amps_encode_word4_cri(uint8_t e22, uint8_t e21, uint8_t e34, uint8_t e33,
                                       uint8_t e32, uint8_t e31)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_CRI_E22] = e22;
	frame.ie[AMPS_IE_CRI_E21] = e21;
	frame.ie[AMPS_IE_CRI_E34] = e34;
	frame.ie[AMPS_IE_CRI_E33] = e33;
	frame.ie[AMPS_IE_CRI_E32] = e32;
	frame.ie[AMPS_IE_CRI_E31] = e31;
	return amps_encode_word(&frame, &word4_third_alert_with_info_cri_message_word, 1);
}

/* CRI Word 5 (Fourth CRI Word) - Elements E4 and E5 (partial) */
static uint64_t amps_encode_word5_cri(uint8_t e44, uint8_t e43, uint8_t e42, uint8_t e41,
                                       uint8_t e54, uint8_t e53)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_CRI_E44] = e44;
	frame.ie[AMPS_IE_CRI_E43] = e43;
	frame.ie[AMPS_IE_CRI_E42] = e42;
	frame.ie[AMPS_IE_CRI_E41] = e41;
	frame.ie[AMPS_IE_CRI_E54] = e54;
	frame.ie[AMPS_IE_CRI_E53] = e53;
	return amps_encode_word(&frame, &word5_alert_with_info_cri_message_word, 1);
}

/* CRI Word 6 (Fifth CRI Word) - Elements E5 (partial) and E6 */
static uint64_t amps_encode_word6_cri(uint8_t e52, uint8_t e51, uint8_t e64, uint8_t e63,
                                       uint8_t e62, uint8_t e61)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_CRI_E52] = e52;
	frame.ie[AMPS_IE_CRI_E51] = e51;
	frame.ie[AMPS_IE_CRI_E64] = e64;
	frame.ie[AMPS_IE_CRI_E63] = e63;
	frame.ie[AMPS_IE_CRI_E62] = e62;
	frame.ie[AMPS_IE_CRI_E61] = e61;
	return amps_encode_word(&frame, &word6_fifth_alert_with_info_cri_mesage_word, 1);
}

/* CRI Word 7 (Sixth CRI Word) - Elements E7 and E8 (partial) */
static uint64_t amps_encode_word7_cri(uint8_t e74, uint8_t e73, uint8_t e72, uint8_t e71,
                                       uint8_t e84, uint8_t e83)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_CRI_E74] = e74;
	frame.ie[AMPS_IE_CRI_E73] = e73;
	frame.ie[AMPS_IE_CRI_E72] = e72;
	frame.ie[AMPS_IE_CRI_E71] = e71;
	frame.ie[AMPS_IE_CRI_E84] = e84;
	frame.ie[AMPS_IE_CRI_E83] = e83;
	return amps_encode_word(&frame, &word7_sixth_alert_with_info_cri_mesage_word, 1);
}

/* CRI Word 8 (Seventh CRI Word) - Element E8 (partial) */
static uint64_t amps_encode_word8_cri(uint8_t e82, uint8_t e81)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_CRI_E82] = e82;
	frame.ie[AMPS_IE_CRI_E81] = e81;
	return amps_encode_word(&frame, &word8_seventh_alert_with_info_cri_mesage_word, 1);
}

/* TCI (Total Charging Information) encoding functions for ORDQ=2 */

/* Word 2 for TCI - same structure as character message */
static uint64_t amps_encode_word2_alert_with_info_tci(uint8_t rl_w, uint8_t signal)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_RL_W] = rl_w;      /* Number of TCI words to follow */
	frame.ie[AMPS_IE_SIGNAL] = signal;  /* Alert signal pitch/cadence */
	frame.ie[AMPS_IE_CPN_RL] = 0;       /* Not used for TCI */
	frame.ie[AMPS_IE_PI] = 0;           /* Not used for TCI */
	frame.ie[AMPS_IE_SI] = 0;           /* Not used for TCI */
	return amps_encode_word(&frame, &word2_first_alert_with_info_word, 1);
}

static uint64_t amps_encode_word2_flash_with_info_tci(uint8_t rl_w)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_RL_W] = rl_w;      /* Number of TCI words to follow */
	frame.ie[AMPS_IE_CPN_RL] = 0;       /* Not used for TCI */
	frame.ie[AMPS_IE_PI] = 0;           /* Not used for TCI */
	frame.ie[AMPS_IE_SI] = 0;           /* Not used for TCI */
	return amps_encode_word(&frame, &word2_first_flash_with_info_word, 1);
}

/* TCI Word 3 (Second TCI Word) - Row 1 and Row 2 (partial) */
static uint64_t amps_encode_word3_tci(uint8_t tci1, uint8_t tci5, uint8_t tci24, uint8_t tci23,
                                       uint8_t tci22, uint8_t tci21)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_TCI1] = tci1;      /* 7 bits: Row 1 digits 1-4 (partial) */
	frame.ie[AMPS_IE_TCI5] = tci5;      /* 1 bit: Row 1 digit 4 (partial) */
	frame.ie[AMPS_IE_TCI24] = tci24;    /* Row 2 digit 4 */
	frame.ie[AMPS_IE_TCI23] = tci23;    /* Row 2 digit 3 */
	frame.ie[AMPS_IE_TCI22] = tci22;    /* Row 2 digit 2 */
	frame.ie[AMPS_IE_TCI21] = tci21;    /* Row 2 digit 1 */
	return amps_encode_word(&frame, &word3_second_alert_with_info_tci_message_word, 1);
}

/* TCI Word 4 (Third TCI Word) - Row 3 and Row 4 (partial) */
static uint64_t amps_encode_word4_tci(uint8_t tci34, uint8_t tci33, uint8_t tci32, uint8_t tci31,
                                       uint8_t tci44, uint8_t tci43)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_TCI34] = tci34;    /* Row 3 digit 4 */
	frame.ie[AMPS_IE_TCI33] = tci33;    /* Row 3 digit 3 */
	frame.ie[AMPS_IE_TCI32] = tci32;    /* Row 3 digit 2 */
	frame.ie[AMPS_IE_TCI31] = tci31;    /* Row 3 digit 1 */
	frame.ie[AMPS_IE_TCI44] = tci44;    /* Row 4 digit 4 */
	frame.ie[AMPS_IE_TCI43] = tci43;    /* Row 4 digit 3 */
	return amps_encode_word(&frame, &word4_third_alert_with_info_tci_message_word, 1);
}

/* TCI Word 5 (Fourth TCI Word) - Row 4 (partial) */
static uint64_t amps_encode_word5_tci(uint8_t tci42, uint8_t tci41)
{
	frame_t frame;

	memset(&frame, 0, sizeof(frame));
	frame.ie[AMPS_IE_T1T2] = 1;
	frame.ie[AMPS_IE_TCI42] = tci42;    /* Row 4 digit 2 */
	frame.ie[AMPS_IE_TCI41] = tci41;    /* Row 4 digit 1 */
	return amps_encode_word(&frame, &word5_alert_with_info_tci_message_word, 1);
}

/* decoder function of a word */
static frame_t *amps_decode_word(uint64_t word, struct def_word *w)
{
	static frame_t frame;
	char spaces[ie_desc_max_len + 1];
	int bits_left, bits;
	uint64_t value;
	int i, t4 = 0;

	memset(&frame, 0, sizeof(frame));

	memset(spaces, ' ', ie_desc_max_len);
	spaces[ie_desc_max_len] = '\0';

	/* sum of bits */
	bits_left = 0;
	for (i = 0; w->ie[i].name; i++)
		bits_left += w->ie[i].bits;

	LOGP(DFRAME, LOGL_INFO, "Received: %s\n", w->name);
	for (i = 0; w->ie[i].name; i++) {
		bits = w->ie[i].bits;
		bits_left -= bits;
		value = (word >> bits_left) & cut_bits[bits];
		frame.ie[w->ie[i].ie] = value;
		if (amps_ie_desc[w->ie[i].ie].decoder)
			LOGP(DFRAME, LOGL_DEBUG, " %s%s: %" PRIu64 " = %s  (%s)\n", spaces + strlen(w->ie[i].name), w->ie[i].name, value, amps_ie_desc[w->ie[i].ie].decoder(value), amps_ie_desc[w->ie[i].ie].desc);
		else
			LOGP(DFRAME, LOGL_DEBUG, " %s%s: %" PRIu64 "  (%s)\n", spaces + strlen(w->ie[i].name), w->ie[i].name, value, amps_ie_desc[w->ie[i].ie].desc);
		/* show result for 3 IEs of table 4 */
		if (w->ie[i].ie == AMPS_IE_LOCAL_MSG_TYPE || w->ie[i].ie == AMPS_IE_ORDQ || w->ie[i].ie == AMPS_IE_ORDER)
			t4++;
		if (t4 == 3) {
			t4 = 0;
			LOGP(DFRAME, LOGL_INFO, " %s--> %s\n", spaces, amps_table4_name(frame.ie[AMPS_IE_LOCAL_MSG_TYPE], frame.ie[AMPS_IE_ORDQ], frame.ie[AMPS_IE_ORDER]));
			/* Explicitly confirm Audit Order for user visibility */
			if (frame.ie[AMPS_IE_ORDER] == 7) {
				LOGP(DFRAME, LOGL_NOTICE, "Received Audit Confirmation (Order 7)\n");
			}
		}
	}

	return &frame;
}

/* get word from data bits and call decoder function */
static void amps_decode_word_focc(amps_t *amps, uint64_t word)
{
	struct def_word *w = NULL;
	int t1t2, ohd = -1, act, scc;
	static frame_t *frame;

	t1t2 = (word >> 38) & 3;

	/* control message */
	if (t1t2 != 3) {
		LOGP_CHAN(DFRAME, LOGL_INFO, "Received Mobile Station Control Message (T1T2 = %d)\n", t1t2);
		if (t1t2 == 1)
			amps->rx_focc_word_count = 1;
		if (t1t2 == 0 || t1t2 == 1) {
			w = &word1_abbreviated_address_word;
			goto decode;
		}
		if (t1t2 == 2) {
			amps->rx_focc_word_count++;
			if (amps->rx_focc_word_count == 2) {
				scc = (word >> 36) & 3;
				if (scc == 3)
					w = &word2_extended_address_word_a;
				else
					w = &word2_extended_address_word_b;
				goto decode;
			}
			LOGP_CHAN(DFRAME, LOGL_INFO, "Decoding of more than 2 Control messages not supported\n");
		}
		return;
	}

	/* overhead message */
	ohd = (word >> 12) & 7;
	switch (ohd) {
	case 6:
		if (!tacs)
			w = &amps_word1_system_parameter_overhead;
		else
			w = &tacs_word1_system_parameter_overhead;
		break;
	case 7:
		w = &word2_system_parameter_overhead;
		break;
	case 4:
		act = (word >> 32) & 15;
		switch (act) {
		case 1:
			w = &rescan_global_action;
			break;
		case 2:
			w = &registration_increment_global_action;
			break;
		case 3:
			w = &location_area_global_action;
			break;
		case 6:
			w = &new_access_channel_set_global_action;
			break;
		case 8:
			w = &overload_control_global_action;
			break;
		case 9:
			w = &access_type_parameters_global_action;
			break;
		case 10:
			w = &access_attempt_parameters_global_action;
			break;
		case 7:
			w = &random_challenge_a_global_action;
			break;
		case 11:
			w = &random_challenge_b_global_action;
			break;
		case 14:
			w = &local_control_1;
			break;
		case 15:
			w = &local_control_2;
			break;
		}
		break;
	case 0:
		w = &registration_id;
		break;
	case 1:
		w = &control_filler;
		break;
	case 2:
		w = &control_channel_information;
		break;
	}

decode:
	if (!w) {
		LOGP_CHAN(DFRAME, LOGL_INFO, "Received Illegal Overhead Message\n");
		return;
	}

	frame = amps_decode_word(word, w);
	/* show control filler delay */
	if (amps->sender.loopback && ohd == 1)
		LOGP_CHAN(DDSP, LOGL_NOTICE, "Round trip delay is %.3f seconds\n", amps->when_received - amps->when_transmitted[frame->ie[AMPS_IE_1111]]);
}

/* get word from data bits and call decoder function
 * return 1 if we expect more frames */
static int amps_decode_word_recc(amps_t *amps, uint64_t word, int first)
{
	struct def_word *w = NULL;
	int msg_count, f, nawc;
	static frame_t *frame;

	f = (word >> 47) & 0x1;
	
	/* NAWC extraction depends on channel type:
	 * - RECC: 3 bits at position 44-46
	 * - RVC: 2 bits at position 45-46
	 */
	if (amps->chan_type == CHAN_TYPE_VC || amps->chan_type == CHAN_TYPE_CC_PC_VC) {
		nawc = (word >> 45) & 0x3;  /* RVC: 2 bits */
	} else {
		nawc = (word >> 44) & 0x7;  /* RECC: 3 bits */
	}

	LOGP_CHAN(DFRAME, LOGL_DEBUG, "RECC/RVC Raw Decode: Word=0x%012lx F=%d NAWC=%d ChanType=%d\n", word, f, nawc, amps->chan_type);

	if (first) {
		memset(amps->rx_recc_dialing, 0, sizeof(amps->rx_recc_dialing));
		amps->rx_recc_word_count = 0;
		amps->rx_recc_nawc = nawc;
		if (f == 0) {
			LOGP_CHAN(DFRAME, LOGL_NOTICE, "Received first word, but F bit is not set.\n");
			return 0;
		}
	} else {
		/* Reverse Voice Channel (RVC) messages are often repeated for redundancy.
		 * Any word with F=1 is the start of a new message (or repetition).
		 * If we were expecting a continuation (F=0) but get F=1, the previous message is incomplete/lost.
		 * We must reset and process this new word.
		 */
		if (f == 1) {
			// LOGP_CHAN(DFRAME, LOGL_DEBUG, "Received F=1 word while in continuation mode. Resetting/Starting new message.\n");
			amps->rx_recc_word_count = 0;
			amps->rx_recc_nawc = nawc;
			/* Proceed as first word */
		} else {
			amps->rx_recc_nawc--;
			if (amps->rx_recc_nawc != nawc) {
				LOGP_CHAN(DFRAME, LOGL_NOTICE, "Received additional word with NAWC mismatch!\n");
			}
		}
	}

	msg_count = amps->rx_recc_word_count;

	if (msg_count == 8) {
		LOGP_CHAN(DFRAME, LOGL_NOTICE, "Received too many words.\n");
		return 0;
	}

	if (amps->chan_type == CHAN_TYPE_VC || amps->chan_type == CHAN_TYPE_CC_PC_VC) {
		/* RVC (Voice Channel) Logic
		 * RVC Format (TIA-553 §2.7.2):
		 * F(1) NAWC(2) T(1) ...
		 * Note: NAWC is only 2 bits on RVC (vs 3 on RECC)
		 * T is at bit 44 (vs 43 on RECC)
		 */
		int t = (word >> 44) & 1;
		int order = (word >> 31) & 0x1F; /* Order is 5 bits at 31-35 */

		LOGP_CHAN(DFRAME, LOGL_DEBUG, "  RVC Fields: T=%d Order=%d (0x%x) NAWC=%d\n", t, order, order, nawc);

		/* Check if we're waiting for ESN word (Word 2 of Serial Number Response)
		 * Word 2 format per TIA/EIA-553-A §2.7.2.1:
		 * F(1)=0 NAWC(2)=00 T(1) ESN(32) P(12)
		 * ESN is at bits 43-12 (32 bits)
		 * F must be 0 (not first word)
		 * Note: T can be 0 or 1 depending on phone implementation
		 * Some phones repeat Word 1 before sending Word 2
		 */
		if (amps->rx_rvc_esn_pending) {
			int f = (word >> 47) & 1;
			/* Raw debug for ESN word */
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "ESN pending - Raw word: 0x%012llx\n", (unsigned long long)word);
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  F=%d NAWC=%d T=%d\n", f, nawc, t);
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  Bits 47-44: %d%d%d%d (F,NAWC,NAWC,T)\n",
				(int)((word >> 47) & 1), (int)((word >> 46) & 1), 
				(int)((word >> 45) & 1), (int)((word >> 44) & 1));
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  Bits 43-12 (ESN): 0x%08x\n", (uint32_t)((word >> 12) & 0xFFFFFFFF));
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  Bits 11-0 (P): 0x%03x\n", (uint32_t)(word & 0xFFF));
			
			/* Check if this is Word 1 repeated (F=1) - skip it and wait for Word 2 */
			if (f == 1) {
				LOGP_CHAN(DFRAME, LOGL_DEBUG, "ESN Word 2: F=1 indicates Word 1 repeat, skipping\n");
				/* Don't clear pending flag - keep waiting for actual Word 2 */
				goto done;
			}
			
			/* F=0 means this is Word 2 (continuation word) with ESN */
			uint32_t esn = (word >> 12) & 0xFFFFFFFF;
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received Serial Number Response Word 2 (ESN=0x%08x)\n", esn);
			amps->rx_rvc_esn_pending = 0;
			amps_rx_esn_response(amps, esn);
			goto done;
		}

		if (t == 1) {
			w = &order_confirmation_rvc_word;
		} else {
			/* T=0 on RVC means Called-Address message (TIA-553 §2.7.2)
			 * Format: F(1) NAWC(2) T(1)=0 DIGITS(32) P(12)
			 * Up to 4 words, each containing 8 digits
			 */
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received RVC Called-Address Word %d (T=0, NAWC=%d)\n", 
				msg_count + 1, nawc);
			
			/* Raw debug: show the 48-bit word and extracted digit bits */
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  Raw word: 0x%012llx\n", (unsigned long long)word);
			/* Extract digit bits manually for debug: bits 43-12 (32 bits of digits) */
			uint32_t digit_bits = (word >> 12) & 0xFFFFFFFF;
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  Digit bits (32): 0x%08x = %c%c%c%c%c%c%c%c\n",
				digit_bits,
				digit2number[(digit_bits >> 28) & 0xF],
				digit2number[(digit_bits >> 24) & 0xF],
				digit2number[(digit_bits >> 20) & 0xF],
				digit2number[(digit_bits >> 16) & 0xF],
				digit2number[(digit_bits >> 12) & 0xF],
				digit2number[(digit_bits >> 8) & 0xF],
				digit2number[(digit_bits >> 4) & 0xF],
				digit2number[(digit_bits >> 0) & 0xF]);
			
			switch (msg_count) {
			case 0:
				w = &word1_called_address;
				break;
			case 1:
				w = &word2_called_address;
				break;
			case 2:
				w = &word3_called_address;
				break;
			case 3:
				w = &word4_called_address;
				break;
			default:
				LOGP_CHAN(DFRAME, LOGL_ERROR, "RVC Called-Address: too many words (%d)\n", msg_count);
				goto done;
			}
			
			/* Decode the word */
			frame = amps_decode_word(word, w);
			if (!frame) {
				LOGP_CHAN(DFRAME, LOGL_ERROR, "Failed to decode RVC Called-Address word\n");
				goto done;
			}
			
			/* Debug: show raw IE values */
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  Decoded IEs: D1=%d D2=%d D3=%d D4=%d D5=%d D6=%d D7=%d D8=%d\n",
				(int)frame->ie[AMPS_IE_DIGIT_1], (int)frame->ie[AMPS_IE_DIGIT_2],
				(int)frame->ie[AMPS_IE_DIGIT_3], (int)frame->ie[AMPS_IE_DIGIT_4],
				(int)frame->ie[AMPS_IE_DIGIT_5], (int)frame->ie[AMPS_IE_DIGIT_6],
				(int)frame->ie[AMPS_IE_DIGIT_7], (int)frame->ie[AMPS_IE_DIGIT_8]);
			
			/* Extract digits from this word
			 * Each word contains 8 digits, but they use different IE indices:
			 * Word 1: DIGIT_1 - DIGIT_8
			 * Word 2: DIGIT_9 - DIGIT_16
			 * Word 3: DIGIT_17 - DIGIT_24
			 * Word 4: DIGIT_25 - DIGIT_32
			 */
			int base = msg_count * 8;
			switch (msg_count) {
			case 0:
				amps->rx_recc_dialing[base + 0] = digit2number[frame->ie[AMPS_IE_DIGIT_1]];
				amps->rx_recc_dialing[base + 1] = digit2number[frame->ie[AMPS_IE_DIGIT_2]];
				amps->rx_recc_dialing[base + 2] = digit2number[frame->ie[AMPS_IE_DIGIT_3]];
				amps->rx_recc_dialing[base + 3] = digit2number[frame->ie[AMPS_IE_DIGIT_4]];
				amps->rx_recc_dialing[base + 4] = digit2number[frame->ie[AMPS_IE_DIGIT_5]];
				amps->rx_recc_dialing[base + 5] = digit2number[frame->ie[AMPS_IE_DIGIT_6]];
				amps->rx_recc_dialing[base + 6] = digit2number[frame->ie[AMPS_IE_DIGIT_7]];
				amps->rx_recc_dialing[base + 7] = digit2number[frame->ie[AMPS_IE_DIGIT_8]];
				break;
			case 1:
				amps->rx_recc_dialing[base + 0] = digit2number[frame->ie[AMPS_IE_DIGIT_9]];
				amps->rx_recc_dialing[base + 1] = digit2number[frame->ie[AMPS_IE_DIGIT_10]];
				amps->rx_recc_dialing[base + 2] = digit2number[frame->ie[AMPS_IE_DIGIT_11]];
				amps->rx_recc_dialing[base + 3] = digit2number[frame->ie[AMPS_IE_DIGIT_12]];
				amps->rx_recc_dialing[base + 4] = digit2number[frame->ie[AMPS_IE_DIGIT_13]];
				amps->rx_recc_dialing[base + 5] = digit2number[frame->ie[AMPS_IE_DIGIT_14]];
				amps->rx_recc_dialing[base + 6] = digit2number[frame->ie[AMPS_IE_DIGIT_15]];
				amps->rx_recc_dialing[base + 7] = digit2number[frame->ie[AMPS_IE_DIGIT_16]];
				break;
			case 2:
				amps->rx_recc_dialing[base + 0] = digit2number[frame->ie[AMPS_IE_DIGIT_17]];
				amps->rx_recc_dialing[base + 1] = digit2number[frame->ie[AMPS_IE_DIGIT_18]];
				amps->rx_recc_dialing[base + 2] = digit2number[frame->ie[AMPS_IE_DIGIT_19]];
				amps->rx_recc_dialing[base + 3] = digit2number[frame->ie[AMPS_IE_DIGIT_20]];
				amps->rx_recc_dialing[base + 4] = digit2number[frame->ie[AMPS_IE_DIGIT_21]];
				amps->rx_recc_dialing[base + 5] = digit2number[frame->ie[AMPS_IE_DIGIT_22]];
				amps->rx_recc_dialing[base + 6] = digit2number[frame->ie[AMPS_IE_DIGIT_23]];
				amps->rx_recc_dialing[base + 7] = digit2number[frame->ie[AMPS_IE_DIGIT_24]];
				break;
			case 3:
				amps->rx_recc_dialing[base + 0] = digit2number[frame->ie[AMPS_IE_DIGIT_25]];
				amps->rx_recc_dialing[base + 1] = digit2number[frame->ie[AMPS_IE_DIGIT_26]];
				amps->rx_recc_dialing[base + 2] = digit2number[frame->ie[AMPS_IE_DIGIT_27]];
				amps->rx_recc_dialing[base + 3] = digit2number[frame->ie[AMPS_IE_DIGIT_28]];
				amps->rx_recc_dialing[base + 4] = digit2number[frame->ie[AMPS_IE_DIGIT_29]];
				amps->rx_recc_dialing[base + 5] = digit2number[frame->ie[AMPS_IE_DIGIT_30]];
				amps->rx_recc_dialing[base + 6] = digit2number[frame->ie[AMPS_IE_DIGIT_31]];
				amps->rx_recc_dialing[base + 7] = digit2number[frame->ie[AMPS_IE_DIGIT_32]];
				break;
			}
			
			LOGP_CHAN(DFRAME, LOGL_INFO, "RVC Called-Address digits so far: '%s'\n", amps->rx_recc_dialing);
			
			amps->rx_recc_word_count++;
			
			/* Check if this is the last word */
			if (nawc == 0) {
				/* All words received - pass to handler */
				LOGP_CHAN(DFRAME, LOGL_INFO, "RVC Called-Address complete: '%s'\n", amps->rx_recc_dialing);
				amps_rx_recc(amps, 0, 0, 0, 0, 0, 0, 0, 0, 0, amps->rx_recc_dialing, 0, 0);
				amps->rx_recc_word_count = 0;
			}
			
			goto done;
		}
	} else {
		/* RECC (Control Channel) Logic */
		/* NAWC is 3 bits, T is at 43. Existing logic is fine for RECC. */
		if (msg_count == 0)
			w = &abbreviated_address_word;

		if (msg_count == 1)
			w = &extended_address_word;

		if (amps->si.word2.s) {
			if (msg_count == 2)
				w = &serial_number_word;
		} else 
			msg_count++;

		/* Word C - Authentication Word (if AUTH is set) */
		/* Word C - Authentication Word (if AUTH is set) */
		if (amps->si.word1.auth) {
			/* If this is an Origination (T=1, Order=0) and it's the last word (nawc=0),
			 * then it MUST be Word D (Digits). Skip Auth logic.
			 */
			if (msg_count == 3 && !(amps->rx_recc_t == 1 && amps->rx_recc_order == 0 && nawc == 0)) {
				// LOGP_CHAN(DFRAME, LOGL_DEBUG, "Decoding Word C (Authentication)\n");
				w = &authentication_word;
			} else {
				if (msg_count == 3)
					LOGP_CHAN(DFRAME, LOGL_DEBUG, "Skipping Optional Word C (Authentication) in favor of Mandatory Word D (Digits)\n");
				msg_count++;
			}
		} else 
			msg_count++;

		/* Word C - Protocol Capability Indicator (if PCI_HOME or PCI_ROAM is set) */
		if (amps->si.acc_type.pci_home || amps->si.acc_type.pci_roam) {
			/* If this is an Origination (T=1, Order=0) and it's the last word (nawc=0),
			 * then it MUST be Word D (Digits). Skip PCI logic.
			 */
			if (msg_count == 4 && !(amps->rx_recc_t == 1 && amps->rx_recc_order == 0 && nawc == 0)) {
				// LOGP_CHAN(DFRAME, LOGL_DEBUG, "Decoding Word C (PCI)\n");
				w = &pci_report_registration_word;
			} else {
				if (msg_count == 4)
					LOGP_CHAN(DFRAME, LOGL_DEBUG, "Skipping Optional Word C (PCI) in favor of Mandatory Word D (Digits)\n");
				msg_count++;
			}
		} else
			msg_count++;

		/* Word D - First word of called address (Origination) */
		if (msg_count == 5)
			w = &first_word_of_the_called_address;

		if (msg_count == 6)
			w = &second_word_of_the_called_address;

		if (msg_count == 7)
			w = &third_word_of_the_called_address;

		if (msg_count == 8)
			w = &fourth_word_of_the_called_address;
	}

	if (!w) {
		/* Log the raw word data at ERROR level for debugging */
		LOGP_CHAN(DFRAME, LOGL_ERROR, "Received unparseable RECC/RVC message: raw=0x%012llx msg_count=%d\n",
			(unsigned long long)word, msg_count);
		if (amps->chan_type == CHAN_TYPE_VC || amps->chan_type == CHAN_TYPE_CC_PC_VC) {
			int nawc_rvc = (word >> 45) & 0x3;
			int t = (word >> 44) & 1;
			int msg_type = (word >> 36) & 0x1F;
			int ordq = (word >> 33) & 0x7;
			int order = (word >> 28) & 0x1F;
			LOGP_CHAN(DFRAME, LOGL_ERROR, "  RVC decode attempt: T=%d NAWC=%d MSG_TYPE=%d ORDQ=%d ORDER=%d\n",
				t, nawc_rvc, msg_type, ordq, order);
		}
		goto done;
	}

	frame = amps_decode_word(word, w);

	/* RVC Extraction: Any Order Confirmation */
	if (w == &order_confirmation_rvc_word) {
		/* Order 26 = PCI Report */
		if (frame->ie[AMPS_IE_ORDER] == 26) {
			amps_rx_pci_report(amps, frame->ie[AMPS_IE_MSPC], frame->ie[AMPS_IE_MSCAP]);
		} else if (frame->ie[AMPS_IE_ORDER] == 3) {
			/* Order 3 = Release from mobile station */
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received Release order from mobile (Order=3 ORDQ=%d) - mobile initiated hangup\n",
				(int)frame->ie[AMPS_IE_ORDQ]);
			amps_rx_release_order(amps, frame->ie[AMPS_IE_ORDQ]);
		} else if (frame->ie[AMPS_IE_ORDER] == 7) {
			/* Order 7 = Audit confirmation */
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received RVC Order Confirmation: Audit (Order=7 ORDQ=%d)\n",
				(int)frame->ie[AMPS_IE_ORDQ]);
		} else if (frame->ie[AMPS_IE_ORDER] == 11) {
			/* Order 11 = Change Power confirmation */
			int power_level = (int)frame->ie[AMPS_IE_ORDQ];
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received RVC Order Confirmation: Change Power to level %d (Order=11 ORDQ=%d)\n",
				power_level, power_level);
			LOGP_CHAN(DFRAME, LOGL_NOTICE, "<<< Change Power Confirmed: Level %d -> %s\n",
				power_level,
				power_level == 0 ? "4.0W (max)" :
				power_level == 1 ? "1.6W" :
				power_level == 2 ? "630mW" :
				power_level == 3 ? "250mW" :
				power_level == 4 ? "100mW" :
				power_level == 5 ? "40mW" :
				power_level == 6 ? "16mW" :
				"6.3mW (min)");
		} else if (frame->ie[AMPS_IE_ORDER] == 18) {
			/* Order 18 = Flash With Info confirmation */
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received RVC Order Confirmation: Flash With Info (Order=18 ORDQ=%d)\n",
				(int)frame->ie[AMPS_IE_ORDQ]);
		} else if (frame->ie[AMPS_IE_ORDER] == 5) {
			/* Order 5 = Message Waiting confirmation
			 * Per TIA/EIA-553-A, mobile sends order confirmation after receiving MWI.
			 * MSG_TYPE echoes back the message count we sent.
			 */
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received RVC Order Confirmation: %s (Order=5 ORDQ=%d MSG_TYPE=%d)\n",
				amps_table4_name((int)frame->ie[AMPS_IE_LOCAL_MSG_TYPE], (int)frame->ie[AMPS_IE_ORDQ], 5),
				(int)frame->ie[AMPS_IE_ORDQ], (int)frame->ie[AMPS_IE_LOCAL_MSG_TYPE]);
		} else if (frame->ie[AMPS_IE_ORDER] == 15 && frame->ie[AMPS_IE_ORDQ] == 1) {
			/* Order 15 with ORDQ=1 = Serial Number Response (Word 1)
			 * Per TIA/EIA-553-A Table 3.7.1-1:
			 * - Order 15 (01111) with ORDQ=001 = "Serial Number Request/Response"
			 * - This is Word 1, Word 2 contains the ESN
			 * - NAWC indicates how many more words are coming
			 */
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "ESN Response Word 1 - Raw word: 0x%012llx\n", (unsigned long long)word);
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  F=%d NAWC=%d T=%d ORDER=%d ORDQ=%d\n",
				(int)((word >> 47) & 1), nawc, (int)((word >> 44) & 1),
				(int)frame->ie[AMPS_IE_ORDER], (int)frame->ie[AMPS_IE_ORDQ]);
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received Serial Number Response Word 1 (Order=15 ORDQ=1 NAWC=%d)\n", nawc);
			if (nawc > 0) {
				/* More words coming - set flag to wait for ESN word */
				amps->rx_rvc_esn_pending = 1;
				LOGP_CHAN(DFRAME, LOGL_DEBUG, "Waiting for ESN in next word\n");
			} else {
				LOGP_CHAN(DFRAME, LOGL_ERROR, "Serial Number Response Word 1 with NAWC=0 - no ESN word!\n");
			}
		} else if (frame->ie[AMPS_IE_ORDER] == 15 && frame->ie[AMPS_IE_ORDQ] == 0) {
			/* Order 15 with ORDQ=0 = Parameter Update Order Confirmation
			 * Per TIA/EIA-553-A Table 3.7.1-1:
			 * - Order 15 (01111) with ORDQ=000 = "Parameter Update Order/Confirmation"
			 * - This is a simple confirmation for parameter updates
			 * - NOT related to ESN request (that uses ORDQ=1)
			 */
			LOGP_CHAN(DFRAME, LOGL_INFO, "Received Parameter Update Order Confirmation (Order=15 ORDQ=0)\n");
		} else {
			/* Unknown/unhandled order - log at ERROR level with full details */
			LOGP_CHAN(DFRAME, LOGL_ERROR, "Received UNHANDLED RVC Order Confirmation: %s (Order=%d ORDQ=%d MSG_TYPE=%d MSPC=%d MSCAP=%d)\n", 
				amps_table4_name(0, (int)frame->ie[AMPS_IE_ORDQ], (int)frame->ie[AMPS_IE_ORDER]),
				(int)frame->ie[AMPS_IE_ORDER], (int)frame->ie[AMPS_IE_ORDQ],
				(int)frame->ie[AMPS_IE_LOCAL_MSG_TYPE],
				(int)frame->ie[AMPS_IE_MSPC], (int)frame->ie[AMPS_IE_MSCAP]);
		}
		/* RVC messages are individual, reset RECC counter */
		amps->rx_recc_word_count = 0;
		goto done;
	}

	if (amps->rx_recc_word_count == 0 && frame) {
		amps->rx_recc_min1 = frame->ie[AMPS_IE_MIN1];
		amps->rx_recc_scm = frame->ie[AMPS_IE_SCM];
		amps->rx_recc_t = frame->ie[AMPS_IE_T];  /* T field: 1=Origination, 0=Paging Response */
	}
	if (amps->rx_recc_word_count == 1 && frame) {
		amps->rx_recc_min2 = frame->ie[AMPS_IE_MIN2];
		amps->rx_recc_msg_type = frame->ie[AMPS_IE_LOCAL_MSG_TYPE];
		amps->rx_recc_ordq = frame->ie[AMPS_IE_ORDQ];
		amps->rx_recc_order = frame->ie[AMPS_IE_ORDER];
		amps->rx_recc_scm |= frame->ie[AMPS_IE_SCM] << 4;
		amps->rx_recc_mpci = frame->ie[AMPS_IE_MPCI];
	}
	if (amps->rx_recc_word_count == 2 && frame) {
		if (amps->si.word2.s)
			amps->rx_recc_esn = frame->ie[AMPS_IE_ESN];
		else
			amps->rx_recc_esn = 0;
	}
	/* Extract MSPC and MSCAP from Word C (PCI) if present */
	if (msg_count == 4 && frame && (amps->si.acc_type.pci_home || amps->si.acc_type.pci_roam)) {
		amps->rx_recc_mspc = frame->ie[AMPS_IE_MSPC];
		amps->rx_recc_mscap = frame->ie[AMPS_IE_MSCAP];
	}
	/* Word D - First word of called address (Origination) */
	if (msg_count == 5 && frame) {
		amps->rx_recc_dialing[0] = digit2number[frame->ie[AMPS_IE_DIGIT_1]];
		amps->rx_recc_dialing[1] = digit2number[frame->ie[AMPS_IE_DIGIT_2]];
		amps->rx_recc_dialing[2] = digit2number[frame->ie[AMPS_IE_DIGIT_3]];
		amps->rx_recc_dialing[3] = digit2number[frame->ie[AMPS_IE_DIGIT_4]];
		amps->rx_recc_dialing[4] = digit2number[frame->ie[AMPS_IE_DIGIT_5]];
		amps->rx_recc_dialing[5] = digit2number[frame->ie[AMPS_IE_DIGIT_6]];
		amps->rx_recc_dialing[6] = digit2number[frame->ie[AMPS_IE_DIGIT_7]];
		amps->rx_recc_dialing[7] = digit2number[frame->ie[AMPS_IE_DIGIT_8]];
	}
	if (msg_count == 6 && frame) {
		amps->rx_recc_dialing[8] = digit2number[frame->ie[AMPS_IE_DIGIT_9]];
		amps->rx_recc_dialing[9] = digit2number[frame->ie[AMPS_IE_DIGIT_10]];
		amps->rx_recc_dialing[10] = digit2number[frame->ie[AMPS_IE_DIGIT_11]];
		amps->rx_recc_dialing[11] = digit2number[frame->ie[AMPS_IE_DIGIT_12]];
		amps->rx_recc_dialing[12] = digit2number[frame->ie[AMPS_IE_DIGIT_13]];
		amps->rx_recc_dialing[13] = digit2number[frame->ie[AMPS_IE_DIGIT_14]];
		amps->rx_recc_dialing[14] = digit2number[frame->ie[AMPS_IE_DIGIT_15]];
		amps->rx_recc_dialing[15] = digit2number[frame->ie[AMPS_IE_DIGIT_16]];
	}
	if (msg_count == 7 && frame) {
		amps->rx_recc_dialing[16] = digit2number[frame->ie[AMPS_IE_DIGIT_17]];
		amps->rx_recc_dialing[17] = digit2number[frame->ie[AMPS_IE_DIGIT_18]];
		amps->rx_recc_dialing[18] = digit2number[frame->ie[AMPS_IE_DIGIT_19]];
		amps->rx_recc_dialing[19] = digit2number[frame->ie[AMPS_IE_DIGIT_20]];
		amps->rx_recc_dialing[20] = digit2number[frame->ie[AMPS_IE_DIGIT_21]];
		amps->rx_recc_dialing[21] = digit2number[frame->ie[AMPS_IE_DIGIT_22]];
		amps->rx_recc_dialing[22] = digit2number[frame->ie[AMPS_IE_DIGIT_23]];
		amps->rx_recc_dialing[23] = digit2number[frame->ie[AMPS_IE_DIGIT_24]];
	}
	if (msg_count == 8 && frame) {
		amps->rx_recc_dialing[24] = digit2number[frame->ie[AMPS_IE_DIGIT_25]];
		amps->rx_recc_dialing[25] = digit2number[frame->ie[AMPS_IE_DIGIT_26]];
		amps->rx_recc_dialing[26] = digit2number[frame->ie[AMPS_IE_DIGIT_27]];
		amps->rx_recc_dialing[27] = digit2number[frame->ie[AMPS_IE_DIGIT_28]];
		amps->rx_recc_dialing[28] = digit2number[frame->ie[AMPS_IE_DIGIT_29]];
		amps->rx_recc_dialing[29] = digit2number[frame->ie[AMPS_IE_DIGIT_30]];
		amps->rx_recc_dialing[30] = digit2number[frame->ie[AMPS_IE_DIGIT_31]];
		amps->rx_recc_dialing[31] = digit2number[frame->ie[AMPS_IE_DIGIT_32]];
	}

	LOGP_CHAN(DFRAME, LOGL_INFO, "expecting %d more word(s) to come\n", amps->rx_recc_nawc);

	if (msg_count >= 3 && amps->rx_recc_nawc == 0) {
		/* if no digit messages are present, send NULL as dial string (paging reply) */
		amps_rx_recc(amps, amps->rx_recc_t, amps->rx_recc_scm, amps->rx_recc_mpci, amps->rx_recc_esn, amps->rx_recc_min1, amps->rx_recc_min2, amps->rx_recc_msg_type, amps->rx_recc_ordq, amps->rx_recc_order, (msg_count > 4) ? amps->rx_recc_dialing : NULL, amps->rx_recc_mspc, amps->rx_recc_mscap);
	}


	amps->rx_recc_word_count++;

done:
	if (amps->rx_recc_nawc > 0)
		return 1;

	return 0;
}

/*
 * encode and decode bits
 */

static const char *dotting = "10101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101";
static const char *sync_word = "11100010010";

char *test1 = "101011101000111001101110100111011111111110100000";
char *test2 = "000100000011011010000000000110101101001110101011";

#if 0
static uint64_t string2bin(const char *string)
{
	uint64_t bin = 0;

	while (*string)
		bin = (bin << 1) | ((*string++) & 1);

	return bin;
}
#endif

static void amps_encode_focc_bits(uint64_t word_a, uint64_t word_b, char *bits)
{
	int i, j, k;

	memcpy(bits + 0, dotting, 10);
	bits[10] = 'i';
	memcpy(bits + 11, sync_word, 11);
	bits[22] = 'i';
	k = 23;
	for (i = 0; i < 5; i++) {
		/* WORD A (msb first) */
		for (j = 39; j >= 0; j--) {
			bits[k++] = ((word_a >> j) & 1) + '0';
			if ((j % 10) == 0)
				bits[k++] = 'i';
		}
		/* WORD B (msb first) */
		for (j = 39; j >= 0; j--) {
			bits[k++] = ((word_b >> j) & 1) + '0';
			if ((j % 10) == 0)
				bits[k++] = 'i';
		}
	}

	if (k != 463)
		abort();
	bits[k] = '\0';

#ifdef BIT_DEBUGGING
	if (loglevel == LOGL_DEBUG) {
		char text[64];

		strncpy(text, bits, 23);
		text[23] = '\0';
		LOGP(DFRAME, LOGL_INFO, "TX FOCC: %s\n", text);
		for (i = 0; i < 10; i++) {
			strncpy(text, bits + 23 + i * 44, 44);
			text[44] = '\0';
			LOGP(DFRAME, LOGL_DEBUG, "  word %c - %s\n", (i & 1) ? 'b' : 'a', text);
		}
	}
#endif
}

static void amps_encode_fvc_bits(uint64_t word_a, char *bits)
{
	int i, j, k;

	
	k = 0;
	for (i = 0; i < 11; i++) {
		if (i == 0) {
			memcpy(bits + k, dotting, 101);
			k += 101;
		} else {
			memcpy(bits + k, dotting, 37);
			k += 37;
		}
		memcpy(bits + k, sync_word, 11);
		k += 11;
		for (j = 39; j >= 0; j--)
			bits[k++] = ((word_a >> j) & 1) + '0';
	}

	if (k != 1032)
		abort();
	bits[k] = '\0';

#ifdef BIT_DEBUGGING
	if (loglevel == LOGL_DEBUG) {
		LOGP(DFRAME, LOGL_INFO, "TX FVC: %s\n", bits);
	}
#endif
}

int amps_encode_frame_focc(amps_t *amps, char *bits)
{
	uint64_t word = 0;
	int debug = !amps->tx_focc_debugged;

	/* init overhead train */
	if (amps->tx_focc_frame_count == 0)
		prepare_sysinfo(&amps->si);
	/* send overhead train */
	if (amps->si.num) {
		word = get_sysinfo(&amps->si, debug);
		if (++amps->tx_focc_frame_count >= amps->si.overhead_repeat)
			amps->tx_focc_frame_count = 0;
		goto send;
	}

	/* see if we can schedule a mobile control message */
	if (!amps->tx_focc_send) {
		transaction_t *trans;
		trans = amps_tx_frame_focc(amps);
		if (trans) {
			int i;
			amps->tx_focc_min1 = trans->min1;
			amps->tx_focc_min2 = trans->min2;
			amps->tx_focc_msg_type = trans->msg_type;
			amps->tx_focc_ordq = trans->ordq;
			amps->tx_focc_order = trans->order;
			amps->tx_focc_chan = trans->chan;
			amps->tx_focc_send = 1;
			amps->tx_focc_word_count = 0;
			amps->tx_focc_word_repeat = 0;
			
			/* Check if this is a Directed Retry (Order 12) - needs 4 words */
			if (trans->order == 12 && trans->retry_num_channels > 0) {
				amps->tx_focc_num_words = 4;  /* Word 1, 2, 3, 4 */
				for (i = 0; i < 6; i++) {
					amps->tx_focc_retry_channels[i] = (i < trans->retry_num_channels) ? 
						trans->retry_channels[i] : 0;
				}
				amps->tx_focc_retry_num_channels = trans->retry_num_channels;
			} else {
				amps->tx_focc_num_words = 2;  /* Normal 2-word message */
				amps->tx_focc_retry_num_channels = 0;
			}
		}
		/* on change of dsp mode */
		if (amps->dsp_mode != DSP_MODE_FRAME_RX_FRAME_TX)
			return 1;
	}
	/* send scheduled mobile control message */
	if (amps->tx_focc_send) {
		if (amps->tx_focc_word_count == 0)
			word = amps_encode_word1_abbreviated_address_word(amps->si.dcc, amps->tx_focc_min1, 1);
		else if (amps->tx_focc_word_count == 1) {
			if (amps->tx_focc_chan)
				word = amps_encode_word2_extended_address_word_b(amps->sat, amps->tx_focc_min2, amps->si.vmac, amps->tx_focc_chan);
			else
				word = amps_encode_word2_extended_address_word_a(amps->tx_focc_min2, amps->tx_focc_msg_type, amps->tx_focc_ordq, amps->tx_focc_order);
		} else if (amps->tx_focc_word_count == 2) {
			/* Word 3 - First Directed-Retry Word (channels 1-3) */
			word = amps_encode_word3_directed_retry(amps->sat,
				amps->tx_focc_retry_channels[0],
				amps->tx_focc_retry_channels[1],
				amps->tx_focc_retry_channels[2]);
		} else if (amps->tx_focc_word_count == 3) {
			/* Word 4 - Second Directed-Retry Word (channels 4-6) */
			word = amps_encode_word4_directed_retry(amps->sat,
				amps->tx_focc_retry_channels[3],
				amps->tx_focc_retry_channels[4],
				amps->tx_focc_retry_channels[5]);
		}
		/* dont wrap frame count until we are done */
		++amps->tx_focc_frame_count;
		if (++amps->tx_focc_word_count == amps->tx_focc_num_words) {
			amps->tx_focc_word_count = 0;
			if (++amps->tx_focc_word_repeat == 3) {
				amps->tx_focc_word_repeat = 0;
				amps->tx_focc_send = 0;
				/* now we may wrap */
				if (amps->tx_focc_frame_count >= amps->si.overhead_repeat)
					amps->tx_focc_frame_count = 0;
			}
		}
		goto send;
	}

	/* send filler */
	word = amps_encode_control_filler(amps, amps->si.dcc, amps->si.filler.cmac, amps->si.filler.sdcc1, amps->si.filler.sdcc2, amps->si.filler.wfom, debug);
	if (++amps->tx_focc_frame_count >= amps->si.overhead_repeat)
		amps->tx_focc_frame_count = 0;
	if (debug)
		LOGP_CHAN(DFRAME, LOGL_INFO, "Subsequent system/filler frames are not shown, to prevent flooding the output.\n");
	amps->tx_focc_debugged = 1;

send:
	amps_encode_focc_bits(word, word, bits);

	return 0;
}

int amps_encode_frame_fvc(amps_t *amps, char *bits)
{
	uint64_t word = 0;

	/* see if we can schedule a mobile control message */
	if (!amps->tx_fvc_send) {
		transaction_t *trans;
		trans = amps_tx_frame_fvc(amps);
		if (trans) {
			amps->tx_fvc_msg_type = trans->msg_type;
			amps->tx_fvc_ordq = trans->ordq;
			amps->tx_fvc_order = trans->order;
			amps->tx_fvc_chan = trans->chan;
			strncpy(amps->tx_fvc_callerid, trans->caller_id, sizeof(amps->tx_fvc_callerid) - 1);
			/* Set defaults */
			amps->tx_fvc_callerid_signal = 1; /* Medium Pitch, Long Cadence */
			/* ie_signal(1):
			 * Pitch = (1 >> 6) & 3 = 0 (Medium)
			 * Cadence = 1 & 0x3f = 1 (Long)
			 */
			amps->tx_fvc_callerid_screen = 3; /* Network Provided */
			if (trans->caller_id[0])
				amps->tx_fvc_callerid_present = 0; /* Presentation Allowed */
			else
				amps->tx_fvc_callerid_present = 2; /* Number Not Available */

			/* Override with custom parameters from transaction if set */
			if (trans->signal_pitch != -1 || trans->signal_cadence != -1) {
				int pitch = (trans->signal_pitch != -1) ? trans->signal_pitch : 0;
				int cadence = (trans->signal_cadence != -1) ? trans->signal_cadence : 1;
				amps->tx_fvc_callerid_signal = (pitch << 6) | (cadence & 0x3f);
			}
			if (trans->presentation_indicator != -1)
				amps->tx_fvc_callerid_present = trans->presentation_indicator;
			if (trans->screening_indicator != -1)
				amps->tx_fvc_callerid_screen = trans->screening_indicator;

			/* Set target SAT for handoff (-1 = use current channel's SAT) */
			amps->tx_fvc_scc = trans->handoff_scc;

			amps->tx_fvc_send = 1;
			amps->tx_fvc_word_count = 0;
			amps->tx_fvc_word_repeat = 0;
		}
		/* on change of dsp mode */
		if (amps->dsp_mode != DSP_MODE_AUDIO_RX_FRAME_TX)
			return 1;
	}

	/* send scheduled mobile control message */
	if (amps->tx_fvc_send) {
		if (amps->tx_fvc_word_count == 0) {
			if (amps->tx_fvc_chan) {
				uint8_t vmac = amps->si.vmac;
				uint8_t scc = amps->sat;  /* Default: current channel's SAT */
				/* Use transaction-specific VMAC if available */
				if (amps->trans_list && (amps->trans_list->state == TRANS_CALL || amps->trans_list->state == TRANS_CALL_CHANGE_POWER || amps->trans_list->state == TRANS_CALL_CHANGE_POWER_SEND))
					vmac = amps->trans_list->current_vmac;
				/* For handoff: use target SAT (tx_fvc_scc) for SCC, current SAT for PSCC
				 * Per TIA/EIA-553-A:
				 * - SCC = SAT Color Code of target channel (what mobile tunes to)
				 * - PSCC = Present SAT Color Code (mobile verifies this matches current)
				 */
				if (amps->tx_fvc_scc >= 0) {
					scc = amps->tx_fvc_scc;
					LOGP_CHAN(DFRAME, LOGL_INFO, "Transmit: Mobile Station Control Message Word 1 Type B (Handoff)\n");
					LOGP_CHAN(DFRAME, LOGL_INFO, "    CHAN=%d, SCC=%d (target), PSCC=%d (current), VMAC=%d\n",
						amps->tx_fvc_chan, scc, amps->sat, vmac);
				} else {
					LOGP_CHAN(DFRAME, LOGL_INFO, "Transmit: Mobile Station Control Message Word 1 Type B\n");
					LOGP_CHAN(DFRAME, LOGL_INFO, "    CHAN=%d, SCC=%d, PSCC=%d, VMAC=%d\n",
						amps->tx_fvc_chan, scc, amps->sat, vmac);
				}
				word = amps_encode_mobile_station_control_message_word1_b(scc, amps->sat, (amps->si.word2.dtx) ? 1 : 0, 0, 0, vmac, amps->tx_fvc_chan);
			} else
				word = amps_encode_mobile_station_control_message_word1_a(amps->sat, amps->tx_fvc_msg_type, amps->tx_fvc_ordq, amps->tx_fvc_order);
			/* done, if we don't have ALERTING with info or FLASH with info */
			if (amps->tx_fvc_order != 17 && amps->tx_fvc_order != 18)
				amps->tx_fvc_send = 0;
		} else if (amps->tx_fvc_word_count == 1) {
			int cpn_rl, rl_w;
			if (amps->tx_fvc_order == 17) {
				/* Alert With Info - Word 2 */
				if (amps->tx_fvc_ordq == 0) {
					/* ORDQ=0: Character message (Caller ID) */
					cpn_rl = strlen(amps->tx_fvc_callerid);
					rl_w = (cpn_rl + 2) / 3;
					word = amps_encode_word2_first_alert_with_info_word(rl_w, amps->tx_fvc_callerid_signal, cpn_rl, amps->tx_fvc_callerid_present, amps->tx_fvc_callerid_screen);
					if (cpn_rl == 0)
						amps->tx_fvc_send = 0;
				} else if (amps->tx_fvc_ordq == 1) {
					/* ORDQ=1: CRI (Charging Rate Indication) */
					rl_w = (amps->tx_fvc_cri_elements + 1) / 2 + 1; /* CRI words needed */
					word = amps_encode_word2_alert_with_info_cri(rl_w, amps->tx_fvc_callerid_signal);
					if (amps->tx_fvc_cri_elements == 0)
						amps->tx_fvc_send = 0;
				} else if (amps->tx_fvc_ordq == 2) {
					/* ORDQ=2: TCI (Total Charging Information) */
					rl_w = (amps->tx_fvc_tci_rows + 1) / 2 + 1; /* TCI words needed */
					word = amps_encode_word2_alert_with_info_tci(rl_w, amps->tx_fvc_callerid_signal);
					if (amps->tx_fvc_tci_rows == 0)
						amps->tx_fvc_send = 0;
				}
			} else {
				/* Flash With Info - Word 2 */
				if (amps->tx_fvc_ordq == 0) {
					/* ORDQ=0: Character message */
					cpn_rl = strlen(amps->tx_fvc_flashinfo);
					rl_w = (cpn_rl + 2) / 3;
					word = amps_encode_word2_first_flash_with_info_word(rl_w, cpn_rl, amps->tx_fvc_flashinfo_pi, amps->tx_fvc_flashinfo_si);
					if (cpn_rl == 0)
						amps->tx_fvc_send = 0;
				} else if (amps->tx_fvc_ordq == 1) {
					/* ORDQ=1: CRI (Charging Rate Indication) */
					rl_w = (amps->tx_fvc_cri_elements + 1) / 2 + 1;
					word = amps_encode_word2_flash_with_info_cri(rl_w);
					if (amps->tx_fvc_cri_elements == 0)
						amps->tx_fvc_send = 0;
				} else if (amps->tx_fvc_ordq == 2) {
					/* ORDQ=2: TCI (Total Charging Information) */
					rl_w = (amps->tx_fvc_tci_rows + 1) / 2 + 1;
					word = amps_encode_word2_flash_with_info_tci(rl_w);
					if (amps->tx_fvc_tci_rows == 0)
						amps->tx_fvc_send = 0;
				}
			}
		} else {
			/* Word 3+ encoding depends on ORDQ */
			if (amps->tx_fvc_ordq == 0) {
				/* ORDQ=0: Character message words */
				const char *characters;
				if (amps->tx_fvc_order == 17) {
					/* Alert With Info - Word 3+ */
					characters = amps->tx_fvc_callerid + (amps->tx_fvc_word_count - 2) * 3;
					word = amps_encode_wordn_n_minus_1th_alert_with_info_word(characters);
				} else {
					/* Flash With Info - Word 3+ */
					characters = amps->tx_fvc_flashinfo + (amps->tx_fvc_word_count - 2) * 3;
					word = amps_encode_wordn_n_minus_1th_flash_with_info_word(characters);
				}
				if (strlen(characters) <= 3)
					amps->tx_fvc_send = 0;
			} else if (amps->tx_fvc_ordq == 1) {
				/* ORDQ=1: CRI words */
				int cri_word = amps->tx_fvc_word_count - 2; /* 0-based CRI word index */
				uint8_t *cri = amps->tx_fvc_cri;
				switch (cri_word) {
				case 0: /* Word 3: E1 and E2 partial */
					word = amps_encode_word3_cri(cri[3], cri[2], cri[1], cri[0],
					                              cri[7], cri[6]);
					if (amps->tx_fvc_cri_elements <= 2)
						amps->tx_fvc_send = 0;
					break;
				case 1: /* Word 4: E2 partial and E3 */
					word = amps_encode_word4_cri(cri[5], cri[4], cri[11], cri[10],
					                              cri[9], cri[8]);
					if (amps->tx_fvc_cri_elements <= 4)
						amps->tx_fvc_send = 0;
					break;
				case 2: /* Word 5: E4 and E5 partial */
					word = amps_encode_word5_cri(cri[15], cri[14], cri[13], cri[12],
					                              cri[19], cri[18]);
					if (amps->tx_fvc_cri_elements <= 6)
						amps->tx_fvc_send = 0;
					break;
				case 3: /* Word 6: E5 partial and E6 */
					word = amps_encode_word6_cri(cri[17], cri[16], cri[23], cri[22],
					                              cri[21], cri[20]);
					if (amps->tx_fvc_cri_elements <= 6)
						amps->tx_fvc_send = 0;
					break;
				case 4: /* Word 7: E7 and E8 partial */
					word = amps_encode_word7_cri(cri[27], cri[26], cri[25], cri[24],
					                              cri[31], cri[30]);
					if (amps->tx_fvc_cri_elements <= 8)
						amps->tx_fvc_send = 0;
					break;
				case 5: /* Word 8: E8 partial */
					word = amps_encode_word8_cri(cri[29], cri[28]);
					amps->tx_fvc_send = 0;
					break;
				default:
					amps->tx_fvc_send = 0;
					word = 0;
					break;
				}
			} else if (amps->tx_fvc_ordq == 2) {
				/* ORDQ=2: TCI words */
				int tci_word = amps->tx_fvc_word_count - 2; /* 0-based TCI word index */
				uint8_t *tci = amps->tx_fvc_tci;
				switch (tci_word) {
				case 0: /* Word 3: Row 1 and Row 2 partial */
					/* TCI1 is 7 bits containing row 1 digits 1-4 packed */
					/* TCI5 is 1 bit (MSB of row 1 digit 4) */
					{
						uint8_t tci1 = ((tci[0] & 0x0F) << 3) | ((tci[1] & 0x0E) >> 1);
						uint8_t tci5 = (tci[1] & 0x01);
						word = amps_encode_word3_tci(tci1, tci5, tci[7], tci[6], tci[5], tci[4]);
					}
					if (amps->tx_fvc_tci_rows <= 2)
						amps->tx_fvc_send = 0;
					break;
				case 1: /* Word 4: Row 3 and Row 4 partial */
					word = amps_encode_word4_tci(tci[11], tci[10], tci[9], tci[8],
					                              tci[15], tci[14]);
					if (amps->tx_fvc_tci_rows <= 4)
						amps->tx_fvc_send = 0;
					break;
				case 2: /* Word 5: Row 4 partial */
					word = amps_encode_word5_tci(tci[13], tci[12]);
					amps->tx_fvc_send = 0;
					break;
				default:
					amps->tx_fvc_send = 0;
					word = 0;
					break;
				}
			}
		}
		amps->tx_fvc_word_count++;
	} else
		return 1;

	amps_encode_fvc_bits(word, bits);

	return 0;
}

/* assemble FOCC bits */
static void amps_decode_bits_focc(amps_t *amps, const char *bits)
{
	char word_string[41];
	uint64_t word_a[5], word_b[5], word;
	int crc_a_ok[5], crc_b_ok[5], crc_ok;
	int idle;
	int i, j, k, crc_i, crc_j;

	bits++; /* skip B/I after sync */
	idle = 0;
	for (i = 0; i < 10; i++) {
		word = 0;
		for (j = 0, k = 0; j < 44; j++) {
			if (j % 11 == 10) {
				idle += (*bits++) & 1;
				continue;
			}
			word_string[k++] = *bits;
			word = (word << 1) | ((*bits++) & 1);
		}
		word_string[k] = '\0';
		if (!strncmp(encode_bch(word_string, 28), word_string + 28, 12))
			crc_ok = 1;
		else
			crc_ok = 0;
		if ((i & 1) == 0) {
			word_a[i >> 1] = word;
			crc_a_ok[i >> 1] = crc_ok;
		} else {
			word_b[i >> 1] = word;
			crc_b_ok[i >> 1] = crc_ok;
		}
	}
	bits -= 440;

	if (idle > 20)
		idle = 1;
	else
		idle = 0;

	LOGP_CHAN(DFRAME, LOGL_INFO, "RX FOCC: B/I = %s\n", (idle) ? "idle" : "busy");
	if (loglevel == LOGL_DEBUG) {
		char text[64];

		for (i = 0; i < 10; i++) {
			strncpy(text, bits + i * 44, 44);
			text[44] = '\0';
			if ((i & 1) == 0)
				LOGP_CHAN(DFRAME, LOGL_DEBUG, "  word a - %s%s\n", text, (crc_a_ok[i >> 1]) ? " ok" : " BAD CRC!");
			else
				LOGP_CHAN(DFRAME, LOGL_DEBUG, "  word b - %s%s\n", text, (crc_b_ok[i >> 1]) ? " ok" : " BAD CRC!");
		}
	}

	for (crc_i = 0; crc_i < 5; crc_i++) {
		if (crc_a_ok[crc_i])
			break;
	}
	if (crc_i < 5) {
		amps_decode_word_focc(amps, word_a[crc_i]);
	}
	for (crc_j = 0; crc_j < 5; crc_j++) {
		if (crc_b_ok[crc_j])
			break;
	}
	if (crc_j < 5 && word_b[crc_j] != word_a[crc_i]) {
		amps_decode_word_focc(amps, word_b[crc_j]);
	}
}

/* assemble RECC bits, return true, if more bits are expected */
static int amps_decode_bits_recc(amps_t *amps, const char *bits, int first)
{
	char word_string[49];
	int8_t dcc = -1;
	uint64_t word_a[5], word;
	int crc_a_ok[5], crc_ok, crc_ok_count = 0;
	int i, j, k, crc_i;
	const char *bits_ = bits; /* for extra check */

	/* decode color code */
	if (first) {
		dcc = 0;
		for (j = 0; j < 7; j++) {
			dcc = (dcc << 1) | ((*bits++) & 1);
		}
		dcc = dcc_decode[dcc];
	}

	/* assemble word */
	for (i = 0; i < 5; i++) {
		word = 0;
		for (j = 0, k = 0; j < 48; j++) {
			word_string[k++] = *bits;
			word = (word << 1) | ((*bits++) & 1);
		}
		word_string[k] = '\0';
		if (!strncmp(encode_bch(word_string, 36), word_string + 36, 12)) {
			crc_ok = 1;
			crc_ok_count++;
		} else
			crc_ok = 0;
		word_a[i] = word;
		crc_a_ok[i] = crc_ok;
	}
	bits -= 240;

	if (crc_ok_count == 0) {
		/* check if we receive frame in a loop */
		crc_ok = 0;
		bits_++; /* skip B/I after sync */
		for (i = 0; i < 5; i++) {
			word = 0;
			for (j = 0, k = 0; j < 44; j++) {
				if (j % 11 == 10) {
					bits_++;
					continue;
				}
				word_string[k++] = *bits_;
				word = (word << 1) | ((*bits_++) & 1);
			}
			word_string[k] = '\0';
			if (!strncmp(encode_bch(word_string, 28), word_string + 28, 12))
				crc_ok++;
		}
		if (crc_ok) {
			LOGP_CHAN(DFRAME, LOGL_NOTICE, "Seems we RX FOCC frame due to loopback, ignoring!\n");
			return 0;
		}
		bits_ -= 221;
	}

	for (crc_i = 0; crc_i < 5; crc_i++) {
		if (crc_a_ok[crc_i])
			break;
	}

	if (first) {
	/* result */
	if (crc_ok_count >= 3) {
		if (loglevel == LOGL_DEBUG || crc_ok_count > 0) {
			LOGP_CHAN(DFRAME, LOGL_INFO, "RX RECC: DCC=%d (%d of 5 CRCs are ok)\n", dcc, crc_ok_count);
			if (dcc != amps->si.dcc) {
				LOGP(DFRAME, LOGL_ERROR, "received DCC=%d mismatches the base station's DCC=%d\n", dcc, amps->si.dcc);
				return 0;
			}
		}
	} else if (crc_ok_count > 0) {
		LOGP_CHAN(DFRAME, LOGL_NOTICE, "RX RECC: Only %d of 5 CRCs are ok (Minimum 3 required)\n", crc_ok_count);
	}
	/* Note: crc_ok_count == 0 is silently ignored - usually SAT tone misinterpreted as FSK */
	if (loglevel == LOGL_DEBUG) {
		char text[64];

		for (i = 0; i < 5; i++) {
			strncpy(text, bits + i * 48, 48);
			text[48] = '\0';
			LOGP_CHAN(DFRAME, LOGL_DEBUG, "  word - %s%s\n", text, (crc_a_ok[i]) ? " ok" : " BAD CRC!");
		}
	}
	}

	if (crc_ok_count > 0)
		return amps_decode_word_recc(amps, word_a[crc_i], first);
	return 0;
}

int amps_decode_frame(amps_t *amps, const char *bits, int count, double level, double quality, int negative)
{
	int more = 0;

	/* not if additional words are received without sync */
	if (count != 240) {
		LOGP_CHAN(DDSP, LOGL_INFO, "RX Level: %.0f%% Quality: %.0f%% Polarity: %s\n", level * 100.0, quality * 100.0, (negative) ? "NEGATIVE" : "POSITIVE");
	}
	if (count == 441) {
		amps_decode_bits_focc(amps, bits);
	} else if (count == 247) {
		more = amps_decode_bits_recc(amps, bits, 1);
	} else if (count == 240) {
		more = amps_decode_bits_recc(amps, bits, 0);
	} else {
		LOGP_CHAN(DFRAME, LOGL_ERROR, "Frame with unknown length = %d, please fix!\n", count);
	}

	return more;
}

