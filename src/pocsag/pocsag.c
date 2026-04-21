/* POCSAG (Radio-Paging Code #1) processing
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

#define CHAN pocsag->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include <osmocom/cc/message.h>
#include "pocsag.h"
#include "frame.h"
#include "dsp.h"

static struct channel_info {
	double		freq_mhz;	/* frequency in megahertz */
	double		deviation_khz;	/* deviation in kilohertz */
	int		baudrate;	/* default baudrate */
        char		*name;		/* name of channel */
} channel_info[] = {
	{ 466.230,	-4.5,	1200,	"Scall" },
	{ 448.475,	-4.5,	1200,	"Quix" },
	{ 448.425,	-4.5,	1200,	"TeLMI" },
	{ 465.970,	-4.5,	1200,	"Skyper" },
	{ 466.075,	-4.5,	1200,	"Cityruf" },
	{ 466.075,	-4.5,	1200,	"Euromessage" },
	{ 439.9875,	-4.5,	1200,	"DAPNET" },
        { 0.0, 0.0, 0, NULL}
};

/*
 * German encoding table (unknown-unknown-german).
 * No specific pager model known for this codepage.
 * Same struct format as Cyrillic table for uniform handling.
 */
static const struct pocsag_charset_entry {
	uint8_t code;		/* 7-bit POCSAG value */
	char utf8[5];		/* UTF-8 representation */
} pocsag_german[] = {
	{ '@',  "\xc2\xa7" },		/* § → '@' (0x40) */
	{ '[',  "\xc3\x84" },		/* Ä → '[' (0x5B) */
	{ '\\', "\xc3\x96" },		/* Ö → '\' (0x5C) */
	{ ']',  "\xc3\x9c" },		/* Ü → ']' (0x5D) */
	{ '{',  "\xc3\xa4" },		/* ä → '{' (0x7B) */
	{ '|',  "\xc3\xb6" },		/* ö → '|' (0x7C) */
	{ '}',  "\xc3\xbc" },		/* ü → '}' (0x7D) */
	{ '~',  "\xc3\x9f" },		/* ß → '~' (0x7E) */
	{ 0, "" },			/* sentinel */
};

/*
 * Cyrillic encoding table (motorola-advisor_linguist-cyrillic).
 *
 * Motorola Advisor Linguist pager character set.
 * Only entries that differ from standard ASCII are listed.
 * For TX, the reverse mapping (UTF-8 → 7-bit) is done by searching this table.
 *
 * Format: { 7-bit value, UTF-8 byte sequence (up to 4 bytes), NUL terminated }
 *
 * Characters that map identically to ASCII (space, digits, punctuation, Latin
 * letters that look like Cyrillic) are NOT listed — they pass through unchanged.
 * Only the Cyrillic-specific mappings are here.
 */
static const struct pocsag_charset_entry pocsag_cyrillic[] = {
	/* Control character region: special symbols */
	{  1, "\xc2\xa4" },		/* ¤ → 0x01 */
	{  5, "\xc2\xa2" },		/* ¢ → 0x05 */
	{  6, "\xc3\x97" },		/* × → 0x06 */
	{  7, "\xc3\xb7" },		/* ÷ → 0x07 */
	{  8, "\xc2\xa3" },		/* £ → 0x08 */
	{  9, "\xc2\xb0" },		/* ° → 0x09 */
	{ 11, "\xc2\xab" },		/* « → 0x0B */
	{ 12, "\xc2\xbf" },		/* ¿ → 0x0C */
	{ 14, "\xc2\xbb" },		/* » → 0x0E */
	{ 15, "\xc2\xa1" },		/* ¡ → 0x0F */
	{ 16, "\\" },			/* \ → 0x10 */
	{ 17, "\xc2\xaf" },		/* ¯ → 0x11 */
	{ 18, "^" },			/* ^ → 0x12 */
	{ 19, "_" },			/* _ → 0x13 */
	{ 20, "\xc2\xa7" },		/* § → 0x14 */
	{ 21, "{" },			/* { → 0x15 */
	{ 22, "|" },			/* | → 0x16 */
	{ 24, "}" },			/* } → 0x18 */
	{ 25, "\xe2\x84\x96" },	/* № → 0x19 */
	{ 26, "\xc2\xb7" },		/* · → 0x1A */
	{ 28, "\xc2\xa5" },		/* ¥ → 0x1C */
	{ 29, "\xc2\xb1" },		/* ± → 0x1D */
	/* Lowercase region: Cyrillic letters mapped to 'a'-'z' range */
	{ 'a', "\xd0\x91" },		/* Б → 'a' (0x61) */
	{ 'b', "\xd0\x93" },		/* Г → 'b' (0x62) */
	{ 'c', "\xd0\x83" },		/* Ѓ → 'c' (0x63) */
	{ 'd', "\xd0\x94" },		/* Д → 'd' (0x64) */
	{ 'e', "\xd0\x81" },		/* Ё → 'e' (0x65) */
	{ 'f', "\xd0\x96" },		/* Ж → 'f' (0x66) */
	{ 'g', "\xd0\x97" },		/* З → 'g' (0x67) */
	{ 'h', "\xd0\x98" },		/* И → 'h' (0x68) */
	{ 'i', "\xd0\x99" },		/* Й → 'i' (0x69) */
	{ 'j', "\xd0\x9b" },		/* Л → 'j' (0x6A) */
	{ 'k', "\xd0\x9f" },		/* П → 'k' (0x6B) */
	{ 'l', "\xd0\xa3" },		/* У → 'l' (0x6C) */
	{ 'm', "\xd0\xa4" },		/* Ф → 'm' (0x6D) */
	{ 'n', "\xd0\xa6" },		/* Ц → 'n' (0x6E) */
	{ 'o', "\xd0\xa7" },		/* Ч → 'o' (0x6F) */
	{ 'p', "\xd0\xa8" },		/* Ш → 'p' (0x70) */
	{ 'q', "\xd0\xa9" },		/* Щ → 'q' (0x71) */
	{ 'r', "\xd0\xaa" },		/* Ъ → 'r' (0x72) */
	{ 's', "\xd0\xab" },		/* Ы → 's' (0x73) */
	{ 't', "\xd0\xac" },		/* Ь → 't' (0x74) */
	{ 'u', "\xd0\xad" },		/* Э → 'u' (0x75) */
	{ 'v', "\xd0\xae" },		/* Ю → 'v' (0x76) */
	{ 'w', "\xd0\xaf" },		/* Я → 'w' (0x77) */
	{ 'x', "\xd0\x82" },		/* Ђ → 'x' (0x78) */
	{ 'y', "\xd0\x84" },		/* Є → 'y' (0x79) */
	{ 'z', "\xd0\x87" },		/* Ї → 'z' (0x7A) */
	/* Symbols remapped in Cyrillic mode */
	{ '\\', "\xd0\x89" },		/* Љ → '\' (0x5C) */
	{ '^', "\xd0\x8a" },		/* Њ → '^' (0x5E) */
	{ '_', "\xd0\x8b" },		/* Ћ → '_' (0x5F) */
	{ '{', "\xd0\x8c" },		/* Ќ → '{' (0x7B) */
	{ '|', "\xd0\x8e" },		/* Ў → '|' (0x7C) */
	{ '}', "\xd0\x8f" },		/* Џ → '}' (0x7D) */
	/* Uppercase Latin letters that ARE Cyrillic lookalikes:
	 * A=А, B=В, C=С, E=Е, H=Н, I=І, K=К, M=М, O=О, P=Р, T=Т, X=Х
	 * These map to the same 7-bit code, so on RX we output the Cyrillic Unicode.
	 */
	{ 'A', "\xd0\x90" },		/* А → 'A' */
	{ 'B', "\xd0\x92" },		/* В → 'B' */
	{ 'C', "\xd0\xa1" },		/* С → 'C' */
	{ 'E', "\xd0\x95" },		/* Е → 'E' */
	{ 'H', "\xd0\x9d" },		/* Н → 'H' */
	{ 'I', "\xd0\x86" },		/* І → 'I' */
	{ 'K', "\xd0\x9a" },		/* К → 'K' */
	{ 'M', "\xd0\x9c" },		/* М → 'M' */
	{ 'O', "\xd0\x9e" },		/* О → 'O' */
	{ 'P', "\xd0\xa0" },		/* Р → 'P' */
	{ 'T', "\xd0\xa2" },		/* Т → 'T' */
	{ 'X', "\xd0\xa5" },		/* Х → 'X' */
	{ 0, "" },			/* sentinel */
};


void pocsag_list_channels(void)
{
        int i;
	char text[16];

        for (i = 0; channel_info[i].name; i++) {
                if (i == 0) {
                        printf("\nFrequency\tDeviation\tPolarity\tBaudrate\tChannel Name\n");
                        printf("--------------------------------------------------------------------------------\n");
                }
		if (channel_info[i].freq_mhz * 1e3 == floor(channel_info[i].freq_mhz * 1e3))
			sprintf(text, "%.3f MHz", channel_info[i].freq_mhz);
		else
			sprintf(text, "%.4f MHz", channel_info[i].freq_mhz);
                printf("%s\t%.3f KHz\t%s\t%d\t\t%s\n", text, fabs(channel_info[i].deviation_khz), (channel_info[i].deviation_khz < 0) ? "negative" : "positive", channel_info[i].baudrate, channel_info[i].name);
        }
        printf("-> Give channel name or any frequency in MHz.\n");
        printf("\n");
}

/* Convert channel name to frequency number of base station. */
double pocsag_channel2freq(const char *kanal, double *deviation, double *polarity, int *baudrate)
{
        int i;

        for (i = 0; channel_info[i].name; i++) {
                if (!strcasecmp(channel_info[i].name, kanal)) {
			if (deviation)
				*deviation = fabs(channel_info[i].deviation_khz) * 1e3;
			if (polarity)
				*polarity = (channel_info[i].deviation_khz > 0) ? 1.0 : -1.0;
			if (baudrate)
				*baudrate = channel_info[i].baudrate;
			return channel_info[i].freq_mhz * 1e6;
                }
        }

	return atof(kanal) * 1e6;
}

const char *pocsag_state_name[] = {
	"IDLE",
	"PREAMBLE",
	"MESSAGE",
	"SILENCE",
};

/*
 * Function names: These are the 2-bit sub-address values (0-3 or A-D).
 * Per POCSAG standard (CCIR Rec. 584), these have NO mandatory correlation
 * to message type. They simply provide 4 sub-addresses per RIC.
 */
const char *pocsag_function_name[4] = {
	"A",
	"B",
	"C",
	"D",
};

/*
 * Message type names: These define how the message content is encoded.
 * This is INDEPENDENT of the function bits per the POCSAG standard.
 */
static const char *pocsag_msg_type_names[] = {
	"auto",
	"tone",
	"numeric",
	"alpha",
};

int pocsag_function_name2value(const char *text)
{
	int i;

	for (i = 0; i < 4; i++) {
		if (!strcasecmp(pocsag_function_name[i], text))
			return i;
		if (text[0] == '0' + i && text[1] == '\0')
			return i;
		if (text[0] == 'A' + i && text[1] == '\0')
			return i;
		if (text[0] == 'a' + i && text[1] == '\0')
			return i;
	}

	return -EINVAL;
}

/*
 * Convert message type name to value.
 * Message type controls encoding (numeric/alpha), independent of function bits.
 */
int pocsag_msg_type_name2value(const char *text)
{
	if (!strcasecmp(text, "auto") || !strcmp(text, "0"))
		return POCSAG_MSG_TYPE_AUTO;
	if (!strcasecmp(text, "tone") || !strcmp(text, "1"))
		return POCSAG_MSG_TYPE_TONE;
	if (!strcasecmp(text, "numeric") || !strcasecmp(text, "num") || !strcmp(text, "2"))
		return POCSAG_MSG_TYPE_NUMERIC;
	if (!strcasecmp(text, "alpha") || !strcasecmp(text, "alphanumeric") || !strcmp(text, "3"))
		return POCSAG_MSG_TYPE_ALPHA;

	return -EINVAL;
}

const char *pocsag_msg_type_name(enum pocsag_msg_type type)
{
	if (type >= 0 && type < 4)
		return pocsag_msg_type_names[type];
	return "unknown";
}

/*
 * Check if number is a valid station ID (RIC + optional function).
 *
 * RIC (Radio Identity Code) is a 21-bit pager address:
 *   - Valid range: 0 to 2,097,151 (2^21 - 1)
 *   - 3 LSBs determine frame position (RIC & 7)
 *   - 18 MSBs are transmitted in address codeword (RIC >> 3)
 *
 * Reserved RICs:
 *   - 2007664-2007671: These RICs have address bits matching the idle
 *     codeword (0x7A89C197). The idle codeword's address portion is
 *     extracted as: (0x7A89C197 >> 10) & 0x1FFFF8 = 2007664.
 *     Using these would cause pagers to misinterpret idle codewords
 *     as valid pages.
 *
 * Format: 7-digit RIC, optionally followed by function (0-3 or A-D)
 */
const char *pocsag_number_valid(const char *number)
{
	int i;
	int ric = 0;

	/* assume that the number has valid length(s) and digits */

	/* Parse 7-digit RIC (0-2097151, but entered as 7 digits with leading zeros) */
	for (i = 0; i < 7; i++) {
		if (number[i] < '0' || number[i] > '9')
			return "Illegal RIC digit (Use 0..9 only)";
		ric = ric * 10 + number[i] - '0';
	}

	/* Validate RIC range: must fit in 21 bits */
	if (ric > 2097151)
		return "Maximum allowed RIC is (2^21)-1. (2097151)";

	/*
	 * Check for reserved idle codeword RIC range.
	 * The idle codeword is 0x7A89C197. When decoded as an address codeword,
	 * (0x7A89C197 >> 10) & 0x1FFFF8 = 2007664. This mask extracts the
	 * 18 address bits with the lower 3 frame bits cleared.
	 * RICs 2007664-2007671 (all 8 frame positions) are therefore reserved,
	 * as they would be indistinguishable from idle codewords.
	 */
	if ((ric & 0xfffffff8) == 2007664)
		return "Illegal RIC 2007664-2007671. (Reserved for idle codeword)";

	/* Optional 8th character: function/sub-address (0-3 or A-D) */
	if (number[7] && !(number[7] >= '0' && number[7] <= '3') && !(number[7] >= 'A' && number[7] <= 'D'))
		return "Illegal function digit #8 (Use 0..3 or A..D only)";
	return NULL;
}

int pocsag_init(void)
{
	return 0;
}

void pocsag_exit(void)
{
}

static const char *print_ric(pocsag_msg_t *msg)
{
	static char text[16];

	sprintf(text, "%07d/%c", msg->ric, msg->function + '0');

	return text;
}

static void pocsag_display_status(void)
{
	sender_t *sender;
	pocsag_t *pocsag;
	pocsag_msg_t *msg;

	display_status_start();
	for (sender = sender_head; sender; sender = sender->next) {
		pocsag = (pocsag_t *) sender;
		display_status_channel(pocsag->sender.kanal, NULL, pocsag_state_name[pocsag->state]);
		{
			int p, s, f;
			for (p = 0; p < POCSAG_NUM_POL; p++)
				for (s = 0; s < POCSAG_NUM_SPD; s++)
					for (f = 0; f < 8; f++)
						for (msg = pocsag->msg_list[p][s][f]; msg; msg = msg->next)
							display_status_subscriber(print_ric(msg), NULL);
		}
	}
	display_status_end();
}

void pocsag_new_state(pocsag_t *pocsag, enum pocsag_state new_state)
{
	if (pocsag->state == new_state)
		return;
	LOGP(DPOCSAG, LOGL_DEBUG, "State change: %s -> %s\n", pocsag_state_name[pocsag->state], pocsag_state_name[new_state]);
	pocsag->state = new_state;
	pocsag_display_status();
}

/*
 * Create msg instance.
 *
 * Note: function (sub-address) and msg_type (encoding) are INDEPENDENT
 * per the POCSAG standard. The function bits provide 4 sub-addresses,
 * while msg_type determines how the message content is encoded.
 */
static pocsag_msg_t *pocsag_msg_create(pocsag_t *pocsag, uint32_t callref, uint32_t ric, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message, size_t message_length)
{
	pocsag_msg_t *msg, **msgp;

	LOGP(DPOCSAG, LOGL_INFO, "Creating msg instance to page RIC '%d' / function '%s' / type '%s'.\n",
	     ric, pocsag_function_name[function], pocsag_msg_type_name(msg_type));

	/* create */
	msg = calloc(1, sizeof(*msg));
	if (!msg) {
		LOGP(DPOCSAG, LOGL_ERROR, "No mem!\n");
		abort();
	}
	if (message_length > sizeof(msg->data)) {
		LOGP(DPOCSAG, LOGL_ERROR, "Text too long!\n");
		message_length = sizeof(msg->data);
	}

	/* init */
	msg->callref = callref;
	msg->ric = ric;
	msg->function = function;
	msg->msg_type = msg_type;
	memcpy(msg->data, message, message_length);
	msg->data_length = message_length;
	msg->padding = pocsag->padding;
	msg->speed = pocsag->default_speed;
	msg->polarity = pocsag->default_polarity;

	/* link to per-frame, per-polarity queue */
	msg->pocsag = pocsag;
	{
		uint8_t frame = ric & 7;
		int pol = pocsag_pol_index(msg->polarity);
		int spd = pocsag_speed_index(msg->speed);
		msgp = &pocsag->msg_list[pol][spd][frame];
		while ((*msgp))
			msgp = &(*msgp)->next;
		(*msgp) = msg;
	}

	/* kick transmitter */
	if (pocsag->state == POCSAG_IDLE) {
		pocsag->tx_speed = msg->speed;
		pocsag->tx_polarity = msg->polarity;
		pocsag->batch_count = 0;
		pocsag->msg_count = 0;
		pocsag_new_state(pocsag, POCSAG_PREAMBLE);
		pocsag->word_count = 0;
	} else
		pocsag_display_status();

	return msg;
}

/* Destroy msg instance */
void pocsag_msg_destroy(pocsag_msg_t *msg)
{
	pocsag_msg_t **msgp;

	/* unlink from per-frame, per-polarity queue */
	{
		uint8_t frame = msg->ric & 7;
		int pol = pocsag_pol_index(msg->polarity);
		int spd = pocsag_speed_index(msg->speed);
		msgp = &msg->pocsag->msg_list[pol][spd][frame];
		while ((*msgp) != msg)
			msgp = &(*msgp)->next;
		(*msgp) = msg->next;
	}

	/* remove from current transmitting message */
	if (msg == msg->pocsag->current_msg)
		msg->pocsag->current_msg = NULL;

	/* destroy */
	free(msg);

	/* update display */
	pocsag_display_status();
}

static int pocsag_scan_or_loopback(pocsag_t *pocsag)
{
	if (pocsag->scan_from < pocsag->scan_to) {
		char message[16];
		uint32_t ric;
		int queued = 0;

		/*
		 * Batch-fill: enqueue one message per frame slot.
		 * With per-frame queues, we simply check if the target
		 * frame's queue is empty before enqueuing.
		 */
		int scan_pol = pocsag_pol_index(pocsag->default_polarity);
		int scan_spd = pocsag_speed_index(pocsag->default_speed);
		for (ric = pocsag->scan_from; ric < pocsag->scan_to; ric++) {
			uint8_t frame = ric & 7;

			/* skip reserved idle codeword RICs (2007664-2007671) */
			if ((ric & 0xfffffff8) == 2007664) {
				pocsag->scan_from = ric + 1;
				continue;
			}

			/* stop if this frame already has a pending message */
			if (pocsag->msg_list[scan_pol][scan_spd][frame])
				break;

			/*
			 * Generate scan message based on configured message type.
			 * Encode the RIC into the message so the receiver can
			 * identify which capcode was hit.
			 */
			switch (pocsag->default_msg_type) {
			case POCSAG_MSG_TYPE_NUMERIC:
				/* 5 digits: upper portion of RIC (RIC/100) */
				sprintf(message, "%05d", ric / 100);
				break;
			case POCSAG_MSG_TYPE_ALPHA:
				/* full RIC as decimal string */
				sprintf(message, "%d", ric);
				break;
			case POCSAG_MSG_TYPE_TONE:
			case POCSAG_MSG_TYPE_AUTO:
			default:
				message[0] = '\0';
			}
			LOGP_CHAN(DPOCSAG, LOGL_NOTICE, "Scan: enqueue RIC %d / function '%s' / type '%s' / msg '%s'\n",
				  ric, pocsag_function_name[pocsag->default_function],
				  pocsag_msg_type_name(pocsag->default_msg_type), message);
			pocsag_msg_create(pocsag, 0, ric, pocsag->default_function,
					  pocsag->default_msg_type, message, strlen(message));
			queued++;
		}

		/* only advance past RICs we actually enqueued */
		pocsag->scan_from = pocsag->scan_from + queued;

		return queued > 0;
	}

	if (pocsag->sender.loopback) {
		LOGP(DPOCSAG, LOGL_INFO, "Sending message for loopback test.\n");
		pocsag_msg_create(pocsag, 0, 1234567, POCSAG_FUNCTION_A, POCSAG_MSG_TYPE_NUMERIC, "1234", 4);
		return 1;
	}

	return 0;
}

/*
 * Return the charset table for a given language, or NULL for default/skyper.
 */
static const struct pocsag_charset_entry *pocsag_charset_table(enum pocsag_language language)
{
	switch (language) {
	case LANGUAGE_GERMAN:	return pocsag_german;
	case LANGUAGE_CYRILLIC:	return pocsag_cyrillic;
	default:		return NULL;
	}
}

/*
 * Handle received message.
 *
 * Note: function (sub-address) and msg_type (encoding) are reported separately
 * as they are independent per the POCSAG standard.
 */
void pocsag_msg_receive(enum pocsag_language language, const char *channel, uint32_t ric, enum pocsag_function function, enum pocsag_msg_type msg_type, int baudrate, double polarity, const char *message)
{
	int msg_len = message ? strlen(message) : 0;
	char text[256 + msg_len * 4], *p;
	struct timeval tv;
	struct tm *tm;
	const struct pocsag_charset_entry *table;
	int i, j;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);

	sprintf(text, "%04d-%02d-%02d %02d:%02d:%02d.%03d @%s %d%s,%s,%d,%s",
		tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec, (int)(tv.tv_usec / 10000.0),
		channel, ric, pocsag_function_name[function], pocsag_msg_type_name(msg_type),
		baudrate, (polarity < 0) ? "normal" : "inverted");
	p = strchr(text, '\0');

	if (message && message[0]) {
		*p++ = ',';

		if (language == LANGUAGE_SKYPER) {
			/* Skyper ROT-1 decode: subtract 1 from each 7-bit value */
			for (i = 0; message[i]; i++) {
				unsigned char c = (unsigned char)message[i] & 0x7f;
				if (c == 0)
					c = 0x7f;
				else
					c = c - 1;
				*p++ = (char)c;
			}
		} else if ((table = pocsag_charset_table(language)) != NULL) {
			/* Table-based decode: 7-bit → UTF-8 */
			for (i = 0; message[i]; i++) {
				unsigned char c = (unsigned char)message[i] & 0x7f;
				for (j = 0; table[j].code; j++) {
					if (table[j].code == c)
						break;
				}
				if (table[j].code) {
					strcpy(p, table[j].utf8);
					p += strlen(p);
				} else
					*p++ = (char)c;
			}
		} else {
			/* Default: pass through */
			strcpy(p, message);
			p += strlen(p);
		}
	}

	*p++ = '\0';

	msg_receive(text);
}

/*
 * Create transceiver instance and link to a list.
 *
 * Note: function (sub-address 0-3) and msg_type (encoding) are independent
 * per the POCSAG standard. The function bits provide 4 sub-addresses per RIC,
 * while msg_type determines how the message content is encoded/decoded.
 */
int pocsag_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int tx, int rx, enum pocsag_language language, int baudrate, double deviation, double polarity, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message, char padding, uint32_t scan_from, uint32_t scan_to, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback, int auto_baud, int auto_polarity, double dedup_window, int max_batches, int network_mode)
{
	pocsag_t *pocsag;
	int rc;

	pocsag = calloc(1, sizeof(*pocsag));
	if (!pocsag) {
		LOGP(DPOCSAG, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	LOGP(DPOCSAG, LOGL_DEBUG, "Creating 'POCSAG' instance for 'Kanal' = %s (sample rate %d).\n", kanal, samplerate);

	/* init general part of transceiver */
	rc = sender_create(&pocsag->sender, kanal, frequency, frequency, device, use_sdr, samplerate, rx_gain, tx_gain, 0, 0, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DPOCSAG, LOGL_ERROR, "Failed to init transceiver process!\n");
		goto error;
	}

	/* init audio processing */
	rc = dsp_init_sender(pocsag, samplerate, (double)baudrate, deviation, polarity, auto_baud, auto_polarity);
	if (rc < 0) {
		LOGP(DPOCSAG, LOGL_ERROR, "Failed to init audio processing!\n");
		goto error;
	}

	pocsag->tx = tx;
	pocsag->rx = rx;
	pocsag->language = language;
	pocsag->default_function = function;
	pocsag->default_msg_type = msg_type;
	pocsag->default_message = message;
	pocsag->scan_from = scan_from;
	pocsag->scan_to = scan_to;
	pocsag->padding = padding;
	pocsag->default_speed = baudrate;
	pocsag->default_polarity = polarity;
	pocsag->max_batches = max_batches;
	pocsag->network_mode = network_mode;
	pocsag->rx_dedup_window = dedup_window;

	pocsag_display_status();

	LOGP(DPOCSAG, LOGL_NOTICE, "Created 'Kanal' %s\n", kanal);

	/* In network mode, start transmitting immediately (preamble + idle batches) */
	if (network_mode && tx) {
		pocsag->tx_speed = baudrate;
		pocsag->tx_polarity = polarity;
		pocsag->batch_count = 0;
		pocsag->msg_count = 0;
		pocsag_new_state(pocsag, POCSAG_PREAMBLE);
		pocsag->word_count = 0;
		LOGP(DPOCSAG, LOGL_INFO, "Network mode: starting continuous transmission.\n");
	}

	pocsag_scan_or_loopback(pocsag);

	return 0;

error:
	pocsag_destroy(&pocsag->sender);

	return rc;
}

/* Destroy transceiver instance and unlink from list. */
void pocsag_destroy(sender_t *sender)
{
	pocsag_t *pocsag = (pocsag_t *) sender;

	LOGP(DPOCSAG, LOGL_DEBUG, "Destroying 'POCSAG' instance for 'Kanal' = %s.\n", sender->kanal);

	{
		int p, s, f;
		for (p = 0; p < POCSAG_NUM_POL; p++)
			for (s = 0; s < POCSAG_NUM_SPD; s++)
				for (f = 0; f < 8; f++)
					while (pocsag->msg_list[p][s][f])
						pocsag_msg_destroy(pocsag->msg_list[p][s][f]);
	}
	dsp_cleanup_sender(pocsag);

	/* free dynamic rx message buffers */
	free(pocsag->rx_msg_data);
	free(pocsag->rx_msg_char_status);
	free(pocsag->rx_msg_data_skyper);
	free(pocsag->rx_msg_data_numeric);
	free(pocsag->rx_msg_num_status);
	free(pocsag->rx_msg_data_raw);

	sender_destroy(&pocsag->sender);
	free(pocsag);
}

/*
 * Application sends us a message, we need to deliver.
 *
 * Format: RIC,function,type[,encoding,message] or RIC,function,type[,message]
 *
 * Per the POCSAG standard (CCIR Rec. 584):
 *   - RIC: 18-bit address (0-2097151, excluding idle codeword range)
 *   - Function: 2-bit sub-address (0-3 or A-D), provides 4 addresses per RIC
 *   - Type: Message encoding (auto/tone/numeric/alpha) - INDEPENDENT of function
 *   - Encoding: Optional codepage (default/unknown-unknown-german/nec-skyper-categories/motorola-advisor_linguist-cyrillic).
 *              If omitted, uses the global -L setting.
 *   - Message: Optional message content
 *
 * The function bits and message type have NO mandatory correlation per the
 * POCSAG standard. Any correlation is manufacturer/pager specific.
 */
static int pocsag_language_name2value(const char *text)
{
	if (!strcasecmp(text, "default") || !strcmp(text, "0"))
		return LANGUAGE_DEFAULT;
	if (!strcasecmp(text, "unknown-unknown-german") || !strcmp(text, "1"))
		return LANGUAGE_GERMAN;
	if (!strcasecmp(text, "nec-skyper-categories") || !strcmp(text, "2"))
		return LANGUAGE_SKYPER;
	if (!strcasecmp(text, "motorola-advisor_linguist-cyrillic") || !strcmp(text, "3"))
		return LANGUAGE_CYRILLIC;
	return -1;
}

/*
 * Parse space-separated key=value options from FIFO message.
 * Modifies language in-place if specified.
 */
static void parse_pocsag_options(const char *opts, int opts_len,
				 enum pocsag_language *language_out,
				 int *speed_out, double *polarity_out,
				 int *repeat_out, double *delay_out,
				 double *interval_out)
{
	char buf[256];
	char *p, *key, *val;
	int len, rc;

	if (opts_len <= 0)
		return;

	len = opts_len;
	if (len >= (int)sizeof(buf))
		len = sizeof(buf) - 1;
	memcpy(buf, opts, len);
	buf[len] = '\0';

	p = buf;
	while (*p) {
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

		if (!strcmp(key, "charset") || !strcmp(key, "lang")) {
			rc = pocsag_language_name2value(val);
			if (rc >= 0)
				*language_out = rc;
			else
				LOGP(DNMT, LOGL_NOTICE, "FIFO: invalid charset '%s', use default/unknown-unknown-german/nec-skyper-categories/motorola-advisor_linguist-cyrillic.\n", val);
		}
		else if (!strcmp(key, "speed")) {
			int spd = atoi(val);
			if (spd != 512 && spd != 1200 && spd != 2400)
				LOGP(DNMT, LOGL_NOTICE, "FIFO: invalid speed '%s', use 512/1200/2400.\n", val);
			else
				*speed_out = spd;
		}
		else if (!strcmp(key, "polarity")) {
			if (val[0] == 'n' || val[0] == 'N')
				*polarity_out = -1.0;
			else if (val[0] == 'i' || val[0] == 'I')
				*polarity_out = 1.0;
			else if (val[0] == 'p' || val[0] == 'P')
				*polarity_out = 1.0;  /* legacy "positive" = inverted */
			else
				LOGP(DNMT, LOGL_NOTICE, "FIFO: invalid polarity '%s', use normal/inverted.\n", val);
		}
		else if (!strcmp(key, "repeat")) {
			int r = atoi(val);
			if (r < 0 || r > 10)
				LOGP(DNMT, LOGL_NOTICE, "FIFO: invalid repeat '%s', use 0-10.\n", val);
			else
				*repeat_out = r;
		}
		else if (!strcmp(key, "delay")) {
			double d = atof(val);
			if (d < 0.0)
				LOGP(DNMT, LOGL_NOTICE, "FIFO: invalid delay '%s', must be >= 0.\n", val);
			else
				*delay_out = d;
		}
		else if (!strcmp(key, "interval")) {
			double iv = atof(val);
			if (iv <= 0.0)
				LOGP(DNMT, LOGL_NOTICE, "FIFO: invalid interval '%s', must be > 0.\n", val);
			else
				*interval_out = iv;
		}
	}
}

/*
 * Application sends us a message via FIFO, we need to deliver.
 *
 * Format: capcode,type,options,message
 *
 * Fields (always 4, comma-separated):
 *   - capcode: RIC with optional function suffix, e.g. 1234567A or 1234567
 *       RIC: 0-2097151 (21-bit pager address)
 *       Function: A-D or 0-3 suffix (default A if omitted)
 *   - type: auto|tone|numeric|alpha (or 0-3)
 *   - options: space-separated key=value pairs (can be empty):
 *       charset=default|german|skyper|cyrillic  (overrides -L)
 *       speed=512|1200|2400  (reserved for future use)
 *       polarity=normal|inverted  (reserved for future use)
 *   - message: message content (may contain escape sequences)
 *
 * Examples:
 *   1234567A,alpha,,Hello World
 *   1234567,numeric,,12345
 *   1234567B,alpha,charset=motorola-advisor_linguist-cyrillic,Привіт
 *   1234567C,alpha,charset=nec-skyper-categories,Hello
 *   1234567,tone,,
 */
void pocsag_msg_send(enum pocsag_language language, const char *text, size_t text_length)
{
	char capcode_string[text_length + 1];
	char type_string[text_length + 1];
	char message[text_length + 1];
	uint32_t ric;
	enum pocsag_function function;
	enum pocsag_msg_type msg_type;
	pocsag_t *pocsag;
	int message_length = 0;
	int i, ii, j;
	int rc;
	int comma_count = 0;
	int comma1 = -1, comma2 = -1, comma3 = -1;
	const char *opts_start;
	int opts_len;

	pocsag = (pocsag_t *) sender_head;
	if (!pocsag) {
		LOGP(DNMT, LOGL_ERROR, "No transmitter instance!\n");
		return;
	}

	/* Function is parsed from capcode below, not from instance default */
	/* Count commas and record positions */
	for (i = 0; i < (int)text_length; i++) {
		if (text[i] == ',') {
			comma_count++;
			if (comma_count == 1)
				comma1 = i;
			else if (comma_count == 2)
				comma2 = i;
			else if (comma_count == 3) {
				comma3 = i;
				break;
			}
		}
	}

	if (comma_count < 3) {
		LOGP(DNMT, LOGL_NOTICE, "Given message MUST be in the format: capcode,type,options,message\n");
		LOGP(DNMT, LOGL_NOTICE, "  capcode: RIC with optional function, e.g. 1234567A (default A)\n");
		LOGP(DNMT, LOGL_NOTICE, "  type: auto|tone|numeric|alpha (or 0-3)\n");
		LOGP(DNMT, LOGL_NOTICE, "  options: space-separated key=value (can be empty):\n");
		LOGP(DNMT, LOGL_NOTICE, "    charset=default|unknown-unknown-german|nec-skyper-categories|motorola-advisor_linguist-cyrillic\n");
		LOGP(DNMT, LOGL_NOTICE, "  message: text content\n");
		LOGP(DNMT, LOGL_NOTICE, "  Example: 1234567A,alpha,,Hello World\n");
		LOGP(DNMT, LOGL_NOTICE, "  Example: 1234567B,alpha,charset=motorola-advisor_linguist-cyrillic,Привіт\n");
		return;
	}

	/* Extract capcode field (RIC + optional function suffix) */
	memcpy(capcode_string, text, comma1);
	capcode_string[comma1] = '\0';

	/* Parse RIC and function from capcode.
	 * Format: digits followed by optional A-D or 0-3 suffix.
	 * Default function is A if not specified. */
	{
		int clen = strlen(capcode_string);
		char last = (clen > 0) ? capcode_string[clen - 1] : '\0';

		if ((last >= 'A' && last <= 'D') || (last >= 'a' && last <= 'd')) {
			if (last >= 'a')
				function = last - 'a';
			else
				function = last - 'A';
			capcode_string[clen - 1] = '\0';
		} else if (clen > 7 && last >= '0' && last <= '3') {
			/* Only treat trailing digit as function if capcode is >7 chars,
			 * to distinguish from 7-digit RICs ending in 0-3 */
			function = last - '0';
			capcode_string[clen - 1] = '\0';
		} else {
			function = POCSAG_FUNCTION_A;
		}
		ric = atoi(capcode_string);
	}

	/* Extract type field */
	{
		int tlen = comma2 - comma1 - 1;
		memcpy(type_string, text + comma1 + 1, tlen);
		type_string[tlen] = '\0';
	}

	/* Extract options field */
	opts_start = text + comma2 + 1;
	opts_len = comma3 - comma2 - 1;

	/* Extract message payload (everything after third comma) */
	if (comma3 + 1 < (int)text_length)
		message_length = scan_message(text + comma3 + 1, text_length - comma3 - 1, message, sizeof(message));

	/* Parse options */
	int repeat = 0;
	int msg_speed = 0;		/* 0 = use default */
	double msg_polarity = 0.0;	/* 0.0 = use default */
	double delay = 0.0;
	double interval = 10.0;
	parse_pocsag_options(opts_start, opts_len, &language, &msg_speed, &msg_polarity, &repeat, &delay, &interval);

	/* Validate RIC */
	if (ric > 2097151) {
		LOGP(DNMT, LOGL_NOTICE, "Illegal RIC %d. Maximum allowed RIC is (2^21)-1. (2097151)\n", ric);
		return;
	}
	if ((ric & 0xfffffff8) == 2007664) {
		LOGP(DNMT, LOGL_NOTICE, "Illegal RIC %d. (Reserved for idle codeword, range 2007664-2007671)\n", ric);
		return;
	}

	/* Validate message type */
	rc = pocsag_msg_type_name2value(type_string);
	if (rc < 0) {
		LOGP(DNMT, LOGL_NOTICE, "Illegal type '%s'. Use auto/tone/numeric/alpha.\n", type_string);
		return;
	}
	msg_type = rc;

	/* Auto-detect message type */
	if (msg_type == POCSAG_MSG_TYPE_AUTO) {
		if (message_length == 0) {
			msg_type = POCSAG_MSG_TYPE_TONE;
		} else {
			int all_numeric = 1;
			for (i = 0; i < message_length; i++) {
				char c = message[i];
				if (!((c >= '0' && c <= '9') || c == 'R' || c == 'U' ||
				      c == ' ' || c == '-' || c == '[' || c == ']')) {
					all_numeric = 0;
					break;
				}
			}
			msg_type = all_numeric ? POCSAG_MSG_TYPE_NUMERIC : POCSAG_MSG_TYPE_ALPHA;
		}
	}

	/* Validate numeric message characters */
	if (msg_type == POCSAG_MSG_TYPE_NUMERIC && message_length > 0) {
		for (i = 0; i < message_length; i++) {
			char c = message[i];
			if (!((c >= '0' && c <= '9') || c == 'R' || c == 'U' ||
			      c == ' ' || c == '-' || c == '[' || c == ']')) {
				LOGP(DNMT, LOGL_ERROR, "Invalid character in numeric message: '%c'\n", c);
				LOGP(DNMT, LOGL_ERROR, "Valid numeric characters are: 0-9 R U space - [ ]\n");
				return;
			}
		}
	}

	/* Handle language-specific character encoding (UTF-8 → 7-bit) */
	if (message_length && language == LANGUAGE_SKYPER) {
		for (i = 0; i < message_length; i++) {
			unsigned char c = (unsigned char)message[i] & 0x7f;
			message[i] = (c + 1) & 0x7f;
		}
	} else if (message_length) {
		const struct pocsag_charset_entry *table = pocsag_charset_table(language);
		if (table) {
			i = 0;
			for (ii = 0; ii < message_length; i++) {
				int found = 0;
				for (j = 0; table[j].code; j++) {
					int ulen = strlen(table[j].utf8);
					if (ulen == 0)
						continue;
					if (ii + ulen > message_length)
						continue;
					if (memcmp(&message[ii], table[j].utf8, ulen) == 0) {
						message[i] = (char)table[j].code;
						ii += ulen;
						found = 1;
						break;
					}
				}
				if (!found)
					message[i] = message[ii++];
			}
			message_length = i;
		}
	}

	LOGP(DNMT, LOGL_INFO, "FIFO: enqueued RIC=%d type=%s function=%s charset=%d len=%d repeat=%d delay=%.1f interval=%.1f\n",
	     ric, pocsag_msg_type_name(msg_type), pocsag_function_name[function],
	     language, message_length, repeat, delay, interval);

	{
		pocsag_msg_t *msg;
		struct timeval tv;

		msg = pocsag_msg_create(pocsag, 0, ric, function, msg_type, message, message_length);
		if (msg) {
			msg->retransmit_max = repeat;
			msg->retransmit_count = 0;
			msg->retransmit_interval = interval;
			msg->send_delay = delay;
			if (msg_speed)
				msg->speed = msg_speed;
			if (msg_polarity != 0.0)
				msg->polarity = msg_polarity;

			/* If locked mode (max_batches=0), discard messages with wrong speed/polarity */
			if (pocsag->max_batches == 0 &&
			    (msg->speed != pocsag->default_speed || msg->polarity != pocsag->default_polarity)) {
				LOGP(DNMT, LOGL_NOTICE, "FIFO: discarding RIC %d — speed %d/%s does not match locked %d/%s (--max-batches 0).\n",
				     ric, msg->speed, (msg->polarity < 0) ? "normal" : "inverted",
				     pocsag->default_speed, (pocsag->default_polarity < 0) ? "normal" : "inverted");
				pocsag_msg_destroy(msg);
				return;
			}

			gettimeofday(&tv, NULL);
			msg->next_send_time = (double)tv.tv_sec + tv.tv_usec / 1e6 + delay;
		}
	}
}

void call_down_clock(void)
{
	sender_t *sender;
	pocsag_t *pocsag;
	pocsag_msg_t *msg;
	struct timeval tv;
	double now;

	gettimeofday(&tv, NULL);
	now = (double)tv.tv_sec + tv.tv_usec / 1e6;

	/* Check if any deferred retransmission has become eligible.
	 * If the transmitter is idle and a message is ready, kick it. */
	for (sender = sender_head; sender; sender = sender->next) {
		pocsag = (pocsag_t *)sender;

		/* Periodic queue stats (every 5 seconds) */
		if (now - pocsag->last_queue_stats_time >= 5.0) {
			int p, s, f;
			int total = 0, per_pol[POCSAG_NUM_POL] = {0};
			for (p = 0; p < POCSAG_NUM_POL; p++) {
				for (s = 0; s < POCSAG_NUM_SPD; s++) {
					for (f = 0; f < 8; f++) {
						for (msg = pocsag->msg_list[p][s][f]; msg; msg = msg->next) {
							per_pol[p]++;
							total++;
						}
					}
				}
			}
			if (total > 0) {
				LOGP_CHAN(DPOCSAG, LOGL_INFO, "Queue stats: %d msgs (normal=%d, inverted=%d) state=%s batch=%d\n",
					  total, per_pol[POCSAG_POL_NORMAL], per_pol[POCSAG_POL_INVERTED],
					  pocsag_state_name[pocsag->state], pocsag->batch_count);
			}
			pocsag->last_queue_stats_time = now;
		}

		if (!pocsag->tx || pocsag->state != POCSAG_IDLE)
			continue;
		{
			int p, s, f;
			int kicked = 0;
			for (p = 0; p < POCSAG_NUM_POL && !kicked; p++)
				for (s = 0; s < POCSAG_NUM_SPD && !kicked; s++)
					for (f = 0; f < 8 && !kicked; f++) {
						for (msg = pocsag->msg_list[p][s][f]; msg; msg = msg->next) {
							if (msg->next_send_time > 0.0 && now >= msg->next_send_time) {
								LOGP_CHAN(DPOCSAG, LOGL_INFO, "Retransmission for RIC %d now eligible, starting TX.\n", msg->ric);
								pocsag->tx_speed = msg->speed;
								pocsag->tx_polarity = msg->polarity;
								pocsag->batch_count = 0;
								pocsag_new_state(pocsag, POCSAG_PREAMBLE);
								pocsag->word_count = 0;
								kicked = 1;
								break;
							}
						}
					}
		}
	}
}

/*
 * Call control starts call towards paging network.
 *
 * Note: Function (sub-address) and message type are handled independently
 * per the POCSAG standard.
 */
int call_down_setup(int callref, const char *caller_id, enum number_type __attribute__((unused)) caller_type, const char *dialing)
{
	char channel = '\0';
	sender_t *sender;
	pocsag_t *pocsag;
	uint32_t ric;
	enum pocsag_function function;
	enum pocsag_msg_type msg_type;
	const char *message;
	int i;
	pocsag_msg_t *msg;

	/* find transmitter */
	for (sender = sender_head; sender; sender = sender->next) {
		/* skip channels that are different than requested */
		if (channel && sender->kanal[0] != channel)
			continue;
		pocsag = (pocsag_t *) sender;
		/* check if base station cannot transmit */
		if (!pocsag->tx)
			continue;
		break;
	}
	if (!sender) {
		if (channel)
			LOGP(DPOCSAG, LOGL_NOTICE, "Cannot page, because given station not available, rejecting!\n");
		else
			LOGP(DPOCSAG, LOGL_NOTICE, "Cannot page, no trasmitting station available, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}

	/*
	 * Get RIC and function (sub-address).
	 * Per POCSAG standard, function bits are sub-addresses with no
	 * mandatory correlation to message type.
	 */
	for (ric = 0, i = 0; i < 7; i++)
		ric = ric * 10 + dialing[i] - '0';
	if (dialing[7] >= '0' && dialing[7] <= '3')
		function = dialing[7]- '0';
	else if (dialing[7] >= 'a' && dialing[7] <= 'd')
		function = dialing[7]- 'a';
	else if (dialing[7] >= 'A' && dialing[7] <= 'D')
		function = dialing[7]- 'A';
	else
		function = pocsag->default_function;

	/* get message */
	if (caller_id[0])
		message = caller_id;
	else
		message = pocsag->default_message;

	/*
	 * Use default message type from config.
	 * Message type is independent of function per POCSAG standard.
	 */
	msg_type = pocsag->default_msg_type;

	/* create call process to page station */
	msg = pocsag_msg_create(pocsag, callref, ric, function, msg_type, message, strlen(message));
	if (!msg)
		return -CAUSE_INVALNUMBER;
	return -CAUSE_NORMAL;

	return 0;
}

/* message was transmitted */
void pocsag_msg_done(pocsag_t *pocsag)
{
	/* start scanning, if enabled, otherwise send loopback sequence, if enabled */
	pocsag_scan_or_loopback(pocsag);
}

void call_down_answer(int __attribute__((unused)) callref, struct timeval __attribute__((unused)) *tv_meter)
{
}

void call_down_proceeding(int __attribute__((unused)) callref)
{
}


static void _release(int __attribute__((unused)) callref, int __attribute__((unused)) cause)
{
	LOGP(DPOCSAG, LOGL_INFO, "Call has been disconnected by network.\n");
}

void call_down_disconnect(int callref, int cause)
{
	_release(callref, cause);

	call_up_release(callref, cause);
}

/* Call control releases call toward mobile station. */
void call_down_release(int callref, int cause)
{
	_release(callref, cause);
}

/* Receive audio from call instance. */
void call_down_audio(void __attribute__((unused)) *decoder, void __attribute__((unused)) *decoder_priv, int __attribute__((unused)) callref, uint16_t __attribute__((unused)) sequence, uint8_t __attribute__((unused)) marker, uint32_t __attribute__((unused)) timestamp, uint32_t __attribute__((unused)) ssrc, uint8_t __attribute__((unused)) *payload, int __attribute__((unused)) payload_len)
{
}

void dump_info(void) {}

