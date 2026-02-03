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

static const char pocsag_lang[9][4] = {
	{ '@', 0xc2, 0xa7, '\0' },
	{ '[', 0xc3, 0x84, '\0' },
	{ '\\', 0xc3, 0x96, '\0' },
	{ ']', 0xc3, 0x9c, '\0' },
	{ '{', 0xc3, 0xa4, '\0' },
	{ '|', 0xc3, 0xb6, '\0' },
	{ '}', 0xc3, 0xbc, '\0' },
	{ '~', 0xc3, 0x9f, '\0' },
	{ '\0' },
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
		for (msg = pocsag->msg_list; msg; msg = msg->next)
			display_status_subscriber(print_ric(msg), NULL);
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

	/* link */
	msg->pocsag = pocsag;
	msgp = &pocsag->msg_list;
	while ((*msgp))
		msgp = &(*msgp)->next;
	(*msgp) = msg;

	/* kick transmitter */
	if (pocsag->state == POCSAG_IDLE) {
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

	/* unlink */
	msgp = &msg->pocsag->msg_list;
	while ((*msgp) != msg)
		msgp = &(*msgp)->next;
	(*msgp) = msg->next;

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

		/*
		 * Generate scan message based on configured message type.
		 * Note: function bits are independent of message type per POCSAG standard.
		 */
		switch (pocsag->default_msg_type) {
		case POCSAG_MSG_TYPE_NUMERIC:
			sprintf(message, "%05d", pocsag->scan_from / 100);
			break;
		case POCSAG_MSG_TYPE_ALPHA:
			sprintf(message, "%02x", pocsag->scan_from / 10000);
			break;
		case POCSAG_MSG_TYPE_TONE:
		case POCSAG_MSG_TYPE_AUTO:
		default:
			message[0] = '\0';
		}
		LOGP_CHAN(DPOCSAG, LOGL_NOTICE, "Transmitting %s message '%s' with RIC '%d' / function '%s'.\n",
			  pocsag_msg_type_name(pocsag->default_msg_type), message, pocsag->scan_from,
			  pocsag_function_name[pocsag->default_function]);
		pocsag_msg_create(pocsag, 0, pocsag->scan_from, pocsag->default_function,
				  pocsag->default_msg_type, message, strlen(message));
		pocsag->scan_from++;
		return 1;
	}

	if (pocsag->sender.loopback) {
		LOGP(DPOCSAG, LOGL_INFO, "Sending message for loopback test.\n");
		pocsag_msg_create(pocsag, 0, 1234567, POCSAG_FUNCTION_A, POCSAG_MSG_TYPE_NUMERIC, "1234", 4);
		return 1;
	}

	return 0;
}

/*
 * Handle received message.
 *
 * Note: function (sub-address) and msg_type (encoding) are reported separately
 * as they are independent per the POCSAG standard.
 */
void pocsag_msg_receive(enum pocsag_language language, const char *channel, uint32_t ric, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message)
{
	char text[256 + strlen(message) * 4], *p;
	struct timeval tv;
	struct tm *tm;
	int i, j;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);

	/*
	 * Output format: timestamp @channel RIC,function,type[,message]
	 * Function is the sub-address (A-D), type is encoding (tone/numeric/alpha).
	 */
	sprintf(text, "%04d-%02d-%02d %02d:%02d:%02d.%03d @%s %d,%s,%s",
		tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec, (int)(tv.tv_usec / 10000.0),
		channel, ric, pocsag_function_name[function], pocsag_msg_type_name(msg_type));
	p = strchr(text, '\0');

	if (message[0]) {
		*p++ = ',';

		if (language == LANGUAGE_DEFAULT) {
			strcpy(p, message);
			p += strlen(p);
		} else {
			for (i = 0; message[i]; i++) {
				/* decode special chracter */
				for (j = 0; pocsag_lang[j][0]; j++) {
					if (pocsag_lang[j][0] == message[i])
						break;
				}
				/* if character matches */
				if (pocsag_lang[j][0]) {
					strcpy(p, pocsag_lang[j] + 1);
					p += strlen(p);
				} else
					*p++ = message[i];
			}
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
int pocsag_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int tx, int rx, enum pocsag_language language, int baudrate, double deviation, double polarity, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message, char padding, uint32_t scan_from, uint32_t scan_to, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback)
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
	rc = dsp_init_sender(pocsag, samplerate, (double)baudrate, deviation, polarity);
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

	pocsag_display_status();

	LOGP(DPOCSAG, LOGL_NOTICE, "Created 'Kanal' %s\n", kanal);

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

	while (pocsag->msg_list)
		pocsag_msg_destroy(pocsag->msg_list);
	dsp_cleanup_sender(pocsag);
	sender_destroy(&pocsag->sender);
	free(pocsag);
}

/*
 * Application sends us a message, we need to deliver.
 *
 * Format: RIC,function,type[,message]
 *
 * Per the POCSAG standard (CCIR Rec. 584):
 *   - RIC: 18-bit address (0-2097151, excluding 1003832 which is idle codeword)
 *   - Function: 2-bit sub-address (0-3 or A-D), provides 4 addresses per RIC
 *   - Type: Message encoding (auto/tone/numeric/alpha) - INDEPENDENT of function
 *   - Message: Optional message content
 *
 * The function bits and message type have NO mandatory correlation per the
 * POCSAG standard. Any correlation is manufacturer/pager specific.
 */
void pocsag_msg_send(enum pocsag_language language, const char *text, size_t text_length)
{
	char ric_string[text_length + 1];
	char function_string[text_length + 1];
	char type_string[text_length + 1];
	char message[text_length];
	uint32_t ric;
	uint8_t function;
	enum pocsag_msg_type msg_type;
	pocsag_t *pocsag;
	int message_length = 0;
	int i, ii, j, k;
	int rc;

	/* Parse RIC */
	for (i = 0; text_length; i++) {
		if (*text == ',')
			break;
		ric_string[i] = *text++;
		text_length--;
	}
	ric_string[i] = '\0';
	if (!text_length) {
inval:
		LOGP(DNMT, LOGL_NOTICE, "Given message MUST be in the following format: RIC,function,type[,<message>]\n");
		LOGP(DNMT, LOGL_NOTICE, "  RIC: 0-2097151 (7 digits)\n");
		LOGP(DNMT, LOGL_NOTICE, "  function: A/B/C/D or 0/1/2/3 (sub-address, per POCSAG standard)\n");
		LOGP(DNMT, LOGL_NOTICE, "  type: auto/tone/numeric/alpha (encoding, INDEPENDENT of function)\n");
		LOGP(DNMT, LOGL_NOTICE, "Note: Per POCSAG standard, function bits are sub-addresses with NO\n");
		LOGP(DNMT, LOGL_NOTICE, "      mandatory correlation to message type.\n");
		return;
	}
	text++;
	text_length--;

	/* Parse function (sub-address) */
	for (i = 0; text_length; i++) {
		if (*text == ',')
			break;
		function_string[i] = *text++;
		text_length--;
	}
	function_string[i] = '\0';
	if (!text_length) {
		goto inval;
	}
	text++;
	text_length--;

	/* Parse type (encoding) */
	for (i = 0; text_length; i++) {
		if (*text == ',')
			break;
		type_string[i] = *text++;
		text_length--;
	}
	type_string[i] = '\0';

	/* Parse optional message */
	if (text_length) {
		text++;
		text_length--;
		message_length = scan_message(text, text_length, message, sizeof(message));
	}

	/* Validate RIC */
	ric = atoi(ric_string);
	if (ric > 2097151) {
		LOGP(DNMT, LOGL_NOTICE, "Illegal RIC %d. Maximum allowed RIC is (2^21)-1. (2097151)\n", ric);
		goto inval;
	}

	if (ric == 1003832) {
		LOGP(DNMT, LOGL_NOTICE, "Illegal RIC 1003832. (Used as idle codeword)\n");
		goto inval;
	}

	/* Validate function (sub-address) */
	rc = pocsag_function_name2value(function_string);
	if (rc < 0) {
		LOGP(DNMT, LOGL_NOTICE, "Illegal function '%s'. Use A/B/C/D or 0/1/2/3.\n", function_string);
		goto inval;
	}
	function = rc;

	/* Validate message type */
	rc = pocsag_msg_type_name2value(type_string);
	if (rc < 0) {
		LOGP(DNMT, LOGL_NOTICE, "Illegal type '%s'. Use auto/tone/numeric/alpha.\n", type_string);
		goto inval;
	}
	msg_type = rc;

	/*
	 * Auto-detect message type if set to AUTO:
	 * - No message content -> TONE
	 * - All numeric chars -> NUMERIC
	 * - Otherwise -> ALPHA
	 *
	 * Note: This is independent of function bits per POCSAG standard.
	 */
	if (msg_type == POCSAG_MSG_TYPE_AUTO) {
		if (message_length == 0) {
			msg_type = POCSAG_MSG_TYPE_TONE;
		} else {
			/* Check if all characters are numeric-compatible */
			int all_numeric = 1;
			for (i = 0; i < message_length; i++) {
				char c = message[i];
				/* POCSAG numeric: 0-9, R, U, space, -, [, ] */
				if (!((c >= '0' && c <= '9') || c == 'R' || c == 'U' ||
				      c == ' ' || c == '-' || c == '[' || c == ']')) {
					all_numeric = 0;
					break;
				}
			}
			msg_type = all_numeric ? POCSAG_MSG_TYPE_NUMERIC : POCSAG_MSG_TYPE_ALPHA;
		}
	}

	/*
	 * Validate numeric message characters.
	 *
	 * Per POCSAG spec, BCD numeric encoding supports only 16 symbols:
	 * 0-9, R (recv/return), U (urgent), space, hyphen, and brackets.
	 * Invalid characters are rejected with an error showing the position.
	 */
	if (msg_type == POCSAG_MSG_TYPE_NUMERIC && message_length > 0) {
		for (i = 0; i < message_length; i++) {
			char c = message[i];
			/* POCSAG numeric BCD: 0-9, R, U, space, -, [, ] */
			if (!((c >= '0' && c <= '9') || c == 'R' || c == 'U' ||
			      c == ' ' || c == '-' || c == '[' || c == ']')) {
				LOGP(DNMT, LOGL_ERROR, "Invalid character in numeric message: '%c'\n", c);
				LOGP(DNMT, LOGL_ERROR, "Valid numeric characters are: 0-9 R U space - [ ]\n");
				return;
			}
		}
	}

	/* Handle language-specific character encoding */
	if (message_length && language != LANGUAGE_DEFAULT) {
		i = 0;
		/* input counter is ii, output counter is i */
		for (ii = 0; ii < message_length; i++) {
			/* encode special chracter */
			for (j = 0; pocsag_lang[j][0]; j++) {
				for (k = 0; pocsag_lang[j][k + 1]; k++) {
					/* break if input buffer ends */
					if (ii + k == message_length)
						break;
					/* break if string does not match */
					if (message[ii + k] != pocsag_lang[j][k + 1])
						break;
				}
				/* break, if k-loop was completed */
				if (!pocsag_lang[j][k + 1])
					break;
			}
			/* if character matches (k-loop was completed, j-loop not) */
			if (pocsag_lang[j][0]) {
				message[i] = pocsag_lang[j][0];
				ii += k;
			} else
				message[i] = message[ii++];
		}
		message_length = i;
	}

	LOGP(DNMT, LOGL_INFO, "Message for RIC '%d' / function '%s' / type '%s' with text '%s'\n",
	     ric, pocsag_function_name[function], pocsag_msg_type_name(msg_type),
	     print_message(message, message_length));

	pocsag = (pocsag_t *) sender_head;
	pocsag_msg_create(pocsag, 0, ric, function, msg_type, message, message_length);
}

void call_down_clock(void)
{
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

