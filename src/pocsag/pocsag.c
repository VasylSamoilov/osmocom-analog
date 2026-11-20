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
#include <osmocom/core/timer.h>
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
static pocsag_msg_t *pocsag_msg_create(pocsag_t *pocsag, uint32_t callref, uint32_t ric, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message, size_t message_length, int speed, double polarity)
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
	msg->speed = speed;
	msg->polarity = polarity;

	/* link to per-frame, per-polarity queue.
	 * The bucket is derived from msg->speed/msg->polarity, so these MUST
	 * be set before linking and MUST NOT be changed afterwards without
	 * re-linking: pocsag_msg_destroy() recomputes the bucket from these
	 * fields, and a mismatch would walk the wrong (possibly empty) list
	 * off its end. */
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
		while ((*msgp) && (*msgp) != msg)
			msgp = &(*msgp)->next;
		if (*msgp == msg)
			(*msgp) = msg->next;
		else
			LOGP(DPOCSAG, LOGL_ERROR, "pocsag_msg_destroy: RIC %d not found in queue [pol=%d spd=%d frame=%d] (bug: speed/polarity changed after linking)\n",
			     msg->ric, pol, spd, frame);
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
					  pocsag->default_msg_type, message, strlen(message),
					  pocsag->default_speed, pocsag->default_polarity);
			queued++;
		}

		/* only advance past RICs we actually enqueued */
		pocsag->scan_from = pocsag->scan_from + queued;

		return queued > 0;
	}

	if (pocsag->sender.loopback) {
		LOGP(DPOCSAG, LOGL_INFO, "Sending message for loopback test.\n");
		pocsag_msg_create(pocsag, 0, 1234567, POCSAG_FUNCTION_A, POCSAG_MSG_TYPE_NUMERIC, "1234", 4,
				  pocsag->default_speed, pocsag->default_polarity);
		return 1;
	}

	return 0;
}

/*
 * Handle received message.  The text is final: the alphanumeric message is
 * decoded with the codepage by the receiver.
 *
 * Note: function (sub-address) and msg_type (encoding) are reported separately
 * as they are independent per the POCSAG standard.
 */
void pocsag_msg_receive(const char *channel, uint32_t ric, enum pocsag_function function, enum pocsag_msg_type msg_type, int baudrate, double polarity, const char *message)
{
	int msg_len = message ? strlen(message) : 0;
	char text[256 + msg_len], *p;
	struct timeval tv;
	struct tm *tm;

	osmo_gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);

	sprintf(text, "%04d-%02d-%02d %02d:%02d:%02d.%03d @%s %d%s,%s,%d,%s",
		tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec, (int)(tv.tv_usec / 10000.0),
		channel, ric, pocsag_function_name[function], pocsag_msg_type_name(msg_type),
		baudrate, (polarity < 0) ? "normal" : "inverted");
	p = strchr(text, '\0');

	if (message && message[0]) {
		*p++ = ',';
		strcpy(p, message);
		p += strlen(p);
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
int pocsag_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int tx, int rx, const struct pager_charset *charset, int baudrate, double deviation, double polarity, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message, char padding, uint32_t scan_from, uint32_t scan_to, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback, int auto_baud, int auto_polarity, double dedup_window, int max_batches, int network_mode)
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
	pocsag->charset = charset;
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
 * The UTF-8 text of an alphanumeric message as the codes of the codepage, in
 * place (a text is never shorter in UTF-8 than its codes).  Returns -1 and
 * logs the character if one has no code.
 */
static int pocsag_encode_text(const struct pager_charset *charset, char *message, int *message_length)
{
	uint8_t codes[*message_length + 1];
	const char *bad = NULL;
	int n;

	n = pager_charset_encode_text(charset, message, *message_length, codes, sizeof(codes), &bad);
	if (n < 0) {
		LOGP(DNMT, LOGL_NOTICE, "Message refused: character at '%.8s' has no code in codepage %s-%s-%s.\n",
		     bad ? bad : "", charset->vendor, charset->model, charset->charset);
		return -1;
	}
	memcpy(message, codes, n);
	*message_length = n;
	return 0;
}

/*
 * Parse space-separated key=value options from FIFO message.
 * charset=<vendor-model-charset> sets the codepage of the message.
 * Returns -1 if the codepage is not one of POCSAG.
 */
static int parse_pocsag_options(const char *opts, int opts_len,
				 const struct pager_charset **charset_out,
				 int *speed_out, double *polarity_out,
				 int *repeat_out, double *delay_out,
				 double *interval_out)
{
	char buf[256];
	char *p, *key, *val;
	int len;

	if (opts_len <= 0)
		return 0;

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

		if (!strcmp(key, "charset")) {
			const struct pager_charset *cs = pager_charset_lookup("pocsag", val);
			if (!cs) {
				LOGP(DNMT, LOGL_NOTICE, "FIFO: '%s' is not a POCSAG codepage, see '--charset list'.\n", val);
				return -1;
			}
			*charset_out = cs;
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

	return 0;
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
 *       charset=<vendor-model-charset>  codepage of the message (overrides --charset)
 *       speed=512|1200|2400
 *       polarity=normal|inverted
 *   - message: message content (UTF-8, may contain escape sequences)
 *
 * Examples:
 *   1234567A,alpha,,Hello World
 *   1234567,numeric,,12345
 *   1234567B,alpha,charset=generic-generic-latin-german,<UTF-8 text>
 *   1234567C,alpha,charset=nec-skyper-ascii-rot1,Hello
 *   1234567,tone,,
 */
void pocsag_msg_send(const struct pager_charset *charset, const char *text, size_t text_length)
{
	char capcode_string[text_length + 1];
	char type_string[text_length + 1];
	char message[text_length + 1];
	uint32_t ric;
	enum pocsag_function function;
	enum pocsag_msg_type msg_type;
	pocsag_t *pocsag;
	int message_length = 0;
	int i;
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
		LOGP(DNMT, LOGL_NOTICE, "    charset=<vendor-model-charset> (see '--charset list')\n");
		LOGP(DNMT, LOGL_NOTICE, "  message: text content\n");
		LOGP(DNMT, LOGL_NOTICE, "  Example: 1234567A,alpha,,Hello World\n");
		LOGP(DNMT, LOGL_NOTICE, "  Example: 1234567B,alpha,charset=generic-generic-latin-german,<UTF-8 text>\n");
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
	if (parse_pocsag_options(opts_start, opts_len, &charset, &msg_speed, &msg_polarity, &repeat, &delay, &interval) < 0)
		return;

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

	/* Alphanumeric: the UTF-8 text as the codes of the codepage */
	if (message_length && msg_type == POCSAG_MSG_TYPE_ALPHA) {
		if (pocsag_encode_text(charset, message, &message_length) < 0)
			return;
	}

	LOGP(DNMT, LOGL_INFO, "FIFO: enqueued RIC=%d type=%s function=%s charset=%s-%s-%s len=%d repeat=%d delay=%.1f interval=%.1f\n",
	     ric, pocsag_msg_type_name(msg_type), pocsag_function_name[function],
	     charset->vendor, charset->model, charset->charset, message_length, repeat, delay, interval);

	{
		pocsag_msg_t *msg;
		struct timeval tv;
		int eff_speed = msg_speed ? msg_speed : pocsag->default_speed;
		double eff_polarity = (msg_polarity != 0.0) ? msg_polarity : pocsag->default_polarity;

		/* If locked mode (max_batches=0), discard messages with wrong
		 * speed/polarity before creating/linking them. */
		if (pocsag->max_batches == 0 &&
		    (eff_speed != pocsag->default_speed || eff_polarity != pocsag->default_polarity)) {
			LOGP(DNMT, LOGL_NOTICE, "FIFO: discarding RIC %d — speed %d/%s does not match locked %d/%s (--max-batches 0).\n",
			     ric, eff_speed, (eff_polarity < 0) ? "normal" : "inverted",
			     pocsag->default_speed, (pocsag->default_polarity < 0) ? "normal" : "inverted");
			return;
		}

		msg = pocsag_msg_create(pocsag, 0, ric, function, msg_type, message, message_length,
					eff_speed, eff_polarity);
		if (msg) {
			msg->retransmit_max = repeat;
			msg->retransmit_count = 0;
			msg->retransmit_interval = interval;
			msg->send_delay = delay;

			osmo_gettimeofday(&tv, NULL);
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

	osmo_gettimeofday(&tv, NULL);
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

	/* Alphanumeric: the UTF-8 text as the codes of the codepage */
	if (msg_type == POCSAG_MSG_TYPE_ALPHA) {
		int len = strlen(message);
		char text[len + 1];

		memcpy(text, message, len + 1);
		if (pocsag_encode_text(pocsag->charset, text, &len) < 0)
			return -CAUSE_INVALNUMBER;
		msg = pocsag_msg_create(pocsag, callref, ric, function, msg_type, text, len,
					pocsag->default_speed, pocsag->default_polarity);
		if (!msg)
			return -CAUSE_INVALNUMBER;
		return -CAUSE_NORMAL;
	}

	/* create call process to page station */
	msg = pocsag_msg_create(pocsag, callref, ric, function, msg_type, message, strlen(message),
				pocsag->default_speed, pocsag->default_polarity);
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

void dump_scheduler_hotkey(void) {}
