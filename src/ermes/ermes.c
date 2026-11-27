/* ERMES (European Radio Message System) transmitter
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
 *
 * The transmitter is a single channel network (FSI 30) that sends continuously: every batch
 * of every subsequence, idle ones with only system information, APTs and message delimiters,
 * so that a pager finds the channel, locks to it and keeps its batch timing.
 *
 * It behaves as an I1 version 01.00 network (ETS 300 133-4 ed.2 §4.6, Table 2 p.14): no IVR1
 * (SSIT 0010) and only the SSI types of edition 1, 0000 (zone/hour/date) and 0001
 * (day/month/year), so that early pagers see nothing they do not know.
 *
 * The parts: the sources here (FIFO and --page lines, call control) put pages in the queue
 * (queue.c, numbered by numbers.c); the scheduler (scheduler.c) builds batch after batch from it
 * with the system information of ssi.c and the batch builder of frame.c; the modulator here turns
 * the batches into samples.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include "../libmobile/main_mobile.h"
#include "ermes.h"

#define MAX_DISPLAY	1.4	/* something above speech level, no emphasis */

const char *ermes_default_text = "";

/* Dibit to level of the modulator: 00 -4687.5, 01 -1562.5, 11 +1562.5, 10 +4687.5 Hz
 * (Table 17 p.26); the levels of fsk_nrz are -1, -1/3, +1/3, +1 of the deviation. */
static const uint8_t dibit_level[4] = { 0, 1, 3, 2 };

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	ermes_t *ermes = (ermes_t *) sender;
	int nbits, i, count;

	while (length) {
		if (ermes->tx_buffer_pos == ermes->tx_buffer_length) {
			nbits = ermes_scheduler_next_batch(ermes);
			count = 0;
			/* the first batch starts where UTC is in it (the scheduler's UTC lock) */
			i = ermes->skip_bits;
			ermes->skip_bits = 0;
			for (; i < nbits; i += 2) {
				int dibit = (ermes->batch_bits[i] << 1) | ermes->batch_bits[i + 1];

				count += fsk_nrz_mod_symbol(&ermes->fsk_mod, dibit_level[dibit], ermes->tx_buffer + count);
			}
			ermes->tx_buffer_length = count;
			ermes->tx_buffer_pos = 0;
		}
		count = ermes->tx_buffer_length - ermes->tx_buffer_pos;
		if (count > length)
			count = length;
		memcpy(samples, ermes->tx_buffer + ermes->tx_buffer_pos, count * sizeof(*samples));
		memset(power, 1, count);
		ermes->tx_buffer_pos += count;
		samples += count;
		power += count;
		length -= count;
	}
}

/* Receive audio from radio unit: no receiver yet */
void sender_receive(sender_t __attribute__((unused)) *sender, sample_t __attribute__((unused)) *samples, int __attribute__((unused)) length, double __attribute__((unused)) rf_level_db)
{
}

int ermes_create(const char *kanal, int channel, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, double polarity, const ermes_network_t *net, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback)
{
	ermes_t *ermes;
	double frequency = ermes_channel_frequency(channel);
	int rc;

	ermes = calloc(1, sizeof(*ermes));
	if (!ermes) {
		LOGP(DERMES, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	rc = sender_create(&ermes->sender, kanal, frequency, frequency, device, use_sdr, samplerate, rx_gain, tx_gain, 0, 0, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DERMES, LOGL_ERROR, "Failed to init transceiver process!\n");
		free(ermes);
		return rc;
	}
	ermes->channel = channel;
	ermes->net = *net;

	/* a sample value of 1.0 is the outer deviation */
	sender_set_fm(&ermes->sender, ERMES_DEVIATION_HZ, ERMES_SYMBOL_RATE, ERMES_DEVIATION_HZ, MAX_DISPLAY);
	fsk_nrz_mod_init(&ermes->fsk_mod, samplerate, (double)ERMES_SYMBOL_RATE, 1.0, polarity);
	ermes->tx_buffer_size = fsk_nrz_mod_max_samples(samplerate, ERMES_SYMBOL_RATE, ERMES_BATCH_CW_LAST * ERMES_CW_BITS / 2) + 16;
	ermes->tx_buffer = calloc(ermes->tx_buffer_size, sizeof(sample_t));
	if (!ermes->tx_buffer) {
		LOGP(DERMES, LOGL_ERROR, "No memory!\n");
		ermes_destroy(&ermes->sender);
		return -ENOMEM;
	}

	LOGP(DERMES, LOGL_NOTICE, "Channel %d (%.4f MHz): %s; alphanumeric codepage %s-%s-%s\n",
	     channel, frequency / 1e6, ermes_network_str(net),
	     ermes_charset->vendor, ermes_charset->model, ermes_charset->charset);
	return 0;
}

void ermes_destroy(sender_t *sender)
{
	ermes_t *ermes = (ermes_t *) sender;

	/* the queue is the network's: emptied when the last channel goes */
	if (!sender_head->next)
		ermes_queue_flush(&ermes_queue);
	free(ermes->tx_buffer);
	sender_destroy(&ermes->sender);
	free(ermes);
}

int ermes_idle(void)
{
	sender_t *s;

	if (ermes_queue.head)
		return 0;
	/* the rest of a long message still to send */
	for (s = sender_head; s; s = s->next) {
		if (((ermes_t *)s)->carry.count)
			return 0;
	}
	return 1;
}

/*
 * Sources
 */

int ermes_msg_send(const char *line)
{
	/* as flex and pocsag: address,type,options,message - the message is the rest of the line */
	char buf[ERMES_MAX_TEXT + 512], *field[3] = { NULL, NULL, NULL }, *p;
	ermes_page_opts_t opts = ermes_default_opts;
	const char *text = ermes_default_text;
	uint32_t la;
	int i;

	if (!sender_head) {
		LOGP(DERMES, LOGL_ERROR, "No transmitter.\n");
		return -1;
	}
	snprintf(buf, sizeof(buf), "%s", line);
	p = buf;
	for (i = 0; i < 3 && (p = strchr(p, ',')); i++) {
		*p++ = '\0';
		field[i] = p;
	}
	if (ermes_parse_address(buf, &la) < 0) {
		LOGP(DERMES, LOGL_ERROR, "Bad address \"%s\" in \"%s\": the local address (decimal or 0x hex) or nec:<label code>.\n", buf, line);
		return -1;
	}
	if (field[0] && *field[0] && (ermes_category_from_name(field[0], &opts.category) < 0 || opts.category == ERMES_CAT_TRANSPARENT)) {
		LOGP(DERMES, LOGL_ERROR, "Bad type \"%s\": tone, numeric or alpha.\n", field[0]);
		return -1;
	}
	if (field[1] && ermes_parse_page_opts(field[1], &opts) < 0)
		return -1;
	if (field[2])
		text = field[2];
	return ermes_queue_page(&ermes_queue, la, &opts, text);
}

const char *ermes_number_valid(const char *number)
{
	static char error[64];
	uint32_t la;

	if (ermes_parse_address(number, &la) < 0) {
		snprintf(error, sizeof(error), "The local address has %d bits (at most %u)", ERMES_LA_BITS, (1u << ERMES_LA_BITS) - 1);
		return error;
	}
	return NULL;
}

/* Call control (built-in console): page the dialled 7 digit local address */
int call_down_setup(int __attribute__((unused)) callref, const char *caller_id, enum number_type __attribute__((unused)) caller_type, const char *dialing)
{
	const char *text = (caller_id && caller_id[0]) ? caller_id : ermes_default_text;
	uint32_t la;

	if (!sender_head)
		return -CAUSE_NOCHANNEL;
	if (ermes_parse_address(dialing, &la) < 0)
		return -CAUSE_INVALNUMBER;
	if (ermes_queue_page(&ermes_queue, la, &ermes_default_opts, text) < 0)
		return -CAUSE_INVALNUMBER;
	return -CAUSE_NORMAL;
}

void call_down_clock(void) {}
void call_down_answer(int __attribute__((unused)) callref, struct timeval __attribute__((unused)) *tv_meter) {}
void call_down_proceeding(int __attribute__((unused)) callref) {}
void call_down_disconnect(int __attribute__((unused)) callref, int __attribute__((unused)) cause) {}
void call_down_release(int __attribute__((unused)) callref, int __attribute__((unused)) cause) {}
void call_down_audio(void __attribute__((unused)) *decoder, void __attribute__((unused)) *decoder_priv, int __attribute__((unused)) callref, uint16_t __attribute__((unused)) sequence, uint8_t __attribute__((unused)) marker, uint32_t __attribute__((unused)) timestamp, uint32_t __attribute__((unused)) ssrc, uint8_t __attribute__((unused)) *payload, int __attribute__((unused)) payload_len) {}
void dump_info(void) {}
void dump_scheduler_hotkey(void) {}
