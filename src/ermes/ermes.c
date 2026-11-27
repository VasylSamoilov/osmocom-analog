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
 * Each channel (-k) sends continuously: every batch of every subsequence, idle ones with only
 * system information, APTs and message delimiters, so that a pager finds the channel, locks to
 * it and keeps its batch timing. One channel is a single channel network (FSI 30); several form
 * one network, each carrying its frequency subset (--fsi, §9.2).
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
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include "../libmobile/main_mobile.h"
#include "ermes.h"
#include "book.h"

#define MAX_DISPLAY	1.4	/* something above speech level, no emphasis */
#define GROUP_PREFIX	"tempgroup:"	/* a group call, as flex */

const char *ermes_default_text = "";

/* Dibit to frequency deviation, as a fraction of the outer deviation: 00 -4687.5, 01 -1562.5,
 * 11 +1562.5, 10 +4687.5 Hz (Table 17 p.26) */
static const double dibit_deviation[4] = { -1.0, -1.0 / 3.0, 1.0, 1.0 / 3.0 };

/* Premodulation pulse shaping (§8.3.3 Fig.16): the symbols as rectangular levels through the
 * 10th order Bessel lowpass with 3 dB at 3.9 kHz that the standard gives as the reference shape.
 * The filter runs at PREMOD_OVERSAMPLE times the sample rate, where the bilinear transform keeps
 * the analog shape inside the masks up to 19 kHz (test_ermes_code checks every mask corner); every
 * PREMOD_OVERSAMPLE-th sample is taken (nothing is left above half the sample rate to fold). */
#define PREMOD_OVERSAMPLE	4
#define PREMOD_ORDER		10
#define PREMOD_CUTOFF_HZ	3900.0

/* the delay of the filter (its group delay at low frequency, nearly constant in the passband), so
 * the batches still leave the antenna on the UTC grid */
static double premod_group_delay(const iir_filter_t *f, int rate)
{
	const double hz = 10.0, w = 2.0 * M_PI * hz / rate;
	double ph = 0.0;
	int k;

	for (k = 0; k < PREMOD_ORDER / 2; k++) {
		ph += atan2(-f[k].a1 * sin(w) - f[k].a2 * sin(2 * w), f[k].a0 + f[k].a1 * cos(w) + f[k].a2 * cos(2 * w))
		    - atan2(-f[k].b1 * sin(w) - f[k].b2 * sin(2 * w), 1.0 + f[k].b1 * cos(w) + f[k].b2 * cos(2 * w));
	}
	return -ph / (2.0 * M_PI * hz);
}

/* The symbols of a batch into samples, from bit 'from' on: returns the number of samples */
static int modulate(ermes_t *ermes, int from, int nbits)
{
	sample_t *sub = ermes->premod_buffer;
	int i, n, nsub = 0, count = 0;

	for (i = from; i < nbits; i += 2) {
		sample_t level = dibit_deviation[(ermes->batch_bits[i] << 1) | ermes->batch_bits[i + 1]] * ermes->polarity;

		/* sub-samples of this symbol: its length at the oversampled rate, the fraction carried on */
		ermes->premod_phase += ermes->premod_symbol_samples;
		n = (int)ermes->premod_phase;
		ermes->premod_phase -= n;
		while (n--)
			sub[nsub++] = level;
	}
	iir_butterworth_process(ermes->premod, PREMOD_ORDER, sub, nsub);
	for (i = 0; i < nsub; i++) {
		if (ermes->premod_decimate++ % PREMOD_OVERSAMPLE == 0)
			ermes->tx_buffer[count++] = sub[i];
	}
	return count;
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	ermes_t *ermes = (ermes_t *) sender;
	int nbits, count;

	while (length) {
		if (ermes->tx_buffer_pos == ermes->tx_buffer_length) {
			nbits = ermes_scheduler_next_batch(ermes);
			/* the first batch starts where UTC is in it (the scheduler's UTC lock) */
			ermes->tx_buffer_length = modulate(ermes, ermes->skip_bits, nbits);
			ermes->skip_bits = 0;
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

/* Receive: FM demodulated samples of the channel (1.0 = the outer deviation) */
void sender_receive(sender_t *sender, sample_t *samples, int length, double __attribute__((unused)) rf_level_db)
{
	ermes_t *ermes = (ermes_t *)sender;

	ermes_dsp_rx_samples(&ermes->rx_dsp, samples, length);
}

#define CHAN ermes->sender.kanal

static void rx_batch(void *priv, const ermes_rx_batch_t *b)
{
	ermes_t *ermes = priv;

	LOGP_CHAN(DERMES, LOGL_INFO, "Received batch %c minute %d subsequence %d: %d-%d PA %d FSI %d ETI %d BAI %d SSIT %d, %d initial address(es), %d codeblocks%s%s%s%s [%d bits corrected, %d codewords in error, %d runs without a header, %d messages without an initial address in 12 s (not shown), sync %d bits off]\n",
		  'A' + b->batch, b->cycle, b->ssn, b->country, b->operator, b->pa, b->fsi, b->eti, b->bai, b->ssit, b->ia_count, b->codeblocks,
		  b->si_ok ? "" : ", system information in error",
		  b->flywheel ? ", no sync (taken on timing)" : "",
		  b->apt != b->apt_expected ? ", APT count not as the formula" : "",
		  b->not_ascending ? ", messages not in ascending order" : "",
		  b->corrected, b->bad, b->unreadable, b->no_ia, b->sync_errors);
}

static void rx_message(void *priv, const ermes_rx_batch_t *b, const ermes_rx_msg_t *m)
{
	ermes_t *ermes = priv;

	LOGP_CHAN(DERMES, LOGL_NOTICE, "Received in batch %c: %s\n", 'A' + b->batch, ermes_rx_msg_str(m));
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
	ermes->group_ctap = -1;	/* no group call; all CTAPs free (calloc) */
	ermes->net = *net;

	/* a sample value of 1.0 is the outer deviation */
	sender_set_fm(&ermes->sender, ERMES_DEVIATION_HZ, ERMES_SYMBOL_RATE, ERMES_DEVIATION_HZ, MAX_DISPLAY);
	ermes->polarity = polarity;
	ermes_rx_init(&ermes->rx, channel, ermes_charset, ermes, rx_batch, rx_message);
	ermes_dsp_rx_init(&ermes->rx_dsp, &ermes->rx, samplerate, polarity);
	iir_bessel_lowpass_init(ermes->premod, PREMOD_CUTOFF_HZ, samplerate * PREMOD_OVERSAMPLE, PREMOD_ORDER);
	ermes->premod_symbol_samples = (double)samplerate * PREMOD_OVERSAMPLE / ERMES_SYMBOL_RATE;
	ermes->premod_delay = premod_group_delay(ermes->premod, samplerate * PREMOD_OVERSAMPLE);
	/* a batch at most, and one sample more per symbol for the fractions */
	ermes->tx_buffer_size = (int)((double)samplerate / ERMES_SYMBOL_RATE * ERMES_BATCH_CW_LAST * ERMES_CW_BITS / 2) + ERMES_BATCH_CW_LAST * ERMES_CW_BITS / 2;
	ermes->tx_buffer = calloc(ermes->tx_buffer_size, sizeof(sample_t));
	ermes->premod_buffer = calloc(ermes->tx_buffer_size * PREMOD_OVERSAMPLE, sizeof(sample_t));
	if (!ermes->tx_buffer || !ermes->premod_buffer) {
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
	free(ermes->premod_buffer);
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
	char buf[ERMES_MAX_TEXT + 512], *field[3] = { NULL, NULL, NULL }, *p, *tok, *save = NULL;
	ermes_page_opts_t opts = ermes_default_opts;
	const char *text = ermes_default_text;
	uint32_t la, members[ERMES_GROUP_MAX_MEMBERS];
	int i, num = 0, group, book;

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
	/* a group call, as flex: "tempgroup:<address> <address> ...,type,options,message" */
	group = !strncasecmp(buf, GROUP_PREFIX, strlen(GROUP_PREFIX));
	/* type "book": change the receiver's book entry, "number=5 fsn=8" in the options field */
	book = (field[0] && !strcasecmp(field[0], "book"));
	/* strtok_r: the checks below may read the book, which tokenizes too */
	for (tok = strtok_r(buf + (group ? strlen(GROUP_PREFIX) : 0), " ", &save); tok; tok = strtok_r(NULL, " ", &save)) {
		if (num == ERMES_GROUP_MAX_MEMBERS || (!group && num == 1)) {
			LOGP(DERMES, LOGL_ERROR, "Too many addresses in \"%s\" (a group call: tempgroup: and at most %d).\n", line, ERMES_GROUP_MAX_MEMBERS);
			return -1;
		}
		if (ermes_parse_address(tok, &la) < 0) {
			LOGP(DERMES, LOGL_ERROR, "Bad address \"%s\" in \"%s\": the local address (decimal or 0x hex) or nec-<label code>.\n", tok, line);
			return -1;
		}
		members[num++] = la;
	}
	if (!num) {
		LOGP(DERMES, LOGL_ERROR, "No address in \"%s\".\n", line);
		return -1;
	}
	if (book) {
		if (group) {
			LOGP(DERMES, LOGL_ERROR, "book takes one receiver.\n");
			return -1;
		}
		return ermes_book_set(members[0], field[1] ? field[1] : "");
	}
	if (field[0] && !strcasecmp(field[0], "lastnum")) {
		/* the latest message number again (-2 §4.3.1.3.3): a header only */
		opts.last_number = 1;
		opts.category = ERMES_CAT_TONE;
	} else if (field[0] && *field[0] && (ermes_category_from_name(field[0], &opts.category) < 0 || opts.category == ERMES_CAT_TRANSPARENT)) {
		LOGP(DERMES, LOGL_ERROR, "Bad type \"%s\": tone, numeric, alpha or lastnum.\n", field[0]);
		return -1;
	}
	if (field[1] && ermes_parse_page_opts(field[1], &opts) < 0)
		return -1;
	if (opts.last_number && (group || opts.retransmit)) {
		LOGP(DERMES, LOGL_ERROR, "lastnum goes to one receiver, without retransmit.\n");
		return -1;
	}
	if (group && opts.external) {
		LOGP(DERMES, LOGL_ERROR, "A group call goes to our own receivers: no opid.\n");
		return -1;
	}
	/* a page that no channel may carry would wait for ever (a book change may fix it) */
	for (i = 0; i < num; i++) {
		if (!ermes_scheduler_reachable(members[i], opts.external ? opts.opid : -1)) {
			LOGP(DERMES, LOGL_ERROR, "No channel carries %s: its FSN (book) is not in any channel's FSI.\n",
			     ermes_print_address(members[i]));
			return -1;
		}
	}
	if (field[2])
		text = field[2];
	if (group)
		return ermes_queue_group(&ermes_queue, members, num, &opts, text);
	return ermes_queue_page(&ermes_queue, members[0], &opts, text);
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
