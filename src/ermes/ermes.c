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
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include "../libmobile/main_mobile.h"
#include "ermes.h"

#define CHAN ermes->sender.kanal
#define MAX_DISPLAY	1.4	/* something above speech level, no emphasis */
#define MAX_PAGES	32	/* initial addresses per batch we fill */

ermes_page_opts_t ermes_default_opts = { .category = ERMES_CAT_TONE, .urgent = 0, .alert = 0 };
const char *ermes_default_text = "";
const struct pager_charset *ermes_charset = &pager_charset_ermes_generic_generic_standard;
/* SSI type sent: ERMES_SSI_ALTERNATE (0000 and 0001 by turns) or one SSIT in every batch */
int ermes_ssi_mode = ERMES_SSI_HOLD;
/* --ssi-hold: subsequences each type is held with --ssi hold (5 = one minute) */
int ermes_ssi_hold = 1;

/* Dibit to level of the modulator: 00 -4687.5, 01 -1562.5, 11 +1562.5, 10 +4687.5 Hz
 * (Table 17 p.26); the levels of fsk_nrz are -1, -1/3, +1/3, +1 of the deviation. */
static const uint8_t dibit_level[4] = { 0, 1, 3, 2 };

/* --clock: the network clock runs this many seconds from the system clock (0 = real time) */
double ermes_clock_offset = 0.0;

/* --year-from: the year sent in the SSI (type 0001) moved into [ermes_year_from, +27] by whole
 * 28-year steps, so that pagers whose firmware calendar ends early (NEC: the Skyper counter
 * overflows after 2026-11, the PA4PY-20A ends 2013) get a year they know. Weekdays and leap years
 * repeat every 28 years from 1901 to 2099, so weekday, month and day stay right. 0 = off. */
int ermes_year_from = 0;
#define CALENDAR_CYCLE	28

static int ermes_year(int year)
{
	if (!ermes_year_from)
		return year;
	while (year >= ermes_year_from + CALENDAR_CYCLE)
		year -= CALENDAR_CYCLE;
	while (year < ermes_year_from)
		year += CALENDAR_CYCLE;
	return year;
}

/* Network time (seconds since the epoch) at the start of the batch being built: the start of
 * the first batch plus the codewords sent since, so it follows the samples, not the wall clock */
static double ermes_network_time(const ermes_t *ermes)
{
	return ermes->clock_start + (double)ermes->cw_sent * ERMES_CW_BITS / ERMES_BIT_RATE;
}

/* --tx-latency: seconds from making a sample to its transmission (the DSP buffer ahead) */
double ermes_tx_latency = -1.0;	/* < 0: dsp_buffer */

/* Lock the batches to UTC (§4.1 p.10, §12): subsequences start every 12 s from the UTC minute,
 * the batches at fixed codeword offsets. Find where the first sample will be on the air in this
 * grid - the batch and the bit in it - so the transmission starts in the middle of that batch.
 * Every start then keeps the same grid, as a real network: a pager that synchronised before keeps
 * waking at the right time (with a grid shifted at every start it wakes too early or too late,
 * misses its batch and is lost until it searches again). */
static void ermes_start_time(ermes_t *ermes)
{
	struct timeval tv;
	struct tm tm;
	double now, into_ssn, bits;
	time_t minute;
	int slot, skip;

	gettimeofday(&tv, NULL);
	now = tv.tv_sec + tv.tv_usec / 1e6 + ermes_clock_offset
	    + ((ermes_tx_latency >= 0) ? ermes_tx_latency : dsp_buffer / 1000.0);
	minute = (time_t)now - ((time_t)now % 60);
	gmtime_r(&minute, &tm);
	ermes->cycle = tm.tm_min;
	into_ssn = now - minute;
	ermes->ssn = (int)(into_ssn / ERMES_SUBSEQUENCE_S);
	into_ssn -= ermes->ssn * ERMES_SUBSEQUENCE_S;
	bits = into_ssn * ERMES_BIT_RATE;
	for (slot = ERMES_BATCHES - 1; slot > 0; slot--) {
		if (bits >= (double)ermes_batch_offset(slot) * ERMES_CW_BITS)
			break;
	}
	ermes->slot = slot;
	/* whole symbols into the batch */
	skip = ((int)(bits - (double)ermes_batch_offset(slot) * ERMES_CW_BITS)) & ~1;
	if (skip >= ermes_batch_length(slot) * ERMES_CW_BITS)
		skip = ermes_batch_length(slot) * ERMES_CW_BITS - 2;
	ermes->skip_bits = skip;
	/* the network time of the first batch is its start in the grid */
	ermes->clock_start = (double)minute + ermes->ssn * ERMES_SUBSEQUENCE_S
			   + (double)ermes_batch_offset(ermes->slot) * ERMES_CW_BITS / ERMES_BIT_RATE;
	ermes->cw_sent = 0;
	ermes->started = 1;
	LOGP_CHAN(DERMES, LOGL_INFO, "Transmission starts locked to %s UTC %02d:%02d:%06.3f: minute (cycle) %d, subsequence %d, time slot %d, %d bits into the batch\n",
		  ermes_clock_offset ? "network time (--clock)" : "", tm.tm_hour, tm.tm_min, now - minute,
		  ermes->cycle, ermes->ssn, ermes->slot, skip);
}

static void ermes_next_slot(ermes_t *ermes)
{
	if (++ermes->slot < ERMES_BATCHES)
		return;
	ermes->slot = 0;
	if (++ermes->ssn < ERMES_SUBSEQUENCES)
		return;
	ermes->ssn = 0;
	ermes->cycle = (ermes->cycle + 1) % ERMES_CYCLES;
}

/* SSI of this batch, local time (§5.3.2.2-3 p.17), by --ssi:
 *   hold (default)  one type in every batch for --ssi-hold subsequences, then the other, so that
 *                   every pager sees the same word at each of its wakes
 *   alternate       0000 / 0001 from batch to batch, the subsequence count added (a subsequence
 *                   has an even number of batches), so an own batch gets both types by turns
 *   strict, pairs   0000 0001 ... and 0000 0000 0001 0001 ... from a batch counter
 *   cycle           the 13 allowed types, one step a batch (edition 2 §5.3.2.1 p.16)
 *   one type        in every batch
 * NEC PA3E6-1A on air (2026-11-26): it sets its clock from SSI only when it acquires the network
 * after power on, one type: 0000 alone gave the hour (weekday stays the default Mo), 0001 alone
 * the weekday (hour 0); mixed schedules gave one of them, the 13-type cycle neither; no update
 * later in battery saving (hold: 0001 a minute after acquisition did not change the weekday).
 * The minute follows the cycle number. */
static void ermes_ssi(ermes_t *ermes, ermes_ssi_t *ssi)
{
	time_t now = (time_t)ermes_network_time(ermes);
	struct tm tm;
	int ssit = ermes_ssi_mode;

	localtime_r(&now, &tm);
	if (ssit == ERMES_SSI_ALTERNATE)
		ssit = ((ermes->slot + ermes->cycle * ERMES_SUBSEQUENCES + ermes->ssn) & 1) ? ERMES_SSIT_DAY_MONTH_YEAR : ERMES_SSIT_ZONE_HOUR_DATE;
	if (ssit == ERMES_SSI_HOLD) {
		/* one type in every batch of ermes_ssi_hold subsequences, then the other. With 1 this is
		 * what the Swiss network sent (recording of 1997-07-03 13:38, country 228: 0000 in every
		 * batch of subsequences 0 and 2, 0001 in 1 and 3) */
		int count = ermes->cycle * ERMES_SUBSEQUENCES + ermes->ssn;

		ssit = ((count / ermes_ssi_hold) & 1) ? ERMES_SSIT_DAY_MONTH_YEAR : ERMES_SSIT_ZONE_HOUR_DATE;
	}
	if (ssit == ERMES_SSI_STRICT || ssit == ERMES_SSI_PAIRS) {
		/* from a running batch counter: strict 0000 0001 0000 ..., pairs 0000 0000 0001 0001 ... */
		int k = (ssit == ERMES_SSI_STRICT) ? ermes->ssit_next : ermes->ssit_next >> 1;

		ssit = (k & 1) ? ERMES_SSIT_DAY_MONTH_YEAR : ERMES_SSIT_ZONE_HOUR_DATE;
		ermes->ssit_next++;
	}
	if (ssit == ERMES_SSI_CYCLE) {
		/* ed.2 §5.3.2.1 p.16: every batch the next of the 13 allowed values, modulo 13 */
		ssit = ermes->ssit_next;
		ermes->ssit_next = (ermes->ssit_next + 1) % ERMES_SSIT_ALLOWED;
	}
	ssi->ssit = ssit;
	switch (ssit) {
	case ERMES_SSIT_ZONE_HOUR_DATE:
		ssi->ssif = ermes_ssif_zone_hour_date(ermes->net.zone, tm.tm_hour, tm.tm_mday);
		break;
	case ERMES_SSIT_DAY_MONTH_YEAR:
		/* struct tm: Sunday = 0, months from 0, years from 1900; ERMES: Monday = 1 .. Sunday = 7 */
		ssi->ssif = ermes_ssif_day_month_year(tm.tm_wday ? tm.tm_wday : 7, tm.tm_mon + 1, ermes_year(tm.tm_year + 1900));
		break;
	case ERMES_SSIT_IVR1:
		/* a network that cycles the SSIT is an I1 version >= 02.01 one (§4.6.1 p.14) */
		ssi->ssif = ermes_ssif_version(ERMES_IVR1_MAIN, ERMES_IVR1_SUB);
		break;
	default:
		/* reserved for future definition: no value is defined, all zero */
		ssi->ssif = 0;
	}
}

/* Message number per address: 1..31, 1 first, +1 per message (Table 8 p.19, §10.2 p.31).
 * The pager checks the sequence and reports missing messages, so the last number of every
 * address is kept in a file across runs ("<address hex> <number>" per line). */
const char *ermes_number_file = NULL;

#define MAX_NUMBERS 1024

static int ermes_number(uint32_t local_address)
{
	static uint32_t addr[MAX_NUMBERS];
	static int last[MAX_NUMBERS];
	static int count = -1;
	FILE *fp;
	int i;

	if (count < 0) {
		count = 0;
		if (ermes_number_file && (fp = fopen(ermes_number_file, "r"))) {
			unsigned int a;
			int n;

			while (count < MAX_NUMBERS && fscanf(fp, "%x %d", &a, &n) == 2) {
				addr[count] = a;
				last[count++] = n;
			}
			fclose(fp);
		}
	}
	for (i = 0; i < count; i++) {
		if (addr[i] == local_address)
			break;
	}
	if (i == count) {
		if (count == MAX_NUMBERS)
			return 1;
		addr[count] = local_address;
		last[count++] = 0;
	}
	last[i] = (last[i] % 31) + 1;

	if (ermes_number_file && (fp = fopen(ermes_number_file, "w"))) {
		int j;

		for (j = 0; j < count; j++)
			fprintf(fp, "0x%06X %d\n", addr[j], last[j]);
		fclose(fp);
	} else if (ermes_number_file)
		LOGP(DERMES, LOGL_ERROR, "Cannot write message numbers to '%s'.\n", ermes_number_file);
	return last[i];
}

/* Build the next batch into batch_bits, take the pages it carries off the queue.
 * Returns the number of bits. */
static int ermes_next_batch(ermes_t *ermes)
{
	ermes_batch_time_t t;
	ermes_ssi_t ssi;
	ermes_page_t pages[MAX_PAGES];
	ermes_msg_t *msgs[MAX_PAGES], **mp, *m;
	int n = 0, sent, i;

	if (!ermes->started)
		ermes_start_time(ermes);

	t.cycle = ermes->cycle;
	t.ssn = ermes->ssn;
	t.slot = ermes->slot;
	t.batch = ermes_channel_batch(ermes->channel, ermes->slot);
	ermes_ssi(ermes, &ssi);

	for (m = ermes->msg_list; m && n < MAX_PAGES; m = m->next) {
		if (ermes_address_batch(m->page.local_address) != t.batch)
			continue;
		msgs[n] = m;
		pages[n++] = m->page;
	}

	sent = ermes_build_batch(&ermes->net, &t, &ssi, pages, n, ermes->batch_bits);
	if (sent < 0) {
		LOGP_CHAN(DERMES, LOGL_ERROR, "Failed to build batch %c, sending it idle.\n", ermes_batch_name(t.batch));
		sent = ermes_build_batch(&ermes->net, &t, &ssi, NULL, 0, ermes->batch_bits);
		n = 0;
	}

	for (i = 0; i < sent; i++) {
		m = msgs[i];
		LOGP_CHAN(DERMES, LOGL_NOTICE, "Page sent to %s: %s; at minute %d, subsequence %d, time slot %d\n",
			  ermes_print_address(m->page.local_address), ermes_page_str(&m->page),
			  t.cycle, t.ssn, t.slot);
		for (mp = &ermes->msg_list; *mp; mp = &(*mp)->next) {
			if (*mp == m) {
				*mp = m->next;
				break;
			}
		}
		free(m);
	}
	LOGP_CHAN(DERMES, LOGL_DEBUG, "Batch %c at minute %d, subsequence %d, time slot %d: %s; %d page(s)\n",
		  ermes_batch_name(t.batch), t.cycle, t.ssn, t.slot, ermes_ssi_str(&ssi), sent);

	ermes->cw_sent += ermes_batch_length(ermes->slot);
	n = ermes_batch_length(ermes->slot) * ERMES_CW_BITS;
	ermes_next_slot(ermes);
	return n;
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	ermes_t *ermes = (ermes_t *) sender;
	int nbits, i, count;

	while (length) {
		if (ermes->tx_buffer_pos == ermes->tx_buffer_length) {
			nbits = ermes_next_batch(ermes);
			count = 0;
			/* the first batch starts where UTC is in it (ermes_start_time) */
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
	ermes_msg_t *m;

	while ((m = ermes->msg_list)) {
		ermes->msg_list = m->next;
		free(m);
	}
	free(ermes->tx_buffer);
	sender_destroy(&ermes->sender);
	free(ermes);
}

/* The options field of a FIFO line, as flex and pocsag: space separated key=value pairs */
int ermes_parse_page_opts(const char *options, ermes_page_opts_t *opts)
{
	char buf[256], *p, *key, *val;
	ermes_page_opts_t o = *opts;
	char *end;
	long v;

	snprintf(buf, sizeof(buf), "%s", options);
	for (p = strtok(buf, " "); p; p = strtok(NULL, " ")) {
		key = p;
		val = strchr(p, '=');
		if (!val) {
			LOGP(DERMES, LOGL_ERROR, "Option \"%s\" is not key=value.\n", key);
			return -1;
		}
		*val++ = '\0';
		if (!strcasecmp(key, "urgent") || !strcasecmp(key, "alert")) {
			int max = !strcasecmp(key, "urgent") ? 1 : (1 << ERMES_ALERT_BITS) - 1;

			v = strtol(val, &end, 10);
			if (end == val || *end || v < 0 || v > max) {
				LOGP(DERMES, LOGL_ERROR, "Option %s=%s is not 0..%d.\n", key, val, max);
				return -1;
			}
			if (!strcasecmp(key, "urgent"))
				o.urgent = v;
			else
				o.alert = v;
		} else if (!strcasecmp(key, "charset")) {
			o.charset = pager_charset_lookup("ermes", val);
			if (!o.charset) {
				LOGP(DERMES, LOGL_ERROR, "Option charset=%s is not an ERMES codepage.\n", val);
				return -1;
			}
		} else {
			LOGP(DERMES, LOGL_ERROR, "Unknown option \"%s\" (urgent, alert, charset).\n", key);
			return -1;
		}
	}
	*opts = o;
	return 0;
}

int ermes_page_queue(ermes_t *ermes, uint32_t local_address, const ermes_page_opts_t *opts, const char *text)
{
	enum ermes_category category = opts->category;
	ermes_msg_t *m, **mp;
	uint32_t data[ERMES_BATCH_CW_LAST];

	if (local_address >> ERMES_LA_BITS) {
		LOGP(DERMES, LOGL_ERROR, "Local address 0x%X has more than %d bits.\n", local_address, ERMES_LA_BITS);
		return -1;
	}
	if (!text)
		text = "";
	if (strlen(text) >= ERMES_MAX_TEXT) {
		LOGP(DERMES, LOGL_ERROR, "Message too long (at most %d bytes).\n", ERMES_MAX_TEXT - 1);
		return -1;
	}
	m = calloc(1, sizeof(*m));
	if (!m)
		return -1;
	strcpy(m->text, (category == ERMES_CAT_TONE) ? "" : text);
	m->page.local_address = local_address;
	m->page.category = category;
	m->page.urgent = opts->urgent;
	m->page.alert = opts->alert;
	m->page.text = m->text;
	m->page.charset = opts->charset ? opts->charset : ermes_charset;
	if (ermes_data_words(&m->page, data, ERMES_BATCH_CW_LAST) < 0) {
		LOGP(DERMES, LOGL_ERROR, "Message \"%s\" can't be sent as %s: a character without a code, or too long.\n",
		     text, ermes_category_name(category));
		free(m);
		return -1;
	}
	m->page.number = ermes_number(local_address);
	for (mp = &ermes->msg_list; *mp; mp = &(*mp)->next)
		;
	*mp = m;
	LOGP(DERMES, LOGL_NOTICE, "Page queued for %s: %s\n", ermes_print_address(local_address), ermes_page_str(&m->page));
	return 0;
}

int ermes_idle(void)
{
	sender_t *s;

	for (s = sender_head; s; s = s->next) {
		if (((ermes_t *)s)->msg_list)
			return 0;
	}
	return 1;
}

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
	return ermes_page_queue((ermes_t *)sender_head, la, &opts, text);
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
	if (ermes_page_queue((ermes_t *)sender_head, la, &ermes_default_opts, text) < 0)
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
