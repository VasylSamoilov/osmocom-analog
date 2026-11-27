/* ERMES receiver: protocol part (batches, messages)
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
#include <stdlib.h>
#include <string.h>
#include "../libpagercharset/pager_charset.h"
#include "rx.h"

#define SYNC_BITS	(2 * ERMES_CW_BITS)	/* preamble and SW1 (§5.2) */
#define HUNT_ERRORS	4	/* bits off in preamble + SW1 to lock: no false locks on noise */
#define TRACK_ERRORS	12	/* bits off where the next sync is expected */
#define LOST_BATCHES	2	/* batches in a row without sync and SI before the lock is dropped */
#define MD_ERRORS	2	/* a delimiter with 3 bits off is "in error" (-5 §7.6 NOTE) */
#define MD_ERRORS_NO_CODEWORD 4	/* ... but still a delimiter when it is no codeword either */
#define APT_ERRORS	3
#define CTAP_BITS	(12 * ERMES_BIT_RATE)	/* the members' search: 12 s (§10.8.2) */

#define INFO(cw)	((cw) >> (ERMES_CW_BITS - ERMES_INFO_BITS))

static const char numeric_chars[] = "0123456789/ U-.%";

void ermes_rx_init(ermes_rx_t *rx, int channel, const struct pager_charset *charset, void *priv,
		   ermes_rx_batch_cb_t batch_cb, ermes_rx_msg_cb_t msg_cb)
{
	memset(rx, 0, sizeof(*rx));
	rx->channel = channel;
	rx->charset = charset;
	rx->priv = priv;
	rx->batch_cb = batch_cb;
	rx->msg_cb = msg_cb;
}

static uint32_t word_at(const uint8_t *bits, int pos)
{
	uint32_t w = 0;
	int i;

	for (i = 0; i < ERMES_CW_BITS; i++)
		w = (w << 1) | bits[pos + i];
	return w;
}

/* bits off from preamble + SW1, in the last 60 bits */
static int sync_distance(uint64_t shift)
{
	return ermes_distance((uint32_t)(shift >> ERMES_CW_BITS), ERMES_PREAMBLE)
	     + ermes_distance((uint32_t)shift, ERMES_SW1);
}

/* the same with the FM polarity the other way: the first bit of every dibit flips (00 <-> 10,
 * 01 <-> 11) */
static int sync_distance_inverted(uint64_t shift)
{
	uint64_t flip = 0;
	int i;

	for (i = 0; i < SYNC_BITS; i += 2)
		flip |= 1ull << (i + 1);
	return sync_distance(shift ^ flip);
}

/* the slot of a batch type on our channel; -1 if not known */
static int batch_slot(const ermes_rx_t *rx, int batch)
{
	int slot;

	if (!rx->channel)
		return -1;
	for (slot = 0; slot < ERMES_BATCHES; slot++) {
		if (ermes_channel_batch(rx->channel, slot) == batch)
			return slot;
	}
	return -1;
}

/*
 * Messages
 */

static void ctap_expire(ermes_rx_t *rx)
{
	int c, i, j;

	for (c = 0; c < 16; c++) {
		for (i = 0, j = 0; i < rx->ctap[c].n; i++) {
			if (rx->bit_count - rx->ctap[c].bit[i] > (uint64_t)CTAP_BITS + ERMES_BATCH_CW_LAST * ERMES_CW_BITS)
				continue;
			rx->ctap[c].la[j] = rx->ctap[c].la[i];
			rx->ctap[c].number[j] = rx->ctap[c].number[i];
			rx->ctap[c].bit[j] = rx->ctap[c].bit[i];
			j++;
		}
		rx->ctap[c].n = j;
	}
}

/* the text of a message: alpha up to EOM, numeric to the end; stops where -5 §7.5/§7.6 say */
static void decode_text(ermes_rx_t *rx, ermes_rx_msg_t *m, const uint32_t *info, const int8_t *err, int n)
{
	int bit, total = n * ERMES_INFO_BITS, len = 0, i, code, bad, last_bad = -2;
	const char *s;

	m->text[0] = '\0';
	if (m->category == ERMES_CAT_NUMERIC) {
		/* §7.5: nothing after a codeword in error */
		for (bit = 0; bit + ERMES_NUMERIC_BITS <= total; bit += ERMES_NUMERIC_BITS) {
			for (i = bit / ERMES_INFO_BITS; i <= (bit + ERMES_NUMERIC_BITS - 1) / ERMES_INFO_BITS; i++) {
				if (err[i] < 0) {
					m->truncated = 1;
					goto numeric_end;
				}
			}
			for (code = 0, i = 0; i < ERMES_NUMERIC_BITS; i++)
				code = (code << 1) | ((info[(bit + i) / ERMES_INFO_BITS] >> (ERMES_INFO_BITS - 1 - (bit + i) % ERMES_INFO_BITS)) & 1);
			if (len < ERMES_RX_TEXT - 1)
				m->text[len++] = numeric_chars[code];
		}
numeric_end:
		/* trailing fill (spaces) */
		while (len && m->text[len - 1] == ' ')
			len--;
		m->text[len] = '\0';
		return;
	}
	if (m->category != ERMES_CAT_ALPHA && m->category != ERMES_CAT_TRANSPARENT)
		return;
	if (m->category == ERMES_CAT_TRANSPARENT) {
		for (i = 0; i < n && len < ERMES_RX_TEXT - 8; i++)
			len += snprintf(m->text + len, ERMES_RX_TEXT - len, "%05X ", info[i]);
		m->text[len ? len - 1 : 0] = '\0';
		return;
	}
	/* §7.6: no character after an EOM or a delimiter in error, or after two codewords in error
	 * in a row; a character in one codeword in error is shown as '?' */
	for (i = 0; i < n; i++) {
		if (err[i] < 0) {
			if (last_bad == i - 1) {
				m->truncated = 1;
				n = i - 1;
				break;
			}
			last_bad = i;
		}
	}
	total = n * ERMES_INFO_BITS;
	for (bit = 0; bit + ERMES_ALPHA_BITS <= total; bit += ERMES_ALPHA_BITS) {
		for (bad = 0, i = bit / ERMES_INFO_BITS; i <= (bit + ERMES_ALPHA_BITS - 1) / ERMES_INFO_BITS; i++)
			bad |= (err[i] < 0);
		for (code = 0, i = 0; i < ERMES_ALPHA_BITS; i++)
			code = (code << 1) | ((info[(bit + i) / ERMES_INFO_BITS] >> (ERMES_INFO_BITS - 1 - (bit + i) % ERMES_INFO_BITS)) & 1);
		if (bad) {
			s = "?";
		} else if (code == ERMES_EOM) {
			m->eom = 1;
			break;
		} else {
			s = rx->charset ? pager_charset_decode(rx->charset, code) : NULL;
			if (!s)
				s = "?";
		}
		if (len + (int)strlen(s) < ERMES_RX_TEXT - 1) {
			strcpy(m->text + len, s);
			len += strlen(s);
		}
	}
	if (!m->eom && !m->truncated)
		m->truncated = 1;	/* no EOM: its codeword was in error or the message was cut */
}

/* one message: its codewords from the header to (not including) the closing delimiter */
static void message(ermes_rx_t *rx, ermes_rx_batch_t *b, const uint32_t *info, const int8_t *err, int n, int batches, int no_ia)
{
	ermes_rx_msg_t m;
	uint64_t h;
	int i, idx, aii, c;

	memset(&m, 0, sizeof(m));
	m.words = n;
	m.batches = batches;
	m.no_ia = no_ia;
	m.opid = -1;
	for (i = 0; i < n; i++) {
		if (err[i] < 0)
			m.bad++;
		else
			m.corrected += err[i];
	}
	if (n < 2 || err[0] < 0 || err[1] < 0) {
		/* no header: not reported as a message, counted in its batch */
		b->unreadable++;
		return;
	}
	/* Fig.10 p.18: local address 22, number 5, EB 1, AII 1, VIF 7 */
	h = ((uint64_t)info[0] << ERMES_INFO_BITS) | info[1];
	m.local_address = h >> 14;
	m.number = (h >> 9) & 31;
	m.external = (h >> 8) & 1;
	aii = (h >> 7) & 1;
	idx = 2;
	if (m.external && idx < n) {
		/* §5.5.2: the home operator's identity, 13 bits and zeros */
		m.opid = info[idx++] >> (ERMES_INFO_BITS - ERMES_OPID_BITS);
	}
	if (aii) {
		m.ait = (h >> 4) & 7;
		m.ain = h & 15;
		if (m.ait == ERMES_AIT_CTAP) {
			m.kind = ERMES_RX_POINTER;
			c = m.ain;
			ctap_expire(rx);
			if (rx->ctap[c].n < ERMES_RX_MAX_MEMBERS) {
				rx->ctap[c].la[rx->ctap[c].n] = m.local_address;
				rx->ctap[c].number[rx->ctap[c].n] = m.number;
				rx->ctap[c].bit[rx->ctap[c].n] = rx->bit_count;
				rx->ctap[c].n++;
			}
			goto report;
		}
		if (m.ait == ERMES_AIT_MISC && m.ain == ERMES_AIN_LAST_NUMBER) {
			m.kind = ERMES_RX_LAST_NUMBER;
			goto report;
		}
		if (m.ait == ERMES_AIT_MISC && m.ain == 0x2 && idx < n) {
			/* Table 15 AIN 0010, Fig.13: UMI 1, ALERT 3, character set 5, category 2 */
			uint32_t aif = info[idx++] >> (ERMES_INFO_BITS - 11);

			m.kind = ERMES_RX_MESSAGE;
			m.urgent = (aif >> 10) & 1;
			m.alert = (aif >> 7) & 7;
			m.category = aif & 3;
			decode_text(rx, &m, info + idx, err + idx, n - idx);
			goto report;
		}
		if ((m.ait == 0 || m.ait == 1 || m.ait == 5 || m.ait == 6) && idx < n) {
			/* §10.4: link counter 16 bits (and zeros), then the data; alphanumeric */
			m.kind = ERMES_RX_SUBMESSAGE;
			m.link_counter = info[idx++] >> (ERMES_INFO_BITS - 16);
			m.category = ERMES_CAT_ALPHA;
			decode_text(rx, &m, info + idx, err + idx, n - idx);
			goto report;
		}
		m.kind = ERMES_RX_OTHER;
		goto report;
	}
	/* Fig.11: RSVD 1, category 2, UMI 1, alert 3 */
	m.category = (h >> 4) & 3;
	m.urgent = (h >> 3) & 1;
	m.alert = h & 7;
	m.kind = ERMES_RX_MESSAGE;
	if ((m.local_address >> (ERMES_LA_BITS - ERMES_IA_BITS)) == ERMES_CTA_IA) {
		/* stage 3: the members are those whose pointer named this CTAP */
		c = m.local_address & 15;
		m.kind = ERMES_RX_GROUP;
		m.no_ia = 0;
		ctap_expire(rx);
		for (i = 0; i < rx->ctap[c].n; i++) {
			m.member[m.members] = rx->ctap[c].la[i];
			m.member_number[m.members++] = rx->ctap[c].number[i];
		}
		rx->ctap[c].n = 0;
	}
	decode_text(rx, &m, info + idx, err + idx, n - idx);
report:
	rx->messages++;
	if (rx->msg_cb)
		rx->msg_cb(rx->priv, b, &m);
}

/* a message delimiter: close to MD, or a bit further off and no codeword either (MD is not a BCH
 * codeword, so a word that decodes is data). Idle fill on a noisy channel is mostly MDs with a
 * few bits off: taken for data they would make messages out of nothing. */
static int is_md(uint32_t cw)
{
	int d = ermes_distance(cw, ERMES_MD);

	if (d <= MD_ERRORS)
		return 1;
	if (d <= MD_ERRORS_NO_CODEWORD && ermes_bch_decode(&cw) < 0)
		return 1;
	return 0;
}

static int has_ia(const ermes_rx_batch_t *b, uint32_t la)
{
	int i;

	for (i = 0; i < b->ia_count; i++) {
		if (b->ia[i] == la >> (ERMES_LA_BITS - ERMES_IA_BITS))
			return 1;
	}
	return 0;
}

/* the message partition: codewords after de-interleaving, err = bits corrected or -1 */
static void message_partition(ermes_rx_t *rx, ermes_rx_batch_t *b, const uint32_t *cw, int n)
{
	uint32_t info[ERMES_RX_MAX_WORDS];
	int8_t err[ERMES_RX_MAX_WORDS];
	int i = 0, k, e, last_la = -1;
	uint32_t w;

#define IS_MD(x)	is_md(x)

	/* the rest of a message of the batch before comes first (§10.5): up to its delimiter */
	if (rx->pending) {
		for (; i < n && !IS_MD(cw[i]); i++) {
			if (rx->pend_words < ERMES_RX_MAX_WORDS) {
				w = cw[i];
				e = ermes_bch_decode(&w);
				rx->pend_info[rx->pend_words] = INFO(w);
				rx->pend_err[rx->pend_words++] = e;
			}
		}
		rx->pend_batches++;
		if (i == n)
			return;	/* runs on further */
		rx->pending = 0;
		message(rx, b, rx->pend_info, rx->pend_err, rx->pend_words, rx->pend_batches, rx->pend_no_ia);
	} else {
		/* anything before the first delimiter is fill (-5 §7.1.2 Fig.4) */
		for (; i < n && !IS_MD(cw[i]); i++)
			b->fill++;
	}
	while (i < n) {
		/* at a delimiter; more delimiters are idle fill (§5.5.6) */
		while (i < n && IS_MD(cw[i]))
			i++;
		if (i == n)
			break;
		for (k = 0; i < n && !IS_MD(cw[i]); i++, k++) {
			w = cw[i];
			e = ermes_bch_decode(&w);
			if (k < ERMES_RX_MAX_WORDS) {
				info[k] = INFO(w);
				err[k] = e;
			}
		}
		if (k > ERMES_RX_MAX_WORDS)
			k = ERMES_RX_MAX_WORDS;
		/* the order the pagers rely on (a5baa58): ascending local address */
		if (k >= 2 && err[0] >= 0 && err[1] >= 0) {
			uint32_t la = ((((uint64_t)info[0] << ERMES_INFO_BITS) | info[1]) >> 14);

			if ((int)la < last_la)
				b->not_ascending++;
			last_la = la;
		}
		if (i == n) {
			/* runs on into the next batch */
			memcpy(rx->pend_info, info, k * sizeof(info[0]));
			memcpy(rx->pend_err, err, k * sizeof(err[0]));
			rx->pend_words = k;
			rx->pend_batches = 1;
			rx->pend_no_ia = (k >= 2 && err[0] >= 0 && err[1] >= 0)
				? !has_ia(b, (uint32_t)((((uint64_t)info[0] << ERMES_INFO_BITS) | info[1]) >> 14)) : 0;
			rx->pending = 1;
			break;
		}
		message(rx, b, info, err, k, 1, (k >= 2 && err[0] >= 0 && err[1] >= 0)
			? !has_ia(b, (uint32_t)((((uint64_t)info[0] << ERMES_INFO_BITS) | info[1]) >> 14)) : 0);
	}
#undef IS_MD
}

/* a whole batch is in rx->bits */
static void batch(ermes_rx_t *rx)
{
	ermes_rx_batch_t b;
	uint32_t w, msg[ERMES_BATCH_CW_LAST];
	int e, a, m, k, len = rx->length;

	memset(&b, 0, sizeof(b));
	b.length = len;
	b.sync_errors = rx->sync_errors;
	b.flywheel = rx->flywheel;

	/* system information (§5.3, Fig.5): country 7, operator 3, PA 6, ETI 1, BAI 1 / FSI 5,
	 * cycle 6, SSN 3, batch 4; SSI (§5.3.2): SSIT 4, SSIF 14 */
	w = word_at(rx->bits, 2 * ERMES_CW_BITS);
	e = ermes_bch_decode(&w);
	if (e >= 0) {
		w = INFO(w);
		b.country = w >> 11;
		b.operator = (w >> 8) & 7;
		b.pa = (w >> 2) & 63;
		b.eti = (w >> 1) & 1;
		b.bai = w & 1;
		b.corrected += e;
	} else
		b.bad++;
	w = word_at(rx->bits, 3 * ERMES_CW_BITS);
	k = ermes_bch_decode(&w);
	if (k >= 0) {
		w = INFO(w);
		b.fsi = w >> 13;
		b.cycle = (w >> 7) & 63;
		b.ssn = (w >> 4) & 7;
		b.batch = w & 15;
		b.corrected += k;
	} else
		b.bad++;
	b.si_ok = (e >= 0 && k >= 0);
	w = word_at(rx->bits, 4 * ERMES_CW_BITS);
	e = ermes_bch_decode(&w);
	if (e >= 0) {
		w = INFO(w);
		b.ssi_ok = 1;
		b.ssit = w >> 14;
		b.ssif = w & 0x3FFF;
		b.corrected += e;
	} else
		b.bad++;

	/* address partition (§5.4): initial addresses, then APTs up to a codeblock boundary, so the
	 * message partition starts at codeword 10, 19, 28, ... (I + J = 5, 14, 23, ...) */
	for (a = 5; a < len; a++) {
		w = word_at(rx->bits, a * ERMES_CW_BITS);
		if (ermes_distance(w, ERMES_APT) <= APT_ERRORS)
			break;
		e = ermes_bch_decode(&w);
		if (e < 0) {
			b.ia_bad++;
			b.bad++;
			continue;
		}
		b.corrected += e;
		if (b.ia_count < ERMES_MAX_IA)
			b.ia[b.ia_count++] = INFO(w);
	}
	for (m = 10; m <= a; m += ERMES_CODEBLOCK_CW)
		;
	if (m > len) {
		b.bad++;
		m = len;
	}
	b.apt = m - a;
	b.apt_expected = ermes_apt_count(b.ia_count + b.ia_bad);
	b.codeblocks = (len - m) / ERMES_CODEBLOCK_CW;

	/* message partition (§7): codeblocks of 9 interleaved codewords */
	for (k = 0; k < b.codeblocks; k++)
		ermes_deinterleave(rx->bits + (m + k * ERMES_CODEBLOCK_CW) * ERMES_CW_BITS, msg + k * ERMES_CODEBLOCK_CW);
	for (k = 0; k < b.codeblocks * ERMES_CODEBLOCK_CW; k++) {
		if (is_md(msg[k]))
			continue;
		w = msg[k];
		e = ermes_bch_decode(&w);
		if (e < 0)
			b.bad++;
		else
			b.corrected += e;
	}
	rx->batches++;
	/* the messages, then the batch with what was counted in them */
	message_partition(rx, &b, msg, b.codeblocks * ERMES_CODEBLOCK_CW);
	if (rx->batch_cb)
		rx->batch_cb(rx->priv, &b);

	/* lost: no sync where expected and no system information */
	if (rx->flywheel && !b.si_ok) {
		if (++rx->lost >= LOST_BATCHES) {
			rx->locked = 0;
			rx->pending = 0;
		}
	} else
		rx->lost = 0;
}

/* the length of the batch, once its SI is in */
static int batch_length(ermes_rx_t *rx)
{
	uint32_t w = word_at(rx->bits, 3 * ERMES_CW_BITS);
	int slot;

	if (ermes_bch_decode(&w) < 0)
		return 0;
	slot = batch_slot(rx, INFO(w) & 15);
	if (slot < 0)
		return 0;
	return ermes_batch_length(slot);
}

void ermes_rx_bit(ermes_rx_t *rx, int bit)
{
	int d;

	rx->bit_count++;
	rx->shift = ((rx->shift << 1) | (bit & 1)) & ((1ull << SYNC_BITS) - 1);

	if (!rx->locked) {
		d = sync_distance(rx->shift);
		if (d > HUNT_ERRORS) {
			if (!rx->inverted_seen && rx->bit_count >= SYNC_BITS && sync_distance_inverted(rx->shift) <= HUNT_ERRORS)
				rx->inverted_seen = 1;
			return;
		}
		/* lock: the batch starts with the 60 bits just seen */
		rx->locked = 1;
		rx->lost = 0;
		rx->pending = 0;
		rx->sync_errors = d;
		rx->flywheel = 0;
		for (d = 0; d < SYNC_BITS; d++)
			rx->bits[d] = (rx->shift >> (SYNC_BITS - 1 - d)) & 1;
		rx->nbits = SYNC_BITS;
		rx->length = 0;
		return;
	}

	rx->bits[rx->nbits++] = bit & 1;
	if (rx->nbits == SYNC_BITS) {
		/* where the next batch was expected: its preamble and SW1 */
		d = sync_distance(rx->shift);
		rx->sync_errors = d;
		rx->flywheel = (d > TRACK_ERRORS);
	}
	if (!rx->length && rx->nbits == 5 * ERMES_CW_BITS) {
		rx->length = batch_length(rx);
		/* channel not known: up to the long batch, then see where the next sync is */
		if (!rx->length && !rx->channel)
			rx->length = -1;
		else if (!rx->length)
			rx->length = ERMES_BATCH_CW;
	}
	if (rx->length == -1 && rx->nbits == ERMES_BATCH_CW * ERMES_CW_BITS + SYNC_BITS) {
		if (sync_distance(rx->shift) <= TRACK_ERRORS) {
			/* the next batch starts here: a short batch, and its sync is in already */
			rx->length = ERMES_BATCH_CW;
			batch(rx);
			if (!rx->locked)
				return;
			memmove(rx->bits, rx->bits + ERMES_BATCH_CW * ERMES_CW_BITS, SYNC_BITS);
			rx->nbits = SYNC_BITS;
			rx->sync_errors = sync_distance(rx->shift);
			rx->flywheel = 0;
			rx->length = 0;
			return;
		}
	}
	if (rx->length == -1 && rx->nbits == ERMES_BATCH_CW_LAST * ERMES_CW_BITS)
		rx->length = ERMES_BATCH_CW_LAST;
	if (rx->length > 0 && rx->nbits == rx->length * ERMES_CW_BITS) {
		batch(rx);
		rx->nbits = 0;
		rx->length = 0;
	}
}

void ermes_rx_dibit(ermes_rx_t *rx, int dibit)
{
	ermes_rx_bit(rx, (dibit >> 1) & 1);
	ermes_rx_bit(rx, dibit & 1);
}

const char *ermes_rx_msg_str(const ermes_rx_msg_t *m)
{
	static char buf[ERMES_RX_TEXT + 512];
	static const char *cat[] = { "tone", "numeric", "alpha", "transparent" };
	char who[64], extra[160] = "";
	int len, i;

	if (m->kind == ERMES_RX_GROUP)
		snprintf(who, sizeof(who), "common temporary address CTAP %d", m->local_address & 15);
	else
		snprintf(who, sizeof(who), "0x%06X", m->local_address);
	if (m->external) {
		len = strlen(who);
		snprintf(who + len, sizeof(who) - len, " (visitor of %s)", m->opid >= 0 ? ermes_opid_str(m->opid) : "?");
	}
	switch (m->kind) {
	case ERMES_RX_POINTER:
		snprintf(buf, sizeof(buf), "group call pointer to %s number %d: the text follows at CTAP %d", who, m->number, m->ain);
		break;
	case ERMES_RX_LAST_NUMBER:
		snprintf(buf, sizeof(buf), "latest message number of %s: %d", who, m->number);
		break;
	case ERMES_RX_OTHER:
		snprintf(buf, sizeof(buf), "message to %s number %d, additional information AIT %d AIN %d", who, m->number, m->ait, m->ain);
		break;
	default:
		if (m->kind == ERMES_RX_SUBMESSAGE)
			snprintf(extra, sizeof(extra), ", submessage AIT %d, %d bits still to come", m->ait, m->link_counter);
		if (m->kind == ERMES_RX_GROUP) {
			len = strlen(extra);
			len += snprintf(extra + len, sizeof(extra) - len, ", group of %d:", m->members);
			for (i = 0; i < m->members && len < (int)sizeof(extra) - 24; i++)
				len += snprintf(extra + len, sizeof(extra) - len, " 0x%06X/%d", m->member[i], m->member_number[i]);
		}
		snprintf(buf, sizeof(buf), "message to %s number %d, %s%s%s%s%s, %s, alert %d%s", who, m->number,
			 cat[m->category & 3], m->text[0] ? " \"" : "", m->text, m->text[0] ? "\"" : "",
			 m->truncated ? " (cut: codewords in error or no EOM)" : "",
			 m->urgent ? "urgent" : "not urgent", m->alert, extra);
	}
	len = strlen(buf);
	if (m->batches > 1)
		len += snprintf(buf + len, sizeof(buf) - len, ", over %d batches", m->batches);
	if (m->no_ia)
		len += snprintf(buf + len, sizeof(buf) - len, ", no initial address in its batch");
	if (m->corrected || m->bad)
		snprintf(buf + len, sizeof(buf) - len, " [%d bits corrected, %d codewords in error]", m->corrected, m->bad);
	return buf;
}
