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
#define APT_ERRORS_NO_CODEWORD 6
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

/* bits of the information fields after the header, MSB of the first word first */
static uint32_t get_bits(const uint32_t *info, int bit, int n)
{
	uint32_t v = 0;
	int i;

	for (i = 0; i < n; i++, bit++)
		v = (v << 1) | ((info[bit / ERMES_INFO_BITS] >> (ERMES_INFO_BITS - 1 - bit % ERMES_INFO_BITS)) & 1);
	return v;
}

/* the state of the bits [bit, bit + n): 0 all from correct codewords, 1 one corrected, 2 one not
 * decodable */
static int bits_state(const int8_t *err, int bit, int n)
{
	int i, s = 0;

	for (i = bit / ERMES_INFO_BITS; i <= (bit + n - 1) / ERMES_INFO_BITS; i++) {
		if (err[i] < 0)
			return 2;
		if (err[i] > 0)
			s = 1;
	}
	return s;
}

/* the start of the batch with the receiver's initial address, if it was sent in the 12 s before
 * the batch starting at 'now' (-4 §10.8.1/§10.8.2: a receiver searches 12 s from the start of that
 * batch); 0 if not */
static uint64_t ia_start(const ermes_rx_t *rx, const ermes_rx_batch_t *b, uint64_t now, uint32_t la)
{
	uint32_t ia = la >> (ERMES_LA_BITS - ERMES_IA_BITS);
	uint64_t best = 0;
	int h, i;

	for (i = 0; i < b->ia_count; i++) {
		if (b->ia[i] == ia)
			return now;
	}
	for (h = 0; h < ERMES_BATCHES + 1; h++) {
		if (!rx->hist[h].start || rx->hist[h].start >= now || now - rx->hist[h].start >= (uint64_t)CTAP_BITS)
			continue;
		for (i = 0; i < rx->hist[h].n; i++) {
			if (rx->hist[h].ia[i] == ia && rx->hist[h].start > best)
				best = rx->hist[h].start;
		}
	}
	return best;
}

static uint32_t text_hash(const char *s)
{
	uint32_t h = 2166136261u;

	while (*s)
		h = (h ^ (uint8_t)*s++) * 16777619u;
	return h;
}

/* message numbers of a receiver: a repeat (-5 §7.7) or numbers skipped (-5 §8.2) */
static void numbers(ermes_rx_t *rx, ermes_rx_msg_t *m)
{
	int i, oldest = 0, expect;
	uint32_t h = text_hash(m->text);

	if (!m->number || m->kind == ERMES_RX_GROUP || m->kind == ERMES_RX_SUBMESSAGE || m->kind == ERMES_RX_OTHER)
		return;
	rx->seen_clock++;
	for (i = 0; i < 256; i++) {
		if (rx->seen[i].used && rx->seen[i].la == m->local_address && rx->seen[i].opid == m->opid)
			break;
		if (rx->seen[i].used < rx->seen[oldest].used)
			oldest = i;
	}
	if (i == 256) {
		i = oldest;
		rx->seen[i].la = m->local_address;
		rx->seen[i].opid = m->opid;
		rx->seen[i].number = m->number;
		rx->seen[i].text_hash = h;
		rx->seen[i].used = rx->seen_clock;
		return;
	}
	rx->seen[i].used = rx->seen_clock;
	if (m->kind == ERMES_RX_LAST_NUMBER) {
		/* the latest number sent: anything after the one seen was missed */
		if (m->number != rx->seen[i].number) {
			m->missing_from = rx->seen[i].number % 31 + 1;
			m->missing_to = m->number;
			rx->seen[i].number = m->number;
		}
		return;
	}
	if (m->number == rx->seen[i].number) {
		m->repeat = (h == rx->seen[i].text_hash) ? 1 : 2;
		return;
	}
	expect = rx->seen[i].number % 31 + 1;
	if (m->number != expect) {
		m->missing_from = expect;
		m->missing_to = (m->number + 29) % 31 + 1;
	}
	rx->seen[i].number = m->number;
	rx->seen[i].text_hash = h;
}

/* the text of a message: alpha up to EOM, numeric to the end; stops where -5 §7.5/§7.6 say */
static void decode_text(ermes_rx_t *rx, ermes_rx_msg_t *m, const uint32_t *info, const int8_t *err, int n)
{
	int bit, total = n * ERMES_INFO_BITS, len = 0, i, code, state, eom_bit = -1;
	const char *s;
	char hex[8];

	m->text[0] = '\0';
	if (m->category == ERMES_CAT_NUMERIC) {
		/* a monitor, not a pager: digits from a codeword in error are '?' and the text goes on
		 * (a pager shows nothing after it, -5 §7.5) */
		for (bit = 0; bit + ERMES_NUMERIC_BITS <= total; bit += ERMES_NUMERIC_BITS) {
			state = bits_state(err, bit, ERMES_NUMERIC_BITS);
			if (len < ERMES_RX_TEXT - 1) {
				m->char_state[len] = state;
				m->text[len++] = (state == 2) ? '?' : numeric_chars[get_bits(info, bit, ERMES_NUMERIC_BITS)];
			}
		}
		/* trailing fill (spaces) */
		while (len && m->text[len - 1] == ' ')
			len--;
		m->text[len] = '\0';
		for (i = 0; i < len; i++) {
			m->chars_corrected += (m->char_state[i] == 1);
			m->chars_bad += (m->char_state[i] == 2);
		}
		m->chars = len;
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
	/* a monitor, not a pager: characters from a codeword in error are '?' and the text goes on (a
	 * pager stops after two in a row, -5 §7.6); the 7-bit grid does not depend on them */
	for (bit = 0; bit + ERMES_ALPHA_BITS <= total; bit += ERMES_ALPHA_BITS) {
		state = bits_state(err, bit, ERMES_ALPHA_BITS);
		code = get_bits(info, bit, ERMES_ALPHA_BITS);
		if (state == 2) {
			s = "?";
		} else if (code == ERMES_EOM) {
			/* the end mark is not shown */
			m->eom = 1;
			eom_bit = bit + ERMES_ALPHA_BITS;
			break;
		} else {
			/* a code without a glyph as <0xNN>: the log line stays one line (LF, CR, ESC
			 * have their glyphs) and the code is not lost */
			s = rx->charset ? pager_charset_decode(rx->charset, code) : NULL;
			if (!s) {
				snprintf(hex, sizeof(hex), "<0x%02X>", code);
				s = hex;
			}
		}
		if (len + (int)strlen(s) < ERMES_RX_TEXT - 1 && m->chars < ERMES_RX_TEXT) {
			strcpy(m->text + len, s);
			len += strlen(s);
			m->char_state[m->chars++] = state;
			m->chars_corrected += (state == 1);
			m->chars_bad += (state == 2);
		}
	}
	if (!m->eom) {
		m->no_eom = 1;	/* its codeword was in error or the message was cut */
		return;
	}
	/* after the EOM: the fill, EOM repeated to the end of the codeword (-4 §5.5.4); anything else
	 * is shown apart, with a warning */
	for (bit = eom_bit; bit < total; bit++) {
		if (bits_state(err, bit, 1) == 2)
			continue;
		if (get_bits(info, bit, 1) != ((ERMES_EOM >> (ERMES_ALPHA_BITS - 1 - (bit - eom_bit) % ERMES_ALPHA_BITS)) & 1)) {
			m->bad_fill = 1;
			break;
		}
	}
	if (m->bad_fill) {
		int alen = 0;

		for (bit = eom_bit; bit + ERMES_ALPHA_BITS <= total; bit += ERMES_ALPHA_BITS) {
			code = get_bits(info, bit, ERMES_ALPHA_BITS);
			if (bits_state(err, bit, ERMES_ALPHA_BITS) == 2)
				s = "?";
			else if (!(s = rx->charset ? pager_charset_decode(rx->charset, code) : NULL)) {
				snprintf(hex, sizeof(hex), "<0x%02X>", code);
				s = hex;
			}
			if (alen + (int)strlen(s) < (int)sizeof(m->after_eom) - 1) {
				strcpy(m->after_eom + alen, s);
				alen += strlen(s);
			}
		}
	}
}

/* one message: its codewords from the header to (not including) the closing delimiter. The caller
 * has checked its header and that its receiver was initially addressed: ia_bit = the start of that
 * batch. missing: the rest was not received (lock lost while it ran on). */
static void message(ermes_rx_t *rx, ermes_rx_batch_t *b, const uint32_t *info, const int8_t *err, int n, int batches,
		    uint64_t ia_bit, int missing)
{
	ermes_rx_msg_t m;
	uint64_t h;
	int i, j, bit, aii, c, len;

	memset(&m, 0, sizeof(m));
	m.words = n;
	m.batches = batches;
	m.opid = -1;
	m.character_set = -1;
	for (i = 0; i < n; i++) {
		if (err[i] < 0)
			m.bad++;
		else
			m.corrected += err[i];
	}
	/* Fig.10 p.18: local address 22, number 5, EB 1, AII 1, VIF 7 */
	h = ((uint64_t)info[0] << ERMES_INFO_BITS) | info[1];
	m.local_address = h >> 14;
	m.number = (h >> 9) & 31;
	m.external = (h >> 8) & 1;
	aii = (h >> 7) & 1;
	bit = 2 * ERMES_INFO_BITS;
	/* §5.5.2: EB = 1, the home operator's identity, 13 bits; the AIF (if any) right after it, then
	 * zeros to the end of the codeword */
	if (m.external && bit + ERMES_OPID_BITS <= n * ERMES_INFO_BITS) {
		m.opid = get_bits(info, bit, ERMES_OPID_BITS);
		bit += ERMES_OPID_BITS;
	}
	m.kind = ERMES_RX_MESSAGE;
	if (aii) {
		m.ait = (h >> 4) & 7;
		m.ain = h & 15;
		if (m.ait == ERMES_AIT_CTAP) {
			/* stage 2: kept for the text, with the start of the member's initial address batch */
			m.kind = ERMES_RX_POINTER;
			c = m.ain;
			for (i = 0, j = 0; i < rx->ctap[c].n; i++) {
				if (rx->cur_start - rx->ctap[c].ia_start[i] >= (uint64_t)CTAP_BITS)
					continue;
				rx->ctap[c].la[j] = rx->ctap[c].la[i];
				rx->ctap[c].number[j] = rx->ctap[c].number[i];
				rx->ctap[c].ia_start[j++] = rx->ctap[c].ia_start[i];
			}
			rx->ctap[c].n = j;
			if (j < ERMES_RX_MAX_MEMBERS) {
				rx->ctap[c].la[j] = m.local_address;
				rx->ctap[c].number[j] = m.number;
				rx->ctap[c].ia_start[j] = ia_bit;
				rx->ctap[c].n++;
			}
			goto report;
		}
		if (m.ait == ERMES_AIT_MISC && m.ain == ERMES_AIN_LAST_NUMBER) {
			m.kind = ERMES_RX_LAST_NUMBER;
			goto report;
		}
		if (m.ait == ERMES_AIT_MISC && m.ain == 0x2 && bit + 11 <= n * ERMES_INFO_BITS) {
			/* Table 15 AIN 0010, Fig.13: UMI 1, ALERT 3, character set 5, category 2 */
			uint32_t aif = get_bits(info, bit, 11);

			bit += 11;
			m.urgent = (aif >> 10) & 1;
			m.alert = (aif >> 7) & 7;
			m.character_set = (aif >> 2) & 31;
			m.category = aif & 3;
			goto text;
		}
		if (m.ait == 0 || m.ait == 1 || m.ait == 5 || m.ait == 6) {
			/* §10.4, Table 11 p.20: AIN = urgent and alert type; AIF = category 2, length 16
			 * (with an additional character set, AIT 110/101: character set 5, reserved 2) */
			len = (m.ait == 0 || m.ait == 1) ? 18 : 25;
			if (bit + len > n * ERMES_INFO_BITS) {
				m.kind = ERMES_RX_OTHER;
				goto report;
			}
			m.kind = ERMES_RX_SUBMESSAGE;
			m.urgent = (m.ain >> 3) & 1;
			m.alert = m.ain & 7;
			m.category = get_bits(info, bit, 2);
			m.link_counter = get_bits(info, bit + 2, 16);
			if (len == 25)
				m.character_set = get_bits(info, bit + 18, 5);
			bit += len;
			goto text;
		}
		m.kind = ERMES_RX_OTHER;
		goto report;
	}
	/* Fig.11: RSVD 1, category 2, UMI 1, alert 3 */
	m.category = (h >> 4) & 3;
	m.urgent = (h >> 3) & 1;
	m.alert = h & 7;
	if ((m.local_address >> (ERMES_LA_BITS - ERMES_IA_BITS)) == ERMES_CTA_IA) {
		/* stage 3: the members whose pointer named this CTAP and whose search (12 s from the
		 * start of their initial address batch, §10.8.2) still runs */
		c = m.local_address & 15;
		m.kind = ERMES_RX_GROUP;
		for (i = 0; i < rx->ctap[c].n; i++) {
			if (rx->cur_start - rx->ctap[c].ia_start[i] >= (uint64_t)CTAP_BITS)
				continue;
			m.member[m.members] = rx->ctap[c].la[i];
			m.member_number[m.members++] = rx->ctap[c].number[i];
		}
		rx->ctap[c].n = 0;
	}
text:
	/* the data starts with the codeword after the header (and OPID/AIF and their zeros) */
	i = (bit + ERMES_INFO_BITS - 1) / ERMES_INFO_BITS;
	if (i <= n)
		decode_text(rx, &m, info + i, err + i, n - i);
	if (missing) {
		m.missing_part = 1;
		len = strlen(m.text);
		snprintf(m.text + len, ERMES_RX_TEXT - len, "<..>");
	}
report:
	numbers(rx, &m);
	rx->messages++;
	if (m.kind == ERMES_RX_MESSAGE || m.kind == ERMES_RX_GROUP || m.kind == ERMES_RX_SUBMESSAGE) {
		if (m.missing_part || m.no_eom)
			rx->incomplete++;
		else if (m.chars_bad)
			rx->damaged++;
		else
			rx->complete++;
	}
	rx->runs_on += (batches > 1);
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

/* codewords from *i up to the next delimiter or the end; *i ends there. Returns how many. */
static int read_run(const uint32_t *cw, int n, int *i, uint32_t *info, int8_t *err)
{
	uint32_t w;
	int k = 0, e;

	for (; *i < n && !is_md(cw[*i]); (*i)++, k++) {
		w = cw[*i];
		e = ermes_bch_decode(&w);
		if (k < ERMES_RX_MAX_WORDS) {
			info[k] = INFO(w);
			err[k] = e;
		}
	}
	return (k > ERMES_RX_MAX_WORDS) ? ERMES_RX_MAX_WORDS : k;
}

/* is a run of codewords a message? Its header must decode and its receiver must have been
 * initially addressed in the 12 s before (a text to a CTA needs none): else it is fill or a
 * message nobody searches for (-4 §10.8.1, -5 7.1.5). *ia_bit = the start of that batch. */
static int accept(ermes_rx_t *rx, ermes_rx_batch_t *b, const uint32_t *info, const int8_t *err, int k, uint64_t *ia_bit)
{
	uint32_t la;

	if (k < 2 || err[0] < 0 || err[1] < 0) {
		b->unreadable++;
		return 0;
	}
	la = ((((uint64_t)info[0] << ERMES_INFO_BITS) | info[1]) >> 14);
	if ((la >> (ERMES_LA_BITS - ERMES_IA_BITS)) == ERMES_CTA_IA) {
		*ia_bit = rx->cur_start;
		return 1;
	}
	*ia_bit = ia_start(rx, b, rx->cur_start, la);
	if (!*ia_bit) {
		b->no_ia++;
		return 0;
	}
	return 1;
}

static void run_on(ermes_rx_t *rx, const uint32_t *info, const int8_t *err, int k, uint64_t ia_bit)
{
	memcpy(rx->pend_info, info, k * sizeof(info[0]));
	memcpy(rx->pend_err, err, k * sizeof(err[0]));
	rx->pend_words = k;
	rx->pend_batches = 1;
	rx->pend_ia_bit = ia_bit;
	rx->pending = 1;
}

/* the message partition: codewords after de-interleaving */
static void message_partition(ermes_rx_t *rx, ermes_rx_batch_t *b, const uint32_t *cw, int n)
{
	uint32_t info[ERMES_RX_MAX_WORDS];
	int8_t err[ERMES_RX_MAX_WORDS];
	int i = 0, k, last_la = -1;
	uint64_t ia_bit;

	if (rx->pending) {
		/* the rest of a message of the batch before comes first (§10.5), up to its delimiter */
		k = read_run(cw, n, &i, info, err);
		if (rx->pend_words + k > ERMES_RX_MAX_WORDS)
			k = ERMES_RX_MAX_WORDS - rx->pend_words;
		memcpy(rx->pend_info + rx->pend_words, info, k * sizeof(info[0]));
		memcpy(rx->pend_err + rx->pend_words, err, k * sizeof(err[0]));
		rx->pend_words += k;
		rx->pend_batches++;
		if (i == n)
			goto out;	/* runs on further */
		rx->pending = 0;
		message(rx, b, rx->pend_info, rx->pend_err, rx->pend_words, rx->pend_batches, rx->pend_ia_bit, 0);
	} else if (rx->open_md) {
		/* the batch before ended with a delimiter: a message may start right here (-5 Fig.5,
		 * Fig.9); fill otherwise (Fig.4) */
		k = read_run(cw, n, &i, info, err);
		if (k && accept(rx, b, info, err, k, &ia_bit)) {
			if (i == n) {
				run_on(rx, info, err, k, ia_bit);
				goto out;
			}
			message(rx, b, info, err, k, 1, ia_bit, 0);
		} else
			b->fill += k;
	} else {
		/* anything before the first delimiter is fill (-5 §7.1.2 Fig.4) */
		for (; i < n && !is_md(cw[i]); i++)
			b->fill++;
	}
	while (i < n) {
		/* at a delimiter; more delimiters are idle fill (§5.5.6) */
		while (i < n && is_md(cw[i]))
			i++;
		if (i == n)
			break;
		k = read_run(cw, n, &i, info, err);
		if (!accept(rx, b, info, err, k, &ia_bit))
			continue;
		/* the order the pagers rely on (a5baa58): ascending local address */
		{
			uint32_t la = ((((uint64_t)info[0] << ERMES_INFO_BITS) | info[1]) >> 14);

			if ((int)la < last_la)
				b->not_ascending++;
			last_la = la;
		}
		if (i == n) {
			run_on(rx, info, err, k, ia_bit);	/* into the next batch */
			break;
		}
		message(rx, b, info, err, k, 1, ia_bit, 0);
	}
out:
	rx->open_md = (!rx->pending && n > 0 && is_md(cw[n - 1]));
}

/* a message that was running on when the lock was lost: reported with <..> for the rest */
static void flush_pending(ermes_rx_t *rx)
{
	if (!rx->pending)
		return;
	rx->pending = 0;
	message(rx, &rx->last_batch, rx->pend_info, rx->pend_err, rx->pend_words, rx->pend_batches, rx->pend_ia_bit, 1);
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
		/* an APT further off is still one when it is no codeword either (APT is not one) */
		if (e < 0 && ermes_distance(word_at(rx->bits, a * ERMES_CW_BITS), ERMES_APT) <= APT_ERRORS_NO_CODEWORD)
			break;
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
	rx->cur_start = rx->batch_start;
	/* the messages, then the batch with what was counted in them */
	message_partition(rx, &b, msg, b.codeblocks * ERMES_CODEBLOCK_CW);
	if (rx->batch_cb)
		rx->batch_cb(rx->priv, &b);
	rx->last_batch = b;

	/* its initial addresses, for the receivers' 12 s search (§10.8) */
	rx->hist[rx->hist_pos].start = rx->cur_start;
	rx->hist[rx->hist_pos].n = b.ia_count;
	memcpy(rx->hist[rx->hist_pos].ia, b.ia, b.ia_count * sizeof(b.ia[0]));
	rx->hist_pos = (rx->hist_pos + 1) % (ERMES_BATCHES + 1);

	/* lost: no sync where expected and no system information */
	if (rx->flywheel && !b.si_ok) {
		if (++rx->lost >= LOST_BATCHES) {
			rx->locked = 0;
			flush_pending(rx);
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
		flush_pending(rx);
		rx->open_md = 0;
		rx->batch_start = rx->bit_count - SYNC_BITS + 1;
		rx->sync_errors = d;
		rx->flywheel = 0;
		for (d = 0; d < SYNC_BITS; d++)
			rx->bits[d] = (rx->shift >> (SYNC_BITS - 1 - d)) & 1;
		rx->nbits = SYNC_BITS;
		rx->length = 0;
		return;
	}

	if (!rx->nbits)
		rx->batch_start = rx->bit_count;
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
			rx->batch_start = rx->bit_count - SYNC_BITS + 1;
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
		if (m->character_set >= 0) {
			len = strlen(extra);
			snprintf(extra + len, sizeof(extra) - len, ", character set %d", m->character_set);
		}
		snprintf(buf, sizeof(buf), "message to %s number %d, %s%s%s%s, %s, alert %d%s", who, m->number,
			 cat[m->category & 3], m->text[0] ? " \"" : "", m->text, m->text[0] ? "\"" : "",
			 m->urgent ? "urgent" : "not urgent", m->alert, extra);
	}
	len = strlen(buf);
	/* status: complete, damaged (characters '?'), incomplete (a part missing <..>, no EOM) */
	if (m->kind == ERMES_RX_MESSAGE || m->kind == ERMES_RX_GROUP || m->kind == ERMES_RX_SUBMESSAGE) {
		if (m->missing_part || m->no_eom)
			len += snprintf(buf + len, sizeof(buf) - len, "; INCOMPLETE (%s)",
					m->missing_part ? "the rest not received" : "no end of message");
		else if (m->chars_bad)
			len += snprintf(buf + len, sizeof(buf) - len, "; DAMAGED (%d of %d characters bad)", m->chars_bad, m->chars);
		else
			len += snprintf(buf + len, sizeof(buf) - len, "; complete");
		if (m->chars_corrected)
			len += snprintf(buf + len, sizeof(buf) - len, ", %d of %d characters corrected", m->chars_corrected, m->chars);
		if (m->bad_fill)
			len += snprintf(buf + len, sizeof(buf) - len, "; WARNING after the end of message not the fill: \"%s\"", m->after_eom);
	}
	if (m->batches > 1)
		len += snprintf(buf + len, sizeof(buf) - len, "; %d words over %d batches", m->words, m->batches);
	if (m->repeat)
		len += snprintf(buf + len, sizeof(buf) - len, "; REPEAT of number %d%s", m->number, m->repeat == 2 ? " with another text" : "");
	if (m->missing_from)
		len += snprintf(buf + len, sizeof(buf) - len, "; numbers %d..%d missing", m->missing_from, m->missing_to);
	if (m->corrected || m->bad)
		snprintf(buf + len, sizeof(buf) - len, " [%d bits corrected, %d codewords in error]", m->corrected, m->bad);
	return buf;
}
