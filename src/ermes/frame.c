/* ERMES air interface: batch builder (L4 information format)
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
 * References are to ETS 300 133-4 ed.2 (November 1997) and ETS 300 133-2 ed.2.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <ctype.h>
#include "frame.h"

#define MAX_MSG_WORDS	(ERMES_BATCH_CW_LAST)
#define FLAG_BITS	1	/* ETI, BAI, RSVD, EB, AII, UMI */
#define ERMES_MAX_TEXT_STR	1200

/* Append a field MSB first: the fields of a word are given in the order of the figure */
static void field(uint64_t *word, uint32_t value, int bits)
{
	*word = (*word << bits) | (value & ((1u << bits) - 1));
}

/* Fig.7 p.17: zone, hour, date, RSVD */
uint32_t ermes_ssif_zone_hour_date(int zone, int hour, int date)
{
	uint64_t w = 0;

	field(&w, zone, ERMES_ZONE_BITS);
	field(&w, hour, ERMES_HOUR_BITS);
	field(&w, date, ERMES_DATE_BITS);
	field(&w, 0, FLAG_BITS);
	return w;
}

/* Fig.8 p.17: day of week (1 = Monday), month (1 = January), year (0 = 1990) */
uint32_t ermes_ssif_day_month_year(int weekday, int month, int year)
{
	uint64_t w = 0;

	field(&w, weekday, ERMES_WEEKDAY_BITS);
	field(&w, month, ERMES_MONTH_BITS);
	field(&w, year - ERMES_YEAR_BASE, ERMES_YEAR_BITS);
	return w;
}

/* Fig.9 p.17: MVR1, SVR1 */
uint32_t ermes_ssif_version(int mvr1, int svr1)
{
	uint64_t w = 0;

	field(&w, mvr1, ERMES_VERSION_BITS);
	field(&w, svr1, ERMES_VERSION_BITS);
	return w;
}

/* Fig.5 p.15, first word: country, operator, PA, ETI, BAI */
uint32_t ermes_si_word1(const ermes_network_t *net, int eti)
{
	uint64_t w = 0;

	field(&w, net->country, ERMES_COUNTRY_BITS);
	field(&w, net->operator, ERMES_OPERATOR_BITS);
	field(&w, net->pa, ERMES_PA_BITS);
	field(&w, eti, FLAG_BITS);
	field(&w, net->bai, FLAG_BITS);
	return w;
}

/* Fig.5 p.15, second word: FSI, cycle, SSN, batch */
uint32_t ermes_si_word2(const ermes_network_t *net, const ermes_batch_time_t *t)
{
	uint64_t w = 0;

	field(&w, net->fsi, ERMES_FSI_BITS);
	field(&w, t->cycle, ERMES_CYCLE_BITS);
	field(&w, t->ssn, ERMES_SSN_BITS);
	field(&w, t->batch, ERMES_BATCH_BITS);
	return w;
}

/* Fig.6 p.16: SSIT, SSIF */
uint32_t ermes_ssi_word(const ermes_ssi_t *ssi)
{
	uint64_t w = 0;

	field(&w, ssi->ssit, ERMES_SSIT_BITS);
	field(&w, ssi->ssif, ERMES_SSIF_BITS);
	return w;
}

/* Fig.10 p.18: local address | message number, external bit, AII, VIF; Fig.11 p.19 VIF with
 * AII = 0: RSVD, paging category, UMI, alert. 36 bits = two information words; the first is
 * the 18 MSBs of the local address = the initial address. */
void ermes_header_words(const ermes_page_t *page, uint32_t info[2])
{
	uint64_t h = 0;

	field(&h, page->local_address, ERMES_LA_BITS);
	field(&h, page->number, ERMES_MSG_NUMBER_BITS);
	field(&h, 0, FLAG_BITS);	/* external bit: home network */
	field(&h, 0, FLAG_BITS);	/* AII: no additional information */
	field(&h, 0, FLAG_BITS);	/* RSVD */
	field(&h, page->category, ERMES_CATEGORY_BITS);
	field(&h, page->urgent, FLAG_BITS);
	field(&h, page->alert, ERMES_ALERT_BITS);

	info[0] = (h >> ERMES_INFO_BITS) & ((1u << ERMES_INFO_BITS) - 1);
	info[1] = h & ((1u << ERMES_INFO_BITS) - 1);
}

/* The codepage of a page: numeric Table B.1, alphanumeric the page's (default Table B.3) */
static const struct pager_charset *page_charset(const ermes_page_t *page)
{
	if (page->category == ERMES_CAT_NUMERIC)
		return &pager_charset_ermes_generic_generic_numeric;
	return page->charset ? page->charset : &pager_charset_ermes_generic_generic_standard;
}

/* Bit writer into 18-bit information words, MSB first (§5.5.4 p.22) */
struct bitwriter {
	uint32_t	*words;
	int		max_words;
	int		nbits;
};

static int put_bits(struct bitwriter *w, uint32_t value, int len)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		int word = w->nbits / ERMES_INFO_BITS;
		int pos = ERMES_INFO_BITS - 1 - (w->nbits % ERMES_INFO_BITS);

		if (word >= w->max_words)
			return -1;
		if (pos == ERMES_INFO_BITS - 1)
			w->words[word] = 0;
		if ((value >> i) & 1)
			w->words[word] |= 1u << pos;
		w->nbits++;
	}
	return 0;
}

/* Fill the rest of the last word with a character repeated, partial characters MSB first */
static int fill_word(struct bitwriter *w, uint32_t pattern, int len)
{
	while (w->nbits % ERMES_INFO_BITS) {
		int left = ERMES_INFO_BITS - (w->nbits % ERMES_INFO_BITS);

		if (left >= len) {
			if (put_bits(w, pattern, len) < 0)
				return -1;
		} else {
			if (put_bits(w, pattern >> (len - left), left) < 0)
				return -1;
		}
	}
	return 0;
}

int ermes_data_words(const ermes_page_t *page, uint32_t *info, int max_words)
{
	struct bitwriter w = { info, max_words, 0 };
	const struct pager_charset *cs = page_charset(page);
	const char *text = page->text ? page->text : "";
	uint8_t codes[ERMES_MAX_MSG_WORDS * ERMES_INFO_BITS / ERMES_NUMERIC_BITS];
	const char *space = " ";
	int n, i, fill;

	if (page->category == ERMES_CAT_TONE)
		return 0;
	if (page->category != ERMES_CAT_NUMERIC && page->category != ERMES_CAT_ALPHA)
		return -1;
	n = pager_charset_encode_text(cs, text, strlen(text), codes, sizeof(codes), NULL);
	if (n < 0)
		return -1;
	for (i = 0; i < n; i++) {
		if (put_bits(&w, codes[i], cs->code_bits) < 0)
			return -1;
	}
	if (page->category == ERMES_CAT_NUMERIC) {
		/* no terminator; space and partial spaces fill the codeword (§5.5.4 p.23) */
		fill = pager_charset_encode(cs, &space);
	} else {
		/* EOM, then EOM and partial EOM fill the codeword (§5.5.4 p.23) */
		if (put_bits(&w, ERMES_EOM, cs->code_bits) < 0)
			return -1;
		fill = ERMES_EOM;
	}
	if (fill_word(&w, fill, cs->code_bits) < 0)
		return -1;
	return w.nbits / ERMES_INFO_BITS;
}

static void put_word(uint8_t *bits, int *n, uint32_t cw)
{
	int i;

	for (i = ERMES_CW_BITS - 1; i >= 0; i--)
		bits[(*n)++] = (cw >> i) & 1;
}

/* Initial address of a page */
static uint32_t page_ia(const ermes_page_t *page)
{
	return ermes_initial_address(page->local_address);
}

int ermes_build_batch(const ermes_network_t *net, const ermes_batch_time_t *t, const ermes_ssi_t *ssi,
		      const ermes_page_t *pages, int num_pages, ermes_carry_t *carry, uint8_t *bits)
{
	uint32_t msg[MAX_MSG_WORDS];	/* message partition codewords before interleaving */
	uint32_t whole[ERMES_MAX_MSG_WORDS + 3];
	int order[ERMES_MAX_IA];
	int i, j, k, n = 0, nmsg, sent, cap, num_cb, nwhole, room;

	if (t->slot < 0 || t->slot >= ERMES_BATCHES || t->batch < 0 || t->batch >= ERMES_BATCHES)
		return -1;
	if (num_pages > ERMES_MAX_IA)
		num_pages = ERMES_MAX_IA;

	nmsg = 0;
	sent = 0;
	if (carry && carry->count) {
		/* the rest of a message of the batch before comes first (§10.5): as much as fits
		 * with no initial address; if it does not all fit, the batch carries only it */
		cap = ermes_max_codeblocks(t->slot, 0) * ERMES_CODEBLOCK_CW;
		nmsg = (carry->count < cap) ? carry->count : cap;
		memcpy(msg, carry->cw, nmsg * sizeof(uint32_t));
		memmove(carry->cw, carry->cw + nmsg, (carry->count - nmsg) * sizeof(uint32_t));
		carry->count -= nmsg;
		if (carry->count)
			num_pages = 0;
		/* its closing delimiter is the leading one of the next message (one delimiter separates
		 * messages, §5.5.5 p.23) */
	} else
		msg[nmsg++] = ERMES_MD;

	/* the pages of this batch type: per message header + data + closing delimiter (§5.5.5); a
	 * message that does not fit whole is started and runs on through the carry, as the last one */
	for (i = 0; i < num_pages; i++) {
		uint32_t hdr[2], data[ERMES_MAX_MSG_WORDS];
		int nd;

		if (ermes_address_batch(pages[i].local_address) != t->batch)
			return -1;
		nd = ermes_data_words(&pages[i], data, ERMES_MAX_MSG_WORDS);
		if (nd < 0)
			return -1;
		ermes_header_words(&pages[i], hdr);
		nwhole = 0;
		whole[nwhole++] = ermes_bch_encode(hdr[0]);
		whole[nwhole++] = ermes_bch_encode(hdr[1]);
		for (j = 0; j < nd; j++)
			whole[nwhole++] = ermes_bch_encode(data[j]);
		whole[nwhole++] = ERMES_MD;
		/* one more initial address takes room from the message partition */
		cap = ermes_max_codeblocks(t->slot, sent + 1) * ERMES_CODEBLOCK_CW;
		room = cap - nmsg;
		if (nwhole <= room) {
			memcpy(msg + nmsg, whole, nwhole * sizeof(uint32_t));
			nmsg += nwhole;
			order[sent++] = i;
			continue;
		}
		/* runs on: the header and at least one data word here, the rest in the next batches */
		if (carry && room >= 3) {
			memcpy(msg + nmsg, whole, room * sizeof(uint32_t));
			nmsg += room;
			memcpy(carry->cw, whole + room, (nwhole - room) * sizeof(uint32_t));
			carry->count = nwhole - room;
			order[sent++] = i;
		}
		break;
	}

	/* synchronization partition (§5.2 p.15): fixed words */
	put_word(bits, &n, ERMES_PREAMBLE);
	put_word(bits, &n, ERMES_SW1);

	/* system information partition (§5.3): SI, SI, SSI */
	put_word(bits, &n, ermes_bch_encode(ermes_si_word1(net, 0)));
	put_word(bits, &n, ermes_bch_encode(ermes_si_word2(net, t)));
	put_word(bits, &n, ermes_bch_encode(ermes_ssi_word(ssi)));

	/* address partition (§5.4 p.18): initial addresses, highest first, then APTs */
	for (i = 0; i < sent; i++) {
		for (j = i + 1; j < sent; j++) {
			if (page_ia(&pages[order[j]]) > page_ia(&pages[order[i]])) {
				k = order[i]; order[i] = order[j]; order[j] = k;
			}
		}
	}
	for (i = 0; i < sent; i++)
		put_word(bits, &n, ermes_bch_encode(page_ia(&pages[order[i]])));
	for (i = 0; i < ermes_apt_count(sent); i++)
		put_word(bits, &n, ERMES_APT);

	/* message partition (§5.5, §7): fill every codeblock, idle ones with delimiters (§5.5.6-7) */
	num_cb = ermes_max_codeblocks(t->slot, sent);
	for (; nmsg < num_cb * ERMES_CODEBLOCK_CW; nmsg++)
		msg[nmsg] = ERMES_MD;
	for (k = 0; k < num_cb; k++) {
		ermes_interleave(&msg[k * ERMES_CODEBLOCK_CW], bits + n);
		n += ERMES_CODEBLOCK_CW * ERMES_CW_BITS;
	}

	if (n != ermes_batch_length(t->slot) * ERMES_CW_BITS)
		return -1;
	return sent;
}

/*
 * Readable names and descriptions
 */

/* Table 9 p.19 */
static const char *const category_names[] = {
	[ERMES_CAT_TONE]	= "tone",
	[ERMES_CAT_NUMERIC]	= "numeric",
	[ERMES_CAT_ALPHA]	= "alpha",
	[ERMES_CAT_TRANSPARENT]	= "transparent",
};

#define NUM_CATEGORIES	((int)(sizeof(category_names) / sizeof(category_names[0])))

const char *ermes_category_name(enum ermes_category category)
{
	if ((int)category < 0 || (int)category >= NUM_CATEGORIES)
		return "?";
	return category_names[category];
}

/* the full name, or its first letter */
int ermes_category_from_name(const char *name, enum ermes_category *category)
{
	int i;

	for (i = 0; i < NUM_CATEGORIES; i++) {
		if (!strcasecmp(name, category_names[i])
		 || (name[0] && !name[1] && tolower((unsigned char)name[0]) == category_names[i][0])) {
			*category = i;
			return 0;
		}
	}
	return -1;
}

char ermes_batch_name(int batch)
{
	if (batch < 0 || batch >= ERMES_BATCHES)
		return '?';
	return 'A' + batch;
}

#define NEC_PREFIX	"nec:"

int ermes_parse_address(const char *s, uint32_t *local_address)
{
	unsigned long v;
	char *end;
	int nec = !strncasecmp(s, NEC_PREFIX, strlen(NEC_PREFIX));

	if (nec)
		s += strlen(NEC_PREFIX);
	if (!*s)
		return -1;
	v = strtoul(s, &end, nec ? 16 : 0);
	if (*end)
		return -1;
	if (nec) {
		if (v & ((1u << ERMES_NEC_SHIFT) - 1))
			return -1;
		v >>= ERMES_NEC_SHIFT;
	}
	if (v >> ERMES_LA_BITS)
		return -1;
	*local_address = v;
	return 0;
}

const char *ermes_print_address(uint32_t local_address)
{
	static char buf[64];

	snprintf(buf, sizeof(buf), "0x%06X (NEC %06X), batch %c", local_address,
		 local_address << ERMES_NEC_SHIFT, ermes_batch_name(ermes_address_batch(local_address)));
	return buf;
}

const char *ermes_network_str(const ermes_network_t *net)
{
	static char buf[160];

	snprintf(buf, sizeof(buf), "zone %d, country %d, operator %d (%d-%d-%d), paging area %d%s, FSI %d%s",
		 net->zone, net->country, net->operator, net->zone, net->country, net->operator,
		 net->pa, net->bai ? ", border area" : "",
		 net->fsi, (net->fsi == ERMES_FSI_ALL) ? " (single channel network)" : "");
	return buf;
}

/* Table 6 p.17: day 1 = Monday, month 1 = January */
static const char *const weekday_names[] = { "?", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday" };
static const char *const month_names[] = { "?", "January", "February", "March", "April", "May", "June", "July",
					   "August", "September", "October", "November", "December" };

#define FIELD(v, shift, bits)	(((v) >> (shift)) & ((1u << (bits)) - 1))

const char *ermes_ssi_str(const ermes_ssi_t *ssi)
{
	static char buf[128];
	uint32_t f = ssi->ssif;
	int a, b;

	switch (ssi->ssit) {
	case ERMES_SSIT_ZONE_HOUR_DATE:
		snprintf(buf, sizeof(buf), "zone %d, local time %d h, day %d of the month",
			 FIELD(f, ERMES_HOUR_BITS + ERMES_DATE_BITS + FLAG_BITS, ERMES_ZONE_BITS),
			 FIELD(f, ERMES_DATE_BITS + FLAG_BITS, ERMES_HOUR_BITS),
			 FIELD(f, FLAG_BITS, ERMES_DATE_BITS));
		break;
	case ERMES_SSIT_DAY_MONTH_YEAR:
		a = FIELD(f, ERMES_MONTH_BITS + ERMES_YEAR_BITS, ERMES_WEEKDAY_BITS);
		b = FIELD(f, ERMES_YEAR_BITS, ERMES_MONTH_BITS);
		snprintf(buf, sizeof(buf), "%s, %s %d",
			 weekday_names[a], (b < (int)(sizeof(month_names) / sizeof(month_names[0]))) ? month_names[b] : "?",
			 ERMES_YEAR_BASE + FIELD(f, 0, ERMES_YEAR_BITS));
		break;
	case ERMES_SSIT_IVR1:
		snprintf(buf, sizeof(buf), "network I1 version %02d.%02d",
			 FIELD(f, ERMES_VERSION_BITS, ERMES_VERSION_BITS), FIELD(f, 0, ERMES_VERSION_BITS));
		break;
	default:
		snprintf(buf, sizeof(buf), "type %d (reserved), field 0x%04X", ssi->ssit, f);
	}
	return buf;
}

const char *ermes_page_str(const ermes_page_t *page)
{
	static char buf[ERMES_MAX_TEXT_STR];
	const char *text = page->text ? page->text : "";

	char number[32];

	if (page->number < 0)
		snprintf(number, sizeof(number), "numbered when sent");
	else if (page->number)
		snprintf(number, sizeof(number), "message number %d", page->number);
	else
		snprintf(number, sizeof(number), "no message number");
	snprintf(buf, sizeof(buf), "%s, %s%s%s%s, %s, alert type %d", number,
		 ermes_category_name(page->category), text[0] ? " \"" : "", text, text[0] ? "\"" : "",
		 page->urgent ? "urgent" : "not urgent", page->alert);
	return buf;
}
