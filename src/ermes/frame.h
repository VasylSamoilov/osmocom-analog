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

#ifndef ERMES_FRAME_H
#define ERMES_FRAME_H

#include <stdint.h>
#include "code.h"
#include "../libpagercharset/pager_charset.h"

/* §5.5.1.1 Table 9 p.19 */
enum ermes_category {
	ERMES_CAT_TONE		= 0,
	ERMES_CAT_NUMERIC	= 1,
	ERMES_CAT_ALPHA		= 2,
	ERMES_CAT_TRANSPARENT	= 3,
};

/* Network identity and the system information that does not change per batch (§5.3.1 p.15) */
typedef struct ermes_network {
	int	zone;		/* 3 bit, sent in SSIT 0000 */
	int	country;	/* 7 bit */
	int	operator;	/* 3 bit */
	int	pa;		/* 6 bit paging area */
	int	bai;		/* border area indicator */
	int	fsi;		/* 5 bit, 30 = single channel network (§9.2 p.29) */
} ermes_network_t;

/* Where in time a batch is (§5.3.1.2 p.16) */
typedef struct ermes_batch_time {
	int	cycle;		/* 0..59 = minute of the hour */
	int	ssn;		/* 0..4 */
	int	slot;		/* 0..15 time slot in the subsequence */
	int	batch;		/* 0..15 batch type A..P sent in this slot (channel dependent) */
} ermes_batch_time_t;

/* A message to send: one individual call, no additional information (AII = 0) */
typedef struct ermes_page {
	uint32_t		local_address;	/* 22 bit; 4 LSBs = batch type */
	int			number;		/* message number 1..31, 0 = numbering off; -1 = not given
						 * yet (a queued page: ermes_page_str only) */
	enum ermes_category	category;
	int			urgent;		/* UMI */
	int			alert;		/* 0..7 */
	const char		*text;		/* UTF-8 */
	const struct pager_charset *charset;	/* alphanumeric codepage, NULL = ERMES standard (Table B.3) */
} ermes_page_t;

/* Supplementary system information for this batch (§5.3.2 p.16-18) */
typedef struct ermes_ssi {
	int		ssit;		/* 4 bit type */
	uint32_t	ssif;		/* 14 bit field */
} ermes_ssi_t;

/* SSIF contents (Fig.7-9 p.17) */
uint32_t ermes_ssif_zone_hour_date(int zone, int hour, int date);
uint32_t ermes_ssif_day_month_year(int weekday, int month, int year);
uint32_t ermes_ssif_version(int mvr1, int svr1);

/* Information words */
uint32_t ermes_si_word1(const ermes_network_t *net, int eti);
uint32_t ermes_si_word2(const ermes_network_t *net, const ermes_batch_time_t *t);
uint32_t ermes_ssi_word(const ermes_ssi_t *ssi);

/* Message header (§5.5.1 Fig.10-11 p.18-19): two 18-bit information words */
void ermes_header_words(const ermes_page_t *page, uint32_t info[2]);

/* Numeric paging (ETS 300 133-2 Table B.1): code 0..15 of a character ("0123456789/ U-.%", u = U),
 * -1 if it has none; and the character of a code */
int ermes_numeric_code(char c);
char ermes_numeric_char(int code);

/* Message data as 18-bit information words after the header, with termination and fill
 * (§5.5.4 p.22-23). Returns the number of words, -1 on a character that can't be coded. */
int ermes_data_words(const ermes_page_t *page, uint32_t *info, int max_words);


/* Readable names and descriptions, for the logs of the transmitter and the receiver.
 * The _str functions return a static buffer (valid until the next call of the same function). */
const char *ermes_category_name(enum ermes_category category);
int ermes_category_from_name(const char *name, enum ermes_category *category);	/* "tone", "t", ... */
char ermes_batch_name(int batch);						/* 0..15 -> 'A'..'P' */

/* Address as the user gives it: "3247680" or "0x318E40" (22-bit local address), or
 * "nec-C63900" (NEC "Tokyo hex" of the pager label = local address * 4). 0 or -1. */
int ermes_parse_address(const char *s, uint32_t *local_address);
/* "0x318E40 (NEC C63900), batch A" */
const char *ermes_print_address(uint32_t local_address);

/* "zone 2, country 1, operator 2 (2-1-2), paging area 0, FSI 30 (single channel network)" */
const char *ermes_network_str(const ermes_network_t *net);
/* "zone 2, local time 21 h, day 26 of the month" / "Thursday, November 1990+36 = 2026" ... */
const char *ermes_ssi_str(const ermes_ssi_t *ssi);
/* "message number 3, alpha "Hello", not urgent, alert type 0" */
const char *ermes_page_str(const ermes_page_t *page);

/* Message data words of one message at most: 1023 alphanumeric characters (7 bits) and EOM */
#define ERMES_MAX_MSG_WORDS	400

/* The rest of a message that did not fit in its batch: its codewords, the closing message
 * delimiter included, sent first in the next batches of the channel, immediately after their
 * address partition and without a delimiter before (§10.5-10.6 p.34, ed.1 §11.5-11.6 p.31) */
typedef struct ermes_carry {
	uint32_t	cw[ERMES_MAX_MSG_WORDS + 3];
	int		count;
} ermes_carry_t;

/* Build one batch. pages[] must all belong to this batch type (the initial address is only
 * sent in its own batch, §4.1 p.10). The message partition starts with the carry of the batch
 * before (if any); then the messages of the pages, each whole, and the last one may run on into
 * the next batches through the carry. With carry NULL every message must fit whole.
 * bits[] gets ermes_batch_length(t->slot) * 30 bits, one per byte, in air order.
 * Returns the number of pages sent (the first ones that fit), or -1 on error. */
int ermes_build_batch(const ermes_network_t *net, const ermes_batch_time_t *t, const ermes_ssi_t *ssi,
		      const ermes_page_t *pages, int num_pages, ermes_carry_t *carry, uint8_t *bits);

#endif /* ERMES_FRAME_H */
