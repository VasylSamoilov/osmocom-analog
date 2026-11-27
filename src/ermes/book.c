/* ERMES subscriber book
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
#include <strings.h>
#include "../liblogging/logging.h"
#include "frame.h"
#include "book.h"

const char *ermes_book_file = NULL;
const char *ermes_book_old_numbers = NULL;

#define LINE_MAX_LEN	512

static ermes_subscriber_t *book;
static int book_count, book_size;
static int loaded;

static ermes_subscriber_t *add(uint32_t local_address)
{
	if (book_count == book_size) {
		int size = book_size ? book_size * 2 : 64;
		ermes_subscriber_t *b = realloc(book, size * sizeof(*b));

		if (!b) {
			LOGP(DERMES, LOGL_ERROR, "No memory for the subscriber book.\n");
			return NULL;
		}
		book = b;
		book_size = size;
	}
	memset(&book[book_count], 0, sizeof(book[0]));
	book[book_count].local_address = local_address;
	book[book_count].fsn = -1;
	return &book[book_count++];
}

/* "number=5": one setting of an entry. 0 or -1 */
static int set(ermes_subscriber_t *s, const char *key, const char *val)
{
	char *end;
	long v;

	if (!strcasecmp(key, "number")) {
		v = strtol(val, &end, 10);
		if (end == val || *end || v < 0 || v > (1 << ERMES_MSG_NUMBER_BITS) - 1)
			return -1;
		s->number = v;
		return 0;
	}
	if (!strcasecmp(key, "fsn")) {
		v = strtol(val, &end, 10);
		if (end == val || *end || v < 0 || v >= ERMES_FSN_COUNT)
			return -1;
		s->fsn = v;
		return 0;
	}
	return -1;
}

/* one line of the book, or of an old numbers file ("<address hex> <number>") */
static void parse_line(char *line, const char *file, int lineno)
{
	char *p, *tok, *val;
	ermes_subscriber_t *s;
	uint32_t la;

	p = strchr(line, '#');
	if (p)
		*p = '\0';
	tok = strtok(line, " \t\r\n");
	if (!tok)
		return;
	if (ermes_parse_address(tok, &la) < 0) {
		LOGP(DERMES, LOGL_ERROR, "%s:%d: \"%s\" is not a local address, line ignored.\n", file, lineno, tok);
		return;
	}
	s = ermes_book_find(la, 1);
	if (!s)
		return;
	while ((tok = strtok(NULL, " \t\r\n"))) {
		val = strchr(tok, '=');
		if (!val) {
			/* old numbers file: the bare number */
			if (set(s, "number", tok) < 0)
				LOGP(DERMES, LOGL_ERROR, "%s:%d: \"%s\" is not key=value, ignored.\n", file, lineno, tok);
			continue;
		}
		*val++ = '\0';
		if (set(s, tok, val) < 0)
			LOGP(DERMES, LOGL_ERROR, "%s:%d: \"%s=%s\" is not known or out of range, ignored.\n", file, lineno, tok, val);
	}
}

static int load_file(const char *file)
{
	char line[LINE_MAX_LEN];
	int lineno = 0;
	FILE *fp;

	fp = fopen(file, "r");
	if (!fp)
		return -1;
	while (fgets(line, sizeof(line), fp))
		parse_line(line, file, ++lineno);
	fclose(fp);
	return 0;
}

static void load(void)
{
	if (loaded)
		return;
	loaded = 1;
	if (!ermes_book_file)
		return;
	if (load_file(ermes_book_file) == 0) {
		LOGP(DERMES, LOGL_INFO, "Subscriber book '%s': %d receiver(s).\n", ermes_book_file, book_count);
		return;
	}
	/* no book yet: take the message numbers of an earlier version over */
	if (ermes_book_old_numbers && load_file(ermes_book_old_numbers) == 0) {
		LOGP(DERMES, LOGL_NOTICE, "Message numbers of %d receiver(s) taken over from '%s' into the subscriber book '%s'.\n",
		     book_count, ermes_book_old_numbers, ermes_book_file);
		ermes_book_save();
	}
}

ermes_subscriber_t *ermes_book_find(uint32_t local_address, int create)
{
	int i;

	load();
	for (i = 0; i < book_count; i++) {
		if (book[i].local_address == local_address)
			return &book[i];
	}
	return create ? add(local_address) : NULL;
}

int ermes_book_save(void)
{
	FILE *fp;
	int i;

	if (!ermes_book_file)
		return 0;
	fp = fopen(ermes_book_file, "w");
	if (!fp) {
		LOGP(DERMES, LOGL_ERROR, "Cannot write the subscriber book '%s'.\n", ermes_book_file);
		return -1;
	}
	fprintf(fp, "# ERMES subscriber book: one receiver per line, <local address> key=value ...\n");
	fprintf(fp, "# number = last message number sent (1..31), fsn = frequency subset number (0..15,\n");
	fprintf(fp, "# missing: not known). Rewritten by the program.\n");
	for (i = 0; i < book_count; i++) {
		fprintf(fp, "0x%06X number=%d", book[i].local_address, book[i].number);
		if (book[i].fsn >= 0)
			fprintf(fp, " fsn=%d", book[i].fsn);
		fprintf(fp, "\n");
	}
	fclose(fp);
	return 0;
}
