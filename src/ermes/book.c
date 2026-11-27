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

static ermes_subscriber_t *add(uint32_t local_address, int opid)
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
	book[book_count].opid = opid;
	book[book_count].fsn = -1;
	book[book_count].hnl = -1;
	book[book_count].enl = -1;
	return &book[book_count++];
}

const char *ermes_book_name(uint32_t local_address, int opid)
{
	static char buf[48];

	if (opid < 0)
		snprintf(buf, sizeof(buf), "0x%06X", local_address);
	else
		snprintf(buf, sizeof(buf), "0x%06X opid=%s", local_address, ermes_opid_str(opid));
	return buf;
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
	/* a visiting receiver's line: part of the key */
	if (!strcasecmp(key, "opid"))
		return ermes_parse_opid(val, &s->opid);
	if (!strcasecmp(key, "hnl") || !strcasecmp(key, "enl")) {
		v = strtol(val, &end, 10);
		if (end == val || *end || v < 0 || v > 5)
			return -1;
		if (!strcasecmp(key, "hnl"))
			s->hnl = v;
		else
			s->enl = v;
		return 0;
	}
	return -1;
}

int ermes_book_active_cycle(uint32_t local_address, int opid, int cycle)
{
	ermes_subscriber_t *s = ermes_book_find(local_address, opid, 0);
	int level, mask;

	/* ours: HNL, default 0; a visitor: ENL, default 2 (Table 19) */
	if (opid < 0)
		level = (s && s->hnl >= 0) ? s->hnl : 0;
	else
		level = (s && s->enl >= 0) ? s->enl : 2;
	mask = (1 << level) - 1;
	return (cycle & mask) == (int)((local_address >> ERMES_BATCH_BITS) & mask);
}

/* "number=5 fsn=8" for one receiver while running (FIFO type "book"); none: just log the entry.
 * All or nothing; saved. 0 or -1 */
int ermes_book_set(uint32_t local_address, const char *settings)
{
	ermes_subscriber_t *s, copy;
	char buf[256], *tok, *save, *eq;

	if (strlen(settings) >= sizeof(buf)) {
		LOGP(DERMES, LOGL_ERROR, "Book settings too long.\n");
		return -1;
	}
	strcpy(buf, settings);
	/* the line is chosen by opid= (a visiting receiver) or is ours */
	memset(&copy, 0, sizeof(copy));
	copy.opid = -1;
	for (tok = strtok_r(buf, " \t\r\n", &save); tok; tok = strtok_r(NULL, " \t\r\n", &save)) {
		if (strncasecmp(tok, "opid=", 5))
			continue;
		if (ermes_parse_opid(tok + 5, &copy.opid) < 0) {
			LOGP(DERMES, LOGL_ERROR, "Bad book setting %s (zone-country-operator).\n", tok);
			return -1;
		}
	}
	strcpy(buf, settings);
	/* checked on a copy: a refused line adds or changes nothing */
	s = ermes_book_find(local_address, copy.opid, 0);
	if (s)
		copy = *s;
	else {
		copy.local_address = local_address;
		copy.fsn = -1;
		copy.hnl = copy.enl = -1;
	}
	for (tok = strtok_r(buf, " \t\r\n", &save); tok; tok = strtok_r(NULL, " \t\r\n", &save)) {
		eq = strchr(tok, '=');
		if (!eq) {
			LOGP(DERMES, LOGL_ERROR, "Book setting \"%s\" is not key=value (number, fsn, hnl, enl, opid).\n", tok);
			return -1;
		}
		*eq = '\0';
		if (set(&copy, tok, eq + 1) < 0) {
			LOGP(DERMES, LOGL_ERROR, "Bad book setting %s=%s (number 0..31, fsn 0..15, hnl/enl 0..5).\n", tok, eq + 1);
			return -1;
		}
	}
	if (!s)
		s = ermes_book_find(local_address, copy.opid, 1);
	if (!s)
		return -1;
	*s = copy;
	ermes_book_save();
	if (s->fsn >= 0)
		LOGP(DERMES, LOGL_NOTICE, "Book: %s number=%d fsn=%d\n", ermes_book_name(s->local_address, s->opid), s->number, s->fsn);
	else
		LOGP(DERMES, LOGL_NOTICE, "Book: %s number=%d (fsn not known)\n", ermes_book_name(s->local_address, s->opid), s->number);
	return 0;
}

/* one line of the book, or of an old numbers file ("<address hex> <number>") */
static void parse_line(char *line, const char *file, int lineno)
{
	char *p, *tok, *val, *save = NULL;
	ermes_subscriber_t *s, copy;
	uint32_t la;

	p = strchr(line, '#');
	if (p)
		*p = '\0';
	tok = strtok_r(line, " \t\r\n", &save);
	if (!tok)
		return;
	if (ermes_parse_address(tok, &la) < 0) {
		LOGP(DERMES, LOGL_ERROR, "%s:%d: \"%s\" is not a local address, line ignored.\n", file, lineno, tok);
		return;
	}
	/* the whole line first: opid= is part of the key */
	memset(&copy, 0, sizeof(copy));
	copy.opid = -1;
	copy.fsn = -1;
	copy.hnl = copy.enl = -1;
	while ((tok = strtok_r(NULL, " \t\r\n", &save))) {
		val = strchr(tok, '=');
		if (!val) {
			/* old numbers file: the bare number */
			if (set(&copy, "number", tok) < 0)
				LOGP(DERMES, LOGL_ERROR, "%s:%d: \"%s\" is not key=value, ignored.\n", file, lineno, tok);
			continue;
		}
		*val++ = '\0';
		if (set(&copy, tok, val) < 0)
			LOGP(DERMES, LOGL_ERROR, "%s:%d: \"%s=%s\" is not known or out of range, ignored.\n", file, lineno, tok, val);
	}
	s = ermes_book_find(la, copy.opid, 1);
	if (!s)
		return;
	copy.local_address = la;
	*s = copy;
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

ermes_subscriber_t *ermes_book_find(uint32_t local_address, int opid, int create)
{
	int i;

	load();
	for (i = 0; i < book_count; i++) {
		if (book[i].local_address == local_address && book[i].opid == opid)
			return &book[i];
	}
	return create ? add(local_address, opid) : NULL;
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
	fprintf(fp, "# missing: not known), opid = home operator of a visiting receiver (missing: ours).\n");
	fprintf(fp, "# Rewritten by the program.\n");
	for (i = 0; i < book_count; i++) {
		fprintf(fp, "%s number=%d", ermes_book_name(book[i].local_address, book[i].opid), book[i].number);
		if (book[i].fsn >= 0)
			fprintf(fp, " fsn=%d", book[i].fsn);
		if (book[i].hnl >= 0)
			fprintf(fp, " hnl=%d", book[i].hnl);
		if (book[i].enl >= 0)
			fprintf(fp, " enl=%d", book[i].enl);
		fprintf(fp, "\n");
	}
	fclose(fp);
	return 0;
}
