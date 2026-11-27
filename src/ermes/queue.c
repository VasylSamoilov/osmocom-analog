/* ERMES message queue
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
#include "queue.h"
#include "numbers.h"

ermes_page_opts_t ermes_default_opts = { .category = ERMES_CAT_TONE, .urgent = 0, .alert = 0 };
const struct pager_charset *ermes_charset = &pager_charset_ermes_generic_generic_standard;

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

int ermes_queue_page(ermes_queue_t *queue, uint32_t local_address, const ermes_page_opts_t *opts, const char *text)
{
	enum ermes_category category = opts->category;
	ermes_msg_t *m, **mp;
	uint32_t data[ERMES_MAX_MSG_WORDS];
	int nd;

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
	nd = ermes_data_words(&m->page, data, ERMES_MAX_MSG_WORDS);
	if (nd < 0) {
		LOGP(DERMES, LOGL_ERROR, "Message \"%s\" can't be sent as %s: a character without a code.\n",
		     text, ermes_category_name(category));
		free(m);
		return -1;
	}
	/* a message runs on into the next batches when it does not fit in its own (§10.5-10.6) */
	if (nd > ERMES_MAX_MSG_WORDS) {
		LOGP(DERMES, LOGL_ERROR, "Message of %d data words is longer than %d.\n", nd, ERMES_MAX_MSG_WORDS);
		free(m);
		return -1;
	}
	m->page.number = ermes_number_next(local_address);
	for (mp = &queue->head; *mp; mp = &(*mp)->next)
		;
	*mp = m;
	LOGP(DERMES, LOGL_NOTICE, "Page queued for %s: %s\n", ermes_print_address(local_address), ermes_page_str(&m->page));
	return 0;
}

void ermes_queue_remove(ermes_queue_t *queue, ermes_msg_t *msg)
{
	ermes_msg_t **mp;

	for (mp = &queue->head; *mp; mp = &(*mp)->next) {
		if (*mp == msg) {
			*mp = msg->next;
			break;
		}
	}
	free(msg);
}

void ermes_queue_flush(ermes_queue_t *queue)
{
	ermes_msg_t *m;

	while ((m = queue->head)) {
		queue->head = m->next;
		free(m);
	}
}
