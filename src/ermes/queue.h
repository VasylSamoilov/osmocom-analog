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
 *
 * The sources (FIFO, --page, call control) put messages in; the scheduler takes them out batch
 * by batch (ETS 300 133-3 §13.5.2: input control enters the messages into queues).
 */

#ifndef ERMES_QUEUE_H
#define ERMES_QUEUE_H

#include "frame.h"

#define ERMES_MAX_TEXT	1024	/* bytes of a message text (UTF-8), terminator included */

/* How a page is sent: paging category, UMI and alert type (VIF, Fig.11 / Table 9 p.19) */
typedef struct ermes_page_opts {
	enum ermes_category	category;
	int			urgent;		/* UMI */
	int			alert;		/* 0..7 */
	const struct pager_charset *charset;	/* alphanumeric codepage, NULL = --charset */
} ermes_page_opts_t;

/* defaults of the sources (command line) */
extern ermes_page_opts_t ermes_default_opts;
extern const struct pager_charset *ermes_charset;

typedef struct ermes_msg {
	struct ermes_msg	*next;
	ermes_page_t		page;
	char			text[ERMES_MAX_TEXT];
} ermes_msg_t;

typedef struct ermes_queue {
	ermes_msg_t		*head;		/* in the order they came */
} ermes_queue_t;

/* Options field of a FIFO line (as flex): "urgent=1 alert=3 charset=..." onto opts. 0 or -1 (logged) */
int ermes_parse_page_opts(const char *options, ermes_page_opts_t *opts);

/* Check a page and put it at the end of the queue. 0, or -1 (not queued, logged) */
int ermes_queue_page(ermes_queue_t *queue, uint32_t local_address, const ermes_page_opts_t *opts, const char *text);

/* Take a message off the queue and free it */
void ermes_queue_remove(ermes_queue_t *queue, ermes_msg_t *msg);

/* Free all messages */
void ermes_queue_flush(ermes_queue_t *queue);

#endif /* ERMES_QUEUE_H */
