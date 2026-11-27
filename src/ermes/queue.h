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

/* Priority levels (ETS 300 133-2 §4.3.2 p.21-23, §6.2.6 p.42): 1 goes on the air before lower
 * priorities (delay at most 1 min), 2 is the normal level (2.5 min), 3 is for traffic that is not
 * time sensitive and must not delay 1 and 2 (24 h). Only the network's queue order: no bit on
 * the air (the UMI, urgent, is a separate thing, -4 Table 9 p.19). */
#define ERMES_PRIORITY_HIGH	1
#define ERMES_PRIORITY_NORMAL	2
#define ERMES_PRIORITY_LOW	3

/* How a page is sent: paging category, UMI and alert type (VIF, Fig.11 / Table 9 p.19), and its
 * priority in the queue */
typedef struct ermes_page_opts {
	enum ermes_category	category;
	int			urgent;		/* UMI */
	int			alert;		/* 0..7 */
	const struct pager_charset *charset;	/* alphanumeric codepage, NULL = --charset */
	int			priority;	/* ERMES_PRIORITY_* */
	int			retransmit;	/* transmissions after the first, same number */
	int			retransmit_interval; /* seconds between them */
	int			last_number;	/* type "lastnum": the latest message number again */
	int			external;	/* opid=: a visiting receiver (EB = 1) ... */
	int			opid;		/* ... of this home operator (13 bits) */
} ermes_page_opts_t;

/* Repetition (ETS 300 133-2 §4.3.1.3.1 p.18): the message is sent again with its number, within
 * an interval set by the operator, 5 minutes by default */
#define ERMES_RETRANSMIT_MAX		15
#define ERMES_RETRANSMIT_INTERVAL_S	300
#define ERMES_RETRANSMIT_INTERVAL_MAX_S	3600

/* "priority 2 (normal)" */
const char *ermes_priority_name(int priority);

/* defaults of the sources (command line) */
extern ermes_page_opts_t ermes_default_opts;
extern const struct pager_charset *ermes_charset;

struct ermes;

/* A group call (§10.3): one message to several receivers through a common temporary address.
 * Each member is served by one channel (the one carrying its FSN), one stage per batch, one after
 * another: its initial address alone in its own batch (stage 1; it then searches the subsequent
 * batches), its pointer in the next batch (stage 2; with --group-fig16 with stage 1), and after
 * the last member's pointer, in the next batch, the message once to the CTA (stage 3). At most 32
 * members: a limit of this program, not of the standard. */
#define ERMES_GROUP_MAX_MEMBERS	32
typedef struct ermes_group {
	int			num;
	uint32_t		member[ERMES_GROUP_MAX_MEMBERS];	/* local addresses */
	struct ermes		*channel[ERMES_GROUP_MAX_MEMBERS];	/* serving it, NULL = not yet */
	uint64_t		ia_at[ERMES_GROUP_MAX_MEMBERS];		/* stage 1 sent in the batch at
								 * this codeword count + 1, 0 = not */
	int			done[ERMES_GROUP_MAX_MEMBERS];		/* stage 2 sent */
	int			active;		/* channels sending their part now */
} ermes_group_t;

/* The batch types of a group's members must leave room for its stages within the 16 batches of
 * the members' search (§10.8.2): the initial addresses within 14 batches in a row. 0, or -1 if
 * they are spread over more (logged). */
int ermes_group_fits(const uint32_t *members, int num);

typedef struct ermes_msg {
	struct ermes_msg	*next;
	ermes_group_t		*group;		/* a group call, NULL = an individual page */
	ermes_page_t		page;		/* number -1 until the scheduler sends it first */
	char			text[ERMES_MAX_TEXT];
	int			priority;	/* ERMES_PRIORITY_* */
	double			queued;		/* when it came (s since the epoch, system clock) */
	int			retransmit;	/* transmissions still to come after this one */
	int			retransmit_interval; /* seconds */
	int			sent;		/* transmissions so far */
	double			first_time;	/* network time of the first transmission */
	double			next_time;	/* network time of the next retransmission */
} ermes_msg_t;

typedef struct ermes_queue {
	ermes_msg_t		*head;		/* in the order they came */
} ermes_queue_t;

/* The network's queue: one for all channels; each channel takes the pages it may carry */
extern ermes_queue_t ermes_queue;

/* Options field of a FIFO line (as flex): "priority=1 urgent=1 alert=3 charset=... retransmit=2
 * retransmit_interval=300" onto opts. 0 or -1 (logged) */
int ermes_parse_page_opts(const char *options, ermes_page_opts_t *opts);

/* Check a page and put it at the end of the queue. 0, or -1 (not queued, logged) */
int ermes_queue_page(ermes_queue_t *queue, uint32_t local_address, const ermes_page_opts_t *opts, const char *text);

/* ", sent 2 more times 300 s apart" or "" */
const char *ermes_retransmit_str(const ermes_msg_t *msg);

/* Check a group call to several receivers (flex style "tempgroup:<address> <address> ...") and
 * put it at the end of the queue. 0, or -1 (not queued, logged) */
int ermes_queue_group(ermes_queue_t *queue, const uint32_t *members, int num, const ermes_page_opts_t *opts,
		      const char *text);

/* Take a message off the queue and free it */
void ermes_queue_remove(ermes_queue_t *queue, ermes_msg_t *msg);

/* Free all messages */
void ermes_queue_flush(ermes_queue_t *queue);

#endif /* ERMES_QUEUE_H */
