/* ERMES scheduler: batch clock and batch filling
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
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/main_mobile.h"
#include "ermes.h"
#include "ssi.h"
#include "scheduler.h"

#define CHAN ermes->sender.kanal
#define MAX_PAGES	32	/* initial addresses per batch we fill */

double ermes_clock_offset = 0.0;
double ermes_tx_latency = -1.0;	/* < 0: dsp_buffer */
int ermes_receiver_batch = ERMES_RECEIVER_BATCH_DEFAULT;

/* Network time (seconds since the epoch) at the start of the batch being built: the start of
 * the first batch plus the codewords sent since, so it follows the samples, not the wall clock */
static double network_time(const ermes_t *ermes)
{
	return ermes->clock_start + (double)ermes->cw_sent * ERMES_CW_BITS / ERMES_BIT_RATE;
}

/* Lock the batches to UTC (§4.1 p.10, §12): subsequences start every 12 s from the UTC minute,
 * the batches at fixed codeword offsets. Find where the first sample will be on the air in this
 * grid - the batch and the bit in it - so the transmission starts in the middle of that batch.
 * Every start then keeps the same grid, as a real network: a pager that synchronised before keeps
 * waking at the right time (with a grid shifted at every start it wakes too early or too late,
 * misses its batch and is lost until it searches again). */
/* The grid of the network: computed once, by the first channel that starts; every channel takes
 * the same, so all channels send the same time slot at the same sample (they are fed the same
 * number of samples, libmobile) */
static struct {
	int		done;
	int		cycle, ssn, slot, skip_bits;
	double		clock_start;
} grid;

static void start_time(ermes_t *ermes)
{
	struct timeval tv;
	struct tm tm;
	double now, into_ssn, bits;
	time_t minute;
	int slot, skip;

	if (grid.done) {
		ermes->cycle = grid.cycle;
		ermes->ssn = grid.ssn;
		ermes->slot = grid.slot;
		ermes->skip_bits = grid.skip_bits;
		ermes->clock_start = grid.clock_start;
		ermes->cw_sent = 0;
		ermes->started = 1;
		return;
	}
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
	grid.cycle = ermes->cycle;
	grid.ssn = ermes->ssn;
	grid.slot = ermes->slot;
	grid.skip_bits = ermes->skip_bits;
	grid.clock_start = ermes->clock_start;
	grid.done = 1;
	LOGP_CHAN(DERMES, LOGL_INFO, "Transmission starts locked to %s UTC %02d:%02d:%06.3f: minute (cycle) %d, subsequence %d, time slot %d, %d bits into the batch\n",
		  ermes_clock_offset ? "network time (--clock)" : "", tm.tm_hour, tm.tm_min, now - minute,
		  ermes->cycle, ermes->ssn, ermes->slot, skip);
}

/* 1 if this channel may carry pages for the receiver: its FSI has the receiver's FSN; a receiver
 * whose FSN is not known (not in the book) goes on any channel, the first with room */
static int fsn_on_channel(const ermes_t *ermes, uint32_t local_address)
{
	const ermes_subscriber_t *s = ermes_book_find(local_address, 0);

	if (!s || s->fsn < 0)
		return 1;
	return ermes_fsi_has_fsn(ermes->net.fsi, s->fsn);
}

int ermes_scheduler_reachable(uint32_t local_address)
{
	sender_t *s;

	for (s = sender_head; s; s = s->next) {
		if (fsn_on_channel((ermes_t *)s, local_address))
			return 1;
	}
	return 0;
}

static void next_slot(ermes_t *ermes)
{
	if (++ermes->slot < ERMES_BATCHES)
		return;
	ermes->slot = 0;
	if (++ermes->ssn < ERMES_SUBSEQUENCES)
		return;
	ermes->ssn = 0;
	ermes->cycle = (ermes->cycle + 1) % ERMES_CYCLES;
}

/*
 * Group calls (§10.3, §10.8.2). A channel sends one group call at a time, for the members it
 * carries: stage 1 and 2 in each member's batch (its initial address, then a header with its own
 * next number and AII 1, AIT 111, AIN = CTAP), and in the batch of the last member the message
 * itself to the common temporary address with number 00000 (stage 3). Every batch type comes once
 * in 16 time slots, so all members and the start of stage 3 are within 12 s of the first member's
 * batch, as the members search no longer (§10.8.2). While a call is on, the channel sends nothing
 * else, and it starts one only with no message running on (the carry), so no batch lacks room.
 */
#define GROUP_SEARCH_S	12.0	/* §10.8.2 */

const char *ermes_ctap_state_name(enum ermes_ctap_state state)
{
	switch (state) {
	case ERMES_CTAP_FREE:		return "free";
	case ERMES_CTAP_ADDRESSING:	return "addressing";
	case ERMES_CTAP_MESSAGE:	return "message";
	case ERMES_CTAP_HOLD:		return "hold";
	}
	return "?";
}

static void ctap_state(ermes_t *ermes, int c, enum ermes_ctap_state state, const char *why)
{
	ermes_ctap_slot_t *s = &ermes->ctap[c];

	LOGP_CHAN(DERMES, LOGL_INFO, "CTAP %d: %s -> %s (%s)\n", c, ermes_ctap_state_name(s->state),
		  ermes_ctap_state_name(state), why);
	s->state = state;
}

/* each batch: a message that ran on has ended, a hold has run out */
static void ctap_update(ermes_t *ermes, double net_now)
{
	int c;

	for (c = 0; c < ERMES_CTAP_COUNT; c++) {
		ermes_ctap_slot_t *s = &ermes->ctap[c];

		if (s->state == ERMES_CTAP_MESSAGE && !ermes->carry.count)
			ctap_state(ermes, c, ERMES_CTAP_HOLD, "the message is sent");
		if (s->state == ERMES_CTAP_HOLD && net_now >= s->free_at)
			ctap_state(ermes, c, ERMES_CTAP_FREE, "the members' search is over");
	}
}

/* take the next group call with members this channel carries, if a CTAP is free */
static void group_start(ermes_t *ermes)
{
	ermes_msg_t *m;
	ermes_group_t *g;
	ermes_ctap_slot_t *s;
	char why[64];
	int prio, i, k, c, count;

	for (prio = ERMES_PRIORITY_HIGH; prio <= ERMES_PRIORITY_LOW; prio++) {
		for (m = ermes_queue.head; m; m = m->next) {
			if (m->priority != prio)
				continue;
			/* in the order they came: an individual page before it for this channel goes first
			 * (a retransmission waiting for its time does not hold the call) */
			if (!m->group) {
				if (!m->sent && fsn_on_channel(ermes, m->page.local_address))
					return;
				continue;
			}
			g = m->group;
			for (count = 0, i = 0; i < g->num; i++)
				count += (!g->channel[i] && !g->done[i] && fsn_on_channel(ermes, g->member[i]));
			if (!count)
				continue;
			for (k = 0; k < ERMES_CTAP_COUNT; k++) {
				c = (ermes->ctap_next + k) % ERMES_CTAP_COUNT;
				if (ermes->ctap[c].state == ERMES_CTAP_FREE)
					break;
			}
			if (k == ERMES_CTAP_COUNT) {
				LOGP_CHAN(DERMES, LOGL_DEBUG, "Group call waits for a free CTAP\n");
				return;
			}
			for (i = 0; i < g->num; i++) {
				if (!g->channel[i] && !g->done[i] && fsn_on_channel(ermes, g->member[i]))
					g->channel[i] = ermes;
			}
			g->active++;
			s = &ermes->ctap[c];
			s->msg = m;
			s->members = s->left = count;
			s->first = s->last = -1.0;
			ermes->group_ctap = c;
			ermes->ctap_next = (c + 1) % ERMES_CTAP_COUNT;
			snprintf(why, sizeof(why), "a group call to %d receiver(s)", count);
			ctap_state(ermes, c, ERMES_CTAP_ADDRESSING, why);
			LOGP_CHAN(DERMES, LOGL_NOTICE, "Group call starts: %d receiver(s) on this channel, the message to %s\n",
				  count, ermes_print_address(ermes_cta(c)));
			return;
		}
	}
}

/* the pages of the group call in this batch: stage 2 of its members of this batch type, and stage
 * 3 when none is left after them. Returns how many. */
static int group_pages(ermes_t *ermes, const ermes_batch_time_t *t, ermes_msg_t **msgs, ermes_page_t *pages)
{
	ermes_ctap_slot_t *s = &ermes->ctap[ermes->group_ctap];
	ermes_msg_t *m = s->msg;
	ermes_group_t *g = m->group;
	int i, n = 0;

	for (i = 0; i < g->num && n < MAX_PAGES - 1; i++) {
		if (g->channel[i] != ermes || g->done[i] || ermes_address_batch(g->member[i]) != t->batch)
			continue;
		memset(&pages[n], 0, sizeof(pages[n]));
		pages[n].local_address = g->member[i];
		pages[n].number = ermes_number_peek(g->member[i], 0);
		pages[n].group_pointer = ermes->group_ctap + 1;
		msgs[n++] = m;
	}
	if (s->left == n) {
		pages[n] = m->page;
		pages[n].local_address = ermes_cta(ermes->group_ctap);
		pages[n].number = 0;
		msgs[n++] = m;
	}
	return n;
}

/* after the batch: the pages of the group call that went out */
static void group_sent(ermes_t *ermes, const ermes_batch_time_t *t, const ermes_page_t *pages, int n, int sent,
		       double net_now)
{
	int c = ermes->group_ctap;
	ermes_ctap_slot_t *s = &ermes->ctap[c];
	ermes_msg_t *m = s->msg;
	ermes_group_t *g = m->group;
	int i, j, all;

	if (sent < n)
		LOGP_CHAN(DERMES, LOGL_ERROR, "Group call: %d of its %d message(s) did not fit in batch %c\n",
			  n - sent, n, ermes_batch_name(t->batch));
	for (i = 0; i < sent; i++) {
		if (pages[i].group_pointer) {
			if (ermes_number_next(pages[i].local_address) != pages[i].number)
				LOGP_CHAN(DERMES, LOGL_ERROR, "Message number %d taken out of order\n", pages[i].number);
			for (j = 0; j < g->num; j++) {
				if (g->member[j] == pages[i].local_address)
					g->done[j] = 1;
			}
			s->left--;
			if (s->first < 0)
				s->first = net_now;
			s->last = net_now;
			LOGP_CHAN(DERMES, LOGL_NOTICE, "Group call to %s: %s; at minute %d, subsequence %d, time slot %d\n",
				  ermes_print_address(pages[i].local_address), ermes_page_str(&pages[i]), t->cycle, t->ssn, t->slot);
			continue;
		}
		/* stage 3 */
		LOGP_CHAN(DERMES, LOGL_NOTICE, "Group call message to %s, %.1f s after the first member's batch: %s; at minute %d, subsequence %d, time slot %d\n",
			  ermes_print_address(pages[i].local_address), net_now - s->first, ermes_page_str(&pages[i]),
			  t->cycle, t->ssn, t->slot);
		if (net_now - s->first >= GROUP_SEARCH_S)
			LOGP_CHAN(DERMES, LOGL_ERROR, "Group call message %.1f s after the first member's batch: its receivers search only %.0f s\n",
				  net_now - s->first, GROUP_SEARCH_S);
		s->free_at = s->last + GROUP_SEARCH_S;
		s->msg = NULL;
		ermes->group_ctap = -1;
		if (ermes->carry.count)
			ctap_state(ermes, c, ERMES_CTAP_MESSAGE, "the message runs on into the next batches");
		else
			ctap_state(ermes, c, ERMES_CTAP_HOLD, "the message is sent");
		g->active--;
		for (all = 1, j = 0; j < g->num; j++)
			all &= g->done[j];
		if (all && !g->active)
			ermes_queue_remove(&ermes_queue, m);
	}
}

int ermes_scheduler_next_batch(ermes_t *ermes)
{
	ermes_batch_time_t t;
	ermes_ssi_t ssi;
	ermes_page_t pages[MAX_PAGES];
	ermes_msg_t *msgs[MAX_PAGES], *m;
	struct timeval tv;
	double now, net_now;
	int n = 0, sent, i, carried, prio, ahead, same, group;

	if (!ermes->started)
		start_time(ermes);

	t.cycle = ermes->cycle;
	t.ssn = ermes->ssn;
	t.slot = ermes->slot;
	t.batch = ermes_channel_batch(ermes->channel, ermes->slot);
	ermes_ssi_select(&ermes->net, &t, (time_t)network_time(ermes), &ermes->ssit_next, &ssi);

	/* the pages of this batch type, highest priority first, then in the order they came (head
	 * of the line in each priority class, ETS 300 133-3 §13.5.5.2 p.174). Each gets the next
	 * number of its address in this order (several of one address follow on); the numbers are
	 * taken only for the pages that are sent. */
	/* times of transmissions are network time, from the samples: it runs at the speed of the
	 * output, faster than the wall clock when only a wave file is written. The arrival of a page
	 * is wall clock (the source does not know the network time). */
	gettimeofday(&tv, NULL);
	now = tv.tv_sec + tv.tv_usec / 1e6;
	net_now = network_time(ermes);
	ctap_update(ermes, net_now);
	if (ermes->group_ctap < 0 && !ermes->carry.count)
		group_start(ermes);
	group = (ermes->group_ctap >= 0);
	if (group)
		n = group_pages(ermes, &t, msgs, pages);
	for (prio = ERMES_PRIORITY_HIGH; prio <= ERMES_PRIORITY_LOW && !group; prio++) {
		for (m = ermes_queue.head; m && n < MAX_PAGES; m = m->next) {
			if (m->group || m->priority != prio || ermes_address_batch(m->page.local_address) != t.batch)
				continue;
			/* only on a channel whose frequency subset has the receiver's FSN (§9.2) */
			if (!fsn_on_channel(ermes, m->page.local_address))
				continue;
			/* a retransmission waits for its time */
			if (m->sent && m->next_time > net_now)
				continue;
			/* at most --receiver-batch messages to one receiver in a batch; the others wait
			 * for its next batch, in order (the standard allows any number, §10.2) */
			for (same = 0, ahead = 0, i = 0; i < n; i++) {
				if (pages[i].local_address != m->page.local_address)
					continue;
				same++;
				ahead += (msgs[i]->page.number < 0);
			}
			if (same >= ermes_receiver_batch)
				continue;
			msgs[n] = m;
			pages[n] = m->page;
			/* a retransmission keeps the number of its first transmission */
			if (m->page.number < 0)
				pages[n].number = ermes_number_peek(m->page.local_address, ahead);
			n++;
		}
	}

	carried = ermes->carry.count;
	sent = ermes_build_batch(&ermes->net, &t, &ssi, pages, n, &ermes->carry, ermes->batch_bits);
	if (sent < 0) {
		LOGP_CHAN(DERMES, LOGL_ERROR, "Failed to build batch %c, sending it idle.\n", ermes_batch_name(t.batch));
		ermes->carry.count = 0;
		sent = ermes_build_batch(&ermes->net, &t, &ssi, NULL, 0, NULL, ermes->batch_bits);
		n = 0;
	}
	if (carried)
		LOGP_CHAN(DERMES, LOGL_INFO, "Batch %c continues a message: %d codewords%s\n", ermes_batch_name(t.batch),
			  carried - ermes->carry.count, ermes->carry.count ? ", more in the next batch" : ", its end");
	else if (ermes->carry.count)
		LOGP_CHAN(DERMES, LOGL_INFO, "The last message of batch %c runs on into the next batch (%d codewords)\n",
			  ermes_batch_name(t.batch), ermes->carry.count);

	if (group) {
		group_sent(ermes, &t, pages, n, sent, net_now);
		sent = 0;
	}
	for (i = 0; i < sent; i++) {
		char when[64];

		m = msgs[i];
		/* the batch builder sends the first pages given, in this order */
		if (m->page.number < 0) {
			if (ermes_number_next(m->page.local_address) != pages[i].number)
				LOGP_CHAN(DERMES, LOGL_ERROR, "Message number %d taken out of order\n", pages[i].number);
			m->page.number = pages[i].number;
		}
		if (!m->sent++) {
			m->first_time = net_now;
			snprintf(when, sizeof(when), "%.1f s after it came", now - m->queued);
		} else
			snprintf(when, sizeof(when), "transmission %d, %.0f s after the first", m->sent, net_now - m->first_time);
		LOGP_CHAN(DERMES, LOGL_NOTICE, "Page sent to %s, %s, %s: %s; at minute %d, subsequence %d, time slot %d%s\n",
			  ermes_print_address(m->page.local_address), ermes_priority_name(m->priority),
			  when, ermes_page_str(&pages[i]), t.cycle, t.ssn, t.slot,
			  m->retransmit ? "; again later" : "");
		if (m->retransmit) {
			/* again with its number after the interval (ETS 300 133-2 §4.3.1.3.1) */
			m->retransmit--;
			m->next_time = net_now + m->retransmit_interval;
			continue;
		}
		ermes_queue_remove(&ermes_queue, m);
	}
	LOGP_CHAN(DERMES, LOGL_DEBUG, "Batch %c at minute %d, subsequence %d, time slot %d: %s; %d page(s)\n",
		  ermes_batch_name(t.batch), t.cycle, t.ssn, t.slot, ermes_ssi_str(&ssi), sent);

	ermes->cw_sent += ermes_batch_length(ermes->slot);
	n = ermes_batch_length(ermes->slot) * ERMES_CW_BITS;
	next_slot(ermes);
	return n;
}
