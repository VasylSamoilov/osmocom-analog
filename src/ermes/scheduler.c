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
#include <sys/timex.h>
#include <unistd.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/main_mobile.h"
#include "../libmobile/get_time.h"
#include "ermes.h"
#include "ssi.h"
#include "scheduler.h"

#define CHAN ermes->sender.kanal
#define MAX_PAGES	ERMES_MAX_IA	/* pages per batch: as many as it has initial addresses (Table 1) */

double ermes_clock_offset = 0.0;
double ermes_tx_latency = -1.0;	/* < 0: dsp_buffer */
int ermes_group_fig16 = 0;

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

/* The system clock against UTC (NTP): the grid is set from it at the start. ETS 300 133-4 §12.4
 * wants the cycles within ±2 ms of UTC; better than NTP is not possible this way. Said once. */
static void system_clock_check(void)
{
	struct timex tx;
	int state;

	if (sample_clock_active())
		return;	/* a run of wave files: the samples are the time */
	memset(&tx, 0, sizeof(tx));
	state = ntp_adjtime(&tx);
	if (state < 0) {
		LOGP(DERMES, LOGL_NOTICE, "System clock: its NTP state can't be read; the grid is set from it (ETS 300 133-4 §12.4: within ±2 ms of UTC).\n");
		return;
	}
	if (state == TIME_ERROR || (tx.status & STA_UNSYNC)) {
		/* systemd-timesyncd (SNTP) sets the clock without the kernel's error estimate: it
		 * marks a synchronised clock with this file */
		if (access("/run/systemd/timesync/synchronized", F_OK) == 0) {
			LOGP(DERMES, LOGL_INFO, "System clock: synchronised by systemd-timesyncd (SNTP, no error estimate, typically a few ms; chrony gives an estimate and better time).\n");
			return;
		}
		LOGP(DERMES, LOGL_NOTICE, "System clock NOT synchronised (NTP): the grid is set from it, ETS 300 133-4 §12.4 wants the cycles within ±2 ms of UTC. Run chrony or ntpd (timedatectl set-ntp true).\n");
		return;
	}
	LOGP(DERMES, (tx.esterror > 2000) ? LOGL_NOTICE : LOGL_INFO,
	     "System clock: NTP synchronised, estimated error %.2f ms (maximum %.1f ms)%s\n",
	     tx.esterror / 1000.0, tx.maxerror / 1000.0,
	     (tx.esterror > 2000) ? " - more than the ±2 ms of ETS 300 133-4 §12.4: a better time source (local NTP server, GPS)" : "");
}

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
	system_clock_check();
	gettimeofday(&tv, NULL);
	/* the first sample made now is on the air after the latency and the premodulation filter */
	now = tv.tv_sec + tv.tv_usec / 1e6 + ermes_clock_offset
	    + ((ermes_tx_latency >= 0) ? ermes_tx_latency : dsp_buffer / 1000.0) + ermes->premod_delay;
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
static int fsn_on_channel(const ermes_t *ermes, uint32_t local_address, int opid)
{
	const ermes_subscriber_t *s = ermes_book_find(local_address, opid, 0);

	if (!s || s->fsn < 0)
		return 1;
	return ermes_fsi_has_fsn(ermes->net.fsi, s->fsn);
}

int ermes_scheduler_reachable(uint32_t local_address, int opid)
{
	sender_t *s;

	for (s = sender_head; s; s = s->next) {
		if (fsn_on_channel((ermes_t *)s, local_address, opid))
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
 * carries, one stage per batch, one after another. Stage 1: a member's initial address alone in its
 * own batch; it then searches its local address "in subsequent batches" (§10.3 - not in the current
 * one, as for an individual call, §10.2). Stage 2: in the next batch, a header to its local
 * address with its own next number and AII 1, AIT 111, AIN = CTAP (no initial address, no data).
 * With --group-fig16 stage 2 goes with stage 1, in the member's own batch (-5 §7.8 Fig.16).
 * Stage 3: in the batch after the last member's stage 2, the message itself to the common
 * temporary address with number 00000, ended as any message (EOM, fill, delimiter). The call ends
 * there; the CTAP is held until 12 s after the last member's initial address (§10.8.2). A call
 * starts at a time slot from which its members' batches, plus the two batches of stage 2 and 3,
 * are within 16 batches: the members search only 12 s from their initial address. While a call is
 * on, the channel sends nothing else, and it starts one only with no message running on (the
 * carry), so no batch lacks room. A member gets no other message from its initial address until
 * the message is sent (also while it runs on); in the hold it does.
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

/* 1 if the receiver is a member of a group call whose message is not sent yet: a CTAP of some
 * channel addressing or sending it (also while the message runs on into the next batches). Until
 * then it gets no other message. Once the message is sent the CTAP holds (not reused while the
 * members may still search for it) but the members take other messages from the next batch. */
static int in_group_call(uint32_t local_address)
{
	sender_t *sd;
	int c, i;

	for (sd = sender_head; sd; sd = sd->next) {
		ermes_t *e = (ermes_t *)sd;

		for (c = 0; c < ERMES_CTAP_COUNT; c++) {
			if (e->ctap[c].state == ERMES_CTAP_FREE || e->ctap[c].state == ERMES_CTAP_HOLD)
				continue;
			for (i = 0; i < e->ctap[c].members; i++) {
				if (e->ctap[c].member[i] == local_address)
					return 1;
			}
		}
	}
	return 0;
}

/* take the next group call with members this channel carries, if a CTAP is free */
static void group_start(ermes_t *ermes)
{
	ermes_msg_t *m;
	ermes_group_t *g;
	ermes_ctap_slot_t *s;
	char why[64];
	int prio, i, k, c, count, off, lo, hi;

	for (prio = ERMES_PRIORITY_HIGH; prio <= ERMES_PRIORITY_LOW; prio++) {
		for (m = ermes_queue.head; m; m = m->next) {
			if (m->priority != prio)
				continue;
			/* in the order they came: an individual page before it for this channel goes first
			 * (a retransmission waiting for its time does not hold the call) */
			if (!m->group) {
				if (!m->sent && fsn_on_channel(ermes, m->page.local_address, ERMES_PAGE_OPID(&m->page)))
					return;
				continue;
			}
			g = m->group;
			/* the members it would serve, and the time slots of their batches from now */
			lo = ERMES_BATCHES;
			hi = -1;
			for (count = 0, i = 0; i < g->num; i++) {
				if (g->channel[i] || g->done[i] || !fsn_on_channel(ermes, g->member[i], -1))
					continue;
				count++;
				off = (ermes_address_batch(g->member[i]) - ermes_channel_batch(ermes->channel, ermes->slot)
				       + ERMES_BATCHES) % ERMES_BATCHES;
				if (off < lo)
					lo = off;
				if (off > hi)
					hi = off;
			}
			if (!count)
				continue;
			/* stage 3 two batches after the last initial address, within 16 from the first */
			if (hi + 2 - lo > ERMES_BATCHES - 1) {
				LOGP_CHAN(DERMES, LOGL_DEBUG, "Group call waits for a time slot from which its stages fit\n");
				return;
			}
			/* all members of a group get the same CTA (-4 §10.3 p.32): the first channel takes
			 * a CTAP free on every channel, the others use the same one when it is free there */
			if (g->ctap >= 0) {
				c = g->ctap;
				k = (ermes->ctap[c].state == ERMES_CTAP_FREE) ? 0 : ERMES_CTAP_COUNT;
			} else {
				for (k = 0; k < ERMES_CTAP_COUNT; k++) {
					sender_t *sd;

					c = (ermes->ctap_next + k) % ERMES_CTAP_COUNT;
					for (sd = sender_head; sd; sd = sd->next) {
						if (((ermes_t *)sd)->ctap[c].state != ERMES_CTAP_FREE)
							break;
					}
					if (!sd)
						break;
				}
			}
			if (k == ERMES_CTAP_COUNT) {
				LOGP_CHAN(DERMES, LOGL_DEBUG, "Group call waits for a free CTAP\n");
				return;
			}
			g->ctap = c;
			s = &ermes->ctap[c];
			for (count = 0, i = 0; i < g->num; i++) {
				if (!g->channel[i] && !g->done[i] && fsn_on_channel(ermes, g->member[i], -1)) {
					g->channel[i] = ermes;
					s->member[count++] = g->member[i];
				}
			}
			g->active++;
			s->msg = m;
			s->members = s->left = s->ia_left = count;
			s->first = s->last = -1.0;
			s->last_pointer_at = 0;
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

/* this batch, for group_pages and group_sent: the codeword count at its start + 1 (0 = never) */
static uint64_t batch_id(const ermes_t *ermes)
{
	return ermes->cw_sent + 1;
}

/* the pages of the group call in this batch: stage 1 of the members of this batch type, stage 2
 * of the members whose stage 1 was in an earlier batch, stage 3 after the last stage 2. Returns
 * how many. */
static int group_pages(ermes_t *ermes, const ermes_batch_time_t *t, ermes_msg_t **msgs, ermes_page_t *pages)
{
	ermes_ctap_slot_t *s = &ermes->ctap[ermes->group_ctap];
	ermes_msg_t *m = s->msg;
	ermes_group_t *g = m->group;
	uint64_t now = batch_id(ermes);
	int i, n = 0;

	for (i = 0; i < g->num && n < MAX_PAGES - 1; i++) {
		if (g->channel[i] != ermes || g->done[i])
			continue;
		if (!g->ia_at[i]) {
			/* stage 1: the initial address alone, in its own batch; with --group-fig16 the
			 * pointer with it (stage 1 and 2 in one batch, as -5 §7.8 Fig.16 "IA APT MD
			 * MESSAGE 1") */
			if (ermes_address_batch(g->member[i]) != t->batch
			 || !ermes_book_active_cycle(g->member[i], -1, t->cycle))
				continue;
			memset(&pages[n], 0, sizeof(pages[n]));
			pages[n].local_address = g->member[i];
			if (ermes_group_fig16) {
				pages[n].number = ermes_number_peek(g->member[i], -1, 0);
				pages[n].group_pointer = ermes->group_ctap + 1;
			} else
				pages[n].ia_only = 1;
			msgs[n++] = m;
		} else if (g->ia_at[i] < now) {
			/* stage 2: the pointer, in the batch after its initial address */
			memset(&pages[n], 0, sizeof(pages[n]));
			pages[n].local_address = g->member[i];
			pages[n].number = ermes_number_peek(g->member[i], -1, 0);
			pages[n].group_pointer = ermes->group_ctap + 1;
			pages[n].no_ia = 1;
			msgs[n++] = m;
		}
	}
	/* stage 3: in the batch after the last stage 2 */
	if (!s->left && s->last_pointer_at && s->last_pointer_at < now) {
		pages[n] = m->page;
		pages[n].local_address = ermes_cta(ermes->group_ctap);
		pages[n].number = 0;
		msgs[n++] = m;
	}
	/* a batch of stage 2 or 3 alone gets a PAD in its address partition (ermes_build_batch) */
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
		if (pages[i].ia_only || (pages[i].group_pointer && !pages[i].no_ia)) {
			for (j = 0; j < g->num; j++) {
				if (g->member[j] == pages[i].local_address)
					g->ia_at[j] = batch_id(ermes);
			}
			s->ia_left--;
			if (s->first < 0)
				s->first = net_now;
			s->last = net_now;
			LOGP_CHAN(DERMES, LOGL_NOTICE, "Group call to %s: stage 1, the initial address; at minute %d, subsequence %d, time slot %d\n",
				  ermes_print_address(pages[i].local_address), t->cycle, t->ssn, t->slot);
			if (pages[i].ia_only)
				continue;
			/* Fig.16: the pointer with it, below */
		}
		if (pages[i].group_pointer) {
			if (ermes_number_next(pages[i].local_address, -1) != pages[i].number)
				LOGP_CHAN(DERMES, LOGL_ERROR, "Message number %d taken out of order\n", pages[i].number);
			for (j = 0; j < g->num; j++) {
				if (g->member[j] == pages[i].local_address)
					g->done[j] = 1;
			}
			s->left--;
			s->last_pointer_at = batch_id(ermes);
			LOGP_CHAN(DERMES, LOGL_NOTICE, "Group call to %s: stage 2, %s; at minute %d, subsequence %d, time slot %d\n",
				  ermes_print_address(pages[i].local_address), ermes_page_str(&pages[i]), t->cycle, t->ssn, t->slot);
			continue;
		}
		/* stage 3 */
		LOGP_CHAN(DERMES, LOGL_NOTICE, "Group call message to %s (stage 3), %.1f s after the first member's batch: %s; at minute %d, subsequence %d, time slot %d\n",
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
	int n = 0, sent, i, carried, prio, ahead, group;

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
			if (!fsn_on_channel(ermes, m->page.local_address, ERMES_PAGE_OPID(&m->page)))
				continue;
			/* only in a cycle the receiver listens in (-4 §11.4: HNL at home, ENL abroad) */
			if (!ermes_book_active_cycle(m->page.local_address, ERMES_PAGE_OPID(&m->page), t.cycle))
				continue;
			/* a member of a group call gets nothing else until its message is sent */
			if (!m->page.external && in_group_call(m->page.local_address))
				continue;
			/* a retransmission waits for its time */
			if (m->sent && m->next_time > net_now)
				continue;
			/* any number of messages to one receiver in a batch (§10.2), as room allows; the
			 * ones before it take the numbers ahead of it */
			for (ahead = 0, i = 0; i < n; i++) {
				if (pages[i].local_address == m->page.local_address
				 && ERMES_PAGE_OPID(&pages[i]) == ERMES_PAGE_OPID(&m->page))
					ahead += (msgs[i]->page.number < 0 && !msgs[i]->page.last_number);
			}
			/* lastnum: the latest number, including new ones before it in this batch; none
			 * sent yet: nothing to repeat */
			if (m->page.last_number && !ermes_number_peek(m->page.local_address, ERMES_PAGE_OPID(&m->page), ahead - 1)) {
				LOGP_CHAN(DERMES, LOGL_ERROR, "No message was sent to %s yet: no latest message number to send.\n",
					  ermes_print_address(m->page.local_address));
				ermes_queue_remove(&ermes_queue, m);
				break;
			}
			msgs[n] = m;
			pages[n] = m->page;
			/* a retransmission keeps the number of its first transmission */
			if (m->page.last_number)
				pages[n].number = ermes_number_peek(m->page.local_address, ERMES_PAGE_OPID(&m->page), ahead - 1);
			else if (m->page.number < 0)
				pages[n].number = ermes_number_peek(m->page.local_address, ERMES_PAGE_OPID(&m->page), ahead);
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
		/* the batch builder sends the first pages given (on the air in ascending order of
		 * local address; several to one address keep the order of their numbers) */
		if (m->page.number < 0 && !m->page.last_number) {
			if (ermes_number_next(m->page.local_address, ERMES_PAGE_OPID(&m->page)) != pages[i].number)
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
			/* again with its number, the repeats spread over the window from the first
			 * transmission (ETS 300 133-2 §4.3.1.3.1): with 2 in 300 s at +150 s, +300 s */
			m->retransmit--;
			m->next_time = m->first_time + (double)m->retransmit_window * (m->retransmit_total - m->retransmit)
				       / m->retransmit_total;
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
