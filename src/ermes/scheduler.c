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
static void start_time(ermes_t *ermes)
{
	struct timeval tv;
	struct tm tm;
	double now, into_ssn, bits;
	time_t minute;
	int slot, skip;

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
	LOGP_CHAN(DERMES, LOGL_INFO, "Transmission starts locked to %s UTC %02d:%02d:%06.3f: minute (cycle) %d, subsequence %d, time slot %d, %d bits into the batch\n",
		  ermes_clock_offset ? "network time (--clock)" : "", tm.tm_hour, tm.tm_min, now - minute,
		  ermes->cycle, ermes->ssn, ermes->slot, skip);
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

int ermes_scheduler_next_batch(ermes_t *ermes)
{
	ermes_batch_time_t t;
	ermes_ssi_t ssi;
	ermes_page_t pages[MAX_PAGES];
	ermes_msg_t *msgs[MAX_PAGES], *m;
	struct timeval tv;
	int n = 0, sent, i, carried, prio, ahead;

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
	for (prio = ERMES_PRIORITY_HIGH; prio <= ERMES_PRIORITY_LOW; prio++) {
		for (m = ermes->queue.head; m && n < MAX_PAGES; m = m->next) {
			if (m->priority != prio || ermes_address_batch(m->page.local_address) != t.batch)
				continue;
			msgs[n] = m;
			pages[n] = m->page;
			for (ahead = 0, i = 0; i < n; i++)
				ahead += (pages[i].local_address == m->page.local_address);
			pages[n++].number = ermes_number_peek(m->page.local_address, ahead);
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

	gettimeofday(&tv, NULL);
	for (i = 0; i < sent; i++) {
		m = msgs[i];
		/* the batch builder sends the first pages given, in this order */
		if (ermes_number_next(m->page.local_address) != pages[i].number)
			LOGP_CHAN(DERMES, LOGL_ERROR, "Message number %d taken out of order\n", pages[i].number);
		LOGP_CHAN(DERMES, LOGL_NOTICE, "Page sent to %s, %s, %.1f s after it came: %s; at minute %d, subsequence %d, time slot %d\n",
			  ermes_print_address(m->page.local_address), ermes_priority_name(m->priority),
			  tv.tv_sec + tv.tv_usec / 1e6 - m->queued, ermes_page_str(&pages[i]),
			  t.cycle, t.ssn, t.slot);
		ermes_queue_remove(&ermes->queue, m);
	}
	LOGP_CHAN(DERMES, LOGL_DEBUG, "Batch %c at minute %d, subsequence %d, time slot %d: %s; %d page(s)\n",
		  ermes_batch_name(t.batch), t.cycle, t.ssn, t.slot, ermes_ssi_str(&ssi), sent);

	ermes->cw_sent += ermes_batch_length(ermes->slot);
	n = ermes_batch_length(ermes->slot) * ERMES_CW_BITS;
	next_slot(ermes);
	return n;
}
