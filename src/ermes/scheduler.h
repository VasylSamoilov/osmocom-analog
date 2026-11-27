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

#ifndef ERMES_SCHEDULER_H
#define ERMES_SCHEDULER_H

#include <stdint.h>

struct ermes;

/* --clock: the network clock runs this many seconds from the system clock (0 = real time) */
extern double ermes_clock_offset;

/* --tx-latency: seconds from making a sample to its transmission (< 0: the DSP buffer) */
extern double ermes_tx_latency;

/* --receiver-batch: messages to one receiver in one batch at most; more wait for its next batch.
 * A network choice: the standard allows any number (§10.2, one initial address each). */
#define ERMES_RECEIVER_BATCH_DEFAULT	2
#define ERMES_RECEIVER_BATCH_MAX	32
extern int ermes_receiver_batch;

/* Build the next batch of the channel into ermes->batch_bits: its system information, the
 * pages it carries (taken off the queue) and the continued part of a long message. The first
 * call locks the batches to UTC (ermes->skip_bits: bits of the first batch already past).
 * Returns the number of bits. (ETS 300 133-3 §13.5.4: the allocation algorithm) */
int ermes_scheduler_next_batch(struct ermes *ermes);

/* 1 if a channel carries the receiver: its FSI has the receiver's FSN (book), or the FSN is not
 * known. A page for a receiver no channel carries could never be sent. */
int ermes_scheduler_reachable(uint32_t local_address);

#endif /* ERMES_SCHEDULER_H */
