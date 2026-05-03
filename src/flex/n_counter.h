/* FLEX per-capcode message number (N) counter with LRU eviction.
 *
 * Maintains a hash table of (capcode → N_value) entries, bounded
 * at FLEX_N_COUNTER_MAX entries.  When the table is full, the
 * least-recently-used entry is evicted (its counter resets to 0
 * on next use).
 *
 * The N counter is 6-bit (0-63), wrapping.  Per ARIB STD-43A §8.5,
 * the pager checks numbered messages separately for each address,
 * so N must be sequential per capcode.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef FLEX_N_COUNTER_H
#define FLEX_N_COUNTER_H

#include <stdint.h>

/* Forward declaration — full struct in flex.h */
struct flex_tx_polarity;

/* Initialize the N_Counter hash table for a polarity.
 * Allocates the bucket array.  Returns 0 on success, -1 on failure. */
int flex_n_counter_init(struct flex_tx_polarity *pol);

/* Free all N_Counter entries and the bucket array. */
void flex_n_counter_cleanup(struct flex_tx_polarity *pol);

/* Look up or create an N_Counter entry for capcode.
 * Advances N (mod 64), moves entry to LRU head.
 * If table is full, evicts the LRU tail.
 * Returns the NEW N value (0-63) after advancing. */
uint8_t flex_n_counter_get(struct flex_tx_polarity *pol,
			   uint64_t capcode, uint32_t abs_frame);

/* Look up the current N value for capcode WITHOUT advancing.
 * Returns the current N (0-63), or -1 if capcode not in table.
 * Moves entry to LRU head (it was recently referenced). */
int flex_n_counter_peek(struct flex_tx_polarity *pol, uint64_t capcode);

/* Set the N counter for capcode to a specific value (0-63).
 * Creates the entry if it doesn't exist.  Moves to LRU head.
 * Use when an explicit msgnum= is provided via FIFO to keep
 * the counter in sync for subsequent auto-assigned messages. */
void flex_n_counter_set(struct flex_tx_polarity *pol,
			uint64_t capcode, uint32_t abs_frame, uint8_t value);

#endif /* FLEX_N_COUNTER_H */
