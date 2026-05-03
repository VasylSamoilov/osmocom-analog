/* FLEX per-capcode message number (N) counter with LRU eviction.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdlib.h>
#include <string.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "flex.h"
#include "n_counter.h"

/* Hash function: simple modulo on capcode */
static inline uint32_t n_hash(uint64_t capcode)
{
	return (uint32_t)(capcode % FLEX_N_COUNTER_BUCKETS);
}

/* Remove entry from LRU doubly-linked list (does not free). */
static void lru_remove(flex_tx_polarity_t *pol, flex_n_counter_t *e)
{
	if (e->lru_prev)
		e->lru_prev->lru_next = e->lru_next;
	else
		pol->n_lru_head = e->lru_next;

	if (e->lru_next)
		e->lru_next->lru_prev = e->lru_prev;
	else
		pol->n_lru_tail = e->lru_prev;

	e->lru_prev = NULL;
	e->lru_next = NULL;
}

/* Insert entry at LRU head (most recently used). */
static void lru_push_head(flex_tx_polarity_t *pol, flex_n_counter_t *e)
{
	e->lru_prev = NULL;
	e->lru_next = pol->n_lru_head;
	if (pol->n_lru_head)
		pol->n_lru_head->lru_prev = e;
	pol->n_lru_head = e;
	if (!pol->n_lru_tail)
		pol->n_lru_tail = e;
}

/* Remove entry from its hash chain. */
static void hash_remove(flex_tx_polarity_t *pol, flex_n_counter_t *e)
{
	uint32_t bucket = n_hash(e->capcode);
	flex_n_counter_t **pp = &pol->n_hash_buckets[bucket];

	while (*pp) {
		if (*pp == e) {
			*pp = e->hash_next;
			e->hash_next = NULL;
			return;
		}
		pp = &(*pp)->hash_next;
	}
}

/* Evict the LRU tail entry (least recently used). */
static void evict_lru_tail(flex_tx_polarity_t *pol)
{
	flex_n_counter_t *victim = pol->n_lru_tail;
	if (!victim)
		return;

	lru_remove(pol, victim);
	hash_remove(pol, victim);
	free(victim);
	pol->n_counter_count--;
}

/* Find entry by capcode in hash table.  Returns NULL if not found. */
static flex_n_counter_t *hash_find(flex_tx_polarity_t *pol, uint64_t capcode)
{
	uint32_t bucket = n_hash(capcode);
	flex_n_counter_t *e = pol->n_hash_buckets[bucket];

	while (e) {
		if (e->capcode == capcode)
			return e;
		e = e->hash_next;
	}
	return NULL;
}

/* Create a new entry, insert into hash and LRU head. */
static flex_n_counter_t *entry_create(flex_tx_polarity_t *pol,
				      uint64_t capcode, uint32_t abs_frame)
{
	flex_n_counter_t *e;
	uint32_t bucket;

	/* Evict if at capacity */
	while (pol->n_counter_count >= FLEX_N_COUNTER_MAX)
		evict_lru_tail(pol);

	e = calloc(1, sizeof(*e));
	if (!e)
		return NULL;

	e->capcode = capcode;
	e->n_value = 0;
	e->last_used_abs = abs_frame;

	/* Insert into hash chain */
	bucket = n_hash(capcode);
	e->hash_next = pol->n_hash_buckets[bucket];
	pol->n_hash_buckets[bucket] = e;

	/* Insert at LRU head */
	lru_push_head(pol, e);
	pol->n_counter_count++;

	return e;
}

int flex_n_counter_init(flex_tx_polarity_t *pol)
{
	pol->n_hash_buckets = calloc(FLEX_N_COUNTER_BUCKETS,
				     sizeof(flex_n_counter_t *));
	if (!pol->n_hash_buckets)
		return -1;

	pol->n_lru_head = NULL;
	pol->n_lru_tail = NULL;
	pol->n_counter_count = 0;
	return 0;
}

void flex_n_counter_cleanup(flex_tx_polarity_t *pol)
{
	flex_n_counter_t *e, *next;

	if (!pol->n_hash_buckets)
		return;

	/* Walk LRU list and free all entries */
	e = pol->n_lru_head;
	while (e) {
		next = e->lru_next;
		free(e);
		e = next;
	}

	free(pol->n_hash_buckets);
	pol->n_hash_buckets = NULL;
	pol->n_lru_head = NULL;
	pol->n_lru_tail = NULL;
	pol->n_counter_count = 0;
}

uint8_t flex_n_counter_get(flex_tx_polarity_t *pol,
			   uint64_t capcode, uint32_t abs_frame)
{
	flex_n_counter_t *e = hash_find(pol, capcode);

	if (e) {
		/* Move to LRU head */
		lru_remove(pol, e);
		lru_push_head(pol, e);
		e->last_used_abs = abs_frame;
		/* Advance N (mod 64) */
		e->n_value = (e->n_value + 1) & 0x3F;
		return e->n_value;
	}

	/* New entry — starts at N=0, first get returns N=1 */
	e = entry_create(pol, capcode, abs_frame);
	if (!e)
		return 0; /* fallback on alloc failure */

	e->n_value = (e->n_value + 1) & 0x3F;
	return e->n_value;
}

int flex_n_counter_peek(flex_tx_polarity_t *pol, uint64_t capcode)
{
	flex_n_counter_t *e = hash_find(pol, capcode);

	if (!e)
		return -1;

	/* Move to LRU head (recently referenced) */
	lru_remove(pol, e);
	lru_push_head(pol, e);

	return (int)e->n_value;
}

void flex_n_counter_set(flex_tx_polarity_t *pol,
			uint64_t capcode, uint32_t abs_frame, uint8_t value)
{
	flex_n_counter_t *e = hash_find(pol, capcode);

	if (e) {
		lru_remove(pol, e);
		lru_push_head(pol, e);
		e->last_used_abs = abs_frame;
		e->n_value = value & 0x3F;
		return;
	}

	/* Create new entry with the explicit value */
	e = entry_create(pol, capcode, abs_frame);
	if (e)
		e->n_value = value & 0x3F;
}
