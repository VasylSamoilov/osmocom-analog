/* ERMES air interface: coding layer (L3 BCH, L2 interleaving, fixed words, timing numbers)
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
 * References are to ETS 300 133-4 ed.2 (November 1997), PDF page = printed page.
 */

#include <string.h>
#include "code.h"

#define CW_MASK		((1u << ERMES_CW_BITS) - 1)
#define INFO_MASK	((1u << ERMES_INFO_BITS) - 1)

/* §6 p.24: check word = remainder of m(x) * x^12 divided by g(x) */
static uint32_t bch_remainder(uint32_t word30)
{
	int i;

	for (i = ERMES_CW_BITS - 1; i >= ERMES_CHECK_BITS; i--) {
		if (word30 & (1u << i))
			word30 ^= (uint32_t)ERMES_BCH_POLY << (i - ERMES_CHECK_BITS);
	}
	return word30;
}

uint32_t ermes_bch_encode(uint32_t info)
{
	uint32_t word = (info & INFO_MASK) << ERMES_CHECK_BITS;

	return word | bch_remainder(word);
}

/* Syndrome -> error pattern for all single and double errors (distance 6, §6 p.24).
 * 30 + 435 patterns, all with distinct syndromes. 0 = no entry. */
static uint32_t syndrome_table[1 << ERMES_CHECK_BITS];
static int syndrome_table_ready;

static void syndrome_table_init(void)
{
	int i, j;

	memset(syndrome_table, 0, sizeof(syndrome_table));
	for (i = 0; i < ERMES_CW_BITS; i++) {
		syndrome_table[bch_remainder(1u << i)] = 1u << i;
		for (j = i + 1; j < ERMES_CW_BITS; j++)
			syndrome_table[bch_remainder((1u << i) | (1u << j))] = (1u << i) | (1u << j);
	}
	syndrome_table_ready = 1;
}

int ermes_bch_decode(uint32_t *cw)
{
	uint32_t syndrome, error;

	if (!syndrome_table_ready)
		syndrome_table_init();

	*cw &= CW_MASK;
	syndrome = bch_remainder(*cw);
	if (!syndrome)
		return 0;
	error = syndrome_table[syndrome];
	if (!error)
		return -1;
	*cw ^= error;
	return __builtin_popcount(error);
}

int ermes_distance(uint32_t a, uint32_t b)
{
	return __builtin_popcount((a ^ b) & CW_MASK);
}

void ermes_interleave(const uint32_t cw[ERMES_CODEBLOCK_CW], uint8_t bits[ERMES_CODEBLOCK_CW * ERMES_CW_BITS])
{
	int col, row, n = 0;

	for (col = ERMES_CW_BITS - 1; col >= 0; col--)
		for (row = 0; row < ERMES_CODEBLOCK_CW; row++)
			bits[n++] = (cw[row] >> col) & 1;
}

void ermes_deinterleave(const uint8_t bits[ERMES_CODEBLOCK_CW * ERMES_CW_BITS], uint32_t cw[ERMES_CODEBLOCK_CW])
{
	int col, row, n = 0;

	for (row = 0; row < ERMES_CODEBLOCK_CW; row++)
		cw[row] = 0;
	for (col = ERMES_CW_BITS - 1; col >= 0; col--)
		for (row = 0; row < ERMES_CODEBLOCK_CW; row++)
			cw[row] |= (uint32_t)(bits[n++] & 1) << col;
}

/* §4.1 p.10: "the final batch in every subsequence" is the last in time (slot 15). On
 * channel 1 that is batch type P, on other channels another type (Fig.3), so the length
 * goes with the slot: batch boundaries then line up in time on all channels, which the
 * scanning procedure of Fig.3 needs. */
int ermes_batch_length(int slot)
{
	return (slot == ERMES_BATCHES - 1) ? ERMES_BATCH_CW_LAST : ERMES_BATCH_CW;
}

int ermes_batch_offset(int slot)
{
	return slot * ERMES_BATCH_CW;
}

int ermes_apt_count(int num_ia)
{
	return 9 - ((4 + num_ia) % 9);
}

int ermes_max_codeblocks(int slot, int num_ia)
{
	int words = ermes_batch_length(slot) - 5 - num_ia - ermes_apt_count(num_ia);

	return (words < 0) ? 0 : words / ERMES_CODEBLOCK_CW;
}

/* Fig.3 p.11: channel order down the figure; the rank in it is the shift in batches */
static const int channel_order[ERMES_BATCHES] = { 1, 3, 5, 7, 9, 11, 13, 15, 16, 14, 12, 10, 8, 6, 4, 2 };

int ermes_channel_first_batch(int channel)
{
	int rank;

	for (rank = 0; rank < ERMES_BATCHES; rank++) {
		if (channel_order[rank] == channel)
			return (ERMES_BATCHES - rank) % ERMES_BATCHES;
	}
	return -1;
}

int ermes_channel_batch(int channel, int slot)
{
	int first = ermes_channel_first_batch(channel);

	if (first < 0)
		return -1;
	return (first + slot) % ERMES_BATCHES;
}

double ermes_channel_frequency(int channel)
{
	return ERMES_CHANNEL_BASE_HZ + channel * ERMES_CHANNEL_STEP_HZ;
}
