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

#ifndef ERMES_CODE_H
#define ERMES_CODE_H

#include <stdint.h>

/* §8.3.2 Table 18 p.26 */
#define ERMES_BIT_RATE		6250
#define ERMES_SYMBOL_RATE	3125

/* §4.1 p.10, §4.5 p.14 */
#define ERMES_CW_BITS		30
#define ERMES_INFO_BITS		18
#define ERMES_CHECK_BITS	12
#define ERMES_BATCHES		16	/* A..P */
#define ERMES_SUBSEQUENCES	5	/* per cycle (minute) */
#define ERMES_CYCLES		60	/* per sequence (hour) */
#define ERMES_BATCH_CW		154	/* batches A..O */
#define ERMES_BATCH_CW_LAST	190	/* batch P */
#define ERMES_SUBSEQUENCE_CW	2500	/* 12 s */
#define ERMES_CODEBLOCK_CW	9	/* §7 p.25 interleaving depth */
#define ERMES_MAX_IA		139	/* Table 1 p.13 */
#define ERMES_SUBSEQUENCE_S	12	/* seconds, §4.5 p.14 */

/* Table 17 p.26: frequency offset of the outer symbols (10 and 00); the inner ones are 1/3 */
#define ERMES_DEVIATION_HZ	4687.5

/* Field widths (bits) */
#define ERMES_ZONE_BITS		3	/* §5.3.1.1 p.16 */
#define ERMES_COUNTRY_BITS	7
#define ERMES_OPERATOR_BITS	3
#define ERMES_PA_BITS		6
#define ERMES_FSI_BITS		5	/* Fig.5 p.15 */
#define ERMES_CYCLE_BITS	6
#define ERMES_SSN_BITS		3
#define ERMES_BATCH_BITS	4
#define ERMES_SSIT_BITS		4	/* Fig.6 p.16 */
#define ERMES_SSIF_BITS		14
#define ERMES_HOUR_BITS		5	/* Fig.7 p.17 */
#define ERMES_DATE_BITS		5
#define ERMES_WEEKDAY_BITS	3	/* Fig.8 p.17 */
#define ERMES_MONTH_BITS	4
#define ERMES_YEAR_BITS		7
#define ERMES_YEAR_BASE		1990
#define ERMES_VERSION_BITS	7	/* MVR1, SVR1, Fig.9 p.17 */
#define ERMES_LA_BITS		22	/* local address, §3.1 p.8 */
#define ERMES_IA_BITS		18	/* initial address = 18 MSBs of the local address */
#define ERMES_MSG_NUMBER_BITS	5	/* Fig.10 p.18 */
#define ERMES_MSG_NUMBER_MAX	31	/* 1..31, 0 = no number (ETS 300 133-2 §4.3.1.3 p.18) */
#define ERMES_CATEGORY_BITS	2	/* Fig.11 p.19 */
#define ERMES_ALERT_BITS	3
#define ERMES_NUMERIC_BITS	4	/* ETS 300 133-2 B.1 p.45 */
#define ERMES_NUMERIC_CODES	16	/* Table B.1: the 16 characters of numeric paging */
#define ERMES_ALPHA_BITS	7	/* ETS 300 133-2 B.2 p.46 */
#define ERMES_FSI_ALL		30	/* single channel network: every FSN (§9.2 p.29) */
#define ERMES_CHANNELS		16	/* §8.2 p.25: n = 1..16 */
#define ERMES_CHANNEL_BASE_HZ	169400000.0
#define ERMES_CHANNEL_STEP_HZ	25000.0

/* End Of Message character of alphanumeric messages: DC1 = 0010001 (§5.5.4 p.23) */
#define ERMES_EOM		0x11

/* Local address -> initial address (18 MSBs) and batch type (4 LSBs), §3.1 p.8, Fig.1 */
static inline uint32_t ermes_initial_address(uint32_t local_address)
{
	return (local_address >> ERMES_BATCH_BITS) & ((1u << ERMES_IA_BITS) - 1);
}

static inline int ermes_address_batch(uint32_t local_address)
{
	return local_address & ((1u << ERMES_BATCH_BITS) - 1);
}

/* Common temporary addresses of group calls (§10.3 p.32, Table 16 p.22): the group initial
 * address, eighteen ones, and the 4-bit pointer (CTAP 0..15) in place of the batch type. Not the
 * address of a receiver. */
#define ERMES_CTA_IA		((1u << ERMES_IA_BITS) - 1)
#define ERMES_CTAP_COUNT	16
static inline int ermes_is_cta(uint32_t local_address)
{
	return ermes_initial_address(local_address) == ERMES_CTA_IA;
}

static inline uint32_t ermes_cta(int ctap)
{
	return (ERMES_CTA_IA << ERMES_BATCH_BITS) | (ctap & ((1u << ERMES_BATCH_BITS) - 1));
}

/* SSI types (Table 4 p.16) */
#define ERMES_SSIT_ZONE_HOUR_DATE	0x0
#define ERMES_SSIT_DAY_MONTH_YEAR	0x1
#define ERMES_SSIT_IVR1			0x2
#define ERMES_SSIT_ALLOWED		13	/* 0000..1100 are cycled (ed.2 §5.3.2.1 p.16) */

/* I1 version of edition 2 (§4.1 p.10: "version 02.01"), Table 7 p.17 */
#define ERMES_IVR1_MAIN		2
#define ERMES_IVR1_SUB		1

/* NEC ERMES pagers show and program the local address in "Tokyo hex" = local address * 4
 * (ERM1BV2 ROM map: ADDRESS1 default 318E40 = C63900 TOKYO HEX) */
#define ERMES_NEC_SHIFT		2

/* Fixed 30-bit words, not BCH codewords: match them as patterns. */
#define ERMES_PREAMBLE		0x08888888	/* §5.2 p.15 */
#define ERMES_SW1		0x2288282A	/* §5.2 p.15 */
#define ERMES_APT		0x24E188E4	/* §5.4 p.18 */
#define ERMES_MD		0x3579FBBB	/* §5.5.5 p.23 */

/* §6 p.24: g(x) = x^12+x^11+x^9+x^7+x^6+x^3+x^2+1 = 15315 octal */
#define ERMES_BCH_POLY		015315

/* Encode 18 information bits (bit 17 = MSB = first on air) into a 30-bit codeword:
 * information c29..c12, check c11..c0. */
uint32_t ermes_bch_encode(uint32_t info);

/* Correct up to two errors in a 30-bit codeword (in place).
 * Returns the number of corrected bits (0, 1, 2) or -1 if uncorrectable. */
int ermes_bch_decode(uint32_t *cw);

/* Hamming distance between two 30-bit words */
int ermes_distance(uint32_t a, uint32_t b);

/* §7 p.25: 9 codewords are the rows of a 9 x 30 matrix, sent column by column,
 * bit 29 of codewords 1..9 first. bits[] holds 270 bits, one per byte, air order. */
void ermes_interleave(const uint32_t cw[ERMES_CODEBLOCK_CW], uint8_t bits[ERMES_CODEBLOCK_CW * ERMES_CW_BITS]);
void ermes_deinterleave(const uint8_t bits[ERMES_CODEBLOCK_CW * ERMES_CW_BITS], uint32_t cw[ERMES_CODEBLOCK_CW]);

/* Length in codewords of the batch in time slot 0..15 of a subsequence (slot 15 = 190) */
int ermes_batch_length(int slot);

/* Codeword offset of time slot 0..15 from the start of its subsequence */
int ermes_batch_offset(int slot);

/* Table 1 note 1 p.13: number of APTs after I initial addresses */
int ermes_apt_count(int num_ia);

/* Table 1 note 2 p.13: maximum number of message codeblocks after I initial addresses */
int ermes_max_codeblocks(int slot, int num_ia);

/* Fig.3 p.11: batch type sent first in a subsequence on channel 1..16.
 * Channel 01 starts with A, 03 with P, 05 with O ... 16 with I, 14 with H ... 02 with B. */
int ermes_channel_first_batch(int channel);

/* Batch type sent in slot 0..15 of a subsequence on a channel */
int ermes_channel_batch(int channel, int slot);

/* §8.2 p.25: nominal frequency of channel 1..16 in Hz */
double ermes_channel_frequency(int channel);

/* §9.2 Fig.18 p.30: frequency subsets. A receiver has an FSN 0..15; a channel sends an FSI
 * 0..30, the subset of FSNs whose messages it carries: 0..15 one FSN, 16..23 two, 24..27 four,
 * 28..29 eight, 30 all. E.g. FSI 27 carries FSN 12..15; FSN 12 listens to FSI 12, 22, 27, 29, 30. */
#define ERMES_FSN_COUNT		16
#define ERMES_FSI_MAX		30
/* the first FSN of an FSI subset and how many; -1 if the FSI is not 0..30 */
int ermes_fsi_subset(int fsi, int *count);
/* 1 if the channel's FSI carries messages for the FSN */
int ermes_fsi_has_fsn(int fsi, int fsn);
/* 1 if two FSIs have an FSN in common (a page could go on both channels) */
int ermes_fsi_overlap(int fsi_a, int fsi_b);

#endif /* ERMES_CODE_H */
