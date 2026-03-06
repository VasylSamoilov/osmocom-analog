/* FLEX protocol frame encoding
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * Implementation based on FLEX protocol specification (Motorola).
 * Encoding logic referenced from tinyflex by Davidson Francis (Theldus),
 * released into the public domain (Unlicense).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef FLEX_FRAME_H
#define FLEX_FRAME_H

#include <stdint.h>
#include <stddef.h>

/* ===== BCH(31,21) Error Correction (Spec Section 3.5.2) ===== */

/* Generator polynomial: x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1
 * Binary: 11101101001 = 0x769 */
#define FLEX_BCH_POLY		0x769
#define FLEX_BCH_DATA_BITS	21
#define FLEX_BCH_ECC_BITS	10
#define FLEX_BCH_PARITY_BITS	1
#define FLEX_CODEWORD_BITS	(FLEX_BCH_DATA_BITS + FLEX_BCH_ECC_BITS + FLEX_BCH_PARITY_BITS)

/* ===== Per-Word BCH Status (Spec Section 3.3.3) =====
 *
 * Each word in a received block goes through BCH(31,21) error correction
 * and even parity check.  The result is one of four states:
 *   NOT_RECEIVED — default; word was never populated (phase not active)
 *   CLEAN        — syndrome=0, parity OK; no errors detected
 *   CORRECTED    — 1-2 bit errors corrected by BCH
 *   UNCORRECTABLE — >2 bit errors; BCH failed, word is unusable
 */
enum flex_word_status {
	FLEX_WORD_NOT_RECEIVED = 0,	/* default state */
	FLEX_WORD_CLEAN,		/* no errors */
	FLEX_WORD_CORRECTED,		/* BCH corrected 1-2 bit errors */
	FLEX_WORD_UNCORRECTABLE,	/* BCH failed, data invalid */
};

/* ===== Frame Structure (Spec Section 3.3) ===== */

#define FLEX_BASE_BAUD		1600	/* S1/FIW symbol rate: always 1600 baud */
#define FLEX_BLOCKS_PER_FRAME	11
#define FLEX_WORDS_PER_BLOCK	8
#define FLEX_WORDS_PER_FRAME	(FLEX_BLOCKS_PER_FRAME * FLEX_WORDS_PER_BLOCK)

/* ===== Idle Word Patterns (Spec Section 3.4.1) ===== */

/* Idle word 1: all ones (even-indexed idle words) */
#define FLEX_IDLE_WORD_1	0xFFFFFFFFU
/* Idle word 2: all zeros (odd-indexed idle words) */
#define FLEX_IDLE_WORD_2	0x00000000U

/* ===== Emergency Re-Synchronization (Spec Section 3.2.1) ===== */

/* Default ERS cycle count.
 * Each cycle = BS + AR + BS_inv + AR_inv = 12 bytes = 96 bits.
 * 35 cycles provides ~2.1 seconds at 1600 baud.
 *
 * ERS is a standalone re-sync burst, NOT part of a data frame.
 * It forces pagers to re-acquire synchronization before data frames
 * are transmitted. ERS is emitted as a separate event in the TX stream.
 *
 * Per Section 3.2.1, the ERS duration depends on the maximum collapse
 * cycle value in use:
 *   m=0 (collapse=0): pager decodes ALL frames → ERS needs only 1.875 sec
 *   m=7 (collapse=7): pager decodes every 128th frame → ERS needs 4 minutes
 *
 * Default mode uses collapse=0, so 35 cycles (~2.1 sec) is sufficient. */
#define FLEX_ERS_CYCLES		35

/* ===== Capcode Address Ranges (Spec Section 3.8, Appendix A) ===== */

/* Short address: 7-digit capcodes (1 address word) */
#define FLEX_SHORT_ADDR_MIN	1ULL
#define FLEX_SHORT_ADDR_MAX	1933312ULL
#define FLEX_SHORT_ADDR_OFFSET	32768U		/* Added to capcode before encoding */

/* Long address: 9-10 digit capcodes (2 address words) */
#define FLEX_LONG_ADDR_MIN	2101249ULL
#define FLEX_LONG_ADDR_MAX	4297068542ULL

/* Long address set boundaries (Spec Reference Document A, Section 5.15.5) */
#define FLEX_LONG_SET12_MIN	2101249ULL
#define FLEX_LONG_SET12_MAX	1075843072ULL
#define FLEX_LONG_SET34_MIN	1075843073ULL
#define FLEX_LONG_SET34_MAX	3223326720ULL
#define FLEX_LONG_SET23_MIN	3223326721ULL
#define FLEX_LONG_SET23_MAX	4297068542ULL

/* Long address encoding offsets (derived from spec conversion tables) */
#define FLEX_LONG_OFFSET_A	2068481ULL	/* Sets 1-2, 1-3, 1-4 */
#define FLEX_LONG_OFFSET_B	2068479ULL	/* Set 2-3 */
#define FLEX_LONG_W2_SET12	2097151U	/* w2 base for set 1-2 */
#define FLEX_LONG_W2_SET34	1933312U	/* w2 base for sets 1-3, 1-4 */
#define FLEX_LONG_W1_SET23	2064383U	/* w1 base for set 2-3 */
#define FLEX_LONG_W2_SET23	1867776U	/* w2 base for set 2-3 */

/* ===== Message Limits ===== */

/* Maximum alpha message words: frame capacity minus BIW + addr + vector overhead */
#define FLEX_MAX_MSG_WORDS_ALPHA	(FLEX_WORDS_PER_FRAME - 4)
/* 3 characters per 21-bit word, minus 4 bytes overhead (fragment flags, signature, checksum) */
#define FLEX_MAX_CHARS_ALPHA		((FLEX_MAX_MSG_WORDS_ALPHA * 3) - 4)
/* Maximum numeric message words (spec limit) */
#define FLEX_MAX_MSG_WORDS_NUMERIC	8
/* 5 nibbles per 21-bit word (21/4=5), minus 2 bits overhead, divided by 4 bits per digit */
#define FLEX_MAX_CHARS_NUMERIC		(((FLEX_MAX_MSG_WORDS_NUMERIC * FLEX_BCH_DATA_BITS) - 2) >> 2)
/* Maximum hex message words and characters */
#define FLEX_MAX_MSG_WORDS_HEX		FLEX_MAX_MSG_WORDS_ALPHA
#define FLEX_MAX_CHARS_HEX		(FLEX_MAX_MSG_WORDS_HEX * 5)

/* ===== Vector Word Types (Spec Section 3.9) =====
 *
 * 3-bit type field extracted as (viw >> 4) & 0x7.
 * Values per ARIB STD-43A and confirmed by PDW (MODE_*) and
 * multimon-ng (FLEX_PAGETYPE_*):
 *   0 = Secure, 1 = Short Instruction, 2 = Tone-Only/Short Message,
 *   3 = Standard Numeric, 4 = Special Format Numeric,
 *   5 = Alphanumeric, 6 = HEX/Binary, 7 = Numbered Numeric */

#define FLEX_VECTOR_TYPE_SECURE		0x0	/* Secure message (3.9.5) */
#define FLEX_VECTOR_TYPE_SHORT_INSTR	0x1	/* Short instruction (3.9.6) */
#define FLEX_VECTOR_TYPE_TONE		0x2	/* Tone-only / short message (3.9.2) */
#define FLEX_VECTOR_TYPE_NUMERIC	0x3	/* Standard numeric (3.9.1) */
#define FLEX_VECTOR_TYPE_SPECIAL_NUM	0x4	/* Special format numeric (3.9.1) */
#define FLEX_VECTOR_TYPE_ALPHA		0x5	/* Alphanumeric (3.9.4) */
#define FLEX_VECTOR_TYPE_HEX_BINARY	0x6	/* HEX/Binary (3.9.3) */
#define FLEX_VECTOR_TYPE_NUMBERED_NUM	0x7	/* Numbered numeric (3.9.1) */

/* ===== Vector Word Bit Fields (Spec Section 3.9) =====
 *
 * Vector information word layout (21 data bits):
 *   bits 0-3:   checksum (4-bit nibble sum)
 *   bits 4-6:   type (3-bit vector type, see FLEX_VECTOR_TYPE_*)
 *   bits 7-13:  msg_start (7-bit word offset where message begins)
 *   bits 14-20: msg_words (7-bit message word count)
 *
 * For numeric vectors (types 3, 4, 7), bits 14-16 are msg_words (3 bits)
 * and bits 17-20 are the K-bit checksum (4 bits).
 *
 * For short instruction (type 1), bits 7-20 are 14-bit instruction data. */

#define FLEX_VEC_TYPE_SHIFT	4
#define FLEX_VEC_TYPE_MASK	0x07
#define FLEX_VEC_START_SHIFT	7
#define FLEX_VEC_START_MASK	0x7F
#define FLEX_VEC_LEN_SHIFT	14
#define FLEX_VEC_LEN_MASK	0x7F	/* alpha/hex: 7-bit length */
#define FLEX_VEC_NUM_LEN_MASK	0x07	/* numeric: 3-bit length */
#define FLEX_VEC_NUM_KBIT_SHIFT	17
#define FLEX_VEC_NUM_KBIT_MASK	0x0F
#define FLEX_VEC_INSTR_SHIFT	7
#define FLEX_VEC_INSTR_MASK	0x3FFF	/* 14-bit instruction data */

/* Extract fields from a decoded (21-bit) vector word */
#define FLEX_VEC_TYPE(viw)	(((viw) >> FLEX_VEC_TYPE_SHIFT) & FLEX_VEC_TYPE_MASK)
#define FLEX_VEC_START(viw)	(((viw) >> FLEX_VEC_START_SHIFT) & FLEX_VEC_START_MASK)
#define FLEX_VEC_LEN(viw)	(((viw) >> FLEX_VEC_LEN_SHIFT) & FLEX_VEC_LEN_MASK)
#define FLEX_VEC_INSTR_DATA(viw) (((viw) >> FLEX_VEC_INSTR_SHIFT) & FLEX_VEC_INSTR_MASK)

/* ===== FIW Bit Fields (Spec Section 3.6) =====
 *
 * Frame Information Word layout (21 data bits):
 *   bits 0-3:   checksum (sum of all nibbles + bit20 = 0xF)
 *   bits 4-7:   cycle (0-14)
 *   bits 8-14:  frame (0-127)
 *   bit  15:    roaming flag (n)
 *   bit  16:    repeat flag (r)
 *   bits 17-20: low traffic flags (t) */

#define FLEX_FIW_CHECKSUM_MASK	0x0F
#define FLEX_FIW_CYCLE_SHIFT	4
#define FLEX_FIW_CYCLE_MASK	0x0F
#define FLEX_FIW_FRAME_SHIFT	8
#define FLEX_FIW_FRAME_MASK	0x7F
#define FLEX_FIW_ROAMING_SHIFT	15
#define FLEX_FIW_REPEAT_SHIFT	16
#define FLEX_FIW_TRAFFIC_SHIFT	17
#define FLEX_FIW_TRAFFIC_MASK	0x0F
#define FLEX_FIW_CHECKSUM_OK	0x0F	/* expected checksum result */

/* ===== BIW1 Bit Fields (Spec Section 3.7.1) =====
 *
 * Block Information Word 1 layout (21 data bits):
 *   bits 0-3:   checksum
 *   bits 4-7:   priority address count (0-15)
 *   bits 8-9:   address field start offset (add 1 for actual word index)
 *   bits 10-15: vector field start offset (1-63)
 *   bits 16-17: carry-on (0-3)
 *   bits 18-20: collapse (0-7) */

#define FLEX_BIW1_PRIO_SHIFT	4
#define FLEX_BIW1_PRIO_MASK	0x0F
#define FLEX_BIW1_ASTART_SHIFT	8
#define FLEX_BIW1_ASTART_MASK	0x03
#define FLEX_BIW1_VSTART_SHIFT	10
#define FLEX_BIW1_VSTART_MASK	0x3F
#define FLEX_BIW1_CARRY_SHIFT	16
#define FLEX_BIW1_CARRY_MASK	0x03
#define FLEX_BIW1_COLLAPSE_SHIFT 18
#define FLEX_BIW1_COLLAPSE_MASK	0x07

/* BIW1 idle detection: all-zeros or all-ones in 21-bit data field */
#define FLEX_BIW_IDLE_ZEROS	0x00000000U
#define FLEX_BIW_IDLE_ONES	0x001FFFFFU

/* ===== BIW2/3/4 Type Field (Spec Section 3.7.2, Table 3.7.2-1) =====
 *
 * Block Information Words 2, 3, and 4 share a common layout:
 *   bits 0-3:   checksum (x)
 *   bits 4-6:   type field (f2 f1 f0) — determines word format
 *   bits 7-20:  data field (s0-s13) — content depends on type
 *
 * Type values:
 *   000 = SSID1 (local channel ID, coverage zone)
 *   001 = Date (month/day/year)
 *   010 = Time (second/minute/hour)
 *   011 = Reserved
 *   100 = Reserved
 *   101 = System Information (timezone, system messages)
 *   110 = Reserved
 *   111 = SSID2 (country code, traffic management flags)
 */
#define FLEX_BIW_TYPE_SHIFT	4
#define FLEX_BIW_TYPE_MASK	0x07

#define FLEX_BIW_TYPE_SSID1	0x00	/* 000: SSID1 (local ID, coverage zone) */
#define FLEX_BIW_TYPE_DATE	0x01	/* 001: month/day/year */
#define FLEX_BIW_TYPE_TIME	0x02	/* 010: second/minute/hour */
#define FLEX_BIW_TYPE_SYSINFO	0x05	/* 101: system information */
#define FLEX_BIW_TYPE_SSID2	0x07	/* 111: SSID2 (country code, TMF) */

/* ===== BIW Type 000: SSID1 (Spec Section 3.7.2) =====
 *
 * Local channel ID and coverage zone.
 * Per PDW: "SSID/Local ID's (i8-i0)(512) & Coverage Zones (c4-c0)(32)"
 *
 *   bits 7-11:  coverage zone (c4-c0, 5 bits, 0-31)
 *   bits 12-20: local ID (i8-i0, 9 bits, 0-511)
 */
#define FLEX_BIW_SSID1_COVERAGE_SHIFT	7
#define FLEX_BIW_SSID1_COVERAGE_MASK	0x1F	/* 5 bits */
#define FLEX_BIW_SSID1_LOCALID_SHIFT	12
#define FLEX_BIW_SSID1_LOCALID_MASK	0x01FF	/* 9 bits */

/* Table 3.7.2-3: Time Zone conversion table.
 * Index = 5-bit zone area code (Z4..Z0), value = offset from UTC in minutes.
 * Entry 16 (10000) is reserved/unspecified in the standard (marked "—"). */
#define FLEX_TZ_ENTRIES		32
#define FLEX_TZ_RESERVED	16	/* zone code 10000 = reserved */

/* Table 3.7.2-3 timezone offset in minutes, indexed by 5-bit zone code.
 * Codes 0-15 map to whole-hour offsets -12h..+12h (with -0h at 0).
 * Codes 16-31 map to fractional offsets per the standard table.
 * Code 16 is reserved (shown as "—" in standard); we use 0. */
static const int flex_tz_table[FLEX_TZ_ENTRIES] = {
	/* 00000 (-0h)  */ 0,
	/* 00001 (+1h)  */ 60,
	/* 00010 (+2h)  */ 120,
	/* 00011 (+3h)  */ 180,
	/* 00100 (+4h)  */ 240,
	/* 00101 (+5h)  */ 300,
	/* 00110 (+6h)  */ 360,
	/* 00111 (+7h)  */ 420,
	/* 01000 (+8h)  */ 480,
	/* 01001 (+9h)  */ 540,	/* Japan */
	/* 01010 (+10h) */ 600,
	/* 01011 (+11h) */ 660,
	/* 01100 (+12h) */ 720,
	/* 01101 (+3h30m) */ 210,
	/* 01110 (+4h30m) */ 270,
	/* 01111 (+5h30m) */ 330,
	/* 10000 (reserved) */ 0,
	/* 10001 (+5h45m) */ 345,
	/* 10010 (+6h30m) */ 390,
	/* 10011 (+9h30m) */ 570,
	/* 10100 (-3h30m) */ -210,
	/* 10101 (-11h) */ -660,
	/* 10110 (-10h) */ -600,
	/* 10111 (-9h)  */ -540,
	/* 11000 (-8h)  */ -480,
	/* 11001 (-7h)  */ -420,
	/* 11010 (-6h)  */ -360,
	/* 11011 (-5h)  */ -300,
	/* 11100 (-4h)  */ -240,
	/* 11101 (-3h)  */ -180,
	/* 11110 (-2h)  */ -120,
	/* 11111 (-1h)  */ -60,
};

/* Convert 5-bit zone code to UTC offset in minutes */
static inline int flex_tz_to_minutes(uint32_t zone_code)
{
	if (zone_code >= FLEX_TZ_ENTRIES)
		return 0;
	return flex_tz_table[zone_code];
}

/* Format timezone offset as human-readable string.
 * buf must be at least 20 bytes.  Returns buf. */
static inline const char *flex_tz_format(int offset_min, char *buf, int bufsz)
{
	int sign = (offset_min < 0) ? -1 : 1;
	int abs_min = (offset_min < 0) ? -offset_min : offset_min;
	int h = abs_min / 60;
	int m = abs_min % 60;

	if (m == 0)
		snprintf(buf, bufsz, "UTC%+dh", sign * h);
	else
		snprintf(buf, bufsz, "UTC%+dh%02dm", sign * h, m);
	return buf;
}

/* Find the 5-bit zone code for a given UTC offset in minutes.
 * Returns the zone code, or FLEX_TZ_RESERVED if no match. */
static inline uint32_t flex_tz_from_minutes(int offset_min)
{
	uint32_t i;
	for (i = 0; i < FLEX_TZ_ENTRIES; i++) {
		if (i == FLEX_TZ_RESERVED)
			continue;
		if (flex_tz_table[i] == offset_min)
			return i;
	}
	return FLEX_TZ_RESERVED;
}

/* ===== BIW Type 001: Date (Spec Section 3.7.2) =====
 *
 * Month, day, year.
 *   bits 7-10:  year (y4-y0, 5 bits, 0-31, base year = 1994)
 *   bits 7-11:  year is 5 bits at shift 7 per PDW evidence
 *
 * Standard layout (from Table 3.7.2-1, type 001):
 *   bits 7-11:  year (y4-y0, 5 bits, 0-31 → 1994-2025)
 *   bits 12-16: day (d4-d0, 5 bits, 1-31)
 *   bits 17-20: month (m3-m0, 4 bits, 1-12)
 *
 * Note: Our TX encoder (flex_create_biw3) uses a different layout:
 *   bits 8-11: month, bits 12-16: day, no year.
 *   This is a known TX discrepancy — the TX doesn't set the type
 *   field (bits 4-6) either.  The RX decoder follows the standard.
 */
#define FLEX_BIW_DATE_YEAR_SHIFT	7
#define FLEX_BIW_DATE_YEAR_MASK		0x1F
#define FLEX_BIW_DATE_YEAR_BASE		1994
#define FLEX_BIW_DATE_DAY_SHIFT		12
#define FLEX_BIW_DATE_DAY_MASK		0x1F
#define FLEX_BIW_DATE_MONTH_SHIFT	17
#define FLEX_BIW_DATE_MONTH_MASK	0x0F

/* ===== BIW Type 010: Time (Spec Section 3.7.2) =====
 *
 * Hour, minute, second.
 * Standard layout (from Table 3.7.2-1, type 010):
 *   bits 7-11:  hour (h4-h0, 5 bits, 0-23)
 *   bits 12-17: minute (m5-m0, 6 bits, 0-59)
 *   bits 18-20: second (s2-s0, 3 bits, 0-7 in 1/8 minute = 7.5s steps)
 */
#define FLEX_BIW_TIME_HOUR_SHIFT	7
#define FLEX_BIW_TIME_HOUR_MASK		0x1F
#define FLEX_BIW_TIME_MINUTE_SHIFT	12
#define FLEX_BIW_TIME_MINUTE_MASK	0x3F
#define FLEX_BIW_TIME_SECOND_SHIFT	18
#define FLEX_BIW_TIME_SECOND_MASK	0x07
#define FLEX_BIW_TIME_SECOND_STEP	7.5	/* each unit = 7.5 seconds */

/* ===== BIW Type 101: System Information (Spec Section 3.7.2) =====
 *
 * System messages and timezone extension.
 *   bits 7-10:  A3-A0 (4 bits, system message type)
 *   bits 11-20: I9-I0 (10 bits, system info data)
 *
 * A3-A0 values per Table 3.7.2-2:
 *   0000 = Message for all subscriber units
 *   0001 = Message for all Home subscriber units
 *   0010 = Message for all Roaming subscriber units
 *   0011 = Message for all SSID subscriber units
 *   0100 = Time-related (extended seconds + DST + timezone)
 *   0101 = Additional time instruction
 *   0110 = Channel setup instruction
 *   0111-1111 = Reserved
 */
#define FLEX_BIW_SYSINFO_A_SHIFT	7
#define FLEX_BIW_SYSINFO_A_MASK		0x0F
#define FLEX_BIW_SYSINFO_I_SHIFT	11
#define FLEX_BIW_SYSINFO_I_MASK		0x03FF	/* 10 bits */

/* ===== BIW Type 111: SSID2 (Spec Section 3.7.2) =====
 *
 * Country code and traffic management flags.
 *   bits 7-10:  traffic management flags (T3-T0, 4 bits)
 *   bits 11-20: country code (c9-c0, 10 bits, ITU-T E.212)
 */
#define FLEX_BIW_SSID2_TMF_SHIFT	7
#define FLEX_BIW_SSID2_TMF_MASK		0x0F
#define FLEX_BIW_SSID2_COUNTRY_SHIFT	11
#define FLEX_BIW_SSID2_COUNTRY_MASK	0x03FF	/* 10 bits */

/* Extract BIW type field from a decoded 21-bit BIW word */
static inline uint32_t flex_biw_type(uint32_t biw)
{
	return (biw >> FLEX_BIW_TYPE_SHIFT) & FLEX_BIW_TYPE_MASK;
}

/* Human-readable name for BIW type */
static inline const char *flex_biw_type_name(uint32_t btype)
{
	switch (btype) {
	case FLEX_BIW_TYPE_SSID1:   return "SSID1";
	case FLEX_BIW_TYPE_DATE:    return "Date";
	case FLEX_BIW_TYPE_TIME:    return "Time";
	case FLEX_BIW_TYPE_SYSINFO: return "SysInfo";
	case FLEX_BIW_TYPE_SSID2:   return "SSID2";
	default:                    return "Reserved";
	}
}

/* ===== BIW Date Year Equivalence (for years >2025) =====
 *
 * The standard's 5-bit year field covers 1994-2025.  For years beyond
 * 2025, find a calendar-equivalent year in range: same leap/non-leap
 * status AND same Jan 1 weekday, so pagers display correct day-of-week.
 * E.g. 2026 (Thu Jan 1, non-leap) → 2015 (Thu Jan 1, non-leap).
 *
 * Tomohiko Sakamoto's algorithm for day-of-week (no mktime needed). */
static inline int flex_biw_is_leap(int y)
{
	return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

static inline int flex_biw_jan1_dow(int y)
{
	static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
	return (y + y/4 - y/100 + y/400 + t[0] + 1) % 7;
}

/* Map any year to a calendar-equivalent year in 1994-2025.
 * Returns the original year if already in range. */
static inline int flex_biw_equiv_year(int year)
{
	int target_leap, target_dow, y;

	if (year >= FLEX_BIW_DATE_YEAR_BASE &&
	    year <= FLEX_BIW_DATE_YEAR_BASE + (int)FLEX_BIW_DATE_YEAR_MASK)
		return year;

	target_leap = flex_biw_is_leap(year);
	target_dow = flex_biw_jan1_dow(year);

	for (y = FLEX_BIW_DATE_YEAR_BASE + (int)FLEX_BIW_DATE_YEAR_MASK;
	     y >= FLEX_BIW_DATE_YEAR_BASE; y--) {
		if (flex_biw_is_leap(y) == target_leap &&
		    flex_biw_jan1_dow(y) == target_dow)
			return y;
	}

	return FLEX_BIW_DATE_YEAR_BASE +
	       ((year - FLEX_BIW_DATE_YEAR_BASE) & FLEX_BIW_DATE_YEAR_MASK);
}

/* Reverse: given a decoded BIW year (1994-2025), find the real year
 * closest to sys_year that is calendar-equivalent. */
static inline int flex_biw_real_year(int biw_year, int sys_year)
{
	int y;

	if (sys_year <= FLEX_BIW_DATE_YEAR_BASE + (int)FLEX_BIW_DATE_YEAR_MASK)
		return biw_year; /* no wrap era */

	/* Search upward from biw_year in steps that preserve calendar */
	for (y = biw_year; y <= sys_year + 32; y++) {
		if (y > FLEX_BIW_DATE_YEAR_BASE + (int)FLEX_BIW_DATE_YEAR_MASK &&
		    flex_biw_equiv_year(y) == biw_year &&
		    y >= sys_year - 5 && y <= sys_year + 5)
			return y;
	}

	return biw_year; /* no match found, return as-is */
}

/* ===== Address Word Type Ranges (Spec Table 3.8.1-1) =====
 *
 * After BCH decode, the 21-bit address word value determines its type.
 * All ranges below are the raw 21-bit word values (NOT capcodes).
 *
 * Long Address words (used in pairs):
 *   LA1: 0x000001–0x008000  (1–32,768)         32,768 addresses
 *   LA3: 0x1E0001–0x1E8000  (1,966,081–1,998,848)  32,768
 *   LA4: 0x1E8001–0x1F0000  (1,998,849–2,031,616)  32,768
 *   LA2: 0x1F7FFF–0x1FFFFE  (2,064,383–2,097,150)  32,768
 *
 * Short Address (individual, single word):
 *   SA:  0x008001–0x1E0000  (32,769–1,966,080)  1,933,312
 *
 * Special single-word address types (between LA4 and LA2):
 *   Reserved Short:     0x1F0001–0x1F27FF  (2,031,617–2,041,855)  10,239
 *   Info Service:       0x1F2800–0x1F67FF  (2,041,856–2,058,239)  16,384
 *   Network:            0x1F6800–0x1F77FF  (2,058,240–2,062,335)   4,096
 *   Temporary:          0x1F7800–0x1F780F  (2,062,336–2,062,351)      16
 *   Operator Messaging: 0x1F7810–0x1F781F  (2,062,352–2,062,367)      16
 *   Reserved Short 2:   0x1F7820–0x1F7FFE  (2,062,368–2,064,382)   2,015
 */

/* Long Address word ranges */
#define FLEX_LA1_MIN	0x000001U	/* 1 */
#define FLEX_LA1_MAX	0x008000U	/* 32,768 */
#define FLEX_LA2_MIN	0x1F7FFFU	/* 2,064,383 */
#define FLEX_LA2_MAX	0x1FFFFEU	/* 2,097,150 */
#define FLEX_LA3_MIN	0x1E0001U	/* 1,966,081 */
#define FLEX_LA3_MAX	0x1E8000U	/* 1,998,848 */
#define FLEX_LA4_MIN	0x1E8001U	/* 1,998,849 */
#define FLEX_LA4_MAX	0x1F0000U	/* 2,031,616 */

/* Short Address range */
#define FLEX_ADDR_SHORT_MIN	0x008001U	/* 32,769 */
#define FLEX_ADDR_SHORT_MAX	0x1E0000U	/* 1,966,080 */

/* Special address ranges (Table 3.8.1-1 remarks)
 *
 * These are single-word address types between LA4 and LA2 that serve
 * special protocol functions.  They are NOT user-assignable capcodes.
 *
 * Reserved Short:     Reserved for future use (no defined behavior).
 * Info Service:       Under study (no defined behavior yet).
 * Network:            NID system messages (Section 6.1.2).
 *                     Transmitted using Secure (type 0) vector.
 *                     Message contains Service Area ID, Multiplier,
 *                     and Traffic Management Flags.
 * Temporary:          Group messaging (Section 5.2).
 *                     16 addresses assigned via short instruction vectors.
 *                     Used for temporary group address assignment.
 * Operator Messaging: System messages and change instructions (Section 3.8.2.4).
 *                     LSB 0000–0100 (5 addrs): System Messages
 *                       0000 = all pagers
 *                       0001 = all pagers in Home
 *                       0010 = all Roaming pagers
 *                       0011 = all SSID pagers
 *                       0100 = Time-related for all pagers
 *                     LSB 1110–1111 (2 addrs): system change instructions
 *                     LSB 0101–1101 (9 addrs): reserved for future use
 */
#define FLEX_ADDR_RSVD_SHORT1_MIN	0x1F0001U	/* 2,031,617 — reserved for future use */
#define FLEX_ADDR_RSVD_SHORT1_MAX	0x1F27FFU	/* 2,041,855 */
#define FLEX_ADDR_INFO_SVC_MIN		0x1F2800U	/* 2,041,856 — under study */
#define FLEX_ADDR_INFO_SVC_MAX		0x1F67FFU	/* 2,058,239 */
#define FLEX_ADDR_NETWORK_MIN		0x1F6800U	/* 2,058,240 — NID (Section 6.1.2) */
#define FLEX_ADDR_NETWORK_MAX		0x1F77FFU	/* 2,062,335 */
#define FLEX_ADDR_TEMPORARY_MIN		0x1F7800U	/* 2,062,336 — group messaging (Section 5.2) */
#define FLEX_ADDR_TEMPORARY_MAX		0x1F780FU	/* 2,062,351 */
#define FLEX_ADDR_OPER_MSG_MIN		0x1F7810U	/* 2,062,352 — system messages (Section 3.8.2.4) */
#define FLEX_ADDR_OPER_MSG_MAX		0x1F781FU	/* 2,062,367 */
#define FLEX_ADDR_RSVD_SHORT2_MIN	0x1F7820U	/* 2,062,368 — reserved for future use */
#define FLEX_ADDR_RSVD_SHORT2_MAX	0x1F7FFEU	/* 2,064,382 */

/* Operator Messaging Address sub-types (Section 3.8.2.4).
 * The 4 LSBs of the 21-bit address word determine the function. */
#define FLEX_OPER_MSG_LSB_MASK		0x0FU
#define FLEX_OPER_MSG_SYSMSG_MAX	0x04U	/* LSB 0000–0100: System Messages */
#define FLEX_OPER_MSG_INSTR_MIN		0x0EU	/* LSB 1110: system change instruction */
#define FLEX_OPER_MSG_INSTR_MAX		0x0FU	/* LSB 1111: system change instruction */

/* Operator Messaging System Message sub-type LSB values (Section 3.8.2.4) */
#define FLEX_OPER_MSG_ALL_PAGERS	0x00U	/* all pagers */
#define FLEX_OPER_MSG_HOME		0x01U	/* all pagers in Home */
#define FLEX_OPER_MSG_ROAMING		0x02U	/* all Roaming pagers */
#define FLEX_OPER_MSG_SSID		0x03U	/* all SSID pagers */
#define FLEX_OPER_MSG_TIME		0x04U	/* Time-related for all pagers */

/* ===== Temporary Address Slots (Spec Section 5.2) =====
 *
 * 16 temporary address slots (0x1F7800–0x1F780F) are assigned via
 * short instruction vectors.  The slot index is the 4 LSBs of the
 * temporary address word value. */
#define FLEX_TEMP_ADDR_SLOTS		16
#define FLEX_TEMP_ADDR_SLOT_MASK	0x0FU

/* Maximum capcodes per temporary group slot.
 * Each assignment takes 2 frame words (address + vector), and a frame
 * has 88 words total minus BIW/other overhead, so ~40 assignments max
 * across all 16 slots.  8 per slot is a practical upper bound. */
#define FLEX_TEMP_GROUP_MAX_MEMBERS	8

/* Extract temporary address slot index (0-15) from address word */
static inline uint32_t flex_temp_addr_slot(uint32_t aw)
{
	return (aw - FLEX_ADDR_TEMPORARY_MIN) & FLEX_TEMP_ADDR_SLOT_MASK;
}

/* ===== Short Instruction Type Field (Spec Section 3.9.6, ARIB STD-43A §5.2) =====
 *
 * The 14-bit instruction data from a short instruction vector (type 1)
 * is extracted by FLEX_VEC_INSTR_DATA() from bits 7-20 of the 21-bit word.
 *
 * Within the 14-bit value (Fig. 3.9.6-1):
 *   bits 0-2:   i0 i1 i2  (3-bit instruction type)
 *   bits 3-13:  d0-d10    (11-bit instruction-specific data)
 *
 * For Temporary Address (i2i1i0 = 000, Table 3.9.6-1):
 *   bits 3-9:   f0-f6  (7-bit target frame number for group message)
 *   bits 10-13: a0-a3  (4-bit temp address slot index, 0-15)
 *
 * The system sends each group member's individual address + short
 * instruction vector telling it "listen on temp slot a3-a0 in frame
 * f6-f0."  Multiple capcodes can be assigned to the same slot.
 * Then the group message is sent to that temp address in the
 * designated frame, reaching all assigned pagers.
 *
 * Per §5.2: "The assigned Temporary Address is only valid for the
 * Frame in which the Temporary Address is transmitted."
 */
#define FLEX_INSTR_TYPE_MASK		0x07U	/* bits 0-2: instruction type (i2 i1 i0) */
#define FLEX_INSTR_DATA_SHIFT		3	/* instruction-specific data starts at bit 3 */
#define FLEX_INSTR_FRAME_SHIFT		3	/* bits 3-9: target frame (f0-f6) */
#define FLEX_INSTR_FRAME_MASK		0x7FU	/* 7-bit frame number */
#define FLEX_INSTR_SLOT_SHIFT		10	/* bits 10-13: temp address slot (a0-a3) */
#define FLEX_INSTR_SLOT_MASK		0x0FU	/* 4-bit slot index */

/* Instruction type values (i2 i1 i0) */
#define FLEX_INSTR_TYPE_TEMP_ADDR	0x00U	/* 000: Temporary Address assignment (§5.2) */
#define FLEX_INSTR_TYPE_SYS_EVENT	0x01U	/* 001: System Event Notification (§3.8.2.4) */

/* Extract instruction type (i2 i1 i0) from 14-bit instruction data */
static inline uint32_t flex_instr_type(uint32_t instr_data)
{
	return instr_data & FLEX_INSTR_TYPE_MASK;
}

/* Extract temp address slot (a3-a0) from 14-bit instruction data.
 * Only valid when instruction type = FLEX_INSTR_TYPE_TEMP_ADDR. */
static inline uint32_t flex_instr_slot(uint32_t instr_data)
{
	return (instr_data >> FLEX_INSTR_SLOT_SHIFT) & FLEX_INSTR_SLOT_MASK;
}

/* Extract target frame number (f6-f0) from 14-bit instruction data.
 * Only valid when instruction type = FLEX_INSTR_TYPE_TEMP_ADDR. */
static inline uint32_t flex_instr_frame(uint32_t instr_data)
{
	return (instr_data >> FLEX_INSTR_FRAME_SHIFT) & FLEX_INSTR_FRAME_MASK;
}

/* Human-readable name for instruction type */
static inline const char *flex_instr_type_name(uint32_t itype)
{
	switch (itype) {
	case FLEX_INSTR_TYPE_TEMP_ADDR: return "TempAddr";
	case FLEX_INSTR_TYPE_SYS_EVENT: return "SysEvent";
	default:                        return "Reserved";
	}
}

/* ===== Network Message Payload (Spec Section 6.1.2) =====
 *
 * Network (NID) messages are transmitted using Secure (type 0) vectors.
 * The message payload contains system information fields.
 *
 * First message word layout (21 bits):
 *   bits 0-11:  Service Area ID (12 bits, 0-4095)
 *   bits 12-15: Coverage Zone Count (4 bits, 0-15)
 *   bits 16-20: Traffic Management Flags (5 bits)
 *
 * Traffic Management Flags (Section 6.1.2):
 *   bit 16: System overload indicator
 *   bit 17: Reserved
 *   bit 18: Reserved
 *   bit 19: Reserved
 *   bit 20: Reserved
 */
#define FLEX_NET_AREA_ID_MASK		0x0FFFU		/* bits 0-11 */
#define FLEX_NET_AREA_ID_BITS		12
#define FLEX_NET_COVERAGE_SHIFT		12
#define FLEX_NET_COVERAGE_MASK		0x0FU		/* bits 12-15 */
#define FLEX_NET_TRAFFIC_SHIFT		16
#define FLEX_NET_TRAFFIC_MASK		0x1FU		/* bits 16-20 */
#define FLEX_NET_OVERLOAD_BIT		(1U << 16)	/* bit 16: overload */

/* Extract network message fields from first message word */
static inline uint32_t flex_net_area_id(uint32_t mw)
{
	return mw & FLEX_NET_AREA_ID_MASK;
}

static inline uint32_t flex_net_coverage_zones(uint32_t mw)
{
	return (mw >> FLEX_NET_COVERAGE_SHIFT) & FLEX_NET_COVERAGE_MASK;
}

static inline uint32_t flex_net_traffic_flags(uint32_t mw)
{
	return (mw >> FLEX_NET_TRAFFIC_SHIFT) & FLEX_NET_TRAFFIC_MASK;
}

static inline int flex_net_is_overloaded(uint32_t mw)
{
	return (mw & FLEX_NET_OVERLOAD_BIT) ? 1 : 0;
}

/* ===== Operator Messaging Sub-type Classification (Section 3.8.2.4) ===== */

/* Operator messaging sub-type categories */
enum flex_oper_msg_category {
	FLEX_OPER_CAT_SYSMSG,		/* System message (LSB 0x00-0x04) */
	FLEX_OPER_CAT_CHANGE_INSTR,	/* Change instruction (LSB 0x0E-0x0F) */
	FLEX_OPER_CAT_RESERVED,		/* Reserved (LSB 0x05-0x0D) */
};

/* Classify an operator messaging address word into its category */
static inline enum flex_oper_msg_category flex_oper_msg_category(uint32_t aw)
{
	uint32_t lsb = aw & FLEX_OPER_MSG_LSB_MASK;
	if (lsb <= FLEX_OPER_MSG_SYSMSG_MAX)
		return FLEX_OPER_CAT_SYSMSG;
	if (lsb >= FLEX_OPER_MSG_INSTR_MIN && lsb <= FLEX_OPER_MSG_INSTR_MAX)
		return FLEX_OPER_CAT_CHANGE_INSTR;
	return FLEX_OPER_CAT_RESERVED;
}

/* Human-readable name for operator messaging category */
static inline const char *flex_oper_msg_category_name(enum flex_oper_msg_category cat)
{
	switch (cat) {
	case FLEX_OPER_CAT_SYSMSG:       return "SystemMessage";
	case FLEX_OPER_CAT_CHANGE_INSTR: return "ChangeInstruction";
	case FLEX_OPER_CAT_RESERVED:     return "Reserved";
	}
	return "?";
}

/* ===== Address Word Group/Temporary Flags (Spec Section 3.8.2.2) =====
 *
 * In the 21-bit address word, bit 20 (MSB) is the group flag and bit 19
 * is the temporary group flag.  These must be extracted BEFORE the
 * remaining bits are classified per Table 3.8.1-1.
 *
 * Bit layout:
 *   bit 20:    G  — group address flag (1 = group, 0 = individual)
 *   bit 19:    T  — temporary group flag (1 = temporary, 0 = common)
 *   bits 0-18: base address word (classified per Table 3.8.1-1)
 *
 * For individual addresses (G=0), bits 19-20 are part of the normal
 * address word value and the full 21-bit word is classified directly.
 * For group addresses (G=1), bit 19 distinguishes common vs temporary
 * group, and bits 0-18 encode the base capcode. */

#define FLEX_ADDR_GROUP_BIT	(1U << 20)	/* bit 20: group flag */
#define FLEX_ADDR_TEMP_BIT	(1U << 19)	/* bit 19: temporary group flag */
#define FLEX_ADDR_BASE_MASK	0x0007FFFFU	/* bits 0-18: base address (19 bits) */

/* Extract group/temporary flags and base address from a 21-bit address word.
 *
 * After BCH decode, call this BEFORE flex_classify_addr_word() to separate
 * the group/temp flags from the base address.
 *
 * Parameters:
 *   aw         — 21-bit BCH-decoded address word
 *   is_group   — output: 1 if group address, 0 if individual
 *   is_temp    — output: 1 if temporary group, 0 otherwise
 *                (only meaningful when is_group=1)
 *
 * Returns the base address word for classification:
 *   - If G=0 (individual): returns aw unchanged (full 21 bits)
 *   - If G=1 (group): returns bits 0-18 (base address without flags)
 */
static inline uint32_t flex_decode_addr_flags(uint32_t aw, int *is_group, int *is_temp)
{
	*is_group = (aw & FLEX_ADDR_GROUP_BIT) ? 1 : 0;
	*is_temp = (aw & FLEX_ADDR_TEMP_BIT) ? 1 : 0;

	if (*is_group)
		return aw & FLEX_ADDR_BASE_MASK;
	return aw;
}

/* ===== Address Word Type Classification (Spec Table 3.8.1-1) ===== */

enum flex_addr_type {
	FLEX_ADDR_LONG1,		/* Long Address 1 (pair word) */
	FLEX_ADDR_SHORT,		/* Short Address (individual) */
	FLEX_ADDR_LONG3,		/* Long Address 3 (pair word) */
	FLEX_ADDR_LONG4,		/* Long Address 4 (pair word) */
	FLEX_ADDR_RSVD_SHORT,		/* Reserved Short Address */
	FLEX_ADDR_INFO_SVC,		/* Information Service Address */
	FLEX_ADDR_NETWORK,		/* Network Address (see 6.1.2) */
	FLEX_ADDR_TEMPORARY,		/* Temporary Address */
	FLEX_ADDR_OPER_MSG,		/* Operator Messaging Address (see 3.8.2.4) */
	FLEX_ADDR_LONG2,		/* Long Address 2 (pair word) */
	FLEX_ADDR_UNKNOWN,		/* Unknown / invalid */
};

/* Classify a 21-bit address word per Table 3.8.1-1.
 * Returns the address type enum value. */
static inline enum flex_addr_type flex_classify_addr_word(uint32_t aw)
{
	if (aw >= FLEX_LA1_MIN      && aw <= FLEX_LA1_MAX)      return FLEX_ADDR_LONG1;
	if (aw >= FLEX_ADDR_SHORT_MIN && aw <= FLEX_ADDR_SHORT_MAX) return FLEX_ADDR_SHORT;
	if (aw >= FLEX_LA3_MIN      && aw <= FLEX_LA3_MAX)      return FLEX_ADDR_LONG3;
	if (aw >= FLEX_LA4_MIN      && aw <= FLEX_LA4_MAX)      return FLEX_ADDR_LONG4;
	if (aw >= FLEX_ADDR_RSVD_SHORT1_MIN && aw <= FLEX_ADDR_RSVD_SHORT1_MAX) return FLEX_ADDR_RSVD_SHORT;
	if (aw >= FLEX_ADDR_INFO_SVC_MIN    && aw <= FLEX_ADDR_INFO_SVC_MAX)    return FLEX_ADDR_INFO_SVC;
	if (aw >= FLEX_ADDR_NETWORK_MIN     && aw <= FLEX_ADDR_NETWORK_MAX)     return FLEX_ADDR_NETWORK;
	if (aw >= FLEX_ADDR_TEMPORARY_MIN   && aw <= FLEX_ADDR_TEMPORARY_MAX)   return FLEX_ADDR_TEMPORARY;
	if (aw >= FLEX_ADDR_OPER_MSG_MIN    && aw <= FLEX_ADDR_OPER_MSG_MAX)    return FLEX_ADDR_OPER_MSG;
	if (aw >= FLEX_ADDR_RSVD_SHORT2_MIN && aw <= FLEX_ADDR_RSVD_SHORT2_MAX) return FLEX_ADDR_RSVD_SHORT;
	if (aw >= FLEX_LA2_MIN      && aw <= FLEX_LA2_MAX)      return FLEX_ADDR_LONG2;
	return FLEX_ADDR_UNKNOWN;
}

/* Human-readable name for an address word type. */
static inline const char *flex_addr_type_name(enum flex_addr_type t)
{
	switch (t) {
	case FLEX_ADDR_LONG1:      return "Long1";
	case FLEX_ADDR_SHORT:      return "Short";
	case FLEX_ADDR_LONG3:      return "Long3";
	case FLEX_ADDR_LONG4:      return "Long4";
	case FLEX_ADDR_RSVD_SHORT: return "RsvdShort";
	case FLEX_ADDR_INFO_SVC:   return "InfoSvc";
	case FLEX_ADDR_NETWORK:    return "Network";
	case FLEX_ADDR_TEMPORARY:  return "Temporary";
	case FLEX_ADDR_OPER_MSG:   return "OperMsg";
	case FLEX_ADDR_LONG2:      return "Long2";
	case FLEX_ADDR_UNKNOWN:    return "Unknown";
	}
	return "?";
}

/* Check if an address word is a long address component (needs a pair).
 * NOTE: For group addresses, pass the base word (after flex_decode_addr_flags)
 * since the group bit would push the value out of the LA ranges. */
static inline int flex_addr_is_long(uint32_t aw)
{
	enum flex_addr_type t = flex_classify_addr_word(aw);
	return (t == FLEX_ADDR_LONG1 || t == FLEX_ADDR_LONG2 ||
		t == FLEX_ADDR_LONG3 || t == FLEX_ADDR_LONG4);
}

/* ===== RX Log Flag Characters (shared constants for human-readable output) =====
 *
 * Single-character flags used in RX log lines to indicate address properties.
 * Defined as constants to avoid hardcoded characters scattered through dsp.c.
 *
 * Address type flags (mutually exclusive):
 *   'S' = Short individual    'L' = Long individual
 *   'N' = Network (NID)       'T' = Temporary address slot
 *   'O' = Operator messaging  'I' = Info service
 *   'R' = Reserved short      '?' = Unknown
 *
 * Group modifier flags (appended after type):
 *   'G' = Group address       'g' = Temporary group address
 */
#define FLEX_RX_FLAG_SHORT	'S'
#define FLEX_RX_FLAG_LONG	'L'
#define FLEX_RX_FLAG_NETWORK	'N'
#define FLEX_RX_FLAG_TEMPORARY	'T'
#define FLEX_RX_FLAG_OPER_MSG	'O'
#define FLEX_RX_FLAG_INFO_SVC	'I'
#define FLEX_RX_FLAG_RESERVED	'R'
#define FLEX_RX_FLAG_UNKNOWN	'?'
#define FLEX_RX_FLAG_GROUP	'G'
#define FLEX_RX_FLAG_TEMP_GROUP	'g'

/* Get the single-character address type flag for RX logging.
 * Returns one of the FLEX_RX_FLAG_* characters. */
static inline char flex_addr_type_flag(enum flex_addr_type t, int is_long)
{
	switch (t) {
	case FLEX_ADDR_SHORT:      return FLEX_RX_FLAG_SHORT;
	case FLEX_ADDR_LONG1:
	case FLEX_ADDR_LONG2:
	case FLEX_ADDR_LONG3:
	case FLEX_ADDR_LONG4:      return FLEX_RX_FLAG_LONG;
	case FLEX_ADDR_NETWORK:    return FLEX_RX_FLAG_NETWORK;
	case FLEX_ADDR_TEMPORARY:  return FLEX_RX_FLAG_TEMPORARY;
	case FLEX_ADDR_OPER_MSG:   return FLEX_RX_FLAG_OPER_MSG;
	case FLEX_ADDR_INFO_SVC:   return FLEX_RX_FLAG_INFO_SVC;
	case FLEX_ADDR_RSVD_SHORT: return FLEX_RX_FLAG_RESERVED;
	case FLEX_ADDR_UNKNOWN:    return FLEX_RX_FLAG_UNKNOWN;
	}
	/* Fallback for non-classified but known long */
	if (is_long)
		return FLEX_RX_FLAG_LONG;
	return FLEX_RX_FLAG_UNKNOWN;
}

/* Get the group modifier flag character for RX logging.
 * Returns 'G' for group, 'g' for temporary group, ' ' for individual. */
static inline char flex_group_flag_char(int is_group, int is_temp)
{
	if (is_group && is_temp)
		return FLEX_RX_FLAG_TEMP_GROUP;
	if (is_group)
		return FLEX_RX_FLAG_GROUP;
	return ' ';
}

/* Long address set name from w1/w2 types. */
static inline const char *flex_long_set_name(uint32_t w1, uint32_t w2)
{
	enum flex_addr_type t1 = flex_classify_addr_word(w1);
	enum flex_addr_type t2 = flex_classify_addr_word(w2);

	if (t1 == FLEX_ADDR_LONG1 && t2 == FLEX_ADDR_LONG2) return "Set1-2";
	if (t1 == FLEX_ADDR_LONG1 && t2 == FLEX_ADDR_LONG3) return "Set1-3";
	if (t1 == FLEX_ADDR_LONG1 && t2 == FLEX_ADDR_LONG4) return "Set1-4";
	if (t1 == FLEX_ADDR_LONG2 && t2 == FLEX_ADDR_LONG3) return "Set2-3";
	return "InvalidSet";
}

/* Capcode-level address type name (for TX logging).
 * Returns "Short", "Long(Set1-2)", etc. based on capcode range.
 * Also identifies special protocol addresses in the gap. */
static inline const char *flex_capcode_type_name(uint64_t capcode)
{
	if (capcode >= FLEX_SHORT_ADDR_MIN && capcode <= FLEX_SHORT_ADDR_MAX)
		return "Short";
	if (capcode >= FLEX_LONG_SET12_MIN && capcode <= FLEX_LONG_SET12_MAX)
		return "Long(Set1-2)";
	if (capcode >= FLEX_LONG_SET34_MIN && capcode <= FLEX_LONG_SET34_MAX)
		return "Long(Set3-4)";
	if (capcode >= FLEX_LONG_SET23_MIN && capcode <= FLEX_LONG_SET23_MAX)
		return "Long(Set2-3)";
	/* Special addresses in the gap between short and long ranges */
	if (capcode > FLEX_SHORT_ADDR_MAX && capcode < FLEX_LONG_ADDR_MIN) {
		uint32_t aw = (uint32_t)(capcode + FLEX_SHORT_ADDR_OFFSET);
		enum flex_addr_type t = flex_classify_addr_word(aw);
		return flex_addr_type_name(t);
	}
	return "Invalid";
}

/* Check if an address word is a special (non-user-assignable) type.
 * Returns 1 for Network, Temporary, OperMsg, InfoSvc, RsvdShort. */
static inline int flex_addr_is_special(uint32_t aw)
{
	enum flex_addr_type t = flex_classify_addr_word(aw);
	return (t == FLEX_ADDR_RSVD_SHORT || t == FLEX_ADDR_INFO_SVC ||
		t == FLEX_ADDR_NETWORK   || t == FLEX_ADDR_TEMPORARY ||
		t == FLEX_ADDR_OPER_MSG);
}

/* Check if a capcode maps to a special protocol address.
 * Special addresses occupy the gap between short (max 1933312) and
 * long (min 2101249) capcode ranges.  Returns 1 if special. */
static inline int flex_capcode_is_special(uint64_t capcode)
{
	uint32_t aw;
	if (capcode > FLEX_SHORT_ADDR_MAX && capcode < FLEX_LONG_ADDR_MIN) {
		aw = (uint32_t)(capcode + FLEX_SHORT_ADDR_OFFSET);
		return flex_addr_is_special(aw);
	}
	return 0;
}

/* Classify a capcode's special address type.
 * Returns the address type, or FLEX_ADDR_UNKNOWN if not special. */
static inline enum flex_addr_type flex_capcode_special_type(uint64_t capcode)
{
	uint32_t aw;
	if (capcode > FLEX_SHORT_ADDR_MAX && capcode < FLEX_LONG_ADDR_MIN) {
		aw = (uint32_t)(capcode + FLEX_SHORT_ADDR_OFFSET);
		if (flex_addr_is_special(aw))
			return flex_classify_addr_word(aw);
	}
	return FLEX_ADDR_UNKNOWN;
}

/* Operator Messaging sub-type name from 4 LSBs of address word.
 * Per Section 3.8.2.4:
 *   LSB 0000 = SysMsg(All), 0001 = SysMsg(Home), 0010 = SysMsg(Roaming),
 *   0011 = SysMsg(SSID), 0100 = SysMsg(Time),
 *   1110-1111 = ChangeInstr, 0101-1101 = Reserved */
static inline const char *flex_oper_msg_subtype_name(uint32_t aw)
{
	uint32_t lsb = aw & FLEX_OPER_MSG_LSB_MASK;
	switch (lsb) {
	case FLEX_OPER_MSG_ALL_PAGERS: return "SysMsg(All)";
	case FLEX_OPER_MSG_HOME:       return "SysMsg(Home)";
	case FLEX_OPER_MSG_ROAMING:    return "SysMsg(Roaming)";
	case FLEX_OPER_MSG_SSID:       return "SysMsg(SSID)";
	case FLEX_OPER_MSG_TIME:       return "SysMsg(Time)";
	case FLEX_OPER_MSG_INSTR_MIN:
	case FLEX_OPER_MSG_INSTR_MAX:  return "ChangeInstr";
	default:                       return "Reserved";
	}
}

/* Special address detail string for logging.
 * Returns a brief description of the special address type and its
 * standard reference, or NULL for non-special addresses. */
static inline const char *flex_special_addr_detail(uint32_t aw)
{
	enum flex_addr_type t = flex_classify_addr_word(aw);
	switch (t) {
	case FLEX_ADDR_NETWORK:    return "NID system message (§6.1.2, Secure type)";
	case FLEX_ADDR_TEMPORARY:  return "group messaging (§5.2)";
	case FLEX_ADDR_OPER_MSG:   return NULL; /* caller uses flex_oper_msg_subtype_name() */
	case FLEX_ADDR_INFO_SVC:   return "info service (under study)";
	case FLEX_ADDR_RSVD_SHORT: return "reserved for future use";
	default:                   return NULL;
	}
}

/* ===== Address Decode Helpers (Spec Appendix A, Section 6) =====
 *
 * Shared inline functions for RX address decoding.  These are the exact
 * inverse of the TX encode_short_address() / encode_long_address() in
 * frame.c, derived from the standard's conversion formulas.
 */

/* Decode a short address word to capcode.
 * Inverse of: aw = capcode + FLEX_SHORT_ADDR_OFFSET */
static inline uint64_t flex_decode_short_address(uint32_t aw)
{
	return (uint64_t)aw - FLEX_SHORT_ADDR_OFFSET;
}

/* Decode a long address (2 words) to capcode.
 * Detects the address set from w1/w2 ranges and applies the correct
 * inverse formula per Spec Appendix A Section 6.
 *
 * Returns the capcode, or 0 on invalid w1/w2 combination. */
static inline uint64_t flex_decode_long_address(uint32_t w1, uint32_t w2)
{
	/* Set 1-2: w1 in LA1, w2 in LA2
	 * capcode = w1 + (2097151 - w2) * 32768 + 2068480 */
	if (w1 >= FLEX_LA1_MIN && w1 <= FLEX_LA1_MAX &&
	    w2 >= FLEX_LA2_MIN && w2 <= FLEX_LA2_MAX)
		return (uint64_t)w1
		       + (uint64_t)(FLEX_LONG_W2_SET12 - w2) * FLEX_SHORT_ADDR_OFFSET
		       + (FLEX_LONG_OFFSET_A - 1);

	/* Set 1-3 / 1-4: w1 in LA1, w2 in LA3 or LA4
	 * capcode = w1 + (w2 - 1933312) * 32768 + 2068480 */
	if (w1 >= FLEX_LA1_MIN && w1 <= FLEX_LA1_MAX &&
	    w2 >= FLEX_LA3_MIN && w2 <= FLEX_LA4_MAX)
		return (uint64_t)w1
		       + (uint64_t)(w2 - FLEX_LONG_W2_SET34) * FLEX_SHORT_ADDR_OFFSET
		       + (FLEX_LONG_OFFSET_A - 1);

	/* Set 2-3: w1 in LA2, w2 in LA3
	 * capcode = (w1 - 2064383) + (w2 - 1867776) * 32768 + 2068479 */
	if (w1 >= FLEX_LA2_MIN && w1 <= FLEX_LA2_MAX &&
	    w2 >= FLEX_LA3_MIN && w2 <= FLEX_LA3_MAX)
		return (uint64_t)(w1 - FLEX_LONG_W1_SET23)
		       + (uint64_t)(w2 - FLEX_LONG_W2_SET23) * FLEX_SHORT_ADDR_OFFSET
		       + FLEX_LONG_OFFSET_B;

	return 0; /* invalid combination */
}

/* 21-bit data mask (useful for masking BCH-decoded words) */
#define FLEX_DATA_MASK		((1U << FLEX_BCH_DATA_BITS) - 1)

/* ===== Numeric BCD Character Table (ARIB STD-43A Section 3.10.2, Table 3.10.2.1-1) =====
 *
 * 4-bit BCD encoding for numeric messages (B3=MSB, B0=LSB).
 * Single shared table used by both TX encoder and RX decoder.
 *
 *   0x0-0x9 = digits '0'-'9'
 *   0xA     = Spare (was linefeed pre-G1.4, now reserved — displayed as '.')
 *   0xB     = U (urgency)
 *   0xC     = Space
 *   0xD     = Hyphen/dash
 *   0xE     = ] (right bracket)
 *   0xF     = [ (left bracket) */
#define FLEX_NUM_BCD_SPARE	0x0A
#define FLEX_NUM_BCD_URGENCY	0x0B
#define FLEX_NUM_BCD_SPACE	0x0C
#define FLEX_NUM_BCD_HYPHEN	0x0D
#define FLEX_NUM_BCD_RBRACKET	0x0E
#define FLEX_NUM_BCD_LBRACKET	0x0F

/* ASCII representation of the spare nibble (0xA).
 * '.' chosen because it is visually distinct, does not collide with any
 * defined BCD character, and clearly signals a reserved/unused position. */
#define FLEX_NUM_BCD_SPARE_CHAR	'.'

/* Nibble→char decode table (index = 4-bit BCD value, value = ASCII char).
 * Defined as a macro so it can be used to initialize a static const array
 * in any translation unit without multiple-definition issues. */
#define FLEX_NUM_BCD_TABLE \
	{ '0','1','2','3','4','5','6','7','8','9', \
	  FLEX_NUM_BCD_SPARE_CHAR,'U',' ','-',']','[' }

/* Char→nibble encode: inline helper for TX.
 * Returns the 4-bit BCD nibble for a given ASCII character. */
static inline uint8_t flex_num_char_to_bcd(uint8_t ch)
{
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	switch (ch) {
	case FLEX_NUM_BCD_SPARE_CHAR:
				 return FLEX_NUM_BCD_SPARE;
	case 'U': case 'u': return FLEX_NUM_BCD_URGENCY;
	case ' ':            return FLEX_NUM_BCD_SPACE;
	case '-': case '_':  return FLEX_NUM_BCD_HYPHEN;
	case ']':            return FLEX_NUM_BCD_RBRACKET;
	case '[':            return FLEX_NUM_BCD_LBRACKET;
	default:             return 0;
	}
}

/* Numeric message overhead bits (Spec Section 3.10.2):
 *   Standard/Special numeric: 2 overhead bits at start of first word
 *   Numbered numeric: 10-bit header (message number + retrieval) + 2 overhead = 12 bits */
#define FLEX_NUM_OVERHEAD_BITS		2
#define FLEX_NUM_NUMBERED_HDR_BITS	10
#define FLEX_NUM_NUMBERED_SKIP_BITS	(FLEX_NUM_NUMBERED_HDR_BITS + FLEX_NUM_OVERHEAD_BITS)

/* ===== Alpha Message Fragment Flags (Spec Section 3.8.8.3) ===== */

/* f0f1 = 11 indicates initial (and possibly only) fragment */
#define FLEX_ALPHA_FRAG_INITIAL		0x1800U

/* Fragment number F (2 bits) — modulo 3 sequence per spec:
 *   First fragment:  F=11 (3)
 *   Second:          F=00 (0) — skips 11 to avoid confusion with new initial
 *   Third:           F=01 (1)
 *   Fourth:          F=10 (2)
 *   Fifth:           F=00 (0), etc.
 * Sequence: 3, 0, 1, 2, 0, 1, 2, 0, 1, 2, ...
 */
static inline uint32_t flex_fragment_number(int fragment_index)
{
	if (fragment_index <= 0)
		return 3; /* initial fragment: F=11 */
	return (uint32_t)((fragment_index - 1) % 3);
}

/* Alpha message header word (1st word) bit layout:
 *   bits 0-9:   K  (10-bit fragment checksum)
 *   bit  10:    C  (message continued flag)
 *   bits 11-12: F  (2-bit fragment number, mod 3)
 *   bits 13-18: N  (6-bit message number, 0-63)
 *   bit  19:    R  (message retrieval flag)
 *   bit  20:    M  (mail drop flag)
 */
#define FLEX_ALPHA_HDR_K_MASK		0x000003FFU	/* bits 0-9 */
#define FLEX_ALPHA_HDR_K_BITS		10
#define FLEX_ALPHA_HDR_C_SHIFT		10
#define FLEX_ALPHA_HDR_C_MASK		(1U << 10)
#define FLEX_ALPHA_HDR_F_SHIFT		11
#define FLEX_ALPHA_HDR_F_MASK		(0x03U << 11)
#define FLEX_ALPHA_HDR_N_SHIFT		13
#define FLEX_ALPHA_HDR_N_MASK		(0x3FU << 13)
#define FLEX_ALPHA_HDR_R_SHIFT		19
#define FLEX_ALPHA_HDR_R_MASK		(1U << 19)
#define FLEX_ALPHA_HDR_M_SHIFT		20
#define FLEX_ALPHA_HDR_M_MASK		(1U << 20)

/* K checksum groups: 3 groups per word (bits 0-7, 8-15, 16-20) */
#define FLEX_ALPHA_K_GRP1_MASK		0xFFU		/* bits 0-7 */
#define FLEX_ALPHA_K_GRP2_SHIFT		8
#define FLEX_ALPHA_K_GRP2_MASK		0xFFU		/* bits 8-15 */
#define FLEX_ALPHA_K_GRP3_SHIFT		16
#define FLEX_ALPHA_K_GRP3_MASK		0x1FU		/* bits 16-20 */

/* HEX/Binary message header word (1st word) bit layout (Spec §3.10.1.2):
 *   bits 0-11:  K  (12-bit fragment checksum)
 *   bit  12:    C  (message continued flag)
 *   bits 13-14: F  (2-bit fragment number, mod 3)
 *   bits 15-20: N  (6-bit message number, 0-63)
 */
#define FLEX_HEX_HDR_K_MASK		0x00000FFFU	/* bits 0-11 */
#define FLEX_HEX_HDR_K_BITS		12
#define FLEX_HEX_HDR_C_SHIFT		12
#define FLEX_HEX_HDR_C_MASK		(1U << 12)
#define FLEX_HEX_HDR_F_SHIFT		13
#define FLEX_HEX_HDR_F_MASK		(0x03U << 13)
#define FLEX_HEX_HDR_N_SHIFT		15
#define FLEX_HEX_HDR_N_MASK		(0x3FU << 15)

/* HEX/Binary 2nd word (first fragment only, Spec §3.10.1.2):
 *   bit  0:     R  (message retrieval flag)
 *   bit  1:     M  (mail drop flag)
 *   bit  2:     D  (display direction: 0=LTR, 1=RTL)
 *   bit  3:     H  (header message flag)
 *   bits 4-7:   B  (blocking length, bits per char; 0000=16)
 *   bit  8:     I  (status info field enabler)
 *   bits 9-12:  s  (reserved, default 0000)
 *   bits 13-20: S  (8-bit signature)
 */
#define FLEX_HEX_HDR2_R_SHIFT		0
#define FLEX_HEX_HDR2_R_MASK		(1U << 0)
#define FLEX_HEX_HDR2_M_SHIFT		1
#define FLEX_HEX_HDR2_M_MASK		(1U << 1)
#define FLEX_HEX_HDR2_D_SHIFT		2
#define FLEX_HEX_HDR2_D_MASK		(1U << 2)
#define FLEX_HEX_HDR2_H_SHIFT		3
#define FLEX_HEX_HDR2_H_MASK		(1U << 3)
#define FLEX_HEX_HDR2_B_SHIFT		4
#define FLEX_HEX_HDR2_B_MASK		(0x0FU << 4)
#define FLEX_HEX_HDR2_I_SHIFT		8
#define FLEX_HEX_HDR2_I_MASK		(1U << 8)
#define FLEX_HEX_HDR2_S_SHIFT		13
#define FLEX_HEX_HDR2_S_MASK		(0xFFU << 13)

/* Signature field: bits 0-6 of the first data word (after header) */
#define FLEX_ALPHA_SIG_MASK		0x7FU		/* 7-bit signature */
#define FLEX_ALPHA_SIG_BITS		7

/* Character field: 7 bits per character, 3 per 21-bit word */
#define FLEX_ALPHA_CHAR_BITS		7
#define FLEX_ALPHA_CHAR_MASK		0x7FU
#define FLEX_ALPHA_CHAR1_SHIFT		0		/* bits 0-6 */
#define FLEX_ALPHA_CHAR2_SHIFT		7		/* bits 7-13 */
#define FLEX_ALPHA_CHAR3_SHIFT		14		/* bits 14-20 */

/* ETX termination character */
#define FLEX_ALPHA_ETX			0x03U

/* ===== Frame Buffer Size ===== */

/*
 * Total bytes for one encoded FLEX frame:
 *   ERS:   35 cycles * (2+4+2+4) bytes  = 420 bytes
 *   S1:    BS1(4) + A1(4) + B(2) + A1_inv(4) = 14 bytes
 *   FIW:   1 codeword                   =   4 bytes
 *   S2:    25 ms at data rate (bit-level buffer):
 *          1600bps/2FSK:  40 bits  =  5 bytes
 *          3200bps/2FSK:  80 bits  = 10 bytes
 *          3200bps/4FSK:  80 bits  = 10 bytes
 *          6400bps/4FSK: 160 bits  = 20 bytes
 *   Frame: 88 words * 4 bytes            = 352 bytes (1 phase)
 *          or 4 * 88 * 4                 = 1408 bytes (4 phases at 6400 bps)
 *   Total max (4-phase, 6400):           = 420+14+4+20+1408 = 1866 bytes
 */
#define FLEX_BUFFER_SIZE	1872

/* ===== Error Codes ===== */

#define FLEX_ERR_INVALID_MESSAGE	1
#define FLEX_ERR_INVALID_CAPCODE	2
#define FLEX_ERR_INVALID_BUFFER		3
#define FLEX_ERR_INVALID_GROUP		4

/* ===== Message Type Constants ===== */
/* (int values to avoid circular dependency with flex.h enum) */

#define FLEX_FRAME_MSG_TYPE_AUTO		0
#define FLEX_FRAME_MSG_TYPE_TONE	1
#define FLEX_FRAME_MSG_TYPE_NUMERIC	2
#define FLEX_FRAME_MSG_TYPE_ALPHA	3
#define FLEX_FRAME_MSG_TYPE_HEX		4
#define FLEX_FRAME_MSG_TYPE_INSTRUCTION	5
#define FLEX_FRAME_MSG_TYPE_SHORT	6

/* ===== Sync Codes (ARIB STD-43A Table 3.2-5) =====
 *
 * 16-bit outer sync codes extracted from the 64-bit S1 sync word.
 * The full 64-bit sync is: AAAA:BBBBBBBB:CCCC where
 *   BBBBBBBB = FLEX_SYNC_MARKER (0xA6C6AAAA)
 *   AAAA = outer code (identifies mode)
 *   CCCC = ~AAAA (complement for polarity detection)
 *
 * The RX decoder sees the INVERTED A-code in the upper 16 bits
 * (because the sync word is transmitted as inv.A + marker + A,
 * and the shift register captures inv.A first).
 *
 * Each define below shows:
 *   - The spec bit pattern from Table 3.2-5 (LSB-left as transmitted)
 *   - The corresponding byte array from frame.c (MSB-first, for TX)
 *   - The inverted upper-16 value used by the RX sync detector
 *
 * Verification: for each Ax, the normal byte array is the spec bit
 * pattern read MSB-first.  The _inv byte array is the bitwise inverse.
 * The 16-bit code here equals the first two bytes of the _inv array.
 */

/* A1: 1600bps/2FSK — spec: 0111100011110011 0101100100111001
 * Normal bytes: {0x78,0xF3,0x59,0x39}  Inv bytes: {0x87,0x0C,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x870C */
#define FLEX_SYNC_A1		0x870C

/* A2: 3200bps/2FSK — spec: 1000010011100111 0101100100111001
 * Normal bytes: {0x84,0xE7,0x59,0x39}  Inv bytes: {0x7B,0x18,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x7B18 */
#define FLEX_SYNC_A2		0x7B18

/* A3: 3200bps/4FSK (1600 baud) — spec: 0100111110010111 0101100100111001
 * Normal bytes: {0x4F,0x97,0x59,0x39}  Inv bytes: {0xB0,0x68,0xA6,0xC6}
 * RX outer code = inv upper16 = 0xB068 */
#define FLEX_SYNC_A3		0xB068

/* A4: 6400bps/4FSK (3200 baud) — spec: 0010000101011111 0101100100111001
 * Normal bytes: {0x21,0x5F,0x59,0x39}  Inv bytes: {0xDE,0xA0,0xA6,0xC6}
 * RX outer code = inv upper16 = 0xDEA0 */
#define FLEX_SYNC_A4		0xDEA0

/* A5 (Reserved) — spec: 1101110101001011 0101100100111001
 * Normal bytes: {0xDD,0x4B,0x59,0x39}  Inv bytes: {0x22,0xB4,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x22B4 */
#define FLEX_SYNC_A5		0x22B4

/* A6 (Reserved) — spec: 0001011000111011 0101100100111001
 * Normal bytes: {0x16,0x3B,0x59,0x39}  Inv bytes: {0xE9,0xC4,0xA6,0xC6}
 * RX outer code = inv upper16 = 0xE9C4 */
#define FLEX_SYNC_A6		0xE9C4

/* A7 (Reserved): ReFLEX protocol — spec: 1011001110000011 0101100100111001
 * Normal bytes: {0xB3,0x83,0x59,0x39}  Inv bytes: {0x4C,0x7C,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x4C7C
 * Motorola ReFLEX uses A7 reserved code; same physical layer as A4. */
#define FLEX_SYNC_A7		0x4C7C
#define FLEX_SYNC_REFLEX	FLEX_SYNC_A7

/* A8 (Reserved) — spec: 0110001101000001 0101100100111001
 * Normal bytes: {0x63,0x41,0x59,0x39}  Inv bytes: {0x9C,0xBE,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x9CBE */
#define FLEX_SYNC_A8		0x9CBE

/* A9 (Reserved) — spec: 0001101111100010 0101100100111001
 * Normal bytes: {0x1B,0xE2,0x59,0x39}  Inv bytes: {0xE4,0x1D,0xA6,0xC6}
 * RX outer code = inv upper16 = 0xE41D */
#define FLEX_SYNC_A9		0xE41D

/* A10 (Reserved) — spec: 0010110010000110 0101100100111001
 * Normal bytes: {0x2C,0x86,0x59,0x39}  Inv bytes: {0xD3,0x79,0xA6,0xC6}
 * RX outer code = inv upper16 = 0xD379 */
#define FLEX_SYNC_A10		0xD379

/* A11 (Reserved) — spec: 1010010111101000 0101100100111001
 * Normal bytes: {0xA5,0xE8,0x59,0x39}  Inv bytes: {0x5A,0x17,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x5A17 */
#define FLEX_SYNC_A11		0x5A17

/* A12 (Reserved) — spec: 1001001010001100 0101100100111001
 * Normal bytes: {0x92,0x8C,0x59,0x39}  Inv bytes: {0x6D,0x73,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x6D73 */
#define FLEX_SYNC_A12		0x6D73

/* A13 (Reserved) — spec: 0110111010011000 0101100100111001
 * Normal bytes: {0x6E,0x98,0x59,0x39}  Inv bytes: {0x91,0x67,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x9167 */
#define FLEX_SYNC_A13		0x9167

/* A14 (Reserved) — spec: 1011111001011010 0101100100111001
 * Normal bytes: {0xBE,0x5A,0x59,0x39}  Inv bytes: {0x41,0xA5,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x41A5 */
#define FLEX_SYNC_A14		0x41A5

/* A15 (Reserved) — spec: 1111000110011101 0101100100111001
 * Normal bytes: {0xF1,0x9D,0x59,0x39}  Inv bytes: {0x0E,0x62,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x0E62 */
#define FLEX_SYNC_A15		0x0E62

/* Ar: ERS re-synchronization — spec: 1100101100100000 0101100100111001
 * Normal bytes: {0xCB,0x20,0x59,0x39}  Inv bytes: {0x34,0xDF,0xA6,0xC6}
 * RX outer code = inv upper16 = 0x34DF (same convention as A1–A4)
 *
 * Per Section 3.2.1: ERS uses the same BS+A+BS_inv+A_inv structure as S1.
 * The receiver detects Ar through the normal sync detector (which always
 * returns the inv upper-16).  When detected, the pager must re-synchronize
 * its frame timing — this is NOT a data frame, no FIW/S2/DATA follows. */
#define FLEX_SYNC_AR		0x34DF

/* FLEX sync marker: the middle 32 bits of the 64-bit sync word.
 * Per ARIB STD-43A Section 3.2, the 64-bit sync is AAAA:BBBBBBBB:CCCC
 * where BBBBBBBB = 0xA6C6AAAA and AAAA ^ CCCC = 0xFFFF. */
#define FLEX_SYNC_MARKER	0xA6C6AAAAul

/* S2 (Sync Part 2) C pattern: 16 decoded bits, identical across all modes.
 * Spec Tables 3.2-1 through 3.2-4: C = 1110110110000100 = 0xED84
 * inv.C = 0001001001111011 = 0x127B */
#define FLEX_S2_C		0xED84
#define FLEX_S2_C_INV		0x127B

/* ===== Modulation Type (ARIB STD-43A Table 3.2-2) ===== */

/* Distinguishes 2-FSK (A1, A2) from 4-FSK (A3, A4) at the same symbol rate */
enum flex_mod_type {
	FLEX_MOD_2FSK = 0,	/* 2-level FSK (A1, A2) — default */
	FLEX_MOD_4FSK = 1,	/* 4-level FSK (A3, A4) */
};

/* ===== Multi-Message Frame Encoding API ===== */

/* A single message to be packed into a frame */
typedef struct flex_frame_msg {
	uint64_t	capcode;
	int		msg_type;		/* FLEX_FRAME_MSG_TYPE_* */
	const char	*message;
	int		message_length;
	int		speed;			/* 1600, 3200, or 6400 (default 1600) */
	double		polarity;		/* -1.0 or +1.0 (default -1.0) */
	int		priority;		/* 1 = priority, 0 = normal */
	int		charset;		/* 0 = ASCII, 1 = KANJI */
	int		is_group;		/* 0 = individual, 1 = group */
	int		is_temp_group;		/* 0 = common group, 1 = temporary group */
	int		sequence_num;		/* message numbering (-1 = disabled) */
	const char	*source_id;		/* source indication (NULL = none) */
	int		short_msg_idx;		/* short message index, -1 = N/A */
	int		blocking_length;	/* HEX/Binary B field: bits per character.
					 * 1-15 = that many bits, 0 = 16 bits.
					 * Spec §3.10.1.2, default 1 (raw bits). */
	int		phase;			/* phase override: -1=auto (default), 0=A, 1=B, 2=C, 3=D */

	/* Fragment state (Spec Section 3.10.1.3 / 4.2).
	 * Set by flex_fragment_queue() for multi-fragment messages.
	 * fragment_index=0, total_fragments=0 means unfragmented. */
	int		fragment_index;		/* 0-based index within message */
	int		total_fragments;	/* total fragment count (0 = not fragmented) */
} flex_frame_msg_t;

/* Frame encoding parameters */
typedef struct flex_frame_params {
	uint32_t	cycle;			/* FIW cycle (0-14) */
	uint32_t	frame;			/* FIW frame (0-127) */
	uint32_t	roaming;		/* FIW roaming flag n */
	int		collapse;		/* BIW1 collapse value (0-7) */
	int		carry_on;		/* BIW1 carry-on (0-3 frames, Spec §3.7.1) */
	int		biw_time;		/* include BIW3/BIW4 time broadcast */
	uint32_t	local_id;		/* BIW2 local ID (9 bits, 0-511) */
	uint32_t	coverage_id;		/* BIW2 coverage zone (5 bits, 0-31) */
	int		bitrate;		/* bit rate: 1600, 3200, or 6400 (bps, not baud) */
	int		modulation_type;	/* FLEX_MOD_2FSK or FLEX_MOD_4FSK */
	int		charset;		/* 0 = ASCII, 1 = KANJI */
	int		single_phase;		/* 1 = force single-phase output (for
						 * network mode per-phase encoding) */
} flex_frame_params_t;

/* ===== Phase Multiplexing (Spec Section 3.3) ===== */

/* Maximum phases: 2 at 3200 bps (2FSK or 4FSK), 4 at 6400 bps */
#define FLEX_MAX_PHASES		4

/* Per-phase channel data (Spec Sections 3.3.3, 3.3.4).
 *
 * Each phase is an independent channel carrying 88 words (11 blocks × 8).
 * After reception, each word goes through BCH(31,21) error correction
 * and even parity check.  The result is stored per word:
 *   words[i]  — 21-bit info on success, raw 32-bit codeword on failure
 *   status[i] — BCH decode outcome for this word
 *
 * RX reception context records the conditions under which this phase
 * was received, per Section 3.3.4 (phase assignment). */
typedef struct flex_phase_data {
	uint32_t		words[FLEX_WORDS_PER_FRAME];
	enum flex_word_status	status[FLEX_WORDS_PER_FRAME];
	int			word_count;	/* TX: actual words used */
	int			idle_count;	/* RX: idle word counter */

	/* RX reception context (set once before BCH decode) */
	int			rx_phase;	/* 0=A, 1=B, 2=C, 3=D; -1=not received */
	uint32_t		rx_cycle;	/* FIW cycle number (0-14) */
	uint32_t		rx_frame;	/* FIW frame number (0-127) */
	int			rx_baud;	/* symbol rate (1600 or 3200) */
	int			rx_levels;	/* 2=2FSK, 4=4FSK */
	int			rx_polarity;	/* 0=normal, 1=inverted */
} flex_phase_data_t;

/* ===== Public API ===== */

/* Encode a complete FLEX frame (ERS + sync + FIW + data + interleave).
 * Returns bytes written to buffer, or 0 on error (with *error set). */
size_t flex_encode_frame(uint64_t capcode, int msg_type,
			 const char *message, uint8_t *buffer,
			 size_t buffer_size, int *error);

/* Initialize frame params with defaults matching current one-shot behavior */
void flex_frame_params_default(flex_frame_params_t *params);

/* Encode a frame with multiple messages.
 * Returns bytes written to buffer, or 0 on error.
 * *msgs_packed is set to the number of messages that fit in the frame. */
size_t flex_encode_frame_multi(const flex_frame_msg_t *msgs, int msg_count,
			       const flex_frame_params_t *params,
			       uint8_t *buffer, size_t buffer_size,
			       int *msgs_packed, int *error);

/* Split a long message into fragments.
 * Returns number of fragments. */
int flex_fragment_message(const char *message, int msg_type,
			  int max_chars_per_fragment,
			  char **fragments, int max_fragments);

/* Encode a multi-phase frame (3200 bps: 2 phases, 6400 bps: 4 phases).
 * 3200/4FSK also uses 2 phases (A, B) packed into 4-level symbols.
 * Phase data words are interleaved in the output per Section 3.3.
 * Returns bytes written to buffer, or 0 on error. */
size_t flex_encode_frame_phased(const flex_phase_data_t *phases, int num_phases,
				const flex_frame_params_t *params,
				uint8_t *buffer, size_t buffer_size,
				int *error);

/* Auto-detect message type from content */
int flex_detect_msg_type(const char *message, int length);

/* Validate capcode (returns 1 if valid, 0 if invalid) */
int flex_capcode_valid(uint64_t capcode);

/* Group address encoding (Section 3.8.2.2) */
uint32_t flex_encode_group_address(uint64_t group_capcode, int is_temporary);

/* Temporary address assignment (Section 3.8.2.3) */
uint32_t flex_encode_temp_address(uint64_t capcode, uint64_t temp_addr);

/* Network address encoding (Section 6.1.2).
 * Encodes a network (NID) address word from the raw address value.
 * addr_offset: offset within the network range (0 to FLEX_ADDR_NETWORK_MAX - FLEX_ADDR_NETWORK_MIN).
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid offset. */
uint32_t flex_encode_network_address(uint32_t addr_offset);

/* Network message payload encoding (Section 6.1.2).
 * Encodes the first message word containing Service Area ID, Coverage
 * Zone Count, and Traffic Management Flags.
 * Returns the encoded 32-bit BCH codeword. */
uint32_t flex_encode_network_payload(uint32_t area_id, uint32_t coverage_zones,
				     uint32_t traffic_flags);

/* Operator messaging address encoding (Section 3.8.2.4).
 * Encodes an operator messaging address word from the sub-type LSB (0x00-0x0F).
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid sub-type. */
uint32_t flex_encode_oper_msg_address(uint32_t subtype_lsb);

/* Individual encoding functions (exposed for testing) */
uint32_t flex_encode_word(uint32_t dw);
uint32_t flex_word_checksum(uint32_t dw);
uint32_t reverse_bits32(uint32_t v);
void flex_interleave_block(uint32_t block_num, uint32_t *frame_words);

/* Fill a phase's word array with the proper idle pattern per Section 3.4.1.
 * For 4FSK modes, LSB phases get all-zeros instead of alternating. */
void flex_fill_idle_phase(uint32_t *words, int phase_index,
			  int mod_type, int bitrate);
uint32_t flex_create_fiw(uint32_t cycle, uint32_t frame, uint32_t n,
			 uint32_t r, uint32_t t);
uint32_t flex_create_biw1(uint32_t prio, uint32_t e_biw,
			  uint32_t s_vfield, uint32_t carry,
			  uint32_t collapse);
uint32_t flex_create_biw2(uint32_t local_id, uint32_t coverage_id);
uint32_t flex_create_biw3(uint32_t month, uint32_t day);
uint32_t flex_create_biw4(uint32_t hour, uint32_t minute);

/* Generate ERS (Emergency Re-Synchronization) burst into buffer.
 * ERS is a standalone re-sync burst, separate from data frames.
 * Each ERS cycle = BS(2) + AR(4) + BS_inv(2) + AR_inv(4) = 12 bytes.
 * Returns bytes written (cycles * 12), or 0 on error. */
size_t flex_generate_ers(uint8_t *buffer, size_t buffer_size, int cycles);

/* Generate a POCSAG idle batch for POCSAG mixing.
 * Produces: preamble (18 words of 0xAAAAAAAA) + 1 batch (sync + 16 idle codewords).
 * Returns bytes written, or 0 on error. */
size_t flex_generate_pocsag_idle(uint8_t *buffer, size_t buffer_size);

/* ===== Split Sync/Data Encoding (ARIB STD-43A Section 3.2) =====
 *
 * The sync portion (S1 + FIW) is ALWAYS at 1600/2FSK.
 * The data portion (S2 + DATA) is at the frame's target speed.
 * These functions produce the two parts separately so the DSP
 * can transmit them at the correct speeds without mid-buffer switching.
 */

/* Encode the sync portion: S1 (BS1 + Ax + B + Ax_inv) + FIW.
 * Always 18 bytes, always at 1600/2FSK.
 * Returns bytes written (18), or 0 on error. */
size_t flex_encode_sync(const flex_frame_params_t *params,
			uint8_t *buffer, size_t buffer_size);

/* Encode the data portion: S2 + interleaved phase data.
 * Transmitted at the frame's target speed/modulation after
 * the speed switch from 1600/2FSK (S1+FIW).
 * S2 structure: BS2 + C(16 bits) + inv.BS2 + inv.C(16 bits)
 *   S2 is always 25 ms at the data symbol rate.
 * For single-phase (1600bps/2FSK): S2(5) + DATA(352) = 357 bytes.
 * For 2-phase (3200bps/2FSK): S2(10) + DATA(704) = 714 bytes.
 * For 2-phase (3200bps/4FSK): S2(10) + DATA(704) = 714 bytes.
 * For 4-phase (6400bps/4FSK): S2(20) + DATA(1408) = 1428 bytes.
 * Returns bytes written, or 0 on error. */
size_t flex_encode_data(const flex_frame_msg_t *msgs, int msg_count,
			const flex_frame_params_t *params,
			uint8_t *buffer, size_t buffer_size,
			int *msgs_packed, int *error);

/* Text utilities */
const char *flex_print_message(const char *message, int message_length);
int flex_scan_message(const char *message_input, int message_input_length,
		      char *message_output, int message_output_length);

#endif /* FLEX_FRAME_H */
