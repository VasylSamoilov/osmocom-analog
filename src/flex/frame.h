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

/* ===== BCH(31,21) Error Correction ===== */

/* Generator polynomial: x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1
 * Binary: 11101101001 = 0x769 */
#define FLEX_BCH_POLY		0x769
#define FLEX_BCH_DATA_BITS	21
#define FLEX_BCH_ECC_BITS	10
#define FLEX_BCH_PARITY_BITS	1
#define FLEX_CODEWORD_BITS	(FLEX_BCH_DATA_BITS + FLEX_BCH_ECC_BITS + FLEX_BCH_PARITY_BITS)

/* ===== Per-Word BCH Status =====
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

/* ===== Frame Structure ===== */

#define FLEX_BASE_BAUD		1600	/* S1/FIW symbol rate: always 1600 baud */
#define FLEX_BLOCKS_PER_FRAME	11
#define FLEX_WORDS_PER_BLOCK	8
#define FLEX_WORDS_PER_FRAME	(FLEX_BLOCKS_PER_FRAME * FLEX_WORDS_PER_BLOCK)

/* ===== Idle Word Patterns =====
 *
 * When no addresses, vectors, or messages are present, the frame
 * can be shortened to block 0 only (S1 + FI + S2 + BI + IB).
 * Unused blocks are filled with idle words that produce a 1,0 bit
 * pattern at 1600 bps on the channel.
 *
 * Idle fill uses alternating all-1s and all-0s 32-bit words.
 * For 4FSK modes, LSB phases use all-zeros so the resulting
 * 4-level symbols stay at the two extreme levels (±4800 Hz),
 * reproducing the same 1600 bps binary waveform. */

/* Idle word 1: all ones (even-indexed idle words) */
#define FLEX_IDLE_WORD_1	0xFFFFFFFFU
/* Idle word 2: all zeros (odd-indexed idle words) */
#define FLEX_IDLE_WORD_2	0x00000000U

/* ===== Multiple Transmission Subframe Structure =====
 *
 * When multiple transmission is active (FIW r=1), the 88-word frame
 * is divided into N subframes (N = num_transmissions).
 *
 * Subframe sizes:
 *   2x: 2 subframes × 44 words = 88 words
 *   3x: 3 subframes × 29 words = 87 words (+1 extra idle word)
 *   4x: 4 subframes × 22 words = 88 words
 *
 * Word numbers within each subframe start at 0.
 * Word 0 always contains the Block Information Word.
 *
 * Each subframe is transmitted at a repeat interval equal to the
 * System Collapse cycle (2^m frames).  The repeat unit =
 * num_transmissions × repeat_interval.
 *
 * Per spec: "the same bit stream as transmitted for the 1st
 * transmission is transmitted for the Block Information Word,
 * Address Field, Vector Field, Message Field and Idle Blocks
 * in the 2nd, 3rd and 4th transmissions." */

/* Subframe word counts per transmission count */
#define FLEX_SUBFRAME_WORDS_2X	44
#define FLEX_SUBFRAME_WORDS_3X	29
#define FLEX_SUBFRAME_WORDS_4X	22

/* Get subframe word count for a given number of transmissions.
 * Returns 88 for 1x (no subframing), or the per-subframe count. */
static inline int flex_subframe_words(int num_transmissions)
{
	switch (num_transmissions) {
	case 2:  return FLEX_SUBFRAME_WORDS_2X;
	case 3:  return FLEX_SUBFRAME_WORDS_3X;
	case 4:  return FLEX_SUBFRAME_WORDS_4X;
	default: return FLEX_WORDS_PER_FRAME; /* 1x = full 88 words */
	}
}

/* Compute the word offset within the full 88-word frame for a given
 * subframe index.  For 3x, the last subframe has an extra idle word
 * appended (87 + 1 = 88). */
static inline int flex_subframe_offset(int num_transmissions, int subframe_index)
{
	return subframe_index * flex_subframe_words(num_transmissions);
}

/* ===== Multi-Message Frame Packing Constants =====
 *
 * Limits and intervals for the multi-message scheduler that collects
 * multiple eligible messages per frame and passes them as a batch to
 * the frame encoder (flex_encode_frame_multi).
 *
 * Fragment interval values per ARIB STD-43A §4.2 ④. */

/* Upper bound on candidate messages collected per frame or phase */
#define FLEX_MAX_CANDIDATES		64

/* Max tracked in-flight fragment streams (§4.2 ①②) */
#define FLEX_MAX_INFLIGHT		32

/* Max frames between consecutive fragments — single-channel (§4.2 ④) */
#define FLEX_FRAG_INTERVAL_SINGLE	32

/* Max frames between consecutive fragments — shared/multi-tx (§4.2 ④) */
#define FLEX_FRAG_INTERVAL_SHARED	128

/* Max subframes per frame (4x transmission) */
#define FLEX_MAX_FRAG_SLOTS		4

/* Max tracked collapse-cycle fragment streams */
#define FLEX_MAX_COLLAPSE_STATE		32

/* ===== Emergency Re-Synchronization ===== */

/* Default ERS cycle count.
 * Each cycle = BS + AR + BS_inv + AR_inv = 12 bytes = 96 bits.
 * 35 cycles provides ~2.1 seconds at 1600 baud.
 *
 * ERS is a standalone re-sync burst, NOT part of a data frame.
 * It forces pagers to re-acquire synchronization before data frames
 * are transmitted. ERS is emitted as a separate event in the TX stream.
 *
 * The ERS duration depends on the maximum collapse
 * cycle value in use:
 *   m=0 (collapse=0): pager decodes ALL frames → ERS needs only 1.875 sec
 *   m=7 (collapse=7): pager decodes every 128th frame → ERS needs 4 minutes
 *
 * Default mode uses collapse=0, so 35 cycles (~2.1 sec) is sufficient. */
#define FLEX_ERS_CYCLES		35

/* ===== Capcode Address Ranges ===== */

/* Short address: 7-digit capcodes (1 address word) */
#define FLEX_SHORT_ADDR_MIN	1ULL
#define FLEX_SHORT_ADDR_MAX	1933312ULL
#define FLEX_SHORT_ADDR_OFFSET	32768U		/* Added to capcode before encoding */

/* Long address: 9-10 digit capcodes (2 address words) */
#define FLEX_LONG_ADDR_MIN	2101249ULL
#define FLEX_LONG_ADDR_MAX	4297068542ULL

/* Long address set boundaries (3 overlapping sets, w1+w2 encoding) */
#define FLEX_LONG_SET12_MIN	2101249ULL
#define FLEX_LONG_SET12_MAX	1075843072ULL
#define FLEX_LONG_SET34_MIN	1075843073ULL
#define FLEX_LONG_SET34_MAX	3223326720ULL
#define FLEX_LONG_SET23_MIN	3223326721ULL
#define FLEX_LONG_SET23_MAX	4297068542ULL

/* Long address encoding offsets (used in capcode ↔ w1/w2 conversion) */
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

/* ===== Vector Word Types =====
 *
 * V2V1V0 type field extracted as (viw >> 4) & 0x7.
 *
 *   V   Message                  RX tag  Overview
 *   000 Secure Message           SEC     Operator control (t1t0 subtype)
 *   001 (Short Instruction)      INS     14-bit instruction data
 *   010 (Tone-Only/Short Msg)    TON     Alert only, no message body
 *   011 Standard Numeric         NUM     4-bit BCD, 1 char in 4 bits
 *   100 Special Format Numeric   SNUM    BCD, display per ID-ROM
 *   101 Alphanumeric             ALN     7-bit chars, 1 char in 7 bits
 *   110 HEX/Binary               HEX     Raw hex/binary data
 *   111 Numbered Numeric         NNUM    BCD with message number N
 *
 * Confirmed by PDW (MODE_*) and multimon-ng (FLEX_PAGETYPE_*). */

#define FLEX_VECTOR_TYPE_SECURE		0x0	/* Secure message */
#define FLEX_VECTOR_TYPE_SHORT_INSTR	0x1	/* Short instruction */
#define FLEX_VECTOR_TYPE_TONE		0x2	/* Tone-only / short message */
#define FLEX_VECTOR_TYPE_NUMERIC	0x3	/* Standard numeric */
#define FLEX_VECTOR_TYPE_SPECIAL_NUM	0x4	/* Special format numeric */
#define FLEX_VECTOR_TYPE_ALPHA		0x5	/* Alphanumeric */
#define FLEX_VECTOR_TYPE_HEX_BINARY	0x6	/* HEX/Binary */
#define FLEX_VECTOR_TYPE_NUMBERED_NUM	0x7	/* Numbered numeric */

/* ===== Vector Word Bit Fields =====
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

/* ===== FIW Bit Fields =====
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

/* ===== BIW1 Bit Fields =====
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

/* BIW1 idle detection: when BIW word 0 is all-zeros
 * or all-ones in the 21-bit data field, the frame contains no
 * addresses, vectors, or messages — skip decode. */
#define FLEX_BIW_IDLE_ZEROS	0x00000000U
#define FLEX_BIW_IDLE_ONES	0x001FFFFFU

/* ===== BIW2/3/4 Type Field =====
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

/* ===== BIW Type 000: SSID1 =====
 *
 * Local channel ID and coverage zone.
 * Per PDW: "SSID/Local ID's (i8-i0)(512) & Coverage Zones (c4-c0)(32)"
 *
 *   bits 7-11:  coverage zone (c4-c0, 5 bits, 0-31)
 *   bits 12-20: local ID (i8-i0, 9 bits, 0-511)
 *
 * LID with Coverage Zone, Country Code and Traffic Management Flag
 * defines a specific simulcast coverage area.
 *
 * Spec requirements:
 * - On RF channels supporting SSID Roaming, BIW 000 (SSID1) must be
 *   transmitted in every Frame.
 */
#define FLEX_BIW_SSID1_COVERAGE_SHIFT	7
#define FLEX_BIW_SSID1_COVERAGE_MASK	0x1F	/* 5 bits */
#define FLEX_BIW_SSID1_LOCALID_SHIFT	12
#define FLEX_BIW_SSID1_LOCALID_MASK	0x01FF	/* 9 bits */

/* Time Zone conversion table.
 * Index = 5-bit zone area code (Z4..Z0), value = offset from UTC in minutes.
 * Entry 16 (10000) is reserved/unspecified (marked "—"). */
#define FLEX_TZ_ENTRIES		32
#define FLEX_TZ_RESERVED	16	/* zone code 10000 = reserved */

/* Timezone offset in minutes, indexed by 5-bit zone code.
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

/* ===== BIW Type 001: Date =====
 *
 * Month, day, year.
 * Layout (BIW type 001, 21 data bits):
 *   bits 7-11:  year (Y4-Y0, 5 bits, 00000-11111, 1994-2025)
 *   bits 12-16: day (d4-d0, 5 bits, 00001-11111, 1-31)
 *   bits 17-20: month (m3-m0, 4 bits, 0001-1100, Jan-Dec)
 *
 * Date, Time and Time Zone based on the Standard time in each region
 * are transmitted by BIW 001, 010 and 101 respectively.  To display
 * or update the data more frequently, the information can be
 * transmitted in other Frames.
 */
#define FLEX_BIW_DATE_YEAR_SHIFT	7
#define FLEX_BIW_DATE_YEAR_MASK		0x1F
#define FLEX_BIW_DATE_YEAR_BASE		1994
#define FLEX_BIW_DATE_DAY_SHIFT		12
#define FLEX_BIW_DATE_DAY_MASK		0x1F
#define FLEX_BIW_DATE_MONTH_SHIFT	17
#define FLEX_BIW_DATE_MONTH_MASK	0x0F

/* ===== BIW Type 010: Time =====
 *
 * Hour, minute, second.
 * Layout (BIW type 010, 21 data bits):
 *   bits 7-11:  hour (h4-h0, 5 bits, 00000-10111, 0-23)
 *   bits 12-17: minute (m5-m0, 6 bits, 000000-111011, 0-59)
 *   bits 18-20: second (s2-s0, 3 bits, 000-111, 1/8 minute = 7.5s steps)
 *
 * The synchronization to the real time is based on the rising edge
 * of the 1st bit of the Bit Sync 1 of the Frame 0 for the Cycle
 * which contains that Frame.  (In the case of multiple transmission,
 * the Frame first transmission is done.)
 *
 * The Second field can be extended to 1/64 minute (0.9375s) resolution
 * by the S5-S3 field in BIW SysInfo type 101 (A=0100 or A=0101).
 */
#define FLEX_BIW_TIME_HOUR_SHIFT	7
#define FLEX_BIW_TIME_HOUR_MASK		0x1F
#define FLEX_BIW_TIME_MINUTE_SHIFT	12
#define FLEX_BIW_TIME_MINUTE_MASK	0x3F
#define FLEX_BIW_TIME_SECOND_SHIFT	18
#define FLEX_BIW_TIME_SECOND_MASK	0x07
#define FLEX_BIW_TIME_SECOND_STEP	7.5	/* each unit = 7.5 seconds */

/* ===== BIW Type 101: System Information =====
 *
 * System messages, timezone, DST, and extended seconds.
 *   bits 7-10:  A3-A0 (4 bits, system message type — see A values below)
 *   bits 11-20: I9-I0 (10 bits, system info data — layout depends on A)
 *
 * A3-A0 values:
 *   0000 = System Message for all subscriber units
 *   0001 = System Message for all pagers in Home
 *   0010 = System Message for all Roaming pagers
 *   0011 = System Message for all SSID pagers
 *   0100 = Time related for all pagers and additional Time Instruction
 *   0101 = Additional Time Instruction
 *   0110 = Channel Set Up Instruction
 *   0111-1111 = Reserved
 *
 * For A=0100 and A=0101 (time-related), the I field is structured as:
 *   I9-I7 (3 bits): S5-S3 extended Second field (0-7, 1/64 minute
 *                   = 0.9375s steps).  Extends the 3-bit Second field
 *                   in BIW Time word (type 010) for finer resolution.
 *   I6    (1 bit):  reserved
 *   I5    (1 bit):  L0 Day Light Saving Time flag.
 *                   L0=0: transmitted time is Day Light Saving Time.
 *                   L0=1: transmitted time is standard time.
 *   I4-I0 (5 bits): Z4-Z0 Time Zone code (5-bit zone code, 0-31).
 *
 * For A=0110 (Channel Set Up Instruction), the I field is:
 *   I9    (1 bit):  B0 System Message Bit (1=channel supports SysMsg)
 *   I8    (1 bit):  N0 NID System Message Bit (1=supports NID SysMsg)
 *   I7-I6 (2 bits): O1-O0 Maximum Carry On for roaming pagers
 *   I5-I0 (6 bits): F5-F0 Frame Offset (1-63)
 *
 * Spec requirements:
 * - BIW 101 for System Messages may appear once per phase per frame.
 * - When BIW 101 A=0000-0100 is present in Frame 0, corresponding
 *   vectors (except Secure) go at end of vector field, and System
 *   Messages are in the Message field.
 * - Tone-Only addresses cannot be in frames carrying System Messages.
 * - At least 1 time-related BIW (001, 010, or 101) must be in each
 *   phase of Frame 0 Cycle 0.
 */
#define FLEX_BIW_SYSINFO_A_SHIFT	7
#define FLEX_BIW_SYSINFO_A_MASK		0x0F
#define FLEX_BIW_SYSINFO_I_SHIFT	11
#define FLEX_BIW_SYSINFO_I_MASK		0x03FF	/* 10 bits */

/* SysInfo A-type values */
#define FLEX_BIW_SYSINFO_A_MSG_ALL	0x00	/* 0000: System Message for all pagers */
#define FLEX_BIW_SYSINFO_A_MSG_HOME	0x01	/* 0001: System Message for Home pagers */
#define FLEX_BIW_SYSINFO_A_MSG_ROAM	0x02	/* 0010: System Message for Roaming pagers */
#define FLEX_BIW_SYSINFO_A_MSG_SSID	0x03	/* 0011: System Message for SSID pagers */
#define FLEX_BIW_SYSINFO_A_TIME	0x04	/* 0100: Time related + additional Time Instr */
#define FLEX_BIW_SYSINFO_A_TIME_ADD	0x05	/* 0101: Additional Time Instruction */
#define FLEX_BIW_SYSINFO_A_CHAN_SETUP	0x06	/* 0110: Channel Set Up Instruction */

/* SysInfo I-field sub-fields for A=0100/0101 (time-related).
 *
 * I-field layout (10 bits, within the 21-bit data word at SYSINFO_I_SHIFT):
 *   I0-I4: Z0-Z4 timezone zone code (5 bits, 0-31)
 *   I5:    L0 DST flag (0=DST, 1=standard time)
 *   I6:    reserved
 *   I7-I9: S3-S5 extended seconds (3 bits, 1/64 min = 0.9375s steps)
 *
 * These offsets are relative to the I field value (after >> SYSINFO_I_SHIFT). */
#define FLEX_BIW_SYSINFO_TZ_MASK	0x1F	/* I4-I0: timezone zone code (5 bits) */
#define FLEX_BIW_SYSINFO_DST_SHIFT	5	/* I5: DST flag */
#define FLEX_BIW_SYSINFO_DST_MASK	0x01
#define FLEX_BIW_SYSINFO_EXTSEC_SHIFT	7	/* I7-I9: extended seconds (3 bits) */
#define FLEX_BIW_SYSINFO_EXTSEC_MASK	0x07

/* SysInfo I-field sub-fields for A=0110 (Channel Set Up Instruction).
 * Offsets relative to the I field value. */
#define FLEX_BIW_SYSINFO_FRAME_OFS_MASK  0x3F	/* I5-I0: Frame Offset F5-F0 (6 bits) */
#define FLEX_BIW_SYSINFO_CARRY_ON_SHIFT  6	/* I7-I6: Max Carry On O1-O0 (2 bits) */
#define FLEX_BIW_SYSINFO_CARRY_ON_MASK   0x03
#define FLEX_BIW_SYSINFO_NID_BIT	 8	/* I8: N0 NID System Message Bit */
#define FLEX_BIW_SYSINFO_SYSMSG_BIT	 9	/* I9: B0 System Message Bit */

/* ===== BIW Type 111: SSID2 =====
 *
 * Country code and traffic management flags.
 *   bits 7-10:  traffic management flags (T3-T0, 4 bits)
 *   bits 11-20: country code (c9-c0, 10 bits, ITU-T E.212 Annex A)
 *
 * Country Codes comply with CCITT (ITU-T) E.212 Annex A.
 * Japan's Country Code is "440".
 *
 * The 4 Traffic Management Flags indicate which of the four traffic
 * groups the channel is assigned to.  After a roaming pager detects
 * a channel with matching LID, Coverage Zone, and Country Code, the
 * pager checks which of the 4 flags corresponds to its group.  When
 * more than 1 flag is set to 0, the pager searches for other channels
 * with the same LID/Coverage/Country that have its flag set to 1.
 *
 * Spec requirements:
 * - On RF channels supporting SSID Roaming, BIW 000 (SSID1) must be
 *   transmitted in every Frame; BIW 111 (SSID2) must be transmitted
 *   in Frame 0 through Frame 3.
 * - If channels are shared or mixed on one channel ("channel mixing"),
 *   transmission of Frame 0 through Frame 3 must not be blocked.
 */
#define FLEX_BIW_SSID2_TMF_SHIFT	7
#define FLEX_BIW_SSID2_TMF_MASK		0x0F
#define FLEX_BIW_SSID2_COUNTRY_SHIFT	11
#define FLEX_BIW_SSID2_COUNTRY_MASK	0x03FF	/* 10 bits */

/* ===== ITU-T E.212 Mobile Country Code (MCC) Lookup =====
 *
 * Per ITU Operational Bulletin No. 1117 (1 February 2017),
 * Complement to Recommendation ITU-T E.212 (09/2016).
 *
 * The SSID2 country code field (c9-c0, 10 bits) carries the 3-digit
 * MCC.  This table maps MCC values to country/area names for RX
 * display and TX validation.
 *
 * Notes:
 *   a. MCCs beginning with 0, 1, or 8 are reserved for future use.
 *   b. UAE: Dubai=431, Abu Dhabi=430.
 *   c. 901 = International Mobile, shared code.
 *   d. Kosovo designation per UNSCR 1244 / ICJ Opinion.
 *
 * Only assigned codes (202-901) are included.  Codes not in this
 * table are either reserved or unassigned. */

typedef struct flex_mcc_entry {
	uint16_t	code;
	const char	*name;
} flex_mcc_entry_t;

static const flex_mcc_entry_t flex_mcc_table[] = {
	{ 202, "Greece" },
	{ 204, "Netherlands" },
	{ 206, "Belgium" },
	{ 208, "France" },
	{ 212, "Monaco" },
	{ 213, "Andorra" },
	{ 214, "Spain" },
	{ 216, "Hungary" },
	{ 218, "Bosnia-Herzegovina" },
	{ 219, "Croatia" },
	{ 220, "Serbia" },
	{ 221, "Kosovo" },
	{ 222, "Italy" },
	{ 225, "Vatican" },
	{ 226, "Romania" },
	{ 228, "Switzerland" },
	{ 230, "Czech Republic" },
	{ 231, "Slovakia" },
	{ 232, "Austria" },
	{ 234, "UK" },
	{ 235, "UK" },
	{ 238, "Denmark" },
	{ 240, "Sweden" },
	{ 242, "Norway" },
	{ 244, "Finland" },
	{ 246, "Lithuania" },
	{ 247, "Latvia" },
	{ 248, "Estonia" },
	{ 250, "Russia" },
	{ 255, "Ukraine" },
	{ 257, "Belarus" },
	{ 259, "Moldova" },
	{ 260, "Poland" },
	{ 262, "Germany" },
	{ 266, "Gibraltar" },
	{ 268, "Portugal" },
	{ 270, "Luxembourg" },
	{ 272, "Ireland" },
	{ 274, "Iceland" },
	{ 276, "Albania" },
	{ 278, "Malta" },
	{ 280, "Cyprus" },
	{ 282, "Georgia" },
	{ 283, "Armenia" },
	{ 284, "Bulgaria" },
	{ 286, "Turkey" },
	{ 288, "Faroe Islands" },
	{ 290, "Greenland" },
	{ 292, "San Marino" },
	{ 293, "Slovenia" },
	{ 294, "Macedonia" },
	{ 295, "Liechtenstein" },
	{ 297, "Montenegro" },
	{ 302, "Canada" },
	{ 308, "St. Pierre & Miquelon" },
	{ 310, "USA" },
	{ 311, "USA" },
	{ 312, "USA" },
	{ 313, "USA" },
	{ 314, "USA" },
	{ 315, "USA" },
	{ 316, "USA" },
	{ 330, "Puerto Rico" },
	{ 332, "US Virgin Islands" },
	{ 334, "Mexico" },
	{ 338, "Jamaica" },
	{ 340, "Guadeloupe/Martinique" },
	{ 342, "Barbados" },
	{ 344, "Antigua & Barbuda" },
	{ 346, "Cayman Islands" },
	{ 348, "British Virgin Islands" },
	{ 350, "Bermuda" },
	{ 352, "Grenada" },
	{ 354, "Montserrat" },
	{ 356, "St. Kitts & Nevis" },
	{ 358, "St. Lucia" },
	{ 360, "St. Vincent & Grenadines" },
	{ 362, "Curacao/Sint Maarten/BES" },
	{ 363, "Aruba" },
	{ 364, "Bahamas" },
	{ 365, "Anguilla" },
	{ 366, "Dominica" },
	{ 368, "Cuba" },
	{ 370, "Dominican Republic" },
	{ 372, "Haiti" },
	{ 374, "Trinidad & Tobago" },
	{ 376, "Turks & Caicos" },
	{ 400, "Azerbaijan" },
	{ 401, "Kazakhstan" },
	{ 402, "Bhutan" },
	{ 404, "India" },
	{ 405, "India" },
	{ 406, "India" },
	{ 410, "Pakistan" },
	{ 412, "Afghanistan" },
	{ 413, "Sri Lanka" },
	{ 414, "Myanmar" },
	{ 415, "Lebanon" },
	{ 416, "Jordan" },
	{ 417, "Syria" },
	{ 418, "Iraq" },
	{ 419, "Kuwait" },
	{ 420, "Saudi Arabia" },
	{ 421, "Yemen" },
	{ 422, "Oman" },
	{ 424, "UAE" },
	{ 425, "Israel" },
	{ 426, "Bahrain" },
	{ 427, "Qatar" },
	{ 428, "Mongolia" },
	{ 429, "Nepal" },
	{ 430, "UAE (Abu Dhabi)" },
	{ 431, "UAE (Dubai)" },
	{ 432, "Iran" },
	{ 434, "Uzbekistan" },
	{ 436, "Tajikistan" },
	{ 437, "Kyrgyzstan" },
	{ 438, "Turkmenistan" },
	{ 440, "Japan" },
	{ 441, "Japan" },
	{ 450, "South Korea" },
	{ 452, "Vietnam" },
	{ 454, "Hong Kong" },
	{ 455, "Macao" },
	{ 456, "Cambodia" },
	{ 457, "Laos" },
	{ 460, "China" },
	{ 461, "China" },
	{ 466, "Taiwan" },
	{ 467, "North Korea" },
	{ 470, "Bangladesh" },
	{ 472, "Maldives" },
	{ 502, "Malaysia" },
	{ 505, "Australia" },
	{ 510, "Indonesia" },
	{ 514, "Timor-Leste" },
	{ 515, "Philippines" },
	{ 520, "Thailand" },
	{ 525, "Singapore" },
	{ 528, "Brunei" },
	{ 530, "New Zealand" },
	{ 536, "Nauru" },
	{ 537, "Papua New Guinea" },
	{ 539, "Tonga" },
	{ 540, "Solomon Islands" },
	{ 541, "Vanuatu" },
	{ 542, "Fiji" },
	{ 543, "Wallis & Futuna" },
	{ 544, "American Samoa" },
	{ 545, "Kiribati" },
	{ 546, "New Caledonia" },
	{ 547, "French Polynesia" },
	{ 548, "Cook Islands" },
	{ 549, "Samoa" },
	{ 550, "Micronesia" },
	{ 551, "Marshall Islands" },
	{ 552, "Palau" },
	{ 553, "Tuvalu" },
	{ 554, "Tokelau" },
	{ 555, "Niue" },
	{ 602, "Egypt" },
	{ 603, "Algeria" },
	{ 604, "Morocco" },
	{ 605, "Tunisia" },
	{ 606, "Libya" },
	{ 607, "Gambia" },
	{ 608, "Senegal" },
	{ 609, "Mauritania" },
	{ 610, "Mali" },
	{ 611, "Guinea" },
	{ 612, "Cote d'Ivoire" },
	{ 613, "Burkina Faso" },
	{ 614, "Niger" },
	{ 615, "Togo" },
	{ 616, "Benin" },
	{ 617, "Mauritius" },
	{ 618, "Liberia" },
	{ 619, "Sierra Leone" },
	{ 620, "Ghana" },
	{ 621, "Nigeria" },
	{ 622, "Chad" },
	{ 623, "Central African Rep." },
	{ 624, "Cameroon" },
	{ 625, "Cabo Verde" },
	{ 626, "Sao Tome & Principe" },
	{ 627, "Equatorial Guinea" },
	{ 628, "Gabon" },
	{ 629, "Congo" },
	{ 630, "DR Congo" },
	{ 631, "Angola" },
	{ 632, "Guinea-Bissau" },
	{ 633, "Seychelles" },
	{ 634, "Sudan" },
	{ 635, "Rwanda" },
	{ 636, "Ethiopia" },
	{ 637, "Somalia" },
	{ 638, "Djibouti" },
	{ 639, "Kenya" },
	{ 640, "Tanzania" },
	{ 641, "Uganda" },
	{ 642, "Burundi" },
	{ 643, "Mozambique" },
	{ 645, "Zambia" },
	{ 646, "Madagascar" },
	{ 647, "French Indian Ocean" },
	{ 648, "Zimbabwe" },
	{ 649, "Namibia" },
	{ 650, "Malawi" },
	{ 651, "Lesotho" },
	{ 652, "Botswana" },
	{ 653, "Swaziland" },
	{ 654, "Comoros" },
	{ 655, "South Africa" },
	{ 657, "Eritrea" },
	{ 658, "St. Helena" },
	{ 659, "South Sudan" },
	{ 702, "Belize" },
	{ 704, "Guatemala" },
	{ 706, "El Salvador" },
	{ 708, "Honduras" },
	{ 710, "Nicaragua" },
	{ 712, "Costa Rica" },
	{ 714, "Panama" },
	{ 716, "Peru" },
	{ 722, "Argentina" },
	{ 724, "Brazil" },
	{ 730, "Chile" },
	{ 732, "Colombia" },
	{ 734, "Venezuela" },
	{ 736, "Bolivia" },
	{ 738, "Guyana" },
	{ 740, "Ecuador" },
	{ 742, "French Guiana" },
	{ 744, "Paraguay" },
	{ 746, "Suriname" },
	{ 748, "Uruguay" },
	{ 750, "Falkland Islands" },
	{ 901, "International Mobile" },
};

#define FLEX_MCC_TABLE_SIZE \
	(sizeof(flex_mcc_table) / sizeof(flex_mcc_table[0]))

/* Look up country/area name for a 10-bit MCC value.
 * Returns short name string, or NULL if not found. */
static inline const char *flex_mcc_name(uint32_t mcc)
{
	unsigned int i;
	for (i = 0; i < FLEX_MCC_TABLE_SIZE; i++) {
		if (flex_mcc_table[i].code == mcc)
			return flex_mcc_table[i].name;
	}
	return NULL;
}

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

/* Human-readable name for BIW SysInfo A-type */
static inline const char *flex_biw_sysinfo_a_name(uint32_t a_type)
{
	switch (a_type) {
	case FLEX_BIW_SYSINFO_A_MSG_ALL:    return "SysMsg(All)";
	case FLEX_BIW_SYSINFO_A_MSG_HOME:   return "SysMsg(Home)";
	case FLEX_BIW_SYSINFO_A_MSG_ROAM:   return "SysMsg(Roaming)";
	case FLEX_BIW_SYSINFO_A_MSG_SSID:   return "SysMsg(SSID)";
	case FLEX_BIW_SYSINFO_A_TIME:       return "Time";
	case FLEX_BIW_SYSINFO_A_TIME_ADD:   return "TimeAdd";
	case FLEX_BIW_SYSINFO_A_CHAN_SETUP:  return "ChanSetup";
	default:                             return "Reserved";
	}
}

/* Check if a BIW SysInfo A-type indicates a System Message (A=0000~0100).
 * When BIW101 has A=0000~0100,
 * vectors are placed at the end of the vector field and system messages
 * are in the message field.
 * "Tone-Only Addresses cannot be transmitted in Frames used for
 * transmitting System Messages." */
static inline int flex_biw_sysinfo_is_sysmsg(uint32_t a_type)
{
	return (a_type <= FLEX_BIW_SYSINFO_A_TIME) ? 1 : 0;
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

/* ===== Address Word Type Ranges =====
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

/* Special address ranges (gap between LA4 and LA2)
 *
 * These are single-word address types between LA4 and LA2 that serve
 * special protocol functions.  They are NOT user-assignable capcodes.
 *
 * Reserved Short:     Reserved for future use (no defined behavior).
 * Info Service:       Under study (no defined behavior yet).
 * Network:            NID system messages.
 *                     Transmitted using Secure (type 0) vector.
 *                     Message contains Service Area ID, Multiplier,
 *                     and Traffic Management Flags.
 * Temporary:          Group messaging (16 temp address slots).
 *                     16 addresses assigned via short instruction vectors.
 *                     Used for temporary group address assignment.
 * Operator Messaging: System messages and change instructions.
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
#define FLEX_ADDR_NETWORK_MIN		0x1F6800U	/* 2,058,240 — NID */
#define FLEX_ADDR_NETWORK_MAX		0x1F77FFU	/* 2,062,335 */
#define FLEX_ADDR_TEMPORARY_MIN		0x1F7800U	/* 2,062,336 — group messaging */
#define FLEX_ADDR_TEMPORARY_MAX		0x1F780FU	/* 2,062,351 */
#define FLEX_ADDR_OPER_MSG_MIN		0x1F7810U	/* 2,062,352 — system messages */
#define FLEX_ADDR_OPER_MSG_MAX		0x1F781FU	/* 2,062,367 */
#define FLEX_ADDR_RSVD_SHORT2_MIN	0x1F7820U	/* 2,062,368 — reserved for future use */
#define FLEX_ADDR_RSVD_SHORT2_MAX	0x1F7FFEU	/* 2,064,382 */

/* Operator Messaging Address sub-types.
 * Base address: 1 1111 0111 1000 0001 0000 (0x1F7810).
 * The 4 LSBs select the function:
 *
 *   0x0 SysMsg(All)     — system message for all pagers (content per BIW)
 *   0x1 SysMsg(Home)    — system message for home area pagers
 *   0x2 SysMsg(Roaming) — system message for roaming pagers
 *   0x3 SysMsg(SSID)    — system message for SSID pagers
 *   0x4 SysMsg(Time)    — time related message for all pagers
 *   0x5-0xD Reserved
 *   0xE SSIDChange      — SSID change instruction.
 *                          Transmits change info to SSID pagers:
 *                            (1) traffic split by TMF in SSID
 *                            (2) new frequencies for SSID coverage zones
 *                          Mandatory for TMF-split systems on roaming
 *                          frames (F0-7 NID, F0-3 SSID-only), 2 cycles
 *                          before + 2 after the change (5 cycles total).
 *                          Must be in same phase as SSID.
 *   0xF SysEvent        — system event notification.
 *                          Pre-alerts pagers that a change will occur
 *                          within the next 4 cycles.
 *                          Used with Short Instruction Vector (type 001).
 *                          Must be in each frame for ≥1 full collapse cycle.
 *                          Events:
 *                            (1) traffic split by SSID TMF
 *                            (2) traffic split by NID TMF
 *                            (3) channel set up instruction changes
 *                            (4) new NID frequency
 *                            (5) new SSID coverage zone frequency
 */
#define FLEX_OPER_MSG_LSB_MASK		0x0FU
#define FLEX_OPER_MSG_SYSMSG_MAX	0x04U	/* LSB 0000–0100: system messages */
#define FLEX_OPER_MSG_SSID_CHANGE	0x0EU	/* LSB 1110: SSID change instruction */
#define FLEX_OPER_MSG_SYS_EVENT		0x0FU	/* LSB 1111: system event notification */

/* System Message sub-type LSB values */
#define FLEX_OPER_MSG_ALL_PAGERS	0x00U	/* all pagers */
#define FLEX_OPER_MSG_HOME		0x01U	/* all pagers in home area */
#define FLEX_OPER_MSG_ROAMING		0x02U	/* all roaming pagers */
#define FLEX_OPER_MSG_SSID		0x03U	/* all SSID pagers */
#define FLEX_OPER_MSG_TIME		0x04U	/* time related message for all pagers */

/* ===== Temporary Address Slots =====
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

/* ===== Short Instruction Type Field =====
 *
 * The 14-bit instruction data from a short instruction vector (type 1)
 * is extracted by FLEX_VEC_INSTR_DATA() from bits 7-20 of the 21-bit word.
 *
 * Within the 14-bit value:
 *   bits 0-2:   i0 i1 i2  (3-bit instruction type)
 *   bits 3-13:  d0-d10    (11-bit instruction-specific data)
 *
 * For Temporary Address (i2i1i0 = 000):
 *   bits 3-9:   f0-f6  (7-bit target frame number for group message)
 *   bits 10-13: a0-a3  (4-bit temp address slot index, 0-15)
 *
 * The system sends each group member's individual address + short
 * instruction vector telling it "listen on temp slot a3-a0 in frame
 * f6-f0."  Multiple capcodes can be assigned to the same slot.
 * Then the group message is sent to that temp address in the
 * designated frame, reaching all assigned pagers.
 *
 * Per the spec: "The assigned Temporary Address is only valid for the
 * Frame in which the Temporary Address is transmitted."
 */
#define FLEX_INSTR_TYPE_MASK		0x07U	/* bits 0-2: instruction type (i2 i1 i0) */
#define FLEX_INSTR_DATA_SHIFT		3	/* instruction-specific data starts at bit 3 */
#define FLEX_INSTR_FRAME_SHIFT		3	/* bits 3-9: target frame (f0-f6) */
#define FLEX_INSTR_FRAME_MASK		0x7FU	/* 7-bit frame number */
#define FLEX_INSTR_SLOT_SHIFT		10	/* bits 10-13: temp address slot (a0-a3) */
#define FLEX_INSTR_SLOT_MASK		0x0FU	/* 4-bit slot index */

/* Instruction type values (i2 i1 i0) from short instruction vector.
 *   000 = Temporary Address assignment: assigns pager to a temp group
 *         slot (a3-a0) for group message delivery in target frame (f6-f0).
 *   001 = System Event Notification: pre-alerts pagers about upcoming
 *         system changes within the next 4 cycles.  Used with Operator
 *         Messaging address 0x1F781F.  Must be transmitted in each frame
 *         for at least 1 full collapse cycle duration.
 *         Events: (1) SSID TMF split, (2) NID TMF split,
 *                 (3) channel setup changes, (4) new NID frequency,
 *                 (5) new SSID coverage zone frequency.
 *   010-111 = Reserved for future use. */
#define FLEX_INSTR_TYPE_TEMP_ADDR	0x00U	/* 000: temp address assignment */
#define FLEX_INSTR_TYPE_SYS_EVENT	0x01U	/* 001: system event notification */

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

/* ===== Network Message Payload =====
 *
 * Network (NID) messages are transmitted using Secure (type 0) vectors.
 * The message payload contains system information fields.
 *
 * First message word layout (21 bits):
 *   bits 0-11:  Service Area ID (12 bits, 0-4095)
 *   bits 12-15: Coverage Zone Count (4 bits, 0-15)
 *   bits 16-20: Traffic Management Flags (5 bits)
 *
 * Traffic Management Flags:
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

/* ===== Operator Messaging Sub-type Classification ===== */

/* Operator messaging sub-type categories */
enum flex_oper_msg_category {
	FLEX_OPER_CAT_SYSMSG,		/* system message (LSB 0x00-0x04) */
	FLEX_OPER_CAT_SSID_CHANGE,	/* SSID change instruction (LSB 0x0E) */
	FLEX_OPER_CAT_SYS_EVENT,	/* system event notification (LSB 0x0F) */
	FLEX_OPER_CAT_RESERVED,		/* reserved (LSB 0x05-0x0D) */
};

/* Classify an operator messaging address word into its category */
static inline enum flex_oper_msg_category flex_oper_msg_category(uint32_t aw)
{
	uint32_t lsb = aw & FLEX_OPER_MSG_LSB_MASK;
	if (lsb <= FLEX_OPER_MSG_SYSMSG_MAX)
		return FLEX_OPER_CAT_SYSMSG;
	if (lsb == FLEX_OPER_MSG_SSID_CHANGE)
		return FLEX_OPER_CAT_SSID_CHANGE;
	if (lsb == FLEX_OPER_MSG_SYS_EVENT)
		return FLEX_OPER_CAT_SYS_EVENT;
	return FLEX_OPER_CAT_RESERVED;
}

/* Human-readable name for operator messaging category */
static inline const char *flex_oper_msg_category_name(enum flex_oper_msg_category cat)
{
	switch (cat) {
	case FLEX_OPER_CAT_SYSMSG:       return "SystemMessage";
	case FLEX_OPER_CAT_SSID_CHANGE:  return "SSIDChangeInstruction";
	case FLEX_OPER_CAT_SYS_EVENT:    return "SystemEventNotification";
	case FLEX_OPER_CAT_RESERVED:     return "Reserved";
	}
	return "?";
}

/* ===== Address Word Group/Temporary Flags =====
 *
 * In the 21-bit address word, bit 20 (MSB) is the group flag and bit 19
 * is the temporary group flag.  These must be extracted BEFORE the
 * remaining bits are classified.
 *
 * Bit layout:
 *   bit 20:    G  — group address flag (1 = group, 0 = individual)
 *   bit 19:    T  — temporary group flag (1 = temporary, 0 = common)
 *   bits 0-18: base address word (classified by range)
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

/* ===== Address Word Type Classification ===== */

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

/* Classify a 21-bit address word by range.
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
#define FLEX_RX_FLAG_PRIORITY	'P'	/* address is in priority section of AF */

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
	if (t1 == FLEX_ADDR_LONG2 && t2 == FLEX_ADDR_LONG4) return "Set2-4";
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
 * LSB values:
 *   LSB 0000 = SysMsg(All), 0001 = SysMsg(Home), 0010 = SysMsg(Roaming),
 *   0011 = SysMsg(SSID), 0100 = SysMsg(Time),
 *   0101-1101 = Reserved,
 *   1110 = SSID Change Instruction, 1111 = System Event Notification */
static inline const char *flex_oper_msg_subtype_name(uint32_t aw)
{
	uint32_t lsb = aw & FLEX_OPER_MSG_LSB_MASK;
	switch (lsb) {
	case FLEX_OPER_MSG_ALL_PAGERS: return "SysMsg(All)";
	case FLEX_OPER_MSG_HOME:       return "SysMsg(Home)";
	case FLEX_OPER_MSG_ROAMING:    return "SysMsg(Roaming)";
	case FLEX_OPER_MSG_SSID:       return "SysMsg(SSID)";
	case FLEX_OPER_MSG_TIME:       return "SysMsg(Time)";
	case FLEX_OPER_MSG_SSID_CHANGE: return "SSIDChange";
	case FLEX_OPER_MSG_SYS_EVENT:  return "SysEvent";
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
	case FLEX_ADDR_NETWORK:    return "NID system info (Secure vector, area/zones/traffic)";
	case FLEX_ADDR_TEMPORARY:  return "temp group slot (16 slots, assigned via short instruction)";
	case FLEX_ADDR_OPER_MSG:   return NULL; /* caller uses flex_oper_msg_subtype_name() */
	case FLEX_ADDR_INFO_SVC:   return "info service (under study)";
	case FLEX_ADDR_RSVD_SHORT: return "reserved for future use";
	default:                   return NULL;
	}
}

/* ===== Address Decode Helpers =====
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
 * inverse formula.
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

	/* Set 2-3 / 2-4: w1 in LA2, w2 in LA3 or LA4
	 * capcode = (w1 - 2064383) + (w2 - 1867776) * 32768 + 2068479 */
	if (w1 >= FLEX_LA2_MIN && w1 <= FLEX_LA2_MAX &&
	    w2 >= FLEX_LA3_MIN && w2 <= FLEX_LA4_MAX)
		return (uint64_t)(w1 - FLEX_LONG_W1_SET23)
		       + (uint64_t)(w2 - FLEX_LONG_W2_SET23) * FLEX_SHORT_ADDR_OFFSET
		       + FLEX_LONG_OFFSET_B;

	return 0; /* invalid combination */
}

/* 21-bit data mask (useful for masking BCH-decoded words) */
#define FLEX_DATA_MASK		((1U << FLEX_BCH_DATA_BITS) - 1)

/* ===== Numeric BCD Character Table =====
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

/* Numeric message overhead bits:
 *
 * Standard/Special word layout:
 *   bits 1-2: K5,K4 (checksum overflow) — 2 overhead bits
 *   bits 3+:  BCD nibbles (4 bits each, LSB-first)
 *   → 5 digits per word (bits 3-6, 7-10, 11-14, 15-18, 19-20+carry)
 *   → max 8 words = 41 chars (37-41 chars need 8 words)
 *
 * Numbered word layout (Fig. 3.10.1.1.2-1):
 *   bits 1-2:  K5,K4 (checksum overflow)
 *   bits 3-8:  N5-N0 message number (0-63, displayed as N+1)
 *   bit  9:    R0 retrieval flag (1=check sequence, 0=retransmission)
 *   bit  10:   S0 special format (1=ID-ROM display format)
 *   bits 11+:  BCD nibbles (a0 a1 a2 a3 ...)
 *   → K5K4(2) + N(6) + R(1) + S(1) = 10 skip bits
 *   → max 8 words = 40 chars (37-40 chars need 8 words) */
#define FLEX_NUM_OVERHEAD_BITS		2
#define FLEX_NUM_NUMBERED_HDR_BITS	8	/* N(6) + R(1) + S(1), excludes K5K4 */
#define FLEX_NUM_NUMBERED_SKIP_BITS	(FLEX_NUM_NUMBERED_HDR_BITS + FLEX_NUM_OVERHEAD_BITS)

/* ===== Secure Message Type Field =====
 *
 * Bits 19-20 of the 1st word (header) on ALL secure fragments.
 * Unlike alpha (where bits 19-20 are R/M on initial, U₀/V₀ on continuation),
 * secure messages always use these bits for the type field t1t0.
 *
 *   t1t0=00: 7-bit Alphanumeric Message data (JIS X 0201)
 *   t1t0=10: binary message data
 *   t1t0=01: data defined separately
 *   t1t0=11: reserved
 *
 * Content starts from 2nd word. For t1t0=00, characters are 7-bit
 * (same as alpha). For t1t0=10, data is raw binary with inverse-fill
 * termination. Registration Acknowledgment uses t1t0=00 with
 * operation code "=" ($3D) in 2nd word bits 1-7. */
#define FLEX_SEC_TYPE_ALPHA		0	/* t1t0=00: alphanumeric */
#define FLEX_SEC_TYPE_SEPARATE		1	/* t1t0=01: defined separately */
#define FLEX_SEC_TYPE_BINARY		2	/* t1t0=10: binary data */
#define FLEX_SEC_TYPE_RESERVED		3	/* t1t0=11: reserved */
#define FLEX_SEC_TYPE_SHIFT		19	/* bit position in header word */
#define FLEX_SEC_TYPE_MASK		0x3	/* 2-bit field */

/* ===== Alpha Message Fragment Flags ===== */

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

/* Continuation/final fragment header:
 * Bits 19-20 are U₀/V₀ (Fragment Control), NOT R/M.
 * Same bit positions, different semantics per fragment type.
 *
 * U₀V₀ values (Enhanced Fragmentation Rules):
 *   00 = Enhanced Fragmentation not supported (default)
 *   01 = Reserved (for a second alternative character mode)
 *   10 = Default character mode (fragment starts at char boundary)
 *   11 = Alternative character mode
 *
 * TODO: Enhanced Fragmentation — when U₀V₀ ≠ 00 on continuation/final
 * "Enhanced Fragmentation Rules").  When U₀V₀ ≠ 00 on continuation/final
 * fragments, the pager uses SI ($0F) / SO ($0E) shift-in/shift-out to
 * switch between default and alternative 7-bit character modes within a
 * fragment.  Key differences from Standard Fragmentation (U₀V₀ = 00):
 *   - U₀V₀ = 10: fragment starts in default character mode.
 *   - U₀V₀ = 11: fragment starts in alternative character mode.
 *   - U₀V₀ = 01: reserved for a second alternative mode.
 *   - NUL ($00) padding replaces ETX ($03) for unused character slots.
 *   - Signature (S) computation must exclude ETX ($03) and NUL ($00)
 *     function characters (spec: "function characters NUL and ETX in
 *     Enhanced Fragmentation are not included for calculation").
 *   - RX must strip NUL padding and interpret SI/SO mode switches.
 *   - TX must set U₀V₀ on each continuation/final fragment header
 *     and pad with NUL instead of ETX.
 * Currently only Standard Fragmentation (U₀V₀ = 00, pure 7-bit ASCII)
 * is implemented.  See also: frame.c encode_alpha_message() TX side,
 * dsp.c flex_rx_decode_phase() RX side. */
#define FLEX_ALPHA_HDR_U_SHIFT		19
#define FLEX_ALPHA_HDR_U_MASK		(1U << 19)
#define FLEX_ALPHA_HDR_V_SHIFT		20
#define FLEX_ALPHA_HDR_V_MASK		(1U << 20)

/* K checksum groups: 3 groups per word (bits 0-7, 8-15, 16-20) */
#define FLEX_ALPHA_K_GRP1_MASK		0xFFU		/* bits 0-7 */
#define FLEX_ALPHA_K_GRP2_SHIFT		8
#define FLEX_ALPHA_K_GRP2_MASK		0xFFU		/* bits 8-15 */
#define FLEX_ALPHA_K_GRP3_SHIFT		16
#define FLEX_ALPHA_K_GRP3_MASK		0x1FU		/* bits 16-20 */

/* HEX/Binary message header word (1st word) bit layout:
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

/* HEX/Binary 2nd word (first fragment only):
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
/* Message type constants — single source of truth.
 * frame.h cannot include flex.h (circular dependency), so the canonical
 * integer values live here as #defines.  flex.h's enum flex_msg_type
 * references these defines to guarantee the values stay in sync. */

#define FLEX_FRAME_MSG_TYPE_AUTO		0
#define FLEX_FRAME_MSG_TYPE_TONE	1
#define FLEX_FRAME_MSG_TYPE_NUMERIC	2
#define FLEX_FRAME_MSG_TYPE_ALPHA	3
#define FLEX_FRAME_MSG_TYPE_HEX		4
#define FLEX_FRAME_MSG_TYPE_INSTRUCTION	5
#define FLEX_FRAME_MSG_TYPE_SHORT	6
#define FLEX_FRAME_MSG_TYPE_SECURE	7
#define FLEX_FRAME_MSG_TYPE_SPECIAL_NUM	8
#define FLEX_FRAME_MSG_TYPE_NUMBERED_NUM 9
#define FLEX_FRAME_MSG_TYPE_NUMBERED_SPECIAL 10

/* ===== Sync Codes =====
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
 *   - The spec bit pattern (LSB-left as transmitted)
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
 * ERS uses the same BS+A+BS_inv+A_inv structure as S1.
 * The receiver detects Ar through the normal sync detector (which always
 * returns the inv upper-16).  When detected, the pager must re-synchronize
 * its frame timing — this is NOT a data frame, no FIW/S2/DATA follows. */
#define FLEX_SYNC_AR		0x34DF

/* FLEX sync marker: the middle 32 bits of the 64-bit sync word.
 * The 64-bit sync is AAAA:BBBBBBBB:CCCC
 * where BBBBBBBB = 0xA6C6AAAA and AAAA ^ CCCC = 0xFFFF. */
#define FLEX_SYNC_MARKER	0xA6C6AAAAul

/* S2 (Sync Part 2) C pattern: 16 decoded bits, identical across all modes.
 * Spec Tables 3.2-1 through 3.2-4: C = 1110110110000100 = 0xED84
 * inv.C = 0001001001111011 = 0x127B */
#define FLEX_S2_C		0xED84
#define FLEX_S2_C_INV		0x127B

/* ===== Modulation Type ===== */

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
	int		sequence_num;		/* N field: message number (0-63), or -1 to
					 * disable numbering (R=0, N not set).
					 * When R=1, pager checks
					 * numbered messages; R=0 messages are excluded
					 * from message number order checking.
					 * For fragmented messages, all fragments share
					 * the same N to identify the fragment stream. */
	const char	*source_id;		/* source indication (NULL = none) */
	int		short_msg_idx;		/* short message index, -1 = N/A */
	int		blocking_length;	/* HEX/Binary B field: bits per character.
					 * 1-15 = that many bits, 0 = 16 bits.
					 * Default 1 (raw bits). */
	int		mail_drop;		/* M flag:
					 * 0 = ordinary message (default)
					 * 1 = can be handled separately */
	int		phase;			/* phase override: -1=auto (default), 0=A, 1=B, 2=C, 3=D */

	/* Fragment state.
	 * Set by flex_fragment_queue() for multi-fragment messages.
	 * fragment_index=0, total_fragments=0 means unfragmented. */
	int		fragment_index;		/* 0-based index within message */
	int		total_fragments;	/* total fragment count (0 = not fragmented) */

	/* secure / numbered numeric per-message fields */
	int		secure_subtype;		/* t1t0 pager-side type tag (0-3).
					 * Independent of wire encoding. */
	int		secure_encoding;	/* wire encoding: 0=7-bit alpha, 1=raw binary */
	int		numbered_r;		/* R flag for numbered numeric (default 1,
					 * retransmission scheduler sets 0) */
	int		numbered_s;		/* S flag for numbered numeric: set from msg_type
					 * (0 for nnumeric, 1 for nspecial) */
	int		numbered_msgnum;	/* N field for numbered numeric, -1 = auto */

	/* Dynamic R flags for retransmission scheduling.
	 * R=1 on initial transmission, R=0 on retransmissions.
	 * numbered_r (above) already serves numbered numeric messages. */
	int		alpha_r_flag;		/* R flag for alpha messages (0 or 1, default 1) */
	int		hex_r_flag;		/* R flag for hex/binary messages (0 or 1, default 1) */
} flex_frame_msg_t;

/* Frame encoding parameters */
typedef struct flex_frame_params {
	uint32_t	cycle;			/* FIW cycle (0-14) */
	uint32_t	frame;			/* FIW frame (0-127) */
	uint32_t	roaming;		/* FIW roaming flag n */
	int		collapse;		/* BIW1 collapse value (0-7) */
	int		carry_on;		/* BIW1 carry-on (0-3 frames) */
	int		biw_time;		/* include BIW3/BIW4 time broadcast */
	uint32_t	local_id;		/* BIW2 local ID (9 bits, 0-511) */
	uint32_t	coverage_id;		/* BIW2 coverage zone (5 bits, 0-31) */
	uint32_t	country_code;		/* SSID2 country code (10 bits, ITU-T E.212) */
	uint32_t	tmf;			/* SSID2 traffic management flags (4 bits) */
	int		timezone_code;		/* SysInfo timezone zone code (0-31, -1=none) */
	int		bitrate;		/* bit rate: 1600, 3200, or 6400 (bps, not baud) */
	int		modulation_type;	/* FLEX_MOD_2FSK or FLEX_MOD_4FSK */
	int		charset;		/* 0 = ASCII, 1 = KANJI */
	int		single_phase;		/* 1 = force single-phase output (for
						 * network mode per-phase encoding) */

	/* Multiple transmission (subframe repeat).
	 *
	 * num_transmissions: 1 (default), 2, 3, or 4.
	 *   When >1, FIW r=1 and [t1,t0] encode the count.
	 *   The frame is divided into N subframes of equal size.
	 *
	 * td_collapse: -1 = use system collapse (default).
	 *   5, 6, or 7 = TD Collapse cycle override.
	 *   When set, FIW [t3,t2] encode the override value.
	 *   TD Collapse takes priority over System Collapse.
	 *
	 * subframe_index: which subframe (0..N-1) this encoding
	 *   represents.  The scheduler sets this per transmission.
	 *   Word numbering within the subframe starts at 0.
	 *   Word 0 always contains the Block Information Word. */
	int		num_transmissions;	/* 1, 2, 3, or 4 */
	int		td_collapse;		/* -1=system, 5/6/7=override */
	int		subframe_index;		/* 0..num_transmissions-1 */

	/* BIW Channel Setup (A-type 0x06) */
	int		chan_setup_enabled;	/* 1 = emit channel setup BIW, default 0 */
} flex_frame_params_t;

/* ===== Phase Multiplexing ===== */

/* Maximum phases: 2 at 3200 bps (2FSK or 4FSK), 4 at 6400 bps */
#define FLEX_MAX_PHASES		4

/* Per-phase channel data.
 *
 * Each phase is an independent channel carrying 88 words (11 blocks × 8).
 * After reception, each word goes through BCH(31,21) error correction
 * and even parity check.  The result is stored per word:
 *   words[i]  — 21-bit info on success, raw 32-bit codeword on failure
 *   status[i] — BCH decode outcome for this word
 *
 * RX reception context records the conditions under which this phase
 * was received (phase assignment). */
typedef struct flex_phase_data {
	uint32_t		words[FLEX_WORDS_PER_FRAME];
	enum flex_word_status	status[FLEX_WORDS_PER_FRAME];
	int			word_count;	/* TX: actual words used */
	int			idle_count;	/* RX: idle words seen (all-0s or all-1s),
					 * used to detect shortened frames */

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
 * Phase data words are interleaved in the output.
 * Returns bytes written to buffer, or 0 on error. */
size_t flex_encode_frame_phased(const flex_phase_data_t *phases, int num_phases,
				const flex_frame_params_t *params,
				uint8_t *buffer, size_t buffer_size,
				int *error);

/* Auto-detect message type from content */
int flex_detect_msg_type(const char *message, int length);

/* Validate capcode (returns 1 if valid, 0 if invalid) */
int flex_capcode_valid(uint64_t capcode);

/* Group address encoding */
uint32_t flex_encode_group_address(uint64_t group_capcode, int is_temporary);

/* Temporary address assignment */
uint32_t flex_encode_temp_address(uint64_t capcode, uint64_t temp_addr);

/* Network address encoding.
 * Encodes a network (NID) address word from the raw address value.
 * addr_offset: offset within the network range (0 to FLEX_ADDR_NETWORK_MAX - FLEX_ADDR_NETWORK_MIN).
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid offset. */
uint32_t flex_encode_network_address(uint32_t addr_offset);

/* Network message payload encoding.
 * Encodes the first message word containing Service Area ID, Coverage
 * Zone Count, and Traffic Management Flags.
 * Returns the encoded 32-bit BCH codeword. */
uint32_t flex_encode_network_payload(uint32_t area_id, uint32_t coverage_zones,
				     uint32_t traffic_flags);

/* Operator messaging address encoding.
 * Encodes an operator messaging address word from the sub-type LSB (0x00-0x0F).
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid sub-type. */
uint32_t flex_encode_oper_msg_address(uint32_t subtype_lsb);

/* Individual encoding functions (exposed for testing) */
uint32_t flex_encode_word(uint32_t dw);
uint32_t flex_word_checksum(uint32_t dw);
uint32_t reverse_bits32(uint32_t v);
void flex_interleave_block(uint32_t block_num, uint32_t *frame_words);

/* Fill a phase's word array with idle pattern.
 * For 4FSK modes, LSB phases get all-zeros instead of alternating. */
void flex_fill_idle_phase(uint32_t *words, int phase_index,
			  int mod_type, int bitrate);
uint32_t flex_create_fiw(uint32_t cycle, uint32_t frame, uint32_t n,
			 uint32_t r, uint32_t t);
uint32_t flex_create_biw1(uint32_t prio, uint32_t e_biw,
			  uint32_t s_vfield, uint32_t carry,
			  uint32_t collapse);
uint32_t flex_create_biw2(uint32_t local_id, uint32_t coverage_id);
uint32_t flex_create_biw3(uint32_t year, uint32_t month, uint32_t day);
uint32_t flex_create_biw4(uint32_t hour, uint32_t minute, uint32_t second);
uint32_t flex_create_biw_ssid2(uint32_t country_code, uint32_t tmf);
uint32_t flex_create_biw_sysinfo(uint32_t a_type, uint32_t info);

/* Generate ERS (Emergency Re-Synchronization) burst into buffer.
 * ERS is a standalone re-sync burst, separate from data frames.
 * Each ERS cycle = BS(2) + AR(4) + BS_inv(2) + AR_inv(4) = 12 bytes.
 * Returns bytes written (cycles * 12), or 0 on error. */
size_t flex_generate_ers(uint8_t *buffer, size_t buffer_size, int cycles);

/* Generate a POCSAG idle batch for POCSAG mixing.
 * Produces: preamble (18 words of 0xAAAAAAAA) + 1 batch (sync + 16 idle codewords).
 * Returns bytes written, or 0 on error. */
size_t flex_generate_pocsag_idle(uint8_t *buffer, size_t buffer_size);

/* ===== Split Sync/Data Encoding =====
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

/* Validate message payload for a given type before enqueuing.
 * Returns 0 if valid, or a negative FLEX_ERR_* code on failure.
 * This catches encoding errors early (bad BCD chars, invalid hex,
 * empty instruction payload, etc.) so they never enter the queue
 * and poison frame encoding. */
int flex_msg_validate(int msg_type, const char *data, int data_length,
		      int secure_encoding);

#endif /* FLEX_FRAME_H */
