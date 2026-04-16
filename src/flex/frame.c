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
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "../liblogging/logging.h"
#include "frame.h"

/* Optional per-message configuration (for mail drop flag support). */
struct flex_msg_config {
	uint8_t mail_drop;	/* 0 or 1 */
	int	fragment_index;	/* 0-based fragment index */
	int	total_fragments;/* total count (0 = not fragmented) */
	int	blocking_length;/* HEX/Binary B field: bits/char (1-15, 0=16) */
	int	alpha_r_flag;	/* passed through from flex_frame_msg_t */
	int	hex_r_flag;	/* passed through from flex_frame_msg_t */
	uint32_t precomputed_sig; /* whole-message signature for fragmented alpha.
				   * 0xFFFFFFFF = not set (compute from this fragment).
				   * 0x00-0x7F = use this value directly. */
};

/*
 * Synchronization patterns.
 *
 * All bit patterns below are from the spec tables, presented as in the
 * original tables with "LSB on the left transmitted first".
 *
 * S1 structure (144 bits, always at 1600 baud / 2-level FSK):
 *   BS1(32) + A(32) + B(16) + A_inv(32) + FIW(32)
 *
 * After S1+FIW, the transmitter switches to the frame's target
 * speed/modulation (if different from 1600/2FSK).
 *
 * S2 structure (at target data rate, always 25 ms):
 *   BS2(N) + C(16) + BS2_inv(N) + C_inv(16)
 *
 * === 1600bps/2-level frame speed ===
 *   BS1       1010101010101010 1010101010101010
 *   A (A1)    0111100011110011 0101100100111001
 *   B         0101010101010101
 *   inv.A(A1) 1000011100001100 1010011011000110
 *   Frame Info iiiiiiiiiiiiiiii iiiiiipppppppppppp
 *   BS2       1010
 *   C         1110110110000100
 *   inv.BS2   0101
 *   inv.C     0001001001111011
 *
 * === 3200bps/2-level frame speed ===
 *   BS1       1010101010101010 1010101010101010
 *   A (A2)    1000010011100111 0101100100111001
 *   B         0101010101010101
 *   inv.A(A2) 0111101100011000 1010011011000110
 *   Frame Info iiiiiiiiiiiiiiii iiiiiipppppppppppp
 *   BS2       1010101010101010 10101010
 *   C         1110110110000100
 *   inv.BS2   0101010101010101 01010101
 *   inv.C     0001001001111011
 *
 * === 3200bps/4-level frame speed ===
 *   BS1       1010101010101010 1010101010101010
 *   A (A3)    0100111110010111 0101100100111001
 *   B         0101010101010101
 *   inv.A(A3) 1011000001101000 1010011011000110
 *   Frame Info iiiiiiiiiiiiiiii iiiiiipppppppppppp
 *   BS2       101010101010 (symbol)
 *   C         1110110110000100 (decoded value)
 *   inv.BS2   010101010101 (symbol)
 *   inv.C     0001001001111011 (decoded value)
 *
 * === 6400bps/4-level frame speed ===
 *   BS1       1010101010101010 1010101010101010
 *   A (A4)    0010000101011111 0101100100111001
 *   B         0101010101010101
 *   inv.A(A4) 1101111010100000 1010011011000110
 *   Frame Info iiiiiiiiiiiiiiii iiiiiipppppppppppp
 *   BS2       1010101010101010 1010101010101010 (symbol)
 *   C         1110110110000100 (decoded value)
 *   inv.BS2   0101010101010101 0101010101010101 (symbol)
 *   inv.C     0001001001111011 (decoded value)
 *
 * === "A" binary pattern ===
 *   A1  1600bps/2-level     0111100011110011 0101100100111001
 *   A2  3200bps/2-level     1000010011100111 0101100100111001
 *   A3  3200bps/4-level     0100111110010111 0101100100111001
 *   A4  6400bps/4-level     0010000101011111 0101100100111001
 *   A5  Reserved            1101110101001011 0101100100111001
 *   A6  Reserved            0001011000111011 0101100100111001
 *   A7  Reserved            1011001110000011 0101100100111001
 *   A8  Reserved            0110001101000001 0101100100111001
 *   A9  Reserved            0001101111100010 0101100100111001
 *   A10 Reserved            0010110010000110 0101100100111001
 *   A11 Reserved            1010010111101000 0101100100111001
 *   A12 Reserved            1001001010001100 0101100100111001
 *   A13 Reserved            0110111010011000 0101100100111001
 *   A14 Reserved            1011111001011010 0101100100111001
 *   A15 Reserved            1111000110011101 0101100100111001
 *   Ar  Re-synchronization  1100101100100000 0101100100111001
 *   * "A" = BCH(31,21) code + even parity, transmitted in reverse order
 *
 * _inv variants are bitwise inversions for polarity detection.
 */

/* S1 bit sync patterns */
static const uint8_t sync_bs[]        = {0xAA, 0xAA};
static const uint8_t sync_bs_inv[]    = {0x55, 0x55};
static const uint8_t sync_bs1[]       = {0xAA, 0xAA, 0xAA, 0xAA};

/* A codes — active speeds (MSB-first byte order) */
static const uint8_t sync_a1[]        = {0x78, 0xF3, 0x59, 0x39}; /* 1600/2FSK */
static const uint8_t sync_a1_inv[]    = {0x87, 0x0C, 0xA6, 0xC6};
static const uint8_t sync_a2[]        = {0x84, 0xE7, 0x59, 0x39}; /* 3200/2FSK */
static const uint8_t sync_a2_inv[]    = {0x7B, 0x18, 0xA6, 0xC6};
static const uint8_t sync_a3[]        = {0x4F, 0x97, 0x59, 0x39}; /* 3200/4FSK */
static const uint8_t sync_a3_inv[]    = {0xB0, 0x68, 0xA6, 0xC6};
static const uint8_t sync_a4[]        = {0x21, 0x5F, 0x59, 0x39}; /* 6400/4FSK */
static const uint8_t sync_a4_inv[]    = {0xDE, 0xA0, 0xA6, 0xC6};

/* A codes — reserved
 * Not used by the encoder (no defined frame speed), but kept here
 * for completeness. The 16-bit outer codes FLEX_SYNC_A5..A15 are
 * defined in frame.h for RX detection and logging. */
static const uint8_t sync_a5[]  __attribute__((unused)) = {0xDD, 0x4B, 0x59, 0x39};
static const uint8_t sync_a6[]  __attribute__((unused)) = {0x16, 0x3B, 0x59, 0x39};
static const uint8_t sync_a7[]  __attribute__((unused)) = {0xB3, 0x83, 0x59, 0x39};
static const uint8_t sync_a8[]  __attribute__((unused)) = {0x63, 0x41, 0x59, 0x39};
static const uint8_t sync_a9[]  __attribute__((unused)) = {0x1B, 0xE2, 0x59, 0x39};
static const uint8_t sync_a10[] __attribute__((unused)) = {0x2C, 0x86, 0x59, 0x39};
static const uint8_t sync_a11[] __attribute__((unused)) = {0xA5, 0xE8, 0x59, 0x39};
static const uint8_t sync_a12[] __attribute__((unused)) = {0x92, 0x8C, 0x59, 0x39};
static const uint8_t sync_a13[] __attribute__((unused)) = {0x6E, 0x98, 0x59, 0x39};
static const uint8_t sync_a14[] __attribute__((unused)) = {0xBE, 0x5A, 0x59, 0x39};
static const uint8_t sync_a15[] __attribute__((unused)) = {0xF1, 0x9D, 0x59, 0x39};

/* Ar: ERS re-synchronization */
static const uint8_t sync_ar[]        = {0xCB, 0x20, 0x59, 0x39};
static const uint8_t sync_ar_inv[]    = {0x34, 0xDF, 0xA6, 0xC6};

/* B: baud/level indicator */
static const uint8_t sync_b[]         = {0x55, 0x55};

/*
 * S2 (Sync Part 2) component constants.
 *
 * S2 structure: BS2 + C + inv.BS2 + inv.C
 * S2 is always 25 ms at the data symbol rate.
 *
 * C pattern: 16 decoded bits, identical across all modes.
 *   2FSK: transmitted as 16 symbols (1 bit/symbol)
 *   4FSK: transmitted as 8 symbols (2 bits/symbol = 1 dibit each)
 *
 * BS2: alternating comma pattern for PLL lock at the new symbol rate.
 *   2FSK: alternating bits 1,0,1,0... (1 bit/symbol)
 *   4FSK: alternating dibits 10,00,10,00... (2 bits/symbol)
 *         This produces full-deviation comma on the channel.
 *
 * C and inv.C values are defined in frame.h as FLEX_S2_C / FLEX_S2_C_INV.
 */

/* Copy a sync pattern into the output buffer and advance the pointer. */
#define EMIT_SYNC(ptr, pattern) \
	do { \
		memcpy((ptr), (pattern), sizeof((pattern))); \
		(ptr) += sizeof((pattern)); \
	} while (0)

/* Write a 32-bit codeword big-endian into the buffer and advance. */
#define EMIT_WORD(ptr, word) \
	do { \
		uint32_t _w = (word); \
		(ptr)[0] = (_w >> 24) & 0xFF; \
		(ptr)[1] = (_w >> 16) & 0xFF; \
		(ptr)[2] = (_w >>  8) & 0xFF; \
		(ptr)[3] =  _w        & 0xFF; \
		(ptr) += 4; \
	} while (0)

/* ===== Bit Manipulation Utilities ===== */

/*
 * Even parity of a 32-bit word.
 * Returns 1 if number of set bits is even, 0 if odd.
 *
 * Uses the nibble parity lookup 0x6996 (each bit indicates parity
 * of the corresponding nibble value 0-15).
 */
static uint8_t word_parity(uint32_t x)
{
	x ^= x >> 16;
	x ^= x >> 8;
	x ^= x >> 4;
	x &= 0xF;
	/* Nibble parity table: bit N is 1 if N has even parity */
	return (0x6996 >> x) & 1;
}

/*
 * Reverse all 32 bits of a word.
 * Standard bit-reversal algorithm from "Bit Twiddling Hacks" (public domain).
 */
uint32_t reverse_bits32(uint32_t v)
{
	v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);
	v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);
	v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
	v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
	v = (v >> 16) | (v << 16);
	return v;
}

/* ===== BCH(31,21) Encoding ===== */

/*
 * Encode a 21-bit data word into a 32-bit FLEX codeword.
 *
 * Input:  21 data bits in the upper 21 bits of dw (bits 11-31).
 * Output: [21 data][10 BCH ECC][1 even parity] = 32 bits.
 */
uint32_t flex_encode_word(uint32_t dw)
{
	uint32_t data, dividend, ecc, code31, parity;
	int i;

	data = dw >> (FLEX_CODEWORD_BITS - FLEX_BCH_DATA_BITS);
	dividend = data << FLEX_BCH_ECC_BITS;

	/* Polynomial long division to compute BCH remainder */
	for (i = FLEX_BCH_DATA_BITS + FLEX_BCH_ECC_BITS - 1; i >= FLEX_BCH_ECC_BITS; i--) {
		if ((dividend >> i) & 1)
			dividend ^= FLEX_BCH_POLY << (i - FLEX_BCH_ECC_BITS);
	}

	ecc = dividend & ((1U << FLEX_BCH_ECC_BITS) - 1);
	code31 = (data << FLEX_BCH_ECC_BITS) | ecc;
	parity = word_parity(code31);

	return (code31 << FLEX_BCH_PARITY_BITS) | parity;
}

/* ===== Word Checksum ===== */

/*
 * Compute and insert the 4-bit checksum into bits 0-3 of a data word.
 *
 * The checksum is the ones' complement of the sum of nibbles at
 * bit positions 4-7, 8-11, 12-15, 16-19, and bit 20.
 */
uint32_t flex_word_checksum(uint32_t dw)
{
	uint32_t a, b, c, d, e, csum;

	a = (dw >>  4) & 0xF;
	b = (dw >>  8) & 0xF;
	c = (dw >> 12) & 0xF;
	d = (dw >> 16) & 0xF;
	e = (dw >> 20) & 0x1;
	csum = (~(a + b + c + d + e)) & 0xF;

	return dw | csum;
}

/*
 * Verify the 4-bit checksum of a 21-bit data word (§3.5.1).
 * Returns 1 if valid, 0 if invalid.
 */
int flex_verify_word_checksum(uint32_t dw)
{
	uint32_t sum;

	sum  = (dw      ) & 0xF;
	sum += (dw >>  4) & 0xF;
	sum += (dw >>  8) & 0xF;
	sum += (dw >> 12) & 0xF;
	sum += (dw >> 16) & 0xF;
	sum += (dw >> 20) & 0x1;

	return (sum & 0xF) == 0xF;
}

/* ===== Address Encoding ===== */

static int is_short_address(uint64_t capcode)
{
	return (capcode >= FLEX_SHORT_ADDR_MIN && capcode <= FLEX_SHORT_ADDR_MAX);
}

static int is_long_address(uint64_t capcode)
{
	return (capcode >= FLEX_LONG_ADDR_MIN && capcode <= FLEX_LONG_ADDR_MAX);
}

static int is_capcode_valid(uint64_t capcode, int *is_long)
{
	*is_long = 0;
	if (is_short_address(capcode))
		return 1;
	if (is_long_address(capcode)) {
		*is_long = 1;
		return 1;
	}
	/* Special addresses in the gap between short and long ranges
	 * (Network, Temporary, OperMsg, InfoSvc, Reserved Short).
	 * Encoded like short addresses (capcode + SHORT_ADDR_OFFSET). */
	if (flex_capcode_is_special(capcode))
		return 1;
	return 0;
}

/*
 * Encode a short capcode (FLEX_SHORT_ADDR_MIN–FLEX_SHORT_ADDR_MAX) into a single address word.
 *
 * The 21-bit information word d₀-d₂₀ holds the address value
 * (d₀=LSB, d₂₀=MSB).  The address is transmitted LSB first.
 */
static uint32_t encode_short_address(uint32_t capcode)
{
	uint32_t dw = (capcode + FLEX_SHORT_ADDR_OFFSET) & FLEX_DATA_MASK;
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Encode a long capcode into two address words.
 *
 * Long addresses consist of 2 words (1st word d₀-d₂₀, 2nd word e₀-e₂₀).
 * The combination of Long Address types determines the set:
 *   Set 1-2: LA1 + LA2    Set 1-3: LA1 + LA3    Set 1-4: LA1 + LA4
 *   Set 2-3: LA2 + LA3    Set 2-4: LA2 + LA4
 *
 * Returns 0 on success, -1 if capcode is out of range.
 */
static int encode_long_address(uint64_t capcode, uint32_t words[2])
{
	uint64_t result;
	uint32_t w1, w2;

	if (capcode >= FLEX_LONG_SET12_MIN && capcode <= FLEX_LONG_SET12_MAX) {
		/* Sets 1-2 */
		result = capcode - FLEX_LONG_OFFSET_A;
		w1 = (result % FLEX_SHORT_ADDR_OFFSET) + 1;
		w2 = FLEX_LONG_W2_SET12 - (result / FLEX_SHORT_ADDR_OFFSET);
	} else if (capcode >= FLEX_LONG_SET34_MIN && capcode <= FLEX_LONG_SET34_MAX) {
		/* Sets 1-3 and 1-4 */
		result = capcode - FLEX_LONG_OFFSET_A;
		w1 = (result % FLEX_SHORT_ADDR_OFFSET) + 1;
		w2 = (result / FLEX_SHORT_ADDR_OFFSET) + FLEX_LONG_W2_SET34;
	} else if (capcode >= FLEX_LONG_SET23_MIN && capcode <= FLEX_LONG_SET23_MAX) {
		/* Sets 2-3 and 2-4 */
		result = capcode - FLEX_LONG_OFFSET_B;
		w1 = (result % FLEX_SHORT_ADDR_OFFSET) + FLEX_LONG_W1_SET23;
		w2 = (result / FLEX_SHORT_ADDR_OFFSET) + FLEX_LONG_W2_SET23;
	} else {
		return -1;
	}

	words[0] = flex_encode_word(reverse_bits32(w1));
	words[1] = flex_encode_word(reverse_bits32(w2));
	return 0;
}

int flex_capcode_valid(uint64_t capcode)
{
	int is_long;
	return is_capcode_valid(capcode, &is_long);
}

/* ===== Frame/Block Information Words ===== */

/*
 * Frame Information Word.
 *
 * cycle: Cycle number (0-14)
 * frame: Frame number (0-127)
 * n:     Roaming flag (0/1)
 * r:     Repeat transmission flag (0/1)
 * t:     Low traffic flags (0-15)
 */
uint32_t flex_create_fiw(uint32_t cycle, uint32_t frame, uint32_t n,
			 uint32_t r, uint32_t t)
{
	uint32_t dw = 0;
	dw |= (cycle & FLEX_FIW_CYCLE_MASK)   << FLEX_FIW_CYCLE_SHIFT;
	dw |= (frame & FLEX_FIW_FRAME_MASK)   << FLEX_FIW_FRAME_SHIFT;
	dw |= (n     & 0x01)                  << FLEX_FIW_ROAMING_SHIFT;
	dw |= (r     & 0x01)                  << FLEX_FIW_REPEAT_SHIFT;
	dw |= (t     & FLEX_FIW_TRAFFIC_MASK) << FLEX_FIW_TRAFFIC_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word 1.
 *
 * prio:     Priority address count (0-15)
 * e_biw:    End of BIW / start of address field (0-3)
 * s_vfield: Start of vector field offset (1-63)
 * carry:    Carry-on to next frame (0-3)
 * collapse: Decode schedule (0-7)
 */
uint32_t flex_create_biw1(uint32_t prio, uint32_t e_biw,
			  uint32_t s_vfield, uint32_t carry,
			  uint32_t collapse)
{
	uint32_t dw = 0;
	dw |= (prio     & FLEX_BIW1_PRIO_MASK)     << FLEX_BIW1_PRIO_SHIFT;
	dw |= (e_biw    & FLEX_BIW1_ASTART_MASK)   << FLEX_BIW1_ASTART_SHIFT;
	dw |= (s_vfield & FLEX_BIW1_VSTART_MASK)   << FLEX_BIW1_VSTART_SHIFT;
	dw |= (carry    & FLEX_BIW1_CARRY_MASK)     << FLEX_BIW1_CARRY_SHIFT;
	dw |= (collapse & FLEX_BIW1_COLLAPSE_MASK)  << FLEX_BIW1_COLLAPSE_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Vector Words ===== */

/*
 * Alphanumeric vector (V=101).
 *
 * Bit layout (21-bit data word):
 *   bits 0-3:   x  checksum (4-bit nibble sum)
 *   bits 4-6:   V  type = 101 (alphanumeric)
 *   bits 7-13:  b  message start word offset (7 bits, 0-87)
 *   bits 14-20: n  message word count (7 bits)
 *
 * Example: msg at word 10, 3 words → dw = x:101:0001010:0000011
 */
static uint32_t create_alpha_vector(uint32_t msg_start, uint32_t msg_words)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_ALPHA & FLEX_VEC_TYPE_MASK) << FLEX_VEC_TYPE_SHIFT;
	dw |= (msg_start              & FLEX_VEC_START_MASK) << FLEX_VEC_START_SHIFT;
	dw |= (msg_words              & FLEX_VEC_LEN_MASK)   << FLEX_VEC_LEN_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Secure vector (V=000).
 *
 * Identical layout to alphanumeric vector but with type V=000.
 *   bits 0-3:   x  checksum (4-bit nibble sum)
 *   bits 4-6:   V  type = 000 (secure)
 *   bits 7-13:  b  message start word offset (7 bits, 0-87)
 *   bits 14-20: n  message word count (7 bits)
 */
static uint32_t create_secure_vector(uint32_t msg_start, uint32_t msg_words)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_SECURE & FLEX_VEC_TYPE_MASK) << FLEX_VEC_TYPE_SHIFT;
	dw |= (msg_start               & FLEX_VEC_START_MASK) << FLEX_VEC_START_SHIFT;
	dw |= (msg_words               & FLEX_VEC_LEN_MASK)   << FLEX_VEC_LEN_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Numeric vector (V=011/100/111).
 *
 * Numeric vectors differ from alpha/hex: the upper bits carry the
 * K checksum instead of extra length bits.
 *
 * Bit layout (21-bit data word):
 *   bits 0-3:   x    checksum (4-bit nibble sum)
 *   bits 4-6:   V    type (011=standard, 100=special, 111=numbered)
 *   bits 7-13:  b    message start word offset (7 bits, 0-87)
 *   bits 14-16: n    message word count (3 bits, 1-8)
 *   bits 17-20: K3-0 lower 4 bits of message K checksum
 *
 * The K checksum is 6 bits total: K5-K4 are in the first message
 * word (bits 0-1), K3-K0 are here in the vector word.
 * See encode_numeric_message() for the full K algorithm.
 */
static uint32_t create_numeric_vector(uint32_t msg_start,
				      uint32_t msg_words, uint32_t kbit,
				      uint32_t vector_type)
{
	/* Per ARIB STD-43A §3.9.1, the n field encodes word_count - 1:
	 *   "n: Indicates the number of words in the message =
	 *    1 + 1·n₀ + 2·n₁ + 4·n₂"
	 * So the 3-bit field value = msg_words - 1 (range 0-7 for 1-8 words). */
	uint32_t n_field = (msg_words > 0) ? msg_words - 1 : 0;
	uint32_t dw = 0;
	dw |= (vector_type & FLEX_VEC_TYPE_MASK)  << FLEX_VEC_TYPE_SHIFT;
	dw |= (msg_start   & FLEX_VEC_START_MASK) << FLEX_VEC_START_SHIFT;
	dw |= (n_field      & FLEX_VEC_NUM_LEN_MASK) << FLEX_VEC_LEN_SHIFT;
	dw |= (kbit        & FLEX_VEC_NUM_KBIT_MASK) << FLEX_VEC_NUM_KBIT_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Block Interleaving ===== */

/*
 * Interleave one block: transpose 8 words × 32 bits into 32 bytes × 8 bits.
 * Each output byte contains one bit from each of the 8 words, so burst
 * errors affect at most 1 bit per codeword (BCH can correct up to 2).
 */
void flex_interleave_block(uint32_t block_num, uint32_t *frame_words)
{
	uint32_t src[FLEX_WORDS_PER_BLOCK];
	uint8_t  dst[FLEX_CODEWORD_BITS];
	uint32_t i;

	memcpy(src, frame_words + block_num * FLEX_WORDS_PER_BLOCK, sizeof(src));

	for (i = 0; i < FLEX_CODEWORD_BITS; i++) {
		dst[i] =
			((src[0] >> (31 - i)) & 1) << 7 |
			((src[1] >> (31 - i)) & 1) << 6 |
			((src[2] >> (31 - i)) & 1) << 5 |
			((src[3] >> (31 - i)) & 1) << 4 |
			((src[4] >> (31 - i)) & 1) << 3 |
			((src[5] >> (31 - i)) & 1) << 2 |
			((src[6] >> (31 - i)) & 1) << 1 |
			((src[7] >> (31 - i)) & 1) << 0;
	}

	memcpy(frame_words + block_num * FLEX_WORDS_PER_BLOCK, dst, sizeof(dst));
}

/* De-interleave one 8-word block within a frame.
 * Inverse of flex_interleave_block: transposes the 8x32 bit matrix
 * back from column-major (interleaved) to row-major (raw codewords). */
void flex_deinterleave_block(uint32_t block_num, uint32_t *frame_words)
{
	uint8_t src[FLEX_CODEWORD_BITS];
	uint32_t out[FLEX_WORDS_PER_BLOCK];
	int i, w;

	memcpy(src, frame_words + block_num * FLEX_WORDS_PER_BLOCK, sizeof(src));
	memset(out, 0, sizeof(out));

	for (i = 0; i < (int)FLEX_CODEWORD_BITS; i++) {
		for (w = 0; w < (int)FLEX_WORDS_PER_BLOCK; w++) {
			if (src[i] & (1 << (7 - w)))
				out[w] |= (1U << (31 - i));
		}
	}

	memcpy(frame_words + block_num * FLEX_WORDS_PER_BLOCK, out, sizeof(out));
}

/* Compute per-phase Low Traffic Flags for FIW t-field (§3.6).
 *
 * For each phase, checks whether the address field (AF) fits entirely
 * within block 0 (first FLEX_WORDS_PER_BLOCK words).  The AF includes
 * all address words: priority, normal, and tone-only (short=1 word,
 * long=2 words each).  Vectors, message body, and system message
 * content are NOT part of the AF and do not affect this flag.
 *
 * Returns a 4-bit value: t0=phase A, t1=B, t2=C, t3=D.
 *
 * Per §3.6: "These flags are used for indicating Low Traffic and that
 * all addresses have been allocated to block 0 at an earlier stage.
 * When there is Carry On or a change in the Collapse cycle, flags
 * are not set for 1, even when traffic is low." */
uint32_t flex_compute_low_traffic_flags(const int *phase_af_words,
					int num_phases, int biw_count,
					int carry_on, int collapse_changing)
{
	uint32_t flags = 0;
	int p;

	/* §3.6: flags are suppressed when carry-on is active or
	 * a Collapse cycle change is in progress */
	if (carry_on > 0 || collapse_changing)
		return 0;

	/* Check each phase: AF must fit within block 0.
	 * Block 0 = words 0..(FLEX_WORDS_PER_BLOCK-1).
	 * BIW words precede AF, so AF starts at word biw_count.
	 * Low traffic condition: biw_count + af_words <= FLEX_WORDS_PER_BLOCK */
	for (p = 0; p < num_phases && p < FLEX_MAX_PHASES; p++) {
		if (biw_count + phase_af_words[p] <= FLEX_WORDS_PER_BLOCK)
			flags |= (1U << p);
	}

	/* Speed-dependent pairing (§3.6):
	 *   At 3200 bps (2 phases): t3=t2 and t1=t0
	 *   At 1600 bps (1 phase):  t3=t2=t1=t0
	 * The pairing ensures unused phase bits mirror the active ones. */
	switch (num_phases) {
	case 1:
		/* 1600 bps: single phase A — replicate t0 to all bits */
		flags = (flags & 0x01) ? FLEX_FIW_TRAFFIC_MASK : 0;
		break;
	case 2:
		/* 3200 bps: phases A(t0) and C(t1) — pair t1=t0, t3=t2 */
		{
			uint32_t t0 = (flags >> 0) & 1;
			uint32_t t1 = (flags >> 1) & 1;
			flags = t0 | (t0 << 1) | (t1 << 2) | (t1 << 3);
		}
		break;
	default:
		/* 6400 bps (4 phases): each bit independent */
		break;
	}

	return flags & FLEX_FIW_TRAFFIC_MASK;
}

/*
 * Fill a phase's word array with the idle pattern.
 *
 * Idle blocks produce a 1,0 bit pattern at 1600 bps on the channel.
 * For 2FSK this means alternating all-1s and all-0s words.
 * For 4FSK, the MSB phases (A, and C for 6400) get the alternating
 * pattern, while the LSB phases (C for 3200, B and D for 6400) get
 * all-zeros so that the resulting 4-level symbols are only the two
 * extreme levels (±4800 Hz), reproducing the same 1600 bps waveform.
 *
 * Idle patterns by mode:
 *   1600/2FSK, 3200/2FSK: all phases alternate 0xFFFFFFFF / 0x00000000
 *   3200/4FSK: Phase A alternates, Phase C = all zeros
 *   6400/4FSK: Phases A,C alternate, Phases B,D = all zeros
 *
 * phase_index: 0=A, 1=B(6400) or C(3200), 2=C(6400), 3=D(6400)
 * mod_type: FLEX_MOD_2FSK or FLEX_MOD_4FSK
 * bitrate: 1600, 3200, or 6400 (bps)
 */
void flex_fill_idle_phase(uint32_t *words, int phase_index,
			  int mod_type, int bitrate, int collapse)
{
	int w;
	int is_lsb_phase = 0;

	if (mod_type == FLEX_MOD_4FSK) {
		if (bitrate <= 3200) {
			is_lsb_phase = (phase_index == 1);
		} else {
			is_lsb_phase = (phase_index == 1 || phase_index == 3);
		}
	}

	if (is_lsb_phase) {
		for (w = 0; w < FLEX_WORDS_PER_FRAME; w++)
			words[w] = FLEX_IDLE_WORD_2;
	} else {
		for (w = 0; w < FLEX_WORDS_PER_FRAME; w++)
			words[w] = (w % 2 == 0) ? FLEX_IDLE_WORD_1
						 : FLEX_IDLE_WORD_2;
	}

	/* Write a proper idle BIW1 at word 0.
	 * Without this, the raw idle pattern (all-zeros on LSB phases,
	 * all-ones on MSB phases) decodes to an invalid BIW that causes
	 * the receiver to parse garbage addresses.
	 * Idle BIW1: no priority, no extra BIW, voffset=1 (== aoffset),
	 * no carry-on, collapse from config. */
	words[0] = flex_create_biw1(0, 0, 1, 0, (uint32_t)collapse);
}

/* Generate an idle subframe: BIW1 (with correct collapse, no addresses)
 * followed by idle fill pattern.  Used for empty subframe slots in
 * multiple transmission mode.
 *
 * Every subframe slot must have a valid BIW1 at its start — raw idle
 * pattern without BIW1 causes decoders to misparse the slot as
 * containing addresses (the all-1s/all-0s idle words pass BCH but
 * produce garbage capcodes).
 *
 * The BIW1 has voffset == aoffset (no addresses, no vectors, no
 * messages).  Remaining words are filled with the phase-appropriate
 * idle pattern. */
void flex_encode_idle_subframe(uint32_t *words, int sf_words,
			       const flex_frame_params_t *params)
{
	uint32_t biw1;
	int biw_count = 1; /* BIW1 only for idle subframes */
	uint32_t s_vfield = (uint32_t)biw_count; /* no addresses: voffset == aoffset */
	int w;

	/* Fill with idle pattern — only sf_words, not the full 88.
	 * Use the same phase-aware pattern as flex_fill_idle_phase
	 * but limited to the subframe size. */
	{
		int is_lsb_phase = 0;
		if (params->modulation_type == FLEX_MOD_4FSK) {
			if (params->bitrate <= 3200)
				is_lsb_phase = (params->phase_index == 1);
			else
				is_lsb_phase = (params->phase_index == 1 || params->phase_index == 3);
		}
		for (w = 0; w < sf_words; w++) {
			if (is_lsb_phase)
				words[w] = FLEX_IDLE_WORD_2;
			else
				words[w] = (w % 2 == 0) ? FLEX_IDLE_WORD_1 : FLEX_IDLE_WORD_2;
		}
	}

	/* Write BIW1 at position 0: no priority, no extra BIW,
	 * voffset = 1 (right after BIW1), correct collapse */
	biw1 = flex_create_biw1(0, 0, s_vfield, 0, (uint32_t)params->collapse);
	words[0] = biw1;

	/* NOTE: No block interleaving here.  The caller is responsible
	 * for interleaving the full 88-word assembled frame after all
	 * subframes are placed at their slot offsets.  Interleaving
	 * individual subframes would misalign with the frame's 8-word
	 * block boundaries when placed at non-zero offsets. */
}


/* numeric_char_to_flex() removed — use shared flex_num_char_to_bcd() from frame.h */

/* Forward declarations for hex helpers (defined later, needed by encode_secure_message) */
static int is_valid_hex_char(char c);
static uint8_t hex_char_to_nibble(char c);

static int is_valid_numeric_char(char c)
{
	return (c >= '0' && c <= '9') || c == '-' || c == '_' ||
	       c == '[' || c == ']' || c == ' ' || c == 'U' || c == 'u' ||
	       c == FLEX_NUM_BCD_SPARE_CHAR;
}

/* is_valid_numeric_message() removed — dead code, never called.
 * Validation is done inline at the call site. */

/* ===== Message Encoding ===== */

/*
 * Encode alphanumeric message.
 *
 * Characters are packed 3 per 21-bit word (7 bits each).
 * Unused character slots are filled with ETX (0x03).
 * Includes fragment flags (f0f1), signature (S-bit), and checksum (K-bit).
 *
 * Returns the encoded vector word for this message.  Only body words
 * are written to frame_words starting at *fwc_p.  The caller places
 * the vector word in the Vector Field.
 */
static uint32_t encode_alpha_message(uint32_t *frame_words, const char *msg,
				 uint32_t msg_start, uint32_t *fwc_p,
				 int sequence_num,
				 const void *config)
{
	uint32_t msg_word[FLEX_MAX_MSG_WORDS_ALPHA] = {0};
	uint32_t word_idx, fwc;
	size_t len, max_len;
	uint32_t k_sum;
	int shift;
	uint32_t i;

	len = strlen(msg);
	max_len = (len > FLEX_MAX_CHARS_ALPHA) ? FLEX_MAX_CHARS_ALPHA : len;

	/* First message word (header) — see frame.h for bit layout.
	 * K checksum is computed last after all other fields are set.
	 *
	 * F: Message Fragment Number (2 bits, §3.10.1.3).
	 *   F is a modulo 3 message fragment number which increments by 1
	 *   for each of the consecutive fragments.  The first fragment
	 *   starts with "11" and is incremented by 1 modulo 3 for each of
	 *   the subsequent fragments (11, 00, 01, 10, 00, 01, 10, 00, ...).
	 *   The state for "11" after the first fragment is skipped in order
	 *   to prevent it from being mistaken as the first fragment of a
	 *   non-consecutive message.
	 *
	 * C: Message Continued Flag (1 bit).
	 *   The last fragment is indicated by resetting the Message
	 *   Continued Flag (C) to 0.  C=1 means more fragments follow.
	 *
	 * Applies to Alpha (V=101), HEX/Binary (V=010), Secure (V=000).
	 * Numeric messages (V=011, V=100, V=111) cannot be fragmented (§4.2).
	 */
	{
		int frag_idx = 0, frag_total = 0;
		uint32_t f_val;
		int c_val;

		if (config) {
			const struct flex_msg_config *cfg = config;
			frag_idx = cfg->fragment_index;
			frag_total = cfg->total_fragments;
		}

		f_val = flex_fragment_number(frag_idx);
		c_val = (frag_total > 1 && frag_idx < frag_total - 1) ? 1 : 0;

		msg_word[0] = (f_val << FLEX_ALPHA_HDR_F_SHIFT);
		if (c_val)
			msg_word[0] |= FLEX_ALPHA_HDR_C_MASK;
	}

	/* Message number N — present on ALL fragments (identifies the
	 * fragment stream so the receiver can reassemble them).
	 * N is 6 bits (0-63).  Per spec §4.2: "those message numbers which
	 * are newly assigned to a message must be unique numbers so as to
	 * identify fragments for the same message."  The same N value is
	 * used across all fragments of a single long message.
	 *
	 * Per §4.1: when the full length of a message cannot be contained
	 * in one Frame, the individual message must be broken down into
	 * fragments.  The maximum number of words which can be transmitted
	 * for an alpha message is 84 words (85 message words minus 1 header),
	 * giving a maximum of 251 characters per frame [3 chars/word × 84
	 * words - 1 = 251].  Longer messages are transmitted by
	 * Fragmentation. */
	if (sequence_num >= 0) {
		uint32_t n = (uint32_t)(sequence_num % 64);
		msg_word[0] |= (n << FLEX_ALPHA_HDR_N_SHIFT);
	}

	/* R (retrieval) and M (mail drop) — first fragment only.
	 * Bits 19-20 on continuation/final
	 * fragments are U₀/V₀ (reserved), not R/M.
	 * Per spec: "Fields R through S are only transmitted in
	 * the first fragment." */
	{
		int is_initial = 1;
		if (config) {
			const struct flex_msg_config *cfg = config;
			is_initial = (cfg->total_fragments <= 1 ||
				      cfg->fragment_index == 0);
		}
		if (is_initial) {
			if (sequence_num >= 0) {
				if (config) {
					const struct flex_msg_config *cfg = config;
					if (cfg->alpha_r_flag)
						msg_word[0] |= FLEX_ALPHA_HDR_R_MASK;
				} else {
					msg_word[0] |= FLEX_ALPHA_HDR_R_MASK;
				}
			}
			if (config) {
				const struct flex_msg_config *cfg = config;
				if (cfg->mail_drop)
					msg_word[0] |= FLEX_ALPHA_HDR_M_MASK;
			}
		}
		/* TODO: Enhanced Fragmentation — set U₀/V₀ bits on
		 * continuation/final fragments.  When !is_initial and the message
		 * uses SI/SO character mode switching, set bits 19-20:
		 *   U₀V₀ = 10 if fragment starts in default char mode,
		 *   U₀V₀ = 11 if fragment starts in alternative mode.
		 * Currently always 00 (Standard Fragmentation). */
	}

	/* Pack 7-bit ASCII characters, 3 per word.
	 *
	 * Initial fragment (F=11): first data word has signature in
	 * bits 0-6, characters start at bit 7 (2 chars per word 1).
	 * Continuation fragments (F≠11): no signature field, all
	 * three 7-bit slots are characters starting at bit 0.
	 * Per spec: "Fields R through S are only
	 * transmitted in the first fragment." */
	{
		int is_initial_frag;
		if (config) {
			const struct flex_msg_config *cfg = config;
			is_initial_frag = (cfg->total_fragments <= 1 ||
					   cfg->fragment_index == 0);
		} else {
			is_initial_frag = 1;
		}

		i = 0;
		if (is_initial_frag) {
			shift = FLEX_ALPHA_CHAR2_SHIFT; /* skip signature slot */
		} else {
			shift = 0; /* no signature — start at bit 0 */
		}
		word_idx = 1;

		while (i < max_len) {
			msg_word[word_idx] |= ((uint32_t)msg[i++] & FLEX_ALPHA_CHAR_MASK) << shift;
			shift += FLEX_ALPHA_CHAR_BITS;
			if (shift == FLEX_BCH_DATA_BITS) {
				if (++word_idx >= FLEX_MAX_MSG_WORDS_ALPHA)
					break;
				shift = 0;
			}
		}

		/* Pad unused character slots with ETX per spec.
		 *
		 * TODO: Enhanced Fragmentation —
		 * when U₀V₀ ≠ 00, pad with NUL ($00) instead
		 * of ETX ($03).  Per spec: "the pager must remove function
		 * characters NUL($00) used as fill characters in Enhanced
		 * Fragmentation."  ETX is only used as fill in Standard
		 * Fragmentation mode. */
		if (is_initial_frag) {
			if (shift == FLEX_ALPHA_CHAR2_SHIFT) {
				msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR2_SHIFT) |
						      (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
				word_idx++;
			} else if (shift == FLEX_ALPHA_CHAR3_SHIFT) {
				msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
				word_idx++;
			} else if (shift == 0 && word_idx > 1) {
				/* word boundary — no padding needed */
			}
		} else {
			/* Continuation: 3 chars per word, pad remaining slots */
			if (shift == FLEX_ALPHA_CHAR2_SHIFT) {
				msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR2_SHIFT) |
						      (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
				word_idx++;
			} else if (shift == FLEX_ALPHA_CHAR3_SHIFT) {
				msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
				word_idx++;
			} else if (shift == FLEX_ALPHA_CHAR1_SHIFT) {
				msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR1_SHIFT) |
						      (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR2_SHIFT) |
						      (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
				word_idx++;
			} else if (shift == 0 && word_idx > 1) {
				/* word boundary — no padding needed */
			}
		}

		/* S: 7-bit message signature.
		 * Only present on initial fragment (F=11).
		 *
		 * Per spec: "Signature is defined as the 1's complement of
		 * binary sum for the entire message (including all fragments)
		 * for every 7 bits [a6 a5 a4 a3 a2 a1 a0 + b6 b5 b4 b3 b2
		 * b1 b0 ...] starting from the first 7 bits that directly
		 * follow the Signature Field.  The 7 LSB's of the result is
		 * transmitted as the Message Signature."
		 *
		 * Standard Fragmentation (U₀,V₀ = 0,0 — ASCII/7-bit):
		 *   ETX padding IS included in the sum.  Both TX and RX
		 *   agree on the padding, so the signature matches.
		 *
		 * Enhanced Fragmentation (U₀,V₀ ≠ 0,0 — not implemented):
		 *   Per spec, function characters ETX ($03) and NUL ($00)
		 *   are NOT included in the signature calculation.
		 *
		 * We sum all three 7-bit character slots from each data word
		 * (words 1..N), skipping the signature slot itself (bits 0-6
		 * of word 1).  Since we only use Standard Fragmentation,
		 * ETX padding characters are included in the sum.
		 *
		 * TODO: Enhanced Fragmentation —
		 * when U₀V₀ ≠ 00, exclude ETX ($03) and NUL ($00) from
		 * the signature sum.  Per spec: "function characters NUL
		 * and ETX in Enhanced Fragmentation are not included for
		 * calculation."  This requires tracking which characters
		 * are function chars vs. data during the sum loop. */
		if (is_initial_frag) {
			/* Use precomputed whole-message signature.
			 * Always set by flex_msg_create or flex_encode_frame. */
			if (config) {
				const struct flex_msg_config *cfg = config;
				msg_word[1] |= cfg->precomputed_sig & FLEX_ALPHA_SIG_MASK;
			}
		}
	}

	/* K: 10-bit fragment checksum.
	 * 1's complement of binary sum of all information bits in the
	 * fragment, taken as three groups per word: bits 0-7, 8-15, 16-20.
	 * Computed last since it covers all other fields including S. */
	k_sum = 0;
	for (i = 0; i < word_idx; i++) {
		k_sum += msg_word[i] & FLEX_ALPHA_K_GRP1_MASK;
		k_sum += (msg_word[i] >> FLEX_ALPHA_K_GRP2_SHIFT) & FLEX_ALPHA_K_GRP2_MASK;
		k_sum += (msg_word[i] >> FLEX_ALPHA_K_GRP3_SHIFT) & FLEX_ALPHA_K_GRP3_MASK;
	}
	msg_word[0] |= (~k_sum) & FLEX_ALPHA_HDR_K_MASK;

	/* Write encoded body words to frame (vector word returned to caller) */
	fwc = *fwc_p;
	for (i = 0; i < word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_word[i]));
	*fwc_p = fwc;

	return create_alpha_vector(msg_start, word_idx);
}

/*
 * Encode numeric message.
 *
 * Standard/Special format (V=011/100): 4-bit BCD digits packed LSB-first.
 * First word starts at bit 3 (bits 1-2 = K5,K4 overflow).
 * 5 digits per 21-bit word (bits 3-6, 7-10, 11-14, 15-18, 19-20+carry).
 * Unused positions padded with BCD space (0xC = HEX C).
 *
 * K checksum (6-bit):
 *   1. Sum all message words in 3 groups per word:
 *      group1 = bits 1-8, group2 = bits 9-16, group3 = bits 17-21
 *      (bits 1, 9, 17 are LSB of each group)
 *   2. Take lower 8 bits of total sum
 *   3. Shift 2 MSB right by 6, add to 6 LSB → new 6-bit sum
 *   4. 1's complement → K5-K0
 *   K5,K4 → message word 1 bits 1-2
 *   K3-K0 → vector word bits 17-20
 *
 * Example from spec: "32808590" (8 digits, 2 words)
 *   word1: KK 1100 0100 0001 0000 000  (3,2,8,0,pad)
 *   word2: 1101 0100 1000 0011 0011    (8,5,9,0,pad)
 *   K = 001101 → K5K4=00 in word1, K3-K0=1101 in vector
 *
 * Returns the encoded vector word.  Only body words are written to
 * frame_words starting at *fwc_p.
 */
static uint32_t encode_numeric_message(uint32_t *frame_words, const char *msg,
				   uint32_t msg_start, uint32_t *fwc_p,
				   const void *config)
{
	uint32_t msg_words[FLEX_MAX_MSG_WORDS_NUMERIC] = {0};
	int bit, word_idx, i, b;
	uint32_t k_bit, fwc;

	(void)config;

	/* BCD packing — bit-by-bit, mirrors the RX decoder.
	 * Each character is a 4-bit BCD nibble placed LSB-first
	 * across 21-bit word boundaries without special carry logic. */
	bit = FLEX_NUM_OVERHEAD_BITS;  /* First 2 bits reserved for K5K4 */
	for (i = 0; msg[i] != '\0'; i++) {
		uint8_t nibble = flex_num_char_to_bcd(msg[i]);
		for (b = 0; b < 4; b++) {
			word_idx = bit / FLEX_BCH_DATA_BITS;
			if (word_idx >= FLEX_MAX_MSG_WORDS_NUMERIC)
				break;
			if (nibble & (1 << b))
				msg_words[word_idx] |= (1U << (bit % FLEX_BCH_DATA_BITS));
			bit++;
		}
	}

	/* Pad with BCD space (0xC) to fill remaining nibble slots in last word.
	 * Remaining partial bits stay 0 (Section 3.10.1.1.1:
	 * "Space characters (HEX C) are inserted to fill unused 4-bit
	 * character positions in the last word and 0s are inserted to
	 * fill remaining partial characters.") */
	word_idx = (bit > 0) ? (bit - 1) / FLEX_BCH_DATA_BITS : 0;
	{
		int end_bit = (word_idx + 1) * FLEX_BCH_DATA_BITS;
		while (bit + 4 <= end_bit) {
			for (b = 0; b < 4; b++) {
				if (FLEX_NUM_BCD_SPACE & (1 << b))
					msg_words[bit / FLEX_BCH_DATA_BITS] |=
						(1U << (bit % FLEX_BCH_DATA_BITS));
				bit++;
			}
		}
	}

	/* K checksum.
	 *
	 * Three groups per word: bits 1-8, 9-16, 17-21 (bits 1,9,17 = LSB).
	 * Binary sum all groups across all message words.
	 * Lower 8 bits → shift 2 MSB right 6 → add to 6 LSB → 1's complement.
	 * K5,K4 go into msg_words[0] bits 1-2 (first message word).
	 * K3-K0 go into the vector word (returned via create_numeric_vector). */
	k_bit = 0;
	for (i = 0; i <= word_idx; i++) {
		k_bit += (msg_words[i])       & 0xFF;
		k_bit += (msg_words[i] >> 8)  & 0xFF;
		k_bit += (msg_words[i] >> 16) & 0x1F;
	}
	k_bit &= 0xFF;
	k_bit = (k_bit & 0x3F) + (k_bit >> 6);
	k_bit = ~k_bit;
	msg_words[0] |= FLEX_NUM_K54_FROM_K(k_bit);  /* k5k4 bits */

	LOGP(DFLEX, LOGL_DEBUG,
	     "TX: Numeric encoder: %d words, K=0x%02X (K5K4=%d%d K3-0=0x%X)\n",
	     word_idx + 1, k_bit & 0x3F,
	     (k_bit >> 5) & 1, (k_bit >> 4) & 1, k_bit & 0xF);

	/* Write encoded body words to frame (vector word returned to caller) */
	fwc = *fwc_p;
	for (i = 0; i <= word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_words[i]));
	*fwc_p = fwc;

	return create_numeric_vector(msg_start, word_idx + 1, k_bit,
				    FLEX_VECTOR_TYPE_NUMERIC);
}

/*
 * Secure message encoder (V=000).
 *
 * Two body encoding modes selected by secure_encoding:
 *   0 = alpha: 7-bit character packing identical to encode_alpha_message()
 *   1 = binary: hex nibble packing identical to encode_hex_message()
 *
 * Supports fragmentation with the same C/F/N mechanism as Alpha and HEX:
 *   F = 2-bit fragment number, modulo 3 sequence (11, 00, 01, 10, 00, ...)
 *       — "11" is skipped after the first fragment to avoid ambiguity.
 *   C = message continued flag (1 = more fragments follow, 0 = last/only).
 *       The last fragment is indicated by resetting C to 0.
 *   N = 6-bit message number (0-63) identifying the fragment stream.
 *
 * Key difference from alpha/hex: bits 19-20 of the header word carry the
 * t1t0 subtype on ALL fragments (not R/M on initial, U₀/V₀ on continuation).
 * The C/F/N fragment fields are identical to alpha (same bit positions).
 *
 * Returns a secure vector word (V=000) with 7-bit start / 7-bit length.
 */
static uint32_t encode_secure_message(uint32_t *frame_words, const char *msg,
				      uint32_t msg_start, uint32_t *fwc_p,
				      int sequence_num,
				      const struct flex_msg_config *cfg,
				      int secure_subtype, int secure_encoding,
				      int *error)
{
	if (secure_encoding == 1) {
		/* Binary mode: reuse hex nibble packing */
		uint32_t msg_words[FLEX_MAX_MSG_WORDS_HEX];
		uint32_t fwc;
		size_t len, i;
		int word_idx, data_idx;
		int frag_idx = 0, frag_total = 0;
		int is_initial_frag;
		uint32_t f_val, k_sum;
		int c_val;

		if (cfg) {
			frag_idx = cfg->fragment_index;
			frag_total = cfg->total_fragments;
		}

		is_initial_frag = (frag_total <= 1 || frag_idx == 0);
		f_val = flex_fragment_number(frag_idx);
		c_val = (frag_total > 1 && frag_idx < frag_total - 1) ? 1 : 0;

		/* Validate all characters are hex */
		len = strlen(msg);
		for (i = 0; i < len; i++) {
			if (!is_valid_hex_char(msg[i])) {
				*error = -FLEX_ERR_INVALID_MESSAGE;
				return 0;
			}
		}

		if (len > FLEX_MAX_CHARS_HEX)
			len = FLEX_MAX_CHARS_HEX;

		memset(msg_words, 0, sizeof(msg_words));

		/* Header word 1: C, F, N (same bit positions as alpha).
		 * F = modulo 3 fragment number (11, 00, 01, 10, 00, ...);
		 * C = message continued (1 = more fragments, 0 = last/only);
		 * N = 6-bit message number identifying the fragment stream. */
		msg_words[0] = (f_val << FLEX_ALPHA_HDR_F_SHIFT);
		if (c_val)
			msg_words[0] |= FLEX_ALPHA_HDR_C_MASK;
		if (sequence_num >= 0) {
			uint32_t n = (uint32_t)(sequence_num % 64);
			msg_words[0] |= (n << FLEX_ALPHA_HDR_N_SHIFT);
		}

		/* Bits 19-20: t1t0 subtype on ALL fragments */
		msg_words[0] |= ((uint32_t)(secure_subtype & FLEX_SEC_TYPE_MASK)
				 << FLEX_SEC_TYPE_SHIFT);

		/* Header word 2: initial fragment only (R/M/D/H/B/I/s/S) */
		if (is_initial_frag) {
			int b_field = 0;
			if (cfg)
				b_field = cfg->blocking_length & 0xF;
			msg_words[1] = 0;
			msg_words[1] |= ((uint32_t)b_field << FLEX_HEX_HDR2_B_SHIFT);
			if (sequence_num >= 0)
				msg_words[1] |= FLEX_HEX_HDR2_R_MASK;
			if (cfg && cfg->mail_drop)
				msg_words[1] |= FLEX_HEX_HDR2_M_MASK;
			data_idx = 2;
		} else {
			data_idx = 1;
		}

		/* Pack hex nibbles as continuous bit stream (same as encode_hex_message) */
		{
			int bit = data_idx * FLEX_BCH_DATA_BITS;
			int b;

			for (i = 0; i < len; i++) {
				uint8_t nibble = hex_char_to_nibble(msg[i]);
				for (b = 0; b < 4; b++) {
					int wi = bit / FLEX_BCH_DATA_BITS;
					if (wi >= FLEX_MAX_MSG_WORDS_HEX)
						break;
					if (nibble & (1 << b))
						msg_words[wi] |= (1U << (bit % FLEX_BCH_DATA_BITS));
					bit++;
				}
			}

			word_idx = (bit > 0) ? (bit - 1) / FLEX_BCH_DATA_BITS : data_idx;

			/* Termination fill: inverse of last data bit */
			if (bit > data_idx * FLEX_BCH_DATA_BITS) {
				int last_data_bit = (msg_words[(bit - 1) / FLEX_BCH_DATA_BITS]
						     >> ((bit - 1) % FLEX_BCH_DATA_BITS)) & 1;
				uint32_t fill_bit = last_data_bit ? 0 : 1;
				int end_bit = (word_idx + 1) * FLEX_BCH_DATA_BITS;
				int b2;

				for (b2 = bit; b2 < end_bit; b2++) {
					if (fill_bit)
						msg_words[b2 / FLEX_BCH_DATA_BITS] |=
							(1U << (b2 % FLEX_BCH_DATA_BITS));
				}
				word_idx++;

				uint8_t last_nib = hex_char_to_nibble(msg[len - 1]);
				if ((last_nib == 0x0 || last_nib == 0xF) &&
				    word_idx < FLEX_MAX_MSG_WORDS_HEX) {
					uint32_t extra = 0;
					if (fill_bit)
						for (b2 = 0; b2 < 21; b2++)
							extra |= (1U << b2);
					msg_words[word_idx] = extra;
					word_idx++;
				}
			}
		}

		/* Signature: initial fragment only */
		if (is_initial_frag) {
			if (cfg)
				msg_words[1] |= (cfg->precomputed_sig & 0xFF) << FLEX_HEX_HDR2_S_SHIFT;
		}

		/* K: 10-bit fragment checksum (same as alpha K) */
		k_sum = 0;
		for (i = 0; i < (size_t)word_idx; i++) {
			k_sum += msg_words[i] & FLEX_ALPHA_K_GRP1_MASK;
			k_sum += (msg_words[i] >> FLEX_ALPHA_K_GRP2_SHIFT) & FLEX_ALPHA_K_GRP2_MASK;
			k_sum += (msg_words[i] >> FLEX_ALPHA_K_GRP3_SHIFT) & FLEX_ALPHA_K_GRP3_MASK;
		}
		msg_words[0] |= (~k_sum) & FLEX_ALPHA_HDR_K_MASK;

		LOGP(DFLEX, LOGL_DEBUG,
		     "TX: Secure binary encoder: V=000 t1t0=%d, %d words, "
		     "frag=%d/%d C=%d\n",
		     secure_subtype, word_idx,
		     frag_idx, frag_total > 0 ? frag_total : 1, c_val);

		fwc = *fwc_p;
		for (i = 0; i < (size_t)word_idx; i++)
			frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_words[i]));
		*fwc_p = fwc;

		return create_secure_vector(msg_start, word_idx);
	}

	/* Registration Acknowledgment mode (secure_encoding == 2).
	 * Per §3.10.1.4-2: V=000, t1t0=00, 2nd word has opcode "="
	 * (0x3D) in bits 0-6, ETX in bits 7-20. */
	if (secure_encoding == FLEX_SEC_ENC_REGACK) {
		uint32_t msg_word[2] = {0};
		uint32_t fwc, k_sum;
		uint32_t f_val = flex_fragment_number(0);

		/* Header word: F=11 (initial), C=0 (not continued), t1t0=00 */
		msg_word[0] = (f_val << FLEX_ALPHA_HDR_F_SHIFT);
		if (sequence_num >= 0) {
			uint32_t n = (uint32_t)(sequence_num % 64);
			msg_word[0] |= (n << FLEX_ALPHA_HDR_N_SHIFT);
		}
		/* t1t0=00 (alpha) — already zero */

		/* 2nd word: opcode "=" in bits 0-6, ETX in bits 7-13 and 14-20 */
		msg_word[1] = (FLEX_SEC_REGACK_OPCODE & FLEX_SEC_REGACK_OPCODE_MASK)
			    << FLEX_SEC_REGACK_OPCODE_SHIFT;
		msg_word[1] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR2_SHIFT);
		msg_word[1] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);

		/* K checksum */
		k_sum = 0;
		k_sum += msg_word[0] & FLEX_ALPHA_K_GRP1_MASK;
		k_sum += (msg_word[0] >> FLEX_ALPHA_K_GRP2_SHIFT) & FLEX_ALPHA_K_GRP2_MASK;
		k_sum += (msg_word[0] >> FLEX_ALPHA_K_GRP3_SHIFT) & FLEX_ALPHA_K_GRP3_MASK;
		k_sum += msg_word[1] & FLEX_ALPHA_K_GRP1_MASK;
		k_sum += (msg_word[1] >> FLEX_ALPHA_K_GRP2_SHIFT) & FLEX_ALPHA_K_GRP2_MASK;
		k_sum += (msg_word[1] >> FLEX_ALPHA_K_GRP3_SHIFT) & FLEX_ALPHA_K_GRP3_MASK;
		msg_word[0] |= (~k_sum) & FLEX_ALPHA_HDR_K_MASK;

		LOGP(DFLEX, LOGL_DEBUG,
		     "TX: Secure REGACK encoder: V=000 t1t0=00, opcode=0x%02X\n",
		     FLEX_SEC_REGACK_OPCODE);

		fwc = *fwc_p;
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_word[0]));
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_word[1]));
		*fwc_p = fwc;

		return create_secure_vector(msg_start, 2);
	}

	/* Alpha mode (secure_encoding == 0): 7-bit character packing */
	{
		uint32_t msg_word[FLEX_MAX_MSG_WORDS_ALPHA] = {0};
		uint32_t word_idx, fwc;
		size_t len, max_len;
		uint32_t k_sum;
		int shift;
		uint32_t i;
		int frag_idx = 0, frag_total = 0;
		uint32_t f_val;
		int c_val;

		if (cfg) {
			frag_idx = cfg->fragment_index;
			frag_total = cfg->total_fragments;
		}

		f_val = flex_fragment_number(frag_idx);
		c_val = (frag_total > 1 && frag_idx < frag_total - 1) ? 1 : 0;

		len = strlen(msg);
		max_len = (len > FLEX_MAX_CHARS_ALPHA) ? FLEX_MAX_CHARS_ALPHA : len;

		/* Header word: C, F, N (same bit positions as alpha).
		 * F = modulo 3 fragment number (11, 00, 01, 10, 00, ...);
		 * C = message continued (1 = more fragments, 0 = last/only);
		 * N = 6-bit message number identifying the fragment stream. */
		msg_word[0] = (f_val << FLEX_ALPHA_HDR_F_SHIFT);
		if (c_val)
			msg_word[0] |= FLEX_ALPHA_HDR_C_MASK;
		if (sequence_num >= 0) {
			uint32_t n = (uint32_t)(sequence_num % 64);
			msg_word[0] |= (n << FLEX_ALPHA_HDR_N_SHIFT);
		}

		/* Bits 19-20: t1t0 subtype on ALL fragments (not R/M) */
		msg_word[0] |= ((uint32_t)(secure_subtype & FLEX_SEC_TYPE_MASK)
				<< FLEX_SEC_TYPE_SHIFT);

		/* Pack 7-bit ASCII characters, 3 per word.
		 * Secure messages do NOT have a 7-bit signature field --
		 * all 3 character slots are used from the 2nd word onward.
		 * Characters start at bit 0 for all fragments. */
		i = 0;
		shift = 0;
		word_idx = 1;

		while (i < max_len) {
			msg_word[word_idx] |= ((uint32_t)msg[i++] & FLEX_ALPHA_CHAR_MASK) << shift;
			shift += FLEX_ALPHA_CHAR_BITS;
			if (shift == FLEX_BCH_DATA_BITS) {
				if (++word_idx >= FLEX_MAX_MSG_WORDS_ALPHA)
					break;
				shift = 0;
			}
		}

		/* Pad unused character slots with ETX */
		/* Pad unused character slots with ETX.
		 * Secure messages use all 3 character slots per word
		 * (no signature field), so padding is the same for
		 * initial and continuation fragments. */
		if (shift == FLEX_ALPHA_CHAR2_SHIFT) {
			msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR2_SHIFT) |
					      (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
			word_idx++;
		} else if (shift == FLEX_ALPHA_CHAR3_SHIFT) {
			msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
			word_idx++;
		} else if (shift == FLEX_ALPHA_CHAR1_SHIFT) {
			msg_word[word_idx] |= (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR1_SHIFT) |
					      (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR2_SHIFT) |
					      (FLEX_ALPHA_ETX << FLEX_ALPHA_CHAR3_SHIFT);
			word_idx++;
		} else if (shift == 0 && word_idx > 1) {
			/* word boundary -- no padding needed */
		}

		/* Secure messages do NOT have a 7-bit signature.
		 * Integrity is provided by the 10-bit K checksum only. */
		k_sum = 0;
		for (i = 0; i < word_idx; i++) {
			k_sum += msg_word[i] & FLEX_ALPHA_K_GRP1_MASK;
			k_sum += (msg_word[i] >> FLEX_ALPHA_K_GRP2_SHIFT) & FLEX_ALPHA_K_GRP2_MASK;
			k_sum += (msg_word[i] >> FLEX_ALPHA_K_GRP3_SHIFT) & FLEX_ALPHA_K_GRP3_MASK;
		}
		msg_word[0] |= (~k_sum) & FLEX_ALPHA_HDR_K_MASK;

		LOGP(DFLEX, LOGL_DEBUG,
		     "TX: Secure alpha encoder: V=000 t1t0=%d, %d words, "
		     "frag=%d/%d C=%d\n",
		     secure_subtype, word_idx,
		     frag_idx, frag_total > 0 ? frag_total : 1, c_val);

		fwc = *fwc_p;
		for (i = 0; i < word_idx; i++)
			frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_word[i]));
		*fwc_p = fwc;

		return create_secure_vector(msg_start, word_idx);
	}
}

/*
 * Special format numeric message encoder (V=100).
 *
 * Identical BCD encoding to standard numeric — same packing, K checksum,
 * padding, and character set.  Only the vector type differs (V=100 vs V=011).
 * Single-frame only; max 41 BCD characters.
 */
static uint32_t encode_special_numeric_message(uint32_t *frame_words,
					       const char *msg,
					       uint32_t msg_start,
					       uint32_t *fwc_p,
					       int *error)
{
	uint32_t msg_words[FLEX_MAX_MSG_WORDS_NUMERIC] = {0};
	int bit, word_idx, i, b;
	uint32_t k_bit, fwc;
	size_t len;

	len = strlen(msg);

	/* Enforce 41-character max */
	if (len > FLEX_MAX_CHARS_NUMERIC) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/* Validate all characters are valid BCD */
	for (i = 0; msg[i] != '\0'; i++) {
		if (!is_valid_numeric_char(msg[i])) {
			*error = -FLEX_ERR_INVALID_MESSAGE;
			return 0;
		}
	}

	/* BCD packing — bit-by-bit, identical to encode_numeric_message() */
	bit = FLEX_NUM_OVERHEAD_BITS;
	for (i = 0; msg[i] != '\0'; i++) {
		uint8_t nibble = flex_num_char_to_bcd(msg[i]);
		for (b = 0; b < 4; b++) {
			word_idx = bit / FLEX_BCH_DATA_BITS;
			if (word_idx >= FLEX_MAX_MSG_WORDS_NUMERIC)
				break;
			if (nibble & (1 << b))
				msg_words[word_idx] |= (1U << (bit % FLEX_BCH_DATA_BITS));
			bit++;
		}
	}

	/* Pad with BCD space, remaining partial bits stay 0 */
	word_idx = (bit > 0) ? (bit - 1) / FLEX_BCH_DATA_BITS : 0;
	{
		int end_bit = (word_idx + 1) * FLEX_BCH_DATA_BITS;
		while (bit + 4 <= end_bit) {
			for (b = 0; b < 4; b++) {
				if (FLEX_NUM_BCD_SPACE & (1 << b))
					msg_words[bit / FLEX_BCH_DATA_BITS] |=
						(1U << (bit % FLEX_BCH_DATA_BITS));
				bit++;
			}
		}
	}

	/* K checksum — identical to standard numeric */
	k_bit = 0;
	for (i = 0; i <= word_idx; i++) {
		k_bit += (msg_words[i])       & 0xFF;
		k_bit += (msg_words[i] >> 8)  & 0xFF;
		k_bit += (msg_words[i] >> 16) & 0x1F;
	}
	k_bit &= 0xFF;
	k_bit = (k_bit & 0x3F) + (k_bit >> 6);
	k_bit = ~k_bit;
	msg_words[0] |= FLEX_NUM_K54_FROM_K(k_bit);

	LOGP(DFLEX, LOGL_DEBUG,
	     "TX: Special numeric encoder: V=100, %d words, "
	     "K=0x%02X (K5K4=%d%d K3-0=0x%X)\n",
	     word_idx + 1, k_bit & 0x3F,
	     (k_bit >> 5) & 1, (k_bit >> 4) & 1, k_bit & 0xF);

	fwc = *fwc_p;
	for (i = 0; i <= word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_words[i]));
	*fwc_p = fwc;

	return create_numeric_vector(msg_start, word_idx + 1, k_bit,
				    FLEX_VECTOR_TYPE_SPECIAL_NUM);
}

/*
 * Numbered numeric message encoder (V=111).
 *
 * Extends the BCD encoding with a header in the first message word
 * (Fig. 3.10.1.1.2-1):
 *   bits 0-1:  K5,K4 (checksum overflow, same as standard numeric)
 *   bits 2-7:  N5-N0 message number (0-63)
 *   bit  8:    R0 retrieval flag
 *   bit  9:    S0 special format flag
 *   bits 10+:  BCD digit data (starting at bit offset 10)
 *
 * Max 40 BCD characters (8 words × 21 bits - 10 skip bits = 158 data bits
 * = 39.5 nibbles → 39 full nibbles + 2 leftover bits).  Single-frame only.
 */
static uint32_t encode_numbered_numeric_message(uint32_t *frame_words,
						const char *msg,
						uint32_t msg_start,
						uint32_t *fwc_p,
						int msg_num, int r_flag,
						int s_flag, int *error)
{
	uint32_t msg_words[FLEX_MAX_MSG_WORDS_NUMERIC] = {0};
	int bit, word_idx, i, b;
	uint32_t k_bit, fwc;
	size_t len;

	/* Max 39 characters for numbered numeric */
	len = strlen(msg);
	if (len > 39) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/* Validate all characters are valid BCD */
	for (i = 0; msg[i] != '\0'; i++) {
		if (!is_valid_numeric_char(msg[i])) {
			*error = -FLEX_ERR_INVALID_MESSAGE;
			return 0;
		}
	}

	/* Encode header in first message word (after K5K4 bits 0-1):
	 *   bits 2-7:  N5-N0 message number (FLEX_NUM_N_SHIFT/MASK)
	 *   bit  8:    R0 retrieval flag (FLEX_NUM_R_SHIFT)
	 *   bit  9:    S0 special format flag (FLEX_NUM_S_SHIFT) */
	msg_words[0] |= ((uint32_t)(msg_num & FLEX_NUM_N_MASK) << FLEX_NUM_N_SHIFT);
	if (r_flag)
		msg_words[0] |= (1U << FLEX_NUM_R_SHIFT);
	if (s_flag)
		msg_words[0] |= (1U << FLEX_NUM_S_SHIFT);

	/* BCD packing — bit-by-bit, starts after 10-bit header */
	bit = FLEX_NUM_NUMBERED_SKIP_BITS;
	for (i = 0; msg[i] != '\0'; i++) {
		uint8_t nibble = flex_num_char_to_bcd(msg[i]);
		for (b = 0; b < 4; b++) {
			word_idx = bit / FLEX_BCH_DATA_BITS;
			if (word_idx >= FLEX_MAX_MSG_WORDS_NUMERIC)
				break;
			if (nibble & (1 << b))
				msg_words[word_idx] |= (1U << (bit % FLEX_BCH_DATA_BITS));
			bit++;
		}
	}

	/* Pad with BCD space, remaining partial bits stay 0 */
	word_idx = (bit > 0) ? (bit - 1) / FLEX_BCH_DATA_BITS : 0;
	{
		int end_bit = (word_idx + 1) * FLEX_BCH_DATA_BITS;
		while (bit + 4 <= end_bit) {
			for (b = 0; b < 4; b++) {
				if (FLEX_NUM_BCD_SPACE & (1 << b))
					msg_words[bit / FLEX_BCH_DATA_BITS] |=
						(1U << (bit % FLEX_BCH_DATA_BITS));
				bit++;
			}
		}
	}

	/* K checksum over all message words including header bits */
	k_bit = 0;
	for (i = 0; i <= word_idx; i++) {
		k_bit += (msg_words[i])       & 0xFF;
		k_bit += (msg_words[i] >> 8)  & 0xFF;
		k_bit += (msg_words[i] >> 16) & 0x1F;
	}
	k_bit &= 0xFF;
	k_bit = (k_bit & 0x3F) + (k_bit >> 6);
	k_bit = ~k_bit;
	msg_words[0] |= FLEX_NUM_K54_FROM_K(k_bit);  /* K5K4 in bits 0-1 */

	LOGP(DFLEX, LOGL_DEBUG,
	     "TX: Numbered numeric encoder: V=111, N=%d S=%d R=%d, "
	     "%d words, K=0x%02X (K5K4=%d%d K3-0=0x%X)\n",
	     msg_num, s_flag, r_flag, word_idx + 1, k_bit & 0x3F,
	     (k_bit >> 5) & 1, (k_bit >> 4) & 1, k_bit & 0xF);

	fwc = *fwc_p;
	for (i = 0; i <= word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_words[i]));
	*fwc_p = fwc;

	return create_numeric_vector(msg_start, word_idx + 1, k_bit,
				    FLEX_VECTOR_TYPE_NUMBERED_NUM);
}

/* ===== Message Type Detection ===== */

int flex_detect_msg_type(const char *message, int length)
{
	int i;

	if (!message || length <= 0)
		return FLEX_FRAME_MSG_TYPE_TONE;

	for (i = 0; i < length; i++) {
		if (!is_valid_numeric_char(message[i]))
			return FLEX_FRAME_MSG_TYPE_ALPHA;
	}

	return FLEX_FRAME_MSG_TYPE_NUMERIC;
}

/* ===== Complete Frame Encoding ===== */

/*
 * Encode a complete FLEX frame (backward-compatible single-message API).
 *
 * Assembles: ERS burst + S1 sync + FIW + S2 sync + data block.
 * Delegates frame encoding to flex_encode_frame_multi() with one-shot
 * default parameters (cycle=0, frame=0, collapse=0, 1600 baud).
 *
 * Returns bytes written to buffer, or 0 on error.
 */
size_t flex_encode_frame(uint64_t capcode, int msg_type,
			 const char *message, uint8_t *buffer,
			 size_t buffer_size, int *error)
{
	flex_frame_msg_t fmsg;
	flex_frame_params_t params;
	uint8_t *out;
	size_t ers_len, frame_len;
	int msgs_packed = 0;

	if (!error)
		return 0;
	*error = 0;

	if (!buffer || buffer_size < FLEX_BUFFER_SIZE) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	out = buffer;

	/* Prepend ERS burst (35 cycles at 1600 baud) */
	ers_len = flex_generate_ers(out, buffer_size, FLEX_ERS_CYCLES);
	if (ers_len == 0) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}
	out += ers_len;

	/* Resolve AUTO type */
	if (msg_type == FLEX_FRAME_MSG_TYPE_AUTO)
		msg_type = flex_detect_msg_type(message,
			message ? (int)strlen(message) : 0);

	/* Build single-message descriptor */
	memset(&fmsg, 0, sizeof(fmsg));
	fmsg.capcode = capcode;
	fmsg.msg_type = msg_type;
	fmsg.message = message;
	fmsg.message_length = message ? (int)strlen(message) : 0;
	fmsg.speed = 1600;
	fmsg.polarity = FLEX_DEFAULT_POLARITY;
	fmsg.sequence_num = -1;
	fmsg.temp_delivery_slot = -1;
	fmsg.short_msg_type = FLEX_SMSG_TYPE_NUMERIC;
	fmsg.short_msg_source = 0;
	fmsg.short_msg_number = 0;
	fmsg.short_msg_r = 0;

	/* Pre-compute message signature for types that use it */
	fmsg.precomputed_sig = 0xFFFFFFFF;
	if (msg_type == FLEX_FRAME_MSG_TYPE_ALPHA ||
	    msg_type == FLEX_FRAME_MSG_TYPE_SECURE) {
		/* 7-bit alpha signature: sum of all chars excluding ETX */
		uint32_t sum = 0;
		int k;
		for (k = 0; k < fmsg.message_length; k++) {
			unsigned char ch = (unsigned char)message[k] & 0x7F;
			if (ch != 0x03) sum += ch;
		}
		fmsg.precomputed_sig = (~sum) & 0x7F;
	} else if (msg_type == FLEX_FRAME_MSG_TYPE_HEX) {
		/* 8-bit hex signature: sum of data bytes */
		uint32_t sum = 0;
		int bit_pos = 0;
		uint8_t accum = 0;
		int accum_bits = 0;
		int k;
		for (k = 0; k < fmsg.message_length; k++) {
			uint8_t nib;
			char c = message[k];
			if (c >= '0' && c <= '9') nib = c - '0';
			else if (c >= 'A' && c <= 'F') nib = c - 'A' + 10;
			else if (c >= 'a' && c <= 'f') nib = c - 'a' + 10;
			else continue;
			int b;
			for (b = 0; b < 4; b++) {
				accum |= (uint8_t)(((nib >> b) & 1) << accum_bits);
				accum_bits++;
				bit_pos++;
				if (accum_bits == 8) { sum += accum; accum = 0; accum_bits = 0; }
			}
		}
		if (accum_bits > 0) sum += accum;
		fmsg.precomputed_sig = (~sum) & 0xFF;
	}

	/* One-shot default parameters */
	flex_frame_params_default(&params);

	/* Delegate to multi-message encoder (produces S1 + FIW + S2 + data) */
	frame_len = flex_encode_frame_multi(&fmsg, 1, &params,
					    out, buffer_size - ers_len,
					    &msgs_packed, error);
	if (frame_len == 0)
		return 0;

	return ers_len + frame_len;
}

/* ===== Text Utilities ===== */

static const char *ctrl_names[32] = {
	"<NUL>", "<SOH>", "<STX>", "<ETX>", "<EOT>", "<ENQ>", "<ACK>", "<BEL>",
	"<BS>",  "<HT>",  "<LF>",  "<VT>",  "<FF>",  "<CR>",  "<SO>",  "<SI>",
	"<DLE>", "<DC1>", "<DC2>", "<DC3>", "<DC4>", "<NAK>", "<SYN>", "<ETB>",
	"<CAN>", "<EM>",  "<SUB>", "<ESC>", "<FS>",  "<GS>",  "<RS>",  "<US>",
};

static const char *del_name = "<DEL>";

/*
 * Convert message to printable string, replacing control characters
 * with symbolic names (e.g. <LF>, <CR>).
 */
const char *flex_print_message(const char *message, int message_length)
{
	static char buf[1024];
	const char *sym;
	int i, pos;

	for (i = 0, pos = 0; i < message_length; i++) {
		if (message[i] >= 0 && message[i] <= 31)
			sym = ctrl_names[(int)message[i]];
		else if (message[i] == 127)
			sym = del_name;
		else {
			if (pos < (int)sizeof(buf) - 1)
				buf[pos++] = message[i];
			continue;
		}
		int slen = strlen(sym);
		if (pos + slen >= (int)sizeof(buf))
			break;
		memcpy(buf + pos, sym, slen);
		pos += slen;
	}
	buf[pos] = '\0';
	return buf;
}

/*
 * Scan message string, converting symbolic control character names
 * back to actual control character values.
 *
 * Returns number of output characters written.
 */
int flex_scan_message(const char *message_input, int message_input_length,
		      char *message_output, int message_output_length)
{
	int i, out, j, clen;

	for (i = 0, out = 0; i < message_input_length && out < message_output_length; out++) {
		if (message_input[i] == '<') {
			/* Try to match a control character name */
			for (j = 0; j < 32; j++) {
				clen = strlen(ctrl_names[j]);
				if (clen <= message_input_length - i &&
				    !memcmp(message_input + i, ctrl_names[j], clen))
					break;
			}
			if (j < 32) {
				message_output[out] = j;
				i += clen;
			} else {
				clen = strlen(del_name);
				if (clen <= message_input_length - i &&
				    !memcmp(message_input + i, del_name, clen)) {
					message_output[out] = 127;
					i += clen;
				} else {
					message_output[out] = '<';
					i++;
				}
			}
		} else {
			message_output[out] = message_input[i];
			i++;
		}
	}

	return out;
}

/* ===== Pre-enqueue Payload Validation =====
 *
 * Validates message payload for a given type BEFORE it enters the TX queue.
 * Catches the same errors that the per-type encoders would catch at frame
 * encoding time, so bad messages never poison a frame batch.
 *
 * Returns 0 if valid, or -FLEX_ERR_INVALID_MESSAGE on failure. */
int flex_msg_validate(int msg_type, const char *data, int data_length,
		      int secure_encoding)
{
	int i;

	switch (msg_type) {
	case FLEX_FRAME_MSG_TYPE_INSTRUCTION:
		/* Requires non-empty decimal integer 0-16383 */
		if (!data || data_length <= 0)
			return -FLEX_ERR_INVALID_MESSAGE;
		{
			unsigned long val;
			char *endptr;
			char buf[32];
			int len = data_length;
			if (len >= (int)sizeof(buf))
				len = (int)sizeof(buf) - 1;
			memcpy(buf, data, len);
			buf[len] = '\0';
			val = strtoul(buf, &endptr, 10);
			if (*endptr != '\0' || val > 0x3FFF)
				return -FLEX_ERR_INVALID_MESSAGE;
		}
		break;

	case FLEX_FRAME_MSG_TYPE_SPECIAL_NUM:
		/* BCD characters only, max 41 */
		if (!data || data_length <= 0)
			return -FLEX_ERR_INVALID_MESSAGE;
		if (data_length > FLEX_MAX_CHARS_NUMERIC)
			return -FLEX_ERR_INVALID_MESSAGE;
		for (i = 0; i < data_length; i++) {
			if (!is_valid_numeric_char(data[i]))
				return -FLEX_ERR_INVALID_MESSAGE;
		}
		break;

	case FLEX_FRAME_MSG_TYPE_NUMBERED_NUM:
	case FLEX_FRAME_MSG_TYPE_NUMBERED_SPECIAL:
		/* BCD characters only, max 39 */
		if (!data || data_length <= 0)
			return -FLEX_ERR_INVALID_MESSAGE;
		if (data_length > 39)
			return -FLEX_ERR_INVALID_MESSAGE;
		for (i = 0; i < data_length; i++) {
			if (!is_valid_numeric_char(data[i]))
				return -FLEX_ERR_INVALID_MESSAGE;
		}
		break;

	case FLEX_FRAME_MSG_TYPE_HEX:
		/* Hex characters only */
		if (!data || data_length <= 0)
			return -FLEX_ERR_INVALID_MESSAGE;
		for (i = 0; i < data_length; i++) {
			if (!is_valid_hex_char(data[i]))
				return -FLEX_ERR_INVALID_MESSAGE;
		}
		break;

	case FLEX_FRAME_MSG_TYPE_SECURE:
		if (!data || data_length <= 0)
			return -FLEX_ERR_INVALID_MESSAGE;
		if (secure_encoding == 1) {
			/* Binary mode: hex characters only */
			for (i = 0; i < data_length; i++) {
				if (!is_valid_hex_char(data[i]))
					return -FLEX_ERR_INVALID_MESSAGE;
			}
		}
		/* Alpha mode: any 7-bit ASCII is fine */
		break;

	default:
		/* Other types (tone, alpha, numeric) either have no payload
		 * requirements or are validated elsewhere. */
		break;
	}

	return 0;
}

void flex_frame_params_default(flex_frame_params_t *params)
{
	memset(params, 0, sizeof(*params));
	params->bitrate = 1600;
	params->modulation_type = FLEX_MOD_2FSK;
	params->timezone_code = -1; /* no timezone by default */
	params->sysmsg_a_type = -1; /* no system message BIW by default */
	params->num_transmissions = 1; /* single transmission (no repeat) */
	params->td_collapse = -1; /* use system collapse cycle */
	params->subframe_index = 0;
}

/*
 * Block Information Word 2 (type 000 = SSID1).
 *
 * local_id:    Local ID (9 bits, 0-511)
 * coverage_id: Coverage zone ID (5 bits, 0-31)
 */
uint32_t flex_create_biw2(uint32_t local_id, uint32_t coverage_id)
{
	uint32_t dw = 0;
	/* type field bits 4-6 = 000 (SSID1) — already zero */
	dw |= (coverage_id & FLEX_BIW_SSID1_COVERAGE_MASK) << FLEX_BIW_SSID1_COVERAGE_SHIFT;
	dw |= (local_id    & FLEX_BIW_SSID1_LOCALID_MASK)  << FLEX_BIW_SSID1_LOCALID_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word 3 (type 001 = Date).
 *
 * year:  Year offset (5 bits, 0-31, base 1994 → 1994-2025).
 *        Caller should use flex_biw_equiv_year() for years >2025.
 * month: Month of year (4 bits, 0001-1100, Jan-Dec)
 * day:   Day of month (5 bits, 00001-11111, 1-31)
 */
uint32_t flex_create_biw3(uint32_t year, uint32_t month, uint32_t day)
{
	uint32_t dw = 0;
	/* type field bits 4-6 = 001 (Date) */
	dw |= (uint32_t)FLEX_BIW_TYPE_DATE << FLEX_BIW_TYPE_SHIFT;
	dw |= (year  & FLEX_BIW_DATE_YEAR_MASK)  << FLEX_BIW_DATE_YEAR_SHIFT;
	dw |= (day   & FLEX_BIW_DATE_DAY_MASK)   << FLEX_BIW_DATE_DAY_SHIFT;
	dw |= (month & FLEX_BIW_DATE_MONTH_MASK)  << FLEX_BIW_DATE_MONTH_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word 4 (type 010 = Time).
 *
 * hour:   Hour of day (5 bits, 00000-10111, 0-23)
 * minute: Minute of hour (6 bits, 000000-111011, 0-59)
 * second: Second in 7.5s steps (3 bits, 000-111, 1/8 minute)
 *
 * The Second field can be extended to 1/64 minute (0.9375s) resolution
 * by the S5-S3 field in BIW SysInfo type 101 (A=0100 or A=0101).
 */
uint32_t flex_create_biw4(uint32_t hour, uint32_t minute, uint32_t second)
{
	uint32_t dw = 0;
	/* type field bits 4-6 = 010 (Time) */
	dw |= (uint32_t)FLEX_BIW_TYPE_TIME << FLEX_BIW_TYPE_SHIFT;
	dw |= (hour   & FLEX_BIW_TIME_HOUR_MASK)   << FLEX_BIW_TIME_HOUR_SHIFT;
	dw |= (minute & FLEX_BIW_TIME_MINUTE_MASK)  << FLEX_BIW_TIME_MINUTE_SHIFT;
	dw |= (second & FLEX_BIW_TIME_SECOND_MASK)  << FLEX_BIW_TIME_SECOND_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word — SSID2 (type 111).
 *
 * country_code: ITU-T E.212 country code (10 bits, 0-1023)
 * tmf:          Traffic management flags (4 bits, 0-15)
 */
uint32_t flex_create_biw_ssid2(uint32_t country_code, uint32_t tmf)
{
	uint32_t dw = 0;
	/* type field bits 4-6 = 111 (SSID2) */
	dw |= (uint32_t)FLEX_BIW_TYPE_SSID2 << FLEX_BIW_TYPE_SHIFT;
	dw |= (tmf          & FLEX_BIW_SSID2_TMF_MASK)     << FLEX_BIW_SSID2_TMF_SHIFT;
	dw |= (country_code & FLEX_BIW_SSID2_COUNTRY_MASK)  << FLEX_BIW_SSID2_COUNTRY_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word — System Information (type 101).
 *
 * BIW word layout (21 data bits):
 *   bits 0-3:   checksum (x)
 *   bits 4-6:   type = 101 (SysInfo)
 *   bits 7-10:  A3-A0 system message type
 *   bits 11-20: I9-I0 system info data (layout depends on A-type)
 *
 * a_type: A3-A0 (4 bits). Use FLEX_BIW_SYSINFO_A_* constants:
 *   0000 = SysMsg for all pagers      0001 = SysMsg for home pagers
 *   0010 = SysMsg for roaming pagers  0011 = SysMsg for SSID pagers
 *   0100 = Time instruction           0101 = Additional time instruction
 *   0110 = Channel Set Up instruction 0111-1111 = Reserved
 *
 * info: I9-I0 (10 bits). Layout per A-type:
 *
 *   A=0100/0101 (Time):
 *     I4-I0: timezone zone code Z4-Z0 (5 bits, 0-31)
 *     I5:    L0 DST flag (0=DST active, 1=standard time)
 *     I6:    reserved (0)
 *     I7-I9: S5-S3 extended seconds (3 bits, 0.9375s steps)
 *     Example: zone=14 (UTC+0), DST=1, esec=0 → info=0x034
 *
 *   A=0110 (Channel Set Up):
 *     I5-I0: F5-F0 Frame Offset (6 bits, 1-63)
 *     I7-I6: O1-O0 Max Carry On for roaming pagers (2 bits, 0-3)
 *     I8:    N0 NID System Message Bit (1=NID sysmsg present)
 *     I9:    B0 System Message Bit (1=sysmsg present in frame)
 *     Example: frame_ofs=10, carry_on=2, nid=0, sysmsg=1 → info=0x28A
 *
 *   A=0000~0011 (System Messages):
 *     I-field is message-type-specific. When used, vectors (except
 *     Secure) go at end of VF, message body in MF.
 */
uint32_t flex_create_biw_sysinfo(uint32_t a_type, uint32_t info)
{
	uint32_t dw = 0;
	/* type field bits 4-6 = 101 (SysInfo) */
	dw |= (uint32_t)FLEX_BIW_TYPE_SYSINFO << FLEX_BIW_TYPE_SHIFT;
	dw |= (a_type & FLEX_BIW_SYSINFO_A_MASK) << FLEX_BIW_SYSINFO_A_SHIFT;
	dw |= (info   & FLEX_BIW_SYSINFO_I_MASK) << FLEX_BIW_SYSINFO_I_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Generate a standalone ERS (Emergency Re-Synchronization) burst.
 *
 * Used by network mode to stream the ERS burst during FLEX_STATE_NET_ERS
 * without buffering the entire burst into the frame buffer.
 *
 * Each ERS cycle = BS(2) + AR(4) + BS_inv(2) + AR_inv(4) = 12 bytes (96 bits).
 * Returns bytes written (cycles * 12), or 0 on error.
 *
 * Requirements: 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7
 */
size_t flex_generate_ers(uint8_t *buffer, size_t buffer_size, int cycles)
{
	uint8_t *out;
	int i;
	size_t needed;

	if (!buffer || cycles <= 0)
		return 0;

	needed = (size_t)cycles * 12;
	if (buffer_size < needed)
		return 0;

	out = buffer;
	for (i = 0; i < cycles; i++) {
		EMIT_SYNC(out, sync_bs);
		EMIT_SYNC(out, sync_ar);
		EMIT_SYNC(out, sync_bs_inv);
		EMIT_SYNC(out, sync_ar_inv);
	}

	return needed;
}

/* Split a long message into fragments (§4.1).
 *
 * Per §4.1: when the full length of a message cannot be contained in one
 * Frame, the individual message must be broken down into fragments.  For
 * an Alphanumeric Message with a Short Address using single transmission,
 * after removing one BIW, one address word, and one vector word from the
 * maximum 88 words, the maximum length of message words is 85.  After the
 * 1st word (header) is removed, the maximum number of words for the message
 * is 84 words.  As 3 characters can be sent per word, the maximum number
 * of characters which can be sent in one Frame is 251 [3 chars/word × 84
 * words - 1 = 251].  Longer messages are transmitted by Fragmentation.
 *
 * Returns the number of fragments created. Each fragment is a newly
 * allocated string (caller must free). If the message fits in one
 * fragment, return 1 with fragments[0] = strdup(message).
 *
 * Fragmentation applies to Alpha (V=101), HEX/Binary (V=010), and Secure
 * (V=000) message types.  Per §4.2 ③: Numeric Messages (V=011, V=100,
 * V=111) cannot be fragmented.
 *
 * Each fragment carries:
 *   F: 2-bit Fragment Number — modulo 3 sequence (11, 00, 01, 10, 00, ...).
 *      "11" is skipped after the first to avoid ambiguity.
 *   C: Message Continued Flag — 1 for all but the last fragment, which
 *      resets C to 0 to indicate the end of the fragment stream.
 *   N: 6-bit Message Number (0-63) — same across all fragments.
 *
 * The caller (flex_fragment_queue) handles assigning retrieval numbers,
 * fragment_index, total_fragments, and C/F flags on each flex_msg_t. */
int flex_fragment_message(const char *message, int msg_type,
			  int max_chars_per_fragment,
			  char **fragments, int max_fragments)
{
	int len, count, i, chunk;

	(void)msg_type; /* reserved for future per-type fragment sizing */

	if (!message || !fragments || max_fragments <= 0 || max_chars_per_fragment <= 0)
		return 0;

	len = (int)strlen(message);

	/* Message fits in a single fragment */
	if (len <= max_chars_per_fragment) {
		fragments[0] = strdup(message);
		if (!fragments[0])
			return 0;
		return 1;
	}

	/* Compute number of fragments needed */
	count = (len + max_chars_per_fragment - 1) / max_chars_per_fragment;
	if (count > max_fragments)
		count = max_fragments;

	for (i = 0; i < count; i++) {
		int offset = i * max_chars_per_fragment;
		int remaining = len - offset;

		chunk = (remaining < max_chars_per_fragment) ? remaining : max_chars_per_fragment;

		fragments[i] = malloc(chunk + 1);
		if (!fragments[i]) {
			/* Cleanup on allocation failure */
			int j;
			for (j = 0; j < i; j++)
				free(fragments[j]);
			return 0;
		}
		memcpy(fragments[i], message + offset, chunk);
		fragments[i][chunk] = '\0';
	}

	return count;
}

/* ===== Temporary Address Assignment ===== */

/*
 * Encode a temporary address word for the address field.
 * Base address = 0x1F7800 (1 1111 0111 1000 0000 0000₂).
 *
 * The Temporary Address is obtained by adding binary 0000 through 1111
 * (as indicated by a₃~a₀ of the Short Instruction Vector) to the base.
 *
 * temp_addr: the 21-bit address word value (FLEX_ADDR_TEMPORARY_MIN + slot).
 *
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid input.
 */
uint32_t flex_encode_temp_address(uint32_t temp_addr)
{
	uint32_t dw;

	/* Validate: must be in the temporary address range */
	if (temp_addr < FLEX_ADDR_TEMPORARY_MIN || temp_addr > FLEX_ADDR_TEMPORARY_MAX)
		return 0;

	/* Encode the temporary address value as a 21-bit BCH word */
	dw = temp_addr & FLEX_DATA_MASK;

	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Network Address Encoding ===== */

/*
 * Encode a network (NID) address word.
 *
 * Network addresses occupy the range FLEX_ADDR_NETWORK_MIN (0x1F6800)
 * to FLEX_ADDR_NETWORK_MAX (0x1F77FF) — 4096 addresses.
 * The addr_offset parameter selects which address within that range (0-based).
 *
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid offset.
 */
uint32_t flex_encode_network_address(uint32_t addr_offset)
{
	uint32_t aw;

	if (addr_offset > (FLEX_ADDR_NETWORK_MAX - FLEX_ADDR_NETWORK_MIN))
		return 0;

	aw = (FLEX_ADDR_NETWORK_MIN + addr_offset) & FLEX_DATA_MASK;
	return flex_encode_word(reverse_bits32(aw));
}

/*
 * Encode a network message payload word.
 *
 * Packs Service Area ID, Coverage Zone Count, and Traffic Management
 * Flags into a single 21-bit message word.
 *
 * Bit layout:
 *   bits 0-11:  Service Area ID (12 bits, 0-4095)
 *   bits 12-15: Coverage Zone Count (4 bits, 0-15)
 *   bits 16-20: Traffic Management Flags (5 bits)
 */
uint32_t flex_encode_network_payload(uint32_t area_id, uint32_t coverage_zones,
				     uint32_t traffic_flags)
{
	uint32_t dw = 0;

	dw |= (area_id & FLEX_NET_AREA_ID_MASK);
	dw |= ((coverage_zones & FLEX_NET_COVERAGE_MASK) << FLEX_NET_COVERAGE_SHIFT);
	dw |= ((traffic_flags & FLEX_NET_TRAFFIC_MASK) << FLEX_NET_TRAFFIC_SHIFT);

	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Operator Messaging Address Encoding ===== */

/*
 * Encode an operator messaging address word.
 *
 * System Messages can be
 * transmitted in three ways:
 *   (a) BIW101 only: vectors at end of VF, messages in MF.
 *   (b) BIW101 + Operator Messaging Address: both BIW and operator
 *       address in AF, operator vector in VF, message in MF.
 *   (c) Operator Messaging Address only: operator address in AF,
 *       operator vector in VF, message in MF (no BIW101 needed).
 *
 * The sub-type LSB (4 bits) selects the specific function:
 *   0x00: System Message for all pagers (content per BIW)
 *   0x01: System Message for home area pagers
 *   0x02: System Message for roaming pagers
 *   0x03: System Message for SSID pagers
 *   0x04: Time related Message for all pagers
 *   0x05-0x0D: Reserved
 *   0x0E: SSID Change Instruction — transmits TMF split / new frequency
 *          info to SSID pagers.  Mandatory for TMF-split systems on
 *          roaming frames, 5 cycles around the change.
 *   0x0F: System Event Notification — pre-alerts pagers about changes
 *          within next 4 cycles.  Used with Short Instruction Vector
 *          (type 001).  Events: SSID/NID TMF split, channel setup
 *          changes, new NID/SSID frequency.
 *
 * FIFO usage: "sysmsg,<lsb>,,<payload>" sends via mode (c).
 *   Example: "sysmsg,0,,System maintenance" → capcode 2062352 (0x1F7810)
 *   Example: "sysmsg,14,,"                  → capcode 2062366 (0x1F781E)
 *
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid sub-type.
 */
uint32_t flex_encode_oper_msg_address(uint32_t subtype_lsb)
{
	uint32_t aw;

	if (subtype_lsb > FLEX_OPER_MSG_LSB_MASK)
		return 0;

	aw = (FLEX_ADDR_OPER_MSG_MIN + subtype_lsb) & FLEX_DATA_MASK;
	return flex_encode_word(reverse_bits32(aw));
}

/* ===== Hex/Binary Vector and Message Encoding ===== */

/*
 * Create a hex/binary vector word.
 *
 * The hex vector uses the same type code (011) as numeric but with
 * k-bits = 0110 to indicate hex/binary mode.
 * This distinguishes it from standard numeric vectors which use
 * different k-bit values.
 *
 * Bit layout (21-bit data word before BCH encoding):
 *   bits  4-6:  vector type (110 = 0x6, HEX/Binary)
 *   bits  7-13: message start word offset
 *   bits 14-20: message word count (7 bits)
 *
 * Previously this used type 0x3 (numeric) with k-bits=0110 to signal
 * hex mode.  Corrected to type 0x6 per the standard and reference
 * decoders (PDW MODE_BINARY=6, multimon-ng FLEX_PAGETYPE_BINARY=6).
 */
static uint32_t create_hex_vector(uint32_t msg_start, uint32_t msg_words)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_HEX_BINARY & FLEX_VEC_TYPE_MASK) << FLEX_VEC_TYPE_SHIFT;
	dw |= (msg_start                   & FLEX_VEC_START_MASK) << FLEX_VEC_START_SHIFT;
	dw |= (msg_words                   & FLEX_VEC_LEN_MASK)   << FLEX_VEC_LEN_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Validate that a character is a valid hexadecimal digit.
 * Returns 1 for 0-9, A-F, a-f; 0 otherwise.
 */
static int is_valid_hex_char(char c)
{
	return (c >= '0' && c <= '9') ||
	       (c >= 'A' && c <= 'F') ||
	       (c >= 'a' && c <= 'f');
}

/*
 * Convert a hex character to its 4-bit nibble value.
 * Caller must ensure c is a valid hex character.
 */
static uint8_t hex_char_to_nibble(char c)
{
	if (c >= '0' && c <= '9')
		return (uint8_t)(c - '0');
	if (c >= 'A' && c <= 'F')
		return (uint8_t)(c - 'A' + 10);
	/* a-f */
	return (uint8_t)(c - 'a' + 10);
}

/*
 * Encode hex/binary message.
 *
 * Hex characters are converted to 4-bit nibbles and packed 5 per
 * 21-bit message word (5 × 4 = 20 bits used, 1 bit unused).
 *
 * Packing order within each word:
 *   bits  0-3:  nibble 0 (first hex char)
 *   bits  4-7:  nibble 1
 *   bits  8-11: nibble 2
 *   bits 12-15: nibble 3
 *   bits 16-19: nibble 4
 *   bit  20:    unused (0)
 *
 * Returns the encoded vector word.  Only body words are written to
 * frame_words starting at *fwc_p.
 *
 * On invalid input (non-hex characters), sets *error = -FLEX_ERR_INVALID_MESSAGE
 * and returns 0 without writing any words.
 */
static uint32_t encode_hex_message(uint32_t *frame_words, const char *msg,
			       uint32_t msg_start, uint32_t *fwc_p,
			       int sequence_num,
			       const void *config, int *error)
{
	uint32_t msg_words[FLEX_MAX_MSG_WORDS_HEX];
	uint32_t fwc;
	size_t len, i;
	int word_idx, data_idx;
	int frag_idx = 0, frag_total = 0;
	int is_initial_frag;
	uint32_t f_val, k_sum;
	int c_val;

	if (config) {
		const struct flex_msg_config *cfg = config;
		frag_idx = cfg->fragment_index;
		frag_total = cfg->total_fragments;
	}

	is_initial_frag = (frag_total <= 1 || frag_idx == 0);
	f_val = flex_fragment_number(frag_idx);
	c_val = (frag_total > 1 && frag_idx < frag_total - 1) ? 1 : 0;

	/* Validate all characters are hex */
	len = strlen(msg);
	for (i = 0; i < len; i++) {
		if (!is_valid_hex_char(msg[i])) {
			*error = -FLEX_ERR_INVALID_MESSAGE;
			return 0;
		}
	}

	if (len > FLEX_MAX_CHARS_HEX)
		len = FLEX_MAX_CHARS_HEX;

	/* Build message words array:
	 *   word 0:     header1 (K/C/F/N — K filled last)
	 *   word 1:     header2 (R/M/D/H/B/I/s/S — initial fragment only)
	 *   word 2+:    data (5 hex digits per 21-bit word)
	 * Continuation fragments skip header2, so data starts at word 1. */
	memset(msg_words, 0, sizeof(msg_words));

	/* Header word 1: C, F, N (K computed last).
	 *
	 * F: Message Fragment Number (2 bits, §3.10.1.2).
	 *   F is a modulo 3 message fragment number which increments by 1
	 *   for each of the consecutive fragments.  The first fragment
	 *   starts with "11" and is incremented by 1 modulo 3 for each of
	 *   the subsequent fragments (11, 00, 01, 10, 00, 01, 10, 00, ...).
	 *   The state for "11" after the first fragment is skipped in order
	 *   to prevent it from being mistaken as the first fragment of a
	 *   non-consecutive message.
	 *
	 * C: Message Continued Flag (1 bit).
	 *   The last fragment is indicated by resetting the Message
	 *   Continued Flag (C) to 0.  C=1 means more fragments follow.
	 *
	 * N: Message Number (6 bits, 0-63).
	 *   Identifies the fragment stream — same N across all fragments
	 *   of one message. */
	msg_words[0] = (f_val << FLEX_HEX_HDR_F_SHIFT);
	if (c_val)
		msg_words[0] |= FLEX_HEX_HDR_C_MASK;
	if (sequence_num >= 0) {
		uint32_t n = (uint32_t)(sequence_num % 64);
		msg_words[0] |= (n << FLEX_HEX_HDR_N_SHIFT);
	}

	/* Header word 2: initial fragment only */
	if (is_initial_frag) {
		/* R=retrieval, M=mail_drop, D=0 (LTR), H=0,
		 * B=blocking_length (bits/char), I=0, s=0, S=0 (signature) */
		int b_field = 1; /* default B=0001 (1 bit/char) per spec */
		if (config) {
			const struct flex_msg_config *cfg = config;
			b_field = cfg->blocking_length & 0xF;
		}
		msg_words[1] = 0;
		msg_words[1] |= ((uint32_t)b_field << FLEX_HEX_HDR2_B_SHIFT);
		if (sequence_num >= 0 && config) {
			const struct flex_msg_config *cfg = config;
			if (cfg->hex_r_flag)
				msg_words[1] |= FLEX_HEX_HDR2_R_MASK;
		}
		if (config) {
			const struct flex_msg_config *cfg = config;
			if (cfg->mail_drop)
				msg_words[1] |= FLEX_HEX_HDR2_M_MASK;
		}
		data_idx = 2;

		/* Log blocking length and data metrics */
		{
			int bl = b_field ? b_field : 16;
			int data_bits = (int)len * 4;
			int blocks = (bl > 0) ? (data_bits + bl - 1) / bl : 0;
			LOGP(DFLEX, LOGL_DEBUG,
			     "TX: HEX/Binary encoder: B=%d (%d bits/char), "
			     "data=%d bits, %d blocks\n",
			     b_field, bl, data_bits, blocks);
		}
	} else {
		data_idx = 1;
	}

	/* Pack hex nibbles as a continuous bit stream across 21-bit words.
	 * Per standard Fig 3.10.1.2-1, data bits fill all 21 bits of each
	 * word with nibble boundaries crossing word boundaries freely.
	 *
	 * Termination (Section 3.10.1.2): remaining bits after the last
	 * data nibble are filled with the inverse of the last data bit.
	 * If the last nibble is all-0 or all-1, an extra word of inverse
	 * fill is appended. */
	{
		int bit = data_idx * FLEX_BCH_DATA_BITS;
		int b;

		for (i = 0; i < len; i++) {
			uint8_t nibble = hex_char_to_nibble(msg[i]);
			for (b = 0; b < 4; b++) {
				int wi = bit / FLEX_BCH_DATA_BITS;
				if (wi >= FLEX_MAX_MSG_WORDS_HEX)
					break;
				if (nibble & (1 << b))
					msg_words[wi] |= (1U << (bit % FLEX_BCH_DATA_BITS));
				bit++;
			}
		}

		word_idx = (bit > 0) ? (bit - 1) / FLEX_BCH_DATA_BITS : data_idx;

		/* Termination fill (Spec §3.10.1.2 rules 2 & 3):
		 *
		 * Rule (2): When data ends in the middle of the last word,
		 * fill remaining bits with the inverse of the last data bit.
		 * When data ends exactly at a word boundary, no fill needed.
		 *
		 * Rule (3): For the LAST fragment only (C=0): same as (2),
		 * EXCEPT when data ends exactly at a word boundary AND the
		 * last character is all-0s or all-1s — in that case, append
		 * an extra word filled with the inverse of the last data bit.
		 *
		 * The extra word prevents ambiguity: without it, the decoder
		 * can't distinguish "message ended here" from "message data
		 * that happens to look like fill". */
		if (bit > data_idx * FLEX_BCH_DATA_BITS) {
			int last_data_bit = (msg_words[(bit - 1) / FLEX_BCH_DATA_BITS]
					     >> ((bit - 1) % FLEX_BCH_DATA_BITS)) & 1;
			uint32_t fill_bit = last_data_bit ? 0 : 1;
			int remainder = bit % FLEX_BCH_DATA_BITS;

			if (remainder != 0) {
				/* Data ends mid-word: fill remaining bits */
				int end_bit = (word_idx + 1) * FLEX_BCH_DATA_BITS;
				for (b = bit; b < end_bit; b++) {
					if (fill_bit)
						msg_words[b / FLEX_BCH_DATA_BITS] |=
							(1U << (b % FLEX_BCH_DATA_BITS));
				}
			}
			word_idx++;

			/* Rule (3): extra word only when last fragment,
			 * data ends at word boundary, and last char is
			 * all-0s or all-1s */
			if (c_val == 0 && remainder == 0 &&
			    word_idx < FLEX_MAX_MSG_WORDS_HEX) {
				uint8_t last_nib = hex_char_to_nibble(msg[len - 1]);
				if (last_nib == 0x0 || last_nib == 0xF) {
					uint32_t extra = 0;
					if (fill_bit)
						for (b = 0; b < 21; b++)
							extra |= (1U << b);
					msg_words[word_idx] = extra;
					word_idx++;
				}
			}
		}
	}

	/* S: 8-bit message signature (header2 bits 13-20).
	 * Only present on initial fragment.
	 *
	 * Per spec: "Signature is defined as the 1's complement of binary
	 * sum for the entire message (including all fragments) for every
	 * 8 bits, beginning with the first 8 bits which follow directly
	 * after the Signature Field.  The 8 LSB of the result is
	 * transmitted as the Message Signature."
	 *
	 * For a single-fragment message, the data words start at index 2
	 * (after hdr1 and hdr2).  We sum every 8 bits of the raw data
	 * words (excluding termination fill bits in the last word).
	 * Termination bits are NOT included in the calculation.
	 *
	 * The bit stream is taken from the data words only (not headers),
	 * packed as a flat bit sequence: word[data_idx] bits 0-20, then
	 * word[data_idx+1] bits 0-20, etc.  We group these into 8-bit
	 * chunks and sum them, then take the 1's complement (8-bit). */
	if (is_initial_frag) {
		/* Use precomputed whole-message signature.
		 * Always set by flex_msg_create or flex_encode_frame. */
		if (config) {
			const struct flex_msg_config *cfg = config;
			msg_words[1] |= (cfg->precomputed_sig & 0xFF) << FLEX_HEX_HDR2_S_SHIFT;
		}
	}

	/* K: 12-bit fragment checksum.
	 * 1's complement of binary sum of all information bits in the
	 * fragment, taken as three groups per word: bits 0-7, 8-15, 16-20.
	 * Same algorithm as alpha but 12-bit mask.
	 * Computed last since it covers all other fields including S. */
	k_sum = 0;
	for (i = 0; i < (size_t)word_idx; i++) {
		k_sum += msg_words[i] & 0xFFU;
		k_sum += (msg_words[i] >> 8) & 0xFFU;
		k_sum += (msg_words[i] >> 16) & 0x1FU;
	}
	msg_words[0] |= (~k_sum) & FLEX_HEX_HDR_K_MASK;

	/* Write encoded body words to frame (vector word returned to caller).
	 * For long addresses,
	 * body[0] is placed at Vy in the Vector Field by the caller. */
	fwc = *fwc_p;
	{
		char dbg[512];
		int dpos = 0;
		dpos += snprintf(dbg + dpos, sizeof(dbg) - dpos,
				 "TX: HEX body: start=%u words=%d data_idx=%d nibbles=%d:",
				 msg_start, word_idx, data_idx, (int)len);
		for (i = 0; i < (size_t)word_idx && dpos < (int)sizeof(dbg) - 10; i++)
			dpos += snprintf(dbg + dpos, sizeof(dbg) - dpos,
					 " [%d]=0x%05X", (int)i, msg_words[i] & FLEX_DATA_MASK);
		LOGP(DFLEX, LOGL_INFO, "%s\n", dbg);
	}
	for (i = 0; i < (size_t)word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_words[i]));
	*fwc_p = fwc;

	return create_hex_vector(msg_start, word_idx);
}

/*
 * Create a short instruction vector word.
 *
 * The short instruction vector encodes instruction data directly in the
 * vector word — no message body words are needed (similar to tone-only).
 *
 * Bit layout of the 21-bit data word:
 *   bits  0-3:  checksum (filled by flex_word_checksum)
 *   bits  4-6:  vector type = 001 (0x1 = FLEX_VECTOR_TYPE_SHORT_INSTR)
 *   bits  7-20: instruction data (14 bits, range 0-16383)
 */
static uint32_t create_short_instruction_vector(uint32_t instruction_data)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_SHORT_INSTR & FLEX_VEC_TYPE_MASK) << FLEX_VEC_TYPE_SHIFT;
	dw |= (instruction_data & FLEX_VEC_INSTR_MASK) << FLEX_VEC_INSTR_SHIFT;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Encode a short instruction message.
 *
 * Parses the message string as a decimal integer instruction value,
 * creates the short instruction vector word, and writes it to frame_words.
 * No message body words are produced — all data is in the vector word.
 *
 * On invalid input (non-numeric string, value out of 14-bit range, or
 * NULL/empty message), sets *error = -FLEX_ERR_INVALID_MESSAGE and
 * returns without writing any words.
 */
/*
 * Encode a short instruction message.
 *
 * Parses the message string as a decimal integer instruction value,
 * creates the short instruction vector word.  No message body words
 * are produced — all data is in the vector word.
 *
 * Returns the encoded vector word, or 0 on error.
 * On invalid input (non-numeric string, value out of 14-bit range, or
 * NULL/empty message), sets *error = -FLEX_ERR_INVALID_MESSAGE.
 */
static uint32_t encode_instruction_message(const char *msg, int *error)
{
	unsigned long val;
	char *endptr;

	if (!msg || !*msg) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/* Parse as decimal integer */
	val = strtoul(msg, &endptr, 10);

	/* Reject if not a clean integer or has trailing garbage */
	if (*endptr != '\0') {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/* 14-bit range: 0-16383 */
	if (val > 0x3FFF) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	return create_short_instruction_vector((uint32_t)val);
}

/* ===== Multi-Message Frame Encoding ===== */

/* Forward declaration — defined after flex_encode_frame_multi(). */
static uint32_t encode_kanji_message(uint32_t *frame_words, const char *msg,
				 uint32_t msg_start, uint32_t *fwc_p,
				 int *error);

/*
 * Estimate the number of message body words for a given message type and content.
 * Used in the first pass to determine if a message fits in the remaining frame capacity.
 *
 * Returns the number of 21-bit message body words (excluding the vector word).
 * Returns -1 on error (invalid message content for the given type).
 */
static int estimate_msg_words(const flex_frame_msg_t *msg)
{
	size_t len;

	switch (msg->msg_type) {
	case FLEX_FRAME_MSG_TYPE_TONE:
		return 0; /* vector word only */

	case FLEX_FRAME_MSG_TYPE_INSTRUCTION:
		return 0; /* vector word only */

	case FLEX_FRAME_MSG_TYPE_SHORT:
		return 0; /* vector word only */

	case FLEX_FRAME_MSG_TYPE_ALPHA:
		if (!msg->message || !msg->message[0])
			return -1;
		len = (msg->message_length > 0)
			? (size_t)msg->message_length
			: strlen(msg->message);

		if (msg->charset == 1) {
			/* KANJI: 16-bit chars (2 bytes each), 1 char per word.
			 * Total = 1 (header) + num_chars. */
			size_t num_chars = len / 2;
			size_t max_chars = FLEX_MAX_MSG_WORDS_ALPHA - 1;
			if (num_chars == 0)
				return -1;
			if (num_chars > max_chars)
				num_chars = max_chars;
			return 1 + (int)num_chars;
		}

		if (len > FLEX_MAX_CHARS_ALPHA)
			return -1;
		/*
		 * Alpha packing: word 0 = header (frag flags + checksum).
		 *
		 * Initial fragment (F=11) or unfragmented:
		 *   Word 1 holds signature (bits 0-6) + 2 chars (bits 7-13, 14-20).
		 *   Subsequent words hold 3 chars each (shifts 0, 7, 14).
		 *   Total = 1 (header) + ceil((len + 2) / 3).
		 *   The +2 accounts for the signature slot eating one char position.
		 *
		 * Continuation fragment (F≠11):
		 *   No signature field — all 3 char slots available from word 1.
		 *   Total = 1 (header) + ceil(len / 3).
		 */
		{
			int is_continuation = (msg->total_fragments > 1 &&
					       msg->fragment_index > 0);
			if (is_continuation)
				return 1 + (int)((len + 2) / 3);
			else
				return 1 + (int)((len + 2 + 2) / 3);
		}

	case FLEX_FRAME_MSG_TYPE_NUMERIC:
		if (!msg->message || !msg->message[0])
			return -1;
		len = (msg->message_length > 0)
			? (size_t)msg->message_length
			: strlen(msg->message);
		if (len > FLEX_MAX_CHARS_NUMERIC)
			return -1;
		/*
		 * Numeric packing: 4-bit nibbles, 5 per 21-bit word.
		 * First 2 bits reserved for checksum, so first word holds
		 * floor((21-2)/4) = 4 nibbles effectively, but the encoder
		 * packs continuously. Total = ceil((len * 4 + 2) / 21).
		 * Simplified: ceil((len + 1) / 5) works for small counts,
		 * but the encoder caps at FLEX_MAX_MSG_WORDS_NUMERIC (8).
		 */
		{
			int bits_needed = (int)len * 4 + 2;
			int words = (bits_needed + 20) / 21;
			if (words > FLEX_MAX_MSG_WORDS_NUMERIC)
				words = FLEX_MAX_MSG_WORDS_NUMERIC;
			if (words < 1)
				words = 1;
			return words;
		}

	case FLEX_FRAME_MSG_TYPE_HEX:
		if (!msg->message || !msg->message[0])
			return -1;
		len = (msg->message_length > 0)
			? (size_t)msg->message_length
			: strlen(msg->message);
		if (len > FLEX_MAX_CHARS_HEX)
			return -1;
		/* Hex packing: 5 nibbles per 21-bit data word.
		 * Header words: hdr1 (always) + hdr2 (initial fragment only).
		 * Initial fragment: 2 headers + ceil(len/5) data words.
		 * Continuation:     1 header  + ceil(len/5) data words.
		 *
		 * Termination: if the last nibble is all-0
		 * or all-1 (0x0 or 0xF), an extra fill word may be appended.
		 * We add +1 worst-case to ensure the frame has room. */
		{
			int is_continuation = (msg->total_fragments > 1 &&
					       msg->fragment_index > 0);
			int hdr_words = is_continuation ? 1 : 2;
			int data_words = (int)((len + 4) / 5);
			int is_last_frag = (msg->total_fragments <= 1 ||
					    msg->fragment_index == msg->total_fragments - 1);
			/* Extra termination word possible on last fragment */
			if (is_last_frag && len > 0) {
				const char *m = msg->message;
				char last_ch = m[len - 1];
				if (last_ch == '0' || last_ch == 'f' || last_ch == 'F')
					data_words++;
			}
			return hdr_words + data_words;
		}

	case FLEX_FRAME_MSG_TYPE_SECURE:
		if (!msg->message || !msg->message[0])
			return -1;
		len = (msg->message_length > 0)
			? (size_t)msg->message_length
			: strlen(msg->message);
		if (msg->secure_encoding == 1) {
			/* Binary encoding: same estimation as hex */
			if (len > FLEX_MAX_CHARS_HEX)
				return -1;
			{
				int is_continuation = (msg->total_fragments > 1 &&
						       msg->fragment_index > 0);
				int hdr_words = is_continuation ? 1 : 2;
				int data_words = (int)((len + 4) / 5);
				int is_last_frag = (msg->total_fragments <= 1 ||
						    msg->fragment_index == msg->total_fragments - 1);
				if (is_last_frag && len > 0) {
					const char *m = msg->message;
					char last_ch = m[len - 1];
					if (last_ch == '0' || last_ch == 'f' || last_ch == 'F')
						data_words++;
				}
				return hdr_words + data_words;
			}
		} else {
			/* Alpha encoding: same estimation as alpha */
			if (len > FLEX_MAX_CHARS_ALPHA)
				return -1;
			{
				int is_continuation = (msg->total_fragments > 1 &&
						       msg->fragment_index > 0);
				if (is_continuation)
					return 1 + (int)((len + 2) / 3);
				else
					return 1 + (int)((len + 2 + 2) / 3);
			}
		}

	case FLEX_FRAME_MSG_TYPE_SPECIAL_NUM:
		if (!msg->message || !msg->message[0])
			return -1;
		len = (msg->message_length > 0)
			? (size_t)msg->message_length
			: strlen(msg->message);
		if (len > FLEX_MAX_CHARS_NUMERIC)
			return -1;
		/* Same as standard numeric: ceil((len*4+2)/21) */
		{
			int bits_needed = (int)len * 4 + 2;
			int words = (bits_needed + 20) / 21;
			if (words > FLEX_MAX_MSG_WORDS_NUMERIC)
				words = FLEX_MAX_MSG_WORDS_NUMERIC;
			if (words < 1)
				words = 1;
			return words;
		}

	case FLEX_FRAME_MSG_TYPE_NUMBERED_NUM:
		if (!msg->message || !msg->message[0])
			return -1;
		len = (msg->message_length > 0)
			? (size_t)msg->message_length
			: strlen(msg->message);
		if (len > 39)
			return -1;
		/* ceil((len*4+10)/21) — 10-bit header (K5K4 + N + R + S) */
		{
			int bits_needed = (int)len * 4 + FLEX_NUM_NUMBERED_SKIP_BITS;
			int words = (bits_needed + 20) / 21;
			if (words > FLEX_MAX_MSG_WORDS_NUMERIC)
				words = FLEX_MAX_MSG_WORDS_NUMERIC;
			if (words < 1)
				words = 1;
			return words;
		}

	default:
		return -1;
	}
}

/*
 * Encode a FLEX frame with multiple messages.
 *
 * Frame layout (88-word data area):
 *   Word 0:                          BIW1
 *   Word 1:                          BIW2 (if biw_time or local_id/coverage_id set)
 *   Word 2:                          BIW3 (if biw_time set)
 *   Word 3:                          BIW4 (if biw_time set)
 *   Words e_biw+1..s_vfield-1:       Address words (priority first, then normal)
 *   Words s_vfield..msg_start-1:     Vector words
 *   Words msg_start..87:             Message data words + idle fill
 *
 * Algorithm:
 *   1. Write sync section (S1 + FIW + S2) to buffer
 *   2. Compute BIW count based on params
 *   3. First pass: compute per-message word counts, greedily pack messages
 *   4. Write BIW words, address words, vector + message words
 *   5. Fill remaining with idle pattern, interleave, write to buffer
 *
 * Returns bytes written to buffer, or 0 on error.
 * Sets *msgs_packed to the number of messages that fit.
 * Sets *error to a negative FLEX_ERR_* code on failure.
 *
 * Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7, 6.4, 6.5, 10.3, 16.2, 16.3
 */

/* ===== Sync Component Helpers ===== */

/*
 * Encode S1: BS1(4) + Ax(4) + B(2) + Ax_inv(4) = 14 bytes.
 *
 * S1 is always transmitted at 1600/2FSK. The A code identifies the
 * frame's target speed/modulation:
 *   A1 = 1600/2FSK, A2 = 3200/2FSK, A3 = 3200/4FSK, A4 = 6400/4FSK
 *
 * Returns bytes written (14), or 0 on error.
 */
static size_t flex_encode_s1(int bitrate, int modulation_type,
			     uint8_t *buffer, size_t buffer_size)
{
	uint8_t *out;

	if (!buffer || buffer_size < 14)
		return 0;

	out = buffer;

	/* BS1: 32-bit alternating 1010... pattern */
	EMIT_SYNC(out, sync_bs1);

	/* A + B + A_inv: selected by target speed/modulation */
	switch (bitrate) {
	case 3200:
		if (modulation_type == FLEX_MOD_4FSK) {
			EMIT_SYNC(out, sync_a3);
			EMIT_SYNC(out, sync_b);
			EMIT_SYNC(out, sync_a3_inv);
		} else {
			EMIT_SYNC(out, sync_a2);
			EMIT_SYNC(out, sync_b);
			EMIT_SYNC(out, sync_a2_inv);
		}
		break;
	case 6400:
		EMIT_SYNC(out, sync_a4);
		EMIT_SYNC(out, sync_b);
		EMIT_SYNC(out, sync_a4_inv);
		break;
	default: /* 1600 */
		EMIT_SYNC(out, sync_a1);
		EMIT_SYNC(out, sync_b);
		EMIT_SYNC(out, sync_a1_inv);
		break;
	}

	return (size_t)(out - buffer);
}

/*
 * Encode FIW (Frame Information Word): 4 bytes (32-bit BCH codeword).
 *
 * Returns bytes written (4), or 0 on error.
 */
static size_t flex_encode_fiw(const flex_frame_params_t *params,
			      uint8_t *buffer, size_t buffer_size)
{
	uint8_t *out;

	if (!params || !buffer || buffer_size < 4)
		return 0;

	out = buffer;
	{
		/* Compute FIW r and t fields from multiple transmission params.
		 *
		 * FIW r and t fields (§3.6):
		 *   When num_transmissions=1: r=0, t3-t0 = Low Traffic Flags
		 *     per phase (t0=A, t1=B, t2=C, t3=D).
		 *     Flag=1 means address field ends within block 0 for
		 *     that phase — pager may sleep early.  Flag=0 = normal.
		 *   When num_transmissions>1: r=1, and:
		 *     [t1,t0] = num_transmissions (01=2x, 10=3x, 11=4x)
		 *     [t3,t2] = TD Collapse override (00=system, 01=6, 10=7, 11=5) */
		uint32_t r_flag = 0;
		uint32_t t_field = 0;

		if (params->num_transmissions > 1) {
			uint32_t t10, t32;

			r_flag = 1;

			switch (params->num_transmissions) {
			case 2:  t10 = 0x01; break;
			case 3:  t10 = 0x02; break;
			case 4:  t10 = 0x03; break;
			default: t10 = 0x00; break; /* reserved */
			}

			switch (params->td_collapse) {
			case 6:  t32 = 0x01; break;
			case 7:  t32 = 0x02; break;
			case 5:  t32 = 0x03; break;
			default: t32 = 0x00; break; /* use system collapse */
			}

			t_field = t10 | (t32 << 2);
		} else {
			/* Single transmission (r=0): use per-phase Low Traffic
			 * Flags computed by the scheduler (§3.6).
			 * Bit layout: t0=A, t1=B, t2=C, t3=D. */
			t_field = params->low_traffic_flags & FLEX_FIW_TRAFFIC_MASK;
		}

		uint32_t fiw_cw = flex_create_fiw(params->cycle, params->frame,
						  params->roaming, r_flag, t_field);
		EMIT_WORD(out, fiw_cw);
	}

	return 4;
}

/*
 * BS2 bit count for a given bitrate.
 *
 * Returns the number of BITS in one BS2 field, or 0 for invalid bitrate.
 * For 4FSK modes, this is symbol_count × 2 (dibits).
 *   1600bps/2FSK (A1):  4 sym × 1 bit/sym =  4 bits
 *   3200bps/2FSK (A2): 24 sym × 1 bit/sym = 24 bits
 *   3200bps/4FSK (A3): 12 sym × 2 bit/sym = 24 bits
 *   6400bps/4FSK (A4): 32 sym × 2 bit/sym = 64 bits
 */
static int flex_bs2_bits(int bitrate)
{
	switch (bitrate) {
	case 1600: return 4;
	case 3200: return 24;
	case 6400: return 64;
	default:   return 0;
	}
}

/*
 * Compute S2 byte size for a given speed/modulation without encoding.
 *
 * S2 structure: BS2(N) + C(16) + BS2_inv(N) + C_inv(16)
 * Total bits = 2*N + 32, where N varies per speed (see flex_bs2_bits).
 *
 * Returns byte count, or 0 for invalid bitrate.
 */
static size_t flex_s2_size(int bitrate)
{
	int n = flex_bs2_bits(bitrate);

	if (n == 0)
		return 0;

	return (size_t)(2 * n + 32 + 7) / 8;
}

/*
 * Encode S2 (second sync block) into buffer.
 *
 * S2 is the first thing transmitted at the frame's target data rate,
 * after the speed switch from 1600/2FSK (S1+FIW).
 * Structure: BS2 + C + inv.BS2 + inv.C
 *
 * S2 is always 25 ms at the data
 * symbol rate.  The buffer is encoded at the BIT level (after
 * symbol-to-dibit expansion for 4FSK modes).
 *
 * Per speed/modulation (from standard Tables 3.2-1 through 3.2-4):
 *
 *   Mode          BS2        C       inv.BS2   inv.C   Total
 *   ----          ---        -       -------   -----   -----
 *   1600bps/2FSK:  4 bits + 16 bits +  4 bits + 16 bits =  40 bits ( 5 bytes)
 *                  4 sym    16 sym     4 sym    16 sym   =  40 symbols @ 1600 baud
 *
 *   3200bps/2FSK: 24 bits + 16 bits + 24 bits + 16 bits =  80 bits (10 bytes)
 *                 24 sym    16 sym    24 sym    16 sym   =  80 symbols @ 3200 baud
 *
 *   3200bps/4FSK: 24 bits + 16 bits + 24 bits + 16 bits =  80 bits (10 bytes)
 *                 12 sym     8 sym    12 sym     8 sym   =  40 symbols @ 1600 baud
 *                 (each 4FSK symbol = 1 dibit = 2 bits)
 *
 *   6400bps/4FSK: 64 bits + 16 bits + 64 bits + 16 bits = 160 bits (20 bytes)
 *                 32 sym     8 sym    32 sym     8 sym   =  80 symbols @ 3200 baud
 *                 (each 4FSK symbol = 1 dibit = 2 bits)
 *
 * BS2 pattern:
 *   2FSK: alternating bits 1,0,1,0... (1 bit/symbol)
 *   4FSK: alternating 4-level symbols → dibits 10,00,10,00... (2 bits/symbol)
 *
 * C pattern: always 16 decoded bits = 0xED84, regardless of modulation.
 *   2FSK: 16 symbols (1 bit each)
 *   4FSK:  8 symbols (2 bits each, dibit pairs from the 16-bit pattern)
 *
 * Returns bytes written, or 0 on error.
 */
static size_t flex_encode_s2(int bitrate, int mod_type,
			     uint8_t *buffer, size_t buffer_size)
{
	int bs2_bits, bit_pos, i;
	size_t total_bits, total_bytes;
	int is_4fsk = (mod_type == FLEX_MOD_4FSK);

	bs2_bits = flex_bs2_bits(bitrate);
	if (bs2_bits == 0)
		return 0;

	total_bits = (size_t)bs2_bits + 16 + (size_t)bs2_bits + 16;
	total_bytes = (total_bits + 7) / 8;

	if (!buffer || buffer_size < total_bytes)
		return 0;

	memset(buffer, 0, total_bytes);
	bit_pos = 0;

	/* BS2: alternating comma pattern for PLL lock at new symbol rate.
	 * 2FSK: 1 bit/symbol, pattern = 1,0,1,0... (alternating bits)
	 * 4FSK: 2 bits/symbol (dibit), pattern = 10,00,10,00...
	 *   symbol "1" → dibit 10, symbol "0" → dibit 00
	 *   (full deviation comma on the channel, per standard appendix) */
	if (is_4fsk) {
		int sym;
		for (sym = 0; sym < bs2_bits / 2; sym++) {
			if ((sym & 1) == 0) {
				/* symbol 1 → dibit 10 */
				buffer[bit_pos / 8] |= (0x80 >> (bit_pos % 8));
			}
			/* symbol 0 → dibit 00 (nothing to set) */
			bit_pos += 2;
		}
	} else {
		for (i = 0; i < bs2_bits; i++) {
			if ((i & 1) == 0)
				buffer[bit_pos / 8] |= (0x80 >> (bit_pos % 8));
			bit_pos++;
		}
	}

	/* C: 16 decoded bits (0xED84), MSB first in the buffer.
	 * On air: 16 symbols for 2FSK, 8 symbols for 4FSK. */
	for (i = 15; i >= 0; i--) {
		if (FLEX_S2_C & (1 << i))
			buffer[bit_pos / 8] |= (0x80 >> (bit_pos % 8));
		bit_pos++;
	}

	/* inv.BS2: inverted alternating comma pattern.
	 * 2FSK: 0,1,0,1... (inverted bits)
	 * 4FSK: symbols 0,1,0,1... → dibits 00,10,00,10... */
	if (is_4fsk) {
		int sym;
		for (sym = 0; sym < bs2_bits / 2; sym++) {
			if ((sym & 1) == 1) {
				/* symbol 1 → dibit 10 */
				buffer[bit_pos / 8] |= (0x80 >> (bit_pos % 8));
			}
			/* symbol 0 → dibit 00 (nothing to set) */
			bit_pos += 2;
		}
	} else {
		for (i = 0; i < bs2_bits; i++) {
			if ((i & 1) == 1)
				buffer[bit_pos / 8] |= (0x80 >> (bit_pos % 8));
			bit_pos++;
		}
	}

	/* inv.C: 16 decoded bits (0x127B), MSB first in the buffer.
	 * On air: 16 symbols for 2FSK, 8 symbols for 4FSK. */
	for (i = 15; i >= 0; i--) {
		if (FLEX_S2_C_INV & (1 << i))
			buffer[bit_pos / 8] |= (0x80 >> (bit_pos % 8));
		bit_pos++;
	}

	return total_bytes;
}

size_t flex_encode_frame_multi(const flex_frame_msg_t *msgs, int msg_count,
			       const flex_frame_params_t *params,
			       uint8_t *buffer, size_t buffer_size,
			       int *msgs_packed, int *error)
{
	uint32_t frame_words[FLEX_WORDS_PER_FRAME];
	uint8_t *out;
	uint32_t fwc, i;
	int err_local = 0;

	/* Pre-fill frame with idle pattern per §3.4.1 Table 3.4.1-1:
	 * "Necessary information is written over the default for a block,
	 * while Idle blocks are maintained as they are."
	 * Pattern depends on phase and modulation — MSB phases use
	 * alternating all-1s/all-0s, LSB phases (4FSK) use all-zeros
	 * to produce the same 1600bps binary waveform on the channel. */
	flex_fill_idle_phase(frame_words, params->phase_index,
			     params->modulation_type, params->bitrate, params->collapse);

	/* Per-message bookkeeping for the packing pass */
	struct msg_info {
		int	addr_words;	/* 1 (short) or 2 (long) */
		int	vector_words;	/* 1 (short) or 2 (long) */
		int	msg_words;	/* body words (0 for tone/instruction/short) */
		int	is_long;	/* 1 if long capcode */
		int	packed;		/* 1 if included in this frame */
	} info[256]; /* max 256 messages per call — generous upper bound */

	/* Ordered indices: priority first, normal, tone-only last */
	int order[256];
	int n_prio = 0, n_norm = 0;
	int prio_addr_words = 0;
	int total_addr = 0, total_vector = 0, total_msg = 0;
	int biw_count, e_biw;
	int capacity;
	int packed_count = 0;

	/* ---- Validate inputs ---- */

	if (!error)
		error = &err_local;
	*error = 0;

	if (msgs_packed)
		*msgs_packed = 0;

	if (!buffer || buffer_size < FLEX_BUFFER_SIZE) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	if (!params) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	/* Clamp msg_count to our static array limit */
	if (msg_count > 256)
		msg_count = 256;

	out = buffer;

	/* ---- S1 + FIW + S2 ---- */
	out += flex_encode_s1(params->bitrate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));
	out += flex_encode_fiw(params, out,
			       buffer_size - (size_t)(out - buffer));
	out += flex_encode_s2(params->bitrate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));

	/* ---- Compute BIW count ---- */
	/* Build list of extra BIW words to emit (max 3, limited by
	 * BIW1 e_biw field = 2 bits).  The
	 * transmission order of BIW 2/3/4 is not regulated — receivers
	 * dispatch on the type field.  We prioritize:
	 *   1. SSID1 (type 000) — required for roaming
	 *   2. SSID2 (type 111) — required for roaming per spec
	 *   3. Date  (type 001) — if biw_time enabled
	 *   4. Time  (type 010) — if biw_time enabled
	 *   5. SysInfo/timezone (type 101) — if timezone_code set
	 * If more than 3 are needed, SysInfo is dropped (it can be
	 * transmitted in other frames per spec). */

	biw_count = 1; /* BIW1 always present */
	e_biw = 0;

	{
		int extra = 0;

		/* SSID1 (BIW000): emit when --ssid or --nid configured. */
		if (params->local_id || params->coverage_id)
			extra++;
		if ((params->country_code || params->tmf) && params->frame <= 3)
			extra++;
		if (params->biw_time)
			extra += 2; /* Date + Time */
		if (params->timezone_code >= 0 && params->timezone_code < (int)FLEX_TZ_ENTRIES)
			extra++;
		/* BIW101 for system message (method (b), §3.9.2).
		 * Reuses timezone slot if already counted. */
		if (params->sysmsg_a_type >= 0 && params->sysmsg_a_type <= 3) {
			if (params->timezone_code < 0 ||
			    params->timezone_code >= (int)FLEX_TZ_ENTRIES)
				extra++;
		}
		if (params->chan_setup_enabled && params->frame <= 3)
			extra++;

		/* Clamp to 3 (max e_biw) — drop SysInfo first if needed */
		if (extra > 3)
			extra = 3;

		e_biw = extra;
		biw_count = 1 + extra;
	}

	/* ---- First pass: compute per-message word counts ---- */

	if (msg_count > 0 && msgs) {
		for (i = 0; i < (uint32_t)msg_count; i++) {
			int is_long_addr = 0;
			int mw;

			/* Validate capcode */
			if (msgs[i].temp_delivery_slot >= 0) {
				/* Temp address delivery: single address word
				 * (FLEX_ADDR_TEMPORARY_MIN + slot) per §3.8.2.3 */
				info[i].addr_words = 1;
				info[i].is_long = 0;
			} else {
				if (!is_capcode_valid(msgs[i].capcode, &is_long_addr)) {
					LOGP(DFLEX, LOGL_ERROR,
					     "TX: rejecting invalid capcode %" PRIu64
					     " (not short 1-%u, not long %llu+, not special)\n",
					     msgs[i].capcode,
					     (unsigned)FLEX_SHORT_ADDR_MAX,
					     (unsigned long long)FLEX_LONG_ADDR_MIN);
					info[i].packed = 0;
					info[i].addr_words = 0;
					info[i].vector_words = 0;
					info[i].msg_words = 0;
					info[i].is_long = 0;
					continue;
				}
				info[i].addr_words = is_long_addr ? 2 : 1;
				info[i].is_long = is_long_addr;
			}

			/* Frame layout:
			 * (A) Short address: 1 addr word (Ax) → 1 vector word (Vx)
			 * (B) Long address:  2 addr words (Ax,Ay) → 2 vector words (Vx,Vy)
			 * Per Sections 3.9.1/3.9.3/3.9.4: "the 1st word of the
			 * message is placed at the 2nd word of the vector" — Vy
			 * holds body[0], not a copy of Vx. */
			info[i].vector_words = info[i].is_long ? 2 : 1;

			/* Method (a) system message: no address, no normal
			 * vector.  Body goes to MF, system message vector
			 * at end of VF is handled separately. */
			if (msgs[i].sysmsg_method == 'a') {
				info[i].addr_words = 0;
				info[i].vector_words = 0;
				info[i].is_long = 0;
			}

			/* Tone-only addresses have no vector word and no message
			 * body.
			 * They sit at the end of the address field. */
			if (msgs[i].msg_type == FLEX_FRAME_MSG_TYPE_TONE) {
				info[i].vector_words = 0;
				info[i].msg_words = 0;
				info[i].packed = 0;
				continue;
			}

			/* Estimate message body words */
			mw = estimate_msg_words(&msgs[i]);
			if (mw < 0) {
				LOGP(DFLEX, LOGL_ERROR,
				     "TX: rejecting invalid message for capcode %" PRIu64
				     " type=%d len=%d\n",
				     msgs[i].capcode, msgs[i].msg_type,
				     msgs[i].message_length);
				info[i].packed = 0;
				info[i].msg_words = 0;
				continue;
			}
			info[i].msg_words = mw;
			info[i].packed = 0;
		}
	}

	/* ---- Build ordering: priority first, normal, tone-only last ----
	 *
	 * "The Tone-Only Address is
	 * positioned at the end of the Address Field, as it does not
	 * require related vectors."  Tone-only addresses have no
	 * corresponding vector word, so they must come after all
	 * addresses that do have vectors.  The address/vector pairing
	 * (address/vector pairing) only covers the non-tone addresses. */

	int n_tone = 0;

	/* Pass 1: priority messages (non-tone) */
	for (i = 0; i < (uint32_t)msg_count; i++) {
		if (info[i].addr_words == 0 && info[i].vector_words == 0
		    && msgs[i].sysmsg_method != 'a')
			continue; /* skip invalid */
		if (msgs[i].priority && msgs[i].msg_type != FLEX_FRAME_MSG_TYPE_TONE)
			order[n_prio++] = (int)i;
	}
	/* Pass 2: normal messages (non-tone) */
	for (i = 0; i < (uint32_t)msg_count; i++) {
		if (info[i].addr_words == 0 && info[i].vector_words == 0
		    && msgs[i].sysmsg_method != 'a')
			continue; /* skip invalid */
		if (!msgs[i].priority && msgs[i].msg_type != FLEX_FRAME_MSG_TYPE_TONE)
			order[n_prio + n_norm++] = (int)i;
	}
	/* Pass 3: tone-only messages last (no vector needed) */
	for (i = 0; i < (uint32_t)msg_count; i++) {
		if (info[i].addr_words == 0 && info[i].vector_words == 0
		    && msgs[i].sysmsg_method != 'a')
			continue; /* skip invalid */
		if (msgs[i].msg_type == FLEX_FRAME_MSG_TYPE_TONE)
			order[n_prio + n_norm + n_tone++] = (int)i;
	}

	/* ---- Greedy packing ---- */

	capacity = FLEX_WORDS_PER_FRAME - biw_count; /* words available after BIWs */

	/* When using multiple transmission (subframe repeat),
	 * the frame is divided into N subframes with reduced word counts:
	 *   2x=44, 3x=29, 4x=22 words per subframe.
	 * Capacity is limited to the subframe size minus BIW words. */
	if (params->num_transmissions > 1) {
		int sf_words = flex_subframe_words(params->num_transmissions);
		capacity = sf_words - biw_count;
		if (capacity < 0)
			capacity = 0;
	}

	/* Pack non-tone messages (priority + normal) — these need addr + vector + msg words.
	 * Per Sections 3.9.1/3.9.3/3.9.4: for long addresses with body words,
	 * body[0] is placed at Vy (already counted in vector_words), so the
	 * Message Field only needs msg_words-1 slots. */
	for (i = 0; i < (uint32_t)(n_prio + n_norm); i++) {
		int idx = order[i];
		int mf_words = info[idx].msg_words;
		int needed;

		/* Long address: body[0] absorbed into Vy slot in VF */
		if (info[idx].is_long && mf_words > 0)
			mf_words--;

		needed = info[idx].addr_words + info[idx].vector_words
			   + mf_words;

		if (needed > capacity)
			break; /* no more room */

		info[idx].packed = 1;
		total_addr += info[idx].addr_words;
		total_vector += info[idx].vector_words;
		total_msg += mf_words;
		capacity -= needed;
		packed_count++;

		if ((int)i < n_prio)
			prio_addr_words += info[idx].addr_words;
	}

	/* Reserve an extra vector word at end of VF for BIW101 system
	 * message (method (b), §3.9.2, Fig. 3.7.2-2).
	 *
	 * When BIW101 A=0000~0011 is present, the spec requires a
	 * system message vector at the END of the vector field pointing
	 * to the same message body as the operator messaging vector.
	 * This is a duplicate — both vectors reference the same MF words.
	 *
	 * Only added when a system message (operator msg LSB 0-3) was
	 * actually packed into this frame. */
	int sysmsg_extra_vec = 0;
	int sysmsg_packed_idx = -1;
	if (params->sysmsg_a_type >= 0 && params->sysmsg_a_type <= 3) {
		/* Find the packed system message */
		for (i = 0; i < (uint32_t)(n_prio + n_norm); i++) {
			int idx = order[i];
			uint32_t aw;
			if (!info[idx].packed)
				continue;
			aw = (uint32_t)(msgs[idx].capcode + FLEX_SHORT_ADDR_OFFSET);
			if (aw >= FLEX_ADDR_OPER_MSG_MIN &&
			    aw <= FLEX_ADDR_OPER_MSG_MAX &&
			    (aw & FLEX_OPER_MSG_LSB_MASK) <= FLEX_BIW_SYSINFO_A_MSG_SSID) {
				if (capacity >= 1) {
					sysmsg_extra_vec = 1;
					sysmsg_packed_idx = idx;
					total_vector++;
					capacity--;
				}
				break;
			}
		}
	}

	/* Pack tone-only messages — these need only addr words (no vector, no msg body).
	 * Tone-only addresses sit at the end of the
	 * address field, after the vector field start offset.  They don't
	 * consume vector or message word slots.
	 *
	 * Note:
	 * "Tone-Only Addresses (without vectors) cannot be transmitted in
	 * Frames used for transmitting System Messages."
	 *
	 * This restriction applies when the frame contains actual system
	 * message content in the Message Field — i.e., when BIW101 has
	 * A=0000~0011 (SysMsg for all/home/roaming/SSID pagers) with
	 * vectors at the end of VF and body words in MF, or when an
	 * Operator Messaging Address carries system message content.
	 *
	 * A=0100 (timezone) only carries data in the I-field of the BIW
	 * word itself — no vectors or MF content — so the restriction
	 * does not apply to timezone-only frames.
	 *
	 * Tone-only exclusion is enforced in the scheduler
	 * (flex.c candidate collection, has_sysmsg_content flag).
	 *
	 * TODO §3.9.2 method (a): BIW101 A=0000~0011 as implicit
	 * address — emit BIW101 in BIW field, place system message
	 * vector at END of vector field (not normal position), body
	 * in MF, R=0.  For fragmented system messages, BIW101 must
	 * be re-stated in each fragment's frame.  Requires frame
	 * layout changes in flex_encode_frame_multi().
	 *
	 * DONE §3.9.2 method (b): BIW101 + Operator Messaging
	 * Address together — when both BIW101 and an Operator
	 * Messaging Address target the same audience, emit both.
	 * BIW in BIW field, operator address in AF, operator
	 * vector in VF, system message vector at end of VF,
	 * message in MF.
	 * Implemented via params->sysmsg_a_type: scheduler pre-scans
	 * for Operator Messaging capcodes with LSB 0-3, sets the
	 * A-type, and the BIW writing section emits BIW101 with
	 * the matching A-type.  The system message vector (same
	 * format for method (a) and (b)) is placed at the end of
	 * VF after all normal vectors (Fig. 3.7.2-2 (b)).
	 *
	 * TODO §3.9.2 NID: Network Address must appear twice
	 * consecutively in AF within frames 0-7.  First defines
	 * NID, second initiates associated system message.  Short
	 * Message Vectors not allowed for NID system messages.
	 *
	 * TODO §3.8.2.4 SysEvent scheduling: System Event
	 * Notification (LSB=0xF) must be transmitted in each
	 * Frame for ≥1 full collapse cycle.  Pre-alerts pagers
	 * about changes within 4 cycles.  Uses Short Instruction
	 * Vector (i=001).  Requires scheduler to auto-repeat.
	 *
	 * TODO §3.8.2.4 SSID Change scheduling: SSID Change
	 * Instruction (LSB=0xE) must be in roaming frames
	 * (F0-7 NID, F0-3 SSID-only) for 2 cycles before +
	 * 2 cycles after the change (5 cycles total).  Must be
	 * in same phase as SSID.  Desirable: once/hour after. */
	{
		int tone_addr_words = 0;
		for (i = 0; i < (uint32_t)n_tone; i++) {
			int idx = order[n_prio + n_norm + (int)i];
			int needed = info[idx].addr_words; /* no vector, no msg */

			if (needed > capacity)
				break;

			info[idx].packed = 1;
			tone_addr_words += info[idx].addr_words;
			capacity -= needed;
			packed_count++;
		}
		/* Tone-only addresses are part of the address field but
		 * come after the vector field start point.  Include them
		 * in total_addr for the address writing loop. */
		total_addr += tone_addr_words;
	}

	/* ---- Write BIW1 ---- */

	fwc = 0;
	{
		uint32_t s_vfield = (uint32_t)(biw_count + total_addr);

		/* Validate BIW1 field limits.
		 *
		 * p field (§8.1, §3.7.1): 4 bits, max 15 priority address
		 * words.  A long priority address consumes 2 words, so at
		 * most 15 short or 7 long priority addresses per phase.
		 *
		 * s_vfield (§3.7.1): 6 bits, max 63.  This is the word
		 * index where the Vector Field starts (= biw_count + all
		 * AF words including priority, normal, and tone-only).
		 * Overflow means the address field is too large for the
		 * frame — the pager would mislocate the vector field. */
		if (prio_addr_words > (int)FLEX_BIW1_PRIO_MASK) {
			LOGP(DFLEX, LOGL_ERROR,
			     "TX: BIW1 priority address words (%d) exceeds"
			     " 4-bit max (%d) — clamping.  Some priority"
			     " addresses will appear as normal to the pager.\n",
			     prio_addr_words, (int)FLEX_BIW1_PRIO_MASK);
			prio_addr_words = (int)FLEX_BIW1_PRIO_MASK;
		}
		if (s_vfield > FLEX_BIW1_VSTART_MASK) {
			LOGP(DFLEX, LOGL_ERROR,
			     "TX: BIW1 vector field start (%u) exceeds"
			     " 6-bit max (%u) — frame has too many addresses."
			     "  Pager will mislocate the vector field.\n",
			     s_vfield, (uint32_t)FLEX_BIW1_VSTART_MASK);
			if (!*error)
				*error = -FLEX_ERR_INVALID_MESSAGE;
		}

		frame_words[fwc++] = flex_create_biw1(
			(uint32_t)prio_addr_words,
			(uint32_t)e_biw,
			s_vfield,
			(uint32_t)params->carry_on,
			(uint32_t)params->collapse);
	}

	/* ---- Write BIW2/3/4 if enabled ---- */
	/* Emit extra BIW words in priority order.  Receivers identify
	 * each word by its type field, so order doesn't matter. */
	{
		int slots_left = biw_count - 1; /* extra slots available */
		time_t biw_now = 0;
		struct tm biw_tm;
		int biw_tm_valid = 0;

		/* SSID1 (type 000) — emit when --ssid or --nid configured. */
		if (slots_left > 0 && (params->local_id || params->coverage_id)) {
			frame_words[fwc++] = flex_create_biw2(
				params->local_id,
				params->coverage_id);
			slots_left--;
			LOGP(DFLEX, LOGL_INFO,
			     "TX: BIW SSID1 LID=%u CZ=%u\n",
			     params->local_id, params->coverage_id);
		}

		/* SSID2 (type 111) — per §6.1.1.3, SSID2 must be
		 * transmitted in frames 0 through 3 only. */
		if (slots_left > 0 && (params->country_code || params->tmf) &&
		    params->frame <= 3) {
			const char *cname = flex_mcc_name(params->country_code);
			frame_words[fwc++] = flex_create_biw_ssid2(
				params->country_code,
				params->tmf);
			slots_left--;
			LOGP(DFLEX, LOGL_INFO,
			     "TX: BIW SSID2 country=%u (%s) tmf=0x%X\n",
			     params->country_code,
			     cname ? cname : "unknown",
			     params->tmf);
		}

		/* Date + Time (types 001, 010)
		 *
		 * TODO §6.1.1.3 Note 1: Time-related BIW rotation.
		 * In multi-phase modes, T1(Date/001), T2(Time/010),
		 * and T3(SysInfo-Time/101) must rotate across phases
		 * in Cycle 0, Frame 0.  At 6400bps (4 phases):
		 *   Cycle N frame 0: a→T(1+N%3), b→T(2+N%3), etc.
		 * At 3200bps (2 phases): similar with 2-phase mapping.
		 * At 1600bps: all time BIWs in phase A, >2 allowed
		 * if BIW slots are available.
		 * Currently all time BIWs go in every phase — correct
		 * for 1600bps but over-emits for multi-phase modes. */
		/* Capture current time once for all time-related BIWs.
		 * Used by Date (BIW3), Time (BIW4), and SysInfo timezone. */
		if (params->biw_time || (params->timezone_code >= 0 &&
					 params->timezone_code < (int)FLEX_TZ_ENTRIES)) {
			biw_now = time(NULL);
			if (params->timezone_code >= 0)
				localtime_r(&biw_now, &biw_tm);
			else
				gmtime_r(&biw_now, &biw_tm);
			biw_tm_valid = 1;
		}

		if (params->biw_time && biw_tm_valid) {
			if (slots_left > 0) {
				int real_year = biw_tm.tm_year + 1900;
				int equiv = flex_biw_equiv_year(real_year);
				uint32_t year_field = (uint32_t)(equiv - FLEX_BIW_DATE_YEAR_BASE);
				frame_words[fwc++] = flex_create_biw3(
					year_field,
					(uint32_t)(biw_tm.tm_mon + 1),
					(uint32_t)biw_tm.tm_mday);
				slots_left--;
				LOGP(DFLEX, LOGL_INFO,
				     "TX: BIW DATE %04d-%02d-%02d (year field=%u%s)\n",
				     real_year,
				     biw_tm.tm_mon + 1,
				     biw_tm.tm_mday,
				     year_field,
				     (equiv != real_year) ? " equiv" : "");
			}
			if (slots_left > 0) {
				uint32_t sec_step = (uint32_t)(biw_tm.tm_sec / 7.5);
				if (sec_step > 7) sec_step = 7;
				frame_words[fwc++] = flex_create_biw4(
					(uint32_t)biw_tm.tm_hour,
					(uint32_t)biw_tm.tm_min,
					sec_step);
				slots_left--;
				LOGP(DFLEX, LOGL_INFO,
				     "TX: BIW TIME %02d:%02d:%04.1f\n",
				     biw_tm.tm_hour,
				     biw_tm.tm_min,
				     sec_step * FLEX_BIW_TIME_SECOND_STEP);
			}
		}

		/* SysInfo BIW101 — system message or timezone.
		 *
		 * Only one type 101 word per phase per spec:
		 * "The transmission of a System Message by Block
		 * Information Word 101 must be one time per each phase."
		 *
		 * Priority: system message A-type (0000-0011) takes
		 * precedence over timezone (A=0100) when both are
		 * configured, since the system message requires the
		 * BIW101 to be present per method (b) (Fig. 3.7.2-2).
		 * Timezone can be transmitted in other frames. */
		if (slots_left > 0 &&
		    params->sysmsg_a_type >= 0 &&
		    params->sysmsg_a_type <= 3) {
			/* Method (b): BIW101 with A=0000~0011 alongside
			 * Operator Messaging Address (§3.9.2, Fig. 3.7.2-2).
			 * I-field is reserved (0) for A=0000~0011. */
			frame_words[fwc++] = flex_create_biw_sysinfo(
				(uint32_t)params->sysmsg_a_type, 0);
			slots_left--;
			LOGP(DFLEX, LOGL_INFO,
			     "TX: BIW101 SysMsg A=%d (%s)\n",
			     params->sysmsg_a_type,
			     flex_biw_sysinfo_a_name(
				     (uint32_t)params->sysmsg_a_type));
		} else if (slots_left > 0 &&
			   params->timezone_code >= 0 &&
			   params->timezone_code < (int)FLEX_TZ_ENTRIES) {
			int tz_min = flex_tz_to_minutes((uint32_t)params->timezone_code);
			char tzbuf[20];
			uint32_t tz_info = (uint32_t)params->timezone_code & FLEX_BIW_SYSINFO_TZ_MASK;

			/* DST flag (L0): auto-detect from system time.
			 * Spec: L0=0 means DST active, L0=1 means standard time. */
			if (biw_tm_valid) {
				if (biw_tm.tm_isdst <= 0)
					tz_info |= (1U << FLEX_BIW_SYSINFO_DST_SHIFT);
				/* Extended seconds (S5-S3): fine sub-step within
				 * the 7.5s coarse step from BIW4.
				 * 6-bit combined = floor(sec / 0.9375).
				 * S2-S0 (BIW4) = combined >> 3 (coarse 7.5s).
				 * S5-S3 (here) = combined & 7 (fine 0.9375s). */
				{
					uint32_t combined = (uint32_t)(biw_tm.tm_sec / 0.9375);
					if (combined > 63) combined = 63;
					tz_info |= ((combined & 7) << FLEX_BIW_SYSINFO_EXTSEC_SHIFT);
				}
			}

			frame_words[fwc++] = flex_create_biw_sysinfo(
				FLEX_BIW_SYSINFO_A_TIME, tz_info);
			slots_left--;
			LOGP(DFLEX, LOGL_INFO,
			     "TX: BIW SYSINFO timezone zone=%d (%s)\n",
			     params->timezone_code,
			     flex_tz_format(tz_min, tzbuf, sizeof(tzbuf)));
		}

		/* Channel Setup (A-type 0x06).
		 * Per §6.1.1.3 Note 3: "When Frame Offset is supported,
		 * BIW101 must be transmitted in Frame 3 at minimum."
		 * Emit in frames 0-3 following the dotted-box pattern. */
		if (slots_left > 0 && params->chan_setup_enabled &&
		    params->frame <= 3) {
			uint32_t info = 0;
			/* Frame offset: derived from collapse and current frame number */
			uint32_t frame_ofs = params->collapse > 0
				? (params->frame % (1U << params->collapse)) : 0;
			info |= (frame_ofs & FLEX_BIW_SYSINFO_FRAME_OFS_MASK);
			/* Carry-on: reuse existing carry_on field from params */
			info |= ((params->carry_on & FLEX_BIW_SYSINFO_CARRY_ON_MASK)
				 << FLEX_BIW_SYSINFO_CARRY_ON_SHIFT);
			/* N0: set if current frame contains a NID system message */
			int has_nid_sysmsg = 0;
			if (has_nid_sysmsg)
				info |= (1U << FLEX_BIW_SYSINFO_NID_BIT);
			/* B0: set if current frame contains a system message.
			 * Now derived from sysmsg_a_type (method (b)). */
			int has_sysmsg = (params->sysmsg_a_type >= 0 &&
					  params->sysmsg_a_type <= 3) ? 1 : 0;
			if (has_sysmsg)
				info |= (1U << FLEX_BIW_SYSINFO_SYSMSG_BIT);
			frame_words[fwc++] = flex_create_biw_sysinfo(
				FLEX_BIW_SYSINFO_A_CHAN_SETUP, info);
			slots_left--;
			LOGP(DFLEX, LOGL_INFO,
			     "TX: BIW_CHAN_SETUP frame_offset=%u carry_on=%d N0=%d B0=%d\n",
			     frame_ofs, params->carry_on, has_nid_sysmsg, has_sysmsg);
		}
	}

	/* ---- Write address words ----
	 * Order: priority (non-tone), normal (non-tone), tone-only.
	 * Tone-only addresses are at the end of the address field. */

	for (i = 0; i < (uint32_t)(n_prio + n_norm + n_tone); i++) {
		int idx = order[i];
		if (!info[idx].packed)
			continue;

		/* Method (a): no address word in AF */
		if (msgs[idx].sysmsg_method == 'a')
			continue;

		if (msgs[idx].temp_delivery_slot >= 0) {
			/* Temp address DELIVERY (§3.8.2.3) */
			uint32_t ta = FLEX_ADDR_TEMPORARY_MIN
				    + (uint32_t)msgs[idx].temp_delivery_slot;
			frame_words[fwc++] = flex_encode_temp_address(ta);
			LOGP(DFLEX, LOGL_DEBUG,
			     "TX: AF[%u] Temporary addr slot=%d aw=0x%05X\n",
			     fwc - 1, msgs[idx].temp_delivery_slot, ta);
		} else if (info[idx].is_long) {
			uint32_t aw[2] = {0, 0};
			encode_long_address(msgs[idx].capcode, aw);
			frame_words[fwc++] = aw[0];
			frame_words[fwc++] = aw[1];
			LOGP(DFLEX, LOGL_DEBUG,
			     "TX: AF[%u,%u] long addr %s cap=%" PRIu64 "\n",
			     fwc - 2, fwc - 1,
			     flex_capcode_type_name(msgs[idx].capcode),
			     msgs[idx].capcode);
		} else {
			uint32_t sdw = ((uint32_t)msgs[idx].capcode + FLEX_SHORT_ADDR_OFFSET)
				& FLEX_DATA_MASK;
			frame_words[fwc++] = encode_short_address(
				(uint32_t)msgs[idx].capcode);
			LOGP(DFLEX, LOGL_DEBUG,
			     "TX: AF[%u] short addr cap=%" PRIu64 " aw=0x%05X\n",
			     fwc - 1, msgs[idx].capcode, sdw);
		}
	}

	/* ---- Write Vector Field and Message Field ----
	 *
	 * The frame has
	 * contiguous regions after the Block Information:
	 *   [BI][AF][VF][MF][IB]
	 *
	 * Frame layout:
	 *   (A) Short address: 1 addr word (Ax) → 1 vector word (Vx)
	 *   (B) Long address:  2 addr words (Ax,Ay) → 2 vector words (Vx,Vy)
	 *   (C) Tone-only: address only, no vector, no message
	 *
	 * Per Sections 3.9.1, 3.9.3, 3.9.4 — long address rule:
	 *   "When a Long Address is used, a 2nd word is required for the
	 *    vector, and the 1st word of the message is placed at the 2nd
	 *    word of the vector."
	 *
	 * So for long addresses with body words:
	 *   VF: [Vx][body[0]]   — Vy slot holds the first body word
	 *   MF: [body[1]..body[n-1]]  — remaining n-1 body words
	 *   Vector b field → MF start (body[1]) for multi-word messages
	 *   Vector n field → total body word count (including body[0])
	 *
	 * For short addresses:
	 *   VF: [Vx]
	 *   MF: [body[0]..body[n-1]]  — all n body words
	 *   Vector b field → MF start (body[0])
	 *   Vector n field → total body word count
	 *
	 * Approach:
	 *   1. Encode each message into a temp body buffer
	 *   2. Build vector word with correct b field
	 *   3. Write Vx to VF; if long, write body[0] to Vy slot in VF
	 *   4. Write remaining body words to MF
	 */
	{
		/* Temp buffer for encoded body words from each message */
		uint32_t body_buf[FLEX_WORDS_PER_FRAME];
		uint32_t body_count;  /* number of body words produced */

		uint32_t vec_fwc;     /* write cursor in Vector Field */
		uint32_t msg_fwc;     /* write cursor in Message Field */
		uint32_t mf_start;    /* absolute word index where MF begins */

		/* System message vector for end of VF (method (a)/(b)).
		 * Generated from the message's start/count after encoding,
		 * same vector format for both methods. */
		uint32_t sysmsg_vw = 0;

		vec_fwc = (uint32_t)(biw_count + total_addr);
		mf_start = (uint32_t)(biw_count + total_addr + total_vector);
		msg_fwc = mf_start;

		for (i = 0; i < (uint32_t)(n_prio + n_norm); i++) {
			int idx = order[i];
			int enc_err = 0;
			uint32_t vw = 0;
			uint32_t body_fwc = 0; /* cursor into body_buf */
			uint32_t msg_start_for_vec;
			uint32_t j;

			if (!info[idx].packed)
				continue;

			/* Compute the vector b field (message start word)
			 * BEFORE calling the encoder, so the vector word is
			 * created with the correct value.
			 *
			 * For short addresses: b = msg_fwc (MF cursor).
			 * For long addresses with body: b = msg_fwc (MF start
			 * for body[1..n-1]).
			 *
			 * Exception — numeric, single-word
			 * message with long address: b = Vy position.  We
			 * handle this as a post-fixup since we don't know the
			 * body count until after encoding.
			 *
			 * For most cases, msg_fwc is correct. */
			/* Compute the vector b field (message start word)
			 * BEFORE calling the encoder, so the vector word is
			 * created with the correct value.
			 *
			 * For hex/alpha:
			 *   b = MF start for this message (always).
			 * For numeric:
			 *   b = Vy position if single-word message and long,
			 *   b = MF start otherwise.
			 *
			 * For short addresses or no body: b = msg_fwc.
			 * For long addresses: b = msg_fwc (body[1] in MF).
			 * Special: numeric, 1 body word, long → b = Vy pos. */
			if (info[idx].is_long && info[idx].msg_words == 1 &&
			    msgs[idx].msg_type == FLEX_FRAME_MSG_TYPE_NUMERIC) {
				/* Numeric single-word long: "the word number at the top
				 * of the message, indicated by the vector is
				 * the word number of the 2nd word of the
				 * vector if the message consists of one word" */
				msg_start_for_vec = vec_fwc + 1; /* Vy position */
			} else {
				msg_start_for_vec = msg_fwc;
			}

			/* Encode message body into temp buffer.
			 * Each encoder writes body words to body_buf starting
			 * at body_fwc=0 and returns the vector word. */
			switch (msgs[idx].msg_type) {
			case FLEX_FRAME_MSG_TYPE_ALPHA:
				if (msgs[idx].charset == 1) {
					vw = encode_kanji_message(body_buf,
						msgs[idx].message,
						msg_start_for_vec, &body_fwc,
						&enc_err);
				} else {
					struct flex_msg_config acfg;
					memset(&acfg, 0, sizeof(acfg));
					acfg.fragment_index = msgs[idx].fragment_index;
					acfg.total_fragments = msgs[idx].total_fragments;
					acfg.mail_drop = msgs[idx].mail_drop;
					acfg.alpha_r_flag = msgs[idx].alpha_r_flag;
					acfg.precomputed_sig = msgs[idx].precomputed_sig;
					vw = encode_alpha_message(body_buf,
						msgs[idx].message,
						msg_start_for_vec, &body_fwc,
						msgs[idx].sequence_num,
						&acfg);
				}
				break;

			case FLEX_FRAME_MSG_TYPE_NUMERIC:
				vw = encode_numeric_message(body_buf,
					msgs[idx].message,
					msg_start_for_vec, &body_fwc,
					NULL);
				break;

			case FLEX_FRAME_MSG_TYPE_HEX: {
				struct flex_msg_config hcfg;
				memset(&hcfg, 0, sizeof(hcfg));
				hcfg.fragment_index = msgs[idx].fragment_index;
				hcfg.total_fragments = msgs[idx].total_fragments;
				hcfg.blocking_length = msgs[idx].blocking_length;
				hcfg.mail_drop = msgs[idx].mail_drop;
				hcfg.hex_r_flag = msgs[idx].hex_r_flag;
				hcfg.precomputed_sig = msgs[idx].precomputed_sig;
				vw = encode_hex_message(body_buf,
					msgs[idx].message,
					msg_start_for_vec, &body_fwc,
					msgs[idx].sequence_num,
					&hcfg,
					&enc_err);
				break;
			}

			case FLEX_FRAME_MSG_TYPE_INSTRUCTION:
				vw = encode_instruction_message(
					msgs[idx].message,
					&enc_err);
				break;

			case FLEX_FRAME_MSG_TYPE_SHORT:
				/* Short Message Vector (§3.9.2, Table 3.9.2-1).
				 * All data in the vector word, no body words. */
				{
					uint32_t dw = 0;
					dw |= (FLEX_VECTOR_TYPE_TONE & FLEX_VEC_TYPE_MASK)
					      << FLEX_VEC_TYPE_SHIFT;

					switch (msgs[idx].short_msg_type) {
					case FLEX_SMSG_TYPE_NUMERIC: {
						/* t=00: 3-digit BCD (short) or 8-digit (long).
						 * 1st word: d0-d3=digit a, d4-d7=digit b, d8-d11=digit c.
						 * 2nd word (long only): d0-d3=digit d, d4-d7=digit e,
						 *   d8-d11=digit f, d12-d15=digit g, d16-d19=digit h,
						 *   d20=spare (0).
						 * Unused digits = space (0xC). */
						const char *smsg = msgs[idx].message;
						int slen = msgs[idx].message_length;
						int max_digits = info[idx].is_long
							? FLEX_SMSG_NUM_LONG_DIGITS
							: FLEX_SMSG_NUM_SHORT_DIGITS;
						uint8_t bcd_digits[8];
						int di;
						/* t1t0 = 00 — already zero */
						for (di = 0; di < max_digits; di++) {
							bcd_digits[di] = (di < slen && smsg[di])
								? flex_num_char_to_bcd((uint8_t)smsg[di])
								: FLEX_NUM_BCD_SPACE;
						}
						/* 1st word: digits a,b,c */
						for (di = 0; di < 3 && di < max_digits; di++) {
							dw |= ((uint32_t)bcd_digits[di] & 0xF)
							      << (FLEX_SMSG_D_SHIFT + di * FLEX_SMSG_NUM_DIGIT_BITS);
						}
						/* 2nd word (long addr): digits d,e,f,g,h.
						 * Stored in body_buf[0] so the Vy slot
						 * logic writes it to the vector field. */
						if (info[idx].is_long) {
							uint32_t dw2 = 0;
							for (di = 3; di < 8; di++) {
								dw2 |= ((uint32_t)bcd_digits[di] & 0xF)
								       << ((di - 3) * FLEX_SMSG_NUM_DIGIT_BITS);
							}
							/* d32 = spare, set to 0 (bit 20) — already zero */
							body_buf[body_fwc++] = flex_encode_word(
								reverse_bits32(dw2));
						}
						break;
					}
					case FLEX_SMSG_TYPE_SOURCE:
						/* t=01: source codes S2S1S0 (0-7).
						 * Message field = source code as decimal. */
						dw |= (uint32_t)FLEX_SMSG_TYPE_SOURCE
						      << FLEX_SMSG_T_SHIFT;
						{
							int src = 0;
							if (msgs[idx].message && msgs[idx].message_length > 0)
								src = atoi(msgs[idx].message);
							if (src < 0 || src > 7) src = 0;
							dw |= ((uint32_t)src & FLEX_SMSG_SRC_MASK)
							      << FLEX_SMSG_D_SHIFT;
						}
						break;
					case FLEX_SMSG_TYPE_NUMBERED:
						/* t=10: S2S1S0 + N5-N0 + R0.
						 * Message field = message number N (0-63).
						 * ssource and sr from options. */
						dw |= (uint32_t)FLEX_SMSG_TYPE_NUMBERED
						      << FLEX_SMSG_T_SHIFT;
						{
							uint32_t d = 0;
							int n = 0;
							if (msgs[idx].message && msgs[idx].message_length > 0)
								n = atoi(msgs[idx].message);
							if (n < 0 || n > 63) n = 0;
							d |= (uint32_t)msgs[idx].short_msg_source
							     & FLEX_SMSG_NUMB_SRC_MASK;
							d |= ((uint32_t)n
							      & FLEX_SMSG_NUMB_N_MASK)
							     << FLEX_SMSG_NUMB_N_SHIFT;
							d |= ((uint32_t)msgs[idx].short_msg_r
							      & FLEX_SMSG_NUMB_R_MASK)
							     << FLEX_SMSG_NUMB_R_SHIFT;
							dw |= d << FLEX_SMSG_D_SHIFT;
						}
						break;
					default:
						enc_err = -FLEX_ERR_INVALID_MESSAGE;
						break;
					}

					if (!enc_err) {
						dw = flex_word_checksum(dw);
						vw = flex_encode_word(reverse_bits32(dw));
					}
				}
				break;

			case FLEX_FRAME_MSG_TYPE_SECURE: {
				struct flex_msg_config scfg;
				memset(&scfg, 0, sizeof(scfg));
				scfg.fragment_index = msgs[idx].fragment_index;
				scfg.total_fragments = msgs[idx].total_fragments;
				scfg.mail_drop = msgs[idx].mail_drop;
				scfg.blocking_length = msgs[idx].blocking_length;
				scfg.alpha_r_flag = msgs[idx].alpha_r_flag;
				scfg.hex_r_flag = msgs[idx].hex_r_flag;
				scfg.precomputed_sig = msgs[idx].precomputed_sig;
				vw = encode_secure_message(body_buf,
					msgs[idx].message,
					msg_start_for_vec, &body_fwc,
					msgs[idx].sequence_num,
					&scfg,
					msgs[idx].secure_subtype,
					msgs[idx].secure_encoding,
					&enc_err);
				break;
			}

			case FLEX_FRAME_MSG_TYPE_SPECIAL_NUM:
				vw = encode_special_numeric_message(body_buf,
					msgs[idx].message,
					msg_start_for_vec, &body_fwc,
					&enc_err);
				break;

			case FLEX_FRAME_MSG_TYPE_NUMBERED_NUM:
				vw = encode_numbered_numeric_message(body_buf,
					msgs[idx].message,
					msg_start_for_vec, &body_fwc,
					msgs[idx].numbered_msgnum,
					msgs[idx].numbered_r,
					msgs[idx].numbered_s,
					&enc_err);
				break;

			default:
				enc_err = -FLEX_ERR_INVALID_MESSAGE;
				break;
			}

			if (enc_err && !*error)
				*error = enc_err;

			body_count = body_fwc;

			/* Write Vx to Vector Field.
			 * Method (a): no normal vector — skip Vx write. */
			if (msgs[idx].sysmsg_method != 'a') {
				if (vec_fwc < FLEX_WORDS_PER_FRAME)
					frame_words[vec_fwc++] = vw;
			}

			/* Capture vector word for system message at end of VF
			 * (§3.9.2, Fig. 3.7.2-2).  Same vector for method
			 * (a) and (b) — generated from message encoding. */
			if (sysmsg_extra_vec && idx == sysmsg_packed_idx)
				sysmsg_vw = vw;

			/* For long addresses: write Vy to VF.
			 * Method (a): no address, no Vy.
			 * Most types: Vy = body[0] ("the 1st word of the
			 * message is placed at the 2nd word of the vector").
			 * Instruction/Short: Vy = all zeros (d11-d31 unused,
			 * per §3.9.6 note: "All unused bits are set to 0"). */
			if (info[idx].is_long && msgs[idx].sysmsg_method != 'a') {
				if (msgs[idx].msg_type == FLEX_FRAME_MSG_TYPE_INSTRUCTION) {
					/* No body — 2nd vector word is zeros */
					if (vec_fwc < FLEX_WORDS_PER_FRAME)
						frame_words[vec_fwc++] = flex_encode_word(0);
				} else if (msgs[idx].msg_type == FLEX_FRAME_MSG_TYPE_SHORT &&
					   body_count > 0 && vec_fwc < FLEX_WORDS_PER_FRAME) {
					/* Short msg long addr: 2nd word has digits d-h */
					frame_words[vec_fwc++] = body_buf[0];
				} else if (msgs[idx].msg_type == FLEX_FRAME_MSG_TYPE_SHORT) {
					/* Short msg long addr but no body (e.g. source/numbered) */
					if (vec_fwc < FLEX_WORDS_PER_FRAME)
						frame_words[vec_fwc++] = flex_encode_word(0);
				} else if (body_count > 0 && vec_fwc < FLEX_WORDS_PER_FRAME) {
					frame_words[vec_fwc++] = body_buf[0];
				} else if (vec_fwc < FLEX_WORDS_PER_FRAME) {
					frame_words[vec_fwc++] = flex_encode_word(0);
				}
			}

			/* Write remaining body words to Message Field.
			 * For long addresses: body[1..n-1] (body[0] is in Vy).
			 * For short addresses: body[0..n-1] (all in MF). */
			{
				uint32_t start = info[idx].is_long ? 1 : 0;
				for (j = start; j < body_count; j++) {
					if (msg_fwc < FLEX_WORDS_PER_FRAME)
						frame_words[msg_fwc++] = body_buf[j];
				}
			}
		}

		/* Write system message vector at end of VF
		 * (§3.9.2, Fig. 3.7.2-2).
		 *
		 * Same vector word for method (a) and (b) — points to
		 * the message body in MF with the correct start/count.
		 * Placed at the end of the vector field per spec:
		 * "corresponding vectors except Secure vector are
		 * transmitted at the end of the vector field." */
		if (sysmsg_extra_vec && sysmsg_vw) {
			if (vec_fwc < mf_start && vec_fwc < FLEX_WORDS_PER_FRAME) {
				frame_words[vec_fwc++] = sysmsg_vw;
				LOGP(DFLEX, LOGL_DEBUG,
				     "TX: VF[%u] SysMsg duplicate vector"
				     " (method (b), end of VF)\n",
				     vec_fwc - 1);
			}
		}

		/* Advance fwc past all written data */
		fwc = msg_fwc;
	}

	/* ---- Idle fill is already in place from pre-fill ----
	 * The frame was pre-filled with idle pattern at initialization.
	 * Data words were written over the default starting from word 0.
	 * Words beyond fwc retain their idle pattern unchanged, per §3.4.1:
	 * "Idle blocks are maintained as they are." */

	/* ---- Subframe placement for multiple transmission ----
	 *
	 * When num_transmissions > 1, the 88-word frame is divided into
	 * N subframe slots.  The CALLER (scheduler) is responsible for
	 * assembling all N slots into the full 88-word frame.
	 *
	 * The encoder produces content for ONE subframe at position 0
	 * (words 0..sf_words-1).  The caller then:
	 *   1. Extracts the sf_words from position 0
	 *   2. Places them at the correct slot offset in the 88-word frame
	 *   3. Fills other slots with cached retransmissions or idle BIW
	 *
	 * This function does NOT move content to a target slot offset.
	 * The subframe_index in params is used only for BIW1 encoding
	 * (the pager needs to know which copy it's receiving).
	 *
	 * For 3x, the extra word (position 87) is set to idle by the
	 * caller when assembling the full frame. */

	/* ---- Block-boundary fixup for decoder compatibility ----
	 *
	 * Per the standard (§3.4.1, §3.5.1), a compliant decoder uses
	 * BIW and vector metadata to locate data words — it never needs
	 * to scan for idle patterns.  The 4-bit checksum (§3.5.1) on
	 * BIW/vector/FIW words guarantees they can't be all-zeros or
	 * all-ones (checksum would be wrong), but message body words
	 * have no such protection and CAN legitimately be all-zeros
	 * (e.g. hex termination fill) or all-ones.
	 *
	 * However, PDW and multimon-ng use a non-standard optimization:
	 * they check the last word of each 8-word block (indices 7, 15,
	 * 23, ..., 87) for all-zeros or all-ones, and if found, assume
	 * the rest of the frame is idle and stop processing.
	 *
	 * To maintain interoperability with these decoders, we apply a
	 * 1-bit fixup: flip bit 0 of any all-zeros/all-ones codeword
	 * at a block boundary when real data follows.  BCH(31,21) can
	 * correct up to 2-bit errors, so the receiver's BCH decoder
	 * recovers the original 21-bit data word transparently.
	 *
	 * Only applied when there's real (non-idle) data after the block.
	 * Enabled by --hack-for-non-standard-decoders (default OFF). */
	if (params->hack_nonstandard_decoders) {
		uint32_t last_data_word = 0;

		/* Find the last non-idle word index */
		for (i = FLEX_WORDS_PER_FRAME; i > 0; i--) {
			uint32_t w = frame_words[i - 1];
			if (w != FLEX_IDLE_WORD_1 && w != FLEX_IDLE_WORD_2) {
				last_data_word = i - 1;
				break;
			}
		}

		/* Check each block boundary */
		for (i = FLEX_WORDS_PER_BLOCK - 1;
		     i < FLEX_WORDS_PER_FRAME;
		     i += FLEX_WORDS_PER_BLOCK) {
			if (i >= last_data_word)
				break; /* no real data after this block */
			if (frame_words[i] == 0x00000000U ||
			    frame_words[i] == 0xFFFFFFFFU) {
				frame_words[i] ^= 1U;
				LOGP(DFLEX, LOGL_INFO,
				     "TX: Block-boundary fixup: word %u "
				     "flipped bit 0 (was %s)\n",
				     i, (frame_words[i] ^ 1U) == 0
					? "all-zeros" : "all-ones");
			}
		}
	}

	/* ---- Block interleaving ---- */

	for (i = 0; i < FLEX_BLOCKS_PER_FRAME; i++)
		flex_interleave_block(i, frame_words);

	/* ---- Write interleaved data to output buffer ---- */

	memcpy(out, frame_words, FLEX_WORDS_PER_FRAME * 4);
	out += FLEX_WORDS_PER_FRAME * 4;

	if (msgs_packed)
		*msgs_packed = packed_count;

	return (size_t)(out - buffer);
}


/* ===== Bit-Level Phase Interleaving ===== */

/*
 * Bit-interleave phase data into an output byte buffer.
 *
 * At 3200/6400 baud, the transmitted bit stream alternates between
 * phases at the BIT level, not the word level.  PDW's de-interleaver
 * (Flex.cpp lines 1159-1180) confirms this:
 *
 *   3200/2FSK (2 phases A,C):
 *     hbit=0 → bit goes to phase A, hbit=1 → bit goes to phase C
 *     Pattern: A_bit, C_bit, A_bit, C_bit, ...
 *
 *   3200/4FSK (2 phases A,C):
 *     Each 4-level symbol carries A_bit (MSB) and C_bit (LSB).
 *     No phase-toggle interleaving — every symbol goes to A+C.
 *     PDW (g_sps=1600, level=4): phase_A from MSB, phase_B from LSB.
 *     multimon-ng (Sync.baud=1600, levels=4): same, no toggle.
 *
 *   6400/4FSK (4 phases A,B,C,D):
 *     hbit=0 → dibit MSB→A, LSB→B; hbit=1 → dibit MSB→C, LSB→D
 *     Symbol stream alternates: (A,B), (C,D), (A,B), (C,D), ...
 *
 * For 2FSK encoding (which is what the DSP layer sends for 3200/2FSK),
 * we must bit-interleave: output bit 0 = A_bit0, bit 1 = C_bit0,
 * bit 2 = A_bit1, bit 3 = C_bit1, etc.
 *
 * For 3200/4FSK, each dibit = (A_bit, C_bit), no interleaving needed.
 *
 * For 6400/4FSK, dibits alternate between phase pairs (A,B) and (C,D).
 *
 * Parameters:
 *   phases     — array of phase data (each has 88 words)
 *   num_phases — 1 (1600/2FSK), 2 (3200/2FSK or 3200/4FSK), 4 (6400/4FSK)
 *   mod_type   — FLEX_MOD_2FSK or FLEX_MOD_4FSK
 *   out        — output buffer (must hold interleaved data)
 *
 * Returns bytes written.
 */

static size_t flex_interleave_phases(const flex_phase_data_t *phases,
				     int num_phases, int mod_type,
				     int bitrate, uint8_t *out)
{
	int w, bit;
	uint8_t *dst = out;
	uint8_t cur_byte = 0;
	int out_bit = 0;

	(void)bitrate; /* mode is determined by num_phases + mod_type */

	if (num_phases == 1) {
		/* Single phase (1600/2FSK): no interleaving, just serialize */
		for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
			uint32_t word = phases[0].words[w];
			*dst++ = (word >> 24) & 0xFF;
			*dst++ = (word >> 16) & 0xFF;
			*dst++ = (word >>  8) & 0xFF;
			*dst++ =  word        & 0xFF;
		}
		return (size_t)(dst - out);
	}

	if (num_phases == 2 && mod_type == FLEX_MOD_2FSK) {
		/*
		 * 3200/2FSK bit-level interleaving.
		 *
		 * PDW (Flex.cpp lines 1159-1180) at g_sps==3200, level==2:
		 *   hbit=0 → bit goes to phase A
		 *   hbit=1 → bit goes to phase C, bct++
		 *
		 * So the transmitted bit stream must alternate:
		 *   A_bit0, C_bit0, A_bit1, C_bit1, ...
		 *
		 * Total: 88 words × 32 bits × 2 = 5632 bits = 704 bytes.
		 */
		for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
			uint32_t wa = phases[0].words[w];
			uint32_t wc = phases[1].words[w];

			for (bit = 31; bit >= 0; bit--) {
				/* Phase A bit */
				cur_byte = (cur_byte << 1) | ((wa >> bit) & 1);
				out_bit++;
				if (out_bit == 8) {
					*dst++ = cur_byte;
					cur_byte = 0;
					out_bit = 0;
				}

				/* Phase C bit */
				cur_byte = (cur_byte << 1) | ((wc >> bit) & 1);
				out_bit++;
				if (out_bit == 8) {
					*dst++ = cur_byte;
					cur_byte = 0;
					out_bit = 0;
				}
			}
		}

		/* Flush (shouldn't happen: 88×32×2 = 5632 = 704×8) */
		if (out_bit > 0) {
			cur_byte <<= (8 - out_bit);
			*dst++ = cur_byte;
		}
		return (size_t)(dst - out);
	}

	if (num_phases == 2 && mod_type == FLEX_MOD_4FSK) {
		/*
		 * 3200/4FSK: 2 phases (A, C) packed into 4-level symbols.
		 *
		 * Per spec: "Of the first 2 bits for
		 * 3200bps/4-level FSK, bit 0a is converted into the MSB
		 * for the symbol and bit 0c into the LSB."
		 *
		 * Each symbol carries one bit from phase A (MSB) and one
		 * bit from phase C (LSB).  No phase-toggle interleaving —
		 * the symbol rate is 1600 sym/s (same as 1600/2FSK).
		 *
		 * multimon-ng (Sync.baud=1600, levels=4):
		 *   bit_a = (sym > 1)          → phase A (MSB)
		 *   bit_b = (sym==1)||(sym==2) → phase C (LSB)
		 *   phase_toggle forced to 0 (baud==1600)
		 *
		 * PDW (g_sps=1600, level=4):
		 *   phase_A from (gin < 2)         → MSB
		 *   phase_B from (gin==0)||(gin==3) → LSB
		 *
		 * Total: 88 words × 32 bits × 2 phases = 5632 bits
		 *       = 2816 dibits = 704 bytes.
		 */
		for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
			uint32_t wa = phases[0].words[w];  /* phase A */
			uint32_t wc = phases[1].words[w];  /* phase C */

			for (bit = 31; bit >= 0; bit--) {
				uint8_t msb = (wa >> bit) & 1;  /* A → MSB */
				uint8_t lsb = (wc >> bit) & 1;  /* C → LSB */
				cur_byte = (cur_byte << 2) | (msb << 1) | lsb;
				out_bit += 2;
				if (out_bit == 8) {
					*dst++ = cur_byte;
					cur_byte = 0;
					out_bit = 0;
				}
			}
		}

		if (out_bit > 0) {
			cur_byte <<= (8 - out_bit);
			*dst++ = cur_byte;
		}
		return (size_t)(dst - out);
	}

	/*
	 * 6400/4FSK: 4 phases (A, B, C, D) with dibit interleaving.
	 *
	 * Per spec: "Of the first 2 bits for the
	 * 6400bps/4-level FSK, bit 0a is converted into the MSB for
	 * the symbol and bit 0b into the LSB."
	 *
	 * Transmission order: 0a, 0b, 0c, 0d, 1a, 1b, 1c, 1d, ...
	 * At 4-level: symbols alternate (A,B) and (C,D) pairs.
	 *   Even symbols: MSB=A, LSB=B
	 *   Odd symbols:  MSB=C, LSB=D
	 *
	 * PDW (g_sps=3200, level=4):
	 *   hbit=0 → MSB→A, LSB→B
	 *   hbit=1 → MSB→C, LSB→D, bct++
	 *
	 * multimon-ng (Sync.baud=3200, levels=4):
	 *   phase_toggle alternates 0/1 per symbol
	 *   toggle=0 → bit_a→A, bit_b→B
	 *   toggle=1 → bit_a→C, bit_b→D
	 *
	 * Total: 88 words × 32 bits × 4 phases = 11264 bits
	 *       = 5632 dibits = 1408 bytes.
	 */
	{
		/* Phase pairing: (A,B) then (C,D) */
		for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
			uint32_t wa = phases[0].words[w];
			uint32_t wb = phases[1].words[w];
			uint32_t wc = phases[2].words[w];
			uint32_t wd = phases[3].words[w];

			for (bit = 31; bit >= 0; bit--) {
				uint8_t msb, lsb;

				/* Even symbol: (A, B) */
				msb = (wa >> bit) & 1;
				lsb = (wb >> bit) & 1;
				cur_byte = (cur_byte << 2) | (msb << 1) | lsb;
				out_bit += 2;
				if (out_bit == 8) {
					*dst++ = cur_byte;
					cur_byte = 0;
					out_bit = 0;
				}

				/* Odd symbol: (C, D) */
				msb = (wc >> bit) & 1;
				lsb = (wd >> bit) & 1;
				cur_byte = (cur_byte << 2) | (msb << 1) | lsb;
				out_bit += 2;
				if (out_bit == 8) {
					*dst++ = cur_byte;
					cur_byte = 0;
					out_bit = 0;
				}
			}
		}

		/* Flush (shouldn't happen: 88×32×4 = 11264 bits = 1408×8) */
		if (out_bit > 0) {
			cur_byte <<= (8 - out_bit);
			*dst++ = cur_byte;
		}
	}

	return (size_t)(dst - out);
}



/* ===== Multi-Phase Frame Encoding ===== */

/*
 * Encode a multi-phase FLEX frame for 3200/6400 bps operation.
 *
 * At higher baud rates, the frame carries multiple independent phases:
 *   3200 bps (2-FSK): 2 phases (A, C) — bit-level interleave
 *   3200 bps (4-FSK): 2 phases (A, C) — dibit packed
 *   6400 bps (4-FSK): 4 phases (A, B, C, D)
 *
 * Each phase is an independent set of 88 data words (BIW + addresses +
 * vectors + message data + idle fill), already block-interleaved by the
 * caller (typically via flex_encode_frame_multi per phase).
 *
 * The output format is:
 *   S1:  BS1(4) + Ax(4) + B(2) + Ax_inv(4) = 14 bytes
 *   FIW: 1 codeword = 4 bytes
 *   S2:  C block repeated per baud rate (5/10/20 bytes)
 *   Data: bit-interleaved phase data (see flex_interleave_phases)
 *
 * For 3200/2FSK, data is BIT-interleaved: A_bit, C_bit, A_bit, C_bit...
 * For 3200/4FSK, data is dibit-packed: each symbol = (A_bit, C_bit).
 * For 6400/4FSK, data is dibit-interleaved: (A,B), (C,D) alternating.
 *
 * The sync pattern (A1/A2/A3/A4) is selected based on bitrate and
 * modulation_type in params.
 *
 * Returns bytes written to buffer, or 0 on error (with *error set).
 */
size_t flex_encode_frame_phased(const flex_phase_data_t *phases, int num_phases,
				const flex_frame_params_t *params,
				uint8_t *buffer, size_t buffer_size,
				int *error)
{
	uint8_t *out;
	int err_local = 0;

	if (!error)
		error = &err_local;
	*error = 0;

	if (!phases || !params || !buffer) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	/* Validate phase count vs baud rate */
	if (num_phases < 1 || num_phases > FLEX_MAX_PHASES) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	/* Validate baud/modulation combination */
	if (params->bitrate == 1600 && params->modulation_type == FLEX_MOD_4FSK) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}
	if (params->bitrate == 6400 && params->modulation_type == FLEX_MOD_2FSK) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/* Compute output size:
	 *   S1: 14 bytes (BS1 + Ax + B + Ax_inv)
	 *   FIW: 4 bytes
	 *   S2: flex_s2_size() bytes (per speed/modulation)
	 *   Data: 88 * num_phases * 4 bytes
	 */
	{
		size_t s2_bytes = flex_s2_size(params->bitrate);
		size_t needed = 14 + 4 + s2_bytes
			      + (size_t)(FLEX_WORDS_PER_FRAME * num_phases * 4);
		if (buffer_size < needed) {
			*error = -FLEX_ERR_INVALID_BUFFER;
			return 0;
		}
	}

	out = buffer;

	/* ---- S1 + FIW + S2 ---- */
	out += flex_encode_s1(params->bitrate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));
	out += flex_encode_fiw(params, out,
			       buffer_size - (size_t)(out - buffer));
	out += flex_encode_s2(params->bitrate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));

	/* ---- Data: bit-interleaved phase data ---- */

	/*
	 * Phase interleaving depends on modulation type:
	 *   3200/2FSK: bit-level (A_bit, C_bit, A_bit, C_bit, ...)
	 *   3200/4FSK: dibit-packed, each symbol = (A_bit, C_bit)
	 *   6400/4FSK: dibit-level, phase pairs (A,B) and (C,D) alternate
	 * See flex_interleave_phases() for details and PDW evidence.
	 */
	out += flex_interleave_phases(phases, num_phases,
				      params->modulation_type,
				      params->bitrate, out);

	return (size_t)(out - buffer);
}


/* ===== Split Sync/Data Encoding ===== */

/*
 * Encode the sync portion of a FLEX frame: S1 + FIW.
 *
 * The sync portion is ALWAYS transmitted
 * at 1600 bps / 2-level FSK, regardless of the frame's data speed.
 * The receiver uses the A code to determine the target speed, then
 * switches after FIW to receive S2 + DATA at that rate.
 *
 * Layout:
 *   BS1:     4 bytes (32 bits) — alternating 1010... bit sync
 *   Ax:      4 bytes (32 bits) — sync code identifying speed/modulation
 *   B:       2 bytes (16 bits) — baud/level indicator
 *   Ax_inv:  4 bytes (32 bits) — inverted sync code
 *   FIW:     4 bytes (32 bits) — BCH-encoded frame information word
 *   Total:  18 bytes
 *
 * The sync code (A1/A2/A3/A4) is selected based on bitrate and
 * modulation_type in params, telling the receiver what speed to
 * expect after the speed switch that follows S1+FIW.
 *
 * Returns bytes written (18), or 0 on error.
 */
size_t flex_encode_sync(const flex_frame_params_t *params,
			uint8_t *buffer, size_t buffer_size)
{
	uint8_t *out;

	if (!params || !buffer || buffer_size < 18)
		return 0;

	out = buffer;

	/* S1: BS1(4) + Ax(4) + B(2) + Ax_inv(4) = 14 bytes */
	out += flex_encode_s1(params->bitrate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));

	/* FIW: 4 bytes */
	out += flex_encode_fiw(params, out,
			       buffer_size - (size_t)(out - buffer));

	return (size_t)(out - buffer);
}


/*
 * Encode the data portion of a FLEX frame: S2 + interleaved phase data.
 *
 * After S1+FIW (always 1600/2FSK), the transmitter switches to the
 * frame's target speed/modulation. S2 and DATA are both transmitted
 * at this target rate.
 *
 * S2 structure: BS2(N) + C(16) + BS2_inv(N) + C_inv(16)
 *   N varies per speed — see flex_encode_s2() for details.
 *
 * For multi-phase speeds (3200/6400), the message is encoded into
 * phase 0 and remaining phases are filled with idle.
 * Phase data is then interleaved into the output:
 *   3200/2FSK (2 phases): BIT-level — A_bit, C_bit, A_bit, C_bit, ...
 *   3200/4FSK (2 phases): dibit-packed — each symbol = (A_bit, C_bit)
 *   6400/4FSK (4 phases): dibit-level — (A,B), (C,D) alternating
 * See flex_interleave_phases() for details and PDW evidence.
 *
 * Output sizes:
 *   1600/2FSK (1 phase):  S2(5)  + 352  = 357 bytes
 *   3200/2FSK (2 phases): S2(10) + 704  = 714 bytes
 *   3200/4FSK (2 phases): S2(10) + 704  = 714 bytes
 *   6400/4FSK (4 phases): S2(20) + 1408 = 1428 bytes
 *
 * Returns bytes written, or 0 on error.
 */
size_t flex_encode_data(const flex_frame_msg_t *msgs, int msg_count,
			const flex_frame_params_t *params,
			uint8_t *buffer, size_t buffer_size,
			int *msgs_packed, int *error)
{
	uint8_t tmp[FLEX_BUFFER_SIZE + 32];
	uint8_t *out;
	size_t full_len, s2_len, sync_overhead;
	int err_local = 0;
	int num_phases, p, w;

	if (!error)
		error = &err_local;
	*error = 0;

	if (!params || !buffer) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	/* S2 size for this speed/modulation */
	s2_len = flex_s2_size(params->bitrate);
	if (s2_len == 0) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	/* Phase count from speed/modulation:
	 *   A1 (1600/2FSK) → 1, A2 (3200/2FSK) → 2,
	 *   A3 (3200/4FSK) → 2, A4 (6400/4FSK) → 4 */
	if (params->bitrate >= 6400 && params->modulation_type == FLEX_MOD_4FSK)
		num_phases = 4;
	else if (params->bitrate >= 3200)
		num_phases = 2;
	else
		num_phases = 1;

	/* Check output buffer can hold S2 + interleaved DATA */
	{
		size_t needed = s2_len
			+ (size_t)(FLEX_WORDS_PER_FRAME * num_phases * 4);
		if (buffer_size < needed) {
			*error = -FLEX_ERR_INVALID_BUFFER;
			return 0;
		}
	}

	/* Sync overhead in flex_encode_frame_multi output: S1(14)+FIW(4)+S2 */
	sync_overhead = 14 + 4 + s2_len;

	/* Use single_phase=1 so flex_encode_frame_multi always produces
	 * 88 words regardless of speed — we handle phase interleaving here. */
	{
		flex_frame_params_t phase_params = *params;
		phase_params.single_phase = 1;

		/* Phase data: 88 words per phase */
		flex_phase_data_t phases[FLEX_MAX_PHASES];
		memset(phases, 0, sizeof(phases));

		/* Determine target phase for the message.
		 * phase >= 0: explicit override, mapped to internal index.
		 * phase < 0 (auto): use phase 0 (A).
		 *
		 * At 3200 (2 phases): A=0→idx 0, C=2→idx 1.
		 * At 6400 (4 phases): A=0, B=1, C=2, D=3 direct. */
		int target_phase = 0;
		if (msg_count == 1 && msgs[0].phase >= 0) {
			if (num_phases >= 4)
				target_phase = msgs[0].phase % 4;
			else if (num_phases == 2)
				target_phase = (msgs[0].phase >= 2) ? 1 : 0;
		}
		phase_params.phase_index = target_phase;

		/* Encode the actual message(s) into a temp buffer */
		full_len = flex_encode_frame_multi(msgs, msg_count, &phase_params,
						   tmp, sizeof(tmp),
						   msgs_packed, error);
		if (full_len == 0)
			return 0;

		if (sync_overhead + (size_t)(FLEX_WORDS_PER_FRAME * 4) > full_len) {
			*error = -FLEX_ERR_INVALID_BUFFER;
			return 0;
		}

		/* Extract 88 data words into the target phase (skip S1+FIW+S2) */
		{
			uint8_t *dp = tmp + sync_overhead;
			for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
				phases[target_phase].words[w] =
					((uint32_t)dp[0] << 24) |
					((uint32_t)dp[1] << 16) |
					((uint32_t)dp[2] << 8) |
					 (uint32_t)dp[3];
				dp += 4;
			}
			phases[target_phase].word_count = FLEX_WORDS_PER_FRAME;
		}

		/* Fill remaining phases with idle pattern.
		 * For 4FSK, LSB phases get all-zeros; MSB phases alternate.
		 * Idle words must be block-interleaved just like message data,
		 * because the phase interleaver operates on interleaved words
		 * and the receiver de-interleaves blocks per-phase. */
		for (p = 0; p < num_phases; p++) {
			if (p == target_phase)
				continue;
			flex_fill_idle_phase(phases[p].words, p,
					     params->modulation_type,
					     params->bitrate, params->collapse);
			/* Block-interleave the idle phase */
			{
				int blk;
				for (blk = 0; blk < FLEX_BLOCKS_PER_FRAME; blk++)
					flex_interleave_block(blk, phases[p].words);
			}
			phases[p].word_count = FLEX_WORDS_PER_FRAME;
		}

		/* Write S2 + interleaved phase data */
		out = buffer;

		out += flex_encode_s2(params->bitrate, params->modulation_type,
				      out, buffer_size - (size_t)(out - buffer));

		{
			/* Bit-interleave phase data (see flex_interleave_phases) */
			size_t il_len = flex_interleave_phases(phases, num_phases,
						      params->modulation_type,
						      params->bitrate, out);
			out += il_len;
		}
	}

	return (size_t)(out - buffer);
}


/* ===== KANJI Character Encoding ===== */

/*
 * Encode a KANJI (16-bit character) message.
 *
 * KANJI mode packs one 16-bit character per 21-bit word (5 padding bits),
 * compared to 7-bit ASCII mode which packs 3 characters per word.
 *
 * Word layout:
 *   Word 0: fragment flags (f0f1) + K-bit checksum (same as alpha)
 *   Word 1+: bits [0..15] = 16-bit character, bits [16..20] = 0 (padding)
 *
 * The vector word uses the same alpha vector type (FLEX_VECTOR_TYPE_ALPHA).
 * The receiver distinguishes KANJI from ASCII via the BIW charset field.
 *
 * Parameters follow the same convention as encode_alpha_message():
 *   frame_words  — 88-word frame data array
 *   msg          — UTF-16 character data as raw bytes (2 bytes per char)
 *   msg_start    — word index where message body starts
 *   fwc_p        — pointer to frame word counter (updated on return)
 *   error        — set to negative error code on failure
 *
 * Returns the encoded vector word.  Only body words are written to
 * frame_words starting at *fwc_p.
 */
static uint32_t encode_kanji_message(uint32_t *frame_words, const char *msg,
				 uint32_t msg_start, uint32_t *fwc_p,
				 int *error)
{
	uint32_t msg_word[FLEX_MAX_MSG_WORDS_ALPHA] = {0};
	uint32_t word_idx, fwc;
	size_t byte_len, num_chars, max_chars;
	uint32_t k_bit;
	uint32_t i;

	if (!msg || !fwc_p) {
		if (error)
			*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/*
	 * Message is raw bytes representing 16-bit characters.
	 * Each character = 2 bytes (big-endian).
	 */
	byte_len = strlen(msg);
	num_chars = byte_len / 2;

	if (num_chars == 0) {
		if (error)
			*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/* Max chars: frame capacity minus header word, 1 char per word */
	max_chars = FLEX_MAX_MSG_WORDS_ALPHA - 1;
	if (num_chars > max_chars)
		num_chars = max_chars;

	/* Word 0: fragment flags f0f1=11 (initial fragment).
	 * Kanji messages currently only support single-fragment encoding
	 * (no C/F/N fragmentation fields — always F=11, C=0). */
	msg_word[0] = FLEX_ALPHA_FRAG_INITIAL;

	/* Pack 16-bit characters, 1 per 21-bit word */
	word_idx = 1;
	for (i = 0; i < (uint32_t)num_chars; i++) {
		uint16_t ch = ((uint8_t)msg[i * 2] << 8)
			    | (uint8_t)msg[i * 2 + 1];
		msg_word[word_idx++] = (uint32_t)ch; /* bits 0-15, bits 16-20 = 0 */
	}

	/* K-bit: 10-bit checksum over all message words (same as alpha) */
	k_bit = 0;
	for (i = 0; i < word_idx; i++) {
		k_bit += (msg_word[i])       & 0xFF;
		k_bit += (msg_word[i] >> 8)  & 0xFF;
		k_bit += (msg_word[i] >> 16) & 0x1F;
	}
	msg_word[0] |= (~k_bit) & 0x3FF;

	/* Write encoded body words to frame (vector word returned to caller) */
	fwc = *fwc_p;
	for (i = 0; i < word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_word[i]));
	*fwc_p = fwc;

	return create_alpha_vector(msg_start, word_idx);
}

/*
 * POCSAG protocol constants for idle batch generation.
 * Per ITU-R M.584 / POCSAG specification.
 */
#define POCSAG_PREAMBLE_WORD	0xAAAAAAAAU
#define POCSAG_PREAMBLE_WORDS	18	/* 576 bits of alternating 1/0 */
#define POCSAG_SYNC_WORD	0x7CD215D8U
#define POCSAG_IDLE_WORD	0x7A89C197U
#define POCSAG_BATCH_CODEWORDS	16	/* 8 address/data pairs = 16 codewords */

/*
 * Generate a POCSAG idle batch for POCSAG/FLEX mixed-mode operation.
 *
 * Output format:
 *   Preamble: 18 words of 0xAAAAAAAA (576 bits alternating 1/0)
 *   Batch:    1 sync codeword (0x7CD215D8)
 *           + 16 idle codewords (0x7A89C197)
 *
 * Total: (18 + 1 + 16) = 35 words = 140 bytes.
 *
 * This is used in network mode when a frame slot is designated for
 * POCSAG mixing. The DSP streams this at 1200 baud 2-FSK.
 *
 * Returns bytes written to buffer, or 0 on error.
 */
size_t flex_generate_pocsag_idle(uint8_t *buffer, size_t buffer_size)
{
	uint8_t *out;
	size_t needed;
	int i;

	/* Total: 18 preamble + 1 sync + 16 idle = 35 words × 4 bytes */
	needed = (POCSAG_PREAMBLE_WORDS + 1 + POCSAG_BATCH_CODEWORDS) * 4;

	if (!buffer || buffer_size < needed)
		return 0;

	out = buffer;

	/* Preamble: 18 words of alternating bits */
	for (i = 0; i < POCSAG_PREAMBLE_WORDS; i++) {
		EMIT_WORD(out, POCSAG_PREAMBLE_WORD);
	}

	/* Sync codeword */
	EMIT_WORD(out, POCSAG_SYNC_WORD);

	/* 16 idle codewords (8 address/data pairs, all idle) */
	for (i = 0; i < POCSAG_BATCH_CODEWORDS; i++) {
		EMIT_WORD(out, POCSAG_IDLE_WORD);
	}

	return (size_t)(out - buffer);
}


/* ===== Message Numbering ===== */

/*
 * NOTE: create_numbered_alpha_vector() removed — dead code.
 * Message number (N) and retrieval flag (R) encoding is handled
 * directly in encode_alpha_message() with correct bit positions
 * per the standard:
 *   bits 13-18: N (6-bit message number, 0-63)
 *   bit  19:    R (message retrieval flag)
 *
 * NOTE: encode_source_indication() removed — dead code.
 * Source indication (SOH/STX framing) is handled
 * inline at the call site in flex.c where the message is composed.
 */
