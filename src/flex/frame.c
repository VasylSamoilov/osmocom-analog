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
#include <time.h>
#include "frame.h"

/* Optional per-message configuration (for mail drop flag support). */
struct flex_msg_config {
	uint8_t mail_drop;  /* 0 or 1 */
};

/*
 * Synchronization patterns (Spec Section 3.2).
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
 * === Table 3.2-1: 1600bps/2-level frame speed ===
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
 * === Table 3.2-2: 3200bps/2-level frame speed ===
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
 * === Table 3.2-3: 3200bps/4-level frame speed ===
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
 * === Table 3.2-4: 6400bps/4-level frame speed ===
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
 * === Table 3.2-5: "A" binary pattern ===
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

/* A codes — active speeds (Table 3.2-5, MSB-first byte order) */
static const uint8_t sync_a1[]        = {0x78, 0xF3, 0x59, 0x39}; /* 1600/2FSK */
static const uint8_t sync_a1_inv[]    = {0x87, 0x0C, 0xA6, 0xC6};
static const uint8_t sync_a2[]        = {0x84, 0xE7, 0x59, 0x39}; /* 3200/2FSK */
static const uint8_t sync_a2_inv[]    = {0x7B, 0x18, 0xA6, 0xC6};
static const uint8_t sync_a3[]        = {0x4F, 0x97, 0x59, 0x39}; /* 3200/4FSK */
static const uint8_t sync_a3_inv[]    = {0xB0, 0x68, 0xA6, 0xC6};
static const uint8_t sync_a4[]        = {0x21, 0x5F, 0x59, 0x39}; /* 6400/4FSK */
static const uint8_t sync_a4_inv[]    = {0xDE, 0xA0, 0xA6, 0xC6};

/* A codes — reserved (Table 3.2-5)
 * Not used by the encoder (no defined frame speed), but kept here
 * for completeness. The RX decoder in dsp.c detects these via
 * FLEX_SYNC_A5..A15 defines and logs them as unsupported. */
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

/* Ar: ERS re-synchronization (Table 3.2-5) */
static const uint8_t sync_ar[]        = {0xCB, 0x20, 0x59, 0x39};
static const uint8_t sync_ar_inv[]    = {0x34, 0xDF, 0xA6, 0xC6};

/* B: baud/level indicator */
static const uint8_t sync_b[]         = {0x55, 0x55};

/*
 * S2 (Second Sync) component constants (Spec Section 3.2).
 *
 * S2 structure: BS2(N) + C(16) + BS2_inv(N) + C_inv(16)
 * See tables above for per-speed BS2 lengths and bit patterns.
 *
 * For 4FSK modes, BS2/BS2_inv are symbol patterns where each symbol
 * maps to a dibit: symbol 1→10, symbol 0→00 (per Fig 3.2-3, 3.2-4).
 *
 * C and C_inv are always 16-bit decoded values, same across all modes.
 */
#define S2_C		0xED84	/* 16 bits: 1110110110000100 */
#define S2_C_INV	0x127B	/* 16 bits: 0001001001111011 */

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
static uint32_t reverse_bits32(uint32_t v)
{
	v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);
	v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);
	v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
	v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
	v = (v >> 16) | (v << 16);
	return v;
}

/* ===== BCH(31,21) Encoding (Spec Section 3.5.2) ===== */

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

/* ===== Word Checksum (Spec Section 3.5.1) ===== */

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

/* ===== Address Encoding (Spec Section 3.8, Appendix A) ===== */

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
	return 0;
}

/*
 * Encode a short capcode (1-1933312) into a single address word.
 * (Spec Appendix A: CAPCODE to binary conversion)
 */
static uint32_t encode_short_address(uint32_t capcode)
{
	uint32_t dw = (capcode + FLEX_SHORT_ADDR_OFFSET) & ((1U << FLEX_BCH_DATA_BITS) - 1);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Encode a long capcode into two address words.
 * (Spec Reference Document A, Section 5.15.5)
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
		/* Set 2-3 */
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

/* ===== Frame/Block Information Words (Spec Sections 3.6, 3.7) ===== */

/*
 * Frame Information Word (Spec Section 3.6).
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
	dw |= (cycle & 0x0F) <<  4;
	dw |= (frame & 0x7F) <<  8;
	dw |= (n     & 0x01) << 15;
	dw |= (r     & 0x01) << 16;
	dw |= (t     & 0x0F) << 17;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word 1 (Spec Section 3.7.1).
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
	dw |= (prio     & 0x0F) <<  4;
	dw |= (e_biw    & 0x03) <<  8;
	dw |= (s_vfield & 0x3F) << 10;
	dw |= (carry    & 0x03) << 16;
	dw |= (collapse & 0x07) << 18;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Vector Words (Spec Section 3.9) ===== */

/* Alphanumeric vector (Spec Section 3.9.4) */
static uint32_t create_alpha_vector(uint32_t msg_start, uint32_t msg_words)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_ALPHA & 0x07) <<  4;
	dw |= (msg_start              & 0x7F) <<  7;
	dw |= (msg_words              & 0x7F) << 14;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/* Numeric vector (Spec Section 3.9.1) */
static uint32_t create_numeric_vector(uint32_t msg_start,
				      uint32_t msg_words, uint32_t kbit)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_NUMERIC & 0x07) <<  4;
	dw |= (msg_start                & 0x7F) <<  7;
	dw |= (msg_words                & 0x07) << 14;
	dw |= (kbit                     & 0x0F) << 17;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/* Tone-only vector (Spec Section 3.9.2) */
static uint32_t create_tone_vector(void)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_TONE & 0x07) << 4;
	dw |= (0x01) << 7;  /* Message type t1t0 = 01 */

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Block Interleaving (Spec Section 3.3) ===== */

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

/* ===== Numeric Character Table (Spec Section 3.10.2, Table 3.10.2.1) ===== */

static uint8_t numeric_char_to_flex(uint8_t ch)
{
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	switch (ch) {
	case 'U': case 'u': return 0xB;  /* Urgency */
	case ' ':            return 0xC;  /* Space */
	case '-': case '_':  return 0xD;  /* Hyphen */
	case ']':            return 0xE;  /* Right bracket */
	case '[':            return 0xF;  /* Left bracket */
	default:             return 0;
	}
}

static int is_valid_numeric_char(char c)
{
	return (c >= '0' && c <= '9') || c == '-' || c == '_' ||
	       c == '[' || c == ']' || c == ' ' || c == 'U' || c == 'u';
}

static int is_valid_numeric_message(const char *msg)
{
	while (*msg) {
		if (!is_valid_numeric_char(*msg))
			return 0;
		msg++;
	}
	return 1;
}

/* ===== Message Encoding (Spec Reference Document A, Section 3.8.8) ===== */

/*
 * Encode alphanumeric message (Spec Section 3.8.8.3).
 *
 * Characters are packed 3 per 21-bit word (7 bits each).
 * Unused character slots are filled with ETX (0x03).
 * Includes fragment flags (f0f1), signature (S-bit), and checksum (K-bit).
 */
static void encode_alpha_message(uint32_t *frame_words, const char *msg,
				 uint32_t msg_start, uint32_t *fwc_p,
				 int is_long, const void *config)
{
	uint32_t msg_word[FLEX_MAX_MSG_WORDS_ALPHA] = {0};
	uint32_t word_idx, fwc;
	size_t len, max_len;
	uint32_t s_bit, k_bit;
	int shift;
	uint32_t i;

	len = strlen(msg);
	max_len = (len > FLEX_MAX_CHARS_ALPHA) ? FLEX_MAX_CHARS_ALPHA : len;

	/* First word: fragment flags f0f1=11 (initial fragment) in bits 11-12 */
	msg_word[0] = FLEX_ALPHA_FRAG_INITIAL;

	/* Mail drop flag if configured */
	if (config) {
		const struct flex_msg_config *cfg = config;
		if (cfg->mail_drop)
			msg_word[0] |= (1U << 20);
	}

	/* Pack 7-bit ASCII characters, 3 per word */
	i = 0;
	shift = 7;  /* First word starts at bit 7 (bits 0-6 reserved for signature) */
	word_idx = 1;

	while (i < max_len) {
		msg_word[word_idx] |= ((uint32_t)msg[i++] & 0x7F) << shift;
		shift += 7;
		if (shift == FLEX_BCH_DATA_BITS) {
			if (++word_idx >= FLEX_MAX_MSG_WORDS_ALPHA)
				break;
			shift = 0;
		}
	}

	/* Pad unused character slots with ETX (0x03) per spec */
	if (shift == 7) {
		msg_word[word_idx] |= (0x03U << 7) | (0x03U << 14);
		word_idx++;
	} else if (shift == 14) {
		msg_word[word_idx] |= (0x03U << 14);
		word_idx++;
	}

	/* S-bit: 7-bit signature = ones' complement of sum of all character values */
	s_bit = 0;
	for (i = 1; i < word_idx; i++) {
		s_bit += (msg_word[i])       & 0x7F;
		s_bit += (msg_word[i] >> 7)  & 0x7F;
		s_bit += (msg_word[i] >> 14) & 0x7F;
	}
	msg_word[1] |= (~s_bit) & 0x7F;

	/* K-bit: 10-bit checksum over all message words */
	k_bit = 0;
	for (i = 0; i < word_idx; i++) {
		k_bit += (msg_word[i])       & 0xFF;
		k_bit += (msg_word[i] >> 8)  & 0xFF;
		k_bit += (msg_word[i] >> 16) & 0x1F;
	}
	msg_word[0] |= (~k_bit) & 0x3FF;

	/* Write vector word and encoded message words to frame */
	fwc = *fwc_p;
	frame_words[fwc++] = create_alpha_vector(msg_start + is_long, word_idx);
	for (i = 0; i < word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_word[i]));
	*fwc_p = fwc;
}

/*
 * Encode numeric message (Spec Section 3.8.8.1).
 *
 * Digits are packed as 4-bit nibbles, 5 per 21-bit word.
 * Includes K-bit checksum.
 */
static void encode_numeric_message(uint32_t *frame_words, const char *msg,
				   uint32_t msg_start, uint32_t *fwc_p,
				   int is_long, const void *config)
{
	uint32_t msg_words[FLEX_MAX_MSG_WORDS_NUMERIC] = {0};
	uint8_t last_ch;
	int last_shift, bit_shift, word_idx, i;
	uint32_t k_bit, fwc;
	uint8_t ch;

	(void)config;

	last_shift = 0;
	bit_shift = 2;  /* First 2 bits reserved for checksum overflow */
	word_idx = 0;
	i = 0;

	while (msg[i] != '\0' && word_idx < FLEX_MAX_MSG_WORDS_NUMERIC) {
		if (bit_shift < FLEX_BCH_DATA_BITS) {
			if (last_shift) {
				/* Carry bits from previous nibble that crossed word boundary */
				ch = numeric_char_to_flex(last_ch) >> (4 - last_shift);
				msg_words[word_idx] |= ((uint32_t)ch << bit_shift);
				bit_shift += last_shift;
				last_shift = 0;
				continue;
			}
			ch = numeric_char_to_flex(msg[i++]);
			msg_words[word_idx] |= ((uint32_t)ch << bit_shift) & ((1U << FLEX_BCH_DATA_BITS) - 1);
			bit_shift += 4;
			continue;
		}
		/* Nibble crossed word boundary — save overflow */
		last_ch = msg[i - 1];
		last_shift = bit_shift - FLEX_BCH_DATA_BITS;
		bit_shift = 0;
		word_idx++;
	}

	/* Pad remaining space with 0xC (space character) */
	for (; bit_shift < 18; bit_shift += 4)
		msg_words[word_idx] |= ((uint32_t)0xC << bit_shift);

	/* K-bit checksum */
	k_bit = 0;
	for (i = 0; i <= word_idx; i++) {
		k_bit += (msg_words[i])       & 0xFF;
		k_bit += (msg_words[i] >> 8)  & 0xFF;
		k_bit += (msg_words[i] >> 16) & 0x1F;
	}
	k_bit &= 0xFF;
	k_bit = (k_bit & 0x3F) + (k_bit >> 6);
	k_bit = ~k_bit;
	msg_words[0] |= (k_bit >> 4) & 0x3;  /* k5k4 bits */

	/* Write vector word and encoded message words to frame */
	fwc = *fwc_p;
	frame_words[fwc++] = create_numeric_vector(msg_start + is_long,
						   word_idx, k_bit);
	for (i = 0; i <= word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_words[i]));
	*fwc_p = fwc;
}

/* Encode tone-only message (Spec Section 3.8.7.2) — vector word only. */
static void encode_tone_message(uint32_t *frame_words, const char *msg,
				uint32_t msg_start, uint32_t *fwc_p,
				int is_long, const void *config)
{
	(void)msg;
	(void)msg_start;
	(void)is_long;
	(void)config;
	frame_words[(*fwc_p)++] = create_tone_vector();
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
	fmsg.polarity = -1.0;
	fmsg.sequence_num = -1;
	fmsg.short_msg_idx = -1;

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

void flex_frame_params_default(flex_frame_params_t *params)
{
	memset(params, 0, sizeof(*params));
	params->baud_rate = 1600;
	params->modulation_type = FLEX_MOD_2FSK;
}

/*
 * Block Information Word 2 (Spec Section 3.7.2).
 *
 * local_id:        Local ID (4 bits, 0-15)
 * coverage_id:     Coverage zone ID (2 bits, 0-3)
 * repeat:          Repeat indicator (1 bit, 0-1)
 * timezone_offset: Timezone offset in half-hours from UTC (-12..+12),
 *                  stored as (offset + 12) in 6 bits (0-24 range)
 */
uint32_t flex_create_biw2(uint32_t local_id, uint32_t coverage_id,
			  uint32_t repeat, int timezone_offset)
{
	uint32_t tz = (uint32_t)((timezone_offset + 12) & 0x3F);
	uint32_t dw = 0;
	dw |= (coverage_id & 0x03) <<  8;
	dw |= (local_id    & 0x0F) << 10;
	dw |= (repeat      & 0x01) << 14;
	dw |= (tz          & 0x3F) << 15;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word 3 (Spec Section 3.7.3).
 *
 * month: Month of year (4 bits, 1-12)
 * day:   Day of month (5 bits, 1-31)
 */
uint32_t flex_create_biw3(uint32_t month, uint32_t day)
{
	uint32_t dw = 0;
	dw |= (month & 0x0F) <<  8;
	dw |= (day   & 0x1F) << 12;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Block Information Word 4 (Spec Section 3.7.4).
 *
 * hour:   Hour of day (5 bits, 0-23)
 * minute: Minute of hour (6 bits, 0-59)
 */
uint32_t flex_create_biw4(uint32_t hour, uint32_t minute)
{
	uint32_t dw = 0;
	dw |= (hour   & 0x1F) <<  8;
	dw |= (minute & 0x3F) << 13;

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

/* Split a long message into fragments that each fit within max_chars_per_fragment.
 * Returns the number of fragments created. Each fragment is a newly allocated
 * string (caller must free). If the message fits in one fragment, return 1
 * with fragments[0] = strdup(message).
 *
 * The caller (flex_fragment_queue) handles assigning retrieval numbers,
 * fragment_index, total_fragments, and f0f1 flags on each flex_msg_t. */
int flex_fragment_message(const char *message, int msg_type,
			  int max_chars_per_fragment,
			  char **fragments, int max_fragments)
{
	int len, count, i, chunk;

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

/* ===== Group Address Encoding (Spec Section 3.8.2.2) ===== */

/*
 * Encode a group address word for common or temporary group addresses.
 *
 * Group addresses use the same short/long address encoding as individual
 * addresses, but with the group flag indicated by setting bit 20 (the MSB
 * of the 21-bit data word) to 1. For temporary group addresses, bit 19
 * is also set.
 *
 * Returns the encoded 32-bit BCH codeword, or 0 on invalid capcode.
 */
uint32_t flex_encode_group_address(uint64_t group_capcode, int is_temporary)
{
	uint32_t dw;

	/* Validate capcode range using the same rules as individual addresses */
	if (!flex_capcode_valid(group_capcode))
		return 0;

	/*
	 * Build the 21-bit data word from the capcode.
	 * Short addresses: single word with offset.
	 * Long addresses: use only the first word (w1) for the group address word.
	 */
	if (is_short_address(group_capcode)) {
		dw = ((uint32_t)group_capcode + FLEX_SHORT_ADDR_OFFSET)
			& ((1U << FLEX_BCH_DATA_BITS) - 1);
	} else if (is_long_address(group_capcode)) {
		uint64_t result;
		uint32_t w1;

		if (group_capcode >= FLEX_LONG_SET12_MIN &&
		    group_capcode <= FLEX_LONG_SET12_MAX) {
			result = group_capcode - FLEX_LONG_OFFSET_A;
			w1 = (result % FLEX_SHORT_ADDR_OFFSET) + 1;
		} else if (group_capcode >= FLEX_LONG_SET34_MIN &&
			   group_capcode <= FLEX_LONG_SET34_MAX) {
			result = group_capcode - FLEX_LONG_OFFSET_A;
			w1 = (result % FLEX_SHORT_ADDR_OFFSET) + 1;
		} else if (group_capcode >= FLEX_LONG_SET23_MIN &&
			   group_capcode <= FLEX_LONG_SET23_MAX) {
			result = group_capcode - FLEX_LONG_OFFSET_B;
			w1 = (result % FLEX_SHORT_ADDR_OFFSET) + FLEX_LONG_W1_SET23;
		} else {
			return 0;
		}
		dw = w1 & ((1U << FLEX_BCH_DATA_BITS) - 1);
	} else {
		return 0;
	}

	/* Set bit 20 (group flag) — MSB of the 21-bit data word */
	dw |= (1U << 20);

	/* Set bit 19 (temporary group flag) if requested */
	if (is_temporary)
		dw |= (1U << 19);

	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Temporary Address Assignment (Spec Section 3.8.2.3) ===== */

/*
 * Encode a temporary address assignment word.
 *
 * Maps a permanent capcode to a temporary address. The temporary address
 * value is encoded as a 21-bit BCH word. Both the permanent capcode and
 * the temporary address must be valid.
 *
 * Returns the encoded 32-bit BCH codeword for the temporary address,
 * or 0 on invalid input.
 */
uint32_t flex_encode_temp_address(uint64_t capcode, uint64_t temp_addr)
{
	uint32_t dw;

	/* Validate permanent capcode */
	if (!flex_capcode_valid(capcode))
		return 0;

	/* Validate temporary address — must fit in 21-bit data word */
	if (temp_addr == 0 || temp_addr > ((1ULL << FLEX_BCH_DATA_BITS) - 1))
		return 0;

	/* Encode the temporary address value as a 21-bit BCH word */
	dw = (uint32_t)temp_addr & ((1U << FLEX_BCH_DATA_BITS) - 1);

	return flex_encode_word(reverse_bits32(dw));
}

/* ===== Hex/Binary Vector and Message Encoding (Spec Section 3.9.3) ===== */

/*
 * Create a hex/binary vector word.
 *
 * The hex vector uses the same type code (011) as numeric but with
 * k-bits = 0110 to indicate hex/binary mode per Section 3.9.3.
 * This distinguishes it from standard numeric vectors which use
 * different k-bit values.
 *
 * Bit layout (21-bit data word before BCH encoding):
 *   bits  4-6:  vector type (011 = 0x3)
 *   bits  7-13: message start word offset
 *   bits 14-16: message word count (3 bits)
 *   bits 17-20: k-bits = 0110 (hex/binary indicator)
 */
static uint32_t create_hex_vector(uint32_t msg_start, uint32_t msg_words)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_HEX_BINARY & 0x07) <<  4;
	dw |= (msg_start                   & 0x7F) <<  7;
	dw |= (msg_words                   & 0x07) << 14;
	dw |= (0x06)                                << 17; /* k-bits = 0110: hex/binary mode */

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
 * Encode hex/binary message (Spec Section 3.9.3).
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
 * The function writes the hex vector word followed by encoded message
 * words to frame_words, updating *fwc_p.
 *
 * On invalid input (non-hex characters), sets *error = -FLEX_ERR_INVALID_MESSAGE
 * and returns without writing any words.
 */
static void encode_hex_message(uint32_t *frame_words, const char *msg,
			       uint32_t msg_start, uint32_t *fwc_p,
			       int is_long, int *error)
{
	uint32_t msg_words[FLEX_MAX_MSG_WORDS_HEX];
	uint32_t fwc;
	size_t len, i;
	int word_idx, nibble_idx;

	/* Validate all characters are hex */
	len = strlen(msg);
	for (i = 0; i < len; i++) {
		if (!is_valid_hex_char(msg[i])) {
			*error = -FLEX_ERR_INVALID_MESSAGE;
			return;
		}
	}

	/* Limit to maximum hex characters */
	if (len > FLEX_MAX_CHARS_HEX)
		len = FLEX_MAX_CHARS_HEX;

	/* Pack nibbles into 21-bit words, 5 nibbles per word */
	memset(msg_words, 0, sizeof(msg_words));
	word_idx = 0;
	nibble_idx = 0;

	for (i = 0; i < len; i++) {
		uint8_t nibble = hex_char_to_nibble(msg[i]);
		msg_words[word_idx] |= ((uint32_t)nibble << (nibble_idx * 4));
		nibble_idx++;
		if (nibble_idx >= 5) {
			nibble_idx = 0;
			word_idx++;
			if (word_idx >= FLEX_MAX_MSG_WORDS_HEX)
				break;
		}
	}

	/* Account for partial last word */
	if (nibble_idx > 0)
		word_idx++;

	/* Write vector word and encoded message words to frame */
	fwc = *fwc_p;
	frame_words[fwc++] = create_hex_vector(msg_start + is_long, word_idx);
	for (i = 0; i < (size_t)word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_words[i]));
	*fwc_p = fwc;
}

/*
 * Create a short instruction vector word (Spec Section 3.9.6).
 *
 * The short instruction vector encodes instruction data directly in the
 * vector word — no message body words are needed (similar to tone-only).
 *
 * Bit layout of the 21-bit data word:
 *   bits  0-3:  checksum (filled by flex_word_checksum)
 *   bits  4-6:  vector type = 111 (0x7 = FLEX_VECTOR_TYPE_SHORT_INSTR)
 *   bits  7-20: instruction data (14 bits, range 0-16383)
 */
static uint32_t create_short_instruction_vector(uint32_t instruction_data)
{
	uint32_t dw = 0;
	dw |= (FLEX_VECTOR_TYPE_SHORT_INSTR & 0x07) << 4;
	dw |= (instruction_data & 0x3FFF) << 7;

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Encode a short instruction message (Spec Section 3.9.6).
 *
 * Parses the message string as a decimal integer instruction value,
 * creates the short instruction vector word, and writes it to frame_words.
 * No message body words are produced — all data is in the vector word.
 *
 * On invalid input (non-numeric string, value out of 14-bit range, or
 * NULL/empty message), sets *error = -FLEX_ERR_INVALID_MESSAGE and
 * returns without writing any words.
 */
static void encode_instruction_message(uint32_t *frame_words, const char *msg,
				       uint32_t msg_start, uint32_t *fwc_p,
				       int is_long, int *error)
{
	unsigned long val;
	char *endptr;

	(void)msg_start;
	(void)is_long;

	if (!msg || !*msg) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return;
	}

	/* Parse as decimal integer */
	val = strtoul(msg, &endptr, 10);

	/* Reject if not a clean integer or has trailing garbage */
	if (*endptr != '\0') {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return;
	}

	/* 14-bit range: 0-16383 */
	if (val > 0x3FFF) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return;
	}

	frame_words[(*fwc_p)++] = create_short_instruction_vector((uint32_t)val);
}

/* ===== Multi-Message Frame Encoding ===== */

/* Forward declaration — defined after flex_encode_frame_multi(). */
static void encode_kanji_message(uint32_t *frame_words, const char *msg,
				 uint32_t msg_start, uint32_t *fwc_p,
				 int is_long, int *error);

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
		 * Word 1 holds 2 chars (shift starts at 7, so bits 7 and 14).
		 * Subsequent words hold 3 chars each (shifts 0, 7, 14).
		 * Total message words = 1 (header) + ceil((len + 2) / 3).
		 * The +2 accounts for the 2 ETX pad chars that always fill
		 * the remainder of the last word.
		 */
		return 1 + (int)((len + 2 + 2) / 3);

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
		/* Hex packing: 5 nibbles per 21-bit word */
		return (int)((len + 4) / 5);

	default:
		return -1;
	}
}

/*
 * Encode a FLEX frame with multiple messages (ARIB STD-43A compliant).
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
 * frame's target speed/modulation (Table 3.2-5):
 *   A1 = 1600/2FSK, A2 = 3200/2FSK, A3 = 3200/4FSK, A4 = 6400/4FSK
 *
 * Returns bytes written (14), or 0 on error.
 */
static size_t flex_encode_s1(int baud_rate, int modulation_type,
			     uint8_t *buffer, size_t buffer_size)
{
	uint8_t *out;

	if (!buffer || buffer_size < 14)
		return 0;

	out = buffer;

	/* BS1: 32-bit alternating 1010... pattern */
	EMIT_SYNC(out, sync_bs1);

	/* A + B + A_inv: selected by target speed/modulation */
	switch (baud_rate) {
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
	EMIT_WORD(out, flex_create_fiw(params->cycle, params->frame,
				       params->roaming, 0, 0));

	return 4;
}

/*
 * Compute S2 byte size for a given speed/modulation without encoding.
 *
 * S2 structure: BS2(N) + C(16) + BS2_inv(N) + C_inv(16)
 * Total bits = 2*N + 32, where N (bs2_bits) varies per speed:
 *   1600/2FSK:  N=4   → 40 bits  =  5 bytes
 *   3200/2FSK:  N=24  → 80 bits  = 10 bytes
 *   3200/4FSK:  N=24  → 80 bits  = 10 bytes
 *   6400/4FSK:  N=64  → 160 bits = 20 bytes
 *
 * Returns byte count, or 0 for invalid baud_rate.
 */
static size_t flex_s2_size(int baud_rate)
{
	int bs2_bits;

	switch (baud_rate) {
	case 1600: bs2_bits = 4;  break;
	case 3200: bs2_bits = 24; break;
	case 6400: bs2_bits = 64; break;
	default:   return 0;
	}

	return (size_t)(2 * bs2_bits + 32 + 7) / 8;
}

/*
 * Encode S2 (second sync block) into buffer.
 *
 * S2 is the first thing transmitted at the frame's target data rate,
 * after the speed switch from 1600/2FSK (S1+FIW).
 * Structure: BS2(N) + C(16) + BS2_inv(N) + C_inv(16)
 * All sizes in bits (after symbol-to-dibit mapping for 4FSK).
 *
 * Per speed/modulation (from spec bit stream tables):
 *   1600/2FSK (Table 3.2-1):  BS2=4 bits   → total  40 bits ( 5 bytes)
 *   3200/2FSK (Table 3.2-2):  BS2=24 bits  → total  80 bits (10 bytes)
 *   3200/4FSK (Table 3.2-3):  BS2=24 bits  → total  80 bits (10 bytes)
 *     (12 symbols × 2 bits/symbol = 24 bits)
 *   6400/4FSK (Table 3.2-4):  BS2=64 bits  → total 160 bits (20 bytes)
 *     (32 symbols × 2 bits/symbol = 64 bits)
 *
 * For 2FSK, BS2 is alternating bits: 1,0,1,0... → 0xAA pattern
 * For 4FSK, BS2 is alternating symbols: 1,0,1,0... where each symbol
 *   maps to a dibit: symbol 1→10, symbol 0→00 → 0x88 pattern
 *   (per Fig 3.2-3, 3.2-4 decoded data)
 *
 * C and C_inv are always 16-bit values (decoded/bit-level),
 * regardless of modulation.
 *
 * Returns bytes written, or 0 on error.
 */
static size_t flex_encode_s2(int baud_rate, int mod_type,
			     uint8_t *buffer, size_t buffer_size)
{
	int bs2_bits, bit_pos, i;
	size_t total_bits, total_bytes;
	int is_4fsk = (mod_type == FLEX_MOD_4FSK);

	/* BS2 bit length from spec tables */
	switch (baud_rate) {
	case 1600: bs2_bits = 4;  break;
	case 3200: bs2_bits = 24; break;
	case 6400: bs2_bits = 64; break;
	default:   return 0;
	}

	total_bits = (size_t)bs2_bits + 16 + (size_t)bs2_bits + 16;
	total_bytes = (total_bits + 7) / 8;

	if (!buffer || buffer_size < total_bytes)
		return 0;

	memset(buffer, 0, total_bytes);
	bit_pos = 0;

	/* BS2: alternating symbol pattern, bit encoding depends on modulation.
	 * 2FSK: symbol 1→bit 1, symbol 0→bit 0 → alternating 10101010...
	 * 4FSK: symbol 1→dibit 10, symbol 0→dibit 00 → 10001000...
	 * Pattern is N/2 symbols (4FSK) or N bits (2FSK): 1,0,1,0,... */
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

	/* C: 16 bits, MSB first */
	for (i = 15; i >= 0; i--) {
		if (S2_C & (1 << i))
			buffer[bit_pos / 8] |= (0x80 >> (bit_pos % 8));
		bit_pos++;
	}

	/* BS2_inv: inverted alternating symbol pattern.
	 * 2FSK: 01010101...
	 * 4FSK: symbols 0,1,0,1... → 00,10,00,10... → 00100010... */
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

	/* C_inv: 16 bits, MSB first */
	for (i = 15; i >= 0; i--) {
		if (S2_C_INV & (1 << i))
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
	uint32_t frame_words[FLEX_WORDS_PER_FRAME] = {0};
	uint8_t *out;
	uint32_t fwc, i;
	int err_local = 0;

	/* Per-message bookkeeping for the packing pass */
	struct msg_info {
		int	addr_words;	/* 1 (short) or 2 (long) */
		int	vector_words;	/* always 1 */
		int	msg_words;	/* body words (0 for tone/instruction/short) */
		int	is_long;	/* 1 if long capcode */
		int	packed;		/* 1 if included in this frame */
	} info[256]; /* max 256 messages per call — generous upper bound */

	/* Ordered indices: priority messages first, then normal */
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
	out += flex_encode_s1(params->baud_rate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));
	out += flex_encode_fiw(params, out,
			       buffer_size - (size_t)(out - buffer));
	out += flex_encode_s2(params->baud_rate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));

	/* ---- Compute BIW count ---- */

	biw_count = 1; /* BIW1 always present */
	e_biw = 0;

	if (params->local_id || params->coverage_id || params->timezone_offset) {
		biw_count = 2; /* BIW1 + BIW2 */
		e_biw = 1;
	}

	if (params->biw_time) {
		biw_count = 4; /* BIW1 + BIW2 + BIW3 + BIW4 */
		e_biw = 3;
	}

	/* ---- First pass: compute per-message word counts ---- */

	if (msg_count > 0 && msgs) {
		for (i = 0; i < (uint32_t)msg_count; i++) {
			int is_long_addr = 0;
			int mw;

			/* Validate capcode */
			if (msgs[i].is_group) {
				if (!flex_capcode_valid(msgs[i].capcode)) {
					info[i].packed = 0;
					info[i].addr_words = 0;
					info[i].vector_words = 0;
					info[i].msg_words = 0;
					info[i].is_long = 0;
					continue;
				}
				info[i].addr_words = 1; /* group = 1 address word */
				info[i].is_long = 0;
			} else {
				if (!is_capcode_valid(msgs[i].capcode, &is_long_addr)) {
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

			info[i].vector_words = 1;

			/* Estimate message body words */
			mw = estimate_msg_words(&msgs[i]);
			if (mw < 0) {
				/* Invalid message — skip */
				info[i].packed = 0;
				info[i].msg_words = 0;
				continue;
			}
			info[i].msg_words = mw;
			info[i].packed = 0;
		}
	}

	/* ---- Build ordering: priority messages first, then normal ---- */

	for (i = 0; i < (uint32_t)msg_count; i++) {
		if (info[i].addr_words == 0 && info[i].vector_words == 0)
			continue; /* skip invalid */
		if (msgs[i].priority)
			order[n_prio++] = (int)i;
	}
	for (i = 0; i < (uint32_t)msg_count; i++) {
		if (info[i].addr_words == 0 && info[i].vector_words == 0)
			continue; /* skip invalid */
		if (!msgs[i].priority)
			order[n_prio + n_norm++] = (int)i;
	}

	/* ---- Greedy packing ---- */

	capacity = FLEX_WORDS_PER_FRAME - biw_count; /* words available after BIWs */

	for (i = 0; i < (uint32_t)(n_prio + n_norm); i++) {
		int idx = order[i];
		int needed = info[idx].addr_words + info[idx].vector_words
			   + info[idx].msg_words;

		if (needed > capacity)
			break; /* no more room */

		info[idx].packed = 1;
		total_addr += info[idx].addr_words;
		total_vector += info[idx].vector_words;
		total_msg += info[idx].msg_words;
		capacity -= needed;
		packed_count++;

		if ((int)i < n_prio)
			prio_addr_words += info[idx].addr_words;
	}

	/* ---- Write BIW1 ---- */

	fwc = 0;
	{
		uint32_t s_vfield = (uint32_t)(biw_count + total_addr);
		frame_words[fwc++] = flex_create_biw1(
			(uint32_t)prio_addr_words,
			(uint32_t)e_biw,
			s_vfield,
			0, /* carry */
			(uint32_t)params->collapse);
	}

	/* ---- Write BIW2/3/4 if enabled ---- */

	if (biw_count >= 2) {
		frame_words[fwc++] = flex_create_biw2(
			params->local_id,
			params->coverage_id,
			0, /* repeat */
			params->timezone_offset);
	}

	if (biw_count >= 4) {
		/* Get current system time for BIW3/BIW4 */
		time_t now = time(NULL);
		struct tm tm_now;
		gmtime_r(&now, &tm_now);

		frame_words[fwc++] = flex_create_biw3(
			(uint32_t)(tm_now.tm_mon + 1),
			(uint32_t)tm_now.tm_mday);
		frame_words[fwc++] = flex_create_biw4(
			(uint32_t)tm_now.tm_hour,
			(uint32_t)tm_now.tm_min);
	}

	/* ---- Write address words (priority first, then normal) ---- */

	for (i = 0; i < (uint32_t)(n_prio + n_norm); i++) {
		int idx = order[i];
		if (!info[idx].packed)
			continue;

		if (msgs[idx].is_group) {
			frame_words[fwc++] = flex_encode_group_address(
				msgs[idx].capcode, 0);
		} else if (info[idx].is_long) {
			uint32_t aw[2];
			encode_long_address(msgs[idx].capcode, aw);
			frame_words[fwc++] = aw[0];
			frame_words[fwc++] = aw[1];
		} else {
			frame_words[fwc++] = encode_short_address(
				(uint32_t)msgs[idx].capcode);
		}
	}

	/* ---- Write vector + message words for each packed message ---- */

	/* msg_start tracks where the next message's body words begin.
	 * It starts after BIW + address + vector words. */
	{
		uint32_t msg_start_word = (uint32_t)(biw_count + total_addr
						     + total_vector);

		for (i = 0; i < (uint32_t)(n_prio + n_norm); i++) {
			int idx = order[i];
			int enc_err = 0;
			if (!info[idx].packed)
				continue;

			switch (msgs[idx].msg_type) {
			case FLEX_FRAME_MSG_TYPE_ALPHA:
				if (msgs[idx].charset == 1) {
					encode_kanji_message(frame_words,
						msgs[idx].message,
						msg_start_word,
						&fwc,
						info[idx].is_long,
						&enc_err);
				} else {
					encode_alpha_message(frame_words,
						msgs[idx].message,
						msg_start_word,
						&fwc,
						info[idx].is_long,
						NULL);
				}
				break;

			case FLEX_FRAME_MSG_TYPE_NUMERIC:
				encode_numeric_message(frame_words,
					msgs[idx].message,
					msg_start_word,
					&fwc,
					info[idx].is_long,
					NULL);
				break;

			case FLEX_FRAME_MSG_TYPE_TONE:
				encode_tone_message(frame_words,
					NULL,
					msg_start_word,
					&fwc,
					info[idx].is_long,
					NULL);
				break;

			case FLEX_FRAME_MSG_TYPE_HEX:
				encode_hex_message(frame_words,
					msgs[idx].message,
					msg_start_word,
					&fwc,
					info[idx].is_long,
					&enc_err);
				break;

			case FLEX_FRAME_MSG_TYPE_INSTRUCTION:
				encode_instruction_message(frame_words,
					msgs[idx].message,
					msg_start_word,
					&fwc,
					info[idx].is_long,
					&enc_err);
				break;

			case FLEX_FRAME_MSG_TYPE_SHORT:
				/*
				 * Short message: tone/short-message vector (type 010)
				 * with message index encoded. Produces 1 vector word,
				 * 0 body words.
				 *
				 * Validate index range 0-127.
				 */
				{
					int sidx = msgs[idx].short_msg_idx;
					if (sidx < 0 || sidx > 127) {
						enc_err = -FLEX_ERR_INVALID_MESSAGE;
					} else {
						uint32_t dw = 0;
						dw |= (FLEX_VECTOR_TYPE_TONE & 0x07) << 4;
						/* t1t0 = 00 for short message sub-type */
						dw |= ((uint32_t)sidx & 0x7F) << 9;
						dw = flex_word_checksum(dw);
						frame_words[fwc++] = flex_encode_word(
							reverse_bits32(dw));
					}
				}
				break;

			default:
				enc_err = -FLEX_ERR_INVALID_MESSAGE;
				break;
			}

			/* Advance msg_start_word past this message's body words */
			msg_start_word += (uint32_t)info[idx].msg_words;

			if (enc_err && !*error)
				*error = enc_err;
		}
	}

	/* ---- Fill remaining words with alternating idle pattern ---- */

	for (; fwc < FLEX_WORDS_PER_FRAME; fwc++)
		frame_words[fwc] = (fwc % 2 == 0) ? FLEX_IDLE_WORD_1
						   : FLEX_IDLE_WORD_2;

	/* ---- Block interleaving (Spec Section 3.3) ---- */

	for (i = 0; i < FLEX_BLOCKS_PER_FRAME; i++)
		flex_interleave_block(i, frame_words);

	/* ---- Write interleaved data to output buffer ---- */

	memcpy(out, frame_words, FLEX_WORDS_PER_FRAME * 4);
	out += FLEX_WORDS_PER_FRAME * 4;

	if (msgs_packed)
		*msgs_packed = packed_count;

	return (size_t)(out - buffer);
}


/* ===== Bit-Level Phase Interleaving (Spec Section 3.3) ===== */

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
 *   3200/4FSK (4 phases A,B,C,D):
 *     hbit=0 → dibit MSB→A, LSB→B; hbit=1 → dibit MSB→C, LSB→D
 *     At the 2FSK bit level this is still alternating A,C per bit,
 *     with B,D carried by the 4FSK second level.
 *
 *   6400/4FSK: same as 3200/4FSK (symbol rate is 3200 baud).
 *
 * For 2FSK encoding (which is what the DSP layer sends for 3200/2FSK),
 * we must bit-interleave: output bit 0 = A_bit0, bit 1 = C_bit0,
 * bit 2 = A_bit1, bit 3 = C_bit1, etc.
 *
 * For 4FSK encoding, the DSP layer sends dibits.  Each dibit's MSB
 * comes from one phase pair (A or C) and LSB from the other (B or D).
 * The interleaving at the symbol level alternates phase pairs:
 * symbol 0 → (A,B), symbol 1 → (C,D), symbol 2 → (A,B), ...
 * This is equivalent to word-level interleaving since each 32-bit
 * word = 16 symbols, and the alternation is per-symbol.
 *
 * This function handles the 2FSK case (num_phases == 2).
 * For 4FSK (num_phases == 4), word-level interleaving is correct
 * because the DSP sends dibits and the phase pairing is handled
 * by the 4-level modulation itself.
 *
 * Parameters:
 *   phases     — array of phase data (each has 88 words)
 *   num_phases — number of phases (2 for 3200/2FSK)
 *   mod_type   — FLEX_MOD_2FSK or FLEX_MOD_4FSK
 *   out        — output buffer (must hold 88 * num_phases * 4 bytes)
 *
 * Returns bytes written.
 */

static size_t flex_interleave_phases(const flex_phase_data_t *phases,
				     int num_phases, int mod_type,
				     int baud_rate, uint8_t *out)
{
	int w, bit;
	uint8_t *dst = out;
	uint8_t cur_byte = 0;
	int out_bit = 0;

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
		 * 3200/2FSK bit-level interleaving (Section 3.3.2).
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

	/*
	 * 4FSK modes: bit-level interleaving into dibits (Section 3.3.2).
	 *
	 * The DSP layer (fsk4_block_encode) reads dibits from the buffer:
	 *   sym = (word >> 30) & 0x03; word <<= 2;
	 * Each dibit = one 4-level symbol.
	 *
	 * PDW (Flex.cpp) at g_sps==3200, level==4:
	 *   hbit=0 → symbol MSB→A[bct], LSB→B[bct]
	 *   hbit=1 → symbol MSB→C[bct], LSB→D[bct], bct++
	 *
	 * So the transmitted symbol stream must alternate:
	 *   (A_bit0,C_bit0), (B_bit0,D_bit0), (A_bit1,C_bit1), ...
	 *
	 * Spec Section 3.3.2 confirms:
	 *   3200/4FSK: "bit 0a → MSB, bit 0c → LSB" for first symbol
	 *     → symbol = (A_bit, C_bit), then (B_bit, D_bit)
	 *     Phase pairing: (A,C) and (B,D) alternate per symbol.
	 *
	 *   6400/4FSK: "bit 0a → MSB, bit 0b → LSB" for first symbol
	 *     → symbol = (A_bit, B_bit), then (C_bit, D_bit)
	 *     Phase pairing: (A,B) and (C,D) alternate per symbol.
	 *
	 * We output dibits packed MSB-first into bytes, matching
	 * fsk4_block_encode's consumption order.
	 *
	 * Total: 88 words × 32 bits × 4 phases = 11264 bits
	 *       = 5632 dibits = 11264 bits = 1408 bytes.
	 */
	{
		/* Phase indices for dibit construction.
		 * Even symbols: MSB from p_msb0, LSB from p_lsb0
		 * Odd symbols:  MSB from p_msb1, LSB from p_lsb1
		 *
		 * 3200/4FSK (spec): (A,C) then (B,D) → indices (0,2),(1,3)
		 * 6400/4FSK (spec): (A,B) then (C,D) → indices (0,1),(2,3)
		 */
		int p_msb0, p_lsb0, p_msb1, p_lsb1;

		if (baud_rate >= 6400) {
			/* 6400/4FSK: (A,B) then (C,D) */
			p_msb0 = 0; p_lsb0 = 1;
			p_msb1 = 2; p_lsb1 = 3;
		} else {
			/* 3200/4FSK: (A,C) then (B,D) */
			p_msb0 = 0; p_lsb0 = 2;
			p_msb1 = 1; p_lsb1 = 3;
		}

		for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
			uint32_t w0m = phases[p_msb0].words[w];
			uint32_t w0l = phases[p_lsb0].words[w];
			uint32_t w1m = phases[p_msb1].words[w];
			uint32_t w1l = phases[p_lsb1].words[w];

			for (bit = 31; bit >= 0; bit--) {
				uint8_t msb, lsb;

				/* Even symbol: (p_msb0, p_lsb0) */
				msb = (w0m >> bit) & 1;
				lsb = (w0l >> bit) & 1;
				cur_byte = (cur_byte << 2) | (msb << 1) | lsb;
				out_bit += 2;
				if (out_bit == 8) {
					*dst++ = cur_byte;
					cur_byte = 0;
					out_bit = 0;
				}

				/* Odd symbol: (p_msb1, p_lsb1) */
				msb = (w1m >> bit) & 1;
				lsb = (w1l >> bit) & 1;
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



/* ===== Multi-Phase Frame Encoding (Spec Section 3.3) ===== */

/*
 * Encode a multi-phase FLEX frame for 3200/6400 bps operation.
 *
 * At higher baud rates, the frame carries multiple independent phases:
 *   3200 bps (2-FSK): 2 phases (A, C)
 *   3200 bps (4-FSK): 4 phases (A, B, C, D)
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
 * For 3200/4FSK, data is dibit-interleaved: (A,C), (B,D) alternating.
 * For 6400/4FSK, data is dibit-interleaved: (A,B), (C,D) alternating.
 *
 * The sync pattern (A1/A2/A3/A4) is selected based on baud_rate and
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
	if (params->baud_rate == 1600 && params->modulation_type == FLEX_MOD_4FSK) {
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}
	if (params->baud_rate == 6400 && params->modulation_type == FLEX_MOD_2FSK) {
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
		size_t s2_bytes = flex_s2_size(params->baud_rate);
		size_t needed = 14 + 4 + s2_bytes
			      + (size_t)(FLEX_WORDS_PER_FRAME * num_phases * 4);
		if (buffer_size < needed) {
			*error = -FLEX_ERR_INVALID_BUFFER;
			return 0;
		}
	}

	out = buffer;

	/* ---- S1 + FIW + S2 ---- */
	out += flex_encode_s1(params->baud_rate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));
	out += flex_encode_fiw(params, out,
			       buffer_size - (size_t)(out - buffer));
	out += flex_encode_s2(params->baud_rate, params->modulation_type,
			      out, buffer_size - (size_t)(out - buffer));

	/* ---- Data: bit-interleaved phase data ---- */

	/*
	 * Phase interleaving depends on modulation type:
	 *   3200/2FSK: bit-level (A_bit, C_bit, A_bit, C_bit, ...)
	 *   3200/4FSK: dibit-level, phase pairs (A,C) and (B,D) alternate
	 *   6400/4FSK: dibit-level, phase pairs (A,B) and (C,D) alternate
	 * See flex_interleave_phases() for details and PDW evidence.
	 */
	out += flex_interleave_phases(phases, num_phases,
				      params->modulation_type,
				      params->baud_rate, out);

	return (size_t)(out - buffer);
}


/* ===== Split Sync/Data Encoding (ARIB STD-43A Section 3.2) ===== */

/*
 * Encode the sync portion of a FLEX frame: S1 + FIW.
 *
 * Per ARIB STD-43A Section 3.2, the sync portion is ALWAYS transmitted
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
 * The sync code (A1/A2/A3/A4) is selected based on baud_rate and
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
	out += flex_encode_s1(params->baud_rate, params->modulation_type,
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
 * phase 0 and remaining phases are filled with idle frames. Phase
 * data is then interleaved into the output:
 *   3200/2FSK (2 phases): BIT-level — A_bit, C_bit, A_bit, C_bit, ...
 *   3200/4FSK (4 phases): dibit-level — (A,C), (B,D) alternating
 *   6400/4FSK (4 phases): dibit-level — (A,B), (C,D) alternating
 * See flex_interleave_phases() for details and PDW evidence.
 *
 * Output sizes:
 *   1600/2FSK (1 phase):  S2(5)  + 352  = 357 bytes
 *   3200/2FSK (2 phases): S2(10) + 704  = 714 bytes
 *   3200/4FSK (4 phases): S2(10) + 1408 = 1418 bytes
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
	s2_len = flex_s2_size(params->baud_rate);
	if (s2_len == 0) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	/* Phase count from speed/modulation:
	 *   A1 (1600/2FSK) → 1, A2 (3200/2FSK) → 2,
	 *   A3 (3200/4FSK) → 4, A4 (6400/4FSK) → 4 */
	if (params->baud_rate >= 3200 && params->modulation_type == FLEX_MOD_4FSK)
		num_phases = 4;
	else if (params->baud_rate >= 3200)
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

		/* Phase 0: encode the actual message(s) */
		full_len = flex_encode_frame_multi(msgs, msg_count, &phase_params,
						   tmp, sizeof(tmp),
						   msgs_packed, error);
		if (full_len == 0)
			return 0;

		if (sync_overhead + (size_t)(FLEX_WORDS_PER_FRAME * 4) > full_len) {
			*error = -FLEX_ERR_INVALID_BUFFER;
			return 0;
		}

		/* Extract 88 data words from phase 0 (skip S1+FIW+S2) */
		{
			uint8_t *dp = tmp + sync_overhead;
			for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
				phases[0].words[w] =
					((uint32_t)dp[0] << 24) |
					((uint32_t)dp[1] << 16) |
					((uint32_t)dp[2] << 8) |
					 (uint32_t)dp[3];
				dp += 4;
			}
			phases[0].word_count = FLEX_WORDS_PER_FRAME;
		}

		/* Fill remaining phases with idle frames */
		for (p = 1; p < num_phases; p++) {
			flex_frame_msg_t idle_msg;
			int idle_packed = 0;

			memset(&idle_msg, 0, sizeof(idle_msg));
			idle_msg.capcode = 1;
			idle_msg.msg_type = FLEX_FRAME_MSG_TYPE_TONE;
			idle_msg.message = "";
			idle_msg.message_length = 0;
			idle_msg.speed = params->baud_rate;
			idle_msg.polarity = -1.0;

			full_len = flex_encode_frame_multi(&idle_msg, 1, &phase_params,
							   tmp, sizeof(tmp),
							   &idle_packed, &err_local);
			if (full_len > 0) {
				uint8_t *dp = tmp + sync_overhead;
				for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
					phases[p].words[w] =
						((uint32_t)dp[0] << 24) |
						((uint32_t)dp[1] << 16) |
						((uint32_t)dp[2] << 8) |
						 (uint32_t)dp[3];
					dp += 4;
				}
				phases[p].word_count = FLEX_WORDS_PER_FRAME;
			}
		}

		/* Write S2 + interleaved phase data */
		out = buffer;

		out += flex_encode_s2(params->baud_rate, params->modulation_type,
				      out, buffer_size - (size_t)(out - buffer));

		/* Bit-interleave phase data (see flex_interleave_phases) */
		out += flex_interleave_phases(phases, num_phases,
					      params->modulation_type,
					      params->baud_rate, out);
	}

	return (size_t)(out - buffer);
}


/* ===== KANJI Character Encoding (ARIB STD-43A Section 3.10.2.3) ===== */

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
 *   is_long      — 1 if long capcode (affects vector word encoding)
 *   error        — set to negative error code on failure
 */
static void encode_kanji_message(uint32_t *frame_words, const char *msg,
				 uint32_t msg_start, uint32_t *fwc_p,
				 int is_long, int *error)
{
	uint32_t msg_word[FLEX_MAX_MSG_WORDS_ALPHA] = {0};
	uint32_t word_idx, fwc;
	size_t byte_len, num_chars, max_chars;
	uint32_t k_bit;
	uint32_t i;

	if (!msg || !fwc_p) {
		if (error)
			*error = -FLEX_ERR_INVALID_MESSAGE;
		return;
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
		return;
	}

	/* Max chars: frame capacity minus header word, 1 char per word */
	max_chars = FLEX_MAX_MSG_WORDS_ALPHA - 1;
	if (num_chars > max_chars)
		num_chars = max_chars;

	/* Word 0: fragment flags f0f1=11 (initial fragment) */
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

	/* Write vector word and encoded message words to frame */
	fwc = *fwc_p;
	frame_words[fwc++] = create_alpha_vector(msg_start + is_long, word_idx);
	for (i = 0; i < word_idx; i++)
		frame_words[fwc++] = flex_encode_word(reverse_bits32(msg_word[i]));
	*fwc_p = fwc;
}


/* ===== POCSAG Idle Batch Generation ===== */

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


/* ===== Message Numbering (ARIB STD-43A Section 8.4) ===== */

/*
 * Encode message sequence number into a vector word extension.
 *
 * Per Section 8.4, the Message Numbering service assigns a 6-bit
 * message number (N, range 0-63) to each message for duplicate
 * detection and ordering at the pager. The retrieval flag R is
 * normally 1 (first transmission) and 0 on retransmission.
 *
 * The message number is encoded in the vector word's upper data bits:
 *   Bits 14-19: N (6-bit message number, 0-63)
 *   Bit 20:     R (retrieval flag, 1 = first, 0 = retransmit)
 *
 * This function creates a "numbered" variant of the alpha vector
 * (type 101 with numbering extension). The msg_start and msg_words
 * fields occupy the same positions as the standard alpha vector.
 *
 * Parameters:
 *   msg_start    — word index where message body starts (7 bits)
 *   msg_words    — number of message body words (7 bits)
 *   sequence_num — message number 0-63 (wraps via caller)
 *
 * Returns: BCH-encoded 32-bit vector codeword.
 */
static uint32_t create_numbered_alpha_vector(uint32_t msg_start,
					     uint32_t msg_words,
					     int sequence_num)
{
	uint32_t dw = 0;

	/* Vector type: alpha (101) */
	dw |= (FLEX_VECTOR_TYPE_ALPHA & 0x07) << 4;
	/* Message start word offset */
	dw |= (msg_start & 0x7F) << 7;
	/* Message word count */
	dw |= (msg_words & 0x7F) << 14;

	/*
	 * When sequence_num >= 0, the message number is encoded in the
	 * first message word's header (fragment flags area), not in the
	 * vector word itself. The vector type remains alpha (101).
	 *
	 * The actual N value is carried in the message header word:
	 *   Bits 0-5:  N (message number, 0-63)
	 *   Bit 6:     R (retrieval flag, 1 = first transmission)
	 *
	 * This function returns the standard alpha vector; the caller
	 * is responsible for encoding N into the message header word.
	 */
	(void)sequence_num; /* N is encoded in message header, not vector */

	dw = flex_word_checksum(dw);
	return flex_encode_word(reverse_bits32(dw));
}

/*
 * Encode message number into the first message word (header).
 *
 * Per Section 8.4, when message numbering is active:
 *   Bits 0-5 of the header word carry N (message number, 0-63)
 *   Bit 6 carries R (retrieval flag: 1 = first, 0 = retransmit)
 *
 * The fragment flags (f0f1) and checksum (K) occupy their normal
 * positions in the remaining bits.
 *
 * Parameters:
 *   header_word  — pointer to the first message word (modified in place)
 *   sequence_num — message number 0-63
 *
 * The sequence counter wraps at 63 (maximum value per spec).
 */
static void encode_message_number(uint32_t *header_word, int sequence_num)
{
	if (!header_word || sequence_num < 0)
		return;

	/* Clamp to 6-bit range (0-63) */
	uint32_t n = (uint32_t)(sequence_num % 64);

	/* R = 1 (first transmission) */
	uint32_t r = 1;

	/* Encode N in bits 0-5, R in bit 6 of the header word.
	 * Clear existing bits in that range first. */
	*header_word &= ~0x7FU;       /* clear bits 0-6 */
	*header_word |= (n & 0x3F);   /* N: bits 0-5 */
	*header_word |= (r << 6);     /* R: bit 6 */
}


/* ===== Source Indication (ARIB STD-43A Section 8.5) ===== */

/*
 * Encode source indication (callback number / originator ID) into
 * the message header area.
 *
 * Per Section 8.5, the Source Indication service transmits a short
 * identifier for the calling party. When source_id is provided
 * (non-NULL, non-empty), it is prepended to the message content
 * as a SOH-delimited field in the alpha message body:
 *
 *   <SOH> source_id <STX> message_content
 *
 * SOH (0x01) marks the start of the source indication field.
 * STX (0x02) marks the transition to the actual message content.
 *
 * When source_id is NULL or empty, no source indication is added
 * and the message is encoded normally.
 *
 * Parameters:
 *   dest        — output buffer for the combined message
 *   dest_size   — size of output buffer
 *   source_id   — source identifier string (NULL = omit)
 *   message     — original message content
 *
 * Returns: length of the combined message written to dest,
 *          or 0 if source_id is NULL/empty (message unchanged).
 */
static int encode_source_indication(char *dest, int dest_size,
				    const char *source_id,
				    const char *message)
{
	int pos = 0;
	int src_len, msg_len;

	if (!dest || dest_size < 4)
		return 0;

	if (!source_id || source_id[0] == '\0')
		return 0;

	src_len = (int)strlen(source_id);
	msg_len = message ? (int)strlen(message) : 0;

	/* Need: SOH + source_id + STX + message + NUL */
	if (1 + src_len + 1 + msg_len + 1 > dest_size)
		return 0;

	/* SOH: start of source indication */
	dest[pos++] = 0x01;

	/* Source identifier */
	memcpy(dest + pos, source_id, src_len);
	pos += src_len;

	/* STX: start of message text */
	dest[pos++] = 0x02;

	/* Original message content */
	if (msg_len > 0) {
		memcpy(dest + pos, message, msg_len);
		pos += msg_len;
	}

	dest[pos] = '\0';
	return pos;
}
