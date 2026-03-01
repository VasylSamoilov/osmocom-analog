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
#include "frame.h"

/* Optional per-message configuration (for mail drop flag support). */
struct flex_msg_config {
	uint8_t mail_drop;  /* 0 or 1 */
};

/*
 * Synchronization patterns (Spec Section 3.2, Tables 3.2-1 and 3.2-5).
 *
 * BS  = Bit Sync: alternating 1/0 pattern (16 bits)
 * A1  = Frame Sync code for 1600 baud, 2-level FSK (32 bits)
 * Ar  = ERS (Emergency Re-Synchronization) frame sync (32 bits)
 *       Shares lower 16 bits with A1; upper 16 bits differ.
 *       Used in the ERS preamble to force pager re-acquisition.
 * B   = Baud/level indicator (16 bits)
 * C   = Second sync block, follows FIW (40 bits)
 *
 * _inv variants are bitwise inversions for polarity detection.
 * All values verified against ARIB STD-43A Tables 3.2-1 and 3.2-5.
 *
 * Bit patterns (binary, LSB transmitted first per spec):
 *   BS:      1010101010101010
 *   A1:      0111100011110011 0101100100111001
 *   Ar:      1100101100100000 0101100100111001
 *   inv.A1:  1000011100001100 1010011011000110
 *   inv.Ar:  0011010011011111 1010011011000110
 */
static const uint8_t sync_bs[]        = {0xAA, 0xAA};
static const uint8_t sync_bs_inv[]    = {0x55, 0x55};
static const uint8_t sync_bs1[]       = {0xAA, 0xAA, 0xAA, 0xAA};
static const uint8_t sync_a1[]        = {0x78, 0xF3, 0x59, 0x39};
static const uint8_t sync_a1_inv[]    = {0x87, 0x0C, 0xA6, 0xC6};
static const uint8_t sync_ar[]        = {0xCB, 0x20, 0x59, 0x39};
static const uint8_t sync_ar_inv[]    = {0x34, 0xDF, 0xA6, 0xC6};
static const uint8_t sync_b[]         = {0x55, 0x55};
static const uint8_t sync_c[]         = {0xAE, 0xD8, 0x45, 0x12, 0x7B};

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
 * Encode a complete FLEX frame.
 *
 * Assembles: ERS preamble + S1 sync + FIW + S2 sync + data block
 * (BIW + address + vector + message + idle fill + interleave).
 *
 * Returns bytes written to buffer, or 0 on error.
 */
size_t flex_encode_frame(uint64_t capcode, int msg_type,
			 const char *message, uint8_t *buffer,
			 size_t buffer_size, int *error)
{
	uint32_t frame_words[FLEX_WORDS_PER_FRAME] = {0};
	uint8_t *out;
	uint32_t addr_words[2] = {0};
	int is_long;
	uint32_t fwc, i;

	if (!error)
		return 0;
	*error = 0;

	/* Resolve AUTO type */
	if (msg_type == FLEX_FRAME_MSG_TYPE_AUTO)
		msg_type = flex_detect_msg_type(message,
			message ? (int)strlen(message) : 0);

	/* Validate message */
	switch (msg_type) {
	case FLEX_FRAME_MSG_TYPE_ALPHA:
		if (!message || !*message || strlen(message) > FLEX_MAX_CHARS_ALPHA) {
			*error = -FLEX_ERR_INVALID_MESSAGE;
			return 0;
		}
		break;
	case FLEX_FRAME_MSG_TYPE_NUMERIC:
		if (!message || !*message || strlen(message) > FLEX_MAX_CHARS_NUMERIC) {
			*error = -FLEX_ERR_INVALID_MESSAGE;
			return 0;
		}
		if (!is_valid_numeric_message(message)) {
			*error = -FLEX_ERR_INVALID_MESSAGE;
			return 0;
		}
		break;
	case FLEX_FRAME_MSG_TYPE_TONE:
		break;
	default:
		*error = -FLEX_ERR_INVALID_MESSAGE;
		return 0;
	}

	/* Validate capcode */
	if (!is_capcode_valid(capcode, &is_long)) {
		*error = -FLEX_ERR_INVALID_CAPCODE;
		return 0;
	}

	/* Validate output buffer */
	if (!buffer || buffer_size < FLEX_BUFFER_SIZE) {
		*error = -FLEX_ERR_INVALID_BUFFER;
		return 0;
	}

	out = buffer;
	fwc = 0;

	/* === Preamble: Emergency Re-Synchronization (Spec Section 3.2.1) ===
	 *
	 * The ERS preamble forces pagers to re-acquire synchronization.
	 * Pattern per cycle: BS + Ar + BS_inv + Ar_inv (12 bytes, 96 bits).
	 *
	 * Duration requirement depends on the maximum collapse cycle value:
	 *   m=0: 1.875 sec (pager decodes all frames)
	 *   m=7: 4 minutes (pager decodes every 128th frame)
	 *
	 * Default mode uses collapse=0, so 35 cycles (~2.1 sec at 1600 baud)
	 * exceeds the minimum 1.875 sec requirement.
	 */
	for (i = 0; i < FLEX_ERS_CYCLES; i++) {
		EMIT_SYNC(out, sync_bs);
		EMIT_SYNC(out, sync_ar);
		EMIT_SYNC(out, sync_bs_inv);
		EMIT_SYNC(out, sync_ar_inv);
	}

	/* === S1: Frame synchronization (Spec Section 3.2.1) === */
	EMIT_SYNC(out, sync_bs1);
	EMIT_SYNC(out, sync_a1);
	EMIT_SYNC(out, sync_b);
	EMIT_SYNC(out, sync_a1_inv);

	/* === Frame Information Word (Spec Section 3.6) ===
	 *
	 * Default mode: hardcoded cycle=0, frame=0, collapse=0.
	 *
	 * Frame/cycle in the FIW = "what time is it" (pager's clock reference).
	 * Collapse in BIW1 = "how often should I wake up" (battery-saving schedule).
	 * These two mechanisms are independent.
	 *
	 * With collapse=0 (m=0, 2^0=1), the pager decodes ALL frames because
	 * the wake condition `frame_number mod 2^m == n mod 2^m` simplifies to
	 * `frame_number mod 1 == n mod 1` → `0 == 0` → always true.
	 * Frame assignment becomes irrelevant — the pager never skips a frame.
	 *
	 * This means frame/cycle values could be advanced to real wall-clock
	 * time without breaking anything: the pager would still decode every
	 * frame, just with accurate time references. Collapse=0 is the correct
	 * default for any scenario that doesn't require battery saving.
	 *
	 * For future real-time sync:
	 *   cycle = (minute % 60) / 4       (0-14)
	 *   frame = f(second, subsecond)     (0-127)
	 * See FLEX_STD43A_IMPROVEMENT_PLAN.md Section 1.1.
	 */
	EMIT_WORD(out, flex_create_fiw(0, 0, 0, 0, 0));

	/* === S2: C block (Spec Section 3.2.2) === */
	EMIT_SYNC(out, sync_c);

	/* === Data block === */

	/* BIW1: vector field starts after BIW + address words.
	 *
	 * collapse=0 (last parameter) is the default mode.
	 *
	 * === Address-to-Frame Assignment (Spec Appendix A, Section 3) ===
	 *
	 * Each capcode maps to a frame and phase:
	 *   Frame = (INT[capcode / 16]) mod 128       (0-127)
	 *   Phase = (INT[capcode / 4])  mod 4          (0-3)
	 *
	 * Small capcode example — capcode 1337:
	 *   Frame = INT(1337/16) mod 128 = 83 mod 128 = 83
	 *   Phase = INT(1337/4)  mod 4   = 334 mod 4  = 2 (phase C)
	 *
	 * Large capcode example — capcode 1000000:
	 *   Frame = INT(1000000/16) mod 128 = 62500 mod 128 = 36
	 *   Phase = INT(1000000/4)  mod 4   = 250000 mod 4  = 0 (phase A)
	 *
	 * === Collapse Cycle (Spec Section 3.1.2) ===
	 *
	 * The collapse value m (0-7) in BIW1 controls battery saving.
	 * Pager wakes every 2^m frames. Wake condition:
	 *   frame_number mod 2^m == assigned_frame mod 2^m
	 *
	 * The pager compares the lower (7-m) bits of the frame number.
	 * Smaller m = more frames decoded = shorter paging delay = more
	 * battery drain. Larger m = fewer frames = longer delay = less drain.
	 *
	 * Each frame = 1.875 sec. Worst-case delay = 2^m * 1.875 sec.
	 *
	 * === TX Scheduling ===
	 *
	 * The transmitter must place each message in a frame where:
	 *   tx_frame mod 2^m == assigned_frame mod 2^m
	 *
	 * To find the next valid TX frame from the current frame `f`:
	 *   target = assigned_frame mod 2^m
	 *   remainder = f mod 2^m
	 *   if (remainder <= target)
	 *       next_frame = f + (target - remainder)
	 *   else
	 *       next_frame = f + (2^m - remainder + target)
	 *
	 * With m=0, 2^m=1, so target=0, remainder=0, next_frame=f.
	 * Any frame is valid — transmit immediately.
	 *
	 * === Worked Examples ===
	 *
	 * Capcode 1337 (short addr, assigned frame n=83):
	 *
	 * 1) No power saving (m=0, default mode):
	 *    2^0=1. frame mod 1 == 83 mod 1 → 0==0 (always true).
	 *    Pager decodes ALL 128 frames. TX: any frame.
	 *    Worst-case delay: 1.875 sec (1 frame period).
	 *    ERS needs: 1.875 sec.
	 *
	 * 2) Small operator (m=3, collapse cycle=8):
	 *    2^3=8. frame mod 8 == 83 mod 8 = 3.
	 *    Pager wakes on: 3,11,19,27,35,43,51,59,67,75,83,91,99,
	 *    107,115,123 (16 frames/cycle).
	 *    TX: must place message in frame where frame mod 8 == 3.
	 *    Worst-case delay: 8 * 1.875 = 15 sec.
	 *    ERS needs: 15 sec.
	 *
	 * 3) Large operator (m=4, standard FLEX collapse):
	 *    2^4=16. frame mod 16 == 83 mod 16 = 3.
	 *    Pager wakes on: 3,19,35,51,67,83,99,115 (8 frames/cycle).
	 *    TX: must place message in frame where frame mod 16 == 3.
	 *    Worst-case delay: 16 * 1.875 = 30 sec.
	 *    ERS needs: 30 sec.
	 *
	 * 4) Maximum battery saving (m=7, collapse cycle=128):
	 *    2^7=128. frame mod 128 == 83 mod 128 = 83.
	 *    Pager wakes on frame 83 ONLY (1 frame/cycle).
	 *    TX: must place message in frame 83 exactly.
	 *    Worst-case delay: 128 * 1.875 = 240 sec (4 min).
	 *    ERS needs: 240 sec (full cycle).
	 *
	 * Capcode 1000000 (short addr, assigned frame n=36):
	 *
	 * 1) No power saving (m=0): same as above — any frame, 1.875 sec.
	 *
	 * 2) Small operator (m=3):
	 *    frame mod 8 == 36 mod 8 = 4.
	 *    Pager wakes on: 4,12,20,28,36,44,52,60,68,76,84,92,100,
	 *    108,116,124 (16 frames/cycle).
	 *    TX: frame where frame mod 8 == 4.
	 *    Worst-case delay: 15 sec.
	 *
	 * 3) Large operator (m=4):
	 *    frame mod 16 == 36 mod 16 = 4.
	 *    Pager wakes on: 4,20,36,52,68,84,100,116 (8 frames/cycle).
	 *    TX: frame where frame mod 16 == 4.
	 *    Worst-case delay: 30 sec.
	 *
	 * 4) Maximum battery saving (m=7):
	 *    frame mod 128 == 36. Pager wakes on frame 36 only.
	 *    TX: frame 36 exactly. Worst-case delay: 240 sec.
	 *
	 * With m=0 (default), the wake condition is trivially true for all
	 * frames, so the pager decodes everything regardless of its capcode.
	 * TX scheduling is unnecessary — transmit in the current frame.
	 */
	frame_words[fwc++] = flex_create_biw1(0, 0, 2 + is_long, 0, 0);

	/* Address word(s) */
	if (is_long) {
		encode_long_address(capcode, addr_words);
		frame_words[fwc++] = addr_words[0];
		frame_words[fwc++] = addr_words[1];
	} else {
		frame_words[fwc++] = encode_short_address((uint32_t)capcode);
	}

	/* Message encoding (vector + data words) */
	switch (msg_type) {
	case FLEX_FRAME_MSG_TYPE_ALPHA:
		encode_alpha_message(frame_words, message,
				     3 + is_long, &fwc, is_long, NULL);
		break;
	case FLEX_FRAME_MSG_TYPE_NUMERIC:
		encode_numeric_message(frame_words, message,
				       3 + is_long, &fwc, is_long, NULL);
		break;
	case FLEX_FRAME_MSG_TYPE_TONE:
		encode_tone_message(frame_words, NULL,
				    3 + is_long, &fwc, is_long, NULL);
		break;
	}

	/* Fill remaining words with alternating idle pattern (Spec Section 3.4.1) */
	for (; fwc < FLEX_WORDS_PER_FRAME; fwc++) {
		frame_words[fwc] = (fwc % 2 == 0) ? FLEX_IDLE_WORD_1 : FLEX_IDLE_WORD_2;
	}

	/* Block interleaving (Spec Section 3.3) */
	for (i = 0; i < FLEX_BLOCKS_PER_FRAME; i++)
		flex_interleave_block(i, frame_words);

	/* Write interleaved data to output buffer */
	EMIT_SYNC(out, frame_words);

	return (size_t)(out - buffer);
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
