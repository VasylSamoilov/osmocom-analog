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

/* ===== Frame Structure (Spec Section 3.3) ===== */

#define FLEX_BAUD_RATE		1600
#define FLEX_BLOCKS_PER_FRAME	11
#define FLEX_WORDS_PER_BLOCK	8
#define FLEX_WORDS_PER_FRAME	(FLEX_BLOCKS_PER_FRAME * FLEX_WORDS_PER_BLOCK)

/* ===== Idle Word Patterns (Spec Section 3.4.1) ===== */

/* Idle word 1: all ones (even-indexed idle words) */
#define FLEX_IDLE_WORD_1	0xFFFFFFFFU
/* Idle word 2: all zeros (odd-indexed idle words) */
#define FLEX_IDLE_WORD_2	0x00000000U

/* ===== Emergency Re-Synchronization (Spec Section 3.2.1) ===== */

/* Number of ERS cycles in preamble.
 * Each cycle = BS + AR + BS_inv + AR_inv = 12 bytes = 96 bits.
 * 35 cycles provides ~2.1 seconds of preamble at 1600 baud.
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

/* ===== Vector Word Types (Spec Section 3.9) ===== */

#define FLEX_VECTOR_TYPE_TONE		0x2	/* Tone-only / short message (3.9.2) */
#define FLEX_VECTOR_TYPE_NUMERIC	0x3	/* Standard numeric (3.9.1) */
#define FLEX_VECTOR_TYPE_ALPHA		0x5	/* Alphanumeric (3.9.4) */

/* ===== Alpha Message Fragment Flags (Spec Section 3.8.8.3) ===== */

/* f0f1 = 11 indicates initial (and possibly only) fragment */
#define FLEX_ALPHA_FRAG_INITIAL		0x1800U

/* ===== Frame Buffer Size ===== */

/*
 * Total bytes for one encoded FLEX frame:
 *   ERS:   35 cycles * (2+4+2+4) bytes  = 420 bytes
 *   S1:    BS1(4) + A1(4) + B(2) + A1_inv(4) = 14 bytes
 *   FIW:   1 codeword                   =   4 bytes
 *   S2:    C block                       =   5 bytes
 *   Frame: 88 words * 4 bytes            = 352 bytes
 *   Total:                               = 795 bytes
 */
#define FLEX_BUFFER_SIZE	795

/* ===== Error Codes ===== */

#define FLEX_ERR_INVALID_MESSAGE	1
#define FLEX_ERR_INVALID_CAPCODE	2
#define FLEX_ERR_INVALID_BUFFER		3

/* ===== Message Type Constants ===== */
/* (int values to avoid circular dependency with flex.h enum) */

#define FLEX_FRAME_MSG_TYPE_AUTO	0
#define FLEX_FRAME_MSG_TYPE_TONE	1
#define FLEX_FRAME_MSG_TYPE_NUMERIC	2
#define FLEX_FRAME_MSG_TYPE_ALPHA	3

/* ===== Public API ===== */

/* Encode a complete FLEX frame (ERS + sync + FIW + data + interleave).
 * Returns bytes written to buffer, or 0 on error (with *error set). */
size_t flex_encode_frame(uint64_t capcode, int msg_type,
			 const char *message, uint8_t *buffer,
			 size_t buffer_size, int *error);

/* Auto-detect message type from content */
int flex_detect_msg_type(const char *message, int length);

/* Validate capcode (returns 1 if valid, 0 if invalid) */
int flex_capcode_valid(uint64_t capcode);

/* Individual encoding functions (exposed for testing) */
uint32_t flex_encode_word(uint32_t dw);
uint32_t flex_word_checksum(uint32_t dw);
void flex_interleave_block(uint32_t block_num, uint32_t *frame_words);
uint32_t flex_create_fiw(uint32_t cycle, uint32_t frame, uint32_t n,
			 uint32_t r, uint32_t t);
uint32_t flex_create_biw1(uint32_t prio, uint32_t e_biw,
			  uint32_t s_vfield, uint32_t carry,
			  uint32_t collapse);

/* Text utilities */
const char *flex_print_message(const char *message, int message_length);
int flex_scan_message(const char *message_input, int message_input_length,
		      char *message_output, int message_output_length);

#endif /* FLEX_FRAME_H */
