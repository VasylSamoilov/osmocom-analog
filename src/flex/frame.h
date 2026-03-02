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

/* ===== Vector Word Types (Spec Section 3.9) ===== */

#define FLEX_VECTOR_TYPE_TONE		0x2	/* Tone-only / short message (3.9.2) */
#define FLEX_VECTOR_TYPE_NUMERIC	0x3	/* Standard numeric (3.9.1) */
#define FLEX_VECTOR_TYPE_HEX_BINARY	0x3	/* type 011 with hex k-bits (3.9.3) */
#define FLEX_VECTOR_TYPE_ALPHA		0x5	/* Alphanumeric (3.9.4) */
#define FLEX_VECTOR_TYPE_SHORT_INSTR	0x7	/* type 111 (3.9.6) */

/* ===== Alpha Message Fragment Flags (Spec Section 3.8.8.3) ===== */

/* f0f1 = 11 indicates initial (and possibly only) fragment */
#define FLEX_ALPHA_FRAG_INITIAL		0x1800U

/* ===== Frame Buffer Size ===== */

/*
 * Total bytes for one encoded FLEX frame:
 *   ERS:   35 cycles * (2+4+2+4) bytes  = 420 bytes
 *   S1:    BS1(4) + A1(4) + B(2) + A1_inv(4) = 14 bytes
 *   FIW:   1 codeword                   =   4 bytes
 *   S2:    C block (25 ms at data rate) =   5-20 bytes
 *          1600: 5, 3200: 10, 6400: 20
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

/* ===== Modulation Type (ARIB STD-43A Table 3.2-2) ===== */

/* Distinguishes 2-FSK (A1, A2) from 4-FSK (A3, A4) at the same baud rate */
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
	int		sequence_num;		/* message numbering (-1 = disabled) */
	const char	*source_id;		/* source indication (NULL = none) */
	int		short_msg_idx;		/* short message index, -1 = N/A */
} flex_frame_msg_t;

/* Frame encoding parameters */
typedef struct flex_frame_params {
	uint32_t	cycle;			/* FIW cycle (0-14) */
	uint32_t	frame;			/* FIW frame (0-127) */
	uint32_t	roaming;		/* FIW roaming flag n */
	int		collapse;		/* BIW1 collapse value (0-7) */
	int		biw_time;		/* include BIW3/BIW4 time broadcast */
	uint32_t	local_id;		/* BIW2 local ID */
	uint32_t	coverage_id;		/* BIW2 coverage ID */
	int		timezone_offset;	/* BIW2 timezone (half-hours from UTC) */
	int		baud_rate;		/* 1600, 3200, or 6400 */
	int		modulation_type;	/* FLEX_MOD_2FSK or FLEX_MOD_4FSK */
	int		charset;		/* 0 = ASCII, 1 = KANJI */
} flex_frame_params_t;

/* ===== Phase Multiplexing (Spec Section 3.3) ===== */

/* Maximum phases: 2 at 3200 bps, 4 at 6400 bps */
#define FLEX_MAX_PHASES		4

/* Phase data for multi-phase frames.
 * Each phase carries an independent set of up to 88 data words. */
typedef struct flex_phase_data {
	uint32_t	words[FLEX_WORDS_PER_FRAME];	/* 88 words per phase */
	int		word_count;			/* actual words used */
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

/* Individual encoding functions (exposed for testing) */
uint32_t flex_encode_word(uint32_t dw);
uint32_t flex_word_checksum(uint32_t dw);
void flex_interleave_block(uint32_t block_num, uint32_t *frame_words);
uint32_t flex_create_fiw(uint32_t cycle, uint32_t frame, uint32_t n,
			 uint32_t r, uint32_t t);
uint32_t flex_create_biw1(uint32_t prio, uint32_t e_biw,
			  uint32_t s_vfield, uint32_t carry,
			  uint32_t collapse);
uint32_t flex_create_biw2(uint32_t local_id, uint32_t coverage_id,
			  uint32_t repeat, int timezone_offset);
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

/* Text utilities */
const char *flex_print_message(const char *message, int message_length);
int flex_scan_message(const char *message_input, int message_input_length,
		      char *message_output, int message_output_length);

#endif /* FLEX_FRAME_H */
