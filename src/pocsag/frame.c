/* POCSAG framing
 *
 * (C) 2021 by Andreas Eversberg <jolly@eversberg.eu>
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
 * IMPORTANT NOTE ON FUNCTION BITS VS MESSAGE TYPE:
 *
 * Per the original CCIR/ETSI POCSAG specification (CCIR Rec. 584), the
 * 2-bit function field in the address codeword is ONLY used to provide
 * four possible sub-addresses for a given pager. It has NO mandatory
 * semantic meaning related to message type.
 *
 * Message type (tone-only, numeric, alphanumeric) is determined by the
 * CONTENT of the message codewords, not by the function bits.
 *
 * Any correlation between function bits and message type is manufacturer/
 * pager specific (e.g., Motorola, Swissphone), not defined by the protocol.
 *
 * This implementation treats function bits and message type as INDEPENDENT:
 *   - Function bits (0-3 or A-D): Sub-address selection
 *   - Message type (tone/numeric/alpha): Content encoding
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "pocsag.h"
#include "frame.h"
#include <ctype.h>

#define CHAN pocsag->sender.kanal

#define PREAMBLE_COUNT		18
#define CODEWORD_PREAMBLE	0xaaaaaaaa
#define CODEWORD_SYNC		0x7cd215d8
#define CODEWORD_IDLE		0x7a89c197
#define IDLE_BATCHES		2

/* Initial and growth size for dynamic rx message buffers */
#define RX_BUF_INIT		256
#define RX_BUF_GROW		256

/*
 * Maximum message codewords per single message (safety limit).
 *
 * At 2400 baud this is ~7 minutes of continuous data — far beyond any
 * real-world POCSAG message. Prevents unbounded memory growth from
 * malformed signals or stuck decoders.
 *
 * 100000 alpha chars ÷ ~2.86 chars/codeword ≈ 35000 codewords.
 */
#define RX_MSG_MAX_CODEWORDS	35000

/*
 * Ensure rx buffer has room for at least one more element.
 * buf: pointer to buffer pointer (char** or uint8_t**)
 * len: current length
 * alloc: pointer to allocated size
 * elem_size: sizeof element
 * Returns 0 on success, -1 on alloc failure.
 */
static int rx_buf_ensure(void **buf, int len, int *alloc, int elem_size)
{
	if (len < *alloc)
		return 0;
	int new_alloc = *alloc ? *alloc + RX_BUF_GROW : RX_BUF_INIT;
	void *p = realloc(*buf, new_alloc * elem_size);
	if (!p)
		return -1;
	*buf = p;
	*alloc = new_alloc;
	return 0;
}

static const char numeric[16] = "0123456789RU -][";

static const char *ctrl_char[32] = {
	"<NUL>",
	"<SOH>",
	"<STX>",
	"<ETX>",
	"<EOT>",
	"<ENQ>",
	"<ACK>",
	"<BEL>",
	"<BS>",
	"<HT>",
	"<LF>",
	"<VT>",
	"<FF>",
	"<CR>",
	"<SO>",
	"<SI>",
	"<DLE>",
	"<DC1>",
	"<DC2",
	"<DC3>",
	"<DC4>",
	"<NAK>",
	"<SYN>",
	"<ETB>",
	"<CAN>",
	"<EM>",
	"<SUB>",
	"<ESC>",
	"<FS>",
	"<GS>",
	"<RS>",
	"<US>",
};

static const char *del_char = "<DEL>";

const char *print_message(const char *message, int message_length)
{
	static char message_print[1024];
	const char *c;
	int i, ii, clen;

	/* i is input counter, ii is output counter */
	for (i = 0, ii = 0; i < message_length; i++) {
		if (message[i] >= 0 && message[i] <= 31)
			c = ctrl_char[(int)message[i]];
		else if (message[i] == 127)
			c = del_char;
		else {
			message_print[ii++] = message[i];
			continue;
		}
		clen = strlen(c);
		if (ii + clen == sizeof(message_print))
			break;
		memcpy(message_print + ii, c, clen);
		ii += clen;
	}
	message_print[ii++] = '\0';

	return message_print;
}

int scan_message(const char *message_input, int message_input_length, char *message_output, int message_output_length)
{
	int i, ii, j, clen;

	/* i is input counter, ii is output counter */
	for (i = 0, ii = 0; i < message_input_length; ii++) {
		if (ii == message_output_length)
			break;
		if (message_input[i] == '<') {
			/* maybe a control character ? */
			for (j = 0; j < 32; j++) {
				clen = strlen(ctrl_char[j]);
				/* skip, if control sequence would not fit into the input buffer */
				if (clen <= message_input_length - i && !memcmp(message_input + i, ctrl_char[j], clen)) {
					/* found control sequence, so break the loop */
					break;
				}
			}
			if (j < 32) {
				/* if loop was not completed, use the found character */
				message_output[ii] = j;
				i += clen;
			} else {
				clen = strlen(del_char);
				/* skip, if control sequence would not fit into the input buffer */
				if (clen <= i - message_input_length && !memcmp(message_input + i, del_char, clen)) {
					/* found control sequence, copy DEL character */
					message_output[ii] = 127;
					i += clen;
				} else {
					/* found no control sequence, copy '<' character */
					message_output[ii] = '<';
					i++;
				}
			}
		} else {
			/* no control character */
			message_output[ii] = message_input[i];
			i++;
		}
	}

	return ii;
}

static uint32_t pocsag_crc(uint32_t word)
{
	uint32_t denominator = 0x76900000;
	int i;

	word <<= 10;

	for (i = 0; i < 21; i++) {
		if ((word >> (30 - i)) & 1)
			word ^= denominator;
		denominator >>= 1;
	}

	return word & 0x3ff;
}

static uint32_t pocsag_parity(uint32_t word)
{
	word ^= word >> 16;
	word ^= word >> 8;
	word ^= word >> 4;
	word ^= word >> 2;
	word ^= word >> 1;

	return word & 1;
}

/*
 * BCH(31,21) error correction for POCSAG codewords.
 *
 * A POCSAG codeword is 32 bits: [21 data] [10 CRC] [1 parity].
 * The BCH code operates on the upper 31 bits (data + CRC); the parity
 * bit covers all 31 bits.
 *
 * Generator polynomial: g(x) = x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1
 *                             = 0x769
 *
 * This implementation computes the syndrome, then brute-force searches for
 * 1-bit and 2-bit error patterns that zero the syndrome. This matches the
 * approach used by gr-pager (bch3121.cc) and multimon-ng.
 *
 * Returns:
 *   0  — no errors (or corrected)
 *  -1  — uncorrectable (>2 bit errors)
 *
 * On success, *word is corrected in place.
 */
/*
 * BCH(31,21) error correction for POCSAG codewords.
 *
 * Returns:
 *   0          - no errors (codeword was already valid)
 *   >0         - number of bits corrected (1 or 2)
 *   -1         - uncorrectable (3+ bit errors)
 *
 * On success, *word is corrected in place.
 * If corrections_out is non-NULL, the bitmask of flipped bit positions is stored there.
 */
int pocsag_bch_correct(uint32_t *word, uint32_t *corrections_out)
{
	uint32_t cw, syndrome, mask, coeff;
	int i, j;

	if (corrections_out)
		*corrections_out = 0;

	cw = *word >> 1;

	syndrome = cw;
	mask = 1u << 30;
	coeff = 0x769u << 20;

	for (i = 21; i > 0; i--, mask >>= 1, coeff >>= 1) {
		if (syndrome & mask)
			syndrome ^= coeff;
	}

	if (pocsag_parity(*word))
		syndrome |= (1u << 10);

	if (syndrome == 0)
		return 0; /* no errors */

	/* Try all single-bit flips */
	for (i = 0; i < 32; i++) {
		uint32_t flip = 1u << i;
		uint32_t trial = *word ^ flip;
		uint32_t s;

		cw = trial >> 1;
		s = cw;
		mask = 1u << 30;
		coeff = 0x769u << 20;
		for (j = 21; j > 0; j--, mask >>= 1, coeff >>= 1) {
			if (s & mask)
				s ^= coeff;
		}
		if (pocsag_parity(trial))
			s |= (1u << 10);

		if (s == 0) {
			*word = trial;
			if (corrections_out)
				*corrections_out = flip;
			return 1;
		}
	}

	/* Try all two-bit flip combinations */
	for (i = 0; i < 32; i++) {
		for (j = i + 1; j < 32; j++) {
			uint32_t flip = (1u << i) | (1u << j);
			uint32_t trial = *word ^ flip;
			uint32_t s;
			int k;

			cw = trial >> 1;
			s = cw;
			mask = 1u << 30;
			coeff = 0x769u << 20;
			for (k = 21; k > 0; k--, mask >>= 1, coeff >>= 1) {
				if (s & mask)
					s ^= coeff;
			}
			if (pocsag_parity(trial))
				s |= (1u << 10);

			if (s == 0) {
				*word = trial;
				if (corrections_out)
					*corrections_out = flip;
				return 2;
			}
		}
	}

	return -1; /* uncorrectable */
}

/*
 * Validate/correct a received codeword.
 * Returns: <0 uncorrectable, 0 clean, >0 number of bits corrected.
 * *corrections_out receives the bitmask of flipped bit positions (0 if clean).
 */
static int debug_word(uint32_t *word, int slot, uint32_t *corrections_out)
{
	int bch_rc;

	*corrections_out = 0;

	if (pocsag_crc(*word >> 11) != ((*word >> 1) & 0x3ff) || pocsag_parity(*word)) {
		bch_rc = pocsag_bch_correct(word, corrections_out);
		if (bch_rc < 0) {
			LOGP(DPOCSAG, LOGL_NOTICE, "Uncorrectable error in codeword 0x%08x.\n", *word);
			return -EINVAL;
		}
		LOGP(DPOCSAG, LOGL_INFO, "BCH corrected %d bit(s) in codeword -> 0x%08x (mask: 0x%08x).\n",
		     bch_rc, *word, *corrections_out);
	}

	if (*word == CODEWORD_SYNC) {
		LOGP(DPOCSAG, LOGL_DEBUG, "-> valid sync word\n");
		return 0;
	}

	if (*word == CODEWORD_IDLE) {
		LOGP(DPOCSAG, LOGL_DEBUG, "-> valid idle word\n");
		return 0;
	}

	if (!(*word & 0x80000000)) {
		LOGP(DPOCSAG, LOGL_DEBUG, "-> valid address word: RIC = '%d', function = '%d' (%s)\n",
		     ((*word >> 10) & 0x1ffff8) + slot, (*word >> 11) & 0x3,
		     pocsag_function_name[(*word >> 11) & 0x3]);
	} else {
		LOGP(DPOCSAG, LOGL_DEBUG, "-> valid message word: message = '0x%05x'\n",
		     (*word >> 11) & 0xfffff);
	}

	return *corrections_out ? __builtin_popcount(*corrections_out) : 0;
}

static uint32_t encode_address(pocsag_msg_t *msg)
{
	uint32_t word;

	/* compose message */
	word = 0x0;

	/* RIC */
	word = (word << 18) | (msg->ric >> 3);
	word = (word << 2) | msg->function;

	word = (word << 10) | pocsag_crc(word);
	word = (word << 1) | pocsag_parity(word);

	return word;
}

static void decode_address(uint32_t word, uint8_t slot, uint32_t *ric, enum pocsag_function *function)
{
	*ric = ((word >> 10) & 0x1ffff8) + slot;
	*function = (word >> 11) & 0x3;
}

static uint32_t encode_numeric(pocsag_msg_t *msg)
{
	uint8_t digit[5] = { 0xc, 0xc, 0xc, 0xc, 0xc };
	int index, i;
	uint32_t word;

	/* get characters from string */
	index = 0;
	while (msg->data_index < msg->data_length) {
		for (i = 0; i < 16; i++) {
			if (numeric[i] == msg->data[msg->data_index])
				break;
		}
		msg->data_index++;
		if (i < 16)
			digit[index++] = i;
		if (index == 5)
			break;
	}

	/* compose message */
	word = 0x1;
	for (i = 0; i < 5; i++) {
		word = (word << 1) | (digit[i] & 0x1);
		word = (word << 1) | ((digit[i] >> 1) & 0x1);
		word = (word << 1) | ((digit[i] >> 2) & 0x1);
		word = (word << 1) | ((digit[i] >> 3) & 0x1);
	}

	word = (word << 10) | pocsag_crc(word);
	word = (word << 1) | pocsag_parity(word);

	return word;
}

static void decode_numeric(pocsag_t *pocsag, uint32_t word, uint32_t corrections)
{
	uint8_t digit;
	int i, need;

	for (i = 0; i < 5; i++) {
		need = pocsag->rx_msg_data_length_numeric + 1;
		if (need > pocsag->rx_msg_numeric_alloc) {
			int new_alloc = pocsag->rx_msg_numeric_alloc ? pocsag->rx_msg_numeric_alloc + RX_BUF_GROW : RX_BUF_INIT;
			char *nd = realloc(pocsag->rx_msg_data_numeric, new_alloc);
			uint8_t *ns = realloc(pocsag->rx_msg_num_status, new_alloc);
			if (!nd || !ns)
				return;
			pocsag->rx_msg_data_numeric = nd;
			pocsag->rx_msg_num_status = ns;
			pocsag->rx_msg_numeric_alloc = new_alloc;
		}

		/* Grow raw nibble buffer in parallel */
		need = pocsag->rx_msg_numeric_nibble_count + 1;
		if (need > pocsag->rx_msg_nibble_alloc) {
			int new_alloc = pocsag->rx_msg_nibble_alloc ? pocsag->rx_msg_nibble_alloc + RX_BUF_GROW : RX_BUF_INIT;
			uint8_t *nn = realloc(pocsag->rx_msg_numeric_nibbles, new_alloc);
			if (!nn)
				return;
			pocsag->rx_msg_numeric_nibbles = nn;
			pocsag->rx_msg_nibble_alloc = new_alloc;
		}

		/*
		 * Check if any of the 4 source bits for this digit were affected.
		 * corrections == 0xfffff800 means uncorrectable (all data bits bad),
		 * otherwise individual set bits mean BCH-corrected positions.
		 */
		int bit0 = 30 - i * 4, bit1 = 29 - i * 4, bit2 = 28 - i * 4, bit3 = 27 - i * 4;
		int any_affected = ((corrections >> bit0) | (corrections >> bit1) |
				    (corrections >> bit2) | (corrections >> bit3)) & 1;
		/* Status: 0=ok, 1=corrected, 2=uncorrectable */
		uint8_t status = 0;
		if (any_affected)
			status = (corrections == 0xfffff800) ? 2 : 1;

		digit = (word >> bit3) & 0x1;
		digit = (digit << 1) | ((word >> bit2) & 0x1);
		digit = (digit << 1) | ((word >> bit1) & 0x1);
		digit = (digit << 1) | ((word >> bit0) & 0x1);
		pocsag->rx_msg_data_numeric[pocsag->rx_msg_data_length_numeric] = numeric[digit];
		pocsag->rx_msg_num_status[pocsag->rx_msg_data_length_numeric] = status;
		pocsag->rx_msg_data_length_numeric++;

		/* Store raw nibble for scoring heuristics */
		pocsag->rx_msg_numeric_nibbles[pocsag->rx_msg_numeric_nibble_count++] = digit;
	}
}

static uint32_t encode_alpha(pocsag_msg_t *msg)
{
	int bits;
	uint32_t word;

	/* compose message */
	word = 0x1;
	bits = 0;

	/* get character from string */
	while (msg->data_index < msg->data_length) {
		if ((msg->data[msg->data_index] & 0x80)) {
			msg->data_index++;
			continue;
		}
		while (42) {
			word = (word << 1) | ((msg->data[msg->data_index] >> msg->bit_index) & 1);
			bits++;
			if (++msg->bit_index == 7) {
				msg->bit_index = 0;
				msg->data_index++;
				break;
			}
			if (bits == 20)
				break;
		}
		if (bits == 20)
			break;
	}

	/*
	 * Pad remaining space with complete padding characters.
	 *
	 * Per POCSAG spec (AN142, CCIR Rec. 584): "The last codeword is filled
	 * with unprintable characters such as end of message, end of text, or
	 * null. Null is the only character which can be incomplete."
	 *
	 * The spec also states: "Alphanumeric messages should be padded with null."
	 *
	 * IMPORTANT: Padding characters must be encoded LSB-first, just like
	 * message characters. The LSb of each ASCII character is transmitted
	 * first, so we must bit-reverse the 7-bit padding value before inserting.
	 * For NULL (0x00), bit-reversal has no effect. For other characters like
	 * EOT (0x04), the bit order matters for correct decoding.
	 */
	if (bits <= 13) {
		/* Bit-reverse the 7-bit padding character (LSB-first encoding) */
		uint8_t pad = msg->padding & 0x7F;
		uint8_t reversed = 0;
		reversed |= ((pad >> 0) & 1) << 6;
		reversed |= ((pad >> 1) & 1) << 5;
		reversed |= ((pad >> 2) & 1) << 4;
		reversed |= ((pad >> 3) & 1) << 3;
		reversed |= ((pad >> 4) & 1) << 2;
		reversed |= ((pad >> 5) & 1) << 1;
		reversed |= ((pad >> 6) & 1) << 0;
		do {
			word = (word << 7) | reversed;
			bits += 7;
		} while (bits <= 13);
	}

	/*
	 * Fill remaining bits with zeros (incomplete NULL character).
	 * Per spec: "Null is the only character which can be incomplete."
	 */
	if (bits < 20)
		word <<= 20 - bits;

	word = (word << 10) | pocsag_crc(word);
	word = (word << 1) | pocsag_parity(word);

	return word;
}

static void decode_alpha(pocsag_t *pocsag, uint32_t word, uint32_t corrections)
{
	int i;
	int need;

	for (i = 0; i < 20; i++) {
		int bit_pos = 30 - i;
		int bit_corrected = (corrections >> bit_pos) & 1;

		need = pocsag->rx_msg_data_length + 1;
		if (need > pocsag->rx_msg_data_alloc) {
			int new_alloc = pocsag->rx_msg_data_alloc ? pocsag->rx_msg_data_alloc + RX_BUF_GROW : RX_BUF_INIT;
			char *nd = realloc(pocsag->rx_msg_data, new_alloc);
			uint8_t *ns = realloc(pocsag->rx_msg_char_status, new_alloc);
			if (!nd || !ns)
				return;
			pocsag->rx_msg_data = nd;
			pocsag->rx_msg_char_status = ns;
			pocsag->rx_msg_data_alloc = new_alloc;
		}

		if (!pocsag->rx_msg_bit_index) {
			pocsag->rx_msg_data[pocsag->rx_msg_data_length] = 0x00;
			pocsag->rx_msg_cur_char_bad = 0;
		}
		pocsag->rx_msg_data[pocsag->rx_msg_data_length] >>= 1;
		pocsag->rx_msg_data[pocsag->rx_msg_data_length] |= ((word >> bit_pos) & 0x1) << 6;
		/*
		 * Track per-char status: 0=ok, 1=corrected, 2=uncorrectable.
		 * corrections == 0xfffff800 means the entire codeword was
		 * uncorrectable (all data bits flagged as bad).
		 */
		if (bit_corrected) {
			if (corrections == 0xfffff800)
				pocsag->rx_msg_cur_char_bad = 2;
			else if (pocsag->rx_msg_cur_char_bad < 1)
				pocsag->rx_msg_cur_char_bad = 1;
		}
		if (++pocsag->rx_msg_bit_index == 7) {
			pocsag->rx_msg_char_status[pocsag->rx_msg_data_length] = pocsag->rx_msg_cur_char_bad;
			pocsag->rx_msg_bit_index = 0;
			pocsag->rx_msg_data_length++;
		}
	}
}

/*
 * Decode Skyper (ROT-1 Caesar cipher on alpha).
 * Skyper rubric names and content use ROT-1: each 7-bit character is
 * shifted by +1 during encoding, so we subtract 1 to decode.
 * This runs in parallel with normal alpha decode.
 */
static void decode_skyper(pocsag_t *pocsag)
{
	int i;

	/* Derive Skyper text from the already-decoded alpha buffer */
	pocsag->rx_msg_data_length_skyper = pocsag->rx_msg_data_length;

	/* Ensure skyper buffer is large enough */
	if (pocsag->rx_msg_data_length > 0) {
		void *p = realloc(pocsag->rx_msg_data_skyper, pocsag->rx_msg_data_length);
		if (!p)
			return;
		pocsag->rx_msg_data_skyper = p;
	}

	for (i = 0; i < pocsag->rx_msg_data_length; i++) {
		unsigned char c = (unsigned char)pocsag->rx_msg_data[i];
		/* ROT-1 decode: subtract 1, wrap 0x00 -> 0x7F within 7-bit range */
		if (c == 0)
			pocsag->rx_msg_data_skyper[i] = 0x7F;
		else
			pocsag->rx_msg_data_skyper[i] = c - 1;
	}
}

/*
 * Store raw message bits for base64 output.
 * Packs the 20 data bits from each message codeword into a byte buffer.
 */
static void store_raw_bits(pocsag_t *pocsag, uint32_t word)
{
	int i;

	for (i = 0; i < 20; i++) {
		int byte_idx = pocsag->rx_msg_data_raw_bits / 8;
		int bit_idx = 7 - (pocsag->rx_msg_data_raw_bits % 8); /* MSB first */

		if (rx_buf_ensure((void **)&pocsag->rx_msg_data_raw, byte_idx + 1,
				  &pocsag->rx_msg_raw_alloc, sizeof(uint8_t)) < 0)
			return;

		if (pocsag->rx_msg_data_raw_bits % 8 == 0)
			pocsag->rx_msg_data_raw[byte_idx] = 0;

		if ((word >> (30 - i)) & 1)
			pocsag->rx_msg_data_raw[byte_idx] |= (1u << bit_idx);

		pocsag->rx_msg_data_raw_bits++;
	}
}

/*
 * Heuristic scoring for message type detection.
 *
 * Combines approaches from multimon-ng and PDW:
 *   - multimon-ng: per-character class scoring (printable, special, control)
 *   - PDW: penalizes "bad" numeric chars (U, [, ], *) with escalating weights
 *
 * Returns a score where higher = more likely to be this type.
 * Negative scores indicate the content is unlikely to be this type.
 */
/*
 * Count trailing fill characters in the alpha decode buffer.
 * POCSAG alpha messages are padded with NULL (0x00) or space (0x20)
 * depending on the transmitter implementation.
 *
 * NULL padding: per POCSAG spec (AN142, CCIR Rec. 584), the standard
 * fill character. Coincidental NULL is rare (1/128 per 7-bit slot).
 *
 * Space padding: some transmitters pad with spaces instead of NULL.
 * Space (0x20 = 0100000) is also unlikely to appear coincidentally
 * when alpha bits are misinterpreted as numeric nibbles.
 *
 * Both are counted as fill, but only a contiguous run of the same
 * character from the end is counted (no mixing).
 */
static int count_alpha_fill(const char *data, int len)
{
	int fill = 0;
	int i;
	char fill_char;

	if (len == 0)
		return 0;

	/* Determine which fill character is used (check last char) */
	fill_char = data[len - 1];
	if (fill_char != '\0' && fill_char != ' ')
		return 0;

	for (i = len - 1; i >= 0; i--) {
		if (data[i] == fill_char)
			fill++;
		else
			break;
	}
	return fill;
}

/*
 * Count trailing fill nibbles in the numeric nibble buffer.
 * POCSAG numeric messages are padded with 0xC (space) nibbles.
 * Fill is a structural signal but weaker than alpha fill since
 * coincidental 0xC nibbles are more likely (1/16 per nibble vs 1/128).
 */
static int count_numeric_fill(const uint8_t *nibbles, int count)
{
	int fill = 0;
	int i;

	for (i = count - 1; i >= 0; i--) {
		if (nibbles[i] == 0x0C)
			fill++;
		else
			break;
	}
	return fill;
}

/*
 * Score the plausibility of an alpha interpretation.
 *
 * Modeled after the GSC/Golay alpha scorer: evaluates each character
 * against expected alpha content patterns, then adds a quadratic fill
 * bonus for trailing NULL padding characters.
 *
 * Scoring per character:
 *   +3  letters (A-Z, a-z), digits (0-9), or space
 *   -2  other printable ASCII (punctuation, special chars)
 *   -5  non-printable control characters
 *    0  common whitespace (newline, tab) and EOT/NULL padding
 *
 * Fill bonus: fill^2 * 2 (quadratic, strong — coincidental NULL is
 * very rare at 1/128 per 7-bit slot).
 */
static int score_alpha(const char *data, int len, int fill)
{
	int score = 0;
	int content_chars = 0;
	int i;

	for (i = 0; i < len; i++) {
		unsigned char c = (unsigned char)data[i];

		/* Exclude trailing fill characters (NULL or space padding).
		 * Fill chars are scored via the fill bonus below, not here. */
		if (i >= len - fill && (c == 0x00 || c == ' '))
			continue;

		/* Also skip embedded NULLs (incomplete chars at boundaries) */
		if (c == 0x00)
			continue;

		content_chars++;

		if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
		    (c >= '0' && c <= '9') || c == ' ') {
			score += 3; /* alphanumeric or space — strong alpha signal */
		} else if (c >= 0x20 && c <= 0x7E) {
			score -= 2; /* other printable ASCII — mild penalty */
		} else if (c == '\n' || c == '\r' || c == '\t') {
			score += 0; /* common whitespace — neutral */
		} else if (c == 0x04) {
			score += 0; /* EOT padding — neutral */
		} else {
			score -= 5; /* non-printable control chars — strong penalty */
		}
	}

	/* Fill bonus only applies when there is actual content before the
	 * fill. All-fill with no content (e.g. numeric 0000 producing
	 * all-zero bits that look like NULL) is not evidence of alpha. */
	if (content_chars > 0) {
		/* Fill characters are strong structural evidence: the encoder
		 * pads unused 7-bit slots with NULL or space. Coincidental fill
		 * in the wrong interpretation is very rare (1/128 per 7-bit slot).
		 *
		 * POCSAG-specific: numeric gets 5 nibbles per codeword vs ~2.85
		 * alpha chars. Short alpha messages are at a scoring disadvantage
		 * because the same bits produce more numeric nibbles (which may
		 * coincidentally look like digits). Alpha fill is therefore
		 * weighted more heavily to compensate. */
		score += fill * fill * 3 + fill * 5;
	}

	return score;
}

/*
 * Score the plausibility of a numeric interpretation.
 *
 * Modeled after the GSC/Golay numeric scorer: operates on raw 4-bit
 * nibbles (not decoded ASCII characters) to distinguish digits from
 * artifacts. Adds a quadratic fill bonus for trailing 0xC (space) padding.
 *
 * Scoring per nibble:
 *   +3  digit (0x0–0x9)
 *   -1  'U' (0xB) when it's the only U and at position 0
 *       (urgent-prefix convention: "U" + phone number)
 *  -15  'U' (0xB) otherwise — very rare in real numeric messages;
 *       common artifact when alpha data is misinterpreted as numeric
 *   -2  space (0xC), hyphen (0xD), brackets (0xE, 0xF)
 *   -3  'R' (0xA spare) — uncommon in real numeric messages
 *
 * Fill bonus: fill^2 (quadratic, weaker than alpha fill since
 * coincidental 0xC nibbles are more likely: 1/16 vs 1/128).
 *
 * Normalization: scores are scaled to per-character rate then multiplied
 * by a reference length to make alpha and numeric scores comparable
 * despite their different bit densities (4-bit nibbles vs 7-bit chars).
 * Without this, numeric always gets ~1.75x more scoring opportunities
 * per codeword, biasing short messages toward numeric.
 */
static int score_numeric_nibbles(const uint8_t *nibbles, int count, int fill)
{
	int raw_score = 0;
	int scored_count = 0;
	int digit_count = 0;
	int i;
	int u_count = 0;
	int first_u = -1;
	int r_count = 0;

	/* Pre-scan for 'U' (0xB) and 'R' (0xA) nibbles */
	for (i = 0; i < count; i++) {
		/* Skip fill */
		if (nibbles[i] == 0x0C && i >= count - fill)
			continue;
		if (nibbles[i] == 0x0B) {
			u_count++;
			if (first_u < 0)
				first_u = i;
		}
		if (nibbles[i] == 0x0A)
			r_count++;
	}

	/*
	 * Urgent-prefix heuristic: a single 'U' as the first non-fill nibble
	 * followed by phone-number digits is a legitimate paging convention
	 * (urgent message). Only reduce the penalty when:
	 *   - exactly one 'U' in the entire message
	 *   - it's the first nibble (position 0)
	 */
	int urgent_prefix = (u_count == 1 && first_u == 0);

	for (i = 0; i < count; i++) {
		uint8_t n = nibbles[i];

		/* Exclude fill nibbles (0xC = space padding) */
		if (n == 0x0C && i >= count - fill)
			continue;

		scored_count++;

		if (n <= 0x09) {
			raw_score += 3; /* digit (0-9) — strong numeric signal */
			digit_count++;
		} else if (n == 0x0B) {
			if (urgent_prefix)
				raw_score -= 1; /* leading U = urgent prefix — mild penalty */
			else
				raw_score -= 15; /* stray U — strong artifact signal */
		} else if (n == 0x0A) {
			/* 'R' (return/received): at most one R at the start is
			 * plausible in a real numeric message. Multiple R's are
			 * a strong misdetection signal. */
			if (r_count > 1)
				raw_score -= 15; /* multiple R — strong artifact */
			else
				raw_score -= 5;  /* single R — uncommon but possible */
		} else if (n >= 0x0C && n <= 0x0E) {
			raw_score -= 2; /* space, hyphen, bracket — mild penalty */
		} else if (n == 0x0F) {
			raw_score -= 2; /* ']' — mild penalty */
		}
	}

	/* Normalize: numeric gets 5 nibbles per codeword vs ~2.85 alpha chars.
	 * Scale the raw score by (7/4) ≈ the bit-width ratio, implemented as
	 * score * 4 / 7 to avoid floating point. This makes per-codeword
	 * contribution comparable between the two interpretations. */
	int score;
	if (scored_count > 0)
		score = raw_score * 4 / 7;
	else
		score = 0;

	/* E.164 length penalty: real phone numbers have at most 15 digits
	 * (not counting separators, brackets, U, R, spaces). Numeric
	 * messages with more actual digits than this are almost certainly
	 * alpha data misinterpreted as numeric. Apply a progressive penalty
	 * for each digit beyond 15. */
	if (digit_count > 15)
		score -= (digit_count - 15) * 5;

	/* Bracket structure validation: in real numeric messages, brackets
	 * are used for area codes like [212] — always '[' (0xE) followed
	 * by ']' (0xF) with digits inside. Malformed patterns are strong
	 * evidence of alpha data misinterpreted as numeric:
	 *   - ']' before '[' (close before open)
	 *   - unmatched '[' or ']' (hanging brackets)
	 *   - empty brackets '[]' (no content inside)
	 *   - nested brackets '[[' or ']]'
	 * Each violation gets a heavy penalty. */
	{
		int bracket_depth = 0;
		int bracket_violations = 0;
		int bracket_content = 0; /* digits inside current bracket pair */

		for (i = 0; i < count; i++) {
			uint8_t n = nibbles[i];

			/* Skip fill */
			if (n == 0x0C && i >= count - fill)
				continue;

			if (n == 0x0F) { /* '[' (index 15 in BCD table) */
				if (bracket_depth > 0)
					bracket_violations++; /* nested open */
				bracket_depth++;
				bracket_content = 0;
			} else if (n == 0x0E) { /* ']' (index 14 in BCD table) */
				if (bracket_depth <= 0)
					bracket_violations++; /* close before open */
				else if (bracket_content == 0)
					bracket_violations++; /* empty brackets */
				bracket_depth--;
			} else if (bracket_depth > 0 && n <= 0x09) {
				bracket_content++;
			}
		}
		/* Unmatched open brackets */
		if (bracket_depth > 0)
			bracket_violations += bracket_depth;

		score -= bracket_violations * 10;
	}

	/* Consecutive non-digit penalty: real phone numbers never have two
	 * separators in a row (e.g. "- ", "  ", "][", "R ", "U-").  Each
	 * pair of adjacent non-digit, non-fill nibbles is penalized as a
	 * strong misdetection signal. */
	{
		int prev_is_nondigit = 0;

		for (i = 0; i < count; i++) {
			uint8_t n = nibbles[i];

			/* Skip fill */
			if (n == 0x0C && i >= count - fill)
				continue;

			if (n > 0x09) {
				/* Non-digit nibble */
				if (prev_is_nondigit)
					score -= 5;
				prev_is_nondigit = 1;
			} else {
				prev_is_nondigit = 0;
			}
		}
	}

	/* Fill nibbles (0xC = space) are evidence this is the correct type,
	 * but weaker than alpha fill since coincidental 0xC nibbles are more
	 * likely (1/16 per nibble vs 1/128 for alpha fill). Bonus scales
	 * quadratically. */
	score += fill * fill;

	return score;
}

/*
 * Check if a decoded string contains any printable content worth displaying.
 * Returns 1 if at least one printable non-padding character exists.
 */
static int is_printable(const char *data, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		unsigned char c = (unsigned char)data[i];
		if (c >= 32 && c <= 126)
			return 1;
	}
	return 0;
}

/*
 * Minimal base64 encoder for raw message data output.
 */
static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const uint8_t *in, int in_len, char *out, int out_size)
{
	int i, o = 0;

	for (i = 0; i < in_len; ) {
		int remaining = in_len - i;
		uint32_t a = in[i++];
		uint32_t b = (remaining > 1) ? in[i++] : 0;
		uint32_t c = (remaining > 2) ? in[i++] : 0;
		uint32_t triple = (a << 16) | (b << 8) | c;

		if (o + 4 >= out_size)
			break;
		out[o++] = b64[(triple >> 18) & 0x3F];
		out[o++] = b64[(triple >> 12) & 0x3F];
		out[o++] = (remaining > 1) ? b64[(triple >> 6) & 0x3F] : '=';
		out[o++] = (remaining > 2) ? b64[triple & 0x3F] : '=';
	}
	if (o < out_size)
		out[o] = '\0';
	return o;
}

/*
 * Uncertainty threshold for POCSAG dual-decode: when the absolute
 * difference between alpha and numeric content scores is below this
 * value, the result is flagged as uncertain in the log output.
 */
#define POCSAG_GUESS_UNCERTAIN_THRESHOLD 10

/*
 * Alpha prior bias: alpha messages are significantly more common than
 * numeric in modern POCSAG usage. This flat bonus tips ambiguous cases
 * toward alpha without affecting clear numeric messages (which typically
 * win by 15+ points). A numeric message needs to outscore alpha by at
 * least this margin to be selected.
 */
#define POCSAG_ALPHA_PRIOR_BIAS 2

static enum pocsag_msg_type detect_msg_type(pocsag_t *pocsag)
{
	int alpha_fill, numeric_fill;
	int sa, sn;

	if (pocsag->rx_msg_data_length == 0)
		return POCSAG_MSG_TYPE_TONE;

	/* Count trailing fill in both interpretations */
	alpha_fill = count_alpha_fill(pocsag->rx_msg_data, pocsag->rx_msg_data_length);
	numeric_fill = count_numeric_fill(pocsag->rx_msg_numeric_nibbles,
					  pocsag->rx_msg_numeric_nibble_count);

	/* Unified scoring: content + fill bonus folded into a single score,
	 * matching the GSC/Golay discriminator approach. */
	sa = score_alpha(pocsag->rx_msg_data, pocsag->rx_msg_data_length, alpha_fill);
	sn = score_numeric_nibbles(pocsag->rx_msg_numeric_nibbles,
				   pocsag->rx_msg_numeric_nibble_count, numeric_fill);

	/* Apply alpha prior bias */
	sa += POCSAG_ALPHA_PRIOR_BIAS;

	/* Short-message alpha boost: messages with 1-2 data codewords
	 * (≤10 nibbles / ≤5 alpha chars) are where the heuristic struggles
	 * most due to the 4-bit vs 7-bit density asymmetry. Short alpha
	 * messages (OK, #1, GO) are common; short numeric that isn't a
	 * clean digit string is rare. Add extra alpha lean for short
	 * messages only — this doesn't affect longer messages where
	 * numeric phone numbers are more plausible.
	 * rx_msg_codewords includes the address codeword, so ≤3 means
	 * 1-2 message codewords. */
	if (pocsag->rx_msg_codewords <= 3)
		sa += 3;

	/* Long-message alpha override: real numeric messages are phone
	 * numbers — at most ~6 data codewords (15 digits + separators +
	 * fill). Messages with 7+ data codewords (rx_msg_codewords >= 8
	 * including address) cannot be numeric. Force alpha regardless
	 * of scores. */
	if (pocsag->rx_msg_codewords >= 8) {
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG,
			  "Type heuristic: %d codewords — too long for numeric, forcing alpha.\n",
			  pocsag->rx_msg_codewords);
		return POCSAG_MSG_TYPE_ALPHA;
	}

	/* Log scoring details for diagnostics */
	{
		int score_diff = sa - sn;
		if (score_diff < 0) score_diff = -score_diff;
		int uncertain = (score_diff < POCSAG_GUESS_UNCERTAIN_THRESHOLD) ? 1 : 0;
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG,
			  "Type heuristic: alpha_score=%d (fill=%d, +%d bias) numeric_score=%d (fill=%d) diff=%d%s\n",
			  sa, alpha_fill, POCSAG_ALPHA_PRIOR_BIAS, sn, numeric_fill,
			  sa - sn, uncertain ? " [uncertain]" : "");
	}

	if (sn > sa)
		return POCSAG_MSG_TYPE_NUMERIC;

	return POCSAG_MSG_TYPE_ALPHA;
}


/* get codeword from scheduler */
int64_t get_codeword(pocsag_t *pocsag)
{
	pocsag_msg_t *msg;
	uint32_t word = 0; // make GCC happy
	uint8_t slot = (pocsag->word_count - 1) >> 1;
	uint8_t subslot = (pocsag->word_count - 1) & 1;

	/* no codeword, if not transmitting */
	if (!pocsag->tx)
		return -1;

	/* transmitter state */
	switch (pocsag->state) {
	case POCSAG_IDLE:
		return -1;
	case POCSAG_PREAMBLE:
		if (!pocsag->word_count)
			LOGP_CHAN(DPOCSAG, LOGL_INFO, "Sending preamble.\n");
		/* transmit preamble */
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Sending 32 bits of preamble pattern 0x%08x.\n", CODEWORD_PREAMBLE);
		if (++pocsag->word_count == PREAMBLE_COUNT) {
			pocsag_new_state(pocsag, POCSAG_MESSAGE);
			pocsag->word_count = 0; 
			pocsag->idle_count = 0;
		}
		word =  CODEWORD_PREAMBLE;
		break;
	case POCSAG_MESSAGE:
		if (!pocsag->word_count)
			LOGP_CHAN(DPOCSAG, LOGL_INFO, "Sending batch.\n");
		/* send sync */
		if (pocsag->word_count == 0) {
			LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Sending 32 bits of sync pattern 0x%08x.\n", CODEWORD_SYNC);
			/* count codewords */
			++pocsag->word_count;
			word = CODEWORD_SYNC;
			break;
		}
		/* send message data, if there is an ongoing message */
		if ((msg = pocsag->current_msg)) {
			/* reset idle counter */
			pocsag->idle_count = 0;
			/*
			 * Encode data based on msg_type, NOT function bits.
			 * Per POCSAG standard, message encoding is independent
			 * of the function (sub-address) field.
			 */
			switch (msg->msg_type) {
			case POCSAG_MSG_TYPE_NUMERIC:
				word = encode_numeric(msg);
				break;
			case POCSAG_MSG_TYPE_ALPHA:
				word = encode_alpha(msg);
				break;
			case POCSAG_MSG_TYPE_TONE:
			case POCSAG_MSG_TYPE_AUTO:
			default:
				word = CODEWORD_IDLE; /* tone-only: no message codewords */
			}
			/* if message is complete, reset index and handle retransmission */
			if (msg->data_index == msg->data_length) {
				pocsag->current_msg = NULL;
				if (msg->retransmit_count < msg->retransmit_max) {
					/* Re-enqueue for retransmission with delay */
					struct timeval tv;
					msg->retransmit_count++;
					msg->data_index = 0;
					msg->bit_index = 0;
					gettimeofday(&tv, NULL);
					msg->next_send_time = (double)tv.tv_sec + tv.tv_usec / 1e6 + msg->retransmit_interval;
					LOGP_CHAN(DPOCSAG, LOGL_INFO, "Retransmission %d/%d for RIC %d scheduled in %.0fs.\n",
						  msg->retransmit_count, msg->retransmit_max, msg->ric, msg->retransmit_interval);
					/* msg stays in msg_list, will be picked up when eligible */
				} else {
					msg->data_index = 0;
					pocsag_msg_destroy(msg);
				}
				pocsag_msg_done(pocsag);
			}
			/* prevent 'use-after-free' from this point on */
			msg = NULL;
			LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Sending 32 bits of message codeword 0x%08x (frame %d.%d).\n", word, slot, subslot);
			/* count codewords */
			if (++pocsag->word_count == 17)
				pocsag->word_count = 0;
			break;
		}
		/* if we are about to send an address codeword, we search for a pending message */
		for (msg = pocsag->msg_list; msg; msg = msg->next) {
			/* if a message matches the right time slot */
			if ((msg->ric & 7) == slot) {
				/* skip if waiting for retransmit delay */
				if (msg->next_send_time > 0.0) {
					struct timeval tv;
					double now;
					gettimeofday(&tv, NULL);
					now = (double)tv.tv_sec + tv.tv_usec / 1e6;
					if (now < msg->next_send_time)
						continue;
				}
				break;
			}
		}
		if (msg) {
			/*
			 * Log message with both function (sub-address) and msg_type (encoding).
			 * Per POCSAG standard, these are independent.
			 */
			LOGP_CHAN(DPOCSAG, LOGL_INFO, "Sending message to RIC '%d' / function '%s' / type '%s'\n",
				  msg->ric, pocsag_function_name[msg->function], pocsag_msg_type_name(msg->msg_type));
			/* reset idle counter */
			pocsag->idle_count = 0;
			/* encode address */
			word = encode_address(msg);
			/*
			 * Link message if there is data to be sent.
			 * Decision is based on msg_type, NOT function bits.
			 * Per POCSAG standard, message type is independent of sub-address.
			 */
			if (msg->msg_type == POCSAG_MSG_TYPE_NUMERIC || msg->msg_type == POCSAG_MSG_TYPE_ALPHA) {
				LOGP_CHAN(DPOCSAG, LOGL_INFO, " -> Message text is \"%s\".\n", print_message(msg->data, msg->data_length));
				pocsag->current_msg = msg;
				msg->data_index = 0;
				msg->bit_index = 0;
			} else {
				/* tone-only: handle retransmission or remove */
				if (msg->retransmit_count < msg->retransmit_max) {
					struct timeval tv;
					msg->retransmit_count++;
					gettimeofday(&tv, NULL);
					msg->next_send_time = (double)tv.tv_sec + tv.tv_usec / 1e6 + msg->retransmit_interval;
					LOGP_CHAN(DPOCSAG, LOGL_INFO, "Tone retransmission %d/%d for RIC %d scheduled in %.0fs.\n",
						  msg->retransmit_count, msg->retransmit_max, msg->ric, msg->retransmit_interval);
				} else {
					pocsag_msg_destroy(msg);
				}
				pocsag_msg_done(pocsag);
				/* prevent 'use-after-free' from this point on */
				msg = NULL;
			}
			LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Sending 32 bits of address codeword 0x%08x (frame %d.%d).\n", word, slot, subslot);
			/* count codewords */
			if (++pocsag->word_count == 17)
				pocsag->word_count = 0;
			break;
		}
		/* no message, so we send idle pattern */
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Sending 32 bits of idle pattern 0x%08x (frame %d.%d).\n", CODEWORD_IDLE, slot, subslot);
		/* count codewords */
		if (++pocsag->word_count == 17) {
			pocsag->word_count = 0;
			/*
			 * Go idle if no message is ready to send.
			 * Messages waiting for retransmit delay are in msg_list
			 * but not eligible yet — don't keep transmitting idle
			 * batches while waiting. The transmitter will restart
			 * with preamble when a message becomes eligible.
			 */
			int any_ready = 0;
			{
				pocsag_msg_t *m;
				struct timeval tv;
				double now;
				gettimeofday(&tv, NULL);
				now = (double)tv.tv_sec + tv.tv_usec / 1e6;
				for (m = pocsag->msg_list; m; m = m->next) {
					if (m->next_send_time <= 0.0 || now >= m->next_send_time) {
						any_ready = 1;
						break;
					}
				}
			}
			if (!any_ready && pocsag->idle_count++ == IDLE_BATCHES) {
				LOGP_CHAN(DPOCSAG, LOGL_INFO, "Transmission done.\n");
				LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Reached %d of idle batches, turning transmitter off.\n", IDLE_BATCHES);
				pocsag_new_state(pocsag, POCSAG_IDLE);
			}
		}
		word = CODEWORD_IDLE;
		break;
	}

	if (word != CODEWORD_PREAMBLE) {
		uint32_t dummy;
		debug_word(&word, slot, &dummy);
	}

	return word;
}

static void done_rx_msg(pocsag_t *pocsag)
{
	const char *text;
	int sa, sn, ss;
	int raw_bytes;

	if (!pocsag->rx_msg_valid)
		return;

	pocsag->rx_msg_valid = 0;

	/* Compute sliding BER */
	double ber = 0.0;
	{
		int total_errors = 0, total_bits = 0, i;
		for (i = 0; i < pocsag->rx_ber.count; i++) {
			total_errors += pocsag->rx_ber.errors[i];
			total_bits += pocsag->rx_ber.total[i];
		}
		if (total_bits > 0)
			ber = (double)total_errors / (double)total_bits;
	}

	/* Detect message type based on content scoring */
	pocsag->rx_msg_type = detect_msg_type(pocsag);

	/* Log message header with correction stats and BER */
	LOGP_CHAN(DPOCSAG, LOGL_INFO, "Received message from RIC '%d' / function '%s' / type '%s'"
		  " / %d baud / %s polarity"
		  " (%d codewords, %d corrected, %d uncorrectable, BER %.1e)\n",
		  pocsag->rx_msg_ric, pocsag_function_name[pocsag->rx_msg_function],
		  pocsag_msg_type_name(pocsag->rx_msg_type),
		  pocsag->rx_baud_locked,
		  (pocsag->rx_polarity_locked < 0) ? "normal" : "inverted",
		  pocsag->rx_msg_codewords, pocsag->rx_msg_corrected,
		  pocsag->rx_msg_uncorrectable, ber);

	if (pocsag->rx_msg_type == POCSAG_MSG_TYPE_TONE) {
		pocsag_msg_receive(pocsag->language, pocsag->sender.kanal, pocsag->rx_msg_ric,
				   pocsag->rx_msg_function, pocsag->rx_msg_type,
				   pocsag->rx_baud_locked, pocsag->rx_polarity_locked, NULL);
		return;
	}

	/* Derive Skyper (ROT-1) from alpha buffer */
	decode_skyper(pocsag);

	/* Score all candidates (with fill detection) */
	int alpha_fill = count_alpha_fill(pocsag->rx_msg_data, pocsag->rx_msg_data_length);
	int numeric_fill = count_numeric_fill(pocsag->rx_msg_numeric_nibbles,
					      pocsag->rx_msg_numeric_nibble_count);
	sa = score_alpha(pocsag->rx_msg_data, pocsag->rx_msg_data_length, alpha_fill);
	sn = score_numeric_nibbles(pocsag->rx_msg_numeric_nibbles,
				   pocsag->rx_msg_numeric_nibble_count, numeric_fill);
	ss = score_alpha(pocsag->rx_msg_data_skyper, pocsag->rx_msg_data_length_skyper, 0);

	int alpha_printable = is_printable(pocsag->rx_msg_data, pocsag->rx_msg_data_length);
	int numeric_printable = is_printable(pocsag->rx_msg_data_numeric, pocsag->rx_msg_data_length_numeric);
	int skyper_printable = is_printable(pocsag->rx_msg_data_skyper, pocsag->rx_msg_data_length_skyper);

	/* Build an ordered list of candidates by score */
	struct {
		const char *label;
		const char *data;
		const uint8_t *status; /* per-char correction status, or NULL */
		int len;
		int score;
	} cand[3];
	int ncand = 0;

	if (alpha_printable) {
		cand[ncand].label = "Alpha";
		cand[ncand].data = pocsag->rx_msg_data;
		cand[ncand].status = pocsag->rx_msg_char_status;
		cand[ncand].len = pocsag->rx_msg_data_length;
		cand[ncand].score = sa;
		ncand++;
	}
	if (numeric_printable) {
		cand[ncand].label = "Numeric";
		cand[ncand].data = pocsag->rx_msg_data_numeric;
		cand[ncand].status = pocsag->rx_msg_num_status;
		cand[ncand].len = pocsag->rx_msg_data_length_numeric;
		cand[ncand].score = sn;
		ncand++;
	}
	if (skyper_printable) {
		cand[ncand].label = "NEC-Skyper";
		cand[ncand].data = pocsag->rx_msg_data_skyper;
		cand[ncand].status = pocsag->rx_msg_char_status; /* same source bits as alpha */
		cand[ncand].len = pocsag->rx_msg_data_length_skyper;
		cand[ncand].score = ss;
		ncand++;
	}

	/* Simple insertion sort by score descending */
	{
		int i, j;
		for (i = 1; i < ncand; i++) {
			typeof(cand[0]) tmp = cand[i];
			j = i - 1;
			while (j >= 0 && cand[j].score < tmp.score) {
				cand[j + 1] = cand[j];
				j--;
			}
			cand[j + 1] = tmp;
		}
	}

	/* Log candidates in score order, marking corrected/bad characters */
	{
		int i;
		for (i = 0; i < ncand; i++) {
			text = print_message(cand[i].data, cand[i].len);
			/* Count corrected and bad characters */
			int n_corrected = 0, n_bad = 0, k;
			if (cand[i].status) {
				for (k = 0; k < cand[i].len; k++) {
					if (cand[i].status[k] == 1) n_corrected++;
					else if (cand[i].status[k] >= 2) n_bad++;
				}
			}
			if (n_corrected || n_bad)
				LOGP_CHAN(DPOCSAG, LOGL_INFO, " -> %s (score %d, %d corrected, %d bad): \"%s\"\n",
					  cand[i].label, cand[i].score, n_corrected, n_bad, text);
			else
				LOGP_CHAN(DPOCSAG, LOGL_INFO, " -> %s (score %d): \"%s\"\n",
					  cand[i].label, cand[i].score, text);
		}
	}

	/* Always log base64-encoded raw message bits last */
	raw_bytes = (pocsag->rx_msg_data_raw_bits + 7) / 8;
	if (raw_bytes > 0) {
		int b64_size = ((raw_bytes + 2) / 3) * 4 + 1;
		char *b64buf = malloc(b64_size);
		if (b64buf) {
			base64_encode(pocsag->rx_msg_data_raw, raw_bytes, b64buf, b64_size);
			LOGP_CHAN(DPOCSAG, LOGL_INFO, " -> Raw: %s\n", b64buf);
			free(b64buf);
		}
	}

	/* Send the highest-scoring printable candidate to upper layer */
	if (ncand > 0) {
		/*
		 * RX dedup: check message history for retransmission.
		 * If a matching (ric, function) message with the same codeword count
		 * exists within the dedup window, attempt codeword-level recovery
		 * and suppress duplicate delivery.
		 */
		int is_duplicate = 0;
		if (pocsag->rx_dedup_window > 0.0 && pocsag->rx_msg_cw_count > 0) {
			struct timeval tv;
			double now;
			int h;

			gettimeofday(&tv, NULL);
			now = (double)tv.tv_sec + tv.tv_usec / 1e6;

			for (h = 0; h < POCSAG_RX_HISTORY_MAX; h++) {
				struct pocsag_rx_history_entry *he = &pocsag->rx_history[h];
				if (!he->active)
					continue;
				if (now - he->timestamp > pocsag->rx_dedup_window)
					continue;
				if (he->ric != pocsag->rx_msg_ric || he->function != pocsag->rx_msg_function)
					continue;
				if (he->baudrate != pocsag->rx_baud_locked || he->polarity != pocsag->rx_polarity_locked)
					continue;
				if (he->codeword_count != pocsag->rx_msg_cw_count)
					continue;

				/* Match found — attempt codeword-level recovery */
				int recovered = 0, k;
				for (k = 0; k < pocsag->rx_msg_cw_count; k++) {
					if (pocsag->rx_msg_cw_status[k] == -1 && he->cw_status[k] >= 0) {
						/* Current copy uncorrectable, history copy good — recover */
						pocsag->rx_msg_cw_buf[k] = he->codewords[k];
						pocsag->rx_msg_cw_status[k] = he->cw_status[k];
						recovered++;
					}
					if (he->cw_status[k] == -1 && pocsag->rx_msg_cw_status[k] >= 0) {
						/* History copy uncorrectable, current copy good — update history */
						he->codewords[k] = pocsag->rx_msg_cw_buf[k];
						he->cw_status[k] = pocsag->rx_msg_cw_status[k];
					}
				}
				/* Update history timestamp to latest reception */
				he->timestamp = now;

				if (recovered)
					LOGP_CHAN(DPOCSAG, LOGL_INFO, "Retransmission dedup: recovered %d codeword(s) from previous copy (%d baud, %s).\n",
						  recovered, pocsag->rx_baud_locked,
						  (pocsag->rx_polarity_locked < 0) ? "normal" : "inverted");
				else
					LOGP_CHAN(DPOCSAG, LOGL_INFO, "Retransmission dedup: duplicate suppressed (RIC %d%s, %d baud, %s).\n",
						  pocsag->rx_msg_ric, pocsag_function_name[pocsag->rx_msg_function],
						  pocsag->rx_baud_locked,
						  (pocsag->rx_polarity_locked < 0) ? "normal" : "inverted");
				is_duplicate = 1;
				break;
			}

			/* Store current message in history (replace oldest or first empty) */
			{
				int best = -1;
				double oldest = 1e18;
				for (h = 0; h < POCSAG_RX_HISTORY_MAX; h++) {
					if (!pocsag->rx_history[h].active) {
						best = h;
						break;
					}
					if (pocsag->rx_history[h].timestamp < oldest) {
						oldest = pocsag->rx_history[h].timestamp;
						best = h;
					}
				}
				if (best >= 0) {
					struct pocsag_rx_history_entry *he = &pocsag->rx_history[best];
					he->ric = pocsag->rx_msg_ric;
					he->function = pocsag->rx_msg_function;
					he->msg_type = pocsag->rx_msg_type;
					he->baudrate = pocsag->rx_baud_locked;
					he->polarity = pocsag->rx_polarity_locked;
					he->codeword_count = pocsag->rx_msg_cw_count;
					memcpy(he->codewords, pocsag->rx_msg_cw_buf,
					       pocsag->rx_msg_cw_count * sizeof(uint32_t));
					memcpy(he->cw_status, pocsag->rx_msg_cw_status,
					       pocsag->rx_msg_cw_count * sizeof(int8_t));
					he->timestamp = now;
					he->active = 1;
				}
			}
		}

		if (!is_duplicate) {
			text = print_message(cand[0].data, cand[0].len);
			pocsag_msg_receive(pocsag->language, pocsag->sender.kanal, pocsag->rx_msg_ric,
					   pocsag->rx_msg_function, pocsag->rx_msg_type,
					   pocsag->rx_baud_locked, pocsag->rx_polarity_locked, text);
		}
	}
}

void put_codeword(pocsag_t *pocsag, uint32_t word, int8_t slot, int8_t subslot)
{
	int rc;
	uint32_t corrections = 0;
	int error_bits;

	if (slot < 0 && word == CODEWORD_SYNC) {
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Received 32 bits of sync pattern 0x%08x.\n", CODEWORD_SYNC);
		return;
	}

	if (word == CODEWORD_IDLE) {
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Received 32 bits of idle pattern 0x%08x.\n", CODEWORD_IDLE);
	} else
	if (!(word & 0x80000000))
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Received 32 bits of address codeword 0x%08x (frame %d.%d).\n", word, slot, subslot);
	else
		LOGP_CHAN(DPOCSAG, LOGL_DEBUG, "Received 32 bits of message codeword 0x%08x (frame %d.%d).\n", word, slot, subslot);

	/* Validate/correct codeword. Returns <0 uncorrectable, 0 clean, >0 bits corrected. */
	rc = debug_word(&word, slot, &corrections);

	/* Update sliding BER window */
	error_bits = (rc < 0) ? 32 : __builtin_popcount(corrections);
	{
		int idx = pocsag->rx_ber.head;
		pocsag->rx_ber.errors[idx] = error_bits;
		pocsag->rx_ber.total[idx] = 32;
		pocsag->rx_ber.head = (idx + 1) % POCSAG_BER_WINDOW;
		if (pocsag->rx_ber.count < POCSAG_BER_WINDOW)
			pocsag->rx_ber.count++;
	}

	if (rc < 0) {
		/* Uncorrectable — mark all 20 data bits as bad for current codeword */
		if (pocsag->rx_msg_valid) {
			if (pocsag->rx_msg_codewords >= RX_MSG_MAX_CODEWORDS) {
				LOGP_CHAN(DPOCSAG, LOGL_NOTICE, "Message exceeded %d codewords, truncating.\n", RX_MSG_MAX_CODEWORDS);
				done_rx_msg(pocsag);
				return;
			}
			pocsag->rx_msg_codewords++;
			pocsag->rx_msg_uncorrectable++;
			/* store codeword for dedup history */
			if (pocsag->rx_msg_cw_count < POCSAG_RX_HISTORY_CW_MAX) {
				pocsag->rx_msg_cw_buf[pocsag->rx_msg_cw_count] = word;
				pocsag->rx_msg_cw_status[pocsag->rx_msg_cw_count] = -1;
				pocsag->rx_msg_cw_count++;
			}
			/* Feed uncorrectable marker (all bits bad) to decode paths */
			uint32_t all_data_bits = 0xfffff800; /* bits 31-11 */
			decode_alpha(pocsag, word, all_data_bits);
			decode_numeric(pocsag, word, all_data_bits);
			store_raw_bits(pocsag, word);
		}
		done_rx_msg(pocsag);
		return;
	}

	if (word == CODEWORD_IDLE) {
		done_rx_msg(pocsag);
		return;
	}

	if (!(word & 0x80000000)) {
		/* Address codeword - start of new message */
		done_rx_msg(pocsag);
		pocsag->rx_msg_valid = 1;
		decode_address(word, slot, &pocsag->rx_msg_ric, &pocsag->rx_msg_function);
		pocsag->rx_msg_data_length = 0;
		pocsag->rx_msg_data_length_numeric = 0;
		pocsag->rx_msg_data_length_skyper = 0;
		pocsag->rx_msg_data_raw_bits = 0;
		pocsag->rx_msg_bit_index = 0;
		pocsag->rx_msg_cur_char_bad = 0;
		pocsag->rx_msg_numeric_nibble_count = 0;
		pocsag->rx_msg_codewords = 1;
		pocsag->rx_msg_corrected = (rc > 0) ? 1 : 0;
		pocsag->rx_msg_uncorrectable = 0;
		pocsag->rx_msg_type = POCSAG_MSG_TYPE_TONE;
		/* store address codeword for history */
		pocsag->rx_msg_cw_count = 0;
	} else {
		/* Message codeword - decode content in parallel */
		if (!pocsag->rx_msg_valid)
			return;
		if (pocsag->rx_msg_codewords >= RX_MSG_MAX_CODEWORDS) {
			LOGP_CHAN(DPOCSAG, LOGL_NOTICE, "Message exceeded %d codewords, truncating.\n", RX_MSG_MAX_CODEWORDS);
			done_rx_msg(pocsag);
			return;
		}
		pocsag->rx_msg_codewords++;
		if (rc > 0)
			pocsag->rx_msg_corrected++;
		/* store codeword for dedup history */
		if (pocsag->rx_msg_cw_count < POCSAG_RX_HISTORY_CW_MAX) {
			pocsag->rx_msg_cw_buf[pocsag->rx_msg_cw_count] = word;
			pocsag->rx_msg_cw_status[pocsag->rx_msg_cw_count] = (rc > 0) ? 1 : 0;
			pocsag->rx_msg_cw_count++;
		}
		decode_alpha(pocsag, word, corrections);
		decode_numeric(pocsag, word, corrections);
		store_raw_bits(pocsag, word);
	}
}

