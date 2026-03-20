/* Mobitex frame processing
 *
 * Frame-level FEC, CRC, interleaving, scrambling, and header/block processing.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "mobitex.h"
#include "frame.h"

/* Frame sync patterns (18 known network IDs) */
const uint16_t mobitex_frsyncs[18] = {
	0xC4D7, 0xB433, 0xA2F7, 0xEB90, 0xEE16, 0x4BCC, 0x3B48, 0x5D08, 0x146F,
	0x11E9, 0xEB23, 0xEF45, 0x09D7, 0x6877, 0x14DC, 0x10BA, 0xF628, 0x9788
};

/* Reversed polarity frame sync patterns */
const uint16_t mobitex_frevsyncs[18] = {
	0x3B28, 0x4BCC, 0x5D08, 0x146F, 0x11E9, 0xB433, 0xC4B7, 0xA2F7, 0xEB90,
	0xEE16, 0x14DC, 0x10BA, 0xF628, 0x9788, 0xEB23, 0xEF45, 0x09D7, 0x6877
};

/* ---- Bit Scrambler ---- */

void mobitex_scrambler_reset(uint16_t *sr)
{
	*sr = MOBITEX_SCRAMBLER_INIT;
}

int mobitex_scrambler_bit(uint16_t *sr, int scrambling)
{
	int output;

	if (!scrambling)
		return 0;

	/* XOR taps at bit positions 0 and 4 (positions 1 and 5, 1-indexed) */
	output = (*sr & 1) ^ ((*sr >> MOBITEX_SCRAMBLER_TAP1) & 1);

	/* Shift right and feed output into MSB (bit 8) */
	*sr >>= 1;
	*sr |= (output << MOBITEX_SCRAMBLER_TAP2);

	return output;
}

/* ---- FEC (12,8) Hamming ---- */

/* Portable popcount for systems without __builtin_popcount */
static int popcount8(unsigned int x)
{
	x = x - ((x >> 1) & 0x55);
	x = (x & 0x33) + ((x >> 2) & 0x33);
	return (x + (x >> 4)) & 0x0F;
}

int mobitex_fec_encode(int data_byte)
{
	int d = data_byte & 0xFF;
	int parity;

	parity  = (popcount8(d & MOBITEX_FEC_H3) & 1) << 3;
	parity |= (popcount8(d & MOBITEX_FEC_H2) & 1) << 2;
	parity |= (popcount8(d & MOBITEX_FEC_H1) & 1) << 1;
	parity |= (popcount8(d & MOBITEX_FEC_H0) & 1);

	return (d << 4) | parity;
}

int mobitex_fec_decode(int *codeword)
{
	/*
	 * H-matrix column table: for each of the 12 bit positions (11 down to 0),
	 * the 4-bit syndrome value that corresponds to an error in that position.
	 *
	 * Positions 11-4 are data bits (d7-d0). The column for data bit i is
	 * formed by extracting bit i from each of h3, h2, h1, h0.
	 *
	 * Positions 3-0 are parity bits, forming an identity matrix:
	 *   pos 3 → syndrome 8, pos 2 → 4, pos 1 → 2, pos 0 → 1
	 */
	static const int h_column[12] = {
		/* bit 11 (d7) */ ((0xEC >> 7) & 1) << 3 | ((0xD3 >> 7) & 1) << 2 | ((0xBA >> 7) & 1) << 1 | ((0x75 >> 7) & 1),
		/* bit 10 (d6) */ ((0xEC >> 6) & 1) << 3 | ((0xD3 >> 6) & 1) << 2 | ((0xBA >> 6) & 1) << 1 | ((0x75 >> 6) & 1),
		/* bit  9 (d5) */ ((0xEC >> 5) & 1) << 3 | ((0xD3 >> 5) & 1) << 2 | ((0xBA >> 5) & 1) << 1 | ((0x75 >> 5) & 1),
		/* bit  8 (d4) */ ((0xEC >> 4) & 1) << 3 | ((0xD3 >> 4) & 1) << 2 | ((0xBA >> 4) & 1) << 1 | ((0x75 >> 4) & 1),
		/* bit  7 (d3) */ ((0xEC >> 3) & 1) << 3 | ((0xD3 >> 3) & 1) << 2 | ((0xBA >> 3) & 1) << 1 | ((0x75 >> 3) & 1),
		/* bit  6 (d2) */ ((0xEC >> 2) & 1) << 3 | ((0xD3 >> 2) & 1) << 2 | ((0xBA >> 2) & 1) << 1 | ((0x75 >> 2) & 1),
		/* bit  5 (d1) */ ((0xEC >> 1) & 1) << 3 | ((0xD3 >> 1) & 1) << 2 | ((0xBA >> 1) & 1) << 1 | ((0x75 >> 1) & 1),
		/* bit  4 (d0) */ ((0xEC >> 0) & 1) << 3 | ((0xD3 >> 0) & 1) << 2 | ((0xBA >> 0) & 1) << 1 | ((0x75 >> 0) & 1),
		/* bit  3 (p3) */ 0x08,
		/* bit  2 (p2) */ 0x04,
		/* bit  1 (p1) */ 0x02,
		/* bit  0 (p0) */ 0x01,
	};
	int c, data, recv_parity, expected_parity, syndrome;
	int i;

	c = *codeword & 0xFFF;
	data = (c >> 4) & 0xFF;
	recv_parity = c & 0x0F;

	/* Recompute expected parity from data bits (same as encode) */
	expected_parity  = (popcount8(data & MOBITEX_FEC_H3) & 1) << 3;
	expected_parity |= (popcount8(data & MOBITEX_FEC_H2) & 1) << 2;
	expected_parity |= (popcount8(data & MOBITEX_FEC_H1) & 1) << 1;
	expected_parity |= (popcount8(data & MOBITEX_FEC_H0) & 1);

	syndrome = expected_parity ^ recv_parity;

	if (syndrome == 0)
		return 0; /* No error */

	/* Check if syndrome matches any H-matrix column */
	for (i = 0; i < 12; i++) {
		if (h_column[i] == syndrome) {
			/* Single-bit error at position (11 - i) in the 12-bit codeword */
			c ^= (1 << (11 - i));
			*codeword = c;
			return 1; /* Corrected single-bit error */
		}
	}

	/* Syndrome doesn't match any column → uncorrectable multi-bit error */
	return -1;
}

/* ---- CRC-16 ---- */

void mobitex_crc_reset(uint16_t *sr, uint16_t *cr)
{
	*sr = MOBITEX_CRC_SR_INIT;
	*cr = MOBITEX_CRC_CR_INIT;
}

void mobitex_crc_bit(uint16_t *sr, uint16_t *cr, int bit)
{
	if (bit & 1)
		*cr ^= *sr;
	if (*sr & 0x8000)
		*sr = (*sr << 1) ^ MOBITEX_CRC_POLY;
	else
		*sr = (*sr << 1);
}

uint16_t mobitex_crc_finalize(uint16_t cr)
{
	return cr;
}

/* ---- Interleaving ---- */

/*
 * Interleave 20 twelve-bit FEC codewords into a 240-bit block.
 * Column-wise mapping: bit j of codeword i → position (j × 20) + i.
 * Bit 11 is the MSB of each codeword, bit 0 is the LSB.
 */
void mobitex_interleave(const uint16_t codewords[20], uint8_t bits[240])
{
	int i, j;

	for (i = 0; i < 20; i++) {
		for (j = 0; j < 12; j++) {
			bits[j * 20 + i] = (codewords[i] >> (11 - j)) & 1;
		}
	}
}

/*
 * De-interleave a 240-bit block back into 20 twelve-bit FEC codewords.
 * Inverse of mobitex_interleave: position (j × 20) + i → bit j of codeword i.
 * Matches the PDW barfrog() de-interleaving logic.
 */
void mobitex_deinterleave(const uint8_t bits[240], uint16_t codewords[20])
{
	int i, j;

	for (i = 0; i < 20; i++) {
		uint16_t cw = 0;
		for (j = 0; j < 12; j++) {
			cw = (cw << 1) | (bits[j * 20 + i] & 1);
		}
		codewords[i] = cw;
	}
}

/* ---- Data Block Processing ---- */

int mobitex_encode_block(mobitex_t *mobitex, const uint8_t data[18],
                         uint8_t bits_out[240])
{
	uint16_t codewords[20];
	uint16_t crc_sr, crc_cr;
	uint16_t crc;
	int i, j;

	/* Step 1: FEC-encode each of 18 data bytes */
	for (i = 0; i < 18; i++)
		codewords[i] = mobitex_fec_encode(data[i]);

	/* Step 2: Compute CRC-16 over 144 data bits (LSB first per byte) */
	mobitex_crc_reset(&crc_sr, &crc_cr);
	for (i = 0; i < 18; i++) {
		for (j = 0; j < 8; j++)
			mobitex_crc_bit(&crc_sr, &crc_cr, (data[i] >> j) & 1);
	}
	crc = mobitex_crc_finalize(crc_cr);

	/* Step 3: Split CRC into high and low bytes, FEC-encode each */
	codewords[18] = mobitex_fec_encode((crc >> 8) & 0xFF);
	codewords[19] = mobitex_fec_encode(crc & 0xFF);

	/* Step 4: Interleave 20 codewords into 240 bits */
	mobitex_interleave(codewords, bits_out);

	/* Step 5: Scramble 240 bits.
	 * The scrambler runs continuously across all blocks in a frame —
	 * callers must reset it once before the first block. */
	for (i = 0; i < 240; i++)
		bits_out[i] ^= mobitex_scrambler_bit(&mobitex->scrambler_tx_sr, mobitex->scrambling);

	return 0;
}

int mobitex_decode_block(mobitex_t *mobitex, const uint8_t bits_in[240],
                         uint8_t data_out[18], int *fec_errors)
{
	uint8_t bits[240];
	uint16_t codewords[20];
	uint16_t crc_sr, crc_cr;
	uint16_t recv_crc;
	int total_errors = 0;
	int rc;
	int i, j;

	/* Step 1: Descramble 240 bits (copy first, then XOR with scrambler).
	 * The scrambler runs continuously across all blocks in a frame —
	 * callers must reset it once before the first block. */
	for (i = 0; i < 240; i++)
		bits[i] = bits_in[i] ^ mobitex_scrambler_bit(&mobitex->scrambler_rx_sr, mobitex->scrambling);

	/* Step 2: De-interleave 240 bits into 20 twelve-bit codewords */
	mobitex_deinterleave(bits, codewords);

	/* Step 3: FEC-decode each codeword, track error count */
	for (i = 0; i < 20; i++) {
		int cw = codewords[i];
		rc = mobitex_fec_decode(&cw);
		if (rc < 0)
			total_errors += 2; /* uncorrectable counts as 2 */
		else
			total_errors += rc;
		codewords[i] = cw;
	}

	*fec_errors = total_errors;

	/* Step 4: Extract 18 data bytes and 2 CRC bytes from decoded codewords */
	for (i = 0; i < 18; i++)
		data_out[i] = (codewords[i] >> 4) & 0xFF;

	recv_crc = (((codewords[18] >> 4) & 0xFF) << 8) |
	            ((codewords[19] >> 4) & 0xFF);

	/* Step 5: Recompute CRC-16 over 144 data bits (LSB first per byte) */
	mobitex_crc_reset(&crc_sr, &crc_cr);
	for (i = 0; i < 18; i++) {
		for (j = 0; j < 8; j++)
			mobitex_crc_bit(&crc_sr, &crc_cr, (data_out[i] >> j) & 1);
	}

	/* Step 6: Compare computed CRC with received CRC */
	if (mobitex_crc_finalize(crc_cr) != recv_crc)
		return -1; /* CRC failure */

	return 0;
}

/* ---- Frame Header ---- */

/*
 * Encode a frame header into 24 over-the-air bits.
 *
 * 24-bit layout (matching PDW's shift-register extraction):
 *   bits 0-7:   CB1 data (8 bits, MSB first)
 *   bits 8-15:  CB2 data (8 bits, MSB first)
 *   bits 16-19: CB1 FEC parity (4 bits, MSB first)
 *   bits 20-23: CB2 FEC parity (4 bits, MSB first)
 *
 * PDW's frame_sync() sets bc=25 then immediately decrements to 24,
 * so it collects exactly 24 bits after the framesync match. No spare bit.
 *
 * Control byte 1 data packing:
 *   bits 7-2: Base ID (6 bits)
 *   bits 1-0: upper 2 bits of Area ID
 *
 * Control byte 2 data packing:
 *   bits 7-4: lower 4 bits of Area ID
 *   bits 3-0: CFlags (4 bits)
 */
void mobitex_encode_header(const mobitex_frame_header_t *hdr,
                           uint8_t bits_out[MOBITEX_HEADER_BITS])
{
	uint8_t cb1_data, cb2_data;
	int cb1_codeword, cb2_codeword;
	int i;

	/* Pack fields into two control byte data values */
	cb1_data = ((hdr->base_id & 0x3F) << 2) | ((hdr->area_id >> 4) & 0x03);
	cb2_data = ((hdr->area_id & 0x0F) << 4) | (hdr->cflags & 0x0F);

	/* FEC-encode each control byte (8-bit data → 12-bit codeword) */
	cb1_codeword = mobitex_fec_encode(cb1_data);
	cb2_codeword = mobitex_fec_encode(cb2_data);

	/* 24-bit layout matching PDW's shift-register extraction:
	 *   bits 0-7:   CB1 data (MSB first)
	 *   bits 8-15:  CB2 data (MSB first)
	 *   bits 16-19: CB1 FEC parity (MSB first)
	 *   bits 20-23: CB2 FEC parity (MSB first)
	 *
	 * PDW's frame_sync() sets bc=25 then immediately decrements to 24,
	 * so it collects exactly 24 bits after the framesync match. There is
	 * no spare bit — the 24 bits are two 12-bit FEC codewords laid out
	 * as [data1(8)][data2(8)][parity1(4)][parity2(4)]. */

	/* Bits 0-7: CB1 data (MSB first) */
	for (i = 0; i < 8; i++)
		bits_out[i] = (cb1_data >> (7 - i)) & 1;

	/* Bits 8-15: CB2 data (MSB first) */
	for (i = 0; i < 8; i++)
		bits_out[8 + i] = (cb2_data >> (7 - i)) & 1;

	/* Bits 16-19: CB1 FEC parity (MSB first) */
	for (i = 0; i < 4; i++)
		bits_out[16 + i] = ((cb1_codeword & 0x0F) >> (3 - i)) & 1;

	/* Bits 20-23: CB2 FEC parity (MSB first) */
	for (i = 0; i < 4; i++)
		bits_out[20 + i] = ((cb2_codeword & 0x0F) >> (3 - i)) & 1;
}

/*
 * Decode a frame header from 24 over-the-air bits.
 *
 * 24-bit layout (no spare bit, matching PDW's actual extraction):
 *   bits 0-7:   CB1 data (8 bits, MSB first)
 *   bits 8-15:  CB2 data (8 bits, MSB first)
 *   bits 16-19: CB1 FEC parity (4 bits, MSB first)
 *   bits 20-23: CB2 FEC parity (4 bits, MSB first)
 *
 * Reconstructs the two 12-bit FEC codewords, FEC-decodes each, and
 * extracts the header fields.
 *
 * On FEC failure, retains the previous Base ID in hdr (per PDW behavior).
 *
 * Returns 0 on success, -1 on FEC failure.
 */
int mobitex_decode_header(const uint8_t bits_in[MOBITEX_HEADER_BITS],
                          mobitex_frame_header_t *hdr)
{
	int cb1_codeword, cb2_codeword;
	int cb1_data_bits, cb2_data_bits;
	int cb1_parity, cb2_parity;
	int fec1, fec2;
	uint8_t prev_base_id;
	int i;

	/* Extract CB1 data (8 bits) from bits 0-7 */
	cb1_data_bits = 0;
	for (i = 0; i < 8; i++)
		cb1_data_bits = (cb1_data_bits << 1) | (bits_in[i] & 1);

	/* Extract CB2 data (8 bits) from bits 8-15 */
	cb2_data_bits = 0;
	for (i = 0; i < 8; i++)
		cb2_data_bits = (cb2_data_bits << 1) | (bits_in[8 + i] & 1);

	/* Extract CB1 parity (4 bits) from bits 16-19 */
	cb1_parity = 0;
	for (i = 0; i < 4; i++)
		cb1_parity = (cb1_parity << 1) | (bits_in[16 + i] & 1);

	/* Extract CB2 parity (4 bits) from bits 20-23 */
	cb2_parity = 0;
	for (i = 0; i < 4; i++)
		cb2_parity = (cb2_parity << 1) | (bits_in[20 + i] & 1);

	/* Reconstruct 12-bit FEC codewords: [data(8) << 4 | parity(4)] */
	cb1_codeword = (cb1_data_bits << 4) | cb1_parity;
	cb2_codeword = (cb2_data_bits << 4) | cb2_parity;

	/* Save previous Base ID in case FEC fails */
	prev_base_id = hdr->base_id;

	/* FEC-decode each control byte */
	fec1 = mobitex_fec_decode(&cb1_codeword);
	fec2 = mobitex_fec_decode(&cb2_codeword);

	if (fec1 < 0 || fec2 < 0) {
		/* Uncorrectable FEC error: retain previous Base ID */
		hdr->base_id = prev_base_id;
		return -1;
	}

	/* Extract corrected data bytes */
	cb1_data_bits = (cb1_codeword >> 4) & 0xFF;
	cb2_data_bits = (cb2_codeword >> 4) & 0xFF;

	/* Unpack header fields */
	hdr->base_id = (cb1_data_bits >> 2) & 0x3F;
	hdr->area_id = ((cb1_data_bits & 0x03) << 4) | ((cb2_data_bits >> 4) & 0x0F);
	hdr->cflags  = cb2_data_bits & 0x0F;
	hdr->parity  = (cb1_codeword & 0x0F) << 4 | (cb2_codeword & 0x0F);

	return 0;
}

/* ---- Link Control ---- */

void mobitex_parse_link_control(const uint8_t block[20],
                                mobitex_link_control_t *lc)
{
	lc->dest_man     = ((uint32_t)block[0] << 16) | ((uint32_t)block[1] << 8) | block[2];
	lc->frame_id     = block[3] & 0x1F;
	lc->seq_num      = block[4] & 0x0F;
	lc->bytes_last   = (((block[3] >> 5) & 0x01) << 4) | ((block[4] >> 4) & 0x0F);
	lc->block_length = block[5];
}

void mobitex_build_link_control(const mobitex_link_control_t *lc,
                                uint8_t block[20])
{
	block[0] = (lc->dest_man >> 16) & 0xFF;
	block[1] = (lc->dest_man >> 8) & 0xFF;
	block[2] = lc->dest_man & 0xFF;
	block[3] = (lc->frame_id & 0x1F) | (((lc->bytes_last >> 4) & 0x01) << 5);
	block[4] = (lc->seq_num & 0x0F) | ((lc->bytes_last & 0x0F) << 4);
	block[5] = lc->block_length;
}

/* ---- MPAK Header ---- */

void mobitex_parse_mpak_header(const uint8_t block[20],
                               mobitex_mpak_header_t *mpak)
{
	/* Bytes 6-8: Sender MAN (24 bits) — MIS octets 1-3 */
	mpak->sender_man   = ((uint32_t)block[6] << 16) | ((uint32_t)block[7] << 8) | block[8];
	/* Bytes 9-11: Destination MAN (24 bits) — MIS octets 4-6 */
	mpak->dest_man     = ((uint32_t)block[9] << 16) | ((uint32_t)block[10] << 8) | block[11];

	/* Byte 12: MIS octet 7 — subscription flags + traffic state
	 *   bits 7-5: traffic state (3 bits, values 0-6)
	 *   bit 4:    reserve flag (always 0)
	 *   bit 3:    UNKNOWN_F
	 *   bit 2:    SENDLIST_F (address list included)
	 *   bit 1:    POSACK_F (positive acknowledgement requested)
	 *   bit 0:    MAILBOX_F (may be placed in network mailbox) */
	mpak->traffic_state = (block[12] >> 5) & 0x07;
	mpak->reserve_f     = (block[12] >> 4) & 0x01;
	mpak->unknown_f     = (block[12] >> 3) & 0x01;
	mpak->sendlist_f    = (block[12] >> 2) & 0x01;
	mpak->posack_f      = (block[12] >> 1) & 0x01;
	mpak->mailbox_f     = (block[12] >> 0) & 0x01;
	mpak->address_list  = mpak->sendlist_f; /* backward compat alias */

	/* Byte 13: MIS octet 8 — packet class, external flag, packet type
	 *   bits 7-6: packet class (2 bits: 0=PSUBCOM, 3=DTESERV)
	 *   bit 5:    EXTERN_F (external network traffic)
	 *   bits 4-0: packet type (5 bits) */
	mpak->mpak_class   = (block[13] >> 6) & 0x03;
	mpak->extern_f     = (block[13] >> 5) & 0x01;
	mpak->mpak_type    = block[13] & 0x1F;

	/* Byte 17: Higher Protocol ID (for HPDATA packets) */
	mpak->hpid         = block[17];
}

void mobitex_build_mpak_header(const mobitex_mpak_header_t *mpak,
                               uint8_t block[20])
{
	/* Bytes 6-8: Sender MAN (24 bits) — MIS octets 1-3 */
	block[6]  = (mpak->sender_man >> 16) & 0xFF;
	block[7]  = (mpak->sender_man >> 8) & 0xFF;
	block[8]  = mpak->sender_man & 0xFF;
	/* Bytes 9-11: Destination MAN (24 bits) — MIS octets 4-6 */
	block[9]  = (mpak->dest_man >> 16) & 0xFF;
	block[10] = (mpak->dest_man >> 8) & 0xFF;
	block[11] = mpak->dest_man & 0xFF;

	/* Byte 12: MIS octet 7 — traffic state (bits 7-5) + flags (bits 4-0) */
	block[12] = ((mpak->traffic_state & 0x07) << 5)
	          | ((mpak->reserve_f & 0x01) << 4)
	          | ((mpak->unknown_f & 0x01) << 3)
	          | ((mpak->sendlist_f & 0x01) << 2)
	          | ((mpak->posack_f & 0x01) << 1)
	          | ((mpak->mailbox_f & 0x01) << 0);

	/* Byte 13: MIS octet 8 — class (bits 7-6) + EXTERN_F (bit 5) + type (bits 4-0) */
	block[13] = ((mpak->mpak_class & 0x03) << 6)
	          | ((mpak->extern_f & 0x01) << 5)
	          | (mpak->mpak_type & 0x1F);

	block[14] = 0; /* application-specific, zeroed */
	block[15] = 0;
	block[16] = 0;
	block[17] = mpak->hpid;
}

/* ---- Frame Sync ---- */

int mobitex_match_frsync(uint16_t word, int *index)
{
	int i;

	for (i = 0; i < 18; i++) {
		if (word == mobitex_frsyncs[i]) {
			*index = i;
			return 1;
		}
		if (word == mobitex_frevsyncs[i]) {
			*index = i;
			return -1;
		}
	}

	return 0;
}

/* ---- Frequency Mapping ---- */

typedef struct mobitex_fbi_entry {
	int    fbi;             /* FBI code */
	double base_freq_mhz;  /* Base frequency in MHz */
	double spacing_mhz;    /* Channel spacing in MHz */
} mobitex_fbi_entry_t;

static const mobitex_fbi_entry_t mobitex_fbi_table[] = {
	{ 0, 819.0,  0.00625 },  /* 800 MHz band, 6.25 kHz spacing */
	{ 3, 380.0,  0.0125  },  /* 400 MHz band, 12.5 kHz spacing (same as FBI 5) */
	{ 4, 890.0,  0.0125  },  /* 900 MHz band, 12.5 kHz spacing */
	{ 5, 380.0,  0.0125  },  /* 400 MHz band, 12.5 kHz spacing */
};

double mobitex_fbi_to_freq(int fbi, int channel_number)
{
	size_t i;

	for (i = 0; i < sizeof(mobitex_fbi_table) / sizeof(mobitex_fbi_table[0]); i++) {
		if (mobitex_fbi_table[i].fbi == fbi)
			return mobitex_fbi_table[i].base_freq_mhz + channel_number * mobitex_fbi_table[i].spacing_mhz;
	}

	return -1.0;
}
