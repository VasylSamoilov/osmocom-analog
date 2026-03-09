/* Golay/GSC transcoding (encoding only - maybe)
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * Inspired by GSC code written by Brandon Creighton <cstone@pobox.com>.
 *
 * Inspired by GOLAY code written by Robert Morelos-Zaragoza
 * <robert@spectra.eng.hawaii.edu>.
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

/* Golay code was is use since 1973, the GSC extension was used after 1982.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/param.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/main_mobile.h"
#include "../libmobile/cause.h"
#include "golay.h"
#include "dsp.h"

/* Terminal Input Flow (Appendix IV)
 *
 * Selector-level systems:
 *   Input format: NNX-PPPP
 *     NNX  = telephone exchange code
 *     PPPP = pager user code (assigned by operator)
 *   Terminal maps NNX-PPPP to internal GSC code (I G1 G0 A2 A1 A0).
 *
 * DTMF (touch code) systems:
 *   Input format: NNX-XXXX-PPPP-f-DDDD
 *     NNX  = telephone exchange code
 *     XXXX = system access code
 *     PPPP = pager user code
 *     f    = Function Plan "A" post-entry digit:
 *              1-4 = voice, 5-8 = alphanumeric, 9/0 = tone-only
 *     DDDD = optional numeric data digits
 *   Terminal cross-references pager user code to internal GSC code.
 *
 * The internal GSC code (I G1 G0 A2 A1 A0) maps to the over-air
 * encoding via encode_address(): I selects preamble, G1G0 selects
 * Word 1, and A2A1A0 are encoded into Word 2.
 */

/* Create transceiver instance and link to a list. */
int golay_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, double deviation, double polarity, int tx, int rx, int auto_polarity, const char *message, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback, const char *voice_dir, int voice_monitor)
{
	gsc_t *gsc;
	int rc;

	gsc = calloc(1, sizeof(*gsc));
	if (!gsc) {
		LOGP(DGOLAY, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	LOGP(DGOLAY, LOGL_DEBUG, "Creating 'GOLAY' instance for frequency = %s (sample rate %d).\n", kanal, samplerate);

	/* init general part of transceiver */
	rc = sender_create(&gsc->sender, kanal, frequency, frequency, device, use_sdr, samplerate, rx_gain, tx_gain, 0, 0, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DGOLAY, LOGL_ERROR, "Failed to init transceiver process!\n");
		goto error;
	}

	/* init audio processing */
	rc = dsp_init_sender(gsc, samplerate, deviation, polarity);
	if (rc < 0) {
		LOGP(DGOLAY, LOGL_ERROR, "Failed to init audio processing!\n");
		goto error;
	}

	gsc->tx = tx;
	gsc->rx = rx;
	gsc->default_message = message;

	/* Initialize RX decoder state */
	if (rx) {
		gsc->rx_bit_num = 0;
		gsc->rx_bit_index = 0;
		gsc->rx_state = RX_IDLE;
		gsc->rx_preamble_index = 0;
		gsc->rx_confirm_index = -1;
		gsc->rx_confirm_count = 0;
		gsc->rx_confirm_bit_count = 0;
		gsc->rx_auto_polarity = auto_polarity;
		gsc->rx_polarity_inverted = 0;
		if (auto_polarity)
			LOGP(DGOLAY, LOGL_INFO, "RX polarity: auto-detect.\n");
		else
			LOGP(DGOLAY, LOGL_INFO, "RX polarity: locked to %s.\n", (polarity < 0) ? "negative" : "positive");
		LOGP(DGOLAY, LOGL_INFO, "Receive mode enabled.\n");
	}

	/* Voice recording options */
	gsc->voice_dir = voice_dir;
	gsc->voice_monitor = voice_monitor;
	gsc->voice_recording = 0;
	if (voice_dir && rx)
		LOGP(DGOLAY, LOGL_INFO, "Voice recording enabled, output dir: %s\n", voice_dir);
	if (voice_monitor && rx)
		LOGP(DGOLAY, LOGL_INFO, "Voice monitor mode enabled.\n");

	LOGP(DGOLAY, LOGL_NOTICE, "Created %s%s%s for frequency %s\n",
		tx ? "transmitter" : "",
		(tx && rx) ? " and " : "",
		rx ? "receiver" : "",
		kanal);

	return 0;

error:
	golay_destroy(&gsc->sender);

	return rc;
}

static void golay_msg_destroy(gsc_t *gsc, gsc_msg_t *msg);

/* Destroy transceiver instance and unlink from list. */
void golay_destroy(sender_t *sender)
{
	gsc_t *gsc = (gsc_t *) sender;

	LOGP(DGOLAY, LOGL_DEBUG, "Destroying 'GOLAY' instance for frequency = %s.\n", sender->kanal);

	/* Stop any in-progress voice recording */
	if (gsc->voice_recording) {
		wave_destroy_record(&gsc->voice_rec);
		gsc->voice_recording = 0;
	}

	while (gsc->msg_list)
		golay_msg_destroy(gsc, gsc->msg_list);
	dsp_cleanup_sender(gsc);
	sender_destroy(&gsc->sender);
	free(gsc);
}

/* Create message and add to queue */
static gsc_msg_t *golay_msg_create(gsc_t *gsc, const char *address, const char *text, enum gsc_msg_type type)
{
	gsc_msg_t *msg, **msgp;

	if (strlen(address) != sizeof(msg->address) - 1) {
		LOGP(DGOLAY, LOGL_NOTICE, "Address has incorrect length, cannot page!\n");
		return NULL;
	}
	if (strlen(text) > sizeof(msg->data) - 1) {
		LOGP(DGOLAY, LOGL_NOTICE, "Given test is too long, cannot page!\n");
		return NULL;
	}

	/* get type from last digit, if automatic type is given */
	if (type == TYPE_AUTO) {
		switch (address[6]) {
			case '1': type = TYPE_VOICE; break;
			case '2': type = TYPE_VOICE; break;
			case '3': type = TYPE_VOICE; break;
			case '4': type = TYPE_VOICE; break;
			case '5': type = TYPE_ALPHA; break;
			case '6': type = TYPE_ALPHA; break;
			case '7': type = TYPE_ALPHA; break;
			case '8': type = TYPE_ALPHA; break;
			case '9': type = TYPE_TONE;  break;
			case '0': type = TYPE_TONE;  break;
			default:
				LOGP(DGOLAY, LOGL_NOTICE, "Illegal function suffix '%c' in last address digit.\n", address[6]);
				return NULL;
		}
	} else
		LOGP(DGOLAY, LOGL_INFO, "Overriding message type as defined by sender.\n");

	LOGP(DGOLAY, LOGL_INFO, "Creating msg instance to page address '%s'.\n", address);

	/* create */
	msg = calloc(1, sizeof(*msg));
	if (!msg) {
		LOGP(DGOLAY, LOGL_ERROR, "No mem!\n");
		abort();
	}

	/* init */
	memcpy(msg->address, address, MIN(sizeof(msg->address) - 1, strlen(address) + 1));
	msg->type = type;
	memcpy(msg->data, text, MIN(sizeof(msg->data) - 1, strlen(text) + 1));

	/* link */
	msgp = &gsc->msg_list;
	while ((*msgp))
		msgp = &(*msgp)->next;
	(*msgp) = msg;

	return msg;
}

/* Remove and destroy msg from queue */
static void golay_msg_destroy(gsc_t *gsc, gsc_msg_t *msg)
{
	gsc_msg_t **msgp;

	/* unlink */
	msgp = &gsc->msg_list;
	while ((*msgp) != msg)
		msgp = &(*msgp)->next;
	(*msgp) = msg->next;

	/* destroy */
	free(msg);
}

/* uncomment this for showing encoder tables ("<parity> <information>", LSB is the right most bit) */
//#define DEBUG_TABLE

static uint32_t golay_table[4096];

/* Golay syndrome lookup table: indexed by 11-bit syndrome (0-2047),
 * value is the 23-bit error pattern for correctable errors (weight <= 3).
 * Entries with value 0xFFFFFFFF indicate uncorrectable error patterns.
 * Built during init_golay(). */
static uint32_t golay_syndrome[2048];

#define X22	0x00400000
#define X11	0x00000800
#define MASK12	0xfffff800
#define GEN_GOL	0x00000c75

/* Compute the 11-bit syndrome of a 23-bit Golay codeword.
 * The syndrome is the remainder after dividing by the generator polynomial.
 * A syndrome of 0 means the codeword is valid (no errors). */
static uint32_t golay_syndrome_calc(uint32_t codeword)
{
	uint32_t syndrome, aux;

	syndrome = codeword;
	aux = X22;
	if (syndrome >= X11) {
		while (syndrome & MASK12) {
			while (!(aux & syndrome))
				aux = aux >> 1;
			syndrome ^= (aux / X11) * GEN_GOL;
		}
	}

	return syndrome;
}

/* generate golay encoding table. the redundancy is shifted 12 bits */
void init_golay(void)
{
	uint32_t syndrome, aux;
	int data;
	uint32_t error, s;
	int i, j, k;

	for (data = 0; data < 4096; data++) {
		syndrome = data << 11;
		/* calculate syndrome */
		aux = X22;
		if (syndrome >= X11) {
			while (syndrome & MASK12) {
				while (!(aux & syndrome))
					aux = aux >> 1;
				syndrome ^= (aux / X11) * GEN_GOL;
			}
		}
		golay_table[data] = data | (syndrome << 12);
#ifdef DEBUG_TABLE
		printf("Golay %4d: ", data);
		for (int i = 22; i >= 0; i--) {
			if (i == 11)
				printf(" ");
			printf("%d", (golay_table[data] >> i) & 1);
		}
		printf("\n");
#endif
	}

	/* Build syndrome lookup table for Golay(23,12) error correction.
	 * For each correctable error pattern (weight 1, 2, or 3 over 23 bits),
	 * compute its syndrome and store: golay_syndrome[syndrome] = error_pattern.
	 * The syndrome is 11 bits (2048 entries). */

	/* Initialize all entries as uncorrectable */
	for (i = 0; i < 2048; i++)
		golay_syndrome[i] = 0xFFFFFFFF;

	/* Syndrome 0 = no errors */
	golay_syndrome[0] = 0x00000000;

	/* Weight-1 error patterns: single bit flipped in 23-bit codeword */
	for (i = 0; i < 23; i++) {
		error = (1U << i);
		s = golay_syndrome_calc(error);
		golay_syndrome[s] = error;
	}

	/* Weight-2 error patterns: two bits flipped */
	for (i = 0; i < 23; i++) {
		for (j = i + 1; j < 23; j++) {
			error = (1U << i) | (1U << j);
			s = golay_syndrome_calc(error);
			golay_syndrome[s] = error;
		}
	}

	/* Weight-3 error patterns: three bits flipped */
	for (i = 0; i < 23; i++) {
		for (j = i + 1; j < 23; j++) {
			for (k = j + 1; k < 23; k++) {
				error = (1U << i) | (1U << j) | (1U << k);
				s = golay_syndrome_calc(error);
				golay_syndrome[s] = error;
			}
		}
	}
}

static uint16_t bch_table[128];

/* BCH syndrome lookup table: indexed by 8-bit syndrome (0-255),
 * value is the 15-bit error pattern for correctable errors (weight <= 2).
 * Entries with value 0xFFFF indicate uncorrectable error patterns.
 * Built during init_bch(). */
static uint16_t bch_syndrome[256];

#define X14	0x4000
#define X8	0x0100
#define MASK7	0xff00
#define GEN_BCH	0x00000117

/* Compute the 8-bit syndrome of a 15-bit BCH codeword.
 * The syndrome is the remainder after dividing by the generator polynomial.
 * A syndrome of 0 means the codeword is valid (no errors). */
static uint16_t bch_syndrome_calc(uint16_t codeword)
{
	uint16_t syndrome, aux;

	syndrome = codeword;
	aux = X14;
	if (syndrome >= X8) {
		while (syndrome & MASK7) {
			while (!(aux & syndrome))
				aux = aux >> 1;
			syndrome ^= (aux / X8) * GEN_BCH;
		}
	}

	return syndrome;
}

/* generate bch encoding table. the redundancy is shifted 7 bits */
void init_bch(void)
{
	uint16_t syndrome, aux;
	int data;
	uint16_t error, s;
	int i, j;

	for (data = 0; data < 128; data++) {
		syndrome = data << 8;
		/* calculate syndrome */
		aux = X14;
		if (syndrome >= X8) {
			while (syndrome & MASK7) {
				while (!(aux & syndrome))
					aux = aux >> 1;
				syndrome ^= (aux / X8) * GEN_BCH;
			}
		}
		bch_table[data] = data | (syndrome << 7);
#ifdef DEBUG_TABLE
		printf("BCH %3d: ", data);
		for (int i = 14; i >= 0; i--) {
			if (i == 6)
				printf(" ");
			printf("%d", (bch_table[data] >> i) & 1);
		}
		printf("\n");
#endif
	}

	/* Build syndrome lookup table for BCH(15,7) error correction.
	 * For each correctable error pattern (weight 1 or 2 over 15 bits),
	 * compute its syndrome and store: bch_syndrome[syndrome] = error_pattern.
	 * The syndrome is 8 bits (256 entries). */

	/* Initialize all entries as uncorrectable */
	for (i = 0; i < 256; i++)
		bch_syndrome[i] = 0xFFFF;

	/* Syndrome 0 = no errors */
	bch_syndrome[0] = 0x0000;

	/* Weight-1 error patterns: single bit flipped in 15-bit codeword */
	for (i = 0; i < 15; i++) {
		error = (1U << i);
		s = bch_syndrome_calc(error);
		bch_syndrome[s] = error;
	}

	/* Weight-2 error patterns: two bits flipped */
	for (i = 0; i < 15; i++) {
		for (j = i + 1; j < 15; j++) {
			error = (1U << i) | (1U << j);
			s = bch_syndrome_calc(error);
			bch_syndrome[s] = error;
		}
	}
}

static inline uint32_t calc_golay(uint16_t data)
{
	return golay_table[data & 0xfff];
}

static inline uint16_t calc_bch(uint16_t data)
{
	return bch_table[data & 0x7f];
}

/* Decode a 23-bit Golay(23,12) codeword, correcting up to 3 bit errors.
 * The codeword format is: bits 0-11 = information, bits 12-22 = parity.
 * On success, writes the decoded 12-bit value to *data and returns 0.
 * Returns -1 if the codeword has more than 3 bit errors (uncorrectable). */
int decode_golay(uint32_t codeword, uint16_t *data)
{
	uint32_t syndrome, error_pattern, corrected;

	/* Compute the 11-bit syndrome of the received codeword */
	syndrome = golay_syndrome_calc(codeword);

	/* Syndrome 0 means no errors — codeword is valid */
	if (syndrome == 0) {
		*data = codeword & 0xFFF;
		return 0;
	}

	/* Look up the error pattern for this syndrome */
	error_pattern = golay_syndrome[syndrome];
	if (error_pattern == 0xFFFFFFFF) {
		LOGP(DGOLAY, LOGL_NOTICE, "Golay decode failed: uncorrectable error (syndrome 0x%03x).\n", syndrome);
		return -1;
	}

	/* Apply correction by XORing the error pattern with the received codeword */
	corrected = codeword ^ error_pattern;

	/* Extract the 12 information bits */
	*data = corrected & 0xFFF;

	LOGP(DGOLAY, LOGL_DEBUG, "Golay decode corrected errors (syndrome 0x%03x, pattern 0x%06x).\n", syndrome, error_pattern);

	return 0;
}

/* Decode a 15-bit BCH(15,7) codeword, correcting up to 2 bit errors.
 * The codeword format is: bits 0-6 = information, bits 7-14 = parity.
 * On success, writes the decoded 7-bit value to *data and returns 0.
 * Returns -1 if the codeword has more than 2 bit errors (uncorrectable). */
int decode_bch(uint16_t codeword, uint8_t *data)
{
	uint16_t syndrome, error_pattern, corrected;

	/* Compute the 8-bit syndrome of the received codeword */
	syndrome = bch_syndrome_calc(codeword);

	/* Syndrome 0 means no errors — codeword is valid */
	if (syndrome == 0) {
		*data = codeword & 0x7F;
		return 0;
	}

	/* Look up the error pattern for this syndrome */
	error_pattern = bch_syndrome[syndrome];
	if (error_pattern == 0xFFFF) {
		LOGP(DGOLAY, LOGL_NOTICE, "BCH decode failed: uncorrectable error (syndrome 0x%02x).\n", syndrome);
		return -1;
	}

	/* Apply correction by XORing the error pattern with the received codeword */
	corrected = codeword ^ error_pattern;

	/* Extract the 7 information bits */
	*data = corrected & 0x7F;

	LOGP(DGOLAY, LOGL_DEBUG, "BCH decode corrected errors (syndrome 0x%02x, pattern 0x%04x).\n", syndrome, error_pattern);

	return 0;
}



static const uint16_t preamble_values[] = {
	2030, 1628, 3198,  647,  191, 3315, 1949, 2540, 1560, 2335,
};

static const uint32_t start_code = 713;
const uint32_t activation_code = 2563;

/* Rep. 900-2 Table VI */
static const uint16_t word1s[50] = {
	 721, 2731, 2952, 1387, 1578, 1708, 2650, 1747, 2580, 1376,
	2692,  696, 1667, 3800, 3552, 3424, 1384, 3595,  876, 3124,
	2285, 2608,  899, 3684, 3129, 2124, 1287, 2616, 1647, 3216,
	 375, 1232, 2824, 1840,  408, 3127, 3387,  882, 3468, 3267,
	1575, 3463, 3152, 2572, 1252, 2592, 1552,  835, 1440,  160,
};

/* Rep. 900-2 Table VII (left column) */
static char encode_alpha(char c)
{
	if (c >= 'a' && c <= 'z')
		c = c - 'a' + 'A';
	switch (c) {
	case 0x0a:
	case 0x0d:
		LOGP(DGOLAY, LOGL_DEBUG, " -> CR/LF character.\n");
		c = 0x3c;
		break;
	case '{':
		LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
		c = 0x3b;
		break;
	case '}':
		LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
		c = 0x3d;
		break;
	case '\\':
		LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
		c = 0x20;
		break;
	case '_':
		/* '_' (0x5F) falls outside the standard 0x20-0x5D range;
		 * assigned to the otherwise unused code 0x3F. */
		LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
		c = 0x3f;
		break;
	default:
		if (c < 0x20 || c > 0x5d) {
			LOGP(DGOLAY, LOGL_DEBUG, " -> ' ' character.\n");
			c = 0x20;
		} else {
			LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
			c = c - 0x20;
		}
	}
	return c;
}

/* Rep. 900-2 Table VII (right columns) */
static char encode_numeric(char c)
{
	switch (c) {
	case 'u':
	case 'U':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'U' character.\n");
		c = 0xb;
		break;
	case ' ':
		LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
		c = 0xc;
		break;
	case '-':
		LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
		c = 0xd;
		break;
	case '=':
	case '*':
		LOGP(DGOLAY, LOGL_DEBUG, " -> '*' character.\n");
		c = 0xe;
		break;
	case 'a':
	case 'A':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'A' character.\n");
		c = 0xf0;
		break;
	case 'b':
	case 'B':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'B' character.\n");
		c = 0xf1;
		break;
	case 'c':
	case 'C':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'C' character.\n");
		c = 0xf2;
		break;
	case 'd':
	case 'D':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'D' character.\n");
		c = 0xf3;
		break;
	case 'e':
	case 'E':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'E' character.\n");
		c = 0xf4;
		break;
	case 'f':
	case 'F':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'F' character.\n");
		c = 0xf6;
		break;
	case 'g':
	case 'G':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'G' character.\n");
		c = 0xf7;
		break;
	case 'h':
	case 'H':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'H' character.\n");
		c = 0xf8;
		break;
	case 'j':
	case 'J':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'J' character.\n");
		c = 0xf9;
		break;
	case 'l':
	case 'L':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'L' character.\n");
		c = 0xfb;
		break;
	case 'n':
	case 'N':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'N' character.\n");
		c = 0xfc;
		break;
	case 'p':
	case 'P':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'P' character.\n");
		c = 0xfd;
		break;
	case 'r':
	case 'R':
		LOGP(DGOLAY, LOGL_DEBUG, " -> 'r' character.\n");
		c = 0xfe;
		break;
	default:
		if (c >= '0' && c <= '9') {
			LOGP(DGOLAY, LOGL_DEBUG, " -> '%c' character.\n", c);
			c = c - '0';
		} else {
			LOGP(DGOLAY, LOGL_DEBUG, " -> ' ' character.\n");
			c = 0xc;
		}
	}
	return c;
}

/* Pager Code Assignment Plans (Motorola GSC Standard, Figure 7.1)
 *
 * A GSC functional address is 7 digits: I G1 G0 A2 A1 A0 F
 *   I    = Index digit (0-9), selects preamble: preamble = (I + G0) % 10
 *   G1G0 = Group digits (00-99), selects Word 1 via word1s[G1G0 % 50]
 *   A2A1A0 = Address digits, encoded into Word 2
 *   F    = Function suffix (1-0), determines message type per Function Plan "A"
 *
 * System size depends on the Code Assignment Plan:
 *   Plan A: 50 codes    (1 preamble,  50 Word 1s, 1 address per W1)
 *   Plan B: 500 codes   (1 preamble,  50 Word 1s, 10 addresses per W1)
 *   Plan C: 5,000 codes (10 preambles, 50 Word 1s, 10 addresses per W1)
 *   Plan D: 100,000 codes (10 preambles, 50 Word 1s, 200 addresses per W1)
 *
 * Maximum group call size per plan:
 *   Plan A: 50 pagers   Plan B: 50 pagers
 *   Plan C: 500 pagers  Plan D: 500 pagers
 *
 * Invalid GSC codes (Appendix I):
 *   illegal_low[16]:  W2 values invalid when G1G0 = 00-49
 *   illegal_high[7]:  W2 values invalid when G1G0 = 50-99
 *   These arise from the address arithmetic producing values that
 *   collide with protocol-reserved codewords or cause ambiguity
 *   in the W2-to-address reverse mapping.
 */
static int encode_address(const char *code, int *preamble, uint16_t *word1, uint16_t *word2)
{
	static const uint16_t illegal_low[16] = {   0,  25,  51, 103, 206, 340, 363, 412, 445, 530, 642, 726, 782, 810, 825, 877 };
	static const uint16_t illegal_high[7] = {   0, 292, 425, 584, 631, 841, 851 };
	int idx, g0, g1, a0, a1, a2, ap0, ap, ap1, ap2, ap3, b1b0, b3b2, g1g0, a2a1a0;
	int i;

	for (i = 0; i < 7; i++) {
		if (code[i] < '0' || code[i] > '9')
			break;
	}
	if (code[i]) {
		LOGP(DGOLAY, LOGL_NOTICE, "Invalid functional address character. Only 0..9 are allowed.\n");
		return -EINVAL;
	}

	idx = code[0] - '0';
	g1 = code[1] - '0';
	g0 = code[2] - '0';
	a2 = code[3] - '0';
	a1 = code[4] - '0';
	a0 = code[5] - '0';

	*preamble = (idx + g0) % 10;

	ap = a2 * 200 + a1 * 20 + a0 * 2;
	ap3 = ap / 1000;
	ap2 = (ap / 100) % 10;
	ap1 = (ap / 10) % 10;
	ap0 = ap % 10;

	b1b0 = (ap1 * 10 + ap0) / 2;
	b3b2 = (ap3 * 10 + ap2);

	g1g0 = (g1 * 10 + g0);
	if (g1g0 >= 50) {
		*word1 = word1s[g1g0 - 50];
		*word2 = b3b2 * 100 + b1b0 + 50;
	} else {
		*word1 = word1s[g1g0];
		*word2 = b3b2 * 100 + b1b0;
	}

	a2a1a0 = a2 * 100 + a1 * 10 + a0;
	if (g1g0 < 50) {
		for (i = 0; i < 16; i++) {
			if (a2a1a0 == illegal_low[i])
				break;
		}
		if (i < 16) {
			LOGP(DGOLAY, LOGL_NOTICE, "Functional address has invlid value '%03d' for last three characters.\n", a2a1a0);
			return -EINVAL;
		}
	} else {
		for (i = 0; i < 7; i++) {
			if (a2a1a0 == illegal_high[i])
				break;
		}
		if (i < 7) {
			LOGP(DGOLAY, LOGL_NOTICE, "Functional address has invlid value '%03d' for last three characters.\n", a2a1a0);
			return -EINVAL;
		}
	}

	return 0;
}

/* Reverse-map a decoded W1 value back to group digits (G1, G0).
 *
 * Searches the word1s[50] table (Rep. 900-2 Table VI) for the given W1 value.
 * The table index directly gives g1g0 in the low range (0-49). For the high
 * range (g1g0 50-99), the same table entry is used (index + 50), but the
 * caller must determine high vs low by examining the W2 value.
 *
 * Returns 0 on success (g1 and g0 written), -1 if w1 not found in table. */
int reverse_word1(uint16_t w1, int *g1, int *g0)
{
	int i;

	/* Linear search through the 50-entry word1s table */
	for (i = 0; i < 50; i++) {
		if (word1s[i] == w1) {
			*g1 = i / 10;
			*g0 = i % 10;
			LOGP(DGOLAY, LOGL_DEBUG, "Reverse W1: value %u found at index %d -> G1=%d G0=%d.\n", w1, i, *g1, *g0);
			return 0;
		}
	}

	LOGP(DGOLAY, LOGL_DEBUG, "Reverse W1: value %u not found in word1s table.\n", w1);
	return -1;
}

/* Reverse-map a decoded W2 value back to address digits (A2, A1, A0).
 *
 * This is the exact inverse of the W2 arithmetic in encode_address():
 *   ap = a2*200 + a1*20 + a0*2
 *   ap3 = ap/1000, ap2 = (ap/100)%10, ap1 = (ap/10)%10, ap0 = ap%10
 *   b1b0 = (ap1*10 + ap0) / 2
 *   b3b2 = ap3*10 + ap2
 *   w2 = b3b2*100 + b1b0 [+ 50 if g1g0 >= 50]
 *
 * The reverse steps:
 *   1. If g1g0 >= 50, subtract 50 offset from w2
 *   2. b3b2 = raw / 100, b1b0 = raw % 100
 *   3. Recover ap digits: ap3 = b3b2/10, ap2 = b3b2%10,
 *      ap1 = (b1b0*2)/10, ap0 = (b1b0*2)%10
 *   4. ap = ap3*1000 + ap2*100 + ap1*10 + ap0
 *   5. a2 = ap/200, a1 = (ap/20)%10, a0 = (ap/2)%10
 *   6. Validate a2a1a0 against illegal_low[]/illegal_high[] tables
 *
 * g1g0: the full group digit pair (0-99), used to select high/low range.
 * Returns 0 on success (a2, a1, a0 written), -1 if mapping is invalid. */
int reverse_word2(uint16_t w2, int g1g0, int *a2, int *a1, int *a0)
{
	/* These tables match the ones in encode_address() — duplicated here
	 * because the encoder's copies are local to that function. */
	static const uint16_t illegal_low[16] = {   0,  25,  51, 103, 206, 340, 363, 412, 445, 530, 642, 726, 782, 810, 825, 877 };
	static const uint16_t illegal_high[7] = {   0, 292, 425, 584, 631, 841, 851 };
	int raw, b3b2, b1b0;
	int ap3, ap2, ap1, ap0, ap;
	int a2a1a0;
	int i;

	/* Step 1: Remove the +50 offset for high-range group digits */
	if (g1g0 >= 50)
		raw = w2 - 50;
	else
		raw = w2;

	/* Step 2: Extract b3b2 and b1b0 from the raw W2 value */
	b3b2 = raw / 100;
	b1b0 = raw % 100;

	/* Step 3: Recover the four ap digits */
	ap3 = b3b2 / 10;
	ap2 = b3b2 % 10;
	ap1 = (b1b0 * 2) / 10;
	ap0 = (b1b0 * 2) % 10;

	/* Step 4: Reconstruct the full ap value */
	ap = ap3 * 1000 + ap2 * 100 + ap1 * 10 + ap0;

	/* Step 5: Recover address digits from ap = a2*200 + a1*20 + a0*2 */
	*a2 = ap / 200;
	*a1 = (ap / 20) % 10;
	*a0 = (ap / 2) % 10;

	/* Validate digit range: each must be 0-9 */
	if (*a2 > 9 || *a1 > 9 || *a0 > 9) {
		LOGP(DGOLAY, LOGL_DEBUG, "Reverse W2: digits out of range (A2=%d A1=%d A0=%d).\n", *a2, *a1, *a0);
		return -1;
	}

	/* Step 6: Validate against illegal address tables */
	a2a1a0 = (*a2) * 100 + (*a1) * 10 + (*a0);
	if (g1g0 < 50) {
		for (i = 0; i < 16; i++) {
			if (a2a1a0 == (int)illegal_low[i]) {
				LOGP(DGOLAY, LOGL_DEBUG, "Reverse W2: address %03d is in illegal_low table.\n", a2a1a0);
				return -1;
			}
		}
	} else {
		for (i = 0; i < 7; i++) {
			if (a2a1a0 == (int)illegal_high[i]) {
				LOGP(DGOLAY, LOGL_DEBUG, "Reverse W2: address %03d is in illegal_high table.\n", a2a1a0);
				return -1;
			}
		}
	}

	LOGP(DGOLAY, LOGL_DEBUG, "Reverse W2: w2=%u g1g0=%d -> raw=%d b3b2=%d b1b0=%d ap=%d -> A2=%d A1=%d A0=%d.\n",
		w2, g1g0, raw, b3b2, b1b0, ap, *a2, *a1, *a0);

	return 0;
}

/* Rep. 900-2 Table VII (left column, inverse mapping)
 *
 * Reverse the encode_alpha() mapping: 6-bit code (0x00-0x3F) back to ASCII.
 *
 * The encoder maps ASCII 0x20-0x5D to codes 0x00-0x3D by subtracting 0x20,
 * with special-case overrides for '\\' -> 0x20, CR/LF -> 0x3C,
 * '{' -> 0x3B, and '}' -> 0x3D.  Lowercase letters are uppercased first.
 *
 * Because '\\' and '@' both encode to 0x20, and '{'/'}' collide with
 * '['/']' at codes 0x3B/0x3D, the decoder uses the default arithmetic
 * mapping (code + 0x20) for those codes, recovering '@', '[', and ']'.
 * The '\\', '{', and '}' mappings are lossy in the encoder.
 *
 * Code 0x3C is the CR/LF special case (decoded as CR).
 * Code 0x3E is the NULL fill character used for padding empty slots.
 * Code 0x3F is assigned to '_' (0x5F), which falls outside the standard
 * 0x20-0x5D arithmetic range.
 */
static const char alpha_decode_table[64] = {
	/* 0x00 */ ' ', '!', '"', '#', '$', '%', '&', '\'',
	/* 0x08 */ '(', ')', '*', '+', ',', '-', '.', '/',
	/* 0x10 */ '0', '1', '2', '3', '4', '5', '6', '7',
	/* 0x18 */ '8', '9', ':', ';', '<', '=', '>', '?',
	/* 0x20 */ '@', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
	/* 0x28 */ 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
	/* 0x30 */ 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W',
	/* 0x38 */ 'X', 'Y', 'Z', '[', '\r', ']', '\0', '_',
};

char decode_alpha(uint8_t code)
{
	/* Mask to 6 bits per Rep. 900-2 Table VII */
	code &= 0x3F;
	return alpha_decode_table[code];
}

/* Rep. 900-2 Table VII (right columns, inverse mapping)
 *
 * Reverse the encode_numeric() mapping: 4-bit code (0x0-0xF) back to character.
 *
 * The encoder uses a two-byte scheme for shifted (letter) characters:
 * the first nibble is 0xF (shift prefix), the second nibble is the letter code.
 * The decoder tracks shift state via the caller-provided *shifted flag.
 *
 * numeric_decode_table[16] — unshifted codes:
 *   0x0-0x9: '0'-'9'
 *   0xA:     '\0' (NULL fill, used for padding empty digit slots)
 *   0xB:     'U'
 *   0xC:     ' ' (space)
 *   0xD:     '-' (hyphen)
 *   0xE:     '*' (asterisk; encoder also accepts '=' as alias)
 *   0xF:     '\0' (shift prefix — sets shifted mode, no character emitted)
 *
 * numeric_shift_table[16] — codes after 0xF prefix:
 *   0x0: 'A', 0x1: 'B', 0x2: 'C', 0x3: 'D', 0x4: 'E',
 *   0x5: '?' (unused), 0x6: 'F', 0x7: 'G', 0x8: 'H', 0x9: 'J',
 *   0xA: '?' (unused), 0xB: 'L', 0xC: 'N', 0xD: 'P', 0xE: 'R',
 *   0xF: '?' (unused)
 *
 * Note: gaps at 0x5, 0xA, 0xF in the shift table correspond to letters
 * (I, K, Q) that are omitted from the GSC numeric character set.
 */
static const char numeric_decode_table[16] = {
	/* 0x0 */ '0', '1', '2', '3', '4', '5', '6', '7',
	/* 0x8 */ '8', '9', '\0', 'U', ' ', '-', '*', '\0',
};

static const char numeric_shift_table[16] = {
	/* 0x0 */ 'A', 'B', 'C', 'D', 'E', '?', 'F', 'G',
	/* 0x8 */ 'H', 'J', '?', 'L', 'N', 'P', 'R', '?',
};

/* Decode a single 4-bit numeric code, handling the shift prefix mechanism.
 *
 * The *shifted flag tracks whether the previous code was the 0xF shift prefix:
 *   - If *shifted == 0 and code == 0xF: set *shifted = 1, return '\0'
 *     (shift prefix consumed, no character produced yet)
 *   - If *shifted == 0 and code != 0xF: return numeric_decode_table[code]
 *   - If *shifted == 1: return numeric_shift_table[code], reset *shifted = 0
 *
 * Returns the decoded character, or '\0' if the code is a shift prefix
 * or NULL fill (0xA). */
char decode_numeric(uint8_t code, int *shifted)
{
	char c;

	/* Mask to 4 bits per Rep. 900-2 Table VII numeric encoding */
	code &= 0x0F;

	if (*shifted) {
		/* Previous nibble was 0xF shift prefix — decode shifted letter */
		c = numeric_shift_table[code];
		*shifted = 0;
		LOGP(DGOLAY, LOGL_DEBUG, "Decode numeric (shifted): 0x%x -> '%c'.\n", code, c);
		return c;
	}

	if (code == 0x0F) {
		/* Shift prefix: consume this nibble, next nibble is the letter code */
		*shifted = 1;
		LOGP(DGOLAY, LOGL_DEBUG, "Decode numeric: 0xF shift prefix.\n");
		return '\0';
	}

	c = numeric_decode_table[code];
	LOGP(DGOLAY, LOGL_DEBUG, "Decode numeric: 0x%x -> '%c'.\n", code, c);
	return c;
}

/* Score the plausibility of an alpha interpretation.
 * Evaluates each character in the decoded alpha string against
 * the GSC alpha character set (Rep. 900-2 Table VII).
 *
 * Scoring per character:
 *   +5  printable alphanumeric (A-Z, 0-9, space)
 *   -2  special/punctuation (other printable GSC alpha chars)
 *   -5  non-printable, control, or outside valid GSC alpha table
 *   +2*fill^2  quadratic fill bonus (strong for many fill chars)
 *
 * Fill characters (decoded from 6-bit code 0x3E, which maps to '\0')
 * are excluded from per-character scoring but contribute via the
 * fill bonus.
 *
 * Returns the total score (sum of per-character contributions). */
int gsc_score_alpha(const char *str, int len, int fill)
{
	int score = 0;
	int i;

	for (i = 0; i < len; i++) {
		char c = str[i];

		/* Exclude fill characters (decode_alpha(0x3E) == '\0') */
		if (c == '\0')
			continue;

		if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ' ') {
			/* Alphanumeric or space: +3 */
			score += 3;
		} else if (c >= 0x20 && c <= 0x7E) {
			/* Other printable ASCII (special/punctuation): -2 */
			score -= 2;
		} else {
			/* Non-printable, control, or unmapped: -5 */
			score -= 5;
		}
	}

	/* Fill characters are strong evidence this is the correct type:
	 * the encoder pads unused 6-bit slots with 0x3E. Coincidental
	 * fill in the wrong interpretation is very rare (1/64 per slot).
	 * Bonus scales quadratically: a few fill chars are moderate
	 * evidence, many fill chars are overwhelming evidence. */
	score += fill * fill * 2;

	return score;
}

/* Score the plausibility of a numeric interpretation.
 * Evaluates each nibble in the decoded numeric string against
 * the GSC numeric character set.
 *
 * Scoring per nibble (operates on the raw 4-bit nibble values,
 * not the decoded ASCII, to distinguish shift prefixes):
 *   +3  digit (nibble 0x0–0x9)
 *  -15  'U' (nibble 0xB) — very rare in real numeric messages;
 *       common artifact when alpha data is misinterpreted as numeric
 *   -2  space (0xC), hyphen (0xD), asterisk (0xE)
 *   -2  shift prefix (0xF) — uncommon in numeric messages
 *   +fill^2  quadratic fill bonus (weaker than alpha fill since
 *            coincidental 0x0A nibbles are more likely: 1/16 vs 1/64)
 *
 * Fill nibbles (0x0A) are excluded from per-nibble scoring but
 * contribute via the fill bonus.
 *
 * Returns the total score (sum of per-nibble contributions). */
int gsc_score_numeric(const uint8_t *nibbles, int count, int fill)
{
	int score = 0;
	int i;

	for (i = 0; i < count; i++) {
		uint8_t n = nibbles[i];

		/* Exclude fill nibbles */
		if (n == 0x0A)
			continue;

		if (n <= 0x09) {
			/* Digit (0-9): +3 */
			score += 3;
		} else if (n == 0x0B) {
			/* 'U': -15 (very rare in real numeric messages;
			 * common artifact when alpha data is misinterpreted
			 * as numeric due to nibble 0x0B appearing in random
			 * bit patterns) */
			score -= 15;
		} else if (n >= 0x0C && n <= 0x0E) {
			/* Space (0xC), hyphen (0xD), asterisk (0xE): -2 */
			score -= 2;
		} else if (n == 0x0F) {
			/* Shift prefix: -2 (uncommon in numeric messages) */
			score -= 2;
		}
	}

	/* Fill nibbles are evidence this is the correct type, but weaker
	 * than alpha fill since coincidental 0x0A nibbles are more likely
	 * (1/16 per nibble vs 1/64 for alpha fill). Bonus scales
	 * quadratically: a few fill nibbles are weak evidence (could be
	 * coincidental), many fill nibbles are strong evidence. */
	score += fill * fill;

	return score;
}

/* Determine the winning interpretation using fill-count priority
 * with content-score tiebreaker.
 *
 * Decision logic:
 *   1) If one interpretation has trailing fill and the other doesn't,
 *      the one with fill wins (fill is inserted by the encoder to pad
 *      short messages — its presence is a strong structural signal).
 *   2) If both have fill, the one with more fill wins (shorter message
 *      = more fill = stronger signal).
 *   3) If neither has fill (full block), compare content scores —
 *      higher score wins.
 *
 * Sets msg->type to the winner, populates score/method fields.
 * Sets msg->guess_uncertain if |alpha_score - numeric_score| < threshold. */
void gsc_discriminate(gsc_rx_msg_t *msg)
{
	int score_diff;

	/* Determine uncertainty based on score difference */
	score_diff = msg->alpha_score - msg->numeric_score;
	if (score_diff < 0)
		score_diff = -score_diff;
	msg->guess_uncertain = (score_diff < GSC_GUESS_UNCERTAIN_THRESHOLD) ? 1 : 0;

	/* Compare unified scores (content + fill are already folded in) */
	msg->guess_method = 1; /* content-score (unified) */

	if (msg->alpha_score > msg->numeric_score) {
		msg->guess_winner = TYPE_ALPHA;
	} else if (msg->numeric_score > msg->alpha_score) {
		msg->guess_winner = TYPE_NUMERIC;
	} else {
		/* Scores equal — default to alpha, flag uncertain */
		msg->guess_winner = TYPE_ALPHA;
		msg->guess_uncertain = 1;
	}

	/* Set backward-compatible fields */
	msg->type = msg->guess_winner;
	if (msg->guess_winner == TYPE_ALPHA)
		strcpy(msg->data, msg->alpha_data);
	else
		strcpy(msg->data, msg->numeric_data);
}

/* Read a duplicate-transmitted Golay codeword from the bitstream.
 * The encoder's queue_dup() transmits each of the 23 codeword bits twice
 * (LSB first), producing 46 bits total. This function reads those 46 bits,
 * resolves each pair to a single bit:
 *   - If both bits agree, use that value.
 *   - If they disagree, prefer the first bit (the Golay error corrector
 *     handles any residual errors).
 * Advances *pos by 46 and returns the resolved 23-bit codeword. */
static uint32_t read_dup_golay(const uint8_t *bits, int *pos)
{
	uint32_t codeword = 0;
	int p = *pos;
	int i;

	for (i = 0; i < 23; i++) {
		uint8_t bit1 = bits[p];
		uint8_t bit2 = bits[p + 1];
		uint8_t resolved;

		if (bit1 == bit2) {
			resolved = bit1;
		} else {
			/* Bits disagree — prefer first bit, log for diagnostics */
			resolved = bit1;
			LOGP(DGOLAY, LOGL_DEBUG, "Duplicate bit disagreement at bit %d (pos %d): %d vs %d, using %d.\n",
				i, p, bit1, bit2, resolved);
		}

		codeword |= ((uint32_t)resolved << i);
		p += 2;
	}

	*pos = p;

	LOGP(DGOLAY, LOGL_DEBUG, "Read duplicate Golay codeword: 0x%06x (from pos %d, 46 bits).\n",
		codeword, *pos - 46);

	return codeword;
}

/* De-interleave 8 BCH(15,7) codewords from the bitstream.
 *
 * This is the exact inverse of the encoder's interleaving loop in queue_batch():
 *
 *   for (j = 0; j < 15; j++)
 *       for (k = 0; k < 8; k++)
 *           queue_bit(gsc, (bch[k] >> j) & 1);
 *
 * The encoder writes 120 bits total (15 bit-positions × 8 codewords).
 * For each bit position j (0..14), it writes bit j of codewords k=0..7
 * in order.  We reverse this by reading in the same j,k order and
 * reconstructing each codeword bit-by-bit.
 *
 * Parameters:
 *   bits  - raw bitstream array
 *   pos   - pointer to current read position; advanced by 120 on return
 *   bch   - output array of 8 reconstructed 15-bit BCH codewords
 */
static void deinterleave_bch(const uint8_t *bits, int *pos, uint16_t bch[8])
{
	int p = *pos;
	int j, k;

	/* 15 = BCH codeword length, 8 = number of interleaved codewords per block */
	const int bch_codeword_bits = 15;
	const int bch_codewords_per_block = 8;

	/* zero output codewords before OR-ing in bits */
	for (k = 0; k < bch_codewords_per_block; k++)
		bch[k] = 0;

	/* reverse the encoder's interleaving: for each bit position j,
	 * read bit j of each codeword k in sequence */
	for (j = 0; j < bch_codeword_bits; j++) {
		for (k = 0; k < bch_codewords_per_block; k++) {
			bch[k] |= (uint16_t)(bits[p] & 1) << j;
			p++;
		}
	}

	LOGP(DGOLAY, LOGL_DEBUG, "De-interleaved %d BCH codewords from pos %d (%d bits).\n",
		bch_codewords_per_block, *pos, bch_codeword_bits * bch_codewords_per_block);

	*pos = p;
}


/* Decode a complete GSC batch from the TX bit buffer.
 *
 * This is the exact inverse of queue_batch(). It walks the bitstream in
 * gsc->bit[] and reconstructs the functional address, message type, and
 * message content into the provided gsc_rx_msg_t structure.
 *
 * Bitstream layout (produced by queue_batch()):
 *   [28-bit comma] [18 x dup(golay(preamble), 23)]
 *   [28-bit comma] [dup(golay(713), 23)] [1-bit inv] [dup(~golay(713), 23)]
 *   [28-bit comma] [dup(golay(W1)^inv, 23)] [1-bit inv] [dup(golay(W2)^inv, 23)]
 *   {message payload -- varies by type}
 *
 * Returns 0 on success, -1 on decode failure.
 *
 * When 'force' is non-zero, the decoder reports whatever it managed to
 * decode (partial results) instead of returning -1 when the message
 * type cannot be determined from insufficient post-address bits. This
 * is used when the caller knows no more bits will arrive (timeout or
 * end-of-transmission detected). */
int decode_batch(gsc_t *gsc, gsc_rx_msg_t *msg, int force)
{
	const uint8_t *bits = gsc->bit;
	int pos = 0;
	int total_bits = gsc->bit_num;

	/* Protocol constants (matching encoder's hardcoded values) */
	const int comma_len = 28;		/* comma sequence length */
	const int preamble_reps = 18;		/* preamble codeword repetitions */
	const int dup_bits = 46;		/* duplicate-transmitted Golay codeword (23 x 2) */
	const int bch_block_bits = 120;		/* interleaved BCH block (15 x 8) */
	const int tone_comma_len = 121 * 8;	/* tone-only comma (968 bits) */

	uint32_t codeword;
	uint16_t decoded_value;
	int rc, i;

	/* Preamble majority-vote tracking */
	uint16_t preamble_votes[10];
	int preamble_idx = -1;
	int best_count;

	/* Address decoding variables */
	uint16_t w1_value, w2_value;
	int w1_inverted, w2_inverted;
	uint8_t function;
	int g1, g0, a2, a1, a0;
	int g1g0, idx;
	char suffix;
	int remaining;
	enum gsc_msg_type detected_type;

	/* Data block decoding variables */
	uint16_t bch_cw[8];
	uint8_t d[8];
	uint8_t checksum;
	uint8_t contbit;
	int shifted;
	char c;

	memset(msg, 0, sizeof(*msg));

	LOGP(DGOLAY, LOGL_DEBUG, "Decoding batch: %d bits in buffer.\n", total_bits);

	/* ================================================================
	 * Stage 1: PREAMBLE
	 *
	 * Encoder: queue_comma(gsc, 28, golay & 1)
	 *          for (i = 0; i < 18; i++) queue_dup(gsc, golay, 23)
	 *
	 * Layout: 28-bit comma + 18 x 46-bit dup Golay = 856 bits
	 * ================================================================ */

	if (pos + comma_len + preamble_reps * dup_bits > total_bits) {
		LOGP(DGOLAY, LOGL_NOTICE, "Not enough bits for preamble (%d available, %d needed).\n",
			total_bits - pos, comma_len + preamble_reps * dup_bits);
		return -1;
	}

	/* Skip 28-bit comma sequence */
	pos += comma_len;

	/* Read 18 duplicate Golay codewords and majority-vote the preamble */
	memset(preamble_votes, 0, sizeof(preamble_votes));

	for (i = 0; i < preamble_reps; i++) {
		int j;

		codeword = read_dup_golay(bits, &pos);
		rc = decode_golay(codeword, &decoded_value);
		if (rc < 0) {
			LOGP(DGOLAY, LOGL_DEBUG, "Preamble rep %d: Golay decode failed, skipping.\n", i);
			msg->error_count++;
			continue;
		}

		/* Match decoded value against preamble_values[0..9] */
		for (j = 0; j < 10; j++) {
			if (decoded_value == preamble_values[j]) {
				preamble_votes[j]++;
				break;
			}
		}
		if (j == 10) {
			LOGP(DGOLAY, LOGL_DEBUG, "Preamble rep %d: value %u not in preamble table.\n",
				i, decoded_value);
			msg->error_count++;
		}
	}

	/* Select preamble index with the most votes */
	best_count = 0;
	for (i = 0; i < 10; i++) {
		if (preamble_votes[i] > best_count) {
			best_count = preamble_votes[i];
			preamble_idx = i;
		}
	}

	if (preamble_idx < 0) {
		LOGP(DGOLAY, LOGL_DEBUG, "Preamble detection failed: no valid codewords decoded.\n");
		return -1;
	}

	msg->preamble_index = preamble_idx;
	LOGP(DGOLAY, LOGL_DEBUG, "Preamble detected: index %d (value %u, %d/%d votes).\n",
		preamble_idx, preamble_values[preamble_idx], best_count, preamble_reps);

	/* ================================================================
	 * Stage 2: START CODE
	 *
	 * Encoder: golay = calc_golay(start_code)
	 *          queue_comma(gsc, 28, golay & 1)
	 *          queue_dup(gsc, golay, 23)
	 *          golay ^= 0x7fffff
	 *          queue_bit(gsc, (golay & 1) ^ 1)
	 *          queue_dup(gsc, golay, 23)
	 *
	 * Layout: 28 + 46 + 1 + 46 = 121 bits
	 * ================================================================ */

	if (pos + comma_len + dup_bits + 1 + dup_bits > total_bits) {
		LOGP(DGOLAY, LOGL_NOTICE, "Not enough bits for start code.\n");
		return -1;
	}

	/* Skip 28-bit comma */
	pos += comma_len;

	/* Read and verify first codeword (normal start code = 713) */
	codeword = read_dup_golay(bits, &pos);
	rc = decode_golay(codeword, &decoded_value);
	if (rc < 0) {
		LOGP(DGOLAY, LOGL_NOTICE, "Start code: first Golay decode failed.\n");
		return -1;
	}
	if (decoded_value != start_code) {
		LOGP(DGOLAY, LOGL_NOTICE, "Start code: expected %u, got %u.\n",
			(unsigned)start_code, decoded_value);
		return -1;
	}

	/* Skip 1-bit inverted comma */
	pos += 1;

	/* Read and verify complement codeword (~start_code) */
	codeword = read_dup_golay(bits, &pos);
	rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
	if (rc < 0) {
		LOGP(DGOLAY, LOGL_NOTICE, "Start code: complement Golay decode failed.\n");
		msg->error_count++;
	} else if (decoded_value != start_code) {
		LOGP(DGOLAY, LOGL_NOTICE, "Start code: complement decoded to %u, expected %u.\n",
			decoded_value, (unsigned)start_code);
		msg->error_count++;
	}

	LOGP(DGOLAY, LOGL_DEBUG, "Start code verified (value %u).\n", (unsigned)start_code);

	/* ================================================================
	 * Stage 3: ADDRESS
	 *
	 * Encoder: golay = calc_golay(word1)
	 *          if (function & 0x2) golay ^= 0x7fffff
	 *          queue_comma(gsc, 28, golay & 1)
	 *          queue_dup(gsc, golay, 23)
	 *          golay = calc_golay(word2)
	 *          if (function & 0x1) golay ^= 0x7fffff
	 *          queue_bit(gsc, (golay & 1) ^ 1)
	 *          queue_dup(gsc, golay, 23)
	 *
	 * Layout: 28 + 46 + 1 + 46 = 121 bits
	 *
	 * Inversion detection: try decode_golay on raw codeword; if the
	 * decoded value is in word1s[], W1 is NOT inverted. Otherwise
	 * try decode_golay on (codeword ^ 0x7FFFFF).
	 * ================================================================ */

	if (pos + comma_len + dup_bits + 1 + dup_bits > total_bits) {
		LOGP(DGOLAY, LOGL_NOTICE, "Not enough bits for address.\n");
		return -1;
	}

	/* Skip 28-bit comma */
	pos += comma_len;

	/* --- Decode W1 with inversion detection --- */
	codeword = read_dup_golay(bits, &pos);
	w1_inverted = 0;

	rc = decode_golay(codeword, &decoded_value);
	if (rc == 0) {
		/* Check if decoded value is a valid W1 (in word1s[] table) */
		int found = 0;
		for (i = 0; i < 50; i++) {
			if (decoded_value == word1s[i]) {
				found = 1;
				break;
			}
		}
		if (found) {
			w1_value = decoded_value;
		} else {
			/* Decoded OK but not in table -- must be inverted */
			rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
			if (rc == 0) {
				w1_value = decoded_value;
				w1_inverted = 1;
			} else {
				LOGP(DGOLAY, LOGL_NOTICE, "W1: normal not in table, complement decode failed.\n");
				return -1;
			}
		}
	} else {
		/* Normal decode failed -- try complement */
		rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
		if (rc == 0) {
			w1_value = decoded_value;
			w1_inverted = 1;
		} else {
			LOGP(DGOLAY, LOGL_NOTICE, "W1: both normal and complement decode failed.\n");
			return -1;
		}
	}

	LOGP(DGOLAY, LOGL_DEBUG, "W1 decoded: %u (inverted=%d).\n", w1_value, w1_inverted);

	/* Reverse-map W1 -> G1, G0 (needed before W2 decode for range validation) */
	rc = reverse_word1(w1_value, &g1, &g0);
	if (rc < 0) {
		LOGP(DGOLAY, LOGL_NOTICE, "Address: W1 value %u not in word1s table.\n", w1_value);
		return -1;
	}
	g1g0 = g1 * 10 + g0;

	/* Skip 1-bit inverted comma */
	pos += 1;

	/* --- Decode W2 with inversion and range detection ---
	 *
	 * Three ambiguities must be resolved simultaneously:
	 *
	 * 1) Inversion: the encoder XORs the Golay codeword with 0x7FFFFF
	 *    when function bit 0 is set. Both the normal and complement
	 *    Golay decodes succeed (complement of a valid codeword is valid),
	 *    so we can't tell from Golay alone.
	 *
	 * 2) High/low range: reverse_word1() always returns g1g0 in the
	 *    low range (0-49). If the actual g1g0 is 50-99, the same W1
	 *    value is used (word1s[g1g0-50]). The W2 encoding differs by
	 *    a +50 offset for high range.
	 *
	 * 3) Both: we must try all 4 combinations of (normal/complement)
	 *    × (low/high range) and use reverse_word2() success as the
	 *    discriminator.
	 *
	 * If exactly one combination succeeds, use it. If multiple succeed
	 * (rare but possible), prefer non-inverted and low-range. */
	codeword = read_dup_golay(bits, &pos);
	w2_inverted = 0;

	{
		uint16_t w2_try[2] = { 0, 0 };	/* [0]=normal, [1]=complement */
		int golay_ok[2] = { 0, 0 };
		int try_g1g0[2];			/* [0]=low, [1]=high */
		int best_inv = -1, best_range = -1;
		int ta2, ta1, ta0;
		int inv, rng;

		try_g1g0[0] = g1g0;
		try_g1g0[1] = g1g0 + 50;

		rc = decode_golay(codeword, &decoded_value);
		if (rc == 0) { w2_try[0] = decoded_value; golay_ok[0] = 1; }

		rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
		if (rc == 0) { w2_try[1] = decoded_value; golay_ok[1] = 1; }

		/* Try all 4 combinations: inv={0,1} × range={0,1} */
		for (inv = 0; inv < 2; inv++) {
			if (!golay_ok[inv])
				continue;
			for (rng = 0; rng < 2; rng++) {
				if (reverse_word2(w2_try[inv], try_g1g0[rng], &ta2, &ta1, &ta0) == 0) {
					if (best_inv < 0) {
						best_inv = inv;
						best_range = rng;
						a2 = ta2; a1 = ta1; a0 = ta0;
					}
					LOGP(DGOLAY, LOGL_DEBUG, "W2 candidate: value=%u inv=%d g1g0=%d -> A2=%d A1=%d A0=%d.\n",
						w2_try[inv], inv, try_g1g0[rng], ta2, ta1, ta0);
				}
			}
		}

		if (best_inv < 0) {
			LOGP(DGOLAY, LOGL_NOTICE, "W2: no valid address from any combination.\n");
			return -1;
		}

		w2_value = w2_try[best_inv];
		w2_inverted = best_inv;

		/* Update g1g0 if high range was selected */
		if (best_range == 1) {
			g1g0 = try_g1g0[1];
			g1 = g1g0 / 10;
			g0 = g1g0 % 10;
		}
	}

	LOGP(DGOLAY, LOGL_DEBUG, "W2 decoded: %u (inverted=%d, g1g0=%d).\n", w2_value, w2_inverted, g1g0);

	/* Function variable from inversion pattern:
	 *   bit 1 = W1 inverted, bit 0 = W2 inverted */
	function = (w1_inverted ? 0x2 : 0) | (w2_inverted ? 0x1 : 0);

	/* Index digit: encoder sets preamble = (idx + g0) % 10,
	 * so idx = (preamble - g0 + 10) % 10 */
	idx = (preamble_idx - g0 + 10) % 10;

	/* Detect message type by examining what follows the address.
	 *
	 * The three message types produce distinct bitstream signatures:
	 *   Voice:  28-bit comma + Golay pair (activation_code = 2563) = 121 bits
	 *   Data:   1-bit inverted comma + 120 interleaved BCH bits   = 121 bits
	 *   Tone:   968-bit comma sequence (121 * 8)
	 *
	 * Strategy: peek ahead for activation code to detect voice,
	 * then use remaining bit count to distinguish data vs tone.
	 * The encoder produces exact bit counts, so remaining == 968
	 * unambiguously identifies tone-only.
	 *
	 * If there aren't enough remaining bits to distinguish the type
	 * (minimum 121 for any post-address content), we cannot determine
	 * the message type yet — return -1 so the caller can buffer more
	 * bits and retry. */
	remaining = total_bits - pos;

	/* Type detection strategy:
	 *
	 * The three message types produce distinct post-address signatures:
	 *   Voice:  28-bit comma + dup Golay pair (activation_code) = 121 bits
	 *   Data:   1-bit inverted comma + 120 interleaved BCH bits = 121 bits/block
	 *   Tone:   968-bit comma sequence (121 * 8)
	 *
	 * The periodic decode fires every 46 bits, so we may attempt type
	 * detection before the full tone comma (968 bits) has arrived.
	 * With only ~144 remaining bits, we can't distinguish "tone comma
	 * still being received" from "alpha data block" by bit count alone.
	 *
	 * Solution: probe the first BCH block. If all 8 codewords decode
	 * and the checksum matches, it's data (~1/128 false positive rate).
	 * If the probe fails and we don't have 968+ bits yet, return -1
	 * (need more data) unless force=1. */
	if (remaining < 1 + bch_block_bits) {
		if (!force) {
			LOGP(DGOLAY, LOGL_DEBUG, "Not enough post-address bits to determine type (%d remaining), need more data.\n", remaining);
			return -1;
		}
		LOGP(DGOLAY, LOGL_NOTICE, "Forced decode: only %d post-address bits, cannot determine message type.\n", remaining);
		detected_type = TYPE_TONE;
	} else {
		detected_type = TYPE_TONE;

		/* Check for voice: peek for activation code after 28-bit comma */
		if (remaining >= comma_len + dup_bits + 1 + dup_bits) {
			int peek_pos = pos + comma_len;
			uint32_t peek_cw = read_dup_golay(bits, &peek_pos);
			uint16_t peek_val;
			if (decode_golay(peek_cw, &peek_val) == 0 && peek_val == activation_code)
				detected_type = TYPE_VOICE;
		}

		/* If not voice, distinguish data blocks from tone-only.
		 *
		 * Probe the first BCH block: skip 1-bit inverted comma,
		 * de-interleave 120 bits, decode all 8 BCH codewords, and
		 * verify the checksum. A valid checksum is strong evidence
		 * of a data block. */
		if (detected_type != TYPE_VOICE) {
			int data_detected = 0;

			if (remaining >= 1 + bch_block_bits) {
				/* Probe the first BCH block without advancing pos */
				int probe_pos = pos + 1; /* skip 1-bit inverted comma */
				uint16_t probe_bch[8];
				uint8_t probe_d[8];
				uint8_t probe_cksum;
				int probe_ok = 1;
				int pk;

				deinterleave_bch(bits, &probe_pos, probe_bch);
				for (pk = 0; pk < 8; pk++) {
					if (decode_bch(probe_bch[pk], &probe_d[pk]) < 0) {
						probe_ok = 0;
						break;
					}
				}

				if (probe_ok) {
					probe_cksum = 0;
					for (pk = 0; pk < 7; pk++)
						probe_cksum += calc_bch(probe_d[pk]);
					probe_cksum &= 0x7f;

					if (probe_cksum == probe_d[7]) {
						data_detected = 1;

						/* Distinguish alpha from numeric.
						 *
						 * Both use identical BCH encoding, so the
						 * checksum can't tell them apart. The only
						 * difference is data packing:
						 *   Alpha:   8 six-bit chars from 7 data words
						 *   Numeric: 12 four-bit nibbles from 7 data words
						 *
						 * Strategy: unpack the probe block both ways
						 * and count fill characters. The encoder pads
						 * unused slots with fill values:
						 *   Alpha fill:   0x3E (6-bit code)
						 *   Numeric fill:  0x0A (4-bit nibble)
						 *
						 * A block with trailing fill chars in one
						 * scheme but not the other is a strong signal.
						 * For full blocks (no fill), we default to
						 * alpha here — the final type decision is
						 * made by gsc_discriminate() after both
						 * decode stages run with content scoring. */
						{
							uint8_t a_ch[8], n_nib[12];
							int a_fill = 0, n_fill = 0;
							int nk;

							/* Alpha unpack (6-bit) */
							a_ch[0] = probe_d[0] & 0x3f;
							a_ch[1] = ((probe_d[0] >> 6) | (probe_d[1] << 1)) & 0x3f;
							a_ch[2] = ((probe_d[1] >> 5) | (probe_d[2] << 2)) & 0x3f;
							a_ch[3] = ((probe_d[2] >> 4) | (probe_d[3] << 3)) & 0x3f;
							a_ch[4] = ((probe_d[3] >> 3) | (probe_d[4] << 4)) & 0x3f;
							a_ch[5] = ((probe_d[4] >> 2) | (probe_d[5] << 5)) & 0x3f;
							a_ch[6] = (probe_d[5] >> 1) & 0x3f;
							a_ch[7] = probe_d[6] & 0x3f;

							/* Numeric unpack (4-bit) */
							n_nib[0]  = probe_d[0] & 0xf;
							n_nib[1]  = ((probe_d[0] >> 4) | (probe_d[1] << 3)) & 0xf;
							n_nib[2]  = (probe_d[1] >> 1) & 0xf;
							n_nib[3]  = ((probe_d[1] >> 5) | (probe_d[2] << 2)) & 0xf;
							n_nib[4]  = (probe_d[2] >> 2) & 0xf;
							n_nib[5]  = ((probe_d[2] >> 6) | (probe_d[3] << 1)) & 0xf;
							n_nib[6]  = (probe_d[3] >> 3) & 0xf;
							n_nib[7]  = probe_d[4] & 0xf;
							n_nib[8]  = ((probe_d[4] >> 4) | (probe_d[5] << 3)) & 0xf;
							n_nib[9]  = (probe_d[5] >> 1) & 0xf;
							n_nib[10] = ((probe_d[5] >> 5) | (probe_d[6] << 2)) & 0xf;
							n_nib[11] = (probe_d[6] >> 2) & 0xf;

							/* Count alpha fill chars (0x3E) */
							for (nk = 7; nk >= 0; nk--) {
								if (a_ch[nk] == 0x3e)
									a_fill++;
								else
									break;
							}

							/* Count numeric fill nibbles (0x0A) */
							for (nk = 11; nk >= 0; nk--) {
								if (n_nib[nk] == 0x0a)
									n_fill++;
								else
									break;
							}

							LOGP(DGOLAY, LOGL_DEBUG, "BCH probe type: alpha_fill=%d, numeric_fill=%d.\n",
								a_fill, n_fill);

							/* Decision logic:
							 * 1) If one has fill and the other doesn't,
							 *    the one with fill is the correct type.
							 * 2) If both have fill, prefer the one with
							 *    more fill (shorter message).
							 * 3) If neither has fill (full block), default
							 *    to alpha — gsc_discriminate() will make
							 *    the final type decision using content
							 *    scoring after both decode stages run. */
							if (n_fill > 0 && a_fill == 0) {
								detected_type = TYPE_NUMERIC;
							} else if (a_fill > 0 && n_fill == 0) {
								detected_type = TYPE_ALPHA;
							} else if (n_fill > a_fill) {
								detected_type = TYPE_NUMERIC;
							} else if (a_fill > n_fill) {
								detected_type = TYPE_ALPHA;
							} else {
								detected_type = TYPE_ALPHA;
							}
						}

						LOGP(DGOLAY, LOGL_DEBUG, "BCH probe: valid checksum, detected %s message.\n",
							detected_type == TYPE_NUMERIC ? "numeric" : "alpha");
					} else {
						LOGP(DGOLAY, LOGL_DEBUG, "BCH probe: checksum mismatch (0x%02x != 0x%02x).\n",
							probe_cksum, probe_d[7]);
					}
				} else {
					LOGP(DGOLAY, LOGL_DEBUG, "BCH probe: decode failed, not a data block.\n");
				}
			}

			/* If BCH probe didn't identify data and we don't have
			 * enough bits for a full tone comma yet, we can't tell —
			 * request more data unless forced. */
			if (!data_detected && remaining < tone_comma_len) {
				if (!force) {
					LOGP(DGOLAY, LOGL_DEBUG, "Type ambiguous: %d post-address bits (need %d for tone), waiting for more data.\n",
						remaining, tone_comma_len);
					return -1;
				}
				LOGP(DGOLAY, LOGL_NOTICE, "Forced decode: %d post-address bits, defaulting to tone-only.\n", remaining);
				detected_type = TYPE_TONE;
			}
		}
	}

	LOGP(DGOLAY, LOGL_INFO, "Message type detected: %s (%d post-address bits).\n",
		detected_type == TYPE_VOICE ? "voice" :
		detected_type == TYPE_ALPHA ? "alpha" :
		detected_type == TYPE_NUMERIC ? "numeric" : "tone",
		remaining);

	/* Assign function suffix per Function Plan "A":
	 *   Voice:   function 0-3 -> suffix 1-4
	 *   Alpha:   function 0-3 -> suffix 5-8
	 *   Tone:    function 0   -> suffix 9, function 1 -> suffix 0 */
	switch (detected_type) {
	case TYPE_VOICE:
		suffix = '1' + function;
		break;
	case TYPE_ALPHA:
	case TYPE_NUMERIC:
		suffix = '5' + function;
		break;
	default: /* TYPE_TONE */
		suffix = (function == 0) ? '9' : '0';
		break;
	}

	/* Build the 7-digit functional address: I G1 G0 A2 A1 A0 suffix */
	msg->address[0] = '0' + idx;
	msg->address[1] = '0' + g1;
	msg->address[2] = '0' + g0;
	msg->address[3] = '0' + a2;
	msg->address[4] = '0' + a1;
	msg->address[5] = '0' + a0;
	msg->address[6] = suffix;
	msg->address[7] = '\0';

	msg->address_number = function + 1;
	msg->type = detected_type;

	LOGP(DGOLAY, LOGL_INFO, "Address decoded: '%s' (number %d, function %d, type %s).\n",
		msg->address, msg->address_number, function,
		detected_type == TYPE_VOICE ? "voice" :
		detected_type == TYPE_ALPHA ? "alpha" :
		detected_type == TYPE_NUMERIC ? "numeric" : "tone");

	/* ================================================================
	 * Stages 4 & 5: DUAL-DECODE DATA BLOCKS
	 *
	 * Both alpha and numeric decode stages run unconditionally when
	 * data is detected (TYPE_ALPHA or TYPE_NUMERIC from the BCH probe).
	 * Stage 4 populates msg->alpha_data[] and counts msg->alpha_fill.
	 * Stage 5 populates msg->numeric_data[], msg->numeric_nibbles[],
	 * msg->numeric_nibble_count, and counts msg->numeric_fill.
	 * The bitstream position is saved before Stage 4 and restored
	 * before Stage 5 so both decode the same data blocks.
	 * ================================================================ */

	if (detected_type == TYPE_ALPHA || detected_type == TYPE_NUMERIC) {
		int data_start_pos = pos; /* save position for numeric re-decode */
		int alpha_need_bits = 0;   /* 1 = alpha loop broke due to insufficient bits */
		int numeric_need_bits = 0; /* 1 = numeric loop broke due to insufficient bits */

		/* ============================================================
		 * Stage 4: ALPHA DATA BLOCKS
		 *
		 * Encoder per block:
		 *   queue_bit(gsc, (bch[0] & 1) ^ 1)   [1-bit inverted comma]
		 *   for (j=0; j<15; j++)
		 *     for (k=0; k<8; k++)
		 *       queue_bit(gsc, (bch[k] >> j) & 1)  [120 interleaved bits]
		 *
		 * Decoder: skip 1 bit, deinterleave 120 bits -> 8 BCH codewords,
		 * decode each, verify checksum, unpack 6-bit characters.
		 *
		 * Checksum: encoder computes checksum on the 15-bit encoded BCH
		 * codewords (bch[0]+...+bch[6]) & 0x7f, then BCH-encodes it.
		 * Decoder re-encodes decoded data with calc_bch() to reconstruct
		 * the 15-bit codewords for checksum verification.
		 * ============================================================ */
		{
			int alpha_pos = 0;
			int alpha_broke_early = 0; /* 1 = loop broke due to not enough bits */
			contbit = 1;

			for (i = 0; contbit && i < MAX_ADB; i++) {
				int k, j;

				if (pos + 1 + bch_block_bits > total_bits) {
					LOGP(DGOLAY, LOGL_NOTICE, "Alpha block %d: not enough bits.\n", i);
					alpha_broke_early = 1;
					break;
				}

				/* Skip 1-bit inverted comma */
				pos += 1;

				/* De-interleave 120 bits into 8 BCH codewords */
				deinterleave_bch(bits, &pos, bch_cw);

				/* Decode each BCH codeword to 7-bit data */
				int block_ok = 1;
				for (k = 0; k < 8; k++) {
					rc = decode_bch(bch_cw[k], &d[k]);
					if (rc < 0) {
						LOGP(DGOLAY, LOGL_NOTICE, "Alpha block %d: BCH[%d] decode failed.\n", i, k);
						d[k] = 0;
						msg->error_count++;
						block_ok = 0;
					}
				}

				/* Verify checksum: encoder sums the 15-bit encoded codewords,
				 * so we re-encode the decoded data to reconstruct them. */
				if (block_ok) {
					checksum = 0;
					for (k = 0; k < 7; k++)
						checksum += calc_bch(d[k]);
					checksum &= 0x7f;

					if (checksum != d[7]) {
						LOGP(DGOLAY, LOGL_NOTICE, "Alpha block %d: checksum mismatch (0x%02x != 0x%02x).\n",
							i, checksum, d[7]);
						msg->error_count++;
					}
				}

				/* Unpack 7 data words into 8 six-bit characters.
				 * Exact inverse of encoder packing:
				 *   bch[0] = (msg[0] | (msg[1] << 6)) & 0x7f
				 *   bch[1] = ((msg[1] >> 1) | (msg[2] << 5)) & 0x7f
				 *   ...
				 *   bch[6] = (contbit << 6) | msg[7] */
				{
					uint8_t ch[8];
					ch[0] = d[0] & 0x3f;
					ch[1] = ((d[0] >> 6) | (d[1] << 1)) & 0x3f;
					ch[2] = ((d[1] >> 5) | (d[2] << 2)) & 0x3f;
					ch[3] = ((d[2] >> 4) | (d[3] << 3)) & 0x3f;
					ch[4] = ((d[3] >> 3) | (d[4] << 4)) & 0x3f;
					ch[5] = ((d[4] >> 2) | (d[5] << 5)) & 0x3f;
					ch[6] = (d[5] >> 1) & 0x3f;
					ch[7] = d[6] & 0x3f;
					contbit = (d[6] >> 6) & 1;

					for (j = 0; j < 8; j++) {
						c = decode_alpha(ch[j]);
						if (alpha_pos < (int)sizeof(msg->alpha_data) - 1) {
							if (c != '\0')
								msg->alpha_data[alpha_pos++] = c;
							else if (ch[j] == 0x3e)
								msg->alpha_fill++;
						}
					}
				}

				LOGP(DGOLAY, LOGL_DEBUG, "Alpha block %d decoded, contbit=%d.\n", i, contbit);
			}

			msg->alpha_data[alpha_pos] = '\0';
			LOGP(DGOLAY, LOGL_INFO, "Alpha interpretation: '%s' (%d chars, %d fill).\n",
				msg->alpha_data, alpha_pos, msg->alpha_fill);

			/* Track why the loop exited with contbit still set:
			 * - hit MAX_ADB limit: the message has more blocks than
			 *   alpha can represent; this is expected for long numeric
			 *   messages and should NOT block the decode.
			 * - broke early (not enough bits): genuinely need more data. */
			if (contbit && alpha_broke_early)
				alpha_need_bits = 1;
		}

		/* ============================================================
		 * Stage 5: NUMERIC DATA BLOCKS
		 *
		 * Same de-interleave and BCH decode as alpha, but 4-bit nibble
		 * packing instead of 6-bit character packing.
		 *
		 * Encoder packing (7 data words -> 12 nibbles):
		 *   bch[0] = (msg[0] | (msg[1] << 4)) & 0x7f
		 *   bch[1] = ((msg[1] >> 3) | (msg[2] << 1) | (msg[3] << 5)) & 0x7f
		 *   ...
		 *   bch[6] = (contbit << 6) | (msg[10] >> 2) | (msg[11] << 2)
		 *
		 * Re-reads from the same bitstream position as Stage 4.
		 * ============================================================ */
		{
			int numeric_pos = 0;
			int numeric_broke_early = 0; /* 1 = loop broke due to not enough bits */
			pos = data_start_pos; /* restore position to re-decode as numeric */
			contbit = 1;
			shifted = 0;
			msg->numeric_nibble_count = 0;

			for (i = 0; contbit && i < MAX_NDB; i++) {
				int k, j;

				if (pos + 1 + bch_block_bits > total_bits) {
					LOGP(DGOLAY, LOGL_NOTICE, "Numeric block %d: not enough bits.\n", i);
					numeric_broke_early = 1;
					break;
				}

				/* Skip 1-bit inverted comma */
				pos += 1;

				/* De-interleave 120 bits into 8 BCH codewords */
				deinterleave_bch(bits, &pos, bch_cw);

				/* Decode each BCH codeword */
				int block_ok = 1;
				for (k = 0; k < 8; k++) {
					rc = decode_bch(bch_cw[k], &d[k]);
					if (rc < 0) {
						LOGP(DGOLAY, LOGL_NOTICE, "Numeric block %d: BCH[%d] decode failed.\n", i, k);
						d[k] = 0;
						msg->error_count++;
						block_ok = 0;
					}
				}

				/* Verify checksum (same as alpha: sum of re-encoded codewords) */
				if (block_ok) {
					checksum = 0;
					for (k = 0; k < 7; k++)
						checksum += calc_bch(d[k]);
					checksum &= 0x7f;

					if (checksum != d[7]) {
						LOGP(DGOLAY, LOGL_NOTICE, "Numeric block %d: checksum mismatch (0x%02x != 0x%02x).\n",
							i, checksum, d[7]);
						msg->error_count++;
					}
				}

				/* Unpack 7 data words into 12 four-bit nibbles */
				{
					uint8_t nib[12];
					nib[0]  = d[0] & 0xf;
					nib[1]  = ((d[0] >> 4) | (d[1] << 3)) & 0xf;
					nib[2]  = (d[1] >> 1) & 0xf;
					nib[3]  = ((d[1] >> 5) | (d[2] << 2)) & 0xf;
					nib[4]  = (d[2] >> 2) & 0xf;
					nib[5]  = ((d[2] >> 6) | (d[3] << 1)) & 0xf;
					nib[6]  = (d[3] >> 3) & 0xf;
					nib[7]  = d[4] & 0xf;
					nib[8]  = ((d[4] >> 4) | (d[5] << 3)) & 0xf;
					nib[9]  = (d[5] >> 1) & 0xf;
					nib[10] = ((d[5] >> 5) | (d[6] << 2)) & 0xf;
					nib[11] = (d[6] >> 2) & 0xf;
					contbit = (d[6] >> 6) & 1;

					for (j = 0; j < 12; j++) {
						/* Store raw nibble for scoring */
						if (msg->numeric_nibble_count < (int)sizeof(msg->numeric_nibbles))
							msg->numeric_nibbles[msg->numeric_nibble_count++] = nib[j];

						c = decode_numeric(nib[j], &shifted);
						if (c != '\0' && numeric_pos < (int)sizeof(msg->numeric_data) - 1)
							msg->numeric_data[numeric_pos++] = c;
						else if (c == '\0' && nib[j] == 0x0a)
							msg->numeric_fill++;
					}
				}

				LOGP(DGOLAY, LOGL_DEBUG, "Numeric block %d decoded, contbit=%d.\n", i, contbit);
			}

			msg->numeric_data[numeric_pos] = '\0';
			LOGP(DGOLAY, LOGL_INFO, "Numeric interpretation: '%s' (%d chars, %d fill, %d nibbles).\n",
				msg->numeric_data, numeric_pos, msg->numeric_fill, msg->numeric_nibble_count);

			/* Track why the loop exited with contbit still set (same
			 * logic as the alpha stage above). */
			if (contbit && numeric_broke_early)
				numeric_need_bits = 1;
		}

		/* ---- Decide whether to wait for more data or proceed ----
		 *
		 * We only return -1 (need more data) if BOTH stages genuinely
		 * need more bits. If one stage completed (contbit=0) or hit its
		 * block limit (contbit=1 but i>=MAX), the message data is fully
		 * available in the bitstream — the other stage just can't
		 * represent it. Proceed to discrimination.
		 *
		 * Cases:
		 *   alpha complete + numeric complete     -> proceed
		 *   alpha complete + numeric hit limit     -> proceed
		 *   alpha complete + numeric need bits     -> proceed (alpha is authoritative)
		 *   alpha hit limit + numeric complete     -> proceed
		 *   alpha hit limit + numeric hit limit    -> proceed (both maxed out)
		 *   alpha hit limit + numeric need bits    -> proceed (alpha consumed all blocks)
		 *   alpha need bits + numeric complete     -> proceed (numeric is authoritative)
		 *   alpha need bits + numeric hit limit    -> proceed (numeric consumed its blocks)
		 *   alpha need bits + numeric need bits    -> WAIT (genuinely incomplete)
		 */
		if (alpha_need_bits && numeric_need_bits && !force) {
			LOGP(DGOLAY, LOGL_DEBUG, "Both alpha and numeric stages need more data, waiting.\n");
			return -1;
		}

		/* ============================================================
		 * Scoring and discrimination: score both interpretations and
		 * pick the winner using unified scoring (content + fill).
		 * ============================================================ */
		msg->alpha_score = gsc_score_alpha(msg->alpha_data, strlen(msg->alpha_data), msg->alpha_fill);
		msg->numeric_score = gsc_score_numeric(msg->numeric_nibbles, msg->numeric_nibble_count, msg->numeric_fill);
		gsc_discriminate(msg);

		LOGP(DGOLAY, LOGL_INFO, "Discrimination: winner=%s, method=%s, alpha_score=%d, numeric_score=%d, uncertain=%d.\n",
			msg->guess_winner == TYPE_ALPHA ? "alpha" : "numeric",
			"unified-score",
			msg->alpha_score, msg->numeric_score, msg->guess_uncertain);
	}

	/* ================================================================
	 * Stage 6: ACTIVATION CODE (VOICE)
	 *
	 * Encoder: golay = calc_golay(activation_code)
	 *          queue_comma(gsc, 28, golay & 1)
	 *          queue_dup(gsc, golay, 23)
	 *          golay ^= 0x7fffff
	 *          queue_bit(gsc, (golay & 1) ^ 1)
	 *          queue_dup(gsc, golay, 23)
	 *
	 * Layout: 28 + 46 + 1 + 46 = 121 bits
	 * ================================================================ */

	if (detected_type == TYPE_VOICE) {
		if (pos + comma_len + dup_bits + 1 + dup_bits > total_bits) {
			LOGP(DGOLAY, LOGL_NOTICE, "Not enough bits for activation code.\n");
			msg->error_count++;
		} else {
			/* Skip 28-bit comma */
			pos += comma_len;

			/* Read and verify activation code (2563) */
			codeword = read_dup_golay(bits, &pos);
			rc = decode_golay(codeword, &decoded_value);
			if (rc < 0) {
				LOGP(DGOLAY, LOGL_NOTICE, "Activation code: Golay decode failed.\n");
				msg->error_count++;
			} else if (decoded_value != activation_code) {
				LOGP(DGOLAY, LOGL_NOTICE, "Activation code: expected %u, got %u.\n",
					(unsigned)activation_code, decoded_value);
				msg->error_count++;
			} else {
				LOGP(DGOLAY, LOGL_INFO, "Activation code verified (value %u).\n",
					(unsigned)activation_code);
			}

			/* Skip 1-bit inverted comma */
			pos += 1;

			/* Read and verify complement */
			codeword = read_dup_golay(bits, &pos);
			rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
			if (rc < 0) {
				LOGP(DGOLAY, LOGL_NOTICE, "Activation code: complement decode failed.\n");
				msg->error_count++;
			} else if (decoded_value != activation_code) {
				LOGP(DGOLAY, LOGL_NOTICE, "Activation code: complement = %u, expected %u.\n",
					decoded_value, (unsigned)activation_code);
				msg->error_count++;
			}
		}

		msg->type = TYPE_VOICE;
		LOGP(DGOLAY, LOGL_INFO, "Voice message detected for address '%s'.\n", msg->address);
	}

	/* ================================================================
	 * Stage 7: TONE-ONLY
	 *
	 * Encoder: queue_comma(gsc, 121 * 8, 1) = 968 bits alternating
	 *
	 * The tone-only comma has already been identified by the type
	 * detection logic. Skip past it.
	 * ================================================================ */

	if (detected_type == TYPE_TONE) {
		if (remaining >= tone_comma_len)
			pos += tone_comma_len;
		else
			pos += remaining;

		msg->type = TYPE_TONE;
		LOGP(DGOLAY, LOGL_INFO, "Tone-only message for address '%s'.\n", msg->address);
	}

	/* ================================================================
	 * Stage 8: BATCH MODE
	 *
	 * Currently single-message decode only.
	 *
	 * TODO: Batch mode (up to 16 addresses per batch):
	 *   - After one address+message, check for additional address pairs
	 *   - Extended batch: second start code without new preamble
	 *   - New preamble: start of new individual transmission
	 *   - Interleaved tone-only during voice page alert period
	 * ================================================================ */

	msg->decode_ok = 1;

	LOGP(DGOLAY, LOGL_INFO, "Batch decode complete: address='%s' type=%s data='%s' errors=%d.\n",
		msg->address,
		msg->type == TYPE_VOICE ? "voice" :
		msg->type == TYPE_ALPHA ? "alpha" :
		msg->type == TYPE_NUMERIC ? "numeric" : "tone",
		msg->data, msg->error_count);

	return 0;
}

static inline void queue_reset(gsc_t *gsc)
{
	gsc->bit_index = 0;
	gsc->bit_num = 0;
	gsc->bit_ac = 0;
	gsc->bit_overflow = 0;
}

static inline void queue_bit(gsc_t *gsc, int bit)
{
	if (gsc->bit_num == sizeof(gsc->bit))
		gsc->bit_overflow = 1;
	if (gsc->bit_overflow) {
		gsc->bit_num++;
		return;
	}
	gsc->bit[gsc->bit_num++] = bit;
}

static inline void queue_dup(gsc_t *gsc, uint32_t data, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		queue_bit(gsc, (data >> i) & 1);
		queue_bit(gsc, (data >> i) & 1);
	}
}

static inline void queue_comma(gsc_t *gsc, int bits, uint8_t polarity)
{
	int i;

	for (i = 0; i < bits; i++) {
		queue_bit(gsc, polarity);
		polarity = !polarity;
	}
}

static int queue_batch(gsc_t *gsc, const char *address, enum gsc_msg_type type, const char *message)
{
	int preamble;
	uint16_t word1, word2;
	uint8_t function;
	uint32_t golay;
	uint16_t bch[8];
	uint8_t msg[12], digit, shifted, contbit, checksum;
	int i, j, k;
	int rc;

	queue_reset(gsc);

	/* check address length */
	if (!address || strlen(address) != 7) {
		LOGP(DGOLAY, LOGL_NOTICE, "Invalid functional address '%s' size. Only 7 digits are allowed.\n", address);
		return -EINVAL;
	}

	/* calculate address */
	rc = encode_address(address, &preamble, &word1, &word2);
	if (rc < 0)
		return rc;

	/* get function from last digit */
	switch (address[6]) {
		case '1': function = 0; break;
		case '2': function = 1; break;
		case '3': function = 2; break;
		case '4': function = 3; break;
		case '5': function = 0; break;
		case '6': function = 1; break;
		case '7': function = 2; break;
		case '8': function = 3; break;
		case '9': function = 0; break;
		case '0': function = 1; break;
		default:
			LOGP(DGOLAY, LOGL_NOTICE, "Illegal function suffix '%c' in last address digit.\n", address[6]);
			return -EINVAL;
	}

	switch (type) {
	case TYPE_ALPHA:
	case TYPE_NUMERIC:
		LOGP(DGOLAY, LOGL_INFO, "Coding text message for functional address '%s' and message '%s'.\n", address, message);
		break;
	case TYPE_VOICE:
		LOGP(DGOLAY, LOGL_INFO, "Coding voice message for functional address %s with wave file '%s'.\n", address, message);
		break;
	default:
		LOGP(DGOLAY, LOGL_INFO, "Coding tone only message for functional address %s.\n", address);
	}

	/* encode preamble and store */
	LOGP(DGOLAY, LOGL_DEBUG, "Encoding preamble '%d'.\n", preamble);
	golay = calc_golay(preamble_values[preamble]);
	queue_comma(gsc, 28, golay & 1);
	for (i = 0; i < 18; i++) {
		queue_dup(gsc, golay, 23);
	}

	/* encode start code and store */
	LOGP(DGOLAY, LOGL_DEBUG, "Encoding start code.\n");
	golay = calc_golay(start_code);
	queue_comma(gsc, 28, golay & 1);
	queue_dup(gsc, golay, 23);
	golay ^= 0x7fffff;
	queue_bit(gsc, (golay & 1) ^ 1);
	queue_dup(gsc, golay, 23);

	/* encode address and store */
	LOGP(DGOLAY, LOGL_DEBUG, "Encoding address words '%d' and '%d'.\n", word1, word2);
	golay = calc_golay(word1);
	if (function & 0x2)
		golay ^= 0x7fffff;
	queue_comma(gsc, 28, golay & 1);
	queue_dup(gsc, golay, 23);
	golay = calc_golay(word2);
	if (function & 0x1)
		golay ^= 0x7fffff;
	queue_bit(gsc, (golay & 1) ^ 1);
	queue_dup(gsc, golay, 23);

	/* encode message */
	switch (type) {
	case TYPE_ALPHA:
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding %d alphanumeric digits.\n", (int)strlen(message));
		for (i = 0; *message; i++) {
			if (i == MAX_ADB) {
				LOGP(DGOLAY, LOGL_NOTICE, "Message overflows %d characters, cropping message.\n", MAX_ADB * 8);
			}
			for (j = 0; *message && j < 8; j++) {
				msg[j] = encode_alpha(*message++);
			}
			/* fill empty characters with NULL */
			while (j < 8)
				msg[j++] = 0x3e;
			/* 8 characters + continue-bit */
			bch[0] = calc_bch((msg[0] | (msg[1] << 6)) & 0x7f);
			bch[1] = calc_bch(((msg[1] >> 1) | (msg[2] << 5)) & 0x7f);
			bch[2] = calc_bch(((msg[2] >> 2) | (msg[3] << 4)) & 0x7f);
			bch[3] = calc_bch(((msg[3] >> 3) | (msg[4] << 3)) & 0x7f);
			bch[4] = calc_bch(((msg[4] >> 4) | (msg[5] << 2)) & 0x7f);
			bch[5] = calc_bch(((msg[5] >> 5) | (msg[6] << 1)) & 0x7f);
			if (*message && i < MAX_ADB)
				contbit = 1;
			else
				contbit = 0;
			bch[6] = calc_bch((contbit << 6) | msg[7]);
			/* checksum */
			checksum = bch[0] + bch[1] + bch[2] + bch[3] + bch[4] + bch[5] + bch[6];
			bch[7] = calc_bch(checksum & 0x7f);
			/* store comma bit */
			queue_bit(gsc, (bch[0] & 1) ^ 1); // inverted first bit
			/* store interleaved bits */
			for (j = 0; j < 15; j++) {
				for (k = 0; k < 8; k++)
					queue_bit(gsc, (bch[k] >> j) & 1);
			}
		}
		break;
	case TYPE_NUMERIC:
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding %d numeric digits.\n", (int)strlen(message));
		shifted = 0;
		for (i = 0; *message; i++) {
			if (i == MAX_NDB) {
				LOGP(DGOLAY, LOGL_NOTICE, "Message overflows %d characters, cropping message.\n", MAX_NDB * 12);
			}
			for (j = 0; *message && j < 12; j++) {
				/* get next digit or shifted digit */
				if (shifted) {
					digit = shifted & 0xf;
					shifted = 0;
				} else
					digit = encode_numeric(*message);
				/* if digit is extended, use the shifted code and later the digit itself */
				if (digit > 0xf) {
					shifted = digit;
					msg[j] = digit >> 4;
				} else {
					msg[j] = digit;
					message++;
				}
			}
			/* fill empty digits with NULL */
			while (j < 12)
				msg[j++] = 0xa;
			/* 8 digits + continue-bit */
			bch[0] = calc_bch((msg[0] | (msg[1] << 4)) & 0x7f);
			bch[1] = calc_bch(((msg[1] >> 3) | (msg[2] << 1) | (msg[3] << 5)) & 0x7f);
			bch[2] = calc_bch(((msg[3] >> 2) | (msg[4] << 2) | (msg[5] << 6)) & 0x7f);
			bch[3] = calc_bch(((msg[5] >> 1) | (msg[6] << 3)) & 0x7f);
			bch[4] = calc_bch((msg[7] | (msg[8] << 4)) & 0x7f);
			bch[5] = calc_bch(((msg[8] >> 3) | (msg[9] << 1) | (msg[10] << 5)) & 0x7f);
			if (*message && i < MAX_NDB)
				contbit = 1;
			else
				contbit = 0;
			bch[6] = calc_bch((contbit << 6) | (msg[10] >> 2) | (msg[11] << 2));
			/* checksum */
			checksum = bch[0] + bch[1] + bch[2] + bch[3] + bch[4] + bch[5] + bch[6];
			bch[7] = calc_bch(checksum & 0x7f);
			/* store comma bit */
			queue_bit(gsc, (bch[0] & 1) ^ 1); // inverted first bit
			/* store interleaved bits */
			for (j = 0; j < 15; j++) {
				for (k = 0; k < 8; k++)
					queue_bit(gsc, (bch[k] >> j) & 1);
			}
		}
		break;
	case TYPE_VOICE:
		/* store wave file name */
		memcpy(gsc->wave_tx_filename, message, MIN(sizeof(gsc->wave_tx_filename) - 1, strlen(message) + 1));
		/* store bit number for activation code. this is used to play the AC again after voice message. */
		gsc->bit_ac = gsc->bit_num;
		/* encode activation code and store */
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding activation code.\n");
		golay = calc_golay(activation_code);
		queue_comma(gsc, 28, golay & 1);
		queue_dup(gsc, golay, 23);
		golay ^= 0x7fffff;
		queue_bit(gsc, (golay & 1) ^ 1);
		queue_dup(gsc, golay, 23);
		break;
	default:
		/* encode comma after message and store */
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding 'comma' sequence after message.\n");
		queue_comma(gsc, 121 * 8, 1);
	}

	/* check overflow */
	if (gsc->bit_overflow) {
		LOGP(DGOLAY, LOGL_ERROR, "Bit stream (%d bits) overflows bit buffer size (%d bits), please fix!\n", gsc->bit_num, (int)sizeof(gsc->bit));
		return -EOVERFLOW;
	}

	return 0;
}

/* get next bit
 *
 * if there is no message, return -1, so that the transmitter is turned off.
 *
 * if there is a message, return next bit to be transmitted.
 *
 * if there is a message in the queue, encode message and return its first bit.
 *
 * if there is a voice message, return 2 at the end, to tell the DSP to send voice.
 */
int8_t get_bit(gsc_t *gsc)
{
	gsc_msg_t *msg;
	int rc;

	/* if currently transmiting message, send next bit */
	if (gsc->bit_num) {
		/* Transmission complete. */
		if (gsc->bit_index == gsc->bit_num) {
			/* on voice message... */
			if (gsc->bit_ac) {
				/* rewind to play the AC again after voice transmission */
				gsc->bit_index = gsc->bit_ac;
				gsc->bit_ac = 0;
				/* indicate voice message to DSP */
				return 2;
			}
			queue_reset(gsc);
			LOGP(DGOLAY, LOGL_INFO, "Done transmitting message.\n");
			goto next_msg;
		}
		return gsc->bit[gsc->bit_index++];
	}

next_msg:
	msg = gsc->msg_list;

	/* no message pending, turn transmitter off */
	if (!msg)
		return -1;

	/* encode first message in queue */
	rc = queue_batch(gsc, msg->address, msg->type, msg->data);
	if (rc >= 0)
		LOGP(DGOLAY, LOGL_INFO, "Transmitting message to address '%s'.\n", msg->address);
	golay_msg_destroy(gsc, msg);
	if (rc < 0)
		goto next_msg;

	/* return first bit */
	return gsc->bit[gsc->bit_index++];
}

void golay_msg_send(const char *text)
{
	char buffer[strlen(text) + 1], *p = buffer, *address_string, *message;
	gsc_t *gsc;
	enum gsc_msg_type type = TYPE_AUTO;

	strcpy(buffer, text);
	address_string = strsep(&p, ",");
	message = p;

	/* If no comma was found, p is NULL — this is a tone-only/auto
	 * message with just an address and no payload. */
	if (!message) {
		message = "";
	}

	switch ((message[0] << 8) | message[1]) {
	case ('a' << 8) | ',':
		type = TYPE_ALPHA;
		message += 2;
		break;
	case ('n' << 8) | ',':
		type = TYPE_NUMERIC;
		message += 2;
		break;
	case ('v' << 8) | ',':
		type = TYPE_VOICE;
		message += 2;
		break;
	default:
		message = "";
	}

	gsc = (gsc_t *) sender_head;
	golay_msg_create(gsc, address_string, message, type);
}

/* Output a decoded RX message to the log and to the receive FIFO.
 *
 * For data messages (TYPE_ALPHA or TYPE_NUMERIC), outputs both alpha and
 * numeric interpretations with scores, fill counts, and winner marker.
 * The winning interpretation is marked [WINNER] (or [WINNER?] if uncertain).
 *
 * Format written to MSG_RECEIVED for data messages:
 *   "<address>,<winner_type>,<polarity>,<winner_data>,<alpha_data>,<numeric_data>,<alpha_score>,<numeric_score>,<method>,<uncertain>\n"
 *
 * For tone-only and voice messages, the existing format is retained:
 *   "<address>,<type>,<polarity>,<data>\n"
 */
void golay_msg_receive(const gsc_rx_msg_t *msg)
{
	const char *pol_str = msg->polarity_inverted ? "-" : "+";
	FILE *fp;
	int is_data;

	is_data = (msg->type == TYPE_ALPHA || msg->type == TYPE_NUMERIC);

	if (is_data) {
		/* Dual-decode output: show both interpretations with scores */
		const char *winner_str = msg->guess_winner == TYPE_ALPHA ? "alpha" : "numeric";
		const char *alpha_marker, *numeric_marker;
		const char *method_str = "unified-score";
		int delta;

		if (msg->guess_winner == TYPE_ALPHA) {
			alpha_marker = msg->guess_uncertain ? "[WINNER?]" : "[WINNER]";
			numeric_marker = "";
		} else {
			alpha_marker = "";
			numeric_marker = msg->guess_uncertain ? "[WINNER?]" : "[WINNER]";
		}

		LOGP(DGOLAY, LOGL_NOTICE, "Received message for address '%s' (errors=%d, polarity=%s):\n",
			msg->address, msg->error_count, pol_str);
		LOGP(DGOLAY, LOGL_NOTICE, "  Alpha %s: '%s' (score=%d, fill=%d)\n",
			alpha_marker, msg->alpha_data, msg->alpha_score, msg->alpha_fill);
		LOGP(DGOLAY, LOGL_NOTICE, "  Numeric %s: '%s' (score=%d, fill=%d)\n",
			numeric_marker, msg->numeric_data, msg->numeric_score, msg->numeric_fill);

		if (msg->guess_uncertain) {
			delta = msg->alpha_score - msg->numeric_score;
			if (delta < 0)
				delta = -delta;
			LOGP(DGOLAY, LOGL_NOTICE, "  Method: %s, UNCERTAIN (delta=%d)\n",
				method_str, delta);
		} else {
			LOGP(DGOLAY, LOGL_NOTICE, "  Method: %s\n", method_str);
		}

		fp = fopen("/tmp/golay_msg_received", "a");
		if (fp) {
			fprintf(fp, "%s,%s,%s,%s,%s,%s,%d,%d,%s,%d\n",
				msg->address, winner_str, pol_str, msg->data,
				msg->alpha_data, msg->numeric_data,
				msg->alpha_score, msg->numeric_score,
				method_str, msg->guess_uncertain);
			fclose(fp);
		} else {
			LOGP(DGOLAY, LOGL_ERROR, "Failed to open /tmp/golay_msg_received for writing.\n");
		}
	} else {
		/* Tone-only and voice: retain existing single-line format */
		const char *type_str;

		switch (msg->type) {
		case TYPE_VOICE:
			type_str = "voice";
			break;
		default:
			type_str = "tone";
			break;
		}

		LOGP(DGOLAY, LOGL_NOTICE, "Received %s message for address '%s': '%s' (errors=%d, polarity=%s).\n",
			type_str, msg->address, msg->data, msg->error_count, pol_str);

		fp = fopen("/tmp/golay_msg_received", "a");
		if (fp) {
			fprintf(fp, "%s,%s,%s,%s\n", msg->address, type_str, pol_str, msg->data);
			fclose(fp);
		} else {
			LOGP(DGOLAY, LOGL_ERROR, "Failed to open /tmp/golay_msg_received for writing.\n");
		}
	}
}

void call_down_clock(void)
{
}

/* Call control starts call towards paging network. */
int call_down_setup(int __attribute__((unused)) callref, const char *caller_id, enum number_type __attribute__((unused)) caller_type, const char *dialing)
{
	char channel = '\0';
	sender_t *sender;
	gsc_t *gsc;
	const char *address;
	const char *message;
	gsc_msg_t *msg;

	/* find transmitter */
	for (sender = sender_head; sender; sender = sender->next) {
		/* skip channels that are different than requested */
		if (channel && sender->kanal[0] != channel)
			continue;
		gsc = (gsc_t *) sender;
		/* check if base station cannot transmit */
		if (!gsc->tx)
			continue;
		break;
	}
	if (!sender) {
		if (channel)
			LOGP(DGOLAY, LOGL_NOTICE, "Cannot page, because given station not available, rejecting!\n");
		else
			LOGP(DGOLAY, LOGL_NOTICE, "Cannot page, no trasmitting station available, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}

	/* get address */
	address = dialing;

	/* get message */
	if (caller_id[0])
		message = caller_id;
	else
		message = gsc->default_message;

	/* create call process to page station */
	msg = golay_msg_create(gsc, address, message, TYPE_AUTO);
	if (!msg)
		return -CAUSE_INVALNUMBER;
	return -CAUSE_NORMAL;
}

void call_down_answer(int __attribute__((unused)) callref, struct timeval __attribute__((unused)) *tv_meter)
{
}

void call_down_proceeding(int __attribute__((unused)) callref)
{
}


static void _release(int __attribute__((unused)) callref, int __attribute__((unused)) cause)
{
	LOGP(DGOLAY, LOGL_INFO, "Call has been disconnected by network.\n");
}

void call_down_disconnect(int callref, int cause)
{
	_release(callref, cause);

	call_up_release(callref, cause);
}

/* Call control releases call toward mobile station. */
void call_down_release(int callref, int cause)
{
	_release(callref, cause);
}

/* Receive audio from call instance. */
void call_down_audio(void __attribute__((unused)) *decoder, void __attribute__((unused)) *decoder_priv, int __attribute__((unused)) callref, uint16_t __attribute__((unused)) sequence, uint8_t __attribute__((unused)) marker, uint32_t __attribute__((unused)) timestamp, uint32_t __attribute__((unused)) ssrc, uint8_t __attribute__((unused)) *payload, int __attribute__((unused)) payload_len)
{
}

void dump_info(void) {}

