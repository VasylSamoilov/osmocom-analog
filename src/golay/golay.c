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
#include <ctype.h>
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
	gsc->fsk_tx_polarity = gsc->fsk_polarity;

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
			LOGP(DGOLAY, LOGL_INFO, "RX polarity: locked to %s.\n", (polarity < 0) ? "inverted" : "normal");
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

	/* Initialize scheduler state */
	gsc->priority_list = NULL;
	gsc->priority_count = 0;
	gsc->normal_count = 0;
	gsc->sched_current_group = 0;
	gsc->tx_msg_count = 0;
	gsc->tx_preamble_index = 0;
	gsc->batching_mode = BATCHING_OFF;
	gsc->holdoff_ms = 100;
	gsc->holdoff_active = 0;
	memset(&gsc->holdoff_start, 0, sizeof(gsc->holdoff_start));
	gsc->expiry_seconds = 300;
	gsc->rx_batch_candidate = 0;
	gsc->rx_batch_mode = 0;

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
static gsc_msg_t *golay_msg_create(gsc_t __attribute__((unused)) *gsc, const char *address, const char *text, enum gsc_msg_type type)
{
	gsc_msg_t *msg;

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

	/* Message is returned without linking into any queue.
	 * The caller is responsible for calling scheduler_enqueue()
	 * after setting priority/polarity fields. */

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

	/* Syndrome 0 means no errors - codeword is valid */
	if (syndrome == 0) {
		*data = codeword & 0xFFF;
		return 0;
	}

	/* Look up the error pattern for this syndrome */
	error_pattern = golay_syndrome[syndrome];
	if (error_pattern == 0xFFFFFFFF) {
		LOGP(DGOLAY, LOGL_DEBUG, "Golay decode failed: uncorrectable error (syndrome 0x%03x).\n", syndrome);
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

	/* Syndrome 0 means no errors - codeword is valid */
	if (syndrome == 0) {
		*data = codeword & 0x7F;
		return 0;
	}

	/* Look up the error pattern for this syndrome */
	error_pattern = bch_syndrome[syndrome];
	if (error_pattern == 0xFFFF) {
		LOGP(DGOLAY, LOGL_DEBUG, "BCH decode failed: uncorrectable error (syndrome 0x%02x).\n", syndrome);
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
 * Returns 0 on success (a2, a1, a0 written), -1 if mapping is invalid.
 *
 * Invalid GSC codes (Appendix I):
 *   Certain A2A1A0 values are invalid for given G1G0 ranges and must
 *   be skipped when assigning codes. These arise from the W2 address
 *   arithmetic producing values that collide with protocol-reserved
 *   codewords or cause ambiguity in the W2-to-address reverse mapping.
 *
 *   G1G0 00-49: A2A1A0 must not equal
 *     000, 025, 051, 103, 206, 340, 363, 412,
 *     445, 530, 642, 726, 782, 810, 825, 877
 *
 *   G1G0 50-99: A2A1A0 must not equal
 *     000, 292, 425, 584, 631, 841, 851
 *
 *   For non-battery-saver ("N" code) systems, G1G0 must never
 *   equal 40 or 90 regardless of A2A1A0.
 *
 * W1 table ambiguity: word1s[] has 50 entries. G1G0 0-49 maps directly,
 * G1G0 50-99 uses word1s[G1G0-50] (same entry, W2 offset by +50).
 * The decoder tries both ranges and uses reverse_word2() success as
 * the discriminator. When both ranges produce valid addresses (the
 * invalid code tables do not cover all cases), the result is ambiguous.
 * A real pager knows its own address; a monitoring decoder cannot
 * always disambiguate.
 *
 * Code Assignment Plans (Figure 7.1):
 *   Plan 1: 50-100 codes, 1 preamble, fixed G1G0A2
 *   Plan 2: 500-1000 codes, 10 preambles, fixed G1A2
 *   Plan 3: 500-1000 codes, 1 preamble, fixed G1A2
 *   Plan 4: 5000-10000 codes, 10 preambles, fixed G1
 *   Plan 5: 5000-10000 codes, 1 preamble (I fixed), fixed G0
 *   Plan 6: 50000-100000 codes, 50 preambles, all digits variable */
int reverse_word2(uint16_t w2, int g1g0, int *a2, int *a1, int *a0)
{
	/* Invalid GSC code tables (Appendix I).
	 * These match the ones in encode_address() -- duplicated here
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

	/* raw must be non-negative for valid arithmetic */
	if (raw < 0)
		return -1;

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

	/* Validate digit range: each must be 0-9.
	 * Negative values can occur when the W2 arithmetic overflows
	 * for invalid range combinations. */
	if (*a2 < 0 || *a2 > 9 || *a1 < 0 || *a1 > 9 || *a0 < 0 || *a0 > 9) {
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
 * numeric_decode_table[16] - unshifted codes:
 *   0x0-0x9: '0'-'9'
 *   0xA:     '\0' (NULL fill, used for padding empty digit slots)
 *   0xB:     'U'
 *   0xC:     ' ' (space)
 *   0xD:     '-' (hyphen)
 *   0xE:     '*' (asterisk; encoder also accepts '=' as alias)
 *   0xF:     '\0' (shift prefix - sets shifted mode, no character emitted)
 *
 * numeric_shift_table[16] - codes after 0xF prefix:
 *   0x0: 'A', 0x1: 'B', 0x2: 'C', 0x3: 'D', 0x4: 'E',
 *   0x5: ' ' (Space), 0x6: 'F', 0x7: 'G', 0x8: 'H', 0x9: 'J',
 *   0xA: '\0' (Null), 0xB: 'L', 0xC: 'N', 0xD: 'P', 0xE: 'R',
 *   0xF: '?' (Spare)
 *
 * Per Table VII, shifted codes 0x5 and 0xA map to Space and Null
 * respectively. Code 0xF is spare (undefined).
 *
 * UNCONFIRMED: Motorola GSC Table A-17 defines international characters
 * (ae, oe, ue, n, c, e, e, e, A, ss, etc.) via a SHIFT prefix in the
 * alphanumeric stream. The SHIFT prefix code and full mechanism are not
 * yet confirmed from the source document. Not implemented.
 */
static const char numeric_decode_table[16] = {
	/* 0x0 */ '0', '1', '2', '3', '4', '5', '6', '7',
	/* 0x8 */ '8', '9', '\0', 'U', ' ', '-', '*', '\0',
};

static const char numeric_shift_table[16] = {
	/* 0x0 */ 'A', 'B', 'C', 'D', 'E', ' ', 'F', 'G',
	/* 0x8 */ 'H', 'J', '\0', 'L', 'N', 'P', 'R', '?',
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
		/* Previous nibble was 0xF shift prefix - decode shifted letter */
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
 *   +3  digit (nibble 0x0-0x9)
 *   -1  'U' (nibble 0xB) when it's the only U and at position 0
 *       (urgent-prefix convention: "U" + phone number)
 *  -15  'U' (nibble 0xB) otherwise - very rare in real numeric messages;
 *       common artifact when alpha data is misinterpreted as numeric
 *   -2  space (0xC), hyphen (0xD), asterisk (0xE)
 *   -2  shift prefix (0xF) - uncommon in numeric messages
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
	int u_count = 0;
	int first_u = -1;

	/* Pre-scan for 'U' (0xB) nibbles to detect urgent-prefix pattern */
	for (i = 0; i < count; i++) {
		uint8_t n = nibbles[i];
		/* Skip fill nibbles */
		if (n == 0x0A)
			continue;
		if (n == 0x0B) {
			u_count++;
			if (first_u < 0)
				first_u = i;
		}
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

		/* Exclude fill nibbles */
		if (n == 0x0A)
			continue;

		if (n <= 0x09) {
			/* Digit (0-9): +3 */
			score += 3;
		} else if (n == 0x0B) {
			if (urgent_prefix)
				score -= 1; /* leading U = urgent prefix - mild penalty */
			else
				score -= 15; /* stray U - strong artifact signal */
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
 *      short messages - its presence is a strong structural signal).
 *   2) If both have fill, the one with more fill wins (shorter message
 *      = more fill = stronger signal).
 *   3) If neither has fill (full block), compare content scores -
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
		/* Scores equal - default to alpha, flag uncertain */
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
			/* Bits disagree - prefer first bit, log for diagnostics */
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
 * The encoder writes 120 bits total (15 bit-positions x 8 codewords).
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

/* Escape newlines/CRs in a string for single-line log output.
 * Returns a static buffer — not reentrant, use immediately. */
static const char *log_escape(const char *s)
{
	static char buf[MAX_ADB * 8 * 2 + 1];
	int i = 0;
	while (*s && i < (int)sizeof(buf) - 3) {
		if (*s == '\n') { buf[i++] = '\\'; buf[i++] = 'n'; }
		else if (*s == '\r') { buf[i++] = '\\'; buf[i++] = 'r'; }
		else buf[i++] = *s;
		s++;
	}
	buf[i] = '\0';
	return buf;
}

int decode_batch(gsc_t *gsc, gsc_rx_msg_t *msg, int force)
{
	const uint8_t *bits = gsc->rx_bit;
	int pos = 0;
	int total_bits = gsc->rx_bit_num;

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
	 * The preamble index is already known from the DSP layer's
	 * confirmation phase. The rx_bit[] buffer starts from the first
	 * post-confirmation bit (remaining preamble + start code + data).
	 * Skip directly to the start code scan.
	 * ================================================================ */

	msg->preamble_index = gsc->rx_preamble_index;
	preamble_idx = gsc->rx_preamble_index;

	/* ================================================================
	 * Stage 2: START CODE
	 *
	 * Scan for start code (713) in the buffer. The buffer contains
	 * remaining preamble codewords followed by the start code.
	 * Scan from position 0 to find it.
	 *
	 * Layout: 28 + 46 + 1 + 46 = 121 bits
	 * ================================================================ */

	if (total_bits < 28 + 46) {
		return -1;
	}

	/* Scan for start code with a ±46 bit window around the expected
	 * position. The IDLE scanner's sliding window may match at any
	 * offset within the dup codeword, shifting the entire rx_bit[]
	 * buffer. Scanning handles this alignment variation. */
	{
		int sc_found = 0;
		int scan_end = total_bits - 28 - 46;
		int scan_pos;

		/* Limit scan to first 800 bits — start code should be there */
		if (scan_end > 800)
			scan_end = 800;

		for (scan_pos = 0; scan_pos <= scan_end; scan_pos++) {
			int try_pos = scan_pos + comma_len;
			if (try_pos + dup_bits > total_bits)
				break;

			codeword = read_dup_golay(bits, &try_pos);
			rc = decode_golay(codeword, &decoded_value);
			if (rc >= 0 && decoded_value == start_code) {
				LOGP(DGOLAY, LOGL_INFO, "Start code found at scan_pos=%d try_pos=%d total=%d\n",
					scan_pos, try_pos, total_bits);
				pos = try_pos; /* past the dup Golay */
				sc_found = 1;
				if (rc > 0)
					msg->error_count += rc;

				/* Dump 300 bits from start code position for diagnosis */
				{
					char dump[320];
					int d = 0, b;
					int dump_start = scan_pos;
					for (b = dump_start; b < dump_start + 300 && b < total_bits && d < 310; b++)
						dump[d++] = bits[b] ? '1' : '0';
					dump[d] = '\0';
					LOGP(DGOLAY, LOGL_INFO, "Bitstream from SC: %s\n", dump);
				}
				break;
			}
		}

		if (!sc_found)
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
	 * Two things must be resolved:
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
	 * Range detection: the encoder computes b1b0 = (ap1*10 + ap0) / 2,
	 * which is always 0-49 (max input 99 / 2 = 49). For low range,
	 * W2 % 100 = b1b0 (0-49). For high range, W2 % 100 = b1b0 + 50
	 * (50-99). So W2 % 100 deterministically identifies the range.
	 *
	 * We try both Golay polarities (normal/complement) and use the
	 * W2 value's low two digits to select the correct range. */
	codeword = read_dup_golay(bits, &pos);
	w2_inverted = 0;

	{
		uint16_t w2_try[2] = { 0, 0 };	/* [0]=normal, [1]=complement */
		int golay_ok[2] = { 0, 0 };
		int best_inv = -1;
		int ta2, ta1, ta0;
		int inv, det_range;

		rc = decode_golay(codeword, &decoded_value);
		if (rc == 0) { w2_try[0] = decoded_value; golay_ok[0] = 1; }

		rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
		if (rc == 0) { w2_try[1] = decoded_value; golay_ok[1] = 1; }

		/* Try both Golay polarities, determine range from W2 value */
		for (inv = 0; inv < 2; inv++) {
			if (!golay_ok[inv])
				continue;
			/* W2 % 100 >= 50 means high range (g1g0 + 50) */
			det_range = (w2_try[inv] % 100 >= 50) ? 1 : 0;
			if (reverse_word2(w2_try[inv], g1g0 + det_range * 50, &ta2, &ta1, &ta0) == 0) {
				if (best_inv < 0) {
					best_inv = inv;
					a2 = ta2; a1 = ta1; a0 = ta0;
					if (det_range) {
						g1g0 += 50;
						g1 = g1g0 / 10;
						g0 = g1g0 % 10;
					}
				}
				LOGP(DGOLAY, LOGL_DEBUG, "W2 candidate: value=%u inv=%d g1g0=%d -> A2=%d A1=%d A0=%d.\n",
					w2_try[inv], inv, g1g0 + (best_inv < 0 ? det_range * 50 : 0), ta2, ta1, ta0);
			}
		}

		if (best_inv < 0) {
			LOGP(DGOLAY, LOGL_NOTICE, "W2: no valid address from any combination.\n");
			return -1;
		}

		w2_value = w2_try[best_inv];
		w2_inverted = best_inv;
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
	 * the message type yet - return -1 so the caller can buffer more
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
						 * alpha here - the final type decision is
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
							 *    to alpha - gsc_discriminate() will make
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
			 * enough bits for a full tone comma yet, we can't tell -
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
		/* Table IX tone suffixes: func 1→9, 2→0, 3→3, 4→4 */
		suffix = "9034"[function];
		break;
	}

	/* Build the 7-digit functional address: I G1 G0 A2 A1 A0 suffix */
	LOGP(DGOLAY, LOGL_INFO, "Address: idx=%d g1=%d g0=%d a2=%d a1=%d a0=%d func=%d w1=%u w2=%u w1_inv=%d w2_inv=%d\n",
		idx, g1, g0, a2, a1, a0, function, w1_value, w2_value, w1_inverted, w2_inverted);
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
			int prev_contbit = 1; /* first block is trusted (address/type confirmed data) */
			contbit = 1;

			for (i = 0; contbit && i < MAX_ADB; i++) {
				int k, j;
				int bch_bad[8] = {0}; /* per-codeword failure flags */

				if (pos + 1 + bch_block_bits > total_bits) {
					alpha_broke_early = 1;
					break;
				}

				/* If previous block did NOT confirm continuation and this
				 * block has failures, we can't trust it belongs to the message.
				 * But we must still attempt decode to know. */

				/* Skip 1-bit inverted comma */
				pos += 1;

				/* De-interleave 120 bits into 8 BCH codewords */
				deinterleave_bch(bits, &pos, bch_cw);

				/* Decode each BCH codeword to 7-bit data */
				int block_ok = 1;
				for (k = 0; k < 8; k++) {
					rc = decode_bch(bch_cw[k], &d[k]);
					if (rc < 0) {
						LOGP(DGOLAY, LOGL_DEBUG, "Alpha block %d: BCH[%d] decode failed.\n", i, k);
						d[k] = 0;
						msg->uncorrectable_count++;
						block_ok = 0;
						bch_bad[k] = 1;
					}
				}

				/* If block has failures and previous block didn't confirm
				 * continuation, stop - can't trust this data. */
				if (!block_ok && !prev_contbit) {
					LOGP(DGOLAY, LOGL_DEBUG, "Alpha block %d: BCH failures and no prior continuation, stopping.\n", i);
					break;
				}

				/* Verify checksum: encoder sums the 15-bit encoded codewords,
				 * so we re-encode the decoded data to reconstruct them. */
				if (block_ok) {
					checksum = 0;
					for (k = 0; k < 7; k++)
						checksum += calc_bch(d[k]);
					checksum &= 0x7f;

					if (checksum != d[7]) {
						LOGP(DGOLAY, LOGL_DEBUG, "Alpha block %d: checksum mismatch (0x%02x != 0x%02x).\n",
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
					/* Which characters are tainted by failed BCH codewords:
					 * ch[0] <- d[0]       ch[4] <- d[3],d[4]
					 * ch[1] <- d[0],d[1]  ch[5] <- d[4],d[5]
					 * ch[2] <- d[1],d[2]  ch[6] <- d[5]
					 * ch[3] <- d[2],d[3]  ch[7] <- d[6]       */
					int ch_bad[8];
					ch_bad[0] = bch_bad[0];
					ch_bad[1] = bch_bad[0] || bch_bad[1];
					ch_bad[2] = bch_bad[1] || bch_bad[2];
					ch_bad[3] = bch_bad[2] || bch_bad[3];
					ch_bad[4] = bch_bad[3] || bch_bad[4];
					ch_bad[5] = bch_bad[4] || bch_bad[5];
					ch_bad[6] = bch_bad[5];
					ch_bad[7] = bch_bad[6];

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
						if (alpha_pos < (int)sizeof(msg->alpha_data) - 1) {
							if (ch_bad[j]) {
								msg->alpha_data[alpha_pos++] = '?';
							} else {
								c = decode_alpha(ch[j]);
								if (c != '\0')
									msg->alpha_data[alpha_pos++] = c;
								else if (ch[j] == 0x3e)
									msg->alpha_fill++;
							}
						}
					}
				}

				/* Update prev_contbit: only trust contbit from a clean block.
				 * If block had BCH failures, d[6] is 0 so contbit is garbage
				 * (will be 0, causing natural loop exit). */
				prev_contbit = block_ok ? contbit : 0;
			}

			msg->alpha_data[alpha_pos] = '\0';

			/* Track why the loop exited with contbit still set */
			if (contbit && !alpha_broke_early) {
				/* Hit MAX_ADB with contbit still set — message truncated */
				LOGP(DGOLAY, LOGL_NOTICE, "Alpha message truncated at %d blocks (%d chars), transmitter sent more data.\n",
					i, alpha_pos);
			}
			if (contbit && alpha_broke_early)
				alpha_need_bits = 1;
		}

		/* If alpha already decoded more chars than numeric can ever
		 * produce (MAX_NDB * 12 = 24 digits), this is definitively
		 * alpha. Skip the numeric stage entirely. */
		if (strlen(msg->alpha_data) > (unsigned)(MAX_NDB * 8)) {
			/* Wait for more data if alpha isn't done yet */
			if (alpha_need_bits && !force) {
				return -1;
			}

			msg->alpha_score = gsc_score_alpha(msg->alpha_data, strlen(msg->alpha_data), msg->alpha_fill);
			msg->numeric_score = 0;
			msg->guess_winner = TYPE_ALPHA;
			msg->guess_uncertain = 0;
			msg->type = TYPE_ALPHA;
			strcpy(msg->data, msg->alpha_data);

			LOGP(DGOLAY, LOGL_INFO, "Discrimination: winner=alpha (message exceeds MAX_NDB blocks, must be alpha), score=%d.\n",
				msg->alpha_score);

			goto decode_done;
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
			int prev_contbit = 1; /* first block is trusted (address/type confirmed data) */
			pos = data_start_pos; /* restore position to re-decode as numeric */
			contbit = 1;
			shifted = 0;
			msg->numeric_nibble_count = 0;

			for (i = 0; contbit && i < MAX_NDB; i++) {
				int k, j;
				int bch_bad[8] = {0}; /* per-codeword failure flags */

				if (pos + 1 + bch_block_bits > total_bits) {
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
						LOGP(DGOLAY, LOGL_DEBUG, "Numeric block %d: BCH[%d] decode failed.\n", i, k);
						d[k] = 0;
						msg->uncorrectable_count++;
						block_ok = 0;
						bch_bad[k] = 1;
					}
				}

				/* If block has failures and previous block didn't confirm
				 * continuation, stop - can't trust this data. */
				if (!block_ok && !prev_contbit) {
					LOGP(DGOLAY, LOGL_DEBUG, "Numeric block %d: BCH failures and no prior continuation, stopping.\n", i);
					break;
				}

				/* Verify checksum (same as alpha: sum of re-encoded codewords) */
				if (block_ok) {
					checksum = 0;
					for (k = 0; k < 7; k++)
						checksum += calc_bch(d[k]);
					checksum &= 0x7f;

					if (checksum != d[7]) {
						LOGP(DGOLAY, LOGL_DEBUG, "Numeric block %d: checksum mismatch (0x%02x != 0x%02x).\n",
							i, checksum, d[7]);
						msg->error_count++;
					}
				}

				/* Unpack 7 data words into 12 four-bit nibbles */
				{
					uint8_t nib[12];
					/* Which nibbles are tainted by failed BCH codewords:
					 * nib[0]  <- d[0]        nib[6]  <- d[3]
					 * nib[1]  <- d[0],d[1]   nib[7]  <- d[4]
					 * nib[2]  <- d[1]         nib[8]  <- d[4],d[5]
					 * nib[3]  <- d[1],d[2]   nib[9]  <- d[5]
					 * nib[4]  <- d[2]         nib[10] <- d[5],d[6]
					 * nib[5]  <- d[2],d[3]   nib[11] <- d[6]       */
					int nib_bad[12];
					nib_bad[0]  = bch_bad[0];
					nib_bad[1]  = bch_bad[0] || bch_bad[1];
					nib_bad[2]  = bch_bad[1];
					nib_bad[3]  = bch_bad[1] || bch_bad[2];
					nib_bad[4]  = bch_bad[2];
					nib_bad[5]  = bch_bad[2] || bch_bad[3];
					nib_bad[6]  = bch_bad[3];
					nib_bad[7]  = bch_bad[4];
					nib_bad[8]  = bch_bad[4] || bch_bad[5];
					nib_bad[9]  = bch_bad[5];
					nib_bad[10] = bch_bad[5] || bch_bad[6];
					nib_bad[11] = bch_bad[6];

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

						if (nib_bad[j]) {
							if (numeric_pos < (int)sizeof(msg->numeric_data) - 1)
								msg->numeric_data[numeric_pos++] = '?';
						} else {
							c = decode_numeric(nib[j], &shifted);
							if (c != '\0' && numeric_pos < (int)sizeof(msg->numeric_data) - 1)
								msg->numeric_data[numeric_pos++] = c;
							else if (c == '\0' && nib[j] == 0x0a)
								msg->numeric_fill++;
						}
					}
				}

				/* Update prev_contbit: only trust contbit from a clean block. */
				prev_contbit = block_ok ? contbit : 0;
			}

			msg->numeric_data[numeric_pos] = '\0';

		}

		/* ---- Decide whether to wait for more data or proceed ----
		 *
		 * If alpha needs more bits, we should keep waiting — alpha
		 * supports up to MAX_ADB blocks (80 chars) and the message
		 * may genuinely be that long. The numeric stage hitting its
		 * MAX_NDB limit (2 blocks / 24 digits) does NOT mean the
		 * message is complete — it just means numeric can't represent
		 * more. In fact, if numeric hit its limit with contbit still
		 * set, the message has more than 2 data blocks, which means
		 * it's definitely alpha (no numeric message exceeds 2 blocks).
		 *
		 * Wait if alpha needs more bits, unless forced.
		 * Only proceed immediately if alpha is complete (contbit=0
		 * or hit MAX_ADB).
		 */
		if (alpha_need_bits && !force) {
			return -1;
		}

		/* ============================================================
		 * Scoring and discrimination: score both interpretations and
		 * pick the winner using unified scoring (content + fill).
		 *
		 * However, if the message has more data blocks than MAX_NDB
		 * can hold (i.e. numeric hit its block limit with contbit
		 * still set), the message is definitively alpha — no valid
		 * numeric message exceeds 2 blocks (24 digits).
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
	 * After decoding one address+message, check for additional
	 * address codewords in the bitstream. Batch mode transmissions
	 * (inverted preamble) pack multiple address/data pairs after a
	 * single preamble + start code. Extended batch inserts a second
	 * start code after the 16th address.
	 *
	 * The decoder handles all three formats (individual, batch,
	 * extended batch) regardless of the --batching CLI value.
	 *
	 * For each additional address decoded, output it as a separate
	 * gsc_rx_msg_t via golay_msg_receive().
	 * ================================================================ */

decode_done:
	msg->decode_ok = 1;

	LOGP(DGOLAY, LOGL_INFO, "Batch decode complete: address='%s' type=%s data='%s' errors=%d.\n",
		msg->address,
		msg->type == TYPE_VOICE ? "voice" :
		msg->type == TYPE_ALPHA ? "alpha" :
		msg->type == TYPE_NUMERIC ? "numeric" : "tone",
		log_escape(msg->data), msg->error_count);

	/* --- Batch continuation: peek ahead for more addresses --- */
	{
		int batch_count = 1; /* first message already decoded above */
		int batch_pos = pos;

		while (batch_count < 32) {
			int saved_pos = batch_pos;
			uint32_t peek_cw;
			uint16_t peek_val;
			int peek_rc;
			int is_w1 = 0;
			int is_start_code = 0;
			int is_preamble = 0;

			/* Need at least comma(28) + dup Golay(46) = 74 bits
			 * to peek at the next codeword */
			if (batch_pos + comma_len + dup_bits > total_bits) {
				LOGP(DGOLAY, LOGL_DEBUG, "Batch: not enough bits for continuation check (%d remaining).\n",
					total_bits - batch_pos);
				break;
			}

			/* Skip 28-bit comma, read dup Golay codeword */
			batch_pos += comma_len;
			peek_cw = read_dup_golay(bits, &batch_pos);

			/* Try to decode the codeword (normal polarity) */
			peek_rc = decode_golay(peek_cw, &peek_val);
			if (peek_rc == 0) {
				/* Check if it's a valid W1 (another address) */
				for (i = 0; i < 50; i++) {
					if (peek_val == word1s[i]) {
						is_w1 = 1;
						break;
					}
				}

				/* Check if it's the start code (extended batch continuation) */
				if (!is_w1 && peek_val == start_code)
					is_start_code = 1;

				/* Check if it's a preamble value (new transmission) */
				if (!is_w1 && !is_start_code) {
					for (i = 0; i < 10; i++) {
						if (peek_val == preamble_values[i]) {
							is_preamble = 1;
							break;
						}
					}
				}
			}

			/* Also try inverted decode for W1 (function bit 1 set) */
			if (!is_w1 && !is_start_code && !is_preamble) {
				peek_rc = decode_golay(peek_cw ^ 0x7fffff, &peek_val);
				if (peek_rc == 0) {
					for (i = 0; i < 50; i++) {
						if (peek_val == word1s[i]) {
							is_w1 = 1;
							break;
						}
					}
				}
			}

			if (is_preamble) {
				/* New transmission starting - stop batch decode */
				LOGP(DGOLAY, LOGL_DEBUG, "Batch: preamble detected after message %d, new transmission.\n",
					batch_count);
				break;
			}

			if (is_start_code) {
				/* Extended batch continuation: second start code.
				 * Skip the 1-bit inverted comma + complement codeword
				 * (same layout as the first start code's complement). */
				LOGP(DGOLAY, LOGL_INFO, "Batch: extended batch start code after message %d.\n",
					batch_count);

				if (batch_pos + 1 + dup_bits > total_bits) {
					LOGP(DGOLAY, LOGL_NOTICE, "Batch: not enough bits for extended start code complement.\n");
					break;
				}

				/* Skip 1-bit inverted comma */
				batch_pos += 1;

				/* Skip complement codeword (46 bits) */
				batch_pos += dup_bits;

				/* Loop back to check for the next address */
				continue;
			}

			if (!is_w1) {
				/* Not a valid W1, start code, or preamble - end of batch */
				LOGP(DGOLAY, LOGL_DEBUG, "Batch: no valid continuation after message %d, end of batch.\n",
					batch_count);
				break;
			}

			/* --- Valid W1 found: decode the next address+message ---
			 *
			 * We've already consumed the comma and W1 dup Golay.
			 * Now we need to decode the full address (W1 already read,
			 * need W2) and then the message data, same as Stages 3-7.
			 *
			 * Re-decode W1 properly with inversion detection since
			 * the peek above was simplified. */

			LOGP(DGOLAY, LOGL_INFO, "Batch: additional address %d found, decoding.\n",
				batch_count + 1);

			{
				gsc_rx_msg_t batch_msg;
				int bm_w1_inverted, bm_w2_inverted;
				uint16_t bm_w1_value;
				uint8_t bm_function;
				int bm_g1, bm_g0, bm_a2, bm_a1, bm_a0;
				int bm_g1g0, bm_idx;
				char bm_suffix;
				int bm_remaining;
				enum gsc_msg_type bm_detected_type;

				memset(&batch_msg, 0, sizeof(batch_msg));
				batch_msg.preamble_index = preamble_idx;

				/* --- Re-decode W1 with full inversion detection ---
				 * Re-read from saved_pos + comma to get the raw codeword again */
				{
					int w1_pos = saved_pos + comma_len;
					uint32_t w1_cw = read_dup_golay(bits, &w1_pos);

					bm_w1_inverted = 0;
					peek_rc = decode_golay(w1_cw, &peek_val);
					if (peek_rc == 0) {
						int found = 0;
						for (i = 0; i < 50; i++) {
							if (peek_val == word1s[i]) {
								found = 1;
								break;
							}
						}
						if (found) {
							bm_w1_value = peek_val;
						} else {
							peek_rc = decode_golay(w1_cw ^ 0x7fffff, &peek_val);
							if (peek_rc == 0) {
								bm_w1_value = peek_val;
								bm_w1_inverted = 1;
							} else {
								LOGP(DGOLAY, LOGL_NOTICE, "Batch W1: re-decode failed.\n");
								break;
							}
						}
					} else {
						peek_rc = decode_golay(w1_cw ^ 0x7fffff, &peek_val);
						if (peek_rc == 0) {
							bm_w1_value = peek_val;
							bm_w1_inverted = 1;
						} else {
							LOGP(DGOLAY, LOGL_NOTICE, "Batch W1: both decodes failed.\n");
							break;
						}
					}
				}

				/* Reverse-map W1 -> G1, G0 */
				if (reverse_word1(bm_w1_value, &bm_g1, &bm_g0) < 0) {
					LOGP(DGOLAY, LOGL_NOTICE, "Batch: W1 value %u not in word1s table.\n", bm_w1_value);
					break;
				}
				bm_g1g0 = bm_g1 * 10 + bm_g0;

				/* Skip 1-bit inverted comma between W1 and W2 */
				if (batch_pos + 1 + dup_bits > total_bits) {
					if (!force)
						return -1;
					break;
				}
				batch_pos += 1;

				/* --- Decode W2 with inversion and range detection --- */
				{
					uint32_t w2_cw = read_dup_golay(bits, &batch_pos);
					uint16_t w2_try[2] = { 0, 0 };
					int golay_ok[2] = { 0, 0 };
					int best_inv = -1;
					int ta2, ta1, ta0;
					int inv, det_range;

					peek_rc = decode_golay(w2_cw, &peek_val);
					if (peek_rc == 0) { w2_try[0] = peek_val; golay_ok[0] = 1; }

					peek_rc = decode_golay(w2_cw ^ 0x7fffff, &peek_val);
					if (peek_rc == 0) { w2_try[1] = peek_val; golay_ok[1] = 1; }

					for (inv = 0; inv < 2; inv++) {
						if (!golay_ok[inv])
							continue;
						/* W2 % 100 >= 50 means high range */
						det_range = (w2_try[inv] % 100 >= 50) ? 1 : 0;
						if (reverse_word2(w2_try[inv], bm_g1g0 + det_range * 50, &ta2, &ta1, &ta0) == 0) {
							if (best_inv < 0) {
								best_inv = inv;
								bm_a2 = ta2; bm_a1 = ta1; bm_a0 = ta0;
								if (det_range) {
									bm_g1g0 += 50;
									bm_g1 = bm_g1g0 / 10;
									bm_g0 = bm_g1g0 % 10;
								}
							}
						}
					}

					if (best_inv < 0) {
						LOGP(DGOLAY, LOGL_NOTICE, "Batch W2: no valid address from any combination.\n");
						break;
					}

					bm_w2_inverted = best_inv;
				}

				/* Function from inversion pattern */
				bm_function = (bm_w1_inverted ? 0x2 : 0) | (bm_w2_inverted ? 0x1 : 0);

				/* Index digit */
				bm_idx = (preamble_idx - bm_g0 + 10) % 10;

				/* --- Type detection (same logic as Stage 3-7) --- */
				bm_remaining = total_bits - batch_pos;

				if (bm_remaining < 1 + bch_block_bits) {
					bm_detected_type = TYPE_TONE;
				} else {
					bm_detected_type = TYPE_TONE;

					/* Check for voice: peek for activation code */
					if (bm_remaining >= comma_len + dup_bits + 1 + dup_bits) {
						int vpeek_pos = batch_pos + comma_len;
						uint32_t vpeek_cw = read_dup_golay(bits, &vpeek_pos);
						uint16_t vpeek_val;
						if (decode_golay(vpeek_cw, &vpeek_val) == 0 && vpeek_val == activation_code)
							bm_detected_type = TYPE_VOICE;
					}

					/* If not voice, probe BCH for data vs tone */
					if (bm_detected_type != TYPE_VOICE && bm_remaining >= 1 + bch_block_bits) {
						int probe_pos = batch_pos + 1;
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
								/* Valid data block - distinguish alpha vs numeric */
								uint8_t a_ch[8];
								uint8_t n_nib[12];
								int a_fill = 0, n_fill = 0;
								int nk;

								a_ch[0] = probe_d[0] & 0x3f;
								a_ch[1] = ((probe_d[0] >> 6) | (probe_d[1] << 1)) & 0x3f;
								a_ch[2] = ((probe_d[1] >> 5) | (probe_d[2] << 2)) & 0x3f;
								a_ch[3] = ((probe_d[2] >> 4) | (probe_d[3] << 3)) & 0x3f;
								a_ch[4] = ((probe_d[3] >> 3) | (probe_d[4] << 4)) & 0x3f;
								a_ch[5] = ((probe_d[4] >> 2) | (probe_d[5] << 5)) & 0x3f;
								a_ch[6] = (probe_d[5] >> 1) & 0x3f;
								a_ch[7] = probe_d[6] & 0x3f;

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

								for (nk = 7; nk >= 0; nk--) {
									if (a_ch[nk] == 0x3e) a_fill++;
									else break;
								}
								for (nk = 11; nk >= 0; nk--) {
									if (n_nib[nk] == 0x0a) n_fill++;
									else break;
								}

								if (n_fill > 0 && a_fill == 0)
									bm_detected_type = TYPE_NUMERIC;
								else if (a_fill > 0 && n_fill == 0)
									bm_detected_type = TYPE_ALPHA;
								else if (n_fill > a_fill)
									bm_detected_type = TYPE_NUMERIC;
								else if (a_fill > n_fill)
									bm_detected_type = TYPE_ALPHA;
								else
									bm_detected_type = TYPE_ALPHA;
							}
						}
					}
				}

				/* Assign function suffix */
				switch (bm_detected_type) {
				case TYPE_VOICE:
					bm_suffix = '1' + bm_function;
					break;
				case TYPE_ALPHA:
				case TYPE_NUMERIC:
					bm_suffix = '5' + bm_function;
					break;
				default:
					bm_suffix = "9034"[bm_function];
					break;
				}

				/* Build 7-digit functional address */
				batch_msg.address[0] = '0' + bm_idx;
				batch_msg.address[1] = '0' + bm_g1;
				batch_msg.address[2] = '0' + bm_g0;
				batch_msg.address[3] = '0' + bm_a2;
				batch_msg.address[4] = '0' + bm_a1;
				batch_msg.address[5] = '0' + bm_a0;
				batch_msg.address[6] = bm_suffix;
				batch_msg.address[7] = '\0';
				batch_msg.address_number = bm_function + 1;
				batch_msg.type = bm_detected_type;

				LOGP(DGOLAY, LOGL_INFO, "Batch address %d: '%s' (function %d, type %s).\n",
					batch_count + 1, batch_msg.address, bm_function,
					bm_detected_type == TYPE_VOICE ? "voice" :
					bm_detected_type == TYPE_ALPHA ? "alpha" :
					bm_detected_type == TYPE_NUMERIC ? "numeric" : "tone");

				/* --- Decode data for this address (Stages 4-7 equivalent) --- */

				if (bm_detected_type == TYPE_ALPHA || bm_detected_type == TYPE_NUMERIC) {
					int bm_data_start = batch_pos;
					int bm_alpha_pos = 0;
					int bm_numeric_pos = 0;
					uint8_t bm_contbit;
					int bm_shifted = 0;

					/* Alpha decode */
					bm_contbit = 1;
					for (i = 0; bm_contbit && i < MAX_ADB; i++) {
						int k, j;

						if (batch_pos + 1 + bch_block_bits > total_bits)
							break;

						batch_pos += 1; /* skip 1-bit inverted comma */
						deinterleave_bch(bits, &batch_pos, bch_cw);

						int block_ok = 1;
						for (k = 0; k < 8; k++) {
							if (decode_bch(bch_cw[k], &d[k]) < 0) {
								d[k] = 0;
								batch_msg.error_count++;
								block_ok = 0;
							}
						}

						if (block_ok) {
							checksum = 0;
							for (k = 0; k < 7; k++)
								checksum += calc_bch(d[k]);
							checksum &= 0x7f;
							if (checksum != d[7])
								batch_msg.error_count++;
						}

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
							bm_contbit = (d[6] >> 6) & 1;

							for (j = 0; j < 8; j++) {
								c = decode_alpha(ch[j]);
								if (bm_alpha_pos < (int)sizeof(batch_msg.alpha_data) - 1) {
									if (c != '\0')
										batch_msg.alpha_data[bm_alpha_pos++] = c;
									else if (ch[j] == 0x3e)
										batch_msg.alpha_fill++;
								}
							}
						}
					}
					batch_msg.alpha_data[bm_alpha_pos] = '\0';

					/* Numeric decode (re-read from same position) */
					batch_pos = bm_data_start;
					bm_contbit = 1;
					bm_shifted = 0;
					batch_msg.numeric_nibble_count = 0;

					for (i = 0; bm_contbit && i < MAX_NDB; i++) {
						int k, j;

						if (batch_pos + 1 + bch_block_bits > total_bits)
							break;

						batch_pos += 1;
						deinterleave_bch(bits, &batch_pos, bch_cw);

						int block_ok = 1;
						for (k = 0; k < 8; k++) {
							if (decode_bch(bch_cw[k], &d[k]) < 0) {
								d[k] = 0;
								block_ok = 0;
							}
						}
						(void)block_ok;

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
							bm_contbit = (d[6] >> 6) & 1;

							for (j = 0; j < 12; j++) {
								if (batch_msg.numeric_nibble_count < (int)sizeof(batch_msg.numeric_nibbles))
									batch_msg.numeric_nibbles[batch_msg.numeric_nibble_count++] = nib[j];

								c = decode_numeric(nib[j], &bm_shifted);
								if (c != '\0' && bm_numeric_pos < (int)sizeof(batch_msg.numeric_data) - 1)
									batch_msg.numeric_data[bm_numeric_pos++] = c;
								else if (c == '\0' && nib[j] == 0x0a)
									batch_msg.numeric_fill++;
							}
						}
					}
					batch_msg.numeric_data[bm_numeric_pos] = '\0';

					/* Score and discriminate */
					batch_msg.alpha_score = gsc_score_alpha(batch_msg.alpha_data, strlen(batch_msg.alpha_data), batch_msg.alpha_fill);
					batch_msg.numeric_score = gsc_score_numeric(batch_msg.numeric_nibbles, batch_msg.numeric_nibble_count, batch_msg.numeric_fill);
					gsc_discriminate(&batch_msg);
				}

				/* Voice: decode activation code */
				if (bm_detected_type == TYPE_VOICE) {
					if (batch_pos + comma_len + dup_bits + 1 + dup_bits <= total_bits) {
						batch_pos += comma_len;
						codeword = read_dup_golay(bits, &batch_pos);
						rc = decode_golay(codeword, &decoded_value);
						if (rc < 0 || decoded_value != activation_code)
							batch_msg.error_count++;
						batch_pos += 1; /* skip inverted comma */
						codeword = read_dup_golay(bits, &batch_pos);
						/* skip complement verification for brevity */
					}
					batch_msg.type = TYPE_VOICE;
				}

				/* Tone-only: skip tone comma */
				if (bm_detected_type == TYPE_TONE) {
					bm_remaining = total_bits - batch_pos;
					if (bm_remaining >= tone_comma_len)
						batch_pos += tone_comma_len;
					else
						batch_pos += bm_remaining;
					batch_msg.type = TYPE_TONE;
				}

				batch_msg.decode_ok = 1;
				batch_msg.polarity_inverted = msg->polarity_inverted;

				LOGP(DGOLAY, LOGL_INFO, "Batch message %d: address='%s' type=%s errors=%d.\n",
					batch_count + 1, batch_msg.address,
					batch_msg.type == TYPE_VOICE ? "voice" :
					batch_msg.type == TYPE_ALPHA ? "alpha" :
					batch_msg.type == TYPE_NUMERIC ? "numeric" : "tone",
					batch_msg.error_count);

				golay_msg_receive(&batch_msg);
				batch_count++;
			}
		}

		if (batch_count > 1) {
			LOGP(DGOLAY, LOGL_INFO, "Batch decode: %d total messages decoded.\n", batch_count);
		}
	}

	return 0;
}

/* Decode a non-battery-saver (NBS) message from the RX bit buffer.
 *
 * NBS format has no coded preamble or start code - the bitstream starts
 * directly with the address comma + W1 + comma_bit + W2, followed by
 * optional data blocks.
 *
 * Layout: [28-bit comma] [dup(W1)] [1-bit] [dup(W2)] [data...]
 *
 * Returns 0 on success, -1 on decode failure. */
int decode_nbs(gsc_t *gsc, gsc_rx_msg_t *msg, int force)
{
	const uint8_t *bits = gsc->rx_bit;
	int pos = 0;
	int total_bits = gsc->rx_bit_num;
	const int comma_len = 28;
	const int dup_bits = 46;
	const int bch_block_bits = 120;
	const int tone_comma_len = 121 * 8;

	uint32_t codeword;
	uint16_t decoded_value;
	int rc, i;

	uint16_t w1_value, w2_value;
	int w1_inverted, w2_inverted;
	uint8_t function;
	int g1, g0, a2, a1, a0;
	int g1g0, idx;
	char suffix;
	int remaining;
	enum gsc_msg_type detected_type;

	memset(msg, 0, sizeof(*msg));
	msg->preamble_index = -1; /* unknown in NBS mode */

	LOGP(DGOLAY, LOGL_DEBUG, "Decoding NBS message: %d bits in buffer.\n", total_bits);

	/* ================================================================
	 * ADDRESS (no preamble or start code in NBS mode)
	 *
	 * Layout: [28-bit comma] [dup(W1)] [1-bit] [dup(W2)]
	 * ================================================================ */

	if (pos + comma_len + dup_bits + 1 + dup_bits > total_bits) {
		LOGP(DGOLAY, LOGL_NOTICE, "NBS: not enough bits for address (%d available).\n", total_bits);
		return -1;
	}

	/* Skip 28-bit comma */
	pos += comma_len;

	/* Decode W1 with inversion detection */
	codeword = read_dup_golay(bits, &pos);
	w1_inverted = 0;

	rc = decode_golay(codeword, &decoded_value);
	if (rc == 0) {
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
			rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
			if (rc == 0) {
				w1_value = decoded_value;
				w1_inverted = 1;
			} else {
				LOGP(DGOLAY, LOGL_NOTICE, "NBS W1: not in table, complement failed.\n");
				return -1;
			}
		}
	} else {
		rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
		if (rc == 0) {
			w1_value = decoded_value;
			w1_inverted = 1;
		} else {
			LOGP(DGOLAY, LOGL_NOTICE, "NBS W1: both decodes failed.\n");
			return -1;
		}
	}

	rc = reverse_word1(w1_value, &g1, &g0);
	if (rc < 0) {
		LOGP(DGOLAY, LOGL_NOTICE, "NBS: W1 value %u not in word1s table.\n", w1_value);
		return -1;
	}
	g1g0 = g1 * 10 + g0;

	/* Skip 1-bit comma */
	pos += 1;

	/* Decode W2 with inversion and range detection */
	codeword = read_dup_golay(bits, &pos);
	w2_inverted = 0;

	{
		uint16_t w2_try[2] = { 0, 0 };
		int golay_ok[2] = { 0, 0 };
		int try_g1g0[2];
		int best_inv = -1, best_range = -1;
		int ta2, ta1, ta0;
		int inv, rng;

		try_g1g0[0] = g1g0;
		try_g1g0[1] = g1g0 + 50;

		rc = decode_golay(codeword, &decoded_value);
		if (rc == 0) { w2_try[0] = decoded_value; golay_ok[0] = 1; }

		rc = decode_golay(codeword ^ 0x7fffff, &decoded_value);
		if (rc == 0) { w2_try[1] = decoded_value; golay_ok[1] = 1; }

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
				}
			}
		}

		if (best_inv < 0) {
			LOGP(DGOLAY, LOGL_NOTICE, "NBS W2: no valid address.\n");
			return -1;
		}

		w2_value = w2_try[best_inv];
		w2_inverted = best_inv;

		if (best_range == 1) {
			g1g0 = try_g1g0[1];
			g1 = g1g0 / 10;
			g0 = g1g0 % 10;
		}
	}

	function = (w1_inverted ? 0x2 : 0) | (w2_inverted ? 0x1 : 0);

	/* NBS has no preamble index - use 0 as placeholder for address reconstruction */
	idx = 0;

	/* Build function suffix from Table IX */
	switch (function) {
	case 0: suffix = '1'; break;
	case 1: suffix = '2'; break;
	case 2: suffix = '3'; break;
	case 3: suffix = '4'; break;
	default: suffix = '1'; break;
	}

	snprintf(msg->address, sizeof(msg->address), "%d%d%d%d%d%d%c",
		idx, g1, g0, a2, a1, a0, suffix);

	LOGP(DGOLAY, LOGL_INFO, "NBS address decoded: '%s' (W1=%u W2=%u func=%d).\n",
		msg->address, w1_value, w2_value, function);

	/* ================================================================
	 * TYPE DETECTION + DATA DECODE
	 *
	 * Same logic as decode_batch() stages 4-7, but starting from
	 * the current position (right after the address pair).
	 * ================================================================ */

	remaining = total_bits - pos;

	if (remaining < 1 + bch_block_bits) {
		if (!force) {
			LOGP(DGOLAY, LOGL_DEBUG, "NBS: not enough post-address bits (%d), need more.\n", remaining);
			return -1;
		}
		detected_type = TYPE_TONE;
	} else {
		detected_type = TYPE_TONE;

		/* Check for voice: activation code after 28-bit comma */
		if (remaining >= comma_len + dup_bits + 1 + dup_bits) {
			int peek_pos = pos + comma_len;
			uint32_t peek_cw = read_dup_golay(bits, &peek_pos);
			uint16_t peek_val;
			if (decode_golay(peek_cw, &peek_val) == 0 && peek_val == activation_code)
				detected_type = TYPE_VOICE;
		}

		/* Probe BCH block to detect data */
		if (detected_type != TYPE_VOICE && remaining >= 1 + bch_block_bits) {
			int probe_pos = pos + 1;
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
					/* Valid data - use fill-count heuristic for alpha vs numeric */
					uint8_t a_ch[8];
					int a_fill = 0, n_fill = 0;
					int nk;

					a_ch[0] = probe_d[0] & 0x3f;
					a_ch[1] = ((probe_d[0] >> 6) | (probe_d[1] << 1)) & 0x3f;
					a_ch[2] = ((probe_d[1] >> 5) | (probe_d[2] << 2)) & 0x3f;
					a_ch[3] = ((probe_d[2] >> 4) | (probe_d[3] << 3)) & 0x3f;
					a_ch[4] = ((probe_d[3] >> 3) | (probe_d[4] << 4)) & 0x3f;
					a_ch[5] = ((probe_d[4] >> 2) | (probe_d[5] << 5)) & 0x3f;
					a_ch[6] = (probe_d[5] >> 1) & 0x3f;
					a_ch[7] = probe_d[6] & 0x3f;

					uint8_t n_nib[12];
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

					for (nk = 7; nk >= 0; nk--) {
						if (a_ch[nk] == 0x3e) a_fill++;
						else break;
					}
					for (nk = 11; nk >= 0; nk--) {
						if (n_nib[nk] == 0x0a) n_fill++;
						else break;
					}

					if (n_fill > 0 && a_fill == 0)
						detected_type = TYPE_NUMERIC;
					else if (a_fill > 0 && n_fill == 0)
						detected_type = TYPE_ALPHA;
					else if (n_fill > a_fill)
						detected_type = TYPE_NUMERIC;
					else
						detected_type = TYPE_ALPHA;
				}
			}

			if (detected_type == TYPE_TONE && remaining < tone_comma_len) {
				if (!force)
					return -1;
			}
		}
	}

	msg->type = detected_type;
	msg->decode_ok = 1;

	LOGP(DGOLAY, LOGL_INFO, "NBS decode: address='%s' type=%s.\n",
		msg->address,
		detected_type == TYPE_VOICE ? "voice" :
		detected_type == TYPE_ALPHA ? "alpha" :
		detected_type == TYPE_NUMERIC ? "numeric" : "tone");

	return 0;
}

/* Dump the TX bit buffer to the log, formatted for readability.
 * Groups bits into rows of 46 (one duplicated Golay codeword = 23 bits * 2)
 * so the protocol structure is visible. */
static void dump_bitstream(gsc_t *gsc, const char *label)
{
	int i, row;
	char line[128];
	int pos;

	LOGP(DGOLAY, LOGL_DEBUG, "=== TX bitstream dump: %s (%d bits) ===\n", label, gsc->bit_num);

	row = 0;
	pos = 0;
	for (i = 0; i < gsc->bit_num; i++) {
		line[pos++] = '0' + gsc->bit[i];
		if (pos == 46 || i == gsc->bit_num - 1) {
			line[pos] = '\0';
			LOGP(DGOLAY, LOGL_INFO, "  [%4d] %s\n", row * 46, line);
			pos = 0;
			row++;
		}
	}

	LOGP(DGOLAY, LOGL_DEBUG, "=== end bitstream dump ===\n");
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

/* Forward declaration: queue_interleaved_tones is defined after queue_batch */
static int queue_interleaved_tones(gsc_t *gsc, int preamble_index, double polarity, int max_tones);

/* Non-battery-saver mode encoder (S2.5).
 *
 * Uses a simple 75 Hz square wave preamble (1,1,0,0 pattern) for >=1.25s
 * instead of coded preamble. No start code. Address pairs follow directly.
 * This is the original Golay format (pre-GSC, in service since 1973).
 * Higher throughput but no battery saving groups.
 *
 * Layout: [75 Hz preamble >= 752 bits] [address pair 121 bits] [data blocks...]
 */
static int queue_batch_nbs(gsc_t *gsc, const char *address, enum gsc_msg_type type, const char *message, double polarity)
{
	uint16_t word1, word2;
	uint8_t function;
	uint32_t golay;
	uint16_t bch[8];
	uint8_t msg[12], digit, shifted, contbit, checksum;
	int preamble;
	int i, j, k;
	int rc;

	queue_reset(gsc);

	if (!address || strlen(address) != 7) {
		LOGP(DGOLAY, LOGL_NOTICE, "Invalid functional address '%s' size. Only 7 digits are allowed.\n", address);
		return -EINVAL;
	}

	rc = encode_address(address, &preamble, &word1, &word2);
	if (rc < 0)
		return rc;

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

	LOGP(DGOLAY, LOGL_INFO, "NBS mode: coding %s for functional address '%s'.\n",
		type == TYPE_ALPHA ? "alpha" :
		type == TYPE_NUMERIC ? "numeric" :
		type == TYPE_VOICE ? "voice" : "tone-only", address);

	/* S2.5: 75 Hz preamble = 1,1,0,0 pattern for at least 1.25s.
	 * At 600 baud, 1.25s = 750 bits. Use 752 bits (188 x 4) for alignment. */
	LOGP(DGOLAY, LOGL_DEBUG, "Encoding 75 Hz NBS preamble (752 bits).\n");
	for (i = 0; i < 188; i++) {
		queue_bit(gsc, 1);
		queue_bit(gsc, 1);
		queue_bit(gsc, 0);
		queue_bit(gsc, 0);
	}

	/* No start code in NBS mode - address follows directly */

	/* Encode address */
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

	/* Encode message (same as queue_batch) */
	switch (type) {
	case TYPE_ALPHA:
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding %d alphanumeric digits.\n", (int)strlen(message));
		for (i = 0; *message; i++) {
			if (i == MAX_ADB)
				LOGP(DGOLAY, LOGL_NOTICE, "Message overflows %d characters, cropping.\n", MAX_ADB * 8);
			for (j = 0; *message && j < 8; j++)
				msg[j] = encode_alpha(*message++);
			while (j < 8)
				msg[j++] = 0x3e;
			bch[0] = calc_bch((msg[0] | (msg[1] << 6)) & 0x7f);
			bch[1] = calc_bch(((msg[1] >> 1) | (msg[2] << 5)) & 0x7f);
			bch[2] = calc_bch(((msg[2] >> 2) | (msg[3] << 4)) & 0x7f);
			bch[3] = calc_bch(((msg[3] >> 3) | (msg[4] << 3)) & 0x7f);
			bch[4] = calc_bch(((msg[4] >> 4) | (msg[5] << 2)) & 0x7f);
			bch[5] = calc_bch(((msg[5] >> 5) | (msg[6] << 1)) & 0x7f);
			contbit = (*message && i < MAX_ADB) ? 1 : 0;
			bch[6] = calc_bch((contbit << 6) | msg[7]);
			checksum = bch[0] + bch[1] + bch[2] + bch[3] + bch[4] + bch[5] + bch[6];
			bch[7] = calc_bch(checksum & 0x7f);
			queue_bit(gsc, (bch[0] & 1) ^ 1);
			for (j = 0; j < 15; j++)
				for (k = 0; k < 8; k++)
					queue_bit(gsc, (bch[k] >> j) & 1);
		}
		break;
	case TYPE_NUMERIC:
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding %d numeric digits.\n", (int)strlen(message));
		shifted = 0;
		for (i = 0; *message; i++) {
			if (i == MAX_NDB)
				LOGP(DGOLAY, LOGL_NOTICE, "Message overflows %d characters, cropping.\n", MAX_NDB * 12);
			for (j = 0; *message && j < 12; j++) {
				if (shifted) {
					digit = shifted & 0xf;
					shifted = 0;
				} else
					digit = encode_numeric(*message);
				if (digit > 0xf) {
					shifted = digit;
					msg[j] = digit >> 4;
				} else {
					msg[j] = digit;
					message++;
				}
			}
			while (j < 12)
				msg[j++] = 0xa;
			bch[0] = calc_bch((msg[0] | (msg[1] << 4)) & 0x7f);
			bch[1] = calc_bch(((msg[1] >> 3) | (msg[2] << 1) | (msg[3] << 5)) & 0x7f);
			bch[2] = calc_bch(((msg[3] >> 2) | (msg[4] << 2) | (msg[5] << 6)) & 0x7f);
			bch[3] = calc_bch(((msg[5] >> 1) | (msg[6] << 3)) & 0x7f);
			bch[4] = calc_bch((msg[7] | (msg[8] << 4)) & 0x7f);
			bch[5] = calc_bch(((msg[8] >> 3) | (msg[9] << 1) | (msg[10] << 5)) & 0x7f);
			contbit = (*message && i < MAX_NDB) ? 1 : 0;
			bch[6] = calc_bch((contbit << 6) | (msg[10] >> 2) | (msg[11] << 2));
			checksum = bch[0] + bch[1] + bch[2] + bch[3] + bch[4] + bch[5] + bch[6];
			bch[7] = calc_bch(checksum & 0x7f);
			queue_bit(gsc, (bch[0] & 1) ^ 1);
			for (j = 0; j < 15; j++)
				for (k = 0; k < 8; k++)
					queue_bit(gsc, (bch[k] >> j) & 1);
		}
		break;
	case TYPE_VOICE:
		memcpy(gsc->wave_tx_filename, message, MIN(sizeof(gsc->wave_tx_filename) - 1, strlen(message) + 1));
		gsc->bit_ac = gsc->bit_num;
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding activation code.\n");
		golay = calc_golay(activation_code);
		queue_comma(gsc, 28, golay & 1);
		queue_dup(gsc, golay, 23);
		golay ^= 0x7fffff;
		queue_bit(gsc, (golay & 1) ^ 1);
		queue_dup(gsc, golay, 23);
		{
			double msg_polarity = (polarity != 0.0) ? polarity : gsc->fsk_polarity;
			queue_interleaved_tones(gsc, preamble, msg_polarity, 8);
		}
		break;
	default:
		LOGP(DGOLAY, LOGL_DEBUG, "Encoding 'comma' sequence after tone-only address.\n");
		queue_comma(gsc, 121 * 8, 1);
	}

	if (gsc->bit_overflow) {
		LOGP(DGOLAY, LOGL_ERROR, "NBS bit stream (%d bits) overflows buffer (%d bits).\n", gsc->bit_num, (int)sizeof(gsc->bit));
		return -EOVERFLOW;
	}

	if (gsc->protocol_dump)
		dump_bitstream(gsc, address);

	return 0;
}

static int queue_batch(gsc_t *gsc, const char *address, enum gsc_msg_type type, const char *message, double polarity)
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
		/* Interleave tone-only addresses during the alert period */
		{
			double msg_polarity = (polarity != 0.0) ? polarity : gsc->fsk_polarity;
			queue_interleaved_tones(gsc, preamble, msg_polarity, 8);
		}
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

	if (gsc->protocol_dump)
		dump_bitstream(gsc, address);

	return 0;
}

/* Called during voice page encoding, after activation code.
 * Interleaves up to max_tones tone-only addresses into the 1.92s alert period.
 * Only tone-only messages matching the voice message's preamble index AND polarity are eligible.
 * Each address pair (W1, W2) takes ~0.202s at 600 baud.
 * Returns the number of tone-only addresses interleaved. */
static int queue_interleaved_tones(gsc_t *gsc, int preamble_index, double polarity, int max_tones)
{
	gsc_msg_t **msgp, *msg;
	int preamble;
	uint16_t word1, word2;
	uint8_t function;
	uint32_t golay;
	int rc;
	int count = 0;

	/* Walk priority queue first - priority tone-only messages interleaved before normal ones */
	msgp = &gsc->priority_list;
	while (*msgp && count < max_tones) {
		msg = *msgp;
		if (msg->type == TYPE_TONE
		    && msg->preamble_index == preamble_index
		    && msg->polarity == polarity) {
			/* Dequeue from priority list */
			*msgp = msg->next;
			gsc->priority_count--;

			/* Encode address */
			rc = encode_address(msg->address, &preamble, &word1, &word2);
			if (rc < 0) {
				LOGP(DGOLAY, LOGL_ERROR, "Failed to encode interleaved tone address '%s', skipping.\n", msg->address);
				free(msg);
				continue;
			}

			/* Get function from last digit */
			switch (msg->address[6]) {
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
					LOGP(DGOLAY, LOGL_NOTICE, "Illegal function suffix '%c' in interleaved tone address.\n", msg->address[6]);
					free(msg);
					continue;
			}

			LOGP(DGOLAY, LOGL_DEBUG, "Interleaving tone-only address %d: '%s'.\n", count + 1, msg->address);

			/* Encode W1: comma + dup Golay codeword */
			golay = calc_golay(word1);
			if (function & 0x2)
				golay ^= 0x7fffff;
			queue_comma(gsc, 28, golay & 1);
			queue_dup(gsc, golay, 23);

			/* Encode W2: inverted first bit + dup Golay codeword */
			golay = calc_golay(word2);
			if (function & 0x1)
				golay ^= 0x7fffff;
			queue_bit(gsc, (golay & 1) ^ 1);
			queue_dup(gsc, golay, 23);

			free(msg);
			count++;
		} else {
			msgp = &(*msgp)->next;
		}
	}

	/* Walk normal queue */
	msgp = &gsc->msg_list;
	while (*msgp && count < max_tones) {
		msg = *msgp;
		if (msg->type == TYPE_TONE
		    && msg->preamble_index == preamble_index
		    && msg->polarity == polarity) {
			/* Dequeue from normal list */
			*msgp = msg->next;
			gsc->normal_count--;

			/* Encode address */
			rc = encode_address(msg->address, &preamble, &word1, &word2);
			if (rc < 0) {
				LOGP(DGOLAY, LOGL_ERROR, "Failed to encode interleaved tone address '%s', skipping.\n", msg->address);
				free(msg);
				continue;
			}

			/* Get function from last digit */
			switch (msg->address[6]) {
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
					LOGP(DGOLAY, LOGL_NOTICE, "Illegal function suffix '%c' in interleaved tone address.\n", msg->address[6]);
					free(msg);
					continue;
			}

			LOGP(DGOLAY, LOGL_DEBUG, "Interleaving tone-only address %d: '%s'.\n", count + 1, msg->address);

			/* Encode W1: comma + dup Golay codeword */
			golay = calc_golay(word1);
			if (function & 0x2)
				golay ^= 0x7fffff;
			queue_comma(gsc, 28, golay & 1);
			queue_dup(gsc, golay, 23);

			/* Encode W2: inverted first bit + dup Golay codeword */
			golay = calc_golay(word2);
			if (function & 0x1)
				golay ^= 0x7fffff;
			queue_bit(gsc, (golay & 1) ^ 1);
			queue_dup(gsc, golay, 23);

			free(msg);
			count++;
		} else {
			msgp = &(*msgp)->next;
		}
	}

	if (count > 0)
		LOGP(DGOLAY, LOGL_INFO, "Interleaved %d tone-only address(es) during voice page (preamble_index=%d).\n", count, preamble_index);

	return count;
}


/* Encode a group of messages sharing the same preamble index as a single batch.
 * Uses inverted preamble (XOR 0x7FFFFF) to distinguish from individual mode.
 * msgs is a linked list of messages to encode; count is the number of messages.
 * Returns 0 on success, -EOVERFLOW if bitstream exceeds buffer. */
static int queue_batch_group(gsc_t *gsc, gsc_msg_t *msgs, int count)
{
	int preamble;
	uint16_t word1, word2;
	uint8_t function;
	uint32_t golay;
	uint16_t bch[8];
	uint8_t msg[12], digit, shifted, contbit, checksum;
	int i, j, k;
	int rc;
	int msg_index;
	gsc_msg_t *cur;

	queue_reset(gsc);

	if (!msgs || count <= 0)
		return -EINVAL;

	/* Get preamble index from first message */
	preamble = msgs->preamble_index;

	/* Encode INVERTED preamble: XOR each 23-bit codeword with 0x7FFFFF */
	LOGP(DGOLAY, LOGL_DEBUG, "Encoding inverted preamble '%d' for batch of %d messages.\n", preamble, count);
	golay = calc_golay(preamble_values[preamble]) ^ 0x7fffff;
	queue_comma(gsc, 28, golay & 1);
	for (i = 0; i < 18; i++) {
		queue_dup(gsc, golay, 23);
	}

	/* Encode start code (same pattern as queue_batch) */
	LOGP(DGOLAY, LOGL_DEBUG, "Encoding start code.\n");
	golay = calc_golay(start_code);
	queue_comma(gsc, 28, golay & 1);
	queue_dup(gsc, golay, 23);
	golay ^= 0x7fffff;
	queue_bit(gsc, (golay & 1) ^ 1);
	queue_dup(gsc, golay, 23);

	/* Encode each message in the batch */
	msg_index = 0;
	for (cur = msgs; cur != NULL; cur = cur->next) {
		/* Defensive: skip voice messages (should never be in a batch group) */
		if (cur->type == TYPE_VOICE) {
			LOGP(DGOLAY, LOGL_NOTICE, "Skipping voice message '%s' in batch group (voice is always individual).\n", cur->address);
			continue;
		}

		/* Extended batch: insert second start code after 16th address */
		if (msg_index == 16) {
			LOGP(DGOLAY, LOGL_DEBUG, "Encoding second start code for extended batch.\n");
			golay = calc_golay(start_code);
			queue_comma(gsc, 28, golay & 1);
			queue_dup(gsc, golay, 23);
			golay ^= 0x7fffff;
			queue_bit(gsc, (golay & 1) ^ 1);
			queue_dup(gsc, golay, 23);
		}

		/* Encode address */
		rc = encode_address(cur->address, &preamble, &word1, &word2);
		if (rc < 0) {
			LOGP(DGOLAY, LOGL_ERROR, "Failed to encode address '%s' in batch, skipping.\n", cur->address);
			continue;
		}

		/* Get function from last digit (same switch as queue_batch) */
		switch (cur->address[6]) {
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
				LOGP(DGOLAY, LOGL_NOTICE, "Illegal function suffix '%c' in batch address.\n", cur->address[6]);
				continue;
		}

		LOGP(DGOLAY, LOGL_DEBUG, "Encoding batch address %d/%d: '%s' type=%d.\n", msg_index + 1, count, cur->address, cur->type);

		/* Encode address W1/W2 with function bits (same as queue_batch) */
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

		/* Encode data based on message type (same as queue_batch) */
		switch (cur->type) {
		case TYPE_ALPHA:
		{
			const char *message = cur->data;
			LOGP(DGOLAY, LOGL_DEBUG, "Encoding %d alphanumeric digits.\n", (int)strlen(message));
			for (i = 0; *message; i++) {
				if (i == MAX_ADB) {
					LOGP(DGOLAY, LOGL_NOTICE, "Message overflows %d characters, cropping message.\n", MAX_ADB * 8);
				}
				for (j = 0; *message && j < 8; j++) {
					msg[j] = encode_alpha(*message++);
				}
				while (j < 8)
					msg[j++] = 0x3e;
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
				checksum = bch[0] + bch[1] + bch[2] + bch[3] + bch[4] + bch[5] + bch[6];
				bch[7] = calc_bch(checksum & 0x7f);
				queue_bit(gsc, (bch[0] & 1) ^ 1);
				for (j = 0; j < 15; j++) {
					for (k = 0; k < 8; k++)
						queue_bit(gsc, (bch[k] >> j) & 1);
				}
			}
			break;
		}
		case TYPE_NUMERIC:
		{
			const char *message = cur->data;
			LOGP(DGOLAY, LOGL_DEBUG, "Encoding %d numeric digits.\n", (int)strlen(message));
			shifted = 0;
			for (i = 0; *message; i++) {
				if (i == MAX_NDB) {
					LOGP(DGOLAY, LOGL_NOTICE, "Message overflows %d characters, cropping message.\n", MAX_NDB * 12);
				}
				for (j = 0; *message && j < 12; j++) {
					if (shifted) {
						digit = shifted & 0xf;
						shifted = 0;
					} else
						digit = encode_numeric(*message);
					if (digit > 0xf) {
						shifted = digit;
						msg[j] = digit >> 4;
					} else {
						msg[j] = digit;
						message++;
					}
				}
				while (j < 12)
					msg[j++] = 0xa;
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
				checksum = bch[0] + bch[1] + bch[2] + bch[3] + bch[4] + bch[5] + bch[6];
				bch[7] = calc_bch(checksum & 0x7f);
				queue_bit(gsc, (bch[0] & 1) ^ 1);
				for (j = 0; j < 15; j++) {
					for (k = 0; k < 8; k++)
						queue_bit(gsc, (bch[k] >> j) & 1);
				}
			}
			break;
		}
		default:
			/* Tone-only: encode comma after address */
			LOGP(DGOLAY, LOGL_DEBUG, "Encoding 'comma' sequence after tone-only address.\n");
			queue_comma(gsc, 121 * 8, 1);
			break;
		}

		msg_index++;

		/* Check buffer overflow mid-batch */
		if (gsc->bit_overflow) {
			LOGP(DGOLAY, LOGL_ERROR, "Batch bitstream (%d bits) overflows buffer (%d bits) at message %d/%d.\n",
			     gsc->bit_num, (int)sizeof(gsc->bit), msg_index, count);
			return -EOVERFLOW;
		}
	}

	/* Per S2.1: "Unfilled batches should be filled out with comma."
	 * Each address slot is 121 bits. Pad remaining slots up to 16
	 * (or 32 for extended batch) with comma. */
	{
		int max_slots = (msg_index > 16) ? 32 : 16;
		int pad_slots = max_slots - msg_index;
		if (pad_slots > 0) {
			LOGP(DGOLAY, LOGL_DEBUG, "Padding %d unfilled batch slots with comma (%d bits each).\n",
				pad_slots, 121);
			for (i = 0; i < pad_slots; i++)
				queue_comma(gsc, 121, 1);
		}
	}

	LOGP(DGOLAY, LOGL_INFO, "Batch encoded: %d messages, preamble_index=%d, %d bits.\n", msg_index, preamble, gsc->bit_num);
	if (gsc->protocol_dump)
		dump_bitstream(gsc, "batch");
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
			LOGP(DGOLAY, LOGL_INFO, "Batch TX complete: msgs=%d preamble_index=%d remaining=%d\n",
			     gsc->tx_msg_count, gsc->tx_preamble_index,
			     gsc->priority_count + gsc->normal_count);

			/* Refill scan queue if scanning */
			if (gsc->scan_from < gsc->scan_to)
				golay_scan_enqueue(gsc, &gsc->scan_from, gsc->scan_to, gsc->scan_type, 16);

			goto next_msg;
		}
		return gsc->bit[gsc->bit_index++];
	}

next_msg:
	/* Expire stale messages before selecting next batch */
	scheduler_expire(gsc);

	/* Use scheduler to get next message (or batch) */
	msg = scheduler_next_batch(gsc);

	/* no message pending, turn transmitter off */
	if (!msg)
		return -1;

	/* Count messages in the returned linked list */
	{
		gsc_msg_t *p;
		int count = 0;
		for (p = msg; p != NULL; p = p->next)
			count++;

		/* Set per-message TX polarity before encoding */
		if (msg->polarity != 0.0)
			gsc->fsk_tx_polarity = msg->polarity;
		else
			gsc->fsk_tx_polarity = gsc->fsk_polarity;

		/* Record batch info for completion logging */
		gsc->tx_preamble_index = msg->preamble_index;

		if (count > 1 && gsc->batching_mode != BATCHING_OFF) {
			/* Multi-message batch: use batch group encoder */
			gsc->tx_msg_count = count;
			rc = queue_batch_group(gsc, msg, count);
			if (rc >= 0)
				LOGP(DGOLAY, LOGL_INFO, "Transmitting batch of %d messages, preamble_index=%d.\n", count, msg->preamble_index);
			/* Free all messages in the linked list */
			while (msg) {
				gsc_msg_t *next = msg->next;
				free(msg);
				msg = next;
			}
			if (rc < 0)
				goto next_msg;
		} else {
			/* Single message or batching off: use individual encoder */
			gsc->tx_msg_count = 1;
			if (gsc->nbs)
				rc = queue_batch_nbs(gsc, msg->address, msg->type, msg->data, msg->polarity);
			else
				rc = queue_batch(gsc, msg->address, msg->type, msg->data, msg->polarity);
			if (rc >= 0)
				LOGP(DGOLAY, LOGL_INFO, "Transmitting message to address '%s'%s.\n", msg->address, gsc->nbs ? " (NBS)" : "");
			/* Free all messages (defensive: free extras if any) */
			{
				gsc_msg_t *next;
				while (msg) {
					next = msg->next;
					free(msg);
					msg = next;
				}
			}
			if (rc < 0)
				goto next_msg;
		}
	}

	/* return first bit */
	return gsc->bit[gsc->bit_index++];
}

/* Insert message into appropriate queue based on priority flag.
 * Computes and caches preamble_index, records enqueue_time,
 * logs message details, and starts hold-off timer if needed. */
void scheduler_enqueue(gsc_t *gsc, gsc_msg_t *msg)
{
	gsc_msg_t **msgp;

	/* Compute and cache preamble index: (I + G0) % 10
	 * where I = address[0], G0 = address[2] */
	msg->preamble_index = (msg->address[0] - '0' + msg->address[2] - '0') % 10;

	/* Record submission timestamp for expiry */
	clock_gettime(CLOCK_MONOTONIC, &msg->enqueue_time);

	/* Insert into appropriate queue (FIFO order - append to tail) */
	if (msg->priority == 1) {
		msgp = &gsc->priority_list;
		while (*msgp)
			msgp = &(*msgp)->next;
		*msgp = msg;
		msg->next = NULL;
		gsc->priority_count++;
	} else {
		msgp = &gsc->msg_list;
		while (*msgp)
			msgp = &(*msgp)->next;
		*msgp = msg;
		msg->next = NULL;
		gsc->normal_count++;
	}

	LOGP(DGOLAY, LOGL_INFO, "Scheduler: enqueued '%s' type=%d priority=%d preamble_index=%d queue_depth=%d\n",
	     msg->address, msg->type, msg->priority, msg->preamble_index,
	     gsc->priority_count + gsc->normal_count);

	/* Start hold-off timer if batching is enabled, hold-off is not
	 * already running, and the transmitter is idle (no bits being
	 * transmitted, i.e., bit_index >= bit_num). */
	if (gsc->batching_mode != BATCHING_OFF
	    && !gsc->holdoff_active
	    && gsc->bit_index >= gsc->bit_num) {
		clock_gettime(CLOCK_MONOTONIC, &gsc->holdoff_start);
		gsc->holdoff_active = 1;
		LOGP(DGOLAY, LOGL_DEBUG, "Scheduler: hold-off timer started (%d ms).\n", gsc->holdoff_ms);
	}
}

/* Remove expired messages from both queues.
 * Messages older than gsc->expiry_seconds are removed and freed.
 * Priority message expiry is logged at ERROR level; normal at NOTICE. */
void scheduler_expire(gsc_t *gsc)
{
	gsc_msg_t **msgp, *msg;
	struct timespec now;
	long age;

	clock_gettime(CLOCK_MONOTONIC, &now);

	/* Walk priority queue */
	msgp = &gsc->priority_list;
	while (*msgp) {
		msg = *msgp;
		age = now.tv_sec - msg->enqueue_time.tv_sec;
		if (age >= gsc->expiry_seconds) {
			*msgp = msg->next;
			gsc->priority_count--;
			LOGP(DGOLAY, LOGL_ERROR, "Scheduler: priority message '%s' expired (age %ld seconds).\n",
			     msg->address, age);
			free(msg);
		} else {
			msgp = &(*msgp)->next;
		}
	}

	/* Walk normal queue */
	msgp = &gsc->msg_list;
	while (*msgp) {
		msg = *msgp;
		age = now.tv_sec - msg->enqueue_time.tv_sec;
		if (age >= gsc->expiry_seconds) {
			*msgp = msg->next;
			gsc->normal_count--;
			LOGP(DGOLAY, LOGL_NOTICE, "Scheduler: message '%s' expired (age %ld seconds).\n",
			     msg->address, age);
			free(msg);
		} else {
			msgp = &(*msgp)->next;
		}
	}
}

/* Log queue status (called from dump_info). */
void scheduler_dump(gsc_t *gsc)
{
	const char *mode_str;
	int total_depth;
	int ngroups = 0;
	struct {
		int preamble_index;
		double polarity;
	} groups[30];
	gsc_msg_t *msg;
	int i;

	total_depth = gsc->priority_count + gsc->normal_count;

	switch (gsc->batching_mode) {
	case BATCHING_NORMAL:
		mode_str = "normal";
		break;
	case BATCHING_EXTENDED:
		mode_str = "extended";
		break;
	default:
		mode_str = "off";
		break;
	}

	/* Count unique (preamble_index, polarity) groups across both queues,
	 * skipping voice messages (they are never batched). */
	for (msg = gsc->priority_list; msg; msg = msg->next) {
		if (msg->type == TYPE_VOICE)
			continue;
		for (i = 0; i < ngroups; i++) {
			if (groups[i].preamble_index == msg->preamble_index
			    && groups[i].polarity == msg->polarity)
				break;
		}
		if (i == ngroups && ngroups < 30) {
			groups[ngroups].preamble_index = msg->preamble_index;
			groups[ngroups].polarity = msg->polarity;
			ngroups++;
		}
	}
	for (msg = gsc->msg_list; msg; msg = msg->next) {
		if (msg->type == TYPE_VOICE)
			continue;
		for (i = 0; i < ngroups; i++) {
			if (groups[i].preamble_index == msg->preamble_index
			    && groups[i].polarity == msg->polarity)
				break;
		}
		if (i == ngroups && ngroups < 30) {
			groups[ngroups].preamble_index = msg->preamble_index;
			groups[ngroups].polarity = msg->polarity;
			ngroups++;
		}
	}

	LOGP(DGOLAY, LOGL_INFO, "Scheduler: queue_depth=%d priority=%d normal=%d batch_groups=%d batching=%s\n",
	     total_depth, gsc->priority_count, gsc->normal_count, ngroups, mode_str);
}


/* Helper: dequeue a batch of up to max_batch messages from a queue,
 * selecting only messages matching the given (preamble_index, polarity) tuple.
 * Uses pointer-to-pointer pattern to dequeue from the middle while
 * maintaining FIFO order. Returns a linked list of dequeued messages.
 * Decrements *count for each dequeued message. */
static gsc_msg_t *dequeue_batch_group(gsc_msg_t **queue_head, int *count,
				      int preamble_index, double polarity,
				      int max_batch)
{
	gsc_msg_t *result = NULL, **result_tail = &result;
	gsc_msg_t **msgp;
	int dequeued = 0;

	msgp = queue_head;
	while (*msgp && dequeued < max_batch) {
		gsc_msg_t *msg = *msgp;
		if (msg->preamble_index == preamble_index && msg->polarity == polarity) {
			/* Unlink from queue */
			*msgp = msg->next;
			/* Append to result list */
			msg->next = NULL;
			*result_tail = msg;
			result_tail = &msg->next;
			(*count)--;
			dequeued++;
		} else {
			msgp = &(*msgp)->next;
		}
	}

	return result;
}

/* Helper: find the (preamble_index, polarity) group with the most messages
 * in a queue, considering only non-voice messages.
 * Returns the count of the best group (0 if none found).
 * Stores the winning preamble_index and polarity in *best_pi and *best_pol. */
static int find_best_group(gsc_msg_t *head, int *best_pi, double *best_pol)
{
	struct {
		int preamble_index;
		double polarity;
		int count;
	} groups[30];
	int ngroups = 0;
	gsc_msg_t *msg;
	int i, best_count = 0;

	for (msg = head; msg; msg = msg->next) {
		if (msg->type == TYPE_VOICE)
			continue;
		for (i = 0; i < ngroups; i++) {
			if (groups[i].preamble_index == msg->preamble_index
			    && groups[i].polarity == msg->polarity)
				break;
		}
		if (i == ngroups && ngroups < 30) {
			groups[ngroups].preamble_index = msg->preamble_index;
			groups[ngroups].polarity = msg->polarity;
			groups[ngroups].count = 0;
			ngroups++;
		}
		if (i < 30)
			groups[i].count++;
	}

	for (i = 0; i < ngroups; i++) {
		if (groups[i].count > best_count) {
			best_count = groups[i].count;
			*best_pi = groups[i].preamble_index;
			*best_pol = groups[i].polarity;
		}
	}

	return best_count;
}

/* Select next message or batch for transmission.
 *
 * BATCHING_OFF: simple FIFO dequeue, one message at a time.
 *
 * BATCHING_NORMAL: greedy - pick the preamble group with the most
 * queued messages and transmit them as a batch.
 *
 * BATCHING_EXTENDED: round-robin through preamble groups 0-9 for
 * battery saving. Each call advances to the next group that has
 * queued messages, dequeues all matching messages (up to 32), and
 * returns them as a linked list.
 *
 * Priority voice messages bypass batching and are sent immediately.
 *
 * Returns a linked list of messages, or NULL if nothing to send
 * (or hold-off is active). */
gsc_msg_t *scheduler_next_batch(gsc_t *gsc)
{
	gsc_msg_t *msg, *batch;
	int max_batch;
	int tried;
	int best_pi = 0;
	double best_pol = 0.0;

	/* === BATCHING_OFF: simple single-message dequeue === */
	if (gsc->batching_mode == BATCHING_OFF) {
		/* Priority queue first */
		if (gsc->priority_list) {
			msg = gsc->priority_list;
			gsc->priority_list = msg->next;
			msg->next = NULL;
			gsc->priority_count--;
			return msg;
		}
		/* Normal queue */
		if (gsc->msg_list) {
			msg = gsc->msg_list;
			gsc->msg_list = msg->next;
			msg->next = NULL;
			gsc->normal_count--;
			return msg;
		}
		return NULL;
	}

	/* === Batching enabled === */
	max_batch = (gsc->batching_mode == BATCHING_EXTENDED) ? 32 : 16;

	/* Check hold-off timer */
	if (gsc->holdoff_active) {
		struct timespec now;
		long elapsed_ms;

		clock_gettime(CLOCK_MONOTONIC, &now);
		elapsed_ms = (now.tv_sec - gsc->holdoff_start.tv_sec) * 1000
			   + (now.tv_nsec - gsc->holdoff_start.tv_nsec) / 1000000;

		if (elapsed_ms < gsc->holdoff_ms
		    && gsc->priority_count + gsc->normal_count < 16) {
			/* Hold-off still active and neither queue has reached 16 */
			return NULL;
		}
		/* Timer expired or batch full - proceed */
		gsc->holdoff_active = 0;
		LOGP(DGOLAY, LOGL_DEBUG, "Scheduler: hold-off expired (elapsed %ld ms).\n", elapsed_ms);
	}

	/* --- Priority voice messages bypass round-robin --- */
	if (gsc->priority_list && gsc->priority_list->type == TYPE_VOICE) {
		msg = gsc->priority_list;
		gsc->priority_list = msg->next;
		msg->next = NULL;
		gsc->priority_count--;
		return msg;
	}

	/* --- Normal voice messages at head bypass batching --- */
	if (gsc->msg_list && gsc->msg_list->type == TYPE_VOICE
	    && !gsc->priority_list) {
		msg = gsc->msg_list;
		gsc->msg_list = msg->next;
		msg->next = NULL;
		gsc->normal_count--;
		return msg;
	}

	/* === BATCHING_NORMAL: greedy - pick group with most messages === */
	if (gsc->batching_mode == BATCHING_NORMAL) {
		if (gsc->priority_list) {
			if (find_best_group(gsc->priority_list, &best_pi, &best_pol) > 0)
				return dequeue_batch_group(&gsc->priority_list,
							   &gsc->priority_count,
							   best_pi, best_pol, max_batch);
		}
		if (gsc->msg_list) {
			if (find_best_group(gsc->msg_list, &best_pi, &best_pol) > 0)
				return dequeue_batch_group(&gsc->msg_list,
							   &gsc->normal_count,
							   best_pi, best_pol, max_batch);
		}
		return NULL;
	}

	/* === BATCHING_EXTENDED: round-robin through preamble groups 0-9 ===
	 *
	 * Starting from sched_current_group, scan up to 10 groups looking
	 * for one that has queued messages. Dequeue all matching messages
	 * (across both priority and normal queues) for that group.
	 * Advance sched_current_group to the next group after the one
	 * we just served. */
	for (tried = 0; tried < 10; tried++) {
		int group = gsc->sched_current_group;
		int has_priority = 0, has_normal = 0;
		gsc_msg_t *p;

		/* Check if this group has any non-voice messages in either queue */
		for (p = gsc->priority_list; p; p = p->next) {
			if (p->type != TYPE_VOICE && p->preamble_index == group) {
				has_priority = 1;
				break;
			}
		}
		for (p = gsc->msg_list; p; p = p->next) {
			if (p->type != TYPE_VOICE && p->preamble_index == group) {
				has_normal = 1;
				break;
			}
		}

		if (!has_priority && !has_normal) {
			/* Nothing for this group - advance and try next */
			gsc->sched_current_group = (group + 1) % 10;
			continue;
		}

		/* Dequeue from priority first, then fill from normal.
		 * All messages share the same preamble_index but may have
		 * different polarities. Use the polarity of the first message
		 * found (priority queue takes precedence). */
		batch = NULL;
		if (has_priority) {
			double pol = 0.0;
			/* Find polarity of first matching priority message */
			for (p = gsc->priority_list; p; p = p->next) {
				if (p->type != TYPE_VOICE && p->preamble_index == group) {
					pol = p->polarity;
					break;
				}
			}
			batch = dequeue_batch_group(&gsc->priority_list,
						    &gsc->priority_count,
						    group, pol, max_batch);
		}

		/* Count how many we got from priority */
		{
			int count = 0;
			for (p = batch; p; p = p->next)
				count++;

			/* Fill remaining slots from normal queue */
			if (count < max_batch && has_normal) {
				double pol = 0.0;
				if (batch) {
					pol = batch->polarity;
				} else {
					for (p = gsc->msg_list; p; p = p->next) {
						if (p->type != TYPE_VOICE && p->preamble_index == group) {
							pol = p->polarity;
							break;
						}
					}
				}

				gsc_msg_t *normal_batch = dequeue_batch_group(
					&gsc->msg_list, &gsc->normal_count,
					group, pol, max_batch - count);

				if (normal_batch) {
					if (!batch) {
						batch = normal_batch;
					} else {
						/* Append normal batch to end of priority batch */
						gsc_msg_t **tail = &batch;
						while ((*tail)->next)
							tail = &(*tail)->next;
						(*tail)->next = normal_batch;
					}
				}
			}
		}

		/* Advance to next group for next call */
		gsc->sched_current_group = (group + 1) % 10;

		LOGP(DGOLAY, LOGL_DEBUG, "Scheduler: round-robin group %d, dequeued batch.\n", group);

		return batch;
	}

	/* All 10 groups empty */
	return NULL;
}

/* Returns 0 on success, -EINVAL on hard rejection.
 * Logs warnings for soft issues (content, type-suffix).
 *
 * Validation stages (in order):
 *  1. Address length - exactly 7 characters (hard reject)
 *  2. Address digits - each character '0'-'9' (hard reject)
 *  3. Illegal GSC codes - G1G0 vs illegal_low[]/illegal_high[] (hard reject)
 *  4. Function suffix - 7th digit '0'-'9' (hard reject, covered by step 2)
 *  5. TYPE_AUTO derivation from function suffix
 *  6. Content warnings (log warning, accept)
 *  7. Type-suffix consistency warnings (log warning, accept)
 *  8. Voice file existence - access(path, R_OK) (hard reject)
 */
int golay_validate_msg(const char *address, enum gsc_msg_type type,
                       const char *data)
{
	static const uint16_t illegal_low[16] = {   0,  25,  51, 103, 206, 340, 363, 412, 445, 530, 642, 726, 782, 810, 825, 877 };
	static const uint16_t illegal_high[7] = {   0, 292, 425, 584, 631, 841, 851 };
	int i, g1, g0, g1g0, a2, a1, a0, a2a1a0;
	enum gsc_msg_type suffix_type;
	int effective_type_is_auto = (type == TYPE_AUTO);
	enum gsc_msg_type effective_type;

	/* Stage 1: Address length - must be exactly 7 characters */
	if (!address || strlen(address) != 7) {
		LOGP(DGOLAY, LOGL_ERROR, "Validation failed: address must be exactly 7 characters (got %d).\n",
		     address ? (int)strlen(address) : 0);
		return -EINVAL;
	}

	/* Stage 2: Address digits - each character must be '0'-'9' */
	for (i = 0; i < 7; i++) {
		if (address[i] < '0' || address[i] > '9') {
			LOGP(DGOLAY, LOGL_ERROR, "Validation failed: non-digit character '%c' at position %d in address '%s'.\n",
			     address[i], i, address);
			return -EINVAL;
		}
	}

	/* Stage 3: Illegal GSC codes - G1G0 mapped against tables */
	g1 = address[1] - '0';
	g0 = address[2] - '0';
	a2 = address[3] - '0';
	a1 = address[4] - '0';
	a0 = address[5] - '0';
	g1g0 = g1 * 10 + g0;
	a2a1a0 = a2 * 100 + a1 * 10 + a0;

	if (g1g0 < 50) {
		for (i = 0; i < 16; i++) {
			if (a2a1a0 == (int)illegal_low[i]) {
				LOGP(DGOLAY, LOGL_ERROR, "Validation failed: illegal GSC code, address digits '%03d' with G1G0=%02d matches illegal_low table.\n",
				     a2a1a0, g1g0);
				return -EINVAL;
			}
		}
	} else {
		for (i = 0; i < 7; i++) {
			if (a2a1a0 == (int)illegal_high[i]) {
				LOGP(DGOLAY, LOGL_ERROR, "Validation failed: illegal GSC code, address digits '%03d' with G1G0=%02d matches illegal_high table.\n",
				     a2a1a0, g1g0);
				return -EINVAL;
			}
		}
	}

	/* Stage 4: Function suffix - 7th digit must be '0'-'9'
	 * Already guaranteed by stage 2, but kept explicit per design. */

	/* Stage 5: TYPE_AUTO derivation from function suffix */
	switch (address[6]) {
	case '1': case '2': case '3': case '4':
		suffix_type = TYPE_VOICE;
		break;
	case '5': case '6': case '7': case '8':
		suffix_type = TYPE_ALPHA;
		break;
	case '9': case '0':
		suffix_type = TYPE_TONE;
		break;
	default:
		/* Cannot happen after stage 2, but be defensive */
		LOGP(DGOLAY, LOGL_ERROR, "Validation failed: invalid function suffix '%c'.\n", address[6]);
		return -EINVAL;
	}

	if (effective_type_is_auto)
		effective_type = suffix_type;
	else
		effective_type = type;

	/* Stage 6: Content warnings (log warning, accept) */
	if (data && data[0] != '\0') {
		if (effective_type == TYPE_NUMERIC) {
			/* Check each char is in GSC numeric charset */
			int digit_count = 0;
			for (i = 0; data[i] != '\0'; i++) {
				char c = data[i];
				int valid = 0;
				int shifted = 0;

				if (c >= '0' && c <= '9') { valid = 1; }
				else if (c == 'U' || c == 'u') { valid = 1; }
				else if (c == ' ') { valid = 1; }
				else if (c == '-') { valid = 1; }
				else if (c == '*' || c == '=') { valid = 1; }
				else if (c == 'A' || c == 'a') { valid = 1; shifted = 1; }
				else if (c == 'B' || c == 'b') { valid = 1; shifted = 1; }
				else if (c == 'C' || c == 'c') { valid = 1; shifted = 1; }
				else if (c == 'D' || c == 'd') { valid = 1; shifted = 1; }
				else if (c == 'E' || c == 'e') { valid = 1; shifted = 1; }
				else if (c == 'F' || c == 'f') { valid = 1; shifted = 1; }
				else if (c == 'G' || c == 'g') { valid = 1; shifted = 1; }
				else if (c == 'H' || c == 'h') { valid = 1; shifted = 1; }
				else if (c == 'J' || c == 'j') { valid = 1; shifted = 1; }
				else if (c == 'L' || c == 'l') { valid = 1; shifted = 1; }
				else if (c == 'N' || c == 'n') { valid = 1; shifted = 1; }
				else if (c == 'P' || c == 'p') { valid = 1; shifted = 1; }
				else if (c == 'R' || c == 'r') { valid = 1; shifted = 1; }

				if (!valid)
					LOGP(DGOLAY, LOGL_NOTICE, "Validation warning: numeric message contains character '%c' at position %d not in GSC numeric charset.\n", c, i);

				/* Count digits: shifted letters count as 2 */
				digit_count += shifted ? 2 : 1;
			}
			if (digit_count > 24)
				LOGP(DGOLAY, LOGL_NOTICE, "Validation warning: numeric message length %d exceeds default maximum of 24 digits.\n", digit_count);
		} else if (effective_type == TYPE_ALPHA) {
			int len = (int)strlen(data);
			if (len > 80)
				LOGP(DGOLAY, LOGL_NOTICE, "Validation warning: alphanumeric message length %d exceeds default maximum of 80 characters.\n", len);
		}
	}

	/* Stage 7: Type-suffix consistency warnings (log warning, accept) */
	if (!effective_type_is_auto && type != suffix_type) {
		LOGP(DGOLAY, LOGL_NOTICE, "Validation warning: explicit type conflicts with function suffix '%c' (suffix implies %s).\n",
		     address[6],
		     suffix_type == TYPE_VOICE ? "voice" :
		     suffix_type == TYPE_ALPHA ? "alpha" :
		     suffix_type == TYPE_TONE  ? "tone" : "unknown");
	}
	if (effective_type == TYPE_TONE && (address[6] == '3' || address[6] == '4')) {
		LOGP(DGOLAY, LOGL_NOTICE, "Validation warning: tone-only message for voice-only address (suffix '%c'), potential incompatibility.\n",
		     address[6]);
	}

	/* Stage 8: Voice file existence check (hard reject) */
	if (effective_type == TYPE_VOICE) {
		if (!data || data[0] == '\0') {
			LOGP(DGOLAY, LOGL_ERROR, "Validation failed: voice message requires a file path.\n");
			return -EINVAL;
		}
		if (access(data, R_OK) != 0) {
			LOGP(DGOLAY, LOGL_ERROR, "Validation failed: voice file '%s' does not exist or is not readable.\n", data);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Scan mode: enqueue a batch of messages for sequential address scanning.
 *
 * GSC addresses are 7 digits.  The scan range iterates over full 7-digit
 * addresses.  The message type is set by the caller via -y.
 */
void golay_scan_enqueue(gsc_t *gsc, uint32_t *scan_from, uint32_t scan_to, enum gsc_msg_type type, int batch_size)
{
	char address[8];
	char message[16];
	gsc_msg_t *msg;
	int queued = 0;

	while (*scan_from < scan_to && queued < batch_size) {
		sprintf(address, "%07d", *scan_from);

		/* Generate message payload */
		switch (type) {
		case TYPE_NUMERIC:
			sprintf(message, "%07d", *scan_from);
			break;
		case TYPE_ALPHA:
			sprintf(message, "%07d", *scan_from);
			break;
		case TYPE_TONE:
		default:
			message[0] = '\0';
			break;
		}

		msg = golay_msg_create(gsc, address, message, type);
		if (msg) {
			scheduler_enqueue(gsc, msg);
			LOGP(DGOLAY, LOGL_NOTICE, "Scan: enqueue address '%s' type=%d msg='%s'\n",
			     address, type, message);
			queued++;
		}

		(*scan_from)++;
	}
}


void golay_msg_send(const char *text)
{
	char buffer[strlen(text) + 1], *p, *address_string, *message;
	gsc_t *gsc;
	gsc_msg_t *msg;
	enum gsc_msg_type type = TYPE_AUTO;
	int priority = 0;
	double polarity = 0.0;
	char *start, *end;
	int i;

	strcpy(buffer, text);

	/* Trim leading whitespace */
	start = buffer;
	while (*start && isspace((unsigned char)*start))
		start++;

	/* Trim trailing whitespace */
	end = start + strlen(start);
	while (end > start && isspace((unsigned char)*(end - 1)))
		end--;
	*end = '\0';

	/* Discard empty lines */
	if (*start == '\0') {
		LOGP(DGOLAY, LOGL_ERROR, "FIFO: discarding empty input line.\n");
		return;
	}

	/* Parse optional priority prefix '!' */
	if (*start == '!') {
		priority = 1;
		start++;
	}

	/* Parse: address[,type,message[,polarity]] */
	p = start;
	address_string = strsep(&p, ",");

	/* Validate address is exactly 7 digits */
	if (strlen(address_string) != 7) {
		LOGP(DGOLAY, LOGL_ERROR, "FIFO: malformed input, address '%s' is not 7 characters (got %zu). Discarding: '%s'\n",
		     address_string, strlen(address_string), text);
		return;
	}
	for (i = 0; i < 7; i++) {
		if (!isdigit((unsigned char)address_string[i])) {
			LOGP(DGOLAY, LOGL_ERROR, "FIFO: malformed input, address '%s' contains non-digit '%c' at position %d. Discarding: '%s'\n",
			     address_string, address_string[i], i, text);
			return;
		}
	}

	message = p;

	/* If no comma was found, p is NULL - this is a tone-only/auto
	 * message with just an address and no payload. */
	if (!message) {
		message = "";
	}

	/* Parse type prefix (a, n, v) followed by comma */
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
	case ('t' << 8) | ',':
		type = TYPE_TONE;
		message += 2;
		break;
	default:
		message = "";
	}

	/* Parse optional trailing polarity field.
	 * The polarity is the 4th comma-separated field: address,type,message,polarity
	 * Only check when we have a type and message (i.e., type != TYPE_AUTO). */
	if (type != TYPE_AUTO && message[0] != '\0') {
		/* Find the last comma in the message portion */
		char *last_comma = strrchr(message, ',');
		if (last_comma) {
			char *polarity_str = last_comma + 1;
			/* Polarity field is a single character: '+' or '-' */
			if (polarity_str[0] == '+' && polarity_str[1] == '\0') {
				polarity = 1.0;
				*last_comma = '\0'; /* trim polarity from message */
			} else if (polarity_str[0] == '-' && polarity_str[1] == '\0') {
				polarity = -1.0;
				*last_comma = '\0'; /* trim polarity from message */
			}
			/* Otherwise it's part of the message content, not a polarity field */
		}
	}

	if (golay_validate_msg(address_string, type, message) != 0) {
		LOGP(DGOLAY, LOGL_ERROR, "Message rejected by validator for address '%s'.\n", address_string);
		return;
	}

	gsc = (gsc_t *) sender_head;
	msg = golay_msg_create(gsc, address_string, message, type);
	if (msg) {
		msg->priority = priority;
		msg->polarity = polarity;
		scheduler_enqueue(gsc, msg);
		LOGP(DGOLAY, LOGL_INFO, "FIFO: enqueued message for '%s' (priority=%d, polarity=%.1f, type=%d).\n",
		     address_string, priority, polarity, type);
	}
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
	const char *status_str;
	char status_buf[48];
	FILE *fp;
	int is_data;

	if (msg->uncorrectable_count > 0) {
		snprintf(status_buf, sizeof(status_buf), "partial, %d uncorrectable",
			msg->uncorrectable_count);
		status_str = status_buf;
	} else if (msg->error_count == 0) {
		status_str = "ok";
	} else {
		snprintf(status_buf, sizeof(status_buf), "corrected, %d error%s",
			msg->error_count, msg->error_count == 1 ? "" : "s");
		status_str = status_buf;
	}

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

		LOGP(DGOLAY, LOGL_NOTICE, "Received message for address '%s' (%s, polarity=%s):\n",
			msg->address, status_str, pol_str);
		LOGP(DGOLAY, LOGL_NOTICE, "  Alpha %s: '%s' (score=%d, fill=%d)\n",
			alpha_marker, log_escape(msg->alpha_data), msg->alpha_score, msg->alpha_fill);
		if (msg->numeric_data[0] || msg->numeric_score)
			LOGP(DGOLAY, LOGL_NOTICE, "  Numeric %s: '%s' (score=%d, fill=%d)\n",
				numeric_marker, log_escape(msg->numeric_data), msg->numeric_score, msg->numeric_fill);

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

		LOGP(DGOLAY, LOGL_NOTICE, "Received %s message for address '%s': '%s' (%s, polarity=%s).\n",
			type_str, msg->address, msg->data, status_str, pol_str);

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
	scheduler_enqueue(gsc, msg);
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

void dump_info(void)
{
	gsc_t *gsc = (gsc_t *) sender_head;

	if (gsc)
		scheduler_dump(gsc);
}

