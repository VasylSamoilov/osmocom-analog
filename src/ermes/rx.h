#ifndef ERMES_RX_H
#define ERMES_RX_H

/* ERMES receiver, the protocol part: dibits in, batches and messages out (ETS 300 133-4).
 *
 * A monitor, not a pager: every batch and every message is decoded, whatever its address,
 * with or without an initial address, in any order. What a pager would do differently (no
 * initial address, not in ascending order) is reported, not acted on. No samples here: the DSP
 * (dsp.c) turns samples into dibits, the unit test feeds the encoder's own bits and captures. */

#include <stdint.h>
#include "code.h"
#include "frame.h"

#define ERMES_RX_MAX_MEMBERS	32
#define ERMES_RX_TEXT		4096
#define ERMES_RX_MAX_WORDS	(ERMES_MAX_MSG_WORDS + 8)	/* header, OPID, AIF, data */

/* what a batch carried: system information and address partition */
typedef struct ermes_rx_batch {
	int		si_ok;		/* both SI words decoded */
	int		country, operator, pa, eti, bai, fsi;
	int		cycle, ssn, batch;	/* batch type 0..15 = A..P */
	int		ssi_ok, ssit, ssif;
	int		length;		/* codewords: 154 or 190 */
	int		ia_count;	/* initial addresses, I */
	uint32_t	ia[ERMES_MAX_IA];
	int		ia_bad;		/* initial addresses not decodable */
	int		apt, apt_expected; /* J found, J by the formula (§5.4, -5 §7) */
	int		codeblocks;	/* K */
	int		corrected;	/* bits corrected in the batch */
	int		bad;		/* codewords not decodable */
	int		sync_errors;	/* bits off in preamble and SW1 */
	int		flywheel;	/* sync not found where expected, batch taken on timing */
	int		fill;		/* codewords before the first delimiter that were fill */
	int		not_ascending;	/* messages not in ascending order of local address */
	int		unreadable;	/* runs of codewords between delimiters without a header */
} ermes_rx_batch_t;

enum ermes_rx_kind {
	ERMES_RX_MESSAGE,	/* an individual message (AII 0, or AIT 100 AIN 0010 codepage) */
	ERMES_RX_POINTER,	/* group call stage 2: AII 1 AIT 111, AIN = CTAP (§10.3) */
	ERMES_RX_GROUP,		/* group call stage 3: message to a common temporary address */
	ERMES_RX_LAST_NUMBER,	/* AII 1 AIT 100 AIN 0001 (-2 §4.3.1.3.3) */
	ERMES_RX_SUBMESSAGE,	/* long message part: AII 1 AIT 000/001/110/101, link counter (§10.4) */
	ERMES_RX_OTHER,		/* any other additional information (remote programming, reserved) */
};

typedef struct ermes_rx_msg {
	enum ermes_rx_kind kind;
	uint32_t	local_address;
	int		number;
	int		external, opid;	/* EB, and the OPID that follows the header (§5.5.2) */
	int		ait, ain;	/* with AII 1 */
	int		category, urgent, alert;
	int		link_counter;	/* submessage: bits still to come (§10.4) */
	char		text[ERMES_RX_TEXT];	/* UTF-8 */
	int		eom;		/* alpha: the end of message character was found */
	int		truncated;	/* text cut where the standard says to stop (-5 §7.5, §7.6) */
	int		words;		/* codewords, header included */
	int		corrected;	/* bits corrected */
	int		bad;		/* codewords not decodable */
	int		batches;	/* batches it spans (§10.5) */
	int		no_ia;		/* its initial address was not in its first batch */
	int		members;	/* group text: the pointers seen for its CTAP (12 s) */
	uint32_t	member[ERMES_RX_MAX_MEMBERS];
	int		member_number[ERMES_RX_MAX_MEMBERS];
} ermes_rx_msg_t;

typedef void (*ermes_rx_batch_cb_t)(void *priv, const ermes_rx_batch_t *batch);
typedef void (*ermes_rx_msg_cb_t)(void *priv, const ermes_rx_batch_t *batch, const ermes_rx_msg_t *msg);

typedef struct ermes_rx {
	int		channel;	/* 1..16: the long batch from the slot order; 0: found by the next sync */
	void		*priv;
	ermes_rx_batch_cb_t batch_cb;
	ermes_rx_msg_cb_t msg_cb;
	const struct pager_charset *charset;

	/* bit level */
	uint64_t	shift;		/* the last 60 bits */
	uint64_t	bit_count;
	int		locked;
	int		inverted_seen;
	uint8_t		bits[ERMES_BATCH_CW_LAST * ERMES_CW_BITS + 60];
	int		nbits;		/* bits of the batch collected, preamble and SW1 included */
	int		length;		/* codewords of the batch, 0 = not known yet */
	int		sync_errors;
	int		flywheel;
	int		lost;		/* batches in a row taken without sync */

	/* a message running on into the next batch (§10.5) */
	int		pending;
	uint32_t	pend_info[ERMES_RX_MAX_WORDS];
	int8_t		pend_err[ERMES_RX_MAX_WORDS];
	int		pend_words;
	int		pend_batches;
	int		pend_no_ia;

	/* group calls: the pointers per CTAP, 12 s (§10.8.2) */
	struct {
		int		n;
		uint32_t	la[ERMES_RX_MAX_MEMBERS];
		int		number[ERMES_RX_MAX_MEMBERS];
		uint64_t	bit[ERMES_RX_MAX_MEMBERS];
	} ctap[16];

	/* counts */
	int		batches, messages;
} ermes_rx_t;

/* channel 1..16 (0: not known), charset: the network's codepage for alphanumeric text (a pager
 * is told another only per message, Table 15) */
void ermes_rx_init(ermes_rx_t *rx, int channel, const struct pager_charset *charset, void *priv,
		   ermes_rx_batch_cb_t batch_cb, ermes_rx_msg_cb_t msg_cb);
/* one symbol: two bits, first bit high (00 01 11 10 = lowest to highest deviation) */
void ermes_rx_dibit(ermes_rx_t *rx, int dibit);
void ermes_rx_bit(ermes_rx_t *rx, int bit);
/* "message to 0x318E40 number 4 alpha ..." */
const char *ermes_rx_msg_str(const ermes_rx_msg_t *msg);

#endif /* ERMES_RX_H */
