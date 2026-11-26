/* ERMES (European Radio Message System) transmitter
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
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
 */

#include "../libmobile/sender.h"
#include "../libfsk/fsk_nrz.h"
#include "frame.h"

#define ERMES_MAX_TEXT	256

/* --ssi: the SSI types by turns, the 13-value cycle of edition 2, or one SSIT (ERMES_SSIT_*) in every batch */
#define ERMES_SSI_ALTERNATE	-1
#define ERMES_SSI_CYCLE		-2
#define ERMES_SSI_STRICT	-3	/* 0000 0001 by turns every batch, from a batch counter */
#define ERMES_SSI_PAIRS		-4	/* 0000 0000 0001 0001 ... */
#define ERMES_SSI_HOLD		-5	/* one type for --ssi-hold subsequences, then the other */
extern int ermes_ssi_hold;
extern int ermes_ssi_mode;

/* --clock: seconds between the network clock and the system clock */
extern double ermes_clock_offset;

/* --tx-latency: seconds from making a sample to its transmission (< 0: the DSP buffer) */
extern double ermes_tx_latency;

typedef struct ermes_msg {
	struct ermes_msg	*next;
	ermes_page_t		page;
	char			text[ERMES_MAX_TEXT];
} ermes_msg_t;

typedef struct ermes {
	sender_t		sender;
	int			channel;		/* 1..16 */
	ermes_network_t		net;

	/* position of the batch being sent: counted on from the start, 2500 codewords = 12 s */
	int			cycle, ssn, slot;
	int			started;
	double			clock_start;		/* network time of the first batch (s since the epoch) */
	uint64_t		cw_sent;		/* codewords sent since */
	int			ssit_next;		/* --ssi cycle: SSIT of the next batch */
	int			skip_bits;		/* bits of the first batch already past at start (UTC lock) */

	/* queue */
	ermes_msg_t		*msg_list;

	/* modulator */
	fsk_nrz_mod_t		fsk_mod;
	sample_t		*tx_buffer;
	int			tx_buffer_size, tx_buffer_length, tx_buffer_pos;
	uint8_t			batch_bits[ERMES_BATCH_CW_LAST * ERMES_CW_BITS];
} ermes_t;

int ermes_create(const char *kanal, int channel, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, double polarity, const ermes_network_t *net, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback);
void ermes_destroy(sender_t *sender);

/* Queue a page: "<local address>[,tone|numeric|alpha[,<text>]]"; the address is decimal or 0x hex */
int ermes_msg_send(const char *line);

/* Queue a page given as values. Returns 0 or -1 (not queued, logged) */
/* How a page is sent: paging category, UMI and alert type (VIF, Fig.11 / Table 9 p.19) */
typedef struct ermes_page_opts {
	enum ermes_category	category;
	int			urgent;		/* UMI */
	int			alert;		/* 0..7 */
	const struct pager_charset *charset;	/* alphanumeric codepage, NULL = --charset */
} ermes_page_opts_t;

extern ermes_page_opts_t ermes_default_opts;

/* Options field of a FIFO line (as flex): "urgent=1 alert=3 charset=..." onto opts. 0 or -1 (logged) */
int ermes_parse_page_opts(const char *options, ermes_page_opts_t *opts);

int ermes_page_queue(ermes_t *ermes, uint32_t local_address, const ermes_page_opts_t *opts, const char *text);

/* 1 if nothing is queued */
int ermes_idle(void);

/* Number check of the console: 7 digit decimal local address */
const char *ermes_number_valid(const char *number);
