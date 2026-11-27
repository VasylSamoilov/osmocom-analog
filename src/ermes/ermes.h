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
#include "queue.h"
#include "book.h"
#include "numbers.h"
#include "ssi.h"
#include "scheduler.h"

/* One transmitter (channel): sources -> queue -> scheduler -> batch builder -> modulator */
typedef struct ermes {
	sender_t		sender;
	int			channel;		/* 1..16 */
	ermes_network_t		net;

	/* position of the batch being sent: counted on from the start, 2500 codewords = 12 s */
	int			cycle, ssn, slot;
	int			started;
	double			clock_start;		/* network time of the first batch (s since the epoch) */
	uint64_t		cw_sent;		/* codewords sent since */
	int			ssit_next;		/* batch counter of the --ssi test schedules */
	int			skip_bits;		/* bits of the first batch already past at start (UTC lock) */


	/* modulator */
	fsk_nrz_mod_t		fsk_mod;
	sample_t		*tx_buffer;
	int			tx_buffer_size, tx_buffer_length, tx_buffer_pos;
	uint8_t			batch_bits[ERMES_BATCH_CW_LAST * ERMES_CW_BITS];
	ermes_carry_t		carry;			/* rest of a message running on into the next batches */
} ermes_t;

int ermes_create(const char *kanal, int channel, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, double polarity, const ermes_network_t *net, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback);
void ermes_destroy(sender_t *sender);

/* Queue a page from a FIFO or --page line, as flex and pocsag: "address,type,options,message"
 * (the message is the rest of the line); the address decimal, 0x hex or nec-<label code> */
int ermes_msg_send(const char *line);

/* 1 if nothing is queued */
int ermes_idle(void);

/* Number check of the console: 7 digit decimal local address */
const char *ermes_number_valid(const char *number);
