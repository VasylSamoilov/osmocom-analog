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
#include "../libfilter/iir_filter.h"
#include "frame.h"
#include "queue.h"
#include "book.h"
#include "numbers.h"
#include "ssi.h"
#include "scheduler.h"

/* A common temporary address pointer (CTAP) of a channel: free -> addressing (stage 1 and 2: the
 * members get their initial address and the pointer, each in its batch, or with --group-fig16
 * both in one) -> message (stage 3 runs on into further batches) -> hold (the members may still
 * search, §10.8.2: no reuse until 12 s after the last member's batch) -> free */
enum ermes_ctap_state {
	ERMES_CTAP_FREE = 0,
	ERMES_CTAP_ADDRESSING,
	ERMES_CTAP_MESSAGE,
	ERMES_CTAP_HOLD,
};

typedef struct ermes_ctap_slot {
	enum ermes_ctap_state	state;
	ermes_msg_t		*msg;		/* the group call (addressing) */
	int			members;	/* its receivers on this channel */
	int			ia_left;	/* whose stage 1 (initial address) is still to send */
	int			left;		/* whose stage 2 (pointer) is still to send */
	uint64_t		last_pointer_at;	/* the batch of the last stage 2 (scheduler), 0 = none */
	double			first, last;	/* network time of the first and last member's initial address */
	double			free_at;	/* hold: network time it becomes free */
	/* its receivers on this channel: they get no other message while it is addressing or
	 * message; in hold they do */
	uint32_t		member[ERMES_GROUP_MAX_MEMBERS];
} ermes_ctap_slot_t;

const char *ermes_ctap_state_name(enum ermes_ctap_state state);

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
	double			polarity;		/* 1.0 normal, -1.0 inverted */
	iir_filter_t		premod[5];		/* premodulation Bessel lowpass (§8.3.3), 10th order */
	double			premod_symbol_samples;	/* samples a symbol at its (oversampled) rate */
	double			premod_phase;		/* the fraction of a sample carried on */
	unsigned int		premod_decimate;	/* the oversampled samples, to take every n-th */
	double			premod_delay;		/* its delay in s (UTC lock) */
	sample_t		*premod_buffer;
	sample_t		*tx_buffer;
	int			tx_buffer_size, tx_buffer_length, tx_buffer_pos;
	uint8_t			batch_bits[ERMES_BATCH_CW_LAST * ERMES_CW_BITS];
	ermes_carry_t		carry;			/* rest of a message running on into the next batches */

	/* group calls (§10.3): the 16 common temporary address pointers of this channel, each a
	 * state machine (scheduler.c); one call is addressed at a time */
	ermes_ctap_slot_t	ctap[ERMES_CTAP_COUNT];
	int			group_ctap;		/* the slot of the call being sent, -1 = none */
	int			ctap_next;		/* where the search for a free one starts */
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
