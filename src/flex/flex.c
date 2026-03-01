/* FLEX pager transmitter processing
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

#define CHAN flex->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include <osmocom/cc/message.h>
#include "flex.h"
#include "frame.h"
#include "dsp.h"

static const char *flex_state_name[] = {
	"IDLE",
	"PREAMBLE",
	"MESSAGE",
};

static const char *flex_msg_type_names[] = {
	"auto",
	"tone",
	"numeric",
	"alpha",
};

const char *flex_msg_type_name(enum flex_msg_type type)
{
	if (type >= 0 && type < 4)
		return flex_msg_type_names[type];
	return "unknown";
}

static const char *print_capcode(flex_msg_t *msg)
{
	static char text[32];

	sprintf(text, "%" PRIu64, msg->capcode);

	return text;
}

static void flex_display_status(void)
{
	sender_t *sender;
	flex_t *flex;
	flex_msg_t *msg;

	display_status_start();
	for (sender = sender_head; sender; sender = sender->next) {
		flex = (flex_t *) sender;
		display_status_channel(flex->sender.kanal, NULL, flex_state_name[flex->state]);
		for (msg = flex->msg_list; msg; msg = msg->next)
			display_status_subscriber(print_capcode(msg), NULL);
	}
	display_status_end();
}

static void flex_new_state(flex_t *flex, enum flex_state new_state)
{
	if (flex->state == new_state)
		return;
	LOGP(DFLEX, LOGL_DEBUG, "State change: %s -> %s\n", flex_state_name[flex->state], flex_state_name[new_state]);
	flex->state = new_state;
	flex_display_status();
}

/*
 * Create msg instance.
 */
flex_msg_t *flex_msg_create(flex_t *flex, uint64_t capcode,
			    enum flex_msg_type msg_type,
			    const char *data, int data_length)
{
	flex_msg_t *msg, **msgp;

	LOGP(DFLEX, LOGL_INFO, "Creating msg instance to page capcode '%" PRIu64 "' / type '%s'.\n",
	     capcode, flex_msg_type_name(msg_type));

	/* create */
	msg = calloc(1, sizeof(*msg));
	if (!msg) {
		LOGP(DFLEX, LOGL_ERROR, "No mem!\n");
		abort();
	}
	if ((size_t)data_length > sizeof(msg->data)) {
		LOGP(DFLEX, LOGL_ERROR, "Text too long!\n");
		data_length = sizeof(msg->data);
	}

	/* init */
	msg->capcode = capcode;
	msg->msg_type = msg_type;
	if (data && data_length > 0)
		memcpy(msg->data, data, data_length);
	msg->data_length = data_length;

	/* link to tail of list */
	msg->flex = flex;
	msgp = &flex->msg_list;
	while ((*msgp))
		msgp = &(*msgp)->next;
	(*msgp) = msg;

	/* kick transmitter */
	if (flex->state == FLEX_STATE_IDLE) {
		flex_new_state(flex, FLEX_STATE_PREAMBLE);
	} else
		flex_display_status();

	return msg;
}

/* Destroy msg instance */
void flex_msg_destroy(flex_msg_t *msg)
{
	flex_msg_t **msgp;

	/* unlink */
	msgp = &msg->flex->msg_list;
	while ((*msgp) != msg)
		msgp = &(*msgp)->next;
	(*msgp) = msg->next;

	/* destroy */
	free(msg);

	/* update display */
	flex_display_status();
}

/* Number of idle frames to send after all messages before returning to IDLE */
#define FLEX_IDLE_BATCHES	2

/*
 * Get next frame for transmission.
 *
 * Called by DSP layer when it needs a new frame to modulate.
 * Encodes the head message into frame_buffer, destroys it after encoding,
 * and manages state transitions.
 *
 * Returns 1 if a frame is ready in frame_buffer, 0 if no frame to send.
 */
int flex_get_next_frame(flex_t *flex)
{
	flex_msg_t *msg;
	int error = 0;
	size_t len;

	/* no frame if not transmitting */
	if (!flex->tx)
		return 0;

	switch (flex->state) {
	case FLEX_STATE_IDLE:
		return 0;

	case FLEX_STATE_PREAMBLE:
		/* transition to message state — preamble is part of the encoded frame */
		LOGP_CHAN(DFLEX, LOGL_INFO, "Preamble complete, entering message state.\n");
		flex_new_state(flex, FLEX_STATE_MESSAGE);
		flex->idle_count = 0;
		/* fall through to encode first message */
		/* fall through */

	case FLEX_STATE_MESSAGE:
		msg = flex->msg_list;
		if (msg) {
			/* reset idle counter when we have a message */
			flex->idle_count = 0;

			/* encode the head message into frame buffer */
			len = flex_encode_frame(msg->capcode, (int)msg->msg_type,
						msg->data, flex->frame_buffer,
						FLEX_BUFFER_SIZE, &error);

			/* destroy the transmitted message */
			flex_msg_destroy(msg);
			msg = NULL;

			if (error || len == 0) {
				LOGP_CHAN(DFLEX, LOGL_NOTICE, "Failed to encode FLEX frame (error=%d), skipping.\n", error);
				/* try next message if available */
				if (flex->msg_list)
					return flex_get_next_frame(flex);
				/* no more messages — check scan/loopback or go idle */
				goto check_idle;
			}

			flex->frame_buffer_length = (int)len;
			flex->frame_buffer_pos = 0;

			LOGP_CHAN(DFLEX, LOGL_INFO, "Encoded FLEX frame (%d bytes).\n", (int)len);
			return 1;
		}

check_idle:
		/* no messages in queue — check scan/loopback for more work */
		if (flex_scan_or_loopback(flex)) {
			/* scan/loopback enqueued a new message, encode it */
			return flex_get_next_frame(flex);
		}

		/* no more messages and no scan/loopback — count idle batches */
		if (flex->idle_count++ >= FLEX_IDLE_BATCHES) {
			LOGP_CHAN(DFLEX, LOGL_INFO, "Transmission done.\n");
			LOGP_CHAN(DFLEX, LOGL_DEBUG, "Reached %d idle batches, turning transmitter off.\n", FLEX_IDLE_BATCHES);
			flex_new_state(flex, FLEX_STATE_IDLE);
			return 0;
		}

		LOGP_CHAN(DFLEX, LOGL_DEBUG, "Idle batch %d of %d.\n", flex->idle_count, FLEX_IDLE_BATCHES);
		return 0;
	}

	return 0;
}

int flex_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int tx, double deviation, double polarity, enum flex_msg_type msg_type, const char *message, uint64_t scan_from, uint64_t scan_to, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback)
{
	flex_t *flex;
	int rc;

	flex = calloc(1, sizeof(*flex));
	if (!flex) {
		LOGP(DFLEX, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	LOGP(DFLEX, LOGL_DEBUG, "Creating 'FLEX' instance for 'Kanal' = %s (sample rate %d).\n", kanal, samplerate);

	/* init general part of transceiver */
	rc = sender_create(&flex->sender, kanal, frequency, frequency, device, use_sdr, samplerate, rx_gain, tx_gain, 0, 0, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DFLEX, LOGL_ERROR, "Failed to init transceiver process!\n");
		goto error;
	}

	/* init audio processing */
	rc = dsp_init_sender(flex, samplerate, deviation, polarity);
	if (rc < 0) {
		LOGP(DFLEX, LOGL_ERROR, "Failed to init audio processing!\n");
		goto error;
	}

	flex->tx = tx;
	flex->default_msg_type = msg_type;
	flex->default_message = message;
	/* NOTE: fsk_deviation and fsk_polarity are set by dsp_init_sender() above.
	 * Do NOT overwrite fsk_deviation with the raw deviation (4800 Hz) here —
	 * it must stay at 1.0 (normalized) because the sender framework multiplies
	 * samples by speech_deviation to get actual Hz. */
	flex->scan_from = scan_from;
	flex->scan_to = scan_to;

	flex_display_status();

	LOGP(DFLEX, LOGL_NOTICE, "Created 'Kanal' %s\n", kanal);

	/* start scanning, if enabled, otherwise send loopback sequence, if enabled */
	flex_scan_or_loopback(flex);

	return 0;

error:
	flex_destroy(&flex->sender);

	return rc;
}

void flex_destroy(sender_t *sender)
{
	flex_t *flex = (flex_t *) sender;

	LOGP(DFLEX, LOGL_DEBUG, "Destroying 'FLEX' instance for 'Kanal' = %s.\n", sender->kanal);

	while (flex->msg_list)
		flex_msg_destroy(flex->msg_list);
	dsp_cleanup_sender(flex);
	sender_destroy(&flex->sender);
	free(flex);
}

/*
 * Scan or loopback: generate test messages for scanning or loopback mode.
 *
 * Scan mode: sequentially transmit to each capcode in the scan range.
 * Loopback mode: continuously generate test messages for self-testing.
 *
 * Returns 1 if a message was enqueued, 0 otherwise.
 */
int flex_scan_or_loopback(flex_t *flex)
{
	if (flex->scan_from < flex->scan_to) {
		char message[16];

		/* Generate scan message based on configured message type */
		switch (flex->default_msg_type) {
		case FLEX_MSG_TYPE_NUMERIC:
			sprintf(message, "%05d", (int)(flex->scan_from / 100));
			break;
		case FLEX_MSG_TYPE_ALPHA:
			sprintf(message, "%02x", (int)(flex->scan_from / 10000));
			break;
		case FLEX_MSG_TYPE_TONE:
		case FLEX_MSG_TYPE_AUTO:
		default:
			message[0] = '\0';
		}
		LOGP_CHAN(DFLEX, LOGL_NOTICE, "Transmitting %s message '%s' with capcode '%" PRIu64 "'.\n",
			  flex_msg_type_name(flex->default_msg_type), message, flex->scan_from);
		flex_msg_create(flex, flex->scan_from, flex->default_msg_type,
				message, strlen(message));
		flex->scan_from++;
		return 1;
	}

	if (flex->sender.loopback) {
		LOGP(DFLEX, LOGL_INFO, "Sending message for loopback test.\n");
		flex_msg_create(flex, 1234567, FLEX_MSG_TYPE_NUMERIC, "1234", 4);
		return 1;
	}

	return 0;
}

void call_down_clock(void)
{
}

/*
 * Validate a dialed capcode string from the console.
 *
 * FLEX capcodes can be 1-10 digits (short: 1-1933312, long: 2101249-4297068542).
 * Returns NULL if valid, or an error string if invalid.
 */
const char *flex_number_valid(const char *number)
{
	uint64_t capcode;
	int i;

	/* check all digits */
	for (i = 0; number[i]; i++) {
		if (number[i] < '0' || number[i] > '9')
			return "Illegal capcode digit (use 0..9 only)";
	}

	capcode = strtoull(number, NULL, 10);
	if (!flex_capcode_valid(capcode))
		return "Invalid FLEX capcode (short: 1-1933312, long: 2101249-4297068542)";

	return NULL;
}

int call_down_setup(int callref, const char *caller_id, enum number_type __attribute__((unused)) caller_type, const char *dialing)
{
	sender_t *sender;
	flex_t *flex;
	const char *message;
	flex_msg_t *msg;

	/* find transmitter */
	for (sender = sender_head; sender; sender = sender->next) {
		flex = (flex_t *) sender;
		if (flex->tx)
			break;
	}
	if (!sender) {
		LOGP(DFLEX, LOGL_NOTICE, "Cannot page, no transmitting station available, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}

	/* get message */
	if (caller_id[0])
		message = caller_id;
	else
		message = flex->default_message;

	/* parse capcode and create message */
	{
		uint64_t capcode = strtoull(dialing, NULL, 10);

		LOGP(DFLEX, LOGL_INFO, "Paging capcode '%" PRIu64 "' with %s message '%s'.\n",
		     capcode, flex_msg_type_name(flex->default_msg_type), message);

		msg = flex_msg_create(flex, capcode, flex->default_msg_type,
				      message, strlen(message));
		if (!msg)
			return -CAUSE_INVALNUMBER;
	}

	call_up_release(callref, CAUSE_NORMAL);

	return 0;
}

void call_down_answer(int __attribute__((unused)) callref, struct timeval __attribute__((unused)) *tv_meter)
{
}

void call_down_proceeding(int __attribute__((unused)) callref)
{
}

void call_down_disconnect(int callref, int cause)
{
	call_up_release(callref, cause);
}

void call_down_release(int __attribute__((unused)) callref, int __attribute__((unused)) cause)
{
}

void call_down_audio(void __attribute__((unused)) *decoder, void __attribute__((unused)) *decoder_priv, int __attribute__((unused)) callref, uint16_t __attribute__((unused)) sequence, uint8_t __attribute__((unused)) marker, uint32_t __attribute__((unused)) timestamp, uint32_t __attribute__((unused)) ssrc, uint8_t __attribute__((unused)) *payload, int __attribute__((unused)) payload_len)
{
}

void dump_info(void) {}
