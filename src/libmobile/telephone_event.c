/* RFC 4733 telephone-event: DTMF and flash in RTP
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

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <osmocom/core/select.h>
#include <osmocom/cc/session.h>
#include <osmocom/cc/rtp.h>
#include "../liblogging/logging.h"
#include "telephone_event.h"

static const char dtmf_chars[] = "0123456789*#ABCD";

static void fmtp_set(osmo_cc_fmtp_pv_t *fmtp, const char *param, const char *value)
{
	free(fmtp->param);
	free(fmtp->value);
	fmtp->param = (param) ? strdup(param) : NULL;
	fmtp->value = (param) ? strdup(value) : NULL;
}

void te_fmtp_encode(osmo_cc_fmtp_pv_t *fmtp, size_t fmtp_size, void __attribute__((unused)) *priv)
{
	if (fmtp_size < 2)
		return;
	fmtp_set(&fmtp[0], "0-16", "");
	fmtp_set(&fmtp[1], NULL, NULL);
}

/* any set of events is fine: we send only what we have */
int te_fmtp_decode(const osmo_cc_fmtp_pv_t __attribute__((unused)) *fmtp, void __attribute__((unused)) *priv)
{
	return 0;
}

/*
 * The same packets as before with libosmo-cc's osmo_cc_rtp_send_event():
 * a start packet with the marker bit and three end packets (E bit), all with
 * the time stamp of the event start, on the audio socket and SSRC, at volume
 * -10 dBm0. Then the time stamp moves on by the duration.
 */
void te_send(osmo_cc_session_codec_t *te, uint8_t event, uint16_t duration_ms)
{
	uint16_t duration = duration_ms * 8;	/* 8000 Hz clock */
	uint8_t payload[4];
	int i;

	if (!te || !te->media)
		return;

	payload[0] = event;
	payload[1] = 10;		/* E = 0, R = 0, volume = 10 */
	payload[2] = duration >> 8;
	payload[3] = duration;
	osmo_cc_rtp_send(te, payload, sizeof(payload), 1, 1, 0);

	payload[1] |= 0x80;		/* E */
	for (i = 0; i < 3; i++)
		osmo_cc_rtp_send(te, payload, sizeof(payload), 0, 1, 0);

	te->media->tx_timestamp += duration;

	LOGP(DCALL, LOGL_INFO, "Sent RFC 4733 event %d ('%c'), %d ms, payload type %d\n",
	     event, (event == TE_EVENT_FLASH) ? '!' : te_digit_from_event(event), duration_ms, te->payload_type_remote);
}

int te_receive(struct te_rx *rx, uint32_t timestamp, const uint8_t *payload, int payload_len,
	       uint8_t *event, uint16_t *duration)
{
	uint8_t e;

	if (payload_len < 4) {
		LOGP(DCALL, LOGL_NOTICE, "RFC 4733 packet too short (%d bytes)\n", payload_len);
		return 0;
	}
	e = payload[1] >> 7;
	LOGP(DCALL, LOGL_DEBUG, "RFC 4733 RX: event %d, end %d, time stamp %u, duration %d\n",
	     payload[0], e, timestamp, (payload[2] << 8) | payload[3]);

	/* report at the end; the end packet is sent three times */
	if (!e)
		return 0;
	if (rx->valid && rx->last_timestamp == timestamp && rx->last_event == payload[0])
		return 0;
	rx->valid = 1;
	rx->last_timestamp = timestamp;
	rx->last_event = payload[0];

	*event = payload[0];
	*duration = (payload[2] << 8) | payload[3];
	return 1;
}

int te_event_from_digit(char digit)
{
	const char *p;

	if (digit >= 'a' && digit <= 'd')
		digit -= 'a' - 'A';
	if (!digit)
		return -1;
	p = strchr(dtmf_chars, digit);
	return (p) ? (int)(p - dtmf_chars) : -1;
}

char te_digit_from_event(uint8_t event)
{
	return (event < 16) ? dtmf_chars[event] : 0;
}
