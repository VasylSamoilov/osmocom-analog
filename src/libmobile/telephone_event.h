#pragma once

/* RFC 4733 telephone-event: DTMF and flash in RTP.
 *
 * libosmo-cc negotiates 'telephone-event' like a codec: it is an entry of the
 * codec list, osmo_cc_helper_audio_accept_te() / _negotiate_te() return the
 * negotiated one, and its packets arrive at the audio receiver with that codec.
 */

#include <stdint.h>
#include <osmocom/cc/session.h>

#define TE_EVENT_FLASH	16

/* receiver state: an event is reported once, at its first end packet */
struct te_rx {
	int		valid;
	uint32_t	last_timestamp;
	uint8_t		last_event;
};

/* fmtp of our offer: events 0-15 (DTMF) and 16 (flash) */
void te_fmtp_encode(osmo_cc_fmtp_pv_t *fmtp, size_t fmtp_size, void *priv);
int te_fmtp_decode(const osmo_cc_fmtp_pv_t *fmtp, void *priv);

/* send one event: start packet (marker) and three end packets */
void te_send(osmo_cc_session_codec_t *te, uint8_t event, uint16_t duration_ms);

/* returns 1 and the event, if the packet ends an event not reported yet */
int te_receive(struct te_rx *rx, uint32_t timestamp, const uint8_t *payload, int payload_len,
	       uint8_t *event, uint16_t *duration);

/* DTMF character <-> event 0..15, -1 / 0 if none */
int te_event_from_digit(char digit);
char te_digit_from_event(uint8_t event);
