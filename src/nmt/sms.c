/* NMT SMS (short message service) processing
 *
 * (C) 2016 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "nmt.h"

/* SMS-call clearing timers, NMT Doc 450-3/900-3 Annex 24 para 1.1:
 *  - no DMS frame received within 10 s           -> clear
 *  - no response to a sent SMS packet within 20 s -> clear
 *  - after the last SMS packet, wait at least 5 s before clearing
 *    (lets the final acknowledgement arrive through data-link buffering)
 */
#define SMS_PACKET_TO		20,0	/* awaiting a response to a sent SMS packet */
#define SMS_DMS_TO		10,0	/* awaiting the next DMS frame on a live link */
#define SMS_RELEASE_TO		5,0	/* min wait before clearing after the last packet */

/* TP-Message-Type-Indicator (TP-MTI) */
#define	MTI_SMS_DELIVER		0x00 /* SC -> MS */
#define	MTI_SMS_DELIVER_REPORT	0x00 /* MS -> SC */
#define	MTI_SMS_STATUS_REPORT	0x02 /* SC -> MS */
#define	MTI_SMS_COMMAND		0x02 /* MS -> SC */
#define	MTI_SMS_SUBMIT		0x01 /* MS -> SC */
#define	MTI_SMS_SUBMIT_REPORT	0x01 /* SC -> MS */
#define MTI_MASK		0x03 /* Bits 0 and 1 */

/* TP-More-Messages-to-Send (TP-MMS) */
#define MMS_NORE		0x00
#define MMS_NO_MORE		0x04
#define MMS_MASK		0x04 /* Bit 2 */	

/* TP-Validity-Period-Format (TP-VPF) */
#define VPF_NOT_PRESENT		0x00
#define VPF_PRESENT_INTEGER	0x10
#define VPF_PRESENT_SEMI_OCTET	0x18
#define VPF_MASK		0x18 /* Bits 3 and 4 */

/* TP-Status-Report-Indication (TP-SRI) */
#define SRI_NO_REPORT		0x00
/* TP-DCS coding (Annex 24 para 2.2.3.10 / para 4): 0x00 = GSM default 7-bit;
 * 0x30..0x3f = NMT 8-bit alphabets (one octet per character), low nibble =
 * character set (0 Cyrillic, 1 Baltic, 2 Central, 3 Western). We originate the
 * GSM-compatible NMT Set 0 when an 8-bit message is needed. */
#define DCS_GSM_7BIT		0x00
#define DCS_NMT_8BIT_SET0	0x30
#define SRI_REPORT		0x20
#define SRI_MASK		0x20

/* TP-Status-Report-Request (TP-SRR) */
#define SSR_NO_REPORT		0x00
#define SSR_REPORT		0x20
#define SSR_MASK		0x20

/* TP-Failure-Cause (TP-FCS) */
#define FCS_BUSY		0xc0
#define FCS_NO_SC_SUBSCRIPTION	0xc1
#define FSC_SC_SYSTEM_FAILURE	0xC2
#define FSC_DEST_SME_BARRED	0xC4
#define FSC_ERROR_IN_MS		0xD2
#define FSC_MEMORY_EXCEEDED	0xD3
#define FSC_UNSPECIFIED_ERROR	0xFF

/* RP-Message-Type-Indicator (RP-MTI) */
#define RP_MO_DATA		0x00	/* MS -> SC */
#define RP_MT_DATA		0x01	/* SC -> MS */
#define RP_MT_ACK		0x02	/* MS -> SC */
#define RP_MO_ACK		0x03	/* SC -> MS */
#define RP_MT_ERROR		0x04	/* MS -> SC */
#define RP_MO_ERROR		0x05	/* SC -> MS */
#define RP_SM_MEMORY_AVAILABLE	0x06	/* MS -> SC */
#define RP_SM_READY_TO_RECEIVE	0x07	/* MS -> SC */
#define RP_SM_NO_MESSAGE	0x07	/* SC -> MS */
#define RP_MTI_MASK		0x07

/* RP IEs */
#define RP_IE_USER_DATA		0x41 /* wrong in NMT Doc.450-3 1998-04-03 */
#define RP_IE_CAUSE		0x42

/* SC -> MS header */
static const char sms_header[] = {
	0x01, 0x18, 0x53, 0x4d, 0x53, 0x48, 0x18, 'A', 'B', 'C', 0x02
};

/*
 * init and exit
 */

static void sms_timeout(void *data);

/* init instance */
int sms_init_sender(nmt_t *nmt)
{
	osmo_timer_setup(&nmt->sms_timer, sms_timeout, nmt);

	return 0;
}

/* Cleanup transceiver instance. */
void sms_cleanup_sender(nmt_t *nmt)
{
	sms_reset(nmt);
	osmo_timer_del(&nmt->sms_timer);
}

/*
 * send to lower layer
 */

/* encode header */
static int encode_header(uint8_t *data)
{
	memcpy(data, sms_header, sizeof(sms_header));

	return sizeof(sms_header);
}

/* encode address fields */
static int encode_address(uint8_t *data, const char *address, uint8_t type, uint8_t plan)
{
	int length = 1;
	uint8_t digit;
	int i, j;

	LOGP(DSMS, LOGL_DEBUG, "Encode SC->MS header\n");

	data[length++] = 0x80 | (type << 4) | plan;
	j = 0;
	for (i = 0; address[i]; i++) {
		if (address[i] >= '1' && address[i] <= '9')
			digit = address[i] - '0';
		else if (address[i] == '0')
			digit = 10;
		else if (address[i] == '*')
			digit = 11;
		else if (address[i] == '#')
			digit = 12;
		else if (address[i] == '+')
			digit = 13;
		else
			continue;
		if ((j & 1) == 0)
			data[length] = digit;
		else
			data[length++] |= digit << 4;
		j++;
	}
	if ((j & 1))
		data[length++] |= 0xf0;

	/* length field: number of semi-octets */
	data[0] = j;

	return length;
}

/* encode time stamp */
static int encode_time(uint8_t *data, time_t timestamp, int local)
{
	struct tm *tm = (local) ? localtime(&timestamp) : gmtime(&timestamp);
	int length = 0;
	uint8_t digit1, digit2;
	int quarters, sign;

	LOGP(DSMS, LOGL_DEBUG, "Encode time stamp '%02d.%02d.%02d %02d:%02d:%02d'\n", tm->tm_mday, tm->tm_mon + 1, tm->tm_year % 100, tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* year */
	digit1 = (tm->tm_year % 100) / 10;
	if (digit1 == 0)
		digit1 = 10;
	digit2 = tm->tm_year % 10;
	if (digit2 == 0)
		digit2 = 10;
	data[length++] = (digit2 << 4) | digit1;

	/* month */
	digit1 = (tm->tm_mon + 1) / 10;
	if (digit1 == 0)
		digit1 = 10;
	digit2 = (tm->tm_mon + 1) % 10;
	if (digit2 == 0)
		digit2 = 10;
	data[length++] = (digit2 << 4) | digit1;

	/* day */
	digit1 = tm->tm_mday / 10;
	if (digit1 == 0)
		digit1 = 10;
	digit2 = tm->tm_mday % 10;
	if (digit2 == 0)
		digit2 = 10;
	data[length++] = (digit2 << 4) | digit1;

	/* hour */
	digit1 = tm->tm_hour / 10;
	if (digit1 == 0)
		digit1 = 10;
	digit2 = tm->tm_hour % 10;
	if (digit2 == 0)
		digit2 = 10;
	data[length++] = (digit2 << 4) | digit1;

	/* min */
	digit1 = tm->tm_min / 10;
	if (digit1 == 0)
		digit1 = 10;
	digit2 = tm->tm_min % 10;
	if (digit2 == 0)
		digit2 = 10;
	data[length++] = (digit2 << 4) | digit1;

	/* sec */
	digit1 = tm->tm_sec / 10;
	if (digit1 == 0)
		digit1 = 10;
	digit2 = tm->tm_sec % 10;
	if (digit2 == 0)
		digit2 = 10;
	data[length++] = (digit2 << 4) | digit1;

	/* zone */
	quarters = (local) ? (timezone / 900) : 0;
	if (quarters < 0) {
		quarters = -quarters;
		sign = 1;
	} else {
		quarters = -quarters;
		sign = 0;
	}
	data[length++] = (quarters << 4) | (sign << 3) | (quarters >> 4);

	return length;
}

/* encode user data */
static int encode_userdata(uint8_t *data, const char *message)
{
	int length = 1;
	uint8_t character;
	int i, j, pos;

	LOGP(DSMS, LOGL_DEBUG, "Encode user data '%s'\n", message);

	j = 0;
	pos = 0;
	for (i = 0; message[i]; i++) {
		if ((int8_t)message[i] >= 0) {
			character = message[i]; /* 0..0x7f: representable in GSM 7-bit */
		} else {
			/* Annex 24 para 2.2.3.10 uses characters 0..0x7f; a byte above
			 * that range is not representable in the 7-bit alphabet. Log the
			 * substitution rather than dropping it silently. */
			LOGP(DSMS, LOGL_NOTICE, "SMS character 0x%02x at position %d is outside the 7-bit alphabet, substituting '?'\n",
				(uint8_t)message[i], i);
			character = '?';
		}
		j++;
		if (pos == 0) {
			/* character fits and is aligned to the right, new octet */
			data[length] = character;
			pos = 7;
		} else {
			/* character is shifted by pos */
			data[length] |= character << pos;
			if (pos > 1) {
				/* not all bits fit in octet, so fill the rest to next octet */
				length++;
				data[length] = character >> (8 - pos);
				pos--;
			} else {
				/* all bits fit in octet, so go to next octet */
				pos = 0;
				length++;
			}
		}
	}
	if (pos)
		length++;

	/* length field: number of characters */
	data[0] = j;

	return length;
}

/* Encode user data as NMT 8-bit characters: one character per octet
 * (Annex 24 para 2.2.3.13 / para 4.3, TP-DCS coding group 0011).  Unlike the
 * 7-bit path this stores each byte verbatim, so national letters (0x80..0xff)
 * survive.  'msg_len' is the octet/character count (the message is a C string,
 * so an embedded NUL cannot be represented -- see sms_deliver). */
static int encode_userdata_8(uint8_t *data, const char *message, int msg_len)
{
	data[0] = msg_len;			/* TP-UDL = number of 8-bit characters */
	memcpy(data + 1, message, msg_len);
	return 1 + msg_len;
}

/* deliver SMS (SC->MS) */
int sms_deliver(nmt_t *nmt, uint8_t ref, const char *orig_address, uint8_t orig_type, uint8_t orig_plan, time_t timestamp, int local, const char *message)
{
	uint8_t data[256], *tpdu_length;
	int length = 0;
	int orig_len;
	int msg_len;
	int i, need_8bit;
	uint8_t dcs;

	LOGP(DSMS, LOGL_INFO, "Delivering SMS from upper layer\n");

	orig_len = strlen(orig_address);
	msg_len = strlen(message);

	if (orig_len > 24) {
		LOGP(DSMS, LOGL_NOTICE, "Originator Address too long (%d characters)\n", orig_len);
		return -EINVAL;
	}

	/* Choose the coding by content (Annex 24 para 2.2.3.10): the GSM 7-bit
	 * alphabet carries only 0x00..0x7f, so if the message contains any byte
	 * >= 0x80 (a national letter) it must be sent with an NMT 8-bit alphabet
	 * (one octet per character).  A C string cannot hold an embedded NUL, so
	 * byte 0x00 inside a message is not representable through this interface. */
	need_8bit = 0;
	for (i = 0; i < msg_len; i++) {
		if ((uint8_t)message[i] >= 0x80) {
			need_8bit = 1;
			break;
		}
	}

	/* Length ceilings (para 2.2.3.13): 160 characters for 7-bit (-> 140
	 * octets) and 160 (optionally 255) for 8-bit (-> up to 255 octets). Both
	 * fit data[256] once the header/RP/TP overhead is added only for <=160. */
	if (!need_8bit && msg_len > 160) {
		LOGP(DSMS, LOGL_NOTICE, "Message too long (%d 7-bit characters, max 160)\n", msg_len);
		return -EINVAL;
	}
	if (need_8bit && msg_len > 160) {
		/* the extended 255 case would overflow data[256] with the ~30-octet
		 * header/RP/TP overhead, so cap at 160 here and say so */
		LOGP(DSMS, LOGL_NOTICE, "8-bit message too long (%d characters, max 160 supported)\n", msg_len);
		return -EINVAL;
	}
	dcs = need_8bit ? DCS_NMT_8BIT_SET0 : DCS_GSM_7BIT;

	/* HEADER */
	length = encode_header(data);

	/* RP */
	data[length++] = RP_MT_DATA;
	data[length++] = ref;
	data[length++] = RP_IE_USER_DATA;
	tpdu_length = data + length++;

	/* TP */
	data[length++] = MTI_SMS_DELIVER | MMS_NO_MORE | VPF_NOT_PRESENT | SRI_NO_REPORT;
	length += encode_address(data + length, orig_address, orig_type, orig_plan); /* TP-OA */
	data[length++] = 0; /* TP-PID */
	data[length++] = dcs; /* TP-DCS */
	length += encode_time(data + length, timestamp, local);
	if (need_8bit) {
		LOGP(DSMS, LOGL_INFO, "Message contains 8-bit characters, using NMT 8-bit alphabet (DCS 0x%02x)\n", dcs);
		length += encode_userdata_8(data + length, message, msg_len);
	} else {
		length += encode_userdata(data + length, message);
	}

	/* RP length */
	*tpdu_length = length - (uint8_t)(tpdu_length - data) - 1;
	LOGP(DSMS, LOGL_DEBUG, " -> TPDU length = %d\n", *tpdu_length);

	nmt->sms.mt = 1;
	dms_send(nmt, data, length, 1);

	/* start timer */
	osmo_timer_schedule(&nmt->sms_timer, SMS_PACKET_TO);

	return 0;
}

/* report SMS (SC->MS) */
static void sms_submit_report(nmt_t *nmt, uint8_t ref, int error)
{
	uint8_t data[64];
	int length = 0;

	LOGP(DSMS, LOGL_INFO, "Sending Submit Report (%s)\n", (error) ? "error" : "ok");

	/* HEADER */
	length = encode_header(data);

	/* RP */
	data[length++] = (error) ? RP_MO_ERROR : RP_MO_ACK;
	data[length++] = ref;

	dms_send(nmt, data, length, 1);
}

/*
 * receive from lower layer
 */

/* decode 7-bit character message from 8 bit data */
/* Decode a GSM 7-bit packed octet stream into characters.
 * 'octets' is the number of packed input octets; 'max_chars' bounds the output
 * so a trailing partial septet (septet packing can yield one more character
 * than the declared TP-UDL) can never write past message[max_chars]. The caller
 * sizes 'message' as max_chars + 1 for the NUL. */
static void decode_message_7(const uint8_t *data, int octets, int max_chars, char *message)
{
	int fill;
	int i;
	int chars = 0;
	uint16_t result;

	fill = 0;
	result = 0;
	for (i = 0; i < octets; i++) {
		result |= data[i] << fill;
		fill += 8;
		while (fill >= 7) {
			if (chars >= max_chars) {
				/* declared length reached: ignore any leftover bits */
				fill = 0;
				break;
			}
			*message++ = result & 0x7f;
			chars++;
			result >>= 7;
			fill -= 7;
		}
	}
	*message = '\0';
}

static const char digits2ascii[16] = {
	'?', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', '0', '*', '#', '+', '?', '?' };

static void decode_address(const uint8_t *data, int digits, char *address)
{
	int i;

	for (i = 0; i < digits; i++) {
		if (!(i & 1))
			*address++ = digits2ascii[(*data) & 0xf];
		else
			*address++ = digits2ascii[(*data++) >> 4];
	}
	*address++ = '\0';
}

/* decode sms submit message
 * return 1 if done, -1 if failed, 0, if more data is required */
static int decode_sms_submit(nmt_t *nmt, const uint8_t *data, int length)
{
	uint8_t ref, msg_ref;
	const uint8_t *orig_data, *tpdu_data, *dest_data, *msg_data;
	int orig_len, tpdu_len, dest_len, msg_len;
	int orig_digits, dest_digits, msg_chars;
	uint8_t orig_type, orig_plan, dest_type, dest_plan;
	int tp_vpf = VPF_NOT_PRESENT;
	int coding = 0;
	int rc;

	LOGP(DSMS, LOGL_DEBUG, "decode_sms_submit: got %d bytes\n", length);

	/* need the 2-octet RP header (MTI + message reference) before anything */
	if (length < 2) {
		LOGP(DSMS, LOGL_DEBUG, "SMS still incomplete, waiting for RP header\n");
		return 0;
	}

	/* decode ref */
	ref = data[1];
	data += 2;
	length -= 2;

	/* do we have originator address length ? */
	if (length < 2) {
		LOGP(DSMS, LOGL_DEBUG, "SMS still incomplete, waiting for originator address\n");
		return 0;
	}
	orig_data = 2 + data;
	orig_digits = data[0];
	orig_type = (data[1] >> 4) & 0x7;
	orig_plan = data[1] & 0x0f;
	orig_len = (orig_digits + 1) >> 1;
	if (length < 2 + orig_len) {
		LOGP(DSMS, LOGL_DEBUG, "SMS still incomplete, waiting for originator address digits (got %d of %d)\n", length - 1, orig_len);
		return 0;
	}
	data += 2 + orig_len;
	length -= 2 + orig_len;
	
	/* do we have user data IE ? */
	if (length < 2) {
		LOGP(DSMS, LOGL_DEBUG, "SMS still incomplete, waiting for user data IE\n");
		return 0;
	}
	if (data[0] != RP_IE_USER_DATA) {
		LOGP(DSMS, LOGL_NOTICE, "missing user data IE\n");
		return -FSC_ERROR_IN_MS;
	}
	tpdu_len = data[1];
	tpdu_data = 2 + data;
	if (length < 2 + tpdu_len) {
		LOGP(DSMS, LOGL_DEBUG, "SMS still incomplete, waiting for TPDU to be complete\n");
		return 0;
	}
	data += 2 + tpdu_len;
	length -= 2 + tpdu_len;

	/* decode orig address */
	char orig_address[orig_digits + 1];
	decode_address(orig_data, orig_digits, orig_address);
	LOGP(DSMS, LOGL_DEBUG, "Decoded originating address: '%s'\n", orig_address);

	/* go into TP */
	data = tpdu_data;
	length = tpdu_len;

	/* check msg_type */
	if (length < 1) {
		LOGP(DSMS, LOGL_NOTICE, "short read user data IE\n");
		return -FSC_ERROR_IN_MS;
	}
	if ((data[0] & MTI_MASK) != MTI_SMS_SUBMIT) {
		LOGP(DSMS, LOGL_NOTICE, "especting SUBMIT MTI, but got 0x%02x\n", data[0]);
		return -FSC_ERROR_IN_MS;
	}
	tp_vpf = data[0] & VPF_MASK;
	data++;
	length--;

	/* decode msg ref */
	if (length < 1) {
		LOGP(DSMS, LOGL_NOTICE, "short read user data IE\n");
		return -FSC_ERROR_IN_MS;
	}
	msg_ref = data[0];
	data++;
	length--;

	/* decode dest address */
	if (length < 2) {
		LOGP(DSMS, LOGL_NOTICE, "short read user data IE\n");
		return -FSC_ERROR_IN_MS;
	}
	dest_data = 2 + data;
	dest_digits = data[0];
	dest_type = (data[1] >> 4) & 0x7;
	dest_plan = data[1] & 0x0f;
	dest_len = (dest_digits + 1) >> 1;
	if (length < 2 + dest_len) {
		LOGP(DSMS, LOGL_NOTICE, "short read user data IE\n");
		return -FSC_ERROR_IN_MS;
	}
	data += 2 + dest_len;
	length -= 2 + dest_len;
	char dest_address[dest_digits + 1];
	decode_address(dest_data, dest_digits, dest_address);
	LOGP(DSMS, LOGL_DEBUG, "Decoded destination address: '%s'\n", dest_address);

	/* skip above protocol identifier */
	if (length < 1) {
		LOGP(DSMS, LOGL_NOTICE, "short read above protocol identifier IE\n");
		return -FSC_ERROR_IN_MS;
	}
	data++;
	length--;

	/* decode data coding scheme */
	if (length < 1) {
		LOGP(DSMS, LOGL_NOTICE, "short data coding scheme IE\n");
		return -FSC_ERROR_IN_MS;
	}
	if (data[0] == 0x00) {
		LOGP(DSMS, LOGL_DEBUG, "SMS coding is 7 bits (got 0x%02x)\n", data[0]);
		coding = 7;
	} else if ((data[0] & 0xf0) == 0x30) {
		LOGP(DSMS, LOGL_DEBUG, "SMS coding is 8 bits (got 0x%02x)\n", data[0]);
		coding = 8;
	} else {
		LOGP(DSMS, LOGL_NOTICE, "SMS coding unsupported (got 0x%02x)\n", data[0]);
		return -FSC_ERROR_IN_MS;
	}
	data++;
	length--;

	/* skip validity period (Annex 24 para 2.2.3.12): the integer/relative form
	 * is 1 octet, the absolute semi-octet form is 7 octets.  Skip exactly the
	 * right number so the user-data parse stays aligned. */
	if (tp_vpf != VPF_NOT_PRESENT) {
		int vp_len = (tp_vpf == VPF_PRESENT_SEMI_OCTET) ? 7 : 1;
		if (length < vp_len) {
			LOGP(DSMS, LOGL_NOTICE, "short read validity period IE (need %d octets)\n", vp_len);
			return -FSC_ERROR_IN_MS;
		}
		data += vp_len;
		length -= vp_len;
	}

	/* decode data message text */
	if (length < 1) {
		LOGP(DSMS, LOGL_NOTICE, "short read user data IE\n");
		return -FSC_ERROR_IN_MS;
	}
	msg_data = data + 1;
	msg_chars = data[0];
	/* Annex 24 para 2.2.3.13: at most 160 characters for both codings
	 * (optionally 255 for the 8-bit NMT alphabets). Reject an over-long
	 * declared length rather than trusting an attacker/garbled count. */
	if (coding == 7) {
		if (msg_chars > 160) {
			LOGP(DSMS, LOGL_NOTICE, "SMS user data too long: %d 7-bit characters (max 160), rejecting\n", msg_chars);
			return -FSC_ERROR_IN_MS;
		}
		msg_len = (msg_chars * 7 + 7) / 8;
	} else {
		if (msg_chars > 255) {
			LOGP(DSMS, LOGL_NOTICE, "SMS user data too long: %d 8-bit characters (max 255), rejecting\n", msg_chars);
			return -FSC_ERROR_IN_MS;
		}
		if (msg_chars > 160)
			LOGP(DSMS, LOGL_INFO, "SMS user data uses the optional extended 8-bit length (%d characters)\n", msg_chars);
		msg_len = msg_chars;
	}
	if (length < 1 + msg_len) {
		LOGP(DSMS, LOGL_NOTICE, "short read user data IE\n");
		return -FSC_ERROR_IN_MS;
	}
	char message[msg_chars + 1];
	if (coding == 7) {
		/* bound the decode by the declared character count (septet packing
		 * can otherwise emit one extra character and overflow message[]) */
		decode_message_7(msg_data, msg_len, msg_chars, message);
		LOGP(DSMS, LOGL_DEBUG, "Decoded message: '%s'\n", message);
	} else {
		memcpy(message, msg_data, msg_len);
		message[msg_len] = '\0';
		LOGP(DSMS, LOGL_DEBUG, "Included message: '%s'\n", message);
	}

	LOGP(DSMS, LOGL_INFO, "Submitting SMS to upper layer\n");

	rc = sms_submit(nmt, ref, orig_address, orig_type, orig_plan, msg_ref, dest_address, dest_type, dest_plan, message);
	if (rc < 0)
		return -FSC_SC_SYSTEM_FAILURE;

	return 1;
}

/* decode deliver report
 * return 1 if done, -1 if failed, 0, if more data is required */
static int decode_deliver_report(nmt_t *nmt, const uint8_t *data, int length)
{
	uint8_t ref, cause = 0;
	int error = 0;

	ref = data[1];

	if ((data[0] & RP_MTI_MASK) == RP_MT_ERROR) {
		error = 1;
		if (length < 4) {
			LOGP(DSMS, LOGL_DEBUG, "deliver report still incomplete, waiting for cause IE\n");
			return 0;
		}
		if (length < 4 + data[3]) {
			LOGP(DSMS, LOGL_DEBUG, "deliver report still incomplete, waiting for cause IE content\n");
			return 0;
		}
		if (data[2] == RP_IE_CAUSE && data[3] > 0)
			cause = data[4];
		LOGP(DSMS, LOGL_INFO, "Received Delivery report: ERROR, cause=%d\n", cause);
	} else
		LOGP(DSMS, LOGL_INFO, "Received Delivery report: OK\n");

	sms_deliver_report(nmt, ref, error, cause);

	return 1;
}

/* receive from DMS layer */
void dms_receive(nmt_t *nmt, const uint8_t *data, int length, int __attribute__((unused)) eight_bits)
{
	sms_t *sms = &nmt->sms;
	int space;
	int rc = 0;
	char debug_text[length * 5 + 1];
	int i;

	for (i = 0; i < length; i++)
		sprintf(debug_text + i * 5, " 0x%02x", data[i]);
	debug_text[length * 5] = '\0';

	/* restart timer */
	osmo_timer_schedule(&nmt->sms_timer, SMS_DMS_TO);

	LOGP(DSMS, LOGL_DEBUG, "Received %d bytes from DMS layer:%s\n", length, debug_text);

	if (sms->mt && !sms->data_sent) {
		LOGP(DSMS, LOGL_NOTICE, "Ignoring data while we transmit data\n");
		return;
	}

	/* append received data */
	space = sizeof(sms->rx_buffer) - sms->rx_count;
	if (space < length) {
		LOGP(DSMS, LOGL_NOTICE, "Received message exceeds RX buffer, terminating call!\n");
release:
		osmo_timer_schedule(&nmt->sms_timer, SMS_RELEASE_TO);
		return;
	}
	memcpy(sms->rx_buffer + sms->rx_count, data, length);
	sms->rx_count += length;

	/* go into buffer */
	data = sms->rx_buffer;
	length = sms->rx_count;

	/* check if complete */
	if (length < 2)
		return;

	LOGP(DSMS, LOGL_DEBUG, "Processing RP message: first bytes 0x%02x 0x%02x, MTI=%d, length=%d\n",
		data[0], data[1], data[0] & RP_MTI_MASK, length);

	switch (data[0] & RP_MTI_MASK) {
	case RP_MO_DATA:
		rc = decode_sms_submit(nmt, data, length);
		if (rc < 0)
			sms_submit_report(nmt, data[1], -rc);
		else if (rc > 0) {
			sms_submit_report(nmt, data[1], 0);
		}
		/* no release, we release afeter the report */
		rc = 0;
		break;
	case RP_MT_DATA:
		/* RP_MT_DATA (1) is SC->MS, should not be received from mobile.
		 * Some phones may echo back or send unexpected data.
		 * Log and ignore to avoid call release.
		 */
		LOGP(DSMS, LOGL_NOTICE, "Received unexpected RP_MT_DATA (0x%02x) from mobile, ignoring.\n", data[0]);
		rc = 0;
		break;
	case RP_MT_ACK:
		rc = decode_deliver_report(nmt, data, length);
		break;
	case RP_MO_ACK:
		/* RP_MO_ACK (3) is SC->MS, should not be received from mobile. */
		LOGP(DSMS, LOGL_NOTICE, "Received unexpected RP_MO_ACK (0x%02x) from mobile, ignoring.\n", data[0]);
		rc = 0;
		break;
	case RP_MT_ERROR:
		rc = decode_deliver_report(nmt, data, length);
		break;
	case RP_MO_ERROR:
		/* RP_MO_ERROR (5) is SC->MS, should not be received from mobile. */
		LOGP(DSMS, LOGL_NOTICE, "Received unexpected RP_MO_ERROR (0x%02x) from mobile, ignoring.\n", data[0]);
		rc = 0;
		break;
	case RP_SM_MEMORY_AVAILABLE:
		LOGP(DSMS, LOGL_NOTICE, "Received MEMORY-AVAILABLE message.\n");
		rc = 0;
		break;
	case RP_SM_READY_TO_RECEIVE:
		LOGP(DSMS, LOGL_NOTICE, "Received READY-TO-RECEIVE message.\n");
		rc = 0;
		break;
	default:
		LOGP(DSMS, LOGL_NOTICE, "Received unknown RP message type %d (0x%02x, MTI=%d).\n",
			data[0], data[0], data[0] & RP_MTI_MASK);
		rc = -1;
	}
	if (rc)
		goto release;
	
	return;
}

static void sms_timeout(void *data)
{
	nmt_t *nmt = data;

	sms_release(nmt);
}

/* all data has been sent to mobile */
void dms_all_sent(nmt_t *nmt)
{
	sms_t *sms = &nmt->sms;

	if (!sms->data_sent) {
		if (!sms->mt) {
			LOGP(DSMS, LOGL_DEBUG, "Done sending submit report, releasing.\n");
			osmo_timer_schedule(&nmt->sms_timer, SMS_RELEASE_TO);
		}
		sms->data_sent = 1;
		LOGP(DSMS, LOGL_DEBUG, "DMS layer indicates acknowledge of sent data\n");
	}
}

void sms_reset(nmt_t *nmt)
{
	sms_t *sms = &nmt->sms;

	LOGP(DSMS, LOGL_DEBUG, "Resetting SMS states\n");
	osmo_timer_del(&nmt->sms_timer);

	memset(sms, 0, sizeof(*sms));
}

