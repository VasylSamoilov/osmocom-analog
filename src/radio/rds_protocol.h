/* RDS Protocol Server - XDR-GTK, RDS Spy, and UECP protocols
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
 *
 * ============================================================
 * ARCHITECTURE
 * ============================================================
 * - Ring buffers decouple radio processing from network I/O
 * - Radio thread writes to TX ring buffer (never blocks, drops if full)
 * - Poll function drains TX ring to socket (non-blocking)
 * - RX commands parsed from RX ring buffer in small batches
 * - All socket/serial I/O is O_NONBLOCK
 *
 * ============================================================
 * PROTOCOL 1: XDR-GTK (ASCII, Decoder Output + Tuner Control)
 * ============================================================
 * Source: XDR-GTK tuner.c (https://github.com/kkonradpl/xdr-gtk)
 * Direction: Bidirectional (decoder->client, client->tuner)
 * Transport: TCP or Serial
 * Line terminator: \n (LF)
 *
 * --- OUTPUT (Radio -> Client) ---
 *
 * OK                      Startup acknowledgment
 *                         Mapping: Sent on 'x' command
 *
 * X                       Shutdown notification
 *                         Mapping: Not implemented (we don't shutdown)
 *
 * T<freq_khz>             Current frequency in kHz
 *                         Example: T98500 (98.5 MHz)
 *                         Mapping: rds_server_send_freq()
 *
 * S<mode><signal>[,cci,aci]  Signal strength report
 *                         mode: s=stereo, m=mono, S/M=forced
 *                         signal: dBm or dBuV (float)
 *                         Example: Ss-45.2 (stereo, -45.2 dBm)
 *                         Mapping: rds_server_send_signal()
 *
 * P<XXXX>[?...]           PI code with error indication
 *                         XXXX: 4 hex digits
 *                         ?: one per error level (1-3)
 *                         Example: P2201?? (PI=2201, 2 errors)
 *                         Mapping: rds_server_send_pi()
 *
 * R<AAAA><BBBB><CCCC><DDDD><EE>  RDS group (18 hex chars)
 *                         AAAA-DDDD: Block A-D (4 hex each)
 *                         EE: Error byte (2 bits per block)
 *                              bits 7-6: Block A errors
 *                              bits 5-4: Block B errors
 *                              bits 3-2: Block C errors
 *                              bits 1-0: Block D errors
 *                              00=none, 01=small, 10=large, 11=uncorr
 *                         Example: R220100B4E2010000 (Group 0A)
 *                         Mapping: rds_server_send_group()
 *
 * N<level>                Stereo pilot level (0-100)
 *                         Mapping: Not implemented
 *
 * <cmd><value>            Setting confirmations (echo)
 *                         Mapping: Echoed after setting commands
 *
 * --- INPUT (Client -> Radio) ---
 *
 * x                       Status/init request
 *                         Response: OK
 *                         Mapping: tx_write_str(srv, "OK\n")
 *
 * T<freq_khz>             Tune to frequency
 *                         Example: T98500 (tune to 98.5 MHz)
 *                         Mapping: srv->tune_cb(freq, srv->cb_arg)
 *
 * A<0-3>                  Set AGC mode
 *                         Mapping: srv->setting_cb("A", val, ...)
 *
 * B<0-2>                  Stereo/Mono mode
 *                         0=stereo, 1=mono, 2=auto
 *                         Mapping: srv->setting_cb("B", val, ...)
 *
 * D<0-1>                  De-emphasis
 *                         0=50µs (EU), 1=75µs (US)
 *                         Mapping: srv->setting_cb("D", val, ...)
 *
 * F<0-15>                 Bandwidth filter
 *                         Mapping: srv->setting_cb("F", val, ...)
 *
 * G<0-3>                  RF/IF gain
 *                         Mapping: srv->setting_cb("G", val, ...)
 *
 * Y<0-100>                Volume
 *                         Mapping: srv->setting_cb("Y", val, ...)
 *
 * Z<0-3>                  Antenna select
 *                         Mapping: srv->setting_cb("Z", val, ...)
 *
 * Q<level>                Squelch level
 *                         Mapping: srv->setting_cb("Q", val, ...)
 *
 * I<ms>                   Signal sampling interval (10-10000 ms)
 *                         Default: 100ms
 *                         Mapping: srv->signal_interval_ms
 *
 * ============================================================
 * PROTOCOL 2: RDS Spy (ASCII, Decoder Output Only)
 * ============================================================
 * Source: XDR-GTK rdsspy.c, RDS Spy software format
 * Direction: Output only (decoder->client)
 * Transport: TCP or Serial
 * Line terminator: \r\n (CRLF)
 *
 * --- OUTPUT (Radio -> Client) ---
 *
 * G:\r\n<AAAA><BBBB><CCCC><DDDD>\r\n\r\n
 *                         RDS group output
 *                         Each block: 4 hex chars or "----" if bad
 *                         Example: G:\r\n2201E2010000\r\n\r\n
 *                         Mapping: rds_server_send_group()
 *
 * G:\r\nRESET\r\n\r\n     Reset notification (frequency change)
 *                         Mapping: rds_server_send_reset()
 *
 * --- INPUT ---
 * None (output-only protocol, input discarded)
 *
 * ============================================================
 * PROTOCOL 3: UECP (Binary, Encoder Control, IEC 62106-10)
 * ============================================================
 * Source: IEC 62106-10:2021, R22_039_1_pw72.pdf
 * Direction: Input only (client->encoder)
 * Transport: TCP or Serial
 * Frame format: Binary with byte stuffing
 *
 * --- FRAME FORMAT ---
 *
 * +-----+-------+-----+-----+---------+-------+-----+
 * | STA |  ADD  | SQC | MFL |   MSG   |  CRC  | STP |
 * |0xFE |2 bytes|  1  |  1  | 0-255   |   2   |0xFF |
 * +-----+-------+-----+-----+---------+-------+-----+
 *
 * STA (0xFE): Start delimiter
 * ADD: Site address (10 bits) + Encoder address (6 bits)
 *      Byte 0: [enc_addr(5:0)][site_addr(9:8)]
 *      Byte 1: site_addr(7:0)
 *      0x0000 = global (all encoders)
 * SQC: Sequence counter (1-255, 0=disabled)
 * MFL: Message field length (0-255)
 * MSG: Message elements (MEC + data)
 * CRC: CRC-16 ITU-T (x^16+x^12+x^5+1) over ADD..MSG
 * STP (0xFF): Stop delimiter
 *
 * --- BYTE STUFFING (within ADD..CRC) ---
 * 0xFD -> 0xFD 0x00
 * 0xFE -> 0xFD 0x01
 * 0xFF -> 0xFD 0x02
 *
 * --- MESSAGE ELEMENT CODES (MEC) ---
 *
 * MEC 0x01: PI (Programme Identification)
 *           Data: DSN(1) + PSN(1) + PI(2)
 *           Mapping: rds_enc_set_pi(enc, pi)
 *
 * MEC 0x02: PS (Programme Service Name)
 *           Data: DSN(1) + PSN(1) + PS(8)
 *           Mapping: rds_enc_set_ps(enc, ps)
 *
 * MEC 0x03: TP/TA (Traffic Programme/Announcement)
 *           Data: DSN(1) + PSN(1) + flags(1)
 *                 bit 0: TA, bit 1: TP
 *           Mapping: rds_enc_set_tp(enc, tp)
 *                    rds_enc_set_ta(enc, ta)
 *
 * MEC 0x04: DI/PTYI (Decoder Info + Dynamic PTY)
 *           Data: DSN(1) + PSN(1) + flags(1)
 *           Mapping: rds_enc_set_di(enc, ...)
 *           Status: Not implemented
 *
 * MEC 0x07: PTY (Programme Type)
 *           Data: DSN(1) + PSN(1) + PTY(1)
 *                 PTY: 0-31
 *           Mapping: rds_enc_set_pty(enc, pty)
 *
 * MEC 0x0A: RT (RadioText)
 *           Data: DSN(1) + PSN(1) + MEL(1) + config(1) + text(1-64)
 *                 MEL: length of config+text
 *                 config: bit 0 = A/B flag
 *           Mapping: rds_enc_set_radiotext(enc, rt)
 *
 * MEC 0x0D: RTC (Real Time Clock)
 *           Data: MJD + time
 *           Mapping: System time used (not implemented)
 *
 * MEC 0x13: AF (Alternative Frequencies)
 *           Data: DSN(1) + PSN(1) + AF list
 *           Mapping: rds_enc_af_set_method_a(enc, ...)
 *           Status: Not implemented
 *
 * MEC 0x17: Request
 *           Data: Request type
 *           Mapping: Not implemented (query mode)
 *
 * MEC 0x18: ACK (Acknowledgement)
 *           Data: code(1) + [seq(1)]
 *                 0x00=OK, 0x01=CRC error, 0x03=unknown,
 *                 0x06=param error
 *           Mapping: uecp_send_ack()
 *
 * MEC 0x19: CT On/Off (Clock Time enable)
 *           Data: DSN(1) + on/off(1)
 *           Mapping: Not implemented
 *
 * MEC 0x1A: ECC/SLC (Extended Country Code)
 *           Data: DSN(1) + variant(1) + data(1-2)
 *                 variant 0: ECC (8 bits)
 *                 variant 3: Language code
 *           Mapping: rds_enc_set_ecc(enc, ecc)
 *                    rds_enc_set_language(enc, lang)
 *
 * MEC 0x3E: PTYN (Programme Type Name)
 *           Data: DSN(1) + PSN(1) + PTYN(8)
 *           Mapping: rds_enc_set_ptyn(enc, ptyn)
 *
 * --- OUTPUT (Encoder -> Client) ---
 *
 * ACK frames sent in response to commands with SQC != 0
 * Format: Same frame structure with MEC 0x18
 *
 * ============================================================
 * IMPLEMENTATION STATUS
 * ============================================================
 *
 * XDR-GTK Output:
 *   [x] R - RDS groups          rds_server_send_group()
 *   [x] P - PI code             rds_server_send_pi()
 *   [x] S - Signal strength     rds_server_send_signal()
 *   [x] T - Frequency           rds_server_send_freq()
 *   [x] OK - Init ack           tx_write_str()
 *   [ ] N - Pilot level         Not implemented
 *   [ ] X - Shutdown            Not implemented
 *
 * XDR-GTK Input:
 *   [x] x - Init request        Returns "OK"
 *   [x] T - Tune                Via tune_cb callback
 *   [x] A,B,D,F,G,Y,Z,Q         Via setting_cb callback
 *   [x] I - Signal interval     srv->signal_interval_ms
 *   [x] S - Spectral scan       Params ignored, trigger returns empty U
 *
 * RDS Spy Output:
 *   [x] G: groups               rds_server_send_group()
 *   [x] RESET                   rds_server_send_reset()
 *
 * UECP Input:
 *   [x] 0x01 PI                 rds_enc_set_pi()
 *   [x] 0x02 PS                 rds_enc_set_ps()
 *   [x] 0x03 TP/TA              rds_enc_set_tp/ta()
 *   [x] 0x04 DI/PTYI            rds_enc_set_di()
 *   [x] 0x05 M/S                rds_enc_set_ms()
 *   [x] 0x07 PTY                rds_enc_set_pty()
 *   [x] 0x09 RTC Correction     ct_time_offset
 *   [x] 0x0A RT                 rds_enc_set_radiotext()
 *   [x] 0x0B PSN Enable         Logged (single-service encoder)
 *   [x] 0x0D RTC                local_offset updated
 *   [x] 0x13 AF                 Logged (TODO: AF code conversion)
 *   [x] 0x14 EON-AF             Logged (TODO: EON-AF parsing)
 *   [x] 0x17 Request            uecp_handle_request()
 *   [x] 0x19 CT On/Off          ct_enabled
 *   [x] 0x1A ECC                rds_enc_set_ecc()
 *   [x] 0x28 Make PSN List      Logged (single-service encoder)
 *   [x] 0x3E PTYN               rds_enc_set_ptyn()
 *   [x] 0x3F EON Enable         Logged
 *
 * UECP Output:
 *   [x] 0x18 ACK                uecp_send_ack()
 *   [x] Response frames         uecp_send_mec_response()
 */

#ifndef _RDS_PROTOCOL_H
#define _RDS_PROTOCOL_H

#include <stdint.h>

/* Forward declarations */
struct rds_encoder;

/* ============================================================
 * Ring Buffer (lock-free single producer/consumer)
 * ============================================================ */

/* Ring buffer size - must be power of 2.
 * 32768 needed to hold full spectral scan U response (~3KB for 206 points)
 * plus normal signal/RDS traffic without dropping. */
#define RDS_RING_SIZE		32768
#define RDS_RING_MASK		(RDS_RING_SIZE - 1)

typedef struct {
	uint8_t		data[RDS_RING_SIZE];
	volatile int	head;	/* Write position (producer) */
	volatile int	tail;	/* Read position (consumer) */
} rds_ring_t;

/* Ring buffer operations (inline for performance) */
static inline void rds_ring_init(rds_ring_t *r) {
	r->head = r->tail = 0;
}

static inline int rds_ring_used(const rds_ring_t *r) {
	return (r->head - r->tail) & RDS_RING_MASK;
}

static inline int rds_ring_free(const rds_ring_t *r) {
	return RDS_RING_SIZE - 1 - rds_ring_used(r);
}

static inline int rds_ring_empty(const rds_ring_t *r) {
	return r->head == r->tail;
}

/* Write bytes to ring. Returns bytes written (may be less if full). */
static inline int rds_ring_write(rds_ring_t *r, const uint8_t *buf, int len) {
	int free_space = rds_ring_free(r);
	int i;
	if (len > free_space)
		len = free_space;
	for (i = 0; i < len; i++) {
		r->data[r->head] = buf[i];
		r->head = (r->head + 1) & RDS_RING_MASK;
	}
	return len;
}

/* Read bytes from ring. Returns bytes read. */
static inline int rds_ring_read(rds_ring_t *r, uint8_t *buf, int len) {
	int used = rds_ring_used(r);
	int i;
	if (len > used)
		len = used;
	for (i = 0; i < len; i++) {
		buf[i] = r->data[r->tail];
		r->tail = (r->tail + 1) & RDS_RING_MASK;
	}
	return len;
}

/* Peek at data without consuming */
static inline int rds_ring_peek(const rds_ring_t *r, uint8_t *buf, int len) {
	int used = rds_ring_used(r);
	int tail = r->tail;
	int i;
	if (len > used)
		len = used;
	for (i = 0; i < len; i++) {
		buf[i] = r->data[tail];
		tail = (tail + 1) & RDS_RING_MASK;
	}
	return len;
}

/* Discard bytes from ring */
static inline void rds_ring_discard(rds_ring_t *r, int len) {
	int used = rds_ring_used(r);
	if (len > used)
		len = used;
	r->tail = (r->tail + len) & RDS_RING_MASK;
}

/* ============================================================
 * Endpoint Configuration
 * ============================================================ */

typedef enum {
	RDS_EP_NONE = 0,
	RDS_EP_TCP,
	RDS_EP_SERIAL
} rds_ep_type_t;

typedef enum {
	RDS_FLOW_NONE = 0,
	RDS_FLOW_XONXOFF,
	RDS_FLOW_RTSCTS
} rds_flow_t;

typedef enum {
	RDS_PROTO_XDR_GTK = 0,	/* Decoder output + control */
	RDS_PROTO_RDSSPY,	/* Decoder output only */
	RDS_PROTO_UECP,		/* Encoder control (binary) */
	RDS_PROTO_ASCII_G	/* PIRA ASCII G encoder control (text) */
} rds_proto_t;

#define RDS_EP_IP_LEN		64
#define RDS_EP_DEV_LEN		256

typedef struct {
	rds_ep_type_t	type;
	char		ip[RDS_EP_IP_LEN];
	int		port;
	char		device[RDS_EP_DEV_LEN];
	int		speed;
	int		bits;
	char		parity;
	int		stopbits;
	rds_flow_t	flow;
} rds_endpoint_t;

/* ============================================================
 * Constants
 * ============================================================ */

/* RDS block error levels */
#define RDS_ERR_NONE		0
#define RDS_ERR_SMALL		1
#define RDS_ERR_LARGE		2
#define RDS_ERR_UNCORR		3

/* Invalid fd */
#define RDS_FD_NONE		(-1)

/* XDR-GTK TCP authentication (SHA1 challenge-response) */
#define XDR_AUTH_SALT_LEN	16	/* Random salt bytes sent to client */
#define XDR_AUTH_HASH_LEN	40	/* SHA1 hex digest length */
#define XDR_AUTH_TIMEOUT_S	5	/* Seconds to wait for client hash */

typedef enum {
	XDR_AUTH_NONE = 0,	/* Serial or no-password: skip handshake */
	XDR_AUTH_WAIT,		/* Sent salt, waiting for client hash */
	XDR_AUTH_OK		/* Authenticated, normal operation */
} xdr_auth_state_t;

/* UECP frame constants (IEC 62106-10) */
#define UECP_STA		0xFE
#define UECP_STP		0xFF
#define UECP_STUFF		0xFD
#define UECP_FRAME_MAX		263

/* UECP Message Element Codes (IEC 62106-10:2021 Table A.1) */
#define UECP_MEC_PI		0x01	/* Programme Identification */
#define UECP_MEC_PS		0x02	/* Programme Service name */
#define UECP_MEC_TP_TA		0x03	/* TP/TA flags */
#define UECP_MEC_DI_PTYI	0x04	/* DI + Dynamic PTY Indicator */
#define UECP_MEC_MS		0x05	/* Music/Speech */
#define UECP_MEC_PTY		0x07	/* Programme Type */
#define UECP_MEC_RTC_CORR	0x09	/* Real time clock correction */
#define UECP_MEC_RT		0x0A	/* RadioText */
#define UECP_MEC_PSN_ENABLE	0x0B	/* PSN enable/disable */
#define UECP_MEC_RTC		0x0D	/* Real time clock for CT */
#define UECP_MEC_RDS_LEVEL	0x0E	/* RDS level */
#define UECP_MEC_AF		0x13	/* Alternative Frequencies */
#define UECP_MEC_EON_AF		0x14	/* EON Alternative Frequencies */
#define UECP_MEC_EON_TA_CTRL	0x15	/* EON-TA control */
#define UECP_MEC_GROUP_SEQ	0x16	/* Group sequence (data-stream 0) */
#define UECP_MEC_REQUEST	0x17	/* Request message */
#define UECP_MEC_ACK		0x18	/* Message acknowledgement */
#define UECP_MEC_CT_ONOFF	0x19	/* CT on/off */
#define UECP_MEC_ECC		0x1A	/* ECC and slow label settings */
#define UECP_MEC_DATA_SET_SEL	0x1C	/* Data set select */
#define UECP_MEC_REF_INPUT	0x1D	/* Reference input selection */
#define UECP_MEC_RDS_ONOFF	0x1E	/* RDS on/off (subcarrier 0) */
#define UECP_MEC_LPS		0x21	/* Long PS name */
#define UECP_MEC_RDS_PHASE	0x22	/* RDS phase (subcarrier 0) */
#define UECP_MEC_SITE_ADDR	0x23	/* Site address */
#define UECP_MEC_FREE_FORMAT	0x24	/* Free format data group (data-stream 0) */
#define UECP_MEC_TDC		0x26	/* Transparent Data Channel */
#define UECP_MEC_ENC_ADDR	0x27	/* Encoder address */
#define UECP_MEC_MAKE_PSN_LIST	0x28	/* Make PSN list */
#define UECP_MEC_GROUP_VARIANT	0x29	/* Group variant code sequence (data-stream 0) */
#define UECP_MEC_TA_CTRL	0x2A	/* TA control */
#define UECP_MEC_EWS		0x2B	/* Emergency Warning System */
#define UECP_MEC_COMM_MODE	0x2C	/* Communication mode */
#define UECP_MEC_MANUFACTURER	0x2D	/* Manufacturer/operator specific */
#define UECP_MEC_LINKAGE	0x2E	/* Linkage information */
#define UECP_MEC_TMC		0x30	/* Traffic Message Channel */
#define UECP_MEC_EXT_GROUP_SEQ	0x38	/* Extended group sequence (data-stream 0) */
#define UECP_MEC_ACCESS_RIGHT	0x3A	/* Encoder access right */
#define UECP_MEC_PORT_MODE	0x3B	/* Port configuration - Mode */
#define UECP_MEC_PORT_SPEED	0x3C	/* Port configuration - Speed */
#define UECP_MEC_PORT_TIMEOUT	0x3D	/* Port configuration - Timeout */
#define UECP_MEC_PTYN		0x3E	/* Programme Type Name */
#define UECP_MEC_EON_ENABLE	0x3F	/* EON enable/disable */
#define UECP_MEC_ODA_CONFIG	0x40	/* ODA configuration + Short message (A/B) */
#define UECP_MEC_ODA_IDENT	0x41	/* ODA identification group usage sequence (A/B) */
#define UECP_MEC_ODA_FREE	0x42	/* ODA free-format (old, replaced by 0x46) */
#define UECP_MEC_ODA_PRIORITY	0x43	/* ODA relative priority (A/B) */
#define UECP_MEC_ODA_BURST	0x44	/* ODA burst mode */
#define UECP_MEC_ODA_SPIN	0x45	/* ODA spinning wheel timing */
#define UECP_MEC_ODA_DATA	0x46	/* ODA data */
#define UECP_MEC_ODA_ACCESS	0x47	/* ODA data command access right */
#define UECP_MEC_DAB_DL_CMD	0x48	/* DAB Dynamic Label command */
#define UECP_MEC_ODA_AID_C1	0x50	/* ODA-AID channel assignment type C (Alt 1) */
#define UECP_MEC_ODA_AID_C2	0x51	/* ODA-AID channel assignment type C (Alt 2) */
#define UECP_MEC_PRIORITY_C	0x53	/* Relative priority (type C) */
#define UECP_MEC_BURST_C	0x54	/* Burst mode (type C) */
#define UECP_MEC_RFT_ALT2	0x55	/* RFT file data (Alt 2) */
#define UECP_MEC_ODA_DATA_C	0x56	/* ODA data (type C) */
#define UECP_MEC_RFT_ALT1	0x57	/* RFT file data (Alt 1) */
#define UECP_MEC_RFT_VARIANTS	0x58	/* RFT file variants 2-7 (Alt 1) */
#define UECP_MEC_FILE_SEQ	0x59	/* File sequence for AID in RFT */
#define UECP_MEC_GROUP_SEQ_C	0x61	/* Group sequence (type C) */
#define UECP_MEC_EXT_GS_C	0x83	/* Extended group sequence (type C) */
#define UECP_MEC_DAB_DL_MSG	0xAA	/* DAB Dynamic Label message */
#define UECP_MEC_UPPER_LEVEL	0xE0	/* Upper stream level */
#define UECP_MEC_UPPER_ONOFF	0xE1	/* Upper streams on/off */
/* EON Enable flag bits — aliases for UECP (same as RDS_EON_FLAG_* in rds.h) */
#define UECP_EON_FLAG_PS	RDS_EON_FLAG_PS
#define UECP_EON_FLAG_AF	RDS_EON_FLAG_AF
#define UECP_EON_FLAG_LINK	RDS_EON_FLAG_LINK
#define UECP_EON_FLAG_PTY	RDS_EON_FLAG_PTY
#define UECP_EON_FLAG_PIN	RDS_EON_FLAG_PIN
#define UECP_EON_FLAG_BCAST	RDS_EON_FLAG_BCAST
#define UECP_EON_FLAG_TA	RDS_EON_FLAG_TA

/* UECP manufacturer-specific identifiers */
#define UECP_MFR_ID_HI		'O'	/* Manufacturer designation: "OA" = osmocom-analog */
#define UECP_MFR_ID_LO		'A'
#define UECP_MFR_MODEL		"osmocom-analog"

/* UECP ACK codes */
#define UECP_ACK_OK		0x00
#define UECP_ACK_CRC_ERR	0x01
#define UECP_ACK_UNKNOWN	0x03
#define UECP_ACK_PARAM_ERR	0x06

/* UECP addressing constants (IEC 62106-10 §6) */
#define UECP_ADDR_GLOBAL	0x00	/* Global: all sites/encoders */
#define UECP_SITE_ADDR_MASK	0x03FF	/* 10-bit site address */
#define UECP_ENC_ADDR_MASK	0x3F	/* 6-bit encoder address */

/* UECP address control bits (MEC 0x23, 0x27) */
#define UECP_ADDR_CTRL_REMOVE	0x00	/* Remove specified address */
#define UECP_ADDR_CTRL_ADD	0x01	/* Add specified address */
#define UECP_ADDR_CTRL_CLEAR	0x02	/* Remove all addresses */

/* UECP access right constants (MEC 0x3A) */
#define UECP_ACCESS_ALL_MECS	0xFF	/* Apply to all MECs */
#define UECP_ACCESS_ALL_PORTS	0xFF	/* Apply to all ports */
#define UECP_ACCESS_ENABLED	0x01	/* Access enabled */

/* UECP on/off flag values */
#define UECP_FLAG_OFF		0x00	/* Feature disabled */
#define UECP_FLAG_ON		0x01	/* Feature enabled */

/* UECP DSN/PSN constants */
#define UECP_DSN_CURRENT	0x00	/* Current data set */
#define UECP_PSN_MAIN		0x00	/* Main programme service */

/* UECP RDS signal defaults */
#define UECP_RDS_LEVEL_DEFAULT	2000	/* Default RDS level in mV p-p */
#define UECP_RDS_PHASE_DEFAULT	0	/* Default phase: 0.0 degrees */

/* MEC name lookup (for logging) */
const char *uecp_mec_name(uint8_t mec);

/* ============================================================
 * Server State
 * ============================================================ */

typedef struct rds_server {
	/* Config */
	rds_endpoint_t	ep;
	rds_proto_t	proto;
	
	/* File descriptors (non-blocking) */
	int		listen_fd;
	int		client_fd;
	
	/* Ring buffers for decoupled I/O */
	rds_ring_t	tx_ring;	/* Outgoing data to client */
	rds_ring_t	rx_ring;	/* Incoming data from client */
	
	/* UECP frame parser state */
	int		uecp_in_frame;
	int		uecp_escaped;
	uint8_t		uecp_frame[UECP_FRAME_MAX];
	int		uecp_len;
	
	/* XDR-GTK state */
	int		freq_khz;
	int		deemphasis;		/* 0=50µs (EU), 1=75µs (US), -1=unknown */
	double		signal_dbm;
	int		stereo;
	int		forced_mono;		/* B1=forced mono, B0=auto stereo */
	double		pilot_mag;		/* pilot magnitude (0.0-1.0, normalized to deviation) */
	int		signal_interval_ms;
	int64_t		last_signal_us;
	
	/* UECP addressing */
	uint16_t	site_addr;
	uint8_t		enc_addr;
	uint8_t		uecp_seq;
	
	/* Encoder reference (UECP) */
	struct rds_encoder *encoder;
	
	/* UECP group decoder (for human-readable logging of Free Format groups) */
	struct rds_decoder *uecp_decoder;
	
	/* Callbacks (XDR-GTK) */
	void		(*tune_cb)(int freq_khz, void *arg);
	void		(*setting_cb)(const char *name, int val, void *arg);
	void		*cb_arg;
	
	/* Stats */
	uint64_t	groups_sent;
	uint64_t	groups_dropped;	/* Dropped due to full buffer */
	uint64_t	tx_bytes;
	uint64_t	rx_bytes;

	/* Connection tracking */
	char		client_ip[RDS_EP_IP_LEN];  /* IP of current/last client */
	int		client_port;               /* Port of current/last client */
	int64_t		connect_time_us;           /* time_us() when client connected */
	int64_t		last_rx_us;                /* time_us() of last received data (diagnostic) */
	uint64_t	total_connections;         /* Total accepted connections (lifetime) */

	/* XDR-GTK TCP authentication */
	char		password[256];             /* Password (empty = no auth required) */
	xdr_auth_state_t auth_state;           /* Current auth state */
	uint8_t		auth_salt[XDR_AUTH_SALT_LEN]; /* Random salt sent to client */
	int64_t		auth_deadline_us;          /* Deadline for receiving client hash */

	/* ASCII-G SETSPY monitoring */
	int		setspy_remaining;          /* Groups left to send in RDS Spy format (0=off) */

	/* XDR-GTK login tune suppression:
	 * After we send OK/a1, XDR-GTK's connection_dialog_callback fires within
	 * ~100ms and unconditionally sends T<conf.initial_freq>.  We suppress the
	 * first T command received within this grace window so the client adopts
	 * our current frequency instead of retuning to its last saved one. */
	int64_t		login_tune_grace_us;       /* Deadline: ignore first T until this time */

	/* XDR-GTK spectral scan state */
	int		scan_start_khz;    /* Sa: scan start frequency (kHz) */
	int		scan_stop_khz;     /* Sb: scan stop frequency (kHz) */
	int		scan_step_khz;     /* Sc: scan step (kHz) */
	int		scan_filter;       /* Sf: filter index (XDR mode) */
	int		scan_bw_khz;       /* Sw: bandwidth (kHz) */
	int		scan_antenna;      /* Sz: antenna index */
	int		scan_continuous;   /* 1 = Sm (continuous), 0 = S (single pass) */
	int		scan_active;       /* 1 = scan in progress */
	int		scan_orig_freq_khz;/* operating frequency before scan started */
	int		scan_points_total; /* total number of scan points */
	int		scan_points_done;  /* points measured so far */
	/* Per-point measurement state (max 2048 points: 87.5-108 MHz / 10 kHz step) */
#define SCAN_MAX_POINTS 2048
	uint8_t		scan_measured[SCAN_MAX_POINTS]; /* 1 = point measured */
	double		scan_power_acc[SCAN_MAX_POINTS]; /* accumulated dBFS */
	int		scan_power_cnt[SCAN_MAX_POINTS]; /* frame count per point */
	/* FFT accumulation buffer */
#define SCAN_FFT_SIZE 4096
	double		scan_fft_i[SCAN_FFT_SIZE];
	double		scan_fft_q[SCAN_FFT_SIZE];
	int		scan_fft_fill;     /* samples accumulated so far */
	int		scan_settle_frames;/* FFT frames to discard after retune */
	/* scan result accumulation */
	char		scan_result[16384];/* U response buffer */
	int		scan_result_len;
	/* scan retune callback */
	void		(*scan_cb)(int freq_khz, void *arg);
	void		*scan_cb_arg;
} rds_server_t;

/* ============================================================
 * API
 * ============================================================ */

/* Parse endpoint string.
 * TCP: "ip:port" (e.g. "127.0.0.1:7373")
 * Serial: "device,speed,8N1[,flow]"
 * Returns 0 on success, -1 on error. */
int rds_ep_parse(const char *str, rds_endpoint_t *ep);

/* Initialize server. All I/O is non-blocking.
 * Returns 0 on success, -1 on error. */
int rds_server_init(rds_server_t *srv, const char *endpoint,
		    rds_proto_t proto, struct rds_encoder *enc);

/* Cleanup server. */
void rds_server_cleanup(rds_server_t *srv);

/* Poll for I/O. Call from main loop.
 * - Accepts new connections
 * - Drains TX ring to socket
 * - Reads socket to RX ring
 * - Processes RX commands
 * Never blocks. Returns 0 normally, -1 on fatal error. */
int rds_server_poll(rds_server_t *srv);

/* Queue RDS group for transmission.
 * Writes to TX ring buffer - never blocks.
 * If buffer full, data is dropped (real-time, stale data useless). */
void rds_server_send_group(rds_server_t *srv,
			   const uint16_t blocks[4],
			   const uint8_t errors[4]);

/* Queue PI code (XDR-GTK). */
void rds_server_send_pi(rds_server_t *srv, uint16_t pi, int err);

/* Queue signal report (XDR-GTK). */
void rds_server_send_signal(rds_server_t *srv, double dbm,
			    int stereo, int forced_mono);

/* Queue frequency (XDR-GTK). */
void rds_server_send_freq(rds_server_t *srv, int freq_khz);

/* Queue reset (RDS Spy). */
void rds_server_send_reset(rds_server_t *srv);

/* Update signal state for periodic reporting. */
void rds_server_update_signal(rds_server_t *srv, double dbm, int stereo, double pilot_mag);

/* Set callbacks (XDR-GTK). */
void rds_server_set_callbacks(rds_server_t *srv,
			      void (*tune)(int freq_khz, void *arg),
			      void (*setting)(const char *name, int val, void *arg),
			      void *arg);

/* Check if client connected. */
int rds_server_connected(const rds_server_t *srv);

/* Set password for XDR-GTK TCP authentication.
 * Empty string or NULL disables password (no-auth mode).
 * Must be called before any client connects. */
void rds_server_set_password(rds_server_t *srv, const char *password);

/* Set de-emphasis for XDR-GTK (sent to client on login).
 * val: 0=50µs (EU), 1=75µs (US), -1=unknown/don't send. */
void rds_server_set_deemphasis(rds_server_t *srv, int val);

/* Get protocol name. */
const char *rds_proto_name(rds_proto_t proto);

/* Spectral scan support (XDR-GTK).
 * Call rds_server_scan_feed_iq() from the main loop after each sdr_read().
 * It FFTs the raw IQ buffer and fills in all scan points within the current
 * SDR window — no retuning needed if the SDR bandwidth covers the full range.
 * Only retuning when the window needs to shift to cover remaining points.
 *
 * scan_cb is called only when a retune is needed.
 * Returns 1 if scan still active, 0 if complete. */
void rds_server_set_scan_callback(rds_server_t *srv,
				  void (*cb)(int freq_khz, void *arg), void *arg);
int rds_server_scan_feed_iq(rds_server_t *srv,
			    const float *iq_buf, int count,
			    double center_hz, int sdr_rate);
/* Legacy poll stub — no-op, kept for compatibility. */
int rds_server_scan_poll(rds_server_t *srv);

#endif /* _RDS_PROTOCOL_H */
