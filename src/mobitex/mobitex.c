/* Mobitex protocol engine
 *
 * Protocol state machine, frame sequencing, link control parsing,
 * MPAK assembly/disassembly, and message dispatch.
 */

#define CHAN mobitex->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include <osmocom/cc/message.h>
#include "mobitex.h"
#include "frame.h"
#include "dsp.h"

/* Forward declarations for static functions */
static void mobitex_tx_frame(mobitex_t *mobitex, mobitex_msg_t *msg);
static int mobitex_emit_preamble(mobitex_t *mobitex, uint8_t *bits);
static void mobitex_assemble_message(mobitex_t *mobitex);
static void mobitex_tx_response(mobitex_t *mobitex, uint8_t frame_type, uint32_t dest_man);

int mobitex_create(const char *kanal, double rx_frequency, double tx_frequency,
                   const char *device, int use_sdr, int samplerate,
                   double rx_gain, double tx_gain,
                   int tx, int rx, int baudrate,
                   double deviation, double polarity,
                   int scrambling, int ramnet,
                   uint16_t bitsync, int frsync_index,
                   int base_id, int area_id, uint32_t base_man,
                   double svp_interval, int svp_type, int fbi,
                   uint8_t txpow, uint8_t rssi_proc, uint8_t rssi_period,
                   uint8_t scan_time, uint8_t bad_base, uint8_t good_base,
                   uint8_t better_base, uint16_t upfreq, uint16_t dofreq,
                   uint8_t cycle_time, uint8_t time_to_next,
                   uint8_t transaction_time, uint8_t eval_current,
                   uint8_t eval_others, uint8_t deep_sleep_inhibit,
                   uint8_t slot_length, uint8_t free_slots, uint8_t rand_slots,
                   uint8_t max_rep, uint8_t max_access, uint8_t timeout,
                   double fri_interval,
                   const char *write_rx_wave, const char *write_tx_wave,
                   const char *read_rx_wave, const char *read_tx_wave,
                   int loopback)
{
	mobitex_t *mobitex;
	int rc;

	mobitex = calloc(1, sizeof(*mobitex));
	if (!mobitex) {
		LOGP(DDSP, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	if (baudrate == 1200) {
		LOGP(DDSP, LOGL_NOTICE, "Mobitex-1200 mode selected (1200 baud, FFSK, 6-byte blocks, 21-bit CRC).\n");
		LOGP(DDSP, LOGL_NOTICE, "Mobitex-1200: 8-bit bitsync (0x%02X), %d-bit framesync, %d-bit baseID.\n",
		     (bitsync == MOBITEX_1200_BITSYNC_MOBILE) ? MOBITEX_1200_BITSYNC_MOBILE : MOBITEX_1200_BITSYNC_BASE,
		     MOBITEX_1200_FRSYNC_BITS, MOBITEX_1200_BASEID_BITS);
		/* TODO: Full Mobitex-1200 protocol support.
		 * The upper protocol layers (MPAK, DTESERV, Link_Control) are shared
		 * between both modes. The frame-level processing (block size, CRC,
		 * sync patterns) needs mode-specific paths in frame.c and dsp.c. */
	} else {
		LOGP(DDSP, LOGL_DEBUG, "Mobitex-8000 mode selected (8000 baud, GMSK).\n");
	}

	LOGP(DDSP, LOGL_DEBUG, "Creating 'Mobitex' instance for 'Kanal' = %s (sample rate %d).\n", kanal, samplerate);

	/* init general part of transceiver — FDD: separate TX and RX frequencies */
	rc = sender_create(&mobitex->sender, kanal, tx_frequency, rx_frequency, device, use_sdr, samplerate, rx_gain, tx_gain, 0, 0, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DDSP, LOGL_ERROR, "Failed to init transceiver process!\n");
		goto error;
	}

	/* init audio processing */
	rc = dsp_init_sender(mobitex, samplerate, baudrate, deviation, polarity);
	if (rc < 0) {
		LOGP(DDSP, LOGL_ERROR, "Failed to init audio processing!\n");
		goto error;
	}

	/* set configuration fields */
	mobitex->tx = tx;
	mobitex->rx = rx;
	mobitex->scrambling = scrambling;
	mobitex->ramnet = ramnet;
	mobitex->baudrate = baudrate;
	mobitex->bitsync = bitsync;
	mobitex->frsync_index = frsync_index;
	mobitex->tx_base_id = base_id;
	mobitex->tx_area_id = area_id;
	mobitex->tx_base_man = base_man;
	mobitex->tx_svp_interval = svp_interval;
	mobitex->tx_svp_type = svp_type;
	mobitex->tx_fbi = fbi;
	mobitex->tx_svp_timer = svp_interval;
	mobitex->tx_svp_auto = (tx ? 1 : 0);
	mobitex->tx_svp_seq = 0;

	/* SVP1/SVP3 parameters for base station TX (roaming + link) */
	mobitex->tx_txpow = txpow;
	mobitex->tx_rssi_proc = rssi_proc;
	mobitex->tx_rssi_period = rssi_period;
	mobitex->tx_scan_time = scan_time;
	mobitex->tx_bad_base = bad_base;
	mobitex->tx_good_base = good_base;
	mobitex->tx_better_base = better_base;
	mobitex->tx_upfreq = upfreq;
	mobitex->tx_dofreq = dofreq;

	/* SVP6 parameters for base station TX
	 * (ROM maps these to roaming params: RSSI_PROC, RSSI_PERIOD,
	 *  SCAN_TIME, BAD_BASE, GOOD_BASE, BETTER_BASE) */
	mobitex->tx_cycle_time = cycle_time;
	mobitex->tx_time_to_next = time_to_next;
	mobitex->tx_transaction_time = transaction_time;
	mobitex->tx_eval_current = eval_current;
	mobitex->tx_eval_others = eval_others;
	mobitex->tx_deep_sleep_inhibit = deep_sleep_inhibit;

	/* FRI parameters for base station TX */
	mobitex->tx_slot_length = slot_length;
	mobitex->tx_free_slots = free_slots;
	mobitex->tx_rand_slots = rand_slots;
	mobitex->tx_max_rep = max_rep;
	mobitex->tx_max_access = max_access;
	mobitex->tx_max_speech = 0;
	mobitex->tx_timeout = timeout;
	mobitex->tx_prio = 0;
	mobitex->tx_fri_interval = fri_interval;
	mobitex->tx_fri_timer = fri_interval;
	mobitex->state = MOBITEX_IDLE;
	mobitex->tx_state = MOBITEX_IDLE;
	mobitex->tx_access_retries = 0;
	mobitex->tx_access_max_retries = 3;
	mobitex->tx_access_backoff = 0.0;

	LOGP(DDSP, LOGL_NOTICE, "Created 'Kanal' %s\n", kanal);

	return 0;

error:
	mobitex_destroy(&mobitex->sender);

	return rc;
}

void mobitex_destroy(sender_t *sender)
{
	mobitex_t *mobitex = (mobitex_t *)sender;
	mobitex_msg_t *msg;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Destroying 'Mobitex' instance for 'Kanal' = %s.\n", sender->kanal);

	/* free TX message queue */
	while (mobitex->msg_list) {
		msg = mobitex->msg_list;
		mobitex->msg_list = msg->next;
		free(msg);
	}

	dsp_cleanup_sender(mobitex);
	sender_destroy(&mobitex->sender);
	free(mobitex);
}

/* ---- Frame type name lookup (matching PDW numbering) ---- */
static const char *frametype_names[] = {
	[0x00] = "Unknown",
	[0x01] = "MRM",    [0x02] = "ACK",    [0x03] = "NACK",
	[0x04] = "REB",    [0x05] = "RES",    [0x06] = "ABD",
	[0x07] = "ABL",    [0x08] = "ATD",    [0x09] = "ATL",
	[0x0A] = NULL,     [0x0B] = NULL,
	[0x0C] = "BKD",    [0x0D] = "BKT",    [0x0E] = "FRI",
	[0x0F] = "SVP",    [0x10] = "TST",    [0x11] = "AKT",
	[0x12] = "NAT",    [0x13] = "BBT",
};
#define FRAMETYPE_NAMES_COUNT (sizeof(frametype_names) / sizeof(frametype_names[0]))

/* ---- DTE service message type lookup (32 entries, MPAK class 3) ---- */
static const char *dteserv_names[32] = {
	[0]  = "UNKNOWN",
	[1]  = "LOGIN REQUEST",
	[2]  = "LOGIN GRANTED",
	[3]  = "LOGIN REFUSED",
	[4]  = "LOGOUT",
	[5]  = "LOGOUT ORDER",
	[6]  = "BORN",
	[7]  = "ACTIVE",
	[8]  = "INACTIVE",
	[9]  = "DIE",
	[10] = "LIVE",
	[11] = "ROAMING ORDER",
	[12] = "ROAMING MESSAGE",
	/* 13-14: Reserved */
	[15] = "GROUPLIST",
	[16] = "FLEXREQ",
	[17] = "FLEXLIST",
	[18] = "INFOREQ",
	[19] = "INFO",
	[20] = "TIME",
	[21] = "AREALIST",
	[22] = "ESN REQUEST",
	[23] = "ESN INFO",
	[24] = "MODE",
	/* 25-29: Reserved */
	[30] = "APPLICATION OPTIONS",
	[31] = "LOW POWER",
};

static const char *mobitex_frametype_name(uint8_t frame_id)
{
	if (frame_id < FRAMETYPE_NAMES_COUNT && frametype_names[frame_id])
		return frametype_names[frame_id];
	return "Unknown";
}

static const char *mobitex_dteserv_name(uint8_t mpak_type)
{
	if (mpak_type < 32 && dteserv_names[mpak_type])
		return dteserv_names[mpak_type];
	return "Reserved";
}

/* ---- Traffic state name lookup (MIS octet 7, bits 5-7) ---- */
static const char *traffic_state_names[] = {
	[0] = "OK",
	[1] = "FROM_MAIL",
	[2] = "IN_MAIL",
	[3] = "NO_TRANSFER",
	[4] = "ILLEGAL",
	[5] = "CONGEST",
	[6] = "ERROR",
};

static const char *mobitex_traffic_state_name(uint8_t ts)
{
	if (ts <= 6)
		return traffic_state_names[ts];
	return "Unknown";
}

/* ---- PSUBCOM type name lookup (MIS octet 8, bits 0-4, class=0) ---- */
static const char *mobitex_psubcom_name(uint8_t mpak_type, uint8_t extern_f)
{
	if (extern_f)
		return (mpak_type == 1) ? "EXTPAK" : "EXT_UNKNOWN";
	switch (mpak_type) {
	case 1: return "TEXT";
	case 2: return "DATA";
	case 3: return "STATUS";
	case 4: return "HPDATA";
	default: return "UNKNOWN";
	}
}

/* ---- DTESERV packet decoders ----
 * These decode the type-dependent component of each DTESERV packet type
 * per the MIS Packet Formats specification. The primary block bytes 6-17
 * contain the MPAK common component; the type-dependent component starts
 * at byte 6 of the FIRST FOLLOWING data block (block 1), or for packets
 * that fit in the primary block, at bytes 14-17 of block 0.
 *
 * For DTESERV packets, the "sender" in the MPAK common component is the
 * terminal subscription MAN, and the "addressee" is the network (MAN=1).
 * The type-dependent fields carry the actual service data.
 */

/* Decode BORN (Terminal Active for First Time) — MIS 8:42
 * Type-dependent: octets 9-11 = ESN (24 bits), octets 12-17 = reserved
 * In our block layout: block0 bytes 14-16 = ESN (bytes 14,15,16) */
static void mobitex_decode_dteserv_born(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t esn = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	             | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	             | (uint8_t)mobitex->rx_data_blocks[0][16];
	snprintf(buf, bufsize, "ESN=%06X", esn);
}

/* Decode ACTIVE (Terminal Active) — MIS 8:45
 * Same ESN layout as BORN */
static void mobitex_decode_dteserv_active(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t esn = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	             | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	             | (uint8_t)mobitex->rx_data_blocks[0][16];
	snprintf(buf, bufsize, "ESN=%06X", esn);
}

/* Decode ROAM (Roaming Message) — MIS 8:54
 * Same ESN layout as BORN/ACTIVE */
static void mobitex_decode_dteserv_roam(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t esn = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	             | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	             | (uint8_t)mobitex->rx_data_blocks[0][16];
	snprintf(buf, bufsize, "ESN=%06X", esn);
}

/* Decode LOGINREQ (Login Request) — MIS 8:31
 * Type-dependent: octets 9-11 = personal/HG subscription MAN,
 *                 octets 12-19 = password (8 chars, Mobitex text code)
 * In our block layout: block0 bytes 14-16 = subscription MAN
 * Password spans block0 byte 17 + block1 bytes 0-6 (if block_length > 1) */
static void mobitex_decode_dteserv_loginreq(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t sub_man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	                 | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	                 | (uint8_t)mobitex->rx_data_blocks[0][16];
	char password[9];
	int i;

	/* Password: byte 17 of block 0 + bytes 0-6 of block 1 */
	password[0] = (char)(uint8_t)mobitex->rx_data_blocks[0][17];
	if (mobitex->link_control.block_length > 1) {
		for (i = 0; i < 7 && i < MOBITEX_DATA_BYTES; i++)
			password[1 + i] = (char)(uint8_t)mobitex->rx_data_blocks[1][i];
	} else {
		for (i = 1; i < 8; i++)
			password[i] = ' ';
	}
	password[8] = '\0';

	snprintf(buf, bufsize, "SubMAN=%06X Password=\"%.8s\"", sub_man, password);
}

/* Decode LOGINGRA (Login Granted) — MIS 8:34
 * Type-dependent: octets 9-11 = personal/HG subscription MAN */
static void mobitex_decode_dteserv_logingra(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t sub_man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	                 | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	                 | (uint8_t)mobitex->rx_data_blocks[0][16];
	snprintf(buf, bufsize, "SubMAN=%06X", sub_man);
}

/* Decode LOGINREF (Login Refused) — MIS 8:36
 * Type-dependent: octets 9-11 = personal/HG subscription MAN,
 *                 octet 12 = cause code */
static void mobitex_decode_dteserv_loginref(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t sub_man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	                 | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	                 | (uint8_t)mobitex->rx_data_blocks[0][16];
	uint8_t cause = (uint8_t)mobitex->rx_data_blocks[0][17];
	snprintf(buf, bufsize, "SubMAN=%06X Cause=%d", sub_man, cause);
}

/* Decode LOGOUT — MIS 8:38
 * Type-dependent: octets 9-11 = personal/HG subscription MAN */
static void mobitex_decode_dteserv_logout(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t sub_man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	                 | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	                 | (uint8_t)mobitex->rx_data_blocks[0][16];
	snprintf(buf, bufsize, "SubMAN=%06X", sub_man);
}

/* Decode LOGOUTORD (Logout Order) — MIS 8:40
 * Type-dependent: octets 9-11 = personal/HG subscription MAN */
static void mobitex_decode_dteserv_logoutord(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t sub_man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	                 | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	                 | (uint8_t)mobitex->rx_data_blocks[0][16];
	snprintf(buf, bufsize, "SubMAN=%06X", sub_man);
}

/* Decode TIME (Time Information) — MIS 8:67
 * Type-dependent: octets 9-11 = time (3 bytes, BCD or binary)
 * In our block layout: block0 bytes 14-16 */
static void mobitex_decode_dteserv_time(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint8_t b0 = (uint8_t)mobitex->rx_data_blocks[0][14];
	uint8_t b1 = (uint8_t)mobitex->rx_data_blocks[0][15];
	uint8_t b2 = (uint8_t)mobitex->rx_data_blocks[0][16];
	/* Time is typically encoded as hours/minutes/seconds or a 24-bit counter */
	snprintf(buf, bufsize, "Time=%02X:%02X:%02X", b0, b1, b2);
}

/* Decode GROUPLIST (List of Group MANs) — MIS 8:56
 * Type-dependent: octet 9 = number of groups, then 3-byte MANs
 * In our block layout: block0 byte 14 = count, bytes 15-17 = first MAN,
 * then block1+ for remaining MANs */
static void mobitex_decode_dteserv_grouplist(mobitex_t *mobitex, char *buf, int bufsize)
{
	int count = (uint8_t)mobitex->rx_data_blocks[0][14];
	int pos, i, blk, byte_idx;
	uint32_t man;

	pos = snprintf(buf, bufsize, "Groups=%d", count);

	/* First MAN from block0 bytes 15-17 */
	if (count > 0 && pos < bufsize - 1) {
		man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 16)
		    | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][16] << 8)
		    | (uint8_t)mobitex->rx_data_blocks[0][17];
		pos += snprintf(buf + pos, bufsize - pos, " MAN=%06X", man);
	}

	/* Remaining MANs from subsequent blocks */
	byte_idx = 0;
	blk = 1;
	for (i = 1; i < count && blk < mobitex->link_control.block_length && blk < MOBITEX_MAX_BLOCKS; i++) {
		if (byte_idx + 2 >= MOBITEX_DATA_BYTES) {
			blk++;
			byte_idx = 0;
		}
		if (blk >= mobitex->link_control.block_length || blk >= MOBITEX_MAX_BLOCKS)
			break;
		man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[blk][byte_idx] << 16)
		    | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[blk][byte_idx + 1] << 8)
		    | (uint8_t)mobitex->rx_data_blocks[blk][byte_idx + 2];
		if (pos < bufsize - 1)
			pos += snprintf(buf + pos, bufsize - pos, " MAN=%06X", man);
		byte_idx += 3;
	}
}

/* Decode ESNREQ (ESN Requested) — MIS 8:72
 * No type-dependent data */
static void mobitex_decode_dteserv_esnreq(mobitex_t *mobitex __attribute__((unused)), char *buf, int bufsize)
{
	snprintf(buf, bufsize, "(ESN requested)");
}

/* Decode ESNINFO (ESN Information) — MIS 8:73
 * Type-dependent: octets 9-11 = ESN */
static void mobitex_decode_dteserv_esninfo(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint32_t esn = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][14] << 16)
	             | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 8)
	             | (uint8_t)mobitex->rx_data_blocks[0][16];
	snprintf(buf, bufsize, "ESN=%06X", esn);
}

/* Decode MODE (Mode Information) — MIS 8:75
 * Type-dependent: octet 9 = mode byte */
static void mobitex_decode_dteserv_mode(mobitex_t *mobitex, char *buf, int bufsize)
{
	uint8_t mode = (uint8_t)mobitex->rx_data_blocks[0][14];
	snprintf(buf, bufsize, "Mode=%d", mode);
}

/* Decode ROAMORD (Roaming Order) — MIS 8:53
 * No type-dependent data (just the common component) */
static void mobitex_decode_dteserv_roamord(mobitex_t *mobitex __attribute__((unused)), char *buf, int bufsize)
{
	snprintf(buf, bufsize, "(roaming order)");
}

/* Decode FLEXREQ (List of Logged-in MANs Requested) — MIS 8:58
 * No type-dependent data */
static void mobitex_decode_dteserv_flexreq(mobitex_t *mobitex __attribute__((unused)), char *buf, int bufsize)
{
	snprintf(buf, bufsize, "(flex list requested)");
}

/* Decode FLEXLIST (List of Subscriptions Logged-in) — MIS 8:60
 * Type-dependent: octet 9 = number of subscriptions, then 3-byte MANs */
static void mobitex_decode_dteserv_flexlist(mobitex_t *mobitex, char *buf, int bufsize)
{
	int count = (uint8_t)mobitex->rx_data_blocks[0][14];
	int pos, i, blk, byte_idx;
	uint32_t man;

	pos = snprintf(buf, bufsize, "Subscriptions=%d", count);

	if (count > 0 && pos < bufsize - 1) {
		man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][15] << 16)
		    | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[0][16] << 8)
		    | (uint8_t)mobitex->rx_data_blocks[0][17];
		pos += snprintf(buf + pos, bufsize - pos, " MAN=%06X", man);
	}

	byte_idx = 0;
	blk = 1;
	for (i = 1; i < count && blk < mobitex->link_control.block_length && blk < MOBITEX_MAX_BLOCKS; i++) {
		if (byte_idx + 2 >= MOBITEX_DATA_BYTES) {
			blk++;
			byte_idx = 0;
		}
		if (blk >= mobitex->link_control.block_length || blk >= MOBITEX_MAX_BLOCKS)
			break;
		man = ((uint32_t)(uint8_t)mobitex->rx_data_blocks[blk][byte_idx] << 16)
		    | ((uint32_t)(uint8_t)mobitex->rx_data_blocks[blk][byte_idx + 1] << 8)
		    | (uint8_t)mobitex->rx_data_blocks[blk][byte_idx + 2];
		if (pos < bufsize - 1)
			pos += snprintf(buf + pos, bufsize - pos, " MAN=%06X", man);
		byte_idx += 3;
	}
}

/* Decode INFOREQ (Terminal Information Requested) — MIS 8:62
 * No type-dependent data */
static void mobitex_decode_dteserv_inforeq(mobitex_t *mobitex __attribute__((unused)), char *buf, int bufsize)
{
	snprintf(buf, bufsize, "(info requested)");
}

/* Decode INFO (Terminal Information) — MIS 8:63
 * Type-dependent: variable-length terminal information.
 * Assemble from data blocks like a regular message. */
static void mobitex_decode_dteserv_info(mobitex_t *mobitex, char *buf, int bufsize)
{
	int blk, k, n, pos = 0;

	for (blk = 1; blk < mobitex->link_control.block_length && blk < MOBITEX_MAX_BLOCKS; blk++) {
		if (blk == mobitex->link_control.block_length - 1 && mobitex->link_control.bytes_last > 0)
			n = mobitex->link_control.bytes_last;
		else
			n = MOBITEX_DATA_BYTES;
		if (n > MOBITEX_DATA_BYTES)
			n = MOBITEX_DATA_BYTES;
		for (k = 0; k < n && pos < bufsize - 3; k++) {
			uint8_t b = (uint8_t)mobitex->rx_data_blocks[blk][k];
			/* Hex-dump info bytes */
			pos += snprintf(buf + pos, bufsize - pos, "%02X", b);
		}
	}
	if (pos == 0)
		snprintf(buf, bufsize, "(empty)");
}

/* Decode AREALIST (Area ID Information) — MIS 8:69
 * Type-dependent: list of area IDs */
static void mobitex_decode_dteserv_arealist(mobitex_t *mobitex, char *buf, int bufsize)
{
	int blk, k, pos = 0;

	for (blk = 1; blk < mobitex->link_control.block_length && blk < MOBITEX_MAX_BLOCKS; blk++) {
		for (k = 0; k + 1 < MOBITEX_DATA_BYTES && pos < bufsize - 8; k += 2) {
			uint16_t area = ((uint8_t)mobitex->rx_data_blocks[blk][k] << 8)
			              | (uint8_t)mobitex->rx_data_blocks[blk][k + 1];
			if (area == 0)
				break;
			pos += snprintf(buf + pos, bufsize - pos, " Area=%04X", area);
		}
	}
	if (pos == 0)
		snprintf(buf, bufsize, "(empty)");
}

/* Dispatch DTESERV type-dependent decoding. Returns decoded text in buf. */
static void mobitex_decode_dteserv(mobitex_t *mobitex, uint8_t mpak_type, char *buf, int bufsize)
{
	switch (mpak_type) {
	case MPAK_DTE_BORN:       mobitex_decode_dteserv_born(mobitex, buf, bufsize); break;
	case MPAK_DTE_ACTIVE:     mobitex_decode_dteserv_active(mobitex, buf, bufsize); break;
	case MPAK_DTE_ROAM:       mobitex_decode_dteserv_roam(mobitex, buf, bufsize); break;
	case MPAK_DTE_LOGINREQ:   mobitex_decode_dteserv_loginreq(mobitex, buf, bufsize); break;
	case MPAK_DTE_LOGINGRA:   mobitex_decode_dteserv_logingra(mobitex, buf, bufsize); break;
	case MPAK_DTE_LOGINREF:   mobitex_decode_dteserv_loginref(mobitex, buf, bufsize); break;
	case MPAK_DTE_LOGOUT:     mobitex_decode_dteserv_logout(mobitex, buf, bufsize); break;
	case MPAK_DTE_LOGOUTORD:  mobitex_decode_dteserv_logoutord(mobitex, buf, bufsize); break;
	case MPAK_DTE_TIME:       mobitex_decode_dteserv_time(mobitex, buf, bufsize); break;
	case MPAK_DTE_GROUPLIST:  mobitex_decode_dteserv_grouplist(mobitex, buf, bufsize); break;
	case MPAK_DTE_ESNREQ:     mobitex_decode_dteserv_esnreq(mobitex, buf, bufsize); break;
	case MPAK_DTE_ESNINFO:    mobitex_decode_dteserv_esninfo(mobitex, buf, bufsize); break;
	case MPAK_DTE_MODE:       mobitex_decode_dteserv_mode(mobitex, buf, bufsize); break;
	case MPAK_DTE_ROAMORD:    mobitex_decode_dteserv_roamord(mobitex, buf, bufsize); break;
	case MPAK_DTE_FLEXREQ:    mobitex_decode_dteserv_flexreq(mobitex, buf, bufsize); break;
	case MPAK_DTE_FLEXLIST:   mobitex_decode_dteserv_flexlist(mobitex, buf, bufsize); break;
	case MPAK_DTE_INFOREQ:    mobitex_decode_dteserv_inforeq(mobitex, buf, bufsize); break;
	case MPAK_DTE_INFO:       mobitex_decode_dteserv_info(mobitex, buf, bufsize); break;
	case MPAK_DTE_AREALIST:   mobitex_decode_dteserv_arealist(mobitex, buf, bufsize); break;
	case MPAK_DTE_INACTIVE:
	case MPAK_DTE_DIE:
	case MPAK_DTE_LIVE:
	case MPAK_DTE_APPOPTS:
	case MPAK_DTE_LOWPOWER:
		/* These have no type-dependent data or are handled generically */
		buf[0] = '\0';
		break;
	default:
		snprintf(buf, bufsize, "(type %d)", mpak_type);
		break;
	}
}

/* Decode PSUBCOM type-dependent component.
 * TEXT: octets 9-11 = time, octets 12+ = text (Mobitex text code)
 * DATA: octets 9-11 = time, octets 12+ = data (binary)
 * STATUS: octets 9-11 = time, octet 12 = status code
 * HPDATA: octets 9-11 = time, octet 12 = protocol ID, octets 13+ = data */
static void mobitex_decode_psubcom(mobitex_t *mobitex, uint8_t mpak_type, uint8_t extern_f, char *buf, int bufsize)
{
	int pos = 0;
	uint8_t time_b[3];

	/* Time field from block0 bytes 14-16 (MPAK octets 9-11) */
	time_b[0] = (uint8_t)mobitex->rx_data_blocks[0][14];
	time_b[1] = (uint8_t)mobitex->rx_data_blocks[0][15];
	time_b[2] = (uint8_t)mobitex->rx_data_blocks[0][16];

	if (extern_f) {
		pos += snprintf(buf + pos, bufsize - pos, "Time=%02X%02X%02X ",
				time_b[0], time_b[1], time_b[2]);
		/* EXTPAK: raw data follows */
		mobitex_assemble_message(mobitex);
		snprintf(buf + pos, bufsize - pos, "%s", mobitex->rx_msg_buf);
		return;
	}

	switch (mpak_type) {
	case MPAK_PSUB_STATUS: {
		/* STATUS: time + 1-byte status code at block0 byte 17 */
		uint8_t status_code = (uint8_t)mobitex->rx_data_blocks[0][17];
		snprintf(buf, bufsize, "Time=%02X%02X%02X StatusCode=%d",
			 time_b[0], time_b[1], time_b[2], status_code);
		break;
	}
	case MPAK_PSUB_HPDATA:
		/* HPDATA: time + protocol ID at block0 byte 17, then data */
		pos += snprintf(buf + pos, bufsize - pos, "Time=%02X%02X%02X HPID=%d ",
				time_b[0], time_b[1], time_b[2],
				(uint8_t)mobitex->rx_data_blocks[0][17]);
		mobitex_assemble_message(mobitex);
		snprintf(buf + pos, bufsize - pos, "%s", mobitex->rx_msg_buf);
		break;
	case MPAK_PSUB_TEXT:
	case MPAK_PSUB_DATA:
	default:
		/* TEXT/DATA: time + message data from subsequent blocks */
		pos += snprintf(buf + pos, bufsize - pos, "Time=%02X%02X%02X ",
				time_b[0], time_b[1], time_b[2]);
		mobitex_assemble_message(mobitex);
		snprintf(buf + pos, bufsize - pos, "%s", mobitex->rx_msg_buf);
		break;
	}
}

/* Assemble message data from multi-block frame into rx_msg_buf.
 * Primary block bytes 6-17 are MPAK header for MRM, so message data
 * starts from block 1. Last block uses bytes_last to determine valid bytes. */
static void mobitex_assemble_message(mobitex_t *mobitex)
{
	int block_length = mobitex->link_control.block_length;
	int bytes_last = mobitex->link_control.bytes_last;
	int blk, k, n;

	mobitex->rx_msg_len = 0;

	/* Data blocks start at block 1 (block 0 is primary with link control + MPAK header) */
	for (blk = 1; blk < block_length && blk < MOBITEX_MAX_BLOCKS; blk++) {
		/* Determine how many bytes are valid in this block */
		if (blk == block_length - 1 && bytes_last > 0)
			n = bytes_last;
		else
			n = MOBITEX_DATA_BYTES;

		if (n > MOBITEX_DATA_BYTES)
			n = MOBITEX_DATA_BYTES;

		for (k = 0; k < n; k++) {
			if (mobitex->rx_msg_len < (int)sizeof(mobitex->rx_msg_buf) - 1)
				mobitex->rx_msg_buf[mobitex->rx_msg_len++] = (char)mobitex->rx_data_blocks[blk][k];
		}
	}

	mobitex->rx_msg_buf[mobitex->rx_msg_len] = '\0';
}

/* Process MRM frame: parse MPAK header, decode type-dependent data, dispatch */
static void mobitex_rx_process_mrm(mobitex_t *mobitex)
{
	uint8_t block20[20];
	char base_id_str[5];
	char decoded_text[6000];
	char flags_str[64];
	char fec_note[32];
	int flags_pos;
	int k;

	/* Reconstruct 20-byte block from primary block data for MPAK parsing */
	for (k = 0; k < MOBITEX_DATA_BYTES; k++)
		block20[k] = (uint8_t)mobitex->rx_data_blocks[0][k];
	block20[18] = 0;
	block20[19] = 0;

	mobitex_parse_mpak_header(block20, &mobitex->mpak_header);

	/* Log MPAK common component with all MIS fields */
	LOGP_CHAN(DDSP, LOGL_INFO, "MRM: SenderMAN=%06X DestMAN=%06X Class=%d Type=%d "
		  "TrafficState=%s(%d) Flags=[%s%s%s%s%s] HPID=%d\n",
		  mobitex->mpak_header.sender_man,
		  mobitex->mpak_header.dest_man,
		  mobitex->mpak_header.mpak_class,
		  mobitex->mpak_header.mpak_type,
		  mobitex_traffic_state_name(mobitex->mpak_header.traffic_state),
		  mobitex->mpak_header.traffic_state,
		  mobitex->mpak_header.mailbox_f ? "MAILBOX " : "",
		  mobitex->mpak_header.posack_f ? "POSACK " : "",
		  mobitex->mpak_header.sendlist_f ? "SENDLIST " : "",
		  mobitex->mpak_header.unknown_f ? "UNKNOWN " : "",
		  mobitex->mpak_header.extern_f ? "EXTERN " : "",
		  mobitex->mpak_header.hpid);

	/* Build flags string for output */
	flags_pos = 0;
	flags_str[0] = '\0';
	if (mobitex->mpak_header.traffic_state != MPAK_TS_OK)
		flags_pos += snprintf(flags_str + flags_pos, sizeof(flags_str) - flags_pos,
				      " [%s]", mobitex_traffic_state_name(mobitex->mpak_header.traffic_state));
	if (mobitex->mpak_header.posack_f)
		flags_pos += snprintf(flags_str + flags_pos, sizeof(flags_str) - flags_pos, " [POSACK]");
	if (mobitex->mpak_header.mailbox_f)
		flags_pos += snprintf(flags_str + flags_pos, sizeof(flags_str) - flags_pos, " [MAILBOX]");
	if (mobitex->mpak_header.unknown_f)
		flags_pos += snprintf(flags_str + flags_pos, sizeof(flags_str) - flags_pos, " [UNKNOWN]");
	if (mobitex->mpak_header.extern_f)
		flags_pos += snprintf(flags_str + flags_pos, sizeof(flags_str) - flags_pos, " [EXTERN]");

	/* Decode type-dependent component based on packet class */
	decoded_text[0] = '\0';
	if (mobitex->mpak_header.mpak_class == MPAK_CLASS_DTESERV) {
		LOGP_CHAN(DDSP, LOGL_INFO, "DTESERV: %s (type %d)\n",
			  mobitex_dteserv_name(mobitex->mpak_header.mpak_type),
			  mobitex->mpak_header.mpak_type);
		mobitex_decode_dteserv(mobitex, mobitex->mpak_header.mpak_type,
				       decoded_text, sizeof(decoded_text));
	} else if (mobitex->mpak_header.mpak_class == MPAK_CLASS_PSUBCOM) {
		LOGP_CHAN(DDSP, LOGL_INFO, "PSUBCOM: %s (type %d)\n",
			  mobitex_psubcom_name(mobitex->mpak_header.mpak_type,
					       mobitex->mpak_header.extern_f),
			  mobitex->mpak_header.mpak_type);
		mobitex_decode_psubcom(mobitex, mobitex->mpak_header.mpak_type,
				       mobitex->mpak_header.extern_f,
				       decoded_text, sizeof(decoded_text));
	} else {
		/* Unknown class: fall back to raw message assembly */
		mobitex_assemble_message(mobitex);
		snprintf(decoded_text, sizeof(decoded_text), "%s", mobitex->rx_msg_buf);
	}

	/* Append FEC correction note if errors were corrected */
	fec_note[0] = '\0';
	if (mobitex->rx_fec_errors_total > 0)
		snprintf(fec_note, sizeof(fec_note), " [FEC:%d]",
			 mobitex->rx_fec_errors_total);

	/* Build final message: class/type name + flags + decoded data + FEC note */
	{
		char final_msg[8192];
		const char *class_type_name;
		int pos = 0;

		if (mobitex->mpak_header.mpak_class == MPAK_CLASS_DTESERV)
			class_type_name = mobitex_dteserv_name(mobitex->mpak_header.mpak_type);
		else if (mobitex->mpak_header.mpak_class == MPAK_CLASS_PSUBCOM)
			class_type_name = mobitex_psubcom_name(mobitex->mpak_header.mpak_type,
							       mobitex->mpak_header.extern_f);
		else
			class_type_name = "UNKNOWN";

		pos = snprintf(final_msg, sizeof(final_msg), "%s", class_type_name);
		if (flags_str[0])
			pos += snprintf(final_msg + pos, sizeof(final_msg) - pos, "%s", flags_str);
		if (decoded_text[0])
			pos += snprintf(final_msg + pos, sizeof(final_msg) - pos, ": %s", decoded_text);
		if (fec_note[0])
			pos += snprintf(final_msg + pos, sizeof(final_msg) - pos, "%s", fec_note);

		/* Format Base ID as 4-char hex string (AreaID + BaseID) */
		snprintf(base_id_str, sizeof(base_id_str), "%02X%02X",
			 mobitex->frame_header.area_id, mobitex->frame_header.base_id);

		/* Dispatch to message receive handler */
		mobitex_msg_receive(CHAN, base_id_str,
				    mobitex->link_control.frame_id,
				    mobitex->mpak_header.dest_man,
				    mobitex->mpak_header.sender_man,
				    final_msg);
	}
}

/* Process SVP frame: parse sweep type and type-specific parameters.
 *
 * Byte layout reverse-engineered from Palm VIIx ROM MobitexProcessSweepFrame
 * at 0x10dd3fb2, cross-referenced with PA01 response builder at 0x10dcfe40
 * and MIS Open PA01 parameter specification.
 *
 * Primary block layout (all sweep types):
 *   bytes 0-5:  Link Control (MAN, FrameID=0x0F, SeqNum, BlkLen, BytesLast)
 *   byte 6:     Sweep type (0-6)
 *
 * SVP1/SVP3 (FRI + link parameters, bytes 7-17 memcpy'd as 11-byte block):
 *   byte 7:  MAX_REP         byte 13: RAND_SLOTS
 *   byte 8:  SLOT_LENGTH     byte 14: MAX_SPEECH
 *   byte 9:  FREE_SLOTS      byte 15: MAX_ACCESS
 *   byte 10: RSSI_PERIOD     byte 16: TXPOW
 *   byte 11: TIMEOUT         byte 17: (reserved/prio)
 *   byte 12: (reserved)
 *   Block 1+: neighbour channel list (2-byte BE channel numbers)
 *
 * SVP2/SVP4 (channel change):
 *   byte 7:     direction (1=uplink)
 *   bytes 10-11: channel number (16-bit big-endian)
 *
 * SVP5 (traffic list / group query):
 *   bytes 14+: MAN list with bitmask
 *
 * SVP6 (battery-saving parameters, PA07 command, individual byte assignments):
 *   byte 7:  CYCLE_TIME          byte 10: EVAL_CURRENT
 *   byte 8:  TIME_TO_NEXT        byte 11: EVAL_OTHERS
 *   byte 9:  TRANSACTION_TIME    byte 12: (MAN count)
 *   byte 13 upper nibble: DEEP_SLEEP_INHIBIT
 */
static void mobitex_rx_process_svp(mobitex_t *mobitex)
{
	char base_id_str[5];
	char svp_text[1024];
	int pos;
	int blk, k, ch_num;
	double freq;
	uint8_t stype;

	/* Primary block byte 6 = sweep type, byte 9 = FBI */
	stype = (uint8_t)mobitex->rx_data_blocks[0][6];
	mobitex->sweep.type = stype;
	mobitex->sweep.fbi = (uint8_t)mobitex->rx_data_blocks[0][9];
	mobitex->sweep.has_channel_list = (mobitex->link_control.block_length > 1) ? 1 : 0;

	/* Format Base ID as 4-char hex string (AreaID + BaseID) */
	snprintf(base_id_str, sizeof(base_id_str), "%02X%02X",
		 mobitex->frame_header.area_id, mobitex->frame_header.base_id);

	pos = snprintf(svp_text, sizeof(svp_text), "SweepType=%d FBI=%d",
		       stype, mobitex->sweep.fbi);

	switch (stype) {
	case 1:
	case 3:
		/* SVP1/SVP3: FRI + link parameters in bytes 7-17 */
		mobitex->sweep.max_rep      = (uint8_t)mobitex->rx_data_blocks[0][7];
		mobitex->sweep.slot_length  = (uint8_t)mobitex->rx_data_blocks[0][8];
		mobitex->sweep.free_slots   = (uint8_t)mobitex->rx_data_blocks[0][9];
		mobitex->sweep.rssi_period  = (uint8_t)mobitex->rx_data_blocks[0][10];
		mobitex->sweep.timeout      = (uint8_t)mobitex->rx_data_blocks[0][11];
		mobitex->sweep.rand_slots   = (uint8_t)mobitex->rx_data_blocks[0][13];
		mobitex->sweep.max_speech   = (uint8_t)mobitex->rx_data_blocks[0][14];
		mobitex->sweep.max_access   = (uint8_t)mobitex->rx_data_blocks[0][15];
		mobitex->sweep.txpow        = (uint8_t)mobitex->rx_data_blocks[0][16];

		LOGP_CHAN(DDSP, LOGL_INFO, "SVP%d: MaxRep=%d SlotLen=%d FreeSlots=%d "
			  "RandSlots=%d MaxAccess=%d MaxSpeech=%d Timeout=%d "
			  "TxPow=%d RssiPeriod=%d\n",
			  stype,
			  mobitex->sweep.max_rep, mobitex->sweep.slot_length,
			  mobitex->sweep.free_slots, mobitex->sweep.rand_slots,
			  mobitex->sweep.max_access, mobitex->sweep.max_speech,
			  mobitex->sweep.timeout, mobitex->sweep.txpow,
			  mobitex->sweep.rssi_period);

		pos += snprintf(svp_text + pos, sizeof(svp_text) - pos,
				" MaxRep=%d SlotLen=%d FreeSlots=%d RandSlots=%d "
				"MaxAccess=%d TxPow=%d Timeout=%d",
				mobitex->sweep.max_rep, mobitex->sweep.slot_length,
				mobitex->sweep.free_slots, mobitex->sweep.rand_slots,
				mobitex->sweep.max_access, mobitex->sweep.txpow,
				mobitex->sweep.timeout);
		break;

	case 2:
	case 4: {
		/* SVP2/SVP4: channel change — byte 7 = direction, bytes 10-11 = channel */
		uint8_t direction = (uint8_t)mobitex->rx_data_blocks[0][7];
		uint16_t channel = ((uint8_t)mobitex->rx_data_blocks[0][10] << 8)
				 | (uint8_t)mobitex->rx_data_blocks[0][11];

		if (direction == 1) {
			mobitex->sweep.upfreq = channel;
			freq = mobitex_fbi_to_freq(mobitex->sweep.fbi, channel);
			LOGP_CHAN(DDSP, LOGL_INFO, "SVP%d: UPFREQ Ch=%d (%.4f MHz)\n",
				  stype, channel, freq);
			pos += snprintf(svp_text + pos, sizeof(svp_text) - pos,
					" UPFREQ=%d (%.4fMHz)", channel, freq);
		} else {
			mobitex->sweep.dofreq = channel;
			freq = mobitex_fbi_to_freq(mobitex->sweep.fbi, channel);
			LOGP_CHAN(DDSP, LOGL_INFO, "SVP%d: DOFREQ Ch=%d (%.4f MHz)\n",
				  stype, channel, freq);
			pos += snprintf(svp_text + pos, sizeof(svp_text) - pos,
					" DOFREQ=%d (%.4fMHz)", channel, freq);
		}
		break;
	}

	case 6:
		/* SVP6: battery-saving parameters (PA07 command) */
		mobitex->sweep.cycle_time        = (uint8_t)mobitex->rx_data_blocks[0][7];
		mobitex->sweep.time_to_next      = (uint8_t)mobitex->rx_data_blocks[0][8];
		mobitex->sweep.transaction_time  = (uint8_t)mobitex->rx_data_blocks[0][9];
		mobitex->sweep.eval_current      = (uint8_t)mobitex->rx_data_blocks[0][10];
		mobitex->sweep.eval_others       = (uint8_t)mobitex->rx_data_blocks[0][11];
		mobitex->sweep.deep_sleep_inhibit = ((uint8_t)mobitex->rx_data_blocks[0][13]) >> 4;

		LOGP_CHAN(DDSP, LOGL_INFO, "SVP6: CycleTime=%d TimeToNext=%d TransactionTime=%d "
			  "EvalCurrent=%d EvalOthers=%d DeepSleepInhibit=%d\n",
			  mobitex->sweep.cycle_time, mobitex->sweep.time_to_next,
			  mobitex->sweep.transaction_time, mobitex->sweep.eval_current,
			  mobitex->sweep.eval_others, mobitex->sweep.deep_sleep_inhibit);

		pos += snprintf(svp_text + pos, sizeof(svp_text) - pos,
				" CycleTime=%d TimeToNext=%d TransactionTime=%d "
				"EvalCurrent=%d EvalOthers=%d DeepSleepInhibit=%d",
				mobitex->sweep.cycle_time, mobitex->sweep.time_to_next,
				mobitex->sweep.transaction_time, mobitex->sweep.eval_current,
				mobitex->sweep.eval_others, mobitex->sweep.deep_sleep_inhibit);
		break;

	case 5:
		/* SVP5: traffic list / group query — log raw bytes */
		LOGP_CHAN(DDSP, LOGL_INFO, "SVP5: (traffic list query)\n");
		break;

	default:
		LOGP_CHAN(DDSP, LOGL_INFO, "SVP: SweepType=%d (unknown)\n", stype);
		break;
	}

	/* Display neighbour/slave channel frequencies from additional data blocks
	 * (SVP1/SVP3 carry channel lists in blocks 1+) */
	if (mobitex->sweep.has_channel_list && (stype == 1 || stype == 3)) {
		for (blk = 1; blk < mobitex->link_control.block_length && blk < MOBITEX_MAX_BLOCKS; blk++) {
			for (k = 0; k + 1 < MOBITEX_DATA_BYTES; k += 2) {
				ch_num = ((uint8_t)mobitex->rx_data_blocks[blk][k] << 8)
				       | (uint8_t)mobitex->rx_data_blocks[blk][k + 1];
				if (ch_num == 0)
					break;
				freq = mobitex_fbi_to_freq(mobitex->sweep.fbi, ch_num);
				if (freq > 0)
					pos += snprintf(svp_text + pos, sizeof(svp_text) - pos,
							" Ch%d=%.4fMHz", ch_num, freq);
				else
					pos += snprintf(svp_text + pos, sizeof(svp_text) - pos,
							" Ch%d=FBI?%d", ch_num, mobitex->sweep.fbi);
				if (pos >= (int)sizeof(svp_text) - 1)
					break;
			}
			if (pos >= (int)sizeof(svp_text) - 1)
				break;
		}
	}

	mobitex_msg_receive(CHAN, base_id_str,
			    mobitex->link_control.frame_id,
			    mobitex->link_control.dest_man,
			    0, svp_text);
}

/* ---- Channel Access (Slotted Aloha) ---- */

/* Return 1 if this instance is configured as a mobile (bitsync == 0x3333) */
int mobitex_is_mobile_mode(mobitex_t *mobitex)
{
	return (mobitex->bitsync == MOBITEX_BITSYNC_MOBILE) ? 1 : 0;
}

/* Build and transmit an access request frame (ABD or ABL).
 * Mobile-originated: uses MOBITEX_BITSYNC_MOBILE (0x3333) for bitsync.
 * frame_type must be MOBITEX_FT_ABD (0x06) or MOBITEX_FT_ABL (0x07). */
static void mobitex_tx_access_request(mobitex_t *mobitex, uint8_t frame_type)
{
	uint8_t bits[500]; /* preamble(128) + framesync(16) + header(24) + block(240) */
	uint8_t block_bits[MOBITEX_BLOCK_BITS];
	uint8_t block20[20];
	uint8_t data18[MOBITEX_DATA_BYTES];
	mobitex_frame_header_t hdr;
	mobitex_link_control_t lc;
	int pos, i, nsamples;
	uint16_t sync_word;
	int frsync_idx;
	uint32_t dest_man = 0;

	/* Use dest_man from the first queued message, if any */
	if (mobitex->msg_list)
		dest_man = mobitex->msg_list->dest_man;

	/* Bitsync preamble: 8 repetitions for receiver acquisition */
	pos = mobitex_emit_preamble(mobitex, bits);

	/* Framesync */
	frsync_idx = mobitex->frsync_index;
	if (frsync_idx < 0 || frsync_idx >= 18)
		frsync_idx = 1; /* default: index 1 = 0xB433 (US RAM Mobile Data) */
	sync_word = mobitex_frsyncs[frsync_idx];
	for (i = 15; i >= 0; i--)
		bits[pos++] = (sync_word >> i) & 1;

	/* Frame header */
	memset(&hdr, 0, sizeof(hdr));
	hdr.base_id = mobitex->tx_base_id;
	hdr.area_id = mobitex->tx_area_id;
	hdr.cflags = 0; /* data block follows */
	mobitex_encode_header(&hdr, bits + pos);
	pos += MOBITEX_HEADER_BITS;

	/* Primary data block with Link Control */
	memset(block20, 0, sizeof(block20));
	memset(&lc, 0, sizeof(lc));
	lc.dest_man = dest_man;
	lc.frame_id = frame_type;
	lc.seq_num = 0;
	lc.block_length = 1;
	lc.bytes_last = 0;
	mobitex_build_link_control(&lc, block20);

	memcpy(data18, block20, MOBITEX_DATA_BYTES);
	mobitex_scrambler_reset(&mobitex->scrambler_tx_sr);
	mobitex_encode_block(mobitex, data18, block_bits);
	memcpy(bits + pos, block_bits, MOBITEX_BLOCK_BITS);
	pos += MOBITEX_BLOCK_BITS;

	/* Modulate and push to TX buffer */
	nsamples = fsk_encode_bits(mobitex, bits, pos);
	mobitex->fsk_tx_buffer_length = nsamples;
	mobitex->fsk_tx_buffer_pos = 0;

	LOGP_CHAN(DDSP, LOGL_INFO, "TX Access Request: %s (0x%02X) DestMAN=%06X, %d bits, %d samples.\n",
		  mobitex_frametype_name(frame_type), frame_type, dest_man, pos, nsamples);
}

/* Handle received access grant (ATD/ATL): dequeue pending message and transmit it. */
static void mobitex_rx_access_grant(mobitex_t *mobitex)
{
	mobitex_msg_t *msg;

	/* Clear retry state — access was granted */
	mobitex->tx_access_retries = 0;
	mobitex->tx_access_backoff = 0.0;

	/* Dequeue the first pending message */
	msg = mobitex->msg_list;
	if (!msg) {
		LOGP_CHAN(DDSP, LOGL_NOTICE, "Access grant received but no pending message.\n");
		return;
	}

	LOGP_CHAN(DDSP, LOGL_INFO, "Access granted, transmitting queued message (DestMAN=%06X).\n",
		  msg->dest_man);

	/* Transmit the message frame */
	mobitex_tx_frame(mobitex, msg);

	/* Remove from queue and free */
	mobitex->msg_list = msg->next;
	free(msg);
}

/* Handle received channel change (BKD/BKT): extract target TX/RX frequency pair. */
static void mobitex_rx_channel_change(mobitex_t *mobitex)
{
	uint16_t tx_channel, rx_channel;
	double tx_freq, rx_freq;
	int fbi;

	/* Extract channel numbers from primary data block payload (bytes 6-9).
	 * Bytes 6-7: target TX channel (big-endian)
	 * Bytes 8-9: target RX channel (big-endian) */
	tx_channel = ((uint8_t)mobitex->rx_data_blocks[0][6] << 8)
		   | (uint8_t)mobitex->rx_data_blocks[0][7];
	rx_channel = ((uint8_t)mobitex->rx_data_blocks[0][8] << 8)
		   | (uint8_t)mobitex->rx_data_blocks[0][9];

	/* Use FBI from last received sweep, default 0 */
	fbi = mobitex->sweep.fbi;

	tx_freq = mobitex_fbi_to_freq(fbi, tx_channel);
	rx_freq = mobitex_fbi_to_freq(fbi, rx_channel);

	LOGP_CHAN(DDSP, LOGL_INFO, "Channel change request: TXch=%d (%.4f MHz) RXch=%d (%.4f MHz) FBI=%d\n",
		  tx_channel, tx_freq, rx_channel, rx_freq, fbi);

	/* TODO: Actual frequency change would require sender reconfiguration.
	 * For now, log the request. A real implementation would call into the
	 * SDR/audio backend to retune TX and RX frequencies. */
}

/* Process a complete received frame. Called from dsp.c when all blocks are received. */
void mobitex_rx_frame_complete(mobitex_t *mobitex)
{
	uint8_t frame_id = mobitex->link_control.frame_id;
	const char *ftname = mobitex_frametype_name(frame_id);
	char base_id_str[5];

	/* Format Base ID as 4-char hex string (AreaID + BaseID) */
	snprintf(base_id_str, sizeof(base_id_str), "%02X%02X",
		 mobitex->frame_header.area_id, mobitex->frame_header.base_id);

	LOGP_CHAN(DDSP, LOGL_INFO, "RX Frame: %s (0x%02X) BaseID=%s NetID=%d CFlags=%d BlkLen=%d BytesLast=%d FEC=%d\n",
		  ftname, frame_id,
		  base_id_str,
		  mobitex->rx_frsync_index,
		  mobitex->frame_header.cflags,
		  mobitex->link_control.block_length,
		  mobitex->link_control.bytes_last,
		  mobitex->rx_fec_errors_total);

	switch (frame_id) {
	case MOBITEX_FT_MRM:
		mobitex_rx_process_mrm(mobitex);
		/* Base station mode: auto-ACK received MRM */
		if (mobitex->tx && !mobitex_is_mobile_mode(mobitex)) {
			LOGP_CHAN(DDSP, LOGL_INFO, "Base: sending ACK to MAN=%06X\n",
				  mobitex->link_control.dest_man);
			mobitex_tx_response(mobitex, MOBITEX_FT_ACK,
					    mobitex->link_control.dest_man);
		}
		break;
	case MOBITEX_FT_SVP:
		mobitex_rx_process_svp(mobitex);
		break;
	case MOBITEX_FT_ABD:
	case MOBITEX_FT_ABL:
		LOGP_CHAN(DDSP, LOGL_INFO, "Access request %s (0x%02X): DestMAN=%06X\n",
			  ftname, frame_id, mobitex->link_control.dest_man);
		mobitex_msg_receive(CHAN, base_id_str, frame_id,
				    mobitex->link_control.dest_man, 0, "");
		/* Base station mode: auto-grant access */
		if (mobitex->tx && !mobitex_is_mobile_mode(mobitex)) {
			uint8_t grant_type = (frame_id == MOBITEX_FT_ABD)
					   ? MOBITEX_FT_ATD : MOBITEX_FT_ATL;
			LOGP_CHAN(DDSP, LOGL_INFO, "Base: sending %s to MAN=%06X\n",
				  mobitex_frametype_name(grant_type),
				  mobitex->link_control.dest_man);
			mobitex_tx_response(mobitex, grant_type,
					    mobitex->link_control.dest_man);
		}
		break;
	case MOBITEX_FT_ACK:
	case MOBITEX_FT_NACK:
	case MOBITEX_FT_REB:
	case MOBITEX_FT_RES:
	case MOBITEX_FT_FRI:
	case MOBITEX_FT_TST:
	case MOBITEX_FT_AKT:
	case MOBITEX_FT_NAT:
	case MOBITEX_FT_BBT:
		LOGP_CHAN(DDSP, LOGL_INFO, "Frame type %s (0x%02X): DestMAN=%06X\n",
			  ftname, frame_id, mobitex->link_control.dest_man);
		mobitex_msg_receive(CHAN, base_id_str, frame_id,
				    mobitex->link_control.dest_man, 0, "");
		break;
	case MOBITEX_FT_ATD:
	case MOBITEX_FT_ATL:
		LOGP_CHAN(DDSP, LOGL_INFO, "Access grant %s (0x%02X): DestMAN=%06X\n",
			  ftname, frame_id, mobitex->link_control.dest_man);
		mobitex_msg_receive(CHAN, base_id_str, frame_id,
				    mobitex->link_control.dest_man, 0, "");
		if (mobitex_is_mobile_mode(mobitex))
			mobitex_rx_access_grant(mobitex);
		break;
	case MOBITEX_FT_BKD:
	case MOBITEX_FT_BKT:
		LOGP_CHAN(DDSP, LOGL_INFO, "Channel change %s (0x%02X): DestMAN=%06X\n",
			  ftname, frame_id, mobitex->link_control.dest_man);
		mobitex_msg_receive(CHAN, base_id_str, frame_id,
				    mobitex->link_control.dest_man, 0, "");
		mobitex_rx_channel_change(mobitex);
		break;
	default:
		LOGP_CHAN(DDSP, LOGL_NOTICE, "Unknown frame type 0x%02X\n", frame_id);
		break;
	}
}

/* Number of bitsync repetitions for TX preamble.
 * The Mobitex frame header is 56 bits total: 16 bitsync + 16 framesync + 24 control.
 * Real base stations transmit ROSI frames continuously back-to-back, so the
 * receiver's bit clock is always locked from the preceding frame's data.
 * We use 1 repetition (16 bits) which matches the spec exactly — the continuous
 * stream of filler frames keeps the clock locked between data-carrying frames. */
#define MOBITEX_TX_PREAMBLE_REPS  1

/* Emit bitsync preamble into bit buffer. Returns number of bits written. */
static int mobitex_emit_preamble(mobitex_t *mobitex, uint8_t *bits)
{
	int pos = 0, rep, i;
	uint16_t sync_word = mobitex->bitsync;

	for (rep = 0; rep < MOBITEX_TX_PREAMBLE_REPS; rep++) {
		for (i = 15; i >= 0; i--)
			bits[pos++] = (sync_word >> i) & 1;
	}
	return pos;
}

/* Generate a header-only ROSI filler frame into the TX buffer.
 * Real Mobitex base stations transmit continuously — between data-carrying
 * SVP/FRI/MRM frames, they send header-only frames (CFlags != 0) to keep
 * the mobile's bit clock locked via the bitsync pattern.
 * Without these fillers, the modem sees unmodulated carrier (no transitions)
 * and loses clock sync, causing "No Coverage" on the mobile. */
void mobitex_tx_filler(mobitex_t *mobitex)
{
	uint8_t bits[64]; /* bitsync(16) + framesync(16) + header(24) = 56 bits */
	mobitex_frame_header_t hdr;
	int pos, i, nsamples;
	uint16_t sync_word;
	int frsync_idx;

	/* Bitsync preamble */
	pos = mobitex_emit_preamble(mobitex, bits);

	/* Framesync */
	frsync_idx = mobitex->frsync_index;
	if (frsync_idx < 0 || frsync_idx >= 18)
		frsync_idx = 1; /* default: index 1 = 0xB433 (US RAM Mobile Data) */
	sync_word = mobitex_frsyncs[frsync_idx];
	for (i = 15; i >= 0; i--)
		bits[pos++] = (sync_word >> i) & 1;

	/* Header with CFlags != 0 — signals no data blocks follow */
	memset(&hdr, 0, sizeof(hdr));
	hdr.base_id = mobitex->tx_base_id;
	hdr.area_id = mobitex->tx_area_id;
	hdr.cflags = 0x0F;
	mobitex_encode_header(&hdr, bits + pos);
	pos += MOBITEX_HEADER_BITS;

	/* Modulate — preserve phase continuity from previous frame */
	nsamples = fsk_encode_bits(mobitex, bits, pos);
	mobitex->fsk_tx_buffer_length = nsamples;
	mobitex->fsk_tx_buffer_pos = 0;
}

/* Build and transmit an SVP (sweep) frame.
 * svp_type == 0: header-only SVP (CFlags != 0, no data blocks)
 * svp_type 1/3: FRI + link + roaming parameters + optional channel list
 * svp_type 2/4: channel change (UPFREQ/DOFREQ)
 * svp_type 6:   battery-saving parameters (PA07 command)
 *
 * Byte layout matches Palm VIIx ROM MobitexProcessSweepFrame. */
static void mobitex_tx_svp(mobitex_t *mobitex)
{
	uint8_t bits[600]; /* preamble(128) + framesync(16) + header(24) + block(240) */
	uint8_t block_bits[MOBITEX_BLOCK_BITS];
	mobitex_frame_header_t hdr;
	int pos, i, nsamples;
	uint16_t sync_word;
	int frsync_idx;

	/* Bitsync preamble: 8 repetitions for receiver acquisition */
	pos = mobitex_emit_preamble(mobitex, bits);

	/* Framesync: 16 bits MSB first */
	frsync_idx = mobitex->frsync_index;
	if (frsync_idx < 0 || frsync_idx >= 18)
		frsync_idx = 1; /* default: index 1 = 0xB433 (US RAM Mobile Data) */
	sync_word = mobitex_frsyncs[frsync_idx];
	for (i = 15; i >= 0; i--)
		bits[pos++] = (sync_word >> i) & 1;

	/* Frame header: 24 bits */
	memset(&hdr, 0, sizeof(hdr));
	hdr.base_id = mobitex->tx_base_id;
	hdr.area_id = mobitex->tx_area_id;

	if (mobitex->tx_svp_type == 0) {
		/* Header-only SVP: CFlags != 0 means no data blocks follow */
		hdr.cflags = 0x0F;
		mobitex_encode_header(&hdr, bits + pos);
		pos += MOBITEX_HEADER_BITS;
	} else {
		/* Full SVP with data block: CFlags == 0 means data blocks follow */
		uint8_t block20[20];
		uint8_t data18[MOBITEX_DATA_BYTES];
		mobitex_link_control_t lc;

		hdr.cflags = 0;
		mobitex_encode_header(&hdr, bits + pos);
		pos += MOBITEX_HEADER_BITS;

		/* Build primary data block */
		memset(block20, 0, sizeof(block20));

		/* Link Control: MAN = base station MAN, Frame ID = 0x0F (SVP), block_length = 1 */
		memset(&lc, 0, sizeof(lc));
		lc.dest_man = mobitex->tx_base_man;
		lc.frame_id = MOBITEX_FT_SVP;
		lc.seq_num = 0;
		lc.block_length = 1;
		lc.bytes_last = 0;
		mobitex_build_link_control(&lc, block20);

		/* Byte 6: sweep type */
		block20[6] = mobitex->tx_svp_type;

		/* Fill type-specific parameters */
		switch (mobitex->tx_svp_type) {
		case 1:
		case 3:
			/* SVP1/SVP3: FRI + link parameters in bytes 7-17 */
			block20[7]  = mobitex->tx_max_rep;
			block20[8]  = mobitex->tx_slot_length;
			block20[9]  = mobitex->tx_free_slots;
			block20[10] = mobitex->tx_rssi_period;
			block20[11] = mobitex->tx_timeout;
			block20[12] = 0; /* reserved */
			block20[13] = mobitex->tx_rand_slots;
			block20[14] = mobitex->tx_max_speech;
			block20[15] = mobitex->tx_max_access;
			block20[16] = mobitex->tx_txpow;
			block20[17] = mobitex->tx_prio;
			break;

		case 2:
		case 4:
			/* SVP2/SVP4: channel change command
			 * byte 7 = direction: 1 = switch to this channel
			 * bytes 10-11 = downlink channel number
			 * The mobile retunes its receiver to this channel and
			 * automatically derives the uplink via duplex offset. */
			block20[7]  = 1; /* direction: 1 = channel change */
			block20[10] = (mobitex->tx_dofreq >> 8) & 0xFF;
			block20[11] = mobitex->tx_dofreq & 0xFF;
			break;

		case 6:
			/* SVP6: battery-saving parameters (PA07 command) */
			block20[7]  = mobitex->tx_cycle_time;
			block20[8]  = mobitex->tx_time_to_next;
			block20[9]  = mobitex->tx_transaction_time;
			block20[10] = mobitex->tx_eval_current;
			block20[11] = mobitex->tx_eval_others;
			block20[12] = 0; /* MAN count (traffic list, not used in simple TX) */
			block20[13] = (mobitex->tx_deep_sleep_inhibit & 0x0F) << 4;
			break;

		default:
			/* SVP5 or unknown: just sweep type in byte 6 */
			break;
		}

		/* Encode primary block */
		memcpy(data18, block20, MOBITEX_DATA_BYTES);
		mobitex_scrambler_reset(&mobitex->scrambler_tx_sr);
		mobitex_encode_block(mobitex, data18, block_bits);
		memcpy(bits + pos, block_bits, MOBITEX_BLOCK_BITS);
		pos += MOBITEX_BLOCK_BITS;
	}

	/* Modulate and push to DSP TX buffer */
	nsamples = fsk_encode_bits(mobitex, bits, pos);
	mobitex->fsk_tx_buffer_length = nsamples;
	mobitex->fsk_tx_buffer_pos = 0;

	LOGP_CHAN(DDSP, LOGL_INFO, "TX SVP: type=%d, %d bits, %d samples.\n",
		  mobitex->tx_svp_type, pos, nsamples);
}

/* Build and transmit a FRI (free) frame.
 * FRI starts a free cycle, telling mobiles when they can transmit.
 * Primary block carries FRI parameters in bytes 6-13:
 *   byte 6:  SLOT_LENGTH    byte 10: MAX_ACCESS
 *   byte 7:  FREE_SLOTS     byte 11: MAX_SPEECH
 *   byte 8:  RAND_SLOTS     byte 12: TIMEOUT
 *   byte 9:  MAX_REP        byte 13: PRIO */
static void mobitex_tx_fri(mobitex_t *mobitex)
{
	uint8_t bits[500];
	uint8_t block_bits[MOBITEX_BLOCK_BITS];
	uint8_t block20[20];
	uint8_t data18[MOBITEX_DATA_BYTES];
	mobitex_frame_header_t hdr;
	mobitex_link_control_t lc;
	int pos, i, nsamples;
	uint16_t sync_word;
	int frsync_idx;

	pos = mobitex_emit_preamble(mobitex, bits);

	frsync_idx = mobitex->frsync_index;
	if (frsync_idx < 0 || frsync_idx >= 18)
		frsync_idx = 1; /* default: index 1 = 0xB433 (US RAM Mobile Data) */
	sync_word = mobitex_frsyncs[frsync_idx];
	for (i = 15; i >= 0; i--)
		bits[pos++] = (sync_word >> i) & 1;

	memset(&hdr, 0, sizeof(hdr));
	hdr.base_id = mobitex->tx_base_id;
	hdr.area_id = mobitex->tx_area_id;
	hdr.cflags = 0;
	mobitex_encode_header(&hdr, bits + pos);
	pos += MOBITEX_HEADER_BITS;

	memset(block20, 0, sizeof(block20));
	memset(&lc, 0, sizeof(lc));
	lc.dest_man = mobitex->tx_base_man;
	lc.frame_id = MOBITEX_FT_FRI;
	lc.seq_num = 0;
	lc.block_length = 1;
	lc.bytes_last = 0;
	mobitex_build_link_control(&lc, block20);

	/* FRI parameters */
	block20[6]  = mobitex->tx_slot_length;
	block20[7]  = mobitex->tx_free_slots;
	block20[8]  = mobitex->tx_rand_slots;
	block20[9]  = mobitex->tx_max_rep;
	block20[10] = mobitex->tx_max_access;
	block20[11] = mobitex->tx_max_speech;
	block20[12] = mobitex->tx_timeout;
	block20[13] = mobitex->tx_prio;

	memcpy(data18, block20, MOBITEX_DATA_BYTES);
	mobitex_scrambler_reset(&mobitex->scrambler_tx_sr);
	mobitex_encode_block(mobitex, data18, block_bits);
	memcpy(bits + pos, block_bits, MOBITEX_BLOCK_BITS);
	pos += MOBITEX_BLOCK_BITS;

	nsamples = fsk_encode_bits(mobitex, bits, pos);
	mobitex->fsk_tx_buffer_length = nsamples;
	mobitex->fsk_tx_buffer_pos = 0;

	LOGP_CHAN(DDSP, LOGL_INFO, "TX FRI: SlotLen=%d FreeSlots=%d RandSlots=%d MaxRep=%d, %d bits.\n",
		  mobitex->tx_slot_length, mobitex->tx_free_slots,
		  mobitex->tx_rand_slots, mobitex->tx_max_rep, pos);
}

/* Build and transmit a single-block base station response frame.
 * Used for ACK, NACK, ATD, ATL — all have the same structure:
 * primary block with link control only, frame_id identifies the type. */
static void mobitex_tx_response(mobitex_t *mobitex, uint8_t frame_type, uint32_t dest_man)
{
	uint8_t bits[500];
	uint8_t block_bits[MOBITEX_BLOCK_BITS];
	uint8_t block20[20];
	uint8_t data18[MOBITEX_DATA_BYTES];
	mobitex_frame_header_t hdr;
	mobitex_link_control_t lc;
	int pos, i, nsamples;
	uint16_t sync_word;
	int frsync_idx;

	pos = mobitex_emit_preamble(mobitex, bits);

	frsync_idx = mobitex->frsync_index;
	if (frsync_idx < 0 || frsync_idx >= 18)
		frsync_idx = 1; /* default: index 1 = 0xB433 (US RAM Mobile Data) */
	sync_word = mobitex_frsyncs[frsync_idx];
	for (i = 15; i >= 0; i--)
		bits[pos++] = (sync_word >> i) & 1;

	memset(&hdr, 0, sizeof(hdr));
	hdr.base_id = mobitex->tx_base_id;
	hdr.area_id = mobitex->tx_area_id;
	hdr.cflags = 0;
	mobitex_encode_header(&hdr, bits + pos);
	pos += MOBITEX_HEADER_BITS;

	memset(block20, 0, sizeof(block20));
	memset(&lc, 0, sizeof(lc));
	lc.dest_man = dest_man;
	lc.frame_id = frame_type;
	lc.seq_num = 0;
	lc.block_length = 1;
	lc.bytes_last = 0;
	mobitex_build_link_control(&lc, block20);

	memcpy(data18, block20, MOBITEX_DATA_BYTES);
	mobitex_scrambler_reset(&mobitex->scrambler_tx_sr);
	mobitex_encode_block(mobitex, data18, block_bits);
	memcpy(bits + pos, block_bits, MOBITEX_BLOCK_BITS);
	pos += MOBITEX_BLOCK_BITS;

	nsamples = fsk_encode_bits(mobitex, bits, pos);
	mobitex->fsk_tx_buffer_length = nsamples;
	mobitex->fsk_tx_buffer_pos = 0;

	LOGP_CHAN(DDSP, LOGL_INFO, "TX %s: DestMAN=%06X, %d bits, %d samples.\n",
		  mobitex_frametype_name(frame_type), dest_man, pos, nsamples);
}

/* Check SVP idle timer and transmit SVP frame when it expires.
 * Also handles mobile access request retries with random backoff.
 * Called from sender_send() when TX is enabled and the buffer is empty.
 * 'samples' is the number of audio samples in the current callback,
 * used to compute elapsed time for the countdown. */
void mobitex_tx_idle_tick(mobitex_t *mobitex, int samples)
{
	double elapsed;

	/* TX buffer still has data — don't interfere */
	if (mobitex->fsk_tx_buffer_length)
		return;

	elapsed = (double)samples / (double)mobitex->samplerate;

	/* Mobile mode: handle access request retries with random backoff */
	if (mobitex_is_mobile_mode(mobitex) && mobitex->msg_list) {
		/* Backoff timer active — count it down */
		if (mobitex->tx_access_backoff > 0) {
			mobitex->tx_access_backoff -= elapsed;
			if (mobitex->tx_access_backoff <= 0) {
				mobitex->tx_access_backoff = 0;
				/* Retry limit check */
				if (mobitex->tx_access_retries >= mobitex->tx_access_max_retries) {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Access request max retries (%d) reached, dropping message.\n",
						  mobitex->tx_access_max_retries);
					/* Drop the message */
					mobitex_msg_t *msg = mobitex->msg_list;
					mobitex->msg_list = msg->next;
					free(msg);
					mobitex->tx_access_retries = 0;
				} else {
					/* Retry the access request */
					uint8_t ft = MOBITEX_FT_ABD;
					if (mobitex->msg_list->frame_type == MOBITEX_FT_ABL)
						ft = MOBITEX_FT_ABL;
					LOGP_CHAN(DDSP, LOGL_INFO, "Access request retry %d/%d.\n",
						  mobitex->tx_access_retries + 1,
						  mobitex->tx_access_max_retries);
					mobitex_tx_access_request(mobitex, ft);
					mobitex->tx_access_retries++;
					/* Set backoff for next potential retry: random 0.1–1.0 seconds */
					mobitex->tx_access_backoff = 0.1 + ((double)(rand() % 900)) / 1000.0;
				}
			}
			return;
		}

		/* No backoff active and message queued — send initial access request */
		if (mobitex->tx_access_retries == 0) {
			uint8_t ft = MOBITEX_FT_ABD;
			if (mobitex->msg_list->frame_type == MOBITEX_FT_ABL)
				ft = MOBITEX_FT_ABL;
			mobitex_tx_access_request(mobitex, ft);
			mobitex->tx_access_retries++;
			/* Set backoff timer for ACK timeout / retry */
			mobitex->tx_access_backoff = 0.1 + ((double)(rand() % 900)) / 1000.0;
		}
		return;
	}

	/* Only auto-transmit SVP when enabled */
	if (!mobitex->tx_svp_auto)
		return;

	/* Message TX takes priority over SVP/FRI */
	if (mobitex->msg_list)
		return;

	/* Count down the FRI timer (base station mode only) */
	if (!mobitex_is_mobile_mode(mobitex) && mobitex->tx_fri_interval > 0) {
		mobitex->tx_fri_timer -= elapsed;
		if (mobitex->tx_fri_timer <= 0) {
			mobitex_tx_fri(mobitex);
			mobitex->tx_fri_timer = mobitex->tx_fri_interval;
			return; /* FRI sent this tick, SVP next time */
		}
	}

	/* Count down the SVP timer */
	mobitex->tx_svp_timer -= elapsed;

	if (mobitex->tx_svp_timer <= 0) {
		/* Rotate SVP types: SVP1 and SVP6 alternate so the mobile
		 * receives both link/roaming params and battery-saving params.
		 * MIS requires roam signal at least twice per second.
		 * Sequence: SVP1, SVP6, SVP1, SVP6, ...
		 * If SVP2/4 channel info is configured (upfreq or dofreq != 0),
		 * insert SVP2 every 4th cycle: SVP1, SVP6, SVP1, SVP2, ... */
		switch (mobitex->tx_svp_seq % 4) {
		case 0:
		case 2:
			mobitex->tx_svp_type = 1; /* SVP1: FRI + link + roaming */
			break;
		case 1:
			mobitex->tx_svp_type = 6; /* SVP6: battery-saving */
			break;
		case 3:
			if (mobitex->tx_upfreq != 0 || mobitex->tx_dofreq != 0)
				mobitex->tx_svp_type = 2; /* SVP2: channel info */
			else
				mobitex->tx_svp_type = 6; /* SVP6 again if no channel info */
			break;
		}
		mobitex->tx_svp_seq++;

		mobitex_tx_svp(mobitex);
		mobitex->tx_svp_timer = mobitex->tx_svp_interval;
	}
}

/* Build and transmit a complete Mobitex frame for the given message. */
static void mobitex_tx_frame(mobitex_t *mobitex, mobitex_msg_t *msg)
{
	uint8_t block20[20];
	uint8_t data18[MOBITEX_DATA_BYTES];
	uint8_t bits[5200]; /* max frame: 128+16+24+20*240 = 4968 bits */
	uint8_t block_bits[MOBITEX_BLOCK_BITS];
	mobitex_link_control_t lc;
	mobitex_mpak_header_t mpak;
	mobitex_frame_header_t hdr;
	int num_data_blocks, block_length, bytes_last;
	int pos, i, blk, n, nsamples;
	uint16_t sync_word;
	int frsync_idx;

	/* Calculate number of data blocks needed (excluding primary block).
	 * Primary block carries link control + MPAK header, not message data. */
	if (msg->data_length <= 0)
		num_data_blocks = 0;
	else
		num_data_blocks = (msg->data_length + MOBITEX_DATA_BYTES - 1) / MOBITEX_DATA_BYTES;

	/* block_length = primary block + data blocks */
	block_length = 1 + num_data_blocks;

	/* bytes_last: valid bytes in the last data block */
	if (num_data_blocks == 0)
		bytes_last = 0;
	else {
		bytes_last = msg->data_length % MOBITEX_DATA_BYTES;
		if (bytes_last == 0)
			bytes_last = MOBITEX_DATA_BYTES;
	}

	/* ---- Build primary block (block 0) ---- */
	memset(block20, 0, sizeof(block20));

	/* Link Control (bytes 0-5) */
	memset(&lc, 0, sizeof(lc));
	lc.dest_man = msg->dest_man;
	lc.frame_id = msg->frame_type;
	lc.seq_num = 0;
	lc.block_length = block_length;
	lc.bytes_last = bytes_last;
	mobitex_build_link_control(&lc, block20);

	/* MPAK header (bytes 6-17) for MRM frames */
	if (msg->frame_type == MOBITEX_FT_MRM) {
		memset(&mpak, 0, sizeof(mpak));
		mpak.sender_man = msg->sender_man;
		mpak.dest_man = msg->dest_man;
		mpak.mpak_type = msg->mpak_type;
		mpak.mpak_class = msg->mpak_class;
		mpak.hpid = 0;
		mobitex_build_mpak_header(&mpak, block20);
	}

	/* ---- Assemble frame bit stream ---- */

	/* Bitsync preamble: 8 repetitions for receiver acquisition */
	pos = mobitex_emit_preamble(mobitex, bits);

	/* Framesync: 16 bits MSB first */
	frsync_idx = mobitex->frsync_index;
	if (frsync_idx < 0 || frsync_idx >= 18)
		frsync_idx = 1; /* default: index 1 = 0xB433 (US RAM Mobile Data) */
	sync_word = mobitex_frsyncs[frsync_idx];
	for (i = 15; i >= 0; i--)
		bits[pos++] = (sync_word >> i) & 1;

	/* Frame header: 24 bits */
	memset(&hdr, 0, sizeof(hdr));
	hdr.base_id = mobitex->tx_base_id;
	hdr.area_id = mobitex->tx_area_id;
	hdr.cflags = 0; /* CFlags=0 means data blocks follow */
	mobitex_encode_header(&hdr, bits + pos);
	pos += MOBITEX_HEADER_BITS;

	/* Primary block (block 0): encode 18 data bytes from block20 */
	memcpy(data18, block20, MOBITEX_DATA_BYTES);
	mobitex_scrambler_reset(&mobitex->scrambler_tx_sr);
	mobitex_encode_block(mobitex, data18, block_bits);
	memcpy(bits + pos, block_bits, MOBITEX_BLOCK_BITS);
	pos += MOBITEX_BLOCK_BITS;

	/* Subsequent data blocks (message payload) */
	for (blk = 0; blk < num_data_blocks; blk++) {
		memset(data18, 0, MOBITEX_DATA_BYTES);

		/* Determine valid bytes in this block */
		if (blk == num_data_blocks - 1 && bytes_last > 0)
			n = bytes_last;
		else
			n = MOBITEX_DATA_BYTES;

		memcpy(data18, msg->data + blk * MOBITEX_DATA_BYTES, n);

		mobitex_encode_block(mobitex, data18, block_bits);
		memcpy(bits + pos, block_bits, MOBITEX_BLOCK_BITS);
		pos += MOBITEX_BLOCK_BITS;
	}

	/* ---- Modulate and push to DSP TX buffer ---- */
	nsamples = fsk_encode_bits(mobitex, bits, pos);
	mobitex->fsk_tx_buffer_length = nsamples;
	mobitex->fsk_tx_buffer_pos = 0;

	LOGP_CHAN(DDSP, LOGL_INFO, "TX frame: %d blocks, %d bits, %d samples.\n",
		  block_length, pos, nsamples);
}

void mobitex_msg_send(const char *text, int length)
{
	sender_t *sender;
	mobitex_t *mobitex = NULL;
	mobitex_msg_t *msg, **tailp;

	/* Find a TX-enabled mobitex instance */
	for (sender = sender_head; sender; sender = sender->next) {
		mobitex = (mobitex_t *)sender;
		if (mobitex->tx)
			break;
	}
	if (!sender) {
		LOGP(DDSP, LOGL_ERROR, "No TX-enabled Mobitex instance!\n");
		return;
	}

	/* Allocate and populate message */
	msg = calloc(1, sizeof(*msg));
	if (!msg) {
		LOGP(DDSP, LOGL_ERROR, "No memory for TX message!\n");
		return;
	}

	msg->mobitex = mobitex;
	msg->frame_type = MOBITEX_FT_MRM;
	msg->mpak_type = 0;
	msg->mpak_class = MPAK_CLASS_DTESERV;
	msg->dest_man = 0;
	msg->sender_man = 0;

	/* Copy message data */
	if (length > (int)sizeof(msg->data))
		length = (int)sizeof(msg->data);
	memcpy(msg->data, text, length);
	msg->data_length = length;

	/* Append to message queue */
	for (tailp = &mobitex->msg_list; *tailp; tailp = &(*tailp)->next)
		;
	*tailp = msg;

	/* Build and transmit the frame immediately */
	mobitex_tx_frame(mobitex, msg);
}

void mobitex_msg_receive(const char *channel, const char *base_id,
                         uint8_t frame_type, uint32_t dest_man,
                         uint32_t sender_man, const char *message)
{
	char text[6000];
	int pos;
	const char *ftname;

	/* Look up frame type name */
	if (frame_type < FRAMETYPE_NAMES_COUNT && frametype_names[frame_type])
		ftname = frametype_names[frame_type];
	else
		ftname = "Unknown";

	/* Format: channel, base_id, frame type, MANs, message */
	pos = snprintf(text, sizeof(text),
		       "@%s BaseID=%s %s DestMAN=%06X SenderMAN=%06X",
		       channel, base_id, ftname, dest_man, sender_man);

	/* Append message content if present */
	if (message && message[0]) {
		pos += snprintf(text + pos, sizeof(text) - pos, ": %s", message);
	}

	msg_receive(text);
}

/* ---- Required callbacks for libmobile ---- */

void dump_info(void) {}

void call_down_clock(void) {}

int call_down_setup(int __attribute__((unused)) callref, const char __attribute__((unused)) *caller_id, enum number_type __attribute__((unused)) caller_type, const char __attribute__((unused)) *dialing)
{
	return -CAUSE_NORMAL;
}

void call_down_answer(int __attribute__((unused)) callref, struct timeval __attribute__((unused)) *tv_meter) {}

void call_down_proceeding(int __attribute__((unused)) callref) {}

void call_down_disconnect(int callref, int cause)
{
	call_up_release(callref, cause);
}

void call_down_release(int __attribute__((unused)) callref, int __attribute__((unused)) cause) {}

void call_down_audio(void __attribute__((unused)) *decoder, void __attribute__((unused)) *decoder_priv, int __attribute__((unused)) callref, uint16_t __attribute__((unused)) sequence, uint8_t __attribute__((unused)) marker, uint32_t __attribute__((unused)) timestamp, uint32_t __attribute__((unused)) ssrc, uint8_t __attribute__((unused)) *payload, int __attribute__((unused)) payload_len) {}
