/*
 * RDS Frame Encoding and Decoding
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * Frame-level encoding and decoding for RDS groups and blocks.
 * Implements IEC 62106:2018 / NRSC-4-B frame structure.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef _RDSFRAME_H
#define _RDSFRAME_H

#include <stdint.h>
#include <time.h>

/* ============================================================
 * RDS Group Types (IEC 62106 Table 3)
 *
 * Group type code = (type << 1) | version
 *   Type: 0-15
 *   Version: A=0, B=1
 *
 * Version A groups carry different data in Block C.
 * Version B groups repeat PI code in Block C (for mobile reception).
 * ============================================================ */

enum rds_group_type {
	RDS_GROUP_0A  = 0x00,	/* Basic tuning and switching (PS, AF) */
	RDS_GROUP_0B  = 0x01,	/* Basic tuning (PS only, PI in C) */
	RDS_GROUP_1A  = 0x02,	/* Programme Item Number + slow labeling */
	RDS_GROUP_1B  = 0x03,	/* Programme Item Number (PI in C) */
	RDS_GROUP_2A  = 0x04,	/* RadioText (64 chars) */
	RDS_GROUP_2B  = 0x05,	/* RadioText (32 chars, PI in C) */
	RDS_GROUP_3A  = 0x06,	/* Application identification (ODA) */
	RDS_GROUP_3B  = 0x07,	/* Open Data Application */
	RDS_GROUP_4A  = 0x08,	/* Clock-time and date */
	RDS_GROUP_4B  = 0x09,	/* Open Data Application */
	RDS_GROUP_5A  = 0x0A,	/* Transparent data channels */
	RDS_GROUP_5B  = 0x0B,	/* Transparent data channels */
	RDS_GROUP_6A  = 0x0C,	/* In-house applications */
	RDS_GROUP_6B  = 0x0D,	/* In-house applications */
	RDS_GROUP_7A  = 0x0E,	/* Radio paging */
	RDS_GROUP_7B  = 0x0F,	/* Open Data Application */
	RDS_GROUP_8A  = 0x10,	/* Traffic Message Channel (TMC) */
	RDS_GROUP_8B  = 0x11,	/* Open Data Application */
	RDS_GROUP_9A  = 0x12,	/* Emergency Warning System (EWS) */
	RDS_GROUP_9B  = 0x13,	/* Open Data Application */
	RDS_GROUP_10A = 0x14,	/* Programme Type Name (PTYN) */
	RDS_GROUP_10B = 0x15,	/* Open Data Application */
	RDS_GROUP_11A = 0x16,	/* Open Data Application */
	RDS_GROUP_11B = 0x17,	/* Open Data Application */
	RDS_GROUP_12A = 0x18,	/* Open Data Application */
	RDS_GROUP_12B = 0x19,	/* Open Data Application */
	RDS_GROUP_13A = 0x1A,	/* Enhanced Radio Paging */
	RDS_GROUP_13B = 0x1B,	/* Open Data Application */
	RDS_GROUP_14A = 0x1C,	/* Enhanced Other Networks (EON) */
	RDS_GROUP_14B = 0x1D,	/* Enhanced Other Networks (EON) */
	RDS_GROUP_15A = 0x1E,	/* Defined in RBDS only */
	RDS_GROUP_15B = 0x1F,	/* Fast switching information */
};

/* ============================================================
 * RDS Group Definition Table (IEC 62106 S6)
 *
 * Similar to NMT nmt_frame[] - provides metadata for each group type.
 * ============================================================ */

typedef struct rds_group_def {
	enum rds_group_type	type;		/* Group type code */
	const char		*name;		/* Short name (e.g. "0A") */
	const char		*iec_section;	/* IEC 62106 section reference */
	const char		*description;	/* Human-readable description */
} rds_group_def_t;

/* ============================================================
 * RDS Field Definition (IEC 62106 S3)
 *
 * Similar to AMPS def_ie - describes individual fields within blocks.
 * ============================================================ */

typedef struct rds_field_def {
	const char	*name;		/* Field name (e.g. "TP", "PTY") */
	int		bits;		/* Number of bits */
	int		shift;		/* Bit position (0 = LSB) */
	uint16_t	mask;		/* Bitmask for extraction */
	const char	*iec_ref;	/* IEC 62106 reference */
	const char	*description;	/* Human-readable description */
	const char	*(*decoder)(uint64_t value);  /* Value decoder function */
} rds_field_def_t;

/* ============================================================
 * RDS Decode Status (for decoder)
 *
 * Tracks the quality/reliability of each decoded field.
 * Used by decoder to indicate confidence level, and by display
 * routines to show status (e.g. color-coded output).
 * ============================================================ */

enum rds_decode_status {
	RDS_STATUS_NONE      = 0,	/* No data received yet */
	RDS_STATUS_VALID     = 1,	/* Valid, no errors detected */
	RDS_STATUS_CORRECTED = 2,	/* Valid, FEC corrected 1-2 bit errors */
	RDS_STATUS_ERROR     = 3,	/* Uncorrectable error, data unreliable */
};

/* ============================================================
 * RDS Frame (Decoded Group)
 *
 * Similar to NMT frame_t - holds decoded field values.
 *
 * Status tracking is per-block only. Field status is derived from
 * the source block:
 *   Block A (idx 0): PI
 *   Block B (idx 1): group type, TP, PTY, group-specific bits
 *   Block C (idx 2): AF, RT chars 1-2, CT data, ECC, etc.
 *   Block D (idx 3): PS chars, RT chars 3-4, PIN, etc.
 * ============================================================ */

typedef struct rds_frame {
	enum rds_group_type	group_type;	/* Decoded group type */
	uint16_t		pi;		/* Programme Identification (Block A) */

	/* Per-block decode status and timestamp (A=0, B=1, C=2, D=3)
	 * All fields derived from a block inherit its status/time */
	uint8_t			block_status[4];  /* enum rds_decode_status */
	time_t			block_time[4];    /* Unix timestamp (0=never received) */

	/* Block B common fields (IEC 62106 S3.1.3) - status from block_status[1] */
	uint8_t			tp;		/* Traffic Programme flag */
	uint8_t			pty;		/* Programme Type (0-31) */

	/* Group 0A/0B specific (IEC 62106 S6.1.5.1) */
	uint8_t			ta;		/* Traffic Announcement - from B */
	uint8_t			ms;		/* Music/Speech - from B */
	uint8_t			di;		/* Decoder Identification bit - from B */
	uint8_t			ps_segment;	/* PS segment (0-3) - from B */
	uint8_t			af1;		/* Alternative Frequency 1 - from C */
	uint8_t			af2;		/* Alternative Frequency 2 - from C */
	char			ps_chars[2];	/* PS characters - from D */

	/* Group 1A specific (IEC 62106 S6.1.5.2) */
	uint8_t			la;		/* Linkage Actuator - from C */
	uint8_t			slc_variant;	/* Slow Labeling Code variant - from C */
	uint16_t		slc_data;	/* Slow Labeling Code data - from C */
	uint16_t		pin;		/* Programme Item Number - from D */

	/* Group 2A/2B specific (IEC 62106 S6.1.5.3) */
	uint8_t			rt_ab;		/* RadioText A/B flag - from B */
	uint8_t			rt_segment;	/* RadioText segment - from B */
	char			rt_chars[4];	/* RT: chars 0-1 from C, 2-3 from D */

	/* Group 4A specific (IEC 62106 S6.1.5.4) */
	uint32_t		mjd;		/* Modified Julian Date - from B+C */
	uint8_t			hour;		/* Hour - from C+D */
	uint8_t			minute;		/* Minute - from D */
	int8_t			tz_offset;	/* Timezone offset - from D */

	/* Group 10A specific (IEC 62106 S6.1.5.8) */
	uint8_t			ptyn_ab;	/* PTYN A/B flag - from B */
	uint8_t			ptyn_segment;	/* PTYN segment - from B */
	char			ptyn_chars[4];	/* PTYN: chars 0-1 from C, 2-3 from D */

	/* Group 3A specific (IEC 62106 S6.1.5.5) - ODA Identification */
	uint8_t			app_group;	/* Application Group Type code - from B */
	uint16_t		aid;		/* Application Identification - from D */

	/* Group 14A/14B specific (IEC 62106 S6.1.5.14) - Enhanced Other Networks */
	uint8_t			eon_tp_on;	/* TP flag for Other Network - from B */
	uint8_t			eon_variant;	/* EON variant code (0-15) - from B */
	uint8_t			eon_ta_on;	/* TA flag for Other Network - from C (v13) */
	uint8_t			eon_pty_on;	/* PTY for Other Network - from C (v13) */
	uint16_t		eon_on_pi;	/* PI of Other Network - from D */
	char			eon_ps_chars[2]; /* PS chars for ON - from C (v0-3) */
	uint8_t			eon_af1;	/* AF1 for ON - from C (v4) */
	uint8_t			eon_af2;	/* AF2 for ON - from C (v4) */
	uint8_t			eon_la;		/* Linkage Actuator - from C (v12) */
	uint16_t		eon_lsn;	/* Linkage Set Number - from C (v12) */
	uint16_t		eon_pin;	/* PIN for ON - from C (v14) */
	uint16_t		eon_bcast;	/* Broadcaster data - from C (v15) */

	/* Group 7A specific (EN 50067 Annex M) - Radio Paging */
	uint8_t			paging_ab;	/* A/B flag - from B bit 4 */
	uint8_t			paging_psac;	/* PSAC value - from B bits 3-0 */
	uint16_t		paging_block_c;	/* Raw Block C data */
	uint16_t		paging_block_d;	/* Raw Block D data */

	/* Group 13A specific (EN 50067 Annex M) - Enhanced Radio Paging */
	uint8_t			erp_sty;	/* Sub-type - from B bits 4-2 */
	uint8_t			erp_cs;		/* Cycle Selection - from C bits 15-14 */
	uint8_t			erp_it;		/* Interval Number - from C bits 13-10 */
	uint32_t		erp_notify;	/* 25 notification bits - from C+D */
	uint8_t			erp_s1;		/* Sort indicator - from D bit 0 */

} rds_frame_t;

/* Status helper: convert status to display character */
static inline char rds_status_char(enum rds_decode_status status)
{
	switch (status) {
	case RDS_STATUS_VALID:     return '+';  /* Good */
	case RDS_STATUS_CORRECTED: return '?';  /* Corrected */
	case RDS_STATUS_ERROR:     return 'X';  /* Error */
	default:                   return '_';  /* No data */
	}
}

/* ============================================================
 * Function Prototypes
 * ============================================================ */

/* Initialize frame tables (call once at startup) */
int rds_frame_init(void);

/* Get group name from type code */
const char *rds_group_name(enum rds_group_type type);

/* Get group description from type code */
const char *rds_group_description(enum rds_group_type type);

/* Encode a 26-bit block with CRC and offset word
 * Returns: 26-bit block value
 * IEC 62106 SB.1 */
uint32_t rds_block_encode(uint16_t data, uint16_t offset_word, int debug);

/* Calculate CRC-10 for 16-bit data word
 * IEC 62106 Annex B */
uint16_t rds_crc_calc(uint16_t data);

/* Calculate syndrome for 26-bit block (decoder)
 * IEC 62106 Annex B */
uint16_t rds_syndrome_calc(uint32_t block);

/* Pack 4 blocks (104 bits) into 13-byte group buffer */
void rds_group_pack(uint32_t blocks[4], uint8_t *group, int debug);

/* Decode a group using field definition tables (like NMT disassemble_frame)
 * Extracts common and group-specific fields into rds_frame_t
 * @param blocks      4 decoded 16-bit data words (without checkwords)
 * @param block_status  Decode status for each block (RDS_STATUS_*)
 * @param frame       Output: populated frame structure
 * @param debug       Enable debug logging
 * @param verbose     Enable verbose educational logging */
void rds_group_decode(const uint16_t blocks[4], const uint8_t block_status[4],
                      rds_frame_t *frame, int debug, int verbose);

#endif /* _RDSFRAME_H */
