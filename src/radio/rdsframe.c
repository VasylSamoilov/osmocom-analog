/*
 * RDS Frame Encoding and Decoding
 *
 * (C) 2025-2026 by osmocom-analog authors
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rdsframe.h"
#include "rds.h"
#include "../liblogging/logging.h"

/* ============================================================
 * IEC 62106:2018 Section Reference Key
 * ============================================================
 * This module references the following IEC 62106 sections:
 *
 * Block Structure:
 *   Block structure  - S3.1.3   Block structure (PI, Group, TP, PTY in Block B)
 *   TP flag          - S3.2.1.2 Traffic Programme identification
 *   PTY codes        - S3.2.1.3 Programme Type codes (0-31)
 *   M/S switch       - S3.2.1.4 Music/Speech switching
 *   DI codes         - S3.2.1.5 Decoder Identification and Dynamic PTY
 *   TA flag          - S3.2.1.6 Traffic Announcement identification
 *
 * Group Types:
 *   Type 0 groups    - S6.1.5.1  Basic tuning and switching (PS, AF)
 *   Type 1 groups    - S6.1.5.2  Programme Item Number, slow labelling codes
 *   Type 2 groups    - S6.1.5.3  RadioText (RT), 64 or 32 characters
 *   Type 3A groups   - S6.1.5.19 Application ID for Open Data
 *   Type 4A groups   - S6.1.5.4  Clock-time and date (CT)
 *   Type 5 groups    - S6.1.5.5  Transparent Data Channels (TDC)
 *   Type 6 groups    - S6.1.5.6  In-House applications (IH)
 *   Type 7A groups   - S6.1.5.7  Radio Paging (RP)
 *   Type 8A groups   - S6.1.5.18 Traffic Message Channel (TMC)
 *   Type 9A groups   - S6.1.5.9  Emergency Warning System (EWS)
 *   Type 10A groups  - S6.1.5.8  Programme Type Name (PTYN)
 *   Type 13A groups  - S6.1.5.10 Enhanced Radio Paging (ERP)
 *   Type 14 groups   - S6.1.5.14 Enhanced Other Networks (EON)
 *   Type 15B groups  - S6.1.5.16 Fast switching information
 *
 * Tables and Annexes:
 *   Table 9          - Slow Labelling Code variants (ECC, TMC, Language)
 *   Annex B          - Check word calculation (CRC-10, syndrome)
 *   Annex G          - Modified Julian Date (MJD) conversion
 * ============================================================ */

/* ============================================================
 * RDS Group Definition Table (IEC 62106 Table 3)
 *
 * Provides human-readable information for each group type.
 * Similar to NMT nmt_frame[] table structure.
 * ============================================================ */

static const rds_group_def_t rds_group_table[] = {
	/* Type          Name    IEC Section                 Description */
	{ RDS_GROUP_0A,  "0A",   "S6.1.5.1 Type 0 (PS/AF)",  "Basic tuning and switching information" },
	{ RDS_GROUP_0B,  "0B",   "S6.1.5.1 Type 0 (PS/AF)",  "Basic tuning (PI in Block C)" },
	{ RDS_GROUP_1A,  "1A",   "S6.1.5.2 Type 1 (PIN/SLC)", "Programme Item Number + slow labeling codes" },
	{ RDS_GROUP_1B,  "1B",   "S6.1.5.2 Type 1 (PIN/SLC)", "Programme Item Number (PI in Block C)" },
	{ RDS_GROUP_2A,  "2A",   "S6.1.5.3 Type 2 (RT)",     "RadioText (64 characters)" },
	{ RDS_GROUP_2B,  "2B",   "S6.1.5.3 Type 2 (RT)",     "RadioText (32 characters, PI in Block C)" },
	{ RDS_GROUP_3A,  "3A",   "S6.1.5.19 Type 3A (ODA)",  "Application identification for ODA" },
	{ RDS_GROUP_3B,  "3B",   "ODA",                      "Open Data Application" },
	{ RDS_GROUP_4A,  "4A",   "S6.1.5.4 Type 4A (CT)",    "Clock-time and date" },
	{ RDS_GROUP_4B,  "4B",   "ODA",                      "Open Data Application" },
	{ RDS_GROUP_5A,  "5A",   "S6.1.5.5 Type 5 (TDC)",    "Transparent data channels" },
	{ RDS_GROUP_5B,  "5B",   "S6.1.5.5 Type 5 (TDC)",    "Transparent data channels" },
	{ RDS_GROUP_6A,  "6A",   "S6.1.5.6 Type 6 (IH)",     "In-house applications" },
	{ RDS_GROUP_6B,  "6B",   "S6.1.5.6 Type 6 (IH)",     "In-house applications" },
	{ RDS_GROUP_7A,  "7A",   "S6.1.5.7 Type 7A (RP)",    "Radio paging" },
	{ RDS_GROUP_7B,  "7B",   "ODA",                      "Open Data Application" },
	{ RDS_GROUP_8A,  "8A",   "S6.1.5.18 Type 8A (TMC)",  "Traffic Message Channel" },
	{ RDS_GROUP_8B,  "8B",   "ODA",                      "Open Data Application" },
	{ RDS_GROUP_9A,  "9A",   "S6.1.5.9 Type 9A (EWS)",   "Emergency Warning System" },
	{ RDS_GROUP_9B,  "9B",   "ODA",                      "Open Data Application" },
	{ RDS_GROUP_10A, "10A",  "S6.1.5.8 Type 10A (PTYN)", "Programme Type Name" },
	{ RDS_GROUP_10B, "10B",  "ODA",                      "Open Data Application" },
	{ RDS_GROUP_11A, "11A",  "ODA",                      "Open Data Application" },
	{ RDS_GROUP_11B, "11B",  "ODA",                      "Open Data Application" },
	{ RDS_GROUP_12A, "12A",  "ODA",                      "Open Data Application" },
	{ RDS_GROUP_12B, "12B",  "ODA",                      "Open Data Application" },
	{ RDS_GROUP_13A, "13A",  "S6.1.5.10 Type 13A (ERP)", "Enhanced Radio Paging" },
	{ RDS_GROUP_13B, "13B",  "ODA",                      "Open Data Application" },
	{ RDS_GROUP_14A, "14A",  "S6.1.5.14 Type 14 (EON)",  "Enhanced Other Networks" },
	{ RDS_GROUP_14B, "14B",  "S6.1.5.14 Type 14 (EON)",  "Enhanced Other Networks" },
	{ RDS_GROUP_15A, "15A",  "RBDS only",                "Long PS name (RBDS)" },
	{ RDS_GROUP_15B, "15B",  "S6.1.5.16 Type 15B",       "Fast switching information" },
	{ 0, NULL, NULL, NULL }  /* Terminator */
};

/* ============================================================
 * Block B Field Definitions (IEC 62106 S3.1.3)
 *
 * Common fields present in all group types.
 * Similar to AMPS amps_ie_desc[] structure.
 * ============================================================ */

/* Field decoder: Yes/No flag */
static const char *rds_field_yes_no(uint64_t value)
{
	return value ? "Yes" : "No";
}

/* Field decoder: Group type */
static const char *rds_field_group_type(uint64_t value)
{
	int type = (value >> 1) & 0x0F;
	int version = value & 1;
	static char result[8];
	snprintf(result, sizeof(result), "%d%c", type, version ? 'B' : 'A');
	return result;
}

/* Field decoder: Programme Type (uses rds_tables.c) */
static const char *rds_field_pty(uint64_t value)
{
	/* Returns pointer to static string from rds_tables.c */
	extern const char *rds_get_pty_name(uint8_t pty, int rbds);
	return rds_get_pty_name((uint8_t)(value & 0x1F), 0);
}

static const rds_field_def_t rds_block_b_common[] = {
	/* Name     Bits  Shift  Mask    IEC Ref                Description                  Decoder */
	{ "GROUP",  5,    11,    0xF800, "S3.1.3 Block struct", "Group type code (0A-15B)",  rds_field_group_type },
	{ "TP",     1,    10,    0x0400, "S3.2.1.2 TP flag",    "Traffic Programme",         rds_field_yes_no },
	{ "PTY",    5,    5,     0x03E0, "S3.2.1.3 PTY codes",  "Programme Type (0-31)",     rds_field_pty },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* ============================================================
 * Group-Specific Field Decoders
 * ============================================================ */

/* Field decoder: Music/Speech (IEC 62106 S3.2.1.4) */
static const char *rds_field_music_speech(uint64_t value)
{
	return value ? "Music" : "Speech";
}

/* Field decoder: Slow Labeling Code Variant (IEC 62106 Table 9) */
static const char *rds_field_slc_variant(uint64_t value)
{
	static const char *variants[] = {
		"ECC + Paging",      /* 0 */
		"TMC ID",            /* 1 */
		"Paging ID",         /* 2 */
		"Language",          /* 3 */
		"Reserved",          /* 4 */
		"Reserved",          /* 5 */
		"Broadcaster use",   /* 6 */
		"EWS channel"        /* 7 */
	};
	return variants[value & 0x07];
}

/* ============================================================
 * Group 0A/0B Block B Payload (IEC 62106 S6.1.5.1)
 * Basic tuning and switching information
 *
 * Block B bits 4-0:
 *   Bit 4:    TA (Traffic Announcement)
 *   Bit 3:    M/S (Music/Speech)
 *   Bit 2:    DI (Decoder Identification, meaning depends on segment)
 *   Bits 1-0: PS segment address (0-3)
 * ============================================================ */

static const rds_field_def_t rds_group_0a_fields[] = {
	/* Name   Bits  Shift  Mask    IEC Ref              Description                Decoder */
	{ "TA",   1,    4,     0x0010, "S3.2.1.6 TA flag",  "Traffic Announcement",    rds_field_yes_no },
	{ "M/S",  1,    3,     0x0008, "S3.2.1.4 M/S",      "Music/Speech",            rds_field_music_speech },
	{ "DI",   1,    2,     0x0004, "S3.2.1.5 DI",       "Decoder Identification",  NULL },
	{ "C1C0", 2,    0,     0x0003, "S6.1.5.1 Type 0",   "PS segment (0-3)",        NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* ============================================================
 * Group 1A Block B/C Fields (IEC 62106 S6.1.5.2)
 * Programme Item Number and slow labelling codes
 * ============================================================ */

/* Block B payload (bits 4-0) */
static const rds_field_def_t rds_group_1a_b_fields[] = {
	/* Name   Bits  Shift  Mask    IEC Ref              Description           Decoder */
	{ "RP",   5,    0,     0x001F, "S6.1.5.2 Type 1",   "Radio Paging codes", NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* Block C: Slow Labeling Codes */
static const rds_field_def_t rds_group_1a_c_fields[] = {
	/* Name      Bits  Shift  Mask    IEC Ref              Description             Decoder */
	{ "LA",      1,    15,    0x8000, "S6.1.5.2 Type 1",   "Linkage Actuator",     rds_field_yes_no },
	{ "VAR",     3,    12,    0x7000, "Table 9 SLC",       "Variant code (0-7)",   rds_field_slc_variant },
	{ "PAYLOAD", 12,   0,     0x0FFF, "S6.1.5.2 Type 1",   "Variant-specific data", NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* Block D: Programme Item Number (PIN) */
static const rds_field_def_t rds_group_1a_d_fields[] = {
	/* Name     Bits  Shift  Mask    IEC Ref              Description        Decoder */
	{ "DAY",    5,    11,    0xF800, "S6.1.5.2 Type 1",   "PIN day (1-31)",  NULL },
	{ "HOUR",   5,    6,     0x07C0, "S6.1.5.2 Type 1",   "PIN hour (0-24)", NULL },
	{ "MINUTE", 6,    0,     0x003F, "S6.1.5.2 Type 1",   "PIN minute (0-59)", NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* ============================================================
 * Group 2A/2B Block B Payload (IEC 62106 S6.1.5.3)
 * RadioText
 *
 * Block B bits 4-0:
 *   Bit 4:    A/B flag (text change indicator)
 *   Bits 3-0: RT segment address (0-15)
 * ============================================================ */

static const rds_field_def_t rds_group_2a_fields[] = {
	/* Name   Bits  Shift  Mask    IEC Ref              Description            Decoder */
	{ "A/B",  1,    4,     0x0010, "S6.1.5.3 Type 2",   "Text A/B flag",       NULL },
	{ "SEG",  4,    0,     0x000F, "S6.1.5.3 Type 2",   "RT segment (0-15)",   NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* ============================================================
 * Group 4A Fields (IEC 62106 S6.1.5.4)
 * Clock-Time and Date
 *
 * MJD spans Block B (2 bits) and Block C (15 bits)
 * Hour spans Block C (1 bit) and Block D (4 bits)
 * ============================================================ */

/* Block B payload (bits 1-0): MJD upper 2 bits */
static const rds_field_def_t rds_group_4a_b_fields[] = {
	/* Name    Bits  Shift  Mask    IEC Ref               Description      Decoder */
	{ "MJD_H", 2,    0,     0x0003, "S6.1.5.4 Type 4A",   "MJD bits 16-15", NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* Block C: MJD lower 15 bits + Hour bit 4 */
static const rds_field_def_t rds_group_4a_c_fields[] = {
	/* Name     Bits  Shift  Mask    IEC Ref               Description     Decoder */
	{ "MJD_L",  15,   1,     0xFFFE, "S6.1.5.4 Type 4A",   "MJD bits 14-0", NULL },
	{ "HOUR_H", 1,    0,     0x0001, "S6.1.5.4 Type 4A",   "Hour bit 4",   NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* Block D: Hour lower 4 bits + Minute + Timezone */
static const rds_field_def_t rds_group_4a_d_fields[] = {
	/* Name     Bits  Shift  Mask    IEC Ref               Description           Decoder */
	{ "HOUR_L", 4,    12,    0xF000, "S6.1.5.4 Type 4A",   "Hour bits 3-0",      NULL },
	{ "MIN",    6,    6,     0x0FC0, "S6.1.5.4 Type 4A",   "Minute (0-59)",      NULL },
	{ "TZ_SGN", 1,    5,     0x0020, "S6.1.5.4 Type 4A",   "Offset sign (0=+)",  NULL },
	{ "TZ_OFF", 5,    0,     0x001F, "S6.1.5.4 Type 4A",   "Offset half-hours",  NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* ============================================================
 * Group 10A Block B Payload (IEC 62106 S6.1.5.8)
 * Programme Type Name (PTYN)
 *
 * Block B bits 4-0:
 *   Bit 4:   A/B flag
 *   Bits 3-1: Reserved (0)
 *   Bit 0:   Segment (0=chars 1-4, 1=chars 5-8)
 * ============================================================ */

static const rds_field_def_t rds_group_10a_fields[] = {
	/* Name   Bits  Shift  Mask    IEC Ref                Description           Decoder */
	{ "A/B",  1,    4,     0x0010, "S6.1.5.8 Type 10A",   "PTYN A/B flag",      NULL },
	{ "SEG",  1,    0,     0x0001, "S6.1.5.8 Type 10A",   "PTYN segment (0-1)", NULL },
	{ NULL, 0, 0, 0, NULL, NULL, NULL }
};

/* ============================================================
 * Parity Check Matrix (IEC 62106 Annex B - Syndrome calc)
 *
 * Used for syndrome calculation in decoder.
 * Each entry corresponds to one bit position (25 down to 0).
 * ============================================================ */

static const uint32_t rds_parity_check_matrix[26] = {
	0x200,  /* bit 25 (MSB of data) */
	0x100,
	0x080,
	0x040,
	0x020,
	0x010,
	0x008,
	0x004,
	0x002,
	0x001,  /* bit 16 (LSB of data) */
	0x2DC,  /* bit 15 (MSB of checkword) - IEC 62106 Table B.1 */
	0x16E,
	0x0B7,
	0x287,
	0x39F,
	0x313,
	0x355,
	0x376,
	0x1BB,
	0x201,
	0x3DC,
	0x1EE,
	0x0F7,
	0x2A7,
	0x38F,
	0x31B   /* bit 0 (LSB of checkword) */
};

/* ============================================================
 * Function Implementations
 * ============================================================ */

/* Initialize frame tables - verify consistency */
int rds_frame_init(void)
{
	int i;

	/* Verify group table is properly terminated and ordered */
	for (i = 0; rds_group_table[i].name != NULL; i++) {
		if (rds_group_table[i].type != i) {
			LOGP(DFRAME, LOGL_ERROR, 
			     "RDS group table mismatch at index %d: expected type %d, got %d\n",
			     i, i, rds_group_table[i].type);
			return -1;
		}
	}

	LOGP(DFRAME, LOGL_DEBUG, "RDS frame tables initialized: %d group types\n", i);
	return 0;
}

/* Get group name from type code */
const char *rds_group_name(enum rds_group_type type)
{
	if (type < 0 || type > 0x1F)
		return "???";
	if (rds_group_table[type].name == NULL)
		return "???";
	return rds_group_table[type].name;
}

/* Get group description from type code */
const char *rds_group_description(enum rds_group_type type)
{
	if (type < 0 || type > 0x1F)
		return "Unknown group type";
	if (rds_group_table[type].description == NULL)
		return "Unknown group type";
	return rds_group_table[type].description;
}

/* Calculate CRC-10 for 16-bit data word
 * Generator polynomial: g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1
 * Binary: 10110111001 = 0x5B9
 * IEC 62106 Annex B */
uint16_t rds_crc_calc(uint16_t data)
{
	uint32_t reg = (uint32_t)data << 10;
	int i;

	/* Process 16 data bits */
	for (i = 0; i < 16; i++) {
		if (reg & 0x02000000) { /* Check MSB (bit 25) */
			reg ^= (RDS_CRC_POLY << 15);
		}
		reg <<= 1;
	}

	/* Remainder is in top 10 bits */
	return (reg >> 16) & RDS_CHECK_MASK;
}

/* Calculate syndrome for 26-bit block
 * IEC 62106 Annex B - used by decoder */
uint16_t rds_syndrome_calc(uint32_t block)
{
	uint16_t syndrome = 0;
	int k;

	for (k = 0; k < 26; k++) {
		if ((block >> (25 - k)) & 1) {
			syndrome ^= rds_parity_check_matrix[k];
		}
	}
	return syndrome;
}

/* Encode a 26-bit block with CRC and offset word
 * IEC 62106 SB.1 */
uint32_t rds_block_encode(uint16_t data, uint16_t offset_word, int debug)
{
	uint16_t crc = rds_crc_calc(data);
	uint16_t checkword = crc ^ offset_word;
	uint32_t block = ((uint32_t)data << 10) | checkword;

	if (debug) {
		LOGP(DFRAME, LOGL_DEBUG,
		     "  Block: data=%04X crc=%03X offset=%03X -> %07X\n",
		     data, crc, offset_word, block);
	}

	return block;
}

/* Pack 4 blocks (104 bits) into 13-byte group buffer
 * IEC 62106 S3.1.1 */
void rds_group_pack(uint32_t blocks[4], uint8_t *group, int debug)
{
	if (debug) {
		LOGP(DFRAME, LOGL_DEBUG,
		     "RDS Group: A=%07X B=%07X C=%07X D=%07X\n",
		     blocks[0], blocks[1], blocks[2], blocks[3]);
	}

	/* Pack 104 bits (4 x 26) into 13 bytes
	 * Block A: bits 103-78 -> group[0..3] (partial)
	 * Block B: bits 77-52  -> group[3..6] (partial)
	 * Block C: bits 51-26  -> group[6..9] (partial)
	 * Block D: bits 25-0   -> group[9..12] */

	group[0]  = (blocks[0] >> 18) & 0xFF;
	group[1]  = (blocks[0] >> 10) & 0xFF;
	group[2]  = (blocks[0] >> 2) & 0xFF;
	group[3]  = ((blocks[0] & 0x03) << 6) | ((blocks[1] >> 20) & 0x3F);
	group[4]  = (blocks[1] >> 12) & 0xFF;
	group[5]  = (blocks[1] >> 4) & 0xFF;
	group[6]  = ((blocks[1] & 0x0F) << 4) | ((blocks[2] >> 22) & 0x0F);
	group[7]  = (blocks[2] >> 14) & 0xFF;
	group[8]  = (blocks[2] >> 6) & 0xFF;
	group[9]  = ((blocks[2] & 0x3F) << 2) | ((blocks[3] >> 24) & 0x03);
	group[10] = (blocks[3] >> 16) & 0xFF;
	group[11] = (blocks[3] >> 8) & 0xFF;
	group[12] = blocks[3] & 0xFF;
}

/* ============================================================
 * Table-Driven Group Decoder (like NMT disassemble_frame)
 *
 * Uses field definition tables to extract and log fields.
 * ============================================================ */

/* Helper: extract field value using definition */
static inline uint16_t rds_field_extract(uint16_t block, const rds_field_def_t *field)
{
	return (block & field->mask) >> field->shift;
}

/* Helper: log fields from a definition table */
static void rds_log_fields(const rds_field_def_t *fields, uint16_t block,
                           int debug, int verbose)
{
	const rds_field_def_t *f;



	for (f = fields; f->name != NULL; f++) {
		uint16_t value = rds_field_extract(block, f);

		if (debug) {
			LOGP(DFRAME, LOGL_DEBUG, "  %s=%d", f->name, value);
			if (f->decoder)
				LOGP(DFRAME, LOGL_DEBUG, " (%s)", f->decoder(value));
			LOGP(DFRAME, LOGL_DEBUG, "\n");
		}

		if (verbose && f->decoder) {
			LOGP(DFRAME, LOGL_INFO, "  %s: %s = %s\n",
			     f->iec_ref, f->description, f->decoder(value));
		}
	}
}

/* Decode a group using field definition tables */
void rds_group_decode(const uint16_t blocks[4], const uint8_t block_status[4],
                      rds_frame_t *frame, int debug, int verbose)
{
	uint16_t b2 = blocks[1];
	int i;

	/* Initialize frame */
	memset(frame, 0, sizeof(*frame));
	for (i = 0; i < 4; i++) {
		frame->block_status[i] = block_status[i];
		frame->block_time[i] = time(NULL);
	}

	/* Block A: PI (always) */
	frame->pi = blocks[0];

	/* Block B: Common fields - extract using table */
	frame->group_type = (b2 >> 11) & 0x1F;  /* Bits 15-11 */
	frame->tp = (b2 >> 10) & 1;
	frame->pty = (b2 >> 5) & 0x1F;

	/* Log group header */
	if (debug || verbose) {
		const char *name = rds_group_name(frame->group_type);
		const char *desc = rds_group_description(frame->group_type);
		LOGP(DFRAME, LOGL_DEBUG, "RDS Decode: Group %s - %s\n", name, desc);
	}

	/* Log common Block B fields */
	if (debug)
		rds_log_fields(rds_block_b_common, b2, debug, 0);

	/* Group-specific decoding - use appropriate field tables */
	switch (frame->group_type) {
	case RDS_GROUP_0A:
	case RDS_GROUP_0B:
		/* Group 0: Basic tuning (PS, TA, M/S, DI) */
		frame->ta = (b2 >> 4) & 1;
		frame->ms = (b2 >> 3) & 1;
		frame->di = (b2 >> 2) & 1;
		frame->ps_segment = b2 & 0x03;

		if (debug)
			rds_log_fields(rds_group_0a_fields, b2, debug, 0);

		/* Block D: PS characters */
		if (block_status[3] != RDS_STATUS_ERROR) {
			frame->ps_chars[0] = (blocks[3] >> 8) & 0xFF;
			frame->ps_chars[1] = blocks[3] & 0xFF;
		}
		break;

	case RDS_GROUP_2A:
	case RDS_GROUP_2B:
		/* Group 2: RadioText */
		frame->rt_ab = (b2 >> 4) & 1;
		frame->rt_segment = b2 & 0x0F;

		if (debug)
			rds_log_fields(rds_group_2a_fields, b2, debug, 0);

		/* Block C: RT chars 0-1 (2A only) */
		if (frame->group_type == RDS_GROUP_2A && block_status[2] != RDS_STATUS_ERROR) {
			frame->rt_chars[0] = (blocks[2] >> 8) & 0xFF;
			frame->rt_chars[1] = blocks[2] & 0xFF;
		}
		/* Block D: RT chars 2-3 (or 0-1 for 2B) */
		if (block_status[3] != RDS_STATUS_ERROR) {
			frame->rt_chars[2] = (blocks[3] >> 8) & 0xFF;
			frame->rt_chars[3] = blocks[3] & 0xFF;
		}
		break;

	case RDS_GROUP_1A:
		/* Group 1A: PIN + Slow Labeling Codes */
		if (debug)
			rds_log_fields(rds_group_1a_b_fields, b2, debug, 0);

		/* Block C: Slow Labeling Codes */
		if (block_status[2] != RDS_STATUS_ERROR) {
			frame->la = (blocks[2] >> 15) & 1;
			frame->slc_variant = (blocks[2] >> 12) & 0x07;
			frame->slc_data = blocks[2] & 0x0FFF;
			if (debug)
				rds_log_fields(rds_group_1a_c_fields, blocks[2], debug, 0);
		}

		/* Block D: PIN */
		if (block_status[3] != RDS_STATUS_ERROR) {
			frame->pin = blocks[3];
			if (debug)
				rds_log_fields(rds_group_1a_d_fields, blocks[3], debug, 0);
		}
		break;

	case RDS_GROUP_4A:
		/* Group 4A: Clock-Time */
		if (debug)
			rds_log_fields(rds_group_4a_b_fields, b2, debug, 0);

		/* MJD spans Block B (2 bits) and Block C (15 bits) */
		frame->mjd = ((b2 & 0x03) << 15);
		if (block_status[2] != RDS_STATUS_ERROR) {
			frame->mjd |= (blocks[2] >> 1) & 0x7FFF;
			frame->hour = ((blocks[2] & 1) << 4);
			if (debug)
				rds_log_fields(rds_group_4a_c_fields, blocks[2], debug, 0);
		}

		/* Hour + Minute + TZ from Block D */
		if (block_status[3] != RDS_STATUS_ERROR) {
			frame->hour |= (blocks[3] >> 12) & 0x0F;
			frame->minute = (blocks[3] >> 6) & 0x3F;
			int tz_sign = (blocks[3] >> 5) & 1;
			int tz_off = blocks[3] & 0x1F;
			frame->tz_offset = tz_sign ? -tz_off : tz_off;
			if (debug)
				rds_log_fields(rds_group_4a_d_fields, blocks[3], debug, 0);
		}
		break;

	case RDS_GROUP_10A:
		/* Group 10A: PTYN */
		frame->ptyn_ab = (b2 >> 4) & 1;
		frame->ptyn_segment = b2 & 1;

		if (debug)
			rds_log_fields(rds_group_10a_fields, b2, debug, 0);

		/* Block C: PTYN chars 0-1 */
		if (block_status[2] != RDS_STATUS_ERROR) {
			frame->ptyn_chars[0] = (blocks[2] >> 8) & 0xFF;
			frame->ptyn_chars[1] = blocks[2] & 0xFF;
		}
		/* Block D: PTYN chars 2-3 */
		if (block_status[3] != RDS_STATUS_ERROR) {
			frame->ptyn_chars[2] = (blocks[3] >> 8) & 0xFF;
			frame->ptyn_chars[3] = blocks[3] & 0xFF;
		}
		break;

	default:
		/* Other groups: log as unknown for now */
		if (debug)
			LOGP(DFRAME, LOGL_DEBUG, "  (group type not fully decoded)\n");
		break;
	}
}

