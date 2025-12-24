/*
 * RDS (Radio Data System) encoder and decoder
 *
 * (C) 2025-2026 by osmocom-analog authors
 * All Rights Reserved
 *
 * Implements IEC 62106 / NRSC-4-B RDS encoding and decoding
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef _RDS_H
#define _RDS_H

#include <stdint.h>
#include <time.h>
#include "../libsample/sample.h"
#include "rdsframe.h"  /* For rds_group_type, rds_decode_status enums */

/* ============================================================
 * RDS CONSTANTS - IEC 62106 / NRSC-4-B Specification
 * ============================================================
 * All values derived from IEC 62106-1:2018 and related standards.
 * ============================================================ */

/* ============================================================
 * Subcarrier and Pilot (IEC 62106 S2.2, ITU-R BS.450-4 S2.2.3)
 * ============================================================ */

/* Stereo pilot frequency (Hz) */
#define RDS_PILOT_FREQ          19000.0

/* Subcarrier = 3 x pilot frequency */
#define RDS_SUBCARRIER_MULT     3
#define RDS_SUBCARRIER          (RDS_PILOT_FREQ * RDS_SUBCARRIER_MULT)  /* 57000 Hz */

/* Phase tolerance: +/-10deg to pilot 3rd harmonic (0deg or +/-90deg lock allowed) */
#define RDS_PHASE_TOLERANCE_DEG 10.0

/* ============================================================
 * Data Rate (IEC 62106 S2.1)
 * ============================================================ */

/* Subcarrier / Data Rate ratio */
#define RDS_CLOCK_DIVISOR       48

/* Data rate = 57000 / 48 = 1187.5 bps */
#define RDS_BITRATE             (RDS_SUBCARRIER / RDS_CLOCK_DIVISOR)

/* Symbol period in seconds */
#define RDS_SYMBOL_PERIOD       (1.0 / RDS_BITRATE)  /* ~842 us */

/* ============================================================
 * Injection Levels (IEC 62106 S4.6, ITU-R BS.450-4 S2.2.3.2)
 *
 * Expressed as fraction of +/-75 kHz maximum FM deviation.
 * ============================================================ */

/* RDS injection range (IEC 62106 S4.6) */
#define RDS_INJECTION_MIN       (1.0 / 75.0)    /* +/-1.0 kHz = 1.3% */
#define RDS_INJECTION_MAX       (7.5 / 75.0)    /* +/-7.5 kHz = 10% */
#define RDS_INJECTION_OPTIMAL   (2.0 / 75.0)    /* +/-2.0 kHz = 2.7% (IEC recommended) */
#define RDS_INJECTION_NRSC      (5.0 / 75.0)    /* +/-5.0 kHz = 6.7% (NRSC-G300 practice) */

/* Default injection level (5% = +/-3.75 kHz) */
#define RDS_INJECTION           0.05

/* Pilot injection (ITU-R BS.450-4 S2.2.2.4) */
#define RDS_PILOT_INJECTION_MIN 0.08            /* 8% */
#define RDS_PILOT_INJECTION_MAX 0.10            /* 10% */

/* ============================================================
 * Block Structure (IEC 62106 S3.1)
 * ============================================================ */

#define RDS_BLOCK_BITS          26      /* Total bits per block */
#define RDS_DATA_BITS           16      /* Information word bits */
#define RDS_CHECK_BITS          10      /* Checkword bits */

#define RDS_BLOCKS_PER_GROUP    4       /* Blocks A, B, C/C', D */
#define RDS_GROUP_BITS          (RDS_BLOCK_BITS * RDS_BLOCKS_PER_GROUP)  /* 104 bits */
#define RDS_GROUP_BYTES         13      /* Bytes to store 104 bits */

/* Bit masks */
#define RDS_DATA_MASK           0xFFFF  /* 16-bit data mask */
#define RDS_CHECK_MASK          0x3FF   /* 10-bit checkword mask */
#define RDS_BLOCK_MASK          0x3FFFFFFUL  /* 26-bit block mask */

/* ============================================================
 * CRC-10 (IEC 62106 Annex B)
 *
 * Generator polynomial: g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1
 * Binary: 10110111001 = 0x5B9
 * ============================================================ */

#define RDS_CRC_POLY            0x5B9

/* ============================================================
 * Offset Words and Syndromes (EN 50067 / IEC 62106 Annex B)
 *
 * Encoder (IEC 62106 SB.1):
 *   checkword = crc(16-bit data) XOR offset_word
 *   block = (data << 10) | checkword  ->  26 bits transmitted
 *
 * Decoder (EN 50067 SB.2.1, Fig B.3):
 *   For each received 26-bit block, compute:
 *     syndrome = Sigma(bit[k] x H[k])  for k = 0..25
 *   where H is the parity check matrix derived from g(x).
 *
 *   A valid error-free block yields a syndrome that identifies
 *   which offset word was used (and thus which block A/B/C/D).
 *
 * EN 50067 Table B.1 (values from standard):
 *
 *   Block  Offset Word (binary)  Syndrome (binary)
 *   -----  --------------------  -----------------
 *   A      0011111100 (0x0FC)    1111011000 (0x3D8)
 *   B      0110011000 (0x198)    1111010100 (0x3D4)
 *   C      0101101000 (0x168)    1001011100 (0x25C)
 *   C'     1101010000 (0x350)    1111001100 (0x3CC)
 *   D      0110110100 (0x1B4)    1001011000 (0x258)
 *
 * See rdsframe.c:rds_parity_check_matrix[] for H values.
 * ============================================================ */

#define RDS_OFFSET_A            0x0FC
#define RDS_OFFSET_B            0x198
#define RDS_OFFSET_C            0x168
#define RDS_OFFSET_Cp           0x350
#define RDS_OFFSET_D            0x1B4

#define RDS_SYNDROME_A          0x3D8
#define RDS_SYNDROME_B          0x3D4
#define RDS_SYNDROME_C          0x25C
#define RDS_SYNDROME_Cp         0x3CC
#define RDS_SYNDROME_D          0x258

/* ============================================================
 * Alternative Frequencies (IEC 62106 S6.2.1.6.1, Table 11)
 * ============================================================ */

#define RDS_AF_NOT_USED         0x00    /* Code 0: Not to be used */
#define RDS_AF_FILLER           0xCD    /* Code 205: Filler code */
#define RDS_AF_NO_AF            0xE0    /* Code 224: No AF exists */
#define RDS_AF_NO_AF_PAIR       0xE0E0  /* 16-bit filler for Block C when no AF */

/* ============================================================
 * Pulse Shaping Filter (IEC 62106 S2.3, Figure 3)
 *
 * The overall data-channel spectrum has 100% cosine roll-off.
 * TX and RX each apply Square Root Raised Cosine (SRRC) filters.
 * ============================================================ */

#define RDS_FILTER_ROLLOFF      1.0     /* alpha = 1.0 (100% cosine roll-off) */
#define RDS_FILTER_SPAN_SYMBOLS 3       /* Filter spans 3 symbol periods */

/* Filter sample rate should give integer samples per symbol.
 * 228000 Hz = 192 x 1187.5 bps */
#define RDS_FILTER_SAMPLERATE   228000.0
#define RDS_SAMPLES_PER_SYMBOL  192     /* At 228 kHz sample rate */
#define RDS_FILTER_LENGTH       (RDS_FILTER_SPAN_SYMBOLS * RDS_SAMPLES_PER_SYMBOL)  /* 576 */

/* ============================================================
 * Group Types (IEC 62106 Table 3 / NRSC-4-B Table 3)
 *
 * Group type code = (type << 1) | version
 * Type: 0-15, Version: A=0, B=1
 * ============================================================ */

/* Group type enum defined in rdsframe.h (enum rds_group_type)
 * RDS_GROUP_0A = 0x00, RDS_GROUP_0B = 0x01, ..., RDS_GROUP_15B = 0x1F
 */

/* ============================================================
 * Block B: Common Fields (IEC 62106 S3.1.3)
 * All groups have the same Block B structure for bits 15-5
 *
 *   Bits 15-12: Group Type (0-15)
 *   Bit 11:     Version (0=A, 1=B)
 *   Bit 10:     TP (Traffic Program)
 *   Bits 9-5:   PTY (Programme Type, 0-31)
 *   Bits 4-0:   Group-specific (5 bits)
 * ============================================================ */

#define RDS_B2_TYPE_SHIFT       12
#define RDS_B2_GROUP_SHIFT      11      /* Shift for Group Type code (Type + Version) */
#define RDS_B2_TYPE_MASK        0xF000  /* Bits 15-12 */
#define RDS_B2_VERSION_BIT      11
#define RDS_B2_VERSION_MASK     0x0800  /* Bit 11 */
#define RDS_B2_TP_BIT           10
#define RDS_B2_TP_MASK          0x0400  /* Bit 10 */
#define RDS_B2_PTY_SHIFT        5
#define RDS_B2_PTY_MASK         0x03E0  /* Bits 9-5 */
#define RDS_B2_PAYLOAD_MASK     0x001F  /* Bits 4-0 (group-specific) */

/* ============================================================
 * Group 0A/0B Bit Fields (IEC 62106 S6.1.5.1)
 * Basic tuning and switching information
 *
 * Block B payload (bits 4-0):
 *   Bit 4:   TA (Traffic Announcement)
 *   Bit 3:   M/S (Music/Speech)
 *   Bit 2:   DI (Decoder Information, depends on segment)
 *   Bits 1-0: PS segment address (0-3)
 *
 * Block C (0A): AF1 (8 bits) + AF2 (8 bits)
 * Block C (0B): PI repeat
 * Block D: PS characters (2 per group)
 * ============================================================ */

#define RDS_0A_TA_BIT           4
#define RDS_0A_TA_MASK          0x0010
#define RDS_0A_MS_BIT           3
#define RDS_0A_MS_MASK          0x0008
#define RDS_0A_DI_BIT           2
#define RDS_0A_DI_MASK          0x0004
#define RDS_0A_SEG_MASK         0x0003  /* Bits 1-0: PS segment (0-3) */

/* Block C: Alternative Frequencies */
#define RDS_0A_AF1_SHIFT        8
#define RDS_0A_AF1_MASK         0xFF00
#define RDS_0A_AF2_MASK         0x00FF

/* ============================================================
 * Group 2A/2B Bit Fields (IEC 62106 S6.1.5.3)
 * RadioText
 *
 * Block B payload (bits 4-0):
 *   Bit 4:    A/B flag (text change indicator)
 *   Bits 3-0: RT segment address (0-15 for 2A, 0-15 for 2B)
 *
 * Block C (2A): RT chars [seg*4] and [seg*4+1]
 * Block C (2B): PI repeat
 * Block D: RT chars [seg*4+2] and [seg*4+3] for 2A
 *          RT chars [seg*2] and [seg*2+1] for 2B
 * ============================================================ */

#define RDS_2A_AB_BIT           4
#define RDS_2A_AB_MASK          0x0010
#define RDS_2A_SEG_MASK         0x000F  /* Bits 3-0: RT segment (0-15) */

/* ============================================================
 * Group 1A/1B Bit Fields (IEC 62106 S6.1.5.2)
 * Programme Item Number and slow labelling codes
 *
 * Block C (Group 1A only):
 *   Bit 15:    LA (Linkage Actuator) - indicates linkage information
 *   Bits 14-12: Variant code (0-7) for slow labelling
 *   Bits 11-0:  Variant-specific payload (12 bits)
 *
 * Block D (both 1A and 1B):
 *   Bits 15-11: PIN day (1-31, 0 = not used)
 *   Bits 10-6:  PIN hour (0-24, 24 = end time)
 *   Bits 5-0:   PIN minute (0-59)
 * ============================================================ */

/* Block C: Linkage Actuator and Variant */
#define RDS_1A_LA_BIT           15
#define RDS_1A_LA_MASK          0x8000
#define RDS_1A_VARIANT_SHIFT    12
#define RDS_1A_VARIANT_MASK     0x7000  /* Bits 14-12 */
#define RDS_1A_PAYLOAD_MASK     0x0FFF  /* Bits 11-0 */

/* Slow Labelling Variant Codes (IEC 62106 Table 9) */
#define RDS_1A_VARIANT_ECC      0       /* Extended Country Code (+ paging) */
#define RDS_1A_VARIANT_TMC_ID   1       /* TMC identification */
#define RDS_1A_VARIANT_PAGER    2       /* Paging (deprecated) */
#define RDS_1A_VARIANT_LANGUAGE 3       /* Language codes */
#define RDS_1A_VARIANT_BCAST    6       /* Broadcaster use */
#define RDS_1A_VARIANT_EWS      7       /* EWS channel number */

/* Block D: Programme Item Number (PIN) */
#define RDS_PIN_DAY_SHIFT       11
#define RDS_PIN_DAY_MASK        0xF800  /* Bits 15-11 (5 bits) */
#define RDS_PIN_HOUR_SHIFT      6
#define RDS_PIN_HOUR_MASK       0x07C0  /* Bits 10-6 (5 bits) */
#define RDS_PIN_MINUTE_MASK     0x003F  /* Bits 5-0 (6 bits) */

/* ============================================================
 * Group 4A Bit Fields (IEC 62106 S6.1.5.4)
 * Clock-Time and Date
 *
 * The time is transmitted using Modified Julian Date (MJD) which spans
 * blocks B and C, plus hour/minute and timezone offset in blocks C and D.
 *
 * Block B (bits 1-0): MJD bits 16-15 (uppermost 2 bits)
 * Block C (bits 15-1): MJD bits 14-0 (lower 15 bits)
 * Block C (bit 0) + Block D (bits 15-12): Hour (5 bits, 0-23)
 * Block D (bits 11-6): Minute (6 bits, 0-59)
 * Block D (bit 5): Timezone sign (0=+, 1=-)
 * Block D (bits 4-0): Timezone offset in half-hours (0-24)
 * ============================================================ */

/* MJD extraction - spans B2 and B3 */
#define RDS_4A_MJD_B2_MASK      0x0003  /* Lower 2 bits of B2 */
#define RDS_4A_MJD_B3_SHIFT     1       /* B3 bits 15-1 */
#define RDS_4A_MJD_B3_MASK      0xFFFE  /* Bits 15-1 of B3 */

/* Hour extraction - spans B3 and B4 */
#define RDS_4A_HOUR_B3_BIT      0       /* Bit 0 of B3 (hour bit 4) */
#define RDS_4A_HOUR_B4_SHIFT    12      /* Bits 15-12 of B4 (hour bits 3-0) */
#define RDS_4A_HOUR_B4_MASK     0xF000

/* Minute and timezone - in B4 */
#define RDS_4A_MINUTE_SHIFT     6
#define RDS_4A_MINUTE_MASK      0x0FC0  /* Bits 11-6 */
#define RDS_4A_TZ_SIGN_BIT      5       /* Bit 5: 0=positive, 1=negative */
#define RDS_4A_TZ_SIGN_MASK     0x0020
#define RDS_4A_TZ_OFFSET_MASK   0x001F  /* Bits 4-0: half-hour offset (0-24) */

/* ============================================================
 * Group 10A Bit Fields (IEC 62106 S6.1.5.8)
 * Programme Type Name (PTYN)
 *
 * Block B: [4]AB_FLAG
 * Block C: [0]Segment (bit 0 of address)
 * Block D: 4 characters of PTYN (requires 2 groups for 8 chars)
 * Wait! Group 10A is different from 2A (RadioText).
 *
 * Structure:
 * Block B: [4]AB_FLAG (Toggles)
 * Block B: [3:0] Address (only bit 0 used for segment?)
 * NO! Group 10A uses Block B bits 0-4 for A/B flag and segment?
 * Let's check the spec/reference.
 *
 * Reference check (Standard Implementation):
 * Block B: PTY (5 bits), TP (1 bit), PTY (5 bits) - Standard Block B
 * Bits 4-0 of Block B in Group 10A are:
 *  - Bit 4: A/B flag (if 1, PTYN is being updated/changed?)
 *  - Bits 3-1: Reserved/Spare (set to 0)
 *  - Bit 0: A/B segment address (0 = chars 1-4, 1 = chars 5-8)
 *
 * PTYN is 8 characters long.
 * Block C: Chars 1-2 (or 5-6)
 * Block D: Chars 3-4 (or 7-8)
 * NOTE: PTYN is rarely used.
 * ============================================================ */
#define RDS_10A_AB_FLAG_BIT     4
#define RDS_10A_SEGMENT_BIT     0       /* 0=First 4 chars, 1=Last 4 chars */

/* ============================================================
 * Group 14A/14B Bit Fields (IEC 62106 S6.1.5.14)
 * Enhanced Other Networks (EON)
 *
 * Block B:
 *   Contains PTY and TP for the *current* network (bits 15-5).
 *   Bits 4: Reserved (Group 14BTA) or Usage Code (14A)
 *   Bits 3-0: Usage Code (Group 14A)
 *
 * Group 14A Usage Codes (Block B bits 3-0):
 *   0-3:  PS Name character pairs for ON (Other Network)
 *   4:    AFs for ON (Method B - mapped frequency)
 *   5-9:  Mapped AFs
 *   12:   Linkage/E-Linkage information
 *   13:   PTY of ON + TA of ON
 *   14:   PIN of ON
 *   15:   Reserved
 *
 * Block C: AFs or other data for ON (depends on usage code)
 * Block D: ON-PI (PI code of the Other Network)
 *
 * Group 14B:
 *   Transmits TA flags for multiple other networks.
 *   Block D: ON-PI.
 * ============================================================ */
#define RDS_14A_USAGE_MASK      0x000F  /* Block B bits 3-0 */
#define RDS_14A_TP_ON_BIT       4       /* Group 14B only: TA for ON */
#define RDS_14B_TA_ON_BIT       3       /* Group 14B: bit 3 of Block B? No, Block C... check spec */
/* 
 * Correction: 
 * Group 14A: Block B bits 3-0 used for variant.
 * Group 14B: Block B bit 4 is usually 0. Block C contains TA flags? 
 * Actually, 14B is rarely used compared to 14A variants.
 * 
 * 14A Variant 13 (0xD):
 *   Block C bits 15-11: ON-PTY
 *   Block C bit 0:      ON-TA (Traffic Announcement)
 *   Block C bit 4:      ON-TP (Traffic Programme)
 */
#define RDS_14A_VARIANT_PS_0    0       /* chars 1-2 */
#define RDS_14A_VARIANT_PS_1    1       /* chars 3-4 */
#define RDS_14A_VARIANT_PS_2    2       /* chars 5-6 */
#define RDS_14A_VARIANT_PS_3    3       /* chars 7-8 */
#define RDS_14A_VARIANT_AF      4       /* Alternative Freqs */
#define RDS_14A_VARIANT_INFO    13      /* PTY, TA, TP for ON */
#define RDS_14A_VARIANT_PIN     14      /* PIN for ON */


/* ============================================================
 * Text Field Sizes (IEC 62106 S7.12, S6.1.5.3)
 * ============================================================ */

#define RDS_PS_LENGTH           8       /* Program Service name: 8 chars */
#define RDS_RT_LENGTH_A         64      /* RadioText 2A: 64 chars max */
#define RDS_RT_LENGTH_B         32      /* RadioText 2B: 32 chars max */
#define RDS_PTYN_LENGTH         8       /* Program Type Name: 8 chars */

/* ============================================================
 * Program Type (PTY) Codes (IEC 62106 Annex F / NRSC-4-B Table F.2)
 * ============================================================ */

#define RDS_PTY_NONE            0       /* No PTY or undefined */
#define RDS_PTY_NEWS            1       /* News */
#define RDS_PTY_AFFAIRS         2       /* Current Affairs / Information */
#define RDS_PTY_INFO            2       /* RBDS: Information */
#define RDS_PTY_SPORT           3       /* Sports */
#define RDS_PTY_ALARM           31      /* Alarm / Emergency */

/* ============================================================
 * Special Control Codes
 * ============================================================ */

/* RadioText terminator (IEC 62106 S6.1.5.3) */
#define RDS_RT_TERMINATOR       0x0D    /* Carriage return */
#define RDS_RT_NEWLINE          0x0A    /* Line feed (preferred break) */

/* ============================================================
 * RDS2 SPECIFICATION (IEC 62106-2:2021)
 * ============================================================
 *
 * RDS2 is the next-generation evolution of RDS, standardized in
 * IEC 62106:2018 with updates in IEC 62106-2:2021. It maintains
 * full backward compatibility with legacy RDS receivers.
 *
 * KEY ENHANCEMENTS:
 * -----------------
 * 1. Up to 4x data capacity (4750 bps vs 1187.5 bps)
 * 2. UTF-8 character support (full Unicode)
 * 3. Extended RadioText (eRT): 128 bytes vs 64 chars
 * 4. Long PS names via Group 15A (UTF-8)
 * 5. File transfer up to 163 KB (RDS2 File Transfer / RFT)
 * 6. IPv6 address transmission for hybrid FM/IP radio
 * 7. Extended AF coding down to 64.1 MHz (OIRT band)
 *
 * ADDITIONAL SUBCARRIERS (IEC 62106-2 S4):
 * ----------------------------------------
 * RDS2 introduces 3 additional BPSK subcarriers:
 *
 *   Subcarrier   Frequency    Notes
 *   ---------    ---------    -----
 *   Stream 1     57.0 kHz     Original RDS (3 x 19 kHz pilot)
 *   Stream 2     66.5 kHz     RDS2 additional stream
 *   Stream 3     71.25 kHz    RDS2 additional stream
 *   Stream 4     76.0 kHz     RDS2 additional stream (4 x 19 kHz)
 *
 * NOTE: Only streams 1 and 4 are exact harmonics of pilot!
 *       Streams 2 and 3 require independent oscillators.
 *
 * FM MULTIPLEX SPECTRUM WITH RDS2:
 * --------------------------------
 *   0-15 kHz     : Mono audio (L+R)
 *   19 kHz       : Pilot tone (+/-10%)
 *   23-53 kHz    : Stereo difference (L-R) on 38 kHz subcarrier
 *   57 kHz       : RDS stream 1 (original, +/-2-5 kHz deviation)
 *   66.5 kHz     : RDS2 stream 2
 *   71.25 kHz    : RDS2 stream 3
 *   76 kHz       : RDS2 stream 4
 *
 * GROUP TYPE C (RDS2-specific):
 * -----------------------------
 * Data on additional subcarriers uses new "Group Type C" structure.
 * Unlike classic A/B groups, Type C is optimized for streaming data.
 *
 * REAL-WORLD ADOPTION (as of 2024):
 * ---------------------------------
 * - Very limited deployment (pilot projects in DE, FR)
 * - Requires new encoder AND receiver hardware
 * - Most car radios are still RDS-only
 * - HD Radio (US) competes for same use cases
 * ============================================================ */

/* RDS2 Subcarrier frequencies (Hz) */
#define RDS2_STREAM1_FREQ       57000.0   /* Original RDS (3 x pilot) */
#define RDS2_STREAM2_FREQ       66500.0   /* Additional stream */
#define RDS2_STREAM3_FREQ       71250.0   /* Additional stream */
#define RDS2_STREAM4_FREQ       76000.0   /* Additional stream (4 x pilot) */

/* RDS2 bandwidth requirements */
#define RDS2_BANDWIDTH_MAX      80000.0   /* Upper edge at 76 kHz + margin */

/* RDS2 Extended RadioText */
#define RDS2_ERT_LENGTH         128       /* Extended RadioText: 128 bytes UTF-8 */

/* RDS2 File Transfer limits */
#define RDS2_RFT_MAX_SIZE       (163 * 1024)  /* 163 KB max file size */

/*
 * TODO: RDS2 Implementation Checklist
 * ------------------------------------
 * [ ] Add BPSK modulators for streams 2-4 (66.5, 71.25, 76 kHz)
 * [ ] Implement Group Type C encoder
 * [ ] Add UTF-8 RadioText/PS support
 * [ ] Implement RDS2 File Transfer (RFT) protocol
 * [ ] Add Extended AF coding for OIRT band (64.1-87.5 MHz)
 * [ ] Phase-lock streams 1 & 4 to pilot, free-run 2 & 3
 */

/* ============================================================
 * RDS ENCODER
 * ============================================================ */

/* Forward declaration */
struct rds_decoder;

typedef struct rds_encoder {
	/* Configuration */
	uint16_t	pi;		/* Program Identification */
	uint8_t		pty;		/* Program Type (0-31) */
	uint8_t		tp;		/* Traffic Program flag */
	uint8_t		ta;		/* Traffic Announcement flag */
	uint8_t		ms;		/* Music/Speech flag */
	char		ps[9];		/* Program Service name (8 chars + NUL) */
	char		rt[65];		/* RadioText (64 chars + NUL) */
	uint8_t		rt_ab;		/* RadioText A/B flag */
	
	/* Extended Configuration (Phase 2) */
	uint8_t		ecc;		/* Extended Country Code */
	uint8_t		language;	/* Language Code */
	char		ptyn[9];	/* Program Type Name (8 chars + NUL) */
	uint8_t		ptyn_ab;	/* PTYN A/B flag */
	int		ct_enabled;	/* Clock-Time transmission enabled */
	time_t		ct_time_offset;	/* Offset from system time for CT (seconds) */
	int8_t		local_offset;	/* Local Time Offset from UTC (half-hours) */
	uint8_t		pin_day;	/* PIN: Day (0=none) */
	uint8_t		pin_hour;	/* PIN: Hour */
	uint8_t		pin_minute;	/* PIN: Minute */

	/* Encoder state */
	double		samplerate;
	double		phase;		/* subcarrier phase */
	double		phasestep;	/* phase increment per sample */
	double		bit_phase;	/* bit timing phase */
	double		bit_phasestep;	/* bit timing increment */
	int		last_diff_bit;	/* previous differentially encoded bit */
	int		bit_history[3];	/* history of diff encoded bits (+1/-1) */
	float		*waveform_biphase; /* Runtime-generated RRC biphase waveform */

	/* Group generation state */
	uint8_t		group_buffer[13]; /* current group (104 bits) */
	int		group_bit_pos;	/* position in current group */
	int		ps_segment;	/* current PS segment (0-3) */
	int		rt_segment;	/* current RT segment (0-15) */
	int		ptyn_segment;	/* current PTYN segment (0-1) */
	int		group_count;	/* (legacy) for group rotation */
	uint64_t	group_sequence;	/* Total groups generated */
	time_t		last_ct_minute;	/* Timestamp of last CT injection */
	
} rds_encoder_t;

/* Initialize RDS encoder */
int rds_encoder_init(rds_encoder_t *rds, double samplerate, uint16_t pi, 
		     const char *ps, const char *rt, uint8_t pty, const char *ptyn);

/* Generate RDS samples to add to FM baseband (phase-locked to pilot) */
void rds_encoder_process(rds_encoder_t *rds, sample_t *samples, int num,
			 double pilot_phase, double pilot_phasestep);

/* Set RadioText (can be changed dynamically) */
void rds_set_radiotext(rds_encoder_t *rds, const char *rt);

/* Set Traffic Announcement flag */
void rds_set_ta(rds_encoder_t *rds, int ta);

/* Cleanup */
void rds_encoder_exit(rds_encoder_t *rds);

/* ============================================================
 * RDS DECODER
 * ============================================================ */

/* Simple IIR filter state */
typedef struct {
	double a[6];	/* Feedback coefficients (a[0]=1.0) */
	double b[6];	/* Feedforward coefficients */
	double x[6];	/* Input history */
	double y[6];	/* Output history */
	int order;
} rds_iir_filter_t;

typedef struct rds_decoder {
	/* Configuration */
	double		samplerate;	/* sample rate of signal */
	int		debug;		/* debug logging */
	int		verbose;	/* verbose logging (human-readable) */
	
	/* Tunable State */
	double		freq_subcarrier;	/* Current locked frequency (~57000 Hz) */
	double		phase_subcarrier;	/* Current subcarrier phase (0..2pi) */
	
	/* PLL */
	rds_iir_filter_t	filter_pll;	/* Loop filter for PLL */
	rds_iir_filter_t	filter_2400_i;	/* 2.4kHz LPF for I */
	rds_iir_filter_t	filter_2400_q;	/* 2.4kHz LPF for Q */
	
	/* Clock Recovery (Subcarrier-locked) */
	double		clock_offset;		/* Phase offset for 1187.5 Hz clock */
	int		prev_clock_bit;		/* Previous clock level (+1/-1) */
	double		prev_bb_sample;		/* Previous baseband sample (for zero cross) */
	double		integrator;		/* Integrate-and-dump accumulator */
	
	/* Biphase Decoding */
	int		curr_bit;		/* Current raw bit */
	double		prev_integral;		/* Previous integral value */
	int		biphase_counter;	/* Counter for phase ambiguity check */
	int		reading_frame;		/* 0 or 1 (phase alignment) */
	int		total_errors[2];	/* Error counters for both phases */
	
	/* Bit recovery */
	int		last_diff_bit;		/* Last differential bit for decoding */
	int		bit_count;
	uint32_t	shift_reg;		/* 26-bit shift register */
	
	/* Block/Group assembly */
	uint16_t	blocks[4];	/* A, B, C, D */
	int		block_idx;	/* current block (0-3) */
	int		synced;		/* block sync acquired */
	int		errors;		/* consecutive error count */
	int		bit_count_in_block; /* Flywheel counter (0..25) */
	int		group_mask;	/* Bitmask of valid blocks (A=1, B=2, C=4, D=8) */
	uint8_t		block_status[4]; /* Per-block status: 0=valid, 1=FEC-corrected, 2=error */
	
	/* Multi-Hit Sync Acquisition (IEC 62106 Annex B) */
	#define SYNC_THRESHOLD 4       /* Need 5+ hits to acquire sync (was 2, increased for noise immunity) */
	#define SYNC_CONFIRM_BITS 520  /* 5 groups * 104 bits */
	#define SYNC_LOSS_GROUPS 4     /* Lose sync after 4 consecutive bad groups (increased from 2) */
	int		sync_hits[26][4];      /* Hit count by [offset][pseudoBlock] */
	long		sync_hit_time[26][4];  /* Last hit time for aging */
	long		bit_time;              /* Global bit counter */
	int		nb_ok;                 /* Valid blocks in current group */
	int		nb_unsync;             /* Consecutive groups with no valid blocks */

	/* BER Tracking */
	#define BER_WINDOW_SIZE 12
	float		ber_history[BER_WINDOW_SIZE];
	int		ber_history_idx;
	double		ber_accumulator;
	int		group_error_count;
	int		blocks_in_group;
	
	/* Statistics */
	long		blocks_received;
	long		blocks_ok;
	long		blocks_bad;
	double		ber_percent;
	
	/* Decoded data */
	uint16_t	pi;
	uint8_t		pi_status;		/* Status of PI decode */
	uint8_t		pty;
	uint8_t		pty_status;		/* Status of PTY decode */
	uint8_t		tp;
	uint8_t		tp_status;		/* Status of TP decode */
	uint8_t		ta;
	uint8_t		ta_status;		/* Status of TA decode */
	char		ps[9];
	uint8_t		ps_status[8];	/* Per-char status: enum rds_decode_status */
	char		rt[65];
	uint8_t		rt_status[64];	/* Per-char status for RadioText */
	uint8_t		rt_ab;
	int		ps_segments;
	int		rt_segments;
	int		groups_received;
	
	/* Group 1A/1B: Slow Labeling Codes (IEC 62106 S6.1.5.2) */
	uint8_t		linkage_actuator;	/* LA flag */
	uint8_t		ecc;			/* Extended Country Code (variant 0) */
	uint8_t		ecc_status;		/* Status of ECC decode */
	uint8_t		language_code;		/* Language code (variant 3) */
	uint8_t		language_status;	/* Status of language decode */
	uint16_t	pin;			/* Programme Item Number (raw) */
	uint8_t		pin_status;		/* Status of PIN decode */
	uint8_t		pin_day;		/* PIN: Day (1-31) */
	uint8_t		pin_hour;		/* PIN: Hour (0-23) */
	uint8_t		pin_minute;		/* PIN: Minute (0-59) */
	
	/* Group 4A: Clock-Time and Date (IEC 62106 S6.1.5.4) */
	uint32_t	ct_mjd;			/* Modified Julian Date */
	uint8_t		ct_hour;		/* Hour (0-23 UTC) */
	uint8_t		ct_minute;		/* Minute (0-59) */
	int8_t		ct_offset;		/* Local time offset in half-hours (-24 to +24) */
	int		ct_valid;		/* CT received at least once */
	uint8_t		ct_status;		/* Status of CT decode */
	
	/* Group 10A: Programme Type Name (IEC 62106 S6.1.5.8) */
	char		ptyn[9];		/* PTY Name (8 chars) */
	uint8_t		ptyn_status[8];		/* Per-char status for PTYN */
	uint8_t		ptyn_ab;		/* A/B flag for PTYN */
	int		ptyn_segments;		/* Received segments mask (bit 0=first half, 1=second half) */

	/* Group 14A/14B: Enhanced Other Networks (IEC 62106 S6.1.5.14) */
	uint16_t	on_pi;			/* PI of Other Network (current slot) */
	char		on_ps[9];		/* PS of Other Network */
	int		on_ps_segments;		/* Received PS segments mask for ON */
	uint8_t		on_pty;			/* PTY of ON */
	uint8_t		on_tp;			/* TP flag of ON */
	uint8_t		on_ta;			/* TA flag of ON */
	uint16_t	on_pin;			/* PIN of ON */
	
	/* Status timing */
	double		status_timer;
	double		status_interval;
} rds_decoder_t;

/* Initialize RDS decoder
 * debug: Enable debug logging (raw hex codes, --rds-debug)
 * verbose: Enable verbose/info logging (human-readable, --rds-verbose)
 */
int rds_decoder_init(rds_decoder_t *rds, double samplerate, int debug, int verbose);

/* Process FM baseband samples, extract RDS data
 * pilot_phase: current 19 kHz pilot phase from stereo decoder (or free-run if 0)
 * pilot_phasestep: phase increment per sample for pilot (2pi x 19000 / samplerate)
 */
void rds_decoder_process(rds_decoder_t *rds, sample_t *samples, int num,
                         double pilot_phase, double pilot_phasestep);

/* Get decoded data (returns 1 if new data available) */
int rds_get_pi(rds_decoder_t *rds, uint16_t *pi);
int rds_get_ps(rds_decoder_t *rds, char *ps);  /* ps must be at least 9 bytes */
int rds_get_rt(rds_decoder_t *rds, char *rt);  /* rt must be at least 65 bytes */

/* Print decoder status (PI, PS, RT, BER) */
void rds_decoder_status(rds_decoder_t *rds);

/* Cleanup */
void rds_decoder_exit(rds_decoder_t *rds);

#endif /* _RDS_H */
