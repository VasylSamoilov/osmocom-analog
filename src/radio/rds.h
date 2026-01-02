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

/* Macros for human-readable preset definitions */

/* --- Variadic Macro Helpers (Count and Map) --- */
#define PP_NARG(...) PP_NARG_(__VA_ARGS__,PP_RSEQ_N())
#define PP_NARG_(...) PP_ARG_N(__VA_ARGS__)
#define PP_ARG_N( \
 _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
 _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
 _21,_22,_23,_24,_25,N,...) N
#define PP_RSEQ_N() \
 25,24,23,22,21,20,19,18,17,16,15,14,13,12,11, \
 10,9,8,7,6,5,4,3,2,1,0

#define MAP(c, f, ...) MAP_(PP_NARG(__VA_ARGS__), c, f, __VA_ARGS__)
#define MAP_(N, c, f, ...) MAP__(N, c, f, __VA_ARGS__)
#define MAP__(N, c, f, ...) MAP_##N(c, f, __VA_ARGS__)

#define MAP_1(c, f, x)      f(x)
#define MAP_2(c, f, x, ...) f(x) c() MAP_1(c, f, __VA_ARGS__)
#define MAP_3(c, f, x, ...) f(x) c() MAP_2(c, f, __VA_ARGS__)
#define MAP_4(c, f, x, ...) f(x) c() MAP_3(c, f, __VA_ARGS__)
#define MAP_5(c, f, x, ...) f(x) c() MAP_4(c, f, __VA_ARGS__)
#define MAP_6(c, f, x, ...) f(x) c() MAP_5(c, f, __VA_ARGS__)
#define MAP_7(c, f, x, ...) f(x) c() MAP_6(c, f, __VA_ARGS__)
#define MAP_8(c, f, x, ...) f(x) c() MAP_7(c, f, __VA_ARGS__)
#define MAP_9(c, f, x, ...) f(x) c() MAP_8(c, f, __VA_ARGS__)
#define MAP_10(c, f, x, ...) f(x) c() MAP_9(c, f, __VA_ARGS__)
#define MAP_11(c, f, x, ...) f(x) c() MAP_10(c, f, __VA_ARGS__)
#define MAP_12(c, f, x, ...) f(x) c() MAP_11(c, f, __VA_ARGS__)
#define MAP_13(c, f, x, ...) f(x) c() MAP_12(c, f, __VA_ARGS__)
#define MAP_14(c, f, x, ...) f(x) c() MAP_13(c, f, __VA_ARGS__)
#define MAP_15(c, f, x, ...) f(x) c() MAP_14(c, f, __VA_ARGS__)
#define MAP_16(c, f, x, ...) f(x) c() MAP_15(c, f, __VA_ARGS__)
#define MAP_17(c, f, x, ...) f(x) c() MAP_16(c, f, __VA_ARGS__)
#define MAP_18(c, f, x, ...) f(x) c() MAP_17(c, f, __VA_ARGS__)
#define MAP_19(c, f, x, ...) f(x) c() MAP_18(c, f, __VA_ARGS__)
#define MAP_20(c, f, x, ...) f(x) c() MAP_19(c, f, __VA_ARGS__)
#define MAP_21(c, f, x, ...) f(x) c() MAP_20(c, f, __VA_ARGS__)
#define MAP_22(c, f, x, ...) f(x) c() MAP_21(c, f, __VA_ARGS__)
#define MAP_23(c, f, x, ...) f(x) c() MAP_22(c, f, __VA_ARGS__)
#define MAP_24(c, f, x, ...) f(x) c() MAP_23(c, f, __VA_ARGS__)
#define MAP_25(c, f, x, ...) f(x) c() MAP_24(c, f, __VA_ARGS__)

#define COMMA() ,

/* AF frequency code conversion for preset usage */
#define RDS_AF_MHZ(mhz)  ( (uint8_t)( ((mhz) - 87.5) * 10.0 + 0.5 ) )


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

/* Default injection level - use NRSC commercial level for better decode reliability */
#define RDS_INJECTION           RDS_INJECTION_NRSC  /* 6.7% = +/-5.0 kHz */

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
 * Alternative Frequencies (AF) - Group 0A Block C
 * EN 50067 S3.2.1.6, IEC 62106 Table 11
 *
 * AF enables seamless frequency switching for mobile receivers.
 * Receivers store AFs, monitor RSSI, and retune to stronger AF
 * with matching PI when signal weakens - preventing dropouts.
 *
 * Each Group 0A Block C carries two 8-bit AF codes.
 *
 * AF CODE RANGES (8-bit):
 *   0x00 (0):       Not to be used
 *   0x01-0xCC (1-204): FM frequencies 87.6-107.9 MHz
 *                   Formula: freq_MHz = 87.5 + (code * 0.1)
 *   0xCD (205):     Filler code (ignore)
 *   0xCE-0xDF (206-223): Reserved
 *   0xE0-0xF9 (224-249): AF count codes (N = code - 224, 1-25 AFs)
 *   0xFA (250):     LF/MF frequency follows (next byte is LF/MF)
 *   0xFB-0xFF (251-255): Reserved
 *

 *
 * ============================================================
 * METHOD A (S3.2.1.6.3) - Simple list, up to 25 AFs
 * ============================================================
 * Used for small/medium networks where all AFs are equivalent.
 *
 * Structure:
 *   Group 0A #1 Block C: [count_code (224+N), AF1]
 *   Group 0A #2 Block C: [AF2, AF3]
 *   Group 0A #3 Block C: [AF4, AF5] ... etc
 *   Use filler 0xCD if odd number of AFs
 *
 * Example (5 AFs: 98.5, 99.3, 101.1, 102.7, 105.0 MHz):
 *   Group 0A #1: [229, 111]  // 229=5 AFs, 111=98.6MHz
 *   Group 0A #2: [118, 136]  // 118=99.3MHz, 136=101.1MHz
 *   Group 0A #3: [152, 175]  // 152=102.7MHz, 175=105.0MHz
 *
 * ============================================================
 * METHOD B (S3.2.1.6.4) - Paired, for >25 AFs or regional
 * ============================================================
 * Used for large networks (>25 AFs) or regional variants.
 * Each transmitter broadcasts its own frequency paired with alternatives.
 *
 * Raw list structure:
 *   [tx_freq, tx_freq, af1, tx_freq, af2, tx_freq, af3, ...]
 *
 * Block C encoding:
 *   Group 0A #1 Block C: [count_code (224+N), tx_freq]  - header
 *   Group 0A #2 Block C: [tx_freq, af1]                 - tx_freq always first
 *   Group 0A #3 Block C: [tx_freq, af2]                 - tx_freq always first
 *   ... (up to 12 pairs, 13 frequencies total)
 *
 * Regional detection: compare afN values ACROSS pairs
 *   - af1 < af2 < af3 (ascending): Same content
 *   - af1 > af2 > af3 (descending): Regional variants
 *
 * Example (tx at 89.3 MHz, 3 alternatives - same content):
 *   Group 0A #1: [227, 18]   // 227=3 freqs, 18=89.3MHz (tx_freq)
 *   Group 0A #2: [18, 95]    // [tx_freq, af1] af1=95=97.0 MHz
 *   Group 0A #3: [18, 120]   // [tx_freq, af2] af2=120=99.5 MHz (95<120: same)
 *   Group 0A #4: [18, 142]   // [tx_freq, af3] af3=142=101.7 MHz (120<142: same)
 *
 * NOTE: Method B detected heuristically when tx_freq repeats in pairs.
 *
 * ============================================================ */

#define RDS_AF_NOT_USED         0x00    /* Code 0: Not to be used */
#define RDS_AF_FILLER           0xCD    /* Code 205: Filler code */
#define RDS_AF_NO_AF            0xE0    /* Code 224: No AF exists */
#define RDS_AF_LF_MF_FOLLOWS    0xFA    /* Code 250: LF/MF frequency follows */
#define RDS_AF_NO_AF_PAIR       0xE0E0  /* 16-bit filler for Block C when no AF */

/* IEC 62106 Table 11 - AF Code Ranges */
#define RDS_AF_VHF_MIN          1       /* First valid VHF AF code */
#define RDS_AF_VHF_MAX          204     /* Last valid VHF AF code (107.9 MHz) */
#define RDS_AF_RESERVED1_MIN    206     /* Reserved range 206-223 start */
#define RDS_AF_RESERVED1_MAX    223     /* Reserved range 206-223 end */
#define RDS_AF_COUNT_MIN        224     /* Count code 0 (No AF) */
#define RDS_AF_COUNT_MAX        249     /* Count code 25 (max AFs) */
#define RDS_AF_RESERVED2_MIN    251     /* Reserved range 251-255 start */
/* Note: RDS_AF_RESERVED2_MAX is 255, omitted since uint8_t max */

/* AF frequency base (IEC 62106 Table 11) */
#define RDS_AF_FM_BASE          875     /* FM: 87.5 MHz in 0.1 MHz units */

/* AF entry flags bitmask (used in rds_af_item_t.flags) */
#define RDS_AF_FLAG_REGIONAL    0x01    /* Regional variant (Method B) */
#define RDS_AF_FLAG_LF          0x02    /* LF band frequency */
#define RDS_AF_FLAG_MF          0x04    /* MF band frequency */

/* Detect RBDS (US/Canada) from Extended Country Code
 * ECC 0xA0-0xA5 = Americas (RBDS, 10kHz AM spacing, US PTY names)
 * All others = RDS (EU/world, 9kHz AM spacing, EU PTY names)
 * See NRSC-4-B for RBDS, IEC 62106 Annex D for ECC tables */
#define RDS_ECC_RBDS_MIN        0xA0
#define RDS_ECC_RBDS_MAX        0xA5
#define RDS_IS_RBDS(ecc)        ((ecc) >= RDS_ECC_RBDS_MIN && (ecc) <= RDS_ECC_RBDS_MAX)

/* Detect RBDS from de-emphasis time constant (heuristic)
 * Only 50µs (±1µs tolerance) triggers RBDS assumption.
 * All other values (including 75µs) default to RDS, which is more common globally.
 * This is used for initial PTY name display before ECC is received. */
#define RDS_IS_RBDS_EMPHASIS(us)  ((us) >= 49.0 && (us) <= 51.0)


/* Segment masks for cycling */
#define RDS_PS_SEG_MASK         0x03    /* 4 segments for PS (0-3) */
#define RDS_RT_SEG_MASK         0x0F    /* 16 segments for RT (0-15) */

/* AF storage limits (EN 50067 S3.2.1.6)

 * Method A: max 25 AFs in simple list (new data overwrites old)
 * Method B: multiple sub-lists, each with tuning_freq + up to 12 pairs
 *           max 10 sub-lists; new replaces matching tuning_freq or oldest
 */
#define RDS_AF_MAX_METHOD_A     25      /* Method A: max 25 AFs */

/* ============================================================
 * AF METHOD A - Dedicated Structures (IEC 62106 S3.2.1.6.3)
 * ============================================================
 * Method A transmits a simple list of up to 25 AFs.
 * LF/MF frequencies count as 2 slots each (250 + code pair).
 * ============================================================ */

/* AF frequency type identifier */
typedef enum {
	RDS_AF_FREQ_VHF = 0,	/* VHF FM 87.6-107.9 MHz */
	RDS_AF_FREQ_LF,		/* LF 153-279 kHz */
	RDS_AF_FREQ_MF		/* MF 531-1602 kHz (RDS) or 540-1700 kHz (RBDS) */
} rds_af_freq_type_t;

/* AF Method A Encoder List - Human-readable input format */
typedef struct {
	uint8_t  slot_count;		/* Total slots used (LF/MF = 2 each) */
	uint8_t  vhf_count;		/* Number of VHF frequencies */
	uint8_t  lf_mf_count;		/* Number of LF/MF frequencies */
	uint16_t vhf_freq[25];		/* VHF frequencies in 0.1 MHz (e.g., 910 = 91.0 MHz) */
	uint16_t lf_mf_freq[12];	/* LF/MF frequencies in kHz (e.g., 225, 1008) */
	uint8_t  lf_mf_type[12];	/* rds_af_freq_type_t: RDS_AF_FREQ_LF or RDS_AF_FREQ_MF */
} rds_af_method_a_t;

/* AF Method A Decoder List - Rebuilt from RDS stream */
typedef struct {
	uint16_t pi;			/* PI this list was received for (for change detection) */
	uint8_t  expected_count;	/* Count from code 224-249 (1-25 slots) */
	uint8_t  received_count;	/* Actually received so far */
	uint16_t freq[25];		/* Decoded frequencies (0.1 MHz VHF or kHz LF/MF) */
	uint8_t  type[25];		/* rds_af_freq_type_t per frequency */
	uint8_t  status[25];		/* Block decode status: RDS_STATUS_* per slot */
	uint8_t  complete;		/* 1 when all expected freqs received */
	uint8_t  lf_mf_follows;		/* State: 1 if previous code was 250 (marker) */
	
	/* Last successfully decoded complete list (all good/corrected status) */
	uint16_t last_good_pi;		/* PI when last good was received */
	uint16_t last_good_tuning;	/* Tuning frequency when received (0 for now) */
	uint8_t  last_good_count;	/* Number of freqs in last_good */
	uint16_t last_good_freq[25];	/* Frequencies from last good decode */
	uint8_t  last_good_type[25];	/* Types from last good decode */
	time_t   last_good_time;	/* Timestamp of last good decode */
} rds_af_method_a_dec_t;

/* AF Method A Helper Functions */
int rds_af_method_a_parse(const char *input, rds_af_method_a_t *out);
int rds_af_method_a_build_codes(const rds_af_method_a_t *af, uint8_t *codes, int max_codes);

/* ============================================================
 * AF METHOD B - Paired Frequency Lists (IEC 62106 S3.2.1.6.4)
 * ============================================================
 * Method B is used when:
 *   - Number of AFs exceeds 25
 *   - Frequencies belong to different regions/programmes
 *
 * Each list: [count+tuning_freq header] + up to 12 [tuning, AF] pairs
 *
 * PAIR ORDERING (F1 | F2):
 *   F1 < F2 (ascending)  = Same programme on both frequencies
 *   F1 > F2 (descending) = Regional variant / different programme
 *
 * Tuning frequency can appear in EITHER position of the pair.
 * The AF is the frequency that is NOT the tuning frequency.
 *
 * Example (tuning = 89.3 MHz):
 *   89.3 | 99.5  → F1<F2, same programme, AF=99.5
 *   88.8 | 89.3  → F1<F2, same programme, AF=88.8
 *   102.6| 89.3  → F1>F2, regional, AF=102.6
 *   89.3 | 89.0  → F1>F2, regional, AF=89.0
 * ============================================================ */

#define RDS_AF_METHOD_B_MAX_LISTS   8   /* Max separate Method B lists per preset */
#define RDS_AF_METHOD_B_MAX_AFS    12   /* Max AFs per list (excl tuning freq) */
#define RDS_AF_METHOD_B_HISTORY_MAX 10  /* Max history entries for decoder */

/* AF Method B Encoder List - Human-readable input format
 * Format: "T89.3, 99.5, 88.8, R102.6, R89.0"
 *   T prefix = tuning frequency (required first element)
 *   R prefix = regional variant (will use descending order)
 *   No prefix = same programme (will use ascending order) */
typedef struct {
	uint16_t tuning_freq;                       /* Tuning frequency in 0.1 MHz */
	uint8_t  af_count;                          /* Number of AFs (max 12) */
	uint16_t af_freq[RDS_AF_METHOD_B_MAX_AFS];  /* AF frequencies in 0.1 MHz */
	uint8_t  af_is_regional[RDS_AF_METHOD_B_MAX_AFS]; /* 1=regional, 0=same programme */
} rds_af_method_b_list_t;

typedef struct {
	uint8_t list_count;                          /* Number of lists */
	rds_af_method_b_list_t lists[RDS_AF_METHOD_B_MAX_LISTS];
} rds_af_method_b_t;

/* AF Method B Decoder - Single history entry */
typedef struct {
	uint16_t pi;                                /* PI this list was received for */
	uint16_t tuning_freq;                       /* Tuning frequency in 0.1 MHz */
	uint8_t  expected_count;                    /* Expected freq count from code */
	uint8_t  received_count;                    /* Actually received */
	uint16_t af_freq[RDS_AF_METHOD_B_MAX_AFS];  /* AF frequencies */
	uint8_t  af_is_regional[RDS_AF_METHOD_B_MAX_AFS]; /* Regional flags */
	uint8_t  af_status[RDS_AF_METHOD_B_MAX_AFS];/* Decode status per AF */
	time_t   timestamp;                         /* Unix timestamp when received */
	uint8_t  complete;                          /* 1 when all expected freqs received */
} rds_af_method_b_history_t;

/* AF Method B Decoder State */
typedef struct {
	/* Current list being assembled */
	uint16_t pi;                                /* PI tracking */
	uint16_t tuning_freq;                       /* Current tuning frequency */
	uint8_t  expected_count;                    /* From count code */
	uint8_t  received_count;                    /* Pairs received so far */
	uint16_t af_freq[RDS_AF_METHOD_B_MAX_AFS];
	uint8_t  af_is_regional[RDS_AF_METHOD_B_MAX_AFS];
	uint8_t  af_status[RDS_AF_METHOD_B_MAX_AFS];
	
	/* History of complete lists (newest first) */
	rds_af_method_b_history_t history[RDS_AF_METHOD_B_HISTORY_MAX];
	uint8_t  history_count;                     /* Valid entries in history */
} rds_af_method_b_dec_t;

/* AF Collector - buffers codes before method determination */
typedef struct {
	uint16_t pi;                     /* PI tracking for invalidation */
	uint8_t  expected_count;         /* From count code (1-25) */
	uint8_t  received_pairs;         /* AF code pairs received */
	uint8_t  codes[52];              /* Raw AF codes as received */
	uint8_t  status[52];             /* Per-code decode status */
	uint16_t tuning_freq;            /* Second byte of header (potential Method B tuning) */
	uint8_t  header_received;        /* 1 if count+second byte received */
} rds_af_collector_t;

/* AF Method B Helper Functions */
int rds_af_method_b_parse(const char *input, rds_af_method_b_list_t *out);
int rds_af_method_b_build_codes(const rds_af_method_b_list_t *list, uint8_t *codes, int max_codes);

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

/* Block D: Programme Item Number (PIN) - IEC 62106:2015 S6.1.5.2
 *
 * HISTORICAL CONTEXT (1984-1990s):
 *   PIN enabled "VCR-like" radio recording. Users selected a programme from
 *   published schedules (newspapers, guides) and entered its PIN into their
 *   receiver. The receiver would:
 *     1. Tune to the station and monitor for the matching PIN
 *     2. Activate when transmitted PIN matched (programme actually started)
 *     3. Trigger recording, turn on radio, or switch audio output
 *   This compensated for schedule delays/overruns - more reliable than timers.
 *
 * PIN bit fields are used in:
 *   - Group 1A Block D:    PIN for THIS service (current PI)
 *   - Group 14A variant 14: PIN for LINKED service (ON-PI via EON)
 *
 * PIN FIELD STRUCTURE:
 *   Day (5 bits):    Day of month 1-31, or 0 = PIN not used
 *   Hour (5 bits):   Hour 0-23, or 24 = programme end time
 *   Minute (6 bits): Minute 0-59
 *
 * DATE RESOLUTION:
 *   PIN only contains day-of-month (no month/year). Receiver uses CT
 *   (Group 4A) for month/year context to disambiguate today vs tomorrow.
 *
 * IMPORTANT DISTINCTIONS:
 *   - PIN ≠ PTY: PIN identifies WHEN a programme starts, PTY identifies
 *     WHAT content type is currently playing
 *   - PIN was for specific programme items from published schedules,
 *     NOT for searching by category (that's PTY's job)
 *
 * MODERN STATUS (post-2018):
 *   PIN is considered OBSOLETE in updated IEC 62106 revisions.
 *   Modern alternatives: app-based streaming, DAB, podcast scheduling.
 *   Most stations transmit PIN = 0x0000 (no PIN).
 *
 * EDGE CASES:
 *   - Day = 0:     PIN not used (no scheduled programme)
 *   - PIN = 0x0000: "No valid PIN" - most commonly transmitted value
 *   - Hour = 24:   Programme end time (rare usage)
 */
#define RDS_PIN_DAY_SHIFT       11
#define RDS_PIN_DAY_MASK        0xF800  /* Bits 15-11 (5 bits): Day 1-31, 0=unused */
#define RDS_PIN_HOUR_SHIFT      6
#define RDS_PIN_HOUR_MASK       0x07C0  /* Bits 10-6 (5 bits): Hour 0-23, 24=end */
#define RDS_PIN_MINUTE_MASK     0x003F  /* Bits 5-0 (6 bits): Minute 0-59 */

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
 * Block B payload (bits 4-0):
 *   Bit 4:   A/B flag (toggles when PTYN changes)
 *   Bits 3-1: Reserved (set to 0)
 *   Bit 0:   Segment address (0=chars 1-4, 1=chars 5-8)
 *
 * Block C: PTYN chars 1-2 or 5-6
 * Block D: PTYN chars 3-4 or 7-8
 * ============================================================ */
#define RDS_10A_AB_FLAG_BIT     4
#define RDS_10A_AB_MASK         0x0010
#define RDS_10A_SEGMENT_BIT     0       /* 0=First 4 chars, 1=Last 4 chars */
#define RDS_10A_SEGMENT_MASK    0x0001

/* ============================================================
 * Group 3A Bit Fields (IEC 62106 S6.1.5.5)
 * Open Data Application (ODA) Identification
 *
 * Block B payload (bits 4-0):
 *   Bits 4-0: Application Group Type Code (indicates which group carries ODA)
 *
 * Block C: Application-specific message (ODA-dependent)
 * Block D: Application Identification (AID) - 16-bit registered code
 * ============================================================ */
#define RDS_3A_APP_GROUP_MASK   0x001F  /* Block B bits 4-0: App group type code */

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
#define RDS_14A_TP_ON_BIT       4       /* Block B bit 4: TP for ON */
#define RDS_14A_TP_ON_MASK      0x0010

/* 14A Usage Variant Codes (IEC 62106 Table 17) */
#define RDS_14A_VARIANT_PS_0    0       /* chars 1-2 */
#define RDS_14A_VARIANT_PS_1    1       /* chars 3-4 */
#define RDS_14A_VARIANT_PS_2    2       /* chars 5-6 */
#define RDS_14A_VARIANT_PS_3    3       /* chars 7-8 */
#define RDS_14A_VARIANT_AF      4       /* Alternative Freqs */
#define RDS_14A_VARIANT_MAP_5   5       /* Mapped freq (tuned->variant) */
#define RDS_14A_VARIANT_MAP_6   6       /* Mapped freq */
#define RDS_14A_VARIANT_MAP_7   7       /* Mapped freq */
#define RDS_14A_VARIANT_MAP_8   8       /* Mapped freq */
#define RDS_14A_VARIANT_MAP_9   9       /* Mapped freq */
#define RDS_14A_VARIANT_LINK    12      /* Linkage/E-Linkage info */
#define RDS_14A_VARIANT_INFO    13      /* PTY, TA, TP for ON */
#define RDS_14A_VARIANT_PIN     14      /* PIN for ON */
#define RDS_14A_VARIANT_BCAST   15      /* Broadcaster data (reserved) */

/* 14A Variant 12 (Linkage) Block C fields */
#define RDS_14A_LINK_LA_BIT     15      /* Linkage Actuator */
#define RDS_14A_LINK_LA_MASK    0x8000
#define RDS_14A_LINK_LSN_MASK   0x0FFF  /* Linkage Set Number (12 bits) */

/* 14A Variant 13 (Info) Block C fields */
#define RDS_14A_INFO_PTY_SHIFT  11
#define RDS_14A_INFO_PTY_MASK   0xF800  /* ON-PTY (5 bits) */
#define RDS_14A_INFO_TA_BIT     0       /* ON-TA flag */
#define RDS_14A_INFO_TA_MASK    0x0001

/* Group 14B Block B payload */
#define RDS_14B_TA_BIT          4       /* TA for ON (Block B bit 4) */
#define RDS_14B_TA_MASK         0x0010

/* ============================================================
 * Group 15B Bit Fields (IEC 62106 S6.1.5.16)
 * Fast Basic Tuning and Switching Information
 *
 * Purpose: Allows fast TA/TP detection for mobile receivers.
 * Structure mirrors Group 0B but in Block D format.
 *
 * Block B payload (bits 4-0):
 *   Bit 4:   TA (Traffic Announcement)
 *   Bit 3:   M/S (Music/Speech)
 *   Bit 2:   DI (Decoder Information, depends on segment)
 *   Bits 1-0: PS segment address (0-3)
 *
 * Block C: PI repeat (same as Block A)
 * Block D: Repeat of Block B payload structure
 * ============================================================ */
#define RDS_15B_TA_BIT          4
#define RDS_15B_TA_MASK         0x0010
#define RDS_15B_MS_BIT          3
#define RDS_15B_MS_MASK         0x0008
#define RDS_15B_DI_BIT          2
#define RDS_15B_DI_MASK         0x0004
#define RDS_15B_SEG_MASK        0x0003  /* Bits 1-0 */

/* ============================================================
 * Field Validation Macros
 * Runtime validation for RDS field ranges per IEC 62106
 * ============================================================ */
#define RDS_VALID_PTY(pty)        ((pty) <= 31)
#define RDS_VALID_HOUR(h)         ((h) <= 23)
#define RDS_VALID_MINUTE(m)       ((m) <= 59)
#define RDS_VALID_TZ_OFFSET(o)    ((o) >= -24 && (o) <= 24)
#define RDS_VALID_PIN_DAY(d)      ((d) >= 0 && (d) <= 31)
#define RDS_VALID_AF_CODE(c)      ((c) >= 1 && (c) <= 204)
#define RDS_VALID_MJD(mjd)        ((mjd) >= 15079)  /* Jan 1, 1900 */


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
 * ENHANCED OTHER NETWORKS (EON) DATA STRUCTURES
 * IEC 62106 S6.1.5.14 - Group 14A/14B
 * ============================================================ */

#define RDS_EON_MAX_ENTRIES     64      /* Maximum tracked Other Networks */
#define RDS_EON_MAX_AF          25      /* Max AFs per Other Network */
#define RDS_EON_MAX_MAPPED_AF   8       /* Max mapped AFs (variants 5-9) */

/* Mapped AF pair: maps tuned frequency to Other Network frequency */
typedef struct {
	uint8_t		tuned_af;	/* AF code of current tuned freq (1-204) */
	uint8_t		on_af;		/* AF code of Other Network freq (1-204) */
} rds_mapped_af_t;

/* EON entry for one Other Network */
typedef struct {
	uint16_t	pi;		/* PI code of Other Network (0 = unused) */
	char		ps[9];		/* PS name (8 chars + NUL) */
	uint8_t		ps_segments;	/* Received PS segments bitmask (0-3) */
	uint8_t		pty;		/* Programme Type */
	uint8_t		tp;		/* Traffic Programme flag */
	uint8_t		ta;		/* Traffic Announcement flag */
	
	/* AF list (variant 4) */
	uint16_t	af[RDS_EON_MAX_AF];	/* Frequencies in 0.1 MHz (e.g. 1001=100.1) */
	uint8_t		af_count;		/* Number of valid AF entries */
	
	/* Mapped AFs (variants 5-9) */
	rds_mapped_af_t	mapped_af[RDS_EON_MAX_MAPPED_AF];
	uint8_t		mapped_af_count;
	
	/* Linkage (variant 12) */
	uint8_t		linkage_la;	/* Linkage Actuator */
	uint16_t	linkage_lsn;	/* Linkage Set Number (12 bits) */
	
	/* PIN (variant 14) - for LINKED service (this ON), not current station
	 * IEC 62106:2015 S6.1.5.2 - receiver uses CT for month context */
	uint16_t	pin;		/* Programme Item Number (raw 16-bit) */
	uint8_t		pin_day;	/* Day of month 1-31 (0 = PIN not used) */
	uint8_t		pin_hour;	/* Hour 0-23, or 24 = end time */
	uint8_t		pin_minute;	/* Minute 0-59 */
	
	/* Broadcaster data (variant 15) */
	uint16_t	broadcaster_data;
	
	/* Housekeeping */
	uint32_t	last_update;	/* Timestamp of last update (group count) */
} rds_eon_entry_t;

/* ============================================================
 * GROUP VERSION SELECTION (IEC 62106)
 * ============================================================
 * Each RDS group type (0-15) has two versions: A and B.
 *
 * Version A: Block C carries group-specific data (AF, SLC, RT chars)
 * Version B: Block C carries PI repeat for faster station identification
 *
 * Trade-off:
 *   - A versions: More data capacity (37 bits payload)
 *   - B versions: Faster PI identification, better mobile reception (21 bits)
 *
 * Selection per group type:
 *   - Group 0: 0A if AF list needed, 0B otherwise
 *   - Group 1: 1A if ECC/Language needed, 1B for PIN only
 *   - Group 2: 2A for 64-char RT, 2B for 32-char with faster cycling
 *
 * Important: Never mix A and B for the SAME group type in one stream.
 *            Different group types CAN use different versions (e.g., 0A + 2B).
 * ============================================================ */

typedef enum {
	RDS_GROUP_VERSION_AUTO = 0,	/* Auto-detect based on data */
	RDS_GROUP_VERSION_A,		/* Force version A */
	RDS_GROUP_VERSION_B		/* Force version B */
} rds_group_version_t;

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
	uint8_t		ms;		/* Music/Speech flag (1=Music, 0=Speech) */
	int		debug;		/* Debug logging (--rds-debug) */
	int		verbose;	/* Verbose logging (--rds-verbose) */
	
	/* Decoder Identification (DI) flags - transmitted via Group 0A/0B */
	uint8_t		di_stereo;		/* d3: Stereo (1) or Mono (0) */
	uint8_t		di_artificial_head;	/* d2: Artificial head recording */
	uint8_t		di_compressed;		/* d1: Compressed audio */
	uint8_t		di_dynamic_pty;		/* d0: Dynamic PTY indicator */
	
	/* Group 0A: Alternative Frequencies - Method A only
	 * Uses human-readable frequency string format (IEC 62106 S3.2.1.6.3) */
	rds_af_method_a_t af_method_a;		/* Method A frequency list */
	int		af_method_a_segment;	/* Current segment (pair index) for cycling */
	
	/* Group 0A: Alternative Frequencies - Method B
	 * Uses paired frequency format (IEC 62106 S3.2.1.6.4) */
	rds_af_method_b_t af_method_b;		/* Method B frequency lists */
	int		af_method_b_list_idx;	/* Current list being transmitted */
	int		af_method_b_pair_idx;	/* Current pair within list (0=header) */
	uint8_t		use_method_b;		/* 1 if using Method B instead of A */
	
	/* Group 0A/0B: Pre-computed AF code buffer for transmission
	 * Built from af_method_a or af_method_b at preset load time */
	uint8_t		af_codes[52];		/* AF code sequence (max 25 slots + filler) */
	int		af_code_count;		/* Number of codes in af_codes[] */
	
	/* Group version selection */
	uint8_t		use_0b;		/* Use Group 0B (PI repeat) instead of 0A (AF) */
	uint8_t		use_2b;		/* Use Group 2B (32-char RT) instead of 2A (64-char) */
	uint8_t		use_1b;		/* Use Group 1B (PIN only) instead of 1A (ECC/Lang) */

	
	char		ps[9];		/* Program Service name (8 chars + NUL) */
	char		rt[65];		/* RadioText (64 chars + NUL) */
	uint8_t		rt_ab;		/* RadioText A/B flag */
	uint8_t		rt_last_version;	/* Last RT version: 0=2A, 1=2B, 0xFF=none */
	
	/* Extended Configuration (Phase 2) */
	uint8_t		ecc;		/* Extended Country Code */
	uint8_t		language;	/* Language Code */
	char		ptyn[9];	/* Program Type Name (8 chars + NUL) */
	uint8_t		ptyn_ab;	/* PTYN A/B flag */
	int		ct_enabled;	/* Clock-Time transmission enabled */
	time_t		ct_time_offset;	/* Offset from system time for CT (seconds) */
	int8_t		local_offset;	/* Local Time Offset from UTC (half-hours) */
	/* PIN (Programme Item Number) - for THIS service (current PI)
	 * IEC 62106:2015 S6.1.5.2 - receiver uses CT for month context */
	uint8_t		pin_day;	/* Day of month 1-31 (0 = PIN not used) */
	uint8_t		pin_hour;	/* Hour 0-23, or 24 = end time */
	uint8_t		pin_minute;	/* Minute 0-59 */
	uint8_t		linkage_actuator;/* LA flag for Group 1A (IEC 62106 S6.1.5.2) */
	
	/* Group 1A SLC variants (IEC 62106 Table 9) - set non-zero to enable */
	uint16_t	tmc_id;		/* TMC identification (variant 1, 12 bits) */
	uint16_t	ews_channel;	/* EWS channel ID (variant 7, 12 bits) */
	uint16_t	slc_broadcaster;/* Broadcaster data (variant 6, 12 bits) */

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
	int		warmup_countdown;	/* Groups remaining in warmup mode (0 = done) */
	int		slc_variant;	/* Group 1A: current SLC variant index */
	
	/* Group 14A/14B: Enhanced Other Networks (EON) TX Configuration */
	rds_eon_entry_t	eon_tx[RDS_EON_MAX_ENTRIES];	/* Other Networks to transmit */
	int		eon_tx_count;		/* Number of configured ONs */
	int		eon_tx_index;		/* Current ON being transmitted */
	int		eon_tx_variant;		/* Current variant within ON */
	int		eon_enabled;		/* Enable EON transmission */
	
	/* Fixed Group Sequence Scheduler */
	#define RDS_SCHEDULER_MAX_LEN 64
	enum rds_group_type group_sched_buffer[RDS_SCHEDULER_MAX_LEN];
	int			group_sched_len;
	int			group_sched_index;
	

	
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

/* Update group scheduler sequence (call after changing PTY, PTYN, or EON) */
void rds_scheduler_update(rds_encoder_t *rds);

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
	
	/* Group 0A/0B: Music/Speech and Decoder Identification (IEC 62106 S6.1.5.1) */
	uint8_t		ms;			/* Music (1) / Speech (0) flag */
	uint8_t		ms_status;		/* Status of M/S decode */
	uint8_t		di_stereo;		/* d3: Stereo (1) or Mono (0) */
	uint8_t		di_artificial_head;	/* d2: Artificial head recording */
	uint8_t		di_compressed;		/* d1: Compressed audio */
	uint8_t		di_dynamic_pty;		/* d0: Dynamic PTY indicator */
	uint8_t		di_status;		/* Status of DI decode (any flag) */
	
	/* Group 0A: Alternative Frequencies - Method A only
	 * Decoded AF list with PI tracking and per-slot decode status.
	 * PI-change invalidates the list (IEC 62106 S3.2.1.6.3). */
	rds_af_method_a_dec_t af_method_a_dec;
	
	/* Group 0A: Alternative Frequencies - Method B with history
	 * Decoded AF pairs with regional flags and timestamp history.
	 * PI-change invalidates the current list (IEC 62106 S3.2.1.6.4). */
	rds_af_method_b_dec_t af_method_b_dec;
	
	/* AF Collector - buffers codes before method determination */
	rds_af_collector_t af_collector;
	
	char		ps[9];
	uint8_t		ps_status[8];	/* Per-char status: enum rds_decode_status */
	
	/* Group 2A/2B: RadioText (IEC 62106 S6.1.5.3)
	 * EN 50067: "A mixture of type 2A and type 2B groups must not be used
	 * when transmitting any one given message." Therefore we maintain
	 * separate buffers and A/B flags for each version.
	 * 
	 * rt[] is the display buffer - points to whichever version (2A/2B) 
	 * was last received. When A/B flag changes, the corresponding buffer
	 * is cleared and segments are reset. */
	char		rt_2a[65];		/* 2A buffer: 64 chars + NUL */
	uint8_t		rt_2a_status[64];	/* Per-char status for 2A */
	uint8_t		rt_2a_ab;		/* 2A A/B flag */
	uint16_t	rt_2a_segments;		/* 2A segment bitmask (0-15) */
	
	char		rt_2b[33];		/* 2B buffer: 32 chars + NUL */
	uint8_t		rt_2b_status[32];	/* Per-char status for 2B */
	uint8_t		rt_2b_ab;		/* 2B A/B flag */
	uint16_t	rt_2b_segments;		/* 2B segment bitmask (0-15) */
	
	char		rt[65];			/* Display buffer (copy of last received) */
	uint8_t		rt_status[64];		/* Per-char status for display */
	uint8_t		rt_ab;			/* Display A/B flag */
	uint8_t		rt_version;		/* 0=2A was last, 1=2B was last */
	uint8_t		rt_display_version;	/* Version in display buffer: 0=2A, 1=2B, 0xFF=none */
	
	int		ps_segments;
	int		rt_segments;		/* Legacy: combined segment count */
	int		groups_received;

	
	/* Group 1A/1B: Slow Labeling Codes (IEC 62106 S6.1.5.2) */
	uint8_t		linkage_actuator;	/* LA flag */
	uint8_t		ecc;			/* Extended Country Code (variant 0) */
	uint8_t		ecc_status;		/* Status of ECC decode */
	uint8_t		language_code;		/* Language code (variant 3) */
	uint8_t		language_status;	/* Status of language decode */
	uint16_t	pin;			/* Programme Item Number (raw 16-bit) */
	uint8_t		pin_status;		/* Status of PIN decode */
	uint8_t		pin_day;		/* Day of month 1-31 (0 = not used) */
	uint8_t		pin_hour;		/* Hour 0-23, or 24 = end time */
	uint8_t		pin_minute;		/* Minute 0-59 */
	
	/* Group 1A SLC variants (IEC 62106 Table 9) */
	uint16_t	tmc_id;			/* TMC identification (variant 1, 12 bits) */
	uint8_t		tmc_id_status;		/* Status of TMC ID decode */
	uint16_t	ews_channel;		/* EWS channel ID (variant 7, 12 bits) */
	uint8_t		ews_channel_status;	/* Status of EWS decode */
	uint16_t	slc_broadcaster;	/* Broadcaster data (variant 6, 12 bits) */
	uint8_t		slc_broadcaster_status;	/* Status of broadcaster decode */
	
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
	rds_eon_entry_t	eon[RDS_EON_MAX_ENTRIES];	/* Other Network database */
	int		eon_count;			/* Number of valid EON entries */
	
	/* Legacy single-slot fields (for API compatibility, point to eon[0]) */
	uint16_t	on_pi;			/* PI of most recent Other Network */
	char		on_ps[9];		/* PS of most recent Other Network */
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
int rds_decoder_init(rds_decoder_t *rds, double samplerate, int debug, int verbose, double time_constant_us);

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
