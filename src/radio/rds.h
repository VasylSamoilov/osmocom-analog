/*
 * RDS (Radio Data System) encoder and decoder
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
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
 * 75us (+/-1us tolerance) triggers RBDS assumption (Americas: USA, Canada, S. Korea).
 * 50us is used in Europe/Australia/most of world (RDS).
 * This is used for initial PTY name display before ECC is received. */
#define RDS_IS_RBDS_EMPHASIS(us)  ((us) >= 74.0 && (us) <= 76.0)


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
 *   89.3 | 99.5  -> F1<F2, same programme, AF=99.5
 *   88.8 | 89.3  -> F1<F2, same programme, AF=88.8
 *   102.6| 89.3  -> F1>F2, regional, AF=102.6
 *   89.3 | 89.0  -> F1>F2, regional, AF=89.0
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
 *   - PIN != PTY: PIN identifies WHEN a programme starts, PTY identifies
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
 * Group 7A Bit Fields (EN 50067 Annex M / IEC 62106)
 * Radio Paging (or ODA when paging not enabled)
 *
 * Block B payload (bits 4-0):
 *   Bit  4:     A/B Flag (paging call indicator, toggles per new call)
 *   Bits 3-0:   PSAC (Paging Segment Address Code)
 *
 * Block C: Paging address or data (depends on PSAC)
 * Block D: Paging address or data (depends on PSAC)
 * ============================================================ */
#define RDS_7A_AB_FLAG_BIT      4
#define RDS_7A_AB_FLAG_MASK     0x0010
#define RDS_7A_PSAC_MASK        0x000F  /* Bits 3-0: PSAC value */

/* PSAC values (EN 50067 Annex M Table M.2) */
#define RDS_7A_PSAC_TONE        0x0     /* Tone-only (1 group) */
#define RDS_7A_PSAC_NUM10_ADDR  0x2     /* 10-digit numeric: address group */
#define RDS_7A_PSAC_NUM10_DATA  0x3     /* 10-digit numeric: data group */
#define RDS_7A_PSAC_NUM18_ADDR  0x4     /* 18-digit numeric: address group */
#define RDS_7A_PSAC_NUM18_DATA1 0x5     /* 18-digit numeric: data group 1 */
#define RDS_7A_PSAC_NUM18_DATA2 0x6     /* 18-digit numeric: data group 2 */
#define RDS_7A_PSAC_INTL15      0x7     /* International 15-digit */
#define RDS_7A_PSAC_ALPHA_ADDR  0x8     /* Alphanumeric: address group */
#define RDS_7A_PSAC_ALPHA_DATA_FIRST 0x9 /* Alphanumeric: first data group */
#define RDS_7A_PSAC_ALPHA_DATA_LAST  0xE /* Alphanumeric: last data group (cycles 9-E) */
#define RDS_7A_PSAC_ALPHA_END   0xF     /* Alphanumeric: end-of-message */

/* BCD encoding for paging digits (EN 50067 Annex M) */
#define RDS_PAGING_BCD_SPACE    0xA     /* Space character in BCD */
#define RDS_PAGING_BCD_MAX      0x9     /* Maximum valid digit nibble */

/* Paging message limits */
#define RDS_PAGING_ADDR_MAX     999999  /* Maximum 6-digit address */
#define RDS_PAGING_NUM10_DIGITS 10      /* 10-digit numeric message */
#define RDS_PAGING_NUM18_DIGITS 18      /* 18-digit numeric message */
#define RDS_PAGING_ALPHA_MAX    80      /* Maximum alphanumeric chars */
#define RDS_PAGING_ALPHA_PER_GROUP 4    /* Characters per alpha data group */
#define RDS_PAGING_MAX_GROUPS   22      /* Max groups for 80-char alpha */
#define RDS_PAGING_QUEUE_MAX    16      /* Maximum queued messages */
#define RDS_PAGING_DEFAULT_REPEATS   2  /* Default retransmission count */
#define RDS_PAGING_DEFAULT_INTERVAL  5  /* Default repeat interval (seconds) */
#define RDS_PAGING_DEFAULT_TIMEOUT  30  /* Default reassembly timeout (seconds) */
#define RDS_PAGING_DEFAULT_RPC       4  /* Default RPC (groups 00-99, sync 00) */

/* ============================================================
 * Group 13A Bit Fields (EN 50067 Annex M / IEC 62106)
 * Enhanced Radio Paging (or ODA when paging not enabled)
 *
 * Block B payload (bits 4-0):
 *   Bits 4-2: STY (Sub-Type, 000 for address notification)
 *   Bits 1-0: Reserved
 *
 * Block C (sub-type 000):
 *   Bits 15-14: CS (Cycle Selection)
 *   Bits 13-10: IT (Interval Number, 0-9)
 *   Bits 9-0:   Notification bits 24-15
 *
 * Block D (sub-type 000):
 *   Bits 15-1:  Notification bits 14-0
 *   Bit  0:     S1 (Sort indicator)
 * ============================================================ */
#define RDS_13A_STY_SHIFT       2
#define RDS_13A_STY_MASK        0x001C  /* Bits 4-2: Sub-type */
#define RDS_13A_CS_SHIFT        14
#define RDS_13A_CS_MASK         0xC000  /* Block C bits 15-14 */
#define RDS_13A_IT_SHIFT        10
#define RDS_13A_IT_MASK         0x3C00  /* Block C bits 13-10 */
#define RDS_13A_NOTIFY_HI_MASK  0x03FF  /* Block C bits 9-0: notify 24-15 */
#define RDS_13A_NOTIFY_LO_SHIFT 1
#define RDS_13A_NOTIFY_LO_MASK  0xFFFE  /* Block D bits 15-1: notify 14-0 */
#define RDS_13A_S1_MASK         0x0001  /* Block D bit 0: sort indicator */

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

/* Group 3B - ODA-only (IEC 62106 S6.1.5.5)
 * Group 3B has no standard meaning; entirely ODA-application dependent.
 * Block C contains PI repeat (version B standard behavior).
 * Block D contains ODA-specific payload. */
#define RDS_3B_PAYLOAD_MASK     0x001F  /* Block B bits 4-0 (ODA-specific) */

/* ============================================================
 * Open Data Application (ODA) Identification Codes
 * EBU/RDS Forum ODA Registry - Common Applications
 * ============================================================ */
#define RDS_ODA_AID_DAB_XREF    0x0093  /* DAB Cross-Reference */
#define RDS_ODA_AID_RT_PLUS     0x4BD7  /* RadioText+ (RT+) */
#define RDS_ODA_AID_ERT_PLUS    0x4BD8  /* RT+ for Enhanced RadioText */
#define RDS_ODA_AID_ERT         0x6552  /* Enhanced RadioText (eRT) */
#define RDS_ODA_AID_TMC_ALERT   0xCD46  /* RDS-TMC Alert-C */
#define RDS_ODA_AID_TMC_ALERT2  0xCD47  /* RDS-TMC Alert-C (alt) */
#define RDS_ODA_AID_STATION_LOGO 0xFF7F /* RFT: Station logo */

/* Maximum simultaneous ODA configurations */
#define RDS_ODA_MAX_CONFIGS     8

/* ODA Configuration Entry (Group 3A transmission)
 * Supports multiple simultaneous ODAs (e.g., RT+ + TMC + eRT)
 * Each ODA maps to a carrier group type and gets cycled 3A announcements */
typedef struct {
	uint8_t  carrier_group;   /* Which group carries ODA (0-31: 0A-15B) */
	uint16_t aid;             /* Application ID (16-bit, e.g. 0x4BD7 = RT+) */
	uint16_t message;         /* ODA-specific message (Block C) */
	uint8_t  enabled;         /* 1 = transmit 3A for this ODA */
} rds_oda_config_t;

/* ODA Application Entry - Decoder tracking (from Group 3A)
 * Maps group types to registered ODA applications */
typedef struct {
	uint8_t  carrier_group;   /* Group type carrying this ODA (0-31) */
	uint16_t aid;             /* Registered Application ID */
	uint16_t message;         /* Last received ODA message (Block C) */
	uint8_t  registered;      /* 1 if registered via 3A, 0 if empty */
	time_t   timestamp;       /* When registered */
} rds_oda_app_t;


/* ============================================================
 * RT+ (RadioText Plus) - IEC 62106-6
 * Content-type tagging for RadioText and Enhanced RadioText
 * ============================================================
 *
 * RT+ allows marking semantic segments within RadioText (e.g., artist,
 * title, URL). Announced via Group 3A, carried on a configurable ODA
 * group (typically 11A).
 *
 * Each RT+ group can contain up to 2 tags. Each tag specifies:
 *   - content_type: What the segment represents (0-63)
 *   - start: Start position in RadioText (0-63)
 *   - length: Length of segment (1-64)
 *
 * RT+ also operates on eRT (Enhanced RadioText) via AID 0x4BD8.
 *
 * RT+ Bit Fields (IEC 62106-6):
 * Block B bits 4-0:
 *   bit 4: item_toggle
 *   bit 3: item_running
 *   bits 2-0: tag1 content_type[5:3] (high 3 bits)
 * Block C:
 *   bits 15-13: tag1 content_type[2:0] (low 3 bits)
 *   bits 12-7: tag1 start (6 bits, 0-63)
 *   bits 6-1: tag1 length-1 (6 bits, 0-63 -> length 1-64)
 *   bit 0: tag2 content_type[5] (high 1 bit, if tag2 exists)
 * Block D:
 *   bits 15-11: tag2 content_type[4:0] (low 5 bits, if tag2 exists)
 *   bits 10-5: tag2 start (6 bits, 0-63)
 *   bits 4-0: tag2 length-1 (5 bits, 0-31 -> length 1-32)
 * ============================================================ */
#define RDS_RTPLUS_TOGGLE_BIT         4       /* Block B bit 4: item toggle */
#define RDS_RTPLUS_TOGGLE_MASK        0x0010
#define RDS_RTPLUS_ITEM_RUNNING_BIT   3       /* Block B bit 3: item running */
#define RDS_RTPLUS_ITEM_RUNNING_MASK  0x0008
#define RDS_RTPLUS_TAG1_CT_HIGH_BITS  0       /* Block B bits 2-0: tag1 content_type[5:3] */
#define RDS_RTPLUS_TAG1_CT_HIGH_MASK  0x0007
#define RDS_RTPLUS_TAG1_CT_HIGH_SHIFT 3       /* Shift for high 3 bits */
#define RDS_RTPLUS_TAG1_CT_LOW_BITS   13      /* Block C bits 15-13: tag1 content_type[2:0] */
#define RDS_RTPLUS_TAG1_CT_LOW_MASK   0xE000
#define RDS_RTPLUS_TAG1_CT_LOW_SHIFT  13
#define RDS_RTPLUS_TAG1_START_BITS    7       /* Block C bits 12-7: tag1 start */
#define RDS_RTPLUS_TAG1_START_MASK    0x1FC0
#define RDS_RTPLUS_TAG1_START_SHIFT   7
#define RDS_RTPLUS_TAG1_LEN_BITS      1       /* Block C bits 6-1: tag1 length-1 */
#define RDS_RTPLUS_TAG1_LEN_MASK      0x003E
#define RDS_RTPLUS_TAG1_LEN_SHIFT     1
#define RDS_RTPLUS_TAG2_CT_HIGH_BIT   0       /* Block C bit 0: tag2 content_type[5] */
#define RDS_RTPLUS_TAG2_CT_HIGH_MASK  0x0001  /* 1 bit */
#define RDS_RTPLUS_TAG2_CT_HIGH_SHIFT 5       /* Shift for high bit */
#define RDS_RTPLUS_TAG2_CT_LOW_BITS   11      /* Block D bits 15-11: tag2 content_type[4:0] */
#define RDS_RTPLUS_TAG2_CT_LOW_MASK   0xF800  /* 5 bits */
#define RDS_RTPLUS_TAG2_CT_LOW_SHIFT  11
#define RDS_RTPLUS_TAG2_START_BITS    6       /* Block D bits 10-5: tag2 start */
#define RDS_RTPLUS_TAG2_START_MASK    0x07E0
#define RDS_RTPLUS_TAG2_START_SHIFT   5
#define RDS_RTPLUS_TAG2_LEN_BITS      5       /* Block D bits 4-0: tag2 length-1 */
#define RDS_RTPLUS_TAG2_LEN_MASK      0x001F
#define RDS_RTPLUS_TAG2_LEN_SHIFT     0
#define RDS_RTPLUS_CONTENT_TYPE_BITS  6       /* Total bits for content_type */
#define RDS_RTPLUS_MAX_START          63      /* Maximum start position */
#define RDS_RTPLUS_MAX_LEN_TAG1       64      /* Maximum length for tag1 */
#define RDS_RTPLUS_MAX_LEN_TAG2       32      /* Maximum length for tag2 */
#define RDS_RTPLUS_MAX_TAGS           2       /* Maximum tags per group */

/* RT+ Content Type Constants (RDS Forum R06/040_1)
 * These match the indices in rds_tables.c rtplus_content_types[] array */
#define RDS_RTPLUS_CT_DUMMY           0       /* dummy_class */
#define RDS_RTPLUS_CT_ITEM_TITLE      1       /* item.title */
#define RDS_RTPLUS_CT_ITEM_ALBUM      2       /* item.album */
#define RDS_RTPLUS_CT_ITEM_TRACKNUM   3       /* item.tracknumber */
#define RDS_RTPLUS_CT_ITEM_ARTIST     4       /* item.artist */
#define RDS_RTPLUS_CT_ITEM_COMPOSITION 5      /* item.composition */
#define RDS_RTPLUS_CT_ITEM_MOVEMENT   6       /* item.movement */
#define RDS_RTPLUS_CT_ITEM_CONDUCTOR  7       /* item.conductor */
#define RDS_RTPLUS_CT_ITEM_COMPOSER   8       /* item.composer */
#define RDS_RTPLUS_CT_ITEM_BAND       9       /* item.band */
#define RDS_RTPLUS_CT_ITEM_COMMENT    10      /* item.comment */
#define RDS_RTPLUS_CT_ITEM_GENRE      11      /* item.genre */
#define RDS_RTPLUS_CT_INFO_NEWS       12      /* info.news */
#define RDS_RTPLUS_CT_INFO_NEWS_LOCAL 13      /* info.news.local */
#define RDS_RTPLUS_CT_INFO_STOCKMARKET 14     /* info.stockmarket */
#define RDS_RTPLUS_CT_INFO_SPORT      15      /* info.sport */
#define RDS_RTPLUS_CT_INFO_LOTTERY    16      /* info.lottery */
#define RDS_RTPLUS_CT_INFO_HOROSCOPE  17      /* info.horoscope */
#define RDS_RTPLUS_CT_INFO_DAILY_DIV   18     /* info.daily_diversion */
#define RDS_RTPLUS_CT_INFO_HEALTH      19     /* info.health */
#define RDS_RTPLUS_CT_INFO_EVENT       20     /* info.event */
#define RDS_RTPLUS_CT_INFO_SCENE       21     /* info.scene */
#define RDS_RTPLUS_CT_INFO_CINEMA      22     /* info.cinema */
#define RDS_RTPLUS_CT_INFO_TV          23     /* info.tv */
#define RDS_RTPLUS_CT_INFO_DATE_TIME   24     /* info.date_time */
#define RDS_RTPLUS_CT_INFO_WEATHER     25     /* info.weather */
#define RDS_RTPLUS_CT_INFO_TRAFFIC     26     /* info.traffic */
#define RDS_RTPLUS_CT_INFO_ALARM       27     /* info.alarm */
#define RDS_RTPLUS_CT_INFO_AD          28     /* info.advertisement */
#define RDS_RTPLUS_CT_INFO_URL         29     /* info.url */
#define RDS_RTPLUS_CT_INFO_OTHER       30     /* info.other */
#define RDS_RTPLUS_CT_STATIONNAME_SHORT 31    /* stationname.short */
#define RDS_RTPLUS_CT_STATIONNAME_LONG  32    /* stationname.long */
#define RDS_RTPLUS_CT_PROGRAMME_NOW     33    /* programme.now */
#define RDS_RTPLUS_CT_PROGRAMME_NEXT    34    /* programme.next */
#define RDS_RTPLUS_CT_PROGRAMME_PART    35    /* programme.part */
#define RDS_RTPLUS_CT_PROGRAMME_HOST    36    /* programme.host */
#define RDS_RTPLUS_CT_PROGRAMME_EDITORIAL 37  /* programme.editorial_staff */
#define RDS_RTPLUS_CT_PROGRAMME_FREQ     38   /* programme.frequency */
#define RDS_RTPLUS_CT_PROGRAMME_HOMEPAGE  39  /* programme.homepage */
#define RDS_RTPLUS_CT_PROGRAMME_SUBCHAN   40  /* programme.subchannel */
#define RDS_RTPLUS_CT_PHONE_HOTLINE       41  /* phone.hotline */
#define RDS_RTPLUS_CT_PHONE_STUDIO        42  /* phone.studio */
#define RDS_RTPLUS_CT_PHONE_OTHER         43  /* phone.other */
#define RDS_RTPLUS_CT_SMS_STUDIO          44  /* sms.studio */
#define RDS_RTPLUS_CT_SMS_OTHER           45  /* sms.other */
#define RDS_RTPLUS_CT_EMAIL_HOTLINE       46  /* email.hotline */
#define RDS_RTPLUS_CT_EMAIL_STUDIO        47  /* email.studio */
#define RDS_RTPLUS_CT_EMAIL_OTHER         48  /* email.other */
#define RDS_RTPLUS_CT_MMS_OTHER            49  /* mms.other */
#define RDS_RTPLUS_CT_CHAT                 50  /* chat */
#define RDS_RTPLUS_CT_CHAT_CENTRE          51  /* chat.centre */
#define RDS_RTPLUS_CT_VOTE_QUESTION        52  /* vote.question */
#define RDS_RTPLUS_CT_VOTE_CENTRE          53  /* vote.centre */
#define RDS_RTPLUS_CT_PLACE                59  /* place (Descriptor) */
#define RDS_RTPLUS_CT_APPOINTMENT          60  /* appointment (Descriptor) */
#define RDS_RTPLUS_CT_IDENTIFIER           61  /* identifier (Descriptor) */
#define RDS_RTPLUS_CT_PURCHASE             62  /* purchase (Descriptor) */
#define RDS_RTPLUS_CT_GET_DATA             63  /* get_data (Descriptor) */

/* RT+ Tag structure - identifies a substring in RadioText or eRT */
typedef struct {
	uint8_t  content_type;  /* Content type (0-63, see rds_tables.h) */
	uint8_t  start;         /* Start position in RT/eRT (0-63 for RT, 0-127 for eRT) */
	uint8_t  length;        /* Length of tag (1-64 for RT, 1-128 for eRT) */
} rds_rtplus_tag_t;

/* RT+ Encoder Configuration */
typedef struct {
	uint8_t  carrier_group;     /* ODA carrier group (e.g., 22 for 11A) */
	uint8_t  cb;                /* Class/Type flag (from 3A message bit 12) */
	uint8_t  scb;               /* Server Control Bits (4 bits, from 3A message bits 11-8) */
	uint8_t  template_num;      /* Template number (8 bits, from 3A message bits 7-0) */
	uint8_t  toggle;            /* Item toggle (changes when tags change) */
	uint8_t  item_running;      /* Item running flag */
	rds_rtplus_tag_t tags[2];   /* Up to 2 tags per group */
	uint8_t  tag_count;         /* Number of valid tags (0-2) */
} rds_rtplus_encoder_t;

/* RT+ Decoder State */
typedef struct {
	uint8_t  carrier_group;     /* ODA carrier group */
	uint8_t  cb;                /* Class/Type flag from 3A */
	uint8_t  scb;               /* Server Control Bits from 3A */
	uint8_t  template_num;      /* Template number from 3A */
	uint8_t  toggle;            /* Current item toggle */
	uint8_t  item_running;      /* Item running flag */
	rds_rtplus_tag_t tags[2];   /* Last received tags */
	uint8_t  tag_count;         /* Number of valid tags */
	uint8_t  registered;        /* 1 if RT+ ODA registered via 3A */
	time_t   timestamp;         /* When last updated */
} rds_rtplus_decoder_t;


/* ============================================================
 * eRT (Enhanced RadioText) - RDS2 / IEC 62106-6
 * 128-byte RadioText with UTF-8/UCS-2 encoding
 * ============================================================
 *
 * eRT extends standard RadioText from 64 to 128 bytes and adds
 * support for UTF-8 and UCS-2 encoding.
 * Announced via Group 3A, carried on a configurable ODA group.
 *
 * Buffer Size: 128 bytes maximum (32 segments x 4 bytes)
 *
 * Message Termination:
 *   - eRT messages are terminated by a carriage return (CR, 0x0D) byte
 *   - This indicates the end of the text string
 *   - Receivers use this CR to determine where the displayable message ends
 *
 * Padding for Shorter Messages:
 *   - If the text (including the terminating CR) is shorter than 128 bytes,
 *     the remaining bytes are padded with space characters (0x20)
 *   - This is analogous to classic RadioText (RT), where short messages
 *     are space-padded
 *
 * Configuration via Group 3A message bits (IEC 62106-6):
 *   Bit 0 (b0): encoding (0=UCS-2, 1=UTF-8) on stream 0; always 1 on streams 1-3
 *   Bit 1 (b1): direction (0=LTR, always 0 per spec)
 *   Bits 5-2 (b2-b5): chartable (0=E3, others reserved) - must be 0 for backwards compatibility
 *   Bits 15-6 (b6-b15): RFU (Reserved for Future Use) - must be 0
 *
 * eRT transmission: 32 segments x 4 bytes = 128 bytes total
 *
 * eRT Bit Fields:
 * Block B bits 4-0: segment address (0-31)
 * Block C: eRT bytes [segment*4] and [segment*4+1]
 * Block D: eRT bytes [segment*4+2] and [segment*4+3]
 *
 * UTF-8 Encoding Considerations:
 *   - UTF-8 is variable-length encoding (1-4 bytes per character)
 *   - Multi-byte UTF-8 characters MUST NOT be split across the 128-byte boundary
 *   - Encoder validates UTF-8 and truncates at character boundaries
 *   - Decoder uses sanitize_utf8() to replace invalid/incomplete sequences with '?'
 *     for display (does not affect stored string)
 *
 * eRT+ Tags:
 *   - eRT+ tags address CHARACTER position in string, not byte position!
 *   - This is critical for UTF-8 where multi-byte characters exist
 *   - Example: "Cafe" = 4 characters, 5 bytes (e = 2 bytes UTF-8)
 *     Tag with start=3, length=1 should extract "e" (2 bytes), not byte 3
 *   - eRT+ tags can address the full 128 characters of eRT text
 *   - Note: RT+ is limited to 64 characters for standard RT and combination of RT+ with eRT (without eRT+)
 * ============================================================ */
#define RDS_ERT_SEGMENT_MASK          0x001F  /* Block B bits 4-0: segment (0-31) */
#define RDS_ERT_SEGMENTS              32      /* Total segments (32 x 4 = 128 bytes) */
#define RDS_ERT_BYTES_PER_SEGMENT     4       /* Bytes per segment */
#define RDS_ERT_3A_ENCODING_BIT       0       /* Group 3A message bit 0: encoding */
#define RDS_ERT_3A_ENCODING_MASK      0x0001
#define RDS_ERT_3A_DIRECTION_BIT      1       /* Group 3A message bit 1: direction */
#define RDS_ERT_3A_DIRECTION_MASK     0x0002
#define RDS_ERT_3A_CHARTABLE_BITS     2       /* Group 3A message bits 5-2: chartable */
#define RDS_ERT_3A_CHARTABLE_MASK     0x003C
#define RDS_ERT_3A_CHARTABLE_SHIFT    2
/* Bits 6-15 are RFU (Reserved for Future Use) and must be set to 0 */
#define RDS_ERT_3A_RFU_MASK           0xFFC0  /* Bits 15-6: RFU (must be 0) */

/* ============================================================
 * RT+ Group 3A Message Bits (EBU Technical Review, July 2006)
 *
 * Block C of Group 3A carries ODA-specific control data.
 * For RT+ (AID=0x4BD7), the 16-bit message layout is:
 *
 *   b15 b14 b13  b12   b11 b10 b9 b8   b7 b6 b5 b4 b3 b2 b1 b0
 *   rfu rfu rfu  CB    --- SCB ------   ------ Template number --
 *
 *   CB flag (b12):     0 = no template available
 *                      1 = template available for ongoing programme
 *   SCB (b11-b8):      Server Control Bits - distinguishes programmes
 *                      sharing the same PI code (e.g. regional stations)
 *   Template (b7-b0):  Template number for receiver display layout
 *                      (only meaningful when CB=1)
 *   rfu (b15-b13):     Reserved, must be 0
 *
 * Standard value: 0x0000 (no template, no SCB, all rfu=0)
 * This matches all known real-world RT+ implementations (KCRC, etc.)
 * ============================================================ */
#define RDS_RTPLUS_3A_CB_BIT          12      /* Group 3A message bit 12: CB flag */
#define RDS_RTPLUS_3A_CB_MASK         0x1000
#define RDS_RTPLUS_3A_SCB_BITS        8       /* Group 3A message bits 11-8: SCB */
#define RDS_RTPLUS_3A_SCB_MASK        0x0F00
#define RDS_RTPLUS_3A_SCB_SHIFT       8
#define RDS_RTPLUS_3A_TEMPLATE_BITS   0       /* Group 3A message bits 7-0: template number */
#define RDS_RTPLUS_3A_TEMPLATE_MASK   0x00FF

/* eRT Group 3A Message Construction Helpers
 * According to IEC 62106-6:
 *   Bit 0 (b0): encoding (0=UCS-2, 1=UTF-8) on stream 0; always 1 on streams 1-3
 *   Bit 1 (b1): direction (0=LTR, always 0 per spec)
 *   Bits 5-2 (b2-b5): chartable (0=E3, others reserved) - must be 0 for backwards compatibility
 *   Bits 15-6 (b6-b15): RFU - must be 0
 */
#define RDS_ERT_3A_MSG(encoding, direction, chartable) \
	((((encoding) << RDS_ERT_3A_ENCODING_BIT) | \
	  ((direction) << RDS_ERT_3A_DIRECTION_BIT) | \
	  ((chartable) << RDS_ERT_3A_CHARTABLE_SHIFT)) & ~RDS_ERT_3A_RFU_MASK)

/* Common eRT message values */
#define RDS_ERT_3A_MSG_UTF8_LTR_E3    RDS_ERT_3A_MSG(RDS_ERT_ENCODING_UTF8, 0, 0)  /* UTF-8, LTR, E3 chartable */
#define RDS_ERT_3A_MSG_UCS2_LTR_E3    RDS_ERT_3A_MSG(RDS_ERT_ENCODING_UCS2, 0, 0)  /* UCS-2, LTR, E3 chartable */
#define RDS_ERT_3A_MSG_UTF8_RTL_E3    RDS_ERT_3A_MSG(RDS_ERT_ENCODING_UTF8, 1, 0)  /* UTF-8, RTL, E3 chartable */
#define RDS_ERT_3A_MSG_UCS2_RTL_E3    RDS_ERT_3A_MSG(RDS_ERT_ENCODING_UCS2, 1, 0)  /* UCS-2, RTL, E3 chartable */

#define RDS_ERT_LENGTH          128     /* eRT: 128 bytes maximum */
#define RDS_ERT_CR_TERMINATOR   0x0D    /* Carriage return: message terminator */
#define RDS_ERT_PADDING_BYTE    0x20    /* Space character: padding for shorter messages */

/* eRT Chartable values (bits 5-2 in Group 3A message) */
#define RDS_ERT_CHARTABLE_E3    0       /* Chartable E3 (default, for backwards compatibility) */
#define RDS_ERT_CHARTABLE_MAX   15      /* Maximum chartable value (4 bits: 0-15) */
#define RDS_ERT_CHARTABLE_DEFAULT RDS_ERT_CHARTABLE_E3  /* Default chartable value */

/* eRT Text Encoding (from Group 3A message bit 0) */
typedef enum {
	RDS_ERT_ENCODING_UCS2 = 0,  /* UCS-2 (16-bit Unicode) */
	RDS_ERT_ENCODING_UTF8 = 1   /* UTF-8 (variable-length) */
} rds_ert_encoding_t;

/* eRT Text Direction (from Group 3A message bit 1) */
typedef enum {
	RDS_ERT_DIR_LTR = 0,        /* Left-to-right */
	RDS_ERT_DIR_RTL = 1         /* Right-to-left (Arabic, Hebrew) */
} rds_ert_direction_t;

/* eRT Encoder Configuration */
typedef struct {
	uint8_t  carrier_group;     /* ODA carrier group for eRT data */
	uint8_t  encoding;          /* rds_ert_encoding_t */
	uint8_t  direction;         /* rds_ert_direction_t */
	uint8_t  chartable;  /* Raw chartable value (RDS_ERT_CHARTABLE_E3 to RDS_ERT_CHARTABLE_MAX) as received. Default: RDS_ERT_CHARTABLE_DEFAULT. Not used for character mapping for now. */
	uint8_t  ab;                /* A/B flag - toggled on content change */
	uint8_t  ert[RDS_ERT_LENGTH + 1];  /* 128 bytes + NUL */
	uint8_t  length;            /* Current length (includes CR terminator if present) */
	uint8_t  segment;           /* Current segment (0-31) */
} rds_ert_encoder_t;

/* eRT Decoder State */
typedef struct {
	uint8_t  carrier_group;     /* ODA carrier group */
	uint8_t  encoding;          /* rds_ert_encoding_t */
	uint8_t  direction;         /* rds_ert_direction_t */
	uint8_t  chartable;  /* Raw chartable value (RDS_ERT_CHARTABLE_E3 to RDS_ERT_CHARTABLE_MAX) as received. Default: RDS_ERT_CHARTABLE_DEFAULT. Not used for character mapping for now. */
	uint8_t  ert[RDS_ERT_LENGTH + 1];  /* 128 bytes + NUL */
	uint8_t  ert_status[RDS_ERT_LENGTH];  /* Per-byte decode status */
	uint32_t segments_received; /* Bitmask of 32 segments */
	uint8_t  registered;        /* 1 if eRT ODA registered via 3A */
	time_t   timestamp;         /* When last updated */
} rds_ert_decoder_t;


/* ============================================================
 * Group 14A/14B Bit Fields (IEC 62106 S6.1.5.14)

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
 * RDS PAGING TYPES (EN 50067 Annex M)
 * ============================================================ */

/* Paging message types */
enum rds_paging_msg_type {
	RDS_PAGING_TONE  = 0,	/* Tone-only (1 group) */
	RDS_PAGING_NUM10 = 1,	/* 10-digit numeric (2 groups) */
	RDS_PAGING_NUM18 = 2,	/* 18-digit numeric (3 groups) */
	RDS_PAGING_ALPHA = 3,	/* Alphanumeric up to 80 chars */
};

/* Queued paging message */
typedef struct rds_paging_msg {
	struct rds_paging_msg	*next;
	enum rds_paging_msg_type type;
	uint32_t	address;	/* 6-digit BCD address (0-999999) */
	char		data[81];	/* Message content (max 80 + NUL) */
	int		data_len;	/* Length of message data */
	int		repeats_left;	/* Remaining retransmissions */
	int		repeat_interval;/* Seconds between retransmissions */
	time_t		next_send_time;	/* Earliest time for next transmission */
} rds_paging_msg_t;

/* Paging encoder state - embedded in rds_encoder_t */
typedef struct rds_paging_enc {
	int		enabled;	/* Paging enabled flag */
	int		enhanced;	/* Enhanced paging (13A) enabled */
	uint8_t		rpc;		/* Radio Paging Code (0-31) for Group 1A */
	uint8_t		ab_flag;	/* Current A/B flag (toggles per new call) */

	/* Current transmission state */
	int		tx_active;	/* Currently transmitting a message */
	rds_paging_msg_t *tx_msg;	/* Message being transmitted (owned copy) */
	int		tx_group_idx;	/* Current group index within message */
	int		tx_total_groups;/* Total groups needed for current message */

	/* Pre-computed group data for current message */
	uint16_t	tx_blocks_c[RDS_PAGING_MAX_GROUPS];
	uint16_t	tx_blocks_d[RDS_PAGING_MAX_GROUPS];
	uint8_t		tx_psac_seq[RDS_PAGING_MAX_GROUPS];

	/* Message queue */
	rds_paging_msg_t *queue_head;
	rds_paging_msg_t *queue_tail;
	int		queue_count;

	/* 13A enhanced paging state */
	uint32_t	notify_bits;	/* Address notification bits (25 bits) */
	uint8_t		cycle_selection;/* CS field (0-3) */
} rds_paging_enc_t;

/* Paging decoder state - embedded in rds_decoder_t */
typedef struct rds_paging_dec {
	/* Reassembly state */
	int		assembling;	/* Currently assembling a multi-group message */
	enum rds_paging_msg_type msg_type;
	uint32_t	address;	/* Decoded address */
	uint8_t		ab_flag;	/* Last seen A/B flag */
	int		expected_psac;	/* Next expected PSAC value */
	char		msg_buf[81];	/* Reassembly buffer */
	int		msg_len;	/* Current length in buffer */
	time_t		assembly_start;	/* Timestamp when assembly began */
	int		timeout_sec;	/* Configurable reassembly timeout */

	/* Last complete message (for display/API) */
	int		msg_valid;	/* 1 if a complete message is available */
	enum rds_paging_msg_type last_type;
	uint32_t	last_address;
	char		last_msg[81];
	int		last_msg_len;

	/* Group 1A RPC state */
	uint8_t		rpc;		/* Last received Radio Paging Code */
	uint8_t		rpc_group_desig;/* Decoded group designation (bits 4-2) */
	uint8_t		rpc_batt_sync;	/* Battery saving sync (bits 1-0) */
	uint8_t		rpc_valid;	/* RPC received at least once */

	/* Group 13A enhanced paging state */
	uint32_t	notify_bits;	/* Address notification bits (25 bits) */
	uint8_t		notify_sty;	/* Last STY sub-type */
	uint8_t		cycle_selection;/* CS field */
	uint8_t		interval_num;	/* Current interval number (0-9) */
	uint8_t		enhanced_valid;	/* 13A data received at least once */
} rds_paging_dec_t;

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
	
	/* Group 3A: Open Data Application (ODA) TX Configuration
	 * Supports multiple simultaneous ODAs with cycling 3A announcements */
	rds_oda_config_t oda[RDS_ODA_MAX_CONFIGS];
	int		oda_count;		/* Number of configured ODAs */
	int		oda_cycle_idx;		/* Current index for 3A cycling */
	
	/* RT+ (RadioText Plus) - Group 11A (or configurable)
	 * Content-type tagging for RT and eRT */
	rds_rtplus_encoder_t rtplus;		/* RT+ for standard RadioText */
	rds_rtplus_encoder_t ert_plus;		/* RT+ for Enhanced RadioText */
	
	/* eRT (Enhanced RadioText) - 128-byte UTF-8/UCS-2 text */
	rds_ert_encoder_t ert;			/* eRT encoder configuration */
	
	/* Radio Paging (EN 50067 Annex M) - Group 7A, 1A RPC, 13A */
	rds_paging_enc_t paging;		/* Paging encoder state */
	
	/* Fixed Group Sequence Scheduler */
	#define RDS_SCHEDULER_MAX_LEN 64
	enum rds_group_type group_sched_buffer[RDS_SCHEDULER_MAX_LEN];
	int			group_sched_len;
	int			group_sched_index;
	
	/* ============================================================
	 * Dynamic PS (Scrolling Programme Service Name)
	 * ============================================================
	 *
	 * HISTORICAL CONTEXT:
	 *   Scrolling PS was never part of the RDS standard (IEC 62106).
	 *   It was a de facto practice used by European commercial broadcasters
	 *   from the late 1990s onward. Broadcasters exploited the fact that
	 *   receivers simply display whatever 8-character PS string they last
	 *   decoded, so by rapidly overwriting PS with shifted windows of a
	 *   longer message, the display appeared to scroll.
	 *
	 *   The EBU and ITU have repeatedly discouraged this practice because:
	 *   - It violates the PS field's intended purpose (static station ID)
	 *   - It causes display flicker on receivers with PS stability timers
	 *   - It can confuse station preset memory on car radios
	 *   - RadioText (RT) is the proper field for dynamic messages
	 *
	 *   Nevertheless, dynamic PS remains widely used in practice.
	 *
	 * SCROLLING PATTERNS SUPPORTED:
	 *
	 *   RDS_DPS_SCROLL (Window shift / character scroll):
	 *     The de facto standard. Text is padded with spaces and shifted
	 *     one character at a time through an 8-character window.
	 *     Example for "NOW PLAYING SONG":
	 *       "  NOW PL" -> " NOW PLA" -> "NOW PLAY" -> "OW PLAYI" -> ...
	 *     Used heavily by European commercial stations.
	 *     Step size: 1 character. Recommended update: 500-1000ms.
	 *
	 *   RDS_DPS_WORD (Word-step scrolling):
	 *     Instead of character-by-character scrolling, full words are
	 *     swapped. Less flicker on slow receivers.
	 *     Example: "NOW PLAY" -> "PLAYING " -> "SONG    "
	 *     Popular on German and Nordic broadcasts.
	 *     Recommended update: 1000-2000ms.
	 *
	 *   RDS_DPS_PAGE (Paging / alternating messages):
	 *     Two or more fixed 8-char PS strings alternate.
	 *     Example: "RADIO 1 " -> "HOT HITS" -> "RADIO 1 " -> ...
	 *     Technically not scrolling. Considered less abusive by regulators.
	 *     Recommended update: 2000-4000ms.
	 *
	 * TIMING AND RECEIVER COMPATIBILITY:
	 *
	 *   The update_interval_ms parameter controls how often the PS window
	 *   advances. This is measured in COMPLETED PS TRANSMISSIONS, not
	 *   wall-clock time, because:
	 *
	 *   1. Each PS transmission requires 4 Group 0 transmissions (segments
	 *      0-3), each carrying 2 characters. A full PS = 4 groups.
	 *
	 *   2. At 1187.5 bps with 104 bits/group = ~11.4 groups/second.
	 *      With ~50% Group 0 allocation, that's ~5.7 Group 0/sec.
	 *      So one full PS cycle takes ~0.7 seconds minimum.
	 *
	 *   3. Many real-world encoders repeated each PS value 2-4 times
	 *      before advancing, to ensure receivers with PS stability
	 *      timers (1-2 seconds) could latch the value.
	 *
	 *   The ps_dyn_repeat parameter controls how many complete PS
	 *   transmissions (4 segments each) occur before advancing to the
	 *   next scroll position. Recommended: 2-4 repeats.
	 *
	 *   MINIMUM TIMING (ps_dyn_repeat=1):
	 *     ~0.7s per step. This is the absolute fastest and will cause
	 *     flicker/missed updates on many receivers. Use only for testing.
	 *
	 *   RECOMMENDED TIMING (ps_dyn_repeat=3):
	 *     ~2.1s per step. Compatible with most receivers including
	 *     early Philips/Blaupunkt tuners and automotive head units
	 *     with PS debounce filters.
	 *
	 *   CONSERVATIVE TIMING (ps_dyn_repeat=5):
	 *     ~3.5s per step. Safe for all known receivers.
	 *
	 * RECEIVER-SIDE BEHAVIOR THAT SHAPED THESE PATTERNS:
	 *   - PS stability timer (1-2s): Some radios only update PS after
	 *     two identical full decodes
	 *   - Some radios blank PS if it changes too often
	 *   - Car radios often cache the first stable PS and ignore later
	 *     updates until signal loss
	 *   - This is why broadcasters repeated each PS value many times,
	 *     avoided fast scrolling, and used trailing spaces instead of
	 *     wraparound jumps
	 * ============================================================ */
	
	/* Dynamic PS state */
	int		ps_dyn_enabled;		/* 1 if dynamic PS is active */
	int		ps_dyn_mode;		/* RDS_DPS_SCROLL, _WORD, or _PAGE */
	char		ps_dyn_text[256];	/* Full message text (RDS-encoded) */
	int		ps_dyn_text_len;	/* Length of full message */
	int		ps_dyn_scroll_pos;	/* Current scroll position (char offset) */
	int		ps_dyn_repeat;		/* Repeats per PS value before advancing */
	int		ps_dyn_repeat_count;	/* Current repeat counter */
	int		ps_dyn_ps_cycles;	/* Completed PS cycles (4 segments = 1 cycle) */
	char		ps_dyn_static_ps[9];	/* Saved static PS for restore on stop */
	char		ps_dyn_delimiter;	/* Page delimiter char (default '|', 0 = '|') */
	
	/* Word-step mode: pre-computed word boundaries */
	int		ps_dyn_word_offsets[32];/* Start offset of each 8-char word page */
	int		ps_dyn_word_count;	/* Number of word pages */
	int		ps_dyn_word_index;	/* Current word page index */
	
	/* Page mode: pre-computed page list */
	char		ps_dyn_pages[32][9];	/* Up to 32 alternating 8-char pages */
	int		ps_dyn_page_count;	/* Number of pages */
	int		ps_dyn_page_index;	/* Current page index */
	
} rds_encoder_t;

/* Initialize RDS encoder */
int rds_encoder_init(rds_encoder_t *rds, double samplerate, uint16_t pi, 
		     const char *ps, const char *rt, uint8_t pty, const char *ptyn);

/* Generate RDS samples to add to FM baseband (phase-locked to pilot) */
void rds_encoder_process(rds_encoder_t *rds, sample_t *samples, int num,
			 double pilot_phase, double pilot_phasestep);

/* Set RadioText (can be changed dynamically) - converts UTF-8 to RDS charset */
void rds_enc_set_radiotext(rds_encoder_t *rds, const char *rt);

/* Set Traffic Announcement flag (1=enabled, 0=disabled) */
void rds_enc_set_ta(rds_encoder_t *rds, int ta);

/* ODA (Open Data Application) Configuration API
 * Group 3A announces ODAs; each ODA needs a carrier group for its data.
 *
 * carrier_group: Which group carries ODA data (0-31: 0A=0, 0B=1, ..., 15B=31)
 *                Common: 8A=16 (TMC), 11A=22 (RT+), 12A=24
 * aid:           Application ID from EBU ODA Registry (e.g., 0x4BD7 = RT+)
 * message:       ODA-specific 16-bit message for Group 3A Block C
 */
/* Add ODA configuration (returns 0 on success, -1 on error) */
int rds_enc_oda_add(rds_encoder_t *rds, uint8_t carrier_group, uint16_t aid, uint16_t message);
/* Remove ODA by AID (returns 0 on success, -1 if not found) */
int rds_enc_oda_remove(rds_encoder_t *rds, uint16_t aid);
/* Clear all ODA configurations */
void rds_enc_oda_clear(rds_encoder_t *rds);

/* ============================================================
 * RT+ (RadioText Plus) Encoder API
 * ============================================================ */
/* Add RT+ tag to RadioText
 * content_type: Content type code (0-63, see rds_tables.h)
 * start: Start position in RT (0-63 for RT, 0-127 for eRT)
 * length: Length of tag (1-64 for RT, 1-128 for eRT)
 * 
 * Call rds_oda_add() first to register RT+ on a carrier group:
 *   rds_oda_add(rds, 22, RDS_ODA_AID_RT_PLUS, message);
 * 
 * Returns 0 on success, -1 on validation error
 */
int rds_enc_rtplus_set_tags(rds_encoder_t *rds,
                            uint8_t ct1, uint8_t start1, uint8_t len1,
                            uint8_t ct2, uint8_t start2, uint8_t len2);
void rds_enc_rtplus_clear_tags(rds_encoder_t *rds);
void rds_enc_rtplus_set_toggle(rds_encoder_t *rds, int toggle);
void rds_enc_rtplus_set_item_running(rds_encoder_t *rds, int running);
/* RT+ Getters (encoder state) */
int rds_enc_rtplus_get_tag_count(const rds_encoder_t *rds);
int rds_enc_rtplus_get_tag(const rds_encoder_t *rds, int index,
                            uint8_t *content_type, uint8_t *start, uint8_t *length);
/* eRT+ API (same as RT+ but for eRT) */
int rds_enc_ert_plus_set_tags(rds_encoder_t *rds,
                              uint8_t ct1, uint8_t start1, uint8_t len1,
                              uint8_t ct2, uint8_t start2, uint8_t len2);
void rds_enc_ert_plus_clear_tags(rds_encoder_t *rds);
void rds_enc_ert_plus_set_toggle(rds_encoder_t *rds, int toggle);
void rds_enc_ert_plus_set_item_running(rds_encoder_t *rds, int running);
int rds_enc_ert_plus_get_tag_count(const rds_encoder_t *rds);
int rds_enc_ert_plus_get_tag(const rds_encoder_t *rds, int index,
                              uint8_t *content_type, uint8_t *start, uint8_t *length);

/* ============================================================
 * eRT (Enhanced RadioText) Encoder API
 * ============================================================ */
/* Set eRT text (UTF-8 or UCS-2, up to 128 bytes)
 * Call rds_oda_add() first with appropriate message bits:
 *   Bit 0: encoding (0=UCS2, 1=UTF8)
 *   Bit 1: direction (0=LTR, 1=RTL)
 *   Bits 5-2: chartable (0=E3, others reserved)
 * 
 * Example:
 *   uint16_t ert_msg = (1 << 0);  // UTF-8 encoding
 *   rds_oda_add(rds, 24, RDS_ODA_AID_ERT, ert_msg);
 *   rds_enc_set_ert(rds, (uint8_t *)"128 bytes of text...", 128);
 */
void rds_enc_set_ert(rds_encoder_t *rds, const uint8_t *text, size_t len);
void rds_enc_set_ert_with_ab(rds_encoder_t *rds, const uint8_t *text, size_t len, int ab_flag);
void rds_enc_clear_ert(rds_encoder_t *rds);
void rds_enc_get_ert(const rds_encoder_t *rds, uint8_t *text, size_t *len, size_t max_len);
void rds_enc_set_ert_ab(rds_encoder_t *rds, int ab);
int rds_enc_get_ert_ab(const rds_encoder_t *rds);
/* Get/set eRT chartable value (RDS_ERT_CHARTABLE_E3 to RDS_ERT_CHARTABLE_MAX)
 * Default: RDS_ERT_CHARTABLE_DEFAULT (RDS_ERT_CHARTABLE_E3)
 * Note: Chartable is not used for character mapping for now.
 * Setting non-zero values will log a warning. */
int rds_enc_get_ert_chartable(const rds_encoder_t *rds);
void rds_enc_set_ert_chartable(rds_encoder_t *rds, uint8_t chartable);

/* UTF-8/UCS-2 utility functions for eRT processing */
size_t rds_sanitize_utf8(const char *src, size_t src_len, char *dst, size_t dst_size);
size_t rds_ucs2_to_utf8(const uint8_t *ucs2, size_t ucs2_len, char *utf8, size_t utf8_size);
size_t rds_utf8_to_ucs2(const char *utf8, size_t utf8_len, uint8_t *ucs2, size_t ucs2_size);


/* Update group scheduler sequence (call after changing PTY, PTYN, EON, or ODA) */
void rds_scheduler_update(rds_encoder_t *rds);

/* ============================================================
 * Dynamic RDS Configuration API - Phase 1: Core Fields
 * ============================================================ */

/* PI (Programme Identification) - 16-bit station identifier */
void rds_enc_set_pi(rds_encoder_t *rds, uint16_t pi);
uint16_t rds_enc_get_pi(const rds_encoder_t *rds);

/* PS (Programme Service Name) - 8-character station name */
void rds_enc_set_ps(rds_encoder_t *rds, const char *ps);
void rds_enc_clear_ps(rds_encoder_t *rds);
void rds_enc_get_ps(const rds_encoder_t *rds, char *ps, size_t len);

/* ============================================================
 * Dynamic PS (Scrolling Programme Service Name)
 * ============================================================
 * Extends the static 8-character PS with time-sequenced overwriting
 * to create the appearance of scrolling text on receiver displays.
 *
 * WARNING: Dynamic PS is NOT part of the RDS standard (IEC 62106).
 * The EBU discourages its use. RadioText (Group 2A) is the proper
 * mechanism for dynamic text. Use dynamic PS only when you
 * specifically need receiver-display scrolling for legacy reasons.
 *
 * Usage:
 *   rds_enc_set_dynamic_ps(rds, "NOW PLAYING: BOHEMIAN RHAPSODY BY QUEEN",
 *                          RDS_DPS_SCROLL, 3, 0);
 *
 * Page mode with default '|' delimiter:
 *   rds_enc_set_dynamic_ps(rds, "RADIO 1 |HOT HITS|98.5 FM ",
 *                          RDS_DPS_PAGE, 5, 0);
 *
 * Page mode with '\n' delimiter (allows '|' in display):
 *   rds_enc_set_dynamic_ps(rds, "||||||||\\nOSMO FM \\n||||||||",
 *                          RDS_DPS_PAGE, 1, '\n');
 *
 * To stop scrolling and return to static PS:
 *   rds_enc_stop_dynamic_ps(rds);
 *   rds_enc_set_ps(rds, "RADIO 1 ");
 * ============================================================ */

/* Dynamic PS scrolling modes */
typedef enum {
	/* Character-by-character window shift (most common historical pattern).
	 * Text is padded with leading/trailing spaces and shifted 1 char at a time.
	 * De facto standard used by most European commercial stations. */
	RDS_DPS_SCROLL = 0,
	
	/* Word-step scrolling. Full words are swapped rather than character-scrolled.
	 * Fewer PS changes, less flicker on slow receivers.
	 * Popular on German and Nordic broadcasts. */
	RDS_DPS_WORD = 1,
	
	/* Paging / alternating fixed messages. Two or more 8-char strings alternate.
	 * Input text is split on a configurable delimiter (default '|'):
	 *   "RADIO 1 |HOT HITS|98.5 FM "
	 * Technically not scrolling. Considered least abusive by regulators. */
	RDS_DPS_PAGE = 2
} rds_dynamic_ps_mode_t;

/* Recommended repeat counts for different receiver compatibility levels.
 * These are in units of COMPLETE PS TRANSMISSIONS (4 Group 0 groups each).
 * Actual wall-clock time depends on the scheduler's Group 0 allocation.
 * The API logs the computed approximate interval at setup time. */
#define RDS_DPS_REPEAT_FAST     1   /* 1 PS tx/step - fastest, causes flicker */
#define RDS_DPS_REPEAT_NORMAL   3   /* 3 PS tx/step - compatible with most receivers */
#define RDS_DPS_REPEAT_SAFE     5   /* 5 PS tx/step - safe for all known receivers */

/**
 * Enable dynamic PS (scrolling/paging programme service name).
 *
 * Takes a string longer than 8 characters and scrolls it through the
 * 8-character PS display using the specified mode and repeat count.
 *
 * For strings <= 8 characters, this falls back to static PS (no scrolling).
 *
 * @param rds     Encoder state
 * @param text    Full message text (UTF-8, will be converted to RDS charset).
 *                For RDS_DPS_PAGE mode, use a delimiter to separate pages
 *                (default '|'): "RADIO 1 |HOT HITS|98.5 FM "
 * @param mode    Scrolling mode (RDS_DPS_SCROLL, RDS_DPS_WORD, RDS_DPS_PAGE)
 * @param repeat  Number of complete PS transmissions per scroll step.
 *                Each PS transmission = 4 Group 0 groups. The actual
 *                wall-clock time per step depends on the scheduler's
 *                Group 0 allocation (logged at setup time).
 *                Minimum: 1 (fastest - WARNING: causes flicker on many receivers)
 *                Recommended: 3 (normal - compatible with most receivers)
 *                Conservative: 5 (safe for all known receivers)
 *                Values < 1 are clamped to 1 with a warning.
 * @param delimiter  Page delimiter character for RDS_DPS_PAGE mode.
 *                   Pass 0 or '|' for the default pipe delimiter.
 *                   Use '\n' to allow '|' in page content.
 *                   Ignored for SCROLL and WORD modes.
 */
void rds_enc_set_dynamic_ps(rds_encoder_t *rds, const char *text,
                            rds_dynamic_ps_mode_t mode, int repeat,
                            char delimiter);

/**
 * Stop dynamic PS and return to static PS mode.
 * The current PS display value is frozen (not cleared).
 * Call rds_enc_set_ps() afterwards to set a new static PS.
 */
void rds_enc_stop_dynamic_ps(rds_encoder_t *rds);

/**
 * Check if dynamic PS is currently active.
 * @return 1 if scrolling/paging is active, 0 if static PS
 */
int rds_enc_is_dynamic_ps(const rds_encoder_t *rds);

/**
 * Get the full dynamic PS source text.
 * @param text  Output buffer (should be at least 256 bytes)
 * @param len   Buffer size
 */
void rds_enc_get_dynamic_ps_text(const rds_encoder_t *rds, char *text, size_t len);

/* PTY (Programme Type) - 5-bit program type code (0-31) */
void rds_enc_set_pty(rds_encoder_t *rds, uint8_t pty);
uint8_t rds_enc_get_pty(const rds_encoder_t *rds);

/* PTYN (Programme Type Name) - 8-character program type name */
void rds_enc_set_ptyn(rds_encoder_t *rds, const char *ptyn);
void rds_enc_clear_ptyn(rds_encoder_t *rds);
void rds_enc_get_ptyn(const rds_encoder_t *rds, char *ptyn, size_t len);

/* TP (Traffic Programme) - 1 if station broadcasts traffic info */
void rds_enc_set_tp(rds_encoder_t *rds, int tp);
int rds_enc_get_tp(const rds_encoder_t *rds);

/* MS (Music/Speech) - 1=Music, 0=Speech */
void rds_enc_set_ms(rds_encoder_t *rds, int ms);
int rds_enc_get_ms(const rds_encoder_t *rds);

/* ============================================================
 * Dynamic RDS Configuration API - Phase 2: Extended Info
 * ============================================================ */

/* ECC (Extended Country Code) - 8-bit extended country code */
void rds_enc_set_ecc(rds_encoder_t *rds, uint8_t ecc);
void rds_enc_clear_ecc(rds_encoder_t *rds);
uint8_t rds_enc_get_ecc(const rds_encoder_t *rds);

/* Language Code - 8-bit language identifier (ISO 639) */
void rds_enc_set_language(rds_encoder_t *rds, uint8_t lang);
void rds_enc_clear_language(rds_encoder_t *rds);
uint8_t rds_enc_get_language(const rds_encoder_t *rds);

/* PIN (Programme Item Number) - day (1-31), hour (0-23), minute (0-59) */
void rds_enc_set_pin(rds_encoder_t *rds, uint8_t day, uint8_t hour, uint8_t minute);
void rds_enc_clear_pin(rds_encoder_t *rds);
void rds_enc_get_pin(const rds_encoder_t *rds, uint8_t *day, uint8_t *hour, uint8_t *minute);

/* DI (Decoder Information) - stereo, artificial_head, compressed, dynamic_pty flags */
void rds_enc_set_di(rds_encoder_t *rds, int stereo, int artificial, int compressed, int dynamic_pty);
void rds_enc_get_di(const rds_encoder_t *rds, int *stereo, int *artificial, int *compressed, int *dynamic_pty);

/* ============================================================
 * Dynamic RDS Configuration API - Phase 3: Alternative Frequencies
 * ============================================================ */

/* AF Method A - Set alternative frequencies from string (e.g., "87500,88100,89300") */
int rds_enc_af_set_method_a(rds_encoder_t *rds, const char *af_string);
int rds_enc_af_get_method_a_count(const rds_encoder_t *rds);

/* AF Method B - List Management */
/* Add AF list from string format */
int rds_enc_af_method_b_add(rds_encoder_t *rds, const char *af_string);
/* Add complete AF list with tuning frequency and regional flags */
int rds_enc_af_method_b_add_list(rds_encoder_t *rds, uint16_t tuning_freq, const uint16_t *af_freqs, const uint8_t *af_is_regional, uint8_t af_count);
/* Remove AF list by tuning frequency */
int rds_enc_af_method_b_remove_list(rds_encoder_t *rds, uint16_t tuning_freq);
/* Remove AF list by index */
int rds_enc_af_method_b_remove_list_by_index(rds_encoder_t *rds, int index);

/* AF Method B - Entry Management */
/* Add single AF entry to a list */
int rds_enc_af_method_b_add_entry(rds_encoder_t *rds, uint16_t tuning_freq, uint16_t af_freq, int is_regional);
/* Remove single AF entry from a list */
int rds_enc_af_method_b_remove_entry(rds_encoder_t *rds, uint16_t tuning_freq, uint16_t af_freq);
/* Remove AF entry by index */
int rds_enc_af_method_b_remove_entry_by_index(rds_encoder_t *rds, uint16_t tuning_freq, int af_index);

/* AF Clear and Getters */
/* Clear all alternative frequencies */
void rds_enc_af_clear(rds_encoder_t *rds);
/* Get current AF method (0=Method A, 1=Method B, -1=none) */
int rds_enc_af_get_method(const rds_encoder_t *rds);
/* Get number of AF Method B lists */
int rds_enc_af_method_b_get_list_count(const rds_encoder_t *rds);
/* Get AF Method B list by index */
int rds_enc_af_method_b_get_list(const rds_encoder_t *rds, int index, uint16_t *tuning_freq, uint16_t *af_freqs, uint8_t *af_is_regional, uint8_t *af_count, size_t max_afs);

/* ============================================================
 * Dynamic RDS Configuration API - Phase 4: RadioText
 * ============================================================ */

/* RadioText - Clear RadioText buffer */
void rds_enc_clear_radiotext(rds_encoder_t *rds);
/* Get RadioText (converts RDS charset to UTF-8 for display) */
void rds_enc_get_radiotext(const rds_encoder_t *rds, char *rt, size_t len);

/* ============================================================
 * Dynamic RDS Configuration API - Phase 5: EON
 * ============================================================ */

/* Add Enhanced Other Network entry */
int rds_enc_eon_add(rds_encoder_t *rds, uint16_t pi, const char *ps, uint8_t pty, uint8_t tp);
/* Set Traffic Announcement flag for EON entry */
int rds_enc_eon_set_ta(rds_encoder_t *rds, uint16_t pi, int ta);
/* Remove EON entry by PI */
int rds_enc_eon_remove(rds_encoder_t *rds, uint16_t pi);
/* Clear all EON entries */
void rds_enc_eon_clear(rds_encoder_t *rds);
int rds_enc_eon_get_count(const rds_encoder_t *rds);
int rds_enc_eon_get_entry(const rds_encoder_t *rds, int index, uint16_t *pi, char *ps, size_t ps_len, uint8_t *pty, uint8_t *tp, uint8_t *ta);

/* ============================================================
 * Paging Encoder API (EN 50067 Annex M)
 * ============================================================ */
/* BCD encoding/decoding utilities */
int rds_paging_bcd_encode(const char *digits, int len, uint8_t *nibbles, int max_nibbles);
int rds_paging_bcd_decode(const uint8_t *nibbles, int count, char *digits, int max_len);

/* Address packing/unpacking */
void rds_paging_addr_pack(uint32_t address, uint16_t *block_c, uint8_t *block_d_hi);
uint32_t rds_paging_addr_unpack(uint16_t block_c, uint8_t block_d_hi);

/* Queue paging messages */
int rds_enc_paging_send_tone(rds_encoder_t *rds, uint32_t address,
			     int repeats, int interval_sec);
int rds_enc_paging_send_numeric(rds_encoder_t *rds, uint32_t address,
				const char *digits, int repeats, int interval_sec);
int rds_enc_paging_send_alpha(rds_encoder_t *rds, uint32_t address,
			      const char *text, int repeats, int interval_sec);

/* Configuration */
void rds_enc_paging_enable(rds_encoder_t *rds, int enable);
void rds_enc_paging_set_rpc(rds_encoder_t *rds, uint8_t rpc);
void rds_enc_paging_set_enhanced(rds_encoder_t *rds, int enable);

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
	int		force_rbds;	/* force RBDS decoding (--rbds flag) */
	double		time_constant_us; /* de-emphasis time constant (50=RDS, 75=RBDS) */
	int		rbds_hint_logged; /* 1 after emphasis/mode mismatch hint logged */
	int		ecc_mismatch_logged; /* 1 after ECC vs mode mismatch logged */
	
	/* Tunable State */
	double		freq_subcarrier;	/* Current locked frequency (~57000 Hz) */
	double		phase_subcarrier;	/* Current subcarrier phase (0..2pi) */
	
	/* PLL */
	rds_iir_filter_t	filter_pll;	/* Loop filter for PLL */
	rds_iir_filter_t	filter_2400_i;	/* 2.4kHz LPF for I */
	rds_iir_filter_t	filter_2400_q;	/* 2.4kHz LPF for Q */
	
	/* AGC for Costas PLL (normalizes baseband level so error signal is meaningful) */
	double		agc_gain;		/* Current AGC gain */
	double		agc_alpha;		/* AGC smoothing constant */
	
	/* Clock Recovery (Subcarrier-locked) */
	double		clock_offset;		/* Phase offset for 1187.5 Hz clock */
	int		prev_clock_bit;		/* Previous clock level (+1/-1) */
	double		prev_bb_sample;		/* Previous baseband sample (for zero cross) */
	double		integrator;		/* Integrate-and-dump accumulator */
	int		decimate_counter;	/* Decimation counter (per-instance) */
	
	/* PLL Lock Detection (RdsSurveyor2-style) */
	double		lock_sum_i;		/* Running sum of |bb_i| at symbol dumps */
	double		lock_sum_q;		/* Running sum of |bb_q| at symbol dumps */
	int		lock_count;		/* Number of symbol dumps in window */
	int		pll_locked;		/* 1 if PLL is locked (I >> Q) */
	
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
	int		groups_skipped_no_block_b;	/* Groups skipped: Block B missing */
	int		blocks_missing[4];		/* Per-block missing counts [A,B,C,D] */
	
	/* Group type distribution counters (0A=0, 0B=1, 1A=2, ... 15B=31) */
	int		group_type_counts[32];
	
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
	char		ps_prev[9];		/* Previous PS (segment-level, for scrolling) */
	char		ps_prev_successful[9];	/* Previous successful full PS decode */
	uint8_t		ps_status[8];	/* Per-char status: enum rds_decode_status */
	int		ps_changes;	/* Number of PS string changes (scrolling) */
	
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

	/* Group 3A: ODA Application Registry (decoded from 3A groups)
	 * Maps group types to ODA applications - up to 32 slots (0A-15B) */
	rds_oda_app_t	oda_apps[32];		/* ODA registry indexed by group type code */
	int		oda_app_count;		/* Number of registered ODAs */

	/* RT+ (RadioText Plus) - Decoder state
	 * Content-type tagging for RT and eRT */
	rds_rtplus_decoder_t rtplus;		/* RT+ tags for standard RadioText */
	rds_rtplus_decoder_t ert_plus;		/* RT+ tags for Enhanced RadioText */
	
	/* eRT (Enhanced RadioText) - Decoder state */
	rds_ert_decoder_t ert_dec;		/* eRT current (partial) reception */
	rds_ert_decoder_t ert_dec_complete;	/* eRT last complete reception */
	rds_rtplus_decoder_t ert_plus_complete;	/* eRT+ last complete tags */
	uint8_t ert_has_complete;		/* 1 if we have complete eRT */
	uint8_t ert_plus_has_complete;		/* 1 if we have complete eRT+ tags */
	uint8_t ert_ab;				/* Current A/B flag for change detection */

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
	
	/* Radio Paging (EN 50067 Annex M) - Group 7A, 1A RPC, 13A */
	int		paging_enabled;		/* Decode paging groups (--paging) */
	rds_paging_dec_t paging_dec;		/* Paging decoder state */
	
	/* Status timing */
	double		status_timer;
	double		status_interval;
	
	/* Signal-level debug instrumentation */
	double		signal_debug_timer;
	double		signal_debug_interval;
	double		sig_bb_i_peak;		/* Peak baseband I since last report */
	double		sig_bb_q_peak;		/* Peak baseband Q since last report */
	double		sig_bb_i_sum;		/* Sum of |bb_i| for average */
	double		sig_bb_q_sum;		/* Sum of |bb_q| for average */
	double		sig_pll_err_peak;	/* Peak PLL error since last report */
	double		sig_pll_err_sum;	/* Sum of |pll_err| for average */
	double		sig_input_peak;		/* Peak input sample level */
	double		sig_input_sum;		/* Sum of |input| for average */
	double		sig_integrator_peak;	/* Peak integrator dump value */
	double		sig_integrator_sum;	/* Sum of |integrator| for average */
	long		sig_sample_count;	/* Samples since last report */
	long		sig_dump_count;		/* Integrator dumps since last report */
	long		sig_zc_count;		/* Zero crossings since last report */
	int		sig_frame_flips;	/* Biphase frame realignments */
	int		sig_blocks_ok_period;	/* Blocks OK in this period */
	int		sig_blocks_fec_period;	/* Blocks FEC-corrected in this period */
	int		sig_blocks_bad_period;	/* Blocks bad in this period */
	int		sig_groups_period;	/* Groups decoded in this period */
	int		sig_block_miss[4];	/* Per-block misses in this period [A,B,C,D] */
	int		sig_block_wrongoff;	/* Wrong-offset syndrome matches in period */
	int		sig_bits_period;	/* Bits decoded in this period */
	double		sig_eye_sum;		/* Sum of symbol confidence values */
	double		sig_eye_min;		/* Minimum symbol confidence in period */
	long		sig_eye_count;		/* Symbol decisions in period */
	int		sig_eye_weak;		/* Low-confidence symbol decisions */
	int		sig_biphase_err[2];	/* Biphase error counts [frame0, frame1] */
	
	/* File output for captured RDS data */
	FILE		*hexrds_file;		/* Output file for .hexrds format */
	FILE		*bitstream_file;	/* Output file for .rds bitstream format */
	uint8_t		bitstream_byte;		/* Accumulator for bitstream output */
	int		bitstream_bit_count;	/* Bits accumulated (0-7) */
	
	/* Protocol server callback (for XDR-GTK, RDS Spy output) */
	void		(*group_callback)(const uint16_t blocks[4], const uint8_t status[4], void *arg);
	void		*group_callback_arg;
} rds_decoder_t;

/* Initialize RDS decoder
 * debug: Enable debug logging (raw hex codes, --rds-debug)
 * verbose: Enable verbose/info logging (human-readable, --rds-verbose)
 * force_rbds: Force RBDS decoding (callsign + US PTY names, --rbds)
 */
int rds_decoder_init(rds_decoder_t *rds, double samplerate, int debug, int verbose, double time_constant_us, int force_rbds);

/* Process FM baseband samples, extract RDS data
 * pilot_phase: current 19 kHz pilot phase from stereo decoder (or free-run if 0)
 * pilot_phasestep: phase increment per sample for pilot (2pi x 19000 / samplerate)
 */
void rds_decoder_process(rds_decoder_t *rds, sample_t *samples, int num,
                         double pilot_phase, double pilot_phasestep);

/* Feed a pre-decoded group directly into the decoder (bypasses DSP/PLL/sync).
 * blocks[4]: 4 data words, status[4]: decode status per block */
void rds_decoder_feed_group(rds_decoder_t *rds, const uint16_t blocks[4], const uint8_t status[4]);

/* Feed a single decoded bit into the decoder's sync state machine.
 * Bypasses DSP/PLL/biphase -- used for binary .rds bitstream files.
 * bit: 0 or 1 (MSB first, matching RDS over-the-air bit order). */
void rds_decoder_feed_bit(rds_decoder_t *rds, int bit);

/* Get decoded data (returns 1 if new data available) */
/* Get decoded PI (Programme Identification) - returns 1 if new data available */
int rds_dec_get_pi(rds_decoder_t *rds, uint16_t *pi);
/* Get decoded PS (Programme Service name) - ps must be at least 9 bytes, returns 1 if new */
int rds_dec_get_ps(rds_decoder_t *rds, char *ps);
/* Get decoded RT (RadioText) - rt must be at least 65 bytes, returns 1 if new */
int rds_dec_get_rt(rds_decoder_t *rds, char *rt);

/* Print decoder status (PI, PS, RT, BER) */
void rds_decoder_status(rds_decoder_t *rds);

/* ============================================================
 * RT+ and eRT Decoder API
 * ============================================================ */
/* RT+ Decoder Getters */
int rds_dec_rtplus_get_tag_count(const rds_decoder_t *rds);
int rds_dec_rtplus_get_tag(const rds_decoder_t *rds, int index,
                            uint8_t *content_type, uint8_t *start, uint8_t *length);
int rds_dec_rtplus_get_toggle(const rds_decoder_t *rds);
int rds_dec_rtplus_get_item_running(const rds_decoder_t *rds);
/* eRT+ Decoder Getters */
int rds_dec_ert_plus_get_tag_count(const rds_decoder_t *rds);
int rds_dec_ert_plus_get_tag(const rds_decoder_t *rds, int index,
                              uint8_t *content_type, uint8_t *start, uint8_t *length);
/* eRT Decoder Getters */
void rds_dec_get_ert(const rds_decoder_t *rds, uint8_t *text, size_t *len, size_t max_len);
int rds_dec_get_ert_encoding(const rds_decoder_t *rds);
int rds_dec_get_ert_direction(const rds_decoder_t *rds);
/* Get eRT chartable value (RDS_ERT_CHARTABLE_E3 to RDS_ERT_CHARTABLE_MAX)
 * Default: RDS_ERT_CHARTABLE_DEFAULT (RDS_ERT_CHARTABLE_E3)
 * Note: Chartable is not used for character mapping for now. */
int rds_dec_get_ert_chartable(const rds_decoder_t *rds);

/* eRT+ tag text extraction with character position addressing */
size_t rds_ert_extract_tag_text(const rds_ert_decoder_t *ert, 
                                 size_t char_start, size_t char_len,
                                 char *out, size_t out_size);

/* Cleanup */
void rds_decoder_exit(rds_decoder_t *rds);

/* File output for captured RDS data */
int rds_decoder_set_hexrds_file(rds_decoder_t *rds, const char *filename);
int rds_decoder_set_bitstream_file(rds_decoder_t *rds, const char *filename);

/* Set callback for decoded RDS groups (for protocol servers) */
void rds_decoder_set_group_callback(rds_decoder_t *rds,
                                    void (*callback)(const uint16_t blocks[4], const uint8_t status[4], void *arg),
                                    void *arg);

#endif /* _RDS_H */
