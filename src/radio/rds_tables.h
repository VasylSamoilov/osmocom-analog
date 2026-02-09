/*
 * RDS (Radio Data System) Lookup Tables
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * Human-readable lookup tables for RDS codes.
 * Reference: IEC 62106 Annexes, NRSC-4-B
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef _RDS_TABLES_H
#define _RDS_TABLES_H

#include <stdint.h>

/* ============================================================
 * Programme Type (PTY) Names
 * IEC 62106 Annex F (RDS) / NRSC-4-B Annex F (RBDS)
 * ============================================================ */

/* Get PTY name string (0-31)
 * rbds: 0 = European RDS, 1 = US RBDS variant */
const char *rds_get_pty_name(uint8_t pty, int rbds);

/* ============================================================
 * Language Codes
 * IEC 62106 Annex J (128 entries)
 * ============================================================ */

/* Get language name from code (0-127) */
const char *rds_get_language_name(uint8_t code);

/* ============================================================
 * Country Codes
 * IEC 62106 Annex D (ECC + CC mapping)
 * ============================================================ */

/* Get 2-letter ISO country code from PI country code and ECC
 * cc: upper 4 bits of PI (1-15)
 * ecc: Extended Country Code from Group 1A variant 0 */
const char *rds_get_country_code(uint8_t cc, uint8_t ecc);

/* Get full country name from ECC lookup index (0-227) */
const char *rds_get_ecc_country_name(int index);

/* Get human-readable ECC region name
 * Returns descriptive string like "Europe 1", "Americas - US", etc.
 * For undefined ECCs returns "Not defined in EN 50067" */
const char *rds_get_ecc_name(uint8_t ecc);

/* Check if ECC value is defined in EN 50067 Table D.1 / Annex N.
 * Valid: E0-E4 (Europe), A0-A5 (Americas), D0-D3 (Africa), F0-F2 (Asia/Pacific).
 * Returns 1 if valid, 0 if not defined. */
int rds_is_valid_ecc(uint8_t ecc);

/* Get North American (RBDS) Call Sign from PI code
 * Returns pointer to static buffer (not thread-safe) or NULL if not a valid call sign */
const char *rds_get_callsign(uint16_t pi);

/* Get PI code from North American (RBDS) Call Sign
 * Returns 0 on failure (invalid call sign) */
uint16_t rds_get_pi_from_callsign(const char *call);

/* ============================================================
 * Decoder Identification (DI) - Group 0A/0B
 * IEC 62106 S3.2.1.5
 *
 * DI bits are transmitted one per Group 0:
 *   addr 0: d0 - Dynamic PTY (still relevant)
 *   addr 1: d1 - Compressed (obsolete per IEC 62106-2:2021)
 *   addr 2: d2 - Artificial head (obsolete)
 *   addr 3: d3 - Stereo (obsolete - use pilot detection)
 * ============================================================ */

/* Get DI short name by address (0-3) */
const char *rds_get_di_name(uint8_t addr);

/* Get DI human-readable description */
const char *rds_get_di_description(uint8_t addr);

/* Get meaning of DI value (0 or 1) for given address */
const char *rds_get_di_value_name(uint8_t addr, int value);

/* ============================================================
 * Music/Speech (MS) Flag - Group 0A/0B
 * IEC 62106 S3.2.1.3
 * ============================================================ */

/* Get MS flag meaning: 0="Speech", 1="Music" */
const char *rds_get_ms_name(int ms);

/* ============================================================
 * Traffic Programme (TP) and Traffic Announcement (TA) Flags
 * IEC 62106 S3.2.1.2
 * ============================================================ */

/* Get TP flag meaning */
const char *rds_get_tp_name(int tp);

/* Get TA flag meaning */
const char *rds_get_ta_name(int ta);

/* Get combined TP/TA interpretation (EON referral, announcement, etc.) */
const char *rds_get_tp_ta_description(int tp, int ta);

/* ============================================================
 * Alternative Frequencies (AF) - Group 0A Block C
 * IEC 62106 S6.2.1.6.1
 * ============================================================ */

/* Convert AF code (1-204) to frequency in 0.1 MHz units (876-1079) */
int rds_af_code_to_freq(uint8_t code);

/* Convert frequency (0.1 MHz units) to AF code (1-204) */
uint8_t rds_freq_to_af_code(int freq_tenth_mhz);

/* Get human-readable AF code type description */
const char *rds_get_af_code_description(uint8_t code);

/* Check if AF code is a valid FM frequency (1-204) */
int rds_is_valid_af_code(uint8_t code);

/* Check if AF pair is regional variant (freq1 > freq2 = descending = regional) */
int rds_af_is_regional_pair(uint16_t freq1, uint16_t freq2);

/* Check if AF list is Method B (heuristic: odd count, tuning freq repeats) */
int rds_af_is_method_b(const uint16_t *af, int count);

/* LF/MF frequency conversion (code 250 prefix frequencies)
 * LF band: 153-279 kHz (codes 1-15), always 9 kHz spacing
 * MF band: 531-1602 kHz EU / 540-1710 kHz US (codes 16-135)
 */
uint8_t rds_lf_freq_to_code(int freq_khz);
uint8_t rds_mf_freq_to_code(int freq_khz, int is_us);
int rds_lf_mf_code_to_freq(uint8_t code, int is_us);

/* ============================================================
 * Open Data Application (ODA) Names
 * RDS Forum registration documents
 * ============================================================ */

/* Get ODA application name from AID */
const char *rds_get_oda_name(uint16_t aid);

/* ============================================================
 * RadioText+ (RT+) Content Types
 * RDS Forum R06/040_1 (2006-07-21)
 * ============================================================ */

/* Get RT+ content type name (0-64) */
const char *rds_get_rtplus_content_type(uint8_t type);

/* ============================================================
 * RDS Character Map (IEC 62106 Annex E)
 * ============================================================
 * RDS uses an 8-bit character encoding based on ISO 8859-1/15
 * with RDS-specific modifications for accented characters.
 *
 * ASCII (0x20-0x7F) is a subset - these map directly.
 * Extended characters (0x80-0xFF) are RDS-specific.
 * ============================================================ */

/* Get UTF-8 string for RDS character code (0-255)
 * For display purposes - control chars return their UTF-8 equivalents */
const char *rds_get_char(uint8_t code);

/* Get display-safe UTF-8 string for RDS character code
 * Control characters (0x00-0x1F except 0x0A, 0x0D) are shown as "<XX>"
 * CR (0x0D) is shown as "<0D>" (RT terminator)
 * LF (0x0A) is shown as "<LF>" (preferred line break)
 * Returns pointer to static buffer (not thread-safe) */
const char *rds_char_to_display(uint8_t code);

/* Convert RDS text buffer to display-safe UTF-8 string
 * Handles control characters as <XX> format
 * @param rds_text: RDS 8-bit character buffer
 * @param len: Length of buffer (max 64 for RT, 8 for PS)
 * @param out: Output buffer for UTF-8 string
 * @param out_size: Size of output buffer (should be 4*len for safety)
 * @return: Number of characters written */
int rds_text_to_display(const uint8_t *rds_text, int len, char *out, int out_size);

/* Convert UTF-8 string to RDS 8-bit encoding
 * - ASCII characters map directly
 * - LF (0x0A) is preserved as preferred line break
 * - CR (0x0D) terminates the message
 * - Other control characters are ignored
 * - Non-encodable Unicode chars are replaced with space and warned once
 *
 * @param utf8: Input UTF-8 string
 * @param rds_out: Output RDS 8-bit buffer
 * @param max_len: Maximum output length (64 for RT, 8 for PS)
 * @param warn_unencodable: If non-NULL, set to 1 if any chars couldn't be encoded
 * @return: Actual output length (excluding any padding) */
int rds_encode_text(const char *utf8, uint8_t *rds_out, int max_len, int *warn_unencodable);

/* Validate UTF-8 string for RDS encoding
 * Logs warning for any characters that cannot be encoded
 * @return: Number of non-encodable characters found */
int rds_validate_text(const char *utf8, const char *field_name);

/* ============================================================
 * Linkage Information Codes (LIC)
 * IEC 62106 Group 14A
 * ============================================================ */

/* Get human-readable LIC name for code (0-15) */
const char *rds_get_linkage_name(uint8_t lic);

/* ============================================================
 * DAB Frequencies
 * ============================================================ */

/* Get DAB Channel Name (e.g. "5A") from frequency in kHz.
 * Returns "??" if not found. */
const char *rds_get_dab_channel_name(uint32_t frequency);

/* ============================================================
 * Programme Identification (PI) Code Decode
 * EN 50067 Annex D / NRSC-4-B Annex D
 *
 * PI structure (16 bits):
 *   b15-b12: Country Code (CC) -- 1-F valid, 0 invalid (EN 50067 D.1)
 *   b11-b8:  Coverage Area Code -- L/I/N/S/R1-R12 (EN 50067 D.4)
 *   b7-b0:   Programme Reference Number -- 0 not assigned (EN 50067 D.5)
 *
 * For RBDS (North America), PI encodes call sign instead.
 * ============================================================ */

/* Decoded PI code fields */
typedef struct {
	uint16_t	pi;		/* Raw PI code */
	uint8_t		cc;		/* Country Code (b15-b12), 0 = invalid */
	uint8_t		coverage;	/* Coverage Area Code (b11-b8) */
	uint8_t		ref;		/* Programme Reference Number (b7-b0) */
	const char	*cc_name;	/* ISO country code, or list when ECC unknown */
	const char	*coverage_name;	/* "Local", "National", "R3", etc. */
	const char	*callsign;	/* RBDS call sign or NULL */
	const char	*pi_note;	/* Warning for special PI values, or NULL */
	int		cc_valid;	/* 1 if CC is in range 1-F */
	int		ref_valid;	/* 1 if ref != 0 (0 = not assigned) */
	int		is_rbds;	/* 1 if decoded as RBDS (ECC or forced) */
} rds_pi_info_t;

/* Decode PI code into its constituent fields (EN 50067 D.1-D.5).
 * Works for both RDS and RBDS. For RBDS, also populates callsign.
 * ecc: Extended Country Code (from Group 1A), 0 if unknown.
 * force_rbds: 1 to force RBDS decoding (callsign + US PTY names).
 * Shared by encoder and decoder. */
void rds_decode_pi(uint16_t pi, uint8_t ecc, int force_rbds, rds_pi_info_t *info);

/* Get all possible countries for a CC value across all ECCs.
 * Returns comma-separated ISO codes in a static buffer.
 * Used when ECC is unknown to show what CC could mean.
 * EN 50067 Table D.1: CC is ambiguous without ECC. */
const char *rds_get_cc_countries(uint8_t cc);

/* Get coverage area name from code (0-F).
 * EN 50067 D.4: 0=Local, 1=International, 2=National, 3=Supra-regional,
 * 4-F = R1-R12 */
const char *rds_get_coverage_name(uint8_t code);

/* Get Linked Station Name (e.g. "NPR-1") from PI code.
 * Returns NULL if not found. */
const char *rds_get_linked_station_name(uint16_t pi);

#endif /* _RDS_TABLES_H */
