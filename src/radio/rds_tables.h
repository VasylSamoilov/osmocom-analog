/*
 * RDS (Radio Data System) Lookup Tables
 *
 * (C) 2025-2026 by osmocom-analog authors
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
 * Used for educational logging to explain what ECC value means */
const char *rds_get_ecc_name(uint8_t ecc);

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
 * RDS Character Map
 * IEC 62106 Annex E - RDS character set
 * ============================================================ */

/* Get UTF-8 string for RDS character code (0-255) */
const char *rds_get_char(uint8_t code);

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
 * Linked Station Codes (RBDS)
 * ============================================================ */

/* Get Linked Station Name (e.g. "NPR-1") from PI code.
 * Returns NULL if not found. */
const char *rds_get_linked_station_name(uint16_t pi);

#endif /* _RDS_TABLES_H */
