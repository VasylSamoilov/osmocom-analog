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

/* Get human-readable ECC region name
 * Returns descriptive string like "Europe 1", "Americas - US", etc.
 * Used for educational logging to explain what ECC value means */
const char *rds_get_ecc_name(uint8_t ecc);

/* ============================================================
 * Decoder Identification (DI) Codes
 * IEC 62106 S3.2.1.5
 * ============================================================ */

/* Get DI meaning by address (0-3)
 * 0: d3 - Stereo
 * 1: d2 - Artificial head
 * 2: d1 - Compressed
 * 3: d0 - Dynamic PTY */
const char *rds_get_di_name(uint8_t addr);

/* ============================================================
 * Open Data Application (ODA) Names
 * RDS Forum registration documents
 * ============================================================ */

/* Get ODA application name from AID */
const char *rds_get_oda_name(uint16_t aid);

#endif /* _RDS_TABLES_H */
