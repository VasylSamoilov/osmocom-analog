/* main function
 *
 * (C) 2018 by Andreas Eversberg <jolly@eversberg.eu>
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
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libclipper/clipper.h"
#include "radio.h"
#include "rds_tables.h"
#include "../libfm/fm.h"

#define CLIP_POINT	0.85
#define DC_CUTOFF	30.0 // Wikipedia: UKW-Rundfunk
#define STEREO_BW	15000.0
#define PILOT_FREQ	19000.0
#define PILOT_BW	5.0
#define PHASE_ERROR_TOLERANCE	3.0	/* ITU-R BS.450-4 S2.2.2.5: +/-3deg */
#define PHASE_ERROR_AVG_SAMPLES	10000	/* samples to average for phase error */

/* ============================================================
 * RDS Preset Configuration System
 * Press 'f' during operation to cycle between presets.
 * 
 * Country is determined by PI prefix (first hex digit) + ECC combination.
 * See IEC 62106 Annex D for official tables.
 * TEF6686 lookup: docs/rds/TEF6686_ESP32/src/TEF6686.cpp lines 848-1155
 * Country name table: docs/rds/TEF6686_ESP32/src/TEF6686.h lines 111-340
 * ============================================================ */

typedef struct rds_preset {
	const char	*name;		/* Preset name for display */
	uint16_t	pi;		/* Programme Identification */
	const char	*ps;		/* Programme Service (8 chars max) */
	const char	*rt;		/* RadioText: 64 chars max (2A) or 32 chars max (2B), per IEC 62106 */
	uint8_t		pty;		/* Programme Type (0-31) */
	const char	*ptyn;		/* PTY Name (8 chars max or NULL) */
	uint8_t		tp;		/* Traffic Programme flag */
	uint8_t		ms;		/* Music/Speech (1=Music) */
	uint8_t		ecc;		/* Extended Country Code */
	uint8_t		lang;		/* Language Code */
	
	/* AF Method A - Human-readable format (IEC 62106 S3.2.1.6.3)
	 * Format: "91.0, 102.5, LF225, MF1008"
	 * - VHF frequencies in MHz (87.6-107.9)
	 * - LFxxx for LF band in kHz (153-279)
	 * - MFxxx for MF band in kHz (531-1602)
	 * LF/MF count as 2 slots each. Max 25 total slots. */
	const char	*af_method_a_str;
	
	/* AF Method B - Paired frequency lists (IEC 62106 S3.2.1.6.4)
	 * Format: "T89.3, 99.5, 88.8, R102.6, R89.0"
	 *   T prefix = tuning frequency (required first element)
	 *   R prefix = regional variant (F1 > F2 descending order)
	 *   No prefix = same programme (F1 < F2 ascending order)
	 * Max 12 AFs per list. Multiple lists transmitted sequentially. */
	const char	*af_method_b_str[RDS_AF_METHOD_B_MAX_LISTS];
	uint8_t		af_method_b_count;	/* Number of Method B lists */
	
	const char	*callsign;	/* RBDS Call Sign (overrides/sets PI if PI=0) */
	/* Group 1A PIN - Programme Item Number for THIS station (current PI)
	 * HISTORICAL: Enabled "VCR-like" radio recording (1984-1990s).
	 * Users entered PIN from published schedule; receiver triggered
	 * recording when transmitted PIN matched programme start.
	 * NOW OBSOLETE: Most stations transmit 0x0000 (no PIN). */
	uint8_t		pin_day;	/* Day of month 1-31 (0 = not used) */
	uint8_t		pin_hour;	/* Hour 0-23, or 24 = end time */
	uint8_t		pin_minute;	/* Minute 0-59 (prefer 00/15/30/45) */
	/* EON (Enhanced Other Networks) - Group 14A */
	struct {
		uint16_t	pi;		/* Other Network PI code */
		const char	*ps;		/* Other Network PS name */
		uint8_t		pty;		/* Other Network PTY */
		uint8_t		tp;		/* Other Network TP flag */
		uint8_t		ta;		/* Other Network TA flag (variant 13) */
		/* AF (variant 4) - AF codes 1-204 */
		uint8_t		af[2];		/* Two AF codes for Other Network */
		uint8_t		af_count;	/* 0-2 */
		/* Mapped AF (variants 5-9) */
		struct {
			uint8_t	tuned;		/* Tuned freq AF code */
			uint8_t	on;		/* ON freq AF code */
		} mapped_af[2];
		uint8_t		mapped_af_count; /* 0-2 */
		/* Linkage (variant 12) */
		uint8_t		linkage_la;	/* Linkage Actuator */
		uint16_t	linkage_lsn;	/* Linkage Set Number (12 bits) */
		/* PIN (variant 14) - Programme Item Number for Other Network
		 * This PIN applies to the LINKED station (eon.pi), NOT the current station.
		 * Day=0 means "no PIN" (IEC 62106 S6.1.5.2).
		 * Example: News bulletin starts at 14:30 on day 15 of month */
		uint8_t		pin_day;	/* Day 1-31 (0 = not used) */
		uint8_t		pin_hour;	/* Hour 0-23 (24 = end time) */
		uint8_t		pin_minute;	/* Minute 0-59 */
		/* Broadcaster data (variant 15) */
		uint16_t	broadcaster_data;
	} eon[2];				/* Up to 2 Other Networks per preset */
	uint8_t		eon_count;		/* Number of EON entries (0-2) */
	
	/* Group version selection (0=auto, 1=force A, 2=force B)
	 * Use RDS_GROUP_VERSION_AUTO/A/B from rds.h
	 * When 0 (AUTO): version auto-detected from data:
	 *   - Group 0: 0B if af_count==0, else 0A
	 *   - Group 1: 1B if ecc==0 && lang==0, else 1A
	 *   - Group 2: 2B if strlen(rt)<=32, else 2A */
	uint8_t		group0_version;		/* 0A vs 0B */
	uint8_t		group1_version;		/* 1A vs 1B */
	uint8_t		group2_version;		/* 2A vs 2B */

} rds_preset_t;

/* RDS Presets - add more as needed
 * RT max length: 64 chars (Group 2A) or 32 chars (Group 2B) - terminated with CR (0x0D)
 * PS max length: 8 chars (padded with spaces)
 * PTYN max length: 8 chars */
static const rds_preset_t rds_presets[] = {
	/* ============================================================
	 * Press 'f' to cycle to normal presets.
	 * ============================================================ */

	{
		.name     = "Ukraine (RDS)",
		/* PI Structure for Europe (IEC 62106):
		 * [Country(4)][Coverage(4)][Reference(8)]
		 * Coverage Codes:
		 *  0=Local, 1=International, 2=National, 3=Supra-regional
		 *  4-F = Regional Area 1-12
		 * Here: 6 (Ukraine) + A (Regional 7) + CE (Ref) */
		.pi       = 0x6ACE,
		.ps       = "Osmo RDS",
		/* 64-char RT (max for Group 2A) - use full capacity for demo */
		.rt       = "osmocom-analog FM Radio - Open Source Broadcast FM RDS Encoder!",
		.pty      = 10,		/* Pop music (RDS) */
		.ptyn     = "OsmoPTYN",
		.tp       = 1,
		.ms       = 1,
		.ecc      = 0xE4,	/* Ukraine with PI prefix 6 */
		.lang     = 73,		/* Ukrainian (LIC code from IEC 62106 Annex J) */
		/* AF Method A with LF frequency (AM simulcast at 225 kHz) */
		.af_method_a_str = "LF225",
		/* Group 1A PIN - Legacy "VCR-like" recording feature (1984-1990s)
		 * Example: "Evening News" listed in newspaper as starting 18:00 on 15th.
		 * User enters PIN (day=15, 18:00) into receiver; when station transmits
		 * matching PIN, receiver triggers recording - compensating for overruns. */
		.pin_day  = 15, .pin_hour = 18, .pin_minute = 0,
		/* EON: Other Networks (Group 14A) - Full test configuration */
		.eon = {
			{
				.pi = 0x6B01, .ps = "UA News ", .pty = 1, .tp = 1, .ta = 1,
				/* AF variant 4: ON frequencies at 90.7 and 93.2 MHz */
				/* AF code = freq*10 - 875:  90.7*10-875=32, 93.2*10-875=57 */
				.af = { RDS_AF_MHZ(90.7), RDS_AF_MHZ(93.2) }, .af_count = 2,
				/* Mapped AF variant 5: 100.0 MHz (tuned) -> 90.7 MHz (ON) */
				/* Codes: 100.0*10-875=125 -> 90.7*10-875=32 */
				.mapped_af = { { .tuned = RDS_AF_MHZ(100.0), .on = RDS_AF_MHZ(90.7) } }, .mapped_af_count = 1,
				/* Linkage variant 12 */
				.linkage_la = 1, .linkage_lsn = 0x123,
				/* PIN variant 14 - Legacy cross-network recording (1980s-90s)
				 * Example: "World News" on linked station UA News (0x6B01)
				 * listed at 19:00 on 15th. Receiver could switch to linked
				 * station and record when that station's PIN matched. */
				.pin_day = 15, .pin_hour = 19, .pin_minute = 0,
				/* Broadcaster data variant 15 */
				.broadcaster_data = 0xABCD,
			},
			{
				.pi = 0x6C02, .ps = "Traffic1", .pty = 22, .tp = 1, .ta = 0,
				/* AF variant 4: ON frequencies at 94.7 and 97.2 MHz */
				/* AF code = freq*10 - 875:  94.7*10-875=72, 97.2*10-875=97 */
				.af = { RDS_AF_MHZ(94.7), RDS_AF_MHZ(97.2) }, .af_count = 2,
				/* No mapped AF for this one */
				.mapped_af_count = 0,
				/* Linkage variant 12 */
				.linkage_la = 0, .linkage_lsn = 0x456,
				/* PIN variant 14 - Legacy cross-network recording
				 * Example: "Traffic Report" on Traffic1 (0x6C02) at 07:30
				 * on 16th. Advanced receivers could auto-tune and record. */
				.pin_day = 16, .pin_hour = 7, .pin_minute = 30,
				/* Broadcaster data variant 15 */
				.broadcaster_data = 0x1234,
			},
		},
		.eon_count = 2,
	},
	{
		.name     = "USA (RBDS)",
		/*.pi       = 0xABCD,*/	/* PI=Axxx = USA/RBDS region */
		/* RBDS PI Codes (NRSC-4-B):
		 *  1000 - 994F: Computed from Call Sign (e.g. WNYC -> 796E)
		 *  9950 - 9EFF: 3-Letter Call Signs
		 *  AFxx / A0xx: Linked Stations / Regional
		 *  Bxxx, Dxxx, Exxx: Linked National Networks */
		.callsign = "WNYC",	/* Popular call sign (derived PI=796E) */
		.ps       = "OsmoRBDS",
		/* 64-char RT (max for Group 2A) - use full capacity for demo */
		.rt       = "osmocom-analog FM Radio - Open Source Broadcast FM RDBS Encoder",
		.pty      = 9,		/* Top 40 (RBDS) */
		.ptyn     = "Top 40  ",
		.tp       = 0,
		.ms       = 1,
		.ecc      = 0xA0,	/* USA (RBDS region) with PI prefix A */
		.lang     = 9,		/* English */
		.af_method_a_str = "92.5, 97.5, 102.5",
		/* Group 1A PIN - not used (0x0000 = most common value)
		 * Many US stations don't use PIN, so we demonstrate day=0 */
		.pin_day  = 0, .pin_hour = 0, .pin_minute = 0,
		/* group*_version omitted = AUTO: 0A (has AF), 2A (RT>32), 1A (has ECC) */
	},
	/* ============================================================
	 * MINIMAL PRESET - Mandatory Group 0 Only
	 * ============================================================
	 * Demonstrates: Minimum compliant RDS stream.
	 * Only Group 0B transmitted (PS name with PI repeat).
	 * No RT, No ECC, No PTYN, No EON → those groups won't transmit.
	 * Use case: Small station, minimal bandwidth.
	 * ============================================================ */
	{
		.name     = "Minimal",
		.pi       = 0x1234,
		.ps       = "MINIMAL ",
		.ms       = 1,
		.group0_version = RDS_GROUP_VERSION_B,  /* Force 0B (no AF) */
		/* All other fields omitted = 0 = no data → only Group 0 transmits */
	},
	/* ============================================================
	 * MOBILE PRESET - B Versions for Fast PI Identification
	 * ============================================================
	 * Demonstrates: All B versions for improved mobile reception.
	 * PI repeat in Block C of every group helps receivers lock faster.
	 * Trade-off: Less data capacity (32-char RT, no ECC).
	 * Use case: Mobile/in-car listening, weak signal areas.
	 * ============================================================ */
	{
		.name     = "Mobile",
		.pi       = 0x5678,
		.ps       = "MOBILE  ",
		.rt       = "Fast cycling RadioText",  /* ≤32 chars for 2B */
		.pty      = 10,
		.ms       = 1,
		.group0_version = RDS_GROUP_VERSION_B,  /* 0B: PI repeat */
		.group2_version = RDS_GROUP_VERSION_B,  /* 2B: 32-char, faster */
		/* No ECC/Lang → Group 1 won't transmit */
	},
	/* ============================================================
	 * MIXED PRESET - AF List + Fast RadioText (0A + 2B)
	 * ============================================================
	 * Demonstrates: Mixed A/B for different group types.
	 * 0A for AF list (need Block C for frequencies).
	 * 2B for fast RadioText cycling (trade 64→32 chars for speed).
	 * Use case: Regional station with AF, wants quick RT updates.
	 * ============================================================ */
	{
		.name     = "AF + Fast RT",
		.pi       = 0x9ABC,
		.ps       = "MIXED   ",
		.rt       = "Quick updates via 2B",
		.pty      = 3,
		.ms       = 1,
		.af_method_a_str = "90.5, 93.5, 96.5",
		.group0_version = RDS_GROUP_VERSION_A,  /* 0A: need AF list */
		.group2_version = RDS_GROUP_VERSION_B,  /* 2B: faster RT cycling */
	},
	/* ============================================================
	 * AUTO DEMO PRESET - Let Data Decide A/B Versions
	 * ============================================================
	 * Demonstrates: Auto-detection from data (all version fields = 0).
	 * - No AF → auto-selects 0B
	 * - RT ≤32 chars → auto-selects 2B
	 * - No ECC/Lang → auto-selects 1B (if PIN set)
	 * Use case: Understanding auto-detection behavior.
	 * ============================================================ */
	{
		.name     = "Auto Demo",
		.pi       = 0xDEF0,
		.ps       = "AUTODEMO",
		.rt       = "Short text for 2B",  /* ≤32 chars → auto: 2B */
		.pty      = 5,
		.ms       = 1,
		.pin_day  = 20, .pin_hour = 12, .pin_minute = 0,  /* PIN set → Group 1 transmits */
		/* No AF → auto: 0B
		 * No ECC/Lang → auto: 1B (PIN only)
		 * RT ≤32 → auto: 2B
		 * group*_version all 0 = AUTO */
	},
	{
		.name     = "Method B Test",
		.pi       = 0xAFB1,
		.ps       = "AFMETHB ",
		.rt       = "Testing RDS AF Method B with regional variants",
		.pty      = 15,
		.ecc      = 0xE0,
		.lang     = 9,
		.ms       = 1,
		/* AF Method B: Example from IEC 62106 S3.2.1.6.4
		 * List 1: Tuning 89.3 MHz, AFs: 99.5, 101.7, 88.8 (same), R102.6, R89.0 (regional)
		 * List 2: Tuning 99.5 MHz, AFs: 89.3, 100.9 (same), R104.8, R89.1 (regional) */
		.af_method_b_str = {
			"T89.3, 99.5, 101.7, 88.8, R102.6, R89.0",
			"T99.5, 89.3, 100.9, R104.8, R89.1",
		},
		.af_method_b_count = 2,
		.group0_version = RDS_GROUP_VERSION_A,
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - German (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: German umlauts and ß from RDS charset.
	 * Characters: ä (0x91), ö (0x97), ü (0x99), ß (0x8D)
	 *             Ä (0xD1), Ö (0xD7), Ü (0xD9)
	 * ============================================================ */
	{
		.name     = "German Demo",
		.pi       = 0xD314,
		.ps       = "WÜRZBÜRG",
		.rt       = "Größe, Müller, Schöne Grüße! Fünf Äpfel für Österreich.",
		.pty      = 10,
		.ptyn     = "Größe   ",	/* PTYN with ö, ß */
		.ms       = 1,
		.ecc      = 0xE0,	/* Germany */
		.lang     = 8,		/* German */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - French (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: French accented chars from RDS charset.
	 * Characters: à (0x81), é (0x82), è (0x83), ê (0x92), ë (0x93)
	 *             î (0x94), ï (0x95), ô (0x96), û (0x98), ù (0x89)
	 *             ç (0x9B), œ (0xF3)
	 * ============================================================ */
	{
		.name     = "French Demo",
		.pi       = 0xF201,
		.ps       = "CAFÉ  FM",
		.rt       = "Bienvenue à Noël! Très bel été, où êtes-vous? Ça va!",
		.pty      = 14,		/* Classical */
		.ptyn     = "Évén't  ",	/* PTYN with é */
		.ms       = 1,
		.ecc      = 0xE1,	/* France */
		.lang     = 15,		/* French */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - Spanish + Euro (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: Spanish ñ and symbols from RDS charset.
	 * Characters: ñ (0x9A), Ñ (0x8A), € (0xA9), ¿ (0xB9), ¡ (0x8E)
	 * ============================================================ */
	{
		.name     = "Spanish Demo",
		.pi       = 0xE502,
		.ps       = "ESPAÑA  ",
		.rt       = "¡Buenas Señor! ¿Cuánto? Mañana €100. ¡Niño pequeño!",
		.pty      = 6,		/* Drama */
		.ptyn     = "Señales ",	/* PTYN with ñ */
		.ms       = 1,
		.ecc      = 0xE2,	/* Spain */
		.lang     = 14,		/* Spanish */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - Nordic (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: Norwegian/Danish/Swedish special characters.
	 * Characters: Å (0xE1), Æ (0xE2), Ø (0xE7)
	 *             å (0xF1), æ (0xF2), ø (0xF7)
	 * ============================================================ */
	{
		.name     = "Nordic Demo",
		.pi       = 0xF503,
		.ps       = "ÅRHUS FM",
		.rt       = "Velkommen! Ål, Æble, Øl fra København. Blåbær og Rødgrød!",
		.pty      = 12,		/* Light classical */
		.ptyn     = "Søndags ",	/* Sunday in Danish with ø */
		.ms       = 1,
		.ecc      = 0xE2,	/* Denmark/Norway */
		.lang     = 7,		/* Danish */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - Greek (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: Greek letters available in RDS charset.
	 * Note: RDS Annex E has very limited Greek: α (0xA1), π (0xA8)
	 * Real Greek stations use Latin transliteration for RDS.
	 * ============================================================ */
	{
		.name     = "Greek Demo",
		.pi       = 0x1F01,
		.ps       = "ATHINA  ",
		.rt       = "Kalimera! To α kai to π einai ellinika. FM Ellada!",
		.pty      = 11,		/* Rock music */
		.ptyn     = "Mousiki ",	/* Music in transliterated Greek */
		.ms       = 1,
		.ecc      = 0xE1,	/* Greece */
		.lang     = 18,		/* Greek */
	},
	/* ============================================================
	 * AF METHOD A STRING DEMO (IEC 62106 S3.2.1.6.3)
	 * ============================================================
	 * Demonstrates: New human-readable AF Method A string format.
	 * Format: "freq_mhz, freq_mhz, LFxxx, MFxxx"
	 * LF/MF count as 2 slots each (encoded as [250, code] pairs).
	 * ============================================================ */
	{
		.name     = "AF Method A Demo",
		.pi       = 0xAF01,
		.ps       = "AF DEMO ",
		.rt       = "Testing AF Method A with VHF, LF, and MF frequencies!",
		.pty      = 15,
		.ms       = 1,
		.ecc      = 0xE0,	/* Germany */
		.lang     = 8,		/* German */
		/* NEW: Human-readable AF string format
		 * 3 VHF (91.0, 95.5, 102.3) + 1 LF (225 kHz) + 1 MF (1008 kHz)
		 * Total slots: 3 + 2 + 2 = 7 slots */
		.af_method_a_str = "91.0, 95.5, 102.3, LF225, MF1008",
	},
	/* End of presets */
};

#define RDS_PRESET_COUNT (sizeof(rds_presets) / sizeof(rds_presets[0]))

static int rds_current_preset = 0;

/* Additional settings not in presets (common to all) */
static uint8_t rds_ta        = 0;		/* Traffic Announcement */

static uint8_t rds_di_artificial_head = 0;
static uint8_t rds_di_compressed = 0;
static uint8_t rds_di_dynamic_pty = 0;

/* Programme Item Number (PIN) is now defined per-preset in rds_preset_t.
 * See preset .pin_day, .pin_hour, .pin_minute fields. */
static int rds_ct_enabled    = 1;		/* Clock-Time enabled */

/* Group version selection is now per-preset via group0/1/2_version fields.
 * See rds_preset_t and RDS_GROUP_VERSION_AUTO/A/B enum. */


static char freq_name[2][64];

/* User overrides */
static uint16_t rds_user_pi = 0;
static char *rds_user_callsign = NULL;

/* Helper to load AFs from preset - supports both Method A and Method B */
static void load_preset_afs(rds_encoder_t *enc, const rds_preset_t *p)
{
	/* Clear AF structures */
	memset(&enc->af_method_a, 0, sizeof(enc->af_method_a));
	memset(&enc->af_method_b, 0, sizeof(enc->af_method_b));
	memset(enc->af_codes, 0, sizeof(enc->af_codes));
	enc->af_code_count = 0;
	enc->af_method_a_segment = 0;
	enc->af_method_b_list_idx = 0;
	enc->af_method_b_pair_idx = 0;
	enc->use_method_b = 0;
	
	/* Check for Method B lists first (takes priority over Method A) */
	if (p->af_method_b_count > 0 && p->af_method_b_str[0]) {
		for (int i = 0; i < p->af_method_b_count && i < RDS_AF_METHOD_B_MAX_LISTS; i++) {
			if (p->af_method_b_str[i] && p->af_method_b_str[i][0]) {
				if (rds_af_method_b_parse(p->af_method_b_str[i], 
				                          &enc->af_method_b.lists[i]) == 0) {
					enc->af_method_b.list_count++;
					LOGP(DRADIO, LOGL_INFO, "AF Method B[%d]: Loaded tuning=%.1f, %d AFs\n",
					     i, enc->af_method_b.lists[i].tuning_freq / 10.0,
					     enc->af_method_b.lists[i].af_count);
				}
			}
		}
		if (enc->af_method_b.list_count > 0) {
			enc->use_method_b = 1;
			LOGP(DRADIO, LOGL_INFO, "AF Method B: Enabled with %d list(s)\n", 
			     enc->af_method_b.list_count);
			return;
		}
	}
	
	/* Parse AF Method A string (human-readable format) */
	if (p->af_method_a_str && p->af_method_a_str[0]) {
		if (rds_af_method_a_parse(p->af_method_a_str, &enc->af_method_a) == 0) {
			/* Build pre-computed code sequence for transmission */
			enc->af_code_count = rds_af_method_a_build_codes(
				&enc->af_method_a, enc->af_codes, sizeof(enc->af_codes));
			
			if (enc->af_code_count > 0) {
				LOGP(DRADIO, LOGL_INFO, "AF Method A: Loaded '%s' -> %d codes (%d VHF + %d LF/MF = %d frequencies)\n",
				     p->af_method_a_str, enc->af_code_count,
				     enc->af_method_a.vhf_count, enc->af_method_a.lf_mf_count, 
				     enc->af_method_a.slot_count);
			}
		}
	}
}

/* Apply current RDS preset to encoder (for runtime switching) */
static void rds_apply_preset(radio_t *radio)
{
	const rds_preset_t *p = &rds_presets[rds_current_preset];
	rds_encoder_t *enc = &radio->rds_enc;
	
	/* Update PI (will require receiver to re-sync) */
	uint16_t pi = p->pi;

	/* If PI is 0 in preset, try to derive from callsign */
	if (pi == 0 && p->callsign) {
		pi = rds_get_pi_from_callsign(p->callsign);
	}

	if (rds_user_pi)
		pi = rds_user_pi;
	else if (rds_user_callsign) {
		uint16_t cpi = rds_get_pi_from_callsign(rds_user_callsign);
		if (cpi) pi = cpi;
	}
	enc->pi = pi;
	
	/* Update PS (IEC 62106 uses limited charset - convert UTF-8 to RDS) */
	memset(enc->ps, ' ', 8);
	enc->ps[8] = '\0';
	if (p->ps && p->ps[0] != '\0') {
		rds_validate_text(p->ps, "PS");
		int warn = 0;
		rds_encode_text(p->ps, (uint8_t *)enc->ps, 8, &warn);
	}
	
	/* Update RadioText (IEC 62106 uses limited charset - convert UTF-8 to RDS) */
	memset(enc->rt, ' ', 64);
	enc->rt[64] = '\0';
	if (p->rt && p->rt[0] != '\0') {
		rds_validate_text(p->rt, "RT");
		int warn = 0;
		int len = rds_encode_text(p->rt, (uint8_t *)enc->rt, 64, &warn);
		if (len < 64) enc->rt[len] = '\r';
	}
	enc->rt_ab = !enc->rt_ab;  /* Toggle A/B to signal text change */
	enc->rt_segment = 0;
	
	/* Update PTY and PTYN (IEC 62106 S6.1.5.8)
	 * PTYN uses same Annex E charset as RadioText - 8 chars max */
	enc->pty = p->pty & 0x1F;
	memset(enc->ptyn, ' ', 8);
	enc->ptyn[8] = '\0';
	if (p->ptyn && p->ptyn[0] != '\0') {
		/* Validate and encode UTF-8 to RDS charset */
		rds_validate_text(p->ptyn, "PTYN");
		int warn = 0;
		rds_encode_text(p->ptyn, (uint8_t *)enc->ptyn, 8, &warn);
	}
	enc->ptyn_ab = !enc->ptyn_ab;
	
	/* Update traffic and mode flags */
	enc->tp = p->tp;
	enc->ms = p->ms;
	
	/* Update DI stereo flag based on broadcast mode (-S flag) */
	enc->di_stereo = radio->stereo ? 1 : 0;
	
	/* Update country codes */
	enc->ecc = p->ecc;
	enc->language = p->lang;
	
	/* Update Group 1A PIN (Programme Item Number) for THIS station */
	enc->pin_day = p->pin_day;
	enc->pin_hour = p->pin_hour;
	enc->pin_minute = p->pin_minute;
	
	/* Update Alternative Frequencies (Unified Storage) */
	load_preset_afs(enc, p);
	
	/* Update EON (Enhanced Other Networks) - Group 14A - ALL variants */
	if (p->eon_count > 0) {
		for (int i = 0; i < p->eon_count && i < RDS_EON_MAX_ENTRIES; i++) {
			rds_eon_entry_t *eon = &enc->eon_tx[i];
			memset(eon, 0, sizeof(*eon));
			
			/* Basic info (variants 0-3, 13) */
			eon->pi = p->eon[i].pi;
			memset(eon->ps, ' ', 8);
			eon->ps[8] = '\0';
			if (p->eon[i].ps) {
				int len = strlen(p->eon[i].ps);
				if (len > 8) len = 8;
				memcpy(eon->ps, p->eon[i].ps, len);
			}
			eon->pty = p->eon[i].pty;
			eon->tp = p->eon[i].tp;
			eon->ta = p->eon[i].ta;
			
			/* AF (variant 4) - convert to 0.1 MHz format */
			if (p->eon[i].af_count > 0) {
				for (int j = 0; j < p->eon[i].af_count && j < RDS_EON_MAX_AF; j++) {
					eon->af[j] = 875 + p->eon[i].af[j];  /* Convert code to 0.1MHz */
				}
				eon->af_count = p->eon[i].af_count;
			}
			
			/* Mapped AF (variants 5-9) */
			if (p->eon[i].mapped_af_count > 0) {
				for (int j = 0; j < p->eon[i].mapped_af_count && j < RDS_EON_MAX_MAPPED_AF; j++) {
					eon->mapped_af[j].tuned_af = p->eon[i].mapped_af[j].tuned;
					eon->mapped_af[j].on_af = p->eon[i].mapped_af[j].on;
				}
				eon->mapped_af_count = p->eon[i].mapped_af_count;
			}
			
			/* Linkage (variant 12) */
			eon->linkage_la = p->eon[i].linkage_la;
			eon->linkage_lsn = p->eon[i].linkage_lsn;
			
			/* PIN (variant 14) */
			eon->pin_day = p->eon[i].pin_day;
			eon->pin_hour = p->eon[i].pin_hour;
			eon->pin_minute = p->eon[i].pin_minute;
			
			/* Broadcaster data (variant 15) */
			eon->broadcaster_data = p->eon[i].broadcaster_data;
		}
		enc->eon_tx_count = p->eon_count;
		enc->eon_enabled = 1;
		enc->eon_tx_index = 0;
		enc->eon_tx_variant = 0;
	} else {
		enc->eon_tx_count = 0;
		enc->eon_enabled = 0;
	}
	
	/* Reset PS segment to restart transmission */
	enc->ps_segment = 0;
	
	/* Reset warmup mode: Group 0 only for ~5 seconds (57 groups @ 11.4/sec) */
	enc->warmup_countdown = 57;
	
	/* --------------------------------------------------------
	 * Group Version Selection (A vs B)
	 * --------------------------------------------------------
	 * Priority: Manual override > Auto-detection
	 * AUTO (0) = detect from data, A (1) = force A, B (2) = force B
	 * -------------------------------------------------------- */
	
	/* Group 0: 0A (with AF) vs 0B (PI repeat, no AF) */
	if (p->group0_version == RDS_GROUP_VERSION_AUTO)
		enc->use_0b = (p->af_method_a_str == NULL || p->af_method_a_str[0] == '\0');  /* No AF → 0B */
	else
		enc->use_0b = (p->group0_version == RDS_GROUP_VERSION_B);
	
	/* Group 1: 1A (ECC/Lang/PIN) vs 1B (PIN only) */
	if (p->group1_version == RDS_GROUP_VERSION_AUTO)
		enc->use_1b = (p->ecc == 0 && p->lang == 0);  /* No ECC/Lang → 1B */
	else
		enc->use_1b = (p->group1_version == RDS_GROUP_VERSION_B);
	
	/* Group 2: 2A (64-char RT) vs 2B (32-char, faster) */
	if (p->group2_version == RDS_GROUP_VERSION_AUTO) {
		size_t rt_len = p->rt ? strlen(p->rt) : 0;
		enc->use_2b = (rt_len > 0 && rt_len <= 32);  /* Short RT → 2B */
	} else {
		enc->use_2b = (p->group2_version == RDS_GROUP_VERSION_B);
	}
	
	/* Debug test mode */
	LOGP(DRADIO, LOGL_INFO, "RDS Preset: %s (PI=%04X) Groups: %s/%s/%s\n",
	     p->name, enc->pi,
	     enc->use_0b ? "0B" : "0A",
	     enc->use_1b ? "1B" : "1A",
	     enc->use_2b ? "2B" : "2A");
	     
	/* Rebuild group scheduler to reflect new configuration (e.g. enable/disable 10A PTYN) */
	rds_scheduler_update(enc);
}

/* Cycle to next RDS preset */
void rds_next_preset(radio_t *radio)
{
	rds_current_preset = (rds_current_preset + 1) % RDS_PRESET_COUNT;
	rds_apply_preset(radio);
}

int radio_init(radio_t *radio, int buffer_size, int samplerate, double frequency, const char *tx_wave_file, const char *rx_wave_file, const char *tx_audiodev, const char *rx_audiodev, enum modulation modulation, double bandwidth, double deviation, double modulation_index, double time_constant_us, double volume, int stereo, int rds, int rds2, int sca_67k, int sca_92k, int rds_debug, int rds_verbose)
{
	int rc = -EINVAL;
	double safe_scaler = 1.0;


	/* 
	 * GAIN SCALING & CLIPPER SETUP
	 * ----------------------------
	 * FM broadcast uses pre-emphasis to boost high frequencies before transmission,
	 * which are then de-emphasized on receive. This creates a headroom problem:
	 *
	 * Pre-emphasis boost at 50us (European) / 75us (US):
	 *   500 Hz: +3 dB,  1 kHz: +6 dB,  5 kHz: +14 dB,  15 kHz: +17 dB
	 *
	 * IMPORTANT: Input levels should be reduced to avoid clipping!
	 * - Use -V 0.5 or lower for full-scale input (e.g., test tones)
	 * - Music/speech with typical dynamics usually works at -V 0.8
	 * - The soft clipper will activate on peaks, creating odd harmonics
	 *
	 * 1. Headroom for Pilot/RDS subcarriers
	 */
	if (stereo)
		safe_scaler -= 0.10; /* Reserve 10% for 19 kHz Pilot Tone */
	if (rds || rds2)
		safe_scaler -= 0.05; /* Reserve 5% for 57 kHz RDS Subcarrier */

	/* 
	 * 2. Pre-emphasis Gain Strategy (Standard Broadcast Practice):
	 *    - Normalize for unity gain at 1 kHz reference tone
	 *    - High-frequency peaks (>5 kHz) will hit the soft clipper
	 *    - This maximizes loudness while clipper prevents over-deviation
	 */

	if (safe_scaler < 1.0) {
		LOGP(DRADIO, LOGL_NOTICE, "Auto-scaling input volume by %.3f to reserve headroom for Pilot/RDS.\n", safe_scaler);
		volume *= safe_scaler;
	}

	/* Soft clipper at 1.0 (maximum deviation). Peaks exceeding this are limited. */
	clipper_init(1.0);

	memset(radio, 0, sizeof(*radio));
	radio->buffer_size = buffer_size;
	radio->volume = volume;
	radio->stereo = stereo;
	radio->rds = rds;
	radio->rds2 = rds2;
	radio->sca_67k = sca_67k;
	radio->sca_92k = sca_92k;
	radio->tx_wave_file = tx_wave_file;
	radio->modulation = modulation;
	radio->signal_samplerate = samplerate;
	radio->audio_bandwidth = bandwidth;

	switch (radio->modulation) {
	case MODULATION_FM:
		radio->fm_deviation = deviation;
		radio->signal_bandwidth = deviation + bandwidth;
		if (radio->stereo) {
			radio->signal_bandwidth = deviation + 53000.0;
			radio->audio_bandwidth = STEREO_BW;
		}
		if (radio->rds)
			radio->signal_bandwidth = deviation + 60000.0;
		if (radio->rds2)
			radio->signal_bandwidth = deviation + 80000.0;
		/* SCA extends bandwidth further */
		if (radio->sca_67k)
			radio->signal_bandwidth = deviation + 75000.0;
		if (radio->sca_92k)
			radio->signal_bandwidth = deviation + 100000.0;
		break;
	case MODULATION_AM_DSB:
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		/* level is 1.0, which is full amplitude */
		radio->signal_bandwidth = bandwidth;
		break;
	case MODULATION_NONE:
		LOGP(DRADIO, LOGL_ERROR, "Wrong modulation, please fix!\n");
		goto error;
	}

	if (tx_wave_file) {
		/* open wave file */
		int _samplerate = 0;
		radio->tx_audio_channels = 0;
		rc = wave_create_playback(&radio->wave_tx_play, tx_wave_file, &_samplerate, &radio->tx_audio_channels, 1.0);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to create WAVE playback instance!\n");
			goto error;
		}
		if (radio->tx_audio_channels != 1 && radio->tx_audio_channels != 2)
		{
			LOGP(DRADIO, LOGL_ERROR, "WAVE file must have one or two channels!\n");
			goto error;
		}
		radio->tx_audio_samplerate = _samplerate;
		radio->tx_audio_mode = AUDIO_MODE_WAVEFILE;
	} else if (tx_audiodev) {
#ifdef HAVE_ALSA
		/* open audio device */
		radio->tx_audio_samplerate = 48000;
		radio->tx_audio_channels = (stereo) ? 2 : 1;
		radio->tx_sound = sound_open(SOUND_DIR_REC, tx_audiodev, NULL, NULL, NULL, radio->tx_audio_channels, 0.0, radio->tx_audio_samplerate, radio->buffer_size, 1.0, 1.0, 0.0, 2.0);
		if (!radio->tx_sound) {
			rc = -EIO;
			LOGP(DRADIO, LOGL_ERROR, "Failed to open sound device!\n");
			goto error;
		}
		jitter_create(&radio->tx_dejitter[0], "left", radio->tx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		jitter_create(&radio->tx_dejitter[1], "right", radio->tx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		radio->tx_audio_mode = AUDIO_MODE_AUDIODEV;
#else
		rc = -ENOTSUP;
		LOGP(DRADIO, LOGL_ERROR, "No sound card support compiled in!\n");
		goto error;
#endif
	} else {
		int i;
		double phase;
		/* use built-in sample sound */
		radio->tx_audio_samplerate = samplerate;
		radio->tx_audio_channels = (radio->stereo) ? 2 : 1;
		radio->testtone_length = radio->tx_audio_samplerate;
		radio->testtone[0] = calloc(radio->testtone_length * 2, sizeof(sample_t));
		if (!radio->testtone[0]) {
			rc = -ENOMEM;
			LOGP(DRADIO, LOGL_ERROR, "Failed to allocate test sound buffer!\n");
			goto error;
		}
		radio->testtone[1] = radio->testtone[0] + radio->testtone_length;
		/* generate tone */
		phase = 2.0 * M_PI * 1000.0 / radio->tx_audio_samplerate;
		if (radio->stereo) {
			/* Stereo test: L=1kHz, R=400Hz for clear separation verification
			 * This creates constant L-R content for strong 38kHz subcarrier
			 * Unlike alternating L/R, this gives continuous stereo modulation */
			double phase_l = 2.0 * M_PI * 1000.0 / radio->tx_audio_samplerate;
			double phase_r = 2.0 * M_PI * 400.0 / radio->tx_audio_samplerate;
			for (i = 0; i < radio->testtone_length; i++) {
				radio->testtone[0][i] = sin(i * phase_l);  /* Left: 1 kHz */
				radio->testtone[1][i] = sin(i * phase_r);  /* Right: 400 Hz */
			}
		} else {
			for (i = 0; i < radio->testtone_length; i++) {
				radio->testtone[0][i] = sin(i * phase);
			}
		}
		radio->tx_audio_mode = AUDIO_MODE_TESTTONE;
	}

	if (rx_wave_file) {
		/* open wave file */
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (radio->stereo) ? 2 : 1;
		rc = wave_create_record(&radio->wave_rx_rec, rx_wave_file, radio->rx_audio_samplerate, radio->rx_audio_channels, 1.0);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to create WAVE record instance!\n");
			goto error;
		}
		radio->rx_audio_mode |= AUDIO_MODE_WAVEFILE;
	}
	if (rx_audiodev) {
#ifdef HAVE_ALSA
		/* open audio device */
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (stereo) ? 2 : 1;
		/* check if we use same device */
		radio->rx_sound = sound_open(SOUND_DIR_PLAY, rx_audiodev, NULL, NULL, NULL, radio->rx_audio_channels, 0.0, radio->rx_audio_samplerate, radio->buffer_size, 1.0, 1.0, 0.0, 2.0);
		if (!radio->rx_sound) {
			rc = -EIO;
			LOGP(DRADIO, LOGL_ERROR, "Failed to open sound device!\n");
			goto error;
		}
		jitter_create(&radio->rx_dejitter[0], "left", radio->rx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		jitter_create(&radio->rx_dejitter[1], "right", radio->rx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		radio->rx_audio_mode |= AUDIO_MODE_AUDIODEV;
#else
		rc = -ENOTSUP;
		LOGP(DRADIO, LOGL_ERROR, "No sound card support compiled in!\n");
		goto error;
#endif
	}
	/* if no sink was selected, we use dummy settings */
	if (!rx_wave_file && !rx_audiodev) {
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (stereo) ? 2 : 1;
	}

	/* check if sample rate is too low */
	if (radio->tx_audio_samplerate > radio->signal_samplerate) {
		rc = -EINVAL;
		LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your audio sample rate is %.0f.\n", radio->signal_samplerate, radio->tx_audio_samplerate);
		LOGP(DRADIO, LOGL_ERROR, "Please select a sample rate that is higher or equal the audio sample rate!\n");
		goto error;
	}
	if (radio->rx_audio_samplerate > radio->signal_samplerate) {
		rc = -EINVAL;
		LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your audio sample rate is %.0f.\n", radio->signal_samplerate, radio->rx_audio_samplerate);
		LOGP(DRADIO, LOGL_ERROR, "Please select a sample rate that is higher or equal the audio sample rate!\n");
		goto error;
	}
	if (radio->signal_samplerate < radio->signal_bandwidth * 2 / 0.75) {
		rc = -EINVAL;
		LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your signal's bandwidth %.0f.\n", radio->signal_samplerate, radio->signal_bandwidth);
		LOGP(DRADIO, LOGL_ERROR, "Your signal processing sample rate must be at least one third greater than the signal's double bandwidth. Use at least %.0f.\n", radio->signal_bandwidth * 2.0 / 0.75);
		goto error;
	}

	iir_highpass_init(&radio->tx_dc_removal[0], DC_CUTOFF, radio->tx_audio_samplerate, 1);
	iir_highpass_init(&radio->tx_dc_removal[1], DC_CUTOFF, radio->tx_audio_samplerate, 1);

	/* init DC blocker state */
	radio->tx_dc_prev_x[0] = 0.0;
	radio->tx_dc_prev_x[1] = 0.0;
	radio->tx_dc_prev_y[0] = 0.0;
	radio->tx_dc_prev_y[1] = 0.0;

	/* stereo pilot tone phase */
	radio->pilot_phasestep = 2.0 * M_PI * PILOT_FREQ / radio->signal_samplerate;

	/* stere decoding filters */
	iir_lowpass_init(&radio->rx_lp_pilot_I, PILOT_BW, radio->signal_samplerate, 2);
        iir_lowpass_init(&radio->rx_lp_pilot_Q, PILOT_BW, radio->signal_samplerate, 2);
        iir_lowpass_init(&radio->rx_lp_sum, STEREO_BW, radio->signal_samplerate, 2);
        iir_lowpass_init(&radio->rx_lp_diff, STEREO_BW, radio->signal_samplerate, 2);

	/* init sample rate conversion, use complete bandwidth for resample filter */
	/* 
	 * RECONSTRUCTION/ANTI-ALIASING FILTER
	 * -----------------------------------
	 * We use a strict 15kHz cutoff (Standard FM Bandwidth) for the upsampler.
	 * 
	 * JUSTIFICATION:
	 * 1. The previous default (audio_samplerate / 2) allowed ultrasonic images to pass 
	 *    through the 2nd order filter.
	 * 2. These ultrasonic images (e.g. at 24kHz+) folded back into the audible band 
	 *    during SDR modulation, creating "8-bit like" hiss and intermodulation noise.
	 * 3. 15kHz creates a clean, hard stop before the 19kHz stereo pilot, protecting
	 *    the pilot from interference and the audio from aliasing.
	 * 
	 * Note: Filter order was also increased to 4 in libsamplerate/samplerate.c
	 */
	rc = init_samplerate(&radio->tx_resampler[0], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;
	rc = init_samplerate(&radio->tx_resampler[1], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;
	rc = init_samplerate(&radio->rx_resampler[0], radio->rx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;
	rc = init_samplerate(&radio->rx_resampler[1], radio->rx_audio_samplerate, radio->signal_samplerate, 15000.0);
	if (rc < 0)
		goto error;

	/* init display of wave form */
	sprintf(freq_name[0], "%.4f MHz", frequency / 1e6);
	display_wave_init(&radio->dispwav[0], radio->rx_audio_samplerate, freq_name[0]);

	/* init filters (using signal sample rate) */
	switch (radio->modulation) {
	case MODULATION_FM:
		if (time_constant_us > 0.0) {
			radio->emphasis = 1;
			LOGP(DRADIO, LOGL_INFO, "Using emphasis cut-off at %.0f Hz.\n", timeconstant2cutoff(time_constant_us));
		}
	/* init emphasis */
	if (radio->emphasis) {
		double tau = time_constant_us / 1e6;
		if (fm_fast_math_enabled()) {
			/* Initialize optimized 1st-order emphasis filters (TX/RX) */
			init_emphasis_fast(&radio->fm_emphasis_fast[0], radio->signal_samplerate, tau, 12000.0);
			/* Initialize RX DC blocking filter (30 Hz cutoff - low enough to preserve bass) */
			init_dc_filter_fast(&radio->rx_dc_filter[0], radio->signal_samplerate, DC_CUTOFF);
			if (radio->stereo) {
				init_emphasis_fast(&radio->fm_emphasis_fast[1], radio->signal_samplerate, tau, 12000.0);
				init_dc_filter_fast(&radio->rx_dc_filter[1], radio->signal_samplerate, DC_CUTOFF);
			}
		} else {
			/* time constant - convert from µs to seconds */
			init_emphasis(&radio->fm_emphasis[0], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
			if (radio->stereo)
				init_emphasis(&radio->fm_emphasis[1], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
		}
	}
		rc = fm_mod_init(&radio->fm_mod, radio->signal_samplerate, 0.0, 1.0);
		if (rc < 0)
			goto error;
		rc = fm_demod_init(&radio->fm_demod, radio->signal_samplerate, 0.0, 2 * radio->signal_bandwidth);
		if (rc < 0)
			goto error;
		if (stereo) {
			sprintf(freq_name[0], "%.4f MHz left", frequency / 1e6);
			sprintf(freq_name[1], "%.4f MHz right", frequency / 1e6);
			display_wave_init(&radio->dispwav[1], samplerate, freq_name[1]);
		}
		/* Initialize RDS encoder if enabled */
		if (rds || rds2) {
			/* Select preset based on emphasis (heuristic for region):
			 * Only 50µs → USA (RBDS) - preset 1
			 * All other → RDS (default, more common globally) - preset 0 */
			if (RDS_IS_RBDS_EMPHASIS(time_constant_us)) {
				rds_current_preset = 1;  /* USA/RBDS (50µs) */
				LOGP(DRADIO, LOGL_INFO, "Emphasis %.0fµs detected: using RBDS (USA) preset\n", time_constant_us);
			} else {
				rds_current_preset = 0;  /* RDS (default, more common globally) */
				LOGP(DRADIO, LOGL_INFO, "Emphasis %.0fµs detected: using RDS preset (default)\n", time_constant_us);
			}
			const rds_preset_t *p = &rds_presets[rds_current_preset];
			
			uint16_t pi = p->pi;

			/* If PI is 0 in preset, try to derive from callsign */
			if (pi == 0 && p->callsign) {
				pi = rds_get_pi_from_callsign(p->callsign);
			}

			if (rds_user_pi)
				pi = rds_user_pi;
			else if (rds_user_callsign) {
				uint16_t cpi = rds_get_pi_from_callsign(rds_user_callsign);
				if (cpi) pi = cpi;
			}

			rds_encoder_init(&radio->rds_enc, radio->signal_samplerate,
				pi, p->ps, p->rt, p->pty, p->ptyn);
			
			/* Apply preset configuration to encoder */
			radio->rds_enc.debug = rds_debug;
			radio->rds_enc.verbose = rds_verbose;
			radio->rds_enc.tp = p->tp;
			radio->rds_enc.ta = rds_ta;
			radio->rds_enc.ms = p->ms;
			
			/* Decoder Identification (DI) flags */
			radio->rds_enc.di_stereo = stereo ? 1 : 0;
			radio->rds_enc.di_artificial_head = rds_di_artificial_head;
			radio->rds_enc.di_compressed = rds_di_compressed;
			radio->rds_enc.di_dynamic_pty = rds_di_dynamic_pty;
			
			/* Extended codes (Group 1A) */
			radio->rds_enc.ecc = p->ecc;
			radio->rds_enc.language = p->lang;
			
			/* Programme Item Number (Group 1A Block D) - from preset */
			radio->rds_enc.pin_day = p->pin_day;
			radio->rds_enc.pin_hour = p->pin_hour;
			radio->rds_enc.pin_minute = p->pin_minute;
			
			/* Clock-Time (Group 4A) */
			radio->rds_enc.ct_enabled = rds_ct_enabled;
			
			/* Alternative Frequencies (Group 0A Block C) */
			load_preset_afs(&radio->rds_enc, p);
			
			/* EON: Enhanced Other Networks (Group 14A) - ALL variants */
			if (p->eon_count > 0) {
				for (int i = 0; i < p->eon_count && i < RDS_EON_MAX_ENTRIES; i++) {
					rds_eon_entry_t *eon = &radio->rds_enc.eon_tx[i];
					memset(eon, 0, sizeof(*eon));
					
					/* Basic info (variants 0-3, 13) */
					eon->pi = p->eon[i].pi;
					memset(eon->ps, ' ', 8);
					eon->ps[8] = '\0';
					if (p->eon[i].ps) {
						int len = strlen(p->eon[i].ps);
						if (len > 8) len = 8;
						memcpy(eon->ps, p->eon[i].ps, len);
					}
					eon->pty = p->eon[i].pty;
					eon->tp = p->eon[i].tp;
					eon->ta = p->eon[i].ta;
					
					/* AF (variant 4) - convert to 0.1 MHz format */
					if (p->eon[i].af_count > 0) {
						for (int j = 0; j < p->eon[i].af_count && j < RDS_EON_MAX_AF; j++) {
							eon->af[j] = 875 + p->eon[i].af[j];
						}
						eon->af_count = p->eon[i].af_count;
					}
					
					/* Mapped AF (variants 5-9) */
					if (p->eon[i].mapped_af_count > 0) {
						for (int j = 0; j < p->eon[i].mapped_af_count && j < RDS_EON_MAX_MAPPED_AF; j++) {
							eon->mapped_af[j].tuned_af = p->eon[i].mapped_af[j].tuned;
							eon->mapped_af[j].on_af = p->eon[i].mapped_af[j].on;
						}
						eon->mapped_af_count = p->eon[i].mapped_af_count;
					}
					
					/* Linkage (variant 12) */
					eon->linkage_la = p->eon[i].linkage_la;
					eon->linkage_lsn = p->eon[i].linkage_lsn;
					
					/* PIN (variant 14) */
					eon->pin_day = p->eon[i].pin_day;
					eon->pin_hour = p->eon[i].pin_hour;
					eon->pin_minute = p->eon[i].pin_minute;
					
					/* Broadcaster data (variant 15) */
					eon->broadcaster_data = p->eon[i].broadcaster_data;
				}
				radio->rds_enc.eon_tx_count = p->eon_count;
				radio->rds_enc.eon_enabled = 1;
				LOGP(DRADIO, LOGL_INFO, "RDS EON: %d Other Networks configured (all variants)\n", p->eon_count);
			}
			
			/* Group version selection (A vs B) with auto-detection
			 * Priority: Manual override > Auto-detection */
			
			/* Group 0: 0A (with AF) vs 0B (PI repeat, no AF) */
			if (p->group0_version == RDS_GROUP_VERSION_AUTO)
				radio->rds_enc.use_0b = (p->af_method_a_str == NULL || p->af_method_a_str[0] == '\0');
			else
				radio->rds_enc.use_0b = (p->group0_version == RDS_GROUP_VERSION_B);
			
			/* Group 1: 1A (ECC/Lang/PIN) vs 1B (PIN only) */
			if (p->group1_version == RDS_GROUP_VERSION_AUTO)
				radio->rds_enc.use_1b = (p->ecc == 0 && p->lang == 0);
			else
				radio->rds_enc.use_1b = (p->group1_version == RDS_GROUP_VERSION_B);
			
			/* Group 2: 2A (64-char RT) vs 2B (32-char, faster) */
			if (p->group2_version == RDS_GROUP_VERSION_AUTO) {
				size_t rt_len = p->rt ? strlen(p->rt) : 0;
				radio->rds_enc.use_2b = (rt_len > 0 && rt_len <= 32);
			} else {
				radio->rds_enc.use_2b = (p->group2_version == RDS_GROUP_VERSION_B);
			}
			
			/* Debug test mode */
			LOGP(DRADIO, LOGL_INFO, "RDS Preset: %s (PI=%04X) Groups: %s/%s/%s\n",
			     p->name, radio->rds_enc.pi,
			     radio->rds_enc.use_0b ? "0B" : "0A",
			     radio->rds_enc.use_1b ? "1B" : "1A",
			     radio->rds_enc.use_2b ? "2B" : "2A");
			
			rds_decoder_init(&radio->rds_dec, radio->signal_samplerate, rds_debug, rds_verbose, time_constant_us);

			
			/*
			 * TODO: RDS2 Encoder Initialization (IEC 62106-2:2021)
			 * ----------------------------------------------------
			 * If rds2 flag is set, initialize additional encoder for streams 2-4:
			 *
			 *   if (rds2) {
			 *       rds2_encoder_init(&radio->rds2_enc, radio->signal_samplerate);
			 *   }
			 *
			 * The rds2_encoder_t would handle:
			 * - Stream 2:  66.5 kHz subcarrier (independent NCO)
			 * - Stream 3:  71.25 kHz subcarrier (independent NCO)
			 * - Stream 4:  76 kHz subcarrier (4 x pilot, can be locked)
			 * - Group Type C encoding for extended data
			 * - UTF-8 Extended RadioText (128 bytes)
			 * - RDS2 File Transfer (RFT) protocol
			 *
			 * Note: The radio_t struct would need: rds2_encoder_t rds2_enc;
			 * See rds.c for stub implementation.
			 */
		}
		/* Initialize SCA encoder/decoder if enabled */
		if (sca_67k || sca_92k) {
			sca_encoder_init(&radio->sca_enc, radio->signal_samplerate, sca_67k, sca_92k);
			sca_decoder_init(&radio->sca_dec, radio->signal_samplerate, sca_67k, sca_92k);
		}
		break;
	case MODULATION_AM_DSB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		/* modulation index 0.0 = no envelope, bias 1.0
		 * modulation index 1.0 = envelope +-0.5, bias 0.5
		 * modulation index 0.5 = envelope +-0.25, bias 0.75
		 */
		double gain = modulation_index / 2.0;
		double bias = 1.0 - gain;
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, gain, bias);
		if (rc < 0)
			goto error;
		rc = am_demod_init(&radio->am_demod, radio->signal_samplerate, 0.0, radio->signal_bandwidth, 1.0 / modulation_index);
		if (rc < 0)
			goto error;
		break;
	case MODULATION_AM_USB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, 1.0, 0.0);
		if (rc < 0)
			goto error;
		break;
	case MODULATION_AM_LSB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, 1.0, 0.0);
		if (rc < 0)
			goto error;
		break;

	default:
		break;
	}
	
	if (radio->tx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio source is %.0f Hz.\n", radio->tx_audio_samplerate / 2.0);
	if (radio->rx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio sink is %.0f Hz.\n", radio->rx_audio_samplerate / 2.0);
	LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio signal is %.0f Hz.\n", radio->audio_bandwidth);
	LOGP(DRADIO, LOGL_INFO, "Bandwidth of modulated signal is %.0f Hz.\n", radio->signal_bandwidth);
	if (radio->tx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Sample rate of audio source is %.0f Hz.\n", radio->tx_audio_samplerate);
	if (radio->rx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Sample rate of audio sink is %.0f Hz.\n", radio->rx_audio_samplerate);
	LOGP(DRADIO, LOGL_INFO, "Sample rate of signal is %.0f Hz.\n", radio->signal_samplerate);

	/* one or two audio channels */
	if (radio->tx_audio_channels != 1 && radio->tx_audio_channels != 2)
	{
		LOGP(DRADIO, LOGL_ERROR, "Wrong number of audio channels, please fix!\n");
		goto error;
	}

	/* audio buffers: how many sample for audio (rounded down) */
	int tx_size = (int)((double)buffer_size / radio->tx_resampler[0].factor);
	int rx_size = (int)((double)buffer_size / radio->rx_resampler[0].factor);
	if (tx_size > rx_size)
		radio->audio_buffer_size = tx_size;
	else
		radio->audio_buffer_size = rx_size;
	radio->audio_buffer = calloc(radio->audio_buffer_size * 2, sizeof(*radio->audio_buffer));
	if (!radio->audio_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* signal buffers */
	radio->signal_buffer_size = buffer_size;
	radio->signal_buffer = calloc(radio->signal_buffer_size * 3, sizeof(*radio->signal_buffer));
	radio->signal_power_buffer = calloc(radio->signal_buffer_size, sizeof(*radio->signal_power_buffer));
	if (!radio->signal_buffer || !radio->signal_power_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* temporary I/Q/carrier buffers, used while demodulating */
	radio->I_buffer = calloc(buffer_size, sizeof(*radio->I_buffer));
	radio->Q_buffer = calloc(buffer_size, sizeof(*radio->Q_buffer));
	radio->carrier_buffer = calloc(buffer_size, sizeof(*radio->carrier_buffer));
	if (!radio->I_buffer || !radio->Q_buffer || !radio->carrier_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	return 0;

error:
	radio_exit(radio);
	return rc;
}

void radio_exit(radio_t *radio)
{
	if (radio->audio_buffer) {
		free(radio->audio_buffer);
		radio->audio_buffer = NULL;
	}
	if (radio->signal_buffer) {
		free(radio->signal_buffer);
		radio->signal_buffer = NULL;
	}
	if (radio->signal_power_buffer) {
		free(radio->signal_power_buffer);
		radio->signal_power_buffer = NULL;
	}
	if (radio->I_buffer) {
		free(radio->I_buffer);
		radio->I_buffer = NULL;
	}
	if (radio->Q_buffer) {
		free(radio->Q_buffer);
		radio->Q_buffer = NULL;
	}
	if (radio->carrier_buffer) {
		free(radio->carrier_buffer);
		radio->carrier_buffer = NULL;
	}
	if (radio->tx_audio_mode == AUDIO_MODE_WAVEFILE) {
		wave_destroy_playback(&radio->wave_tx_play);
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if ((radio->rx_audio_mode & AUDIO_MODE_WAVEFILE)) {
		wave_destroy_record(&radio->wave_rx_rec);
		radio->rx_audio_mode = AUDIO_MODE_NONE;
	}
#ifdef HAVE_ALSA
	if (radio->tx_sound) {
		sound_close(radio->tx_sound);
		/* if same device was used */
		if (radio->tx_sound == radio->rx_sound)
			radio->rx_sound = NULL;
		radio->tx_sound = NULL;
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if (radio->rx_sound) {
		sound_close(radio->rx_sound);
		radio->rx_sound = NULL;
		radio->rx_audio_mode = AUDIO_MODE_NONE;
	}
#endif
	jitter_destroy(&radio->tx_dejitter[0]);
	jitter_destroy(&radio->tx_dejitter[1]);
	jitter_destroy(&radio->rx_dejitter[0]);
	jitter_destroy(&radio->rx_dejitter[1]);
	if (radio->tx_audio_mode == AUDIO_MODE_TESTTONE) {
		free(radio->testtone[0]);
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if (radio->modulation == MODULATION_FM)
		fm_mod_exit(&radio->fm_mod);
	else
		am_mod_exit(&radio->am_mod);
}

int radio_start(radio_t __attribute__((unused)) *radio)
{
#ifdef HAVE_ALSA
	int rc;

	/* start rx sound */
	if (radio->rx_sound) {
		rc = sound_start(radio->rx_sound);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to start receiving from audio device..\n");
			return rc;
		}
	}

	/* start tx sound, if different device */
	if (radio->tx_sound && radio->tx_sound != radio->rx_sound)  {
		rc = sound_start(radio->tx_sound);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to start transmitting to audio device..\n");
			return rc;
		}
	}
#endif

	return 0;
}

int radio_tx(radio_t *radio, float *baseband, int signal_num)
{
	int i;
	int __attribute__((unused)) rc;
	int audio_num;
	sample_t *audio_samples[2];
	sample_t *signal_samples[3];
	uint8_t *signal_power;
#ifdef HAVE_ALSA
	jitter_frame_t *jf;
#endif

	if (signal_num > radio->buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > buffer_size, please fix!.\n");
		abort();
	}

	/* audio buffers: how many sample for audio (rounded down) */
	audio_num = (int)((double)signal_num / radio->tx_resampler[0].factor);
	if (audio_num > radio->audio_buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "audio_num > audio_buffer_size, please fix!.\n");
		abort();
	}
	audio_samples[0] = radio->audio_buffer;
	audio_samples[1] = radio->audio_buffer + radio->audio_buffer_size;

	/* signal buffers: a bit more samples to be safe */
	signal_num = (int)((double)audio_num * radio->tx_resampler[0].factor + 0.5) + 10;
	if (signal_num > radio->signal_buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > signal_buffer_size, please fix!.\n");
		abort();
	}
	signal_samples[0] = radio->signal_buffer;
	signal_samples[1] = radio->signal_buffer + radio->signal_buffer_size;
	signal_samples[2] = radio->signal_buffer + radio->signal_buffer_size * 2;
	signal_power = radio->signal_power_buffer;

	/* get audio to be sent */
	switch (radio->tx_audio_mode) {
	case AUDIO_MODE_WAVEFILE:
		wave_read(&radio->wave_tx_play, audio_samples, audio_num);
		
		if (!radio->wave_tx_play.left) {
			int rc;
			int _samplerate = 0;
			wave_destroy_playback(&radio->wave_tx_play);
			rc = wave_create_playback(&radio->wave_tx_play, radio->tx_wave_file, &_samplerate, &radio->tx_audio_channels, 1.0);
			if (rc < 0) {
				LOGP(DRADIO, LOGL_ERROR, "Failed to re-open wave file.\n");
				return rc;
			}
		}
		break;
#ifdef HAVE_ALSA
	case AUDIO_MODE_AUDIODEV:
		rc = sound_read(radio->tx_sound, audio_samples, radio->audio_buffer_size, radio->tx_audio_channels, NULL);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to read from sound device (rc = %d)!\n", audio_num);
			if (rc == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
		jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)audio_samples[0], rc * sizeof(*(audio_samples[0])), 0, radio->tx_sequence[0], radio->tx_timestamp[0], 123);
		if (jf)
			jitter_save(&radio->tx_dejitter[0], jf);
		radio->tx_sequence[0] += 1;
		radio->tx_timestamp[0] += rc;
		jitter_load_samples(&radio->tx_dejitter[0], (uint8_t *)audio_samples[0], audio_num, sizeof(*(audio_samples[0])), NULL, NULL);
		if (radio->tx_audio_channels == 2) {
			jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)audio_samples[1], rc * sizeof(*(audio_samples[1])), 0, radio->tx_sequence[1], radio->tx_timestamp[1], 123);
			if (jf)
				jitter_save(&radio->tx_dejitter[1], jf);
			radio->tx_sequence[1] += 1;
			radio->tx_timestamp[1] += rc;
			jitter_load_samples(&radio->tx_dejitter[1], (uint8_t *)audio_samples[1], audio_num, sizeof(*(audio_samples[1])), NULL, NULL);
		}
		break;
#endif
	case AUDIO_MODE_TESTTONE:
		for (i = 0; i < audio_num; i++) {
			audio_samples[0][i] = radio->testtone[0][radio->testtone_pos];
			audio_samples[1][i] = radio->testtone[1][radio->testtone_pos];
			radio->testtone_pos = (radio->testtone_pos + 1) % radio->testtone_length;
		}
		break;
	default:
		LOGP(DRADIO, LOGL_ERROR, "Wrong audio mode, please fix!\n");
		return -EINVAL;
	}





	/* convert mono/stereo, generate differential signal */
	/* (Skip this if we want pure clean signal, but let's keep it to test stereo proc) */
	if (radio->stereo && radio->tx_audio_channels == 1) {
		/* mono to stereo: scale sum to 90%, differential signal is 0 */
		for (i = 0; i < audio_num; i++) {
			audio_samples[0][i] *= 0.9;
			audio_samples[1][i] = 0.0;
		}
	}
	if (radio->stereo && radio->tx_audio_channels == 2) {
		/* stereo: sum is 90%, diffential is 90% */
		double left, right;
		for (i = 0; i < audio_num; i++) {
			left = audio_samples[0][i];
			right = audio_samples[1][i];
			audio_samples[0][i] = (left + right) * 0.45;
			audio_samples[1][i] = (left - right) * 0.45;
		}
	}
	if (!radio->stereo && radio->tx_audio_channels == 2) {
		/* stereo to mono: sum both channel */
		for (i = 0; i < audio_num; i++)
			audio_samples[0][i] = (audio_samples[0][i] + audio_samples[1][i]) / 2.0;
	}



	/* remove DC */
	// iir_process(&radio->tx_dc_removal[0], audio_samples[0], audio_num);
	// if (radio->stereo)
	// 	iir_process(&radio->tx_dc_removal[1], audio_samples[1], audio_num);
	
	/* 
	 * DC OFFSET REMOVAL
	 * -----------------
	 * We use a recursive DC blocker filter: y[n] = x[n] - x[n-1] + R * y[n-1]
	 * R = 0.9995 corresponds to a cutoff of approx 10Hz at 48kHz.
	 * 
	 * JUSTIFICATION:
	 * 1. The previous IIR highpass filter from libfilter was ineffective, leaving a DC 
	 *    offset (up to 0.02) in the signal.
	 * 2. This DC offset caused asymmetric clipping and generated a strong 2nd harmonic 
	 *    distortion (2 kHz tone from a 1 kHz fundamental).
	 * 3. This manual implementation ensures the signal is centered at 0.0 before modulation.
	 */
	{
		double R = 0.9995;
		double x, y;
		int i;
		
		/* Channel 0 (Left/Mono) */
		for (i = 0; i < audio_num; i++) {
			x = audio_samples[0][i];
			y = x - radio->tx_dc_prev_x[0] + R * radio->tx_dc_prev_y[0];
			radio->tx_dc_prev_x[0] = x;
			radio->tx_dc_prev_y[0] = y;
			audio_samples[0][i] = y;
		}

		/* Channel 1 (Right) */
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++) {
				x = audio_samples[1][i];
				y = x - radio->tx_dc_prev_x[1] + R * radio->tx_dc_prev_y[1];
				radio->tx_dc_prev_x[1] = x;
				radio->tx_dc_prev_y[1] = y;
				audio_samples[1][i] = y;
			}
		}
	}



	/* gain volume */
	if (radio->volume != 1.0) {
		for (i = 0; i < audio_num; i++)
			audio_samples[0][i] *= radio->volume;
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++)
				audio_samples[1][i] *= radio->volume;
		}
	}

	/* upsample */
	signal_num = samplerate_upsample_output_num(&radio->tx_resampler[0], audio_num);
	samplerate_upsample(&radio->tx_resampler[0], audio_samples[0], audio_num, signal_samples[0], signal_num);
	if (radio->stereo)
		samplerate_upsample(&radio->tx_resampler[1], audio_samples[1], audio_num, signal_samples[1], signal_num);

	/* prepare baseband */
	memset(baseband, 0, sizeof(float) * 2 * signal_num);
	memset(signal_power, 1, signal_num);

	/* filter audio (remove DC, remove high frequencies, pre-emphasis)
	 * and modulate */
	switch (radio->modulation) {
	case MODULATION_FM:
		if (radio->emphasis) {
			if (fm_fast_math_enabled())
				pre_emphasis_fast(&radio->fm_emphasis_fast[0], signal_samples[0], signal_num);
			else
				pre_emphasis(&radio->fm_emphasis[0], signal_samples[0], signal_num);
		}

		clipper_process(signal_samples[0], signal_num);
		if (radio->stereo) {
			if (radio->emphasis) {
				if (fm_fast_math_enabled())
					pre_emphasis_fast(&radio->fm_emphasis_fast[1], signal_samples[1], signal_num);
				else
					pre_emphasis(&radio->fm_emphasis[1], signal_samples[1], signal_num);
			}
			clipper_process(signal_samples[1], signal_num);
		}
		
		/* Advance pilot phase if Stereo OR RDS is enabled */
		if (radio->stereo || radio->rds || radio->rds2) {
			double phasestep = radio->pilot_phasestep;
			double phase = radio->tx_pilot_phase;
			double start_phase = phase; /* Capture start phase for RDS */
			
			for (i = 0; i < signal_num; i++) {
				/* Add pilot tone only if Stereo */
				if (radio->stereo) {
					/* Add pilot (19 kHz) and stereo diff (38 kHz) */
					if (fm_fast_math_enabled()) {
						double sc_sin, sc_cos;
						/* 19 kHz pilot */
						fm_fast_sincos(phase * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
						signal_samples[0][i] += sc_sin * 0.1;
						/* 38 kHz stereo subcarrier (2x pilot) */
						fm_fast_sincos(phase * 2.0 * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
						signal_samples[0][i] += signal_samples[1][i] * sc_sin;
					} else {
						signal_samples[0][i] += sin(phase) * 0.1;
						signal_samples[0][i] += signal_samples[1][i] * sin(phase * 2);
					}
				}
				
				phase += phasestep;
				if (phase >= 2.0 * M_PI)
					phase -= 2.0 * M_PI;
			}
			radio->tx_pilot_phase = phase;

			/* Add RDS subcarrier if enabled (phase-locked to pilot at 3x frequency) */
			if (radio->rds || radio->rds2) {
				rds_encoder_process(&radio->rds_enc, signal_samples[0], signal_num,
						    start_phase, radio->pilot_phasestep);
				
				/*
				 * TODO: RDS2 Additional Subcarriers (IEC 62106-2:2021)
				 * ---------------------------------------------------
				 * If rds2 flag is set, we should add 3 more BPSK streams:
				 *
				 *   Stream 2:  66.5 kHz   (free-running, NOT pilot harmonic)
				 *   Stream 3:  71.25 kHz  (free-running, NOT pilot harmonic)
				 *   Stream 4:  76.0 kHz   (4 x pilot, CAN be phase-locked)
				 *
				 * Implementation would look like:
				 *   if (radio->rds2) {
				 *       rds2_encoder_process(&radio->rds2_enc, signal_samples[0],
				 *                            signal_num, start_phase, 
				 *                            radio->pilot_phasestep);
				 *   }
				 *
				 * Note: Streams 2 & 3 require independent NCOs because:
				 *   - 66.5 kHz = 19 kHz x 3.5   (not integer harmonic)
				 *   - 71.25 kHz = 19 kHz x 3.75 (not integer harmonic)
				 *
				 * Stream 4 can use: sin(pilot_phase * 4) for 76 kHz
				 *
				 * Injection level: ~2-5% per stream (same as original RDS)
				 * Total RDS2 injection: up to 4 x 5% = 20% (aggressive)
				 *
				 * See rds.c for rds2_encoder_t stub structure.
				 */
			}
		}
		for (i = 0; i < signal_num; i++)
			signal_samples[0][i] *= radio->fm_deviation;
		fm_modulate_complex(&radio->fm_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	case MODULATION_AM_DSB:
		/* also clip to prevent overshooting after audio filtering */
		clipper_process(signal_samples[0], signal_num);
		iir_process(&radio->tx_am_bw_limit, signal_samples[0], signal_num);
		am_modulate_complex(&radio->am_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		/* also clip to prevent overshooting after audio filtering */
		clipper_process(signal_samples[0], signal_num);
		iir_process(&radio->tx_am_bw_limit, signal_samples[0], signal_num);
		am_modulate_complex(&radio->am_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	default:
		break;
	}

	return signal_num;
}

int radio_rx(radio_t *radio, float *baseband, int signal_num)
{
	int i;
	int audio_num;
	sample_t *samples[3];
	double p;
#ifdef HAVE_ALSA
	jitter_frame_t *jf;
#endif

	if (signal_num > radio->buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > buffer_size, please fix!.\n");
		abort();
	}

	if (signal_num > radio->signal_buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > signal_buffer_size, please fix!.\n");
		abort();
	}
	samples[0] = radio->signal_buffer;
	samples[1] = radio->signal_buffer + radio->signal_buffer_size;
	samples[2] = radio->signal_buffer + radio->signal_buffer_size * 2;

	switch (radio->modulation) {
	case MODULATION_FM:
		fm_demodulate_complex(&radio->fm_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer);
		for (i = 0; i < signal_num; i++)
			samples[0][i] /= radio->fm_deviation;
		/* Decode RDS from FM baseband if enabled */
		if (radio->rds || radio->rds2) {
			/* Pass pilot phase WITH phase offset compensation (same as stereo decoder)
			 * The rx_pll_freq_offset compensates for IIR filter group delay and
			 * TX/RX frequency offset. RDS at 57 kHz = 3 x pilot needs 3x the offset.
			 */
			double rds_pilot_phase = radio->rx_pilot_phase - radio->rx_pll_freq_offset;
			rds_decoder_process(&radio->rds_dec, samples[0], signal_num,
			                    rds_pilot_phase, radio->pilot_phasestep);
		}
		if (radio->stereo) {
			/*
			 * FM STEREO DEMODULATION (Pilot-Tone System)
			 * ==========================================
			 * FM stereo uses DSB-SC modulation at 38 kHz (2x the 19 kHz pilot).
			 * The stereo difference signal (L-R) is modulated onto this subcarrier.
			 *
			 * Challenge: The narrow-band IIR pilot filter (5 Hz BW) needed to extract
			 * the 19 kHz pilot from the FM baseband introduces significant group delay.
			 * This makes per-sample phase tracking unreliable (the measured phase drifts
			 * continuously as our local oscillator advances).
			 *
			 * Solution: Block-level phase offset tracking
			 * 1. Demodulate using local 38 kHz oscillator with a fixed phase offset
			 * 2. Measure the actual phase offset once per block (from filtered I/Q)
			 * 3. Slowly track the average offset (~1 second time constant)
			 *
			 * The tracked offset compensates for:
			 * - IIR filter group delay (~13deg at 5 Hz BW)
			 * - Any transmitter/receiver frequency offset
			 * - SDR sample rate inaccuracies
			 */
			
			/* Step 1: Stereo demodulation using local 38 kHz oscillator */
			/* Apply tracked phase offset compensation */
			double phase_offset = radio->rx_pll_freq_offset;
			p = radio->rx_pilot_phase;
			for (i = 0; i < signal_num; i++) {
				/* 38 kHz carrier = 2x pilot phase, minus compensation offset */
				double carrier_38k = (p - phase_offset) * 2.0;
				if (fm_fast_math_enabled()) {
					double sc_sin, sc_cos;
					fm_fast_sincos(carrier_38k * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
					samples[1][i] = samples[0][i] * sc_sin * 2.0;
				} else {
					samples[1][i] = samples[0][i] * sin(carrier_38k) * 2.0;
				}
				
				p += radio->pilot_phasestep;
				if (p >= 2.0 * M_PI)
					p -= 2.0 * M_PI;
			}
			
			/* Step 2: Measure phase offset using I/Q mixing at end of block */
			/* Compute I and Q simultaneously using sincos for efficiency */
			p = radio->rx_pilot_phase;
			for (i = 0; i < signal_num; i++) {
				double sc_sin, sc_cos;
				if (fm_fast_math_enabled()) {
					fm_fast_sincos(p * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
				} else {
					sincos(p, &sc_sin, &sc_cos);
				}
				radio->I_buffer[i] = samples[0][i] * sc_cos;  /* I component */
				radio->Q_buffer[i] = samples[0][i] * sc_sin;  /* Q component */
				p += radio->pilot_phasestep;
				if (p >= 2.0 * M_PI)
					p -= 2.0 * M_PI;
			}
			/* Filter I channel (copy to samples[2] for in-place filtering) */
			for (i = 0; i < signal_num; i++)
				samples[2][i] = radio->I_buffer[i];
			iir_process(&radio->rx_lp_pilot_I, samples[2], signal_num);
			double I_end = samples[2][signal_num - 1];
			
			/* Filter Q channel */
			for (i = 0; i < signal_num; i++)
				samples[2][i] = radio->Q_buffer[i];
			iir_process(&radio->rx_lp_pilot_Q, samples[2], signal_num);
			double Q_end = samples[2][signal_num - 1];
			
			double pilot_mag = sqrt(I_end * I_end + Q_end * Q_end);
			
			if (pilot_mag > 1e-9) {
				/* Measured phase = offset between our oscillator and received pilot */
				double measured_offset = atan2(Q_end, I_end);
				
				/* Normalize to -45..+45 degrees for 90deg periodicity of sin(2x) */
				while (measured_offset > M_PI/4) measured_offset -= M_PI/2;
				while (measured_offset < -M_PI/4) measured_offset += M_PI/2;
				
				/* Track average offset with slow IIR (time constant ~1 second) */
				double alpha = 1.0 / (radio->signal_samplerate * 1.0);  /* 1 second TC */
				radio->rx_pll_freq_offset += alpha * signal_num * (measured_offset - radio->rx_pll_freq_offset);
			}
			
			/* Update pilot phase for next block */
			radio->rx_pilot_phase = p;
			
			/* Diagnostics (every ~1 second) */
			{
				static int diag_count = 0;
				diag_count += signal_num;
				if (diag_count >= 1000000) {
					diag_count = 0;
					/* double offset_deg = radio->rx_pll_freq_offset * (180.0 / M_PI); */
					/* LOGP(DRADIO, LOGL_DEBUG, "Stereo: offset=%.1fdeg pilot=%.6f\n", offset_deg, pilot_mag); */
				}
			}
			
			/* Filter stereo channels to match bandwidth */
			iir_process(&radio->rx_lp_sum, samples[0], signal_num);
			iir_process(&radio->rx_lp_diff, samples[1], signal_num);
		}
		if (radio->emphasis) {
			/* RX path: DC filter → de-emphasis
			 * DC blocking removes any DC offset from FM demodulator output.
			 * De-emphasis restores flat frequency response. */
			if (fm_fast_math_enabled()) {
				dc_filter_fast(&radio->rx_dc_filter[0], samples[0], signal_num);
				de_emphasis_fast(&radio->fm_emphasis_fast[0], samples[0], signal_num);
				if (radio->stereo) {
					dc_filter_fast(&radio->rx_dc_filter[1], samples[1], signal_num);
					de_emphasis_fast(&radio->fm_emphasis_fast[1], samples[1], signal_num);
				}
			} else {
				dc_filter(&radio->fm_emphasis[0], samples[0], signal_num);
				de_emphasis(&radio->fm_emphasis[0], samples[0], signal_num);
				if (radio->stereo) {
					dc_filter(&radio->fm_emphasis[1], samples[1], signal_num);
					de_emphasis(&radio->fm_emphasis[1], samples[1], signal_num);
				}
			}
		}
		break;
	case MODULATION_AM_DSB:
		am_demodulate_complex(&radio->am_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer, radio->carrier_buffer);
		break;
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		am_demodulate_complex(&radio->am_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer, radio->carrier_buffer);
		break;
	default:
		break;
	}

	/* downsample */
	audio_num = samplerate_downsample(&radio->rx_resampler[0], samples[0], signal_num);
	if (radio->stereo)
		samplerate_downsample(&radio->rx_resampler[1], samples[1], signal_num);

	/* dampen volume */
	if (radio->volume != 1.0) {
		for (i = 0; i < audio_num; i++)
			samples[0][i] /= radio->volume;
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++)
				samples[1][i] /= radio->volume;
		}
	}

	/* convert mono/stereo, (from differential signal) */
	if (radio->stereo && radio->rx_audio_channels == 1) {
		/* stereo to mono */
		for (i = 0; i < audio_num; i++) {
			samples[0][i] = (samples[0][i] + samples[1][i]) / 2.0;
		}
	}
	if (radio->stereo && radio->rx_audio_channels == 2) {
		/* stereo from differential */
		double sum, diff;
		for (i = 0; i < audio_num; i++) {
			sum = samples[0][i];
			diff = samples[1][i];
			samples[0][i] = sum + diff / 2.0;
			samples[1][i] = sum - diff / 2.0;
		}
	}
	if (!radio->stereo && radio->rx_audio_channels == 2) {
		/* mono to stereo: clone channel */
		for (i = 0; i < audio_num; i++)
			samples[1][i] = samples[0][i];
	}

	/* display wave */
	display_wave(&radio->dispwav[0], samples[0], audio_num, 1.0);
	if (radio->stereo && radio->rx_audio_channels == 2)
		display_wave(&radio->dispwav[1], samples[1], audio_num, 1.0);

	/* store received audio */
	if ((radio->rx_audio_mode & AUDIO_MODE_WAVEFILE))
		wave_write(&radio->wave_rx_rec, samples, audio_num);
#ifdef HAVE_ALSA
	if ((radio->rx_audio_mode & AUDIO_MODE_AUDIODEV) && audio_num > 0) {
		jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)samples[0], audio_num * sizeof(*(samples[0])), 0, radio->rx_sequence[0], radio->rx_timestamp[0], 123);
		if (jf)
			jitter_save(&radio->rx_dejitter[0], jf);
		radio->rx_sequence[0] += 1;
		radio->rx_timestamp[0] += audio_num;
		if (radio->rx_audio_channels == 2) {
			jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)samples[1], audio_num * sizeof(*(samples[1])), 0, radio->rx_sequence[1], radio->rx_timestamp[1], 123);
			if (jf)
				jitter_save(&radio->rx_dejitter[1], jf);
			radio->rx_sequence[1] += 1;
			radio->rx_timestamp[1] += audio_num;
		}
	}
	if ((radio->rx_audio_mode & AUDIO_MODE_AUDIODEV)) {
		audio_num = sound_get_tosend(radio->rx_sound, radio->signal_buffer_size);
		if (audio_num < 0) {
			LOGP(DDSP, LOGL_ERROR, "Failed to get number of samples in buffer (rc = %d)!\n", audio_num);
			if (audio_num == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
		jitter_load_samples(&radio->rx_dejitter[0], (uint8_t *)samples[0], audio_num, sizeof(*samples), NULL, NULL);
		if (radio->rx_audio_channels == 2)
			jitter_load_samples(&radio->rx_dejitter[1], (uint8_t *)samples[1], audio_num, sizeof(*samples), NULL, NULL);
		// printf("channels=%d num=%d\n", radio->rx_audio_channels, audio_num);
		audio_num = sound_write(radio->rx_sound, samples, NULL, audio_num, NULL, NULL, radio->rx_audio_channels);
		if (audio_num < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to write to sound device (rc = %d)!\n", audio_num);
			if (audio_num == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
	}
#endif

	return signal_num;
}


void radio_set_callsign(const char *callsign)
{
	if (rds_user_callsign)
		free(rds_user_callsign);
	rds_user_callsign = callsign ? strdup(callsign) : NULL;
}

void radio_set_pi(uint16_t pi)
{
	rds_user_pi = pi;
}
