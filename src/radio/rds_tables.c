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

#include "rds_tables.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>
#include "../liblogging/logging.h"

/* ============================================================
 * Programme Type (PTY) Names - RDS (European)
 * IEC 62106 Annex F (EN 50067:1998, pp. 77-78)
 * ============================================================ */
static const char *pty_names_rds[32] = {
	"No PTY",           /* 0 */
	"News",             /* 1 */
	"Current affairs",  /* 2 */
	"Information",      /* 3 */
	"Sport",            /* 4 */
	"Education",        /* 5 */
	"Drama",            /* 6 */
	"Culture",          /* 7 */
	"Science",          /* 8 */
	"Varied",           /* 9 */
	"Pop music",        /* 10 */
	"Rock music",       /* 11 */
	"Easy listening",   /* 12 */
	"Light classical",  /* 13 */
	"Serious classical",/* 14 */
	"Other music",      /* 15 */
	"Weather",          /* 16 */
	"Finance",          /* 17 */
	"Children's",       /* 18 */
	"Social affairs",   /* 19 */
	"Religion",         /* 20 */
	"Phone-in",         /* 21 */
	"Travel",           /* 22 */
	"Leisure",          /* 23 */
	"Jazz music",       /* 24 */
	"Country music",    /* 25 */
	"National music",   /* 26 */
	"Oldies music",     /* 27 */
	"Folk music",       /* 28 */
	"Documentary",      /* 29 */
	"Alarm test",       /* 30 */
	"Alarm"             /* 31 */
};

/* ============================================================
 * Programme Type (PTY) Names - RBDS (US)
 * NRSC-4-B Annex F (pp. 95-96)
 * ============================================================ */
static const char *pty_names_rbds[32] = {
	"No PTY",           /* 0 */
	"News",             /* 1 */
	"Information",      /* 2 */
	"Sports",           /* 3 */
	"Talk",             /* 4 */
	"Rock",             /* 5 */
	"Classic rock",     /* 6 */
	"Adult hits",       /* 7 */
	"Soft rock",        /* 8 */
	"Top 40",           /* 9 */
	"Country",          /* 10 */
	"Oldies",           /* 11 */
	"Soft",             /* 12 */
	"Nostalgia",        /* 13 */
	"Jazz",             /* 14 */
	"Classical",        /* 15 */
	"R&B",              /* 16 */
	"Soft R&B",         /* 17 */
	"Language",         /* 18 */
	"Religious music",  /* 19 */
	"Religious talk",   /* 20 */
	"Personality",      /* 21 */
	"Public",           /* 22 */
	"College",          /* 23 */
	"Spanish talk",     /* 24 */
	"Spanish music",    /* 25 */
	"Hip hop",          /* 26 */
	"(unassigned)",     /* 27 */
	"(unassigned)",     /* 28 */
	"Weather",          /* 29 */
	"Emergency test",   /* 30 */
	"Emergency"         /* 31 */
};

const char *rds_get_pty_name(uint8_t pty, int rbds)
{
	if (pty > 31)
		return "Invalid";
	return rbds ? pty_names_rbds[pty] : pty_names_rds[pty];
}

/* ============================================================
 * Language Codes
 * IEC 62106 Annex J (EN 50067:1998, p. 84)
 * ============================================================ */
static const char *rds_language_codes_group_1a[128] = {
	"Unknown",       /* 0 */
	"Albanian",      /* 1 */
	"Breton",        /* 2 */
	"Catalan",       /* 3 */
	"Croatian",      /* 4 */
	"Welsh",         /* 5 */
	"Czech",         /* 6 */
	"Danish",        /* 7 */
	"German",        /* 8 */
	"English",       /* 9 */
	"Spanish",       /* 10 */
	"Esperanto",     /* 11 */
	"Estonian",      /* 12 */
	"Basque",        /* 13 */
	"Faroese",       /* 14 */
	"French",        /* 15 */
	"Frisian",       /* 16 */
	"Irish",         /* 17 */
	"Gaelic",        /* 18 */
	"Galician",      /* 19 */
	"Icelandic",     /* 20 */
	"Italian",       /* 21 */
	"Lappish",       /* 22 */
	"Latin",         /* 23 */
	"Latvian",       /* 24 */
	"Luxembourgian", /* 25 */
	"Lithuanian",    /* 26 */
	"Hungarian",     /* 27 */
	"Maltese",       /* 28 */
	"Dutch",         /* 29 */
	"Norwegian",     /* 30 */
	"Occitan",       /* 31 */
	"Polish",        /* 32 */
	"Portuguese",    /* 33 */
	"Romanian",      /* 34 */
	"Romansh",       /* 35 */
	"Serbian",       /* 36 */
	"Slovak",        /* 37 */
	"Slovene",       /* 38 */
	"Finnish",       /* 39 */
	"Swedish",       /* 40 */
	"Turkish",       /* 41 */
	"Flemish",       /* 42 */
	"Walloon",       /* 43 */
	NULL, NULL, NULL, NULL, /* 44-47 reserved */
	NULL, NULL, NULL, NULL, /* 48-51 reserved */
	NULL, NULL, NULL, NULL, /* 52-55 reserved */
	NULL, NULL, NULL, NULL, /* 56-59 reserved */
	NULL, NULL, NULL, NULL, /* 60-63 reserved */
	"Background",    /* 64 */
	NULL, NULL, NULL, /* 65-67 reserved */
	NULL,            /* 68 reserved */
	"Zulu",          /* 69 */
	"Vietnamese",    /* 70 */
	"Uzbek",         /* 71 */
	"Urdu",          /* 72 */
	"Ukrainian",     /* 73 */
	"Thai",          /* 74 */
	"Telugu",        /* 75 */
	"Tatar",         /* 76 */
	"Tamil",         /* 77 */
	"Tadzhik",       /* 78 */
	"Swahili",       /* 79 */
	"Sranan Tongo",  /* 80 */
	"Somali",        /* 81 */
	"Sinhalese",     /* 82 */
	"Shona",         /* 83 */
	"Serbo-Croat",   /* 84 */
	"Ruthenian",     /* 85 */
	"Russian",       /* 86 */
	"Quechua",       /* 87 */
	"Pushtu",        /* 88 */
	"Punjabi",       /* 89 */
	"Persian",       /* 90 */
	"Papamiento",    /* 91 */
	"Oriya",         /* 92 */
	"Nepali",        /* 93 */
	"Ndebele",       /* 94 */
	"Marathi",       /* 95 */
	"Moldovian",     /* 96 */
	"Malaysian",     /* 97 */
	"Malagasy",      /* 98 */
	"Macedonian",    /* 99 */
	"Laotian",       /* 100 */
	"Korean",        /* 101 */
	"Khmer",         /* 102 */
	"Kazakh",        /* 103 */
	"Kannada",       /* 104 */
	"Japanese",      /* 105 */
	"Indonesian",    /* 106 */
	"Hindi",         /* 107 */
	"Hebrew",        /* 108 */
	"Hausa",         /* 109 */
	"Gurani",        /* 110 */
	"Gujarati",      /* 111 */
	"Greek",         /* 112 */
	"Georgian",      /* 113 */
	"Fulani",        /* 114 */
	"Dari",          /* 115 */
	"Chuvash",       /* 116 */
	"Chinese",       /* 117 */
	"Burmese",       /* 118 */
	"Bulgarian",     /* 119 */
	"Bengali",       /* 120 */
	"Belorussian",   /* 121 */
	"Bambora",       /* 122 */
	"Azerbaijani",   /* 123 */
	"Assamese",      /* 124 */
	"Armenian",      /* 125 */
	"Arabic",        /* 126 */
	"Amharic"        /* 127 */
};

const char *rds_get_language_name(uint8_t code)
{
	if (code > 127)
		return "Invalid";
	const char *name = rds_language_codes_group_1a[code];
	return name ? name : "(reserved)";
}

/* ============================================================
 * Decoder Identification (DI) - Group 0A/0B
 * IEC 62106 S3.2.1.5 (EN 50067:1998, p. 41)
 *
 * DI bits are transmitted one per Group 0, interleaved with PS segments:
 *   Segment 0 (chars 0-1) -> d3 (stereo)
 *   Segment 1 (chars 2-3) -> d2 (artificial head)
 *   Segment 2 (chars 4-5) -> d1 (compressed)
 *   Segment 3 (chars 6-7) -> d0 (dynamic PTY)
 *
 * OBSOLESCENCE (IEC 62106-2:2021):
 *   d1-d3 are considered OBSOLETE - modern receivers detect stereo
 *   via 19 kHz pilot tone independently. Only d0 (dynamic PTY) remains
 *   relevant for indicating PTY changes during a programme.
 * ============================================================ */
typedef struct {
	const char *name;        /* Short identifier */
	const char *description; /* Human-readable description */
	const char *value_0;     /* Meaning when bit = 0 */
	const char *value_1;     /* Meaning when bit = 1 */
} rds_di_info_t;

static const rds_di_info_t di_info[4] = {
	/* d0 - transmitted with segment 3 */
	{ "dynamic_pty", "Dynamic PTY Indicator",
	  "Static PTY (fixed)", "Dynamic PTY (changes)" },
	/* d1 - transmitted with segment 2 (obsolete) */
	{ "compressed", "Audio Compression",
	  "Not compressed", "Compressed" },
	/* d2 - transmitted with segment 1 (obsolete) */
	{ "artificial_head", "Artificial Head Recording",
	  "No", "Yes (binaural)" },
	/* d3 - transmitted with segment 0 (obsolete) */
	{ "stereo", "Stereo/Mono",
	  "Mono", "Stereo" }
};

const char *rds_get_di_name(uint8_t addr)
{
	if (addr > 3)
		return "invalid";
	return di_info[addr].name;
}

const char *rds_get_di_description(uint8_t addr)
{
	if (addr > 3)
		return "Invalid DI address";
	return di_info[addr].description;
}

const char *rds_get_di_value_name(uint8_t addr, int value)
{
	if (addr > 3)
		return "invalid";
	return value ? di_info[addr].value_1 : di_info[addr].value_0;
}

/* ============================================================
 * Music/Speech (MS) Flag - Group 0A/0B
 * IEC 62106 S3.2.1.3
 *
 * Indicates if the current programme content is music or speech.
 * Receivers may use this to adjust audio processing (e.g., EQ).
 * ============================================================ */
const char *rds_get_ms_name(int ms)
{
	return ms ? "Music" : "Speech";
}

/* ============================================================
 * Traffic Programme (TP) and Traffic Announcement (TA) Flags
 * IEC 62106 S3.2.1.2
 *
 * TP indicates the station carries traffic information.
 * TA indicates a traffic announcement is currently in progress.
 *
 * Combined meanings:
 *   TP=0, TA=0: Not a traffic programme
 *   TP=1, TA=0: Traffic programme, no announcement now
 *   TP=1, TA=1: Traffic announcement in progress
 *   TP=0, TA=1: EON referral (tune to linked TP station)
 * ============================================================ */
const char *rds_get_tp_name(int tp)
{
	return tp ? "Traffic" : "No traffic";
}

const char *rds_get_ta_name(int ta)
{
	return ta ? "Announcement" : "No announcement";
}

const char *rds_get_tp_ta_description(int tp, int ta)
{
	if (tp && ta)
		return "Traffic announcement in progress";
	else if (tp && !ta)
		return "Traffic programme (no announcement now)";
	else if (!tp && ta)
		return "EON referral to linked traffic station";
	else
		return "Not a traffic programme";
}

/* ============================================================
 * Alternative Frequencies (AF) - Group 0A Block C
 * EN 50067 S3.2.1.6, IEC 62106 Table 11
 *
 * AF enables seamless frequency switching for mobile receivers.
 *
 * Code ranges:
 *   0x00 (0):       Not to be used
 *   0x01-0xCC (1-204): FM frequencies 87.6-107.9 MHz
 *   0xCD (205):     Filler code
 *   0xCE-0xDF (206-223): Reserved
 *   0xE0-0xF9 (224-249): AF count (N = code - 224, 1-25 AFs)
 *   0xFA (250):     LF/MF frequency follows
 *   0xFB-0xFF (251-255): Reserved
 *
 * Method A (S3.2.1.6.3): Simple list up to 25 AFs
 *   [count+AF1] [AF2,AF3] [AF4,AF5] ...
 *
 * Method B (S3.2.1.6.4): Paired, for >25 AFs or regional
 *   [count,tuned] [freq1,freq2] [freq1,freq2] ...
 *   - Ascending (freq1 < freq2): same content
 *   - Descending (freq1 > freq2): regional variant
 *
 * NOTE: Method B is detected heuristically (no control code).
 * ============================================================ */

/* Convert AF code (1-204) to frequency in 0.1 MHz units (e.g., 1000 = 100.0 MHz) */
int rds_af_code_to_freq(uint8_t code)
{
	if (code >= 1 && code <= 204)
		return 875 + code;  /* 87.6-108.0 MHz in 0.1 MHz steps */
	return 0;  /* Invalid */
}

/* Convert frequency (0.1 MHz units) to AF code */
uint8_t rds_freq_to_af_code(int freq_tenth_mhz)
{
	if (freq_tenth_mhz >= 876 && freq_tenth_mhz <= 1079)
		return (uint8_t)(freq_tenth_mhz - 875);
	return 0;  /* Invalid */
}

/* Convert LF frequency (kHz) to AF code (1-15)
 * LF band: 153-279 kHz, always 9 kHz spacing
 * Formula: code = (freq_kHz - 144) / 9
 */
uint8_t rds_lf_freq_to_code(int freq_khz)
{
	if (freq_khz >= 153 && freq_khz <= 279) {
		int code = (freq_khz - 144) / 9;
		if (code >= 1 && code <= 15)
			return (uint8_t)code;
	}
	return 0;  /* Invalid */
}

/* Convert MF frequency (kHz) to AF code (16-135)
 * MF band: region-dependent spacing
 *   EU/RDS: 531-1602 kHz, 9 kHz spacing
 *   US/RBDS: 540-1710 kHz, 10 kHz spacing
 * Parameters:
 *   freq_khz = frequency in kHz
 *   is_us = 1 for RBDS (10 kHz), 0 for RDS (9 kHz)
 */
uint8_t rds_mf_freq_to_code(int freq_khz, int is_us)
{
	int spacing = is_us ? 10 : 9;
	int base_freq = is_us ? 540 : 531;
	int max_freq = is_us ? 1710 : 1602;
	
	if (freq_khz >= base_freq && freq_khz <= max_freq) {
		int code = 16 + (freq_khz - base_freq) / spacing;
		if (code >= 16 && code <= 135)
			return (uint8_t)code;
	}
	return 0;  /* Invalid */
}

/* Convert LF/MF AF code to frequency (kHz)
 * Parameters:
 *   code = LF (1-15) or MF (16-135) code
 *   is_us = 1 for RBDS (10 kHz), 0 for RDS (9 kHz)
 */
int rds_lf_mf_code_to_freq(uint8_t code, int is_us)
{
	if (code >= 1 && code <= 15) {
		/* LF band: always 9 kHz spacing */
		return 144 + code * 9;
	} else if (code >= 16 && code <= 135) {
		/* MF band: region-dependent */
		int spacing = is_us ? 10 : 9;
		int mf_base = is_us ? 380 : 387;  /* 540-16*10 or 531-16*9 */
		return mf_base + code * spacing;
	}
	return 0;  /* Invalid */
}

/* Get human-readable description of AF code */
const char *rds_get_af_code_description(uint8_t code)
{
	if (code == 0)
		return "Not used";
	else if (code >= 1 && code <= 204)
		return "FM frequency";
	else if (code == 205)
		return "Filler";
	else if (code >= 224 && code <= 249)
		return "AF count";
	else if (code == 250)
		return "LF/MF follows";
	else
		return "Reserved";
}

/* Check if AF code is a valid FM frequency (1-204) */
int rds_is_valid_af_code(uint8_t code)
{
	return (code >= 1 && code <= 204);
}

/* Check if an AF pair indicates regional variant (Method B pair ordering)
 * Per EN 50067 S3.2.1.6.4: compare values in the pair [freq1, freq2]
 *   - freq1 < freq2 (ascending): Same content
 *   - freq1 > freq2 (descending): Regional variant (different content)
 * Returns: 1 if regional variant (descending), 0 if same content (ascending)
 */
int rds_af_is_regional_pair(uint16_t freq1, uint16_t freq2)
{
	/* Descending order (freq1 > freq2) indicates regional variant */
	return (freq1 > freq2) ? 1 : 0;
}

/* Check if AF list appears to be Method B (heuristic detection)
 * Based on redsea's algorithm:
 *   1. Odd number of frequencies
 *   2. First frequency (tuning) repeats in every pair
 * 
 * Parameters:
 *   af[] = frequency list in 0.1 MHz units
 *   count = number of frequencies
 * Returns: 1 if Method B detected, 0 if Method A
 */
int rds_af_is_method_b(const uint16_t *af, int count)
{
	int i;
	uint16_t tuned;
	
	/* Method B has odd number of elements, at least 3 */
	if (count < 3 || (count % 2) != 1)
		return 0;
	
	/* First element is the tuning frequency */
	tuned = af[0];
	
	/* Check that tuning freq appears in every pair */
	for (i = 1; i < count - 1; i += 2) {
		if (af[i] != tuned && af[i + 1] != tuned)
			return 0;  /* Neither freq is tuned → not Method B */
	}
	
	return 1;
}

/* ============================================================
 * Country Codes
 * IEC 62106 Annex D (simplified - major countries only)
 * Full table requires ECC + CC matrix lookup
 * ============================================================ */

/* Structure for ECC-based country lookup */
typedef struct {
	uint8_t ecc;
	const char *countries[15];  /* CC 1-15 */
} ecc_country_t;

static const ecc_country_t country_table[] = {
	/* ECC 0xE0: Europe 1 */
	{ 0xE0, { "DE", "DZ", "AD", "IL", "IT", "BE", "RU", "PS", "AL", "AT", "HU", "MT", "DE", "--", "EG" }},
	/* ECC 0xE1: Europe 2 */
	{ 0xE1, { "GR", "CY", "SM", "CH", "JO", "FI", "LU", "BG", "DK", "GI", "IQ", "GB", "LY", "RO", "FR" }},
	/* ECC 0xE2: Europe 3 */
	{ 0xE2, { "MA", "CZ", "PL", "VA", "SK", "SY", "TN", "--", "LI", "IS", "MC", "LT", "RS", "ES", "NO" }},
	/* ECC 0xE3: Europe 4 */
	{ 0xE3, { "ME", "IE", "TR", "MK", "--", "--", "--", "NL", "LV", "LB", "AZ", "HR", "KZ", "SE", "BY" }},
	/* ECC 0xE4: Europe 5 */
	{ 0xE4, { "MD", "EE", "KG", "--", "--", "UA", "XK", "PT", "SI", "AM", "--", "GE", "--", "--", "BA" }},
	/* ECC 0xA0: Americas - US */
	{ 0xA0, { "US", "US", "US", "US", "US", "US", "US", "US", "US", "US", "US", "--", "US", "US", "--" }},
	/* ECC 0xA1: Americas - Canada */
	{ 0xA1, { "--", "--", "--", "--", "--", "--", "--", "--", "--", "--", "CA", "CA", "CA", "CA", "GL" }},
	/* ECC 0xF0: Asia/Pacific 1 */
	{ 0xF0, { "AU", "AU", "AU", "AU", "AU", "AU", "AU", "AU", "SA", "AF", "MM", "CN", "KP", "BH", "MY" }},
	/* ECC 0xF1: Asia/Pacific 2 */
	{ 0xF1, { "KI", "BT", "BD", "PK", "FJ", "OM", "NR", "IR", "NZ", "SB", "BN", "LK", "TW", "KR", "HK" }},
	/* ECC 0xF2: Asia/Pacific 3 */
	{ 0xF2, { "KW", "QA", "KH", "WS", "IN", "MO", "VN", "PH", "JP", "SG", "MV", "ID", "AE", "NP", "VU" }},
	{ 0, { NULL }}  /* Terminator */
};

const char *rds_get_country_code(uint8_t cc, uint8_t ecc)
{
	if (cc < 1 || cc > 15)
		return "--";
	
	for (const ecc_country_t *e = country_table; e->ecc != 0; e++) {
		if (e->ecc == ecc) {
			const char *code = e->countries[cc - 1];
			return code ? code : "--";
		}
	}
	return "--";
}

/* ============================================================
 * Extended Country Code (ECC) Country Names
 * IEC 62106 Annex D / NRSC-4-B Annex N
 * ============================================================ */
static const char *ecc_country_names[] = {
	"Germany",      /* 0 */
	"Greece",       /* 1 */
	"Morocco",      /* 2 */
	"Moldova",      /* 3 */
	"Algeria",      /* 4 */
	"Cyprus",       /* 5 */
	"Czech Rep.",   /* 6 */
	"Ireland",      /* 7 */
	"Estonia",      /* 8 */
	"Andorra",      /* 9 */
	"San Marino",   /* 10 */
	"Poland",       /* 11 */
	"Turkey",       /* 12 */
	"Israel",       /* 13 */
	"Switzerland",  /* 14 */
	"Vatican",      /* 15 */
	"Macedonia",    /* 16 */
	"Italy",        /* 17 */
	"Jordan",       /* 18 */
	"Slovakia",     /* 19 */
	"Belgium",      /* 20 */
	"Finland",      /* 21 */
	"Syria",        /* 22 */
	"Serbia",       /* 23 */
	"Ukraine",      /* 24 */
	"Russia",       /* 25 */
	"Luxembourg",   /* 26 */
	"Tunisia",      /* 27 */
	"Palestine",    /* 28 */
	"Bulgaria",     /* 29 */
	"Madeira",      /* 30 */
	"Netherlands",  /* 31 */
	"Portugal",     /* 32 */
	"Albania",      /* 33 */
	"Denmark",      /* 34 */
	"Liechtenstein",/* 35 */
	"Latvia",       /* 36 */
	"Slovenia",     /* 37 */
	"Austria",      /* 38 */
	"Gibraltar",    /* 39 */
	"Iceland",      /* 40 */
	"Lebanon",      /* 41 */
	"Hungary",      /* 42 */
	"Iraq",         /* 43 */
	"Monaco",       /* 44 */
	"Malta",        /* 45 */
	"United Kingdom",/* 46 */
	"Lithuania",    /* 47 */
	"Croatia",      /* 48 */
	"Libya",        /* 49 */
	"Canary Islands",/* 50 */
	"Romania",      /* 51 */
	"Spain",        /* 52 */
	"Sweden",       /* 53 */
	"Egypt",        /* 54 */
	"France",       /* 55 */
	"Norway",       /* 56 */
	"Belarus",      /* 57 */
	"Bosnia Herz.", /* 58 */
	"Montenegro",   /* 59 */
	"Armenia",      /* 60 */
	"Azerbaijan",   /* 61 */
	"Kosovo",       /* 62 */
	"Kyrgyzstan",   /* 63 */
	"Turkmenistan", /* 64 */
	"Tajikistan",   /* 65 */
	"Uzbekistan",   /* 66 */
	"Malawi",       /* 67 */
	"Mali",         /* 68 */
	"Mauritania",   /* 69 */
	"Mauritius",    /* 70 */
	"Mongolia",     /* 71 */
	"Mozambique",   /* 72 */
	"Namibia",      /* 73 */
	"Niger",        /* 74 */
	"Nigeria",      /* 75 */
	"Oman",         /* 76 */
	"Qatar",        /* 77 */
	"Rwanda",       /* 78 */
	"Sao Tome",     /* 79 */
	"Saudi Arabia", /* 80 */
	"Senegal",      /* 81 */
	"Seychelles",   /* 82 */
	"Sierra Leone", /* 83 */
	"Somalia",      /* 84 */
	"South Africa", /* 85 */
	"South Sudan",  /* 86 */
	"Sudan",        /* 87 */
	"Eswatini",     /* 88 */
	"Tanzania",     /* 89 */
	"Togo",         /* 90 */
	"Uganda",       /* 91 */
	"Western Sahara",/* 92 */
	"Yemen",        /* 93 */
	"Zambia",       /* 94 */
	"Zimbabwe",     /* 95 */
	"Angola",       /* 96 */
	"Ascension Isl.",/* 97 */
	"Bahrain",      /* 98 */
	"Benin",        /* 99 */
	"Botswana",     /* 100 */
	"Burkina Faso", /* 101 */
	"Burundi",      /* 102 */
	"Cabinda",      /* 103 */
	"Cameroon",     /* 104 */
	"Cape Verde",   /* 105 */
	"Central Africa",/* 106 */
	"Chad",         /* 107 */
	"Comoros",      /* 108 */
	"DR Congo",     /* 109 */
	"Congo",        /* 110 */
	"Cote d'Ivoire",/* 111 */
	"Djibouti",     /* 112 */
	"Eq. Guinea",   /* 113 */
	"Eritrea",      /* 114 */
	"Ethiopia",     /* 115 */
	"Gabon",        /* 116 */
	"Gambia",       /* 117 */
	"Georgia",      /* 118 */
	"Ghana",        /* 119 */
	"Guinea",       /* 120 */
	"Guinea-Bissau",/* 121 */
	"Kazakhstan",   /* 122 */
	"Kenya",        /* 123 */
	"Kuwait",       /* 124 */
	"Lesotho",      /* 125 */
	"Liberia",      /* 126 */
	"Madagascar",   /* 127 */
	"UAE",          /* 128 */
	"Anguilla",     /* 129 */
	"Antigua",      /* 130 */
	"Argentina",    /* 131 */
	"Aruba",        /* 132 */
	"Barbados",     /* 133 */
	"Belize",       /* 134 */
	"Bermuda",      /* 135 */
	"Bolivia",      /* 136 */
	"Brazil",       /* 137 */
	"Canada",       /* 138 */
	"Cayman Islands",/* 139 */
	"Chile",        /* 140 */
	"Colombia",     /* 141 */
	"Costa Rica",   /* 142 */
	"Cuba",         /* 143 */
	"Dominica",     /* 144 */
	"Dominican Rep.",/* 145 */
	"El Salvador",  /* 146 */
	"Ecuador",      /* 147 */
	"Falkland Isl.",/* 148 */
	"Greenland",    /* 149 */
	"Grenada",      /* 150 */
	"Guadeloupe",   /* 151 */
	"Guatemala",    /* 152 */
	"Guyana",       /* 153 */
	"Haiti",        /* 154 */
	"Honduras",     /* 155 */
	"Jamaica",      /* 156 */
	"Martinique",   /* 157 */
	"Mexico",       /* 158 */
	"Montserrat",   /* 159 */
	"Neth. Antilles",/* 160 */
	"Nicaragua",    /* 161 */
	"Panama",       /* 162 */
	"Paraguay",     /* 163 */
	"Peru",         /* 164 */
	"Puerto Rico",  /* 165 */
	"St. Kitts",    /* 166 */
	"St. Lucia",    /* 167 */
	"St. Pierre",   /* 168 */
	"St. Vincent",  /* 169 */
	"Suriname",     /* 170 */
	"Trinidad",     /* 171 */
	"Turks & Caicos",/* 172 */
	"USA",          /* 173 */
	"Uruguay",      /* 174 */
	"Venezuela",    /* 175 */
	"Virgin Isl. UK",/* 176 */
	"Virgin Isl. US",/* 177 */
	"Afghanistan",  /* 178 */
	"Australia ACT",/* 179 */
	"Australia NSW",/* 180 */
	"Australia VIC",/* 181 */
	"Australia QLD",/* 182 */
	"Australia SA", /* 183 */
	"Australia WA", /* 184 */
	"Australia TAS",/* 185 */
	"Australia NT", /* 186 */
	"Bhutan",       /* 187 */
	"Brunei",       /* 188 */
	"Cambodia",     /* 189 */
	"China",        /* 190 */
	"Fiji",         /* 191 */
	"Hong Kong",    /* 192 */
	"India",        /* 193 */
	"Indonesia",    /* 194 */
	"Iran",         /* 195 */
	"Japan",        /* 196 */
	"Kiribati",     /* 197 */
	"North Korea",  /* 198 */
	"South Korea",  /* 199 */
	"Laos",         /* 200 */
	"Macao",        /* 201 */
	"Malaysia",     /* 202 */
	"Maldives",     /* 203 */
	"Marshall Isl.",/* 204 */
	"Micronesia",   /* 205 */
	"Myanmar",      /* 206 */
	"Nauru",        /* 207 */
	"Nepal",        /* 208 */
	"New Zealand",  /* 209 */
	"Pakistan",     /* 210 */
	"Papua N.G.",   /* 211 */
	"Philippines",  /* 212 */
	"Samoa",        /* 213 */
	"Singapore",    /* 214 */
	"Solomon Isl.", /* 215 */
	"Sri Lanka",    /* 216 */
	"Taiwan",       /* 217 */
	"Thailand",     /* 218 */
	"Tonga",        /* 219 */
	"Vanuatu",      /* 220 */
	"Vietnam",      /* 221 */
	"Bahamas",      /* 222 */
	"Brazil BM",    /* 223 */
	"Brazil EC",    /* 224 */
	"Brazil AN",    /* 225 */
	"USA/VI/PR",    /* 226 */
	"Bangladesh"    /* 227 */
};

#define ECC_COUNTRY_COUNT (sizeof(ecc_country_names) / sizeof(ecc_country_names[0]))

const char *rds_get_ecc_country_name(int index)
{
	if (index < 0 || (size_t)index >= ECC_COUNTRY_COUNT)
		return "(Unknown)";
	return ecc_country_names[index];
}

/* ============================================================
 * Extended Country Code (ECC) Region Names (legacy)
 * IEC 62106 Annex D - Human-readable region descriptions
 * ============================================================ */
typedef struct {
	uint8_t ecc;
	const char *name;
} ecc_name_t;

static const ecc_name_t ecc_names[] = {
	/* European ECCs (0xE0-0xE4) */
	{ 0xE0, "Europe 1 (DE, IT, AT, etc.)" },
	{ 0xE1, "Europe 2 (GR, CH, FI, GB, FR)" },
	{ 0xE2, "Europe 3 (CZ, PL, ES, NO)" },
	{ 0xE3, "Europe 4 (IE, TR, NL, SE)" },
	{ 0xE4, "Europe 5 (UA, PT, SI)" },
	/* American ECCs (0xA0-0xA6) */
	{ 0xA0, "Americas - United States" },
	{ 0xA1, "Americas - Canada" },
	{ 0xA2, "Americas - Caribbean/South" },
	{ 0xA3, "Americas - Caribbean/Central" },
	{ 0xA4, "Americas - Central/South" },
	{ 0xA5, "Americas - Mexico" },
	/* African ECCs (0xD0-0xD3) */
	{ 0xD0, "Africa 1" },
	{ 0xD1, "Africa 2" },
	{ 0xD2, "Africa 3" },
	{ 0xD3, "Africa 4" },
	/* Asia/Pacific ECCs (0xF0-0xF3) */
	{ 0xF0, "Asia/Pacific 1 (AU, CN)" },
	{ 0xF1, "Asia/Pacific 2 (NZ, KR, HK)" },
	{ 0xF2, "Asia/Pacific 3 (IN, JP, SG)" },
	{ 0xF3, "Asia/Pacific 4 (TH)" },
	{ 0, NULL }  /* Terminator */
};

const char *rds_get_ecc_name(uint8_t ecc)
{
	for (const ecc_name_t *e = ecc_names; e->name != NULL; e++) {
		if (e->ecc == ecc)
			return e->name;
	}
	return "(Unknown ECC region)";
}

/* ============================================================
 * Open Data Application (ODA) IDs and Names
 * RDS Forum ODA registration documents
 * ============================================================ */

typedef struct {
	uint16_t aid;
	const char *name;
} oda_app_t;

static const oda_app_t oda_apps[] = {
	{ 0x0000, "None" },
	{ 0x0093, "Cross referencing DAB within RDS" },
	{ 0x0BCB, "Leisure & Practical Info for Drivers" },
	{ 0x0C24, "ELECTRABEL-DSM 7" },
	{ 0x0CC1, "WiPla Broadcast Control Signal" },
	{ 0x0D45, "RDS-TMC: ALERT-C (testing)" },
	{ 0x0D8B, "ELECTRABEL-DSM 18" },
	{ 0x0E2C, "ELECTRABEL-DSM 3" },
	{ 0x0E31, "ELECTRABEL-DSM 13" },
	{ 0x0F87, "ELECTRABEL-DSM 2" },
	{ 0x125F, "I-FM-RDS for Fixed and Mobile" },
	{ 0x1BDA, "ELECTRABEL-DSM 1" },
	{ 0x1C5E, "ELECTRABEL-DSM 20" },
	{ 0x1C68, "ITIS In-vehicle database" },
	{ 0x1CB1, "ELECTRABEL-DSM 10" },
	{ 0x1D47, "ELECTRABEL-DSM 4" },
	{ 0x1DC2, "CITIBUS 4" },
	{ 0x1DC5, "Encrypted TTI ALERT-Plus (testing)" },
	{ 0x1E8F, "ELECTRABEL-DSM 17" },
	{ 0x4400, "RDS-Light" },
	{ 0x4AA1, "RASANT" },
	{ 0x4AB7, "ELECTRABEL-DSM 9" },
	{ 0x4BA2, "ELECTRABEL-DSM 5" },
	{ 0x4BD7, "RT+ (RadioText Plus)" },
	{ 0x4BD8, "RT+ for eRT" },
	{ 0x4C59, "CITIBUS 2" },
	{ 0x4D87, "Radio Commerce System (RCS)" },
	{ 0x4D95, "ELECTRABEL-DSM 16" },
	{ 0x4D9A, "ELECTRABEL-DSM 11" },
	{ 0x50DD, "Disaster/Emergency Warning" },
	{ 0x5757, "Personal weather station" },
	{ 0x6363, "Hybradio RDS-Net (testing)" },
	{ 0x6365, "RDS2 9-bit AF lists" },
	{ 0x6552, "eRT (Enhanced RadioText)" },
	{ 0x6A7A, "Warning receiver" },
	{ 0x7373, "Enhanced early warning system" },
	{ 0xA112, "NL Alert system" },
	{ 0xA911, "Data FM Selective Multipoint" },
	{ 0xABCE, "Fleximax" },
	{ 0xABCF, "RF Power Monitoring" },
	{ 0xBE22, "Push-Ad" },
	{ 0xC350, "NRSC Song title and artist" },
	{ 0xC3A1, "Personal Radio Service" },
	{ 0xC3B0, "iTunes tagging" },
	{ 0xC3C3, "Traffic Plus" },
	{ 0xC4D4, "eEAS" },
	{ 0xC549, "Smart Grid Broadcast Channel" },
	{ 0xC563, "ID Logic" },
	{ 0xC6A7, "Veil Enabled Interactive Device" },
	{ 0xC737, "UMC - Utility Message Channel" },
	{ 0xCB73, "CITIBUS 1" },
	{ 0xCB97, "ELECTRABEL-DSM 14" },
	{ 0xCC21, "CITIBUS 3" },
	{ 0xCD19, "TokenMe" },
	{ 0xCD46, "RDS-TMC: ALERT-C" },
	{ 0xCD47, "RDS-TMC: ALERT-C" },
	{ 0xCD9E, "ELECTRABEL-DSM 8" },
	{ 0xCE6B, "Encrypted TTI ALERT-Plus (service)" },
	{ 0xE123, "APS Gateway" },
	{ 0xE1C1, "Action code" },
	{ 0xE319, "ELECTRABEL-DSM 12" },
	{ 0xE411, "Beacon downlink" },
	{ 0xE440, "ELECTRABEL-DSM 15" },
	{ 0xE4A6, "ELECTRABEL-DSM 19" },
	{ 0xE5D7, "ELECTRABEL-DSM 6" },
	{ 0xE911, "EAS open protocol" },
	{ 0xFF70, "Internet connection" },
	{ 0xFF7F, "RFT: Station logo" },
	{ 0xFF80, "RFT: Slideshow" },
	{ 0xFF81, "RFT: Journaline" },
	{ 0, NULL }  /* Terminator */
};

const char *rds_get_oda_name(uint16_t aid)
{
	for (const oda_app_t *o = oda_apps; o->name != NULL; o++) {
		if (o->aid == aid)
			return o->name;
	}
	return "(Unknown ODA)";
}


/* ============================================================
 * Linkage Information Codes (LIC)
 * IEC 62106 Group 14A
 * ============================================================ */
static const char *rds_linkage_codes[16] = {
	"No linkage",                       /* 0x0 */
	"Same Programme",                   /* 0x1 */
	"Temporarily Related Service",      /* 0x2 */
	"Traffic Information Service",      /* 0x3 */
	"Emergency / Alarm",                /* 0x4 */
	"Programme-Related Data",           /* 0x5 */
	"Digital Audio Service (DAB Link)", /* 0x6 */
	"Other Audio Service",              /* 0x7 */
	"Reserved (0x8)",                   /* 0x8 */
	"Reserved (0x9)",                   /* 0x9 */
	"Reserved (0xA)",                   /* 0xA */
	"Reserved (0xB)",                   /* 0xB */
	"Reserved (0xC)",                   /* 0xC */
	"Reserved (0xD)",                   /* 0xD */
	"Reserved (0xE)",                   /* 0xE */
	"Reserved (0xF)"                    /* 0xF */
};

const char *rds_get_linkage_name(uint8_t lic)
{
	if (lic > 15) return "Invalid";
	return rds_linkage_codes[lic];
}

/* ============================================================
 * DAB Frequencies (Blocks 13A/13B + ODA 0x0093)
 * Maps frequency (kHz) to DAB Channel ID (e.g. 5A)
 * Reference: TEF6686 / ETSI EN 300 401
 * ============================================================ */
static const struct {
	uint32_t frequency;
	const char *label;
} rds_dab_frequencies[] = {
	{ 174928,  "5A"}, { 176640,  "5B"}, { 178352,  "5C"}, { 180064,  "5D"},
	{ 181936,  "6A"}, { 183648,  "6B"}, { 185360,  "6C"}, { 187072,  "6D"},
	{ 188928,  "7A"}, { 190640,  "7B"}, { 192352,  "7C"}, { 194064,  "7D"},
	{ 195936,  "8A"}, { 197648,  "8B"}, { 199360,  "8C"}, { 201072,  "8D"},
	{ 202928,  "9A"}, { 204640,  "9B"}, { 206352,  "9C"}, { 208064,  "9D"},
	{ 209936, "10A"}, { 211648, "10B"}, { 213360, "10C"}, { 215072, "10D"},
	{ 216928, "11A"}, { 218640, "11B"}, { 220352, "11C"}, { 222064, "11D"},
	{ 223936, "12A"}, { 225648, "12B"}, { 227360, "12C"}, { 229072, "12D"},
	{ 230784, "13A"}, { 232496, "13B"}, { 234208, "13C"}, { 235776, "13D"},
	{ 237488, "13E"}, { 239200, "13F"}, {1452960,  "LA"}, {1454672,  "LB"},
	{1456384,  "LC"}, {1458096,  "LD"}, {1459808,  "LE"}, {1461520,  "LF"},
	{1463232,  "LG"}, {1464944,  "LH"}, {1466656,  "LI"}, {1468368,  "LJ"},
	{1470080,  "LK"}, {1471792,  "LL"}, {1473504,  "LM"}, {1475216,  "LN"},
	{1476928,  "LO"}, {1478640,  "LP"}, {1480352,  "LQ"}, {1482064,  "LR"},
	{1483776,  "LS"}, {1485488,  "LT"}, {1487200,  "LU"}, {1488912,  "LV"},
	{1490624,  "LW"},
	{ 0, NULL }
};

const char *rds_get_dab_channel_name(uint32_t frequency)
{
	for (int i = 0; rds_dab_frequencies[i].label != NULL; i++) {
		if (rds_dab_frequencies[i].frequency == frequency)
			return rds_dab_frequencies[i].label;
	}
	return "??";
}

/* ============================================================
 * Linked Station Codes (North America / RBDS)
 * NRSC-4-B / Consumer Receivers
 * ============================================================ */
static const struct {
	uint16_t pi;
	const char *label;
} rds_linked_station_codes[] = {
	{0xB001, "NPR-1"},
	{0xB002, "CBC English - Radio One"}, {0xB003, "CBC English - Radio Two"},
	{0xB004, "CBC French => Radio-Canada - Première Chaîne"},
	{0xB005, "CBC French => Radio-Canada - Espace Musique"},
	{0xB006, "CBC"},   {0xB007, "CBC"},   {0xB008, "CBC"},   {0xB009, "CBC"},
	{0xB00A, "NPR-2"}, {0xB00B, "NPR-3"}, {0xB00C, "NPR-4"},
	{0xB00D, "NPR-5"}, {0xB00E, "NPR-6"},
	{0, NULL}
};

const char *rds_get_linked_station_name(uint16_t pi)
{
	for (int i = 0; rds_linked_station_codes[i].label != NULL; i++) {
		if (rds_linked_station_codes[i].pi == pi)
			return rds_linked_station_codes[i].label;
	}
	return NULL; /* Not found */
}
/* ============================================================
 * RadioText+ (RT+) Content Types
 * RDS Forum R06/040_1 (2006-07-21)
 * ============================================================ */
static const char *rtplus_content_types[] = {
	"dummy_class",          /* 0 */
	"item.title",           /* 1 */
	"item.album",           /* 2 */
	"item.tracknumber",     /* 3 */
	"item.artist",          /* 4 */
	"item.composition",     /* 5 */
	"item.movement",        /* 6 */
	"item.conductor",       /* 7 */
	"item.composer",        /* 8 */
	"item.band",            /* 9 */
	"item.comment",         /* 10 */
	"item.genre",           /* 11 */
	"info.news",            /* 12 */
	"info.news.local",      /* 13 */
	"info.stockmarket",     /* 14 */
	"info.sport",           /* 15 */
	"info.lottery",         /* 16 */
	"info.horoscope",       /* 17 */
	"info.daily_diversion", /* 18 */
	"info.health",          /* 19 */
	"info.event",           /* 20 */
	"info.scene",           /* 21 */
	"info.cinema",          /* 22 */
	"info.tv",              /* 23 */
	"info.date_time",       /* 24 */
	"info.weather",         /* 25 */
	"info.traffic",         /* 26 */
	"info.alarm",           /* 27 */
	"info.advertisement",   /* 28 */
	"info.url",             /* 29 */
	"info.other",           /* 30 */
	"stationname.short",    /* 31 */
	"stationname.long",     /* 32 */
	"programme.now",        /* 33 */
	"programme.next",       /* 34 */
	"programme.part",       /* 35 */
	"programme.host",       /* 36 */
	"programme.editorial_staff", /* 37 */
	"programme.frequency",  /* 38 */
	"programme.homepage",   /* 39 */
	"programme.subchannel", /* 40 */
	"phone.hotline",        /* 41 */
	"phone.studio",         /* 42 */
	"phone.other",          /* 43 */
	"sms.studio",           /* 44 */
	"sms.other",            /* 45 */
	"email.hotline",        /* 46 */
	"email.studio",         /* 47 */
	"email.other",          /* 48 */
	"mms.other",            /* 49 */
	"chat",                 /* 50 */
	"chat.centre",          /* 51 */
	"vote.question",        /* 52 */
	"vote.centre",          /* 53 */
	NULL, NULL, NULL, NULL, /* 54-57 reserved */
	NULL, NULL,             /* 58-59 reserved */
	"place",                /* 60 */
	"appointment",          /* 61 */
	"identifier",           /* 62 */
	"purchase",             /* 63 */
	"get_data"              /* 64 */
};

#define RTPLUS_TYPE_COUNT (sizeof(rtplus_content_types) / sizeof(rtplus_content_types[0]))

const char *rds_get_rtplus_content_type(uint8_t type)
{
	if (type >= RTPLUS_TYPE_COUNT)
		return "(Unknown)";
	const char *name = rtplus_content_types[type];
	return name ? name : "(reserved)";
}

/* ============================================================
 * RDS Character Map (Basic + Extended)
 * IEC 62106 Annex E - RDS character set
 * Maps 8-bit RDS codes to UTF-8 strings
 * ============================================================ */
static const char *rds_charmap[256] = {
	/* 0x00-0x0F: Control characters */
	" ", " ", " ", " ", " ", " ", " ", " ",
	" ", " ", "\n", " ", " ", "\r", " ", " ",
	/* 0x10-0x1F: Control characters */
	" ", " ", " ", " ", " ", " ", " ", " ",
	" ", " ", " ", " ", " ", " ", " ", " ",
	/* 0x20-0x2F: Basic ASCII punctuation */
	" ", "!", "\"", "#", "\xC2\xA4", "%", "&", "'",
	"(", ")", "*", "+", ",", "-", ".", "/",
	/* 0x30-0x3F: Digits */
	"0", "1", "2", "3", "4", "5", "6", "7",
	"8", "9", ":", ";", "<", "=", ">", "?",
	/* 0x40-0x4F: Uppercase A-O */
	"@", "A", "B", "C", "D", "E", "F", "G",
	"H", "I", "J", "K", "L", "M", "N", "O",
	/* 0x50-0x5F: Uppercase P-Z and punctuation */
	"P", "Q", "R", "S", "T", "U", "V", "W",
	"X", "Y", "Z", "[", "\\", "]", "\xE2\x80\x95", "_",
	/* 0x60-0x6F: Lowercase a-o */
	"\xE2\x95\x91", "a", "b", "c", "d", "e", "f", "g",
	"h", "i", "j", "k", "l", "m", "n", "o",
	/* 0x70-0x7F: Lowercase p-z and punctuation */
	"p", "q", "r", "s", "t", "u", "v", "w",
	"x", "y", "z", "{", "|", "}", "\xC2\xAF", "\x7F",
	/* 0x80-0x8F: Latin extended (accented lowercase) */
	"\xC3\xA1", "\xC3\xA0", "\xC3\xA9", "\xC3\xA8",
	"\xC3\xAD", "\xC3\xAC", "\xC3\xB3", "\xC3\xB2",
	"\xC3\xBA", "\xC3\xB9", "\xC3\x91", "\xC3\x87",
	"\xC5\x9E", "\xC3\x9F", "\xC2\xA1", "\xC4\xB2",
	/* 0x90-0x9F: More Latin extended */
	"\xC3\xA2", "\xC3\xA4", "\xC3\xAA", "\xC3\xAB",
	"\xC3\xAE", "\xC3\xAF", "\xC3\xB4", "\xC3\xB6",
	"\xC3\xBB", "\xC3\xBC", "\xC3\xB1", "\xC3\xA7",
	"\xC5\x9F", "\xC4\x9F", "\xC4\xB1", "\xC4\xB3",
	/* 0xA0-0xAF: Symbols and Greek */
	"\xC2\xAA", "\xCE\xB1", "\xC2\xA9", "\xE2\x80\xB0",
	"\xC4\x9E", "\xC4\x9B", "\xC5\x88", "\xC5\x91",
	"\xCF\x80", "\xE2\x82\xAC", "\xC2\xA3", "$",
	"\xE2\x86\x90", "\xE2\x86\x91", "\xE2\x86\x92", "\xE2\x86\x93",
	/* 0xB0-0xBF: More symbols */
	"\xC2\xBA", "\xC2\xB9", "\xC2\xB2", "\xC2\xB3",
	"\xC2\xB1", "\xC4\xB0", "\xC5\x84", "\xC5\xB1",
	"\xC2\xB5", "\xC2\xBF", "\xC3\xB7", "\xC2\xB0",
	"\xC2\xBC", "\xC2\xBD", "\xC2\xBE", "\xC2\xA7",
	/* 0xC0-0xCF: Accented uppercase */
	"\xC3\x81", "\xC3\x80", "\xC3\x89", "\xC3\x88",
	"\xC3\x8D", "\xC3\x8C", "\xC3\x93", "\xC3\x92",
	"\xC3\x9A", "\xC3\x99", "\xC5\x98", "\xC4\x8C",
	"\xC5\xA0", "\xC5\xBD", "\xC4\x90", "\xC4\xBF",
	/* 0xD0-0xDF: More accented uppercase */
	"\xC3\x82", "\xC3\x84", "\xC3\x8A", "\xC3\x8B",
	"\xC3\x8E", "\xC3\x8F", "\xC3\x94", "\xC3\x96",
	"\xC3\x9B", "\xC3\x9C", "\xC5\x99", "\xC4\x8D",
	"\xC5\xA1", "\xC5\xBE", "\xC4\x91", "\xC5\x80",
	/* 0xE0-0xEF: Nordic/special uppercase */
	"\xC3\x83", "\xC3\x85", "\xC3\x86", "\xC5\x92",
	"\xC5\xB7", "\xC3\x9D", "\xC3\x95", "\xC3\x98",
	"\xC3\x9E", "\xC5\x8A", "\xC5\x94", "\xC4\x86",
	"\xC5\x9A", "\xC5\xB9", "\xC5\xA6", "\xC3\xB0",
	/* 0xF0-0xFF: Nordic/special lowercase */
	"\xC3\xA3", "\xC3\xA5", "\xC3\xA6", "\xC5\x93",
	"\xC5\xB5", "\xC3\xBD", "\xC3\xB5", "\xC3\xB8",
	"\xC3\xBE", "\xC5\x8B", "\xC5\x95", "\xC4\x87",
	"\xC5\x9B", "\xC5\xBA", "\xC5\xA7", " "
};

const char *rds_get_char(uint8_t code)
{
	return rds_charmap[code];
}

/* ============================================================
 * RDS Character Display Functions
 * ============================================================ */

const char *rds_char_to_display(uint8_t code)
{
	static char hex_buf[8];
	
	/* Control characters shown as Unicode control pictures or <XX> */
	if (code < 0x20) {
		if (code == 0x0A) {
			return "\xE2\x90\x8A";  /* U+240A ␊ Line Feed symbol */
		} else if (code == 0x0D) {
			return "\xE2\x90\x8D";  /* U+240D ␍ Carriage Return symbol */
		} else {
			snprintf(hex_buf, sizeof(hex_buf), "<%02X>", code);
			return hex_buf;
		}
	}
	
	/* DEL character (0x7F) */
	if (code == 0x7F) {
		return "<7F>";
	}
	
	/* Normal printable character - return UTF-8 equivalent */
	return rds_charmap[code];
}

int rds_text_to_display(const uint8_t *rds_text, int len, char *out, int out_size)
{
	int written = 0;
	int i;
	
	if (!rds_text || !out || out_size < 1) return 0;
	
	for (i = 0; i < len && written < out_size - 1; i++) {
		uint8_t code = rds_text[i];
		const char *disp = rds_char_to_display(code);
		int disp_len = strlen(disp);
		
		if (written + disp_len >= out_size) break;
		
		strcpy(out + written, disp);
		written += disp_len;
		
		/* Stop at CR terminator (but still display it) */
		if (code == 0x0D) break;
	}
	
	out[written] = '\0';
	return written;
}

/* ============================================================
 * Reverse Character Map: UTF-8 Code Point -> RDS Code
 * ============================================================ */

/* Structure for reverse lookup */
typedef struct {
	uint32_t codepoint;  /* Unicode code point */
	uint8_t rds_code;    /* RDS 8-bit code */
} unicode_to_rds_t;

/* Sorted by codepoint for binary search */
static const unicode_to_rds_t unicode_to_rds_map[] = {
	/* ASCII printable (0x20-0x7E) map 1:1 - not in table */
	/* Extended characters from Annex E */
	{0x00A1, 0x8E},  /* ¡ */
	{0x00A3, 0xAA},  /* £ */
	{0x00A4, 0x24},  /* ¤ (currency sign at 0x24, replaces $) */
	{0x00A7, 0xBF},  /* § */
	{0x00A9, 0xA2},  /* © */
	{0x00AA, 0xA0},  /* ª */
	{0x00AF, 0x7E},  /* ¯ (macron) */
	{0x00B0, 0xBB},  /* ° */
	{0x00B1, 0xB4},  /* ± */
	{0x00B2, 0xB2},  /* ² */
	{0x00B3, 0xB3},  /* ³ */
	{0x00B5, 0xB8},  /* µ */
	{0x00B9, 0xB1},  /* ¹ */
	{0x00BA, 0xB0},  /* º */
	{0x00BC, 0xBC},  /* ¼ */
	{0x00BD, 0xBD},  /* ½ */
	{0x00BE, 0xBE},  /* ¾ */
	{0x00BF, 0xB9},  /* ¿ */
	{0x00C0, 0xC1},  /* À */
	{0x00C1, 0xC0},  /* Á */
	{0x00C2, 0xD0},  /* Â */
	{0x00C3, 0xE0},  /* Ã */
	{0x00C4, 0xD1},  /* Ä */
	{0x00C5, 0xE1},  /* Å */
	{0x00C6, 0xE2},  /* Æ */
	{0x00C7, 0x8B},  /* Ç */
	{0x00C8, 0xC3},  /* È */
	{0x00C9, 0xC2},  /* É */
	{0x00CA, 0xD2},  /* Ê */
	{0x00CB, 0xD3},  /* Ë */
	{0x00CC, 0xC5},  /* Ì */
	{0x00CD, 0xC4},  /* Í */
	{0x00CE, 0xD4},  /* Î */
	{0x00CF, 0xD5},  /* Ï */
	{0x00D0, 0xCE},  /* Ð */
	{0x00D1, 0x8A},  /* Ñ */
	{0x00D2, 0xC7},  /* Ò */
	{0x00D3, 0xC6},  /* Ó */
	{0x00D4, 0xD6},  /* Ô */
	{0x00D5, 0xE6},  /* Õ */
	{0x00D6, 0xD7},  /* Ö */
	{0x00D7, 0xBA},  /* × (division symbol) */
	{0x00D8, 0xE7},  /* Ø */
	{0x00D9, 0xC9},  /* Ù */
	{0x00DA, 0xC8},  /* Ú */
	{0x00DB, 0xD8},  /* Û */
	{0x00DC, 0xD9},  /* Ü */
	{0x00DD, 0xE5},  /* Ý */
	{0x00DE, 0xE8},  /* Þ */
	{0x00DF, 0x8D},  /* ß */
	{0x00E0, 0x81},  /* à */
	{0x00E1, 0x80},  /* á */
	{0x00E2, 0x90},  /* â */
	{0x00E3, 0xF0},  /* ã */
	{0x00E4, 0x91},  /* ä */
	{0x00E5, 0xF1},  /* å */
	{0x00E6, 0xF2},  /* æ */
	{0x00E7, 0x9B},  /* ç */
	{0x00E8, 0x83},  /* è */
	{0x00E9, 0x82},  /* é */
	{0x00EA, 0x92},  /* ê */
	{0x00EB, 0x93},  /* ë */
	{0x00EC, 0x85},  /* ì */
	{0x00ED, 0x84},  /* í */
	{0x00EE, 0x94},  /* î */
	{0x00EF, 0x95},  /* ï */
	{0x00F0, 0xEF},  /* ð */
	{0x00F1, 0x9A},  /* ñ */
	{0x00F2, 0x87},  /* ò */
	{0x00F3, 0x86},  /* ó */
	{0x00F4, 0x96},  /* ô */
	{0x00F5, 0xF6},  /* õ */
	{0x00F6, 0x97},  /* ö */
	{0x00F7, 0xBA},  /* ÷ */
	{0x00F8, 0xF7},  /* ø */
	{0x00F9, 0x89},  /* ù */
	{0x00FA, 0x88},  /* ú */
	{0x00FB, 0x98},  /* û */
	{0x00FC, 0x99},  /* ü */
	{0x00FD, 0xF5},  /* ý */
	{0x00FE, 0xF8},  /* þ */
	{0x0132, 0x8F},  /* Ĳ (Dutch ligature) */
	{0x0133, 0x9F},  /* ĳ */
	{0x0141, 0xCF},  /* Ł */
	{0x0142, 0xDF},  /* ł */
	{0x0152, 0xE3},  /* Œ */
	{0x0153, 0xF3},  /* œ */
	{0x0158, 0xCA},  /* Ř */
	{0x0159, 0xDA},  /* ř */
	{0x015A, 0xEC},  /* Ś */
	{0x015B, 0xFC},  /* ś */
	{0x015E, 0x8C},  /* Ş */
	{0x015F, 0x9C},  /* ş */
	{0x0160, 0xCC},  /* Š */
	{0x0161, 0xDC},  /* š */
	{0x0166, 0xEE},  /* Ŧ */
	{0x0167, 0xFE},  /* ŧ */
	{0x017D, 0xCD},  /* Ž */
	{0x017E, 0xDD},  /* ž */
	{0x010C, 0xCB},  /* Č */
	{0x010D, 0xDB},  /* č */
	{0x0110, 0xCE},  /* Đ */
	{0x0111, 0xDE},  /* đ */
	{0x011A, 0xA5},  /* Ě */
	{0x011B, 0xA5},  /* ě */
	{0x0130, 0xB5},  /* İ (Turkish I with dot) */
	{0x0131, 0x9E},  /* ı (Turkish dotless i) */
	{0x013F, 0x3F},  /* Ŀ */
	{0x0140, 0x60},  /* ŀ */
	{0x0147, 0xA6},  /* Ň */
	{0x0148, 0xA6},  /* ň */
	{0x0150, 0xA7},  /* Ő */
	{0x0151, 0xA7},  /* ő */
	{0x0154, 0xEA},  /* Ŕ */
	{0x0155, 0xFA},  /* ŕ */
	{0x0164, 0xB6},  /* Ť */
	{0x0165, 0xB6},  /* ť */
	{0x0170, 0xB7},  /* Ű */
	{0x0171, 0xB7},  /* ű */
	{0x0176, 0xE4},  /* Ŷ */
	{0x0177, 0xF4},  /* ŷ */
	{0x0179, 0xED},  /* Ź */
	{0x017A, 0xFD},  /* ź */
	{0x0186, 0xEB},  /* Ɔ */
	{0x0187, 0xFB},  /* Ƈ */
	{0x018A, 0xE9},  /* Ɗ */
	{0x019A, 0x9E},  /* ƚ */
	{0x01A4, 0x9D},  /* Ƥ */
	{0x01B5, 0xA4},  /* Ƶ */
	{0x03B1, 0xA1},  /* α (Greek alpha) */
	{0x03C0, 0xA8},  /* π (Greek pi) */
	{0x2015, 0x5E},  /* ― (horizontal bar) */
	{0x2030, 0xA3},  /* ‰ (per mille) */
	{0x20AC, 0xA9},  /* € (Euro) */
	{0x2190, 0xAC},  /* ← */
	{0x2191, 0xAD},  /* ↑ */
	{0x2192, 0xAE},  /* → */
	{0x2193, 0xAF},  /* ↓ */
	{0x2551, 0x60},  /* ║ (box drawing) */
};

#define UNICODE_TO_RDS_COUNT (sizeof(unicode_to_rds_map) / sizeof(unicode_to_rds_map[0]))

/* Decode UTF-8 sequence, return codepoint and advance pointer */
static uint32_t utf8_decode(const char **p)
{
	const uint8_t *s = (const uint8_t *)*p;
	uint32_t cp;
	
	if ((s[0] & 0x80) == 0) {
		cp = s[0];
		*p += 1;
	} else if ((s[0] & 0xE0) == 0xC0) {
		cp = ((s[0] & 0x1F) << 6) | (s[1] & 0x3F);
		*p += 2;
	} else if ((s[0] & 0xF0) == 0xE0) {
		cp = ((s[0] & 0x0F) << 12) | ((s[1] & 0x3F) << 6) | (s[2] & 0x3F);
		*p += 3;
	} else if ((s[0] & 0xF8) == 0xF0) {
		cp = ((s[0] & 0x07) << 18) | ((s[1] & 0x3F) << 12) | 
		     ((s[2] & 0x3F) << 6) | (s[3] & 0x3F);
		*p += 4;
	} else {
		cp = '?';
		*p += 1;
	}
	return cp;
}

/* Binary search for codepoint in reverse map */
static int find_rds_code(uint32_t codepoint, uint8_t *rds_code)
{
	/* ASCII printable maps directly */
	if (codepoint >= 0x20 && codepoint <= 0x7E) {
		*rds_code = (uint8_t)codepoint;
		return 1;
	}
	
	/* LF and CR are special */
	if (codepoint == 0x0A || codepoint == 0x0D) {
		*rds_code = (uint8_t)codepoint;
		return 1;
	}
	
	/* Binary search in extended map */
	int lo = 0, hi = UNICODE_TO_RDS_COUNT - 1;
	while (lo <= hi) {
		int mid = (lo + hi) / 2;
		if (unicode_to_rds_map[mid].codepoint == codepoint) {
			*rds_code = unicode_to_rds_map[mid].rds_code;
			return 1;
		} else if (unicode_to_rds_map[mid].codepoint < codepoint) {
			lo = mid + 1;
		} else {
			hi = mid - 1;
		}
	}
	
	return 0; /* Not found */
}

int rds_encode_text(const char *utf8, uint8_t *rds_out, int max_len, int *warn_unencodable)
{
	int out_len = 0;
	const char *p = utf8;
	int warned = 0;
	
	if (!utf8 || !rds_out || max_len < 1) return 0;
	
	while (*p && out_len < max_len) {
		uint32_t cp = utf8_decode(&p);
		uint8_t rds_code;
		
		/* Skip control characters except LF and CR */
		if (cp < 0x20 && cp != 0x0A && cp != 0x0D) {
			continue;
		}
		
		/* CR terminates the message */
		if (cp == 0x0D) {
			if (out_len < max_len) {
				rds_out[out_len++] = 0x0D;
			}
			break;
		}
		
		if (find_rds_code(cp, &rds_code)) {
			rds_out[out_len++] = rds_code;
		} else {
			/* Non-encodable - replace with '?' (standard convention) */
			rds_out[out_len++] = '?';
			warned = 1;
		}
	}
	
	if (warn_unencodable) {
		*warn_unencodable = warned;
	}
	
	return out_len;
}

int rds_validate_text(const char *utf8, const char *field_name)
{
	const char *p = utf8;
	int bad_count = 0;
	
	if (!utf8) return 0;
	
	while (*p) {
		uint32_t cp = utf8_decode(&p);
		uint8_t rds_code;
		
		/* Skip control characters - they're filtered */
		if (cp < 0x20 && cp != 0x0A && cp != 0x0D) {
			continue;
		}
		
		if (!find_rds_code(cp, &rds_code)) {
			bad_count++;
		}
	}
	
	if (bad_count > 0) {
		LOGP(DRADIO, LOGL_NOTICE, 
		     "RDS %s: %d character(s) cannot be encoded in RDS charset, replaced with '?'\n",
		     field_name ? field_name : "text", bad_count);
	}
	
	return bad_count;
}


/* ============================================================
 * RBDS Call Sign Decoding (North America)
 * NRSC-4-B (2011), Annex D.7
 * ============================================================ */

/* 3-Letter Call Sign Exceptions (Range 9950-9EFF) */
typedef struct {
	uint16_t pi;
	const char *call;
} three_letter_code_t;

static const three_letter_code_t three_letter_codes[] = {
	{0x99A5, "KBW"}, {0x9992, "KOY"}, {0x9978, "WHO"}, {0x99A6, "KCY"},
	{0x9993, "KPQ"}, {0x999C, "WHP"}, {0x9990, "KDB"}, {0x9964, "KQV"},
	{0x999D, "WIL"}, {0x99A7, "KDF"}, {0x9994, "KSD"}, {0x997A, "WIP"},
	{0x9950, "KEX"}, {0x9965, "KSL"}, {0x99B3, "WIS"}, {0x9951, "KFH"},
	{0x9966, "KUJ"}, {0x997B, "WJR"}, {0x9952, "KFI"}, {0x9995, "KUT"},
	{0x99B4, "WJW"}, {0x9953, "KGA"}, {0x9967, "KVI"}, {0x99B5, "WJZ"},
	{0x9991, "KGB"}, {0x9968, "KWG"}, {0x997C, "WKY"}, {0x9954, "KGO"},
	{0x9996, "KXL"}, {0x997D, "WLS"}, {0x9955, "KGU"}, {0x9997, "KXO"},
	{0x997E, "WLW"}, {0x9956, "KGW"}, {0x996B, "KYW"}, {0x999E, "WMC"},
	{0x9957, "KGY"}, {0x9999, "WBT"}, {0x999F, "WMT"}, {0x99AA, "KHQ"},
	{0x996D, "WBZ"}, {0x9981, "WOC"}, {0x9958, "KID"}, {0x996E, "WDZ"},
	{0x99A0, "WOI"}, {0x9959, "KIT"}, {0x996F, "WEW"}, {0x9983, "WOL"},
	{0x995A, "KJR"}, {0x999A, "WGH"}, {0x9984, "WOR"}, {0x995B, "KLO"},
	{0x9971, "WGL"}, {0x99A1, "WOW"}, {0x995C, "KLZ"}, {0x9972, "WGN"},
	{0x99B9, "WRC"}, {0x995D, "KMA"}, {0x9973, "WGR"}, {0x99A2, "WRR"},
	{0x995E, "KMJ"}, {0x999B, "WGY"}, {0x99A3, "WSB"}, {0x995F, "KNX"},
	{0x9975, "WHA"}, {0x99A4, "WSM"}, {0x9960, "KOA"}, {0x9976, "WHB"},
	{0x9988, "WWJ"}, {0x99AB, "KOB"}, {0x9977, "WHK"}, {0x9989, "WWL"},
	{0, NULL}
};

/* Linked Station Codes */
static const three_letter_code_t linked_codes[] = {
	{0xB001, "NPR-1"}, {0xB002, "CBC Radio One"}, {0xB003, "CBC Radio Two"},
	{0xB004, "SRC Premiere"}, {0xB005, "SRC Espace Mus"},
	{0xB006, "CBC"}, {0xB007, "CBC"}, {0xB008, "CBC"}, {0xB009, "CBC"},
	{0xB00A, "NPR-2"}, {0xB00B, "NPR-3"}, {0xB00C, "NPR-4"},
	{0xB00D, "NPR-5"}, {0xB00E, "NPR-6"},
	{0, NULL}
};

const char *rds_get_callsign(uint16_t pi)
{
	static char callsun_buf[16];
	
	/* Adjust for nibble-zero exceptions (NRSC-4-B D.7) */
	if ((pi & 0xFFF0) == 0xAFA0 && (pi & 0x000F) < 0x000A) {
		pi <<= 12;
	} else if ((pi & 0xFF00) == 0xAF00) {
		pi <<= 8;
	} else if ((pi & 0xF000) == 0xA000) {
		pi = (uint16_t)(((pi & 0x0F00) << 4) | (pi & 0x00FF));
	}

	/* Check 3-Letter Exceptions */
	if (pi >= 0x9950 && pi <= 0x9EFF) {
		for (const three_letter_code_t *c = three_letter_codes; c->call != NULL; c++) {
			if (c->pi == pi) return c->call;
		}
		/* Fallback for unmapped codes in 3-letter range */
		return "W??"; 
	}

	/* Check Linked Stations (B, D, E) */
	if ((pi >> 12) == 0xB || (pi >> 12) == 0xD || (pi >> 12) == 0xE) {
		for (const three_letter_code_t *l = linked_codes; l->call != NULL; l++) {
			if (l->pi == (pi & 0xF0FF)) return l->call;
		}
		return "LINK"; 
	}

	/* Standard 4-Letter Calculation */
	if (pi >= 0x1000 && pi <= 0x994F) {
		char prefix;
		uint32_t val;
		
		if (pi <= 0x54A7) {
			prefix = 'K';
			val = pi - 0x1000;
		} else {
			prefix = 'W';
			val = pi - 0x54A8;
		}
		
		/* Base 26 decode */
		callsun_buf[0] = prefix;
		callsun_buf[1] = 'A' + ((val / (26 * 26)) % 26);
		callsun_buf[2] = 'A' + ((val / 26) % 26);
		callsun_buf[3] = 'A' + (val % 26);
		callsun_buf[4] = '\0';
		
		return callsun_buf;
	}

	return NULL;
}

uint16_t rds_get_pi_from_callsign(const char *call)
{
	if (!call)
		return 0;

	size_t len = strlen(call);

	/* Check 3-Letter Exceptions */
	if (len == 3) {
		for (const three_letter_code_t *c = three_letter_codes; c->call != NULL; c++) {
			if (strcasecmp(c->call, call) == 0)
				return c->pi;
		}
		return 0;
	}

	/* Check Linked Stations */
	/* Helper for linked stations: check against linked_codes */
	/* linked_codes contains "NPR-1" etc, so length > 3 */
	for (const three_letter_code_t *l = linked_codes; l->call != NULL; l++) {
		if (strcasecmp(l->call, call) == 0)
			return l->pi | 0xB000; /* Re-apply B-linkage? No, l->pi is 4 digits in table? */
			/* Wait, linked_codes table in rds_callsign.c has full PI?
			   Let's check code in rds_callsign.c artifact:
			   {0xB001, "NPR-1"}, ...
			   Yes, full PI.
			 */
	}

	/* Check 4-Letter Standard */
	if (len == 4) {
		char c1 = toupper((unsigned char)call[0]);
		if (c1 != 'K' && c1 != 'W')
			return 0;
		
		/* Verify remaining chars are letters */
		if (!isalpha(call[1]) || !isalpha(call[2]) || !isalpha(call[3]))
			return 0;

		uint32_t val = 0;
		val += (toupper((unsigned char)call[1]) - 'A') * 676;
		val += (toupper((unsigned char)call[2]) - 'A') * 26;
		val += (toupper((unsigned char)call[3]) - 'A');
		
		uint16_t pi = 0;
		if (c1 == 'K') {
			pi = 0x1000 + val;
			if (pi > 0x54A7) return 0; /* Overflow check, though math says it fits */
		} else {
			pi = 0x54A8 + val;
			if (pi > 0x994F) return 0;
		}
		return pi;
	}

	return 0;
}
