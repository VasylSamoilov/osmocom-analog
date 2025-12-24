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
static const char *language_names[128] = {
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
	NULL, NULL,       /* 68-69 reserved */
	"Zulu",          /* 70 */
	"Vietnamese",    /* 71 */
	"Uzbek",         /* 72 */
	"Urdu",          /* 73 */
	"Ukrainian",     /* 74 */
	"Thai",          /* 75 */
	"Telugu",        /* 76 */
	"Tatar",         /* 77 */
	"Tamil",         /* 78 */
	"Tadzhik",       /* 79 */
	"Swahili",       /* 80 */
	"Sranan Tongo",  /* 81 */
	"Somali",        /* 82 */
	"Sinhalese",     /* 83 */
	"Shona",         /* 84 */
	"Serbo-Croat",   /* 85 */
	"Ruthenian",     /* 86 */
	"Russian",       /* 87 */
	"Quechua",       /* 88 */
	"Pushtu",        /* 89 */
	"Punjabi",       /* 90 */
	"Persian",       /* 91 */
	"Papamiento",    /* 92 */
	"Oriya",         /* 93 */
	"Nepali",        /* 94 */
	"Ndebele",       /* 95 */
	"Marathi",       /* 96 */
	"Moldovian",     /* 97 */
	"Malaysian",     /* 98 */
	"Malagasy",      /* 99 */
	"Macedonian",    /* 100 */
	"Laotian",       /* 101 */
	"Korean",        /* 102 */
	"Khmer",         /* 103 */
	"Kazakh",        /* 104 */
	"Kannada",       /* 105 */
	"Japanese",      /* 106 */
	"Indonesian",    /* 107 */
	"Hindi",         /* 108 */
	"Hebrew",        /* 109 */
	"Hausa",         /* 110 */
	"Gurani",        /* 111 */
	"Gujarati",      /* 112 */
	"Greek",         /* 113 */
	"Georgian",      /* 114 */
	"Fulani",        /* 115 */
	"Dari",          /* 116 */
	"Chuvash",       /* 117 */
	"Chinese",       /* 118 */
	"Burmese",       /* 119 */
	"Bulgarian",     /* 120 */
	"Bengali",       /* 121 */
	"Belorussian",   /* 122 */
	"Bambora",       /* 123 */
	"Azerbaijani",   /* 124 */
	"Assamese",      /* 125 */
	"Armenian",      /* 126 */
	"Arabic"         /* 127 */
};

const char *rds_get_language_name(uint8_t code)
{
	if (code > 127)
		return "Invalid";
	const char *name = language_names[code];
	return name ? name : "(reserved)";
}

/* ============================================================
 * Decoder Identification (DI) Names
 * IEC 62106 S3.2.1.5 (EN 50067:1998, p. 41)
 * Address determines which DI bit is being transmitted
 * ============================================================ */
static const char *di_names[4] = {
	"dynamic_pty",    /* addr 0: d0 - Dynamic PTY indicator */
	"compressed",     /* addr 1: d1 - Compressed audio */
	"artificial_head",/* addr 2: d2 - Artificial head recording */
	"stereo"          /* addr 3: d3 - Stereo broadcast */
};

const char *rds_get_di_name(uint8_t addr)
{
	if (addr > 3)
		return "invalid";
	return di_names[addr];
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
 * Extended Country Code (ECC) Region Names
 * IEC 62106 Annex D - Human-readable region descriptions
 * Used for educational logging to explain what ECC value means
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
 * Open Data Application (ODA) Names
 * RDS Forum registration documents
 * ============================================================ */

typedef struct {
	uint16_t aid;
	const char *name;
} oda_app_t;

static const oda_app_t oda_apps[] = {
	{ 0x0000, "None" },
	{ 0x0093, "DAB cross-reference" },
	{ 0x0D45, "TMC ALERT-C" },
	{ 0x4BD7, "RT+" },
	{ 0x4BD8, "RT+ for eRT" },
	{ 0x6552, "eRT (enhanced RadioText)" },
	{ 0xC350, "NRSC Song Title/Artist" },
	{ 0xC3B0, "iTunes Tagging" },
	{ 0xC3C3, "NAVTEQ Traffic Plus" },
	{ 0xCD46, "TMC ALERT-C" },
	{ 0xCD47, "TMC ALERT-C" },
	{ 0xFF7F, "RFT Station logo" },
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
