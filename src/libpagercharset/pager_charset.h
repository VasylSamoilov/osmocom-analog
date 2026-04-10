/*
 * Pager character set library — shared codepage tables for paging protocols.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Supports protocols with different code widths:
 *   POCSAG: 7-bit (128 codes)
 *   FLEX:   7-bit (128 codes)
 *   Golay:  6-bit (64 codes)
 *   RDS:    8-bit (256 codes) — IEC 62106 Annex E and pager variants
 *
 * Each codepage is identified by four fields:
 *   protocol - paging protocol ("pocsag", "flex", "golay", "rds")
 *   vendor   - pager manufacturer ("motorola", "philips", "generic", "nokia", ...)
 *   model    - pager model or family ("advisor-linguist", "prg2220", "nkp2", ...)
 *   charset  - character set description ("ascii", "latin-german", "annex-e", ...)
 */

#ifndef PAGER_CHARSET_H
#define PAGER_CHARSET_H

#include <stdint.h>
#include <stddef.h>

#define PAGER_CHARSET_MAX_CODES	256	/* max entries (8-bit protocols) */

/*
 * Single entry in a pager codepage table.
 *
 * A NULL-terminated array of UTF-8 strings:
 *   [0]   = primary glyph, emitted on RX decode.  NULL = unmapped/control.
 *   [1..] = aliases, additional UTF-8 strings accepted on TX encode.
 *            Terminated by NULL (remaining slots zero-init to NULL).
 *
 * Examples:
 *   { "A" }                              — plain ASCII, no aliases
 *   { "А", "A", "a", "а" }              — Cyrillic А, TX also accepts Latin A/a and lowercase а
 *   { "Е", "E", "e", "е", "Ё", "ё" }   — Cyrillic Е with 5 aliases
 *   { NULL }                             — unmapped code (control char)
 *
 * PAGER_CHARSET_ENTRY_MAX defines the array size (primary + aliases + NULL).
 * If you need more than 7 aliases for a single code, increase this.
 */
#define PAGER_CHARSET_ENTRY_MAX	8

typedef const char *pager_charset_entry[PAGER_CHARSET_ENTRY_MAX];

/*
 * Complete pager codepage.
 *
 * Naming: protocol-vendor-model-charset
 *   protocol  - "pocsag", "flex", "golay"
 *   vendor    - "motorola", "philips", "nec", "generic"
 *   model     - "advisor", "advisor-linguist", "prg2220", "generic"
 *   charset   - "ascii", "latin-german", "latin-cyrillic", "cyrillic"
 *
 * code_bits - number of bits per character code (6 for Golay, 7 for POCSAG/FLEX)
 * num_codes - number of entries: 1 << code_bits (64 or 128)
 * entries   - array of num_codes entries, indexed by code value
 */
struct pager_charset {
	const char			*protocol;
	const char			*vendor;
	const char			*model;
	const char			*charset;
	int				code_bits;
	int				num_codes;
	const pager_charset_entry	*entries;
};

/* --- Lookup API --- */

/* Find charset by field values.  Any field may be NULL to match all.
 * Returns first match, or NULL if not found. */
const struct pager_charset *pager_charset_find(const char *protocol,
					       const char *vendor,
					       const char *model,
					       const char *charset);

/* Find charset by combined "protocol-vendor-model-charset" string.
 * Returns NULL if not found. */
const struct pager_charset *pager_charset_by_name(const char *name);

/* RX decode: code → UTF-8 string.  Returns NULL if unmapped.
 * Code is masked to code_bits width automatically. */
const char *pager_charset_decode(const struct pager_charset *cs, uint8_t code);

/* TX encode: consume one UTF-8 character from *utf8, return code value.
 * Advances *utf8 past the consumed character.  Returns -1 if no match. */
int pager_charset_encode(const struct pager_charset *cs, const char **utf8);

/* Print all registered charsets to stdout. */
void pager_charset_list(void);

/* --- Built-in POCSAG codepages (7-bit, 128 codes) --- */

extern const struct pager_charset pager_charset_pocsag_generic_generic_ascii;
extern const struct pager_charset pager_charset_pocsag_motorola_advisor_ascii;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_german;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_swiss;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_french;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_swedish;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_portuguese;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_spanish;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_british;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_italian;
extern const struct pager_charset pager_charset_pocsag_generic_generic_latin_danish_norwegian;
extern const struct pager_charset pager_charset_pocsag_generic_generic_ascii_uppercase;
extern const struct pager_charset pager_charset_pocsag_motorola_bravo_alpha_ascii_uppercase;
extern const struct pager_charset pager_charset_pocsag_motorola_advisor_linguist_latin_cyrillic;
extern const struct pager_charset pager_charset_pocsag_motorola_scriptor_lx2_latin_cyrillic;
extern const struct pager_charset pager_charset_pocsag_nec_maxima_latin_cyrillic;
extern const struct pager_charset pager_charset_pocsag_nec_optima_latin_cyrillic;
extern const struct pager_charset pager_charset_pocsag_truly_supervisor_latin_cyrillic;
extern const struct pager_charset pager_charset_pocsag_philips_prg2220_latin_cyrillic;
extern const struct pager_charset pager_charset_pocsag_motorola_advisor_cyrillic;
extern const struct pager_charset pager_charset_pocsag_nec_21a_latin_cyrillic;

/* --- Built-in Golay/GSC codepages (6-bit, 64 codes) --- */

extern const struct pager_charset pager_charset_golay_generic_generic_ascii;

/* --- Built-in FLEX codepages (7-bit, 128 codes) --- */

extern const struct pager_charset pager_charset_flex_generic_generic_ascii;

/* --- Built-in RDS codepages (8-bit, 256 codes) --- */

extern const struct pager_charset pager_charset_rds_generic_generic_annex_e;
extern const struct pager_charset pager_charset_rds_nokia_nkp2_annex_e;

#endif /* PAGER_CHARSET_H */
