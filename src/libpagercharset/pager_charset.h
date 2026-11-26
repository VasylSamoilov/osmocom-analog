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
 *   protocol - paging protocol ("pocsag", "flex", "golay", "ermes", "rds")
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

/* Find the codepage of a protocol by its "vendor-model-charset" key (not case
 * sensitive).  Returns NULL if the protocol has no such codepage. */
const struct pager_charset *pager_charset_lookup(const char *protocol, const char *key);

/* Print the "vendor-model-charset" keys of the codepages of a protocol. */
void pager_charset_list_protocol(const char *protocol);

/* Print the table of a codepage: one line a code, its glyph and the aliases
 * accepted on TX; a code without a glyph is shown as <0xNN>. */
void pager_charset_print(const struct pager_charset *cs);

/* TX: the codes of a UTF-8 text of length bytes, a glyph or an alias for each
 * character, or <0xNN> (a code of the codepage); a control byte (0x00-0x1F,
 * 0x7F) of a 7 or 8-bit codepage is the code of its glyph (LF, CR), else its
 * own code.  Returns the number of codes, or -1 if a character has no code
 * (*bad, if given, points to it) or out is too small. */
int pager_charset_encode_text(const struct pager_charset *cs, const char *utf8, int length,
			      uint8_t *out, int out_size, const char **bad);

/* RX: the UTF-8 text of codes, <0xNN> for a code without a glyph.  The text
 * is cut to out_size - 1 bytes and always terminated.  Returns its length. */
int pager_charset_decode_text(const struct pager_charset *cs, const uint8_t *codes,
			      int length, char *out, int out_size);

/* --- FLEX Japanese Text Service (STD-43A 1.3.1): Shift JIS in HEX/binary --- */

/* The hex digits of a UTF-8 text of length bytes in Shift JIS characters of
 * blocking bits (8 or 16), the digits of a character the lowest first.
 * Returns the number of digits, or -1 and *error (if given) if a character
 * has no Shift JIS code (8 bits: no 1-byte code) or hex is too small. */
int pager_kanji_encode(const char *utf8, int length, int blocking, char *hex, int hex_size,
		       const char **error);

/* The UTF-8 text of the hex digits of Shift JIS characters of blocking bits
 * (8 or 16); a character with a lost digit is '?', a code without a
 * character <0xNN>/<0xNNNN>.  Right to left (D = 1): the first character is
 * the last of the text.  Returns the length of the text. */
int pager_kanji_decode(const char *hex, int blocking, int right_to_left, char *out, int out_size);

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
extern const struct pager_charset pager_charset_pocsag_nec_maxima_compact_cyrillic;
extern const struct pager_charset pager_charset_pocsag_swissphone_de506_ascii;
extern const struct pager_charset pager_charset_pocsag_swissphone_de506_cyrillic;
extern const struct pager_charset pager_charset_pocsag_partner_generic_cyrillic;
extern const struct pager_charset pager_charset_pocsag_multitone_generic_ascii;
extern const struct pager_charset pager_charset_pocsag_nec_skyper_ascii_rot1;

/* --- Built-in Golay/GSC codepages (6-bit, 64 codes) --- */

extern const struct pager_charset pager_charset_golay_generic_generic_ascii;
extern const struct pager_charset pager_charset_ermes_generic_generic_standard;
extern const struct pager_charset pager_charset_ermes_generic_generic_numeric;

/* --- Built-in FLEX codepages (7-bit, 128 codes) --- */

extern const struct pager_charset pager_charset_flex_generic_generic_ascii;
extern const struct pager_charset pager_charset_flex_motorola_linguist_cyrillic;
extern const struct pager_charset pager_charset_flex_motorola_linguist_icelandic;
extern const struct pager_charset pager_charset_flex_motorola_linguist_polish;
extern const struct pager_charset pager_charset_flex_motorola_linguist_vietnamese;
extern const struct pager_charset pager_charset_flex_motorola_linguist_arabic;
extern const struct pager_charset pager_charset_flex_motorola_linguist_arabic_english;
extern const struct pager_charset pager_charset_flex_motorola_linguist_turkish;
extern const struct pager_charset pager_charset_flex_motorola_linguist_hebrew;
extern const struct pager_charset pager_charset_flex_motorola_linguist_greek;
extern const struct pager_charset pager_charset_flex_motorola_linguist_latin1;
extern const struct pager_charset pager_charset_flex_motorola_linguist_latin2;
extern const struct pager_charset pager_charset_flex_motorola_linguist_latin3;
extern const struct pager_charset pager_charset_flex_motorola_linguist_latin4;

/* --- Built-in RDS codepages (8-bit, 256 codes) --- */

extern const struct pager_charset pager_charset_rds_generic_generic_annex_e;
extern const struct pager_charset pager_charset_rds_nokia_nkp2_annex_e;
extern const struct pager_charset pager_charset_rds_nokia_finder_text_cyrillic;

/* --- Built-in NMT SMS codepages (7-bit GSM, 8-bit NMT sets) --- */

extern const struct pager_charset pager_charset_nmt_generic_generic_gsm;
extern const struct pager_charset pager_charset_nmt_generic_generic_cyrillic;
extern const struct pager_charset pager_charset_nmt_nokia_generic_cyrillic;
extern const struct pager_charset pager_charset_nmt_generic_generic_baltic;
extern const struct pager_charset pager_charset_nmt_generic_generic_central;
extern const struct pager_charset pager_charset_nmt_generic_generic_western;
extern const struct pager_charset pager_charset_nmt_generic_generic_thai;

#endif /* PAGER_CHARSET_H */
