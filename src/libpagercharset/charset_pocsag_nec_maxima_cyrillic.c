/*
 * POCSAG NEC Maxima/Compact Cyrillic codepage.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: pocsag
 * Vendor:   nec
 * Model:    maxima-compact
 * Charset:  cyrillic
 *
 * Pagers: NEC Maxima, NEC Compact, OI Electric PB2301.
 *
 * NOTE: This is NOT the same as the Advisor Linguist / NEC 21A (Skyper)
 * codepage. The NEC Maxima uses a completely different Cyrillic mapping
 * where letters are scattered across 0x41-0x77 in a non-alphabetic,
 * non-phonetic order.
 *
 * Layout:
 *   0x20-0x3F: Standard ASCII punctuation and digits.
 *   0x40-0x5F: Latin uppercase A-Z (standard ASCII).
 *   0x60:      unmapped (backtick not available).
 *   0x61-0x77: Cyrillic А-Я (non-sequential, vendor-specific order).
 *   0x7B-0x7E: ASCII symbols {|}~.
 *
 * Case-insensitive: lowercase Latin maps to uppercase.
 * Cyrillic uppercase and lowercase map to the same codes.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00-0x1F: Control region — unmapped */
	/* 0x00 */ { NULL },
	/* 0x01 */ { NULL },
	/* 0x02 */ { NULL },
	/* 0x03 */ { NULL },
	/* 0x04 */ { NULL },
	/* 0x05 */ { NULL },
	/* 0x06 */ { NULL },
	/* 0x07 */ { NULL },
	/* 0x08 */ { NULL },
	/* 0x09 */ { NULL },
	/* 0x0A */ { "\n" },
	/* 0x0B */ { NULL },
	/* 0x0C */ { NULL },
	/* 0x0D */ { "\r" },
	/* 0x0E */ { NULL },
	/* 0x0F */ { NULL },
	/* 0x10 */ { NULL },
	/* 0x11 */ { NULL },
	/* 0x12 */ { NULL },
	/* 0x13 */ { NULL },
	/* 0x14 */ { NULL },
	/* 0x15 */ { NULL },
	/* 0x16 */ { NULL },
	/* 0x17 */ { NULL },
	/* 0x18 */ { NULL },
	/* 0x19 */ { NULL },
	/* 0x1A */ { NULL },
	/* 0x1B */ { NULL },
	/* 0x1C */ { NULL },
	/* 0x1D */ { NULL },
	/* 0x1E */ { NULL },
	/* 0x1F */ { NULL },

	/* 0x20-0x3F: Standard ASCII punctuation and digits */
	/* 0x20 */ { " " },
	/* 0x21 */ { "!" },
	/* 0x22 */ { "\"" },
	/* 0x23 */ { "#" },
	/* 0x24 */ { "$" },
	/* 0x25 */ { "%" },
	/* 0x26 */ { "&" },
	/* 0x27 */ { "'" },
	/* 0x28 */ { "(" },
	/* 0x29 */ { ")" },
	/* 0x2A */ { "*" },
	/* 0x2B */ { "+" },
	/* 0x2C */ { "," },
	/* 0x2D */ { "-" },
	/* 0x2E */ { "." },
	/* 0x2F */ { "/" },
	/* 0x30 */ { "0" },
	/* 0x31 */ { "1" },
	/* 0x32 */ { "2" },
	/* 0x33 */ { "3" },
	/* 0x34 */ { "4" },
	/* 0x35 */ { "5" },
	/* 0x36 */ { "6" },
	/* 0x37 */ { "7" },
	/* 0x38 */ { "8" },
	/* 0x39 */ { "9" },
	/* 0x3A */ { ":" },
	/* 0x3B */ { ";" },
	/* 0x3C */ { "<" },
	/* 0x3D */ { "=" },
	/* 0x3E */ { ">" },
	/* 0x3F */ { "?" },

	/* 0x40-0x5F: Latin uppercase (standard ASCII) */
	/* 0x40 */ { "@" },
	/* 0x41 */ { "A", "a" },
	/* 0x42 */ { "B", "b" },
	/* 0x43 */ { "C", "c" },
	/* 0x44 */ { "D", "d" },
	/* 0x45 */ { "E", "e" },
	/* 0x46 */ { "F", "f" },
	/* 0x47 */ { "G", "g" },
	/* 0x48 */ { "H", "h" },
	/* 0x49 */ { "I", "i" },
	/* 0x4A */ { "J", "j" },
	/* 0x4B */ { "K", "k" },
	/* 0x4C */ { "L", "l" },
	/* 0x4D */ { "M", "m" },
	/* 0x4E */ { "N", "n" },
	/* 0x4F */ { "O", "o" },
	/* 0x50 */ { "P", "p" },
	/* 0x51 */ { "Q", "q" },
	/* 0x52 */ { "R", "r" },
	/* 0x53 */ { "S", "s" },
	/* 0x54 */ { "T", "t" },
	/* 0x55 */ { "U", "u" },
	/* 0x56 */ { "V", "v" },
	/* 0x57 */ { "W", "w" },
	/* 0x58 */ { "X", "x" },
	/* 0x59 */ { "Y", "y" },
	/* 0x5A */ { "Z", "z" },
	/* 0x5B */ { "[" },
	/* 0x5C */ { "\\" },
	/* 0x5D */ { "]" },
	/* 0x5E */ { "^" },
	/* 0x5F */ { "_" },

	/* 0x60-0x7F: Cyrillic А-Я (NEC Maxima vendor-specific order).
	 * Case-insensitive. */
	/* 0x60 */ { NULL },                  /* backtick not available */
	/* 0x61 */ { "А", "а" },              /* U+0410/U+0430 */
	/* 0x62 */ { "Г", "г" },              /* U+0413/U+0433 */
	/* 0x63 */ { NULL },                  /* unmapped */
	/* 0x64 */ { "Д", "д" },              /* U+0414/U+0434 */
	/* 0x65 */ { "Е", "е", "Ё", "ё" },   /* U+0415/U+0435 + U+0401/U+0451 */
	/* 0x66 */ { "Ж", "ж" },              /* U+0416/U+0436 */
	/* 0x67 */ { "З", "з" },              /* U+0417/U+0437 */
	/* 0x68 */ { "И", "и" },              /* U+0418/U+0438 */
	/* 0x69 */ { "Й", "й" },              /* U+0419/U+0439 */
	/* 0x6A */ { "Л", "л" },              /* U+041B/U+043B */
	/* 0x6B */ { "П", "п" },              /* U+041F/U+043F */
	/* 0x6C */ { "У", "у" },              /* U+0423/U+0443 */
	/* 0x6D */ { "Ф", "ф" },              /* U+0424/U+0444 */
	/* 0x6E */ { "Ц", "ц" },              /* U+0426/U+0446 */
	/* 0x6F */ { "Ч", "ч" },              /* U+0427/U+0447 */
	/* 0x70 */ { "Ш", "ш" },              /* U+0428/U+0448 */
	/* 0x71 */ { "Щ", "щ" },              /* U+0429/U+0449 */
	/* 0x72 */ { "Ъ", "ъ" },              /* U+042A/U+044A */
	/* 0x73 */ { "Ы", "ы" },              /* U+042B/U+044B */
	/* 0x74 */ { "Ь", "ь" },              /* U+042C/U+044C */
	/* 0x75 */ { "Э", "э" },              /* U+042D/U+044D */
	/* 0x76 */ { "Ю", "ю" },              /* U+042E/U+044E */
	/* 0x77 */ { "Я", "я" },              /* U+042F/U+044F */
	/* 0x78 */ { NULL },
	/* 0x79 */ { NULL },
	/* 0x7A */ { NULL },
	/* 0x7B */ { "{" },
	/* 0x7C */ { "|" },
	/* 0x7D */ { "}" },
	/* 0x7E */ { "~" },
	/* 0x7F */ { NULL },
};

/*
 * Note on dual-mapped Latin/Cyrillic lookalikes:
 * Cyrillic letters that visually resemble Latin letters (Б→B, В→B,
 * Е→E, К→K, М→M, Н→H, О→O, Р→P, С→C, Т→T, Х→X) are mapped to
 * their Latin equivalents in the 0x41-0x5A range. The pager displays
 * the Latin glyph which visually matches the Cyrillic letter.
 */

const struct pager_charset pager_charset_pocsag_nec_maxima_compact_cyrillic = {
	.protocol  = "pocsag",
	.vendor    = "nec",
	.model     = "maxima-compact",
	.charset   = "cyrillic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
