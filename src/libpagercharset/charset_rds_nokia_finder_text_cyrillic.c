/*
 * RDS Nokia Finder Text pager Cyrillic character set.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: rds
 * Vendor:   nokia
 * Model:    finder-text
 * Charset:  cyrillic
 *
 * 8-bit character set (256 codes) of the Nokia Finder Text pager, received
 * through RDS alphanumeric paging (EN 50067 Annex M).
 *
 * Layout:
 *   0x10-0x1F: CP437 graphic symbols (arrows, triangles, pilcrow, section).
 *   0x20-0x7E: ASCII.
 *   0x80-0x9F: Cyrillic capitals A-Ya (CP866 order).
 *   0xA0-0xAF: Cyrillic small a-pe (CP866 order).
 *   0xE0-0xEF: Cyrillic small er-ya (CP866 order).
 *   0xF0-0xFF: CP437 math symbols (triple bar ... superscript 2, block).
 *
 * Codes that show a blank or a glyph without a Unicode equivalent are
 * unmapped: 0x00, 0x02-0x07, 0x0C, 0x0E-0x0F, 0x7F, 0xB0-0xDF, 0xFF.
 * 0x01 shows an enter-key glyph. 0x02-0x07, 0x0C and 0x0E-0x0F show display
 * user-interface symbols (boxes with marks inside) that have no Unicode
 * equivalent. 0x08-0x0B show small triangles (up, down, right, left).
 * 0x0D has no glyph: the pager cuts the message at it and drops the rest.
 * It is mapped to CR.
 * The pager has no Ukrainian-only letters and no yo.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[256] = {
	/* 0x00-0x0F: control region */
	/* 0x00 */ { NULL },
	/* 0x01 */ { "↵" },   /* U+21B5 */
	/* 0x02 */ { NULL },
	/* 0x03 */ { NULL },
	/* 0x04 */ { NULL },
	/* 0x05 */ { NULL },
	/* 0x06 */ { NULL },
	/* 0x07 */ { NULL },
	/* 0x08 */ { "▴" },   /* U+25B4 */
	/* 0x09 */ { "▾" },   /* U+25BE */
	/* 0x0A */ { "▸" },   /* U+25B8 */
	/* 0x0B */ { "◂" },   /* U+25C2 */
	/* 0x0C */ { NULL },
	/* 0x0D */ { "\r" },
	/* 0x0E */ { NULL },
	/* 0x0F */ { NULL },

	/* 0x10-0x1F: CP437 graphic symbols */
	/* 0x10 */ { "►" },   /* U+25BA */
	/* 0x11 */ { "◄" },   /* U+25C4 */
	/* 0x12 */ { "↕" },   /* U+2195 */
	/* 0x13 */ { "‼" },   /* U+203C */
	/* 0x14 */ { "¶" },   /* U+00B6 */
	/* 0x15 */ { "§" },   /* U+00A7 */
	/* 0x16 */ { "▬" },   /* U+25AC */
	/* 0x17 */ { "↨" },   /* U+21A8 */
	/* 0x18 */ { "↑" },   /* U+2191 */
	/* 0x19 */ { "↓" },   /* U+2193 */
	/* 0x1A */ { "→" },   /* U+2192 */
	/* 0x1B */ { "←" },   /* U+2190 */
	/* 0x1C */ { "∟" },   /* U+221F */
	/* 0x1D */ { "↔" },   /* U+2194 */
	/* 0x1E */ { "▲" },   /* U+25B2 */
	/* 0x1F */ { "▼" },   /* U+25BC */

	/* 0x20-0x3F: ASCII punctuation and digits */
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

	/* 0x40-0x5F: ASCII uppercase */
	/* 0x40 */ { "@" },
	/* 0x41 */ { "A" },
	/* 0x42 */ { "B" },
	/* 0x43 */ { "C" },
	/* 0x44 */ { "D" },
	/* 0x45 */ { "E" },
	/* 0x46 */ { "F" },
	/* 0x47 */ { "G" },
	/* 0x48 */ { "H" },
	/* 0x49 */ { "I" },
	/* 0x4A */ { "J" },
	/* 0x4B */ { "K" },
	/* 0x4C */ { "L" },
	/* 0x4D */ { "M" },
	/* 0x4E */ { "N" },
	/* 0x4F */ { "O" },
	/* 0x50 */ { "P" },
	/* 0x51 */ { "Q" },
	/* 0x52 */ { "R" },
	/* 0x53 */ { "S" },
	/* 0x54 */ { "T" },
	/* 0x55 */ { "U" },
	/* 0x56 */ { "V" },
	/* 0x57 */ { "W" },
	/* 0x58 */ { "X" },
	/* 0x59 */ { "Y" },
	/* 0x5A */ { "Z" },
	/* 0x5B */ { "[" },
	/* 0x5C */ { "\\" },
	/* 0x5D */ { "]" },
	/* 0x5E */ { "^" },
	/* 0x5F */ { "_" },

	/* 0x60-0x7F: ASCII lowercase */
	/* 0x60 */ { "`" },
	/* 0x61 */ { "a" },
	/* 0x62 */ { "b" },
	/* 0x63 */ { "c" },
	/* 0x64 */ { "d" },
	/* 0x65 */ { "e" },
	/* 0x66 */ { "f" },
	/* 0x67 */ { "g" },
	/* 0x68 */ { "h" },
	/* 0x69 */ { "i" },
	/* 0x6A */ { "j" },
	/* 0x6B */ { "k" },
	/* 0x6C */ { "l" },
	/* 0x6D */ { "m" },
	/* 0x6E */ { "n" },
	/* 0x6F */ { "o" },
	/* 0x70 */ { "p" },
	/* 0x71 */ { "q" },
	/* 0x72 */ { "r" },
	/* 0x73 */ { "s" },
	/* 0x74 */ { "t" },
	/* 0x75 */ { "u" },
	/* 0x76 */ { "v" },
	/* 0x77 */ { "w" },
	/* 0x78 */ { "x" },
	/* 0x79 */ { "y" },
	/* 0x7A */ { "z" },
	/* 0x7B */ { "{" },
	/* 0x7C */ { "|" },
	/* 0x7D */ { "}" },
	/* 0x7E */ { "~" },
	/* 0x7F */ { NULL },

	/* 0x80-0x9F: Cyrillic capitals */
	/* 0x80 */ { "А" },   /* U+0410 */
	/* 0x81 */ { "Б" },   /* U+0411 */
	/* 0x82 */ { "В" },   /* U+0412 */
	/* 0x83 */ { "Г" },   /* U+0413 */
	/* 0x84 */ { "Д" },   /* U+0414 */
	/* 0x85 */ { "Е" },   /* U+0415 */
	/* 0x86 */ { "Ж" },   /* U+0416 */
	/* 0x87 */ { "З" },   /* U+0417 */
	/* 0x88 */ { "И" },   /* U+0418 */
	/* 0x89 */ { "Й" },   /* U+0419 */
	/* 0x8A */ { "К" },   /* U+041A */
	/* 0x8B */ { "Л" },   /* U+041B */
	/* 0x8C */ { "М" },   /* U+041C */
	/* 0x8D */ { "Н" },   /* U+041D */
	/* 0x8E */ { "О" },   /* U+041E */
	/* 0x8F */ { "П" },   /* U+041F */
	/* 0x90 */ { "Р" },   /* U+0420 */
	/* 0x91 */ { "С" },   /* U+0421 */
	/* 0x92 */ { "Т" },   /* U+0422 */
	/* 0x93 */ { "У" },   /* U+0423 */
	/* 0x94 */ { "Ф" },   /* U+0424 */
	/* 0x95 */ { "Х" },   /* U+0425 */
	/* 0x96 */ { "Ц" },   /* U+0426 */
	/* 0x97 */ { "Ч" },   /* U+0427 */
	/* 0x98 */ { "Ш" },   /* U+0428 */
	/* 0x99 */ { "Щ" },   /* U+0429 */
	/* 0x9A */ { "Ъ" },   /* U+042A */
	/* 0x9B */ { "Ы" },   /* U+042B */
	/* 0x9C */ { "Ь" },   /* U+042C */
	/* 0x9D */ { "Э" },   /* U+042D */
	/* 0x9E */ { "Ю" },   /* U+042E */
	/* 0x9F */ { "Я" },   /* U+042F */

	/* 0xA0-0xAF: Cyrillic small a-pe */
	/* 0xA0 */ { "а" },   /* U+0430 */
	/* 0xA1 */ { "б" },   /* U+0431 */
	/* 0xA2 */ { "в" },   /* U+0432 */
	/* 0xA3 */ { "г" },   /* U+0433 */
	/* 0xA4 */ { "д" },   /* U+0434 */
	/* 0xA5 */ { "е" },   /* U+0435 */
	/* 0xA6 */ { "ж" },   /* U+0436 */
	/* 0xA7 */ { "з" },   /* U+0437 */
	/* 0xA8 */ { "и" },   /* U+0438 */
	/* 0xA9 */ { "й" },   /* U+0439 */
	/* 0xAA */ { "к" },   /* U+043A */
	/* 0xAB */ { "л" },   /* U+043B */
	/* 0xAC */ { "м" },   /* U+043C */
	/* 0xAD */ { "н" },   /* U+043D */
	/* 0xAE */ { "о" },   /* U+043E */
	/* 0xAF */ { "п" },   /* U+043F */

	/* 0xB0-0xDF: blank on the pager */
	/* 0xB0 */ { NULL },
	/* 0xB1 */ { NULL },
	/* 0xB2 */ { NULL },
	/* 0xB3 */ { NULL },
	/* 0xB4 */ { NULL },
	/* 0xB5 */ { NULL },
	/* 0xB6 */ { NULL },
	/* 0xB7 */ { NULL },
	/* 0xB8 */ { NULL },
	/* 0xB9 */ { NULL },
	/* 0xBA */ { NULL },
	/* 0xBB */ { NULL },
	/* 0xBC */ { NULL },
	/* 0xBD */ { NULL },
	/* 0xBE */ { NULL },
	/* 0xBF */ { NULL },
	/* 0xC0 */ { NULL },
	/* 0xC1 */ { NULL },
	/* 0xC2 */ { NULL },
	/* 0xC3 */ { NULL },
	/* 0xC4 */ { NULL },
	/* 0xC5 */ { NULL },
	/* 0xC6 */ { NULL },
	/* 0xC7 */ { NULL },
	/* 0xC8 */ { NULL },
	/* 0xC9 */ { NULL },
	/* 0xCA */ { NULL },
	/* 0xCB */ { NULL },
	/* 0xCC */ { NULL },
	/* 0xCD */ { NULL },
	/* 0xCE */ { NULL },
	/* 0xCF */ { NULL },
	/* 0xD0 */ { NULL },	/* small hex digit glyphs 0-F, no Unicode equivalent */
	/* 0xD1 */ { NULL },
	/* 0xD2 */ { NULL },
	/* 0xD3 */ { NULL },
	/* 0xD4 */ { NULL },
	/* 0xD5 */ { NULL },
	/* 0xD6 */ { NULL },
	/* 0xD7 */ { NULL },
	/* 0xD8 */ { NULL },
	/* 0xD9 */ { NULL },
	/* 0xDA */ { NULL },
	/* 0xDB */ { NULL },
	/* 0xDC */ { NULL },
	/* 0xDD */ { NULL },
	/* 0xDE */ { NULL },
	/* 0xDF */ { NULL },

	/* 0xE0-0xEF: Cyrillic small er-ya */
	/* 0xE0 */ { "р" },   /* U+0440 */
	/* 0xE1 */ { "с" },   /* U+0441 */
	/* 0xE2 */ { "т" },   /* U+0442 */
	/* 0xE3 */ { "у" },   /* U+0443 */
	/* 0xE4 */ { "ф" },   /* U+0444 */
	/* 0xE5 */ { "х" },   /* U+0445 */
	/* 0xE6 */ { "ц" },   /* U+0446 */
	/* 0xE7 */ { "ч" },   /* U+0447 */
	/* 0xE8 */ { "ш" },   /* U+0448 */
	/* 0xE9 */ { "щ" },   /* U+0449 */
	/* 0xEA */ { "ъ" },   /* U+044A */
	/* 0xEB */ { "ы" },   /* U+044B */
	/* 0xEC */ { "ь" },   /* U+044C */
	/* 0xED */ { "э" },   /* U+044D */
	/* 0xEE */ { "ю" },   /* U+044E */
	/* 0xEF */ { "я" },   /* U+044F */

	/* 0xF0-0xFF: CP437 math symbols */
	/* 0xF0 */ { "≡" },   /* U+2261 */
	/* 0xF1 */ { "±" },   /* U+00B1 */
	/* 0xF2 */ { "≥" },   /* U+2265 */
	/* 0xF3 */ { "≤" },   /* U+2264 */
	/* 0xF4 */ { "⌠" },   /* U+2320 */
	/* 0xF5 */ { "⌡" },   /* U+2321 */
	/* 0xF6 */ { "÷" },   /* U+00F7 */
	/* 0xF7 */ { "≈" },   /* U+2248 */
	/* 0xF8 */ { "°" },   /* U+00B0 */
	/* 0xF9 */ { "∙" },   /* U+2219 */
	/* 0xFA */ { "·" },   /* U+00B7 */
	/* 0xFB */ { "√" },   /* U+221A */
	/* 0xFC */ { "ⁿ" },   /* U+207F */
	/* 0xFD */ { "²" },   /* U+00B2 */
	/* 0xFE */ { "■" },   /* U+25A0 */
	/* 0xFF */ { NULL },
};

const struct pager_charset pager_charset_rds_nokia_finder_text_cyrillic = {
	.protocol  = "rds",
	.vendor    = "nokia",
	.model     = "finder-text",
	.charset   = "cyrillic",
	.code_bits = 8,
	.num_codes = 256,
	.entries   = entries,
};
