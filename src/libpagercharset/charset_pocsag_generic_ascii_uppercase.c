/*
 * POCSAG uppercase-only ASCII codepage.
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
 * Vendor:   generic
 * Model:    generic
 * Charset:  ascii-uppercase
 *
 * For pagers that only display uppercase Latin letters.
 * 0x20-0x3F: punctuation and digits (standard ASCII).
 * 0x40-0x5F: uppercase A-Z and symbols (standard ASCII).
 * 0x60-0x7E: unmapped (pager has no lowercase glyphs).
 *
 * Lowercase input maps to uppercase via aliases (.upper() behavior).
 * — (U+2014 em dash) maps to - (0x2D).
 * ` (U+0060 backtick) is unmapped (pager does not display it).
 *
 * Pagers: Motorola Bravo Alpha, and other uppercase-only models.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00-0x1F: Control region */
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

	/* 0x20-0x3F: Punctuation and digits */
	/* 0x20 */ { " " },
	/* 0x21 */ { "!" },
	/* 0x22 */ { "\"" },
	/* 0x23 */ { "#" },
	/* 0x24 */ { "$" },
	/* 0x25 */ { "%" },
	/* 0x26 */ { "&" },
	/* 0x27 */ { "'", "`" },                                  /* U+0027 apostrophe + U+0060 backtick (pager shows same glyph) */
	/* 0x28 */ { "(" },
	/* 0x29 */ { ")" },
	/* 0x2A */ { "*" },
	/* 0x2B */ { "+" },
	/* 0x2C */ { "," },
	/* 0x2D */ { "-", "—" },                                  /* U+002D hyphen + U+2014 em-dash */
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

	/* 0x40-0x5F: Uppercase Latin + symbols.
	 * Lowercase aliases for .upper() behavior. */
	/* 0x40 */ { "@" },                                       /* U+0040 */
	/* 0x41 */ { "A", "a" },                                  /* U+0041, U+0061 */
	/* 0x42 */ { "B", "b" },                                  /* U+0042, U+0062 */
	/* 0x43 */ { "C", "c" },                                  /* U+0043, U+0063 */
	/* 0x44 */ { "D", "d" },                                  /* U+0044, U+0064 */
	/* 0x45 */ { "E", "e" },                                  /* U+0045, U+0065 */
	/* 0x46 */ { "F", "f" },                                  /* U+0046, U+0066 */
	/* 0x47 */ { "G", "g" },                                  /* U+0047, U+0067 */
	/* 0x48 */ { "H", "h" },                                  /* U+0048, U+0068 */
	/* 0x49 */ { "I", "i" },                                  /* U+0049, U+0069 */
	/* 0x4A */ { "J", "j" },                                  /* U+004A, U+006A */
	/* 0x4B */ { "K", "k" },                                  /* U+004B, U+006B */
	/* 0x4C */ { "L", "l" },                                  /* U+004C, U+006C */
	/* 0x4D */ { "M", "m" },                                  /* U+004D, U+006D */
	/* 0x4E */ { "N", "n" },                                  /* U+004E, U+006E */
	/* 0x4F */ { "O", "o" },                                  /* U+004F, U+006F */
	/* 0x50 */ { "P", "p" },                                  /* U+0050, U+0070 */
	/* 0x51 */ { "Q", "q" },                                  /* U+0051, U+0071 */
	/* 0x52 */ { "R", "r" },                                  /* U+0052, U+0072 */
	/* 0x53 */ { "S", "s" },                                  /* U+0053, U+0073 */
	/* 0x54 */ { "T", "t" },                                  /* U+0054, U+0074 */
	/* 0x55 */ { "U", "u" },                                  /* U+0055, U+0075 */
	/* 0x56 */ { "V", "v" },                                  /* U+0056, U+0076 */
	/* 0x57 */ { "W", "w" },                                  /* U+0057, U+0077 */
	/* 0x58 */ { "X", "x" },                                  /* U+0058, U+0078 */
	/* 0x59 */ { "Y", "y" },                                  /* U+0059, U+0079 */
	/* 0x5A */ { "Z", "z" },                                  /* U+005A, U+007A */
	/* 0x5B */ { "[" },                                       /* U+005B */
	/* 0x5C */ { "\\" },                                      /* U+005C */
	/* 0x5D */ { "]" },                                       /* U+005D */
	/* 0x5E */ { "^" },                                       /* U+005E */
	/* 0x5F */ { "_" },                                       /* U+005F */

	/* 0x60-0x7F: Lowercase codes — pager displays as uppercase.
	 * Primary = uppercase (what pager shows).
	 * TX: encoder scans from 0x00 upward, so lowercase input "a" matches
	 * the alias at 0x41 before reaching 0x61 — uppercase code is sent.
	 * These entries exist only for RX decode of lowercase codes. */
	/* 0x60 */ { "`" },                                       /* U+0060 — displays as ' on pager */
	/* 0x61 */ { "A" },                                       /* displays as A */
	/* 0x62 */ { "B" },                                       /* displays as B */
	/* 0x63 */ { "C" },                                       /* displays as C */
	/* 0x64 */ { "D" },                                       /* displays as D */
	/* 0x65 */ { "E" },                                       /* displays as E */
	/* 0x66 */ { "F" },                                       /* displays as F */
	/* 0x67 */ { "G" },                                       /* displays as G */
	/* 0x68 */ { "H" },                                       /* displays as H */
	/* 0x69 */ { "I" },                                       /* displays as I */
	/* 0x6A */ { "J" },                                       /* displays as J */
	/* 0x6B */ { "K" },                                       /* displays as K */
	/* 0x6C */ { "L" },                                       /* displays as L */
	/* 0x6D */ { "M" },                                       /* displays as M */
	/* 0x6E */ { "N" },                                       /* displays as N */
	/* 0x6F */ { "O" },                                       /* displays as O */
	/* 0x70 */ { "P" },                                       /* displays as P */
	/* 0x71 */ { "Q" },                                       /* displays as Q */
	/* 0x72 */ { "R" },                                       /* displays as R */
	/* 0x73 */ { "S" },                                       /* displays as S */
	/* 0x74 */ { "T" },                                       /* displays as T */
	/* 0x75 */ { "U" },                                       /* displays as U */
	/* 0x76 */ { "V" },                                       /* displays as V */
	/* 0x77 */ { "W" },                                       /* displays as W */
	/* 0x78 */ { "X" },                                       /* displays as X */
	/* 0x79 */ { "Y" },                                       /* displays as Y */
	/* 0x7A */ { "Z" },                                       /* displays as Z */
	/* 0x7B */ { "{" },                                       /* U+007B */
	/* 0x7C */ { "|" },                                       /* U+007C */
	/* 0x7D */ { "}" },                                       /* U+007D */
	/* 0x7E */ { "~" },                                       /* U+007E */
	/* 0x7F */ { NULL },
};

const struct pager_charset pager_charset_pocsag_generic_generic_ascii_uppercase = {
	.protocol  = "pocsag",
	.vendor    = "generic",
	.model     = "generic",
	.charset   = "ascii-uppercase",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};

const struct pager_charset pager_charset_pocsag_motorola_bravo_alpha_ascii_uppercase = {
	.protocol  = "pocsag",
	.vendor    = "motorola",
	.model     = "bravo-alpha",
	.charset   = "ascii-uppercase",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
