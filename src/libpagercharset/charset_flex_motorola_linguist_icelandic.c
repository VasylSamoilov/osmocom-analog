/*
 * FLEX Icelandic codepage — Motorola FLX2 Linguist (Table A-3).
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: flex
 * Vendor:   motorola
 * Model:    linguist
 * Charset:  icelandic
 *
 * Standard 7-bit ASCII with Icelandic characters replacing some
 * control character positions in columns 0-1 (0x02-0x0F, 0x12-0x1F).
 * Columns 2-7 are unmodified ASCII.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00 */ { NULL },  /* NUL */
	/* 0x01 */ { NULL },  /* SOH */
	/* 0x02 */ { "Á" },           /* U+00C1 — replaces STX */
	/* 0x03 */ { "á" },           /* U+00E1 — replaces ETX */
	/* 0x04 */ { "Í" },           /* U+00CD — replaces EOT */
	/* 0x05 */ { "í" },           /* U+00ED — replaces ENQ */
	/* 0x06 */ { "Ö" },           /* U+00D6 — replaces ACK */
	/* 0x07 */ { "ö" },           /* U+00F6 — replaces BEL */
	/* 0x08 */ { NULL },  /* BS  */
	/* 0x09 */ { "ð" },           /* U+00F0 — replaces HT */
	/* 0x0A */ { "\n" },  /* LF  */
	/* 0x0B */ { "Ý" },           /* U+00DD — replaces VT */
	/* 0x0C */ { "ý" },           /* U+00FD — replaces FF */
	/* 0x0D */ { "\r" },  /* CR  */
	/* 0x0E */ { "Ú" },           /* U+00DA — replaces SO */
	/* 0x0F */ { "ú" },           /* U+00FA — replaces SI */
	/* 0x10 */ { NULL },  /* DLE */
	/* 0x11 */ { NULL },  /* DC1 */
	/* 0x12 */ { "Á" },           /* U+00C1 — replaces DC2 */
	/* 0x13 */ { "á" },           /* U+00E1 — replaces DC3 */
	/* 0x14 */ { "Í" },           /* U+00CD — replaces DC4 */
	/* 0x15 */ { "í" },           /* U+00ED — replaces NAK */
	/* 0x16 */ { "Ó" },           /* U+00D3 — replaces SYN */
	/* 0x17 */ { "ó" },           /* U+00F3 — replaces ETB */
	/* 0x18 */ { "Þ" },           /* U+00DE — replaces CAN */
	/* 0x19 */ { "þ" },           /* U+00FE — replaces EM */
	/* 0x1A */ { "Ð" },           /* U+00D0 — replaces SUB */
	/* 0x1B */ { NULL },  /* ESC */
	/* 0x1C */ { "Æ" },           /* U+00C6 — replaces FS */
	/* 0x1D */ { "æ" },           /* U+00E6 — replaces GS */
	/* 0x1E */ { "É" },           /* U+00C9 — replaces RS */
	/* 0x1F */ { "é" },           /* U+00E9 — replaces US */
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
	/* 0x7F */ { NULL },  /* DEL */
};

const struct pager_charset pager_charset_flex_motorola_linguist_icelandic = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "icelandic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
