/*
 * FLEX Latin-4 codepage — Motorola FLX2 Linguist
 * (Table A-11, "ISO 8859-4 / Part 4: Latin Alphabet Number 4").
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
 * Charset:  iso-8859-4
 *
 * Transcribed from the rendered manual page at 600 dpi (image is ground truth),
 * NOT generated from the ISO-8859-4 standard. 7-bit (128-code) pager CG-ROM
 * layout: byte = (column << 4) | row. North-European / Baltic Latin
 * (Estonian/Latvian/Lithuanian). All-caps display set.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* Column 0 (0x00-0x0F): control + Latin-1 symbols */
	/* 0x00 */ { NULL },           /* NUL */
	/* 0x01 */ { "¤" },            /* U+00A4 */
	/* 0x02 */ { "µ" },            /* U+00B5 */
	/* 0x03 */ { NULL },           /* ETX */
	/* 0x04 */ { NULL },           /* EOT */
	/* 0x05 */ { "¢" },            /* U+00A2 */
	/* 0x06 */ { "×" },            /* U+00D7 */
	/* 0x07 */ { "÷" },            /* U+00F7 */
	/* 0x08 */ { "£" },            /* U+00A3 */
	/* 0x09 */ { "°" },            /* U+00B0 */
	/* 0x0A */ { "\n" },           /* LF */
	/* 0x0B */ { "=" },            /* col 0 row B */
	/* 0x0C */ { ">" },            /* col 0 row C */
	/* 0x0D */ { "\r" },           /* CR */
	/* 0x0E */ { "@" },            /* col 0 row E */
	/* 0x0F */ { "[" },            /* col 0 row F */
	/* Column 1 (0x10-0x1F): symbols + ETB/ESC/US */
	/* 0x10 */ { "\\" },           /* U+005C */
	/* 0x11 */ { "]" },            /* U+005D */
	/* 0x12 */ { "^" },            /* U+005E */
	/* 0x13 */ { "_" },            /* U+005F */
	/* 0x14 */ { "§" },            /* U+00A7 */
	/* 0x15 */ { "{" },            /* U+007B */
	/* 0x16 */ { "|" },            /* U+007C */
	/* 0x17 */ { NULL },           /* ETB */
	/* 0x18 */ { "}" },            /* U+007D */
	/* 0x19 */ { "~" },            /* U+007E */
	/* 0x1A */ { "·" },            /* U+00B7 */
	/* 0x1B */ { NULL },           /* ESC */
	/* 0x1C */ { "¥" },            /* U+00A5 */
	/* 0x1D */ { "±" },            /* U+00B1 */
	/* 0x1E */ { ";" },            /* col 1 row E */
	/* 0x1F */ { NULL },           /* US */
	/* Column 2 (0x20-0x2F): ASCII punctuation */
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
	/* Column 3 (0x30-0x3F): digits + Latin-4 letters */
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
	/* 0x3B */ { "Ģ" },            /* U+0122 G with cedilla (col 3 row B) */
	/* 0x3C */ { "Ū" },            /* U+016A U with macron (col 3 row C) */
	/* 0x3D */ { "Ų" },            /* U+0172 U with ogonek (col 3 row D) */
	/* 0x3E */ { "Ũ" },            /* U+0168 U with tilde (col 3 row E) */
	/* 0x3F */ { "?" },
	/* Column 4 (0x40-0x4F): Ą + A-O */
	/* 0x40 */ { "Ą" },            /* U+0104 A with ogonek */
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
	/* Column 5 (0x50-0x5F): P-Z + Latin-4 letters */
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
	/* 0x5B */ { "Ŧ" },            /* U+0166 T with stroke (col 5 row B) */
	/* 0x5C */ { "Ļ" },            /* U+013B L with cedilla (col 5 row C) */
	/* 0x5D */ { "Ņ" },            /* U+0145 N with cedilla (col 5 row D) */
	/* 0x5E */ { "Đ" },            /* U+0110 D with stroke (col 5 row E) — reads D-bar/eng */
	/* 0x5F */ { "Ŗ" },            /* U+0156 R with cedilla (col 5 row F) */
	/* Column 6 (0x60-0x6F): grave + Latin-4 capitals */
	/* 0x60 */ { "`" },            /* U+0060 grave */
	/* 0x61 */ { "Á" },            /* U+00C1 */
	/* 0x62 */ { "Â" },            /* U+00C2 */
	/* 0x63 */ { "Ã" },            /* U+00C3 */
	/* 0x64 */ { "Ä" },            /* U+00C4 */
	/* 0x65 */ { "Å" },            /* U+00C5 */
	/* 0x66 */ { "Æ" },            /* U+00C6 */
	/* 0x67 */ { "Į" },            /* U+012E I with ogonek */
	/* 0x68 */ { "Č" },            /* U+010C C with caron */
	/* 0x69 */ { "É" },            /* U+00C9 */
	/* 0x6A */ { "Ę" },            /* U+0118 E with ogonek */
	/* 0x6B */ { "Ë" },            /* U+00CB */
	/* 0x6C */ { "Ė" },            /* U+0116 E with dot above */
	/* 0x6D */ { "Í" },            /* U+00CD */
	/* 0x6E */ { "Î" },            /* U+00CE */
	/* 0x6F */ { "Ī" },            /* U+012A I with macron */
	/* Column 7 (0x70-0x7F): Latin-4 capitals + SP */
	/* 0x70 */ { "Đ" },            /* U+0110 D with stroke */
	/* 0x71 */ { "Ā" },            /* U+0100 A with macron */
	/* 0x72 */ { "Ķ" },            /* U+0136 K with cedilla */
	/* 0x73 */ { "Ô" },            /* U+00D4 */
	/* 0x74 */ { "Õ" },            /* U+00D5 */
	/* 0x75 */ { "Ö" },            /* U+00D6 */
	/* 0x76 */ { "Ø" },            /* U+00D8 */
	/* 0x77 */ { "Ú" },            /* U+00DA */
	/* 0x78 */ { "Û" },            /* U+00DB */
	/* 0x79 */ { "Ü" },            /* U+00DC */
	/* 0x7A */ { "Ž" },            /* U+017D Z with caron */
	/* 0x7B */ { "Ē" },            /* U+0112 E with macron */
	/* 0x7C */ { "Õ" },            /* U+00D5 — col 7 row C reads O-tilde/O-macron; UNCERTAIN */
	/* 0x7D */ { "Ĩ" },            /* U+0128 I with tilde */
	/* 0x7E */ { "Š" },            /* U+0160 S with caron */
	/* 0x7F */ { " " },            /* SP */
};

const struct pager_charset pager_charset_flex_motorola_linguist_latin4 = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "iso-8859-4",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
