/*
 * FLEX Latin-3 codepage — Motorola FLX2 Linguist
 * (Table A-10, "ISO 8859-3 / Part 3: Latin Alphabet Number 3").
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
 * Charset:  iso-8859-3
 *
 * Transcribed from the rendered manual page at 600 dpi (image is ground truth),
 * NOT generated from the ISO-8859-3 standard. 7-bit (128-code) pager CG-ROM
 * layout: byte = (column << 4) | row. South-European Latin (Maltese/Esperanto:
 * Ĉ Ĝ Ĥ Ĵ Ŝ Ŭ Ċ Ġ Ħ İ Ş). All-caps display set.
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
	/* 0x0C */ { "¿" },            /* U+00BF col 0 row C */
	/* 0x0D */ { "\r" },           /* CR */
	/* 0x0E */ { "@" },            /* col 0 row E */
	/* 0x0F */ { NULL },           /* col 0 row F (blank) */
	/* Column 1 (0x10-0x1F): symbols + ETB/ESC/US */
	/* 0x10 */ { "¬" },            /* U+00AC macron/not — reads overbar */
	/* 0x11 */ { "¯" },            /* U+00AF macron */
	/* 0x12 */ { "^" },            /* U+005E */
	/* 0x13 */ { "_" },            /* U+005F */
	/* 0x14 */ { "ß" },            /* reads s-sharp/section; UNCERTAIN */
	/* 0x15 */ { "{" },            /* U+007B */
	/* 0x16 */ { "|" },            /* U+007C */
	/* 0x17 */ { NULL },           /* ETB */
	/* 0x18 */ { "}" },            /* U+007D */
	/* 0x19 */ { "~" },            /* U+007E */
	/* 0x1A */ { "·" },            /* U+00B7 */
	/* 0x1B */ { NULL },           /* ESC */
	/* 0x1C */ { "¥" },            /* U+00A5 */
	/* 0x1D */ { "±" },            /* U+00B1 */
	/* 0x1E */ { "·" },            /* U+00B7 */
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
	/* Column 3 (0x30-0x3F): digits + Latin-3 letters */
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
	/* 0x3D */ { "Ĥ" },            /* U+0124 H with circumflex (col 3 row D) */
	/* 0x3E */ { ">" },
	/* 0x3F */ { "?" },
	/* Column 4 (0x40-0x4F): Ŝ + A-O */
	/* 0x40 */ { "Ŝ" },            /* U+015C S with circumflex */
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
	/* Column 5 (0x50-0x5F): P-Z + Latin-3 letters */
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
	/* 0x5B */ { "[" },            /* U+005B */
	/* 0x5C */ { "Ż" },            /* U+017B Z with dot above */
	/* 0x5D */ { "]" },            /* U+005D */
	/* 0x5E */ { "Ş" },            /* U+015E S with cedilla */
	/* 0x5F */ { "İ" },            /* U+0130 I with dot above */
	/* Column 6 (0x60-0x6F): grave + Latin-3 capitals */
	/* 0x60 */ { "`" },            /* U+0060 grave */
	/* 0x61 */ { "Á" },            /* U+00C1 */
	/* 0x62 */ { "Â" },            /* U+00C2 */
	/* 0x63 */ { "Ħ" },            /* U+0126 H with stroke */
	/* 0x64 */ { "Ã" },            /* U+00C3 */
	/* 0x65 */ { "Ċ" },            /* U+010A C with dot above */
	/* 0x66 */ { "Ĉ" },            /* U+0108 C with circumflex */
	/* 0x67 */ { "Ç" },            /* U+00C7 */
	/* 0x68 */ { "È" },            /* U+00C8 */
	/* 0x69 */ { "É" },            /* U+00C9 */
	/* 0x6A */ { "Ê" },            /* U+00CA */
	/* 0x6B */ { "Ë" },            /* U+00CB */
	/* 0x6C */ { "Ì" },            /* U+00CC */
	/* 0x6D */ { "Í" },            /* U+00CD */
	/* 0x6E */ { "Î" },            /* U+00CE */
	/* 0x6F */ { "Ï" },            /* U+00CF */
	/* Column 7 (0x70-0x7F): Latin-3 capitals */
	/* 0x70 */ { "Ğ" },            /* U+011E G with breve */
	/* 0x71 */ { "À" },            /* U+00C0 */
	/* 0x72 */ { "Ó" },            /* U+00D3 */
	/* 0x73 */ { "Ô" },            /* U+00D4 */
	/* 0x74 */ { "Ġ" },            /* U+0120 G with dot above */
	/* 0x75 */ { "Ö" },            /* U+00D6 */
	/* 0x76 */ { "Ĝ" },            /* U+011C G with circumflex */
	/* 0x77 */ { "Ú" },            /* U+00DA */
	/* 0x78 */ { "Û" },            /* U+00DB */
	/* 0x79 */ { "Ü" },            /* U+00DC */
	/* 0x7A */ { "Ù" },            /* U+00D9 */
	/* 0x7B */ { "Ŭ" },            /* U+016C U with breve */
	/* 0x7C */ { "Ì" },            /* col 7 row C — reads I-grave; UNCERTAIN */
	/* 0x7D */ { "Ĵ" },            /* U+0134 J with circumflex */
	/* 0x7E */ { "Ñ" },            /* U+00D1 */
	/* 0x7F */ { " " },            /* SP */
};

const struct pager_charset pager_charset_flex_motorola_linguist_latin3 = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "iso-8859-3",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
