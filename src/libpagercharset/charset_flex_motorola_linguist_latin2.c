/*
 * FLEX Latin-2 codepage — Motorola FLX2 Linguist
 * (Table A-9, "ISO 8859-2 / Part 2: Latin Alphabet Number 2").
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
 * Charset:  iso-8859-2
 *
 * Transcribed from the rendered manual page at 600 dpi (image is ground truth),
 * NOT generated from the ISO-8859-2 standard. 7-bit (128-code) pager CG-ROM
 * layout: byte = (column << 4) | row. Central-European Latin (Czech/Slovak/
 * Polish/Hungarian). All-caps display set (no lowercase accented forms).
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
	/* Column 3 (0x30-0x3F): digits + Latin-2 letters */
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
	/* 0x3B */ { "Ł" },            /* U+0141 L with stroke */
	/* 0x3C */ { "Ľ" },            /* U+013D L with caron */
	/* 0x3D */ { "Ť" },            /* U+0164 T with caron */
	/* 0x3E */ { "Ň" },            /* U+0147 N with caron */
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
	/* Column 5 (0x50-0x5F): P-Z + Latin-2 letters */
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
	/* 0x5B */ { "Ĵ" },            /* col 5 row B — J-like with caron tail; UNCERTAIN (reads J-caron) */
	/* 0x5C */ { "Ż" },            /* U+017B Z with dot above */
	/* 0x5D */ { "Ů" },            /* U+016E U with ring above */
	/* 0x5E */ { "Ş" },            /* U+015E S with cedilla */
	/* 0x5F */ { "Ý" },            /* U+00DD Y with acute */
	/* Column 6 (0x60-0x6F): grave + Latin-2 capitals */
	/* 0x60 */ { "`" },            /* U+0060 grave */
	/* 0x61 */ { "Á" },            /* U+00C1 */
	/* 0x62 */ { "Â" },            /* U+00C2 */
	/* 0x63 */ { "Ă" },            /* U+0102 A with breve */
	/* 0x64 */ { "Ä" },            /* U+00C4 */
	/* 0x65 */ { "Ĺ" },            /* U+0139 L with acute */
	/* 0x66 */ { "Ć" },            /* U+0106 C with acute */
	/* 0x67 */ { "Ç" },            /* U+00C7 */
	/* 0x68 */ { "Č" },            /* U+010C C with caron */
	/* 0x69 */ { "É" },            /* U+00C9 */
	/* 0x6A */ { "Ę" },            /* U+0118 E with ogonek */
	/* 0x6B */ { "Ë" },            /* U+00CB */
	/* 0x6C */ { "Ě" },            /* U+011A E with caron */
	/* 0x6D */ { "Í" },            /* U+00CD */
	/* 0x6E */ { "Î" },            /* U+00CE */
	/* 0x6F */ { "Ď" },            /* U+010E D with caron */
	/* Column 7 (0x70-0x7F): Latin-2 capitals + SP */
	/* 0x70 */ { "Đ" },            /* U+0110 D with stroke */
	/* 0x71 */ { "Ŕ" },            /* U+0154 R with acute */
	/* 0x72 */ { "Ó" },            /* U+00D3 */
	/* 0x73 */ { "Ô" },            /* U+00D4 */
	/* 0x74 */ { "Ő" },            /* U+0150 O with double acute */
	/* 0x75 */ { "Ö" },            /* U+00D6 */
	/* 0x76 */ { "Ř" },            /* U+0158 R with caron */
	/* 0x77 */ { "Ú" },            /* U+00DA */
	/* 0x78 */ { "Ű" },            /* U+0170 U with double acute */
	/* 0x79 */ { "Ü" },            /* U+00DC */
	/* 0x7A */ { "Ž" },            /* U+017D Z with caron */
	/* 0x7B */ { "Ź" },            /* U+0179 Z with acute */
	/* 0x7C */ { "Ń" },            /* U+0143 N with acute */
	/* 0x7D */ { "Ś" },            /* U+015A S with acute */
	/* 0x7E */ { "Š" },            /* U+0160 S with caron */
	/* 0x7F */ { " " },            /* SP */
};

const struct pager_charset pager_charset_flex_motorola_linguist_latin2 = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "iso-8859-2",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
