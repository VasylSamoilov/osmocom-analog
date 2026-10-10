/*
 * FLEX Latin-1 codepage — Motorola FLX2 Linguist (Table A-8, "ISO 8859-1").
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
 * Charset:  iso-8859-1
 *
 * 7-bit (128-code) FLEX display codepage, NAMED after ISO 8859-1 but
 * laid out as the pager's own CG-ROM: byte = (col << 4) | row.
 * Transcribed from the rendered page image (ground truth), not generated
 * from the ISO standard.
 *
 * Column 0 = control + symbols (NUL, ETX, EOT, LF, CR + ¤ µ ¢ × ÷ £ ° « ¿ » ¡).
 * Column 1 = symbols / diacritic marks (¬ ¯ ^ _ § ² | ³ ~ · · ESC ¥ ± · US).
 * Columns 2-3 = ASCII punctuation + digits.
 * Columns 4-5 = Latin A-Z region (with [ \ ] etc.).
 * Columns 6-7 = accented letters (à-style grave/acute/circumflex row, Latin-1).
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* --- Column 0 (0x00-0x0F): control + symbols --- */
	/* 0x00 */ { NULL },           /* NUL */
	/* 0x01 */ { "¤" },            /* U+00A4 currency sign */
	/* 0x02 */ { "µ" },            /* U+00B5 micro sign */
	/* 0x03 */ { NULL },           /* ETX */
	/* 0x04 */ { NULL },           /* EOT */
	/* 0x05 */ { "¢" },            /* U+00A2 cent sign */
	/* 0x06 */ { "×" },            /* U+00D7 multiplication sign */
	/* 0x07 */ { "÷" },            /* U+00F7 division sign */
	/* 0x08 */ { "£" },            /* U+00A3 pound sign */
	/* 0x09 */ { "°" },            /* U+00B0 degree sign */
	/* 0x0A */ { "\n" },           /* LF */
	/* 0x0B */ { "«" },            /* U+00AB left guillemet */
	/* 0x0C */ { "¿" },            /* U+00BF inverted question mark */
	/* 0x0D */ { "\r" },           /* CR */
	/* 0x0E */ { "»" },            /* U+00BB right guillemet */
	/* 0x0F */ { "¡" },            /* U+00A1 inverted exclamation mark */
	/* --- Column 1 (0x10-0x1F): symbols / marks --- */
	/* 0x10 */ { "¬" },            /* U+00AC not sign */
	/* 0x11 */ { "¯" },            /* U+00AF macron */
	/* 0x12 */ { "^" },            /* circumflex accent */
	/* 0x13 */ { "_" },            /* low line (underscore) */
	/* 0x14 */ { "§" },            /* U+00A7 section sign */
	/* 0x15 */ { "²" },            /* U+00B2 superscript two */
	/* 0x16 */ { "|" },            /* vertical bar / broken bar */
	/* 0x17 */ { NULL },           /* ETB */
	/* 0x18 */ { "³" },            /* U+00B3 superscript three */
	/* 0x19 */ { "~" },            /* tilde */
	/* 0x1A */ { "·" },            /* U+00B7 middle dot */
	/* 0x1B */ { NULL },           /* ESC */
	/* 0x1C */ { "¥" },            /* U+00A5 yen sign */
	/* 0x1D */ { "±" },            /* U+00B1 plus-minus sign */
	/* 0x1E */ { "·" },            /* U+00B7 middle dot */
	/* 0x1F */ { NULL },           /* US */
	/* --- Column 2 (0x20-0x2F): punctuation --- */
	/* 0x20 */ { " " },            /* SP */
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
	/* --- Column 3 (0x30-0x3F): digits + punctuation --- */
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
	/* --- Column 4 (0x40-0x4F): @ A-O --- */
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
	/* --- Column 5 (0x50-0x5F): P-Z + brackets --- */
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
	/* 0x5E */ { "Þ" },            /* U+00DE Latin capital thorn */
	/* 0x5F */ { "Ý" },            /* U+00DD Latin capital Y acute */
	/* --- Column 6 (0x60-0x6F): grave + acute/circumflex capitals --- */
	/* 0x60 */ { "`" },            /* grave accent */
	/* 0x61 */ { "Á" },            /* U+00C1 */
	/* 0x62 */ { "Â" },            /* U+00C2 */
	/* 0x63 */ { "Ã" },            /* U+00C3 */
	/* 0x64 */ { "Ä" },            /* U+00C4 */
	/* 0x65 */ { "Å" },            /* U+00C5 */
	/* 0x66 */ { "Æ" },            /* U+00C6 */
	/* 0x67 */ { "Ç" },            /* U+00C7 */
	/* 0x68 */ { "È" },            /* U+00C8 */
	/* 0x69 */ { "É" },            /* U+00C9 */
	/* 0x6A */ { "Ê" },            /* U+00CA */
	/* 0x6B */ { "Ë" },            /* U+00CB */
	/* 0x6C */ { "Ì" },            /* U+00CC */
	/* 0x6D */ { "Í" },            /* U+00CD */
	/* 0x6E */ { "Î" },            /* U+00CE */
	/* 0x6F */ { "Ï" },            /* U+00CF */
	/* --- Column 7 (0x70-0x7F): remaining accented capitals + braces --- */
	/* 0x70 */ { "Ð" },            /* U+00D0 Latin capital eth */
	/* 0x71 */ { "À" },            /* U+00C0 */
	/* 0x72 */ { "Ó" },            /* U+00D3 */
	/* 0x73 */ { "Ô" },            /* U+00D4 */
	/* 0x74 */ { "Õ" },            /* U+00D5 */
	/* 0x75 */ { "Ø" },            /* U+00D8 */
	/* 0x76 */ { "Ú" },            /* U+00DA */
	/* 0x77 */ { "Û" },            /* U+00DB */
	/* 0x78 */ { "Ü" },            /* U+00DC */
	/* 0x79 */ { "Ù" },            /* U+00D9 */
	/* 0x7A */ { "{" },            /* UNCERTAIN — low-res col-7 lower rows */
	/* 0x7B */ { "{" },            /* UNCERTAIN */
	/* 0x7C */ { "}" },            /* UNCERTAIN */
	/* 0x7D */ { "}" },            /* UNCERTAIN */
	/* 0x7E */ { "Ñ" },            /* U+00D1 — UNCERTAIN placement */
	/* 0x7F */ { " " },            /* SP (DEL position shows SP) */
};

const struct pager_charset pager_charset_flex_motorola_linguist_latin1 = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "iso-8859-1",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
