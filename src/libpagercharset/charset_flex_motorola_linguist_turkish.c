/*
 * FLEX Turkish codepage — Motorola FLX2 Linguist (Table A-15, "ISO/IEC 8859-9
 * Part 9: Latin Alphabet Number 5").
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
 * Charset:  turkish
 *
 * Transcribed from the rendered manual page (ground truth), NOT generated from
 * the ISO-8859-9 standard. This is a 7-bit (128-code) pager CG-ROM layout:
 * byte = (column << 4) | row. It is NOT 8-bit ISO-8859-9.
 *
 * Column 0  = control codes + Latin-1 symbols (¤ µ ¢ × ÷ £ ° « ¿ » ¡ …).
 * Column 1  = more symbols/diacritics (¬ ¯ ^ _ § ² ³ | ~ · ¥ ± ·) + ETB/ESC/US.
 * Column 2  = ASCII punctuation.
 * Column 3  = digits 0-9 + : ; < = > ?.
 * Columns 4-5 = @ A-Z plus [ \ ] and the Turkish Ş / İ.
 * Columns 6-7 = Latin-1 accented capitals + the Turkish Ğ (0x70).
 *
 * TURKISH-SPECIFIC letters present (all UPPERCASE — no lowercase forms in set):
 *   0x5E Ş  U+015E   0x5F İ  U+0130 (dotted capital I)   0x70 Ğ  U+011E
 * NOTE: lowercase Turkish ğ (U+011F), ı (U+0131), ş (U+015F) are NOT in this
 * display set; this is an all-caps CG-ROM.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* Column 0 (0x00-0x0F): control + Latin-1 symbols */
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
	/* Column 1 (0x10-0x1F): symbols/diacritics + ETB/ESC/US */
	/* 0x10 */ { "¬" },            /* U+00AC not sign */
	/* 0x11 */ { "¯" },            /* U+00AF macron */
	/* 0x12 */ { "^" },            /* U+005E circumflex accent */
	/* 0x13 */ { "_" },            /* U+005F low line */
	/* 0x14 */ { "§" },            /* U+00A7 section sign */
	/* 0x15 */ { "²" },            /* U+00B2 superscript two */
	/* 0x16 */ { "|" },            /* U+007C vertical line (broken bar glyph) */
	/* 0x17 */ { NULL },           /* ETB */
	/* 0x18 */ { "³" },            /* U+00B3 superscript three */
	/* 0x19 */ { "~" },            /* U+007E tilde */
	/* 0x1A */ { "·" },            /* U+00B7 middle dot */
	/* 0x1B */ { NULL },           /* ESC */
	/* 0x1C */ { "¥" },            /* U+00A5 yen sign */
	/* 0x1D */ { "±" },            /* U+00B1 plus-minus sign */
	/* 0x1E */ { "·" },            /* U+00B7 middle dot */
	/* 0x1F */ { NULL },           /* US */
	/* Column 2 (0x20-0x2F): ASCII punctuation */
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
	/* Column 3 (0x30-0x3F): digits + punctuation */
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
	/* Column 4 (0x40-0x4F): @ A-O */
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
	/* Column 5 (0x50-0x5F): P-Z, brackets, Turkish Ş İ */
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
	/* 0x5E */ { "Ş" },            /* U+015E Latin capital S with cedilla (Turkish) */
	/* 0x5F */ { "İ" },            /* U+0130 Latin capital I with dot above (Turkish) */
	/* Column 6 (0x60-0x6F): grave + Latin-1 accented capitals */
	/* 0x60 */ { "`" },            /* U+0060 grave accent */
	/* 0x61 */ { "Á" },            /* U+00C1 */
	/* 0x62 */ { "Â" },            /* U+00C2 */
	/* 0x63 */ { "Ã" },            /* U+00C3 */
	/* 0x64 */ { "Ä" },            /* U+00C4 */
	/* 0x65 */ { "Å" },            /* U+00C5 */
	/* 0x66 */ { "Æ" },            /* U+00C6 */
	/* 0x67 */ { "Ç" },            /* U+00C7 (Turkish C with cedilla) */
	/* 0x68 */ { "È" },            /* U+00C8 */
	/* 0x69 */ { "É" },            /* U+00C9 */
	/* 0x6A */ { "Ê" },            /* U+00CA */
	/* 0x6B */ { "Ë" },            /* U+00CB */
	/* 0x6C */ { "Ì" },            /* U+00CC */
	/* 0x6D */ { "Í" },            /* U+00CD */
	/* 0x6E */ { "Î" },            /* U+00CE */
	/* 0x6F */ { "Ï" },            /* U+00CF */
	/* Column 7 (0x70-0x7F): Turkish Ğ + Latin-1 accented capitals */
	/* 0x70 */ { "Ğ" },            /* U+011E Latin capital G with breve (Turkish) */
	/* 0x71 */ { "À" },            /* U+00C0 */
	/* 0x72 */ { "Ó" },            /* U+00D3 */
	/* 0x73 */ { "Ô" },            /* U+00D4 */
	/* 0x74 */ { "Õ" },            /* U+00D5 */
	/* 0x75 */ { "Ö" },            /* U+00D6 */
	/* 0x76 */ { "Ø" },            /* U+00D8 */
	/* 0x77 */ { "Ú" },            /* U+00DA */
	/* 0x78 */ { "Û" },            /* U+00DB */
	/* 0x79 */ { "Ü" },            /* U+00DC */
	/* 0x7A */ { "Ù" },            /* U+00D9 */
	/* 0x7B */ { "{" },            /* U+007B left curly bracket */
	/* 0x7C */ { "Ò" },            /* U+00D2 */
	/* 0x7D */ { "}" },            /* U+007D right curly bracket */
	/* 0x7E */ { "Ñ" },            /* U+00D1 */
	/* 0x7F */ { " " },            /* SP */
};

const struct pager_charset pager_charset_flex_motorola_linguist_turkish = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "turkish",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
