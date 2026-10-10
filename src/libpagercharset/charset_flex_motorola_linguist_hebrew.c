/*
 * FLEX Hebrew codepage — Motorola FLX2 Linguist (Table A-14, ISO 8859-8).
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
 * Charset:  hebrew
 *
 * Transcribed from the rendered manual page (image is ground truth), NOT
 * generated from the Unicode ISO-8859-8 standard: this is the pager's own
 * 7-bit CG-ROM layout.  byte = (column << 4) | row.
 *
 * Layout:
 *   Column 0-1 - control region + symbol glyphs (shared across the A-12..A-15
 *                family on this page: ¤ µ ¢ × ÷ £ ° « ¿ » ¡ ¬ ½ ¦ § ² ³ © ¥ ± ·)
 *   Column 2-3 - ASCII punctuation + digits
 *   Column 4-5 - Latin A-Z region (@..O, P..Z plus [ \ ] ^ _)
 *   Column 6-7 - Hebrew alphabet, laid out in Unicode block order
 *                U+05D0 (alef) .. U+05EA (tav), including the 5 final forms,
 *                at codes 0x60..0x7A.  RTL script: the code->glyph mapping is
 *                recorded by CELL POSITION, not reordered.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* --- Column 0 (0x00-0x0F): controls + symbols --- */
	/* 0x00 */ { NULL },           /* NUL */
	/* 0x01 */ { "\xc2\xa4" },     /* U+00A4 ¤ currency sign */
	/* 0x02 */ { "\xc2\xb5" },     /* U+00B5 µ micro sign */
	/* 0x03 */ { NULL },           /* ETX */
	/* 0x04 */ { NULL },           /* EOT */
	/* 0x05 */ { "\xc2\xa2" },     /* U+00A2 ¢ cent sign */
	/* 0x06 */ { "\xc3\x97" },     /* U+00D7 × multiplication sign */
	/* 0x07 */ { "\xc3\xb7" },     /* U+00F7 ÷ division sign */
	/* 0x08 */ { "\xc2\xa3" },     /* U+00A3 £ pound sign */
	/* 0x09 */ { "\xc2\xb0" },     /* U+00B0 ° degree sign */
	/* 0x0A */ { "\n" },           /* LF  */
	/* 0x0B */ { "\xc2\xab" },     /* U+00AB « left guillemet */
	/* 0x0C */ { "\xc2\xbf" },     /* U+00BF ¿ inverted question mark */
	/* 0x0D */ { "\r" },           /* CR  */
	/* 0x0E */ { "\xc2\xbb" },     /* U+00BB » right guillemet */
	/* 0x0F */ { "\xc2\xa1" },     /* U+00A1 ¡ inverted exclamation mark */
	/* --- Column 1 (0x10-0x1F): symbols + controls --- */
	/* 0x10 */ { "\xc2\xac" },     /* U+00AC ¬ not sign */
	/* 0x11 */ { "\xc2\xad" },     /* U+00AD soft hyphen (rendered as a plain overbar/dash)  / * UNCERTAIN: short horizontal bar, distinct from 0x10 */
	/* 0x12 */ { "\xc2\xbd" },     /* U+00BD ½ one half */
	/* 0x13 */ { "\xc2\xa6" },     /* U+00A6 ¦ broken bar */
	/* 0x14 */ { "\xc2\xa7" },     /* U+00A7 § section sign  / * UNCERTAIN: reversed-S glyph */
	/* 0x15 */ { "\xc2\xb2" },     /* U+00B2 ² superscript two */
	/* 0x16 */ { "`" },            /* grave accent  / * UNCERTAIN: small tick at top-right of cell */
	/* 0x17 */ { NULL },           /* ETB */
	/* 0x18 */ { "\xc2\xb3" },     /* U+00B3 ³ superscript three */
	/* 0x19 */ { "\xc2\xa9" },     /* U+00A9 © copyright sign */
	/* 0x1A */ { "\xc2\xb7" },     /* U+00B7 · middle dot */
	/* 0x1B */ { NULL },           /* ESC */
	/* 0x1C */ { "\xc2\xa5" },     /* U+00A5 ¥ yen sign */
	/* 0x1D */ { "\xc2\xb1" },     /* U+00B1 ± plus-minus sign */
	/* 0x1E */ { "." },            /* full stop / low dot  / * UNCERTAIN */
	/* 0x1F */ { NULL },           /* US  */
	/* --- Column 2 (0x20-0x2F): punctuation --- */
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
	/* 0x5E */ { "^" },
	/* 0x5F */ { "_" },
	/* --- Column 6 (0x60-0x6F): Hebrew alef .. final-nun --- */
	/* 0x60 */ { "\xd7\x90" },     /* U+05D0 א HEBREW LETTER ALEF */
	/* 0x61 */ { "\xd7\x91" },     /* U+05D1 ב HEBREW LETTER BET */
	/* 0x62 */ { "\xd7\x92" },     /* U+05D2 ג HEBREW LETTER GIMEL */
	/* 0x63 */ { "\xd7\x93" },     /* U+05D3 ד HEBREW LETTER DALET */
	/* 0x64 */ { "\xd7\x94" },     /* U+05D4 ה HEBREW LETTER HE */
	/* 0x65 */ { "\xd7\x95" },     /* U+05D5 ו HEBREW LETTER VAV */
	/* 0x66 */ { "\xd7\x96" },     /* U+05D6 ז HEBREW LETTER ZAYIN */
	/* 0x67 */ { "\xd7\x97" },     /* U+05D7 ח HEBREW LETTER HET */
	/* 0x68 */ { "\xd7\x98" },     /* U+05D8 ט HEBREW LETTER TET */
	/* 0x69 */ { "\xd7\x99" },     /* U+05D9 י HEBREW LETTER YOD */
	/* 0x6A */ { "\xd7\x9a" },     /* U+05DA ך HEBREW LETTER FINAL KAF */
	/* 0x6B */ { "\xd7\x9b" },     /* U+05DB כ HEBREW LETTER KAF */
	/* 0x6C */ { "\xd7\x9c" },     /* U+05DC ל HEBREW LETTER LAMED */
	/* 0x6D */ { "\xd7\x9d" },     /* U+05DD ם HEBREW LETTER FINAL MEM */
	/* 0x6E */ { "\xd7\x9e" },     /* U+05DE מ HEBREW LETTER MEM */
	/* 0x6F */ { "\xd7\x9f" },     /* U+05DF ן HEBREW LETTER FINAL NUN */
	/* --- Column 7 (0x70-0x7F): Hebrew nun .. tav + brackets --- */
	/* 0x70 */ { "\xd7\xa0" },     /* U+05E0 נ HEBREW LETTER NUN */
	/* 0x71 */ { "\xd7\xa1" },     /* U+05E1 ס HEBREW LETTER SAMEKH */
	/* 0x72 */ { "\xd7\xa2" },     /* U+05E2 ע HEBREW LETTER AYIN */
	/* 0x73 */ { "\xd7\xa3" },     /* U+05E3 ף HEBREW LETTER FINAL PE */
	/* 0x74 */ { "\xd7\xa4" },     /* U+05E4 פ HEBREW LETTER PE */
	/* 0x75 */ { "\xd7\xa5" },     /* U+05E5 ץ HEBREW LETTER FINAL TSADI */
	/* 0x76 */ { "\xd7\xa6" },     /* U+05E6 צ HEBREW LETTER TSADI */
	/* 0x77 */ { "\xd7\xa7" },     /* U+05E7 ק HEBREW LETTER QOF */
	/* 0x78 */ { "\xd7\xa8" },     /* U+05E8 ר HEBREW LETTER RESH */
	/* 0x79 */ { "\xd7\xa9" },     /* U+05E9 ש HEBREW LETTER SHIN */
	/* 0x7A */ { "\xd7\xaa" },     /* U+05EA ת HEBREW LETTER TAV */
	/* 0x7B */ { "{" },
	/* 0x7C */ { "|" },
	/* 0x7D */ { "}" },
	/* 0x7E */ { "~" },
	/* 0x7F */ { " " },            /* SP */
};

const struct pager_charset pager_charset_flex_motorola_linguist_hebrew = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "hebrew",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
