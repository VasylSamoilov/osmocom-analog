/*
 * FLEX Greek codepage — Motorola FLX2 Linguist (Table A-13, ISO/IEC 8859-7).
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
 * Charset:  greek
 *
 * Transcribed from the rendered manual page (image is ground truth), NOT
 * generated from the Unicode ISO-8859-7 standard: this is the pager's own
 * 7-bit CG-ROM layout.  byte = (column << 4) | row.
 *
 * Layout:
 *   Column 0-1 - control region + symbol glyphs (shared across the A-12..A-15
 *                family on this page: ¤ µ ¢ × ÷ £ ° « ¿ » ¡ ¬ ½ ¦ § ² ³ © ¥ ± ·)
 *   Column 2-3 - ASCII punctuation + digits
 *   Column 4-5 - Latin A-Z region (@..O, P..Z plus [ \ ] ^ _)
 *   Column 6-7 - Greek capital letters + accented capitals + breathing marks
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
	/* 0x11 */ { NULL },           /* blank */
	/* 0x12 */ { "\xc2\xbd" },     /* U+00BD ½ one half */
	/* 0x13 */ { "\xc2\xa6" },     /* U+00A6 ¦ broken bar */
	/* 0x14 */ { "\xc2\xa7" },     /* U+00A7 § section sign  / * UNCERTAIN: reversed-S glyph */
	/* 0x15 */ { "\xc2\xb2" },     /* U+00B2 ² superscript two */
	/* 0x16 */ { NULL },           /* blank */
	/* 0x17 */ { NULL },           /* ETB */
	/* 0x18 */ { "\xc2\xb3" },     /* U+00B3 ³ superscript three */
	/* 0x19 */ { "\xc2\xa9" },     /* U+00A9 © copyright sign */
	/* 0x1A */ { NULL },           /* blank */
	/* 0x1B */ { NULL },           /* ESC */
	/* 0x1C */ { "\xc2\xa5" },     /* U+00A5 ¥ yen sign */
	/* 0x1D */ { "\xc2\xb1" },     /* U+00B1 ± plus-minus sign */
	/* 0x1E */ { "\xc2\xb7" },     /* U+00B7 · middle dot */
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
	/* --- Column 6 (0x60-0x6F): Greek capitals + accented --- */
	/* 0x60 */ { "`" },            /* grave accent */
	/* 0x61 */ { "\xce\x93" },     /* U+0393 Γ GREEK CAPITAL GAMMA */
	/* 0x62 */ { "\xce\x94" },     /* U+0394 Δ GREEK CAPITAL DELTA */
	/* 0x63 */ { "\xce\x86" },     /* U+0386 Ά GREEK CAPITAL ALPHA WITH TONOS */
	/* 0x64 */ { "\xce\x88" },     /* U+0388 Έ GREEK CAPITAL EPSILON WITH TONOS */
	/* 0x65 */ { "\xce\x89" },     /* U+0389 Ή GREEK CAPITAL ETA WITH TONOS */
	/* 0x66 */ { "\xce\x8a" },     /* U+038A Ί GREEK CAPITAL IOTA WITH TONOS */
	/* 0x67 */ { "\xce\x98" },     /* U+0398 Θ GREEK CAPITAL THETA */
	/* 0x68 */ { "\xce\xa8" },     /* U+03A8 Ψ GREEK CAPITAL PSI */
	/* 0x69 */ { "\xce\xa9" },     /* U+03A9 Ω GREEK CAPITAL OMEGA */
	/* 0x6A */ { "\xce\xaa" },     /* U+03AA Ϊ GREEK CAPITAL IOTA WITH DIALYTIKA */
	/* 0x6B */ { "\xce\x9b" },     /* U+039B Λ GREEK CAPITAL LAMBDA */
	/* 0x6C */ { "\xce\x8c" },     /* U+038C Ό GREEK CAPITAL OMICRON WITH TONOS */
	/* 0x6D */ { "\xce\x8f" },     /* U+038F Ώ GREEK CAPITAL OMEGA WITH TONOS  / * UNCERTAIN: ornate/blackletter omega glyph */
	/* 0x6E */ { "\xca\xbd" },     /* U+02BD ʽ MODIFIER LETTER REVERSED COMMA (rough breathing)  / * UNCERTAIN */
	/* 0x6F */ { "\xca\xbc" },     /* U+02BC ʼ MODIFIER LETTER APOSTROPHE (smooth breathing)  / * UNCERTAIN */
	/* --- Column 7 (0x70-0x7F): Greek capitals + brackets --- */
	/* 0x70 */ { "\xce\xa0" },     /* U+03A0 Π GREEK CAPITAL PI */
	/* 0x71 */ { "\xce\xa3" },     /* U+03A3 Σ GREEK CAPITAL SIGMA */
	/* 0x72 */ { "\xce\xa5" },     /* U+03A5 Υ GREEK CAPITAL UPSILON */
	/* 0x73 */ { "\xce\xa6" },     /* U+03A6 Φ GREEK CAPITAL PHI */
	/* 0x74 */ { "\xce\xab" },     /* U+03AB Ϋ GREEK CAPITAL UPSILON WITH DIALYTIKA */
	/* 0x75 */ { "\xce\x8e" },     /* U+038E Ύ GREEK CAPITAL UPSILON WITH TONOS */
	/* 0x76 */ { "\xce\x9e" },     /* U+039E Ξ GREEK CAPITAL XI */
	/* 0x77 */ { "\xc2\xb7" },     /* U+00B7 · middle dot (ano teleia) */
	/* 0x78 */ { "'" },            /* apostrophe */
	/* 0x79 */ { "\xe2\x80\x94" }, /* U+2014 — em dash (horizontal bar)  / * UNCERTAIN: long dash */
	/* 0x7A */ { "\xe2\x80\xa5" }, /* U+2025 ‥ two dot leader  / * UNCERTAIN: raised double-dot-with-mark glyph */
	/* 0x7B */ { "{" },
	/* 0x7C */ { "|" },
	/* 0x7D */ { "}" },
	/* 0x7E */ { "~" },
	/* 0x7F */ { " " },            /* SP */
};

const struct pager_charset pager_charset_flex_motorola_linguist_greek = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "greek",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
