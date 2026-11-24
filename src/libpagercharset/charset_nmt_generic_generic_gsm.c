/*
 * NMT SMS GSM default character set.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: nmt
 * Vendor:   generic
 * Model:    generic
 * Charset:  gsm
 *
 * 7-bit character set (128 codes) of NMT SMS with TP-DCS 0000 0000, the
 * Default GSM Alphabet (NMT Doc 450-3/900-3 Annex 24 2.2.3.10, 4.2), which
 * every NMT MS with SMS sends and receives.  It is also the first half
 * (0x00-0x7F) of every NMT 8-bit character set.
 *
 * 0x1B is the escape to the GSM extension table (a character of two codes):
 * it is unmapped here, the SMS coding handles it.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00 */ { "@" },
	/* 0x01 */ { "£" },   /* U+00A3 */
	/* 0x02 */ { "$" },
	/* 0x03 */ { "¥" },   /* U+00A5 */
	/* 0x04 */ { "è" },   /* U+00E8 */
	/* 0x05 */ { "é" },   /* U+00E9 */
	/* 0x06 */ { "ù" },   /* U+00F9 */
	/* 0x07 */ { "ì" },   /* U+00EC */
	/* 0x08 */ { "ò" },   /* U+00F2 */
	/* 0x09 */ { "Ç" },   /* U+00C7 */
	/* 0x0A */ { "␊", "\n" },   /* U+240A, LF */
	/* 0x0B */ { "Ø" },   /* U+00D8 */
	/* 0x0C */ { "ø" },   /* U+00F8 */
	/* 0x0D */ { "␍", "\r" },   /* U+240D, CR */
	/* 0x0E */ { "Å" },   /* U+00C5 */
	/* 0x0F */ { "å" },   /* U+00E5 */
	/* 0x10 */ { "Δ" },   /* U+0394 */
	/* 0x11 */ { "_" },
	/* 0x12 */ { "Φ" },   /* U+03A6 */
	/* 0x13 */ { "Γ" },   /* U+0393 */
	/* 0x14 */ { "Λ" },   /* U+039B */
	/* 0x15 */ { "Ω" },   /* U+03A9 */
	/* 0x16 */ { "Π" },   /* U+03A0 */
	/* 0x17 */ { "Ψ" },   /* U+03A8 */
	/* 0x18 */ { "Σ" },   /* U+03A3 */
	/* 0x19 */ { "Θ" },   /* U+0398 */
	/* 0x1A */ { "Ξ" },   /* U+039E */
	/* 0x1B */ { NULL },  /* ESC: extension table */
	/* 0x1C */ { "Æ" },   /* U+00C6 */
	/* 0x1D */ { "æ" },   /* U+00E6 */
	/* 0x1E */ { "ß" },   /* U+00DF */
	/* 0x1F */ { "É" },   /* U+00C9 */
	/* 0x20 */ { " " },
	/* 0x21 */ { "!" },
	/* 0x22 */ { "\"" },
	/* 0x23 */ { "#" },
	/* 0x24 */ { "¤" },   /* U+00A4 */
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
	/* 0x40 */ { "¡" },   /* U+00A1 */
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
	/* 0x5B */ { "Ä" },   /* U+00C4 */
	/* 0x5C */ { "Ö" },   /* U+00D6 */
	/* 0x5D */ { "Ñ" },   /* U+00D1 */
	/* 0x5E */ { "Ü" },   /* U+00DC */
	/* 0x5F */ { "§" },   /* U+00A7 */
	/* 0x60 */ { "¿" },   /* U+00BF */
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
	/* 0x7B */ { "ä" },   /* U+00E4 */
	/* 0x7C */ { "ö" },   /* U+00F6 */
	/* 0x7D */ { "ñ" },   /* U+00F1 */
	/* 0x7E */ { "ü" },   /* U+00FC */
	/* 0x7F */ { "à" },   /* U+00E0 */
};

const struct pager_charset pager_charset_nmt_generic_generic_gsm = {
	.protocol  = "nmt",
	.vendor    = "generic",
	.model     = "generic",
	.charset   = "gsm",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
