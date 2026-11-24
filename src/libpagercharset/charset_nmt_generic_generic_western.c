/*
 * NMT SMS Character Set 3 - Western.
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
 * Charset:  western
 *
 * 8-bit character set (256 codes) of NMT SMS with TP-DCS 0011 0011: NMT
 * Character Set 3 - Western Alphabet, Danish, Dutch, English, Finnish,
 * French, German, Icelandic, Norwegian, Swedish (NMT Doc 450-3/900-3 Annex
 * 24 v.8.0 4.3.4).  0x00-0x7F are the Default GSM Character Set; 0x80-0x91
 * and 0xA0-0xB1 as printed in the standard; the codes it does not print are
 * unmapped.  0x83, 0xA0, 0xA4 and 0xAD are printed "<... is also included in
 * GSM Set>": they are Ç, à, è and ù, which TX takes as their GSM codes (the
 * first).
 */

#include "pager_charset.h"

static const pager_charset_entry entries[256] = {
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
	/* 0x0A */ { "\n" },  /* LF  */
	/* 0x0B */ { "Ø" },   /* U+00D8 */
	/* 0x0C */ { "ø" },   /* U+00F8 */
	/* 0x0D */ { "\r" },  /* CR  */
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
	/* 0x80 */ { "À" },   /* U+00C0 */
	/* 0x81 */ { "Á" },   /* U+00C1 */
	/* 0x82 */ { "Â" },   /* U+00C2 */
	/* 0x83 */ { "Ç" },   /* U+00C7 — also GSM 0x09 */
	/* 0x84 */ { "È" },   /* U+00C8 */
	/* 0x85 */ { "Ê" },   /* U+00CA */
	/* 0x86 */ { "Ë" },   /* U+00CB */
	/* 0x87 */ { "Í" },   /* U+00CD */
	/* 0x88 */ { "Î" },   /* U+00CE */
	/* 0x89 */ { "Ï" },   /* U+00CF */
	/* 0x8A */ { "Ð" },   /* U+00D0 */
	/* 0x8B */ { "Ó" },   /* U+00D3 */
	/* 0x8C */ { "Ô" },   /* U+00D4 */
	/* 0x8D */ { "Ù" },   /* U+00D9 */
	/* 0x8E */ { "Ú" },   /* U+00DA */
	/* 0x8F */ { "Û" },   /* U+00DB */
	/* 0x90 */ { "Ý" },   /* U+00DD */
	/* 0x91 */ { "Þ" },   /* U+00DE */
	/* 0x92 */ { NULL },
	/* 0x93 */ { NULL },
	/* 0x94 */ { NULL },
	/* 0x95 */ { NULL },
	/* 0x96 */ { NULL },
	/* 0x97 */ { NULL },
	/* 0x98 */ { NULL },
	/* 0x99 */ { NULL },
	/* 0x9A */ { NULL },
	/* 0x9B */ { NULL },
	/* 0x9C */ { NULL },
	/* 0x9D */ { NULL },
	/* 0x9E */ { NULL },
	/* 0x9F */ { NULL },
	/* 0xA0 */ { "à" },   /* U+00E0 — also GSM 0x7F */
	/* 0xA1 */ { "á" },   /* U+00E1 */
	/* 0xA2 */ { "â" },   /* U+00E2 */
	/* 0xA3 */ { "ç" },   /* U+00E7 */
	/* 0xA4 */ { "è" },   /* U+00E8 — also GSM 0x04 */
	/* 0xA5 */ { "ê" },   /* U+00EA */
	/* 0xA6 */ { "ë" },   /* U+00EB */
	/* 0xA7 */ { "í" },   /* U+00ED */
	/* 0xA8 */ { "î" },   /* U+00EE */
	/* 0xA9 */ { "ï" },   /* U+00EF */
	/* 0xAA */ { "ð" },   /* U+00F0 */
	/* 0xAB */ { "ó" },   /* U+00F3 */
	/* 0xAC */ { "ô" },   /* U+00F4 */
	/* 0xAD */ { "ù" },   /* U+00F9 — also GSM 0x06 */
	/* 0xAE */ { "ú" },   /* U+00FA */
	/* 0xAF */ { "û" },   /* U+00FB */
	/* 0xB0 */ { "ý" },   /* U+00FD */
	/* 0xB1 */ { "þ" },   /* U+00FE */
	/* 0xB2 */ { NULL },
	/* 0xB3 */ { NULL },
	/* 0xB4 */ { NULL },
	/* 0xB5 */ { NULL },
	/* 0xB6 */ { NULL },
	/* 0xB7 */ { NULL },
	/* 0xB8 */ { NULL },
	/* 0xB9 */ { NULL },
	/* 0xBA */ { NULL },
	/* 0xBB */ { NULL },
	/* 0xBC */ { NULL },
	/* 0xBD */ { NULL },
	/* 0xBE */ { NULL },
	/* 0xBF */ { NULL },
	/* 0xC0 */ { NULL },
	/* 0xC1 */ { NULL },
	/* 0xC2 */ { NULL },
	/* 0xC3 */ { NULL },
	/* 0xC4 */ { NULL },
	/* 0xC5 */ { NULL },
	/* 0xC6 */ { NULL },
	/* 0xC7 */ { NULL },
	/* 0xC8 */ { NULL },
	/* 0xC9 */ { NULL },
	/* 0xCA */ { NULL },
	/* 0xCB */ { NULL },
	/* 0xCC */ { NULL },
	/* 0xCD */ { NULL },
	/* 0xCE */ { NULL },
	/* 0xCF */ { NULL },
	/* 0xD0 */ { NULL },
	/* 0xD1 */ { NULL },
	/* 0xD2 */ { NULL },
	/* 0xD3 */ { NULL },
	/* 0xD4 */ { NULL },
	/* 0xD5 */ { NULL },
	/* 0xD6 */ { NULL },
	/* 0xD7 */ { NULL },
	/* 0xD8 */ { NULL },
	/* 0xD9 */ { NULL },
	/* 0xDA */ { NULL },
	/* 0xDB */ { NULL },
	/* 0xDC */ { NULL },
	/* 0xDD */ { NULL },
	/* 0xDE */ { NULL },
	/* 0xDF */ { NULL },
	/* 0xE0 */ { NULL },
	/* 0xE1 */ { NULL },
	/* 0xE2 */ { NULL },
	/* 0xE3 */ { NULL },
	/* 0xE4 */ { NULL },
	/* 0xE5 */ { NULL },
	/* 0xE6 */ { NULL },
	/* 0xE7 */ { NULL },
	/* 0xE8 */ { NULL },
	/* 0xE9 */ { NULL },
	/* 0xEA */ { NULL },
	/* 0xEB */ { NULL },
	/* 0xEC */ { NULL },
	/* 0xED */ { NULL },
	/* 0xEE */ { NULL },
	/* 0xEF */ { NULL },
	/* 0xF0 */ { NULL },
	/* 0xF1 */ { NULL },
	/* 0xF2 */ { NULL },
	/* 0xF3 */ { NULL },
	/* 0xF4 */ { NULL },
	/* 0xF5 */ { NULL },
	/* 0xF6 */ { NULL },
	/* 0xF7 */ { NULL },
	/* 0xF8 */ { NULL },
	/* 0xF9 */ { NULL },
	/* 0xFA */ { NULL },
	/* 0xFB */ { NULL },
	/* 0xFC */ { NULL },
	/* 0xFD */ { NULL },
	/* 0xFE */ { NULL },
	/* 0xFF */ { NULL },
};

const struct pager_charset pager_charset_nmt_generic_generic_western = {
	.protocol  = "nmt",
	.vendor    = "generic",
	.model     = "generic",
	.charset   = "western",
	.code_bits = 8,
	.num_codes = 256,
	.entries   = entries,
};
