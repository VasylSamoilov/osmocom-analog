/*
 * RDS Nokia NKP2 pager character set.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: rds
 * Vendor:   nokia
 * Model:    nkp2
 * Charset:  annex-e
 *
 * 8-bit character set (256 codes) as implemented in the Nokia NKP2
 * Text pager.  Based on IEC 62106 Annex E but with deviations:
 *
 *   0x24 = $ (dollar sign, not ¤ as in standard Annex E)
 *   0x5E = blank (not ― horizontal bar)
 *   0x5F = blank (not _)
 *   0x60 = blank (not ║)
 *   0x7C = ¦ (broken bar, not |)
 *   0x7E = blank (not ¯)
 *   0x7F = blank (not DEL)
 *
 * Many extended characters (0x80-0xFF) that exist in standard Annex E
 * are blank on the Nokia pager — its character ROM is a strict subset.
 *
 * Verified by sending raw bytes 0x00-0xFF via RDS enhanced paging
 * raw bytes 0x00-0xFF to a real Nokia NKP2 pager and photographing the
 * display (32 photos, 8 bytes each).
 *
 * See also: docs/pager_charset_nokia.md
 */

#include "pager_charset.h"

static const pager_charset_entry entries[256] = {
	/* 0x00-0x1F: Control characters (all blank on pager) */
	/* 0x00 */ { NULL },
	/* 0x01 */ { NULL },
	/* 0x02 */ { NULL },
	/* 0x03 */ { NULL },
	/* 0x04 */ { NULL },
	/* 0x05 */ { NULL },
	/* 0x06 */ { NULL },
	/* 0x07 */ { NULL },
	/* 0x08 */ { NULL },
	/* 0x09 */ { NULL },
	/* 0x0A */ { "\n" },  /* LF */
	/* 0x0B */ { NULL },
	/* 0x0C */ { NULL },
	/* 0x0D */ { "\r" },  /* CR */
	/* 0x0E */ { NULL },
	/* 0x0F */ { NULL },
	/* 0x10 */ { NULL },
	/* 0x11 */ { NULL },
	/* 0x12 */ { NULL },
	/* 0x13 */ { NULL },
	/* 0x14 */ { NULL },
	/* 0x15 */ { NULL },
	/* 0x16 */ { NULL },
	/* 0x17 */ { NULL },
	/* 0x18 */ { NULL },
	/* 0x19 */ { NULL },
	/* 0x1A */ { NULL },
	/* 0x1B */ { NULL },
	/* 0x1C */ { NULL },
	/* 0x1D */ { NULL },
	/* 0x1E */ { NULL },
	/* 0x1F */ { NULL },

	/* 0x20-0x3F: ASCII punctuation and digits (0x24 = $ not ¤) */
	/* 0x20 */ { " " },
	/* 0x21 */ { "!" },
	/* 0x22 */ { "\"" },
	/* 0x23 */ { "#" },
	/* 0x24 */ { "$" },   /* Nokia: $ (standard Annex E has ¤) */
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

	/* 0x40-0x5F: Uppercase (0x5E-0x5F blank on Nokia) */
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
	/* 0x5E */ { NULL },  /* blank (standard: U+2015 ―) */
	/* 0x5F */ { NULL },  /* blank (standard: _) */

	/* 0x60-0x7F: Lowercase (0x60, 0x7E-0x7F blank; 0x7C = ¦) */
	/* 0x60 */ { NULL },  /* blank (standard: U+2551 ║) */
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
	/* 0x7C */ { "¦" },       /* U+00A6 broken bar */
	/* 0x7D */ { "}" },
	/* 0x7E */ { NULL },  /* blank (standard: U+00AF ¯) */
	/* 0x7F */ { NULL },  /* blank */

	/* 0x80-0x8F: Accented lowercase (0x8C=Ş blank, 0x8F=Ĳ blank) */
	/* 0x80 */ { "á" },   /* U+00E1 */
	/* 0x81 */ { "à" },   /* U+00E0 */
	/* 0x82 */ { "é" },   /* U+00E9 */
	/* 0x83 */ { "è" },   /* U+00E8 */
	/* 0x84 */ { "í" },   /* U+00ED */
	/* 0x85 */ { "ì" },   /* U+00EC */
	/* 0x86 */ { "ó" },   /* U+00F3 */
	/* 0x87 */ { "ò" },   /* U+00F2 */
	/* 0x88 */ { "ú" },   /* U+00FA */
	/* 0x89 */ { "ù" },   /* U+00F9 */
	/* 0x8A */ { "Ñ" },   /* U+00D1 */
	/* 0x8B */ { "Ç" },   /* U+00C7 */
	/* 0x8C */ { NULL },  /* blank (standard: U+015E Ş) */
	/* 0x8D */ { "ß" },   /* U+00DF */
	/* 0x8E */ { "¡" },   /* U+00A1 */
	/* 0x8F */ { NULL },  /* blank (standard: U+0132 Ĳ) */

	/* 0x90-0x9F: More accented (0x9C=ş, 0x9E=ı, 0x9F=ĳ blank) */
	/* 0x90 */ { "â" },   /* U+00E2 */
	/* 0x91 */ { "ä" },   /* U+00E4 */
	/* 0x92 */ { "ê" },   /* U+00EA */
	/* 0x93 */ { "ë" },   /* U+00EB */
	/* 0x94 */ { "î" },   /* U+00EE */
	/* 0x95 */ { "ï" },   /* U+00EF */
	/* 0x96 */ { "ô" },   /* U+00F4 */
	/* 0x97 */ { "ö" },   /* U+00F6 */
	/* 0x98 */ { "û" },   /* U+00FB */
	/* 0x99 */ { "ü" },   /* U+00FC */
	/* 0x9A */ { "ñ" },   /* U+00F1 */
	/* 0x9B */ { "ç" },   /* U+00E7 */
	/* 0x9C */ { NULL },  /* blank (standard: U+015F ş) */
	/* 0x9D */ { "ğ" },   /* U+011F */
	/* 0x9E */ { NULL },  /* blank (standard: U+0131 ı) */
	/* 0x9F */ { NULL },  /* blank (standard: U+0133 ĳ) */

	/* 0xA0-0xAF: Symbols (many blank on Nokia) */
	/* 0xA0 */ { NULL },  /* blank (standard: U+00AA ª) */
	/* 0xA1 */ { NULL },  /* blank (standard: U+03B1 α) */
	/* 0xA2 */ { NULL },  /* blank (standard: U+00A9 ©) */
	/* 0xA3 */ { NULL },  /* blank (standard: U+2030 ‰) */
	/* 0xA4 */ { "Ğ" },   /* U+011E */
	/* 0xA5 */ { "ě" },   /* U+011B */
	/* 0xA6 */ { "ň" },   /* U+0148 */
	/* 0xA7 */ { "ő" },   /* U+0151 */
	/* 0xA8 */ { NULL },  /* blank (standard: U+03C0 π) */
	/* 0xA9 */ { NULL },  /* blank (standard: U+20AC €) */
	/* 0xAA */ { "£" },   /* U+00A3 */
	/* 0xAB */ { "$" },
	/* 0xAC */ { "←" },   /* U+2190 */
	/* 0xAD */ { "↑" },   /* U+2191 */
	/* 0xAE */ { "→" },   /* U+2192 */
	/* 0xAF */ { "↓" },   /* U+2193 */

	/* 0xB0-0xBF: More symbols (many blank on Nokia) */
	/* 0xB0 */ { NULL },  /* blank (standard: U+00BA º) */
	/* 0xB1 */ { NULL },  /* blank (standard: U+00B9 ¹) */
	/* 0xB2 */ { NULL },  /* blank (standard: U+00B2 ²) */
	/* 0xB3 */ { NULL },  /* blank (standard: U+00B3 ³) */
	/* 0xB4 */ { NULL },  /* blank (standard: U+00B1 ±) */
	/* 0xB5 */ { "İ" },   /* U+0130 */
	/* 0xB6 */ { "ń" },   /* U+0144 */
	/* 0xB7 */ { "ű" },   /* U+0171 */
	/* 0xB8 */ { "µ" },   /* U+00B5 */
	/* 0xB9 */ { "¿" },   /* U+00BF */
	/* 0xBA */ { NULL },  /* blank (standard: U+00F7 ÷) */
	/* 0xBB */ { "°" },   /* U+00B0 */
	/* 0xBC */ { NULL },  /* blank (standard: U+00BC ¼) */
	/* 0xBD */ { NULL },  /* blank (standard: U+00BD ½) */
	/* 0xBE */ { NULL },  /* blank (standard: U+00BE ¾) */
	/* 0xBF */ { "§" },   /* U+00A7 */

	/* 0xC0-0xCF: Accented uppercase (all supported on Nokia) */
	/* 0xC0 */ { "Á" },   /* U+00C1 */
	/* 0xC1 */ { "À" },   /* U+00C0 */
	/* 0xC2 */ { "É" },   /* U+00C9 */
	/* 0xC3 */ { "È" },   /* U+00C8 */
	/* 0xC4 */ { "Í" },   /* U+00CD */
	/* 0xC5 */ { "Ì" },   /* U+00CC */
	/* 0xC6 */ { "Ó" },   /* U+00D3 */
	/* 0xC7 */ { "Ò" },   /* U+00D2 */
	/* 0xC8 */ { "Ú" },   /* U+00DA */
	/* 0xC9 */ { "Ù" },   /* U+00D9 */
	/* 0xCA */ { "Ř" },   /* U+0158 */
	/* 0xCB */ { "Č" },   /* U+010C */
	/* 0xCC */ { "Š" },   /* U+0160 */
	/* 0xCD */ { "Ž" },   /* U+017D */
	/* 0xCE */ { "Đ" },   /* U+0110 */
	/* 0xCF */ { "Ŀ" },   /* U+013F */

	/* 0xD0-0xDF: More accented uppercase (all supported) */
	/* 0xD0 */ { "Â" },   /* U+00C2 */
	/* 0xD1 */ { "Ä" },   /* U+00C4 */
	/* 0xD2 */ { "Ê" },   /* U+00CA */
	/* 0xD3 */ { "Ë" },   /* U+00CB */
	/* 0xD4 */ { "Î" },   /* U+00CE */
	/* 0xD5 */ { "Ï" },   /* U+00CF */
	/* 0xD6 */ { "Ô" },   /* U+00D4 */
	/* 0xD7 */ { "Ö" },   /* U+00D6 */
	/* 0xD8 */ { "Û" },   /* U+00DB */
	/* 0xD9 */ { "Ü" },   /* U+00DC */
	/* 0xDA */ { "ř" },   /* U+0159 */
	/* 0xDB */ { "č" },   /* U+010D */
	/* 0xDC */ { "š" },   /* U+0161 */
	/* 0xDD */ { "ž" },   /* U+017E */
	/* 0xDE */ { "đ" },   /* U+0111 */
	/* 0xDF */ { "ŀ" },   /* U+0140 */

	/* 0xE0-0xEF: Nordic/special (some blank on Nokia) */
	/* 0xE0 */ { "Ã" },   /* U+00C3 */
	/* 0xE1 */ { "Å" },   /* U+00C5 */
	/* 0xE2 */ { "Æ" },   /* U+00C6 */
	/* 0xE3 */ { NULL },  /* blank (standard: U+0152 Œ) */
	/* 0xE4 */ { NULL },  /* blank (standard: U+0177 ŷ) */
	/* 0xE5 */ { "Ý" },   /* U+00DD */
	/* 0xE6 */ { "Õ" },   /* U+00D5 */
	/* 0xE7 */ { "Ø" },   /* U+00D8 */
	/* 0xE8 */ { NULL },  /* blank (standard: U+00DE Þ) */
	/* 0xE9 */ { NULL },  /* blank (standard: U+014A Ŋ) */
	/* 0xEA */ { "Ŕ" },   /* U+0154 */
	/* 0xEB */ { "Ć" },   /* U+0106 */
	/* 0xEC */ { "Ś" },   /* U+015A */
	/* 0xED */ { "Ź" },   /* U+0179 */
	/* 0xEE */ { NULL },  /* blank (standard: U+0166 Ŧ) */
	/* 0xEF */ { NULL },  /* blank (standard: U+00F0 ð) */

	/* 0xF0-0xFF: Nordic/special lowercase (some blank on Nokia) */
	/* 0xF0 */ { "ã" },   /* U+00E3 */
	/* 0xF1 */ { "å" },   /* U+00E5 */
	/* 0xF2 */ { "æ" },   /* U+00E6 */
	/* 0xF3 */ { NULL },  /* blank (standard: U+0153 œ) */
	/* 0xF4 */ { "ŵ" },   /* U+0175 */
	/* 0xF5 */ { "ý" },   /* U+00FD */
	/* 0xF6 */ { "õ" },   /* U+00F5 */
	/* 0xF7 */ { "ø" },   /* U+00F8 */
	/* 0xF8 */ { NULL },  /* blank (standard: U+00FE þ) */
	/* 0xF9 */ { NULL },  /* blank (standard: U+014B ŋ) */
	/* 0xFA */ { "ŕ" },   /* U+0155 */
	/* 0xFB */ { "ć" },   /* U+0107 */
	/* 0xFC */ { "ś" },   /* U+015B */
	/* 0xFD */ { "ź" },   /* U+017A */
	/* 0xFE */ { NULL },  /* blank (standard: U+0167 ŧ) */
	/* 0xFF */ { NULL },
};

const struct pager_charset pager_charset_rds_nokia_nkp2_annex_e = {
	.protocol  = "rds",
	.vendor    = "nokia",
	.model     = "nkp2",
	.charset   = "annex-e",
	.code_bits = 8,
	.num_codes = 256,
	.entries   = entries,
};
