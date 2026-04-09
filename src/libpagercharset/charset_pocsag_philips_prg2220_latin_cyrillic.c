/*
 * POCSAG Philips PRG2220 Latin/Cyrillic codepage.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: pocsag
 * Vendor:   philips
 * Model:    prg2220
 * Charset:  latin-cyrillic
 *
 * Verified against real pager hardware.
 * Pagers: Philips PRG2220, PRG2310
 *
 * Layout:
 *   0x20-0x3F: Standard ASCII punctuation and digits.
 *   0x40-0x5F: Latin uppercase A-Z (no Cyrillic remapping here).
 *   0x60-0x7F: Cyrillic А-Я sequential (А=0x60 through Я=0x7F).
 *
 * Only alias: Ё maps to same code as Е (0x65) — both Е and Ё at 0x65.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00-0x1F: Control region — unmapped */
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
	/* 0x0A */ { "\n" },
	/* 0x0B */ { NULL },
	/* 0x0C */ { NULL },
	/* 0x0D */ { "\r" },
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

	/* 0x20-0x3F: Standard ASCII */
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

	/* 0x40-0x5F: Latin uppercase (standard ASCII layout) */
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
	/* 0x5E */ { "^" },
	/* 0x5F */ { "_" },

	/* 0x60-0x7F: Cyrillic А-Я sequential.
	 * Only verified alias: Ё at 0x65 (both Е and Ё map to same code). */
	/* 0x60 */ { "А" },                   /* U+0410 cyr-A */
	/* 0x61 */ { "Б" },                   /* U+0411 cyr-Be */
	/* 0x62 */ { "В" },                   /* U+0412 cyr-Ve */
	/* 0x63 */ { "Г" },                   /* U+0413 cyr-Ghe */
	/* 0x64 */ { "Д" },                   /* U+0414 cyr-De */
	/* 0x65 */ { "Е", "Ё" },              /* U+0415 cyr-Ie + U+0401 cyr-Io */
	/* 0x66 */ { "Ж" },                   /* U+0416 cyr-Zhe */
	/* 0x67 */ { "З" },                   /* U+0417 cyr-Ze */
	/* 0x68 */ { "И" },                   /* U+0418 cyr-I */
	/* 0x69 */ { "Й" },                   /* U+0419 cyr-Short-I */
	/* 0x6A */ { "К" },                   /* U+041A cyr-Ka */
	/* 0x6B */ { "Л" },                   /* U+041B cyr-El */
	/* 0x6C */ { "М" },                   /* U+041C cyr-Em */
	/* 0x6D */ { "Н" },                   /* U+041D cyr-En */
	/* 0x6E */ { "О" },                   /* U+041E cyr-O */
	/* 0x6F */ { "П" },                   /* U+041F cyr-Pe */
	/* 0x70 */ { "Р" },                   /* U+0420 cyr-Er */
	/* 0x71 */ { "С" },                   /* U+0421 cyr-Es */
	/* 0x72 */ { "Т" },                   /* U+0422 cyr-Te */
	/* 0x73 */ { "У" },                   /* U+0423 cyr-U */
	/* 0x74 */ { "Ф" },                   /* U+0424 cyr-Ef */
	/* 0x75 */ { "Х" },                   /* U+0425 cyr-Ha */
	/* 0x76 */ { "Ц" },                   /* U+0426 cyr-Tse */
	/* 0x77 */ { "Ч" },                   /* U+0427 cyr-Che */
	/* 0x78 */ { "Ш" },                   /* U+0428 cyr-Sha */
	/* 0x79 */ { "Щ" },                   /* U+0429 cyr-Shcha */
	/* 0x7A */ { "Ъ" },                   /* U+042A cyr-Hard-Sign */
	/* 0x7B */ { "Ы" },                   /* U+042B cyr-Yeru */
	/* 0x7C */ { "Ь" },                   /* U+042C cyr-Soft-Sign */
	/* 0x7D */ { "Э" },                   /* U+042D cyr-E */
	/* 0x7E */ { "Ю" },                   /* U+042E cyr-Yu */
	/* 0x7F */ { "Я" },                   /* U+042F cyr-Ya */
};

const struct pager_charset pager_charset_pocsag_philips_prg2220_latin_cyrillic = {
	.protocol  = "pocsag",
	.vendor    = "philips",
	.model     = "prg2220",
	.charset   = "latin-cyrillic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
