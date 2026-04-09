/*
 * POCSAG Motorola Advisor full Cyrillic codepage.
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
 * Vendor:   motorola
 * Model:    advisor
 * Charset:  cyrillic
 *
 * Verified against real pager hardware.
 * Pagers: Motorola Advisor, Scriptor LX1
 *
 * Layout:
 *   0x20-0x3F: Standard ASCII punctuation and digits.
 *   0x40-0x5E: Cyrillic lowercase (phonetic order, not alphabetic).
 *   0x5F:      underscore.
 *   0x60-0x7E: Cyrillic uppercase (same phonetic order).
 *
 * Aliases where two codepoints map to the same 7-bit code:
 *   0x45: е + ё
 *   0x58: ъ + ь
 *   0x65: Е + Ё
 *   0x78: Ъ + Ь
 * No invented cross-case folding.
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

	/* 0x20-0x3F: Standard ASCII punctuation and digits */
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

	/* 0x40-0x5F: Cyrillic lowercase (phonetic order) */
	/* 0x40 */ { "ю" },                   /* U+044E cyr-yu */
	/* 0x41 */ { "а" },                   /* U+0430 cyr-a */
	/* 0x42 */ { "б" },                   /* U+0431 cyr-be */
	/* 0x43 */ { "ц" },                   /* U+0446 cyr-tse */
	/* 0x44 */ { "д" },                   /* U+0434 cyr-de */
	/* 0x45 */ { "е", "ё" },              /* U+0435 cyr-ie + U+0451 cyr-io */
	/* 0x46 */ { "ф" },                   /* U+0444 cyr-ef */
	/* 0x47 */ { "г" },                   /* U+0433 cyr-ghe */
	/* 0x48 */ { "х" },                   /* U+0445 cyr-kha */
	/* 0x49 */ { "и" },                   /* U+0438 cyr-i */
	/* 0x4A */ { "й" },                   /* U+0439 cyr-short-i */
	/* 0x4B */ { "к" },                   /* U+043A cyr-ka */
	/* 0x4C */ { "л" },                   /* U+043B cyr-el */
	/* 0x4D */ { "м" },                   /* U+043C cyr-em */
	/* 0x4E */ { "н" },                   /* U+043D cyr-en */
	/* 0x4F */ { "о" },                   /* U+043E cyr-o */
	/* 0x50 */ { "п" },                   /* U+043F cyr-pe */
	/* 0x51 */ { "я" },                   /* U+044F cyr-ya */
	/* 0x52 */ { "р" },                   /* U+0440 cyr-er */
	/* 0x53 */ { "с" },                   /* U+0441 cyr-es */
	/* 0x54 */ { "т" },                   /* U+0442 cyr-te */
	/* 0x55 */ { "у" },                   /* U+0443 cyr-u */
	/* 0x56 */ { "ж" },                   /* U+0436 cyr-zhe */
	/* 0x57 */ { "в" },                   /* U+0432 cyr-ve */
	/* 0x58 */ { "ъ", "ь" },              /* U+044A cyr-hard-sign + U+044C cyr-soft-sign */
	/* 0x59 */ { "ы" },                   /* U+044B cyr-yeru */
	/* 0x5A */ { "з" },                   /* U+0437 cyr-ze */
	/* 0x5B */ { "ш" },                   /* U+0448 cyr-sha */
	/* 0x5C */ { "э" },                   /* U+044D cyr-e */
	/* 0x5D */ { "щ" },                   /* U+0449 cyr-shcha */
	/* 0x5E */ { "ч" },                   /* U+0447 cyr-che */
	/* 0x5F */ { "_" },

	/* 0x60-0x7F: Cyrillic uppercase (same phonetic order) */
	/* 0x60 */ { "Ю" },                   /* U+042E cyr-Yu */
	/* 0x61 */ { "А" },                   /* U+0410 cyr-A */
	/* 0x62 */ { "Б" },                   /* U+0411 cyr-Be */
	/* 0x63 */ { "Ц" },                   /* U+0426 cyr-Tse */
	/* 0x64 */ { "Д" },                   /* U+0414 cyr-De */
	/* 0x65 */ { "Е", "Ё" },              /* U+0415 cyr-Ie + U+0401 cyr-Io */
	/* 0x66 */ { "Ф" },                   /* U+0424 cyr-Ef */
	/* 0x67 */ { "Г" },                   /* U+0413 cyr-Ghe */
	/* 0x68 */ { "Х" },                   /* U+0425 cyr-Ha */
	/* 0x69 */ { "И" },                   /* U+0418 cyr-I */
	/* 0x6A */ { "Й" },                   /* U+0419 cyr-Short-I */
	/* 0x6B */ { "К" },                   /* U+041A cyr-Ka */
	/* 0x6C */ { "Л" },                   /* U+041B cyr-El */
	/* 0x6D */ { "М" },                   /* U+041C cyr-Em */
	/* 0x6E */ { "Н" },                   /* U+041D cyr-En */
	/* 0x6F */ { "О" },                   /* U+041E cyr-O */
	/* 0x70 */ { "П" },                   /* U+041F cyr-Pe */
	/* 0x71 */ { "Я" },                   /* U+042F cyr-Ya */
	/* 0x72 */ { "Р" },                   /* U+0420 cyr-Er */
	/* 0x73 */ { "С" },                   /* U+0421 cyr-Es */
	/* 0x74 */ { "Т" },                   /* U+0422 cyr-Te */
	/* 0x75 */ { "У" },                   /* U+0423 cyr-U */
	/* 0x76 */ { "Ж" },                   /* U+0416 cyr-Zhe */
	/* 0x77 */ { "В" },                   /* U+0412 cyr-Ve */
	/* 0x78 */ { "Ъ", "Ь" },              /* U+042A cyr-Hard-Sign + U+042C cyr-Soft-Sign */
	/* 0x79 */ { "Ы" },                   /* U+042B cyr-Yeru */
	/* 0x7A */ { "З" },                   /* U+0417 cyr-Ze */
	/* 0x7B */ { "Ш" },                   /* U+0428 cyr-Sha */
	/* 0x7C */ { "Э" },                   /* U+042D cyr-E */
	/* 0x7D */ { "Щ" },                   /* U+0429 cyr-Shcha */
	/* 0x7E */ { "Ч" },                   /* U+0427 cyr-Che */
	/* 0x7F */ { NULL },                  /* DEL */
};

const struct pager_charset pager_charset_pocsag_motorola_advisor_cyrillic = {
	.protocol  = "pocsag",
	.vendor    = "motorola",
	.model     = "advisor",
	.charset   = "cyrillic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
