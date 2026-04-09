/*
 * POCSAG NEC 21A Skyper Latin/Cyrillic codepage.
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
 * Vendor:   nec
 * Model:    21a
 * Charset:  latin-cyrillic-uppercase
 *
 * Verified against real pager hardware.
 *
 * Uppercase-only codepage containing both Latin and Cyrillic characters.
 * The codepage has no lowercase glyphs — all letters are uppercase.
 * Similar to Motorola Advisor Linguist but with key differences:
 *   - Serbian letters (Ѝ,Љ,Њ,Ђ,Ќ,Ў,Џ) in control region 0x10-0x16
 *     instead of symbol positions 0x5C/0x5E/0x5F/0x7B-0x7D.
 *   - 0x5C,0x5E,0x5F,0x7B-0x7D are standard ASCII (\,^,_,{,|,}).
 *   - No special symbols in control region (no ¤,¢,×,÷,£,°, etc.).
 *   - 0x10: Ѝ (U+040D) — not present in Linguist codepage.
 *   - Ђ appears at both 0x13 and 0x78.
 *
 * The codepage has no lowercase glyphs — all letters are uppercase.
 * Latin primary where lookalike.
 * Lowercase maps to uppercase.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00-0x1F: Control region — Serbian/Macedonian letters at 0x10-0x16 */
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
	/* 0x10 */ { "Ѝ", "ѝ" },                                  /* U+040D cyr-I-grave, U+045D cyr-i-grave */
	/* 0x11 */ { "Љ", "љ" },                                  /* U+0409 cyr-Lje, U+0459 cyr-lje */
	/* 0x12 */ { "Њ", "њ" },                                  /* U+040A cyr-Nje, U+045A cyr-nje */
	/* 0x13 */ { "Ђ", "ђ" },                                  /* U+0402 cyr-Dje, U+0452 cyr-dje */
	/* 0x14 */ { "Ќ", "ќ" },                                  /* U+040C cyr-Kje, U+045C cyr-kje */
	/* 0x15 */ { "Ў", "ў" },                                  /* U+040E cyr-Short-U, U+045E cyr-short-u */
	/* 0x16 */ { "Џ", "џ" },                                  /* U+040F cyr-Dzhe, U+045F cyr-dzhe */
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
	/* 0x27 */ { "'", "`" },                                  /* U+0027 apostrophe + U+0060 backtick */
	/* 0x28 */ { "(" },
	/* 0x29 */ { ")" },
	/* 0x2A */ { "*" },
	/* 0x2B */ { "+" },
	/* 0x2C */ { "," },
	/* 0x2D */ { "-", "—" },                                  /* U+002D hyphen + U+2014 em-dash */
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

	/* 0x40-0x5F: Uppercase Latin/Cyrillic lookalikes + standard ASCII symbols.
	 * Latin primary. Lowercase aliases for .upper() behavior. */
	/* 0x40 */ { "@" },                                       /* U+0040 */
	/* 0x41 */ { "A", "А", "a", "а" },                       /* U+0041 lat-A, U+0410 cyr-A, U+0061 lat-a, U+0430 cyr-a */
	/* 0x42 */ { "B", "В", "b", "в" },                       /* U+0042 lat-B, U+0412 cyr-Ve, U+0062 lat-b, U+0432 cyr-ve */
	/* 0x43 */ { "C", "С", "c", "с" },                       /* U+0043 lat-C, U+0421 cyr-Es, U+0063 lat-c, U+0441 cyr-es */
	/* 0x44 */ { "D", "d" },                                  /* U+0044 lat-D, U+0064 lat-d */
	/* 0x45 */ { "E", "Е", "e", "е" },                       /* U+0045 lat-E, U+0415 cyr-Ie, U+0065 lat-e, U+0435 cyr-ie */
	/* 0x46 */ { "F", "f" },                                  /* U+0046 lat-F, U+0066 lat-f */
	/* 0x47 */ { "G", "g" },                                  /* U+0047 lat-G, U+0067 lat-g */
	/* 0x48 */ { "H", "Н", "h", "н" },                       /* U+0048 lat-H, U+041D cyr-En, U+0068 lat-h, U+043D cyr-en */
	/* 0x49 */ { "I", "І", "i", "і", "ɪ" },                  /* U+0049 lat-I, U+0406 cyr-I-ukr, U+0069 lat-i, U+0456 cyr-i-ukr, U+026A lat-small-cap-I */
	/* 0x4A */ { "J", "j" },                                  /* U+004A lat-J, U+006A lat-j */
	/* 0x4B */ { "K", "К", "k", "к" },                       /* U+004B lat-K, U+041A cyr-Ka, U+006B lat-k, U+043A cyr-ka */
	/* 0x4C */ { "L", "l" },                                  /* U+004C lat-L, U+006C lat-l */
	/* 0x4D */ { "M", "М", "m", "м" },                       /* U+004D lat-M, U+041C cyr-Em, U+006D lat-m, U+043C cyr-em */
	/* 0x4E */ { "N", "n" },                                  /* U+004E lat-N, U+006E lat-n */
	/* 0x4F */ { "O", "О", "o", "о" },                       /* U+004F lat-O, U+041E cyr-O, U+006F lat-o, U+043E cyr-o */
	/* 0x50 */ { "P", "Р", "p", "р" },                       /* U+0050 lat-P, U+0420 cyr-Er, U+0070 lat-p, U+0440 cyr-er */
	/* 0x51 */ { "Q", "q" },                                  /* U+0051 lat-Q, U+0071 lat-q */
	/* 0x52 */ { "R", "r" },                                  /* U+0052 lat-R, U+0072 lat-r */
	/* 0x53 */ { "S", "s" },                                  /* U+0053 lat-S, U+0073 lat-s */
	/* 0x54 */ { "T", "Т", "t", "т" },                       /* U+0054 lat-T, U+0422 cyr-Te, U+0074 lat-t, U+0442 cyr-te */
	/* 0x55 */ { "U", "u" },                                  /* U+0055 lat-U, U+0075 lat-u */
	/* 0x56 */ { "V", "v" },                                  /* U+0056 lat-V, U+0076 lat-v */
	/* 0x57 */ { "W", "w" },                                  /* U+0057 lat-W, U+0077 lat-w */
	/* 0x58 */ { "X", "Х", "x", "х" },                       /* U+0058 lat-X, U+0425 cyr-Ha, U+0078 lat-x, U+0445 cyr-ha */
	/* 0x59 */ { "Y", "y" },                                  /* U+0059 lat-Y, U+0079 lat-y */
	/* 0x5A */ { "Z", "z" },                                  /* U+005A lat-Z, U+007A lat-z */
	/* 0x5B */ { "[" },                                       /* U+005B */
	/* 0x5C */ { "\\" },                                      /* U+005C — standard ASCII (not Љ like Linguist) */
	/* 0x5D */ { "]" },                                       /* U+005D */
	/* 0x5E */ { "^" },                                       /* U+005E — standard ASCII (not Њ like Linguist) */
	/* 0x5F */ { "_" },                                       /* U+005F — standard ASCII (not Ћ like Linguist) */

	/* 0x60-0x7F: Cyrillic letters + standard ASCII symbols at 0x7B-0x7D.
	 * Lowercase Cyrillic aliases for .upper() behavior. */
	/* 0x60 */ { "`" },                                       /* U+0060 */
	/* 0x61 */ { "Б", "б" },                                  /* U+0411 cyr-Be, U+0431 cyr-be */
	/* 0x62 */ { "Г", "г" },                                  /* U+0413 cyr-Ghe, U+0433 cyr-ghe */
	/* 0x63 */ { "Ѓ", "Ґ", "ѓ", "ґ" },                       /* U+0403 cyr-Gje, U+0490 cyr-Ghe-upturn, U+0453 cyr-gje, U+0491 cyr-ghe-upturn */
	/* 0x64 */ { "Д", "д" },                                  /* U+0414 cyr-De, U+0434 cyr-de */
	/* 0x65 */ { "Ё", "ё" },                                  /* U+0401 cyr-Io, U+0451 cyr-io */
	/* 0x66 */ { "Ж", "ж" },                                  /* U+0416 cyr-Zhe, U+0436 cyr-zhe */
	/* 0x67 */ { "З", "з" },                                  /* U+0417 cyr-Ze, U+0437 cyr-ze */
	/* 0x68 */ { "И", "и" },                                  /* U+0418 cyr-I, U+0438 cyr-i */
	/* 0x69 */ { "Й", "й" },                                  /* U+0419 cyr-Short-I, U+0439 cyr-short-i */
	/* 0x6A */ { "Л", "л" },                                  /* U+041B cyr-El, U+043B cyr-el */
	/* 0x6B */ { "П", "п" },                                  /* U+041F cyr-Pe, U+043F cyr-pe */
	/* 0x6C */ { "У", "у" },                                  /* U+0423 cyr-U, U+0443 cyr-u */
	/* 0x6D */ { "Ф", "ф" },                                  /* U+0424 cyr-Ef, U+0444 cyr-ef */
	/* 0x6E */ { "Ц", "ц" },                                  /* U+0426 cyr-Tse, U+0446 cyr-tse */
	/* 0x6F */ { "Ч", "ч" },                                  /* U+0427 cyr-Che, U+0447 cyr-che */
	/* 0x70 */ { "Ш", "ш" },                                  /* U+0428 cyr-Sha, U+0448 cyr-sha */
	/* 0x71 */ { "Щ", "щ" },                                  /* U+0429 cyr-Shcha, U+0449 cyr-shcha */
	/* 0x72 */ { "Ъ", "ъ" },                                  /* U+042A cyr-Hard-Sign, U+044A cyr-hard-sign */
	/* 0x73 */ { "Ы", "ы" },                                  /* U+042B cyr-Yeru, U+044B cyr-yeru */
	/* 0x74 */ { "Ь", "ь" },                                  /* U+042C cyr-Soft-Sign, U+044C cyr-soft-sign */
	/* 0x75 */ { "Э", "э" },                                  /* U+042D cyr-E, U+044D cyr-e */
	/* 0x76 */ { "Ю", "ю" },                                  /* U+042E cyr-Yu, U+044E cyr-yu */
	/* 0x77 */ { "Я", "я" },                                  /* U+042F cyr-Ya, U+044F cyr-ya */
	/* 0x78 */ { "Ђ", "ђ" },                                  /* U+0402 cyr-Dje, U+0452 cyr-dje — also at 0x13 */
	/* 0x79 */ { "Є", "є" },                                  /* U+0404 cyr-Ie-ukr, U+0454 cyr-ie-ukr */
	/* 0x7A */ { "Ї", "Ï", "ї", "ï" },                       /* U+0407 cyr-Yi, U+00CF lat-I-diaeresis, U+0457 cyr-yi, U+00EF lat-i-diaeresis */
	/* 0x7B */ { "{" },                                       /* U+007B — standard ASCII (not Ќ like Linguist) */
	/* 0x7C */ { "|" },                                       /* U+007C — standard ASCII (not Ў like Linguist) */
	/* 0x7D */ { "}" },                                       /* U+007D — standard ASCII (not Џ like Linguist) */
	/* 0x7E */ { "~" },                                       /* U+007E */
	/* 0x7F */ { NULL },
};

const struct pager_charset pager_charset_pocsag_nec_21a_latin_cyrillic = {
	.protocol  = "pocsag",
	.vendor    = "nec",
	.model     = "21a",
	.charset   = "latin-cyrillic-uppercase",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
