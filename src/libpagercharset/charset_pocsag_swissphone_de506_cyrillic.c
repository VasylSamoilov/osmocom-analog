/*
 * POCSAG SwissPhone DE506 Cyrillic codepage.
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
 * Vendor:   swissphone
 * Model:    de506
 * Charset:  cyrillic
 *
 * Pagers: SwissPhone DE506 Prestige (Cyrillic firmware).
 * Also used by: Partner pagers (identical codepage).
 *
 * Cyrillic replaces Latin letters entirely. No Latin A-Z available.
 * Uppercase А-Я mapped to 0x41-0x5A (same positions as Latin A-Z).
 * Lowercase а-я mapped to 0x61-0x7A (same positions as Latin a-z).
 * Special overflow characters use punctuation positions:
 *   Ь→0x5E(^), Э→0x40(@), Ю→0x5F(_), Я→0x23(#) for uppercase.
 *   ь→0x7E(~), э→0x60(`), ю→0x7F(DEL), я→0x24($) for lowercase.
 *
 * Layout:
 *   0x20-0x3F: Digits and some punctuation (partial ASCII).
 *   0x40-0x5F: Cyrillic uppercase А-Я (replacing Latin A-Z).
 *   0x60-0x7F: Cyrillic lowercase а-я (replacing Latin a-z).
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00-0x1F: Control region */
	/* 0x00 */ { NULL }, /* 0x01 */ { NULL }, /* 0x02 */ { NULL }, /* 0x03 */ { NULL },
	/* 0x04 */ { NULL }, /* 0x05 */ { NULL }, /* 0x06 */ { NULL }, /* 0x07 */ { NULL },
	/* 0x08 */ { NULL }, /* 0x09 */ { NULL }, /* 0x0A */ { "\n" }, /* 0x0B */ { NULL },
	/* 0x0C */ { NULL }, /* 0x0D */ { "\r" }, /* 0x0E */ { NULL }, /* 0x0F */ { NULL },
	/* 0x10 */ { NULL }, /* 0x11 */ { NULL }, /* 0x12 */ { NULL }, /* 0x13 */ { NULL },
	/* 0x14 */ { NULL }, /* 0x15 */ { NULL }, /* 0x16 */ { NULL }, /* 0x17 */ { NULL },
	/* 0x18 */ { NULL }, /* 0x19 */ { NULL }, /* 0x1A */ { NULL }, /* 0x1B */ { NULL },
	/* 0x1C */ { NULL }, /* 0x1D */ { NULL }, /* 0x1E */ { NULL }, /* 0x1F */ { NULL },

	/* 0x20-0x3F: Digits and partial punctuation */
	/* 0x20 */ { " " },
	/* 0x21 */ { "!" },
	/* 0x22 */ { "\"" },
	/* 0x23 */ { "Я", "я" },              /* U+042F/U+044F — Я replaces # */
	/* 0x24 */ { NULL },                  /* $ not available (lowercase я uses this in TX) */
	/* 0x25 */ { "%" },
	/* 0x26 */ { "&" },
	/* 0x27 */ { NULL },                  /* ' replaced by . in KCC xlat7 */
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
	/* 0x3B */ { NULL },                  /* ; not available */
	/* 0x3C */ { NULL },                  /* < not available */
	/* 0x3D */ { "=" },
	/* 0x3E */ { NULL },                  /* > not available */
	/* 0x3F */ { "?" },

	/* 0x40-0x5F: Cyrillic uppercase А-Щ sequential + overflow */
	/* 0x40 */ { "Э", "э" },              /* U+042D/U+044D — Э replaces @ */
	/* 0x41 */ { "А", "а" },              /* U+0410/U+0430 */
	/* 0x42 */ { "Б", "б" },              /* U+0411/U+0431 */
	/* 0x43 */ { "В", "в" },              /* U+0412/U+0432 */
	/* 0x44 */ { "Г", "г" },              /* U+0413/U+0433 */
	/* 0x45 */ { "Д", "д" },              /* U+0414/U+0434 */
	/* 0x46 */ { "Е", "е", "Ё", "ё" },   /* U+0415/U+0435 + U+0401/U+0451 */
	/* 0x47 */ { "Ж", "ж" },              /* U+0416/U+0436 */
	/* 0x48 */ { "З", "з" },              /* U+0417/U+0437 */
	/* 0x49 */ { "И", "и" },              /* U+0418/U+0438 */
	/* 0x4A */ { "Й", "й" },              /* U+0419/U+0439 */
	/* 0x4B */ { "К", "к" },              /* U+041A/U+043A */
	/* 0x4C */ { "Л", "л" },              /* U+041B/U+043B */
	/* 0x4D */ { "М", "м" },              /* U+041C/U+043C */
	/* 0x4E */ { "Н", "н" },              /* U+041D/U+043D */
	/* 0x4F */ { "О", "о" },              /* U+041E/U+043E */
	/* 0x50 */ { "П", "п" },              /* U+041F/U+043F */
	/* 0x51 */ { "Р", "р" },              /* U+0420/U+0440 */
	/* 0x52 */ { "С", "с" },              /* U+0421/U+0441 */
	/* 0x53 */ { "Т", "т" },              /* U+0422/U+0442 */
	/* 0x54 */ { "У", "у" },              /* U+0423/U+0443 */
	/* 0x55 */ { "Ф", "ф" },              /* U+0424/U+0444 */
	/* 0x56 */ { "Х", "х" },              /* U+0425/U+0445 */
	/* 0x57 */ { "Ц", "ц" },              /* U+0426/U+0446 */
	/* 0x58 */ { "Ч", "ч" },              /* U+0427/U+0447 */
	/* 0x59 */ { "Ш", "ш" },              /* U+0428/U+0448 */
	/* 0x5A */ { "Щ", "щ" },              /* U+0429/U+0449 */
	/* 0x5B */ { "Ъ", "ъ" },              /* U+042A/U+044A */
	/* 0x5C */ { "Ы", "ы" },              /* U+042B/U+044B */
	/* 0x5D */ { "]" },
	/* 0x5E */ { "Ь", "ь" },              /* U+042C/U+044C — replaces ^ */
	/* 0x5F */ { "Ю", "ю" },              /* U+042E/U+044E — replaces _ */

	/* 0x60-0x7F: Cyrillic lowercase а-щ sequential + overflow */
	/* 0x60 */ { NULL },                  /* lowercase э uses this in TX encoding */
	/* 0x61 */ { NULL },                  /* lowercase а — use 0x41 */
	/* 0x62 */ { NULL },                  /* lowercase б — use 0x42 */
	/* 0x63 */ { NULL },                  /* etc. */
	/* 0x64 */ { NULL },
	/* 0x65 */ { NULL },
	/* 0x66 */ { NULL },
	/* 0x67 */ { NULL },
	/* 0x68 */ { NULL },
	/* 0x69 */ { NULL },
	/* 0x6A */ { NULL },
	/* 0x6B */ { NULL },
	/* 0x6C */ { NULL },
	/* 0x6D */ { NULL },
	/* 0x6E */ { NULL },
	/* 0x6F */ { NULL },
	/* 0x70 */ { NULL },
	/* 0x71 */ { NULL },
	/* 0x72 */ { NULL },
	/* 0x73 */ { NULL },
	/* 0x74 */ { NULL },
	/* 0x75 */ { NULL },
	/* 0x76 */ { NULL },
	/* 0x77 */ { NULL },
	/* 0x78 */ { NULL },
	/* 0x79 */ { NULL },
	/* 0x7A */ { NULL },
	/* 0x7B */ { "{" },
	/* 0x7C */ { "|" },
	/* 0x7D */ { "}" },
	/* 0x7E */ { NULL },                  /* lowercase ь uses this in TX */
	/* 0x7F */ { NULL },                  /* lowercase ю uses this in TX */
};

/*
 * Note: The TX encoding defines a lowercase Cyrillic range at 0x61-0x7A.
 * However, the SwissPhone DE506 Cyrillic firmware displays uppercase Cyrillic at positions 0x41-0x5A. The
 * lowercase range 0x61-0x7A on the pager display shows the same
 * uppercase glyphs (the pager ROM has no lowercase Cyrillic).
 * For RX decode, only the 0x40-0x5F range is meaningful.
 */

const struct pager_charset pager_charset_pocsag_swissphone_de506_cyrillic = {
	.protocol  = "pocsag",
	.vendor    = "swissphone",
	.model     = "de506",
	.charset   = "cyrillic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};

/* Partner pagers use the identical codepage */
const struct pager_charset pager_charset_pocsag_partner_generic_cyrillic = {
	.protocol  = "pocsag",
	.vendor    = "partner",
	.model     = "generic",
	.charset   = "cyrillic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
