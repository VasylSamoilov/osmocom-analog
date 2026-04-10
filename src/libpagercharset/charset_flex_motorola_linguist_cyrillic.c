/*
 * FLEX Cyrillic codepage — Motorola FLX2 Linguist (Table A-2).
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
 * Charset:  cyrillic
 *
 * Columns 0-3 are standard ASCII (control + printable).
 * Columns 4-7 (0x40-0x7F) are replaced with Cyrillic (JCUKEN layout).
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* 0x00 */ { NULL },  /* NUL */
	/* 0x01 */ { NULL },  /* SOH */
	/* 0x02 */ { NULL },  /* STX */
	/* 0x03 */ { NULL },  /* ETX */
	/* 0x04 */ { NULL },  /* EOT */
	/* 0x05 */ { NULL },  /* ENQ */
	/* 0x06 */ { NULL },  /* ACK */
	/* 0x07 */ { NULL },  /* BEL */
	/* 0x08 */ { NULL },  /* BS  */
	/* 0x09 */ { NULL },  /* HT  */
	/* 0x0A */ { "\n" },  /* LF  */
	/* 0x0B */ { NULL },  /* VT  */
	/* 0x0C */ { NULL },  /* FF  */
	/* 0x0D */ { "\r" },  /* CR  */
	/* 0x0E */ { NULL },  /* SO  */
	/* 0x0F */ { NULL },  /* SI  */
	/* 0x10 */ { NULL },  /* DLE */
	/* 0x11 */ { NULL },  /* DC1 */
	/* 0x12 */ { NULL },  /* DC2 */
	/* 0x13 */ { NULL },  /* DC3 */
	/* 0x14 */ { NULL },  /* DC4 */
	/* 0x15 */ { NULL },  /* NAK */
	/* 0x16 */ { NULL },  /* SYN */
	/* 0x17 */ { NULL },  /* ETB */
	/* 0x18 */ { NULL },  /* CAN */
	/* 0x19 */ { NULL },  /* EM  */
	/* 0x1A */ { NULL },  /* SUB */
	/* 0x1B */ { NULL },  /* ESC */
	/* 0x1C */ { NULL },  /* FS  */
	/* 0x1D */ { NULL },  /* GS  */
	/* 0x1E */ { NULL },  /* RS  */
	/* 0x1F */ { NULL },  /* US  */
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
	/* 0x40 */ { "ю" },           /* U+044E */
	/* 0x41 */ { "а" },           /* U+0430 */
	/* 0x42 */ { "б" },           /* U+0431 */
	/* 0x43 */ { "ц" },           /* U+0446 */
	/* 0x44 */ { "д" },           /* U+0434 */
	/* 0x45 */ { "е" },           /* U+0435 */
	/* 0x46 */ { "ф" },           /* U+0444 */
	/* 0x47 */ { "г" },           /* U+0433 */
	/* 0x48 */ { "х" },           /* U+0445 */
	/* 0x49 */ { "и" },           /* U+0438 */
	/* 0x4A */ { "й" },           /* U+0439 */
	/* 0x4B */ { "к" },           /* U+043A */
	/* 0x4C */ { "л" },           /* U+043B */
	/* 0x4D */ { "м" },           /* U+043C */
	/* 0x4E */ { "н" },           /* U+043D */
	/* 0x4F */ { "о" },           /* U+043E */
	/* 0x50 */ { "п" },           /* U+043F */
	/* 0x51 */ { "я" },           /* U+044F */
	/* 0x52 */ { "р" },           /* U+0440 */
	/* 0x53 */ { "с" },           /* U+0441 */
	/* 0x54 */ { "т" },           /* U+0442 */
	/* 0x55 */ { "у" },           /* U+0443 */
	/* 0x56 */ { "ж" },           /* U+0436 */
	/* 0x57 */ { "в" },           /* U+0432 */
	/* 0x58 */ { "ь" },           /* U+044C */
	/* 0x59 */ { "ы" },           /* U+044B */
	/* 0x5A */ { "з" },           /* U+0437 */
	/* 0x5B */ { "ш" },           /* U+0448 */
	/* 0x5C */ { "э" },           /* U+044D */
	/* 0x5D */ { "щ" },           /* U+0449 */
	/* 0x5E */ { "ч" },           /* U+0447 */
	/* 0x5F */ { "_" },
	/* 0x60 */ { "Ю" },           /* U+042E */
	/* 0x61 */ { "А" },           /* U+0410 */
	/* 0x62 */ { "Б" },           /* U+0411 */
	/* 0x63 */ { "Ц" },           /* U+0426 */
	/* 0x64 */ { "Д" },           /* U+0414 */
	/* 0x65 */ { "Е" },           /* U+0415 */
	/* 0x66 */ { "Ф" },           /* U+0424 */
	/* 0x67 */ { "Г" },           /* U+0413 */
	/* 0x68 */ { "Х" },           /* U+0425 */
	/* 0x69 */ { "И" },           /* U+0418 */
	/* 0x6A */ { "Й" },           /* U+0419 */
	/* 0x6B */ { "К" },           /* U+041A */
	/* 0x6C */ { "Л" },           /* U+041B */
	/* 0x6D */ { "М" },           /* U+041C */
	/* 0x6E */ { "Н" },           /* U+041D */
	/* 0x6F */ { "О" },           /* U+041E */
	/* 0x70 */ { "П" },           /* U+041F */
	/* 0x71 */ { "Я" },           /* U+042F */
	/* 0x72 */ { "Р" },           /* U+0420 */
	/* 0x73 */ { "С" },           /* U+0421 */
	/* 0x74 */ { "Т" },           /* U+0422 */
	/* 0x75 */ { "У" },           /* U+0423 */
	/* 0x76 */ { "Ж" },           /* U+0416 */
	/* 0x77 */ { "В" },           /* U+0412 */
	/* 0x78 */ { "Ь" },           /* U+042C */
	/* 0x79 */ { "Ы" },           /* U+042B */
	/* 0x7A */ { "З" },           /* U+0417 */
	/* 0x7B */ { "Ш" },           /* U+0428 */
	/* 0x7C */ { "Э" },           /* U+042D */
	/* 0x7D */ { "Щ" },           /* U+0429 */
	/* 0x7E */ { "Ч" },           /* U+0427 */
	/* 0x7F */ { NULL },  /* DEL */
};

const struct pager_charset pager_charset_flex_motorola_linguist_cyrillic = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "cyrillic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
