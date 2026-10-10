/*
 * FLEX Vietnamese codepage — Motorola FLX2 Linguist (Table A-7).
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
 * Charset:  vietnamese
 *
 * Re-transcribed from the clean rendered manual page (pages/17A_arabic_A5-166.png,
 * Table A-7) at high zoom.  Control column 0 interleaves control codes with Latin
 * letters/symbols; columns 2,3,5,6,7 carry the Vietnamese vowels with the full
 * tone+quality diacritic matrix (A/Â/Ă groups, E/Ê, O/Ô/Ơ, U/Ư, I, Y).  Mapped to
 * precomposed Unicode (Latin-1 Supplement + Latin Extended-A/B + Latin Extended
 * Additional, U+1Exx).
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* Column 0 (0x00-0x0F) — control + Latin/symbols */
	/* 0x00 */ { NULL },           /* NUL */
	/* 0x01 */ { "O" },
	/* 0x02 */ { "-" },
	/* 0x03 */ { "+" },
	/* 0x04 */ { NULL },           /* EOT */
	/* 0x05 */ { "I" },
	/* 0x06 */ { "=" },
	/* 0x07 */ { "S" },
	/* 0x08 */ { "$" },
	/* 0x09 */ { "B" },
	/* 0x0A */ { "\n" },           /* LF */
	/* 0x0B */ { "G" },
	/* 0x0C */ { "." },
	/* 0x0D */ { "\r" },           /* CR */
	/* 0x0E */ { NULL },           /* SO */
	/* 0x0F */ { NULL },           /* SI */
	/* Column 1 (0x10-0x1F) — Latin/symbols + control */
	/* 0x10 */ { "F" },
	/* 0x11 */ { "%" },
	/* 0x12 */ { "J" },
	/* 0x13 */ { "“" },            /* U+201C left double quotation mark */
	/* 0x14 */ { "W" },
	/* 0x15 */ { "!" },
	/* 0x16 */ { "Z" },
	/* 0x17 */ { "*" },
	/* 0x18 */ { "Q" },
	/* 0x19 */ { ")" },
	/* 0x1A */ { "(" },
	/* 0x1B */ { NULL },           /* ESC */
	/* 0x1C */ { "?" },
	/* 0x1D */ { "," },
	/* 0x1E */ { "#" },
	/* 0x1F */ { "/" },
	/* Column 2 (0x20-0x2F) — A / Â / Ă groups */
	/* 0x20 */ { " " },            /* SP */
	/* 0x21 */ { "Â" },           /* U+00C2 */
	/* 0x22 */ { "Ă" },           /* U+0102 */
	/* 0x23 */ { "Ả" },           /* U+1EA2 A + hook above */
	/* 0x24 */ { "Ã" },           /* U+00C3 */
	/* 0x25 */ { "À" },           /* U+00C0 */
	/* 0x26 */ { "Á" },           /* U+00C1 */
	/* 0x27 */ { "Ạ" },           /* U+1EA0 A + dot below */
	/* 0x28 */ { "Ẩ" },           /* U+1EA8 Â + hook above */
	/* 0x29 */ { "Ẫ" },           /* U+1EAA Â + tilde */
	/* 0x2A */ { "Ấ" },           /* U+1EA4 Â + acute */
	/* 0x2B */ { "Ầ" },           /* U+1EA6 Â + grave */
	/* 0x2C */ { "Ậ" },           /* U+1EAC Â + dot below */
	/* 0x2D */ { "Ẳ" },           /* U+1EB2 Ă + hook above */
	/* 0x2E */ { "Ẵ" },           /* U+1EB4 Ă + tilde */
	/* 0x2F */ { "Ắ" },           /* U+1EAE Ă + acute */
	/* Column 3 (0x30-0x3F) — digits + Ă/Ê/E groups */
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
	/* 0x3B */ { "Ặ" },           /* U+1EB6 Ă + dot below */
	/* 0x3C */ { "Ê" },           /* U+00CA */
	/* 0x3D */ { "Ẻ" },           /* U+1EBA E + hook above */
	/* 0x3E */ { "Ẽ" },           /* U+1EBC E + tilde */
	/* 0x3F */ { "É" },           /* U+00C9 */
	/* Column 4 (0x40-0x4F) — Latin A-N + E/Ê/Đ/Y groups */
	/* 0x40 */ { "È" },           /* U+00C8 */
	/* 0x41 */ { "A" },
	/* 0x42 */ { "B" },
	/* 0x43 */ { "C" },
	/* 0x44 */ { "D" },
	/* 0x45 */ { "E" },
	/* 0x46 */ { "Đ" },           /* U+0110 D with stroke */
	/* 0x47 */ { "Ỷ" },           /* U+1EF6 Y + hook above */
	/* 0x48 */ { "H" },
	/* 0x49 */ { "Ẹ" },           /* U+1EB8 E + dot below */
	/* 0x4A */ { "Ể" },           /* U+1EC2 Ê + hook above */
	/* 0x4B */ { "K" },
	/* 0x4C */ { "L" },
	/* 0x4D */ { "M" },
	/* 0x4E */ { "N" },
	/* 0x4F */ { "Ễ" },           /* U+1EC4 Ê + tilde */
	/* Column 5 (0x50-0x5F) — Latin P-Y + Ê/Ô/O groups */
	/* 0x50 */ { "P" },
	/* 0x51 */ { "Ằ" },           /* U+1EB0 Ă + grave */
	/* 0x52 */ { "R" },
	/* 0x53 */ { "Ý" },           /* U+00DD */
	/* 0x54 */ { "T" },
	/* 0x55 */ { "U" },
	/* 0x56 */ { "V" },
	/* 0x57 */ { "Ỹ" },           /* U+1EF8 Y + tilde */
	/* 0x58 */ { "X" },
	/* 0x59 */ { "Y" },
	/* 0x5A */ { "Ế" },           /* U+1EBE Ê + acute */
	/* 0x5B */ { "Ề" },           /* U+1EC0 Ê + grave */
	/* 0x5C */ { "Ệ" },           /* U+1EC6 Ê + dot below */
	/* 0x5D */ { "Ô" },           /* U+00D4 */
	/* 0x5E */ { "Ỏ" },           /* U+1ECE O + hook above */
	/* 0x5F */ { "Õ" },           /* U+00D5 */
	/* Column 6 (0x60-0x6F) — O / Ô / Ơ groups */
	/* 0x60 */ { "Ó" },           /* U+00D3 */
	/* 0x61 */ { "Ò" },           /* U+00D2 */
	/* 0x62 */ { "Ọ" },           /* U+1ECC O + dot below */
	/* 0x63 */ { "Ổ" },           /* U+1ED4 Ô + hook above */
	/* 0x64 */ { "Ỗ" },           /* U+1ED6 Ô + tilde */
	/* 0x65 */ { "Ố" },           /* U+1ED0 Ô + acute */
	/* 0x66 */ { "Ồ" },           /* U+1ED2 Ô + grave */
	/* 0x67 */ { "Ộ" },           /* U+1ED8 Ô + dot below */
	/* 0x68 */ { "ơ" },           /* U+01A1 small o with horn */
	/* 0x69 */ { "Ở" },           /* U+1EDE Ơ + hook above */
	/* 0x6A */ { "Õ" },           /* U+00D5 */
	/* 0x6B */ { "Ớ" },           /* U+1EDA Ơ + acute */
	/* 0x6C */ { "Ờ" },           /* U+1EDC Ơ + grave */
	/* 0x6D */ { "Ợ" },           /* U+1EE2 Ơ + dot below */
	/* 0x6E */ { "Ư" },           /* U+01AF U with horn */
	/* 0x6F */ { "Ủ" },           /* U+1EE6 U + hook above */
	/* Column 7 (0x70-0x7F) — U / Ư + I / Y groups */
	/* 0x70 */ { "Ũ" },           /* U+0168 */
	/* 0x71 */ { "Ú" },           /* U+00DA */
	/* 0x72 */ { "Ù" },           /* U+00D9 */
	/* 0x73 */ { "Ụ" },           /* U+1EE4 U + dot below */
	/* 0x74 */ { "Ủ" },           /* U+1EE6 U + hook above */
	/* 0x75 */ { "Ũ" },           /* U+0168 U + tilde */
	/* 0x76 */ { "Ú" },           /* U+00DA U + acute */
	/* 0x77 */ { "Ù" },           /* U+00D9 U + grave */
	/* 0x78 */ { "Ụ" },           /* U+1EE4 U + dot below */
	/* 0x79 */ { "Ỉ" },           /* U+1EC8 I + hook above */
	/* 0x7A */ { "Ĩ" },           /* U+0128 */
	/* 0x7B */ { "Ì" },           /* U+00CC */
	/* 0x7C */ { "Í" },           /* U+00CD */
	/* 0x7D */ { "Ị" },           /* U+1ECA I + dot below */
	/* 0x7E */ { "Ỵ" },           /* U+1EF4 Y + dot below */
	/* 0x7F */ { NULL },
};

const struct pager_charset pager_charset_flex_motorola_linguist_vietnamese = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "vietnamese",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
