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
 * WARNING: Unclear image, hard to OCR.  Vietnamese diacritics need
 * careful verification against original FLX2 Linguist documentation.
 *
 * Heavily modified 7-bit layout for Vietnamese.
 * Columns 0-1 contain Latin letters, punctuation, and control chars
 * in non-standard positions.  Columns 2-3 contain Vietnamese vowels
 * with diacritics.  Columns 4-7 contain more Vietnamese characters.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* Column 0 (0x00-0x0F) */
	/* 0x00 */ { NULL },  /* NUL */
	/* 0x01 */ { "O" },
	/* 0x02 */ { "-" },
	/* 0x03 */ { "+" },
	/* 0x04 */ { NULL },  /* EOT */
	/* 0x05 */ { "I" },
	/* 0x06 */ { "=" },
	/* 0x07 */ { "S" },
	/* 0x08 */ { "$" },
	/* 0x09 */ { "B" },
	/* 0x0A */ { "\n" },  /* LF  */
	/* 0x0B */ { "G" },
	/* 0x0C */ { "." },
	/* 0x0D */ { "\r" },  /* CR  */
	/* 0x0E */ { NULL },  /* SO  */
	/* 0x0F */ { NULL },  /* SI  */
	/* Column 1 (0x10-0x1F) */
	/* 0x10 */ { "F" },
	/* 0x11 */ { "%" },
	/* 0x12 */ { "J" },
	/* 0x13 */ { "\xe2\x80\x9c" }, /* U+201C left double quotation mark */
	/* 0x14 */ { "W" },
	/* 0x15 */ { "!" },
	/* 0x16 */ { "Z" },
	/* 0x17 */ { "*" },
	/* 0x18 */ { "Q" },
	/* 0x19 */ { ")" },
	/* 0x1A */ { "(" },
	/* 0x1B */ { NULL },  /* ESC */
	/* 0x1C */ { "?" },
	/* 0x1D */ { "," },
	/* 0x1E */ { "#" },
	/* 0x1F */ { "/" },
	/* Column 2 (0x20-0x2F) */
	/* 0x20 */ { " " },           /* SP */
	/* 0x21 */ { "Â" },           /* U+00C2 */
	/* 0x22 */ { "Ă" },           /* U+0102 */
	/* 0x23 */ { "Ả" },           /* U+1EA2 */
	/* 0x24 */ { "Ã" },           /* U+00C3 */
	/* 0x25 */ { "À" },           /* U+00C0 */
	/* 0x26 */ { "Á" },           /* U+00C1 */
	/* 0x27 */ { "Ạ" },           /* U+1EA0 */
	/* 0x28 */ { "Ấ" },           /* U+1EA4 */
	/* 0x29 */ { "Ầ" },           /* U+1EA6 */
	/* 0x2A */ { "Ẩ" },           /* U+1EA8 */
	/* 0x2B */ { "Ẫ" },           /* U+1EAA */
	/* 0x2C */ { "Ậ" },           /* U+1EAC */
	/* 0x2D */ { "Ắ" },           /* U+1EAE */
	/* 0x2E */ { "Ằ" },           /* U+1EB0 */
	/* 0x2F */ { "Ẳ" },           /* U+1EB2 */
	/* Column 3 (0x30-0x3F) */
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
	/* 0x3B */ { "Ặ" },           /* U+1EB6 */
	/* 0x3C */ { "Ê" },           /* U+00CA */
	/* 0x3D */ { "Ẻ" },           /* U+1EBA */
	/* 0x3E */ { "Ẽ" },           /* U+1EBC */
	/* 0x3F */ { "É" },           /* U+00C9 */
	/* Column 4 (0x40-0x4F) */
	/* 0x40 */ { "È" },           /* U+00C8 */
	/* 0x41 */ { "A" },
	/* 0x42 */ { "B" },
	/* 0x43 */ { "C" },
	/* 0x44 */ { "D" },
	/* 0x45 */ { "E" },
	/* 0x46 */ { "Đ" },           /* U+0110 */
	/* 0x47 */ { "Ỷ" },           /* U+1EF6 */
	/* 0x48 */ { "H" },
	/* 0x49 */ { "Ẹ" },           /* U+1EB8 */
	/* 0x4A */ { "Ế" },           /* U+1EBE */
	/* 0x4B */ { "K" },
	/* 0x4C */ { "L" },
	/* 0x4D */ { "M" },
	/* 0x4E */ { "N" },
	/* 0x4F */ { "Ề" },           /* U+1EC0 */
	/* Column 5 (0x50-0x5F) */
	/* 0x50 */ { "P" },
	/* 0x51 */ { "Ả" },           /* U+1EA2 — duplicate, display variant */
	/* 0x52 */ { "R" },
	/* 0x53 */ { "Ý" },           /* U+00DD */
	/* 0x54 */ { "T" },
	/* 0x55 */ { "U" },
	/* 0x56 */ { "V" },
	/* 0x57 */ { "Ỹ" },           /* U+1EF8 */
	/* 0x58 */ { "X" },
	/* 0x59 */ { "Y" },
	/* 0x5A */ { "Ể" },           /* U+1EC2 */
	/* 0x5B */ { "Ễ" },           /* U+1EC4 */
	/* 0x5C */ { "Ệ" },           /* U+1EC6 */
	/* 0x5D */ { "Ô" },           /* U+00D4 */
	/* 0x5E */ { "Ỏ" },           /* U+1ECE */
	/* 0x5F */ { "Õ" },           /* U+00D5 */
	/* Column 6 (0x60-0x6F) */
	/* 0x60 */ { "Ó" },           /* U+00D3 */
	/* 0x61 */ { "Ò" },           /* U+00D2 */
	/* 0x62 */ { "Ọ" },           /* U+1ECC */
	/* 0x63 */ { "Ổ" },           /* U+1ED4 */
	/* 0x64 */ { "Ỗ" },           /* U+1ED6 */
	/* 0x65 */ { "Ố" },           /* U+1ED0 */
	/* 0x66 */ { "Ồ" },           /* U+1ED2 */
	/* 0x67 */ { "Ộ" },           /* U+1ED8 */
	/* 0x68 */ { "Ơ" },           /* U+01A0 */
	/* 0x69 */ { "Ớ" },           /* U+1EDA */
	/* 0x6A */ { "Ờ" },           /* U+1EDC */
	/* 0x6B */ { "Ó" },           /* U+00D3 — display variant */
	/* 0x6C */ { "Ọ" },           /* U+1ECC — display variant */
	/* 0x6D */ { "Ỡ" },           /* U+1EDE */
	/* 0x6E */ { "U" },           /* U */
	/* 0x6F */ { "Ủ" },           /* U+1EE6 */
	/* Column 7 (0x70-0x7F) */
	/* 0x70 */ { "Ũ" },           /* U+0168 */
	/* 0x71 */ { "Ú" },           /* U+00DA */
	/* 0x72 */ { "Ù" },           /* U+00D9 */
	/* 0x73 */ { "Ụ" },           /* U+1EE4 */
	/* 0x74 */ { "Ị" },           /* U+1ECA */
	/* 0x75 */ { "Ĩ" },           /* U+0128 */
	/* 0x76 */ { "Ì" },           /* U+00CC */
	/* 0x77 */ { "Í" },           /* U+00CD */
	/* 0x78 */ { "Ị" },           /* U+1ECA — display variant */
	/* 0x79 */ { "Ỉ" },           /* U+1EC8 */
	/* 0x7A */ { "Y" },
	/* 0x7B */ { NULL },
	/* 0x7C */ { NULL },
	/* 0x7D */ { NULL },
	/* 0x7E */ { NULL },
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
