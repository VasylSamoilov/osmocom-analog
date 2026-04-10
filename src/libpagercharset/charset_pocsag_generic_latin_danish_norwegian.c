/*
 * POCSAG Danish/Norwegian codepage (Table A-18).
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
 * Vendor:   generic
 * Model:    generic
 * Charset:  latin-danish-norwegian
 *
 * Standard 7-bit ASCII with substitutions for Danish/Norwegian characters:
 *   0x40 '@' → §    0x5B '[' → Æ    0x5C '\' → Ø    0x5D ']' → Å
 *   0x7B '{' → æ    0x7C '|' → ø    0x7D '}' → å    0x7E '~' → -
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
	/* 0x40 */ { "§" },           /* U+00A7 section sign — replaces @ */
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
	/* 0x5B */ { "Æ" },           /* U+00C6 — replaces [ */
	/* 0x5C */ { "Ø" },           /* U+00D8 — replaces \ */
	/* 0x5D */ { "Å" },           /* U+00C5 — replaces ] */
	/* 0x5E */ { "^" },
	/* 0x5F */ { "_" },
	/* 0x60 */ { "`" },
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
	/* 0x7B */ { "œ" },           /* U+0153 — replaces { */
	/* 0x7C */ { "ø" },           /* U+00F8 — replaces | */
	/* 0x7D */ { "å" },           /* U+00E5 — replaces } */
	/* 0x7E */ { "-" },           /* hyphen-minus — replaces ~ */
	/* 0x7F */ { NULL },  /* DEL */
};

const struct pager_charset pager_charset_pocsag_generic_generic_latin_danish_norwegian = {
	.protocol  = "pocsag",
	.vendor    = "generic",
	.model     = "generic",
	.charset   = "latin-danish-norwegian",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
