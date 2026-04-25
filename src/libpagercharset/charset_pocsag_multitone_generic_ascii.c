/*
 * POCSAG MultiTone generic ASCII codepage.
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
 * Vendor:   multitone
 * Model:    generic
 * Charset:  ascii-uppercase
 *
 * Pagers: MultiTone RPR series.
 *
 * Standard ASCII with case folding: lowercase a-z maps to uppercase A-Z.
 * No Cyrillic support. The pager ROM only contains uppercase Latin glyphs.
 *
 * Layout:
 *   0x20-0x3F: Standard ASCII punctuation and digits.
 *   0x40-0x5F: Uppercase Latin A-Z + standard symbols.
 *   0x60:      backtick (displayed as backtick on pager).
 *   0x61-0x7A: Displayed as uppercase A-Z (same glyphs as 0x41-0x5A).
 *   0x7B-0x7E: ASCII symbols {|}~.
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

	/* 0x20-0x3F: Standard ASCII */
	/* 0x20 */ { " " },  /* 0x21 */ { "!" },  /* 0x22 */ { "\"" }, /* 0x23 */ { "#" },
	/* 0x24 */ { "$" },  /* 0x25 */ { "%" },  /* 0x26 */ { "&" },  /* 0x27 */ { "'" },
	/* 0x28 */ { "(" },  /* 0x29 */ { ")" },  /* 0x2A */ { "*" },  /* 0x2B */ { "+" },
	/* 0x2C */ { "," },  /* 0x2D */ { "-" },  /* 0x2E */ { "." },  /* 0x2F */ { "/" },
	/* 0x30 */ { "0" },  /* 0x31 */ { "1" },  /* 0x32 */ { "2" },  /* 0x33 */ { "3" },
	/* 0x34 */ { "4" },  /* 0x35 */ { "5" },  /* 0x36 */ { "6" },  /* 0x37 */ { "7" },
	/* 0x38 */ { "8" },  /* 0x39 */ { "9" },  /* 0x3A */ { ":" },  /* 0x3B */ { ";" },
	/* 0x3C */ { "<" },  /* 0x3D */ { "=" },  /* 0x3E */ { ">" },  /* 0x3F */ { "?" },

	/* 0x40-0x5F: Uppercase Latin */
	/* 0x40 */ { "@" },
	/* 0x41 */ { "A", "a" },  /* 0x42 */ { "B", "b" },  /* 0x43 */ { "C", "c" },
	/* 0x44 */ { "D", "d" },  /* 0x45 */ { "E", "e" },  /* 0x46 */ { "F", "f" },
	/* 0x47 */ { "G", "g" },  /* 0x48 */ { "H", "h" },  /* 0x49 */ { "I", "i" },
	/* 0x4A */ { "J", "j" },  /* 0x4B */ { "K", "k" },  /* 0x4C */ { "L", "l" },
	/* 0x4D */ { "M", "m" },  /* 0x4E */ { "N", "n" },  /* 0x4F */ { "O", "o" },
	/* 0x50 */ { "P", "p" },  /* 0x51 */ { "Q", "q" },  /* 0x52 */ { "R", "r" },
	/* 0x53 */ { "S", "s" },  /* 0x54 */ { "T", "t" },  /* 0x55 */ { "U", "u" },
	/* 0x56 */ { "V", "v" },  /* 0x57 */ { "W", "w" },  /* 0x58 */ { "X", "x" },
	/* 0x59 */ { "Y", "y" },  /* 0x5A */ { "Z", "z" },
	/* 0x5B */ { "[" },  /* 0x5C */ { "\\" }, /* 0x5D */ { "]" },
	/* 0x5E */ { "^" },  /* 0x5F */ { "_" },

	/* 0x60-0x7F: Lowercase displays as uppercase (ROM has no lowercase glyphs) */
	/* 0x60 */ { "`" },
	/* 0x61 */ { "A" },  /* 0x62 */ { "B" },  /* 0x63 */ { "C" },
	/* 0x64 */ { "D" },  /* 0x65 */ { "E" },  /* 0x66 */ { "F" },
	/* 0x67 */ { "G" },  /* 0x68 */ { "H" },  /* 0x69 */ { "I" },
	/* 0x6A */ { "J" },  /* 0x6B */ { "K" },  /* 0x6C */ { "L" },
	/* 0x6D */ { "M" },  /* 0x6E */ { "N" },  /* 0x6F */ { "O" },
	/* 0x70 */ { "P" },  /* 0x71 */ { "Q" },  /* 0x72 */ { "R" },
	/* 0x73 */ { "S" },  /* 0x74 */ { "T" },  /* 0x75 */ { "U" },
	/* 0x76 */ { "V" },  /* 0x77 */ { "W" },  /* 0x78 */ { "X" },
	/* 0x79 */ { "Y" },  /* 0x7A */ { "Z" },
	/* 0x7B */ { "{" },  /* 0x7C */ { "|" },  /* 0x7D */ { "}" },
	/* 0x7E */ { "~" },  /* 0x7F */ { NULL },
};

const struct pager_charset pager_charset_pocsag_multitone_generic_ascii = {
	.protocol  = "pocsag",
	.vendor    = "multitone",
	.model     = "generic",
	.charset   = "ascii-uppercase",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
