/*
 * Golay/GSC standard 6-bit ASCII codepage.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: golay
 * Vendor:   generic
 * Model:    generic
 * Charset:  ascii
 *
 * ITU-R Rep. 900-2 Table VII character set.
 * 6-bit codes (0x00-0x3F), 64 entries.  Uppercase only.
 *
 * The mapping is: displayed_char = code + 0x20 (arithmetic offset),
 * except for special cases:
 *   0x3C = CR (carriage return, not '<')
 *   0x3E = NUL fill (padding, not '>')
 *   0x3F = '_' (0x5F, outside the 0x20-0x5D arithmetic range)
 *
 * No lowercase letters exist in this character set.
 * TX encode: lowercase input is uppercased before lookup.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[64] = {
	/* 0x00 */ { " " },        /* SP  (0x20) */
	/* 0x01 */ { "!" },        /* !   (0x21) */
	/* 0x02 */ { "\"" },       /* "   (0x22) */
	/* 0x03 */ { "#" },        /* #   (0x23) */
	/* 0x04 */ { "$" },        /* $   (0x24) */
	/* 0x05 */ { "%" },        /* %   (0x25) */
	/* 0x06 */ { "&" },        /* &   (0x26) */
	/* 0x07 */ { "'" },        /* '   (0x27) */
	/* 0x08 */ { "(" },        /* (   (0x28) */
	/* 0x09 */ { ")" },        /* )   (0x29) */
	/* 0x0A */ { "*" },        /* *   (0x2A) */
	/* 0x0B */ { "+" },        /* +   (0x2B) */
	/* 0x0C */ { "," },        /* ,   (0x2C) */
	/* 0x0D */ { "-" },        /* -   (0x2D) */
	/* 0x0E */ { "." },        /* .   (0x2E) */
	/* 0x0F */ { "/" },        /* /   (0x2F) */
	/* 0x10 */ { "0" },        /* 0   (0x30) */
	/* 0x11 */ { "1" },        /* 1   (0x31) */
	/* 0x12 */ { "2" },        /* 2   (0x32) */
	/* 0x13 */ { "3" },        /* 3   (0x33) */
	/* 0x14 */ { "4" },        /* 4   (0x34) */
	/* 0x15 */ { "5" },        /* 5   (0x35) */
	/* 0x16 */ { "6" },        /* 6   (0x36) */
	/* 0x17 */ { "7" },        /* 7   (0x37) */
	/* 0x18 */ { "8" },        /* 8   (0x38) */
	/* 0x19 */ { "9" },        /* 9   (0x39) */
	/* 0x1A */ { ":" },        /* :   (0x3A) */
	/* 0x1B */ { ";" },        /* ;   (0x3B) */
	/* 0x1C */ { "<" },        /* <   (0x3C) */
	/* 0x1D */ { "=" },        /* =   (0x3D) */
	/* 0x1E */ { ">" },        /* >   (0x3E) */
	/* 0x1F */ { "?" },        /* ?   (0x3F) */
	/* 0x20 */ { "@", "\\" },  /* @   (0x40) — also accepts backslash (lossy) */
	/* 0x21 */ { "A", "a" },   /* A   (0x41) */
	/* 0x22 */ { "B", "b" },   /* B   (0x42) */
	/* 0x23 */ { "C", "c" },   /* C   (0x43) */
	/* 0x24 */ { "D", "d" },   /* D   (0x44) */
	/* 0x25 */ { "E", "e" },   /* E   (0x45) */
	/* 0x26 */ { "F", "f" },   /* F   (0x46) */
	/* 0x27 */ { "G", "g" },   /* G   (0x47) */
	/* 0x28 */ { "H", "h" },   /* H   (0x48) */
	/* 0x29 */ { "I", "i" },   /* I   (0x49) */
	/* 0x2A */ { "J", "j" },   /* J   (0x4A) */
	/* 0x2B */ { "K", "k" },   /* K   (0x4B) */
	/* 0x2C */ { "L", "l" },   /* L   (0x4C) */
	/* 0x2D */ { "M", "m" },   /* M   (0x4D) */
	/* 0x2E */ { "N", "n" },   /* N   (0x4E) */
	/* 0x2F */ { "O", "o" },   /* O   (0x4F) */
	/* 0x30 */ { "P", "p" },   /* P   (0x50) */
	/* 0x31 */ { "Q", "q" },   /* Q   (0x51) */
	/* 0x32 */ { "R", "r" },   /* R   (0x52) */
	/* 0x33 */ { "S", "s" },   /* S   (0x53) */
	/* 0x34 */ { "T", "t" },   /* T   (0x54) */
	/* 0x35 */ { "U", "u" },   /* U   (0x55) */
	/* 0x36 */ { "V", "v" },   /* V   (0x56) */
	/* 0x37 */ { "W", "w" },   /* W   (0x57) */
	/* 0x38 */ { "X", "x" },   /* X   (0x58) */
	/* 0x39 */ { "Y", "y" },   /* Y   (0x59) */
	/* 0x3A */ { "Z", "z" },   /* Z   (0x5A) */
	/* 0x3B */ { "[", "{" },   /* [   (0x5B) — also accepts { (lossy) */
	/* 0x3C */ { "\r", "\n" }, /* CR  (special) — also accepts LF */
	/* 0x3D */ { "]", "}" },   /* ]   (0x5D) — also accepts } (lossy) */
	/* 0x3E */ { NULL },       /* NUL fill (padding character) */
	/* 0x3F */ { "_" },        /* _   (0x5F, outside arithmetic range) */
};

const struct pager_charset pager_charset_golay_generic_generic_ascii = {
	.protocol  = "golay",
	.vendor    = "generic",
	.model     = "generic",
	.charset   = "ascii",
	.code_bits = 6,
	.num_codes = 64,
	.entries   = entries,
};
