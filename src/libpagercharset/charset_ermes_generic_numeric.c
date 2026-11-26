/*
 * ERMES numeric 4-bit codepage.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Protocol: ermes
 * Vendor:   generic
 * Model:    generic
 * Charset:  numeric
 *
 * ETS 300 133-2 ed.2 (November 1997) Annex B, Table B.1 p.45: "Characters for numeric
 * paging", 4 bits a character (b4 = MSB). Space (0xB) is also the fill of the last
 * codeword (ETS 300 133-4 §5.5.4 p.23). TX accepts a lowercase u.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[16] = {
	/* 0x0 */ { "0" },
	/* 0x1 */ { "1" },
	/* 0x2 */ { "2" },
	/* 0x3 */ { "3" },
	/* 0x4 */ { "4" },
	/* 0x5 */ { "5" },
	/* 0x6 */ { "6" },
	/* 0x7 */ { "7" },
	/* 0x8 */ { "8" },
	/* 0x9 */ { "9" },
	/* 0xA */ { "/" },
	/* 0xB */ { " " },
	/* 0xC */ { "U", "u" },
	/* 0xD */ { "-" },
	/* 0xE */ { "." },
	/* 0xF */ { "%" },
};

const struct pager_charset pager_charset_ermes_generic_generic_numeric = {
	.protocol  = "ermes",
	.vendor    = "generic",
	.model     = "generic",
	.charset   = "numeric",
	.code_bits = 4,
	.num_codes = 16,
	.entries   = entries,
};
