/*
 * FLEX Arabic codepage — Motorola FLX2 Linguist (Table A-5).
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
 * Charset:  arabic
 *
 * WARNING: Unclear image, hard to OCR.  Arabic glyphs need careful
 * verification against original FLX2 Linguist documentation.
 *
 * Full Arabic alphabet set.  Columns 0-1 contain sparse Latin/control
 * and some Arabic letters.  Column 2 has ASCII punctuation with Arabic
 * comma/semicolon/question mark.  Column 3 has digits.  Columns 4-7
 * contain Arabic letters and presentation forms.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* Column 0 (0x00-0x0F) */
	/* 0x00 */ { NULL },
	/* 0x01 */ { "A" },
	/* 0x02 */ { "P" },
	/* 0x03 */ { "M" },
	/* 0x04 */ { NULL },
	/* 0x05 */ { "ح" },           /* U+062D haa */
	/* 0x06 */ { NULL },
	/* 0x07 */ { "ع" },           /* U+0639 ain */
	/* 0x08 */ { NULL },
	/* 0x09 */ { "ء" },           /* U+0621 hamza */
	/* 0x0A */ { NULL },
	/* 0x0B */ { "چ" },           /* U+0686 tcheh */
	/* 0x0C */ { "ژ" },           /* U+0698 jeh */
	/* 0x0D */ { NULL },
	/* 0x0E */ { NULL },
	/* 0x0F */ { NULL },
	/* Column 1 (0x10-0x1F) */
	/* 0x10 */ { "ج" },           /* U+062C jeem */
	/* 0x11 */ { NULL },
	/* 0x12 */ { "خ" },           /* U+062E khaa */
	/* 0x13 */ { NULL },
	/* 0x14 */ { "غ" },           /* U+063A ghain */
	/* 0x15 */ { NULL },
	/* 0x16 */ { "ع" },           /* U+0639 ain */
	/* 0x17 */ { NULL },
	/* 0x18 */ { "ح" },           /* U+062D haa */
	/* 0x19 */ { "خ" },           /* U+062E khaa */
	/* 0x1A */ { "پ" },           /* U+067E peh */
	/* 0x1B */ { NULL },
	/* 0x1C */ { "ڗ" },           /* U+0697 */
	/* 0x1D */ { "ع" },           /* U+0639 ain */
	/* 0x1E */ { NULL },
	/* 0x1F */ { NULL },
	/* Column 2 (0x20-0x2F) */
	/* 0x20 */ { " " },
	/* 0x21 */ { "!" },
	/* 0x22 */ { "\"" },
	/* 0x23 */ { "#" },
	/* 0x24 */ { "$" },
	/* 0x25 */ { "%" },
	/* 0x26 */ { "٢" },           /* U+0662 Arabic-Indic digit two */
	/* 0x27 */ { "س" },           /* U+0633 seen */
	/* 0x28 */ { "(" },
	/* 0x29 */ { ")" },
	/* 0x2A */ { "*" },
	/* 0x2B */ { "+" },
	/* 0x2C */ { "،" },           /* U+060C Arabic comma */
	/* 0x2D */ { "-" },
	/* 0x2E */ { "." },
	/* 0x2F */ { "/" },
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
	/* 0x3B */ { "؛" },           /* U+061B Arabic semicolon */
	/* 0x3C */ { "<" },
	/* 0x3D */ { "=" },
	/* 0x3E */ { ">" },
	/* 0x3F */ { "؟" },           /* U+061F Arabic question mark */
	/* Column 4 (0x40-0x4F) — Arabic letters */
	/* 0x40 */ { "ئ" },           /* U+0626 yeh with hamza above */
	/* 0x41 */ { "-" },           /* dash/kashida */
	/* 0x42 */ { "آ" },           /* U+0622 alef with madda */
	/* 0x43 */ { "آ" },           /* U+0622 alef with madda (form) */
	/* 0x44 */ { "ج" },           /* U+062C jeem */
	/* 0x45 */ { "١" },           /* U+0661 Arabic-Indic digit one */
	/* 0x46 */ { "ت" },           /* U+062A teh */
	/* 0x47 */ { "ا" },           /* U+0627 alef */
	/* 0x48 */ { "ب" },           /* U+0628 beh */
	/* 0x49 */ { "ة" },           /* U+0629 teh marbuta */
	/* 0x4A */ { "ث" },           /* U+062B theh */
	/* 0x4B */ { "ث" },           /* U+062B theh (form) */
	/* 0x4C */ { "ج" },           /* U+062C jeem */
	/* 0x4D */ { "ح" },           /* U+062D haa */
	/* 0x4E */ { "خ" },           /* U+062E khaa */
	/* 0x4F */ { "د" },           /* U+062F dal */
	/* Column 5 (0x50-0x5F) — Arabic letters continued */
	/* 0x50 */ { "ذ" },           /* U+0630 thal */
	/* 0x51 */ { "ر" },           /* U+0631 reh */
	/* 0x52 */ { "ز" },           /* U+0632 zain */
	/* 0x53 */ { "س" },           /* U+0633 seen */
	/* 0x54 */ { "ش" },           /* U+0634 sheen */
	/* 0x55 */ { "ص" },           /* U+0635 sad */
	/* 0x56 */ { "ض" },           /* U+0636 dad */
	/* 0x57 */ { "ط" },           /* U+0637 tah */
	/* 0x58 */ { "ظ" },           /* U+0638 zah */
	/* 0x59 */ { "ع" },           /* U+0639 ain */
	/* 0x5A */ { "غ" },           /* U+063A ghain */
	/* 0x5B */ { "لا" },          /* lam-alef ligature */
	/* 0x5C */ { "ـ" },           /* U+0640 tatweel/kashida */
	/* 0x5D */ { "ك" },           /* U+0643 kaf */
	/* 0x5E */ { "ه" },           /* U+0647 heh */
	/* 0x5F */ { "ء" },           /* U+0621 hamza */
	/* Column 6 (0x60-0x6F) — Arabic letters / forms */
	/* 0x60 */ { "ـ" },           /* U+0640 tatweel */
	/* 0x61 */ { "ف" },           /* U+0641 feh */
	/* 0x62 */ { "ق" },           /* U+0642 qaf */
	/* 0x63 */ { "ك" },           /* U+0643 kaf */
	/* 0x64 */ { "ل" },           /* U+0644 lam */
	/* 0x65 */ { "ـ" },           /* U+0640 tatweel */
	/* 0x66 */ { "ن" },           /* U+0646 noon */
	/* 0x67 */ { "ه" },           /* U+0647 heh */
	/* 0x68 */ { "و" },           /* U+0648 waw */
	/* 0x69 */ { "ى" },           /* U+0649 alef maksura */
	/* 0x6A */ { "ي" },           /* U+064A yeh */
	/* 0x6B */ { "ا" },           /* U+0627 alef */
	/* 0x6C */ { "ت" },           /* U+062A teh */
	/* 0x6D */ { "ـ" },           /* U+0640 tatweel */
	/* 0x6E */ { "ـ" },           /* U+0640 tatweel */
	/* 0x6F */ { "ـ" },           /* U+0640 tatweel */
	/* Column 7 (0x70-0x7F) — Arabic letters / forms */
	/* 0x70 */ { "ذ" },           /* U+0630 thal */
	/* 0x71 */ { "و" },           /* U+0648 waw */
	/* 0x72 */ { "ز" },           /* U+0632 zain */
	/* 0x73 */ { "ع" },           /* U+0639 ain */
	/* 0x74 */ { "غ" },           /* U+063A ghain */
	/* 0x75 */ { "آ" },           /* U+0622 alef with madda */
	/* 0x76 */ { "ـ" },           /* U+0640 tatweel */
	/* 0x77 */ { "ك" },           /* U+0643 kaf */
	/* 0x78 */ { "ل" },           /* U+0644 lam */
	/* 0x79 */ { "ت" },           /* U+062A teh */
	/* 0x7A */ { "ـ" },           /* U+0640 tatweel */
	/* 0x7B */ { "ف" },           /* U+0641 feh */
	/* 0x7C */ { "آ" },           /* U+0622 alef with madda */
	/* 0x7D */ { "ت" },           /* U+062A teh */
	/* 0x7E */ { "ـ" },           /* U+0640 tatweel */
	/* 0x7F */ { NULL },  /* DEL */
};

const struct pager_charset pager_charset_flex_motorola_linguist_arabic = {
	.protocol  = "flex",
	.vendor    = "motorola",
	.model     = "linguist",
	.charset   = "arabic",
	.code_bits = 7,
	.num_codes = 128,
	.entries   = entries,
};
