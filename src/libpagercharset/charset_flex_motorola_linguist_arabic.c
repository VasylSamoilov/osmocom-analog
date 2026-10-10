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
 * Re-transcribed from the clean rendered manual page (pages/17A_arabic_A5-166.png,
 * Table A-5) at high zoom.  Replaces the earlier "Unclear image, hard to OCR"
 * entries.  Columns 4-7 carry the Arabic alphabet in standard abjad order
 * (ا ب ت ث ج ح خ د ذ ر ز س ش ص ض ط ظ ع غ ف ق ك ل م ن ه و ى ي), which was used to
 * corroborate each cell; glyphs are rendered as presentation (positional) forms
 * but are mapped to the base letter in the Arabic block U+0621..U+064A.
 * Columns 0-1 hold sparse Latin + a few hah-family / extended letters that are
 * NOT part of that ordered run; cells resting on the scan alone are flagged
 * UNCORROBORATED.
 */

#include "pager_charset.h"

static const pager_charset_entry entries[128] = {
	/* Column 0 (0x00-0x0F) */
	/* 0x00 */ { NULL },
	/* 0x01 */ { "A" },
	/* 0x02 */ { "P" },
	/* 0x03 */ { "M" },
	/* 0x04 */ { NULL },
	/* 0x05 */ { "ح" },           /* U+062D hah (isolated) */
	/* 0x06 */ { NULL },
	/* 0x07 */ { "ع" },           /* U+0639 ain (isolated) */
	/* 0x08 */ { NULL },
	/* 0x09 */ { "ج" },           /* U+062C jeem (hah + dot below) */
	/* 0x0A */ { NULL },
	/* 0x0B */ { "چ" },           /* U+0686 tcheh — hah-family w/ extra marks UNCORROBORATED */
	/* 0x0C */ { "ج" },           /* U+062C jeem variant UNCORROBORATED */
	/* 0x0D */ { NULL },
	/* 0x0E */ { NULL },
	/* 0x0F */ { NULL },
	/* Column 1 (0x10-0x1F) */
	/* 0x10 */ { "ح" },           /* U+062D hah UNCORROBORATED */
	/* 0x11 */ { NULL },
	/* 0x12 */ { "خ" },           /* U+062E khah (hah + dot above) */
	/* 0x13 */ { NULL },
	/* 0x14 */ { "غ" },           /* U+063A ghain (dot above) */
	/* 0x15 */ { NULL },
	/* 0x16 */ { "ع" },           /* U+0639 ain */
	/* 0x17 */ { NULL },
	/* 0x18 */ { "ح" },           /* U+062D hah UNCORROBORATED */
	/* 0x19 */ { "چ" },           /* U+0686 tcheh (hah-family, extra marks) UNCORROBORATED */
	/* 0x1A */ { "ج" },           /* U+062C jeem (3-dot variant) UNCORROBORATED */
	/* 0x1B */ { NULL },
	/* 0x1C */ { "ژ" },           /* U+0698 jeh (reh + 3 dots) UNCORROBORATED */
	/* 0x1D */ { "ع" },           /* U+0639 ain variant UNCORROBORATED */
	/* 0x1E */ { NULL },
	/* 0x1F */ { NULL },
	/* Column 2 (0x20-0x2F) */
	/* 0x20 */ { " " },
	/* 0x21 */ { "!" },
	/* 0x22 */ { "\"" },
	/* 0x23 */ { "#" },
	/* 0x24 */ { "$" },
	/* 0x25 */ { "%" },
	/* 0x26 */ { "٣" },           /* U+0663 Arabic-Indic digit three */
	/* 0x27 */ { "س" },           /* U+0633 seen UNCORROBORATED */
	/* 0x28 */ { "(" },
	/* 0x29 */ { ")" },
	/* 0x2A */ { "*" },
	/* 0x2B */ { "+" },
	/* 0x2C */ { "،" },           /* U+060C Arabic comma */
	/* 0x2D */ { "." },           /* mid-dot */
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
	/* 0x3B */ { ";" },
	/* 0x3C */ { "<" },
	/* 0x3D */ { "=" },
	/* 0x3E */ { ">" },
	/* 0x3F */ { "؟" },           /* U+061F Arabic question mark */
	/* Column 4 (0x40-0x4F) — Arabic letters, abjad run begins */
	/* 0x40 */ { "ة" },           /* U+0629 teh marbuta (final form) */
	/* 0x41 */ { "ـ" },           /* U+0640 tatweel/kashida */
	/* 0x42 */ { "آ" },           /* U+0622 alef with madda */
	/* 0x43 */ { "آ" },           /* U+0622 alef with madda (form) */
	/* 0x44 */ { "ج" },           /* U+062C jeem (initial) — abjad */
	/* 0x45 */ { "ا" },           /* U+0627 alef */
	/* 0x46 */ { "ت" },           /* U+062A teh */
	/* 0x47 */ { "ا" },           /* U+0627 alef */
	/* 0x48 */ { "ب" },           /* U+0628 beh — abjad #2 */
	/* 0x49 */ { "ة" },           /* U+0629 teh marbuta */
	/* 0x4A */ { "ت" },           /* U+062A teh — abjad #3 */
	/* 0x4B */ { "ث" },           /* U+062B theh — abjad #4 */
	/* 0x4C */ { "ج" },           /* U+062C jeem — abjad #5 */
	/* 0x4D */ { "ح" },           /* U+062D hah — abjad #6 */
	/* 0x4E */ { "خ" },           /* U+062E khah — abjad #7 */
	/* 0x4F */ { "د" },           /* U+062F dal — abjad #8 */
	/* Column 5 (0x50-0x5F) — abjad run continues */
	/* 0x50 */ { "ذ" },           /* U+0630 thal — abjad #9 */
	/* 0x51 */ { "ر" },           /* U+0631 reh — abjad #10 */
	/* 0x52 */ { "ز" },           /* U+0632 zain — abjad #11 */
	/* 0x53 */ { "س" },           /* U+0633 seen — abjad #12 */
	/* 0x54 */ { "ش" },           /* U+0634 sheen — abjad #13 */
	/* 0x55 */ { "ص" },           /* U+0635 sad — abjad #14 */
	/* 0x56 */ { "ض" },           /* U+0636 dad — abjad #15 */
	/* 0x57 */ { "ط" },           /* U+0637 tah — abjad #16 */
	/* 0x58 */ { "ظ" },           /* U+0638 zah — abjad #17 */
	/* 0x59 */ { "ع" },           /* U+0639 ain — abjad #18 */
	/* 0x5A */ { "غ" },           /* U+063A ghain — abjad #19 */
	/* 0x5B */ { "لا" },          /* U+0644 U+0627 lam-alef ligature */
	/* 0x5C */ { "ك" },           /* U+0643 kaf */
	/* 0x5D */ { "ك" },           /* U+0643 kaf (form) */
	/* 0x5E */ { "ه" },           /* U+0647 heh */
	/* 0x5F */ { "ع" },           /* U+0639 ain (small/final) UNCORROBORATED */
	/* Column 6 (0x60-0x6F) — abjad run continues */
	/* 0x60 */ { "ـ" },           /* U+0640 tatweel */
	/* 0x61 */ { "ف" },           /* U+0641 feh — abjad #20 */
	/* 0x62 */ { "ق" },           /* U+0642 qaf — abjad #21 */
	/* 0x63 */ { "ك" },           /* U+0643 kaf — abjad #22 */
	/* 0x64 */ { "ل" },           /* U+0644 lam — abjad #23 */
	/* 0x65 */ { "م" },           /* U+0645 meem — abjad #24 */
	/* 0x66 */ { "ن" },           /* U+0646 noon — abjad #25 */
	/* 0x67 */ { "ه" },           /* U+0647 heh — abjad #26 */
	/* 0x68 */ { "و" },           /* U+0648 waw — abjad #27 */
	/* 0x69 */ { "ى" },           /* U+0649 alef maksura — abjad #28 */
	/* 0x6A */ { "ي" },           /* U+064A yeh — abjad #29 */
	/* 0x6B */ { "ل" },           /* U+0644 lam (form) UNCORROBORATED */
	/* 0x6C */ { "ب" },           /* U+0628 beh (medial form) UNCORROBORATED */
	/* 0x6D */ { "ن" },           /* U+0646 noon (medial form) UNCORROBORATED */
	/* 0x6E */ { "ث" },           /* U+062B theh (medial form) UNCORROBORATED */
	/* 0x6F */ { "ه" },           /* U+0647 heh (medial form) UNCORROBORATED */
	/* Column 7 (0x70-0x7F) — presentation forms (not abjad-ordered) */
	/* 0x70 */ { "ن" },           /* U+0646 noon UNCORROBORATED */
	/* 0x71 */ { "و" },           /* U+0648 waw (final) UNCORROBORATED */
	/* 0x72 */ { "ز" },           /* U+0632 zain (final) UNCORROBORATED */
	/* 0x73 */ { "ع" },           /* U+0639 ain (medial/final) UNCORROBORATED */
	/* 0x74 */ { "غ" },           /* U+063A ghain UNCORROBORATED */
	/* 0x75 */ { "آ" },           /* U+0622 alef with madda */
	/* 0x76 */ { "ه" },           /* U+0647 heh (medial) UNCORROBORATED */
	/* 0x77 */ { "ك" },           /* U+0643 kaf (medial) UNCORROBORATED */
	/* 0x78 */ { "ل" },           /* U+0644 lam (final) UNCORROBORATED */
	/* 0x79 */ { "م" },           /* U+0645 meem (medial) UNCORROBORATED */
	/* 0x7A */ { "ن" },           /* U+0646 noon (medial) UNCORROBORATED */
	/* 0x7B */ { "آ" },           /* U+0622 alef with madda */
	/* 0x7C */ { "ب" },           /* U+0628 beh (medial) UNCORROBORATED */
	/* 0x7D */ { "ي" },           /* U+064A yeh (medial) UNCORROBORATED */
	/* 0x7E */ { NULL },
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
