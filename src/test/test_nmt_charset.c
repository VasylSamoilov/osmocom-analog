/* Test of the NMT SMS codepages: each set of NMT Doc 450-3/900-3 Annex 24 4.3
 * encodes and decodes its letters, CR and LF are one-line glyphs, and the
 * codes are those of the standard and of the phone read on air.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "../libpagercharset/pager_charset.h"

static int failures = 0;

#define CHECK(cond) do { if (!(cond)) { printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

/* the codes of a text in a codepage, -1 if a character has none */
static int codes(const char *key, const char *text, uint8_t *out, int size)
{
	const struct pager_charset *cs = pager_charset_lookup("nmt", key);

	if (!cs)
		return -1;
	return pager_charset_encode_text(cs, text, strlen(text), out, size, NULL);
}

/* the text decoded from codes of a codepage is the text encoded */
static int round_trip(const char *key, const char *text)
{
	const struct pager_charset *cs = pager_charset_lookup("nmt", key);
	uint8_t c[256];
	char out[1024];
	int n = codes(key, text, c, sizeof(c));

	if (!cs || n < 0)
		return 0;
	pager_charset_decode_text(cs, c, n, out, sizeof(out));
	return strcmp(out, text) == 0;
}

int main(void)
{
	uint8_t c[64];
	int n;

	/* each codepage of Annex 24 4.3 round trips a text of its letters */
	CHECK(round_trip("generic-generic-gsm", "Hello @ £ ÄÖÜ ¿ ΔΩ"));
	CHECK(round_trip("generic-generic-cyrillic", "Привіт ЄЇҐ ёЁ"));
	CHECK(round_trip("generic-generic-baltic", "Šķēle ŊŧĐ"));
	CHECK(round_trip("generic-generic-central", "Łódź ŐűĞİı ůŮ"));
	CHECK(round_trip("generic-generic-western", "Þórður Ðê"));
	CHECK(round_trip("generic-generic-thai", "สวัสดี ฿๑"));

	/* the codes of the standard (set 0, 4.3.1) and of the Nokia read on air */
	n = codes("generic-generic-cyrillic", "ЄІЇАЯЁяё", c, sizeof(c));
	CHECK(n == 8 && c[0] == 0x80 && c[1] == 0x81 && c[2] == 0x82 && c[3] == 0x83
	      && c[4] == 0xC2 && c[5] == 0xC4 && c[6] == 0xE2 && c[7] == 0xE4);
	n = codes("nokia-generic-cyrillic", "ABC-УЇЖЄ", c, sizeof(c));
	CHECK(n == 8 && c[1] == 0x42 && c[4] == 0x96 && c[5] == 0x82 && c[6] == 0x89 && c[7] == 0x80);
	CHECK(pager_charset_lookup("nmt", "nokia-generic-cyrillic")->entries
	      == pager_charset_lookup("nmt", "generic-generic-cyrillic")->entries);

	/* a letter also in the GSM half goes as its GSM code (set 2 0x82, set 3 0xA0) */
	n = codes("generic-generic-central", "Ç", c, sizeof(c));
	CHECK(n == 1 && c[0] == 0x09);
	n = codes("generic-generic-western", "à", c, sizeof(c));
	CHECK(n == 1 && c[0] == 0x7F);

	/* CR and LF: one-line glyphs on RX, the control characters still taken on TX */
	CHECK(!strcmp(pager_charset_decode(pager_charset_lookup("nmt", "generic-generic-gsm"), 0x0A), "␊"));
	CHECK(!strcmp(pager_charset_decode(pager_charset_lookup("nmt", "generic-generic-cyrillic"), 0x0D), "␍"));
	n = codes("generic-generic-gsm", "a\nb\r", c, sizeof(c));
	CHECK(n == 4 && c[1] == 0x0A && c[3] == 0x0D);

	/* the escape to the extension table is no character; GSM has no Cyrillic */
	CHECK(pager_charset_decode(pager_charset_lookup("nmt", "generic-generic-gsm"), 0x1B) == NULL);
	CHECK(codes("generic-generic-gsm", "Привіт", c, sizeof(c)) < 0);

	if (failures) {
		printf("%d failures\n", failures);
		return EXIT_FAILURE;
	}
	printf("NMT codepage tests passed\n");
	return EXIT_SUCCESS;
}
