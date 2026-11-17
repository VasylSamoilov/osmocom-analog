/*
 * Pager character set library — lookup and conversion functions.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RX: code → UTF-8 (direct array index, O(1))
 * TX: UTF-8 char → code (linear scan, checking primary then aliases)
 */

#include "pager_charset.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>

/* Registry of all built-in codepages.  Add new entries here. */
static const struct pager_charset *registry[] = {
	/* POCSAG */
	&pager_charset_pocsag_generic_generic_ascii,
	&pager_charset_pocsag_motorola_advisor_ascii,
	&pager_charset_pocsag_generic_generic_latin_german,
	&pager_charset_pocsag_generic_generic_latin_swiss,
	&pager_charset_pocsag_generic_generic_latin_french,
	&pager_charset_pocsag_generic_generic_latin_swedish,
	&pager_charset_pocsag_generic_generic_latin_portuguese,
	&pager_charset_pocsag_generic_generic_latin_spanish,
	&pager_charset_pocsag_generic_generic_latin_british,
	&pager_charset_pocsag_generic_generic_latin_italian,
	&pager_charset_pocsag_generic_generic_latin_danish_norwegian,
	&pager_charset_pocsag_generic_generic_ascii_uppercase,
	&pager_charset_pocsag_motorola_bravo_alpha_ascii_uppercase,
	&pager_charset_pocsag_motorola_advisor_linguist_latin_cyrillic,
	&pager_charset_pocsag_motorola_scriptor_lx2_latin_cyrillic,
	&pager_charset_pocsag_nec_maxima_latin_cyrillic,
	&pager_charset_pocsag_nec_optima_latin_cyrillic,
	&pager_charset_pocsag_truly_supervisor_latin_cyrillic,
	&pager_charset_pocsag_philips_prg2220_latin_cyrillic,
	&pager_charset_pocsag_motorola_advisor_cyrillic,
	&pager_charset_pocsag_nec_21a_latin_cyrillic,
	&pager_charset_pocsag_nec_maxima_compact_cyrillic,
	&pager_charset_pocsag_swissphone_de506_ascii,
	&pager_charset_pocsag_swissphone_de506_cyrillic,
	&pager_charset_pocsag_partner_generic_cyrillic,
	&pager_charset_pocsag_multitone_generic_ascii,
	&pager_charset_pocsag_nec_skyper_ascii_rot1,
	/* Golay/GSC */
	&pager_charset_golay_generic_generic_ascii,
	/* FLEX */
	&pager_charset_flex_generic_generic_ascii,
	&pager_charset_flex_motorola_linguist_cyrillic,
	&pager_charset_flex_motorola_linguist_icelandic,
	&pager_charset_flex_motorola_linguist_polish,
	&pager_charset_flex_motorola_linguist_vietnamese,
	&pager_charset_flex_motorola_linguist_arabic,
	&pager_charset_flex_motorola_linguist_arabic_english,
	&pager_charset_flex_motorola_linguist_turkish,
	&pager_charset_flex_motorola_linguist_hebrew,
	&pager_charset_flex_motorola_linguist_greek,
	&pager_charset_flex_motorola_linguist_latin1,
	&pager_charset_flex_motorola_linguist_latin2,
	&pager_charset_flex_motorola_linguist_latin3,
	&pager_charset_flex_motorola_linguist_latin4,
	/* RDS */
	&pager_charset_rds_generic_generic_annex_e,
	&pager_charset_rds_nokia_nkp2_annex_e,
	&pager_charset_rds_nokia_finder_text_cyrillic,
	NULL,
};

static int field_match(const char *pattern, const char *value)
{
	if (!pattern)
		return 1;
	return strcasecmp(pattern, value) == 0;
}

const struct pager_charset *pager_charset_find(const char *protocol,
					       const char *vendor,
					       const char *model,
					       const char *charset)
{
	int i;

	for (i = 0; registry[i]; i++) {
		if (field_match(protocol, registry[i]->protocol) &&
		    field_match(vendor, registry[i]->vendor) &&
		    field_match(model, registry[i]->model) &&
		    field_match(charset, registry[i]->charset))
			return registry[i];
	}
	return NULL;
}

const struct pager_charset *pager_charset_by_name(const char *name)
{
	int i;
	char buf[256];
	size_t len;

	if (!name)
		return NULL;

	for (i = 0; registry[i]; i++) {
		len = snprintf(buf, sizeof(buf), "%s-%s-%s-%s",
			       registry[i]->protocol, registry[i]->vendor,
			       registry[i]->model, registry[i]->charset);
		if (len < sizeof(buf) && strcasecmp(buf, name) == 0)
			return registry[i];
	}
	return NULL;
}

const char *pager_charset_decode(const struct pager_charset *cs, uint8_t code)
{
	int mask;

	if (!cs)
		return NULL;
	mask = cs->num_codes - 1;
	code &= mask;
	return cs->entries[code][0];
}

static int utf8_char_len(unsigned char c)
{
	if ((c & 0x80) == 0x00) return 1;
	if ((c & 0xE0) == 0xC0) return 2;
	if ((c & 0xF0) == 0xE0) return 3;
	if ((c & 0xF8) == 0xF0) return 4;
	return 1;
}

int pager_charset_encode(const struct pager_charset *cs, const char **utf8)
{
	const char *p;
	int clen, i, j;

	if (!cs || !utf8 || !*utf8 || !**utf8)
		return -1;

	p = *utf8;
	clen = utf8_char_len((unsigned char)*p);

	/* A glyph of several characters (Arabic lam-alef) first, the longest one */
	{
		int best = -1, best_len = clen;

		for (i = 0; i < cs->num_codes; i++) {
			for (j = 0; j < PAGER_CHARSET_ENTRY_MAX && cs->entries[i][j]; j++) {
				int n = (int)strlen(cs->entries[i][j]);

				if (n > best_len && strncmp(cs->entries[i][j], p, n) == 0) {
					best = i;
					best_len = n;
				}
			}
		}
		if (best >= 0) {
			*utf8 = p + best_len;
			return best;
		}
	}

	for (i = 0; i < cs->num_codes; i++) {
		/* Check primary [0] and all aliases [1..] */
		for (j = 0; j < PAGER_CHARSET_ENTRY_MAX && cs->entries[i][j]; j++) {
			if ((int)strlen(cs->entries[i][j]) == clen &&
			    memcmp(cs->entries[i][j], p, clen) == 0) {
				*utf8 = p + clen;
				return i;
			}
		}
	}

	*utf8 = p + clen;
	return -1;
}

void pager_charset_list(void)
{
	int i;

	printf("Available pager charsets:\n");
	printf("  %-8s %-12s %-20s %-20s %s\n",
	       "Protocol", "Vendor", "Model", "Charset", "Bits");
	printf("  %-8s %-12s %-20s %-20s %s\n",
	       "--------", "------", "-----", "-------", "----");
	for (i = 0; registry[i]; i++) {
		printf("  %-8s %-12s %-20s %-20s %d\n",
		       registry[i]->protocol,
		       registry[i]->vendor,
		       registry[i]->model,
		       registry[i]->charset,
		       registry[i]->code_bits);
	}
}

const struct pager_charset *pager_charset_lookup(const char *protocol, const char *key)
{
	int i;
	char buf[256];
	size_t len;

	if (!protocol || !key)
		return NULL;

	for (i = 0; registry[i]; i++) {
		if (strcasecmp(registry[i]->protocol, protocol))
			continue;
		len = snprintf(buf, sizeof(buf), "%s-%s-%s",
			       registry[i]->vendor, registry[i]->model, registry[i]->charset);
		if (len < sizeof(buf) && strcasecmp(buf, key) == 0)
			return registry[i];
	}
	return NULL;
}

void pager_charset_list_protocol(const char *protocol)
{
	int i;

	for (i = 0; registry[i]; i++) {
		if (protocol && strcasecmp(registry[i]->protocol, protocol))
			continue;
		printf("  %s-%s-%s\n", registry[i]->vendor, registry[i]->model, registry[i]->charset);
	}
}

/* A glyph for the table: LF, CR and space by name */
static const char *print_glyph(const char *glyph)
{
	if (!strcmp(glyph, "\n"))
		return "<LF>";
	if (!strcmp(glyph, "\r"))
		return "<CR>";
	if (!strcmp(glyph, " "))
		return "<SP>";
	return glyph;
}

void pager_charset_print(const struct pager_charset *cs)
{
	int i, j;

	if (!cs)
		return;

	printf("%s %s-%s-%s (%d-bit, %d codes)\n", cs->protocol, cs->vendor, cs->model,
	       cs->charset, cs->code_bits, cs->num_codes);
	for (i = 0; i < cs->num_codes; i++) {
		const char *glyph = cs->entries[i][0];

		if (!glyph) {
			printf("0x%02X  <0x%02X>\n", i, i);
			continue;
		}
		printf("0x%02X  %s", i, print_glyph(glyph));
		for (j = 1; j < PAGER_CHARSET_ENTRY_MAX && cs->entries[i][j]; j++)
			printf("%s%s", (j == 1) ? "  TX also: " : " ", print_glyph(cs->entries[i][j]));
		printf("\n");
	}
}

/* <0xNN> at p: its value, else -1 */
static int escaped_code(const char *p)
{
	int v = 0, i;

	if (strncasecmp(p, "<0x", 3))
		return -1;
	for (i = 3; i < 5; i++) {
		char c = p[i];

		v <<= 4;
		if (c >= '0' && c <= '9')
			v |= c - '0';
		else if (c >= 'a' && c <= 'f')
			v |= c - 'a' + 10;
		else if (c >= 'A' && c <= 'F')
			v |= c - 'A' + 10;
		else
			return -1;
	}
	return (p[5] == '>') ? v : -1;
}

int pager_charset_encode_text(const struct pager_charset *cs, const char *utf8, int length,
			      uint8_t *out, int out_size, const char **bad)
{
	const char *p = utf8, *start, *end;
	int n = 0, code;
	char ch[8];

	if (!cs || !utf8 || !out)
		return -1;
	end = utf8 + length;

	while (p < end) {
		unsigned char c = (unsigned char)*p;

		if (n >= out_size)
			return -1;
		if ((c < 0x20 || c == 0x7f) && cs->code_bits >= 7) {
			/* The code of its glyph (LF, CR), else its own */
			const char *q = ch;

			ch[0] = (char)c;
			ch[1] = '\0';
			code = c ? pager_charset_encode(cs, &q) : -1;
			out[n++] = (code >= 0) ? (uint8_t)code : c;
			p++;
			continue;
		}
		code = (end - p >= 6) ? escaped_code(p) : -1;
		if (code >= 0 && code < cs->num_codes) {
			out[n++] = (uint8_t)code;
			p += 6;
			continue;
		}
		/* One character, terminated, for the lookup */
		start = p;
		{
			int clen = utf8_char_len(c), i;
			const char *q = ch;

			if (clen > end - p)
				clen = end - p;
			for (i = 0; i < clen; i++)
				ch[i] = p[i];
			/* The rest of the text for a glyph of several characters */
			for (; i < (int)sizeof(ch) - 1 && p + i < end && p[i]; i++)
				ch[i] = p[i];
			ch[i] = '\0';
			code = pager_charset_encode(cs, &q);
			p += q - ch;
		}
		if (code < 0) {
			if (bad)
				*bad = start;
			return -1;
		}
		out[n++] = (uint8_t)code;
	}
	return n;
}

int pager_charset_decode_text(const struct pager_charset *cs, const uint8_t *codes,
			      int length, char *out, int out_size)
{
	int i, n = 0, len;
	char esc[8];
	const char *glyph;

	if (!out || out_size <= 0)
		return 0;
	out[0] = '\0';
	if (!cs || !codes)
		return 0;

	for (i = 0; i < length; i++) {
		glyph = pager_charset_decode(cs, codes[i]);
		if (!glyph) {
			snprintf(esc, sizeof(esc), "<0x%02X>", codes[i] & (cs->num_codes - 1));
			glyph = esc;
		}
		len = strlen(glyph);
		if (n + len >= out_size)
			break;
		memcpy(out + n, glyph, len);
		n += len;
	}
	out[n] = '\0';
	return n;
}
