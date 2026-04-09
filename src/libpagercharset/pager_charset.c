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
	/* Golay/GSC */
	&pager_charset_golay_generic_generic_ascii,
	/* FLEX */
	&pager_charset_flex_generic_generic_ascii,
	/* RDS */
	&pager_charset_rds_generic_generic_annex_e,
	&pager_charset_rds_nokia_nkp2_annex_e,
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
