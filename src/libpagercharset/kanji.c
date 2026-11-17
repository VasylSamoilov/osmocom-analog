/*
 * FLEX Japanese Text Service: Shift JIS characters of HEX/binary messages.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * STD-43A 1.3.1 (Table 1.3.1-2: HEX vector, Shift JIS code), 3.10.1.2: the
 * text is a HEX/binary message of characters of the blocking length B,
 * 16 bits (B = 0000) for Shift JIS: a 2-byte code of a KANJI character
 * (byte 1 0x81-0x9F or 0xE0-0xFC) or the code of a 1-byte character; 8 bits
 * for the 1-byte characters only.  The data bits of a character are sent
 * the least significant first: as hex digits (4 bits each, the first bit
 * the least significant) a 16-bit character is 4 digits, the lowest first.
 */

#include "pager_charset.h"
#include <iconv.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static int lead_byte(unsigned char b)
{
	return (b >= 0x81 && b <= 0x9f) || (b >= 0xe0 && b <= 0xfc);
}

int pager_kanji_encode(const char *utf8, int length, int blocking, char *hex, int hex_size,
		       const char **error)
{
	iconv_t cd;
	char *in, *out, *sjis;
	size_t in_left, out_left, size;
	int i, n = 0, d, digits;

	if (blocking != 8 && blocking != 16) {
		if (error)
			*error = "Kanji: a blocking length of 8 or 16 bits";
		return -1;
	}
	digits = blocking / 4;

	cd = iconv_open("SHIFT_JIS", "UTF-8");
	if (cd == (iconv_t)-1) {
		if (error)
			*error = "Kanji: no Shift JIS conversion on this system";
		return -1;
	}
	size = (size_t)length * 2 + 1;
	sjis = malloc(size);
	if (!sjis) {
		iconv_close(cd);
		return -1;
	}
	in = (char *)utf8;
	in_left = length;
	out = sjis;
	out_left = size;
	if (iconv(cd, &in, &in_left, &out, &out_left) == (size_t)-1) {
		iconv_close(cd);
		free(sjis);
		if (error)
			*error = "Kanji: a character has no Shift JIS code";
		return -1;
	}
	iconv_close(cd);
	size -= out_left;

	for (i = 0; i < (int)size; i++) {
		unsigned value = (unsigned char)sjis[i];

		if (lead_byte(value) && i + 1 < (int)size) {
			if (blocking == 8) {
				free(sjis);
				if (error)
					*error = "Kanji: a 2-byte character in 8-bit characters";
				return -1;
			}
			value = (value << 8) | (unsigned char)sjis[++i];
		}
		if (n + digits >= hex_size) {
			free(sjis);
			if (error)
				*error = "Kanji: text too long";
			return -1;
		}
		for (d = 0; d < digits; d++)
			hex[n++] = "0123456789ABCDEF"[(value >> (4 * d)) & 0xf];
	}
	hex[n] = '\0';
	free(sjis);
	return n;
}

static int digit_value(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return -1;
}

/* The UTF-8 text of one Shift JIS character; <0xNN> or <0xNNNN> if it has none */
static void sjis_char(iconv_t cd, unsigned value, char *out, size_t out_size)
{
	char bytes[2], *in = bytes, *o = out;
	size_t in_left, out_left = out_size - 1;

	if (value > 0xff) {
		bytes[0] = (char)(value >> 8);
		bytes[1] = (char)(value & 0xff);
		in_left = 2;
	} else {
		bytes[0] = (char)value;
		in_left = 1;
	}
	if (cd != (iconv_t)-1)
		iconv(cd, NULL, NULL, NULL, NULL);
	if (cd == (iconv_t)-1 || iconv(cd, &in, &in_left, &o, &out_left) == (size_t)-1 || in_left) {
		snprintf(out, out_size, (value > 0xff) ? "<0x%04X>" : "<0x%02X>", value);
		return;
	}
	*o = '\0';
}

int pager_kanji_decode(const char *hex, int blocking, int right_to_left, char *out, int out_size)
{
	const int digits = (blocking == 8) ? 2 : 4;
	int count = (int)strlen(hex) / digits + 1, nchars = 0, i, n = 0, len;
	char (*chars)[16];
	iconv_t cd;
	const char *p = hex;

	if (out_size <= 0)
		return 0;
	out[0] = '\0';
	chars = calloc(count, sizeof(*chars));
	if (!chars)
		return 0;
	cd = iconv_open("UTF-8", "SHIFT_JIS");

	while (*p) {
		unsigned value = 0;
		int lost = 0, d;

		for (d = 0; d < digits && *p; d++, p++) {
			int v = digit_value(*p);

			if (v < 0)
				lost = 1;
			else
				value |= (unsigned)v << (4 * d);
		}
		if (lost || d < digits)
			strcpy(chars[nchars++], "?");
		else if (value != 0)
			sjis_char(cd, value, chars[nchars++], sizeof(chars[0]));
	}
	if (cd != (iconv_t)-1)
		iconv_close(cd);

	/* Display direction D = 1 (3.10.1.2): the first character is shown at the right */
	for (i = 0; i < nchars; i++) {
		const char *c = chars[right_to_left ? nchars - 1 - i : i];

		len = strlen(c);
		if (n + len >= out_size)
			break;
		memcpy(out + n, c, len);
		n += len;
	}
	out[n] = '\0';
	free(chars);
	return n;
}
