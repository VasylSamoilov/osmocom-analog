/* JJY time signal protocol (Japan, 40 kHz and 60 kHz)
 *
 * Based on https://en.wikipedia.org/wiki/JJY
 * Reference: txtempus by Henner Zeller
 *
 * JJY is similar to WWVB but with inverted power levels:
 *   HIGH then LOW (vs WWVB's LOW then HIGH)
 *   200ms high = sync/marker
 *   500ms high = bit 1
 *   800ms high = bit 0
 * Transmits Japan Standard Time (JST = UTC+9).
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <time.h>
#include <stdint.h>
#include <string.h>
#include "timesignal.h"

/* Padded BCD with zero bit between digit groups (same as WWVB) */
static uint64_t to_padded5_bcd(int n)
{
	return (((n / 100) % 10) << 10) | (((n / 10) % 10) << 5) | (n % 10);
}

static uint64_t to_bcd(int n)
{
	return (((n / 100) % 10) << 8) | (((n / 10) % 10) << 4) | (n % 10);
}

static int parity(uint64_t d, int from, int to_including)
{
	int result = 0;
	int bit;
	for (bit = from; bit <= to_including; bit++) {
		if (d & (1ULL << bit))
			result++;
	}
	return result & 1;
}

static void jjy_prepare_minute(struct time_protocol *proto, time_t t)
{
	struct tm breakdown;

	localtime_r(&t, &breakdown);  /* If in JP, this is JST */

	proto->time_bits = 0;
	proto->time_bits |= to_padded5_bcd(breakdown.tm_min) << (59 - 8);
	proto->time_bits |= to_padded5_bcd(breakdown.tm_hour) << (59 - 18);
	proto->time_bits |= to_padded5_bcd(breakdown.tm_yday + 1) << (59 - 33);
	proto->time_bits |= to_bcd(breakdown.tm_year % 100) << (59 - 48);
	proto->time_bits |= to_bcd(breakdown.tm_wday) << (59 - 52);

	/* Parity bits */
	proto->time_bits |= (uint64_t)parity(proto->time_bits, 59 - 18, 59 - 12) << (59 - 36);  /* PA1 */
	proto->time_bits |= (uint64_t)parity(proto->time_bits, 59 - 8, 59 - 1) << (59 - 37);    /* PA2 */
}

static struct second_mod jjy_get_modulation(struct time_protocol *proto, int sec)
{
	struct second_mod mod;

	memset(&mod, 0, sizeof(mod));

	if (sec == 0 || sec % 10 == 9 || sec > 59) {
		/* sync/marker: 200ms HIGH then LOW */
		mod.segments[0].power = CARRIER_HIGH;
		mod.segments[0].duration_ms = 200;
		mod.segments[1].power = CARRIER_LOW;
		mod.segments[1].duration_ms = 0;
		mod.count = 2;
	} else {
		int bit = (proto->time_bits >> (59 - sec)) & 1;
		/* bit 1 = 500ms HIGH, bit 0 = 800ms HIGH, then LOW */
		mod.segments[0].power = CARRIER_HIGH;
		mod.segments[0].duration_ms = bit ? 500 : 800;
		mod.segments[1].power = CARRIER_LOW;
		mod.segments[1].duration_ms = 0;
		mod.count = 2;
	}
	return mod;
}

void jjy40_protocol_init(struct time_protocol *proto)
{
	proto->name = "JJY40";
	proto->carrier_frequency = 40000.0;
	proto->prepare_minute = jjy_prepare_minute;
	proto->get_modulation = jjy_get_modulation;
}

void jjy60_protocol_init(struct time_protocol *proto)
{
	proto->name = "JJY60";
	proto->carrier_frequency = 60000.0;
	proto->prepare_minute = jjy_prepare_minute;
	proto->get_modulation = jjy_get_modulation;
}
