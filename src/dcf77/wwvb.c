/* WWVB time signal protocol (USA, 60 kHz)
 *
 * Based on https://en.wikipedia.org/wiki/WWVB
 * Reference: txtempus by Henner Zeller
 *
 * WWVB transmits BCD-encoded UTC time with amplitude modulation:
 *   200ms low = bit 0
 *   500ms low = bit 1
 *   800ms low = sync/marker
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

/* WWVB uses padded BCD with a zero bit between digit groups */
static uint64_t to_padded5_bcd(int n)
{
	return (((n / 100) % 10) << 10) | (((n / 10) % 10) << 5) | (n % 10);
}

static int is_leap_year(int year)
{
	return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

static void wwvb_prepare_minute(struct time_protocol *proto, time_t t)
{
	struct tm breakdown;
	struct tm local_now, local_tomorrow;
	time_t tomorrow_t;

	gmtime_r(&t, &breakdown);  /* WWVB transmits UTC */

	proto->time_bits = 0;
	proto->time_bits |= to_padded5_bcd(breakdown.tm_min) << (59 - 8);
	proto->time_bits |= to_padded5_bcd(breakdown.tm_hour) << (59 - 18);
	proto->time_bits |= to_padded5_bcd(breakdown.tm_yday + 1) << (59 - 33);
	proto->time_bits |= to_padded5_bcd(breakdown.tm_year % 100) << (59 - 53);
	proto->time_bits |= (uint64_t)is_leap_year(breakdown.tm_year + 1900) << (59 - 55);

	/* DST bits from local time */
	localtime_r(&t, &local_now);
	tomorrow_t = t + 86400;
	localtime_r(&tomorrow_t, &local_tomorrow);
	proto->time_bits |= (uint64_t)(local_tomorrow.tm_isdst ? 1 : 0) << (59 - 57);
	proto->time_bits |= (uint64_t)(local_now.tm_isdst ? 1 : 0) << (59 - 58);
}

static struct second_mod wwvb_get_modulation(struct time_protocol *proto, int sec)
{
	struct second_mod mod;

	memset(&mod, 0, sizeof(mod));

	if (sec == 0 || sec % 10 == 9 || sec > 59) {
		/* sync/marker: 800ms low */
		mod.segments[0].power = CARRIER_LOW;
		mod.segments[0].duration_ms = 800;
		mod.segments[1].power = CARRIER_HIGH;
		mod.segments[1].duration_ms = 0;
		mod.count = 2;
	} else {
		int bit = (proto->time_bits >> (59 - sec)) & 1;
		/* bit 1 = 500ms low, bit 0 = 200ms low */
		mod.segments[0].power = CARRIER_LOW;
		mod.segments[0].duration_ms = bit ? 500 : 200;
		mod.segments[1].power = CARRIER_HIGH;
		mod.segments[1].duration_ms = 0;
		mod.count = 2;
	}
	return mod;
}

void wwvb_protocol_init(struct time_protocol *proto)
{
	proto->name = "WWVB";
	proto->carrier_frequency = 60000.0;
	proto->prepare_minute = wwvb_prepare_minute;
	proto->get_modulation = wwvb_get_modulation;
}
