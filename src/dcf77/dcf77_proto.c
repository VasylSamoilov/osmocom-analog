/* DCF77 time signal protocol wrapper for generic interface
 *
 * This wraps the DCF77 encoding into the generic time_protocol interface,
 * matching the txtempus DCF77 implementation for the basic time bits.
 * Weather data is handled separately by the existing dcf77.c code.
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

static uint64_t to_bcd(int n)
{
	return (((n / 10) % 10) << 4) | (n % 10);
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

static void dcf77_proto_prepare_minute(struct time_protocol *proto, time_t t)
{
	struct tm breakdown;

	t += 60;  /* DCF77 sends the upcoming minute */
	localtime_r(&t, &breakdown);

	proto->time_bits = 0;
	proto->time_bits |= (uint64_t)(breakdown.tm_isdst ? 1 : 0) << 17;
	proto->time_bits |= (uint64_t)(breakdown.tm_isdst ? 0 : 1) << 18;
	proto->time_bits |= (uint64_t)1 << 20;  /* start time bit */
	proto->time_bits |= to_bcd(breakdown.tm_min) << 21;
	proto->time_bits |= to_bcd(breakdown.tm_hour) << 29;
	proto->time_bits |= to_bcd(breakdown.tm_mday) << 36;
	proto->time_bits |= to_bcd(breakdown.tm_wday ? breakdown.tm_wday : 7) << 42;
	proto->time_bits |= to_bcd(breakdown.tm_mon + 1) << 45;
	proto->time_bits |= to_bcd(breakdown.tm_year % 100) << 50;

	proto->time_bits |= (uint64_t)parity(proto->time_bits, 21, 27) << 28;
	proto->time_bits |= (uint64_t)parity(proto->time_bits, 29, 34) << 35;
	proto->time_bits |= (uint64_t)parity(proto->time_bits, 36, 57) << 58;
}

static struct second_mod dcf77_proto_get_modulation(struct time_protocol *proto, int second)
{
	struct second_mod mod;

	memset(&mod, 0, sizeof(mod));

	if (second >= 59) {
		/* second 59: no reduction (minute sync) */
		mod.segments[0].power = CARRIER_HIGH;
		mod.segments[0].duration_ms = 0;
		mod.count = 1;
		return mod;
	}

	{
		int bit = (proto->time_bits >> second) & 1;
		/* bit 1 = 200ms low, bit 0 = 100ms low */
		mod.segments[0].power = CARRIER_LOW;
		mod.segments[0].duration_ms = bit ? 200 : 100;
		mod.segments[1].power = CARRIER_HIGH;
		mod.segments[1].duration_ms = 0;
		mod.count = 2;
	}
	return mod;
}

void dcf77_protocol_init(struct time_protocol *proto)
{
	proto->name = "DCF77";
	proto->carrier_frequency = 77500.0;
	proto->prepare_minute = dcf77_proto_prepare_minute;
	proto->get_modulation = dcf77_proto_get_modulation;
}
