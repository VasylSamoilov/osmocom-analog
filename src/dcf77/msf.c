/* MSF time signal protocol (United Kingdom, 60 kHz)
 *
 * Based on https://en.wikipedia.org/wiki/Time_from_NPL_(MSF)
 * Reference: txtempus by Henner Zeller
 *
 * MSF transmits two bits per second (A and B streams) using on-off keying:
 *   Second 0: 500ms OFF (minute marker)
 *   Others: 100ms OFF, then A-bit (100ms OFF or HIGH), then B-bit (100ms OFF or HIGH)
 * Sends upcoming minute in local UK time (BST/GMT).
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

static uint64_t odd_parity(uint64_t d, int from, int to_including)
{
	int result = 0;
	int bit;
	for (bit = from; bit <= to_including; bit++) {
		if (d & (1ULL << bit))
			result++;
	}
	return (result & 1) == 0;
}

static void msf_prepare_minute(struct time_protocol *proto, time_t t)
{
	struct tm breakdown;

	t += 60;  /* MSF sends the upcoming minute */
	localtime_r(&t, &breakdown);

	/* A bits (time_bits) */
	proto->time_bits = 0b1111110ULL;  /* bits 52-58: minute ID */
	proto->time_bits |= to_bcd(breakdown.tm_year % 100) << (59 - 24);
	proto->time_bits |= to_bcd(breakdown.tm_mon + 1) << (59 - 29);
	proto->time_bits |= to_bcd(breakdown.tm_mday) << (59 - 35);
	proto->time_bits |= to_bcd(breakdown.tm_wday) << (59 - 38);
	proto->time_bits |= to_bcd(breakdown.tm_hour) << (59 - 44);
	proto->time_bits |= to_bcd(breakdown.tm_min) << (59 - 51);

	/* B bits (time_bits_b) — parity and DST */
	proto->time_bits_b = 0;
	proto->time_bits_b |= odd_parity(proto->time_bits, 59 - 24, 59 - 17) << (59 - 54);
	proto->time_bits_b |= odd_parity(proto->time_bits, 59 - 35, 59 - 25) << (59 - 55);
	proto->time_bits_b |= odd_parity(proto->time_bits, 59 - 38, 59 - 36) << (59 - 56);
	proto->time_bits_b |= odd_parity(proto->time_bits, 59 - 51, 59 - 39) << (59 - 57);
	proto->time_bits_b |= (uint64_t)(breakdown.tm_isdst ? 1 : 0) << (59 - 58);
}

static struct second_mod msf_get_modulation(struct time_protocol *proto, int second)
{
	struct second_mod mod;
	int a, b;

	memset(&mod, 0, sizeof(mod));

	if (second == 0) {
		/* minute marker: 500ms OFF */
		mod.segments[0].power = CARRIER_OFF;
		mod.segments[0].duration_ms = 500;
		mod.segments[1].power = CARRIER_HIGH;
		mod.segments[1].duration_ms = 0;
		mod.count = 2;
		return mod;
	}

	a = (proto->time_bits >> (59 - second)) & 1;
	b = (proto->time_bits_b >> (59 - second)) & 1;

	/* 100ms OFF, then A-bit segment (100ms), then B-bit segment (100ms), then HIGH */
	mod.segments[0].power = CARRIER_OFF;
	mod.segments[0].duration_ms = 100;
	mod.segments[1].power = a ? CARRIER_OFF : CARRIER_HIGH;
	mod.segments[1].duration_ms = 100;
	mod.segments[2].power = b ? CARRIER_OFF : CARRIER_HIGH;
	mod.segments[2].duration_ms = 100;
	mod.segments[3].power = CARRIER_HIGH;
	mod.segments[3].duration_ms = 0;
	mod.count = 4;

	return mod;
}

void msf_protocol_init(struct time_protocol *proto)
{
	proto->name = "MSF";
	proto->carrier_frequency = 60000.0;
	proto->prepare_minute = msf_prepare_minute;
	proto->get_modulation = msf_get_modulation;
}
