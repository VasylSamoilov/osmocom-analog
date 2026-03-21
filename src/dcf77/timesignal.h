/* Generic LF time signal protocol definitions
 *
 * Supports DCF77, WWVB, MSF, JJY40, JJY60
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef TIMESIGNAL_H
#define TIMESIGNAL_H

#include <time.h>
#include <stdint.h>

/* Time signal service types */
enum time_service {
	TIME_SERVICE_DCF77 = 0,
	TIME_SERVICE_WWVB,
	TIME_SERVICE_MSF,
	TIME_SERVICE_JJY40,
	TIME_SERVICE_JJY60,
	TIME_SERVICE_COUNT
};

/* Carrier power levels for modulation */
enum carrier_power {
	CARRIER_OFF = 0,	/* carrier completely off (used by MSF) */
	CARRIER_LOW,		/* carrier attenuated to ~15% (used by DCF77, WWVB) */
	CARRIER_HIGH		/* full carrier power */
};

/* A single modulation segment within one second */
struct mod_segment {
	enum carrier_power power;
	int duration_ms;	/* 0 = fill remainder of second */
};

#define MAX_MOD_SEGMENTS 4

/* Modulation pattern for one second */
struct second_mod {
	struct mod_segment segments[MAX_MOD_SEGMENTS];
	int count;
};

/* Protocol descriptor — filled in by each protocol implementation */
struct time_protocol {
	const char *name;
	double carrier_frequency;	/* Hz */
	/* Prepare the 59/60-bit frame for the given minute */
	void (*prepare_minute)(struct time_protocol *proto, time_t t);
	/* Get modulation pattern for a given second (0-59) */
	struct second_mod (*get_modulation)(struct time_protocol *proto, int second);
	/* Protocol-private data */
	uint64_t time_bits;
	uint64_t time_bits_b;	/* MSF uses two bit streams (A and B) */
};

/* Protocol init functions */
void dcf77_protocol_init(struct time_protocol *proto);
void wwvb_protocol_init(struct time_protocol *proto);
void msf_protocol_init(struct time_protocol *proto);
void jjy40_protocol_init(struct time_protocol *proto);
void jjy60_protocol_init(struct time_protocol *proto);

/* Lookup service by name, returns -1 if not found */
int time_service_by_name(const char *name);
const char *time_service_name(int service);
double time_service_frequency(int service);
void time_service_init_protocol(int service, struct time_protocol *proto);

#endif /* TIMESIGNAL_H */
