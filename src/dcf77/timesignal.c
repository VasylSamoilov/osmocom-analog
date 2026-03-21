/* Generic LF time signal protocol support
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <string.h>
#include <strings.h>
#include "timesignal.h"

static const struct {
	const char *name;
	double frequency;
	void (*init)(struct time_protocol *proto);
} services[TIME_SERVICE_COUNT] = {
	[TIME_SERVICE_DCF77] = { "DCF77",  77500.0, dcf77_protocol_init },
	[TIME_SERVICE_WWVB]  = { "WWVB",   60000.0, wwvb_protocol_init },
	[TIME_SERVICE_MSF]   = { "MSF",    60000.0, msf_protocol_init },
	[TIME_SERVICE_JJY40] = { "JJY40",  40000.0, jjy40_protocol_init },
	[TIME_SERVICE_JJY60] = { "JJY60",  60000.0, jjy60_protocol_init },
};

int time_service_by_name(const char *name)
{
	int i;
	for (i = 0; i < TIME_SERVICE_COUNT; i++) {
		if (!strcasecmp(name, services[i].name))
			return i;
	}
	return -1;
}

const char *time_service_name(int service)
{
	if (service < 0 || service >= TIME_SERVICE_COUNT)
		return "???";
	return services[service].name;
}

double time_service_frequency(int service)
{
	if (service < 0 || service >= TIME_SERVICE_COUNT)
		return 0.0;
	return services[service].frequency;
}

void time_service_init_protocol(int service, struct time_protocol *proto)
{
	if (service < 0 || service >= TIME_SERVICE_COUNT)
		return;
	memset(proto, 0, sizeof(*proto));
	services[service].init(proto);
}
