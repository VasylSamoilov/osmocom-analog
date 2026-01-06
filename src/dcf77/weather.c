
/* implementation of DCF77 transmitter and receiver, including weather
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "../liblogging/logging.h"
#include "dcf77.h"
#include "weather.h"
#include "weather_crypt.h"
#include "weather_pic.h"

double get_time(void);

const char *datasets_0_59[8] = {
	"Maximum values 1st day (today)",
	"Minimum values 1st day (today)",
	"Maximum values 2nd day (tomorrow)",
	"Minimum values 2nd day (tomorrow)",
	"Maximum values 3rd day (in two days)",
	"Minimum values 3rd day (in two days)",
	"Maximum values 4th day (in three days)",
	"Wind and weather anomaly 4th day (in three days)",
};

const char *datasets_60_89[2] = {
	"Maximum values 1st day (following day)",
	"Maximum values 2nd day (2nd following day)",
};

const char *region_name[90] = {
	"F - Bordeaux, Aquitaine (Suedwestfrankreich)",
	"F - La Rochelle, Poitou-Charentes (Westkueste Frankreichs)",
	"F - Paris, Ile-de-France (Pariser Becken)",
	"F - Brest, Bretagne",
	"F - Clermont-Ferrand (Massif Central), Auvergne (Zentralmassiv)",
	"F - Beziers, Languedoc-Roussillon",
	"B - Bruxelles, Brussel (Benelux)",
	"F - Dijon (Bourgogne), Bourgogne (Ostfrankreich / Burgund)",
	"F - Marseille, Provence-Alpes-Côte d'Azur",
	"F - Lyon (Rhone-Alpes), Rhone-Alpes (Rhonetal)",
	"F - Grenoble (Savoie), Rhone-Alpes (Franz. Alpen)",
	"CH - La Chaux de Fond, Jura",
	"D - Frankfurt am Main, Hessen (Unterer Rheingraben)",
	"D - Trier, Westliches Mittelgebirge",
	"D - Duisburg, Nordrhein-Westfalen",
	"GB - Swansea, Wales (Westl. England / Wales)",
	"GB - Manchester, England (Noerdliches England)",
	"F - le Havre, Haute-Normandie (Normandie)",
	"GB - London, England (Suedostengland / London)",
	"D - Bremerhaven, Bremen (Nordseekueste)",
	"DK - Herning, Ringkobing (Nordwestliches Juetland)",
	"DK - Arhus, Arhus (Oestliches Juetland)",
	"D - Hannover, Niedersachsen (Norddeutschland)",
	"DK - Kobenhavn, Staden Kobenhaven (Seeland)",
	"D - Rostock, Mecklenburg-Vorpommern (Ostseekueste)",
	"D - Ingolstadt, Bayern (Donautal)",
	"D - Muenchen, Bayern (Suedbayern)",
	"I - Bolzano, Trentino-Alto Adige (Suedtirol)",
	"D - Nuernberg, Bayern (Nordbayern)",
	"D - Leipzig, Sachsen",
	"D - Erfurt, Thueringen",
	"CH - Lausanne, Genferseeregion (Westl. Schweizer Mitteland)",
	"CH - Zuerich (Oestl. Schweizer Mittelland)",
	"CH - Adelboden (Westl. Schweizer Alpennordhang)",
	"CH - Sion, Wallis",
	"CH - Glarus, Oestlicher Schweizer Alpennordhang",
	"CH - Davos, Graubuenden",
	"D - Kassel, Hessen (Mittelgebirge Ost)",
	"CH - Locarno, Tessin",
	"I - Sestriere, Piemont. Alpen",
	"I - Milano, Lombardia (Poebene)",
	"I - Roma, Lazio (Toskana)",
	"NL - Amsterdam, Noord-Holland (Holland)",
	"I - Genova, Liguria (Golf von Genua)",
	"I - Venezia, Veneto (Pomuendung)",
	"F - Strasbourg, Alsace (Oberer Rheingraben)",
	"A - Klagenfurt, Kaernten (Oesterreich. Alpensuedhang)",
	"A - Innsbruck, Tirol (Inneralpine Gebiete Oesterreichs)",
	"A - Salzburg, Bayr. / Oesterreich. Alpennordhang",
	"SK (Oesterreich / Slovakia) - Wien / Bratislava",
	"CZ - Praha, Prag (Tschechisches Becken)",
	"CZ - Decin, Severocesky (Erzgebirge)",
	"D - Berlin, Ostdeutschland",
	"S - Goeteborg, Goeteborgs och Bohus Laen (Westkueste Schweden)",
	"S - Stockholm, Stockholms Laen (Stockholm)",
	"S - Kalmar, Kalmar Laen (Schwedische Ostseekueste)",
	"S - Joenkoeping, Joenkoepings Laen (Suedschweden)",
	"D - Donaueschingen, Baden-Wuerttemberg (Schwarzwald / Schwaebische Alb)",
	"N - Oslo",
	"D - Stuttgart, Baden-Wuerttemberg (Noerdl. Baden Wuerttemberg)",
	"I - Napoli",
	"I - Ancona",
	"I - Bari",
	"HU - Budapest",
	"E - Madrid",
	"E - Bilbao",
	"I - Palermo",
	"E - Palma de Mallorca",
	"E - Valencia",
	"E - Barcelona",
	"AND - Andorra",
	"E - Sevilla",
	"P - Lissabon",
	"I - Sassari, (Sardinien / Korsika)",
	"E - Gijon",
	"IRL - Galway",
	"IRL - Dublin",
	"GB - Glasgow",
	"N - Stavanger",
	"N - Trondheim",
	"S - Sundsvall",
	"PL - Gdansk",
	"PL - Warszawa",
	"PL - Krakow",
	"S - Umea",
	"S - Oestersund",
	"CH - Samedan",
	"CR - Zagreb",
	"CH - Zermatt",
	"CR - Split",
};

/* NOTE: This mapping is correct! dcf77logs.de use it, esp32-dcf77-weatherman uses it, my TFA clock uses it!
 * Other information are incorrect.
 */
const char *weathers_day[16] = {
	"Reserved",
	"Sunny",
	"Partly clouded",
	"Mostly clouded",
	"Overcast",
	"High fog",
	"Fog",
	"Showers",
	"Light rain",
	"Heavy rain",
	"Frontal storms",
	"Heat storms",
	"Sleet showers",
	"Snow showers",
	"Sleet",
	"Snow",
};

/* Enhanced weather descriptions when combined with "schweres Wetter" (extreme weather day/night)
 * Only codes 1, 6, 7, 9, 10, 12, 13, 14, 15 have special meanings.
 * NULL means no enhanced meaning for that code.
 */
const char *weathers_extreme[16] = {
	NULL,                          /* 0: Reserved */
	"Extreme heat",                /* 1: Sonnig + schweres Wetter = grosse Hitze */
	NULL,                          /* 2: leicht bewölkt */
	NULL,                          /* 3: vorwiegend bewölkt */
	NULL,                          /* 4: bedeckt */
	NULL,                          /* 5: Hochnebel */
	"Dense fog (<50m visibility)", /* 6: Nebel + schweres Wetter = Nebel Sicht unter 50m */
	"Heavy showers",               /* 7: Regenschauer + schweres Wetter = Kurze starke Niederschläge */
	NULL,                          /* 8: leichter Regen */
	"Extreme precipitation",       /* 9: starker Regen + schweres Wetter = extreme Niederschläge */
	"Severe storms",               /* 10: Frontengewitter + schweres Wetter = starke Gewitter */
	NULL,                          /* 11: Wärmegewitter */
	"Heavy sleet showers",         /* 12: Schneeregenschauer + schweres Wetter */
	"Heavy snow showers",          /* 13: Schneeschauer + schweres Wetter */
	"Heavy sleet",                 /* 14: Schneeregen + schweres Wetter */
	"Heavy snowfall",              /* 15: Schneefall + schweres Wetter */
};

const char *weathers_night[16] = {
	"Reserved",
	"Clear",
	"Partly clouded",
	"Mostly clouded",
	"Overcast",
	"High fog",
	"Fog",
	"Showers",
	"Light rain",
	"Heavy rain",
	"Frontal storms",
	"Heat storms",
	"Sleet showers",
	"Snow showers",
	"Sleet",
	"Snow",
};

const char *extremeweathers[16] = {
	"None",
	"Heavy Weather 24 hrs.",
	"Heavy weather Day",
	"Heavy weather Night",
	"Storm 24hrs.",
	"Storm Day",
	"Storm Night",
	"Wind gusts Day",
	"Wind gusts Night",
	"Icy rain morning",
	"Icy rain afternoon",
	"Icy rain night",
	"Fine dust",
	"Ozon",
	"Radiation",
	"High water",
};

const char *probabilities[8] = {
	"0 %",
	"15 %",
	"30 %",
	"45 %",
	"60 %",
	"75 %",
	"90 %",
	"100 %",
};

const char *winddirections[16] = {
	"North",
	"Northeast",
	"East",
	"Southeast",
	"South",
	"Southwest",
	"West",
	"Northwest",
	"Changeable",
	"Foen",
	"Biese N/O",
	"Mistral N",
	"Scirocco S",
	"Tramont W",
	"reserved",
	"reserved",
};

const char *windstrengths[8] = {
	"0",
	"0-2",
	"3-4",
	"5-6",
	"7",
	"8",
	"9",
	">=10",
};

const char *yesno[2] = {
	"No",
	"Yes",
};
const char *anomaly1[4] = {
	"Same Weather",
	"Jump 1",
	"Jump 2",
	"Jump 3",
};
const char *anomaly2[4] = {
	"0-2 hrs",
	"2-4 hrs",
	"5-6 hrs",
	"7-8 hrs",
};

/* show a list of weather data values */
void list_weather(void)
{
	time_t timestamp, t;
	struct tm *tm;
	int i, j;

	/* get time stamp of this day, but at 22:00 UTC */
	timestamp = floor(get_time());
	timestamp -= timestamp % 86400;
	timestamp += 79200;

	printf("\n");
	printf("List of all regions\n");
	printf("-------------------\n");
	for (i = 0; i < 90; i++) {
		printf("Region: %2d = %s\n", i, region_name[i]);
		for (j = 0; j < 8; j++) {
			/* get local time where transmission starts */
			if (i < 60) {
				t = timestamp + 180 * i + 10800 * j;
				tm = localtime(&t);
				printf(" -> Transmission at %02d:%02d of %s\n", tm->tm_hour, tm->tm_min, datasets_0_59[j]);
			} else if (j < 2) {
				t = timestamp + 180 * (i - 60) + 10800 * 7 + 5400 * j;
				tm = localtime(&t);
				printf(" -> Transmission at %02d:%02d of %s\n", tm->tm_hour, tm->tm_min, datasets_60_89[j]);
			}
		}
	}

	printf("\n");
	printf("List of all weathers\n");
	printf("--------------------\n");
	for (i = 0; i < 16; i++) {
		if (i == 1)
			printf("Weather: %2d = %s  (day) %s  (night)\n", i, weathers_day[i], weathers_night[i]);
		else
			printf("Weather: %2d = %s  (day and night)\n", i, weathers_day[i]);
		print_weather_pic(i, i);
	}
	printf("\n");

	printf("List of all extreme weathers\n");
	printf("----------------------------\n");
	for (i = 1; i < 16; i++) {
		printf("Extreme: %2d = %s\n", i, extremeweathers[i]);
	}
	printf("\n");
}

static const char *show_bits(uint64_t value, int bits)
{
	static char bit[128];
	int i;

	for (i = 0; i < bits; i++)
		bit[i] = '0' + ((value >> i) & 1);
	sprintf(bit + i, "(%" PRIu64 ")", value);

	return bit;
}

static void display_weather_temperature(const char *desc, uint32_t weather)
{
	int value;

	value = (weather >> 16) & 0x3f;
	switch (value) {
	case 0:
		printf("%s%s = < -21 degrees C\n", desc, show_bits(value, 6));
		break;
	case 63:
		printf("%s%s = > 40 degrees C\n", desc, show_bits(value, 6));
		break;
	default:
		printf("%s%s = %d degrees C\n", desc, show_bits(value, 6), value - 22);
	}
}

/*
 * TX
 */

/* Adjust given time stamp to the time stamp when the weather of the given
 * region istransmitted. An offset is used to start several minutes before
 * the transmission of the region starts, so the receiver has time to sync
 * to the signal first, so it will not miss that weather info.
 *
 * Note that this will only set the start of transmitting weather of the
 * current day (and day temperature).
 */
time_t dcf77_start_weather(time_t timestamp, int region, int offset)
{
	int hour, min;

	/* hour+min at UTC */
	if (region < 60) {
		/* first dataset starts at 22:00 UTC */
		hour = (22 + region / 20) % 24;
	} else {
		/* seventh dataset starts at 19:00 UTC */
		hour = (19 + (region - 60) / 20) % 24;
	}
	min = (region % 20) * 3;
	LOGP(DDCF77, LOGL_INFO, "Setting UTC time for region %d = %s (to %02d:%02d).\n", region, region_name[region], hour, min);

	/* reset to 0:00 UTC at same day */
	timestamp -= (timestamp % 86400);

	/* add time to start */
	timestamp += hour * 3600 + min * 60;

	/* substract offset */
	LOGP(DDCF77, LOGL_INFO, "Setting timestamp offset to %d minutes.\n", offset);
	timestamp -= 60 * offset;

	return timestamp;
}

/* set weather to transmit on all regions */
void dcf77_set_weather(dcf77_t *dcf77, int weather_day, int weather_night, int extreme, int rain, int wind_dir, int wind_bft, int temperature_day, int temperature_night)
{
	dcf77_tx_t *tx = &dcf77->tx;

	tx->weather = 1;
	tx->weather_day = weather_day;
	tx->weather_night = weather_night;
	tx->rain = rain;
	tx->extreme = extreme;
	tx->wind_dir = wind_dir;
	tx->wind_bft = wind_bft;
	tx->temperature_day = temperature_day;
	tx->temperature_night = temperature_night;
}

/* generate weather frame from weather data */
static uint64_t generate_weather(time_t timestamp, int local_minute, int local_hour, int german_next_minute, int german_utc_hour, int weather_day, int weather_night, int extreme, int rain, int wind_dir, int wind_bft, int temperature_day, int temperature_night)
{
	int dataset = ((german_utc_hour + 2) % 24) * 20 + (german_next_minute / 3); /* data sets since 22:00 UTC */
	struct tm *tm;
	uint64_t key;
	uint32_t weather;
	int value, temperature;
	int i;
	int best, diff;

	/* generate key from time stamp of next minute */
	timestamp += 60;
	tm = localtime(&timestamp);
	key = 0;
	key |= (uint64_t)(tm->tm_min % 10) << 0;
	key |= (uint64_t)(tm->tm_min / 10) << 4;
	key |= (uint64_t)(tm->tm_hour % 10) << 8;
	key |= (uint64_t)(tm->tm_hour / 10) << 12;
	key |= (uint64_t)(tm->tm_mday % 10) << 16;
	key |= (uint64_t)(tm->tm_mday / 10) << 20;
	key |= (uint64_t)((tm->tm_mon + 1) % 10) << 24;
	key |= (uint64_t)((tm->tm_mon + 1) / 10) << 28;
	if (tm->tm_wday > 0)
		key |= (uint64_t)(tm->tm_wday) << 29;
	else
		key |= (uint64_t)(7) << 29;
	key |= (uint64_t)(tm->tm_year % 10) << 32;
	key |= (uint64_t)((tm->tm_year / 10) % 10) << 36;

	/* generate weather data */
	timestamp -= 120;
	weather = 0;
	LOGP(DFRAME, LOGL_INFO, "Encoding weather for dataset %d/480\n", dataset);
	printf("Preparing Weather INFO\n");
	printf("----------------------\n");
	printf("Time (UTC):          %02d:%02d\n", (int)(timestamp / 3600) % 24, (int)(timestamp / 60) % 60);
	printf("Time (LOCAL/DCF77):  %02d:%02d\n", local_hour, local_minute);
	/* dataset and region for 0..59 */
	printf("Dataset:             %s\n", datasets_0_59[dataset / 60]);
	value = dataset % 60;
	printf("Region:              %d = %s\n", value, region_name[value]);
	/* calc. weather of region */
	if (weather_day < 0 || weather_day > 15)
		weather_day = 1;
	weather |= weather_day << 0;
	if (weather_night < 0 || weather_night > 15)
		weather_night = 1;
	weather |= weather_night << 4;
	/* calc. temperature of region 0..59 (day/night) or region 60..89 (day) */
	if (((dataset / 60) & 1) == 0 || (dataset / 60) == 7)
		temperature = temperature_day + 22;
	else
		temperature = temperature_night + 22;
	if (temperature < 0)
		temperature = 0;
	if (temperature > 63)
		value = 63;
	weather |= temperature << 16;
	/* show weather of region 0..59 */
	if ((dataset / 60) < 7) {
		printf("Weather (day):       %s = %s\n", show_bits(weather_day, 4), weathers_day[weather_day]);
		printf("Weather (night):     %s = %s\n", show_bits(weather_night, 4), weathers_night[weather_night]);
		/* Show enhanced meaning if extreme weather (1-3) is set */
		if (extreme >= 1 && extreme <= 3) {
			if (weathers_extreme[weather_day])
				printf("  -> Day extreme weather:   %s\n", weathers_extreme[weather_day]);
			if (weathers_extreme[weather_night])
				printf("  -> Night extreme weather: %s\n", weathers_extreme[weather_night]);
		}
	}
	/* show extreme/wind/rain of region 0..59 */
	if (((dataset / 60) & 1) == 0) {
		/* even datasets, this is 'Day' data */
		if (extreme < 0 || extreme > 15)
			value = 0;
		else
			value = extreme;
		printf("Extreme weather:     %s = %s\n", show_bits(value, 4), extremeweathers[value]);
		weather |= value << 8;
		best = 0;
		for (i = 0; i < 8; i++) {
			diff = abs(atoi(probabilities[i]) - rain);
			if (i == 0 || diff < best) {
				best = diff;
				value = i;
			}
		}
		printf("Rain Probability:    %s = %s (best match for %d)\n", show_bits(value, 3), probabilities[value], rain);
		weather |= value << 12;
		value = 0;
		printf("Anomaly:             %s = %s\n", show_bits(value, 1), yesno[value]);
		weather |= value << 15;
		display_weather_temperature("Temperature (day):   ", weather);
	} else {
		/* odd datasets, this is 'Night' data */
		if (wind_dir < 0 || wind_dir > 15)
			value = 8;
		else
			value = wind_dir;
		printf("Wind direction:      %s = %s\n", show_bits(value, 4), winddirections[value]);
		weather |= value << 8;
		if (wind_bft < 1)
			value = 0;
		else
		if (wind_bft < 7)
			value = (wind_bft + 1) / 2;
		else
		if (wind_bft < 10)
			value = wind_bft - 3;
		else
			value = 7;
		printf("Wind strength:       %s = %s (best match for %d)\n", show_bits(value, 3), windstrengths[value], wind_bft);
		weather |= value << 12;
		value = 0;
		printf("Anomaly:             %s = %s\n", show_bits(value, 1), yesno[value]);
		weather |= value << 15;
		value = temperature_night + 22;
		if (value < 0)
			value = 0;
		if (value > 63)
			value = 63;
		weather |= value << 16;
		if ((dataset / 60) < 7)
			display_weather_temperature("Temperature (night): ", weather);
	}
	/* show picture of weather for region 0..59 */
	if ((dataset / 60) < 7) {
		print_weather_pic(weather_day, weather_night);
	}
	/* show weather and temperature of of region 60..89 */
	if ((dataset / 60) == 7) {
		printf("Dataset:             %s\n", 60 + datasets_60_89[(dataset % 60) / 30]);
		value = 60 + (dataset % 30);
		printf("Region:              %d = %s\n", value, region_name[value]);
		printf("Weather (day):       %s = %s\n", show_bits(weather_day, 4), weathers_day[weather_day]);
		printf("Weather (night):     %s = %s\n", show_bits(weather_night, 4), weathers_night[weather_night]);
		display_weather_temperature("Temperature:         ", weather);
		/* show picture of weather for region 60..89 */
		print_weather_pic(weather_day, weather_night);
	}

	/* the magic '10' bit string */
	weather |= 0x1 << 22;

	/* encode */
	return weather_encode(weather, key);
}

/* transmit chunk of weather data for each minute */
uint16_t tx_weather(dcf77_tx_t *tx, time_t timestamp, int local_minute, int local_hour, int zone)
{
	int index = local_minute % 3;
	int german_utc_hour;
	uint16_t chunk;

	if (index == 0) {
		/* Convert hour to (German) UTC. This is the UTC, if your local time would be German time.
		 * This is required, because German time converted to UTC is used to define what weather information is transmitted. */
		german_utc_hour = local_hour - 1;
		if (zone & 1)
			german_utc_hour--;
		if (german_utc_hour < 0)
			german_utc_hour += 24;
		tx->weather_cipher = generate_weather(timestamp, local_minute, local_hour, local_minute, german_utc_hour, tx->weather_day, tx->weather_night, tx->extreme, tx->rain, tx->wind_dir, tx->wind_bft, tx->temperature_day, tx->temperature_night);
		LOGP(DFRAME, LOGL_INFO, "Transmitting first chunk of weather info.\n");
		chunk = (tx->weather_cipher & 0x3f) << 1; /* bit 2-7 */
		chunk |= (tx->weather_cipher & 0x0fc0) << 2; /* bit 9-14 */
		tx->weather_cipher >>= 12;
		return chunk;
	}

	LOGP(DFRAME, LOGL_INFO, "Transmitting %s chunk of weather info.\n", (index == 1) ? "second" : "third");
	chunk = tx->weather_cipher & 0x3fff;
	tx->weather_cipher >>= 14;
	return chunk;
}

/*
 * RX
 */

/* display weather data from weather frame */
static void display_weather(uint32_t weather, int minute, int utc_hour)
{
	int dataset = ((utc_hour + 2) % 24) * 20 + (minute / 3); /* data sets since 22:00 UTC */
	int value, weather_day = 0, weather_night = 0;

	LOGP(DFRAME, LOGL_INFO, "Decoding weather for dataset %d/480\n", dataset);
	printf("Received Weather INFO\n");
	printf("---------------------\n");
	printf("Time (UTC):          %02d:%02d\n", utc_hour, minute);
	printf("Dataset:             %s\n", datasets_0_59[dataset / 60]);
	value = dataset % 60;
	printf("Region:              %d = %s\n", value, region_name[value]);
	if ((dataset / 60) < 7) {
		weather_day = (weather >> 0) & 0xf;
		printf("Weather (day):       %s = %s\n", show_bits(weather_day, 4), weathers_day[weather_day]);
		weather_night = (weather >> 4) & 0xf;
		printf("Weather (night):     %s = %s\n", show_bits(weather_night, 4), weathers_night[weather_night]);
	}
	if (((dataset / 60) & 1) == 0) {
		/* even datasets, this is 'Day' data */
		int extreme_val = 0;
		if (((weather >> 15) & 1) == 0) {
			extreme_val = (weather >> 8) & 0xf;
			printf("Extreme weather:     %s = %s\n", show_bits(extreme_val, 4), extremeweathers[extreme_val]);
			/* Show enhanced meaning if extreme weather (1-3) is set */
			if (extreme_val >= 1 && extreme_val <= 3) {
				if (weathers_extreme[weather_day])
					printf("  -> Day extreme weather:   %s\n", weathers_extreme[weather_day]);
				if (weathers_extreme[weather_night])
					printf("  -> Night extreme weather: %s\n", weathers_extreme[weather_night]);
			}
		} else {
			value = (weather >> 8) & 0x3;
			printf("Relative weather:    %s = %s\n", show_bits(value, 2), anomaly1[value]);
			value = (weather >> 10) & 0x3;
			printf("Sunshine:            %s = %s\n", show_bits(value, 2), anomaly2[value]);
		}
		value = (weather >> 12) & 0x7;
		printf("Rain Probability:    %s = %s\n", show_bits(value, 3), probabilities[value]);
		value = (weather >> 15) & 0x1;
		printf("Anomaly:             %s = %s\n", show_bits(value, 1), yesno[value]);
		display_weather_temperature("Temperature (day):   ", weather);
	} else {
		/* odd datasets, this is 'Night' data */
		if (((weather >> 15) & 1) == 0) {
			value = (weather >> 8) & 0xf;
			printf("Wind direction:      %s = %s\n", show_bits(value, 4), winddirections[value]);
		} else {
			value = (weather >> 8) & 0x3;
			printf("Relative weather:    %s = %s\n", show_bits(value, 2), anomaly1[value]);
			value = (weather >> 10) & 0x3;
			printf("Sunshine:            %s = %s\n", show_bits(value, 2), anomaly2[value]);
		}
		value = (weather >> 12) & 0x7;
		printf("Wind strength:       %s = %s\n", show_bits(value, 3), windstrengths[value]);
		value = (weather >> 15) & 0x1;
		printf("Anomaly:             %s = %s\n", show_bits(value, 1), yesno[value]);
		if ((dataset / 60) < 7)
			display_weather_temperature("Temperature (night): ", weather);
	}
	if ((dataset / 60) < 7) {
		print_weather_pic(weather_day, weather_night);
	}
	if ((dataset / 60) == 7) {
		printf("Dataset:             %s\n", 60 + datasets_60_89[(dataset % 60) / 30]);
		value = 60 + (dataset % 30);
		printf("Region:              %d = %s\n", value, region_name[value]);
		weather_day = (weather >> 0) & 0xf;
		printf("Weather (day):       %s = %s\n", show_bits(weather_day, 4), weathers_day[weather_day]);
		weather_night = (weather >> 4) & 0xf;
		printf("Weather (night):     %s = %s\n", show_bits(weather_night, 4), weathers_night[weather_night]);
		display_weather_temperature("Temperature:         ", weather);
		print_weather_pic(weather_day, weather_night);
	}
}

/* reset weather frame */
void rx_weather_reset(dcf77_rx_t *rx)
{
	rx->weather_index = 0;
	rx->weather_cipher = 0;
	rx->weather_key = 0;
}

/* receive weather chunk */
void rx_weather(dcf77_rx_t *rx, int minute, int hour, int zone, uint64_t frame)
{
	int index = (minute + 2) % 3;
	int32_t weather;

	if (rx->weather_index == 0 && index != 0) {
		LOGP(DFRAME, LOGL_INFO, "Skipping weather info chunk, waiting for new start of weather info.\n");
		return;
	}

	if (index == 0) {
		rx_weather_reset(rx);
		/* convert hour to UTC */
		rx->weather_utc_hour = hour - 1;
		if (zone & 1)
			rx->weather_utc_hour--;
		if (rx->weather_utc_hour < 0)
			rx->weather_utc_hour += 24;
		rx->weather_cipher |= (frame >> 2) & 0x3f; /* bit 2-7 */
		rx->weather_cipher |= (frame >> 3) & 0x0fc0; /* bit 9-14 */
		rx->weather_index++;
		if (((frame & 0x0002)) || ((frame & 0x0100)) || !rx->weather_cipher) {
			LOGP(DFRAME, LOGL_INFO, "There is no weather info in this received minute.\n");
			rx_weather_reset(rx);
			return;
		}
		LOGP(DFRAME, LOGL_INFO, "Got first chunk of weather info.\n");
		return;
	}
	if (rx->weather_index == 1 && index == 1) {
		LOGP(DFRAME, LOGL_INFO, "Got second chunk of weather info.\n");
		rx->weather_cipher |= (frame << 11) & 0x3fff000; /* bit 1-14 */
		rx->weather_key |= (frame >> 21) & 0x7f;
		rx->weather_key |= ((frame >> 29) & 0x3f) << 8;
		rx->weather_key |= ((frame >> 36) & 0x3f) << 16;
		rx->weather_key |= ((frame >> 45) & 0x1f) << 24;
		rx->weather_key |= ((frame >> 42) & 0x07) << 29;
		rx->weather_key |= ((frame >> 50) & 0xff) << 32;
		rx->weather_index++;
		return;
	}
	if (rx->weather_index == 2 && index == 2) {
		LOGP(DFRAME, LOGL_INFO, "Got third chunk of weather info.\n");
		rx->weather_cipher |= (frame << 25) & 0xfffc000000; /* bit 1-14 */
		weather = weather_decode(rx->weather_cipher, rx->weather_key);
		if (weather < 0)
			LOGP(DFRAME, LOGL_NOTICE, "Failed to decrypt weather info, checksum error.\n");
		else {
			/* in index 2 we transmit minute + 3 (next minute), so we substract 3 */
			display_weather(weather, (minute + 57) % 60, rx->weather_utc_hour);
		}
		rx_weather_reset(rx);
		return;
	}

	rx_weather_reset(rx);
	LOGP(DFRAME, LOGL_INFO, "Got weather info chunk out of order, waiting for new start of weather info.\n");
}

