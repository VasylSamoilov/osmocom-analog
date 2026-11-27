/* ERMES supplementary system information (SSI) schedule
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
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

#include "ssi.h"

/* --ssi: the SSI schedule, ERMES_SSI_HOLD for a network (default), the others for tests (ssi.h) */
int ermes_ssi_mode = ERMES_SSI_HOLD;
/* --ssi-hold: subsequences each type is held with --ssi hold (1 as the real networks; 5 = one
 * minute for tests) */
int ermes_ssi_hold = 1;

/* --year-from: the year sent in the SSI (type 0001) moved into [ermes_year_from, +27] by whole
 * 28-year steps, so that pagers whose firmware calendar ends early (NEC: the Skyper counter
 * overflows after 2026-11, the PA4PY-20A ends 2013) get a year they know. Weekdays and leap years
 * repeat every 28 years from 1901 to 2099, so weekday, month and day stay right. 0 = off. */
int ermes_year_from = 0;
#define CALENDAR_CYCLE	28

static int ermes_year(int year)
{
	if (!ermes_year_from)
		return year;
	while (year >= ermes_year_from + CALENDAR_CYCLE)
		year -= CALENDAR_CYCLE;
	while (year < ermes_year_from)
		year += CALENDAR_CYCLE;
	return year;
}

/* SSI of this batch, local time (§5.3.2.2-3 p.17), by --ssi:
 *   hold (default)  the network's schedule: the hour starts with 0000 (zone/hour/date) in every
 *                   batch of its first subsequence, 0001 (day/month/year) in the next, and so on
 *                   by turns (--ssi-hold subsequences each, default 1), as the Swiss (1997) and
 *                   Dutch (2014) networks sent. The zone reaches every pager at least once per
 *                   hour (ed.1 §6.3.1.1 p.15), every 24 s here.
 * For tests only (receiver behaviour, not network schedules):
 *   alternate       0000 / 0001 from batch to batch, the subsequence count added (a subsequence
 *                   has an even number of batches), so an own batch gets both types by turns
 *   strict, pairs   0000 0001 ... and 0000 0000 0001 0001 ... from a batch counter
 *   cycle           the 13 allowed types, one step a batch (edition 2 §5.3.2.1 p.16): edition 1
 *                   pagers cannot use the types they do not know
 *   one type        zone-hour-date or day-month-year in every batch
 * NEC PA3E6-1A on air (2026-11-26/27): it sets its clock from SSI only when it acquires the network
 * after power on, from one type: 0000 alone gave the hour (weekday stays its default Mo), 0001
 * alone the weekday (hour 0); every mixed schedule gave one of them, the 13-type cycle neither; no
 * update later (hold for a minute each, a date change at midnight). No ERMES document says when a
 * receiver takes the time (docs/SSI_TIME_RULES.md): an NEC firmware choice. The minute it takes
 * from the cycle number, also only then: afterwards its own clock runs on (a pager set by a
 * --clock 15:58 network still showed 16:06 while it received our real-time network). */
void ermes_ssi_select(const ermes_network_t *net, const ermes_batch_time_t *t, time_t now, int *counter,
		      ermes_ssi_t *ssi)
{
	struct tm tm;
	int ssit = ermes_ssi_mode;

	localtime_r(&now, &tm);
	if (ssit == ERMES_SSI_ALTERNATE)
		ssit = ((t->slot + t->cycle * ERMES_SUBSEQUENCES + t->ssn) & 1) ? ERMES_SSIT_DAY_MONTH_YEAR : ERMES_SSIT_ZONE_HOUR_DATE;
	if (ssit == ERMES_SSI_HOLD) {
		/* one type in every batch of ermes_ssi_hold subsequences, then the other, counted from
		 * the start of the hour: n = 5 * cycle + subsequence, so the hour starts with 0000 in its
		 * first subsequence and the types go by turns from there. An hour has 300 subsequences
		 * (even), so the turns go on across hours and days, and a restart, which takes its
		 * position from UTC, keeps them. With 1 (default) this is what two real networks sent:
		 * Switzerland 1997-07-03 13:38 (cycle 38: 0000 0001 0000 0001 in subsequences 0..3) and
		 * the Netherlands 2014-07-20 16:13 (cycle 13: 0001 in subsequence 2 = n 67, 0000 in 3) */
		int count = t->cycle * ERMES_SUBSEQUENCES + t->ssn;

		ssit = ((count / ermes_ssi_hold) & 1) ? ERMES_SSIT_DAY_MONTH_YEAR : ERMES_SSIT_ZONE_HOUR_DATE;
	}
	if (ssit == ERMES_SSI_STRICT || ssit == ERMES_SSI_PAIRS) {
		/* from a running batch counter: strict 0000 0001 0000 ..., pairs 0000 0000 0001 0001 ... */
		int k = (ssit == ERMES_SSI_STRICT) ? *counter : *counter >> 1;

		ssit = (k & 1) ? ERMES_SSIT_DAY_MONTH_YEAR : ERMES_SSIT_ZONE_HOUR_DATE;
		(*counter)++;
	}
	if (ssit == ERMES_SSI_CYCLE) {
		/* ed.2 §5.3.2.1 p.16: every batch the next of the 13 allowed values, modulo 13 */
		ssit = *counter;
		*counter = (*counter + 1) % ERMES_SSIT_ALLOWED;
	}
	ssi->ssit = ssit;
	switch (ssit) {
	case ERMES_SSIT_ZONE_HOUR_DATE:
		ssi->ssif = ermes_ssif_zone_hour_date(net->zone, tm.tm_hour, tm.tm_mday);
		break;
	case ERMES_SSIT_DAY_MONTH_YEAR:
		/* struct tm: Sunday = 0, months from 0, years from 1900; ERMES: Monday = 1 .. Sunday = 7 */
		ssi->ssif = ermes_ssif_day_month_year(tm.tm_wday ? tm.tm_wday : 7, tm.tm_mon + 1, ermes_year(tm.tm_year + 1900));
		break;
	case ERMES_SSIT_IVR1:
		/* a network that cycles the SSIT is an I1 version >= 02.01 one (§4.6.1 p.14) */
		ssi->ssif = ermes_ssif_version(ERMES_IVR1_MAIN, ERMES_IVR1_SUB);
		break;
	default:
		/* reserved for future definition: no value is defined, all zero */
		ssi->ssif = 0;
	}
}
