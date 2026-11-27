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

#ifndef ERMES_SSI_H
#define ERMES_SSI_H

#include <time.h>
#include "frame.h"

/* --ssi: the SSI schedule. HOLD is the network schedule (default); the others are for tests of
 * receivers only, as is one SSIT (ERMES_SSIT_*) in every batch */
#define ERMES_SSI_HOLD		-5	/* network: 0000 / 0001 by turns per subsequence from the
					 * hour start (--ssi-hold) */
#define ERMES_SSI_ALTERNATE	-1	/* test: 0000 / 0001 by turns every batch */
#define ERMES_SSI_CYCLE		-2	/* test: the 13 types of edition 2, one a batch */
#define ERMES_SSI_STRICT	-3	/* test: 0000 0001 by turns every batch, from a batch counter */
#define ERMES_SSI_PAIRS		-4	/* test: 0000 0000 0001 0001 ... */
extern int ermes_ssi_hold;
extern int ermes_ssi_mode;

/* --year-from: SSI year moved into [year, year+27] by 28-year steps (0 = off) */
extern int ermes_year_from;

/* The SSI of a batch at network time now (local time), by --ssi. counter is the running batch
 * counter of the test schedules (strict, pairs, cycle), kept by the caller, 0 at start. */
void ermes_ssi_select(const ermes_network_t *net, const ermes_batch_time_t *t, time_t now, int *counter,
		      ermes_ssi_t *ssi);

#endif /* ERMES_SSI_H */
