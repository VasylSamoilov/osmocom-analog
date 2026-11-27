/* ERMES message numbers
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
 *
 * The pager checks the sequence and reports missing messages, so the last number of every
 * address is kept in a file across runs ("<address hex> <number>" per line).
 */

#include <stdio.h>
#include "../liblogging/logging.h"
#include "numbers.h"

const char *ermes_number_file = NULL;

#define MAX_NUMBERS	1024
#define LAST_NUMBER	31

int ermes_number_next(uint32_t local_address)
{
	static uint32_t addr[MAX_NUMBERS];
	static int last[MAX_NUMBERS];
	static int count = -1;
	FILE *fp;
	int i;

	if (count < 0) {
		count = 0;
		if (ermes_number_file && (fp = fopen(ermes_number_file, "r"))) {
			unsigned int a;
			int n;

			while (count < MAX_NUMBERS && fscanf(fp, "%x %d", &a, &n) == 2) {
				addr[count] = a;
				last[count++] = n;
			}
			fclose(fp);
		}
	}
	for (i = 0; i < count; i++) {
		if (addr[i] == local_address)
			break;
	}
	if (i == count) {
		if (count == MAX_NUMBERS)
			return 1;
		addr[count] = local_address;
		last[count++] = 0;
	}
	last[i] = (last[i] % LAST_NUMBER) + 1;

	if (ermes_number_file && (fp = fopen(ermes_number_file, "w"))) {
		int j;

		for (j = 0; j < count; j++)
			fprintf(fp, "0x%06X %d\n", addr[j], last[j]);
		fclose(fp);
	} else if (ermes_number_file)
		LOGP(DERMES, LOGL_ERROR, "Cannot write message numbers to '%s'.\n", ermes_number_file);
	return last[i];
}
