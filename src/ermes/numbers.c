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
 * address is kept in the subscriber book across runs.
 */

#include "numbers.h"
#include "book.h"

#define LAST_NUMBER	31

/* the last number given to the address, 0 if none */
static int last_number(uint32_t local_address)
{
	ermes_subscriber_t *s = ermes_book_find(local_address, 0);

	return s ? s->number : 0;
}

int ermes_number_peek(uint32_t local_address, int ahead)
{
	return ((last_number(local_address) + ahead) % LAST_NUMBER) + 1;
}

int ermes_number_next(uint32_t local_address)
{
	ermes_subscriber_t *s = ermes_book_find(local_address, 1);

	if (!s)
		return 1;
	s->number = (s->number % LAST_NUMBER) + 1;
	ermes_book_save();
	return s->number;
}
