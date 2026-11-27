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
 */

#ifndef ERMES_NUMBERS_H
#define ERMES_NUMBERS_H

#include <stdint.h>

/* The counters are kept in the subscriber book (book.h). The next message number of a receiver:
 * one counter per receiver - local address and home operator (opid, -1 = ours; a visitor with the
 * same local address is another receiver) - 1..31 from 1, +1 per message (ETS 300 133-2 §4.3.1.3
 * p.18, -4 Table 8 p.19, §10.2 p.31). Takes the number. */
int ermes_number_next(uint32_t local_address, int opid);

/* The number ermes_number_next() gives after ahead more numbers of the receiver are taken,
 * without taking it (ahead 0 = the next one) */
int ermes_number_peek(uint32_t local_address, int opid, int ahead);

#endif /* ERMES_NUMBERS_H */
