/* ERMES subscriber book
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
 * What the network keeps per receiver (RIC), as the home network controller's subscriber
 * database (ETS 300 133-3 Table 63 p.154): for now the message counter. A text file, one receiver
 * per line, "<local address> key=value ...", kept across runs and rewritten when it changes.
 */

#ifndef ERMES_BOOK_H
#define ERMES_BOOK_H

#include <stdint.h>

typedef struct ermes_subscriber {
	uint32_t	local_address;	/* 22 bit */
	int		number;		/* last message number given, 1..31, 0 = none yet */
	int		fsn;		/* frequency subset number 0..15 (§9.2), -1 = not known: the
					 * page goes on the first channel that has room for it */
} ermes_subscriber_t;

/* --book: the file (NULL = kept in memory only). ermes_book_old_numbers: a message number file of
 * an earlier version ("<address hex> <number>" per line), read when the book file does not exist
 * yet, so the numbers go on. */
extern const char *ermes_book_file;
extern const char *ermes_book_old_numbers;

/* The entry of a receiver; with create a new one (all defaults) if it has none. NULL if there is
 * none (or no memory). The book is read from the file at first use. */
ermes_subscriber_t *ermes_book_find(uint32_t local_address, int create);

/* Write the book to its file. 0 or -1 (logged) */
int ermes_book_save(void);

#endif /* ERMES_BOOK_H */
