/* The message FIFO of a transmitter, or a file of its command lines
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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "../liblogging/logging.h"
#include "get_time.h"
#include "msg_fifo.h"

/* The named pipe at fifo_path, or the file of command lines (read as the
 * pipe is, in order; no pipe is made) */
int msg_fifo_open(msg_fifo_t *fifo, const char *fifo_path, const char *file)
{
	memset(fifo, 0, sizeof(*fifo));
	fifo->fd = -1;

	if (file) {
		fifo->file = file;
		fifo->fd = open(file, O_RDONLY | O_NONBLOCK);
		if (fifo->fd < 0) {
			fprintf(stderr, "Failed to open command file '%s'!\n", file);
			return -errno;
		}
		return 0;
	}

	unlink(fifo_path);
	if (mkfifo(fifo_path, 0666) < 0) {
		fprintf(stderr, "Failed to create message send FIFO '%s'!\n", fifo_path);
		return -errno;
	}
	fifo->fifo_path = fifo_path;
	fifo->fd = open(fifo_path, O_RDWR | O_NONBLOCK);
	if (fifo->fd < 0) {
		fprintf(stderr, "Failed to open message send FIFO '%s'!\n", fifo_path);
		return -errno;
	}

	return 0;
}

/* (also on a fifo never opened: static, all zero) */
void msg_fifo_close(msg_fifo_t *fifo)
{
	if (fifo->fd > 0)
		close(fifo->fd);
	fifo->fd = -1;
	if (fifo->fifo_path)
		unlink(fifo->fifo_path);
	fifo->fifo_path = NULL;
}

/* Read what was written; the lines are then taken with msg_fifo_line() */
void msg_fifo_read(msg_fifo_t *fifo)
{
	int rc, i;

	if (fifo->fd < 0)
		return;

	/* the lines taken make room */
	if (fifo->pos > 0) {
		memmove(fifo->buffer, fifo->buffer + fifo->pos, fifo->len - fifo->pos);
		fifo->len -= fifo->pos;
		fifo->pos = 0;
	}

	/* a full buffer without a line end: no line fits, discard it */
	if (fifo->len == (int)sizeof(fifo->buffer)) {
		for (i = 0; i < fifo->len; i++) {
			if (fifo->buffer[i] == '\r' || fifo->buffer[i] == '\n')
				break;
		}
		if (i == fifo->len) {
			LOGP(DSENDER, LOGL_ERROR, "Message FIFO overflow (%d bytes) without a line end, discarding!\n", fifo->len);
			fifo->len = 0;
		}
		return;
	}

	rc = read(fifo->fd, fifo->buffer + fifo->len, sizeof(fifo->buffer) - fifo->len);
	if (rc > 0)
		fifo->len += rc;
	else if (rc == 0 && fifo->file)
		fifo->eof = 1;
}

/* The next complete line (NUL terminated, CR/LF removed, empty lines
 * skipped), NULL when there is none yet */
char *msg_fifo_line(msg_fifo_t *fifo)
{
	int i;

	while (fifo->pos < fifo->len) {
		for (i = fifo->pos; i < fifo->len; i++) {
			if (fifo->buffer[i] == '\r' || fifo->buffer[i] == '\n')
				break;
		}
		if (i == fifo->len)
			return NULL;
		fifo->buffer[i] = '\0';
		if (i > fifo->pos) {
			char *line = fifo->buffer + fifo->pos;
			fifo->pos = i + 1;
			return line;
		}
		fifo->pos = i + 1;
	}

	return NULL;
}

/* Batch with a command file: once every line was taken and the transmitter
 * has nothing left to send (idle), the run ends tail seconds later, so a
 * receiver of the same run can finish. Returns 1 when it is time to end. */
int msg_fifo_batch_end(msg_fifo_t *fifo, int idle, double tail)
{
	if (!fifo->file || !sample_clock_active())
		return 0;
	if (fifo->end_time == 0.0) {
		if (fifo->eof && fifo->pos >= fifo->len && idle) {
			fifo->end_time = get_time() + tail;
			LOGP(DSENDER, LOGL_NOTICE, "Batch: command file done and nothing left to send, ending in %.2f s.\n", tail);
		}
		return 0;
	}

	return get_time() >= fifo->end_time;
}
