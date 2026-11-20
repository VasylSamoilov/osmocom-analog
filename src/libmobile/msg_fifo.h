#ifndef _MSG_FIFO_H
#define _MSG_FIFO_H

/* The message FIFO of a transmitter: the command lines written to a named
 * pipe, or read from a file instead (--fifo-file, batch runs on IQ files). */

#define MSG_FIFO_BUFFER	4352

typedef struct msg_fifo {
	const char	*fifo_path;	/* named pipe made (NULL when a file is read) */
	const char	*file;		/* file of command lines (NULL for the pipe) */
	int		fd;
	int		eof;		/* the file is read to its end */
	char		buffer[MSG_FIFO_BUFFER];
	int		len;		/* bytes in the buffer */
	int		pos;		/* start of the next line */
	double		end_time;	/* batch: the time to end the run, 0 before */
} msg_fifo_t;

int msg_fifo_open(msg_fifo_t *fifo, const char *fifo_path, const char *file);
void msg_fifo_close(msg_fifo_t *fifo);
void msg_fifo_read(msg_fifo_t *fifo);
char *msg_fifo_line(msg_fifo_t *fifo);
int msg_fifo_batch_end(msg_fifo_t *fifo, int idle, double tail);

#endif /* _MSG_FIFO_H */
