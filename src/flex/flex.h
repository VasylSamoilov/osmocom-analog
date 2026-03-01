#ifndef FLEX_H
#define FLEX_H

#include "../libmobile/sender.h"
#include "frame.h"

/* Forward declarations */
struct flex;
struct flex_msg;

/* Message types */
enum flex_msg_type {
	FLEX_MSG_TYPE_AUTO = 0,
	FLEX_MSG_TYPE_TONE,
	FLEX_MSG_TYPE_NUMERIC,
	FLEX_MSG_TYPE_ALPHA,
};

/* Transmitter state machine states */
enum flex_state {
	FLEX_STATE_IDLE = 0,
	FLEX_STATE_PREAMBLE,
	FLEX_STATE_MESSAGE,
};

/* Instance of outgoing message (singly-linked list node) */
typedef struct flex_msg {
	struct flex_msg		*next;
	struct flex		*flex;
	uint64_t		capcode;
	enum flex_msg_type	msg_type;
	char			data[256];
	int			data_length;
} flex_msg_t;

/* Instance of FLEX transmitter — embeds sender_t as first member */
typedef struct flex {
	sender_t		sender;

	/* config */
	int			tx;
	const char		*default_message;
	uint64_t		default_capcode;
	enum flex_msg_type	default_msg_type;
	double			fsk_deviation;
	double			fsk_polarity;

	/* state machine */
	enum flex_state		state;
	int			idle_count;

	/* scan/loopback */
	uint64_t		scan_from, scan_to;

	/* message queue (singly-linked list) */
	flex_msg_t		*msg_list;

	/* frame buffer — filled by frame.c, consumed by dsp.c
	 * FLEX pre-encodes the entire frame because of block interleaving,
	 * unlike POCSAG which generates codewords on-the-fly. */
	uint8_t			frame_buffer[FLEX_BUFFER_SIZE];
	int			frame_buffer_length;
	int			frame_buffer_pos;

	/* DSP state — identical field pattern to pocsag_t,
	 * reused by identical fsk_block_encode() and dsp_init_ramp() code */
	double			fsk_bitduration;
	double			fsk_bitstep;
	sample_t		fsk_ramp_up[256];
	sample_t		fsk_ramp_down[256];
	sample_t		*fsk_tx_buffer;
	int			fsk_tx_buffer_size;
	int			fsk_tx_buffer_length;
	int			fsk_tx_buffer_pos;
	double			fsk_tx_phase;
	uint8_t			fsk_tx_lastbit;
} flex_t;

/* Instance lifecycle */
int flex_create(const char *kanal, double frequency, const char *device, int use_sdr,
		int samplerate, double rx_gain, double tx_gain, int tx,
		double deviation, double polarity, enum flex_msg_type msg_type,
		const char *message, uint64_t scan_from, uint64_t scan_to,
		const char *write_rx_wave, const char *write_tx_wave,
		const char *read_rx_wave, const char *read_tx_wave, int loopback);
void flex_destroy(sender_t *sender);

/* Message queue management */
flex_msg_t *flex_msg_create(flex_t *flex, uint64_t capcode,
			    enum flex_msg_type msg_type,
			    const char *data, int data_length);
void flex_msg_destroy(flex_msg_t *msg);

/* Frame buffer management */
int flex_get_next_frame(flex_t *flex);

/* Scan and loopback support */
int flex_scan_or_loopback(flex_t *flex);

/* Utility */
const char *flex_msg_type_name(enum flex_msg_type type);
const char *flex_number_valid(const char *number);

#endif /* FLEX_H */
