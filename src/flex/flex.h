#ifndef FLEX_H
#define FLEX_H

#include "../libmobile/sender.h"
#include "../libfilter/iir_filter.h"
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
	FLEX_MSG_TYPE_HEX,
	FLEX_MSG_TYPE_INSTRUCTION,
	FLEX_MSG_TYPE_SHORT,
};

/* Transmitter state machine states */
enum flex_state {
	FLEX_STATE_IDLE = 0,
	FLEX_STATE_ERS,			/* ERS re-sync burst at network start */
	FLEX_STATE_MESSAGE,
	/* Network mode states */
	FLEX_STATE_NET_ERS,		/* Initial ERS at network startup */
	FLEX_STATE_NET_FRAME,		/* Continuous frame transmission */
};

/* Instance of outgoing message (singly-linked list node) */
typedef struct flex_msg {
	struct flex_msg		*next;
	struct flex		*flex;
	uint64_t		capcode;
	enum flex_msg_type	msg_type;
	char			data[4096];		/* increased for long messages */
	int			data_length;

	/* per-message parameters (set from FIFO key=value fields or CLI defaults) */
	int			speed;			/* 1600, 3200, or 6400 (default 1600) */
	int			modulation_type;	/* FLEX_MOD_2FSK or FLEX_MOD_4FSK (default 2FSK) */
	double			polarity;		/* -1.0 or +1.0 (default -1.0 = negative) */
	int			priority;		/* 0 = normal, 1 = priority */
	int			charset;		/* 0 = ASCII, 1 = KANJI */
	int			is_group;		/* 0 = individual, 1 = group */
	int			is_temp_group;		/* 0 = common group, 1 = temporary */
	char			source_id[64];		/* source/callback identifier ('\0' = none) */
	int			short_msg_index;	/* short message index (-1 = N/A) */
	int			phase;			/* phase (channel) override:
						 * -1=auto (default, scheduler assigns
						 * from capcode per ARIB STD-43A),
						 * 0=A, 1=B, 2=C, 3=D.
						 * Only meaningful for multi-phase modes:
						 *   3200/2FSK: 0=A, 1=C
						 *   3200/4FSK: 0=A, 1=C
						 *   6400/4FSK: 0=A, 1=B, 2=C, 3=D */

	/* fragmentation state (set by scheduler for long messages) */
	int			fragment_index;
	int			total_fragments;
	uint32_t		retrieval_num;
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

	/* Sync buffer — S1 + FIW, always transmitted at 1600/2FSK.
	 * Per ARIB STD-43A Section 3.2, the sync portion is always
	 * at 1600 baud regardless of the frame's data speed.
	 * S1 = BS1(4) + Ax(4) + B(2) + Ax_inv(4) = 14 bytes
	 * FIW = 4 bytes (32-bit BCH codeword)
	 * Total: 18 bytes max */
	uint8_t			sync_buffer[20];
	int			sync_buffer_length;
	int			sync_buffer_pos;

	/* Data buffer — S2 + DATA, transmitted at the frame's target speed
	 * after the speed switch from 1600/2FSK (S1+FIW).
	 * S2 = BS2(N) + C(16) + BS2_inv(N) + C_inv(16), N per speed table
	 * DATA = interleaved phase words (352/704/1408 bytes) */
	uint8_t			frame_buffer[FLEX_BUFFER_SIZE];
	int			frame_buffer_length;
	int			frame_buffer_pos;

	/* Target speed/modulation for the data portion (S2+DATA).
	 * S1+FIW always at 1600/2FSK; speed switches after FIW. */
	int			frame_target_speed;
	int			frame_target_mod_type;

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
	int			dbg_frame_symbols;	/* debug: total symbols emitted this frame */
	int			dbg_frame_samples;	/* debug: total samples emitted this frame */
	int			dbg_tx_bitcount;	/* debug: global TX bit counter for bit dump */

	/* network mode config */
	int			network_mode;		/* 0 = one-shot (default), 1 = network */
	int			collapse;		/* 0-7, default 0 */
	int			ers_cycles_override;	/* -1 = auto, >0 = manual override */
	int			no_ers;			/* skip ERS in single-shot mode */
	int			biw_time_enabled;	/* BIW3/BIW4 time broadcast */
	int			lpf_enabled;		/* baseband LPF */

	/* per-message defaults (CLI -M message and FIFO fallback) */
	int			default_speed;		/* 1600, 3200, 6400 — default 1600 */
	double			default_polarity;	/* -1.0 or +1.0 — default -1.0 */
	int			default_charset;	/* 0 = ASCII, 1 = KANJI — default 0 */
	int			default_phase;		/* -1=auto (scheduler), 0=A, 1=B, 2=C, 3=D */

	/* fixed-mode flags (CLI --speed / --polarity lock) */
	int			fixed_speed;		/* -1 = not fixed, else 1600/3200/6400 */
	int			fixed_mod_type;		/* FLEX_MOD_2FSK (default) or FLEX_MOD_4FSK */
	double			fixed_polarity;		/* 0.0 = not fixed, else -1.0/+1.0 */

	/* current frame parameters (set per-frame by scheduler/state machine) */
	int			current_frame_speed;
	int			current_frame_mod_type;	/* FLEX_MOD_2FSK or FLEX_MOD_4FSK */
	double			current_frame_polarity;

	/* roaming config */
	uint32_t		ssid;			/* System Sub-ID */
	uint32_t		nid;			/* Network ID */
	int			roaming_active;		/* FIW n flag */

	/* scheduler state */
	uint32_t		sched_fallback_cycle;
	uint32_t		sched_fallback_frame;
	int			sched_ers_done;		/* network mode: ERS already sent */
	uint32_t		sched_last_cycle;
	uint32_t		sched_last_frame;

	/* ERS streaming state (network mode) */
	int			ers_total_cycles;	/* total ERS cycles to send */
	int			ers_sent_cycles;	/* ERS cycles sent so far */

	/* message numbering */
	uint32_t		msg_sequence;		/* monotonic counter, wraps at max */
	uint32_t		frag_retrieval_seq;	/* monotonic retrieval number for fragments */

	/* 4-FSK state (6400 bps) */
	sample_t		fsk4_ramps[4][4][256];	/* [from][to][phase] */
	uint8_t			fsk4_tx_last_level;	/* 0-3 */

	/* baseband LPF state */
	iir_filter_t		lpf;			/* IIR biquad lowpass */
	double			lpf_coeffs[64];		/* (unused, reserved) */
	double			lpf_history[64];	/* (unused, reserved) */
	int			lpf_taps;		/* (unused, reserved) */
	int			lpf_pos;		/* (unused, reserved) */

	/* wav-test mode: exit after TX completes */
	int			wav_test_mode;

	/* POCSAG mixing */
	int			pocsag_mix_enabled;
	uint8_t			pocsag_frame_slots[128]; /* 1 = POCSAG, 0 = FLEX */

	/* RX state */
	struct {
		int		enabled;
		int		rx_state;		/* RX_HUNT_ERS / RX_SYNC / RX_FRAME */
		int		rx_mode;		/* detected mode: RX_MODE_A1, RX_MODE_A3 */
		uint32_t	shift_reg;		/* bit shift register for sync detection */
		uint32_t	frame_words[FLEX_WORDS_PER_FRAME * 4]; /* decoded 32-bit words (up to 4 phases) */
		int		word_count;		/* words received so far */
		int		bit_count;		/* bits received in current word */
		int		block_count;		/* blocks received */
		double		fsk_rx_phase;		/* sample phase tracking */
		uint8_t		fsk_rx_lastbit;		/* last demodulated bit */

		/* FIW decode */
		uint32_t	fiw_cycle;
		uint32_t	fiw_frame;

		/* message decode state */
		uint32_t	rx_capcode;		/* current address being decoded */
		int		rx_msg_type;		/* vector type */
		int		rx_msg_start;		/* message word start index */
		int		rx_msg_words;		/* message word count */
		char		rx_msg_data[1024];	/* decoded message text */
		int		rx_msg_data_length;
	} rx;
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
