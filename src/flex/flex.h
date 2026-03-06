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
	int			blocking_length;	/* HEX/Binary B field: bits per character.
						 * 1-15 = that many bits, 0 = 16 bits.
						 * Spec §3.10.1.2, default 1 (raw bits). */
	int			mail_drop;		/* M flag (Spec §3.10.1.3):
						 * 0 = ordinary message (default)
						 * 1 = can be handled separately from
						 *     ordinary messages (mail drop) */
	int			phase;			/* phase (channel) override:
						 * -1=auto (default, scheduler assigns
						 * from capcode per ARIB STD-43A),
						 * 0=A, 1=B, 2=C, 3=D.
						 * Only meaningful for multi-phase modes:
						 *   3200/2FSK: 0=A, 1=C
						 *   3200/4FSK: 0=A, 1=C
						 *   6400/4FSK: 0=A, 1=B, 2=C, 3=D */

	/* fragmentation state (set by scheduler for long messages).
	 * All fragments of one message share the same retrieval_num,
	 * which becomes the N (message number) field in the alpha/hex
	 * header word.  Per spec §3.10.1.3: "those message numbers
	 * which are newly assigned to a message must be unique numbers
	 * so as to identify fragments for the same message." */
	int			fragment_index;
	int			total_fragments;
	uint32_t		retrieval_num;	/* → N field (6 bits, 0-63) */
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
	 * S2 = BS2 + C(16 bits) + inv.BS2 + inv.C(16 bits)
	 * DATA = interleaved phase words (352/704/1408 bytes)
	 * Buffer is at the BIT level; the DSP layer converts bits to
	 * symbols (1:1 for 2FSK, 2 bits per symbol for 4FSK). */
	uint8_t			frame_buffer[FLEX_BUFFER_SIZE];
	int			frame_buffer_length;
	int			frame_buffer_pos;

	/* Target speed/modulation for the data portion (S2+DATA).
	 * S1+FIW always at 1600/2FSK; speed switches after FIW.
	 * frame_target_speed is the BIT rate (bps): 1600, 3200, or 6400.
	 * The DSP layer derives the symbol rate (baud) from this. */
	int			frame_target_speed;
	int			frame_target_mod_type;

	/* DSP state — field names match pocsag_t for shared fsk_block_encode().
	 * NOTE: fsk_bitduration/fsk_bitstep are SAMPLES PER SYMBOL, not per bit.
	 * Named "bit" for POCSAG compatibility where bit=symbol (2FSK only).
	 * For 4FSK modes, these are per-symbol (1 symbol = 2 bits = 1 dibit). */
	double			fsk_bitduration;	/* samples per symbol */
	double			fsk_bitstep;		/* 1.0 / fsk_bitduration */
	sample_t		fsk_ramp_up[256];
	sample_t		fsk_ramp_down[256];
	sample_t		*fsk_tx_buffer;
	int			fsk_tx_buffer_size;
	int			fsk_tx_buffer_length;
	int			fsk_tx_buffer_pos;
	double			fsk_tx_phase;
	uint8_t			fsk_tx_lastbit;
	int	dbg_frame_symbols;	/* debug: total symbols emitted this frame */
	int	dbg_frame_samples;	/* debug: total samples emitted this frame */

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
	int			default_blocking_length; /* HEX/Binary B field default: 1-15 bits/char,
						  * 0=16. Spec §3.10.1.2, default 1 (raw bits). */

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

	/* ERS streaming state (network mode and FIFO-triggered) */
	int			ers_total_cycles;	/* total ERS cycles to send */
	int			ers_sent_cycles;	/* ERS cycles sent so far */

	/* Dual-polarity ERS is unnecessary.
	 *
	 * The ERS cycle structure is BS + Ar + BS_inv + Ar_inv.
	 * Inverting all bits (= flipping polarity) produces
	 * BS_inv + Ar_inv + BS + Ar — the same continuous stream
	 * shifted by half a cycle (48 bits).  Both Ar and Ar_inv
	 * appear in every cycle, so a pager of either polarity will
	 * detect the re-sync pattern regardless of TX polarity.
	 *
	 * This is by design: Section 3.2 uses the A + inv.A structure
	 * precisely so that polarity is irrelevant for sync detection. */

	/* message numbering */
	uint32_t		msg_sequence;		/* monotonic counter, wraps at 64 (N is 6 bits) */
	uint32_t		frag_retrieval_seq;	/* monotonic retrieval number for fragments (6-bit, 0-63) */

	/* 4-FSK state (3200bps/4FSK and 6400bps/4FSK modes) */
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

	/* BCH(31,21) decode statistics — accumulated per frame across
	 * all BCH-protected fields (S1 A-code, FIW, data words).
	 * Reset at each new S1 sync detection. */
	struct {
		unsigned int	clean;		/* syndrome=0, no correction needed */
		unsigned int	corrected;	/* 1-2 bit errors, corrected by BCH */
		unsigned int	uncorrectable;	/* >2 bit errors, BCH failed */
		unsigned int	total;		/* total codewords processed */
	} bch_stats;

	/* RX state — standard-compliant FLEX decoder (ARIB STD-43A) */
	struct {
		int		enabled;

		/* PLL-based symbol timing recovery (per multimon-ng buildSymbol) */
		int		sample_freq;		/* audio sample rate (e.g. 48000) */
		int64_t		pll_phase;		/* phase accumulator */
		int		pll_locked;		/* 1 = locked to signal */
		double		pll_zero;		/* DC offset estimate */
		double		pll_envelope;		/* signal envelope estimate */
		double		pll_envelope_sum;	/* running sum for envelope */
		int		pll_envelope_count;	/* sample count for envelope */
		double		pll_last_sample;	/* previous sample for zero-crossing */
		int		pll_symcount[4];	/* symbol vote counters per period */
		int		pll_nonconsec;		/* non-consecutive zero crossings */
		int		pll_timeout;		/* timeout counter */
		uint64_t	pll_sample_count;	/* total samples processed */
		uint64_t	pll_symbol_count;	/* total symbols produced */
		uint64_t	pll_lock_buf;		/* lock pattern shift register */

		/* Demodulator state */
		int		baud;			/* current symbol rate in baud (not bps) */
		int		polarity;		/* 0 = normal, 1 = inverted */

		/* State machine */
		int		rx_state;		/* RX_STATE_SYNC1..DATA */

		/* Sync detection — 128-bit shift register for full S1.
		 * S1 = BS1(32) + A(32) + B(16) + inv.A(32) = 112 bits.
		 * sync_buf_hi:sync_buf_lo form a single 128-bit register,
		 * shifted left 1 bit per symbol during SYNC1 and the first
		 * 16 symbols of FIW (to capture the full inv.A tail).
		 * Sync fires at bit 96 (after BS1+A+B+inv.A_upper16) when
		 * the marker appears in sync_buf_lo.  After 16 more shifts,
		 * the full inv.A is available for combined error correction. */
		uint64_t	sync_buf_hi;		/* upper 64 bits of 128-bit shift register */
		uint64_t	sync_buf_lo;		/* lower 64 bits (marker detection here) */
		uint32_t	sync_a_code;		/* A code extracted at sync detection */
		int		sync_baud;		/* symbol rate from sync (baud, not bps) */
		int		sync_levels;		/* 2=2FSK (1 bit/sym), 4=4FSK (2 bits/sym) */
		int		reflex;			/* 1 = ReFLEX sync detected (stub) */

		/* S1 tail state — 16 symbols after sync detection to
		 * complete inv.A reception.  Register keeps shifting. */
		int		s1_tail_count;		/* symbol counter (0..16) */

		/* FIW state (always 1600/2FSK, 1 bit per symbol) */
		int		fiw_count;		/* bit counter in FIW state (0..32) */
		uint32_t	fiw_rawdata;		/* accumulated FIW bits (32-bit codeword) */

		/* FIW decode result */
		uint32_t	fiw_cycle;
		uint32_t	fiw_frame;

		/* S2 state — evidence-based timing correction.
		 * We scan the full S2 window + 2 symbols, recording where
		 * C and inv.C are found.  At the end, we combine both
		 * positions to decide whether to correct the data boundary.
		 * Correction is applied only when evidence is strong and
		 * the offset is within ±2 symbols of nominal. */
		int		sync2_count;		/* S2 symbol counter */
		uint16_t	sync2_shiftreg;		/* 16-bit shift register for C pattern */
		int		sync2_c_found;		/* 1 = C (first half) detected */
		int		sync2_c_pos;		/* symbol position where C was found */
		int		sync2_c_errs;		/* bit errors in C match */
		int		sync2_cinv_found;	/* 1 = inv.C (second half) detected */
		int		sync2_cinv_pos;		/* symbol position where inv.C was found */
		int		sync2_cinv_errs;	/* bit errors in inv.C match */
		unsigned char	sync2_sym_buf[4];	/* symbols from [nominal-2 .. nominal+2) */
		int		sync2_sym_buf_count;	/* symbols buffered */
		int		sync2_sym_buf_start;	/* symbol index of first buffered sym */

		/* Data state */
		int		data_count;		/* data symbol counter */

		/* Per-phase channel data (Spec Section 3.3.3).
		 * Each phase is an independent channel with 88 words
		 * and per-word BCH status. */
		flex_phase_data_t phase[FLEX_MAX_PHASES];
		int		phase_toggle;		/* alternates 0/1 at 3200 baud */
		int		data_bit_counter;	/* de-interleave index counter */

		/* Per-phase temporary group tracking (Spec Section 5.2).
		 *
		 * Temporary addresses (0x1F7800–0x1F780F) are 16 group slots.
		 * The system assigns pagers to a temp group in three phases:
		 *
		 * 1) SETUP: Each pager's individual address + short instruction
		 *    vector is transmitted.  The instruction tells the pager:
		 *    "listen on temp slot a3-a0 in frame f6-f0."
		 *    Multiple pagers receiving the same slot+frame = same group.
		 *    Standard: "the address and Short Instruction Vector for
		 *    each pager are transmitted first."
		 *
		 * 2) DELIVERY: In the designated frame, the temp address slot
		 *    appears with a vector + message.  All assigned pagers
		 *    receive it.
		 *    Standard: "Then all the pagers within the same group
		 *    receive the Temporary Address with vector and message
		 *    in the Frame designated by the Short Instruction Vector."
		 *    Standard: "The Temporary Address is valid until the Group
		 *    message ends. It is also possible to break down Group
		 *    messages which stretch over several Frames."
		 *    Standard: "All dynamic group messages using temporary
		 *    addresses are considered to be fragmented regardless of
		 *    message length."
		 *
		 * 3) TEARDOWN: Automatic — no explicit remove.
		 *    Standard: "When the complete message is built, the pager
		 *    automatically returns to normal operation."
		 *    Standard: "In cases when the Temporary Address is not
		 *    detected within the designated Frame, the pager must
		 *    automatically return to normal operation immediately."
		 *    Standard: "The first transmission of the Group message
		 *    must be started within 128 Frames from the first
		 *    transmission of the Short Instruction Vector."
		 *
		 * Assignments persist across frames until:
		 *   a) The group message completes (C=0 in alpha header, or
		 *      non-fragmented message type like tone/numeric), OR
		 *   b) 128 frames elapse without delivery (timeout per §5.2).
		 *
		 * Each phase has its own independent set of 16 slots.
		 * Each slot can have up to FLEX_TEMP_GROUP_MAX_MEMBERS
		 * capcodes assigned. */
		struct {
			uint64_t	capcodes[FLEX_TEMP_GROUP_MAX_MEMBERS];
			uint32_t	target_frame;	/* f6-f0 from instruction */
			uint32_t	setup_frame;	/* frame number where SETUP was observed */
			uint32_t	setup_cycle;	/* cycle number where SETUP was observed */
			int		count;		/* number of capcodes assigned */
			int		active;		/* 1 = assignment active, 0 = empty */
		} temp_addr_map[FLEX_MAX_PHASES][FLEX_TEMP_ADDR_SLOTS];

		/* BIW state — track SSID/coverage/timezone across frames.
		 * Log at NOTICE level when values change, DEBUG when repeated. */
		struct {
			uint32_t	local_id;
			uint32_t	coverage;
			uint32_t	country;
			uint32_t	tmf;
			int		seen;		/* 1 = at least one SSID1 received */
			int		seen_ssid2;	/* 1 = at least one SSID2 received */
		} biw;

		/* Fragment reassembly state (Spec Section 3.10.1.3 / 4.2).
		 *
		 * Tracks in-progress fragmented messages per capcode.
		 * A fragment stream is identified by capcode + message number (N).
		 * Fragments arrive with C=1 (continued) until the final C=0.
		 * F cycles through modulo-3 sequence: 11,00,01,10,00,01,10,...
		 *
		 * Per spec: "The transmission interval between each fragment
		 * must be 32 Frames (equivalent to 1 minute) or less."
		 * We use a 64-frame timeout (generous) to expire stale entries. */
#define FLEX_REASM_SLOTS	16	/* max concurrent reassembly streams */
#define FLEX_REASM_MAX_LEN	4096	/* max reassembled message length */
#define FLEX_REASM_TIMEOUT	64	/* frames before expiry */
		struct {
			uint64_t	capcode;
			int		msg_num;	/* N field (0-63) */
			int		expected_f;	/* next expected F value */
			int		msg_type;	/* FLEX_VECTOR_TYPE_ALPHA or _HEX_BINARY */
			int		blocking;	/* HEX/Binary B field from initial frag */
			char		buf[FLEX_REASM_MAX_LEN];
			int		len;		/* bytes accumulated */
			uint32_t	last_frame;	/* frame number of last fragment */
			uint32_t	last_cycle;	/* cycle number of last fragment */
			int		active;		/* 1 = in progress */
		} reasm[FLEX_REASM_SLOTS];
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

/* ERS (Emergency Re-Synchronization) trigger.
 * Polarity is irrelevant for ERS — the BS+Ar+BS_inv+Ar_inv cycle
 * structure ensures pagers of either polarity detect the re-sync. */
void flex_trigger_ers(flex_t *flex);

/* Frame buffer management */
int flex_get_next_frame(flex_t *flex);

/* Scan and loopback support */
int flex_scan_or_loopback(flex_t *flex);

/* Utility */
const char *flex_msg_type_name(enum flex_msg_type type);
const char *flex_number_valid(const char *number);

#endif /* FLEX_H */
