#ifndef FLEX_H
#define FLEX_H

#include "../libmobile/sender.h"
#include "../libfilter/iir_filter.h"
#include "frame.h"

/* Number of independent polarity networks (NORMAL=0, INVERTED=1).
 * Each polarity is a separate logical network with independent
 * temp group tracking, reassembly, message history, and BIW state. */
#define FLEX_RX_POLARITIES	2
#define FLEX_POL_NORMAL		0
#define FLEX_POL_INVERTED	1
#define FLEX_RX_MAX_SUBFRAMES	4	/* max num_transmissions */

/* Per-subframe storage for one copy of a repeated transmission.
 * Stores only the subframe-local words (sf_words), not the full 88. */
typedef struct flex_rx_subframe {
	uint32_t		words[FLEX_WORDS_PER_FRAME];	/* sf_words decoded words (max 44 for 2x) */
	enum flex_word_status	status[FLEX_WORDS_PER_FRAME];	/* sf_words BCH statuses */
	int			word_count;			/* actual subframe word count */
	int			received;			/* 1 = this copy has data */
	int			copy_index;			/* which transmission (0=1st, 1=2nd, ...) */
	int			clean;				/* 1 = all words CLEAN or CORRECTED */
	int			uncorrectable_count;		/* number of UNCORRECTABLE words */
} flex_rx_subframe_t;

/* Subframe store for one phase's worth of repeated transmissions.
 * Accumulates N copies of the same subframe content across frames,
 * combines best-of per word, and releases when complete or timed out. */
typedef struct flex_rx_subframe_store {
	flex_rx_subframe_t	subframes[FLEX_RX_MAX_SUBFRAMES];
	int			num_expected;		/* fiw_num_transmissions (2/3/4) */
	int			num_received;		/* count of received copies */
	int			sf_words;		/* words per subframe (44/29/22) */
	int			decoded;		/* 1 = already decoded (early release) */
	uint32_t		base_frame;		/* frame number of first subframe */
	uint32_t		base_cycle;		/* cycle of first subframe */
	uint32_t		timeout_frame;		/* absolute frame for timeout */
	int			active;			/* 1 = accumulating subframes */
} flex_rx_subframe_store_t;

/* Message history for RX retransmission detection (Req 17-18) */
#define FLEX_RX_MSG_HISTORY_MAX		256
#define FLEX_RX_MSG_HISTORY_WINDOW	1920	/* frames (1 hour) */
#define FLEX_RX_MSG_MAX_WORDS		88	/* max words per message */

typedef struct flex_rx_msg_entry {
	/* Primary key fields */
	uint64_t		capcode;
	int			msg_num;		/* N (0-63) */
	int			vec_type;		/* FLEX_VECTOR_TYPE_* */
	int			rx_phase;		/* 0=A, 1=B, 2=C, 3=D */

	/* Validation fields (must match for retransmission confirmation) */
	int			word_count;		/* message word count (mw2 - mw1 + 1) */
	int			frag_count;		/* total fragments (from C/F flags) */
	int			is_long;		/* short vs long address format */
	int			blocking;		/* blocking value (hex messages) */

	/* Per-word data for cross-retransmission recovery */
	uint32_t		words[FLEX_RX_MSG_MAX_WORDS];
	enum flex_word_status	word_status[FLEX_RX_MSG_MAX_WORDS];
	int			has_uncorrectable;	/* 1 = at least one UNCORRECTABLE word */

	/* Housekeeping */
	uint32_t		frame_abs;		/* absolute frame when stored */
	int			active;			/* 1 = entry in use */
} flex_rx_msg_entry_t;

/* Forward declarations */
struct flex;
struct flex_msg;

/* Message types.
 * Values MUST match the FLEX_FRAME_MSG_TYPE_* defines in frame.h —
 * those are the single source of truth for the integer constants.
 * frame.h uses #defines because it cannot include flex.h (circular). */
enum flex_msg_type {
	FLEX_MSG_TYPE_AUTO		= FLEX_FRAME_MSG_TYPE_AUTO,
	FLEX_MSG_TYPE_TONE		= FLEX_FRAME_MSG_TYPE_TONE,
	FLEX_MSG_TYPE_NUMERIC		= FLEX_FRAME_MSG_TYPE_NUMERIC,
	FLEX_MSG_TYPE_ALPHA		= FLEX_FRAME_MSG_TYPE_ALPHA,
	FLEX_MSG_TYPE_HEX		= FLEX_FRAME_MSG_TYPE_HEX,
	FLEX_MSG_TYPE_INSTRUCTION	= FLEX_FRAME_MSG_TYPE_INSTRUCTION,
	FLEX_MSG_TYPE_SHORT		= FLEX_FRAME_MSG_TYPE_SHORT,
	FLEX_MSG_TYPE_SECURE		= FLEX_FRAME_MSG_TYPE_SECURE,
	FLEX_MSG_TYPE_SPECIAL_NUM	= FLEX_FRAME_MSG_TYPE_SPECIAL_NUM,
	FLEX_MSG_TYPE_NUMBERED_NUM	= FLEX_FRAME_MSG_TYPE_NUMBERED_NUM,
	FLEX_MSG_TYPE_NUMBERED_SPECIAL	= FLEX_FRAME_MSG_TYPE_NUMBERED_SPECIAL,
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
	double			polarity;		/* +1.0 = normal, -1.0 = inverted */
	int			priority;		/* 0 = normal, 1 = priority */
	int			charset;		/* 0 = ASCII, 1 = KANJI */
	int			is_temp_group;		/* 1 = deliver via Temporary Address (§5.2) */
	int			temp_delivery_slot;	/* internal: temp addr delivery slot (0-15)
						 * set by flex_tempgroup_enqueue(), -1 = normal.
						 * Frame builder uses FLEX_ADDR_TEMPORARY_MIN + slot
						 * as address word (§3.8.2.3). NOT user-settable. */
	char			source_id[64];		/* source/callback identifier ('\0' = none) */
	int			short_msg_type;		/* FLEX_SMSG_TYPE_* (0-3, default 0=numeric) */
	int			short_msg_source;	/* source code S (0-7) for t=01/10 */
	int			short_msg_number;	/* message number N (0-63) for t=10 */
	int			short_msg_r;		/* retrieval flag R (0/1) for t=10 */
	int			blocking_length;	/* HEX/Binary B field: bits per character.
						 * 1-15 = that many bits, 0 = 16 bits.
						 * Default 1 (raw bits). */
	int			mail_drop;		/* M flag in alpha/hex header word 1:
						 * 0 = ordinary message (default)
						 * 1 = mail drop — pager may store/handle
						 *     separately from ordinary messages.
						 * Applies to both alpha and hex types. */
	int			phase;			/* phase (channel) override:
						 * -1=auto (default, scheduler assigns
						 * from capcode per ARIB STD-43A),
						 * 0=A, 1=B, 2=C, 3=D.
						 * Only meaningful for multi-phase modes:
						 *   3200/2FSK: 0=A, 1=C
						 *   3200/4FSK: 0=A, 1=C
						 *   6400/4FSK: 0=A, 1=B, 2=C, 3=D */

	/* secure / numbered numeric per-message fields */
	int			secure_subtype;		/* t1t0 pager-side type tag (0-3), default 0.
						 * Independent of wire encoding (secure_encoding). */
	int			secure_encoding;	/* wire encoding: 0=7-bit alpha, 1=raw binary.
						 * Controls how body bytes are packed on the wire. */
	int			numbered_r;		/* retrieval flag R, default 1.
						 * Retransmission scheduler sets R=0
						 * on retransmissions with same N. */
	int			numbered_s;		/* special format flag S: set automatically
						 * from msg_type (0 for NUMBERED_NUM,
						 * 1 for NUMBERED_SPECIAL). Not operator-set. */
	int			numbered_msgnum;	/* message number N (0-63), -1 = auto-assign */

	/* fragmentation state (set by scheduler for long messages).
	 * All fragments of one message share the same retrieval_num,
	 * which becomes the N (message number) field in the alpha/hex
	 * header word.  Message numbers newly assigned to a message
	 * must be unique to identify fragments of the same message. */
	int			fragment_index;
	int			total_fragments;
	uint32_t		retrieval_num;	/* → N field (6 bits, 0-63) */

	/* retransmission scheduling state */
	int			retransmit_max;		/* 0-15: retransmissions after initial TX (0=none) */
	int			retransmit_count;	/* retransmissions completed so far */
	int			retransmit_interval;	/* frames between retransmissions (1-1920, default 128) */
	int			send_delay;		/* frames to defer initial TX (0-1920, default 0) */
	uint32_t		next_send_frame;	/* absolute frame (cycle*128+frame) for next eligibility */
	int			assigned_n;		/* N assigned at initial TX (-1=unassigned, 0-63) */

	/* System message transmission method (§3.9.2, Fig. 3.7.2-2).
	 *   'a' = BIW101 only (no operator address, vector at end of VF)
	 *   'b' = BIW101 + Operator Messaging Address (default for LSB 0-3)
	 *   'c' = Operator Messaging Address only (no BIW101)
	 *   0   = not a system message (normal pager message) */
	char			sysmsg_method;
} flex_msg_t;

/* TX temp group slot states */
#define FLEX_TG_FREE		0	/* slot available */
#define FLEX_TG_SETUP		1	/* SETUP instructions queued/sending */
#define FLEX_TG_DELIVERY	2	/* DELIVERY message queued/sending */

/* ===== Per-Polarity TX Scheduling State (Req 1, 7) =====
 *
 * Each polarity is an independent logical network with its own
 * message queue, N counters, fragment tracking, BIW carousel,
 * and temp group slots.  Indexed by FLEX_POL_NORMAL (0) or
 * FLEX_POL_INVERTED (1). */

/* TX polarity count — matches RX side */
#define FLEX_TX_POLARITIES	2

/* N_Counter table: per-(capcode, polarity) message numbering.
 * LRU-bounded to prevent unbounded memory growth. */
#define FLEX_N_COUNTER_MAX	25000
#define FLEX_N_COUNTER_BUCKETS	4096

/* Message queue depth limit per polarity */
#define FLEX_MSG_QUEUE_MAX	32768

/* BIW carousel: tracks last-transmitted frame per BIW type per phase.
 * Used for least-recently-transmitted rotation. */
enum flex_biw_type_id {
	BIW_SSID1 = 0,		/* type 000 */
	BIW_DATE,		/* type 001 */
	BIW_TIME,		/* type 010 */
	BIW_SYSINFO_TZ,	/* type 101, A=0100 */
	BIW_SYSINFO_MSG,	/* type 101, A=0000-0011 */
	BIW_CHAN_SETUP,		/* type 101, A=0110 */
	BIW_SSID2,		/* type 111 */
	BIW_TYPE_COUNT
};

typedef struct flex_biw_carousel {
	uint32_t	last_tx_abs[BIW_TYPE_COUNT];	/* absolute frame of last TX per type */
} flex_biw_carousel_t;

/* N_Counter entry: hash-chained + LRU doubly-linked list node */
typedef struct flex_n_counter {
	uint64_t	capcode;
	uint8_t		n_value;	/* 0-63, wrapping */
	uint32_t	last_used_abs;	/* absolute frame for LRU ordering */
	struct flex_n_counter *hash_next;	/* hash chain */
	struct flex_n_counter *lru_prev;	/* LRU doubly-linked list */
	struct flex_n_counter *lru_next;
} flex_n_counter_t;

/* In-flight fragment tracking entry (max FLEX_MAX_INFLIGHT per polarity) */
typedef struct flex_inflight_frag {
	uint64_t	capcode;
	uint32_t	retrieval_num;		/* N field (0-63) */
	int		total_fragments;
	int		last_sent_idx;		/* last fragment_index transmitted */
	uint32_t	last_sent_abs;		/* absolute frame of last fragment */
	uint32_t	first_sent_abs;		/* absolute frame of first fragment */
	int		active;			/* 1 = in progress */
} flex_inflight_frag_t;

/* Per-polarity TX scheduling state container */
typedef struct flex_tx_polarity {
	/* Message queue (singly-linked list) */
	flex_msg_t		*msg_list;
	int			msg_count;

	/* Per-capcode message numbering (LRU hash table) */
	flex_n_counter_t	**n_hash_buckets;	/* hash bucket array [FLEX_N_COUNTER_BUCKETS] */
	flex_n_counter_t	*n_lru_head;		/* most recently used */
	flex_n_counter_t	*n_lru_tail;		/* least recently used (eviction candidate) */
	int			n_counter_count;	/* current entries in table */

	/* Fragment retrieval counter (0-63, wrapping) */
	uint32_t		frag_retrieval_seq;

	/* In-flight fragment tracking */
	flex_inflight_frag_t	inflight[FLEX_MAX_INFLIGHT];
	int			n_inflight;

	/* BIW carousel state: per-phase rotation tracking */
	flex_biw_carousel_t	biw_carousel[FLEX_MAX_PHASES];

	/* TX temp group slots (16 slots) */
	struct {
		int		state;		/* FLEX_TG_FREE/SETUP/DELIVERY */
		uint64_t	capcodes[FLEX_TEMP_GROUP_MAX_MEMBERS];
		int		count;
		int		setups_sent;
		uint32_t	target_frame;
		uint32_t	setup_abs;
		flex_msg_t	*delivery_msg;
	} tx_temp[FLEX_TEMP_ADDR_SLOTS];

	/* Last frame transmitted on this polarity */
	uint32_t		last_abs_frame;
} flex_tx_polarity_t;

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
	uint64_t		scan_start;		/* initial scan_from for progress */
	double			scan_start_time;	/* wall clock at scan start */
	uint64_t		scan_last_progress;	/* last capcode when progress was printed */

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
	int			chan_setup_enabled;	/* BIW channel setup emission */
	int			hack_nonstandard_decoders; /* block-boundary fixup for PDW/multimon-ng */
	int			lpf_enabled;		/* baseband LPF */

	/* Multiple transmission config (Spec Section 3.4.2).
	 *
	 * When num_transmissions > 1, the 88-word frame is divided into
	 * N subframes, each transmitted at a repeat interval equal to
	 * the System Collapse cycle (2^m frames).
	 *
	 * Subframe sizes (Fig. 3.4.2-1):
	 *   2x: 44 words per subframe (2 subframes)
	 *   3x: 29 words per subframe (3 subframes, +1 extra word)
	 *   4x: 22 words per subframe (4 subframes)
	 *
	 * The repeat unit = num_transmissions × repeat_interval frames.
	 * Word numbering within each subframe starts at 0.
	 * Word 0 always contains the Block Information Word.
	 *
	 * FIW encoding: r=1, [t1,t0]=num_tx, [t3,t2]=td_collapse.
	 * Per spec: "Carry On is not allowed for multiple transmission." */
	int			num_transmissions;	/* 1 (default), 2, 3, or 4 */
	int			td_collapse;		/* -1=use system collapse (default),
						 * 5, 6, or 7 = TD Collapse override */

	/* Per-slot repeat state for multi-transmission scheduling (§3.4.2).
	 *
	 * The 88-word frame has N subframe slots (N = num_transmissions).
	 * Each slot carries independent content.  The same content in a
	 * slot is retransmitted N times across N frames in the repeat unit.
	 * Slot position = copy number for that content group.
	 *
	 * Scheduling flow per frame:
	 *   1. For each slot: if slot_copy > 0 and < num_transmissions,
	 *      re-emit cached content (next copy).  Increment slot_copy.
	 *   2. For free slots (slot_copy == 0 or == num_transmissions):
	 *      pick messages from queue, encode into subframe, cache.
	 *      Set slot_copy = 1.
	 *   3. If no messages for a free slot: emit idle BIW.
	 *   4. If all slots idle and queue empty: can switch to non-repeat.
	 *   5. Assemble N subframes into 88-word frame at their offsets.
	 *
	 * slot_copy[i]:
	 *   0 = free (no content, emit idle)
	 *   1..N = copy number transmitted so far
	 *   When slot_copy reaches num_transmissions, slot becomes free.
	 *
	 * slot_content[i]: cached encoded subframe words (sf_words long).
	 *   Stored after first encoding so copies 2..N are identical
	 *   ("the same bit stream" per spec). */
	int			slot_copy[FLEX_RX_MAX_SUBFRAMES];
	uint32_t		slot_content[FLEX_RX_MAX_SUBFRAMES][FLEX_WORDS_PER_FRAME];
	int			slot_content_words[FLEX_RX_MAX_SUBFRAMES];

	/* Parameter change guard (§3.4.2).
	 *
	 * "If the system has paging information being transmitted when
	 * changing the collapse cycle, transmission speed, modulation
	 * scheme or the number of repeated transmissions, the base
	 * station waits until transmission of the multiple transmission
	 * units for the paging information is completed, then changes
	 * these parameters.  Once the parameters are changed, Frames
	 * without paging information are transmitted for the duration
	 * of one repeat unit at the value before the change."
	 *
	 * States:
	 *   0 = normal (no pending change)
	 *   1 = draining: waiting for current repeat unit to complete
	 *   2 = cooldown: sending idle frames at old params for one repeat unit
	 */
	int			param_change_state;
	uint32_t		param_change_deadline;	/* absolute frame when current state ends */
	/* Snapshot of old parameters during change transition */
	int			old_collapse;
	int			old_num_transmissions;
	int			old_td_collapse;
	int			old_bitrate;
	int			old_mod_type;
	/* Pending new parameters (applied after cooldown) */
	int			pending_collapse;
	int			pending_num_transmissions;
	int			pending_td_collapse;

	/* per-message defaults (CLI -M message and FIFO fallback) */
	int			default_speed;		/* 1600, 3200, 6400 — default 1600 */
	double			default_polarity;	/* +1.0 = normal, -1.0 = inverted */
	int			default_charset;	/* 0 = ASCII, 1 = KANJI — default 0 */
	int			default_phase;		/* -1=auto (scheduler), 0=A, 1=B, 2=C, 3=D */
	int			default_blocking_length; /* HEX/Binary B field default: 1-15 bits/char,
						  * 0=16. Default 1 (raw bits). */

	/* fixed-mode flags (CLI --speed / --polarity lock) */
	int			fixed_speed;		/* -1 = not fixed, else 1600/3200/6400 */
	int			fixed_mod_type;		/* FLEX_MOD_2FSK (default) or FLEX_MOD_4FSK */
	double			fixed_polarity;		/* 0.0 = not fixed, else +1.0(normal)/-1.0(inverted) */

	/* current frame parameters (set per-frame by scheduler/state machine) */
	int			current_frame_speed;
	int			current_frame_mod_type;	/* FLEX_MOD_2FSK or FLEX_MOD_4FSK */
	double			current_frame_polarity;

	/* roaming config */
	uint32_t		ssid;			/* System Sub-ID */
	uint32_t		nid;			/* Network ID */
	uint32_t		country_code;		/* SSID2 country code (10 bits, ITU-T E.212) */
	uint32_t		tmf;			/* SSID2 traffic management flags (4 bits) */
	int			timezone_code;		/* SysInfo timezone zone code (0-31, -1=auto) */
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

	/* Per-polarity TX scheduling state (Req 1, 7).
	 * tx_pol[0] = normal, tx_pol[1] = inverted.
	 * In single-polarity mode, only tx_pol[0] is used.
	 * The old global msg_list, msg_sequence, frag_retrieval_seq,
	 * and tx_temp[] above are retained for backward compatibility
	 * during migration — new code should use tx_pol[]. */
	flex_tx_polarity_t	tx_pol[FLEX_TX_POLARITIES];
	int			last_tx_polarity;	/* 0=normal, 1=inverted — for round-robin */

	/* ===== TX Temporary Group Scheduling (§3.8.2.3, §3.9.6) =====
	 *
	 * Manages the 3-phase temp group protocol on the TX side:
	 *   SETUP → DELIVERY → TEARDOWN
	 *
	 * FIFO command: "tempgroup:cap1 cap2 cap3,type,options,message"
	 *
	 * Flow:
	 *   1. Allocate a free slot (0-15) on the target phase
	 *   2. Enqueue SETUP: instruction messages for each capcode
	 *      (individual addr + short instruction vector type=000)
	 *   3. Enqueue DELIVERY: temp address word + vector + message
	 *      (scheduled for target_frame from SETUP)
	 *   4. TEARDOWN: free slot when delivery completes (C=0)
	 *
	 * Per spec: "The first transmission of the Group message must
	 * be started within 128 Frames from the first transmission of
	 * the Short Instruction Vector." */

	/* TX temp group slot state */
	struct {
		int		state;		/* FLEX_TG_FREE/SETUP/DELIVERY */
		uint64_t	capcodes[FLEX_TEMP_GROUP_MAX_MEMBERS];
		int		count;		/* number of member capcodes */
		int		setups_sent;	/* SETUP instructions sent so far */
		uint32_t	target_frame;	/* frame for DELIVERY (f6-f0) */
		uint32_t	setup_abs;	/* absolute frame of first SETUP */
		flex_msg_t	*delivery_msg;	/* pointer to DELIVERY message in queue */
	} tx_temp[FLEX_TEMP_ADDR_SLOTS];	/* 16 slots (shared across phases for TX) */

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

		/* FIW decode result (Spec Section 3.6, Fig. 3.6-1) */
		uint32_t	fiw_cycle;
		uint32_t	fiw_frame;
		uint32_t	fiw_roaming;		/* n: 1=roaming allowed */
		uint32_t	fiw_repeat;		/* r: 1=multiple transmission active */
		uint32_t	fiw_traffic;		/* t3-t0: depends on r value */

		/* Multiple transmission state (Spec Section 3.4.2).
		 *
		 * When r=1, [t1,t0] encode the number of transmissions:
		 *   01=2x, 10=3x, 11=4x, 00=reserved.
		 * And [t3,t2] encode the TD Collapse cycle override:
		 *   00=use System Collapse, 01=value 6, 10=value 7, 11=value 5.
		 *
		 * When r=0, the number of transmissions is 1x and [t3-t0]
		 * are Low Traffic Flags per phase (Section 3.6).
		 *
		 * A Frame is divided into N subframes (N = num_transmissions).
		 * Subframe word counts: 2x=44, 3x=29(+1 extra), 4x=22.
		 * Each subframe is transmitted at a repeat interval equal
		 * to the System Collapse cycle (2^m frames). */
		int		fiw_num_transmissions;	/* 1, 2, 3, or 4 */
		int		fiw_td_collapse;	/* -1=use system, else 5/6/7 */

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

		/* Per-phase channel data.
		 * Each phase is an independent channel with 88 words
		 * and per-word BCH status. */
		flex_phase_data_t phase[FLEX_MAX_PHASES];
		int		phase_toggle;		/* alternates 0/1 at 3200 baud */
		int		data_bit_counter;	/* de-interleave index counter */

		/* Per-phase temporary group tracking.
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
		 *   b) 128 frames elapse without delivery (timeout, ~2 minutes).
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
		} temp_addr_map[FLEX_RX_POLARITIES][FLEX_MAX_PHASES][FLEX_TEMP_ADDR_SLOTS];

		/* BIW state — track SSID/coverage/timezone/date/time across frames.
		 * Log at NOTICE level when values change, DEBUG when repeated. */
		struct {
			uint32_t	local_id;
			uint32_t	coverage;
			uint32_t	country;
			uint32_t	tmf;
			int		seen;		/* 1 = at least one SSID1 received */
			int		seen_ssid2;	/* 1 = at least one SSID2 received */
			/* Date (type 001) */
			int		date_year;	/* decoded year (with equiv mapping) */
			uint32_t	date_month;
			uint32_t	date_day;
			int		seen_date;
			/* Time (type 010) */
			uint32_t	time_hour;
			uint32_t	time_minute;
			uint32_t	time_second;	/* raw 3-bit value (0-7, ×7.5s) */
			int		seen_time;
			/* SysInfo timezone (type 101, A=0100/0101) */
			uint32_t	timezone_zone;	/* 5-bit zone code (Z4-Z0) */
			int		timezone_offset_min; /* UTC offset in minutes */
			int		timezone_dst;	/* L0: 0=DST, 1=standard time */
			uint32_t	timezone_extsec; /* S5-S3: extended seconds (0-7,
						  * 1/64 min = 0.9375s steps) */
			int		seen_timezone;
			/* SysInfo system message (type 101, A=0000~0011) */
			int		sysmsg_a_type;	/* -1=none, 0-3=A-type */
		} biw[FLEX_RX_POLARITIES];

		/* Fragment reassembly state.
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
			int		kanji;		/* 1 = kanji/Shift-JIS 16-bit extraction
						 * Set from rx_kanji_enabled on first
						 * fragment so all fragments in a stream
						 * use consistent 16-bit extraction. */
			int		secure_subtype;	/* t1t0 value from secure message header
						 * (-1 = not a secure message) */
			uint32_t	rx_sig;		/* signature from first fragment */
			uint32_t	sig_sum;	/* accumulated signature sum */
			int		sig_valid;	/* 1 = all fragments had clean/corrected words */
			char		word_status[256]; /* per-word BCH status across fragments */
			int		ws_len;		/* length of word_status */
			char		buf[FLEX_REASM_MAX_LEN];
			int		len;		/* bytes accumulated */
			uint32_t	last_frame;	/* frame number of last fragment */
			uint32_t	last_cycle;	/* cycle number of last fragment */
			int		active;		/* 1 = in progress */
		} reasm[FLEX_RX_POLARITIES][FLEX_REASM_SLOTS];

		/* Subframe combining state (Req 15-16).
		 * Per-phase storage for word-level results across repeated subframes.
		 * Only active when fiw_num_transmissions > 1.
		 * Key: (cycle, frame%repeat_interval, phase, collapse parameters).
		 * No message awareness — all 88 words benefit regardless of capcode. */
		flex_rx_subframe_store_t subframe_store[FLEX_RX_POLARITIES][FLEX_MAX_PHASES];

		/* Message history for retransmission detection (Req 17-18).
		 * Stores recently decoded messages keyed by (capcode, N, vec_type, phase)
		 * with validation fields (word_count, frag_count, is_long, blocking)
		 * for duplicate suppression and cross-retransmission word recovery.
		 * This is a separate structure from the subframe_store — it operates
		 * at the message layer after parsing, not at the frame/physical layer. */
		flex_rx_msg_entry_t msg_history[FLEX_RX_POLARITIES][FLEX_RX_MSG_HISTORY_MAX];
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

/* TX Temporary Group (§3.8.2.3) */
int flex_tempgroup_enqueue(flex_t *flex, const uint64_t *capcodes,
			   const int *collapses, int count,
			   enum flex_msg_type msg_type, const char *data,
			   int data_length, int speed, int modulation_type,
			   double polarity, int priority, int phase);
void flex_tempgroup_tick(flex_t *flex, uint32_t abs_frame);

#endif /* FLEX_H */
