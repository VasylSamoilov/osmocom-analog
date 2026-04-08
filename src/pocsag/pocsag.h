#include "../libmobile/sender.h"
#include "../libfsk_pll/fsk_pll.h"

/*
 * POCSAG RIC (Radio Identity Code) - Pager Address Structure:
 *
 * The RIC is a 21-bit unique identifier for each pager, providing up to
 * 2,097,152 (2^21) unique addresses. Per CCIR Rec. 584 (POCSAG standard):
 *
 * RIC Formation:
 *   The 21-bit RIC is split across two components:
 *
 *   1. Frame Position (3 LSBs): Bits 0-2 of the RIC
 *      - Determines which of the 8 frames within a batch the pager listens to
 *      - Calculated as: frame = RIC & 0x7 (i.e., RIC mod 8)
 *      - Each batch contains 8 frames (frame 0 through frame 7)
 *      - Each frame contains 2 codeword slots (slot 0 and slot 1)
 *      - Pagers power-save by only waking up during their designated frame
 *
 *   2. Address Codeword (18 MSBs): Bits 3-20 of the RIC
 *      - Transmitted in bits 2-19 of the address codeword
 *      - Calculated as: address_bits = RIC >> 3
 *
 * Batch Structure:
 *   - 1 sync codeword (0x7CD215D8)
 *   - 8 frames x 2 codewords = 16 codewords
 *   - Total: 17 codewords per batch (544 bits at 32 bits each)
 *
 * Valid RIC Range:
 *   - Minimum: 0
 *   - Maximum: 2,097,151 (2^21 - 1 = 0x1FFFFF)
 *
 * Reserved/Invalid RICs:
 *   - 2007664-2007671: Reserved for idle codeword pattern
 *     The idle codeword (0x7A89C197) when decoded as an address codeword
 *     gives: (0x7A89C197 >> 10) & 0x1FFFF8 = 2007664.
 *     All 8 frame positions (2007664-2007671) are reserved.
 *     Using these RICs would cause pagers to misinterpret idle codewords.
 *
 * Example RIC Breakdown:
 *   RIC = 1234567 (decimal) = 0x12D687
 *   - Frame position: 1234567 & 7 = 7 (frame 7)
 *   - Address bits: 1234567 >> 3 = 154320 = 0x25AD0
 *   - Pager wakes only during frame 7 of each batch
 *
 * Effective Addresses per Frame:
 *   Since there are 8 frames, each frame handles 2^18 = 262,144 RICs.
 *   With 4 function sub-addresses per RIC, each frame serves 1,048,576
 *   unique pager addresses.
 */

/*
 * POCSAG Function Bits (also called "FC" or "Function Code"):
 *
 * According to the original CCIR/ETSI POCSAG specification (CCIR Rec. 584,
 * POCSAG 512/1200/2400), the function field is ONLY:
 *   - A 2-bit value (0-3)
 *   - Used to provide four possible sub-addresses for a given pager
 *   - With NO semantic meaning assigned to message type
 *
 * The POCSAG standard does NOT mandate any correlation between function bits
 * and message type. Message type (tone-only, numeric, alphanumeric) is
 * determined by the CONTENT of the message codewords, not by the function bits.
 *
 * Any correlation that exists comes from specific pager manufacturers, not
 * from the protocol itself:
 *
 *   - Motorola (Advisor, Bravo, etc.): Each capcode has 4 sub-addresses
 *     (function bits), each can be programmed with a different alert type,
 *     but there is no fixed link to message encoding.
 *
 *   - Swissphone (RE529/RE629 series): Function bits correspond to
 *     Mailbox A, B, C, D - again no fixed link to message type.
 *
 * Therefore, this implementation treats function bits as independent from
 * message type. The enum values are named neutrally (A, B, C, D) to avoid
 * implying any message type correlation.
 */
enum pocsag_function {
	POCSAG_FUNCTION_A = 0,	/* Function 0: Sub-address A */
	POCSAG_FUNCTION_B,	/* Function 1: Sub-address B */
	POCSAG_FUNCTION_C,	/* Function 2: Sub-address C */
	POCSAG_FUNCTION_D,	/* Function 3: Sub-address D */
};

/* POCSAG Message Type - defines content encoding (see function bits docs above) */
enum pocsag_msg_type {
	POCSAG_MSG_TYPE_AUTO = 0,	/* Auto-detect based on content */
	POCSAG_MSG_TYPE_TONE,		/* Tone-only (no message codewords) */
	POCSAG_MSG_TYPE_NUMERIC,	/* BCD-encoded numeric message */
	POCSAG_MSG_TYPE_ALPHA,		/* 7-bit ASCII alphanumeric message */
};

extern const char *pocsag_function_name[4];

enum pocsag_state {
	POCSAG_IDLE = 0,
	POCSAG_PREAMBLE,
	POCSAG_MESSAGE,
	POCSAG_SILENCE,		/* TX silence gap between speed/polarity switches */
};

enum pocsag_language {
	LANGUAGE_DEFAULT = 0,
	LANGUAGE_GERMAN,		/* unknown-unknown-german */
	LANGUAGE_SKYPER,		/* nec-skyper-categories */
	LANGUAGE_CYRILLIC,		/* motorola-advisor_linguist-cyrillic */
};

struct pocsag;

/* instance of outgoing message */
typedef struct pocsag_msg {
	struct pocsag_msg	*next;
	struct pocsag		*pocsag;
	int			callref;		/* call reference */
	uint32_t		ric;			/* Full 21-bit RIC: upper 18 bits transmitted, lower 3 bits = frame */
	enum pocsag_function	function;		/* sub-address (0-3) */
	enum pocsag_msg_type	msg_type;		/* message encoding type */
	char			data[256];		/* message to be transmitted */
	int			data_length;		/* length of message that is not 0-terminated */
	int			data_index;		/* current character transmitting */
	int			bit_index;		/* current bit transmitting */
	char			padding;		/* EOT or other padding */

	/* per-message TX parameters */
	int			speed;			/* baud rate: 512, 1200, 2400 */
	double			polarity;		/* -1.0 = normal, +1.0 = inverted */

	/* retransmission scheduling */
	int			retransmit_max;		/* total retransmissions after initial TX (0=none) */
	int			retransmit_count;	/* retransmissions completed so far */
	double			retransmit_interval;	/* seconds between retransmissions */
	double			send_delay;		/* seconds to defer initial TX (0=immediate) */
	double			next_send_time;		/* wall-clock time (seconds since epoch) for next eligibility */
} pocsag_msg_t;

/* instance of pocsag transmitter/receiver */
typedef struct pocsag {
	sender_t		sender;

	/* system info */
	int			tx; 			/* can transmit */
	int			rx;			/* can receive */
	enum pocsag_language	language;		/* special characters */
	enum pocsag_function	default_function;	/* default sub-address */
	enum pocsag_msg_type	default_msg_type;	/* default message type for encoding */
	const char 		*default_message;	/* default message, if caller has no caller ID */
	char			padding;		/* EOT or other padding */
	int			default_speed;		/* default baud rate from CLI */
	double			default_polarity;	/* default polarity from CLI */
	int			max_batches;		/* 0 = locked to CLI speed/polarity */

	/* tx states */
	enum pocsag_state	state;			/* state (idle, preamble, message) */
	pocsag_msg_t		*current_msg;		/* msg, if message codewords are transmitted */
	int			word_count;		/* counter for codewords */
	int			idle_count;		/* counts when to go idle */
	int			batch_count;		/* batches sent in current transmission */
	int			tx_speed;		/* baud rate of current transmission */
	double			tx_polarity;		/* polarity of current transmission */
	uint32_t		scan_from, scan_to;	/* if not equal: scnning mode */

	/* rx states */
	int			rx_msg_valid;		/* currently in receiving message state */
	uint32_t		rx_msg_ric;		/* ric of message */
	enum pocsag_function	rx_msg_function;	/* sub-address of message */
	enum pocsag_msg_type	rx_msg_type;		/* detected message type */

	/* rx message buffers — all dynamic (malloc/realloc) */
	char			*rx_msg_data;		/* alpha decode buffer */
	uint8_t			*rx_msg_char_status;	/* alpha per-char: 0=ok 1=corrected 2=bad */
	int			rx_msg_data_length;	/* alpha chars received */
	int			rx_msg_data_alloc;	/* alpha buffer allocated size */

	char			*rx_msg_data_skyper;	/* skyper decode buffer (derived from alpha) */
	int			rx_msg_data_length_skyper;

	char			*rx_msg_data_numeric;	/* numeric decode buffer */
	uint8_t			*rx_msg_num_status;	/* numeric per-char status */
	int			rx_msg_data_length_numeric;
	int			rx_msg_numeric_alloc;	/* numeric buffer allocated size */

	uint8_t			*rx_msg_numeric_nibbles;/* raw 4-bit nibbles (before char mapping) */
	int			rx_msg_numeric_nibble_count;
	int			rx_msg_nibble_alloc;	/* nibble buffer allocated size */

	uint8_t			*rx_msg_data_raw;	/* raw message bytes (packed bits) */
	int			rx_msg_data_raw_bits;
	int			rx_msg_raw_alloc;	/* raw buffer allocated size */

	int			rx_msg_bit_index;	/* current bit within 7-bit alpha char */
	uint8_t			rx_msg_cur_char_bad;	/* accumulates status for current alpha char */

	/* rx message stats */
	int			rx_msg_codewords;	/* total codewords in this message */
	int			rx_msg_corrected;	/* BCH-corrected codewords */
	int			rx_msg_uncorrectable;	/* uncorrectable codewords */

	/*
	 * Sliding BER window: tracks error rates over last N codewords.
	 * Persistent across messages for continuous signal quality monitoring.
	 */
#define POCSAG_BER_WINDOW	32
	struct {
		int		errors[POCSAG_BER_WINDOW];	/* bit errors per codeword slot */
		int		total[POCSAG_BER_WINDOW];	/* total bits per slot (32) */
		int		head;				/* ring buffer write position */
		int		count;				/* slots filled */
	} rx_ber;

	/*
	 * RX message history for retransmission dedup and recovery.
	 *
	 * POCSAG pagers have configurable dedup windows (15s, 30s, 1m, 2m).
	 * Base stations retransmit the same message 2-3 times within this window.
	 * We store recent messages keyed by (ric, function) and compare incoming
	 * messages against the history:
	 *   - If a duplicate is found within the dedup window, suppress the alarm
	 *     and attempt to recover uncorrectable codewords from the previous copy.
	 *   - If no match, treat as a new message.
	 */
#define POCSAG_RX_HISTORY_MAX		64
#define POCSAG_RX_HISTORY_CW_MAX	256	/* max stored codewords per entry */
	struct pocsag_rx_history_entry {
		uint32_t		ric;
		enum pocsag_function	function;
		enum pocsag_msg_type	msg_type;
		int			baudrate;
		double			polarity;
		int			codeword_count;		/* message codewords stored */
		uint32_t		codewords[POCSAG_RX_HISTORY_CW_MAX];
		int8_t			cw_status[POCSAG_RX_HISTORY_CW_MAX]; /* 0=ok, 1=corrected, -1=uncorrectable */
		double			timestamp;		/* wall-clock time when received */
		int			active;
	} rx_history[POCSAG_RX_HISTORY_MAX];
	double			rx_dedup_window;	/* dedup window in seconds (0=disabled) */

	/* current message raw codewords for history storage */
	int			rx_msg_cw_count;
	uint32_t		rx_msg_cw_buf[POCSAG_RX_HISTORY_CW_MAX];
	int8_t			rx_msg_cw_status[POCSAG_RX_HISTORY_CW_MAX];

	/* calls */
	pocsag_msg_t		*msg_list;		/* linked list of all calls */

	/* dsp states */
	double			fsk_deviation;		/* deviation of FSK signal on sound card */
	double			fsk_polarity;		/* polarity of FSK signal (-1.0 = bit '1' is down) — RX */
	double			fsk_tx_polarity;	/* TX polarity (may differ during speed switching) */
	sample_t		fsk_ramp_up[256];	/* samples of upward ramp shape */
	sample_t		fsk_ramp_down[256];	/* samples of downward ramp shape */
	double			fsk_bitduration;	/* duration of a bit in samples — used for TX buffer sizing */
	double			fsk_tx_bitduration;	/* TX bit duration (may differ during speed switching) */
	double			fsk_tx_bitstep;		/* TX bit step */
	sample_t		*fsk_tx_buffer;		/* tx buffer for one data block */
	int			fsk_tx_buffer_size;	/* size of tx buffer (in samples) */
	int			fsk_tx_buffer_length;	/* usage of buffer (in samples) */
	int			fsk_tx_buffer_pos;	/* current position sending buffer */
	double			fsk_tx_phase;		/* current bit position */
	uint8_t			fsk_tx_lastbit;		/* last bit of last message, to correctly ramp */
	fsk_pll_t		fsk_rx_pll;		/* locked decoder PLL (FLEX-style) */
	uint64_t		fsk_rx_word;		/* 64-bit shift register: upper 32 = history, lower 32 = sync/codeword */
	int			fsk_rx_sync;		/* counts down to next sync */
	int			fsk_rx_index;		/* counts bits of received codeword */
	int			fsk_rx_bit_count;	/* bits since last FSC (for non-FSC codeword check) */

	/* auto-detection state for baud rate and polarity */
	int			rx_auto_baud;		/* 1 = auto-detect baud rate */
	int			rx_auto_polarity;	/* 1 = auto-detect polarity */
	int			rx_baud_locked;		/* baud rate currently locked */
	double			rx_polarity_locked;	/* polarity currently locked */
	int			samplerate;		/* stored for reconfiguration */
	int			rx_rate_locked;		/* 1 = locked to a baud rate (between batches) */
	int			rx_resync_countdown;	/* bits remaining to find next FSC before unlock */

	/*
	 * Multi-rate receiver: up to 3 parallel decoders (512/1200/2400).
	 * Each tracks its own phase and shift register independently.
	 * When not auto-detecting baud, only slot [0] is used.
	 */
	struct rx_baud_state {
		int		baudrate;	/* baud rate for this slot */
		fsk_pll_t	pll;		/* PLL instance (FLEX-style) */
		uint64_t	word;		/* 64-bit shift register: upper 32 = history, lower 32 = sync detect */
		uint64_t	word_inv;	/* same, inverted polarity */
	} rx_baud[3];
	int			rx_baud_count;	/* number of active baud slots */
} pocsag_t;

int msg_receive(const char *text);

int pocsag_function_name2value(const char *text);
int pocsag_msg_type_name2value(const char *text);
const char *pocsag_msg_type_name(enum pocsag_msg_type type);
void pocsag_list_channels(void);
double pocsag_channel2freq(const char *kanal, double *deviation, double *polarity, int *baudrate);
const char *pocsag_number_valid(const char *number);
void pocsag_add_id(const char *id);
int pocsag_init(void);
void pocsag_exit(void);
int pocsag_init(void);
void pocsag_exit(void);
void pocsag_new_state(pocsag_t *pocsag, enum pocsag_state new_state);
void pocsag_msg_receive(enum pocsag_language language, const char *channel, uint32_t ric, enum pocsag_function function, enum pocsag_msg_type msg_type, int baudrate, double polarity, const char *message);
int pocsag_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int tx, int rx, enum pocsag_language language, int baudrate, double deviation, double polarity, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message, char padding, uint32_t scan_from, uint32_t scan_to, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback, int auto_baud, int auto_polarity, double dedup_window, int max_batches);
void pocsag_destroy(sender_t *sender);
void pocsag_msg_send(enum pocsag_language language, const char *text, size_t text_length);
void pocsag_msg_destroy(pocsag_msg_t *msg);
void pocsag_get_id(pocsag_t *euro, char *id);
void pocsag_receive_id(pocsag_t *euro, char *id);
void pocsag_msg_done(pocsag_t *pocsag);
