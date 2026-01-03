#include "../libmobile/sender.h"

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
 *   - 8 frames × 2 codewords = 16 codewords
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
};

enum pocsag_language {
	LANGUAGE_DEFAULT = 0,
	LANGUAGE_GERMAN,
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

	/* tx states */
	enum pocsag_state	state;			/* state (idle, preamble, message) */
	pocsag_msg_t		*current_msg;		/* msg, if message codewords are transmitted */
	int			word_count;		/* counter for codewords */
	int			idle_count;		/* counts when to go idle */
	uint32_t		scan_from, scan_to;	/* if not equal: scnning mode */

	/* rx states */
	int			rx_msg_valid;		/* currently in receiving message state */
	uint32_t		rx_msg_ric;		/* ric of message */
	enum pocsag_function	rx_msg_function;	/* sub-address of message */
	enum pocsag_msg_type	rx_msg_type;		/* detected message type */
	char			rx_msg_data[256];	/* data buffer */
	int			rx_msg_data_length;	/* complete characters received */
	int			rx_msg_bit_index;	/* current bit received for alphanumeric */

	/* calls */
	pocsag_msg_t		*msg_list;		/* linked list of all calls */

	/* dsp states */
	double			fsk_deviation;		/* deviation of FSK signal on sound card */
	double			fsk_polarity;		/* polarity of FSK signal (-1.0 = bit '1' is down) */
	sample_t		fsk_ramp_up[256];	/* samples of upward ramp shape */
	sample_t		fsk_ramp_down[256];	/* samples of downward ramp shape */
	double			fsk_bitduration;	/* duration of a bit in samples */
	double			fsk_bitstep;		/* fraction of a bit each sample */
	sample_t		*fsk_tx_buffer;		/* tx buffer for one data block */
	int			fsk_tx_buffer_size;	/* size of tx buffer (in samples) */
	int			fsk_tx_buffer_length;	/* usage of buffer (in samples) */
	int			fsk_tx_buffer_pos;	/* current position sending buffer */
	double			fsk_tx_phase;		/* current bit position */
	uint8_t			fsk_tx_lastbit;		/* last bit of last message, to correctly ramp */
	double			fsk_rx_phase;		/* current sample position */
	uint8_t			fsk_rx_lastbit;		/* last bit of last message, to detect level */
	uint32_t		fsk_rx_word;		/* shift register to receive codeword */
	int			fsk_rx_sync;		/* counts down to next sync */
	int			fsk_rx_index;		/* counts bits of received codeword */
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
void pocsag_msg_receive(enum pocsag_language language, const char *channel, uint32_t ric, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message);
int pocsag_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int tx, int rx, enum pocsag_language language, int baudrate, double deviation, double polarity, enum pocsag_function function, enum pocsag_msg_type msg_type, const char *message, char padding, uint32_t scan_from, uint32_t scan_to, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback);
void pocsag_destroy(sender_t *sender);
void pocsag_msg_send(enum pocsag_language language, const char *text, size_t text_length);
void pocsag_msg_destroy(pocsag_msg_t *msg);
void pocsag_get_id(pocsag_t *euro, char *id);
void pocsag_receive_id(pocsag_t *euro, char *id);
void pocsag_msg_done(pocsag_t *pocsag);
