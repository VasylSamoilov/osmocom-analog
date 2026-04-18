#include <time.h>
#include "../libmobile/sender.h"

enum gsc_msg_type {
	TYPE_AUTO = 0,	/* Defined by 7th digit */
	TYPE_TONE,	/* TONE only */
	TYPE_VOICE,	/* TONE + VOICE */
	TYPE_ALPHA,	/* TONE + DATA */
	TYPE_NUMERIC,	/* TONE + DATA */
};

/* RX decoder state machine */
enum rx_state {
	RX_IDLE,		/* waiting for preamble */
	RX_PREAMBLE,		/* receiving preamble codewords */
	RX_START_CODE,		/* expecting start code pair */
	RX_ADDRESS,		/* expecting address word pair */
	RX_DATA,		/* receiving data blocks */
	RX_VOICE,		/* voice message follows */
	RX_TONE,		/* tone-only message detected */
	RX_DONE			/* message decode complete */
};

/* Uncertainty threshold for dual-decode guess mode: when the absolute
 * difference between alpha and numeric content scores is below this
 * value, the result is flagged as uncertain. */
#define GSC_GUESS_UNCERTAIN_THRESHOLD 10

#define MAX_ADB		32	/* 256 characters (protocol uses continuation bit, no fixed limit) */
#define MAX_NDB		2	/* 24 digits */

/* instance of decoded (received) message */
typedef struct gsc_rx_msg {
	/* --- existing fields (unchanged) --- */
	char			address[8];		/* 7-digit functional address + NUL */
	int			address_number;		/* 1-4 */
	enum gsc_msg_type	type;			/* winner type: TYPE_ALPHA or TYPE_NUMERIC */
	char			data[MAX_ADB * 8 + 1];		/* winner's decoded message (for backward compat) */
	int			preamble_index;		/* 0-9 */
	int			error_count;		/* corrected errors (Golay/BCH succeeded with corrections) */
	int			uncorrectable_count;	/* uncorrectable errors (BCH decode failed entirely) */
	int			decode_ok;		/* 1 = success, 0 = failure */
	int			polarity_inverted;	/* 1 = received with inverted polarity */

	/* --- new fields for dual-decode guess mode --- */
	char			alpha_data[MAX_ADB * 8 + 1];	/* alpha interpretation string */
	char			numeric_data[MAX_NDB * 12 + 1];	/* numeric interpretation string */
	int			alpha_score;		/* content score for alpha interpretation */
	int			numeric_score;		/* content score for numeric interpretation */
	int			alpha_fill;		/* trailing fill count in alpha decode */
	int			numeric_fill;		/* trailing fill count in numeric decode */
	enum gsc_msg_type	guess_winner;		/* TYPE_ALPHA or TYPE_NUMERIC */
	int			guess_uncertain;	/* 1 = scores too close to be confident */
	int			guess_method;		/* 0 = fill-count, 1 = content-score */

	/* raw nibble array for numeric scoring (before character mapping) */
	uint8_t			numeric_nibbles[MAX_NDB * 12 + 1];	/* raw 4-bit nibbles */
	int			numeric_nibble_count;	/* number of nibbles decoded */
} gsc_rx_msg_t;


/* instance of outgoing message */
typedef struct gsc_msg {
	struct gsc_msg		*next;
	char			address[8];		/* 7 digits + EOL */
	enum gsc_msg_type	type;			/* type of message */
	char			data[256];		/* message to be transmitted */
	/* --- New fields --- */
	int			priority;		/* 0 = normal, 1 = priority */
	double			polarity;		/* per-message FSK polarity: 1.0 = normal, -1.0 = inverted, 0.0 = use instance default */
	struct timespec		enqueue_time;		/* CLOCK_MONOTONIC timestamp at enqueue */
	int			preamble_index;		/* cached (address[0]-'0' + address[2]-'0') % 10 */
} gsc_msg_t;

/* Known GSC Pager Models and Capabilities (Appendix III)
 *
 * DIMENSION 1000:
 *   2 codes, addresses 1-2 per code, Tone & Voice / Tone Only
 *   Individual call + third individual/group call on 2nd code
 *
 * BPR 2000 Display:
 *   1 code, addresses 1-4, Tone Only / Data
 *   Individual call only
 *
 * OPTRX Display:
 *   2 codes, addresses 1-4 per code, Tone Only / Tone & Voice / Data
 *   Individual call on 1st code, individual + group call on 2nd code
 *
 * Notes:
 *   - Only one pager function per address
 *   - 2nd code Word 2 must differ from 1st code Word 2
 *   - Address number (1-4) is determined by the function variable:
 *     function = (W1_inverted << 1) | W2_inverted
 */

enum gsc_batching_mode {
	BATCHING_OFF = 0,	/* individual mode (current behavior, default) */
	BATCHING_NORMAL = 1,	/* batch mode: up to 16 addresses per preamble */
	BATCHING_EXTENDED = 2,	/* extended batch: up to 32 addresses per preamble */
};

typedef struct gsc {
	sender_t		sender;
	int			tx;

	gsc_msg_t		*msg_list;		/* queue of messages */
	const char		*default_message;

	/* current trasmitting message */
	uint8_t			bit[16384];
	int			bit_num;
	int			bit_ac;			/* where activation code starts (voice only). */
	int			bit_index;		/* when playing out */
	int			bit_overflow;

	/* dsp states */
	double			fsk_deviation;		/* deviation of FSK signal on sound card */
	double			fsk_polarity;		/* FSK polarity: 1.0 = normal (per GSC standard), -1.0 = inverted */
	double			fsk_tx_polarity;	/* effective polarity for current TX message */
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

	/* voice message */
	int			wait_2_sec;		/* counter to wait 2 seconds before playback */
	char			wave_tx_filename[256];
	int			wave_tx_samplerate;
	int			wave_tx_channels;
	wave_play_t		wave_tx_play;		/* wave playback */
	samplerate_t		wave_tx_upsample;	/* wave upsampler */

	/* receive mode */
	int			rx;			/* receive mode enabled */
	int			rx_auto_polarity;	/* 1 = auto-detect polarity from preamble */
	int			rx_polarity_inverted;	/* 1 = current batch uses inverted polarity */

	/* RX bit buffer (mirrors TX bit buffer) */
	uint8_t			rx_bit[16384];		/* received bit buffer */
	int			rx_bit_num;		/* number of bits received */
	int			rx_bit_index;		/* current decode position */

	/* RX DSP state */
	double			fsk_rx_phase;		/* bit clock phase for recovery */
	sample_t		fsk_rx_last_sample;	/* previous sample for zero-crossing */
	int			fsk_rx_last_bit;	/* last demodulated bit */

	/* RX decoder state machine */
	enum rx_state		rx_state;		/* current decoder state */
	int			rx_preamble_index;	/* detected preamble index (0-9) */
	int			rx_preamble_count;	/* preamble codewords seen so far */

	/* RX preamble shift register (46 bits for one dup Golay codeword) */
	uint8_t			rx_shift[46];		/* circular buffer of last 46 bits */
	int			rx_shift_count;		/* total bits shifted in (saturates at 46) */
	int			rx_no_transition;	/* consecutive same-value bits (end-of-TX detect) */

	/* RX preamble confirmation phase:
	 * After the first preamble hit in RX_IDLE, we enter RX_PREAMBLE
	 * and collect additional 46-bit codewords. We require
	 * PREAMBLE_LOCK_THRESHOLD consecutive codewords matching the same
	 * preamble index before committing to a lock. Raw bits from each
	 * confirmed codeword are saved so we can reconstruct the rx_bit[]
	 * buffer without needing access to calc_golay(). */
	int			rx_confirm_index;	/* preamble index being confirmed (-1 = none) */
	uint8_t			rx_confirm_bits[46 * 18]; /* raw bits from confirmed preamble codewords */
	int			rx_confirm_count;	/* confirmed codewords so far (including initial) */
	int			rx_confirm_bit_count;	/* bits accumulated in 46-bit sub-register */

	/* RX voice recording */
	const char		*voice_dir;		/* output folder for voice recordings (NULL = disabled) */
	int			voice_monitor;		/* 1 = also play voice to audio output */
	wave_rec_t		voice_rec;		/* wave file writer for voice recording */
	int			voice_recording;	/* 1 = wave file is open and recording */
	int			voice_alert_wait;	/* samples remaining in 2-sec alert pause */
	int			voice_timeout;		/* samples remaining before 2-min timeout */
	char			voice_address[8];	/* address of current voice message */
	char			voice_filename[512];	/* path of current voice WAV file */
	samplerate_t		voice_downsample;	/* resampler: DSP rate -> 8000 Hz */

	/* --- Scheduler --- */
	gsc_msg_t		*priority_list;		/* priority queue head (FIFO order) */
	int			priority_count;		/* priority queue depth */
	int			normal_count;		/* normal queue depth (msg_list) */

	/* --- Battery saver round-robin --- */
	int			sched_current_group;	/* next preamble group to transmit (0-9) */

	/* --- TX batch tracking (for completion logging) --- */
	int			tx_msg_count;		/* messages in current TX batch */
	int			tx_preamble_index;	/* preamble index of current TX batch */

	/* --- Batch mode --- */
	int			batching_mode;		/* 0=off, 1=normal, 2=extended */

	/* --- Hold-off timer --- */
	int			holdoff_ms;		/* configurable hold-off period (default 100) */
	struct timespec		holdoff_start;		/* when hold-off started */
	int			holdoff_active;		/* 1 = timer running */

	/* --- Message expiry --- */
	int			expiry_seconds;		/* configurable expiry timeout (default 300) */

	/* --- RX batch mode disambiguation --- */
	int			rx_batch_candidate;	/* 1 = inverted preamble seen, awaiting start code check */
	int			rx_batch_mode;		/* 1 = confirmed batch mode for current reception */

	/* --- Protocol dump --- */
	int			protocol_dump;		/* 1 = dump TX bitstream to log */

	/* --- Non-battery-saver mode (§2.5) --- */
	int			nbs;			/* 1 = use 75 Hz preamble, no start code */

	/* --- NBS RX state --- */
	int			rx_nbs_count;		/* consecutive 1,1,0,0 pattern matches */
	uint8_t			rx_nbs_shift[4];	/* last 4 bits for pattern detection */
	int			rx_nbs_shift_count;	/* bits accumulated in NBS shift register */
	int			rx_nbs_locked;		/* 1 = NBS preamble detected, buffering data */

	/* --- Scan mode --- */
	uint32_t		scan_from;		/* current scan position (6-digit base) */
	uint32_t		scan_to;		/* scan end (exclusive) */
	enum gsc_msg_type	scan_type;		/* message type for scan */
} gsc_t;

int golay_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, double deviation, double polarity, int tx, int rx, int auto_polarity, const char *message, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback, const char *voice_dir, int voice_monitor);
void golay_destroy(sender_t *sender);

void init_golay(void);
void init_bch(void);

int8_t get_bit(gsc_t *gsc);
void golay_msg_send(const char *buffer);
void golay_msg_receive(const gsc_rx_msg_t *msg);
void golay_scan_enqueue(gsc_t *gsc, uint32_t *scan_from, uint32_t scan_to, enum gsc_msg_type type, int batch_size);

/* validator API */
int golay_validate_msg(const char *address, enum gsc_msg_type type,
                       const char *data);

/* scheduler API */
void scheduler_enqueue(gsc_t *gsc, gsc_msg_t *msg);
void scheduler_expire(gsc_t *gsc);
gsc_msg_t *scheduler_next_batch(gsc_t *gsc);
void scheduler_dump(gsc_t *gsc);

/* decoder API */
int decode_golay(uint32_t codeword, uint16_t *data);
int decode_bch(uint16_t codeword, uint8_t *data);
int decode_batch(gsc_t *gsc, gsc_rx_msg_t *msg, int force);
int decode_nbs(gsc_t *gsc, gsc_rx_msg_t *msg, int force);
int reverse_word1(uint16_t w1, int *g1, int *g0);
int reverse_word2(uint16_t w2, int g1g0, int *a2, int *a1, int *a0);
char decode_alpha(uint8_t code);
char decode_numeric(uint8_t code, int *shifted);
int gsc_score_alpha(const char *str, int len, int fill);
int gsc_score_numeric(const uint8_t *nibbles, int count, int fill);
void gsc_discriminate(gsc_rx_msg_t *msg);

extern const uint32_t activation_code;

