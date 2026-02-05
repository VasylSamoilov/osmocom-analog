#include "../libgoertzel/goertzel.h"
#include "../libmobile/sender.h"
#include <osmocom/core/timer.h>
#include "../libcompandor/compandor.h"
typedef struct amps amps_t;
#include "sysinfo.h"
#include "transaction.h"

enum dsp_mode {
	DSP_MODE_OFF,			/* channel not active (VC) */
	DSP_MODE_AUDIO_RX_AUDIO_TX,	/* stream audio */
	DSP_MODE_AUDIO_RX_FRAME_TX,	/* stream audio, send frames */
	DSP_MODE_AUDIO_RX_SILENCE_TX,	/* stream audio, send silence */
	DSP_MODE_FRAME_RX_FRAME_TX,	/* send and decode frames */
};

enum amps_chan_type {
	CHAN_TYPE_CC,		/* control channel */
	CHAN_TYPE_PC,		/* paging channel */
	CHAN_TYPE_CC_PC,	/* combined CC + PC */
	CHAN_TYPE_VC,		/* voice channel */
	CHAN_TYPE_CC_PC_VC,	/* combined CC + PC + TC */
};

enum amps_state {
	STATE_NULL,		/* power off state */
	STATE_IDLE,		/* channel is not in use */
	STATE_BUSY,		/* channel busy (call) */
};

enum fsk_rx_sync {
	FSK_SYNC_NONE,		/* we are not in sync and wait for valid dotting sequence */
	FSK_SYNC_DOTTING,	/* we received a valid dotting sequence and check for sync sequence */
	FSK_SYNC_POSITIVE,	/* we have valid sync and read all the bits of the frame */
	FSK_SYNC_NEGATIVE,	/* as above, but negative sync (high frequency deviation detected as low signal)  */
};

/* SAT (Supervisory Audio Tone) state machine states per TIA/EIA-553-A */
enum sat_state {
	SAT_STATE_NONE,		/* No valid SAT detected */
	SAT_STATE_5970,		/* SAT at 5970 Hz detected (SCC 0) */
	SAT_STATE_6000,		/* SAT at 6000 Hz detected (SCC 1) */
	SAT_STATE_6030,		/* SAT at 6030 Hz detected (SCC 2) */
};

/* SAT frequency classification result */
enum sat_freq_class {
	SAT_FREQ_INVALID,	/* Frequency outside valid windows or below threshold */
	SAT_FREQ_5970,		/* 5955-5985 Hz window (SCC 0) */
	SAT_FREQ_6000,		/* 5985-6015 Hz window (SCC 1) */
	SAT_FREQ_6030,		/* 6015-6045 Hz window (SCC 2) */
};

#define FSK_MAX_BITS		1032	/* maximum number of bits to process (FVC with dotting+sync) */

struct amps {
	sender_t		sender;
	compandor_t		cstate;
	int			pre_emphasis;		/* use pre_emphasis by this instance */
	int			de_emphasis;		/* use de_emphasis by this instance */
	emphasis_t		estate;
	emphasis_t		estate_rx;
	emphasis_fast_t		estate_tx_fast;		/* TX pre-emphasis using correct shelf filter */
	iir_filter_t		tx_post_filter;
	iir_filter_t		rx_pre_filter;
	iir_filter_t		rx_notch_filter;
	iir_filter_t		rx_hpf;
	iir_filter_t		rx_voice_hpf;		/* voice band HPF (300 Hz) at 8kHz for expander */
	iir_filter_t		rx_voice_lpf;		/* voice band LPF (3400 Hz) at 8kHz for expander */

	/* sender's states */
	enum amps_chan_type	chan_type;
	enum amps_state		state;
	int			channel_busy;		/* indicate channel is busy while receiving */

	/* display measurements */
	dispmeasparam_t		*dmp_frame_level;
	dispmeasparam_t		*dmp_frame_quality;
	dispmeasparam_t		*dmp_sat_level;
	dispmeasparam_t		*dmp_sat_quality;

	/* system info */
	amps_si			si;
	int			send_callerid;		/* if set, caller ID is transmitted */

	/* cell nr selection */
	int			cell_auto;		/* if set, cell_nr is selected automatically */

	/* dsp states */
	enum dsp_mode		dsp_mode;		/* current mode: audio, durable tone 0 or 1, paging */
	int			flip_polarity;		/* 1 = flip */
	double			fsk_deviation;		/* deviation of FSK signal on sound card */
	sample_t		fsk_ramp_up[256];	/* samples of upward ramp shape */
	sample_t		fsk_ramp_down[256];	/* samples of downward ramp shape */
	double			fsk_bitduration;	/* duration of one bit in samples */
	double			fsk_bitstep;		/* fraction of one bit each sample */
	/* tx bits generation */
	char			fsk_tx_frame[FSK_MAX_BITS + 1];	/* +1 because 0-termination */
	int			fsk_tx_frame_pos;	/* current position sending bits */
	sample_t		*fsk_tx_buffer;		/* tx buffer for one data block */
	int			fsk_tx_buffer_size;	/* size of tx buffer (in samples) */
	int			fsk_tx_buffer_length;	/* usage of buffer (in samples) */
	int			fsk_tx_buffer_pos;	/* current position sending buffer */
	double			fsk_tx_phase;		/* current bit position */
	char			fsk_tx_last_bit;	/* save last bit of frame (for next frame's ramp) */
	/* high-pass filter to remove DC offset from RX signal */
	double			highpass_factor;	/* high pass filter factor */
	double			highpass_x_last;	/* last input value */
	double			highpass_y_last;	/* last output value */
	/* rx detection of bits and sync */
	sample_t		fsk_rx_last_sample;	/* last sample (for level change detection) */
	double			fsk_rx_elapsed;		/* bit duration since last level change */
	enum fsk_rx_sync	fsk_rx_sync;		/* sync state */
	uint16_t		fsk_rx_sync_register;	/* shift register to detect sync word */
	int			fsk_rx_sync_tolerant;	/* be more tolerant to sync */
	/* the dotting buffer stores the elapsed samples, so we can calculate
	 * an average time of zero-crossings during dotting sequence.
	 * this buffer wrpps every 256 values */
	double			fsk_rx_dotting_elapsed[256]; /* dotting buffer with elapsed samples since last zero-crossing */
	uint8_t			fsk_rx_dotting_pos;	/* position of next value in dotting buffer */
	int			fsk_rx_dotting_life;	/* counter to expire when no sync was found after dotting */
	double			fsk_rx_dotting_average;	/* last average slope position of dotting sequnece. */
	/* the ex buffer holds the duration of one bit, and wraps every
	 * bit. */
	double			fsk_rx_bitcount;	/* counts the bit. if it reaches or exceeds 1, the bit is complete and the next bit starts */
	sample_t		*fsk_rx_window;		/* rx buffer for one bit */
	int			fsk_rx_window_length;	/* length of rx buffer */
	int			fsk_rx_window_half;	/* half of length of rx buffer */
	int			fsk_rx_window_begin;	/* where to begin detecting level */
	int			fsk_rx_window_end;	/* where to end detecting level */
	int			fsk_rx_window_pos;	/* current position in buffer */
	/* the rx buffer received one frame until rx length */
	char			fsk_rx_frame[FSK_MAX_BITS + 1];	/* +1 because 0-termination */
	int			fsk_rx_frame_length;	/* length of expected frame */
	int			fsk_rx_frame_count;	/* count number of received bit */
	double			fsk_rx_frame_level;	/* sum of level of all bits */
	double			fsk_rx_frame_quality;	/* sum of quality of all bits */
	/* RECC frame states */
	int			rx_recc_nawc;		/* counts down received words */
	int			rx_recc_word_count;	/* counts up received words */
	uint8_t			rx_recc_t;		/* T field: 1=Origination, 0=Paging Response */
	uint32_t		rx_recc_min1;		/* mobile id */
	uint16_t		rx_recc_min2;
	uint8_t			rx_recc_msg_type;	/* message (3 values) */
	uint8_t			rx_recc_ordq;
	uint8_t			rx_recc_order;
	uint32_t		rx_recc_esn;
	uint32_t		rx_recc_scm;
	uint8_t			rx_recc_mpci;
	uint8_t			rx_recc_mspc;		/* from Word C - Protocol Capability */
	uint8_t			rx_recc_mscap;		/* from Word C - Protocol Capability */
	char			rx_recc_dialing[33];	/* received dial string */
	int			rx_rvc_esn_pending;	/* waiting for ESN word (Order 15 response) */
	/* FOCC frame states */
	int			rx_focc_word_count;	/* counts received words */
	int			tx_focc_frame_count;	/* used to schedule system information */
	int			tx_focc_send;		/* if set, send message words */
	uint32_t		tx_focc_min1;		/* mobile id */
	uint16_t		tx_focc_min2;
	int			tx_focc_chan;		/* channel to assign for voice call */
	uint8_t			tx_focc_msg_type;	/* message (3 values) */
	uint8_t			tx_focc_ordq;
	uint8_t			tx_focc_order;
	int			tx_focc_word_count;	/* counts transmitted words in a multi word message */
	int			tx_focc_word_repeat;	/* counts repeats of multi word message */
	int			tx_focc_debugged;	/* indicator to prevent debugging all SI/filler frames */
	/* Directed Retry fields for FOCC */
	int			tx_focc_retry_channels[6];	/* channel positions for Directed Retry */
	int			tx_focc_retry_num_channels;	/* number of channels */
	int			tx_focc_num_words;	/* total words in message (2 for normal, 4 for Directed Retry) */
	/* FVC frame states */
	int			tx_fvc_send;		/* if set, send message words */
	int			tx_fvc_chan;		/* channel to assign for voice call */
	int			tx_fvc_scc;		/* target SAT color code for handoff (-1 = use current) */
	uint8_t			tx_fvc_msg_type;	/* message (3 values) */
	uint8_t			tx_fvc_ordq;
	uint8_t			tx_fvc_order;
	char			tx_fvc_callerid[34];	/* caller ID */
	int			tx_fvc_callerid_present;/* presentation of caller ID */
	int			tx_fvc_callerid_screen;	/* screening of caller ID */
	int			tx_fvc_callerid_signal;	/* signal to send in conjunction with caller ID */
	int			tx_fvc_word_count;	/* counts transmitted words in a muli word message */
	int			tx_fvc_word_repeat;	/* counts repeats of mulit word message */
	/* Flash With Info fields */
	char			tx_fvc_flashinfo[34];	/* Flash With Info message */
	int			tx_fvc_flashinfo_pi;	/* presentation indicator */
	int			tx_fvc_flashinfo_si;	/* screening indicator */
	/* CRI/TCI fields for Alert/Flash With Info ORDQ=1,2 */
	uint8_t			tx_fvc_cri[32];		/* CRI data: 8 elements x 4 BCD digits */
	int			tx_fvc_cri_elements;	/* number of CRI elements (1-8) */
	uint8_t			tx_fvc_tci[16];		/* TCI data: 4 rows x 4 BCD digits */
	int			tx_fvc_tci_rows;	/* number of TCI rows (1-4) */
	/* SAT tone */
	int			sat;			/* use SAT tone 0..2 */
	int			sat_samples;		/* number of samples in buffer for supervisory detection */
	goertzel_t		sat_goertzel[5];	/* filter for SAT signal decoding */
	sample_t		*sat_filter_spl;	/* array with sample buffer for supervisory detection */
	int			sat_filter_pos;		/* current sample position in filter_spl */
	double			sat_phaseshift65536[3];	/* how much the phase of sine wave changes per sample */
	double			sat_phase65536;		/* current phase */
	int			sat_print;		/* counts when to print result */
	int			dtx_state;		/* 1 = high (fast sat detection) */
	int			sat_detected;		/* current detection state flag (delayed detection) */
	int			sat_detect_count;	/* current number of consecutive detections/losses */
	/* Enhanced SAT state machine per TIA/EIA-553-A */
	enum sat_state		sat_state;		/* current SAT state machine state */
	enum sat_state		sat_pending_state;	/* pending state during persistence window */
	int			sat_state_count;	/* persistence counter for state transitions */
	double			sat_level_db;		/* current SAT level in dB (relative to nominal) */
	enum sat_freq_class	sat_freq_detected;	/* currently detected SAT frequency class */
	double			sat_goertzel_levels[3];	/* levels of all 3 SAT frequencies */
	int			sig_detected;		/* current detection state flag (delayed detection) */
	int			sig_detect_count;	/* current number of consecutive detections/losses */
	/* Fast ST detection for handoff (smaller window, more frequent checks) */
	goertzel_t		fast_st_goertzel[2];	/* ST and noise reference for fast detection */
	sample_t		*fast_st_buffer;	/* buffer for fast ST detection */
	int			fast_st_samples;	/* number of samples in fast ST buffer (~20ms) */
	int			fast_st_pos;		/* current position in fast ST buffer */
	int			fast_st_detected;	/* fast ST detection flag */
	int			fast_st_count;		/* consecutive fast ST detections */

	transaction_t		*trans_list;		/* list of transactions */

	/* delay measurement in loopback mode */
	double			when_received;		/* time stamp of received frame start (start of dotting) */
	double			when_transmitted[16];	/* time stamps of filler frames with different count */
	int			when_count;		/* counter of the filler frame */
};

void amps_display_status(void);
void amps_channel_list(void);
int amps_channel_by_short_name(const char *short_name);
const char *chan_type_short_name(enum amps_chan_type chan_type);
const char *chan_type_long_name(enum amps_chan_type chan_type);
double amps_channel2freq(int channel, int uplink);
enum amps_chan_type amps_channel2type(int channel);
const char *amps_channel2band(int channel);
const char *amps_min22number(uint16_t min2);
const char *amps_min12number(uint32_t min1);
int amps_number2min(const char *number, uint32_t *min1, uint16_t *min2);
const char *amps_min2number(uint32_t min1, uint16_t min2);
void amps_encode_esn(uint32_t *esn, uint8_t mfr, uint32_t serial);
void amps_decode_esn(uint32_t esn, uint8_t *mfr, uint32_t *serial);
const char *amps_scm(uint8_t scm);
const char *amps_power_level_name(int level);
int amps_create(const char *kanal, enum amps_chan_type chan_type, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int pre_emphasis, int de_emphasis, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, amps_si *si, uint16_t sid, uint8_t sat, int polarity, int send_callerid, int tolerant, int loopback);
void amps_destroy(sender_t *sender);
void amps_go_idle(amps_t *amps);
void amps_rx_signaling_tone(amps_t *amps, int tone, double quality);
void amps_rx_sat(amps_t *amps, int tone, double quality);
void amps_rx_sat_mismatch(amps_t *amps, enum sat_state expected, enum sat_state detected);
const char *sat_state_name(enum sat_state state);
void amps_rx_recc(amps_t *amps, uint8_t t, uint8_t scm, uint8_t mpci, uint32_t esn, uint32_t min1, uint16_t min2, uint8_t msg_type, uint8_t ordq, uint8_t order, const char *dialing, uint8_t mspc, uint8_t mscap);
transaction_t *amps_tx_frame_focc(amps_t *amps);
transaction_t *amps_tx_frame_fvc(amps_t *amps);
void amps_display_status();
int amps_flash_with_info(const char *number, const char *message, int pi, int si);
int amps_flash_with_cri(const char *number, const char *cri_data);
int amps_flash_with_tci(const char *number, const char *tci_data);
int amps_alert_with_cri(const char *number, const char *cri_data);
int amps_alert_with_tci(const char *number, const char *tci_data);
int amps_pci_query(const char *number);
int amps_audit_order(const char *number);
int amps_alert_order(const char *number);
int amps_abbreviated_alert(const char *number);
int amps_release_order(const char *number);
int amps_reorder(const char *number);
int amps_mwi(const char *number, int count, int type);
int amps_stopalert(const char *number);
int amps_intercept(const char *number);
int amps_send_called_address(const char *number);
int amps_maintenance(const char *number);
int amps_silent_page(const char *number);
int amps_change_power_order(const char *number, int level);
int amps_serial_number_request(const char *number);
int amps_local_control(const char *number, int code);
int amps_disable_dtmf(const char *number);
int amps_handoff(const char *number, int new_channel);
int amps_directed_retry(const char *number, int *channels, int num_channels, int last_try);
int amps_rescan(const char *number);
void amps_rx_pci_report(amps_t *amps, uint8_t mspc, uint8_t mscap);
void amps_rx_release_order(amps_t *amps, uint8_t ordq);
void amps_rx_esn_response(amps_t *amps, uint32_t esn);
