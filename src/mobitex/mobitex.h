#ifndef MOBITEX_H
#define MOBITEX_H

#include "../libmobile/sender.h"

/* ---- Protocol Constants ---- */

/* Bit sync patterns (Mobitex-8000: 16-bit) */
#define MOBITEX_BITSYNC_BASE    0xCCCC  /* Base station -> mobile */
#define MOBITEX_BITSYNC_MOBILE  0x3333  /* Mobile -> base station */

/* Mobitex-1200 specific constants */
#define MOBITEX_1200_BITSYNC_BASE   0xAA    /* 8-bit base station bitsync */
#define MOBITEX_1200_BITSYNC_MOBILE 0x55    /* 8-bit mobile station bitsync */
#define MOBITEX_1200_CRC_POLY       0x3B9FB5 /* 21-bit CRC polynomial */
#define MOBITEX_1200_BLOCK_BYTES    6       /* Data block size for 1200 baud */
#define MOBITEX_1200_FRSYNC_BITS    10      /* Frame sync bits */
#define MOBITEX_1200_BASEID_BITS    10      /* Base ID bits */

/* FEC (12,8) Hamming matrix constants */
#define MOBITEX_FEC_H3  0xEC
#define MOBITEX_FEC_H2  0xD3
#define MOBITEX_FEC_H1  0xBA
#define MOBITEX_FEC_H0  0x75

/* CRC-16 parameters */
#define MOBITEX_CRC_POLY    0x0811
#define MOBITEX_CRC_SR_INIT 0xF8E7
#define MOBITEX_CRC_CR_INIT 0x2A5D

/* Scrambler parameters */
#define MOBITEX_SCRAMBLER_INIT  0x1E  /* 9-bit LFSR initial value */
#define MOBITEX_SCRAMBLER_TAP1  4     /* Bit position 5 (0-indexed = 4) */
#define MOBITEX_SCRAMBLER_TAP2  8     /* Bit position 9 (0-indexed = 8) */

/* Data block dimensions */
#define MOBITEX_BLOCK_BYTES     20    /* Bytes per block (after FEC) */
#define MOBITEX_BLOCK_BITS      240   /* Bits per block (20 x 12) */
#define MOBITEX_DATA_BYTES      18    /* User data bytes per block */
#define MOBITEX_CRC_BYTES       2     /* CRC bytes per block */
#define MOBITEX_FEC_BITS        12    /* Bits per FEC codeword */
#define MOBITEX_MAX_BLOCKS      20    /* Max data blocks per frame */
#define MOBITEX_MAX_FEC_ERRORS  10    /* Max FEC errors before block discard */

/* Frame header dimensions */
#define MOBITEX_HEADER_BITS     24    /* 2 control bytes x 12 FEC bits */
#define MOBITEX_BITSYNC_BITS    16
#define MOBITEX_FRSYNC_BITS     16

/* ---- Enums ---- */

/* Mobitex frame types (matching PDW / original numbering) */
enum mobitex_frametype {
	MOBITEX_FT_MRM  = 0x01,  /* Message data */
	MOBITEX_FT_ACK  = 0x02,  /* Acknowledgement */
	MOBITEX_FT_NACK = 0x03,  /* Negative acknowledgement */
	MOBITEX_FT_REB  = 0x04,  /* Retransmission request */
	MOBITEX_FT_RES  = 0x05,  /* Retransmission */
	MOBITEX_FT_ABD  = 0x06,  /* Access request data (Accessbegäran Data) */
	MOBITEX_FT_ABL  = 0x07,  /* Access request alarm (Accessbegäran Larm) */
	MOBITEX_FT_ATD  = 0x08,  /* Access grant data (Accesstillstånd Data) */
	MOBITEX_FT_ATL  = 0x09,  /* Access grant alarm (Accesstillstånd Larm) */
	/* 0x0A, 0x0B: unused */
	MOBITEX_FT_BKD  = 0x0C,  /* Channel change data */
	MOBITEX_FT_BKT  = 0x0D,  /* Channel change voice */
	MOBITEX_FT_FRI  = 0x0E,  /* Free (idle) */
	MOBITEX_FT_SVP  = 0x0F,  /* Sweep */
	MOBITEX_FT_TST  = 0x10,  /* Silent */
	MOBITEX_FT_AKT  = 0x11,  /* Activity query */
	MOBITEX_FT_NAT  = 0x12,  /* Negative access grant voice */
	MOBITEX_FT_BBT  = 0x13,  /* Change base voice */
};

/* Protocol state */
enum mobitex_state {
	MOBITEX_IDLE = 0,       /* Waiting for message to send / next SVP interval */
	MOBITEX_TX_FRAME,       /* Transmitting a queued message frame */
	MOBITEX_TX_SVP,         /* Transmitting periodic SVP sweep frame */
};

/* MPAK classes (MIS octet 8, bits 6-7) */
enum mobitex_mpak_class {
	MPAK_CLASS_PSUBCOM  = 0,
	MPAK_CLASS_1        = 1,
	MPAK_CLASS_2        = 2,
	MPAK_CLASS_DTESERV  = 3,
};

/* MPAK traffic states (MIS octet 7, bits 5-7) */
enum mobitex_traffic_state {
	MPAK_TS_OK          = 0,  /* No problems during switching */
	MPAK_TS_FROM_MAIL   = 1,  /* Message comes from network mailbox */
	MPAK_TS_IN_MAIL     = 2,  /* Placed in network mailbox */
	MPAK_TS_NO_TRANSFER = 3,  /* Addressee cannot be reached */
	MPAK_TS_ILLEGAL     = 4,  /* Could not be switched by network */
	MPAK_TS_CONGEST     = 5,  /* Network congested */
	MPAK_TS_ERROR       = 6,  /* Technical error in network */
};

/* PSUBCOM packet types (MIS octet 8, bits 0-4, class=0) */
enum mobitex_psubcom_type {
	MPAK_PSUB_TEXT      = 1,  /* Text message */
	MPAK_PSUB_DATA      = 2,  /* Data message */
	MPAK_PSUB_STATUS    = 3,  /* Status message */
	MPAK_PSUB_HPDATA    = 4,  /* Data with Higher Protocol ID */
	MPAK_PSUB_EXTPAK    = 1,  /* External packet (EXTERN_F=1) */
};

/* DTESERV packet types (MIS octet 8, bits 0-4, class=3) */
enum mobitex_dteserv_type {
	MPAK_DTE_LOGINREQ   = 1,
	MPAK_DTE_LOGINGRA   = 2,
	MPAK_DTE_LOGINREF   = 3,
	MPAK_DTE_LOGOUT     = 4,
	MPAK_DTE_LOGOUTORD  = 5,
	MPAK_DTE_BORN       = 6,
	MPAK_DTE_ACTIVE     = 7,
	MPAK_DTE_INACTIVE   = 8,
	MPAK_DTE_DIE        = 9,
	MPAK_DTE_LIVE       = 10,
	MPAK_DTE_ROAMORD    = 11,
	MPAK_DTE_ROAM       = 12,
	MPAK_DTE_GROUPLIST  = 15,
	MPAK_DTE_FLEXREQ    = 16,
	MPAK_DTE_FLEXLIST   = 17,
	MPAK_DTE_INFOREQ    = 18,
	MPAK_DTE_INFO       = 19,
	MPAK_DTE_TIME       = 20,
	MPAK_DTE_AREALIST   = 21,
	MPAK_DTE_ESNREQ     = 22,
	MPAK_DTE_ESNINFO    = 23,
	MPAK_DTE_MODE       = 24,
	MPAK_DTE_APPOPTS    = 30,
	MPAK_DTE_LOWPOWER   = 31,
};

/* ---- Structs ---- */

/* Frame header (extracted from 24-bit over-the-air header) */
typedef struct mobitex_frame_header {
	uint8_t  base_id;       /* 6-bit Base ID */
	uint8_t  area_id;       /* 6-bit Area ID */
	uint8_t  cflags;        /* 4-bit Control Flags */
	uint8_t  parity;        /* 8-bit Parity */
} mobitex_frame_header_t;

/* Link control (bytes 0-5 of primary data block) */
typedef struct mobitex_link_control {
	uint32_t dest_man;      /* 24-bit MAN (bytes 0-2): destination for data, source for SVP */
	uint8_t  frame_id;      /* 5-bit Frame ID (byte 3 bits 0-4) */
	uint8_t  seq_num;       /* 4-bit Sequence Number (byte 4 bits 0-3) */
	uint8_t  block_length;  /* 8-bit Block Length (byte 5) */
	uint8_t  bytes_last;    /* 5-bit Bytes in Last Block */
} mobitex_link_control_t;

/* MPAK header (bytes 6-17 of primary data block) */
typedef struct mobitex_mpak_header {
	uint32_t sender_man;    /* 24-bit Sender MAN (bytes 6-8) */
	uint32_t dest_man;      /* 24-bit Destination MAN (bytes 9-11) */
	uint8_t  traffic_state; /* 3-bit Traffic State (octet 7, bits 5-7) */
	uint8_t  mailbox_f;     /* MAILBOX_F (octet 7, bit 0) */
	uint8_t  posack_f;      /* POSACK_F (octet 7, bit 1) */
	uint8_t  sendlist_f;    /* SENDLIST_F (octet 7, bit 2) = address_list */
	uint8_t  unknown_f;     /* UNKNOWN_F (octet 7, bit 3) */
	uint8_t  reserve_f;     /* Reserve flag (octet 7, bit 4) */
	uint8_t  extern_f;      /* EXTERN_F (octet 8, bit 5) */
	uint8_t  mpak_type;     /* 5-bit MPAK Type (octet 8, bits 0-4) */
	uint8_t  mpak_class;    /* 2-bit MPAK Class (octet 8, bits 6-7) */
	uint8_t  address_list;  /* Alias for sendlist_f (backward compat) */
	uint8_t  hpid;          /* 8-bit Higher Protocol ID (byte 17) */
} mobitex_mpak_header_t;

/* Sweep frame info (SVP parameters from PA01/PA07 / MIS spec)
 *
 * SVP1/SVP3 carry FRI + link + roaming parameters in bytes 7-17:
 *   byte 7:  MAX_REP        byte 13: RAND_SLOTS
 *   byte 8:  SLOT_LENGTH    byte 14: MAX_SPEECH
 *   byte 9:  FREE_SLOTS     byte 15: MAX_ACCESS
 *   byte 10: RSSI_PERIOD    byte 16: TXPOW
 *   byte 11: TIMEOUT        byte 17: (reserved/prio)
 *   byte 12: (reserved)
 *   Bytes 18+ contain neighbour/slave channel list (2-byte BE per channel).
 *   Block 1+ also carries roaming parameters (MIS <SVP1> source):
 *     RSSI_PROC, SCAN_TIME, BAD_BASE, GOOD_BASE, BETTER_BASE
 *
 * SVP6 carries battery-saving parameters (PA07 command, MIS 24:104):
 *   byte 7:  CYCLE_TIME           byte 10: EVAL_CURRENT
 *   byte 8:  TIME_TO_NEXT         byte 11: EVAL_OTHERS
 *   byte 9:  TRANSACTION_TIME     byte 12: (MAN count)
 *   byte 13 upper nibble: DEEP_SLEEP_INHIBIT
 *
 * SVP2/SVP4 carry channel change info:
 *   byte 7:  direction (1=uplink, 2=downlink)
 *   bytes 10-11: channel number (16-bit big-endian)
 *
 * Byte layout reverse-engineered from Palm VIIx ROM MobitexProcessSweepFrame
 * and PA01/PA07 response builders, cross-referenced with MIS Open spec.
 */
typedef struct mobitex_sweep {
	uint8_t  type;          /* Sweep type (0-6) */
	uint8_t  fbi;           /* Frequency Band Identification */
	uint8_t  channels;      /* Number of neighbour channels */
	int      has_channel_list;

	/* SVP1/SVP3 parameters (FRI + link) */
	uint8_t  max_rep;       /* Max retransmissions before roaming eval */
	uint8_t  slot_length;   /* FRI slot length (units of 32/bitrate) */
	uint8_t  free_slots;    /* Total FRI slots */
	uint8_t  rand_slots;    /* Random FRI slots */
	uint8_t  max_access;    /* Max blocks in MRM without access request */
	uint8_t  max_speech;    /* Max blocks in line connection without access */
	uint8_t  timeout;       /* Access timeout in seconds */
	uint8_t  txpow;         /* TX power reduction (0-255 dB below nominal) */

	/* SVP1/SVP3 roaming parameters (MIS <SVP1> source, PA01 command) */
	uint8_t  rssi_proc;     /* RSSI measurement method: 0=FRAME, 1=Continuous */
	uint8_t  rssi_period;   /* Roaming algorithm time (0-255 × 20ms) */
	uint8_t  scan_time;     /* Scan time for surrounding channels (0-255 × 100ms) */
	uint8_t  bad_base;      /* Lowest usable signal from current base (dBµV) */
	uint8_t  good_base;     /* Acceptable signal threshold (dBµV) */
	uint8_t  better_base;   /* Signal improvement threshold for roaming (dB) */

	/* SVP6 parameters (battery-saving, PA07 command) */
	uint8_t  cycle_time;        /* Time between operated states (0-255 × 250ms) */
	uint8_t  time_to_next;      /* Time from SVP6 to next operating state (0-255 × 250ms) */
	uint8_t  transaction_time;  /* Time in operating state after ACK (0-255 × 250ms) */
	uint8_t  eval_current;      /* Time to evaluate current channel (0-255 sec) */
	uint8_t  eval_others;       /* Time to evaluate other channels (0-255 RSSI periods) */
	uint8_t  deep_sleep_inhibit; /* Deep sleep inhibit count (0-15 SVP6 frames) */

	/* SVP2/SVP4 channel info */
	uint16_t upfreq;        /* Uplink channel number */
	uint16_t dofreq;        /* Downlink channel number */
} mobitex_sweep_t;

/* Outgoing message */
typedef struct mobitex_msg {
	struct mobitex_msg *next;
	struct mobitex     *mobitex;
	int                callref;
	uint32_t           dest_man;
	uint32_t           sender_man;
	uint8_t            frame_type;
	uint8_t            mpak_type;
	uint8_t            mpak_class;
	char               data[5000];
	int                data_length;
} mobitex_msg_t;

/* Top-level transceiver instance */
typedef struct mobitex {
	sender_t            sender;             /* MUST be first member (for casting) */

	/* configuration */
	int                 tx;                 /* TX enabled */
	int                 rx;                 /* RX enabled */
	int                 scrambling;         /* Bit scrambling enabled */
	int                 ramnet;             /* RAMnet CFlags checking */
	int                 baudrate;           /* 8000 or 1200 */
	uint16_t            bitsync;            /* Expected bit sync (0xCCCC or 0x3333) */
	int                 frsync_index;       /* Expected frame sync index (-1=any) */

	/* protocol state */
	enum mobitex_state  state;
	mobitex_frame_header_t frame_header;
	mobitex_link_control_t link_control;
	mobitex_mpak_header_t  mpak_header;
	mobitex_sweep_t     sweep;

	/* RX state */
	int                 rx_frsync_index;    /* Frame sync index detected on RX (-1=none) */
	int                 rx_block_number;    /* Current block being received */
	int                 rx_bit_count;       /* Bits remaining in current frame */
	uint8_t             rx_block_buf[240];  /* Raw bits for current data block */
	int                 rx_block_pos;       /* Position in block buffer */
	int                 rx_data_blocks[20][20]; /* Decoded blocks */
	char                rx_msg_buf[5000];   /* Assembled message text */
	int                 rx_msg_len;         /* Message length */
	int                 rx_fec_errors;      /* FEC error count for current block */
	int                 rx_fec_errors_total; /* Total FEC errors across all blocks in frame */

	/* TX state */
	enum mobitex_state  tx_state;
	mobitex_msg_t      *current_msg;
	mobitex_msg_t      *msg_list;

	/* Default TX: periodic SVP sweep when idle */
	uint8_t             tx_base_id;         /* 6-bit Base ID for TX frame headers */
	uint8_t             tx_area_id;         /* 6-bit Area ID for TX frame headers */
	uint32_t            tx_base_man;        /* 24-bit Base station MAN for link control */
	uint8_t             tx_svp_type;        /* Sweep type for next TX (rotated automatically) */
	uint8_t             tx_fbi;             /* FBI for TX SVP frames */
	double              tx_svp_interval;    /* SVP interval in seconds (default 1.0) */
	double              tx_svp_timer;       /* Countdown to next SVP transmission */
	int                 tx_svp_auto;        /* Auto-transmit SVP when idle */
	int                 tx_svp_seq;         /* Rotation counter for SVP type cycling */

	/* TX SVP1/SVP3 parameters (roaming + link, broadcast to mobiles) */
	uint8_t             tx_txpow;           /* TX power reduction for mobiles (dB) */
	uint8_t             tx_rssi_proc;       /* RSSI method: 0=FRAME, 1=Continuous */
	uint8_t             tx_rssi_period;     /* Roaming time (×20ms) */
	uint8_t             tx_scan_time;       /* Scan time (×100ms) */
	uint8_t             tx_bad_base;        /* Bad base threshold (dBµV) */
	uint8_t             tx_good_base;       /* Good base threshold (dBµV) */
	uint8_t             tx_better_base;     /* Better base threshold (dB) */
	uint16_t            tx_upfreq;          /* Uplink channel number */
	uint16_t            tx_dofreq;          /* Downlink channel number */

	/* TX SVP6 parameters (battery-saving, PA07 command) */
	uint8_t             tx_cycle_time;          /* Time between operated states (×250ms) */
	uint8_t             tx_time_to_next;        /* Time from SVP6 to operating state (×250ms) */
	uint8_t             tx_transaction_time;    /* Time in operating state after ACK (×250ms) */
	uint8_t             tx_eval_current;        /* Evaluate current channel (sec) */
	uint8_t             tx_eval_others;         /* Evaluate other channels (RSSI periods) */
	uint8_t             tx_deep_sleep_inhibit;  /* Deep sleep inhibit (SVP6 frames) */

	/* TX FRI parameters (base station free cycle) */
	uint8_t             tx_slot_length;     /* FRI slot length */
	uint8_t             tx_free_slots;      /* Total FRI slots */
	uint8_t             tx_rand_slots;      /* Random FRI slots */
	uint8_t             tx_max_rep;         /* Max retransmissions */
	uint8_t             tx_max_access;      /* Max blocks without access request */
	uint8_t             tx_max_speech;      /* Max blocks for line connection */
	uint8_t             tx_timeout;         /* Access timeout (seconds) */
	uint8_t             tx_prio;            /* Priority level */
	double              tx_fri_interval;    /* FRI interval in seconds (unused, kept for CLI compat) */
	double              tx_fri_timer;       /* (unused, FRI now follows SVP directly) */
	int                 tx_fri_pending;     /* 1 = FRI should follow the SVP just sent */

	/* Channel access (Slotted Aloha) retry state */
	int                 tx_access_retries;      /* Current retry count */
	int                 tx_access_max_retries;  /* Max retries (default 3) */
	double              tx_access_backoff;      /* Current backoff timer (seconds) */

	/* DSP state */
	double              fsk_deviation;
	double              fsk_polarity;
	sample_t            fsk_ramp_up[256];
	sample_t            fsk_ramp_down[256];
	double              fsk_bitduration;
	double              fsk_bitstep;
	sample_t           *fsk_tx_buffer;
	int                 fsk_tx_buffer_size;
	int                 fsk_tx_buffer_length;
	int                 fsk_tx_buffer_pos;
	double              fsk_tx_phase;
	uint8_t             fsk_tx_lastbit;
	double              fsk_rx_phase;
	uint8_t             fsk_rx_lastbit;
	uint32_t            fsk_rx_word;        /* 32-bit shift register for sync detection */
	int                 fsk_rx_sync;        /* Nonzero when synced to data blocks */
	int                 fsk_rx_index;       /* Bit index within current block */
	int                 samplerate;

	/* Scrambler state (separate for TX and RX to avoid loopback corruption) */
	uint16_t            scrambler_tx_sr;    /* 9-bit LFSR state for TX */
	uint16_t            scrambler_rx_sr;    /* 9-bit LFSR state for RX */
} mobitex_t;

/* ---- Public API (mobitex.c) ---- */
int  mobitex_create(const char *kanal, double rx_frequency, double tx_frequency,
                    const char *device, int use_sdr, int samplerate,
                    double rx_gain, double tx_gain,
                    int tx, int rx, int baudrate,
                    double deviation, double polarity,
                    int scrambling, int ramnet,
                    uint16_t bitsync, int frsync_index,
                    int base_id, int area_id, uint32_t base_man,
                    double svp_interval, int svp_type, int fbi,
                    uint8_t txpow, uint8_t rssi_proc, uint8_t rssi_period,
                    uint8_t scan_time, uint8_t bad_base, uint8_t good_base,
                    uint8_t better_base, uint16_t upfreq, uint16_t dofreq,
                    uint8_t cycle_time, uint8_t time_to_next,
                    uint8_t transaction_time, uint8_t eval_current,
                    uint8_t eval_others, uint8_t deep_sleep_inhibit,
                    uint8_t slot_length, uint8_t free_slots, uint8_t rand_slots,
                    uint8_t max_rep, uint8_t max_access, uint8_t timeout,
                    double fri_interval,
                    const char *write_rx_wave, const char *write_tx_wave,
                    const char *read_rx_wave, const char *read_tx_wave,
                    int loopback);
void mobitex_destroy(sender_t *sender);
void mobitex_msg_send(const char *text, int length);
void mobitex_msg_receive(const char *channel, const char *base_id,
                         uint8_t frame_type, uint32_t dest_man,
                         uint32_t sender_man, const char *message);

void mobitex_rx_frame_complete(mobitex_t *mobitex);
void mobitex_tx_idle_tick(mobitex_t *mobitex, int samples);
void mobitex_tx_filler(mobitex_t *mobitex);
int  mobitex_is_mobile_mode(mobitex_t *mobitex);

int msg_receive(const char *text);

#endif /* MOBITEX_H */
