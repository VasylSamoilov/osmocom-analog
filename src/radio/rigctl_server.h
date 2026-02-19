/* Rigctl Server - Hamlib-compatible remote control protocol
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ============================================================
 * OVERVIEW
 * ============================================================
 * Implements a hamlib-compatible rigctld TCP/serial server for remote
 * control of osmoradio. Compatible with Gpredict, fldigi, WSJT-X, GQRX,
 * SDR++, rigctl CLI, and other hamlib clients.
 *
 * Protocol reference: Hamlib rigctld man page (doc/man1/rigctld.1)
 * See also: docs/rds/Hamlib/rigs/dummy/gqrx.c for GQRX client impl
 *
 * ============================================================
 * TX/RX OPERATION
 * ============================================================
 * This server supports simultaneous TX and RX operation:
 * - Commands can operate on TX, RX, or both depending on context
 * - Frequency commands (f/F) operate on the active direction(s)
 * - Mode commands (m/M) can set different modes for TX and RX
 * - Split mode allows different TX and RX frequencies
 *
 * ============================================================
 * TODO: UNIMPLEMENTED FEATURES
 * ============================================================
 * The following features need implementation for full functionality:
 *
 * 1. CW MODES (CW, CWU, CWL, CWR):
 *    - Currently mapped to USB/LSB as placeholders
 *    - Need: BFO offset, CW sidetone, keyer interface
 *    - Implementation: Add CW demodulator/modulator to radio.c
 *
 * 2. AMS MODE (AM Synchronous):
 *    - Currently mapped to AM as placeholder
 *    - Need: PLL-based carrier recovery, selectable sideband
 *    - Implementation: Add sync AM demodulator to radio.c
 *
 * 3. PTT CONTROL:
 *    - Currently returns error (not implemented)
 *    - Need: Callback to switch TX/RX, GPIO control for hardware PTT
 *    - Implementation: Add set_ptt/get_ptt callbacks, wire to SDR
 *
 * 4. FREQUENCY VALIDATION:
 *    - Currently accepts any frequency
 *    - Need: Validate against SDR hardware limits
 *    - Need: Apply upconverter/downconverter offset
 *    - Implementation: Use sdr_caps struct for validation
 *
 * 5. SPLIT MODE:
 *    - Currently returns error
 *    - Need: Separate TX and RX VFOs
 *    - Implementation: Track tx_freq/rx_freq independently
 *
 * 6. HARDWARE GAIN CONTROL:
 *    - Currently returns error for gain queries
 *    - Need: Query SDR for available gains
 *    - Implementation: Add gain enumeration from SoapySDR
 *
 * 7. CTCSS (Continuous Tone-Coded Squelch System):
 *    - Currently returns 0 (no tone) for compatibility
 *    - Need: Sub-audible tone generator (67.0-254.1 Hz)
 *    - Need: Goertzel filter or FFT-based tone detector
 *    - Used for: repeater access, interference rejection
 *    - Implementation: Add to FM modulator/demodulator
 *
 * 8. DCS (Digital Coded Squelch):
 *    - Currently returns 0 (no code) for compatibility
 *    - Need: 23-bit digital code encoder/decoder
 *    - Common codes: 023, 025, 026, 031, 032, etc.
 *    - Used for: repeater access, group calling
 *    - Implementation: Add DCS detector to FM demodulator
 *
 * ============================================================
 * PROTOCOL: Hamlib rigctld (line-based ASCII)
 * ============================================================
 * Default port: 7356 (GQRX default), 4532 (Hamlib default)
 * Line terminator: \n (LF)
 * Reply format: RPRT 0 (success) or RPRT -N (error code)
 *
 * --- FREQUENCY CONTROL ---
 * f              Get frequency [Hz]
 * F <freq>       Set frequency [Hz]
 * i              Get split TX frequency [Hz]
 * I <freq>       Set split TX frequency [Hz]
 *
 * --- MODE CONTROL ---
 * m              Get mode and passband (two lines: mode\nbandwidth)
 * M <mode> [bw]  Set mode and passband [Hz]
 * M ?            List supported modes
 *                Modes: OFF, FM, WFM, WFM_ST, AM, AMS*, USB, LSB, CW*, CWU*, CWL*, CWR*
 *                (* = TODO: currently mapped to similar mode)
 *
 * --- VFO CONTROL ---
 * v              Get current VFO (returns "VFOA")
 * V <vfo>        Set VFO (accepts "VFOA" only)
 * V ?            List available VFOs
 *
 * --- SPLIT/DUPLEX ---
 * s              Get split status (returns "0\nVFOA\n")
 * S <split> <vfo> Set split (TODO: not yet implemented)
 *
 * --- PTT CONTROL ---
 * t              Get PTT status (0=RX, 1=TX)
 * T <ptt>        Set PTT (TODO: not yet implemented)
 *
 * --- LEVEL GET/SET ---
 * l ?            List readable levels
 * l STRENGTH     Signal strength [dBFS]
 * l SQL          Squelch threshold [dBFS]
 * l AF           Audio gain [dB]
 * l TX_RMS       TX RMS level [dBFS] (TX mode)
 * l TX_PEAK      TX peak level [dBFS] (TX mode)
 * l DEVIATION    FM deviation [Hz]
 * l MOD_INDEX    AM modulation index (0.0-1.0)
 * L ?            List writable levels
 * L SQL <val>    Set squelch threshold
 * L AF <val>     Set audio gain
 * L DEVIATION <v> Set FM deviation
 * L MOD_INDEX <v> Set AM modulation index
 *
 * --- FUNCTION GET/SET ---
 * u ?            List readable functions
 * u DSP          DSP running status (0/1)
 * u RECORD       Recording status (0/1)
 * u RDS          RDS decoder/encoder status (0/1)
 * u STEREO       Stereo status (0/1)
 * U DSP <0|1>    Start/stop DSP
 * U RECORD <0|1> Start/stop recording
 * U RDS <0|1>    Enable/disable RDS
 *
 * --- PARAMETER GET (RDS) ---
 * p ?            List parameters
 * p RDS_PI       RDS PI code (hex, e.g. "2201")
 * p RDS_PS_NAME  RDS Program Service name
 * p RDS_RADIOTEXT RDS RadioText message
 *
 * --- DCD (SQUELCH DETECT) ---
 * \get_dcd       Get DCD status (0=closed/no signal, 1=open/signal)
 *
 * --- HAMLIB COMPATIBILITY ---
 * \chk_vfo       VFO check (returns "0" = no VFO mode)
 * \dump_state    Full hamlib state dump for capability discovery
 * \dump_caps     Dump rig capabilities
 * 1              Same as \dump_caps
 * \get_powerstat Power status (returns "1" = on)
 * \set_powerstat Set power (always succeeds)
 * \get_vfo_info  Get VFO info (freq, mode, width, split, satmode)
 * \get_vfo_list  Get available VFOs
 * \get_modes     Get supported modes with bandwidths
 * \get_rig_info  Get rig info in key=value format
 * \set_vfo_opt   Set VFO option (always succeeds)
 * \hamlib_version Get Hamlib version string
 * \halt          Close connection (same as 'q')
 * _              Get version string
 * q / Q          Close connection
 *
 * ============================================================
 * USAGE
 * ============================================================
 * TCP:    --rigctl-server 7356
 *         --rigctl-server 127.0.0.1:7356
 * Serial: --rigctl-server /dev/ttyUSB0,9600,8N1
 *         --rigctl-server /dev/ttyUSB0,115200,8N1,rtscts
 *
 * Access control (optional, default allows all hosts):
 *         --rigctl-allowed-hosts "127.0.0.1,192.168.1.100"
 *
 * ============================================================
 * TESTING WITH RIGCTL CLI
 * ============================================================
 * IMPORTANT: Use model 2 (NET rigctl) to connect to this server!
 * The default model 1 (Dummy) does NOT connect to remote servers.
 *
 * Correct usage:
 *   rigctl -m 2 -r localhost:7356
 *   rigctl -m 2 -r localhost:7356 -vvvv    (verbose for debugging)
 *
 * Common mistake (WRONG - uses Dummy rig, ignores -r):
 *   rigctl -r localhost:7356               (missing -m 2)
 *
 * Available commands in rigctl:
 *   f          - get frequency
 *   F 100e6    - set frequency to 100 MHz
 *   m          - get mode and bandwidth
 *   M FM 15000 - set FM mode with 15kHz bandwidth
 *   l STRENGTH - get signal strength
 *   q          - quit
 *
 * ============================================================
 * CLIENT COMPATIBILITY
 * ============================================================
 * Tested with:
 * - rigctl CLI (hamlib-utils)
 *
 * May work with other Hamlib rigctld clients (Gpredict, fldigi,
 * WSJT-X, etc.) but not tested.
 */

#ifndef RIGCTL_SERVER_H
#define RIGCTL_SERVER_H

#include <stdint.h>

/* Forward declarations */
struct radio;
struct signal_meter;

/* ============================================================
 * Ring Buffer (same as rds_protocol.h)
 * ============================================================ */

#define RIGCTL_RING_SIZE	4096
#define RIGCTL_RING_MASK	(RIGCTL_RING_SIZE - 1)

typedef struct {
	uint8_t		data[RIGCTL_RING_SIZE];
	volatile int	head;
	volatile int	tail;
} rigctl_ring_t;

static inline void rigctl_ring_init(rigctl_ring_t *r) {
	r->head = r->tail = 0;
}

static inline int rigctl_ring_used(const rigctl_ring_t *r) {
	return (r->head - r->tail) & RIGCTL_RING_MASK;
}

static inline int rigctl_ring_free(const rigctl_ring_t *r) {
	return RIGCTL_RING_SIZE - 1 - rigctl_ring_used(r);
}

static inline int rigctl_ring_write(rigctl_ring_t *r, const uint8_t *buf, int len) {
	int free_space = rigctl_ring_free(r);
	int i;
	if (len > free_space)
		len = free_space;
	for (i = 0; i < len; i++) {
		r->data[r->head] = buf[i];
		r->head = (r->head + 1) & RIGCTL_RING_MASK;
	}
	return len;
}

static inline int rigctl_ring_read(rigctl_ring_t *r, uint8_t *buf, int len) {
	int used = rigctl_ring_used(r);
	int i;
	if (len > used)
		len = used;
	for (i = 0; i < len; i++) {
		buf[i] = r->data[r->tail];
		r->tail = (r->tail + 1) & RIGCTL_RING_MASK;
	}
	return len;
}

/* ============================================================
 * Endpoint Configuration
 * ============================================================ */

typedef enum {
	RIGCTL_EP_NONE = 0,
	RIGCTL_EP_TCP,
	RIGCTL_EP_SERIAL
} rigctl_ep_type_t;

typedef enum {
	RIGCTL_FLOW_NONE = 0,
	RIGCTL_FLOW_XONXOFF,
	RIGCTL_FLOW_RTSCTS
} rigctl_flow_t;

#define RIGCTL_EP_IP_LEN	64
#define RIGCTL_EP_DEV_LEN	256

typedef struct {
	rigctl_ep_type_t type;
	char		ip[RIGCTL_EP_IP_LEN];
	int		port;
	char		device[RIGCTL_EP_DEV_LEN];
	int		speed;
	int		bits;
	char		parity;
	int		stopbits;
	rigctl_flow_t	flow;
} rigctl_endpoint_t;

/* ============================================================
 * SDR Capabilities
 * ============================================================
 * Describes the hardware capabilities and configuration of the SDR.
 * Used for frequency validation and upconverter offset handling.
 * Populated from sdr_query_caps() at startup.
 *
 * TODO: Store computed frequency ranges from SDR hardware at startup
 *       and on retune for both SoapySDR and UHD backends:
 *       - Query actual hardware limits via SoapySDRDevice_getFrequencyRange()
 *         and uhd_usrp_get_rx_freq_range() / uhd_usrp_get_tx_freq_range()
 *       - Account for upconverter offset (rx_upconverter/tx_upconverter)
 *         which may differ for RX and TX directions
 *       - Hamlib supports separate rx_range_list and tx_range_list in
 *         dump_state, so we can report different ranges per direction
 *       - Update ranges on retune if SDR has tunable front-end filters
 *         that change the valid frequency range
 *       - IMPORTANT: Never query hardware from subroutines! Query once
 *         during SDR setup/startup/retune and store results for later use.
 */

typedef struct rigctl_sdr_caps {
	/* Frequency range (Hz) - 0 means unknown/unlimited
	 * Populated at SDR startup/retune, never queried from subroutines */
	double		rx_freq_min;
	double		rx_freq_max;
	double		tx_freq_min;
	double		tx_freq_max;

	/* Upconverter offset (Hz) - e.g., +125000000 for Ham-It-Up
	 * These are ADDED to the user-visible frequency to get SDR frequency.
	 * Example: User tunes 7.1 MHz, upconverter=125 MHz -> SDR tunes 132.1 MHz
	 * Note: RX and TX can have different upconverters (e.g., transverter) */
	double		rx_upconverter;
	double		tx_upconverter;

	/* Gain range (dB) */
	double		rx_gain_min;
	double		rx_gain_max;
	double		tx_gain_min;
	double		tx_gain_max;

	/* Gain element names (space-separated) */
	char		rx_gain_names[256];
	char		tx_gain_names[256];

	/* Sample rate limits */
	double		sample_rate_min;
	double		sample_rate_max;

	/* Flags */
	int		has_rx;
	int		has_tx;
	int		is_split;	/* Separate TX/RX devices */
} rigctl_sdr_caps_t;

/* ============================================================
 * Callbacks
 * ============================================================ */

typedef struct rigctl_callbacks {
	/* Frequency control */
	int (*set_rx_freq)(double freq_hz, void *arg);
	int (*set_tx_freq)(double freq_hz, void *arg);
	double (*get_rx_freq)(void *arg);
	double (*get_tx_freq)(void *arg);

	/* PTT control
	 * TODO: Implement proper PTT handling for TX/RX switching
	 * - set_ptt(1) should enable TX, set_ptt(0) should enable RX
	 * - get_ptt() returns current PTT state
	 * - For full-duplex operation, both TX and RX can be active
	 */
	int (*set_ptt)(int ptt, void *arg);
	int (*get_ptt)(void *arg);

	/* Recording control */
	int (*start_recording)(void *arg);
	int (*stop_recording)(void *arg);
	int (*is_recording)(void *arg);

	/* Callback context */
	void *arg;
} rigctl_callbacks_t;

/* ============================================================
 * Server State
 * ============================================================ */

#define RIGCTL_FD_NONE		(-1)
#define RIGCTL_MAX_CLIENTS	4
#define RIGCTL_CMD_MAX		256

/* Per-client state */
typedef struct {
	int		fd;
	int		close_pending;	/* Close after flushing tx_ring */
	rigctl_ring_t	tx_ring;
	rigctl_ring_t	rx_ring;
	char		cmd_buf[RIGCTL_CMD_MAX];
	int		cmd_len;
	char		client_ip[RIGCTL_EP_IP_LEN];
	int		client_port;
	int64_t		connect_time_us;
} rigctl_client_t;

typedef struct rigctl_server {
	/* Config */
	rigctl_endpoint_t ep;
	char		allowed_hosts[512];

	/* File descriptors */
	int		listen_fd;
	rigctl_client_t	clients[RIGCTL_MAX_CLIENTS];
	int		num_clients;

	/* Serial mode (single client) */
	int		serial_fd;

	/* SDR capabilities and configuration */
	rigctl_sdr_caps_t sdr_caps;

	/* Radio state (cached for queries) */
	double		rx_freq;	/* RX frequency (Hz) */
	double		tx_freq;	/* TX frequency (Hz) */
	int		rx_mode;	/* RX modulation (enum modulation) */
	int		tx_mode;	/* TX modulation (enum modulation) */
	int		mode;		/* Current/primary mode (for compat) */
	int		rx_bandwidth;	/* RX filter bandwidth (Hz) */
	int		tx_bandwidth;	/* TX filter bandwidth (Hz) */
	int		bandwidth;	/* Current/primary bandwidth (for compat) */
	double		signal_level;	/* RX signal level (dBFS) */
	double		squelch_level;	/* Squelch threshold (dBFS) */
	double		audio_gain;	/* Audio gain (dB) */
	double		rf_gain;	/* RF/IF gain (dB) */
	double		deviation;	/* FM deviation (Hz) */
	double		mod_index;	/* AM modulation index (0.0-1.0) */
	int		stereo;		/* Stereo enabled */
	int		rds_enabled;	/* RDS encoder/decoder enabled */
	int		dsp_running;	/* DSP processing active */
	int		recording;	/* Recording active */
	int		tx_enabled;	/* TX mode enabled */
	int		rx_enabled;	/* RX mode enabled */
	int		ptt;		/* PTT state (0=RX, 1=TX) */
	int		split;		/* Split mode (separate TX/RX freq) */
	double		tx_power;	/* TX power level (dBm or watts) */

	/* TX status */
	double		tx_rms;
	double		tx_peak_i;
	double		tx_peak_q;

	/* RDS data */
	uint16_t	rds_pi;
	char		rds_ps[9];
	char		rds_rt[65];

	/* Hardware gains */
	char		gain_names[256];	/* space-separated */
	double		gains[16];
	int		num_gains;

	/* Callbacks */
	rigctl_callbacks_t cb;

	/* References */
	struct radio	*radio;
	struct signal_meter *meter;

	/* Stats */
	uint64_t	rx_bytes;
	uint64_t	tx_bytes;
	uint64_t	total_connections;
} rigctl_server_t;

/* ============================================================
 * API
 * ============================================================ */

/* Parse endpoint string.
 * TCP: "ip:port" or just "port" (binds to 0.0.0.0)
 * Serial: "device,speed,8N1[,flow]"
 * Returns 0 on success, -1 on error. */
int rigctl_ep_parse(const char *str, rigctl_endpoint_t *ep);

/* Initialize server.
 * Returns 0 on success, -1 on error. */
int rigctl_server_init(rigctl_server_t *srv, const char *endpoint,
		       const char *allowed_hosts);

/* Cleanup server. */
void rigctl_server_cleanup(rigctl_server_t *srv);

/* Set callbacks. */
void rigctl_server_set_callbacks(rigctl_server_t *srv,
				 const rigctl_callbacks_t *cb);

/* Set radio reference. */
void rigctl_server_set_radio(rigctl_server_t *srv, struct radio *radio);

/* Set signal meter reference. */
void rigctl_server_set_meter(rigctl_server_t *srv, struct signal_meter *meter);

/* Set TX/RX mode flags. */
void rigctl_server_set_mode_flags(rigctl_server_t *srv, int tx, int rx);

/* Set SDR capabilities (frequency limits, upconverter offset, etc.).
 * Call after init to configure hardware-specific limits. */
void rigctl_server_set_sdr_caps(rigctl_server_t *srv, const rigctl_sdr_caps_t *caps);

/* Poll for I/O. Call from main loop. Never blocks. */
int rigctl_server_poll(rigctl_server_t *srv);

/* Update signal level (call periodically). */
void rigctl_server_set_signal(rigctl_server_t *srv, double level_dbfs);

/* Update TX status. */
void rigctl_server_set_tx_status(rigctl_server_t *srv,
				 double rms, double peak_i, double peak_q);

/* Update frequency (after retune). */
void rigctl_server_set_frequency(rigctl_server_t *srv, double rx_freq, double tx_freq);

/* Update mode. */
void rigctl_server_set_modulation(rigctl_server_t *srv, int mode, int bandwidth,
				  double deviation, double mod_index);

/* Update RDS data. */
void rigctl_server_set_rds(rigctl_server_t *srv, uint16_t pi,
			   const char *ps, const char *rt);

/* Update stereo/RDS flags. */
void rigctl_server_set_flags(rigctl_server_t *srv, int stereo, int rds);

/* Check if any client connected. */
int rigctl_server_connected(const rigctl_server_t *srv);

#endif /* RIGCTL_SERVER_H */
