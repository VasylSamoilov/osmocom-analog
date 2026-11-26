/* Radio main function
 *
 * (C) 2018 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _GNU_SOURCE

enum paging_signal;

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libsdr/sdr_config.h"
#include "../libsdr/sdr.h"
#include "../libsdr/sdr_status.h"
#include "../liboptions/options.h"
#include <osmocom/cc/misc.h>
#include "radio.h"
#include "rds.h"
#include "rds_protocol.h"
#include "rds_tables.h"
#include "rigctl_server.h"
#include "signal_meter.h"
#include "../libmobile/mcc.h"

#define DEFAULT_LO_OFFSET -1000000.0
#define RDS_PAGING_PIPE_DEFAULT "/tmp/rds_paging_send"

/* Protocol server callback context */
typedef struct {
	rds_server_t *xdr_srv;
	rds_server_t *rdsspy_srv;
	rds_server_t *asciig_srv;
	radio_t *radio;
	struct station *station;	/* the station the servers belong to */
} rds_callback_ctx_t;

/* one station: its options, RDS, servers and files */
typedef struct station {
	const char *rds_paging_pipe_path;
	int paging_pipe_fd;
	int paging_pipe_made;	/* this station made the FIFO (it removes it at exit) */
	char paging_buf[4096];	/* paging FIFO: the partial line */
	int paging_buf_len;

	/* Signal meter (file-scope so tune callback can access it) */
	signal_meter_t *meter;
	int spectral_freq_idx;	/* index of our registered frequency in spectral measurement */

	double frequency;
	int dsp_samplerate;
	const char *tx_wave_file;
	const char *rx_wave_file;
	const char *tx_audiodev;
	const char *rx_audiodev;
	enum modulation modulation;
	int rx, tx;
	/* Audio bandwidth defaults (Hz) - NOT RF/signal bandwidth.
	 * FM broadcast audio: 30 Hz - 15 kHz (matches 50/75µs pre-emphasis curve).
	 *
	 * Carson's Rule: BW = 2 * (Δf + fm)
	 *   where Δf = peak deviation (75 kHz), fm = max modulating freq
	 *
	 * Our variables map to Carson's Rule as:
	 *   Δf = deviation (75000 Hz)
	 *   fm = bandwidth_fm (15000 Hz) for mono, or highest subcarrier for stereo/RDS
	 *   baseband_extent = Δf + fm (one-sided max freq), RF bandwidth = 2x this
	 *
	 * Theoretical vs practical RF bandwidth:
	 *   Mono:   2*(75k+15k) = 180 kHz — fits 200 kHz channel
	 *   Stereo: 2*(75k+53k) = 256 kHz — theoretical, but fits 200 kHz
	 *   RDS:    2*(75k+60k) = 270 kHz — theoretical, but fits 200 kHz
	 *
	 * Real-world FM+RDS fits in 200 kHz because:
	 *   - RDS at 57 kHz is only ~5% injection level (low energy)
	 *   - High-frequency sidebands fall off rapidly
	 *   - Carson's Rule is worst-case; actual spectrum is narrower
	 *
	 * We use the theoretical values for baseband_extent to ensure our
	 * filters and sample rates capture the full baseband spectrum.
	 */
	double bandwidth_am;	/* AM audio bandwidth (Hz) */
	double bandwidth_fm;	/* FM audio bandwidth (Hz) */
	double bandwidth;	/* User-specified audio bandwidth override */
	double deviation;	/* FM deviation (Hz), ±75kHz for broadcast */
	double modulation_index;
	double time_constant_us;
	double volume;
	int stereo;
	int rds;
	int rds2;
	int rds_debug;
	int rds_verbose;
	int rds_force_rbds;
	int sca_67k;
	int sca_92k;
	int am_compandor;
	int fm_compandor;
	int rds_paging;
	int rds_paging_rpc;		/* Default RPC=4: group desig 001, batt sync 00 */
	uint16_t rds_paging_cc;		/* E.212 MCC for international (0=not set) */
	uint8_t rds_paging_opc;		/* OPC for international (0=not set) */
	uint8_t rds_paging_pac;		/* PAC for paging area (0=all areas) */
	int rds_paging_battery;		/* pagers use battery saving: place calls in their interval */
	uint8_t rds_ecc;		/* Extended Country Code for 1A variant 0 (0 = not set) */
	uint16_t rds_pi_opt;		/* --pi, for the ECC / PI country digit check */
	int rds_paging_13a;		/* enhanced paging 13A address notification */
	int rds_paging_scan;		/* address range scan enabled */
	uint32_t rds_paging_scan_from;
	uint32_t rds_paging_scan_to;
	int rds_paging_scan_type;	/* enum rds_paging_msg_type: NUM10 */
	const char *rds_hexrds_file;
	const char *rds_bitstream_file;
	const char *rds_tx_hexrds_file;
	const char *rds_tx_bitstream_file;
	const char *rds_tmc_events_file;
	const char *rds_tmc_locations_path;

	/* Protocol server endpoints */
	const char *xdr_gtk_server;
	const char *rdsspy_server;
	const char *uecp_server;
	const char *asciig_server;
	const char *rigctl_server_endpoint;
	const char *rigctl_allowed_hosts;
	const char *xdr_gtk_password;

	int channelizer_rate; /* IQ rate to the radio, 0 = automatic */
	radio_opts_t radio_opts; /* the station's settings outside radio_init()'s parameters (radio_opts_init() before the options) */
	int use_polyphase;
	int use_afc;
	double afc_tc_ms;
	double afc_max_hz;

	/* running (station_init() .. station_exit()) */
	int iq_rate;		/* IQ rate between the SDR and the radio */
	int buffer_size;	/* IQ samples per SDR read / write */
	float *sendbuff;
	radio_t radio;
	rds_server_t xdr_srv, rdsspy_srv, uecp_srv, asciig_srv;
	rigctl_server_t rigctl_srv;
	int xdr_enabled, rdsspy_enabled, uecp_enabled, asciig_enabled;
	int rigctl_enabled;
	rds_callback_ctx_t rds_cb_ctx;
} station_t;

/* Options (docs/RADIO-STATIONS.md): the config file and the command line
 * options before the first -f fill station_default; the first command line
 * -f starts the station as a copy of it, the options after it are that
 * station's. Process options (-v, --fast-math) and the SDR options are
 * process-wide wherever they stand. One station for now: a second -f is
 * refused. After the options st points at the station, never at the
 * defaults. */
static station_t station_default;
static station_t station0;
static int num_stations;
static int options_from_command_line;
static station_t *st = &station_default;

/* the initial values of a station's settings */
static void station_defaults(station_t *s)
{
	memset(s, 0, sizeof(*s));
	s->rds_paging_pipe_path = RDS_PAGING_PIPE_DEFAULT;
	s->paging_pipe_fd = -1;
	s->meter = NULL;
	s->spectral_freq_idx = -1;
	s->frequency = 0.0;
	s->dsp_samplerate = 100000;
	s->modulation = MODULATION_NONE;
	s->rx = 0;
	s->tx = 0;
	s->bandwidth_am = 4500.0;
	s->bandwidth_fm = 15000.0;
	s->bandwidth = 0.0;
	s->deviation = 75000.0;
	s->modulation_index = 1.0;
	s->time_constant_us = 50.0;
	s->volume = 1.0;
	s->rds_paging_rpc = 4;
	s->rds_paging_scan_type = 1;
	s->channelizer_rate = 0;
	s->use_polyphase = 0;
	s->use_afc = 0;
	s->afc_tc_ms = 300.0;
	s->afc_max_hz = 5000.0;
	radio_opts_init(&s->radio_opts);
}

sender_t *sender_head = NULL;
int use_sdr = 0;
int num_kanal = 1; /* only one channel used for debugging */
int rt_prio = 0;
int fast_math = 0;

sender_t *get_sender_by_empfangsfrequenz(double __attribute__((unused)) freq) { return NULL; }

static int dsp_buffer = 30;

/* global variable to quit main loop */
int quit = 0;

/* Tune callback for XDR-GTK (RX side): retunes the SDR receiver */
static void radio_tune_cb(int freq_khz, void *arg)
{
	rds_callback_ctx_t *ctx = (rds_callback_ctx_t *)arg;
	station_t *s = ctx->station;
	double freq_hz = freq_khz * 1000.0;

	/* Dump RDS status before re-tuning (if RDS enabled and have data) */
	if (s->rds && ctx->radio) {
		if (ctx->radio->rds_dec.blocks_ok > 0) {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz, last known RDS data:\n",
			     s->frequency / 1e6);
			rds_decoder_status(&ctx->radio->rds_dec);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz (no valid RDS blocks received)\n",
			     s->frequency / 1e6);
		}
	}

	LOGP(DRADIO, LOGL_NOTICE, "Re-tuning SDR RX to %d kHz\n", freq_khz);

	if (sdr_set_rx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Failed to re-tune SDR to %d kHz\n", freq_khz);
		return;
	}

	/* Update the station's frequency and XDR-GTK server state */
	s->frequency = freq_hz;
	if (ctx->xdr_srv)
		ctx->xdr_srv->freq_khz = freq_khz;

	/* Clear stale measurements from previous frequency */
	if (s->meter)
		signal_meter_reset(s->meter);

	/* Reset RDS decoder for new station */
	if (s->rds && ctx->radio)
		rds_decoder_reset(&ctx->radio->rds_dec);

	/* Update spectral measurement for new frequency */
	sdr_spectral_set_center(freq_hz);
	sdr_spectral_clear_freq();
	s->spectral_freq_idx = sdr_spectral_register_freq(freq_hz);

	/* Tell RDS-Spy to reset its decoder state (new station) */
	if (ctx->rdsspy_srv)
		rds_server_send_reset(ctx->rdsspy_srv);
}

/* Scan retune callback for XDR-GTK spectral scan:
 * called only when the scan engine needs to shift the SDR window. */
static void radio_scan_cb(int freq_khz, void *arg)
{
	rds_callback_ctx_t *ctx = (rds_callback_ctx_t *)arg;
	double freq_hz = freq_khz * 1000.0;

	if (sdr_set_rx_frequency(freq_hz) < 0)
		return;

	ctx->station->frequency = freq_hz;
	if (ctx->xdr_srv)
		ctx->xdr_srv->freq_khz = freq_khz;

	/* Update spectral center for scan retune */
	sdr_spectral_set_center(freq_hz);
}

/* Setting callback for XDR-GTK: handles B (stereo/mono) and other settings */
static void radio_setting_cb(const char *name, int val, void *arg)
{
	rds_callback_ctx_t *ctx = (rds_callback_ctx_t *)arg;

	if (!ctx || !ctx->radio)
		return;

	if (name[0] == 'B') {
		/* B0 = auto stereo, B1 = forced mono */
		radio_set_forced_mono(ctx->radio, val != 0);
	}
}

static void radio_tx_tune_cb(int freq_khz, void *arg)
{
	rds_callback_ctx_t *ctx = (rds_callback_ctx_t *)arg;
	double freq_hz = freq_khz * 1000.0;

	LOGP(DRADIO, LOGL_NOTICE, "Re-tuning SDR TX to %d kHz\n", freq_khz);

	if (sdr_set_tx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Failed to re-tune SDR TX to %d kHz\n", freq_khz);
		return;
	}

	ctx->station->frequency = freq_hz;
}

/* Rigctl callbacks (arg: the station) */
static int rigctl_set_rx_freq(double freq_hz, void *arg)
{
	station_t *s = (station_t *)arg;
	radio_t *r = &s->radio;

	/* Dump RDS status before re-tuning (if RDS enabled and have data) */
	if (s->rds) {
		if (r->rds_dec.blocks_ok > 0) {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz, last known RDS data:\n",
			     s->frequency / 1e6);
			rds_decoder_status(&r->rds_dec);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz (no valid RDS blocks received)\n",
			     s->frequency / 1e6);
		}
	}

	LOGP(DRADIO, LOGL_NOTICE, "Rigctl: Re-tuning SDR RX to %.0f Hz\n", freq_hz);

	if (sdr_set_rx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Rigctl: Failed to re-tune SDR RX\n");
		return -1;
	}

	s->frequency = freq_hz;

	/* Clear stale measurements */
	if (s->meter)
		signal_meter_reset(s->meter);

	/* Reset RDS decoder for new station */
	if (s->rds)
		rds_decoder_reset(&r->rds_dec);

	/* Update spectral measurement */
	sdr_spectral_set_center(freq_hz);
	sdr_spectral_clear_freq();
	s->spectral_freq_idx = sdr_spectral_register_freq(freq_hz);

	return 0;
}

static int rigctl_set_tx_freq(double freq_hz, void *arg)
{
	station_t *s = (station_t *)arg;

	LOGP(DRADIO, LOGL_NOTICE, "Rigctl: Re-tuning SDR TX to %.0f Hz\n", freq_hz);

	if (sdr_set_tx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Rigctl: Failed to re-tune SDR TX\n");
		return -1;
	}

	s->frequency = freq_hz;
	return 0;
}

static double rigctl_get_rx_freq(void *arg)
{
	return ((station_t *)arg)->frequency;
}

static double rigctl_get_tx_freq(void *arg)
{
	return ((station_t *)arg)->frequency;
}

/* Callback for decoded RDS groups - forwards to protocol servers */
static void rds_group_callback(const uint16_t blocks[4], const uint8_t status[4], void *arg)
{
	rds_callback_ctx_t *ctx = (rds_callback_ctx_t *)arg;

	if (ctx->xdr_srv) {
		rds_server_send_group(ctx->xdr_srv, blocks, status);
		/* XDR-GTK needs explicit PI messages to drive its rds_timeout counter.
		 * Send P<XXXX> for every group where block A is usable (error < uncorrectable).
		 * Map RDS_STATUS_* + BER to XDR-GTK error levels:
		 *   RDS_STATUS_ERROR -> 3 (severe, ⁇)
		 *   RDS_STATUS_CORRECTED + BER > 10% -> 2 (moderate, visible ?)
		 *   RDS_STATUS_CORRECTED + BER <= 10% -> 1 (minor, faint ?)
		 *   RDS_STATUS_VALID + BER >= 50% -> 2 (moderate, visible ?)
		 *   RDS_STATUS_VALID + BER > 20% -> 1 (minor, faint ?)
		 *   RDS_STATUS_VALID + BER <= 20% -> 0 (no error indicator) */
		if (status[0] < RDS_ERR_UNCORR) {
			int err_level = 0;
			double ber = (ctx->radio) ? ctx->radio->rds_dec.ber_percent : 0.0;
			if (status[0] == RDS_STATUS_ERROR) {
				err_level = 3;
			} else if (status[0] == RDS_STATUS_CORRECTED) {
				err_level = (ber > 10.0) ? 2 : 1;
			} else {
				/* RDS_STATUS_VALID */
				if (ber >= 50.0)
					err_level = 2;
				else if (ber > 20.0)
					err_level = 1;
			}
			rds_server_send_pi(ctx->xdr_srv, blocks[0], err_level);
		}
	}
	if (ctx->rdsspy_srv)
		rds_server_send_group(ctx->rdsspy_srv, blocks, status);
	if (ctx->asciig_srv)
		rds_server_send_group(ctx->asciig_srv, blocks, status);
}

static void sighandler(int sigset)
{
	if (sigset == SIGHUP)
		return;
	if (sigset == SIGPIPE)
		return;

//	clear_console_text();
	printf("Signal received: %d\n", sigset);

	quit = 1;
}

static int get_char()
{
	struct timeval tv = {0, 0};
	fd_set fds;
	char c = 0;
	int __attribute__((__unused__)) rc;

	FD_ZERO(&fds);
	FD_SET(0, &fds);
	select(0+1, &fds, NULL, NULL, &tv);
	if (FD_ISSET(0, &fds)) {
		rc = read(0, &c, 1);
		return c;
	} else
		return -1;
}

static void print_help(const char *arg0)

{
	printf("Usage: %s --sdr-soapy|--sdr-uhd <sdr options> -f <frequency> -M <modulation> -R|-T [options]\n", arg0);
	/*      -                                                                             - */
	printf("\noptions:\n");
	printf(" -h --help\n");
	printf("        This help\n");
	printf(" --config [~/]<path to config file>\n");
	printf("        Give a config file to use. If it starts with '~/', path is at home dir.\n");
	printf("        Each line in config file is one option, '-' or '--' must not be given!\n");
	printf(" -f --frequency <frequency>\n");
	printf("        Give frequency in Hertz. Starts the station: station options before\n");
	printf("        it (and in the config file) are defaults, the ones after it are the\n");
	printf("        station's. One -f for now (several stations: docs/RADIO-STATIONS.md).\n");
	printf(" -s --samplerate <sample rate>\n");
	printf("        Give signal processing sample rate in Hz. (default = %d)\n", st->dsp_samplerate);
	printf("        This sample rate must be high enough for the signal's spectrum to fit.\n");
	printf("        I will inform you, if this bandwidth is too low.\n");
	printf(" -r --tx-wave-file <filename>\n");
	printf("        Input transmitted audio from wave file\n");
	printf(" -w --rx-wave-file <filename>\n");
	printf("        Output received audio to wave file\n");
	printf(" -a --audio-device hw:<card>,<device>\n");
	printf("        Input audio from sound card's device number\n");
	printf(" -M --modulation fm | am | usb | lsb\n");
	printf("        fm = Frequency modulation to be used for VHF.\n");
	printf("        am = Amplitude modulation to be used for long/medium/short wave.\n");
	printf("        usb = Amplitude modulation with upper side band only.\n");
	printf("        lsb = Amplitude modulation with lower side band only.\n");
	printf(" -R --rx\n");
	printf("        Receive radio signal.\n");
	printf(" -T --tx\n");
	printf("        Transmit radio signal.\n");
	printf(" -B --bandwidth\n");
	printf("        Give bandwidth of audio frequency. (default AM=%.0f FM=%.0f)\n", st->bandwidth_am, st->bandwidth_fm);
	printf(" -D --deviation\n");
	printf("        Give deviation of frequency modulated signal. (default %.0f)\n", st->deviation);
	printf(" -I --modulation-index 0..1\n");
	printf("        Give modulation index of amplitude modulated signal. (default %.0f)\n", st->modulation_index);
	printf(" -C --compandor\n");
	printf("        Enable audio compressor for AM to improve modulation depth.\n");
	printf("        Uses 2:1 compression with 5ms attack, 200ms recovery.\n");
	printf("    --fm-compandor\n");
	printf("        Enable audio compressor for FM to improve broadcast loudness.\n");
	printf("        Uses 2:1 compression with 1ms attack, 50ms recovery.\n");
	printf("        Raises average modulation to match commercial FM stations.\n");
	printf(" -E --emphasis <uS> | 0\n");
	printf("        Use given time constant of pre- and de-emphasis for frequency\n");
	printf("        modulation. Give 0 to disable emphasis. (default = %.0f uS)\n", st->time_constant_us);
	printf("        VHF broadcast 50 uS in Europe and 75 uS in the United States.\n");
	printf("        Other radio FM should use 530 uS, to cover complete speech spectrum.\n");
	printf(" -V --volume %.3f\n", st->volume);
	printf("        Change volume of radio side. (Gains transmission, dampens reception)\n");
	printf(" -S --stereo\n");
	printf("        Enables stereo carrier for frequency modulated UHF broadcast.\n");
	printf("        It uses the 'Pilot-tone' system.\n");
	printf("    --rds\n");
	printf("        Enables RDS (Radio Data System) 57 kHz subcarrier.\n");
	printf("        Reserves bandwidth for RDS but does not encode data (placeholder).\n");
	printf("    --rds2\n");
	printf("        Enables RDS2 with additional subcarriers up to 80 kHz.\n");
	printf("    --rds-debug\n");
	printf("        Enable RDS decoder debug logging (raw hex codes).\n");
	printf("    --rds-verbose\n");
	printf("        Enable RDS decoder verbose logging (human-readable interpretation).\n");
	printf("    --rbds\n");
	printf("        Force RBDS decoding (callsign lookup, US PTY names).\n");
	printf("    --rds-paging [rpc]\n");
	printf("        Enable RDS Radio Paging (EN 50067 Annex M, Groups 7A/1A/13A).\n");
	printf("        Implies --rds. Messages are sent via a named pipe (FIFO).\n");
	printf("        RPC = Radio Paging Code (0-31, default 4).\n");
	printf("          Encodes group designation (which address ranges) and battery\n");
	printf("          saving sync interval. RPC=0 means no paging. Default 4 covers\n");
	printf("          address groups 00-99 with sync=0 (recommended).\n");
	printf("        FIFO format: address,type,options,message\n");
	printf("        Types: tone, numeric, numeric10, numeric18, alpha, vnum, functions, intl15, intlfunc\n");
	printf("        tone, numeric and alpha are basic paging (EN 50067 M.2).\n");
	printf("        vnum, functions and any message with ni=1 use the enhanced protocol\n");
	printf("        (M.3.5). They need a pager that implements it and --rds-paging-opc\n");
	printf("        1-15. A basic pager ignores the control byte and shows the digits\n");
	printf("        as garbled alpha text.\n");
	printf("        Options (space-separated key=value, can be empty):\n");
	printf("          repeat=N      total send count, 1-15 (default 1)\n");
	printf("          interval=N    seconds between repeats (default 5)\n");
	printf("          tonetype=N    tone alert type 0-7 (tone-only messages, default 0)\n");
	printf("          ni=0|1        NI = National/International flag (default 0=national)\n");
	printf("          cc=NNN        MCC = Mobile Country Code per ITU E.212 (e.g. 255=Ukraine)\n");
	printf("          opc=N         OPC = Operator Code 1-15 for international messages\n");
	printf("        International mode (ni=1, EN 50067 M.3.5.7):\n");
	printf("          Requires cc= and opc= (per-message or --rds-paging-cc/opc).\n");
	printf("          Only alpha and vnum support NI. Tone and basic numeric do not.\n");
	printf("          MCC is 3 BCD digits (e.g. 255=Ukraine, 262=Germany, 208=France).\n");
	printf("          Recommended max alpha=78 chars, vnum=156 digits (vs 80/160 national).\n");
	printf("        International mode on a basic pager (EN 50067 M.2, roaming):\n");
	printf("          Basic paging has only intl15 and intlfunc as international\n");
	printf("          formats (M.2.1.6.5/6); cc must be the pager's home MCC.\n");
	printf("          The pager is set to the foreign PI country digit (--pi).\n");
	printf("          It ignores NI: ni=1 alpha shows I1I2I3OPC as two characters.\n");
	printf("          Tested pager: national types not shown in this mode. Example:\n");
	printf("          --pi D2E7 --rds-ecc E0, pager set to D, home MCC 255 (Ukraine):\n");
	printf("          echo '200078,intl15,cc=255,123456789012345' > %s\n", st->rds_paging_pipe_path);
	printf("        Message length limits (EN 50067):\n");
	printf("          numeric 18 digits, national alpha 80 chars, intl15 15 digits,\n");
	printf("          intlfunc 7 digits: longer messages are rejected.\n");
	printf("          vnum, functions and international alpha only have a recommended\n");
	printf("          maximum: longer messages are sent with a warning up to 10 times\n");
	printf("          that value and rejected beyond it. With battery saving a call must\n");
	printf("          fit the three interval windows of its address (about 90 groups).\n");
	printf("        Valid message characters (others are rejected):\n");
	printf("          numeric, numeric10, numeric18, vnum, intl15: digits 0-9 and space\n");
	printf("          (space is sent as hex A).\n");
	printf("          functions, intlfunc: hex digits 0-9 A-F (a-f too) and space.\n");
	printf("          alpha: any 8-bit character, see the pager charset; <0xCC> sends a raw byte.\n");
	printf("          tone: no message.\n");
	printf("        Hex escapes: <0xCC> in message sends raw byte 0xCC.\n");
	printf("        Examples:\n");
	printf("          echo '200078,tone,,' > %s\n", st->rds_paging_pipe_path);
	printf("          echo '200078,tone,tonetype=3 repeat=2,' > %s\n", st->rds_paging_pipe_path);
	printf("          echo '200078,numeric,,1234567890' > %s\n", st->rds_paging_pipe_path);
	printf("          echo '200078,alpha,,Hello World' > %s\n", st->rds_paging_pipe_path);
	printf("          echo '200078,alpha,repeat=3 interval=5,Hello, World!' > %s\n", st->rds_paging_pipe_path);
	printf("          echo '200078,alpha,ni=1 cc=255 opc=1,Intl msg' > %s\n", st->rds_paging_pipe_path);
	printf("          echo '200078,vnum,,31415926535' > %s\n", st->rds_paging_pipe_path);
	printf("          echo '200078,vnum,ni=1 cc=262 opc=3,0123456789' > %s\n", st->rds_paging_pipe_path);
	printf("    --rds-paging-fifo <path>\n");
	printf("        Path for the RDS paging named pipe (default %s).\n", RDS_PAGING_PIPE_DEFAULT);
	printf("    --rds-paging-cc <mcc>\n");
	printf("        MCC = Mobile Country Code, default for international paging (200-999).\n");
	printf("        Per ITU-T E.212. Used when ni=1 and no per-message cc= is given.\n");
	printf("        Examples: 255=Ukraine, 262=Germany, 208=France, 260=Poland.\n");
	printf("    --rds-paging-opc <opc>\n");
	printf("        OPC = Operator Code (0-15). Identifies the paging operator on this\n");
	printf("        channel. Broadcast in Group 1A for receiver channel locking.\n");
	printf("        0 = no enhanced paging (default, recommended unless multi-operator).\n");
	printf("        Also used as default OPC for international messages (ni=1).\n");
	printf("    --rds-paging-pac <pac>\n");
	printf("        PAC = Paging Area Code (0-63). Network-wide area filter broadcast\n");
	printf("        in Group 1A variant 2. Pagers outside this area ignore messages.\n");
	printf("        0 = all areas (default, recommended for single-transmitter setups).\n");
	printf("    --rds-ecc <hex> | list\n");
	printf("        ECC = Extended Country Code (hex 01-FF, EN 50067 Annex D), sent in\n");
	printf("        Group 1A variant 0. Together with the first digit of the PI it\n");
	printf("        identifies the country (e.g. PI Dxxx + ECC E0 = Germany). Required\n");
	printf("        with --rds-paging and --tx: paging pagers lock to the channel by\n");
	printf("        country (PI country digit and ECC, EN 50067 M.3.2.3).\n");
	printf("        'list' prints the ECC / PI country digit -> country mapping.\n");
	printf("    --rds-paging-13a\n");
	printf("        Enhanced paging address notification (EN 50067 M.3.4.1): one\n");
	printf("        Group 13A (25 notification bits) right after the first marker of\n");
	printf("        each interval. The bits flag pending messages for the sub groups\n");
	printf("        (address digits Z2Z3) of the pagers of that interval, so enhanced\n");
	printf("        pagers with a clear bit can go back to battery saving at once.\n");
	printf("    --rds-paging-battery-saving\n");
	printf("        For pagers that use battery saving (EN 50067 M.2.1.3). Each minute\n");
	printf("        has ten 6 s intervals I0..I9; a pager sleeps outside the interval of\n");
	printf("        the last digit of its address. With this option 1A carries the\n");
	printf("        interval sync bits and a call is only started inside the window of\n");
	printf("        its address: interval d and the two following (d = last digit), and\n");
	printf("        it must end inside the window. Other calls wait in the queue, so a\n");
	printf("        single call can be delayed up to 54 s. Clock-time (4A) is forced on\n");
	printf("        and the schedule is 4A, 1A, 0A (4 per second, PS) and 7A. Without this option\n");
	printf("        calls are sent at once, as for pagers that listen all the time.\n");
	printf("    --rds-paging-scan <from> <to>\n");
	printf("        Scan through the given paging address range (0-999999) with one\n");
	printf("        message per address. The message carries the 6-digit address, so\n");
	printf("        you can tell which address a pager responded to. Requires\n");
	printf("        --rds-paging; the RPC must cover the scanned group codes (default\n");
	printf("        RPC 4 covers 00-99). The group schedule is: 4A each minute, 1A at\n");
	printf("        the battery saving interval layout of EN 50067 M.2.1.3 (13 per 6 s\n");
	printf("        interval, 1A #1/#2 one second apart, interval sync bits in B1/B0),\n");
	printf("        type 0 groups two per second for PS (EN 50067 3.1.3 minimum when\n");
	printf("        capacity is needed for other applications) and 7A for the rest.\n");
	printf("        Clock-time (4A) is forced on. The FIFO stays usable.\n");
	printf("        Starts with the \"Paging\" RDS preset: PS \"PAGING\", PTY 0, TP 0,\n");
	printf("        no AF, LIC, PIN, RT or other services; PI from --pi, ECC from\n");
	printf("        --rds-ecc. 'f' still cycles through the other presets.\n");
	printf("    --rds-paging-scan-type numeric | alpha | tone\n");
	printf("        Message type for the scan (default numeric, 10 digit; alpha sends\n");
	printf("        the address as text; tone sends a tone-only page).\n");
	printf("        Groups per call: tone 1, numeric 2, alpha 2 if the range stays in one\n");
	printf("        group code (first two digits, e.g. 120000 129999: the pager shows\n");
	printf("        the last four digits), else alpha 3 (all six digits).\n");
	printf("    --rds-hexrds-file <filename>\n");
	printf("        Write received RDS groups to file in hexrds format.\n");
	printf("        Format: \"XXXX XXXX XXXX XXXX\" per line (---- for missing blocks).\n");
	printf("    --rds-bitstream-file <filename>\n");
	printf("        Write received RDS bitstream to file in binary format.\n");
	printf("        Raw bits packed MSB first, compatible with .rds files.\n");
	printf("    --rds-tx-hexrds-file <filename>\n");
	printf("        Transmit RDS groups from hexrds file (bypasses encoder).\n");
	printf("        Format: \"XXXX XXXX XXXX XXXX\" per line. Loops continuously.\n");
	printf("        Transmitted groups are logged using the decoder.\n");
	printf("    --rds-tx-bitstream-file <filename>\n");
	printf("        Transmit RDS bitstream from binary file (bypasses encoder).\n");
	printf("        Raw bits packed MSB first. Loops continuously.\n");
	printf("        Transmitted groups are logged using the decoder.\n");
	printf("    --rds-tmc-events <filename>\n");
	printf("        Load TMC event code descriptions from a semicolon-delimited CSV\n");
	printf("        (docs/rds/tmc_events.csv) for human-readable TMC event names in\n");
	printf("        the verbose log. Format: Code;Description;...\n");
	printf("        Source: OSM wiki TMC/Event_Code_List (1552 ALERT-C events).\n");
	printf("    --rds-tmc-locations <path>\n");
	printf("        Load TMC location descriptions for human-readable road names in\n");
	printf("        the verbose log. Accepts either:\n");
	printf("          - A standard ISO 14819-3 LCL directory with NAMES.DAT + ROADS.DAT\n");
	printf("            (produces 'A1 - road name' per LCD code)\n");
	printf("          - A plain CSV file: LCD;Name (e.g. converted from gr-rds Italy\n");
	printf("            tmc_locations_italy.h or any national NAMES.DAT)\n");
	printf("    --xdr-gtk-server <endpoint>\n");
	printf("        Enable XDR-GTK protocol server for RDS decoder output.\n");
	printf("        TCP: <ip>:<port> (e.g., 127.0.0.1:7373)\n");
	printf("        Serial: <device>,<speed>,<8N1>[,<flow>]\n");
	printf("    --xdr-gtk-password <password>\n");
	printf("        Set password for XDR-GTK TCP authentication (SHA1 challenge-response).\n");
	printf("        If not set, clients connect without a password.\n");
	printf("    --rdsspy-server <endpoint>\n");
	printf("        Enable RDS Spy protocol server for RDS decoder output.\n");
	printf("        TCP: <ip>:<port> (e.g., 127.0.0.1:7374)\n");
	printf("        Serial: <device>,<speed>,<8N1>[,<flow>]\n");
	printf("    --uecp-server <endpoint>\n");
	printf("        Enable UECP protocol server for RDS encoder control.\n");
	printf("        TCP: <ip>:<port> (e.g., 127.0.0.1:5001)\n");
	printf("        Serial: <device>,<speed>,<8N1>[,<flow>]\n");
	printf("    --asciig-server <endpoint>\n");
	printf("        Enable ASCII-G protocol server for RDS encoder control.\n");
	printf("        TCP: <ip>:<port> (e.g., 127.0.0.1:5002)\n");
	printf("        Serial: <device>,<speed>,<8N1>[,<flow>]\n");
	printf("    --rigctl-server <endpoint>\n");
	printf("        Enable hamlib-compatible rigctl server for remote control.\n");
	printf("        TCP: <ip>:<port> (e.g., 127.0.0.1:7356) or just <port>\n");
	printf("        Serial: <device>,<speed>,<8N1>[,<flow>]\n");
	printf("        Supports: f/F (freq), m/M (mode), l/L (levels), u/U (funcs)\n");
	printf("        Test with: rigctl -m 2 -r localhost:<port>\n");
	printf("    --rigctl-allowed-hosts <hosts>\n");
	printf("        Comma-separated list of allowed client IPs for rigctl.\n");
	printf("        If not set, all hosts are allowed.\n");
	printf("    --call-sign <WXXX>\n");
	printf("        Set RBDS Call Sign (e.g. WNYC) to automatically configure PI code.\n");
	printf("    --pi <HEX>\n");
	printf("        Set RDS PI code (hexadecimal), overriding preset or call sign.\n");
	printf("    --sca-67k\n");
	printf("        Enable 67 kHz SCA (Subsidiary Communications) subcarrier.\n");
	printf("    --sca-92k\n");
	printf("        Enable 92 kHz SCA (Subsidiary Communications) subcarrier.\n");
	printf("    --fast-math\n");
	printf("        Use fast math approximation for slow CPU / ARM based systems.\n");
	printf("    --channelizer-rate <rate>\n");
	printf("        IQ rate the SDR's channelizer delivers to the radio (Hz). Default: the lowest\n");
	printf("        SDR rate / 2^k that holds the signal, if the SDR rate is above the DSP rate.\n");
	printf("    --polyphase-resampler\n");
	printf("        Use polyphase FIR resampler for audio/signal rate conversion.\n");
	printf("        Enables bidirectional sample rate conversion with better quality.\n");
	printf("    --limesdr\n");
	printf("        Auto-select several required options for LimeSDR\n");
	printf("    --limesdr-mini\n");
	printf("        Auto-select several required options for LimeSDR Mini\n");
	printf("    --afc\n");
	printf("        Enable AFC (Automatic Frequency Control) for mono FM.\n");
	printf("        Corrects frequency offset using DC measurement after FM demod.\n");
	printf("    --afc-tc <ms>\n");
	printf("        AFC time constant in milliseconds (default 300).\n");
	printf("        Larger values = slower but more stable tracking.\n");
	printf("    --afc-max <Hz>\n");
	printf("        Maximum AFC correction in Hz (default 5000).\n");
	logging_print_help();
	sdr_config_print_help();
}

#define	OPT_FAST_MATH		1007
#define	OPT_RDS			1008
#define	OPT_RDS2		1009
#define	OPT_SCA_67K		1010
#define	OPT_SCA_92K		1011
#define	OPT_RDS_DEBUG		1012
#define	OPT_RDS_VERBOSE		1013
#define	OPT_CHANNELIZER		1014
#define	OPT_CHANNELIZER_RATE	1015
#define OPT_LIMESDR		1100
#define OPT_LIMESDR_MINI	1101
#define OPT_CALL_SIGN		1102
#define OPT_PI			1103
#define OPT_POLYPHASE		1104
#define OPT_RBDS		1105
#define OPT_RDS_PAGING		1106
#define OPT_RDS_HEXRDS_FILE	1107
#define OPT_RDS_BITSTREAM_FILE	1108
#define OPT_AFC			1118
#define OPT_AFC_TC		1119
#define OPT_AFC_MAX		1120
#define OPT_XDR_GTK_SERVER	1109
#define OPT_RDSSPY_SERVER	1110
#define OPT_UECP_SERVER		1111
#define OPT_ASCIIG_SERVER	1112
#define OPT_XDR_GTK_PASSWORD	1113
#define OPT_RIGCTL_SERVER	1114
#define OPT_RIGCTL_ALLOWED	1115
#define OPT_RDS_TX_HEXRDS_FILE	1116
#define OPT_RDS_TX_BITSTREAM_FILE 1117
#define OPT_RDS_TMC_EVENTS	1126
#define OPT_RDS_TMC_LOCATIONS	1127
#define OPT_RDS_PAGING_FIFO	1121
#define OPT_RDS_PAGING_CC	1122
#define OPT_RDS_PAGING_OPC	1123
#define OPT_RDS_PAGING_PAC	1124
#define OPT_FM_COMPANDOR	1125
#define OPT_RDS_PAGING_SCAN	1128
#define OPT_RDS_PAGING_SCAN_TYPE 1129
#define OPT_RDS_PAGING_BATTERY	1130
#define OPT_RDS_ECC		1131
#define OPT_RDS_PAGING_13A	1132

static void add_options(void)
{
	option_add('h', "help", 0);
	option_add('v', "verbose", 1);
	option_add('f', "frequency", 1);
	option_add('s', "samplerate", 1);
	option_add('r', "tx-wave-file", 1);
	option_add('w', "rx-wave-file", 1);
	option_add('a', "audio-device", 1);
	option_add('M', "modulation", 1);
	option_add('R', "rx", 0);
	option_add('T', "tx", 0);
	option_add('B', "bandwidth", 1);
	option_add('D', "deviation", 1);
	option_add('I', "modulation-index", 1);
	option_add('C', "compandor", 0);
	option_add(OPT_FM_COMPANDOR, "fm-compandor", 0);
	option_add('E', "emphasis", 1);
	option_add('V', "volume", 1);
	option_add('S', "stereo", 0);
	option_add(OPT_RDS, "rds", 0);
	option_add(OPT_RDS2, "rds2", 0);
	option_add(OPT_RDS_DEBUG, "rds-debug", 0);
	option_add(OPT_RDS_VERBOSE, "rds-verbose", 0);
	option_add(OPT_SCA_67K, "sca-67k", 0);
	option_add(OPT_SCA_92K, "sca-92k", 0);
	option_add(OPT_FAST_MATH, "fast-math", 0);
	option_add(OPT_CHANNELIZER_RATE, "channelizer-rate", 1);
	option_add(OPT_LIMESDR, "limesdr", 0);
	option_add(OPT_LIMESDR_MINI, "limesdr-mini", 0);
	option_add(OPT_CALL_SIGN, "call-sign", 1);
	option_add(OPT_PI, "pi", 1);
	option_add(OPT_POLYPHASE, "polyphase-resampler", 0);
	option_add(OPT_RBDS, "rbds", 0);
	option_add(OPT_RDS_PAGING, "rds-paging", 1);
	option_add(OPT_RDS_PAGING_FIFO, "rds-paging-fifo", 1);
	option_add(OPT_RDS_PAGING_CC, "rds-paging-cc", 1);
	option_add(OPT_RDS_PAGING_OPC, "rds-paging-opc", 1);
	option_add(OPT_RDS_PAGING_PAC, "rds-paging-pac", 1);
	option_add(OPT_RDS_PAGING_BATTERY, "rds-paging-battery-saving", 0);
	option_add(OPT_RDS_ECC, "rds-ecc", 1);
	option_add(OPT_RDS_PAGING_13A, "rds-paging-13a", 0);
	option_add(OPT_RDS_PAGING_SCAN, "rds-paging-scan", 2);
	option_add(OPT_RDS_PAGING_SCAN_TYPE, "rds-paging-scan-type", 1);
	option_add(OPT_RDS_HEXRDS_FILE, "rds-hexrds-file", 1);
	option_add(OPT_RDS_BITSTREAM_FILE, "rds-bitstream-file", 1);
	option_add(OPT_RDS_TX_HEXRDS_FILE, "rds-tx-hexrds-file", 1);
	option_add(OPT_RDS_TX_BITSTREAM_FILE, "rds-tx-bitstream-file", 1);
	option_add(OPT_RDS_TMC_EVENTS, "rds-tmc-events", 1);
	option_add(OPT_RDS_TMC_LOCATIONS, "rds-tmc-locations", 1);
	option_add(OPT_XDR_GTK_SERVER, "xdr-gtk-server", 1);
	option_add(OPT_RDSSPY_SERVER, "rdsspy-server", 1);
	option_add(OPT_UECP_SERVER, "uecp-server", 1);
	option_add(OPT_ASCIIG_SERVER, "asciig-server", 1);
	option_add(OPT_XDR_GTK_PASSWORD, "xdr-gtk-password", 1);
	option_add(OPT_RIGCTL_SERVER, "rigctl-server", 1);
	option_add(OPT_RIGCTL_ALLOWED, "rigctl-allowed-hosts", 1);
	option_add(OPT_AFC, "afc", 0);
	option_add(OPT_AFC_TC, "afc-tc", 1);
	option_add(OPT_AFC_MAX, "afc-max", 1);
        sdr_config_add_options();
}

static int handle_options(int short_option, int argi, char **argv)
{
	int rc;

	switch (short_option) {
	case 'h':
		print_help(argv[0]);
		return 0;
	case 'v':
		rc = parse_logging_opt(argv[argi]);
		if (rc > 0)
			return 0;
		if (rc < 0)
			return rc;
		break;
	case 'f':
		/* in the config file: the default frequency */
		if (options_from_command_line) {
			if (num_stations) {
				fprintf(stderr, "A second '-f' (several stations) is not supported yet, one '-f' per osmoradio!\n");
				return -EINVAL;
			}
			station0 = station_default;
			st = &station0;
			num_stations = 1;
		}
		st->frequency = atof(argv[argi]);
		break;
	case 's':
		st->dsp_samplerate = atof(argv[argi]);
		break;
	case 'r':
		st->tx_wave_file = options_strdup(argv[argi]);
		break;
	case 'w':
		st->rx_wave_file = options_strdup(argv[argi]);
		break;
	case 'a':
		st->tx_audiodev = options_strdup(argv[argi]);
		st->rx_audiodev = options_strdup(argv[argi]);
		break;
	case 'M':
		if (!strcasecmp(argv[argi], "fm"))
			st->modulation = MODULATION_FM;
		else
		if (!strcasecmp(argv[argi], "am"))
			st->modulation = MODULATION_AM_DSB;
		else
		if (!strcasecmp(argv[argi], "usb"))
			st->modulation = MODULATION_AM_USB;
		else
		if (!strcasecmp(argv[argi], "lsb"))
			st->modulation = MODULATION_AM_LSB;
		else
		{
			fprintf(stderr, "Invalid modulation option, use '-h' for help!\n");
			return -EINVAL;
		}
		break;
	case 'R':
		st->rx = 1;
		break;
	case 'T':
		st->tx = 1;
		break;
	case 'B':
		st->bandwidth = atof(argv[argi]);
		break;
	case 'D':
		st->deviation = atof(argv[argi]);
		break;
	case 'I':
		st->modulation_index = atof(argv[argi]);
		if (st->modulation_index < 0.0 || st->modulation_index > 1.0) {
			fprintf(stderr, "Invalid modulation index, use '-h' for help!\n");
			return -EINVAL;
		}
		break;
	case 'C':
		st->am_compandor = 1;
		break;
	case OPT_FM_COMPANDOR:
		st->fm_compandor = 1;
		break;
	case 'E':
		st->time_constant_us = atof(argv[argi]);
		break;
	case 'V':
		st->volume = atof(argv[argi]);
		break;
	case 'S':
		st->stereo = 1;
		break;
	case OPT_RDS:
		st->rds = 1;
		break;
	case OPT_RDS2:
		st->rds2 = 1;
		break;
	case OPT_RDS_DEBUG:
		st->rds_debug = 1;
		break;
	case OPT_RDS_VERBOSE:
		st->rds_verbose = 1;
		break;
	case OPT_RBDS:
		st->rds_force_rbds = 1;
		break;
	case OPT_SCA_67K:
		st->sca_67k = 1;
		break;
	case OPT_SCA_92K:
		st->sca_92k = 1;
		break;
	case OPT_FAST_MATH:
		fast_math = 1;
		break;
	case OPT_CHANNELIZER_RATE:
		st->channelizer_rate = atoi(argv[argi]);
		break;
	case OPT_CALL_SIGN:
		st->radio_opts.rds_callsign = options_strdup(argv[argi]);
		break;
	case OPT_PI:
		st->rds_pi_opt = (uint16_t)strtoul(argv[argi], NULL, 16);
		st->radio_opts.rds_pi = st->rds_pi_opt;
		break;
	case OPT_POLYPHASE:
		st->use_polyphase = 1;
		break;
	case OPT_RDS_PAGING:
		st->rds_paging = 1;
		st->rds = 1;  /* auto-enable RDS */
		{
			int rpc_val = atoi(argv[argi]);
			if (rpc_val < 0 || rpc_val > 31) {
				fprintf(stderr, "Invalid RPC value %d (must be 0-31), use '-h' for help!\n", rpc_val);
				return -EINVAL;
			}
			st->rds_paging_rpc = rpc_val;
		}
		break;
	case OPT_RDS_PAGING_FIFO:
		st->rds_paging_pipe_path = argv[argi];
		break;
	case OPT_RDS_PAGING_CC:
		st->rds_paging_cc = (uint16_t)atoi(argv[argi]);
		if (!mcc_valid(st->rds_paging_cc)) {
			const char *cname = mcc_name(st->rds_paging_cc);
			if (!cname) {
				fprintf(stderr, "Invalid E.212 MCC %d (must be 200-999), use '-h' for help!\n", st->rds_paging_cc);
				return -EINVAL;
			}
		}
		{
			const char *cname = mcc_name(st->rds_paging_cc);
			fprintf(stderr, "RDS Paging: international MCC=%d (%s)\n",
				st->rds_paging_cc, cname ? cname : "unknown");
		}
		break;
	case OPT_RDS_PAGING_OPC:
		st->rds_paging_opc = (uint8_t)atoi(argv[argi]);
		if (st->rds_paging_opc > 15) {
			fprintf(stderr, "Invalid OPC %d (must be 0-15), use '-h' for help!\n", st->rds_paging_opc);
			return -EINVAL;
		}
		break;
	case OPT_RDS_PAGING_PAC:
		st->rds_paging_pac = (uint8_t)atoi(argv[argi]);
		if (st->rds_paging_pac > 63) {
			fprintf(stderr, "Invalid PAC %d (must be 0-63), use '-h' for help!\n", st->rds_paging_pac);
			return -EINVAL;
		}
		break;
	case OPT_RDS_PAGING_BATTERY:
		st->rds_paging_battery = 1;
		break;
	case OPT_RDS_PAGING_13A:
		st->rds_paging_13a = 1;
		break;
	case OPT_RDS_ECC: {
		const char *a = argv[argi];
		char *end;
		unsigned long v;

		if (!strcasecmp(a, "list")) {
			rds_ecc_table_print(stdout);
			exit(0);
		}
		v = strtoul(a, &end, 16);
		if (*a == '\0' || *end != '\0' || v < 0x01 || v > 0xff) {
			fprintf(stderr, "Invalid ECC '%s' (hex 01-FF or 'list'), use '-h' for help!\n", a);
			return -EINVAL;
		}
		st->rds_ecc = (uint8_t)v;
		st->rds = 1;  /* auto-enable RDS */
		break;
	}
	case OPT_RDS_PAGING_SCAN: {
		long from = atol(argv[argi++]);
		long to = atol(argv[argi++]);

		if (from < 0 || to < from || to > RDS_PAGING_ADDR_MAX) {
			fprintf(stderr, "Invalid paging scan range %ld-%ld (must be 0-%d, from <= to), use '-h' for help!\n",
				from, to, RDS_PAGING_ADDR_MAX);
			return -EINVAL;
		}
		st->rds_paging_scan = 1;
		st->rds_paging_scan_from = from;
		st->rds_paging_scan_to = to;
		st->radio_opts.rds_paging_preset = 1;
		break;
	}
	case OPT_RDS_PAGING_SCAN_TYPE:
		if (!strcasecmp(argv[argi], "numeric"))
			st->rds_paging_scan_type = RDS_PAGING_NUM10;
		else if (!strcasecmp(argv[argi], "alpha"))
			st->rds_paging_scan_type = RDS_PAGING_ALPHA;
		else if (!strcasecmp(argv[argi], "tone"))
			st->rds_paging_scan_type = RDS_PAGING_TONE;
		else {
			fprintf(stderr, "Invalid paging scan type '%s' (numeric, alpha or tone), use '-h' for help!\n", argv[argi]);
			return -EINVAL;
		}
		break;
	case OPT_RDS_HEXRDS_FILE:
		st->rds_hexrds_file = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_RDS_BITSTREAM_FILE:
		st->rds_bitstream_file = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_RDS_TX_HEXRDS_FILE:
		st->rds_tx_hexrds_file = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_RDS_TX_BITSTREAM_FILE:
		st->rds_tx_bitstream_file = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_RDS_TMC_EVENTS:
		st->rds_tmc_events_file = options_strdup(argv[argi]);
		break;
	case OPT_RDS_TMC_LOCATIONS:
		st->rds_tmc_locations_path = options_strdup(argv[argi]);
		break;
	case OPT_XDR_GTK_SERVER:
		st->xdr_gtk_server = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_XDR_GTK_PASSWORD:
		st->xdr_gtk_password = options_strdup(argv[argi]);
		break;
	case OPT_RIGCTL_SERVER:
		st->rigctl_server_endpoint = options_strdup(argv[argi]);
		break;
	case OPT_RIGCTL_ALLOWED:
		st->rigctl_allowed_hosts = options_strdup(argv[argi]);
		break;
	case OPT_RDSSPY_SERVER:
		st->rdsspy_server = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_UECP_SERVER:
		st->uecp_server = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_ASCIIG_SERVER:
		st->asciig_server = options_strdup(argv[argi]);
		st->rds = 1;  /* auto-enable RDS */
		break;
	case OPT_LIMESDR:
		{
			char *argv_lime[] = { argv[0],
				"--sdr-soapy",
				"--sdr-device-args", "driver=lime",
				"--sdr-rx-antenna", "LNAL",
				"--sdr-rx-gain", "50",
				"--sdr-tx-gain", "50",
				"--sdr-samplerate", "5000000",
				"--sdr-bandwidth", "15000000",
			};
			int argc_lime = sizeof(argv_lime) / sizeof (*argv_lime);
			return options_command_line(argc_lime, argv_lime, handle_options);
		}
	case OPT_LIMESDR_MINI:
		{
			char *argv_lime[] = { argv[0],
				"--sdr-soapy",
				"--sdr-device-args", "driver=lime",
				"--sdr-rx-antenna", "LNAW",
				"--sdr-tx-antenna", "BAND2",
				"--sdr-rx-gain", "50",
				"--sdr-tx-gain", "50",
				"--sdr-samplerate", "5000000",
				"--sdr-bandwidth", "15000000",
			};
			int argc_lime = sizeof(argv_lime) / sizeof (*argv_lime);
			return options_command_line(argc_lime, argv_lime, handle_options);
		}
	case OPT_AFC:
		st->use_afc = 1;
		break;
	case OPT_AFC_TC:
		st->afc_tc_ms = atof(argv[argi]);
		if (st->afc_tc_ms < 10.0) st->afc_tc_ms = 10.0;
		if (st->afc_tc_ms > 10000.0) st->afc_tc_ms = 10000.0;
		break;
	case OPT_AFC_MAX:
		st->afc_max_hz = atof(argv[argi]);
		if (st->afc_max_hz < 100.0) st->afc_max_hz = 100.0;
		if (st->afc_max_hz > 50000.0) st->afc_max_hz = 50000.0;
		break;
	default:
		return sdr_config_handle_options(short_option, argi, argv);
	}

	return 1;
}

/* Decode <0xCC> hex escapes in a message string in-place.
 * Returns new length. E.g. "AB<0x0D>CD" -> "AB\rCD" (len=5).
 * Invalid escapes are passed through verbatim. */
static int paging_decode_hex_escapes(char *msg, int len)
{
	int r = 0, w = 0;

	while (r < len) {
		if (r + 5 < len &&
		    msg[r] == '<' && msg[r+1] == '0' && msg[r+2] == 'x' &&
		    msg[r+5] == '>') {
			/* Try to parse 2 hex digits */
			char hex[3] = { msg[r+3], msg[r+4], '\0' };
			char *end;
			unsigned long val = strtoul(hex, &end, 16);
			if (end == hex + 2) {
				msg[w++] = (char)(uint8_t)val;
				r += 6;
				continue;
			}
		}
		msg[w++] = msg[r++];
	}
	msg[w] = '\0';
	return w;
}

/* Parse space-separated key=value options for paging FIFO.
 * Supported: repeat=N interval=N tonetype=N ni=0|1 cc=NNN opc=N */
static void paging_parse_options(const char *opts, int opts_len,
				 int *repeat, int *interval,
				 uint8_t *tonetype, uint8_t *ni,
				 uint16_t *country_code, uint8_t *opc)
{
	char buf[256];
	char *p, *token, *saveptr;

	*repeat = 0;
	*interval = 0;
	*tonetype = 0;
	*ni = 0;
	*country_code = 0;
	*opc = 0;

	if (!opts || opts_len <= 0)
		return;

	if (opts_len >= (int)sizeof(buf))
		opts_len = sizeof(buf) - 1;
	memcpy(buf, opts, opts_len);
	buf[opts_len] = '\0';

	for (token = strtok_r(buf, " \t", &saveptr);
	     token;
	     token = strtok_r(NULL, " \t", &saveptr)) {
		if ((p = strchr(token, '=')) != NULL) {
			*p++ = '\0';
			if (!strcasecmp(token, "repeat"))
				*repeat = atoi(p);
			else if (!strcasecmp(token, "interval"))
				*interval = atoi(p);
			else if (!strcasecmp(token, "tonetype"))
				*tonetype = (uint8_t)atoi(p);
			else if (!strcasecmp(token, "ni"))
				*ni = (uint8_t)atoi(p);
			else if (!strcasecmp(token, "cc"))
				*country_code = (uint16_t)atoi(p);
			else if (!strcasecmp(token, "opc"))
				*opc = (uint8_t)atoi(p);
		}
	}
}

/* Process a single paging command line.
 * Format: address,type,options,message  (always 3 commas)
 * Types: tone, numeric, numeric10, numeric18, alpha
 * Options: space-separated key=value (can be empty):
 *   repeat=N      total send count (1-15, default 1)
 *   interval=N    seconds between repeats (default 5)
 *   tonetype=N    E2-E0 for tone-only (0-7, default 0)
 *   ni=0|1        national/international (default 0)
 *   cc=NNN        E.212 MCC for international (e.g. 255=Ukraine)
 *   opc=N         Operator Code 1-15 for international
 * Message: everything after 3rd comma (commas in payload preserved) */
static void paging_process_line(rds_encoder_t *enc, char *line)
{
	int i, comma_count = 0;
	int comma1 = -1, comma2 = -1, comma3 = -1;
	int len = strlen(line);
	char addr_buf[16], type_buf[32];
	int repeat, interval;
	uint8_t tonetype, ni;
	uint16_t country_code;
	uint8_t opc;
	uint32_t address;
	char *msg;
	int msg_len;

	/* Find exactly 3 commas */
	for (i = 0; i < len; i++) {
		if (line[i] == ',') {
			comma_count++;
			if (comma_count == 1) comma1 = i;
			else if (comma_count == 2) comma2 = i;
			else if (comma_count == 3) { comma3 = i; break; }
		}
	}

	if (comma_count < 3) {
		LOGP(DRADIO, LOGL_NOTICE,
		     "RDS paging FIFO: format is address,type,options,message\n"
		     "  address: 0-999999\n"
		     "  types:   tone|numeric|numeric10|numeric18|alpha|vnum|functions|intl15|intlfunc\n"
		     "  options: repeat=N interval=N tonetype=N ni=0|1 cc=NNN opc=N\n"
		     "  message: payload after 3rd comma (commas allowed)\n"
		     "  examples:\n"
		     "    200078,tone,,\n"
		     "    200078,tone,tonetype=3 repeat=2,\n"
		     "    200078,numeric,,1234567890\n"
		     "    200078,alpha,,Hello World\n"
		     "    200078,alpha,repeat=3 interval=5,Hello, World!\n"
		     "    200078,alpha,ni=1 cc=255 opc=1,International msg\n"
		     "    200078,vnum,,31415926535\n");
		return;
	}

	/* Extract address */
	if (comma1 >= (int)sizeof(addr_buf)) comma1 = sizeof(addr_buf) - 1;
	memcpy(addr_buf, line, comma1);
	addr_buf[comma1] = '\0';
	address = (uint32_t)atoi(addr_buf);
	if (address > RDS_PAGING_ADDR_MAX) {
		LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: address %u out of range (max %d)\n",
		     address, RDS_PAGING_ADDR_MAX);
		return;
	}

	/* Extract type */
	{
		int tlen = comma2 - comma1 - 1;
		if (tlen >= (int)sizeof(type_buf)) tlen = sizeof(type_buf) - 1;
		memcpy(type_buf, line + comma1 + 1, tlen);
		type_buf[tlen] = '\0';
	}

	/* Parse options */
	paging_parse_options(line + comma2 + 1, comma3 - comma2 - 1,
			     &repeat, &interval, &tonetype, &ni,
			     &country_code, &opc);

	/* International mode: fill in defaults from encoder config if not
	 * specified per-message. ni=1 requires cc and opc. */
	if (ni) {
		if (!country_code)
			country_code = enc->paging.intl_country_code;
		if (!opc)
			opc = enc->paging.intl_opc;
		if (!country_code || !opc) {
			LOGP(DRADIO, LOGL_ERROR,
			     "RDS paging FIFO: ni=1 requires cc=NNN and opc=N "
			     "(per-message or --rds-paging-cc/--rds-paging-opc)\n");
			return;
		}
		if (!mcc_valid(country_code)) {
			LOGP(DRADIO, LOGL_ERROR,
			     "RDS paging FIFO: invalid MCC %d (must be 200-999)\n",
			     country_code);
			return;
		}
	}

	/* Message is everything after 3rd comma */
	msg = line + comma3 + 1;
	msg_len = len - comma3 - 1;

	/* Decode <0xCC> hex escapes in message (in-place) */
	msg_len = paging_decode_hex_escapes(msg, msg_len);

	if (!strcasecmp(type_buf, "tone")) {
		if (ni) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: tone-only has no NI bit (EN 50067 M.3.5.2)\n");
			return;
		}
		if (rds_enc_paging_send_tone(enc, address, repeat, interval, tonetype) < 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: TONE addr=%02u-%04u rejected, nothing sent\n",
			     address / 10000, address % 10000);
			return;
		}
		LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: TONE addr=%02u-%04u repeat=%d interval=%d tonetype=%d\n",
		     address / 10000, address % 10000, repeat, interval, tonetype);
	} else if (!strcasecmp(type_buf, "numeric") || !strcasecmp(type_buf, "numeric10") || !strcasecmp(type_buf, "numeric18")) {
		if (msg_len <= 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: numeric requires message digits\n");
			return;
		}
		if (ni) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: basic numeric (10/18) has no NI bit; use vnum for international numeric\n");
			return;
		}
		if (rds_enc_paging_send_numeric(enc, address, msg, repeat, interval, ni) < 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: NUMERIC addr=%02u-%04u len=%d rejected, nothing sent\n",
			     address / 10000, address % 10000, msg_len);
			return;
		}
		LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: NUMERIC addr=%02u-%04u len=%d repeat=%d interval=%d msg=\"%s\"\n",
		     address / 10000, address % 10000, msg_len, repeat, interval, msg);
	} else if (!strcasecmp(type_buf, "alpha")) {
		if (msg_len <= 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: alpha requires message text\n");
			return;
		}
		if (rds_enc_paging_send_alpha(enc, address, msg, repeat, interval, ni, country_code, opc) < 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: ALPHA addr=%02u-%04u len=%d rejected, nothing sent\n",
			     address / 10000, address % 10000, msg_len);
			return;
		}
		if (ni) {
			const char *cname = mcc_name(country_code);
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: ALPHA addr=%02u-%04u len=%d repeat=%d interval=%d intl MCC=%d(%s) OPC=%d msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval,
			     country_code, cname ? cname : "?", opc, msg);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: ALPHA addr=%02u-%04u len=%d repeat=%d interval=%d nat msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval, msg);
		}
	} else if (!strcasecmp(type_buf, "vnum")) {
		if (msg_len <= 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: vnum requires message digits\n");
			return;
		}
		if (rds_enc_paging_send_vnum(enc, address, msg, repeat, interval, ni, country_code, opc) < 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: VNUM addr=%02u-%04u len=%d rejected, nothing sent\n",
			     address / 10000, address % 10000, msg_len);
			return;
		}
		if (ni) {
			const char *cname = mcc_name(country_code);
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: VNUM addr=%02u-%04u len=%d repeat=%d interval=%d intl MCC=%d(%s) OPC=%d msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval,
			     country_code, cname ? cname : "?", opc, msg);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: VNUM addr=%02u-%04u len=%d repeat=%d interval=%d nat msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval, msg);
		}
	} else if (!strcasecmp(type_buf, "functions") || !strcasecmp(type_buf, "func")) {
		if (msg_len <= 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: functions requires hex digits\n");
			return;
		}
		if (rds_enc_paging_send_functions(enc, address, msg, repeat, interval, ni, country_code, opc) < 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: FUNC addr=%02u-%04u len=%d rejected, nothing sent\n",
			     address / 10000, address % 10000, msg_len);
			return;
		}
		if (ni) {
			const char *cname = mcc_name(country_code);
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: FUNC addr=%02u-%04u len=%d repeat=%d interval=%d intl MCC=%d(%s) OPC=%d msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval,
			     country_code, cname ? cname : "?", opc, msg);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: FUNC addr=%02u-%04u len=%d repeat=%d interval=%d nat msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval, msg);
		}
	} else if (!strcasecmp(type_buf, "intl15")) {
		if (msg_len <= 0 || !country_code) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: intl15 requires digits and cc=\n");
			return;
		}
		if (rds_enc_paging_send_intl15(enc, address, msg, repeat, interval, country_code) < 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: INTL15 addr=%02u-%04u len=%d rejected, nothing sent\n",
			     address / 10000, address % 10000, msg_len);
			return;
		}
		LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: INTL15 addr=%02u-%04u len=%d MCC=%d msg=\"%s\"\n",
		     address / 10000, address % 10000, msg_len, country_code, msg);
	} else if (!strcasecmp(type_buf, "intlfunc")) {
		if (msg_len <= 0 || !country_code) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: intlfunc requires hex digits and cc=\n");
			return;
		}
		if (rds_enc_paging_send_intlfunc(enc, address, msg, repeat, interval, country_code) < 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: INTLFUNC addr=%02u-%04u len=%d rejected, nothing sent\n",
			     address / 10000, address % 10000, msg_len);
			return;
		}
		LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: INTLFUNC addr=%02u-%04u len=%d MCC=%d msg=\"%s\"\n",
		     address / 10000, address % 10000, msg_len, country_code, msg);
	} else {
		LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: unknown type '%s'\n", type_buf);
	}
}

static void paging_pipe_handler(station_t *s)
{
	rds_encoder_t *enc = &s->radio.rds_enc;
	char *buf = s->paging_buf;
	int buf_len = s->paging_buf_len;
	int rc, i, start;
	int space = sizeof(s->paging_buf) - buf_len;

	if (s->paging_pipe_fd < 0)
		return;

	rc = read(s->paging_pipe_fd, buf + buf_len, space);
	if (rc > 0) {
		buf_len += rc;

		/* Overflow handling */
		if (buf_len == (int)sizeof(s->paging_buf)) {
			int has_newline = 0;
			for (i = 0; i < buf_len; i++) {
				if (buf[i] == '\r' || buf[i] == '\n') {
					has_newline = 1;
					break;
				}
			}
			if (!has_newline) {
				LOGP(DRADIO, LOGL_ERROR, "RDS paging pipe: buffer overflow, discarding!\n");
				s->paging_buf_len = 0;
				return;
			}
		}

		/* Process all complete lines */
		start = 0;
		for (i = 0; i < buf_len; i++) {
			if (buf[i] == '\r' || buf[i] == '\n') {
				buf[i] = '\0';
				if (i > start)
					paging_process_line(enc, buf + start);
				start = i + 1;
			}
		}

		/* Retain any partial line */
		if (start > 0 && start < buf_len) {
			memmove(buf, buf + start, buf_len - start);
			buf_len -= start;
		} else if (start >= buf_len) {
			buf_len = 0;
		}
		s->paging_buf_len = buf_len;
	}
}

/* Start a station after the options and the SDR configuration: its radio,
 * meter, RDS files, paging, protocol servers and buffer. s->iq_rate is set.
 * The SDR is not open yet (sdr = NULL). Returns < 0 on error (the caller
 * calls station_exit()); bad options still exit as before. */
static int station_init(station_t *s, void *sdr)
{
	int rc;

	/* now we have buffer size and sample rate
	 * buffer size is proportional to INPUT sample rate (SDR rate)
	 */
	s->buffer_size = s->iq_rate * dsp_buffer / 1000;

	s->radio_opts.polyphase = s->use_polyphase;

	rc = radio_init(&s->radio, s->buffer_size, s->iq_rate, s->frequency, s->tx_wave_file, s->rx_wave_file, (s->tx) ? s->tx_audiodev : NULL, (s->rx) ? s->rx_audiodev : NULL, s->modulation, s->bandwidth, s->deviation, s->modulation_index, s->time_constant_us, s->volume, s->stereo, s->rds, s->rds2, s->sca_67k, s->sca_92k, s->rds_debug, s->rds_verbose, s->am_compandor, s->fm_compandor, s->rds_force_rbds, &s->radio_opts);
	if (rc < 0) {
		fprintf(stderr, "Failed to initialize radio with given options, exitting!\n");
		exit(0);
	}

	/* Configure AFC if enabled */
	if (s->use_afc && s->modulation == MODULATION_FM) {
		radio_afc_set_time_constant(&s->radio, s->afc_tc_ms / 1000.0);
		radio_afc_set_max_correction(&s->radio, s->afc_max_hz);
		radio_afc_enable(&s->radio, 1);
	}

	/* Initialize signal meter (FFT-based, same approach as scan engine).
	 * Uses 4096-point FFT with ±20 kHz integration and processing gain.
	 * Output: SNR in dB (signal - noise).
	 * The dbf_offset is a fallback before noise floor is established. */
	s->meter = signal_meter_init(s->iq_rate, 50.0);
	if (s->meter)
		signal_meter_register_thread(s->meter, "main", gettid());

	/* NOTE: spectral frequency registration moved after sdr_open_channelizer()
	 * because sdr_status_init() inside sdr_open zeroes the entire sdr_status
	 * struct, wiping any previously registered frequencies. */

	/* Set up RDS file output if requested */
	if (s->rds_hexrds_file) {
		rc = rds_decoder_set_hexrds_file(&s->radio.rds_dec, s->rds_hexrds_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open hexrds output file, exitting!\n");
			exit(0);
		}
	}
	if (s->rds_bitstream_file) {
		rc = rds_decoder_set_bitstream_file(&s->radio.rds_dec, s->rds_bitstream_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open bitstream output file, exitting!\n");
			exit(0);
		}
	}

	/* Load optional TMC event description table (docs/rds/tmc_events.csv).
	 * When loaded, the RDS 8A decoder logs human-readable event names. */
	if (s->rds_tmc_events_file) {
		if (rds_tmc_load_events(s->rds_tmc_events_file) < 0)
			fprintf(stderr, "Warning: failed to load TMC events from '%s'\n",
			        s->rds_tmc_events_file);
	}

	/* Load optional TMC location table (ISO 14819-3 LCL directory or CSV).
	 * When loaded, the RDS 8A decoder logs human-readable road names. */
	if (s->rds_tmc_locations_path) {
		if (rds_tmc_load_locations(s->rds_tmc_locations_path) < 0)
			fprintf(stderr, "Warning: failed to load TMC locations from '%s'\n",
			        s->rds_tmc_locations_path);
	}

	/* Set up RDS TX file input if requested (bypasses encoder) */
	if (s->rds_tx_hexrds_file && s->rds_tx_bitstream_file) {
		fprintf(stderr, "Cannot use both --rds-tx-hexrds-file and --rds-tx-bitstream-file\n");
		exit(0);
	}
	if (s->rds_tx_hexrds_file) {
		if (!s->tx) {
			fprintf(stderr, "Warning: --rds-tx-hexrds-file requires --tx mode\n");
		}
		rc = rds_enc_set_tx_hexrds_file(&s->radio.rds_enc, s->rds_tx_hexrds_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open TX hexrds input file, exitting!\n");
			exit(0);
		}
	}
	if (s->rds_tx_bitstream_file) {
		if (!s->tx) {
			fprintf(stderr, "Warning: --rds-tx-bitstream-file requires --tx mode\n");
		}
		rc = rds_enc_set_tx_bitstream_file(&s->radio.rds_enc, s->rds_tx_bitstream_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open TX bitstream input file, exitting!\n");
			exit(0);
		}
	}

	/* Enable RDS paging if requested */
	if (s->rds_paging) {
		rds_enc_paging_set_enabled(&s->radio.rds_enc, 1);
		rds_enc_paging_set_rpc(&s->radio.rds_enc, s->rds_paging_rpc);
		rds_enc_paging_set_opc(&s->radio.rds_enc, s->rds_paging_opc);
		rds_enc_paging_set_pac(&s->radio.rds_enc, s->rds_paging_pac);
		/* International paging defaults (EN 50067 M.3.5.7) */
		rds_enc_paging_set_cc(&s->radio.rds_enc, s->rds_paging_cc);
		if (s->rds_paging_battery)
			rds_enc_paging_set_battery_saving(&s->radio.rds_enc, 1);
		if (s->rds_paging_13a)
			rds_enc_paging_set_13a_notify(&s->radio.rds_enc, 1);
		/* Enable paging decoder so 7A/13A are decoded as paging, not ODA */
		s->radio.rds_dec.paging_enabled = 1;
		/* Create named pipe for paging commands */
		unlink(s->rds_paging_pipe_path);
		rc = mkfifo(s->rds_paging_pipe_path, 0666);
		if (rc < 0) {
			fprintf(stderr, "Failed to create paging FIFO '%s': %s\n",
				s->rds_paging_pipe_path, strerror(errno));
		} else {
			s->paging_pipe_made = 1;
			s->paging_pipe_fd = open(s->rds_paging_pipe_path, O_RDWR | O_NONBLOCK);
			if (s->paging_pipe_fd < 0)
				fprintf(stderr, "Failed to open paging FIFO '%s': %s\n",
					s->rds_paging_pipe_path, strerror(errno));
			else {
				char rpc_buf[80];
				printf("RDS Paging enabled: %s, OPC=%d%s, PAC=%d%s",
				       rds_paging_rpc_desc(s->rds_paging_rpc, rpc_buf, sizeof(rpc_buf)),
				       s->rds_paging_opc,
				       s->rds_paging_opc ? "" : " (no enhanced paging)",
				       s->rds_paging_pac,
				       s->rds_paging_pac ? "" : " (all areas)");
				if (s->rds_paging_cc) {
					const char *cname = mcc_name(s->rds_paging_cc);
					printf(", intl MCC=%d (%s)",
					       s->rds_paging_cc, cname ? cname : "unknown");
				}
				printf(", pipe: %s\n", s->rds_paging_pipe_path);
			}
		}
	}

	if (s->rds_paging_scan) {
		if (!s->rds_paging) {
			fprintf(stderr, "--rds-paging-scan requires --rds-paging, use '-h' for help!\n");
			exit(0);
		}
		if (!s->tx) {
			fprintf(stderr, "--rds-paging-scan requires --tx, use '-h' for help!\n");
			exit(0);
		}
		rc = rds_enc_paging_scan_start(&s->radio.rds_enc, s->rds_paging_scan_from, s->rds_paging_scan_to,
					       (enum rds_paging_msg_type)s->rds_paging_scan_type);
		if (rc < 0) {
			fprintf(stderr, "Failed to start paging scan\n");
			exit(0);
		}
		printf("RDS Paging scan: addresses %06u-%06u\n",
		       (unsigned)s->rds_paging_scan_from, (unsigned)s->rds_paging_scan_to);
	}

	/* the servers' callbacks find their station here */
	s->rds_cb_ctx.station = s;

	/* Initialize protocol servers */
	memset(&s->xdr_srv, 0, sizeof(s->xdr_srv));
	memset(&s->rdsspy_srv, 0, sizeof(s->rdsspy_srv));
	memset(&s->uecp_srv, 0, sizeof(s->uecp_srv));
	memset(&s->asciig_srv, 0, sizeof(s->asciig_srv));
	memset(&s->rigctl_srv, 0, sizeof(s->rigctl_srv));

	/* RX-only servers: XDR-GTK and RDS-Spy */
	if (s->xdr_gtk_server) {
		if (!s->rx)
			fprintf(stderr, "Warning: --xdr-gtk-server is for RX mode (use with --rx)\n");
		rc = rds_server_init(&s->xdr_srv, s->xdr_gtk_server, RDS_PROTO_XDR_GTK, NULL);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize XDR-GTK server\n");
			return -1;
		}
		if (s->xdr_gtk_password)
			rds_server_set_password(&s->xdr_srv, s->xdr_gtk_password);
		s->xdr_srv.freq_khz = (int)(s->frequency / 1000.0);
		/* Tell XDR-GTK our de-emphasis setting: 0=50µs (EU), 1=75µs (US) */
		if (s->time_constant_us == 50.0)
			rds_server_set_deemphasis(&s->xdr_srv, 0);
		else if (s->time_constant_us == 75.0)
			rds_server_set_deemphasis(&s->xdr_srv, 1);
		s->xdr_enabled = 1;
		printf("XDR-GTK protocol server enabled: %s%s\n",
		       s->xdr_gtk_server, s->xdr_gtk_password ? " (password protected)" : "");
	}

	if (s->rdsspy_server) {
		if (!s->rx)
			fprintf(stderr, "Warning: --rdsspy-server is for RX mode (use with --rx)\n");
		rc = rds_server_init(&s->rdsspy_srv, s->rdsspy_server, RDS_PROTO_RDSSPY, NULL);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize RDS Spy server\n");
			return -1;
		}
		s->rdsspy_enabled = 1;
		printf("RDS Spy protocol server enabled: %s\n", s->rdsspy_server);
	}

	/* TX-only servers: UECP and ASCII-G */
	if (s->uecp_server) {
		if (!s->tx)
			fprintf(stderr, "Warning: --uecp-server is for TX mode (use with --tx)\n");
		rc = rds_server_init(&s->uecp_srv, s->uecp_server, RDS_PROTO_UECP, &s->radio.rds_enc);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize UECP server\n");
			return -1;
		}
		s->uecp_enabled = 1;
		printf("UECP protocol server enabled: %s\n", s->uecp_server);
	}

	if (s->asciig_server) {
		if (!s->tx)
			fprintf(stderr, "Warning: --asciig-server is for TX mode (use with --tx)\n");
		rc = rds_server_init(&s->asciig_srv, s->asciig_server, RDS_PROTO_ASCII_G, &s->radio.rds_enc);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize ASCII-G server\n");
			return -1;
		}
		s->asciig_srv.freq_khz = (int)(s->frequency / 1000.0);
		s->asciig_enabled = 1;
		printf("ASCII-G protocol server enabled: %s\n", s->asciig_server);
	}

	/* Rigctl server (hamlib-compatible remote control) */
	if (s->rigctl_server_endpoint) {
		rc = rigctl_server_init(&s->rigctl_srv, s->rigctl_server_endpoint, s->rigctl_allowed_hosts);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize rigctl server\n");
			return -1;
		}
		rigctl_server_set_mode_flags(&s->rigctl_srv, s->tx, s->rx);
		rigctl_server_set_frequency(&s->rigctl_srv, s->frequency, s->frequency);
		rigctl_server_set_modulation(&s->rigctl_srv, s->modulation, (int)s->radio.rf_bandwidth, s->deviation, s->modulation_index);
		rigctl_server_set_flags(&s->rigctl_srv, s->stereo, s->rds);
		rigctl_server_set_radio(&s->rigctl_srv, &s->radio);
		rigctl_server_set_meter(&s->rigctl_srv, s->meter);

		/* Query and set SDR capabilities
		 *
		 * TODO: Currently sdr_query_caps() returns cached/configured values.
		 *       Should query actual hardware frequency ranges at startup via:
		 *       - soapy_query_freq_range() for SoapySDR devices
		 *       - uhd_query_freq_range() for UHD devices
		 *       Also need to update stored ranges on retune if SDR has tunable
		 *       front-end filters. Account for upconverter offsets which
		 *       may differ for RX and TX directions (e.g., transverter).
		 *       Hamlib supports separate rx_range_list and tx_range_list.
		 *       IMPORTANT: Never query hardware from subroutines! Query once
		 *       during SDR setup/startup/retune and store results for later use. */
		if (sdr) {
			sdr_caps_t sdr_caps;
			if (sdr_query_caps(&sdr_caps) == 0) {
				rigctl_sdr_caps_t rigctl_caps = {
					.rx_freq_min = sdr_caps.rx_freq_min,
					.rx_freq_max = sdr_caps.rx_freq_max,
					.tx_freq_min = sdr_caps.tx_freq_min,
					.tx_freq_max = sdr_caps.tx_freq_max,
					.rx_upconverter = sdr_caps.rx_upconverter,
					.tx_upconverter = sdr_caps.tx_upconverter,
					.rx_gain_min = sdr_caps.rx_gain_min,
					.rx_gain_max = sdr_caps.rx_gain_max,
					.tx_gain_min = sdr_caps.tx_gain_min,
					.tx_gain_max = sdr_caps.tx_gain_max,
					.has_rx = sdr_caps.has_rx,
					.has_tx = sdr_caps.has_tx,
					.is_split = sdr_caps.is_split,
				};
				strncpy(rigctl_caps.rx_gain_names, sdr_caps.rx_gain_names,
					sizeof(rigctl_caps.rx_gain_names) - 1);
				strncpy(rigctl_caps.tx_gain_names, sdr_caps.tx_gain_names,
					sizeof(rigctl_caps.tx_gain_names) - 1);
				rigctl_server_set_sdr_caps(&s->rigctl_srv, &rigctl_caps);
			}
		}

		/* Set up callbacks */
		rigctl_callbacks_t rigctl_cb = {
			.set_rx_freq = rigctl_set_rx_freq,
			.set_tx_freq = rigctl_set_tx_freq,
			.get_rx_freq = rigctl_get_rx_freq,
			.get_tx_freq = rigctl_get_tx_freq,
			.arg = s
		};
		rigctl_server_set_callbacks(&s->rigctl_srv, &rigctl_cb);
		s->rigctl_enabled = 1;
		printf("Rigctl server enabled: %s\n", s->rigctl_server_endpoint);
	}

	/* Set up RDS decoder callback for RX protocol servers (XDR-GTK, RDS-Spy) */
	if (s->xdr_enabled || s->rdsspy_enabled) {
		s->rds_cb_ctx.xdr_srv = s->xdr_enabled ? &s->xdr_srv : NULL;
		s->rds_cb_ctx.rdsspy_srv = s->rdsspy_enabled ? &s->rdsspy_srv : NULL;
		s->rds_cb_ctx.asciig_srv = NULL;
		s->rds_cb_ctx.radio = &s->radio;
		rds_decoder_set_group_callback(&s->radio.rds_dec, rds_group_callback, &s->rds_cb_ctx);
	}

	/* Wire tune callback into XDR-GTK and RDS-Spy servers (RX tuner control) */
	if (s->xdr_enabled)
		rds_server_set_callbacks(&s->xdr_srv, radio_tune_cb, radio_setting_cb, &s->rds_cb_ctx);
	if (s->rdsspy_enabled)
		rds_server_set_callbacks(&s->rdsspy_srv, radio_tune_cb, NULL, &s->rds_cb_ctx);
	/* Wire tune callback into ASCII-G server (TX tuner control) */
	if (s->asciig_enabled)
		rds_server_set_callbacks(&s->asciig_srv, radio_tx_tune_cb, NULL, &s->rds_cb_ctx);

	/* Wire spectral scan callback into XDR-GTK server */
	if (s->xdr_enabled && s->rx)
		rds_server_set_scan_callback(&s->xdr_srv, radio_scan_cb, &s->rds_cb_ctx);


	s->sendbuff = calloc(s->buffer_size * 2, sizeof(*s->sendbuff));
	if (!s->sendbuff) {
		fprintf(stderr, "No mem!\n");
		return -1;
	}

	return 0;
}

/* One pass of a station: its paging pipe, servers, RX (read, meter, radio)
 * and TX (radio, write). Returns < 0 to stop, 1 when radio_tx() had no
 * audio (the caller skips the keys this pass, as before), else 0. */
static int station_poll(station_t *s, void *sdr)
{
	int tosend, got;

	/* Check paging pipe for new messages */
	if (s->paging_pipe_fd >= 0)
		paging_pipe_handler(s);

	/* Poll protocol servers (non-blocking) */
	if (s->xdr_enabled)
		rds_server_poll(&s->xdr_srv);
	if (s->rdsspy_enabled)
		rds_server_poll(&s->rdsspy_srv);
	if (s->uecp_enabled)
		rds_server_poll(&s->uecp_srv);
	if (s->asciig_enabled)
		rds_server_poll(&s->asciig_srv);
	if (s->rigctl_enabled)
		rigctl_server_poll(&s->rigctl_srv);

	got = 0;
	if (s->rx) {
		got = sdr_read(sdr, (void *)s->sendbuff, s->buffer_size, 0, NULL);
		/* Feed raw IQ into scan engine if active (FFT-based, no retuning needed
		 * unless scan range exceeds SDR bandwidth) */
		if (s->xdr_enabled && s->xdr_srv.scan_active)
			rds_server_scan_feed_iq(&s->xdr_srv, s->sendbuff, got,
						s->frequency, sdr_get_samplerate(sdr));

		/* Feed IQ into spectral measurement (same data/rate as scan) */
		sdr_spectral_feed_iq(s->sendbuff, got);

		/* Feed IQ into signal meter for time-domain RMS */
		signal_meter_feed_iq(s->meter, s->sendbuff, got);
		/* Use last known SNR estimate for stereo quieting heuristics in radio_rx(). */
		radio_set_rx_snr(&s->radio, signal_meter_get_snr(s->meter));

		got = radio_rx(&s->radio, s->sendbuff, got);
		if (got < 0)
			return -1;

		/* Update 19 kHz stereo pilot info from radio_rx() results */
		signal_meter_set_stereo_pilot(s->meter, s->radio.rx_pilot_mag_avg,
					      s->radio.rx_pilot_locked);

		/* Feed FFT-based measurements to signal meter.
		 * Both signal power (at tuned frequency) and noise floor (minimum of offsets)
		 * come from FFT - same method as scan for consistent dBf output. */
		if (sdr_spectral_is_valid()) {
			double nf = sdr_spectral_get_noise_floor();
			signal_meter_set_noise_floor(s->meter, nf);
			if (s->spectral_freq_idx >= 0)
				signal_meter_set_signal_power(s->meter, sdr_spectral_get_power(s->spectral_freq_idx));
		} else {
			const sdr_status_t *ss = sdr_status_get();
			if (ss && ss->rx.valid)
				signal_meter_set_noise_floor(s->meter, ss->rx.noise_floor_db);
		}
		/* Refresh SNR after FFT/noise updates for next RX iteration. */
		radio_set_rx_snr(&s->radio, signal_meter_get_snr(s->meter));

		/* Feed signal level to XDR-GTK for periodic S reports */
		if (s->xdr_enabled) {
			double sig = signal_meter_get_level_dbf(s->meter);
			int is_stereo = s->radio.stereo && s->radio.rx_pilot_locked && !s->radio.rx_forced_mono;
			rds_server_update_signal(&s->xdr_srv,
				(sig > SIGNAL_METER_NO_VALUE) ? sig : -100.0,
				is_stereo,
				s->radio.rx_pilot_mag_avg);
		}

		/* Feed signal level to rigctl server */
		if (s->rigctl_enabled) {
			double sig = signal_meter_get_level_dbfs(s->meter);
			rigctl_server_set_signal(&s->rigctl_srv,
				(sig > SIGNAL_METER_NO_VALUE) ? sig : -100.0);
		}

		/* CPU update (self-throttles to ~1/sec via internal wall-clock check) */
		signal_meter_update_cpu(s->meter);
	}

	/* TX processing - only if TX mode is enabled */
	if (s->tx) {
		tosend = sdr_get_tosend(sdr, s->buffer_size);
		if (tosend > s->buffer_size)
			tosend = s->buffer_size;
		if (tosend == 0)
			return 0;
		/* perform radio modulation */
		tosend = radio_tx(&s->radio, s->sendbuff, tosend);
		if (tosend <= 0) {
			if (tosend < 0)
				return -1;
			return 1; /* radio_tx returned 0: no audio to process, skip SDR write */
		}
		/* write to SDR */
		sdr_write(sdr, (void *)s->sendbuff, NULL, tosend, NULL, NULL, 0);
	}

	return 0;
}

/* Stop a station (after the SDR is closed): meter, paging pipe, protocol
 * servers, radio, buffer. Safe after a partial station_init(). */
static void station_exit(station_t *s)
{
	free(s->sendbuff);
	s->sendbuff = NULL;
	signal_meter_free(s->meter);
	s->meter = NULL;
	/* Clean up paging pipe */
	if (s->paging_pipe_fd >= 0)
		close(s->paging_pipe_fd);
	s->paging_pipe_fd = -1;
	/* only the FIFO this station made (another station's may be there) */
	if (s->paging_pipe_made)
		unlink(s->rds_paging_pipe_path);
	s->paging_pipe_made = 0;

	/* Clean up protocol servers */
	if (s->xdr_enabled)
		rds_server_cleanup(&s->xdr_srv);
	if (s->rdsspy_enabled)
		rds_server_cleanup(&s->rdsspy_srv);
	if (s->uecp_enabled)
		rds_server_cleanup(&s->uecp_srv);
	if (s->asciig_enabled)
		rds_server_cleanup(&s->asciig_srv);
	if (s->rigctl_enabled)
		rigctl_server_cleanup(&s->rigctl_srv);
	s->xdr_enabled = s->rdsspy_enabled = s->uecp_enabled = s->asciig_enabled = s->rigctl_enabled = 0;

	radio_exit(&s->radio);
}

int main(int argc, char *argv[])
{
	int rc, argi;
	struct termios term, term_orig;
	int c;
	void *sdr = NULL;

	loglevel = LOGL_NOTICE;
	logging_init();

	sdr_config_init(DEFAULT_LO_OFFSET);
	station_defaults(st);

	/* handle options / config file */
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/radio.conf", handle_options);
	if (rc < 0)
		return 0;
	options_from_command_line = 1;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;
	/* no -f on the command line: the station is the defaults */
	if (!num_stations) {
		station0 = station_default;
		num_stations = 1;
	}
	st = &station0;

	if (st->frequency == 0.0) {
		printf("No frequency given, I suggest to use 100000000 (100 MHz) and FM\n\n");
		print_help(argv[0]);
		exit(0);
	}

	/* --rds-ecc: check ECC + PI country digit against EN 50067 Table D.1.
	 * A mismatch is only a warning: the ECC is still transmitted as given. */
	if (st->rds_ecc && st->rds_pi_opt) {
		uint8_t pi_cc = st->rds_pi_opt >> 12;
		const char *c = rds_get_country_code(pi_cc, st->rds_ecc);

		if (c && strcmp(c, "--")) {
			fprintf(stderr, "RDS: ECC %02X + PI %04X (country digit %X) = country %s\n", st->rds_ecc, st->rds_pi_opt, pi_cc, c);
		} else {
			const char *s = rds_ecc_suggest(pi_cc);

			fprintf(stderr, "Warning: ECC %02X + PI %04X (country digit %X) is no country in EN 50067 Table D.1.\n", st->rds_ecc, st->rds_pi_opt, pi_cc);
			fprintf(stderr, "         Receivers show no or a wrong country, pagers may not lock to this channel.\n");
			fprintf(stderr, "         ECC %02X is still transmitted as given.\n", st->rds_ecc);
			if (s[0]) {
				fprintf(stderr, "         For PI country digit %X use --rds-ecc %s\n", pi_cc, s);
				fprintf(stderr, "         or set --pi to your country digit, see '--rds-ecc list'.\n");
			} else {
				fprintf(stderr, "         PI country digit %X is not a country digit (1-F), set --pi to your\n", pi_cc);
				fprintf(stderr, "         country digit, see '--rds-ecc list'.\n");
			}
		}
	}
	if (st->rds_ecc)
		st->radio_opts.rds_ecc = st->rds_ecc;	/* overrides the preset ECC, also on preset change */

	if (st->rds_paging && st->tx && !st->rds_ecc) {
		fprintf(stderr, "RDS paging TX requires --rds-ecc <hex> (Extended Country Code, EN 50067 Annex D), use '-h' for help!\n");
		exit(0);
	}
	/* 13A enhanced paging needs the operator code (EN 50067 M.3.2.7.2) */
	if (st->rds_paging_13a && (st->rds_paging_opc < 1 || st->rds_paging_opc > 15)) {
		fprintf(stderr, "--rds-paging-13a requires --rds-paging-opc 1-15 (EN 50067 M.3.2.7.2), use '-h' for help!\n");
		exit(0);
	}

	/* global inits */
	fm_init(fast_math);
	am_init(fast_math);

	/* Determine input samplerate */
	if (st->channelizer_rate > 0)
		st->iq_rate = st->channelizer_rate;
	else
		st->iq_rate = st->dsp_samplerate;

	rc = sdr_configure(st->iq_rate);
	if (rc < 0)
		return rc;
	if (rc == 0) {
		fprintf(stderr, "Please select SDR, use '-h' for help!\n");
		exit(0);
	}

	/* Set default bandwidth based on modulation type (must be before channelizer rate calc) */
	if (st->bandwidth == 0) {
		if (st->modulation == MODULATION_FM)
			st->bandwidth = st->bandwidth_fm;
		else
			st->bandwidth = st->bandwidth_am;
	}

	/* Calculate baseband_extent (max baseband frequency, one-sided)
	 * Used for channelizer rate calculation before radio_init.
	 * After radio_init, use radio.baseband_extent instead. */
	double baseband_extent = 0;
	if (st->modulation == MODULATION_FM) {
		double audio_bw = st->bandwidth;
		/* FM broadcast with stereo/RDS needs more bandwidth */
		if (st->rds || st->rds2) audio_bw = 60000.0;
		else if (st->stereo) audio_bw = 53000.0;
		baseband_extent = st->deviation + audio_bw;
	} else {
		/* AM: baseband_extent = audio bandwidth */
		baseband_extent = st->bandwidth;
	}

	/* The SDR's channelizer takes the band around the frequency to the
	 * radio's IQ rate: as given, else the lowest SDR rate / 2^k that holds
	 * the baseband (halfband stages only) */
	if (st->channelizer_rate == 0 && sdr_config && sdr_config->samplerate > st->dsp_samplerate)
		st->iq_rate = sdr_calculate_optimal_rate(sdr_config->samplerate, baseband_extent);

	if (sdr_config)
		LOGP(DSDR, LOGL_NOTICE, "Channelizer: SDR=%d Hz -> IQ=%d Hz (rf_bw=%.1f kHz, signal=%.1f kHz)\n",
		     sdr_config->samplerate, st->iq_rate,
		     2.0 * baseband_extent / 1000.0, st->dsp_samplerate / 1000.0);
	/* on an SDR backend the SDR rate is the backend's, this program does
	 * not know it: the IQ rate comes from -s unless it is given */
	if (sdr_config && sdr_config->backend && st->iq_rate < baseband_extent * 2.0 / 0.75)
		LOGP(DSDR, LOGL_NOTICE, "On the SDR backend: the IQ rate %d Hz is too low for this signal (%.0f kHz wide, needs %.0f Hz): give the backend's SDR rate with --sdr-samplerate, or --channelizer-rate.\n",
		     st->iq_rate, 2.0 * baseband_extent / 1000.0, ceil(baseband_extent * 2.0 / 0.75));

	if (st->modulation == MODULATION_NONE) {
		fprintf(stderr, "Please select modulation, use '-h' for help!\n");
		exit(0);
	}

	if (st->stereo && st->modulation != MODULATION_FM) {
		fprintf(stderr, "Stereo works with FM only, use '-h' for help!\n");
		exit(0);
	}
	if (!st->rx && !st->tx) {
		fprintf(stderr, "You need to specify --rx (receiver) and/or --tx (transmitter), use '-h' for help!\n");
		exit(0);
	}
	if (st->stereo && st->bandwidth != 15000.0) {
		fprintf(stderr, "Warning: Stereo works with bandwidth of 15 KHz only, using this bandwidth!\n");
	}
	if (st->stereo && st->time_constant_us != 75.0 && st->time_constant_us != 50.0) {
		fprintf(stderr, "Stereo works with time constant of 50 uS or 75 uS only, use '-h' for help!\n");
		exit(0);
	}

	/* Keep SDR direction flags in sync with radio app direction flags.
	 * Without this, --rx / --tx still leaves the opposite SDR path enabled,
	 * which breaks RX-only SDRs (e.g. RTL-SDR) and needlessly initializes
	 * TX-side radio components in receive-only mode. */
	if (st->rx && !st->tx) {
		if (sdr_config->tx_only) {
			fprintf(stderr, "Conflicting options: --rx cannot be combined with --sdr-tx-only\n");
			exit(0);
		}
		sdr_config->rx_only = 1;
	}
	if (st->tx && !st->rx) {
		if (sdr_config->rx_only) {
			fprintf(stderr, "Conflicting options: --tx cannot be combined with --sdr-rx-only\n");
			exit(0);
		}
		sdr_config->tx_only = 1;
	}

	rc = station_init(st, sdr);
	if (rc < 0)
		goto error;

	/* real time priority */
	if (rt_prio > 0) {
		struct sched_param schedp;
		int rc;

		memset(&schedp, 0, sizeof(schedp));
		schedp.sched_priority = rt_prio;
		rc = sched_setscheduler(0, SCHED_RR, &schedp);
		if (rc) {
			fprintf(stderr, "Error setting SCHED_RR with prio %d\n", rt_prio);
			goto error;
		}
	}

	double tx_frequencies[1], rx_frequencies[1];
	int am[1];
	tx_frequencies[0] = st->frequency;
	rx_frequencies[0] = st->frequency;
	am[0] = 0;
	sdr = sdr_open(0, NULL, tx_frequencies, rx_frequencies, am, 0, 0.0, st->iq_rate, st->buffer_size, 1.0, 0.0, 0.0, 0.0);
	if (!sdr)
		goto error;
	sdr_start(sdr);

	/* Configure spectral measurement - center matches SDR tuned frequency */
	sdr_spectral_configure(sdr_get_samplerate(sdr), st->frequency);
	if (st->frequency > 0.0)
		st->spectral_freq_idx = sdr_spectral_register_freq(st->frequency);

	/* prepare terminal */
	tcgetattr(0, &term_orig);
	term = term_orig;
	term.c_lflag &= ~(ISIG|ICANON|ECHO);
	term.c_cc[VMIN]=1;
	term.c_cc[VTIME]=2;
	tcsetattr(0, TCSANOW, &term);

	/* catch signals */
	signal(SIGINT, sighandler);
	signal(SIGHUP, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGPIPE, sighandler);

	printf("Starting radio...\n");
	rc = radio_start(&st->radio);
	if (rc < 0) {
		fprintf(stderr, "Failed to start radio's streaming, exitting!\n");
		goto error_start;
	}

	while (!quit) {
		/* Adaptive sleep: shorter when TX buffer needs feeding.
		 * In TX mode the write thread drains the ring buffer continuously;
		 * sleeping a full 1ms risks starving the hardware (UHD underruns).
		 * Use 100us when TX is active to keep the buffer fed.
		 * In RX-only mode, use 250us to reduce ring buffer pressure
		 * and avoid SDR-side overflow on high sample rate devices. */
		if (st->tx)
			usleep(100);
		else
			usleep(250);

		rc = station_poll(st, sdr);
		if (rc < 0)
			break;
		if (rc > 0)
			continue;

		/* process keyboard input */
next_char:
		c = get_char();
		switch (c) {
		case 3:
			/* quit */
//			if (clear_console_text)
//				clear_console_text();
			printf("CTRL+c received, quitting!\n");
			quit = 1;
			goto next_char;
#if 0
- carrier frequency
- deviation
- modulation index
- stereo pilot
		case 'm':
			/* toggle measurements display */
			display_iq_on(0);
			display_spectrum_on(0);
			display_wave_on(0);
			display_measurements_on(-1);
			goto next_char;
#endif
		case 'q':
			/* toggle IQ display */
			display_measurements_on(0);
			display_spectrum_on(0);
			display_wave_on(0);
			display_iq_on(-1);
			goto next_char;
		case 's':
			/* toggle spectrum display */
			display_measurements_on(0);
			display_iq_on(0);
			display_wave_on(0);
			display_spectrum_on(-1);
			goto next_char;
		case 'w':
			/* toggle wave display */
			display_measurements_on(0);
			display_iq_on(0);
			display_spectrum_on(0);
			display_wave_on(-1);
			goto next_char;
		case 'b':
			calibrate_bias();
			goto next_char;
		case 'd':
			/* dump RDS status */
			if (st->rx && st->rds)
				rds_decoder_status(&st->radio.rds_dec);
			goto next_char;
		case 'f':
			/* cycle RDS presets */
			if (st->tx && st->rds)
				rds_next_preset(&st->radio);
			goto next_char;
		case '1':
			/* flip RT A/B flag */
			if (st->tx && st->rds)
				rds_flip_rt_ab(&st->radio);
			goto next_char;
		case '2':
			/* flip RT+ item_running */
			if (st->tx && st->rds)
				rds_flip_rtplus_item_running(&st->radio);
			goto next_char;
		case '3':
			/* flip RT+ item_toggle */
			if (st->tx && st->rds)
				rds_flip_rtplus_toggle(&st->radio);
			goto next_char;
		}
	}

error_start:
	/* reset signals */
	signal(SIGINT, SIG_DFL);
	signal(SIGHUP, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);

	display_measurements_on(0);
	display_spectrum_on(0);
	display_wave_on(0);
	display_iq_on(0);

	/* reset terminal */
	tcsetattr(0, TCSANOW, &term_orig);
	
error:
	/* reset real time prio */
	if (rt_prio > 0) {
		struct sched_param schedp;

		memset(&schedp, 0, sizeof(schedp));
		schedp.sched_priority = 0;
		sched_setscheduler(0, SCHED_OTHER, &schedp);
	}

	if (sdr)
		sdr_close(sdr);
	station_exit(st);

	/* global exits */
	fm_exit();
	am_exit();

	options_free();

	return 0;
}

void osmo_cc_set_log_cat(int __attribute__((unused)) cc_log_cat) {}

