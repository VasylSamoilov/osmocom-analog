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
#include "rigctl_server.h"
#include "signal_meter.h"
#include "../libmobile/mcc.h"

#define DEFAULT_LO_OFFSET -1000000.0
#define RDS_PAGING_PIPE_DEFAULT "/tmp/rds_paging_send"

static const char *rds_paging_pipe_path = RDS_PAGING_PIPE_DEFAULT;
static int paging_pipe_fd = -1;

/* Signal meter (file-scope so tune callback can access it) */
static signal_meter_t *meter = NULL;
static int spectral_freq_idx = -1;	/* index of our registered frequency in spectral measurement */

sender_t *sender_head = NULL;
int use_sdr = 0;
int num_kanal = 1; /* only one channel used for debugging */
int rt_prio = 0;
int fast_math = 0;

sender_t *get_sender_by_empfangsfrequenz(double __attribute__((unused)) freq) { return NULL; }

static double frequency = 0.0;
static int dsp_samplerate = 100000;
static int dsp_buffer = 30;
static const char *tx_wave_file = NULL;
static const char *rx_wave_file = NULL;
static const char *tx_audiodev = NULL;
static const char *rx_audiodev = NULL;
static enum modulation modulation = MODULATION_NONE;
static int rx = 0, tx = 0;
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
static double bandwidth_am = 4500.0;	/* AM audio bandwidth (Hz) */
static double bandwidth_fm = 15000.0;	/* FM audio bandwidth (Hz) */
static double bandwidth = 0.0;		/* User-specified audio bandwidth override */
static double deviation = 75000.0;	/* FM deviation (Hz), ±75kHz for broadcast */
static double modulation_index = 1.0;
static double time_constant_us = 50.0;
static double volume = 1.0;
static int stereo = 0;
static int rds = 0;
static int rds2 = 0;
static int rds_debug = 0;
static int rds_verbose = 0;
static int rds_force_rbds = 0;
static int sca_67k = 0;
static int sca_92k = 0;
static int am_compandor = 0;
static int fm_compandor = 0;
static int rds_paging = 0;
static int rds_paging_rpc = 4;  /* Default RPC=4: group desig 001, batt sync 00 */
static uint16_t rds_paging_cc = 0;   /* E.212 MCC for international (0=not set) */
static uint8_t rds_paging_opc = 0;   /* OPC for international (0=not set) */
static uint8_t rds_paging_pac = 0;   /* PAC for paging area (0=all areas) */
static const char *rds_hexrds_file = NULL;
static const char *rds_bitstream_file = NULL;
static const char *rds_tx_hexrds_file = NULL;
static const char *rds_tx_bitstream_file = NULL;

/* Protocol server endpoints */
static const char *xdr_gtk_server = NULL;
static const char *rdsspy_server = NULL;
static const char *uecp_server = NULL;
static const char *asciig_server = NULL;
static const char *rigctl_server_endpoint = NULL;
static const char *rigctl_allowed_hosts = NULL;
static const char *xdr_gtk_password = NULL;

/* global variable to quit main loop */
int quit = 0;

/* Protocol server callback context */
typedef struct {
	rds_server_t *xdr_srv;
	rds_server_t *rdsspy_srv;
	rds_server_t *asciig_srv;
	radio_t *radio;
} rds_callback_ctx_t;

/* Tune callback for XDR-GTK (RX side): retunes the SDR receiver */
static void radio_tune_cb(int freq_khz, void *arg)
{
	rds_callback_ctx_t *ctx = (rds_callback_ctx_t *)arg;
	double freq_hz = freq_khz * 1000.0;

	/* Dump RDS status before re-tuning (if RDS enabled and have data) */
	if (rds && ctx->radio) {
		if (ctx->radio->rds_dec.blocks_ok > 0) {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz, last known RDS data:\n",
			     frequency / 1e6);
			rds_decoder_status(&ctx->radio->rds_dec);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz (no valid RDS blocks received)\n",
			     frequency / 1e6);
		}
	}

	LOGP(DRADIO, LOGL_NOTICE, "Re-tuning SDR RX to %d kHz\n", freq_khz);

	if (sdr_set_rx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Failed to re-tune SDR to %d kHz\n", freq_khz);
		return;
	}

	/* Update global frequency and XDR-GTK server state */
	frequency = freq_hz;
	if (ctx->xdr_srv)
		ctx->xdr_srv->freq_khz = freq_khz;

	/* Clear stale measurements from previous frequency */
	if (meter)
		signal_meter_reset(meter);

	/* Reset RDS decoder for new station */
	if (rds && ctx->radio)
		rds_decoder_reset(&ctx->radio->rds_dec);

	/* Update spectral measurement for new frequency */
	sdr_spectral_set_center(freq_hz);
	sdr_spectral_clear_freq();
	spectral_freq_idx = sdr_spectral_register_freq(freq_hz);

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

	frequency = freq_hz;
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
	(void)arg;
	double freq_hz = freq_khz * 1000.0;

	LOGP(DRADIO, LOGL_NOTICE, "Re-tuning SDR TX to %d kHz\n", freq_khz);

	if (sdr_set_tx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Failed to re-tune SDR TX to %d kHz\n", freq_khz);
		return;
	}

	frequency = freq_hz;
}

/* Rigctl callbacks */
static int rigctl_set_rx_freq(double freq_hz, void *arg)
{
	radio_t *r = (radio_t *)arg;

	/* Dump RDS status before re-tuning (if RDS enabled and have data) */
	if (rds && r) {
		if (r->rds_dec.blocks_ok > 0) {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz, last known RDS data:\n",
			     frequency / 1e6);
			rds_decoder_status(&r->rds_dec);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "Leaving %.1f MHz (no valid RDS blocks received)\n",
			     frequency / 1e6);
		}
	}

	LOGP(DRADIO, LOGL_NOTICE, "Rigctl: Re-tuning SDR RX to %.0f Hz\n", freq_hz);

	if (sdr_set_rx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Rigctl: Failed to re-tune SDR RX\n");
		return -1;
	}

	frequency = freq_hz;

	/* Clear stale measurements */
	if (meter)
		signal_meter_reset(meter);

	/* Reset RDS decoder for new station */
	if (rds && r)
		rds_decoder_reset(&r->rds_dec);

	/* Update spectral measurement */
	sdr_spectral_set_center(freq_hz);
	sdr_spectral_clear_freq();
	spectral_freq_idx = sdr_spectral_register_freq(freq_hz);

	return 0;
}

static int rigctl_set_tx_freq(double freq_hz, void *arg)
{
	(void)arg;

	LOGP(DRADIO, LOGL_NOTICE, "Rigctl: Re-tuning SDR TX to %.0f Hz\n", freq_hz);

	if (sdr_set_tx_frequency(freq_hz) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Rigctl: Failed to re-tune SDR TX\n");
		return -1;
	}

	frequency = freq_hz;
	return 0;
}

static double rigctl_get_rx_freq(void *arg)
{
	(void)arg;
	return frequency;
}

static double rigctl_get_tx_freq(void *arg)
{
	(void)arg;
	return frequency;
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
	printf("        Give frequency in Hertz.\n");
	printf(" -s --samplerate <sample rate>\n");
	printf("        Give signal processing sample rate in Hz. (default = %d)\n", dsp_samplerate);
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
	printf("        Give bandwidth of audio frequency. (default AM=%.0f FM=%.0f)\n", bandwidth_am, bandwidth_fm);
	printf(" -D --deviation\n");
	printf("        Give deviation of frequency modulated signal. (default %.0f)\n", deviation);
	printf(" -I --modulation-index 0..1\n");
	printf("        Give modulation index of amplitude modulated signal. (default %.0f)\n", modulation_index);
	printf(" -C --compandor\n");
	printf("        Enable audio compressor for AM to improve modulation depth.\n");
	printf("        Uses 2:1 compression with 5ms attack, 200ms recovery.\n");
	printf("    --fm-compandor\n");
	printf("        Enable audio compressor for FM to improve broadcast loudness.\n");
	printf("        Uses 2:1 compression with 1ms attack, 50ms recovery.\n");
	printf("        Raises average modulation to match commercial FM stations.\n");
	printf(" -E --emphasis <uS> | 0\n");
	printf("        Use given time constant of pre- and de-emphasis for frequency\n");
	printf("        modulation. Give 0 to disable emphasis. (default = %.0f uS)\n", time_constant_us);
	printf("        VHF broadcast 50 uS in Europe and 75 uS in the United States.\n");
	printf("        Other radio FM should use 530 uS, to cover complete speech spectrum.\n");
	printf(" -V --volume %.3f\n", volume);
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
	printf("        Types: tone, numeric, numeric10, numeric18, alpha, vnum\n");
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
	printf("          Max alpha=78 chars, max vnum=156 digits (vs 80/160 national).\n");
	printf("        Hex escapes: <0xCC> in message sends raw byte 0xCC.\n");
	printf("        Examples:\n");
	printf("          echo '200078,tone,,' > %s\n", rds_paging_pipe_path);
	printf("          echo '200078,tone,tonetype=3 repeat=2,' > %s\n", rds_paging_pipe_path);
	printf("          echo '200078,numeric,,1234567890' > %s\n", rds_paging_pipe_path);
	printf("          echo '200078,alpha,,Hello World' > %s\n", rds_paging_pipe_path);
	printf("          echo '200078,alpha,repeat=3 interval=5,Hello, World!' > %s\n", rds_paging_pipe_path);
	printf("          echo '200078,alpha,ni=1 cc=255 opc=1,Intl msg' > %s\n", rds_paging_pipe_path);
	printf("          echo '200078,vnum,,31415926535' > %s\n", rds_paging_pipe_path);
	printf("          echo '200078,vnum,ni=1 cc=262 opc=3,0123456789' > %s\n", rds_paging_pipe_path);
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
	printf("    --channelizer [auto|on|off]\n");
	printf("        Decimation channelizer mode. auto (default): enable when SDR rate > DSP rate.\n");
	printf("        on: always enable. off: never enable (use legacy sample-skip path).\n");
	printf("    --channelizer-rate <rate>\n");
	printf("        Explicitly set channelizer input processing rate (Hz).\n");
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

static int channelizer_rate = 0; /* 0 = disabled */
static int use_channelizer = 0; /* -1=off, 0=auto, 1=on */
static int use_polyphase = 0;
static int use_afc = 0;
static double afc_tc_ms = 300.0;
static double afc_max_hz = 5000.0;

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
#define OPT_RDS_PAGING_FIFO	1121
#define OPT_RDS_PAGING_CC	1122
#define OPT_RDS_PAGING_OPC	1123
#define OPT_RDS_PAGING_PAC	1124
#define OPT_FM_COMPANDOR	1125

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
	option_add(OPT_CHANNELIZER, "channelizer", 1);
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
	option_add(OPT_RDS_HEXRDS_FILE, "rds-hexrds-file", 1);
	option_add(OPT_RDS_BITSTREAM_FILE, "rds-bitstream-file", 1);
	option_add(OPT_RDS_TX_HEXRDS_FILE, "rds-tx-hexrds-file", 1);
	option_add(OPT_RDS_TX_BITSTREAM_FILE, "rds-tx-bitstream-file", 1);
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
		frequency = atof(argv[argi]);
		break;
	case 's':
		dsp_samplerate = atof(argv[argi]);
		break;
	case 'r':
		tx_wave_file = options_strdup(argv[argi]);
		break;
	case 'w':
		rx_wave_file = options_strdup(argv[argi]);
		break;
	case 'a':
		tx_audiodev = options_strdup(argv[argi]);
		rx_audiodev = options_strdup(argv[argi]);
		break;
	case 'M':
		if (!strcasecmp(argv[argi], "fm"))
			modulation = MODULATION_FM;
		else
		if (!strcasecmp(argv[argi], "am"))
			modulation = MODULATION_AM_DSB;
		else
		if (!strcasecmp(argv[argi], "usb"))
			modulation = MODULATION_AM_USB;
		else
		if (!strcasecmp(argv[argi], "lsb"))
			modulation = MODULATION_AM_LSB;
		else
		{
			fprintf(stderr, "Invalid modulation option, use '-h' for help!\n");
			return -EINVAL;
		}
		break;
	case 'R':
		rx = 1;
		break;
	case 'T':
		tx = 1;
		break;
	case 'B':
		bandwidth = atof(argv[argi]);
		break;
	case 'D':
		deviation = atof(argv[argi]);
		break;
	case 'I':
		modulation_index = atof(argv[argi]);
		if (modulation_index < 0.0 || modulation_index > 1.0) {
			fprintf(stderr, "Invalid modulation index, use '-h' for help!\n");
			return -EINVAL;
		}
		break;
	case 'C':
		am_compandor = 1;
		break;
	case OPT_FM_COMPANDOR:
		fm_compandor = 1;
		break;
	case 'E':
		time_constant_us = atof(argv[argi]);
		break;
	case 'V':
		volume = atof(argv[argi]);
		break;
	case 'S':
		stereo = 1;
		break;
	case OPT_RDS:
		rds = 1;
		break;
	case OPT_RDS2:
		rds2 = 1;
		break;
	case OPT_RDS_DEBUG:
		rds_debug = 1;
		break;
	case OPT_RDS_VERBOSE:
		rds_verbose = 1;
		break;
	case OPT_RBDS:
		rds_force_rbds = 1;
		break;
	case OPT_SCA_67K:
		sca_67k = 1;
		break;
	case OPT_SCA_92K:
		sca_92k = 1;
		break;
	case OPT_FAST_MATH:
		fast_math = 1;
		break;
	case OPT_CHANNELIZER:
		if (!strcasecmp(argv[argi], "on"))
			use_channelizer = 1;
		else if (!strcasecmp(argv[argi], "off"))
			use_channelizer = -1;
		else if (!strcasecmp(argv[argi], "auto"))
			use_channelizer = 0;
		else {
			fprintf(stderr, "Invalid --channelizer value '%s', use auto, on, or off.\n", argv[argi]);
			exit(0);
		}
		break;
	case OPT_CHANNELIZER_RATE:
		channelizer_rate = atoi(argv[argi]);
		use_channelizer = 1;
		break;
	case OPT_CALL_SIGN:
		radio_set_callsign(argv[argi]);
		break;
	case OPT_PI:
		radio_set_pi((uint16_t)strtoul(argv[argi], NULL, 16));
		break;
	case OPT_POLYPHASE:
		use_polyphase = 1;
		break;
	case OPT_RDS_PAGING:
		rds_paging = 1;
		rds = 1;  /* auto-enable RDS */
		{
			int rpc_val = atoi(argv[argi]);
			if (rpc_val < 0 || rpc_val > 31) {
				fprintf(stderr, "Invalid RPC value %d (must be 0-31), use '-h' for help!\n", rpc_val);
				return -EINVAL;
			}
			rds_paging_rpc = rpc_val;
		}
		break;
	case OPT_RDS_PAGING_FIFO:
		rds_paging_pipe_path = argv[argi];
		break;
	case OPT_RDS_PAGING_CC:
		rds_paging_cc = (uint16_t)atoi(argv[argi]);
		if (!mcc_valid(rds_paging_cc)) {
			const char *cname = mcc_name(rds_paging_cc);
			if (!cname) {
				fprintf(stderr, "Invalid E.212 MCC %d (must be 200-999), use '-h' for help!\n", rds_paging_cc);
				return -EINVAL;
			}
		}
		{
			const char *cname = mcc_name(rds_paging_cc);
			fprintf(stderr, "RDS Paging: international MCC=%d (%s)\n",
				rds_paging_cc, cname ? cname : "unknown");
		}
		break;
	case OPT_RDS_PAGING_OPC:
		rds_paging_opc = (uint8_t)atoi(argv[argi]);
		if (rds_paging_opc > 15) {
			fprintf(stderr, "Invalid OPC %d (must be 0-15), use '-h' for help!\n", rds_paging_opc);
			return -EINVAL;
		}
		break;
	case OPT_RDS_PAGING_PAC:
		rds_paging_pac = (uint8_t)atoi(argv[argi]);
		if (rds_paging_pac > 63) {
			fprintf(stderr, "Invalid PAC %d (must be 0-63), use '-h' for help!\n", rds_paging_pac);
			return -EINVAL;
		}
		break;
	case OPT_RDS_HEXRDS_FILE:
		rds_hexrds_file = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
		break;
	case OPT_RDS_BITSTREAM_FILE:
		rds_bitstream_file = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
		break;
	case OPT_RDS_TX_HEXRDS_FILE:
		rds_tx_hexrds_file = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
		break;
	case OPT_RDS_TX_BITSTREAM_FILE:
		rds_tx_bitstream_file = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
		break;
	case OPT_XDR_GTK_SERVER:
		xdr_gtk_server = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
		break;
	case OPT_XDR_GTK_PASSWORD:
		xdr_gtk_password = options_strdup(argv[argi]);
		break;
	case OPT_RIGCTL_SERVER:
		rigctl_server_endpoint = options_strdup(argv[argi]);
		break;
	case OPT_RIGCTL_ALLOWED:
		rigctl_allowed_hosts = options_strdup(argv[argi]);
		break;
	case OPT_RDSSPY_SERVER:
		rdsspy_server = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
		break;
	case OPT_UECP_SERVER:
		uecp_server = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
		break;
	case OPT_ASCIIG_SERVER:
		asciig_server = options_strdup(argv[argi]);
		rds = 1;  /* auto-enable RDS */
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
		use_afc = 1;
		break;
	case OPT_AFC_TC:
		afc_tc_ms = atof(argv[argi]);
		if (afc_tc_ms < 10.0) afc_tc_ms = 10.0;
		if (afc_tc_ms > 10000.0) afc_tc_ms = 10000.0;
		break;
	case OPT_AFC_MAX:
		afc_max_hz = atof(argv[argi]);
		if (afc_max_hz < 100.0) afc_max_hz = 100.0;
		if (afc_max_hz > 50000.0) afc_max_hz = 50000.0;
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
		     "  types:   tone|numeric|numeric10|numeric18|alpha|vnum\n"
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
		rds_enc_paging_send_tone(enc, address, repeat, interval, tonetype);
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
		rds_enc_paging_send_numeric(enc, address, msg, repeat, interval, ni);
		LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: NUMERIC addr=%02u-%04u len=%d repeat=%d interval=%d msg=\"%s\"\n",
		     address / 10000, address % 10000, msg_len, repeat, interval, msg);
	} else if (!strcasecmp(type_buf, "alpha")) {
		if (msg_len <= 0) {
			LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: alpha requires message text\n");
			return;
		}
		rds_enc_paging_send_alpha(enc, address, msg, repeat, interval, ni, country_code, opc);
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
		rds_enc_paging_send_vnum(enc, address, msg, repeat, interval, ni, country_code, opc);
		if (ni) {
			const char *cname = mcc_name(country_code);
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: VNUM addr=%02u-%04u len=%d repeat=%d interval=%d intl MCC=%d(%s) OPC=%d msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval,
			     country_code, cname ? cname : "?", opc, msg);
		} else {
			LOGP(DRADIO, LOGL_NOTICE, "RDS Paging FIFO: VNUM addr=%02u-%04u len=%d repeat=%d interval=%d nat msg=\"%s\"\n",
			     address / 10000, address % 10000, msg_len, repeat, interval, msg);
		}
	} else {
		LOGP(DRADIO, LOGL_ERROR, "RDS paging FIFO: unknown type '%s'\n", type_buf);
	}
}

static void paging_pipe_handler(rds_encoder_t *enc)
{
	static char buf[4096];
	static int buf_len = 0;
	int rc, i, start;
	int space = sizeof(buf) - buf_len;

	if (paging_pipe_fd < 0)
		return;

	rc = read(paging_pipe_fd, buf + buf_len, space);
	if (rc > 0) {
		buf_len += rc;

		/* Overflow handling */
		if (buf_len == (int)sizeof(buf)) {
			int has_newline = 0;
			for (i = 0; i < buf_len; i++) {
				if (buf[i] == '\r' || buf[i] == '\n') {
					has_newline = 1;
					break;
				}
			}
			if (!has_newline) {
				LOGP(DRADIO, LOGL_ERROR, "RDS paging pipe: buffer overflow, discarding!\n");
				buf_len = 0;
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
	}
}

int main(int argc, char *argv[])
{
	int rc, argi;
	radio_t radio;
	struct termios term, term_orig;
	int c;
	int buffer_size;
	int input_samplerate;
	void *sdr = NULL;
	float *sendbuff = NULL;
	
	/* Protocol servers */
	rds_server_t xdr_srv, rdsspy_srv, uecp_srv, asciig_srv;
	rigctl_server_t rigctl_srv;
	int xdr_enabled = 0, rdsspy_enabled = 0, uecp_enabled = 0, asciig_enabled = 0;
	int rigctl_enabled = 0;
	rds_callback_ctx_t rds_cb_ctx = { NULL, NULL, NULL, NULL };

	loglevel = LOGL_NOTICE;
	logging_init();

	sdr_config_init(DEFAULT_LO_OFFSET);

	/* handle options / config file */
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/radio.conf", handle_options);
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;

	if (frequency == 0.0) {
		printf("No frequency given, I suggest to use 100000000 (100 MHz) and FM\n\n");
		print_help(argv[0]);
		exit(0);
	}

	/* global inits */
	fm_init(fast_math);
	am_init(fast_math);

	/* Determine input samplerate */
	if (use_channelizer == 1 && channelizer_rate > 0)
		input_samplerate = channelizer_rate;
	else
		input_samplerate = dsp_samplerate;

	rc = sdr_configure(input_samplerate);
	if (rc < 0)
		return rc;
	if (rc == 0) {
		fprintf(stderr, "Please select SDR, use '-h' for help!\n");
		exit(0);
	}

	/* Set default bandwidth based on modulation type (must be before channelizer rate calc) */
	if (bandwidth == 0) {
		if (modulation == MODULATION_FM)
			bandwidth = bandwidth_fm;
		else
			bandwidth = bandwidth_am;
	}

	/* Calculate baseband_extent (max baseband frequency, one-sided)
	 * Used for channelizer rate calculation before radio_init.
	 * After radio_init, use radio.baseband_extent instead. */
	double baseband_extent = 0;
	if (modulation == MODULATION_FM) {
		double audio_bw = bandwidth;
		/* FM broadcast with stereo/RDS needs more bandwidth */
		if (rds || rds2) audio_bw = 60000.0;
		else if (stereo) audio_bw = 53000.0;
		baseband_extent = deviation + audio_bw;
	} else {
		/* AM: baseband_extent = audio bandwidth */
		baseband_extent = bandwidth;
	}

	/* Auto-enable channelizer when SDR rate > DSP rate — decimation is required */
	if (use_channelizer == 0 && sdr_config && sdr_config->samplerate > dsp_samplerate)
		use_channelizer = 1;

	/* Auto-calculate optimal input rate if channelizer is used */
	if (use_channelizer == 1 && channelizer_rate == 0) {
		if (sdr_config) {
			input_samplerate = sdr_calculate_optimal_rate(sdr_config->samplerate, baseband_extent);
		}
	}

	if (use_channelizer == 1 && sdr_config) {
		int ratio = (input_samplerate > 0) ? sdr_config->samplerate / input_samplerate : 0;
		LOGP(DSDR, LOGL_NOTICE, "Channelizer: SDR=%d Hz -> DSP=%d Hz (decimate x%d, rf_bw=%.1f kHz, signal=%.1f kHz)\n",
		     sdr_config->samplerate, input_samplerate, ratio,
		     2.0 * baseband_extent / 1000.0, dsp_samplerate / 1000.0);
	}

	if (modulation == MODULATION_NONE) {
		fprintf(stderr, "Please select modulation, use '-h' for help!\n");
		exit(0);
	}

	if (stereo && modulation != MODULATION_FM) {
		fprintf(stderr, "Stereo works with FM only, use '-h' for help!\n");
		exit(0);
	}
	if (!rx && !tx) {
		fprintf(stderr, "You need to specify --rx (receiver) and/or --tx (transmitter), use '-h' for help!\n");
		exit(0);
	}
	if (stereo && bandwidth != 15000.0) {
		fprintf(stderr, "Warning: Stereo works with bandwidth of 15 KHz only, using this bandwidth!\n");
	}
	if (stereo && time_constant_us != 75.0 && time_constant_us != 50.0) {
		fprintf(stderr, "Stereo works with time constant of 50 uS or 75 uS only, use '-h' for help!\n");
		exit(0);
	}

	/* Keep SDR direction flags in sync with radio app direction flags.
	 * Without this, --rx / --tx still leaves the opposite SDR path enabled,
	 * which breaks RX-only SDRs (e.g. RTL-SDR) and needlessly initializes
	 * TX-side radio components in receive-only mode. */
	if (rx && !tx) {
		if (sdr_config->tx_only) {
			fprintf(stderr, "Conflicting options: --rx cannot be combined with --sdr-tx-only\n");
			exit(0);
		}
		sdr_config->rx_only = 1;
	}
	if (tx && !rx) {
		if (sdr_config->rx_only) {
			fprintf(stderr, "Conflicting options: --tx cannot be combined with --sdr-rx-only\n");
			exit(0);
		}
		sdr_config->tx_only = 1;
	}

	/* now we have buffer size and sample rate 
	 * buffer size is proportional to INPUT sample rate (SDR rate)
	 */
	buffer_size = input_samplerate * dsp_buffer / 1000;

	/* Set polyphase resampler mode before init */
	if (use_polyphase)
		radio_set_polyphase(1);

	rc = radio_init(&radio, buffer_size, input_samplerate, frequency, tx_wave_file, rx_wave_file, (tx) ? tx_audiodev : NULL, (rx) ? rx_audiodev : NULL, modulation, bandwidth, deviation, modulation_index, time_constant_us, volume, stereo, rds, rds2, sca_67k, sca_92k, rds_debug, rds_verbose, am_compandor, fm_compandor, rds_force_rbds);
	if (rc < 0) {
		fprintf(stderr, "Failed to initialize radio with given options, exitting!\n");
		exit(0);
	}

	/* Configure AFC if enabled */
	if (use_afc && modulation == MODULATION_FM) {
		radio_afc_set_time_constant(&radio, afc_tc_ms / 1000.0);
		radio_afc_set_max_correction(&radio, afc_max_hz);
		radio_afc_enable(&radio, 1);
	}

	/* Initialize signal meter (FFT-based, same approach as scan engine).
	 * Uses 4096-point FFT with ±20 kHz integration and processing gain.
	 * Output: SNR in dB (signal - noise).
	 * The dbf_offset is a fallback before noise floor is established. */
	meter = signal_meter_init(input_samplerate, 50.0);
	if (meter)
		signal_meter_register_thread(meter, "main", gettid());

	/* NOTE: spectral frequency registration moved after sdr_open_channelizer()
	 * because sdr_status_init() inside sdr_open zeroes the entire sdr_status
	 * struct, wiping any previously registered frequencies. */

	/* Set up RDS file output if requested */
	if (rds_hexrds_file) {
		rc = rds_decoder_set_hexrds_file(&radio.rds_dec, rds_hexrds_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open hexrds output file, exitting!\n");
			exit(0);
		}
	}
	if (rds_bitstream_file) {
		rc = rds_decoder_set_bitstream_file(&radio.rds_dec, rds_bitstream_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open bitstream output file, exitting!\n");
			exit(0);
		}
	}

	/* Set up RDS TX file input if requested (bypasses encoder) */
	if (rds_tx_hexrds_file && rds_tx_bitstream_file) {
		fprintf(stderr, "Cannot use both --rds-tx-hexrds-file and --rds-tx-bitstream-file\n");
		exit(0);
	}
	if (rds_tx_hexrds_file) {
		if (!tx) {
			fprintf(stderr, "Warning: --rds-tx-hexrds-file requires --tx mode\n");
		}
		rc = rds_enc_set_tx_hexrds_file(&radio.rds_enc, rds_tx_hexrds_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open TX hexrds input file, exitting!\n");
			exit(0);
		}
	}
	if (rds_tx_bitstream_file) {
		if (!tx) {
			fprintf(stderr, "Warning: --rds-tx-bitstream-file requires --tx mode\n");
		}
		rc = rds_enc_set_tx_bitstream_file(&radio.rds_enc, rds_tx_bitstream_file);
		if (rc < 0) {
			fprintf(stderr, "Failed to open TX bitstream input file, exitting!\n");
			exit(0);
		}
	}

	/* Enable RDS paging if requested */
	if (rds_paging) {
		rds_enc_paging_set_enabled(&radio.rds_enc, 1);
		rds_enc_paging_set_rpc(&radio.rds_enc, rds_paging_rpc);
		rds_enc_paging_set_opc(&radio.rds_enc, rds_paging_opc);
		rds_enc_paging_set_pac(&radio.rds_enc, rds_paging_pac);
		/* International paging defaults (EN 50067 M.3.5.7) */
		rds_enc_paging_set_cc(&radio.rds_enc, rds_paging_cc);
		/* Enable paging decoder so 7A/13A are decoded as paging, not ODA */
		radio.rds_dec.paging_enabled = 1;
		/* Create named pipe for paging commands */
		unlink(rds_paging_pipe_path);
		rc = mkfifo(rds_paging_pipe_path, 0666);
		if (rc < 0) {
			fprintf(stderr, "Failed to create paging FIFO '%s': %s\n",
				rds_paging_pipe_path, strerror(errno));
		} else {
			paging_pipe_fd = open(rds_paging_pipe_path, O_RDWR | O_NONBLOCK);
			if (paging_pipe_fd < 0)
				fprintf(stderr, "Failed to open paging FIFO '%s': %s\n",
					rds_paging_pipe_path, strerror(errno));
			else {
				char rpc_buf[80];
				printf("RDS Paging enabled: %s, OPC=%d%s, PAC=%d%s",
				       rds_paging_rpc_desc(rds_paging_rpc, rpc_buf, sizeof(rpc_buf)),
				       rds_paging_opc,
				       rds_paging_opc ? "" : " (no enhanced paging)",
				       rds_paging_pac,
				       rds_paging_pac ? "" : " (all areas)");
				if (rds_paging_cc) {
					const char *cname = mcc_name(rds_paging_cc);
					printf(", intl MCC=%d (%s)",
					       rds_paging_cc, cname ? cname : "unknown");
				}
				printf(", pipe: %s\n", rds_paging_pipe_path);
			}
		}
	}

	/* Initialize protocol servers */
	memset(&xdr_srv, 0, sizeof(xdr_srv));
	memset(&rdsspy_srv, 0, sizeof(rdsspy_srv));
	memset(&uecp_srv, 0, sizeof(uecp_srv));
	memset(&asciig_srv, 0, sizeof(asciig_srv));
	memset(&rigctl_srv, 0, sizeof(rigctl_srv));

	/* RX-only servers: XDR-GTK and RDS-Spy */
	if (xdr_gtk_server) {
		if (!rx)
			fprintf(stderr, "Warning: --xdr-gtk-server is for RX mode (use with --rx)\n");
		rc = rds_server_init(&xdr_srv, xdr_gtk_server, RDS_PROTO_XDR_GTK, NULL);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize XDR-GTK server\n");
			goto error;
		}
		if (xdr_gtk_password)
			rds_server_set_password(&xdr_srv, xdr_gtk_password);
		xdr_srv.freq_khz = (int)(frequency / 1000.0);
		/* Tell XDR-GTK our de-emphasis setting: 0=50µs (EU), 1=75µs (US) */
		if (time_constant_us == 50.0)
			rds_server_set_deemphasis(&xdr_srv, 0);
		else if (time_constant_us == 75.0)
			rds_server_set_deemphasis(&xdr_srv, 1);
		xdr_enabled = 1;
		printf("XDR-GTK protocol server enabled: %s%s\n",
		       xdr_gtk_server, xdr_gtk_password ? " (password protected)" : "");
	}

	if (rdsspy_server) {
		if (!rx)
			fprintf(stderr, "Warning: --rdsspy-server is for RX mode (use with --rx)\n");
		rc = rds_server_init(&rdsspy_srv, rdsspy_server, RDS_PROTO_RDSSPY, NULL);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize RDS Spy server\n");
			goto error;
		}
		rdsspy_enabled = 1;
		printf("RDS Spy protocol server enabled: %s\n", rdsspy_server);
	}

	/* TX-only servers: UECP and ASCII-G */
	if (uecp_server) {
		if (!tx)
			fprintf(stderr, "Warning: --uecp-server is for TX mode (use with --tx)\n");
		rc = rds_server_init(&uecp_srv, uecp_server, RDS_PROTO_UECP, &radio.rds_enc);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize UECP server\n");
			goto error;
		}
		uecp_enabled = 1;
		printf("UECP protocol server enabled: %s\n", uecp_server);
	}

	if (asciig_server) {
		if (!tx)
			fprintf(stderr, "Warning: --asciig-server is for TX mode (use with --tx)\n");
		rc = rds_server_init(&asciig_srv, asciig_server, RDS_PROTO_ASCII_G, &radio.rds_enc);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize ASCII-G server\n");
			goto error;
		}
		asciig_srv.freq_khz = (int)(frequency / 1000.0);
		asciig_enabled = 1;
		printf("ASCII-G protocol server enabled: %s\n", asciig_server);
	}

	/* Rigctl server (hamlib-compatible remote control) */
	if (rigctl_server_endpoint) {
		rc = rigctl_server_init(&rigctl_srv, rigctl_server_endpoint, rigctl_allowed_hosts);
		if (rc < 0) {
			fprintf(stderr, "Failed to initialize rigctl server\n");
			goto error;
		}
		rigctl_server_set_mode_flags(&rigctl_srv, tx, rx);
		rigctl_server_set_frequency(&rigctl_srv, frequency, frequency);
		rigctl_server_set_modulation(&rigctl_srv, modulation, (int)radio.rf_bandwidth, deviation, modulation_index);
		rigctl_server_set_flags(&rigctl_srv, stereo, rds);
		rigctl_server_set_radio(&rigctl_srv, &radio);
		rigctl_server_set_meter(&rigctl_srv, meter);

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
				rigctl_server_set_sdr_caps(&rigctl_srv, &rigctl_caps);
			}
		}

		/* Set up callbacks */
		rigctl_callbacks_t rigctl_cb = {
			.set_rx_freq = rigctl_set_rx_freq,
			.set_tx_freq = rigctl_set_tx_freq,
			.get_rx_freq = rigctl_get_rx_freq,
			.get_tx_freq = rigctl_get_tx_freq,
			.arg = &radio
		};
		rigctl_server_set_callbacks(&rigctl_srv, &rigctl_cb);
		rigctl_enabled = 1;
		printf("Rigctl server enabled: %s\n", rigctl_server_endpoint);
	}

	/* Set up RDS decoder callback for RX protocol servers (XDR-GTK, RDS-Spy) */
	if (xdr_enabled || rdsspy_enabled) {
		rds_cb_ctx.xdr_srv = xdr_enabled ? &xdr_srv : NULL;
		rds_cb_ctx.rdsspy_srv = rdsspy_enabled ? &rdsspy_srv : NULL;
		rds_cb_ctx.asciig_srv = NULL;
		rds_cb_ctx.radio = &radio;
		rds_decoder_set_group_callback(&radio.rds_dec, rds_group_callback, &rds_cb_ctx);
	}

	/* Wire tune callback into XDR-GTK and RDS-Spy servers (RX tuner control) */
	if (xdr_enabled)
		rds_server_set_callbacks(&xdr_srv, radio_tune_cb, radio_setting_cb, &rds_cb_ctx);
	if (rdsspy_enabled)
		rds_server_set_callbacks(&rdsspy_srv, radio_tune_cb, NULL, &rds_cb_ctx);
	/* Wire tune callback into ASCII-G server (TX tuner control) */
	if (asciig_enabled)
		rds_server_set_callbacks(&asciig_srv, radio_tx_tune_cb, NULL, NULL);

	/* Wire spectral scan callback into XDR-GTK server */
	if (xdr_enabled && rx)
		rds_server_set_scan_callback(&xdr_srv, radio_scan_cb, &rds_cb_ctx);


	sendbuff = calloc(buffer_size * 2, sizeof(*sendbuff));
	if (!sendbuff) {
		fprintf(stderr, "No mem!\n");
		goto error;
	}

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
	tx_frequencies[0] = frequency;
	rx_frequencies[0] = frequency;
	am[0] = 0;
	sdr = sdr_open_channelizer(0, NULL, tx_frequencies, rx_frequencies, am, 0, 0.0, input_samplerate, buffer_size, 1.0, 0.0, 0.0, 0.0, (use_channelizer == 1), fast_math);
	if (!sdr)
		goto error;
	sdr_start(sdr);

	/* Configure spectral measurement - center matches SDR tuned frequency */
	sdr_spectral_configure(sdr_get_samplerate(sdr), frequency);
	if (frequency > 0.0)
		spectral_freq_idx = sdr_spectral_register_freq(frequency);

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
	rc = radio_start(&radio);
	if (rc < 0) {
		fprintf(stderr, "Failed to start radio's streaming, exitting!\n");
		goto error_start;
	}

	int tosend, got;
#if 0 /* TX debug logging - uncomment to enable */
	static int main_loop_dbg_cnt = 0;
#endif
	while (!quit) {
		/* Adaptive sleep: shorter when TX buffer needs feeding.
		 * In TX mode the write thread drains the ring buffer continuously;
		 * sleeping a full 1ms risks starving the hardware (UHD underruns).
		 * Use 100us when TX is active to keep the buffer fed.
		 * In RX-only mode, use 250us to reduce ring buffer pressure
		 * and avoid SDR-side overflow on high sample rate devices. */
		if (tx)
			usleep(100);
		else
			usleep(250);
		/* Check paging pipe for new messages */
		if (paging_pipe_fd >= 0)
			paging_pipe_handler(&radio.rds_enc);
		
		/* Poll protocol servers (non-blocking) */
		if (xdr_enabled)
			rds_server_poll(&xdr_srv);
		if (rdsspy_enabled)
			rds_server_poll(&rdsspy_srv);
		if (uecp_enabled)
			rds_server_poll(&uecp_srv);
		if (asciig_enabled)
			rds_server_poll(&asciig_srv);
		if (rigctl_enabled)
			rigctl_server_poll(&rigctl_srv);
		
		got = 0;
		if (rx) {
			got = sdr_read(sdr, (void *)sendbuff, buffer_size, 0, NULL);
			/* Feed raw IQ into scan engine if active (FFT-based, no retuning needed
			 * unless scan range exceeds SDR bandwidth) */
			if (xdr_enabled && xdr_srv.scan_active)
				rds_server_scan_feed_iq(&xdr_srv, sendbuff, got,
							frequency, sdr_get_samplerate(sdr));

			/* Feed IQ into spectral measurement (same data/rate as scan) */
			sdr_spectral_feed_iq(sendbuff, got);

			/* Feed IQ into signal meter for time-domain RMS */
			signal_meter_feed_iq(meter, sendbuff, got);
			/* Use last known SNR estimate for stereo quieting heuristics in radio_rx(). */
			radio_set_rx_snr(&radio, signal_meter_get_snr(meter));

			got = radio_rx(&radio, sendbuff, got);
			if (got < 0)
				break;

			/* Update 19 kHz stereo pilot info from radio_rx() results */
			signal_meter_set_stereo_pilot(meter, radio.rx_pilot_mag_avg,
						      radio.rx_pilot_locked);

			/* Feed FFT-based measurements to signal meter.
			 * Both signal power (at tuned frequency) and noise floor (minimum of offsets)
			 * come from FFT - same method as scan for consistent dBf output. */
			if (sdr_spectral_is_valid()) {
				double nf = sdr_spectral_get_noise_floor();
				signal_meter_set_noise_floor(meter, nf);
				if (spectral_freq_idx >= 0)
					signal_meter_set_signal_power(meter, sdr_spectral_get_power(spectral_freq_idx));
			} else {
				const sdr_status_t *ss = sdr_status_get();
				if (ss && ss->rx.valid)
					signal_meter_set_noise_floor(meter, ss->rx.noise_floor_db);
			}
			/* Refresh SNR after FFT/noise updates for next RX iteration. */
			radio_set_rx_snr(&radio, signal_meter_get_snr(meter));

			/* Feed signal level to XDR-GTK for periodic S reports */
			if (xdr_enabled) {
				double sig = signal_meter_get_level_dbf(meter);
				int is_stereo = radio.stereo && radio.rx_pilot_locked && !radio.rx_forced_mono;
				rds_server_update_signal(&xdr_srv,
					(sig > SIGNAL_METER_NO_VALUE) ? sig : -100.0,
					is_stereo,
					radio.rx_pilot_mag_avg);
			}

			/* Feed signal level to rigctl server */
			if (rigctl_enabled) {
				double sig = signal_meter_get_level_dbfs(meter);
				rigctl_server_set_signal(&rigctl_srv,
					(sig > SIGNAL_METER_NO_VALUE) ? sig : -100.0);
			}

			/* CPU update (self-throttles to ~1/sec via internal wall-clock check) */
			signal_meter_update_cpu(meter);
		}
		
		/* TX processing - only if TX mode is enabled */
		if (tx) {
			tosend = sdr_get_tosend(sdr, buffer_size);
#if 0 /* TX debug logging - uncomment to enable */
			int tosend_raw = tosend;
#endif
			if (tosend > buffer_size)
				tosend = buffer_size;
			if (tosend == 0)
				goto next_char;
#if 0 /* TX debug logging - uncomment to enable */
			/* Log main loop buffer sizes periodically (~1/sec) */
			if (++main_loop_dbg_cnt >= 333) {
				LOGP(DRADIO, LOGL_DEBUG, "Main loop: buffer_size=%d tosend_raw=%d tosend_capped=%d got=%d\n",
				     buffer_size, tosend_raw, tosend, got);
				main_loop_dbg_cnt = 0;
			}
#endif
			/* perform radio modulation */
			tosend = radio_tx(&radio, sendbuff, tosend);
			if (tosend <= 0) {
				if (tosend < 0)
					break;
				continue; /* radio_tx returned 0: no audio to process, skip SDR write */
			}
			/* write to SDR */
			sdr_write(sdr, (void *)sendbuff, NULL, tosend, NULL, NULL, 0);
		}

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
			if (rx && rds)
				rds_decoder_status(&radio.rds_dec);
			goto next_char;
		case 'f':
			/* cycle RDS presets */
			if (tx && rds)
				rds_next_preset(&radio);
			goto next_char;
		case '1':
			/* flip RT A/B flag */
			if (tx && rds)
				rds_flip_rt_ab(&radio);
			goto next_char;
		case '2':
			/* flip RT+ item_running */
			if (tx && rds)
				rds_flip_rtplus_item_running(&radio);
			goto next_char;
		case '3':
			/* flip RT+ item_toggle */
			if (tx && rds)
				rds_flip_rtplus_toggle(&radio);
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

	free(sendbuff);
	signal_meter_free(meter);
	meter = NULL;
	if (sdr)
		sdr_close(sdr);
	/* Clean up paging pipe */
	if (paging_pipe_fd >= 0)
		close(paging_pipe_fd);
	unlink(rds_paging_pipe_path);
	
	/* Clean up protocol servers */
	if (xdr_enabled)
		rds_server_cleanup(&xdr_srv);
	if (rdsspy_enabled)
		rds_server_cleanup(&rdsspy_srv);
	if (uecp_enabled)
		rds_server_cleanup(&uecp_srv);
	if (asciig_enabled)
		rds_server_cleanup(&asciig_srv);
	if (rigctl_enabled)
		rigctl_server_cleanup(&rigctl_srv);
	
	radio_exit(&radio);

	/* global exits */
	fm_exit();
	am_exit();

	options_free();

	return 0;
}

void osmo_cc_set_log_cat(int __attribute__((unused)) cc_log_cat) {}

