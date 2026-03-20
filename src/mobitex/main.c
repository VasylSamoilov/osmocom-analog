/* Mobitex main
 *
 * CLI option handling and transceiver lifecycle.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/main_mobile.h"
#include "../liboptions/options.h"
#include "../libfm/fm.h"
#include "mobitex.h"
#include "dsp.h"
#include "frame.h"

static int tx = 0;
static int rx = 1;
static double rx_frequency = 0.0;
static double tx_frequency = 0.0;
static int fbi = 4;
static int rx_channel = -1;
static int tx_channel = -1;
static double deviation = 2000.0;
static double polarity = 1.0;
static int scrambling = 1;
static int ramnet = 1;
static const char *direction = "base";
static int baudrate = 8000;
static int frsync_index = 1;  /* Default: index 1 = 0xB433 (US RAM Mobile Data / BellSouth) */
static int base_id = 0;
static int area_id = 0;
static uint32_t base_man = 1;
static double svp_interval = 1.0;
static int svp_type = 1;

/* SVP1/SVP3 parameters (roaming + link, base station broadcasts to mobiles) */
static int txpow = 0;
static int rssi_proc = 1;
static int rssi_period = 5;
static int scan_time = 15;
static int bad_base = 1;
static int good_base = 5;
static int better_base = 6;
static int upfreq_ch = 0;
static int dofreq_ch = 0;

/* SVP6 parameters — despite the battery-saving names inherited from PDW,
 * the Palm VIIx ROM actually stores these as roaming parameters:
 *   byte 7 (cycle_time)       → RSSI_PROC    (0=Frame, 1=Continuous)
 *   byte 8 (time_to_next)     → RSSI_PERIOD   (×20ms)
 *   byte 9 (transaction_time) → SCAN_TIME     (×100ms)
 *   byte 10 (eval_current)    → BAD_BASE      (dBµV)
 *   byte 11 (eval_others)     → GOOD_BASE     (dBµV)
 *   byte 13>>4 (deep_sleep_inhibit) → BETTER_BASE (dB)
 * Defaults match the MIS PA01 example response. */
static int cycle_time = 1;
static int time_to_next = 5;
static int transaction_time = 15;
static int eval_current = 1;
static int eval_others = 5;
static int deep_sleep_inhibit = 6;

/* FRI parameters (base station free cycle) */
static int slot_length = 17;
static int free_slots = 5;
static int rand_slots = 5;
static int max_rep = 3;
static int max_access = 1;
static int timeout_sec = 10;
static double fri_interval = 0.5;

void print_help(const char *arg0)
{
	main_mobile_print_help(arg0, "--rx-freq <MHz> | --rx-channel <num> --fbi <code>");
	printf("\nFrequency options (Mobitex is FDD — TX and RX use separate frequencies):\n");
	printf(" -T --tx\n");
	printf("        Enable transmitter.\n");
	printf(" -R --rx\n");
	printf("        Enable receiver. (default)\n");
	printf("    --rx-freq <MHz>\n");
	printf("        Receive frequency in MHz (what this instance listens on).\n");
	printf("    --tx-freq <MHz>\n");
	printf("        Transmit frequency in MHz (what this instance transmits on).\n");
	printf("    --fbi <code>\n");
	printf("        Frequency Band Identification for channel number lookup (default %d).\n", fbi);
	printf("        0=800MHz (819+ch*6.25kHz), 3=400MHz (380+ch*12.5kHz),\n");
	printf("        4=900MHz (890+ch*12.5kHz), 5=400MHz (380+ch*12.5kHz).\n");
	printf("    --rx-channel <number>\n");
	printf("        RX channel number (0-8191). Computes RX frequency from FBI.\n");
	printf("    --tx-channel <number>\n");
	printf("        TX channel number (0-8191). Computes TX frequency from FBI.\n");
	printf("\n");
	printf("    Channel number examples (FBI 4, 900 MHz, BellSouth/RAM):\n");
	printf("        ch 3661 = 935.7625 MHz (downlink, base station transmits)\n");
	printf("        ch  541 = 896.7625 MHz (uplink, base station receives)\n");
	printf("        Duplex offset: 3120 channels = 39 MHz\n");
	printf("    To monitor a base station downlink (RX-only, default):\n");
	printf("        mobitex --rx-channel 3661 --fbi 4\n");
	printf("    To operate as base station (TX on downlink, RX on uplink):\n");
	printf("        mobitex -T -R --tx-channel 3661 --rx-channel 541 --fbi 4\n");
	printf("\nModulation options:\n");
	printf(" -D --deviation <Hz>\n");
	printf("        FSK deviation in Hz (default %.0f).\n", deviation);
	printf(" -P --polarity -1 | negative | 1 | positive\n");
	printf("        FSK polarity (default positive).\n");
	printf("\nProtocol options:\n");
	printf("    --scrambling 0 | 1\n");
	printf("        Enable/disable bit scrambling (default %d).\n", scrambling);
	printf("    --ramnet 0 | 1\n");
	printf("        Enable/disable RAMnet CFlags checking (default %d).\n", ramnet);
	printf("    --direction base | mobile\n");
	printf("        Expected transmission direction (default %s).\n", direction);
	printf("    --baud 8000 | 1200\n");
	printf("        Baud rate (default %d).\n", baudrate);
	printf("    --frsync <index>\n");
	printf("        Frame sync index 0-17, -1=any (default %d = 0xB433 US RAM).\n", frsync_index);
	printf("\nTX options (base station mode):\n");
	printf("    --base-id <id>\n");
	printf("        6-bit Base ID for TX headers 0-63 (default %d).\n", base_id);
	printf("    --area-id <id>\n");
	printf("        6-bit Area ID for TX headers 0-63 (default %d).\n", area_id);
	printf("    --base-man <hex>\n");
	printf("        24-bit Base station MAN for SVP link control (default %06X).\n", base_man);
	printf("    --svp-interval <seconds>\n");
	printf("        Seconds between SVP frames when idle (default %.1f).\n", svp_interval);
	printf("    --svp-type <type>\n");
	printf("        Initial sweep type 0-6, auto-rotates SVP1/SVP6 (default %d).\n", svp_type);
	printf("    --txpow <dB>\n");
	printf("        TX power reduction for mobiles, 0-255 dB (default %d).\n", txpow);
	printf("    --rssi-proc <0|1>\n");
	printf("        RSSI measurement method: 0=FRAME, 1=Continuous (default %d, SVP1/SVP3).\n", rssi_proc);
	printf("    --rssi-period <n>\n");
	printf("        Roaming algorithm time, n×20ms (default %d, SVP1/SVP3).\n", rssi_period);
	printf("    --scan-time <n>\n");
	printf("        Scan time for surrounding channels, n×100ms (default %d, SVP1/SVP3).\n", scan_time);
	printf("    --bad-base <n>\n");
	printf("        Lowest usable signal threshold dBµV (default %d, SVP1/SVP3).\n", bad_base);
	printf("    --good-base <n>\n");
	printf("        Acceptable signal threshold dBµV (default %d, SVP1/SVP3).\n", good_base);
	printf("    --better-base <n>\n");
	printf("        Signal improvement threshold for roaming dB (default %d, SVP1/SVP3).\n", better_base);
	printf("    --upfreq <channel>\n");
	printf("        Uplink channel number for SVP2/4 (default %d).\n", upfreq_ch);
	printf("    --dofreq <channel>\n");
	printf("        Downlink channel number for SVP2/4 (default %d).\n", dofreq_ch);
	printf("    --cycle-time <n>\n");
	printf("        Battery-saving cycle time, n×250ms, 0=disabled (default %d, SVP6).\n", cycle_time);
	printf("    --time-to-next <n>\n");
	printf("        Time from SVP6 to operating state, n×250ms (default %d, SVP6).\n", time_to_next);
	printf("    --transaction-time <n>\n");
	printf("        Time in operating state after ACK, n×250ms, 255=max (default %d, SVP6).\n", transaction_time);
	printf("    --eval-current <n>\n");
	printf("        Evaluate current channel time in seconds (default %d, SVP6).\n", eval_current);
	printf("    --eval-others <n>\n");
	printf("        Evaluate other channels in RSSI periods (default %d, SVP6).\n", eval_others);
	printf("    --deep-sleep-inhibit <n>\n");
	printf("        Deep sleep inhibit count 0-15 SVP6 frames (default %d, SVP6).\n", deep_sleep_inhibit);
	printf("    --slot-length <n>\n");
	printf("        FRI slot length in units of 32/bitrate (default %d).\n", slot_length);
	printf("    --free-slots <n>\n");
	printf("        Total FRI slots (default %d).\n", free_slots);
	printf("    --rand-slots <n>\n");
	printf("        Random FRI slots (default %d).\n", rand_slots);
	printf("    --max-rep <n>\n");
	printf("        Max retransmissions before roaming eval (default %d).\n", max_rep);
	printf("    --max-access <n>\n");
	printf("        Max blocks in MRM without access request (default %d).\n", max_access);
	printf("    --timeout <seconds>\n");
	printf("        Access timeout in seconds (default %d).\n", timeout_sec);
	printf("    --fri-interval <seconds>\n");
	printf("        Seconds between FRI frames when idle (default %.1f, 0=disable).\n", fri_interval);
	main_mobile_print_station_id();
	main_mobile_print_hotkeys();
}

#define OPT_RX_FREQ     256
#define OPT_TX_FREQ     257
#define OPT_SCRAMBLING  258
#define OPT_RAMNET      259
#define OPT_DIRECTION   260
#define OPT_BAUD        261
#define OPT_FRSYNC      262
#define OPT_BASE_ID     263
#define OPT_AREA_ID     264
#define OPT_BASE_MAN    270
#define OPT_SVP_INT     265
#define OPT_SVP_TYPE    266
#define OPT_FBI         267
#define OPT_RX_CHANNEL  268
#define OPT_TX_CHANNEL  269
#define OPT_TXPOW       271
#define OPT_RSSI_PROC   272
#define OPT_RSSI_PERIOD 273
#define OPT_SCAN_TIME   274
#define OPT_BAD_BASE    275
#define OPT_GOOD_BASE   276
#define OPT_BETTER_BASE 277
#define OPT_UPFREQ      278
#define OPT_DOFREQ      279
#define OPT_SLOT_LEN    280
#define OPT_FREE_SLOTS  281
#define OPT_RAND_SLOTS  282
#define OPT_MAX_REP     283
#define OPT_MAX_ACCESS  284
#define OPT_TIMEOUT     285
#define OPT_FRI_INT     286
#define OPT_CYCLE_TIME  287
#define OPT_TIME_TO_NEXT 288
#define OPT_TRANS_TIME  289
#define OPT_EVAL_CUR    290
#define OPT_EVAL_OTH    291
#define OPT_DEEP_SLEEP  292

static void add_options(void)
{
	main_mobile_add_options();
	option_add('T', "tx", 0);
	option_add('R', "rx", 0);
	option_add(OPT_RX_FREQ, "rx-freq", 1);
	option_add(OPT_TX_FREQ, "tx-freq", 1);
	option_add('D', "deviation", 1);
	option_add('P', "polarity", 1);
	option_add(OPT_SCRAMBLING, "scrambling", 1);
	option_add(OPT_RAMNET, "ramnet", 1);
	option_add(OPT_DIRECTION, "direction", 1);
	option_add(OPT_BAUD, "baud", 1);
	option_add(OPT_FRSYNC, "frsync", 1);
	option_add(OPT_BASE_ID, "base-id", 1);
	option_add(OPT_AREA_ID, "area-id", 1);
	option_add(OPT_BASE_MAN, "base-man", 1);
	option_add(OPT_SVP_INT, "svp-interval", 1);
	option_add(OPT_SVP_TYPE, "svp-type", 1);
	option_add(OPT_FBI, "fbi", 1);
	option_add(OPT_RX_CHANNEL, "rx-channel", 1);
	option_add(OPT_TX_CHANNEL, "tx-channel", 1);
	option_add(OPT_TXPOW, "txpow", 1);
	option_add(OPT_RSSI_PROC, "rssi-proc", 1);
	option_add(OPT_RSSI_PERIOD, "rssi-period", 1);
	option_add(OPT_SCAN_TIME, "scan-time", 1);
	option_add(OPT_BAD_BASE, "bad-base", 1);
	option_add(OPT_GOOD_BASE, "good-base", 1);
	option_add(OPT_BETTER_BASE, "better-base", 1);
	option_add(OPT_UPFREQ, "upfreq", 1);
	option_add(OPT_DOFREQ, "dofreq", 1);
	option_add(OPT_SLOT_LEN, "slot-length", 1);
	option_add(OPT_FREE_SLOTS, "free-slots", 1);
	option_add(OPT_RAND_SLOTS, "rand-slots", 1);
	option_add(OPT_MAX_REP, "max-rep", 1);
	option_add(OPT_MAX_ACCESS, "max-access", 1);
	option_add(OPT_TIMEOUT, "timeout", 1);
	option_add(OPT_FRI_INT, "fri-interval", 1);
	option_add(OPT_CYCLE_TIME, "cycle-time", 1);
	option_add(OPT_TIME_TO_NEXT, "time-to-next", 1);
	option_add(OPT_TRANS_TIME, "transaction-time", 1);
	option_add(OPT_EVAL_CUR, "eval-current", 1);
	option_add(OPT_EVAL_OTH, "eval-others", 1);
	option_add(OPT_DEEP_SLEEP, "deep-sleep-inhibit", 1);
}

static int handle_options(int short_option, int argi, char **argv)
{
	switch (short_option) {
	case 'T':
		tx = 1;
		break;
	case 'R':
		rx = 1;
		break;
	case OPT_RX_FREQ:
		rx_frequency = atof(argv[argi]) * 1e6; /* MHz to Hz */
		break;
	case OPT_TX_FREQ:
		tx_frequency = atof(argv[argi]) * 1e6; /* MHz to Hz */
		break;
	case 'D':
		deviation = atof(argv[argi]);
		break;
	case 'P':
		if (argv[argi][0] == 'n' || argv[argi][0] == 'N')
			polarity = -1.0;
		else if (argv[argi][0] == 'p' || argv[argi][0] == 'P')
			polarity = 1.0;
		else if (atoi(argv[argi]) == -1)
			polarity = -1.0;
		else if (atoi(argv[argi]) == 1)
			polarity = 1.0;
		else {
			fprintf(stderr, "Given polarity is not positive nor negative, use '-h' for help.\n");
			return -EINVAL;
		}
		break;
	case OPT_SCRAMBLING:
		scrambling = atoi(argv[argi]);
		break;
	case OPT_RAMNET:
		ramnet = atoi(argv[argi]);
		break;
	case OPT_DIRECTION:
		direction = argv[argi];
		break;
	case OPT_BAUD:
		baudrate = atoi(argv[argi]);
		if (baudrate != 8000 && baudrate != 1200) {
			fprintf(stderr, "Baud rate must be 8000 or 1200.\n");
			return -EINVAL;
		}
		break;
	case OPT_FRSYNC:
		frsync_index = atoi(argv[argi]);
		break;
	case OPT_BASE_ID:
		base_id = atoi(argv[argi]);
		break;
	case OPT_AREA_ID:
		area_id = atoi(argv[argi]);
		break;
	case OPT_BASE_MAN:
		base_man = (uint32_t)strtoul(argv[argi], NULL, 16);
		break;
	case OPT_SVP_INT:
		svp_interval = atof(argv[argi]);
		break;
	case OPT_SVP_TYPE:
		svp_type = atoi(argv[argi]);
		break;
	case OPT_FBI:
		fbi = atoi(argv[argi]);
		break;
	case OPT_RX_CHANNEL:
		rx_channel = atoi(argv[argi]);
		break;
	case OPT_TX_CHANNEL:
		tx_channel = atoi(argv[argi]);
		break;
	case OPT_TXPOW:
		txpow = atoi(argv[argi]);
		break;
	case OPT_RSSI_PROC:
		rssi_proc = atoi(argv[argi]);
		break;
	case OPT_RSSI_PERIOD:
		rssi_period = atoi(argv[argi]);
		break;
	case OPT_SCAN_TIME:
		scan_time = atoi(argv[argi]);
		break;
	case OPT_BAD_BASE:
		bad_base = atoi(argv[argi]);
		break;
	case OPT_GOOD_BASE:
		good_base = atoi(argv[argi]);
		break;
	case OPT_BETTER_BASE:
		better_base = atoi(argv[argi]);
		break;
	case OPT_UPFREQ:
		upfreq_ch = atoi(argv[argi]);
		break;
	case OPT_DOFREQ:
		dofreq_ch = atoi(argv[argi]);
		break;
	case OPT_SLOT_LEN:
		slot_length = atoi(argv[argi]);
		break;
	case OPT_FREE_SLOTS:
		free_slots = atoi(argv[argi]);
		break;
	case OPT_RAND_SLOTS:
		rand_slots = atoi(argv[argi]);
		break;
	case OPT_MAX_REP:
		max_rep = atoi(argv[argi]);
		break;
	case OPT_MAX_ACCESS:
		max_access = atoi(argv[argi]);
		break;
	case OPT_TIMEOUT:
		timeout_sec = atoi(argv[argi]);
		break;
	case OPT_FRI_INT:
		fri_interval = atof(argv[argi]);
		break;
	case OPT_CYCLE_TIME:
		cycle_time = atoi(argv[argi]);
		break;
	case OPT_TIME_TO_NEXT:
		time_to_next = atoi(argv[argi]);
		break;
	case OPT_TRANS_TIME:
		transaction_time = atoi(argv[argi]);
		break;
	case OPT_EVAL_CUR:
		eval_current = atoi(argv[argi]);
		break;
	case OPT_EVAL_OTH:
		eval_others = atoi(argv[argi]);
		break;
	case OPT_DEEP_SLEEP:
		deep_sleep_inhibit = atoi(argv[argi]);
		break;
	default:
		return main_mobile_handle_options(short_option, argi, argv);
	}

	return 1;
}

int msg_receive(const char *text)
{
	printf("Received: %s\n", text);
	return 0;
}

int main(int argc, char *argv[])
{
	int rc, argi;
	uint16_t bitsync;

	/* mobitex does not use emphasis */
	uses_emphasis = 0;

	/* init mobile interface */
	main_mobile_init("0123456789", NULL, NULL, NULL, NULL);

	/* handle options / config file */
	add_options();
	rc = options_config_file(argc, argv, "~/.osmocom/analog/mobitex.conf", handle_options);
	if (rc < 0)
		return 0;
	argi = options_command_line(argc, argv, handle_options);
	if (argi <= 0)
		return argi;

	/* select bitsync based on direction and baud rate */
	if (baudrate == 1200) {
		if (direction && direction[0] == 'm')
			bitsync = MOBITEX_1200_BITSYNC_MOBILE;
		else
			bitsync = MOBITEX_1200_BITSYNC_BASE;
	} else {
		if (direction && direction[0] == 'm')
			bitsync = MOBITEX_BITSYNC_MOBILE;
		else
			bitsync = MOBITEX_BITSYNC_BASE;
	}

	/* default to RX-only */
	if (!tx && !rx)
		rx = 1;

	/* TX & RX if loopback */
	if (loopback)
		tx = rx = 1;

	/* inits */
	fm_init(fast_math);

	/* resolve channel numbers to frequencies via FBI */
	if (rx_channel >= 0) {
		double f = mobitex_fbi_to_freq(fbi, rx_channel);
		if (f < 0) {
			fprintf(stderr, "Unknown FBI %d for --rx-channel. Use 0, 3, 4, or 5.\n", fbi);
			goto fail;
		}
		rx_frequency = f * 1e6;
		printf("RX channel %d (FBI %d) -> %.5f MHz\n", rx_channel, fbi, f);
	}
	if (tx_channel >= 0) {
		double f = mobitex_fbi_to_freq(fbi, tx_channel);
		if (f < 0) {
			fprintf(stderr, "Unknown FBI %d for --tx-channel. Use 0, 3, 4, or 5.\n", fbi);
			goto fail;
		}
		tx_frequency = f * 1e6;
		printf("TX channel %d (FBI %d) -> %.5f MHz\n", tx_channel, fbi, f);
	}

	/* Auto-populate SVP2 downlink channel from TX channel if not
	 * explicitly set.  The mobile uses the downlink channel to tune
	 * its receiver and automatically derives the uplink channel by
	 * subtracting the duplex offset (3120 channels for FBI 4). */
	if (dofreq_ch == 0 && tx_channel >= 0)
		dofreq_ch = tx_channel;

	/* Mobitex is FDD — TX and RX use separate frequencies.
	 * Refuse to operate if both TX and RX are enabled but only one
	 * frequency is given, since they cannot share the same frequency.
	 * When only RX or only TX is active, the missing frequency is
	 * unused and can safely default to 0. */
	if (tx && rx && !loopback) {
		if (rx_frequency > 0.0 && tx_frequency <= 0.0) {
			fprintf(stderr, "Mobitex is FDD: TX enabled but no --tx-freq given. "
			        "Use --tx-freq or disable TX.\n");
			goto fail;
		}
		if (tx_frequency > 0.0 && rx_frequency <= 0.0) {
			fprintf(stderr, "Mobitex is FDD: RX enabled but no --rx-freq given. "
			        "Use --rx-freq or disable RX.\n");
			goto fail;
		}
		if (rx_frequency > 0.0 && tx_frequency > 0.0 &&
		    rx_frequency == tx_frequency) {
			fprintf(stderr, "Mobitex is FDD: TX and RX frequencies must differ.\n");
			goto fail;
		}
	}

	/* create transceiver instance */
	if (rx_frequency > 0.0 || tx_frequency > 0.0) {
		rc = mobitex_create("mobitex", rx_frequency, tx_frequency,
		                    num_device ? dsp_device[0] : "", use_sdr,
		                    dsp_samplerate, rx_gain, tx_gain,
		                    tx, rx, baudrate,
		                    deviation, polarity,
		                    scrambling, ramnet,
		                    bitsync, frsync_index,
		                    base_id, area_id, base_man,
		                    svp_interval, svp_type, fbi,
		                    txpow, rssi_proc, rssi_period,
		                    scan_time, bad_base, good_base,
		                    better_base, upfreq_ch, dofreq_ch,
		                    cycle_time, time_to_next,
		                    transaction_time, eval_current,
		                    eval_others, deep_sleep_inhibit,
		                    slot_length, free_slots, rand_slots,
		                    max_rep, max_access, timeout_sec,
		                    fri_interval,
		                    write_rx_wave, write_tx_wave,
		                    read_rx_wave, read_tx_wave,
		                    loopback);
		if (rc < 0) {
			fprintf(stderr, "Failed to create Mobitex transceiver. Quitting!\n");
			goto fail;
		}
	}

	main_mobile_loop("mobitex", &quit, NULL, "");

fail:
	/* destroy transceiver instance */
	while (sender_head)
		mobitex_destroy(sender_head);

	/* exits */
	main_mobile_exit();
	fm_exit();

	options_free();

	return 0;
}
