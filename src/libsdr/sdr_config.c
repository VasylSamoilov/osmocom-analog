/* Config for SDR
 *
 * (C) 2017 by Andreas Eversberg <jolly@eversberg.eu>
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

enum paging_signal;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liboptions/options.h"
#include "../liblogging/logging.h"
#include "sdr.h"
#include "sdr_config.h"
#ifdef HAVE_SOAPY
#include "soapy.h"
#endif
#ifdef HAVE_UHD
#include "uhd.h"
#endif
#ifdef HAVE_RPITX
#include "rpitx.h"
#endif

static int got_init = 0;
extern int use_sdr;
sdr_config_t *sdr_config = NULL;

void sdr_config_init(double lo_offset)
{
	sdr_config = calloc(1, sizeof(*sdr_config));
	memset(sdr_config, 0, sizeof(*sdr_config));
	sdr_config->device_args = "";
	sdr_config->stream_args = "";
	sdr_config->tune_args = "";
	sdr_config->lo_offset = lo_offset;
	sdr_config->timestamps = 1;

	got_init = 1;
}

void sdr_config_print_help(void)
{
	printf("\nSDR options:\n");
	/*      -                                                                             - */
#ifdef HAVE_UHD
	printf("    --sdr-uhd\n");
	printf("        Force UHD driver\n");
#endif
#ifdef HAVE_SOAPY
	printf("    --sdr-soapy\n");
	printf("        Force SoapySDR driver\n");
#endif
#ifdef HAVE_RPITX
	printf("    --sdr-rpitx\n");
	printf("        Force rpitx driver (Raspberry Pi GPIO TX, TX-only)\n");
#endif
	printf("    --sdr-channel <channel #>\n");
	printf("        Give channel number for multi channel SDR device (default = %d)\n", sdr_config->channel);
	printf("    --sdr-device-args <args>\n");
	printf("    --sdr-stream-args <args>\n");
	printf("    --sdr-tune-args <args>\n");
	printf("        Optional SDR device arguments, separated by comma\n");
	printf("        e.g. --sdr-device-args <key>=<value>[,<key>=<value>[,...]]\n");
	printf("    --sdr-samplerate <samplerate>\n");
	printf("        Sample rate to use with SDR. By default it equals the regular sample\n");
	printf("        rate.\n");
	printf("    --sdr-lo-offset <Hz>\n");
	printf("        Give frequency offset in Hz to move the local oscillator away from the\n");
	printf("        target frequency. (default = %.0f)\n", sdr_config->lo_offset);
	printf("    --sdr-bandwidth <bandwidth>\n");
	printf("        Give IF filter bandwidth to use. If not, sample rate is used.\n");
	printf("    --sdr-rx-antenna <name>\n");
	printf("        SDR device's RX antenna name, use 'list' to get a list\n");
	printf("    --sdr-tx-antenna <name>\n");
	printf("        SDR device's TX antenna name, use 'list' to get a list\n");
	printf("    --sdr-clock-source <name>\n");
	printf("        SDR device's clock sourc name, use 'list' to get a list\n");
	printf("    --sdr-rx-gain <gain>\n");
	printf("        SDR device's RX gain in dB (default = %.1f)\n", sdr_config->rx_gain);
	printf("    --sdr-tx-gain <gain>\n");
	printf("        SDR device's TX gain in dB (default = %.1f)\n", sdr_config->tx_gain);
	printf("    --write-iq-rx-wave <file>\n");
	printf("        Write received IQ data to given wave file.\n");
	printf("    --write-iq-tx-wave <file>\n");
	printf("        Write transmitted IQ data to given wave file.\n");
	printf("    --read-iq-rx-wave <file>\n");
	printf("        Replace received IQ data by given wave file.\n");
	printf("    --read-iq-tx-wave <file>\n");
	printf("        Replace transmitted IQ data by given wave file.\n");
	printf("    --sdr-swap-links\n");
	printf("        Swap RX and TX frequencies for loopback tests over the air.\n");
	printf("    --sdr-timestamps 1 | 0\n");
	printf("        Use TX timestamps on UHD device. (default = %d)\n", sdr_config->timestamps);
	printf("\nSplit device options (mutually exclusive with unified options above):\n");
	printf("    --sdr-tx-device-args <args>\n");
	printf("        TX SDR device arguments (enables split mode)\n");
	printf("    --sdr-tx-samplerate <rate>\n");
	printf("        TX sample rate\n");
	printf("    --sdr-tx-bandwidth <Hz>\n");
	printf("        TX IF bandwidth\n");
	printf("    --sdr-tx-lo-offset <Hz>\n");
	printf("        TX LO frequency offset\n");
	printf("    --sdr-rx-device-args <args>\n");
	printf("        RX SDR device arguments (enables split mode)\n");
	printf("    --sdr-rx-samplerate <rate>\n");
	printf("        RX sample rate\n");
	printf("    --sdr-rx-bandwidth <Hz>\n");
	printf("        RX IF bandwidth\n");
	printf("    --sdr-rx-lo-offset <Hz>\n");
	printf("        RX LO frequency offset\n");
	printf("\n  Mode options:\n");
	printf("    --sdr-tx-only\n");
	printf("        TX-only mode (no RX device needed)\n");
	printf("    --sdr-rx-only\n");
	printf("        RX-only mode (no TX device needed)\n");
	printf("\n  Per-device overrides (split mode only):\n");
	printf("    --sdr-tx-channel <channel #>\n");
	printf("        TX device channel number\n");
	printf("    --sdr-rx-channel <channel #>\n");
	printf("        RX device channel number\n");
	printf("    --sdr-tx-stream-args <args>\n");
	printf("        TX device stream arguments\n");
	printf("    --sdr-rx-stream-args <args>\n");
	printf("        RX device stream arguments\n");
	printf("    --sdr-tx-tune-args <args>\n");
	printf("        TX device tune arguments\n");
	printf("    --sdr-rx-tune-args <args>\n");
	printf("        RX device tune arguments\n");
	printf("    --sdr-tx-clock-source <name>\n");
	printf("        TX device clock source\n");
	printf("    --sdr-rx-clock-source <name>\n");
	printf("        RX device clock source\n");
	printf("\n  Per-device driver selection (split mode only):\n");
#ifdef HAVE_UHD
	printf("    --sdr-tx-uhd\n");
	printf("        Use UHD driver for TX device\n");
	printf("    --sdr-rx-uhd\n");
	printf("        Use UHD driver for RX device\n");
#endif
#ifdef HAVE_SOAPY
	printf("    --sdr-tx-soapy\n");
	printf("        Use SoapySDR driver for TX device\n");
	printf("    --sdr-rx-soapy\n");
	printf("        Use SoapySDR driver for RX device\n");
#endif
#ifdef HAVE_RPITX
	printf("    --sdr-tx-rpitx\n");
	printf("        Use rpitx driver for TX device (Raspberry Pi GPIO, TX-only)\n");
#endif
	printf("\nUpconverter options:\n");
	printf("    --sdr-upconverter <Hz>\n");
	printf("        Upconverter offset (Hz) for both TX and RX. Can be positive or negative.\n");
	printf("        SDR tunes to (target + offset). E.g., 125000000 for Ham-It-Up.\n");
	printf("    --sdr-tx-upconverter <Hz>\n");
	printf("        TX-specific upconverter offset (Hz).\n");
	printf("    --sdr-rx-upconverter <Hz>\n");
	printf("        RX-specific upconverter offset (Hz).\n");
	printf("\nFrequency correction:\n");
	printf("    --sdr-ppm <correction>\n");
	printf("        Roles of crystal oscillator in PPM (parts per million).\n");
	printf("        Positive value means crystal is fast, negative means slow.\n");
	printf("        Applies to both TX and RX, or unified device.\n");
	printf("    --sdr-tx-ppm <correction>\n");
	printf("        PPM correction for TX device (split mode only).\n");
	printf("    --sdr-rx-ppm <correction>\n");
	printf("        PPM correction for RX device (split mode only).\n");
}

void sdr_config_print_hotkeys(void)
{
	printf("Press 'q' key to toggle display of RX I/Q vector.\n");
	printf("Press 's' key to toggle display of RX spectrum.\n");
	printf("Press 'b' key to remove DC level.\n");
}

#define	OPT_SDR_UHD		1500
#define	OPT_SDR_SOAPY		1501
#define	OPT_SDR_CHANNEL		1502
#define	OPT_SDR_DEVICE_ARGS	1503
#define	OPT_SDR_STREAM_ARGS	1504
#define	OPT_SDR_TUNE_ARGS	1505
#define	OPT_SDR_RX_ANTENNA	1506
#define	OPT_SDR_TX_ANTENNA	1507
#define	OPT_SDR_CLOCK_SOURCE	1508
#define	OPT_SDR_RX_GAIN		1509
#define	OPT_SDR_TX_GAIN		1510
#define	OPT_SDR_SAMPLERATE	1511
#define	OPT_SDR_LO_OFFSET	1512
#define	OPT_SDR_BANDWIDTH	1513
#define	OPT_WRITE_IQ_RX_WAVE	1514
#define	OPT_WRITE_IQ_TX_WAVE	1515
#define	OPT_READ_IQ_RX_WAVE	1516
#define	OPT_READ_IQ_TX_WAVE	1517
#define	OPT_SDR_SWAP_LINKS	1518
#define	OPT_SDR_TIMESTAMPS	1519
/* Split device options */
#define	OPT_SDR_TX_DEVICE	1520
#define	OPT_SDR_TX_SAMPLERATE	1521
#define	OPT_SDR_TX_BANDWIDTH	1522
#define	OPT_SDR_TX_LO_OFFSET	1523
#define	OPT_SDR_RX_DEVICE	1524
#define	OPT_SDR_RX_SAMPLERATE	1525
#define	OPT_SDR_RX_BANDWIDTH	1526
#define	OPT_SDR_RX_LO_OFFSET	1527
/* Upconverter options */
#define	OPT_SDR_UPCONVERTER	1528
#define	OPT_SDR_TX_UPCONVERTER	1529
#define	OPT_SDR_RX_UPCONVERTER	1530
/* TX/RX mode and per-device override options */
#define	OPT_SDR_TX_ONLY		1531
#define	OPT_SDR_RX_ONLY		1532
#define	OPT_SDR_TX_CHANNEL	1533
#define	OPT_SDR_RX_CHANNEL	1534
#define	OPT_SDR_TX_STREAM_ARGS	1535
#define	OPT_SDR_RX_STREAM_ARGS	1536
#define	OPT_SDR_TX_TUNE_ARGS	1537
#define	OPT_SDR_RX_TUNE_ARGS	1538
#define	OPT_SDR_TX_CLOCK_SOURCE	1539
#define	OPT_SDR_RX_CLOCK_SOURCE	1540
#define	OPT_SDR_TX_UHD		1541
#define	OPT_SDR_TX_SOAPY	1542
#define	OPT_SDR_RX_UHD		1543
#define	OPT_SDR_RX_SOAPY	1544
#ifdef HAVE_RPITX
#define	OPT_SDR_RPITX		1545
#define	OPT_SDR_TX_RPITX	1546
#endif
/* PPM frequency correction */
#define	OPT_SDR_PPM		1547
#define	OPT_SDR_TX_PPM		1548
#define	OPT_SDR_RX_PPM		1549

void sdr_config_add_options(void)
{
	option_add(OPT_SDR_UHD, "sdr-uhd", 0);
	option_add(OPT_SDR_SOAPY, "sdr-soapy", 0);
#ifdef HAVE_RPITX
	option_add(OPT_SDR_RPITX, "sdr-rpitx", 0);
#endif
	option_add(OPT_SDR_CHANNEL, "sdr-channel", 1);
	option_add(OPT_SDR_DEVICE_ARGS, "sdr-device-args", 1);
	option_add(OPT_SDR_STREAM_ARGS, "sdr-stream-args", 1);
	option_add(OPT_SDR_TUNE_ARGS, "sdr-tune-args", 1);
	option_add(OPT_SDR_SAMPLERATE, "sdr-samplerate", 1);
	option_add(OPT_SDR_LO_OFFSET, "sdr-lo-offset", 1);
	option_add(OPT_SDR_BANDWIDTH, "sdr-bandwidth", 1);
	option_add(OPT_SDR_RX_ANTENNA, "sdr-rx-antenna", 1);
	option_add(OPT_SDR_TX_ANTENNA, "sdr-tx-antenna", 1);
	option_add(OPT_SDR_CLOCK_SOURCE, "sdr-clock-source", 1);
	option_add(OPT_SDR_RX_GAIN, "sdr-rx-gain", 1);
	option_add(OPT_SDR_TX_GAIN, "sdr-tx-gain", 1);
	option_add(OPT_WRITE_IQ_RX_WAVE, "write-iq-rx-wave", 1);
	option_add(OPT_WRITE_IQ_TX_WAVE, "write-iq-tx-wave", 1);
	option_add(OPT_READ_IQ_RX_WAVE, "read-iq-rx-wave", 1);
	option_add(OPT_READ_IQ_TX_WAVE, "read-iq-tx-wave", 1);
	option_add(OPT_SDR_SWAP_LINKS, "sdr-swap-links", 0);
	option_add(OPT_SDR_TIMESTAMPS, "sdr-timestamps", 1);
	/* Split device options */
	option_add(OPT_SDR_TX_DEVICE, "sdr-tx-device-args", 1);
	option_add(OPT_SDR_TX_SAMPLERATE, "sdr-tx-samplerate", 1);
	option_add(OPT_SDR_TX_BANDWIDTH, "sdr-tx-bandwidth", 1);
	option_add(OPT_SDR_TX_LO_OFFSET, "sdr-tx-lo-offset", 1);
	option_add(OPT_SDR_RX_DEVICE, "sdr-rx-device-args", 1);
	option_add(OPT_SDR_RX_SAMPLERATE, "sdr-rx-samplerate", 1);
	option_add(OPT_SDR_RX_BANDWIDTH, "sdr-rx-bandwidth", 1);
	option_add(OPT_SDR_RX_LO_OFFSET, "sdr-rx-lo-offset", 1);
	/* Upconverter options */
	option_add(OPT_SDR_UPCONVERTER, "sdr-upconverter", 1);
	option_add(OPT_SDR_TX_UPCONVERTER, "sdr-tx-upconverter", 1);
	option_add(OPT_SDR_RX_UPCONVERTER, "sdr-rx-upconverter", 1);
	/* TX/RX mode and per-device override options */
	option_add(OPT_SDR_TX_ONLY, "sdr-tx-only", 0);
	option_add(OPT_SDR_RX_ONLY, "sdr-rx-only", 0);
	option_add(OPT_SDR_TX_CHANNEL, "sdr-tx-channel", 1);
	option_add(OPT_SDR_RX_CHANNEL, "sdr-rx-channel", 1);
	option_add(OPT_SDR_TX_STREAM_ARGS, "sdr-tx-stream-args", 1);
	option_add(OPT_SDR_RX_STREAM_ARGS, "sdr-rx-stream-args", 1);
	option_add(OPT_SDR_TX_TUNE_ARGS, "sdr-tx-tune-args", 1);
	option_add(OPT_SDR_RX_TUNE_ARGS, "sdr-rx-tune-args", 1);
	option_add(OPT_SDR_TX_CLOCK_SOURCE, "sdr-tx-clock-source", 1);
	option_add(OPT_SDR_RX_CLOCK_SOURCE, "sdr-rx-clock-source", 1);
	option_add(OPT_SDR_TX_UHD, "sdr-tx-uhd", 0);
	option_add(OPT_SDR_TX_SOAPY, "sdr-tx-soapy", 0);
	option_add(OPT_SDR_RX_UHD, "sdr-rx-uhd", 0);
	option_add(OPT_SDR_RX_SOAPY, "sdr-rx-soapy", 0);
#ifdef HAVE_RPITX
	option_add(OPT_SDR_TX_RPITX, "sdr-tx-rpitx", 0);
#endif
	/* PPM frequency correction */
	option_add(OPT_SDR_PPM, "sdr-ppm", 1);
	option_add(OPT_SDR_TX_PPM, "sdr-tx-ppm", 1);
	option_add(OPT_SDR_RX_PPM, "sdr-rx-ppm", 1);
}

int sdr_config_handle_options(int short_option, int argi, char **argv)
{
	switch (short_option) {
	case OPT_SDR_UHD:
#ifdef HAVE_UHD
		sdr_config->uhd = 1;
		use_sdr = 1;
#else
		fprintf(stderr, "UHD SDR support not compiled in!\n");
		return -EINVAL;
#endif
		break;
	case OPT_SDR_SOAPY:
#ifdef HAVE_SOAPY
		sdr_config->soapy = 1;
		use_sdr = 1;
#else
		fprintf(stderr, "SoapySDR support not compiled in!\n");
		return -EINVAL;
#endif
		break;
#ifdef HAVE_RPITX
	case OPT_SDR_RPITX:
		sdr_config->rpitx = 1;
		use_sdr = 1;
		break;
#endif
	case OPT_SDR_CHANNEL:
		sdr_config->channel = atoi(argv[argi]);
		break;
	case OPT_SDR_DEVICE_ARGS:
		sdr_config->device_args = options_strdup(argv[argi]);
		break;
	case OPT_SDR_STREAM_ARGS:
		sdr_config->stream_args = options_strdup(argv[argi]);
		break;
	case OPT_SDR_TUNE_ARGS:
		sdr_config->tune_args = options_strdup(argv[argi]);
		break;
	case OPT_SDR_SAMPLERATE:
		sdr_config->samplerate = atoi(argv[argi]);
		sdr_config->sdr_samplerate_given = 1;
		break;
	case OPT_SDR_LO_OFFSET:
		sdr_config->lo_offset = atof(argv[argi]);
		break;
	case OPT_SDR_BANDWIDTH:
		sdr_config->bandwidth = atof(argv[argi]);
		break;
	case OPT_SDR_RX_ANTENNA:
		sdr_config->rx_antenna = options_strdup(argv[argi]);
		break;
	case OPT_SDR_TX_ANTENNA:
		sdr_config->tx_antenna = options_strdup(argv[argi]);
		break;
	case OPT_SDR_CLOCK_SOURCE:
		sdr_config->clock_source = options_strdup(argv[argi]);
		break;
	case OPT_SDR_RX_GAIN:
		sdr_config->rx_gain = atof(argv[argi]);
		break;
	case OPT_SDR_TX_GAIN:
		sdr_config->tx_gain = atof(argv[argi]);
		break;
	case OPT_WRITE_IQ_RX_WAVE:
		sdr_config->write_iq_rx_wave = options_strdup(argv[argi]);
		break;
	case OPT_WRITE_IQ_TX_WAVE:
		sdr_config->write_iq_tx_wave = options_strdup(argv[argi]);
		break;
	case OPT_READ_IQ_RX_WAVE:
		sdr_config->read_iq_rx_wave = options_strdup(argv[argi]);
		break;
	case OPT_READ_IQ_TX_WAVE:
		sdr_config->read_iq_tx_wave = options_strdup(argv[argi]);
		break;
	case OPT_SDR_SWAP_LINKS:
		sdr_config->swap_links = 1;
		break;
	case OPT_SDR_TIMESTAMPS:
		sdr_config->timestamps = atoi(argv[argi]);
		break;
	/* Split device options */
	case OPT_SDR_TX_DEVICE:
		sdr_config->tx_device_args = options_strdup(argv[argi]);
		sdr_config->split_mode = 1;
		break;
	case OPT_SDR_TX_SAMPLERATE:
		sdr_config->tx_samplerate = atoi(argv[argi]);
		break;
	case OPT_SDR_TX_BANDWIDTH:
		sdr_config->tx_bandwidth = atof(argv[argi]);
		break;
	case OPT_SDR_TX_LO_OFFSET:
		sdr_config->tx_lo_offset = atof(argv[argi]);
		break;
	case OPT_SDR_RX_DEVICE:
		sdr_config->rx_device_args = options_strdup(argv[argi]);
		sdr_config->split_mode = 1;
		break;
	case OPT_SDR_RX_SAMPLERATE:
		sdr_config->rx_samplerate = atoi(argv[argi]);
		break;
	case OPT_SDR_RX_BANDWIDTH:
		sdr_config->rx_bandwidth = atof(argv[argi]);
		break;
	case OPT_SDR_RX_LO_OFFSET:
		sdr_config->rx_lo_offset = atof(argv[argi]);
		break;
	/* Upconverter options */
	case OPT_SDR_UPCONVERTER:
		sdr_config->tx_upconverter = atof(argv[argi]);
		sdr_config->rx_upconverter = sdr_config->tx_upconverter;
		break;
	case OPT_SDR_TX_UPCONVERTER:
		sdr_config->tx_upconverter = atof(argv[argi]);
		break;
	case OPT_SDR_RX_UPCONVERTER:
		sdr_config->rx_upconverter = atof(argv[argi]);
		break;
	/* TX/RX mode options */
	case OPT_SDR_TX_ONLY:
		sdr_config->tx_only = 1;
		break;
	case OPT_SDR_RX_ONLY:
		sdr_config->rx_only = 1;
		break;
	/* Per-device split override options */
	case OPT_SDR_TX_CHANNEL:
		sdr_config->tx_channel = atoi(argv[argi]);
		sdr_config->tx_channel_given = 1;
		break;
	case OPT_SDR_RX_CHANNEL:
		sdr_config->rx_channel = atoi(argv[argi]);
		sdr_config->rx_channel_given = 1;
		break;
	case OPT_SDR_TX_STREAM_ARGS:
		sdr_config->tx_stream_args = options_strdup(argv[argi]);
		break;
	case OPT_SDR_RX_STREAM_ARGS:
		sdr_config->rx_stream_args = options_strdup(argv[argi]);
		break;
	case OPT_SDR_TX_TUNE_ARGS:
		sdr_config->tx_tune_args = options_strdup(argv[argi]);
		break;
	case OPT_SDR_RX_TUNE_ARGS:
		sdr_config->rx_tune_args = options_strdup(argv[argi]);
		break;
	case OPT_SDR_TX_CLOCK_SOURCE:
		sdr_config->tx_clock_source = options_strdup(argv[argi]);
		break;
	case OPT_SDR_RX_CLOCK_SOURCE:
		sdr_config->rx_clock_source = options_strdup(argv[argi]);
		break;
	/* Per-device driver selection options */
	case OPT_SDR_TX_UHD:
#ifdef HAVE_UHD
		sdr_config->tx_uhd = 1;
		sdr_config->split_mode = 1;
		use_sdr = 1;
#else
		fprintf(stderr, "UHD SDR support not compiled in!\n");
		return -EINVAL;
#endif
		break;
	case OPT_SDR_TX_SOAPY:
#ifdef HAVE_SOAPY
		sdr_config->tx_soapy = 1;
		sdr_config->split_mode = 1;
		use_sdr = 1;
#else
		fprintf(stderr, "SoapySDR support not compiled in!\n");
		return -EINVAL;
#endif
		break;
#ifdef HAVE_RPITX
	case OPT_SDR_TX_RPITX:
		sdr_config->tx_rpitx = 1;
		sdr_config->split_mode = 1;
		use_sdr = 1;
		break;
#endif
	case OPT_SDR_RX_UHD:
#ifdef HAVE_UHD
		sdr_config->rx_uhd = 1;
		sdr_config->split_mode = 1;
		use_sdr = 1;
#else
		fprintf(stderr, "UHD SDR support not compiled in!\n");
		return -EINVAL;
#endif
		break;
	case OPT_SDR_RX_SOAPY:
#ifdef HAVE_SOAPY
		sdr_config->rx_soapy = 1;
		sdr_config->split_mode = 1;
		use_sdr = 1;
#else
		fprintf(stderr, "SoapySDR support not compiled in!\n");
		return -EINVAL;
#endif
		break;
	/* PPM frequency correction */
	case OPT_SDR_PPM:
		sdr_config->ppm = atof(argv[argi]);
		sdr_config->tx_ppm = sdr_config->ppm;
		sdr_config->rx_ppm = sdr_config->ppm;
		break;
	case OPT_SDR_TX_PPM:
		sdr_config->tx_ppm = atof(argv[argi]);
		break;
	case OPT_SDR_RX_PPM:
		sdr_config->rx_ppm = atof(argv[argi]);
		break;
	default:
		return -EINVAL;
	}

	return 1;
}

int sdr_configure(int samplerate)
{
	int have_unified_driver = sdr_config->uhd || sdr_config->soapy
#ifdef HAVE_RPITX
		|| sdr_config->rpitx
#endif
		;
	int have_split_driver = sdr_config->tx_uhd || sdr_config->tx_soapy ||
#ifdef HAVE_RPITX
	                        sdr_config->tx_rpitx ||
#endif
	                        sdr_config->rx_uhd || sdr_config->rx_soapy;

	if (!got_init) {
		fprintf(stderr, "sdr_config_init was not called, please fix!\n");
		abort();
	}

	/* no sdr selected -> return 0 */
	if (!have_unified_driver && !have_split_driver)
		return 0;

	if ((sdr_config->uhd == 1 && sdr_config->soapy == 1)) {
		fprintf(stderr, "You must choose which one you want: --sdr-uhd or --sdr-soapy\n");
		exit(0);
	}
#ifdef HAVE_RPITX
	if ((sdr_config->uhd == 1 && sdr_config->rpitx == 1) || (sdr_config->soapy == 1 && sdr_config->rpitx == 1)) {
		fprintf(stderr, "You must choose one driver: --sdr-uhd, --sdr-soapy, or --sdr-rpitx\n");
		exit(0);
	}
	/* rpitx is TX-only: force tx_only mode in unified mode */
	if (sdr_config->rpitx && !sdr_config->split_mode) {
		LOGP(DSDR, LOGL_NOTICE, "rpitx driver: forcing TX-only mode\n");
		sdr_config->tx_only = 1;
	}
#endif

	/* tx_only and rx_only are mutually exclusive */
	if (sdr_config->tx_only && sdr_config->rx_only) {
		LOGP(DSDR, LOGL_ERROR, "Cannot use --sdr-tx-only and --sdr-rx-only together\n");
		fprintf(stderr, "Cannot use --sdr-tx-only and --sdr-rx-only together\n");
		exit(0);
	}

	/* tx_only/rx_only cannot be combined with split_mode when using separate TX/RX devices.
	 * However, we allow rx_only with rx-specific driver (e.g., --sdr-rx-soapy) and
	 * tx_only with tx-specific driver (e.g., --sdr-tx-soapy) since that's a valid use case
	 * for RX-only or TX-only SDR devices like RTL-SDR. */
	if (sdr_config->split_mode) {
		/* Check if we have actual split devices (both TX and RX specified) */
		int have_tx_device = sdr_config->tx_device_args || sdr_config->tx_uhd || sdr_config->tx_soapy
#ifdef HAVE_RPITX
			|| sdr_config->tx_rpitx
#endif
			;
		int have_rx_device = sdr_config->rx_device_args || sdr_config->rx_uhd || sdr_config->rx_soapy;
		
		/* Only error if trying to use tx_only/rx_only with actual split TX+RX devices */
		if (sdr_config->tx_only && have_tx_device && have_rx_device) {
			LOGP(DSDR, LOGL_ERROR, "Cannot combine --sdr-tx-only with both TX and RX device specifications\n");
			fprintf(stderr, "Cannot combine --sdr-tx-only with both TX and RX device specifications\n");
			exit(0);
		}
		if (sdr_config->rx_only && have_tx_device && have_rx_device) {
			LOGP(DSDR, LOGL_ERROR, "Cannot combine --sdr-rx-only with both TX and RX device specifications\n");
			fprintf(stderr, "Cannot combine --sdr-rx-only with both TX and RX device specifications\n");
			exit(0);
		}
		
		/* If rx_only with only RX driver specified, convert to unified mode for simplicity */
		if (sdr_config->rx_only && have_rx_device && !have_tx_device) {
			/* This is valid: RX-only with RX-specific driver like RTL-SDR */
			LOGP(DSDR, LOGL_NOTICE, "RX-only mode: converting to unified mode with RX driver\n");
			sdr_config->split_mode = 0;
			/* Copy RX settings to unified settings */
			if (sdr_config->rx_device_args &&
			    (!sdr_config->device_args || sdr_config->device_args[0] == '\0'))
				sdr_config->device_args = sdr_config->rx_device_args;
			if (sdr_config->rx_soapy) {
				sdr_config->soapy = 1;
				sdr_config->rx_soapy = 0;
			}
			if (sdr_config->rx_uhd) {
				sdr_config->uhd = 1;
				sdr_config->rx_uhd = 0;
			}
			sdr_config->rx_device_args = NULL;
		}
		/* If tx_only with only TX driver specified, convert to unified mode for simplicity */
		if (sdr_config->tx_only && have_tx_device && !have_rx_device) {
			/* This is valid: TX-only with TX-specific driver */
			LOGP(DSDR, LOGL_NOTICE, "TX-only mode: converting to unified mode with TX driver\n");
			sdr_config->split_mode = 0;
			/* Copy TX settings to unified settings */
			if (sdr_config->tx_device_args &&
			    (!sdr_config->device_args || sdr_config->device_args[0] == '\0'))
				sdr_config->device_args = sdr_config->tx_device_args;
			if (sdr_config->tx_soapy) {
				sdr_config->soapy = 1;
				sdr_config->tx_soapy = 0;
			}
			if (sdr_config->tx_uhd) {
				sdr_config->uhd = 1;
				sdr_config->tx_uhd = 0;
			}
#ifdef HAVE_RPITX
			sdr_config->tx_rpitx = 0;
#endif
			sdr_config->tx_device_args = NULL;
		}
	}

	/* Per-device split options require split_mode */
	if (!sdr_config->split_mode) {
		if (sdr_config->tx_channel_given || sdr_config->rx_channel_given) {
			fprintf(stderr, "Option --sdr-tx-channel/--sdr-rx-channel requires split device mode (--sdr-tx-device-args/--sdr-rx-device-args)\n");
			exit(0);
		}
		if (sdr_config->tx_stream_args || sdr_config->rx_stream_args) {
			fprintf(stderr, "Option --sdr-tx-stream-args/--sdr-rx-stream-args requires split device mode\n");
			exit(0);
		}
		if (sdr_config->tx_tune_args || sdr_config->rx_tune_args) {
			fprintf(stderr, "Option --sdr-tx-tune-args/--sdr-rx-tune-args requires split device mode\n");
			exit(0);
		}
		if (sdr_config->tx_clock_source || sdr_config->rx_clock_source) {
			fprintf(stderr, "Option --sdr-tx-clock-source/--sdr-rx-clock-source requires split device mode\n");
			exit(0);
		}
		if (sdr_config->tx_uhd || sdr_config->tx_soapy || sdr_config->rx_uhd || sdr_config->rx_soapy
#ifdef HAVE_RPITX
		    || sdr_config->tx_rpitx
#endif
		) {
			fprintf(stderr, "Per-device driver options (--sdr-tx-uhd/--sdr-tx-soapy/--sdr-rx-uhd/--sdr-rx-soapy"
#ifdef HAVE_RPITX
				"/--sdr-tx-rpitx"
#endif
				") require split device mode\n");
			exit(0);
		}
	}

	/* Conflicting per-device driver flags */
	if (sdr_config->tx_uhd + sdr_config->tx_soapy
#ifdef HAVE_RPITX
	    + sdr_config->tx_rpitx
#endif
	    > 1) {
		fprintf(stderr, "Cannot use more than one of --sdr-tx-uhd, --sdr-tx-soapy"
#ifdef HAVE_RPITX
			", --sdr-tx-rpitx"
#endif
			"\n");
		exit(0);
	}
	if (sdr_config->rx_uhd && sdr_config->rx_soapy) {
		fprintf(stderr, "Cannot use both --sdr-rx-uhd and --sdr-rx-soapy\n");
		exit(0);
	}

	/* Application-level mode restriction */
	if (sdr_check_separate_device_support(sdr_config->tx_only, sdr_config->rx_only, sdr_config->split_mode)) {
		fprintf(stderr, "This application requires a single full-duplex SDR device.\n");
		exit(0);
	}

	/* Check mutual exclusivity: unified vs split mode */
	if (sdr_config->split_mode && sdr_config->device_args[0] != '\0') {
		fprintf(stderr, "Cannot use --sdr-device-args with --sdr-tx-device-args or --sdr-rx-device-args (mutually exclusive)\n");
		exit(0);
	}

	/* Validate split mode has at least one device or driver */
	if (sdr_config->split_mode) {
		int have_tx = sdr_config->tx_device_args || sdr_config->tx_uhd || sdr_config->tx_soapy
#ifdef HAVE_RPITX
			|| sdr_config->tx_rpitx
#endif
			;
		int have_rx = sdr_config->rx_device_args || sdr_config->rx_uhd || sdr_config->rx_soapy;
		if (!have_tx && !have_rx) {
			fprintf(stderr, "Split mode requires at least --sdr-tx-device-args/--sdr-tx-uhd/--sdr-tx-soapy"
#ifdef HAVE_RPITX
				"/--sdr-tx-rpitx"
#endif
				" or --sdr-rx-device-args/--sdr-rx-uhd/--sdr-rx-soapy\n");
			exit(0);
		}
		/* Set empty device args if not specified (for auto-detect) */
		if (!sdr_config->tx_device_args)
			sdr_config->tx_device_args = "";
		if (!sdr_config->rx_device_args)
			sdr_config->rx_device_args = "";
		/* NOTE: Split sample rate defaults are applied AFTER auto-selection below */
		/* Resolve per-device parameters with unified fallback */
		if (!sdr_config->tx_channel_given)
			sdr_config->tx_channel = sdr_config->channel;
		if (!sdr_config->rx_channel_given)
			sdr_config->rx_channel = sdr_config->channel;
		if (!sdr_config->tx_stream_args)
			sdr_config->tx_stream_args = sdr_config->stream_args;
		if (!sdr_config->rx_stream_args)
			sdr_config->rx_stream_args = sdr_config->stream_args;
		if (!sdr_config->tx_tune_args)
			sdr_config->tx_tune_args = sdr_config->tune_args;
		if (!sdr_config->rx_tune_args)
			sdr_config->rx_tune_args = sdr_config->tune_args;
		if (!sdr_config->tx_clock_source)
			sdr_config->tx_clock_source = sdr_config->clock_source;
		if (!sdr_config->rx_clock_source)
			sdr_config->rx_clock_source = sdr_config->clock_source;
		/* Resolve per-device driver selection */
		if (!sdr_config->tx_uhd && !sdr_config->tx_soapy
#ifdef HAVE_RPITX
		    && !sdr_config->tx_rpitx
#endif
		) {
			sdr_config->tx_uhd = sdr_config->uhd;
			sdr_config->tx_soapy = sdr_config->soapy;
#ifdef HAVE_RPITX
			sdr_config->tx_rpitx = sdr_config->rpitx;
#endif
		}
		if (!sdr_config->rx_uhd && !sdr_config->rx_soapy) {
			sdr_config->rx_uhd = sdr_config->uhd;
			sdr_config->rx_soapy = sdr_config->soapy;
		}
	}

	if (sdr_config->samplerate == 0)
		sdr_config->samplerate = samplerate;
	if (sdr_config->bandwidth == 0.0)
		sdr_config->bandwidth = (double)sdr_config->samplerate;

	/* Auto-select sample rates if bandwidth hint is set and not already done */
	if (sdr_config->bandwidth_hint_set && !sdr_config->auto_selection_done) {
		sdr_rate_info_t rates = {0};
		int have_rates = 0;
		int selected_rate = 0;

		LOGP(DSDR, LOGL_INFO, "Auto-selecting SDR sample rate based on bandwidth hint %.0f Hz\n",
		     sdr_config->bandwidth_hint);

		if (sdr_config->split_mode) {
			/* Split mode: query TX and RX devices separately */
			int tx_rate = 0, rx_rate = 0;
			sdr_rate_info_t bw_info = {0};
			double tx_bw = 0, rx_bw = 0;

			/* If user specified -s, use that as DSP rate constraint so
			 * both TX and RX SDR rates are >= DSP rate (and ideally
			 * integer multiples of it). */
			int split_dsp_rate = sdr_config->samplerate_given ? samplerate : 0;

			/* Query TX device */
			if (sdr_config->tx_device_args && sdr_config->tx_device_args[0]) {
#ifdef HAVE_SOAPY
				if (!tx_rate && sdr_config->tx_soapy) {
					double min_if_bw = 0;
					LOGP(DSDR, LOGL_INFO, "Querying TX SoapySDR device...\n");
					if (soapy_query_device_info(sdr_config->tx_device_args, 0, sdr_config->tx_channel, &rates, &bw_info) == 0) {
						min_if_bw = bw_info.min_rate;
						if (sdr_select_optimal_rate(sdr_config->bandwidth_hint, min_if_bw, split_dsp_rate, &rates, &tx_rate) == 0) {
							LOGP(DSDR, LOGL_INFO, "TX device: selected rate %d Hz\n", tx_rate);
							if (bw_info.num_rates > 0 || bw_info.is_continuous) {
								if (soapy_select_bandwidth(sdr_config->bandwidth_hint, &bw_info, &tx_bw) == 0)
									LOGP(DSDR, LOGL_INFO, "TX device: selected bandwidth %.0f Hz\n", tx_bw);
							}
						}
						sdr_rate_info_free(&rates);
					}
					sdr_rate_info_free(&bw_info);
				}
#endif
#ifdef HAVE_UHD
				if (!tx_rate && sdr_config->tx_uhd) {
					LOGP(DSDR, LOGL_INFO, "Querying TX UHD device for supported rates...\n");
					if (uhd_query_sample_rates(sdr_config->tx_device_args, 0, sdr_config->tx_channel, &rates) == 0) {
						if (sdr_select_optimal_rate(sdr_config->bandwidth_hint, 0, split_dsp_rate, &rates, &tx_rate) == 0)
							LOGP(DSDR, LOGL_INFO, "TX device supports rate: %d Hz\n", tx_rate);
						sdr_rate_info_free(&rates);
					}
				}
#endif
			}

			/* Query RX device */
			if (sdr_config->rx_device_args && sdr_config->rx_device_args[0]) {
#ifdef HAVE_SOAPY
				if (!rx_rate && sdr_config->rx_soapy) {
					double min_if_bw = 0;
					LOGP(DSDR, LOGL_INFO, "Querying RX SoapySDR device...\n");
					if (soapy_query_device_info(sdr_config->rx_device_args, 1, sdr_config->rx_channel, &rates, &bw_info) == 0) {
						min_if_bw = bw_info.min_rate;
						if (sdr_select_optimal_rate(sdr_config->bandwidth_hint, min_if_bw, split_dsp_rate, &rates, &rx_rate) == 0) {
							LOGP(DSDR, LOGL_INFO, "RX device: selected rate %d Hz\n", rx_rate);
							if (bw_info.num_rates > 0 || bw_info.is_continuous) {
								if (soapy_select_bandwidth(sdr_config->bandwidth_hint, &bw_info, &rx_bw) == 0)
									LOGP(DSDR, LOGL_INFO, "RX device: selected bandwidth %.0f Hz\n", rx_bw);
							}
						}
						sdr_rate_info_free(&rates);
					}
					sdr_rate_info_free(&bw_info);
				}
#endif
#ifdef HAVE_UHD
				if (!rx_rate && sdr_config->rx_uhd) {
					LOGP(DSDR, LOGL_INFO, "Querying RX UHD device for supported rates...\n");
					if (uhd_query_sample_rates(sdr_config->rx_device_args, 1, sdr_config->rx_channel, &rates) == 0) {
						if (sdr_select_optimal_rate(sdr_config->bandwidth_hint, 0, split_dsp_rate, &rates, &rx_rate) == 0)
							LOGP(DSDR, LOGL_INFO, "RX device supports rate: %d Hz\n", rx_rate);
						sdr_rate_info_free(&rates);
					}
				}
#endif
			}

			/* Store auto-selected bandwidths for later use */
			sdr_config->tx_auto_bandwidth = tx_bw;
			sdr_config->rx_auto_bandwidth = rx_bw;

			/* Set TX/RX rates if found */
			if (tx_rate > 0 && sdr_config->tx_samplerate == 0)
				sdr_config->tx_samplerate = tx_rate;
			if (rx_rate > 0 && sdr_config->rx_samplerate == 0)
				sdr_config->rx_samplerate = rx_rate;

			/* Set master SDR rate for the sdr.c open check.
			 * When user specified -s, DSP rate is fixed, so master rate
			 * must be >= DSP rate.  Use the smaller of TX/RX (both are
			 * already >= DSP rate thanks to split_dsp_rate constraint).
			 * When user did NOT specify -s, DSP rate will be adjusted
			 * to match, so use the smaller rate as before. */
			if (tx_rate > 0 && rx_rate > 0) {
				selected_rate = (tx_rate < rx_rate) ? tx_rate : rx_rate;
				LOGP(DSDR, LOGL_NOTICE, "Split mode: TX=%d Hz, RX=%d Hz, DSP=%d Hz (no resample on %s path)\n",
				     tx_rate, rx_rate, selected_rate, (tx_rate < rx_rate) ? "TX" : "RX");
			} else if (tx_rate > 0) {
				selected_rate = tx_rate;
				LOGP(DSDR, LOGL_NOTICE, "Split mode: TX=%d Hz, DSP=%d Hz\n", tx_rate, selected_rate);
			} else if (rx_rate > 0) {
				selected_rate = rx_rate;
				LOGP(DSDR, LOGL_NOTICE, "Split mode: RX=%d Hz, DSP=%d Hz\n", rx_rate, selected_rate);
			}

			if (selected_rate > 0)
				sdr_config->samplerate = selected_rate;

		} else {
			/* Unified mode: query single device */
			const char *dev_args = sdr_config->device_args;
			sdr_rate_info_t bw_info = {0};
			double min_if_bw = 0;
			double auto_bw = 0;

#ifdef HAVE_SOAPY
			if (!have_rates && sdr_config->soapy) {
				LOGP(DSDR, LOGL_INFO, "Querying SoapySDR device...\n");
				have_rates = (soapy_query_device_info(dev_args, 0, sdr_config->channel, &rates, &bw_info) == 0);
				if (have_rates)
					min_if_bw = bw_info.min_rate;
			}
#endif
#ifdef HAVE_UHD
			if (!have_rates && sdr_config->uhd) {
				LOGP(DSDR, LOGL_INFO, "Querying UHD device for supported rates...\n");
				have_rates = (uhd_query_sample_rates(dev_args, 0, sdr_config->channel, &rates) == 0);
			}
#endif

			if (have_rates) {
				if (sdr_config->samplerate_given) {
					/* User set -s: find SDR rate that is integer multiple of DSP rate */
					LOGP(DSDR, LOGL_INFO, "User specified DSP rate %d Hz, finding compatible SDR rate\n", samplerate);
					if (sdr_select_optimal_rate(sdr_config->bandwidth_hint, min_if_bw, samplerate, &rates, &selected_rate) == 0) {
						sdr_config->samplerate = selected_rate;
						LOGP(DSDR, LOGL_NOTICE, "Auto-selected SDR rate: %d Hz (oversample=%dx)\n",
						     selected_rate, selected_rate / samplerate);
						/* Select optimal bandwidth >= required bandwidth */
						if (bw_info.num_rates > 0 || bw_info.is_continuous) {
							if (soapy_select_bandwidth(sdr_config->bandwidth_hint, &bw_info, &auto_bw) == 0)
								LOGP(DSDR, LOGL_INFO, "Selected bandwidth: %.0f Hz\n", auto_bw);
						}
					}
				} else {
					/* User didn't set -s: find smallest supported SDR rate */
					LOGP(DSDR, LOGL_INFO, "No explicit DSP rate (-s), finding smallest supported SDR rate\n");
					if (sdr_select_optimal_rate(sdr_config->bandwidth_hint, min_if_bw, 0, &rates, &selected_rate) == 0) {
						sdr_config->samplerate = selected_rate;
						LOGP(DSDR, LOGL_NOTICE, "Auto-selected SDR rate: %d Hz\n", selected_rate);
						LOGP(DSDR, LOGL_NOTICE, "DSP rate will be adjusted to match (oversample=1, no resampling)\n");
						/* Select optimal bandwidth >= required bandwidth */
						if (bw_info.num_rates > 0 || bw_info.is_continuous) {
							if (soapy_select_bandwidth(sdr_config->bandwidth_hint, &bw_info, &auto_bw) == 0)
								LOGP(DSDR, LOGL_INFO, "Selected bandwidth: %.0f Hz\n", auto_bw);
						}
					}
				}
				sdr_rate_info_free(&rates);
			} else {
				LOGP(DSDR, LOGL_NOTICE, "Could not query device rates, using default rate %d Hz\n",
				     sdr_config->samplerate);
			}

			/* Store auto-selected bandwidth (same for TX and RX in unified mode) */
			sdr_config->tx_auto_bandwidth = auto_bw;
			sdr_config->rx_auto_bandwidth = auto_bw;
			sdr_rate_info_free(&bw_info);
		}

		/* Mark auto-selection as done so we don't re-query on second sdr_configure() call */
		sdr_config->auto_selection_done = 1;
	}

	/* Apply split mode sample rate defaults AFTER auto-selection (so they use auto-selected rate) */
	if (sdr_config->split_mode) {
		if (sdr_config->tx_samplerate == 0)
			sdr_config->tx_samplerate = sdr_config->samplerate;
		if (sdr_config->rx_samplerate == 0)
			sdr_config->rx_samplerate = sdr_config->samplerate;
		/* Use auto-selected bandwidth if available, else use samplerate */
		if (sdr_config->tx_bandwidth == 0.0) {
			if (sdr_config->tx_auto_bandwidth > 0)
				sdr_config->tx_bandwidth = sdr_config->tx_auto_bandwidth;
			else
				sdr_config->tx_bandwidth = (double)sdr_config->tx_samplerate;
		}
		if (sdr_config->rx_bandwidth == 0.0) {
			if (sdr_config->rx_auto_bandwidth > 0)
				sdr_config->rx_bandwidth = sdr_config->rx_auto_bandwidth;
			else
				sdr_config->rx_bandwidth = (double)sdr_config->rx_samplerate;
		}
	}

	/* Update bandwidth to match final sample rate if not explicitly set */
	if (sdr_config->bandwidth == (double)samplerate && sdr_config->samplerate != samplerate) {
		/* Use auto-selected bandwidth if available, else use samplerate */
		if (sdr_config->tx_auto_bandwidth > 0)
			sdr_config->bandwidth = sdr_config->tx_auto_bandwidth;
		else
			sdr_config->bandwidth = (double)sdr_config->samplerate;
	}

	/* sdr selected -> return 1 */
	return 1;
}

/* Weak symbol — applications override to restrict modes */
int __attribute__((weak)) sdr_check_separate_device_support(
    int tx_only, int rx_only, int split_mode)
{
	(void)tx_only;
	(void)rx_only;
	(void)split_mode;
	return 0;  /* Default: allow all modes */
}

sdr_mode_t sdr_get_mode(void)
{
	if (sdr_config->tx_only) return SDR_MODE_TX_ONLY;
	if (sdr_config->rx_only) return SDR_MODE_RX_ONLY;
	if (sdr_config->split_mode) return SDR_MODE_SPLIT;
	return SDR_MODE_SINGLE;
}



/* Usable bandwidth of IQ rate (same as in sdr.c) */
#define USABLE_BANDWIDTH	0.75

/* Free rate info structure */
void sdr_rate_info_free(sdr_rate_info_t *info)
{
	if (!info)
		return;
	if (info->rates) {
		free(info->rates);
		info->rates = NULL;
	}
	info->num_rates = 0;
}

/* Check if a rate is supported by the device */
static int rate_is_supported(const sdr_rate_info_t *info, double rate)
{
	int i;

	if (info->is_continuous) {
		return (rate >= info->min_rate && rate <= info->max_rate);
	} else {
		for (i = 0; i < info->num_rates; i++) {
			if (fabs(info->rates[i] - rate) < 1.0)
				return 1;
		}
		return 0;
	}
}

/* Find smallest supported rate >= min_rate */
static double find_smallest_supported(const sdr_rate_info_t *info, double min_rate)
{
	int i;
	double best = 0;

	if (info->is_continuous) {
		/* For continuous range, just use min_rate if it's within range */
		if (min_rate >= info->min_rate && min_rate <= info->max_rate)
			return min_rate;
		if (min_rate < info->min_rate)
			return info->min_rate;
		return 0; /* min_rate exceeds max supported */
	}

	/* For discrete rates, find smallest >= min_rate */
	for (i = 0; i < info->num_rates; i++) {
		if (info->rates[i] >= min_rate) {
			if (best == 0 || info->rates[i] < best)
				best = info->rates[i];
		}
	}
	return best;
}

/**
 * Select optimal SDR sample rate
 *
 * @param min_bandwidth  Minimum required bandwidth (Hz)
 * @param min_if_bw      Minimum IF bandwidth supported by device (Hz), 0 if no constraint
 * @param dsp_rate       DSP sample rate (Hz), 0 if not set by user
 * @param info           Device rate capabilities
 * @param out_rate       Output: selected SDR rate
 * @return 0 on success, -1 on failure
 */
int sdr_select_optimal_rate(double min_bandwidth, double min_if_bw, int dsp_rate, const sdr_rate_info_t *info, int *out_rate)
{
	double min_required;
	int multipliers[] = {1, 2, 4, 8, 16, 32, 64};
	int num_mult = sizeof(multipliers) / sizeof(multipliers[0]);
	int best_rate = 0;
	int i;
	double candidate;

	if (!info || !out_rate)
		return -1;

	/* Minimum rate from bandwidth requirement */
	min_required = min_bandwidth / USABLE_BANDWIDTH;

	/* If device has minimum IF bandwidth, sample rate must be >= that
	 * (because IF bandwidth is typically set to sample rate) */
	if (min_if_bw > 0 && min_if_bw > min_required) {
		LOGP(DSDR, LOGL_INFO, "Rate selection: min_bandwidth=%.0f Hz, min_required=%.0f Hz, min_if_bw=%.0f Hz\n",
		     min_bandwidth, min_required, min_if_bw);
		LOGP(DSDR, LOGL_INFO, "Device min IF bandwidth %.0f Hz > min_required, using IF bandwidth as floor\n",
		     min_if_bw);
		min_required = min_if_bw;
	} else {
		LOGP(DSDR, LOGL_INFO, "Rate selection: min_bandwidth=%.0f Hz, min_required=%.0f Hz (bandwidth/%.2f)\n",
		     min_bandwidth, min_required, USABLE_BANDWIDTH);
	}

	/* If DSP rate is set, SDR rate must be integer multiple */
	if (dsp_rate > 0) {
		LOGP(DSDR, LOGL_INFO, "DSP rate set to %d Hz, finding integer multiple SDR rate\n", dsp_rate);

		/* Ensure min_required is at least dsp_rate */
		if (min_required < dsp_rate)
			min_required = dsp_rate;

		/* Try integer multiples of DSP rate */
		for (i = 0; i < num_mult; i++) {
			candidate = (double)dsp_rate * multipliers[i];

			/* Must meet minimum requirement */
			if (candidate < min_required) {
				LOGP(DSDR, LOGL_DEBUG, "  Candidate %d Hz (x%d) < min_required %.0f Hz, skipping\n",
				     (int)candidate, multipliers[i], min_required);
				continue;
			}

			/* Must be supported by device */
			if (!rate_is_supported(info, candidate)) {
				LOGP(DSDR, LOGL_DEBUG, "  Candidate %d Hz (x%d) not supported by device, skipping\n",
				     (int)candidate, multipliers[i]);
				continue;
			}

			/* First valid candidate is smallest */
			best_rate = (int)candidate;
			LOGP(DSDR, LOGL_INFO, "  Selected SDR rate: %d Hz (x%d oversample)\n",
			     best_rate, multipliers[i]);
			break;
		}

		/* If no integer multiple works, try any supported rate */
		if (best_rate == 0) {
			double any_rate = find_smallest_supported(info, min_required);
			if (any_rate > 0) {
				best_rate = (int)any_rate;
				LOGP(DSDR, LOGL_NOTICE, "No integer multiple of DSP rate %d available, using %d Hz\n",
				     dsp_rate, best_rate);
				LOGP(DSDR, LOGL_NOTICE, "Warning: Non-integer oversample ratio may cause audio artifacts\n");
			}
		}
	} else {
		/* No DSP rate constraint - find smallest supported rate >= min_required */
		LOGP(DSDR, LOGL_INFO, "No DSP rate constraint, finding smallest supported rate >= %.0f Hz\n", min_required);

		double rate = find_smallest_supported(info, min_required);
		if (rate > 0) {
			best_rate = (int)rate;
			LOGP(DSDR, LOGL_INFO, "  Selected rate: %d Hz (will set DSP rate to match)\n", best_rate);
		}
	}

	if (best_rate == 0) {
		LOGP(DSDR, LOGL_ERROR, "No suitable sample rate found! Device range: %.0f - %.0f Hz, required: >= %.0f Hz\n",
		     info->min_rate, info->max_rate, min_required);
		return -1;
	}

	*out_rate = best_rate;
	return 0;
}

/**
 * Set bandwidth hint for auto rate selection
 *
 * Called by applications before sdr_configure() to declare bandwidth requirements.
 *
 * @param max_deviation   Maximum FM deviation (Hz)
 * @param max_modulation  Maximum modulation bandwidth (Hz)
 * @param num_channels    Number of channels
 * @param channel_spacing Channel spacing (Hz), 0 for single channel
 */
void sdr_config_set_bandwidth(double max_deviation, double max_modulation, int num_channels, double channel_spacing)
{
	double channel_bw;
	double total_bw;

	if (!sdr_config) {
		LOGP(DSDR, LOGL_ERROR, "sdr_config_set_bandwidth called before sdr_config_init!\n");
		return;
	}

	/* Per-channel bandwidth: 2 * (deviation + modulation) */
	channel_bw = 2.0 * (max_deviation + max_modulation);

	/* Total bandwidth for all channels */
	if (num_channels > 1 && channel_spacing > 0) {
		/* Multiple channels: span from first to last + channel bandwidth */
		total_bw = (num_channels - 1) * channel_spacing + channel_bw;
	} else {
		total_bw = channel_bw;
	}

	sdr_config->bandwidth_hint = total_bw;
	sdr_config->bandwidth_hint_set = 1;

	LOGP(DSDR, LOGL_INFO, "Bandwidth hint set: %.0f Hz total (%d channels, %.0f Hz each, spacing %.0f Hz)\n",
	     total_bw, num_channels, channel_bw, channel_spacing);
	LOGP(DSDR, LOGL_INFO, "  Per-channel: 2 * (%.1f deviation + %.1f modulation) = %.0f Hz\n",
	     max_deviation, max_modulation, channel_bw);
}


/**
 * Early SDR rate selection - call BEFORE creating senders
 *
 * This function sets the bandwidth hint and calls sdr_configure() to select
 * the optimal SDR sample rate. If auto-selection finds a rate different from
 * the default dsp_samplerate, it updates the passed dsp_samplerate pointer.
 *
 * @param required_bandwidth  Pre-calculated required bandwidth (Hz)
 * @param dsp_samplerate      Pointer to DSP sample rate (updated if auto-selected)
 * @return 0 if no SDR selected, 1 if SDR configured, <0 on error
 */
int sdr_select_rate(double required_bandwidth, int *dsp_samplerate)
{
	int rc;
	int original_rate;

	if (!sdr_config || !dsp_samplerate) {
		LOGP(DSDR, LOGL_ERROR, "sdr_select_rate: invalid parameters\n");
		return -EINVAL;
	}

	original_rate = *dsp_samplerate;

	LOGP(DSDR, LOGL_INFO, "=== SDR Rate Selection ===\n");
	LOGP(DSDR, LOGL_INFO, "Required bandwidth: %.0f Hz (%.1f kHz)\n",
	     required_bandwidth, required_bandwidth / 1e3);
	LOGP(DSDR, LOGL_INFO, "Initial DSP rate: %d Hz\n", original_rate);
	LOGP(DSDR, LOGL_INFO, "User set -s: %s, User set --sdr-samplerate: %s\n",
	     sdr_config->samplerate_given ? "YES" : "NO",
	     sdr_config->sdr_samplerate_given ? "YES" : "NO");

	/* Set bandwidth hint directly (pre-calculated by caller) */
	sdr_config->bandwidth_hint = required_bandwidth;
	sdr_config->bandwidth_hint_set = 1;

	/* Call sdr_configure to do the actual rate selection */
	rc = sdr_configure(original_rate);
	if (rc < 0) {
		LOGP(DSDR, LOGL_ERROR, "sdr_configure failed!\n");
		return rc;
	}
	if (rc == 0) {
		LOGP(DSDR, LOGL_DEBUG, "No SDR selected\n");
		return 0;
	}

	/* Check if auto-selection changed the rate */
	if (!sdr_config->samplerate_given && !sdr_config->sdr_samplerate_given &&
	    sdr_config->bandwidth_hint_set && sdr_config->samplerate != original_rate) {
		LOGP(DSDR, LOGL_NOTICE, "Auto-selected SDR rate: %d Hz (was %d Hz)\n",
		     sdr_config->samplerate, original_rate);
		LOGP(DSDR, LOGL_NOTICE, "Updating DSP rate to match SDR rate (oversample=1, no resampling)\n");
		*dsp_samplerate = sdr_config->samplerate;
	} else {
		LOGP(DSDR, LOGL_INFO, "Using DSP rate: %d Hz, SDR rate: %d Hz\n",
		     *dsp_samplerate, sdr_config->samplerate);
	}

	LOGP(DSDR, LOGL_INFO, "=== SDR Rate Selection Complete ===\n");
	return 1;
}
