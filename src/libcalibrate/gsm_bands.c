/* GSM Band Definitions and ARFCN Conversion
 *
 * (C) 2026 Osmocom-analog contributors
 * GPLv3
 */

#include "calibrate.h"

/* GSM-900 (primary, E-GSM): ARFCN 0-124, 975-1023
 * Downlink: 935-960 MHz (primary), 925-935 MHz (extended)
 */
#define GSM900_ARFCN_START      0
#define GSM900_ARFCN_END        124
#define GSM900_DL_BASE          935.0e6
#define GSM900_CHANNEL_SPACING  200000.0

/* GSM-1800 (DCS): ARFCN 512-885
 * Downlink: 1805.2-1879.8 MHz
 */
#define GSM1800_ARFCN_START     512
#define GSM1800_ARFCN_END       885
#define GSM1800_DL_BASE         1805.2e6

/* GSM-1900 (PCS): ARFCN 512-810
 * Downlink: 1930.2-1989.8 MHz
 */
#define GSM1900_ARFCN_START     512
#define GSM1900_ARFCN_END       810
#define GSM1900_DL_BASE         1930.2e6

/* GSM-850: ARFCN 128-251
 * Downlink: 869.2-893.8 MHz
 */
#define GSM850_ARFCN_START      128
#define GSM850_ARFCN_END        251
#define GSM850_DL_BASE          869.2e6

/* Convert ARFCN to downlink frequency in Hz */
double arfcn_to_freq(int arfcn, int gsm_band)
{
	switch (gsm_band) {
	case GSM_BAND_900:
		if (arfcn >= 0 && arfcn <= 124)
			return GSM900_DL_BASE + arfcn * GSM900_CHANNEL_SPACING;
		/* Extended GSM-900 (ARFCN 975-1023) maps to 925-935 MHz */
		if (arfcn >= 975 && arfcn <= 1023)
			return 925.0e6 + (arfcn - 975) * GSM900_CHANNEL_SPACING;
		break;
	case GSM_BAND_1800:
		if (arfcn >= 512 && arfcn <= 885)
			return GSM1800_DL_BASE + (arfcn - 512) * GSM900_CHANNEL_SPACING;
		break;
	case GSM_BAND_1900:
		if (arfcn >= 512 && arfcn <= 810)
			return GSM1900_DL_BASE + (arfcn - 512) * GSM900_CHANNEL_SPACING;
		break;
	case GSM_BAND_850:
		if (arfcn >= 128 && arfcn <= 251)
			return GSM850_DL_BASE + (arfcn - 128) * GSM900_CHANNEL_SPACING;
		break;
	}
	return 0.0;
}

/* Get band parameters for wideband scanning */
void gsm_band_params(int gsm_band, double *center_freq, double *bandwidth,
                     int *arfcn_start, int *arfcn_end)
{
	switch (gsm_band) {
	case GSM_BAND_900:
		/* Cover 935-960 MHz (primary GSM-900 downlink) */
		*center_freq = 947.5e6;
		*bandwidth = 25.0e6;
		*arfcn_start = GSM900_ARFCN_START;
		*arfcn_end = GSM900_ARFCN_END;
		break;
	case GSM_BAND_1800:
		/* Split into two scans due to wide bandwidth */
		*center_freq = 1842.5e6;
		*bandwidth = 75.0e6;
		*arfcn_start = GSM1800_ARFCN_START;
		*arfcn_end = GSM1800_ARFCN_END;
		break;
	case GSM_BAND_1900:
		*center_freq = 1960.0e6;
		*bandwidth = 60.0e6;
		*arfcn_start = GSM1900_ARFCN_START;
		*arfcn_end = GSM1900_ARFCN_END;
		break;
	case GSM_BAND_850:
		*center_freq = 881.5e6;
		*bandwidth = 25.0e6;
		*arfcn_start = GSM850_ARFCN_START;
		*arfcn_end = GSM850_ARFCN_END;
		break;
	default:
		*center_freq = 0;
		*bandwidth = 0;
		*arfcn_start = 0;
		*arfcn_end = 0;
	}
}
