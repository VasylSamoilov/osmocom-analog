/* LTE (E-UTRA) operating bands and EARFCN, 3GPP TS 36.101 V18.7.0
 * Table 5.7.3-1 (downlink and uplink channel numbers)
 *
 * F_DL = F_DL_low + 0.1 (N_DL - N_Offs-DL)  [MHz]
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stddef.h>
#include <math.h>
#include "lte_bands.h"

#define F(mhz10) (mhz10)

/* band, DL low (100 kHz), N_Offs-DL, last N_DL, UL low (100 kHz), N_Offs-UL,
 * last N_UL, duplex. Bands 15, 16 and 55..63 are not defined; 64 is
 * reserved. */
const lte_band_t lte_bands[] = {
	{   1, F(21100),     0,   599, F(19200),  18000,  18599, LTE_FDD },
	{   2, F(19300),   600,  1199, F(18500),  18600,  19199, LTE_FDD },
	{   3, F(18050),  1200,  1949, F(17100),  19200,  19949, LTE_FDD },
	{   4, F(21100),  1950,  2399, F(17100),  19950,  20399, LTE_FDD },
	{   5, F( 8690),  2400,  2649, F( 8240),  20400,  20649, LTE_FDD },
	{   6, F( 8750),  2650,  2749, F( 8300),  20650,  20749, LTE_FDD },
	{   7, F(26200),  2750,  3449, F(25000),  20750,  21449, LTE_FDD },
	{   8, F( 9250),  3450,  3799, F( 8800),  21450,  21799, LTE_FDD },
	{   9, F(18449),  3800,  4149, F(17499),  21800,  22149, LTE_FDD },
	{  10, F(21100),  4150,  4749, F(17100),  22150,  22749, LTE_FDD },
	{  11, F(14759),  4750,  4949, F(14279),  22750,  22949, LTE_FDD },
	{  12, F( 7290),  5010,  5179, F( 6990),  23010,  23179, LTE_FDD },
	{  13, F( 7460),  5180,  5279, F( 7770),  23180,  23279, LTE_FDD },
	{  14, F( 7580),  5280,  5379, F( 7880),  23280,  23379, LTE_FDD },
	{  17, F( 7340),  5730,  5849, F( 7040),  23730,  23849, LTE_FDD },
	{  18, F( 8600),  5850,  5999, F( 8150),  23850,  23999, LTE_FDD },
	{  19, F( 8750),  6000,  6149, F( 8300),  24000,  24149, LTE_FDD },
	{  20, F( 7910),  6150,  6449, F( 8320),  24150,  24449, LTE_FDD },
	{  21, F(14959),  6450,  6599, F(14479),  24450,  24599, LTE_FDD },
	{  22, F(35100),  6600,  7399, F(34100),  24600,  25399, LTE_FDD },
	{  23, F(21800),  7500,  7699, F(20000),  25500,  25699, LTE_FDD },
	{  24, F(15250),  7700,  8039, F(16265),  25700,  26039, LTE_FDD },
	{  25, F(19300),  8040,  8689, F(18500),  26040,  26689, LTE_FDD },
	{  26, F( 8590),  8690,  9039, F( 8140),  26690,  27039, LTE_FDD },
	{  27, F( 8520),  9040,  9209, F( 8070),  27040,  27209, LTE_FDD },
	{  28, F( 7580),  9210,  9659, F( 7030),  27210,  27659, LTE_FDD },
	{  29, F( 7170),  9660,  9769,        0,      0,     -1, LTE_SDL },
	{  30, F(23500),  9770,  9869, F(23050),  27660,  27759, LTE_FDD },
	{  31, F( 4625),  9870,  9919, F( 4525),  27760,  27809, LTE_FDD },
	{  32, F(14520),  9920, 10359,        0,      0,     -1, LTE_SDL },
	{  33, F(19000), 36000, 36199, F(19000),  36000,  36199, LTE_TDD },
	{  34, F(20100), 36200, 36349, F(20100),  36200,  36349, LTE_TDD },
	{  35, F(18500), 36350, 36949, F(18500),  36350,  36949, LTE_TDD },
	{  36, F(19300), 36950, 37549, F(19300),  36950,  37549, LTE_TDD },
	{  37, F(19100), 37550, 37749, F(19100),  37550,  37749, LTE_TDD },
	{  38, F(25700), 37750, 38249, F(25700),  37750,  38249, LTE_TDD },
	{  39, F(18800), 38250, 38649, F(18800),  38250,  38649, LTE_TDD },
	{  40, F(23000), 38650, 39649, F(23000),  38650,  39649, LTE_TDD },
	{  41, F(24960), 39650, 41589, F(24960),  39650,  41589, LTE_TDD },
	{  42, F(34000), 41590, 43589, F(34000),  41590,  43589, LTE_TDD },
	{  43, F(36000), 43590, 45589, F(36000),  43590,  45589, LTE_TDD },
	{  44, F( 7030), 45590, 46589, F( 7030),  45590,  46589, LTE_TDD },
	{  45, F(14470), 46590, 46789, F(14470),  46590,  46789, LTE_TDD },
	{  46, F(51500), 46790, 54539, F(51500),  46790,  54539, LTE_TDD },
	{  47, F(58550), 54540, 55239, F(58550),  54540,  55239, LTE_TDD },
	{  48, F(35500), 55240, 56739, F(35500),  55240,  56739, LTE_TDD },
	{  49, F(35500), 56740, 58239, F(35500),  56740,  58239, LTE_TDD },
	{  50, F(14320), 58240, 59089, F(14320),  58240,  59089, LTE_TDD },
	{  51, F(14270), 59090, 59139, F(14270),  59090,  59139, LTE_TDD },
	{  52, F(33000), 59140, 60139, F(33000),  59140,  60139, LTE_TDD },
	{  53, F(24835), 60140, 60254, F(24835),  60140,  60254, LTE_TDD },
	{  54, F(16700), 60255, 60304, F(16700),  60255,  60304, LTE_TDD },
	{  65, F(21100), 65536, 66435, F(19200), 131072, 131971, LTE_FDD },
	{  66, F(21100), 66436, 67335, F(17100), 131972, 132671, LTE_FDD },
	{  67, F( 7380), 67336, 67535,        0,      0,     -1, LTE_SDL },
	{  68, F( 7530), 67536, 67835, F( 6980), 132672, 132971, LTE_FDD },
	{  69, F(25700), 67836, 68335,        0,      0,     -1, LTE_SDL },
	{  70, F(19950), 68336, 68585, F(16950), 132972, 133121, LTE_FDD },
	{  71, F( 6170), 68586, 68935, F( 6630), 133122, 133471, LTE_FDD },
	{  72, F( 4610), 68936, 68985, F( 4510), 133472, 133521, LTE_FDD },
	{  73, F( 4600), 68986, 69035, F( 4500), 133522, 133571, LTE_FDD },
	{  74, F(14750), 69036, 69465, F(14270), 133572, 134001, LTE_FDD },
	{  75, F(14320), 69466, 70315,        0,      0,     -1, LTE_SDL },
	{  76, F(14270), 70316, 70365,        0,      0,     -1, LTE_SDL },
	{  85, F( 7280), 70366, 70545, F( 6980), 134002, 134181, LTE_FDD },
	{  87, F( 4200), 70546, 70595, F( 4100), 134182, 134231, LTE_FDD },
	{  88, F( 4220), 70596, 70645, F( 4120), 134232, 134281, LTE_FDD },
	{ 103, F( 7570), 70646, 70655, F( 7870), 134282, 134291, LTE_FDD },
	{ 106, F( 9350), 70656, 70705, F( 8960), 134292, 134341, LTE_FDD },
	{   0, 0, 0, 0, 0, 0, 0, LTE_FDD },
};

const lte_band_t *lte_band_get(int band)
{
	const lte_band_t *b;

	for (b = lte_bands; b->band; b++) {
		if (b->band == band)
			return b;
	}
	return NULL;
}

int lte_band_dl_range(int band, int *first, int *last, double *f_first, double *f_last)
{
	const lte_band_t *b = lte_band_get(band);

	if (!b)
		return -1;
	if (first)
		*first = b->n_offs_dl;
	if (last)
		*last = b->n_dl_last;
	if (f_first)
		*f_first = b->dl_low_100k * 1e5;
	if (f_last)
		*f_last = (b->dl_low_100k + (b->n_dl_last - b->n_offs_dl)) * 1e5;
	return 0;
}

double lte_earfcn_to_freq(int earfcn, int *band)
{
	const lte_band_t *b;

	for (b = lte_bands; b->band; b++) {
		if (earfcn >= b->n_offs_dl && earfcn <= b->n_dl_last) {
			if (band)
				*band = b->band;
			return (b->dl_low_100k + (earfcn - b->n_offs_dl)) * 1e5;
		}
	}
	if (band)
		*band = 0;
	return 0.0;
}

int lte_freq_to_earfcn(int band, double freq_hz)
{
	const lte_band_t *b = lte_band_get(band);
	double n;
	int earfcn;

	if (!b)
		return -1;
	n = freq_hz / 1e5 - b->dl_low_100k;
	earfcn = (int)floor(n + 0.5);
	if (fabs(n - earfcn) > 1e-3)
		return -1;
	earfcn += b->n_offs_dl;
	if (earfcn < b->n_offs_dl || earfcn > b->n_dl_last)
		return -1;
	return earfcn;
}

const char *lte_duplex_name(enum lte_duplex d)
{
	switch (d) {
	case LTE_TDD:
		return "TDD";
	case LTE_SDL:
		return "SDL";
	default:
		return "FDD";
	}
}
