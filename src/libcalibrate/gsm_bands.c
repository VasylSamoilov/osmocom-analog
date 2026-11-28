/* GSM bands and ARFCNs, 3GPP TS 45.005 clause 2
 *
 * Downlink (base transmit) carrier frequencies, Table 2-2 (fixed
 * designation); GSM 750 has no fixed designation (Table 2-1, dynamic
 * mapping), its ARFCNs here are the usual 438..511 (as libosmocore). In GSM
 * 750 the downlink is the lower block (747-763 MHz).
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
#include <strings.h>
#include "calibrate.h"

/* DL = f_ref + 200 kHz * (n - n_ref) for first <= n <= last */
static const gsm_band_def_t gsm_bands[] = {
	/* id, name, option, { first, last, f_ref, n_ref } x 2, dynamic */
	{ GSM_BAND_450,  "GSM-450",  "450",  { {  259,  293,  460.6e6,  259 }, { 0, -1, 0, 0 } }, 0 },
	{ GSM_BAND_480,  "GSM-480",  "480",  { {  306,  340,  489.0e6,  306 }, { 0, -1, 0, 0 } }, 0 },
	{ GSM_BAND_750,  "GSM-750",  "750",  { {  438,  511,  747.2e6,  438 }, { 0, -1, 0, 0 } }, 1 },
	{ GSM_BAND_850,  "GSM-850",  "850",  { {  128,  251,  869.2e6,  128 }, { 0, -1, 0, 0 } }, 0 },
	/* P-, E-, R-GSM 900 nest; the first in this order names a carrier */
	{ GSM_BAND_900,  "GSM-900",  "900",  { {    1,  124,  935.0e6,    0 }, { 0, -1, 0, 0 } }, 0 },
	{ GSM_BAND_E900, "E-GSM-900", "e900", { {    0,  124,  935.0e6,    0 }, {  975, 1023, 935.0e6, 1024 } }, 0 },
	{ GSM_BAND_R900, "R-GSM-900", "r900", { {    0,  124,  935.0e6,    0 }, {  955, 1023, 935.0e6, 1024 } }, 0 },
	{ GSM_BAND_1800, "DCS-1800", "1800", { {  512,  885, 1805.2e6,  512 }, { 0, -1, 0, 0 } }, 0 },
	{ GSM_BAND_1900, "PCS-1900", "1900", { {  512,  810, 1930.2e6,  512 }, { 0, -1, 0, 0 } }, 0 },
	{ 0, NULL, NULL, { { 0, -1, 0, 0 }, { 0, -1, 0, 0 } }, 0 },
};

const gsm_band_def_t *gsm_band_at(int i)
{
	if (i < 0 || i >= (int)(sizeof(gsm_bands) / sizeof(gsm_bands[0])) - 1)
		return NULL;
	return &gsm_bands[i];
}

const gsm_band_def_t *gsm_band_def(int id)
{
	for (int i = 0; gsm_bands[i].id; i++)
		if (gsm_bands[i].id == id)
			return &gsm_bands[i];
	return NULL;
}

int gsm_band_parse(const char *arg)
{
	for (int i = 0; gsm_bands[i].id; i++)
		if (!strcasecmp(arg, gsm_bands[i].arg))
			return gsm_bands[i].id;
	/* the spec's names */
	if (!strcasecmp(arg, "p900") || !strcasecmp(arg, "pgsm900") || !strcasecmp(arg, "gsm900"))
		return GSM_BAND_900;
	if (!strcasecmp(arg, "egsm900") || !strcasecmp(arg, "egsm"))
		return GSM_BAND_E900;
	if (!strcasecmp(arg, "rgsm900") || !strcasecmp(arg, "rgsm"))
		return GSM_BAND_R900;
	if (!strcasecmp(arg, "dcs1800") || !strcasecmp(arg, "dcs"))
		return GSM_BAND_1800;
	if (!strcasecmp(arg, "pcs1900") || !strcasecmp(arg, "pcs"))
		return GSM_BAND_1900;
	return -1;
}

/* Downlink frequency of an ARFCN in a band (Hz), 0 if not in it */
double arfcn_to_freq(int arfcn, int gsm_band)
{
	const gsm_band_def_t *b = gsm_band_def(gsm_band);

	if (!b)
		return 0.0;
	for (int r = 0; r < 2; r++)
		if (arfcn >= b->run[r].first && arfcn <= b->run[r].last)
			return b->run[r].f_ref + 200e3 * (arfcn - b->run[r].n_ref);
	return 0.0;
}

/* ARFCN of a downlink frequency in a band, -1 if none */
int gsm_freq_to_arfcn(int gsm_band, double freq_hz)
{
	const gsm_band_def_t *b = gsm_band_def(gsm_band);

	if (!b)
		return -1;
	for (int r = 0; r < 2; r++) {
		double k = (freq_hz - b->run[r].f_ref) / 200e3;
		int n = (int)(k + ((k < 0) ? -0.5 : 0.5)) + b->run[r].n_ref;

		if (n >= b->run[r].first && n <= b->run[r].last
		 && freq_hz - arfcn_to_freq(n, gsm_band) < 1.0 && freq_hz - arfcn_to_freq(n, gsm_band) > -1.0)
			return n;
	}
	return -1;
}

/* the lowest and highest downlink carrier of a band (Hz); 0, -1 unknown */
int gsm_band_dl_range(int gsm_band, double *f_lo, double *f_hi)
{
	const gsm_band_def_t *b = gsm_band_def(gsm_band);
	double lo = 0.0, hi = 0.0;

	if (!b)
		return -1;
	for (int r = 0; r < 2; r++) {
		if (b->run[r].last < b->run[r].first)
			continue;
		double a = arfcn_to_freq(b->run[r].first, gsm_band), z = arfcn_to_freq(b->run[r].last, gsm_band);
		if (lo == 0.0 || a < lo)
			lo = a;
		if (z > hi)
			hi = z;
	}
	*f_lo = lo;
	*f_hi = hi;
	return 0;
}

const char *gsm_band_name(int gsm_band)
{
	const gsm_band_def_t *b = gsm_band_def(gsm_band);

	return b ? b->name : "Unknown";
}
