/* LTE (E-UTRA) operating bands and EARFCN, 3GPP TS 36.101 Table 5.7.3-1
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef LTE_BANDS_H
#define LTE_BANDS_H

enum lte_duplex {
	LTE_FDD,
	LTE_TDD,
	LTE_SDL,	/* downlink only (carrier aggregation) */
};

typedef struct lte_band {
	int		band;
	int		dl_low_100k;	/* F_DL_low in units of 100 kHz */
	int		n_offs_dl;	/* N_Offs-DL */
	int		n_dl_last;	/* last downlink EARFCN */
	int		ul_low_100k;	/* F_UL_low, 0 if no uplink */
	int		n_offs_ul;
	int		n_ul_last;
	enum lte_duplex	duplex;
} lte_band_t;

/* all bands of the table, terminated by band 0 */
extern const lte_band_t lte_bands[];

/* band by number, NULL if none */
const lte_band_t *lte_band_get(int band);

/* downlink EARFCN range and the frequencies of its first and last channel
 * (Hz). Returns 0, or -1 for an unknown band. */
int lte_band_dl_range(int band, int *first, int *last, double *f_first, double *f_last);

/* downlink frequency (Hz) of an EARFCN, the band it belongs to (first
 * match) in *band if not NULL. Returns 0 for an unknown EARFCN. */
double lte_earfcn_to_freq(int earfcn, int *band);

/* downlink EARFCN of a frequency in a band, -1 if not on its raster */
int lte_freq_to_earfcn(int band, double freq_hz);

const char *lte_duplex_name(enum lte_duplex d);

#endif /* LTE_BANDS_H */
