/* LTE downlink physical layer pieces for cell identification
 *
 * Sequences and the BCH channel code, written from 3GPP TS 36.211 and
 * TS 36.212 (no external library):
 *   - pseudo-random (Gold) sequence, 36.211 7.2
 *   - PSS Zadoff-Chu sequences, 36.211 6.11.1
 *   - SSS m-sequences, 36.211 6.11.2
 *   - cell-specific reference signals (CRS), 36.211 6.10.1
 *   - BCH: CRC-16 with the antenna-port mask (36.212 5.3.1.1), tail-biting
 *     convolutional code K=7 rate 1/3 (5.1.3.1), rate matching (5.1.4.2),
 *     and the MIB fields (36.331 MasterInformationBlock).
 *
 * The frequency-domain view used here is the centre 6 resource blocks:
 * k' = 0..71 (k' = 36 is the first subcarrier above DC). FFT bins of a
 * 128 point FFT at 1.92 MS/s are k' - 36 below DC and k' - 35 above.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef LTE_PHY_H
#define LTE_PHY_H

#include <stdint.h>
#include <complex.h>

#define LTE_NFFT		128	/* FFT size at 1.92 MS/s */
#define LTE_RATE		1.92e6	/* sample rate of the processing */
#define LTE_SC			72	/* subcarriers of the centre 6 RB */
#define LTE_PBCH_BITS_N		1920	/* M_bit, normal CP */
#define LTE_PBCH_BITS_E		1728	/* M_bit, extended CP */
#define LTE_BCH_A		24	/* MIB bits */
#define LTE_BCH_K		40	/* MIB + CRC */
#define LTE_BCH_CODED		120	/* 3 * K */

/* FFT bin (-64..63) of centre-6-RB subcarrier k' (0..71) */
static inline int lte_kp_bin(int kp) { return (kp < 36) ? kp - 36 : kp - 35; }

/* Gold sequence c(0..len-1), one bit per byte */
void lte_gold(uint32_t c_init, uint8_t *c, int len);

/* PSS d_u(n), n = 0..61, for N_ID^(2) = nid2 */
void lte_pss_seq(int nid2, float complex *d);

/* SSS m0, m1 from N_ID^(1) (Table 6.11.2.1-1) */
void lte_sss_m(int nid1, int *m0, int *m1);

/* SSS d(n), n = 0..61, values +-1; sf5 = 0 for subframes 0-4, 1 for 5-9 */
void lte_sss_seq(int nid1, int nid2, int sf5, float *d);

/* CRS r_{l,ns}(m') for the centre 6 RB: m = 0..11 (m' = m + 104) */
void lte_crs_seq(int pci, int ns, int l, int ext_cp, float complex *r);

/* CRS subcarrier offset (0..5) of antenna port p in symbol l of slot ns:
 * k' = 6m + offset. l is 0, N_symb-3 for ports 0/1, 1 for ports 2/3. */
int lte_crs_offset(int pci, int port, int ns, int l);

/* CRC-16 of n bits (gCRC16 = D^16 + D^12 + D^5 + 1, zero init) */
uint16_t lte_crc16(const uint8_t *bits, int n);

/* CRC mask by number of antenna ports (1, 2, 4), Table 5.3.1.1-1 */
uint16_t lte_bch_mask(int ports);

/* Encode 24 MIB bits for the given number of antenna ports into E rate
 * matched bits (E = 1920 or 1728), not scrambled. */
void lte_bch_encode(const uint8_t *mib, int ports, uint8_t *e, int E);

/* index into the 120 coded bits (stream * 40 + k) of rate matched bit e */
int lte_bch_coded_index(int e);

/* Tail-biting Viterbi on 120 soft values (index stream * 40 + k, positive
 * means bit 0). Returns the 40 decoded bits c_0..c_39. */
void lte_bch_viterbi(const float *soft, uint8_t *c);

/* The decoded block passes the CRC with the mask of 'ports' antenna ports */
int lte_bch_check(const uint8_t *c, int ports);

/* MIB fields, 36.331 */
typedef struct lte_mib {
	int	n_rb;		/* 6, 15, 25, 50, 75, 100; 0 if invalid */
	int	phich_ext;	/* PHICH duration extended */
	int	phich_res;	/* 0: 1/6, 1: 1/2, 2: 1, 3: 2 */
	int	sfn8;		/* SFN bits 9..2 */
	int	spare;		/* 10 bits, later releases use them */
} lte_mib_t;

void lte_mib_parse(const uint8_t *a, lte_mib_t *mib);
const char *lte_phich_res_name(int res);

#endif /* LTE_PHY_H */
