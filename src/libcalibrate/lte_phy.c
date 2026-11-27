/* LTE downlink physical layer pieces for cell identification
 *
 * Written from 3GPP TS 36.211 V15.9.0 (7.2, 6.10.1, 6.11) and TS 36.212
 * V15.9.0 (5.1.1, 5.1.3.1, 5.1.4.2, 5.3.1). See lte_phy.h.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <string.h>
#include <math.h>
#include "lte_phy.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 36.211 7.2: c(n) = x1(n + 1600) + x2(n + 1600), registers hold x(n..n+30)
 * in bits 0..30 */
void lte_gold(uint32_t c_init, uint8_t *c, int len)
{
	uint32_t x1 = 1, x2 = c_init & 0x7fffffff, f1, f2;
	int n;

	for (n = 0; n < 1600 + len; n++) {
		if (n >= 1600)
			c[n - 1600] = (x1 ^ x2) & 1;
		f1 = ((x1 >> 3) ^ x1) & 1;
		f2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
		x1 = (x1 >> 1) | (f1 << 30);
		x2 = (x2 >> 1) | (f2 << 30);
	}
}

/* 36.211 6.11.1.1, root indices Table 6.11.1.1-1 */
void lte_pss_seq(int nid2, float complex *d)
{
	static const int root[3] = { 25, 29, 34 };
	int u = root[nid2], n;

	for (n = 0; n < 62; n++) {
		double a;
		if (n <= 30)
			a = -M_PI * u * n * (n + 1) / 63.0;
		else
			a = -M_PI * u * (n + 1) * (n + 2) / 63.0;
		d[n] = cos(a) + I * sin(a);
	}
}

/* 36.211 6.11.2.1 */
void lte_sss_m(int nid1, int *m0, int *m1)
{
	int qp = nid1 / 30;
	int q = (nid1 + qp * (qp + 1) / 2) / 30;
	int mp = nid1 + q * (q + 1) / 2;

	*m0 = mp % 31;
	*m1 = (*m0 + mp / 31 + 1) % 31;
}

/* m-sequence of length 31, x(0..4) = 0,0,0,0,1, x(i+5) = sum of taps */
static void mseq31(int t1, int t2, int t3, int t4, int *s)
{
	int x[31], i;

	memset(x, 0, sizeof(x));
	x[4] = 1;
	for (i = 0; i < 26; i++) {
		int v = x[i];
		if (t1)
			v ^= x[i + 1];
		if (t2)
			v ^= x[i + 2];
		if (t3)
			v ^= x[i + 3];
		if (t4)
			v ^= x[i + 4];
		x[i + 5] = v;
	}
	for (i = 0; i < 31; i++)
		s[i] = 1 - 2 * x[i];
}

void lte_sss_seq(int nid1, int nid2, int sf5, float *d)
{
	int st[31], ct[31], zt[31], m0, m1, n;

	mseq31(0, 1, 0, 0, st);	/* x(i+5) = x(i+2) + x(i) */
	mseq31(0, 0, 1, 0, ct);	/* x(i+5) = x(i+3) + x(i) */
	mseq31(1, 1, 0, 1, zt);	/* x(i+5) = x(i+4) + x(i+2) + x(i+1) + x(i) */
	lte_sss_m(nid1, &m0, &m1);
	for (n = 0; n < 31; n++) {
		int s0 = st[(n + m0) % 31];
		int s1 = st[(n + m1) % 31];
		int c0 = ct[(n + nid2) % 31];
		int c1 = ct[(n + nid2 + 3) % 31];
		int z0 = zt[(n + (m0 % 8)) % 31];
		int z1 = zt[(n + (m1 % 8)) % 31];
		if (!sf5) {
			d[2 * n] = s0 * c0;
			d[2 * n + 1] = s1 * c1 * z0;
		} else {
			d[2 * n] = s1 * c0;
			d[2 * n + 1] = s0 * c1 * z1;
		}
	}
}

/* 36.211 6.10.1.1 */
void lte_crs_seq(int pci, int ns, int l, int ext_cp, float complex *r)
{
	uint8_t c[2 * 116];
	uint32_t c_init;
	int m;

	c_init = 1024u * (uint32_t)(7 * (ns + 1) + l + 1) * (uint32_t)(2 * pci + 1) + 2 * pci + (ext_cp ? 0 : 1);
	lte_gold(c_init, c, 2 * 116);
	for (m = 0; m < 12; m++) {
		int mp = m + 104;
		r[m] = ((1 - 2 * c[2 * mp]) + I * (1 - 2 * c[2 * mp + 1])) * (float)M_SQRT1_2;
	}
}

/* 36.211 6.10.1.2 */
int lte_crs_offset(int pci, int port, int ns, int l)
{
	int v;

	switch (port) {
	case 0:
		v = (l == 0) ? 0 : 3;
		break;
	case 1:
		v = (l == 0) ? 3 : 0;
		break;
	case 2:
		v = 3 * (ns & 1);
		break;
	default:
		v = 3 + 3 * (ns & 1);
		break;
	}
	return (v + pci % 6) % 6;
}

/* 36.212 5.1.1, parity p0 is the MSB of the result */
uint16_t lte_crc16(const uint8_t *bits, int n)
{
	uint16_t reg = 0;
	int i;

	for (i = 0; i < n; i++) {
		int fb = ((reg >> 15) & 1) ^ (bits[i] & 1);
		reg <<= 1;
		if (fb)
			reg ^= 0x1021;
	}
	return reg;
}

/* Table 5.3.1.1-1, x_ant,0 is the MSB */
uint16_t lte_bch_mask(int ports)
{
	if (ports == 2)
		return 0xffff;
	if (ports == 4)
		return 0x5555;
	return 0x0000;
}

/* 36.212 5.1.3.1: G0 = 133, G1 = 171, G2 = 165 (octal); the MSB tap is the
 * input bit c_k, the LSB tap is c_(k-6) */
static const unsigned cc_gen[3] = { 0133, 0171, 0165 };

static inline int parity7(unsigned v)
{
	v &= 0x7f;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1;
}

/* rate matching: rate matched bit e -> coded bit (stream * 40 + k) */
static int16_t rm_tab[LTE_PBCH_BITS_N];
static int rm_ready;

/* Table 5.1.4-2 */
static const int cc_perm[32] = {
	1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31,
	0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30,
};

static void rm_init(void)
{
	int w2d[192], i, k, j, e;
	const int R = 2, ND = 2 * 32 - LTE_BCH_K;	/* rows, dummy bits */

	for (i = 0; i < 3; i++) {
		for (k = 0; k < 32 * R; k++) {
			int col = k / R, row = k % R;
			int y = cc_perm[col] + 32 * row;
			w2d[i * 64 + k] = (y < ND) ? -1 : i * LTE_BCH_K + (y - ND);
		}
	}
	for (e = 0, j = 0; e < LTE_PBCH_BITS_N; j++) {
		if (w2d[j % 192] >= 0)
			rm_tab[e++] = w2d[j % 192];
	}
	rm_ready = 1;
}

int lte_bch_coded_index(int e)
{
	if (!rm_ready)
		rm_init();
	return rm_tab[e];
}

void lte_bch_encode(const uint8_t *mib, int ports, uint8_t *e, int E)
{
	uint8_t c[LTE_BCH_K], d[LTE_BCH_CODED];
	uint16_t crc;
	unsigned s = 0;
	int i, k;

	memcpy(c, mib, LTE_BCH_A);
	crc = lte_crc16(mib, LTE_BCH_A) ^ lte_bch_mask(ports);
	for (i = 0; i < 16; i++)
		c[LTE_BCH_A + i] = (crc >> (15 - i)) & 1;
	for (i = 0; i < 6; i++)
		s |= (unsigned)c[LTE_BCH_K - 1 - i] << (5 - i);
	for (k = 0; k < LTE_BCH_K; k++) {
		unsigned r = ((unsigned)c[k] << 6) | s;
		for (i = 0; i < 3; i++)
			d[i * LTE_BCH_K + k] = parity7(r & cc_gen[i]);
		s = r >> 1;
	}
	for (i = 0; i < E; i++)
		e[i] = d[lte_bch_coded_index(i)];
}

/* circular Viterbi: three passes over the tail-biting block, the bits of
 * the middle pass are taken */
void lte_bch_viterbi(const float *soft, uint8_t *c)
{
	static uint8_t out[64][2][3];	/* [next state][predecessor bit][stream] */
	static int out_ready;
	float metric[64], next[64];
	uint8_t dec[3 * LTE_BCH_K][64];
	int t, ns, x, g, best;

	if (!out_ready) {
		for (ns = 0; ns < 64; ns++) {
			for (x = 0; x < 2; x++) {
				unsigned s = ((ns << 1) & 63) | x;
				unsigned r = ((unsigned)(ns >> 5) << 6) | s;
				for (g = 0; g < 3; g++)
					out[ns][x][g] = parity7(r & cc_gen[g]);
			}
		}
		out_ready = 1;
	}
	for (ns = 0; ns < 64; ns++)
		metric[ns] = 0;
	for (t = 0; t < 3 * LTE_BCH_K; t++) {
		int k = t % LTE_BCH_K;
		float s0 = soft[k], s1 = soft[LTE_BCH_K + k], s2 = soft[2 * LTE_BCH_K + k];
		float mx = -1e30;
		for (ns = 0; ns < 64; ns++) {
			float m[2];
			for (x = 0; x < 2; x++) {
				int ps = ((ns << 1) & 63) | x;
				const uint8_t *o = out[ns][x];
				m[x] = metric[ps] + (o[0] ? -s0 : s0) + (o[1] ? -s1 : s1) + (o[2] ? -s2 : s2);
			}
			if (m[1] > m[0]) {
				next[ns] = m[1];
				dec[t][ns] = 1;
			} else {
				next[ns] = m[0];
				dec[t][ns] = 0;
			}
			if (next[ns] > mx)
				mx = next[ns];
		}
		for (ns = 0; ns < 64; ns++)
			metric[ns] = next[ns] - mx;
	}
	best = 0;
	for (ns = 1; ns < 64; ns++) {
		if (metric[ns] > metric[best])
			best = ns;
	}
	ns = best;
	for (t = 3 * LTE_BCH_K - 1; t >= 0; t--) {
		if (t >= LTE_BCH_K && t < 2 * LTE_BCH_K)
			c[t - LTE_BCH_K] = ns >> 5;
		ns = ((ns << 1) & 63) | dec[t][ns];
	}
}

int lte_bch_check(const uint8_t *c, int ports)
{
	uint16_t crc = lte_crc16(c, LTE_BCH_A) ^ lte_bch_mask(ports), rx = 0;
	int i;

	for (i = 0; i < 16; i++)
		rx = (rx << 1) | (c[LTE_BCH_A + i] & 1);
	return crc == rx;
}

/* 36.331 MasterInformationBlock, UPER: dl-Bandwidth (3), phich-Duration (1),
 * phich-Resource (2), systemFrameNumber (8), spare (10); a0 is the first bit */
void lte_mib_parse(const uint8_t *a, lte_mib_t *mib)
{
	static const int bw[8] = { 6, 15, 25, 50, 75, 100, 0, 0 };
	int i, v;

	mib->n_rb = bw[(a[0] << 2) | (a[1] << 1) | a[2]];
	mib->phich_ext = a[3];
	mib->phich_res = (a[4] << 1) | a[5];
	for (v = 0, i = 6; i < 14; i++)
		v = (v << 1) | a[i];
	mib->sfn8 = v;
	for (v = 0, i = 14; i < 24; i++)
		v = (v << 1) | a[i];
	mib->spare = v;
}

const char *lte_phich_res_name(int res)
{
	static const char *n[4] = { "1/6", "1/2", "1", "2" };
	return n[res & 3];
}
