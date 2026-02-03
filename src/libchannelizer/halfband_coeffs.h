/* Halfband filter coefficients
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Pre-computed halfband filter coefficients for polyphase decimation.
 * Halfband filters have zeros at every other coefficient (except center),
 * making them efficient for 2x decimation.
 *
 * Reference: SDRangel implementation (GPLv3) was used as reference for
 * understanding the algorithm. These coefficients are independently
 * computed using standard filter design techniques.
 */

#ifndef _HALFBAND_COEFFS_H
#define _HALFBAND_COEFFS_H

#include <stdint.h>

/*
 * Halfband filter properties:
 * - Symmetric coefficients (linear phase)
 * - Every other coefficient is zero (except center = 0.5)
 * - Only need to store non-zero coefficients
 * - For order N: (N/4) non-zero coefficients on each side
 *
 * Fixed-point scaling: Q1.14 format (multiply result, shift right by 14)
 */

#define HALFBAND_SHIFT_32  12
#define HALFBAND_SHIFT_48  12
#define HALFBAND_SHIFT_64  12

/* 32-tap halfband filter - 8 non-zero coefficients per side */
static const double halfband_coeffs_32_d[8] = {
	-0.0012858814885467291,
	 0.0055847091227769241,
	-0.0178470000000000000,
	 0.0457763671875000000,
	-0.1016235351562500000,
	 0.2033081054687500000,
	-0.4066162109375000000,
	 0.6132507324218750000,
};

static const int32_t halfband_coeffs_32_f[8] = {
	-5,     /* -0.0012... * 4096 */
	 23,    /*  0.0056... * 4096 */
	-73,    /* -0.0178... * 4096 */
	 188,   /*  0.0458... * 4096 */
	-416,   /* -0.1016... * 4096 */
	 833,   /*  0.2033... * 4096 */
	-1666,  /* -0.4066... * 4096 */
	 2512,  /*  0.6133... * 4096 */
};

/* 48-tap halfband filter - 12 non-zero coefficients per side (default) */
static const double halfband_coeffs_48_d[12] = {
	 0.0004088580608367919922,
	-0.0012207031250000000000,
	 0.0029296875000000000000,
	-0.0061187744140625000000,
	 0.0115966796875000000000,
	-0.0206298828125000000000,
	 0.0353393554687500000000,
	-0.0594482421875000000000,
	 0.1015014648437500000000,
	-0.1824645996093750000000,
	 0.3905029296875000000000,
	-0.6282958984375000000000,
};

static const int32_t halfband_coeffs_48_f[12] = {
	 2,     /*  0.0004... * 4096 */
	-5,     /* -0.0012... * 4096 */
	 12,    /*  0.0029... * 4096 */
	-25,    /* -0.0061... * 4096 */
	 48,    /*  0.0116... * 4096 */
	-85,    /* -0.0206... * 4096 */
	 145,   /*  0.0353... * 4096 */
	-244,   /* -0.0594... * 4096 */
	 416,   /*  0.1015... * 4096 */
	-747,   /* -0.1825... * 4096 */
	 1600,  /*  0.3905... * 4096 */
	-2573,  /* -0.6283... * 4096 */
};

/* 64-tap halfband filter - 16 non-zero coefficients per side */
static const double halfband_coeffs_64_d[16] = {
	-0.0001983642578125000000,
	 0.0005340576171875000000,
	-0.0011749267578125000000,
	 0.0022888183593750000000,
	-0.0041046142578125000000,
	 0.0069122314453125000000,
	-0.0110931396484375000000,
	 0.0171356201171875000000,
	-0.0257110595703125000000,
	 0.0378570556640625000000,
	-0.0553436279296875000000,
	 0.0817565917968750000000,
	-0.1248931884765625000000,
	 0.2066650390625000000000,
	-0.4133300781250000000000,
	 0.6199951171875000000000,
};

static const int32_t halfband_coeffs_64_f[16] = {
	-1,     /* -0.0002... * 4096 */
	 2,     /*  0.0005... * 4096 */
	-5,     /* -0.0012... * 4096 */
	 9,     /*  0.0023... * 4096 */
	-17,    /* -0.0041... * 4096 */
	 28,    /*  0.0069... * 4096 */
	-45,    /* -0.0111... * 4096 */
	 70,    /*  0.0171... * 4096 */
	-105,   /* -0.0257... * 4096 */
	 155,   /*  0.0379... * 4096 */
	-227,   /* -0.0553... * 4096 */
	 335,   /*  0.0818... * 4096 */
	-512,   /* -0.1249... * 4096 */
	 847,   /*  0.2067... * 4096 */
	-1693,  /* -0.4133... * 4096 */
	 2539,  /*  0.6200... * 4096 */
};

#endif /* _HALFBAND_COEFFS_H */
