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
 * Halfband filters: symmetric, every other coefficient zero, centre 0.5;
 * only the non-zero coefficients of one side are stored, coeffs[0]
 * outermost. Designed with scipy.signal.remez, passband edge 0.2, stopband
 * edge 0.3 of the higher rate. Rejection measured with the filters in
 * halfband.c (docs/CHANNELIZER.md).
 */

#ifndef _HALFBAND_COEFFS_H
#define _HALFBAND_COEFFS_H

/* order 32, 8 non-zero coefficients per side */
static const double halfband_coeffs_32[8] = {
	-0.0021073308325794464987,
	 0.0046487325756916124744,
	-0.0094253483284870952230,
	 0.0172067476367727817399,
	-0.0297810315775497574942,
	 0.0515223422707157402423,
	-0.0984181610957268748763,
	 0.3156779045528796401321,
};

/* order 48, 12 non-zero coefficients per side: 81 dB or more */
static const double halfband_coeffs_48[12] = {
	-0.0001977105251997833513,
	 0.0005764314330547719170,
	-0.0013517112003954103502,
	 0.0027287085752015815057,
	-0.0049876118611699516228,
	 0.0084988500499251825454,
	-0.0137878410812425702958,
	 0.0217125673840823096850,
	-0.0339794813324906450069,
	 0.0549444008859947749523,
	-0.1006571255760314448358,
	 0.3164572492646141044226,
};

/* order 64, 16 non-zero coefficients per side: 104 dB or more */
static const double halfband_coeffs_64[16] = {
	-0.0000198070212550087549,
	 0.0000723396304630611518,
	-0.0001954236268858796754,
	 0.0004425211776644344614,
	-0.0008908465302898264156,
	 0.0016450395316165058578,
	-0.0028415163239110959557,
	 0.0046550597055525031603,
	-0.0073123928860005169006,
	 0.0111233842122661127222,
	-0.0165555831623171757772,
	 0.0244216866968013750216,
	-0.0364053572625170351884,
	 0.0568675541052285626886,
	-0.1018932313888856466821,
	 0.3168836360601109958246,
};

#endif /* _HALFBAND_COEFFS_H */
