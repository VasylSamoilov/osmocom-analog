/* Fast math utilities — NCO lookup table for sin/cos
 *
 * 65536-entry sine table with cosine as offset view.
 * Extracted from libfm for project-wide reuse.
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "fastmath.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float *sin_tab = NULL;
static float *cos_tab = NULL;
static int ref_count = 0;

int fastmath_init(void)
{
	int i;

	if (ref_count++ > 0)
		return 0;  /* Already initialized */

	/*
	 * NCO Lookup Table: 65536 entries (2^16)
	 *   Angular resolution: 0.0055° (0.000096 rad)
	 *   Max amplitude error: ~0.005%
	 *   Memory: (65536 + 16384) x 4 bytes = ~320 KB
	 *
	 * Cosine shares the same allocation, offset by 16384 (90°).
	 */
	sin_tab = calloc(65536 + 16384, sizeof(*sin_tab));
	if (!sin_tab) {
		fprintf(stderr, "fastmath: no memory for lookup table\n");
		ref_count--;
		return -1;
	}
	cos_tab = sin_tab + 16384;

	for (i = 0; i < 65536 + 16384; i++)
		sin_tab[i] = sin(2.0 * M_PI * (double)i / 65536.0);

	return 0;
}

void fastmath_exit(void)
{
	if (--ref_count > 0)
		return;

	if (sin_tab) {
		free(sin_tab);
		sin_tab = NULL;
		cos_tab = NULL;
	}
	ref_count = 0;
}

int fastmath_enabled(void)
{
	return sin_tab != NULL;
}

void fastmath_sincos(double phase, double *sin_out, double *cos_out)
{
	uint16_t idx = (uint16_t)phase;
	*sin_out = sin_tab[idx];
	*cos_out = cos_tab[idx];
}
