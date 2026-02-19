/* Fast math utilities — NCO lookup table for sin/cos
 *
 * Extracted from libfm for reuse across the project.
 * Phase convention: 0..65536 maps to 0..2π
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef FASTMATH_H
#define FASTMATH_H

/* Initialize fast math lookup tables.
 * Returns 0 on success, <0 on error.
 * Safe to call multiple times (ref-counted). */
int fastmath_init(void);

/* Release fast math resources.
 * Must be called once per fastmath_init(). */
void fastmath_exit(void);

/* Check if fast math is available */
int fastmath_enabled(void);

/* Fast sin/cos using 65536-entry lookup table.
 * Phase is in range 0..65536 (maps to 0..2π).
 * Wraps automatically via uint16_t truncation. */
void fastmath_sincos(double phase, double *sin_out, double *cos_out);

/* Convert radians to table phase units */
static inline double fastmath_rad_to_phase(double radians)
{
	return radians * (65536.0 / (2.0 * 3.14159265358979323846));
}

/* Convert Hz offset + sample rate to phase step (in table units per sample) */
static inline double fastmath_hz_to_step(double freq_hz, double samplerate)
{
	return 65536.0 * freq_hz / samplerate;
}

#endif /* FASTMATH_H */
