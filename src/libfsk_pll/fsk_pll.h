/* FSK Symbol Timing PLL Library
 *
 * PLL-based clock recovery for 2-level and 4-level FSK signals from
 * FM discriminator output. Ported from the FLEX demodulator's
 * flex_rx_build_symbol() (multimon-ng heritage).
 *
 * Features:
 *   - Proportional phase correction with dual rates (locked/unlocked)
 *   - Zero crossing detection on raw sample values (not quantized)
 *   - DC offset removal IIR
 *   - Majority voting over middle 80% of symbol period
 *   - Mid-symbol crossing detection for signal quality assessment
 *
 * (C) 2024 - osmocom-analog
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef _LIB_FSK_PLL_H
#define _LIB_FSK_PLL_H

#include <stdint.h>
#include "../libsample/sample.h"

/*
 * PLL tuning constants (per multimon-ng / FLEX, empirically validated).
 *
 * PHASE_LOCKED_RATE: Proportional correction applied to phase error when
 *   the PLL is locked (tracking a known signal). Smaller = more stable,
 *   less responsive to jitter.
 *
 * PHASE_UNLOCKED_RATE: Correction rate when acquiring (not yet locked).
 *   Larger = faster acquisition, but more susceptible to noise.
 *
 * DC_OFFSET_FILTER: Time constant for DC removal IIR, in seconds.
 *   The IIR is: dc = dc * (sr * tc) / (sr * tc + 1) + sample / (sr * tc + 1)
 *   At 0.010s and 31250 Hz: pole at 0.99997, -3dB at ~3 Hz.
 */
#define FSK_PLL_PHASE_LOCKED_RATE	0.045
#define FSK_PLL_PHASE_UNLOCKED_RATE	0.050
#define FSK_PLL_DC_OFFSET_FILTER	0.010

/* Maximum consecutive mid-symbol crossings before rejecting sync candidate */
#define FSK_PLL_NONCONSEC_MAX_SYNC	10

/* Maximum symbol periods with no zero crossings (timeout) */
#define FSK_PLL_TIMEOUT_MAX		100

/* FSK PLL state for one demodulator instance */
typedef struct fsk_pll {
	/* Configuration */
	int		samplerate;	/* audio sample rate */
	int		baudrate;	/* symbol rate */

	/* Phase accumulator (FLEX-style: phase_max = 100 * samplerate) */
	int64_t		phase;		/* current phase */
	int64_t		phase_max;	/* phase accumulator range */
	int64_t		phase_rate;	/* phase increment per sample */

	/* Zero crossing detection on raw samples */
	double		last_sample;	/* previous sample (after DC removal) */

	/* DC offset removal */
	double		dc_offset;	/* running DC estimate */
	int		dc_enabled;	/* 1 = DC removal active */

	/* Majority voting (2-level FSK: high vs low) */
	int		vote_high;	/* votes for bit 1 */
	int		vote_low;	/* votes for bit 0 */

	/* Signal quality tracking */
	int		nonconsec;	/* consecutive mid-symbol crossings */
	int		timeout;	/* symbol periods with no crossing */

	/* Signal level tracking */
	double		level_sum;	/* running sum of |sample| for envelope */
	int		level_count;	/* samples counted for envelope */

	/* Lock state */
	int		locked;		/* 1 = PLL is locked (use slower correction) */
} fsk_pll_t;

/*
 * Initialize an FSK PLL instance.
 *
 * samplerate: audio sample rate (Hz), e.g. 31250
 * baudrate: symbol rate (Hz), e.g. 1200
 */
void fsk_pll_init(fsk_pll_t *pll, int samplerate, int baudrate);

/*
 * Reset PLL state (phase, slicer, votes, DC offset).
 * Called when sync is lost and we need to re-acquire.
 */
void fsk_pll_reset(fsk_pll_t *pll);

/*
 * Process one audio sample through the PLL.
 *
 * sample: FM discriminator output (normalized, ±1.0 typical)
 * polarity: +1.0 or -1.0 (FSK polarity convention)
 * bit_out: if a symbol boundary is reached, the decided bit is stored here
 *
 * Returns 1 if a symbol boundary was reached (bit ready in *bit_out).
 * Returns 0 if no bit yet.
 */
int fsk_pll_process(fsk_pll_t *pll, sample_t sample, double polarity, uint8_t *bit_out);

/*
 * Copy PLL state from one instance to another.
 * Used when transferring from a scanner slot to the locked decoder.
 * Copies phase, last_sample, dc_offset, slicer state, and votes.
 */
void fsk_pll_copy_state(fsk_pll_t *dst, const fsk_pll_t *src);

/*
 * Get the current nonconsec count (for sync gating).
 */
static inline int fsk_pll_get_nonconsec(const fsk_pll_t *pll)
{
	return pll->nonconsec;
}

/*
 * Set the lock state (affects correction rate).
 * locked=1: use slower PHASE_LOCKED_RATE (tracking)
 * locked=0: use faster PHASE_UNLOCKED_RATE (acquiring)
 */
static inline void fsk_pll_set_locked(fsk_pll_t *pll, int locked)
{
	pll->locked = locked;
}

/*
 * Enable/disable DC offset removal.
 */
static inline void fsk_pll_set_dc_removal(fsk_pll_t *pll, int enabled)
{
	pll->dc_enabled = enabled;
}

/*
 * Get average signal level (mean of |sample| over recent samples).
 * Returns 0.0 if no samples counted yet.
 */
static inline double fsk_pll_get_level(const fsk_pll_t *pll)
{
	if (pll->level_count > 0)
		return pll->level_sum / (double)pll->level_count;
	return 0.0;
}

#endif /* _LIB_FSK_PLL_H */
