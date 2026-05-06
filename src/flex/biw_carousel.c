/* FLEX BIW carousel -- dynamic BIW word selection per frame.
 *
 * Decides which BIW2/3/4 words go into each frame:
 *   - Mandatory placement: SSID1 every frame, SSID2 frames 0-3,
 *     system message BIW101 when present.
 *   - Time-related group (DATE, TIME, SYSINFO_TZ): fills remaining
 *     slots via mod-3 round-robin across frames.
 *   - Constraint: at most one BIW type 101 per frame per phase.
 *   - Hard limit: 3 extra BIW words beyond BIW1 (4 total).
 *
 * Time-related BIW scheduling:
 *   DATE, TIME, and SYSINFO_TZ are treated as a group of 3.
 *   They are distributed across frames using abs_frame % 3 rotation:
 *     - 3 free slots: all 3 in every frame (instant coverage).
 *     - 2 free slots: 2 of 3 per frame, rotating which is left out.
 *     - 1 free slot:  1 of 3 per frame, cycling through all three.
 *   Since gcd(2^N, 3) = 1 for any collapse value N, a pager waking
 *   every 2^N frames is guaranteed to see all 3 types within at most
 *   3 wake cycles regardless of its base frame alignment.
 *
 * Time payload computation:
 *   Per FLEX standard §3.7.2, BIW TIME encodes the time at Frame 0
 *   of the current cycle (not the current frame).  The pager adds
 *   frame*1.875s internally.  Since each cycle = 4 minutes:
 *     minute = cycle * 4, second = 0 (always whole-minute boundary).
 *   Wall clock provides base hour and date.
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
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "flex.h"
#include "biw_carousel.h"

/* Zero all last-transmitted timestamps.  Call once at init. */
void flex_biw_carousel_init(flex_biw_carousel_t *carousel)
{
	memset(carousel, 0, sizeof(*carousel));
}

/* Return 1 if type_id is a BIW type 101 variant.
 * Only one BIW101 is allowed per frame per phase. */
static inline int is_biw101(int type_id)
{
	return type_id == BIW_SYSINFO_TZ ||
	       type_id == BIW_SYSINFO_MSG ||
	       type_id == BIW_CHAN_SETUP;
}

/* Time-related BIW types in rotation order. */
static const int time_group[] = { BIW_DATE, BIW_TIME, BIW_SYSINFO_TZ };
#define TIME_GROUP_COUNT 3

/* Select BIW words for one frame.
 *
 * Algorithm:
 *   1. Mandatory placements (SSID1, SSID2, system message).
 *   2. Fill remaining slots with time-related BIWs using mod-3
 *      round-robin keyed on abs_frame.  SYSINFO_TZ (a BIW101) is
 *      skipped if a system message BIW101 already occupies the frame.
 *
 * Outputs:
 *   biw_out[]  -- array of selected BIW type IDs (max BIW_MAX_EXTRA).
 *   *n_biw_out -- count of selected types (0..3).
 *   time_out   -- computed time/date values for the encoder (if any
 *                 time-related BIW was selected).  May be NULL.
 *
 * Returns the number of additional BIW words selected (same as *n_biw_out). */
int flex_biw_carousel_select(flex_biw_carousel_t *carousel,
			     const flex_biw_config_t *cfg,
			     uint32_t frame, uint32_t cycle,
			     uint32_t abs_frame,
			     int *biw_out, int *n_biw_out,
			     flex_biw_time_val_t *time_out)
{
	int n = 0;
	int has_biw101 = 0;
	int needs_time = 0;
	int i;

	(void)carousel; /* no LRT state needed for mod-3 rotation */

	if (time_out)
		memset(time_out, 0, sizeof(*time_out));

	/* ---- Pass 1: mandatory placements ---- */

	/* SSID1 (type 000): every frame when configured */
	if (cfg->ssid1_configured && n < BIW_MAX_EXTRA) {
		biw_out[n++] = BIW_SSID1;
	}

	/* SSID2 (type 111): frames 0-3 when configured */
	if (cfg->ssid2_configured && frame <= 3 && n < BIW_MAX_EXTRA) {
		biw_out[n++] = BIW_SSID2;
	}

	/* System message BIW101 (A=0000-0011): when content present */
	if (cfg->has_sysmsg && n < BIW_MAX_EXTRA) {
		biw_out[n++] = BIW_SYSINFO_MSG;
		has_biw101 = 1;
	}

	/* ---- Pass 2: time-related group (mod-3 rotation) ----
	 *
	 * Fill remaining slots from {DATE, TIME, SYSINFO_TZ}.
	 * Rotation offset = abs_frame % 3 determines priority order.
	 * With K free slots, we pick the first K eligible types from
	 * the rotated sequence.
	 *
	 * This guarantees that across 3 consecutive frames, all 3 types
	 * appear.  Since gcd(2^N, 3) = 1, any pager with collapse=N
	 * sees all 3 within 3 wake cycles. */
	if (cfg->biw_time_enabled && n < BIW_MAX_EXTRA) {
		int rot = (int)(abs_frame % TIME_GROUP_COUNT);

		for (i = 0; i < TIME_GROUP_COUNT && n < BIW_MAX_EXTRA; i++) {
			int idx = (rot + i) % TIME_GROUP_COUNT;
			int type_id = time_group[idx];

			/* SYSINFO_TZ is BIW101 — skip if one already placed */
			if (type_id == BIW_SYSINFO_TZ && has_biw101)
				continue;
			/* SYSINFO_TZ requires timezone to be configured */
			if (type_id == BIW_SYSINFO_TZ && !cfg->timezone_configured)
				continue;

			biw_out[n++] = type_id;
			if (is_biw101(type_id))
				has_biw101 = 1;
			needs_time = 1;
		}
	}

	/* ---- Compute time values ----
	 *
	 * Per FLEX standard §3.7.2: BIW TIME encodes the time at Frame 0
	 * of the current cycle.  The pager adds frame*1.875s internally.
	 * Since cycle*240s is always a whole number of minutes:
	 *   minute = cycle * 4, second = 0.
	 *
	 * Wall clock provides hour and date. */
	if (needs_time && time_out) {
		time_t now = time(NULL);
		struct tm tm_val;

		localtime_r(&now, &tm_val);

		time_out->top_of_hour = (frame == 0 && cycle == 0) ? 1 : 0;
		time_out->hour   = (uint32_t)tm_val.tm_hour;
		time_out->minute = (uint32_t)((int)cycle * 4);
		time_out->second = 0;

		time_out->year  = (uint32_t)(tm_val.tm_year + 1900);
		time_out->month = (uint32_t)(tm_val.tm_mon + 1);
		time_out->day   = (uint32_t)tm_val.tm_mday;

		time_out->tz_code = -1;
		time_out->valid = 1;
	}

	*n_biw_out = n;

	/* Debug log */
	if (n > 0) {
		int has_date = 0, has_time = 0, has_tz = 0;
		for (i = 0; i < n; i++) {
			if (biw_out[i] == BIW_DATE) has_date = 1;
			if (biw_out[i] == BIW_TIME) has_time = 1;
			if (biw_out[i] == BIW_SYSINFO_TZ) has_tz = 1;
		}
		if (has_date || has_time || has_tz) {
			LOGP(DFLEX, LOGL_INFO,
			     "BIW carousel: C%u/F%u selected DATE=%d TIME=%d TZ=%d\n",
			     cycle, frame, has_date, has_time, has_tz);
		}
	}

	return n;
}

/* Update carousel state after a frame has been encoded.
 * Records the absolute frame number for each transmitted BIW type. */
void flex_biw_carousel_update(flex_biw_carousel_t *carousel,
			      const int *biw_types, int n_biw,
			      uint32_t abs_frame)
{
	int i;
	for (i = 0; i < n_biw; i++) {
		int t = biw_types[i];
		if (t >= 0 && t < BIW_TYPE_COUNT)
			carousel->last_tx_abs[t] = abs_frame;
	}
}
