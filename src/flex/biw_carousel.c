/* FLEX BIW carousel -- dynamic BIW word selection per frame.
 *
 * Decides which BIW2/3/4 words go into each frame:
 *   - Mandatory placement: SSID1 every frame, SSID2 frames 0-3,
 *     Time BIW in Frame 0 Cycle 0 (top-of-hour sync).
 *   - Carousel rotation: remaining slots filled by least-recently-
 *     transmitted type, ensuring all configured types get airtime.
 *   - Constraint: at most one BIW type 101 per frame per phase.
 *   - Hard limit: 3 extra BIW words beyond BIW1 (4 total).
 *
 * Also computes time values for Date/Time/Timezone BIWs:
 *   - Frame 0 Cycle 0: exact top-of-hour (xx:00:00), hour rounded
 *     to nearest boundary so 13:59 does not become 13:00.
 *   - Other frames: current wall clock time.
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

/* Return 1 if type_id is configured and eligible for this frame/cycle.
 * SSID1: every frame when configured.
 * SSID2: frames 0-3 only.
 * Time/Date: when biw_time enabled.
 * Timezone: when timezone configured.
 * System message: when content present this frame.
 * Channel setup: frames 0-3 when enabled. */
static int is_eligible(int type_id, const flex_biw_config_t *cfg,
		       uint32_t frame, uint32_t cycle)
{
	(void)cycle;
	switch (type_id) {
	case BIW_SSID1:      return cfg->ssid1_configured;
	case BIW_SSID2:      return cfg->ssid2_configured && frame <= 3;
	case BIW_TIME:       return cfg->biw_time_enabled;
	case BIW_DATE:       return cfg->biw_time_enabled;
	case BIW_SYSINFO_TZ: return cfg->timezone_configured;
	case BIW_SYSINFO_MSG:return cfg->has_sysmsg;
	case BIW_CHAN_SETUP:  return cfg->chan_setup_enabled && frame <= 3;
	}
	return 0;
}

/* Select BIW words for one frame.
 *
 * Two-pass algorithm:
 *   Pass 1 -- mandatory placements (SSID1, SSID2, sysmsg, Time at F0C0).
 *   Pass 2 -- carousel rotation: fill remaining slots with the type
 *             that has gone longest without transmission (LRT-first).
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
	int has_biw101 = 0;   /* 1 once a BIW101 variant is selected */
	int needs_time = 0;   /* 1 if any time-related BIW was picked */
	int selected[BIW_TYPE_COUNT];
	int i;

	memset(selected, 0, sizeof(selected));
	if (time_out)
		memset(time_out, 0, sizeof(*time_out));

	(void)abs_frame; /* used by caller in _update(), not here */

	/* ---- Pass 1: mandatory placements ---- */

	/* SSID1 (type 000): every frame when configured */
	if (cfg->ssid1_configured && n < BIW_MAX_EXTRA) {
		biw_out[n++] = BIW_SSID1;
		selected[BIW_SSID1] = 1;
	}

	/* SSID2 (type 111): frames 0-3 when configured */
	if (cfg->ssid2_configured && frame <= 3 && n < BIW_MAX_EXTRA) {
		biw_out[n++] = BIW_SSID2;
		selected[BIW_SSID2] = 1;
	}

	/* System message BIW101 (A=0000-0011): highest-priority BIW101 */
	if (cfg->has_sysmsg && n < BIW_MAX_EXTRA) {
		biw_out[n++] = BIW_SYSINFO_MSG;
		selected[BIW_SYSINFO_MSG] = 1;
		has_biw101 = 1;
	}

	/* Frame 0 Cycle 0: Time BIW (type 010) is mandatory.
	 * This is the top-of-hour sync point -- pagers calibrate their
	 * real-time clock from this frame.  The time value is set to
	 * exactly xx:00:00 (see time computation below). */
	if (frame == 0 && cycle == 0 && cfg->biw_time_enabled
	    && n < BIW_MAX_EXTRA && !selected[BIW_TIME]) {
		biw_out[n++] = BIW_TIME;
		selected[BIW_TIME] = 1;
		needs_time = 1;
	}

	/* ---- Pass 2: carousel rotation (LRT-first) ----
	 *
	 * Fill remaining slots from eligible, non-selected types.
	 * Pick the type with the smallest last_tx_abs (oldest).
	 * Respect the one-BIW101-per-frame-per-phase constraint. */
	while (n < BIW_MAX_EXTRA) {
		int best = -1;
		uint32_t best_age = 0;

		for (i = 0; i < BIW_TYPE_COUNT; i++) {
			if (selected[i])
				continue;
			if (!is_eligible(i, cfg, frame, cycle))
				continue;
			if (is_biw101(i) && has_biw101)
				continue;
			/* last_tx_abs == 0 means never transmitted -- wins. */
			if (best < 0 || carousel->last_tx_abs[i] < best_age) {
				best = i;
				best_age = carousel->last_tx_abs[i];
			}
		}

		if (best < 0)
			break; /* no more eligible types */

		biw_out[n++] = best;
		selected[best] = 1;
		if (is_biw101(best))
			has_biw101 = 1;
		if (best == BIW_TIME || best == BIW_DATE || best == BIW_SYSINFO_TZ)
			needs_time = 1;
	}

	/* Log any eligible types that were deferred due to slot limit */
	for (i = 0; i < BIW_TYPE_COUNT; i++) {
		if (!selected[i] && is_eligible(i, cfg, frame, cycle)) {
			LOGP(DFLEX, LOGL_DEBUG,
			     "BIW carousel: deferred type %d in F%u/C%u (slot limit).\n",
			     i, frame, cycle);
		}
	}

	/* ---- Compute time values for Date/Time/Timezone BIWs ----
	 *
	 * The carousel owns time computation so the encoder does not
	 * need to know about frame semantics.
	 *
	 * Frame 0 Cycle 0 (top of hour):
	 *   hour = nearest hour boundary (round at :30:00).
	 *   minute = 0, second = 0.
	 *   This avoids transmitting e.g. 13:00 when wall clock is 13:59
	 *   (which would round to 14:00 instead).
	 *
	 * All other frames:
	 *   Current wall clock, seconds quantized to 1/8-minute steps. */
	if (needs_time && time_out) {
		time_t now = time(NULL);
		struct tm tm_val;
		int is_top = (frame == 0 && cycle == 0);

		localtime_r(&now, &tm_val);

		if (is_top) {
			/* Top-of-hour: round to nearest hour boundary.
			 * Frame 0 Cycle 0 should be within ~2 min of the
			 * actual hour mark.  If past :58, use next hour;
			 * if before :02, use current hour.  Outside that
			 * window something is wrong with clock sync but
			 * we still pick the closest hour. */
			int hour = tm_val.tm_hour;
			if (tm_val.tm_min >= 58)
				hour = (hour + 1) % 24;
			time_out->top_of_hour = 1;
			time_out->hour   = (uint32_t)hour;
			time_out->minute = 0;
			time_out->second = 0;
		} else {
			/* Normal frame: current wall clock. */
			time_out->top_of_hour = 0;
			time_out->hour   = (uint32_t)tm_val.tm_hour;
			time_out->minute = (uint32_t)tm_val.tm_min;
			/* Seconds -> 1/8 minute steps (7.5s each, 0-7). */
			time_out->second = (uint32_t)(tm_val.tm_sec / 7.5);
			if (time_out->second > 7)
				time_out->second = 7;
		}

		/* Date: use wall clock date.  For top-of-hour the date
		 * might roll over (23:59 -> next day), but the hour
		 * rounding handles that -- the date stays as-is since
		 * the pager only uses the date field for display. */
		time_out->year  = (uint32_t)(tm_val.tm_year + 1900);
		time_out->month = (uint32_t)(tm_val.tm_mon + 1);
		time_out->day   = (uint32_t)tm_val.tm_mday;

		/* Timezone: caller sets from flex->timezone_code. */
		time_out->tz_code = -1;
		time_out->valid = 1;
	}

	*n_biw_out = n;
	return n;
}

/* Update carousel state after a frame has been encoded.
 * Records the absolute frame number for each transmitted BIW type
 * so the LRT rotation knows which type is oldest next time. */
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
