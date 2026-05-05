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
 * Time payload computation:
 *   - Derived from cycle/frame position (not wall clock).
 *   - Wall clock provides base hour and date only.
 *   - Frame 0 Cycle 0: exact top-of-hour (xx:00:00).
 *   - Other frames: minute and second from cycle*240 + frame*1.875.
 *
 * DATE/TIME scheduling:
 *   - Always eligible when queue is empty (fills idle frames).
 *   - Yields BIW capacity when messages are queued (preserves
 *     address space in block 0 for low_traffic optimization).
 *   - Re-eligible after 3 frames without time TX, even when busy
 *     (ensures all pager base frames see time updates regardless
 *     of collapse value).
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
 *   Pass 2 -- LRT rotation: fill remaining slots with the eligible type
 *             that has gone longest without transmission.
 *
 * DATE/TIME capacity management:
 *   Each BIW word consumes one slot in block 0, reducing address
 *   capacity.  When the message queue is non-empty, DATE/TIME yields
 *   its slots so addresses fit in block 0 (preserving low_traffic flag
 *   for pager battery savings).  After 3 frames without time TX, it
 *   becomes eligible again to ensure all pager base frames receive
 *   time updates regardless of system collapse value.
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
	 * Top-of-hour sync point -- pagers calibrate their real-time
	 * clock from this frame. Value is set to exactly xx:00:00. */
	if (frame == 0 && cycle == 0 && cfg->biw_time_enabled
	    && n < BIW_MAX_EXTRA && !selected[BIW_TIME]) {
		biw_out[n++] = BIW_TIME;
		selected[BIW_TIME] = 1;
		needs_time = 1;
	}

	/* ---- Pass 2: LRT rotation ----
	 *
	 * Time (010) is sent frequently — pagers need it to maintain
	 * their clock.  Date (001) is sent less often but always paired
	 * with Time (never alone).  This matches real network captures:
	 *   - Most frames: Time only (1 slot)
	 *   - Periodically: Date+Time pair (2 slots)
	 *
	 * LRT age is tracked independently for BIW_DATE and BIW_TIME.
	 * When BIW_DATE wins the LRT contest, it pulls BIW_TIME along
	 * (requires 2 free slots).  When BIW_TIME wins alone, it's
	 * emitted without Date (1 slot).
	 *
	 * Eligibility:
	 *   - Queue empty: always eligible.
	 *   - Queue has messages: not eligible (yield capacity to
	 *     addresses so they fit in block 0 / low_traffic).
	 *   - Queue has messages but last time TX was >=3 frames ago:
	 *     eligible again (covers all pager base frames at any
	 *     collapse value by rotating through frame positions). */
	int time_overdue = 0;
	if (cfg->biw_time_enabled && abs_frame > 0) {
		uint32_t last_time = carousel->last_tx_abs[BIW_TIME];
		if (last_time == 0 || (abs_frame - last_time) >= 3)
			time_overdue = 1;
	}

	while (n < BIW_MAX_EXTRA) {
		int best = -1;
		uint32_t best_age = 0;

		for (i = 0; i < BIW_TYPE_COUNT; i++) {
			if (selected[i])
				continue;
			if (!is_eligible(i, cfg, frame, cycle))
				continue;
			/* DATE needs 2 free slots (it always brings TIME) */
			if (i == BIW_DATE && (BIW_MAX_EXTRA - n) < 2)
				continue;
			/* Skip DATE/TIME when queue busy and not overdue */
			if ((i == BIW_DATE || i == BIW_TIME)
			    && cfg->queue_has_messages && !time_overdue)
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

		/* DATE always brings TIME along (never sent alone). */
		if (best == BIW_DATE) {
			biw_out[n++] = BIW_DATE;
			biw_out[n++] = BIW_TIME;
			selected[BIW_DATE] = 1;
			selected[BIW_TIME] = 1;
			needs_time = 1;
		} else {
			biw_out[n++] = best;
			selected[best] = 1;
			if (is_biw101(best))
				has_biw101 = 1;
			if (best == BIW_TIME || best == BIW_SYSINFO_TZ)
				needs_time = 1;
		}
	}

	/* ---- Compute time values from cycle/frame position ----
	 *
	 * FLEX timing structure:
	 *   total_seconds_in_hour = cycle * 240 + frame * 1.875
	 *   minute = floor(total_seconds / 60)
	 *   second = floor((total_seconds mod 60) / 7.5)   [0-7]
	 *
	 * Wall clock provides base hour and date.
	 * Frame 0 Cycle 0: forced to xx:00:00 (top-of-hour). */
	if (needs_time && time_out) {
		time_t now = time(NULL);
		struct tm tm_val;

		localtime_r(&now, &tm_val);

		/* Time from cycle/frame position.
		 * F0C0 naturally produces 00:00.0 (top-of-hour). */
		double total_sec = (double)cycle * 240.0 + (double)frame * 1.875;
		int minute = (int)(total_sec / 60.0);
		int sec_quant = (int)(((double)((int)total_sec % 60) +
				       (total_sec - (int)total_sec)) / 7.5);
		if (sec_quant > 7) sec_quant = 7;

		time_out->top_of_hour = (frame == 0 && cycle == 0) ? 1 : 0;
		time_out->hour   = (uint32_t)tm_val.tm_hour;
		time_out->minute = (uint32_t)minute;
		time_out->second = (uint32_t)sec_quant;

		/* Date from wall clock. */
		time_out->year  = (uint32_t)(tm_val.tm_year + 1900);
		time_out->month = (uint32_t)(tm_val.tm_mon + 1);
		time_out->day   = (uint32_t)tm_val.tm_mday;

		/* Timezone: caller sets from flex->timezone_code. */
		time_out->tz_code = -1;
		time_out->valid = 1;
	}

	*n_biw_out = n;

	/* Debug: log when DATE/TIME is selected */
	if (n > 0) {
		int has_date = 0, has_time = 0;
		for (i = 0; i < n; i++) {
			if (biw_out[i] == BIW_DATE) has_date = 1;
			if (biw_out[i] == BIW_TIME) has_time = 1;
		}
		if (has_date || has_time) {
			LOGP(DFLEX, LOGL_INFO,
			     "BIW carousel: C%u/F%u selected DATE=%d TIME=%d\n",
			     cycle, frame, has_date, has_time);
		}
	}

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
