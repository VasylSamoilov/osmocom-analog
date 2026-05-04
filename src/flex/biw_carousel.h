/* FLEX BIW carousel — dynamic BIW word selection per frame.
 *
 * Selects which BIW2/3/4 words appear in each frame based on:
 *   - Mandatory placement rules (SSID1 every frame, SSID2 frames 0-3,
 *     Time in Frame 0 Cycle 0)
 *   - Least-recently-transmitted carousel rotation for remaining slots
 *   - At most one BIW101 per frame per phase
 *   - 4-word maximum (BIW1 + up to 3 extra)
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef FLEX_BIW_CAROUSEL_H
#define FLEX_BIW_CAROUSEL_H

#include <stdint.h>

/* Forward declarations */
struct flex_biw_carousel;

/* BIW carousel configuration — what's enabled on this system.
 * Passed to the selection function each frame. */
typedef struct flex_biw_config {
	int	ssid1_configured;	/* 1 = --ssid1 set (ssid or nid non-zero) */
	int	ssid2_configured;	/* 1 = --ssid2 set (country_code or tmf non-zero) */
	int	biw_time_enabled;	/* 1 = time/date BIW broadcast enabled */
	int	timezone_configured;	/* 1 = timezone_code >= 0 */
	int	chan_setup_enabled;	/* 1 = channel setup BIW emission enabled */
	int	roaming_active;		/* 1 = FIW n=1 */
	int	has_sysmsg;		/* 1 = system message BIW101 content present this frame */
	int	queue_has_messages;	/* 1 = message queue non-empty (time may yield capacity) */
} flex_biw_config_t;

/* Maximum additional BIW words beyond BIW1 */
#define BIW_MAX_EXTRA	3

/* BIW time value output — tells the encoder what time to use.
 * The carousel computes the correct time value based on frame/cycle:
 *   Frame 0 Cycle 0: top-of-hour (xx:00:00)
 *   Other frames: current wall clock time
 * This avoids the encoder needing to know about frame semantics. */
typedef struct flex_biw_time_val {
	int	valid;		/* 1 = time values are set */
	int	top_of_hour;	/* 1 = this is Frame 0 Cycle 0, use exact top-of-hour */
	uint32_t hour;		/* 0-23 */
	uint32_t minute;	/* 0-59 */
	uint32_t second;	/* 0-7 (1/8 minute steps, 7.5s each) */
	uint32_t year;		/* BIW year field (0-31, relative to 1994) */
	uint32_t month;		/* 1-12 */
	uint32_t day;		/* 1-31 */
	int	tz_code;	/* timezone zone code (0-31), or -1 if not configured */
} flex_biw_time_val_t;

/* Initialize carousel state (zero all last_tx_abs). */
void flex_biw_carousel_init(struct flex_biw_carousel *carousel);

/* Select BIW words for a frame.
 *
 * carousel:     per-phase carousel state (updated by _update after encoding)
 * cfg:          system BIW configuration
 * frame:        frame number (0-127)
 * cycle:        cycle number (0-14)
 * abs_frame:    absolute frame number (for LRU tracking)
 * biw_out:      output array of selected BIW type IDs (max BIW_MAX_EXTRA)
 * n_biw_out:    output: number of BIW words selected (0-3)
 * time_out:     output: time values for Date/Time/Timezone BIWs (if any selected)
 *
 * Returns the number of additional BIW words (0-3). BIW1 is always
 * present and not included in the output. */
int flex_biw_carousel_select(struct flex_biw_carousel *carousel,
			     const flex_biw_config_t *cfg,
			     uint32_t frame, uint32_t cycle,
			     uint32_t abs_frame,
			     int *biw_out, int *n_biw_out,
			     flex_biw_time_val_t *time_out);

/* Update carousel state after a frame is encoded.
 * Call this with the same biw_out/n_biw from _select. */
void flex_biw_carousel_update(struct flex_biw_carousel *carousel,
			      const int *biw_types, int n_biw,
			      uint32_t abs_frame);

#endif /* FLEX_BIW_CAROUSEL_H */
