/* FLEX protocol frame scheduler
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * Wall-clock timing, capcode-to-frame/phase mapping, and ERS duration
 * computation per ARIB STD-43A.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FLEX_SCHEDULER_H
#define FLEX_SCHEDULER_H

#include <stdint.h>
#include <time.h>

/* Forward declaration */
struct flex;

/* Computed frame timing from wall clock */
typedef struct flex_frame_time {
	uint32_t	cycle;		/* 0-14 */
	uint32_t	frame;		/* 0-127 */
	double		frame_offset;	/* fractional position within frame (0.0-1.0) */
} flex_frame_time_t;

/* Capcode scheduling info */
typedef struct flex_capcode_sched {
	uint32_t	assigned_frame;	/* (capcode / 16) mod 128 */
	uint32_t	assigned_phase;	/* (capcode / 4) mod 4 */
} flex_capcode_sched_t;

/* Initialize scheduler (called once at flex_create time) */
int flex_scheduler_init(struct flex *flex);

/* Cleanup scheduler resources */
void flex_scheduler_cleanup(struct flex *flex);

/* Compute current cycle/frame from wall clock.
 * Returns 0 on success, -1 if clock unavailable (sets fallback). */
int flex_scheduler_get_time(struct flex *flex, flex_frame_time_t *ft);

/* Compute assigned frame and phase for a capcode.
 * Per ARIB STD-43A Appendix A Section 3:
 *   frame = (capcode / 16) mod 128
 *   phase = (capcode / 4) mod 4 */
void flex_scheduler_capcode_info(uint64_t capcode, flex_capcode_sched_t *info);

/* Find the next valid frame number for a capcode given collapse value m.
 * With m=0, returns current_frame (any frame valid).
 * With m>0, returns next frame where frame mod 2^m == assigned_frame mod 2^m. */
uint32_t flex_scheduler_next_frame(uint32_t current_frame,
				   uint32_t assigned_frame,
				   int collapse);

/* Compute ERS cycle count for a given collapse value and bit rate (bps).
 * ERS duration must cover 2^m frame periods (each 1.875 sec).
 * Each ERS cycle = 96 bits. */
int flex_scheduler_ers_cycles(int collapse, int bitrate);

/* Select bit rate for the next frame based on queued messages.
 * Returns the speed (bps) that has the most pending messages,
 * defaulting to 1600 if the queue is empty.
 * Also sets *modulation_type_out to the modulation type of the winning group. */
int flex_scheduler_select_speed(struct flex *flex, int *modulation_type_out);

/* Parse --pocsag-mix frame slot specification into pocsag_frame_slots[].
 * Format: comma-separated frame numbers or ranges, e.g. "0,1,2" or "0-7,64-71".
 * Returns 0 on success, -1 on parse error. */
int flex_scheduler_parse_pocsag_slots(struct flex *flex, const char *spec);

/* Check if a frame number is designated as a POCSAG slot.
 * Returns 1 if POCSAG, 0 if FLEX. */
int flex_scheduler_is_pocsag_slot(struct flex *flex, uint32_t frame);

/* Compute repeat interval for multiple transmission (Spec Section 3.4.2).
 * repeat_interval = 2^m frames, where m = td_collapse if set, else system collapse. */
uint32_t flex_scheduler_repeat_interval(int collapse, int td_collapse);

/* Compute repeat unit for multiple transmission (Spec Section 3.4.2, Fig. 3.4.2-3).
 * repeat_unit = num_transmissions × repeat_interval frames. */
uint32_t flex_scheduler_repeat_unit(int num_transmissions, int collapse, int td_collapse);

/* Determine which subframe index (0..num_transmissions-1) to transmit
 * for the current frame number, given the repeat interval.
 * Returns the subframe index, or 0 if num_transmissions <= 1. */
int flex_scheduler_subframe_index(uint32_t frame, int num_transmissions,
				  int collapse, int td_collapse);

/* Request a parameter change (collapse, num_transmissions, td_collapse).
 * Defers the change until the current repeat unit completes, then sends
 * idle frames for one repeat unit at the old params before applying.
 * Returns 1 if deferred, 0 if applied immediately. */
int flex_scheduler_request_param_change(struct flex *flex,
					int new_collapse,
					int new_num_transmissions,
					int new_td_collapse,
					uint32_t current_abs_frame);

/* Advance the parameter change state machine.
 * Called each frame during network mode.
 * Sets *force_idle = 1 during cooldown to suppress message packing. */
void flex_scheduler_param_change_tick(struct flex *flex,
				      uint32_t current_abs_frame,
				      int *force_idle);

#endif /* FLEX_SCHEDULER_H */
