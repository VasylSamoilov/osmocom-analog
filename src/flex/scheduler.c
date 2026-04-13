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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "scheduler.h"
#include "flex.h"
#include "frame.h"
#include "n_counter.h"
#include "biw_carousel.h"

/* Initialize scheduler state.
 * Called once at flex_create time. */
int flex_scheduler_init(struct flex *flex)
{
	int pol, ph;

	flex->sched_fallback_cycle = 0;
	flex->sched_fallback_frame = 0;
	flex->sched_ers_done = 0;
	flex->sched_last_cycle = 0;
	flex->sched_last_frame = 0;
	flex->last_tx_polarity = FLEX_POL_NORMAL;

	/* Initialize per-polarity TX state */
	for (pol = 0; pol < FLEX_TX_POLARITIES; pol++) {
		flex->tx_pol[pol].msg_list = NULL;
		flex->tx_pol[pol].msg_count = 0;
		flex->tx_pol[pol].frag_retrieval_seq = 0;
		flex->tx_pol[pol].n_inflight = 0;
		flex->tx_pol[pol].last_abs_frame = 0;
		memset(flex->tx_pol[pol].inflight, 0,
		       sizeof(flex->tx_pol[pol].inflight));
		memset(flex->tx_pol[pol].tx_temp, 0,
		       sizeof(flex->tx_pol[pol].tx_temp));

		/* N_Counter hash table */
		if (flex_n_counter_init(&flex->tx_pol[pol]) < 0)
			return -1;

		/* BIW carousel per phase */
		for (ph = 0; ph < FLEX_MAX_PHASES; ph++)
			flex_biw_carousel_init(&flex->tx_pol[pol].biw_carousel[ph]);
	}

	return 0;
}

/* Cleanup scheduler resources -- free N_Counter tables. */
void flex_scheduler_cleanup(struct flex *flex)
{
	int pol;
	for (pol = 0; pol < FLEX_TX_POLARITIES; pol++)
		flex_n_counter_cleanup(&flex->tx_pol[pol]);
}

/* Compute current cycle/frame from wall clock.
 *
 * FLEX time-division structure (ARIB STD-43A Section 3.2.1):
 *   - 15 cycles per hour, each 4 minutes (240 seconds)
 *   - 128 frames per cycle, each 1.875 seconds
 *
 * cycle = (minute_of_hour % 60) / 4, range 0-14
 * frame = floor(seconds_in_cycle / 1.875), range 0-127
 *
 * Returns 0 on success, -1 if clock unavailable (uses fallback). */
int flex_scheduler_get_time(struct flex *flex, flex_frame_time_t *ft)
{
	struct timespec ts;
	struct tm tm;
	int sec_in_cycle;
	double total_sec;

	if (clock_gettime(CLOCK_REALTIME, &ts) < 0) {
		/* Fallback: auto-increment from last known frame */
		LOGP(DFLEX, LOGL_ERROR, "clock_gettime failed, using fallback auto-increment\n");
		ft->cycle = flex->sched_fallback_cycle;
		ft->frame = flex->sched_fallback_frame;
		ft->frame_offset = 0.0;
		flex->sched_fallback_frame++;
		if (flex->sched_fallback_frame >= 128) {
			flex->sched_fallback_frame = 0;
			flex->sched_fallback_cycle = (flex->sched_fallback_cycle + 1) % 15;
		}
		return -1;
	}

	gmtime_r(&ts.tv_sec, &tm);

	/* cycle = (minute_of_hour % 60) / 4, range 0-14 */
	ft->cycle = (tm.tm_min % 60) / 4;

	/* seconds within the 4-minute cycle */
	sec_in_cycle = (tm.tm_min % 4) * 60 + tm.tm_sec;
	total_sec = sec_in_cycle + ts.tv_nsec / 1e9;

	/* frame = floor(total_sec / 1.875), range 0-127 */
	ft->frame = (uint32_t)(total_sec / 1.875);
	if (ft->frame > 127)
		ft->frame = 127;

	ft->frame_offset = (total_sec - ft->frame * 1.875) / 1.875;
	return 0;
}

/* Compute assigned frame and phase for a capcode.
 * Per ARIB STD-43A Appendix A Section 3:
 *   assigned_frame = (capcode / 16) mod 128
 *   assigned_phase = (capcode / 4) mod 4
 *
 * The phase assignment determines which data channel (A/B/C/D) a pager
 * listens on.  Pagers only decode their assigned phase to save battery.
 * Phase mapping for each mode:
 *   1600/2FSK (A1): 1 phase  — all capcodes on phase A
 *   3200/2FSK (A2): 2 phases — phase 0→A, phase 1→C  (mod 2)
 *   3200/4FSK (A3): 2 phases — phase 0→A, phase 1→C  (mod 2)
 *   6400/4FSK (A4): 4 phases — phase 0→A, 1→B, 2→C, 3→D
 *
 * Examples:
 *   capcode 1000: (1000/4) mod 4 = 250 mod 4 = 2 → phase C
 *   capcode 1001: (1001/4) mod 4 = 250 mod 4 = 2 → phase C
 *   capcode 1004: (1004/4) mod 4 = 251 mod 4 = 3 → phase D
 *   capcode 1008: (1008/4) mod 4 = 252 mod 4 = 0 → phase A */
void flex_scheduler_capcode_info(uint64_t capcode, flex_capcode_sched_t *info)
{
	info->assigned_frame = (uint32_t)((capcode / 16) % 128);
	info->assigned_phase = (uint32_t)((capcode / 4) % 4);
}

/* Find the next valid frame number for a capcode given collapse value m.
 *
 * With m=0, any frame is valid — return current_frame.
 * With m>0, find next frame where:
 *   frame_number mod 2^m == assigned_frame mod 2^m */
uint32_t flex_scheduler_next_frame(uint32_t current_frame,
				   uint32_t assigned_frame,
				   int collapse)
{
	uint32_t period, target, remainder;

	if (collapse <= 0)
		return current_frame;

	period = 1U << collapse;		/* 2^m */
	target = assigned_frame % period;
	remainder = current_frame % period;

	if (remainder <= target)
		return current_frame + (target - remainder);
	else
		return current_frame + (period - remainder + target);
}

/* Compute ERS cycle count for a given collapse value and baud rate.
 *
 * Minimum ERS duration = 2^m frame periods, each 1.875 sec.
 * Each ERS cycle = 96 bits (BS + Ar + BS_inv + Ar_inv = 4 × 24 bits).
 * Minimum 35 cycles (the current default, ~2.1 sec at 1600 baud). */
int flex_scheduler_ers_cycles(int collapse, int bitrate)
{
	double min_duration;
	int cycles;

	min_duration = (1 << collapse) * 1.875;
	cycles = (int)ceil(min_duration * bitrate / 96.0);

	if (cycles < FLEX_ERS_CYCLES)
		cycles = FLEX_ERS_CYCLES;

	return cycles;
}

/* Select baud rate for the next frame based on queued messages.
 *
 * Walks the message list and counts messages per (speed, modulation_type) group.
 * A3 messages (speed=3200, mod=4FSK) are counted separately from A2 (speed=3200, mod=2FSK).
 * Returns the speed with the most pending messages, and sets *modulation_type_out.
 * Defaults to 1600/2FSK if the queue is empty.
 * On a tie, prefers higher speed (more throughput). */
int flex_scheduler_select_speed(struct flex *flex, int *modulation_type_out)
{
	flex_msg_t *m;
	int c1600 = 0, c3200_2fsk = 0, c3200_4fsk = 0, c6400 = 0;

	for (m = flex->msg_list; m; m = m->next) {
		switch (m->speed) {
		case 3200:
			if (m->modulation_type == FLEX_MOD_4FSK)
				c3200_4fsk++;
			else
				c3200_2fsk++;
			break;
		case 6400: c6400++; break;
		default:   c1600++; break;
		}
	}

	LOGP(DFLEX, LOGL_DEBUG,
	     "Speed grouping: 1600=%d 3200/2fsk=%d 3200/4fsk=%d 6400=%d msgs\n",
	     c1600, c3200_2fsk, c3200_4fsk, c6400);

	if (c6400 > 0 && c6400 >= c3200_2fsk && c6400 >= c3200_4fsk && c6400 >= c1600) {
		*modulation_type_out = FLEX_MOD_4FSK;
		return 6400;
	}
	if (c3200_4fsk > 0 && c3200_4fsk >= c3200_2fsk && c3200_4fsk >= c1600) {
		*modulation_type_out = FLEX_MOD_4FSK;
		return 3200;
	}
	if (c3200_2fsk > 0 && c3200_2fsk >= c1600) {
		*modulation_type_out = FLEX_MOD_2FSK;
		return 3200;
	}
	*modulation_type_out = FLEX_MOD_2FSK;
	return 1600;
}

/* Parse --pocsag-mix frame slot specification.
 * Format: comma-separated frame numbers or ranges.
 * Examples: "0,1,2", "0-7", "0-7,64-71,120"
 * Sets pocsag_frame_slots[N]=1 for each designated frame and
 * pocsag_mix_enabled=1 if any slots were set.
 * Returns 0 on success, -1 on parse error. */
int flex_scheduler_parse_pocsag_slots(struct flex *flex, const char *spec)
{
	char buf[512];
	char *token, *saveptr, *dash;
	unsigned long start, end, i;
	int count = 0;

	if (!spec || !spec[0])
		return -1;

	memset(flex->pocsag_frame_slots, 0, sizeof(flex->pocsag_frame_slots));
	flex->pocsag_mix_enabled = 0;

	/* Work on a copy since strtok_r modifies the string */
	strncpy(buf, spec, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	for (token = strtok_r(buf, ",", &saveptr);
	     token;
	     token = strtok_r(NULL, ",", &saveptr)) {
		/* Skip leading whitespace */
		while (*token == ' ' || *token == '\t')
			token++;

		dash = strchr(token, '-');
		if (dash) {
			/* Range: "start-end" */
			*dash = '\0';
			start = strtoul(token, NULL, 10);
			end = strtoul(dash + 1, NULL, 10);
			if (start > 127 || end > 127 || start > end) {
				LOGP(DFLEX, LOGL_ERROR,
				     "POCSAG mix: invalid range %lu-%lu (must be 0-127).\n",
				     start, end);
				return -1;
			}
			for (i = start; i <= end; i++) {
				flex->pocsag_frame_slots[i] = 1;
				count++;
			}
		} else {
			/* Single frame number */
			start = strtoul(token, NULL, 10);
			if (start > 127) {
				LOGP(DFLEX, LOGL_ERROR,
				     "POCSAG mix: invalid frame %lu (must be 0-127).\n",
				     start);
				return -1;
			}
			flex->pocsag_frame_slots[start] = 1;
			count++;
		}
	}

	if (count > 0) {
		/* Validate: frames 0-3 must not be POCSAG slots (Req 8.7).
		 * These frames are reserved for SSID2 and mandatory BIW words. */
		for (i = 0; i <= 3; i++) {
			if (flex->pocsag_frame_slots[i]) {
				LOGP(DFLEX, LOGL_ERROR,
				     "POCSAG mix: frame %lu is reserved for FLEX (frames 0-3 protected).\n", i);
				memset(flex->pocsag_frame_slots, 0, sizeof(flex->pocsag_frame_slots));
				return -1;
			}
		}
		flex->pocsag_mix_enabled = 1;
		LOGP(DFLEX, LOGL_INFO,
		     "POCSAG mix: %d frame slot(s) designated for POCSAG.\n", count);
	}

	return 0;
}

/* Check if a frame number is designated as a POCSAG slot.
 * Returns 1 if POCSAG, 0 if FLEX. */
int flex_scheduler_is_pocsag_slot(struct flex *flex, uint32_t frame)
{
	if (!flex->pocsag_mix_enabled || frame > 127)
		return 0;
	return flex->pocsag_frame_slots[frame];
}

/* Compute repeat interval for multiple transmission (Spec Section 3.4.2).
 *
 * The repeat interval is the number of frames between successive
 * transmissions of the same subframe content.  It equals 2^m where
 * m is the effective collapse value:
 *   - td_collapse (5/6/7) if set (takes priority per spec)
 *   - system collapse otherwise
 *
 * Example: collapse=3, td_collapse=-1 → interval = 2^3 = 8 frames. */
uint32_t flex_scheduler_repeat_interval(int collapse, int td_collapse)
{
	int m = (td_collapse >= 0) ? td_collapse : collapse;
	if (m < 0) m = 0;
	if (m > 7) m = 7;
	return 1U << m;
}

/* Compute repeat unit for multiple transmission (Spec Section 3.4.2, Fig. 3.4.2-3).
 *
 * repeat_unit = num_transmissions × repeat_interval.
 * This is the total number of frames in one complete cycle of all
 * subframe transmissions. */
uint32_t flex_scheduler_repeat_unit(int num_transmissions, int collapse, int td_collapse)
{
	if (num_transmissions <= 1)
		return flex_scheduler_repeat_interval(collapse, td_collapse);
	return (uint32_t)num_transmissions *
	       flex_scheduler_repeat_interval(collapse, td_collapse);
}

/* Determine which subframe index (0..num_transmissions-1) to transmit
 * for the current frame number.
 *
 * Per Spec Section 3.4.2 / Fig. 3.4.2-3:
 * Within each repeat unit, subframes are transmitted in order at
 * repeat_interval spacing.  The subframe index is:
 *   (frame / repeat_interval) % num_transmissions
 *
 * Returns 0 if num_transmissions <= 1 (no subframing). */
int flex_scheduler_subframe_index(uint32_t frame, int num_transmissions,
				  int collapse, int td_collapse)
{
	uint32_t interval;

	if (num_transmissions <= 1)
		return 0;

	interval = flex_scheduler_repeat_interval(collapse, td_collapse);
	return (int)((frame / interval) % (uint32_t)num_transmissions);
}

/* Request a parameter change (collapse, num_transmissions, td_collapse).
 *
 * Per §3.4.2: "the base station waits until transmission of the multiple
 * transmission units for the paging information is completed, then changes
 * these parameters.  Once the parameters are changed, Frames without paging
 * information are transmitted for the duration of one repeat unit at the
 * value before the change."
 *
 * If num_transmissions <= 1 (no repeat), the change is applied immediately.
 * Otherwise, the scheduler enters a two-phase transition:
 *   1. DRAIN: finish the current repeat unit (send remaining copies)
 *   2. COOLDOWN: send idle frames for one repeat unit at old params
 *   3. Apply new params
 *
 * Returns 1 if the change was deferred (transition started),
 *         0 if applied immediately. */
int flex_scheduler_request_param_change(struct flex *flex,
					int new_collapse,
					int new_num_transmissions,
					int new_td_collapse,
					uint32_t current_abs_frame)
{
	/* No transition needed if not currently using multiple transmission */
	if (flex->num_transmissions <= 1) {
		flex->collapse = new_collapse;
		flex->num_transmissions = new_num_transmissions;
		flex->td_collapse = new_td_collapse;
		return 0;
	}

	/* No change — nothing to do */
	if (new_collapse == flex->collapse &&
	    new_num_transmissions == flex->num_transmissions &&
	    new_td_collapse == flex->td_collapse)
		return 0;

	/* Already in a transition — update pending values */
	if (flex->param_change_state > 0) {
		flex->pending_collapse = new_collapse;
		flex->pending_num_transmissions = new_num_transmissions;
		flex->pending_td_collapse = new_td_collapse;
		return 1;
	}

	/* Snapshot old params */
	flex->old_collapse = flex->collapse;
	flex->old_num_transmissions = flex->num_transmissions;
	flex->old_td_collapse = flex->td_collapse;
	flex->old_bitrate = flex->current_frame_speed;
	flex->old_mod_type = flex->current_frame_mod_type;

	/* Store pending new params */
	flex->pending_collapse = new_collapse;
	flex->pending_num_transmissions = new_num_transmissions;
	flex->pending_td_collapse = new_td_collapse;

	/* Compute drain deadline: end of current repeat unit.
	 * repeat_unit = num_transmissions × 2^collapse frames.
	 * We need to finish the current repeat unit boundary. */
	{
		uint32_t ru = flex_scheduler_repeat_unit(
			flex->num_transmissions, flex->collapse, flex->td_collapse);
		uint32_t next_boundary = ((current_abs_frame / ru) + 1) * ru;
		flex->param_change_deadline = next_boundary % 1920;
	}

	flex->param_change_state = 1; /* DRAIN */
	LOGP(DFLEX, LOGL_INFO,
	     "SCHED: Parameter change requested, draining until frame %u.\n",
	     flex->param_change_deadline);

	return 1;
}

/* Called each frame during network mode to advance the parameter change
 * state machine.  Returns the effective parameters to use for this frame.
 *
 * During DRAIN: use old params, allow messages.
 * During COOLDOWN: use old params, force idle (no messages).
 * After COOLDOWN: apply new params, return to normal.
 *
 * Sets *force_idle = 1 during cooldown to suppress message packing. */
void flex_scheduler_param_change_tick(struct flex *flex,
				      uint32_t current_abs_frame,
				      int *force_idle)
{
	*force_idle = 0;

	if (flex->param_change_state == 0)
		return;

	{
		int32_t diff = (int32_t)current_abs_frame - (int32_t)flex->param_change_deadline;
		int past_deadline = (diff >= 0) && (diff < 960);

		if (flex->param_change_state == 1 && past_deadline) {
			/* DRAIN complete → enter COOLDOWN.
			 * Send idle frames for one repeat unit at old params. */
			uint32_t ru = flex_scheduler_repeat_unit(
				flex->old_num_transmissions,
				flex->old_collapse, flex->old_td_collapse);
			flex->param_change_deadline = (current_abs_frame + ru) % 1920;
			flex->param_change_state = 2;
			LOGP(DFLEX, LOGL_INFO,
			     "SCHED: Drain complete, cooldown until frame %u.\n",
			     flex->param_change_deadline);
		}

		if (flex->param_change_state == 2) {
			*force_idle = 1;

			diff = (int32_t)current_abs_frame - (int32_t)flex->param_change_deadline;
			past_deadline = (diff >= 0) && (diff < 960);

			if (past_deadline) {
				/* COOLDOWN complete → apply new params */
				flex->collapse = flex->pending_collapse;
				flex->num_transmissions = flex->pending_num_transmissions;
				flex->td_collapse = flex->pending_td_collapse;
				flex->param_change_state = 0;
				LOGP(DFLEX, LOGL_INFO,
				     "SCHED: Cooldown complete, new params applied: collapse=%d num_tx=%d td_collapse=%d.\n",
				     flex->collapse, flex->num_transmissions, flex->td_collapse);
			}
		}
	}
}
