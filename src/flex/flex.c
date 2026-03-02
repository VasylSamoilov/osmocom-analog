/* FLEX pager transmitter processing
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
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

#define CHAN flex->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include <osmocom/cc/message.h>
#include "flex.h"
#include "frame.h"
#include "dsp.h"
#include "scheduler.h"

static const char *flex_state_name[] = {
	"IDLE",
	"ERS",
	"MESSAGE",
	"NET_ERS",
	"NET_FRAME",
};

static const char *flex_msg_type_names[] = {
	"auto",
	"tone",
	"numeric",
	"alpha",
	"hex",
	"instruction",
	"short",
};

const char *flex_msg_type_name(enum flex_msg_type type)
{
	if (type >= 0 && type < 7)
		return flex_msg_type_names[type];
	return "unknown";
}

static const char *print_capcode(flex_msg_t *msg)
{
	static char text[32];

	sprintf(text, "%" PRIu64, msg->capcode);

	return text;
}

static void flex_display_status(void)
{
	sender_t *sender;
	flex_t *flex;
	flex_msg_t *msg;

	display_status_start();
	for (sender = sender_head; sender; sender = sender->next) {
		flex = (flex_t *) sender;
		display_status_channel(flex->sender.kanal, NULL, flex_state_name[flex->state]);
		for (msg = flex->msg_list; msg; msg = msg->next)
			display_status_subscriber(print_capcode(msg), NULL);
	}
	display_status_end();
}

static void flex_new_state(flex_t *flex, enum flex_state new_state)
{
	if (flex->state == new_state)
		return;
	LOGP(DFLEX, LOGL_DEBUG, "State change: %s -> %s\n", flex_state_name[flex->state], flex_state_name[new_state]);
	flex->state = new_state;
	flex_display_status();
}

/*
 * Create msg instance.
 */
flex_msg_t *flex_msg_create(flex_t *flex, uint64_t capcode,
			    enum flex_msg_type msg_type,
			    const char *data, int data_length)
{
	flex_msg_t *msg, **msgp;

	LOGP(DFLEX, LOGL_INFO, "Creating msg instance to page capcode '%" PRIu64 "' / type '%s'.\n",
	     capcode, flex_msg_type_name(msg_type));

	/* create */
	msg = calloc(1, sizeof(*msg));
	if (!msg) {
		LOGP(DFLEX, LOGL_ERROR, "No mem!\n");
		abort();
	}
	if ((size_t)data_length > sizeof(msg->data)) {
		LOGP(DFLEX, LOGL_ERROR, "Text too long!\n");
		data_length = sizeof(msg->data);
	}

	/* init */
	msg->capcode = capcode;
	msg->msg_type = msg_type;
	if (data && data_length > 0)
		memcpy(msg->data, data, data_length);
	msg->data_length = data_length;

	/* per-message parameter defaults */
	msg->speed = 1600;
	msg->modulation_type = FLEX_MOD_2FSK;
	msg->polarity = -1.0;
	msg->priority = 0;
	msg->charset = 0;
	msg->is_group = 0;
	msg->is_temp_group = 0;
	msg->source_id[0] = '\0';
	msg->short_msg_index = -1;

	/* fragmentation state defaults */
	msg->fragment_index = 0;
	msg->total_fragments = 0;
	msg->retrieval_num = 0;

	/* link to tail of list */
	msg->flex = flex;
	msgp = &flex->msg_list;
	while ((*msgp))
		msgp = &(*msgp)->next;
	(*msgp) = msg;

	/* kick transmitter */
	if (flex->state == FLEX_STATE_IDLE) {
		if (flex->network_mode)
			flex_new_state(flex, flex->sched_ers_done
				       ? FLEX_STATE_NET_FRAME
				       : FLEX_STATE_NET_ERS);
		else
			flex_new_state(flex, FLEX_STATE_ERS);
	} else
		flex_display_status();

	return msg;
}

/* Destroy msg instance */
void flex_msg_destroy(flex_msg_t *msg)
{
	flex_msg_t **msgp;

	/* unlink */
	msgp = &msg->flex->msg_list;
	while ((*msgp) != msg)
		msgp = &(*msgp)->next;
	(*msgp) = msg->next;

	/* destroy */
	free(msg);

	/* update display */
	flex_display_status();
}

/* Number of idle frames to send after all messages before returning to IDLE */
#define FLEX_IDLE_BATCHES	2

/*
 * Check if a message needs fragmentation and split it if so.
 *
 * Walks the message queue and splits any message that exceeds the
 * single-frame capacity for its type. The original message is replaced
 * with fragment messages, each carrying the same retrieval number.
 *
 * Fragment flags (f0f1) are stored in the fragment_index/total_fragments
 * fields — the alpha encoder will use these when encoding each fragment.
 */
static void flex_fragment_queue(flex_t *flex)
{
	flex_msg_t *msg, *next;
	int max_chars, frag_count, i;
	char *fragments[64]; /* max 64 fragments */

	for (msg = flex->msg_list; msg; msg = next) {
		next = msg->next;

		/* Only fragment alpha, numeric, and hex messages */
		switch (msg->msg_type) {
		case FLEX_MSG_TYPE_ALPHA:
			max_chars = FLEX_MAX_CHARS_ALPHA;
			break;
		case FLEX_MSG_TYPE_NUMERIC:
			max_chars = FLEX_MAX_CHARS_NUMERIC;
			break;
		case FLEX_MSG_TYPE_HEX:
			max_chars = FLEX_MAX_CHARS_HEX;
			break;
		default:
			continue;
		}

		/* Already fragmented? Skip. */
		if (msg->total_fragments > 0)
			continue;

		/* Fits in one frame? Skip. */
		if (msg->data_length <= max_chars)
			continue;

		/* Split the message */
		frag_count = flex_fragment_message(msg->data, (int)msg->msg_type,
						   max_chars, fragments, 64);
		if (frag_count <= 1) {
			/* Fragmentation not needed or failed */
			if (frag_count == 1)
				free(fragments[0]);
			continue;
		}

		/* Assign a retrieval number for this set of fragments */
		uint32_t ret_num = flex->frag_retrieval_seq++;
		flex->frag_retrieval_seq &= 0x7F; /* 7-bit counter per spec */

		LOGP(DFLEX, LOGL_INFO,
		     "Fragmenting message for capcode %" PRIu64 " into %d fragments (retrieval=%u).\n",
		     msg->capcode, frag_count, ret_num);

		/* Create fragment messages and insert them in place of the original.
		 * We insert after the original, then destroy the original. */
		for (i = 0; i < frag_count; i++) {
			flex_msg_t *frag;
			int flen = (int)strlen(fragments[i]);

			frag = flex_msg_create(flex, msg->capcode, msg->msg_type,
					       fragments[i], flen);
			free(fragments[i]);

			if (!frag)
				continue;

			/* Copy per-message parameters from original */
			frag->speed = msg->speed;
			frag->modulation_type = msg->modulation_type;
			frag->polarity = msg->polarity;
			frag->priority = msg->priority;
			frag->charset = msg->charset;
			frag->is_group = msg->is_group;
			frag->is_temp_group = msg->is_temp_group;
			memcpy(frag->source_id, msg->source_id, sizeof(frag->source_id));
			frag->short_msg_index = msg->short_msg_index;

			/* Set fragmentation state */
			frag->fragment_index = i;
			frag->total_fragments = frag_count;
			frag->retrieval_num = ret_num;
		}

		/* Destroy the original (now replaced by fragments) */
		flex_msg_destroy(msg);
	}
}

/*
 * Set up the per-frame speed transition metadata.
 *
 * Per ARIB STD-43A Section 3.2, the S1 sync (BS1 + A + B + A_inv) and FIW
 * are ALWAYS transmitted at 1600 baud / 2-FSK, regardless of the frame's
 * data speed.  The speed transition to the frame's block speed occurs at
 * S2 (the C block), which immediately follows the FIW.
 *
 * For 1600 baud frames, no transition is needed (offset = 0).
 * For 3200/6400 baud frames, the DSP must start at 1600/2FSK and switch
 * to the target speed after consuming S1+FIW bytes.
 *
 * Layout: S1(14) + FIW(4) = 18 bytes at 1600 baud, then S2+DATA at target speed.
 * ERS is a separate event emitted before the frame, not part of it.
 */
static void flex_setup_speed_transition(flex_t *flex, int baud_rate,
					int modulation_type)
{
	if (baud_rate <= 1600) {
		/* No speed transition needed — entire frame at 1600/2FSK */
		flex->frame_speed_switch_offset = 0;
		flex->frame_target_speed = 0;
		flex->frame_target_mod_type = 0;
		return;
	}

	/* S1 = BS1(4) + A(4) + B(2) + A_inv(4) = 14 bytes
	 * FIW = 4 bytes
	 * Total sync portion at 1600 baud = 18 bytes */
	flex->frame_speed_switch_offset = 14 + 4;
	flex->frame_target_speed = baud_rate;
	flex->frame_target_mod_type = modulation_type;

	/* Start DSP at 1600/2FSK for the sync portion */
	dsp_set_speed(flex, 1600, FLEX_MOD_2FSK);

	LOGP_CHAN(DDSP, LOGL_DEBUG,
		  "Speed transition: 1600/2fsk for first 18 bytes, then %d/%s for data.\n",
		  baud_rate,
		  (modulation_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk");
}

/* Map (baud_rate, modulation_type) → phase count per ARIB STD-43A:
 *   A1 (1600, 2FSK) → 1 phase
 *   A2 (3200, 2FSK) → 1 phase
 *   A3 (3200, 4FSK) → 2 phases
 *   A4 (6400, 4FSK) → 4 phases */
static int flex_get_phase_count(int baud_rate, int modulation_type)
{
	if (baud_rate >= 6400 && modulation_type == FLEX_MOD_4FSK)
		return 4;  /* A4 */
	if (baud_rate >= 3200 && modulation_type == FLEX_MOD_4FSK)
		return 2;  /* A3 */
	return 1;  /* A1, A2 */
}

/*
 * Network mode frame generation.
 *
 * In network mode, the transmitter runs continuously:
 * - ERS re-sync burst is emitted first as a separate event (NET_ERS → NET_FRAME)
 * - Subsequent frames are sent continuously with real-time cycle/frame
 * - Idle frames (BIW + idle fill) are sent when no messages are queued
 * - Always returns 1 (frame always available)
 */
static int flex_get_next_frame_network(flex_t *flex)
{
	flex_frame_time_t ft;
	flex_frame_params_t params;
	flex_frame_msg_t frame_msg;
	flex_msg_t *msg;
	int msgs_packed = 0;
	int error = 0;
	size_t len;

	/* === ERS streaming phase ===
	 * ERS is a standalone re-sync burst emitted before data frames.
	 * It forces pagers to re-acquire synchronization. */
	if (!flex->sched_ers_done) {
		/* First call: compute total ERS cycles */
		if (flex->ers_sent_cycles == 0 && flex->ers_total_cycles == 0) {
			if (flex->ers_cycles_override > 0)
				flex->ers_total_cycles = flex->ers_cycles_override;
			else
				flex->ers_total_cycles = flex_scheduler_ers_cycles(
					flex->collapse, 1600);
			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "Network mode: starting ERS burst (%d cycles, %.1f sec at 1600 baud).\n",
				  flex->ers_total_cycles,
				  flex->ers_total_cycles * 96.0 / 1600.0);
		}

		/* Generate ERS chunk */
		if (flex->ers_sent_cycles < flex->ers_total_cycles) {
			int remaining = flex->ers_total_cycles - flex->ers_sent_cycles;
			int max_per_buf = (int)(sizeof(flex->frame_buffer) / 12);
			int chunk = remaining < max_per_buf ? remaining : max_per_buf;

			len = flex_generate_ers(flex->frame_buffer,
						sizeof(flex->frame_buffer),
						chunk);
			if (len == 0) {
				LOGP_CHAN(DFLEX, LOGL_ERROR,
					  "Network mode: failed to generate ERS chunk.\n");
				return 0;
			}

			flex->frame_buffer_length = (int)len;
			flex->frame_buffer_pos = 0;
			flex->frame_speed_switch_offset = 0; /* ERS is all 1600/2FSK */
			flex->ers_sent_cycles += chunk;

			LOGP_CHAN(DFLEX, LOGL_DEBUG,
				  "Network mode: ERS chunk %d/%d cycles.\n",
				  flex->ers_sent_cycles, flex->ers_total_cycles);
			return 1;
		}

		/* ERS complete — mark done and fall through to frame generation */
		flex->sched_ers_done = 1;
		LOGP_CHAN(DFLEX, LOGL_INFO,
			  "Network mode: ERS burst complete (%d cycles).\n",
			  flex->ers_total_cycles);

		/* Transition to NET_FRAME */
		if (flex->state == FLEX_STATE_NET_ERS)
			flex_new_state(flex, FLEX_STATE_NET_FRAME);
	}

	/* === Normal frame generation (ERS already sent) === */

	/* Fragment any oversized messages in the queue before selection */
	flex_fragment_queue(flex);

	/* Periodic queue stats: count pending messages by speed group */
	{
		flex_msg_t *qm;
		int q_total = 0, q1600 = 0, q3200_2 = 0, q3200_4 = 0, q6400 = 0;
		for (qm = flex->msg_list; qm; qm = qm->next) {
			q_total++;
			if (qm->speed == 6400) q6400++;
			else if (qm->speed == 3200 && qm->modulation_type == FLEX_MOD_4FSK) q3200_4++;
			else if (qm->speed == 3200) q3200_2++;
			else q1600++;
		}
		if (q_total > 0)
			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "Scheduler: queue=%d (1600=%d 3200/2fsk=%d 3200/4fsk=%d 6400=%d)\n",
				  q_total, q1600, q3200_2, q3200_4, q6400);
	}

	/* Get current wall-clock cycle/frame */
	flex_scheduler_get_time(flex, &ft);

	/* === POCSAG mixing: check if this frame slot is designated for POCSAG === */
	if (flex_scheduler_is_pocsag_slot(flex, ft.frame)) {
		/* Generate POCSAG idle batch and switch DSP to 1200 baud.
		 * POCSAG uses 512/1200/2400 baud 2-FSK. We use 1200 baud
		 * as the standard rate for mixed-mode operation. */
		len = flex_generate_pocsag_idle(flex->frame_buffer,
						sizeof(flex->frame_buffer));
		if (len == 0) {
			LOGP_CHAN(DFLEX, LOGL_ERROR,
				  "Network mode: failed to generate POCSAG idle batch.\n");
			return 0;
		}

		/* Switch DSP to POCSAG baud rate for this slot */
		dsp_set_speed(flex, 1200, FLEX_MOD_2FSK);

		flex->frame_buffer_length = (int)len;
		flex->frame_buffer_pos = 0;
		flex->frame_speed_switch_offset = 0; /* POCSAG is all 1200/2FSK */
		flex->sched_last_cycle = ft.cycle;
		flex->sched_last_frame = ft.frame;

		LOGP_CHAN(DFLEX, LOGL_DEBUG,
			  "Network mode: POCSAG idle slot C%u/F%u.\n",
			  ft.cycle, ft.frame);
		return 1;
	}

	/* Set up frame parameters — no ERS (already streamed) */
	flex_frame_params_default(&params);
	params.cycle = ft.cycle;
	params.frame = ft.frame;
	params.roaming = flex->roaming_active ? 1 : 0;
	params.collapse = flex->collapse;
	params.biw_time = flex->biw_time_enabled;
	params.baud_rate = flex_scheduler_select_speed(flex, &params.modulation_type);

	/* Don't switch DSP speed here — flex_setup_speed_transition() will
	 * set DSP to 1600/2FSK for the sync portion and schedule the
	 * transition to the target speed after S1+FIW. */

	/* Coverage/roaming fields */
	if (flex->ssid || flex->nid) {
		params.local_id = flex->ssid;
		params.coverage_id = flex->nid;
	}

	/* Find messages eligible for current frame (collapse-aware, speed-grouped).
	 * Only pick messages matching the selected baud rate for this frame.
	 * With collapse=0, all messages at the right speed are eligible.
	 * With collapse>0, also check capcode-to-frame mapping. */
	msg = NULL;
	{
		flex_msg_t *candidate;
		for (candidate = flex->msg_list; candidate; candidate = candidate->next) {
			/* Speed filter: skip messages not matching this frame's speed/modulation */
			if (candidate->speed != params.baud_rate ||
			    candidate->modulation_type != params.modulation_type)
				continue;

			if (flex->collapse <= 0) {
				msg = candidate;
				break;
			}

			/* Collapse filter: check capcode-to-frame mapping */
			{
				flex_capcode_sched_t sched;
				uint32_t next_frame;

				flex_scheduler_capcode_info(candidate->capcode, &sched);
				next_frame = flex_scheduler_next_frame(ft.frame,
								       sched.assigned_frame,
								       flex->collapse);
				if (next_frame == ft.frame) {
					msg = candidate;
					break;
				}
			}
		}
	}

	/* Multi-phase encoding for A3 (2 phases) and A4 (4 phases).
	 * A1 and A2 are single-phase and skip this block. */
	int num_phases = flex_get_phase_count(params.baud_rate, params.modulation_type);
	if (msg && num_phases > 1) {
		flex_phase_data_t phases[FLEX_MAX_PHASES];
		flex_frame_params_t phase_params;
		uint8_t phase_buf[FLEX_BUFFER_SIZE];
		int phase_has_msg[FLEX_MAX_PHASES];
		flex_msg_t *candidate, *next;
		int p, any_msg = 0;

		memset(phases, 0, sizeof(phases));
		memset(phase_has_msg, 0, sizeof(phase_has_msg));

		/* Build per-phase params — same as frame params */
		phase_params = params;

		/* Collect one message per phase from the queue */
		for (candidate = flex->msg_list; candidate; candidate = next) {
			flex_capcode_sched_t sched;
			int phase_idx;

			next = candidate->next;

			if (candidate->speed != params.baud_rate ||
			    candidate->modulation_type != params.modulation_type)
				continue;

			/* Collapse filter */
			if (flex->collapse > 0) {
				uint32_t nf;
				flex_scheduler_capcode_info(candidate->capcode, &sched);
				nf = flex_scheduler_next_frame(ft.frame,
							       sched.assigned_frame,
							       flex->collapse);
				if (nf != ft.frame)
					continue;
			} else {
				flex_scheduler_capcode_info(candidate->capcode, &sched);
			}

			phase_idx = (int)(sched.assigned_phase % (uint32_t)num_phases);

			/* Skip if this phase already has a message */
			if (phase_has_msg[phase_idx])
				continue;

			/* Encode this message into a temp buffer to get the 88 data words */
			memset(&frame_msg, 0, sizeof(frame_msg));
			frame_msg.capcode = candidate->capcode;
			frame_msg.msg_type = (int)candidate->msg_type;
			frame_msg.message = candidate->data;
			frame_msg.message_length = candidate->data_length;
			frame_msg.speed = candidate->speed;
			frame_msg.polarity = candidate->polarity;
			frame_msg.priority = candidate->priority;
			frame_msg.charset = candidate->charset;
			frame_msg.is_group = candidate->is_group;
			frame_msg.sequence_num = (int)(flex->msg_sequence++ & 0x7F);
			frame_msg.source_id = candidate->source_id[0] ? candidate->source_id : NULL;
			frame_msg.short_msg_idx = candidate->short_msg_index;

			len = flex_encode_frame_multi(&frame_msg, 1, &phase_params,
						      phase_buf, sizeof(phase_buf),
						      &msgs_packed, &error);

			if (error || len == 0)
				continue;

			/* Extract the 88 data words from the encoded output.
			 * Layout: S1(14) + FIW(4) + S2(c_bytes) + data
			 * S2 is 5 bytes × (baud_rate/1600) to fill 25 ms. */
			{
				int w;
				int s2_bytes = 5 * (params.baud_rate / 1600);
				uint8_t *dp = phase_buf + 14 + 4 + s2_bytes;
				for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
					phases[phase_idx].words[w] =
						((uint32_t)dp[0] << 24) |
						((uint32_t)dp[1] << 16) |
						((uint32_t)dp[2] << 8) |
						 (uint32_t)dp[3];
					dp += 4;
				}
				phases[phase_idx].word_count = FLEX_WORDS_PER_FRAME;
			}

			phase_has_msg[phase_idx] = 1;
			any_msg = 1;

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "Network mode: phase %d msg capcode=%" PRIu64 " type=%d speed=%d/%s polarity=%s priority=%d charset=%s group=%d seq=%d len=%d.\n",
				  phase_idx, frame_msg.capcode,
				  frame_msg.msg_type, candidate->speed,
				  (candidate->modulation_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
				  (candidate->polarity < 0) ? "neg" : "pos",
				  candidate->priority,
				  candidate->charset ? "kanji" : "ascii",
				  candidate->is_group,
				  frame_msg.sequence_num,
				  candidate->data_length);

			/* Destroy the transmitted message */
			flex_msg_destroy(candidate);
		}

		if (any_msg) {
			/* Fill empty phases with idle data */
			for (p = 0; p < num_phases; p++) {
				if (!phase_has_msg[p]) {
					/* Encode an idle frame for this phase */
					flex_frame_msg_t idle_msg;
					memset(&idle_msg, 0, sizeof(idle_msg));
					idle_msg.capcode = 1;
					idle_msg.msg_type = FLEX_FRAME_MSG_TYPE_TONE;
					idle_msg.message = "";
					idle_msg.message_length = 0;
					idle_msg.speed = params.baud_rate;
					idle_msg.polarity = -1.0;

					len = flex_encode_frame_multi(&idle_msg, 1, &phase_params,
								      phase_buf, sizeof(phase_buf),
								      &msgs_packed, &error);
					if (len > 0) {
						int w;
						int s2_bytes = 5 * (params.baud_rate / 1600);
						uint8_t *dp = phase_buf + 14 + 4 + s2_bytes;
						for (w = 0; w < FLEX_WORDS_PER_FRAME; w++) {
							phases[p].words[w] =
								((uint32_t)dp[0] << 24) |
								((uint32_t)dp[1] << 16) |
								((uint32_t)dp[2] << 8) |
								 (uint32_t)dp[3];
							dp += 4;
						}
						phases[p].word_count = FLEX_WORDS_PER_FRAME;
					}
				}
			}

			/* Encode the multi-phase frame */
			len = flex_encode_frame_phased(phases, num_phases, &params,
						       flex->frame_buffer,
						       sizeof(flex->frame_buffer),
						       &error);
			if (error || len == 0) {
				LOGP_CHAN(DFLEX, LOGL_NOTICE,
					  "Network mode: failed to encode phased frame (error=%d), sending idle.\n", error);
				goto send_idle;
			}

			flex->frame_buffer_length = (int)len;
			flex->frame_buffer_pos = 0;
			flex->sched_last_cycle = ft.cycle;
			flex->sched_last_frame = ft.frame;

			/* Set up speed transition for S1+FIW at 1600 → block speed */
			flex_setup_speed_transition(flex, params.baud_rate,
						    params.modulation_type);

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "Network mode: encoded %d-phase frame C%u/F%u speed=%d/%s polarity=%s collapse=%d roaming=%d.\n",
				  num_phases, ft.cycle, ft.frame, params.baud_rate,
				  (params.modulation_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
				  (flex->fsk_polarity < 0) ? "neg" : "pos",
				  params.collapse, params.roaming);
			return 1;
		}
		/* No eligible messages found — fall through to idle */
		goto send_idle;
	}

	/* Single-phase encoding (1600 bps) */
	if (msg) {
		/* Encode the eligible message */
		memset(&frame_msg, 0, sizeof(frame_msg));
		frame_msg.capcode = msg->capcode;
		frame_msg.msg_type = (int)msg->msg_type;
		frame_msg.message = msg->data;
		frame_msg.message_length = msg->data_length;
		frame_msg.speed = msg->speed;
		frame_msg.polarity = msg->polarity;
		frame_msg.priority = msg->priority;
		frame_msg.charset = msg->charset;
		frame_msg.is_group = msg->is_group;
		frame_msg.sequence_num = (int)(flex->msg_sequence++ & 0x7F);
		frame_msg.source_id = msg->source_id[0] ? msg->source_id : NULL;
		frame_msg.short_msg_idx = msg->short_msg_index;

		len = flex_encode_frame_multi(&frame_msg, 1, &params,
					      flex->frame_buffer,
					      sizeof(flex->frame_buffer),
					      &msgs_packed, &error);

		/* Destroy the transmitted message */
		flex_msg_destroy(msg);

		if (error || len == 0) {
			LOGP_CHAN(DFLEX, LOGL_NOTICE, "Network mode: failed to encode frame (error=%d), sending idle.\n", error);
			goto send_idle;
		}

		flex->frame_buffer_length = (int)len;
		flex->frame_buffer_pos = 0;
		flex->sched_last_cycle = ft.cycle;
		flex->sched_last_frame = ft.frame;

		/* Set up speed transition for S1+FIW at 1600 → block speed */
		flex_setup_speed_transition(flex, params.baud_rate,
					    params.modulation_type);

		LOGP_CHAN(DFLEX, LOGL_INFO,
			  "Network mode: encoded frame C%u/F%u capcode=%" PRIu64 " type=%d speed=%d/%s polarity=%s priority=%d charset=%s group=%d seq=%d len=%d.\n",
			  ft.cycle, ft.frame, frame_msg.capcode,
			  frame_msg.msg_type, params.baud_rate,
			  (params.modulation_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
			  (frame_msg.polarity < 0) ? "neg" : "pos",
			  frame_msg.priority,
			  frame_msg.charset ? "kanji" : "ascii",
			  frame_msg.is_group,
			  frame_msg.sequence_num,
			  frame_msg.message_length);

		return 1;
	}

send_idle:
	/* No messages — send idle frame.
	 * Encode a tone-only to capcode 1 (valid short addr) as a no-op.
	 * The pager won't respond since it's not addressed to it.
	 * This produces a valid frame with BIW + 1 addr + 1 vec + idle fill. */
	memset(&frame_msg, 0, sizeof(frame_msg));
	frame_msg.capcode = 1;
	frame_msg.msg_type = FLEX_FRAME_MSG_TYPE_TONE;
	frame_msg.message = "";
	frame_msg.message_length = 0;
	frame_msg.speed = 1600;
	frame_msg.polarity = -1.0;

	len = flex_encode_frame_multi(&frame_msg, 1, &params,
				      flex->frame_buffer,
				      sizeof(flex->frame_buffer),
				      &msgs_packed, &error);

	if (error || len == 0) {
		LOGP_CHAN(DFLEX, LOGL_ERROR, "Network mode: failed to encode idle frame (error=%d).\n", error);
		return 0;
	}

	flex->frame_buffer_length = (int)len;
	flex->frame_buffer_pos = 0;
	flex->sched_last_cycle = ft.cycle;
	flex->sched_last_frame = ft.frame;

	/* Idle frames use the current params (which may be at higher speed
	 * if the scheduler selected 3200/6400 for this slot). Set up
	 * speed transition so S1+FIW are still at 1600/2FSK. */
	flex_setup_speed_transition(flex, params.baud_rate,
				    params.modulation_type);

	LOGP_CHAN(DFLEX, LOGL_DEBUG, "Network mode: idle frame C%u/F%u.\n",
		  ft.cycle, ft.frame);

	return 1;
}

/*
 * Get next frame for transmission.
 *
 * Called by DSP layer when it needs a new frame to modulate.
 * Encodes the head message into frame_buffer, destroys it after encoding,
 * and manages state transitions.
 *
 * Returns 1 if a frame is ready in frame_buffer, 0 if no frame to send.
 */
int flex_get_next_frame(flex_t *flex)
{
	flex_msg_t *msg;
	int error = 0;
	size_t len;

	/* no frame if not transmitting */
	if (!flex->tx)
		return 0;

	/* Network mode: dispatch to continuous frame generator */
	if (flex->network_mode)
		return flex_get_next_frame_network(flex);

	switch (flex->state) {
	case FLEX_STATE_IDLE:
		return 0;

	case FLEX_STATE_ERS:
		/* Emit ERS (Emergency Re-Synchronization) burst.
		 * ERS is a network-level re-sync command emitted at network
		 * start/restart to force all pagers to re-acquire timing. */
		{
			int ers_cycles;

			if (flex->ers_cycles_override > 0)
				ers_cycles = flex->ers_cycles_override;
			else
				ers_cycles = FLEX_ERS_CYCLES;

			len = flex_generate_ers(flex->frame_buffer,
						FLEX_BUFFER_SIZE,
						ers_cycles);
			if (len == 0) {
				LOGP_CHAN(DFLEX, LOGL_ERROR,
					  "Failed to generate ERS burst.\n");
				flex_new_state(flex, FLEX_STATE_MESSAGE);
				return flex_get_next_frame(flex);
			}

			flex->frame_buffer_length = (int)len;
			flex->frame_buffer_pos = 0;
			flex->frame_speed_switch_offset = 0; /* ERS is all 1600/2FSK */
			dsp_set_speed(flex, 1600, FLEX_MOD_2FSK);

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "ERS burst: %d cycles (%d bytes, %.1f sec at 1600 baud).\n",
				  ers_cycles, (int)len,
				  (double)ers_cycles * 96.0 / 1600.0);

			flex_new_state(flex, FLEX_STATE_MESSAGE);
			flex->idle_count = 0;
			return 1;
		}

	case FLEX_STATE_MESSAGE:
		msg = flex->msg_list;
		if (msg) {
			flex_frame_msg_t frame_msg;
			flex_frame_params_t params;
			int msgs_packed = 0;

			/* reset idle counter when we have a message */
			flex->idle_count = 0;

			/* Don't switch DSP speed here — flex_setup_speed_transition()
			 * will handle starting at 1600/2FSK for sync and switching
			 * to the target speed after S1+FIW. */

			/* Switch DSP polarity if message requires it */
			dsp_set_polarity(flex, msg->polarity);

			/* Build frame message descriptor from queued message */
			memset(&frame_msg, 0, sizeof(frame_msg));
			frame_msg.capcode = msg->capcode;
			frame_msg.msg_type = (int)msg->msg_type;
			frame_msg.message = msg->data;
			frame_msg.message_length = msg->data_length;
			frame_msg.speed = msg->speed;
			frame_msg.polarity = msg->polarity;
			frame_msg.priority = msg->priority;
			frame_msg.charset = msg->charset;
			frame_msg.is_group = msg->is_group;
			frame_msg.sequence_num = (int)(flex->msg_sequence++ & 0x7F);

			/* Build frame params matching the message */
			flex_frame_params_default(&params);
			params.baud_rate = msg->speed;
			params.modulation_type = msg->modulation_type;

			/* Capture log values before destroy */
			{
				uint64_t log_capcode = msg->capcode;
				int log_msg_type = (int)msg->msg_type;
				int log_speed = msg->speed;
				int log_mod_type = msg->modulation_type;
				double log_polarity = msg->polarity;
				int log_data_length = msg->data_length;

				/* destroy the transmitted message */
				flex_msg_destroy(msg);
				msg = NULL;

				/* encode using the same path as network mode */
				len = flex_encode_frame_multi(&frame_msg, 1, &params,
							     flex->frame_buffer,
							     FLEX_BUFFER_SIZE,
							     &msgs_packed, &error);

				if (error || len == 0) {
					LOGP_CHAN(DFLEX, LOGL_NOTICE, "Failed to encode FLEX frame (error=%d), skipping.\n", error);
					if (flex->msg_list)
						return flex_get_next_frame(flex);
					goto check_idle;
				}

				flex->frame_buffer_length = (int)len;
				flex->frame_buffer_pos = 0;

				/* Set up speed transition for S1+FIW at 1600 → block speed */
				flex_setup_speed_transition(flex, params.baud_rate,
							    params.modulation_type);

				LOGP_CHAN(DFLEX, LOGL_INFO,
					  "Encoded FLEX frame (%d bytes): capcode=%" PRIu64 " type=%d speed=%d/%s polarity=%s deviation=%.0f len=%d.\n",
					  (int)len, log_capcode, log_msg_type,
					  log_speed,
					  (log_mod_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
					  (log_polarity < 0) ? "neg" : "pos",
					  flex->sender.speech_deviation,
					  log_data_length);
			}
			return 1;
		}

check_idle:
		/* no messages in queue — check scan/loopback for more work */
		if (flex_scan_or_loopback(flex)) {
			/* scan/loopback enqueued a new message, encode it */
			return flex_get_next_frame(flex);
		}

		/* no more messages and no scan/loopback — count idle batches */
		if (flex->idle_count++ >= FLEX_IDLE_BATCHES) {
			LOGP_CHAN(DFLEX, LOGL_INFO, "Transmission done.\n");
			LOGP_CHAN(DFLEX, LOGL_DEBUG, "Reached %d idle batches, turning transmitter off.\n", FLEX_IDLE_BATCHES);
			flex_new_state(flex, FLEX_STATE_IDLE);
			return 0;
		}

		LOGP_CHAN(DFLEX, LOGL_DEBUG, "Idle batch %d of %d.\n", flex->idle_count, FLEX_IDLE_BATCHES);
		return 0;

	case FLEX_STATE_NET_ERS:
	case FLEX_STATE_NET_FRAME:
		/* Network mode states are handled by flex_get_next_frame_network()
		 * above; reaching here means network_mode is off but state is wrong. */
		LOGP_CHAN(DFLEX, LOGL_ERROR, "Network state %s in one-shot mode, resetting to IDLE.\n",
			  flex_state_name[flex->state]);
		flex_new_state(flex, FLEX_STATE_IDLE);
		return 0;
	}

	return 0;
}

int flex_create(const char *kanal, double frequency, const char *device, int use_sdr, int samplerate, double rx_gain, double tx_gain, int tx, double deviation, double polarity, enum flex_msg_type msg_type, const char *message, uint64_t scan_from, uint64_t scan_to, const char *write_rx_wave, const char *write_tx_wave, const char *read_rx_wave, const char *read_tx_wave, int loopback)
{
	flex_t *flex;
	int rc;

	flex = calloc(1, sizeof(*flex));
	if (!flex) {
		LOGP(DFLEX, LOGL_ERROR, "No memory!\n");
		return -ENOMEM;
	}

	LOGP(DFLEX, LOGL_DEBUG, "Creating 'FLEX' instance for 'Kanal' = %s (sample rate %d).\n", kanal, samplerate);

	/* init general part of transceiver */
	rc = sender_create(&flex->sender, kanal, frequency, frequency, device, use_sdr, samplerate, rx_gain, tx_gain, 0, 0, write_rx_wave, write_tx_wave, read_rx_wave, read_tx_wave, loopback, PAGING_SIGNAL_NONE);
	if (rc < 0) {
		LOGP(DFLEX, LOGL_ERROR, "Failed to init transceiver process!\n");
		goto error;
	}

	/* init audio processing */
	rc = dsp_init_sender(flex, samplerate, deviation, polarity, 0);
	if (rc < 0) {
		LOGP(DFLEX, LOGL_ERROR, "Failed to init audio processing!\n");
		goto error;
	}

	flex->tx = tx;
	flex->default_msg_type = msg_type;
	flex->default_message = message;
	/* NOTE: fsk_deviation and fsk_polarity are set by dsp_init_sender() above.
	 * Do NOT overwrite fsk_deviation with the raw deviation (4800 Hz) here —
	 * it must stay at 1.0 (normalized) because the sender framework multiplies
	 * samples by speech_deviation to get actual Hz. */
	flex->scan_from = scan_from;
	flex->scan_to = scan_to;

	flex_display_status();

	LOGP(DFLEX, LOGL_NOTICE, "Created 'Kanal' %s: samplerate=%d deviation=%.0f polarity=%s tx=%d.\n",
	     kanal, samplerate, deviation, (polarity < 0) ? "neg" : "pos", tx);

	/* start scanning, if enabled, otherwise send loopback sequence, if enabled */
	flex_scan_or_loopback(flex);

	return 0;

error:
	flex_destroy(&flex->sender);

	return rc;
}

void flex_destroy(sender_t *sender)
{
	flex_t *flex = (flex_t *) sender;

	LOGP(DFLEX, LOGL_DEBUG, "Destroying 'FLEX' instance for 'Kanal' = %s.\n", sender->kanal);

	while (flex->msg_list)
		flex_msg_destroy(flex->msg_list);
	dsp_cleanup_sender(flex);
	sender_destroy(&flex->sender);
	free(flex);
}

/*
 * Scan or loopback: generate test messages for scanning or loopback mode.
 *
 * Scan mode: sequentially transmit to each capcode in the scan range.
 * Loopback mode: continuously generate test messages for self-testing.
 *
 * Returns 1 if a message was enqueued, 0 otherwise.
 */
int flex_scan_or_loopback(flex_t *flex)
{
	if (flex->scan_from < flex->scan_to) {
		char message[16];

		/* Generate scan message based on configured message type */
		switch (flex->default_msg_type) {
		case FLEX_MSG_TYPE_NUMERIC:
			sprintf(message, "%05d", (int)(flex->scan_from / 100));
			break;
		case FLEX_MSG_TYPE_ALPHA:
			sprintf(message, "%02x", (int)(flex->scan_from / 10000));
			break;
		case FLEX_MSG_TYPE_TONE:
		case FLEX_MSG_TYPE_AUTO:
		default:
			message[0] = '\0';
		}
		LOGP_CHAN(DFLEX, LOGL_NOTICE, "Transmitting %s message '%s' with capcode '%" PRIu64 "'.\n",
			  flex_msg_type_name(flex->default_msg_type), message, flex->scan_from);
		flex_msg_create(flex, flex->scan_from, flex->default_msg_type,
				message, strlen(message));
		flex->scan_from++;
		return 1;
	}

	if (flex->sender.loopback) {
		LOGP(DFLEX, LOGL_INFO, "Sending message for loopback test.\n");
		flex_msg_create(flex, 1234567, FLEX_MSG_TYPE_NUMERIC, "1234", 4);
		return 1;
	}

	return 0;
}

void call_down_clock(void)
{
}

/*
 * Validate a dialed capcode string from the console.
 *
 * FLEX capcodes can be 1-10 digits (short: 1-1933312, long: 2101249-4297068542).
 * Returns NULL if valid, or an error string if invalid.
 */
const char *flex_number_valid(const char *number)
{
	uint64_t capcode;
	int i;

	/* check all digits */
	for (i = 0; number[i]; i++) {
		if (number[i] < '0' || number[i] > '9')
			return "Illegal capcode digit (use 0..9 only)";
	}

	capcode = strtoull(number, NULL, 10);
	if (!flex_capcode_valid(capcode))
		return "Invalid FLEX capcode (short: 1-1933312, long: 2101249-4297068542)";

	return NULL;
}

int call_down_setup(int callref, const char *caller_id, enum number_type __attribute__((unused)) caller_type, const char *dialing)
{
	sender_t *sender;
	flex_t *flex;
	const char *message;
	flex_msg_t *msg;

	/* find transmitter */
	for (sender = sender_head; sender; sender = sender->next) {
		flex = (flex_t *) sender;
		if (flex->tx)
			break;
	}
	if (!sender) {
		LOGP(DFLEX, LOGL_NOTICE, "Cannot page, no transmitting station available, rejecting!\n");
		return -CAUSE_NOCHANNEL;
	}

	/* get message */
	if (caller_id[0])
		message = caller_id;
	else
		message = flex->default_message;

	/* parse capcode and create message */
	{
		uint64_t capcode = strtoull(dialing, NULL, 10);

		LOGP(DFLEX, LOGL_INFO, "Paging capcode '%" PRIu64 "' with %s message '%s'.\n",
		     capcode, flex_msg_type_name(flex->default_msg_type), message);

		msg = flex_msg_create(flex, capcode, flex->default_msg_type,
				      message, strlen(message));
		if (!msg)
			return -CAUSE_INVALNUMBER;
	}

	call_up_release(callref, CAUSE_NORMAL);

	return 0;
}

void call_down_answer(int __attribute__((unused)) callref, struct timeval __attribute__((unused)) *tv_meter)
{
}

void call_down_proceeding(int __attribute__((unused)) callref)
{
}

void call_down_disconnect(int callref, int cause)
{
	call_up_release(callref, cause);
}

void call_down_release(int __attribute__((unused)) callref, int __attribute__((unused)) cause)
{
}

void call_down_audio(void __attribute__((unused)) *decoder, void __attribute__((unused)) *decoder_priv, int __attribute__((unused)) callref, uint16_t __attribute__((unused)) sequence, uint8_t __attribute__((unused)) marker, uint32_t __attribute__((unused)) timestamp, uint32_t __attribute__((unused)) ssrc, uint8_t __attribute__((unused)) *payload, int __attribute__((unused)) payload_len)
{
}

void dump_info(void) {}
