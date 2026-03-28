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
#include <unistd.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include "../libmobile/main_mobile.h"
#include <osmocom/cc/message.h>
#include "flex.h"
#include "frame.h"
#include "dsp.h"
#include "scheduler.h"

#define FLEX_MAX_QUEUE_DEPTH 256

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
	"secure",
	"special_num",
	"numbered_num",
	"numbered_special",
};

const char *flex_msg_type_name(enum flex_msg_type type)
{
	if (type >= 0 && type < 11)
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
 * Trigger an ERS (Emergency Re-Synchronization) burst.
 *
 * Called from FIFO command handler or at startup.  Sets up ERS parameters
 * and transitions the state machine to the ERS state.
 *
 * Polarity is irrelevant for ERS: the cycle structure is
 * BS + Ar + BS_inv + Ar_inv, so inverting all bits just shifts the
 * continuous stream by half a cycle.  Both Ar and Ar_inv appear in
 * every cycle, so pagers of either polarity detect the re-sync.
 *
 * Duration is calculated from the collapse value using
 * flex_scheduler_ers_cycles(), per Section 3.2.1:
 *   "The Re-synchronization pattern must be transmitted for a continuous
 *    period which is equivalent to the battery saving cycle for a pager
 *    having the maximum Collapse cycle value."
 */
void flex_trigger_ers(flex_t *flex)
{
	int ers_cycles;

	/* Calculate ERS duration from collapse value */
	if (flex->ers_cycles_override > 0)
		ers_cycles = flex->ers_cycles_override;
	else
		ers_cycles = flex_scheduler_ers_cycles(flex->collapse, 1600);

	/* Reset streaming counters (used by network mode) */
	flex->ers_total_cycles = ers_cycles;
	flex->ers_sent_cycles = 0;

	LOGP(DFLEX, LOGL_INFO,
	     "ERS triggered: %d cycles (%.1f sec at 1600 baud).\n",
	     ers_cycles, (double)ers_cycles * 96.0 / 1600.0);

	/* Transition to ERS state */
	if (flex->network_mode) {
		flex->sched_ers_done = 0;
		flex_new_state(flex, FLEX_STATE_NET_ERS);
	} else {
		flex_new_state(flex, FLEX_STATE_ERS);
	}
}

/*
 * Create msg instance.
 */
flex_msg_t *flex_msg_create(flex_t *flex, uint64_t capcode,
			    enum flex_msg_type msg_type,
			    const char *data, int data_length)
{
	flex_msg_t *msg, **msgp;

	LOGP(DFLEX, LOGL_INFO, "Creating msg instance to page capcode '%" PRIu64 "' (%s) / type '%s'.\n",
	     capcode, flex_capcode_type_name(capcode), flex_msg_type_name(msg_type));

	/* Warn if capcode falls in a special protocol address range.
	 * These are not user-assignable per Table 3.8.1-1 but we allow
	 * them through for testing and protocol experimentation. */
	if (flex_capcode_is_special(capcode)) {
		enum flex_addr_type stype = flex_capcode_special_type(capcode);
		uint32_t aw = (uint32_t)(capcode + FLEX_SHORT_ADDR_OFFSET);
		if (stype == FLEX_ADDR_OPER_MSG)
			LOGP(DFLEX, LOGL_NOTICE,
			     "Warning: capcode %" PRIu64 " is special address %s/%s (aw=0x%05X) — not a user-assignable capcode.\n",
			     capcode, flex_addr_type_name(stype),
			     flex_oper_msg_subtype_name(aw), aw);
		else
			LOGP(DFLEX, LOGL_NOTICE,
			     "Warning: capcode %" PRIu64 " is special address %s (aw=0x%05X) — %s.\n",
			     capcode, flex_addr_type_name(stype), aw,
			     flex_special_addr_detail(aw) ? flex_special_addr_detail(aw) : "not a user-assignable capcode");
	}

	/* Queue overflow protection: count all messages including
	 * retransmission-pending ones toward the depth limit. */
	{
		int depth = 0;
		flex_msg_t *qm;
		for (qm = flex->msg_list; qm; qm = qm->next)
			depth++;
		if (depth >= FLEX_MAX_QUEUE_DEPTH) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "Queue overflow: depth=%d, rejecting capcode=%" PRIu64 "\n",
			     depth, capcode);
			return NULL;
		}
	}

	/* create */
	msg = calloc(1, sizeof(*msg));
	if (!msg) {
		LOGP(DFLEX, LOGL_ERROR, "No mem!\n");
		abort();
	}

	/* Validate payload before enqueuing — reject messages that would
	 * cause encoding errors at frame time and block the entire queue.
	 * Note: secure_encoding is not yet known here (set by caller after
	 * create), so secure binary validation must happen at the call site. */
	{
		int verr = flex_msg_validate(msg_type, data, data_length, 0);
		if (verr) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "Rejecting capcode %" PRIu64 " type=%s: invalid payload (len=%d).\n",
			     capcode, flex_msg_type_name(msg_type), data_length);
			free(msg);
			return NULL;
		}
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

	/* per-message parameter defaults (use flex->default_polarity from CLI -P) */
	msg->speed = 1600;
	msg->modulation_type = FLEX_MOD_2FSK;
	msg->polarity = flex->default_polarity ? flex->default_polarity : FLEX_DEFAULT_POLARITY;
	msg->priority = 0;
	msg->charset = 0;
	msg->is_group = 0;
	msg->is_temp_group = 0;
	msg->temp_delivery_slot = -1;
	msg->source_id[0] = '\0';
	msg->short_msg_type = FLEX_SMSG_TYPE_NUMERIC;
	msg->short_msg_source = 0;
	msg->short_msg_number = 0;
	msg->short_msg_r = 0;
	msg->blocking_length = flex->default_blocking_length;
	msg->mail_drop = 0;
	msg->phase = -1;

	/* secure / numbered numeric defaults */
	msg->secure_subtype = 0;
	msg->secure_encoding = 0;
	msg->numbered_r = 1;	/* retransmission scheduler sets R=0
				 * on retransmissions with same N */
	/* S flag derived from message type:
	 *   NUMBERED_NUM     (nnumeric) → S=0: standard digit display
	 *   NUMBERED_SPECIAL (nspecial) → S=1: ID-ROM display (like special/V=100)
	 * Both use V=111 on the wire; S is the only difference. */
	msg->numbered_s = (msg_type == FLEX_MSG_TYPE_NUMBERED_SPECIAL) ? 1 : 0;
	msg->numbered_msgnum = -1;

	/* fragmentation state defaults */
	msg->fragment_index = 0;
	msg->total_fragments = 0;
	msg->retrieval_num = 0;

	/* retransmission scheduling defaults */
	msg->retransmit_max = 0;
	msg->retransmit_count = 0;
	msg->retransmit_interval = 128;
	msg->send_delay = 0;
	msg->next_send_frame = 0;
	msg->assigned_n = -1;

	/* link to tail of list */
	msg->flex = flex;
	msgp = &flex->msg_list;
	while ((*msgp))
		msgp = &(*msgp)->next;
	(*msgp) = msg;

	/* Type-specific enqueue logging */
	switch (msg_type) {
	case FLEX_MSG_TYPE_SECURE:
		LOGP(DFLEX, LOGL_INFO,
		     "Enqueue: capcode=%" PRIu64 " type=secure subtype=%s len=%d\n",
		     capcode,
		     (msg->secure_encoding == 0) ? "alpha" : "binary",
		     data_length);
		break;
	case FLEX_MSG_TYPE_SPECIAL_NUM:
		LOGP(DFLEX, LOGL_INFO,
		     "Enqueue: capcode=%" PRIu64 " type=special_num len=%d\n",
		     capcode, data_length);
		break;
	case FLEX_MSG_TYPE_NUMBERED_NUM:
	case FLEX_MSG_TYPE_NUMBERED_SPECIAL:
		LOGP(DFLEX, LOGL_INFO,
		     "Enqueue: capcode=%" PRIu64 " type=%s msgnum=%d len=%d\n",
		     capcode, flex_msg_type_name(msg->msg_type),
		     msg->numbered_msgnum,
		     data_length);
		break;
	default:
		break;
	}

	/* kick transmitter */
	if (flex->state == FLEX_STATE_IDLE) {
		if (flex->network_mode) {
			if (flex->sched_ers_done)
				flex_new_state(flex, FLEX_STATE_NET_FRAME);
			else
				flex_trigger_ers(flex);
		} else {
			if (flex->no_ers)
				flex_new_state(flex, FLEX_STATE_MESSAGE);
			else
				flex_trigger_ers(flex);
		}
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

/* ===== TX Temporary Group Scheduling (§3.8.2.3, §3.9.6) ===== */

/* Frames of margin between last SETUP and DELIVERY target frame */
#define FLEX_TG_SETUP_MARGIN	4

/* Allocate a free temp slot. Returns 0-15, or -1 if all occupied. */
static int flex_tg_alloc_slot(flex_t *flex)
{
	int i;
	for (i = 0; i < FLEX_TEMP_ADDR_SLOTS; i++) {
		if (flex->tx_temp[i].state == FLEX_TG_FREE)
			return i;
	}
	return -1;
}

/* Free a temp slot after delivery completes. */
static void flex_tg_free_slot(flex_t *flex, int slot)
{
	LOGP(DFLEX, LOGL_INFO,
	     "TX: TEARDOWN temp slot=%d — delivery complete.\n", slot);
	memset(&flex->tx_temp[slot], 0, sizeof(flex->tx_temp[slot]));
}

/*
 * Enqueue a temporary group message.
 *
 * Allocates a temp slot, creates SETUP instruction messages for each
 * member capcode, and creates the DELIVERY message with the temp
 * address word.  All messages go into the normal queue.
 *
 * Returns 0 on success, -1 on failure (no free slot, invalid params).
 */
int flex_tempgroup_enqueue(flex_t *flex, const uint64_t *capcodes, int count,
			   enum flex_msg_type msg_type, const char *data,
			   int data_length, int speed, int modulation_type,
			   double polarity, int priority, int phase)
{
	int slot, i;
	uint32_t target_frame;
	flex_frame_time_t ft;
	uint32_t abs_frame;

	if (count <= 0 || count > FLEX_TEMP_GROUP_MAX_MEMBERS) {
		LOGP(DFLEX, LOGL_NOTICE,
		     "FIFO: tempgroup requires 1-%d capcodes, got %d.\n",
		     FLEX_TEMP_GROUP_MAX_MEMBERS, count);
		return -1;
	}

	/* Validate all capcodes */
	for (i = 0; i < count; i++) {
		if (!flex_capcode_valid(capcodes[i])) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: tempgroup capcode %" PRIu64 " invalid.\n",
			     capcodes[i]);
			return -1;
		}
	}

	/* Allocate slot */
	slot = flex_tg_alloc_slot(flex);
	if (slot < 0) {
		LOGP(DFLEX, LOGL_NOTICE,
		     "FIFO: tempgroup — no free slot (all 16 occupied), rejecting.\n");
		return -1;
	}

	/* Compute target frame: current + count (for SETUPs) + margin */
	if (flex_scheduler_get_time(flex, &ft) < 0) {
		abs_frame = 0;
	} else {
		abs_frame = ft.cycle * 128 + ft.frame;
	}
	target_frame = (abs_frame + (uint32_t)count + FLEX_TG_SETUP_MARGIN) % 128;

	/* Populate slot */
	flex->tx_temp[slot].state = FLEX_TG_SETUP;
	flex->tx_temp[slot].count = count;
	flex->tx_temp[slot].setups_sent = 0;
	flex->tx_temp[slot].target_frame = target_frame;
	flex->tx_temp[slot].setup_abs = abs_frame;
	for (i = 0; i < count; i++)
		flex->tx_temp[slot].capcodes[i] = capcodes[i];

	/* Create SETUP instruction messages (one per capcode) */
	for (i = 0; i < count; i++) {
		uint32_t idata = 0;
		char ibuf[16];
		flex_msg_t *msg;

		idata = (FLEX_INSTR_TYPE_TEMP_ADDR & FLEX_INSTR_TYPE_MASK);
		idata |= ((uint32_t)target_frame & FLEX_INSTR_FRAME_MASK)
			 << FLEX_INSTR_FRAME_SHIFT;
		idata |= ((uint32_t)slot & FLEX_INSTR_SLOT_MASK)
			 << FLEX_INSTR_SLOT_SHIFT;

		snprintf(ibuf, sizeof(ibuf), "%u", idata);
		msg = flex_msg_create(flex, capcodes[i],
				      FLEX_MSG_TYPE_INSTRUCTION,
				      ibuf, (int)strlen(ibuf));
		if (!msg) {
			LOGP(DFLEX, LOGL_ERROR,
			     "TX: tempgroup SETUP failed for cap=%" PRIu64 ".\n",
			     capcodes[i]);
			flex_tg_free_slot(flex, slot);
			return -1;
		}
		msg->speed = speed;
		msg->modulation_type = modulation_type;
		msg->polarity = polarity;
		msg->priority = 1; /* SETUP is priority */
		if (phase >= 0)
			msg->phase = phase;

		LOGP(DFLEX, LOGL_INFO,
		     "TX: SETUP slot=%d frame=%u cap=%" PRIu64 " (%d/%d) idata=0x%04X\n",
		     slot, target_frame, capcodes[i], i + 1, count, idata);
	}

	/* Create DELIVERY message with temp address */
	{
		flex_msg_t *msg;

		/* Use capcode 1 as a placeholder — the frame builder will
		 * use the temp address word instead (temp_delivery_slot >= 0).
		 * The capcode field is not used for addressing. */
		msg = flex_msg_create(flex, 1, msg_type, data, data_length);
		if (!msg) {
			LOGP(DFLEX, LOGL_ERROR,
			     "TX: tempgroup DELIVERY creation failed for slot=%d.\n",
			     slot);
			flex_tg_free_slot(flex, slot);
			return -1;
		}
		msg->speed = speed;
		msg->modulation_type = modulation_type;
		msg->polarity = polarity;
		msg->priority = priority;
		msg->temp_delivery_slot = slot;
		if (phase >= 0)
			msg->phase = phase;

		/* Defer delivery until target frame */
		{
			uint32_t delay = ((target_frame + 128) - (abs_frame % 128)) % 128;
			if (delay == 0)
				delay = 128; /* full cycle if exact match */
			msg->send_delay = (int)delay;
			msg->next_send_frame = abs_frame + delay;
		}

		flex->tx_temp[slot].delivery_msg = msg;

		LOGP(DFLEX, LOGL_INFO,
		     "TX: DELIVERY queued slot=%d frame=%u type=%s members=%d len=%d\n",
		     slot, target_frame, flex_msg_type_name(msg_type),
		     count, data_length);
	}

	return 0;
}

/*
 * Tick the temp group scheduler — called each frame.
 * Checks for completed deliveries and frees slots.
 */
void flex_tempgroup_tick(flex_t *flex, uint32_t abs_frame)
{
	int i;

	for (i = 0; i < FLEX_TEMP_ADDR_SLOTS; i++) {
		if (flex->tx_temp[i].state == FLEX_TG_FREE)
			continue;

		/* Check if delivery message has been consumed (removed from queue) */
		if (flex->tx_temp[i].delivery_msg) {
			flex_msg_t *m;
			int found = 0;
			for (m = flex->msg_list; m; m = m->next) {
				if (m == flex->tx_temp[i].delivery_msg) {
					found = 1;
					break;
				}
			}
			if (!found) {
				/* Delivery consumed — teardown */
				flex_tg_free_slot(flex, i);
				continue;
			}
		}

		/* 128-frame timeout from SETUP */
		{
			uint32_t elapsed = abs_frame - flex->tx_temp[i].setup_abs;
			if (elapsed > 256) /* handle wraparound conservatively */
				elapsed = 256;
			if (elapsed >= 128) {
				LOGP(DFLEX, LOGL_NOTICE,
				     "TX: temp slot=%d TIMEOUT — %u frames since SETUP, clearing.\n",
				     i, elapsed);
				/* Remove delivery msg from queue if still there */
				if (flex->tx_temp[i].delivery_msg) {
					flex_msg_t *m;
					for (m = flex->msg_list; m; m = m->next) {
						if (m == flex->tx_temp[i].delivery_msg) {
							flex_msg_destroy(m);
							break;
						}
					}
				}
				memset(&flex->tx_temp[i], 0, sizeof(flex->tx_temp[i]));
			}
		}
	}
}

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

	/* Compute actual BIW count for this instance.
	 * This determines how many frame words are consumed by BIW,
	 * which reduces the space available for message data. */
	int biw_count = 1; /* BIW1 always present */
	if (flex->ssid || flex->nid)
		biw_count++;
	if (flex->country_code || flex->tmf)
		biw_count++;
	if (flex->biw_time_enabled)
		biw_count += 2; /* Date + Time */
	if (flex->timezone_code >= 0 && flex->timezone_code < (int)FLEX_TZ_ENTRIES)
		biw_count++;
	if (biw_count > 4)
		biw_count = 4; /* max e_biw=3 → 4 total */

	for (msg = flex->msg_list; msg; msg = next) {
		next = msg->next;

		/* Only fragment alpha, numeric, and hex messages */
		switch (msg->msg_type) {
		case FLEX_MSG_TYPE_ALPHA: {
			/* Available message words = 88 - biw - addr - vector.
			 * Short addr = 1 word, long = 2.  Vector = 1 word.
			 * Alpha: chars = (msg_words - 1) * 3 - 2
			 *   (1 header word, signature eats 1 char slot,
			 *    ETX padding eats 1 more). */
			int addr_words = (msg->capcode >= FLEX_LONG_ADDR_MIN) ? 2 : 1;
			int msg_words_avail = FLEX_WORDS_PER_FRAME - biw_count
					      - addr_words - 1; /* 1 vector */
			max_chars = (msg_words_avail - 1) * 3 - 2;
			if (max_chars > FLEX_MAX_CHARS_ALPHA)
				max_chars = FLEX_MAX_CHARS_ALPHA;
			if (max_chars < 1)
				max_chars = 1;
			break;
		}
		case FLEX_MSG_TYPE_NUMERIC:
			max_chars = FLEX_MAX_CHARS_NUMERIC;
			break;
		case FLEX_MSG_TYPE_HEX: {
			/* Available message words = 88 - biw - addr - vector.
			 * HEX: initial fragment has 2 header words (hdr1+hdr2),
			 * continuation has 1 (hdr1 only).
			 * Data words hold 5 hex nibbles each.
			 * Use initial fragment sizing (worst case = 2 headers). */
			int addr_words = (msg->capcode >= FLEX_LONG_ADDR_MIN) ? 2 : 1;
			int msg_words_avail = FLEX_WORDS_PER_FRAME - biw_count
					      - addr_words - 1; /* 1 vector */
			int data_words = msg_words_avail - 2; /* 2 header words */
			max_chars = data_words * 5;
			if (max_chars > FLEX_MAX_CHARS_HEX)
				max_chars = FLEX_MAX_CHARS_HEX;
			if (max_chars < 1)
				max_chars = 1;
			break;
		}
		case FLEX_MSG_TYPE_SECURE: {
			/* Secure messages use the same fragmentation as alpha
			 * (for alpha encoding) or hex (for binary encoding).
			 * The encoding mode is determined by secure_encoding. */
			int addr_words = (msg->capcode >= FLEX_LONG_ADDR_MIN) ? 2 : 1;
			int msg_words_avail = FLEX_WORDS_PER_FRAME - biw_count
					      - addr_words - 1; /* 1 vector */
			if (msg->secure_encoding == 0) {
				/* Alpha encoding: same sizing as alpha */
				max_chars = (msg_words_avail - 1) * 3 - 2;
				if (max_chars > FLEX_MAX_CHARS_ALPHA)
					max_chars = FLEX_MAX_CHARS_ALPHA;
			} else {
				/* Binary encoding: same sizing as hex */
				int data_words = msg_words_avail - 2;
				max_chars = data_words * 5;
				if (max_chars > FLEX_MAX_CHARS_HEX)
					max_chars = FLEX_MAX_CHARS_HEX;
			}
			if (max_chars < 1)
				max_chars = 1;
			break;
		}
		default:
			/* Numeric types (standard, special format, numbered)
			 * are single-frame only — no fragmentation needed. */
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
		flex->frag_retrieval_seq &= 0x3F; /* 6-bit counter: N is 0-63 per spec */

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
			frag->temp_delivery_slot = msg->temp_delivery_slot;
			memcpy(frag->source_id, msg->source_id, sizeof(frag->source_id));
			frag->short_msg_type = msg->short_msg_type;
			frag->short_msg_source = msg->short_msg_source;
			frag->short_msg_number = msg->short_msg_number;
			frag->short_msg_r = msg->short_msg_r;
			frag->blocking_length = msg->blocking_length;
			frag->mail_drop = msg->mail_drop;
			frag->phase = msg->phase;
			frag->secure_subtype = msg->secure_subtype;
			frag->secure_encoding = msg->secure_encoding;

			/* Copy retransmission parameters from original.
			 * retransmit_count/retransmit_max are authoritative
			 * on the first fragment (index 0); other fragments
			 * inherit the values for reference but the scheduler
			 * only checks/updates the first fragment. */
			frag->retransmit_max = msg->retransmit_max;
			frag->retransmit_interval = msg->retransmit_interval;
			frag->send_delay = msg->send_delay;
			frag->next_send_frame = msg->next_send_frame;

			/* Only the first fragment (index 0) is initially
			 * eligible for transmission.  Non-first fragments
			 * become eligible when the preceding fragment's
			 * post-transmit handler sets their next_send_frame.
			 * Place them 959 frames ahead (half the 1920-frame
			 * hour minus 1) so frame_is_eligible() rejects them. */
			if (i > 0)
				frag->next_send_frame = (msg->next_send_frame + 959) % 1920;

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
 * Set up the split sync/data buffers for a frame.
 *
 * Per ARIB STD-43A Section 3.2, the S1 sync (BS1 + A + B + A_inv) and FIW
 * are ALWAYS transmitted at 1600 baud / 2-FSK, regardless of the frame's
 * data speed.  The DSP consumes sync_buffer first at 1600/2FSK, then
 * switches to the target speed for frame_buffer (S2 + DATA).
 *
 * For 1600 baud frames, the target speed is still 1600/2FSK (no actual
 * speed change, but the split is maintained for consistency).
 */
static void flex_setup_frame_buffers(flex_t *flex,
				     const flex_frame_params_t *params,
				     const flex_frame_msg_t *msgs, int msg_count,
				     int *msgs_packed, int *error)
{
	size_t sync_len, data_len;

	/* Sync portion: S1 + FIW → sync_buffer (always 18 bytes at 1600/2FSK) */
	sync_len = flex_encode_sync(params, flex->sync_buffer,
				    sizeof(flex->sync_buffer));
	flex->sync_buffer_length = (int)sync_len;
	flex->sync_buffer_pos = 0;

	/* Data portion: S2 + interleaved phase data → frame_buffer */
	data_len = flex_encode_data(msgs, msg_count, params,
				    flex->frame_buffer,
				    sizeof(flex->frame_buffer),
				    msgs_packed, error);
	flex->frame_buffer_length = (int)data_len;
	flex->frame_buffer_pos = 0;

	/* Target speed for the data portion */
	flex->frame_target_speed = params->bitrate;
	flex->frame_target_mod_type = params->modulation_type;

	/* Start DSP at 1600/2FSK for the sync portion */
	dsp_set_speed(flex, 1600, FLEX_MOD_2FSK);

	LOGP_CHAN(DDSP, LOGL_DEBUG,
		  "Frame buffers: sync=%d bytes (1600/2fsk), data=%d bytes (%d/%s).\n",
		  (int)sync_len, (int)data_len,
		  params->bitrate,
		  (params->modulation_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk");
}

/* Map (bitrate, modulation_type) → phase count per ARIB STD-43A:
 *   A1 (1600bps, 2FSK) → 1 phase
 *   A2 (3200bps, 2FSK) → 2 phases (A, C)
 *   A3 (3200bps, 4FSK) → 2 phases (A, C) — 1600 baud × 4 levels
 *   A4 (6400bps, 4FSK) → 4 phases (A, B, C, D) — 3200 baud × 4 levels */
static int flex_get_phase_count(int bitrate, int modulation_type)
{
	if (bitrate >= 6400 && modulation_type == FLEX_MOD_4FSK)
		return 4;  /* A4 */
	if (bitrate >= 3200 && modulation_type == FLEX_MOD_4FSK)
		return 2;  /* A3: phases A+C packed into 4-level symbols */
	if (bitrate >= 3200)
		return 2;  /* A2 */
	return 1;  /* A1 */
}

/* Map user-facing phase letter to internal phase index.
 *
 * User phase values: A=0, B=1, C=2, D=3 (matching the standard names).
 * Internal phase indices are 0..num_phases-1.
 *
 * At 6400/4FSK (4 phases): A=0, B=1, C=2, D=3 — direct mapping.
 * At 3200 (2 phases): phases are A and C (not A and B).
 *   So user A=0 → index 0, user C=2 → index 1.
 *   User B=1 or D=3 are invalid for 2-phase modes → clamp to 0.
 *
 * Returns internal phase index (0..num_phases-1). */
static int flex_map_phase(int user_phase, int num_phases)
{
	if (user_phase < 0)
		return 0;
	if (num_phases >= 4)
		return user_phase % 4;
	if (num_phases == 2) {
		/* 3200 bps: phases A(0) and C(2) */
		if (user_phase == 2 || user_phase == 3)
			return 1;  /* C or D → internal index 1 (=phase C) */
		return 0;          /* A or B → internal index 0 (=phase A) */
	}
	return 0;  /* single phase */
}

/* Frame eligibility check with 1920-frame wrap-around.
 *
 * Returns 1 if next_send_frame is at or before current_frame,
 * accounting for the hourly wrap at 1920 frames (15 cycles × 128).
 * Uses a half-range threshold of 960: if the signed difference
 * is >= 0 the target is reached; if < -960 the target wrapped
 * around and is actually in the past. */
static inline int frame_is_eligible(uint32_t next_send_frame, uint32_t current_frame)
{
	int32_t diff = (int32_t)current_frame - (int32_t)next_send_frame;

	if (diff >= 0)
		return 1;
	if (diff < -960)
		return 1; /* wrapped */
	return 0;
}

/* Compare two eligible next_send_frame values to determine which
 * represents an earlier deadline.  Returns 1 if a's deadline is
 * earlier (a has been waiting longer), using the modular distance
 * from current_abs.  Both a and b must already be eligible. */
static inline int frame_deadline_earlier(uint32_t a, uint32_t b,
					 uint32_t current_abs)
{
	/* Compute how many frames each has been waiting (modular). */
	int32_t da = (int32_t)current_abs - (int32_t)a;
	int32_t db = (int32_t)current_abs - (int32_t)b;

	/* For the wrapped case (diff < -960), add 1920 to get the
	 * true modular distance within the 1920-frame hour. */
	if (da < -960) da += 1920;
	if (db < -960) db += 1920;

	return da > db;
}

/* Check whether a message type carries R/N flags and thus supports
 * message-level retransmission.
 *
 * Types with R/N: alpha (V=101), hex/binary (V=110), secure (V=000),
 *                 numbered numeric (V=111), numbered special (V=111 S=1).
 * Types without:  tone, standard numeric, special format numeric,
 *                 short instruction, short message. */
static int flex_msg_supports_retransmit(enum flex_msg_type type)
{
	switch (type) {
	case FLEX_MSG_TYPE_ALPHA:
	case FLEX_MSG_TYPE_HEX:
	case FLEX_MSG_TYPE_SECURE:
	case FLEX_MSG_TYPE_NUMBERED_NUM:
	case FLEX_MSG_TYPE_NUMBERED_SPECIAL:
		return 1;
	default:
		return 0;
	}
}

/*
 * Fragment queue helpers for retransmission.
 *
 * Fragments of the same message share the same retrieval_num and are
 * linked as consecutive flex_msg_t nodes in the queue with incrementing
 * fragment_index (0 through total_fragments-1).
 */

/* Find the first fragment (fragment_index == 0) of a message identified
 * by retrieval_num in the queue.  Returns NULL if not found. */
static flex_msg_t *flex_find_first_fragment(flex_t *flex, uint32_t retrieval_num,
					    int total_fragments)
{
	flex_msg_t *m;

	for (m = flex->msg_list; m; m = m->next) {
		if (m->total_fragments == total_fragments &&
		    m->retrieval_num == retrieval_num &&
		    m->fragment_index == 0)
			return m;
	}
	return NULL;
}

/* Find the next fragment (fragment_index == current + 1) of the same
 * message in the queue.  Returns NULL if not found. */
static flex_msg_t *flex_find_next_fragment(flex_t *flex, flex_msg_t *msg)
{
	flex_msg_t *m;
	int next_idx = msg->fragment_index + 1;

	if (next_idx >= msg->total_fragments)
		return NULL;

	for (m = flex->msg_list; m; m = m->next) {
		if (m->total_fragments == msg->total_fragments &&
		    m->retrieval_num == msg->retrieval_num &&
		    m->fragment_index == next_idx)
			return m;
	}
	return NULL;
}

/* Destroy all fragments of a message identified by retrieval_num.
 * Walks the queue and removes every node with matching retrieval_num
 * and total_fragments. */
static void flex_destroy_all_fragments(flex_t *flex, uint32_t retrieval_num,
				       int total_fragments)
{
	flex_msg_t *m, *next;

	for (m = flex->msg_list; m; m = next) {
		next = m->next;
		if (m->total_fragments == total_fragments &&
		    m->retrieval_num == retrieval_num)
			flex_msg_destroy(m);
	}
}

/*
 * Post-transmission lifecycle: decide whether to retain a message for
 * retransmission or destroy it.
 *
 * For non-fragmented messages:
 * - Message types without R/N flags: always destroy immediately.
 * - retransmit_max == 0: no retransmissions configured, destroy.
 * - retransmit_count >= retransmit_max: all retransmissions done, destroy.
 * - Otherwise: increment retransmit_count, schedule next send, retain.
 *
 * For fragmented messages (total_fragments > 1):
 * - retransmit_count/retransmit_max are tracked on the first fragment
 *   (fragment_index == 0); other fragments inherit the state.
 * - Non-last fragments: make the next fragment eligible immediately.
 * - Last fragment: if retransmissions remain, set next_send_frame on
 *   the first fragment and increment retransmit_count once for the
 *   whole cycle.  If done, destroy all fragments.
 */
static void flex_post_transmit(flex_t *flex, flex_msg_t *msg,
			       uint32_t current_abs_frame)
{
	/* Message types without R/N: always destroy */
	if (!flex_msg_supports_retransmit(msg->msg_type)) {
		flex_msg_destroy(msg);
		return;
	}

	/* --- Non-fragmented message path --- */
	if (msg->total_fragments <= 1) {
		if (msg->retransmit_max == 0 || msg->retransmit_count >= msg->retransmit_max) {
			LOGP(DFLEX, LOGL_INFO,
			     "TX_COMPLETE: capcode=%" PRIu64 " type=%s N=%d total_tx=%d\n",
			     msg->capcode, flex_msg_type_name(msg->msg_type),
			     msg->assigned_n, 1 + msg->retransmit_count);
			flex_msg_destroy(msg);
			return;
		}

		/* Retain for retransmission */
		msg->retransmit_count++;
		msg->next_send_frame = (current_abs_frame + (uint32_t)msg->retransmit_interval) % 1920;

		LOGP(DFLEX, LOGL_INFO,
		     "TX_RETRANSMIT: capcode=%" PRIu64 " type=%s N=%d count=%d/%d next_frame=%u\n",
		     msg->capcode, flex_msg_type_name(msg->msg_type),
		     msg->assigned_n, msg->retransmit_count, msg->retransmit_max,
		     msg->next_send_frame);
		return;
	}

	/* --- Fragmented message path --- */

	/* Look up the first fragment to get authoritative retransmission state */
	{
		flex_msg_t *first = flex_find_first_fragment(flex, msg->retrieval_num,
							    msg->total_fragments);

		/* Non-last fragment */
		if (msg->fragment_index < msg->total_fragments - 1) {
			flex_msg_t *next_frag = flex_find_next_fragment(flex, msg);
			if (next_frag)
				next_frag->next_send_frame = current_abs_frame;

			if (!first || first->retransmit_max == 0) {
				/* No retransmission — destroy this fragment now */
				flex_msg_destroy(msg);
			} else {
				/* Retransmission configured — keep fragment but
				 * mark ineligible until retransmission cycle
				 * restarts from the first fragment. */
				msg->next_send_frame = (current_abs_frame + 959) % 1920;
			}
			return;
		}

		/* Last fragment transmitted */

		if (!first) {
			/* First fragment missing (shouldn't happen) — clean up */
			LOGP(DFLEX, LOGL_NOTICE,
			     "Fragment stream incomplete: capcode=%" PRIu64 " retrieval=%u, destroying.\n",
			     msg->capcode, msg->retrieval_num);
			flex_destroy_all_fragments(flex, msg->retrieval_num,
						   msg->total_fragments);
			return;
		}

		if (first->retransmit_max == 0 ||
		    first->retransmit_count >= first->retransmit_max) {
			/* All retransmissions done — destroy all fragments */
			LOGP(DFLEX, LOGL_INFO,
			     "TX_COMPLETE: capcode=%" PRIu64 " type=%s N=%d total_tx=%d (fragmented, %d frags)\n",
			     msg->capcode, flex_msg_type_name(msg->msg_type),
			     first->assigned_n, 1 + first->retransmit_count,
			     msg->total_fragments);
			flex_destroy_all_fragments(flex, msg->retrieval_num,
						   msg->total_fragments);
			return;
		}

		/* Retransmissions remain — increment count on first fragment,
		 * schedule next send on first fragment only.
		 * Subsequent fragments become eligible after preceding is transmitted.
		 * Mark all non-first fragments ineligible by placing them 959 frames
		 * after the first fragment's target (well within the ineligible half
		 * of the 1920-frame modular space relative to the retransmission time). */
		first->retransmit_count++;
		first->next_send_frame = (current_abs_frame + (uint32_t)first->retransmit_interval) % 1920;

		{
			flex_msg_t *frag;
			uint32_t sentinel = (first->next_send_frame + 959) % 1920;
			for (frag = flex->msg_list; frag; frag = frag->next) {
				if (frag->total_fragments == msg->total_fragments &&
				    frag->retrieval_num == msg->retrieval_num &&
				    frag->fragment_index > 0)
					frag->next_send_frame = sentinel;
			}
		}

		LOGP(DFLEX, LOGL_INFO,
		     "TX_RETRANSMIT: capcode=%" PRIu64 " type=%s N=%d count=%d/%d next_frame=%u (fragmented, %d frags)\n",
		     msg->capcode, flex_msg_type_name(msg->msg_type),
		     first->assigned_n, first->retransmit_count, first->retransmit_max,
		     first->next_send_frame, msg->total_fragments);
	}
}

/* Compute which phase should carry SSID1 for a given frame (§6.1.1.3).
 *
 * 6400bps/4-phase: SSID1 rotates A→B→C→D per frame (frame % 4).
 * 3200bps/2-phase: phases a,b → phase A; phases c,d → phase C.
 *   So frame%4 in {0,1} → phase 0 (A), frame%4 in {2,3} → phase 1 (C).
 * 1600bps/1-phase: always phase 0 (A).
 *
 * Returns the phase index (0-based) that should carry SSID1.
 * SSID2 follows the same rotation pattern per §6.1.1.3 rule (3). */
static int flex_ssid_phase(uint32_t frame, int num_phases)
{
	int slot = (int)(frame % 4);
	switch (num_phases) {
	case 4:  return slot;			/* A=0, B=1, C=2, D=3 */
	case 2:  return (slot < 2) ? 0 : 1;	/* A=0 for 0,1; C=1 for 2,3 */
	default: return 0;			/* single phase */
	}
}

/* Compute BIW word count from frame params.
 * Returns 1–4 (BIW1 + up to 3 extra BIW words).
 *
 * Must match the encoder's inline BIW computation in
 * flex_encode_frame_multi() for consistency between the
 * scheduler's capacity estimate and the actual encoded frame. */
static int flex_compute_biw_count(const flex_frame_params_t *params)
{
	int extra = 0;

	if (params->local_id || params->coverage_id)
		extra++;
	if ((params->country_code || params->tmf) && params->frame <= 3)
		extra++;
	if (params->biw_time)
		extra += 2; /* Date + Time */
	if (params->timezone_code >= 0 &&
	    params->timezone_code < (int)FLEX_TZ_ENTRIES)
		extra++;
	if (params->chan_setup_enabled && params->frame <= 3)
		extra++;
	if (extra > 3)
		extra = 3;

	return 1 + extra;
}

/* Estimate total word cost for a queued message (address + vector + body).
 * Mirrors estimate_msg_words() logic in frame.c but operates on flex_msg_t
 * directly (queue entry struct) rather than flex_frame_msg_t (encoder input).
 * Returns total words needed, or -1 if the message is invalid.
 *
 * Address/vector overhead:
 *   Short address (capcode ≤ 2,097,151 or group): 1 addr + 1 vector word
 *   Long address  (capcode > 2,097,151, non-group): 2 addr + 2 vector words
 *   Tone-only: address words only (no vector, no body)
 *
 * For long addresses, body word count is reduced by 1 because body[0]
 * is absorbed into the second vector word (Vy). */
static int flex_estimate_msg_cost(const flex_msg_t *msg)
{
	int is_long = (msg->capcode >= FLEX_LONG_ADDR_MIN && !msg->is_group);
	int aw = is_long ? 2 : 1;
	int vw, bw;
	size_t len;

	if (msg->msg_type == FLEX_MSG_TYPE_TONE)
		return aw; /* address only, no vector, no body */

	vw = is_long ? 2 : 1;

	/* Estimate body words based on message type and length.
	 * Same formulas as estimate_msg_words() in frame.c. */
	len = (msg->data_length > 0) ? (size_t)msg->data_length : 0;

	switch (msg->msg_type) {
	case FLEX_MSG_TYPE_INSTRUCTION:
	case FLEX_MSG_TYPE_SHORT:
		bw = 0; /* vector word only */
		break;

	case FLEX_MSG_TYPE_ALPHA: {
		if (len == 0)
			return -1;
		if (msg->charset == 1) {
			/* KANJI: 16-bit chars, 1 char per word + 1 header */
			size_t num_chars = len / 2;
			if (num_chars == 0)
				return -1;
			bw = 1 + (int)num_chars;
		} else {
			int is_continuation = (msg->total_fragments > 1 &&
					       msg->fragment_index > 0);
			if (is_continuation)
				bw = 1 + (int)((len + 2) / 3);
			else
				bw = 1 + (int)((len + 2 + 2) / 3);
		}
		break;
	}

	case FLEX_MSG_TYPE_NUMERIC: {
		if (len == 0)
			return -1;
		int bits_needed = (int)len * 4 + 2;
		bw = (bits_needed + 20) / 21;
		if (bw > FLEX_MAX_MSG_WORDS_NUMERIC)
			bw = FLEX_MAX_MSG_WORDS_NUMERIC;
		if (bw < 1)
			bw = 1;
		break;
	}

	case FLEX_MSG_TYPE_HEX: {
		if (len == 0)
			return -1;
		int is_continuation = (msg->total_fragments > 1 &&
				       msg->fragment_index > 0);
		int hdr_words = is_continuation ? 1 : 2;
		int data_words = (int)((len + 4) / 5);
		int is_last_frag = (msg->total_fragments <= 1 ||
				    msg->fragment_index == msg->total_fragments - 1);
		if (is_last_frag && len > 0) {
			char last_ch = msg->data[len - 1];
			if (last_ch == '0' || last_ch == 'f' || last_ch == 'F')
				data_words++;
		}
		bw = hdr_words + data_words;
		break;
	}

	case FLEX_MSG_TYPE_SECURE: {
		if (len == 0)
			return -1;
		if (msg->secure_encoding == 1) {
			/* Binary encoding: same as hex */
			int is_continuation = (msg->total_fragments > 1 &&
					       msg->fragment_index > 0);
			int hdr_words = is_continuation ? 1 : 2;
			int data_words = (int)((len + 4) / 5);
			int is_last_frag = (msg->total_fragments <= 1 ||
					    msg->fragment_index == msg->total_fragments - 1);
			if (is_last_frag && len > 0) {
				char last_ch = msg->data[len - 1];
				if (last_ch == '0' || last_ch == 'f' || last_ch == 'F')
					data_words++;
			}
			bw = hdr_words + data_words;
		} else {
			/* Alpha encoding */
			int is_continuation = (msg->total_fragments > 1 &&
					       msg->fragment_index > 0);
			if (is_continuation)
				bw = 1 + (int)((len + 2) / 3);
			else
				bw = 1 + (int)((len + 2 + 2) / 3);
		}
		break;
	}

	case FLEX_MSG_TYPE_SPECIAL_NUM: {
		if (len == 0)
			return -1;
		int bits_needed = (int)len * 4 + 2;
		bw = (bits_needed + 20) / 21;
		if (bw > FLEX_MAX_MSG_WORDS_NUMERIC)
			bw = FLEX_MAX_MSG_WORDS_NUMERIC;
		if (bw < 1)
			bw = 1;
		break;
	}

	case FLEX_MSG_TYPE_NUMBERED_NUM:
	case FLEX_MSG_TYPE_NUMBERED_SPECIAL: {
		if (len == 0)
			return -1;
		/* ceil((len*4+10)/21) — 10-bit header (K5K4 + N + R + S) */
		int bits_needed = (int)len * 4 + FLEX_NUM_NUMBERED_SKIP_BITS;
		bw = (bits_needed + 20) / 21;
		if (bw > FLEX_MAX_MSG_WORDS_NUMERIC)
			bw = FLEX_MAX_MSG_WORDS_NUMERIC;
		if (bw < 1)
			bw = 1;
		break;
	}

	default:
		return -1;
	}

	/* Long address: body[0] absorbed into Vy */
	if (is_long && bw > 0)
		bw--;

	return aw + vw + bw;
}

/* Candidate entry for multi-message collection.
 * Built during three-pass candidate collection in the scheduler.
 * File-scope only — not exposed in headers. */
typedef struct {
	flex_msg_t      *msg;           /* pointer into msg_list */
	int              est_words;     /* estimated total word cost */
	int              addr_words;    /* 1 (short) or 2 (long) */
	int              vec_words;     /* 0 (tone), 1 (short), 2 (long) */
	int              body_words;    /* message body words */
	int              is_long;       /* 1 if capcode > 2,097,151 */
} flex_candidate_t;

/* In-flight fragment tracking entry.
 * Built by scanning the queue before candidate collection.
 * A capcode is "in-flight" if it has a fragmented message where
 * fragment_index 0 has been transmitted (assigned_n >= 0) but
 * the last fragment has not yet been transmitted.
 * File-scope only — not exposed in headers. */
typedef struct {
	uint64_t         capcode;
	uint32_t         retrieval_num;  /* identifies the fragment stream */
	int              total_fragments;
	int              last_sent_idx;  /* highest fragment_index already sent */
	uint32_t         last_sent_frame;/* abs frame when last fragment was sent */
} flex_inflight_frag_t;

/* Fragment-transmission slot: one (fragment, transmission_repeat) pair
 * to be placed in a specific subframe of a specific frame.
 * File-scope only — not exposed in headers. */
typedef struct {
	flex_msg_t      *msg;           /* pointer to the fragment in msg_list */
	int              fragment_idx;  /* 0-based fragment index */
	int              tx_repeat;     /* 1-based transmission repeat number */
	int              subframe_idx;  /* 0-based subframe index within frame */
} flex_frag_slot_t;

/* Per-frame fragment plan: which fragment-transmission pairs go into
 * this frame, and how many continuing frames have been used so far.
 * File-scope only — not exposed in headers. */
typedef struct {
	uint32_t         frame_number;          /* absolute frame number */
	int              is_collapse_aligned;   /* 1 if this is a Collapse_Aligned_Frame */
	int              is_continuing;         /* 1 if this is a Continuing_Frame (α frame) */
	int              n_frag_slots;          /* number of fragment slots placed */
	flex_frag_slot_t frag_slots[FLEX_MAX_FRAG_SLOTS]; /* max 4 subframes per frame */
	int              remaining_capacity;    /* words left after fragments for co-packing */
} flex_frame_frag_plan_t;

/* Collapse-cycle state for an in-flight fragment stream.
 * File-scope only — not exposed in headers. */
typedef struct {
	uint64_t         capcode;
	uint32_t         retrieval_num;
	int              total_fragments;
	int              next_fragment_idx;     /* next fragment to transmit */
	int              next_tx_repeat;        /* next transmission repeat (1-based) */
	int              alpha_used;            /* continuing frames used so far */
	int              alpha_max;             /* max continuing frames = 2^m - 1 */
	uint32_t         collapse_frame;        /* the Collapse_Aligned_Frame that started this burst */
} flex_collapse_frag_state_t;

/* Check if a frame is collapse-aligned for a given capcode.
 * Returns 1 if frame_number mod 2^m == assigned_frame mod 2^m.
 * When collapse <= 0, every frame is "aligned" (return 1). */
static inline int is_collapse_aligned(uint32_t frame_number,
                                      uint32_t assigned_frame,
                                      int collapse)
{
	if (collapse <= 0)
		return 1; /* no collapse — every frame is "aligned" */
	uint32_t period = 1U << collapse;
	return (frame_number % period) == (assigned_frame % period);
}

/* Scan the message queue to identify capcodes with in-flight fragmented
 * transmissions (§4.2 ①②).  A capcode is "in-flight" when:
 *   - A fragment with fragment_index == 0 has been sent (assigned_n >= 0)
 *   - The last fragment (fragment_index == total_fragments - 1) has NOT
 *     been sent yet
 *
 * Returns the number of in-flight entries written to inflight[]. */
static int flex_scan_inflight(flex_t *flex, flex_inflight_frag_t *inflight,
                              int max_inflight)
{
	int n_inflight = 0;
	flex_msg_t *m, *f;

	for (m = flex->msg_list; m && n_inflight < max_inflight; m = m->next) {
		flex_msg_t *last;
		int highest_sent_idx;
		int already_tracked;
		int k;

		/* Only interested in fragment_index==0 of multi-fragment streams */
		if (m->total_fragments <= 1)
			continue;
		if (m->fragment_index != 0)
			continue;
		/* Fragment 0 must have been transmitted at least once */
		if (m->assigned_n < 0)
			continue;

		/* Search for the last fragment and track highest sent index */
		last = NULL;
		highest_sent_idx = 0;
		for (f = flex->msg_list; f; f = f->next) {
			if (f->total_fragments != m->total_fragments)
				continue;
			if (f->retrieval_num != m->retrieval_num)
				continue;
			if (f->assigned_n >= 0 && f->fragment_index > highest_sent_idx)
				highest_sent_idx = f->fragment_index;
			if (f->fragment_index == m->total_fragments - 1)
				last = f;
		}

		/* If last fragment exists and has been sent, stream is complete */
		if (last && last->assigned_n >= 0)
			continue;

		/* Avoid duplicate entries for the same capcode */
		already_tracked = 0;
		for (k = 0; k < n_inflight; k++) {
			if (inflight[k].capcode == m->capcode) {
				already_tracked = 1;
				break;
			}
		}
		if (already_tracked)
			continue;

		/* Record this capcode as in-flight */
		inflight[n_inflight].capcode = m->capcode;
		inflight[n_inflight].retrieval_num = m->retrieval_num;
		inflight[n_inflight].total_fragments = m->total_fragments;
		inflight[n_inflight].last_sent_idx = highest_sent_idx;
		inflight[n_inflight].last_sent_frame = 0;
		n_inflight++;
	}

	return n_inflight;
}

/* Check if a message is excluded by fragmentation rules (§4.2 ①②).
 * Returns 1 if the message should be skipped, 0 if allowed.
 *
 * Rules:
 *  ① Same capcode can't start new fragmented TX while one is in-flight
 *  ② Same capcode can't appear for unfragmented msg while fragment in-flight
 *  Exception: the NEXT eligible fragment of the in-flight stream IS allowed */
static inline int frag_excluded(const flex_msg_t *m,
                                const flex_inflight_frag_t *inflight,
                                int n_inflight)
{
	int k;
	for (k = 0; k < n_inflight; k++) {
		if (inflight[k].capcode != m->capcode)
			continue;

		/* Same capcode has in-flight fragment stream */

		/* Is this message the next fragment of THAT stream? */
		if (m->total_fragments == inflight[k].total_fragments &&
		    m->retrieval_num == inflight[k].retrieval_num &&
		    m->fragment_index == inflight[k].last_sent_idx + 1) {
			return 0; /* allowed: this IS the continuation */
		}

		/* §4.2 ①: new fragmented message for same capcode → blocked */
		if (m->total_fragments > 1)
			return 1;

		/* §4.2 ②: unfragmented message for same capcode → blocked */
		return 1;
	}
	return 0; /* capcode has no in-flight fragments → allowed */
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
	/* Multi-message candidate collection */
	flex_candidate_t candidates[FLEX_MAX_CANDIDATES];
	int n_candidates = 0;
	flex_inflight_frag_t inflight[FLEX_MAX_INFLIGHT];
	int n_inflight = 0;
	int est_used = 0;
	int capacity = 0;

	/* === ERS streaming phase ===
	 * ERS is a standalone re-sync burst emitted before data frames.
	 * Polarity is irrelevant: each cycle contains both Ar and Ar_inv,
	 * so pagers of either polarity detect the re-sync pattern. */
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
			flex->sync_buffer_length = 0; /* ERS has no sync portion */
			flex->sync_buffer_pos = 0;
			flex->ers_sent_cycles += chunk;

			LOGP_CHAN(DFLEX, LOGL_DEBUG,
				  "Network mode: ERS chunk %d/%d cycles.\n",
				  flex->ers_sent_cycles, flex->ers_total_cycles);
			return 1;
		}

		/* ERS complete */
		LOGP_CHAN(DFLEX, LOGL_INFO,
			  "Network mode: ERS burst complete (%d cycles).\n",
			  flex->ers_total_cycles);
		flex->sched_ers_done = 1;

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

	/* Tick temp group scheduler */
	flex_tempgroup_tick(flex, ft.cycle * 128 + ft.frame);

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
		flex->sync_buffer_length = 0; /* POCSAG has no FLEX sync portion */
		flex->sync_buffer_pos = 0;
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
	params.chan_setup_enabled = flex->chan_setup_enabled;
	params.hack_nonstandard_decoders = flex->hack_nonstandard_decoders;
	params.bitrate = flex_scheduler_select_speed(flex, &params.modulation_type);

	/* Multiple transmission params (Spec Section 3.4.2) */
	params.num_transmissions = flex->num_transmissions;
	params.td_collapse = flex->td_collapse;
	if (flex->num_transmissions > 1) {
		params.subframe_index = flex_scheduler_subframe_index(
			ft.frame, flex->num_transmissions,
			flex->collapse, flex->td_collapse);
	}

	/* DSP speed is set by flex_setup_frame_buffers() or the phased
	 * encoding path — always starts at 1600/2FSK for sync_buffer,
	 * then switches to target speed for frame_buffer. */

	/* Coverage/roaming fields */
	if (flex->ssid || flex->nid) {
		params.local_id = flex->ssid;
		params.coverage_id = flex->nid;
	}
	if (flex->country_code || flex->tmf) {
		params.country_code = flex->country_code;
		params.tmf = flex->tmf;
	}
	params.timezone_code = flex->timezone_code;

	/* === Multi-message candidate collection ===
	 * Collect multiple eligible messages into a candidate list,
	 * ordered by priority class and deadline.
	 *
	 * Three-pass collection:
	 *   Pass 0: Priority messages (non-tone), fragment continuations first
	 *   Pass 1: Normal messages (non-tone), fragment continuations first
	 *   Pass 2: Tone-only messages (if no system message content)
	 *
	 * Within each pass, messages are insertion-sorted by composite key:
	 *   (is_fragment_continuation DESC, next_send_frame ASC)
	 */
	uint32_t current_abs = ft.cycle * 128 + ft.frame;

	/* Scan for in-flight fragment streams (§4.2 ①②) */
	n_inflight = flex_scan_inflight(flex, inflight, FLEX_MAX_INFLIGHT);

	/* Compute available frame capacity */
	{
		int biw_count = flex_compute_biw_count(&params);
		if (params.num_transmissions > 1)
			capacity = flex_subframe_words(params.num_transmissions) - biw_count;
		else
			capacity = FLEX_WORDS_PER_FRAME - biw_count;

		/* Determine if tone-only messages are excluded.
		 * Per spec §3.9.2: "Tone-Only Addresses cannot be
		 * transmitted in Frames used for transmitting System
		 * Messages."  This applies when the frame contains
		 * system message content — i.e., an Operator Messaging
		 * Address or Network Address with a message body.
		 * Timezone-only BIW (A=0100) does NOT exclude tone-only
		 * since it carries data only in the I-field, not in MF. */
		int has_sysmsg_content = 0;
		{
			flex_msg_t *scan;
			for (scan = flex->msg_list; scan; scan = scan->next) {
				enum flex_addr_type stype;
				if (scan->speed != params.bitrate ||
				    scan->modulation_type != params.modulation_type)
					continue;
				if (!frame_is_eligible(scan->next_send_frame, current_abs))
					continue;
				stype = flex_capcode_special_type(scan->capcode);
				if (stype == FLEX_ADDR_OPER_MSG ||
				    stype == FLEX_ADDR_NETWORK) {
					has_sysmsg_content = 1;
					break;
				}
			}
		}

		/* Co-packing (§4.2.3, Req 20.1-20.5):
		 *
		 * The three-pass collection inherently implements co-packing
		 * of other-capcode messages alongside fragments. Fragment
		 * continuations sort first within each priority class via
		 * the composite sort key (is_continuation DESC, deadline ASC),
		 * ensuring fragment addresses/vectors are placed before
		 * other-capcode addresses in the encoded frame. The
		 * frag_excluded() check prevents same-capcode conflicts
		 * (§4.2 ①②) but allows different capcodes. Remaining
		 * capacity after fragments is naturally filled with
		 * other-capcode messages as est_used accumulates across
		 * all candidates.
		 *
		 * Per-frame independent co-packing (Req 20.5) is inherent
		 * in the frame-at-a-time architecture: each frame cycle
		 * runs the full collection fresh with est_used starting
		 * at 0, and unpacked messages remain in the queue for
		 * the next frame. */

		/* Three-pass collection */
		int pass;
		for (pass = 0; pass < 3; pass++) {
			/* Build a temporary sorted list of eligible messages for this pass */
			flex_msg_t *sorted[FLEX_MAX_CANDIDATES];
			int n_sorted = 0;
			flex_msg_t *candidate;

			for (candidate = flex->msg_list; candidate; candidate = candidate->next) {
				int is_tone, m_is_cont, pos;

				/* Filter: speed, modulation, eligibility */
				if (candidate->speed != params.bitrate ||
				    candidate->modulation_type != params.modulation_type)
					continue;
				if (!frame_is_eligible(candidate->next_send_frame, current_abs))
					continue;

				/* Collapse filter */
				if (flex->collapse > 0) {
					flex_capcode_sched_t sched;
					uint32_t nf;
					flex_scheduler_capcode_info(candidate->capcode, &sched);
					nf = flex_scheduler_next_frame(ft.frame,
								       sched.assigned_frame,
								       flex->collapse);
					if (nf != ft.frame)
						continue;
				}

				/* System message frame 0 preference (§3.9.2).
				 *
				 * "A System Message must be initiated in Frame 0
				 * when a BIW position is open for System Messaging.
				 * If a BIW position in Frame 0 is not available,
				 * the first Frame transmitted after Frame 0 which
				 * has an available BIW position is to be used."
				 *
				 * Skip system messages (Operator Messaging and
				 * Network addresses) in non-frame-0 frames, unless
				 * the message has been waiting for more than one
				 * full cycle (128 frames) — safety valve to prevent
				 * starvation if frame 0 is perpetually full. */
				{
					enum flex_addr_type stype =
						flex_capcode_special_type(candidate->capcode);
					if ((stype == FLEX_ADDR_OPER_MSG ||
					     stype == FLEX_ADDR_NETWORK) &&
					    ft.frame != 0) {
						int32_t wait = (int32_t)current_abs
							     - (int32_t)candidate->next_send_frame;
						if (wait < 0)
							wait += 1920;
						if (wait < 128)
							continue; /* defer to frame 0 */
						LOGP(DFLEX, LOGL_INFO,
						     "Scheduler: sysmsg capcode=%" PRIu64
						     " waited %d frames, allowing non-F0"
						     " frame %u (§3.9.2 fallback)\n",
						     candidate->capcode, wait,
						     ft.frame);
					}
				}

				/* Pass filter */
				is_tone = (candidate->msg_type == FLEX_MSG_TYPE_TONE);
				if (pass == 0 && (!candidate->priority || is_tone))
					continue;
				if (pass == 1 && (candidate->priority || is_tone))
					continue;
				if (pass == 2 && !is_tone)
					continue;
				if (pass == 2 && has_sysmsg_content)
					continue;

				/* Fragmentation address exclusion (§4.2 ①②) */
				if (frag_excluded(candidate, inflight, n_inflight)) {
					LOGP(DFLEX, LOGL_DEBUG,
					     "Scheduler: skip capcode=%" PRIu64 " frag_idx=%d — "
					     "fragmentation address exclusion (§4.2)\n",
					     candidate->capcode, candidate->fragment_index);
					continue;
				}

				if (n_sorted >= FLEX_MAX_CANDIDATES)
					break;

				/* Determine if this is a fragment continuation */
				m_is_cont = 0;
				{
					int k;
					for (k = 0; k < n_inflight; k++) {
						if (inflight[k].capcode == candidate->capcode &&
						    candidate->total_fragments == inflight[k].total_fragments &&
						    candidate->retrieval_num == inflight[k].retrieval_num &&
						    candidate->fragment_index == inflight[k].last_sent_idx + 1) {
							m_is_cont = 1;
							break;
						}
					}
				}

				/* Insertion sort by (is_continuation DESC, deadline ASC) */
				pos = n_sorted;
				while (pos > 0) {
					flex_msg_t *s = sorted[pos - 1];
					int s_is_cont = 0;
					int k;
					for (k = 0; k < n_inflight; k++) {
						if (inflight[k].capcode == s->capcode &&
						    s->total_fragments == inflight[k].total_fragments &&
						    s->retrieval_num == inflight[k].retrieval_num &&
						    s->fragment_index == inflight[k].last_sent_idx + 1) {
							s_is_cont = 1;
							break;
						}
					}
					if (m_is_cont && !s_is_cont) {
						pos--;
						continue;
					}
					if (!m_is_cont && s_is_cont)
						break;
					if (frame_deadline_earlier(candidate->next_send_frame,
								  s->next_send_frame, current_abs))
						pos--;
					else
						break;
				}
				{
					int k;
					for (k = n_sorted; k > pos; k--)
						sorted[k] = sorted[k - 1];
				}
				sorted[pos] = candidate;
				n_sorted++;
			}

			/* Add sorted candidates until capacity exhausted */
			{
				int i;
				for (i = 0; i < n_sorted && n_candidates < FLEX_MAX_CANDIDATES; i++) {
					flex_msg_t *m = sorted[i];
					int is_long = (m->capcode >= FLEX_LONG_ADDR_MIN && !m->is_group);
					int cost = flex_estimate_msg_cost(m);
					int aw, vw, bw;

					if (cost < 0)
						continue; /* invalid message */

					if (est_used + cost > capacity) {
						LOGP(DFLEX, LOGL_DEBUG,
						     "Scheduler: skip capcode=%" PRIu64
						     " est=%d words (capacity=%d used=%d)\n",
						     m->capcode, cost, capacity, est_used);
						continue; /* skip, try smaller */
					}

					aw = is_long ? 2 : 1;
					if (m->msg_type == FLEX_MSG_TYPE_TONE) {
						vw = 0;
						bw = 0;
					} else {
						vw = is_long ? 2 : 1;
						bw = cost - aw - vw;
					}

					candidates[n_candidates].msg = m;
					candidates[n_candidates].est_words = cost;
					candidates[n_candidates].addr_words = aw;
					candidates[n_candidates].vec_words = vw;
					candidates[n_candidates].body_words = bw;
					candidates[n_candidates].is_long = is_long;
					n_candidates++;
					est_used += cost;
				}
			}
		}
	}

	/* Set msg pointer for multi-phase gate check below.
	 * If we collected any candidates, msg is non-NULL to enter
	 * the encoding path; otherwise fall through to idle. */
	msg = (n_candidates > 0) ? candidates[0].msg : NULL;

	/* Multi-phase encoding for A2/A3 (2 phases) and A4 (4 phases).
	 * A1 is single-phase and skips this block. */
	int num_phases = flex_get_phase_count(params.bitrate, params.modulation_type);
	if (msg && num_phases > 1) {
		flex_phase_data_t phases[FLEX_MAX_PHASES];
		flex_frame_params_t phase_params;
		uint8_t phase_buf[FLEX_BUFFER_SIZE];
		int i, j, p, any_msg = 0;

		/* Per-phase candidate distribution (Req 2.1, 2.2).
		 * The three-pass collection already built candidates[].
		 * Distribute them into per-phase lists by capcode phase. */
		flex_candidate_t phase_cands[FLEX_MAX_PHASES][FLEX_MAX_CANDIDATES];
		int phase_n_cands[FLEX_MAX_PHASES];
		int phase_est_used[FLEX_MAX_PHASES];

		memset(phases, 0, sizeof(phases));
		memset(phase_n_cands, 0, sizeof(phase_n_cands));
		memset(phase_est_used, 0, sizeof(phase_est_used));

		/* Build per-phase params — same as frame params but force
		 * single-phase output so we can extract the 88 data words */
		phase_params = params;
		phase_params.single_phase = 1;

		{
			int biw_count = flex_compute_biw_count(&params);
			int phase_capacity = FLEX_WORDS_PER_FRAME - biw_count;

			/* Distribute candidates to per-phase lists */
			for (i = 0; i < n_candidates; i++) {
				flex_msg_t *m = candidates[i].msg;
				flex_capcode_sched_t sched;
				int phase_idx;

				flex_scheduler_capcode_info(m->capcode, &sched);
				phase_idx = (int)(sched.assigned_phase % (uint32_t)num_phases);
				if (m->phase >= 0)
					phase_idx = flex_map_phase(m->phase, num_phases);

				if (phase_n_cands[phase_idx] >= FLEX_MAX_CANDIDATES)
					continue;
				if (phase_est_used[phase_idx] + candidates[i].est_words > phase_capacity)
					continue;

				phase_cands[phase_idx][phase_n_cands[phase_idx]] = candidates[i];
				phase_n_cands[phase_idx]++;
				phase_est_used[phase_idx] += candidates[i].est_words;
			}
		}

		/* Carry-on computation for multi-phase (Spec §3.7.1 / §4.2.1).
		 *
		 * Per spec: "values for Carry On must be identical for all
		 * phases in one Frame."  Scan the queue for fragment messages
		 * eligible for this frame and compute carry-on from the max
		 * remaining fragment count.  Alpha-bounded per §4.2.2 Rule ②
		 * (Req 16.4, 17.1).  Must be done before per-phase encoding
		 * since BIW1 is baked into the encoded words.
		 * Only meaningful with collapse > 0.
		 * Per spec: "Carry On is not allowed for multiple transmission." */
		if (flex->collapse > 0 && flex->num_transmissions <= 1) {
			flex_msg_t *qm;
			int max_remaining = 0;
			for (qm = flex->msg_list; qm; qm = qm->next) {
				if (qm->speed != params.bitrate ||
				    qm->modulation_type != params.modulation_type)
					continue;
				if (qm->total_fragments > 1 &&
				    qm->fragment_index < qm->total_fragments - 1) {
					int rem = qm->total_fragments - 1 - qm->fragment_index;
					if (rem > max_remaining)
						max_remaining = rem;
				}
			}
			/* Alpha constraint (§4.2.2 Rule ②, Req 16.4, 17.1):
			 * carry-on cannot exceed alpha_max = 2^m - 1, since
			 * the number of continuing frames must be strictly
			 * less than the collapse cycle length. */
			{
				int alpha_max = (1 << flex->collapse) - 1;
				if (max_remaining > alpha_max)
					max_remaining = alpha_max;
			}
			if (max_remaining > 0) {
				params.carry_on = (max_remaining > 3) ? 3 : max_remaining;
				phase_params.carry_on = params.carry_on;
			}
		}

		/* Encode each phase independently (Req 2.2, 2.3) */
		for (p = 0; p < num_phases; p++) {
			if (phase_n_cands[p] == 0) {
				flex_fill_idle_phase(phases[p].words, p,
						     params.modulation_type,
						     params.bitrate);
				phases[p].word_count = FLEX_WORDS_PER_FRAME;
				continue;
			}

			/* Per-phase SSID placement (§6.1.1.3).
			 * SSID1/SSID2 rotate across phases per frame.
			 * Clear SSID fields for phases that shouldn't
			 * carry them this frame. */
			{
				int ssid_p = flex_ssid_phase(ft.frame, num_phases);
				phase_params.local_id = (p == ssid_p) ? params.local_id : 0;
				phase_params.coverage_id = (p == ssid_p) ? params.coverage_id : 0;
				phase_params.country_code = (p == ssid_p) ? params.country_code : 0;
				phase_params.tmf = (p == ssid_p) ? params.tmf : 0;
			}

			/* Build flex_frame_msg_t array for this phase.
			 * Same field copying and R/N assignment as single-phase
			 * (task 4.2, Req 8.1-8.3). */
			{
				flex_frame_msg_t phase_msgs[FLEX_MAX_CANDIDATES];
				int phase_packed = 0;

				for (j = 0; j < phase_n_cands[p]; j++) {
					flex_msg_t *m = phase_cands[p][j].msg;
					flex_frame_msg_t *fm = &phase_msgs[j];
					memset(fm, 0, sizeof(*fm));

					fm->capcode = m->capcode;
					fm->msg_type = (m->msg_type == FLEX_MSG_TYPE_NUMBERED_SPECIAL)
						? FLEX_FRAME_MSG_TYPE_NUMBERED_NUM
						: (int)m->msg_type;
					fm->message = m->data;
					fm->message_length = m->data_length;
					fm->speed = m->speed;
					fm->polarity = m->polarity;
					fm->priority = m->priority;
					fm->charset = m->charset;
					fm->is_group = m->is_group;
					fm->is_temp_group = m->is_temp_group;
					fm->temp_delivery_slot = m->temp_delivery_slot;
					fm->source_id = m->source_id[0] ? m->source_id : NULL;
					fm->short_msg_type = m->short_msg_type;
					fm->short_msg_source = m->short_msg_source;
					fm->short_msg_number = m->short_msg_number;
					fm->short_msg_r = m->short_msg_r;
					fm->blocking_length = m->blocking_length;
					fm->mail_drop = m->mail_drop;
					fm->fragment_index = m->fragment_index;
					fm->total_fragments = m->total_fragments;
					fm->secure_subtype = m->secure_subtype;
					fm->secure_encoding = m->secure_encoding;
					fm->numbered_s = m->numbered_s;

					/* R/N flag assignment — independent per message */
					{
						int is_retransmission = (m->assigned_n >= 0);
						if (!is_retransmission) {
							if (m->total_fragments > 1)
								m->assigned_n = (int)m->retrieval_num;
							else
								m->assigned_n = (int)(flex->msg_sequence++ & 0x3F);
							fm->sequence_num = m->assigned_n;
							/* R=1 for normal initial TX.
							 * R=0 when msg->numbered_r is explicitly 0
							 * (system messages per spec §3.9.2). */
							fm->alpha_r_flag = m->numbered_r ? 1 : 0;
							fm->hex_r_flag = m->numbered_r ? 1 : 0;
							fm->numbered_r = m->numbered_r ? 1 : 0;
							if (m->retransmit_max > 0) {
								LOGP(DFLEX, LOGL_INFO,
								     "TX_INITIAL: capcode=%" PRIu64 " type=%s N=%d retransmit_max=%d\n",
								     m->capcode, flex_msg_type_name(m->msg_type),
								     m->assigned_n, m->retransmit_max);
							}
						} else {
							fm->sequence_num = m->assigned_n;
							fm->alpha_r_flag = 0;
							fm->hex_r_flag = 0;
							fm->numbered_r = 0;
						}
					}

					if (m->numbered_msgnum >= 0)
						fm->numbered_msgnum = m->numbered_msgnum;
					else
						fm->numbered_msgnum = fm->sequence_num;
				}

				/* Encode this phase */
				len = flex_encode_frame_multi(phase_msgs, phase_n_cands[p],
							     &phase_params, phase_buf,
							     sizeof(phase_buf),
							     &phase_packed, &error);

				if (error || len == 0) {
					/* Encoding failed for this phase — fill idle,
					 * other phases proceed normally (Req 2.2) */
					flex_fill_idle_phase(phases[p].words, p,
							     params.modulation_type,
							     params.bitrate);
					phases[p].word_count = FLEX_WORDS_PER_FRAME;
					error = 0; /* reset for next phase */
					continue;
				}

				/* Extract 88 data words from encoded output.
				 * Layout: S1(14) + FIW(4) + S2(c_bytes) + data
				 * S2 is 5 bytes × (bitrate/1600) to fill 25 ms. */
				{
					int w;
					int s2_bytes = 5 * (params.bitrate / 1600);
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

				any_msg = 1;

				/* Post-transmit for packed messages in this phase (Req 2.3).
				 * Iterate by array index — safe even if flex_post_transmit()
				 * destroys/unlinks messages from msg_list. */
				for (j = 0; j < phase_packed; j++) {
					flex_post_transmit(flex, phase_cands[p][j].msg, current_abs);
				}

				LOGP_CHAN(DFLEX, LOGL_INFO,
					  "Network mode: phase %d packed %d/%d msgs.\n",
					  p, phase_packed, phase_n_cands[p]);
			}
		}

		if (any_msg) {
			/* Fill empty phases with proper idle pattern (Section 3.4.1)
			 * — phases with no candidates were already filled above,
			 * but phases that had candidates but failed encoding were
			 * also filled. This loop catches any remaining gaps. */
			for (p = 0; p < num_phases; p++) {
				if (phases[p].word_count == 0) {
					flex_fill_idle_phase(phases[p].words, p,
							     params.modulation_type,
							     params.bitrate);
					phases[p].word_count = FLEX_WORDS_PER_FRAME;
				}
			}

			/* Encode sync → sync_buffer, data → frame_buffer.
			 * flex_encode_frame_phased produces S1+FIW+S2+DATA
			 * in one buffer, but we need the split for DSP.
			 * Use a temp buffer, then split. */
			{
				uint8_t phased_buf[FLEX_BUFFER_SIZE];
				size_t phased_len;

				phased_len = flex_encode_frame_phased(phases, num_phases,
								      &params, phased_buf,
								      sizeof(phased_buf),
								      &error);
				if (error || phased_len == 0) {
					LOGP_CHAN(DFLEX, LOGL_NOTICE,
						  "Network mode: failed to encode phased frame (error=%d), sending idle.\n", error);
					goto send_idle;
				}

				/* Split: first 18 bytes = sync, rest = data */
				memcpy(flex->sync_buffer, phased_buf, 18);
				flex->sync_buffer_length = 18;
				flex->sync_buffer_pos = 0;

				flex->frame_buffer_length = (int)(phased_len - 18);
				memcpy(flex->frame_buffer, phased_buf + 18,
				       flex->frame_buffer_length);
				flex->frame_buffer_pos = 0;
			}

			flex->sched_last_cycle = ft.cycle;
			flex->sched_last_frame = ft.frame;

			/* Set target speed for data portion, start DSP at 1600/2FSK */
			flex->frame_target_speed = params.bitrate;
			flex->frame_target_mod_type = params.modulation_type;
			dsp_set_speed(flex, 1600, FLEX_MOD_2FSK);

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "Network mode: encoded %d-phase frame C%u/F%u speed=%d/%s polarity=%s collapse=%d roaming=%d num_tx=%d sf=%d/%d.\n",
				  num_phases, ft.cycle, ft.frame, params.bitrate,
				  (params.modulation_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
				  (flex->fsk_polarity < 0) ? "neg" : "pos",
				  params.collapse, params.roaming,
				  params.num_transmissions,
				  params.subframe_index, params.num_transmissions);
			return 1;
		}
		/* No eligible messages found — fall through to idle */
		goto send_idle;
	}

	/* Single-phase encoding (1600 bps or any speed) */
	if (msg) {
		flex_frame_msg_t frame_msgs[FLEX_MAX_CANDIDATES];
		int i;

		/* Build flex_frame_msg_t array from candidates.
		 * Each message gets independent R/N flag assignment
		 * based on its own retransmission state (Req 8.1-8.3). */
		for (i = 0; i < n_candidates; i++) {
			flex_msg_t *m = candidates[i].msg;
			flex_frame_msg_t *fm = &frame_msgs[i];
			memset(fm, 0, sizeof(*fm));

			fm->capcode = m->capcode;
			fm->msg_type = (m->msg_type == FLEX_MSG_TYPE_NUMBERED_SPECIAL)
				? FLEX_FRAME_MSG_TYPE_NUMBERED_NUM
				: (int)m->msg_type;
			fm->message = m->data;
			fm->message_length = m->data_length;
			fm->speed = m->speed;
			fm->polarity = m->polarity;
			fm->priority = m->priority;
			fm->charset = m->charset;
			fm->is_group = m->is_group;
			fm->is_temp_group = m->is_temp_group;
			fm->temp_delivery_slot = m->temp_delivery_slot;
			fm->source_id = m->source_id[0] ? m->source_id : NULL;
			fm->short_msg_type = m->short_msg_type;
			fm->short_msg_source = m->short_msg_source;
			fm->short_msg_number = m->short_msg_number;
			fm->short_msg_r = m->short_msg_r;
			fm->blocking_length = m->blocking_length;
			fm->mail_drop = m->mail_drop;
			fm->fragment_index = m->fragment_index;
			fm->total_fragments = m->total_fragments;
			fm->secure_subtype = m->secure_subtype;
			fm->secure_encoding = m->secure_encoding;
			fm->numbered_s = m->numbered_s;

			/* R/N flag assignment — independent per message */
			{
				int is_retransmission = (m->assigned_n >= 0);
				if (!is_retransmission) {
					/* Initial transmission: assign fresh N, set R=1
					 * (or R=0 for system messages per §3.9.2). */
					if (m->total_fragments > 1)
						m->assigned_n = (int)m->retrieval_num;
					else
						m->assigned_n = (int)(flex->msg_sequence++ & 0x3F);
					fm->sequence_num = m->assigned_n;
					fm->alpha_r_flag = m->numbered_r ? 1 : 0;
					fm->hex_r_flag = m->numbered_r ? 1 : 0;
					fm->numbered_r = m->numbered_r ? 1 : 0;
					if (m->retransmit_max > 0) {
						LOGP(DFLEX, LOGL_INFO,
						     "TX_INITIAL: capcode=%" PRIu64 " type=%s N=%d retransmit_max=%d\n",
						     m->capcode, flex_msg_type_name(m->msg_type),
						     m->assigned_n, m->retransmit_max);
					}
				} else {
					/* Retransmission: reuse assigned N, set R=0 */
					fm->sequence_num = m->assigned_n;
					fm->alpha_r_flag = 0;
					fm->hex_r_flag = 0;
					fm->numbered_r = 0;
				}
			}

			/* Auto-assign numbered_msgnum from sequence counter when -1 */
			if (m->numbered_msgnum >= 0)
				fm->numbered_msgnum = m->numbered_msgnum;
			else
				fm->numbered_msgnum = fm->sequence_num;
		}

		/* Carry-on computation (Spec Section 3.7.1 / 4.2.1).
		 *
		 * Scan entire queue for max remaining fragment count across
		 * all eligible messages (not just packed ones — Req 9.1).
		 * Alpha-bounded per §4.2.2 Rule ② (Req 16.4, 17.1).
		 * Only meaningful with collapse > 0 (battery saving active);
		 * with collapse=0 pagers decode every frame anyway.
		 * Per spec: "Carry On is not allowed for multiple transmission." */
		if (flex->collapse > 0 && flex->num_transmissions <= 1) {
			flex_msg_t *qm;
			int max_remaining = 0;
			for (qm = flex->msg_list; qm; qm = qm->next) {
				if (qm->speed != params.bitrate ||
				    qm->modulation_type != params.modulation_type)
					continue;
				if (qm->total_fragments > 1 &&
				    qm->fragment_index < qm->total_fragments - 1) {
					int remaining = qm->total_fragments - 1 - qm->fragment_index;
					if (remaining > max_remaining)
						max_remaining = remaining;
				}
			}
			/* Alpha constraint (§4.2.2 Rule ②, Req 16.4, 17.1):
			 * carry-on cannot exceed alpha_max = 2^m - 1, since
			 * the number of continuing frames must be strictly
			 * less than the collapse cycle length. */
			{
				int alpha_max = (1 << flex->collapse) - 1;
				if (max_remaining > alpha_max)
					max_remaining = alpha_max;
			}
			if (max_remaining > 0)
				params.carry_on = (max_remaining > 3) ? 3 : max_remaining;
		}

		/* Collapse-cycle fragment placement (§4.2.2, Req 15.1, 15.4).
		 *
		 * When collapse > 0, the first fragment (fragment_index == 0)
		 * of a NEW fragmented stream must only be placed in a
		 * Collapse_Aligned_Frame for the target capcode. If the
		 * current frame is not collapse-aligned for a fragment's
		 * capcode, remove it from the candidate list (defer to next
		 * collapse-aligned frame).
		 *
		 * Continuing fragments (fragment_index > 0) are already
		 * eligible if they passed the three-pass collection — the
		 * alpha constraint is enforced via carry-on bounding (task 10).
		 *
		 * This filter runs after candidate collection but before
		 * encoding, so deferred fragments remain in the queue for
		 * the next frame cycle. */
		if (flex->collapse > 0) {
			int dst = 0;
			uint32_t alpha_max = (1U << flex->collapse) - 1;
			for (i = 0; i < n_candidates; i++) {
				flex_msg_t *m = candidates[i].msg;

				/* Only check first fragments of new streams */
				if (m->total_fragments > 1 && m->fragment_index == 0 &&
				    m->assigned_n < 0) {
					/* New fragment stream — check collapse alignment */
					flex_capcode_sched_t sched;
					flex_scheduler_capcode_info(m->capcode, &sched);

					if (!is_collapse_aligned(ft.frame,
								 sched.assigned_frame,
								 flex->collapse)) {
						LOGP(DFLEX, LOGL_DEBUG,
						     "Scheduler: defer new frag stream capcode=%"
						     PRIu64 " — frame %u not collapse-aligned "
						     "(assigned=%u collapse=%d)\n",
						     m->capcode, ft.frame,
						     sched.assigned_frame,
						     flex->collapse);
						est_used -= candidates[i].est_words;
						continue; /* remove from candidates */
					}
				}

				/* Non-continuous frame handling (§4.2.2, Req 18):
				 *
				 * Continuing fragments (fragment_index > 0) that
				 * passed the three-pass collection are eligible for
				 * this frame. However, if this frame is beyond the
				 * alpha window for the fragment's capcode, the
				 * fragment must be deferred to the next
				 * Collapse_Aligned_Frame.
				 *
				 * In the frame-at-a-time scheduler architecture,
				 * non-continuous frame handling is largely implicit:
				 * - If a continuing frame is unavailable (used for
				 *   other traffic), the fragment stays in the queue
				 *   and is re-evaluated next frame cycle
				 * - Skipped frames do not count against the alpha
				 *   limit — only frames where fragments are actually
				 *   placed consume alpha budget
				 * - The carry-on field in BIW1 (task 10) tells
				 *   pagers how many frames to stay awake; if a
				 *   continuing frame is skipped, the pager may
				 *   sleep, but the fragment will be retransmitted
				 *   at the next Collapse_Aligned_Frame
				 * - The fragment interval constraint (32/128 frames)
				 *   provides the outer bound for retransmission
				 *
				 * This check is a safety net: the three-pass
				 * collection already filters by collapse mapping,
				 * so continuing fragments outside the alpha window
				 * should not normally appear here. */
				if (m->total_fragments > 1 && m->fragment_index > 0) {
					flex_capcode_sched_t sched;
					flex_scheduler_capcode_info(m->capcode, &sched);

					if (!is_collapse_aligned(ft.frame,
								 sched.assigned_frame,
								 flex->collapse)) {
						/* Frame is not collapse-aligned — check
						 * if within alpha window */
						uint32_t period = 1U << flex->collapse;
						uint32_t aligned_frame = ft.frame - (ft.frame % period)
							+ (sched.assigned_frame % period);
						if (aligned_frame > ft.frame)
							aligned_frame -= period;

						if (ft.frame > aligned_frame + alpha_max) {
							LOGP(DFLEX, LOGL_DEBUG,
							     "Scheduler: defer continuing frag capcode=%"
							     PRIu64 " idx=%d — frame %u beyond alpha "
							     "window (aligned=%u alpha_max=%u)\n",
							     m->capcode, m->fragment_index,
							     ft.frame, aligned_frame,
							     alpha_max);
							est_used -= candidates[i].est_words;
							continue; /* defer to next Collapse_Aligned_Frame */
						}
					}
				}

				if (dst != i)
					candidates[dst] = candidates[i];
				dst++;
			}
			if (dst != n_candidates) {
				LOGP(DFLEX, LOGL_DEBUG,
				     "Scheduler: collapse filter removed %d fragment candidates\n",
				     n_candidates - dst);
				n_candidates = dst;
			}
		}

		/* Update msg pointer after collapse filtering — may have
		 * removed all candidates */
		if (n_candidates == 0) {
			goto send_idle;
		}

		/* Encode: pass full candidate array to encoder (Req 1.3) */
		flex_setup_frame_buffers(flex, &params, frame_msgs, n_candidates,
					&msgs_packed, &error);

		if (error || flex->frame_buffer_length == 0) {
			LOGP_CHAN(DFLEX, LOGL_NOTICE,
				  "Network mode: failed to encode frame (error=%d), sending idle.\n", error);
			goto send_idle;
		}

		/* Post-transmit for packed messages (Req 7.1-7.3).
		 * Iterate by array index — safe even if flex_post_transmit()
		 * destroys/unlinks messages, because candidates[i].msg
		 * pointers were captured before any post-transmit calls. */
		for (i = 0; i < msgs_packed; i++) {
			flex_msg_t *m = candidates[i].msg;

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "Network mode: packed[%d] capcode=%" PRIu64 " type=%s priority=%d seq=%d\n",
				  i, m->capcode, flex_msg_type_name(m->msg_type),
				  m->priority, m->assigned_n);

			flex_post_transmit(flex, m, current_abs);
		}

		/* Log packing summary (Req 14.1) */
		LOGP_CHAN(DFLEX, LOGL_INFO,
			  "Network mode: encoded frame C%u/F%u packed %d/%d msgs words=%d/%d speed=%d/%s num_tx=%d sf=%d/%d.\n",
			  ft.cycle, ft.frame, msgs_packed, n_candidates,
			  est_used, capacity,
			  params.bitrate,
			  (params.modulation_type == FLEX_MOD_4FSK) ? "4fsk" : "2fsk",
			  params.num_transmissions,
			  params.subframe_index, params.num_transmissions);

		flex->sched_last_cycle = ft.cycle;
		flex->sched_last_frame = ft.frame;

		return 1;
	}

send_idle:
	/* No messages — send idle frame */
	memset(&frame_msg, 0, sizeof(frame_msg));
	frame_msg.capcode = 1;
	frame_msg.msg_type = FLEX_FRAME_MSG_TYPE_TONE;
	frame_msg.message = "";
	frame_msg.message_length = 0;
	frame_msg.temp_delivery_slot = -1;
	frame_msg.speed = 1600;
	frame_msg.polarity = FLEX_DEFAULT_POLARITY;

	flex_setup_frame_buffers(flex, &params, &frame_msg, 1,
				&msgs_packed, &error);

	if (error || flex->frame_buffer_length == 0) {
		LOGP_CHAN(DFLEX, LOGL_ERROR, "Network mode: failed to encode idle frame (error=%d).\n", error);
		return 0;
	}

	flex->sched_last_cycle = ft.cycle;
	flex->sched_last_frame = ft.frame;

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
		 * ERS forces pagers to re-acquire timing.  Duration is
		 * pre-computed by flex_trigger_ers() from collapse value.
		 *
		 * Polarity is irrelevant: each ERS cycle contains both
		 * Ar and Ar_inv, so pagers of either polarity will detect
		 * the re-sync pattern. */
		{
			int ers_cycles;

			/* flex_trigger_ers() pre-computes ers_total_cycles;
			 * for legacy startup paths, fall back to override
			 * or calculate from collapse. */
			if (flex->ers_total_cycles > 0)
				ers_cycles = flex->ers_total_cycles;
			else if (flex->ers_cycles_override > 0)
				ers_cycles = flex->ers_cycles_override;
			else
				ers_cycles = flex_scheduler_ers_cycles(
					flex->collapse, 1600);

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
			flex->sync_buffer_length = 0; /* ERS has no sync portion */
			flex->sync_buffer_pos = 0;
			dsp_set_speed(flex, 1600, FLEX_MOD_2FSK);

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "ERS burst: %d cycles (%d bytes, %.1f sec at 1600 baud).\n",
				  ers_cycles, (int)len,
				  (double)ers_cycles * 96.0 / 1600.0);

			/* Reset ERS state and move to message transmission */
			flex->ers_total_cycles = 0;
			flex->ers_sent_cycles = 0;
			flex_new_state(flex, FLEX_STATE_MESSAGE);
			flex->idle_count = 0;
			return 1;
		}


	case FLEX_STATE_MESSAGE:
		/* Fragment any oversized messages before selection */
		flex_fragment_queue(flex);

		msg = flex->msg_list;
		if (msg) {
			flex_frame_msg_t frame_msg;
			flex_frame_params_t params;
			int msgs_packed = 0;
			double polarity;
			int is_inverted_pass = (flex->idle_count == 1);

			/* One-shot mode: transmit twice — once with default
			 * polarity, once with inverted polarity, then exit.
			 * idle_count tracks which pass:
			 *   0 = first TX (default polarity)
			 *   1 = second TX (inverted polarity) */

			if (is_inverted_pass) {
				polarity = (msg->polarity < 0) ? 1.0 : -1.0;
			} else {
				polarity = msg->polarity;
			}
			dsp_set_polarity(flex, polarity);

			/* Build frame message descriptor */
			memset(&frame_msg, 0, sizeof(frame_msg));
			frame_msg.capcode = msg->capcode;
			frame_msg.msg_type = (msg->msg_type == FLEX_MSG_TYPE_NUMBERED_SPECIAL)
				? FLEX_FRAME_MSG_TYPE_NUMBERED_NUM
				: (int)msg->msg_type;
			frame_msg.message = msg->data;
			frame_msg.message_length = msg->data_length;
			frame_msg.speed = msg->speed;
			frame_msg.polarity = polarity;
			frame_msg.priority = msg->priority;
			frame_msg.charset = msg->charset;
			frame_msg.is_group = msg->is_group;
			frame_msg.is_temp_group = msg->is_temp_group;
			frame_msg.temp_delivery_slot = msg->temp_delivery_slot;
			frame_msg.sequence_num = (msg->total_fragments > 1)
				? (int)msg->retrieval_num
				: (int)(flex->msg_sequence & 0x3F);
			frame_msg.phase = msg->phase;
			frame_msg.blocking_length = msg->blocking_length;
			frame_msg.mail_drop = msg->mail_drop;
			frame_msg.fragment_index = msg->fragment_index;
			frame_msg.total_fragments = msg->total_fragments;
			frame_msg.short_msg_type = msg->short_msg_type;
			frame_msg.short_msg_source = msg->short_msg_source;
			frame_msg.short_msg_number = msg->short_msg_number;
			frame_msg.short_msg_r = msg->short_msg_r;
			frame_msg.numbered_s = msg->numbered_s;
			frame_msg.numbered_msgnum = msg->numbered_msgnum;
			/* R flag: R=1 for normal, R=0 for system msgs (§3.9.2) */
			frame_msg.alpha_r_flag = msg->numbered_r ? 1 : 0;
			frame_msg.hex_r_flag = msg->numbered_r ? 1 : 0;
			frame_msg.numbered_r = msg->numbered_r ? 1 : 0;

			/* Frame params: always cycle=0, frame=0.
			 * Include BIW time + timezone when configured. */
			flex_frame_params_default(&params);
			params.cycle = 0;
			params.frame = 0;
			params.bitrate = msg->speed;
			params.modulation_type = msg->modulation_type;
			params.biw_time = flex->biw_time_enabled;
			params.chan_setup_enabled = flex->chan_setup_enabled;
			params.hack_nonstandard_decoders = flex->hack_nonstandard_decoders;
			params.collapse = flex->collapse;
			if (flex->ssid || flex->nid) {
				params.local_id = flex->ssid;
				params.coverage_id = flex->nid;
			}
			if (flex->country_code || flex->tmf) {
				params.country_code = flex->country_code;
				params.tmf = flex->tmf;
			}
			params.timezone_code = flex->timezone_code;

			flex_setup_frame_buffers(flex, &params, &frame_msg, 1,
						&msgs_packed, &error);

			if (error || flex->frame_buffer_length == 0) {
				LOGP_CHAN(DFLEX, LOGL_NOTICE,
					  "Failed to encode FLEX frame (error=%d).\n", error);
				flex_msg_destroy(msg);
				goto oneshot_done;
			}

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "One-shot: C0/F0 %s polarity=%s capcode=%" PRIu64
				  " type=%d speed=%d len=%d.\n",
				  is_inverted_pass ? "(inverted)" : "(default)",
				  (polarity < 0) ? "neg" : "pos",
				  msg->capcode, (int)msg->msg_type,
				  msg->speed, msg->data_length);

			if (is_inverted_pass) {
				/* Second pass done — destroy message.
				 * Set idle_count=2 so the next call falls
				 * through to oneshot_done after the DSP
				 * finishes outputting this frame. */
				flex_msg_destroy(msg);
				flex->idle_count = 2;
			} else {
				/* First pass done — mark for inverted pass */
				flex->idle_count = 1;
				/* Bump sequence so inverted pass uses same N */
				flex->msg_sequence++;
			}
			return 1;
		}

oneshot_done:
		LOGP_CHAN(DFLEX, LOGL_INFO, "One-shot: transmission complete.\n");
		flex_new_state(flex, FLEX_STATE_IDLE);
		/* Delayed quit: let RX finish decoding the last frame.
		 * quit_after_time is checked in myhandler() every loop
		 * iteration (~1ms).  Set it to now + 250ms. */
		{
			extern double quit_after_time;
			extern double get_time(void);
			quit_after_time = get_time() + 0.25;
		}
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

	/* Enable RX path — always on so we can monitor our own transmission
	 * (loopback) or receive off-air FLEX signals for debugging.
	 * State starts at 0 (== RX_STATE_SYNC1) from calloc. */
	flex->rx.enabled = 1;
	flex->rx.sample_freq = samplerate;
	flex->rx.baud = 1600;

	flex_display_status();

	LOGP(DFLEX, LOGL_NOTICE, "Created 'Kanal' %s: samplerate=%d deviation=%.0f polarity=%s tx=%d.\n",
	     kanal, samplerate, deviation, (polarity < 0) ? "neg" : "pos", tx);

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
		const char *msg_text;
		int msg_len;

		/* Use CLI -M message if provided, otherwise generate a default */
		if (flex->default_message && flex->default_message[0]) {
			msg_text = flex->default_message;
			msg_len = strlen(msg_text);
		} else {
			static char autobuf[16];
			switch (flex->default_msg_type) {
			case FLEX_MSG_TYPE_NUMERIC:
				sprintf(autobuf, "%05d", (int)(flex->scan_from / 100));
				break;
			case FLEX_MSG_TYPE_ALPHA:
				sprintf(autobuf, "%02x", (int)(flex->scan_from / 10000));
				break;
			case FLEX_MSG_TYPE_TONE:
			case FLEX_MSG_TYPE_AUTO:
			default:
				autobuf[0] = '\0';
			}
			msg_text = autobuf;
			msg_len = strlen(autobuf);
		}
		LOGP_CHAN(DFLEX, LOGL_NOTICE, "Transmitting %s message '%s' with capcode '%" PRIu64 "' (%s).\n",
			  flex_msg_type_name(flex->default_msg_type), msg_text, flex->scan_from,
			  flex_capcode_type_name(flex->scan_from));
		{
			flex_msg_t *msg;
			msg = flex_msg_create(flex, flex->scan_from, flex->default_msg_type,
					      msg_text, msg_len);
			if (msg) {
				/* Apply fixed-mode speed/modulation if set */
				if (flex->fixed_speed != -1) {
					msg->speed = flex->fixed_speed;
					msg->modulation_type = flex->fixed_mod_type;
				}
				/* Apply fixed-mode polarity if set */
				if (flex->fixed_polarity != 0.0)
					msg->polarity = flex->fixed_polarity;
				/* Apply default phase if set */
				if (flex->default_phase >= 0)
					msg->phase = flex->default_phase;
			}
		}
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
 * Accepts both plain numeric capcodes and Extended CAPCODE format
 * per ARIB STD-43A Appendix A:
 *   Plain:    1234567 (short) or 007005031 (long)
 *   Extended: [R][fff][b]<A-Z><digits>  e.g. 3E007005031, A1234567, 5U0000100
 *
 * For Extended CAPCODEs, the alpha prefix and optional frame/collapse/roaming
 * metadata are stripped — only the trailing numeric address is validated.
 *
 * Returns NULL if valid, or an error string if invalid.
 */
const char *flex_number_valid(const char *number)
{
	uint64_t capcode;
	int i;
	int has_alpha = 0;
	const char *alpha_pos = NULL;
	const char *digits;

	if (!number || !*number)
		return "Empty capcode string";

	/* Check if this is an Extended CAPCODE (contains alpha characters) */
	for (i = 0; number[i]; i++) {
		char ch = number[i];
		if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z')) {
			has_alpha = 1;
			alpha_pos = &number[i];
		}
	}

	if (!has_alpha) {
		/* Plain numeric — all characters must be digits */
		for (i = 0; number[i]; i++) {
			if (number[i] < '0' || number[i] > '9')
				return "Illegal capcode digit (use 0..9 only)";
		}
		digits = number;
	} else {
		/* Extended CAPCODE — validate structure:
		 * Everything after the last alpha must be digits (the address).
		 * Everything before it must be valid prefix chars. */
		if (!alpha_pos || !*(alpha_pos + 1))
			return "Extended CAPCODE: alpha prefix must be followed by digits";

		/* Validate prefix characters before alpha */
		{
			const char *c;
			for (c = number; c < alpha_pos; c++) {
				char ch = *c;
				int is_digit = (ch >= '0' && ch <= '9');
				int is_roaming = (ch == 'P' || ch == 'p' ||
						  ch == 'Q' || ch == 'q' ||
						  ch == 'R' || ch == 'r' ||
						  ch == 'S' || ch == 's');
				if (!is_digit && !is_roaming)
					return "Extended CAPCODE: invalid prefix (expect [P|Q|R|S][frame][collapse])";
			}
		}

		/* Validate primary alpha is A-Z */
		{
			char a = *alpha_pos;
			if (a >= 'a' && a <= 'z')
				a = a - 'a' + 'A';
			if (a < 'A' || a > 'Z')
				return "Extended CAPCODE: invalid alpha prefix (expect A-Z)";
		}

		/* Validate trailing digits */
		{
			const char *d;
			for (d = alpha_pos + 1; *d; d++) {
				if (*d < '0' || *d > '9')
					return "Extended CAPCODE: address must be numeric digits after alpha prefix";
			}
		}

		digits = alpha_pos + 1;
	}

	capcode = strtoull(digits, NULL, 10);
	if (!flex_capcode_valid(capcode)) {
		static char errbuf[128];
		snprintf(errbuf, sizeof(errbuf),
			 "Invalid FLEX capcode (short: %" PRIu64 "-%" PRIu64
			 ", long: %" PRIu64 "-%" PRIu64 ")",
			 (uint64_t)FLEX_SHORT_ADDR_MIN, (uint64_t)FLEX_SHORT_ADDR_MAX,
			 (uint64_t)FLEX_LONG_ADDR_MIN, (uint64_t)FLEX_LONG_ADDR_MAX);
		return errbuf;
	}

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

	/* parse capcode and create message.
	 * Supports both plain numeric and Extended CAPCODE format
	 * (e.g. "3E007005031" → address=007005031). */
	{
		uint64_t capcode;
		const char *dp = dialing;
		const char *alpha_pos = NULL;
		int i;

		/* Find last alpha character — if present, digits after it
		 * are the numeric address (Extended CAPCODE format). */
		for (i = 0; dp[i]; i++) {
			char ch = dp[i];
			if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'))
				alpha_pos = &dp[i];
		}

		if (alpha_pos && *(alpha_pos + 1))
			capcode = strtoull(alpha_pos + 1, NULL, 10);
		else
			capcode = strtoull(dialing, NULL, 10);

		LOGP(DFLEX, LOGL_INFO, "Paging capcode '%" PRIu64 "' (%s) (dialed '%s') with %s message '%s'.\n",
		     capcode, flex_capcode_type_name(capcode), dialing, flex_msg_type_name(flex->default_msg_type), message);

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
