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
#include "n_counter.h"
#include "biw_carousel.h"

#define FLEX_MAX_QUEUE_DEPTH 256

/* Map polarity double (+1.0/-1.0) to index (0=normal, 1=inverted). */
static inline int pol_index(double polarity)
{
	return (polarity < 0) ? FLEX_POL_INVERTED : FLEX_POL_NORMAL;
}

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

	/* Queue overflow protection: check per-polarity depth limit.
	 * Use the per-polarity msg_count for O(1) check.
	 * At this point the message's polarity isn't set yet — use
	 * the default polarity.  The actual polarity may be overridden
	 * later by FIFO options, but the default is the best estimate. */
	{
		int pi = pol_index(flex->default_polarity ? flex->default_polarity : FLEX_DEFAULT_POLARITY);
		if (flex->tx_pol[pi].msg_count >= FLEX_MSG_QUEUE_MAX) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "Queue overflow: polarity=%s depth=%d, rejecting capcode=%" PRIu64 "\n",
			     (pi == FLEX_POL_INVERTED) ? "inverted" : "normal",
			     flex->tx_pol[pi].msg_count, capcode);
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

	/* Apply fixed-mode polarity if set (CLI --fixed-polarity).
	 * This overrides the default and any per-message setting. */
	if (flex->fixed_polarity != 0.0)
		msg->polarity = flex->fixed_polarity;
	msg->priority = 0;
	msg->charset = 0;
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

	/* link to tail of list (legacy global queue) */
	msg->flex = flex;
	msgp = &flex->msg_list;
	while ((*msgp))
		msgp = &(*msgp)->next;
	(*msgp) = msg;

	/* Also link to per-polarity queue (new scheduler path).
	 * Both lists share the same flex_msg_t nodes — the per-polarity
	 * list uses a separate linkage via the same ->next pointer,
	 * so for now we just track the count.  Full migration to
	 * per-polarity-only lists happens in task 6+. */
	{
		int pi = pol_index(msg->polarity);
		flex->tx_pol[pi].msg_count++;
	}

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

	/* Log polarity assignment at DEBUG level (Req 11.3) */
	LOGP(DFLEX, LOGL_DEBUG,
	     "Enqueue: capcode=%" PRIu64 " polarity=%s queue_depth=%d\n",
	     capcode,
	     (msg->polarity < 0) ? "inverted" : "normal",
	     flex->tx_pol[pol_index(msg->polarity)].msg_count);

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
	int pi;

	/* decrement per-polarity count */
	pi = pol_index(msg->polarity);
	if (msg->flex->tx_pol[pi].msg_count > 0)
		msg->flex->tx_pol[pi].msg_count--;

	/* unlink from global list */
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

/* Frames of lead time before first SETUP (queue processing headroom) */
#define FLEX_TG_QUEUE_LEAD	2

/* Allocate a free temp slot for a given polarity. Returns 0-15, or -1 if all occupied. */
static int flex_tg_alloc_slot(flex_t *flex, int pol)
{
	int i;
	for (i = 0; i < FLEX_TEMP_ADDR_SLOTS; i++) {
		if (flex->tx_pol[pol].tx_temp[i].state == FLEX_TG_FREE)
			return i;
	}
	return -1;
}

/* Free a temp slot after delivery completes. */
static void flex_tg_free_slot(flex_t *flex, int pol, int slot)
{
	LOGP(DFLEX, LOGL_INFO,
	     "TX: TEARDOWN temp slot=%d pol=%s — delivery complete.\n",
	     slot, (pol == FLEX_POL_INVERTED) ? "inverted" : "normal");
	memset(&flex->tx_pol[pol].tx_temp[slot], 0, sizeof(flex->tx_pol[pol].tx_temp[slot]));
}

/*
 * Enqueue a temporary group message (§5.2).
 *
 * Pre-computes the exact frame for every SETUP and the DELIVERY,
 * then pins each message to its frame via next_send_frame.
 *
 * Schedule:
 *   - Each SETUP is pinned to the next collapse-aligned frame for
 *     that pager's capcode (deterministic from capcode + collapse).
 *   - Frame N (DELIVERY) = last SETUP frame + margin, mod 128.
 *   - N must be ≤128 frames from the first SETUP (§5.2).
 *   - DELIVERY is pinned to the absolute frame corresponding to N.
 *
 * The scheduler honours next_send_frame: messages aren't eligible
 * until that frame.  With priority=1 on SETUPs, they'll go out
 * immediately when their frame arrives.
 *
 * Returns 0 on success, -1 on failure.
 */
int flex_tempgroup_enqueue(flex_t *flex, const uint64_t *capcodes,
			   const int *collapses, int count,
			   enum flex_msg_type msg_type, const char *data,
			   int data_length, int speed, int modulation_type,
			   double polarity, int priority, int phase)
{
	int slot, i, pol;
	uint32_t target_frame;
	flex_frame_time_t ft;
	uint32_t abs_frame, current_frame;
	/* Per-pager: absolute frame when SETUP should be sent */
	uint32_t setup_abs[FLEX_TEMP_GROUP_MAX_MEMBERS];
	uint32_t first_setup_abs, last_setup_abs, delivery_abs;
	int sys_collapse = flex->collapse;

	pol = pol_index(polarity);

	if (count <= 0 || count > FLEX_TEMP_GROUP_MAX_MEMBERS) {
		LOGP(DFLEX, LOGL_NOTICE,
		     "FIFO: tempgroup requires 1-%d capcodes, got %d.\n",
		     FLEX_TEMP_GROUP_MAX_MEMBERS, count);
		return -1;
	}

	for (i = 0; i < count; i++) {
		if (!flex_capcode_valid(capcodes[i])) {
			LOGP(DFLEX, LOGL_NOTICE,
			     "FIFO: tempgroup capcode %" PRIu64 " invalid.\n",
			     capcodes[i]);
			return -1;
		}
	}

	slot = flex_tg_alloc_slot(flex, pol);
	if (slot < 0) {
		LOGP(DFLEX, LOGL_NOTICE,
		     "FIFO: tempgroup — no free slot (all 16 occupied on %s polarity), rejecting.\n",
		     (pol == FLEX_POL_INVERTED) ? "inverted" : "normal");
		return -1;
	}

	if (flex_scheduler_get_time(flex, &ft) < 0) {
		abs_frame = 0;
	} else {
		abs_frame = ft.cycle * 128 + ft.frame;
	}
	current_frame = abs_frame % 128;

	/* --- Compute exact frame for each SETUP --- */
	first_setup_abs = UINT32_MAX;
	last_setup_abs = 0;

	for (i = 0; i < count; i++) {
		flex_capcode_sched_t sched;
		int m;
		uint32_t next_f, delta;

		flex_scheduler_capcode_info(capcodes[i], &sched);
		m = (collapses && collapses[i] >= 0) ? collapses[i] : sys_collapse;

		/* Next collapse-aligned frame number (may be > 127).
		 * Start from current + lead time so the message is
		 * safely queued before the scheduler reaches that frame. */
		next_f = flex_scheduler_next_frame(current_frame + FLEX_TG_QUEUE_LEAD,
						   sched.assigned_frame, m);

		/* Convert to absolute frame */
		delta = next_f - current_frame;
		setup_abs[i] = abs_frame + delta;

		if (setup_abs[i] < first_setup_abs)
			first_setup_abs = setup_abs[i];
		if (setup_abs[i] > last_setup_abs)
			last_setup_abs = setup_abs[i];

		LOGP(DFLEX, LOGL_INFO,
		     "TX: tempgroup SETUP[%d] cap=%" PRIu64 " collapse=%d "
		     "assigned_frame=%u pinned_frame=%u (abs=%u)\n",
		     i, capcodes[i], m, sched.assigned_frame,
		     setup_abs[i] % 128, setup_abs[i]);
	}

	/* --- Compute frame N (DELIVERY) --- */
	delivery_abs = last_setup_abs + FLEX_TG_SETUP_MARGIN;
	target_frame = delivery_abs % 128;

	/* §5.2: first transmission must start within 128 frames of first SETUP */
	if (delivery_abs - first_setup_abs > 127) {
		LOGP(DFLEX, LOGL_NOTICE,
		     "FIFO: tempgroup DELIVERY frame %u exceeds 128-frame limit "
		     "from first SETUP (span=%u).\n",
		     target_frame, delivery_abs - first_setup_abs);
		flex_tg_free_slot(flex, pol, slot);
		return -1;
	}

	LOGP(DFLEX, LOGL_INFO,
	     "TX: tempgroup schedule: first_setup=%u last_setup=%u "
	     "delivery(N)=%u (abs=%u) slot=%d\n",
	     first_setup_abs % 128, last_setup_abs % 128,
	     target_frame, delivery_abs, slot);

	/* --- Populate slot --- */
	flex->tx_pol[pol].tx_temp[slot].state = FLEX_TG_SETUP;
	flex->tx_pol[pol].tx_temp[slot].count = count;
	flex->tx_pol[pol].tx_temp[slot].setups_sent = 0;
	flex->tx_pol[pol].tx_temp[slot].target_frame = target_frame;
	flex->tx_pol[pol].tx_temp[slot].setup_abs = abs_frame;
	for (i = 0; i < count; i++)
		flex->tx_pol[pol].tx_temp[slot].capcodes[i] = capcodes[i];

	/* --- Create SETUP messages, each pinned to its frame --- */
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
			flex_tg_free_slot(flex, pol, slot);
			return -1;
		}
		msg->speed = speed;
		msg->modulation_type = modulation_type;
		msg->polarity = polarity;
		msg->priority = 1;
		if (phase >= 0)
			msg->phase = phase;

		/* Pin to exact frame */
		msg->next_send_frame = setup_abs[i] % 1920;

		LOGP(DFLEX, LOGL_INFO,
		     "TX: SETUP slot=%d N=%u cap=%" PRIu64 " (%d/%d) "
		     "idata=0x%04X pinned_abs=%u\n",
		     slot, target_frame, capcodes[i], i + 1, count,
		     idata, setup_abs[i]);
	}

	/* --- Create DELIVERY message, pinned to frame N --- */
	{
		flex_msg_t *msg;

		msg = flex_msg_create(flex, 1, msg_type, data, data_length);
		if (!msg) {
			LOGP(DFLEX, LOGL_ERROR,
			     "TX: tempgroup DELIVERY creation failed for slot=%d.\n",
			     slot);
			flex_tg_free_slot(flex, pol, slot);
			return -1;
		}
		msg->speed = speed;
		msg->modulation_type = modulation_type;
		msg->polarity = polarity;
		msg->priority = priority;
		msg->temp_delivery_slot = slot;
		if (phase >= 0)
			msg->phase = phase;

		/* Pin to frame N */
		msg->next_send_frame = delivery_abs % 1920;
		msg->send_delay = (int)(delivery_abs - abs_frame);

		flex->tx_pol[pol].tx_temp[slot].delivery_msg = msg;

		LOGP(DFLEX, LOGL_INFO,
		     "TX: DELIVERY queued slot=%d N=%u type=%s members=%d "
		     "len=%d pinned_abs=%u\n",
		     slot, target_frame, flex_msg_type_name(msg_type),
		     count, data_length, delivery_abs);
	}

	return 0;
}

/*
 * Tick the temp group scheduler — called each frame.
 * Checks for completed deliveries and frees slots.
 */
void flex_tempgroup_tick(flex_t *flex, uint32_t abs_frame)
{
	int pol, i;

	for (pol = 0; pol < FLEX_TX_POLARITIES; pol++) {
		for (i = 0; i < FLEX_TEMP_ADDR_SLOTS; i++) {
			if (flex->tx_pol[pol].tx_temp[i].state == FLEX_TG_FREE)
				continue;

			/* Check if delivery message has been consumed (removed from queue) */
			if (flex->tx_pol[pol].tx_temp[i].delivery_msg) {
				flex_msg_t *m;
				int found = 0;
				for (m = flex->msg_list; m; m = m->next) {
					if (m == flex->tx_pol[pol].tx_temp[i].delivery_msg) {
						found = 1;
						break;
					}
				}
				if (!found) {
					/* Delivery consumed — teardown */
					flex_tg_free_slot(flex, pol, i);
					continue;
				}
			}

			/* 128-frame timeout from SETUP */
			{
				uint32_t elapsed = abs_frame - flex->tx_pol[pol].tx_temp[i].setup_abs;
				if (elapsed > 256) /* handle wraparound conservatively */
					elapsed = 256;
				if (elapsed >= 128) {
					LOGP(DFLEX, LOGL_NOTICE,
					     "TX: temp slot=%d pol=%s TIMEOUT — %u frames since SETUP, clearing.\n",
					     i, (pol == FLEX_POL_INVERTED) ? "inverted" : "normal", elapsed);
					/* Remove delivery msg from queue if still there */
					if (flex->tx_pol[pol].tx_temp[i].delivery_msg) {
						flex_msg_t *m;
						for (m = flex->msg_list; m; m = m->next) {
							if (m == flex->tx_pol[pol].tx_temp[i].delivery_msg) {
								flex_msg_destroy(m);
								break;
							}
						}
					}
					memset(&flex->tx_pol[pol].tx_temp[i], 0, sizeof(flex->tx_pol[pol].tx_temp[i]));
				}
			}
		}
	}
}

/*
 * Check if a message needs fragmentation and split it if so (§4.1, §4.2).
 *
 * Per §4.1: when the full length of a message cannot be contained in one
 * Frame, the individual message must be broken down into fragments.  The
 * respective fragments need to be transmitted over several Frames in order
 * for the complete message to be transmitted as one message.
 *
 * Walks the message queue and splits any message that exceeds the
 * single-frame capacity for its type.  The original message is replaced
 * with fragment messages, each carrying the same retrieval number (N).
 *
 * Fragment scheduling constraints (§4.2):
 *   ① Once an address is used to begin transmitting a fragmented message,
 *     that same address must not be used to start a new fragmented
 *     transmission until the first fragmented transmission has been
 *     completed.  (Enforced by frag_excluded() at scheduling time.)
 *   ② For the duration that an address is being used to send a fragmented
 *     message, that same address must not appear more than once in any
 *     Frame to send an unfragmented message.  (Enforced by frag_excluded().)
 *   ③ Numeric Messages (V=011, V=100, V=111) cannot be fragmented.
 *   ④ The transmission interval between each fragment of the same message
 *     must be 32 Frames or less (FLEX_FRAG_INTERVAL_SINGLE).  When the
 *     channel is shared with another system or uses multiple transmission /
 *     Multi-area/Roaming, the interval can be 128 Frames
 *     (FLEX_FRAG_INTERVAL_SHARED).  (Enforced by frag_excluded().)
 *
 * Each fragment carries:
 *   F: 2-bit Fragment Number — modulo 3 sequence (11, 00, 01, 10, 00, ...).
 *      "11" is skipped after the first fragment.
 *   C: Message Continued Flag — 1 = more fragments follow; the last
 *      fragment resets C to 0.
 *   N: 6-bit Message Number (0-63) — same across all fragments.
 *
 * The first fragment (index 0) is immediately eligible for transmission;
 * subsequent fragments are held ineligible until the preceding fragment's
 * post-transmit handler releases them.  The last fragment triggers
 * retransmission cycling or stream destruction.
 *
 * Carry-on (§3.7.1, §4.2.1): when fragments span multiple frames, the
 * scheduler sets the BIW1 carry-on field (0-3) to indicate how many
 * additional frames the pager should continue decoding beyond its
 * assigned collapse frame.  Carry-on is not allowed for multiple
 * transmission (num_transmissions > 1).
 *
 * Polarity: retrieval numbers (N) are assigned per-polarity via
 * frag_retrieval_seq to avoid N collisions across polarities.
 *
 * Phase: fragments inherit the phase assignment from the original message
 * (capcode-based or explicit override).  The scheduler's phase assignment
 * ensures all fragments of a message appear on the same phase.
 *
 * Collapse: the first fragment respects the collapse schedule — it is
 * only eligible on frames matching the capcode's assigned collapse frame.
 * Continuation fragments (index > 0) also respect the collapse schedule
 * by default, but when carry-on is set (§4.2.1), they are allowed on
 * frames within the carry-on window beyond the collapse frame.  This
 * ensures the pager (which is told to keep decoding via BIW1 carry-on)
 * can receive the continuation fragments without waiting for the next
 * collapse-aligned frame.
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

		/* Only fragment alpha, hex, and secure messages.
		 * Per §4.2 ③: Numeric Messages (V=011, V=100, V=111)
		 * cannot be fragmented. */
		switch (msg->msg_type) {
		case FLEX_MSG_TYPE_ALPHA: {
			/* Per §4.1: available message words = 88 - biw - addr - vector.
			 * Short addr = 1 word, long = 2.  Vector = 1 word.
			 * Alpha: chars = (msg_words - 1) * 3 - 2
			 *   (1 header word, signature eats 1 char slot,
			 *    ETX padding eats 1 more).
			 * For a Short Address with 1 BIW: 88-1-1-1 = 85 msg words,
			 * 84 data words × 3 chars - 1 = 251 chars max (per §4.1). */
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
			/* Per §4.2 ③: Numeric Messages (those having a vector
			 * type of 011, 100, 111) cannot be fragmented.
			 * Standard numeric, special format, and numbered
			 * numeric are single-frame only. */
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

		/* Assign a retrieval number (N) for this set of fragments.
		 * Per §4.2: N identifies all fragments of the same message.
		 * Use the per-polarity counter to avoid N collisions across
		 * polarities (normal vs inverted). */
		int frag_pol = pol_index(msg->polarity);
		uint32_t ret_num = flex->tx_pol[frag_pol].frag_retrieval_seq++;
		flex->tx_pol[frag_pol].frag_retrieval_seq &= 0x3F; /* 6-bit counter: N is 0-63 per spec */

		LOGP(DFLEX, LOGL_INFO,
		     "Fragmenting message for capcode %" PRIu64 " into %d fragments (retrieval=%u).\n",
		     msg->capcode, frag_count, ret_num);

		/* Create fragment messages and insert them in place of the original.
		 * We insert after the original, then destroy the original.
		 *
		 * Each fragment inherits the original message's polarity, phase,
		 * speed, and modulation type — ensuring all fragments of a
		 * message are transmitted on the same polarity/phase/speed. */
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
			 * eligible for transmission — it is the special case
			 * that starts the fragment stream.  Non-first fragments
			 * become eligible only when the preceding fragment's
			 * post-transmit handler sets their next_send_frame to
			 * the current frame (immediate chaining per §4.2 ④).
			 *
			 * Place non-first fragments 959 frames ahead (half the
			 * 1920-frame hour minus 1) so frame_is_eligible()
			 * rejects them until explicitly released.
			 *
			 * The first fragment respects the capcode's collapse
			 * schedule — it will only be picked for frames matching
			 * the capcode's assigned collapse frame. */
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
 * Fragments of the same message share the same retrieval_num (N) and are
 * linked as consecutive flex_msg_t nodes in the queue with incrementing
 * fragment_index (0 through total_fragments-1).
 *
 * All fragments inherit the same polarity, phase, speed, and modulation
 * type from the original message, ensuring the entire fragment stream
 * is transmitted consistently on the same channel parameters.
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
 *
 * - First fragment (index 0): special case — it is the only fragment
 *   initially eligible for transmission.  After it is sent, the next
 *   fragment (index 1) becomes immediately eligible via next_send_frame.
 *   The first fragment also holds the authoritative retransmit_count.
 *
 * - Non-last fragments (index 1..N-2): after transmission, make the
 *   next fragment eligible immediately (next_send_frame = current).
 *   If retransmission is configured, keep the fragment but mark it
 *   ineligible until the retransmission cycle restarts from fragment 0.
 *
 * - Last fragment (index N-1): special case — completing the last
 *   fragment finishes one full pass of the fragment stream.  If
 *   retransmissions remain, increment retransmit_count on the first
 *   fragment, schedule its next_send_frame for the retransmission
 *   interval, and mark all non-first fragments ineligible.  If done,
 *   destroy all fragments.
 *
 * Per §4.2 ④: the transmission interval between consecutive fragments
 * must not exceed 32 Frames (or 128 when channel is shared).  The
 * immediate-eligibility chaining (next_send_frame = current_abs)
 * ensures fragments go out in consecutive eligible frames, well within
 * the interval limit.
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
	/* BIW101 for system message (method (b), §3.9.2).
	 * When an Operator Messaging Address with LSB 0-3 is present,
	 * emit BIW101 with the matching A-type.  This replaces the
	 * timezone BIW101 slot if both are requested (only one type 101
	 * word per phase per spec). */
	if (params->sysmsg_a_type >= 0 && params->sysmsg_a_type <= 3) {
		if (params->timezone_code < 0 ||
		    params->timezone_code >= (int)FLEX_TZ_ENTRIES)
			extra++; /* no timezone slot yet, add one */
		/* else: reuse the timezone slot for sysmsg A-type */
	}
	if (params->chan_setup_enabled && params->frame <= 3)
		extra++;
	if (extra > 3)
		extra = 3;

	return 1 + extra;
}

/* Compute available message words for a specific phase.
 *
 * Builds the exact params that the encoder will see for this phase
 * (SSID/timezone/sysmsg rotate across phases per §6.1.1.3), then
 * computes BIW count via flex_compute_biw_count() — the same function
 * the encoder uses — ensuring scheduler and encoder agree exactly.
 *
 * phase_idx: 0-based phase index
 * num_phases: total phases (1, 2, or 4)
 * params: frame-level params (with all BIW fields populated)
 *
 * Returns available words for message content (addr + vec + body). */
static int flex_phase_capacity(const flex_frame_params_t *params,
			       int phase_idx, int num_phases)
{
	flex_frame_params_t pp = *params;
	int ssid_phase = flex_ssid_phase(params->frame, num_phases);
	int biw_count;
	int cap;

	/* SSID1/SSID2 only in the designated phase (§6.1.1.3) */
	if (phase_idx != ssid_phase) {
		pp.local_id = 0;
		pp.coverage_id = 0;
		pp.country_code = 0;
		pp.tmf = 0;
	}

	/* SysInfo BIW101 — only one per phase per spec.
	 * System message BIW101 (A=0000~0011) follows SSID rotation.
	 * Timezone BIW101 (A=0100) also follows SSID rotation. */
	if (phase_idx != ssid_phase) {
		pp.sysmsg_a_type = -1;
		pp.timezone_code = -1;
	}

	/* Channel setup BIW only in SSID phase */
	if (phase_idx != ssid_phase)
		pp.chan_setup_enabled = 0;

	/* Date/Time BIW rotation across phases (§6.1.1.3 Note 1).
	 * TODO: implement proper T1/T2/T3 rotation for multi-phase.
	 * Currently all phases emit date+time — conservative (over-counts
	 * BIW, under-counts capacity) but safe. */

	biw_count = flex_compute_biw_count(&pp);
	cap = FLEX_WORDS_PER_FRAME - biw_count;

	/* Reserve 1 word for system message vector at end of VF
	 * (method (b), §3.9.2) — only in the phase carrying BIW101 */
	if (phase_idx == ssid_phase &&
	    pp.sysmsg_a_type >= 0 && pp.sysmsg_a_type <= 3)
		cap--;

	return cap;
}

/* Estimate total word cost for a queued message (address + vector + body).
 * Mirrors estimate_msg_words() logic in frame.c but operates on flex_msg_t
 * directly (queue entry struct) rather than flex_frame_msg_t (encoder input).
 * Returns total words needed, or -1 if the message is invalid.
 *
 * Address/vector overhead:
 *   Short address (capcode ≤ 1,933,312): 1 addr + 1 vector word
 *   Long address  (capcode ≥ 2,101,249): 2 addr + 2 vector words
 *   Temp address delivery: 1 addr + 1 vector word
 *   Tone-only: address words only (no vector, no body)
 *
 * For long addresses, body word count is reduced by 1 because body[0]
 * is absorbed into the second vector word (Vy). */
static int flex_estimate_msg_cost(const flex_msg_t *msg)
{
	int is_long = (msg->capcode >= FLEX_LONG_ADDR_MIN
		       && msg->temp_delivery_slot < 0);
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
 * Now defined in flex.h as flex_inflight_frag_t. */

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
 * This is used to enforce the §4.2 address exclusion rules: while a
 * fragmented stream is in-flight for a capcode, no new fragmented or
 * unfragmented message for that same capcode may be transmitted (except
 * the next continuation fragment of the in-flight stream itself).
 *
 * Also tracks last_sent_abs for §4.2 ④ interval checking (32/128 frames).
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

		/* Search for the last fragment and track highest sent index.
		 * Also find the most recently sent fragment to get its
		 * transmission frame for interval checking. */
		last = NULL;
		highest_sent_idx = 0;
		uint32_t latest_sent_abs = 0;
		for (f = flex->msg_list; f; f = f->next) {
			if (f->total_fragments != m->total_fragments)
				continue;
			if (f->retrieval_num != m->retrieval_num)
				continue;
			if (f->assigned_n >= 0 && f->fragment_index > highest_sent_idx) {
				highest_sent_idx = f->fragment_index;
				/* next_send_frame of a just-sent fragment is
				 * set to current_abs by post_transmit, so it
				 * approximates the frame it was sent in. */
				if (f->next_send_frame > 0)
					latest_sent_abs = f->next_send_frame;
			}
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
		inflight[n_inflight].last_sent_abs = latest_sent_abs;
		inflight[n_inflight].first_sent_abs = m->next_send_frame;
		inflight[n_inflight].active = 1;
		n_inflight++;
	}

	return n_inflight;
}

/* Check if a message is excluded by fragmentation rules (§4.2).
 * Returns 1 if the message should be skipped, 0 if allowed.
 *
 * Enforces the following spec requirements:
 *
 *  §4.2 ①: Once an address is used to begin transmitting a fragmented
 *     message, that same address must not be used to start a new
 *     fragmented transmission until the first has been completed.
 *
 *  §4.2 ②: For the duration that an address is being used to send a
 *     fragmented message, that same address must not appear more than
 *     once in any Frame to send an unfragmented message.
 *
 *  §4.2 ④: The transmission interval between each fragment of the same
 *     message must be 32 Frames or less.  When the channel is shared
 *     with another system, or in the case of multiple transmission or
 *     Multi-area/Roaming channel, the interval can be 128 Frames.
 *
 *  Exception: the NEXT eligible fragment of the in-flight stream IS
 *     allowed (it must go out to maintain the fragment interval).
 *
 * interval_limit: max frames between consecutive fragments
 *   (FLEX_FRAG_INTERVAL_SINGLE=32 or FLEX_FRAG_INTERVAL_SHARED=128).
 * current_abs: current absolute frame number.
 * interval_warning: set to 1 if interval is within 4 frames of limit. */
static inline int frag_excluded(const flex_msg_t *m,
                                const flex_inflight_frag_t *inflight,
                                int n_inflight,
                                uint32_t current_abs,
                                int interval_limit,
                                int *interval_warning)
{
	int k;
	if (interval_warning)
		*interval_warning = 0;

	for (k = 0; k < n_inflight; k++) {
		if (inflight[k].capcode != m->capcode)
			continue;

		/* Same capcode has in-flight fragment stream */

		/* Is this message the next fragment of THAT stream? */
		if (m->total_fragments == inflight[k].total_fragments &&
		    m->retrieval_num == inflight[k].retrieval_num &&
		    m->fragment_index == inflight[k].last_sent_idx + 1) {
			/* Check interval: if last_sent_abs is known and
			 * the gap would exceed the limit, still allow
			 * the fragment (it MUST go out to avoid timeout)
			 * but set the warning flag. */
			if (inflight[k].last_sent_abs > 0 && current_abs > 0) {
				uint32_t gap = current_abs - inflight[k].last_sent_abs;
				if (gap > (uint32_t)interval_limit) {
					/* Interval exceeded -- fragment stream
					 * is already lost on the pager side.
					 * Still allow it so we finish the stream. */
					LOGP(DFLEX, LOGL_ERROR,
					     "Fragment interval EXCEEDED: cap=%" PRIu64
					     " ret=%u gap=%u limit=%d -- pager may have abandoned.\n",
					     m->capcode, m->retrieval_num,
					     gap, interval_limit);
				}
				if (interval_warning &&
				    gap >= (uint32_t)(interval_limit - 4))
					*interval_warning = 1;
			}
			return 0; /* allowed: this IS the continuation */
		}

		/* 4.2 (1): new fragmented message for same capcode -- blocked */
		if (m->total_fragments > 1)
			return 1;

		/* 4.2 (2): unfragmented message for same capcode -- blocked.
		 *
		 * NOTE: per 3.8.5 rule 3, the standard allows the same
		 * address to appear ONCE per frame for unfragmented messages
		 * during in-flight fragmentation.  We currently block all
		 * unfragmented messages for the same capcode, which is more
		 * restrictive than required but safe.  To relax this, the
		 * caller would need to track per-frame address usage counts
		 * and pass them here. */
		return 1;
	}
	return 0; /* capcode has no in-flight fragments -- allowed */
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
	flex_msg_t *msg;
	int msgs_packed = 0;
	int error = 0;
	size_t len;
	flex_inflight_frag_t inflight[FLEX_MAX_INFLIGHT];
	int n_inflight = 0;

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

	/* === Polarity selection (Req 1.3-1.6) ===
	 *
	 * Select which polarity to transmit this frame period.
	 * Round-robin when both have messages, single when only one,
	 * normal default when idle. */
	{
		int selected_pol;
		int normal_has = flex->tx_pol[FLEX_POL_NORMAL].msg_count > 0;
		int inverted_has = flex->tx_pol[FLEX_POL_INVERTED].msg_count > 0;

		if (flex->fixed_polarity != 0.0) {
			/* Fixed-polarity mode: always use the fixed polarity */
			selected_pol = pol_index(flex->fixed_polarity);
		} else if (normal_has && inverted_has) {
			/* Both have messages: round-robin */
			selected_pol = (flex->last_tx_polarity == FLEX_POL_NORMAL)
				? FLEX_POL_INVERTED : FLEX_POL_NORMAL;
		} else if (normal_has) {
			selected_pol = FLEX_POL_NORMAL;
		} else if (inverted_has) {
			selected_pol = FLEX_POL_INVERTED;
		} else {
			/* Neither has messages: default to normal for idle */
			selected_pol = FLEX_POL_NORMAL;
		}

		/* Apply polarity to DSP */
		dsp_set_polarity(flex, (selected_pol == FLEX_POL_INVERTED) ? -1.0 : 1.0);
		flex->last_tx_polarity = selected_pol;
	}

	/* Refill scan queue if running low */
	if (flex->scan_from < flex->scan_to)
		flex_scan_or_loopback(flex);

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
	/* FIW n flag: set by --roaming CLI flag (§6).
	 * n=1 indicates Roaming Service is provided. Default n=0. */
	params.roaming = flex->roaming_active ? 1 : 0;
	params.collapse = flex->collapse;
	params.biw_time = flex->biw_time_enabled;
	params.chan_setup_enabled = flex->chan_setup_enabled;
	params.hack_nonstandard_decoders = flex->hack_nonstandard_decoders;
	params.bitrate = flex_scheduler_select_speed(flex, &params.modulation_type);

	/* Parameter change guard (§3.4.2).
	 * Tick the state machine and check if we're in cooldown
	 * (force_idle = suppress message packing, send idle frames). */
	{
		uint32_t abs_frame = ft.cycle * 128 + ft.frame;
		int force_idle = 0;
		flex_scheduler_param_change_tick(flex, abs_frame, &force_idle);
		if (force_idle)
			goto send_idle;
	}

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

	/* === BIW carousel selection (Req 3) ===
	 * Select which BIW2/3/4 words go in this frame.
	 * The carousel tracks per-phase, per-polarity state.
	 * Result is used for both capacity estimation and encoding. */
	{
		int selected_pol = flex->last_tx_polarity;
		int phase_idx = 0; /* TODO: per-phase in multi-phase mode */
		flex_biw_carousel_t *car = &flex->tx_pol[selected_pol].biw_carousel[phase_idx];
		flex_biw_config_t biw_cfg;
		int biw_types[BIW_MAX_EXTRA];
		int biw_n = 0;
		flex_biw_time_val_t biw_time_val;

		memset(&biw_cfg, 0, sizeof(biw_cfg));
		biw_cfg.ssid1_configured = (flex->ssid || flex->nid) ? 1 : 0;
		biw_cfg.ssid2_configured = (flex->country_code || flex->tmf) ? 1 : 0;
		biw_cfg.biw_time_enabled = flex->biw_time_enabled;
		biw_cfg.timezone_configured = (flex->timezone_code >= 0) ? 1 : 0;
		biw_cfg.chan_setup_enabled = flex->chan_setup_enabled;
		biw_cfg.roaming_active = flex->roaming_active;
		biw_cfg.has_sysmsg = 0; /* set later if sysmsg detected */

		flex_biw_carousel_select(car, &biw_cfg,
					 ft.frame, ft.cycle,
					 ft.cycle * 128 + ft.frame,
					 biw_types, &biw_n, &biw_time_val);

		/* Store carousel BIW count for capacity estimation.
		 * The old static flex_compute_biw_count() is still used
		 * by the encoder -- this carousel count is for the
		 * scheduler's capacity estimation only.  Full encoder
		 * integration happens when the encoder reads carousel
		 * output instead of computing BIW selection itself. */
		params.carousel_biw_count = 1 + biw_n; /* BIW1 + extras */

		/* Update carousel state after selection.
		 * This must happen even if no messages are packed --
		 * the carousel tracks transmission regardless. */
		flex_biw_carousel_update(car, biw_types, biw_n,
					 ft.cycle * 128 + ft.frame);
	}

	/* FIW n=0 by default. Set by --roaming CLI flag. */

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

	/* Early carry-on computation (§3.7.1, §4.2.1, §4.2.2).
	 *
	 * Must be computed BEFORE the candidate collection loop so the
	 * collapse filter can allow continuation fragments on non-collapse
	 * frames within the carry-on window.
	 *
	 * Per §4.2.1: the 1st fragment is transmitted matching the Pager
	 * Collapse value.  The 2nd and subsequent fragments are generally
	 * sent in Frames which have continuance with the Frame in which
	 * the 1st fragment was transmitted.  The BIW1 carry-on field
	 * (0-3) tells the pager how many additional frames beyond its
	 * assigned collapse frame it should continue decoding.
	 *
	 * Per §4.2.2 Rule ②: carry-on ≤ 2^m - 1 (collapse cycle - 1).
	 * Per spec: "Carry On is not allowed for multiple transmission." */
	if (flex->collapse > 0 && flex->num_transmissions <= 1 && n_inflight > 0) {
		flex_msg_t *qm;
		int max_remaining = 0;
		int sel_pol = flex->last_tx_polarity;
		for (qm = flex->msg_list; qm; qm = qm->next) {
			if (pol_index(qm->polarity) != sel_pol)
				continue;
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
		if (max_remaining > 0) {
			int alpha_max = (1 << flex->collapse) - 1;
			if (max_remaining > alpha_max)
				max_remaining = alpha_max;
			params.carry_on = (max_remaining > 3) ? 3 : max_remaining;
		}
	}

	/* Compute available frame capacity */

	/* Pre-scan: detect system messages to set sysmsg_a_type
	 * before BIW count computation (§3.9.2).
	 *
	 * Also determines tone-only exclusion per §3.9.2:
	 * "Tone-Only Addresses cannot be transmitted in Frames
	 * used for transmitting System Messages."  This applies
	 * when the frame contains system message content — i.e.,
	 * an Operator Messaging Address or Network Address with a
	 * message body.  Timezone-only BIW (A=0100) does NOT
	 * exclude tone-only since it carries data only in the
	 * I-field, not in MF.
	 *
	 * sysmsg_a_type is set for methods (a) and (b) — both
	 * require BIW101.  Method (c) skips BIW101. */
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
			if (stype == FLEX_ADDR_OPER_MSG) {
				uint32_t aw = (uint32_t)(scan->capcode
					+ FLEX_SHORT_ADDR_OFFSET);
				uint32_t lsb = aw & FLEX_OPER_MSG_LSB_MASK;
				has_sysmsg_content = 1;
				/* LSB 0-3 map directly to BIW101
				 * A-type 0000-0011 (§3.7.2-2).
				 * Only set for methods (a) and (b). */
				if (lsb <= FLEX_BIW_SYSINFO_A_MSG_SSID &&
				    params.sysmsg_a_type < 0 &&
				    scan->sysmsg_method != 'c')
					params.sysmsg_a_type = (int)lsb;
				break;
			}
			if (stype == FLEX_ADDR_NETWORK) {
				has_sysmsg_content = 1;
				break;
			}
		}
	}

	/* Compute phase count and per-phase capacities early — needed
	 * for candidate collection directly into per-phase lists. */
	int num_phases = flex_get_phase_count(params.bitrate, params.modulation_type);
	int pcap[FLEX_MAX_PHASES];
	flex_candidate_t phase_cands[FLEX_MAX_PHASES][FLEX_MAX_CANDIDATES];
	int phase_n_cands[FLEX_MAX_PHASES];
	int phase_est_used[FLEX_MAX_PHASES];
	{
		int p;
		for (p = 0; p < num_phases; p++) {
			pcap[p] = flex_phase_capacity(&params, p, num_phases);
			phase_n_cands[p] = 0;
			phase_est_used[p] = 0;
		}
	}

	/* Co-packing (§4.1, §4.2.3):
	 *
	 * Per §4.1: "other paging messages to other pagers can also exist
	 * within the same Frame that a fragmented message is being sent in."
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
	 * other-capcode messages as phase_est_used accumulates
	 * per phase.
	 *
	 * Per-frame independent co-packing is inherent in the
	 * frame-at-a-time architecture: each frame cycle runs the
	 * full collection fresh with phase_est_used starting at 0,
	 * and unpacked messages remain in the queue for the next
	 * frame. */

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

				/* Collapse filter (§3.1.2, §4.2.1).
				 *
				 * Each capcode has an assigned collapse frame; the
				 * pager only decodes frames matching its schedule.
				 *
				 * Exception 1: temp group DELIVERY uses a Temporary
				 * Address pinned to a specific frame, not by capcode.
				 *
				 * Exception 2 (§4.2.1): continuation fragments of an
				 * in-flight fragmented message are allowed on frames
				 * adjacent to the collapse frame, up to carry-on
				 * frames beyond it.  Per spec: "the 2nd and subsequent
				 * fragments are generally sent in Frames which have
				 * continuance with the Frame in which the 1st fragment
				 * was transmitted."  The BIW1 carry-on field tells the
				 * pager to keep decoding these extra frames.
				 *
				 * Without this exception, continuation fragments would
				 * be blocked until the next collapse-aligned frame,
				 * potentially exceeding the §4.2 ④ interval limit
				 * (32/128 frames). */
				if (flex->collapse > 0 &&
				    candidate->temp_delivery_slot < 0) {
					flex_capcode_sched_t sched;
					uint32_t nf;
					flex_scheduler_capcode_info(candidate->capcode, &sched);
					nf = flex_scheduler_next_frame(ft.frame,
								       sched.assigned_frame,
								       flex->collapse);
					if (nf != ft.frame) {
						/* Not the assigned collapse frame.
						 * Allow if this is a continuation fragment
						 * of an in-flight stream AND we are within
						 * carry-on range of the collapse frame. */
						int is_continuation_frag = 0;
						if (candidate->total_fragments > 1 &&
						    candidate->fragment_index > 0) {
							int k;
							for (k = 0; k < n_inflight; k++) {
								if (inflight[k].capcode == candidate->capcode &&
								    candidate->total_fragments == inflight[k].total_fragments &&
								    candidate->retrieval_num == inflight[k].retrieval_num &&
								    candidate->fragment_index == inflight[k].last_sent_idx + 1) {
									is_continuation_frag = 1;
									break;
								}
							}
						}
						if (is_continuation_frag && params.carry_on > 0) {
							/* Check if we're within carry-on range
							 * of the most recent collapse frame.
							 * The carry-on value (1-3) in BIW1
							 * tells the pager to decode this many
							 * extra frames beyond its assigned
							 * collapse frame. */
							uint32_t collapse_cycle = 1U << flex->collapse;
							uint32_t target = sched.assigned_frame % collapse_cycle;
							uint32_t cur_mod = ft.frame % collapse_cycle;
							uint32_t dist;
							/* Distance from most recent collapse frame */
							if (cur_mod >= target)
								dist = cur_mod - target;
							else
								dist = collapse_cycle - target + cur_mod;
							if (dist > 0 && dist <= (uint32_t)params.carry_on) {
								LOGP(DFLEX, LOGL_DEBUG,
								     "Scheduler: allowing continuation frag cap=%" PRIu64
								     " idx=%d on non-collapse frame %u (carry-on=%d, dist=%u)\n",
								     candidate->capcode, candidate->fragment_index,
								     ft.frame, params.carry_on, dist);
							} else {
								continue; /* outside carry-on range */
							}
						} else {
							continue; /* not a continuation or no carry-on */
						}
					}
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

				/* Fragmentation address exclusion and interval check
				 * (§4.2 rules ①②④).
				 *
				 * §4.2 ①: same capcode can't start new fragmented TX
				 *   while one is in-flight.
				 * §4.2 ②: same capcode can't appear for unfragmented
				 *   msg while in-flight.
				 * §4.2 ④: fragment interval must not exceed limit
				 *   (32 frames single / 128 frames shared channel).
				 *
				 * The interval limit depends on channel sharing:
				 *   - Single system: 32 frames (1 minute equivalent)
				 *   - Shared / multiple TX / roaming: 128 frames
				 *     (4 minutes equivalent) */
				{
					int frag_interval = (flex->num_transmissions > 1 ||
							     flex->roaming_active ||
							     flex->pocsag_mix_enabled)
						? FLEX_FRAG_INTERVAL_SHARED
						: FLEX_FRAG_INTERVAL_SINGLE;
					int iw = 0;
					if (frag_excluded(candidate, inflight, n_inflight,
							  current_abs, frag_interval, &iw)) {
						LOGP(DFLEX, LOGL_DEBUG,
						     "Scheduler: skip cap=%" PRIu64 " frag_idx=%d"
						     " -- frag address exclusion\n",
						     candidate->capcode, candidate->fragment_index);
						continue;
					}
					if (iw) {
						LOGP(DFLEX, LOGL_NOTICE,
						     "Scheduler: fragment interval WARNING:"
						     " cap=%" PRIu64 " ret=%u approaching"
						     " limit (%d frames)\n",
						     candidate->capcode,
						     candidate->retrieval_num,
						     frag_interval);
					}
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

			/* Add sorted candidates directly to per-phase lists.
			 * Each phase has its own capacity — no shared limit. */
			{
				int i;
				for (i = 0; i < n_sorted; i++) {
					flex_msg_t *m = sorted[i];
					int is_long = (m->capcode >= FLEX_LONG_ADDR_MIN
						       && m->temp_delivery_slot < 0);
					int cost = flex_estimate_msg_cost(m);
					int aw, vw, bw;
					int phase_idx;
					flex_capcode_sched_t sched;

					if (cost < 0)
						continue; /* invalid message */

					/* Determine target phase */
					flex_scheduler_capcode_info(m->capcode, &sched);
					phase_idx = (int)(sched.assigned_phase % (uint32_t)num_phases);
					if (m->phase >= 0)
						phase_idx = flex_map_phase(m->phase, num_phases);

					/* Single-phase mode: everything goes to phase 0 */
					if (num_phases == 1)
						phase_idx = 0;

					if (phase_n_cands[phase_idx] >= FLEX_MAX_CANDIDATES)
						continue;
					if (phase_est_used[phase_idx] + cost > pcap[phase_idx]) {
						LOGP(DFLEX, LOGL_DEBUG,
						     "Scheduler: skip capcode=%" PRIu64
						     " est=%d words (phase %d capacity=%d used=%d)\n",
						     m->capcode, cost, phase_idx,
						     pcap[phase_idx], phase_est_used[phase_idx]);
						continue;
					}

					aw = is_long ? 2 : 1;
					if (m->msg_type == FLEX_MSG_TYPE_TONE) {
						vw = 0;
						bw = 0;
					} else {
						vw = is_long ? 2 : 1;
						bw = cost - aw - vw;
					}

					phase_cands[phase_idx][phase_n_cands[phase_idx]].msg = m;
					phase_cands[phase_idx][phase_n_cands[phase_idx]].est_words = cost;
					phase_cands[phase_idx][phase_n_cands[phase_idx]].addr_words = aw;
					phase_cands[phase_idx][phase_n_cands[phase_idx]].vec_words = vw;
					phase_cands[phase_idx][phase_n_cands[phase_idx]].body_words = bw;
					phase_cands[phase_idx][phase_n_cands[phase_idx]].is_long = is_long;
					phase_n_cands[phase_idx]++;
					phase_est_used[phase_idx] += cost;
				}
			}
		}

	/* Set msg pointer: if any phase has candidates, enter encoding path */
	{
		int p;
		msg = NULL;
		for (p = 0; p < num_phases; p++) {
			if (phase_n_cands[p] > 0) {
				msg = phase_cands[p][0].msg;
				break;
			}
		}
	}

	/* Unified phase encoding for all modes (1, 2, or 4 phases).
	 * Single-phase (1600/2FSK) is just num_phases=1. */
	if (msg) {
		flex_phase_data_t phases[FLEX_MAX_PHASES];
		flex_frame_params_t phase_params;
		uint8_t phase_buf[FLEX_BUFFER_SIZE];
		int j, p, any_msg = 0;

		memset(phases, 0, sizeof(phases));

		/* Build per-phase params — same as frame params but force
		 * single-phase output so we can extract the 88 data words */
		phase_params = params;
		phase_params.single_phase = 1;

		/* Carry-on: propagate to phase_params.
		 *
		 * params.carry_on was already computed early (before candidate
		 * collection) so the collapse filter could use it.  Copy to
		 * phase_params for per-phase encoding. */
		phase_params.carry_on = params.carry_on;

		/* Encode each phase independently (Req 2.2, 2.3) */
		for (p = 0; p < num_phases; p++) {
			if (phase_n_cands[p] == 0) {
				flex_fill_idle_phase(phases[p].words, p,
						     params.modulation_type,
						     params.bitrate);
				{
					int blk;
					for (blk = 0; blk < FLEX_BLOCKS_PER_FRAME; blk++)
						flex_interleave_block(blk, phases[p].words);
				}
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
				phase_params.phase_index = p;
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
							else {
								/* Per-capcode N_Counter (Req 4.1-4.3) */
								int mp = pol_index(m->polarity);
								uint32_t af = ft.cycle * 128 + ft.frame;
								m->assigned_n = (int)flex_n_counter_get(
									&flex->tx_pol[mp], m->capcode, af);
							}
							fm->sequence_num = m->assigned_n;
							LOGP(DFLEX, LOGL_INFO,
							     "TX: capcode=%" PRIu64 " N=%d R=1 type=%s frags=%d retransmit=%d/%d C%u/F%u\n",
							     m->capcode, m->assigned_n,
							     flex_msg_type_name(m->msg_type),
							     m->total_fragments,
							     m->retransmit_count, m->retransmit_max,
							     ft.cycle, ft.frame);
							/* R=1 for normal initial TX.
							 * R=0 when msg->numbered_r is explicitly 0
							 * (system messages per spec 3.9.2). */
							fm->alpha_r_flag = m->numbered_r ? 1 : 0;
							fm->hex_r_flag = m->numbered_r ? 1 : 0;
							fm->numbered_r = m->numbered_r ? 1 : 0;
							fm->sysmsg_method = m->sysmsg_method;
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
							LOGP(DFLEX, LOGL_INFO,
							     "TX: capcode=%" PRIu64 " N=%d R=0 (retransmit %d/%d) type=%s C%u/F%u\n",
							     m->capcode, m->assigned_n,
							     m->retransmit_count, m->retransmit_max,
							     flex_msg_type_name(m->msg_type),
							     ft.cycle, ft.frame);
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
					{
						int blk;
						for (blk = 0; blk < FLEX_BLOCKS_PER_FRAME; blk++)
							flex_interleave_block(blk, phases[p].words);
					}
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
					/* Block-interleave the idle phase.
					 * The phase interleaver expects all
					 * phases to be block-interleaved;
					 * without this, the receiver's
					 * de-interleave produces garbage. */
					{
						int blk;
						for (blk = 0; blk < FLEX_BLOCKS_PER_FRAME; blk++)
							flex_interleave_block(blk, phases[p].words);
					}
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
				  (flex->fsk_polarity < 0) ? "inverted" : "normal",
				  params.collapse, params.roaming,
				  params.num_transmissions,
				  params.subframe_index, params.num_transmissions);
			return 1;
		}
		/* No eligible messages found — fall through to idle */
		goto send_idle;
	}

send_idle:
	/* No messages — send proper idle frame per §3.4.1 Fig. 3.4.1-3.
	 *
	 * Pass msg_count=0 to flex_encode_frame_multi so it produces a
	 * BIW-only frame: BIW1 with voffset==aoffset (no addresses,
	 * vectors, or messages), remaining words filled with idle pattern
	 * (alternating all-1s / all-0s codewords per Table 3.4.1-1).
	 *
	 * The collapse value in BIW1 is preserved from params so pagers
	 * can maintain their decode schedule even during idle periods. */
	flex_setup_frame_buffers(flex, &params, NULL, 0,
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
		/* Refill scan queue if running low */
		if (flex->scan_from < flex->scan_to)
			flex_scan_or_loopback(flex);

		/* Fragment any oversized messages before selection */
		flex_fragment_queue(flex);

		msg = flex->msg_list;
		if (msg) {
			flex_frame_msg_t frame_msg;
			flex_frame_params_t params;
			int msgs_packed = 0;
			double polarity;
			/* One-shot mode: transmit 4 frames total:
			 *   0 = default polarity, R=1 (new message)
			 *   1 = default polarity, R=0 (retransmission, same N)
			 *   2 = inverted polarity, R=1
			 *   3 = inverted polarity, R=0
			 * idle_count tracks which pass (0-3). */
			int pass = flex->idle_count;
			int is_retransmit = (pass == 1 || pass == 3);
			int is_inverted = (pass >= 2);

			if (is_inverted) {
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
			/* R flag: R=1 for initial TX, R=0 for retransmit pass */
			frame_msg.alpha_r_flag = is_retransmit ? 0 : (msg->numbered_r ? 1 : 0);
			frame_msg.hex_r_flag = is_retransmit ? 0 : (msg->numbered_r ? 1 : 0);
			frame_msg.numbered_r = is_retransmit ? 0 : (msg->numbered_r ? 1 : 0);
			frame_msg.sysmsg_method = msg->sysmsg_method;

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
			/* FIW n flag: set by --roaming CLI flag (§6).
			 * n=1 indicates Roaming Service is provided. Default n=0. */
			if (flex->roaming_active)
				params.roaming = 1;
			if (flex->country_code || flex->tmf) {
				params.country_code = flex->country_code;
				params.tmf = flex->tmf;
			}
			params.timezone_code = flex->timezone_code;

			/* FIW n=0 by default. Set by --roaming CLI flag. */

			/* Set sysmsg_a_type for methods (a)/(b) (§3.9.2).
			 * Detect operator messaging address with LSB 0-3
			 * and method != 'c' to emit BIW101. */
			{
				enum flex_addr_type stype =
					flex_capcode_special_type(msg->capcode);
				if (stype == FLEX_ADDR_OPER_MSG) {
					uint32_t aw = (uint32_t)(msg->capcode
						+ FLEX_SHORT_ADDR_OFFSET);
					uint32_t lsb = aw & FLEX_OPER_MSG_LSB_MASK;
					if (lsb <= FLEX_BIW_SYSINFO_A_MSG_SSID &&
					    msg->sysmsg_method != 'c')
						params.sysmsg_a_type = (int)lsb;
				}
			}

			flex_setup_frame_buffers(flex, &params, &frame_msg, 1,
						&msgs_packed, &error);

			if (error || flex->frame_buffer_length == 0) {
				LOGP_CHAN(DFLEX, LOGL_NOTICE,
					  "Failed to encode FLEX frame (error=%d).\n", error);
				flex_msg_destroy(msg);
				goto oneshot_done;
			}

			LOGP_CHAN(DFLEX, LOGL_INFO,
				  "One-shot: C0/F0 pass=%d polarity=%s R=%d capcode=%" PRIu64
				  " type=%d speed=%d len=%d.\n",
				  pass,
				  (polarity < 0) ? "inverted" : "normal",
				  is_retransmit ? 0 : 1,
				  msg->capcode, (int)msg->msg_type,
				  msg->speed, msg->data_length);

			if (pass >= 3) {
				/* All 4 passes done — destroy message */
				flex->msg_sequence++; /* bump for next message */
				flex_msg_destroy(msg);
				flex->idle_count = 4;
			} else {
				flex->idle_count = pass + 1;
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
	flex->scan_start = scan_from;
	flex->scan_last_progress = scan_from;
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		flex->scan_start_time = (double)tv.tv_sec + tv.tv_usec / 1e6;
	}

	/* Enable RX path — always on so we can monitor our own transmission
	 * (loopback) or receive off-air FLEX signals for debugging.
	 * State starts at 0 (== RX_STATE_SYNC1) from calloc. */
	flex->rx.enabled = 1;
	flex->rx.sample_freq = samplerate;
	flex->rx.baud = 1600;

	/* Init per-polarity TX scheduling state (Req 1, 7).
	 * N_Counter hash tables and BIW carousel for both polarities. */
	{
		int p;
		for (p = 0; p < FLEX_TX_POLARITIES; p++) {
			rc = flex_n_counter_init(&flex->tx_pol[p]);
			if (rc < 0) {
				LOGP(DFLEX, LOGL_ERROR,
				     "Failed to init N_Counter for polarity %d!\n", p);
				goto error;
			}
			{
				int ph;
				for (ph = 0; ph < FLEX_MAX_PHASES; ph++)
					flex_biw_carousel_init(&flex->tx_pol[p].biw_carousel[ph]);
			}
		}
	}

	flex_display_status();

	LOGP(DFLEX, LOGL_NOTICE, "Created 'Kanal' %s: samplerate=%d deviation=%.0f polarity=%s tx=%d.\n",
	     kanal, samplerate, deviation, (polarity < 0) ? "inverted" : "normal", tx);

	return 0;

error:
	flex_destroy(&flex->sender);

	return rc;
}

void flex_destroy(sender_t *sender)
{
	flex_t *flex = (flex_t *) sender;
	int p;

	LOGP(DFLEX, LOGL_DEBUG, "Destroying 'FLEX' instance for 'Kanal' = %s.\n", sender->kanal);

	/* Cleanup per-polarity TX state */
	for (p = 0; p < FLEX_TX_POLARITIES; p++)
		flex_n_counter_cleanup(&flex->tx_pol[p]);

	while (flex->msg_list)
		flex_msg_destroy(flex->msg_list);
	dsp_cleanup_sender(flex);
	sender_destroy(&flex->sender);
	free(flex);
}

/*
 * Scan or loopback: generate test messages for scanning or loopback mode.
 *
 * Scan mode: batch-fill the queue with up to FLEX_SCAN_BATCH_SIZE messages,
 * skipping reserved/special capcodes.  The scheduler packs them efficiently
 * into frames by capcode-to-frame/phase mapping.
 *
 * Called at startup and periodically when the queue runs low.
 *
 * Returns the number of messages enqueued.
 */
#define FLEX_SCAN_BATCH_SIZE	4096
#define FLEX_SCAN_REFILL_THRESHOLD 2048

int flex_scan_or_loopback(flex_t *flex)
{
	if (flex->scan_from < flex->scan_to) {
		int queued = 0;
		int pi = pol_index(flex->default_polarity ? flex->default_polarity : FLEX_DEFAULT_POLARITY);

		/* Only refill if queue is below threshold */
		if (flex->tx_pol[pi].msg_count >= FLEX_SCAN_REFILL_THRESHOLD)
			return 0;

		while (flex->scan_from < flex->scan_to && queued < FLEX_SCAN_BATCH_SIZE) {
			uint64_t cap = flex->scan_from;
			char autobuf[32];
			const char *msg_text;
			int msg_len;
			flex_msg_t *msg;

			flex->scan_from++;

			/* Skip special/reserved capcodes */
			if (flex_capcode_is_special(cap))
				continue;

			/* Validate capcode is in a usable range */
			if (!flex_capcode_valid(cap))
				continue;

			/* Generate message payload */
			if (flex->default_message && flex->default_message[0]) {
				msg_text = flex->default_message;
				msg_len = strlen(msg_text);
			} else {
				switch (flex->default_msg_type) {
				case FLEX_MSG_TYPE_SHORT:
					/* Short numeric: use every BCD slot with
					 * most significant digits of the capcode.
					 * Short addr: first 3 digits.
					 * Long addr: first 8 digits. */
					{
						char full[24];
						int max_d = (cap < FLEX_LONG_ADDR_MIN) ? 3 : 8;
						sprintf(full, "%" PRIu64, cap);
						int flen = strlen(full);
						if (flen <= max_d) {
							memcpy(autobuf, full, flen);
							autobuf[flen] = '\0';
						} else {
							memcpy(autobuf, full, max_d);
							autobuf[max_d] = '\0';
						}
					}
					break;
				case FLEX_MSG_TYPE_NUMERIC:
					sprintf(autobuf, "%" PRIu64, cap);
					break;
				case FLEX_MSG_TYPE_ALPHA:
					sprintf(autobuf, "%" PRIu64, cap);
					break;
				case FLEX_MSG_TYPE_TONE:
				case FLEX_MSG_TYPE_AUTO:
				default:
					autobuf[0] = '\0';
				}
				msg_text = autobuf;
				msg_len = strlen(autobuf);
			}

			msg = flex_msg_create(flex, cap, flex->default_msg_type,
					      msg_text, msg_len);
			if (msg) {
				if (flex->fixed_speed != -1) {
					msg->speed = flex->fixed_speed;
					msg->modulation_type = flex->fixed_mod_type;
				}
				if (flex->fixed_polarity != 0.0)
					msg->polarity = flex->fixed_polarity;
				if (flex->default_phase >= 0)
					msg->phase = flex->default_phase;
				queued++;
			}
		}

		if (queued > 0)
			LOGP_CHAN(DFLEX, LOGL_NOTICE, "Scan: enqueued %d messages (next=%" PRIu64 " end=%" PRIu64 " queue=%d).\n",
				  queued, flex->scan_from, flex->scan_to, flex->tx_pol[pi].msg_count);

		/* Periodic progress with ETA */
		if (flex->scan_from - flex->scan_last_progress >= 500 ||
		    flex->scan_from >= flex->scan_to) {
			uint64_t total = flex->scan_to - flex->scan_start;
			uint64_t done = flex->scan_from - flex->scan_start;
			double pct = total > 0 ? (double)done * 100.0 / (double)total : 100.0;
			struct timeval tv;
			gettimeofday(&tv, NULL);
			double now = (double)tv.tv_sec + tv.tv_usec / 1e6;
			double elapsed = now - flex->scan_start_time;
			double eta = (done > 0 && done < total)
				? elapsed * (double)(total - done) / (double)done
				: 0.0;
			int eta_d = (int)(eta / 86400);
			int eta_h = (int)((eta - eta_d * 86400) / 3600);
			int eta_m = (int)((eta - eta_d * 86400 - eta_h * 3600) / 60);

			LOGP(DFLEX, LOGL_ERROR, "SCAN PROGRESS: %" PRIu64 "/%" PRIu64 " (%.1f%%) ETA %dd %dh %dm\n",
			     done, total, pct, eta_d, eta_h, eta_m);
			flex->scan_last_progress = flex->scan_from;
		}

		return queued;
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
