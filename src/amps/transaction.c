/* C-Netz transaction handling
 *
 * (C) 2016 by Andreas Eversberg <jolly@eversberg.eu>
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include "amps.h"
//#include "database.h"

static const char *trans_state_name(int state)
{
	switch (state) {
	case 0:
		return "IDLE";
	case TRANS_REGISTER_ACK:
		return "REGISTER ACK";
	case TRANS_REGISTER_ACK_SEND:
		return "REGISTER ACK SEND";
	case TRANS_CALL_MO_WAIT_PROCEED:
		return "MO CALL WAIT PROCEED";
	case TRANS_CALL_MO_ASSIGN:
		return "MO CALL ASSIGNMENT";
	case TRANS_CALL_MO_ASSIGN_SEND:
		return "MO CALL ASSIGNMENT SENDING";
	case TRANS_CALL_MO_ASSIGN_CONFIRM:
		return "MO CALL ASSIGNMENT WAIT CONFIRM";
	case TRANS_CALL_MT_ASSIGN:
		return "MT CALL ASSIGNMENT";
	case TRANS_CALL_MT_ASSIGN_SEND:
		return "MT CALL ASSIGNMENT SENDING";
	case TRANS_CALL_MT_ASSIGN_CONFIRM:
		return "MT CALL ASSIGNMENT WAIT CONFIRM";
	case TRANS_CALL_MT_ALERT:
		return "MT CALL ALERT";
	case TRANS_CALL_MT_ALERT_SEND:
		return "MT CALL ALERT SENDING";
	case TRANS_CALL_MT_ALERT_CONFIRM:
		return "MT CALL ALERT WAIT CONFIRM";
	case TRANS_CALL_MT_ANSWER_WAIT:
		return "MT CALL ANSWER WAIT";
	case TRANS_CALL_REJECT:
		return "CALL REJECT";
	case TRANS_CALL_REJECT_SEND:
		return "CALL REJECT SEND";
	case TRANS_CALL:
		return "CALL";
	case TRANS_CALL_CHANGE_POWER:
		return "CHANGE POWER";
	case TRANS_CALL_CHANGE_POWER_SEND:
		return "CHANGE POWER SEND";
	case TRANS_CALL_RELEASE:
		return "CALL RELEASE";
	case TRANS_CALL_RELEASE_SEND:
		return "CALL RELEASE SEND";
	case TRANS_CALL_FLASH_INFO:
		return "FLASH WITH INFO";
	case TRANS_CALL_FLASH_INFO_SEND:
		return "FLASH WITH INFO SEND";
	case TRANS_CALL_PCI_QUERY:
		return "PCI QUERY";
	case TRANS_CALL_PCI_QUERY_SEND:
		return "PCI QUERY SEND";
	case TRANS_CALL_AUDIT:
		return "AUDIT";
	case TRANS_CALL_AUDIT_SEND:
		return "AUDIT SEND";
	case TRANS_AUDIT:
		return "FOCC AUDIT";
	case TRANS_AUDIT_SEND:
		return "FOCC AUDIT SEND";
	case TRANS_PCI:
		return "FOCC PCI";
	case TRANS_PCI_SEND:
		return "FOCC PCI SEND";
	case TRANS_MWI:
		return "FOCC MWI";
	case TRANS_MWI_SEND:
		return "FOCC MWI SEND";
	case TRANS_REORDER:
		return "FOCC REORDER";
	case TRANS_REORDER_SEND:
		return "FOCC REORDER SEND";
	case TRANS_INTERCEPT:
		return "FOCC INTERCEPT";
	case TRANS_INTERCEPT_SEND:
		return "FOCC INTERCEPT SEND";
	case TRANS_PAGE:
		return "PAGE";
	case TRANS_PAGE_SEND:
		return "PAGE SEND";
	case TRANS_PAGE_REPLY:
		return "PAGE REPLY";
	case TRANS_CALL_REORDER:
		return "REORDER";
	case TRANS_CALL_REORDER_SEND:
		return "REORDER SEND";
	case TRANS_CALL_MWI:
		return "MESSAGE WAITING";
	case TRANS_CALL_MWI_SEND:
		return "MESSAGE WAITING SEND";
	case TRANS_CALL_STOP_ALERT:
		return "STOP ALERT";
	case TRANS_CALL_STOP_ALERT_SEND:
		return "STOP ALERT SEND";
	case TRANS_CALL_INTERCEPT:
		return "INTERCEPT";
	case TRANS_CALL_INTERCEPT_SEND:
		return "INTERCEPT SEND";
	case TRANS_CALL_MAINTENANCE:
		return "MAINTENANCE";
	case TRANS_CALL_MAINTENANCE_SEND:
		return "MAINTENANCE SEND";
	case TRANS_CALL_ABB_ALERT:
		return "ABBREVIATED ALERT";
	case TRANS_CALL_ABB_ALERT_SEND:
		return "ABBREVIATED ALERT SEND";
	case TRANS_CALL_ESN_REQUEST:
		return "ESN REQUEST";
	case TRANS_CALL_ESN_REQUEST_SEND:
		return "ESN REQUEST SEND";
	case TRANS_CALL_ESN_REPLY:
		return "ESN REPLY";
	case TRANS_CALL_DIGITS_REQUEST:
		return "DIGITS REQUEST";
	case TRANS_CALL_DIGITS_REQUEST_SEND:
		return "DIGITS REQUEST SEND";
	case TRANS_CALL_DIGITS_REPLY:
		return "DIGITS REPLY";
	case TRANS_CALL_LOCAL_CONTROL:
		return "LOCAL CONTROL";
	case TRANS_CALL_LOCAL_CONTROL_SEND:
		return "LOCAL CONTROL SEND";
	case TRANS_CALL_DISABLE_DTMF:
		return "DISABLE DTMF";
	case TRANS_CALL_DISABLE_DTMF_SEND:
		return "DISABLE DTMF SEND";
	case TRANS_CALL_HANDOFF:
		return "HANDOFF";
	case TRANS_CALL_HANDOFF_SEND:
		return "HANDOFF SEND";
	case TRANS_CALL_HANDOFF_CONFIRM:
		return "HANDOFF CONFIRM";
	case TRANS_DIRECTED_RETRY:
		return "DIRECTED RETRY";
	case TRANS_DIRECTED_RETRY_SEND:
		return "DIRECTED RETRY SEND";
	case TRANS_SILENT_PAGE:
		return "SILENT PAGE";
	case TRANS_SILENT_PAGE_SEND:
		return "SILENT PAGE SEND";
	case TRANS_SILENT_PAGE_REPLY:
		return "SILENT PAGE REPLY";
	case TRANS_SILENT_PAGE_ASSIGN:
		return "SILENT PAGE ASSIGN";
	case TRANS_SILENT_PAGE_ASSIGN_SEND:
		return "SILENT PAGE ASSIGN SEND";
	case TRANS_SILENT_PAGE_ASSIGN_CONFIRM:
		return "SILENT PAGE ASSIGN CONFIRM";
	case TRANS_SILENT_PAGE_MAINTENANCE:
		return "SILENT PAGE MAINTENANCE";
	case TRANS_SILENT_PAGE_MAINTENANCE_SEND:
		return "SILENT PAGE MAINTENANCE SEND";
	default:
		return "<invalid transaction state>";
	}
}

const char *trans_short_state_name(int state)
{
	switch (state) {
	case 0:
		return "IDLE";
	case TRANS_REGISTER_ACK:
	case TRANS_REGISTER_ACK_SEND:
		return "REGISTER";
	case TRANS_CALL_MO_ASSIGN:
	case TRANS_CALL_MO_ASSIGN_SEND:
	case TRANS_CALL_MO_ASSIGN_CONFIRM:
	case TRANS_CALL_MT_ASSIGN:
	case TRANS_CALL_MT_ASSIGN_SEND:
	case TRANS_CALL_MT_ASSIGN_CONFIRM:
		return "ASSIGN";
	case TRANS_CALL_MT_ALERT:
	case TRANS_CALL_MT_ALERT_SEND:
	case TRANS_CALL_MT_ALERT_CONFIRM:
	case TRANS_CALL_MT_ANSWER_WAIT:
		return "ALERT";
	case TRANS_CALL_REJECT:
	case TRANS_CALL_REJECT_SEND:
		return "REJECT";
	case TRANS_CALL:
		return "CALL";
	case TRANS_CALL_RELEASE:
	case TRANS_CALL_RELEASE_SEND:
		return "RELEASE";
	case TRANS_PAGE:
	case TRANS_PAGE_SEND:
	case TRANS_PAGE_REPLY:
		return "PAGE";
	case TRANS_SILENT_PAGE:
	case TRANS_SILENT_PAGE_SEND:
	case TRANS_SILENT_PAGE_REPLY:
	case TRANS_SILENT_PAGE_ASSIGN:
	case TRANS_SILENT_PAGE_ASSIGN_SEND:
	case TRANS_SILENT_PAGE_ASSIGN_CONFIRM:
	case TRANS_SILENT_PAGE_MAINTENANCE:
	case TRANS_SILENT_PAGE_MAINTENANCE_SEND:
		return "SILPAGE";
	case TRANS_CALL_REORDER:
	case TRANS_CALL_REORDER_SEND:
	case TRANS_REORDER:
	case TRANS_REORDER_SEND:
		return "REORDER";
	case TRANS_CALL_MWI:
	case TRANS_CALL_MWI_SEND:
	case TRANS_MWI:
	case TRANS_MWI_SEND:
		return "MWI";
	case TRANS_CALL_STOP_ALERT:
	case TRANS_CALL_STOP_ALERT_SEND:
		return "STOPALRT";
	case TRANS_CALL_INTERCEPT:
	case TRANS_CALL_INTERCEPT_SEND:
	case TRANS_INTERCEPT:
	case TRANS_INTERCEPT_SEND:
		return "INTERCPT";
	case TRANS_CALL_MAINTENANCE:
	case TRANS_CALL_MAINTENANCE_SEND:
		return "MAINT";
	case TRANS_CALL_ABB_ALERT:
	case TRANS_CALL_ABB_ALERT_SEND:
		return "ABBALRT";
	case TRANS_CALL_ESN_REQUEST:
	case TRANS_CALL_ESN_REQUEST_SEND:
	case TRANS_CALL_ESN_REPLY:
		return "ESN";
	case TRANS_CALL_DIGITS_REQUEST:
	case TRANS_CALL_DIGITS_REQUEST_SEND:
	case TRANS_CALL_DIGITS_REPLY:
		return "DIGITS";
	case TRANS_CALL_LOCAL_CONTROL:
	case TRANS_CALL_LOCAL_CONTROL_SEND:
		return "LOCAL";
	case TRANS_CALL_DISABLE_DTMF:
	case TRANS_CALL_DISABLE_DTMF_SEND:
		return "DTMF";
	case TRANS_CALL_HANDOFF:
	case TRANS_CALL_HANDOFF_SEND:
	case TRANS_CALL_HANDOFF_CONFIRM:
		return "HANDOFF";
	case TRANS_DIRECTED_RETRY:
	case TRANS_DIRECTED_RETRY_SEND:
		return "RETRY";
	default:
		return "<invalid transaction state>";
	}
}

/* create transaction */
transaction_t *create_transaction(amps_t *amps, enum amps_trans_state state, uint32_t min1, uint16_t min2, uint32_t esn, uint8_t msg_type, uint8_t ordq, uint8_t order, uint16_t chan)
{
	sender_t *sender;
	transaction_t *trans = NULL;
	amps_t *search_amps;

	/* search transaction for this subscriber */
	for (sender = sender_head; sender; sender = sender->next) {
		search_amps = (amps_t *) sender;
		/* search transaction for this callref */
		trans = search_transaction_number(search_amps, min1, min2);
		if (trans)
			break;
	}
	if (trans) {
		const char *number = amps_min2number(trans->min1, trans->min2);
		int old_callref = trans->callref;
		amps_t *old_amps = trans->amps;
		LOGP(DTRANS, LOGL_NOTICE, "Found already pending transaction for subscriber '%s', deleting!\n", number);
		destroy_transaction(trans);
		if (old_amps) /* should be... */
			amps_go_idle(old_amps);
		if (old_callref)
			call_up_release(old_callref, CAUSE_NORMAL);
	}

	trans = calloc(1, sizeof(*trans));
	if (!trans) {
		LOGP(DTRANS, LOGL_ERROR, "No memory!\n");
		return NULL;
	}

	osmo_timer_setup(&trans->timer, transaction_timeout, trans);

	trans_new_state(trans, state);
	trans->min1 = min1;
	trans->min2 = min2;
	trans->esn = esn;
	trans->msg_type = msg_type;
	trans->ordq = ordq;
	trans->order = order;
	trans->chan = chan;
	trans->vmac_adjust_count = 0;
	trans->vmac_grace_count = 0;
	trans->sat_level_avg = 0.0;
	trans->current_vmac = 0; /* Start with max power */
	/* Initialize VMAC from system info settings */
	trans->current_vmac = amps->si.vmac;
	trans->max_vmac = amps->si.vmac; /* Corresponds to configured Max Power (Min Attenuation) */

	/* Initialize pitch/cadence/pi/si with defaults (or invalid values to indicate fallback) */
	trans->signal_pitch = -1; /* default usage in frame.c */
	trans->signal_cadence = -1; /* default usage in frame.c */
	trans->presentation_indicator = -1;
	trans->screening_indicator = -1;
	
	/* Initialize handoff fields */
	trans->handoff_channel = 0;
	trans->handoff_scc = -1;  /* -1 = use current channel's SAT */

	const char *number = amps_min2number(trans->min1, trans->min2);
	LOGP(DTRANS, LOGL_INFO, "Created transaction for subscriber '%s'\n", number);

	link_transaction(trans, amps);

	return trans;
}

/* destroy transaction */
void destroy_transaction(transaction_t *trans)
{
	unlink_transaction(trans);
	
	const char *number = amps_min2number(trans->min1, trans->min2);
	LOGP(DTRANS, LOGL_INFO, "Destroying transaction for subscriber '%s'\n", number);
	LOGP(DTRANS, LOGL_DEBUG, "DEBUG: Destroying transaction state=%d MIN1=%u MIN2=%u on channel %s\n", 
		trans->state, trans->min1, trans->min2, trans->amps ? trans->amps->sender.kanal : "(no channel)");

	osmo_timer_del(&trans->timer);
	osmo_timer_del(&trans->flash_timer);

	trans_new_state(trans, 0);

	free(trans);
}

/* link transaction to list */
void link_transaction(transaction_t *trans, amps_t *amps)
{
	transaction_t **transp;

	/* attach to end of list, so first transaction is served first */
	LOGP(DTRANS, LOGL_DEBUG, "Linking transaction %p to amps %p\n", trans, amps);
	trans->amps = amps;
	trans->next = NULL;
	transp = &amps->trans_list;
	while (*transp)
		transp = &((*transp)->next);
	*transp = trans;
	amps_display_status();
}

/* unlink transaction from list */
void unlink_transaction(transaction_t *trans)
{
	transaction_t **transp;

	/* unlink */
	LOGP(DTRANS, LOGL_DEBUG, "Unlinking transaction %p from amps %p\n", trans, trans->amps);
	transp = &trans->amps->trans_list;
	while (*transp && *transp != trans)
		transp = &((*transp)->next);
	if (!(*transp)) {
		LOGP(DTRANS, LOGL_ERROR, "Transaction not in list, please fix!!\n");
		abort();
	}
	*transp = trans->next;
	trans->amps = NULL;
	amps_display_status();
}

transaction_t *search_transaction_number(amps_t *amps, uint32_t min1, uint16_t min2)
{
	transaction_t *trans = amps->trans_list;

	while (trans) {
		if (trans->min1 == min1
		 && trans->min2 == min2) {
			const char *number = amps_min2number(trans->min1, trans->min2);
			LOGP(DTRANS, LOGL_DEBUG, "Found transaction for subscriber '%s'\n", number);
			return trans;
		}
		trans = trans->next;
	}

	return NULL;
}

transaction_t *search_transaction_callref(amps_t *amps, int callref)
{
	transaction_t *trans = amps->trans_list;

	/* just in case, this should not happen */
	if (!callref)
		return NULL;
	while (trans) {
		if (trans->callref == callref) {
			const char *number = amps_min2number(trans->min1, trans->min2);
			LOGP(DTRANS, LOGL_DEBUG, "Found transaction for subscriber '%s'\n", number);
			return trans;
		}
		trans = trans->next;
	}

	return NULL;
}

void trans_new_state(transaction_t *trans, int state)
{
	LOGP(DTRANS, LOGL_INFO, "Transaction state %s -> %s\n", trans_state_name(trans->state), trans_state_name(state));
	trans->state = state;
	amps_display_status();
}

void amps_flush_other_transactions(amps_t *amps, transaction_t *trans)
{
	/* flush after this very trans */
	while (trans->next) {
		LOGP(DTRANS, LOGL_NOTICE, "Kicking other pending transaction\n");
		destroy_transaction(trans->next);
	}
	/* flush before this very trans */
	while (amps->trans_list != trans) {
		LOGP(DTRANS, LOGL_NOTICE, "Kicking other pending transaction\n");
		destroy_transaction(amps->trans_list);
	}
}

