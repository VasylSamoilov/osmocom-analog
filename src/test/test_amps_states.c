/* AMPS base station: the orders on the forward voice channel (audit, change power, flash with info,
 * message waiting) in the tasks TIA/EIA-553-A lets the mobile station take them: Waiting for Order
 * (2.6.4.3.1, the alert not confirmed yet), Waiting for Answer (2.6.4.3.2, ringing), Conversation
 * (2.6.4.4). The mobile's signaling tone and the timers keep the meaning of its task while an
 * order is sent (the answer is not lost), and after the order the task goes on.
 *
 * The transaction is set up in its state directly, the frames are taken from the FVC encoder,
 * the signaling tone is given to amps_rx_signaling_tone(), a timer to transaction_timeout().
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/cause.h"
#include "../libmobile/main_mobile.h"
#include "../amps/amps.h"
#include "../amps/frame.h"
#include "../amps/dsp.h"

/* what amps_main.c gives the AMPS library */
const int tacs = 0;
const int jtacs = 0;
const struct number_lengths number_lengths[] = {
	{ 10, "AMPS number (NPA-XXX-XXXX)" },
	{ 0, NULL }
};
const char *number_prefixes[] = {
	"1xxxxxxxxxx",
	"+1xxxxxxxxxx",
	NULL
};

static int failures;

#define CHECK(cond, msg) do { \
	if (cond) { \
		printf("  OK   %s\n", msg); \
	} else { \
		printf("  FAIL %s\n", msg); \
		failures++; \
	} \
} while (0)

#define NUMBER "2162177526"

/* a voice channel with the phone's transaction in a state */
static amps_t *vc;
static transaction_t *trans;

static void setup(enum amps_trans_state state)
{
	uint32_t min1;
	uint16_t min2;

	if (vc) {
		while (vc->trans_list)
			destroy_transaction(vc->trans_list);
	} else {
		vc = calloc(1, sizeof(*vc));
		vc->sender.samplerate = 96000;
		vc->sender.kanal = "356";
		display_measurements_init(&vc->sender.dispmeas, 96000, vc->sender.kanal);
		vc->chan_type = CHAN_TYPE_VC;
		vc->sat = 0;
		vc->si.vmac = 4;
		sender_head = &vc->sender;
	}
	amps_number2min(NUMBER, &min1, &min2);
	trans = create_transaction(vc, state, min1, min2, 0, 0, 0, 0, 356);
	trans->sat_detected = 1;
	vc->dsp_mode = DSP_MODE_AUDIO_RX_AUDIO_TX;
}

/* the FVC frames until the order is out (at most a few); the states it went through are noted */
static int went_through[TRANS_SILENT_PAGE_MAINTENANCE_SEND + 1];

static void send_frames(void)
{
	char bits[2000];
	int i;

	memset(went_through, 0, sizeof(went_through));
	for (i = 0; i < 10 && vc->dsp_mode == DSP_MODE_AUDIO_RX_FRAME_TX; i++) {
		amps_encode_frame_fvc(vc, bits);
		went_through[trans->state] = 1;
	}
}

static void test_ringing_answer_during_order(void)
{
	printf("Message waiting while ringing, the phone answers while it is sent\n");
	setup(TRANS_CALL_MT_ANSWER_WAIT);
	CHECK(amps_mwi(NUMBER, 3, 0) == 0, "message waiting taken while ringing");
	CHECK(trans->state == TRANS_CALL_MWI, "the order goes on the FVC");
	amps_rx_signaling_tone(vc, 0, 1.0);	/* ST off with SAT: answered */
	CHECK(trans->state == TRANS_CALL_MWI && trans->order_return_state == TRANS_CALL,
		"the answer during the order is taken: the call, the order goes again in it");
	send_frames();
	CHECK(trans->state == TRANS_CALL && vc->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX, "then the call with audio");
}


static void test_ringing_order_back(void)
{
	printf("Audit, change power and flash with info while ringing: back to ringing\n");
	setup(TRANS_CALL_MT_ANSWER_WAIT);
	CHECK(amps_audit_order(NUMBER) == 0, "audit taken while ringing");
	CHECK(trans->state == TRANS_CALL_AUDIT, "on the FVC, not the FOCC");
	send_frames();
	CHECK(trans->state == TRANS_CALL_MT_ANSWER_WAIT, "back to ringing after the audit");
	CHECK(vc->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX, "ringing: audio as before");
	CHECK(amps_change_power_order(NUMBER, 3) == 0, "change power taken while ringing");
	send_frames();
	CHECK(trans->state == TRANS_CALL_MT_ANSWER_WAIT, "back to ringing after change power");
	CHECK(amps_flash_with_info(NUMBER, "Hi", 0, 3) == 0, "flash with info taken while ringing");
	send_frames();
	CHECK(trans->state == TRANS_CALL_MT_ANSWER_WAIT, "back to ringing after flash with info");
}

static void test_alert_wait(void)
{
	printf("The alert not confirmed yet (Waiting for Order)\n");
	setup(TRANS_CALL_MT_ALERT_CONFIRM);
	vc->dsp_mode = DSP_MODE_AUDIO_RX_SILENCE_TX;
	CHECK(amps_flash_with_info(NUMBER, "Hi", 0, 3) < 0, "flash with info is not taken before it rings");
	CHECK(amps_change_power_order(NUMBER, 2) == 0, "change power taken");
	send_frames();
	CHECK(trans->state == TRANS_CALL_MT_ALERT_CONFIRM, "back to the alert wait");
	CHECK(vc->dsp_mode == DSP_MODE_AUDIO_RX_SILENCE_TX, "alert wait: silence as before");
	/* the phone confirms the alert while an order is sent: it rings */
	CHECK(amps_audit_order(NUMBER) == 0, "audit taken");
	amps_rx_signaling_tone(vc, 1, 1.0);
	CHECK(trans->state == TRANS_CALL_AUDIT && trans->order_return_state == TRANS_CALL_MT_ANSWER_WAIT,
		"the alert confirmation during the order: ringing, the order goes again in it");
	send_frames();
	CHECK(trans->state == TRANS_CALL_MT_ANSWER_WAIT && vc->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX,
		"then ringing with audio");
	/* the alert timer runs out while an order is sent: the alert again, then the order */
	setup(TRANS_CALL_MT_ALERT_CONFIRM);
	trans->alert_retry = 1;
	CHECK(amps_mwi(NUMBER, 1, 0) == 0, "message waiting taken");
	transaction_timeout(trans);
	CHECK(trans->state == TRANS_CALL_MT_ALERT, "the alert timeout during the order: the alert again");
	send_frames();
	CHECK(went_through[TRANS_CALL_MT_ALERT_SEND] && went_through[TRANS_CALL_MWI_SEND],
		"the alert, then the order is sent");
	CHECK(trans->state == TRANS_CALL_MT_ALERT_CONFIRM && vc->dsp_mode == DSP_MODE_AUDIO_RX_SILENCE_TX
		&& trans->pending_order == TRANS_NULL, "then the alert wait");
}

static void test_call(void)
{
	printf("In the conversation\n");
	setup(TRANS_CALL);
	CHECK(amps_audit_order(NUMBER) == 0, "audit taken");
	CHECK(amps_audit_order(NUMBER) < 0, "a second audit while the first is sent: busy (not on the FOCC)");
	CHECK(trans->state == TRANS_CALL_AUDIT, "the first stays on the FVC");
	send_frames();
	CHECK(trans->state == TRANS_CALL && vc->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX, "back to the call with audio");
	/* the phone's release (its tone of 1.8 s) during an order */
	CHECK(amps_change_power_order(NUMBER, 5) == 0, "change power taken");
	amps_rx_signaling_tone(vc, 1, 1.0);
	CHECK(trans->st_start_time != 0.0, "the tone during the order is timed as in the call");
}

int main(void)
{
	dsp_init();
	init_frame();

	printf("AMPS orders on the voice channel in the tasks of the mobile station\n");
	test_ringing_answer_during_order();
	test_ringing_order_back();
	test_alert_wait();
	test_call();

	if (failures) {
		printf("%d check(s) failed\n", failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
