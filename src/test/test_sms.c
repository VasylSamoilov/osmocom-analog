#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../nmt/nmt.h"

static const uint8_t test_mo_sms_data1[] = {
	0x00, 0x00, 0x00, 0xa1, 0x41, 0x0f, 0x11,
	0x00, 0x04, 0xa1, 0x8a, 0x51,
	0x00, 0x00, 0xff, 0x05, 0xc8, 0x20, 0x93,
	0xf9, 0x7c,
};
static const uint8_t test_mo_sms_data2[] = {
	0x00, 0x02, 0x07, 0xa1, 0xa9, 0x62, 0x65,
	0xf4, 0x41, 0x10, 0x11, 0x02, 0x03, 0xa1,
	0x21, 0xf3,
	0x00, 0x30, 0xff, 0x06, 0x48, 0x61, 0x6c,
	0x6c, 0x6f, 0x21,
};

static const char *test_mo_sms_text1 = "HALLO";
static const char *test_mo_sms_text2 = "Hallo!";

static const char *test_mt_sms_text = "Moin Moin";
static const char *test_mt_sms_tel = "4948416068";
static time_t test_mt_sms_time = 851430904;

static const uint8_t test_mt_sms_data[] = {
	0x01, 0x18, 0x53, 0x4d, 0x53, 0x48, 0x18, 0x41, 0x42, 0x43, 0x02,
	0x01,
	0x01,
	0x41,
	0x1a,
	0x04,
	0x0a, 0x91, 0x94, 0x84, 0x14, 0xa6, 0x86,
	0x00,
	0x00,
	0x69, 0x21, 0x42, 0x21, 0x53, 0x4a, 0x00,
	0x09, 0xcd, 0x77, 0xda, 0x0d, 0x6a, 0xbe, 0xd3, 0x6e,
};

static void assert(int condition, char *why)
{
	printf("%s = %s\n", why, (condition) ? "TRUE" : "FALSE");

	if (!condition) {
		printf("\n******************** FAILED ********************\n\n");
		exit(-1);
	}
}

static void ok(void)
{
	printf("\n OK ;->\n\n");
	sleep(1);
}

static uint8_t dms_buffer[256];
static int dms_buffer_count;
static int expect_reference_pdu;	/* assert against test_mt_sms_data only for the 7-bit reference deliver */
void dms_send(nmt_t __attribute__((unused)) *nmt, const uint8_t *data, int length, int __attribute__((unused)) eight_bits)
{
	int i;

	/* skip deliver report */
	if (length == 13)
		return;

	dms_buffer_count = length;
	memcpy(dms_buffer, data, length);

	if (!expect_reference_pdu)
		return;

	assert(length == sizeof(test_mt_sms_data), "Expecting SMS binary data length to match");
	for (i = 0; i < length; i++) {
		if (data[i] != test_mt_sms_data[i])
			printf("offset: %d  got: 0x%02x  expecting: 0x%02x\n", i, data[i], test_mt_sms_data[i]);
	}
	assert(!memcmp(data, test_mt_sms_data, length), "Expecting SMS binary data to match");
}

void sms_release(nmt_t __attribute__((unused)) *nmt)
{
	printf("(got release from SMS layer)\n");
}

int sms_submit(nmt_t __attribute__((unused)) *nmt, uint8_t __attribute__((unused)) ref, const char __attribute__((unused)) *orig_address, uint8_t __attribute__((unused)) orig_type, uint8_t __attribute__((unused)) orig_plan, int __attribute__((unused)) msg_ref, const char __attribute__((unused)) *dest_address, uint8_t __attribute__((unused)) dest_type, uint8_t __attribute__((unused)) dest_plan, const char *message, int __attribute__((unused)) msg_len)
{
	strcpy((char *)dms_buffer, message);
	dms_buffer_count = strlen(message);

	return 0;
}

void sms_deliver_report(nmt_t __attribute__((unused)) *nmt, uint8_t __attribute__((unused)) ref, int __attribute__((unused)) error, uint8_t __attribute__((unused)) cause)
{
	printf("(got deliver report from SMS layer)\n");
}

extern void main_mobile_loop();

int main(void)
{
	nmt_t *nmt;
	int i;
	int rc;

	/* this is never called, it forces the linker to add mobile functions */
	if (loglevel == -1000) main_mobile_loop();

	loglevel = LOGL_DEBUG;
	logging_init();

	nmt = calloc(1, sizeof(*nmt));
	sms_init_sender(nmt);
	sms_reset(nmt);

	/* deliver */
	printf("(delivering SMS)\n");
	expect_reference_pdu = 1;
	rc = sms_deliver(nmt, 1, test_mt_sms_tel, SMS_TYPE_INTERNATIONAL, SMS_PLAN_ISDN_TEL, test_mt_sms_time, 0, test_mt_sms_text, (int)strlen(test_mt_sms_text));
	expect_reference_pdu = 0;
	assert(rc == 0, "Expecting sms_deliver() to return 0");

	sms_cleanup_sender(nmt);
	free(nmt);

	ok();

	/* deliver an 8-bit (NMT alphabet) message: contains a byte >= 0x80, so
	 * sms_deliver must pick TP-DCS 0x30 and store one octet per character. */
	nmt = calloc(1, sizeof(*nmt));
	sms_init_sender(nmt);
	sms_reset(nmt);

	printf("(delivering SMS with 8-bit characters)\n");
	{
		/* "Hi" + two national letters (0xC0, 0xE4) */
		const char eight_bit_text[] = { 'H', 'i', (char)0xc0, (char)0xe4, '\0' };
		int tlen = (int)strlen(eight_bit_text);
		int k, dcs_pos = -1;

		dms_buffer_count = 0;
		rc = sms_deliver(nmt, 2, test_mt_sms_tel, SMS_TYPE_INTERNATIONAL, SMS_PLAN_ISDN_TEL, test_mt_sms_time, 0, eight_bit_text, tlen);
		assert(rc == 0, "Expecting 8-bit sms_deliver() to return 0");

		/* the DELIVER PDU: ...TP-PID(0) TP-DCS TP-SCTS(7) TP-UDL TP-UD...
		 * find TP-DCS = 0x30 and verify the 8-bit UD carries the raw bytes. */
		for (k = 0; k + 1 + 7 + 1 + tlen <= dms_buffer_count; k++) {
			if (dms_buffer[k] == 0x30
			 && dms_buffer[k + 1 + 7] == tlen
			 && dms_buffer[k + 1 + 7 + 1]     == (uint8_t)'H'
			 && dms_buffer[k + 1 + 7 + 1 + 2] == 0xc0
			 && dms_buffer[k + 1 + 7 + 1 + 3] == 0xe4) {
				dcs_pos = k;
				break;
			}
		}
		assert(dcs_pos >= 0, "Expecting 8-bit DELIVER with DCS 0x30 and one octet per character");
	}

	sms_cleanup_sender(nmt);
	free(nmt);

	ok();

	nmt = calloc(1, sizeof(*nmt));
	sms_init_sender(nmt);
	sms_reset(nmt);

	printf("(submitting SMS 7-bit encoded)\n");
	dms_buffer_count = 0;
	for (i = 0; i < (int)sizeof(test_mo_sms_data1); i++)
		dms_receive(nmt, test_mo_sms_data1 + i, 1, 1);

	assert(dms_buffer_count == (int)strlen(test_mo_sms_text1), "Expecting SMS text length to match");
	assert(!memcmp(dms_buffer, test_mo_sms_text1, dms_buffer_count), "Expecting SMS text to match");

	sms_cleanup_sender(nmt);
	free(nmt);

	ok();

	nmt = calloc(1, sizeof(*nmt));
	sms_init_sender(nmt);
	sms_reset(nmt);

	printf("(submitting SMS 8-bit encoded)\n");
	dms_buffer_count = 0;
	for (i = 0; i < (int)sizeof(test_mo_sms_data2); i++)
		dms_receive(nmt, test_mo_sms_data2 + i, 1, 1);

	assert(dms_buffer_count == (int)strlen(test_mo_sms_text2), "Expecting SMS text length to match");
	assert(!memcmp(dms_buffer, test_mo_sms_text2, dms_buffer_count), "Expecting SMS text to match");

	sms_cleanup_sender(nmt);
	free(nmt);

	ok();

	return 0;
}

void call_down_clock(void) {}

const char *aaimage[] = { NULL };

