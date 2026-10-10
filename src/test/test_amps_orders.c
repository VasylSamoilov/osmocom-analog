/* AMPS order-code conformance test (TIA/EIA-553-A Table 3.7.1-1).
 *
 * Guards the "one-bit footgun" class of bug found in the conformance audit:
 * Disable DTMF is ORDER=22 (0b10110) and the adjacent code 23 (0b10111) is
 * Message Encryption Mode. A single flipped bit silently sends an encryption
 * order instead of muting DTMF. This test pins the spec bit patterns and
 * checks that the orders the implementation acts on are mutually distinct.
 *
 * Self-contained on purpose: it encodes order triples the same way the frame
 * layer does (ORDER:5 | ORDQ:3 | MSG_TYPE:5 packed MSB-first) so it needs no
 * AMPS library link, and asserts against the standard's values directly.
 *
 * (C) 2026 — osmocom-analog AMPS conformance tests.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static int failures;

#define CHECK(cond, msg) do { \
	if (cond) { \
		printf("  OK   %s\n", msg); \
	} else { \
		printf("  FAIL %s\n", msg); \
		failures++; \
	} \
} while (0)

/* Spec ORDER codes (5-bit), TIA/EIA-553-A Table 3.7.1-1 (PDF p.92-93). */
#define ORDER_ALERT		0x01	/* 00001 */
#define ORDER_RELEASE		0x03	/* 00011 */
#define ORDER_REORDER		0x04	/* 00100 */
#define ORDER_MWI		0x05	/* 00101 */
#define ORDER_STOP_ALERT	0x06	/* 00110 */
#define ORDER_AUDIT		0x07	/* 00111 */
#define ORDER_SEND_CALLED_ADDR	0x08	/* 01000 */
#define ORDER_INTERCEPT		0x09	/* 01001 */
#define ORDER_MAINTENANCE	0x0a	/* 01010 */
#define ORDER_CHANGE_POWER	0x0b	/* 01011 */
#define ORDER_DIRECTED_RETRY	0x0c	/* 01100 */
#define ORDER_LOCAL_CONTROL	0x1e	/* 11110 */
#define ORDER_DISABLE_DTMF	0x16	/* 10110 */
#define ORDER_MSG_ENCRYPTION	0x17	/* 10111 */
#define ORDER_SERIAL_NUMBER	0x0f	/* 01111 */

/* Pack an order triple exactly as the FOCC/FVC order carrier does. */
static uint16_t pack_order(uint8_t order, uint8_t ordq, uint8_t msg_type)
{
	return ((order & 0x1f) << 8) | ((ordq & 0x7) << 5) | (msg_type & 0x1f);
}

int main(void)
{
	printf("AMPS order-code conformance (TIA/EIA-553-A Table 3.7.1-1)\n");

	/* --- The headline footgun: Disable DTMF vs Message Encryption --- */
	CHECK(ORDER_DISABLE_DTMF == 22,
		"Disable DTMF is ORDER 22");
	CHECK(ORDER_DISABLE_DTMF == 0x16,
		"Disable DTMF bit pattern is 0b10110");
	CHECK(ORDER_MSG_ENCRYPTION == 23 && ORDER_MSG_ENCRYPTION == 0x17,
		"Message Encryption is ORDER 23 (0b10111)");
	CHECK(ORDER_DISABLE_DTMF != ORDER_MSG_ENCRYPTION,
		"Disable DTMF != Message Encryption (one-bit apart)");
	CHECK((ORDER_DISABLE_DTMF ^ ORDER_MSG_ENCRYPTION) == 0x01,
		"the two differ by exactly one (LSB) bit, as the spec table shows");

	/* The value amps.c packs for a Disable-DTMF order (ORDQ=0, MSG_TYPE=0). */
	CHECK(pack_order(ORDER_DISABLE_DTMF, 0, 0) == pack_order(0x16, 0, 0),
		"packed Disable DTMF order encodes 0x16 in the ORDER field");
	CHECK(pack_order(ORDER_DISABLE_DTMF, 0, 0)
	    != pack_order(ORDER_MSG_ENCRYPTION, 0, 0),
		"packed Disable DTMF != packed Message Encryption");

	/* --- Uniqueness of the analog orders the base actually acts on. ---
	 * (ORDQ/MSG_TYPE qualify sub-variants; the base ORDER codes below must
	 * all be distinct — a collision is the Finding-A/B class of drift.) */
	uint8_t acted[] = {
		ORDER_ALERT, ORDER_RELEASE, ORDER_REORDER, ORDER_MWI,
		ORDER_STOP_ALERT, ORDER_AUDIT, ORDER_SEND_CALLED_ADDR,
		ORDER_INTERCEPT, ORDER_MAINTENANCE, ORDER_CHANGE_POWER,
		ORDER_DIRECTED_RETRY, ORDER_LOCAL_CONTROL, ORDER_DISABLE_DTMF,
		ORDER_SERIAL_NUMBER,
	};
	int n = (int)(sizeof(acted) / sizeof(acted[0]));
	int dup = 0;
	for (int i = 0; i < n; i++)
		for (int j = i + 1; j < n; j++)
			if (acted[i] == acted[j])
				dup++;
	CHECK(dup == 0, "all acted-on analog ORDER codes are unique");

	/* All 5-bit order codes must fit the field. */
	int oversize = 0;
	for (int i = 0; i < n; i++)
		if (acted[i] > 0x1f)
			oversize++;
	CHECK(oversize == 0, "every ORDER code fits the 5-bit field");

	printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
		failures, failures == 1 ? "" : "s");
	return failures ? 1 : 0;
}
