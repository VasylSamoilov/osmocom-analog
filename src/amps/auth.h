#ifndef _AMPS_AUTH_H
#define _AMPS_AUTH_H

#include <stdint.h>

/*
 * AMPS authentication scaffold (TIA/EIA-553-A 2.3.12, "Authentication and
 * Encryption of Signaling Information / User Data").
 *
 * SCOPE: DECODE / STUB / SCAFFOLD ONLY.
 * ------------------------------------
 * This base station does NOT authenticate. There is no CAVE algorithm, no SSD
 * or A-key state, and no code path that verifies, enforces, or denies service
 * based on an authentication response. Authentication defaults to OFF
 * (AUTH is not advertised in the overhead train), and even when the auth words
 * are decoded on receipt they are only LOGGED as a stub -- the mobile is then
 * served exactly as in the no-auth case.
 *
 * This header defines the data model and the per-mobile state container so a
 * future licensed CAVE implementation has a place to attach, plus a stub-logging
 * helper. The cryptographic primitives (Auth_Signature / SSD_Generation, see
 * Figures 2.3.12-2/-3/-6) are intentionally absent and export-restricted.
 *
 * References (PNG page images are ground truth):
 *   - 2.3.12.1    Authentication                              (SSD partitioning, AUTHR)
 *   - Fig 2.3.12-6 SSD Update Message Flow                    (PDF p.30)
 *   - 3.7.1.1     FOCC order words (Unique/SSD/BS Challenge)  (PDF p.90/91)
 */

/* Per-mobile authentication state (reserved for a future CAVE implementation;
 * the scaffold itself only ever holds IDLE). */
enum amps_auth_state {
	AMPS_AUTH_IDLE = 0,	/* no authentication in progress (the only state this scaffold uses) */
	AMPS_AUTH_GLOBAL,	/* global challenge (RAND broadcast); reserved */
	AMPS_AUTH_UNIQUE_SENT,	/* Unique Challenge Order sent; reserved */
	AMPS_AUTH_SSD_UPDATE_SENT,	/* SSD Update Order sent; reserved */
	AMPS_AUTH_BS_CHALLENGE,	/* mobile issued Base Station Challenge; reserved */
};

/* Per-mobile authentication context (decode-only scaffold; carried on the
 * transaction so a future implementation has somewhere to keep state). */
typedef struct amps_auth_ctx {
	enum amps_auth_state	state;
	uint8_t			count;		/* last decoded COUNT (modulo-64), informational */
	uint32_t		authr;		/* last decoded AUTHR/AUTHU (18-bit), informational */
	uint8_t			randc;		/* last decoded RANDC (8-bit), informational */
} amps_auth_ctx;

/* Reset a context to IDLE. */
void amps_auth_ctx_init(amps_auth_ctx *ctx);

/*
 * Log a decoded authentication word as a stub. Records the fields into ctx for
 * reference and emits an INFO line making clear that authentication is NOT
 * performed (decode-only). Never verifies, enforces, or denies.
 */
void amps_auth_log_stub(amps_auth_ctx *ctx, uint32_t authr, uint8_t count, uint8_t randc);

const char *amps_auth_state_name(enum amps_auth_state state);

#endif /* _AMPS_AUTH_H */
