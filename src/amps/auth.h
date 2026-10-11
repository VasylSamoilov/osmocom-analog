#ifndef _AMPS_AUTH_H
#define _AMPS_AUTH_H

#include <stdint.h>

/*
 * AMPS authentication scaffold (TIA/EIA-553-A 2.3.12, "Authentication and
 * Encryption of Signaling Information / User Data").
 *
 * SCOPE OF THIS SCAFFOLD
 * ----------------------
 * This lays down the DATA MODEL, the per-mobile authentication STATE, the
 * message-FLOW skeleton (challenge orders in, response verification, result)
 * and a PLUGGABLE signature boundary. It deliberately does NOT implement the
 * CAVE algorithm or the SSD_Generation_Procedure: CAVE is export-restricted
 * and is not published in TIA/EIA-553-A (the standard references it as an
 * external primitive). The two primitives the whole flow pivots on --
 * Auth_Signature() and SSD_Generation_Procedure() (see Figures 2.3.12-2,
 * 2.3.12-3, 2.3.12-6) -- are exposed here as function pointers so a licensed
 * CAVE implementation can be registered at build/integration time without
 * touching the flow code. With no provider registered the flow runs but every
 * signature is reported UNAVAILABLE, so authentication fails per policy rather
 * than silently passing.
 *
 * References (PNG page images are ground truth):
 *   - 2.3.12.1    Authentication                              (SSD partitioning, AUTHR)
 *   - Fig 2.3.12-2 Computation of AUTHR for MS Registrations  (PDF p.23)
 *   - Fig 2.3.12-3 Computation of AUTHU (Unique Challenge)    (PDF p.24)
 *   - Fig 2.3.12-6 SSD Update Message Flow                    (PDF p.30)
 *   - 3.7.1.1     FOCC order words (Unique/SSD/BS Challenge)  (PDF p.90/91)
 */

/* Per-mobile authentication process state (one in-flight challenge at a time). */
enum amps_auth_state {
	AMPS_AUTH_IDLE = 0,	/* no authentication in progress */
	AMPS_AUTH_GLOBAL,	/* global challenge (RAND broadcast); AUTHR expected with next access */
	AMPS_AUTH_UNIQUE_SENT,	/* Unique Challenge Order sent, awaiting AUTHU */
	AMPS_AUTH_SSD_UPDATE_SENT,	/* SSD Update Order sent, awaiting Base Station Challenge */
	AMPS_AUTH_BS_CHALLENGE,	/* mobile issued Base Station Challenge, AUTHBS owed to mobile */
	AMPS_AUTH_SUCCESS,	/* last challenge verified */
	AMPS_AUTH_FAILED,	/* last challenge failed / unavailable */
};

/* Result of a response verification. */
enum amps_auth_result {
	AMPS_AUTH_RESULT_PASS = 0,	/* received response matched the locally computed one */
	AMPS_AUTH_RESULT_FAIL,		/* mismatch -- authentication failed */
	AMPS_AUTH_RESULT_UNAVAILABLE,	/* no signature provider -> cannot decide (fail-closed by policy) */
};

/*
 * Pluggable signature boundary (the CAVE-based primitives).
 *
 * A licensed implementation registers these via amps_auth_register_provider().
 * Semantics mirror the standard's figures:
 *
 *   auth_signature():  AUTHR/AUTHU/AUTHBS = f(rand, esn, min1, ssd_a)
 *                      'rand_val' is RAND (global), RANDU (unique) or RANDBS
 *                      (BS challenge) depending on the invoking order. Returns
 *                      the 18-bit response in *out18. Non-zero return = computed.
 *
 *   ssd_generation():  (SSD-A_NEW, SSD-B_NEW) = f(a_key, randssd, esn).
 *                      Fills ssd_a_new/ssd_b_new (each 64-bit here as opaque
 *                      storage). Non-zero return = computed.
 */
struct amps_auth_provider {
	const char *name;
	int (*auth_signature)(uint32_t rand_val, uint32_t esn, uint32_t min1,
			      uint64_t ssd_a, uint32_t *out18);
	int (*ssd_generation)(uint64_t a_key, uint64_t randssd, uint32_t esn,
			      uint64_t *ssd_a_new, uint64_t *ssd_b_new);
};

/* Per-mobile authentication context (lives on the transaction). */
typedef struct amps_auth_ctx {
	enum amps_auth_state	state;
	uint8_t			count;		/* modulo-64 call-history COUNT (2.3.12) */
	uint32_t		rand_sent;	/* RAND/RANDU last issued to this mobile */
	uint64_t		randssd_sent;	/* RANDSSD last issued (SSD Update) */
	uint32_t		randbs;		/* RANDBS from the mobile's BS Challenge */
	uint64_t		ssd_a;		/* current SSD-A (signature key) -- opaque */
	uint64_t		ssd_b;		/* current SSD-B (voice/signaling privacy) -- opaque */
	enum amps_auth_result	last_result;
} amps_auth_ctx;

/* Register the CAVE signature provider. Pass NULL to clear (back to unavailable). */
void amps_auth_register_provider(const struct amps_auth_provider *provider);

/* Is a signature provider registered? */
int amps_auth_available(void);

/* Reset a context to IDLE. */
void amps_auth_ctx_init(amps_auth_ctx *ctx);

/*
 * Verify a mobile's authentication response (AUTHR for a global/registration
 * challenge, AUTHU for a Unique Challenge). Computes the expected value via the
 * registered provider and compares. Returns PASS / FAIL / UNAVAILABLE and
 * records it in ctx->last_result + ctx->state.
 *   rand_val: the RAND or RANDU that produced this response.
 *   received: the 18-bit AUTHR/AUTHU received from the mobile.
 */
enum amps_auth_result amps_auth_verify_response(amps_auth_ctx *ctx,
	uint32_t rand_val, uint32_t esn, uint32_t min1, uint32_t received);

/*
 * Compute the base station's AUTHBS answer to a mobile-issued Base Station
 * Challenge (Fig 2.3.12-6). Fills *authbs18. Returns a result code; on
 * UNAVAILABLE *authbs18 is 0 and the base cannot complete the SSD update.
 */
enum amps_auth_result amps_auth_compute_authbs(amps_auth_ctx *ctx,
	uint32_t randbs, uint32_t esn, uint32_t min1, uint32_t *authbs18);

const char *amps_auth_state_name(enum amps_auth_state state);
const char *amps_auth_result_name(enum amps_auth_result result);

#endif /* _AMPS_AUTH_H */
