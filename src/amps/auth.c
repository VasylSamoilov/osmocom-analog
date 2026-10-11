/* AMPS authentication scaffold (TIA/EIA-553-A 2.3.12).
 *
 * See auth.h for scope. This file implements the message-flow skeleton and the
 * dispatch to a pluggable CAVE signature provider. The cryptographic primitives
 * themselves (CAVE / SSD_Generation_Procedure) are intentionally absent and must
 * be supplied by a licensed provider via amps_auth_register_provider().
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "auth.h"

static const struct amps_auth_provider *g_provider = NULL;

void amps_auth_register_provider(const struct amps_auth_provider *provider)
{
	g_provider = provider;
}

int amps_auth_available(void)
{
	return (g_provider && g_provider->auth_signature) ? 1 : 0;
}

void amps_auth_ctx_init(amps_auth_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->state = AMPS_AUTH_IDLE;
	ctx->last_result = AMPS_AUTH_RESULT_UNAVAILABLE;
}

enum amps_auth_result amps_auth_verify_response(amps_auth_ctx *ctx,
	uint32_t rand_val, uint32_t esn, uint32_t min1, uint32_t received)
{
	uint32_t expected = 0;

	/* No CAVE provider -> we cannot compute the expected signature, so we
	 * cannot authenticate. Fail per policy (do NOT silently pass). */
	if (!amps_auth_available()) {
		ctx->last_result = AMPS_AUTH_RESULT_UNAVAILABLE;
		ctx->state = AMPS_AUTH_FAILED;
		return AMPS_AUTH_RESULT_UNAVAILABLE;
	}

	if (!g_provider->auth_signature(rand_val, esn, min1, ctx->ssd_a, &expected)) {
		ctx->last_result = AMPS_AUTH_RESULT_UNAVAILABLE;
		ctx->state = AMPS_AUTH_FAILED;
		return AMPS_AUTH_RESULT_UNAVAILABLE;
	}

	/* AUTHR/AUTHU is an 18-bit field. */
	expected &= 0x3ffff;
	received &= 0x3ffff;

	if (expected == received) {
		ctx->last_result = AMPS_AUTH_RESULT_PASS;
		ctx->state = AMPS_AUTH_SUCCESS;
		return AMPS_AUTH_RESULT_PASS;
	}

	ctx->last_result = AMPS_AUTH_RESULT_FAIL;
	ctx->state = AMPS_AUTH_FAILED;
	return AMPS_AUTH_RESULT_FAIL;
}

enum amps_auth_result amps_auth_compute_authbs(amps_auth_ctx *ctx,
	uint32_t randbs, uint32_t esn, uint32_t min1, uint32_t *authbs18)
{
	uint32_t out = 0;

	*authbs18 = 0;

	if (!amps_auth_available()) {
		ctx->last_result = AMPS_AUTH_RESULT_UNAVAILABLE;
		return AMPS_AUTH_RESULT_UNAVAILABLE;
	}

	/* AUTHBS is Auth_Signature over RANDBS with the NEW SSD-A (Fig 2.3.12-6). */
	if (!g_provider->auth_signature(randbs, esn, min1, ctx->ssd_a, &out)) {
		ctx->last_result = AMPS_AUTH_RESULT_UNAVAILABLE;
		return AMPS_AUTH_RESULT_UNAVAILABLE;
	}

	*authbs18 = out & 0x3ffff;
	ctx->last_result = AMPS_AUTH_RESULT_PASS;
	return AMPS_AUTH_RESULT_PASS;
}

const char *amps_auth_state_name(enum amps_auth_state state)
{
	switch (state) {
	case AMPS_AUTH_IDLE:			return "IDLE";
	case AMPS_AUTH_GLOBAL:			return "GLOBAL-CHALLENGE";
	case AMPS_AUTH_UNIQUE_SENT:		return "UNIQUE-CHALLENGE-SENT";
	case AMPS_AUTH_SSD_UPDATE_SENT:		return "SSD-UPDATE-SENT";
	case AMPS_AUTH_BS_CHALLENGE:		return "BS-CHALLENGE";
	case AMPS_AUTH_SUCCESS:			return "SUCCESS";
	case AMPS_AUTH_FAILED:			return "FAILED";
	}
	return "?";
}

const char *amps_auth_result_name(enum amps_auth_result result)
{
	switch (result) {
	case AMPS_AUTH_RESULT_PASS:		return "PASS";
	case AMPS_AUTH_RESULT_FAIL:		return "FAIL";
	case AMPS_AUTH_RESULT_UNAVAILABLE:	return "UNAVAILABLE (no CAVE provider)";
	}
	return "?";
}
