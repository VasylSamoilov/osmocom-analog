/* AMPS authentication scaffold (TIA/EIA-553-A 2.3.12) -- DECODE / STUB ONLY.
 *
 * See auth.h for scope. This base does not authenticate: no CAVE, no SSD/A-key
 * state, no verify/enforce/deny path. The functions here only initialise the
 * per-mobile context and LOG a decoded authentication word as a stub.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../liblogging/logging.h"
#include "auth.h"

void amps_auth_ctx_init(amps_auth_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->state = AMPS_AUTH_IDLE;
}

void amps_auth_log_stub(amps_auth_ctx *ctx, uint32_t authr, uint8_t count, uint8_t randc)
{
	if (ctx) {
		ctx->authr = authr & 0x3ffff;
		ctx->count = count;
		ctx->randc = randc;
	}
	/* Decode-only: record and report for DEBUG, never verify or enforce.
	 * Authentication cannot be implemented properly here (no CAVE), so this
	 * exists purely so the decoded auth word is visible when debugging. */
	LOGP(DAMPS, LOGL_DEBUG, "Authentication (stub, decode-only): AUTHR=0x%05x COUNT=%d RANDC=0x%02x -- not supported by this base, not verified\n",
		authr & 0x3ffff, count, randc);
}

const char *amps_auth_state_name(enum amps_auth_state state)
{
	switch (state) {
	case AMPS_AUTH_IDLE:			return "IDLE";
	case AMPS_AUTH_GLOBAL:			return "GLOBAL-CHALLENGE";
	case AMPS_AUTH_UNIQUE_SENT:		return "UNIQUE-CHALLENGE-SENT";
	case AMPS_AUTH_SSD_UPDATE_SENT:		return "SSD-UPDATE-SENT";
	case AMPS_AUTH_BS_CHALLENGE:		return "BS-CHALLENGE";
	}
	return "?";
}
