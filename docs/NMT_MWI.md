# NMT Message Waiting Indicator (MWI) Implementation

## Overview

This document describes the Message Waiting Indicator (MWI) feature implementation
for the NMT 450 system, based on section 4.3.3.11 of NMT DOC 450-1.

## NMT Frame 5c - Message Waiting Indicator

### Frame Structure

Frame 5c format: `NNNPYYZXXXXXXFFF`

| Field | Digits | Description |
|-------|--------|-------------|
| NNN | 3 | Channel number (encoded with power level) |
| P | 1 | Prefix = 2 (identifies frame type 5c) |
| YY | 2 | Traffic area |
| Z | 1 | Mobile subscriber country code |
| XXXXXX | 6 | Mobile subscriber number |
| FFF | 3 | Waiting indicator flags (12 bits) |

### FFF Field - Waiting Indicators

The FFF field contains 12 bits structured as: `i1 i2 i3 i4 i5 i6 i'1 i'2 i'3 i'4 i'5 i'6`

Bit order is **MSB-first** (i1 is the most significant bit).
The i' bits are a repetition of i bits for error checking (frame acceptance criteria).

| Indicator | i bit | i' bit | Command Flag |
|-----------|-------|--------|--------------|
| SMS       | i1 (bit 11) | i'1 (bit 5) | 0x01 |
| Voice mail| i2 (bit 10) | i'2 (bit 4) | 0x02 |
| Fax       | i3 (bit 9)  | i'3 (bit 3) | 0x04 |
| E-mail    | i4 (bit 8)  | i'4 (bit 2) | 0x08 |
| Data      | i5 (bit 7)  | i'5 (bit 1) | 0x10 |
| Spare     | i6 (bit 6)  | i'6 (bit 0) | 0x20 |

**Example:** To set SMS indicator, command flag 0x01 encodes to waiting_info 0x820.

## Indicator SET vs RESET Mechanism

### Frame 5c Alone (SET operation)

When frame 5c is sent alone during a call:
- `bit = 1` → Turn ON indicator
- `bit = 0` → **No effect** (does NOT reset indicator)

This is stated in NMT DOC 450-1 section 4.3.3.11:
> "Note that iₙ = 0 carries no information on the corresponding indicator setting
> (does not reset the indicator)"

### Frame 5c + Clearing Signal (UPDATE operation)

When frame 5c is used with clearing signals L(13) or L(15):
- `bit = 1` → Turn ON indicator  
- `bit = 0` → Turn OFF indicator (RESET)

From NMT DOC 450-1 section 4.4.1.16 "Clearing sequences with message indicators":
> "Release-guard is sent AND message flag information is received by the MS in frame 5c"

**Key insight**: Frame 5c must be sent **before** the L(15) clearing signal.

### Clearing Signals

| Line Signal | Code | Meaning |
|-------------|------|---------|
| L(13) | 1101 | Clearing, call transfer activated |
| L(15) | 1111 | Clearing, call transfer NOT activated |

Both signals trigger the MS to read frame 5c and update ALL indicators.

### Clearing Sequence Pattern

Per NMT DOC 450-1, Section 4.4.1.16 "Clearing sequences with message indicators":

```
MTX->BS: 5c|5c|5a(L=13, 15)|5a(L=13, 15)|5a(L=13, 15)|5a(L=13, 15)|6|6|6|6|20(A=12)
```

Simplified sequence (this implementation):
```
5c, 5c, L(15), L(15), L(15), L(15)
```

**Frame order is critical:**
1. Frame 5c is sent FIRST (2 times) - delivers the MWI data to the phone
2. L(15) clearing signal is sent AFTER (4 times) - triggers the phone to:
   - Apply the MWI flags from frame 5c (both SET and RESET)
   - Clear the call transfer indicator
   - Release the call

The phone will respond with L(1) "Release guard" to acknowledge the clearing.

## Control Socket Interface

### Socket Path

`/tmp/nmt_mwi`

This is a named pipe (FIFO) similar to the SMS delivery socket.

### Command Format

```
<subscriber>,set,<indicators>
```

Where:
- `<subscriber>`: 7-digit NMT subscriber number
- `<indicators>`: Space-separated text values OR numeric bitmask (0-31)

### Indicator Text Values

**Note: Text values must be lowercase.**

| Text | Bit | Hex | Description |
|------|-----|-----|-------------|
| `sms` | 0 | 0x01 | SMS message waiting |
| `voice` | 1 | 0x02 | Voice mail waiting |
| `fax` | 2 | 0x04 | Fax waiting |
| `email` | 3 | 0x08 | E-mail waiting |
| `data` | 4 | 0x10 | Data waiting |

### How It Works

The `set` command turns ON the specified indicators:

1. **Out-of-call**: Pages subscriber, sends frame 5c (2x), then releases with L(15) (4x)
2. **In-call**: Sends frame 5c (2x) immediately, accumulates flags in `mwi_pending`

When **any call ends** (phone hangs up or network releases), if MWI flags were set during
the call (`mwi_pending != 0`), a special clearing sequence is sent instead of normal release:

```
5c, 5c, L(15), L(15), L(15), L(15)   ← ACTIVE_STATE_MWI_CLEAR
```

This applies all accumulated flags per NMT spec 4.4.1.16.

### Example Commands

```bash
# Set SMS indicator (new SMS arrived) - text format
echo "3735859,set,sms" > /tmp/nmt_mwi

# Set SMS indicator - numeric format (bit 0 = 1)
echo "3735859,set,1" > /tmp/nmt_mwi

# Set SMS + voicemail - text format
echo "3735859,set,sms voice" > /tmp/nmt_mwi

# Set SMS + voicemail - numeric format (bits 0+1 = 3)
echo "3735859,set,3" > /tmp/nmt_mwi

# Set all indicators
echo "3735859,set,31" > /tmp/nmt_mwi
echo "3735859,set,sms voice fax email data" > /tmp/nmt_mwi
```

## Implementation Details

### Delivery Modes

MWI can be delivered in two modes:

| Mode | Condition | Behavior |
|------|-----------|----------|
| **In-call** | Subscriber has active call | Send frame 5c immediately, accumulate flags |
| **Out-of-call** | Subscriber is idle | Page subscriber, send 5c + clearing sequence |

### Call Flow for Out-of-Call MWI Delivery

```
 1. Parse command from /tmp/nmt_mwi
 2. Check if subscriber has active call → if yes, use in-call mode
 3. Create transaction with MWI parameters (mwi_call=1)
 4. Page subscriber on calling channel
 5. Wait for call acknowledgment
 6. Assign traffic channel
 7. Wait for identity
 8. Send autoanswer order
 9. Enter active state (ACTIVE_STATE_MWI)
10. Send frame 5c (2 frames) - per NMT spec 4.4.1.16
11. Call nmt_release() which sends L(15) clearing (4 frames)
12. Wait for phone to respond with L(1) release guard
13. Done - indicators are SET on phone
14. Destroy transaction
```

**Note:** For out-of-call `set` command, since frame 5c alone cannot RESET indicators
(bit=0 carries no information), only SET is effective. The phone will turn ON the
specified indicators while keeping other indicators unchanged.

### Call Flow for In-Call MWI Delivery

```
 1. Parse command from /tmp/nmt_mwi
 2. Find active call for subscriber
 3. Switch to MWI active state (ACTIVE_STATE_MWI)
 4. Send frame 5c (2 frames) - immediate notification
 5. Accumulate flags in trans->mwi_pending
 6. Return to ACTIVE_STATE_VOICE, DSP_MODE_AUDIO
 7. Call continues normally...
 8. When call ends (phone or network hangup):
    a. Check if mwi_pending is set
    b. If yes: switch to ACTIVE_STATE_MWI_CLEAR
    c. Send frame 5c (2 frames) with accumulated flags
    d. Send L(15) clearing (4 frames)
    e. Phone applies ALL MWI flags (both SET and RESET)
 9. Wait for L(1) release guard
10. Destroy transaction
```

**Important:** The in-call delivery (step 4) only turns ON indicators immediately.
To properly SET/RESET all indicators, the clearing sequence at call end (step 8) is used.
This is per NMT spec - only the clearing sequence triggers full indicator update.

### Transaction Fields

```c
/* MWI-specific fields in transaction_t */
int      mwi_call;          /* 1 = this is an MWI delivery call (out-of-call) */
uint8_t  mwi_flags;         /* indicator flags currently being sent in frame 5c */
uint8_t  mwi_pending;       /* accumulated flags from in-call MWI commands */
```

**Field usage:**
- `mwi_call`: Set to 1 for out-of-call MWI delivery. Used to identify MWI-only calls.
- `mwi_flags`: Active flags being transmitted in frame 5c. Updated before each send.
- `mwi_pending`: Accumulates flags ORed from all in-call `set` commands. When call
  ends, these flags are copied to `mwi_flags` and sent in the clearing sequence.

### Frame 5c waiting_info Encoding

The `waiting_info` field in frame_t is a 12-bit value with MSB-first bit order.
Each indicator bit is duplicated for error checking (i1-i6 and i'1-i'6):

```c
/* Per NMT spec, bit order is MSB-first:
 * bit 11 = i1 (SMS),    bit 5 = i'1
 * bit 10 = i2 (voice),  bit 4 = i'2
 * bit 9  = i3 (fax),    bit 3 = i'3
 * bit 8  = i4 (email),  bit 2 = i'4
 * bit 7  = i5 (data),   bit 1 = i'5
 * bit 6  = i6 (spare),  bit 0 = i'6
 */
uint16_t info = 0;
if (flags & 0x01) info |= 0x820;  /* SMS:   bit 11 + bit 5 */
if (flags & 0x02) info |= 0x410;  /* voice: bit 10 + bit 4 */
if (flags & 0x04) info |= 0x208;  /* fax:   bit 9  + bit 3 */
if (flags & 0x08) info |= 0x104;  /* email: bit 8  + bit 2 */
if (flags & 0x10) info |= 0x082;  /* data:  bit 7  + bit 1 */
frame->waiting_info = info;
```

## Clearing Sequence Details

### Why Clearing Sequence is Needed

Frame 5c alone can only **turn ON** indicators. Per NMT spec section 4.3.3.11:
> "Note that iₙ = 0 carries no information on the corresponding indicator setting
> (does not reset the indicator)"

To **turn OFF** indicators, the clearing sequence is required. The L(15) clearing
signal tells the phone to read frame 5c and apply ALL bits - both 1s (ON) and 0s (OFF).

### Clearing Sequence Format

```
┌─────────────────────────────────────────────────────────────────────┐
│  Frame 5c (x2)  │  L(15) clearing (x4)  │  Wait L(1)  │  Release   │
│  (MWI data)     │  (triggers apply)     │  (phone ack)│            │
└─────────────────────────────────────────────────────────────────────┘
```

### Practical Implications

| Scenario | What Happens |
|----------|--------------|
| `set sms` out-of-call | SMS ON, others unchanged (5c then L(15)) |
| `set sms` in-call | SMS ON immediately, others unchanged |
| Call ends after in-call MWI | All accumulated flags applied via clearing |
| Phone doesn't support MWI | Frame 5c ignored, call proceeds normally |

### Multiple In-Call MWI Commands

If multiple `set` commands are issued during a call, flags are ORed together:

```bash
echo "3735859,set,sms" > /tmp/nmt_mwi    # mwi_pending = 0x01
echo "3735859,set,voice" > /tmp/nmt_mwi  # mwi_pending = 0x03 (0x01 | 0x02)
# When call ends: clearing sequence sent with flags=0x03
```

## Call Transfer Indicator

The clearing signals L(13) and L(15) also control the "call transfer" indicator:

- **L(13)**: Sets call transfer indicator ON (call was forwarded)
- **L(15)**: Sets call transfer indicator OFF (normal call)

For MWI operations, we use L(15) since MWI updates are not related to call forwarding.

## Phone Compatibility

### MWI Support Detection

**There is no way to detect if a phone supports MWI before sending.**

The NMT protocol has no capability exchange mechanism. Mobile stations are categorized as:
- Ordinary mobile stations
- Coin-box mobile stations
- Data mobile stations (DMS)
- Mobile stations with enhanced functions (MWI, CLI, SMS)

This is a manufacturing/type-approval distinction (NMT 450i specification), not discoverable at runtime.

### Behavior with Unsupported Phones

From NMT DOC 450-1, Section 4.7.2:

> *"Generally, frames that cannot be interpreted by the logic, shall be ignored.
> This applies also to frames that have no meaning in an actual signalling sequence."*

**Result: Unsupported phones simply ignore frame 5c - no error, no crash.**

| Scenario | Phone Behavior |
|----------|----------------|
| MWI to phone WITH support | Updates indicators |
| MWI to phone WITHOUT support | Ignores frame 5c, call proceeds normally |

## Acknowledgment Behavior

### Frame 5c Has No Acknowledgment

The MWI frame (5c) has **no defined acknowledgment** in the NMT protocol.
The phone processes it silently or ignores it - we cannot tell which.

### Clearing Signal Acknowledgment

For MWI delivery (both out-of-call and in-call at call end):

| Signal | Direction | Purpose |
|--------|-----------|---------|
| L(15) | MTX→MS | Clearing, triggers indicator update |
| L(1) | MS→MTX | Release guard (acknowledges clearing) |

**L(1) confirms the phone received the clearing signal, but NOT that MWI was processed.**

### What We Can Detect

| Scenario | Observable | Logged As |
|----------|------------|-----------|
| Out-of-call completes | Receive L(1) | "MWI delivery confirmed" |
| Out-of-call timeout | No L(1) received | "MWI delivery timeout" |
| In-call MWI | Call continues | "MWI sent (in-call)" |

### Logging Examples

**Successful out-of-call delivery:**
```
MWI set '3735859': sms (0x01)
MWI out-of-call to '3735859': sms (0x01)
Sending MWI frame 5c.
Send MWI frame 5c: flags=0x01 info=0x820 (sms )
MWI delivery complete, releasing.
MWI delivered!
Send release.
```

**In-call delivery with clearing at call end:**
```
MWI: subscriber '3735859' has active call, sending in-call
MWI in-call to '3735859': sms (0x01)
Sending MWI frame 5c.
MWI in-call complete, returning to voice.
MWI in-call: sms (0x01)
... (call continues) ...
Received clearing by mobile phone in state ACTIVE.
MWI pending (0x01), sending clearing sequence.
Sending MWI clearing: 5c x2 then L(15) x4 (flags=0x01).
MWI clearing sequence sent.
Send release.
```

**Timeout (phone not responding):**
```
MWI: Timeout waiting for release guard - delivery status unknown.
MWI delivery timeout (no ack)!
```

## References

- NMT DOC 450-1, Section 4.3.3.11: Coding of additional information, frame 5c
  - Defines FFF field structure and bit meanings
  - States that iₙ = 0 does NOT reset indicator (when 5c sent alone)
- NMT DOC 450-1, Section 4.4.1.16: Clearing sequences with message indicators [OPTIONAL]
  - Defines the clearing sequence: `5c|5c|5a(L=13,15)|5a|5a|5a|6|6|6|6|20(A=12)`
  - States L(13)/L(15) causes phone to apply ALL indicator bits from 5c
- NMT DOC 450-1, Section 4.3.3.3: Line signals L(n)
  - L(13): Clearing, call transfer activated
  - L(15): Clearing, call transfer NOT activated
- NMT DOC 450-1, Section 4.7.2: Acceptance of signals
  - Unsupported frames are ignored (safe to send 5c to any phone)

## Implementation Notes

### Active States

Two new active states were added to `enum nmt_active_state`:

- `ACTIVE_STATE_MWI` - Sending MWI frame 5c (2 frames per spec)
- `ACTIVE_STATE_MWI_CLEAR` - Sending clearing sequence: 5c (2x) + L(15) (4x)

### Frame Timing and Sequence

Per NMT DOC 450-1 Section 4.4.1.16:

**Out-of-call MWI delivery:**
```
Frame 1-2:  5c (MWI data)      ← ACTIVE_STATE_MWI
Frame 3-6:  L(15) (clearing)   ← STATE_MT_RELEASE
Wait for:   L(1) (release guard from phone)
```

**In-call MWI followed by call end:**
```
During call:
  Frame 1-2:  5c (immediate notification)  ← ACTIVE_STATE_MWI
  Return to voice...

At call end (if mwi_pending set):
  Frame 1-2:  5c (final MWI state)         ← ACTIVE_STATE_MWI_CLEAR
  Frame 3-6:  L(15) (clearing)             ← ACTIVE_STATE_MWI_CLEAR
  Transition to STATE_MT_RELEASE
  Wait for:   L(1) (release guard)
```

**Timeout handling:**
- If L(1) is not received within timeout, the channel is released anyway
- This is logged as "MWI delivery timeout (no ack)"

## Files Modified

- `src/nmt/main.c` - MWI FIFO creation and handler
- `src/nmt/nmt.c` - MWI delivery functions, tx_active MWI handling
- `src/nmt/nmt.h` - MWI function declarations, new active states
- `src/nmt/transaction.h` - MWI fields in transaction structure

