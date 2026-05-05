# FLEX Information Service Address Correlation

## Overview

FLEX Information Service addresses (capcodes 2,009,088-2,025,471) are a
range of 16,384 special addresses per ARIB STD-43A.  The standard states
their use is "under study" with no defined protocol behavior.

In practice, pagers are provisioned with exactly one info service capcode
as a subscribed address.  Messages sent to that capcode with the maildrop
flag (M=1 in the vector word) are displayed with a special "info" icon.

Tested with Motorola Beepwear pagers.

## Test Setup

The test script `test_flex_infosvc.sh` sends an alphanumeric message to
every info service capcode (2,009,088-2,025,471) with `maildrop=1`.
The payload of each message is the capcode itself, so the pager displays
which info service address it received on.

```bash
# FIFO format: capcode,type,flags,payload
echo "${cap},alpha,maildrop=1,${cap}" > /tmp/flex_msg_send
```

## Results

Three pagers with different capcodes each received exactly one info
service message:

| Pager Capcode | Info Service Capcode | Pager Frame | Info Frame | Slot (f%8) |
|---------------|---------------------|-------------|------------|------------|
| 4,705,271     | 2,009,204           | 63          | 7          | 7          |
| 7,005,031     | 2,009,188           | 54          | 6          | 6          |
| 5,237,593     | 2,009,172           | 53          | 5          | 5          |

Each pager received only one capcode out of 16,384.  This confirms the
info service capcode is a programmed subscription, not a broadcast.

## Correlation: Collapse Slot Alignment

The info service capcode is assigned so that its frame falls on the same
**collapse slot** as the pager's primary capcode.  This ensures the pager
receives info service messages on a frame it already wakes for (zero
additional battery cost).

With system collapse=3, the pager wakes every 2^3 = 8 frames.
The collapse slot is: `(assigned_frame) % 8`.

```
Pager frame assignment:  assigned_frame = (capcode / 16) % 128
Pager collapse slot:     slot = assigned_frame % 2^collapse
Pager wakes on frames:   all frames where (frame % 2^collapse) == slot
```

Verification:

```
Pager 4,705,271:  frame = (4705271/16) % 128 = 63,  slot = 63 % 8 = 7
Info  2,009,204:  frame = (2009204/16) % 128 = 7,   7 % 8 = 7  ← same slot ✓

Pager 7,005,031:  frame = (7005031/16) % 128 = 54,  slot = 54 % 8 = 6
Info  2,009,188:  frame = (2009188/16) % 128 = 6,   6 % 8 = 6  ← same slot ✓

Pager 5,237,593:  frame = (5237593/16) % 128 = 53,  slot = 53 % 8 = 5
Info  2,009,172:  frame = (2009172/16) % 128 = 5,   5 % 8 = 5  ← same slot ✓
```

## Why It Works

INFO_BASE (2,009,088) is frame-aligned to frame 0:

```
2,009,088 / 16 = 125,568 = 981 × 128 + 0
```

So `INFO_BASE + N*16` lands on frame N.  Adding less than 16 to a capcode
does not change its frame assignment (integer division by 16 truncates).

The operator assigns:

```
info_capcode = INFO_BASE + slot × 16 + offset
```

Where:
- `slot` = pager's collapse slot, ensures same wake frame
- `offset` = observed as 4 for all three pagers (purpose unknown,
  possibly operator convention; does not affect frame assignment)

## Collapse Value Dependency

This alignment is valid for collapse values 0 through 3.  At collapse ≥ 4
(period ≥ 16), the pager may not wake on the info service frame:

| Collapse | Period | Pager f=63, Info f=7 | Works? |
|----------|--------|---------------------|--------|
| 0        | 1      | wakes every frame   | yes    |
| 1        | 2      | 63%2=1, 7%2=1      | yes    |
| 2        | 4      | 63%4=3, 7%4=3      | yes    |
| 3        | 8      | 63%8=7, 7%8=7      | yes    |
| 4        | 16     | 63%16=15, 7%16=7   | NO     |

The observed pagers use collapse=3, which is the most common value for
consumer FLEX pagers.

## Info Service Address Range

Per ARIB STD-43A (Table in Section 3.8.1):

- Start: 2,009,088 (capcode), address word 0x1F2800
- End: 2,025,471 (capcode), address word 0x1F67FF
- Count: 16,384
- Address word = capcode + 32,768 (short address offset)

## Encoding

The maildrop flag is set in the FLEX vector word:
- Vector type: Standard Alpha (type 5)
- M bit (maildrop) = 1
- This causes the pager to display the message with the "info" icon

## Summary

The info service capcode is not computed by the pager at runtime.  It is
programmed during provisioning.  The operator assigns it based on the
pager's collapse slot to guarantee reception without extra wake-ups.
The standard does not define this assignment algorithm -- it is an
operator-level provisioning convention observed empirically.
