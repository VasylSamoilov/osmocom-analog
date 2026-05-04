# FLEX Information Service Test Results

## Overview

FLEX Information Service addresses (capcodes 2009088–2025471) are a special
range of 16384 addresses used by the infrastructure to deliver system
information pages to pagers. These pages are displayed with a special "info"
icon and handled separately from ordinary messages (maildrop flag).

## Test Setup

The test script `test_flex_infosvc.sh` sends an alphanumeric message to every
info service capcode (2009088–2025471) with `maildrop=1` flag set. The payload
of each message is the capcode itself, so the pager displays which info service
address it received on.

```bash
# FIFO format: capcode,type,flags,payload
echo "${cap},alpha,maildrop=1,${cap}" > /tmp/flex_msg_send
```

## Results

Three pagers with different capcodes received info service pages:

| Pager Capcode | Info Service Capcode Received | Payload Displayed |
|---------------|-------------------------------|-------------------|
| 4705271       | 2009204                       | 2009204           |
| 7005031       | 2009188                       | 2009188           |
| 5237593       | 2009172                       | 2009172           |

All three pagers displayed the message with the "info" designation and
special icon, confirming the maildrop flag is correctly encoded in the
vector word.

## Analysis

The received info service capcodes differ by exactly 16:

```
2009204 - 2009188 = 16
2009188 - 2009172 = 16
```

This stride of 16 matches the FLEX frame assignment formula:
`assigned_frame = (capcode / 16) mod 128`

The info service capcode a pager subscribes to is determined by its own
capcode's frame assignment, ensuring the pager can receive info service
pages on a frame it already wakes for (no additional battery cost).

## Info Service Address Range

- Start: 2009088
- End: 2025471
- Count: 16384 (= 128 frames × 128 addresses per frame)
- Stride: 16 (one info service capcode per frame slot)

## Encoding

The maildrop flag is set in the FLEX vector word:
- Vector type: Standard Alpha (type 5)
- M bit (maildrop) = 1
- This causes the pager to store the message in the "info" mailbox
  rather than alerting as a regular page

## How It Works

1. Infrastructure assigns each pager an info service capcode based on
   its frame assignment
2. Pager stores this capcode in its subscription list
3. When the infrastructure needs to deliver system info (time updates,
   coverage changes, service announcements), it sends to the info
   service capcode range
4. Pager receives on its assigned frame, sees the info service capcode
   in the address field, displays with info icon

## Transmitter Implementation

The transmitter (`src/flex/`) handles info service pages identically to
regular alpha pages — the only difference is the `maildrop=1` flag in
the FIFO input which sets the M bit in the vector word. No special
frame scheduling is needed since the capcode's frame assignment
naturally places the page in the correct frame.
