# NMT: move active call to another voice channel (SCIP), call queueing, and received-frame logging

This describes NMT features added on top of the base osmocom-analog NMT implementation.
All are single-cell and standard-derived (NMT Doc 450-1 / 450-3); none require a second
network node. Items that cannot be implemented faithfully in a single-node emulator
(authentication, coin-box tariff *sending*, BS orders) are intentionally decode-and-log only.

## 1. SCIP — move an active call to another traffic channel

"Switching Call In Progress" (NMT Doc 450-3 §5.4.5). An operator command moves an
in-progress call from its current traffic channel (TC) to another free TC, by ordering
the mobile over the air to retune.

Control FIFO: `/tmp/nmt_move`

    echo "3735859"   > /tmp/nmt_move     # move subscriber 3735859 to any free TC (auto-select)
    echo "3735859,5" > /tmp/nmt_move     # move to a specific channel (5)

Flow (modelled on the AMPS handoff path `amps_handoff`):
  1. `STATE_SCIP_ORDER`: send frame **3a** (channel change order, prefix 5, carrying the
     new TC number in NaNbNc) on the current TC, repeated 4 frames.
  2. The transaction is re-parented to the target TC and the old channel is idled.
  3. `STATE_SCIP_IDENT`: the MS retunes and re-identifies with frame **10b** on the new TC;
     on receipt the call resumes (`STATE_ACTIVE`, audio + supervisory).
  4. Timeout (3 s): MS did not re-appear -> the call is released.

Only a plain voice call in `STATE_ACTIVE` is movable (not MFT/MWI sub-states or data calls).

## 2. Call queueing with priority

When a mobile-terminated call is answered but no TC is free (race during paging), the call
is now **queued** instead of dropped (NMT Doc 450-1 §4.4.1.6/4.4.1.7).

  - `STATE_MT_QUEUE`: the MTX sends queue info on the calling channel —
      frame **2f** (ordinary MS, H8H9H10 = 3F0) or
      frame **2c** (priority MS,  H8H9H10 = 3F1).
  - Priority is detected from a priority-MS call acknowledgement (frame **11b**) during paging.
  - Each frame re-checks for a free TC; as soon as one is available the normal channel
    assignment (frame 2b, `STATE_MT_CHANNEL`) proceeds.
  - Timeout (60 s): no TC became free -> the call is released with CAUSE_NOCHANNEL.

## 3. Received-frame logging (decode-only)

Frames the single-node MTX receives but does not act on are now decoded and logged at
NOTICE instead of being silently dropped, for diagnostics and capture analysis. Each line
states why it is observe-only:

  - frame **13b** — coin-box answer acknowledgement, with the tariff class the MS echoes back;
  - frame **16**  — signed response (SRES); authentication is not implemented;
  - frame **26**  — signal-strength measurement result; handover is not implemented;
  - frame **25**  — channel status information (SCC); not acted on;
  - frame **27/28**— BS management/maintenance response/info; not acted on;
  - frame **10c** — called-MS seizure after a (not implemented) traffic-channel scanning order.

## Not implemented (and why)

  - **Authentication / SIS** (frames 7/16): requires the SIS Authentication Register and a
    SAK programmed into the handset; cannot be verified without such a phone. The current
    unencrypted path (always 5a L(3), never frame 7) is spec-sanctioned for non-SIS phones.
  - **Coin-box tariff *sending*** (frame 5b): requires a charging/tariff source and an
    answer-supervision event that the emulator's call-control layer does not provide. The
    receive side (13b echo) is logged.
  - **BS orders** (frames 20/21/22; replies 25/26/27/28): these are the MTX<->BS interface,
    which is internal to this single-node design and has no transport. They become meaningful
    only if the BS and MTX are split into separate nodes.
