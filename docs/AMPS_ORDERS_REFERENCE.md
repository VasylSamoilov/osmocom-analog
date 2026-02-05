# AMPS/TACS Orders Reference

This document lists all analog orders defined in TIA/EIA-553-A and their implementation status in osmocom-analog. Digital (TDMA) and encryption orders are excluded.

## Terminology

- **FOCC** - Forward Control Channel (BS→MS, idle/paging mode)
- **RECC** - Reverse Control Channel (MS→BS, access/registration)
- **FVC** - Forward Voice Channel (BS→MS, during call)
- **RVC** - Reverse Voice Channel (MS→BS, during call)
- **CRI** - Charging Rate Indication: Displays call cost/rate information to user
- **TCI** - Total Charging Information: Shows accumulated call charges to user
- **ST** - Signaling Tone: 10 kHz tone sent by mobile to confirm orders

## Quick Reference Table

| Order | ORDQ | Name | Channel | TX | RX Confirm |
|-------|------|------|---------|-----|------------|
| 0 | 0 | Page / Origination | FOCC/RECC | ✅ | ✅ (Page Response) |
| 1 | 0 | Alert | FVC | ✅ | ✅ (ST tone) |
| 1 | 1 | Abbreviated Alert | FVC | ✅ | ✅ (ST tone) |
| 3 | 0 | Release | FVC | ✅ | ✅ (ST tone) |
| 4 | 0 | Reorder | FVC | ✅ | - (no confirm) |
| 5 | 0-2 | Message Waiting | FVC | ✅ | ✅ (RVC msg) |
| 6 | 0 | Stop Alert | FVC | ✅ | - (no confirm) |
| 7 | 0 | Audit | FOCC/FVC | ✅ | ✅ (RVC msg / RECC msg) |
| 8 | 0 | Send Called-Address | FVC | ✅ | ✅ (RVC msg) |
| 9 | 0 | Intercept | FVC | ✅ | - (no confirm) |
| 10 | 0 | Maintenance | FVC | ✅ | ✅ (ST tone) |
| 11 | 0-7 | Change Power | FVC | ✅ | ✅ (RVC msg, logged) |
| 12 | 0-1 | Directed Retry | FOCC | ✅ | - (no confirm) |
| 13 | 0-3 | Registration | RECC | - | ✅ (RX only) |
| 15 | 0-1 | Serial Number Req | FVC/RVC | ✅ | ✅ (RX impl) |
| 17 | 0-2 | Alert With Info | FVC | ✅ | ✅ (ST tone) |
| 18 | 0-2 | Flash With Info | FVC | ✅ | ✅ (RVC msg, logged) |
| 26 | 4 | PCI Query/Report | FOCC/FVC | ✅ | ✅ (RECC/RVC msg, parsed) |
| 30 | 0 | Local Control | FVC | ✅ | - (no confirm) |

**Legend:**
- **TX**: Base station can send this order
- **RX Confirm**: Base station receives and processes confirmation
- **ST tone**: Confirmation via 10 kHz Signaling Tone
- **RVC msg**: Confirmation via digital message on Reverse Voice Channel
- **RECC msg**: Confirmation via digital message on Reverse Control Channel

---

## Detailed Order Descriptions

### Order 0: Page / Origination / Page Response

**Channel:** FOCC (BS→MS), RECC (MS→BS)
**Status:** ✅ TX Implemented, ✅ RX Confirmation Implemented

**Description:**
The Page order is sent by the base station on the Forward Control Channel to notify a mobile station of an incoming call. The mobile responds with a Page Response on the RECC. When a mobile originates a call, it sends an Origination message (also Order 0) with dialed digits.

**Confirmation Method:** Digital message on RECC (Page Response)

**Use Cases:**
- Incoming call notification (MT call)
- Mobile-originated call setup (MO call)
- Page response to confirm mobile is present

**Implementation:** 
- TX: `amps_tx_frame_focc()` handles paging via `TRANS_PAGE`
- RX: `amps_rx_recc()` handles origination/page response, triggers `TRANS_CALL_MT_ASSIGN`

---

### Order 1: Alert

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented (ORDQ=0 and ORDQ=1), ✅ ST Confirmation Implemented

**Description:**
The Alert order causes the mobile station to ring/alert the user that an incoming call is being received. The mobile confirms receipt by sending a Signaling Tone (ST) burst - a 10 kHz tone.

**Confirmation Method:** ST (Signaling Tone) - 10 kHz tone detected by `amps_rx_signaling_tone()`

**Variants:**
- **ORDQ=0 (Alert):** Standard ring for incoming call
- **ORDQ=1 (Abbreviated Alert):** Short reminder tone to indicate that previously selected alternative routing features (call forwarding, etc.) are still active

**Use Cases:**
- Ring the phone for incoming call
- Reminder that call forwarding is enabled (abbreviated)

**Implementation:** 
- TX Alert: `amps_tx_frame_fvc()` → `TRANS_CALL_MT_ALERT`
- TX Abbreviated Alert: `amps_tx_frame_fvc()` → `TRANS_CALL_ABB_ALERT` → `TRANS_CALL_ABB_ALERT_SEND`
- RX: ST detection in `dsp.c` → `amps_rx_signaling_tone()` → state `TRANS_CALL_MT_ALERT_CONFIRM` or `TRANS_CALL_ABB_ALERT_SEND`
- On ST detection: Triggers `call_up_alerting()` and moves to `TRANS_CALL_MT_ANSWER_WAIT`
- Console (Alert): `echo "alert,<MIN>" > /tmp/amps_control` - requires active call (`TRANS_CALL` state)
- Console (Abbreviated Alert): `echo "abbalert,<MIN>" > /tmp/amps_control` - requires active call (`TRANS_CALL` state)

---

### Order 3: Release

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented, ✅ ST Confirmation Implemented

**Description:**
The Release order disconnects a call that is being established or is already established. Sent when the remote party hangs up or the network needs to terminate the call. Mobile confirms with ST tone.

**Confirmation Method:** ST (Signaling Tone) - detected in `TRANS_CALL_RELEASE_SEND` state

**Variants:**
- **ORDQ=0:** Standard release
- **ORDQ=2:** Release with Digital Control Channel Information (for handoff to digital)
- **ORDQ=3:** Release Complete (acknowledgment)

**Use Cases:**
- Remote party hangup
- Network-initiated call termination
- Call setup failure

**Implementation:** 
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_RELEASE`
- RX: ST detection triggers call teardown and `amps_go_idle()`

---

### Order 4: Reorder

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented

**Description:**
The Reorder order informs the user that all facilities (circuits/trunks) are busy and the call should be placed again later. The mobile plays a "fast busy" tone (reorder tone) - 480+620 Hz interrupted at 120 IPM (0.25s on, 0.25s off).

Per TIA/EIA-553-A Section 2.6.3.8: After receiving Reorder, the mobile returns to idle (Serving-System Determination Task) without requiring a Release order.

**Confirmation Method:** None - mobile returns to idle automatically

**Use Cases:**
- All voice channels busy
- Network congestion
- Trunk group exhausted

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_REORDER` → `TRANS_CALL_REORDER_SEND`
- After send: Transaction destroyed, channel goes idle
- Auto-mapping: Sent automatically for CAUSE_NOCHANNEL (34) and CAUSE_TEMPFAIL (41) during MO call setup
- Console: `echo "reorder,<MIN>" > /tmp/amps_control`

---

### Order 5: Message Waiting

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented, ✅ RX Confirmation Implemented

**Description:**
The Message Waiting order notifies the mobile station that messages are waiting. The MSG_TYPE field indicates the number of messages (0-30, or 31 for "unknown number"). The mobile typically displays an indicator icon and sends an Order Confirmation.

**Confirmation Method:** Digital Order Confirmation on RVC

**Variants:**
- **ORDQ=0:** Voice messages (voicemail)
- **ORDQ=1:** SMS messages
- **ORDQ=2:** G3-Fax messages

**Use Cases:**
- Voicemail notification
- SMS waiting indicator
- Fax message notification

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_MWI` → `TRANS_CALL_MWI_SEND`
- RX: Order Confirmation logged in `frame.c`
- After send: Returns to `TRANS_CALL` state
- Console: `echo "mwi,<MIN>,<count>[,<type>]" > /tmp/amps_control`
  - count: 0-31 (0=clear, 31=unknown number)
  - type: 0=voice, 1=SMS, 2=fax (optional, default=0)

---


### Order 6: Stop Alert

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented

**Description:**
The Stop Alert order tells the mobile station to discontinue alerting (ringing) the user. Used when the calling party hangs up before the mobile user answers. Unlike Release, this does NOT tear down the voice channel.

**Confirmation Method:** None

**Use Cases:**
- Caller abandons call before answer
- Call forwarding activated during ring
- Network timeout during alerting

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_STOP_ALERT` → `TRANS_CALL_STOP_ALERT_SEND`
- After send: Returns to `TRANS_CALL` state (channel stays up)
- Console: `echo "stopalert,<MIN>" > /tmp/amps_control`

---

### Order 7: Audit

**Channel:** FOCC (BS→MS idle), FVC (BS→MS in-call)
**Status:** ✅ TX Implemented (FOCC + FVC), ✅ RX Confirmation Implemented

**Description:**
The Audit order is used by the base station to determine whether the mobile station is still active in the system. The mobile confirms by sending an Order Confirmation message (digital). This is essentially a "ping" to verify the mobile is still present.

**Confirmation Method:** 
- On FOCC: Digital Order Confirmation on RECC (handled in `amps_rx_recc()`)
- On FVC: Digital Order Confirmation on RVC (handled in `frame.c`, logged with `amps_table4_name()`)

**Use Cases:**
- Verify mobile is still on channel (in-call)
- Check if idle mobile is still registered
- Detect if mobile has moved out of range
- Fraud detection (verify ESN matches)

**Implementation:** 
- TX FOCC: `amps_tx_frame_focc()` → `TRANS_AUDIT` → `TRANS_AUDIT_SEND`
- TX FVC: `amps_tx_frame_fvc()` → `TRANS_CALL_AUDIT` → `TRANS_CALL_AUDIT_SEND`
- RX RECC: `amps_rx_recc()` logs "Audit from <MIN> (ESN = ...)"
- RX RVC: `frame.c` logs "Received RVC Order Confirmation: Audit"
- Console: `echo "audit,<number>" > /tmp/amps_control`

---

### Order 8: Send Called-Address

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented, ✅ RX Confirmation Implemented, ✅ Auto-send after Flash

**Description:**
The Send Called-Address order requests the mobile station to send a message containing dialed-digit information. Used for call transfer and three-way calling scenarios.

**CRITICAL REQUIREMENT (TIA/EIA-553-A Section 2.6.4.4):**
> "Send Called-Address:
> + If received **within 10 seconds of the completion of the last valid flash**, send the called-address to the base station and remain in the Conversation Task.
> + **Otherwise, ignore the order** and remain in the Conversation Task."

**Automatic Behavior:**
Order 8 is automatically sent ~1 second after a flash is detected. This gives the user time to dial digits on the mobile before the base station requests them.

**Typical Workflow (Call Transfer / 3-Way Calling):**
1. User **dials digits** on mobile keypad (e.g., transfer destination "5551234")
2. User presses **FLASH** on mobile (sends 400ms ST burst to base station)
3. Base station **automatically** sends Order 8 after ~1 second
4. Mobile responds with Called-Address message containing the dialed digits

If Order 8 is sent outside the 10-second window after flash, the mobile silently ignores it.

**Confirmation Method:** Digital Called-Address message on RVC (T=0 message with dialed digits)

**Use Cases:**
- Re-request digits after a flash for call transfer
- Three-way calling digit collection
- Feature code entry after flash

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_DIGITS_REQUEST` → `TRANS_CALL_DIGITS_REQUEST_SEND`
- RX: `frame.c` decodes RVC Called-Address (T=0) and extracts digits
- Handler: `amps_rx_recc()` logs the received digits
- Auto-trigger: `flash_timer_callback()` sends Order 8 ~1 second after flash detection
- Manual: `echo "digits,<MIN>" > /tmp/amps_control`

---

### Order 9: Intercept

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented

**Description:**
The Intercept order informs the user of a procedural error made in placing the call. The mobile plays an intercept tone (alternating high-low tone) indicating the number cannot be completed as dialed.

Per TIA/EIA-553-A Section 2.6.3.8: After receiving Intercept, the mobile returns to idle (Serving-System Determination Task) without requiring a Release order.

**Confirmation Method:** None - mobile returns to idle automatically

**Use Cases:**
- Invalid number dialed
- Number not in service
- Restricted destination
- Incomplete dialing

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_INTERCEPT` → `TRANS_CALL_INTERCEPT_SEND`
- After send: Transaction destroyed, channel goes idle
- Auto-mapping: Sent automatically for CAUSE_INVALNUMBER (28) and CAUSE_OUTOFORDER (27) during MO call setup
- Console: `echo "intercept,<MIN>" > /tmp/amps_control`

---

### Order 10: Maintenance

**Channel:** FVC only (BS→MS) - **Cannot be sent on FOCC to idle mobiles**
**Status:** ✅ TX Implemented, ✅ ST Confirmation Implemented, ✅ Silent Page Flow Implemented

**Description:**
The Maintenance order is used to check the operation of a mobile station. All functions are similar to Alert, but the alerting device (ringer) is NOT activated. The mobile confirms with ST (500ms) but doesn't ring.

Per TIA/EIA-553-A Section 2.6.4.4: "Maintenance: Turn on signaling tone, wait 500 ms, and then enter the Waiting for Answer Task"

**IMPORTANT:** Per TIA/EIA-553-A Section 3.6.2.3, Maintenance is NOT in the list of orders that can be sent on FOCC. It can ONLY be sent on FVC during an active call. To test an idle mobile, use the "Silent Page" flow which pages the mobile, assigns a channel, sends Maintenance, and releases.

**Confirmation Method:** ST (Signaling Tone) - 10 kHz tone for 500ms

**Use Cases:**
- Silent test of mobile station during active call
- Verify mobile RF path without disturbing user (during call)
- Network testing/diagnostics
- ESN verification without alerting

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_MAINTENANCE` → `TRANS_CALL_MAINTENANCE_SEND`
- RX: ST detection in `amps_rx_signaling_tone()` → logs "Maintenance test confirmed"
- After ST: Returns to `TRANS_CALL` state
- Console: `echo "maintenance,<MIN>" > /tmp/amps_control` (requires active call)

**Silent Page Flow (for idle mobiles):**
The "Silent Page" feature allows testing idle mobiles by:
1. Paging the mobile on FOCC (same as MT call)
2. Waiting for page reply
3. Assigning voice channel
4. Waiting for SAT confirmation
5. Sending Maintenance order (instead of Alert)
6. Waiting for ST confirmation (500ms)
7. Sending Release order to terminate

Console: `echo "silentpage,<MIN>" > /tmp/amps_control`

Output:
- SUCCESS: `*** Silent Page SUCCESS: Mobile '<MIN>' confirmed operational (ST received) ***`
- FAILED (no page reply): `*** Silent Page FAILED: Mobile '<MIN>' did not respond to page ***`
- FAILED (no SAT): `*** Silent Page FAILED: Mobile '<MIN>' did not confirm SAT ***`
- FAILED (no ST): `*** Silent Page FAILED: Mobile '<MIN>' did not confirm Maintenance (no ST) ***`

---


### Order 11: Change Power

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented, ✅ RX Confirmation Implemented (logged)

**Description:**
The Change Power order adjusts the mobile station's RF transmit power level. AMPS defines 8 power levels (0-7), where level 0 is maximum power and level 7 is minimum. Each step is nominally 4 dB. Mobile confirms with Order Confirmation message on RVC.

**Confirmation Method:** Digital Order Confirmation on RVC (logged in `frame.c`)

**Power Levels:**
| Level | Class I | Class II | Class III |
|-------|---------|----------|-----------|
| 0 | 4.0W (6dBW) | 1.6W (2dBW) | 0.6W (-2dBW) |
| 1 | 1.6W (2dBW) | 1.6W (2dBW) | 0.6W (-2dBW) |
| 2 | 0.6W (-2dBW) | 0.6W (-2dBW) | 0.6W (-2dBW) |
| 3-7 | -4dB per step | -4dB per step | -4dB per step |

**Use Cases:**
- Reduce interference when mobile is close to base
- Increase power when signal is weak
- Battery conservation
- Co-channel interference management

**Implementation:** 
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_CHANGE_POWER`, ORDQ carries power level
- RX: `frame.c` logs "Received RVC Order Confirmation: Change Power to Power Level X"
- After confirmation: Returns to `TRANS_CALL` state with grace period for SAT

---

### Order 12: Directed Retry

**Channel:** FOCC (BS→MS)
**Status:** ❌ Not Implemented

**Description:**
The Directed Retry order redirects a mobile station to attempt access on a different cell's control channel. Used when the current cell is congested but a neighboring cell has capacity.

**Variants:**
- **ORDQ=0:** Not last try (mobile should try again if this fails)
- **ORDQ=1:** Last try (mobile should give up if this fails)

**Use Cases:**
- Cell congestion/overload
- Load balancing between cells
- Better signal available on neighbor cell
- Capacity optimization

**Implementation Notes:**
Would require knowledge of neighbor cell frequencies. Mobile scans provided channel list and attempts access on best signal.

---

### Order 13: Registration

**Channel:** RECC (MS→BS)
**Status:** ✅ RX Implemented, ✅ TX Acknowledgment Implemented

**Description:**
Registration messages are sent by the mobile station to identify itself to the system. The base station acknowledges with a Registration Acknowledgment message on FOCC.

**Confirmation Method:** N/A (mobile-initiated), BS sends Registration Ack

**Variants:**
- **ORDQ=0:** Power-up registration
- **ORDQ=1:** Periodic registration
- **ORDQ=2:** Location area registration
- **ORDQ=3:** Power-down registration (MSG_TYPE=1)

**Use Cases:**
- Mobile power on
- Mobile entering new location area
- Periodic keep-alive
- Mobile power off notification

**Implementation:** 
- RX: `amps_rx_recc()` handles registration message
- TX Ack: `amps_tx_frame_focc()` → `TRANS_REGISTER_ACK` → `TRANS_REGISTER_ACK_SEND`
- Logs: "Registration <MIN> (ESN = ..., SCM, MPCI)"

---

### Order 15: Serial Number Request/Response

**Channel:** FVC (BS→MS), RVC (MS→BS)
**Status:** ✅ TX Implemented, ✅ RX Response Handler Implemented

**Description:**
The Serial Number Request order requests the mobile station to send its Electronic Serial Number (ESN). Used for verification and fraud detection. The mobile responds with a 2-word Serial Number Response containing its 32-bit ESN.

**Confirmation Method:** Digital Serial Number Response on RVC (2-word message)

**Response Format:**
- Word 1: Order=15, ORDQ=1, NAWC=1 (header)
- Word 2: 32-bit ESN

**Use Cases:**
- ESN verification during call
- Fraud detection
- Cloning detection
- Subscriber verification

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_ESN_REQUEST` → `TRANS_CALL_ESN_REQUEST_SEND`
- RX: `amps_rx_esn_response()` in amps.c, called from frame.c
- ESN comparison: Received ESN compared with stored ESN from call setup
- Fraud alert: Logged at ERROR level if ESN mismatch detected
- Console: `echo "esn,<MIN>" > /tmp/amps_control`

**Fraud Detection:**
When the received ESN doesn't match the stored ESN from call setup, a FRAUD ALERT is logged:
```
*** FRAUD ALERT: ESN MISMATCH for '5203495579' ***
  Expected: OKI-123456 (MFR=0x01, Serial=123456)
  Received: MOT-789012 (MFR=0x08, Serial=789012)
  This may indicate a cloned phone or SIM swap!
```

---


### Order 17: Alert With Info

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented (all ORDQ variants), ✅ ST Confirmation Implemented

**Description:**
The Alert With Info order alerts the user of an incoming call AND provides additional information to display. This is the primary mechanism for Caller ID delivery in AMPS.

**Confirmation Method:** ST (Signaling Tone) - same as standard Alert (Order 1)

**Variants:**
- **ORDQ=0:** Alert with Calling Party Number (CPN) - Caller ID ✅
- **ORDQ=1:** Alert with CRI (Charging Rate Indication) - shows call cost rate ✅
- **ORDQ=2:** Alert with TCI (Total Charging Information) - shows accumulated charges ✅

**Message Structure (ORDQ=0):**
- Word 1: Order with signal pitch/cadence
- Word 2: CPN length, presentation indicator, screening indicator
- Word 3+: Caller ID digits (3 per word, up to 15 digits)

**Message Structure (ORDQ=1 - CRI):**
- Word 1: Order with ORDQ=1
- Word 2: RL_W (word count), signal pitch/cadence
- Word 3-8: CRI elements (up to 8 elements × 4 BCD digits)

**Message Structure (ORDQ=2 - TCI):**
- Word 1: Order with ORDQ=2
- Word 2: RL_W (word count), signal pitch/cadence
- Word 3-5: TCI rows (up to 4 rows × 4 BCD digits)

**Presentation Indicator (PI):**
- 0: Presentation allowed (show number)
- 1: Presentation restricted (show "PRIVATE")
- 2: Number not available
- 3: Reserved

**Screening Indicator (SI):**
- 0: User provided, not screened
- 1: User provided, verified and passed
- 2: User provided, verified and failed
- 3: Network provided

**Use Cases:**
- Caller ID display (ORDQ=0)
- Call cost indication (ORDQ=1 CRI)
- Accumulated charges display (ORDQ=2 TCI)

**Implementation:** 
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_MT_ALERT` with order=17
- RX: ST detection in `dsp.c` → `amps_rx_signaling_tone()` → state `TRANS_CALL_MT_ALERT_CONFIRM`
- On ST detection: Triggers `call_up_alerting()` and moves to `TRANS_CALL_MT_ANSWER_WAIT`
- Console: `echo "alertcri,<number>,<cri_data>" > /tmp/amps_control`
- Console: `echo "alerttci,<number>,<tci_data>" > /tmp/amps_control`

**Valid States for alertcri/alerttci:**
- `TRANS_CALL` - Active call (re-alert with charging info)
- `TRANS_CALL_MT_ALERT*` - During MT call alerting phase (normal use case)
- `TRANS_CALL_MT_ANSWER_WAIT` - While waiting for answer

---

### Order 18: Flash With Info

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented (all ORDQ variants), ✅ RX Confirmation Implemented (logged)

**Description:**
The Flash With Info order indicates that special processing is required and provides character information to display. Similar to Alert With Info but sent during an active call.

**Confirmation Method:** Digital Order Confirmation on RVC (logged in `frame.c`)

**Variants:**
- **ORDQ=0:** Flash with character message ✅
- **ORDQ=1:** Flash with CRI (Charging Rate Indication) ✅
- **ORDQ=2:** Flash with TCI (Total Charging Information) ✅

**Message Structure (ORDQ=0):**
- Word 1: Order with ORDQ=0
- Word 2: RL_W (word count), CPN_RL, PI, SI
- Word 3+: Character message (3 per word)

**Message Structure (ORDQ=1 - CRI):**
- Word 1: Order with ORDQ=1
- Word 2: RL_W (word count)
- Word 3-8: CRI elements (up to 8 elements × 4 BCD digits)

**Message Structure (ORDQ=2 - TCI):**
- Word 1: Order with ORDQ=2
- Word 2: RL_W (word count)
- Word 3-5: TCI rows (up to 4 rows × 4 BCD digits)

**Use Cases:**
- Call waiting notification with Caller ID (ORDQ=0)
- Mid-call charging rate update (ORDQ=1 CRI)
- Running call cost display (ORDQ=2 TCI)
- Feature activation confirmation

**Implementation:** 
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_FLASH_INFO` → `TRANS_CALL_FLASH_INFO_SEND`
- RX: `frame.c` logs "Received RVC Order Confirmation: Flash With Info"
- After send: Returns to `TRANS_CALL` state
- Console: `echo "flash,<number>,<message>" > /tmp/amps_control`
- Console: `echo "flashcri,<number>,<cri_data>" > /tmp/amps_control`
- Console: `echo "flashtci,<number>,<tci_data>" > /tmp/amps_control`

---

### Order 26: Protocol Capability Indicator (PCI)

**Channel:** FOCC/FVC (BS→MS), RECC/RVC (MS→BS)
**Status:** ✅ TX Implemented (FOCC + FVC), ✅ RX Confirmation Implemented (parsed and logged)

**Description:**
The PCI Query order requests the mobile station to report its protocol capabilities. The mobile responds with MSPC (Mobile Station Protocol Capability) and MSCAP (Mobile Station Core Analog Protocol) values.

Can be sent in any state:
- **FOCC (idle):** Mobile responds on RECC
- **FVC (in-call):** Mobile responds on RVC

**Confirmation Method:** Digital Order Confirmation on RECC/RVC with MSPC/MSCAP fields (parsed in `amps_rx_pci_report()`)

**Prerequisites:**
- System must broadcast `PCI_HOME=1` and/or `PCI_ROAM=1` in Access Type Parameters overhead message
- System must set `BSCAP=1` when PCI_HOME or PCI_ROAM is set
- These are already configured in `sysinfo.c`

**MSPC Values:**
- 0: EIA-553 (original AMPS)
- 1: IS-54-A (first digital)
- 2: IS-54-B (enhanced digital)
- 3: IS-136 (TDMA)

**MSCAP Values:**
- 0: EIA-553 (original)
- 1: TIA/EIA-553-A (enhanced analog)

**Use Cases:**
- Determine mobile capabilities
- Feature negotiation
- Network planning/statistics
- Handoff decisions (analog vs digital)

**Implementation:** 
- TX FOCC: `amps_tx_frame_focc()` → `TRANS_PCI` → `TRANS_PCI_SEND`
- TX FVC: `amps_tx_frame_fvc()` → `TRANS_CALL_PCI_QUERY` → `TRANS_CALL_PCI_QUERY_SEND`
- RX: `frame.c` detects Order 26 confirmation → calls `amps_rx_pci_report()`
- `amps_rx_pci_report()` logs MSPC/MSCAP values
- Console: `echo "pci,<number>" > /tmp/amps_control` (works for both idle and in-call)

---


### Order 30: Local Control

**Channel:** FVC (BS→MS)
**Status:** ✅ TX Implemented

**Description:**
The Local Control order initiates vendor-specific local control actions in the mobile station. The MSG_TYPE field contains a 5-bit local control code (0-31) that is manufacturer-defined.

**Confirmation Method:** None - vendor-specific behavior

**Use Cases:**
- Vendor-specific features
- Diagnostic modes
- Special test functions
- Proprietary extensions

**Implementation:**
- TX: `amps_tx_frame_fvc()` → `TRANS_CALL_LOCAL_CONTROL` → `TRANS_CALL_LOCAL_CONTROL_SEND`
- After send: Returns to `TRANS_CALL` state
- Console: `echo "local,<MIN>,<code>" > /tmp/amps_control`
  - code: 0-31 (manufacturer-defined)

**WARNING:** This is highly vendor-specific. The meaning of each code depends on the mobile station manufacturer. Without documentation for specific mobile models, the effect of each code is unknown.

---

## CRI and TCI - Important Notice

### Standard Compliance

**IMPORTANT:** The TIA/EIA-553-A standard (November 1999) only defines **ORDQ=0** for both:
- Order 17 (Alert With Info) - Caller ID / Character display
- Order 18 (Flash With Info) - Character message display

The standard's Table 3.7.1-1 "Order and order qualification codes" shows:
```
Order Code  ORDQ  Function
10001       000   Alert With Info
10010       000   Flash With Info
```

**CRI (ORDQ=1) and TCI (ORDQ=2) are NOT defined in TIA/EIA-553-A.**

These variants may be:
- Proprietary extensions by specific equipment vendors
- Defined in TSB-70 (Mobile Station - Land Station Compatibility Specifications)
- Regional extensions (e.g., European charging display requirements)

The standard notes (§2.6.4.3.1, Note 8): "Reference may be made to the current version of TSB-70 for details" regarding reserved bits and features.

### Implementation History

The CRI/TCI word structures were originally defined by Andreas Eversberg in the "AMPS: Caller ID Support" commit, but were **never actually used** - they were placeholder definitions for potential future implementation. The encoding functions, API, and control commands were added later.

Since the word structures were never tested against real equipment, their correctness cannot be verified. The original author may have had access to additional documentation (TSB-70 or vendor specs) that is not available in TIA/EIA-553-A.

### Current Implementation Status

The CRI/TCI implementation uses word structures from the original codebase. This format has NOT been verified against actual mobile station behavior and may not work with real equipment.

**Implementation Notes:**
- CRI/TCI word structures existed in `frame.c` but were unused until now
- Mobile stations may not recognize ORDQ=1 or ORDQ=2 for these orders
- Testing with actual equipment is required to verify compatibility

### CRI - Charging Rate Indication (Unverified)

**WARNING: This format is NOT from TIA/EIA-553-A and may not work with real mobiles.**

CRI is intended to provide real-time call rate information. The implementation uses:
- Up to 8 elements, each containing 4 BCD digits
- Elements are comma-separated in control commands

**Data Format:**
```
E1,E2,E3,E4,E5,E6,E7,E8
```
Each element is up to 4 BCD digits (0-9).

**Example:**
```bash
# Hypothetical: Display rate "0025" (meaning depends on mobile interpretation)
echo "flashcri,0570000000,0025" > /tmp/amps_control

# Hypothetical: Multiple elements
echo "flashcri,0570000000,0100,0000" > /tmp/amps_control
```

**Intended Use Cases (if supported by mobile):**
- Premium rate call warnings
- International call rate display
- Roaming charge notification

### TCI - Total Charging Information (Unverified)

**WARNING: This format is NOT from TIA/EIA-553-A and may not work with real mobiles.**

TCI is intended to provide accumulated call charge information. The implementation uses:
- Up to 4 rows, each containing 4 BCD digits
- Rows are comma-separated in control commands

**Data Format:**
```
R1,R2,R3,R4
```
Each row is up to 4 BCD digits (0-9).

**Example:**
```bash
# Hypothetical: Display charge "1234"
echo "flashtci,0570000000,1234" > /tmp/amps_control

# Hypothetical: Multiple rows
echo "flashtci,0570000000,0567,0000" > /tmp/amps_control
```

**Intended Use Cases (if supported by mobile):**
- Running call cost display
- Budget management
- Prepaid balance indication

### Recommendation

For reliable operation, use **ORDQ=0** variants which are defined in the standard:
- `flash,<MIN>,<message>` - Flash With Info with character message
- Standard Alert With Info for Caller ID (handled automatically during MT call setup)

The CRI/TCI commands (`flashcri`, `flashtci`, `alertcri`, `alerttci`) are provided for experimentation but should not be relied upon for production use without verification against specific mobile station models.

---

## Implementation Priority

### High Priority (Common Use Cases)
1. ~~**Reorder (Order 4)**~~ ✅ Implemented - Better user feedback on congestion
2. ~~**Stop Alert (Order 6)**~~ ✅ Implemented - Graceful handling of abandoned calls
3. ~~**Message Waiting (Order 5)**~~ ✅ Implemented - Voicemail integration

### Medium Priority (Enhanced Features)
4. ~~**Intercept (Order 9)**~~ ✅ Implemented - Better invalid number handling
5. ~~**Maintenance (Order 10)**~~ ✅ Implemented - Silent testing capability
6. **Directed Retry (Order 12)** - Multi-cell load balancing (FOCC order, complex)

### Low Priority (Specialized)
7. ~~**Serial Number Request (Order 15)**~~ ✅ TX Implemented - Fraud detection (RX handler pending)
8. ~~**Abbreviated Alert (Order 1, ORDQ=1)**~~ ✅ Implemented - Feature reminders
9. ~~**Send Called-Address (Order 8)**~~ ✅ Implemented - Digit re-request (requires flash)
10. ~~**Local Control (Order 30)**~~ ✅ Implemented - Vendor-specific

---

## Console Commands

```bash
# Audit (idle or in-call)
echo "audit,<MIN>" > /tmp/amps_control

# PCI Query (idle or in-call)
echo "pci,<MIN>" > /tmp/amps_control

# Flash With Info - Character Message (in-call only, ORDQ=0)
echo "flash,<MIN>,<message>[,<pi>,<si>]" > /tmp/amps_control

# Flash With CRI - Charging Rate Indication (in-call only, ORDQ=1)
# cri_data: E1,E2,E3,... (up to 8 elements, 4 digits each)
echo "flashcri,<MIN>,<cri_data>" > /tmp/amps_control
# Example: echo "flashcri,0570000000,0025,0100" > /tmp/amps_control

# Flash With TCI - Total Charging Information (in-call only, ORDQ=2)
# tci_data: R1,R2,R3,R4 (up to 4 rows, 4 digits each)
echo "flashtci,<MIN>,<tci_data>" > /tmp/amps_control
# Example: echo "flashtci,0570000000,0123,4567" > /tmp/amps_control

# Alert With CRI - Alert with Charging Rate (alerting or in-call, ORDQ=1)
echo "alertcri,<MIN>,<cri_data>" > /tmp/amps_control

# Alert With TCI - Alert with Total Charges (alerting or in-call, ORDQ=2)
echo "alerttci,<MIN>,<tci_data>" > /tmp/amps_control

# Standard Alert (in-call only, ORDQ=0)
echo "alert,<MIN>" > /tmp/amps_control

# Abbreviated Alert (in-call only, ORDQ=1)
echo "abbalert,<MIN>" > /tmp/amps_control

# Release (in-call only)
echo "release,<MIN>" > /tmp/amps_control

# Change Power (in-call only, level 0-7)
echo "power,<MIN>,<level>" > /tmp/amps_control

# Send Called-Address / Digits Request (in-call only, Order 8)
# USAGE: Flash -> Dial digits -> Flash -> digits command (within 10 sec of first flash)
# Mobile only responds if sent within 10 seconds of a flash!
echo "digits,<MIN>" > /tmp/amps_control

# Maintenance (in-call only, Order 10) - Silent test, no ring
echo "maintenance,<MIN>" > /tmp/amps_control

# Reorder (in-call only, Order 4) - All circuits busy tone
echo "reorder,<MIN>" > /tmp/amps_control

# Intercept (in-call only, Order 9) - Invalid number tone
echo "intercept,<MIN>" > /tmp/amps_control

# Message Waiting Indicator (in-call only, Order 5)
# count: 0-31 (0=clear, 31=unknown number)
# type: 0=voice, 1=SMS, 2=fax (optional, default=0)
echo "mwi,<MIN>,<count>[,<type>]" > /tmp/amps_control

# Stop Alert (in-call only, Order 6) - Stop ringing, keep channel
echo "stopalert,<MIN>" > /tmp/amps_control

# Silent Page (idle mobile) - Page + Maintenance + Release
echo "silentpage,<MIN>" > /tmp/amps_control

# Serial Number Request (in-call only, Order 15) - Request ESN
echo "esn,<MIN>" > /tmp/amps_control

# Local Control (in-call only, Order 30) - Vendor-specific
# code: 0-31 (manufacturer-defined)
echo "local,<MIN>,<code>" > /tmp/amps_control
```

# Stop Alert (during alerting, Order 6) - Stop ringing without release
echo "stopalert,<MIN>" > /tmp/amps_control
```

---

## References

- TIA/EIA-553-A: Mobile Station - Base Station Compatibility Standard (November 1999)
- Table 3.7.1-1: Order and order qualification codes
- TSB-70: Mobile Station - Land Station Compatibility Specifications
