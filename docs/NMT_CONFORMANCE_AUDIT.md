# NMT 450 conformance audit — osmocom-analog `src/nmt/`

Page-by-page audit of the implementation against the three core NMT 450
air-interface documents, done section by section with the page PNG as ground
truth for every bit field.

**Method.** Walk the normative spec section by section; for each signal, field,
procedure, timer and state, locate the implementation and classify it. Bit
formats and code tables are confirmed on the page PNG
(`docs/nmt-index/pages/<doc>-page-NNN.png`), never from the OCR/native text layer
alone. Page numbers below are the `nmtdoc.py page <doc> N` CLI numbers (not the
printed page number, which is offset by ~12 in 450-1).

**Documents in scope** (user-confirmed):

| Key | Doc | Pages | Role |
|---|---|---|---|
| `450-1-sysdesc-1997` | NMT DOC 450-1 System Description (1997) | 111 | **Primary** — §4 signalling catalog + frame coding, §5 procedures, channel plan |
| `450-3-ms-1995` | NMT DOC 450-3 Mobile Station (1995) | 272 | Mostly **RF/electrical conformance** (TX power, RX selectivity, vibration) — out of scope for a software modem; signalling cross-refs 450-1 |
| `annex24-sms-1998` | 450-3/900-3 Annex 24 — SMS (1998) | 39 | SMS message layout |

**Status legend**

- **IMPL** — implemented and exercised in a normal call flow.
- **PARTIAL** — present but incomplete or gated.
- **OBSERVE** — decoded and logged on reception, never originated (honest for
  frames with no backend/transport here).
- **ABSENT** — decodable/definable but neither acted on nor logged.
- **SCOPED-OUT** — deliberately not implemented because the single-node
  BS+MTX-collapsed design has no transport/backend for it (BS-internal link,
  SS7 inter-exchange, SIS authentication register, charging backend).

Code dir: `src/nmt/` — `nmt.c` (state machine + tx/rx), `frame.c` (encode/decode
+ format table), `frame.h` (message enum), `transaction.c/.h`, `dms.c`, `sms.c`,
`main.c` (control FIFOs).

---

## 1. §4.2 Signal catalog — direction MTX → MS (spec CLI pp.30–32)

| Frame | Signal (spec function) | Dir | Impl status | Code pointer |
|---|---|---|---|---|
| **1a** | Calling channel indication (continuous) | MTX→MS | **IMPL** | `nmt_set_dsp_mode`/idle tx `nmt.c:656`; `1a_a`/`1a_b` variants also sent |
| **1b** | Combined calling+traffic channel indication | MTX→MS | **IMPL** | tx `nmt.c:671` |
| **2a** | Call to MS on calling channel (paging) | MTX→MS | **IMPL** | `tx_mt_paging` `nmt.c:1089` |
| **2b** | Traffic channel allocation on CC | MTX→MS | **IMPL** | `tx_mt_channel` `nmt.c:1227` |
| **2c** | Queueing info to priority MS on CC | MTX→MS | **IMPL** (this session) | `tx_mt_queue` ternary `nmt.c:1174` |
| **2d** | Traffic channel scanning order on CC | MTX→MS | **ABSENT** | no sender; needs multi-TC scanning model |
| **2e** | Alternative type-of-call (combined CC/TC) | MTX→MS | **ABSENT** | ack (10d) handled rx-side; no sender |
| **2f** | Queueing info to ordinary MS | MTX→MS | **IMPL** (this session) | `tx_mt_queue` ternary `nmt.c:1174` |
| **3a** | TC allocation on TC (switch call in progress / power change) | MTX→MS | **IMPL** (this session, SCIP move-call) | `tx_scip_order` `nmt.c:1599` |
| **3b** | Identity request on TC | MTX→MS | **IMPL** | `nmt.c:624` |
| **3c** | TC allocation on TC, short procedure (fast SCIP) | MTX→MS | **ABSENT** | fast variant of 3a; not sent |
| **3d** | Access-channel allocation (AC/TC) | MTX→MS | **ABSENT** | only relevant with AC/TC channel type |
| **4** | Free traffic channel indication (continuous) | MTX→MS | **IMPL** | idle tx `nmt.c:665` |
| **4b** | Access-channel free indication | MTX→MS | **IMPL** | idle tx `nmt.c:668` |
| **5a** | Line signal (L-values) | MTX→MS | **IMPL** | `set_line_signal` `nmt.c:632`; L=3/5/6/9/12/15 sent — per-L audit in §3 |
| **5a(L=0)** | Answer to coin-box + tariff class | MTX→MS | **SCOPED-OUT** | no charging backend / no answer-supervision event (`call_down_answer` empty) |
| **5a(L=12)** | Automatic answer order (data mode, DMS/SMS) | MTX→MS | **IMPL** | DMS/SMS autoanswer path |
| **5b** | Answer to coin-box (tariff) frame variant | MTX→MS | **SCOPED-OUT** | same as 5a(L=0) |
| **5c** | Clearing with call-transfer / message indicators | MTX→MS | **PARTIAL** | sent `nmt.c:1423`; MWI flags wired, call-transfer flag not driven |
| **6** | Idle frame (MTX→MS waiting) | MTX→MS | **IMPL** | `nmt.c:1061/1291/1352/1459` |
| **7** | Authentication request (RAND) | MTX→MS | **SCOPED-OUT** | needs SIS Authentication Register + SAK; unverifiable without SAK handset |
| **8** | Calling line identity (A-number, optionally encrypted) | MTX→MS | **IMPL** | `nmt_encode_a_number` `frame.c:212`, sent in ringing `nmt.c:1350` |

## 2. §4.2 Signal catalog — direction MS → MTX (spec CLI pp.32–34)

| Frame | Signal (spec function) | Dir | Impl status | Code pointer |
|---|---|---|---|---|
| **10a** | Call-ack from MS / priority seizure on CC | MS→MTX | **IMPL** | rx `nmt.c` (2 handlers) |
| **10b** | Seizure from ordinary MS / identity on TC | MS→MTX | **IMPL** | rx (4 handlers) |
| **10c** | Seizure+identity from called MS after scanning order | MS→MTX | **OBSERVE** (this session) | logged; procedure (2d) not driven |
| **10d** | Seizure (alternative call type ack) | MS→MTX | **IMPL** | rx handler |
| **11a** | — (seizure variant) | MS→MTX | **IMPL** | rx (2 handlers) |
| **11b** | Roaming updating seizure on TC / priority call-ack | MS→MTX | **IMPL** | rx `nmt.c:1120`; sets `trans->priority` |
| **12** | Seizure from coin-box MS | MS→MTX | **PARTIAL** | rx sets `subscr.coinbox=1` `nmt.c:737`; special answer sequence not driven (SCOPED-OUT send side) |
| **13a** | Line signal (MS→MTX) | MS→MTX | **IMPL** | rx (5 handlers) |
| **13b** | Answer-ack from coin-box (echoed tariff class) | MS→MTX | **OBSERVE** (this session) | logged `nmt.c:767`-area |
| **14a/14b** | Digit signal (dialled digits, one per frame) | MS→MTX | **IMPL** | rx (2+2 handlers) |
| **15** | Idle frame (MS→MTX waiting) | MS→MTX | **IMPL** | rx (2 handlers) |
| **16** | Authentication response (SRES) | MS→MTX | **OBSERVE** (this session) | logged; auth not implemented |
| **Register recall (L=5)** | Connect MTX register during conversation | MS→MTX | **OBSERVE** | logs "Register Recall is not supported." |
| **MFT in / out** | 1200-Baud↔MFT converter activate/deactivate | MS→MTX | **IMPL** | ACTIVE_STATE_MFT path |
| **Answer** | MS lifted handset after ringing | MS→MTX | **IMPL** | MT-complete path |
| **Clearing / release-guard** | Release from MS | MS→MTX | **IMPL** | 13a clearing path |

## 3. §4.2 — MTX↔BS internal link (spec CLI pp.34–35) — SCOPED-OUT class

The single-node design collapses BS and MTX into one process; these frames have
no signalling transport here. Decode side is covered (OBSERVE); the MTX never
originates them because there is no external BS to command.

| Frame | Signal | Dir | Impl status | Code pointer |
|---|---|---|---|---|
| **20** | Channel activation order (TX/ø/squelch/compander) | MTX→BS | **SCOPED-OUT** | encoder exists `frame.c`; nothing drives it (same-process state set directly in C) |
| **21b** | Signal-strength measurement order (data/idle/free TC) | MTX→BS | **SCOPED-OUT** | — |
| **21c** | Signal-strength measurement order (TC in use) | MTX→BS | **SCOPED-OUT** | — |
| **22** | Other management/maintenance order | MTX→BS | **SCOPED-OUT** | — |
| **25** | Channel status / SCC info | BS→MTX | **OBSERVE** (this session) | logged in `nmt_receive_frame` |
| **26** | Signal-strength measurement result | BS→MTX | **OBSERVE** (this session) | logged |
| **27** | Response on management/maintenance order | BS→MTX | **OBSERVE** (this session) | logged |
| **28** | Other maintenance info from BS (alarms) | BS→MTX | **OBSERVE** (this session) | logged |

## 4. §4.2.5 — Test channel

| Frame | Signal | Dir | Impl status | Code pointer |
|---|---|---|---|---|
| **30** | Test channel indication | MTX→MS | **IMPL** | tx `nmt.c:674`; rx handled |

---

## 5. §4.3.2 Frame-format table — verification (ground truth on PNG)

The code's `nmt_frame[]` format table (`frame.c:300+`) was cross-checked against
the §4.3.2 frame-format bit-grids. **Every prefix matches** the spec; the 1a/1b/2a
field layouts were PNG-confirmed exact (CLI p.37).

| Frame | Spec P | Code P | Field layout | Match |
|---|---|---|---|---|
| 1a | 12 | 12 | NNNPYYHHHHHHHHHH | ✅ PNG-confirmed |
| 1b | 4 | 4 | NNNPYYHHHHHHHHHH | ✅ |
| 2a | 12 | 12 | NNNPYYZXXXXXXHHH | ✅ PNG-confirmed |
| 2b | 12 | 12 | NNNPYYZXXXXXXnnn | ✅ |
| 2c/2d/2f | 12 | 12 | NNNPYYZXXXXXXHHH | ✅ |
| 3a | 5 | 5 | NNNPYYZXXXXXXnnn | ✅ |
| 3b | 5 | 5 | NNNPYYZXXXXXXHHH | ✅ |
| 3c | 9 | 9 | NNNPYYZXXXXXXnnn | ✅ |
| 4 | 3 | 3 | NNNPYYJJJJJJJHHH | ✅ |
| 5a | 6 | 6 | NNNPYYZXXXXXXLLL | ✅ |
| 5b | 6 | 6 | NNNPYYZXXXXXXLQQ | ✅ |
| 5c | 2 | 2 | NNNPYYZXXXXXXFFF | ✅ |

**A-number + W checksum (§4.3.3.13, CLI p.67, PNG-confirmed):** checksum = mod-256
sum of MH₁/H₂H₃/H₄H₅/H₆H₇; H₁=sequence length, H₂=type (0 unknown/1 nat/2 int/3
not-possible/4 not-allowed). `nmt_encode_a_number` (`frame.c:212`) matches exactly.
**IMPL, correct.**

**Channel-number coding (§4.3.3.1 Note 1, CLI p.46):** power bits `(v&0x600)>>9`,
hundreds `(v&0x100)>>8`, 200-bit `(v&0x800)>>11` — all match. The interleaved
Note-3 scheme and channel-400='39A' coding do **not** match (see §9 and the Gap
summary — a real finding).

## 6. Line-signal L(n) values & acceptance rule (§4.7.2, CLI pp.105–107)

L-value usage confirmed on PNG: **L(0)**=answer-to-coin-box (+Q₁Q₂), **L(9)**=ringing
order, **L(15)**=forced release. Code `set_line_signal` sends L=3 (proceed-to-send),
5 (compandor-in), 6, 9 (ring), 12 (data autoanswer), 15 (release); receives L=1
(release-guard), 4/10 (MFT in/out), 14 (answer).

**Acceptance rule (§4.7.2.3):** a line signal is accepted if "at least two of three
L(n) characters are equal and meaningful" (MTX→MS, 5a) / "at least three characters
equal" (MS→MTX, 13a). **FIXED** (branch `fix/nmt-linesignal-majority-and-ch400`):
`nmt_line_signal_majority()` (`frame.c`) now implements the ≥3-of-5 majority vote,
replacing the former all-5-nibbles-equal check at the 5 × 13a RX sites
(`nmt.c:1317, 1368, 1521, 1713, 1945`). Previously the code required all five equal
and rejected noisy-but-valid frames the standard accepts.

---

## 7. §5 / §4.4 Call-processing procedures & state machines

_(audited against `nmt.c` state machine; frame orders PNG-confirmed per sequence)_

The full procedure matrix (MO Scheme A, MT Scheme B, roaming Scheme D, release,
supervisory/SAT, queueing, SCIP Scheme C2', MFT/MWI) is below. **Frame order
matches the spec for MO, MT, release and queueing.** Deviations found in roaming
confirmation, release line-signal values, and SCIP:

### 7.1 MO call (Scheme A, CLI p.70) — order matches ✅
seizure 10b/12 → ident-req 3b → ident 10b → proceed 5a(L=3) → dial tone →
digits 14a/14b → idle 15 → complete 5a(L=5) → active. All **IMPL**. Auth loop
(3b+7, expect 16) **not realised** — frame 7 SCOPED-OUT, 16 OBSERVE.

### 7.2 MT call (Scheme B, CLI p.71) — order matches ✅
page 2a → ack 10a/11b → assign 2b (or queue 2c/2f) → ident 3b/10b → ring 5a(L=9)
→ A-number (frame 8, optional B3) → answer 13a(L=14) → complete 5a(L=5) → active.
All **IMPL** incl. SMS/DMS autoanswer (L=12) and PMS priority (11b). 10c scanning-
answer **OBSERVE**.

### 7.3 Roaming updating (Scheme D, CLI p.77) — PARTIAL
11a → 3b → 11a → confirmation → forced release → guard L(1). Sequence matches, but
**confirmation sends L(3) where the diagram shows 5a(L=13/15)** — flagged, L-value
UNCONFIRMED against the detailed L-code table. Home-register update SCOPED-OUT.

### 7.4 Release both directions (§4.4.1.3, CLI p.73) — PARTIAL
MS-clearing 13a(L=1)→release; MTX-clearing L(15)→guard L(1). Sequence/direction
match; **release emits L(15) where the diagram shows 5a(L=13/15)** (flagged).
RELEASE_TO=2s matches "max 2s". Stop-ø to BS (20/25) SCOPED-OUT.

### 7.5 Supervisory / SAT (§4.5) — IMPL (MS side)
ø-signal supervision in STATE_ACTIVE, loss→timeout→release(TEMPFAIL). MTX→BS
ø-order (20) SCOPED-OUT; S/N report (25/26) OBSERVE.

### 7.6 Queueing (§4.4.1.6/7) — IMPL ✅
2a→10a→(no TC)→2c/2f loop→2b. `STATE_MT_QUEUE`, QUEUE_TO=60s. Matches note "2b
sent instead of 2c once idle TC available".

### 7.7 SCIP channel-change (Scheme C2', CLI pp.76/107) — now implemented in full
The implemented flow is the **short procedure (Scheme C2', §4.4.1.4.3)**, now following
the standard's full step sequence (branch `fix/nmt-scip-conformant`):
- **order (step 6):** frame **3c** ×4 on the old TC (prefix 9, channel-number field =
  target TC per §4.7.2.4b N₁'N₂'N₃' = NₐNᵦN_c);
- **target armed (steps 4-5):** the target TC radiates its **ø-signal while awaiting the
  MS** (dsp.c emits supervisory in `DSP_MODE_FRAME` for `STATE_SCIP_IDENT`/`_REQ`), so the
  MS retunes onto a supervised channel, not a free one — the AMPS prepare-target-first
  discipline, now present;
- **old TC held (step 8):** the old TC keeps the transaction + speech, supervised, until
  the MS re-identifies — only then is it released (step 11);
- **re-request (step 9):** if no 10b within **T' = 553 ms (4 frames)**, the MTX sends
  **3b** identity requests on the new TC (`STATE_SCIP_IDENT_REQ`) for another T';
- **failure revert:** if the MS still does not appear, the call **stays on the old TC**
  (speech throughconnected there, never torn down) and the new TC is **forced-released** —
  per the standard, not a call drop.

Timers match the spec (T' = 4 frames). **Verification:** builds clean, existing NMT tests
pass; this is a **code-path implementation verified by build + state-machine trace audit**,
NOT an on-air/handset-verified handshake (no scripted MS exists in the test harness).
Remaining non-conformances: operator-triggered rather than signal-strength-automatic (the
measurement-driven trigger needs the BS link, SCOPED-OUT); the MS-side RF-level-A decision
is the phone's, not ours.

### 7.8 MFT / MWI — IMPL
MFT-in L(4), MFT-out L(10); MWI 5c×2 then L(15)×4 per §4.4.1.16. All **IMPL**.

---

## 8. SMS / DMS data services (annex24-sms-1998, 39 pp) vs `sms.c`/`dms.c`

Core SMS transport is **IMPL**: SMS-DELIVER (TX), SMS-SUBMIT (RX decode),
SUBMIT-REPORT, the RP relay layer (MO/MT DATA/ACK/ERROR), DMS link layer (ID/RAND,
RR, 7/8-bit, CRC-16, trailing-zero handling), TP-FCS cause values, SCTS/address
semi-octet coding — all PNG-confirmed. SMS-STATUS-REPORT / SMS-COMMAND are
**SCOPED-OUT** (spec: "not used in this phase"). DMS scrambling/encryption
SCOPED-OUT (ANNEX 21, single-node).

**Real gaps (see Gap summary §10):** outgoing length capped at 140 (spec 160/255);
TP-VP absolute 7-octet form (desync risk); 8-bit NMT alphabet on TX; SMS-call
timers; NR flow-control; outgoing length cap; and the RP-User-Data IEI question
have all since been addressed — see the itemised status in §10. Remaining minor
items: header spare bytes hard-coded, and the MO/MT macro names for RP-ACK/ERROR
swapped (wire values correct, labels misleading).

---

## 9. Channel plan / frequencies / traffic areas vs `countries.c`

Scandinavian 450 plan **IMPL and in-band**: downlink 463–467.475 (ch1–180),
uplink 453.0–457.475 (10 MHz duplex), 25 kHz spacing, extended band ch181–200
(452.5–452.975 / 462.5–462.975), 20 extended channels. Z-nationality Z(5)=DK /
Z(6)=SE / Z(7)=NO / Z(8)=FI and TA-byte = (Z<<4)|area all **IMPL/correct** (PNG
p.47). 900 interleave flag carried in TA field.

**FIXED** (branch `fix/nmt-linesignal-majority-and-ch400`): the 450 path of
`nmt_encode_channel`/`nmt_decode_channel`/`nmt_encode_tc`/`nmt_decode_tc` now
implements the §4.3.3.1 **Note-3 two-bit base selector** (N1 bits 1+4 → base
0/100/200/300) with **channel 400 = '39A' = 0x99A**. Verified: channels 1–399
encode to identical values as before (backward-compatible with the shipped
Scandinavian plan, which tops at 399), and only channel 400 changed from the former
buggy 0x9A0 to the spec 0x99A. (Was: code modelled hundreds as two additive flags
0x100/0x800, emitting 0x9A0 for channel 400 — self-consistent internally but
non-conformant to the 450-1 Note-3 '39A' signalling.)

---

## 10. Consolidated gap summary — real conformance deviations

Everything else is either **IMPL**, correctly **OBSERVE** (decoded+logged, no
originate), or **SCOPED-OUT** by the single-node BS+MTX design (auth register,
charging backend, BS-internal link 20/21/22/25/26/27/28, SS7 inter-exchange). The
genuine deviations, worst-interesting first:

### Behavioural / on-air coding (would matter against a real NMT device)
1. **Channel-400 / interleaved '39A' coding** — **FIXED** (branch
   `fix/nmt-linesignal-majority-and-ch400`): `nmt_encode_channel`/`_tc` + decoders
   now use the Note-3 two-bit base selector; ch400 = 0x99A; channels 1–399
   unchanged. §9.
2. **Line-signal acceptance stricter than spec** — **FIXED** (same branch):
   `nmt_line_signal_majority()` implements the ≥3-of-5 vote at 5 RX sites. §6.
3. **SCIP now implements Scheme C2' in full** — **DONE** (branch
   `fix/nmt-scip-conformant`): 3c order, target ø-signal armed before the MS arrives, old
   TC held until re-identify, 3b re-request after T', and revert-to-old-TC + forced-release
   on failure (no call drop). Code-path verified (build + trace audit), not handset-tested.
   Only the measurement-driven *automatic* trigger remains out (needs BS link, SCOPED-OUT).
   §7.7.
4. **Release / roaming-confirmation line-signal values** (L=15 release, L=3 roaming
   confirm) differ from the diagrams' 5a(L=13/15) — needs L-code-table confirm. §7.3/7.4.

### SMS (verifiable against captures)
5. **Outgoing SMS length cap** — **FIXED** (branch `fix/nmt-sms-length-timers`): raised
   from 140 to the spec **160 characters** (§2.2.3.13; 7-bit → 140 octets, fits data[256]).
   8-bit NMT-alphabet and the optional 255-char case remain unimplemented (gap #8).
6. **TP-VP absolute 7-octet form** — **FIXED** (branch `fix/nmt-sms-length-timers`): the
   SUBMIT decoder now skips the VP by its true length (1 octet integer/relative, 7 octets
   absolute semi-octet) per §2.2.3.12, so VPF=11 no longer desyncs the parse. Also hardened
   the whole SUBMIT decode: RP-header entry guard, per-coding length validation against the
   160/255 ceilings with rejection+log, and a char-bounded `decode_message_7` that cannot
   overflow on a trailing partial septet. §8.
7. **SMS-call timers** — **FIXED** (branch `fix/nmt-sms-length-timers`): mapped to the
   §1.1 values — 20 s awaiting a response to a sent SMS packet, 10 s awaiting the next DMS
   frame, and ≥5 s before clearing after the last packet (was a single 5 s/2 s pair). §8.
8. **8-bit NMT-alphabet SMS** — **FIXED** (branch `fix/nmt-sms-8bit`): `sms_deliver` now
   auto-selects coding by content — any byte ≥ 0x80 (a national letter) is sent with the
   NMT 8-bit alphabet (TP-DCS 0x30, NMT Set 0, one octet per character, §4.3); plain ASCII
   still uses GSM 7-bit. Receive already decodes both. Verified by a round-trip test (8-bit
   DELIVER encode + 8-bit SUBMIT decode). The operator SMS interface is now **length-
   carrying** (`sms_deliver`/`sms_submit`/`sms_string` keep an explicit byte count, not a C
   string), so a message byte **0x00** is preserved end to end; on the text FIFO any
   non-printable byte is written as the escape **`<0xNN>`** both when sending and when
   receiving, making all 256 code points operator-accessible (round-trip proven by
   `test_sms_escape`). The glyph tables for character sets 1–3 (Baltic/Central/Western) are
   the handset's; the SC carries octets verbatim.
9. **DMS NR flow-control** — **FIXED** (branch `fix/nmt-dms-nr`): a received NR
   (receive-not-ready) frame now acknowledges up to (n-1) like RR (shared `dms_ack_upto`)
   AND sets a `peer_busy` condition that holds back new I-frames in
   `trigger_frame_transmission` (a pending RR still goes, being our own ack); a later RR
   clears it. RR ack behaviour unchanged (test_dms passes after the refactor); the NR path
   reuses that tested ack logic plus the busy gate. §8.
10. **RP-User-Data IEI 0x41 vs 0x42** — **RESOLVED** (branch `fix/nmt-rp-iei-doc`): the
    Annex 24 figures (§2.4.3.2/2.4.3.3) draw BOTH the RP-User-Data IEI and the RP-Cause IEI
    as 0x42, which is a documentation typo — two IEs cannot share one identifier. Annex 24
    references GSM 04.11, where RP-User-Data = **0x41** and RP-Cause = **0x42**; the code
    already uses those (distinguishable, interoperable), and the comment now explains why
    rather than just calling the doc "wrong". No code value change. §8.

### Decode fidelity (dormant / cosmetic)
11. **Frame 11b format string carries an extra `Y`** — mis-decodes one idle nibble
    as traffic-area (11b never originated; low impact). §4-fields.
12. **Frame 16 SRES** decoded as flat 12-digit value, not R1R2R3R4×3 with cross-copy
    vote (dormant while auth is logging-only).
13. **Battery-saving frames** (H7 group / H3 period / Y3 operator-id / H8-10 RTC)
    collapse to frame 1a — neither decoded nor logged.
14. **Frame 25 cause value / BS alarm levels** logged as raw hex, not expanded.
15. **RP-ACK/ERROR MO/MT macro names swapped** vs spec convention (wire correct).

### UNCONFIRMED (honest limitations of this audit)
- Roaming/release L-code semantics (L=13 vs 15 vs the "/" notation) not re-confirmed
  on the detailed §4.3.3 line-signal coding table.
- Battery-saving H3 period table read from native text only, not PNG.
- No end-to-end on-air test performed anywhere — all statuses are code-vs-spec
  reads plus the clean build + NMT unit tests, not handset-verified.

---

_Audit complete. §1–§4 = signal catalog; §5–§6 = frame formats + line signals;
§7 = procedures/state machines; §8 = SMS/DMS; §9 = channel plan; §10 = gap summary.
Ground truth: NMT 450-1/450-3/annex24 page PNGs. Code: `src/nmt/` at the current
master._
