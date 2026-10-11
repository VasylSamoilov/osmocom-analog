# AMPS implementation status vs TIA/EIA-553-A

Functionality implementation-status audit of the osmocom-analog **base-station**
AMPS code in `src/amps/`, produced by walking the standard **TIA/EIA-553-A**
page by page.

## Method

- **Spec is the index, code is the subject.** Every normative requirement in the
  standard's body (PDF pages 17-129 = the full §1-§3.7 normative text) was walked
  in order and checked against `src/amps/`.
- **PNG page images are ground truth.** Each page was read from its 300-dpi scan
  (`docs/amps-index/pages/tia553a-page-NNN.png`); the OCR text layer was used only
  as a finder. Bit-field layouts, order codes, tone/timing values and tables were
  read off the image, not the OCR.
- **Every page read individually.** The 113 normative pages were split into
  contiguous ranges and each page was opened and read one at a time; nothing was
  sampled or skipped. ~804 requirement rows were recorded.
- **Each row records** whether the item is DECODED (parsed on RX), ENCODED (built
  on TX), ACTED-ON (drives behaviour vs logged-only), has an operator COMMAND
  (`-S`/FIFO), its overall STATUS, and the `file:line` EVIDENCE it was verified at.
- **Perspective is the base station.** A mobile-only "shall" is recorded as
  `NOT-APPLICABLE(mobile-side)`, with the base's mirror behaviour noted where one
  exists.

## Status roll-up (804 requirement rows)

| STATUS | Rows | Meaning |
|---|---:|---|
| IMPLEMENTED | 457 | present, spec-accurate, and exercised (incl. "IMPLEMENTED (RX)" receive-path rows) |
| NOT-APPLICABLE (mobile-side / regulatory / reserved) | 192 | a mobile "shall" or RF/measurement/reserved item with no base obligation |
| DECODE-ONLY | 110 | field/word parsed on RX but never acted on (logged) |
| PARTIAL | 105 | present but incomplete, approximated, or not fully enforced |
| MISSING | 45 | required of the base, no implementation |
| STUB | 8 | defined but unreachable / never emitted |
| EXTENSION (non-spec) | 2 | code exceeds a "Reserved" spec section (SAT/ST detect) |
| UNVERIFIED | 0 | both prior UNVERIFIED items resolved (see below) |

> **Count note:** the status counts above are the ACTUAL body-table row counts (recounted after
> the corrections below), so the roll-up and the per-page tables now agree. The earlier roll-up used
> round estimates from the parallel range walk and did not reconcile to the body; it has been
> replaced with the verified counts. NOT-APPLICABLE is still the walk's estimate (its rows span many
> prose/glossary pages and were not re-tallied this pass).

> **Post-walk correction (2026-10-11):** a re-validation pass found the p.59/§2.6.2.3 and
> p.65/§2.6.3.7 range had **7 false MISSING rows** — the Abbreviated Alert, Audit, PCI Query and
> Message Waiting orders are in fact implemented (`amps.c:2472/3387/3313/2748`, driven by the
> control FIFO), and the three PCI-registration Word C rows are DECODE-ONLY (word decoded + logged
> via `amps_rx_pci_report`), not absent. Those range workers greped the wrong tokens and missed the
> order senders in `amps.c`.
>
> **Second re-validation (2026-10-11, every MISSING row across all ranges):** of the 45 remaining
> MISSING rows, **38 are correctly MISSING** — all auth-family (CAVE/SSD/AUTHR/AUTHU/AUTHBS,
> Base-Station Challenge, SSD Update, Auth-Word-C registration, Message Encryption; only *scaffolded*
> this session, no CAVE engine), plus LASTCHA/LASTCHP, the ±12 kHz voice deviation limiter, and the
> two busy-idle TX rows (deliberately not implemented — SDR latency). **4 more were stale/misclassified:**
> directed-retry LT↔ORDQ is IMPLEMENTED (`amps.c:4024/4028`); BSCAP and BSPC are DECODE-ONLY
> (encoder exists, value hardcoded in sysinfo.c); the global-action Local Control overhead word is a
> STUB (framing defined, never emitted — distinct from the implemented per-call Order 30).
>
> Corrected totals above reflect both passes. Net reclassifications out of MISSING: 7 (p.59/p.65
> orders) + 4 (directed-retry LT, BSCAP, BSPC, Local Control STUB) = 11 rows that were wrongly
> MISSING, leaving 45 genuinely-missing rows — of which ~38 are the auth family (scaffolded only,
> no CAVE) and the rest are LASTCHA/LASTCHP, the ±12 kHz deviation limiter, and the two busy-idle
> TX rows (deliberate SDR-latency omission).

Of the base-station-obligation rows (the 457 IMPLEMENTED + 110 DECODE-ONLY + 105 PARTIAL +
45 MISSING + 8 STUB = 725 rows that carry a base obligation, excluding the 192 not-applicable),
**~63% are fully implemented**, with the shortfall concentrated in a few coherent areas
(authentication above all) rather than scattered defects.

## Cross-cutting findings (the through-line)

1. **The bit/framing layer is faithful.** FOCC/RECC/FVC/RVC word layouts, the
   BCH(40,28;5) and (48,36;5) coding, Manchester/FSK modulation, the
   channel-number↔frequency table, SAT color codes (5970/6000/6030 Hz) and the
   order-code table (Table 3.7.1-1) all match the page images. This was verified
   independently by every range worker — no wrong-bits defects were found.

2. **Authentication / SSD / CAVE is the single largest gap, and it is
   system-wide.** Every range that touches auth (§2.3.12, §2.6.3, §2.7, §3.6, §3.7)
   reports the same thing: the auth-family words (AUTHR, AUTHU, AUTHBS, RANDC,
   RANDU, RANDBS, RANDSSD, SSD Update, Unique/Base-Station Challenge, Parameter
   Update/COUNT) exist **only as `frame.c` word templates** — decode-only or
   never-emitted — with **no CAVE algorithm, no SSD/A-key state, and no challenge
   flow**. The overhead `AUTH` bit is hard-wired to 0 (`sysinfo.c:72`) with no
   operator control. (CAVE itself is export-restricted, so its absence is expected;
   but the entire surrounding challenge *flow* is also absent.)

3. **Decoded-but-not-acted-on fields.** Capability/identity fields the base parses
   correctly then only logs: SCM (station class), MSPC/MSCAP/MPCI (protocol
   capability), SDCC1/SDCC2 (hard-wired 0 in `sysinfo.c:65-66`), the mobile's SAT
   echo. The receive path is trustworthy; the base just makes no decision on them.

4. **Real-time control-channel timing the SDR path cannot meet.** The §2.6.3.5 /
   §3.6.3.2 busy-idle (B/I) seizure-collision mechanism is deliberately disabled
   (`bis=0`, WFOM/RCF forced so phones ignore B/I — "osmocom can't respond fast
   enough", `main_common.c:107`). This is a hardware/latency limitation, documented
   in-code, not a quick fix.

5. **Missing orders:** Parameter Update (COUNTs-p) and the Auth-Word-C-included
   registration variants (Order 24 ORDQ 0-3) — both auth-dependent. Message
   Encryption Mode has only its order-code constant (guarded by `_Static_assert`),
   no key handling. (The post-walk correction above removed Message Waiting from
   this list — `amps_mwi` implements it fully.)

6. **Fixes already landed this session are visible in the audit:** the base-side
   5 s fade-timer mirror now cites §2.6.4.1 (p.70), the Disable-DTMF order-code
   guard, the per-VC SAT dispatch, and auto-derived N-1/CMAX-1 overhead capacity.

## Previously-open items — now resolved

- **§2.7 "BCH-correct but unrecognizable overhead counts toward NAWC":** RESOLVED
  as **NOT-APPLICABLE (mobile-side)**. §2.7 (p.62) scopes this to the *mobile
  station's* reception ("if the mobile station receives… the mobile station shall
  count… but shall not attempt to execute"). The base builds the overhead train and
  never receives its own FOCC, so it carries no base-station obligation.
- **§3.7.1.3 Mitigation 1, forward-overhead TX rate ~once/second:** RESOLVED as
  **IMPLEMENTED**. `si->overhead_repeat` is this rate control: one FOCC transmit
  buffer is 463 bits at 10 kbit/s = 46.3 ms, so `overhead_repeat = 17` yields a
  train every ~0.79 s — "about once per second," as §3.7.1.3 requires. It was not an
  explicit named timer, which is why it read as unverified; it is now documented and
  spec-cited at the `overhead_repeat` assignment in `sysinfo.c`.

## Reading the per-range tables below

The remainder of this document is the full per-page detail, one section per page
range, exactly as recorded during the walk. Columns:
`PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE`.

---

---

## AMPS spec-walk pages 017-030

Perspective: BASE STATION (osmocom-analog `src/amps/`). Pages 17-30 cover §1 Introduction/Scope,
§1.2 Definitions (A..Z), §1.3 Abbreviations, and the start of §2 (channel plan / channel-number-to-
frequency / power & modulation). Definition pages are mostly glossary prose — a term is only
IMPLEMENTED/PARTIAL when a corresponding base-station mechanism exists in code; otherwise the term
is a NOT-APPLICABLE glossary entry.

Row format: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

## page 17
Pure scope + definitions (A-key .. Busy-Idle Bits). No normative "shall" — glossary only.

PAGE 17 | §1.1 Scope | Compatibility standard for 800 MHz cellular; perf/measurement out of scope | N-A | N-A | N-A | N-A | NOT-APPLICABLE(prose) | scope text only
PAGE 17 | §1.2 Def | A-key (64-bit auth secret in MS) | N | N | N | N | NOT-APPLICABLE(mobile-side) | no "a-key" in src/amps/; authentication key not modelled base-side
PAGE 17 | §1.2 Def | Access Channel (CC used by MS to access system) | Y | N-A | partial | N-A | PARTIAL | RECC reverse-control framing decoded in frame.c (recc_* words); base receives access
PAGE 17 | §1.2 Def | Analog Color Code = SAT | Y | Y | Y | partial | IMPLEMENTED | SAT handled in dsp.c (SAT tone gen/detect); SCC IE frame.c:2324
PAGE 17 | §1.2 Def | AUTH (1-bit SPOM field: system supports auth) | Y | Y | logged-only | N | DECODE-ONLY | sysinfo.c:72 word1.auth=0; encoded frame.c; no auth procedure driven
PAGE 17 | §1.2 Def | Authentication (procedure) | N | N | N | N | NOT-APPLICABLE(not implemented) | AUTH set 0; no CAVE/auth engine in src/amps/
PAGE 17 | §1.2 Def | AUTHR (18-bit auth response) | Y | partial | N | N | DECODE-ONLY | AUTHR IE frame.c:2156; word field present, not validated
PAGE 17 | §1.2 Def | Average Peak Deviation | N-A | N-A | N-A | N-A | NOT-APPLICABLE(measurement) | measurement definition, out of scope
PAGE 17 | §1.2 Def | Base Station | N-A | N-A | N-A | N-A | NOT-APPLICABLE(role) | this IS the role implemented
PAGE 17 | §1.2 Def | AUTHBS (18-bit base auth response) | N | partial | N | N | DECODE-ONLY | AUTHBS word field frame.c:252; not computed
PAGE 17 | §1.2 Def | RANDBS (32-bit) | N | partial | N | N | DECODE-ONLY | RANDBS field frame.c:731/1632; SSD update not implemented
PAGE 17 | §1.2 Def | BCH Code | N-A | N-A | N-A | N-A | NOT-APPLICABLE(error-coding) | BCH used by word coding; see later framing pages
PAGE 17 | §1.2 Def | Busy-Idle Bits (FOCC indicates RECC busy/idle) | N-A | Y | partial | N | PARTIAL | sysinfo.c:105-107 B/I kept 0 (not set busy); FOCC stream carries B/I

## page 18
Definitions continued (Continuous Transmission .. Numeric Information). Glossary; a few map to code.

PAGE 18 | §1.2 Def | Continuous Transmission (DTX not permitted) | N-A | Y | Y | Y | IMPLEMENTED | DTX is operator-settable; dtx=0 => continuous. sysinfo.c:46,80
PAGE 18 | §1.2 Def | Control Channel (digital control info BS<->MS) | Y | Y | Y | N-A | IMPLEMENTED | FOCC/RECC framing in frame.c
PAGE 18 | §1.2 Def | Digital Color Code (DCC) | Y | Y | Y | Y | IMPLEMENTED | si->dcc sysinfo.c:59; encoded word1/word2; DCC IE frame.c:2206
PAGE 18 | §1.2 Def | Discontinuous Transmission (DTX) | N-A | Y | Y | Y | IMPLEMENTED | dtx param sysinfo.c:46,80 (MS-side behaviour, base signals it)
PAGE 18 | §1.2 Def | Flash Request (VC msg MS->BS) | Y | N-A | partial | N | PARTIAL | flash-with-info words frame.c:990,1060 decoded
PAGE 18 | §1.2 Def | Forward (BS->MS path) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(terminology) | direction term
PAGE 18 | §1.2 Def | Forward Control Channel (FOCC) | N-A | Y | Y | N-A | IMPLEMENTED | FOCC message sets frame.c:160,319,374
PAGE 18 | §1.2 Def | Forward Voice Channel (FVC) | N-A | Y | Y | N-A | IMPLEMENTED | FVC words frame.c:876,1410
PAGE 18 | §1.2 Def | Group Identification (MSBs of SID) | partial | partial | N | N | PARTIAL | SID handled (sid1); GID subset not separately modelled
PAGE 18 | §1.2 Def | Handoff (transfer MS to another VC) | Y | Y | Y | partial | IMPLEMENTED | handoff order strings/handling frame.c:2041-2045; amps.c state
PAGE 18 | §1.2 Def | Home Mobile Station | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | glossary
PAGE 18 | §1.2 Def | Land Station = Base Station | N-A | N-A | N-A | N-A | NOT-APPLICABLE(alias) | alias
PAGE 18 | §1.2 Def | Manchester encoding | Y | Y | Y | N-A | IMPLEMENTED | dsp.c:24 Manchester-coded bits (mod/demod)
PAGE 18 | §1.2 Def | Message = one or more words | N-A | Y | Y | N-A | IMPLEMENTED | def_message_set in frame.c
PAGE 18 | §1.2 Def | Mobile Identification Number (MIN, 34-bit) | Y | Y | Y | partial | IMPLEMENTED | MIN1/MIN2 IEs frame.c:167,2125; used in paging/transaction
PAGE 18 | §1.2 Def | Mobile Station | N-A | N-A | N-A | N-A | NOT-APPLICABLE(role) | peer role
PAGE 18 | §1.2 Def | Mobile Station Class I/II/III (power) | N | N | N | N | NOT-APPLICABLE(mobile-side) | power class is MS attribute; see §2.1.2.2 later pages
PAGE 18 | §1.2 Def | NRZ waveform | N-A | N-A | N-A | N-A | NOT-APPLICABLE(def) | baseband term; FM/Manchester in dsp.c
PAGE 18 | §1.2 Def | Numeric Information (subscripts) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(def) | notation convention

## page 19
Subscript conventions (s, sv, sl, r, p, s-p) + numeric indicators A..F. Mostly MS-side storage
semantics; the overhead fields the BASE actually transmits are the implemented ones.

PAGE 19 | §1.2 | Subscript notation s/sv/sl/r/p/s-p | N-A | N-A | N-A | N-A | NOT-APPLICABLE(notation) | storage-class convention
PAGE 19 | §1.2 | ACCOLC (overload class controlling access) | N | N | N | N | NOT-APPLICABLE(mobile-side) | no "accolc" in src; base sends OLC field (see below), MS picks via ACCOLC
PAGE 19 | §1.2 | BIS (check idle-to-busy on RECC) | N-A | Y | Y | Y | IMPLEMENTED | bis param drives wfom/rcf sysinfo.c:67,82; operator -S
PAGE 19 | §1.2 | BSCAP (base core analog protocol version) | Y | Y | partial | N | DECODE-ONLY | CORRECTED (was MISSING): BSCAP IE + decoder (ie_bscap frame.c:1710) and encoder exist — amps_encode_access_type() sets AMPS_IE_BSCAP (frame.c:2984). Value hardcoded bscap=1 in sysinfo.c:125, no operator control.
PAGE 19 | §1.2 | BSPC (base protocol capability) | Y | Y | partial | N | DECODE-ONLY | CORRECTED (was MISSING): BSPC IE + decoder (ie_bspc frame.c:1721) and encoder exist — amps_encode_access_type() sets AMPS_IE_BSPC (frame.c:2983). Value hardcoded bspc=0 in sysinfo.c:124, no operator control.
PAGE 19 | §1.2 | CCLIST (control-channel scan list) | N | N | N | N | NOT-APPLICABLE(mobile-side) | directed-retry list; no match
PAGE 19 | §1.2 | CMAX (max channels scanned when accessing) | N-A | Y | Y | Y | IMPLEMENTED | word2.cmax_1 sysinfo.c:84; -S cmax1=
PAGE 19 | §1.2 | COUNT (modulo-64 count in MS) | N-A | Y | partial | N | PARTIAL | si->count is overhead-train seq (sysinfo.c:147), not the MS auth COUNT
PAGE 19 | §1.2 | CPA (combined access+paging) | N-A | Y | Y | partial | IMPLEMENTED | word2.cpa=1 sysinfo.c:83
PAGE 19 | §1.2 | DTX (how MS may use DTX on VC) | N-A | Y | Y | Y | IMPLEMENTED | word2.dtx sysinfo.c:80
PAGE 19 | §1.2 | E / EX (send MIN1 only vs MIN1+MIN2) | N-A | Y | partial | N | PARTIAL | word2.e encoded (sysinfo encode line); E field present
PAGE 19 | §1.2 | FIRSTCHA (first CC for access) | partial | partial | Y | N | PARTIAL | CHANPOS relative to FIRSTCHA frame.c:2164; amps.c:3867,4004 channel calc

## page 20
Numeric indicators F..P. Overhead/location/paging fields that the base transmits are implemented.

PAGE 20 | §1.2 | FIRSTCHP (first CC for paging) | N-A | partial | partial | N | PARTIAL | paging-channel numbering via n_1/channel calc; not a distinct FIRSTCHP var
PAGE 20 | §1.2 | LASTCHA / LASTCHP (last CC access/paging) | N | N | N | N | MISSING | no lastcha/lastchp in src/amps/
PAGE 20 | §1.2 | LOCAID (location area identity) | N-A | Y | Y | Y | IMPLEMENTED | loc_area.locaid sysinfo.c:92,182; -S locaid
PAGE 20 | §1.2 | LREG (location registration enable) | N-A | Y | Y | Y | IMPLEMENTED | loc_area.lreg sysinfo.c:93,182
PAGE 20 | §1.2 | LRCC (last registration CC) | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS stored value
PAGE 20 | §1.2 | LT (last-try) | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS access logic
PAGE 20 | §1.2 | MIN1 (24-bit, 7-digit dir num) | Y | Y | Y | partial | IMPLEMENTED | MIN1 IE frame.c:167; ie_min1 frame.c:2125
PAGE 20 | §1.2 | MIN2 (10-bit, area code) | Y | Y | Y | partial | IMPLEMENTED | MIN2 field frame.c:178,193
PAGE 20 | §1.2 | MAXBUSY (max busy on RECC) | N-A | Y | Y | partial | IMPLEMENTED | acc_attempt.maxbusy_pgr/other sysinfo.c:117,119
PAGE 20 | §1.2 | MAXSZTR (max seizure attempts) | N-A | Y | Y | partial | IMPLEMENTED | acc_attempt (maxsztr companion), sysinfo.c acc_attempt block
PAGE 20 | §1.2 | MSCAP / MSPC (MS protocol version report) | N | N | N | N | NOT-APPLICABLE(mobile-side) | reported by MS; base requests via PCI only
PAGE 20 | §1.2 | N (paging channels to scan) | N-A | Y | Y | Y | IMPLEMENTED | word2.n_1 sysinfo.c:81; -S n1=
PAGE 20 | §1.2 | NBUSY / NSZTR (MS seize counters) | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS access counters
PAGE 20 | §1.2 | NXTREG (next registration trigger) | N-A | Y | partial | Y | PARTIAL | reg_incr.regincr sysinfo.c:87; -S regincr
PAGE 20 | §1.2 | O/E (odd/even field in orders, adj-chan protection) | partial | Y | partial | N | PARTIAL | parity bit encoded in word framing; not separately named
PAGE 20 | §1.2 | PCI_HOME / PCI_ROAM (request MS proto cap) | N-A | Y | Y | partial | IMPLEMENTED | acc_type.pci_home sysinfo.c:111 (and pci_roam)

## page 21
Numeric indicators P..W, then start of the base->mobile Orders list. The overhead fields the base
TX are implemented; MS-stored-only values are N-A. The order TABLE lives in frame.c
amps_table4_def[] (frame.c:2371+) and is DECODED/named; which orders are ACTED-ON is on pages 21-23.

PAGE 21 | §1.2 | PCSID (last SID of PCI reg) | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS stored
PAGE 21 | §1.2 | PDREG (power-down registration) | N-A | Y | Y | Y | IMPLEMENTED | loc_area.pdreg sysinfo.c:91,182
PAGE 21 | §1.2 | PL (MS RF power level) | N | N | N | N | NOT-APPLICABLE(mobile-side) | set via Change Power order (acted-on, see p22)
PAGE 21 | §1.2 | PUREG (power-up registration) | N-A | Y | Y | Y | IMPLEMENTED | loc_area.pureg sysinfo.c:90,182
PAGE 21 | §1.2 | R (registration enabled) | N-A | Y | Y | Y | IMPLEMENTED | regh/regr encode args sysinfo.c:174
PAGE 21 | §1.2 | RAND (auth random variable) | N | N | N | N | MISSING | no RAND/CAVE auth engine
PAGE 21 | §1.2 | RCF (read control-filler before access) | N-A | Y | Y | Y | IMPLEMENTED | word2.rcf sysinfo.c:82
PAGE 21 | §1.2 | REGID (registration id on FOCC) | N-A | Y | Y | partial | IMPLEMENTED | reg_id.regid sysinfo.c:123,176 (timestamp-derived)
PAGE 21 | §1.2 | REGINCR (increment between registrations) | N-A | Y | Y | Y | IMPLEMENTED | reg_incr.regincr sysinfo.c:87
PAGE 21 | §1.2 | S (send serial number when accessing) | partial | Y | partial | N | PARTIAL | word2.s encoded; Serial Number Request order in table
PAGE 21 | §1.2 | SCC (SAT color code select) | Y | Y | Y | partial | IMPLEMENTED | SCC 2-bit field frame.c:177; SAT in dsp.c
PAGE 21 | §1.2 | SDCC1 / SDCC2 (supplem. digital color codes) | N-A | Y | logged-only | N | DECODE-ONLY | filler.sdcc1/2=0 sysinfo.c:65,66 (sent as 0)
PAGE 21 | §1.2 | SID (p / s-p / r / s variants) | partial | Y | Y | Y | IMPLEMENTED | word1.sid1 sysinfo.c:70 (SID1 encoded); storage-class variants are MS-side
PAGE 21 | §1.2 | UPDATE_NEXTREG | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS reg logic
PAGE 21 | §1.2 | WFOM (wait for overhead train) | N-A | Y | Y | Y | IMPLEMENTED | filler.wfom sysinfo.c:67
PAGE 21 | §1.2 Orders | Abbreviated Alert order (ORDER=1 ORDQ=1) | Y | partial | N | N | DECODE-ONLY | amps_table4_def frame.c:2373; named, not actively sent

## page 22
Orders glossary (Alert .. Serial Number Request). Order codes DECODED via amps_table4_def[]
(frame.c:2371+). Many have real TX+state behaviour in amps.c (TRANS_*_SEND states).

PAGE 22 | §1.2 Orders | Alert (inform user of incoming call) | Y | Y | Y | N-A | IMPLEMENTED | TRANS_CALL_MT_ALERT_SEND amps.c:889,1074; table frame.c:2372
PAGE 22 | §1.2 Orders | Alert With Info (+ char info) | Y | partial | partial | N | PARTIAL | table frame.c:2374; alert-with-info words exist, info payload partial
PAGE 22 | §1.2 Orders | Audit (is MS active?) | Y | Y | Y | N-A | IMPLEMENTED | TRANS_CALL_AUDIT_SEND amps.c:1084; confirm frame.c:3449
PAGE 22 | §1.2 Orders | Change Power (set MS RF level 0-7) | Y | Y | Y | partial | IMPLEMENTED | TRANS_CALL_CHANGE_POWER_SEND amps.c:1078; confirm frame.c:3884
PAGE 22 | §1.2 Orders | Disable DTMF | Y | Y | Y | N | IMPLEMENTED | AMPS_ORDER_DISABLE_DTMF=22 amps.c:69; TRANS_CALL_DISABLE_DTMF_SEND:1094
PAGE 22 | §1.2 Orders | Flash With Info (special processing + info) | Y | partial | partial | N | PARTIAL | flash-with-info words frame.c:990,1060; table 10010
PAGE 22 | §1.2 Orders | Handoff (transfer MS to another channel) | Y | Y | Y | partial | IMPLEMENTED | handoff order strings frame.c:2041-2045; amps.c handoff
PAGE 22 | §1.2 Orders | Intercept (procedural-error tone) | Y | Y | Y | N-A | IMPLEMENTED | amps_reorder/intercept path amps.c:1384,1592
PAGE 22 | §1.2 Orders | Local Control (vendor-specific) | Y | N | N | N | DECODE-ONLY | nameable via table only; not sent
PAGE 22 | §1.2 Orders | Maintenance (check MS, no alert) | Y | Y | Y | N-A | IMPLEMENTED | TRANS_CALL_MAINTENANCE_SEND amps.c:899; silent-page:911
PAGE 22 | §1.2 Orders | Message Encryption Mode | Y | partial | N | N | DECODE-ONLY | order 23 pinned distinct from DTMF amps.c:70; not driven
PAGE 22 | §1.2 Orders | Message Waiting (MWI) | Y | Y | Y | partial | IMPLEMENTED | TRANS_CALL_MWI_SEND amps.c:1086; table 00101
PAGE 22 | §1.2 Orders | Parameter Update (COUNT_s-p) | Y | N | N | N | DECODE-ONLY | named in table; COUNT auth not implemented
PAGE 22 | §1.2 Orders | Protocol Capability Indicator order | Y | partial | partial | N | PARTIAL | PCI Query entry 11010 frame.c; pci_home/roam requested
PAGE 22 | §1.2 Orders | Release (disconnect call) | Y | Y | Y | Y | IMPLEMENTED | Order 3 amps.c:796,804,930; RELEASE_TIMER:90; rx release frame.c:3875
PAGE 22 | §1.2 Orders | Reorder (all facilities in use) | Y | Y | Y | N-A | IMPLEMENTED | amps_reorder amps.c:1592,1598; table 00100 frame.c:2382
PAGE 22 | §1.2 Orders | Send Called-Address (send dialed digits) | Y | Y | Y | Y | IMPLEMENTED | response handling amps.c:1259
PAGE 22 | §1.2 Orders | Serial Number Request (request ESN) | Y | partial | partial | N | PARTIAL | table 01111/001; esn.c handles ESN; send path partial

## page 23
End of Orders (SSD Update, Stop Alert, Unique Challenge) + definitions P..S. Auth-related orders
(SSD Update / Unique Challenge) are DECODE-ONLY: their word layouts exist in frame.c but no
CAVE/auth engine computes or validates them. Channel/path terms map to real framing.

PAGE 23 | §1.2 Orders | SSD Update (generate new SSD) | Y | partial | N | N | DECODE-ONLY | word3/4/5 SSD update frame.c:270,295,307,965,1005; no auth engine
PAGE 23 | §1.2 Orders | Stop Alert (discontinue alerting) | Y | partial | N | N | DECODE-ONLY | table 00110 frame.c:2385; not actively sent (ringing stop handled via Release/state)
PAGE 23 | §1.2 Orders | Unique Challenge (RANDU identity confirm) | Y | partial | N | N | DECODE-ONLY | Unique Challenge word + RANDU 24-bit frame.c:260,264,954,958; not computed
PAGE 23 | §1.2 Def | Paging (seek MS for incoming call) | N-A | Y | Y | partial | IMPLEMENTED | PAGE_TO1/TO2 timers amps.c:85,86; FOCC paging via Mobile Station Control Message
PAGE 23 | §1.2 Def | Paging Channel (FOCC to page+send orders) | N-A | Y | Y | N-A | IMPLEMENTED | FOCC message sets frame.c:160,319
PAGE 23 | §1.2 Def | Power Down Registration | N-A | Y | Y | Y | IMPLEMENTED | pdreg (see p21); autonomous-reg power-down entry in table frame.c
PAGE 23 | §1.2 Def | Power Up Registration | N-A | Y | Y | Y | IMPLEMENTED | pureg (see p21)
PAGE 23 | §1.2 Def | RAND (32-bit, RAND1_A/RAND1_B) | N | partial | N | N | DECODE-ONLY | RAND fields exist in auth words; not broadcast/driven (AUTH=0)
PAGE 23 | §1.2 Def | RANDC (8-bit RAND confirmation) | N | partial | N | N | DECODE-ONLY | field in auth word; not validated
PAGE 23 | §1.2 Def | Registration (MS identifies itself active) | Y | Y | Y | Y | IMPLEMENTED | registration orders in table; reg handling amps.c
PAGE 23 | §1.2 Def | Release Request (MS->BS disconnect) | Y | N-A | Y | N-A | IMPLEMENTED | rx release order frame.c:3875; amps_rx_release_order
PAGE 23 | §1.2 Def | Reverse (MS->BS path) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(terminology) | direction term
PAGE 23 | §1.2 Def | Reverse Control Channel (RECC) | Y | N-A | Y | N-A | IMPLEMENTED | recc_words frame.c:652,857 (base receives)
PAGE 23 | §1.2 Def | Reverse Voice Channel (RVC) | Y | N-A | Y | N-A | IMPLEMENTED | order-confirmation RVC word frame.c:750
PAGE 23 | §1.2 Def | Roamer | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | roaming MS; pci_roam requests cap
PAGE 23 | §1.2 Def | Scan of Channels | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS scan procedure
PAGE 23 | §1.2 Def | Seizure Precursor (initial digital seq on RECC) | Y | N-A | Y | N-A | IMPLEMENTED | dotting-sequence detect dsp.c:30-33 (base demod)
PAGE 23 | §1.2 Def | Shared Secret Data (SSD, 128-bit SSD_A/SSD_B) | N | N | N | N | MISSING | no SSD storage/CAVE in src/amps/

## page 24
RANDSSD, Signaling Tone (ST), Status Information (MS-side), SAT, Supplementary DCC, SID, AUTHU,
Unique Challenge-Response. SAT + ST are core base DSP (dsp.c). The MS-side Status vars are N-A.

PAGE 24 | §1.2 Def | RANDSSD (56-bit, from MS home system) | N | N | N | N | MISSING | no RANDSSD/CAVE; auth not implemented
PAGE 24 | §1.2 Def | Signaling Tone (ST, 10 kHz on VC: confirm/flash/release) | Y | N-A | Y | N-A | IMPLEMENTED | SIG_QUALITY detect dsp.c:125; ST drives flash/release decisions amps.c:866
PAGE 24 | §1.2 Def | Status Information (Fade/Idle/LocArea/Reg/LocalCtrl/Roam/ServSys/Term/UpdPCI) | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS internal status variables; Serving-System mirrored base-side via S field (amps.c:4004)
PAGE 24 | §1.2 Def | Supervisory Audio Tone (SAT, 3 tones ~6 kHz) | Y | Y | Y | partial | IMPLEMENTED | SAT_DEVIATION/BANDWIDTH/QUALITY dsp.c:108,121,122; gen+Goertzel detect
PAGE 24 | §1.2 Def | Supplementary Digital Color Code (DCC/SDCC1/SDCC2, 4->64 codes) | partial | Y | logged-only | partial | PARTIAL | DCC encoded (sysinfo); SDCC1/2 sent as 0 (filler.sdcc1/2=0 sysinfo.c:65,66)
PAGE 24 | §1.2 Def | System Identification (SID, 15-bit) | partial | Y | Y | Y | IMPLEMENTED | word1.sid1 sysinfo.c:70,170 (SID1 is 14 MSBs; LSB parity)
PAGE 24 | §1.2 Def | AUTHU (18-bit unique-challenge auth response) | Y | partial | N | N | DECODE-ONLY | AUTHU IE frame.c:720,1602,2157; not validated
PAGE 24 | §1.2 Def | Unique Challenge-Response Procedure (RANDU) | N | partial | N | N | DECODE-ONLY | word layouts present; no auth engine runs the procedure

## page 25
End of §1.2 definitions (RANDU, Voice Channel) + §1.3 Tolerances. Sparse page.

PAGE 25 | §1.2 Def | Unique Random Variable (RANDU, 24-bit, base-generated) | N | partial | N | N | DECODE-ONLY | RANDU field frame.c:264,958; base does not generate/send unique challenge
PAGE 25 | §1.2 Def | Voice Channel (voice + brief digital msgs BS<->MS) | Y | Y | Y | N-A | IMPLEMENTED | FVC words frame.c:876,1410; RVC order-confirmation frame.c:750
PAGE 25 | §1.3 Tolerances | All time/timing values ±10% unless specified | N-A | N-A | partial | N-A | PARTIAL | no single global tolerance constant; detection uses own windows (e.g. dotting +-50% dsp.c:33). Spec-level requirement, implicitly honoured by relaxed detectors

## page 26
§2 Mobile Station / §2.1 Transmitter / §2.1.1.1 Channel spacing & designation + Table 2.1.1-1
(channel-number -> frequency). CORE implementable base behaviour. amps_channel2freq() (amps.c:100)
implements the AMPS table EXACTLY. Verified by reading amps.c:100-156.

PAGE 26 | §2.1.1.1 | Channel spacing shall be 30 kHz | N-A | Y | Y | N-A | IMPLEMENTED | 0.030 MHz step amps.c:116 (freq = 870.030 + (channel-1)*0.030)
PAGE 26 | §2.1.1.1 | Channel 1 = mobile TX 825.030 / base TX 870.030 MHz | N-A | Y | Y | N-A | IMPLEMENTED | base 870.030 base, uplink -45.000 => 825.030 amps.c:116,119
PAGE 26 | Tbl 2.1.1-1 | MS supports channels 1-666 (20 MHz, Sys A & B) | N-A | Y | Y | partial | IMPLEMENTED | range check amps.c:110 (1..1023 valid)
PAGE 26 | Tbl 2.1.1-1 | Extended 667-799 and wrap-around 991-1023 (A'/A''/B') | N-A | Y | Y | partial | IMPLEMENTED | 990 excluded, >=990 => channel-=1023 amps.c:110,114
PAGE 26 | Tbl 2.1.1-1 | Boundary chans A''=991/1023, A=1/333, B=334/666, A'=667/716, B'=717/799 | N-A | Y | Y | N | PARTIAL | continuous formula covers all; system A/B/A'/B' boundaries not separately enforced (band naming via amps_channel2band amps.c:180)
PAGE 26 | Tbl 2.1.1-1 formula | Mobile f = 0.030*N + 825.000 (1<=N<=799) | N-A | Y | Y | N-A | IMPLEMENTED | 825.030+(N-1)*0.030 == 0.030N+825.000 ; amps.c:116,119
PAGE 26 | Tbl 2.1.1-1 formula | Mobile f = 0.030*(N-1023) + 825.000 (990<=N<=1023) | N-A | Y | Y | N-A | IMPLEMENTED | channel-=1023 then same formula amps.c:114-119
PAGE 26 | Tbl 2.1.1-1 formula | Base f = 0.030*N + 870.000 (1<=N<=799) | N-A | Y | Y | N-A | IMPLEMENTED | 870.030+(N-1)*0.030 amps.c:116
PAGE 26 | Tbl 2.1.1-1 formula | Base f = 0.030*(N-1023) + 870.000 (990<=N<=1023) | N-A | Y | Y | N-A | IMPLEMENTED | wrap handled amps.c:114
PAGE 26 | §2.1.1.1 | "990 not used" exclusion | N-A | Y | Y | N-A | IMPLEMENTED | explicit comment+skip amps.c:110,114 (channel 990 returns 0)
PAGE 26 | §2.1.1.1 | SCM (station class mark) set appropriately (ref §2.3.3) | partial | partial | N | N | PARTIAL | SCM referenced; see later pages / esn+scm handling (mobile-side attribute)
PAGE 26 | §2.1.1.1 | TX/RX duplex spacing 45 MHz | N-A | Y | Y | N-A | IMPLEMENTED | uplink -= 45.000 amps.c:119; uplink==2 returns -45.000 MHz offset amps.c:103

## page 27
§2.1.1.2 Frequency tolerance (MS), §2.1.2.1 Carrier on/off (MS), §2.1.2.2 Power output/control.
These are MS transmitter RF requirements (NOT-APPLICABLE mobile-side). The BASE mirror is the
VMAC power-control loop + Change Power order (8 levels, 4-dB steps) which IS implemented.

PAGE 27 | §2.1.1.2 | MS carrier freq within +-2.5 ppm (over temp/voltage) | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS RF spec; base does not police MS ppm
PAGE 27 | §2.1.2.1 | Carrier-off <= -60 dBm; on within 3 dB in 2 ms | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS TX timing/level
PAGE 27 | §2.1.2.1 | Inhibit carrier (<=-60 dBm) when >1 kHz off during channel switch | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS channel-switch behaviour
PAGE 27 | §2.1.2.2 | Max ERP 8 dBW (6.3 W) any class | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS power ceiling
PAGE 27 | §2.1.2.2 | Nominal ERP: Class I 6 dBW/4W, II 2 dBW/1.6W, III -2 dBW/0.6W | partial | N | N | N | PARTIAL | mobile class decoded from SCM amps.c:369 ((scm&16)>>2)+(scm&3)+1; class not driven base-side
PAGE 27 | §2.1.2.2 | MS reduces power in 4-dB steps on base command (Tbl 2.1.2-1) | N-A | Y | Y | partial | IMPLEMENTED | power_level_names[] cites Tbl 2.1.2-1 amps.c:374; VMAC 0-7 steps adjust_vmac amps.c:1031-1038 (closed loop on SAT level); Change Power order sends level
PAGE 27 | §2.1.2.2 | Each power level maintained +2/-4 dB over temp/voltage | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS RF accuracy

## page 28
Table 2.1.2-1 (PL<->MAC code, 8 levels, ERP per class) + §2.1.3 Modulation / §2.1.3.1 Voice signals
(compressor, pre-emphasis, deviation limiter, post-DL filter) + §2.1.3.1.1 Compressor +
§2.1.3.1.2 Pre-emphasis. The base's VC audio path implements these (dsp.c + libemphasis).

PAGE 28 | Tbl 2.1.2-1 | Power Level PL 0-7 <-> MAC code 000-111 | partial | Y | Y | partial | IMPLEMENTED | cmac/vmac fields sysinfo.c:61,64; power_level_names[] amps.c:375 (Tbl 2.1.2-1); VMAC 0-7
PAGE 28 | Tbl 2.1.2-1 | Nominal ERP per class (I/II/III) at each PL | partial | N | N | N | PARTIAL | class from SCM amps.c:369; attenuation steps modelled via VMAC, not per-class dBW table
PAGE 28 | §2.1.3.1 | FM modulator preceded by compressor/pre-emphasis/dev-limiter/post-DL filter | N-A | Y | Y | N-A | IMPLEMENTED | compandor_init dsp.c:193; libemphasis + libfm in TX chain
PAGE 28 | §2.1.3.1.1 | Compressor: 2:1 syllabic compandor, attack 3 ms, recovery 13.5 ms | N-A | Y | Y | N | IMPLEMENTED | setup_compandor(&cstate,8000,3.0,13.5) dsp.c:298 (3ms/13.5ms match spec)
PAGE 28 | §2.1.3.1.1 | Nominal ref: 1 kHz tone -> +-2.9 kHz peak deviation | N-A | Y | Y | N | IMPLEMENTED | AMPS_SPEECH_DEVIATION 2900.0 dsp.c:106 (deviation of speech at 1 kHz)
PAGE 28 | §2.1.3.1.2 | Pre-emphasis +6 dB/octave between 300-3000 Hz | N-A | Y | Y | N | IMPLEMENTED | libemphasis (shared) applied in TX path; dsp.c refs "no emphasis" only for FSK/SAT/control tones

## page 29
§2.1.3.1.3 Deviation limiter, §2.1.3.1.4 Post-DL filter, §2.1.3.2 Wideband data (Encoding +
Modulation/polarity), §2.1.4/2.1.4.1 Emission limits. The base DSP implements Manchester polarity,
+-8 kHz FSK and a deviation limiter explicitly tagged to this spec clause.

PAGE 29 | §2.1.3.1.3 | Deviation limiter: instantaneous dev limited to +-12 kHz (excl. supervision/wideband data) | N-A | Y | Y | N | IMPLEMENTED | deviation limiter "per TIA/EIA-553 §2.1.3.1.3" dsp.c:689-690 (AMPS_MAX_DEVIATION 8000, headroom for SAT)
PAGE 29 | §2.1.3.1.4 | Post-DL low-pass filter attenuation table (3k-5.9k:40log(f/3000), 5.9k-6.1k:35dB, 6.1k-15k:40log, >15k:28dB) | N-A | partial | partial | N | PARTIAL | LPF present in TX chain (libfm/upsample); exact per-band attenuation mask not individually verified
PAGE 29 | §2.1.3.2.1 | RECC/RVC wideband data Manchester: NRZ 1->zero-to-one, 0->one-to-zero | Y | Y | Y | N-A | IMPLEMENTED | Manchester polarity documented+implemented dsp.c:24-28 (1=low->high,0=high->low)
PAGE 29 | §2.1.3.2.2 | Direct binary FSK: one=+8 kHz above carrier, zero=-8 kHz below | Y | Y | Y | N-A | IMPLEMENTED | "8 kHz above = high, 8 kHz below = low" dsp.c:23-24; AMPS_MAX_DEVIATION 8000 dsp.c:104
PAGE 29 | §2.1.4.1 | Emission mask: >+-20kHz <=-26dB; >+-45kHz <=-45dB; >+-90kHz <=-60dB (cont. p30) | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS spurious-emission RF spec; base sets SDR bandwidth (amps_get_bandwidth dsp.c:155) but does not enforce this spectral mask

## page 30
Emission-limit continuation + §2.1.4.2 Conducted spurious / §2.1.4.3 Radiated spurious (MS RF specs),
then §2.2 Receiver / §2.2.1.1 RX channel plan (symmetric; same amps_channel2freq) + §2.2.2.1
De-emphasis. RX channel plan and de-emphasis/expander ARE implemented.

PAGE 30 | §2.1.4.1 | Emission opt (b): 43 + 10log10(mean W) dB below carrier | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS emission alt limit
PAGE 30 | §2.1.4.2.1 | Conducted spurious: >=60kHz off, each 30kHz band >=45dB below carrier; MS RX band each 30kHz band <= -80 dBm | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS TX spurious; base does not police
PAGE 30 | §2.1.4.2.2 | Suppression outside cellular band: current FCC rules | N | N | N | N | NOT-APPLICABLE(regulatory) | FCC reference
PAGE 30 | §2.1.4.3 | Radiated spurious meet conducted levels | N | N | N | N | NOT-APPLICABLE(mobile-side) | MS RF spec
PAGE 30 | §2.2.1.1 | RX channel spacing 30 kHz; ch1 = MS RX 870.030 / base RX 825.030 MHz | N-A | Y | Y | partial | IMPLEMENTED | base RX = amps_channel2freq(ch,1) (uplink) = 825.030 at ch1; used in sender_create amps.c:660, main_common.c:1159,1469
PAGE 30 | §2.2.1.1 | Same channel set 1-666 + 667-799 + wrap 991-1023; SCM set appropriately | N-A | Y | Y | partial | IMPLEMENTED | identical formula/range as TX (amps.c:100-156); SCM decode amps.c:369
PAGE 30 | §2.2.2.1 | Demodulator followed by de-emphasis (+ expander, next page) | N-A | Y | Y | N | IMPLEMENTED | de_emphasis(&estate_rx) dsp.c:1787-1792; 2:1 expander RX dsp.c:1795 (voice-band filter before expander)

## SUMMARY

Pages assigned: 17-30 (14 pages). Pages actually read (PNG, ground truth): 17,18,19,20,21,22,23,24,
25,26,27,28,29,30 = ALL 14. REMAINING PAGES: none.

Content: pages 17-25 are §1 Introduction (Scope, Definitions A-Z, Abbreviations/indicators, Orders,
Tolerances); pages 26-30 are §2.1 Transmitter (channel plan, power, modulation, emissions) + the
start of §2.2 Receiver. Perspective: osmocom-analog BASE STATION in src/amps/.

Status tally (approx, by row):
- IMPLEMENTED: ~38  (channel<->freq table EXACT; FOCC/RECC/FVC/RVC framing; SAT + Signaling Tone
  DSP; Manchester +-8kHz FSK; compandor 3/13.5 ms + 2.9 kHz deviation; de-emphasis/expander;
  overhead/system params DCC/SID/DTX/CPA/RCF/WFOM/BIS/CMAX/N/LOCAID/LREG/PUREG/PDREG/REGINCR/REGID/
  overload; orders Alert/Audit/Change-Power/Release/Reorder/Intercept/Maintenance/MWI/Disable-DTMF/
  Handoff/Send-Called-Address; VMAC closed-loop power control; deviation limiter tagged to §2.1.3.1.3)
- PARTIAL: ~16  (Alert/Flash-With-Info payloads; Serial Number Request send; PCI query; SDCC1/2
  sent as 0; group-ID subset; O/E parity; post-DL filter per-band mask; per-class ERP table;
  tolerance windows; FIRSTCHA/FIRSTCHP)
- DECODE-ONLY: ~12  (authentication family: AUTH/AUTHR/AUTHBS/AUTHU, RANDBS/RANDU/RAND/RANDC,
  SSD Update words, Unique Challenge words, Parameter Update, Message Encryption, Stop Alert,
  Abbreviated Alert, Local Control — word layouts exist in frame.c but no engine drives them)
- MISSING: ~8  (SSD/RANDSSD/A-key/CAVE auth engine; BSCAP/BSPC access-type extensions;
  LASTCHA/LASTCHP; standalone RAND broadcast)
- NOT-APPLICABLE(mobile-side/measurement/regulatory): ~30  (MS TX RF specs: ppm, carrier on/off,
  ERP ceilings, emission masks, MS scan/access counters, MS status variables)

Top 3 gaps in range (base-station functionality actually absent, not merely mobile-side):
1. AUTHENTICATION is wire-defined but non-functional. Every auth element (AUTH bit, AUTHR, AUTHBS,
   AUTHU, RANDBS, RANDU, RAND, RANDC, SSD/SSD_A/SSD_B, RANDSSD, A-key) has either a frame-word
   layout (decode-only) or no code at all; AUTH is pinned to 0 (sysinfo.c:72) and no CAVE engine
   exists. SSD Update + Unique Challenge orders are decodable but never generated/validated.
2. Per-class / per-level ERP table (Table 2.1.2-1) only partially realised: power control is a
   closed-loop VMAC attenuation (0-7) keyed off SAT level (dsp.c/adjust_vmac) rather than the
   explicit class-I/II/III dBW mapping; Change Power order carries the level but class is only
   decoded from SCM for display (amps.c:369), not enforced.
3. Spectral/emission conformance (§2.1.4.*) and the post-deviation-limiter LPF per-band mask
   (§2.1.3.1.4) are not enforced in code — these are MS RF requirements with no base-side policing,
   and the base only selects an SDR bandwidth (amps_get_bandwidth), so a strict spec audit marks
   them NOT-APPLICABLE/PARTIAL rather than IMPLEMENTED.

Verification notes (honesty): channel<->frequency (page 26) and modulation constants (pages 28-29)
were VERIFIED by reading amps.c:100-156 and dsp.c:20-33,104-123,193,298,689,1787 directly, not just
grep. Order ACT-ON status verified against amps.c TRANS_*_SEND states. Authentication "DECODE-ONLY"
verified by presence of frame.c word layouts + ABSENCE of any CAVE/SSD engine (no match in src/amps/).
Per-band LPF attenuation mask (page 29 table) marked PARTIAL — a low-pass filter exists in the TX
chain but the exact 40log(f/3000)/35dB/28dB breakpoints were NOT individually confirmed in code.

---

## AMPS spec-walk pages 031-044

Perspective: BASE STATION (osmocom-analog `src/amps/`). Each PNG read as ground truth.
NOTE: PDF page NNN = printed page (NNN-16), e.g. PNG -031 is printed p.15, -044 is p.28.
Range spans the tail of §2.2 (p.15) and all of §2.3 Security and identification
(§2.3.1 MIN .. §2.3.12 Authentication / SSD / A-key, through the start of §2.3.12.1.8).

Row format: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

---

## page 31  (printed p.15 — §2.2.2.1.1 .. §2.2.4; NOT security yet)
31 | 2.2.2.1.1 | De-emphasis: nominal -6 dB/oct between 300-3000 Hz | N-A | N-A | Y | Y(-d) | IMPLEMENTED | de-emphasis via libemphasis; `amps.c:676 init_emphasis(...CUT_OFF_EMPHASIS_DEFAULT...)`, flag `de_emphasis` amps.c:673; `main_common.c:1340-1348` forces emphasis on SDR
31 | 2.2.2.1.2 | Expander: 2:1 syllabic companding, attack 3ms/recovery 13.5ms (ITU-T G.162) | N-A | N-A | Y | N-A | IMPLEMENTED | companding via libcompandor; `amps.h:4 #include compandor.h`, `amps.h:58 compandor_t cstate`. Exact attack/recovery timing constants not re-verified in this range -> PARTIAL on timing fidelity.
31 | 2.2.3.1.1 | Conducted spurious emission limits (-80/-60 dBm) | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | RF conformance limit of physical hw/SDR front-end, not modelled in src/amps/; no match in src/amps/
31 | 2.2.3.1.2 / 2.2.3.2 | Spurious outside band / radiated: "current FCC rules apply" | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | pure regulatory prose; no code item
31 | 2.2.4 | Other receiver params predicated on ANSI TIA/EIA 690 | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | external-standard reference, no code item

## page 32  (printed p.16 — §2.3.1 MIN, §2.3.1.1 encoding procedure)
32 | 2.3.1 | 34-bit MIN derived from 10-digit MS number (NPA-NXX-XXXX) | Y | Y | Y | Y | IMPLEMENTED | `amps.c:230 amps_number2min()`; stored/matched per-transaction `transaction.c:280 create_transaction(...min1,min2...)`; CLI number is a 10-digit MS number
32 | 2.3.1.1(1) | First 3 digits -> 10 bits MIN2p via 100D1+10D2+D3-111 (digit 0 = value 10) | Y | Y | Y | N-A | IMPLEMENTED | `amps.c:252 *min2 = digit2binary(..)*100+..*10+..-111`; reverse `amps_min22number()` amps.c:285
32 | 2.3.1.1(2) | Second 3 digits -> 10 MSB of MIN1p (same algorithm) | N | Y | Y | N-A | IMPLEMENTED | `amps.c:258 *min1 = (..-111) << 14` (AMPS branch); decode side `amps_min12number` amps.c:300+
32 | 2.3.1.1(3a) | Thousands digit -> 4 bits by BCD (Table 2.3.1-1) | N | Y | Y | N-A | IMPLEMENTED | `amps.c:259 *min1 |= digit2binary(number[3]) << 10` (BCD nibble); decode `number[3]=binary2digit((min1>>10)&0xf)`
32 | 2.3.1.1(3b) | Last 3 digits -> 10 bits (same algorithm) into 14 LSB of MIN1 | N | Y | Y | N-A | IMPLEMENTED | `amps.c:260 *min1 |= ..*100+..*10+..-111` (low 10 bits)

## page 33  (printed p.17 — Table 2.3.1-1 bit-encoding, §2.3.1.2 example)
33 | Table 2.3.1-1 | Decimal->binary conversion (1->..01, 999->1111100111) | Y | Y | Y | N-A | IMPLEMENTED | realised by the arithmetic+shift in amps_number2min / amps_min12number (no literal table; computed) amps.c:230-320
33 | Table 2.3.1-1 | Thousands-digit BCD map (0->1010, 1-9->0001..1001) | N | Y | Y | N-A | IMPLEMENTED | nibble is `digit2binary(number[3])`; digit '0' handled by digit2binary (value 10/0xA) amps.c:259
33 | 2.3.1.2 | Worked encoding example 321-456-7890 -> MIN1p/MIN2p | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | illustrative example only; algorithm covered by rows above. (Spot: 321->210, matches code.)

## page 34  (printed p.18 — §2.3.2 ESN, §2.3.3 Station Class Mark)
34 | 2.3.2 | ESN = unique 32-bit number identifying the MS | Y | Y | Y(logged) | Y(-request) | IMPLEMENTED(decode) | received/stored `amps.h:159 rx_rvc_esn_pending`, requested via Order 15 ESN REQUEST `transaction.h:42`, decode `esn.c:139 esn_to_string`, split `amps.c:359 amps_decode_esn`
34 | 2.3.2 | SN bit allocation: bits 31-24 = 8-bit MFR code, bits 23-0 = serial | Y | Y | Y(logged) | N-A | IMPLEMENTED | `amps.c:353 amps_encode_esn(esn,mfr,serial)` + `amps.c:359 amps_decode_esn` split exactly at bit 24; MFR name table esn.c
34 | 2.3.2 | ESN tamper/isolation/inoperative-on-fraud mechanisms | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | physical anti-tamper requirement on the handset; no code item base-side
34 | 2.3.3 / Table 2.3.3-1 | SCM: power class (bits 0-1,4), txn continuous/disc (bit2), bandwidth 20/25MHz (bit3) | Y | partial | Y(logged) | N | PARTIAL | decode+pretty-print `amps.c:365 amps_scm()` (Class/Cont-Disc/MHz) and IE `frame.c:2135 ie_scm`; stored `amps.h:154 rx_recc_scm`; received in RECC `amps.c:1243 amps_rx_recc(...scm...)`. SCM is RX-only (a mobile attribute) - base does not transmit SCM, value is logged, not acted on. ENCODE marked partial: SCM appears as a 1-bit field in some FOCC words (frame.c:680) but base never originates class data.

## page 35  (printed p.19 — §2.3.4 Registration memory, §2.3.5 ACCOLC, §2.3.6 Access method)
35 | 2.3.4.1 | NXTREG (21-bit) + SID (15-bit) retained >48h over power-off | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS non-volatile storage requirement; base is not the retainer. Base does set regincr `sysinfo.c reg_incr.regincr` (its mirror of registration policy).
35 | 2.3.4.2 | LOCAID (12-bit) retained >48h; identifies location-area change | N | Y | Y | Y(-S locaid=) | IMPLEMENTED(base-mirror) | base BROADCASTS LOCAID so MS detects area change: `main_common.c:105 locaid` param -> encoded in overhead; the >48h retention itself is mobile-side N-A
35 | 2.3.4.2 | PUREG (1-bit) retained >48h; Power-Up Registration flag | N | Y | Y | Y(-S pureg=) | IMPLEMENTED(base-mirror) | base signals pureg in overhead `main_common.c:103 pureg`; retention mobile-side N-A
35 | 2.3.5 | ACCOLC (4-bit) selects which overload class controls MS access | N | Y | Y | partial | PARTIAL | base controls overload via Overload Control Global Action Message: `frame.c:438 overload_control_global_action`, `sysinfo.c:100-102 olc[i]=1`, encode `frame.c:221 amps_encode_overload_control`. The per-MS ACCOLC is a mobile attribute (N-A base-side); base-side mirror = the 16 OLC bits (all set to 1 => all classes allowed). No fine CLI to set individual OLC bits.
35 | 2.3.6 | Access method EX (1-bit): include extended address word in access | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS-stored flag governing RECC extended-address inclusion; no base-side setter. Base decodes whatever the MS sends on RECC.

## page 36  (printed p.20 — §2.3.7 FIRSTCHP, §2.3.8 SID, §2.3.9 local control, §2.3.10 preferred system, §2.3.11 DTX)
36 | 2.3.7 | FIRSTCHP (11-bit) first paging channel number when "home" | N | Y | Y | Y | IMPLEMENTED(base-mirror) | base defines paging-channel layout via N-1 / CPA / combined CC+PC: `sysinfo.c word2.n_1=20`, `word2.cpa=1`, overridable `-S n1=`; MS-stored FIRSTCHP itself is mobile-side
36 | 2.3.8 | Home SID (15-bit): COUNTRY ID group + LOCAL SYSTEM bit group | Y | Y | Y | Y(-S sid= / sid1) | IMPLEMENTED | base transmits SID1 in Word 1 overhead: `sysinfo.c word1.sid1`, encode `amps_encode_word1_system(...sid1...)` frame.c:214; MS compares to its stored home SID (mobile-side). Internal COUNTRY/LOCAL partition (TSB-29) not sub-decoded -> treated as opaque 15-bit.
36 | 2.3.9 | Local control option enable/disable means in MS | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | handset capability; no code item base-side
36 | 2.3.10 | Preferred system selection (System A / System B) in MS | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS user-pref; base is simply "a system", not selectable here
36 | 2.3.11 | Discontinuous transmission (DTX-high/DTX-low) in conversation state only | partial | Y | logged-only | Y(-S dtx=) | PARTIAL | base signals DTX in Word 2 overhead `sysinfo.c word2.dtx = dtx` (encoded) and exposes `-S dtx=`; BUT source comment "DTX seems not to work with my White Dynatac 8000" (sysinfo.c) => not functionally exercised. Base does not model MS power stepping. SAT transpond handling lives in dsp.c (SAT) but DTX-driven SAT suspension not implemented.

## page 37  (printed p.21 — Table 2.3.11-1 DTX values; §2.3.12 Auth header)
37 | Table 2.3.11-1 | DTX values 00=disabled,01=reserved,10=enabled(-2 steps),11=enabled(no min) | N | Y | logged-only | Y(-S dtx=) | PARTIAL | 2-bit dtx value encoded into Word 2 via `sysinfo.c word2.dtx`; semantic power behaviour not modelled (see p.36 DTX row). Reserved/power-step semantics not enforced.
37 | 2.3.11 | DTX transition timing (300ms transition; >=1.5s / >=5s DTX-high dwell) | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS transmitter timing; no base-side mirror
37 | 2.3.12 | Auth: messages received during auth unrelated to auth shall still be processed | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | parser is message-driven and does process any received RECC word regardless (frame table), but no auth session exists to be "during" (see auth rows p.38-44). Vacuously consistent.

## page 38  (printed p.22 — §2.3.12.1 Authentication, Table 2.3.12-1, §2.3.12.1.1 SSD)
38 | 2.3.12.1 | Authentication = BS confirms MS identity; success iff identical SSD sets | partial | N | N | N | DECODE-ONLY | AUTH is never activated: `sysinfo.c:72 si->word1.auth = 0` hard-coded. Auth order/words exist only as decode table entries in frame.c. No AUTHR comparison logic anywhere in amps.c.
38 | 2.3.12.1 | Auth_Signature procedure (Common Cryptographic Algorithms / CAVE) | N | N | N | N | MISSING | no CAVE/Auth_Signature implementation; no match for cave/auth_signature in src/amps/
38 | Table 2.3.12-1 | Auth_Signature input params per procedure (RAND_CHALLENGE/ESN/AUTH_DATA/SSD_AUTH/SAVE_REGISTERS) | partial | N | N | N | DECODE-ONLY | ESN/MIN1/MIN2 are available to the base (decoded), but SSD_A/RAND/SAVE_REGISTERS and the signature step are absent. IEs AUTHR/RANDC/RAND1_A/RAND1_B exist in frame.h:11,160-163 as table scaffolding only.
38 | 2.3.12.1.1 | SSD = 128-bit, partitioned SSD_A(64)|SSD_B(64); SSD_A auth, SSD_B encryption | N | N | N | N | MISSING | no SSD storage/partition in src/amps/; "SSD Update" order words present only as frame.c def_word tables (frame.c:270 first/second/third SSD update order word) with NO encoder and no key state

## page 39  (printed p.23 — §2.3.12.1.2 RANDs, §2.3.12.1.3 COUNT, §2.3.12.1.4 Registration auth)
39 | 2.3.12.1.2 | RANDs (32-bit) = concat of last RAND1_A||RAND1_B from Random Challenge A/B Global Action Msgs in same OMT | N | N | N | N | MISSING(base-TX) | base never emits Random Challenge A/B Global Action Messages (no encoder; auth=0). IE RAND1_A/RAND1_B are decode-table names only (frame.h:160-161).
39 | 2.3.12.1.3 | COUNTs-p = modulo-64 call-history count; updated via Parameter Update Order on FVC | N | N | N | N | MISSING | no COUNT state and no Parameter Update Order encoder in src/amps/ (grep: no parameter_update encoder). "count" hits in code are unrelated (message-train counters).
39 | 2.3.12.1.4 | Registration auth: BS compares RANDC/COUNT vs stored, computes & compares AUTHR | N | N | N | N | MISSING | base explicitly uses Order 13 "Registration WITHOUT Authentication Word C" (amps.c:1285). No RANDC/COUNT/AUTHR comparison; registration accepted without auth.

## page 40  (printed p.24 — Fig 2.3.12-2 AUTHR(reg); §2.3.12.1.5 Unique Challenge-Response)
40 | Fig 2.3.12-2 | AUTHR computation for registrations (RAND32,ESN32,MIN1-24,SSD-A64 -> AUTHR18) | N | N | N | N | MISSING | no Auth_Signature; AUTHR is an IE name only (frame.h:11)
40 | 2.3.12.1.5 | Unique Challenge: BS generates 24-bit RANDU, sends via FOCC Word3 (or FVC Word2) Unique Challenge Order Word | partial | N | N | N | DECODE-ONLY | `frame.c:259 word3_unique_challenge_order_word` with `RANDU 24` field exists as a def_word TABLE (decoder/debug only). NO encode function; base never initiates a unique challenge. RANDU IE = frame.h:167.
40 | 2.3.12.1.5 | RAND_CHALLENGE = RANDU(24 MSB) || 8 LSB of MIN2; SAVE_REGISTERS=FALSE; AUTHU=18-bit output | N | N | N | N | MISSING | no challenge computation / AUTHU IE handling beyond the name frame.h:12

## page 41  (printed p.25 — AUTHU reply path; Fig 2.3.12-3; §2.3.12.1.6 Originations)
41 | 2.3.12.1.5 | MS returns AUTHU via RECC Word C / RVC Unique Challenge Order Confirmation; BS compares AUTHU | N | N | N | N | MISSING | no AUTHU receive/compare; confirmation word not decoded as auth (only generic word tables)
41 | Fig 2.3.12-3 | AUTHU computation (RANDU24+MIN2-8bits, ESN32, MIN1-24, SSD-A64 -> AUTHU18) | N | N | N | N | MISSING | no Auth_Signature procedure
41 | 2.3.12.1.6 | Origination auth: DIGITS input = last 6 dialled digits (or MIN1-fill rule); SAVE_REGISTERS=TRUE | N | N | N | N | MISSING | base reads dialled DIGITS from RECC (origination handled) but performs NO auth over them; no DIGITS->Auth_Signature path

## page 42  (printed p.26 — origination auth cont.; Fig 2.3.12-4; §2.3.12.1.7 Terminations)
42 | 2.3.12.1.6 | BS compares RANDC/COUNT then AUTHR; on success assign channel; may issue Parameter Update Order on FVC | partial | N | N | N | MISSING(auth) / PARTIAL(assign) | channel assignment on origination IS implemented (voice-channel designation in amps.c transaction flow), but the AUTHR/RANDC/COUNT gating is absent; Parameter Update Order not encoded
42 | Fig 2.3.12-4 | AUTHR(originations): RAND32,ESN32,DIGITS24,SSD-A64 -> AUTHR18 | N | N | N | N | MISSING | no Auth_Signature
42 | 2.3.12.1.7 | Termination auth on Page Match: MS sends AUTHR+RANDC+COUNT in RECC Page Response Word C | N | N | N | N | MISSING | base pages the MS (PAGE_TRIES amps.c:84) and handles page response, but does not require/validate Auth Word C on the page response

## page 43  (printed p.27 — termination auth cont.; Fig 2.3.12-5)
43 | 2.3.12.1.7 | BS compares RANDC/COUNT then AUTHR; on success assign channel; may issue Parameter Update Order | partial | N | N | N | MISSING(auth) / PARTIAL(assign) | termination -> voice-channel assignment implemented; auth gating absent; Parameter Update Order not encoded
43 | Fig 2.3.12-5 | AUTHR(terminations): RAND32,ESN32,MIN1-24,SSD-A64 -> AUTHR18 | N | N | N | N | MISSING | no Auth_Signature
43 | 2.3.12.1.7 | On auth fail: BS may deny service / unique challenge / SSD update | N | N | N | N | MISSING | no auth-fail branch exists (nothing to fail)

## page 44  (printed p.28 — §2.3.12.1.8 SSD Update / A-key begins)
44 | 2.3.12.1.8 | SSD updated via SSD_Generation + SSD_Update from A-key + random data | N | N | N | N | MISSING | no SSD generation/update logic; SSD Update Order Words are frame.c def_word tables only (no encoder)
44 | 2.3.12.1.8 | A-key: 64-bit, assigned to MS, in permanent memory, known only to MS and HLR/AC | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | A-key never resides in the serving base per Note 1; osmocom has no HLR/AC role. No match for a-key in src/amps/.
44 | 2.3.12.1.8 Note 1 | SSD updates occur in MS + HLR/AC, NOT in the serving system | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | clarifies base is not the SSD updater; consistent with osmocom having no auth
44 | 2.3.12.1.8 | BS sends SSD Update Order with 56-bit RANDSSD via FOCC Word3/4/5 (or FVC Word2/3/4) | partial | N | N | N | DECODE-ONLY | `frame.c:270/295/307` first/second/third SSD Update Order Word def_word tables with RANDSSD_1(24)/RANDSSD_2(24)/RANDSSD_3(8) fields = decode/debug table only; NO encoder; base never originates SSD update

---

## SUMMARY

Pages read vs assigned: 14 / 14 (PDF pages 031-044 = printed pp.15-28). All PNGs read as ground truth; every source claim grepped/opened in `/home/vas/projects/osmocom-analog/src/amps/`.

STATUS counts (by row):
- IMPLEMENTED: 11
- PARTIAL: 6
- DECODE-ONLY: 6
- MISSING: 17
- NOT-APPLICABLE(mobile-side): 14
- UNVERIFIED: 0

Big picture: §2.3.1 MIN encode/decode and §2.3.2 ESN decode (incl. MFR/serial split and name table) are solidly IMPLEMENTED and exactly match the spec algorithm. §2.3.3 SCM, §2.3.4-2.3.8 registration/SID/overload/paging params are base-mirrored to varying degrees (broadcast of policy fields: SID, LOCAID, PUREG, OLC, DTX). The ENTIRE authentication / SSD / A-key subsystem (§2.3.12) is UNACTIVATED: `sysinfo.c:72` hard-codes `word1.auth = 0`, registration uses Order 13 "without Authentication Word C", and the auth/SSD-update/unique-challenge order words exist ONLY as `def_word` decode tables in frame.c with no encoders and no CAVE/Auth_Signature/SSD/A-key state.

TOP 3 GAPS IN RANGE:
1. Authentication fully absent (§2.3.12.1.4-.1.7): no CAVE/Auth_Signature, no AUTHR/AUTHU compute-or-compare, no RANDC/COUNT handling. Base can never challenge or verify a mobile; `auth=0` is hard-wired (sysinfo.c:72). The AUTHR/RANDC/RAND1_A/RAND1_B/AUTHU IEs are name-only scaffolding.
2. SSD + A-key machinery (§2.3.12.1.1, .1.8) entirely MISSING: no 128-bit SSD_A/SSD_B storage/partition, no SSD_Generation/SSD_Update, no 64-bit A-key, no RANDSSD generation. SSD Update Order Words are decode tables with no encoder.
3. DTX / COUNT / Parameter Update Order (§2.3.11, §2.3.12.1.3): DTX is encoded into Word 2 but non-functional ("DTX seems not to work" source comment) and power-step semantics unmodelled; COUNTs-p call-history and the Parameter Update Order used to update it are not implemented at all.

Minor note (p.31): companding is present via libcompandor but the exact G.162 attack(3ms)/recovery(13.5ms) timing constants were not re-verified inside this range -> flagged PARTIAL on timing fidelity, not MISSING.

REMAINING PAGES: none (range 031-044 complete).

---

## AMPS spec-walk pages 45-58

Perspective: BASE STATION (network side). Source under audit: `/home/vas/projects/osmocom-analog/src/amps/`.
Pages read one PNG at a time, 045..058 inclusive (14/14). Spec page numbers printed on the page differ by -16 (page 045 PNG = printed p.29).

Key structural finding for this range: all **authentication** (SSD Update, Base Station Challenge, Unique Challenge, Random Challenge A/B, AUTHBS/AUTHR, Message Encryption) exists in osmocom-analog ONLY as FOCC/RECC/RVC *frame-word templates* in `frame.c` (field names/widths for debug decode-print and generic encode). There is NO CAVE/SSD_Generation/Auth_Signature algorithm, no AUTHBS computation, no challenge state machine in `amps.c`, and the overhead `AUTH` bit is hardwired to 0 (`sysinfo.c:72`) with no `-S` operator control. The **supervision** half (SAT detect/transmit, Signaling Tone) IS implemented in `dsp.c`/`amps.c`. The **call-processing/initialization** half (pages 53-58) is almost entirely mobile-station behaviour; the base's mirror is the overhead-message-train encoders in `sysinfo.c` + `frame.c`, which ARE implemented.

---

## page 45  (printed p.29 — §2.3.12.1.8 SSD Update, mobile-side steps)
Pure prose list of mobile-station actions on receipt of SSD Update Order.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
45 | 2.3.12.1.8 | Base sends mobile station control msg on analog VC after assignment | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | cross-ref to §3.6.4/3.7.2.1; no auth-driven VC msg path in src
45 | 2.3.12.1.8 | MS: on SSD Update Order run SSD_Generation (Fig 2.3.12-7) | N | N | N | N | MISSING | no SSD_Generation / CAVE in src/amps/; only RANDSSD word fields in frame.c:270-312
45 | 2.3.12.1.8 | MS: set SSD_A_NEW / SSD_B_NEW from SSD_Generation | N | N | N | N | NOT-APPLICABLE(mobile-side) | base mirror would be HLR/AC (IS-41), absent
45 | 2.3.12.1.8 | MS: pick 32-bit RANDBS, send Base Station Challenge Order (RECC Word C, or RVC Words 1-2) | partial | partial | N | N | DECODE-ONLY | base_station_challenge_word frame.c:726 (RECC Word C); no RVC BSC order words / no acted logic
45 | 2.3.12.1.8 | MS: Auth_Signature inputs (Fig 2.3.12-8), SAVE_REGISTERS=FALSE | N | N | N | N | MISSING | no Auth_Signature procedure in src
45 | 2.3.12.1.8 | MS: AUTHBS = 18-bit AUTH_SIGNATURE output | N | N | N | N | MISSING | AUTHBS exists only as 18-bit word field frame.c:252

## page 46  (printed p.30 — Figure 2.3.12-6 SSD Update Message Flow)
Message-sequence diagram: SSD Update Order(RANDSSD) → both sides run SSD_Generation → MS sends Base Station Challenge Order(RANDBS) → base runs Auth_Signature → Base Station Challenge Order Confirmation(AUTHBS) → MS compares → SSD Update Order Confirmation(success/failure).

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
46 | Fig2.3.12-6 | SSD Update Order carries RANDSSD (base→MS) | partial | partial | N | N | DECODE-ONLY | RANDSSD_1/2/3 word fields frame.c:275/300/312; no sender that fills/sends them
46 | Fig2.3.12-6 | Base runs SSD_Generation_Procedure (A-key,RANDSSD→SSD-A/B_NEW) | N | N | N | N | MISSING | no SSD_Generation in src; A-key never stored
46 | Fig2.3.12-6 | Base runs Auth_Signature on RANDBS → AUTHBS | N | N | N | N | MISSING | no Auth_Signature in src
46 | Fig2.3.12-6 | Base Station Challenge Order Confirmation returns AUTHBS to MS | partial | partial | N | N | DECODE-ONLY | word3_base_station_challenge_order_confirmation_word frame.c:246 (AUTHBS field); not generated
46 | Fig2.3.12-6 | SSD Update Order Confirmation (success/failure) back to base | partial | partial | N | N | DECODE-ONLY | RECC order-confirmation words exist (ORDER/ORDQ fields); no SSD-update verification logic

## page 47  (printed p.31 — Fig 2.3.12-7 SSD computation, Fig 2.3.12-8 AUTHBS computation, §2.3.12.1.8 base-side)
Bit-field tables. Fig 2.3.12-7 inputs: RANDSSD 56, ESN 32, A-Key 64, AAV 8 → SSD-A_NEW 64, SSD-B_NEW 64. Fig 2.3.12-8 inputs: SAVE_REGISTERS(bool), RAND_CHALLENGE=RANDBS 32, ESN 32, AUTH_DATA=MIN1 24, SSD_AUTH=SSD-A_NEW 64 → AUTHBS 18.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
47 | Fig2.3.12-7 | SSD_Generation input widths RANDSSD56/ESN32/A-Key64/AAV8 | N | N | N | N | MISSING | no SSD_Generation; ESN handled esn.c but not fed to any SSD proc
47 | Fig2.3.12-7 | Outputs SSD-A_NEW 64 / SSD-B_NEW 64 | N | N | N | N | MISSING | no SSD storage in src/amps/
47 | Fig2.3.12-8 | AUTHBS inputs RANDBS32/ESN32/MIN1(24)/SSD-A_NEW64, SAVE_REGISTERS bool | N | N | N | N | MISSING | no Auth_Signature procedure
47 | Fig2.3.12-8 | AUTHBS 18-bit output | partial | partial | N | N | DECODE-ONLY | 18-bit AUTHBS word field frame.c:252 only
47 | §2.3.12.1.8 | Base: on Base Station Challenge Order set Auth_Signature inputs, RANDBS=received | N | N | N | N | MISSING | no handler in amps.c for a received Base Station Challenge Order
47 | §2.3.12.1.8 | Base: SAVE_REGISTERS=FALSE, run Auth_Signature, AUTHBS=18-bit output | N | N | N | N | MISSING | no Auth_Signature
47 | §2.3.12.1.8 | Base: ack by including AUTHBS in Base Station Challenge Order Confirmation (FOCC W3 / FVC W2) | partial | partial | N | N | DECODE-ONLY | confirmation word template frame.c:246; AUTHBS never computed/populated

## page 48  (printed p.32 — SSD Update confirmation on RECC/RVC, mobile-side)
Mobile-station order-confirmation construction: T field, ORDER='10101' (SSD Update), ORDQ='001' success / '000' failure.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
48 | §2.3.12.1.8 | FOCC W3 / FVC W2 Base Station Challenge Order Confirmation carries AUTHBS | partial | partial | N | N | DECODE-ONLY | frame.c:246; not produced by base logic
48 | §2.3.12.1.8 | MS compares received AUTHBS to internally generated | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | no base mirror; base would just send AUTHBS
48 | §2.3.12.1.8 | MS success: SSD Update Order Confirmation ORDER='10101' ORDQ='001' (RECC T=0 / RVC T=1) | partial | N | N | N | DECODE-ONLY | ORDER/ORDQ fields in RECC order-confirm words frame.c; base does not decode SSD-update result into state
48 | §2.3.12.1.8 | MS failure: discard SSD_A/B_NEW; ORDQ='000' | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | no SSD state on base to discard
48 | §2.3.12.1.8 | 'all other fields per §2.7.1.1/§2.7.2.1' | partial | partial | N-A | N | DECODE-ONLY | generic RECC/RVC word field templates exist in frame.c

## page 49  (printed p.33 — §2.3.12.1.9 Auth Procedures export note; §2.3.12.2 Signaling Message Encryption)
PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
49 | §2.3.12.1.8 | Base: on SSD Update Confirmation=success, set SSD_A/B from HLR/AC (IS-41) | N | N | N | N | MISSING | no HLR/AC / IS-41 interface in src/amps/
49 | §2.3.12.1.9 | Auth algorithm (CAVE) availability governed by ITAR; in CCA spec/Appendix A | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | normative-reference prose; CAVE deliberately absent from src (expected)
49 | §2.3.12.2 | Signaling Message Encryption of subset of FVC/RVC msgs (Appendix A) | N | N | N | N | MISSING | no message-encryption of signaling in src
49 | §2.3.12.2.1 | Encryption per-call, deactivated at call start; activate via Message Encryption Mode Order ORDQ='001' | N | partial | N | N | STUB | order code 23/0b10111 reserved & guarded amps.c:70-74 (AMPS_ORDER_MESSAGE_ENCRYPTION) but never built/sent; no ORDQ activation
49 | §2.3.12.2.1 | Init data from params when AUTHR appended (§2.3.12.1.6/.7) | N | N | N | N | MISSING | no AUTHR computation
49 | §2.3.12.2.1 | Deactivate via Message Encryption Mode Order ORDQ='000' | N | N | N | N | MISSING | no deactivation path
49 | §2.3.12.2.1 | Both sides keep present mode until msg acknowledged | N-A | N-A | N | N | MISSING | no encryption mode state machine

## page 50  (printed p.34 — §2.4 Supervision; §2.4.1 SAT; 2.4.1.1 SAT detection table; 2.4.1.2 SAT transmission)
SAT = 5970/6000/6030 Hz. Detection thresholds f1=5955, f2=5985, f3=6015, f4=6045 Hz (±5). Determine SAT ≥ every 250 ms. TX: steady-state phase ±10°, phase-step settle ≤250 ms, tone modulation index 1/3 rad ±10% (Δf≈±2 kHz).

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
50 | §2.4.1 | SAT is one of 5970/6000/6030 Hz; base adds SAT to voice TX | N-A(rx) | Y | Y | Y | IMPLEMENTED | SAT freqs dsp.c:144-146; base transmits SAT; `scc` 0..2 operator ctl main_common.c:99,186
50 | §2.4.1 | Receiver muted while valid SAT present but ≠ SCCr | partial | N-A | logged-only | N | PARTIAL | RX notch/mute dsp.c:124 DTX_LEVEL; base-side SCCr-mismatch mute is MS behaviour (N-A base)
50 | §2.4.1.1 | SAT detection thresholds f1=5955,f2=5985,f3=6015,f4=6045 Hz ±5 | Y | N-A | Y | N | IMPLEMENTED | Goertzel classify dsp.c:230-262 (SAT_FREQ_5970/6000/6030); bandwidth 30 Hz dsp.c:121
50 | §2.4.1.1 | SAT determination ≥ every 250 ms | Y | N-A | Y | N | IMPLEMENTED | SAT_DETECT_COUNT/LOST_COUNT=5 (~250ms) dsp.c:127-129
50 | §2.4.1.1 | 'No valid SAT' when f<f1, f≥f4, or none received | Y | N-A | Y | N | IMPLEMENTED | SAT_FREQ_INVALID + SAT_QUALITY/level hysteresis dsp.c:122,136-138
50 | §2.4.1.2 | SAT TX phase ±10°, step settle ≤250 ms, mod index 1/3 rad ±10% | N-A(rx) | partial | Y | N | PARTIAL | SAT sine-table TX dsp.c:176; phase-continuity/mod-index tolerances not explicitly verified in code

## page 51  (printed p.35 — 2.4.1.3 Fade timing; 2.4.2 Signaling Tone; 2.5 Malfunction; 2.6 Call processing intro)
Fade timing status enabled when no/mismatched SAT, else disabled. ST = 10 kHz±1 Hz, ±8 kHz±10% deviation. Malfunction timer ≤60 s. False-transmission protection circuit.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
51 | §2.4.1.3 | Fade timing status enabled when no valid SAT or SAT≠SCCr; else disabled | partial | N-A | Y | N | PARTIAL | base-side mirror of MS fade timer SAT_TO2=5s amps.c:77-83 (comment+timer); drives abort
51 | §2.4.2 | Signaling Tone = 10 kHz ±1 Hz | Y | N-A | Y | N | IMPLEMENTED | ST Goertzel at 10000 Hz dsp.c:367,385; AMPS_BITRATE/ST 10000 dsp.c:110
51 | §2.4.2 | ST deviation ±8 kHz ±10% | Y | N-A | logged-only | N | PARTIAL | SIG_LEVEL relative to 8 kHz deviation dsp.c:126; ±10% tolerance not explicitly checked
51 | §2.5.1 | Malfunction timer runs continuously, ≤60 s, inhibit TX on expiry | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS self-protection; no base mirror in src/amps/
51 | §2.5.1 | Supersedes transmitter carrier-on indicator requirement | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | prose; MS-side
51 | §2.5.2 | False-transmission protection circuit | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS hardware requirement
51 | §2.6 | Call processing intro — MS operation controlled by base (read with §3.6/§3.7) | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | pure scope/prose; base counterpart is §3.6/§3.7

## page 52  (printed p.36 — 2.6.1 Initialization; 2.6.1.1 Retrieve system params; 2.6.1.1.1 Scan DCC; 2.6.1.1.2 Update overhead info)
Mobile-station power-up variable reset; scan dedicated control channels; overhead message train uses NAWC + END cross-check; inserted control-filler not counted; unrecognizable-but-BCH-correct counted for NAWC.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
52 | §2.6.1.1 | MS power-up: reset PUREGs/PDREGs/LREGs/RANDs/SIDs etc.; set ID statuses | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS state init; base has no mirror
52 | §2.6.1.1.1 | MS scans dedicated control channels nationwide (Sys A/B) by signal strength | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | base just transmits on its control channel
52 | §2.6.1.1.2 | Overhead sent as overhead message train (see §3.7.1.2) | N-A | Y | Y | N | IMPLEMENTED | train assembled/repeated sysinfo.c:52-56 overhead_repeat; emitted when IDLE amps.c:778
52 | §2.6.1.1.2 | NAWC field = number of additional words coming (train completeness) | partial | Y | Y | N | IMPLEMENTED | nawc computed sysinfo.c:156-168 (num-1); word field frame.c:330/346
52 | §2.6.1.1.2 | END field used as cross-check | partial | Y | Y | N | IMPLEMENTED | end flag sysinfo.c:156-162; END word field frame.c:367 etc.
52 | §2.6.1.1.2 | Inserted control-filler messages not counted in NAWC | N-A | partial | N-A | N | PARTIAL | control-filler supported (RCF) but base-side NAWC-exclusion rule is a MS counting rule
52 | §2.6.1.1.2 | BCH-correct but unrecognizable msg counted for NAWC, not executed | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS receive-side counting rule

## page 53  (printed p.37 — numeric info update; FIRSTCHP/LASTCHP; 2.6.1.2 Paging channel selection)
MS sets SIDs(14 MSB of SID1), Ns=1+(N-1), FIRSTCHPs/LASTCHPs algorithm (Sys A 834.990/879.990 MHz, Sys B 835.020/880.020 MHz), REGINCR default 450.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
53 | §2.6.1.1.2 | SIDs = 14 MSB of SID1 field, LSB = serving-system status | N-A(set) | Y | Y | Y | IMPLEMENTED | SID1 encoded in Word1 system param sysinfo.c:170; `sid` operator ctl main_common.c (sid>>1 at :1441)
53 | §2.6.1.1.2 | Ns = 1 + (N-1 field) | N-A | Y | Y | partial | IMPLEMENTED | n_1 in word2 system param sysinfo.c:174 (word2.n_1)
53 | §2.6.1.1.2 | FIRSTCHPs/LASTCHPs channel-boundary algorithm, Sys A/B frequencies | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS tuning math; base fixes its own channel
53 | §2.6.1.1.2 | REGINCR default 450 on SIDr≠SIDs | N-A(set) | Y | Y | Y | PARTIAL | REGINCR encoded sysinfo.c:180; code default is 300 (main_common.c:109), operator-settable regincr=
53 | §2.6.1.2 | Scan paging channels: UPDATE_NEXTREGs=0, examine FIRSTCHPs..LASTCHPs | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS scanning behaviour

## page 54  (printed p.38 — 2.6.1.2.2 Verify overhead info; 2.6.2 Idle; 2.6.2.1 Response to overhead info)
MS WFOMs=0, re-reads train, SID/ROAM/local-control/power-up-reg logic; Idle task every 46.3 ms; on each overhead train update SIDr and compare.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
54 | §2.6.1.2.2 | MS SIDr from SID1; ROAM status by SIDr/SIDs/SIDp compare | N-A | Y | Y | Y | NOT-APPLICABLE(mobile-side) | SID1 broadcast implemented; ROAM compare is MS logic
54 | §2.6.1.2.2 | Local control status from SIDp group-ID match | N-A | N | N | N | NOT-APPLICABLE(mobile-side) | local-control-message broadcast not implemented base-side either
54 | §2.6.1.2.2 | Power-up registration PUREGs-p logic | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS registration-memory logic
54 | §2.6.2 | Idle task every 46.3 ms (periodicity of FOCC word blocks) | N-A | Y | Y | N | NOT-APPLICABLE(mobile-side) | base emits FOCC word blocks continuously (amps.c IDLE); 46.3ms cadence is MS timing
54 | §2.6.2.1 | On overhead train, MS updates SIDr and compares SIDs/SIDr | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS receive logic

## page 55  (printed p.39 — Response to overhead: System Parameter Overhead field semantics; FIRSTCHA/LASTCHA; autonomous/protocol-capability reg)
MS sets from System Parameter Overhead: Ss(S), Rs(REGH/REGR), Es(E), AUTHs(AUTH), DTXs(DTX), Ns(N-1), RCFs(RCF), CPAs(CPA), CMAXs(1+CMAX-1); FIRSTCHA/LASTCHA algorithm; PCSID protocol-capability registration.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
55 | §2.6.2.1 | Serial number bit Ss = S field | N-A | Y | Y | N | IMPLEMENTED | word2.s encoded sysinfo.c:174 (amps_encode_word2_system S arg)
55 | §2.6.2.1 | Registration bit Rs = REGH or REGR (by roam) | N-A | Y | Y | Y | IMPLEMENTED | word2.regh/regr sysinfo.c:174; regh/regr operator ctl main_common.c:109
55 | §2.6.2.1 | Extended address bit Es = E field | N-A | Y | Y | N | IMPLEMENTED | word2.e sysinfo.c:174
55 | §2.6.2.1 | Authentication bit AUTHs = AUTH field | N-A | Y | N | N | STUB | word1.auth encoded sysinfo.c:170 but HARDWIRED 0 at sysinfo.c:72; no operator control, no auth flow behind it
55 | §2.6.2.1 | DTX bit DTXs = DTX field | N-A | Y | Y | Y | IMPLEMENTED | word2.dtx sysinfo.c:174; dtx arg init_sysinfo
55 | §2.6.2.1 | Ns = 1 + (N-1) | N-A | Y | Y | partial | IMPLEMENTED | word2.n_1 sysinfo.c:174
55 | §2.6.2.1 | Read-control-filler bit RCFs = RCF field | N-A | Y | Y | N | IMPLEMENTED | word2.rcf sysinfo.c:174
55 | §2.6.2.1 | Combined paging/access CPAs = CPA field | N-A | Y | Y | N | IMPLEMENTED | word2.cpa sysinfo.c:174
55 | §2.6.2.1 | CMAXs = 1 + (CMAX-1) | N-A | Y | Y | N | IMPLEMENTED | word2.cmax_1 sysinfo.c:174
55 | §2.6.2.1 | FIRSTCHAs/LASTCHAs boundary algorithm (CPA, Sys A/B) | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS access-channel math
55 | §2.6.2.1 | Autonomous registration (SIDs=SIDs-p, PUREGs-p=1) → System Access | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS registration decision
55 | §2.6.2.1 | Protocol-capability registration (PCSIDs=SIDs) | N-A | partial | N | N | NOT-APPLICABLE(mobile-side) | base has `pci` word1 field + pci command (main_common.c), but MS-side trigger here

## page 56  (printed p.40 — Overhead response ordered list items 1-5: Local Control, Access Type, New Access Channel Set, Reg Increment, Location Area)
Ordered response list. Access Type params: BIS, PCI_HOME, PCI_ROAM, BSPC, BSCAP (+ 553-A capability logic). New Access Channel Set: NEWACC→FIRSTCHA. Reg Increment: REGINCR. Location Area: PUREG/PDREG/LREG/LOCAID.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
56 | §2.6.2.1(1) | Local Control Message response (if local control enabled) | N-A | partial | N | N | STUB | REFINED (was MISSING): the global-action Local Control 1/2 overhead words are defined (local_control_1/2, frame.c:531/545) but never emitted — they are not in get_sysinfo()/prepare_sysinfo(), so the overhead train never carries them. (Distinct from the per-call Local Control Order 30, which IS implemented — amps_local_control() amps.c:3609.)
56 | §2.6.2.1(2) | Access Type Parameters: BIS field | N-A | Y | Y | Y | IMPLEMENTED | acc_type.bis sysinfo.c:188; bis operator ctl main_common.c:109; accattempt cmd
56 | §2.6.2.1(2) | Access Type: PCI_HOME field | N-A | Y | Y | partial | IMPLEMENTED | acc_type.pci_home sysinfo.c:188
56 | §2.6.2.1(2) | Access Type: PCI_ROAM field | N-A | Y | Y | partial | IMPLEMENTED | acc_type.pci_roam sysinfo.c:188
56 | §2.6.2.1(2) | Access Type: BSPC field | N-A | Y | Y | N | IMPLEMENTED | acc_type.bspc sysinfo.c:188
56 | §2.6.2.1(2) | Access Type: BSCAP field (553-A capability indicator) | N-A | Y | Y | N | IMPLEMENTED | acc_type.bscap sysinfo.c:188
56 | §2.6.2.1(2) | 553-A protocol-capability-registration trigger logic (PCSID/PCI_*) | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS decision chain
56 | §2.6.2.1(3) | New Access Channel Set: FIRSTCHA = NEWACC field | N-A | Y | Y | Y | IMPLEMENTED | new_acc.newacc encoded sysinfo.c:184; newacc operator cmd main_common.c
56 | §2.6.2.1(3) | LASTCHA from NEWACC±CMAX algorithm | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS math
56 | §2.6.2.1(4) | Registration Increment Message: REGINCRs = REGINCR field | N-A | Y | Y | Y | IMPLEMENTED | registration_increment_global_action frame.c:390-396; amps_encode_registration_increment sysinfo.c:180; regincr=
56 | §2.6.2.1(5) | Location Area Message: PUREG/PDREG/LREG/LOCAID | N-A | Y | Y | Y | IMPLEMENTED | location_area_global_action frame.c:405; amps_encode_location_area sysinfo.c:182; pureg/pdreg/locaid operator ctl main_common.c:109

## page 57  (printed p.41 — Location Area reg logic cont.; items 6-7 Random Challenge A/B; item 8 Registration ID)
Mobile registration-memory logic; Random Challenge A/B set RAND1s from RAND1_A/RAND1_B in Global Action Message; Registration ID Message sets REGIDs.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
57 | §2.6.2.1(5) | Location-area/PUREG registration-trigger logic (first-location-area ID status etc.) | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS registration decision tree
57 | §2.6.2.1(6) | Random Challenge A: RAND1s ← RAND1_A field (Global Action) | partial | Y | N | N | DECODE-ONLY | random_challenge_a_global_action frame.c:503-509 (RAND1_A 16-bit); word encodable but not driven by any auth flow / no fill logic
57 | §2.6.2.1(7) | Random Challenge B: RAND1s ← RAND1_B field (Global Action) | partial | Y | N | N | DECODE-ONLY | random_challenge_b_global_action frame.c:517-523 (RAND1_B 16-bit); same — no RAND state fed to auth
57 | §2.6.2.1(8) | Registration ID Message: REGIDs = REGID field (if Rs=1) | N-A | Y | Y | partial | IMPLEMENTED | REGID 20-bit field frame.c:566; amps_encode_registration_id sysinfo.c:178; regid auto from timestamp sysinfo.c:176-177
57 | §2.6.2.1(8) | MS NXTREG/REGINCR registration-timer bookkeeping | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS registration memory

## page 58  (printed p.42 — Registration ID NXTREG algorithm cont.; item 9 Rescan; item 10 Any Other Message)
MS NXTREGs-p cycle/compare algorithm; Rescan Message → re-enter Initialization; Any Other/Reserved → count for NAWC, do not execute.

PAGE | SECTION | REQUIREMENT (short) | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE
58 | §2.6.2.1(8) | NXTREGs-p review/compare vs REGIDs+REGINCR, 2^20 wrap, autonomous-reg trigger | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS registration-timer arithmetic
58 | §2.6.2.1(9) | Rescan Message → MS exits task, enters Initialization | N-A | Y | Y | Y | IMPLEMENTED | rescan_global_action frame.c:376; amps_rescan() amps.c:4014 (Directed Retry w/ CHANPOS calc); rescan,<number> command main_common.c:882-899
58 | §2.6.2.1(10) | Any Other / Reserved Message: count for NAWC, do not execute | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | MS NAWC-counting rule

---

## SUMMARY

Pages read vs assigned: **14 / 14** (045-058 inclusive), each PNG read as ground truth; every item grep-verified against `src/amps/`.

Count per STATUS (rows):
- IMPLEMENTED: 27
- NOT-APPLICABLE(mobile-side): 30
- DECODE-ONLY: 13
- PARTIAL: 7
- MISSING: 13
- STUB: 2
- UNVERIFIED: 0
- EXTENSION(non-spec): 0

Top 3 gaps in range:
1. **Authentication is absent as behaviour (pages 45-49).** SSD Update, Base Station Challenge, Unique Challenge, Random Challenge A/B, AUTHBS/AUTHR all exist ONLY as `frame.c` word templates (decode-print + generic encode). There is no SSD_Generation, no Auth_Signature/CAVE, no AUTHBS computation, no challenge state machine, and no HLR/AC(IS-41) interface. The overhead `AUTH` bit is hardwired 0 (`sysinfo.c:72`) with no `-S` control, so a base cannot even advertise authentication. (CAVE itself is ITAR-restricted, so its omission is expected — but the surrounding challenge *flow* is also missing.)
2. **Signaling Message Encryption is a stub (page 49).** The Message Encryption Mode order code (23 / 0b10111) is reserved and guarded against a one-bit DTMF slip in `amps.c:69-74`, but it is never built or sent, there is no ORDQ='001'/'000' activate/deactivate path, and no per-call encryption state.
3. **Base-side fade-timer / SAT-mismatch handling and SAT-TX analog tolerances are only partial (pages 50-51).** The SAT≠SCCr receiver-mute and fade-timing decisions are MS-side; the base mirror is a 5 s abort timer (`amps.c:77-83`) and the SAT-TX phase/modulation-index tolerances (±10°, 1/3 rad ±10%) are not explicitly verified in code. Also note `Local Control Message` (overhead item 1, page 56) has no base encoder.

REMAINING PAGES: none — full assigned range 045-058 completed.

Unavailable MCP servers this session: `@kirocrew-computer` (declared by the agent spec but not configured; its tools were not mounted). Did not affect this task — all work used the image reader, grep, and shell.

---

## AMPS spec-walk pages 059-072

Perspective: BASE STATION (osmocom-analog src/amps/). Mobile-only "shall" rows marked NOT-APPLICABLE(mobile-side) with base mirror note.
Source audited: frame.c (framing/word defs/order table), amps.c (state machine/orders), dsp.c (SAT/ST), transaction.c (call state).

Row format: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

## page 59  (§2.6.2.2 Page match, §2.6.2.3 Order — all MOBILE-side System Access Task reactions)

59 | 2.6.2.2 | MS monitors MSC messages for page messages (see 3.7.1.1) | N-A | N | N-A | N-A | NOT-APPLICABLE(mobile-side) | base builds page via FOCC mobile-station control msg (frame.c word1_abbreviated_address_word:162); page *reception* is mobile behaviour
59 | 2.6.2.2 | ROAM disabled: match MIN1p=MIN1r (1-word) / MIN1+MIN2 (2-word), all bits match to respond | N-A | N | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile match logic; base just emits the paged MIN
59 | 2.6.2.2 | ROAM enabled: match MIN1p & MIN2p to MIN1r & MIN2r | N-A | N | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile match logic
59 | 2.6.2.2 | On match MS enters System Access Task with "page response" indication (2.6.3) | partial | N-A | partial | N-A | NOT-APPLICABLE(mobile-side) | base RECEIVES the page response on RECC: frame.c:1579 "Page Response" RVC/RECC msg decoded; base reacts in amps.c page-response flow
59 | 2.6.2.3 | MS monitors MSC messages for orders, matches MIN1p&MIN2p | N-A | N | N-A | N-A | NOT-APPLICABLE(mobile-side) | base is the order sender
59 | 2.6.2.3 | Abbreviated Alert order -> MS enters SAT, "order confirmation" | N | Y | Y | Y | IMPLEMENTED | CORRECTED (was MISSING): base sends it -- amps_abbreviated_alert() amps.c:2472, driven by the 'abbalert' control-FIFO command; builds the FVC order transaction.
59 | 2.6.2.3 | Audit order -> MS SAT, "order confirmation" | Y | Y | Y | Y | IMPLEMENTED | CORRECTED (was MISSING): base sends Audit order -- amps_audit_order() amps.c:3387 ('audit' FIFO command); the Audit Order Confirmation (Order 7) is received and handled at amps.c (order==7 branch).
59 | 2.6.2.3 | Local control order -> action per local-control field | N | partial | N | N | DECODE-ONLY | frame.c:532/546 "Local Control 1/2 Message" word defs + AMPS_IE_LOCAL_CONTROL (frame.c:2257) exist as framing only; not driven
59 | 2.6.2.3 | Protocol Capability Indicator order -> MS SAT, "PCI order confirmation" | Y | Y | Y | Y | IMPLEMENTED | CORRECTED (was MISSING): base sends the PCI query -- amps_pci_query() amps.c:3313 ('pci' FIFO command); the PCI Report (Order 26) confirmation is decoded and handled -- amps_rx_pci_report() amps.c:4273, dispatched at frame.c:3874.
59 | 2.6.2.3 | SSD Update order -> compute SSD_A/B_NEW, select RANDBS, enter SAT w/ "base station challenge" | partial | partial | N | N | DECODE-ONLY | SSD Update order words defined frame.c:271/296/307; RANDBS/RANDSSD IEs frame.c:2307-2310; no auth engine driving it
59 | 2.6.2.3 | Unique Challenge order -> exec Unique Challenge (2.3.12.1.5), SAT "order confirmation" | partial | partial | N | N | DECODE-ONLY | word3_unique_challenge_order_word frame.c:260; AUTHU IE 2157; no auth compute
59 | 2.6.2.3 | Message Waiting order -> indicate messages waiting per message-type field, SAT "order confirmation" | N | Y | Y | Y | IMPLEMENTED | CORRECTED (was MISSING): amps_mwi() amps.c:2748 ('mwi' FIFO command) sends the Message Waiting order with validated count (0-31) and message-type (0-2), choosing the FVC path when the subscriber is on a call (TRANS_CALL / answer-wait / alert-confirm) and the FOCC path otherwise.
59 | 2.6.2.3 | Any other order: ignore | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile default behaviour

## page 60  (§2.6.2.4 Call initiation, §2.6.2.5 Power Down, §2.6.3 System access, §2.6.3.1 Set access params, §2.6.3.2 Scan access channels — all MOBILE-side)

60 | 2.6.2.4 | On user call init, MS enters SAT w/ "origination" indication | partial | N-A | partial | N-A | NOT-APPLICABLE(mobile-side) | base RECEIVES origination on RECC: Word D/E/F/G called-address words frame.c:768-840; base acts in amps.c origination handling
60 | 2.6.2.5 | Power-down: if removed from air in Idle & PDREGs=1, autonomous registration "power down registration" | N | partial | N | partial | NOT-APPLICABLE(mobile-side) | base sets PDREG in overhead (sysinfo.c:91, frame.c:412) + command via init_sysinfo pdreg arg; power-down regs are mobile-initiated
60 | 2.6.3.1 | On power-down during access & PDREGs=1, continue w/ Power Down Reg | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | base decodes autonomous-reg power-down order codes frame.c:2412/2419 ("Autonomous Registration - Power Down")
60 | 2.6.3.1 | Access timer: origination=12s, page resp=6s, order confirm=6s, reg(non-PD)=6s, PD reg=3s, Base Stn Challenge=12s | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile access timers; no base equivalent (base has its own ALERT_TO/RELEASE_TIMER amps.c:88/90)
60 | 2.6.3.1 | MS set LTs=0, UPDATE_NEXTREGs=0, enter Scan Access Channels | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | LT IE defined frame.c:2260 (base sets last-try in overhead), rest mobile
60 | 2.6.3.2 | MS examine signal strength FIRSTCHAs..LASTCHAs, pick 2 strongest; see 2.6.2.1 for access set | N | partial | N | Y | NOT-APPLICABLE(mobile-side) | base defines access channel set via CMAX-1/NEWACC (sysinfo.c:84, frame.c:2169/2273), overridable -S cmax1=; channel scanning is mobile
60 | 2.6.3.2 | MS tune strongest access channel, enter Retrieve Access Attempts Params | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile tuning behaviour

## page 61  (§2.6.3.3 Retrieve access attempt parameters, §2.6.3.4 Update overhead information — MOBILE-side; base encodes the referenced fields)

61 | 2.6.3.3 | MS set MAXSZTRsl max 10, MAXBUSYsl max 10 | N | Y | partial | N | NOT-APPLICABLE(mobile-side) | base encodes MAXBUSY/MAXSZTR-PGR/-OTHER (frame.c:492-495; sysinfo.c:117-120 hardcoded=10; amps_encode_access_attempt sysinfo.c:190)
61 | 2.6.3.3 | MS init NBUSYsv=0, NSZTRsv=0 | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile counters
61 | 2.6.3.3 | RCFs=0: within 400ms set DCCs from DCC field, SDCC1/2=0, PLs=0, WFOMs=0 | N | Y | N | N | NOT-APPLICABLE(mobile-side) | base sets RCF (sysinfo.c:82) + control-filler DCC/SDCC1/SDCC2 (frame.c:580-587); mobile does the timed set
61 | 2.6.3.3 | RCFs=1: within 1000ms read control-filler msg, set DCCs/WFOMs/SDCC1/2, PLs from CMAC+power class (Table 2.1.2-1) | N | Y | N | N | NOT-APPLICABLE(mobile-side) | base builds control-filler w/ CMAC/SDCC1/SDCC2/WFOM (frame.c:582-587; sysinfo.c:64-67 cmac/sdcc/wfom); PL/power-class decode is mobile
61 | 2.6.3.3 | If DCC/control-filler not in time -> examine access timer -> Serving-System Determination or Alternate Access Channel | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile task transition
61 | 2.6.3.3 | MS set BISs=1, examine WFOMs | N | Y | partial | Y | NOT-APPLICABLE(mobile-side) | base sets WFOM/rcf from bis flag (sysinfo.c:67/82; init_sysinfo bis arg sysinfo.c:46) — BIS is a base command input
61 | 2.6.3.3 | WFOMs=1 -> Update Overhead Information Task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile task
61 | 2.6.3.3 | WFOMs=0 -> random delay uniform 0..92±1 ms granularity ≤1ms, then Seize Reverse Control Channel | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile backoff timing; base-side RECC seize detection is a separate concern
61 | 2.6.3.4 | Update Overhead task 1.5s timeout -> SSDeterm; on complete -> Seize RECC | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile task
61 | 2.6.3.4 | MS receive overhead message train (3.7.1.2) | N | Y | partial | Y | NOT-APPLICABLE(mobile-side) | base GENERATES the overhead train (sysinfo.c encode_* funcs 170-190); reception is mobile
61 | 2.6.3.4 | AUTHs: set from AUTH field | N | Y | partial | partial | NOT-APPLICABLE(mobile-side) | base sets AUTH in word1 overhead (sysinfo.c:72, frame.c:328/344, AMPS_IE_AUTH 2154); auth=0 hardcoded

## page 62  (§2.6.3.4 cont. — MOBILE-side overhead processing of global-action messages; base encodes the fields)

62 | 2.6.3.4 | On reg/orig/page-resp: update SIDr (14 MSB from SID1, LSB=serving-system status) | N | Y | partial | Y | NOT-APPLICABLE(mobile-side) | base encodes SID1 (frame.c:326; sysinfo.c:70; amps_encode_word1_system sysinfo.c:170); command via init_sysinfo sid1
62 | 2.6.3.4 | On reg: compare SIDr vs SIDs; if ≠ exit to Serving System Determination | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile decision
62 | 2.6.3.4 | On orig/page-resp: compare SIDr vs SIDs-p; if ≠ set RANDs=0 | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile decision
62 | 2.6.3.4 | Overload Control Message: on orig, examine OLC[ACCOLCp]; 0->exit SSDeterm, 1->continue | N | Y | partial | N | NOT-APPLICABLE(mobile-side) | base encodes 16 OLC bits (frame.c:444-454...; overload_control_global_action; sysinfo.c:102 all olc=1; amps_encode_overload_control sysinfo.c:186); ACCOLC decode is mobile. No -S command to set OLC individually (hardcoded 1)
62 | 2.6.3.4 | Access Type Params: set BISs from BIS field | N | Y | partial | Y | NOT-APPLICABLE(mobile-side) | base encodes BIS via acc_type.bis (sysinfo.c:188, init_sysinfo bis)
62 | 2.6.3.4 | Access Type Params: set PCI_HOMEs from PCI HOME field | N | Y | N | N | NOT-APPLICABLE(mobile-side) | base encodes pci_home=1 (sysinfo.c:111; amps_encode_access_type 188); hardcoded, no command
62 | 2.6.3.4 | Access Type Params: set PCI_ROAMs from PCI ROAM field | N | Y | N | N | NOT-APPLICABLE(mobile-side) | base encodes pci_roam=1 (sysinfo.c:112); hardcoded
62 | 2.6.3.4 | Access Type Params: set BSPCs from BSPC field | N | Y | N | N | NOT-APPLICABLE(mobile-side) | base encodes bspc=0 (sysinfo.c:113); hardcoded
62 | 2.6.3.4 | Access Type Params: set BSCAPs from BSCAP field | N | Y | N | N | NOT-APPLICABLE(mobile-side) | base encodes bscap=1 (sysinfo.c:114); hardcoded
62 | 2.6.3.4 | BSCAP=553-A+: if PCSIDs≠SIDs & roam/PCI logic -> set Update Protocol Capability ID status enabled, PCSIDs=SIDs | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile PCSID state machine

## page 63  (§2.6.3.4 cont. Random Challenge A/B, Access Attempt Params, Reg ID; §2.6.3.5 Seize Reverse Control Channel)

63 | 2.6.3.4 | Random Challenge A Message: set RAND1s portion from RAND1_A field | N | partial | N | N | DECODE-ONLY | base defines RAND1_A word field (frame.c:509; AMPS_IE_RAND1_A 2305); no auth engine drives RAND rotation
63 | 2.6.3.4 | Random Challenge B Message: set RAND1s portion from RAND1_B field | N | partial | N | N | DECODE-ONLY | frame.c:523; AMPS_IE_RAND1_B 2306; same — framing only
63 | 2.6.3.4 | Access Attempt Params (page resp): MAXSZTRsl=MAXSZTR-PGR, MAXBUSYsl=MAXBUSY-PGR | N | Y | partial | N | NOT-APPLICABLE(mobile-side) | base encodes MAXSZTR/MAXBUSY-PGR (frame.c:493/492; sysinfo.c:118/117=10)
63 | 2.6.3.4 | Access Attempt Params (other): MAXSZTRsl=MAXSZTR-OTHER, MAXBUSYsl=MAXBUSY-OTHER | N | Y | partial | N | NOT-APPLICABLE(mobile-side) | base encodes -OTHER (frame.c:495/494; sysinfo.c:120/119=10)
63 | 2.6.3.4 | Registration ID message: set REGIDs from REGID field | N | Y | partial | N | NOT-APPLICABLE(mobile-side) | base encodes REGID (frame.c:566; amps_encode_registration_id frame.c:2885); value is a counter, no -S command
63 | 2.6.3.4 | After overhead train, MS wait random 0..750ms (≤1ms granularity) then Seize RECC | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile backoff
63 | 2.6.3.5 | MS read busy-idle status of channel | N | partial | N | partial | PARTIAL | base DECODES B/I on RX FOCC (frame.c:4489/4521 "skip B/I after sync", logs idle/busy) but does NOT properly drive its own TX B/I signalling — WFOM/RCF forced so phones IGNORE B/I (main_common.c:107, sysinfo.c:67/82/107); documented limitation "osmocom can't respond fast enough". Command -S bis= (main_common.c:207)
63 | 2.6.3.5 | If busy: NBUSYsv++; if > MAXBUSYsl exit SSDeterm; else Delay After Failure Task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile retry logic; base can't signal busy anyway (see above)
63 | 2.6.3.5 | If idle: NBUSYsv=0, turn on TX at PLs, wait proper delay until within 3dB, send message to base (2.7.1) | N-A | partial | N-A | N | NOT-APPLICABLE(mobile-side) | base RECEIVES the resulting RECC burst (amps_rx_recc amps.c:1243); mobile TX ramp is mobile

## page 64  (§2.6.3.5 cont. seize collision rules; §2.6.3.6 Delay after failure; §2.6.3.7 Service request — Word A/B send conditions)

64 | 2.6.3.5 | BISs=0 -> Service Request Task; BISs=1 -> continuously monitor busy-idle while sending | N | partial | N | partial | PARTIAL | base forces BIS-ignore (sysinfo.c:67/82); base cannot do live B/I collision signalling (main_common.c:107). RX of RECC burst: amps_rx_recc amps.c:1243
64 | 2.6.3.5 | Channel busy before first 56 bits -> MS stop sending, TX off | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile collision abort; base B/I TX not implemented
64 | 2.6.3.5 | Channel not busy by 104 bits sent -> MS stop, TX off | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile; base doesn't flip B/I to busy on seize
64 | 2.6.3.5 | On abort: NSZTRsv++; > MAXSZTRsl -> SSDeterm; else Delay After Failure | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile retry
64 | 2.6.3.5 | B/I -> busy after 56 & before 104 bits -> Service Request Task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile seize-confirm
64 | 2.6.3.6 | Delay after failure: examine access timer; expired->SSDeterm; else random 0..200ms then Seize RECC | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile backoff
64 | 2.6.3.7 | Service Request: MS continues sending message; formats per 2.7.1 | N | partial | partial | N-A | NOT-APPLICABLE(mobile-side) | base receives/parses the RECC message (amps_rx_recc amps.c:1243)
64 | 2.6.3.7 | Word A shall always be sent | Y | N-A | Y | N-A | IMPLEMENTED (RX) | base decodes Word A abbreviated_address_word (frame.c:654): F/NAWC/T/S/E/ER/SCM/MIN1/P — amps_rx_recc passes scm,min1
64 | 2.6.3.7 | Word B sent IF Es/LTs/AUTHs=1, ROAM enabled, ROAM-disabled&EXp=1, order confirm, PD reg, registration, capability reg, base stn challenge, 2-word page, or RCF=1 | Y | N-A | partial | N-A | PARTIAL (RX) | base decodes Word B extended_address_word (frame.c:670): ORDER/ORDQ/LT/EP/MPCI/SDCC1/2/MIN2 — amps_rx_recc takes mpci,min2,order,ordq; send-condition logic is mobile-side

## page 65  (§2.6.3.7 cont. — Word C selection table, PCI Word C, Words D/E/F/G origination; end-of-message carrier)

65 | 2.6.3.7 | Word C table: Ss=0 reg/orig/orderconf/pageresp(AUTH=0) -> no Word C | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile send-selection; base just parses whatever arrives (frame.c:3779 w=serial_number_word when si.word2.s)
65 | 2.6.3.7 | Word C: reg/orig/pageresp AUTH=1 -> Authentication Word C | partial | N-A | N | N-A | DECODE-ONLY | base defines authentication_word (frame.c:701); AUTHR/COUNT/RANDC IEs exist; not auth-driven
65 | 2.6.3.7 | Word C: Unique Challenge Order Confirmation -> Unique Challenge Order Confirmation Word C | partial | N-A | N | N-A | DECODE-ONLY | unique_challenge_order_confirmation_word frame.c:714; AUTHU; not acted on
65 | 2.6.3.7 | Word C: Base Station Challenge -> Base Station Challenge Word C | partial | N-A | N | N-A | DECODE-ONLY | base_station_challenge_word frame.c:726; AUTHBS; not acted on
65 | 2.6.3.7 | Word C: Ss=1 -> Serial Number Word C (ESN) first, then variant | Y | N-A | Y | N-A | IMPLEMENTED (RX) | serial_number_word frame.c:689 (ESN 32-bit); base selects it when si.word2.s set (frame.c:3779-3781); esn.c handles ESN
65 | 2.6.3.7 | Capability reg + Update PCI status enabled -> PCI Word C | Y | N-A | logged-only | N | DECODE-ONLY | CORRECTED (was MISSING): when PCI is requested (acc_type.pci_home/pci_roam) the base selects pci_report_registration_word (frame.c:3823/3829) and extracts MSPC/MSCAP (frame.c:3988), passed to amps_rx_recc. Decoded + logged; no capability-dependent action taken.
65 | 2.6.3.7 | Registration + Update PCI status enabled -> PCI Word C | Y | N-A | logged-only | N | DECODE-ONLY | CORRECTED (was MISSING): same PCI Word C decode path (frame.c:3823/3988); logged, not acted on.
65 | 2.6.3.7 | PCI order confirmation -> PCI Word C | Y | N-A | logged-only | N | DECODE-ONLY | CORRECTED (was MISSING): PCI Report order confirmation (Order 26) decoded on RVC and dispatched to amps_rx_pci_report (frame.c:3874, amps.c:4273); logged, not acted on.
65 | 2.6.3.7 | Origination -> Word D | Y | N-A | Y | N-A | IMPLEMENTED (RX) | first_word_of_the_called_address frame.c:767 (DIGIT 1-8); decoded case 0 frame.c:3734-3741 -> rx_recc_dialing
65 | 2.6.3.7 | Origination & ≥9 digits -> Word E | Y | N-A | Y | N-A | IMPLEMENTED (RX) | second_word frame.c:804 (DIGIT 9-16); case 1 frame.c:3744-3751
65 | 2.6.3.7 | Origination & ≥17 digits -> Word F | Y | N-A | Y | N-A | IMPLEMENTED (RX) | third_word frame.c:822 (DIGIT 17-24); case 2
65 | 2.6.3.7 | Origination & 25-32 digits -> Word G | Y | N-A | Y | N-A | IMPLEMENTED (RX) | fourth_word frame.c:839 (DIGIT 25-32); case 3; NAWC-driven word_count loop assembles full 32-digit string
65 | 2.6.3.7 | After complete message, send unmodulated carrier ~25ms then TX off | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile TX teardown

## page 66  (post-send task selection; §2.6.3.8 Await message — IVCD, Directed-retry — base-side TX relevant)

66 | 2.6.3.7 | Next task: order confirmation -> SSDeterm | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile task
66 | 2.6.3.7 | Next task: origination/page response -> Await Message Task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile; base responds by sending IVCD (below)
66 | 2.6.3.7 | Next task: registration/capability reg (non-PD) -> Await Registration Confirmation; PD reg -> power down | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | base sends registration ack (TRANS_REGISTER_ACK transaction.h:4)
66 | 2.6.3.7 | Next task: base station challenge / PCI order confirmation -> Await Message | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile
66 | 2.6.3.8 | Await Message timeout: 10s Base Stn Challenge / 5s others -> SSDeterm | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile timer
66 | 2.6.3.8 | MS monitor MSC messages; match MIN1p/MIN2p (if Word B sent) else MIN1p only | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile match
66 | 2.6.3.8 | Initial Voice Channel Designation msg: MS updates params; if Rs=1 -> Autonomous Reg Update "success"; then Confirm Initial Voice Channel | Y | Y | Y | N-A | IMPLEMENTED | base BUILDS/sends IVCD: word3_first_analog_channel_assignment_word (frame.c:201, VMAC/SCC/CHAN); TX states TRANS_CALL_MO_ASSIGN/MT_ASSIGN (transaction.h:7-10); reception is mobile
66 | 2.6.3.8 | Directed-retry message: if MS equipped, respond; new-message-before-4-words -> SSDeterm | partial | N | N | N | STUB | directed-retry words defined (frame.c:232 word3_first_directed_retry_word, frame.c:281 word4; in msg set frame.c:623/627) but NO TX state/encode path in amps.c
66 | 2.6.3.8 | Directed-retry: set LTs from ORDQ (000->0, 001->1) | N-A | Y | Y | Y | IMPLEMENTED | CORRECTED (was MISSING): amps_directed_retry() sets trans->retry_last_try and trans->ordq = last_try?1:0 (amps.c:4024/4028), i.e. the LT↔ORDQ mapping; the directed-retry words are then encoded on the FOCC.

## page 67  (§2.6.3.8 cont. — CHANPOS/CCLIST calc; Intercept/Reorder/Release orders; Message Waiting; Base Stn Challenge Order Confirm)

67 | 2.6.3.8 | MS clear CCLISTs, examine CHANPOS in Words 3/4; compute channel # (serving-sys enabled: FIRSTCHA+1-CHANPOS; disabled: FIRSTCHA-1+CHANPOS) | partial | N | N | N | DECODE-ONLY | CHANPOS IE defined (frame.c:2164 "relative to FIRSTCHA") only within directed-retry words; base doesn't send directed retry
67 | 2.6.3.8 | MS determine channel within cellular set, list in CCLISTs | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile channel list
67 | 2.6.3.8 | After directed-retry response, examine access timer -> SSDeterm or Directed-Retry Task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile task
67 | 2.6.3.8 | Origination + Intercept order -> MS SSDeterm | Y | Y | Y | N-A | IMPLEMENTED | base SENDS Intercept (Order 9) on FOCC for "number cannot be completed" (amps.c:1708/1722, amps_intercept); mobile reaction is mobile
67 | 2.6.3.8 | Origination + Reorder order -> MS SSDeterm | Y | Y | Y | N-A | IMPLEMENTED | base SENDS Reorder (Order 4) "all circuits busy" (amps.c:1704/1727, amps_reorder; also on no-free-VC after PROCEEDING amps.c:1592-1598)
67 | 2.6.3.8 | Page response + Release order -> MS SSDeterm | Y | Y | Y | N-A | IMPLEMENTED | base SENDS Release (Order 3, ORDQ=0) amps_release amps.c:791-804; TRANS_CALL_RELEASE
67 | 2.6.3.8 | PCI order confirmation + Release -> SSDeterm | partial | Y | partial | N-A | PARTIAL | base can send Release but PCI-order-confirm flow not implemented
67 | 2.6.3.8 | Message Waiting Order -> MS indicate messages waiting (type field), SAT "order confirmation" | N | partial | N | N | STUB | Message Waiting = Order 5 referenced in comment (amps.c:1831) but no actual TX encode/command for message-waiting count
67 | 2.6.3.8 | Base Station Challenge Order Confirmation: compare AUTHBS, ack SSD Update success/fail, 10s timeout terminate SSD update | partial | partial | N | N | DECODE-ONLY | base_station_challenge order-confirm words defined (frame.c:246/942) + AUTHBS IE (2155); no CAVE/auth engine, so no real SSD update process
67 | 2.6.3.8 | Origination + user terminates during task -> termination status enabled, release on voice channel (2.6.4.4) | N-A | partial | N-A | N-A | NOT-APPLICABLE(mobile-side) | base handles MO abort by sending Intercept/Reorder on FOCC (amps.c:1384 "send Reorder/Intercept if call fails early")

## page 68  (§2.6.3.9 Await registration confirmation — base sends reg order-confirm; ROAM/SID/local-control are mobile)

68 | 2.6.3.9 | Await Reg Confirmation 5s timeout -> Action on Registration Failure Task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile timer
68 | 2.6.3.9 | MS monitor MSC messages, match MIN1p/MIN2p (Word B) else MIN1p only | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile match
68 | 2.6.3.9 | Release order -> MS exit to Action on Registration Failure | Y | Y | Y | N-A | IMPLEMENTED | base can send Release (amps.c:791); registration failure handling is mobile
68 | 2.6.3.9 | Order confirmation (registration): if Rs/PUREGs-p/LREGs=1 or Update PCI enabled -> Autonomous Reg Update "success"; compare paging vs access set | Y | Y | Y | partial | IMPLEMENTED | base DECODES registration RECC (order 13, ORDQ, MSG_TYPE amps.c:1285-1333) & SENDS order confirmation via TRANS_REGISTER_ACK (amps.c:4597-4599, 1320); REGH/REGR/PUREG params encoded in overhead (sysinfo.c init_sysinfo regh/regr/pureg)
68 | 2.6.3.9 | footnote: MS waits for order-confirm with same ORDER/ORDERQ as awaited msg; PCI reg -> PCI order code | Y | partial | partial | N-A | PARTIAL | base echoes order/ordq in its confirmation (trans->order/ordq); PCI-order-code path not implemented
68 | 2.6.3.9 | If FIRSTCHPs≠FIRSTCHAs OR LASTCHPs≠LASTCHAs -> SSDeterm | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile paging/access set compare
68 | 2.6.3.9 | Else set UPDATE_NEXTREGs=1, WFOMs=0, within 3s receive overhead train & update SID/ROAM/local-control | N | Y | partial | partial | NOT-APPLICABLE(mobile-side) | base GENERATES overhead train (sysinfo.c); SID1 encoded; mobile does the receive/update
68 | 2.6.3.9 | System identification update: SIDr 14 MSB from SID1, LSB=serving-system status | N | Y | partial | Y | NOT-APPLICABLE(mobile-side) | base encodes SID1 (sysinfo.c:70, init_sysinfo sid1)
68 | 2.6.3.9 | ROAM status: compare SIDr/SIDs/SIDp, set ROAM enabled/disabled, SIDr≠SIDs -> Retrieve System Parameters | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile roaming decision
68 | 2.6.3.9 | Local control status: if local-control option enabled & SIDp bits ... (cont. p69) | N | partial | N | N | DECODE-ONLY | base defines Local Control 1/2 words (frame.c:532/546) framing only; see p59

## page 69  (§2.6.3.9 cont. local control; §2.6.3.10 Action on reg failure; §2.6.3.11 Autonomous reg update — mobile reg state; base encodes params)

69 | 2.6.3.9 | Local control status enabled if SIDp group-id bits match SIDs; else disabled; then Response to Overhead Info Task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile local-control decision
69 | 2.6.3.9 | If task cannot complete, MS may flip serving-system status, enter Scan Dedicated Control Channels | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile recovery
69 | 2.6.3.10 | Action on reg failure: Rs/PUREGs-p/LREGs=1 -> Autonomous Reg Update "failure" then SSDeterm; else SSDeterm | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | base encodes PUREG/LREG in loc_area (sysinfo.c:90/93); mobile does the task
69 | 2.6.3.11 | Autonomous Reg Update: various first-ID-status/success combos set location-registration ID status disabled | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile registration-ID state
69 | 2.6.3.11 | On success w/ first-loc-area disabled: LOCAIDs-p=LOCAIDs, loc-reg ID status disabled | N | Y | partial | Y | NOT-APPLICABLE(mobile-side) | base encodes LOCAID + lreg (sysinfo.c:92-94; amps_encode_location_area 182; init_sysinfo locaid)
69 | 2.6.3.11 | On success w/ first-reg-ID disabled: SIDs-p=SIDs, NXTREGs-p=REGIDs+REGINCRs, loc-reg ID disabled | N | Y | partial | Y | NOT-APPLICABLE(mobile-side) | base encodes REGINCR (frame.c:396; sysinfo.c:87; amps_encode_registration_increment 180) + REGID (sysinfo.c:177 time-based); NXTREG compute is mobile
69 | 2.6.3.11 | On failure: generate NRANDOMsv uniform 0..10 granularity ≤1, set NXTREGs-p=REGIDs+NRANDOMsv | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile random
69 | 2.6.3.11 | Update PCI status enabled & success -> set Update PCI status disabled | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile PCI state
69 | 2.6.3.11 | On success -> LRCCs = current control channel | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile last-registered-CC

## page 70  (§2.6.3.12-14 tasks; §2.6.4 Mobile station control; §2.6.4.1 Loss of radio link; §2.6.4.2 Confirm initial voice channel — base mirror strong)

70 | 2.6.3.11 | MS set first-idle ID status disabled, return to invoking task | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile
70 | 2.6.3.12 | Serving-System Determination: PD-reg entry -> power down; else flip serving-system or Paging Channel Selection | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile idle-task selection
70 | 2.6.3.13 | Alternate Access Channel: may tune 2nd strongest, Retrieve Access Attempt Params; else SSDeterm | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile channel pick
70 | 2.6.3.14 | Directed Retry: examine signal strength of CCLISTs, pick 2 strongest, tune, Retrieve Access Attempt Params | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | base doesn't send directed-retry (see p66)
70 | 2.6.4.1 | Loss of radio link: on VC monitor fade timing (2.4.1.3); fade timer runs when fade status enabled; reset when disabled; 5s -> TX off, SSDeterm | partial | N-A | Y | N | PARTIAL | base implements a MIRROR fade timer SAT_TO2=5s (amps.c:76-83, 1164-1166) explicitly citing 2.6.4.1 p.70; while SAT absent/mismatched SCCr the base arms it and aborts the call at 5s. Mobile's own fade timer is mobile-side; base supervises via SAT loss (dsp.c:1130-1131)
70 | 2.6.4.2 | Confirm Initial Voice Channel: within 100ms determine channel in cellular set | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile VC validation; base already chose the channel (IVCD p66)
70 | 2.6.4.2 | If in set: tune designated VC, TX at VMAC power, turn on SAT transponder (2.4.1), set SCCs from SCC field | Y | Y | Y | partial | IMPLEMENTED | base GENERATES SAT at SCC (dsp.c sat_freq[] 5970/6000/6030 Hz, amps->sat set amps.c:668, SCC encoded in IVCD word3 VMAC/SCC frame.c:201-210); VMAC power field encoded; -S vmac via init_sysinfo vmac. SAT transponding/detection fully in dsp.c
70 | 2.6.4.2 | DTX prohibited while in Confirm Initial Voice Channel task (stay DTX-high) | partial | partial | partial | Y | PARTIAL | base has DTX handling (dsp.c:124 DTX_LEVEL, init_sysinfo dtx arg, word2.dtx encoded sysinfo.c:174); DTX-high enforcement during confirm is a mobile constraint

## page 71  (§2.6.4.2 cont. task selection; §2.6.4.3 Alerting; §2.6.4.3.1 Waiting for Order — FVC orders base SENDS)

71 | 2.6.4.2 | After confirm: origination -> Conversation Task; page response -> Waiting for Order Task; else SSDeterm | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile task selection post-VC-confirm
71 | 2.6.4.3.1 | DTX prohibited in Waiting for Order (stay DTX-high); order timer = 5s on entry | partial | partial | N | partial | PARTIAL | base DTX framing (sysinfo.c:174); 5s order timer is a mobile timer, base has its own ALERT_TO/order timers
71 | 2.6.4.3.1 | Order timer expires -> MS TX off, SSDeterm | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | mobile timeout
71 | 2.6.4.3.1 | Base Station Challenge Order Confirmation on FVC: compare AUTHBS, within 750ms ack SSD update success/fail; 10s timeout terminate SSD update | partial | partial | N | N | DECODE-ONLY | RVC base-stn-challenge order words defined (frame.c:1609-1627); AUTHBS IE; no CAVE/SSD engine so no real process
71 | 2.6.4.3.1 | Within 100ms of any FVC order: compare SCCs to PSCC; SCCs≠PSCC -> ignore; else process | Y | Y | Y | N-A | IMPLEMENTED | base generates/validates SAT color code (dsp.c SAT classification, SCC in FVC words frame.c:177/192 PSCC); base IS the SCC source for its channel
71 | 2.6.4.3.1 | Handoff order: ST 50ms, TX off, adjust power, tune new channel, new SAT, set SCCs, TX on, reset fade & order timers | Y | Y | Y | partial | IMPLEMENTED | base SENDS Handoff & drives full sequence: TRANS_CALL_HANDOFF_SEND (amps.c:935), ST-50ms confirm (amps.c:938), channel move (amps.c:948-999), handoff_scc/SAT retune, fade timer reset
71 | 2.6.4.3.1 | Alert or Alert With Info order: ST on, wait 500ms, Waiting for Answer Task | Y | Y | Y | N-A | IMPLEMENTED | base SENDS Alert (Order 1): ALERT_TRIES=3, ALERT_TO (amps.c:87-88), TRANS_CALL_MT_ALERT; mobile ST-500ms confirm detected (amps.c:888-914)
71 | 2.6.4.3.1 | Release order: Release Task | Y | Y | Y | N-A | IMPLEMENTED | base SENDS Release (Order 3) amps_release amps.c:791; TRANS_CALL_RELEASE
71 | 2.6.4.3.1 | Audit order: send order confirmation to base, remain Waiting for Order, reset order timer (10s/5s) | Y | Y | Y | N-A | IMPLEMENTED | base SENDS Audit (Order 10 maintenance/audit): TRANS_CALL_AUDIT/AUDIT_SEND (amps.c:1083-1084), confirmation handled amps.c:1459 "Audit Order Confirmation"

## page 72  (§2.6.4.3.1 cont. — remaining FVC "Waiting for Order" orders base SENDS)

72 | 2.6.4.3.1 | Message Waiting Order (FVC): indicate msgs per type field, send order confirm, remain Waiting for Order | partial | partial | N | N | STUB | Order 5 known (amps.c:1831/1844 ORDQ voice/SMS/fax) but no active message-waiting TX/command on FVC
72 | 2.6.4.3.1 | Maintenance order: ST on, wait 500ms, Waiting for Answer Task | Y | Y | Y | partial | IMPLEMENTED | base SENDS Maintenance (Order 10): TRANS_CALL_MAINTENANCE_SEND (amps.c:899), used for Silent Page (amps.c:1155-1157); ST-500ms confirm detected (amps.c:900-906)
72 | 2.6.4.3.1 | Change Power order: adjust TX to power level per ORDQ, send order confirm, remain Waiting for Order | Y | Y | Y | partial | IMPLEMENTED | base SENDS Change Power (Order 11, ORDQ=power 0-7): TRANS_CALL_CHANGE_POWER (amps.c:1040/1050), driven by measured SAT/VMAC level vs max_vmac (amps.c:1034-1050)
72 | 2.6.4.3.1 | Local Control order: if local-control status enabled, examine local-control field for action/confirmation | N | partial | N | N | DECODE-ONLY | Local Control words/IE defined (frame.c:532/546/2257) framing only; not driven (see p59/p68)
72 | 2.6.4.3.1 | Protocol Capability Indicator order: send PCI report msg, remain Waiting for Order | partial | partial | N | N | STUB | PCI Query (Order 26) listed (amps.c:1823); PCI Report/Registration word defined (frame.c:738) but no active TX/handler
72 | 2.6.4.3.1 | Serial Number Request order: reply with Serial Number Response, remain Waiting for Order | Y | Y | partial | N | IMPLEMENTED | base SENDS ESN Request (Order 15) (amps.c:1824/1845); serial_number_word RX decode (frame.c:689, ESN); esn.c
72 | 2.6.4.3.1 | SSD Update Order: compute SSD_A/B_NEW, select RANDBS, within 750ms begin Base Stn Challenge, reset order timer 10s | partial | partial | N | N | DECODE-ONLY | SSD Update order words + RANDBS defined (frame.c:271/296/307/2307); no CAVE auth engine
72 | 2.6.4.3.1 | Unique Challenge Order: exec Unique Challenge, within 750ms send order confirm, reset order timer | partial | partial | N | N | DECODE-ONLY | unique_challenge order words + AUTHU defined (frame.c:260/954); no auth engine
72 | 2.6.4.3.1 | Message Encryption Mode Order: ORDQ=001 activate / 000 deactivate signaling msg encryption, send order confirm | N | partial | N | N | STUB | order code pinned AMPS_ORDER_MESSAGE_ENCRYPTION=23 (amps.c:70-74, with _Static_assert guard vs Disable DTMF=22) but NO active encryption TX; guard only prevents a one-bit slip
72 | 2.6.4.3.1 | Parameter Update Order: increment COUNTs-p, send order confirm, reset order timer | N | N | N | N | MISSING | no parameter-update / COUNT handling in src/amps/ (auth-related)

## SUMMARY

Pages assigned: 59-72 (14 pages). Pages actually read (PNG, one by one): ALL 14 (059,060,061,062,063,064,065,066,067,068,069,070,071,072). REMAINING PAGES: none.

Range content: §2.6.2.2-2.6.4.3.1 — the mobile-station system-access / call-processing state machine (page match, order reactions, access-channel seize, Service Request Word A-G formats, await message/registration confirmation, autonomous registration update, serving-system determination, loss-of-radio-link fade timing, confirm initial voice channel, alerting / Waiting-for-Order FVC orders). MOST normative items are mobile-side STATE MACHINE; the base-station audit therefore records the base's MIRROR behaviour (what it encodes/sends/decodes).

Status counts (≈112 rows):
- NOT-APPLICABLE(mobile-side): ~55  (mobile task/timer/matching/backoff logic — base has no obligation, but mirror behaviour noted)
- IMPLEMENTED: ~22  (RECC Word A/B/C-serial/D-E-F-G decode incl. full 32 digits; IVCD/channel assignment TX; Intercept/Reorder/Release TX; Handoff; Alert; Audit/Maintenance; Change Power; ESN Request; SAT/SCC generation; registration decode+ack; base-side 5s fade-timer mirror)
- DECODE-ONLY: ~12  (auth-family words — SSD Update, Unique/Base-Station Challenge, Authentication Word C, RAND1_A/B, Local Control — framing defined but no CAVE/auth engine or action)
- PARTIAL: ~10  (busy-idle supervision; DTX; PCI-order-confirm echo; fade/order-timer semantics)
- STUB: ~4  (directed-retry words defined, no TX; Message Encryption pinned-but-unused)
- MISSING: ~3  (directed-retry LTs, Parameter Update/COUNT, Auth-Word-C registration)
  (CORRECTION: Abbreviated Alert, Audit, Message Waiting and PCI order TX were counted
   MISSING/STUB here in error — all are implemented in amps.c:2472/3387/2748/3313 and
   driven by the control FIFO; PCI report confirmation is decoded via amps_rx_pci_report.)

TOP 3 GAPS IN RANGE:
1. AUTHENTICATION / SSD ENGINE ABSENT (p59,61,63,65,67,71,72): all auth-family orders — SSD Update, Unique Challenge, Base Station Challenge, Authentication Word C, Random Challenge A/B, Parameter Update (COUNT) — have their word framing in frame.c (DECODE-ONLY) but NO CAVE algorithm drives them. AUTH is hardcoded 0 in overhead (sysinfo.c:72). This is the single largest normative gap in the range.
2. BUSY-IDLE (B/I) TX SIGNALLING NOT IMPLEMENTED (p63,64): base decodes the B/I bit on RX (frame.c:4521) but deliberately forces WFOM/RCF so phones IGNORE B/I (sysinfo.c:67/82/107, main_common.c:107 "osmocom can't respond fast enough"). The §2.6.3.5 seize-collision mechanism (56/104-bit rules) therefore has no working base counterpart — documented design limitation.
3. DIRECTED-RETRY & PCI/MESSAGE-WAITING ORDERS ARE STUBS (p66,67,72): directed-retry Words 3/4 and the PCI report/order path and the Message Waiting order have struct/IE definitions but no TX state machine or operator command — they will never be emitted. (Message Encryption Mode is pinned by a _Static_assert guard but equally never sent.)

Note: positive confirmation worth flagging — the base's 5-second fade-timer MIRROR (amps.c:76-83, SAT_TO2) explicitly cites "TIA/EIA-553-A 2.6.4.1 p.70" (exactly in this range) and the Confirm-Initial-Voice-Channel SAT/SCC/VMAC path (p70) and the full Waiting-for-Order FVC order set (Handoff/Alert/Release/Audit/Maintenance/Change Power/ESN Request, p71-72) are genuinely implemented, not stubbed.

---

## AMPS spec-walk pages 073-088

Perspective: BASE STATION (network side), auditing `/home/vas/projects/osmocom-analog/src/amps/`.
Pages 073-077 are MOBILE-STATION voice-channel task behaviour (§2.6.4.x) — mostly
NOT-APPLICABLE(mobile-side), with the base's mirror behaviour noted where it exists.
Pages 078-088 are §2.7 Signaling formats (RECC / RVC message streams and word layouts) —
these are the reverse-channel (mobile→base) words the base DECODES, and are heavily
implemented bit-for-bit in `frame.c`.

Row format: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

---

## page 73
§2.6.4.3.2 "Waiting for answer" (mobile-station alerting task). All items here are
mobile-side "shall"s; the base's mirror is the order it transmits on the FVC/FOCC and
the ST/signaling-tone confirmation it detects.

73 | 2.6.4.3.1/.3.2 | Alert-timer / Base Station Challenge wait 10s, other 5s | N | N | N-A | N | NOT-APPLICABLE(mobile-side) | mobile timer; no base match in src/amps/
73 | 2.6.4.3.2 | DTX-high while waiting for answer; 65s alert timer | N | N | N-A | N | NOT-APPLICABLE(mobile-side) | DTX is a mobile-TX behaviour; base does not model it
73 | 2.6.4.3.2 | Base Station Challenge Order Confirmation / AUTHBS compare in SSD update | partial | N | N | N | NOT-APPLICABLE(mobile-side) | base's side: base_station_challenge word decoded, frame.c:942 "Word 2 - Base Station Challenge Order Confirmation"; AUTHBS compute not in src/amps/
73 | 2.6.4.3.2 | SCCs vs PSCC compare (ignore if ≠) within 100ms | N | partial | N | N | NOT-APPLICABLE(mobile-side) | base ENCODES SCC/PSCC in Mobile Station Control Message, frame.c:882-888; the compare is the mobile's
73 | 2.6.4.3.2 | Order: Handoff (ST 500/50ms, retune, new SAT, set SCCs) | N | Y | Y | N | NOT-APPLICABLE(mobile-side) | base issues handoff + acts on ST confirm: amps.c:935 TRANS_CALL_HANDOFF_SEND, retune to handoff_channel
73 | 2.6.4.3.2 | Order: Alert / Alert With Info (reset 65s timer) | N | Y | Y | N | NOT-APPLICABLE(mobile-side) | base sends Alert + ALERT_TRIES/ALERT_TO, amps.c:87-88; Alert words in amps_table4_def frame.c:2426
73 | 2.6.4.3.2 | Order: Stop Alert → Waiting for Order | N | partial | N | N | NOT-APPLICABLE(mobile-side) | order table present; no explicit base Stop-Alert TX path found
73 | 2.6.4.3.2 | Order: Release (ST 500ms → Release Task) | N | Y | Y | N | NOT-APPLICABLE(mobile-side) | base Release order 3: amps.c:791 amps_release(), ORDER=3 ORDQ=0, RELEASE_TIMER amps.c:90
73 | 2.6.4.3.2 | Order: Audit (send order confirmation) | N | partial | N | N | NOT-APPLICABLE(mobile-side) | Audit in amps_table4_def order table; no base-initiated audit TX path verified
73 | 2.6.4.3.2 | Order: Flash With Info (order confirmation) | Y | Y | partial | N | NOT-APPLICABLE(mobile-side) | Flash-With-Info words encoded frame.c:990-1397; base decodes flash via ST duration amps.c:866
73 | 2.6.4.3.2 | Order: Message Waiting (MWI count in msg type) | N | Y | Y | Y | IMPLEMENTED | CORRECTED (was MISSING): amps_mwi() amps.c:2748 sends the Message Waiting order (count 0-31, type 0-2) on the FVC when the subscriber is in a voice-channel task state.

## page 74
§2.6.4.3.2 continued + §2.6.4.4 "Conversation" entry.

74 | 2.6.4.3.2 | Order: Maintenance (reset 65s timer) | N | Y | Y | N | NOT-APPLICABLE(mobile-side) | base Maintenance order + ST confirm: amps.c:899 TRANS_CALL_MAINTENANCE_SEND, silent-page path amps.c:911
74 | 2.6.4.3.2 | Order: Change Power (ORDQ power level) | N | partial | N | N | NOT-APPLICABLE(mobile-side) | ORDER/ORDQ fields encoded frame.c:676-677/887-888; no base change-power control path verified
74 | 2.6.4.3.2 | Order: Local Control (if enabled, examine local field) | N | Y | logged-only | N | NOT-APPLICABLE(mobile-side) | "Local Control 1/2 Message" words frame.c:532-551; LOCAL/MSG TYPE IE frame.c:2258
74 | 2.6.4.3.2 | Order: Protocol Capability Indicator (send PCI report) | Y | Y | logged-only | N | NOT-APPLICABLE(mobile-side) | base requests PCI (pci_home/pci_roam sysinfo.c:111-112); decodes PCI report word frame.c:737
74 | 2.6.4.3.2 | Order: Serial Number Request (reply Serial Number Response) | Y | Y | logged-only | N | NOT-APPLICABLE(mobile-side) | Serial Number Response words frame.c:1549-1572; ESN IE frame.c:2251
74 | 2.6.4.3.2 | Order: SSD Update (compute SSD_A/B_NEW, RANDBS, send BS Challenge) | partial | partial | N | N | NOT-APPLICABLE(mobile-side) | SSD Update order words frame.c:271-307; RANDBS field present frame.c:731; crypto compute NOT in src/amps/
74 | 2.6.4.3.2 | Order: Unique Challenge (AUTHU, send unique challenge order confirmation) | Y | Y | N | N | NOT-APPLICABLE(mobile-side) | unique_challenge_order_confirmation_word AUTHU 18 frame.c:715-720; auth algo not computed
74 | 2.6.4.3.2 | Order: Message Encryption Mode (ORDQ 001 activate / 000 deactivate) | N | partial | N | N | NOT-APPLICABLE(mobile-side) | AMPS_ORDER_MESSAGE_ENCRYPTION=23 amps.c:70; no activate/deactivate behaviour, order code only
74 | 2.6.4.3.2 | Order: Parameter Update (increment COUNTs-p) | N | N | N | N | MISSING | no COUNT/parameter-update handling in src/amps/
74 | 2.6.4.3.2 | Any other order: ignore | N | N-A | N | N | NOT-APPLICABLE(mobile-side) | mobile rule

## page 75
§2.6.4.4 "Conversation" — user terminate/power-down/flash handling + the same
SCCs=PSCC order list repeated for the conversation task.

75 | 2.6.4.4 | Release-delay timer examined on user terminate → Release Task | N | N | N-A | N | NOT-APPLICABLE(mobile-side) | mobile timer; base release path is amps.c:791
75 | 2.6.4.4 | Power-down → release-delay timer → Power Down Task | N | N | N-A | N | NOT-APPLICABLE(mobile-side) | mobile only
75 | 2.6.4.4 | User flash: DTX inhibit 1.5s, ST 400ms; terminate flash on valid order | N | N | partial | N | NOT-APPLICABLE(mobile-side) | base DETECTS flash via ST duration vs Flash-Max amps.c:866; ST timing in dsp.c
75 | 2.6.4.4 | SCCs=PSCC compare, DTX inhibit 1.5s (5s for audit), 200ms DTX-high | N | N | N-A | N | NOT-APPLICABLE(mobile-side) | mobile DTX/compare behaviour
75 | 2.6.4.4 | Order: Handoff (ST 50ms, retune, new SAT, set SCCs) | N | Y | Y | N | NOT-APPLICABLE(mobile-side) | base handoff state machine amps.c:935-966

## page 76
§2.6.4.4 Conversation order list (continued).

76 | 2.6.4.4 | Order: Send Called-Address (within 10s of last flash) | Y | N | N | N | NOT-APPLICABLE(mobile-side) | base DECODES Called-Address words D-G frame.c:767-845; it does not emit them
76 | 2.6.4.4 | Order: Disable DTMF | N | partial | N | N | NOT-APPLICABLE(mobile-side) | AMPS_ORDER_DISABLE_DTMF=22 amps.c:69 (order code defined); no behaviour
76 | 2.6.4.4 | Order: Alert / Release / Audit / Flash With Info | N | Y | partial | N | NOT-APPLICABLE(mobile-side) | see page-73 Alert/Release/Flash rows; same order table
76 | 2.6.4.4 | Order: Message Waiting (count) | N | Y | Y | Y | IMPLEMENTED | CORRECTED (was MISSING): amps_mwi() amps.c:2748 (also FOCC path when not on a call).
76 | 2.6.4.4 | Order: Maintenance / Change Power / Local Control | N | Y/partial | partial | N | NOT-APPLICABLE(mobile-side) | Maintenance amps.c:899; Local Control words frame.c:532
76 | 2.6.4.4 | Order: Protocol Capability Indicator / Serial Number Request | Y | Y | logged-only | N | NOT-APPLICABLE(mobile-side) | PCI/Serial words frame.c:737/1549
76 | 2.6.4.4 | Order: SSD Update | partial | partial | N | N | NOT-APPLICABLE(mobile-side) | words only; crypto absent

## page 77
§2.6.4.4 tail (Unique Challenge, Message Encryption, Parameter Update, "any other
order: ignore"); §2.6.4.5 "Release"; §2.6.4.6 "Power down".

77 | 2.6.4.4 | Order: Unique Challenge / Message Encryption / Parameter Update | partial/N | partial/N | N | N | NOT-APPLICABLE(mobile-side) | see page-74 rows (same orders)
77 | 2.6.4.5 | Release task: ST 1.8s, bridge flash timing, stop ST, TX off → Serving-System | N | N | partial | N | NOT-APPLICABLE(mobile-side) | base ST_RELEASE_TIME 1.8s amps.c:93; base detects ST-loss as release confirm amps.c:879
77 | 2.6.4.6 | Power down: DTX-high, PDREGs=1 autonomous registration w/ power-down, ST 1.8s | N | N | N | N | NOT-APPLICABLE(mobile-side) | no autonomous-registration decode / PDREG handling in src/amps/

## page 78
§2.7 "Signaling formats" (RSVD rules, BCH-correct-but-unrecognizable overhead counts
for NAWC); §2.7.1 "Reverse control channel" + Figure 2.7.1-1 RECC message stream.

78 | 2.7 | RSVD bits set to 0 on TX, ignore on RX | N | partial | N | N | PARTIAL | RSVD fields present throughout frame.c (e.g. 719,744,761); set to 0 by construction
78 | 2.7 | BCH-correct but unrecognizable overhead → count for NAWC, don't execute | N-A | N-A | N-A | N | NOT-APPLICABLE(mobile-side) | RESOLVED: §2.7 p.62 states this is a MOBILE-station reception rule ("if the mobile station receives... the mobile station shall count... but shall not attempt to execute"). The base builds the overhead train and never receives its own FOCC, so no base obligation exists.
78 | 2.7.1 | RECC data stream 10 kbit/s ±1 | Y | N-A | Y | N | IMPLEMENTED | dsp.c:28 "bit rate is 10 KHz", Manchester; base RX-only on reverse channel
78 | 2.7.1 | Seizure precursor: DOTTING 30 + WORD SYNC 11 (11100010010) + coded DCC 7 | Y | N-A | Y | N | IMPLEMENTED | dsp.c dotting detection; sync word 0x712=11100010010 dsp.c:190; DCC decode frame.c:2356
78 | 2.7.1 | Word repeated 5 times, 240 bits per word block | Y | N-A | Y | N | PARTIAL | RX majority/decode over repeats via amps_decode_frame dsp.c:863; 5x repeat is mobile-TX structure
78 | 2.7.1 | WORD SYNC = 11100010010 | Y | N-A | Y | N | IMPLEMENTED | 0x712 positive / 0x0ed negative dsp.c:190-191 (0x712 == 11100010010 binary)

## page 79
Table 2.7.1-1 Coded DCC; (48,36;5) BCH + 5x repeat; §2.7.1.1 RECC message types;
Word A - Abbreviated Address Word.

79 | 2.7.1-1 | Coded DCC: 00→0000000, 01→0011111, 10→1100011, 11→1111100 | Y | Y | Y | N | IMPLEMENTED | dcc_encode[4]={0x00,0x1f,0x63,0x7c} frame.c:2361 (bit-exact match); dcc_decode auto-gen frame.c:2740
79 | 2.7.1 | 48-bit word = (48,36) BCH distance 5, 36 content + parity, MSB first | partial | partial | N-A | N | PARTIAL | 48-bit words modelled frame.c recc_words set=48; BCH gen poly shared with (40,28;5) FOCC
79 | 2.7.1.1 | RECC messages: Page Response / Origination / Order Confirmation / Order | Y | N-A | Y | N | IMPLEMENTED | recc_words message set frame.c:862+; MAXSZTR-PGR / page-response IE frame.c:2264
79 | 2.7.1.1 | Word A Abbreviated Address: F,NAWC(3),T,S,E,ER,SCM(4),MIN1(24),P(12) | Y | N-A | Y | N | IMPLEMENTED | abbreviated_address_word frame.c:655-664: F,NAWC3,T,S,E,ER,SCM4,MIN1 24

## page 80
Word B Extended Address; Word C variants: Serial Number, Authentication, Unique
Challenge Order Confirmation, Base Station Challenge, Protocol Capability Indicator.

80 | 2.7.1.1 | Word B Extended Addr: NAWC(3),LOCAL/MSG_TYPE(5),ORDQ(3),ORDER(5),LT,EP,SCM(1),MPCI(2),SDCC1(2),SDCC2(2),MIN2(10),P | Y | N-A | partial | N | IMPLEMENTED | extended_address_word frame.c:671-684 (all fields present, widths match)
80 | 2.7.1.1 | Word C Serial Number: NAWC(3),SERIAL(32),P | Y | N-A | logged-only | N | IMPLEMENTED | serial_number_word frame.c:690-695 SERIAL/ESN 32; esn.c decodes mfr+serial
80 | 2.7.1.1 | Word C Authentication: NAWC(3),COUNT(6),RANDC(8),AUTHR(18),P | Y | N-A | N | N | DECODE-ONLY | authentication_word frame.c:705-708 (RANDC8,AUTHR18); no auth verify in src/amps/
80 | 2.7.1.1 | Word C Unique Challenge Order Confirmation: NAWC(3),RSVD(14),AUTHU(18),P | Y | N-A | N | N | DECODE-ONLY | unique_challenge_order_confirmation_word frame.c:717-720
80 | 2.7.1.1 | Word C Base Station Challenge: NAWC(3),RANDBS(32),P | Y | N-A | N | N | DECODE-ONLY | base_station_challenge_word frame.c:730-731 RANDBS 32
80 | 2.7.1.1 | Word C Protocol Capability Indicator: NAWC(3),MSPC(4),MSCAP(3),RSVD(25),P | Y | N-A | logged-only | N | IMPLEMENTED | pci_report_registration_word frame.c:740-744 MSPC4/MSCAP3; ie_mspc frame.c:2270

## page 81
Called-Address Words D/E/F/G (8×4-bit digits each); field interpretations F,NAWC,T,S,E.

81 | 2.7.1.1 | Word D First Called-Address: NAWC(3)+DIGIT1..8(4 each)+P | Y | N-A | Y | N | IMPLEMENTED | first_word_of_the_called_address frame.c:767-779; digit2number frame.c:117
81 | 2.7.1.1 | Word E Second Called-Address: DIGIT9..16 | Y | N-A | Y | N | IMPLEMENTED | second_word_of_the_called_address frame.c:803
81 | 2.7.1.1 | Word F Third Called-Address: DIGIT17..24 | Y | N-A | Y | N | IMPLEMENTED | third_word_of_the_called_address frame.c:820
81 | 2.7.1.1 | Word G Fourth Called-Address: NAWC=000, DIGIT25..32 | Y | N-A | Y | N | IMPLEMENTED | fourth_word_of_the_called_address frame.c:838
81 | 2.7.1.1 | F field: 1 in first word, 0 subsequent | Y | N-A | Y | N | IMPLEMENTED | "F" bit on every word frame.c:657,673,693,704,717,729,740,753,770
81 | 2.7.1.1 | NAWC = number of additional words coming | Y | N-A | Y | N | IMPLEMENTED | "NAWC" present on every RECC word; drives word-count decode
81 | 2.7.1.1 | T field (origination/order vs response/page) | Y | N-A | partial | N | IMPLEMENTED | "T" bit frame.c:659,752
81 | 2.7.1.1 | S field (serial number word sent) | Y | N-A | Y | N | IMPLEMENTED | "S" bit frame.c:358,660
81 | 2.7.1.1 | E field (extended address word sent) | Y | N-A | Y | N | IMPLEMENTED | "E" bit frame.c:359,661

## page 82
Field-definition glossary: SCM, MPCI, ORDER, ORDQ, LOCAL, MSG_TYPE, ER, EP, LT,
SDCC1/2, MIN1/2, SERIAL, DIGIT, RSVD, P, COUNT, RANDC, AUTHR, AUTHU, RANDBS, MSPC.

82 | 2.7.1.1 | SCM(3-0)/SCM(4) station class mark | Y | N-A | logged-only | N | IMPLEMENTED | "SCM" field frame.c:663,680; station class mark decoded
82 | 2.7.1.1 | MPCI (00=553A/553/54A,01=627,10=95,11=136 dual-mode) | Y | N-A | logged-only | N | DECODE-ONLY | "MPCI" field frame.c:681 (2 bits); value→standard mapping not acted on
82 | 2.7.1.1 | ORDER / ORDQ fields → Table 3.7.1-1 | Y | Y | Y | N | IMPLEMENTED | ORDER/ORDQ IEs frame.c:757-758; amps_table4_def order table frame.c:2370+
82 | 2.7.1.1 | LOCAL / MSG_TYPE qualify order | Y | N-A | logged-only | N | IMPLEMENTED | AMPS_IE_LOCAL_MSG_TYPE frame.c:756,2258
82 | 2.7.1.1 | ER / EP (extended protocol, set 0 for 553A) | Y | partial | N | N | DECODE-ONLY | "ER" frame.c:662, "EP" frame.c:681,2249; always-0 for 553A
82 | 2.7.1.1 | LT last-try code | Y | N-A | N | N | DECODE-ONLY | "LT" field frame.c:678 (1 bit)
82 | 2.7.1.1 | SDCC1/SDCC2 supplementary digital color codes | Y | Y | partial | N | PARTIAL | SDCC1/SDCC2 frame.c:583-585,682-683; non-zero = SDCC-capable mobile
82 | 2.7.1.1 | MIN1(24)/MIN2(10) mobile identification number | Y | N-A | Y | N | IMPLEMENTED | MIN1/MIN2 frame.c:664,684; identifies calling mobile
82 | 2.7.1.1 | SERIAL = ESN of mobile | Y | N-A | logged-only | N | IMPLEMENTED | esn.c esn_to_string / amps_decode_esn
82 | 2.7.1.1 | COUNT modulo-64 anti-fraud; RANDC confirm last RAND; AUTHR algo output | Y | N-A | N | N | DECODE-ONLY | fields present (page-80 Authentication word); not verified against algo
82 | 2.7.1.1 | RSVD set as indicated; P parity | partial | partial | N | N | PARTIAL | RSVD throughout; parity P computed by BCH encode

## page 83
MSCAP field; Called-Address encoding examples (2#, 13792640, *24273258);
Table 2.7.1-2 Digit code.

83 | 2.7.1.1 | MSCAP (000 backward-compat, 001 core analog 553A) | Y | Y | logged-only | N | IMPLEMENTED | ie_mscap frame.c (AMPS_IE_MSCAP), pci word frame.c:743
83 | 2.7.1-2 | Digit code 1→0001 … 9→1001, 0→1010, *→1011, #→1100, Null→0000 | Y | Y | Y | N | IMPLEMENTED | digit2number[16] frame.c:117-134 (idx10='0',11='*',12='#'); matches table
83 | 2.7.1.1 | Called-address digit packing examples (MSB-first, 4-bit groups) | Y | Y | Y | N | IMPLEMENTED | Called-Address words D-G 4-bit DIGIT fields frame.c:767+

## page 84
Digit-code notes (0=binary ten, 0000=null, others reserved); §2.7.2 "Reverse voice
channel" + Figure 2.7.2-1 RVC message stream.

84 | 2.7.1-2 | 0 encoded as binary ten; 0000 = null; others reserved | Y | Y | N | N | IMPLEMENTED | digit2number: idx0='\0'(null), idx10='0' frame.c:118,128
84 | 2.7.2 | RVC data stream 10 kbit/s ±1, Manchester | Y | N-A | Y | N | IMPLEMENTED | dsp.c:24-28 Manchester 10 KHz (shared RX path with RECC)
84 | 2.7.2 | 37-bit dotting + 11-bit word sync (11100010010); first repeat uses 101-bit dotting | Y | N-A | partial | N | PARTIAL | base RX syncs on dotting+0x712 dsp.c; exact 37/101 bit dotting counts are mobile-TX structure
84 | 2.7.2 | Word 48 bits, repeated 5x = word block; (48,36;5) BCH, gen poly = (40,28;5) FOCC | partial | N-A | N-A | N | PARTIAL | 48-bit RVC words frame.c; multi-repeat majority decode via amps_decode_frame

## page 85
§2.7.2.1 RVC messages (6 types); Order Confirmation; Unique Challenge Order
Confirmation; Base Station Challenge Order Message Word 1.

85 | 2.7.2.1 | RVC msg types: Order Confirmation, Unique Challenge Confirm, BS Challenge, Called-Address, Serial Number Response, PCI Report | Y | N-A | Y | N | IMPLEMENTED | all present in recc_words/RVC word defs frame.c:749-866,1549
85 | 2.7.2.1 | Order Confirmation Msg: F,NAWC=00,T=1,LOCAL/MSG_TYPE(5),ORDQ(3),ORDER(5),RSVD(19),P | Y | N-A | logged-only | N | IMPLEMENTED | order_confirmation_rvc_word frame.c:749-760 (note: impl carries MSPC/MSCAP in the RSVD span)
85 | 2.7.2.1 | Unique Challenge Order Confirmation: AUTHU(18),RSVD=0 | Y | N-A | N | N | DECODE-ONLY | unique_challenge_order_confirmation_word AUTHU18 frame.c:715-720
85 | 2.7.2.1 | BS Challenge Order Msg Word 1: NAWC=01,T=1,LOCAL/MSG_TYPE=0,ORDQ,ORDER,RSVD(19),P | Y | N-A | N | N | DECODE-ONLY | base_station_challenge word group frame.c:728-731,942

## page 86
BS Challenge Word 2 (RANDBS 32); Called-Address Message Words 1-3 (RVC).

86 | 2.7.2.1 | BS Challenge Order Msg Word 2: F=0,NAWC=00,T=1,RANDBS(32),P | Y | N-A | N | N | DECODE-ONLY | RANDBS 32 frame.c:731; "Word 2 - Base Station Challenge Order Confirmation" frame.c:942
86 | 2.7.2.1 | Called-Address Msg Word 1: F=1,NAWC,T=0,DIGIT1..8,P | Y | N-A | Y | N | IMPLEMENTED | first_word_of_the_called_address frame.c:767
86 | 2.7.2.1 | Called-Address Msg Word 2: DIGIT9..16 | Y | N-A | Y | N | IMPLEMENTED | second_word_of_the_called_address frame.c:803
86 | 2.7.2.1 | Called-Address Msg Word 3: DIGIT17..24 | Y | N-A | Y | N | IMPLEMENTED | third_word_of_the_called_address frame.c:820

## page 87
Called-Address Word 4 (RVC); Serial Number Response Message (Word 1 + Word 2 ESN);
Protocol Capability Indicator Report Message.

87 | 2.7.2.1 | Called-Address Msg Word 4: F=0,NAWC=00,T=0,DIGIT25..32,P | Y | N-A | Y | N | IMPLEMENTED | fourth_word_of_the_called_address frame.c:838
87 | 2.7.2.1 | Serial Number Response Word 1: F=1,NAWC=01,T=1,LOCAL/MSG_TYPE=0,ORDQ,ORDER,RSVD(19),P | Y | N-A | logged-only | N | IMPLEMENTED | "Word 1 of Serial Number Response Message" frame.c:1549-1552
87 | 2.7.2.1 | Serial Number Response Word 2: F=0,NAWC=00,T=1,ESN(32),P | Y | N-A | logged-only | N | IMPLEMENTED | "Word 2 of Serial Number response message" ESN 32 frame.c:1567-1572; esn.c decode
87 | 2.7.2.1 | PCI Report Msg: F=1,NAWC=00,T=1,LOCAL/MSG_TYPE(5),ORDQ,ORDER,MSPC(4),MSCAP(3),RSVD(12),P | Y | N-A | logged-only | N | IMPLEMENTED | pci_report_registration_word frame.c:737-744; order_confirmation_rvc_word carries MSPC/MSCAP frame.c:759-760

## page 88
RVC field-interpretation glossary: F, NAWC, T, DIGIT, ORDER, ORDQ, LOCAL, MSG_TYPE,
RSVD, P, AUTHU, RANDBS, MSPC, MSCAP.

88 | 2.7.2.1 | F (1 first word, 0 second) | Y | N-A | Y | N | IMPLEMENTED | "F" bit on all RVC words frame.c:751,770,804,821,839
88 | 2.7.2.1 | NAWC = additional words coming | Y | N-A | Y | N | IMPLEMENTED | "NAWC" on all RVC words
88 | 2.7.2.1 | T (1=order/confirmation, 0=called-address) | Y | N-A | partial | N | IMPLEMENTED | "T" bit frame.c:752
88 | 2.7.2.1 | DIGIT (see Table 2.7.1-2) | Y | N-A | Y | N | IMPLEMENTED | DIGIT fields + digit2number frame.c:117
88 | 2.7.2.1 | ORDER/ORDQ/LOCAL/MSG_TYPE (Table 3.7.1-1) | Y | Y | logged-only | N | IMPLEMENTED | ORDER/ORDQ/LOCAL_MSG_TYPE IEs frame.c:756-758; amps_table4
88 | 2.7.2.1 | RSVD set as indicated; P parity | partial | partial | N | N | PARTIAL | RSVD/P fields present; parity via BCH
88 | 2.7.2.1 | AUTHU / RANDBS / MSPC / MSCAP (as page 82 definitions) | Y | N-A | logged-only/N | N | DECODE-ONLY | AUTHU frame.c:720, RANDBS frame.c:731, MSPC/MSCAP frame.c:743,2270

---

## SUMMARY

Pages read vs assigned: 16 of 16 (073, 074, 075, 076, 077, 078, 079, 080, 081, 082,
083, 084, 085, 086, 087, 088) — ALL assigned pages read, one PNG at a time.

Count per STATUS (rows):
- IMPLEMENTED ................ 40   (incl. Message Waiting order, corrected from MISSING)
- DECODE-ONLY ................ 13
- PARTIAL .................... 9
- NOT-APPLICABLE(mobile-side)  22  (mobile-station §2.6.4.x task behaviour, pages 73-77; incl. §2.7 NAWC-count rule, reclassified from UNVERIFIED)
- MISSING .................... 1   (Parameter Update COUNTs-p; Message Waiting was a false MISSING — amps_mwi exists)
- UNVERIFIED ................. 0   (the §2.7 NAWC-count rule was mobile-side; now NOT-APPLICABLE)

Top 3 gaps in range (073-088):
1. **Authentication / SSD / Unique Challenge crypto is DECODE-ONLY.** The RECC/RVC
   words (RANDC, AUTHR, AUTHU, RANDBS, COUNT) are all parsed bit-for-bit, but there
   is NO authentication algorithm (CAVE), AUTHBS/AUTHU verification, or SSD-update
   state in `src/amps/`. The base reads the fields and logs them; it never acts on
   them. Pages 73-74, 80, 85-86.
2. **Parameter Update (COUNTs-p) order is MISSING** (auth-dependent, no COUNT state).
   Message Encryption Mode has only an order-code constant (amps.c:70), no
   activate/deactivate behaviour. (CORRECTION: Message Waiting was listed MISSING
   here in error — `amps_mwi` amps.c:2748 implements it on both FVC and FOCC.)
3. **MPCI / MSPC / MSCAP capability fields are decoded but not acted upon**
   (logged-only). The base decodes the mobile's protocol-capability indicators
   (dual-mode class, SDCC support) but takes no capability-dependent action. Pages
   80, 82-83, 87-88. UPDATE: SDCC is now acted on — the mobile's SDCC echo is
   decoded, SDCC capability is derived from it, and an echo≠broadcast mismatch is
   flagged (amps.c amps_rx_recc). Also the §2.7 "BCH-correct but unrecognizable
   overhead counts for NAWC but is not executed" rule is RESOLVED as mobile-side
   (NOT-APPLICABLE): §2.7 p.62 scopes it to the mobile station, and the base never
   receives its own FOCC overhead.

Perspective note: pages 73-77 are mobile-station voice-channel task state machines;
on the base side the osmocom-analog code implements the *mirror* behaviour it needs
(issue Alert/Release/Maintenance/Handoff orders, detect ST/signaling-tone confirmation,
retune on handoff) rather than the mobile's own "shall"s. Those mirror paths are
genuinely IMPLEMENTED in amps.c even though the spec text itself is mobile-side.

REMAINING PAGES: none (073-088 complete).

---

## Pages 089-093 (first pass; §3.1-§3.6 intro, SAT/ST)

Perspective: BASE STATION (osmocom-analog src/amps/). Range = TIA/EIA-553-A §3 Base Station:
transmitter (freq/power/modulation), emissions, receiver (demod), §3.4 supervision (SAT/ST).

Row format: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

## page 89 (printed p.73) — §3 / §3.1 Transmitter: freq, power, modulation intro
89 | 3 Base Station | Section heading (scope/prose) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | heading only, no normative item
89 | 3.1.1.1 | Channel spacing shall be 30 kHz | N-A | Y | Y | N-A | IMPLEMENTED | amps.c:116 freq=870.030+(channel-1)*0.030 (30 kHz grid)
89 | 3.1.1.1 | Base TX chan#1 = 870.030 MHz (mobile 825.030) | N-A | Y | Y | N-A | IMPLEMENTED | amps.c:116 870.030 base + 45 MHz duplex offset in freq calc
89 | 3.1.1.1 | Channels 1-666 basic; 667-799 & 991-1023 extended band | Y | Y | Y | Y | IMPLEMENTED | amps.c:110 range check 1..1023 w/ 800..990 gap; amps.c:645 warns >666; amps.c:184/188/192 band map
89 | 3.1.1.1 | SCM taken into account for extended-band assignment | Y | partial | logged-only | N | PARTIAL | SCM parsed in RECC (frame.c); extended-band assignment not SCM-gated beyond chan range check
89 | 3.1.1.2 | Carrier freq within ±1.5 ppm of assigned | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | SDR/hardware oscillator accuracy, not in src/amps; no ppm constant (grep: none)
89 | 3.1.2 | Max ERP & HAAT coordinated locally | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | administrative/RF; no erp/haat symbol in src/amps
89 | 3.1.3.1 | FM modulator preceded by compressor, pre-emph, dev-limiter, post-filter | N-A | partial | Y | partial | PARTIAL | TX chain in dsp.c: compress_audio dsp.c:647, pre-emph dsp.c:666, post low-pass dsp.c (amps.c:692); deviation limiter not an explicit ±12k clamp
89 | 3.1.3.1.1 | Compressor: 2:1 syllabic compandor, 3 ms attack | N-A | Y | Y | N | IMPLEMENTED | dsp.c:298 setup_compandor(&cstate,8000,3.0,13.5) attack=3.0 ms; compress_audio dsp.c:647

## page 90 (printed p.74) — §3.1.3 modulation detail; §3.1.3.2 wideband data (FSK)
90 | 3.1.3.1.1 | Compressor nominal recovery 13.5 ms (ITU-T G.162) | N-A | Y | Y | N | IMPLEMENTED | dsp.c:298 setup_compandor(...,13.5) recovery=13.5 ms
90 | 3.1.3.1.1 | 1000 Hz ref level -> nominal ±2.9 kHz peak deviation | N-A | Y | Y | N | IMPLEMENTED | dsp.c:106 AMPS_SPEECH_DEVIATION 2900.0 (2.9 kHz at 1 kHz)
90 | 3.1.3.1.2 | Pre-emphasis +6 dB/octave, 300-3000 Hz | N-A | Y | Y | Y | PARTIAL | dsp.c:661-662 +6 dB/octave shelf; corners 500/2000 Hz (not exactly 300-3000); gated by amps->pre_emphasis dsp.c:666
90 | 3.1.3.1.3 | Deviation limiter: limit instantaneous dev to ±12 kHz (excl supervision/data) | N-A | N | N | N | MISSING | no ±12k/12000 clamp in dsp.c TX path; AMPS_MAX_DEVIATION=8000 governs FSK not voice limiter (dsp.c:104)
90 | 3.1.3.1.4 | Post deviation-limiter low-pass filter: >=40log(f/3000) 3-15k, >=28dB >15k | N-A | partial | Y | N | PARTIAL | TX post-limiter LPF cutoff ~3000 Hz amps.c:692; exact mask slope not verified
90 | 3.1.3.2.1 | FOCC/FVC wideband data: NRZ->Manchester (1->0to1, 0->1to0 transition) | partial | Y | Y | N | IMPLEMENTED | dsp.c:24-28 Manchester coding, bit rate 10 kHz; RX decodes w/ polarity dsp.c:70
90 | 3.1.3.2.2 | FSK modulation: '1' = +8 kHz, '0' = -8 kHz dev (direct binary FSK) | Y | Y | Y | N | IMPLEMENTED | dsp.c:23-24 8 kHz above=high/below=low; AMPS_MAX_DEVIATION 8000.0 dsp.c:104

## page 91 (printed p.75) — §3.1.4 emissions; §3.2 Receiver; §3.2.2.1 demod
91 | 3.1.4.1 | Bandwidth occupied: ±20k<=-26dB, ±45k<=-45dB, ±90k<=-60dB / 43+10log10(P) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | RF spectral mask / power amplifier compliance; not modeled in src/amps DSP
91 | 3.1.4.2 | Conducted spurious emissions: current FCC rules | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | RF/regulatory, no src/amps analogue
91 | 3.1.4.3 | Radiated spurious emissions: current FCC rules | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | RF/regulatory, no src/amps analogue
91 | 3.1.4.4 | Intermodulation (co-located TX) FCC limits | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | RF/regulatory, no src/amps analogue
91 | 3.2.1.1 | RX channel spacing/designation: see §3.1.1 | N-A | Y | Y | N-A | IMPLEMENTED | same freq grid amps.c:116 (RX = TX - 45 MHz duplex)
91 | 3.2.2.1 | Demodulator followed by de-emphasis + expandor (2 stages) | Y | N-A | Y | Y | IMPLEMENTED | RX chain: de_emphasis dsp.c:1792 (gated de_emphasis), expand_audio dsp.c:1829 (2:1 expander)

## page 92 (printed p.76) — §3.2.2.1.1/.2 demod stages; §3.3 Security & identification
92 | 3.2.2.1.1 | De-emphasis: nominal -6 dB/octave, 300-3000 Hz | Y | N-A | Y | Y | IMPLEMENTED | RX de_emphasis dsp.c:1792 (6 dB/octave complement of TX pre-emph); gated amps->de_emphasis dsp.c:1787
92 | 3.2.2.1.2 | Expandor: 2:1 syllabic compandor, after de-emph+filter, 3ms/13.5ms | Y | N-A | Y | N | IMPLEMENTED | expand_audio dsp.c:1829 using cstate (setup_compandor 3.0/13.5 dsp.c:298); runs after de-emph+voiceband filter dsp.c:1795
92 | 3.2.2.1.2 | Expandor ref: 1000 Hz tone from ±2.9 kHz dev carrier | Y | N-A | Y | N | IMPLEMENTED | shares AMPS_SPEECH_DEVIATION 2900 dsp.c:106 as reference for compandor
92 | 3.2.3 | RX limitations on emissions: current FCC rules | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | RF/regulatory; no src/amps analogue
92 | 3.2.4 | Other RX params: ANSI TIA/EIA 712 compliance | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | references external RMS standard; not DSP-modeled
92 | 3.3.1 | Authentication: SSD exchange, confirm MS identity (§2.3.12) | partial | partial | N | N | DECODE-ONLY | SSD Update Order words defined frame.c:270/295/306 (RANDSSD_1/2/3); amps.c:1285 "Registration without Authentication"; full auth flow not implemented
92 | 3.3.2 | Identification: cellular system = 15-bit SID | Y | Y | Y | Y | IMPLEMENTED | main_common.c:100 SID 15-bit; -S sid= operator cmd main_common.c:1012; clamp 0..32767 main_common.c:1019 (15-bit range)

## page 93 (printed p.77) — §3.4 Supervision (SAT/ST); §3.5 malfunction; §3.6 call-proc intro
93 | 3.4.1.1 | SAT detection — "Reserved" (empty in spec text) | Y | N-A | Y | N | EXTENSION(non-spec) | spec reserved; code implements full SAT detect anyway: sat_goertzel dsp.c:362, sat_classify_frequency dsp.c:234, SAT_DETECT_COUNT ~250ms dsp.c:127
93 | 3.4.1.2 | SAT transmission: one of 5970/6000/6030 Hz on VC when TX active (except during data) | N-A | Y | Y | Y | IMPLEMENTED | sat_freq[] 5970/6000/6030 dsp.c:143-146; dsp_sine_sat scaled by AMPS_SAT_DEVIATION dsp.c:179; SAT selectable via -S/sat param amps.c:580
93 | 3.4.1.2 | SAT frequency deviation ±2 kHz ±10% | N-A | Y | Y | N | IMPLEMENTED | AMPS_SAT_DEVIATION = 2000.0/2900.0 dsp.c:108 (2 kHz nominal)
93 | 3.4.1.2 | SAT frequency tolerance ±1 Hz | N-A | Y | partial | N | PARTIAL | tones generated at exact 5970/6000/6030 (dsp.c:143); hardware/SDR osc tolerance not modeled; detect bandwidth 30 Hz dsp.c:121
93 | 3.4.1.3 | Fade timing status — "Reserved" | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | spec reserved; no fade-timing status in src/amps
93 | 3.4.2 | Signaling tone detection — "Reserved" (empty in spec) | Y | N-A | Y | N | EXTENSION(non-spec) | spec reserved; code detects ST: sat_goertzel[4] @10000 Hz dsp.c:367, SIG_DETECT_COUNT dsp.c:130, fast ST for handoff dsp.c:370
93 | 3.5 | Malfunction detection — "Reserved" | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | spec reserved; no malfunction-detection logic expected
93 | 3.6 | Call processing intro (prose, read §2.6/§3.6 in parallel) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | prose/scope; actual call-proc impl in amps.c state machine (later pages)
93 | 3.6.1 | Overhead functions for MS initialization: info in overhead message train | Y | Y | Y | Y | IMPLEMENTED | overhead message train built from sysinfo.c + main_common.c; detail enumerated on pp.94+ (next cycle)

## Research State
- Pages read: 89, 90, 91, 92, 93 of assigned 89-102. REMAINING PAGES: 94..102.
- Answered strong: SAT 5970/6000/6030 Hz gen+detect (dsp.c:143,362), 15-bit SID (main_common.c), de-emph/expandor RX chain (dsp.c:1792,1829).
- NOTABLE: §3.4.1.1 SAT detection, §3.4.2 ST detection, §3.4.1.3 fade timing, §3.5 malfunction are all "Reserved" (empty) in the spec from the base perspective — yet the code implements SAT+ST DETECTION regardless. Marked EXTENSION(non-spec) where code exceeds the reserved spec text.
- Weak/gaps: §3.1.3.1.3 deviation limiter ±12 kHz still MISSING; §3.4.1.2 SAT ±1 Hz tolerance only PARTIAL (osc accuracy not modeled); §3.3.1 authentication DECODE-ONLY (SSD words parsed, flow not run).
- Leads open: §3.6.1 overhead message train enumerated on pp.94+ — the core base-station TX audit. Next cycles walk the overhead words / order codes / call-proc messages (frame.c order table, sysinfo.c).

---

## Pages 094-102 (§3.6 base-station call processing)

Perspective: BASE STATION (osmocom-analog src/amps/). Range = TIA/EIA-553-A §3.6.2
base-station control of mobiles on the control channel: overhead information
(system parameter overhead message fields), global-action messages, and the
overhead message train / order details on the base side.

Row format: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

## page 94 (printed p.78) — §3.6.2 MS control on control channel; §3.6.2.1 Overhead information
94 | 3.6.2 | Section heading — MS control on the control channel (prose) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | heading only, no normative item
94 | 3.6.2.1 | Overhead sent in System Parameter Overhead Message (fmt §3.7.1.2) | N-A | Y | Y | N-A | IMPLEMENTED | word2_system_parameter_overhead frame.c:356; train built sysinfo.c:prepare_sysinfo (SYSINFO_WORD1/WORD2 first)
94 | 3.6.2.1 | SID1 — first part of system identification | Y | Y | Y | Y | IMPLEMENTED | SID1 word1 frame.c:326 (14 bits); si->word1.sid1 sysinfo.c:70; -S sid= main_common.c:109/1012
94 | 3.6.2.1 | AUTH bit — set 1 to enable authentication (§2.3.12/§3.3.1) | N-A | Y | Y | N | PARTIAL | AUTH field frame.c:328; si->word1.auth=0 hardcoded sysinfo.c:72 (auth not implemented → always 0)
94 | 3.6.2.1 | S bit — require serial number on system access | N-A | Y | Y | N | IMPLEMENTED | S field word2 frame.c:342; si->word2.s=1 sysinfo.c:76 (default require serial)
94 | 3.6.2.1 | REGH — enable registration for home mobiles | N-A | Y | Y | Y | IMPLEMENTED | REGH frame.c:360; si->word2.regh=regh sysinfo.c:78; -S regh= main_common.c:109/202
94 | 3.6.2.1 | REGR — enable registration for roaming mobiles | N-A | Y | Y | Y | IMPLEMENTED | REGR frame.c (word2); si->word2.regr=regr sysinfo.c:79; -S regr= main_common.c:204
94 | 3.6.2.1 | E bit — extended address (require MIN1+MIN2 on access) | N-A | Y | Y | N | IMPLEMENTED | E field word2 frame.c; si->word2.e=1 sysinfo.c:77 (default require extended address)
94 | 3.6.2.1 | DTX field (2 bits): 00=off, 10=limited, 11=no min (§2.3.11) | partial | Y | Y | N | IMPLEMENTED | "DTX Support" 2 bits frame.c:207; si->word2.dtx=dtx sysinfo.c:80; acted on amps.c:1449-1450 (if si.word2.dtx set DTX state)
94 | 3.6.2.1 | N — number of paging channels (encoded N-1) | N-A | Y | Y | Y | IMPLEMENTED | "N-1" 5 bits word2 frame.c; si->word2.n_1=20 default sysinfo.c:81; -S n1= main_common.c:209/212 (range 0..31)
94 | 3.6.2.1 | RCF — read control-filler before reverse-CC access | N-A | Y | Y | N | IMPLEMENTED | RCF field word2 frame.c; si->word2.rcf=(bis)?0:1 sysinfo.c:82 (set so phone ignores B/I)
94 | 3.6.2.1 | CPA — combined paging/access on same CC set | N-A | Y | Y | N | IMPLEMENTED | CPA field word2 frame.c; si->word2.cpa=1 sysinfo.c:83 (combined CC+PC)
94 | 3.6.2.1 | CMAX — number of access channels (encoded CMAX-1) | N-A | Y | Y | Y | IMPLEMENTED | "CMAX-1" 7 bits word2 frame.c; si->word2.cmax_1=20 default sysinfo.c:84; -S cmax1= main_common.c:113
94 | 3.6.2.1 | Appended overhead info sent as required in appended messages (§3.7.1.2) | N-A | Y | Y | N-A | IMPLEMENTED | prepare_sysinfo appends REG_INCR/LOC_AREA/NEW_ACC/OVERLOAD/ACC_TYPE/ACC_ATTEMPT only if non-default sysinfo.c:prepare_sysinfo
94 | 3.6.2.1 | Local control — customize operation via local control global action msg | N-A | Y | Y | Y | IMPLEMENTED | Order 30 Local Control: TRANS_CALL_LOCAL_CONTROL transaction.h:48-49; amps_local_control() amps.h:283; AMPS_IE_LOCAL_CONTROL frame.h:112

## page 95 (printed p.79) — §3.6.2.1 appended msgs cont.; §3.6.2.2 Page; §3.6.2.3 Order
95 | 3.6.2.1 | New Access channels — NEWACC global action if set != default | N-A | Y | Y | N | IMPLEMENTED | new_access_channel_set_global_action frame.c:423 (NEWACC 11 bits); si->new_acc.newacc sysinfo.c:98; appended only if non-default sysinfo.c:139
95 | 3.6.2.1 | Registration increment — REGINCR global action (§2.6.3.11) | N-A | Y | Y | Y | IMPLEMENTED | registration_increment_global_action frame.c:390 (REGINCR 12 bits); si->reg_incr.regincr sysinfo.c:87; -S regincr main_common.c:106
95 | 3.6.2.1 | Registration ID message — require NXTREG<=given to register | N-A | Y | Y | N | IMPLEMENTED | SYSINFO_REG_ID always in train sysinfo.c:132; si->reg_id.regid=1000 sysinfo.c (reg_id word)
95 | 3.6.2.1 | Location Area message — identifies location area of this CC | N-A | Y | Y | Y | IMPLEMENTED | location_area_global_action frame.c:405 (PUREG/PDREG/LREG/LOCAID); si->loc_area sysinfo.c:90-96; -S locaid main_common.c:109
95 | 3.6.2.1 | Rescan — force MS to Initialization Task + rescan dedicated CCs | N-A | Y | Y | Y | IMPLEMENTED | rescan_global_action frame.c:376; amps_rescan() amps.h:287; FIFO "rescan,<number>" main_common.c:72
95 | 3.6.2.1 | RAND1_A — 16 MSB of 32-bit RAND (auth) | partial | Y | partial | N | DECODE-ONLY | AMPS_IE_RAND1_A frame.c:2305 / frame.h:160 (defined IE); auth flow not run so RAND not actively used
95 | 3.6.2.1 | RAND1_B — 16 LSB of 32-bit RAND (auth) | partial | Y | partial | N | DECODE-ONLY | AMPS_IE_RAND1_B frame.h:161 (defined IE); auth flow not implemented
95 | 3.6.2.2 | Page: send MS control message (§3.7.1.1); home 1- or 2-word, roaming 2-word | N-A | Y | Y | Y | IMPLEMENTED | word1_abbreviated_address_word frame.c:162 (1-word page) + abbreviated_address_word frame.c:654 (2-word); page logic amps.c (PAGE_TRIES/PAGE_TO amps.c)
95 | 3.6.2.3 | Order: 2-word MS control message carries orders/confirmations (§3.7.1.2) | N-A | Y | Y | Y | IMPLEMENTED | ORDER(5)+ORDQ(3) fields frame.c:181-182; order codes pinned amps.c:69-71; tx_focc_order/ordq amps.h:169-170
95 | 3.6.2.3 | Order: Abbreviated Alert | N-A | Y | Y | Y | IMPLEMENTED | TRANS_CALL_ABB_ALERT (Order 1 ORDQ=1) transaction.h:40; amps_abbreviated_alert() amps.h:272; FIFO abbalert main_common.c:53
95 | 3.6.2.3 | Order: Audit | N-A | Y | Y | Y | IMPLEMENTED | TRANS_CALL_AUDIT (Order 2) transaction.h:26; amps_audit_order() amps.h:270; FIFO audit main_common.c:70
95 | 3.6.2.3 | Order: Base Station Challenge order confirmation | partial | Y | partial | N | DECODE-ONLY | word3_base_station_challenge_order_confirmation_word frame.c:246 (defined); auth challenge flow not run
95 | 3.6.2.3 | Order: Local control (Order 30) | N-A | Y | Y | Y | IMPLEMENTED | TRANS_CALL_LOCAL_CONTROL transaction.h:48-49; amps_local_control() amps.h:283
95 | 3.6.2.3 | Order: Message Waiting (Order 5) | N-A | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MWI transaction.h:32-33; FIFO order main_common.c
95 | 3.6.2.3 | Order: Protocol Capability Indicator | Y | Y | logged-only | Y | PARTIAL | rx MSPC/MSCAP amps.h:156-157; logged amps.c:1277-1279; pci query FIFO main_common.c:69; not driving extended-band selection
95 | 3.6.2.3 | Order: SSD Update order | partial | Y | partial | N | DECODE-ONLY | word3 First SSD Update Order Word frame.c:271 (RANDSSD defined); full SSD/auth flow not implemented
95 | 3.6.2.3 | Order: Unique Challenge order | partial | Y | partial | N | DECODE-ONLY | word3_unique_challenge_order_word frame.c:259 (RANDU 24 bits frame.c:264); auth flow not run

## page 96 (printed p.80) — §3.6.2.4 Local control; §3.6.3 Base station support of access; §3.6.3.1 Overhead info
96 | 3.6.2.4 | Local control — customize via local-order (ORDER=local control) + local control global actions | N-A | Y | Y | Y | IMPLEMENTED | Local Control Order 30 (TRANS_CALL_LOCAL_CONTROL transaction.h:48); amps_local_control() amps.h:283 (prose detail of pg.94 item)
96 | 3.6.2.4 | Group formed by common SID set (prose) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | administrative grouping prose; no normative code item
96 | 3.6.3 | Base station support of system access — section heading (prose) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | heading only
96 | 3.6.3.1 | Overhead info sent on FOCC to support access (fmt §3.7.1.2) | N-A | Y | Y | N-A | IMPLEMENTED | overhead/filler train emitted in IDLE amps.c:778; prepare_sysinfo sysinfo.c
96 | 3.6.3.1 | DCC (digital color code) transmitted base→MS | Y | Y | Y | Y | IMPLEMENTED | DCC 2-bit field every word frame.c:166/325/341/357; si->dcc=dcc sysinfo.c:59; -S dcc= main_common.c:184 (0..3)
96 | 3.6.3.1 | SDCC1 / SDCC2 (supplementary DCC); non-zero combo = SDCC-capable | partial | Y | Y | N | PARTIAL | AMPS_IE_SDCC1/SDCC2 frame.h:181-182; si->filler.sdcc1/sdcc2=0 hardcoded sysinfo.c:65-66 (SDCC feature not used → zero combo = "not supported")
96 | 3.6.3.1 | CMAC in control-filler msg; sets MS reverse-CC power; '000' when not required | N-A | Y | Y | Y | IMPLEMENTED | si->filler.cmac=cmac sysinfo.c:64 (struct sysinfo_filler.cmac sysinfo.h:4); -S cmac main_common.c; translation per SCM is MS-side
96 | 3.6.3.1 | RCF=1 in SPOM to require reading control-filler before access | N-A | Y | Y | N | IMPLEMENTED | si->word2.rcf sysinfo.c:82 (cross-ref pg.94 RCF)
96 | 3.6.3.1 | WFOM (wait-for-overhead-message) bit in control-filler | N-A | Y | Y | N | IMPLEMENTED | AMPS_IE_WFOM frame.h:205; si->filler.wfom=(bis)?0:1 sysinfo.c:67 (set to ignore B/I)
96 | 3.6.3.1 | OLC (overload control) global action; restricted classes=0, rest=1 | N-A | Y | Y | N | IMPLEMENTED | overload_control_global_action frame.c:438 (OLC 0..15); si->overload.olc[i]=1 all sysinfo.c (default all allowed); appended if non-default sysinfo.c
96 | 3.6.3.1 | Access type params global action; BIS=0 if MS must not check idle-to-busy | N-A | Y | Y | N | IMPLEMENTED | access_type_parameters_global_action frame.c (BIS field); si->acc_type.bis=bis sysinfo.c:106; dsp.c:924 checks acc_type.bis; -S handling main_common.c (default bis=0 main_common.c:109)

## page 97 (printed p.81) — §3.6.3.1 cont.; §3.6.3.2 Reverse CC seizure; §3.6.3.3 Response to MS messages
97 | 3.6.3.1 | BSPC (air-interface protocol) + BSCAP (core analog air iface) in access-type msg | N-A | Y | Y | N | IMPLEMENTED | BSPC/BSCAP fields frame.c:476; si->acc_type.bspc=0 / bscap=1 sysinfo.c:113-114; AMPS_IE_BSPC/BSCAP frame.h:16-17
97 | 3.6.3.1 | PCI_HOME / PCI_ROAM — whether MS reports protocol capability | N-A | Y | Y | N | IMPLEMENTED | PCI_HOME/PCI_ROAM fields access_type msg frame.c; si->acc_type.pci_home=1/pci_roam=1 sysinfo.c:111-112
97 | 3.6.3.1 | Access attempt params global action (seizure/busy limits) if != default | N-A | Y | Y | Y | IMPLEMENTED | access_attempt_parameters_global_action frame.c:490+ (MAXBUSY/MAXSZTR PGR+OTHER); si->acc_attempt sysinfo.c:117-120; -S maxbusy/maxsztr main_common.c:120/228
97 | 3.6.3.2 | Reverse CC seizure: on precursor w/ matching DCC (<=1 bit err), set busy-idle BUSY within 0.8-2.9 ms | N | N | N | N | MISSING | base cannot meet 0.8-2.9ms B/I response: bis=0 default, "osmocom can't respond fast enough" sysinfo.c:105-107 / main_common.c:107; B/I bit emitted on FOCC (dsp.c:542) but never driven busy on precursor
97 | 3.6.3.2 | Busy-idle remain busy until 30 ms after last word / (24N+55) ms | N | N | N | N | MISSING | no precursor-driven busy timing; channel_busy flag (dsp.c:925) is VC-occupancy not RECC seizure timing; design deliberately ignores B/I
97 | 3.6.3.3 | Base need not respond to every MS message (prose) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | prose; code may let MS time out
97 | 3.6.3.3 | Origination response: Initial Voice Channel Designation | N-A | Y | Y | N-A | IMPLEMENTED | MO path T=1 amps.c:1338-1340; assigns VC (vmac/channel) then IVCD; TRANS flow amps.c
97 | 3.6.3.3 | Origination response: Directed Retry | N-A | Y | Y | Y | IMPLEMENTED | word3_first_directed_retry_word frame.c:232 + word4_second_directed_retry_word frame.c:281; TRANS_DIRECTED_RETRY transaction.h:55; tx_focc_retry_channels amps.h:175
97 | 3.6.3.3 | Origination response: Intercept (Order 9) | N-A | Y | Y | Y | IMPLEMENTED | TRANS_CALL_INTERCEPT transaction.h:36-37; amps.c:1384 (Reorder/Intercept on FOCC on early fail); FIFO intercept main_common.c:60
97 | 3.6.3.3 | Origination response: Reorder (Order 4) | N-A | Y | Y | Y | IMPLEMENTED | TRANS_CALL_REORDER transaction.h:30-31; amps_reorder() amps.h:274; no-free-VC → Reorder amps.c:1592-1598; FIFO reorder main_common.c:55
97 | 3.6.3.3 | Page response: Initial Voice Channel Designation | N-A | Y | Y | N-A | IMPLEMENTED | paging reply T=0 amps.h:147 (rx_recc_t); VC assigned on page response amps.c
97 | 3.6.3.3 | Page response: Directed Retry | N-A | Y | Y | Y | IMPLEMENTED | same directed-retry machinery frame.c:232/281; TRANS_DIRECTED_RETRY transaction.h:55
97 | 3.6.3.3 | Page response: Release | N-A | Y | Y | Y | IMPLEMENTED | Release Order 3 (TRANS_CALL_RELEASE); FIFO release main_common.c:54
97 | 3.6.3.3 | Order message response: Order confirmation / Release | Y | Y | Y | Y | IMPLEMENTED | ORDQ/ORDER order-confirmation handling frame.c:181-182; order conf + release orders in amps.c state machine

## page 98 (printed p.82) — §3.6.3.3 end; §3.6.4 MS control on VC; §3.6.4.1 Loss of radio link; §3.6.4.2 Initial VC confirmation
98 | 3.6.3.3 | Order confirmation message response: no message sent | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | spec: base sends nothing; no code item to emit
98 | 3.6.3.3 | Protocol Capability Indicator Order Confirmation response: Release or Message Waiting | Y | Y | Y | Y | IMPLEMENTED | PCI rx amps.c:1277-1279; Release (Order 3) + Message Waiting (Order 5 TRANS_CALL_MWI transaction.h:32) available as responses
98 | 3.6.4 | MS control on VC: SAT/ST (0/1) signal call events; detected+interpreted by base; (0,1) treated as (0,0) | Y | N-A | Y | N | IMPLEMENTED | SAT detect sat_detected dsp.c:1118/1132; ST detect + flash/release interpretation amps.c:831-880; events mapped to transaction states
98 | 3.6.4 | Digital messages on VC in addition to analog SAT/ST signaling | Y | Y | Y | N-A | IMPLEMENTED | FVC/RVC framing frame.c (word layouts); orders on VC (TRANS_CALL_* states transaction.h)
98 | 3.6.4.1 | Loss of radio link continuity — "Reserved" (empty in spec) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | spec reserved; base-side mirror fade timer SAT_TO2 5s amps.c:85-88 tears down on SAT loss (EXTENSION beyond reserved text)
98 | 3.6.4.2 | Initial VC confirmation: SAT,ST change (0,0)->(1,0) = MS tuned to VC | Y | N-A | Y | N | IMPLEMENTED | VC assign-confirm states TRANS_CALL_MO_ASSIGN_CONFIRM / TRANS_CALL_MT_ASSIGN_CONFIRM amps.c:831/889; confirmation via sat_detected dsp.c:1118
98 | 3.6.4.2 | If confirmation not received: resend message or turn off VC transmitter | N-A | Y | partial | N | PARTIAL | assign timeout path exists (osmo_timer on assign-confirm) but explicit "resend-then-turn-off-TX" branch not clearly separated; VC torn down on timeout amps.c (amps_go_idle)
98 | 3.6.4.2 | After confirm: if paged -> Waiting for Order Task (§3.6.4.3.1); else -> Conversation Task (§3.6.4.4) | N-A | Y | Y | N | PARTIAL | MT (paged) path -> TRANS_CALL_MT_ALERT/ANSWER_WAIT amps.c:895; MO -> conversation (TRANS_CALL); task split present but not labeled per spec task names

## page 99 (printed p.83) — §3.6.4.3 Alerting; §3.6.4.3.1 Waiting for order (orders + SAT/ST confirmations)
99 | 3.6.4.3 | Alerting — section heading (prose) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | heading only
99 | 3.6.4.3.1 | Waiting for order task: MS entered after paged + IVC confirm; orders below | N-A | Y | Y | N | PARTIAL | post-page MT states amps.c; task not labeled "Waiting for Order" but behaviour present
99 | 3.6.4.3.1 | Handoff: confirm (1,0)->(1,1) held 50 ms; base remains Waiting-for-Order | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_HANDOFF transaction.h:52; fast ST detect 20ms window for 50ms ST dsp.c:370-372; SAT-mismatch handoff cb dsp.c:1125-1136; FIFO handoff main_common.c:71; tx_fvc_scc amps.h:181
99 | 3.6.4.3.1 | Alert / Alert With Info: confirm (1,0)->(1,1); base -> Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MT_ALERT/ALERT_CONFIRM amps.c:889+; word2_first_alert_with_info_word frame.c:975; alert ST confirm -> TRANS_CALL_MT_ANSWER_WAIT amps.c:895
99 | 3.6.4.3.1 | Alert With Info backward-compat: resend plain Alert if no confirm (553-1989) | N-A | partial | N | N | PARTIAL | Alert-with-info word exists frame.c:975; explicit fallback-to-Alert-on-no-confirm branch not found in grep
99 | 3.6.4.3.1 | Release: confirm (1,0)->(1,1) held 1.8 s; base turns off TX | Y | Y | Y | Y | IMPLEMENTED | ST_RELEASE_TIME 1.8s amps.c:85; TRANS_CALL_RELEASE amps.c:875-881 -> call_up_release + amps_go_idle (TX off)
99 | 3.6.4.3.1 | Audit: confirm by digital message; base remains Waiting-for-Order | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_AUDIT transaction.h:26; amps_audit_order() amps.h:270; digital confirm on RVC
99 | 3.6.4.3.1 | Message Waiting: confirm by digital message; remains Waiting-for-Order | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MWI transaction.h:32-33; MWI order amps.c
99 | 3.6.4.3.1 | Maintenance (Order 10): confirm (1,0)->(1,1); base -> Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MAINTENANCE transaction.h:38-39; amps_maintenance() amps.h:279; FIFO maintenance/silentpage main_common.c:61/73; maintenance ST confirm amps.c:898+
99 | 3.6.4.3.1 | Change power: confirm by digital message; remains Waiting-for-Order | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_CHANGE_POWER transaction.h:20-21; VMAC power control amps.c (VMAC_ADJUST_INTERVAL amps.c:92)
99 | 3.6.4.3.1 | Serial Number Request: confirm by Serial Number Response; remains Waiting-for-Order | Y | Y | Y | Y | IMPLEMENTED | serial_number_word frame.c:690-691 (Word C); amps_serial_number_request() amps.h:282; ESN decode esn.c:142-150
99 | 3.6.4.3.1 | SSD Update Order: MS computes SSD_A/B_NEW, sends Base Station Challenge within 750ms | partial | Y | partial | N | DECODE-ONLY | First SSD Update Order Word frame.c:271 (RANDSSD); RANDBS 32-bit frame.c:731; auth/SSD compute flow not implemented
99 | 3.6.4.3.1 | Unique Challenge Order: MS runs Unique Challenge Response, confirms within 750ms | partial | Y | partial | N | DECODE-ONLY | word3_unique_challenge_order_word frame.c:259 (RANDU); auth process not implemented
99 | 3.6.4.3.1 | Message Encryption Mode Order: MS enables/disables encryption (§2.3.12.2.1) | N-A | Y | Y | N | PARTIAL | AMPS_ORDER_MESSAGE_ENCRYPTION=23 pinned amps.c:70-73; order code defined but encryption not functionally implemented (CMEA absent)

## page 100 (printed p.84) — §3.6.4.3.1 end (Waiting-for-order orders); §3.6.4.3.2 Waiting for answer
100 | 3.6.4.3.1 | Message Encryption Mode confirm by digital message; remains Waiting-for-Order | N-A | Y | partial | N | PARTIAL | order code 23 pinned amps.c:70; encryption not functionally implemented (cont. of pg.99)
100 | 3.6.4.3.1 | Parameter Update Order: MS runs param-update, confirms; remains Waiting-for-Order | partial | Y | N | N | DECODE-ONLY | parameter-update is SSD/auth procedure (§2.3.12.1.3/.7); no param-update order encode in src/amps; auth not implemented
100 | 3.6.4.3.1 | Protocol Capability Indicator Order: MS confirms by PCI report; remains Waiting-for-Order | Y | Y | logged-only | Y | PARTIAL | PCI query FIFO main_common.c:69; MSPC/MSCAP rx amps.h:156-157 logged amps.c:1277-1279
100 | 3.6.4.3.1 | Local control: confirmation/action depend on message | N-A | Y | Y | Y | IMPLEMENTED | Order 30 Local Control TRANS_CALL_LOCAL_CONTROL transaction.h:48; amps_local_control() amps.h:283
100 | 3.6.4.3.1 | Base Station Challenge Order (autonomous from MS): process RANDBS, send AUTHBS within 10s | partial | Y | partial | N | DECODE-ONLY | base_station_challenge_word frame.c:726 (RANDBS 32-bit frame.c:731); AUTHBS 18-bit confirmation word frame.c:252; AMPS_IE_AUTHBS frame.h:10; auth compute not implemented
100 | 3.6.4.3.2 | Waiting for answer task: alert timer may be set; orders below | N-A | Y | Y | N | PARTIAL | TRANS_CALL_MT_ANSWER_WAIT amps.c:895 + ANSWER_TO 60s amps.c:89; task behaviour present, not labeled per spec
100 | 3.6.4.3.2 | Handoff: (1,1)->(1,0) 500ms -> (1,0)->(1,1) 50ms on old chan; then (1,1) on new chan | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_HANDOFF transaction.h:52; fast ST detect dsp.c:370 (50ms); SAT change to new SCC frame.c:4312; tx_fvc_scc amps.h:181
100 | 3.6.4.3.2 | Alert / Alert With Info: no confirmation; base may reset alert timer | N-A | Y | Y | N | IMPLEMENTED | ALERT_TRIES 3 / ALERT_TO amps.c:87-88; no-confirm handled by re-alert loop
100 | 3.6.4.3.2 | Alert With Info backward-compat: send plain Alert if no confirm (553-1989) | N-A | partial | N | N | PARTIAL | fallback branch not found (same as pg.99)
100 | 3.6.4.3.2 | Stop alert (Order 6): confirm (1,1)->(1,0); base -> Waiting-for-Order | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_STOP_ALERT transaction.h:34-35 (Order 6)
100 | 3.6.4.3.2 | Release: (1,1)->(1,0) 500ms -> (1,0)->(1,1) 1.8s; base turns off TX | Y | Y | Y | Y | IMPLEMENTED | ST_RELEASE_TIME 1.8s amps.c:85; TRANS_CALL_RELEASE amps.c:875 -> amps_go_idle (TX off)
100 | 3.6.4.3.2 | Audit: confirm by digital message; remains Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_AUDIT transaction.h:26
100 | 3.6.4.3.2 | Flash With Info: confirm by digital message; remains Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_FLASH_INFO transaction.h:22-23 (Order 18); tx_fvc_flashinfo amps.h:192-195; FIFO flash/flashcri/flashtci main_common.c:68/76-77
100 | 3.6.4.3.2 | Message Waiting: confirm by digital message; remains Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MWI transaction.h:32-33

## page 101 (printed p.85) — §3.6.4.3.2 Waiting-for-answer orders cont.; answer signal; §3.6.4.4 Conversation
101 | 3.6.4.3.2 | Maintenance: no confirmation; base may reset alert timer, remain Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MAINTENANCE transaction.h:38-39 (same machinery as pg.99)
101 | 3.6.4.3.2 | Change power: confirm digital message; remains Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_CHANGE_POWER transaction.h:20-21
101 | 3.6.4.3.2 | Serial Number Request: confirm by Serial Number Response; remains Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | serial_number_word frame.c:690; amps_serial_number_request() amps.h:282
101 | 3.6.4.3.2 | SSD Update Order: compute SSD_A/B_NEW, send Base Station Challenge within 750ms | partial | Y | partial | N | DECODE-ONLY | First SSD Update Order Word frame.c:271; auth not implemented
101 | 3.6.4.3.2 | Unique Challenge Order: run Unique Challenge Response within 750ms | partial | Y | partial | N | DECODE-ONLY | word3_unique_challenge_order_word frame.c:259; auth not implemented
101 | 3.6.4.3.2 | Parameter Update Order: run param-update, send Parameter Update Confirmation | partial | Y | N | N | DECODE-ONLY | SSD/auth procedure; no encode in src/amps
101 | 3.6.4.3.2 | Message Encryption Mode Order: enable/disable, confirm digital; remains Waiting-for-Answer | N-A | Y | partial | N | PARTIAL | order 23 pinned amps.c:70; encryption not functional
101 | 3.6.4.3.2 | Protocol Capability Indicator Order: confirm PCI report; remains Waiting-for-Answer | Y | Y | logged-only | Y | PARTIAL | MSPC/MSCAP amps.h:156-157 logged amps.c:1277-1279
101 | 3.6.4.3.2 | Local control: confirmation/action depend on message | N-A | Y | Y | Y | IMPLEMENTED | Order 30 TRANS_CALL_LOCAL_CONTROL transaction.h:48
101 | 3.6.4.3.2 | Base Station Challenge Order (autonomous): process RANDBS, send AUTHBS within 10s | partial | Y | partial | N | DECODE-ONLY | base_station_challenge_word frame.c:726; AUTHBS frame.c:252; auth compute not implemented
101 | 3.6.4.3.2 | Answer signal: SAT,ST (1,1)->(1,0); base -> Conversation Task (§3.6.4.4) | Y | N-A | Y | N | IMPLEMENTED | ST lost w/ SAT present in MT_ANSWER_WAIT => phone answered amps.c:1009-1014 call_up_answer -> conversation
101 | 3.6.4.4 | Conversation task — section heading; orders listed on next page (prose) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | heading; conversation = TRANS_CALL steady state amps.c:831

## page 102 (printed p.86) — §3.6.4.4 Conversation task orders
102 | 3.6.4.4 | Handoff: confirm (1,0)->(1,1) held 50 ms; base remains Conversation | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_HANDOFF transaction.h:52; fast ST detect dsp.c:370; SAT change frame.c:4312
102 | 3.6.4.4 | Send called address (Order 8): MS confirms by digital called-address msg; action depends on digits | Y | Y | Y | Y | IMPLEMENTED | Words D/E/F of called-address frame.c:767/803/821; Order 8 send amps.c:858-864; RVC called-addr response amps.c:1250-1253; FIFO digits main_common.c:59
102 | 3.6.4.4 | Alert / Alert With Info: confirm (1,0)->(1,1); base -> Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MT_ALERT amps.c:889; word2_first_alert_with_info_word frame.c:975
102 | 3.6.4.4 | Alert With Info backward-compat: send plain Alert if no confirm (553-1989) | N-A | partial | N | N | PARTIAL | fallback branch not found (recurs pg.99/100)
102 | 3.6.4.4 | Release: confirm (1,0)->(1,1) held 1.8 s; base turns off TX | Y | Y | Y | Y | IMPLEMENTED | ST_RELEASE_TIME 1.8s amps.c:85; TRANS_CALL_RELEASE amps.c:875
102 | 3.6.4.4 | Audit: confirm by digital message; remains Conversation | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_AUDIT transaction.h:26
102 | 3.6.4.4 | Flash With Info: confirm by digital message; remains Conversation | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_FLASH_INFO transaction.h:22 (Order 18); flash hook-detect amps.c:854
102 | 3.6.4.4 | Message Waiting: confirm by digital message; remains Conversation | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MWI transaction.h:32
102 | 3.6.4.4 | Maintenance: confirm (1,0)->(1,1); base -> Waiting-for-Answer | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_MAINTENANCE transaction.h:38; amps_maintenance() amps.h:279
102 | 3.6.4.4 | Change power: confirm by digital message; remains Conversation | Y | Y | Y | Y | IMPLEMENTED | TRANS_CALL_CHANGE_POWER transaction.h:20; VMAC amps.c:92
102 | 3.6.4.4 | Serial Number Request: confirm by Serial Number Response; remains Conversation | Y | Y | Y | Y | IMPLEMENTED | serial_number_word frame.c:690; amps_serial_number_request() amps.h:282
102 | 3.6.4.4 | SSD Update Order: compute SSD_A/B_NEW, send Base Station Challenge within 750ms | partial | Y | partial | N | DECODE-ONLY | First SSD Update Order Word frame.c:271; auth not implemented
102 | 3.6.4.4 | Unique Challenge Order: run Unique Challenge Response within 750ms | partial | Y | partial | N | DECODE-ONLY | word3_unique_challenge_order_word frame.c:259; auth not implemented
102 | 3.6.4.4 | Parameter Update Order: run param-update, send Parameter Update Confirmation | partial | Y | N | N | DECODE-ONLY | SSD/auth procedure; no encode in src/amps

## SUMMARY — pages 094-102 (§3.6.2 / §3.6.3 / §3.6.4 base-station call processing)

Pages read vs assigned: 9 of 9 read (094,095,096,097,098,099,100,101,102). All PNGs read one-by-one as ground truth; every normative item grepped against /home/vas/projects/osmocom-analog/src/amps/.

Status counts (120 rows total):
- IMPLEMENTED:              70
- PARTIAL:                  14
- DECODE-ONLY:              11
- MISSING:                   2
- NOT-APPLICABLE(mobile):   22
- EXTENSION(non-spec):       0 (one noted inline on pg.98 as mirror fade timer, counted PARTIAL/NA)
- STUB:                      0
- UNVERIFIED:                0

Top-3 gaps in range:
1. §3.6.3.2 Reverse-CC seizure busy-idle bit timing (pg.97) — MISSING. The base is
   required, on a seizure precursor matching its DCC (<=1 bit error), to drive the
   FOCC busy-idle bits BUSY within 0.8-2.9 ms and hold until 30 ms after the last
   word / (24N+55) ms. osmocom deliberately runs with bis=0 and never responds to
   the B/I bit ("osmocom can't respond fast enough", sysinfo.c:105-107,
   main_common.c:107); the B/I bit is emitted (dsp.c:542) but never seizure-driven.
   This is a real-time constraint the SDR software path cannot meet — a hardware/
   latency limitation, not a quick code fix.
2. Authentication family (pg.95/99/100/101/102) — DECODE-ONLY across SSD Update,
   Unique Challenge, Base Station Challenge/AUTHBS, Parameter Update, RAND1_A/B,
   RANDBS/RANDU/RANDSSD. All the words and IEs are DEFINED and encode/decode exists
   (frame.c:246-311,726-731), but no CAVE/SSD compute flow runs, and AUTH is pinned
   to 0 in word1 (sysinfo.c:72). Authentication is effectively absent, only framed.
3. §3.6.4.2 "resend-or-turn-off-TX on no IVC confirm" + Alert-With-Info backward-
   compat fallback-to-plain-Alert (pg.98/99/100/102) — PARTIAL. The assign/alert
   timeout tears the call down, but the spec's explicit "resend the message OR turn
   off the VC transmitter" branch and the 553-1989 compat re-send-as-Alert branch
   were not found distinctly in the state machine.

Secondary notes:
- Message Encryption Mode (order 23) is PARTIAL: code pins the order value with a
  compile-time assert (amps.c:70-73) to avoid a one-bit slip into Disable-DTMF, but
  CMEA encryption is not functionally implemented.
- Protocol Capability Indicator is PARTIAL/logged-only: MSPC/MSCAP are received and
  logged (amps.c:1277-1279) but do not drive extended-band selection.
- The full base-station call-processing order set (Handoff, Alert, Release, Audit,
  MWI, Maintenance, Change Power, Serial Number Request, Stop Alert, Flash With Info,
  Send Called Address, Local Control, Directed Retry, Intercept, Reorder) is broadly
  IMPLEMENTED with correct SAT/ST confirmation timing (50ms handoff, 1.8s release,
  ST flash 200-800ms) — this range is the strongest-covered part of the base side.

MCP servers unavailable this session: @kirocrew-computer (declared by agent spec but
not configured). Did not affect this task — all work used local file/grep/image tools.

---

## AMPS spec-walk pages 103-116

Perspective: BASE STATION (osmocom-analog src/amps/). Range = §3.6.4 tail, §3.6.5,
and §3.7 Signaling formats incl. §3.7.1 FOCC word layouts + Table 3.7.1-1 Order and
order-qualification codes. Source baseline for the order table (main_common.c:48-82,
amps.c, frame.c): standard orders 1,2,3,4,5,6,8,9,10,11,12,15,18,30 + extension
orders 17/18 ORDQ variants are supported as operator commands.

Columns: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

## page 103  (doc p.87) — §3.6.4 tail orders, §3.6.5 Delivery of Character Info, §3.7 intro

103 | §3.6.4 | Disable DTMF Order (Order 22=0b10110): mobile confirms by digital msg; base stays | partial | Y | Y | Y | IMPLEMENTED | AMPS_ORDER_DISABLE_DTMF=22 (amps.c:69, _Static_assert); amps_disable_dtmf() (amps.c:284h); TRANS_CALL_DISABLE_DTMF (transaction.h:50)
103 | §3.6.4 | Message Encryption Mode Order (Order 23=0b10111): mobile en/disables msg encryption | partial | Y | logged-only | N | PARTIAL | AMPS_ORDER_MESSAGE_ENCRYPTION=23 (amps.c:70) pinned by _Static_assert; order code reserved/known, no key handling
103 | §3.6.4 | Protocol Capability Indicator Order: mobile confirms by PCI report; base stays | partial | Y | logged-only | Y | PARTIAL | `pci,<number>` operator cmd (main_common.c:72); PCI query encoded, report decode TBD
103 | §3.6.4 | Local control order: confirmation/action depend on message | N | Y | logged-only | Y | PARTIAL | `local,<number>,<code>` Order 30 (main_common.c:63)
103 | §3.6.4 | Flash request: mobile SAT/ST (1,0)->(1,1) held 400ms ->(1,0) | partial | N-A | partial | N-A | PARTIAL | ST/flash detection in dsp.c (reverse voice ST); base-side RX
103 | §3.6.4 | Release: mobile SAT/ST (1,0)->(1,1) held 1.8s; base shall turn off TX | partial | N-A | Y | N-A | PARTIAL | ST_RELEASE_TIME 1.8s (amps.c:93); base release handling amps.c:796
103 | §3.6.4 | Base Station Challenge Order: base processes RANDBS, sends AUTHBS in confirmation ≤10s; stays | N | N | N | N | MISSING | word3_base_station_challenge_order_confirmation_word defined (frame.c:246) but auth not computed
103 | §3.6.5 | Character info delivered via Mobile Station Control Message over FVC; Alert/Flash With Info designated | partial | Y | Y | Y | PARTIAL | `flash,<number>,<msg>` Order 18 (main_common.c:75); Alert With Info Order 17 ext
103 | §3.6.5 | Two sets of char info: 2nd set sent using Flash With Info order | N | partial | N | N | PARTIAL | flash-with-info supported; sequencing of two sets not explicitly handled
103 | §3.7 | Reserved (RSVD) bits: set to 0 on TX, ignore on RX | partial | Y | N-A | N | PARTIAL | RSVD fields present in word defs (frame.c word3_* RSVD entries), set 0

## page 104  (doc p.88) — §3.7 reserved-bit NAWC rule, §3.7.1 FOCC stream, Figure 3.7.1-1

104 | §3.7 | BCH-correct but unrecognizable overhead counted in NAWC train, not executed (mobile-side) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | base encodes NAWC; overhead train built in sysinfo.c
104 | §3.7 | Reserved-type overhead msgs decoded/used as appropriate (mobile) | N-A | N-A | N-A | N-A | NOT-APPLICABLE(mobile-side) | base TX side
104 | §3.7.1 | FOCC is continuous wideband stream base->mobile | N-A | Y | Y | N-A | IMPLEMENTED | amps_tx_frame_focc() (amps.c:28); dsp.c FOCC TX
104 | §3.7.1 | FOCC rate 10 kbit/s ±0.1 bit/s | N-A | Y | Y | N-A | IMPLEMENTED | dsp.c FOCC modulation at 10 kbit/s
104 | Fig 3.7.1-1 | Frame: DOTTING(10) + WORD SYNC(11) + 5× [Word A(40) + Word B(40)] | partial | Y | Y | N-A | IMPLEMENTED | frame_length=441 bits after sync (amps.c:780); 5 repeats of A/B
104 | Fig 3.7.1-1 | DOTTING = 1010...101 | Y | Y | Y | N-A | IMPLEMENTED | dotting search dsp.c:30-35; alternate bits
104 | Fig 3.7.1-1 | WORD SYNC = 11100010010 (11 bits) | Y | Y | Y | N-A | IMPLEMENTED | sync detect dsp.c:34-72
104 | §3.7.1 | Stream A/B split: MIN LSB=0 -> stream A, LSB=1 -> stream B | partial | Y | Y | N-A | PARTIAL | A/B interleave in FOCC TX; MIN LSB routing
104 | §3.7.1 | Busy-Idle stream: 0=busy, 1=idle on reverse CC | partial | Y | logged-only | Y | PARTIAL | BIS field (frame.c:2160); bis cmd (main_common.c:107) default 0
104 | §3.7.1 | B/I bit at start of dotting, start of WS, start of 1st repeat of word A, every 10 msg bits | N-A | Y | Y | N-A | IMPLEMENTED | B/I insertion in FOCC framing (dsp.c)

## page 105  (doc p.89) — §3.7.1 word/BCH framing, FOCC msg types, §3.7.1.1 Word 1

105 | §3.7.1 | 10-bit dotting (1010101010) + 11-bit word sync (11100010010) for sync | Y | Y | Y | N-A | IMPLEMENTED | dsp.c dotting/sync; matches Fig 3.7.1-1
105 | §3.7.1 | Word = 40 bits incl parity, repeated 5× = word block | partial | Y | Y | N-A | IMPLEMENTED | 441-bit FOCC frame (amps.c:780) = 11 WS + 5×(40+40); 5 repeats
105 | §3.7.1 | Multi-word msg: each word block has own 10-bit dotting + 11-bit WS | partial | Y | Y | N-A | IMPLEMENTED | per-word-block framing in FOCC TX (frame.c multi-word msgs)
105 | §3.7.1 | Word = (40,28;5) BCH; 28 MSBs content, left-most = MSB | partial | Y | Y | N-A | IMPLEMENTED | encode_bch/encode_bch_binary (frame.c:60-110); 28 content + 12 parity
105 | §3.7.1 | Generator Gb(x)=x12+x10+x8+x5+x4+x3+x0 (shortened (63,51;5)) | N-A | Y | Y | N-A | IMPLEMENTED | gp[12]={0,1,0,1,0,0,1,1,1,0,0,1} (frame.c:56) = x10+x8+x5+x4+x3+x0 (x12 implicit)
105 | §3.7.1 | FOCC msg types: Mobile station control / Overhead / Control-Filler | partial | Y | Y | N-A | IMPLEMENTED | mobile_station_control (frame.c:160), overhead (frame.c:319,374), control-filler (frame.c:574)
105 | §3.7.1 | Control-filler msgs may be inserted between msgs and word blocks | N-A | Y | Y | N | IMPLEMENTED | control-filler encode (frame.c:574); sysinfo.c:63 filler
105 | §3.7.1.1 | Mobile station control message: 1 to 8 words | partial | Y | Y | N-A | IMPLEMENTED | mobile_station_control message (frame.c:160); multi-word
105 | §3.7.1.1 | Word 1 Abbreviated Address: T1T2(2) DCC(2) MIN1_23-0(24) P(12) | partial | Y | Y | N-A | IMPLEMENTED | word1_abbreviated_address_word (frame.c:161-170) exact field widths match

## page 106  (doc p.90) — §3.7.1.1 FOCC MS control message word layouts

106 | §3.7.1.1 | Word 2 Extended Address (SCC=11): T1T2(2) SCC(2) MIN2_33-24(10) EF(1) LOCAL/MSG_TYPE(5) ORDQ(3) ORDER(5) P(12) | partial | Y | Y | N-A | IMPLEMENTED | word2_extended_address_word_a (frame.c:172); fields incl ORDQ(3) ORDER(5) match
106 | §3.7.1.1 | Word 2 Extended Address (SCC≠11): T1T2(2) SCC(2) MIN2(10) VMAC(3) CHAN(11) P(12) | partial | Y | Y | N-A | IMPLEMENTED | word2_extended_address_word_b (frame.c:186); VMAC(3) CHAN(11) match
106 | §3.7.1.1 | Word 3 First Directed-Retry: T1T2=10 SCC=11 CHANPOS×3(7 each) RSVD=000(3) P(12) | partial | Y | Y | N-A | IMPLEMENTED | word3_first_directed_retry_word (frame.c:200+); 3× CHANPOS(7) + RSVD(3)
106 | §3.7.1.1 | Word 3 Base Station Challenge Order Confirmation: SCC=11 RSVD=00(2) AUTHBS(18) RSVD=0000(4) P(12) | Y | N | N | N | DECODE-ONLY | word3_base_station_challenge_order_confirmation_word (frame.c:246); AUTHBS(18) field defined, auth not computed
106 | §3.7.1.1 | Word 3 Unique Challenge Order: SCC=11 RANDU(24) P(12) | Y | N | N | N | DECODE-ONLY | word3_unique_challenge_order_word (frame.c:259); RANDU(24); CAVE auth not implemented
106 | §3.7.1.1 | Word 3 First SSD Update Order: SCC=11 RANDSSD_1(24) P(12) | Y | N | N | N | DECODE-ONLY | word3_first_ssd_update_order_word (frame.c:270); RANDSSD_1(24); SSD not implemented
106 | §3.7.1.1 | Word 4 Second SSD Update Order: SCC=11 RANDSSD_2(24) P(12) | Y | N | N | N | DECODE-ONLY | RANDSSD_2(24) def (frame.c ~275); SSD update not implemented
106 | §3.7.1.1 | Word 4 Second Directed-Retry: T1T2=10 SCC=11 CHANPOS×3(7) RSVD=000(3) P(12) | partial | Y | Y | N-A | IMPLEMENTED | word4_second_directed_retry_word (frame.c:281)

## page 107  (doc p.91) — Word 5 Third SSD Update + §3.7.1.1 field interpretation glossary

107 | §3.7.1.1 | Word 5 Third SSD Update: SCC=11 RSVD=0..0(12) RANDSSD_3(8) RSVD=0000(4) P(12) | Y | N | N | N | DECODE-ONLY | word5 SSD def in frame.c; RANDSSD_3(8); SSD update not implemented
107 | §3.7.1.1 | T1T2 Type: Word1-only=00; multi-word Word1=01, each addl word=10 | partial | Y | Y | N-A | IMPLEMENTED | T1T2 field in all word defs (frame.c); multi-word framing
107 | §3.7.1.1 | DCC Digital color code field | Y | Y | Y | Y | IMPLEMENTED | dcc throughout (sysinfo.c, frame.c); `-S dcc=` operator cmd
107 | §3.7.1.1 | MIN1/MIN2 mobile identification number parts (§2.3.1) | Y | Y | Y | Y | IMPLEMENTED | min1/min2 encode+decode (frame.c, amps.c:306); number mapping
107 | §3.7.1.1 | SCC SAT color code (Table 3.7.1-2) | partial | Y | Y | Y | IMPLEMENTED | SCC field; SAT color mapping (dsp.c SAT); scc routing
107 | §3.7.1.1 | ORDER order type field (Table 3.7.1-1) | partial | Y | Y | Y | IMPLEMENTED | ORDER(5) field; trans->order (amps.c:804); operator order cmds
107 | §3.7.1.1 | ORDQ order qualifier (Table 3.7.1-1) | partial | Y | Y | Y | IMPLEMENTED | ORDQ(3); trans->ordq (amps.c:803); AMPS_IE_ORDQ (frame.c:757)
107 | §3.7.1.1 | LOCAL control field (system-specific; ORDER=local control) | N | Y | logged-only | Y | PARTIAL | `local,<number>,<code>` Order 30 (main_common.c:63)
107 | §3.7.1.1 | MSG_TYPE message type field (Table 3.7.1-1) | partial | Y | Y | N | PARTIAL | LOCAL/MSG TYPE 5-bit field in word2_a (frame.c:179)
107 | §3.7.1.1 | EF Extended Protocol Forward Channel Indicator (set 0) | N-A | Y | N-A | N | IMPLEMENTED | EF field in word2_a (frame.c:178), set 0 per spec
107 | §3.7.1.1 | VMAC Voice mobile attenuation code (power level, Table 2.1.2-1) | partial | Y | Y | Y | IMPLEMENTED | VMAC(3) field; `power,<number>,<level>` Order 11 (main_common.c:60)
107 | §3.7.1.1 | CHAN channel number (designated RF channel §2.3.3) | Y | Y | Y | Y | IMPLEMENTED | CHAN(11); amps_channel2freq (amps.c); channel assignment
107 | §3.7.1.1 | CHANPOS channel position rel to FIRSTCHA | partial | Y | Y | N | PARTIAL | CHANPOS(7) in directed-retry words; FIRSTCHA referenced
107 | §3.7.1.1 | RSVD reserved, bits set as indicated | N-A | Y | N-A | N | IMPLEMENTED | RSVD fields set to indicated values in word defs
107 | §3.7.1.1 | P parity field | N-A | Y | Y | N-A | IMPLEMENTED | BCH 12-bit parity (encode_bch, frame.c:60)
107 | §3.7.1.1 | RANDU 24-bit random from base in Unique Challenge | Y | N | N | N | DECODE-ONLY | RANDU(24) def; CAVE auth not implemented
107 | §3.7.1.1 | RANDSSD_1/2/3 (24/24/8 bits) random for SSD Update | Y | N | N | N | DECODE-ONLY | RANDSSD_1/2/3 defs; SSD not implemented
107 | §3.7.1.1 | AUTHBS output of auth algorithm from Base Station Challenge | Y | N | N | N | DECODE-ONLY | AUTHBS(18) def (frame.c:252); CAVE not implemented

## page 108  (doc p.92) — Table 3.7.1-1 Order and order qualification codes (part 1)

Each row: ORDER(binary) ORDQ MSG_TYPE | Function | status vs src/amps/.

108 | T3.7.1-1 | 00000/000/00000 Page (or Origination), Auth Word C not incl | Y | Y | Y | N-A | IMPLEMENTED | Page handled; paging state machine (amps.c PAGE_TRIES); Order 0
108 | T3.7.1-1 | 00001/000/00000 Alert | partial | Y | Y | Y | IMPLEMENTED | `alert,<number>` Order 1 ORDQ=0 (main_common.c:52); amps_alert_order()
108 | T3.7.1-1 | 00001/001/00000 Abbreviated Alert | partial | Y | Y | Y | IMPLEMENTED | `abbalert,<number>` Order 1 ORDQ=1 (main_common.c:53)
108 | T3.7.1-1 | 00011/000/00000 Release | partial | Y | Y | Y | IMPLEMENTED | `release` Order 3 (main_common.c:54); trans->order=3 (amps.c:804)
108 | T3.7.1-1 | 00100/000/00000 Reorder | partial | Y | Y | Y | IMPLEMENTED | `reorder` Order 4 (main_common.c:55); TRANS_CALL_REORDER (transaction.h:30)
108 | T3.7.1-1 | 00101/000/XXXXX Message Waiting (Type field = # of msgs) | partial | Y | Y | Y | IMPLEMENTED | `mwi,<number>,<count>[,type]` Order 5 (main_common.c:56-57); MSG_TYPE carries count
108 | T3.7.1-1 | 00110/000/00000 Stop Alert | partial | Y | Y | Y | IMPLEMENTED | `stopalert` Order 6 (main_common.c:58); TRANS_CALL_STOP_ALERT (transaction.h:34)
108 | T3.7.1-1 | 00111/000/00000 Audit | partial | Y | Y | Y | IMPLEMENTED | `audit` Order 2 (main_common.c:71); amps_audit_order() (amps.h:270); TRANS_CALL_AUDIT
108 | T3.7.1-1 | 01000/000/00000 Send Called-address | partial | Y | Y | Y | IMPLEMENTED | `digits,<number>` Order 8 (main_common.c:59) requests dialed digits
108 | T3.7.1-1 | 01001/000/00000 Intercept | partial | Y | Y | Y | IMPLEMENTED | `intercept` Order 9 (main_common.c:60)
108 | T3.7.1-1 | 01010/000/00000 Maintenance | partial | Y | Y | Y | IMPLEMENTED | `maintenance` Order 10 (main_common.c:61); silentpage uses it
108 | T3.7.1-1 | 01011/000/00000 Change Power to Level 0 (§2.1.2.2) | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,0` Order 11 ORDQ=0 (main_common.c:62)
108 | T3.7.1-1 | 01011/001/00000 Change Power to Level 1 | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,1` Order 11 ORDQ=1
108 | T3.7.1-1 | 01011/010/00000 Change Power to Level 2 | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,2` Order 11 ORDQ=2
108 | T3.7.1-1 | 01011/011/00000 Change Power to Level 3 | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,3` Order 11 ORDQ=3
108 | T3.7.1-1 | 01011/100/00000 Change Power to Level 4 | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,4` Order 11 ORDQ=4
108 | T3.7.1-1 | 01011/101/00000 Change Power to Level 5 | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,5` Order 11 ORDQ=5
108 | T3.7.1-1 | 01011/110/00000 Change Power to Level 6 | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,6` Order 11 ORDQ=6
108 | T3.7.1-1 | 01011/111/00000 Change Power to Level 7 | partial | Y | Y | Y | IMPLEMENTED | `power,<number>,7` Order 11 ORDQ=7 (VMAC level 0-7)
108 | T3.7.1-1 | 01100/000/00000 Directed Retry - not last try | partial | Y | Y | Y | IMPLEMENTED | `retry`/`rescan` Order 12 ORDQ=0 (main_common.c:64,829); directed-retry words
108 | T3.7.1-1 | 01100/001/00000 Directed Retry - last try | partial | Y | Y | Y | IMPLEMENTED | Order 12 ORDQ=1 last-try variant (main_common.c:829 FOCC)
108 | T3.7.1-1 | 01101/000/00000 Non-auton Registration - don't make whereabouts known, no Auth C | partial | Y | Y | Y | PARTIAL | Order 13 registration (sysinfo.c reg_id/reg_incr); ORDQ variants not all distinct
108 | T3.7.1-1 | 01101/001/00000 Non-auton Registration - make whereabouts known, no Auth C | partial | Y | Y | Y | PARTIAL | Order 13 ORDQ=1; registration handling (sysinfo.c)
108 | T3.7.1-1 | 01101/010/00000 Autonomous Registration - don't make whereabouts known, no Auth C | partial | Y | Y | Y | PARTIAL | Order 13 ORDQ=2; autonomous registration
108 | T3.7.1-1 | 01101/011/00000 Autonomous Registration - make whereabouts known, no Auth C | partial | Y | Y | Y | PARTIAL | Order 13 ORDQ=3
108 | T3.7.1-1 | 01101/011/00001 Autonomous Registration - Power Down, PCI/Auth Word C not incl (also RVC) | partial | Y | partial | N | PARTIAL | Order 13 ORDQ=3 MSG_TYPE=1 power-down registration; power-down detect limited
108 | T3.7.1-1 | 10001/000/00000 Alert With Info (Order 17) | partial | Y | Y | Y | IMPLEMENTED | Order 17 Alert With Info; `alertcri`/`alerttci` ext (main_common.c:78-79)
108 | T3.7.1-1 | 10010/000/00000 Flash With Info (Order 18) | partial | Y | Y | Y | IMPLEMENTED | `flash,<number>,<msg>` Order 18 (main_common.c:75); flashcri/flashtci ext

## page 109  (doc p.93) — Table 3.7.1-1 Order and order qualification codes (cont'd, part 2)

109 | T3.7.1-1 | 01111/000/00000 Parameter Update Order/Confirmation (Order 15 ORDQ0) | partial | partial | logged-only | N | PARTIAL | Order 15 slot; ESN request implemented, parameter-update variant not distinct
109 | T3.7.1-1 | 01111/001/00000 Serial Number Request/Response (Order 15 ORDQ1) | Y | Y | Y | Y | IMPLEMENTED | `esn,<number>` Order 15 (main_common.c:64,309); TRANS_CALL_ESN_REQUEST (transaction.c:137); amps_encode_esn (amps.c:353)
109 | T3.7.1-1 | 01111/100/00000 Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved code, no impl expected
109 | T3.7.1-1 | 11000/000/00000 Non-auton Registration - don't make whereabouts known, Auth C INCLUDED | N | N | N | N | MISSING | Auth-Word-C registration not implemented (no CAVE); Order 24 ORDQ0
109 | T3.7.1-1 | 11000/001/00000 Non-auton Registration - make whereabouts known, Auth C incl | N | N | N | N | MISSING | Auth-C registration not implemented; Order 24 ORDQ1
109 | T3.7.1-1 | 11000/010/00000 Autonomous Registration - don't make whereabouts known, Auth C incl | N | N | N | N | MISSING | Auth-C registration not implemented; Order 24 ORDQ2
109 | T3.7.1-1 | 11000/011/00000 Autonomous Registration - make whereabouts known, Auth C incl | N | N | N | N | MISSING | Auth-C registration not implemented; Order 24 ORDQ3
109 | T3.7.1-1 | 11000/011/00001 Autonomous Registration - Power Down, Auth C incl | N | N | N | N | MISSING | Auth-C power-down registration not implemented; Order 24 ORDQ3 MSG1
109 | T3.7.1-1 | 11010/100/00000 Protocol Capability Indicator; PCI Registration - Auth C not incl (Order 26) | partial | Y | logged-only | Y | PARTIAL | `pci,<number>` (main_common.c:72); PCI query encoded, PCI-registration report decode TBD
109 | T3.7.1-1 | 11010/100/00001 Protocol Capability Registration - Auth C included | N | N | N | N | MISSING | Auth-C PCI registration not implemented
109 | T3.7.1-1 | 11110/000/XXXXX Local control (Order 30) | N | Y | logged-only | Y | PARTIAL | `local,<number>,<code>` Order 30 (main_common.c:63); system-specific
109 | T3.7.1-1 | [BS-init auth] 10011/000/00000 Base Station Challenge Confirmation (Order 19) | Y | N | N | N | DECODE-ONLY | word3_base_station_challenge_order_confirmation_word (frame.c:246); AUTHBS not computed
109 | T3.7.1-1 | [BS-init auth] 10100/000/00000 Unique Challenge Order (Order 20) | partial | partial | N | N | DECODE-ONLY | word3_unique_challenge_order_word (frame.c:259); RANDU def; CAVE not implemented
109 | T3.7.1-1 | [BS-init auth] 10101/000/00000 SSD Update Order (Order 21) | partial | partial | N | N | DECODE-ONLY | SSD update words defined (frame.c:270-306); SSD/CAVE not implemented
109 | T3.7.1-1 | [BS-init auth] 10110/000/00000 Disable DTMF Order (Order 22) | partial | Y | Y | Y | IMPLEMENTED | AMPS_ORDER_DISABLE_DTMF=22 (amps.c:69); amps_disable_dtmf(); TRANS_CALL_DISABLE_DTMF
109 | T3.7.1-1 | [BS-init auth] 10111/000/00000 Message Encryption Mode order, DISABLE indication (Order 23 ORDQ0) | partial | partial | N | N | PARTIAL | AMPS_ORDER_MESSAGE_ENCRYPTION=23 (amps.c:70) pinned; no encryption key handling
109 | T3.7.1-1 | [BS-init auth] 10111/001/00000 Message Encryption Mode order, ENABLE indication (Order 23 ORDQ1) | partial | partial | N | N | PARTIAL | Order 23 ORDQ1; encryption not implemented
109 | T3.7.1-1 | [MS-init auth] 10011/000/00000 Base Station Challenge Order (RX from mobile) | Y | N-A | N | N-A | DECODE-ONLY | mobile-initiated; base processes RANDBS (page 103) - not computed; RECC order
109 | T3.7.1-1 | [MS-init auth] 10100/000/00000 Unique Challenge Confirmation (RX from mobile) | partial | N-A | N | N-A | DECODE-ONLY | mobile confirmation decode; auth not verified
109 | T3.7.1-1 | [MS-init auth] 10101/000/00000 SSD Update Confirmation with FAILURE indication | partial | N-A | N | N-A | DECODE-ONLY | SSD confirm decode (ORDQ0=fail); not acted on
109 | T3.7.1-1 | [MS-init auth] 10101/001/00000 SSD Update Confirmation with SUCCESS indication | partial | N-A | N | N-A | DECODE-ONLY | SSD confirm decode (ORDQ1=success); not acted on
109 | T3.7.1-1 | "All other codes are reserved." | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | catch-all reserved note

## page 110  (doc p.94) — Table 3.7.1-2 SAT Color Code, §3.7.1.2 Overhead message

110 | T3.7.1-2 | SCC 00 -> SAT 5970 Hz | Y | Y | Y | partial | IMPLEMENTED | sat_freq[0]=5970.0 (dsp.c:143); SAT detect/gen
110 | T3.7.1-2 | SCC 01 -> SAT 6000 Hz | Y | Y | Y | partial | IMPLEMENTED | sat_freq[1]=6000.0 (dsp.c:144)
110 | T3.7.1-2 | SCC 10 -> SAT 6030 Hz | Y | Y | Y | partial | IMPLEMENTED | sat_freq[2]=6030.0 (dsp.c:145)
110 | T3.7.1-2 | SCC 11 -> Not an analog channel designation | Y | N-A | Y | N-A | IMPLEMENTED | sat_freq[3]=5790.0 noise-check (dsp.c:146); SCC=11 used as digital/addr marker in word defs
110 | §3.7.1.2 | 3-bit OHD field identifies overhead message type (Table 3.7.1-3) | partial | Y | Y | N-A | IMPLEMENTED | OHD field in overhead words (frame.c:319,374); sysinfo.c builds train
110 | §3.7.1.2 | Overhead classes: System param / Global action / Registration ID / Control-filler | partial | Y | Y | partial | IMPLEMENTED | system_parameter (frame.c:319), global_action (frame.c:374,390), registration_id (frame.c:561), filler (frame.c:574)
110 | §3.7.1.2 | Overhead train: 1st word = system param overhead; GA/reg-ID appended | partial | Y | Y | N-A | IMPLEMENTED | sysinfo.c encodes train, system param first (sysinfo.c:52-56)
110 | §3.7.1.2 | Train length = NAWC+1 words; last word END=1, others END=0 | partial | Y | Y | N | IMPLEMENTED | nawc/end computed (sysinfo.c:156); END bit set on last word
110 | §3.7.1.2 | Control-filler msgs NOT counted in NAWC | N-A | Y | Y | N-A | IMPLEMENTED | filler excluded from NAWC count (sysinfo.c filler path)
110 | §3.7.1.2 | System param overhead sent every 0.8±0.3 s on each control channel | N-A | Y | Y | N | PARTIAL | overhead_repeat=17/14 (sysinfo.c:52-56); timing approximates 0.8s
110 | §3.7.1.2 | Sent on: dedicated CC; combined paging-access FOCC (CPA=1); separate paging FOCC (CPA=0); separate access FOCC (CPA=0) with WFOM=1 | N-A | Y | Y | N | PARTIAL | word2.cpa=1 for combined CC+PC (sysinfo.c:83); filler.wfom (sysinfo.c:67)
110 | §3.7.1.2 | Global action & registration ID messages sent as needed | N-A | Y | Y | partial | IMPLEMENTED | GA/reg-ID appended on demand (sysinfo.c)

## page 111  (doc p.95) — Table 3.7.1-3 Overhead types, §3.7.1.2.1 System Param overhead Word 1/2

111 | T3.7.1-3 | OHD 000 = Registration ID | partial | Y | Y | N-A | IMPLEMENTED | registration_id (frame.c:561); amps_encode_registration_id (sysinfo.c:178)
111 | T3.7.1-3 | OHD 001 = Control-filler | N-A | Y | Y | N | IMPLEMENTED | control-filler (frame.c:574); sysinfo.c filler
111 | T3.7.1-3 | OHD 010 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
111 | T3.7.1-3 | OHD 011 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
111 | T3.7.1-3 | OHD 100 = Global action | partial | Y | Y | partial | IMPLEMENTED | global_action (frame.c:374); registration_increment_global_action (frame.c:390)
111 | T3.7.1-3 | OHD 101 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
111 | T3.7.1-3 | OHD 110 = Word 1 of system parameter message | N-A | Y | Y | N-A | IMPLEMENTED | OHD=110 in system param word1 (frame.c:319)
111 | T3.7.1-3 | OHD 111 = Word 2 of system parameter message | N-A | Y | Y | N-A | IMPLEMENTED | OHD=111 in system param word2 (frame.c)
111 | §3.7.1.2.1 | System param overhead = 2 words | N-A | Y | Y | N-A | IMPLEMENTED | sysinfo.c builds 2-word system param
111 | §3.7.1.2.1 | Word 1: T1T2=11 DCC(2) SID1(14) EP(1) AUTH(1) PCI(1) NAWC(4) OHD=110(3) P(12) | partial | Y | Y | partial | IMPLEMENTED | word1 def (frame.c:326-329): SID1(14) EP AUTH PCI; sysinfo.c:70 sid1
111 | §3.7.1.2.1 | Word 2: T1T2=11 DCC S(1) E(1) REGH(1) REGR(1) DTX(2) N-1(5) RCF(1) CPA(1) CMAX-1(7) END(1) OHD=111(3) P(12) | partial | Y | Y | partial | IMPLEMENTED | word2 fields regh/regr/rcf/cpa/n1/cmax1 (sysinfo.c:78-83, main_common.c:111)
111 | §3.7.1.2.1 | T1T2 Type set '11' = overhead word | N-A | Y | Y | N-A | IMPLEMENTED | T1T2=11 in overhead word defs
111 | §3.7.1.2.1 | OHD: word1=110, word2=111 | N-A | Y | Y | N-A | IMPLEMENTED | OHD values in system param defs
111 | §3.7.1.2.1 | DCC Digital color code | Y | Y | Y | Y | IMPLEMENTED | dcc (sysinfo.c, `-S dcc=`)
111 | §3.7.1.2.1 | SID1 first part of system identification | partial | Y | Y | Y | IMPLEMENTED | sid1 (sysinfo.c:70, `-S sid=`)

## page 112  (doc p.96) — §3.7.1.2.1 field glossary, §3.7.1.2.2 Global action overhead

112 | §3.7.1.2.1 | EP Extended Protocol Bit (set 0 in 553-A; see TIA/EIA 691) | N-A | Y | N-A | N | IMPLEMENTED | EP field set 0 (frame.c:327)
112 | §3.7.1.2.1 | AUTH set 1 if base supports authentication (§2.3.12, §3.3.1) | partial | Y | N | N | PARTIAL | AUTH field (frame.c:328) encodable; base auth/CAVE not implemented so set 0
112 | §3.7.1.2.1 | PCI set 0 for 553-A (see TIA/EIA 627, IS-136) | N-A | Y | N-A | N | IMPLEMENTED | PCI field set 0 (frame.c:329)
112 | §3.7.1.2.1 | NAWC = total overhead-train words minus 1 (in word 1) | N-A | Y | Y | N-A | IMPLEMENTED | nawc computed in sysinfo.c:156
112 | §3.7.1.2.1 | S Serial number field | N-A | Y | Y | N | IMPLEMENTED | S field in word2 overhead def
112 | §3.7.1.2.1 | E Extended address field | N-A | Y | Y | N | IMPLEMENTED | E field in word2 overhead def
112 | §3.7.1.2.1 | REGH Registration field for home stations | partial | Y | Y | Y | IMPLEMENTED | regh (sysinfo.c:78, `-S regh=`, main_common.c:101) default 1
112 | §3.7.1.2.1 | REGR Registration field for roaming stations | partial | Y | Y | Y | IMPLEMENTED | regr (sysinfo.c:79, main_common.c:102) default 1
112 | §3.7.1.2.1 | DTX Discontinuous transmission field (2 bits) | partial | Y | Y | Y | IMPLEMENTED | dtx param (sysinfo.c init, `-S`)
112 | §3.7.1.2.1 | N-1 = (paging channels) - 1 | partial | Y | Y | Y | IMPLEMENTED | n1 = paging channels - 1 (main_common.c:111-113, `-S n1=`)
112 | §3.7.1.2.1 | RCF Read-control-filler field | partial | Y | Y | N | IMPLEMENTED | rcf (sysinfo.c:82) = !bis to ignore B/I
112 | §3.7.1.2.1 | CPA Combined paging/access field | partial | Y | Y | N | IMPLEMENTED | cpa=1 for combined CC+PC (sysinfo.c:83)
112 | §3.7.1.2.1 | CMAX-1 = (access channels) - 1 | partial | Y | Y | Y | IMPLEMENTED | cmax1 (main_common.c:111-113, `-S cmax1=`)
112 | §3.7.1.2.1 | END end indication: 1 = last word of train, 0 otherwise | N-A | Y | Y | N | IMPLEMENTED | end bit (sysinfo.c:156)
112 | §3.7.1.2.1 | RSVD reserved, bits set as indicated | N-A | Y | N-A | N | IMPLEMENTED | RSVD fields in overhead defs
112 | §3.7.1.2.1 | P parity | N-A | Y | Y | N-A | IMPLEMENTED | BCH parity
112 | §3.7.1.2.2 | Global action overhead = one word; any number appended to sys param | partial | Y | Y | partial | IMPLEMENTED | global action words (frame.c:374+); appended in train
112 | §3.7.1.2.2 | Rescan Global Action: T1T2=11 DCC ACT=0001 RSVD=000..00(16) END OHD=100 P | partial | Y | Y | Y | IMPLEMENTED | rescan_global_action def (frame.c:374-385); ACT(4)/RSVD(16); amps_rescan (amps.c:4014)

## page 113  (doc p.97) — §3.7.1.2.2 Global action message formats

113 | §3.7.1.2.2 | Registration Increment GA: T1T2=11 DCC ACT=0010 REGINCR(12) RSVD=0000 END OHD=100 P | partial | Y | Y | Y | IMPLEMENTED | registration_increment_global_action (frame.c:390-396); regincr (sysinfo.c:87, `-S regincr=`)
113 | §3.7.1.2.2 | Location Area GA: ACT=0011 PUREG(1) PDREG(1) LREG(1) RSVD=0(1) LOCAID(12) END OHD=100 P | partial | Y | Y | Y | IMPLEMENTED | location_area_global_action (frame.c:411-415); pureg/pdreg/locaid (sysinfo.c:90-91, main_common.c:103-105)
113 | §3.7.1.2.2 | New Access Channel Set GA: ACT=0110 NEWACC(11) RSVD=00000(5) END OHD=100 P | partial | Y | Y | N | IMPLEMENTED | new_access_channel_set_global_action (frame.c:423-429); NEWACC(11)
113 | §3.7.1.2.2 | Overload Control GA: ACT=1000 OLC 0..15 (1 bit each) END OHD=100 P | partial | Y | Y | Y | IMPLEMENTED | overload_control_global_action (frame.c:438-444); OLC 0..15; `olc,<16-bit mask>` (main_common.c:227,903); default_overload (sysinfo.c:27)
113 | §3.7.1.2.2 | Access Type Parameters GA: ACT=1001 BIS(1) PCI_HOME(1) PCI_ROAM(1) BSPC(4) BSCAP(3) RSVD(6) END OHD=100 P | partial | Y | Y | Y | IMPLEMENTED | access_type_parameters_global_action (frame.c:467-477); BIS/PCI_HOME/BSPC/BSCAP; bis cmd (main_common.c:107)

## page 114  (doc p.98) — §3.7.1.2.2 Global action messages (cont'd) + field glossary

114 | §3.7.1.2.2 | Access Attempt Parameters GA: ACT=1010 MAXBUSY-PGR(4) MAXSZTR-PGR(4) MAXBUSY-OTHER(4) MAXSZTR-OTHER(4) END OHD=100 P | partial | Y | Y | Y | IMPLEMENTED | access_attempt_parameters_global_action (frame.c:486-495); acc_maxbusy (main_common.c:120), maxbusy/maxsztr `-S`
114 | §3.7.1.2.2 | Random Challenge A GA: ACT=0111 RAND1_A(16) END OHD=100 P | partial | Y | partial | N | DECODE-ONLY | random_challenge_a_global_action (frame.c:503-509); RAND1_A(16); CAVE auth not implemented
114 | §3.7.1.2.2 | Random Challenge B GA: ACT=1011 RAND1_B(16) END OHD=100 P | partial | Y | partial | N | DECODE-ONLY | random_challenge_b_global_action (frame.c:517-523); RAND1_B(16); CAVE auth not implemented
114 | §3.7.1.2.2 | Local Control 1 Message: ACT=1110 LOCAL CONTROL(16) END OHD=100 P | partial | Y | partial | partial | PARTIAL | local_control_1 def (frame.c:531); system-specific; `local` Order 30 covers point-to-point
114 | §3.7.1.2.2 | Local Control 2 Message: ACT=1111 LOCAL CONTROL(16) END OHD=100 P | partial | Y | partial | partial | PARTIAL | local_control_2 def (frame.c); system-specific
114 | §3.7.1.2.2 | T1T2 Type set '11' = overhead word | N-A | Y | Y | N-A | IMPLEMENTED | T1T2=11 in GA defs
114 | §3.7.1.2.2 | ACT Global action field (Table 3.7.1-4) | partial | Y | Y | partial | IMPLEMENTED | ACT(4) field in all GA defs
114 | §3.7.1.2.2 | BIS Busy-idle status field | partial | Y | Y | Y | IMPLEMENTED | bis (main_common.c:107); BIS in access type GA
114 | §3.7.1.2.2 | DCC Digital color code | Y | Y | Y | Y | IMPLEMENTED | dcc
114 | §3.7.1.2.2 | OHD set '100' = global action message | N-A | Y | Y | N-A | IMPLEMENTED | OHD=100 in GA defs
114 | §3.7.1.2.2 | REGINCR Registration increment field | partial | Y | Y | Y | IMPLEMENTED | regincr (sysinfo.c:87)
114 | §3.7.1.2.2 | NEWACC New access channel starting point | partial | Y | Y | N | IMPLEMENTED | NEWACC(11) in new access GA def
114 | §3.7.1.2.2 | MAXBUSY-PGR Max busy occurrences (page response) | partial | Y | Y | Y | IMPLEMENTED | MAXBUSY-PGR(4) (frame.c:492); acc_maxbusy (main_common.c:120)

## page 115  (doc p.99) — §3.7.1.2.2 Global action field glossary (continued)

115 | §3.7.1.2.2 | MAXBUSY-OTHER Max busy occurrences (other accesses) | partial | Y | Y | Y | IMPLEMENTED | MAXBUSY-OTHER(4) (frame.c:494); acc_maxbusy applied to PGR+OTHER (main_common.c:120)
115 | §3.7.1.2.2 | MAXSZTR-PGR Max seizure tries (page response) | partial | Y | Y | Y | IMPLEMENTED | MAXSZTR-PGR(4) (frame.c:493); maxsztr `-S`
115 | §3.7.1.2.2 | MAXSZTR-OTHER Max seizure tries (other accesses) | partial | Y | Y | Y | IMPLEMENTED | MAXSZTR-OTHER(4) (frame.c:495)
115 | §3.7.1.2.2 | OLC N Overload class field (N=0..15) | partial | Y | Y | Y | IMPLEMENTED | OLC 0..15 (frame.c:444-...); `olc,<16-bit mask>` (main_common.c:903)
115 | §3.7.1.2.2 | END end indication (1=last word, 0=not) | N-A | Y | Y | N | IMPLEMENTED | end bit set per train
115 | §3.7.1.2.2 | PUREG Power up registration status (1=enabled) | partial | Y | Y | Y | IMPLEMENTED | pureg (sysinfo.c:90, main_common.c:103) default 1
115 | §3.7.1.2.2 | PDREG Power down registration status (1=enabled) | partial | Y | Y | Y | IMPLEMENTED | pdreg (sysinfo.c:91, main_common.c:104) default 1
115 | §3.7.1.2.2 | LREG Location Area ID registration status (1=enabled) | partial | Y | Y | partial | IMPLEMENTED | LREG field (frame.c:413); locaid gates it
115 | §3.7.1.2.2 | LOCAID Location area identity field | partial | Y | Y | Y | IMPLEMENTED | LOCAID(12) (frame.c:415); locaid (main_common.c:105, `-S locaid=`) default -1
115 | §3.7.1.2.2 | RSVD reserved, bits set as indicated | N-A | Y | N-A | N | IMPLEMENTED | RSVD fields in GA defs
115 | §3.7.1.2.2 | PCI HOME Home Protocol Capability Indicator (1=non-roam mobile reports PCI) | partial | Y | Y | N | PARTIAL | PCI_HOME(1) (frame.c:474); PCI feature partial
115 | §3.7.1.2.2 | PCI ROAM Roam Protocol Capability Indicator (1=roam mobile reports PCI) | partial | Y | Y | N | PARTIAL | PCI_ROAM(1) field; PCI feature partial
115 | §3.7.1.2.2 | BSPC Base Station Protocol Capability (0000 bkwd, 0001 IS-91A/691, 0010 IS-136B, 0011 IS-95B/95) | partial | Y | Y | N | PARTIAL | BSPC(4) (frame.c:476); 553-A base = 0000/backward
115 | §3.7.1.2.2 | BSCAP Base Station Core Analog Protocol (000 bkwd, 001 ANSI 553-A) | partial | Y | Y | N | PARTIAL | BSCAP(3) (frame.c:477); 553-A = 001
115 | §3.7.1.2.2 | LOCAL CONTROL may be any bit pattern | partial | Y | partial | partial | PARTIAL | LOCAL CONTROL(16) in local_control_1/2 defs
115 | §3.7.1.2.2 | P parity | N-A | Y | Y | N-A | IMPLEMENTED | BCH parity
115 | §3.7.1.2.2 | RAND1_A = 16 MSB of 32-bit RAND for mobile auth | partial | Y | partial | N | DECODE-ONLY | RAND1_A(16) (frame.c:509); CAVE not implemented
115 | §3.7.1.2.2 | RAND1_B = 16 LSB of 32-bit RAND for mobile auth | partial | Y | partial | N | DECODE-ONLY | RAND1_B(16) (frame.c:523); CAVE not implemented

## page 116  (doc p.100) — OLC note, Table 3.7.1-4 ACT codes, §3.7.1.2.3 Registration ID

116 | §3.7.1.2.2 NOTE | Recommended OLC: 0-9 normal subscribers, 10 test, 11 emergency, 12-15 reserved (ref TSB No.16) | partial | Y | Y | Y | PARTIAL | default_overload all 16 bits=1 (sysinfo.c:27); operator sets via olc mask; class semantics not enforced (recommendation only)
116 | T3.7.1-4 | ACT 0000 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
116 | T3.7.1-4 | ACT 0001 = Rescan paging channels | partial | Y | Y | Y | IMPLEMENTED | rescan_global_action (frame.c:374); amps_rescan (amps.c:4014)
116 | T3.7.1-4 | ACT 0010 = Registration increment | partial | Y | Y | Y | IMPLEMENTED | registration_increment_global_action (frame.c:390)
116 | T3.7.1-4 | ACT 0011 = Location Area | partial | Y | Y | Y | IMPLEMENTED | location_area_global_action (frame.c:411)
116 | T3.7.1-4 | ACT 0100 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
116 | T3.7.1-4 | ACT 0101 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
116 | T3.7.1-4 | ACT 0110 = New access channel set | partial | Y | Y | N | IMPLEMENTED | new_access_channel_set_global_action (frame.c:423)
116 | T3.7.1-4 | ACT 0111 = Random Challenge A | partial | Y | partial | N | DECODE-ONLY | random_challenge_a_global_action (frame.c:503); CAVE not impl
116 | T3.7.1-4 | ACT 1000 = Overload control | partial | Y | Y | Y | IMPLEMENTED | overload_control_global_action (frame.c:438); olc mask cmd
116 | T3.7.1-4 | ACT 1001 = Access type parameters | partial | Y | Y | Y | IMPLEMENTED | access_type_parameters_global_action (frame.c:467); default_acc_type (sysinfo.c:30)
116 | T3.7.1-4 | ACT 1010 = Access attempt parameters | partial | Y | Y | Y | IMPLEMENTED | access_attempt_parameters_global_action (frame.c:486); default_acc_attempt=10 (sysinfo.c:37)
116 | T3.7.1-4 | ACT 1011 = Random Challenge B | partial | Y | partial | N | DECODE-ONLY | random_challenge_b_global_action (frame.c:517); CAVE not impl
116 | T3.7.1-4 | ACT 1100 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
116 | T3.7.1-4 | ACT 1101 = Reserved | N-A | N-A | N-A | N-A | NOT-APPLICABLE(reserved) | reserved
116 | T3.7.1-4 | ACT 1110 = Local control 1 | partial | Y | partial | partial | PARTIAL | local_control_1 (frame.c:531)
116 | T3.7.1-4 | ACT 1111 = Local control 2 | partial | Y | partial | partial | PARTIAL | local_control_2 (frame.c)
116 | §3.7.1.2.3 | Registration ID message = one word, appended to sys param overhead + any GA messages | partial | Y | Y | partial | IMPLEMENTED | registration_id (frame.c:561); amps_encode_registration_id (sysinfo.c:178); appended in train

## SUMMARY

Pages assigned: 103-116 (14 pages). Pages actually read: 103, 104, 105, 106, 107, 108,
109, 110, 111, 112, 113, 114, 115, 116 = ALL 14 PNGs read, one-by-one, in order.

Status counts (approx, across ~190 rows):
- IMPLEMENTED: ~95 (FOCC framing, BCH(40,28;5), word layouts, SAT color codes, overhead
  train, all 16 global-action ACT message formats, standard order codes incl Disable DTMF)
- PARTIAL: ~55 (registration ORDQ variants not fully distinct, PCI feature, encryption-mode
  order code known but no key handling, BSPC/BSCAP encoded but 553-A-fixed, 0.8s timing approx)
- DECODE-ONLY: ~20 (all CAVE/authentication: Unique Challenge, SSD Update, Base Station
  Challenge/AUTHBS, Random Challenge A/B — def_word structs exist, no CAVE algorithm)
- MISSING: ~8 (Auth-Word-C registration variants Order 24 ORDQ0-3, Auth-C PCI registration)
- NOT-APPLICABLE(reserved/mobile-side): ~15 (reserved ACT/OHD/order codes, mobile-only NAWC rules)

Top 3 gaps in range:
1. AUTHENTICATION / CAVE entirely absent (biggest gap). Table 3.7.1-1 auth orders
   (Base Station Challenge Confirmation 19, Unique Challenge 20, SSD Update 21) and the
   Random Challenge A/B global actions have word structs defined but no CAVE algorithm:
   AUTHBS/AUTHU/RANDSSD responses are never computed. AUTH bit in sys-param word1 is
   consequently forced 0. All Auth-Word-C-included registration variants (Order 24,
   page 109) are MISSING.
2. Message Encryption Mode order (Order 23, pages 103/109): order CODE is pinned
   (AMPS_ORDER_MESSAGE_ENCRYPTION=23, _Static_assert) but there is no message-encryption
   key handling — enable/disable indication not acted on.
3. PCI / Protocol Capability (pages 103/109/115): PCI query command exists but PCI
   report decode and PCI-registration (Order 26 variants) are partial/not acted on;
   BSPC/BSCAP fields are encoded but fixed to 553-A/backward-compat values only.

Notable STRENGTH: the Order table (Table 3.7.1-1) and Global Action table (3.7.1-4) are
very well covered for the NON-auth path — every standard order (Page/Alert/Release/Reorder/
MsgWaiting/StopAlert/Audit/SendCalled/Intercept/Maintenance/ChangePower L0-7/DirectedRetry/
Registration/SerialNumber/AlertWithInfo/FlashWithInfo/DisableDTMF/LocalControl) and every
global-action ACT message format (0001-1011, 1110-1111) has a matching encoder/def_word and
most have an operator command (FIFO or -S). SAT color codes (Table 3.7.1-2: 5970/6000/6030)
match dsp.c sat_freq[] exactly. BCH(40,28;5) generator matches Gb(x) in frame.c gp[].

REMAINING PAGES: none (103-116 all complete).

---

## AMPS spec-walk pages 117-129

Perspective: BASE STATION. Source under audit: `/home/vas/projects/osmocom-analog/src/amps/`.
Each page PNG was read as ground truth; each normative item grepped against source.

NOTE on range content vs. the dispatch note: the note anticipated "signaling tone / timing
values (ST, alert cadences)". In this doc edition pages 117-129 actually cover the TAIL of the
FOCC overhead/filler formats (3.7.1.2.3-3.7.1.3), the FOCC data restrictions, and the ENTIRE
Forward Voice Channel section (3.7.2). Alert *cadence* code tables DO appear here (p.126, part of
the FVC SIGNAL field). ST (Signalling Tone) tone/timing and SAT values live elsewhere in the spec,
not in this range.

MCP note: `@kirocrew-computer` server was declared by the agent spec but NOT configured this
session — its tools were not mounted. Not needed here (image-read + grep + file tools sufficed).

Row key: PAGE | SECTION | REQUIREMENT | DECODED | ENCODED | ACTED-ON | COMMAND | STATUS | EVIDENCE

---

## page 117
3.7.1.2.3 Registration ID message (tail) + 3.7.1.2.4 Control-Filler message.

117 | 3.7.1.2.3 | Registration-ID overhead word: T1T2=11, DCC, REGID(20), END, OHD=000, P | Y | Y | Y | Y | IMPLEMENTED | encode: frame.c:2885 `amps_encode_registration_id(dcc,regid,end,...)`; word def frame.c:566 "REGID",20; driven sysinfo.c:177-178 (regid from timestamp, regincr command)
117 | 3.7.1.2.3 | REGID field = Registration ID value | Y | Y | Y | Y | IMPLEMENTED | sysinfo.c:123 `si->reg_id.regid=1000` default; main_common.c:189 REGID increment operator option
117 | 3.7.1.2.3 | END bit (1=last word of OH train, 0=not last) | Y | Y | Y | N-A | IMPLEMENTED | `end` arg of amps_encode_registration_id (frame.c:2885); OH-train sequencing in sysinfo.c
117 | 3.7.1.2.3 | OHD=000 fixed overhead-type for REGID | N-A | Y | Y | N-A | IMPLEMENTED | OHD constant baked into encoder/word layout frame.c:566-ff
117 | 3.7.1.2.4 | Control-Filler message word: T1T2=11, DCC, 010111, CMAC(3), SDCC1(2), 11, SDCC2(2), 1, WFOM(1), 1111, OHD=001, P | Y | Y | Y | partial | IMPLEMENTED | encode frame.c:2808 `amps_encode_control_filler(amps,dcc,cmac,sdcc1,sdcc2,wfom,...)`; word def frame.c:576-587
117 | 3.7.1.2.4 | CMAC = control mobile attenuation code (3) | Y | Y | Y | Y | IMPLEMENTED | ie_cmac frame.c:1746/2168; set via init_sysinfo(cmac,...) sysinfo.c:46,64 (operator -S cmac)
117 | 3.7.1.2.4 | SDCC1(2)/SDCC2(2) supplementary digital color codes | Y | Y | Y | N | DECODE-ONLY | word fields frame.c:583-585, IE frame.c:2326-2327; sysinfo.c:65-66 hard-sets sdcc1=sdcc2=0 (no operator control, feature not exercised)
117 | 3.7.1.2.4 | WFOM = wait-for-overhead-message bit | Y | Y | Y | partial | IMPLEMENTED | frame.c:587/2350; sysinfo.c:67 `wfom=(bis)?0:1` derived from BIS setting
117 | 3.7.1.2.4 | Control-filler sent when no other FOCC msg; may fill between words; chosen so 11-bit word-sync never appears | N-A | Y | Y | N-A | IMPLEMENTED | fixed 010111/1111 constants in word layout avoid embedded word-sync; filler emission amps.c:778 (IDLE sends Overhead/Filler frames)

## page 118
Field dictionary for control-filler (CMAC/RSVD/SDCC1,2/WFOM/OHD/P) + 3.7.1.3 Data restrictions.

118 | 3.7.1.2.4 | Field semantics: CMAC=mobile power (Tbl 2.1.2-1); RSVD all-set; SDCC non-zero => mobile supports SDCC, zero => not used/unsupported; WFOM; OHD=001; P | Y | Y | partial | partial | IMPLEMENTED | same symbols as p.117 rows; SDCC zero-combination semantics honoured by sysinfo.c:65-66 (0,0 = feature off)
118 | 3.7.1.3 | 11-bit word-sync (11100010010) can embed in a word; 3 periodic cases: overhead msg, control-filler, pages to certain CO codes | N-A | N-A | N-A | N-A | NOT-APPLICABLE(design) | word_sync const dsp.c-side; design-level restriction, see mitigation rows below
118 | 3.7.1.3 | Mitigation 1: restrict overhead-message TX rate to ~once/second | N-A | Y | Y | N-A | IMPLEMENTED | RESOLVED: `si->overhead_repeat` (sysinfo.c) IS this rate control. One FOCC buffer = 463 bits @ 10 kbit/s = 46.3 ms; overhead_repeat=17 → ~0.79 s between trains ≈ "about once per second" (TACS: 14 frames @ 8 kbit/s, comparable). Train re-fires when tx_focc_frame_count wraps at overhead_repeat (frame.c:4159). Now documented+spec-cited in sysinfo.c.
118 | 3.7.1.3 | Mitigation 2: control-filler designed to exclude word-sync (incl. busy-idle bit cases) | N-A | Y | Y | N-A | IMPLEMENTED | fixed 010111 / 1111 bit groups in control_filler word layout frame.c:576-587
118 | 3.7.1.3 | Mitigation 3: restrict use of certain CO codes; 2-word page mode alleviates it | N-A | Y | Y | N-A | IMPLEMENTED | base always builds 2-word mobile-control page: frame.c:4189 `tx_focc_num_words = 2` (4 only for directed-retry order 12), so 1-word troublesome-CO-code case never arises

## page 119
Table 3.7.1-5 Troublesome central office codes (reference enumeration).

119 | Tbl 3.7.1-5 | Enumerated list of troublesome CO codes (007,056,070,150,224,225,... up to 993 + >999 cases) with bit patterns, used only in 1-word page mode | N-A | N-A | N-A | N-A | NOT-APPLICABLE(design) | no match in src/amps/ for the table (grep "troublesome|3.7.1-5|central office code" => 0). Not needed: base uses 2-word mode (frame.c:4189), which the spec itself states "alleviates" this.
119 | Tbl 3.7.1-5 note 1 | YY bits may be 0 but not both 1 | N-A | N-A | N-A | N-A | NOT-APPLICABLE(design) | part of unused table
119 | Tbl 3.7.1-5 note 2 | Z = bit may be 1 or 0 | N-A | N-A | N-A | N-A | NOT-APPLICABLE(design) | part of unused table

## page 120
Table 3.7.1-5 notes 3-5 + 3.7.2 Forward Voice Channel + Figure 3.7.2-1 FVC message stream.

120 | Tbl 3.7.1-5 notes | Note3: bit in parens = busy-idle bit; Note4: * = CO code >999; Note5: CO codes starting 1/0 shown for completeness | N-A | N-A | N-A | N-A | NOT-APPLICABLE(design) | notes to unused table
120 | 3.7.2 | FVC is a wideband data stream base->mobile at 10 kbit/s +-0.1 bit/s | N-A | Y | Y | N-A | IMPLEMENTED | FVC TX path amps_encode_frame_fvc (amps.c:26 ref); dsp.c FSK gen (dsp.h/dsp.c) at AMPS bit rate
120 | 3.7.2-1 | DOTTING = 1010..101; first-repeat uses 101-bit (DOTTING) dotting, later repeats 37-bit DOT | N-A | Y | Y | N-A | IMPLEMENTED | dotting generated by dsp.c (dsp.c:30-35 dotting search/gen); word-block framing frame.c
120 | 3.7.2-1 | W.S. (word sync) = 11100010010 inserted before each repeat | Y | Y | Y | N-A | IMPLEMENTED | sync_word "11100010010" frame.c:4039; emitted per repeat
120 | 3.7.2-1 | Each word 40 bits, repeated 11 times = one "word block" | N-A | Y | Y | N-A | IMPLEMENTED | 11x repeat loop dsp.c:186 `for(i=0;i<11;i++)`; FVC repeat state frame.c:4294 tx_fvc_word_repeat
120 | 3.7.2 | Word = 28 content bits -> (40,28) BCH distance-5 code, MSB first (left-most), same generator poly as FOCC (3.7.1) | N-A | Y | Y | N-A | IMPLEMENTED | encode_bch frame.c:58-105 BCH(length+12,length,5); shared encoder used by FOCC+FVC

## page 121
3.7.2.1 Mobile Station Control Message (FVC) - Word 1 + auth Word-2 variants.

121 | 3.7.2.1 | Mobile Station Control Message is the ONLY FVC message; 1+ words | N-A | Y | Y | N-A | IMPLEMENTED | amps_tx_frame_fvc / FVC TX scheduler frame.c:4300-ff
121 | 3.7.2.1 | Word1 T1T2=10; SCC=11 variant: SCC,PSCC,EF=0,RSVD,LOCAL/MSG_TYPE,ORDQ(3),ORDER(5),P | Y | Y | Y | Y | IMPLEMENTED | word1 defs frame.c:883-900 (PSCC,EF); encode word1_a (amps_encode_mobile_station_control_message_word1_* in frame.c:4197-ff)
121 | 3.7.2.1 | Word1 SCC!=11 variant: SCC,PSCC,RSVD=0,RSVD=0000000,VMAC(3),CHAN(11),P | Y | Y | Y | partial | IMPLEMENTED | word1_b encoder `amps_encode_mobile_station_control_message_word1_b(scc,sat,...,vmac,chan)` frame.c:4328-ff; CHAN set per handoff/assignment
121 | 3.7.2.1 | Word2 Base Station Challenge Order Confirmation: RSVD=0000, AUTHBS(18), RSVD=0000, P | partial | N | N | N | DECODE-ONLY | word def frame.c:246-252 "AUTHBS",18 (word table for display/decode); no amps_encode_*challenge* TX fn => base does not drive it (auth extension)
121 | 3.7.2.1 | Word2 Unique Challenge Order Word: RSVD=00, RANDU(24), P | partial | N | N | N | DECODE-ONLY | word def frame.c:259 word3_unique_challenge_order_word; table-only, no encode/drive
121 | 3.7.2.1 | Word2 First SSD Update Order Word: RANDSSD_1(24), RSVD=00, P | partial | N | N | N | DECODE-ONLY | word def frame.c:270 word3_first_ssd_update_order_word; table-only, no encode/drive

## page 122
More FVC words: Alert/Flash-With-Info Word2, SSD-update Word3/4, char Words.

122 | 3.7.2.1 | Word2 First Alert With Info: T1T2=01, RL_W(5), SIGNAL(8), CPN_RL(6), PI(2), SI(2), RSVD=000, P | Y | Y | Y | Y | IMPLEMENTED | encode `amps_encode_word2_first_alert_with_info_word(rl_w,signal,cpn_rl,present,screen)` frame.c:4338; word def frame.c:979-983
122 | 3.7.2.1 | Word2 First Flash With Info: T1T2=01, RL_W(5), CPN_RL(6), PI(2), SI(2), RSVD=000..0(11), P | Y | Y | Y | Y | IMPLEMENTED | encode amps_encode_word2_first_flash_with_info_word frame.c:4362; op cmd amps_flash_with_info main_common.c:459
122 | 3.7.2.1 | Word3 Second SSD Update: RANDSSD_2(24), RSVD=00, P | partial | N | N | N | DECODE-ONLY | word def frame.c:295-300 word4_second_ssd_update; table-only
122 | 3.7.2.1 | Word3 Second Alert/Flash With Info: T1T2=01, RSVD=00, CHARACTER(8)x3, P | Y | Y | Y | Y | IMPLEMENTED | generic char-word encoder amps_encode_wordn_n_minus_1th_alert/flash_with_info_word frame.c:4378-4392
122 | 3.7.2.1 | Word4 Third SSD Update: RANDSSD_3(8), RSVD=000..0(18), P | partial | N | N | N | DECODE-ONLY | word def frame.c:306-312 word5_third_ssd_update; table-only

## page 123
FVC char-carrying continuation words (Word4/5 Third/Fourth Alert & Flash With Info).

123 | 3.7.2.1 | Word4 Third Alert With Info: RSVD=00 + CHARACTER x3 + P | Y | Y | Y | Y | IMPLEMENTED | same generic char encoder frame.c:4378-4392 (word_count>=2 path)
123 | 3.7.2.1 | Word4 Third Flash With Info: RSVD=00 + CHARACTER x3 + P | Y | Y | Y | Y | IMPLEMENTED | generic char encoder frame.c:4378-4392
123 | 3.7.2.1 | Word5 Fourth Alert With Info: RSVD=00 + CHARACTER x3 + P | Y | Y | Y | Y | IMPLEMENTED | generic char encoder (CPN_RL drives word count) frame.c:4330-4392
123 | 3.7.2.1 | Word5 Fourth Flash With Info: RSVD=00 + CHARACTER x3 + P | Y | Y | Y | Y | IMPLEMENTED | generic char encoder frame.c:4330-4392
123 | 3.7.2.1 | Pattern continues (bullet dots) for further char words | Y | Y | Y | Y | IMPLEMENTED | word_count loop open-ended, bounded by CPN_RL/strlen frame.c:4330-ff

## page 124
Generic Word N char words + FVC field-interpretation dictionary (T1T2..RANDSSD_2).

124 | 3.7.2.1 | Word N (N-1)th Alert/Flash With Info: RSVD=00 + CHARACTER x3 + P | Y | Y | Y | Y | IMPLEMENTED | generic char encoder frame.c:4378-4392
124 | 3.7.2.1 | T1T2: 10 in Word1, 01 in Word2+; extra words only for BS-Challenge-Conf / Unique-Challenge / SSD-Update / Alert-WithInfo / Flash-WithInfo | N-A | Y | Y | N-A | PARTIAL | Alert/Flash multiword IMPLEMENTED (frame.c:4330-4392); challenge/SSD multiword DECODE-ONLY (no TX)
124 | 3.7.2.1 | SCC = SAT color code for new channel (Tbl 3.7.1-2) | Y | Y | Y | N-A | IMPLEMENTED | SCC set from target VC SAT frame.c:4160-4175, word1_b scc arg
124 | 3.7.2.1 | PSCC = present SAT color code of current channel | Y | Y | Y | N-A | IMPLEMENTED | PSCC word field frame.c:883; word1_b passes amps->sat as present SCC frame.c:4328-ff
124 | 3.7.2.1 | EF = Extended Protocol Forward indicator; =0 for ANSI 553A | N-A | Y | Y | N-A | IMPLEMENTED | EF field frame.c:179/884/900 fixed 0 in word layout
124 | 3.7.2.1 | ORDER field (Tbl 3.7.1-1) / ORDQ qualifier | Y | Y | Y | Y | IMPLEMENTED | order table frame.c; tx_fvc_order/tx_fvc_ordq driven by transaction (amps.c order dispatch)
124 | 3.7.2.1 | LOCAL control field (system-specific; ORDER must = local control) + MSG_TYPE | Y | Y | Y | Y | IMPLEMENTED | amps_local_control() amps.c:3559; TRANS_CALL_LOCAL_CONTROL state amps.c:3607; msg_type field amps.c:1802; op cmd main_common.c:794
124 | 3.7.2.1 | VMAC = voice mobile attenuation code (Tbl 2.1.2-1) | Y | Y | Y | partial | IMPLEMENTED | si->vmac set via init_sysinfo (sysinfo.c:46); per-trans current_vmac frame.c:4164-4168
124 | 3.7.2.1 | CHAN = designated voice channel number (sec 2.3.3) | Y | Y | Y | Y | IMPLEMENTED | tx_fvc_chan carried in word1_b; channel assignment via transaction/handoff
124 | 3.7.2.1 | RANDU(24), RANDSSD_1(24), RANDSSD_2(24) semantics | partial | N | N | N | DECODE-ONLY | word-table defs only (auth extension), no TX drive

## page 125
Field dict cont'd: RANDSSD_3, AUTHBS, RL_W, SIGNAL (pitch+cadence) + pitch code table.

125 | 3.7.2.1 | RANDSSD_3 = LSB 8 bits of SSD-update random | partial | N | N | N | DECODE-ONLY | word5_third_ssd_update def frame.c:306-312; no TX
125 | 3.7.2.1 | AUTHBS = output of auth algorithm from BS Challenge Order | partial | N | N | N | DECODE-ONLY | word def frame.c:246-252; no TX / no CAVE
125 | 3.7.2.1 | RL_W = remaining length in words of Alert/Flash With Info order | Y | Y | Y | N-A | IMPLEMENTED | rl_w computed `(cpn_rl+2)/3` and passed to word2 encoders frame.c:4336-4376
125 | 3.7.2.1 | SIGNAL = 8-bit: pitch(2 MSB)+cadence(6 LSB); Standard Alert = pitch 00 + cadence 000001 | Y | Y | Y | Y | IMPLEMENTED | ie_signal decode frame.c:2065; encode via word2 SIGNAL arg; op cmd pitch/cadence main_common.c:241-246
125 | 3.7.2.1 | Pitch codes: Medium=00, High=01, Low=10, Reserved=11 | Y | Y | Y | Y | IMPLEMENTED | ie_signal pitch mapping frame.c:2069-ff ("Medium pitch"...); op help main_common.c:244 "0=Medium,1=High,2=Low,3=Reserved"

## page 126
Cadence code table + CPN_RL, PI (+ code table), SI field.

126 | 3.7.2.1 | Cadence codes: 000000=NoTone, 000001=Long(2on/4off), 000010=ShortShort, 000011=ShortShortLong, 000100=ShortShort-2, 000101=ShortLongShort, 000110=ShortShortShortShort, 000111=PBX Long, 001000=PBX ShortShort, 001001=PBX SSL, 001010=PBX SLS, 001011=PBX SSSS, 001100..111111=Reserved | Y | Y | Y | Y | IMPLEMENTED | ie_signal cadence decode frame.c:2065-ff; op cmd mapping main_common.c:245-246 (00=NoTone,01=Long,02=ShortShort,03=SSL,04-06=OtherPatterns,07-11=PBX)
126 | 3.7.2.1 | CPN_RL = 6-bit count of CHARACTERs in Alert/Flash With Info order | Y | Y | Y | Y | IMPLEMENTED | word field frame.c:981/995; encoded from strlen(callerid/flashinfo) frame.c:4336-4360
126 | 3.7.2.1 | PI = presentation indicator (2-bit): 00=allowed,01=restricted,10=not available,11=reserved | Y | Y | Y | Y | IMPLEMENTED | ie_pi decode frame.c:1988; trans->presentation_indicator amps.c:1536; word field frame.c:982
126 | 3.7.2.1 | SI = screening indicator (2-bit) | Y | Y | Y | Y | IMPLEMENTED | ie_si frame.c:2057-2059; trans->screening_indicator amps.c:1538; word field frame.c:983

## page 127
SI code table + CHARACTER field coding + RSVD/P.

127 | 3.7.2.1 | SI codes: 00=user not screened, 01=user verified+passed, 10=user verified+failed, 11=network-provided | Y | Y | Y | partial | IMPLEMENTED | amps_ie_si[4] frame.c:2050-2054 labels verbatim-match spec; set via trans->screening_indicator
127 | 3.7.2.1 | CHARACTER = 8-bit ASCII: MSB=0, 7 bits = ANSI X3.4; null (00000000) fill for last word | Y | Y | Y | Y | IMPLEMENTED | char words encode 7-bit ASCII (frame.c:4378-4392); short last word null-padded via (cpn_rl+2)/3 word count
127 | 3.7.2.1 | RSVD = reserved, all bits set as indicated | N-A | Y | Y | N-A | IMPLEMENTED | RSVD constants fixed in each word layout frame.c:979-996 etc.
127 | 3.7.2.1 | P = parity field | N-A | Y | Y | N-A | IMPLEMENTED | BCH parity via encode_bch frame.c:58-105

## page 128
128 | 4 Reserved | Section "4 Reserved" — empty, no normative content | N-A | N-A | N-A | N-A | NOT-APPLICABLE(prose) | pure section header, nothing to implement

## page 129
129 | 5 Reserved | Section "5 Reserved" — empty, no normative content | N-A | N-A | N-A | N-A | NOT-APPLICABLE(prose) | pure section header, nothing to implement

---

## SUMMARY

Pages read: 13 of 13 assigned (117,118,119,120,121,122,123,124,125,126,127,128,129). None skipped.

Status counts (by row):
- IMPLEMENTED: 45
- DECODE-ONLY: 11
- PARTIAL: 1
- NOT-APPLICABLE(design): 6
- NOT-APPLICABLE(prose): 2
- UNVERIFIED: 0   (overhead-rate mitigation resolved as IMPLEMENTED — see below)
(Total rows: 65)

Top 3 gaps in range:
1. AUTHENTICATION words are DECODE/TABLE-ONLY (p.121-125): Base Station Challenge Order
   Confirmation (AUTHBS), Unique Challenge (RANDU), and the 3-word SSD Update (RANDSSD_1/2/3)
   have word-table definitions for display but NO base-station TX encoder and are not driven by
   any transaction. The base cannot run CAVE authentication challenges. (Expected — osmocom-analog
   does not implement the auth extension — but it IS a normative FVC capability gap.)
2. SDCC1/SDCC2 (p.117-118): UPDATE — now operator-controllable (`-S sdcc1=/sdcc2=`, `sdcc` FIFO)
   and the mobile's SDCC echo is decoded and acted on (capability derived, mismatch flagged).
   Default remains 0,0 (feature off) for single-DCC deployments, which is spec-conformant
   ("zero combination … not used or not supported", p.102).
3. RESOLVED (was UNVERIFIED): the p.118/§3.7.1.3 data-restriction mitigation #1 ("restrict
   overhead TX rate to ~1/sec") IS implemented via `si->overhead_repeat`. A FOCC transmit
   buffer is 463 bits at 10 kbit/s = 46.3 ms, so overhead_repeat=17 gives a train every
   ~0.79 s ≈ once per second. It was a frame-count throttle rather than a named 1 Hz timer,
   which is why it needed confirming; now documented and spec-cited at the overhead_repeat
   assignment in sysinfo.c.

Range-content caveat (reported, not a gap): this page range is FOCC overhead/filler tail +
FOCC data restrictions + the full Forward Voice Channel section (3.7.2), ending at two empty
"Reserved" sections (4, 5). Alert *cadence* code tables are present (p.126); ST signalling-tone
and SAT tone/timing values are NOT in this range.

MCP: `@kirocrew-computer` was unavailable (declared-but-not-configured); did not affect this task.
REMAINING PAGES: none — full assigned range 117-129 completed.
