# SDR clock and calibration: what we know, what to do

Written 2026-11-27 so the research survives. Two topics:
- **A. Calibration**: osmocalibrate, GSM and LTE towers as the frequency reference.
- **B. Time and clock monitoring**: the clock status, the drift measurement, and correction.

Afterwards, A and B get merged, then the `ermes` branch goes back to the osmocom-refactor session.

Repo: `~/projects/osmocom-analog-ermes`, branch `ermes`. Build: `~/.cache/sdrangel-rp-ermes/build`.

## Data points (B210 serial 30AA038, USB 3, UHD 4.8, RX2, warm)

### Against GSM towers (the real reference)

| Measurement | Internal TCXO | External ref ("built-in GPS") | RTL-SDR (R820T TCXO) |
|---|---|---|---|
| kalibrate-uhd/-rtl, GSM-900 ch 76 | +4.168 kHz = 4.39 ppm slow (±6 Hz) | +14.023 kHz = 14.76 ppm slow | +312 Hz = -0.33 ppm |
| osmocalibrate (af61683), ch 76 / 103 / 49 | 4.392 / 4.389 / 4.471 ppm | 14.761 ppm (ch 76) | not run |
| osmocalibrate, GSM-1800 ch 715 | 4.403 ppm | - | - |
| Error at 169.625 MHz (ERMES ch 9) | ≈ -745 Hz (the standard allows ±200 Hz) | ≈ -2.5 kHz | ≈ -56 Hz |

- **Towers, all confirmed by their decoded ID:**

  | Band | Channel | BSIC | Operator | LAC / CI |
  |---|---|---|---|---|
  | GSM-900 | 76 | b1 | A | L1 / C1 |
  | GSM-900 | 103 | b2 | B | L2 / C2 |
  | GSM-900 | 49 (weak, σ 11 Hz) | b3 | C | L3 / C3 |
  | GSM-1800 | 715 | b4 | A | L1 / C4 |

  - Identities are masked here (they locate the bench): operators A/B/C, LAC L1–L3, cell IDs C1–C4, BSIC b1–b4. Equal labels mean equal values.
  - Ch 100 (+36 kHz, σ 375 Hz) is no cell, and is rejected automatically.
  - One country, three operators; all agree.
- **LTE (own decoder, intermediate result, 2026-11-27 23:30; B210 internal, 0.15 s captures at 30.72 MS/s):**
  - **Cells, all MIB-confirmed** (12–15 good decodes each, SFN consistent):
    - band 8: 3575, 3676;
    - band 3: 1275, 1500;
    - band 1: 125, 499;
    - band 40: 39300;
    - band 7: 3025.
  - **They give +4.420 to +4.423 ppm.** GSM gives 4.39–4.405, so 0.02–0.03 ppm apart, within the towers' tolerance.
  - **39498** (band 40, the same PCI p4 as 39300, SINR -2.6 dB, +4.4675 ppm) is suspect as a duplicate or edge alias; it's being checked.
  - **Band 20** (780/806) is empty here.
  - The RTL CellSearch missed the band 8 cells.
  - **CPU** is 51–105 % of real time per EARFCN offline: too high, to come down via the bank.
  - Files: `~/.cache/sdrangel-rp-ermes/lte/scan/`.
- **The external reference is the bad one.** It runs 14.8 ppm slow, against 4.4 ppm for the internal oscillator, even though `ref_locked` = 1: the PLL follows its input, which says nothing about whether the input is right.
  - UHD sees no GPSDO: the only sensor is `ref_locked`, with no `gps_locked`.
  - Debug folder: `~/projects/b210-ref-debug/` (README, evidence, and `fpga/` for the user's code).

### Device time against the PC clocks (ERMES, 5 min runs, TX running)

| Run | Reference | vs CLOCK_MONOTONIC_RAW | vs NTP UTC (timesyncd) |
|---|---|---|---|
| 1 | internal | +40.16 ppm | -4.79 ppm |
| 2 | external | +29.78 ppm | -15.08 ppm |
| 3 | internal | +40.18 ppm | -5.30 ppm |

- **NTP vs the towers:** they agree within 0.3–0.9 ppm after only 5 min. NTP is a usable long-term yardstick, but not a precise one.
- **The PC crystal** is about 45 ppm off; that's raw vs NTP.
- **The internal-to-external difference** is 10.2–10.4 ppm by all three methods. The methods are consistent.
- **The temperature dependence is unknown.** All values are from a warm device; a cold-start series is still to do.

### Tools

| Tool | What it does | Notes |
|---|---|---|
| `~/.cache/sdrangel-rp-kal/kalibrate-uhd/src/kal` | GSM offset with the B210 | `-s GSM900 -g 85 -A 1` to scan, `-c 76 -b GSM900 -g 85 -A 1 [-x]` to measure. Slow, several minutes per band |
| `~/.cache/sdrangel-rp-kal/kalibrate-rtl/src/kal` | GSM offset with the RTL | `-c 76 -b GSM900 -g 40` |
| `~/.cache/sdrangel-rp-kal/LTE-Cell-Scanner-jx/build/src/CellSearch` | LTE cell search, RTL | AGPL-3, so don't copy its code |
| osmocalibrate | our own, about 12 s per tower | `--sdr-uhd --band 900 --arfcn 76 --sdr-rx-gain 50 --sdr-rx-antenna RX2` |

## A. Calibration (osmocalibrate): plan and state

### Done

- **af61683, `src/libcalibrate/gsm_sch.c`:** our own SCH/BCCH decoder, no libraries.
  - A tower is confirmed by 2 or more SCH decodes with the same BSIC and frame numbers that advance correctly, plus SI3.
  - The decoder costs about 0.6 % of a core.
  - The confidence metric was fixed, and the trimmed-mean pairing bug too.

### Problems in the old frame

The old frame is to be rewritten; the measurement modules stay.
- **It bypasses libsdr:** direct `uhd_open`/`uhd_receive`/`soapy_*` calls. That causes an RX overflow (about 25 ms) at the start and at each phase-2 reader start, and libsdr's options and gap handling are skipped.
- **Two channelizer paths:** `channelizer_rx_init` per ARFCN, plus a separate polyphase path (`--channelizer`).
- **Fixed 1 MHz rate,** GSM only, no CPU measurement.
- **Slow scan:** a full GSM-900 scan takes 40 s, because it probes 53 channels one after another.
- **Dead code:** `calibrate_sdr.c` is not built.

### Decision (advisor)

Rewrite the frame and keep the proven modules: `fcch_detect.c`, `gsm_sch.c`, `gsm_bands.c` and the offset statistics. Do it in slices, each compared old vs new on ARFCN 76, 103 and 715.

1. **I/O swap only**: `sdr_open(channels = 0)`, `sdr_read`, `sdr_set_rx_frequency`. Delete `calibrate_sdr.c`. **DONE, 80145d8.**
   - **Result:** ppm within 0.003 of old on all runs. 76: 4.395, 103: 4.392, 715: 4.404, external: 14.765, `--sdr-ppm 4.4` residual 0.011.
   - **Overflow/gap lines:** 0 new vs 2–4 per old run. Logs in `~/.cache/sdrangel-rp-ermes/calib-runs/`; old binary at `~/.cache/sdrangel-rp-ermes/osmocalibrate.old`.
   - **Overflow cause:** osmocalibrate's own pauses with nothing read: 84–88 ms of filter and channel setup at the start of phase 2 (`calibrate.c` about 1712–1751), and 121 ms from start to the first read on 1800. The B210 buffers about 60 ms, so the excess was lost. libsdr's read thread drains the device meanwhile (0.8 s ring). **Not a libsdr or refactor bug.**
   - **CPU +20–25 %** (1.46 → 1.80 s per 11 s run). Cause not measured; the read thread copies, and the band channelizer runs with no stage. Slice 2 must find and remove it (perf), per [[cpu-budget-perf]].
   - **Open for slice 2:**
     - The flush after a retune: the backlog is now kept in the ring, so after a band change (121 ms) the first samples may be from before the LO settled. Drop by device time, or by a count from the retune.
     - ARFCN 103 reads +2.4 Hz vs old, consistently (0.003 ppm, unexplained).
   - **RTL via Soapy:** old 0 of 50 measurements; new -0.401 ppm in 71 s, confirmed (kalibrate-rtl -0.33). One run only.
   - Pass: ppm within 0.02 of the old binary, and **zero** RX overflow or gap lines.
   - If the overflow stays, it's a libsdr bug (read thread / `uhd_receive`), to fix in libsdr. Until this slice proves otherwise, don't claim it is "not the refactor".
   - Check that the upconverter is applied exactly once.
   - `--sdr-ppm` must not leak into the measurement: measure the raw error, or report the result as a residual.
   - The band's shift off DC must not bias the offset.
2. **New scan and selection, auto mode.**
   - **Rate:** the SDR layer picks it (device rates, USB with margin).
   - **Channels:** as many per window as the CPU budget allows (default half a core, with the cost per channel measured in the first second). Windows one after another over the reachable bands.
   - **Order:** FFT power rank, then FCCH on the top N, with SCH as the arbiter.
   - **Stop:** after 3 agreeing confirmed towers.
   - **Options:** `--band all|list`, `--channels auto|N`, `--cpu %`, `--sdr-samplerate`, `--towers N`, `--full`.
   - **Delete:** the polyphase path and the debug-family code. The SCH confirmation replaces most of the pre-validation heuristics.
   - **Output:** the ppm result, plus per tower its ID, offset, and level in dBFS (no fake dBm unless the gain is calibrated) with SNR. No confidence percentage: confirmed IDs plus agreeing towers are the confidence. On disagreement, show the spread.
   - **Measure** time to a confirmed result and CPU, old vs new.
   - **DONE 2026-11-28:** commits e51b83a, d098423, f29423b, 24b38ff, ef00f4e, f2b73d4, 7f2b980, 64e4346. Work files are in `~/.cache/sdrangel-rp-ermes/s2/`: `stats.txt`, `runs/`, `bench/`, and the patches.
   - **CPU of slice 1, the cause (perf, old vs s1):** s1 had +0.85 G cycles per run on ARFCN 76.
     - The band channelizer with nothing to do (NCO, copies): about 0.19 s.
     - The read thread's per-sample ring copy with a modulo: about 0.10 s.
     - The IQ level counters: about 0.05 s.
     - calibrate.c's own second ring and reader thread (iqringbuf).
   - **The fix, and what each part saves** (instructions / cycles, mean of 2):
     - libsdr passes the band through (sdr_rate = dsp_rate, no shift): -0.36 G / -0.13 G.
     - The ring copy as memcpy: -0.31 G / -0.26 G.
     - The IQ counters off in osmocalibrate (`--sdr-iq-status on` turns them back on): -0.47 G / -0.12 G.
     - libchannelizer skips the NCO for a centred channel (bit-exact, tested): -0.06 G / about 0.
     - No second ring.
     - **Total:** 5.95 G / 3.58 G old, 7.02 / 4.43 s1, 5.46 / 3.41 new.
     - getrusage CPU is not a usable yardstick here: the same binary gave 0.9 s and 1.8 s, because the clock went between 1.9 and 4 GHz (`powersave`). Compare cycles and instructions.
   - **Flush after a retune:** `sdr_rx_settle()` in libsdr. It drops the ring until 50 ms after the retune returned.
     - Measured on the B210 (`s2/retune_latency_trace.log`): a retune within a band takes 3 ms, to another band 100–120 ms. For as long as the call takes, the B210 delivers exact zeros (no gap reported). The new frequency is in the ring 1–3 ms after the return, and settles in 2.5 ms.
   - **Auto mode:**
     - **Rate:** `sdr_rx_widest_rate()` (device range, USB speed of that device from sysfs, half of the practical throughput), no wider than the band, and a per-device table entry in main.c. The B210 gets at most 8 MS/s for the CPU: at 16 MS/s the reading took 17–20 % and one channel 17 %, so only 1 channel per window fitted.
     - **Usable part per device** (SDR-FRONTENDS.md): B210 ±0.40, RTL ±0.85 MHz, Lime ±0.30, unknown ±0.35. The window centre is on the half-channel grid, so no carrier is within 100 kHz of the DC.
     - **Channels:** separated by libsdr's engine: the bank from 4 channels, the chain below (chain-only engine added to libsdr). The cost per channel and of the reading is measured in the first second. The budget (`--cpu`, 50 %) covers the whole program with 15 % kept free.
     - **Stop:** 3 confirmed towers within 0.1 ppm of the median.
     - **Output:** one line per carrier. JSON has the same fields.
     - **Deleted:** the polyphase path and the pre-validation and family code.
   - **Bench (one 10 s capture, 16 MS/s, offline), CPU of the separation:**
     - Chain at 16 MS/s: 13.4 % per channel.
     - Bank at 16 MS/s: 15 % plus 6.5 % per channel (1 / 5 / 10 / 20 channels: 21 / 45 / 82 / 151 % vs 13 / 67 / 135 / 286 %).
     - At 8 MS/s: chain 7.3 % per channel, bank 10.5 % plus 4.5 % per channel.
     - Offsets bank vs chain on 76 / 103: within 0.0005 ppm.
     - In the bank, the tail's rational resampler costs 27 % of a band-900 run (`resamp_process`), the bank 21 %, the FCCH detectors 12 %, and SCH/BCCH below 1 %.
   - **Results:** one interleaved loop, old, s1 and HEAD f2b73d4. B210, RX2, gain 50, auto rate (8 MS/s for bands, 1 MS/s for one ARFCN). 0 overflow/gap lines in every HEAD run, also under `perf record`. Time: to the result. CPU: getrusage, with cycles from `perf stat`.

     | Run | old ppm, time, CPU, cycles | s1 ppm, time, CPU, cycles | HEAD ppm, time, CPU, cycles |
     |---|---|---|---|
     | ARFCN 76 | 4.397, 10.9 s, 1.56 s, 3.55 G | 4.398, 11.1 s, 1.99 s, 4.48 G | 4.396, 4.9 s, 1.38 s, 3.06 G |
     | ARFCN 103 | 4.387, 10.9 s, 1.58 s, 3.52 G | 4.388, 11.4 s, 2.00 s, 4.63 G | 4.391, 5.0 s, 1.45 s, 3.11 G |
     | ARFCN 715 | 4.409, 23.5 s, 2.98 s, 5.67 G | 4.409, 23.7 s, 3.84 s, 7.43 G | 4.409, 5.4 s, 1.47 s, 3.18 G |
     | --band 900 | 4.397 (1 carrier), 15.1 s, 2.59 s, 5.32 G | 4.396, 26.2 s, 6.62 s, 12.5 G | 4.394 (3 towers agree), 5.3 s, 2.90 s, 10.9 G |
     | --band 1800 | 4.419 (1 carrier), 54.5 s, 7.40 s, 16.4 G | 4.407, 33.4 s, 6.69 s, 12.8 G | 4.413 (2 agree, "only 2 of 3"), 47.0 s, 17.0 s, 68.2 G |
     | --band 900 --full | – | – | 4.402 (5 agree, 49 flagged), 32.9 s, 13.0 s, 52.7 G |
     | --band 900 -j | – | – | valid JSON, 18.9 s |
     | RTL ARFCN 76, now | – | -0.895, 11.4 s | -1.082 (1.024 MS/s, 5.5 s); -0.975 (1 MS/s, 4.9 s) |

   - **Band scans cost more CPU than old** (decided, see the user's go below). One ARFCN is faster and cheaper. A band gets 3 towers instead of 1, about 3× faster on 900, at 2× (900) to 4× (1800) the cycles.
     - On 1800 most of it goes to weak carriers without a BCCH, 1–2 per window on the chain (`perf`: chain channelizer 50 %, FCCH 9 %).
     - The budget at 50 % of a core including the reading gives 3 channels per window at 8 MS/s.

   - **ARFCN 49** (operator C, BSIC b3, LAC L3, CI C3) reads 4.42–5.43 ppm through the day, the same in every window position (tested at -3.1, -2.5, -0.1 and +3.1 MHz). The tower itself is off and drifting, and is flagged as disagreeing. A channelizer test with a synthetic tone at -3.1 MHz is exact.
   - **ARFCN 103, +2.4 Hz vs old:** this channel's FCCH offsets have σ 4–9 Hz and an outlier family about -1.6 kHz off (the bench: plain mean 4087 Hz, trimmed 4197 Hz). 2.4 Hz is about 1.6 standard errors, and new runs among themselves spread ±3 Hz. That is estimator noise on a noisy channel, not a tuning error. Both paths tune exactly to 955.600000 MHz (UHD log).
   - **Overflows at 16 and 32 MS/s** (instrumented, then the instrumentation removed; patch in `s2/overflow_trace.patch`):
     1. The read thread's idle sleep was the `sdr_open` interval of 1 ms. UHD's receive buffer, computed from UHD's B200 defaults (not read back), is 16 × 2040 samples: 2.0 ms at 16 MS/s, 1.0 ms at 32 MS/s. **Fixed:** osmocalibrate passes half a packet as the interval.
     2. At 32 MS/s a single `uhd_receive()` (timeout 0) sometimes blocks 1.12–1.16 ms inside UHD:
        - The ring was nearly empty, there were no page faults, and only a few involuntary switches.
        - The next call reports the overflow.
        - Seen only with the processing running: 1, 3 and 1 overflows in three `--band 1800` runs of 40–109 s.
        - Not seen reading only (2 × 20 s), and not under `perf record`.
        - At 16 MS/s the 2 ms buffer covers it: 0 overflows in every run after fix 1.
        - **Not explained:** one forced 32 MS/s ARFCN 76 run with fix 1 (`a3_rate32000000`, untraced) had 329 overflow/gap lines; the traced binary did not reproduce it. A candidate is processing slower than real time, the ring filling up; that is not shown.
     - **DONE (user's go, 7f2b980):** `num_recv_frames` for 100 ms at the rate, at most 1024, only from 32 on, RX only, user device args win.
       - **RX latency** (device time now minus the device time of the last sample received, every 0.2 s): unchanged.
         - ARFCN 76 at 1 MS/s: 0.76–1.05 ms mean with 16 frames, 0.76–0.93 ms with 50.
         - ERMES on air (`-k 9`, 960 kS/s, 45 s, TX+RX loopback): 0.70 ms with 16 frames, 0.67 ms with 48.
       - **TX advance:** 47.9 ms in both. 54 and 55 batches decoded, no fault lines.
       - The read-thread interval rule stays per program (osmocalibrate passes its own). libsdr is unchanged there: at ERMES's 960 kS/s the rule gives the same 1 ms, and above 1 MS/s it would add wakeups not measured for the other programs.
       - Priority and pinning: not done.
   - **Band-scan CPU, the user's go (7f2b980, 64e4346):**
     - **Bank tail:** a halfband instead of the rational resampler where the bin rate is 4 × the DSP rate. Same band edges, 105 dB. Worst alias -122/-116 dB against -106/-104 dB with the resampler. Separation at 10 channels, 8 MS/s: 55.4 → 49.7 % of a core. Other rates keep the resampler, and `test_sdr_engine` output is identical.
     - **Weak carriers:** a carrier ends when its first 5 FCCH have a mean SNR below 10 dB (confirmed ones: 12.7–14 dB, carriers without a BCCH: 6.5–8.4 dB). After 8 carriers in a row without a confirmed one, the band's weaker rest is left out; `--full` measures all. A carrier with 20 offsets after 6 s ends.
     - **Rate:** no fixed cap. The SDR opens at the widest rate the device and bus carry, and the first window is a 1 s probe that measures the reading, separation and detection apart. Then the widest rate getting at least 3/4 of the most carriers per window is chosen, and the SDR is reopened at it (about 3 s on the B210). The B210 opens at 16 MS/s (900) or 32 MS/s (1800) and chooses 8 MS/s.
     - **Windows** are centred for the strongest carriers not yet measured.
     - **Results** (ppm / time / cycles), old → HEAD 64e4346:

       | Run | old | HEAD 64e4346 |
       |---|---|---|
       | ARFCN 76 | 4.397 / 10.9 s / 3.5 G | 4.392 / 4.9 s / 3.1 G |
       | ARFCN 103 | 4.387 / 10.9 s / 3.5 G | 4.374 / 5.2 s / 3.2 G |
       | ARFCN 715 | 4.409 / 23.5 s / 5.7 G | 4.402 / 5.1 s / 3.2 G |
       | `--band 900` | 4.397 (1 carrier) / 15.1 s / 5.3 G | 4.389 and 4.387 (4 and 3 towers) / 16.6 and 11.0 s / 25 and 17 G |
       | `--band 1800` | 4.419 (1 carrier) / 54.5 s / 16.4 G | 4.405 (2 towers) / 38.6 and 26.5 s / 62 and 40 G |
       | `--band 900 --full` | – | 5 towers / 53 s / 78 G |

       0 overflow/gap lines throughout.
   - **Open:**
     - Band scans still cost 3–4× old's cycles, for 2–4 towers instead of 1. The 900 start costs about 5 s: the scan, a 1 s probe at 16 MS/s, and the reopen of about 3 s. Changing the rate without a reopen would need a libsdr rate call.
     - **A per-channel DSP rate below 500 k: tried 2026-11-28, does not pay, not committed.**
       - **Method:** offline, one 12 s capture at 8 MS/s around 952.5 MHz with ARFCNs 76, 99 and 103 (the capture is deleted). `s2/bench/bench_rate.c` runs libsdr's engine plus fcch_detect and gsm_sch as osmocalibrate does.

         | DSP rate | Chain, 3 channels: separation / detection per channel | Bank, 4 channels: separation | 76 / 99 / 103 ppm (sd Hz) | SCH good / BCCH good |
         |---|---|---|---|---|
         | 500 k | 22.9 % / 2.5 % | 23.9 % | 4.4140 / 4.4100 / 4.4052 (1.8 / 2.9 / 9.3) | 240–249 / 48–50, all confirmed |
         | 400 k | 25.1 % / 2.4 % | 22.9 % | 4.4193 / 4.4137 / 4.4127 (2.3 / 3.6 / 10.1) | 244–249 / 49–50 |
         | 320 k | 23.8 % / 2.4 % | 21.0 % | 4.4185 / 4.4066 / 4.4076 (2.2 / 6.6 / 12.9) | 249 / 50 |
         | 300 k | 23.2 % / 2.3 % | – | 103 breaks: 1.15 ppm, sd 4.4 kHz | 233 / 47 |
         | 280 k | 22.9 % / 2.1 % | – | 103 not decoded at all | 0 / 0 |

       - **Cycles of the whole bench**, 500 k → 320 k: bank 18.9 → 17.1 G (-10 %), chain 17.9 → 17.9 G (0 %).
       - **Why it does not pay:**
         - The cost is at the SDR rate (the NCO and first halfbands, and the bank), not at the DSP rate.
         - Detection barely changes (2.5 → 2.1 % per channel).
         - osmocalibrate mostly runs the chain with 3 channels.
         - The spread grows (99: sd 3.0 → 6.6 Hz at 320 k), and below 320 k 103 fails.
     - **RTL is mostly drift:** s1 read -0.40 yesterday and -0.895 today; HEAD reads -0.975 to -1.08 today, 0.1–0.2 ppm from s1 (one run each). The confirmed offset is not verified against kalibrate-rtl today.
     - The generic-code inventory (what moves to libsdr) is in the slice 2 report.
3. + 4. **DONE 2026-11-28 (99ebf58; CPU 32e12f8; candidate fix 910f166).** Work files are in `~/.cache/sdrangel-rp-ermes/s3/`: `stats.txt`, `runs/`, `bench/`, `fake/`, and the prototypes `p6.py` and `pssw.py`.
   - **Bands:**
     - GSM from TS 45.005 Table 2-2: 450, 480, 750, 850, P-, E- and R-900, 1800 and 1900. The old E-GSM mapping put ARFCN 975 at 925.0 MHz; it is 925.2. GSM 750 has no fixed numbering, so it uses 438–511.
     - LTE: every band in `lte_bands`.
     - `--band` takes a comma list: `900,lte3`, `gsm`, `lte`, `all`.
     - **Reach** comes from `sdr_query_caps` plus the upconverter:
       - A band given by name that is out of reach is refused with both ranges, e.g. RTL `--band 1900`: "PCS-1900 (1930.2-1989.8 MHz) is out of reach: the device receives 24.0-1764.0 MHz"; `lte7` the same.
       - `all` skips such bands and logs each one ("34 bands ... skipped" on the RTL).
       - A band partly in reach is cut to the part in reach.
       - No usable rate (one GSM channel needs ±300 kHz, one LTE carrier ±600 kHz) is an error.
   - **One spectrum per run.**
     - 25 kHz cells, each from the scan step nearest to it, with the DC strip (±10 kHz) left out.
     - At least 20 ms averaged per step (for PSS/SSS/PBCH).
     - Step centres sit 50 kHz off the 100 kHz raster.
     - Overlapping bands, and GSM and LTE, scan it once.
   - **LTE candidates.**
     - **Rank:** the median of the centre 6 RB cells per EARFCN. A median, so a GSM carrier doesn't lift it. Then the local peaks (±5 EARFCNs, at least 2 dB above the 10th-percentile floor), plus the middle of any flat top wider than 2 MHz (loaded carriers).
     - **Measure** (910f166): each peak with 4 EARFCNs: the peak, its two neighbours, and the one 2 off on the side the top extends to. On a loaded carrier the peak lands up to 2 EARFCNs from the centre (band 3: peak 1498, cell 1500). The chain holds 4, and no bank fits 1.92 MS/s channels (`sdr_engine_bins` 0 at 3.84–15.36 MS/s).
     - **Rule from the captures:** an idle cell shows up as a 1 MHz block (its centre 6 RB), a loaded one as a flat block.
     - **Rejected:** a DC-subcarrier notch detector (false dips, missed the weak cells) and a wideband differential PSS. A ZC sequence differentiates to a pure tone, so that detector has no frequency selectivity and missed 5 of 9 cells.
   - **LTE windows.**
     - One peak's EARFCNs are cut out at 1.92 MS/s on libsdr's engine, one `lte_cell` each.
     - The LO sits 750 kHz off the carrier where the rate allows. On the RTL it sits on the carrier's empty DC subcarrier.
     - Channel starts are staggered by 30 ms, so searches don't coincide.
     - A cell counts when its MIB confirms it.
     - Once any tower is confirmed, the search is ±(1.5 ppm + 2 kHz) around its ppm; before that, ±20 ppm (at most 40 kHz). An empty window then takes 0.2 s.
     - A cell with `cfo_alias_margin` < 3 counts 0.25 in the mean and not as a tower.
     - A band ends after 16 peaks in a row without a confirmed cell (910f166; 6 before). The power rank is weak: UMTS and loaded carriers outrank idle cells, and it moves run to run with the traffic. 1500 came 4th in one run and 10th in another; on the RTL, 3575 came 7th.
     - An EARFCN is searched once per run, even where bands overlap (1 in 65, 7 in 41).
   - **Rates.**
     - With LTE in the run, rates are 1.92 MS/s × 2^k, so LTE needs no resampler. Separation at 7.68 MS/s costs 2.79 G cycles per 10 s of signal, against 5.07 G at 8 MS/s. GSM then runs at 480 kS/s per channel.
     - The RTL runs at 1.92 MS/s.
     - An LTE-only run scans at the widest rate (30.72 MS/s on the B210), then reopens at ≤ 8 MS/s.
   - **RTL:** the result is the LTE frame-timing ppm, marked "(timing)" and explained in the output (section E). `--sdr-ppm` is subtracted from it, because it is a whole-crystal figure.
   - **Output:** one line per LTE cell: band, EARFCN, MHz, PCI, FDD/TDD, bandwidth, ports, offset Hz, ppm, RSRP dBFS / RSRQ / SINR. JSON has the same fields, plus `system`, `ppm_from`, `carrier_ppm`, `timing_ppm`, `alias_margin` and `weight`.
   - **Results** (B210 RX2 gain 50 internal, RTL gain 40; cycles from `perf stat`; 0 overflow/gap lines in every run):

     | Run | Result | Time / CPU / cycles |
     |---|---|---|
     | Run (binary) | Result | Time / CPU / cycles |
     |---|---|---|
     | B210 `--band all --towers 20` (32e12f8: 3 EARFCNs per peak, cut 6) | 4.421 ppm, **11 towers within 0.024**: GSM 76, 1018 (E-GSM), 122, 99 at 4.410–4.412; DCS 715, 868, 514 at 4.420–4.435; LTE 3/1500, 1/250 (new: PCI p7, 10 MHz), 8/3575, 8/3676, all four at 4.427 | 200 s / 85 s (42.7 %) / 343 G; 406 LTE windows. Not rerun with 910f166 (more windows per band) |
     | B210 `--band lte3` (910f166) | 4.425 ppm, EARFCN 1500 PCI p4, 100 RB, alias margin 6.5 | 12.4 s / 9.4 s (**76 %**) / 38.0 G. The 32e12f8 binary missed it (peak 1498, cut 6) |
     | B210 `--band lte1,lte7,lte40 --full` (32e12f8) | 4.426 ppm: only EARFCN 125 (PCI p5, 15 MHz) | 31.8 s / 25.8 s (81 %) / 110 G |
     | B210 `--band 1900` (32e12f8) | clean "no confirmed tower": 6 carriers above the floor, none with FCCH | 17.5 s / 6.9 s / 25.2 G |
     | B210 `--band 900 --arfcn 76` (32e12f8) | 4.403 ppm | 4.6 s / 2.0 s / 4.4 G |
     | B210 `--band 900,lte8 -j` (32e12f8) | valid JSON; 3 GSM towers 4.393–4.407 (stops at 3) | 10.9 s / 18.8 G |
     | RTL `--band lte8` (99ebf58, 2.56 MS/s) | -0.995 ppm (timing ± 0.020), 3575; carrier -1.081 | 8.2 s / 3.6 s / 13.2 G |
     | RTL `--band lte8` (910f166, 1.92 MS/s) | -0.403 ppm (timing), 3575; carrier -0.455 | 13.3 s / 6.0 s / 23.8 G |
     | RTL, about 10 min later: `--arfcn 76`; `lte8` at 2.56 MS/s | GSM 76 carrier -0.591; LTE 3575 / 3676 timing -0.299 / -0.301 (± 0.020 / 0.009), carrier -0.465 / -0.389 | 5.4 s; 12.6 s |
     | RTL `--band 1900`, `--band lte7` | refused, 1.3 s; `--band all`: "34 bands out of the device's reach (24.0-1764.0 MHz) skipped", each listed | – |

     - **The RTL moved by about 0.6 ppm within about 1.5 h:** timing -1.0 → -0.3/-0.4, and GSM 76 carrier -0.98 → -0.59, all together. That is the device (a warm-up or temperature drift), not the method: carrier and timing move together. Between runs minutes apart the timing differs by 0.1 ppm, against a stated ± 0.02. The timing standard error understates the run-to-run spread on the RTL.
     - **LTE windows ignore `--cpu`:** 72–81 % of a core in LTE-only runs. That covers the 30.72 MS/s scan, 4 chains at 7.68 MS/s, and searches without a prior (±20 ppm). GSM runs stay at 33–48 %.

     - Against the earlier data: LTE 4.427 today vs 4.4235–4.4251 on 2026-11-27; GSM 4.410–4.435 vs 4.39–4.41. The whole set drifted up about 0.005–0.02 ppm since then (warm), and they agree with each other to 0.024 ppm.
   - **CPU cut (the user's extra task), 32e12f8:**
     - **perf on `--band 900`:** halfband 27 %, NCO 13 %, FCCH 9 %, channelizer glue 8 %, USB kernel 7 %.
     - **(a) a narrower SDR rate per window, (b) one shared first decimation: not built, not measured.**
       - Windows with 3 carriers span about ±2.9 MHz at 8 MS/s; there the band can't be halved before the channels split.
       - But 3 of the 5 windows on 1800 held only 1–2 carriers, where a narrower rate would pay.
       - (a) needs a rate change without a reopen (a libsdr call; today a reopen costs about 3 s).
       - (b) needs a shared stage in the engine, opt-in so other programs stay bit-exact.
     - **(c), bit-exact, in libchannelizer:**
       - The decimating halfband splits the block by parity and computes two outputs at a time (vector extension), summing in the same order as before.
       - The RX NCO converts the float IQ in its own pass.
       - The NCO itself is the serial phasor chain, latency-bound at about 8 cycles per sample per channel. It can't get faster without changing its numbers, so it is left alone.
     - **Cycles per 10 s of signal, one channel** (`s3/bench/bench_chan.c`, output checksums equal):

       | Separation | Before | After | Change |
       |---|---|---|---|
       | 8 MS/s → 500 k | 3.36 G | 2.71 G | -19 % |
       | 16 MS/s → 500 k | 6.19 G | 5.22 G | -16 % |
       | 7.68 → 1.92 MS/s | 3.78 G | 2.79 G | -26 % |
       | 8 → 1.92 MS/s | 5.81 G | 5.07 G | -13 % |

     - **Checks:** `test_channelizer` and `test_sdr_engine` give the same output. The fake-SDR osmoradio suite (`s3/fake/`) has 0 cases differing.
     - **Whole runs, old → before (64e4346) → after** (towers vary run to run, so per tower):

       | Run | old | before (64e4346) | after |
       |---|---|---|---|
       | `--band 900` | 5.3 G (1 tower) | 14.8 G, 3 towers: 4.9 G per tower, 10.0 s | 22.0 G, 4 towers: 5.5 G per tower, 16.4 s (one window more) |
       | `--band 1800` | 16.4 G | 37.9 G, 2 towers: 19 G per tower, 27.6 s | 25.3 G, 2 towers: 12.6 G per tower, 21.4 s |

       - "After" is 32e12f8: it includes 99ebf58's new scan, which picks a different carrier list (25 vs 20 on 900, 8 vs 12 on 1800). So these whole runs mostly show the scan change, not the libchannelizer change; `--band 900` looks worse per tower for that reason.
       - The bench is the clean figure. A clean whole-run pair (99ebf58 against `s3/fake/old/libchannelizer_old.a`) was not run.
   - **Open:**
     - **Missed LTE cells (the main open point).** Of the 9 known cells, these were not found:
       - band 1 / 499 (also not found with `--full` on 32e12f8);
       - band 7 / 3025 (SINR -2 dB);
       - band 40 / 39300 and 39498 (-2 dB);
       - band 3 / 1275 (-3 dB).

       Band 1 / 125 was found with `--full`, and 1/250 is new.
       - The weak ones sit 2–4 dB above the floor in the 20 ms spectrum. Band 7's windows went to 2646.0 / 2646.3, 1.5 MHz from 2647.5.
       - The 6 RB power rank is the weak link. A sharper rank would help: longer averaging, the 1.08 MHz block's edges, or a cheap PSS pre-check. The `pssw.py` prototype (differential PSS) has no frequency selectivity, so it is not that.
       - `--band all` was not rerun with 910f166's candidate set.
     - **`--band all` takes 200 s:** 406 LTE windows over 69 bands, most of them empty or noise peaks.
     - **Junk in GSM bands:** 450/480/750 are empty here, but each still measures its noise carriers until 8 in a row fail.
     - **The default `--towers 3` stops inside the first band** that gives 3 (GSM-900). So `--band all` reaches LTE only with a higher `--towers` or `--full`.
     - **RTL:** GSM carrier ppm (PLL step) and LTE timing ppm differ by up to about 0.5 ppm, so mixed GSM+LTE runs on the RTL may not agree within 0.1. The GSM side should also use timing (TDMA frame drift); not done.
     - **SIB1/PLMN for LTE:** not done.
     - **The sign of `--sdr-ppm` subtracted from the timing ppm** follows the carrier convention; it is unverified (B.4 is still open).
     - **Halfband work buffers** grow to the block size (50 ms at the SDR rate), allocated in the feeding thread on the first block.
5. **DONE 2026-11-28 (the user's order).** Commits: 15fbdb4 (libsdr), 3f53012 and 9424726 (osmocalibrate). Work files in `~/.cache/sdrangel-rp-ermes/s3/`: `ab_out.txt`, `gt/`, `coh_c.py`, `fake/rate/`, `bench/bench_shared.c`.
   - **libsdr `sdr_set_rx_rate()`, no reopen.**
     - Scope: a band receiver whose band passes unchanged (channels = 0, DSP rate = SDR rate, no LO offset), RX only, at most the rate it was opened at. Its buffers and UHD's `num_recv_frames` are sized for that rate.
     - Steps: the read thread is stopped, then the stream is stopped and drained. Rate and analog bandwidth are set. On UHD the frequency is set again (the master clock may move); on Soapy, time stamps stay only where ns per sample is an integer. The stream starts again and the ring is emptied; `sdr_rx_settle()` counts from the change.
     - Anything else gets `-ENOTSUP` with the reason.
     - **Time on the B210:** 16 → 8 MS/s 461 ms, 30.72 ↔ 3.84 MS/s 388–461 ms, against about 3 s for a reopen.
     - **Fake-driver test** (`s3/fake/rate/`: the real `sdr.c`, fake UHD and Soapy): 7.68 → 3.84 → 1.92 → 4 → 7.68 MS/s. The tone stays at +300 kHz, its phase is continuous in every read (worst step error 7e-8 rad), the rates and bandwidths come out as set, and a rate above the open's is refused. PASSED for UHD and Soapy.
   - **Engine `sdr_engine_group_add_shared()`, opt-in.**
     - The RX chain channels share the NCO to their middle and the halfbands down to the narrowest rate that holds all of them. Without room it falls back to the plain chain; a bug there (offsets kept relative to the middle) is fixed.
     - **Bench:** 3 GSM channels within ±0.5 MHz at 7.68 MS/s cost 18.3 → 11.7 % of a core, with the same levels and residual.
     - **Checks:** `test_sdr_engine` gives the same output, and the fake-SDR osmoradio suite (engine before vs after) has 0 cases differing.
   - **Window stages, one build, interleaved** (`--window-stage`, cycles; 0 overflow/gap lines in all 12 runs):

     | Run | none | rate | shared |
     |---|---|---|---|
     | `--band 900` | 16.1 / 15.8 G (3 towers, 8.0 s) | 16.4 / 21.6 G | 17.3 / 16.0 G |
     | `--band 1800` | 21.1 / 33.9 G (2 towers) | 13.1 / 57.3 G (the second run: 3 towers, 47 s, another carrier list) | 32.5 / 22.1 G |

     - ppm 4.388–4.399 (900) and 4.410–4.417 (1800); the same towers.
     - On 900 neither stage pays: the windows span the whole usable band.
     - On 1800, rate cut the like-for-like pair by 38 %. The spread between runs is the carrier list, not the stage.
     - **Default: `rate`.** The rate chosen after the cost probe is also changed without a reopen now.
   - **LTE candidates from the 10 ms repetition.**
     - **How:** per scan step, 40 ms more of IQ gives the coherence of X(t) and X(t + 10 ms) per 25 kHz cell. CRS, PSS and SSS repeat every frame across the carrier's occupied width; the centre 6 RB repeat less, because the PBCH changes every frame.
     - **Rules:**
       - Blocks, and block pairs across that centre dip, of an LTE width (15–100 RB, tolerance 0.4–1.1 MHz) give their middle, plus both edges if they are wider than that width.
       - Other blocks give their middle and edges.
       - The power peaks follow.
     - **Two detectors failed first:**
       - Differential PSS: no frequency selectivity.
       - PSS repetition at 5 ms: cells 3575 and 1500 send the PSS alternately, coherence 0.40 at 5 ms against 0.97 at 10 ms.
     - **Today's ground truth** (blind scan, `lte_cell` on every EARFCN of 17 captures of 0.2 s, bands 1/3/7/8/40, at 08:30 EET; captures deleted): 125, 250, 499, 1275, 1500, 3025, 3575, 3676, 39498.
       - Absent against yesterday 23:00: 39300 (but osmocalibrate found it at 08:37 and 08:45).
       - New: 250 (PCI p7, 10 MHz).
     - **Offline** (`coh_c.py`, the same rules, on the same captures): the coherence alone gives 250, 499, 1500, 3025, 3575, 3676 (6 of 9). 125 (SINR 2.7 dB), 1275 (0 dB) and 39498 (-4.6 dB) barely repeat above the noise.
     - **On air, `--band lte1,lte3,lte7,lte8,lte40 --full`:**
       - 08:37 (3f53012): 9 cells, 8 of the ground truth (not 250) plus 39300. 73.7 s, 76 % of a core (before the CPU limit), 146 windows.
       - 08:45 (9424726): 8 cells, 7 of the ground truth (not 125, 1275) plus 39300. 100.7 s, 51 %, 176 windows.
       - **No false cell:** every cell is MIB-confirmed with the ground truth's PCI.
       - The weak ones (SINR ≤ 3 dB) come and go between runs.
       - 39498 read 4.592 (alias margin 1.8, low weight) and was flagged as disagreeing.
     - **`--band lte3`:** 1500 found, 12.9 s.
   - **LTE inside `--cpu`:** between LTE windows the band idles (ring drained) while its CPU share is above the budget. `--full` LTE went from 76 % to 51 %; `--band lte3` used 61 %, because the 30.72 MS/s scan and the rate changes are not counted.
   - **GSM frame timing** (`gsm_sch`: SCH burst position against frame number, a regression). `--ppm-from carrier|timing`, timing by default on the RTL; in timing mode the SCH runs until 0.02 ppm (10 s at most).
     - **B210:** GSM timing 4.404–4.426 (± 0.02, 101–125 SCH) against carrier 4.406–4.411; LTE 8 timing 4.401. The method holds.
     - **RTL, mixed `--band 900,lte8`:** GSM timing -1.08 / -0.99 against LTE timing -0.62. Alternating runs a minute apart read GSM -0.85, LTE -0.68, then LTE -0.47.
     - So the RTL crystal drifts about 0.2 ppm a minute while it is in use (warm-up). Agreement on the RTL can't be shown until it has settled.
   - **Both kinds of towers:** with GSM and LTE both asked for, a result needs agreeing towers of both kinds. Once enough GSM towers agree, the other GSM bands are skipped. `--band all` with the default `--towers 3`: GSM 1018, 76, 122 plus LTE 1/499; 4.413 ppm, 4 towers within 0.021, 21.8 s, 28 % of a core.
   - **Open:**
     - Weak LTE cells (SINR ≤ 3 dB) are found in some runs and not others.
     - The LTE CPU limit doesn't count the scan.
     - The RTL drifts during use.
     - The rate stage doesn't help when a window spans its whole band (900).
     - The shared stage is kept as an opt-in.
   - **The plan as written before (3: bands; 4: LTE below):**
   - GSM (45.005): 450, 480, 750, 850, P/E/R-900, 1800, 1900.
   - LTE (36.101): all FDD and TDD bands with EARFCN.
   - Reach comes from `sdr_query_caps` plus the upconverter: "you never know which SDR or upconverter tomorrow".
   - **Refusal:**
     - A band asked for explicitly that's out of reach is refused at start, with the numbers.
     - `all` scans only what's reachable and lists the skipped bands.
     - No band reachable, or no usable rate: error.
4. **LTE.**
   - PSS/SSS give the PCI and the CFO.
   - The MIB (PBCH, CRC-16 with the antenna-port mask, SFN advancing) is the proof of a real cell.
   - PLMN (MCC/MNC) would need SIB1: PDCCH plus turbo-coded PDSCH. Big, so optional and later.
   - Quality: RSRP, RSRQ, SINR.
   - LTE-Cell-Scanner and srsRAN are AGPL-3: reimplement from 3GPP TS 36.211/36.212, using the scanner's logic as a guide.
   - Not tested yet: Soapy, the RTL (reference -0.33 ppm), and the `--channelizer` path.
   - **LTE results (2026-11-27, commit 1d63033, module only, not yet in osmocalibrate):**
     - **Code:** `src/libcalibrate/lte_cell.c/.h` (search, tracking, MIB, offset, quality), `lte_phy.c/.h` (PSS, SSS, CRS, Gold, CRC-16, tail-biting Viterbi, rate matching, MIB fields), `lte_bands.c/.h` (36.101 Table 5.7.3-1, all bands), `src/test/test_lte_cell.c` (self test plus captures).
       - Written from the 3GPP texts in `~/.cache/sdrangel-rp-ermes/lte/spec/`. No library code.
       - Input: one carrier at 1.92 MS/s from the SDR layer (sdr_engine or libchannelizer). No radio access, no resampler.
     - **Confirmation:** two or more MIBs whose CRC passes with the antenna-port mask, whose re-encoded bits agree with the received ones (≥ 0.35), and whose SFN advances by the elapsed frames, with the same bandwidth, PHICH and ports.
     - **Cells found** (B210 RX2, gain 50; RSRP in dBFS at gain 50):

       | EARFCN | MHz | PCI | Type | Ports / BW | RSRP / RSRQ / SINR |
       |---|---|---|---|---|---|
       | 3575 | 937.5 | p1 | FDD | 4 / 25 RB | -37 / -9 / 7 dB |
       | 3676 | 947.6 | p2 | FDD | 2 / 25 RB | -45 / -7 / 18 dB |
       | 1275 | 1812.5 | p3 | FDD | 2 / 75 RB | -59 / -11 / -3 dB |
       | 1500 | 1835.0 | p4 | FDD | 4 / 100 RB | -57 / -9 / 5 dB |
       | 125 | 2122.5 | p5 | FDD | 1 / 75 RB | -53 / -8 / 5 dB |
       | 499 | 2159.9 | p6 | FDD | 4 / 100 RB | -57 / -10 / 5 dB |
       | 39300 | 2365.0 | p4 | TDD | 4 / 100 RB | -65 / -10 / -2 dB |
       | 39498 | 2384.8 | p4 | TDD | 4 / 100 RB | -66 / -11 / -2 dB |
       | 3025 | 2647.5 | p4 | FDD | 4 / 75 RB | -68 / -11 / -2 dB |

       PCIs masked as p1–p7 (equal label = equal PCI).

       - **Band 8 does have LTE** (3575, 3676): the "no LTE in band 8" line above is wrong.
       - Bands 20, 28 and 38 had no cell.
       - **39498 is a second carrier, not a fold.** In one capture centred at 2375 MHz both carriers decode, with the same SFN and frame starts 0.03 µs apart (one synchronised TDD site). The spectrum is flat from 2361 to 2387 MHz (two contiguous 20 MHz carriers, 19.8 MHz apart), and RSRP and SINR differ. Its earlier +0.047 ppm was the 100 Hz alias bug, fixed before the commit; centred on it, it reads 4.4241 ppm.
     - **Agreement** (2 s captures at 7.68 MS/s, LO 2 MHz off the carrier, EARFCN cut out by libchannelizer; GSM ch 76 with `osmocalibrate.old` in the same session, before and after):

       | Clock | GSM ch 76 | LTE (5 cells / 4 bands, internal; 3 cells, external) | Frame timing drift |
       |---|---|---|---|
       | B210 internal | 4.405 / 4.402 | 4.4235 .. 4.4251 | 4.401 .. 4.448 |
       | B210 external | 14.769 / 14.768 | 14.787 .. 14.788 | 14.777 .. 14.788 |
       | RTL (R820T), 1.92 MS/s | -0.934 (own FCCH fit), kalibrate-rtl 0.886 abs. | -1.226 / -1.128 | -1.034 / -1.007 |

       - **B210:** the LTE cells agree within 0.002 ppm of each other. They sit 0.02 ppm above GSM ch 76 with both clocks, so that is the GSM tower's own offset (both are inside the ±0.05 ppm base station tolerance).
       - **RTL:** now about -1.0 ppm, not the -0.33 in the table above (it moved, or the old number is stale). Its carrier offset carries the R820T PLL tuning step (about 440 Hz, so up to ±0.2 ppm at 950 MHz, different per frequency). The frame timing drift (crystal only) is the clean RTL figure. A ppm tool on the RTL should take the timing ppm, or correct for the tuning step.
     - **Carrier offset method.** The CRS phase of port 0 is measured over three baselines, each resolving the ambiguity of the next:
       - 0.5 ms slot to slot (±1 kHz, coarse);
       - 5 ms subframe 0 to 5 (±100 Hz);
       - 10 ms frame to frame (±50 Hz, the finest).

       The timing drift is the same oscillator as the LO, so it joins the coarse stage when the carrier frequency is known. Receiver spurs on CRS positions are masked (more than 2× the median power).
       - Why: B210 spurs inside the channel (e.g. +424 kHz, 34 dB above the floor at LO 2649.5 MHz) pulled the slot-to-slot stage 70–80 Hz and made band 7 read 4.461 or 14.862 ppm (a 100 or 200 Hz alias).
       - **`cfo_alias_margin`** (in standard errors) says how safe the alias choice is. Band 7 at SINR -2 dB still has only 1σ (right now, thanks to the timing). The caller should down-weight cells below about 3, or ask for agreement between cells.
       - Precision once settled: ±0.02 Hz in 2 s.
     - **Negative control:** a blind scan of 3919 EARFCNs in 17 survey captures (0.15 s at 30.72 MS/s; bands 1, 3, 7, 8, 20, 28, 38, 40) confirmed exactly the 9 cells above and no false cell. Noise-only and weak (-8 dB) synthetic signals also gave no cell.
     - **CPU** (this PC):
       - The search is about 0.10 s per carrier, once, at ±40 kHz: 13 offset hypotheses at 7.5 kHz steps × 3 N_ID2 × 4 half-frames. At ±10 kHz it is about 0.045 s.
       - Tracking with a MIB every frame: about 0.5 % of a core.
       - An empty EARFCN: about 0.19 s (two searches of 20 ms each).
       - The channel extraction is not counted: it is libchannelizer or the bank.
       - For a band scan, pick candidates by power first (as for GSM), and use a ppm prior (from GSM or the first cell) to narrow `cfo_max`.
     - **Not done:**
       - SIB1 / PLMN.
       - Integration into osmocalibrate (slice 2 owns `calibrate.c`).
       - Soapy and the sdr_engine path (the module takes the same IQ).
       - The slot-to-slot bias source on weak cells (10–20 Hz) is not understood; the 5 ms stage makes it harmless.
       - libcalibrate.a did not build at commit time, because of the other agent's `calibrate.c` work in progress. The lte objects and test_lte_cell build without warnings through automake.
     - **Captures and tools:** `~/.cache/sdrangel-rp-ermes/lte/`: `cap/`, `xcheck.sh`, `extra.sh`, `scan2/`.

**Part A closed 2026-11-28.** Commits:
- af61683, 80145d8, e51b83a, d098423, f29423b, 24b38ff, ef00f4e, f2b73d4, 7f2b980, 64e4346: GSM, slice 1–2, the RX buffer;
- 1d63033: the LTE module;
- 99ebf58, 32e12f8, 910f166, 15fbdb4, 3f53012, 9424726: all bands, LTE in osmocalibrate, the rate change without reopen, the shared stage, window rates.

Accepted limits:
- Weak LTE cells (SINR ≤ 3 dB) come and go between runs.
- The LTE pre-scan isn't counted in `--cpu`.
- Window rates don't help on GSM-900.
- The RTL drifts about 0.2 ppm/min while in use: warm it up before calibrating. That's input for B.

Parked: PLMN (SIB1), other SDRs (section E).

## B. Time and clock monitoring: plan and state

**Part B done 2026-11-28** (commits 316a654, 8e51667, 92cad9d on top of 9424726). Work files: `~/.cache/sdrangel-rp-ermes/clk/`.

### Done

- **Before (6274aeb, 0c001f5):** `sdr_clock_status_t`, `sdr_clock_poll()` in the main loop, `uhd_clock_poll()`, and the ERMES NTP check. The uncommitted `device_drift()` experiment gave the device-time table above. It is removed; the diff is in `clk/device_drift_experiment.diff`.
- **316a654, libmobile.** At the end, one line: the main loop's passes, the longest, and how many took over 1, 2 and 5 ms (the first 5 s left out).
- **8e51667, libsdr: the control thread (`uhd.c`).**
  - **Locks.** `uhd_ctl_lock` is held around every control call while the device is open: the thread's reads, and `uhd_set_rx_frequency`, `uhd_set_tx_frequency`, `uhd_set_rx_rate` and the new `uhd_set_tx_ppm` from any thread. It is held for one call at a time, so a retune waits for one read at most.
  - **Results** go to a cache under a second lock, `uhd_status_lock`, which is held only for a struct copy. The main loop (`sdr_clock_poll()`, once a second) only copies, so it never waits on USB.
  - **Lifecycle.** The thread starts at the success exits of `uhd_open()`: after the TX probe, also after the USB-reset reopen, and for RX-only opens (osmocalibrate) too. It is stopped and joined first thing in `uhd_close()`, which is also the exit-3 path. It is stopped at the start of a second `uhd_open()`. It waits on a condition variable, so the join is immediate, and it leaves the device alone once TX is dead. All signals are blocked in it.
  - **What it reads.** `ref_locked` every 1 s. Every 10 s: `gps_locked`, the temperature and the device time (bracketed by REALTIME and RAW). The temperature is the `temp` sensor of the RX or TX frontend or the motherboard, whichever exists; on the B210 that is the AD9361, 37–40 °C here. Without a sensor it is NAN.
  - **Lock changes** are counted in the thread. The main loop logs one line when the counters change, at most once a second.
  - **RX time stamps.** `uhd_receive()` reads RAW and REALTIME after every recv (two vDSO calls). Each second, the packet end with the least delay goes to the status as a pair (no control call). The status carries both pairs; the monitor prefers the RX one while it is fresh (under 5 s old).
  - **`sdr_get_tx_ppm()` / `sdr_set_tx_ppm()`.** UHD only (others give -ENOTSUP). The trim tunes with the RF LO of the last tune kept (MANUAL policy) and the DSP taking the change. `tune_result` is now logged at debug level at the open (section E).
  - **No runtime gain setter exists** in libsdr, so there was nothing to lock there.
- **92cad9d, libmobile: the clock monitor** (`clock_drift.c/.h` is pure and has no I/O; `clock_monitor.c/.h`). It is started by `main_mobile_loop()` for every program on an SDR, with the program name and the first TX frequency.
  - **History.** A point every 10 s, a ring of 2048 (5.7 h).
  - **Fits.** Least squares against RAW (all points) and against UTC (only the newest run of NTP-synchronised points).
    - The UTC run restarts when sync is lost or the PC clock steps (more than 600 ppm × Δt + 1 ms between points).
    - The whole history restarts on a device-time jump.
  - **The bound against UTC** = 3 standard errors + 2·eps/span. eps is the kernel's esterror (at least 0.5 ms), or 5 ms assumed for timesyncd, which gives none.
  - **The needs table:**
    - ERMES: carrier ±200 Hz, cycles ±2 ms of UTC (§12.4);
    - NMT (carrier ≥ 800 MHz): ±0.26 ppm.

    Other programs get the status but no needs.
  - **The start line** (one line): reference, lock, time source, temperature, NTP state, the needs, `--sdr-ppm`, the trim mode. It adds a hint where needed:
    - no `--sdr-ppm`: osmocalibrate gives one;
    - a need NTP can never confirm in 5.7 h (NMT's 0.26 ppm): calibrate against towers.
  - **Warnings** (NOTICE, once each) only when certainly violated:
    - **Carrier:** |error| − bound > need.
    - **Cycle timing:** |rate| − bound > 0. It says after how many minutes the cycles pass ±2 ms, and what keeps them (a GPS-disciplined 10 MHz + PPS). `--sdr-ppm` is carrier only.
  - **At most one line of its own per 10 min.**
  - **On demand:** SIGUSR1 (`kill -USR1 <pid>`; the pid is in the start line). One line goes out at the end too.
  - **`--sdr-clock-log <file>`** writes one line per point (both pairs, NTP, temperature, lock) for offline analysis.
  - **The trim, `--sdr-clock-trim off|dry|on`** (off by default). It is open loop: residual = measured rate + `--sdr-ppm` + the trim so far.
    - A step is at most 0.1 ppm, at least 10 min after the last.
    - It needs a UTC run of 30 min or more, and |residual| − bound > |step|/2, i.e. certain to help.
    - Against the last direction it needs twice the bound.
    - `dry` logs "would trim by X ppm (… Hz) to Y"; `on` calls `sdr_set_tx_ppm()`.
  - **ERMES** uses the shared NTP state function (`clock_ntp_state()`).
  - **`--sdr-ppm` help fixed:** positive means a slow crystal, the value osmocalibrate prints (see the sign check below).
  - **`src/test/test_clock_drift`:** synthetic cases, `--ntp <recorded series>` and `--replay <clock log>`.

### Results (B210 30AA038, internal reference, ERMES ch 9 169.625 MHz, TX gain 50 into the RX loop, RX gain 10)

- **Main loop, 5 min runs:**

  | Run | Passes | Longest | Over 1 ms | Over 2 ms | Over 5 ms |
  |---|---|---|---|---|---|
  | base1 (316a654, old poll) | 1857102 | 3.59 ms | - | - | 0 |
  | base2 (316a654, old poll) | 1855992 | 3.69 ms | 389 | 389 | 0 |
  | new1 (thread) | 1855586 | 3.64 ms | - | - | 0 |
  | new2 (thread) | 1856772 | 3.61 ms | 389 | 389 | 0 |
  | new3 (thread) | 1856878 | 3.61 ms | 389 | 389 | 0 |

  - **A null result here.** The 389 long passes (about 1.3 per second, 3.6 ms) are ERMES building a batch (16 per 12 s), in both builds. The old poll never took 1 ms in these baseline runs, so it doesn't show in the pass statistics.
  - **On the thread's side,** a whole tick of reads (ref + GPS + temperature + time) took at most 3.5 / 4.6 / 4.0 ms. Those are the round trips that used to sit in the loop: 7.4 ms in the clk runs, 16 ms seen before. They are off the loop now, but this machine didn't show them in the baseline.
- **TX errors and clean exits:**
  - No TX write failure, underflow, RX gap or overflow line in any of the 5 runs.
  - All 394 looped-back batches per run decoded with 0 codewords in error, before and after.
  - Every run exited 0 on SIGINT.
- **Exit-3 path** (`-DTEST_TX_FAIL=1` build, sends fail from 5 s): "TX writes failed for 2 s … exit code 3". The main loop quits, the thread is joined, and the process exits 3 without hanging; the radio is off.
- **Retune and rate change with the thread running:**
  - `osmocalibrate --band all`: 46 retunes and RX rate changes. Each rate change took 52.8–54.5 ms, against 54.1–54.8 ms before.
  - No overflow or gap.
  - Result 4.371 ppm, 5 towers agree (spread 0.028).
  - 30 s and 38 % CPU, against 21.8 s and 28 % in the earlier note. That's more towers and bands scanned this time (GSM-450/750 energy, 5 towers), not the thread: the per-change cost is the same.
- **Fake-SDR suite** (`s3/fake/suite.sh`, osmoradio on fake_sdr.c with the new uhd.o): all 12 cases the same, 0 sanitizer errors. `test_sdr_lo/engine/bands/backend` pass.
- **`8e51667` builds on its own** (no warnings).
- **Drift, 3 × 5 min (RX time stamps):**
  - **Against RAW:** +40.05 / +40.10 / +40.16 ppm, each ± 0.4–0.6.
  - **Against UTC:** -4.43 / -5.14 / -5.44 ppm (statistically ± 0.4–0.6, but NTP's bound after 5 min is ± 35).
  - **The monitor's own end lines:** -4.35 / -5.11 / -5.35 ppm, the carrier -738 / -867 / -908 Hz "not known yet", the cycles -1.27 / -1.47 / -1.55 ms from UTC after 290 s.
  - **Matches the table above** (+40.16, -4.8 / -5.3).
  - **Scatter:** RX time stamps 0.07–0.09 ms, the device time reads 0.02–0.03 ms. The read is less noisy; the RX pair costs no control call. Both are far below NTP's error.
  - **No slope comparison between the two.** The read's raw time was not logged in these runs (it is now, `--sdr-clock-log` column 11).
- **Warnings in 5 min runs: none, as designed.** With timesyncd (5 ms assumed), 4.4–5 ppm becomes certain after about 35 min (2·5 ms / 4.8 ppm). Only then would the cycle-timing warning come, and the carrier warning (uncalibrated) at about 1.3 h.
- **The dry run, offline** (`test_clock_drift`, synthetic NTP: a 2 ms swing of 34 min plus 0.5 ms white; then the recorded NTP series of this PC, 1.1 h, repeated):
  - **Uncalibrated -4.4 ppm:** the first step after 0.66–0.74 h, then 0.1 ppm per 10 min (29 steps in 5.5 h, 39 in 8 h, the ring wrapping). Never past 0, no reversal.
  - **No step at all for:**
    - calibrated (`--sdr-ppm 4.4`);
    - a 0.3 ppm residual;
    - NTP's error as a ramp of ±5 ms over the run (the worst case of the bound);
    - an NTP step of 100 ms (seen once, the UTC run restarts);
    - a device-time jump (seen once);
    - 8 h calibrated.
  - **Other cases:**
    - NTP lost for an hour: no step until 30 min after it is back.
    - A ramp of 1 ppm over the run, 14.8 ppm (the external reference) and a fast +3 ppm clock: slow steps, the right direction, no reversal.
    - A kink (-4.4 → -1.0 ppm at 2 h, after `--sdr-ppm 4.0`): the fit over the whole history lags; the steps start at 3.5 h, no reversal.
  - **The recorded NTP:** what is left after its line (the PC crystal, +44.8 ppm against NTP) is only 0.18 ms rms, 0.41 ms peak in 1.1 h. The 5 ms assumption is very conservative; it costs time, not correctness.
- **Sign of `--sdr-ppm` (B.4) on the air.**
  - **The RTL:** warmed 10 min at 169.5 MHz. GSM ch 76 frame timing read +0.498 ppm before and +0.520 after (osmocalibrate's sign).
  - **The ERMES carrier:** 20 s RTL captures. The four nominal levels (±1562.5 / ±4687.5 Hz) fitted to the FM histogram; the plain mean is biased by the idle symbol mix.

    | Run | Carrier (RTL raw) | RTL's 0.50–0.52 ppm taken out |
    |---|---|---|
    | A1, no `--sdr-ppm` | -702 Hz | about -788 Hz (-4.64 ppm) |
    | B, `--sdr-ppm 4.4` | +38 Hz | about -48 Hz (-0.28 ppm) |
    | A2, no `--sdr-ppm` | -702 Hz | about -788 Hz |

  - **The sign is right:** `--sdr-ppm +4.4` raises the carrier by 740 Hz (745 by the plain mean), against 746 expected, and the error goes from about -790 to about -50 Hz. A1 and A2 repeat to 0.1 Hz.
  - **So positive `--sdr-ppm` means a slow crystal,** as osmocalibrate prints it. The code (`f × (1 + ppm)`, the same in uhd.c, soapy.c and rpitx) was right; the help text was reversed and is fixed. Only UHD was measured on the air.
  - **The absolute value** is 45 Hz further off than osmocalibrate's 4.371 ppm (-741 Hz) predicts. That is the RTL: its R820T PLL step at 169.5 MHz is unknown (section E), and its timing ppm moved 0.02 during the test.
  - Files: `clk/sign.sh`, `cap_*.u8`, `sign_levels.txt`, `rtlcal_*.log`.

### Open

- **Real trim (`--sdr-clock-trim on`) never ran on a device.** Before enabling it, check on the loopback ("codewords in error") whether a MANUAL-RF tune to the same LO re-locks the AD9361 or makes a TX gap. Only the dry-run logic is proven (offline).
- **Untested:**
  - the lock-change line (no lock change happened in any run);
  - a control call hanging on a really broken transport (TEST_TX_FAIL only fails the sends; the join would wait for UHD's own control timeout);
  - the NMT need (no NMT run).
- **The RX time-stamp pair** is the least-delay packet of each second. Its scatter (0.08 ms) is 3–4 × the bracketed read's. It could take a longer window with the trend removed, but that's not needed while NTP's bound dominates.
- **Longer recordings:**
  - The NTP series covers 1.1 h (2 timesyncd polls of 34 min). A longer one would show timesyncd's frequency steps.
  - The kernel's frequency estimate (+44.36 ppm here) against the device vs RAW (+40.16) gives the device against UTC (-4.2 ppm) at once, close to the towers' 4.37. It is not used, since its error isn't known.
- **ERMES cycle re-lock (C.19)** is still to do; the monitor only warns (after about 35 min with timesyncd).
- **The kink case:** the fit over the whole history reacts slowly to a real rate change. That's acceptable for "very slow" steering; a windowed fit would follow faster.

### Requirements (user)

- **Lab, best effort, save CPU.**
- **Warn**, for example when a protocol needs precision and the clock is the PC's, and propose what to do.
- **Lock changes** are counted and logged at most once a second, only when they change.
- **Logging:** rare, and on demand; don't scare the user or pollute the log.
- **Corrections** only after a dry run that proves them with stats. Very slow and careful long-term steering, no oscillation; correct only if it's mathematically certain to help.
- **Common for all protocols.** Each protocol declares what it needs: ERMES ±2 ms cycle timing to UTC (§12.4), carrier ±200 Hz; NMT-900 ±0.26 ppm; and so on.
- **Temperature:** read for reference only.
- **Short-term PC instability** (jumps, phase noise) is not steered.

### Plan (as approved; all done, see Done and Results above)

1. **Control thread in libsdr** (the refactor session's 4 conditions):
   - One mutex around every control call: the thread's reads and the main thread's set_freq and set_gain.
   - Start after open, probe and reopen; stop and join first thing in `uhd_close`, including the exit-3 path.
   - Safe cache handover: the main loop only copies the cached status under the lock.
   - Proof runs: 3 × 5 min plus a retune. Report the main loop's longest pass, TX errors and a clean exit.
   - Reads: `ref_locked` every 1 s; GPS lock, temperature and device time every 10 s. Prefer RX time stamps for the drift if available.
2. **Shared clock monitor in libmobile.**
   - A small history, and drift fitted against RAW and NTP (only while NTP is synchronised).
   - One status line at the start, and a warning only when a protocol need can't be met.
3. **Correction, approved 2026-11-27: "maybe it can but after some time".**
   - Only after 30 min or more, with NTP synchronised and the offset well beyond its uncertainty.
   - A carrier trim (`--sdr-ppm`-like) of at most 0.1 ppm per step.
   - **Dry run by default** ("would trim by X ppm"); an option turns on the real trim.
   - Calibration (part A) gives the once-off value per device and reference; the monitor watches that it still holds.
   - `--sdr-ppm` corrects only the carrier, not the symbol rate. A 4.4 ppm symbol-rate error doesn't matter to ERMES pagers.
4. **Sign convention of `--sdr-ppm`.** The audit flagged it as possibly reversed. Check it on the air: the RTL (-0.33 ppm) measures the ERMES carrier on the internal reference, about -745 Hz expected. Then set `--sdr-ppm` and check that the error goes toward 0.
5. **Then:** send the refactor session the commit hash and the fast-forward range.

## E. True LO and exact rates per SDR (to do, all devices)

**Principle (user, 2026-11-27).** Tune the hardware coarsely, to frequencies its synthesizer hits exactly, or compute its real LO. Do the rest with libsdr's NCO. Allow only clean sample rates. Measure the clock from timing (LTE frame / GSM TDMA), which doesn't depend on the tuning step, and check it against the carrier offset at several frequencies. Apply `--sdr-ppm` in one place, libsdr: carrier through the tuning or NCO, rate through the resampler.

### Evidence so far

**RTL-SDR (R820T, TCXO 28.8 MHz).** The tuning step is not in the result:

| Method | Result |
|---|---|
| kalibrate-rtl, ch 76 (carrier) | -0.33 ppm (+312 Hz) |
| osmocalibrate, ch 76, via Soapy (carrier) | -0.40 ppm |
| own FCCH fit (carrier) | -0.93 ppm |
| LTE carrier | -1.23 / -1.13 ppm |
| LTE frame timing (crystal only) | -1.03 / -1.01 ppm |

- **Cause:** the R820T fractional-N PLL lands about 440 Hz off, differently per frequency, so up to ±0.2 ppm at 950 MHz. The driver doesn't report the real LO.
- **The rate:** the RTL2832U resampler is exact only for integer divisions of 28.8 MHz (2.4 = /12, 1.92 = /15, 1.2 = /24 MS/s); above 2.4 MS/s it drops samples.
- **In progress:** the PLL formula, the IF/DDC register and `set_freq_correction` (PLL vs resampler) are with the research agent (`docs/SDR-FRONTENDS.md`).

**USRP B210 (AD9361, internal TCXO / external ref):**

| Clock | LTE carrier | LTE timing |
|---|---|---|
| Internal | 5 cells from 937 to 2647 MHz: 4.4235–4.4251 ppm, within 0.002 ppm of each other | 4.401–4.448 ppm (noisier, 2 s captures) |
| External | 14.787–14.788 ppm | 14.777–14.788 ppm |

- **No frequency-dependent tuning step visible** at the 0.002 ppm level, i.e. about 2–5 Hz.
- **GSM ch 76** (950 MHz) reads 0.02 ppm lower with both clocks: that's the tower's own offset, not the B210.
- **Still to check:**
  - UHD's `tune_result` (`actual_rf_freq`, `actual_dsp_freq`) isn't logged by `uhd.c`. Log it once per tune at debug level, to have the real RF LO on record.
  - The AD9361 RF PLL resolution, from the datasheet (research agent).
  - The master clock and decimation for the chosen rate, so it's exact (rate = MCR / 2^k).

### Front-end research (2026-11-28)

The full text is `osmocom-analog-ermes/docs/SDR-FRONTENDS.md` (untracked, to commit). Scripts are in `~/.cache/sdrangel-rp-frontends/`.

**Usable part of the stream:**

| Device | Usable | Notes |
|---|---|---|
| B210, decimation a power of 2, ≥ 4 | flat within 0.6 dB to ±0.40 Fs; 64 dB alias-free to ±0.417 Fs | -24 dB at 0.45, -7 dB at the edge |
| B210, odd decimation | about 21 dB | avoid |
| LMS7002M | 75 dB to ±0.30 Fs, 60 dB to ±0.32 Fs | |
| Pluto | about 59 dB to ±0.40 Fs | USB 2: about 4–6 MS/s |
| RTL | ±0.85 MHz absolute at every rate | its FIR runs at 28.8 MHz |
| HackRF | no digital alias filter; analog filter 0.75 × Fs | |

**RTL tuning:**
- **PLL step:** 878.9 Hz / mix divider, i.e. 439.5 Hz (885–1770 MHz) and 219.7 Hz (442–885 MHz). The error is up to half a step, 0.24–0.49 ppm at 900 MHz.
- **True centre:** computable from librtlsdr's integer formulas. So tune on the PLL grid and put the rest in our NCO.
- **DDC:** the IF register has 6.87 Hz steps and truncates.
- **Exact rates:** 2.048, 2.4, 2.56, 1.92 and 3.2 MS/s.
- **`set_freq_correction`:** whole ppm only, and it acts on both PLL and resampler. Keep `--sdr-ppm` in libsdr.

**Risky in libsdr today (to fix, separate change after slice 2):**
1. **Soapy with an LO offset** (Lime, a B210 via SoapyUHD): `soapy.c:307-321` passes the offset, but `lo_applies` (`sdr.c:485-488`) is UHD only, so `lo_bandwidth()` (`sdr.c:268`) never widens the analog filter. That's the tilt that cost 21 dB at the far edge on the B210 before. Read from the code; check on a LimeSDR.
2. **B210 above 15.36 MS/s** (1 RX) or 7.68 MS/s (2 RX): the master clock equals the rate, with no FPGA decimation, and the automatic offset can't fit. The open would fail at `uhd.c:762` (traced, not run). `sdr_lo.c:43` doesn't look at the master clock.
3. **The automatic offset at -0.54 Fs** (`sdr_lo.c:29-31`) sits in the halfband's transition band: LO leakage only about 20 dB down, at +0.46 Fs. -0.60 Fs would give 66 dB, landing at +0.40.
4. **One 75 % usable fraction for every device** (`sdr.c:64`, `sdr_config.c:1153`, `sdr_lo.c:25`, `calibrate.c:603`, `radio/main.c:2507`). Needs a per-device table: B210 0.40, Lime 0.30, Pluto 0.40 (to measure), RTL ±0.85 MHz, HackRF 0.30, Airspy 0.40, unknown 0.35.
5. **The analog bandwidth defaults to the rate** (`sdr_config.c:893`). On the HackRF it must be 0.75 × Fs. On the B210: 1.5 × Fs with the LO at the centre, Fs + 2·|offset| with it outside. On the Lime: span × 1.2, at least 1.4 MHz.
6. **The RTL's true centre is unknown:** see the PLL point above.
7. **Minor:**
   - `sdr.c:886-888` moves a single channel off the DC when the LO is outside anyway.
   - The fallback in `sdr_calculate_optimal_rate()` (`sdr.c:1303`) may return a rate that doesn't divide the master clock.
   - Rate selection has no "as wide as possible" rule and doesn't prefer master clock / 2^k.

**Decision (user, 2026-11-28): above MCR/4, LO in the middle.**
- Up to 15.36 MS/s (1 RX) or 7.68 MS/s (2 RX), with MCR ≥ 4 × rate: the automatic LO offset as now.
- Above that: the LO in the middle of the band; our own digital shifts per channel; a strip around DC (a few tens of kHz, to measure) blocked for channels; analog bandwidth 1.5 × Fs.
- Usable at 20 MS/s: about ±8 MHz minus the DC strip, against ±6.1 MHz at 15.36 MS/s with offset.
- Open: the IQ mirror level (a strong +f leaves a weak -f; no AD9361 figure published). Measure it on the loopback cable together with the DC strip width and the edges. Replaces today's refusal (risky point 2).

**Tests from the research:**
- the coefficient model as a unit test of the per-device table;
- osmocalibrate at two centres 0.2 Fs apart, to tell aliases from real carriers;
- a B210 loopback tone sweep across ±0.7 Fs (decimation, offset, analog bandwidth);
- one B210 run at 20 MS/s with the automatic offset, to confirm point 2;
- the same sweep with the RTL and a Lime as receiver.

**Gaps:** no published roll-off measurements for the B210, Pluto or Lime; the MAX2837 filter shape and the Pluto default FIR are unconfirmed; no AD9361 RX image-rejection figure.

### Tests to do

1. **B210, now possible.**
   - LTE timing vs carrier, longer captures (30 s) at 3 or more frequencies per band, internal and external. Expect the carrier equal at every frequency to 0.002 ppm, and timing equal to carrier within its uncertainty.
   - With `tune_result` logged: check that the requested and actual RF LO differ only by what UHD reports.
2. **RTL:**
   - The same LTE test, tuned the old way vs the "PLL on a clean grid, the rest by NCO" method. Expect the new carrier ppm to equal the timing ppm (about -1.0) at every frequency.
   - Only clean rates.
   - Repeat kalibrate-rtl on 3 channels: does its -0.33 move with the channel?
3. **Other SDRs** (LimeSDR, HackRF, Pluto, Airspy; the user adds them later): the same test matrix, with the method from the research doc per device. Each gets a row in the evidence tables above.

## D. DSP library cleanup (after slice 2)

Rule (user): the refactor's point is one shared SDR path for all components. Guide: `docs/SDR-BACKEND.md`, "Which path to use".

| Library | Used by (2026-11-27) | Plan |
|---|---|---|
| libchannelizer (2026-11-23: pfb bank, halfband, resamp, fftc, channelizer) | libsdr for every program | **Keep, it's the core.** **Rename to `libchannels`** (user 2026-11-27): its own commit after slice 2 and the deletions; directory, Makefiles, includes and docs, no code change |
| libiqringbuf | nobody (only linked by osmocalibrate) | delete |
| libfastmath | only libcalibrate (the old NCO) | delete after slice 2 |
| libpolyphase | libcalibrate (old path), radio (osmoradio) | radio onto `libchannelizer/resamp.c` (measure CPU and audio), then delete |
| libfft vs `libchannelizer/fftc.c` | old: all programs and displays; new: the bank | two FFTs: measure, keep one |
| libsamplerate | all programs (audio ↔ DSP rate) | keep; later check whether `resamp.c` covers it |

**Generic pieces proposed by the LTE agent** (proposal; all still in the `lte_*` files):
1. A receiver spur finder (long FFT, peaks against the local median), so FCCH and LTE positions can be masked. B210 spurs (e.g. +424 kHz, 34 dB above the floor) pulled the LTE offset 70–80 Hz. Into libsdr or libcalibrate.
2. Phase-based offset over several spans (0.5 / 5 / 10 ms), resolving the alias and fused with the timing. Into libcalibrate.
3. Sample-clock ppm from frame-timing drift (a regression with its standard error): the clean ppm on the RTL. Shared, and for GSM too.
4. An FFT matched-filter search over offset hypotheses: later perhaps `fft_xcorr` in libfft.
5. Share one band power ranking between GSM and LTE.

Check before every merge: `grep -rlE "uhd_open|uhd_receive|soapy_open|soapy_receive|channelizer_rx_init" src --include=*.c` must show nothing outside libsdr, libchannelizer and test. The generic-parts proposals from the slice 2 and LTE agents go here too.

## C. ERMES: fixes and work still to do

The audit (`docs/audit/SUMMARY.md`, 19 defects) is fixed except for the items below. Fixed in:
- **4242cb8:** decoder items 1–8;
- **f375f24, d328b34, 0cf8fc1:** item 9 (control characters);
- **ba8e4f2:** items 10–13, 15 and 16.

The details for each test are in `TEST_BACKLOG.md`.

### Still open from the audit

- **14. The character-set indication** (AIT 100 AIN 0010) is not sent with `--charset` codepages. Latent while only the standard codepage exists; needed for item M2.
- **17. Carrier accuracy** (±200 Hz = ±1.18 ppm). The B210 is -4.4 ppm, i.e. -745 Hz at ch 9. Fix: `--sdr-ppm` from calibration (A) after the sign check (B.4), and document it in ERMES.md. Also measure the Pi/rpitx.
- **18. Symbol rate** ±1 ppm follows the SDR clock; `--sdr-ppm` doesn't correct it. The B210 at 4.4 ppm is out of spec, though pagers don't care. rpitx is far off (192027 for 192000 S/s). A resampler trim would be the fix (later, measured CPU).
- **19. UTC grid** ±2 ms (§12.4). Locked once at the start, then counted in samples: it drifts past 2 ms after about 8 min at 4.4 ppm, and any stall slips it for good. No re-lock. That's part B: the drift monitor and a slow re-lock (dry run first).
- **The audit's doc items:** an rpitx/accuracy section in ERMES.md, and documenting LF/CR as escapes (check after f375f24).

### Missing features, by usefulness (audit)

- **M1.** Long messages §10.4 (submessages, link counter): encoder and decoder reassembly.
- **M2.** Optional character sets with the indicator, especially 00101 Cyrillic incl. Ukrainian (-2 p.62).
- **M3.** A per-receiver profile in the book: type, max length, alert, priority. SM, HNL and ENL are done.
- **M4.** Automatic lastnum at low traffic (backlog N4), deferred delivery, message store and retrieval.
- **M5.** -6 test-signal mode (fixed dibits, PRBS).
- **M6.** Decoder: a "receiver with identity X" mode for the conformance cases, DSP unit tests.
- **M7.** Encoder test hooks: non-delimiter fill, message placed last or delayed, RSVD bits, bit errors, BAI.
- **M8.** JSON control-socket API for the book and pages (FIFO `book` now, the API later; user's decision).
- Lower priority: transparent data TX, remote programming, barring/diversion, time-divided operation.

### On-air tests still open (TEST_BACKLOG.md)

| Group | Tests |
|---|---|
| Group calls | G1 (GC, long text over two batches), G2 (GA again), G3 (`--group-fig16`), G4 (page to a member right after its group text), G5 (group text sharing a batch) |
| lastnum | N1–N3; N4 (automatic lastnum, not built) |
| Visitors on the NEC | X1–X4 (external bit with a foreign OPID or its home OPID, ETI = 1, NEC as a visitor). The user can test the NEC on its home network and as a visitor. |
| Order | O1, O2 |
| Two channels | C1 (group across channels), C2 (inverted polarity) |
| Receiver | R1 (a real foreign signal, weak signal), R2 (CPU) |
| PDW | D1: PDW and the decoder on every new feature |

### Other

- **`device_drift()`** (the uncommitted experiment in `src/ermes/scheduler.c`) is gone: replaced by part B's clock monitor (92cad9d). The diff is kept in `~/.cache/sdrangel-rp-ermes/clk/device_drift_experiment.diff`.
- **Parked side bugs**, other protocols: FLEX/RDS wall clock, AMPS REGID, FuVSt Q = 1, C-Netz help.

## State 2026-11-28

- A and B are done on `ermes`. 78c70c7 commits SDR-FRONTENDS.md.
- Handover sent to the osmocom-refactor session: fast-forward master 18c9be9 → ermes 78c70c7 (91 commits), not pushed.
- Next: B open items:
  - a real trim run on a device;
  - the NMT need;
  - the ERMES cycle re-lock (C.19).
- Then: section E libsdr fixes, D cleanup and rename, C ERMES items.

## Order (user, 2026-11-27)

1. Finish A (the slices).
2. Finish B.
3. Merge both.
4. Merge `ermes` back to the osmocom-refactor agent.
