# SDR backend with registered channels (design, for review)

Status 2026-11-24: steps 1, 2 and 3 built (filter banks, the engine in the
device threads, `osmo-sdr-backend` with `--sdr-backend <socket>` in the
programs). 2026-11-25: two networks (NMT and POCSAG) on one B210 through
the backend. Open: phones on the engine path, broadcast FM next to them,
the Pi 4.

## Goal

One radio, many networks: an SDR backend owns the device; the programs (nmt,
amps, osmoradio, ...) register their RX and TX channels on it. 10, 20, even
100 narrow FM channels and broadcast FM in one SDR stream. A program can still
open its own radio (the same engine inside the program).

## Parts

```
 device (UHD, Soapy, ...)                       programs
   |  SDR rate IQ                                 nmt   amps   osmoradio
 engine (libchannelizer + libsdr)                  ^ |   ^ |    ^ |
   RX: analysis filter bank -> per channel tail ---+ |   | |    | |   DSP rate,
   TX: per channel tail -> synthesis filter bank <---+   | |    | |   0 Hz,
   registry of channels, levels, status                  | |    | |   per channel
 transport: in the program (direct calls)  or  backend process (shared memory)
```

- **Engine.** Device threads, the filter banks, the channel registry. The
  same code runs inside a program (as today, one network) or in the backend
  process (several programs).
- **Filter bank** (per direction, per width class): 2x oversampled
  polyphase FFT bank, in-tree FFT (`fftc.c`), cost independent of the
  channel count. Width classes: narrow FM (NMT, AMPS, ...) and wide
  (broadcast FM) get their own bank, or the per-channel path when it is
  cheaper (one or two channels at a low SDR rate).
- **Per channel tail**, the same guarantee as the per-channel chain (flat to
  0.4 of the DSP rate, nothing else in the whole DSP band):
  RX: fine NCO at the bin rate, a wide resampler bin rate -> 2 x DSP rate
  (keeps +-DSP/2), a sharp 2:1 stage (pass 0.4, stop 0.5 of the DSP rate).
  TX the mirror: 1:2 (pass 0.4, stop 0.5: nothing beyond +-DSP/2 enters the
  bank, else it would be sent), wide resampler to the bin rate, fine NCO.
  The bank keeps +-DSP/2 around every channel clean, so its sub-band spacing
  must be more than the DSP rate (at 48 k: 156 kHz, M = 64 at 10 MS/s).
- **Modulation stays in the program.** fm_mod / fm_demod, AM, levels, the
  protocol: the program exchanges complex samples at its DSP rate with the
  channel at 0 Hz, as `sdr_read()` / `sdr_write()` do today.

## Measured (prototypes, this PC, -O2, in-tree FFT)

10 MS/s, DSP rate 48 k (the default), M = 64 (156 kHz sub-bands), every
channel clean over the whole DSP band:

| | bank (all channels) | per channel | 100 channels |
|---|---|---|---|
| RX | 10 % of a core | 0.47 % | 67 % |
| TX | 10 % of a core | 0.47 % | 62 % |
| today's per-channel chain | - | ~10 % per direction (measured at DSP 200 k) | - |

RX: bank exact against the direct formula (8e-14), random block borders bit
identical; channel level error 0.0001 dB at the right frequency; 211 tones
over the SDR band and 18 critical ones (the neighbours at 25 and 30 kHz, the
DSP band edge, the bank's alias points): nothing above -115.8 dB anywhere in
the DSP band.
TX: bank exact (2.5e-7, float output); two tones 0.0000 / 0.0001 dB,
everything else in the whole SDR band <= -114.3 dBc; a tone at 0.42 of the
DSP rate sent at -0.18 dB, its images <= -104.3 dBc; two FM channels 25 kHz
apart in one sub-band: nothing outside them above -115.6 dB.
Not yet: through the real demodulator against today's chain, the Pi.

## In the programs (step 2, done: `--sdr-filter-bank auto|on|off`)

B210 cable loopback (NMT -l 2), same build, bank vs per-channel chain, 15 s,
DSP 48k; quality and level as NMT measures each received frame (through
the FM demodulator and the decoder), CPU per thread from /proc:

| setup | frames (15 s) | quality avg / min | events | CPU total (main) |
|---|---|---|---|---|
| 2 ch, 1.92 MS/s, bank | 218, 0 invalid | 91.0 / 91 % | 0 | 17 % (12 %) |
| 2 ch, 1.92 MS/s, chain | 218, 0 invalid | 91.5 / 91 % | 0 | 17 % (12 %) |
| 2 ch, 10 MS/s, bank | 216, 0 invalid | 91.1 / 91 % | 0 | 61 % (34 %) |
| 2 ch, 10 MS/s, chain | 216, 0 invalid | 91.1 / 91 % | 0 | 64 % (38 %) |
| 10 ch, 10 MS/s, bank | 1090, 0 invalid | 91.0 / 91 % | 0 | 73 % (46 %) |
| 10 ch, 10 MS/s, chain | 0 (main thread at 100 %) | 53 / 43 % (24 frames) | 412 overflows | 115 % (100 %) |

The device threads (TX ~11 %, RX ~10 % at 10 MS/s) are the UHD transfer,
the same for both. In the main thread ~7 % at 10 MS/s are the ADC/DAC
status counters over every SDR sample (to move into the device threads).

## Engine in the device threads (step 3a, done)

`libsdr/sdr_engine.c`: the banks, the channel groups and their DSP rate
rings; the device RX thread runs the RX bank and the tails, the TX thread
pulls the TX rings through the tails and the TX bank. A group is one
program's channels; a row holds one sample of every channel. Groups are
built by the caller and adopted / dropped by each device thread at a block
border (trylock); a group that falls behind gets silence on TX and loses
rows on RX, counted, the others untouched. TX pulled with
`pfb_chan_tx_need()` / `pfb_chan_tx_pull()` (exact frame counts).

`test_sdr_engine` (null device, TX looped to RX): tones back at 0.000 dB,
the other channel -173 dB, 0 missing / dropped rows; RX: group A bit
identical while B joins and leaves; TX: A within -142.6 dB while B joins,
stalls, transmits and leaves; B's stall counted on B only.

In sdr.c (same B210 loopback as above, DSP 48k, 15 s):

| setup | frames | quality | CPU total | main thread | RX thread | TX thread |
|---|---|---|---|---|---|---|
| 2 ch, 1.92 MS/s | 218, 0 invalid | 91.0 % | 17 % | 5 % (was 12) | 5 % | 6 % |
| 2 ch, 10 MS/s | 216, 0 invalid | 91.3 % | 56 % (was 61) | 8 % (was 34) | 21 % | 21 % |
| 10 ch, 10 MS/s | 1090, 0 invalid | 91.0 % | 71 % (was 73) | 13 % (was 46) | 26 % | 26 % |

10 ch at 10 MS/s for 120 s: 8670 frames, 0 invalid, no device events, no
missing / dropped rows (logged once a second when they change).

Latency, on one clock (time a sample is written -> time it is read back;
noise as TX audio, B210 loopback, NMT 2 ch, DSP 48k, 1.92 MS/s): engine
50.5 ms (49.6..51.2), per-channel chain 52.2 ms (51.2..53.1). The device is
kept a few blocks ahead (>= 5 ms), the group's ring holds the rest of the
buffer.

Split mode (NMT, B210 TX + RTL-SDR RX via Soapy) on the engine: 228 frames,
0 invalid, as the chain. FLEX 2 channels (931.9375 / 931.9625 MHz), TX-only
B210 and RX-only RTL-SDR, engine on both: the TX IQ recorded and decoded
from file: 1472 / 1472 codewords clean on both channels, as with the chain;
one over-the-air recording decoded with the RX bank and with the chain: the
same, all clean, all 5 pages with good checksums. Over the air some runs had
errors on 931.9375 MHz with either path: the RTL-SDR is about -7 ppm off
the B210 (-6.6 kHz at 932 MHz, it drifts), next to where the channel 25 kHz
from its neighbour fails (6..8 kHz: the neighbour gets into its
demodulator; proven by an offset sweep, over the air and from file). Fixed
with --sdr-ppm (it had no effect on Soapy, 43349f5) and the FLEX AFC
(741764e).

(The quality figure is NMT's FSK decoder measure of how close the tone
frequency is to ideal at each bit, averaged over a frame; 91 % is its
ceiling on a clean cable path for every build, old ones included.)

Over the air with phones, B210, filter bank active (2026-11-23):

- NMT 2 channels (90 CC/TC, 92 TC), DSP 48k, SDR 1.92 MS/s, 8 sub-bands:
  call between two phones set up and cleared normally; 48 frames from the
  phones, RX level 93 %, quality 86 % on average (one frame 46 %);
  supervisory tone 87..95. Audio: "the same" as before (user).
- AMPS 354 CC/PC, 375 VC, 396 VC (offsets -315, +315, +945 kHz; the layout
  that failed with the old code), DSP 96k, SDR 3.84 MS/s, 16 sub-bands:
  both phones registered, call alerted with caller ID, answered, voice on
  both voice channels, release confirmed by the mobile. RTL-SDR: the control
  channel at 880.620 MHz, nothing else came up in 880.4..882.1 MHz.
  Audio: "seems also good" (user).

## Rules

1. **Nothing changes under a running channel.** SDR rate, centre frequency,
   gains and the banks are fixed when the backend starts. Registering picks a
   bin and adds a tail, or is refused with the reason: outside the RX or TX
   band (the backend has one of each), a DSP rate above what the bank's
   sub-bands carry (the spacing must be more than the DSP rate; the largest
   DSP rate is set at start), wider than 0.8 of its DSP rate, no channel left.
2. **TX level is fixed per channel.** The backend starts with the most
   channels it carries (N max); each channel's carrier has 1/N max of full
   scale, so the sum never clips and a channel's power never changes when
   others come or go. Fewer channels: the DAC runs below full scale by
   20 log(N max / N) dB. Inside its channel the program sets its own
   level as today (deviation, speech level).
3. **The device threads never wait for a program.** A program that falls
   behind: its TX channel is sent without carrier (zeros), its RX samples are
   dropped; the other channels are not touched. It is logged.
4. **Equal sample counts.** All channels of one program get the same number
   of samples per call (tails started on a common phase), as the programs
   expect today.
5. **SDR rate things are in the backend:** device options, IQ recording at
   the SDR rate, spectrum / IQ displays, ADC clipping and DC status.

## Transport (backend process): local only

- Control: a Unix socket (SOCK_SEQPACKET, `@name` = abstract namespace),
  `libsdr/sdr_backend.c`. One request per connection: the program's
  channels by frequency, its DSP rate, ring length and TX target. Answer:
  accepted (a memfd, sealed against resizing, passed with SCM_RIGHTS; the
  TX scale; the centres) or refused (the reason).
- Samples: the group's view (`sdr_view_t`) in that shared memory: one RX
  and one TX ring per program (a row holds a sample of every channel, so
  all its channels stay in step), single producer, single consumer, lock
  free, complex float at the DSP rate; the counters. No pointers in it;
  each side keeps its own ring sizes, the device threads range-check every
  index the program writes (out of range: nothing in, silence out,
  counted). Latency: one ring fill, set by the program as its buffer.
- A program ending or crashing frees its channels (connection closed); a
  backend ending ends its programs (they see the connection close).
- Programs: `--sdr-backend <socket>` (`libsdr/sdr_client.c`, `sdr.c`
  client mode, the same port code as the engine in-process). Backend:
  `osmo-sdr-backend --tx <MHz> --rx <MHz> --dsp-rate <Hz> --channels <n>`
  with the usual device options.

Tests: `test_sdr_backend` (null device, TX looped to RX, programs as
processes: own channels back, the others' <= -173 dB; a killed program
removed; a bad index counted, nobody else affected; refusal with reason;
the backend going). End to end: RTL-SDR RX only as the clock with the
POCSAG test transmission as RX IQ, `pocsag --sdr-backend`: 6/6 messages;
the client ends within 1 s when the backend stops.

## Steps

1. TX and RX banks into libchannelizer with tests in `test_channelizer`
   (exactness, block borders, levels, rejection over the whole band, CPU).
2. Engine in-process: `sdr.c` uses the banks and tails for its channels
   (the per-channel path stays for small setups), the cheapest chosen by
   measurement. Verify: loopback on the B210 with an RTL-SDR beside it, then
   phones (NMT, AMPS with wide channel spacing).
3. Backend process and the client driver behind `sdr_open` / `sdr_read` /
   `sdr_write` (`--sdr-backend <socket>`). Verify: two programs in one band
   on one B210 (NMT at 465 MHz and POCSAG at 466.23 MHz; NMT and AMPS are
   too far apart for one SDR stream), then broadcast FM next to them.
4. Pi 4 measurements.

Over the air (B210 cable loopback, NMT `-l 2`, DSP 48k, SDR 1.92 MS/s,
TX 60 / RX 40, 15 s, 2026-11-24):

| setup | frames | invalid | quality | CPU |
|---|---|---|---|---|
| in-process filter bank, NMT 90/92 | 216 | 0 | 91.0 % | 20 % |
| backend `--channels 2`, NMT 90/92 | 216 | 0 | 91.0 % | nmt 5 % + backend 13 % |
| backend `--channels 4`, NMT 90/92 and a second NMT 96/98 | 218 / 216 | 0 / 0 | 91.2 / 91.0 % | 5 % + 5 % + backend 15 % |

TX and RX on one device clock: the TX time stamps count on sample by
sample from the RX time at the start plus the device advance (5 ms), so
TX stays locked to RX without drift; each program's RX -> TX delay is the
advance plus its ring target. (Two nmt programs: give each its own
`--sms-deliver`, `--sms-submit` and `--cmd-fifo` paths.)

Two networks on one B210 (2026-11-25, RF loopback, no phones; backend
centre 465.70 MHz TX and RX, `--channels 4`, 1.92 MS/s, DSP 48k, TX 60 /
RX 40, 30 s): nmt 90/92 `-l 2` and `pocsag -k 466.230 -T -R -B 1200`,
six alpha pages from its FIFO, 4 s apart, during the run:

| program | result | CPU |
|---|---|---|
| nmt 90/92 | 434 frames, 0 invalid, quality 91.1 % (min 91 %) | 4 % |
| pocsag 466.230 | 6 / 6 pages received, 0 corrected, 0 uncorrectable | 3 % |
| backend | | 14 % |

NMT as with NMT alone (216 frames per 15 s, 91 %); the pages did not
disturb it.

The backend drains its raw SDR rate ring (`sdr_server_poll()` from its
main loop, every half `--buffer`): RX overflow and ADC clipping are
reported there (the programs only see their channels). Clipping is logged
when it starts, every 10 s while it lasts and when it ends; the counters
follow `--sdr-iq-status`. B210 cable loopback above (TX 60), raw RX
recorded and analysed: at RX 20 the own carriers are -9 dBFS rms (peak
0.68); at RX 30 and up the ADC is full (rms -1 dBFS, 25 - 46 % of the
samples clipped, intermod at +100 / -900 / -950 kHz). The RX 40 numbers
in these tables were taken with the ADC clipping; NMT and POCSAG decoded
the same at RX 0 to 76 (434 frames 91 %, 6 / 6 pages): quality and
frame counts do not show it, they are pass / fail, not a margin. Cable
loopback at TX 60: use RX 10 (about -19 dBFS); there the same 434 frames,
91.2 %, 6 / 6 pages, no clipping. Backend CPU with the counters on / off
16 / 14 % and 15 / 15 % (two pairs of runs): within the run spread.

## Open

- The backend has no IQ / spectrum display (no terminal UI); a status
  interface would carry them (docs/HEADLESS.md step 4).
- The backend's centres are the operator's: a client warns when a
  channel covers one (DC, LO leakage).

- Paging channel (NMT, ...): a second TX channel of the same program that
  carries the switched channel; registered as an ordinary TX channel.
- rpitx stays outside (makes its own carrier).
- Names: `osmo-sdr-backend`? Option names.
