# Channelizer and synthesizer (SDR channels)

Working document for replacing the per-channel frequency handling of libsdr
(RX channelizer, TX synthesizer) for all SDR drivers. rpitx is not included
(it makes the carrier itself).

## Why

- `fm_mod_init` / `fm_demod_init` shift each channel by its offset at the DSP
  rate (`-s`), while the span checks compare against the SDR rate. An offset
  beyond +-DSP/2 folds back (aliasing). Measured 2026-11-23: AMPS 354/375/396
  at `-s 96000` put control channel 354 (offset -315 kHz) at 880.90 MHz
  (folded to -27 kHz); the phones found no network. 354/356/358 put 358
  (+90 kHz) at -6 kHz. Two channels at +-30 kHz with 18 kHz half bandwidth sit
  exactly at the edge (+-48 kHz).
- The existing libchannelizer was only used by osmoradio and calibrate; the
  mobile systems never ran on it.

## Review of the existing libchannelizer (tone tests, unmodified library)

Harness: `~/.cache/sdrangel-rp-rates/chan/tone.c` (not in the tree).

| Item | Measured | Result |
|---|---|---|
| `interpolate_center` (TX), constant 1.0 in | 2.04, 1.00, 1.97, 1.00, ... | broken: one phase is the decimation FIR, the other the input passed through |
| `interpolate_lower/upper`, DC in | -0.25 / +0.25 fs_out, 3.5 dB high | direction right, level wrong (from the item above) |
| `halfband_interp2` (order 48) | flat to 0.4 fs_in, image rejection 81..99 dB | correct |
| decimation CENTER, double | order 48: alias rejection 81 dB, order 64: 104 dB | correct |
| decimation CENTER, fast math (Q12) | order 48: 60..72 dB, order 64: 69..85 dB | below the stated -80 / -103 dB |
| decimation LOWER / UPPER | LOWER keeps +fs/4, UPPER keeps -fs/4 | swapped |
| `center_offset` (residual after the fs/4 chain) | not read by sdr.c | dropped |
| `channelizer_init(960000 -> 96000)` | output rate 120000, no error | wrong rate |
| TX use in sdr.c | chan[0]'s chain over the sum of all channels, whose offsets fm_mod already applied | not a per-channel synthesizer |

Decision (user, 2026-11-23): drop and replace, no compatibility code.

## New design (implemented)

Per channel, both directions, for every SDR driver (libchannelizer,
libsdr/sdr.c):

- RX: NCO at the SDR rate shifts the channel to 0 Hz; halfband 2x
  decimators (double precision) as long as the rate stays an integer of at
  least 1.25 x the DSP rate (or reaches it exactly); then a rational
  polyphase resampler P/Q (Kaiser, 100 dB, passband 0.4, stopband 0.5 of the
  DSP rate) to exactly the DSP rate. The demodulator runs at 0 Hz.
- Each halfband is designed for its stage (105 dB): with the channel at 0 Hz
  a stage only has to keep the band the later stages pass, a part of its
  rate that halves with every stage towards the SDR rate. At 10 MS/s to
  200 kS/s the stages have 12, 12, 20, 24, 40 taps (was 64 each). The design
  searches the shortest Kaiser halfband that reaches the attenuation, checked
  on a grid; designs are cached (first channel ~100 ms, the others reuse it).
- Block processing, stage by stage (not one call per sample and stage).
- TX: the mirror image; each channel's synthesizer adds its channel at the
  SDR rate; the write thread only sends.
- Integer ratios preferred (small P/Q, cheaper), any ratio works.
- The read thread puts the device's IQ into the ring as received;
  sdr_read() takes exactly the input the channelizers need for the requested
  DSP samples. channels = 0 (osmoradio, tv, dcf77): one channelizer at 0 Hz
  over the usable band.
- IQ recordings (--write/read-iq-rx/tx-wave) are at the SDR rate.
- Refused with a message: SDR rate below the DSP rate, a channel wider than
  0.8 of the DSP rate, a channel outside the SDR band.
- The paging channel has its own synthesizer; a channel switched to it feeds
  its own with the carrier off, so all states stay continuous.
- Removed: the old channelizer, channelizer_tx, the Q12 path, the
  LOWER/UPPER modes, the whole-band halfband / IIR / sample-repeat paths,
  the whole-band polyphase resampler of split mode, sdr_open_channelizer(),
  osmoradio --channelizer auto|on|off.

## Measurements

Library (src/test/test_channelizer, PASSED): every halfband as designed for
its stage (passbands 0.005 .. 0.2) flat to 0.01 dB, alias and image
rejection >= 105.7 dB, in blocks of odd length; channel level error < 0.01
dB at the right frequency (7 tones), the input from channelizer_rx_need()
gives exactly the wanted samples per call; everything else in the SDR band
<= -108.6 dB in the whole DSP band (960k/96k, 384k/96k, 1M/96k
non-integer, 10M/200k, 768k/192k); TX wanted line 0.00 dB, everything else
<= -109 dB (960k, 3.84M, 1M, 2.4M, 10M, 768k); TX sample count without
drift.

B210, NMT 2 channels at +-25 kHz, TX to RX by cable (-l 2), old and new
build back to back, 20 s each (SDR 768k, DSP 192k), 3 pairs, with an
RTL-SDR beside it measuring what is sent (rtl_power, 10 kHz around each
channel):

| pair | frames old / new | B210 RX channel old / new | IMD3 old / new | RTL-SDR ch 90, 92 old / new |
|---|---|---|---|---|
| 1 | 238 / 236 | -39.1 / -40.1 dBFS | -48.0 / -48.7 dBc | 2.5, 1.0 / 2.3, 0.9 dB |
| 2 | 238 / 236 | -37.5 / -38.3 dBFS | -49.4 / -49.6 dBc | 3.2, 1.9 / 2.9, 1.6 dB |
| 3 | 238 / 236 | -37.5 / -37.8 dBFS | -49.5 / -49.1 dBc | 2.5, 0.9 / 2.4, 0.8 dB |

0 bad frames. The B210 loopback level moves between sessions (the old build
measured -42 dBFS an hour earlier) for both builds alike; the sent level is
the same within 0.3 dB. 2 frames fewer: start-up (design ~100 ms).

CPU, whole process per thread, 2 channels, 15 s (main / RX thread / TX
thread):

| SDR rate | old | new |
|---|---|---|
| 768k (DSP 192k) | 8 / 2 / 5 % | 11 / 1 / 4 % |
| 1.92M (DSP 192k) | 8 / 3 / 3 % | 21 / 2 / 2 % |
| 10M (DSP 200k) | 9 / 16 / 16 %, 216 frames | 50 / 10 / 10 %, 218 frames (before the per-stage design: 100 %, 6 frames, overflows) |

The per-channel chain costs ~10 % of a core per channel and direction at
10 MS/s: it does not scale to many channels. Next: a polyphase FFT filter
bank (cost independent of the channel count) and one SDR backend that
processes register their channels on (see the plan below).

## Plan: filter bank and shared backend

Prototype (~/.cache/sdrangel-rp-rates/pfb, not in the tree yet): 2x
oversampled polyphase FFT analysis bank, in-tree FFT (Stockham radix 4),
exact against the direct formula (3e-13), >= 111 dB within +-0.6 x the
channel bandwidth; 9 % of a core at 10 MS/s for any number of channels, the
per-channel tail (fine NCO, resampler with band edges) ~0.15 %. Still open:
TX synthesis bank (must filter the channel before the bank: everything else
is sent), block borders, the guarantee change (+-W clean instead of the whole
DSP band), Pi measurements, the backend design.

## Open

- Pi 4 CPU (RTL-SDR RX).
- Over the air: wide channel spacing (AMPS), osmoradio, osmocalibrate.
