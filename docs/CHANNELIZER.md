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
  decimators (order 64, double precision) as long as the rate stays an
  integer of at least 1.25 x the DSP rate (or reaches it exactly); then a
  rational polyphase resampler P/Q (Kaiser, 100 dB, passband 0.4, stopband
  0.5 of the DSP rate) to exactly the DSP rate. The demodulator runs at 0 Hz.
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

Library (src/test/test_channelizer, PASSED): halfband order 64 alias and
image rejection 104.6 dB, flat to 0.01 dB; channel level error < 0.01 dB at
the right frequency (7 tones); everything else in the SDR band <= -104.6 dB
in the whole DSP band (960k/96k, 384k/96k, 1M/96k non-integer); TX wanted
line 0.00 dB, everything else <= -107 dB (960k, 3.84M, 1M, 2.4M); TX sample
count without drift (within one 2^k step).

CPU of the library on this PC (one core, per channel, RX / TX): 960k/96k
2.3 / 3.3 %; 384k/96k 0.6 / 1.0 %; 1.92M/96k 4.1 / 6.4 %; 3.84M/96k 7.8 /
12.4 %; 1M/96k (non-integer) 2.4 / 3.4 %.

B210, NMT 2 channels at +-25 kHz, TX to RX over the air (-l 2), old and new
build back to back, same setup, 30 s each (SDR 768k, DSP 192k):

| build | frames | channel | IMD3 | SNR (16 kHz) | CPU |
|---|---|---|---|---|---|
| old | 382 ok, 0 bad | -45.9 dBFS | -41.6 dBc | 46.4 dB | 17.2 % |
| new | 382 ok, 0 bad | -44.5 dBFS | -44.9 dBc | 48.8 dB | 22.3 % |

At SDR 1.92M: CPU old 17.3 %, new 42.0 % (2 channels x RX and TX per
channel at the full SDR rate). The old code converted the sum of the band
once (with the faults above).

## Open

- CPU: a shared first stage for channels close together (halfbands over the
  whole band while it still holds all channels), same filters, same result.
- Pi 4 CPU (RTL-SDR RX).
- Over the air: wide channel spacing (AMPS), osmoradio, osmocalibrate.
