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

## New design

Per channel, both directions, for every SDR driver:

- RX: NCO at the SDR rate shifts the channel to 0 Hz; halfband 2x stages
  (double precision, order from the measured response) and a polyphase FIR
  decimator for the odd remainder of the ratio reach exactly the DSP rate;
  the demodulator runs at offset 0.
- TX: modulator at offset 0 at the DSP rate; polyphase FIR interpolator for
  the odd factor, halfband 2x interpolation (polyphase form); NCO at the SDR
  rate; channels summed at the SDR rate.
- Every rate ratio and offset is checked; what does not fit is refused with a
  message.

## Tests (to be filled in)

- tone tests of the new blocks: frequency, level, image/alias rejection
- IQ file tests: every channel on frequency, isolation between channels
- CPU per channel (PC, Pi)
- over the air: NMT, AMPS, FLEX with the B210, RTL-SDR RX
