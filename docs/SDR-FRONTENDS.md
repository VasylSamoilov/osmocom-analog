# SDR front-ends: how much of the sample rate is usable

Status 2026-11-27: research note, nothing changed in the code. It covers
how wide a rate libsdr may run, which part of the stream a channel bank may
use, where aliases, the DC and IQ images land, and what the analog filter
should be set to, per device. It ends with what libsdr does today, the
recommended rules, and tests.

Every number is tagged:

- **[doc]**: the manufacturer, a datasheet, or the vendor's own source code
  (FPGA filter coefficients, driver tables).
- **[computed]**: my own calculation from documented coefficients (the
  filter taps of the chip or FPGA, run through a frequency response in
  numpy; the script is described in "Method" at the end). These are exact
  for the digital filters, but they leave out the analog filter, the ADC
  and the real device.
- **[measured]**: a published measurement by someone else (one unit,
  usually).
- **[ours]**: measured on our bench (B210 cable loopback), taken from
  the comments and docs in this tree.
- **[inference]**: my reasoning, not checked. Each one needs a test.

`Fs` is the host sample rate (the rate libsdr gets), `fo` a frequency in
the stream as a fraction of Fs (the stream is -0.5 .. +0.5). "Usable
+-0.40" means channels can sit anywhere in -0.40 Fs .. +0.40 Fs, i.e. 80 %
of the rate.

## Summary

| Device | Rate limits (host) | Digital decimation | Flat (< 0.6 dB) to | Alias-free to (>= 60 dB) | Recommended usable | Notes |
|---|---|---|---|---|---|---|
| USRP B200/B210, Fs <= MCR/4 | MCR <= 61.44 MHz (1 RX), <= 30.72 MHz (2 RX); Fs = MCR / D | AD9361 halfbands + 128-tap FIR, then FPGA: CIC (N=4) + two 47-tap halfbands | +-0.40 [computed] | +-0.417 (64 dB), +-0.27 for 80 dB [computed] | +-0.40 (80 %) | D a power of 2 >= 4 (UHD's automatic MCR does this) |
| B200/B210, Fs = MCR (D = 1) | Fs > 15.36 MS/s (1 RX) or > 7.68 MS/s (2 RX) with the automatic MCR | AD9361 only (127-tap halfband FIR) | digital: +-0.45 [computed]; but the analog filter is capped at bw = MCR = Fs: -1.4 dB at 0.40, -2.4 dB at 0.45 [computed] | +-0.45 (79 dB) [computed] | +-0.40, LO at the centre | the automatic LO offset makes the open fail here (see B210) |
| B200/B210, odd D or D = 2 x odd | only with a forced MCR | CIC alone, or CIC + one halfband | -8.7 dB / -2 dB at 0.4 [computed] | +-0.14 / +-0.28 [computed] | avoid | UHD warns "expect CIC rolloff" [doc] |
| LimeSDR (LMS7002M), decimation >= 2 | LimeSDR USB 61.44 MS/s (USB 3), Mini 30.72 MS/s | HB2 (15 taps) x n, then HB1 (31 taps) | +-0.35 (0.07 dB), +-0.40 (0.55 dB) [computed]; analog LPF ripple about 0.5 dB [measured] | +-0.27 (108 dB), +-0.30 (75 dB), +-0.32 (60 dB) [doc][computed] | +-0.30 (60 %) | at libsdr's +-0.375 only 32 dB [computed] |
| PlutoSDR (AD9363) | USB2: 4 MS/s without drops (ADI), about 6 MS/s measured | AD9363 halfbands + programmable FIR (loaded by the Linux driver / libad9361) | +-0.45 with `fir_128_4` [computed] | +-0.40 (59 dB) with `fir_128_4` [computed] | +-0.40 (measure: the loaded FIR is not confirmed) | no FPGA DDC in the default image |
| RTL-SDR (RTL2832U + R820T/R828D) | 225001-300000 and 900001-3200000 S/s; about 2.4-2.56 MS/s without drops | fixed 32-tap FIR at 28.8 MHz, then a resampler | FIR: +-0.9 MHz absolute (0.9 dB), -3 dB at +-1.2 MHz, at any rate [computed]; plus the tuner IF filter, about as wide as the rate below 2.43 MS/s [doc] | about 30 dB beyond 1.2 MHz at 2.048 MS/s [measured] | 2.048-2.4 MS/s, +-0.85 MHz absolute | the filter does not scale with the rate: below 1 MS/s strong aliases |
| HackRF One | 2-20 MS/s, 8 bit; < 8 MS/s not recommended [doc] | none (only the analog MAX2837 filter) | depends on the analog filter | no digital alias filter at all | Fs >= 8 MS/s, filter 0.75 x Fs, +-0.30 | decimate on the host |
| Airspy R2 / Mini | 10 / 2.5 MS/s (R2), 10 / 6 / 3 MS/s (Mini) | real 12-bit ADC at 2 x Fs, Fs/4 shift and 47-tap halfband on the host | "9 MHz alias/image free" of 10 MS/s [doc] | +-0.45 claimed [doc] | +-0.40 | no DC spike, no IQ image (real ADC) |

The main point: **no device is clean to the edge**, and the "80 %" rule of
thumb holds only for the B210 at a power-of-2 decimation and with about
64 dB of alias rejection. The LMS7002M is clean only to 54..64 % of its
host rate. A channel near the edge gets aliases, and a filter bank on the
host cannot remove them: once a signal has folded into the stream, it
sits on the same frequency as the wanted one.

## How aliases, images and the DC get into the stream

These mechanisms apply to every device.

1. **Decimation aliases.** Every decimate-by-R stage folds everything in
   its input band onto its output band: a signal at fo + k (k = +-1, +-2,
   ... in units of the output rate) lands at fo. A halfband (R = 2) has
   its -6 dB point at exactly 0.5 of its output rate, so the region near
   the edge (0.42..0.5) aliases with the region just outside it
   (0.5..0.58). A complex halfband folds a signal at +0.55 Fs onto
   -0.45 Fs. Only the stopband attenuation of the last stage protects the
   edge.
2. **CIC droop and CIC aliasing.** A CIC (sinc^N) is flat only near DC.
   It is used at high decimation because it costs nothing, and a halfband
   after it keeps only the inner part where the droop is small. An odd
   decimation leaves the CIC as the last stage: about -9 dB at 0.4 Fs and
   only about 20 dB of alias rejection (B210 numbers below).
3. **Insufficient analog anti-alias filter.** With no digital
   decimation (HackRF, the B210 at D = 1, the LMS7002M without its
   decimation), the analog filter alone stands between the ADC and the
   signals beyond Fs/2. A third-order filter gives only about 18 dB per
   octave.
4. **LO leakage and DC offset** sit at the LO frequency. In a zero-IF
   receiver (AD936x, LMS7002M, MAX2837) that is the stream centre unless
   the LO is moved by an offset and a digital NCO shifts the band back.
   The chips have DC correction loops (a notch around DC), so a signal
   right at DC is also distorted.
5. **IQ imbalance image.** A gain or phase mismatch between I and Q
   mirrors every signal at +f around the LO to -f with the image
   rejection ratio (LMS7002M -70 dBc after calibration [doc]; B210 spec
   sheet SSB suppression -35 dBc on TX, no RX figure [doc]; AD9361 RX with
   quadrature tracking: no number found, 50..70 dB is my guess
   [inference]; HackRF: no published figure). With the LO offset
   outside the band, the image of the band falls outside it too, but
   strong signals on the other side of the LO image into the band.
6. **Low-IF tuners (R820T, Airspy)** have no DC and no IQ image in the
   stream (the ADC is real). Instead they have an analog image at 2 x IF
   away, which the R820T rejects by about 65 dB [doc].
7. **Strong signals outside the band.** They are not aliased if the
   filters are good, but they still use up ADC headroom before the
   digital filters: an 8-bit ADC (RTL, HackRF) clips on a strong GSM or
   LTE carrier the channel bank never sees. The analog filter is the only
   protection, so it should not be much wider than needed.

## USRP B200 / B210

### Signal chain

| Stage | What | Tag |
|---|---|---|
| RF | AD9361 (B210, 2 RX), AD9364 (B200, 1 RX), 70 MHz-6 GHz, LNA, quadrature mixer (zero IF) | [doc] Ettus spec sheet |
| TIA | single-pole low pass, calibrated to 2.5 x the baseband bandwidth | [doc] UHD `ad9361_device.cpp` `_calibrate_rx_TIAs`, quoting UG-570 |
| BB filter | third-order Butterworth low pass, corner = RF bandwidth / 2, clipped to 0.143..28 MHz, and to MCR / 2 | [doc] same, `_calibrate_baseband_rx_analog_filter` |
| ADC | sigma-delta, ADC clock = MCR x 8..48 (16 for most MCRs) | [doc] `_setup_rates` |
| AD9361 digital | HB3 (/2 or /3), HB2 (/2), HB1 (/2), programmable FIR (/1, /2 or /4, up to 128 taps). UHD loads a fixed 127-tap halfband (`hb127`) for MCR >= 0.66 MHz, a 128-tap /4 filter below | [doc] `ad9361_filter_taps.h`, `_setup_rx_fir` |
| AD9361 output | the master clock rate (MCR), 0.22..61.44 MHz; 1 RX up to 61.44, 2 RX up to 30.72 | [doc] `AD9361_MAX_CLOCK_RATE`, Ettus manual |
| FPGA DDC | CORDIC (NCO), CIC N = 4 with the odd remainder of the decimation, then up to two halfbands, both `hb47` (47 taps, "83% effective BW, 64dB to 85dB rolloff"). No CIC droop compensation. The cores allow a coefficient reload, but the host never does it (`rx_dsp_core_3000.cpp`: "FIXME: Add code to support REG_DSP_RX_COEFFS"), so `hb47` is what runs | [doc] `fpga/usrp3/lib/dsp/ddc_chain.v` (NEW_HB_DECIM = 1 in `b200_core.v`), `coregen_dsp/hbdec1.xco`, `hb47.coe` |
| Host | Fs = MCR / D. D a multiple of 2 enables halfband 1, of 4 both; the rest goes to the CIC | [doc] UHD `rx_dsp_core_3000.cpp` `set_host_rate` |

### How UHD picks the master clock

When no `master_clock_rate` is given (libsdr never gives one), UHD picks
it [doc, `ad936x_manager.cpp` `get_auto_tick_rate`]:

    MCR = r x 2^floor(log2(MCRmax / r)),   r = LCM of all set TX and RX rates,
    MCRmax = 61.44 MHz / number of channels;  a multiplier of 2 becomes 1.

So D is always a power of 2, and never 2:

| Fs (1 RX) | MCR | D | Path |
|---|---|---|---|
| 1.92 MS/s | 61.44 MHz | 32 | CIC 8 + 2 halfbands |
| 3.84 MS/s | 61.44 | 16 | CIC 4 + 2 HB |
| 10 MS/s | 40 | 4 | 2 HB |
| 15.36 MS/s | 61.44 | 4 | 2 HB |
| 16 MS/s | 16 | 1 | AD9361 only |
| 20 MS/s | 20 | 1 | AD9361 only |
| 30.72 MS/s | 30.72 | 1 | AD9361 only |

With 2 RX (or a TX rate that differs from the RX rate, through the LCM),
the limits halve: D = 1 above 7.68 MS/s. A rate that does not divide the
MCR the user forces gives an odd D: UHD then warns "The requested
decimation is odd; the user should expect CIC rolloff" [doc].

### Passband and aliases [computed]

From the FPGA's CIC and `hb47` coefficients (AD9361 stages ignored; they
are far wider than Fs when D >= 2). Droop is relative to DC; "alias" is
the strongest signal anywhere outside the stream that lands at that
frequency, relative to the same signal in the passband.

| D (path) | droop at 0.35 / 0.40 / 0.45 / 0.50 Fs | alias at 0.40 / 0.45 Fs | alias < -60 dB to | < -80 dB to |
|---|---|---|---|---|
| 2 or 4 (1 or 2 HB, no CIC) | 0.00 / 0.01 / -0.58 / -6.0 dB | -66 / -24 dB | +-0.416 | +-0.19 |
| 8 (CIC 2 + 2 HB) | -0.33 / -0.42 / -1.13 / -6.7 | -64 / -24 | +-0.417 | +-0.23 |
| 16..128 (CIC 4..32 + 2 HB) | -0.43 / -0.56 / -1.30 / -6.9 | -67 / -25 | +-0.417 | +-0.27 |
| 6 or 10 (CIC 3 or 5 + 1 HB) | -1.6 / -2.1 / -3.3 / -9.4 | -46..-49 / -28 | +-0.28 | +-0.16 |
| 3 or 5 (CIC only) | -6.5..-7 / -8.7..-9.3 / -11..-12 / -14..-15 | -21..-23 / -18 | +-0.14 | +-0.08 |
| 1 (AD9361 `hb127` FIR only) | 0.00 / 0.00 / 0.00 / -6.0 | -79 at any frequency to 0.45 | +-0.45 | - |

So at the power-of-2 decimations UHD picks, the B210 stream is flat within
0.6 dB to +-0.40 Fs, and alias-free to 64 dB up to +-0.417 Fs, which is
the "83 % effective BW" Ettus wrote in `hb47.coe`. Beyond that it falls
fast: 24 dB at 0.45, 6 dB at the edge. For 80 dB (a weak channel next
to strong ones) only +-0.27 Fs is clean. The CIC droop is smooth and the
same every time: a level correction can take it out (it matters for
osmocalibrate's power readings, not for decoding).

At D = 1 the AD9361 FIR is the last filter: very flat and clean to
+-0.45 Fs. The 16-bit coefficient quantisation limits it to about 79 dB.

### Analog filter

[doc] UHD sets the RX analog bandwidth to 56 MHz at start-up
(`DEFAULT_BANDWIDTH`), and `set_rx_bandwidth(bw)` tunes both the TIA and
the BB filter. The filter is centred on the RF LO, not on the requested
frequency.

[computed, from a third-order Butterworth at bw / 2 and the TIA pole at
2.5 x that] With bw = Fs and the LO at the centre, the band is -1.4 dB at
+-0.40 Fs and -2.4 dB at +-0.45 Fs. Since the FPGA does the alias
filtering when D >= 2, the analog filter there only keeps blockers out of
the ADC. A bw of 1.5 x Fs keeps the band flat to 0.1 dB. At D = 1 that is
not possible: UHD caps the bandwidth at the MCR (= Fs), so the -1.4 dB at
+-0.40 and -2.4 dB at +-0.45 stay; a power reading near the edge needs a
correction there.

### Blockers and gain [doc, inference]

Spec sheet: IIP3 -20 dBm, NF < 8 dB, ADC SFDR 78 dBc. The AD9361's ADC
sees everything the analog filter passes, at the MCR x 16 rate, so a
strong GSM/LTE carrier just outside the stream does not alias (the
digital filters take it out) but uses ADC headroom and raises IMD. With
the LO outside the stream (bw = Fs + 2 |offset|, 2.08..2.2 x Fs) the
filter admits about 1.1..1.2 Fs of spectrum on the far side of the LO
that nobody uses: the price of keeping the LO off the band. In a crowded
band (cellular downlink) put that side where the signals are weak, or
accept the LO in the stream [inference]. Gain: the AD9361 RX gain table
spreads LNA, mixer, TIA and PGA by itself (UHD's single gain value);
libsdr's single RX gain is fine; keep the strongest signal 10..15 dB under
full scale (libsdr's ADC status counters show it).

[ours] With the LO offset outside the stream, the filter centred on the
LO cut the far edge of the band by 21 dB (B210 loopback, 1.92 MS/s); with
bw = Fs + 2 x |offset| the band was flat within 1.8 dB (`sdr.c`
`lo_bandwidth()`). The calculation above agrees: about -18 dB at the far
edge with bw = Fs.

### LO offset

The AD9361 has an RF and a BB DC offset correction and quadrature
tracking [doc, datasheet "dc offset correction, quadrature correction"].
The B210 spec sheet gives SSB / LO suppression of -35 / 50 dBc (TX side)
[doc]. The RX LO leakage and the residual DC sit at the LO.

libsdr puts the LO at centre - 0.54 Fs (`sdr_lo_offset_auto()`), and the
FPGA CORDIC shifts the band back. [computed] The LO leakage is then
outside the stream, but in the halfband's transition band: it is
attenuated only 19.6 dB and folds to +0.46 Fs (outside +-0.417, so outside
the usable band, but visible). At 0.58 Fs it would be 52 dB down and land
at +0.42; at 0.60 Fs, 66 dB down, landing at +0.40.

The offset works only when the AD9361 output (the MCR) is wide enough to
carry the shifted band: |offset| + 0.42 Fs must stay within the AD9361
FIR's +-0.45 MCR [inference]. With D >= 4 that holds (0.96 Fs < 1.8 Fs).
With D = 1 (Fs > 15.36 MS/s on 1 RX, > 7.68 MS/s on 2 RX) it cannot. What
happens then, from UHD's code [doc]: `tune_xx_subdev_and_dsp()` asks the
DSP for a shift of 0.54 MCR without clipping (AUTO policy),
`get_freq_and_freq_word()` wraps it with `wrap_frequency()` to -0.46 MCR,
and `get_rx_freq()` returns RF - DSP, which is then one MCR off the
request. `uhd.c:762` (`uhd_open_device()`, RX) and its TX copy compare the read-back with the
request and fail the open with "Given RX frequency ... is not supported"
[inference: the chain of these three, not run]. So above 15.36 MS/s
(1 RX) with the automatic offset, libsdr on a B210 should refuse to open
rather than lose part of the band silently. Worth one try on the bench;
either way the LO plan must not choose an offset there.

### Rates and USB

| Item | Value | Tag |
|---|---|---|
| Instantaneous bandwidth | "Up to 56 MHz of instantaneous bandwidth in 1x1", "Up to 30.72 MHz ... in 2x2" | [doc] Ettus B200/B210 spec sheet |
| ADC | 12 bit, "ADC Wideband SFDR 78 dBc"; "SSB/LO Suppression -35/50 dBc" | [doc] spec sheet |
| MCR | "any value between 5 MHz and 61.44 MHz (or 30.72 MHz for dual-channel mode)", "rates above 56 MHz are possible, but not recommended"; the AD9361 driver accepts 220 kHz | [doc] UHD manual, UHD source |
| Automatic MCR | "attempts to use the largest possible clock rate as to enable as many half-band filters as possible"; off when `master_clock_rate` is given | [doc] UHD manual |
| Even decimation | "it is strongly desirable for that ratio to be even" | [doc] UHD manual, general |
| Analog filter | "UHD will automatically configure the analog filters to avoid any aliasing (RX)"; "UHD will not allow you to set bandwidths larger than your current master clock rate" | [doc] UHD manual |
| USB 3 | "Host Sample Rate (16b) 61.44 MS/s"; AN-177: 61.44 MS/s at 16 bit, shared by TX and RX ("Half Duplex"); 8-bit samples double it | [doc] spec sheet, Ettus KB AN-177 |
| USB 3 in practice | overruns reported above about 28 MS/s on some hosts | [community] |
| USB 2 | no official B210 figure found | - |
| Measured edge | ODR (B200, TX side, DAB 2.048 MS/s): interpolation 5, 6, 10, 15 gave "shoulders", 8, 12, 16 none; Ettus advised MCR = 8 x rate (divisible by 4: both halfbands) | [measured] |
| Other projects | osmo-trx: MCR 26 MHz for GSM (D = 24 at 4 sps, 96 at 1 sps: divisible by 4), 51.2 MHz for multi-ARFCN | [doc] osmo-trx `UHDDevice.cpp` |
| Good rates | 1.92 x 2^k MS/s (LTE: 1.92, 3.84, 7.68, 15.36, 30.72) with MCR 61.44 or 30.72; any MCR / 2^k | [inference] from the MCR rule |

Note: UHD's own documentation says the analog filter is set "to avoid
any aliasing", but libsdr calls `set_rx_bandwidth()` itself, and that
setting is what the filter gets; with D >= 2 the digital halfbands do the
alias filtering anyway.

## LimeSDR (LMS7002M)

### Signal chain

| Stage | What | Tag |
|---|---|---|
| RF | 3 RX inputs per channel, LNA, mixer, zero IF, 100 kHz (with the NCO) .. 3.8 GHz | [doc] LMS7002M datasheet |
| TIA | single-pole low pass (RXTIA), 12 dB gain range | [doc] |
| RX LPF | 2nd order low band (0.7..18 MHz) or high band (18..80 MHz); with the TIA a 3rd order response. "RX analog filters are tunable from 0.7 MHz to 80 MHz" | [doc] |
| PGA | 32 dB in 1 dB steps | [doc] |
| ADC | 12 bit, "ADC Sampling Rate 160 MHz" (ENOB 9, SFDR 63 dBc); the clock generator gives 5..640 MHz for the converters | [doc] datasheet v3.2 |
| RXTSP | DC correction, IQ gain and phase correction, NCO, three general-purpose FIRs (GFIR, up to 40 taps), decimation by 1, 2, 4, 8, 16 or 32 with HB2 (15 taps) stages and HB1 (31 taps) last, digital AGC | [doc] |
| Host | ADC rate / decimation | [doc] |

Datasheet numbers: "Using such mixed mode filtering (digital and analog)
provides 60 dB anti alias performance and 40 dB adjacent channel rejection
as the worst case scenario"; "IQ Imbalance Image -70 dBc after
calibration"; "Interpolation/Decimation digital filters stop band
suppression 108 dB" [doc].

### Passband and aliases

[doc] The datasheet defines the passband as fp = x . fclk / K (fclk / K =
the host rate) and states: x <= 0.27 for >= 108 dB image suppression,
x <= 0.30 for >= 75 dB, x <= 0.32 for >= 60 dB. The decimation uses the
same filters.

[computed, from the HB1 and HB2 coefficients in the datasheet, K = 2..32,
all the same]

| fo (Fs) | 0.27 | 0.30 | 0.32 | 0.35 | 0.375 | 0.40 | 0.45 |
|---|---|---|---|---|---|---|---|
| droop | 0.000 dB | | | -0.07 | | -0.55 | -2.2 |
| worst alias | -108 | -75 | -60 | -42.6 | -32.2 | -24.6 | |

So the LMS7002M's own decimation is very clean but narrow: 54 % of the
host rate for 108 dB, 64 % for 60 dB. libsdr's 75 % (+-0.375) gets only
32 dB of alias rejection from the digital part; the analog LPF set
to the rate adds only a few dB at 0.625 Fs, where those aliases come from
(third order Butterworth: -6.8 dB [computed]).

[measured, Lime's measurement report] The RX LPF "is intended to be a 3rd
order Chebychev filter with nominally 0.5dB ripple"; LPFL measured 600 kHz
to 14 MHz, LPFH 10 to 70 MHz. So the analog passband itself has about
0.5 dB of ripple. The analog filter's lower limit in LimeSuite is 1.4 MHz
RF bandwidth [community]; narrower channels rely on the digital filters.

### Rates, oversampling, boards

| Item | Value | Tag |
|---|---|---|
| Digital interface | 48 MHz RF bandwidth MIMO, 96 MHz SISO | [doc] datasheet |
| Oversampling | done "inside the Lime chip i.e. before the FPGA"; factors 1..32; "disabled by default if your bandwidth is larger than 20M" (then K = 1: no digital anti-alias filter) | [community] Lime staff on discourse.myriadrf.org |
| LimeSDR USB | Cyclone IV, Cypress FX3 (USB 3), "61.44MSPS (Limited by USB 3.0 data rate)" | [doc] |
| LimeSDR Mini v1.x | MAX 10 FPGA, FTDI FT601 (USB 3); 30.72 MS/s commonly given, not found on an official page | [doc], [community] |
| LimeSDR Mini 2.0 | ECP5, FT601, "Sample Rate: 30.72 MSPS", "RF Bandwidth: 40 MHz" | [doc] |
| IQ image | -70 dBc after calibration | [doc] |
| DC | RX DC correction (averaging loop); a DC spike about 18 dB above the noise floor reported on a Mini | [doc], [community] |
| NCO | 32 bit, "spurious performance better than -114 dBc" | [doc] |

No published RX roll-off measurement versus sample rate was found.

Blockers [doc, inference]: ADC SFDR 63 dBc (lower than the B210's 78),
ENOB 9. The analog LPF is the only filter before the ADC; set it close to
the used span (not much wider) in a crowded band. Gain is LNA + TIA (12
dB) + PGA (32 dB): for strong signals reduce the LNA first, as the TIA
range is small.

## PlutoSDR (AD9363)

The AD9363 is the AD9361's lower grade: 325 MHz-3.8 GHz, "Tunable channel
bandwidth (BW): up to 20 MHz" [doc, AD9363 datasheet]. The analog and
digital chain is the AD9361's (TIA, third-order BB filter, HB3/HB2/HB1,
programmable 128-tap FIR). Pluto has no CIC/halfband DDC like the B210 in
its default FPGA image; the host rate is the AD9363's output rate, so the
FIR the Linux driver loads decides the edges.

| Item | Value | Tag |
|---|---|---|
| Tuning, bandwidth | "325 MHz - 3.8 GHz", "Tunable channel bandwidth: 200 kHz - 20 MHz"; with the `ad9364` attribute 70 MHz-6 GHz (unofficial) | [doc] ADI Pluto specs, [community] |
| Rates | "Variable output data rates: 61.44 MSPS - 65.1 kSPS"; "Baseband rates below 2.083 MSPS (25MSPS/12) require FIR decimation/interpolation to be set"; the lowest rates need the FPGA's decimate-by-8 | [doc] ADI wiki, AD9361 Linux driver page |
| USB 2 | "streams up to 4MSPS with no dropped samples" (ADI); about 6 MS/s without drops measured by rtl-sdr.com | [doc], [measured] |
| FIR | libad9361 `ad9361_set_bb_rate()` loads by rate: up to 20 MS/s a 128-tap FIR decimating by 4 (`fir_128_4`); only if the program calls it | [doc] libad9361 source |
| FIR response | `fir_128_4` alone: flat within 0.03 dB to 0.40 Fs, -0.06 dB at 0.45, -6.6 dB at 0.50, >= 59 dB from 0.6 Fs (the same filter as UHD's `fir_128_x4`: about 52 dB at 0.55, so aliases onto +-0.45 at 52 dB, onto +-0.40 at 59 dB) | [computed] |

So Pluto, with that FIR, behaves like the B210 at D = 1 with a slightly
weaker filter: +-0.40 Fs usable at about 59 dB [computed]. The analog
filter is centred on the LO and limited to the AD9363's output rate as on
the B210 [inference: the same AD936x driver logic in the Linux kernel, not
read], so expect about -1.4 dB at +-0.40 when bw = Fs. Blockers: the same
chip family as the B210 (the AD9363 a lower grade), 12-bit ADC; the same
advice applies. Which FIR the
Pluto firmware loads by default at a given rate was not confirmed: it has
to be read back (`in_voltage_filter_fir_en`, `filter_fir_config`) or
measured.

## RTL-SDR (RTL2832U + R820T / R820T2 / R828D)

### Signal chain

| Stage | What | Tag |
|---|---|---|
| Tuner | R820T: low IF, 3.57 MHz (6 MHz filter) or 4.57 MHz (7/8 MHz filters); narrow filters 0.35..1.7 MHz plus high-pass steps move the IF to 2.3 MHz - bw / 2 | [doc] R820T datasheet, librtlsdr `tuner_r82xx.c` `r82xx_set_bandwidth` |
| Image | R820T image rejection 65 dBc (typ.); "Correct IQ has no effect" on the R820 [measured, HB9AJG] | [doc], [measured] |
| ADC | real, 8 bit at 28.8 MHz; "ENOB is estimated at ~7" | [doc] librtlsdr `DEF_RTL_XTAL_FREQ`, rtl-sdr.com |
| Digital | IF mixed to 0 Hz, a 32-tap symmetric FIR "running at XTal frequency" (default taps from the Windows driver), then a fractional resampler to the host rate (ratio 28.8 MHz x 2^22 / Fs) | [doc] librtlsdr.c |
| Host | 225001..300000 or 900001..3200000 S/s; 3.2 MS/s maximum, 2.4..2.56 MS/s the highest without drops | [doc] librtlsdr, rtl-sdr.com |

### Passband [computed, measured]

The default FIR runs at 28.8 MHz whatever the host rate, so its passband
is fixed in Hz [computed from `fir_default`]: -0.1 dB at 0.5 MHz, -0.6 at
0.8, -1.5 at 1.0, -3.0 at 1.2, -8.6 at 1.6, -20.6 at 2.0 MHz, then about
-35 dB. This matches an analysis of the same taps on the osmocom-sdr list
(2012): "I get a -3dB bandwidth of 1.2MHz, and the stopband rejection of
-35dB indeed starts at around 2MHz". PA3FWM measured at 2.048 MS/s "at
least 30 dB of attenuation everywhere above 1.2 MHz", and at 256 kS/s the
dongle also receives "1 MHz above and below" [measured].

The tuner's IF filter comes on top [doc, `r82xx_set_bandwidth()` with
bw = the rate, librtlsdr 2.0.3]: at 2.048 MS/s it is the narrow mode with
1.7 MHz low pass + one 350 kHz high-pass step, 2.05 MHz in all, IF 1.625
MHz; at 2.4 MS/s 1.7 + 0.35 + 0.38 = 2.43 MHz, IF 1.815 MHz; only above
2.43 MS/s the 6 MHz filter (IF 3.57 MHz). So below 2.43 MS/s the analog IF
filter is about as wide as the rate and adds its own edge roll-off (the
Airspy measurement of the same filter: -3 dB edges at about 495 kHz and
2088 kHz of the IF [measured]). The 2012/2013 measurements above may
predate this narrow-filter code (the dongles then ran with the 6 MHz
filter); the current edge is then somewhat lower than they show
[inference].

Consequences [computed for the FIR, inference for the total]: at 2.048
MS/s the band edge (+-1.024 MHz) is -1.7 dB from the FIR alone; at 2.4
MS/s -3 dB; at 3.2 MS/s the outer 0.4 MHz per side are gone; the IF
filter adds to it. At low rates the FIR does not protect against aliases,
and the resampler's own filter gives only about 30 dB. The usable width is
about +-0.85 MHz in absolute terms (under 1 dB of FIR droop), so
2.048..2.4 MS/s is the useful range; below 1 MS/s, sample at 2.048 MS/s
and decimate on the host.

### Tuning accuracy: the R820T PLL and the RTL2832U DDC

From librtlsdr 2.0.3 (`tuner_r82xx.c` `r82xx_set_pll()`, `librtlsdr.c`;
the copy in `~/.cache/sdrangel-rp-kal/rtl-sdr`; the osmocom master in
`/home/i/fix/osmocom-analog.compandor/rtl-sdr` has the same PLL code)
[doc, the vendor-documented register use is in the driver only]:

- The tuner's LO is RF + IF (low IF: the 1.625..4.57 MHz above). The
  RTL2832U then shifts the IF to 0 Hz digitally.
- PLL: VCO 1.77..3.54 GHz, a mix divider of 2, 4, ... 64 chosen so that
  LO x divider is in that range, and a fractional-N divider
  `nint + sdm/65536` of 2 x the reference (28.8 MHz, the RTL's crystal):

      LO = 2 x Fref x (nint + sdm / 65536) / mix_div,
      vco_div = round(65536 x LO x mix_div / (2 x Fref)),  nint = vco_div / 65536, sdm = vco_div % 65536

  The step is 2 x 28.8 MHz / 65536 / mix_div = 878.9 Hz / mix_div: 439.5 Hz
  at mix_div 2 (LO 885..1770 MHz), 219.7 Hz at 4 (442..885 MHz), 109.9 Hz
  at 8, and so on. The rounding leaves an error of up to half a step:
  +-220 Hz at 900 MHz, +-110 Hz at 800 MHz, different for every
  frequency [computed from the formula].
- The LO hits a frequency exactly only on that grid. With `sdm` = 0 the
  SDM is switched off (`pw_sdm`), a pure integer-N LO.
- The DDC shift is a 22-bit register: `if_freq = -(IF x 2^22 / Fxtal)`,
  integer division, so 6.87 Hz steps, truncated [doc].
- The actual centre (in units of the nominal crystal) is therefore exactly
  computable from the requested frequency and the rate (which sets the
  IF): LO_prog - IF_prog, both from the integer formulas above. The real
  centre is that times (1 + crystal error), since the tuner, the DDC and
  the resampler all run from the one 28.8 MHz crystal.
- `rtlsdr_set_freq_correction(ppm)` takes an integer ppm. It scales the
  crystal value used in the tuner's PLL calculation and writes the
  resampler's sample-frequency correction register (`-ppm x 2^24 / 1e6`),
  then retunes [doc]. Integer ppm is 900 Hz at 900 MHz. libsdr does not
  use it: `--sdr-ppm` scales the requested frequency instead (`soapy.c`
  near line 382), which leaves the sample rate off by the crystal error
  (harmless at a few ppm).
- Consequence for calibration [inference]: a carrier offset measured on an
  RTL is crystal error x f + the PLL rounding (up to +-220 Hz at 900 MHz,
  0.24 ppm; a full step of 439.5 Hz is 0.49 ppm). That is the size of the
  differences seen between our LTE results and kalibrate-rtl. Measured
  ppm values from different frequencies only agree after the rounding is
  taken out.
- The RTL2832U's DDC register could do fine tuning (6.87 Hz steps), but
  librtlsdr does not export it (`rtlsdr_set_if_freq()` is internal) and
  Soapy has no way to reach it [doc].

Sample rates [doc, `rtlsdr_set_sample_rate()`]: the resampler ratio is
`Fxtal x 2^22 / Fs`, rounded down to a multiple of 4; the actual rate is
`Fxtal x 2^22 / ratio` and the library prints "Exact sample rate is" when
it differs. A rate is exact when 28.8 MHz / Fs x 2^20 is an integer:
2.048, 2.4, 2.56, 1.92, 3.2, 1.024 MS/s are exact; 250 kS/s is not
[computed]. The drops above about 2.4..2.56 MS/s are not the USB 2 bus (3.2
MS/s x 2 bytes = 6.4 MB/s, far below it); the vendor gives no reason
[inference].

### Overload

[measured, HB9AJG] Signals inside the R820T's tracking filter ("about +/-3
MHz wide") of about -62..-68 dBm overload it at full gain; reducing the
gain by 11/20/30 dB raises that by 12/20/30 dB; "Do not switch on RTL AGC
or Tuner AGC"; keep signals at most about 60 dB above the noise floor.

## HackRF One

| Stage | What | Tag |
|---|---|---|
| RF | RFFC5072 mixer (to/from 2.3..2.7 GHz IF) and MAX2837 (MAX2839 on r9) zero-IF transceiver | [doc] hackrf.readthedocs.io |
| Baseband filter | MAX2837 RX low pass, settings 1.75, 2.5, 3.5, 5, 5.5, 6, 7, 8, 9, 10, 12, 14, 15, 20, 24, 28 MHz | [doc] libhackrf `max2837_ft` |
| ADC | MAX5864, 8 bit, "isn't specified to operate at less than 8MHz" | [doc] |
| Digital | none on the HackRF One (CPLD glue only); the HackRF Pro has FPGA decimation | [doc] |
| Host | 2..20 MS/s from the Si5351 (integer MHz exact), High Speed USB (20 MS/s x 2 bytes = 40 MB/s) | [doc], [inference] |

The host library's default filter is "the widest available setting that
is no more than 75% of the sample rate" [doc]. There is no digital
anti-alias filter: everything between the filter's edge and Fs folds into
the stream, attenuated only by the analog filter. HackRF's own advice:
"Using a sampling rate of less than 8MHz is not recommended"; at 2 MS/s
the minimum filter attenuates "about 33dB" at +-2 MHz; sample at 8 MS/s and
decimate 4:1 on the host [doc]. The DC spike is at the centre; offset
tuning is the vendor's advice [doc]. Gains: amp 0/11 dB, LNA 0..40 dB in 8
dB steps, VGA 0..62 dB in 2 dB steps; "A good default setting to start with
is RF=0 (off), IF=16, baseband=16" [doc]. No published IQ image figure.

Usable fraction [inference]: with the filter at 0.75 x Fs and its edge
roll-off, +-0.30 Fs is a safe default; the 8-bit ADC makes aliases of
strong signals visible early.

## Airspy (R2, Mini)

R820T2 low IF, 12-bit ADC at 20 MS/s (R2; "10.4 ENOB, 70dB SNR, 95dB
SFDR"), IF = ADC rate / 4, host conversion to IQ: DC removal, Fs/4 shift,
47-tap halfband, decimation by 2 [doc, airspy.com, libairspy
`iqconverter_float.c`]. "10 MHz panoramic spectrum ... up to 9MHz
alias/image free" (R2), "Up to 6 MHz" (Mini) [doc]. No DC spike and no IQ
image (the ADC is real); the R820T's analog image is at 2 x IF. Rates: R2
10 and 2.5 MS/s, Mini 10, 6 and 3 MS/s [doc]. Not supported by libsdr
today except through Soapy (SoapyAirspy is installed here).

## What libsdr does today

| Place | What | Assessment |
|---|---|---|
| `src/libsdr/sdr.c:64`, `sdr_config.c:1153`, `sdr_lo.c:25`, `libcalibrate/calibrate.c:603`, `radio/main.c:2507` | one usable fraction for every device: 75 % (+-0.375 Fs) | right for the B210 (64 dB there), too wide for the LMS7002M (32 dB), about right for the RTL at 2.4 MS/s, too wide for the HackRF with bw = Fs |
| `sdr_config.c:893` | analog bandwidth = the SDR rate unless given | B210 fine when the LO is at the centre (-1.4 dB at +-0.4); HackRF: a filter of about Fs instead of the vendor default 0.75 x Fs leaves no anti-alias margin [inference: what the Soapy module rounds Fs to was not read] |
| `sdr.c:268` `lo_bandwidth()` | bw = Fs + 2 x |offset| when the LO is outside the stream (UHD only) | correct for UHD (measured) |
| `sdr_lo.c:29-31` | automatic LO offset -0.54 Fs | the LO leakage lands at +0.46 Fs at only -20 dB [computed]; 0.60 Fs would give -66 dB at +0.40 |
| `sdr_lo.c:43` | `lo_outside` = |offset| >= 0.5 Fs, for UHD | does not check the MCR: at D = 1 (Fs > 15.36 MS/s 1 RX, > 7.68 MS/s 2 RX) the shifted band does not fit the AD9361 output; UHD wraps the DSP shift, the read-back is one MCR off and `uhd.c:762` fails the open [doc chain, not run] |
| `sdr.c:485-488` + `soapy.c:307-321` | the same automatic offset is handed to SoapySDR, which applies it when the device has a tuning stage (LimeSDR NCO "BB", UHD through SoapyUHD); `lo_applies` is set only for UHD | for a Soapy device that applies it, `lo_outside` stays 0, so `lo_bandwidth()` does not widen the analog filter: the same tilt that cost 21 dB on the B210 [inference: needs a check on a LimeSDR] |
| `sdr.c:886-888` (and the TX copy at about 760) | one channel: the centre moved one channel bandwidth below it, against the DC, also when the LO is outside the stream | harmless, uses span for nothing |
| `sdr.c:1303` `sdr_calculate_optimal_rate()` | halves the master rate while >= 2.67 x the band; fallback: a power of 2 in Hz | the fallback is a rate that does not divide the master rate (only when the master rate is below the need) |
| `sdr_config.c:1216` `sdr_select_optimal_rate()` | smallest rate the device lists that is >= band / 0.75 and a multiple x1..x64 of the DSP rate | does not prefer B210-friendly rates (MCR / 2^k); for "as wide as possible" there is no rule |
| `uhd.c` | no `master_clock_rate`: UHD's automatic choice | good (power-of-2 D), but D = 1 above MCRmax / 4 is not taken into account by the LO plan |
| RTL tuning (`soapy.c` setFrequency, `--sdr-ppm` scaling) | the requested centre is taken as the true one | the true centre is off by the R820T PLL rounding (up to +-220 Hz at 900 MHz) plus up to 6.9 Hz of DDC truncation; it biases ppm results by up to 0.24..0.49 ppm [computed from librtlsdr's formulas] |

## Recommendations

### Rate selection, per device

- **B210 / B200 (UHD)**: choose Fs = MCR / 2^k with k >= 2 and MCR <=
  61.44 MHz (1 RX) or 30.72 MHz (2 RX): 1.92, 3.84, 7.68, 15.36 MS/s with
  MCR 61.44 (or 30.72), and any multiple of the DSP rate that keeps D a
  power of 2 >= 4. Above MCRmax / 4 (15.36 / 7.68 MS/s) the device runs at
  D = 1: allowed up to MCRmax, but with the LO at the centre (no offset),
  usable +-0.42. Never an odd D, never D = 6 or 10. An explicit
  `master_clock_rate` = 2 x Fs gives D = 2 with one `hb47`, as clean as D =
  4 [computed]; UHD's automatic choice skips it.
- **LimeSDR**: decimation 2..32 always (never K = 1: no digital anti-alias
  filter; the driver's default turns oversampling off above 20 MHz of
  bandwidth), host rate >= span / 0.6, at most 61.44 MS/s (USB), 30.72
  MS/s (Mini).
- **PlutoSDR**: <= 4 MS/s at 16 bit over USB 2 (ADI's figure; 6 MS/s
  measured by others); below 2.083 MS/s the FIR must decimate.
- **RTL-SDR**: 2.048 or 2.4 MS/s (2.56 at most), never 300k..900k; for a
  narrower need take 2.048 MS/s and decimate on the host, rather than a
  low rate with the fixed FIR.
- **HackRF**: 8..20 MS/s, integer MHz; for narrow needs 8 MS/s and
  decimate on the host.
- **Unknown Soapy device**: take `getSampleRateRange()` / `listSampleRates()`
  (libsdr already does), `getBandwidthRange()` / `listBandwidths()` (also
  queried in `soapy.c:1287`), and use the default usable fraction below.
  Soapy has no call for the digital filter's passband; it must come from a
  per-driver table (keyed by `getDriverKey()` / `getHardwareKey()`).

### Usable fraction (per side of the centre)

| Device | Default | Strict (>= 80..100 dB alias) |
|---|---|---|
| B210, D >= 2 | 0.40 | 0.27 |
| B210, D = 1 | 0.42 | 0.42 (79 dB floor) |
| LimeSDR | 0.30 | 0.27 |
| PlutoSDR | 0.40 (to measure) | - |
| RTL-SDR | +-0.85 MHz absolute (0.40 at 2.048 MS/s, 0.35 at 2.4) | - (about 30 dB) |
| HackRF | 0.30 | - (analog only) |
| Airspy | 0.40 | 0.45 claimed |
| unknown | 0.35 | - |

The channel bank should take the fraction from a per-device table, not one
constant, and osmocalibrate's scan step (`calibrate.c:603`) should use the
same table. Near the edge a channel is still decoded, but its power
reading needs the droop correction (B210: the CIC+HB droop is known
exactly; RTL: the fixed FIR is known exactly).

### Analog bandwidth

- B210: 1.5 x Fs with the LO at the centre (flat to 0.1 dB), Fs + 2 x
  |offset| (+ margin) with the LO outside, capped at 56 MHz and at the MCR
  (UHD clips it there).
- LimeSDR: at least the used span x 1.2, at least 1.4 MHz (the LPF's
  lower limit); when the LO offset is applied by the NCO, the same
  Fs + 2 x |offset| as UHD.
- HackRF: 0.75 x Fs (the vendor's default), or less if the channels use
  less: it is the only anti-alias filter.
- RTL: the tuner's automatic choice from the rate (narrow mode, about the
  rate wide, below 2.43 MS/s; 6 MHz above) is acceptable; for the
  flattest band at 2.4 MS/s a bandwidth just above 2.43 MHz selects the 6
  MHz filter (wider: more blocker exposure on an 8-bit ADC) [inference].

### Tuning an RTL so the true centre is known

1. Compute, in libsdr, what librtlsdr will program: the IF from the rate
   (`r82xx_set_bandwidth()` rules), LO = requested + IF, `vco_div` by the
   formula above, LO_prog, IF_prog (22-bit), centre_prog = LO_prog -
   IF_prog.
2. Better: request a centre that lands on the grid. Choose LO_prog on the
   PLL grid nearest to (wanted + IF) and request LO_prog - IF; put the
   remaining difference (wanted - centre_prog, under half a step plus the
   6.87 Hz DDC step) into our own NCO (the channel offsets libsdr already
   applies). Then the stream's centre is known to well under 1 Hz in
   nominal-crystal units, and only the crystal error is left, which is
   exactly what calibration wants to measure.
3. This depends on the librtlsdr version's PLL code (the formula above is
   2.0.3 and current osmocom master; older releases computed the SDM in a
   different way). Check it once per version with a known signal (B210
   TX at a GPSDO-locked frequency, or an LTE/GSM carrier at two
   frequencies with different rounding).
4. Rates: only exact ones (2.048, 2.4, 2.56, 1.92 MS/s ...), never the
   300k..900k gap, at most 2.56 MS/s.
5. Do not use `rtlsdr_set_freq_correction()` (integer ppm); keep
   `--sdr-ppm` as a frequency scale, and apply it before step 1 so the
   rounding is computed on the frequency actually programmed.
- In general: narrow enough to keep strong out-of-band signals out of the
  ADC (8-bit devices especially), wide enough to keep the used span within
  about 0.5 dB.

### LO offset

- UHD with D >= 4: keep the LO outside the stream, but at 0.60 Fs (guard
  0.10 instead of 0.04): the leakage is then 66 dB down at +0.40 instead of
  20 dB at +0.46 [computed]. The analog bandwidth becomes 2.2 x Fs.
- UHD with D = 1 (or D = 2): no automatic offset (LO at the centre); the
  channels are placed off the DC as for Soapy.
- Soapy: apply the offset only where libsdr also widens the analog filter
  (treat an applied offset as `lo_applies`), or do not apply it; for the
  LMS7002M the NCO runs at the ADC rate, so the same "does it fit" rule as
  the B210 applies (K >= 2).
- With the LO outside the band, the IQ image of the band falls outside it,
  but signals on the far side of the LO (0.68..1.48 Fs below the centre at
  offset -0.54 Fs) image into the band at the image rejection ratio (50..70
  dB). Put the LO on the side with fewer strong signals (for a cellular
  downlink, the side of the band edge) [inference].

### DC and IQ image when the LO is at the centre

- Keep channels at least one channel bandwidth off the centre (libsdr
  does), and avoid placing two channels mirror-symmetric around the centre
  when one is much stronger: its image lands on the other at -50..-70 dB.
- The AD936x and LMS7002M DC loops notch a narrow band around DC: no
  channel there.

## Tests that would verify this

Offline (no radio):

1. Put the coefficient model (CIC + `hb47`, `hb127`, LMS7002M HB1/HB2, RTL
   FIR) into `src/test/` as a table check of the usable fraction per
   device, so the table and the code agree.
2. osmocalibrate captures of a GSM band at two centre frequencies 0.2 Fs
   apart: a real carrier moves by 0.2 Fs, an alias moves the other way or
   not at all (the HB9AJG method). Gives the alias level near the edges
   with real strong signals.

B210 TX -> RX cable loopback (TX gain 40, RX 10, with attenuation):

3. **Tone sweep.** TX a single tone from a TX centre that moves with it
   (the tone always at +100 kHz from the TX centre, so the TX's own
   images stay known), RX fixed; step the tone from -0.7 Fs to +0.7 Fs in
   0.01 Fs steps. At each step read the level at the tone (passband
   droop), at its alias position (rejection), at its mirror -f
   around the RX LO (IQ image) and the RX LO position (leakage). Run it at
   1.92 MS/s (D = 32), 15.36 MS/s (D = 4), 20 MS/s (D = 1), and with a
   forced odd D for reference; with the LO offset at 0, -0.54 and -0.60 Fs;
   with bw = Fs, 1.5 Fs and Fs + 2 |offset|. Expected: the tables above.
4. **LO offset at D = 1.** 20 MS/s, automatic offset, tone at +0.3 Fs:
   with the RX on its own (TX from a second device or a fixed TX centre),
   the tone should be missing or attenuated if the inference above is
   right.
5. **Same sweep with the RTL-SDR and a LimeSDR as the RX** (the B210 as
   the source, through an attenuator): RTL FIR edge, the LMS7002M's
   +-0.27..0.32 edge, and whether the Soapy LO offset tilts the LimeSDR
   band.

## Method (computed numbers)

The digital responses were computed with numpy from the documented
coefficients: the B200 FPGA's `hb47.coe` (both halfbands) and a CIC with
N = 4, M = 1, R = the remainder of the decimation, in the order CIC,
halfband 1, halfband 2 (`ddc_chain.v`); UHD's AD9361 FIR tables
(`ad9361_filter_taps.h`); the LMS7002M datasheet's HB1 and HB2
coefficients in the order HB2 ... HB2, HB1; librtlsdr's `fir_default` at
28.8 MHz. For each output frequency the worst alias is the largest
response over all input frequencies that fold onto it. Fixed-point
rounding inside the filters and the analog stages are not included.
Scripts (not in the tree): `~/.cache/sdrangel-rp-frontends/` - `ddc2.py`
(B200 DDC, with `hb47.coe`, `hbdec1.xco`, `hbdec2.xco` from the UHD
repository), `hb.py` (AD9361 FIR tables, reads the local UHD checkout in
`/home/i/uhd`), `lms.py` (LMS7002M halfbands). Run with `python3 -I`.

UHD and librtlsdr source were read for the filter taps, the clock rules
and the PLL, because that is where the hardware's configuration is
written down; the SoapySDR modules were not studied.

## Sources

Device documentation and datasheets:

- Ettus USRP B200/B210 spec sheet (copy at Digi-Key): https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/7161/b200_b210_spec_sheet-3236990.pdf
- UHD manual, B200 series: https://uhd.readthedocs.io/en/latest/page_usrp_b200.html
- UHD source (read locally, the same on GitHub): https://github.com/EttusResearch/uhd
  - `host/lib/usrp/common/ad936x_manager.cpp` (`get_auto_tick_rate`)
  - `host/lib/usrp/b200/b200_io_impl.cpp` (`set_auto_tick_rate`)
  - `host/lib/usrp/cores/rx_dsp_core_3000.cpp` (`set_host_rate`, odd decimation warning)
  - `host/lib/usrp/common/ad9361_driver/ad9361_device.cpp` (`_setup_rates`, filter calibration), `ad9361_filter_taps.h`
  - `host/lib/usrp/multi_usrp.cpp` (`tune_xx_subdev_and_dsp`)
  - `fpga/usrp3/lib/dsp/ddc_chain.v`, `fpga/usrp3/top/b200/b200_core.v`
  - `fpga/usrp3/top/b200/coregen_dsp/hbdec1.xco`, `hbdec2.xco`, `hb47.coe`
- AD9361 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD9361.pdf (datasheet text read from a local copy)
- AD9361 reference manual UG-570 (not opened here; quoted through UHD's comments): https://www.analog.com/media/en/technical-documentation/user-guides/AD9361_Reference_Manual_UG-570.pdf
- AD9363 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD9363.pdf (local copy)
- HackRF documentation: https://hackrf.readthedocs.io/en/latest/hackrf_one.html, https://hackrf.readthedocs.io/en/latest/sampling_rate.html, https://hackrf.readthedocs.io/en/latest/setting_gain.html, https://hackrf.readthedocs.io/en/latest/troubleshooting.html, https://hackrf.readthedocs.io/en/latest/gateware.html
- libhackrf: https://github.com/greatscottgadgets/hackrf/blob/master/host/libhackrf/src/hackrf.c
- MAX5864 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX5864.pdf (local copy)
- R820T datasheet: https://rtl-sdr.com/wp-content/uploads/2013/04/R820T_datasheet-Non_R-20111130_unlocked.pdf
- librtlsdr: https://github.com/osmocom/rtl-sdr/blob/master/src/librtlsdr.c, https://github.com/osmocom/rtl-sdr/blob/master/src/tuner_r82xx.c
- RTL-SDR Blog: https://www.rtl-sdr.com/about-rtl-sdr/, https://www.rtl-sdr.com/v4/
- Airspy: https://airspy.com/airspy-r2/, https://airspy.com/airspy-mini/, https://airspy.com/airspy-hf-plus/, https://github.com/airspy/airspyone_host/blob/master/libairspy/src/iqconverter_float.c

- UHD manual, general (decimation): https://files.ettus.com/manual/page_general.html
- Ettus KB AN-177, USRP bandwidths and sampling rates: https://kb.ettus.com/About_USRP_Bandwidths_and_Sampling_Rates
- osmo-trx UHD device: https://gitea.osmocom.org/cellular-infrastructure/osmo-trx/raw/branch/master/Transceiver52M/device/uhd/UHDDevice.cpp
- AD9361 Linux driver: https://developer.analog.com/docs/linux/drivers/iio-transceiver/ad9361.html
- libad9361 `ad9361_baseband_auto_rate.c`: https://github.com/analogdevicesinc/libad9361-iio/blob/main/ad9361_baseband_auto_rate.c
- ADALM-Pluto specifications: https://wiki.analog.com/university/tools/pluto/devs/specs
- LMS7002M datasheet v3.2 and measurement report: https://github.com/myriadrf/LMS7002M-docs (`LMS7002M_Data_Sheet_v3.2r00.pdf`, `LMS7002M_Measurements-v1_05.pdf`)
- LimeSDR boards: https://limemicro.com/products/boards/limesdr-mini/, https://wiki.myriadrf.org/LimeSDR-USB_hardware_description, https://wiki.myriadrf.org/LimeSDR-Mini_v1.1_hardware_description
- Lime forum, oversampling and LPF: https://discourse.myriadrf.org/t/oversampling-implication-for-data-rate-to-host/3453, https://discourse.myriadrf.org/t/lms7002m-rx-lpf-bandwidth/1302, https://discourse.myriadrf.org/t/minimum-bandwidth-of-the-limesdr/1452

Measurements and statements by others:

- ODR, USRP B200 measurements (interpolation ratios, shoulders): https://wiki.opendigitalradio.org/USRP_B200_Measurements
- usrp-users on odd decimation and CIC roll-off: https://go.mail-archive.com/usrp-users@lists.ettus.com/msg04328.html, https://go.mail-archive.com/usrp-users@lists.ettus.com/msg17998.html, https://mail-archive.com/usrp-users@lists.ettus.com/msg10249.html
- rtl-sdr.com, Pluto USB throughput: https://rtl-sdr.com/some-further-tests-with-the-adalm-pluto-sdr/

- HB9AJG, "Some Measurements on DVB-T Dongles with E4000 and R820T Tuners" (2013): https://wiki.kewl.org/_media/downloads:hb9ajg.pdf
- PA3FWM, RTL-SDR filter measurements: https://pa3fwm.nl/technotes/tn20.html
- osmocom-sdr list on the RTL2832U FIR (2012): https://lists.osmocom.org/pipermail/osmocom-sdr/2012-July/000324.html, https://lists.osmocom.org/pipermail/osmocom-sdr/2012-July/000331.html
- R820T IF filter on the Airspy: https://tleconte.github.io/R820T/r820IF.html
