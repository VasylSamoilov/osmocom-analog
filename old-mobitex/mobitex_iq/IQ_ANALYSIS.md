# Mobitex IQ Capture Analysis

Decoder: `decode_mobitex2.py` + inline FM demod (Butterworth LPF + clock-tracked zero-crossing).

---

## HongKongInternationalAirportSDRSharp_20170914_185539Z_868990901Hz_IQ.wav

- **Center:** 868.9909 MHz  
- **SR:** 39062 Hz (~19 kHz BW each side)  
- **Duration:** 17.1s  
- **Signal power:** very weak (~6µW normalized)

**Result:** 58 header-only frames decoded, 0 data blocks.

| Network | frsync | Area | Base | Frames |
|---------|--------|------|------|--------|
| EE16 (European) | 0xEE16 (idx4) | 59 | 58 | 30 |
| EE16 (European) | 0xEE16 (idx4) | 57 | 58 | 26 |
| EE16 (European) | 0xEE16 (idx4) | 57 | 43 | 1 |

**CFlags distribution:** cflags=1 (59 frames), cflags=3 (1 frame) — all header-only.

This is a **capture of two base stations** (AreaID=57 and AreaID=59, both BaseID=58) transmitting
header-only frames. No SVP/FRI/MRM data blocks were present — this is the idle filler cadence,
or the signal was too weak for block decode (power ~6µW).

The **bitsync direction is MOB (0x3333)** — this capture is monitoring the **uplink** (mobile→base)
frequency. At 868.990 MHz with these area/base IDs this is consistent with a Hong Kong Mobitex
network (possibly the airport ground transport dispatch system active in 2017).

---

## HongKongInternationalAirportSDRSharp_20170914_185618Z_868822805Hz_IQ.wav

- **Center:** 868.8228 MHz  
- **SR:** 39062 Hz  
- **Duration:** 17.7s  

**Result:** 0 frames — signal too weak or not on the exact channel (demod range [-1199, +11249] Hz
is asymmetric, suggesting the captured signal is at an offset within the recorded band).

---

## HongKongInternationalAirportSDRSharp_20170914_185646Z_868765674Hz_IQ.wav

- **Center:** 868.7657 MHz  
- **SR:** 39062 Hz  
- **Duration:** 17.5s  
- **Signal power:** ~0 (essentially noise floor)

**Result:** 0 frames — no signal.

---

## Mobitex-8000 LQ IQ.wav

- **Format:** 2ch 16-bit PCM @ 48kHz (standard WAV, not float32)  
- **Duration:** 27.3s  
- **Signal power:** 0.0225 (moderate)  
- **FM demod freq std:** 649 Hz (vs expected ~2000 Hz for ±2kHz GMSK)

**Result:** 0 frames — below minimum SNR for reliable bit recovery.

The low freq deviation std (649 Hz vs 2000 Hz expected) indicates the signal is either:
1. Very noisy (SNR too low for zero-crossing demod)
2. Received through a narrow IF filter that suppresses deviation
3. At a significantly different deviation than 2 kHz

The file is named "LQ" (Low Quality) — consistent with marginal SNR.
A proper GMSK matched filter or Viterbi decoder would be needed to recover bits from this capture.

---

## Summary

The Hong Kong Airport captures confirm:
- European Mobitex network at 868 MHz (frsync 0xEE16)  
- Two base stations alternating: AreaID=57,59 / BaseID=58
- All-header-only frames (cflags=1) — uplink filler cadence
- Consistent with the 868828 discriminator-audio capture that showed cflags=2 (Silence) frames

The LQ IQ file requires better demodulation (GMSK matched filter) to decode.
The 868765 and 868822 files have no recoverable signal at their nominal center frequencies.
