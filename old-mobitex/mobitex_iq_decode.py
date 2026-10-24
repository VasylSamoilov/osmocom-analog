#!/usr/bin/env python3
"""
Mobitex IQ capture decoder.
Handles SDR# IQ WAV files (float32 stereo or int16 stereo).
Steps:
  1. Frequency correction (find signal peak, shift to DC)
  2. FM demodulation (instantaneous frequency)
  3. Butterworth LPF at baud_rate/2
  4. Resample to integer samples-per-bit (48kHz for 8kbaud = 6 samp/bit)
  5. Zero-crossing clock-tracked bit extraction
  6. Framesync hunt (0xCCCC/0x3333 bitsync + known framesync words)
  7. Header FEC decode
  8. Data block: descramble → deinterleave → (12,8) Hamming FEC → CRC-16
  9. MPAK/SVP/FRI protocol decode
"""

import sys
import os
import struct
import argparse
import numpy as np
from scipy import signal as sp
from scipy.signal import resample_poly
from math import gcd
from collections import Counter

# ── Protocol constants ────────────────────────────────────────────────────────
BAUD = 8000

FRSYNCS = {
    0: 0xC4D7, 1: 0xB433, 2: 0xA2F7, 3: 0xEB90, 4: 0xEE16,
    5: 0x4BCC, 6: 0x3B48, 7: 0x5D08, 8: 0x146F, 9: 0x11E9,
    10: 0xEB23, 11: 0xEF45, 12: 0x09D7, 13: 0x6877, 14: 0x14DC,
    15: 0x10BA, 16: 0xF628, 17: 0x9788,
}
FRSYNC_MAP = {v: k for k, v in FRSYNCS.items()}

BITSYNC_BASE   = 0xCCCC   # base → mobile
BITSYNC_MOBILE = 0x3333   # mobile → base

FNAMES = {
    0x01: 'MRM',  0x02: 'ACK',  0x03: 'NAK',  0x04: 'REB',  0x05: 'RES',
    0x06: 'ABD',  0x07: 'ABL',  0x08: 'ATD',  0x09: 'ATL',
    0x0C: 'BKD',  0x0D: 'BKT',  0x0E: 'FRI',  0x0F: 'SVP',
    0x10: 'TST',  0x11: 'AKT',  0x15: 'AAT',
}
DTE_NAMES = {
    1: 'LOGINREQ', 2: 'LOGINGRA', 3: 'LOGINREF', 4: 'LOGOUT',
    5: 'LOGOUTORD', 6: 'BORN', 7: 'ACTIVE', 8: 'INACTIVE',
    9: 'DIE', 10: 'LIVE', 11: 'ROAMORD', 12: 'ROAM',
    15: 'GROUPLIST', 16: 'FLEXREQ', 17: 'FLEXLIST',
    18: 'INFOREQ', 19: 'INFO', 20: 'TIME', 21: 'AREALIST',
    22: 'ESNREQ', 23: 'ESNINFO',
}
PSUB_NAMES = {1: 'TEXT', 2: 'DATA', 3: 'STATUS', 4: 'HPDATA'}

# ── FEC ───────────────────────────────────────────────────────────────────────
H3, H2, H1, H0 = 0xEC, 0xD3, 0xBA, 0x75

def ones8(v):
    return bin(v & 0xFF).count('1')

def fec_check(cw12):
    """Check/correct 12-bit codeword (8 data MSB + 4 FEC LSB).
    Returns (corrected_data_byte, error_count 0/1/2)."""
    data = (cw12 >> 4) & 0xFF
    fec  =  cw12 & 0x0F
    s = ((ones8(data & H3) & 1) << 3 | (ones8(data & H2) & 1) << 2 |
         (ones8(data & H1) & 1) << 1 |  ones8(data & H0) & 1)
    if (s ^ fec) == 0:
        return data, 0
    for bit in range(8):
        d2 = data ^ (1 << bit)
        s2 = ((ones8(d2 & H3) & 1) << 3 | (ones8(d2 & H2) & 1) << 2 |
              (ones8(d2 & H1) & 1) << 1 |  ones8(d2 & H0) & 1)
        if s2 == fec:
            return d2, 1
    for bit in range(4):
        if s == (fec ^ (1 << bit)):
            return data, 1   # FEC bit error, data intact
    return data, 2            # uncorrectable

# ── CRC-16 ────────────────────────────────────────────────────────────────────
CRC_POLY = 0x0811
CRC_SR0  = 0xF8E7
CRC_CR0  = 0x2A5D

def crc16_mobitex(data18):
    sr, cr = CRC_SR0, CRC_CR0
    for byte in data18:
        for bit in range(8):
            if (byte >> bit) & 1:
                cr ^= sr
            if sr & 0x8000:
                sr = ((sr << 1) & 0xFFFF) ^ CRC_POLY
            else:
                sr = (sr << 1) & 0xFFFF
    return cr

# ── Scrambler / deinterleave / block decode ───────────────────────────────────
def deinterleave(bits240):
    r = [0] * 240
    for col in range(12):
        for row in range(20):
            r[row * 12 + col] = bits240[col * 20 + row]
    return r

def decode_block(bits240, scr_state):
    """Descramble → deinterleave → FEC → CRC.
    Returns (data18, crc_ok, fec_error_count, new_scr_state)."""
    scr = scr_state
    descrambled = []
    for b in bits240:
        descrambled.append(b ^ (scr & 1))
        tap = ((scr >> 0) & 1) ^ ((scr >> 4) & 1)
        scr = (scr >> 1) | (tap << 8)

    deint = deinterleave(descrambled)
    data = []
    errs = 0
    for i in range(20):
        cw = 0
        for j in range(12):
            cw = (cw << 1) | deint[i * 12 + j]
        val, e = fec_check(cw)
        data.append(val)
        errs += e

    data18 = data[:18]
    rx_crc = (data[18] << 8) | data[19]
    calc_crc = crc16_mobitex(data18)
    return data18, (calc_crc == rx_crc), errs, scr

# ── WAV reader ────────────────────────────────────────────────────────────────
def read_iq_wav(path):
    """Read IQ WAV (float32 or int16, mono or stereo).
    Returns (iq_complex_array, sample_rate)."""
    with open(path, 'rb') as f:
        riff = f.read(12)
        if riff[:4] != b'RIFF':
            raise ValueError("Not a RIFF file")
        fmt_tag = n_ch = sr = bps = None
        while True:
            chunk_hdr = f.read(8)
            if len(chunk_hdr) < 8:
                break
            tag  = chunk_hdr[:4]
            size = struct.unpack('<I', chunk_hdr[4:])[0]
            if tag == b'fmt ':
                fmt = f.read(size)
                fmt_tag = struct.unpack('<H', fmt[0:2])[0]
                n_ch    = struct.unpack('<H', fmt[2:4])[0]
                sr      = struct.unpack('<I', fmt[4:8])[0]
                bps     = struct.unpack('<H', fmt[14:16])[0]
                if size > 16:
                    f.read(size - 16)
            elif tag == b'data':
                raw = f.read(size)
                if fmt_tag == 3:      # IEEE float
                    s = np.frombuffer(raw, dtype=np.float32)
                elif fmt_tag == 1:    # PCM int
                    if bps == 16:
                        s = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                    elif bps == 32:
                        s = np.frombuffer(raw, dtype=np.int32).astype(np.float32) / 2**31
                    else:
                        raise ValueError(f"Unsupported PCM {bps}-bit")
                else:
                    raise ValueError(f"Unsupported WAV format {fmt_tag}")
                if n_ch == 2:
                    iq = s[0::2] + 1j * s[1::2]
                else:
                    iq = s.astype(complex)
                return iq, sr
            else:
                f.seek(size, 1)
    raise ValueError("No data chunk found")

# ── Signal analysis ───────────────────────────────────────────────────────────
def find_freq_offset(iq, sr, search_bw=None):
    """Find dominant carrier offset from center using Welch PSD."""
    n = min(len(iq), sr * 4)
    f, pxx = sp.welch(iq[:n], sr, nperseg=min(4096, n // 4),
                      return_onesided=False)
    f   = np.fft.fftshift(f)
    pxx = np.fft.fftshift(pxx)
    if search_bw:
        mask = np.abs(f) <= search_bw
        peak_idx = np.argmax(pxx * mask)
    else:
        peak_idx = np.argmax(pxx)
    return float(f[peak_idx])

def measure_snr(iq, sr, freq_offset=0.0):
    """Estimate SNR in the signal band after frequency correction."""
    t = np.arange(len(iq)) / sr
    iq_c = iq * np.exp(-2j * np.pi * freq_offset * t)
    f, pxx = sp.welch(iq_c, sr, nperseg=4096, return_onesided=False)
    f = np.fft.fftshift(f); pxx = np.fft.fftshift(pxx)
    sig   = np.mean(pxx[np.abs(f) < 8000])
    noise = np.mean(pxx[np.abs(f) > 20000])
    snr_db = 10 * np.log10(sig / noise) if noise > 0 else 0.0
    return snr_db

# ── Demodulation ──────────────────────────────────────────────────────────────
def demodulate(iq, sr, freq_offset=0.0, target_sr=48000):
    """FM demodulate IQ → discriminator output at target_sr."""
    # 1. Frequency correction
    if abs(freq_offset) > 10:
        t = np.arange(len(iq)) / sr
        iq = iq * np.exp(-2j * np.pi * freq_offset * t)

    # 2. Resample to target_sr (must be multiple of BAUD for integer SPB)
    if sr != target_sr:
        g = gcd(target_sr, sr)
        iq = resample_poly(iq, target_sr // g, sr // g)

    # 3. FM demodulation: instantaneous frequency
    phase = np.angle(iq)
    freq  = np.diff(np.unwrap(phase)) * target_sr / (2 * np.pi)

    # 4. Low-pass filter at BAUD/2 to remove noise above the signal band
    b, a = sp.butter(5, (BAUD * 0.6) / (target_sr / 2), btype='low')
    demod = sp.filtfilt(b, a, freq)

    return demod, target_sr

# ── Bit extraction ────────────────────────────────────────────────────────────
def extract_bits(demod, sr, phase_offset=0):
    """Extract bits at fixed clock (sr/BAUD samples per bit)."""
    spb = sr / BAUD   # samples per bit (6.0 at 48kHz, exact)

    # Simple fixed-step sampling — most reliable for GMSK
    bits = []
    pos = float(phase_offset) + spb * 0.5
    while pos < len(demod):
        bits.append(0 if demod[int(pos)] >= 0 else 1)
        pos += spb

    return bits, spb

# ── Frame decoder ─────────────────────────────────────────────────────────────
def decode_bits(bits):
    """Hunt for framesync, decode header + data blocks."""
    frames = []
    shift = 0
    i = 0
    N = len(bits)

    while i < N:
        b = bits[i]; i += 1
        shift = ((shift << 1) | b) & 0xFFFFFFFF
        upper = (shift >> 16) & 0xFFFF
        lower =  shift        & 0xFFFF

        if upper not in (BITSYNC_BASE, BITSYNC_MOBILE):
            continue
        if lower not in FRSYNC_MAP:
            continue

        frsync_idx = FRSYNC_MAP[lower]
        direction  = 'BASE' if upper == BITSYNC_BASE else 'MOB'
        t_approx   = i / BAUD

        # 24-bit header (two 12-bit FEC codewords)
        if i + 24 > N:
            break
        head = bits[i:i + 24]; i += 24
        cw1 = cw2 = 0
        for x in range(12): cw1 = (cw1 << 1) | head[x]
        for x in range(12): cw2 = (cw2 << 1) | head[x + 12]
        cb1, e1 = fec_check(cw1)
        cb2, e2 = fec_check(cw2)
        if e1 > 1 or e2 > 1:
            continue   # bad header FEC, skip

        base_id = (cb1 >> 2) & 0x3F
        area_id = ((cb1 & 0x03) << 4) | ((cb2 >> 4) & 0x0F)
        cflags  =  cb2 & 0x0F

        rec = {
            't': t_approx, 'dir': direction, 'frsync': frsync_idx,
            'base_id': base_id, 'area_id': area_id, 'cflags': cflags,
            'head_fec': e1 + e2,
        }

        if cflags != 0:
            # Header-only frame (filler / Silence / FRI-cf4)
            cf_name = {1: 'hdr-cf1', 2: 'Silence', 4: 'FRI-hdr', 0xF: 'filler'}.get(
                cflags, f'hdr-cf{cflags}')
            rec['frame'] = cf_name
            frames.append(rec)
            continue

        # Data blocks follow
        scr = 0x1E   # scrambler seed
        blocks = []

        for blk_num in range(32):    # max 32 blocks per MIS spec
            if i + 240 > N:
                break
            blk_bits = bits[i:i + 240]; i += 240
            data18, crc_ok, fec_errs, scr = decode_block(blk_bits, scr)
            blocks.append({'data': data18, 'crc_ok': crc_ok, 'fec_errs': fec_errs})

            if blk_num == 0:
                # Parse link control from primary block
                dest_man   = (data18[0] << 16) | (data18[1] << 8) | data18[2]
                frame_id   =  data18[3] & 0x1F
                seq_num    =  data18[4] & 0x0F
                bytes_last = (((data18[3] >> 5) & 1) << 4) | ((data18[4] >> 4) & 0x0F)
                block_len  =  data18[5]
                rec.update({
                    'frame': FNAMES.get(frame_id, f'0x{frame_id:02X}'),
                    'frame_id': frame_id, 'dest_man': dest_man,
                    'seq': seq_num, 'bytes_last': bytes_last,
                    'block_len': block_len,
                })
                if block_len <= 1:
                    break   # single-block frame
            else:
                if blk_num + 1 >= block_len:
                    break   # got all blocks

        rec['blocks'] = blocks
        frames.append(rec)

    return frames

# ── Protocol detail formatter ─────────────────────────────────────────────────
def format_frame(rec):
    """Format a decoded frame for display."""
    if 'blocks' not in rec:
        return rec.get('frame', '?'), ''

    d = rec['blocks'][0]['data']
    fid = rec.get('frame_id', 0)
    crc = '✓' if rec['blocks'][0]['crc_ok'] else '✗'
    fec = rec['blocks'][0]['fec_errs']

    if fid == 0x0F:   # SVP
        t = d[6]
        if t in (1, 3):
            det = (f"SVP{t}  TXPOW={d[7]}dB  RSSI_PROC={'cont' if d[8]&1 else 'frame'}"
                   f"  RSSI_PERIOD={d[9]*20}ms  TIME_TO_NEXT={d[10]}"
                   f"  MAX_REP={d[11]}  BASEST={d[12]}  SCAN_TIME={d[13]*100}ms"
                   f"  BAD_BASE={d[14]}dBuV  GOOD_BASE={d[15]}dBuV  BETTER_BASE={d[16]}dBuV"
                   f"  PRIO={d[17]}")
        elif t == 6:
            det = (f"SVP6  CYCLE={d[7]*250}ms  TIME_TO_NEXT={d[8]*250}ms"
                   f"  TRANSACTION={d[9]*250}ms  EVAL_CUR={d[10]}s"
                   f"  EVAL_OTH={d[11]}  DSI={(d[13]>>4)&0xF}")
        elif t in (2, 4):
            det = f"SVP{t}  DOFREQ={(d[10]<<8)|d[11]}"
        else:
            det = f"SVP{t}"
        raw = ' '.join(f'{x:02X}' for x in d)
        return FNAMES[fid], f"{crc} FEC:{fec}  {det}\n  raw=[{raw}]"

    elif fid == 0x0E:   # FRI
        det = (f"RAND_SLOTS={d[6]}  FREE_SLOTS={d[7]}  MAX_ACCESS={d[8]}"
               f"  SLOT_LENGTH={d[9]}(={d[9]*4}ms)  TIMEOUT={d[11]}  TRAFFIC={d[17]}")
        raw = ' '.join(f'{x:02X}' for x in d)
        return FNAMES[fid], f"{crc} FEC:{fec}  {det}\n  raw=[{raw}]"

    elif fid == 0x01:   # MRM
        sender = (d[6] << 16) | (d[7] << 8) | d[8]
        dest2  = (d[9] << 16) | (d[10] << 8) | d[11]
        state  = (d[12] >> 5) & 0x07
        pclass = (d[13] >> 6) & 0x03
        ef     = (d[13] >> 5) & 0x01
        ptype  =  d[13] & 0x1F
        if pclass == 3:
            pname = DTE_NAMES.get(ptype, f'DTE{ptype}')
        elif pclass == 0:
            pname = 'EXTPAK' if ef else PSUB_NAMES.get(ptype, f'PSUB{ptype}')
        elif pclass == 2:
            pname = f'CONN{ptype}'
        else:
            pname = f'CL{pclass}T{ptype}'
        payload_blocks = rec['blocks'][1:]
        payload = b''.join(bytes(b['data']) for b in payload_blocks)
        det = (f"sender={sender:06X}  dest={dest2:06X}  {pname}"
               f"  state={state}  seq={rec.get('seq',0)}")
        if payload:
            det += f"  payload({len(payload)}B)=[{payload.hex()[:48]}{'...' if len(payload)>24 else ''}]"
        raw = ' '.join(f'{x:02X}' for x in d)
        return FNAMES[fid], f"{crc} FEC:{fec}  {det}\n  raw=[{raw}]"

    elif fid in (0x06, 0x07):
        return FNAMES[fid], f"{crc} FEC:{fec}  MAN={rec.get('dest_man',0):06X}"

    elif fid in (0x02, 0x03, 0x08, 0x09, 0x0B):
        raw = ' '.join(f'{x:02X}' for x in d)
        return FNAMES.get(fid, f'0x{fid:02X}'), f"{crc} FEC:{fec}  dest={rec.get('dest_man',0):06X}  raw=[{raw}]"

    else:
        raw = ' '.join(f'{x:02X}' for x in d)
        return FNAMES.get(fid, f'0x{fid:02X}'), f"{crc} FEC:{fec}  dest={rec.get('dest_man',0):06X}\n  raw=[{raw}]"

# ── Main decode pipeline ──────────────────────────────────────────────────────
def decode_file(path, search_offsets=None, verbose=False, show_fillers=False):
    """Decode one IQ file. Returns list of decoded frame dicts."""
    print(f"\n{'='*72}")
    print(f"FILE: {os.path.basename(path)}")

    iq, sr = read_iq_wav(path)
    duration = len(iq) / sr
    print(f"  SR={sr}Hz  Duration={duration:.1f}s  Samples={len(iq)}")
    print(f"  Power={np.mean(np.abs(iq)**2):.6f}  "
          f"I=[{iq.real.min():.3f},{iq.real.max():.3f}]  "
          f"Q=[{iq.imag.min():.3f},{iq.imag.max():.3f}]")

    # Try a range of frequency offsets (signal may not be exactly at center)
    if search_offsets is None:
        # Auto-detect peak, then also try ±12.5kHz (adjacent Mobitex channels)
        auto_offset = find_freq_offset(iq, sr)
        search_offsets = [auto_offset, 0.0]
        for delta in [12500, -12500, 25000, -25000, 6250, -6250]:
            search_offsets.append(auto_offset + delta)
        search_offsets = list(dict.fromkeys(search_offsets))  # deduplicate

    best_frames = []
    best_score  = -1
    best_offset = 0.0
    best_pol    = 1

    for offset in search_offsets:
        snr = measure_snr(iq, sr, offset)
        if verbose:
            print(f"  Trying offset {offset:+.0f}Hz  SNR={snr:.1f}dB")

        # Demodulate once per offset
        demod, sr2 = demodulate(iq, sr, freq_offset=offset)

        for polarity in [1, -1]:
            # Polarity: negate demod output to swap mark/space (0xCCCC ↔ 0x3333)
            # Note: multiplying IQ by -1 before FM demod has NO effect on the
            # demodulated frequency — only negating the demod output works.
            demod_pol = demod if polarity == 1 else -demod

            for phase in range(6):
                bits, est_spb = extract_bits(demod_pol, sr2, phase_offset=phase)
                frames = decode_bits(bits)
                data_frames = [f for f in frames if 'blocks' in f]
                crc_ok_count = sum(1 for f in data_frames if f['blocks'][0]['crc_ok'])
                fec0_count   = sum(1 for f in frames if f.get('head_fec', 99) == 0)
                score = (crc_ok_count * 1000
                         + len(data_frames) * 10
                         + fec0_count * 5      # reward clean head FEC
                         + len(frames))
                if score > best_score:
                    best_score  = score
                    best_frames = frames
                    best_offset = offset
                    best_pol    = polarity
                    best_phase  = phase
                    if score > 5000:
                        break  # good enough
            if best_score > 5000:
                break
        if best_score > 5000:
            break

    # Second pass: try fine fractional phase offsets around the best integer phase
    if best_score < 100:
        demod_best, sr2 = demodulate(iq, sr, freq_offset=best_offset)
        demod_pol_best = demod_best if best_pol == 1 else -demod_best
        for frac in [x * 0.1 for x in range(1, 10)]:
            ph = best_phase + frac
            bits, _ = extract_bits(demod_pol_best, sr2, phase_offset=ph)
            frames = decode_bits(bits)
            data_frames = [f for f in frames if 'blocks' in f]
            crc_ok_count = sum(1 for f in data_frames if f['blocks'][0]['crc_ok'])
            fec0_count   = sum(1 for f in frames if f.get('head_fec', 99) == 0)
            score = (crc_ok_count * 1000 + len(data_frames) * 10
                     + fec0_count * 5 + len(frames))
            if score > best_score:
                best_score  = score
                best_frames = frames
                best_phase  = ph

    data_f  = [f for f in best_frames if 'blocks' in f]
    hdr_f   = [f for f in best_frames if 'blocks' not in f]
    crc_ok  = sum(1 for f in data_f if f['blocks'][0]['crc_ok'])

    print(f"\n  Best: offset={best_offset:+.0f}Hz  pol={best_pol:+d}  "
          f"phase={best_phase}  SNR={measure_snr(iq, sr, best_offset):.1f}dB")
    print(f"  Frames: {len(best_frames)} total = "
          f"{len(hdr_f)} header-only + {len(data_f)} with data ({crc_ok} CRC-ok)")

    # Frame type summary
    ftypes = Counter(f.get('frame', '?') for f in best_frames if 'blocks' in f)
    if ftypes:
        print(f"  Types: {dict(ftypes)}")

    # Network identity
    nets = Counter((FRSYNCS[f['frsync']], f['area_id'], f['base_id'])
                   for f in best_frames)
    for (fs, area, base), cnt in nets.most_common(5):
        print(f"  Network: frsync=0x{fs:04X} AreaID={area} BaseID={base}  ({cnt} frames)")

    # Timing
    times = [f['t'] for f in best_frames]
    if len(times) > 2:
        diffs = np.diff(times)
        big   = diffs[diffs > 0.05]
        print(f"  Timing: interval min={diffs.min():.3f}s max={diffs.max():.3f}s "
              f"mean={big.mean():.3f}s" if len(big) else "")

    # Detailed frame list
    print(f"\n{'─'*72}")
    print(f"{'Time':>7}  {'Dir':<4}  {'FS':>6}  {'Base':>7}  {'Frame':<8}  Detail")
    print(f"{'─'*72}")

    for f in best_frames:
        if not show_fillers and f.get('frame', '') in ('filler', 'hdr-cf1'):
            continue
        t    = f['t']
        d    = f['dir']
        fs   = f'0x{FRSYNCS[f["frsync"]]:04X}'
        base = f"{f['area_id']}.{f['base_id']}"

        if 'blocks' in f:
            fname, detail = format_frame(f)
            print(f"{t:7.3f}  {d:<4}  {fs:>6}  {base:>7}  {fname:<8}  {detail}")
        else:
            frame_name = f.get('frame', f'HDR(cf={f["cflags"]})')
            print(f"{t:7.3f}  {d:<4}  {fs:>6}  {base:>7}  {frame_name}")

    return best_frames


def main():
    ap = argparse.ArgumentParser(
        description='Mobitex IQ capture decoder',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  %(prog)s *.wav
  %(prog)s capture.wav --verbose
  %(prog)s capture.wav --show-fillers
  %(prog)s capture.wav --offset 12500
""")
    ap.add_argument('files', nargs='+', help='IQ WAV file(s)')
    ap.add_argument('--offset', type=float, default=None,
                    help='Force frequency offset in Hz (default: auto-detect)')
    ap.add_argument('--verbose', '-v', action='store_true',
                    help='Show all offset/polarity attempts')
    ap.add_argument('--show-fillers', action='store_true',
                    help='Show filler and header-only frames')
    ap.add_argument('--summary-only', action='store_true',
                    help='Print summary only, no per-frame detail')
    args = ap.parse_args()

    all_results = {}
    for path in args.files:
        offsets = [args.offset] if args.offset is not None else None
        frames = decode_file(path,
                             search_offsets=offsets,
                             verbose=args.verbose,
                             show_fillers=args.show_fillers)
        all_results[path] = frames

    # Cross-file summary
    if len(args.files) > 1:
        print(f"\n{'='*72}")
        print("SUMMARY ACROSS ALL FILES")
        print(f"{'─'*72}")
        for path, frames in all_results.items():
            data_f = [f for f in frames if 'blocks' in f]
            crc_ok = sum(1 for f in data_f if f['blocks'][0]['crc_ok'])
            nets   = Counter((FRSYNCS[f['frsync']], f['area_id'], f['base_id'])
                             for f in frames)
            top_net = nets.most_common(1)
            net_s = (f"0x{top_net[0][0][0]:04X} Area={top_net[0][0][1]} Base={top_net[0][0][2]}"
                     if top_net else "none")
            print(f"  {os.path.basename(path)[:45]:<45}  "
                  f"{len(frames):3} frames  {len(data_f):3} data  {crc_ok:3} CRC-ok  {net_s}")


if __name__ == '__main__':
    main()
