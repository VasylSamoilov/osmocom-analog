#!/usr/bin/env python3
"""
Mobitex discriminator-audio decoder (scipy version).
Uses Butterworth LPF + clock-tracked zero-crossing FSK demod.
Algorithm: MobyMon mobymon.cpp (IanWraith/MobyMon).
"""

import sys, os, wave, struct, argparse
import numpy as np
from scipy import signal as sp

BAUD = 8000

FRSYNCS = {0:0xC4D7,1:0xB433,2:0xA2F7,3:0xEB90,4:0xEE16,5:0x4BCC,6:0x3B48,
           7:0x5D08,8:0x146F,9:0x11E9,10:0xEB23,11:0xEF45,12:0x09D7,13:0x6877,
           14:0x14DC,15:0x10BA,16:0xF628,17:0x9788}
FRSYNC_MAP = {v:k for k,v in FRSYNCS.items()}

BITSYNC_BASE   = 0xCCCC
BITSYNC_MOBILE = 0x3333

FRAME_NAMES = {
    0x01:'MRM',  0x02:'ACK',  0x03:'NAK',  0x04:'REB',  0x05:'RES',
    0x06:'ABD',  0x07:'ABL',  0x08:'ATD',  0x09:'ATL',
    0x0C:'BKD',  0x0D:'BKT',  0x0E:'FRI',  0x0F:'SVP',
    0x10:'TST',  0x11:'AKT',  0x15:'AAT',
}
DTE_NAMES = {
    1:'LOGINREQ',2:'LOGINGRA',3:'LOGINREF',4:'LOGOUT',5:'LOGOUTORD',
    6:'BORN',7:'ACTIVE',8:'INACTIVE',9:'DIE',10:'LIVE',11:'ROAMORD',12:'ROAM',
    15:'GROUPLIST',16:'FLEXREQ',17:'FLEXLIST',18:'INFOREQ',19:'INFO',
    20:'TIME',21:'AREALIST',22:'ESNREQ',23:'ESNINFO',
}
PSUB_NAMES = {1:'TEXT',2:'DATA',3:'STATUS',4:'HPDATA'}

# FEC
H3,H2,H1,H0 = 0xEC,0xD3,0xBA,0x75

def ones8(v): return bin(v&0xFF).count('1')

def fec_check(cw12):
    data = (cw12 >> 4) & 0xFF
    fec  =  cw12 & 0x0F
    s = ((ones8(data&H3)&1)<<3|(ones8(data&H2)&1)<<2|
         (ones8(data&H1)&1)<<1|(ones8(data&H0)&1))
    syndrome = s ^ fec
    if syndrome == 0: return data, 0
    for bit in range(8):
        d2 = data ^ (1<<bit)
        s2= ((ones8(d2&H3)&1)<<3|(ones8(d2&H2)&1)<<2|
             (ones8(d2&H1)&1)<<1|(ones8(d2&H0)&1))
        if s2 == fec: return d2, 1
    for bit in range(4):
        if s == (fec^(1<<bit)): return data, 1
    return data, 2  # uncorrectable

CRC_POLY=0x0811; CRC_INIT_SR=0xF8E7; CRC_INIT_CR=0x2A5D

def crc16_mobitex(data18):
    sr,cr = CRC_INIT_SR, CRC_INIT_CR
    for byte in data18:
        for bit in range(8):
            if (byte>>bit)&1: cr ^= sr
            if sr&0x8000: sr=((sr<<1)&0xFFFF)^CRC_POLY
            else: sr=(sr<<1)&0xFFFF
    return cr

def descramble(bits, sr_init=0x1E):
    sr = sr_init
    out = []
    for b in bits:
        scr_bit = sr & 1
        tap = ((sr>>0)&1)^((sr>>4)&1)
        sr = (sr>>1)|(tap<<8)
        out.append(b^scr_bit)
    return out

def deinterleave(bits240):
    result = [0]*240
    for col in range(12):
        for row in range(20):
            result[row*12+col] = bits240[col*20+row]
    return result

def decode_block(bits240, scr_state):
    bits_d = []
    sr = scr_state
    for b in bits240:
        scr_bit = sr & 1
        tap = ((sr>>0)&1)^((sr>>4)&1)
        sr = (sr>>1)|(tap<<8)
        bits_d.append(b^scr_bit)
    new_sr = sr

    deint = deinterleave(bits_d)
    data = []; errs = 0
    for i in range(20):
        cw = 0
        for j in range(12): cw=(cw<<1)|deint[i*12+j]
        val, e = fec_check(cw)
        data.append(val); errs += e

    data18 = data[:18]
    rx_crc = (data[18]<<8)|data[19]
    calc_crc = crc16_mobitex(data18)
    return data18, (calc_crc==rx_crc), errs, new_sr

def decode_head_cb(cb1, cb2):
    base_id = (cb1>>2)&0x3F
    area_id = ((cb1&0x03)<<4)|((cb2>>4)&0x0F)
    cflags  = cb2&0x0F
    return base_id, area_id, cflags

def decode_svp(d):
    t = d[6]
    if t in (1,3):
        return (f"SVP{t} TXPOW={d[7]} RSSI_PROC={'C' if d[8]&1 else 'F'} "
                f"RSSI_PERIOD={d[9]*20}ms TIME_TO_NEXT={d[10]} MAX_REP={d[11]} "
                f"BASEST={d[12]} SCAN_TIME={d[13]*100}ms "
                f"BAD={d[14]} GOOD={d[15]} BETTER={d[16]}")
    elif t==6:
        return (f"SVP6 CYCLE={d[7]*250}ms T2NEXT={d[8]*250}ms TRANS={d[9]*250}ms "
                f"EVAL_CUR={d[10]}s EVAL_OTH={d[11]} DSI={(d[13]>>4)&0xF}")
    elif t in (2,4):
        dofreq=(d[10]<<8)|d[11]
        return f"SVP{t} DOFREQ_CH={dofreq}"
    return f"SVP{t}"

def decode_fri(d):
    return (f"FRI RAND={d[6]} FREE={d[7]} MAX_ACC={d[8]} "
            f"SLOT={d[9]}(={d[9]*4}ms) TIMEOUT={d[11]} TRAFFIC={d[17]}")

def decode_lc(d):
    dest=(d[0]<<16)|(d[1]<<8)|d[2]
    fid=d[3]&0x1F; seq=d[4]&0x0F
    bl=(((d[3]>>5)&1)<<4)|((d[4]>>4)&0x0F)
    blen=d[5]
    return dest,fid,seq,bl,blen

def decode_mrm(d):
    b=6
    sender=(d[b]<<16)|(d[b+1]<<8)|d[b+2]
    dest  =(d[b+4]<<16)|(d[b+5]<<8)|d[b+6]
    pclass=(d[b+10]>>1)&0x03
    extern_f=d[b+10]&0x01
    ptype=d[b+11]&0x1F
    if pclass==3:   pname=DTE_NAMES.get(ptype,f'DTE{ptype}')
    elif pclass==0: pname=('EXTPAK' if extern_f else PSUB_NAMES.get(ptype,f'PSUB{ptype}'))
    elif pclass==1: pname={1:'SOS',2:'SOSINFO',3:'SOSACK'}.get(ptype,f'SOS{ptype}')
    elif pclass==2: pname=f'CONN{ptype}'
    else:           pname=f'CL{pclass}T{ptype}'
    return sender, dest, pname


def load_wav(path):
    with wave.open(path) as wf:
        sr=wf.getframerate(); nch=wf.getnchannels(); sw=wf.getsampwidth()
        raw=wf.readframes(wf.getnframes()); nf=wf.getnframes()
    if sw==2: s=np.frombuffer(raw,dtype=np.int16).astype(np.float32)
    elif sw==4: s=np.frombuffer(raw,dtype=np.int32).astype(np.float32)
    else: raise ValueError(f"Unsupported sample width {sw}")
    if nch==2: s=s[::2]
    return s, sr


def extract_bits(samples, sr, polarity=1):
    """LPF + zero-crossing clock-tracked bit extraction."""
    spb = sr / BAUD
    b,a = sp.butter(4, 4000/(sr/2), btype='low')
    filt = sp.filtfilt(b, a, samples)
    if polarity < 0: filt = -filt

    crossings = np.where(np.diff(np.sign(filt)))[0].astype(float)
    if len(crossings) < 4: return []

    # Estimate spb from half-bit crossings
    diffs = np.diff(crossings)
    half = diffs[(diffs>spb*0.3)&(diffs<spb*0.8)]
    full = diffs[(diffs>spb*0.7)&(diffs<spb*1.3)]
    if len(half)>5:   est_spb = float(np.median(half)*2)
    elif len(full)>5: est_spb = float(np.median(full))
    else:             est_spb = spb

    bits = []
    pos = float(crossings[0] + est_spb*0.5)
    ci = 0
    while pos < len(filt):
        bits.append(0 if filt[int(pos)]>=0 else 1)
        next_pos = pos + est_spb
        # Find next crossing in window [pos+0.3*spb, pos+1.3*spb]
        while ci < len(crossings) and crossings[ci] <= pos: ci+=1
        if ci < len(crossings) and crossings[ci] < next_pos + est_spb*0.3:
            # Nudge phase
            next_pos = crossings[ci] + est_spb*0.5
        pos = next_pos
    return bits


def decode_bits(bits, polarity, filename):
    """Full frame decoder from bit stream."""
    results = []
    shift = 0
    i = 0
    N = len(bits)

    while i < N:
        b = bits[i]; i+=1
        shift = ((shift<<1)|b)&0xFFFFFFFF
        upper=(shift>>16)&0xFFFF
        lower= shift&0xFFFF

        if upper not in (BITSYNC_BASE,BITSYNC_MOBILE) or lower not in FRSYNC_MAP:
            continue

        frsync_idx = FRSYNC_MAP[lower]
        direction  = 'BASE' if upper==BITSYNC_BASE else 'MOB'

        # Read 24 header bits
        if i+24 > N: break
        head_bits = bits[i:i+24]; i+=24

        cw1=cw2=0
        for x in range(12): cw1=(cw1<<1)|head_bits[x]
        for x in range(12): cw2=(cw2<<1)|head_bits[x+12]
        cb1,e1=fec_check(cw1)
        cb2,e2=fec_check(cw2)
        if e1>1 or e2>1: continue  # bad header FEC

        base_id,area_id,cflags=decode_head_cb(cb1,cb2)
        t_approx = i/48000.0  # approximate

        if cflags != 0:
            if cflags==4:
                results.append({'t':t_approx,'dir':direction,'frsync':frsync_idx,
                                'base_id':base_id,'area_id':area_id,'cflags':cflags,
                                'frame':'FRI*','detail':'FRI(cflags=4 header)'})
            elif cflags==2:
                results.append({'t':t_approx,'dir':direction,'frsync':frsync_idx,
                                'base_id':base_id,'area_id':area_id,'cflags':cflags,
                                'frame':'TST*','detail':'Silence(cflags=2)'})
            # filler frames: silent
            continue

        # Collect blocks
        scr_state = 0x1E
        blocks = []
        any_crc_ok = False
        block_len_expected = None

        for blk_num in range(32):  # max 32 blocks
            if i+240 > N: break
            blk_bits = bits[i:i+240]; i+=240
            data18,crc_ok,fec_errs,scr_state = decode_block(blk_bits,scr_state)
            blocks.append((data18,crc_ok,fec_errs))
            if crc_ok: any_crc_ok=True

            if blk_num==0:
                dest,fid,seq,bl_hi,blen=decode_lc(data18)
                block_len_expected=blen
                if blen<=1: break
            else:
                if blk_num+1>=block_len_expected: break

        if not blocks: continue
        d0=blocks[0][0]; crc0=blocks[0][1]; fec0=blocks[0][2]
        dest,fid,seq,bl,blen=decode_lc(d0)
        fname=FRAME_NAMES.get(fid,f'0x{fid:02X}')

        r={'t':t_approx,'dir':direction,'frsync':frsync_idx,
           'base_id':base_id,'area_id':area_id,'cflags':cflags,
           'frame':fname,'frame_id':fid,'dest_man':dest,'seq':seq,
           'blocks':blen,'crc_ok':crc0,'fec_errors':fec0,'raw':d0}

        if fid==0x0F:   r['detail']=decode_svp(d0)
        elif fid==0x0E: r['detail']=decode_fri(d0)
        elif fid==0x01:
            try:
                snd,dst,pname=decode_mrm(d0)
                r['detail']=f"MRM sender={snd:06X} dest={dst:06X} {pname}"
                r['sender_man']=snd; r['mpak']=pname
            except: r['detail']=f"MRM dest={dest:06X}"
        elif fid in (0x02,0x03,0x04,0x08,0x09,0x0B):
            r['detail']=f"{fname} dest={dest:06X}"
        elif fid in (0x06,0x07):
            r['detail']=f"{fname} MAN={dest:06X}"
        else:
            r['detail']=f"{fname} dest={dest:06X}"

        results.append(r)

    return results


def process_file(path, show_fillers=False, show_raw=False):
    print(f"\n{'='*70}")
    print(f"FILE: {os.path.basename(path)}")
    print('='*70)

    try:
        samples, sr = load_wav(path)
    except Exception as e:
        print(f"  Error loading: {e}"); return

    spb = sr/BAUD
    duration = len(samples)/sr
    print(f"  Duration: {duration:.1f}s  SR: {sr}Hz  SPB: {spb:.1f}")

    best = []
    best_pol = 1
    for pol in [1,-1]:
        bits = extract_bits(samples, sr, polarity=pol)
        frames = decode_bits(bits, pol, path)
        real = [f for f in frames if '★' not in f.get('frame','') and '*' not in f.get('frame','')]
        print(f"  Polarity {pol:+d}: {len(bits)} bits → {len(frames)} frames "
              f"({sum(1 for f in frames if f.get('crc_ok'))} CRC-ok)")
        if len(frames) > len(best):
            best = frames; best_pol = pol

    if not best:
        print("  No frames decoded.")
        return

    print(f"\n  Best polarity: {best_pol:+d}  →  {len(best)} frames\n")

    # Summary stats
    ftypes = {}
    for f in best:
        ftypes[f['frame']] = ftypes.get(f['frame'],0)+1
    print("  Frame type counts:")
    for k,v in sorted(ftypes.items(),key=lambda x:-x[1]):
        print(f"    {k:<8} {v}")

    # Network identity
    nets = set()
    for f in best:
        nets.add((f['frsync'], f['area_id'], f['base_id']))
    print(f"\n  Networks seen: {len(nets)}")
    for (frs,area,base) in sorted(nets):
        fname={1:'B433/RMDUS(RAM-US)',0:'C4D7',3:'EB90/Telenor',6:'3B48',4:'EE16'}.get(frs,f'idx{frs}')
        print(f"    frsync={fname} AreaID={area} BaseID={base}")

    # Detailed frame list
    print(f"\n  Decoded frames:")
    for f in best:
        if '*' in f.get('frame','') and not show_fillers:
            continue
        crc = '✓' if f.get('crc_ok') else '✗'
        fec = f"FEC:{f['fec_errors']}" if f.get('fec_errors',0)>0 else ''
        detail = f.get('detail','')
        frsync_s = {1:'B433',3:'EB90',6:'3B48',4:'EE16',0:'C4D7'}.get(f['frsync'],f"f{f['frsync']}")
        print(f"    t={f['t']:6.2f}s [{frsync_s}] {f['dir']:<4} "
              f"Base={f['area_id']}.{f['base_id']} "
              f"{f['frame']:<6} {crc} {fec:<8} {detail}")
        if show_raw and 'raw' in f:
            print(f"      raw: {' '.join(f'{b:02X}' for b in f['raw'])}")


if __name__=='__main__':
    ap=argparse.ArgumentParser(description='Mobitex decoder')
    ap.add_argument('files',nargs='+')
    ap.add_argument('--fillers',action='store_true',help='Show filler/silence frames')
    ap.add_argument('--raw',action='store_true',help='Show raw block bytes')
    args=ap.parse_args()
    for path in args.files:
        process_file(path,show_fillers=args.fillers,show_raw=args.raw)
