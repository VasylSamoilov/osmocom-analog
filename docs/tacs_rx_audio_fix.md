# TACS/AMPS RX Audio Path Bug Fixes

## Summary

Fixed three critical bugs in the AMPS/TACS receive audio processing path (`src/amps/dsp.c`) that caused severe audio distortion, metallic noise, and over-amplification during voice calls.

## Date

January 2026

## Symptoms Before Fix

- Speech was "too loud" and distorted
- Metallic/harsh audio quality
- Debug logs showed extreme amplification:
  - Output peaks of 20-75x (should be < 1.0)
  - Expander envelope values erratic
  - Filter stage showing 2x amplification

## Bugs Found and Fixed

### Bug 1: `de_emphasis()` Using Wrong Sample Count

**Location:** `src/amps/dsp.c`, in `sender_receive_audio()`

**Original code:**
```c
count = samplerate_downsample(&amps->sender.srstate, samples, length);

/* de-emphasis (now at 8000 Hz) */
if (amps->de_emphasis)
    de_emphasis(&amps->estate, samples, length);  // BUG: uses 'length'
```

**Problem:** After `samplerate_downsample()`, only `count` samples are valid in the buffer. The rest is garbage/uninitialized memory. Using `length` caused `de_emphasis()` to process garbage data, corrupting the audio.

**Fix:**
```c
count = samplerate_downsample(&amps->sender.srstate, samples, length);

/* de-emphasis (now at 8000 Hz) */
if (amps->de_emphasis)
    de_emphasis(&amps->estate_rx, samples, count);  // FIXED: uses 'count'
```

### Bug 2: Redundant Second `samplerate_downsample()` Call

**Location:** `src/amps/dsp.c`, in `sender_receive_audio()`

**Original code:**
```c
count = samplerate_downsample(&amps->sender.srstate, samples, length);  // First
// ... de_emphasis ...
count = samplerate_downsample(&amps->sender.srstate, samples, length);  // Second - BUG!
expand_audio(&amps->cstate, samples, count);
```

**Problem:** The second `samplerate_downsample()` call used `length` (original sample count) but only `count` samples were valid after the first downsample. This caused:
1. Reading garbage memory beyond valid samples
2. Corrupt output from the downsampler
3. The downsampler state becoming inconsistent

**Evidence:** All other DSP files in the project (cnetz, nmt, r2000, etc.) use only ONE `samplerate_downsample()` call:
- `src/cnetz/dsp.c`: Single downsample
- `src/nmt/dsp.c`: Single downsample  
- `src/r2000/dsp.c`: Single downsample

**Fix:** Removed the redundant second downsample call entirely.

### Bug 3: `de_emphasis()` Using Wrong Filter State

**Location:** `src/amps/dsp.c`, in `sender_receive_audio()`

**Original code:**
```c
/* de-emphasis (now at 8000 Hz) */
if (amps->de_emphasis)
    de_emphasis(&amps->estate, samples, count);  // BUG: wrong state
```

**Problem:** The code uses `amps->estate` but this is initialized for the **high sample rate** (input rate, e.g., 192kHz). After downsampling, audio is at 8000 Hz and should use `amps->estate_rx` which is specifically initialized for 8000 Hz:

From `src/amps/amps.c`:
```c
// estate initialized for high sample rate
rc = init_emphasis(&amps->estate, samplerate, ...);

// estate_rx initialized for 8000 Hz - THIS SHOULD BE USED!
rc = init_emphasis(&amps->estate_rx, 8000, ...);
```

Using the wrong emphasis state meant the de-emphasis filter coefficients were completely wrong for the actual sample rate.

**Fix:**
```c
/* de-emphasis (now at 8000 Hz) - use estate_rx which is initialized for 8000 Hz! */
if (amps->de_emphasis)
    de_emphasis(&amps->estate_rx, samples, count);  // FIXED: correct state
```

## Complete Fixed Code

```c
/* receive audio, but only if call established and SAT detected */
if ((amps->dsp_mode == DSP_MODE_AUDIO_RX_AUDIO_TX || amps->dsp_mode == DSP_MODE_AUDIO_RX_FRAME_TX)
 && trans && trans->callref) {
    int pos, count;
    int i;

    /* Apply RX Pre-Filters */
    iir_process(&amps->rx_hpf, samples, length);
    iir_process(&amps->rx_notch_filter, samples, length);
    iir_process(&amps->rx_pre_filter, samples, length);

    /* Downsample to 8000 Hz */
    count = samplerate_downsample(&amps->sender.srstate, samples, length);

    /* de-emphasis at 8000 Hz - use estate_rx and correct count! */
    if (amps->de_emphasis)
        de_emphasis(&amps->estate_rx, samples, count);

    /* Compandor expansion */
    expand_audio(&amps->cstate, samples, count);

    /* Send to call control */
    // ... rest of code ...
}
```

## Debug Output Comparison

### Before Fix (Broken)
```
RX peaks: in=1.98 filt=4.17 deemph=1.74 out=20.22 (env=4.090)
RX peaks: in=0.93 filt=0.40 deemph=0.10 out=1.73 (env=1.792)
RX peaks: in=1.72 filt=1.85 deemph=0.04 out=0.00 (env=0.033)
```
- Output peaks of 20x+ (severe distortion)
- Erratic envelope values
- Filter stage amplifying signal

### After Fix (Working)
```
RX peaks: in=0.85 filt=0.95 deemph=0.81 out=0.65 (env=0.758)
RX peaks: in=1.23 filt=1.42 deemph=1.28 out=0.79 (env=0.776)
RX peaks: in=1.05 filt=1.27 deemph=0.71 out=0.53 (env=0.669)
```
- Output peaks within normal range (< 1.0)
- Envelope tracking signal properly
- Clean audio quality

## Files Modified

- `src/amps/dsp.c` - Fixed all three bugs in `sender_receive_audio()`

## Testing

Debug logging was added to track signal levels at each processing stage:
```c
LOGP_CHAN(DDSP, LOGL_NOTICE, "RX peaks: in=%.2f filt=%.2f deemph=%.2f out=%.2f (env=%.3f, tacs=%d)\n",
    dbg_in_peak, dbg_filt_peak, dbg_deemph_peak, dbg_out_peak, amps->cstate.e.envelope, tacs);
```

This logging helped identify exactly where the audio corruption was occurring.

## Impact

These bugs likely affected all AMPS and TACS voice calls since the code was written. The combination of:
1. Processing garbage memory
2. Redundant/broken downsampling
3. Wrong filter coefficients

...resulted in severely distorted audio that was masked by the "for some reason, 4000 Hz deviation works better" workaround in the speech deviation constant.

## Speech Deviation Value

### Resolution

After fixing the bugs, the standard **2300 Hz** deviation now works correctly.

The original comment "for some reason, 4000 Hz deviation works better" was because the higher value (4000 Hz) reduced the normalization gain, which **partially compensated for the bugs** that were amplifying garbage data.

With the bugs fixed:
- **2300 Hz** (standard): Audio is loud and clear ✓
- **4000 Hz** (workaround): Still works but quieter than intended

The code now uses the correct TACS standard value of 2300 Hz.

## Debug Logging

Debug logging is currently enabled to track signal levels:
```
DDSP NOTICE ... RX peaks: in=X.XX filt=X.XX deemph=X.XX out=X.XX (env=X.XXX, tacs=1)
```

This can be removed or made conditional (e.g., only with `-d` debug flag) in a future cleanup.
