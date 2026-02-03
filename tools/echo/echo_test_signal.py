#!/usr/bin/env python3
"""
Echo Test Signal Generator

Generates test signals for echo path measurement based on ITU-T P.501 principles.
The signal is designed to measure:
- Echo delay (via cross-correlation of PN sequence)
- Echo Return Loss (ERL) 
- Frequency response of echo path (via chirp)

Signal structure:
  [silence] [PN sequence] [silence] [chirp] [silence]
"""

import numpy as np
from scipy.io import wavfile
from scipy.signal import max_len_seq
import argparse


def generate_mls(n_bits: int = 12, amplitude: float = 0.5) -> np.ndarray:
    """
    Generate Maximum Length Sequence (MLS) for echo delay measurement.
    
    MLS is ideal because:
    - Its autocorrelation is nearly a delta function
    - Cross-correlation with delayed version gives precise delay estimate
    - White spectrum covers all frequencies uniformly
    
    Args:
        n_bits: Number of bits for the sequence (length = 2^n - 1)
        amplitude: Peak amplitude (0 to 1)
    
    Returns:
        MLS samples as float array in range [-amplitude, +amplitude]
    """
    # Generate MLS: values are 0 and 1
    seq, _ = max_len_seq(n_bits)
    # Convert to bipolar: 0 -> -1, 1 -> +1
    bipolar = 2.0 * seq.astype(np.float64) - 1.0
    return bipolar * amplitude


def generate_chirp(duration_s: float, f_start: float, f_end: float,
                   sample_rate: int, amplitude: float = 0.5) -> np.ndarray:
    """
    Generate linear chirp sweep for frequency response analysis.
    
    The chirp allows measuring the echo path's frequency response
    by comparing transmitted and received chirps.
    
    Args:
        duration_s: Duration in seconds
        f_start: Starting frequency in Hz
        f_end: Ending frequency in Hz  
        sample_rate: Sample rate in Hz
        amplitude: Peak amplitude
    
    Returns:
        Chirp signal as float array
    """
    n_samples = int(duration_s * sample_rate)
    t = np.arange(n_samples) / sample_rate
    
    # Linear chirp: frequency increases linearly with time
    # f(t) = f_start + (f_end - f_start) * t / duration
    # phase = integral of 2*pi*f(t) dt
    k = (f_end - f_start) / duration_s
    phase = 2 * np.pi * (f_start * t + 0.5 * k * t**2)
    
    return amplitude * np.sin(phase)


def generate_silence(duration_s: float, sample_rate: int) -> np.ndarray:
    """Generate silence (zeros) for the specified duration."""
    return np.zeros(int(duration_s * sample_rate))


def generate_short_chirp(duration_s: float, sample_rate: int, 
                        amplitude: float = 0.5) -> np.ndarray:
    """
    Generate a short chirp sweep for echo detection markers.
    
    Short chirps are excellent for echo detection because:
    - Sweep through frequencies making them easy to identify
    - Distinctive sound that's recognizable in recordings
    - Sharp onset/offset makes echo delay obvious
    - Broadband content helps with correlation
    
    Args:
        duration_s: Duration in seconds (typically 0.1-0.3s)
        sample_rate: Sample rate in Hz
        amplitude: Peak amplitude (0 to 1)
    
    Returns:
        Chirp sweep as float array
    """
    # Sweep from 500 Hz to 2000 Hz (audible range, good for echo detection)
    f_start = 500
    f_end = 2000
    
    n_samples = int(duration_s * sample_rate)
    t = np.arange(n_samples) / sample_rate
    
    # Linear chirp
    k = (f_end - f_start) / duration_s
    phase = 2 * np.pi * (f_start * t + 0.5 * k * t**2)
    chirp = amplitude * np.sin(phase)
    
    # Apply fade in/out to prevent clicks
    fade_samples = int(0.005 * sample_rate)  # 5ms fade
    if fade_samples > 0 and fade_samples < n_samples // 2:
        # Fade in (raised cosine)
        fade_in = 0.5 * (1 - np.cos(np.pi * np.arange(fade_samples) / fade_samples))
        chirp[:fade_samples] *= fade_in
        
        # Fade out
        fade_out = 0.5 * (1 + np.cos(np.pi * np.arange(fade_samples) / fade_samples))
        chirp[-fade_samples:] *= fade_out
    
    return chirp


def create_efficiency_test_signal(sample_rate: int = 8000,
                                 duration_s: float = 5.0,
                                 mls_bits: int = 12,
                                 amplitude: float = 0.5,
                                 use_chirps: bool = True,
                                 sdr_optimized: bool = False) -> np.ndarray:
    """
    Create extended test signal for AEC efficiency measurement.
    
    This signal is optimized for measuring echo cancellation efficiency,
    convergence time, and ERLE. It combines multiple signal types for
    robust echo detection and analysis.
    
    Structure (with chirps enabled):
      [silence] [chirp] [silence] [continuous MLS] [silence] [chirp] [silence]
    
    Structure (chirps disabled):
      [silence] [continuous MLS repetitions] [silence]
    
    Structure (SDR optimized):
      [silence] [chirp] [silence] [tone bursts] [silence] [MLS] [silence] [chirp] [silence]
    
    The chirps provide:
    - Clear audible markers for manual verification (sweep 500-2000 Hz)
    - Sharp transients for easy echo identification
    - Broadband frequency content for analysis
    - Distinctive sound that's easy to recognize
    
    The continuous MLS provides:
    - Cross-correlation at any point in the signal
    - Sliding window ERL calculation throughout
    - Convergence tracking as AEC adapts
    
    SDR optimization adds:
    - Multiple tone bursts at different frequencies for robust correlation
    - Longer chirps for better SNR in noisy environments
    - Reduced amplitude to prevent clipping/distortion
    
    Args:
        sample_rate: Sample rate in Hz (8000 or 16000 typical)
        duration_s: Total signal duration in seconds (default 5.0)
        mls_bits: Bits for MLS (12 = 4095 samples, 13 = 8191 samples)
        amplitude: Signal amplitude (0 to 1, recommend 0.5 for headroom, 0.3 for SDR)
        use_chirps: Include chirp sweeps for easier echo identification (default True)
        sdr_optimized: Optimize for SDR environments with noise/distortion (default False)
    
    Returns:
        Extended test signal as float array
    """
    parts = []
    
    # Adjust amplitude for SDR if needed
    if sdr_optimized and amplitude > 0.35:
        amplitude = 0.3  # Lower amplitude to prevent clipping in SDR
    
    # 1. Leading silence (0.3s for noise floor measurement)
    silence_duration = 0.3
    parts.append(generate_silence(silence_duration, sample_rate))
    
    if use_chirps:
        # 2. Initial chirp - longer for SDR
        chirp_duration = 0.3 if sdr_optimized else 0.2
        parts.append(generate_short_chirp(chirp_duration, sample_rate, amplitude))
        
        # 3. Short silence after chirp
        parts.append(generate_silence(0.2, sample_rate))
    
    if sdr_optimized:
        # Add tone bursts at multiple frequencies for robust correlation
        # These help with echo detection in noisy SDR environments
        for freq in [800, 1200, 1600]:
            parts.append(generate_tone_burst(freq, 0.15, sample_rate, amplitude))
            parts.append(generate_silence(0.1, sample_rate))
    
    # 4. Calculate remaining time for MLS
    target_samples = int(duration_s * sample_rate)
    trailing_silence_duration = 0.3
    
    if use_chirps:
        # Account for chirps in timing
        final_chirp_duration = 0.3 if sdr_optimized else 0.2
        final_silence_before_chirp = 0.2
        
        used_samples = len(np.concatenate(parts))
        trailing_samples = int((trailing_silence_duration + final_silence_before_chirp + 
                               final_chirp_duration + trailing_silence_duration) * sample_rate)
        mls_samples_needed = target_samples - used_samples - trailing_samples
    else:
        used_samples = len(np.concatenate(parts))
        trailing_samples = int(trailing_silence_duration * sample_rate)
        mls_samples_needed = target_samples - used_samples - trailing_samples
    
    # 5. Generate continuous MLS repetitions to fill the duration
    mls = generate_mls(mls_bits, amplitude)
    mls_length_samples = len(mls)
    
    # Calculate number of repetitions needed
    num_repeats = max(1, int(np.ceil(mls_samples_needed / mls_length_samples)))
    
    # Add continuous MLS repetitions (no gaps)
    mls_continuous = np.tile(mls, num_repeats)
    # Trim to exact length needed
    mls_continuous = mls_continuous[:mls_samples_needed]
    parts.append(mls_continuous)
    
    if use_chirps:
        # 6. Short silence before final chirp
        parts.append(generate_silence(0.2, sample_rate))
        
        # 7. Final chirp - longer for SDR
        chirp_duration = 0.3 if sdr_optimized else 0.2
        parts.append(generate_short_chirp(chirp_duration, sample_rate, amplitude))
    
    # 8. Trailing silence (for echo tail measurement)
    parts.append(generate_silence(trailing_silence_duration, sample_rate))
    
    # Concatenate all parts
    signal = np.concatenate(parts)
    
    # Final trim to exact duration
    if len(signal) > target_samples:
        signal = signal[:target_samples]
    elif len(signal) < target_samples:
        # Pad with silence if needed
        padding = np.zeros(target_samples - len(signal))
        signal = np.concatenate([signal, padding])
    
    return signal


def generate_tone_burst(frequency: float, duration_s: float, 
                       sample_rate: int, amplitude: float = 0.5,
                       fade_ms: float = 5.0) -> np.ndarray:
    """
    Generate a tone burst (single frequency) with smooth fade in/out.
    
    Tone bursts are useful for echo detection in noisy environments because:
    - Single frequency is easy to detect and correlate
    - Clear, audible signal that's easy to identify in recordings
    - Sharp onset/offset makes echo delay obvious
    - Fade prevents clicks and spectral splatter
    
    Args:
        frequency: Tone frequency in Hz
        duration_s: Duration in seconds
        sample_rate: Sample rate in Hz
        amplitude: Peak amplitude (0 to 1)
        fade_ms: Fade in/out duration in milliseconds
    
    Returns:
        Tone burst as float array
    """
    n_samples = int(duration_s * sample_rate)
    t = np.arange(n_samples) / sample_rate
    
    # Generate sine wave
    tone = amplitude * np.sin(2 * np.pi * frequency * t)
    
    # Apply fade in/out to prevent clicks
    fade_samples = int(fade_ms * sample_rate / 1000.0)
    if fade_samples > 0 and fade_samples < n_samples // 2:
        # Fade in (raised cosine)
        fade_in = 0.5 * (1 - np.cos(np.pi * np.arange(fade_samples) / fade_samples))
        tone[:fade_samples] *= fade_in
        
        # Fade out
        fade_out = 0.5 * (1 + np.cos(np.pi * np.arange(fade_samples) / fade_samples))
        tone[-fade_samples:] *= fade_out
    
    return tone


def create_echo_test_signal(sample_rate: int = 8000,
                            mls_bits: int = 12,
                            mls_repeats: int = 2,
                            chirp_duration: float = 0.5,
                            silence_duration: float = 0.3,
                            amplitude: float = 0.5,
                            use_marker_chirps: bool = True) -> np.ndarray:
    """
    Create complete echo test signal.
    
    Structure (with marker chirps):
      [silence] [short chirp] [silence] [MLS x repeats] [silence] [long chirp] [silence] [short chirp] [silence]
    
    Structure (without marker chirps):
      [silence] [MLS x repeats] [silence] [long chirp] [silence]
    
    The marker chirps (short, 500-2000 Hz) provide:
    - Clear audible markers for manual verification
    - Easy identification of echo delay
    - Confirmation that signal is playing/recording
    
    The long chirp (50 Hz to Nyquist) provides:
    - Full frequency response measurement
    
    Args:
        sample_rate: Sample rate in Hz (8000 or 16000 typical)
        mls_bits: Bits for MLS (12 = 4095 samples, 13 = 8191 samples)
        mls_repeats: Number of MLS repetitions (more = better SNR)
        chirp_duration: Duration of long chirp in seconds
        silence_duration: Duration of silence gaps
        amplitude: Signal amplitude (0 to 1, recommend 0.5 for headroom)
        use_marker_chirps: Include short chirps for easier echo identification (default True)
    
    Returns:
        Complete test signal as float array
    """
    parts = []
    
    # 1. Leading silence (for noise floor measurement)
    parts.append(generate_silence(silence_duration, sample_rate))
    
    if use_marker_chirps:
        # 2. Initial marker chirp (500-2000 Hz, 200ms)
        parts.append(generate_short_chirp(0.2, sample_rate, amplitude))
        parts.append(generate_silence(silence_duration, sample_rate))
    
    # 3. MLS sequence (repeated for better correlation SNR)
    mls = generate_mls(mls_bits, amplitude)
    for _ in range(mls_repeats):
        parts.append(mls)
        # Small gap between repeats
        parts.append(generate_silence(0.05, sample_rate))
    
    # 4. Silence gap
    parts.append(generate_silence(silence_duration, sample_rate))
    
    # 5. Long chirp sweep (50 Hz to Nyquist/2) for frequency response
    f_max = sample_rate / 2 * 0.9  # Stay below Nyquist
    parts.append(generate_chirp(chirp_duration, 50, f_max, sample_rate, amplitude))
    
    # 6. Silence gap
    parts.append(generate_silence(silence_duration, sample_rate))
    
    if use_marker_chirps:
        # 7. Final marker chirp (500-2000 Hz, 200ms)
        parts.append(generate_short_chirp(0.2, sample_rate, amplitude))
    
    # 8. Trailing silence (for echo tail measurement)
    parts.append(generate_silence(silence_duration * 2, sample_rate))
    
    return np.concatenate(parts)


def save_wav(signal: np.ndarray, filename: str, sample_rate: int):
    """
    Save signal to 16-bit WAV file.
    
    Args:
        signal: Float signal array (expected range -1 to +1)
        filename: Output filename
        sample_rate: Sample rate in Hz
    """
    # Clip to prevent overflow
    clipped = np.clip(signal, -1.0, 1.0)
    # Convert to 16-bit integer
    int16_signal = (clipped * 32767).astype(np.int16)
    wavfile.write(filename, sample_rate, int16_signal)


def main():
    parser = argparse.ArgumentParser(
        description="Generate echo test signal for measuring echo parameters",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -o echo_test.wav
  %(prog)s -o echo_test_16k.wav -r 16000
  %(prog)s -o echo_test.wav --mls-bits 13 --mls-repeats 3
  %(prog)s -o efficiency_test.wav --efficiency --duration 5.0
        """
    )
    
    parser.add_argument("-o", "--output", default="echo_test_signal.wav",
                        help="Output WAV filename (default: echo_test_signal.wav)")
    parser.add_argument("-r", "--rate", type=int, default=8000,
                        help="Sample rate in Hz (default: 8000)")
    parser.add_argument("--mls-bits", type=int, default=12,
                        help="MLS length in bits (default: 12, gives 4095 samples)")
    parser.add_argument("--mls-repeats", type=int, default=2,
                        help="Number of MLS repetitions (default: 2, ignored with --efficiency)")
    parser.add_argument("--chirp-duration", type=float, default=0.5,
                        help="Chirp duration in seconds (default: 0.5, ignored with --efficiency)")
    parser.add_argument("--amplitude", type=float, default=0.5,
                        help="Signal amplitude 0-1 (default: 0.5)")
    parser.add_argument("--no-chirps", action="store_true",
                        help="Disable marker chirps in the signal")
    parser.add_argument("--sdr", action="store_true",
                        help="Optimize for SDR environments (lower amplitude, tone bursts, longer chirps)")
    parser.add_argument("--efficiency", action="store_true",
                        help="Generate extended efficiency test signal (continuous MLS, default 5s)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Duration in seconds for efficiency signal (default: 5.0, only used with --efficiency)")
    
    args = parser.parse_args()
    
    if args.efficiency:
        # Generate extended efficiency test signal
        print(f"Generating efficiency test signal...")
        print(f"  Sample rate: {args.rate} Hz")
        print(f"  Duration: {args.duration} seconds")
        print(f"  MLS: {args.mls_bits} bits = {2**args.mls_bits - 1} samples (continuous repetitions)")
        print(f"  Amplitude: {args.amplitude}")
        print(f"  Marker chirps: {'disabled' if args.no_chirps else 'enabled (500-2000 Hz sweeps)'}")
        if args.sdr:
            print(f"  SDR optimization: enabled (tone bursts, longer chirps, reduced amplitude)")
        
        signal = create_efficiency_test_signal(
            sample_rate=args.rate,
            duration_s=args.duration,
            mls_bits=args.mls_bits,
            amplitude=args.amplitude,
            use_chirps=not args.no_chirps,
            sdr_optimized=args.sdr
        )
    else:
        # Generate standard echo test signal
        print(f"Generating echo test signal...")
        print(f"  Sample rate: {args.rate} Hz")
        print(f"  MLS: {args.mls_bits} bits = {2**args.mls_bits - 1} samples, {args.mls_repeats} repeats")
        print(f"  Long chirp: {args.chirp_duration}s, 50 Hz to {args.rate/2*0.9:.0f} Hz")
        print(f"  Marker chirps: {'disabled' if args.no_chirps else 'enabled (500-2000 Hz sweeps)'}")
        
        signal = create_echo_test_signal(
            sample_rate=args.rate,
            mls_bits=args.mls_bits,
            mls_repeats=args.mls_repeats,
            chirp_duration=args.chirp_duration,
            amplitude=args.amplitude,
            use_marker_chirps=not args.no_chirps
        )
    
    duration_s = len(signal) / args.rate
    print(f"  Total duration: {duration_s:.2f} seconds ({len(signal)} samples)")
    
    save_wav(signal, args.output, args.rate)
    print(f"  Saved to: {args.output}")


if __name__ == "__main__":
    main()
