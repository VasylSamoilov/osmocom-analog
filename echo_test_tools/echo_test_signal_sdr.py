#!/usr/bin/env python3
"""
SDR-Optimized Echo Test Signal Generator

Generates test signals specifically optimized for SDR environments where:
- Non-linear distortion is common (clipping, compression)
- Frequency response may be limited
- Timing jitter is present
- Sample rate conversion introduces artifacts

Instead of relying on MLS (which is sensitive to distortion), this uses
tone bursts at multiple frequencies for robust echo detection.
"""

import numpy as np
from scipy.io import wavfile
import argparse


def generate_tone_burst(frequency: float, duration_s: float, 
                       sample_rate: int, amplitude: float = 0.7,
                       fade_ms: float = 10.0) -> np.ndarray:
    """
    Generate a tone burst with smooth fade in/out.
    
    Tone bursts are ideal for SDR environments because:
    - Single frequency is robust to distortion
    - Easy to detect even with noise
    - Clear onset/offset for delay measurement
    - Less sensitive to bandwidth limitations
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


def generate_chirp(duration_s: float, f_start: float, f_end: float,
                   sample_rate: int, amplitude: float = 0.7) -> np.ndarray:
    """Generate linear chirp sweep."""
    n_samples = int(duration_s * sample_rate)
    t = np.arange(n_samples) / sample_rate
    
    # Linear chirp
    k = (f_end - f_start) / duration_s
    phase = 2 * np.pi * (f_start * t + 0.5 * k * t**2)
    chirp = amplitude * np.sin(phase)
    
    # Apply fade in/out
    fade_samples = int(0.01 * sample_rate)  # 10ms fade
    if fade_samples > 0 and fade_samples < n_samples // 2:
        fade_in = 0.5 * (1 - np.cos(np.pi * np.arange(fade_samples) / fade_samples))
        chirp[:fade_samples] *= fade_in
        
        fade_out = 0.5 * (1 + np.cos(np.pi * np.arange(fade_samples) / fade_samples))
        chirp[-fade_samples:] *= fade_out
    
    return chirp


def generate_silence(duration_s: float, sample_rate: int) -> np.ndarray:
    """Generate silence (zeros)."""
    return np.zeros(int(duration_s * sample_rate))


def create_sdr_echo_test_signal(sample_rate: int = 8000,
                                duration_s: float = 10.0,
                                amplitude: float = 0.7) -> np.ndarray:
    """
    Create SDR-optimized echo test signal.
    
    Structure:
      [silence]
      [chirp 500-2000 Hz]
      [silence]
      [tone burst 800 Hz]
      [silence]
      [tone burst 1000 Hz]
      [silence]
      [tone burst 1200 Hz]
      [silence]
      [tone burst 1400 Hz]
      [silence]
      [tone burst 1600 Hz]
      [silence]
      [tone burst 800 Hz]
      [silence]
      [tone burst 1000 Hz]
      [silence]
      [tone burst 1200 Hz]
      [silence]
      [chirp 500-2000 Hz]
      [silence]
    
    The multiple tone bursts at different frequencies provide:
    - Robust echo detection even with severe distortion
    - Multiple correlation opportunities
    - Frequency-dependent echo characterization
    - Clear audible markers
    - Longer duration for AEC convergence measurement
    
    Args:
        sample_rate: Sample rate in Hz (8000 or 16000)
        duration_s: Total duration in seconds (default 10.0)
        amplitude: Signal amplitude (default 0.7 for good SNR)
    
    Returns:
        Test signal as float array
    """
    parts = []
    
    # 1. Leading silence (0.5s for noise floor)
    parts.append(generate_silence(0.5, sample_rate))
    
    # 2. Initial chirp marker (500-2000 Hz, 400ms)
    parts.append(generate_chirp(0.4, 500, 2000, sample_rate, amplitude))
    parts.append(generate_silence(0.3, sample_rate))
    
    # 3. First series of tone bursts at different frequencies
    # Each tone burst is 400ms with 300ms silence after
    frequencies = [800, 1000, 1200, 1400, 1600]
    for freq in frequencies:
        parts.append(generate_tone_burst(freq, 0.4, sample_rate, amplitude))
        parts.append(generate_silence(0.3, sample_rate))
    
    # 4. Second series of tone bursts (for longer signal and more data)
    for freq in [800, 1000, 1200]:
        parts.append(generate_tone_burst(freq, 0.4, sample_rate, amplitude))
        parts.append(generate_silence(0.3, sample_rate))
    
    # 5. Final chirp marker (500-2000 Hz, 400ms)
    parts.append(generate_chirp(0.4, 500, 2000, sample_rate, amplitude))
    
    # 6. Trailing silence (0.5s for echo tail)
    parts.append(generate_silence(0.5, sample_rate))
    
    # Concatenate all parts
    signal = np.concatenate(parts)
    
    # Trim or pad to exact duration
    target_samples = int(duration_s * sample_rate)
    if len(signal) > target_samples:
        signal = signal[:target_samples]
    elif len(signal) < target_samples:
        padding = np.zeros(target_samples - len(signal))
        signal = np.concatenate([signal, padding])
    
    return signal


def save_wav(signal: np.ndarray, filename: str, sample_rate: int):
    """Save signal to 16-bit WAV file."""
    # Clip to prevent overflow
    clipped = np.clip(signal, -1.0, 1.0)
    # Convert to 16-bit integer
    int16_signal = (clipped * 32767).astype(np.int16)
    wavfile.write(filename, sample_rate, int16_signal)


def main():
    parser = argparse.ArgumentParser(
        description="Generate SDR-optimized echo test signal",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This signal is specifically designed for SDR environments where:
- Non-linear distortion is common
- Frequency response may be limited
- Timing jitter is present
- MLS signals don't correlate well

The signal uses tone bursts instead of MLS for robust echo detection.

Examples:
  %(prog)s -o sdr_test.wav
  %(prog)s -o sdr_test_16k.wav -r 16000
  %(prog)s -o sdr_test.wav --amplitude 0.25
        """
    )
    
    parser.add_argument("-o", "--output", default="sdr_echo_test.wav",
                        help="Output WAV filename (default: sdr_echo_test.wav)")
    parser.add_argument("-r", "--rate", type=int, default=8000,
                        help="Sample rate in Hz (default: 8000)")
    parser.add_argument("--amplitude", type=float, default=0.7,
                        help="Signal amplitude 0-1 (default: 0.7)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Duration in seconds (default: 10.0)")
    
    args = parser.parse_args()
    
    print(f"Generating SDR-optimized echo test signal...")
    print(f"  Sample rate: {args.rate} Hz")
    print(f"  Duration: {args.duration} seconds")
    print(f"  Amplitude: {args.amplitude}")
    print(f"  Structure: chirp + 8 tone bursts (800-1600 Hz) + chirp")
    
    signal = create_sdr_echo_test_signal(
        sample_rate=args.rate,
        duration_s=args.duration,
        amplitude=args.amplitude
    )
    
    duration_s = len(signal) / args.rate
    print(f"  Total duration: {duration_s:.2f} seconds ({len(signal)} samples)")
    
    save_wav(signal, args.output, args.rate)
    print(f"  Saved to: {args.output}")
    print()
    print("This signal is optimized for:")
    print("  • SDR environments with distortion/clipping")
    print("  • Noisy signal chains (20-200ms delays)")
    print("  • Systems where MLS correlation fails")
    print("  • Digital signal processing with timing jitter")


if __name__ == "__main__":
    main()
