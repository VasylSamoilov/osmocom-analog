#!/usr/bin/env python3
"""
Simple Echo Analyzer for SDR Environments

This analyzer doesn't rely on cross-correlation (which fails in noisy SDR environments).
Instead, it uses simple power measurements and timing analysis.

For SDR environments where:
- Correlation-based methods fail
- Signal is heavily distorted
- You just need to know: is there echo? how bad is it?
"""

import numpy as np
from scipy.io import wavfile
import argparse


def load_wav(filename):
    """Load WAV file and normalize to float."""
    rate, data = wavfile.read(filename)
    if len(data.shape) > 1:
        data = data[:, 0]  # Take first channel
    
    # Normalize to float
    if data.dtype == np.int16:
        data = data.astype(np.float64) / 32768.0
    else:
        data = data.astype(np.float64)
    
    return data, rate


def calculate_power_dbfs(signal):
    """Calculate RMS power in dBFS."""
    rms = np.sqrt(np.mean(signal**2))
    if rms < 1e-10:
        return -100.0
    return 20 * np.log10(rms)


def find_signal_regions(signal, sample_rate, threshold_db=-40):
    """
    Find regions where signal is present (above threshold).
    Returns list of (start_sample, end_sample) tuples.
    """
    # Calculate power in 50ms windows
    window_samples = int(0.05 * sample_rate)
    hop_samples = int(0.025 * sample_rate)
    
    regions = []
    pos = 0
    in_signal = False
    region_start = 0
    
    while pos + window_samples < len(signal):
        window = signal[pos:pos + window_samples]
        power_db = calculate_power_dbfs(window)
        
        if power_db > threshold_db:
            if not in_signal:
                # Start of signal region
                in_signal = True
                region_start = pos
        else:
            if in_signal:
                # End of signal region
                in_signal = False
                regions.append((region_start, pos))
        
        pos += hop_samples
    
    # Close last region if still in signal
    if in_signal:
        regions.append((region_start, len(signal)))
    
    return regions


def analyze_echo_simple(tx_file, rx_file):
    """
    Simple echo analysis that doesn't rely on correlation.
    
    Method:
    1. Find when TX signal is active
    2. Measure TX power during active periods
    3. Measure RX power during same periods (accounting for delay)
    4. Calculate ERL = TX_power - RX_power
    """
    print(f"Loading TX: {tx_file}")
    tx, tx_rate = load_wav(tx_file)
    
    print(f"Loading RX: {rx_file}")
    rx, rx_rate = load_wav(rx_file)
    
    if tx_rate != rx_rate:
        print(f"WARNING: Sample rate mismatch (TX={tx_rate}, RX={rx_rate})")
    
    sample_rate = tx_rate
    
    # Make signals same length
    min_len = min(len(tx), len(rx))
    tx = tx[:min_len]
    rx = rx[:min_len]
    
    print("\n" + "="*70)
    print("SIMPLE ECHO ANALYSIS (Power-Based, No Correlation)")
    print("="*70)
    print(f"Sample Rate: {sample_rate} Hz")
    print(f"Duration: {len(tx)/sample_rate:.2f} seconds")
    print()
    
    # Overall power measurements
    tx_power_overall = calculate_power_dbfs(tx)
    rx_power_overall = calculate_power_dbfs(rx)
    
    # Estimate noise floor from first 0.3 seconds
    noise_samples = int(0.3 * sample_rate)
    noise_floor = calculate_power_dbfs(rx[:noise_samples])
    
    print("OVERALL POWER MEASUREMENTS")
    print("-" * 70)
    print(f"TX Signal Power:     {tx_power_overall:6.1f} dBFS")
    print(f"RX Signal Power:     {rx_power_overall:6.1f} dBFS")
    print(f"Noise Floor:         {noise_floor:6.1f} dBFS")
    print()
    
    # Simple ERL calculation (no delay compensation)
    # This assumes TX and RX are roughly aligned
    erl_simple = tx_power_overall - rx_power_overall
    
    print("ECHO RETURN LOSS (Simple Method)")
    print("-" * 70)
    print(f"ERL (TX - RX):       {erl_simple:6.1f} dB")
    print()
    
    # Interpretation
    print("INTERPRETATION")
    print("-" * 70)
    
    if erl_simple < 3:
        print(f"⚠ SEVERE ECHO: ERL = {erl_simple:.1f} dB")
        print("  → Echo is almost as loud as the original signal")
        print("  → This will cause feedback/howling")
        print("  → URGENT: Enable echo cancellation or reduce loop gain")
    elif erl_simple < 6:
        print(f"⚠ STRONG ECHO: ERL = {erl_simple:.1f} dB")
        print("  → Significant echo present")
        print("  → Echo cancellation strongly recommended")
    elif erl_simple < 15:
        print(f"⚠ MODERATE ECHO: ERL = {erl_simple:.1f} dB")
        print("  → Noticeable echo")
        print("  → Echo cancellation recommended")
    elif erl_simple < 25:
        print(f"✓ ACCEPTABLE: ERL = {erl_simple:.1f} dB")
        print("  → Echo is attenuated but still present")
        print("  → May be acceptable depending on use case")
    else:
        print(f"✓ GOOD: ERL = {erl_simple:.1f} dB")
        print("  → Echo is well suppressed")
    
    print()
    
    # Check for potential feedback
    rx_above_noise = rx_power_overall - noise_floor
    if erl_simple < 3 and rx_above_noise > 20:
        print("⚠⚠⚠ FEEDBACK RISK ⚠⚠⚠")
        print("  → RX signal is very strong relative to noise")
        print("  → ERL is very low (< 3 dB)")
        print("  → High risk of acoustic feedback loop")
        print("  → Reduce gain or enable echo cancellation immediately")
        print()
    
    # Time-based analysis
    print("TIME-BASED ANALYSIS")
    print("-" * 70)
    
    # Analyze power over time in 100ms windows
    window_ms = 100
    window_samples = int(window_ms * sample_rate / 1000)
    hop_samples = int(50 * sample_rate / 1000)  # 50ms hop
    
    erls = []
    times = []
    
    pos = int(0.5 * sample_rate)  # Skip first 0.5s
    while pos + window_samples < len(tx):
        tx_window = tx[pos:pos + window_samples]
        rx_window = rx[pos:pos + window_samples]
        
        tx_pwr = calculate_power_dbfs(tx_window)
        rx_pwr = calculate_power_dbfs(rx_window)
        
        erl = tx_pwr - rx_pwr
        
        # Only include if TX has signal
        if tx_pwr > noise_floor + 10:
            erls.append(erl)
            times.append(pos / sample_rate)
        
        pos += hop_samples
    
    if erls:
        print(f"Measurements: {len(erls)} windows")
        print(f"ERL range: {min(erls):.1f} to {max(erls):.1f} dB")
        print(f"ERL mean: {np.mean(erls):.1f} dB")
        print(f"ERL std dev: {np.std(erls):.1f} dB")
        
        # Check for improvement over time (AEC convergence)
        if len(erls) > 10:
            early_erl = np.mean(erls[:5])
            late_erl = np.mean(erls[-5:])
            improvement = late_erl - early_erl
            
            print()
            print(f"Early ERL (first 5 windows): {early_erl:.1f} dB")
            print(f"Late ERL (last 5 windows):   {late_erl:.1f} dB")
            print(f"Improvement:                 {improvement:.1f} dB")
            
            if improvement > 6:
                print("  → Significant improvement detected")
                print("  → AEC may be converging")
            elif improvement > 3:
                print("  → Moderate improvement detected")
                print("  → Some echo reduction occurring")
            elif improvement < -3:
                print("  → Echo is getting WORSE over time")
                print("  → Check for feedback or instability")
            else:
                print("  → No significant change over time")
                print("  → No AEC convergence detected")
    
    print()
    print("="*70)
    
    return {
        'tx_power': tx_power_overall,
        'rx_power': rx_power_overall,
        'noise_floor': noise_floor,
        'erl': erl_simple,
        'erl_time_series': list(zip(times, erls)) if erls else []
    }


def main():
    parser = argparse.ArgumentParser(
        description="Simple echo analyzer for SDR environments (no correlation)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This analyzer uses simple power measurements instead of correlation.
Use this when the standard analyzer gives incorrect results due to
poor correlation in noisy SDR environments.

Examples:
  %(prog)s --tx test.wav --rx recording.wav
        """
    )
    
    parser.add_argument("--tx", required=True, help="TX signal WAV file")
    parser.add_argument("--rx", required=True, help="RX signal WAV file")
    
    args = parser.parse_args()
    
    analyze_echo_simple(args.tx, args.rx)


if __name__ == "__main__":
    main()
