#!/usr/bin/env python3
"""
Unit tests for echo_test_signal.py

Tests the signal generation functions, particularly the efficiency test signal.
"""

import numpy as np
from echo_test_signal import (
    generate_mls,
    create_echo_test_signal,
    create_efficiency_test_signal
)


def test_create_efficiency_test_signal_basic():
    """Test that create_efficiency_test_signal() generates a signal with correct duration."""
    sample_rate = 8000
    duration_s = 5.0
    
    signal = create_efficiency_test_signal(
        sample_rate=sample_rate,
        duration_s=duration_s,
        mls_bits=12,
        amplitude=0.5
    )
    
    # Verify signal length matches requested duration
    expected_samples = int(sample_rate * duration_s)
    assert len(signal) == expected_samples, \
        f"Expected {expected_samples} samples, got {len(signal)}"
    
    # Verify signal is not all zeros
    assert np.any(signal != 0), "Signal should not be all zeros"
    
    # Verify amplitude is within bounds
    assert np.max(np.abs(signal)) <= 0.5, "Signal amplitude should not exceed 0.5"
    
    print("✓ test_create_efficiency_test_signal_basic passed")


def test_create_efficiency_test_signal_has_silence():
    """Test that the signal has leading and trailing silence."""
    sample_rate = 8000
    duration_s = 5.0
    
    signal = create_efficiency_test_signal(
        sample_rate=sample_rate,
        duration_s=duration_s,
        mls_bits=12,
        amplitude=0.5
    )
    
    # Check leading silence (first 0.3 seconds should be mostly zeros)
    silence_samples = int(0.3 * sample_rate)
    leading_silence = signal[:silence_samples]
    assert np.all(leading_silence == 0), "Leading portion should be silence"
    
    # Check trailing silence (last 0.3 seconds should be mostly zeros)
    trailing_silence = signal[-silence_samples:]
    assert np.all(trailing_silence == 0), "Trailing portion should be silence"
    
    print("✓ test_create_efficiency_test_signal_has_silence passed")


def test_create_efficiency_test_signal_continuous_mls():
    """Test that the MLS portion is continuous (no gaps) when chirps disabled."""
    sample_rate = 8000
    duration_s = 2.0
    mls_bits = 12
    
    # Test with chirps DISABLED to verify continuous MLS
    signal = create_efficiency_test_signal(
        sample_rate=sample_rate,
        duration_s=duration_s,
        mls_bits=mls_bits,
        amplitude=0.5,
        use_chirps=False  # Disable chirps for continuous MLS test
    )
    
    # Skip leading silence (0.3s)
    silence_samples = int(0.3 * sample_rate)
    
    # Get the MLS portion (between leading and trailing silence)
    mls_portion = signal[silence_samples:-silence_samples]
    
    # Verify MLS portion is not empty
    assert len(mls_portion) > 0, "MLS portion should not be empty"
    
    # Verify MLS portion has non-zero values (it's active signal)
    assert np.any(mls_portion != 0), "MLS portion should have non-zero values"
    
    # Generate reference MLS to verify structure
    mls = generate_mls(mls_bits, amplitude=0.5)
    mls_length = len(mls)
    
    # Verify the MLS repeats continuously
    # Check that the pattern repeats by comparing segments
    if len(mls_portion) >= 2 * mls_length:
        first_repeat = mls_portion[:mls_length]
        second_repeat = mls_portion[mls_length:2*mls_length]
        
        # The repeats should be identical (continuous MLS)
        assert np.allclose(first_repeat, second_repeat), \
            "MLS should repeat continuously without gaps"
    
    print("✓ test_create_efficiency_test_signal_continuous_mls passed")


def test_create_efficiency_test_signal_different_durations():
    """Test that different durations produce correctly sized signals."""
    sample_rate = 8000
    
    for duration_s in [1.0, 3.0, 5.0, 10.0]:
        signal = create_efficiency_test_signal(
            sample_rate=sample_rate,
            duration_s=duration_s,
            mls_bits=12,
            amplitude=0.5
        )
        
        expected_samples = int(sample_rate * duration_s)
        assert len(signal) == expected_samples, \
            f"For {duration_s}s: expected {expected_samples} samples, got {len(signal)}"
    
    print("✓ test_create_efficiency_test_signal_different_durations passed")


def test_create_efficiency_test_signal_different_sample_rates():
    """Test that different sample rates work correctly."""
    duration_s = 5.0
    
    for sample_rate in [8000, 16000, 48000]:
        signal = create_efficiency_test_signal(
            sample_rate=sample_rate,
            duration_s=duration_s,
            mls_bits=12,
            amplitude=0.5
        )
        
        expected_samples = int(sample_rate * duration_s)
        assert len(signal) == expected_samples, \
            f"For {sample_rate}Hz: expected {expected_samples} samples, got {len(signal)}"
    
    print("✓ test_create_efficiency_test_signal_different_sample_rates passed")


def test_create_efficiency_test_signal_amplitude():
    """Test that amplitude parameter is respected."""
    sample_rate = 8000
    duration_s = 2.0
    amplitude = 0.3
    
    signal = create_efficiency_test_signal(
        sample_rate=sample_rate,
        duration_s=duration_s,
        mls_bits=12,
        amplitude=amplitude
    )
    
    # Maximum absolute value should not exceed the specified amplitude
    max_amplitude = np.max(np.abs(signal))
    assert max_amplitude <= amplitude, \
        f"Max amplitude {max_amplitude} should not exceed {amplitude}"
    
    # Should have some values close to the amplitude (not all tiny)
    assert max_amplitude > amplitude * 0.9, \
        f"Max amplitude {max_amplitude} should be close to {amplitude}"
    
    print("✓ test_create_efficiency_test_signal_amplitude passed")


def test_create_efficiency_test_signal_mls_bits():
    """Test that different MLS bit lengths work correctly."""
    sample_rate = 8000
    duration_s = 2.0
    
    for mls_bits in [10, 11, 12, 13]:
        signal = create_efficiency_test_signal(
            sample_rate=sample_rate,
            duration_s=duration_s,
            mls_bits=mls_bits,
            amplitude=0.5
        )
        
        expected_samples = int(sample_rate * duration_s)
        assert len(signal) == expected_samples, \
            f"For {mls_bits} bits: expected {expected_samples} samples, got {len(signal)}"
        
        # Verify signal is not all zeros
        assert np.any(signal != 0), f"Signal with {mls_bits} bits should not be all zeros"
    
    print("✓ test_create_efficiency_test_signal_mls_bits passed")


def test_create_efficiency_test_signal_vs_standard():
    """Test that efficiency signal is longer than standard signal."""
    sample_rate = 8000
    
    # Create standard test signal
    standard_signal = create_echo_test_signal(
        sample_rate=sample_rate,
        mls_bits=12,
        mls_repeats=2,
        chirp_duration=0.5,
        amplitude=0.5
    )
    
    # Create efficiency test signal (default 5 seconds)
    efficiency_signal = create_efficiency_test_signal(
        sample_rate=sample_rate,
        duration_s=5.0,
        mls_bits=12,
        amplitude=0.5
    )
    
    # Efficiency signal should be significantly longer
    assert len(efficiency_signal) > len(standard_signal), \
        "Efficiency signal should be longer than standard signal"
    
    # Efficiency signal should be at least 5 seconds
    assert len(efficiency_signal) >= 5 * sample_rate, \
        "Efficiency signal should be at least 5 seconds"
    
    print("✓ test_create_efficiency_test_signal_vs_standard passed")


def test_create_efficiency_test_signal_cross_correlation():
    """Test that the signal allows cross-correlation at any point."""
    sample_rate = 8000
    duration_s = 3.0
    mls_bits = 12
    
    signal = create_efficiency_test_signal(
        sample_rate=sample_rate,
        duration_s=duration_s,
        mls_bits=mls_bits,
        amplitude=0.5
    )
    
    # Generate reference MLS
    mls = generate_mls(mls_bits, amplitude=0.5)
    mls_length = len(mls)
    
    # Skip leading silence
    silence_samples = int(0.3 * sample_rate)
    
    # Test cross-correlation at multiple points in the signal
    # This verifies that MLS is continuous and allows sliding window analysis
    test_points = [
        silence_samples + 1000,
        silence_samples + 5000,
        silence_samples + 10000,
    ]
    
    for start_idx in test_points:
        if start_idx + mls_length < len(signal) - silence_samples:
            segment = signal[start_idx:start_idx + mls_length]
            
            # Cross-correlate with reference MLS
            correlation = np.correlate(segment, mls, mode='valid')
            
            # Should have a strong correlation peak
            max_corr = np.max(np.abs(correlation))
            assert max_corr > 0.1, \
                f"At position {start_idx}: correlation should be strong, got {max_corr}"
    
    print("✓ test_create_efficiency_test_signal_cross_correlation passed")


if __name__ == "__main__":
    # Run all tests
    test_create_efficiency_test_signal_basic()
    test_create_efficiency_test_signal_has_silence()
    test_create_efficiency_test_signal_continuous_mls()
    test_create_efficiency_test_signal_different_durations()
    test_create_efficiency_test_signal_different_sample_rates()
    test_create_efficiency_test_signal_amplitude()
    test_create_efficiency_test_signal_mls_bits()
    test_create_efficiency_test_signal_vs_standard()
    test_create_efficiency_test_signal_cross_correlation()
    
    print("\n✅ All tests passed!")
