#!/usr/bin/env python3
"""
Test for sip_echo_tester.py efficiency mode integration.

This test verifies that the --efficiency flag properly integrates with
the EchoTester class and uses the extended test signal and efficiency analysis.
"""

import sys
import numpy as np
from pathlib import Path

# Import the modules we need
from echo_test_signal import create_efficiency_test_signal
from echo_analyzer import EchoAnalyzer


def test_efficiency_signal_generation():
    """Test that efficiency test signal can be generated."""
    print("Testing efficiency signal generation...")
    
    signal = create_efficiency_test_signal(
        sample_rate=8000,
        duration_s=5.0,
        mls_bits=12,
        amplitude=0.5
    )
    
    # Verify signal properties
    assert len(signal) == 8000 * 5, f"Expected 40000 samples, got {len(signal)}"
    assert isinstance(signal, np.ndarray), "Signal should be numpy array"
    assert signal.dtype == np.float32 or signal.dtype == np.float64, "Signal should be float"
    assert np.max(np.abs(signal)) <= 0.5, "Signal amplitude should not exceed 0.5"
    
    print("✓ Efficiency signal generation works correctly")
    return True


def test_efficiency_analysis_integration():
    """Test that efficiency analysis can be performed on generated signal."""
    print("\nTesting efficiency analysis integration...")
    
    # Generate test signal
    tx_signal = create_efficiency_test_signal(
        sample_rate=8000,
        duration_s=5.0,
        mls_bits=12,
        amplitude=0.5
    )
    
    # Simulate echo by adding delayed and attenuated copy
    delay_samples = 100  # 12.5ms at 8kHz
    attenuation = 0.3
    rx_signal = np.zeros_like(tx_signal)
    rx_signal[delay_samples:] = tx_signal[:-delay_samples] * attenuation
    
    # Add some noise
    noise = np.random.normal(0, 0.01, len(rx_signal))
    rx_signal = rx_signal + noise
    
    # Perform efficiency analysis
    analyzer = EchoAnalyzer(tx_signal, rx_signal, sample_rate=8000)
    measurement = analyzer.efficiency_analysis()
    
    # Verify measurement has efficiency fields
    assert hasattr(measurement, 'erle_db'), "Measurement should have erle_db field"
    assert hasattr(measurement, 'convergence_time_ms'), "Measurement should have convergence_time_ms field"
    assert hasattr(measurement, 'residual_echo_dbfs'), "Measurement should have residual_echo_dbfs field"
    assert hasattr(measurement, 'erl_time_series'), "Measurement should have erl_time_series field"
    assert hasattr(measurement, 'aec_performance'), "Measurement should have aec_performance field"
    assert hasattr(measurement, 'converged'), "Measurement should have converged field"
    
    # Verify report generation
    report = analyzer.generate_efficiency_report(measurement)
    assert isinstance(report, str), "Report should be a string"
    assert len(report) > 0, "Report should not be empty"
    assert "ERLE" in report or "Echo Return Loss Enhancement" in report, "Report should mention ERLE"
    
    # Verify JSON serialization
    measurement_dict = measurement.to_dict()
    assert isinstance(measurement_dict, dict), "to_dict() should return a dictionary"
    assert 'erle_db' in measurement_dict, "Dictionary should contain erle_db"
    assert 'convergence_time_ms' in measurement_dict, "Dictionary should contain convergence_time_ms"
    
    print("✓ Efficiency analysis integration works correctly")
    return True


def test_echo_tester_efficiency_mode_init():
    """Test that EchoTester can be initialized in efficiency mode."""
    print("\nTesting EchoTester efficiency mode initialization...")
    
    # We can't fully test EchoTester without a SIP server, but we can test
    # that it initializes correctly in efficiency mode
    
    # Import here to avoid pjsip dependency issues
    try:
        from sip_echo_tester_pjsip import EchoTester
    except ImportError as e:
        print(f"⚠ Skipping EchoTester test due to import error: {e}")
        return True
    
    # Create a dummy test signal file (not needed in efficiency mode, but required by constructor)
    dummy_signal_path = "dummy_test_signal.wav"
    
    try:
        # Initialize in efficiency mode (should generate signal internally)
        tester = EchoTester(
            server="127.0.0.1",
            server_port=5060,
            username="test",
            password="test",
            my_ip="127.0.0.1",
            sip_port=5061,
            test_signal_path=dummy_signal_path,
            output_dir="test_recordings",
            efficiency_mode=True
        )
        
        # Verify efficiency mode is set
        assert tester.efficiency_mode == True, "Efficiency mode should be enabled"
        
        # Verify test signal was generated
        assert tester.test_signal is not None, "Test signal should be generated"
        assert len(tester.test_signal) > 0, "Test signal should not be empty"
        assert tester.sample_rate == 8000, "Sample rate should be 8000 Hz"
        
        # Verify signal is approximately 5 seconds
        expected_samples = 8000 * 5
        assert abs(len(tester.test_signal) - expected_samples) < 1000, \
            f"Signal should be ~5 seconds ({expected_samples} samples), got {len(tester.test_signal)}"
        
        print("✓ EchoTester efficiency mode initialization works correctly")
        return True
        
    except Exception as e:
        print(f"✗ EchoTester initialization failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("="*60)
    print("  SIP Echo Tester Efficiency Mode Integration Tests")
    print("="*60)
    
    tests = [
        test_efficiency_signal_generation,
        test_efficiency_analysis_integration,
        test_echo_tester_efficiency_mode_init,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"✗ Test {test.__name__} failed with exception: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "="*60)
    print(f"Results: {passed} passed, {failed} failed")
    print("="*60)
    
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
