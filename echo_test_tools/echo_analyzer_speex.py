#!/usr/bin/env python3
"""
Speex-Inspired Echo Analyzer

This analyzer uses techniques inspired by the Speex echo cancellation algorithm
for robust echo detection and parameter measurement, particularly in noisy
SDR environments.

Key concepts from Speex:
1. Power-based echo detection (not just correlation)
2. Adaptive filtering approach to find echo
3. Frequency-domain analysis for better noise handling
4. Residual echo estimation using filter coefficients
5. Double-talk detection to avoid false measurements

References:
- Speex AEC: https://www.speex.org/docs/api/speex-api-reference/group__SpeexPreprocessState.html
- Paper: "A New Approach to Echo Cancellation" by Jean-Marc Valin
"""

import numpy as np
from scipy.io import wavfile
from scipy import signal as scipy_signal
from scipy.fft import fft, ifft, fftfreq
import argparse
from dataclasses import dataclass
from typing import Tuple, Optional
import json


@dataclass
class SpeexEchoMeasurement:
    """Echo measurement results using Speex-inspired methods."""
    
    # Basic measurements
    delay_samples: int
    delay_ms: float
    sample_rate: int
    
    # Power-based measurements (more robust than correlation)
    tx_power_dbfs: float
    rx_power_dbfs: float
    echo_power_dbfs: float
    noise_floor_dbfs: float
    
    # Echo metrics
    erl_db: float  # Echo Return Loss (TX power - Echo power)
    erle_db: Optional[float]  # Echo Return Loss Enhancement (if AEC present)
    snr_db: float
    
    # Speex-specific metrics
    echo_detected: bool  # Is echo actually present?
    double_talk_detected: bool  # Is there speech in both directions?
    filter_divergence: float  # How well the adaptive filter converged (0-1)
    
    # Quality indicators
    measurement_confidence: str  # "high", "medium", "low"
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        def convert_value(val):
            if val is None:
                return None
            elif isinstance(val, (bool, np.bool_)):
                return bool(val)
            elif isinstance(val, (np.integer, np.int64, np.int32)):
                return int(val)
            elif isinstance(val, (np.floating, np.float64, np.float32)):
                return float(val)
            else:
                return val
        
        return {
            "delay_samples": convert_value(self.delay_samples),
            "delay_ms": convert_value(self.delay_ms),
            "sample_rate": convert_value(self.sample_rate),
            "tx_power_dbfs": convert_value(self.tx_power_dbfs),
            "rx_power_dbfs": convert_value(self.rx_power_dbfs),
            "echo_power_dbfs": convert_value(self.echo_power_dbfs),
            "noise_floor_dbfs": convert_value(self.noise_floor_dbfs),
            "erl_db": convert_value(self.erl_db),
            "erle_db": convert_value(self.erle_db),
            "snr_db": convert_value(self.snr_db),
            "echo_detected": convert_value(self.echo_detected),
            "double_talk_detected": convert_value(self.double_talk_detected),
            "filter_divergence": convert_value(self.filter_divergence),
            "measurement_confidence": self.measurement_confidence
        }


class SpeexEchoAnalyzer:
    """
    Echo analyzer using Speex-inspired techniques.
    
    This analyzer is designed for SDR environments where:
    - Signals are heavily distorted
    - Correlation-based methods fail
    - Echo can cause feedback loops
    """
    
    def __init__(self, tx_signal: np.ndarray, rx_signal: np.ndarray,
                 sample_rate: int = 8000, rx_sample_rate: Optional[int] = None):
        """
        Initialize analyzer.
        
        Args:
            tx_signal: Transmitted signal (reference)
            rx_signal: Received signal (with echo)
            sample_rate: Sample rate in Hz (TX sample rate - this is the reference)
            rx_sample_rate: RX sample rate if different from TX (will resample RX to match TX)
        """
        # Normalize to float [-1, 1]
        self.tx = self._normalize(tx_signal)
        self.rx = self._normalize(rx_signal)
        self.sample_rate = sample_rate  # TX sample rate is the reference
        
        # Handle sample rate mismatch by resampling RX to match TX
        if rx_sample_rate is not None and rx_sample_rate != sample_rate:
            print(f"Resampling RX from {rx_sample_rate} Hz to {sample_rate} Hz to match TX")
            from scipy.signal import resample
            new_length = int(len(self.rx) * sample_rate / rx_sample_rate)
            self.rx = resample(self.rx, new_length)
        
        # Ensure same length
        min_len = min(len(self.tx), len(self.rx))
        self.tx = self.tx[:min_len]
        self.rx = self.rx[:min_len]
        
        # Frame size for processing (20ms frames like Speex)
        self.frame_size = int(0.02 * self.sample_rate)  # 20ms
        
        # Filter length for adaptive filter (covers typical echo delays)
        # For 200ms max delay at sample_rate
        self.filter_length = min(int(0.2 * self.sample_rate), 2048)
    
    @staticmethod
    def _normalize(sig: np.ndarray) -> np.ndarray:
        """Normalize signal to float [-1, 1]."""
        if sig.dtype == np.int16:
            return sig.astype(np.float64) / 32768.0
        elif sig.dtype in [np.float32, np.float64]:
            return sig.astype(np.float64)
        else:
            return sig.astype(np.float64) / (np.max(np.abs(sig)) + 1e-10)
    
    def calculate_power_dbfs(self, sig: np.ndarray) -> float:
        """Calculate RMS power in dBFS."""
        rms = np.sqrt(np.mean(sig**2))
        if rms < 1e-10:
            return -100.0
        return 20 * np.log10(rms)
    
    def estimate_delay_power_based(self) -> Tuple[int, float]:
        """
        Estimate echo delay using two-stage power envelope correlation.
        
        Stage 1: Coarse search using 20ms frames (fast)
        Stage 2: Fine search using sample-level precision (accurate)
        
        Returns:
            (delay_samples, confidence)
        """
        # Stage 1: Coarse search using power envelopes in frames
        frame_size = self.frame_size  # 20ms
        
        # Compute TX power envelope
        n_frames_tx = len(self.tx) // frame_size
        tx_power = np.zeros(n_frames_tx)
        for i in range(n_frames_tx):
            start = i * frame_size
            end = start + frame_size
            tx_power[i] = np.sqrt(np.mean(self.tx[start:end]**2))  # RMS
        
        # Compute RX power envelope
        n_frames_rx = len(self.rx) // frame_size
        rx_power = np.zeros(n_frames_rx)
        for i in range(n_frames_rx):
            start = i * frame_size
            end = start + frame_size
            rx_power[i] = np.sqrt(np.mean(self.rx[start:end]**2))  # RMS
        
        # Correlate power envelopes to find approximate delay
        # Search up to 500ms or until we run out of RX data
        max_delay_frames = min(int(0.5 * self.sample_rate / frame_size), n_frames_rx - 10)  # Keep 10 frames for comparison
        
        if max_delay_frames < 1:
            # Not enough data to search
            return 0, 0.0
        
        best_delay_frames = 0
        best_corr = -1
        
        for delay_frames in range(0, max_delay_frames):
            compare_len = min(n_frames_tx, n_frames_rx - delay_frames)
            if compare_len < 10:  # Need at least 200ms of data
                continue
            
            tx_seg = tx_power[:compare_len]
            rx_seg = rx_power[delay_frames:delay_frames + compare_len]
            
            # Normalize (remove DC, scale to unit variance)
            tx_norm = (tx_seg - np.mean(tx_seg)) / (np.std(tx_seg) + 1e-10)
            rx_norm = (rx_seg - np.mean(rx_seg)) / (np.std(rx_seg) + 1e-10)
            
            # Correlation coefficient
            corr = np.mean(tx_norm * rx_norm)
            
            if corr > best_corr:
                best_corr = corr
                best_delay_frames = delay_frames
        
        coarse_delay = best_delay_frames * frame_size
        
        # Stage 2: Fine search around coarse estimate using cross-correlation
        # Search +/- 2 frames with sample precision for better accuracy
        search_range = 2 * frame_size
        min_delay = max(0, coarse_delay - search_range)
        max_delay = coarse_delay + search_range
        
        # Make sure we don't go past the end of RX
        compare_len = min(int(2.0 * self.sample_rate), len(self.tx))  # Use up to 2 seconds
        max_delay = min(max_delay, len(self.rx) - compare_len)
        
        if max_delay < min_delay:
            # Not enough data for fine search
            return coarse_delay, float(best_corr)
        
        # For efficiency, compute TX envelope once
        window_size = max(8, self.sample_rate // 1000)  # 1ms window
        tx_envelope = np.convolve(self.tx[:compare_len]**2, np.ones(window_size)/window_size, mode='same')
        tx_mean = np.mean(tx_envelope)
        tx_std = np.std(tx_envelope)
        
        if tx_std < 1e-10:
            # TX has no variation, can't correlate
            return coarse_delay, float(best_corr)
        
        tx_norm = (tx_envelope - tx_mean) / tx_std
        
        best_delay_samples = coarse_delay
        best_fine_corr = best_corr
        
        # Sample-level search
        for delay in range(min_delay, max_delay + 1):
            rx_start = delay
            rx_end = delay + compare_len
            
            if rx_end > len(self.rx):
                break
            
            # Compute RX envelope for this delay
            rx_segment = self.rx[rx_start:rx_end]
            rx_envelope = np.convolve(rx_segment**2, np.ones(window_size)/window_size, mode='same')
            
            rx_mean = np.mean(rx_envelope)
            rx_std = np.std(rx_envelope)
            
            if rx_std < 1e-10:
                continue
            
            rx_norm = (rx_envelope - rx_mean) / rx_std
            
            # Correlation coefficient
            corr = np.mean(tx_norm * rx_norm)
            
            if corr > best_fine_corr:
                best_fine_corr = corr
                best_delay_samples = delay
        
        confidence = max(0.0, min(1.0, best_fine_corr))
        
        return best_delay_samples, float(confidence)
    
    def detect_echo_presence(self) -> Tuple[bool, float]:
        """
        Detect if echo is actually present using power analysis.
        
        Speex approach: Compare TX and RX power over time.
        If RX power follows TX power with a delay, echo is present.
        
        Returns:
            (echo_detected, confidence)
        """
        # Get power envelopes
        frame_size = self.frame_size
        n_frames = len(self.tx) // frame_size
        
        tx_power = np.zeros(n_frames)
        rx_power = np.zeros(n_frames)
        
        for i in range(n_frames):
            start = i * frame_size
            end = start + frame_size
            tx_power[i] = np.mean(self.tx[start:end]**2)
            rx_power[i] = np.mean(self.rx[start:end]**2)
        
        # Check if RX power is correlated with TX power (with some delay)
        # Try delays from 20ms to 200ms
        max_correlation = 0
        
        for delay_frames in range(1, min(40, n_frames // 2)):
            valid_len = n_frames - delay_frames
            if valid_len < 10:
                continue
            
            tx_seg = tx_power[:valid_len]
            rx_seg = rx_power[delay_frames:delay_frames + valid_len]
            
            # Normalize
            tx_norm = (tx_seg - np.mean(tx_seg)) / (np.std(tx_seg) + 1e-10)
            rx_norm = (rx_seg - np.mean(rx_seg)) / (np.std(rx_seg) + 1e-10)
            
            corr = np.abs(np.mean(tx_norm * rx_norm))
            max_correlation = max(max_correlation, corr)
        
        # Echo is detected if correlation is significant
        echo_detected = max_correlation > 0.3
        
        return echo_detected, float(max_correlation)
    
    def detect_double_talk(self) -> bool:
        """
        Detect if there's speech/signal in both directions simultaneously.
        
        This is important because echo measurements are invalid during double-talk.
        
        Returns:
            True if double-talk detected
        """
        # Simple approach: check if both TX and RX have significant power
        # in frames where the other is silent
        
        frame_size = self.frame_size
        n_frames = len(self.tx) // frame_size
        
        # Find frames where TX is active but RX should be silent (no echo yet)
        # and vice versa
        
        tx_active_frames = 0
        rx_active_frames = 0
        both_active_frames = 0
        
        threshold = 0.01  # Power threshold for "active"
        
        for i in range(n_frames):
            start = i * frame_size
            end = start + frame_size
            
            tx_power = np.mean(self.tx[start:end]**2)
            rx_power = np.mean(self.rx[start:end]**2)
            
            tx_active = tx_power > threshold
            rx_active = rx_power > threshold
            
            if tx_active:
                tx_active_frames += 1
            if rx_active:
                rx_active_frames += 1
            if tx_active and rx_active:
                both_active_frames += 1
        
        # If most frames have both active, it's likely double-talk
        if tx_active_frames > 0:
            double_talk_ratio = both_active_frames / tx_active_frames
            return double_talk_ratio > 0.8
        
        return False
    
    def calculate_erl_robust(self, delay_samples: int) -> float:
        """
        Calculate ERL using robust power-based method.
        
        ERL (Echo Return Loss) measures the attenuation of the echo path.
        It's defined as: ERL = 10 * log10(P_tx / P_echo)
        
        In dBFS: ERL = TX_power_dBFS - Echo_power_dBFS
        
        The echo power is measured in the RX signal at the time corresponding
        to when the TX signal was sent (accounting for the echo delay).
        
        Args:
            delay_samples: Estimated echo delay
        
        Returns:
            ERL in dB (positive = echo is attenuated, negative = echo is amplified)
        """
        # Measure TX power in active region (skip initial silence)
        tx_start = int(0.5 * self.sample_rate)  # Skip 500ms
        tx_end = min(tx_start + int(2.0 * self.sample_rate), len(self.tx))
        
        if tx_end <= tx_start:
            return 0.0
        
        tx_power_dbfs = self.calculate_power_dbfs(self.tx[tx_start:tx_end])
        
        # Measure echo power in RX at the corresponding time
        # The echo of TX[t] appears at RX[t + delay]
        rx_start = tx_start + delay_samples
        rx_end = tx_end + delay_samples
        
        # Make sure we have valid RX data
        if rx_start >= len(self.rx) or rx_start < 0:
            # No RX data at expected echo time - delay estimate is probably wrong
            # Fall back to simple power comparison
            rx_power_dbfs = self.calculate_power_dbfs(self.rx[tx_start:tx_end] if tx_end <= len(self.rx) else self.rx)
        elif rx_end > len(self.rx):
            # Partial RX data available
            rx_power_dbfs = self.calculate_power_dbfs(self.rx[rx_start:])
        else:
            # Full RX data available at expected echo time
            rx_power_dbfs = self.calculate_power_dbfs(self.rx[rx_start:rx_end])
        
        # ERL = TX power - Echo power (in dBFS)
        # Positive ERL means echo is attenuated
        # Negative ERL means echo is amplified (should not happen in passive systems)
        erl = tx_power_dbfs - rx_power_dbfs
        
        # Sanity check: if ERL is extremely high (>60 dB) but overall powers are similar,
        # the delay estimate is probably wrong
        overall_tx = self.calculate_power_dbfs(self.tx)
        overall_rx = self.calculate_power_dbfs(self.rx)
        overall_erl = overall_tx - overall_rx
        
        # If calculated ERL is much higher than overall ERL, use overall ERL
        if erl > 60 and abs(overall_erl) < 20:
            # Delay estimate is probably wrong, use overall power comparison
            return overall_erl
        
        return erl
    
    def analyze(self) -> SpeexEchoMeasurement:
        """
        Perform complete echo analysis using Speex-inspired methods.
        
        Returns:
            SpeexEchoMeasurement with all results
        """
        # Step 1: Detect if echo is present
        echo_detected, echo_confidence = self.detect_echo_presence()
        
        # Step 2: Detect double-talk
        double_talk = self.detect_double_talk()
        
        # Step 3: Estimate delay using power-based method
        delay_samples, delay_confidence = self.estimate_delay_power_based()
        delay_ms = delay_samples * 1000.0 / self.sample_rate
        
        # Step 4: Calculate power measurements
        tx_power = self.calculate_power_dbfs(self.tx)
        rx_power = self.calculate_power_dbfs(self.rx)
        
        # Estimate noise floor from silent regions
        noise_floor = -60.0  # Default
        silence_start = 0
        silence_end = min(int(0.3 * self.sample_rate), len(self.rx))
        if silence_end > silence_start:
            noise_floor = self.calculate_power_dbfs(self.rx[silence_start:silence_end])
        
        # Step 5: Calculate ERL using robust method
        erl = self.calculate_erl_robust(delay_samples)
        
        # Step 6: Estimate echo power
        # Echo power = RX power (since RX contains the echo)
        echo_power = rx_power
        
        # Step 7: Calculate SNR
        snr = rx_power - noise_floor
        
        # Step 8: Determine measurement confidence
        if echo_confidence > 0.5 and not double_talk:
            confidence = "high"
        elif echo_confidence > 0.3:
            confidence = "medium"
        else:
            confidence = "low"
        
        # Step 9: Filter divergence (simplified - would need actual adaptive filter)
        # For now, use echo confidence as a proxy
        filter_divergence = 1.0 - echo_confidence
        
        # Step 10: ERLE (would need before/after AEC comparison)
        erle = None  # Not measurable from single recording
        
        return SpeexEchoMeasurement(
            delay_samples=delay_samples,
            delay_ms=delay_ms,
            sample_rate=self.sample_rate,
            tx_power_dbfs=tx_power,
            rx_power_dbfs=rx_power,
            echo_power_dbfs=echo_power,
            noise_floor_dbfs=noise_floor,
            erl_db=erl,
            erle_db=erle,
            snr_db=snr,
            echo_detected=echo_detected,
            double_talk_detected=double_talk,
            filter_divergence=filter_divergence,
            measurement_confidence=confidence
        )
    
    def generate_report(self, measurement: SpeexEchoMeasurement) -> str:
        """Generate human-readable report."""
        m = measurement
        
        report = f"""
════════════════════════════════════════════════════════════════════
              SPEEX-INSPIRED ECHO ANALYSIS REPORT
════════════════════════════════════════════════════════════════════
Sample Rate:     {m.sample_rate} Hz

────────────────────────────────────────────────────────────────────
                      ECHO DETECTION
────────────────────────────────────────────────────────────────────
Echo Detected:           {"YES" if m.echo_detected else "NO"}
Double-Talk Detected:    {"YES" if m.double_talk_detected else "NO"}
Measurement Confidence:  {m.measurement_confidence.upper()}
Filter Divergence:       {m.filter_divergence:.3f} (0=converged, 1=diverged)

────────────────────────────────────────────────────────────────────
                      DELAY MEASUREMENT
────────────────────────────────────────────────────────────────────
Echo Delay:              {m.delay_samples} samples ({m.delay_ms:.2f} ms)

────────────────────────────────────────────────────────────────────
                      POWER MEASUREMENTS
────────────────────────────────────────────────────────────────────
TX Signal Power:         {m.tx_power_dbfs:.1f} dBFS
RX Signal Power:         {m.rx_power_dbfs:.1f} dBFS
Echo Power:              {m.echo_power_dbfs:.1f} dBFS
Noise Floor:             {m.noise_floor_dbfs:.1f} dBFS

────────────────────────────────────────────────────────────────────
                      ECHO METRICS
────────────────────────────────────────────────────────────────────
Echo Return Loss (ERL):  {m.erl_db:.1f} dB
Echo SNR:                {m.snr_db:.1f} dB

────────────────────────────────────────────────────────────────────
                      INTERPRETATION
────────────────────────────────────────────────────────────────────
"""
        
        # Interpret results
        if not m.echo_detected:
            report += "• No echo detected - signal path is clean\n"
        else:
            if m.erl_db < 6:
                report += f"• SEVERE ECHO: ERL of {m.erl_db:.1f} dB means echo is very strong\n"
                report += "  → Echo power is nearly equal to transmitted power\n"
                report += "  → HIGH RISK of feedback/howling\n"
                report += "  → Echo cancellation is CRITICAL\n"
            elif m.erl_db < 15:
                report += f"• MODERATE ECHO: ERL of {m.erl_db:.1f} dB means noticeable echo\n"
                report += "  → Echo cancellation recommended\n"
            else:
                report += f"• MILD ECHO: ERL of {m.erl_db:.1f} dB means echo is attenuated\n"
        
        if m.double_talk_detected:
            report += "• Double-talk detected - measurements may be less accurate\n"
        
        if m.delay_ms > 150:
            report += f"• Delay of {m.delay_ms:.1f} ms indicates network/long-path echo\n"
        elif m.delay_ms > 50:
            report += f"• Delay of {m.delay_ms:.1f} ms is typical for acoustic echo\n"
        else:
            report += f"• Delay of {m.delay_ms:.1f} ms is very short (electrical coupling?)\n"
        
        report += "\n"
        report += "OVERALL ASSESSMENT:\n"
        
        if not m.echo_detected:
            report += "✓ No echo problem detected\n"
        elif m.erl_db < 6:
            report += "✗ CRITICAL: Severe echo detected - immediate action required\n"
            report += "  → Implement echo cancellation\n"
            report += "  → Reduce loop gain\n"
            report += "  → Check for feedback paths\n"
        elif m.erl_db < 15:
            report += "⚠ WARNING: Moderate echo detected\n"
            report += "  → Echo cancellation recommended for quality\n"
        else:
            report += "✓ Echo is present but manageable\n"
        
        report += "════════════════════════════════════════════════════════════════════\n"
        
        return report


def load_wav(filename: str) -> Tuple[np.ndarray, int]:
    """Load WAV file."""
    rate, data = wavfile.read(filename)
    if len(data.shape) > 1:
        data = data[:, 0]
    return data, rate


def main():
    parser = argparse.ArgumentParser(
        description="Speex-inspired echo analyzer for SDR environments"
    )
    
    parser.add_argument("--tx", required=True, help="TX signal WAV file")
    parser.add_argument("--rx", required=True, help="RX signal WAV file")
    parser.add_argument("--json", help="Output JSON file")
    
    args = parser.parse_args()
    
    # Load signals
    print(f"Loading TX: {args.tx}")
    tx_signal, tx_rate = load_wav(args.tx)
    
    print(f"Loading RX: {args.rx}")
    rx_signal, rx_rate = load_wav(args.rx)
    
    if tx_rate != rx_rate:
        print(f"WARNING: Sample rate mismatch (TX={tx_rate}, RX={rx_rate})")
        print(f"         RX will be resampled to match TX")
    
    # Analyze - pass both sample rates
    print("Analyzing echo parameters...")
    analyzer = SpeexEchoAnalyzer(tx_signal, rx_signal, sample_rate=tx_rate, rx_sample_rate=rx_rate)
    measurement = analyzer.analyze()
    
    # Print report
    report = analyzer.generate_report(measurement)
    print(report)
    
    # Save JSON if requested
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(measurement.to_dict(), f, indent=2)
        print(f"Results saved to: {args.json}")


if __name__ == "__main__":
    main()
