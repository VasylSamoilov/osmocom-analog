# Echo Test Tools

Tools for measuring echo parameters in VoIP/SDR radio audio paths.

## Quick Start

```bash
# 1. Install dependencies (use venv if needed)
pip install numpy scipy pyvoip

# 2. Generate test signal
python echo_test_signal.py -o echo_test_signal.wav

# 3. Start the SIP echo tester
python sip_echo_tester.py --bind 0.0.0.0 --port 5060

# 4. Call the tester from your SIP device
#    It will answer, play test signal, record, and analyze
```

## Components

### echo_test_signal.py
Generates a test signal optimized for echo measurement:
- **MLS (Maximum Length Sequence)** - for precise delay estimation via cross-correlation
- **Chirp sweep** - for frequency response analysis

```bash
python echo_test_signal.py -o test.wav
python echo_test_signal.py -o test.wav -r 16000  # 16kHz
python echo_test_signal.py --help
```

### echo_analyzer.py
Analyzes recorded TX/RX audio pairs to measure:
- **Echo delay** (samples and milliseconds)
- **ERL** (Echo Return Loss in dB)
- **Correlation confidence**

```bash
python echo_analyzer.py --tx transmitted.wav --rx recorded.wav
python echo_analyzer.py --tx test.wav --rx recording.wav --json results.json
```

### sip_echo_tester.py
SIP answering machine that automates the test:
1. Answers any incoming call
2. Plays the test signal
3. Records received audio
4. Analyzes and reports echo parameters

```bash
python sip_echo_tester.py --bind 0.0.0.0 --port 5060
```

## Output

The analyzer produces reports like:
```
════════════════════════════════════════════════════════════════════
                       ECHO ANALYSIS REPORT
════════════════════════════════════════════════════════════════════
Sample Rate:     8000 Hz

Echo Delay:              100 samples (12.50 ms)
Correlation Peak:        0.872 (high confidence)

TX Signal Power:         -6.2 dBFS
RX Signal Power:         -18.4 dBFS

Echo Return Loss (ERL):  12.2 dB
════════════════════════════════════════════════════════════════════
```

## Requirements

- Python 3.8+
- numpy
- scipy
- pyvoip (for SIP functionality)

## Notes

- pyVoIP supports PCMU/PCMA codecs (8kHz, 8-bit G.711)
- For best results, ensure your audio path has minimal compression
- Test signal is ~3 seconds, analysis requires the full recording
