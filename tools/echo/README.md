# Echo Test Tools

Tools for measuring echo parameters in SDR radio audio paths via SIP.

## Quick Start

```bash
# Install dependencies
pip install numpy scipy pjsua2

# Run echo tester (connects to SIP server, answers calls, measures echo)
python sip_echo_tester.py --server 127.0.0.1 --user 615 --password pwd005
```

## Tools

| File | Description |
|------|-------------|
| `sip_echo_tester.py` | SIP answering machine - answers calls, plays test signal, records, analyzes |
| `echo_test_signal.py` | Generates MLS + chirp test signal for echo measurement |
| `echo_analyzer_speex.py` | Speex-inspired analyzer for noisy SDR environments |
| `simple_echo_analyzer.py` | Basic power-based analyzer |

## Usage

### SIP Echo Tester

Registers with a SIP server, answers incoming calls, plays test signal, records audio, and analyzes echo:

```bash
python sip_echo_tester.py --server <sip_server> --user <extension> --password <pwd>
```

Results are saved to `recordings/` with timestamp.

### Generate Test Signal

```bash
python echo_test_signal.py -o test.wav           # 8kHz default
python echo_test_signal.py -o test.wav -r 48000  # 48kHz
```

### Analyze Recordings

```bash
python echo_analyzer_speex.py --tx transmitted.wav --rx recorded.wav
python echo_analyzer_speex.py --tx test.wav --rx recording.wav --json results.json
```

## Output Metrics

- **Echo Delay**: samples and milliseconds
- **ERL**: Echo Return Loss (dB)
- **TX/RX Power**: dBFS levels
- **Correlation confidence**: measurement quality indicator
