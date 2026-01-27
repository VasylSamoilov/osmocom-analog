#!/usr/bin/env python3
"""
Echo Test Runner

Simple script to run the echo test answering machine.

Usage:
  ./run_echo_test.py --server 192.168.1.1 --user echo --password test
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pathlib import Path


def ensure_test_signal():
    """Generate test signal if it doesn't exist."""
    signal_path = Path(__file__).parent / "echo_test_signal.wav"
    if not signal_path.exists():
        print("Generating test signal...")
        from echo_test_signal import create_echo_test_signal, save_wav
        signal = create_echo_test_signal(sample_rate=8000)
        save_wav(signal, str(signal_path), 8000)
        print(f"Created: {signal_path}")
    return str(signal_path)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Echo Test Answering Machine"
    )
    parser.add_argument("--server", required=True,
                        help="SIP server address (host:port, e.g. 127.0.0.1:5060)")
    parser.add_argument("--port", type=int, default=5060,
                        help="Local SIP port (default: 5060)")
    parser.add_argument("--user", required=True,
                        help="SIP username")
    parser.add_argument("--password", default="",
                        help="SIP password")
    parser.add_argument("--output-dir", default="recordings",
                        help="Directory for recordings")
    
    args = parser.parse_args()
    
    # Parse server:port
    if ':' in args.server:
        server_host, server_port = args.server.rsplit(':', 1)
        server_port = int(server_port)
    else:
        server_host = args.server
        server_port = 5060
    
    # Ensure test signal exists
    test_signal = ensure_test_signal()
    
    # Import and run
    try:
        from sip_echo_tester import EchoTester, get_local_ip
    except ImportError as e:
        print(f"ERROR: {e}")
        print("\nInstall dependencies:")
        print("  pip install numpy scipy pjsua2 --break-system-packages")
        return 1
    
    tester = EchoTester(
        server=server_host,
        server_port=server_port,
        username=args.user,
        password=args.password,
        my_ip=get_local_ip(),
        sip_port=args.port,
        test_signal_path=test_signal,
        output_dir=args.output_dir
    )
    
    tester.start()
    return 0


if __name__ == "__main__":
    sys.exit(main())
