#!/usr/bin/env python3
"""
SIP Echo Tester using PJSIP (pjsua2)

A SIP answering machine that:
- Registers with a SIP server
- Answers incoming calls
- Plays the test signal
- Records audio
- Analyzes echo after hangup

Requirements:
- pjsua2 Python bindings
- numpy, scipy for analysis

Usage:
  python sip_echo_tester_pjsip.py --server 127.0.0.1 --user 615 --password pwd005
"""

import pjsua2 as pj
import time
import os
import sys
import argparse
import wave
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class EchoTestCall(pj.Call):
    """Handle call events and audio."""
    
    def __init__(self, acc, tester, call_id=pj.PJSUA_INVALID_ID):
        pj.Call.__init__(self, acc, call_id)
        self.tester = tester
        self.recorder = None
        self.player = None
        self.rec_path = None
        self.connected = False
    
    def onCallState(self, prm):
        ci = self.getInfo()
        print(f"Call state: {ci.stateText}")
        
        if ci.state == pj.PJSIP_INV_STATE_CONFIRMED:
            self.connected = True
            self.start_test()
        elif ci.state == pj.PJSIP_INV_STATE_DISCONNECTED:
            self.on_disconnected()
    
    def onCallMediaState(self, prm):
        ci = self.getInfo()
        for mi in ci.media:
            if mi.type == pj.PJMEDIA_TYPE_AUDIO and mi.status == pj.PJSUA_CALL_MEDIA_ACTIVE:
                print("Audio media active")
    
    def start_test(self):
        """Start playing test signal and recording."""
        print("Starting echo test...")
        
        ci = self.getInfo()
        
        # Find audio media
        for i, mi in enumerate(ci.media):
            if mi.type == pj.PJMEDIA_TYPE_AUDIO and mi.status == pj.PJSUA_CALL_MEDIA_ACTIVE:
                aud_med = self.getAudioMedia(i)
                
                # Start recording
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.rec_path = self.tester.output_dir / f"recording_{timestamp}.wav"
                
                try:
                    self.recorder = pj.AudioMediaRecorder()
                    self.recorder.createRecorder(str(self.rec_path))
                    aud_med.startTransmit(self.recorder)
                    print(f"Recording to: {self.rec_path}")
                except Exception as e:
                    print(f"Recorder error: {e}")
                
                # Start playing test signal
                try:
                    self.player = pj.AudioMediaPlayer()
                    self.player.createPlayer(str(self.tester.test_signal_path), pj.PJMEDIA_FILE_NO_LOOP)
                    self.player.startTransmit(aud_med)
                    print(f"Playing: {self.tester.test_signal_path}")
                except Exception as e:
                    print(f"Player error: {e}")
                
                break
        
        # Schedule hangup
        duration = self.tester.test_signal_duration + 1.0
        self.tester.schedule_hangup(self, duration)
    
    def on_disconnected(self):
        """Clean up and analyze."""
        self.connected = False
        
        # Stop recorder
        if self.recorder:
            try:
                del self.recorder
                self.recorder = None
            except:
                pass
        
        # Stop player
        if self.player:
            try:
                del self.player
                self.player = None
            except:
                pass
        
        # Analyze recording
        time.sleep(0.3)
        if self.rec_path and self.rec_path.exists():
            self.tester.analyze_recording(self.rec_path)
        
        print("\nWaiting for next call...\n")


class EchoTestAccount(pj.Account):
    """Handle account events."""
    
    def __init__(self, tester):
        pj.Account.__init__(self)
        self.tester = tester
        self.current_call = None
    
    def onRegState(self, prm):
        ai = self.getInfo()
        print(f"Registration: {ai.regStatusText} ({ai.regStatus})")
    
    def onIncomingCall(self, prm):
        call = EchoTestCall(self, self.tester, prm.callId)
        ci = call.getInfo()
        print(f"\n{'='*60}")
        print(f"Incoming call from: {ci.remoteUri}")
        print(f"{'='*60}")
        
        self.current_call = call
        
        # Answer
        call_prm = pj.CallOpParam()
        call_prm.statusCode = 200
        call.answer(call_prm)


class EchoTester:
    """Main echo tester."""
    
    def __init__(self, server, port, local_port, user, password, test_signal_path, output_dir):
        self.server = server
        self.port = port
        self.local_port = local_port
        self.user = user
        self.password = password
        self.test_signal_path = Path(test_signal_path)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Get test signal duration
        with wave.open(str(self.test_signal_path), 'r') as w:
            self.test_signal_duration = w.getnframes() / w.getframerate()
        
        print(f"Test signal: {self.test_signal_path} ({self.test_signal_duration:.2f}s)")
        
        self.ep = None
        self.account = None
        self.pending_hangups = []
    
    def schedule_hangup(self, call, delay):
        """Schedule a call to be hung up after delay."""
        import threading
        def do_hangup():
            try:
                # Register this thread with PJSIP
                self.ep.libRegisterThread("hangup_thread")
                if call.connected:
                    print("Hanging up...")
                    call_prm = pj.CallOpParam()
                    call.hangup(call_prm)
            except Exception as e:
                print(f"Hangup error: {e}")
        
        t = threading.Timer(delay, do_hangup)
        t.start()
        self.pending_hangups.append(t)
    
    def analyze_recording(self, rec_path):
        """Analyze the recording for echo."""
        try:
            from echo_analyzer_speex import SpeexEchoAnalyzer
            import numpy as np
            from scipy.io import wavfile
            from scipy import signal as sig
            
            rate, data = wavfile.read(str(rec_path))
            
            # Load TX from test signal
            tx_rate, tx_data = wavfile.read(str(self.test_signal_path))
            
            # Don't resample here - let the analyzer handle it
            tx = tx_data.astype(np.float32) / 32768
            
            if len(data.shape) == 2:
                rx = data[:, 0].astype(np.float32) / 32768
            else:
                rx = data.astype(np.float32) / 32768
            
            # Pass both sample rates to analyzer - it will resample RX to match TX
            analyzer = SpeexEchoAnalyzer(tx, rx, sample_rate=tx_rate, rx_sample_rate=rate)
            measurement = analyzer.analyze()
            report = analyzer.generate_report(measurement)
            print(report)
            
            # Save report
            report_path = rec_path.with_suffix('.txt')
            with open(report_path, 'w') as f:
                f.write(report)
            print(f"Report saved: {report_path}")
            
            # Save JSON
            import json
            json_path = rec_path.with_suffix('.json')
            with open(json_path, 'w') as f:
                json.dump(measurement.to_dict(), f, indent=2)
            print(f"JSON saved: {json_path}")
            
        except Exception as e:
            print(f"Analysis error: {e}")
            import traceback
            traceback.print_exc()
    
    def start(self):
        """Start the echo tester."""
        print(f"\n{'='*60}")
        print(f"  PJSIP Echo Tester (pjsua2)")
        print(f"{'='*60}")
        print(f"  Server: {self.server}:{self.port}")
        print(f"  User:   {self.user}")
        print(f"  Local:  port {self.local_port}")
        print(f"{'='*60}\n")
        
        # Create endpoint
        self.ep = pj.Endpoint()
        self.ep.libCreate()
        
        # Configure
        ep_cfg = pj.EpConfig()
        ep_cfg.logConfig.level = 3
        ep_cfg.logConfig.consoleLevel = 2
        
        # Disable audio device if not needed (avoids JACK errors)
        ep_cfg.medConfig.noVad = True
        
        self.ep.libInit(ep_cfg)
        
        # List available codecs and prioritize PCMU
        print("Available codecs:")
        try:
            codec_list = self.ep.codecEnum2()
            for c in codec_list:
                print(f"  {c.codecId} (priority {c.priority})")
                # Prioritize PCMU for compatibility
                if "PCMU" in c.codecId:
                    try:
                        self.ep.codecSetPriority(c.codecId, 255)
                        print(f"    -> Prioritized!")
                    except:
                        pass
        except Exception as e:
            print(f"Codec enumeration: {e}")       
        # Create transport on specified local port
        sip_tp_cfg = pj.TransportConfig()
        sip_tp_cfg.port = self.local_port
        self.ep.transportCreate(pj.PJSIP_TRANSPORT_UDP, sip_tp_cfg)
        
        # Start
        self.ep.libStart()
        
        # Create account
        acc_cfg = pj.AccountConfig()
        acc_cfg.idUri = f"sip:{self.user}@{self.server}"
        acc_cfg.regConfig.registrarUri = f"sip:{self.server}:{self.port}"
        
        cred = pj.AuthCredInfo()
        cred.scheme = "digest"
        cred.realm = "*"
        cred.username = self.user
        cred.dataType = 0
        cred.data = self.password
        acc_cfg.sipConfig.authCreds.append(cred)
        
        self.account = EchoTestAccount(self)
        self.account.create(acc_cfg)
        
        print("Registered! Waiting for calls...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()
    
    def stop(self):
        """Clean up."""
        for t in self.pending_hangups:
            t.cancel()
        
        if self.account:
            self.account.shutdown()
        if self.ep:
            self.ep.libDestroy()


def ensure_test_signal(output_dir):
    """Generate test signal if needed."""
    signal_path = Path(output_dir) / "echo_test_signal.wav"
    if not signal_path.exists():
        print("Generating test signal...")
        from simple_test_signal import create_simple_echo_test_signal, save_wav
        signal = create_simple_echo_test_signal(sample_rate=8000, amplitude=0.5)
        save_wav(signal, str(signal_path), 8000)
        print(f"Created: {signal_path}")
    return signal_path


def main():
    parser = argparse.ArgumentParser(description="PJSIP Echo Tester")
    parser.add_argument("--server", required=True, help="SIP server")
    parser.add_argument("--port", type=int, default=5060, help="SIP server port")
    parser.add_argument("--local-port", type=int, default=5080, help="Local bind port (default: 5080)")
    parser.add_argument("--user", required=True, help="SIP user")
    parser.add_argument("--password", default="", help="SIP password")
    parser.add_argument("--test-signal", default=None, help="Test signal WAV")
    parser.add_argument("--output-dir", default="recordings", help="Output dir")
    
    args = parser.parse_args()
    
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)
    
    test_signal = Path(args.test_signal) if args.test_signal else ensure_test_signal(output_dir)
    
    if not test_signal.exists():
        print(f"Test signal not found: {test_signal}")
        return 1
    
    tester = EchoTester(
        server=args.server,
        port=args.port,
        local_port=args.local_port,
        user=args.user,
        password=args.password,
        test_signal_path=test_signal,
        output_dir=output_dir
    )
    
    tester.start()
    return 0


if __name__ == "__main__":
    sys.exit(main())
