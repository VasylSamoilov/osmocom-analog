# osmoradio - FM Radio Receiver/Transmitter

osmoradio is an SDR-based FM broadcast radio application supporting stereo, RDS decoding, and integration with XDR-GTK.

## Quick Start

### Basic FM Reception (Mono)
```bash
osmoradio -f 103600000 --rx -M fm --sdr-soapy --sdr-rx-gain 30
```

### Stereo FM with RDS
```bash
osmoradio -f 103600000 --rx -M fm --rds --stereo --sdr-soapy --sdr-rx-gain 30 -a plughw:0,0
```

### High-Quality Reception with UHD (USRP)
```bash
osmoradio -f 103600000 --rx -M fm --rds --stereo --channelizer \
  --sdr-uhd --sdr-rx-gain 35 --sdr-rx-antenna RX2 \
  --samplerate 2000000 -a plughw:0,0
```

### With XDR-GTK Remote Control
```bash
osmoradio -f 103600000 --rx -M fm --rds --stereo --channelizer \
  --sdr-uhd --sdr-rx-gain 35 --sdr-rx-antenna RX2 \
  --xdr-gtk-server 0.0.0.0:7373 --xdr-gtk-password mypassword \
  --samplerate 2000000 -a plughw:0,0
```

## Audio Output Setup

### Using ALSA Loopback for Stereo

The ALSA loopback device defaults to mono. For stereo FM reception, configure it properly:

1. Stop PipeWire/PulseAudio:
```bash
systemctl --user stop pipewire pipewire-pulse wireplumber
```

2. Reload loopback module with stereo support:
```bash
sudo modprobe -r snd-aloop
sudo modprobe snd-aloop pcm_substreams=2 enable=1,1 index=2
```

3. Restart audio services:
```bash
systemctl --user start pipewire pipewire-pulse wireplumber
```

4. Use the loopback device:
```bash
osmoradio -f 103600000 --rx -M fm --stereo -a hw:Loopback,0,0 ...
```

### Permanent Loopback Configuration

Create `/etc/modprobe.d/snd-aloop.conf`:
```
options snd-aloop pcm_substreams=2 enable=1,1
```

Then reboot or reload the module.

### Alternative: Use plughw

If you can't reconfigure the loopback, use `plughw:` prefix for automatic format conversion:
```bash
osmoradio -f 103600000 --rx -M fm --stereo -a plughw:Loopback,0,0 ...
```

### Recording to WAV File

For testing or recording:
```bash
osmoradio -f 103600000 --rx -M fm --stereo --rds -W /tmp/recording.wav ...
```

## Key Options

| Option | Description |
|--------|-------------|
| `-f FREQ` | Frequency in Hz (e.g., 103600000 for 103.6 MHz) |
| `--rx` | Receive mode |
| `--tx` | Transmit mode |
| `-M fm` | FM modulation |
| `--stereo` | Enable stereo decoding |
| `--rds` | Enable RDS decoding |
| `--channelizer` | Use polyphase channelizer (better quality) |
| `-a DEVICE` | ALSA audio device |
| `-W FILE` | Write audio to WAV file |
| `--sdr-soapy` | Use SoapySDR backend |
| `--sdr-uhd` | Use UHD (USRP) backend |
| `--sdr-rx-gain N` | RX gain in dB |
| `--sdr-rx-antenna ANT` | RX antenna (e.g., RX2) |
| `--samplerate N` | SDR sample rate |
| `--xdr-gtk-server HOST:PORT` | Enable XDR-GTK protocol server |
| `--xdr-gtk-password PASS` | XDR-GTK authentication password |
| `-v N` | Volume adjustment |

## Debugging Audio Quality

Enable debug output to monitor signal levels through the processing chain:
```bash
osmoradio ... -d DRADIO:INFO
```

This shows periodic reports of:
- SDR raw IQ levels
- FM demodulator output
- Stereo decoder levels
- De-emphasis filter output
- Resampler statistics
- Final audio output levels

Look for:
- Clipping (samples >= 1.0)
- Discontinuities (sudden jumps)
- Resampler ratio errors
- Sample gaps

## Troubleshooting

### "Sound card only supports 1 channel"
Your audio device doesn't support stereo. Either:
- Use `plughw:` prefix instead of `hw:`
- Configure ALSA loopback for stereo (see above)
- Use a different audio device

### Buffer underruns
Increase latency or use real-time scheduling:
```bash
sudo chrt -f 50 osmoradio ...
```

### Poor stereo separation
- Check pilot lock status in debug output
- Ensure sufficient signal strength
- Try adjusting `--sdr-rx-gain`
