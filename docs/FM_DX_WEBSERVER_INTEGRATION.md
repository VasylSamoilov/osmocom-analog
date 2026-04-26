# FM-DX Webserver Integration with osmoradio

This document describes how to use [FM-DX Webserver](https://github.com/NoobishSVK/fm-dx-webserver) with osmoradio as the SDR backend.

## Overview

FM-DX Webserver is a web-based interface for FM DXing that provides:
- Web-based tuner control from any browser
- Real-time RDS decoding and display
- Signal strength monitoring
- Low-latency audio streaming
- Plugin support

osmoradio implements the XDR-GTK protocol, which is the same protocol used by xdrd (the standard backend for FM-DX Webserver). This allows FM-DX Webserver to connect directly to osmoradio.

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────┐
│   Web Browser   │────▶│  FM-DX Webserver │────▶│  osmoradio  │
│   (Client)      │◀────│  (Node.js)       │◀────│  (SDR)      │
└─────────────────┘     └──────────────────┘     └─────────────┘
     WebSocket              TCP/XDR-GTK           SDR Hardware
```

## Quick Start

### 1. Start osmoradio with XDR-GTK server

```bash
# Using SoapySDR (e.g., RTL-SDR, AirSpy, LimeSDR)
osmoradio --sdr-soapy -f 98500000 -M fm -R --rds \
    --xdr-gtk-server 127.0.0.1:7373

# Using UHD (e.g., USRP)
osmoradio --sdr-uhd -f 98500000 -M fm -R --rds \
    --xdr-gtk-server 127.0.0.1:7373
```

### 2. Configure FM-DX Webserver

Edit your FM-DX Webserver configuration to connect to osmoradio:

```json
{
  "xdrd": {
    "wirelessConnection": true,
    "xdrdIp": "127.0.0.1",
    "xdrdPort": 7373,
    "xdrdPassword": ""
  }
}
```

If you set a password in osmoradio:
```bash
osmoradio ... --xdr-gtk-server 127.0.0.1:7373 --xdr-gtk-password "mypassword"
```

Then configure FM-DX Webserver with the same password:
```json
{
  "xdrd": {
    "xdrdPassword": "mypassword"
  }
}
```

### 3. Start FM-DX Webserver

```bash
cd fm-dx-webserver
npm run webserver
```

### 4. Open in Browser

Navigate to `http://localhost:8080` (or your configured port).

## Supported Features

### Tuning Control
- ✅ Frequency tuning (T command)
- ✅ Antenna selection (Z command)
- ✅ Stereo/Mono mode (B command)
- ✅ De-emphasis (D command)
- ✅ Bandwidth control (W/F commands)
- ✅ Volume control (Y command)
- ✅ AGC control (A command)
- ✅ Squelch (Q command)

### Signal Information
- ✅ Signal strength (S command)
- ✅ Stereo indicator
- ✅ Pilot detection

### RDS Data
- ✅ PI code (P command)
- ✅ RDS groups (R command)
- ✅ PS, RT, PTY, AF (via librdsparser in FM-DX Webserver)

### Advanced Features
- ✅ Spectral scan (S commands)
- ✅ Password authentication
- ✅ Multiple client support (via FM-DX Webserver)

## Command Line Options

### osmoradio XDR-GTK Server Options

| Option | Description |
|--------|-------------|
| `--xdr-gtk-server <endpoint>` | Enable XDR-GTK protocol server |
| `--xdr-gtk-password <password>` | Set authentication password |

Endpoint format:
- TCP: `<ip>:<port>` (e.g., `127.0.0.1:7373`)
- Serial: `<device>,<speed>,<8N1>[,<flow>]` (e.g., `/dev/ttyUSB0,115200,8N1`)

### Example Configurations

#### Basic FM Receiver
```bash
osmoradio --sdr-soapy -f 98500000 -M fm -R --rds \
    --xdr-gtk-server 0.0.0.0:7373
```

#### With Stereo and RDS
```bash
osmoradio --sdr-soapy -f 98500000 -M fm -R --rds --stereo \
    --xdr-gtk-server 0.0.0.0:7373 \
    --sdr-rx-gain 40
```

#### LimeSDR Configuration
```bash
osmoradio --limesdr -f 98500000 -M fm -R --rds --stereo \
    --xdr-gtk-server 0.0.0.0:7373
```

#### With Audio Output
```bash
osmoradio --sdr-soapy -f 98500000 -M fm -R --rds --stereo \
    --xdr-gtk-server 0.0.0.0:7373 \
    -w output.wav
```

## Protocol Details

osmoradio implements the XDR-GTK protocol as documented in `src/radio/rds_protocol.h`. Key messages:

### Server → Client (osmoradio → FM-DX Webserver)

| Message | Description | Example |
|---------|-------------|---------|
| `OK` | Connection acknowledgment | `OK` |
| `T<freq>` | Current frequency (kHz) | `T98500` |
| `S<mode><signal>` | Signal strength | `Ss-45.2` |
| `P<PI>[?...]` | PI code with errors | `P2201??` |
| `R<hex>` | RDS group (18 chars) | `R220100B4E201000000` |
| `G<val>` | iMS/cEQ status | `G11` |
| `Z<val>` | Antenna | `Z0` |
| `D<val>` | De-emphasis | `D0` |

### Client → Server (FM-DX Webserver → osmoradio)

| Message | Description | Example |
|---------|-------------|---------|
| `x` | Status request | `x` |
| `T<freq>` | Tune to frequency | `T98500` |
| `Z<val>` | Select antenna | `Z0` |
| `B<val>` | Stereo/mono mode | `B0` |
| `G<val>` | Set iMS/cEQ | `G11` |
| `Y<val>` | Set volume | `Y100` |

## Troubleshooting

### Connection Issues

1. **FM-DX Webserver can't connect**
   - Verify osmoradio is running with `--xdr-gtk-server`
   - Check IP and port match in both configurations
   - Ensure firewall allows the connection

2. **Authentication fails**
   - Verify password matches in both configurations
   - Password is case-sensitive
   - osmoradio uses ASCII-printable salt characters for compatibility with both XDR-GTK and FM-DX Webserver

### Authentication Protocol Notes

osmoradio implements the XDR-GTK authentication protocol:
1. Server sends 16-byte random salt + newline
2. Client computes SHA1(salt + password) and sends 40-char hex hash + newline
3. Server responds with `a1` (success) or `a0` (failure)

For compatibility with FM-DX Webserver (which has a UTF-8 encoding quirk in its auth implementation), osmoradio generates salt using only ASCII printable characters (0x21-0x7E). This ensures both XDR-GTK (raw bytes) and FM-DX Webserver (UTF-8 interpretation) compute the same hash.

### No RDS Data

1. **Verify RDS is enabled**
   ```bash
   osmoradio ... --rds
   ```

2. **Check signal strength**
   - RDS requires adequate signal strength
   - Try a stronger station

3. **Verify frequency**
   - Ensure you're tuned to an FM broadcast station with RDS

### Audio Issues

FM-DX Webserver handles audio streaming separately. osmoradio provides the demodulated audio which FM-DX Webserver can stream via its built-in audio server.

## See Also

- [FM-DX Webserver Documentation](https://github.com/NoobishSVK/fm-dx-webserver/wiki)
- [XDR-GTK Protocol](src/radio/rds_protocol.h)
- [osmoradio RDS Implementation](src/radio/rds.h)
