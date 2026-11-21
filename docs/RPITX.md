# rpitx: Raspberry Pi GPIO as transmitter

`--sdr-rpitx` (or `--sdr-tx-rpitx` in split mode) sends the TX signal of any
osmocom-analog application on GPIO4 (pin 7) of a Raspberry Pi, using
[librpitx](https://github.com/F5OEO/librpitx). TX only.

rpitx is not an SDR: it moves the frequency of one square wave once per
sample and can key it on and off. The backend (`src/libsdr/rpitx.c`) takes
the phase of the IQ signal and sends its frequency, with the carrier on at a
fixed drive level while the signal is there.

- One FM channel: exact.
- Two FM channels (experimental, not yet tried with NMT or AMPS): both are
  sent on the one carrier. On a Pi 4 off air both reach about 25 dB SINAD
  (test tones, 3 kHz deviation), and a POCSAG page on the weaker one decodes.
  The second channel is at
  `--sdr-rpitx-ratio` (default 0.6, range 0.2 .. 0.8) of the first one's
  level. With 0.6 the first channel is at -0.9 dB, the second at -10 dB, a
  mixing product at -12 dB sits one spacing on the other side of the first
  channel (keep that channel unused), more products further out. Channels can
  be adjacent (25 or 30 kHz).
- Give first the channel that stays on (NMT/AMPS control channel), second the
  one that keys on and off (traffic/voice channel, paging channel). When the
  first channel keys off, the second one gets a glitch of about 1 ms; the
  other way round there is none.
- AM channels only get the carrier keyed on and off.
- More than two channels: refused.

## Options

| option | rpitx meaning |
|---|---|
| `--sdr-tx-gain 0..7` | GPIO drive level, 2..16 mA (default 0, lowest). Measured with librpitx's iqdmasync (relative to level 7): -25.6, -14.6, -10.1, -8.1, -3.3, -2.2, -0.8, 0 dB; the backend's own chain sets the pad differently (about 18 dB more at level 0), not measured per level yet |
| `--sdr-samplerate` | 10000 .. 200000 S/s; up to 250000 without carrier keying |
| `--sdr-tx-ppm` | crystal correction, needed: ±50 ppm is 23 kHz at 460 MHz |
| `--sdr-rpitx-ratio` | level of the second channel, below 1 |

Run as root (DMA, PLL and GPIO through `/dev/mem` and the mailbox).
Filter the output: it is a square wave with strong odd harmonics.

The DMA does not stop with the process. The backend stops it on exit, on
crashes and on the usual signals, but `kill -9` leaves the carrier on (run
any rpitx tool, e.g. `tune`, and stop it with Ctrl-C to switch it off).

librpitx moves only the fraction of the PLL multiplier. Its integer steps are
every 38.4 MHz above 750 MHz on the Pi 3 and every 27 MHz above 375 MHz on
the Pi 4. Within +-samplerate/2 of a step the carrier would jump, so the
backend refuses: e.g. 883.2 MHz on the Pi 3 (AMPS channels around 440). It
also refuses carriers the PLL cannot make: on the Pi 4 above 1 GHz, and
settings that run the PLL below 600 MHz, which freeze the board (librpitx
allows them).

The pin stays off until the first samples reach the DMA, so a process that
stalls while starting does not leave a carrier on air.

With `timeout` in a terminal use `timeout --foreground`: plain `timeout` runs
the program in a background process group, the program is stopped (state `T`)
when it sets up the terminal for its console, and it never feeds the DMA.

## Measured on a Pi 4 (2026-11-21)

Raspberry Pi 4 Model B, Raspberry Pi OS 11 64 bit, librpitx f01bdb6,
`gpu_freq=250` and `force_turbo=1` in `/boot/config.txt`, RTL-SDR next to it.

| test | result |
|---|---|
| POCSAG 1200, one page, 96 kS/s, drive level 0 | decoded (multimon-ng) |
| POCSAG `--network` 70 s, pages from `--fifo-file` | all pages decoded, 96000 S/s measured, FIFO fill 21 ms, 0 underruns, eye 17 dB |
| two channels 434.000 + 434.025, 192 kS/s | 192027 S/s, 0 underruns, CPU 96 % idle; eye 21 dB (first) and 19 dB (second); a page on the second channel decodes |
| test tones (1000 / 1700 Hz, 3 kHz deviation), SINAD | one channel 44 dB; two channels 25 / 24 dB (simulation 44 / 45 dB) |
| levels of the two channels and the product | -1.2 / -11.4 / -12.6 dBc, predicted -0.9 / -10.0 / -11.8 |

The DMA chain matters. librpitx's iqdmasync (frequency + keying) writes 4
control blocks per sample, and the moment of each frequency update jitters:
a broadband floor that grows with the frequency steps. With it, one channel
reached 21..27 dB SINAD and two channels 17 / 8 dB, and a page on the second
channel did not decode. The backend uses its own chain of 2 blocks per
sample (as ngfmdmasync, which cannot key), keying only on the sample where
the carrier switches; see src/libsdr/rpitx_dma.cpp. The test tool behind
these numbers (twoch, SINAD per channel) is not part of the tree.
| PLL follows the frequency samples | flat from 500 Hz to 40 kHz modulation, deviation up to 90 kHz, steps of 33 kHz per sample |
| carrier offset | Pi and RTL-SDR crystals together about +2 ppm |

`rtl_sdr -g 0` is automatic gain, not the lowest gain: use `-g 0.9` next to the Pi.

## Build on the Pi

```sh
git clone https://github.com/F5OEO/rpitx && cd rpitx && ./install.sh   # accept the config.txt change, reboot
git clone https://github.com/F5OEO/librpitx && cd librpitx/src && make && sudo make install
cd osmocom-analog && autoreconf -fi && ./configure && make
```

`configure` must print `Compiling with rpitx support`.

## Test ladder

Take an RTL-SDR IQ capture of each step (`rtl_sdr -f <Hz> -s 1024000 -n 20000000 cap.cu8`).

1. rpitx itself: `sudo ./testnfm.sh` or `sudo pifmrds -freq 107.9`.
2. POCSAG, one channel, one page:
   ```sh
   sudo src/pocsag/pocsag -k 466.230 --sdr-rpitx --sdr-samplerate 96000 \
        --sdr-tx-gain 0 --sdr-tx-ppm <ppm> -B 1200 --oneshot -y alpha -M "HELLO" 1234567
   ```
3. POCSAG continuous (`--network`) for some minutes; read the pacing lines:
   ```
   rpitx: 96000 S/s (nominal 96000), lowest FIFO fill 21.0 ms, longest write 3.0 ms, underruns 0 (0.0 ms), total underruns 0
   ```
   Underruns mean the DMA sent old samples again: the Pi did not keep up.
4. Two channels: `-k 466.230 -k 466.255 ... --sdr-samplerate 192000`.
5. NMT / AMPS with a combined channel, RX by an RTL-SDR (`--sdr-tx-rpitx --sdr-rx-soapy`).
