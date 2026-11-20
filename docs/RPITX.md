# rpitx: Raspberry Pi GPIO as transmitter

`--sdr-rpitx` (or `--sdr-tx-rpitx` in split mode) sends the TX signal of any
osmocom-analog application on GPIO4 (pin 7) of a Raspberry Pi, using
[librpitx](https://github.com/F5OEO/librpitx). TX only.

rpitx is not an SDR: it moves the frequency of one square wave once per
sample and can key it on and off. The backend (`src/libsdr/rpitx.c`) takes
the phase of the IQ signal and sends its frequency, with the carrier on at a
fixed drive level while the signal is there.

- One FM channel: exact.
- Two FM channels: both are sent on the one carrier. The second channel is at
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
| `--sdr-tx-gain 0..7` | GPIO drive level, 2..16 mA (default 0, lowest) |
| `--sdr-samplerate` | 10000 .. 200000 S/s; up to 250000 without carrier keying |
| `--sdr-tx-ppm` | crystal correction, needed: ±50 ppm is 23 kHz at 460 MHz |
| `--sdr-rpitx-ratio` | level of the second channel, below 1 |

Run as root (DMA, PLL and GPIO through `/dev/mem` and the mailbox).
Filter the output: it is a square wave with strong odd harmonics.

The DMA does not stop with the process. The backend stops it on exit, on
crashes and on the usual signals, but `kill -9` leaves the carrier on (run
any rpitx tool, e.g. `tune`, and stop it with Ctrl-C to switch it off).

Above 750 MHz (Pi 3) librpitx moves only the fraction of the PLL multiplier,
which steps every 38.4 MHz. Within +-samplerate/2 of a step the carrier would
jump, so the backend refuses: e.g. 883.2 MHz (AMPS channels around 440).

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
