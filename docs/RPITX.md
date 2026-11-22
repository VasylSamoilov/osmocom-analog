# rpitx: Raspberry Pi GPIO as transmitter

`--sdr-rpitx` (or `--sdr-tx-rpitx` in split mode) sends the TX signal of any
osmocom-analog application on GPIO4 (pin 7) of a Raspberry Pi, using
[librpitx](https://github.com/F5OEO/librpitx). TX only.

rpitx is not an SDR: it moves the frequency of one square wave once per
sample and can key it on and off. The backend (`src/libsdr/rpitx.c`) takes
the phase of the IQ signal and sends its frequency, with the carrier on at a
fixed drive level while the signal is there.

- One FM channel: exact.
- Two FM channels (experimental, NMT tested with a phone, AMPS not yet): both
  are sent on the one carrier. On a Pi 4 off air both reach about 25 dB SINAD
  (test tones, 3 kHz deviation), a POCSAG page on the weaker one decodes, and
  an NMT phone calls on the weaker one (traffic channel next to the calling
  channel).
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
| `--sdr-tx-ppm` | crystal correction on top of librpitx's own (it takes the NTP frequency correction when NTP is synced); without NTP needed: ±50 ppm is 23 kHz at 460 MHz |
| `--sdr-rpitx-ratio` | level of the second channel, below 1 |

Run as root (DMA, PLL and GPIO through `/dev/mem` and the mailbox).
Filter the output: it is a square wave with strong odd harmonics.

The DMA does not stop with the process, so the backend's chain ends after
the last written sample in a terminator: a block that switches GPIO4 to input
and has no next block. Whatever happens to the program (exit, crash,
`kill -9`, a pipe that takes it down), the DMA plays out what was queued (at
most the 80 ms ring), switches the carrier off and stops. Tested on a Pi 3:
0.2 s after `kill -9` the DMA was stopped and the pin an input. If the
program is only late, the same gives a gap without carrier, never old
signal; then the transmission starts again. `pinctrl get 4` (Raspberry Pi
OS 12 and newer) shows the pin: `ip` = off, `a0` = carrier on.

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
The same happens with sudo 1.9.16 (Raspberry Pi OS 13), which runs the
command in a pseudo terminal of its own: `sudo nmt ... | tee log` stops nmt.
Put the pipe inside: `sudo sh -c "nmt ... 2>&1 | tee log"`, or use
`--headless`. The pin stays off in that case (nothing is sent), but the
stopped process only goes with `kill -9`.

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
| NMT-450 with a phone, `-k 1 -Y ua,1`, RX by RTL-SDR (`--sdr-tx-rpitx --sdr-rx-soapy`, TX 192000, RX 960000 S/s) | roaming update, call to the echo test, supervisory signal quality 99 %, 0 underruns |
| NMT-450, two channels `-k 1 -k 2 -T CC -T TC` (463.000 + 463.025 MHz) | the phone's call went on the second (weaker) channel: identity, dialling, supervisory signal 89..97 % quality, peak deviation 4.5 kHz, 0 underruns |
| PLL follows the frequency samples | flat from 500 Hz to 40 kHz modulation, deviation up to 90 kHz, steps of 33 kHz per sample |
| carrier offset | Pi and RTL-SDR crystals together about +2 ppm |

The DMA chain matters. librpitx's iqdmasync (frequency + keying) writes 4
control blocks per sample, and the moment of each frequency update jitters:
a broadband floor that grows with the frequency steps. With it, one channel
reached 21..27 dB SINAD and two channels 17 / 8 dB, and a page on the second
channel did not decode. The backend uses its own chain of 2 blocks per
sample (as ngfmdmasync, which cannot key), keying only on the sample where
the carrier switches; see src/libsdr/rpitx_dma.cpp. The test tool behind
these numbers (twoch, SINAD per channel) is not part of the tree.

`rtl_sdr -g 0` is automatic gain, not the lowest gain: use `-g 0.9` next to the Pi.

### Seen by a USRP B210 (2026-11-21)

NMT two channels (`-k 1 -k 2 -T CC -T TC`, both idle, 463.000 + 463.025 MHz,
drive level 0, bare GPIO4 wire), 200 s, 0 underruns. B210 on the same table
(RX2, its own antenna), each capture compared with one with the Pi off. The Pi
at level 0 is weak there: the B210's floor is only about 24 dB below the
signal in a 16 kHz channel, so everything below that is "not seen", not
"absent".

| what | result |
|---|---|
| levels (16 kHz, relative to all within ±100 kHz) | first channel -1.4, second -8.3, product at 462.975 -12.7, 463.050 -19.5 dBc (predicted -0.9 / -10.0 / -11.8) |
| carrier frequency | about +2 kHz (4 ppm) against the B210's clock (TCXO, ±2 ppm); the RTL-SDR saw +2 ppm. The phone did not mind |
| floor beside the channels, ±60 kHz .. ±800 kHz | same as with the Pi off (below the B210's floor) |
| uplink channels 453.000 / 453.025 | same as with the Pi off: no desense seen at this distance |
| ±20 MHz around the carrier | two narrow unmodulated lines at 474.62 and 477.99 MHz, about -19 dBc, only while rpitx runs; not carrier sidebands (no modulation on them), source not known |
| test tones, SINAD, same transmission received by the Pi's RTL-SDR and the B210 at once (both tuned 112.5 kHz below the carrier, not between the channels: an IQ image of the first channel lands on the second) | two channels: RTL 25.1 / 23.5, B210 24.8 / 22.5 dB (drive level 7: 25.2 / 24.3 and 24.9 / 23.7): the limit is the transmitter. One channel: RTL 39..42, B210 34 dB (weaker signal there) |
| harmonics 926 / 1389 / 2315 MHz | all there, the square wave has even ones too; levels not usable (antennas and B210 not flat over the band, GSM-900 at 926 MHz) |

Filter the output before any antenna, and check the 474..478 MHz lines with
the Pi closer to the B210 or on a cable with an attenuator.

## Bands on a Pi 3 (2026-11-22)

Raspberry Pi 3 Model B, Raspberry Pi OS 13 32 bit, librpitx f01bdb6, set up as
in `PI_SETUP.md`. Test tones (`twoch -J`, the backend's DMA chain, 192 kS/s,
first channel 1000 Hz, second 1700 Hz at ratio 0.6), RTL-SDR next to the Pi.
SINAD in dB; "two" is first / second channel.

| system | carrier MHz | spacing | deviation | one | two |
|---|---|---|---|---|---|
| NMT-450 | 463.0125 | 25 kHz | 3 kHz | 43.9 | 42.1 / 35.7 |
| NMT-450 Thailand/Indonesia | 490.0125 | 20 kHz | 2.4 kHz | 43.9 | 41.4 / 34.7 |
| NMT-F (France, base at 440) | 442.0125 | 25 kHz | 3 kHz | 44.8 | 42.4 / 35.6 |
| JTACS/NTACS | 865.00625 | 12.5 kHz | 2 kHz | 34.6 | 35.0 / 24.6 |
| AMPS | 880.035 | 30 kHz | 3 kHz | 38.6 (41.0 with RTL gain 19.7) | 37.3 / 26.8 |
| ETACS | 925.0125 | 25 kHz | 3 kHz | 39.0 | 37.4 / 26.5 |
| TACS | 942.5125 | 25 kHz | 3 kHz | 39.2 | 38.4 / 26.7 |
| NMT-900 | 947.5125 | 25 kHz | 3 kHz | 39.1 | 38.2 / 26.7 |
| NMT-900, top | 959.0125 | 25 kHz | 3 kHz | 37.8 | 37.1 / 26.5 |

- Below 750 MHz the Pi 3 sends two channels much cleaner than the Pi 4
  (second channel 35 dB against 24 dB on the Pi 4 at 463 MHz).
- Above 750 MHz (PLL at twice the carrier) the second channel stays near
  27 dB whatever the RTL gain (0.9 .. 29.7 dB: 26.7 .. 27.6), so it is the
  transmitter. That is about the quality NMT ran with on the Pi 4.
- JTACS: 12.5 kHz spacing and the 8 kHz wide SINAD filter overlap, the
  second channel figure is pessimistic.
- A B210 a few metres away could not check this: the Pi 3's signal there was
  2 dB (463 MHz) and 33 dB (880 MHz) over its floor.

NMT-450 with two phones on the Pi 3 (`-k 90 -k 91 -T CC -T TC -U 1`,
465.225 / 465.250 MHz, drive level 7, RTL-SDR as receiver): both phones show
the clock, both call the echo test on the second (traffic) channel,
supervisory signal 80..99 % quality, about 1 core of 4 busy. Open: during
calls the transmitter stalled three times for about 60 ms (10 underruns,
10 ms of old samples); one phone keeps sending frames on the calling channel
that do not decode (about 70 % quality, against 87 % on the traffic channel)
and its calls start late.

CPU and underruns on the Pi 3. The stalls during calls came from nmt's main
thread at 100 % of a core: the DSP (FM modulation with sin/cos, filters, FM
demodulation with atan2) runs at the DSP rate for every channel, plus about
14 % for the terminal output. With librpitx's closed ring every short delay
then became a whole ring of old signal (fixed: the chain now ends at the last
written sample, a delay holds the carrier). Main thread, two channels:

| options | idle | in a phone-to-phone call (both channels) |
|---|---|---|
| `-s 192000` | 99.9 % (underruns in calls) | |
| `-s 192000 --headless` | 86 % | |
| `-s 192000 --headless --fast-math` | 62 % | |
| `-s 96000 --headless --fast-math` (rpitx still 192 kS/s) | 37 % | 56 %, no late writes |

On a Pi 3 use `-s 96000 --fast-math --headless --sdr-tx-samplerate 192000`.
Not checked yet: the images the 96 -> 192 kS/s step may leave in the
spectrum. The RTL-SDR gain must be fixed (`--sdr-rx-gain`, automatic gain is
switched off then): next to the Pi's wire at drive level 7 our own signal
raised the uplink floor by 27 dB, a metre away by 12 dB.

AMPS on the Pi 3 (2026-11-22, two Motorola phones, SID 54, system B): with
the control channel on 334, drive level 7, both phones registered (RECC
98 % level, 99..100 % quality, all CRCs good) and a call reached the
network. On 353 or 354 the phones showed "No Service", from the Pi and from
a USRP alike. The cause was not the transmitter: the fork broadcast N-1 = 0
(the number of started control channels), and a phone takes only the first
N dedicated control channels from 334 (B) / 333 (A) as paging channels.
Fixed: N-1 = CMAX-1 = 20 again, as upstream. Not tried yet: the call on a
voice channel (`-k 354 -k 355 -T CC/PC -T VC`). `--headless` cannot be
combined with the echo test (`-e`): headless means the Osmo-CC socket.

Channels lost to the PLL steps (computed with the same rules as the
backend; 1 channel at 48 kS/s, 2 adjacent channels at 192 kS/s):

| system | Pi 3 | Pi 4 |
|---|---|---|
| NMT-450 (all plans), NMT-450 Thailand | none | none |
| NMT-F 440..444.5 | 1 / 8 of 178 (441.6 MHz) | none |
| NMT-900 935..960 | 1 / 3 (960.0) | 2 / 7 (945.0) |
| AMPS 869..894 | 1 / 6 (883.2) | 1 / 6 (891.0) |
| TACS 935..950 | none | 2 / 7 (945.0) |
| ETACS ext. 917..935 | 2 / 7 (921.6) | 2 / 7 (918.0) |
| JTACS/NTACS 860..870 | none | 3 / 16 (864.0) |

(one channel / two channels refused.) Everything else in these bands is
possible on both boards.

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
5. NMT / AMPS with a combined channel, RX by an RTL-SDR:
   ```sh
   sudo src/nmt/nmt -k 1 -Y ua,1 -e -s 192000 --sdr-tx-rpitx --sdr-tx-samplerate 192000 \
        --sdr-rx-soapy --sdr-rx-device-args driver=rtlsdr --sdr-rx-samplerate 960000 \
        --sdr-rx-gain 30 --sdr-tx-gain 0
   ```
   librpitx corrects the Pi crystal from NTP by itself (when NTP is synced),
   so `--sdr-tx-ppm` is usually not needed.
6. Two channels: `-k 1 -k 2 -T CC -T TC`, calling channel first.
