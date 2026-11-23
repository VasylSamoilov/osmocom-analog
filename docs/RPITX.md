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
| `--sdr-tx-gain` | not used: the GPIO drive level is always the maximum (7, 16 mA), the most useful signal of a tiny transmitter. Measured with librpitx's iqdmasync (relative to level 7): -25.6, -14.6, -10.1, -8.1, -3.3, -2.2, -0.8, 0 dB; two channels at a B210: level 0 22 / 15 dB SINAD, level 7 24 / 21 dB |
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
supervisory signal 80..99 % quality, about 1 core of 4 busy. The stalls
during calls are explained and fixed below. While a phone sent on the
traffic channel 91, the receiver of the calling channel 90 next to it
decoded garbage frames (about 70 % quality): the phone's own signal leaking
into the neighbouring uplink, not a phone fault. Keep one channel free
between the two, as for AMPS below.

CPU and underruns on the Pi 3. The stalls during calls came from nmt's main
thread at 100 % of a core: the DSP (FM modulation with sin/cos, filters, FM
demodulation with atan2) runs at the DSP rate for every channel, plus about
14 % for the terminal output. With librpitx's closed ring every short delay
then became a whole ring of old signal (fixed: the chain now ends in a
terminator, a delay gives a gap without carrier). Main thread, two channels:

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
Fixed: N-1 = CMAX-1 = 20 again, as upstream. Then two channels on the one
carrier, `-k 354 -T CC/PC -k 355 -T VC -S sid=54 -e -s 96000 --fast-math`
(880.620 + 880.650 MHz): both phones found the network, a call went to the
voice channel 355 (the weaker, keyed one), SAT quality 99, signaling tone
100, "wonderful sound quality", 0 late writes.

AMPS phone to phone on the Pi 3 (`-k 354 -T CC/PC/VC -k 356 -T VC -x`): the
caller on 356, the called phone paged on 354 and talking there, answered,
audio both ways, released normally. With the channels next to each other
(354 + 355, 30 kHz) the call dropped when the called phone answered: its
signaling tone (10 kHz) while ringing leaked into the receiver of the other
uplink, and when it stopped, that looked like the caller hanging up. Keep
one channel free between the two (60 kHz still fits at 192 kS/s). `--headless` cannot be
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
        --sdr-tx-ppm <ppm> -B 1200 --oneshot -y alpha -M "HELLO" 1234567
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
        --sdr-rx-gain 30
   ```
   librpitx corrects the Pi crystal from NTP by itself (when NTP is synced),
   so `--sdr-tx-ppm` is usually not needed.
6. Two channels: `-k 1 -k 3 -T CC -T TC`, calling channel first, one free
   channel between them (see below).

## Base station setups with one Pi

The Pi sends one carrier, so at most two channels: the first (always on)
must be the control/calling channel, the second the keyed one. Leave one free
channel between the two: with neighbouring channels the uplink of one leaks
into the other (a ringing phone's signalling tone released the other call).
The common part of every command (Pi 4, RX by an RTL-SDR):

```sh
COMMON="-s 192000 --sdr-tx-rpitx --sdr-tx-samplerate 192000 --sdr-rx-soapy \
        --sdr-rx-device-args driver=rtlsdr --sdr-rx-samplerate 960000 \
        --sdr-rx-gain 30"
```

With two channels the choice is between the clock and calls between phones;
both together would need a third channel.

| Setup | Command | What works | Tested on a Pi 4 (2026-11-23) |
|---|---|---|---|
| NMT basic, one channel | `nmt -k 1 -Y ua,1 -T CC/TC -e $COMMON` | calls to and from the network, echo test on 999 | echo test with two phones: clear, 0 underruns |
| NMT clock, two channels | `nmt -k 1 -k 3 -T CC -T TC -U 1 -I 1 -Y ua,1 -e $COMMON` | date and time on the phones (not allowed on a combined CC/TC, NMT Doc 450-1 4.3.3.12.5), calls on the traffic channel; one call at a time, a second caller waits | clock shown, calls of two phones one after the other |
| NMT phone to phone, two channels | `nmt -k 1 -k 3 -T CC/TC -T TC -x -I 1 -Y ua,1 $COMMON` | one phone calls the other (`-x`), caller ID on the ringing phone; no clock (combined channel) | caller ID shown, call through |
| AMPS basic, one channel | `amps -k 354 -T CC/PC/VC -S sid=<sid> -e $COMMON` | calls to and from the network, echo test | (USRP and Pi 3) |
| AMPS phone to phone, two channels | `amps -k 354 -k 356 -T CC/PC/VC -T VC -x -I 1 -S sid=<sid> -S pureg=0 -S pdreg=0 $COMMON` | one phone calls the other, caller ID (Alert With Info), release by either phone; no echo test (999 is refused) | both directions: caller ID shown, sound OK, the other phone released by the Release order |
| FLEX network, transmitter only | `flexpager -k 931.9375 --network --biw-datetime auto --biw-sysinfo auto -s 192000 --sdr-tx-rpitx --sdr-tx-samplerate 192000 --sdr-tx-only`, messages into `/tmp/flex_msg_send` | network mode with date, time and time zone in the BIW; alpha at 1600 and 6400 bps (4-FSK), long text, maildrop, temporary group; no RTL-SDR needed | received by `flexpager --sdr-uhd --sdr-rx-only` on a B210: all decoded, frames 92/92 codewords clean, rpitx never late |

AMPS has no clock broadcast. The console has no number of its own, so a call
from the console shows no caller ID; test caller ID phone to phone. Run
interactively (in tmux) to dial from the console: type the number, then `d`.
Under `sudo`, `~` is root's home: give log files with an absolute path.

Three channels: not supported (refused). A simulation of the phase of three
summed channels gives at best 27 / 25 / 18 dB SINAD (NMT channels 1, 3, 4,
levels 1 / 0.7 / 0.5; with equal spacing the products of the outer channels
land on the middle one), before the Pi's own limit, which already holds two
channels to about 25 dB. Tried on a Pi 4 (2026-11-23, test build, CC + TC +
TC, 192 kS/s; channels 1, 4, 6 need more than 75 % of 192 kS/s): no clock on
the phones, the call on the weakest channel failed (supervisory quality 39 %),
the one on the middle channel was "noisy, at the limit" and lost its
supervisory signal after a few seconds. Not usable on a Pi 4.

Measured afterwards with test tones (twoch with three channels, 1 / 1.7 /
2.3 kHz at 3 kHz deviation, received by the B210 112.5 kHz below the
carrier, drive level 7), SINAD per channel:

| Layout (offsets from the carrier) | Levels | SINAD |
|---|---|---|
| two channels, 25 kHz apart (reference) | 1 / 0.6 | 24.2 / 21.4 dB |
| -37.5 / +12.5 / +37.5 kHz (NMT 1, 3, 4) | 1 / 0.7 / 0.5 | 13.2 / 12.5 / 9.2 dB |
| same | 1 / 0.5 / 0.4 | 17.0 / 13.8 / 7.2 dB |
| same | 1 / 0.4 / 0.35 | 19.0 / 15.2 / 6.9 dB |
| same | 1 / 0.3 / 0.3 | 20.9 / 14.9 / 3.6 dB |
| same, 240 kS/s | 1 / 0.45 / 0.45 | 15.0 / 3.3 / 4.4 dB |
| -62.5 / +12.5 / +62.5 kHz (NMT 1, 4, 6) | 1 / 0.4 / 0.35 | 14.4 / 3.0 / -1.8 dB |
| -75 / 0 / +50 kHz | 1 / 0.5 / 0.4 | 8.6 / 7.7 / -4.2 dB |
| -50 / +25 / +75 kHz | 1 / 0.4 / 0.35 | 18.1 / -0.8 / -7.1 dB |

Weaker second and third channels (their sum below the first, so the sum of
the three never comes near zero) help the first two; the third stays below
10 dB. Wider layouts are worse on air, the opposite of the simulation: the
further the carrier has to swing from sample to sample, the larger the
transmitter's error. Two channels are the practical maximum of a Pi 4.

The drive level is already the highest (7); the pad's slew rate limit
(PADS_GPIO_0 bit 4, librpitx clears it) makes no difference: one carrier
at 463 MHz received at -30.9 / -32.7 dB (limited) and -32.5 / -31.3 dB (not
limited), SNR 46.5..47.4 dB either way. More signal needs an amplifier.
