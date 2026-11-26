# Several stations in one osmoradio (design, 2026-11-25)

Goal (the user): one osmoradio runs several stations of any kind it
supports - broadcast FM (mono, stereo, RDS, SCA), AM (DSB, USB, LSB),
narrow FM - each with its own RDS, servers, files, meter and everything
else it enables. In-process on one SDR and on the SDR backend
(docs/SDR-BACKEND.md), as the other programs.

Today: one osmoradio = one station = one band channel (channels = 0; on
the backend one registration). Several stations = several processes.

## Pieces

1. **A station is one object.** `station_t` holds its `radio_t`, its
   options, its signal meter, its servers (XDR-GTK, RDS Spy, UECP,
   ASCII G, rigctl), its paging pipe, its files (wave in / out, hexrds,
   bitstream, TMC), its buffers and its SDR band index. Everything per
   station; nothing of one station reaches another.
2. **radio.c keeps no station state in statics.** Today the RDS preset,
   PI, callsign, ECC, TA, DI flags, CT, `freq_name` and the polyphase
   switch are file statics: they move into `radio_t`. Debug counters
   (`#if 0` blocks, log timers) are per station too or removed where they
   are dead. Process-wide and read-only after loading: the RDS FEC table
   and the TMC event / location tables (one TMC file set per process;
   different files on two stations are refused at start).
3. **Options: before the first `-f` defaults for all stations, after a
   `-f` that station's.** `-f` starts a station; every station option
   after it (modulation, audio in / out, stereo, RDS fields, servers,
   files, deviation, bandwidth, volume, AFC, ...) is that station's until
   the next `-f`. The config file (~/.osmocom/analog/radio.conf) is read
   first, so it gives defaults. One `-f` anywhere: as today. SDR options
   (`--sdr-*`) and process options (log level, real time priority) are
   process-wide wherever they stand. Flag options (`--stereo`, `--rds`)
   take no value, so per-station lists (`-M fm -M am`) would not work for
   them: positional it is.
4. **libsdr: one `sdr_t`, N bands.** A band = one IQ channel at the
   station's frequency and DSP rate, read / written by index
   (add, read, write, tosend). In-process each band is an engine group
   (its own rate); on the backend each band is its own registration
   (sdr_client_open per band; groups are independent, up to 64). N = 1
   keeps today's channels = 0 path (own centre, the band channelizer)
   unchanged.
5. **Rates are classes on the engine.** A station's DSP rate is not
   `sdr_calculate_optimal_rate()` (SDR rate / 2^k) but the class of its
   kind: the bank's DSP rate (48 k) for AM and NFM (a bank group takes
   any rate up to the bank's through its resampler), 240 k for mono FM,
   480 k for stereo / RDS / SCA. One station alone keeps today's choice.
6. **Wide stations: chain for one, a wide bank from 2 - 3.**
   bench_nwide (docs/SDR-BACKEND.md): the chain costs ~9 % of a core per
   station at 9.6 MS/s, a bank for the wide rate wins from 2 - 3
   stations. Declared at start, never moved at run time: in-process from
   the station list; on the backend by an operator option (the backend
   does not know its clients in advance).
7. **One main loop over the stations first** (read, radio_rx, radio_tx,
   write per station, servers polled per station). Threads only if the
   per-station time measured offline needs them.
8. **Keys are operations, not per station.** osmoradio still reads its
   own terminal keys (m, s, w, b displays; d; f RDS preset; 1 - 3 RDS
   flag flips; q quit) and has no `--headless`; it was not part of the
   docs/HEADLESS.md work (that is the mobile programs' call console).
   Those operations belong to the status and control interface
   (HEADLESS.md step 4), addressed per station. Until that exists: keys
   with one station as today, with several only q; `--headless` for
   osmoradio as for the others.
9. **Start-up refusals:** two stations on the same server endpoint,
   paging pipe, audio device or output file; a station whose band would
   cover the SDR centre in-process (DC, LO leakage; AM carriers first) -
   the centre is placed off all stations; stations outside the SDR
   bandwidth; more wide stations than the chain cap without a wide bank.
10. **Retune per station** moves that station's channel inside the band
    (in-process: the group's offset; backend: re-register), it does not
    move the device. XDR-GTK spectral scan moves the SDR window today:
    with more than one station it is limited to the band the SDR covers
    (or refused).

## Slices (each a usable state, each proven before the next)

1. radio_tx() exact count (done, 399e63e).
2. radio.c statics into `radio_t` (done): `radio_opts_t` (polyphase, PI,
   callsign, ECC, paging preset, TA, DI flags, CT; `radio_opts_init()`)
   given to radio_init() and kept in the radio; the RDS preset index and
   the wave display names in `radio_t`; the radio_set_* setters gone.
   Proof (txfix harness, the radio.c of 399e63e against this one, test
   tone, random requests, RDS CT off): bit-identical TX IQ for FM mono
   (linear, polyphase), stereo, stereo + RDS (preset; PI + ECC; paging
   preset), mono + RDS with a callsign, AM (linear, polyphase); repeatable
   run to run. Found on the way, before and after alike: radio_exit()
   does not free the RDS encoder's biphase waveform (2304 bytes, at exit;
   fixed in step 3, below).
3. `station_t` and the option rule with N = 1, three commits:
   a. the per-station file-scope variables of main.c in one `station_t`
      (`st->`), `station_defaults()` (done);
   b. the option rule (a second `-f` refused until step 5) (done): the
      config file and the options before the first command line `-f`
      fill `station_default`, that `-f` starts the station as a copy of
      it; a `frequency` in the config file is the default frequency, not
      a station start; no `-f` on the command line: the station is the
      defaults. Process-wide: `-h`, `-v`, `--fast-math`, `--limesdr*`
      (SDR presets), all `--sdr-*`. Everything else is a station option.
      Behaviour change: a second `-f` was silently the last one wins, now
      it is refused (exit non-zero, before the SDR is opened). Proof:
      suite.sh all 12 the same against 1812553; opts.sh (same cache dir):
      stereo + RDS + PI + ECC and AM with the station options before
      `-f` = after (TX IQ, SDR calls, log), SDR options before = after,
      config file frequency without `-f` = the same on the command line =
      1812553, config frequency + `-f 101e6` on 101 MHz as before, config
      `-V` overridden after `-f` as before, a second `-f` refused. With
      one station the copy itself cannot be seen: per-station separation
      is proven in step 5. For step 5 (the user, 2026-11-26): every kind
      of station has its own set of options - no `no-` forms. A default
      applies to the stations whose kind it belongs to (`-S`, `--rds*`,
      `--sca*`, `-D`, `-E`, `--fm-compandor`, AFC: FM; `-I`, `-C`: AM;
      audio, files, `-B`, `-V`, `-s`, rigctl: all) and is not applied
      to the others; the same option after the `-f` of a station of
      another kind is refused (as `-S` with AM is today). Needs to know
      per option where it was given (defaults or the station) and the
      option -> kinds table checked against the code before it is
      written.
   c. main() carved into station init / poll / exit; the paging FIFO's
      line buffer into the station (done): `station_init()` (radio,
      AFC, meter, RDS files, TMC, paging + FIFO, servers, buffer),
      `station_poll()` (paging FIFO, servers, RX, TX; the loop's sleep
      and the keys stay in main), `station_exit()` after sdr_close().
      The station holds its running state too: IQ rate, buffer size,
      buffer, `radio_t`, the five servers, the callback context, the
      FIFO's partial line. Dead `#if 0` loop debug counters removed.
      Proof: suite.sh 12 the same and opts.sh against 5b20f91;
      extra3c.sh: a page written in two parts and two lines in one write
      (both decoded and logged as before), RX with XDR-GTK + RDS Spy +
      rigctl servers (RDS Spy client gets the same 552 bytes with PI,
      rigctl `f` the same, wave and hexrds the same), a server that
      fails to start with paging on (same log, no sanitizer / leak
      report, FIFO removed); the moved code diffed against 5b20f91's
      main() after mapping the names: only the function frames, the
      goto / break / continue -> return and the `#if 0` blocks differ.
      Changed order at exit: the buffer and the meter are freed after
      sdr_close() now (before it until 5b20f91); the fake SDR has no
      threads, so this is not measured on a real SDR (libsdr does not
      use the meter). Left for step 5: the server callbacks (tune,
      scan, rigctl frequency) still use the global `st`, they get the
      station through their context then; station_exit() unlinks the
      paging path also for a station without paging (a second station
      would remove another's default pipe: unlink only what this station
      made); settings and running state share `station_t` (the defaults
      carry an unused `radio_t` and servers): split them before there
      are N copies.
      Found (parked, before and after alike): the rigctl SDR caps block
      in the server setup never runs - it checks the SDR handle, which
      is opened later.
   Proof for each: `~/.cache/sdrangel-rp-rates/txfix/suite.sh` - osmoradio
   of two revisions on a fake libsdr (fake_sdr.c in place of sdr.o: no
   device, no threads, pseudo-random tosend / RX chunks, IQ to / from
   files, SIGINT after N samples, optional real-time pacing; ASan +
   UBSan), the same command lines on both: TX IQ bit-identical (FM mono,
   stereo + RDS + PI + ECC, RDS callsign, AM linear / polyphase, NFM, FM
   polyphase, AFC options, all five servers on 127.0.0.1, a config file),
   RX of FM stereo + RDS (wave and hexrds the same), RDS paging through
   `--rds-paging-fifo` (the encoder's log the same), logs the same
   (colours, paths, line numbers, pids out). 3a: all 12 the same against
   778b4f7. A page through the FIFO at ~1 s goes out after the encoder's
   start-up warmup (57 groups, ~5 s, PS only; on purpose: receivers lock
to PI / PS of a station that has just come up before any data): the
paging case runs 12 s
   and needs the 7A groups decoded (an earlier "no 7A" was its 6 s
   window). Fixed on the way: radio_exit() never called
   rds_encoder_exit() / rds_decoder_exit() - the call on air and the
   biphase table leaked, the RDS bitstream file lost its last byte; the
   suite counts LeakSanitizer reports now.
4. libsdr band API; offline test with three bands at different rates
   (engine in-process and two clients on a backend) (done):
   src/libsdr/sdr_bands.[ch]. `sdr_bands_engine(b, engine, rx_centre,
   tx_centre)` (in-process: the engine of an SDR opened in server mode)
   or `sdr_bands_backend(b, socket)`, then `sdr_band_add(b, def)` per band
   (name, RX / TX flags, frequencies, IQ rate, buffer size; returns the
   index or the reason) - one engine group, or one backend registration,
   with one RX and / or one TX channel; `sdr_band_read()` /
   `_write()` / `_tosend()` by index, interleaved float IQ as osmoradio
   has it, TX scaled to the band's level (1 / N max), rings and TX target
   as sdr.c's band client; read returns -EPERM when the backend is gone;
   `sdr_bands_close()`. The API hangs on a bands object, not on the sdr
   handle: `sdr_band_add(sdr, ...)` becomes a thin wrapper in sdr.c in
   step 5. Proof: src/test/test_sdr_bands (null device, TX looped to RX,
   real time, ASan + UBSan clean): AM 48 k at -300 kHz and NFM 24 k at
   +250 kHz on the bank, FM 240 k at +600 kHz on the chain, one loop over
   the three; the same body in-process and on the backend: every band
   all its rows (rate x 0.8 s), own tone 0.000 dB, its mirror <= -166 dB
   (I / Q order), the other bands' tones <= -166 dB, 0 TX rows missing /
   RX rows dropped; refusals: a band outside the SDR band, without
   direction, a third TX band on an engine for two, a chain band over
   SDR_ENGINE_MAX_CHAIN, a read above the buffer size or of a band not
   there; nothing left on the engine / backend after close; the backend
   stopping while a band is registered: read -EPERM. test_sdr_engine and
   test_sdr_backend pass as before. For step 5: close the bands before
   sdr_close() (the engine frees its groups at exit; removing a group of
   stopped device threads waits 2 s and leaks it), i.e. not in
   station_exit() after sdr_close(); osmoradio's loop calls
   sdr_server_poll() (the raw ring, RX status); each station sends at
   1 / N of full scale; server mode refuses when no bank fits, so a set
   of FM stations only still builds a bank - measure its cost; sdr.c's
   band client branch duplicates sdr_bands (one of them goes).
5. N stations: offline (IQ files / engine loop) FM stereo + RDS, AM, NFM
   through the engine and back: SINAD per station, each station's own
   PI / PS decoded, none leaking into another. In parts:
   a. the station through the callbacks, N = 1 (done): the servers'
      context has its station (`rds_callback_ctx_t.station`); the XDR-GTK /
      RDS Spy tune, the scan retune, the ASCII-G tune (its context was
      NULL) and the rigctl callbacks (their argument is the station now,
      was its radio) change and read that station, not the global `st`;
      station_exit() removes the paging FIFO only when this station made
      it (behaviour change: a station without paging no longer removes
      what is at its FIFO path, the default /tmp/rds_paging_send of
      another osmoradio included). Settings and running state stay in one
      `station_t`: the defaults' running part is never started (zeros),
      a station starts as a copy of it, so the split buys nothing but a
      bigger diff. Proof: suite.sh 12 the same, opts.sh, extra3c.sh as
      before, against 89ff013; extra5a.sh: RDS Spy tune to 101 MHz,
      rigctl `F 102000000` then `f` (RX), ASCII-G tune to 103 MHz (TX):
      the same SDR calls (sdr_set_rx_frequency 101 / 102 MHz,
      sdr_set_tx_frequency 103 MHz) and logs (incl. "Leaving 101.0 MHz":
      rigctl sees the frequency RDS Spy set), a file at the FIFO path of a
      station without paging: 89ff013 removes it, now kept. The suite runs
      a case once more when the TX IQ differs: RDS CT (4A) carries the wall
      clock minute, A and B on both sides of a minute edge differed once.
      Not covered: the XDR-GTK scan retune (same change as the tune).
      Found (parked, before and after alike): rigctl `f` after an RDS Spy
      tune still says 100 MHz - the rigctl server keeps its own copy of the
      frequency, the tune of another server does not reach it.
   b. each kind its own options (done): the kinds are the modulations
      (fm, am = DSB, usb, lsb; narrow FM is fm with its -D / -B). One
      table in add_options() (`opt()`: name, parameters, kinds) from
      handle_options() and radio_init():
      fm only: -D, -E, --fm-compandor, -S, --rds, --rds2, --rds-debug,
      --rds-verbose, --sca-67k, --sca-92k, --call-sign, --pi, --rbds, all
      --rds-paging*, --rds-ecc, --rds-hexrds-file, --rds-bitstream-file,
      --rds-tx-*-file, --rds-tmc-*, the RDS servers (--xdr-gtk-server,
      its password - it turns RDS on -, --rdsspy-server, --uecp-server,
      --asciig-server), --afc*; am (DSB) only: -I, -C (radio_init() uses
      them in the DSB branch only); every kind: -f, -s, -r, -w, -a, -M,
      -R, -T, -B, -V, --channelizer-rate, --polyphase-resampler,
      --rigctl-*; process: -h, -v, --fast-math, --limesdr*, --sdr-*. The
      command line options are recorded per station (after its -f, or all
      of them without -f); after parsing, when the modulation is known
      (-M may come last): one of another kind is refused (exit non-zero,
      before the SDR, "Option '--stereo' is for stations with modulation
      fm, this station is am"), and the settings of the other kinds are the
      initial ones again (a default of another kind does not apply).
      Behaviour changes: `-M am -S` refused by the kind check (was "Stereo
      works with FM only", exit 0); `-S` before -f or in the config file
      with an AM station: not applied, the station runs (was refused);
      `-M usb -C`, `-M fm -I`, `-M am -E` refused (were ignored: -E only
      scaled the AM volume, see below). Proof: suite.sh 12 the same,
      opts.sh, extra5a.sh against ad433ad; opts5b.sh: an FM option after
      an AM station's -f refused (-S, -E, -D, --rds, --pi, --afc,
      --fm-compandor, --rdsspy-server), -I on FM and -C on USB refused,
      FM options before -f / in the config file with an AM station = plain
      AM (TX IQ, SDR calls, log), the same config file with an FM station
      = ad433ad (and differs from plain FM: applied), AM options in the
      config file with FM = plain FM, with AM = ad433ad (differs from plain
      AM), `-I 0.5 -C` after -f = ad433ad, no -f with the frequency in the
      config file: the command line is the station's (-S on AM refused).
      Found (parked, before and after alike): radio_init()'s FM
      modulation budget runs for AM too - an AM station's volume is
      scaled by the 1 kHz pre-emphasis gain of the time constant (50 us:
      x 0.956); the AM station keeps the initial 50 us, so its output is
      as before. Known flake: opts.sh compares two runs of B with RDS,
      the CT minute edge can make them differ once (run again).
   c. N stations (done): up to 8 `-f`, each a copy of the defaults; the
      kind check and main's checks per station (station_check_options(),
      station_signal(): the moved code as it was, on the global st set per
      station). One station: everything as before (sdr_open() of its band,
      spectral measurement, keys). Several: the IQ rate is the class of
      the signal (extent x 2 / 0.75 up to 48 k: the bank's DSP rate, up
      to 240 k, else 480 k; or --channelizer-rate); start-up refusals
      (stations_refused(): a server endpoint, a paging FIFO, an audio
      device, an RX wave / hexrds / bitstream file on two stations,
      different TMC tables, overlapping bands of one direction); in-process
      the SDR opened as a backend's device (sdr_server_mode(TX stations),
      sdr_open() without channels at the centre, the bank at 48 k), the
      centre between the lowest and highest station, moved beside a
      station that covers it (+ 10 kHz); on --sdr-backend no sdr_open(),
      a registration per station; a band per station (sdr_bands), each at
      1 / (TX stations) of full scale. The loop polls every station in
      turn (none skips another), sdr_server_poll() every 10 ms; a
      station's RX: its band, the meter, the radio, its servers - not the
      scan, not the spectral measurement (one per process, at one rate);
      the SNR from the SDR's RX status noise floor when valid, else
      unknown (the radio's initial -1: no SNR-driven forced mono; the
      meter says SNR 0 without a noise floor, which forces mono). The tune
      callbacks (XDR-GTK, RDS Spy, ASCII-G, scan, rigctl) refuse: they
      move the SDR (slice 7). No keys with several stations: the terminal
      is left as it is, CTRL+c is SIGINT (the user, 2026-11-26: "we are
      still reading keys?"). Exit: the bands, then the SDR, then the
      stations. Proof (fake SDR, same command lines, c1e260e): suite.sh
      12 the same, opts.sh the same except a second `-f` (a station now),
      opts5b.sh, extra3c.sh, extra5a.sh the same. Several stations
      offline: fake_sdr.c got a server mode - the real engine
      (sdr_engine.c) at --sdr-samplerate, a device thread with TX looped to
      RX in real time (1 ms blocks), optional white noise (FAKE_NOISE),
      sdr_bands.c / sdr_client.c real; FM 100.0 MHz (240 k, chain), AM
      100.6 MHz (48 k, bank), NFM 99.4 MHz (-D 2500 -B 3000, 48 k, bank),
      each TX + RX of a 1 kHz tone (wave in, wave out), centre 100.1 MHz,
      TX 1/3 each: SINAD (sinad.py: the tone fitted, the rest from 100 Hz
      to the band edge) FM 61.2 dB, AM 73.0 dB, NFM 41.8 dB, ASan / UBSan
      clean. Found on the way, fixed (15e3d63): libam's AM demodulator
      gave 0 / 0 = NaN for IQ of exact zeros (a band before its first TX,
      a gap the SDR fills with zeros) and the receiver stayed at full scale
      for good - one station through IQ files with 0.1 s of zeros in front
      too (c1e260e: SINAD -52 dB; fixed: 73.1 dB, without zeros the same
      wave). Not covered offline: sdr.c's server mode with a real device
      (slice 8), several stations on --sdr-backend.
   d. the proof (done): multi5d.sh, four stations on one fake SDR at
      1.92 MS/s (centre 100.0145 MHz, moved off the AM station): S1 FM
      stereo + RDS PI 5A12 99.6 MHz (480 k, chain), S2 FM mono + RDS PI
      5B34 100.4 MHz (480 k, chain), S3 AM 100.0 MHz (48 k, bank), S4 NFM
      99.9 MHz (48 k, bank); each its own tone (700, 1000, 1300, 1900 Hz:
      another station's audio in it would be seen), TX + RX of itself. The
      same command every run, only who sends changes. The meter (xtalk.py):
      the tone fitted by least squares and subtracted, the rest from 100 Hz
      to the band edge; the other tones by their fitted amplitude. (The
      first meter, an FFT with a Hann window and +- 3 bins, read 42 dB
      whenever the half file was not a whole number of bins - 13.999 s
      instead of 14.000 s: window leakage, not the radio; every "42 dB" and
      "999.9 Hz" seen with it was that.) SINAD (dB), one station sending /
      all four / all four again / all four with white noise at -50 dBFS:
      S1 55.4 / 55.4 / 55.4 / 51.9 (L and R alike), S2 71.1 / 71.1 / 71.1
      / 69.8, S3 77.3 / 77.4 / 77.4 / 50.5, S4 96.1 / 94.9 / 94.7 / 61.3.
      The other stations' tones in each: -75 .. -90 dB below its own, and
      the same to 0.1 dB when they do not send (S1 alone: -76.3 / -82.1 /
      -89.5 dB, all four: the same): the station's own residual, the
      crosstalk is below it. A station's TX in the others' bands (SDR rate
      IQ, S1 sending alone, a Blackman window: upper bounds): -145 dB in
      S2's band, -163 dB in S3's, -176 dB in S4's, -127 dB at its own
      mirror. RDS: S1 decodes only 5A12, S2 only 5B34, in every run.
      Negative control, S2 not sending: no RDS groups for S2; with noise
      its audio is noise (every tone within 4 dB of it); without noise
      its FM limiter makes audio of the others' trace (the -145 dB) - in a
      noise-free loop only. No band reported TX rows missing / RX rows
      dropped (osmoradio reports them per band once a second when they
      change, as sdr.c does for its channels). ASan / UBSan clean.
      On the SDR backend (backend5.sh: osmo-sdr-backend built on
      fake_sdr.c, the real engine and the looped device; osmoradio a
      separate process with --sdr-backend): the four stations registered
      (4 programs) and removed at the end; SINAD 55.4 / 71.1 / 77.4 /
      95.6 dB, the same residual at the others' tones, each only its PI;
      both processes ASan clean. Servers and paging per station
      (extra5c.sh): two FM + RDS paging stations, each its RDS Spy port and
      FIFO - each RDS Spy client only its PI (34 x 5A12 / 34 x 5B34), a page
      into station 1's FIFO: 7A groups in station 1's RDS only; rigctl on
      station 2: its frequency, a retune refused ("no retune with several
      stations"); both FIFOs removed at exit. Refusals (opts5c.sh, exit
      non-zero, nothing sent): the same RDS Spy endpoint, XDR-GTK = rigctl
      endpoint, the same paging FIFO, audio device, RX wave file, hexrds
      file, different TMC tables, overlapping bands, 9 stations, a fifth
      wide station (the chain cap), a station beyond the usable band (75 %
      of the SDR rate, as sdr.c; added here - a band past Nyquist was
      taken); the same TX wave (an input) on two stations is fine. An
      error opening the SDR with several stations exits 1 (one station:
      0, as before). Changed on the way: the SNR of several stations is
      not the SDR's RX status - its noise floor is the histogram of the
      whole band, the other stations in it, it would force mono; the SNR
      stays unknown (the stereo blend then follows the program, the
      content cap: an L / R test tone pair is blended, so stereo
      separation is not measured here - L = R tones). Found (parked):
      in-process without any noise the first RDS station on the command
      line (the only one when one sends) has whole groups missing for the
      encoder's warmup (about 5.5 s, groups 4 - 66), the second not;
      white noise at -150 dBFS (the audio the same to 0.1 dB) makes every
      group decode, on the backend every group decodes: the decoder with a
      signal of no noise at all, not one station on another (the only
      statics in rds.c are debug counters and read-only tables). The
      NFM station's SINAD with a 1 kHz tone was 41.8 dB, with 1900 Hz
      95 dB: its harmonics of 1 kHz fall in its 3 kHz band (not looked
      at). Open: a per-station spectral measurement (noise floor: the
      SNR, the stereo blend, the XDR-GTK / rigctl levels) - today one per
      process, the single station's; a PS per station: there is no PS
      option (the PS comes from the preset), so both FM stations send the
      same PS and PS separation is not checked; on --sdr-backend the
      usable band is the backend's (osmoradio does not know its rate);
      for slice 8: where the LO offset puts the DC / LO spur in server
      mode (since 2026-11-26 automatic, just beyond half the SDR rate
      where the driver applies one - see "LO offset" below).

   LO offset (the user, 2026-11-26: no fixed 1 MHz anywhere; calculated
   where the device supports it or given; where not, the SDR layer keeps
   the signal off the DC, TX and RX; the channels do not care): the
   programs' defaults (-1 MHz mobile / radio / backend, -3 MHz tv, -10 kHz
   dcf77) are gone, sdr_config_init() takes none. libsdr/sdr_lo.c decides
   per direction: given (--sdr-lo-offset, --sdr-tx/rx-lo-offset; 0 =
   none) or automatic -(0.5 + 0.04) x the SDR rate - the operating point
   of every B210 run (1.92 MS/s, -1 MHz: just beyond half the rate, the LO
   outside the stream), now scaling with the rate (at 5 MS/s the old -1 MHz
   was inside the stream). UHD applies it (its DSP moves the signal back);
   SoapySDR only knows at open (soapy.c applies it with a tuning stage,
   else drops it and says so), rpitx has no LO (0, nothing moved). The
   channels are placed off the centre as before; the band of a program
   with channels = 0 is moved above the DC (half its IQ rate + 4 % of the
   SDR rate, the device tuned that much lower, retune too) unless the
   driver certainly puts the LO outside the stream (UHD, |offset| >= half
   the rate), when it fits the usable band (75 %), else it stays centred
   with a notice. An LO offset inside the stream: a notice for a channel
   on centre + offset. Proof: test_sdr_lo (the decision, 14 cases: the
   B210 runs' settings give -1036800 Hz, the LO outside, no shift; the
   old -1 MHz given: as before; SoapySDR bands shifted 196.8 / 336 kHz;
   no room; 0 given; rpitx); the real sdr.c on fake UHD / SoapySDR
   drivers (~/.cache/sdrangel-rp-rates/lo/lo_test: one band at 100 MHz, a
   tone 10 kHz above): TX in absolute frequency and RX in the band at
   +10 kHz for UHD 1.92 / 5 MS/s, SoapySDR 1.92 / 2.4 MS/s with 240 /
   480 k bands, 0 given, the old -1 MHz; the device's DC in the band
   128 - 178 dB below the tone when moved, no room (480 k at 0.96 MS/s):
   centred with the notice. The fake-SDR osmoradio suites the same (they
   do not run sdr.c). On the devices (2026-11-26, the user: "you can test
   with real uhd and rtl-sdr"): B210, cable loopback, 1.92 MS/s, TX 60 /
   RX 10, the real sdr.c + uhd.c (~/.cache/sdrangel-rp-rates/lo/b210_bw:
   one band at the full rate, a tone stepped -900 .. +900 kHz): the analog
   filter is centred on the LO, so with the automatic offset (LO 1.04 MHz
   below) and the bandwidth = the rate the band fell by 17.5 dB from -750
   to +750 kHz (21 dB below 4 MHz at +750 kHz), 3 MHz still 2.5 dB, 4 MHz
   flat within 1.8 dB - the rule bandwidth >= rate + 2 |offset| (also the
   2026-11-22 NMT finding at 200 kHz): e7be8ab makes it the default where
   the LO is outside the stream (a bandwidth given stays as given), the
   default then flat within 1.9 dB. The absolute level varies by up to
   9 dB from one device open to the next (the same 4 MHz: -33.1 / -38.4 /
   -36.6 dBFS), only the shape within a run counts; 5 MHz given: flat,
   10 dB lower (not looked at). With no offset (0) a tone at the centre
   came back at -91.5 dBFS instead of about -34: the B210's DC correction
   notches the centre - the channels must stay off it. RTL-SDR (SoapySDR,
   no tuning stage: the LO offset dropped, as before): B210 sends FM mono
   + 1 kHz at 465.70 MHz, the RTL receives with osmoradio (rtl_ab.sh,
   1.92 MS/s, gain 20): a92edd3 (band centred on the RTL's DC) 47.9 /
   48.0 / 47.9 dB SINAD, now (the band 196.8 kHz off the DC) 59.5 / 59.4
   / 59.6 dB, both orders. The B210 TX of a92edd3 vs now (received by the
   RTL, now): 59.5 / 59.5 dB both - the RTL's limit. Not proven: tv and
   dcf77 (band programs, no test here: their rates and the shift only
   logged); SoapySDR devices with a tuning stage (the offset applied: the
   band is moved too, harmless) - none here; SDR-rate IQ recordings made
   with a centred band (--read-iq-*-wave) now play with the band shifted
   on SoapySDR.
6. The wide bank (in-process from the station list, backend option).
7. Per-station retune.
8. B210 run (cable loopback) (done, 2026-11-26, the user allowed the
   radio): b210_multi.sh (~/.cache/sdrangel-rp-rates/lo), the four
   stations of 5d at 465.3 (FM stereo + RDS, 700 Hz), 466.1 (FM + RDS,
   1000 Hz), 465.7 (AM, 1300 Hz), 465.6 MHz (NFM, 1900 Hz), each TX + RX
   of itself, 1.92 MS/s, centre 465.7145 MHz, LO offset and bandwidth
   automatic (3.99 MHz). SINAD (dB) in-process TX 60 / RX 10, all four
   sending (two runs): S1 36.5 / 37.3, S2 57.3 / 58.1, S3 31.9 / 33.0, S4
   37.9 / 38.8; on the backend (osmo-sdr-backend, osmoradio a client):
   41.9 / 62.4 / 37.6 / 43.0 (the device's level differs per open by up
   to 9 dB, see LO offset). RDS on air: each station only its PI (207 -
   225 groups, at most one bad), in-process and on the backend, at every
   gain tried. Alone (the others RX only) 43.2 / 61.0 / 41.7 / 54.8 - not a
   level-fair reference on air: alone a station sends at full scale, with
   four at 1/4 (-12 dB) against the receiver's noise; offline there was no
   noise. Crosstalk on air (another station's tone in a station's audio),
   all four sending, TX 60: -47 dB of the AM 1300 Hz in the NFM station
   100 kHz away, the rest -58 .. -88 dB; alone those bins are -77 .. -90
   dB, offline the bands' isolation is -145 dB or better. It grows with
   the TX gain (AM in NFM: TX 50 / RX 20 -58.6, 55 / 15 -53.4, 60 / 10 -47.0
   dB, ~1.2 dB per dB): the B210 transmitter's nonlinearity, the AM
   envelope cross-modulating the others - an analog limit of four
   carriers through one device at this level, not the software. SINAD
   rises with the TX gain (the receiver's noise: S2 47.5 / 52.7 / 58.1 dB
   at the three settings; TX 50 or 40 with RX 10 is below the FM threshold,
   SINAD 10 - 18 dB, no RDS). The meter (xtalk.py) got a grid before its
   golden section search: over a 9 s window |DFT| has sidelobes every
   0.1 Hz, the search had landed on one (699.73 Hz for 700.00, -17.8 dB
   for 36.6); the offline numbers all read the exact tone and stand.

## RDS per station

Each station its own RDS: an encoder if it sends, a decoder if it
receives (radio_opts_t tx / rx, f26c7dc; was from the SDR's TX-only /
RX-only mode of all stations together). Nothing shared that changes
output: the encoder's timing counters per encoder, a decoder reset
(retune) keeps its callbacks, its paging setting, its reassembly timeout
and the bitstream file's partial byte, groups without block B reach the
servers too, the FEC table is filled at init (47f28c2). Left: the TMC
tables are process-wide (two stations' different tables are refused; a
station without its own logs with another's names), log lines are not
tagged with their station (the layer split, below).

Receiver paging (fceae91): --rds-rx-paging off | auto | on, the
receiver's own setting; the transmitter's --rds-paging no longer turns
it on. Send conservatively, receive tolerantly: the TX sends enhanced
paging only when asked (OPC, 13A, enhanced message types; experimental,
no pager to test it with), the RX decodes what the station signals. auto
(default): a 1A with the RPC group designation (EN 50067 Table M.1, 000 =
no basic paging) or the OPC (M.3.2.2, 0 = no enhanced paging; an
enhanced-only network sends designation 0) not 0 starts it - one from a
block without correction, else two in a row (8acf90a: with two always the
France Culture capture lost its first call, 3 groups after the first 1A)
-, two in a row with both 0 end it; not for a group type a 3A assigns to an ODA
(Table 6 "Available when not used for RP"; AID 0000 = the group's
normal feature, 3.1.5.4). Checked against the EN 50067 text
(RDS/WORK_EN50067_RDS_Standard.pdf): the 1A, 4A, 7A, 13A handling agrees;
the log's designation ranges were shifted (673a96d). Test (rxpage.sh,
~/.cache/sdrangel-rp-rates/txfix): on = the old forced decode (the same
logs); auto decodes the test page; off none (the noise run never synced,
so it shows nothing; the no-false-detection case is the captures: Amitie
89 syncs, no detection). The
captures in docs/rds through osmordstest --rds-rx-paging: France Culture
detected, the same 4 calls as on; the 7 stations without paging nothing,
auto the same logs as off at debug + verbose. Hand-written hexrds
(~/.cache/sdrangel-rp-rds/cases): basic, enhanced only, 1A that tell
nothing, the end (two 1A without paging; one does not), a 3A giving 7A to
an ODA (no paging) and AID 0000 (paging). A retune between two calls
(reset5.sh: RDS Spy tunes on its own connection right after the first
call): the decoder reset, sync again, paging detected again, the second
call decoded, the hexrds / bitstream files and RDS Spy still fed. Not
covered: a retune while a call is half assembled.

Layer split (agreed, in slices, each proven identical before the next):
1. decoder codec: one bits -> blocks -> groups function (it is twice
   today, rds_decoder_process and rds_decoder_feed_bit), out a group with
   its block status (done, 86618f5: rds_decoder_bit());
2. decoder application: hexrds / bitstream files, servers, periodic
   status, display, logs tagged with the station, fed from the codec
   (done: 9596aae rds_bit_received() / rds_group_received(), fb6c608
   rds_decoder_reports(), cb6490a + f1848d8 "(station N) " before each
   line of a station's work with several stations, liblogging's
   per-thread logging_prefix; 05a5db6 its console lines too);
3. decoded data per field (PS, RT, AF, CT, PTYN, EON, TMC, ODA) a store
   the codec fills; display and consumers (paging messages, TMC) read it
   through events (in part: 3a 6a38e64 / 8158013 a received paging call
   is an event with its own record, rds_paging_rx_msg_t, to the log and a
   consumer (osmordstest counts them); 3b d098b8f what a retune keeps is
   one block at the front of rds_decoder_t, the reset clears the rest
   (RDS_DECODER_RECEIVED); 3c 88b222b the display and the status dump
   take a const decoder; 3d-3h one change event, rds_changed(enum
   rds_change) to the display (rds_display_changed, its own state
   'shown') and a consumer (rds_decoder_set_change_callback): PTY, CT,
   the ECC vs RBDS check, a new PS, a finished 2A RadioText, the
   station's paging system (RPC, block D SI, OPC, PAC), 3i the long PS;
   3j a decoder fed only groups or bits (TX log, UECP, osmordstest) is
   made without a demodulator, rds_decoder_init_groups(). Done, with this
   boundary: the display has the lines shown without --rds-verbose and
   the state changes; the --rds-debug / --rds-verbose traces of what is
   being decoded (segments, calls being assembled, ON-PS, 3A, 13A bits)
   stay in the decoder, they are its trace, not a display);
4. encoder: content sources (PS, RT, AF, paging queue, UECP, Dynamic
   PS) apart from the scheduler and the bit / biphase generator. Proof
   first (the fake-SDR suite barely covers the scheduler): enc_harness
   (~/.cache/sdrangel-rp-rds/enc, run.sh <rev> <rev>) runs the encoder
   alone on its public API, on a fake clock from 23:59:30 UTC (time /
   clock_gettime / gettimeofday wrapped), and compares the samples and
   the logs of 7 scenarios: all content (9 group types reach a receiver,
   CT across the minute), Dynamic PS, paging (ten calls of all types at
   fixed times, battery saving, 13A; 13 decoded), the address scan (41),
   the TX hexrds file (twice) and the bitstream file, the CT correction
   (+25 h 17 s, across a day). Negative control: the warmup one group
   shorter shows in 'all' and 'dps'. Done: 4a 55983bc one 'group sent'
   point (rds_group_sent: the TX log decoder); 4b d039fb9 radio.c /
   main.c only through the encoder's API (preset restart, group
   versions, DI, logging, RT A/B; the encoder has no reset, its running
   state needs no keep block); 4c f32bf16 a sent paging call is an event
   with its own record (rds_paging_tx_call_t, start and complete) to the
   log and a consumer; 4d abddc12 UECP through setters (RT A/B kept, EON
   enable; tested with a UECP client, uecp_send.py), 1107f86 Dynamic PS
   advances on a 'PS cycle sent' event. The builders still move their
   own segment cursors (PS, RT, PTYN, AF, EON, TDC): running state of each
   group type, kept with it. Left: the paging queue (application) and its
   interval / slot timing (protocol) are one piece of code.

Ownership (after an architecture review): the decoder and the encoder
each give out through one sink, the station owns the rest.
- RX: f6c099f rds_decoder_sink_t (bit, group + present blocks, a received
  field, a paging message, TDC / IH / TMC, a new station, the 30 s
  report); the station's rds_rx_app_t (radio_t.rds_app) has the hexrds /
  bitstream files, the display and its 'shown' state, the servers' and
  consumers' callbacks. 18c9be9 the last display lines out (CT delta,
  paging auto detection); check: a decoder without a sink fed the
  captures (nosink.c) logs only its FEC table line. 82a4c94 the panel's
  colour is the caller's, the status dump the sink's.
- TX: 73e49ce rds_encoder_sink_t (a group sent, a sent paging call); the
  station's rds_tx_app_t (radio_t.rds_tx_app) has the sent calls' display
  and consumer and the TX monitor (a group-only decoder with its display,
  on with a TX file); the encoder owns no decoder.
- Paging TX timing is covered by the harness: a group duration changed
  (RDS_PAGING_GROUP_US) shows in 'paging' and 'scan'.
Next: the paging queue as a station-owned source (the queue walk, send
times, repeats, the scan and the FIFO; the encoder asks it for the next
call that may start now and keeps the markers, windows, 13A, A/B and the
call counter), then rds.c split into layer files (allowed as an exact
move by line range). Found, not fixed yet: on the bit path a lost block
keeps the previous group's status, the servers get it with the group (the
hexrds file uses the present mask and is right).
