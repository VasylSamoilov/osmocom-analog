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
   c. N stations: several `-f`, the loop over them, the SDR bands.
   d. the proof: FM stereo + RDS, AM, NFM together offline.
6. The wide bank (in-process from the station list, backend option).
7. Per-station retune.
8. B210 run (cable loopback), after asking.
