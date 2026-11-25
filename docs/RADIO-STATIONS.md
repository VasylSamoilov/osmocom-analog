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
   b. the option rule (a second `-f` refused until step 5);
   c. main() carved into station init / poll / exit; the paging FIFO's
      line buffer into the station.
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
   (engine in-process and two clients on a backend).
5. N stations: offline (IQ files / engine loop) FM stereo + RDS, AM, NFM
   through the engine and back: SINAD per station, each station's own
   PI / PS decoded, none leaking into another.
6. The wide bank (in-process from the station list, backend option).
7. Per-station retune.
8. B210 run (cable loopback), after asking.
