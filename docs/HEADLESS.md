# Headless apps, operations outside (proposal)

Status: proposal, 2026-11-22. Step 1 (`--headless`) is done, the rest is
open for decisions.

## Three domains

Each app (nmt, amps, pocsag, ...) mixes three things today:

| domain | what | where it lives now | configured by |
|---|---|---|---|
| radio | SDR or sound card, frequencies, rates, gains, ppm, rpitx | `libsdr`, `libsound`, `sender.c` | `--sdr-*`, `-a`, `-s`, ... |
| protocol | the network: channels, channel types, traffic area, timers, the state machines | each app (`src/nmt`, `src/amps`, ...) | the app's own options |
| operations | everything a person does with a running network: calls, pages, watching, testing | `libmobile/console.c`, the main loop in `libmobile/main_mobile.c`, `libdisplay` | partly options, partly keys pressed at run time |

Radio and protocol are already set on the command line or in
`~/.osmocom/analog/<app>.conf`, once, at start. Operations is the part that
does not fit: it is interactive, needs a terminal, has its own libraries
(ALSA call device, resampler, jitter buffer, test music, echo analysis), and
it is one fixed operator panel inside every app.

## What operations is made of

| part | today | needs a terminal | can go outside through |
|---|---|---|---|
| call phone: dial a mobile, answer, headset audio | `console.c`, `-c hw:..` | yes (keys) | Osmo-CC (exists) |
| paging input: send a page | console dialling, `--fifo`/`--fifo-file`, caller ID text in an Osmo-CC setup | no | Osmo-CC or the fifo (exists) |
| test peer: echo test `-e`, echo analysis, loopback `-l`, test tone to the mobile | `console.c`, refused with `-o` | no | missing |
| status: call states, who registered (`console_inscription`), measurements | `libdisplay` keys `c`, `m`; the console line | yes | missing |
| internal tools: wave, IQ and spectrum views (`w`, `q`, `s`), info and scheduler dumps (`i`, `z`), bias calibration (`b`) | main loop hot keys | yes | missing |
| terminal handling: raw mode, banner, scroll window for the log | main loop | yes | goes away when headless |

The protocols touch operations at one point only: `console_inscription()`
(nmt, amps, cnetz, r2000, mpt1327, fuvst) tells the console which phone
registered, so a call to it can be dialled. Everything else already goes
through Osmo-CC: the console is just the built-in peer of the app's Osmo-CC
endpoint (`console_msg` given to `osmo_cc_new` in `call.c`). With `-o` that
peer is replaced by the socket.

## What it costs

Not CPU: POCSAG on the x86 mock for 20 s used the same time with and without
the console. The cost is use and upkeep:

- it needs the terminal: under `timeout`, a service or a script without
  `--foreground` the app is stopped (SIGTTOU) and never sends;
- the operator panel is the same for paging and for telephone networks,
  where dialling and headset make no sense for a pager;
- echo test, loopback and test tone are tied to the console, so they are
  lost as soon as the app is run with `-o`;
- the panel's code and libraries are in every app.

## Upstream already has a standalone call phone

`osmo-cc-alsa-endpoint` (gitea.osmocom.org/cc/osmo-cc-alsa-endpoint, binary
`osmo-cc-alsa-endpoint`): headset on ALSA, dial from the keyboard, caller
ID, auto alerting / answer, early audio, loopback, Osmo-CC socket. That is
the call phone part of our console, as a program of its own:

```sh
nmt -k 1 -Y ua,1 ... --headless
osmo-cc-alsa-endpoint -a hw:0,0 --cc "remote 127.0.0.1:4200"
```

Missing there against our console: number length check per network, the
registered phone's number to dial back, test music towards the mobile. It
is on libosmo-cc 3.0.0 (we are on 3.2.0).

## Steps

1. **`--headless`** (done, 88a679c). No keyboard, banner, displays, terminal
   left as it is; implies `-o` (or works with `-x`). Tested on the x86 mock in
   a pseudo terminal: plain `timeout` stops the normal app (state T, nothing
   sent), the headless one runs and stops cleanly.
2. **Test peer outside.** Echo, echo analysis, loopback and test tone as an
   Osmo-CC endpoint of their own (a small program, or a mode of the call phone),
   so a headless app can be tested the same way as with a real call. The
   alternative is keeping them inside the app (`call.c`) as an answer mode.
3. **Call phone outside.** Use `osmo-cc-alsa-endpoint`, add what is missing
   (or lift `console.c` into a program of its own). After that the built-in
   console could be left out of builds.
4. **Status and control interface.** A local socket (lines of JSON, one
   event or command per line): channels and their state, registrations
   (what `console_inscription` reports), call states, measurements; commands
   for the internal tools (info dump, scheduler dump, bias calibration). This
   is what a TUI, web page or GUI would use; the IQ/spectrum/wave views would
   stay terminal-only or get raw data later.
5. **Operations front ends.** TUI first (replaces the built-in panel), web
   later; one front end can serve several apps (NMT and POCSAG on one Pi).

## Open decisions

- Test peer: own program, or an answer mode inside the app (step 2)?
- Call phone: contribute to `osmo-cc-alsa-endpoint` (upstream), or our own
  console program?
- Status interface: own socket, or Osmo-CC messages of a new kind? Osmo-CC
  carries calls only; the panel's data is not calls.
- Several apps on one machine: each needs its own Osmo-CC ports (`--cc
  "local ..."`); the front end needs a list of them.
- Built-in console: keep it as the default for people who run the apps by
  hand, or make headless the default once steps 2 and 3 are there?

## Keeping upstream merges easy

Every step adds a switch or a new file; `console.c` and the main loop stay
in place until the outside parts can replace them. Upstream has not changed
`console.c` or `main_mobile.c` since 2025-12-06.
