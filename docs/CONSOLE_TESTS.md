# Console test switches

The built-in console (the call endpoint used when no `-o`/`--headless`
Osmo-CC socket is given) answers calls from phones automatically. Without a
sound device it plays test music to the phone. For measuring the speech path
with a real phone, the console has a few switches, set as environment
variables. They are meant for the test bench, not for operation.

| Variable | Value | What the console does |
|---|---|---|
| `OSMO_CONSOLE_SILENCE` | any | sends silence instead of the test music |
| `OSMO_CONSOLE_TEST` | any | sends a level sequence, 4 s each: silence, then 1 kHz at -60, -50, -40, -30, -20, -10 and 0 dB re the reference tone (1 kHz, 3 kHz deviation on NMT), looped; logs `CONSOLE TEST: <step>` |
| `OSMO_CONSOLE_SPEECH` | file | plays a file to the phone, looped: float32, 8 kHz, mono, 1.0 = peak of the reference tone |
| `OSMO_CONSOLE_DELAY` | seconds | echoes what the phone sends after the given delay (an echo test where you hear yourself later, so your own voice does not mask it) |
| `OSMO_CONSOLE_HANGUP` | seconds | hangs up (network side release) that many seconds after the call was connected |
| `OSMO_CONSOLE_RECORD` | file | records what is sent to and received from the phone, int16 pairs (sent, received), 8 kHz, in the same 20 ms frames, so both are aligned to the sample |

They work in the console's own call (`CONSOLE_CONNECT`), not with the plain
echo test `-e`: with `-e` the received frames are sent straight back and these
switches are not reached. Use `OSMO_CONSOLE_DELAY` for an echo instead. To
hang up during `-e`, press `h` in the console.

The console takes one call at a time; a second call is refused as busy
(cause 17).

## Examples

Recorded speech to the phone, and both directions recorded:

```sh
OSMO_CONSOLE_SPEECH=speech.f32 OSMO_CONSOLE_RECORD=rec.raw nmt -k 1 -Y ua,1 ...
```

Echo after 3 s, the network hangs up after 20 s:

```sh
OSMO_CONSOLE_DELAY=3 OSMO_CONSOLE_HANGUP=20 amps -k 354 -T CC/PC/VC ...
```

Under `sudo` (rpitx), pass them inside the command:
`sudo sh -c "OSMO_CONSOLE_DELAY=3 nmt ... 2>&1 | tee /home/pi/work/log"`.

## How the measurements were done (2026-11-23)

- **Loop with one phone:** `OSMO_CONSOLE_DELAY` plus the per-second `LEVEL`
  logs of NMT and AMPS (what the phone sends, compressed, against what goes
  back to it) gave the loss per speech level (quiet, normal, loud) and the
  limiter actions.
- **Whole path:** a multitone file (speech-like spectrum, three levels) played
  with `OSMO_CONSOLE_SPEECH` and recorded with `OSMO_CONSOLE_RECORD`: the
  phone lying on the table returns its own earpiece through its microphone,
  which gives the echo return per frequency.
- **Echo suppressor:** recorded speech to the phone, first with the phone on
  the table (echo only), then with the user reading over it (double talk).

Read `rec.raw` with numpy: `np.fromfile("rec.raw", np.int16).reshape(-1, 2)`.
