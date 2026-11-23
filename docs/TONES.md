# Tones and announcements

Every system loads the tone set of its network (`-t list` prints it with the
length of each tone and when its phones can hear tones):

| System | Tone set |
|---|---|
| NMT | by the traffic area country (`-Y`): `denmark`, `sweden`, `norway`, `finland`; other countries `denmark` |
| AMPS | `american` |
| TACS / E-TACS | `uk` |
| JTACS | `japan` |
| C-Netz, B-Netz, A-Netz | German sets |
| Radiocom 2000 | `france` |

The NMT sets follow NMT Doc 450-2/900-2 (1995) 9.6, all at 425 Hz:

| Country | Ringing tone | Busy tone | Congestion tone |
|---|---|---|---|
| Denmark | 1 s / 4 s | 0.25 s / 0.25 s | 0.25 s / 0.25 s |
| Finland | 1 s / 4 s | 0.3 s / 0.3 s | 0.3 s / 0.3 s |
| Norway | 1 s / 4 s | 0.5 s / 0.5 s | 0.25 s / 0.25 s |
| Sweden | 1 s / 5 s | 0.25 s / 0.25 s | 0.25 s / 0.75 s |

Dial tone is continuous; the special information tone is the CCITT one. Only
the Danish set has a recorded "out of order" announcement.

## Directions

| Option | Direction | What is played |
|---|---|---|
| `-t 1` (default) | towards the network caller | ringback while the phone rings; the tone of the cause when the phone does not answer or the call to it fails |
| `--mobile-tones 1` (default) | towards the phone | ringback while the network alerts without sending audio; the tone of the cause when the network refuses or ends the phone's call without sending tones |

Towards the phone, the radio system decides whether the phone can hear audio
at that moment. NMT can from address complete on. AMPS/TACS can only on the
voice channel: during set-up a refusal becomes a Reorder or Intercept order,
and the phone makes the tone itself. A normal hang-up after the answer (cause
16) plays nothing and clears the phone at once.

## Length

A tone we play is repeated `--tone-repeats` times (default 3; the cycle of
tone and pause is measured from the tone set), at most `--tone-timeout`
seconds (default 30), then the call clears. Tones sent by the network itself
have no known length and are waited for up to `--tone-timeout`.

In phone-to-phone mode (`-x`) the other leg plays the tone towards the
calling phone (`-t`), e.g. the Danish "out of order" announcement when the
called phone does not answer.
