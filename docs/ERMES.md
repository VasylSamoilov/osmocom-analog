# ERMES

`ermes` is an ERMES paging network (ETS 300 133): batches locked to UTC, the
system information with the time and date, individual pages (tone, numeric,
alphanumeric), message numbers, urgent pages, alert types, priorities,
retransmission and long messages over several batches. One or more channels
(`-k`) form one network with one queue.

References below are to ETS 300 133-4 (air interface, edition 2, 1997) unless
another part is named.

## Running

```
ermes -k 9 --sdr-uhd --sdr-samplerate 960000 -s 48000 --sdr-tx-gain 50
```

Channel numbers are those of the 1997 edition, 1..16 (169.400 + n * 0.025
MHz). Pagers and documents of the 1992 edition count from 0: their channel 8
is `-k 9` (169.625 MHz).

The network identity defaults to zone 2, country 1, operator 2, paging area 0:
the factory setting of NEC ERMES pagers. A pager locks only to its home
network (`--zone`, `--country`, `--operator`, `--pa`).

Pages come from the FIFO (`--fifo`), a file (`--fifo-file`) or `--page`, one
per line, as flex and pocsag:

```
address,type,options,message
nec-C63900,alpha,priority=1 urgent=1,Call home
```

`ermes -h` lists all options. The type defaults to tone (`-y`), and a tone
page drops its text: give `alpha` or `numeric` for a page with text.

Options can also go in `~/.osmocom/analog/ermes.conf`, one per line, as for
the other programs.

## Paging a pager: the short way

1. Find the pager's address: the code on its label (NEC: `CODE C63900`).
2. Find its channel: the frequency it is programmed for (see Channels). A
   pager programmed for "channel 8" in the 1992 numbering is `-k 9`.
3. Find its home network: zone, country, operator. NEC pagers leave the
   factory with 2-1-2, the default here.
4. Start the network and wait until the pager shows the network lock.
5. Send a page:

```
echo 'nec-C63900,alpha,,Hello' > /tmp/ermes_msg_send
```

If the pager does not lock, the network or the channel is wrong, or its FSN
is not carried (see Several channels).

## Addresses

A pager is called by its 22-bit local address (ETS 300 133-4 §3.1). The
address can be given three ways; all three below are the same pager:

| Form          | Example      | Meaning                                    |
|---------------|--------------|--------------------------------------------|
| decimal       | `3247680`    | the local address, 0..4194303              |
| hex           | `0x318E40`   | the local address, 0x0..0x3FFFFF           |
| NEC label     | `nec-C63900` | NEC "Tokyo hex": the local address * 4     |

NEC pagers show and program the address in "Tokyo hex", the local address
shifted left by two bits. The code on the label is this value. A `nec-` code
must be a multiple of 4 (its two lowest bits zero) and at most `FFFFFC`:
`nec-C63901` is refused.

A decimal address may have leading zeros (`0000083` is 83, as the console
dials 7 digits); hex needs `0x`.

The parts of a local address:

| Bits | Part            | What it does                                      |
|------|-----------------|---------------------------------------------------|
| 18   | initial address | sent in the address partition of the batch (§5.4) |
| 4    | batch type      | the lowest 4 bits, 0..15 = batch A..P             |

The pager wakes only in its own batch (§4.1): its initial address is sent
only there. The message header then carries the full 22-bit local address
(§5.5.1 Fig.10).

The radio identity code (RIC, §3.1 Fig.1) is the full identity of a receiver,
35 bits: zone (3), country (7), operator (3), initial address (18), batch
number (4). It is never sent as one word: zone, country and operator are
the network (system information), the initial address goes in the address
partition, the batch number is the batch the page is sent in.

Worked example, the NEC label `C63900`, computed by the program
(`ermes_parse_address`, `ermes_print_address`):

| Step                              | Value                                  |
|-----------------------------------|----------------------------------------|
| label (Tokyo hex)                 | `C63900`                               |
| local address = label / 4         | `0x318E40` = 3247680                   |
| batch = lowest 4 bits             | `0x0` = batch A                        |
| initial address = upper 18 bits   | `0x318E4` = 202980                     |
| RIC with network 2-1-2            | `010 0000001 010 110001100011100100 0000` |

The log shows every address as `0x318E40 (NEC C63900), batch A`.

## Network

The network identity is sent in every batch (§5.3.1, Fig.5). A pager locks
only to its home network: the zone, country and operator it was programmed
with must match.

| Option       | Bits | Range | Default | Sent in                               |
|--------------|------|-------|---------|---------------------------------------|
| `--zone`     | 3    | 0..7  | 2       | SSI type 0000 (zone/hour/date)        |
| `--country`  | 7    | 0..127| 1       | system information, every batch       |
| `--operator` | 3    | 0..7  | 2       | system information, every batch       |
| `--pa`       | 6    | 0..63 | 0       | system information (paging area)      |
| `--fsi`      | 5    | 0..30 | 30 (one channel) | system information (see Several channels) |

Zone and country together are the 3-digit country number of ITU-T E.212
annex A: the first digit is the zone, the last two the country (§5.3.1.1).
The defaults, 2-1-2 and paging area 0, are the factory setting of NEC ERMES
pagers, so a pager as delivered locks without options. The values may be
given in decimal or 0x hex.

Not settable, always 0: the border area indicator (BAI, §9.3) and the
external traffic indicator (ETI, §9.4). The network is a home network only.

Observed with an NEC PA3E6-1A: no lock with country 4 (2-4-2), lock with its
home 2-1-2.

## Channels

Channels are numbered as in the 1997 edition (§8.2): channel n is
169.400 + n * 0.025 MHz, n = 1..16. The 1992 edition and early pagers count
from 0 at 169.425 MHz: their channel n is `-k n+1` here. `ermes -k list`
prints the table.

The batches are shifted from channel to channel (§4.1 Fig.3), so the time
slot of a batch type depends on the channel. The program does this.

| `-k` | Frequency (MHz) | 1992 number | First batch in a subsequence |
|------|-----------------|-------------|------------------------------|
| 1    | 169.425         | 0           | A                            |
| 2    | 169.450         | 1           | B                            |
| 3    | 169.475         | 2           | P                            |
| 4    | 169.500         | 3           | C                            |
| 5    | 169.525         | 4           | O                            |
| 6    | 169.550         | 5           | D                            |
| 7    | 169.575         | 6           | N                            |
| 8    | 169.600         | 7           | E                            |
| 9    | 169.625         | 8           | M                            |
| 10   | 169.650         | 9           | F                            |
| 11   | 169.675         | 10          | L                            |
| 12   | 169.700         | 11          | G                            |
| 13   | 169.725         | 12          | K                            |
| 14   | 169.750         | 13          | H                            |
| 15   | 169.775         | 14          | J                            |
| 16   | 169.800         | 15          | I                            |

Observed: the NEC PA3E6-1A with home channel 8 received on 169.625 MHz
(`-k 9`), not on 169.600 MHz (`-k 8`).

## Sending messages

Pages come from four sources:

| Source             | How                                                        |
|--------------------|------------------------------------------------------------|
| FIFO               | a named pipe, `/tmp/ermes_msg_send` by default (`--fifo <path>`); write one line per page |
| `--fifo-file <f>`  | the lines of a file instead of the FIFO; the program ends when all are sent |
| `--page <line>`    | one page, queued `--page-delay` seconds after start (default 30), so the pager can find the channel first; up to 16 |
| call control       | the console (type the 7-digit local address, then `d`) or the Osmo-CC socket (`-o`) |

A line has four fields, separated by commas:

```
address,type,options,message
```

- `address`: as in Addresses (`3247680`, `0x318E40`, `nec-C63900`).
- `type`: `tone`, `numeric` or `alpha` (or `t`, `n`, `a`). Empty: `-y`.
- `options`: space-separated `key=value` pairs, may be empty.
- `message`: the rest of the line, commas included. Missing: `-M`.

Only the address is required. Empty lines are skipped.

Options of a line, and the command-line option that sets the default for
lines without them:

| Key                   | Range | Default | Command line                 | What it does |
|-----------------------|-------|---------|------------------------------|--------------|
| `priority`            | 1..3  | 2       | `--priority`                 | queue order: 1 before 2 before 3, arrival order within one priority (ETS 300 133-2 §4.3.2); nothing on the air |
| `urgent`              | 0..1  | 0       | `--urgent` (sets 1)          | urgent message indicator, UMI (§5.5.1.1 Table 9) |
| `alert`               | 0..7  | 0       | `--alert`                    | alert type (Table 9); what each type sounds like is up to the pager |
| `charset`             | a codepage name | `--charset` | `--charset`       | codepage of an alphanumeric message (see Codepages) |
| `retransmit`          | 0..15 | 0       | `--retransmit`               | times the page is sent again after the first, with the same number (ETS 300 133-2 §4.3.1.3.1) |
| `retransmit_interval` | 1..3600 | 300   | `--retransmit-interval`      | seconds between the transmissions |

Other command-line defaults: `-y` the type (default tone), `-M` the text
(default empty). A command-line default applies to every page that does not
set the key itself, whatever its type.

A line with an error (bad address, type, option or a character without a
code) is not queued; the log says why.

Examples:

```
nec-C63900,alpha,,Call home
nec-C63900,numeric,,0123 456-789
nec-C63900,tone
nec-C63900,tone,urgent=1 alert=3
0x318E40,alpha,priority=1 urgent=1,Meeting at 10, room 2
3247680,alpha,retransmit=2 retransmit_interval=60,Sent three times, one number
```

Write to the FIFO from a shell:

```
echo 'nec-C63900,alpha,,Hello' > /tmp/ermes_msg_send
```

or put the lines in a file and run with `--fifo-file pages.txt`.

Call control: the console dials exactly 7 digits, the local address in
decimal; it pages with type `-y` and the text `-M`. Addresses below 1000000
are dialled with leading zeros (`0000083`). Over the Osmo-CC socket the
called number is the address and the caller ID, if any, is the text.

### Message numbers

Every page to an address gets the next message number, 1..31, then 1 again
(ETS 300 133-2 §4.3.1.3; §5.5.1 Table 8). The pager checks the sequence and
reports a missing number. The number is given when the page is first sent,
not when it is queued: the numbers of an address go up on the air whatever
the priorities. A retransmission keeps its number. The last number of every
address is kept in the subscriber book across runs.

### Long messages

A message holds up to 1023 bytes of text (UTF-8: a character outside ASCII
takes 2 or 3 bytes). When it does not fit in its batch, it runs on into the
next batches, right after their address partition (§10.5). One batch in time
slots 0..14 holds about 359 alphanumeric characters, the long last slot 451
(computed by the batch builder, one address in the batch).

Observed with the NEC PA3E6-1A: 500 characters over two batches and 1000
characters over three arrived whole.

### Several messages to one pager

The standard allows any number of messages to one receiver in a batch, one
initial address each (§10.2); a batch has room for about 35 tone pages or 15
short texts. The network sends as many to the same receiver in one batch as
fit, numbered in order; the rest follow in its next batches, 12 s apart.
Observed with the NEC PA3E6-1A: 4 in one batch all arrived.

### Group calls

One message to several pagers, as flex (a space-separated address list):

```
tempgroup:nec-C63900 0x00001C 0x000127,alpha,,Meeting at 10
```

The message is sent once (ETS 300 133-4 §10.3), in three stages, one per
batch, one after another for each member:

1. its initial address alone, in its own batch; a group member then searches
   "in subsequent batches" (§10.3), unlike an individual call, which also
   looks in the current one (§10.2);
2. in the next batch, a short header to it: its own next message number and a
   pointer (CTAP 0..15) to a common temporary address (CTA, eighteen ones and
   the pointer);
3. in the batch after the last member's stage 2, the text once to the CTA,
   with message number 0, ended as any message; every member files it under
   the number it was given. The call ends there.

All within 16 batches of the first initial address: the members search only
12 s (§10.8.2). A call starts at a time slot from which its stages fit; a
group whose members' batch types are spread over more than 14 batches in a
row is refused (split it). For one pager in batch A: A, B, C.

Each channel sends one group call at a time and nothing else meanwhile; a
call starts when the individual pages that came before it are sent. The
members get no other message until the call is done: its pointer is held 12 s
after the last member's initial address, then free. The 16 pointers of a
channel each go: free, addressing, message (the text runs on into further
batches), hold, free; the log shows every change. A member is served by the
channel that carries its FSN. At most 32 members; `retransmit=` is refused
for group calls.

A batch that carries a pointer (stage 2), the text (stage 3) or the rest of a
long message continued from the batch before (§10.5) has no initial address
of its own; when no other traffic fills its address partition, it gets a PAD
there: the group initial address, eighteen ones, the highest of
all and nobody's own. The receiver test of group calls has these batches
occupied the same way (ETS 300 133-5 §7.8 Fig.16, TBR 7 §8.15: "PAD: an
initial address, higher than the receiver under test, used to occupy start
of address partition"). Receivers skip a batch whose address partition holds
no initial address: without the PAD the NEC PA3E6-1A lost the end of longer
group texts (or all of them) and then missed its own pages for minutes, and
PDW's decoder showed no group message at all. With it both get the full text
(tested with 40 and 70 characters, one and two members).

## Order of messages in a batch

The initial addresses go highest first (§5.4). For the messages the standard
sets no order, but receivers expect them **lowest local address first**: the
NEC PA3E6-1A stops searching at the first message to a higher address than
its own. Tested on air: its message behind messages to higher addresses was
lost (with 3 to 35 addresses in the batch, tone or alpha), behind messages
to lower addresses it arrived (also as number 18 of 35). So the messages of
a batch go out in ascending order of local address; several to one receiver
keep the order of their numbers. A message that does not fit and runs on into
the next batch must be the last one, so it runs on only when no higher
address follows; otherwise it waits for the receiver's next batch. A batch
takes up to 139 initial addresses (Table 1), as many as fit.

## Message formats

The paging category is sent in the message header (§5.5.1.1 Table 9).

| Type      | Data                                 | Ends with |
|-----------|--------------------------------------|-----------|
| `tone`    | none: an alert only; a text is dropped | - |
| `numeric` | 4 bits a character (ETS 300 133-2 Table B.1) | spaces fill the last codeword |
| `alpha`   | 7 bits a character (Table B.3, or `--charset`) | EOM (DC1) and EOM fill (§5.5.4) |

Numeric characters, the 16 codes of Table B.1:

```
0 1 2 3 4 5 6 7 8 9 / space U - . %
```

A lowercase `u` is sent as `U`. Any other character refuses the page.

Transparent data (category 11) is part of the standard, but not offered:
`-y` and the type field refuse it.

Limits: 1023 bytes of text per message (above), numeric characters only in
numeric pages, codepage characters only in alphanumeric pages.

## Codepages

Alphanumeric pages are coded with a codepage of 128 characters (7 bits). The
ERMES codepages:

| Name                         | Use |
|------------------------------|-----|
| `generic-generic-standard`   | the ERMES character set, ETS 300 133-2 Table B.3: the default |

Numeric pages have no codepage: numeric is a message type with one fixed set
of 16 characters (Table B.1, see Message formats), as in the other paging
protocols.

- `--charset list` lists them.
- `--charset-print <name>` prints a table, e.g.
  `ermes --charset-print generic-generic-standard`.
- `--charset <name>` sets the codepage of alphanumeric pages.
- `charset=<name>` in a line sets it for that page.

The standard table has ASCII letters, digits and most signs, LF and CR, and
`@ £ $ ¥ è é ù ì ò ç Ø ø Å å Δ Φ Γ Λ Ω Π ψ Σ θ Ξ Æ æ ß É ¤ ¡ Ä Ö Ñ Ü § ¿ ä ö
ñ ü à`. It has no `~`, `{`, `}`, `[`, `]`, `\`, `^`, `_`, `` ` `` or `|`, and no
Cyrillic. A page with a character that is not in its codepage is refused.

## Time and date

The network sends the time in the supplementary system information (SSI,
§5.3.2): type 0000 zone/hour/date and type 0001 day/month/year (Fig.7,
Fig.8). Hour and date are local time; the zone is `--zone`. The batches
are locked to UTC: the cycle number (0..59) is the UTC minute, a
subsequence starts every 12 s (§4.1).

| Option | Default | What it does |
|--------|---------|--------------|
| `--ssi hold` | hold | the network schedule: the hour starts with 0000 in every batch of its first subsequence, 0001 in the next, and so on by turns. This is what the Swiss (1997) and Dutch (2014) networks sent. |
| `--ssi-hold <n>` | 1 | subsequences each type is held (5 = one minute, for tests) |
| `--year-from <year>` | the real year | send the year moved by 28-year steps into `<year>`..`<year>+27`, for pagers whose calendar ends early; weekday and date stay right. Range 1990..2090. E.g. 1995: 2026 is sent as 1998. |
| `--clock "YYYY-MM-DD HH:MM:SS"` | the system clock | run the network clock from this local time |
| `--tx-latency <ms>` | the DSP buffer, `-b` (50 ms) | time from making a sample to sending it, so the batches are on time on the air |

The other `--ssi` modes (`alternate`, `strict`, `pairs`, `cycle`,
`zone-hour-date`, `day-month-year`) are for receiver tests only, not for a
network. `cycle` sends the 13 SSI types of edition 2; edition 1 pagers
cannot use it.

The network behaves as an I1 version 01.00 network (§4.6): only SSI types
0000 and 0001, no version (0010).

What the NEC PA3E6-1A does with the time, as observed on the air (not from
the standard, which does not say when a receiver takes the time):

- Before the first lock it shows `--:--`.
- At acquisition it takes the minute from the cycle number and one SSI
  value: 0000 gives the hour (the weekday stays Mo), 0001 the weekday (the
  hour stays 0). Its own clock then runs on.
- The hour appeared complete only after the network had run across a full
  hour (three cases). Most likely it takes the hour at the top of the hour,
  when subsequence 0 carries 0000; not confirmed.
- The 13-type `cycle` gave it neither value.

## Subscriber book

`~/.osmocom/analog/ermes_book` (`--book`) keeps per receiver what the network
must know, one line each:

```
0x318E40 number=26 fsn=8
```

- `number`: the last message number sent (1..31). The pager checks the
  sequence and reports missing messages, so it is kept across runs.
- `fsn`: the receiver's frequency subset number (0..15, below). Without it a
  page goes on the first channel with room.

## Several channels

```
ermes -k 9 -k 12 ...                  # FSI 29 and 28, as the standard says
ermes -k 3 --fsi 25 -k 9 --fsi 26 ... # more channels: give an FSI each
```

Every channel sends a frequency subset indicator (FSI, §9.2 Fig.18): the
receivers it carries, by their frequency subset number (FSN 0..15) stored in
the pager:

| FSI    | FSNs carried                 |
|--------|------------------------------|
| 30     | all (single channel network) |
| 28, 29 | 0-7, 8-15                    |
| 24..27 | four each: 0-3 ... 12-15     |
| 16..23 | two each: 0-1 ... 14-15      |
| 0..15  | that one FSN                 |

A pager locks to a channel only if its FSI carries the pager's FSN (§9.5.2).
The channels' subsets must not overlap. `-P` (polarity) is given once, or
once per channel.

## Scenario: finding a pager's FSN

The FSN is set when the pager is programmed and is often not printed on it.
The pager tells it by locking or not: run one channel with one FSI at a time,
reset the pager (battery out and in) and see whether it shows the network
lock. Halve the range each time, as a binary search over Fig.18:

1. `--fsi 29` (FSN 8-15). Lock: the FSN is 8-15, else 0-7 (check with
   `--fsi 28`).
2. Within the half: the FSI of the upper four (`27` for 12-15, `25` for
   4-7). Lock: those four, else the lower four.
3. The FSI of the upper two of the four (16..23), then the single FSN
   (0..15).
4. Confirm: the single FSN must lock.

Each step:

```
ermes -k 9 --fsi <FSI> --sdr-uhd --sdr-samplerate 960000 -s 48000 --sdr-tx-gain 50
```

Give each step about a minute after the reset; a pager whose FSN is carried
locks within seconds. If it locks with 28 and 29 alike, the pager does not
check the FSI.

Example, NEC PA3E6-1A (2026-11-27): FSI 29 lock, 28 none, 27 none, 21 none,
9 none, 8 lock: FSN 8. Then in the book: `0x318E40 fsn=8`.
