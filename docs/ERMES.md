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
nec:C63900,alpha,priority=1 urgent=1,Call home
```

`ermes -h` lists all options.

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
