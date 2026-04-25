# BeepwearPRO Pager Watch — Alarm Command Protocol

## Overview

The BeepwearPRO pager watch receives commands via standard FLEX alphanumeric pages.
The watch firmware distinguishes control messages from display messages by the `///`
prefix. What follows is a plain-text encoded command string — no binary framing,
no base64, just printable ASCII characters sent as a normal FLEX alpha page.

This document describes the Alarm command (`A`), reverse-engineered from the
`sendpageV2` CGI endpoint at `beepwear.com`.

## Transport

```
GET /cgi-bin/sendpageV2?pin=<PIN>&msg=<URL-encoded message>&provider=<carrier>
```

- `pin` — Subscriber PIN (mapped to a FLEX capcode server-side)
- `msg` — The command string (URL-encoded)
- `provider` — Paging carrier name (e.g. `AirTouch`)

The gateway resolves the PIN to a FLEX capcode (e.g. `0112210`) and transmits
the message content as a standard alphanumeric FLEX page.

## Nibble Encoding

All numeric fields use a consistent single-character encoding per 4-bit value:

```
Value:    0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
Char:    '0' '1' '2' '3' '4' '5' '6' '7' '8' '9' ':' ';' '<' '=' '>' '?'
```

The formula is simply `char = value + 0x30`. This is NOT standard hexadecimal
(which uses `A-F` for 10-15). Instead it uses the ASCII characters immediately
after `'9'`, which are `: ; < = > ?`.

A byte value (0-255) is encoded as two characters: high nibble first, then low nibble.

Examples:
- 23 = 0x17 → `'1'` `'7'`
- 59 = 0x3B → `'3'` `';'`
- 28 = 0x1C → `'1'` `'<'`
- 12 = 0x0C → `'0'` `'<'`

## Alarm Command Format

```
/// A 1 <arm> <hh> <hl> <mh> <ml> <Mh> <Ml> <dh> <dl> <MESSAGE> <ch> <cl>
```

| Field | Chars | Description |
|-------|-------|-------------|
| `///` | 3 | Control escape prefix |
| `A` | 1 | Command type: Alarm |
| `1` | 1 | Fixed value (protocol version or command subtype) |
| arm | 1 | `1` = Arm, `0` = Disarm |
| hh, hl | 2 | Hour (0-23), two nibble-encoded characters |
| mh, ml | 2 | Minute (0-59), two nibble-encoded characters |
| Mh, Ml | 2 | Month (0-12), two nibble-encoded characters |
| dh, dl | 2 | Day of month (0-31), two nibble-encoded characters |
| MESSAGE | 1-16 | Alarm text, uppercase ASCII |
| ch, cl | 2 | Checksum, two nibble-encoded characters |

Total header after `///`: 1 (cmd) + 1 (fixed) + 1 (arm) + 2 (hour) + 2 (min) + 2 (month) + 2 (day) = 11 characters, followed by the message (max 16 chars) and 2-char checksum.

## Alarm Type

The alarm type is implicit from the month and day fields:

| Month | Day | Alarm Type |
|-------|-----|------------|
| 0 | 0 | Daily |
| 0 | 1-31 | Monthly (on that day) |
| 1-12 | 1-31 | Yearly (on that month/day) |

## Checksum

The checksum covers all characters from `A` (inclusive) through the end of the
message text (before the checksum itself):

```
checksum = sum(ASCII value of each character from 'A' to end of MESSAGE) mod 256
```

The result (0-255) is encoded as two nibble characters appended to the message.

## Worked Examples

### Example 1 — Daily, Arm, 07:00, "Wake up"

```
Input:  Arm=yes, Hour=7, Min=0, Month=0, Day=0, Msg="Wake up"
Hour:    7 = 0x07 → '0' '7'
Min:     0 = 0x00 → '0' '0'
Month:   0 = 0x00 → '0' '0'
Day:     0 = 0x00 → '0' '0'
Body:   A 1 1 0 7 0 0 0 0 0 0 W A K E   U P
Sum:    65+49+49+48+55+48+48+48+48+48+48+87+65+75+69+32+85+80 = 1047
Check:  1047 mod 256 = 23 = 0x17 → '1' '7'
Result: ///A1107000000WAKE UP17
```

### Example 2 — Monthly 15th, Arm, 10:00, "Call Mom"

```
Input:  Arm=yes, Hour=10, Min=0, Month=0, Day=15, Msg="Call Mom"
Hour:   10 = 0x0A → '0' ':'
Min:     0 = 0x00 → '0' '0'
Month:   0 = 0x00 → '0' '0'
Day:    15 = 0x0F → '0' '?'
Body:   A 1 1 0 : 0 0 0 0 0 ? C A L L   M O M
Sum:    65+49+49+48+58+48+48+48+48+48+63+67+65+76+76+32+77+79+77 = 1121
Check:  1121 mod 256 = 97 = 0x61 → '6' '1'
Result: ///A110:00000?CALL MOM61
```

### Example 3 — Yearly 12/25, Arm, 00:01, "Merry Xmas"

```
Input:  Arm=yes, Hour=0, Min=1, Month=12, Day=25, Msg="Merry Xmas"
Hour:    0 = 0x00 → '0' '0'
Min:     1 = 0x01 → '0' '1'
Month:  12 = 0x0C → '0' '<'
Day:    25 = 0x19 → '1' '9'
Body:   A 1 1 0 0 0 1 0 < 1 9 M E R R Y   X M A S
Sum:    65+49+49+48+48+48+49+48+60+49+57+77+69+82+82+89+32+88+77+65+83 = 1314
Check:  1314 mod 256 = 34 = 0x22 → '2' '2'
Result: ///A1100010<19MERRY XMAS22
```

## Notes

- The message text is always transmitted in uppercase regardless of input case.
- Maximum message length is 16 characters.
- The `///` prefix is a BeepwearPRO firmware convention, not part of the FLEX standard.
- Other command types beyond `A` (Alarm) likely exist but are not documented here.
