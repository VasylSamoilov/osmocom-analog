# Sharp Zaurus Z-Pager — POCSAG Message Format

## Device

Sharp Zaurus 2MB with Z-Pager application, receiving POCSAG alphanumeric messages.

## Message Format

The Z-Pager parses incoming POCSAG alphanumeric messages using `|` (pipe) as a field delimiter:

```
from_value|subject_value|body_text
```

The pager's UI adds its own "From:", "Subject:", and "Message:" labels — do NOT include those keywords in the message content or they will be doubled.

### Examples

```
Dispatch|Fire Alert|All units respond to 742 Evergreen Terrace.
Ops|Ping|Status OK
Scheduler|Shift Change|You are reassigned to Zone 4 effective 1800.
```

### Plain messages (no fields)

If no `|` delimiters are present, the entire message appears in the body/message area.

## Field Limits

| Field   | Max Characters | Notes                                    |
|---------|---------------|------------------------------------------|
| From    | 27            | Truncated on display beyond 27 chars     |
| Subject | 30            | Truncated on display beyond 30 chars     |
| Body    | No fixed limit| Limited by total message size             |
| **Total message** | **2019** | Pager card becomes inaccessible at 2020+ |

## Control Characters

The Z-Pager does NOT render POCSAG control characters. Characters like `<CR>`, `<LF>`, `<HT>`, `<BEL>` are displayed as `?` on screen. Use `|` pipe delimiters instead for field separation.

## FIFO Format

When sending via the osmocom-analog POCSAG transmitter FIFO:

```
CAPCODE+FUNC,alpha,,from_value|subject_value|body_text
```

Example:
```
0001101A,alpha,,Dispatch|Fire Alert|All units respond immediately.
```

## Tested Capcodes

- 0001101 (functions A-D)
- 0001407 (functions A-D)

## Safety Notes

- Messages exceeding 2019 characters total will crash/lock the pager card
- The pager card becomes inaccessible and requires a reset
- Always stay under 2000 characters total for safety margin
