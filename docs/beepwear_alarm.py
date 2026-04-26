#!/usr/bin/env python3
"""Generate a BeepwearPRO alarm command message.

Usage:
    python beepwear_alarm.py [--disarm] --hour HH --min MM [--month MM] [--day DD] --msg TEXT
"""

import argparse
import sys


def nibble_encode(value: int) -> str:
    """Encode a byte (0-255) as two chars using BeepwearPRO nibble encoding (nibble + 0x30)."""
    hi = (value >> 4) & 0x0F
    lo = value & 0x0F
    return chr(hi + 0x30) + chr(lo + 0x30)


def make_alarm(arm: bool, hour: int, minute: int, month: int, day: int, msg: str) -> str:
    """Build a complete BeepwearPRO alarm command string."""
    msg = msg.upper()[:16]

    body = (
        "A1"
        + ("1" if arm else "0")
        + nibble_encode(hour)
        + nibble_encode(minute)
        + nibble_encode(month)
        + nibble_encode(day)
        + msg
    )

    checksum = sum(ord(c) for c in body) % 256
    return "///" + body + nibble_encode(checksum)


def main():
    parser = argparse.ArgumentParser(
        description="Generate a BeepwearPRO alarm command message"
    )
    parser.add_argument("--disarm", action="store_true", help="Disarm (default: arm)")
    parser.add_argument("--hour", type=int, required=True, help="Hour (0-23)")
    parser.add_argument("--min", type=int, required=True, help="Minute (0-59)")
    parser.add_argument("--month", type=int, default=0, help="Month (0=daily/monthly, 1-12=yearly)")
    parser.add_argument("--day", type=int, default=0, help="Day (0=daily, 1-31)")
    parser.add_argument("--msg", required=True, help="Alarm text (max 16 chars)")
    args = parser.parse_args()

    if not (0 <= args.hour <= 23):
        sys.exit("Error: hour must be 0-23")
    if not (0 <= args.min <= 59):
        sys.exit("Error: minute must be 0-59")
    if not (0 <= args.month <= 12):
        sys.exit("Error: month must be 0-12")
    if not (0 <= args.day <= 31):
        sys.exit("Error: day must be 0-31")

    result = make_alarm(
        arm=not args.disarm,
        hour=args.hour,
        minute=args.min,
        month=args.month,
        day=args.day,
        msg=args.msg,
    )
    print(result)


if __name__ == "__main__":
    main()
