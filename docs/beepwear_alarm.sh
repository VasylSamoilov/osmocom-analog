#!/usr/bin/env bash
# Generate a BeepwearPRO alarm command message
# Usage: ./beepwear_alarm.sh [--disarm] --hour HH --min MM [--month MM] [--day DD] --msg TEXT

set -euo pipefail

ARM=1
HOUR=""
MIN=""
MONTH=0
DAY=0
MSG=""

usage() {
    cat <<EOF
Usage: $0 [--disarm] --hour HH --min MM [--month MM] [--day DD] --msg TEXT

  --disarm      Disarm the alarm (default: arm)
  --hour HH     Hour (0-23)
  --min  MM     Minute (0-59)
  --month MM    Month (0=daily/monthly, 1-12=yearly) [default: 0]
  --day   DD    Day (0=daily, 1-31=monthly/yearly)   [default: 0]
  --msg  TEXT   Alarm text (max 16 chars, will be uppercased)

Alarm type is implicit:
  month=0, day=0    → Daily
  month=0, day=1-31 → Monthly (on that day)
  month=1-12        → Yearly (on that month/day)

Examples:
  $0 --hour 7 --min 0 --msg "Wake up"
  $0 --hour 10 --min 0 --day 15 --msg "Call Mom"
  $0 --hour 0 --min 1 --month 12 --day 25 --msg "Merry Xmas"
EOF
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --disarm) ARM=0; shift ;;
        --hour)   HOUR="$2"; shift 2 ;;
        --min)    MIN="$2";  shift 2 ;;
        --month)  MONTH="$2"; shift 2 ;;
        --day)    DAY="$2";  shift 2 ;;
        --msg)    MSG="$2";  shift 2 ;;
        *)        usage ;;
    esac
done

[[ -z "$HOUR" || -z "$MIN" || -z "$MSG" ]] && usage

# Uppercase the message and truncate to 16 chars
MSG=$(echo "$MSG" | tr '[:lower:]' '[:upper:]' | cut -c1-16)

# Nibble-encode a byte value (0-255) as two characters: value + 0x30 per nibble
nibble_encode() {
    local val=$1
    local hi=$(( (val >> 4) & 0x0F ))
    local lo=$(( val & 0x0F ))
    printf "\\x$(printf '%02x' $((hi + 0x30)))\\x$(printf '%02x' $((lo + 0x30)))"
}

ENC_HOUR=$(nibble_encode "$HOUR")
ENC_MIN=$(nibble_encode "$MIN")
ENC_MONTH=$(nibble_encode "$MONTH")
ENC_DAY=$(nibble_encode "$DAY")

# Build the body (everything after "///")
BODY="A1${ARM}${ENC_HOUR}${ENC_MIN}${ENC_MONTH}${ENC_DAY}${MSG}"

# Checksum: sum of ASCII values of all chars in BODY, mod 256
SUM=0
for (( i=0; i<${#BODY}; i++ )); do
    CHAR="${BODY:$i:1}"
    VAL=$(printf '%d' "'$CHAR")
    SUM=$(( SUM + VAL ))
done
CHK=$(( SUM % 256 ))
ENC_CHK=$(nibble_encode "$CHK")

RESULT="///${BODY}${ENC_CHK}"

echo "$RESULT"
