#!/bin/bash
#
# GSC/Golay RX test script
#
# Sends a comprehensive set of messages via the FIFO to exercise all
# message types, address ranges, payload boundaries, and character sets.
#
# Usage: start golay in TX+RX loopback mode, then run this script.
#   ./golay -k 161.2 -T -R --loopback &
#   sleep 2
#   ./test_messages.sh
#
# FIFO format: <7-digit address>,<type prefix>,<payload>
#   type prefix: a, = alpha  n, = numeric  v, = voice  (none) = auto/tone
#
# Address format: I G1 G0 A2 A1 A0 F
#   F suffix: 1-4 = voice, 5-8 = alpha, 9/0 = tone-only
#
# Payload limits:
#   Alpha:   80 chars max (10 blocks × 8 chars)
#   Numeric: 24 digits max (2 blocks × 12 nibbles)

FIFO="/tmp/golay_msg_send"
DELAY=3  # seconds between messages — enough for TX + decode

if [ ! -p "$FIFO" ]; then
    echo "ERROR: FIFO $FIFO not found. Start golay with -T first."
    exit 1
fi

send() {
    local desc="$1"
    local msg="$2"
    echo "--- $desc"
    echo "$msg" > "$FIFO"
    sleep "$DELAY"
}

echo "=== GSC/Golay TX/RX Test Suite ==="
echo ""

# ============================================================
# 1. TONE-ONLY messages (suffix 9 or 0)
# ============================================================
echo "=== TONE-ONLY ==="

# Min address: index=0, G1G0=00, A2A1A0=001, suffix=9
send "Tone: min cap code (0000019)" \
     "0000019"

# Max address: index=9, G1G0=49, A2A1A0=999, suffix=0
send "Tone: max cap code low range (9499990)" \
     "9499990"

# High range: G1G0=50, A2A1A0=001, suffix=9
send "Tone: high range (0500019)" \
     "0500019"

# Max high range: index=9, G1G0=99, A2A1A0=999, suffix=0
send "Tone: max cap code high range (9999990)" \
     "9999990"

# Normal mid-range address
send "Tone: normal address (1234569)" \
     "1234569"

# ============================================================
# 2. ALPHA messages (suffix 5-8)
# ============================================================
echo ""
echo "=== ALPHA ==="

# --- Payload boundaries ---

# 1 character (minimum payload)
send "Alpha: 1 char, min cap code (0000015,a,X)" \
     "0000015,a,X"

# 8 characters (exactly 1 block)
send "Alpha: 8 chars = 1 block (1234567,a,ABCDEFGH)" \
     "1234567,a,ABCDEFGH"

# 9 characters (boundary: 2 blocks, contbit transition)
send "Alpha: 9 chars = 2 blocks (1234567,a,ABCDEFGHI)" \
     "1234567,a,ABCDEFGHI"

# 16 characters (exactly 2 blocks)
send "Alpha: 16 chars = 2 blocks full (1234567,a,ABCDEFGHIJKLMNOP)" \
     "1234567,a,ABCDEFGHIJKLMNOP"

# 80 characters (maximum: 10 blocks)
send "Alpha: 80 chars = max payload (1234567,a,ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !\"#\$%&()*+,-./:;<=>?@ABCDEFGHIJKLMNOPQRST)" \
     "1234567,a,ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !\"#\$%&()*+,-./:;<=>?@ABCDEFGHIJKLMNOPQRST"

# 81 characters (overflow — should be cropped to 80)
send "Alpha: 81 chars = overflow crop (1234567,a,ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !\"#\$%&()*+,-./:;<=>?@ABCDEFGHIJKLMNOPQRSTU)" \
     "1234567,a,ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !\"#\$%&()*+,-./:;<=>?@ABCDEFGHIJKLMNOPQRSTU"

# --- Full character set ---
# All printable alpha chars: space through ] (0x20-0x5D) plus special: CR, _, [, ]
send "Alpha: full charset (1234568,a, !\"#\$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[_])" \
     "1234568,a, !\"#\$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[_]"

# --- Address variations ---

# Max cap code with alpha
send "Alpha: max cap code low (9499995,a,MAXLOW)" \
     "9499995,a,MAXLOW"

# Max cap code high range
send "Alpha: max cap code high (9999996,a,MAXHIGH)" \
     "9999996,a,MAXHIGH"

# Different function suffixes (5,6,7,8 = alpha with function 0,1,2,3)
send "Alpha: function 0 suffix 5 (1234565,a,FN0)" \
     "1234565,a,FN0"

send "Alpha: function 1 suffix 6 (1234566,a,FN1)" \
     "1234566,a,FN1"

send "Alpha: function 2 suffix 7 (1234567,a,FN2)" \
     "1234567,a,FN2"

send "Alpha: function 3 suffix 8 (1234568,a,FN3)" \
     "1234568,a,FN3"

# ============================================================
# 3. NUMERIC messages (suffix 5-8 with n, prefix)
# ============================================================
echo ""
echo "=== NUMERIC ==="

# --- Payload boundaries ---

# 1 digit (minimum payload)
send "Numeric: 1 digit (1234565,n,7)" \
     "1234565,n,7"

# 12 digits (exactly 1 block)
send "Numeric: 12 digits = 1 block (1234567,n,123456789012)" \
     "1234567,n,123456789012"

# 13 digits (boundary: 2 blocks, contbit transition)
send "Numeric: 13 digits = 2 blocks (1234567,n,1234567890123)" \
     "1234567,n,1234567890123"

# 24 digits (maximum: 2 blocks full)
send "Numeric: 24 digits = max payload (1234567,n,123456789012345678901234)" \
     "1234567,n,123456789012345678901234"

# 25 digits (overflow — should be cropped to 24)
send "Numeric: 25 digits = overflow crop (1234567,n,1234567890123456789012345)" \
     "1234567,n,1234567890123456789012345"

# --- Full character set ---
# Digits 0-9, U, space, hyphen, asterisk
send "Numeric: all unshifted chars (1234568,n,0123456789U -*)" \
     "1234568,n,0123456789U -*"

# Shifted letters: A B C D E F G H J L N P R
send "Numeric: all shifted chars (1234568,n,ABCDEFGHJLNPR)" \
     "1234568,n,ABCDEFGHJLNPR"

# Mixed shifted and unshifted
send "Numeric: mixed shifted+unshifted (1234567,n,12A34B-5C*6D)" \
     "1234567,n,12A34B-5C*6D"

# --- Address variations ---

# Min cap code
send "Numeric: min cap code (0000015,n,42)" \
     "0000015,n,42"

# Max cap code low range
send "Numeric: max cap code low (9499996,n,999)" \
     "9499996,n,999"

# Max cap code high range
send "Numeric: max cap code high (9999997,n,000)" \
     "9999997,n,000"

# ============================================================
# 4. VOICE message (suffix 1-4)
# ============================================================
echo ""
echo "=== VOICE ==="

send "Voice: normal address (1234561,v,tone1.wav)" \
     "1234561,v,/home/i/fix/osmocom-analog.compandor/tone1.wav"

# ============================================================
# 5. AUTO-TYPE messages (no type prefix — type from suffix)
# ============================================================
echo ""
echo "=== AUTO-TYPE (suffix determines type) ==="

# Suffix 5 = alpha auto
send "Auto: suffix 5 = alpha (1234565)" \
     "1234565"

# Suffix 9 = tone auto
send "Auto: suffix 9 = tone (1234569)" \
     "1234569"

# Suffix 1 = voice auto
send "Auto: suffix 1 = voice (1234561)" \
     "1234561"

echo ""
echo "=== Test suite complete ==="
echo "Check /tmp/golay_msg_received for decoded output."
echo "Compare with expected: address, type, polarity, data."
