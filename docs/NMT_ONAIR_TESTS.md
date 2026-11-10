# NMT on-air test list

Changes made against NMT Doc 450-1/900-1, 450-3 Annex 21 and Annex 24 that
have only passed the unit tests (src/test) so far. Each one needs a check
with a real phone. Tick a box when it was seen working, write down what the
phone did when it was not.

Commits: 3a0a6f4 (frame decoding), eb3eb40 (call procedures), 66e1272
(carousel, band advertisement), 284eb4c (sequences and acceptance), 24f9da4
(DMS), 4d1be60 and 277b0ad (SMS).

## Setup

    nmt -k <cc> -T CC -k <tc1> -T TC -k <tc2> -T TC -Y <traffic area> ...

- Two traffic channels are needed for the queue, the SCIP move and the
  "phone busy" tests.
- The log level should show DNMT, DDMS and DSMS at INFO (the frames and the
  states are named there).
- Commands go to /tmp/nmt_cmd (mwi, move, power), short messages to
  /tmp/nmt_sms_deliver; messages from the phone appear in /tmp/nmt_sms_submit.
- The SMSC number is 767 unless -S says otherwise.
- A second SDR receiver recording the downlink makes the frame sequences and
  the DMS gaps visible.

## Calls

- [ ] Call to the phone: ringing order L(9) is sent 4 times, then idle
      frames. The phone rings normally.
- [ ] Caller id (-I 1) with a number longer than 5 digits: the phone shows
      the whole number. The sequence now starts with M(0).
- [ ] Call from the phone: a digit sent during the two L(3) frames (dial
      very fast) is not lost.
- [ ] Call from the phone with 33 digits: the 33rd is ignored, the call is
      set up with the first 32.
- [ ] Compandor (default on, -C 1): L(5) four times after address complete
      (call from the phone) and after the answer (call to the phone). Speech
      sounds right, no pumping. With -C 0 idle frames instead and plain speech.
- [ ] Roaming update: L(3) only 2 frames, then the release. The phone shows
      it is registered.
- [ ] The phone's clearing L(1) is taken in every state (ringing, dialling,
      conversation, MFT, recall).
- [ ] A phone that already has a call and seizes another traffic channel is
      ignored (log: "has a transaction already"), the first call goes on.

## Calling channel, queue, battery saving

- [ ] Several calls to different phones at once: each is paged (2a), max 3
      pages per calling channel at a time; a 4th call is refused (NOCHANNEL).
- [ ] No free traffic channel when the phone answers: 2f (2c for a priority
      phone) is sent twice, the phone shows it waits. When a traffic channel
      frees, the phone is paged again and gets 2b after its answer.
- [ ] Queue timeout: after 60 s without a free traffic channel the call fails.
- [ ] Battery saving (--battery-saving N): the phone still answers pages;
      the page waits for the wake-up window of its group.
- [ ] NMT-900 band advertisement (--cc-band-start/end, --tc-band-start/end)
      together with battery saving: both frames take turns, the phone finds
      the band and still wakes up.
- [ ] 10c (the called phone finds a traffic channel itself after 2d): only
      taken while the call to that phone is still on the calling channels.

## In-call procedures

- [ ] Power change: echo "power <number>,2" > /tmp/nmt_cmd. The phone
      answers with 10b. NMT-450: new level in Na. NMT-900: new level in N1.
      The level is kept for the call; the next call starts with -P.
- [ ] SCIP move: echo "move <number>" > /tmp/nmt_cmd. Old channel: 3c x4,
      then idle frames x4. The phone appears on the new channel within T'
      (553 ms from the first 3c), the call goes on there, power level kept.
- [ ] SCIP failure (phone does not come, e.g. new channel out of range): 3b
      x4 on the new channel, then L(15) x4 there; the call stays on the old one.
- [ ] MFT: press the MFT button. 3 idle frames, then L(4) x4. Digits are
      heard as DTMF on the far end, also when the first digit comes in 14b.
      MFT button again: L(10) x4, 4 idle frames, speech.
- [ ] MFT forced release: phone stops sending (switch it off in the MFT
      state). After 10 s: L(10) x4, 4 idle frames. The call is released by
      supervision later as usual.
- [ ] Register recall: R button. 3 idle frames, L(3) x4, dial tone; the
      tone stops with the first digit. After 8 s without a digit (30 s
      without any digit): L(6) x4, 4 idle frames, speech again.
- [ ] Supervision: every frame received in conversation restarts the 20 s
      timer; a call where the supervisory tone stays is not dropped.

## Message waiting

- [ ] Phone idle: echo "mwi <number>,set,sms" > /tmp/nmt_cmd. The phone is
      called without ringing (L(12)), gets 5c x2 and L(15); the indicator
      shows. A phone that does not know L(12) rings; answering it shows the
      indicator.
- [ ] Phone in conversation: the indicator shows at once (5c x2 mid-call)
      and again after the clearing. During MFT or recall it comes only with
      the clearing.
- [ ] A call to a phone that has a message call pending is refused as busy.

## Short messages, to the phone

    echo "<number>,<originator>,<text>" > /tmp/nmt_sms_deliver

- [ ] Plain text "Hello 123": shown right on the phone.
- [ ] GSM alphabet: "a@b $5 x_y [1] {2} ~|^\" is shown with the right
      characters (before, @ $ _ and the bracket characters came out wrong).
- [ ] National letters with escapes, e.g. "caf<0xe9>": DCS 0x30, shown as
      the national letter (depends on the phone's character set).
- [ ] Time stamp of the message on the phone: right time AND right time zone
      (local time, with daylight saving). OPEN QUESTION: we send the zone as
      semi-octets with a zero units digit as 1010 (UTC = A0h, UTC+2 = 80h),
      the example in Annex 24 p.28 shows 08h for UTC+2. If the phone shows a
      wrong zone or time, try the example's coding.
- [ ] Phone memory full: the phone answers RP-ERROR (cause 22) or NR; the
      log shows the cause; with NR the exchange waits at most 3 s.
- [ ] Weak signal / repeated frames: after the fourth repetition each DMS
      frame is followed by 81 ms without tones (visible in a recording).
- [ ] The speech path is quiet for 200 ms from the start of each DMS frame
      (no clicks from the data in the earpiece).

## Short messages, from the phone

- [ ] Send a message to the SMSC (767): it appears in /tmp/nmt_sms_submit
      with the right text, also with @ $ _ [ ] { } ~ | ^ \.
- [ ] The phone shows the message as sent (RP-ACK), the call is released
      about 5 s later.
- [ ] No compandor order (no L(5)) on the call to the SMSC.
- [ ] The phone asks for waiting messages (SMS-REQUEST, if it has the
      function): it gets SMS-NO-MESSAGES and the call is released.
- [ ] The phone reports memory available (SMMA, after deleting messages when
      memory was full): it gets RP-ACK.
- [ ] A message that the phone sends with a user data header or a validity
      period of the reserved format is still taken.
