/* GSC signal processing
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CHAN gsc->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/param.h>
#include <sys/stat.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "golay.h"
#include "dsp.h"

#define MAX_DISPLAY	1.4	/* something above speech level, no emphasis */
#define VOICE_BANDWIDTH	3000	/* just guessing */

static void dsp_init_ramp(gsc_t *gsc)
{
        double c;
        int i;

        LOGP_CHAN(DDSP, LOGL_DEBUG, "Generating cosine shaped ramp table.\n");
        for (i = 0; i < 256; i++) {
		/* This is mathematically incorrect... */
                if (i < 64)
                        c = 1.0;
		else if (i >= 192)
                        c = -1.0;
		else
	                c = cos((double)(i - 64) / 128.0 * M_PI);
                gsc->fsk_ramp_down[i] = c * gsc->fsk_deviation * gsc->fsk_polarity;
                gsc->fsk_ramp_up[i] = -gsc->fsk_ramp_down[i];
        }
}

/* Init transceiver instance. */
int dsp_init_sender(gsc_t *gsc, int samplerate, double deviation, double polarity)
{
	int rc;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Init DSP for transceiver.\n");

	/* set modulation parameters */
	// NOTE: baudrate equals modulation, because we have a raised cosine ramp of beta = 0.5
	sender_set_fm(&gsc->sender, deviation, 600.0, deviation, MAX_DISPLAY);

	gsc->fsk_bitduration = (double)samplerate / 600.0;
	gsc->fsk_bitstep = 1.0 / gsc->fsk_bitduration;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Use %.4f samples for one bit duration @ %d.\n", gsc->fsk_bitduration, gsc->sender.samplerate);

	gsc->fsk_tx_buffer_size = gsc->fsk_bitduration + 10; /* 1 bit, add some extra to prevent short buffer due to rounding */
	gsc->fsk_tx_buffer = calloc(gsc->fsk_tx_buffer_size, sizeof(sample_t));
	if (!gsc->fsk_tx_buffer) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* create deviation and ramp */
	gsc->fsk_deviation = 1.0; // equals what we set at sender_set_fm()
	gsc->fsk_polarity = polarity;
	dsp_init_ramp(gsc);

	/* initialize RX DSP state for zero-crossing bit recovery */
	gsc->fsk_rx_phase = 0.0;
	gsc->fsk_rx_last_sample = 0.0;
	gsc->fsk_rx_last_bit = 0;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "RX DSP initialized: %.1f baud, polarity %s.\n", 600.0, polarity < 0 ? "inverted" : "normal");

	return 0;

error:
        dsp_cleanup_sender(gsc);

        return -rc;

}

/* Cleanup transceiver instance. */
void dsp_cleanup_sender(gsc_t *gsc)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Cleanup DSP for transceiver.\n");

	if (gsc->fsk_tx_buffer) {
		free(gsc->fsk_tx_buffer);
		gsc->fsk_tx_buffer = NULL;
	}
}


/* encode one bit into samples
 * input: bit
 * output: samples
 * return number of samples */
static int fsk_bit_encode(gsc_t *gsc, uint8_t bit)
{
	/* alloc samples, add 1 in case there is a rest */
	sample_t *spl;
	sample_t *ramp_up, *ramp_down;
	double phase, bitstep, devpol;
	int count;
	uint8_t lastbit;

	devpol = gsc->fsk_deviation * gsc->fsk_tx_polarity;

	/* Swap ramp tables when per-message polarity differs from instance polarity
	 * (the ramps were pre-computed at init time using gsc->fsk_polarity) */
	if (gsc->fsk_tx_polarity != gsc->fsk_polarity) {
		ramp_up = gsc->fsk_ramp_down;
		ramp_down = gsc->fsk_ramp_up;
	} else {
		ramp_up = gsc->fsk_ramp_up;
		ramp_down = gsc->fsk_ramp_down;
	}

	spl = gsc->fsk_tx_buffer;
	phase = gsc->fsk_tx_phase;
	lastbit = gsc->fsk_tx_lastbit;
	bitstep = gsc->fsk_bitstep * 256.0;

	if (lastbit) {
		if (bit) {
			/* stay up */
			do {
				*spl++ = devpol;
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
		} else {
			/* ramp down */
			do {
				*spl++ = ramp_down[(uint8_t)phase];
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
			lastbit = 0;
		}
	} else {
		if (bit) {
			/* ramp up */
			do {
				*spl++ = ramp_up[(uint8_t)phase];
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
			lastbit = 1;
		} else {
			/* stay down */
			do {
				*spl++ = -devpol;
				phase += bitstep;
			} while (phase < 256.0);
			phase -= 256.0;
		}
	}

	/* depending on the number of samples, return the number */
	count = ((uintptr_t)spl - (uintptr_t)gsc->fsk_tx_buffer) / sizeof(*spl);

	gsc->fsk_tx_phase = phase;
	gsc->fsk_tx_lastbit = lastbit;

	return count;
}

/* Resolve the 46-bit shift register into a 23-bit Golay codeword.
 * Each bit is transmitted twice (duplicate encoding), so 46 bits = 23 pairs.
 * When the pair disagrees, prefer the first bit. */
static uint32_t resolve_shift_register(const uint8_t *shift)
{
	uint32_t codeword = 0;
	int i;

	for (i = 0; i < 23; i++) {
		uint8_t bit1 = shift[i * 2];
		uint8_t bit2 = shift[i * 2 + 1];
		uint8_t resolved = (bit1 == bit2) ? bit1 : bit1;
		codeword |= ((uint32_t)resolved << i);
	}

	return codeword;
}

/* Check if a decoded Golay value matches any of the 10 preamble values.
 * Returns the preamble index (0-9) on match, or -1 if no match. */
static int match_preamble(uint16_t value)
{
	/* preamble_values[] is defined in golay.c; we access it via
	 * decode_golay + comparison against known constants. Instead,
	 * replicate the table here to avoid exposing it. */
	static const uint16_t preamble_vals[10] = {
		2030, 1628, 3198,  647,  191, 3315, 1949, 2540, 1560, 2335,
	};
	int i;

	for (i = 0; i < 10; i++) {
		if (value == preamble_vals[i])
			return i;
	}

	return -1;
}

/* Number of consecutive matching preamble codewords required before
 * committing to a preamble lock. A single codeword match can happen
 * on noise; requiring multiple consecutive matches dramatically reduces
 * false locks while still locking quickly on real signals (3 codewords
 * = 138 bits = 0.23 seconds at 600 baud, well within the 18-codeword
 * preamble window). */
#define PREAMBLE_LOCK_THRESHOLD	3

/* Reset the receiver to idle state, clearing all RX buffers. */
static void rx_reset_idle(gsc_t *gsc)
{
	if (gsc->rx_state != RX_IDLE) {
		/* (no log — this fires constantly on noise) */
	}

	/* Stop any in-progress voice recording */
	if (gsc->voice_recording) {
		{
			double dur = (gsc->voice_rec.samplerate > 0) ? (double)gsc->voice_rec.written / gsc->voice_rec.samplerate : 0.0;
			wave_destroy_record(&gsc->voice_rec);
			gsc->voice_recording = 0;
			LOGP_CHAN(DDSP, LOGL_NOTICE, "Voice page ended for address '%s' (%.1fs), saved: %s\n",
				gsc->voice_address, dur, gsc->voice_filename);
		}
	}

	gsc->rx_state = RX_IDLE;
	gsc->rx_bit_num = 0;
	gsc->rx_shift_count = 0;
	gsc->rx_no_transition = 0;
	gsc->rx_confirm_index = -1;
	gsc->rx_confirm_count = 0;
	gsc->rx_confirm_bit_count = 0;
	gsc->rx_polarity_inverted = 0;
	gsc->rx_batch_candidate = 0;
	gsc->rx_batch_mode = 0;
	gsc->rx_nbs_count = 0;
	gsc->rx_nbs_shift_count = 0;
	gsc->rx_nbs_locked = 0;
}

/* Enter voice recording state after a voice message is decoded.
 * Sets up the 2-second alert wait, then recording begins in sender_receive(). */
static void rx_enter_voice(gsc_t *gsc, const gsc_rx_msg_t *msg)
{
	int rc;

	/* Save address for filename and logging */
	memcpy(gsc->voice_address, msg->address, sizeof(gsc->voice_address));
	gsc->voice_filename[0] = '\0';

	LOGP_CHAN(DDSP, LOGL_NOTICE, "Voice page started for address '%s'.\n", msg->address);

	/* 2-second alert pause: pager beeps during this time, audio is not voice yet */
	gsc->voice_alert_wait = gsc->sender.samplerate * 2;

	/* 2-minute recording timeout */
	gsc->voice_timeout = gsc->sender.samplerate * 120;

	/* Reset preamble scanner for stop detection during voice */
	gsc->rx_shift_count = 0;
	gsc->rx_no_transition = 0;
	gsc->rx_bit_num = 0;

	/* Open WAV file if voice_dir is configured */
	if (gsc->voice_dir) {
		/* Ensure output directory exists */
		mkdir(gsc->voice_dir, 0755);

		snprintf(gsc->voice_filename, sizeof(gsc->voice_filename), "%s/golay_voice_page_%ld_%s.wav",
			gsc->voice_dir, (long)time(NULL), msg->address);

		rc = wave_create_record(&gsc->voice_rec, gsc->voice_filename, 22050, 1, gsc->fsk_deviation);
		if (rc < 0) {
			LOGP_CHAN(DDSP, LOGL_ERROR, "Failed to create voice recording '%s'.\n", gsc->voice_filename);
			gsc->voice_recording = 0;
			gsc->voice_filename[0] = '\0';
		} else {
			init_samplerate(&gsc->voice_downsample, 22050, (double)gsc->sender.samplerate, VOICE_BANDWIDTH);
			gsc->voice_recording = 1;
			LOGP_CHAN(DDSP, LOGL_INFO, "Voice recording opened: '%s' (waiting 2 sec alert).\n", gsc->voice_filename);
		}
	} else {
		gsc->voice_recording = 0;
		LOGP_CHAN(DDSP, LOGL_INFO, "Voice message for '%s': no voice-dir configured, skipping recording.\n", msg->address);
	}

	gsc->rx_state = RX_VOICE;
}

/* Feed a recovered bit into the RX state machine.
 *
 * Three-phase approach to avoid false preamble locks on noise:
 *
 * RX_IDLE: Bits are shifted into a 46-bit register. After each bit,
 *   the register is resolved as a duplicate Golay codeword and decoded.
 *   If the decoded value matches a known preamble, we save the raw bits
 *   and transition to RX_PREAMBLE for confirmation.
 *
 * RX_PREAMBLE: We accumulate bits 46 at a time. Each complete 46-bit
 *   group is decoded and checked against the candidate preamble index.
 *   After PREAMBLE_LOCK_THRESHOLD consecutive matches, we lock: the
 *   saved raw bits are copied into rx_bit[] (with a 28-bit comma
 *   placeholder) and we transition to RX_DATA.
 *   A mismatch resets back to RX_IDLE.
 *
 * RX_DATA: Bits accumulate in rx_bit[] until end-of-transmission is
 *   detected (48+ consecutive same-value bits), then decode_batch()
 *   is called. On success or failure, we return to RX_IDLE. */
static void fsk_receive_bit(gsc_t *gsc, uint8_t bit)
{
	gsc_rx_msg_t msg;
	int min_batch_bits;
	int rc;

	memset(&msg, 0, sizeof(msg));

	/* ---- Phase 1: RX_IDLE — scan for first preamble codeword ---- */
	if (gsc->rx_state == RX_IDLE) {
		/* ---- NBS preamble detection (§2.5) ----
		 * Only active when --nbs flag is set.
		 * The 75 Hz preamble is a 1,1,0,0 repeating pattern at 600 baud.
		 * Track the last 4 bits and count consecutive pattern matches.
		 * After 20 cycles (80 bits = 0.133s), lock as NBS preamble.
		 * Once locked, watch for the pattern to break (comma starts),
		 * then transition to RX_DATA for address decoding. */
		if (gsc->nbs) {
			/* Shift bit into 4-bit NBS register */
			if (gsc->rx_nbs_shift_count < 4) {
				gsc->rx_nbs_shift[gsc->rx_nbs_shift_count++] = bit;
			} else {
				gsc->rx_nbs_shift[0] = gsc->rx_nbs_shift[1];
				gsc->rx_nbs_shift[1] = gsc->rx_nbs_shift[2];
				gsc->rx_nbs_shift[2] = gsc->rx_nbs_shift[3];
				gsc->rx_nbs_shift[3] = bit;
			}

			if (gsc->rx_nbs_shift_count >= 4) {
				/* Check for 1,1,0,0 pattern */
				if (gsc->rx_nbs_shift[0] == 1 && gsc->rx_nbs_shift[1] == 1 &&
				    gsc->rx_nbs_shift[2] == 0 && gsc->rx_nbs_shift[3] == 0) {
					gsc->rx_nbs_count++;
					gsc->rx_nbs_shift_count = 0; /* reset for next 4-bit group */
				} else if (gsc->rx_nbs_shift[0] == 0 && gsc->rx_nbs_shift[1] == 0 &&
				           gsc->rx_nbs_shift[2] == 1 && gsc->rx_nbs_shift[3] == 1) {
					/* Inverted 75 Hz pattern — count it too */
					gsc->rx_nbs_count++;
					gsc->rx_nbs_shift_count = 0;
				} else if (gsc->rx_nbs_count >= 20) {
					/* Pattern broke after lock — NBS preamble ended.
					 * The current 4 bits are the start of post-preamble data.
					 * Transition to RX_DATA and start buffering. */
					LOGP_CHAN(DDSP, LOGL_INFO, "RX state: IDLE -> DATA (NBS preamble locked, %d cycles).\n",
						gsc->rx_nbs_count);

					gsc->rx_bit_num = 0;
					gsc->rx_nbs_locked = 1;
					gsc->rx_preamble_index = -1; /* unknown in NBS mode */
					gsc->rx_no_transition = 0;
					gsc->rx_shift_count = 0;
					gsc->rx_state = RX_DATA;

					/* Buffer the 4 bits that broke the pattern */
					{
						int b;
						for (b = 0; b < 4; b++)
							gsc->rx_bit[gsc->rx_bit_num++] = gsc->rx_nbs_shift[b];
					}

					gsc->rx_nbs_count = 0;
					gsc->rx_nbs_shift_count = 0;
					return;
				} else {
					/* Pattern didn't match and not locked — reset */
					gsc->rx_nbs_count = 0;
				}
			}
		}

		/* Shift new bit into the 46-bit register */
		if (gsc->rx_shift_count < 46) {
			gsc->rx_shift[gsc->rx_shift_count++] = bit;
		} else {
			memmove(gsc->rx_shift, gsc->rx_shift + 1, 45);
			gsc->rx_shift[45] = bit;
		}

		/* Need a full 46-bit window before we can try decoding */
		if (gsc->rx_shift_count < 46)
			return;

		/* Try to decode the shift register as a dup Golay codeword */
		{
			uint32_t codeword = resolve_shift_register(gsc->rx_shift);
			uint16_t decoded;
			int inverted = 0;

			if (decode_golay(codeword, &decoded) == 0) {
				int idx = match_preamble(decoded);
				if (idx >= 0)
					goto preamble_candidate;
			}

			/* Normal polarity didn't match — try inverted if auto-detecting */
			if (gsc->rx_auto_polarity) {
				if (decode_golay(codeword ^ 0x7FFFFF, &decoded) == 0) {
					int idx = match_preamble(decoded);
					if (idx >= 0) {
						inverted = 1;
						goto preamble_candidate;
					}
				}
			}
			goto idle_done;

		preamble_candidate:
			{
				int idx = match_preamble(decoded);
				/* Candidate preamble — save raw bits and
				 * enter confirmation phase.
				 *
				 * Auto-polarity disambiguation: when an inverted
				 * preamble is detected, we don't yet know if it's
				 * batch mode (only preamble inverted) or whole-signal
				 * inversion (everything inverted). Set batch_candidate
				 * instead of polarity_inverted; the start code check
				 * after preamble lock will disambiguate. */
				gsc->rx_confirm_index = idx;
				gsc->rx_confirm_count = 1;
				if (inverted) {
					gsc->rx_batch_candidate = 1;
					gsc->rx_polarity_inverted = 0;
				} else {
					gsc->rx_batch_candidate = 0;
					gsc->rx_polarity_inverted = 0;
				}
				memcpy(gsc->rx_confirm_bits, gsc->rx_shift, 46);
				gsc->rx_confirm_bit_count = 0;
				gsc->rx_shift_count = 0;
				gsc->rx_state = RX_PREAMBLE;
			}
		idle_done: ;
		}
		return;
	}

	/* ---- Phase 2: RX_PREAMBLE — confirm consecutive matches ---- */
	if (gsc->rx_state == RX_PREAMBLE) {
		/* Accumulate bits into the shift register, 46 at a time */
		gsc->rx_shift[gsc->rx_confirm_bit_count++] = bit;

		if (gsc->rx_confirm_bit_count < 46)
			return;

		/* We have a complete 46-bit group — decode and verify */
		{
			uint32_t codeword = resolve_shift_register(gsc->rx_shift);
			uint16_t decoded;
			int idx;

			gsc->rx_confirm_bit_count = 0;

			/* Apply same inversion as detected in IDLE.
			 * Both batch_candidate (inverted preamble, unknown if
			 * batch or whole-signal) and polarity_inverted (confirmed
			 * whole-signal) require XOR to decode preamble codewords. */
			if (gsc->rx_polarity_inverted || gsc->rx_batch_candidate)
				codeword ^= 0x7FFFFF;

			if (decode_golay(codeword, &decoded) != 0) {
				/* Decode failed — not a real preamble */
				rx_reset_idle(gsc);
				return;
			}

			idx = match_preamble(decoded);
			if (idx != gsc->rx_confirm_index) {
				/* Different preamble index or not a preamble at all */
				rx_reset_idle(gsc);
				return;
			}

			/* Save the raw bits from this confirmed codeword */
			memcpy(gsc->rx_confirm_bits + gsc->rx_confirm_count * 46,
			       gsc->rx_shift, 46);
			gsc->rx_confirm_count++;

			if (gsc->rx_confirm_count < PREAMBLE_LOCK_THRESHOLD) {
				LOGP_CHAN(DDSP, LOGL_DEBUG, "Preamble confirmation %d/%d (index %d).\n",
					gsc->rx_confirm_count, PREAMBLE_LOCK_THRESHOLD, idx);
				return;
			}

			/* Lock achieved — reconstruct rx_bit[] buffer.
			 * Prepend 28-bit comma placeholder, then copy all
			 * confirmed preamble codeword bits. */
			LOGP_CHAN(DDSP, LOGL_INFO, "RX state: PREAMBLE -> DATA (locked, index %d, %d codewords, %s).\n",
				gsc->rx_confirm_index, gsc->rx_confirm_count,
				gsc->rx_batch_candidate ? "batch candidate" :
				gsc->rx_polarity_inverted ? "inverted" : "normal");

			/* Start rx_bit[] empty — don't copy preamble bits.
			 * The preamble index is already known from confirmation.
			 * DATA state will fill rx_bit[] starting from the first
			 * post-confirmation bit. decode_batch() will scan for
			 * the start code in this buffer. */
			gsc->rx_bit_num = 0;

			gsc->rx_preamble_index = gsc->rx_confirm_index;
			gsc->rx_preamble_count = gsc->rx_confirm_count;
			gsc->rx_no_transition = 0;
			gsc->rx_state = RX_DATA;
		}
		return;
	}

	/* ---- Phase 3a: RX_VOICE — scan for new preamble as stop condition ---- */
	if (gsc->rx_state == RX_VOICE) {
		/* During voice recording, the FSK demodulator still runs on
		 * the audio. We use the demodulated bits only to detect a new
		 * preamble (which means a new transmission is starting and we
		 * should stop recording). The actual audio sample capture
		 * happens in sender_receive().
		 *
		 * No stop detection during the 2-second alert wait — the TX
		 * sends silence during this period (pager is alerting), which
		 * would false-trigger the silence detector. */
		if (gsc->voice_alert_wait > 0)
			return;

		/* Track consecutive same-value bits for silence detection.
		 * rx_bit[0] stores the previous bit value (rx_bit[] is unused
		 * for data buffering during RX_VOICE). */
		if (gsc->rx_bit_num >= 1 && bit == gsc->rx_bit[0])
			gsc->rx_no_transition++;
		else
			gsc->rx_no_transition = 0;
		gsc->rx_bit[0] = bit;
		gsc->rx_bit_num++;

		/* Silence detection: 48+ consecutive same-value bits means
		 * the transmitter stopped. On a real radio this may not fire
		 * (noise produces transitions), but works in loopback. */
		if (gsc->rx_no_transition >= 48) {
			LOGP_CHAN(DDSP, LOGL_INFO, "RX: voice recording stopped (silence detected).\n");
			rx_reset_idle(gsc);
			return;
		}

		/* Preamble scanning: sliding window, check after every bit.
		 * Unlike RX_DATA (which uses fixed 46-bit boundaries), we need
		 * a sliding window here because we have no bit alignment — the
		 * FSK demodulator produces garbage during voice audio, so we
		 * can't predict where the next preamble will start. */
		if (gsc->rx_shift_count < 46) {
			gsc->rx_shift[gsc->rx_shift_count++] = bit;
		} else {
			memmove(gsc->rx_shift, gsc->rx_shift + 1, 45);
			gsc->rx_shift[45] = bit;
		}

		if (gsc->rx_shift_count >= 46) {
			uint32_t codeword = resolve_shift_register(gsc->rx_shift);
			uint16_t decoded;
			int new_inverted = 0;
			int is_preamble = 0;
			int is_activation = 0;

			/* Check for activation code (voice stop sequence) */
			if (decode_golay(codeword, &decoded) == 0) {
				if (match_preamble(decoded) >= 0) {
					is_preamble = 1;
					new_inverted = 0;
				} else if (decoded == activation_code) {
					is_activation = 1;
				}
			}
			if (!is_preamble && !is_activation) {
				if (decode_golay(codeword ^ 0x7FFFFF, &decoded) == 0) {
					if (match_preamble(decoded) >= 0) {
						is_preamble = 1;
						new_inverted = 1;
					} else if (decoded == activation_code) {
						is_activation = 1;
					}
				}
			}

			if (!is_preamble && !is_activation)
				return; /* no match, keep recording */

			if (is_activation) {
				/* Activation code = voice stop sequence */
				LOGP_CHAN(DDSP, LOGL_INFO, "RX: activation code detected during voice, stopping recording.\n");
				if (gsc->voice_recording) {
					double dur = (gsc->voice_rec.samplerate > 0) ? (double)gsc->voice_rec.written / gsc->voice_rec.samplerate : 0.0;
					wave_destroy_record(&gsc->voice_rec);
					gsc->voice_recording = 0;
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Voice page ended for address '%s' (%.1fs), saved: %s\n", gsc->voice_address, dur, gsc->voice_filename);
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Voice page ended for address '%s' (not recorded).\n", gsc->voice_address);
				}
				rx_reset_idle(gsc);
				return;
			}

			/* New preamble detected — stop voice recording and
			 * start confirming the new preamble. */
			{
				int idx = match_preamble(decoded);
				LOGP_CHAN(DDSP, LOGL_INFO, "RX: new preamble (index %d) detected during voice, stopping recording.\n", idx);

				/* Stop recording */
				if (gsc->voice_recording) {
					double dur = (gsc->voice_rec.samplerate > 0) ? (double)gsc->voice_rec.written / gsc->voice_rec.samplerate : 0.0;
					wave_destroy_record(&gsc->voice_rec);
					gsc->voice_recording = 0;
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Voice page ended for address '%s' (%.1fs), saved: %s\n", gsc->voice_address, dur, gsc->voice_filename);
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Voice page ended for address '%s' (not recorded).\n", gsc->voice_address);
				}

				/* Start confirming the new preamble */
				gsc->rx_state = RX_PREAMBLE;
				gsc->rx_confirm_index = idx;
				gsc->rx_confirm_count = 1;
				if (new_inverted) {
					gsc->rx_batch_candidate = 1;
					gsc->rx_polarity_inverted = 0;
				} else {
					gsc->rx_batch_candidate = 0;
					gsc->rx_polarity_inverted = 0;
				}
				memcpy(gsc->rx_confirm_bits, gsc->rx_shift, 46);
				gsc->rx_confirm_bit_count = 0;
				gsc->rx_bit_num = 0;
				gsc->rx_no_transition = 0;

				LOGP_CHAN(DDSP, LOGL_INFO, "RX state: VOICE -> PREAMBLE (new candidate index %d, polarity %s).\n",
					idx, new_inverted ? "inverted" : "normal");
			}
		}
		return;
	}

	/* ---- Phase 3b: RX_DATA — buffer bits until end of transmission ---- */
	if (gsc->rx_bit_num >= (int)sizeof(gsc->rx_bit)) {
		LOGP_CHAN(DDSP, LOGL_INFO, "RX: bit buffer overflow (%d bits), lost lock.\n", gsc->rx_bit_num);
		rx_reset_idle(gsc);
		return;
	}

	/* Buffer the incoming bit.
	 * - If rx_batch_candidate is set, we don't yet know if this is batch
	 *   mode or whole-signal inversion, so buffer WITHOUT inversion.
	 *   The disambiguation check below will resolve this.
	 * - If rx_polarity_inverted is set (confirmed whole-signal inversion),
	 *   flip the bit as before.
	 * - Otherwise, buffer the bit as-is. */
	if (gsc->rx_batch_candidate)
		gsc->rx_bit[gsc->rx_bit_num++] = bit;
	else
		gsc->rx_bit[gsc->rx_bit_num++] = gsc->rx_polarity_inverted ? !bit : bit;

	/* ---- Auto-polarity disambiguation for batch candidate ----
	 *
	 * After preamble lock with rx_batch_candidate set, we need to
	 * determine if this is batch mode (only preamble inverted) or
	 * whole-signal inversion (everything inverted).
	 *
	 * Strategy: once we have enough bits for the start code region
	 * (preamble = 856 bits, start code = 121 bits = 977 total),
	 * extract the first dup Golay codeword of the start code and
	 * try to decode it:
	 *   - If it decodes to start_code (713) without inversion → BATCH MODE
	 *   - If it fails without inversion but succeeds with inversion → WHOLE-SIGNAL INVERSION
	 *   - If both fail → reset to idle (corrupted signal)
	 *
	 * Scan for start code in the buffer. The buffer starts from the
	 * first post-confirmation bit, so the remaining preamble codewords
	 * and start code are at the beginning. */
	if (gsc->rx_batch_candidate && gsc->rx_bit_num >= 28 + 46) {
		uint32_t sc_codeword;
		uint16_t sc_decoded;
		int sc_found = 0;
		int scan_end = gsc->rx_bit_num - 28 - 46;
		int scan;

		/* Limit scan range — start code should be within first ~800 bits */
		if (scan_end > 800)
			scan_end = 800;

		for (scan = 0; scan <= scan_end && !sc_found; scan++) {
			int try_pos = scan + 28; /* skip comma */
			if (try_pos + 46 > gsc->rx_bit_num)
				break;

			sc_codeword = resolve_shift_register(gsc->rx_bit + try_pos);

			if (decode_golay(sc_codeword, &sc_decoded) == 0 && sc_decoded == 713) {
				gsc->rx_batch_mode = 1;
				gsc->rx_batch_candidate = 0;
				gsc->rx_polarity_inverted = 0;
				sc_found = 1;
				LOGP_CHAN(DDSP, LOGL_INFO, "RX: auto-polarity: batch mode confirmed (start code at bit %d).\n", try_pos);
			} else if (decode_golay(sc_codeword ^ 0x7FFFFF, &sc_decoded) == 0 && sc_decoded == 713) {
				gsc->rx_polarity_inverted = 1;
				gsc->rx_batch_candidate = 0;
				gsc->rx_batch_mode = 0;
				sc_found = 1;
				LOGP_CHAN(DDSP, LOGL_INFO, "RX: auto-polarity: negative polarity confirmed (start code at bit %d).\n", try_pos);
				{
					int b;
					for (b = scan; b < gsc->rx_bit_num; b++)
						gsc->rx_bit[b] = !gsc->rx_bit[b];
				}
			}
		}

		if (!sc_found) {
			if (gsc->rx_bit_num < 800) {
				/* Not enough bits yet */
				return;
			}
			LOGP_CHAN(DDSP, LOGL_INFO, "RX: auto-polarity: disambiguation failed, lost lock.\n");
			rx_reset_idle(gsc);
			return;
		}
	}

	/* Track consecutive same-value bits for end-of-transmission detection.
	 * In loopback or clean signals, the PLL freewheels on silence and
	 * produces long runs of the same bit. A run of 48+ identical bits
	 * is well beyond any valid GSC pattern. */
	if (gsc->rx_bit_num >= 2 && gsc->rx_bit[gsc->rx_bit_num - 1] == gsc->rx_bit[gsc->rx_bit_num - 2])
		gsc->rx_no_transition++;
	else
		gsc->rx_no_transition = 0;

	/* Scan for new preamble in parallel with data accumulation.
	 * Requires PREAMBLE_LOCK_THRESHOLD consecutive matches (same as
	 * the IDLE path) to avoid false-triggering on data content that
	 * accidentally matches a preamble codeword. */
	if (gsc->rx_bit_num > 856) {
		gsc->rx_shift[gsc->rx_shift_count % 46] = bit;
		gsc->rx_shift_count++;

		if (gsc->rx_shift_count >= 46) {
			uint32_t codeword = resolve_shift_register(gsc->rx_shift);
			uint16_t decoded;
			int new_idx = -1;
			int new_inverted = 0;

			gsc->rx_shift_count = 0;

			if (decode_golay(codeword, &decoded) == 0)
				new_idx = match_preamble(decoded);
			if (new_idx < 0 && decode_golay(codeword ^ 0x7FFFFF, &decoded) == 0) {
				new_idx = match_preamble(decoded);
				if (new_idx >= 0)
					new_inverted = 1;
			}

			if (new_idx >= 0 && new_idx == gsc->rx_data_preamble_idx) {
				gsc->rx_data_preamble_count++;
			} else if (new_idx >= 0) {
				gsc->rx_data_preamble_idx = new_idx;
				gsc->rx_data_preamble_inv = new_inverted;
				gsc->rx_data_preamble_count = 1;
			} else {
				gsc->rx_data_preamble_count = 0;
			}

			if (gsc->rx_data_preamble_count >= PREAMBLE_LOCK_THRESHOLD) {
				int idx = gsc->rx_data_preamble_idx;
				int inv = gsc->rx_data_preamble_inv;
				int old_bits = gsc->rx_bit_num - gsc->rx_data_preamble_count * 46;

				LOGP_CHAN(DDSP, LOGL_INFO, "RX: new preamble (index %d, %s) confirmed during DATA (%d matches), force-decoding old batch (%d bits).\n",
					idx, inv ? "inverted" : "normal", gsc->rx_data_preamble_count, old_bits);

				if (old_bits >= 242) { /* start(121) + address(121) minimum */
					int saved_rx_num = gsc->rx_bit_num;
					gsc->rx_bit_num = old_bits;
					if (decode_batch(gsc, &msg, 1) == 0) {
						msg.polarity_inverted = gsc->rx_polarity_inverted;
						golay_msg_receive(&msg);
					}
					gsc->rx_bit_num = saved_rx_num;
				}

				gsc->rx_state = RX_PREAMBLE;
				gsc->rx_confirm_index = idx;
				gsc->rx_confirm_count = gsc->rx_data_preamble_count;
				if (inv) {
					gsc->rx_batch_candidate = 1;
					gsc->rx_polarity_inverted = 0;
				} else {
					gsc->rx_batch_candidate = 0;
					gsc->rx_polarity_inverted = 0;
				}
				memcpy(gsc->rx_confirm_bits, gsc->rx_shift, 46);
				gsc->rx_confirm_bit_count = 0;
				gsc->rx_bit_num = 0;
				gsc->rx_no_transition = 0;
				gsc->rx_data_preamble_count = 0;
				return;
			}
		}
	}

	/* Minimum bits for a valid message:
	 * GSC: start(121) + address(121) = 242 bits minimum (preamble not in buffer).
	 * NBS: address(121) only (no preamble or start code). */
	min_batch_bits = gsc->rx_nbs_locked ? 121 : 242;

	if (gsc->rx_bit_num < min_batch_bits)
		return;

	/* End-of-transmission detection.
	 *
	 * Three triggers, whichever fires first:
	 *
	 * 1) No-transition: 48+ consecutive same-value bits. Works in
	 *    loopback and clean signals where the PLL freewheels on
	 *    silence producing a constant bit value.
	 *
	 * 2) Periodic decode attempt: on a real radio, after TX stops the
	 *    demodulator output becomes noise with frequent zero crossings,
	 *    so the PLL produces random bits and the no-transition threshold
	 *    never fires. Instead, we attempt decode_batch() every 46 bits
	 *    (one codeword period) once we have enough data. If it succeeds,
	 *    we're done. This keeps latency low — we decode as soon as the
	 *    complete message is in the buffer, with at most 46 bits of
	 *    trailing noise.
	 *
	 * 3) Maximum batch duration: if we've buffered more bits than the
	 *    longest possible GSC batch without a successful decode, the
	 *    signal is garbage — give up and return to idle. This prevents
	 *    sitting in RX_DATA indefinitely on bad reception.
	 *    Max batch: preamble(856) + start(121) + 2 * [address(121)
	 *    + 32*alpha(3872)] = ~8962 bits for a 2-address batch.
	 *    Use 4096 as a practical limit for individual transmissions.
	 *    Batch transmissions with multiple addresses are handled by
	 *    decode_batch's batch continuation logic. */
	{
		const int max_batch_bits = 4096;
		int do_decode = 0;
		int trim_trailing = 0;
		int force_reset = 0;

		if (gsc->rx_no_transition >= 48) {
			/* Clean end-of-TX: trim trailing same-value bits */
			trim_trailing = 1;
			do_decode = 1;
			LOGP_CHAN(DDSP, LOGL_INFO, "RX: end-of-transmission detected (no-transition, %d bits buffered).\n", gsc->rx_bit_num);
		} else if (gsc->rx_bit_num >= max_batch_bits) {
			/* Exceeded maximum possible batch length — give up */
			force_reset = 1;
			do_decode = 1;
			LOGP_CHAN(DDSP, LOGL_INFO, "RX: max batch duration reached (%d bits), forcing final decode.\n", gsc->rx_bit_num);
		} else if ((gsc->rx_bit_num % 46) == 0) {
			/* Try decode every 46 bits (one codeword period).
			 * This is cheap — decode_batch() just parses a buffer
			 * and returns quickly on structural mismatch. */
			do_decode = 1;
		}

		if (do_decode) {
			int nbits = gsc->rx_bit_num;
			int force = (trim_trailing || force_reset);

			if (trim_trailing) {
				nbits -= gsc->rx_no_transition;
				if (nbits < min_batch_bits)
					nbits = min_batch_bits;
			}

			/* Temporarily set rx_bit_num for decode_batch.
			 * decode_batch reads directly from rx_bit/rx_bit_num
			 * to avoid clobbering the TX bit[] buffer. */
			int saved_rx_num = gsc->rx_bit_num;
			gsc->rx_bit_num = nbits;

			rc = gsc->rx_nbs_locked
				? decode_nbs(gsc, &msg, force)
				: decode_batch(gsc, &msg, force);

			gsc->rx_bit_num = saved_rx_num;

			if (rc == 0) {
				msg.polarity_inverted = gsc->rx_polarity_inverted;
				golay_msg_receive(&msg);
				if (msg.type == TYPE_VOICE) {
					rx_enter_voice(gsc, &msg);
				} else {
					rx_reset_idle(gsc);
				}
			} else if (force) {
				/* Signal is gone or buffer exhausted —
				 * no point continuing, return to idle. */
				LOGP_CHAN(DDSP, LOGL_INFO, "RX: decode failed (%s), lost lock.\n",
					force_reset ? "max duration" : "end-of-TX");
				rx_reset_idle(gsc);
			}
			/* Otherwise keep buffering — message may not be
			 * complete yet (more data blocks to come). */
		}
	}
}

/* Process received audio stream from radio unit.
 *
 * Samples are already FM-demodulated by the sender framework:
 * positive values = positive deviation, negative = negative deviation.
 * We use a zero-crossing detector with PLL bit clock recovery at 600 baud,
 * matching the POCSAG demodulator pattern used elsewhere in this project. */
void sender_receive(sender_t *sender, sample_t *samples, int length, double __attribute__((unused)) rf_level_db)
{
	gsc_t *gsc = (gsc_t *) sender;
	double polarity;
	int i;

	if (!gsc->rx)
		return;

	/* Voice recording: capture raw audio samples during RX_VOICE state.
	 * This runs BEFORE the FSK demodulator so we capture the full buffer,
	 * even if fsk_receive_bit transitions out of RX_VOICE mid-buffer. */
	if (gsc->rx_state == RX_VOICE) {
		int rec_offset = 0;

		/* Handle 2-second alert wait */
		if (gsc->voice_alert_wait > 0) {
			int skip = (gsc->voice_alert_wait < length) ? gsc->voice_alert_wait : length;
			gsc->voice_alert_wait -= skip;
			rec_offset = skip;
			if (gsc->voice_alert_wait == 0)
				LOGP_CHAN(DDSP, LOGL_INFO, "Voice alert wait complete, recording started for '%s'.\n", gsc->voice_address);
		}

		/* Write samples to WAV file (skip alert portion, downsample to 22050 Hz) */
		if (gsc->voice_alert_wait == 0 && gsc->voice_recording && rec_offset < length) {
			int in_count = length - rec_offset;
			sample_t downbuf[in_count];

			memcpy(downbuf, samples + rec_offset, in_count * sizeof(sample_t));
			int out_count = samplerate_downsample(&gsc->voice_downsample, downbuf, in_count);
			sample_t *rec_samples[1] = { downbuf };
			wave_write(&gsc->voice_rec, rec_samples, out_count);
		}

		/* 2-minute timeout: decrement by buffer length */
		gsc->voice_timeout -= length;
		if (gsc->voice_timeout <= 0) {
			LOGP_CHAN(DDSP, LOGL_INFO, "Voice recording timeout (2 min) for '%s'.\n", gsc->voice_address);
			rx_reset_idle(gsc);
			return;
		}
	}

	polarity = gsc->fsk_polarity;

	for (i = 0; i < length; i++) {
		/* Soft PLL bit clock recovery (same approach as multimon-ng).
		 * Sample the demodulated signal, track transitions with a
		 * gradual phase correction instead of hard resets. */
		gsc->fsk_rx_dcd_shreg <<= 1;
		gsc->fsk_rx_dcd_shreg |= ((samples[i] * polarity) > 0.0) ? 1 : 0;

		/* Check for transition */
		if ((gsc->fsk_rx_dcd_shreg ^ (gsc->fsk_rx_dcd_shreg >> 1)) & 1) {
			if (gsc->fsk_rx_phase < 0.5)
				gsc->fsk_rx_phase += gsc->fsk_bitstep / 8.0;
			else
				gsc->fsk_rx_phase -= gsc->fsk_bitstep / 8.0;
		}

		gsc->fsk_rx_phase += gsc->fsk_bitstep;
		if (gsc->fsk_rx_phase >= 1.0) {
			gsc->fsk_rx_phase -= 1.0;
			fsk_receive_bit(gsc, gsc->fsk_rx_dcd_shreg & 1);
		}
	}
}

/* Provide stream of audio toward radio unit */
void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	gsc_t *gsc = (gsc_t *) sender;
	int rc;

again:
	/* play 2 seconds of pause */
	if (gsc->wait_2_sec) {
		int tosend = MIN(length, gsc->wait_2_sec);
		memset(power, 1, tosend);
		memset(samples, 0, sizeof(samples) * tosend);
		power += tosend;
		samples += tosend;
		gsc->wait_2_sec -= tosend;
		if (gsc->wait_2_sec)
			return;
	}

	/* play wave file, if open */
	if (gsc->wave_tx_play.left) {
		int wave_num, s;
		wave_num = samplerate_upsample_input_num(&gsc->wave_tx_upsample, length);
		sample_t buffer[wave_num * 2], *wave_samples[2] = { buffer, buffer + wave_num };
		wave_read(&gsc->wave_tx_play, wave_samples, wave_num);
		if (gsc->wave_tx_channels == 2) {
			for (s = 0; s < wave_num; s++) {
				wave_samples[0][s] += wave_samples[1][s];
			}
		}
		samplerate_upsample(&gsc->wave_tx_upsample, wave_samples[0], wave_num, samples, length);
		if (!gsc->wave_tx_play.left) {
			LOGP_CHAN(DDSP, LOGL_INFO, "Voice message sent.\n");
			wave_destroy_playback(&gsc->wave_tx_play);
			return;
		}
		return;
	}


	/* get FSK bits or start playing wave file */
	if (!gsc->fsk_tx_buffer_length) {
		int8_t bit = get_bit(gsc);

		/* bit == 2 means voice transmission. */
		if (bit == 2) {
			if (gsc->wave_tx_filename[0]) {
				gsc->wave_tx_samplerate = gsc->wave_tx_channels = 0;
				rc = wave_create_playback(&gsc->wave_tx_play, gsc->wave_tx_filename, &gsc->wave_tx_samplerate, &gsc->wave_tx_channels, gsc->fsk_deviation);
				if (rc < 0) {
					gsc->wave_tx_play.left = 0;
					LOGP_CHAN(DDSP, LOGL_ERROR, "Failed to open wave file '%s' for voice message.\n", gsc->wave_tx_filename);
				} else {
					LOGP_CHAN(DDSP, LOGL_INFO, "Sending wave file '%s' for voice message after 2 seconds.\n", gsc->wave_tx_filename);
					init_samplerate(&gsc->wave_tx_upsample, gsc->wave_tx_samplerate, gsc->sender.samplerate, VOICE_BANDWIDTH);
				}
			}
			gsc->wait_2_sec = gsc->sender.samplerate * 2.0;
			goto again;
		}

		/* no message, power is off */
		if (bit < 0) {
			memset(samples, 0, sizeof(samples) * length);
			memset(power, 0, length);
			return;
		}

		/* encode */
		gsc->fsk_tx_buffer_length = fsk_bit_encode(gsc, bit);
		gsc->fsk_tx_buffer_pos = 0;
	}

	/* send encoded bit until end of source or destination buffer is reached */
	while (length) {
		*power++ = 1;
		*samples++ = gsc->fsk_tx_buffer[gsc->fsk_tx_buffer_pos++];
		length--;
		if (gsc->fsk_tx_buffer_pos == gsc->fsk_tx_buffer_length) {
			gsc->fsk_tx_buffer_length = 0;
			break;
		}
	}

	/* do again, if destination buffer is not yet full */
	if (length)
		goto again;
}

