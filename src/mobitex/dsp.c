/* Mobitex signal processing
 *
 * Custom cosine-ramped 2-FSK modulation and zero-crossing demodulation.
 */

#define CHAN mobitex->sender.kanal

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "mobitex.h"
#include "frame.h"
#include "dsp.h"

/* Init transceiver instance. */
int dsp_init_sender(mobitex_t *mobitex, int samplerate, int baudrate,
                    double deviation, double polarity)
{
	int rc;
	double c;
	int i;

	LOGP_CHAN(DDSP, LOGL_DEBUG, "Init DSP for transceiver.\n");

	if (baudrate == 1200) {
		/* Mobitex-1200: FFSK modulation, center 1500 Hz, shift ±600 Hz */
		double ffsk_center = 1500.0;
		double ffsk_shift = 600.0;
		LOGP_CHAN(DDSP, LOGL_NOTICE, "Mobitex-1200 mode: FFSK center=%.0f Hz, shift=±%.0f Hz\n",
			  ffsk_center, ffsk_shift);
		sender_set_fm(&mobitex->sender, ffsk_shift, ffsk_center + ffsk_shift, ffsk_shift, ffsk_center + ffsk_shift);
		/* TODO: Full Mobitex-1200 FFSK DSP implementation.
		 * The FFSK modulator/demodulator requires different signal processing
		 * than the GMSK-approximated 2-FSK used for Mobitex-8000.
		 * For now, the cosine-ramped FSK path below is used as a placeholder. */
	} else {
		/* Mobitex-8000: GMSK-approximated 2-FSK, max_modulation = 8000 */
		sender_set_fm(&mobitex->sender, deviation, 8000, deviation, 8000);
	}

	mobitex->fsk_bitduration = (double)samplerate / (double)baudrate;
	mobitex->fsk_bitstep = 1.0 / mobitex->fsk_bitduration;
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Use %.4f samples for one bit duration @ %d.\n", mobitex->fsk_bitduration, mobitex->sender.samplerate);

	/* allocate TX sample buffer: enough for a full frame
	 * bitsync(16) + framesync(16) + header(24) + 20*240 = 4856 bits max */
	mobitex->fsk_tx_buffer_size = mobitex->fsk_bitduration * 5000 + 10;
	mobitex->fsk_tx_buffer = calloc(mobitex->fsk_tx_buffer_size, sizeof(sample_t));
	if (!mobitex->fsk_tx_buffer) {
		LOGP_CHAN(DDSP, LOGL_ERROR, "No memory!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* create deviation and ramp */
	mobitex->fsk_deviation = 1.0;
	mobitex->fsk_polarity = polarity;

	/* generate cosine shaped ramp table */
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Generating cosine shaped ramp table.\n");
	for (i = 0; i < 256; i++) {
		if (i < 64)
			c = 1.0;
		else if (i >= 192)
			c = -1.0;
		else
			c = cos((double)(i - 64) / 128.0 * M_PI);
		mobitex->fsk_ramp_down[i] = c * mobitex->fsk_deviation * mobitex->fsk_polarity;
		mobitex->fsk_ramp_up[i] = -mobitex->fsk_ramp_down[i];
	}

	/* init RX state */
	mobitex->fsk_rx_phase = 0;
	mobitex->fsk_rx_lastbit = 0;
	mobitex->fsk_rx_word = 0;
	mobitex->fsk_rx_sync = 0;
	mobitex->fsk_rx_index = 0;

	mobitex->samplerate = samplerate;

	return 0;

error:
	dsp_cleanup_sender(mobitex);

	return rc;
}

/* Cleanup transceiver instance. */
void dsp_cleanup_sender(mobitex_t *mobitex)
{
	LOGP_CHAN(DDSP, LOGL_DEBUG, "Cleanup DSP for transceiver.\n");

	if (mobitex->fsk_tx_buffer) {
		free(mobitex->fsk_tx_buffer);
		mobitex->fsk_tx_buffer = NULL;
	}
}

/* Encode an array of individual bits into FSK samples in fsk_tx_buffer.
 * For each bit: if same as lastbit, hold level; if different, traverse ramp.
 * Returns number of samples written. */
int fsk_encode_bits(mobitex_t *mobitex, const uint8_t *bits, int nbits)
{
	sample_t *spl;
	double phase, bitstep, devpol;
	int i, count;
	uint8_t lastbit;

	devpol = mobitex->fsk_deviation * mobitex->fsk_polarity;
	spl = mobitex->fsk_tx_buffer;
	phase = mobitex->fsk_tx_phase;
	lastbit = mobitex->fsk_tx_lastbit;
	bitstep = mobitex->fsk_bitstep * 256.0;

	for (i = 0; i < nbits; i++) {
		uint8_t bit = bits[i] & 1;
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
					*spl++ = mobitex->fsk_ramp_down[(uint8_t)phase];
					phase += bitstep;
				} while (phase < 256.0);
				phase -= 256.0;
				lastbit = 0;
			}
		} else {
			if (bit) {
				/* ramp up */
				do {
					*spl++ = mobitex->fsk_ramp_up[(uint8_t)phase];
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
	}

	count = ((uintptr_t)spl - (uintptr_t)mobitex->fsk_tx_buffer) / sizeof(*spl);

	mobitex->fsk_tx_phase = phase;
	mobitex->fsk_tx_lastbit = lastbit;

	return count;
}

void sender_send(sender_t *sender, sample_t *samples, uint8_t *power, int length)
{
	mobitex_t *mobitex = (mobitex_t *)sender;

	/* if TX not enabled, output silence with carrier off */
	if (!mobitex->tx) {
		memset(samples, 0, sizeof(*samples) * length);
		memset(power, 0, length);
		return;
	}

again:
	/* if no data in buffer, check SVP timer */
	if (!mobitex->fsk_tx_buffer_length) {
		mobitex_tx_idle_tick(mobitex, length);
	}

	/* if still no data, generate a header-only ROSI filler frame.
	 * Real Mobitex base stations transmit continuously — the mobile's
	 * modem needs the bitsync pattern to keep its bit clock locked.
	 * Without fillers, the modem sees unmodulated carrier (no transitions)
	 * and loses sync, causing "No Coverage" on the device. */
	if (!mobitex->fsk_tx_buffer_length) {
		if (!mobitex_is_mobile_mode(mobitex)) {
			mobitex_tx_filler(mobitex);
		}
		/* Mobile mode: no filler, just carrier */
		if (!mobitex->fsk_tx_buffer_length) {
			memset(samples, 0, sizeof(*samples) * length);
			memset(power, 1, length);
			return;
		}
	}

	/* send encoded samples from buffer */
	while (length) {
		*power++ = 1;
		*samples++ = mobitex->fsk_tx_buffer[mobitex->fsk_tx_buffer_pos++];
		length--;
		if (mobitex->fsk_tx_buffer_pos == mobitex->fsk_tx_buffer_length) {
			mobitex->fsk_tx_buffer_length = 0;
			break;
		}
	}

	/* if destination buffer not yet full, try to get more data */
	if (length)
		goto again;
}

/*
 * Zero-crossing FSK demodulator with sync detection state machine.
 *
 * Demodulates audio samples into bits using a phase accumulator with
 * bit-edge resync on zero crossings. A 32-bit shift register detects
 * the combined bitsync+framesync pattern. The state machine then
 * collects header bits and data block bits.
 *
 * States:
 *   0 = HUNT:   scanning for bitsync|framesync in shift register
 *   1 = HEADER: collecting 24 frame header bits
 *   2 = DATA:   collecting 240 bits per data block
 */
static void fsk_decode(mobitex_t *mobitex, sample_t *spl, int length)
{
	int i;
	double polarity = mobitex->fsk_polarity;

	for (i = 0; i < length; i++) {
		uint8_t bit;

		/* Zero-crossing FSK demod with phase accumulator */
		if (spl[i] * polarity > 0.0) {
			if (!mobitex->fsk_rx_lastbit) {
				/* zero crossing: resync phase */
				mobitex->fsk_rx_phase = -0.5;
				mobitex->fsk_rx_lastbit = 1;
			} else {
				mobitex->fsk_rx_phase += mobitex->fsk_bitstep;
				if (mobitex->fsk_rx_phase < 1.0)
					continue;
				mobitex->fsk_rx_phase -= 1.0;
			}
			bit = 1;
		} else {
			if (mobitex->fsk_rx_lastbit) {
				/* zero crossing: resync phase */
				mobitex->fsk_rx_phase = -0.5;
				mobitex->fsk_rx_lastbit = 0;
			} else {
				mobitex->fsk_rx_phase += mobitex->fsk_bitstep;
				if (mobitex->fsk_rx_phase < 1.0)
					continue;
				mobitex->fsk_rx_phase -= 1.0;
			}
			bit = 0;
		}

		/* State machine */
		switch (mobitex->fsk_rx_sync) {
		case 0: /* HUNT mode */
			mobitex->fsk_rx_word = (mobitex->fsk_rx_word << 1) | bit;

			/* Check for bitsync (upper 16) + framesync (lower 16) */
			{
				uint16_t upper = (mobitex->fsk_rx_word >> 16) & 0xFFFF;
				uint16_t lower = mobitex->fsk_rx_word & 0xFFFF;

				if (upper == mobitex->bitsync) {
					int frsync_idx;
					int pol = mobitex_match_frsync(lower, &frsync_idx);
					if (pol != 0) {
						/* Filter for specific frsync if configured */
						if (mobitex->frsync_index >= 0 && frsync_idx != mobitex->frsync_index)
							break;

						if (pol < 0) {
							/* Inverted polarity: flip RX polarity */
							polarity = -polarity;
							mobitex->fsk_polarity = polarity;
							LOGP_CHAN(DDSP, LOGL_INFO, "Inverted polarity detected, flipping.\n");
						}

						LOGP_CHAN(DDSP, LOGL_INFO, "Sync found: frsync index %d.\n", frsync_idx);

						/* Store detected frsync index for message display */
						mobitex->rx_frsync_index = frsync_idx;

						/* Transition to HEADER state */
						mobitex->fsk_rx_sync = 1;
						mobitex->fsk_rx_index = 0;
					}
				}
			}
			break;

		case 1: /* HEADER: collect 24 bits */
			mobitex->rx_block_buf[mobitex->fsk_rx_index++] = bit;
			if (mobitex->fsk_rx_index >= MOBITEX_HEADER_BITS) {
				mobitex_frame_header_t hdr;
				memset(&hdr, 0, sizeof(hdr));

				if (mobitex_decode_header(mobitex->rx_block_buf, &hdr) < 0) {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Header FEC failure, returning to HUNT.\n");
					mobitex->fsk_rx_sync = 0;
					break;
				}

				mobitex->frame_header = hdr;
				LOGP_CHAN(DDSP, LOGL_INFO, "Header: BaseID=%d AreaID=%d CFlags=%d\n",
					  hdr.base_id, hdr.area_id, hdr.cflags);

				/* RAMnet check: CFlags bits 1-2 == 0 means data blocks follow.
				 * The original checks only bits 1 and 2 of the second control
				 * byte: (cb2 & 0x60) == 0 on the 12-bit codeword, which
				 * corresponds to bits 1-2 of the 8-bit data byte. */
				if (mobitex->ramnet && (hdr.cflags & 0x06) != 0) {
					LOGP_CHAN(DDSP, LOGL_DEBUG, "RAMnet: CFlags!=0, no data blocks.\n");
					mobitex->fsk_rx_sync = 0;
					break;
				}

				/* Transition to DATA state */
				mobitex->fsk_rx_sync = 2;
				mobitex->fsk_rx_index = 0;
				mobitex->rx_block_number = 0;
				mobitex->rx_block_pos = 0;
				mobitex->rx_fec_errors_total = 0;
				memset(&mobitex->link_control, 0, sizeof(mobitex->link_control));

				/* Reset RX scrambler once before first data block.
				 * The scrambler runs continuously across all blocks. */
				mobitex_scrambler_reset(&mobitex->scrambler_rx_sr);
			}
			break;

		case 2: /* DATA: collect 240 bits per block */
			mobitex->rx_block_buf[mobitex->rx_block_pos++] = bit;
			if (mobitex->rx_block_pos >= MOBITEX_BLOCK_BITS) {
				uint8_t data[MOBITEX_DATA_BYTES];
				int fec_errors = 0;
				int rc;

				rc = mobitex_decode_block(mobitex, mobitex->rx_block_buf, data, &fec_errors);
				mobitex->rx_fec_errors = fec_errors;
				mobitex->rx_fec_errors_total += fec_errors;

				if (rc == 0 && fec_errors <= MOBITEX_MAX_FEC_ERRORS) {
					int k;
					/* Store decoded data bytes */
					for (k = 0; k < MOBITEX_DATA_BYTES; k++)
						mobitex->rx_data_blocks[mobitex->rx_block_number][k] = data[k];

					if (mobitex->rx_block_number == 0) {
						/* Primary block: parse link control from full 20-byte decoded block.
						 * We need to reconstruct the 20-byte block for parsing. */
						uint8_t block20[20];
						memcpy(block20, data, MOBITEX_DATA_BYTES);
						block20[18] = 0;
						block20[19] = 0;
						mobitex_parse_link_control(block20, &mobitex->link_control);
						LOGP_CHAN(DDSP, LOGL_INFO, "Link Control: DestMAN=%06X FrameID=%02X BlkLen=%d\n",
							  mobitex->link_control.dest_man,
							  mobitex->link_control.frame_id,
							  mobitex->link_control.block_length);
					}
				} else {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Block %d: %s (FEC errors: %d)\n",
						  mobitex->rx_block_number,
						  (rc < 0) ? "CRC failure" : "too many FEC errors",
						  fec_errors);
				}

				mobitex->rx_block_number++;
				mobitex->rx_block_pos = 0;

				/* Check if all expected blocks received */
				if (mobitex->rx_block_number > 0 &&
				    mobitex->link_control.block_length > 0 &&
				    mobitex->rx_block_number >= mobitex->link_control.block_length) {
					LOGP_CHAN(DDSP, LOGL_INFO, "Frame complete: %d blocks received.\n",
						  mobitex->rx_block_number);
					mobitex_rx_frame_complete(mobitex);
					mobitex->fsk_rx_sync = 0;
				}

				/* Safety limit */
				if (mobitex->rx_block_number >= MOBITEX_MAX_BLOCKS) {
					LOGP_CHAN(DDSP, LOGL_NOTICE, "Max blocks reached, returning to HUNT.\n");
					mobitex->fsk_rx_sync = 0;
				}
			}
			break;
		}
	}
}

/* Process received audio stream from radio unit. */
void sender_receive(sender_t *sender, sample_t *samples, int length, double rf_level_db)
{
	mobitex_t *mobitex = (mobitex_t *)sender;
	(void)rf_level_db;

	if (!mobitex->rx)
		return;

	fsk_decode(mobitex, samples, length);
}
