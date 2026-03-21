/* Generic LF time signal baseband TX encoder
 *
 * Generates AM envelope samples from any time_protocol implementation.
 * Used by the SDR path — the SDR handles the actual carrier frequency.
 *
 * (C) 2022 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../liblogging/logging.h"
#include "../libsample/sample.h"
#include "timesignal.h"
#include "timesignal_tx.h"

/* Amplitude levels */
#define LEVEL_HIGH	0.9
#define LEVEL_LOW	(0.9 * 0.15)	/* 15% of full carrier */
#define LEVEL_OFF	0.0

static double power_to_level(enum carrier_power power)
{
	switch (power) {
	case CARRIER_HIGH: return LEVEL_HIGH;
	case CARRIER_LOW:  return LEVEL_LOW;
	case CARRIER_OFF:  return LEVEL_OFF;
	}
	return LEVEL_HIGH;
}

void timesignal_tx_init(struct timesignal_tx *tx, struct time_protocol *proto,
			int samplerate)
{
	memset(tx, 0, sizeof(*tx));
	tx->proto = proto;
	tx->samplerate = samplerate;
	tx->samples_per_second = (double)samplerate;
	tx->sample_counter = 0.0;
	tx->second = 0;
	tx->level = LEVEL_HIGH;
	tx->seg_index = 0;
	tx->seg_sample = 0.0;
	/* carrier oscillator for sound card mode */
	tx->carrier_phase = 0.0;
	tx->carrier_phase_step = 2.0 * M_PI * proto->carrier_frequency / (double)samplerate;
}

void timesignal_tx_start(struct timesignal_tx *tx, time_t timestamp, double sub_sec)
{
	tx->second = timestamp % 60;
	tx->timestamp = timestamp - tx->second;
	tx->sample_counter = sub_sec * tx->samples_per_second;

	/* prepare frame for current minute */
	tx->proto->prepare_minute(tx->proto, tx->timestamp);

	/* get modulation for current second */
	tx->current_mod = tx->proto->get_modulation(tx->proto, tx->second);
	tx->seg_index = 0;
	tx->seg_sample = 0.0;

	/* advance to correct segment based on sub_sec position */
	{
		double ms_into_second = sub_sec * 1000.0;
		double ms_consumed = 0.0;
		int i;
		for (i = 0; i < tx->current_mod.count; i++) {
			int dur = tx->current_mod.segments[i].duration_ms;
			if (dur == 0) {
				/* fill segment — we're here */
				tx->seg_index = i;
				tx->seg_sample = (ms_into_second - ms_consumed) * tx->samplerate / 1000.0;
				break;
			}
			if (ms_into_second < ms_consumed + dur) {
				tx->seg_index = i;
				tx->seg_sample = (ms_into_second - ms_consumed) * tx->samplerate / 1000.0;
				break;
			}
			ms_consumed += dur;
		}
	}

	if (tx->seg_index < tx->current_mod.count)
		tx->level = power_to_level(tx->current_mod.segments[tx->seg_index].power);
	else
		tx->level = LEVEL_HIGH;
}

void timesignal_tx_encode(struct timesignal_tx *tx, sample_t *samples, int length)
{
	int i;

	for (i = 0; i < length; i++) {
		samples[i] = tx->level;
		tx->sample_counter += 1.0;

		/* check segment transition within current second */
		tx->seg_sample += 1.0;
		if (tx->seg_index < tx->current_mod.count) {
			int dur = tx->current_mod.segments[tx->seg_index].duration_ms;
			if (dur > 0) {
				double seg_samples = (double)dur * tx->samplerate / 1000.0;
				if (tx->seg_sample >= seg_samples) {
					tx->seg_sample -= seg_samples;
					tx->seg_index++;
					if (tx->seg_index < tx->current_mod.count)
						tx->level = power_to_level(tx->current_mod.segments[tx->seg_index].power);
					else
						tx->level = LEVEL_HIGH;
				}
			}
			/* dur == 0 means fill remainder, no transition */
		}

		/* second boundary */
		if (tx->sample_counter >= tx->samples_per_second) {
			tx->sample_counter -= tx->samples_per_second;
			tx->second++;
			if (tx->second >= 60) {
				tx->second = 0;
				tx->timestamp += 60;
				tx->proto->prepare_minute(tx->proto, tx->timestamp);
			}
			/* get modulation for new second */
			tx->current_mod = tx->proto->get_modulation(tx->proto, tx->second);
			tx->seg_index = 0;
			tx->seg_sample = 0.0;
			if (tx->current_mod.count > 0)
				tx->level = power_to_level(tx->current_mod.segments[0].power);
			else
				tx->level = LEVEL_HIGH;
		}
	}
}

/* Encode with carrier for sound card output.
 * Generates: sin(2π * freq * t) * envelope
 * Same as dcf77_encode() but protocol-agnostic. */
void timesignal_tx_encode_carrier(struct timesignal_tx *tx, sample_t *samples, int length)
{
	int i;

	/* first generate the envelope */
	timesignal_tx_encode(tx, samples, length);

	/* multiply by carrier */
	for (i = 0; i < length; i++) {
		samples[i] *= sin(tx->carrier_phase);
		tx->carrier_phase += tx->carrier_phase_step;
		if (tx->carrier_phase >= 2.0 * M_PI)
			tx->carrier_phase -= 2.0 * M_PI;
	}
}
