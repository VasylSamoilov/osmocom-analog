/* Generic LF time signal baseband TX encoder */

#ifndef TIMESIGNAL_TX_H
#define TIMESIGNAL_TX_H

#include <time.h>
#include "../libsample/sample.h"
#include "timesignal.h"

struct timesignal_tx {
	struct time_protocol *proto;
	int samplerate;
	double samples_per_second;
	double sample_counter;		/* fractional position within second */
	time_t timestamp;		/* start of current minute */
	int second;			/* current second (0-59) */
	double level;			/* current output amplitude */
	struct second_mod current_mod;	/* modulation for current second */
	int seg_index;			/* current segment within second */
	double seg_sample;		/* sample position within segment */
	/* carrier oscillator (for sound card mode) */
	double carrier_phase;
	double carrier_phase_step;
};

void timesignal_tx_init(struct timesignal_tx *tx, struct time_protocol *proto,
			int samplerate);
void timesignal_tx_start(struct timesignal_tx *tx, time_t timestamp, double sub_sec);
void timesignal_tx_encode(struct timesignal_tx *tx, sample_t *samples, int length);
void timesignal_tx_encode_carrier(struct timesignal_tx *tx, sample_t *samples, int length);

#endif /* TIMESIGNAL_TX_H */
