#ifndef ERMES_DSP_H
#define ERMES_DSP_H

/* ERMES receiver, the signal part: FM demodulated samples in (1.0 = the outer deviation),
 * symbols out to rx.c. 4-PAM at 3125 Bd (ETS 300 133-4 §8.3): symbol clock by a Gardner timing
 * loop, levels by decision-directed DC (the carrier offset) and gain. */

#include "rx.h"

typedef struct ermes_dsp_rx {
	double		step;		/* symbols per sample */
	double		phase;		/* 0..1 through the symbol, 1 = its centre */
	double		last;		/* the sample before */
	double		mid;		/* the value half a symbol ago (between two symbols) */
	double		prev;		/* the value at the last symbol centre */
	int		have_mid;
	double		dc;		/* offset of the levels (carrier offset / deviation) */
	double		gain;		/* outer level (1.0 nominal) */
	double		polarity;	/* +1 or -1 (-P) */
	uint64_t	symbols;
	ermes_rx_t	*rx;
} ermes_dsp_rx_t;

void ermes_dsp_rx_init(ermes_dsp_rx_t *dsp, ermes_rx_t *rx, int samplerate, double polarity);
void ermes_dsp_rx_samples(ermes_dsp_rx_t *dsp, const double *samples, int length);

#endif /* ERMES_DSP_H */
