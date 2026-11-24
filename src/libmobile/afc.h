/* AFC for the FSK paging receivers (FLEX, POCSAG, GSC)
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * As SDRangel's radio paging demodulator (modemradiopaging/, plugins/
 * channelrx/demodradiopaging/radiopagingdemodsink.cpp): the carrier offset is
 * the DC of the FM discriminator. While hunting it is the mean of the last
 * symbols (64); on a known pattern (a sync word, a preamble) the levels are
 * fitted by least squares and held for the transmission. The offset found
 * moves the channel's demodulator (sender_set_rx_trim(): its oscillator and
 * its filter, the "leg") onto the signal. A reception confirmed (two fits or
 * a message) keeps its trim as the hunt centre, else the trim goes back to
 * the hunt centre when it ends; without fits the hunt centre decays to 0
 * (exp(-1/30) a second).
 *
 * Units: a sample value of 1.0 is dev_hz of frequency (the decoders get the
 * discriminator divided by the speech deviation).
 */

#ifndef _AFC_H
#define _AFC_H

struct sender;	/* sender.h (no include guard there) */

#define AFC_HIST		256	/* symbols kept */
#define AFC_HUNT_WINDOW		64	/* symbols of the hunting mean */

typedef struct afc {
	struct sender	*sender;
	double		range;		/* most trim (Hz), 0 = off */
	double		dev_hz;		/* Hz of a sample value of 1.0 */
	double		trim;		/* trim set (Hz) */
	double		hunt_centre;	/* trim while hunting (Hz) */
	double		pending;	/* DC change from the next block on (sample units) */
	int		fits;		/* fits in this reception */
	int		confirmed;	/* this reception confirmed */
	long		samples;	/* samples since the last hunt centre decay */
	double		last_fit_hz;	/* carrier offset of the last fit (Hz) */
	double		hist[AFC_HIST];	/* raw symbol values (DC included), oldest first from pos */
	int		hist_pos, hist_n;
} afc_t;

void afc_init(afc_t *afc, struct sender *sender, double range_hz, double dev_hz);
/* a symbol's raw value (DC included, sample units) */
void afc_symbol(afc_t *afc, double raw);
/* hunting: mean of the last n symbols (DC) and the mean distance from it
 * (deviation); returns the number of symbols used */
int afc_hunt(const afc_t *afc, int n, double *dc, double *dev);
/* least squares fit x = dc + dev * l of the last n symbols, symbols_ago
 * symbols before the newest, to the known levels l[0..n-1] (oldest first);
 * guard: |dev| within 0.6 .. 1.6 of dev_expected (if > 0). Returns 1 and
 * dc, dev if it fits. */
int afc_fit(const afc_t *afc, const double *levels, int n, int symbols_ago, double dev_expected, double *dc, double *dev);
/* a fit: dc (sample units, relative to the demodulator now) moves the trim */
void afc_measured(afc_t *afc, double dc);
/* the reception ended; decoded = a message (frame) was received */
void afc_release(afc_t *afc, int decoded);
/* at the start of each received block: the hunt centre decay; returns the
 * DC change (sample units) that the trim set before brings from this block
 * on (the history moved already): add it to the decoder's DC estimate */
double afc_block(afc_t *afc, int length, int samplerate);

#endif /* _AFC_H */
