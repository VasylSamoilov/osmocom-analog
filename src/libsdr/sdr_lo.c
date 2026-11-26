/* Where the SDR's local oscillator and its DC go.
 *
 * The programs' channels are made at their DSP rate around 0 Hz; it is the
 * SDR layer that places them and keeps the device's DC (LO leakage, DC
 * offset) off them:
 *  - an LO offset the driver applies (UHD: RF LO at centre + offset, its
 *    DSP moves the signal back): automatic just beyond half the SDR rate,
 *    the LO outside the stream (the B210 runs: 1.92 MS/s, -1 MHz); an
 *    --sdr-lo-offset given is used as given (0 = none);
 *  - without one (SoapySDR without a tuning stage after the RF LO - only
 *    known at open -, an offset of 0, or one inside the stream) the DC is
 *    at the centre of the stream (or at centre + offset): programs with
 *    channels have them placed off it (sdr.c), a program that takes one
 *    band (channels = 0: osmoradio, tv, dcf77) gets its band shifted above
 *    the centre by half its IQ rate and a guard, the device tuned that much
 *    lower - when the band stays in the usable band (75 % of the rate);
 *  - rpitx makes its carrier itself: no LO, no offset, no shift. */

#include <math.h>
#include "sdr_lo.h"

/* the guard beyond half the rate, of the SDR rate */
#define LO_GUARD	0.04
/* the usable band of the SDR rate, one side (75 %, as sdr.c) */
#define USABLE_HALF	0.375
/* the band's signal of its IQ rate, one side (the channelizer's passband) */
#define BAND_HALF	0.4

double sdr_lo_offset_auto(int sdr_rate)
{
	return -(0.5 + LO_GUARD) * sdr_rate;
}

void sdr_lo_plan(sdr_lo_plan_t *p, int sdr_rate, int given, double offset, int lo_applies, int no_lo, int band_rate)
{
	p->lo_offset = 0.0;
	p->lo_outside = 0;
	p->band_shift = 0.0;
	p->band_no_room = 0;
	if (no_lo)
		return;
	p->lo_offset = given ? offset : sdr_lo_offset_auto(sdr_rate);
	p->lo_outside = (lo_applies && fabs(p->lo_offset) >= 0.5 * sdr_rate);
	if (!band_rate || p->lo_outside)
		return;
	/* the DC in the stream (at the centre, or at centre + offset if the
	 * driver applies one inside it): the band above it by half its rate
	 * and a guard */
	{
		double dc = lo_applies ? p->lo_offset : 0.0;
		double shift = dc + 0.5 * band_rate + LO_GUARD * sdr_rate;

		if (fabs(shift) + BAND_HALF * band_rate > USABLE_HALF * sdr_rate) {
			p->band_no_room = 1;
			return;
		}
		p->band_shift = shift;
	}
}
