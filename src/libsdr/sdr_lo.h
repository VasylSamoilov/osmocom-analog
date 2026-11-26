/* Where the SDR's local oscillator and its DC go (sdr_lo.c) */

/* the decision for one direction */
typedef struct sdr_lo_plan {
	double	lo_offset;	/* to the driver: RF LO at the centre + this (0 = none) */
	int	lo_outside;	/* that puts the LO (its DC, leakage) outside the stream (|offset| >= rate / 2) */
	double	band_shift;	/* channels = 0: the band this far above the centre of the stream, the device tuned that much lower */
	int	band_no_room;	/* channels = 0: a shift was needed but the band would leave the usable band */
} sdr_lo_plan_t;

/* the automatic LO offset: just beyond half the SDR rate */
double sdr_lo_offset_auto(int sdr_rate);

/* given: an --sdr-lo-offset was given (offset, also 0 = none), else automatic;
 * lo_applies: the driver moves its RF LO by the offset and back in its DSP
 * (UHD: yes; SoapySDR: only known at open, so no; rpitx: no LO at all, the
 * offset is 0 and no shift); band_rate: the IQ rate of the band of a
 * program with channels = 0 (0: channels, or a backend's device - the
 * channels' placement keeps them off the centre). */
void sdr_lo_plan(sdr_lo_plan_t *p, int sdr_rate, int given, double offset, int lo_applies, int no_lo, int band_rate);
