#ifndef _LIB_FM_H
#define _LIB_FM_H

#include "../libfilter/iir_filter.h"
#include <stdint.h>

int fm_init(int fast_math);
void fm_exit(void);

/* Fast math lookup access (valid after fm_init with fast_math=1) */
int fm_fast_math_enabled(void);
void fm_fast_sincos(double phase, double *sin_out, double *cos_out);

enum fm_mod_state {
	MOD_STATE_OFF,		/* transmitter off, no IQ vector */
	MOD_STATE_ON,		/* transmitter on, FM modulated IQ vector */
	MOD_STATE_RAMP_UP,	/* use half cos to ramp up IQ vector */
	MOD_STATE_RAMP_DOWN,	/* use half cos to ramp down IQ vector */
};

typedef struct fm_mod {
	double samplerate;	/* sample rate of in and out */
	double offset;		/* offset to calculated center frequency */
	double amplitude;	/* how much amplitude to add to the buff */
	double phase;		/* current phase of FM (used to shift and modulate ) */
	enum fm_mod_state state;/* state of transmit power */
	double *ramp_tab;	/* half cosine ramp up */
	int ramp;		/* current ramp position */
	int ramp_length;	/* number of values in ramp */
} fm_mod_t;

int fm_mod_init(fm_mod_t *mod, double samplerate, double offset, double amplitude);
void fm_mod_exit(fm_mod_t *mod);
void fm_modulate_complex(fm_mod_t *mod, sample_t *frequency, uint8_t *power, int num, float *baseband);

typedef struct fm_demod {
	double samplerate;	/* sample rate of in and out */
	double phase;		/* current rotation phase (used to shift) */
	double rot;		/* rotation step per sample to shift rx frequency (used to shift) */
	double rot_base;	/* base rotation (from init offset, before AFC) */
	double last_phase;	/* last phase of FM (used to demodulate) */
	iir_filter_t lp[2];	/* filters received IQ signal */
	/* AFC (Automatic Frequency Control) - FLL on IQ before lowpass.
	 * Measures carrier offset from IQ phase differences, then adjusts
	 * the IQ mixer to re-center the signal before the lowpass filter. */
	struct {
		int	enabled;
		/* FLL state */
		double	fll_last_phase;		/* previous IQ phase */
		double	fll_freq;		/* IIR-filtered freq (rad/sample) */
		double	fll_alpha;		/* IIR coeff = 1/(tc*samplerate) */
		int	fll_initialized;
		/* Parameters */
		double	time_constant_s;	/* IIR time constant */
		double	max_correction_hz;	/* clamp limit */
		/* Status */
		double	freq_error_hz;		/* measured carrier offset (Hz) */
		double	correction_hz;		/* applied NCO correction (Hz) */
		double	peak_error_hz;		/* peak error seen (Hz) */
		uint64_t update_count;		/* number of AFC updates */
	} afc;
} fm_demod_t;

int fm_demod_init(fm_demod_t *demod, double samplerate, double offset, double bandwidth);
void fm_demod_exit(fm_demod_t *demod);
void fm_demod_set_offset(fm_demod_t *demod, double offset_hz);
void fm_demod_afc_enable(fm_demod_t *demod, double time_constant_s, double max_correction_hz);
void fm_demod_afc_disable(fm_demod_t *demod);
double fm_demod_afc_get_correction(fm_demod_t *demod);
double fm_demod_afc_get_freq_error(fm_demod_t *demod);
double fm_demod_afc_get_peak_error(fm_demod_t *demod);
void fm_demod_afc_reset_peak(fm_demod_t *demod);
void fm_demodulate_complex(fm_demod_t *demod, sample_t *frequency, int length, float *baseband, sample_t *I, sample_t *Q);
void fm_demodulate_real(fm_demod_t *demod, sample_t *frequency, int length, sample_t *baseband, sample_t *I, sample_t *Q);

#endif /* _LIB_FM_H */
