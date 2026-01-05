/* Polyphase resampler for arbitrary sample rate conversion
 *
 * Based on SDRangel's polyphase interpolator design.
 * Supports both upsampling and downsampling with FIR filtering.
 *
 * (C) 2026
 * GPLv3
 */

#ifndef POLYPHASE_H
#define POLYPHASE_H

#include "../libsample/sample.h"

/* Opaque state structure */
typedef struct polyphase_state {
	double		input_rate;
	double		output_rate;
	double		distance;		/* input_rate / output_rate */
	double		distance_remain;	/* fractional position */
	int		phase_steps;		/* number of polyphase filters */
	int		num_taps;		/* taps per phase */
	float		*taps;			/* polyphase filter coefficients */
	float		*aligned_taps;		/* aligned for SIMD */
	sample_t	*samples;		/* delay line */
	int		ptr;			/* delay line pointer */
	sample_t	last_sample;		/* for interpolation */
} polyphase_t;

/* Initialize polyphase resampler
 * 
 * @param state       Pointer to state structure
 * @param input_rate  Input sample rate in Hz
 * @param output_rate Output sample rate in Hz
 * @param cutoff      Filter cutoff frequency in Hz
 * @param phase_steps Number of polyphase decompositions (16 typical)
 * @return 0 on success, negative on error
 */
int polyphase_init(polyphase_t *state, double input_rate, double output_rate,
                   double cutoff, int phase_steps);

/* Resample audio samples
 *
 * Works for any input/output ratio:
 * - input_rate > output_rate: decimation (downsample)
 * - input_rate < output_rate: interpolation (upsample)
 *
 * @param state      Resampler state
 * @param input      Input samples
 * @param input_num  Number of input samples
 * @param output     Output buffer
 * @param output_max Maximum output samples (buffer size)
 * @return Number of output samples produced
 */
int polyphase_resample(polyphase_t *state, const sample_t *input, int input_num,
                       sample_t *output, int output_max);

/* Calculate expected output size for given input size */
int polyphase_output_num(polyphase_t *state, int input_num);

/* Calculate required input size for desired output size */
int polyphase_input_num(polyphase_t *state, int output_num);

/* Free resampler resources */
void polyphase_free(polyphase_t *state);

#endif /* POLYPHASE_H */
