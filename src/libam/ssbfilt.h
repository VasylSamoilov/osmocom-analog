/* FFT-based SSB filter - based on SDRangel's fftfilt
 *
 * (C) 2024 
 * Based on SDRangel by Edouard Griffiths, F4EXB
 * License: GPL v3
 *
 * Uses overlap-add FFT convolution for sideband selection.
 */

#ifndef SSBFILT_H
#define SSBFILT_H

typedef struct {
	int flen;        /* FFT length */
	int flen2;       /* flen / 2 */
	int m;           /* log2(flen) for FFT */
	double *filter_r; /* Filter frequency response (real) */
	double *filter_i; /* Filter frequency response (imag) */
	double *data_r;   /* Input data buffer (real) */
	double *data_i;   /* Input data buffer (imag) */
	double *ovlbuf_r; /* Overlap buffer (real) */
	double *ovlbuf_i; /* Overlap buffer (imag) */
	double *output_r; /* Output buffer (real) */
	double *output_i; /* Output buffer (imag) */
	int inptr;       /* Input pointer */
	int outptr;      /* Output read pointer for sample-by-sample reading */
	int outavail;    /* Number of output samples available */
} ssbfilt_t;

/* Initialize SSB filter with given FFT length (must be power of 2) */
int ssbfilt_init(ssbfilt_t *filt, int fftlen, double bandwidth, double samplerate);

/* Cleanup SSB filter */
void ssbfilt_exit(ssbfilt_t *filt);

/* Process one input sample, get one output sample
 * Returns the filtered I/Q output via out_r and out_i
 */
void ssbfilt_process(ssbfilt_t *filt, double in_r, double in_i, int usb, double *out_r, double *out_i);

#endif

