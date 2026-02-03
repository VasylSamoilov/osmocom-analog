/* FFT-based SSB filter - based on SDRangel's fftfilt
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * Based on SDRangel by Edouard Griffiths, F4EXB
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssbfilt.h"
#include "../libfft/fft.h"



/* Calculate log2 of a power-of-2 number */
static int log2_int(int n)
{
	int m = 0;
	while ((1 << m) < n) m++;
	return m;
}

int ssbfilt_init(ssbfilt_t *filt, int fftlen, double bandwidth, double samplerate)
{
	int i;
	double f2 = bandwidth / samplerate;  /* normalized cutoff frequency */
	
	memset(filt, 0, sizeof(*filt));
	
	filt->flen = fftlen;
	filt->flen2 = fftlen >> 1;
	filt->m = log2_int(fftlen);
	
	/* Allocate buffers */
	filt->filter_r = calloc(fftlen, sizeof(double));
	filt->filter_i = calloc(fftlen, sizeof(double));
	filt->data_r = calloc(fftlen, sizeof(double));
	filt->data_i = calloc(fftlen, sizeof(double));
	filt->ovlbuf_r = calloc(filt->flen2, sizeof(double));
	filt->ovlbuf_i = calloc(filt->flen2, sizeof(double));
	filt->output_r = calloc(filt->flen2, sizeof(double));
	filt->output_i = calloc(filt->flen2, sizeof(double));
	
	if (!filt->filter_r || !filt->filter_i || !filt->data_r || 
	    !filt->data_i || !filt->ovlbuf_r || !filt->ovlbuf_i ||
	    !filt->output_r || !filt->output_i) {
		ssbfilt_exit(filt);
		return -1;
	}
	
	/* Create lowpass filter directly in frequency domain.
	 * This avoids the impulse response centering issues.
	 * The filter passes bins from DC to bandwidth, zeros the rest.
	 */
	{
		int passband_bins = (int)(f2 * filt->flen + 0.5);  /* Number of bins for passband */
		if (passband_bins < 1) passband_bins = 1;
		if (passband_bins > filt->flen2) passband_bins = filt->flen2;
		
		fprintf(stderr, "SSB filter init: flen=%d, flen2=%d, f2=%.6f, passband_bins=%d (bw=%.1f, sr=%.1f)\n",
		        filt->flen, filt->flen2, f2, passband_bins, bandwidth, samplerate);
		
		/* Zero all filter coefficients */
		for (i = 0; i < filt->flen; i++) {
			filt->filter_r[i] = 0.0;
			filt->filter_i[i] = 0.0;
		}
		
		/* Set passband bins (positive frequencies: 0 to passband_bins) */
		for (i = 0; i <= passband_bins; i++) {
			filt->filter_r[i] = 1.0;
		}
		
		/* Set passband bins (negative frequencies: flen-passband_bins to flen-1) */
		for (i = filt->flen - passband_bins; i < filt->flen; i++) {
			filt->filter_r[i] = 1.0;
		}
		
		/* Apply raised cosine rolloff for smooth transition (5 bins) */
		int rolloff = 5;
		if (rolloff > passband_bins) rolloff = passband_bins;
		for (i = 0; i < rolloff; i++) {
			double alpha = 0.5 * (1.0 + cos(M_PI * (double)i / rolloff));
			/* Positive edge */
			if (passband_bins + i < filt->flen2) {
				filt->filter_r[passband_bins + i] = alpha;
			}
			/* Negative edge */
			int neg_idx = filt->flen - passband_bins - i;
			if (neg_idx > filt->flen2 && neg_idx < filt->flen) {
				filt->filter_r[neg_idx] = alpha;
			}
		}
	}
	
	/* Filter is already in frequency domain - no FFT needed.
	 * Filter coefficients are already unity gain in passband. */
	
	filt->inptr = 0;
	filt->outptr = 0;
	filt->outavail = 0;
	
	return 0;
}

void ssbfilt_exit(ssbfilt_t *filt)
{
	if (filt->filter_r) free(filt->filter_r);
	if (filt->filter_i) free(filt->filter_i);
	if (filt->data_r) free(filt->data_r);
	if (filt->data_i) free(filt->data_i);
	if (filt->ovlbuf_r) free(filt->ovlbuf_r);
	if (filt->ovlbuf_i) free(filt->ovlbuf_i);
	if (filt->output_r) free(filt->output_r);
	if (filt->output_i) free(filt->output_i);
	memset(filt, 0, sizeof(*filt));
}

/* Internal: run FFT filter block when input buffer is full */
static void ssbfilt_run_block(ssbfilt_t *filt, int usb)
{
	int i;
	double tr, ti;
	
	/* Forward FFT */
	fft_process(1, filt->m, filt->data_r, filt->data_i);
	
	/* Zero DC */
	filt->data_r[0] = 0.0;
	filt->data_i[0] = 0.0;
	
	/* SSB filtering - exactly like SDRangel */
	if (usb) {
		/* USB: keep positive freq bins, zero negative */
		for (i = 1; i < filt->flen2; i++) {
			tr = filt->data_r[i] * filt->filter_r[i] - filt->data_i[i] * filt->filter_i[i];
			ti = filt->data_r[i] * filt->filter_i[i] + filt->data_i[i] * filt->filter_r[i];
			filt->data_r[i] = tr;
			filt->data_i[i] = ti;
		}
		for (i = filt->flen2; i < filt->flen; i++) {
			filt->data_r[i] = 0.0;
			filt->data_i[i] = 0.0;
		}
	} else {
		/* LSB: zero positive freq bins, filter negative bins with filter[flen2+i] */
		for (i = 1; i < filt->flen2; i++) {
			filt->data_r[i] = 0.0;
			filt->data_i[i] = 0.0;
		}
		for (i = filt->flen2; i < filt->flen; i++) {
			/* Use filter[i] (i.e., filter[flen2+offset]) for negative freq bins - matches SDRangel */
			tr = filt->data_r[i] * filt->filter_r[i] - filt->data_i[i] * filt->filter_i[i];
			ti = filt->data_r[i] * filt->filter_i[i] + filt->data_i[i] * filt->filter_r[i];
			filt->data_r[i] = tr;
			filt->data_i[i] = ti;
		}
	}
	
	/* Inverse FFT */
	fft_process(-1, filt->m, filt->data_r, filt->data_i);
	
	/* Overlap-add into output buffer */
	for (i = 0; i < filt->flen2; i++) {
		filt->output_r[i] = filt->ovlbuf_r[i] + filt->data_r[i];
		filt->output_i[i] = filt->ovlbuf_i[i] + filt->data_i[i];
		filt->ovlbuf_r[i] = filt->data_r[i + filt->flen2];
		filt->ovlbuf_i[i] = filt->data_i[i + filt->flen2];
	}
	
	/* Clear data buffer */
	memset(filt->data_r, 0, filt->flen * sizeof(double));
	memset(filt->data_i, 0, filt->flen * sizeof(double));
	
	/* Mark output available */
	filt->outptr = 0;
	filt->outavail = filt->flen2;
}

/* Process one input sample, return one output sample (sample-by-sample API) */
void ssbfilt_process(ssbfilt_t *filt, double in_r, double in_i, int usb, double *out_r, double *out_i)
{
	/* Add input to accumulation buffer */
	filt->data_r[filt->inptr] = in_r;
	filt->data_i[filt->inptr] = in_i;
	filt->inptr++;
	
	/* When input buffer full, run FFT filter */
	if (filt->inptr >= filt->flen2) {
		filt->inptr = 0;
		ssbfilt_run_block(filt, usb);
	}
	
	/* Return buffered output (or zero if not yet available) */
	if (filt->outavail > 0) {
		*out_r = filt->output_r[filt->outptr];
		*out_i = filt->output_i[filt->outptr];
		filt->outptr++;
		filt->outavail--;
	} else {
		*out_r = 0.0;
		*out_i = 0.0;
	}
}

