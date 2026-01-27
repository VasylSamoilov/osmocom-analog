/*
Copyright (c) 2003-2004, Mark Borgerding

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the author nor the names of any contributors may be used to endorse
      or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef KISS_FTR_H
#define KISS_FTR_H

#include "kiss_fft.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct kiss_fftr_state *kiss_fftr_cfg;

/**
 * kiss_fftr_alloc - Initialize real FFT state
 * @nfft: FFT size (must be even)
 * @inverse_fft: 0 for forward FFT, 1 for inverse FFT
 * @mem: Optional pre-allocated memory
 * @lenmem: Size of pre-allocated memory
 * @return: Real FFT configuration handle
 */
kiss_fftr_cfg kiss_fftr_alloc(int nfft, int inverse_fft, void *mem, size_t *lenmem);

/**
 * kiss_fftr - Forward real FFT
 * @cfg: Real FFT configuration
 * @timedata: Input real array (nfft samples)
 * @freqdata: Output complex array (nfft/2+1 samples)
 */
void kiss_fftr(kiss_fftr_cfg cfg, const kiss_fft_scalar *timedata, kiss_fft_cpx *freqdata);

/**
 * kiss_fftr2 - Forward real FFT to packed format
 */
void kiss_fftr2(kiss_fftr_cfg st, const kiss_fft_scalar *timedata, kiss_fft_scalar *freqdata);

/**
 * kiss_fftri - Inverse real FFT
 */
void kiss_fftri(kiss_fftr_cfg cfg, const kiss_fft_cpx *freqdata, kiss_fft_scalar *timedata);

/**
 * kiss_fftri2 - Inverse real FFT from packed format
 */
void kiss_fftri2(kiss_fftr_cfg st, const kiss_fft_scalar *freqdata, kiss_fft_scalar *timedata);

#define kiss_fftr_free speex_free

#ifdef __cplusplus
}
#endif

#endif /* KISS_FTR_H */

