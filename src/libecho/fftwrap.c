/* Copyright (C) 2005-2006 Jean-Marc Valin
   File: fftwrap.c

   Wrapper for various FFTs - using Kiss FFT implementation

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   - Neither the name of the Xiph.org Foundation nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "arch.h"
#include "os_support.h"
#include "kiss_fftr.h"
#include "kiss_fft.h"

struct kiss_config {
    kiss_fftr_cfg forward;
    kiss_fftr_cfg backward;
    int N;
};

void *spx_fft_init(int size)
{
    struct kiss_config *table;
    table = (struct kiss_config *)speex_alloc(sizeof(struct kiss_config));
    table->forward = kiss_fftr_alloc(size, 0, NULL, NULL);
    table->backward = kiss_fftr_alloc(size, 1, NULL, NULL);
    table->N = size;
    return table;
}

void spx_fft_destroy(void *table)
{
    struct kiss_config *t = (struct kiss_config *)table;
    kiss_fftr_free(t->forward);
    kiss_fftr_free(t->backward);
    speex_free(table);
}

void spx_fft(void *table, spx_word16_t *in, spx_word16_t *out)
{
    int i;
    float scale;
    struct kiss_config *t = (struct kiss_config *)table;
    scale = 1.f / t->N;
    kiss_fftr2(t->forward, in, out);
    for (i = 0; i < t->N; i++)
        out[i] *= scale;
}

void spx_ifft(void *table, spx_word16_t *in, spx_word16_t *out)
{
    struct kiss_config *t = (struct kiss_config *)table;
    kiss_fftri2(t->backward, in, out);
}

