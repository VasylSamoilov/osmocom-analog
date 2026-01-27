/* Copyright (C) 2003-2008 Jean-Marc Valin

   File: mdf.c
   Echo canceller based on the MDF algorithm (see below)

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

   1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

/*
   The echo canceller is based on the MDF algorithm described in:

   J. S. Soo, K. K. Pang Multidelay block frequency adaptive filter,
   IEEE Trans. Acoust. Speech Signal Process., Vol. ASSP-38, No. 2,
   February 1990.

   We use the Alternatively Updated MDF (AUMDF) variant. Robustness to
   double-talk is achieved using a variable learning rate as described in:

   Valin, J.-M., On Adjusting the Learning Rate in Frequency Domain Echo
   Cancellation With Double-Talk. IEEE Transactions on Audio,
   Speech and Language Processing, Vol. 15, No. 3, pp. 1030-1034, 2007.
   http://people.xiph.org/~jm/papers/valin_taslp2006.pdf

   There is no explicit double-talk detection, but a continuous variation
   in the learning rate based on residual echo, double-talk and background
   noise.
*/

#include <math.h>
#include <string.h>
#include "arch.h"
#include "speex_echo.h"
#include "fftwrap.h"
#include "pseudofloat.h"
#include "math_approx.h"
#include "os_support.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define WEIGHT_SHIFT 0

/* Use two-path filter for robustness */
#define TWO_PATH

static const spx_float_t MIN_LEAK = .005f;
static const spx_float_t VAR1_SMOOTH = .36f;
static const spx_float_t VAR2_SMOOTH = .7225f;
static const spx_float_t VAR1_UPDATE = .5f;
static const spx_float_t VAR2_UPDATE = .25f;
static const spx_float_t VAR_BACKTRACK = 4.f;
#define TOP16(x) (x)

/* PLAYBACK_DELAY controls the internal buffer size for the async API.
 * This compensates for jitter between playback and capture calls.
 * Value of 6 = ~96ms buffer at 16ms frames, enough for typical jitter.
 */
#define PLAYBACK_DELAY 6

void speex_echo_get_residual(SpeexEchoState *st, spx_word32_t *Yout, int len);

/** Speex echo cancellation state. */
struct SpeexEchoState_ {
    int frame_size;           /**< Number of samples processed each time */
    int window_size;
    int M;
    int cancel_count;
    int adapted;
    int saturated;
    int screwed_up;
    int C;                    /** Number of input channels (microphones) */
    int K;                    /** Number of output channels (loudspeakers) */
    spx_int32_t sampling_rate;
    spx_word16_t spec_average;
    spx_word16_t beta0;
    spx_word16_t beta_max;
    spx_word32_t sum_adapt;
    spx_word16_t leak_estimate;
    spx_float_t adapt_rate;   /* Adaptation rate multiplier (1.0 = default) */

    spx_word16_t *e;      /* scratch */
    spx_word16_t *x;      /* Far-end input buffer (2N) */
    spx_word16_t *X;      /* Far-end buffer (M+1 frames) in frequency domain */
    spx_word16_t *input;  /* scratch */
    spx_word16_t *y;      /* scratch */
    spx_word16_t *last_y;
    spx_word16_t *Y;      /* scratch */
    spx_word16_t *E;
    spx_word32_t *PHI;    /* scratch */
    spx_word32_t *W;      /* (Background) filter weights */
#ifdef TWO_PATH
    spx_word16_t *foreground; /* Foreground filter weights */
    spx_word32_t Davg1;  /* 1st recursive average of the residual power difference */
    spx_word32_t Davg2;  /* 2nd recursive average of the residual power difference */
    spx_float_t Dvar1;   /* Estimated variance of 1st estimator */
    spx_float_t Dvar2;   /* Estimated variance of 2nd estimator */
#endif
    spx_word32_t *power;  /* Power of the far-end signal */
    spx_float_t *power_1; /* Inverse power of far-end */
    spx_word16_t *wtmp;   /* scratch */
    spx_word32_t *Rf;     /* scratch */
    spx_word32_t *Yf;     /* scratch */
    spx_word32_t *Xf;     /* scratch */
    spx_word32_t *Eh;
    spx_word32_t *Yh;
    spx_float_t Pey;
    spx_float_t Pyy;
    spx_word16_t *window;
    spx_word16_t *prop;
    void *fft_table;
    spx_word16_t *memX, *memD, *memE;
    spx_word16_t preemph;
    spx_word16_t notch_radius;
    spx_mem_t *notch_mem;

    /* Internal playback buffer for async mode */
    spx_int16_t *play_buf;
    int play_buf_pos;
    int play_buf_started;

    /* Diagnostic tracking */
    spx_word32_t diag_Sxx;    /* Last far-end signal power */
    spx_word32_t diag_Syy;    /* Last estimated echo power */
    spx_word32_t diag_See;    /* Last residual error power */
    spx_word32_t diag_Sdd;    /* Last near-end input power */
    spx_word32_t diag_Sff;    /* Last foreground filter error */
};

static inline void filter_dc_notch16(const spx_int16_t *in, spx_word16_t radius, spx_word16_t *out, int len, spx_mem_t *mem, int stride)
{
    int i;
    spx_word16_t den2;
    den2 = radius * radius + .7f * (1 - radius) * (1 - radius);
    for (i = 0; i < len; i++) {
        spx_word16_t vin = in[i * stride];
        spx_word32_t vout = mem[0] + vin;
        mem[0] = mem[1] + 2 * (-vin + radius * vout);
        mem[1] = vin - den2 * vout;
        out[i] = radius * vout;
    }
}

static inline spx_word32_t mdf_inner_prod(const spx_word16_t *x, const spx_word16_t *y, int len)
{
    spx_word32_t sum = 0;
    len >>= 1;
    while (len--) {
        spx_word32_t part = 0;
        part = MAC16_16(part, *x++, *y++);
        part = MAC16_16(part, *x++, *y++);
        sum = ADD32(sum, SHR32(part, 6));
    }
    return sum;
}

static inline void power_spectrum(const spx_word16_t *X, spx_word32_t *ps, int N)
{
    int i, j;
    ps[0] = MULT16_16(X[0], X[0]);
    for (i = 1, j = 1; i < N - 1; i += 2, j++) {
        ps[j] = MULT16_16(X[i], X[i]) + MULT16_16(X[i + 1], X[i + 1]);
    }
    ps[j] = MULT16_16(X[i], X[i]);
}

static inline void power_spectrum_accum(const spx_word16_t *X, spx_word32_t *ps, int N)
{
    int i, j;
    ps[0] += MULT16_16(X[0], X[0]);
    for (i = 1, j = 1; i < N - 1; i += 2, j++) {
        ps[j] += MULT16_16(X[i], X[i]) + MULT16_16(X[i + 1], X[i + 1]);
    }
    ps[j] += MULT16_16(X[i], X[i]);
}

static inline void spectral_mul_accum(const spx_word16_t *X, const spx_word32_t *Y, spx_word16_t *acc, int N, int M)
{
    int i, j;
    for (i = 0; i < N; i++)
        acc[i] = 0;
    for (j = 0; j < M; j++) {
        acc[0] += X[0] * Y[0];
        for (i = 1; i < N - 1; i += 2) {
            acc[i] += (X[i] * Y[i] - X[i + 1] * Y[i + 1]);
            acc[i + 1] += (X[i + 1] * Y[i] + X[i] * Y[i + 1]);
        }
        acc[i] += X[i] * Y[i];
        X += N;
        Y += N;
    }
}
#define spectral_mul_accum16 spectral_mul_accum

static inline void weighted_spectral_mul_conj(const spx_float_t *w, const spx_float_t p, const spx_word16_t *X, const spx_word16_t *Y, spx_word32_t *prod, int N)
{
    int i, j;
    spx_float_t W;
    W = FLOAT_AMULT(p, w[0]);
    prod[0] = FLOAT_MUL32(W, MULT16_16(X[0], Y[0]));
    for (i = 1, j = 1; i < N - 1; i += 2, j++) {
        W = FLOAT_AMULT(p, w[j]);
        prod[i] = FLOAT_MUL32(W, MAC16_16(MULT16_16(X[i], Y[i]), X[i + 1], Y[i + 1]));
        prod[i + 1] = FLOAT_MUL32(W, MAC16_16(MULT16_16(-X[i + 1], Y[i]), X[i], Y[i + 1]));
    }
    W = FLOAT_AMULT(p, w[j]);
    prod[i] = FLOAT_MUL32(W, MULT16_16(X[i], Y[i]));
}

static inline void mdf_adjust_prop(const spx_word32_t *W, int N, int M, int P, spx_word16_t *prop)
{
    int i, j, p_idx;
    spx_word16_t max_sum = 1;
    spx_word32_t prop_sum = 1;
    for (i = 0; i < M; i++) {
        spx_word32_t tmp = 1;
        for (p_idx = 0; p_idx < P; p_idx++)
            for (j = 0; j < N; j++)
                tmp += MULT16_16(EXTRACT16(SHR32(W[p_idx * N * M + i * N + j], 18)), EXTRACT16(SHR32(W[p_idx * N * M + i * N + j], 18)));
        prop[i] = spx_sqrt(tmp);
        if (prop[i] > max_sum)
            max_sum = prop[i];
    }
    for (i = 0; i < M; i++) {
        prop[i] += MULT16_16_Q15(QCONST16(.1f, 15), max_sum);
        prop_sum += EXTEND32(prop[i]);
    }
    for (i = 0; i < M; i++) {
        prop[i] = DIV32(MULT16_16(QCONST16(.99f, 15), prop[i]), prop_sum);
    }
}

/** Creates a new echo canceller state */
EXPORT SpeexEchoState *speex_echo_state_init(int frame_size, int filter_length)
{
    return speex_echo_state_init_mc(frame_size, filter_length, 1, 1);
}

EXPORT SpeexEchoState *speex_echo_state_init_mc(int frame_size, int filter_length, int nb_mic, int nb_speakers)
{
    int i, N, M, C, K;
    SpeexEchoState *st = (SpeexEchoState *)speex_alloc(sizeof(SpeexEchoState));

    st->K = nb_speakers;
    st->C = nb_mic;
    C = st->C;
    K = st->K;

    st->frame_size = frame_size;
    st->window_size = 2 * frame_size;
    N = st->window_size;
    M = st->M = (filter_length + st->frame_size - 1) / frame_size;
    st->cancel_count = 0;
    st->sum_adapt = 0;
    st->saturated = 0;
    st->screwed_up = 0;
    st->sampling_rate = 8000;
    st->spec_average = DIV32_16(SHL32(EXTEND32(st->frame_size), 15), st->sampling_rate);
    st->beta0 = (2.0f * st->frame_size) / st->sampling_rate;
    st->beta_max = (.5f * st->frame_size) / st->sampling_rate;
    st->leak_estimate = 0;
    st->adapt_rate = 1.0f;  /* Default adaptation rate multiplier */

    st->fft_table = spx_fft_init(N);

    st->e = (spx_word16_t *)speex_alloc(C * N * sizeof(spx_word16_t));
    st->x = (spx_word16_t *)speex_alloc(K * N * sizeof(spx_word16_t));
    st->input = (spx_word16_t *)speex_alloc(C * st->frame_size * sizeof(spx_word16_t));
    st->y = (spx_word16_t *)speex_alloc(C * N * sizeof(spx_word16_t));
    st->last_y = (spx_word16_t *)speex_alloc(C * N * sizeof(spx_word16_t));
    st->Yf = (spx_word32_t *)speex_alloc((st->frame_size + 1) * sizeof(spx_word32_t));
    st->Rf = (spx_word32_t *)speex_alloc((st->frame_size + 1) * sizeof(spx_word32_t));
    st->Xf = (spx_word32_t *)speex_alloc((st->frame_size + 1) * sizeof(spx_word32_t));
    st->Yh = (spx_word32_t *)speex_alloc((st->frame_size + 1) * sizeof(spx_word32_t));
    st->Eh = (spx_word32_t *)speex_alloc((st->frame_size + 1) * sizeof(spx_word32_t));

    st->X = (spx_word16_t *)speex_alloc(K * (M + 1) * N * sizeof(spx_word16_t));
    st->Y = (spx_word16_t *)speex_alloc(C * N * sizeof(spx_word16_t));
    st->E = (spx_word16_t *)speex_alloc(C * N * sizeof(spx_word16_t));
    st->W = (spx_word32_t *)speex_alloc(C * K * M * N * sizeof(spx_word32_t));
#ifdef TWO_PATH
    st->foreground = (spx_word16_t *)speex_alloc(M * N * C * K * sizeof(spx_word16_t));
#endif
    st->PHI = (spx_word32_t *)speex_alloc(N * sizeof(spx_word32_t));
    st->power = (spx_word32_t *)speex_alloc((frame_size + 1) * sizeof(spx_word32_t));
    st->power_1 = (spx_float_t *)speex_alloc((frame_size + 1) * sizeof(spx_float_t));
    st->window = (spx_word16_t *)speex_alloc(N * sizeof(spx_word16_t));
    st->prop = (spx_word16_t *)speex_alloc(M * sizeof(spx_word16_t));
    st->wtmp = (spx_word16_t *)speex_alloc(N * sizeof(spx_word16_t));

    for (i = 0; i < N; i++)
        st->window[i] = .5f - .5f * cos(2 * M_PI * i / N);

    for (i = 0; i <= st->frame_size; i++)
        st->power_1[i] = FLOAT_ONE;
    for (i = 0; i < N * M * K * C; i++)
        st->W[i] = 0;
    {
        /* Initialize prop with a SLOW decay to allow finding echoes at longer delays.
         * 
         * Original Speex used exp(-2.4/M) which decays too fast for SDR systems
         * with 60-200ms echo delays. We use exp(-0.5/M) for much slower decay,
         * giving later blocks more weight while still preferring recent blocks.
         * 
         * For M=32: original decay ~0.93, new decay ~0.98
         * Block 0: 100%, Block 5: ~90%, Block 15: ~75%, Block 31: ~55%
         */
        spx_word32_t sum = 0;
        spx_word16_t decay = SHR32(spx_exp(NEG16(DIV32_16(QCONST16(0.5f, 11), M))), 1);
        st->prop[0] = QCONST16(.7f, 15);
        sum = EXTEND32(st->prop[0]);
        for (i = 1; i < M; i++) {
            st->prop[i] = MULT16_16_Q15(st->prop[i - 1], decay);
            sum = ADD32(sum, EXTEND32(st->prop[i]));
        }
        for (i = M - 1; i >= 0; i--) {
            st->prop[i] = DIV32(MULT16_16(QCONST16(.8f, 15), st->prop[i]), sum);
        }
    }

    st->memX = (spx_word16_t *)speex_alloc(K * sizeof(spx_word16_t));
    st->memD = (spx_word16_t *)speex_alloc(C * sizeof(spx_word16_t));
    st->memE = (spx_word16_t *)speex_alloc(C * sizeof(spx_word16_t));
    st->preemph = QCONST16(.9f, 15);
    if (st->sampling_rate < 12000)
        st->notch_radius = QCONST16(.9f, 15);
    else if (st->sampling_rate < 24000)
        st->notch_radius = QCONST16(.982f, 15);
    else
        st->notch_radius = QCONST16(.992f, 15);

    st->notch_mem = (spx_mem_t *)speex_alloc(2 * C * sizeof(spx_mem_t));
    st->adapted = 0;
    st->Pey = st->Pyy = FLOAT_ONE;

#ifdef TWO_PATH
    st->Davg1 = st->Davg2 = 0;
    st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
#endif

    st->play_buf = (spx_int16_t *)speex_alloc(K * (PLAYBACK_DELAY + 1) * st->frame_size * sizeof(spx_int16_t));
    st->play_buf_pos = PLAYBACK_DELAY * st->frame_size;
    /* Start with play_buf_started = 1 so playback frames aren't discarded.
     * The original Speex design expected capture() to be called first, but
     * in SDR systems TX and RX may start in any order. */
    st->play_buf_started = 1;

    return st;
}

/** Resets echo canceller state */
EXPORT void speex_echo_state_reset(SpeexEchoState *st)
{
    int i, M, N, C, K;
    st->cancel_count = 0;
    st->screwed_up = 0;
    N = st->window_size;
    M = st->M;
    C = st->C;
    K = st->K;
    for (i = 0; i < N * M; i++)
        st->W[i] = 0;
#ifdef TWO_PATH
    for (i = 0; i < N * M; i++)
        st->foreground[i] = 0;
#endif
    for (i = 0; i < N * (M + 1); i++)
        st->X[i] = 0;
    for (i = 0; i <= st->frame_size; i++) {
        st->power[i] = 0;
        st->power_1[i] = FLOAT_ONE;
        st->Eh[i] = 0;
        st->Yh[i] = 0;
    }
    for (i = 0; i < st->frame_size; i++) {
        st->last_y[i] = 0;
    }
    for (i = 0; i < N * C; i++) {
        st->E[i] = 0;
    }
    for (i = 0; i < N * K; i++) {
        st->x[i] = 0;
    }
    for (i = 0; i < 2 * C; i++)
        st->notch_mem[i] = 0;
    for (i = 0; i < C; i++)
        st->memD[i] = st->memE[i] = 0;
    for (i = 0; i < K; i++)
        st->memX[i] = 0;

    st->saturated = 0;
    st->adapted = 0;
    st->sum_adapt = 0;
    st->Pey = st->Pyy = FLOAT_ONE;
#ifdef TWO_PATH
    st->Davg1 = st->Davg2 = 0;
    st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
#endif
    for (i = 0; i < (PLAYBACK_DELAY + 1) * st->frame_size; i++)
        st->play_buf[i] = 0;
    st->play_buf_pos = PLAYBACK_DELAY * st->frame_size;
    /* Keep play_buf_started = 1 so playback frames aren't discarded after reset */
    st->play_buf_started = 1;
}

/** Destroys an echo canceller state */
EXPORT void speex_echo_state_destroy(SpeexEchoState *st)
{
    spx_fft_destroy(st->fft_table);

    speex_free(st->e);
    speex_free(st->x);
    speex_free(st->input);
    speex_free(st->y);
    speex_free(st->last_y);
    speex_free(st->Yf);
    speex_free(st->Rf);
    speex_free(st->Xf);
    speex_free(st->Yh);
    speex_free(st->Eh);

    speex_free(st->X);
    speex_free(st->Y);
    speex_free(st->E);
    speex_free(st->W);
#ifdef TWO_PATH
    speex_free(st->foreground);
#endif
    speex_free(st->PHI);
    speex_free(st->power);
    speex_free(st->power_1);
    speex_free(st->window);
    speex_free(st->prop);
    speex_free(st->wtmp);
    speex_free(st->memX);
    speex_free(st->memD);
    speex_free(st->memE);
    speex_free(st->notch_mem);

    speex_free(st->play_buf);
    speex_free(st);
}

EXPORT void speex_echo_capture(SpeexEchoState *st, const spx_int16_t *rec, spx_int16_t *out)
{
    int i;
    st->play_buf_started = 1;
    if (st->play_buf_pos >= st->frame_size) {
        speex_echo_cancellation(st, rec, st->play_buf, out);
        st->play_buf_pos -= st->frame_size;
        for (i = 0; i < st->play_buf_pos; i++)
            st->play_buf[i] = st->play_buf[i + st->frame_size];
    } else {
        speex_warning("No playback frame available (your application is buggy and/or got xruns)");
        if (st->play_buf_pos != 0) {
            speex_warning("internal playback buffer corruption?");
            st->play_buf_pos = 0;
        }
        for (i = 0; i < st->frame_size; i++)
            out[i] = rec[i];
    }
}

EXPORT void speex_echo_playback(SpeexEchoState *st, const spx_int16_t *play)
{
    if (!st->play_buf_started) {
        speex_warning("discarded first playback frame");
        return;
    }
    if (st->play_buf_pos <= PLAYBACK_DELAY * st->frame_size) {
        int i;
        for (i = 0; i < st->frame_size; i++)
            st->play_buf[st->play_buf_pos + i] = play[i];
        st->play_buf_pos += st->frame_size;
        if (st->play_buf_pos <= (PLAYBACK_DELAY - 1) * st->frame_size) {
            speex_warning("Auto-filling the buffer (your application is buggy and/or got xruns)");
            for (i = 0; i < st->frame_size; i++)
                st->play_buf[st->play_buf_pos + i] = play[i];
            st->play_buf_pos += st->frame_size;
        }
    } else {
        speex_warning("Had to discard a playback frame (your application is buggy and/or got xruns)");
    }
}

/** Performs echo cancellation on a frame (deprecated) */
EXPORT void speex_echo_cancel(SpeexEchoState *st, const spx_int16_t *in, const spx_int16_t *far_end, spx_int16_t *out, spx_int32_t *Yout)
{
    (void)Yout;
    speex_echo_cancellation(st, in, far_end, out);
}

/** Performs echo cancellation on a frame */
EXPORT void speex_echo_cancellation(SpeexEchoState *st, const spx_int16_t *in, const spx_int16_t *far_end, spx_int16_t *out)
{
    int i, j, chan, speak;
    int N, M, C, K;
    spx_word32_t Syy, See, Sxx, Sdd, Sff;
#ifdef TWO_PATH
    spx_word32_t Dbf;
    int update_foreground;
#endif
    spx_word32_t Sey;
    spx_word16_t ss, ss_1;
    spx_float_t Pey = FLOAT_ONE, Pyy = FLOAT_ONE;
    spx_float_t alpha, alpha_1;
    spx_word16_t RER;
    spx_word32_t tmp32;

    N = st->window_size;
    M = st->M;
    C = st->C;
    K = st->K;

    st->cancel_count++;
    ss = .35f / M;
    ss_1 = 1 - ss;

    for (chan = 0; chan < C; chan++) {
        filter_dc_notch16(in + chan, st->notch_radius, st->input + chan * st->frame_size, st->frame_size, st->notch_mem + 2 * chan, C);
        for (i = 0; i < st->frame_size; i++) {
            spx_word32_t tmp32_local;
            tmp32_local = SUB32(EXTEND32(st->input[chan * st->frame_size + i]), EXTEND32(MULT16_16_P15(st->preemph, st->memD[chan])));
            st->memD[chan] = st->input[chan * st->frame_size + i];
            st->input[chan * st->frame_size + i] = EXTRACT16(tmp32_local);
        }
    }

    for (speak = 0; speak < K; speak++) {
        for (i = 0; i < st->frame_size; i++) {
            spx_word32_t tmp32_local;
            st->x[speak * N + i] = st->x[speak * N + i + st->frame_size];
            tmp32_local = SUB32(EXTEND32(far_end[i * K + speak]), EXTEND32(MULT16_16_P15(st->preemph, st->memX[speak])));
            st->x[speak * N + i + st->frame_size] = EXTRACT16(tmp32_local);
            st->memX[speak] = far_end[i * K + speak];
        }
    }

    for (speak = 0; speak < K; speak++) {
        for (j = M - 1; j >= 0; j--) {
            for (i = 0; i < N; i++)
                st->X[(j + 1) * N * K + speak * N + i] = st->X[j * N * K + speak * N + i];
        }
        spx_fft(st->fft_table, st->x + speak * N, &st->X[speak * N]);
    }

    Sxx = 0;
    for (speak = 0; speak < K; speak++) {
        Sxx += mdf_inner_prod(st->x + speak * N + st->frame_size, st->x + speak * N + st->frame_size, st->frame_size);
        power_spectrum_accum(st->X + speak * N, st->Xf, N);
    }

    Sff = 0;
    for (chan = 0; chan < C; chan++) {
#ifdef TWO_PATH
        spectral_mul_accum16(st->X, st->foreground + chan * N * K * M, st->Y + chan * N, N, M * K);
        spx_ifft(st->fft_table, st->Y + chan * N, st->e + chan * N);
        for (i = 0; i < st->frame_size; i++)
            st->e[chan * N + i] = SUB16(st->input[chan * st->frame_size + i], st->e[chan * N + i + st->frame_size]);
        Sff += mdf_inner_prod(st->e + chan * N, st->e + chan * N, st->frame_size);
#endif
    }

    if (st->adapted)
        mdf_adjust_prop(st->W, N, M, C * K, st->prop);
    if (st->saturated == 0) {
        for (chan = 0; chan < C; chan++) {
            for (speak = 0; speak < K; speak++) {
                for (j = M - 1; j >= 0; j--) {
                    weighted_spectral_mul_conj(st->power_1, FLOAT_SHL(PSEUDOFLOAT(st->prop[j]), -15), &st->X[(j + 1) * N * K + speak * N], st->E + chan * N, st->PHI, N);
                    for (i = 0; i < N; i++)
                        st->W[chan * N * K * M + j * N * K + speak * N + i] += st->PHI[i];
                }
            }
        }
    } else {
        st->saturated--;
    }

    for (chan = 0; chan < C; chan++) {
        for (speak = 0; speak < K; speak++) {
            for (j = 0; j < M; j++) {
                if (j == 0 || st->cancel_count % (M - 1) == j - 1) {
                    spx_ifft(st->fft_table, &st->W[chan * N * K * M + j * N * K + speak * N], st->wtmp);
                    for (i = st->frame_size; i < N; i++) {
                        st->wtmp[i] = 0;
                    }
                    spx_fft(st->fft_table, st->wtmp, &st->W[chan * N * K * M + j * N * K + speak * N]);
                }
            }
        }
    }

    for (i = 0; i <= st->frame_size; i++)
        st->Rf[i] = st->Yf[i] = st->Xf[i] = 0;

    Dbf = 0;
    See = 0;
#ifdef TWO_PATH
    for (chan = 0; chan < C; chan++) {
        spectral_mul_accum(st->X, st->W + chan * N * K * M, st->Y + chan * N, N, M * K);
        spx_ifft(st->fft_table, st->Y + chan * N, st->y + chan * N);
        for (i = 0; i < st->frame_size; i++)
            st->e[chan * N + i] = SUB16(st->e[chan * N + i + st->frame_size], st->y[chan * N + i + st->frame_size]);
        Dbf += 10 + mdf_inner_prod(st->e + chan * N, st->e + chan * N, st->frame_size);
        for (i = 0; i < st->frame_size; i++)
            st->e[chan * N + i] = SUB16(st->input[chan * st->frame_size + i], st->y[chan * N + i + st->frame_size]);
        See += mdf_inner_prod(st->e + chan * N, st->e + chan * N, st->frame_size);
    }
#endif

#ifndef TWO_PATH
    Sff = See;
#endif

#ifdef TWO_PATH
    st->Davg1 = ADD32(MULT16_32_Q15(QCONST16(.6f, 15), st->Davg1), MULT16_32_Q15(QCONST16(.4f, 15), SUB32(Sff, See)));
    st->Davg2 = ADD32(MULT16_32_Q15(QCONST16(.85f, 15), st->Davg2), MULT16_32_Q15(QCONST16(.15f, 15), SUB32(Sff, See)));
    st->Dvar1 = FLOAT_ADD(FLOAT_MULT(VAR1_SMOOTH, st->Dvar1), FLOAT_MUL32U(MULT16_32_Q15(QCONST16(.4f, 15), Sff), MULT16_32_Q15(QCONST16(.4f, 15), Dbf)));
    st->Dvar2 = FLOAT_ADD(FLOAT_MULT(VAR2_SMOOTH, st->Dvar2), FLOAT_MUL32U(MULT16_32_Q15(QCONST16(.15f, 15), Sff), MULT16_32_Q15(QCONST16(.15f, 15), Dbf)));

    update_foreground = 0;
    if (FLOAT_GT(FLOAT_MUL32U(SUB32(Sff, See), ABS32(SUB32(Sff, See))), FLOAT_MUL32U(Sff, Dbf)))
        update_foreground = 1;
    else if (FLOAT_GT(FLOAT_MUL32U(st->Davg1, ABS32(st->Davg1)), FLOAT_MULT(VAR1_UPDATE, (st->Dvar1))))
        update_foreground = 1;
    else if (FLOAT_GT(FLOAT_MUL32U(st->Davg2, ABS32(st->Davg2)), FLOAT_MULT(VAR2_UPDATE, (st->Dvar2))))
        update_foreground = 1;

    if (update_foreground) {
        st->Davg1 = st->Davg2 = 0;
        st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
        for (i = 0; i < N * M * C * K; i++)
            st->foreground[i] = EXTRACT16(PSHR32(st->W[i], 16));
        for (chan = 0; chan < C; chan++)
            for (i = 0; i < st->frame_size; i++)
                st->e[chan * N + i + st->frame_size] = MULT16_16_Q15(st->window[i + st->frame_size], st->e[chan * N + i + st->frame_size]) + MULT16_16_Q15(st->window[i], st->y[chan * N + i + st->frame_size]);
    } else {
        int reset_background = 0;
        if (FLOAT_GT(FLOAT_MUL32U(NEG32(SUB32(Sff, See)), ABS32(SUB32(Sff, See))), FLOAT_MULT(VAR_BACKTRACK, FLOAT_MUL32U(Sff, Dbf))))
            reset_background = 1;
        if (FLOAT_GT(FLOAT_MUL32U(NEG32(st->Davg1), ABS32(st->Davg1)), FLOAT_MULT(VAR_BACKTRACK, st->Dvar1)))
            reset_background = 1;
        if (FLOAT_GT(FLOAT_MUL32U(NEG32(st->Davg2), ABS32(st->Davg2)), FLOAT_MULT(VAR_BACKTRACK, st->Dvar2)))
            reset_background = 1;
        if (reset_background) {
            for (i = 0; i < N * M * C * K; i++)
                st->W[i] = SHL32(EXTEND32(st->foreground[i]), 16);
            for (chan = 0; chan < C; chan++) {
                for (i = 0; i < st->frame_size; i++)
                    st->y[chan * N + i + st->frame_size] = st->e[chan * N + i + st->frame_size];
                for (i = 0; i < st->frame_size; i++)
                    st->e[chan * N + i] = SUB16(st->input[chan * st->frame_size + i], st->y[chan * N + i + st->frame_size]);
            }
            See = Sff;
            st->Davg1 = st->Davg2 = 0;
            st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
        }
    }
#endif

    Sey = Syy = Sdd = 0;
    for (chan = 0; chan < C; chan++) {
        for (i = 0; i < st->frame_size; i++) {
            spx_word32_t tmp_out;
#ifdef TWO_PATH
            tmp_out = SUB32(EXTEND32(st->input[chan * st->frame_size + i]), EXTEND32(st->e[chan * N + i + st->frame_size]));
#else
            tmp_out = SUB32(EXTEND32(st->input[chan * st->frame_size + i]), EXTEND32(st->y[chan * N + i + st->frame_size]));
#endif
            tmp_out = ADD32(tmp_out, EXTEND32(MULT16_16_P15(st->preemph, st->memE[chan])));
            if (in[i * C + chan] <= -32000 || in[i * C + chan] >= 32000) {
                if (st->saturated == 0)
                    st->saturated = 1;
            }
            out[i * C + chan] = WORD2INT(tmp_out);
            st->memE[chan] = tmp_out;
        }

        for (i = 0; i < st->frame_size; i++) {
            st->e[chan * N + i + st->frame_size] = st->e[chan * N + i];
            st->e[chan * N + i] = 0;
        }

        Sey += mdf_inner_prod(st->e + chan * N + st->frame_size, st->y + chan * N + st->frame_size, st->frame_size);
        Syy += mdf_inner_prod(st->y + chan * N + st->frame_size, st->y + chan * N + st->frame_size, st->frame_size);
        Sdd += mdf_inner_prod(st->input + chan * st->frame_size, st->input + chan * st->frame_size, st->frame_size);

        spx_fft(st->fft_table, st->e + chan * N, st->E + chan * N);
        for (i = 0; i < st->frame_size; i++)
            st->y[i + chan * N] = 0;
        spx_fft(st->fft_table, st->y + chan * N, st->Y + chan * N);

        power_spectrum_accum(st->E + chan * N, st->Rf, N);
        power_spectrum_accum(st->Y + chan * N, st->Yf, N);
    }

    if (!(Syy >= 0 && Sxx >= 0 && See >= 0)) {
        st->screwed_up += 50;
        for (i = 0; i < st->frame_size * C; i++)
            out[i] = 0;
    } else if (SHR32(Sff, 2) > ADD32(Sdd, SHR32(MULT16_16(N, 10000), 6))) {
        st->screwed_up++;
    } else {
        st->screwed_up = 0;
    }
    if (st->screwed_up >= 50) {
        speex_warning("The echo canceller started acting funny and got slapped (reset). It swears it will behave now.");
        speex_echo_state_reset(st);
        return;
    }

    See = MAX32(See, SHR32(MULT16_16(N, 100), 6));

    for (speak = 0; speak < K; speak++) {
        Sxx += mdf_inner_prod(st->x + speak * N + st->frame_size, st->x + speak * N + st->frame_size, st->frame_size);
        power_spectrum_accum(st->X + speak * N, st->Xf, N);
    }

    for (j = 0; j <= st->frame_size; j++)
        st->power[j] = MULT16_32_Q15(ss_1, st->power[j]) + 1 + MULT16_32_Q15(ss, st->Xf[j]);

    for (j = st->frame_size; j >= 0; j--) {
        spx_float_t Eh_local, Yh_local;
        Eh_local = PSEUDOFLOAT(st->Rf[j] - st->Eh[j]);
        Yh_local = PSEUDOFLOAT(st->Yf[j] - st->Yh[j]);
        Pey = FLOAT_ADD(Pey, FLOAT_MULT(Eh_local, Yh_local));
        Pyy = FLOAT_ADD(Pyy, FLOAT_MULT(Yh_local, Yh_local));
        st->Eh[j] = (1 - st->spec_average) * st->Eh[j] + st->spec_average * st->Rf[j];
        st->Yh[j] = (1 - st->spec_average) * st->Yh[j] + st->spec_average * st->Yf[j];
    }

    Pyy = FLOAT_SQRT(Pyy);
    Pey = FLOAT_DIVU(Pey, Pyy);

    tmp32 = MULT16_32_Q15(st->beta0, Syy);
    if (tmp32 > MULT16_32_Q15(st->beta_max, See))
        tmp32 = MULT16_32_Q15(st->beta_max, See);
    alpha = FLOAT_DIV32(tmp32, See);
    alpha_1 = FLOAT_SUB(FLOAT_ONE, alpha);
    st->Pey = FLOAT_ADD(FLOAT_MULT(alpha_1, st->Pey), FLOAT_MULT(alpha, Pey));
    st->Pyy = FLOAT_ADD(FLOAT_MULT(alpha_1, st->Pyy), FLOAT_MULT(alpha, Pyy));
    if (FLOAT_LT(st->Pyy, FLOAT_ONE))
        st->Pyy = FLOAT_ONE;
    if (FLOAT_LT(st->Pey, FLOAT_MULT(MIN_LEAK, st->Pyy)))
        st->Pey = FLOAT_MULT(MIN_LEAK, st->Pyy);
    if (FLOAT_GT(st->Pey, st->Pyy))
        st->Pey = st->Pyy;
    st->leak_estimate = FLOAT_EXTRACT16(FLOAT_SHL(FLOAT_DIVU(st->Pey, st->Pyy), 14));
    if (st->leak_estimate > 16383)
        st->leak_estimate = 32767;
    else
        st->leak_estimate = SHL16(st->leak_estimate, 1);

    RER = (.0001f * Sxx + 3.f * MULT16_32_Q15(st->leak_estimate, Syy)) / See;
    if (RER < Sey * Sey / (1 + See * Syy))
        RER = Sey * Sey / (1 + See * Syy);
    if (RER > .5f)
        RER = .5f;

    /* Check if filter has adapted enough to switch to adapted mode
     * 
     * The original SpeexDSP used M << 15 which doesn't work in floating-point.
     * We use a practical threshold based on:
     * - Minimum adaptation time (~5-10 seconds of active signal)
     * - Low leak estimate indicating the filter has found the echo path
     * 
     * For VoIP with SDR latency, we use relaxed thresholds since
     * the long delay path makes perfect convergence harder.
     */
    {
        spx_word32_t adapt_threshold = 30;  /* Lower threshold for faster adaptation */
        
        if (!st->adapted && st->sum_adapt > adapt_threshold && 
            st->leak_estimate < 0.85f) {  /* Relaxed for high-latency paths */
            st->adapted = 1;
        }
    }

    if (st->adapted) {
        /* Adapted mode: use leak-based adaptation with rate multiplier */
        for (i = 0; i <= st->frame_size; i++) {
            spx_word32_t r, e_local;
            r = MULT16_32_Q15(st->leak_estimate, SHL32(st->Yf[i], 3));
            e_local = SHL32(st->Rf[i], 3) + 1;
            if (r > .5f * e_local)
                r = .5f * e_local;
            r = MULT16_32_Q15(QCONST16(.7f, 15), r) + MULT16_32_Q15(QCONST16(.3f, 15), (spx_word32_t)(MULT16_32_Q15(RER, e_local)));
            /* Apply user-configurable adaptation rate multiplier to continue learning */
            r = (spx_word32_t)(r * st->adapt_rate);
            st->power_1[i] = FLOAT_SHL(FLOAT_DIV32_FLOAT(r, FLOAT_MUL32U(e_local, st->power[i] + 10)), WEIGHT_SHIFT + 16);
        }
        /* Track continued adaptation progress */
        st->sum_adapt = ADD32(st->sum_adapt, (spx_word16_t)(st->adapt_rate));
    } else {
        /* Learning mode: use power-based adaptation with rate multiplier */
        spx_word16_t adapt_rate = 0;

        if (Sxx > SHR32(MULT16_16(N, 1000), 6)) {
            tmp32 = MULT16_32_Q15(QCONST16(.25f, 15), Sxx);
            if (tmp32 > .25f * See)
                tmp32 = .25f * See;
            adapt_rate = FLOAT_EXTRACT16(FLOAT_SHL(FLOAT_DIV32(tmp32, See), 15));
            /* Apply user-configurable adaptation rate multiplier */
            adapt_rate = (spx_word16_t)(adapt_rate * st->adapt_rate);
        }
        for (i = 0; i <= st->frame_size; i++)
            st->power_1[i] = FLOAT_SHL(FLOAT_DIV32(EXTEND32(adapt_rate), ADD32(st->power[i], 10)), WEIGHT_SHIFT + 1);

        st->sum_adapt = ADD32(st->sum_adapt, adapt_rate);
    }

    for (i = 0; i < st->frame_size; i++)
        st->last_y[i] = st->last_y[st->frame_size + i];
    if (st->adapted) {
        for (i = 0; i < st->frame_size; i++)
            st->last_y[st->frame_size + i] = in[i] - out[i];
    }

    /* Store diagnostic values for monitoring */
    st->diag_Sxx = Sxx;
    st->diag_Syy = Syy;
    st->diag_See = See;
    st->diag_Sdd = Sdd;
    st->diag_Sff = Sff;
}

EXPORT void speex_echo_get_stats(SpeexEchoState *st, speex_echo_stats_t *stats)
{
    spx_word32_t epsilon = 1;  /* Avoid division by zero */
    int i, j;
    int N, M;
    int peak_block = 0;
    spx_word32_t max_energy = 1;  /* Use fixed-point like mdf_adjust_prop */

    if (!st || !stats)
        return;

    /* Initialize all fields to safe defaults */
    memset(stats, 0, sizeof(*stats));

    stats->adapted = st->adapted;
    stats->saturated = st->saturated;
    stats->screwed_up = st->screwed_up;
    stats->leak_estimate = st->leak_estimate;
    stats->sum_adapt = st->sum_adapt;

    /* Convert power values to displayable floats */
    stats->Sxx = (float)st->diag_Sxx;
    stats->Syy = (float)st->diag_Syy;
    stats->See = (float)st->diag_See;
    stats->Sdd = (float)st->diag_Sdd;
    stats->Sff = (float)st->diag_Sff;

    /* Calculate ERLE (Echo Return Loss Enhancement) in dB
     * ERLE = 10 * log10(Sdd / See) when echo is being cancelled
     * Higher values = better echo cancellation
     */
    if (st->diag_Sdd > epsilon && st->diag_See > epsilon) {
        stats->erle_db = 10.0f * log10f((float)st->diag_Sdd / (float)st->diag_See);
        if (stats->erle_db < 0.0f)
            stats->erle_db = 0.0f;  /* ERLE can't be negative */
        if (stats->erle_db > 50.0f)
            stats->erle_db = 50.0f; /* Cap at reasonable max */
    } else {
        stats->erle_db = 0.0f;
    }

    /* Instantaneous ERLE based on foreground filter */
    if (st->diag_Sff > epsilon && st->diag_See > epsilon) {
        stats->erle_inst_db = 10.0f * log10f((float)st->diag_Sff / (float)st->diag_See);
        if (stats->erle_inst_db < -20.0f)
            stats->erle_inst_db = -20.0f;
        if (stats->erle_inst_db > 50.0f)
            stats->erle_inst_db = 50.0f;
    } else {
        stats->erle_inst_db = 0.0f;
    }

    /* Find peak filter block - indicates where the algorithm found the echo.
     * Use the same calculation as mdf_adjust_prop() to ensure correctness.
     * For single channel (C=1, K=1), W indexing is: W[block * N + freq_bin]
     */
    N = st->window_size;
    M = st->M;
    for (i = 0; i < M; i++) {
        spx_word32_t tmp = 1;
        for (j = 0; j < N; j++) {
            /* Same calculation as mdf_adjust_prop: shift by 18, extract 16 bits, square */
            spx_word16_t w = EXTRACT16(SHR32(st->W[i * N + j], 18));
            tmp += MULT16_16(w, w);
        }
        if (tmp > max_energy) {
            max_energy = tmp;
            peak_block = i;
        }
    }
    stats->peak_block = peak_block;
    /* Convert block index to milliseconds: block * frame_size / sample_rate * 1000 */
    stats->peak_block_ms = (float)peak_block * (float)st->frame_size / (float)st->sampling_rate * 1000.0f;
}

/* Compute spectrum of estimated echo for use in an echo post-filter (preprocessor) */
void speex_echo_get_residual(SpeexEchoState *st, spx_word32_t *residual_echo, int len)
{
    int i;
    spx_word16_t leak2;
    int N;
    
    (void)len; /* unused, we use st->frame_size */
    
    N = st->window_size;

    /* Apply hanning window to last echo estimate */
    for (i=0;i<N;i++)
        st->y[i] = MULT16_16_Q15(st->window[i],st->last_y[i]);
      
    /* Compute power spectrum of the echo */
    spx_fft(st->fft_table, st->y, st->Y);
    power_spectrum(st->Y, residual_echo, N);
      
    /* Scale by leak estimate */
    if (st->leak_estimate>.5)
        leak2 = 1;
    else
        leak2 = 2*st->leak_estimate;
    
    /* Estimate residual echo */
    for (i=0;i<=st->frame_size;i++)
        residual_echo[i] = (spx_int32_t)MULT16_32_Q15(leak2,residual_echo[i]);
}

EXPORT int speex_echo_ctl(SpeexEchoState *st, int request, void *ptr)
{
    switch (request) {
    case SPEEX_ECHO_GET_FRAME_SIZE:
        (*(int *)ptr) = st->frame_size;
        break;
    case SPEEX_ECHO_SET_SAMPLING_RATE:
        st->sampling_rate = (*(int *)ptr);
        st->spec_average = DIV32_16(SHL32(EXTEND32(st->frame_size), 15), st->sampling_rate);
        st->beta0 = (2.0f * st->frame_size) / st->sampling_rate;
        st->beta_max = (.5f * st->frame_size) / st->sampling_rate;
        if (st->sampling_rate < 12000)
            st->notch_radius = QCONST16(.9f, 15);
        else if (st->sampling_rate < 24000)
            st->notch_radius = QCONST16(.982f, 15);
        else
            st->notch_radius = QCONST16(.992f, 15);
        break;
    case SPEEX_ECHO_GET_SAMPLING_RATE:
        (*(int *)ptr) = st->sampling_rate;
        break;
    case SPEEX_ECHO_GET_IMPULSE_RESPONSE_SIZE:
        *((spx_int32_t *)ptr) = st->M * st->frame_size;
        break;
    case SPEEX_ECHO_GET_IMPULSE_RESPONSE:
        {
            int M_local = st->M, N_local = st->window_size, n = st->frame_size, i_local, j_local;
            spx_int32_t *filt = (spx_int32_t *)ptr;
            for (j_local = 0; j_local < M_local; j_local++) {
                spx_ifft(st->fft_table, &st->W[j_local * N_local], st->wtmp);
                for (i_local = 0; i_local < n; i_local++)
                    filt[j_local * n + i_local] = PSHR32(MULT16_16(32767, st->wtmp[i_local]), WEIGHT_SHIFT);
            }
        }
        break;
    case SPEEX_ECHO_GET_ADAPTED:
        (*(int *)ptr) = st->adapted;
        break;
    case SPEEX_ECHO_GET_LEAK_ESTIMATE:
        (*(float *)ptr) = st->leak_estimate;
        break;
    case SPEEX_ECHO_SET_ADAPT_RATE:
        {
            float rate = *(float *)ptr;
            if (rate < 0.1f) rate = 0.1f;   /* Minimum 0.1x */
            if (rate > 10.0f) rate = 10.0f; /* Maximum 10x */
            st->adapt_rate = rate;
        }
        break;
    case SPEEX_ECHO_GET_ADAPT_RATE:
        (*(float *)ptr) = st->adapt_rate;
        break;
    default:
        speex_warning_int("Unknown speex_echo_ctl request: ", request);
        return -1;
    }
    return 0;
}

