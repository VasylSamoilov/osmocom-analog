/* Copyright (C) 2003 Epic Games (written by Jean-Marc Valin)
   Copyright (C) 2004-2006 Epic Games 
   
   File: preprocess.c
   Preprocessor with denoising based on the algorithm by Ephraim and Malah

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

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "speex_preprocess.h"
#include "speex_echo.h"
#include "arch.h"
#include "fftwrap.h"
#include "filterbank.h"
#include "math_approx.h"
#include "os_support.h"

#ifndef M_PI
#define M_PI 3.14159263
#endif

#define NB_BANDS 24

#define SPEECH_PROB_START_DEFAULT       QCONST16(0.35f,15)
#define SPEECH_PROB_CONTINUE_DEFAULT    QCONST16(0.20f,15)
#define NOISE_SUPPRESS_DEFAULT       -15
#define ECHO_SUPPRESS_DEFAULT        -40
#define ECHO_SUPPRESS_ACTIVE_DEFAULT -15

#ifndef NULL
#define NULL 0
#endif

#define SQR(x) ((x)*(x))
#define SQR16_Q15(x) (MULT16_16_Q15((x),(x)))

#define DIV32_16_Q8(a,b) ((a)/(b))
#define DIV32_16_Q15(a,b) ((a)/(b))
#define SNR_SCALING 1.f
#define SNR_SCALING_1 1.f
#define SNR_SHIFT 0
#define FRAC_SCALING 1.f
#define FRAC_SCALING_1 1.f
#define FRAC_SHIFT 0
#define NOISE_SHIFT 0
#define EXPIN_SCALING 1.f
#define EXPIN_SCALING_1 1.f
#define EXPOUT_SCALING_1 1.f

/** Speex pre-processor state. */
struct SpeexPreprocessState_ {
   /* Basic info */
   int    frame_size;
   int    ps_size;
   int    sampling_rate;
   int    nbands;
   FilterBank *bank;
   
   /* Parameters */
   int    denoise_enabled;
   int    vad_enabled;
   int    dereverb_enabled;
   spx_word16_t  reverb_decay;
   spx_word16_t  reverb_level;
   spx_word16_t speech_prob_start;
   spx_word16_t speech_prob_continue;
   int    noise_suppress;
   int    echo_suppress;
   int    echo_suppress_active;
   SpeexEchoState *echo_state;
   
   spx_word16_t speech_prob;

   /* DSP-related arrays */
   spx_word16_t *frame;
   spx_word16_t *ft;
   spx_word32_t *ps;
   spx_word16_t *gain2;
   spx_word16_t *gain_floor;
   spx_word16_t *window;
   spx_word32_t *noise;
   spx_word32_t *reverb_estimate;
   spx_word32_t *old_ps;
   spx_word16_t *gain;
   spx_word16_t *prior;
   spx_word16_t *post;

   spx_word32_t *S;
   spx_word32_t *Smin;
   spx_word32_t *Stmp;
   int *update_prob;

   spx_word16_t *zeta;
   spx_word32_t *echo_noise;
   spx_word32_t *residual_echo;

   /* Misc */
   spx_word16_t *inbuf;
   spx_word16_t *outbuf;

   /* AGC stuff */
   int    agc_enabled;
   float  agc_level;
   float  loudness_accum;
   float *loudness_weight;
   float  loudness;
   float  agc_gain;
   float  max_gain;
   float  max_increase_step;
   float  max_decrease_step;
   float  prev_loudness;
   float  init_max;

   int    nb_adapt;
   int    was_speech;
   int    min_count;
   void  *fft_lookup;
};

static void conj_window(spx_word16_t *w, int len)
{
   int i;
   for (i=0;i<len;i++)
   {
      spx_word16_t tmp;
      spx_word16_t x = DIV32_16(MULT16_16(QCONST16(4.f,13),i),len);
      int inv=0;
      if (x<QCONST16(1.f,13))
      {
      } else if (x<QCONST16(2.f,13))
      {
         x=QCONST16(2.f,13)-x;
         inv=1;
      } else if (x<QCONST16(3.f,13))
      {
         x=x-QCONST16(2.f,13);
         inv=1;
      } else {
         x=QCONST16(2.f,13)-x+QCONST16(2.f,13);
      }
      x = MULT16_16_Q14(QCONST16(1.271903f,14), x);
      tmp = SQR16_Q15(QCONST16(.5f,15)-MULT16_16_P15(QCONST16(.5f,15),spx_cos_norm(SHL32(EXTEND32(x),2))));
      if (inv)
         tmp=SUB16(Q15_ONE,tmp);
      w[i]=spx_sqrt(SHL32(EXTEND32(tmp),15));
   }
}

static inline spx_word32_t hypergeom_gain(spx_word32_t xx)
{
   int ind;
   float integer, frac;
   float x;
   static const float table[21] = {
      0.82157f, 1.02017f, 1.20461f, 1.37534f, 1.53363f, 1.68092f, 1.81865f,
      1.94811f, 2.07038f, 2.18638f, 2.29688f, 2.40255f, 2.50391f, 2.60144f,
      2.69551f, 2.78647f, 2.87458f, 2.96015f, 3.04333f, 3.12431f, 3.20326f};
   x = EXPIN_SCALING_1*xx;
   integer = floor(2*x);
   ind = (int)integer;
   if (ind<0)
      return FRAC_SCALING;
   if (ind>19)
      return FRAC_SCALING*(1+.1296/x);
   frac = 2*x-integer;
   return FRAC_SCALING*((1-frac)*table[ind] + frac*table[ind+1])/sqrt(x+.0001f);
}

static inline spx_word16_t qcurve(spx_word16_t x)
{
   return 1.f/(1.f+.15f/(SNR_SCALING_1*x));
}

static void compute_gain_floor(int noise_suppress, int effective_echo_suppress, 
                               spx_word32_t *noise, spx_word32_t *echo, 
                               spx_word16_t *gain_floor, int len)
{
   int i;
   float echo_floor;
   float noise_floor;

   noise_floor = exp(.2302585f*noise_suppress);
   echo_floor = exp(.2302585f*effective_echo_suppress);

   for (i=0;i<len;i++)
      gain_floor[i] = FRAC_SCALING*sqrt(noise_floor*PSHR32(noise[i],NOISE_SHIFT) + echo_floor*echo[i])/sqrt(1+PSHR32(noise[i],NOISE_SHIFT) + echo[i]);
}

/* Get residual echo estimate from echo canceller */
void speex_echo_get_residual(SpeexEchoState *st, spx_word32_t *Yout, int len);


SpeexPreprocessState *speex_preprocess_state_init(int frame_size, int sampling_rate)
{
   int i;
   int N, N3, N4, M;

   SpeexPreprocessState *st = (SpeexPreprocessState *)speex_alloc(sizeof(SpeexPreprocessState));
   st->frame_size = frame_size;
   st->ps_size = st->frame_size;

   N = st->ps_size;
   N3 = 2*N - st->frame_size;
   N4 = st->frame_size - N3;
   (void)N4;
   
   st->sampling_rate = sampling_rate;
   st->denoise_enabled = 1;
   st->vad_enabled = 0;
   st->dereverb_enabled = 0;
   st->reverb_decay = 0;
   st->reverb_level = 0;
   st->noise_suppress = NOISE_SUPPRESS_DEFAULT;
   st->echo_suppress = ECHO_SUPPRESS_DEFAULT;
   st->echo_suppress_active = ECHO_SUPPRESS_ACTIVE_DEFAULT;

   st->speech_prob_start = SPEECH_PROB_START_DEFAULT;
   st->speech_prob_continue = SPEECH_PROB_CONTINUE_DEFAULT;

   st->echo_state = NULL;
   
   st->nbands = NB_BANDS;
   M = st->nbands;
   st->bank = filterbank_new(M, sampling_rate, N, 1);
   
   st->frame = (spx_word16_t*)speex_alloc(2*N*sizeof(spx_word16_t));
   st->window = (spx_word16_t*)speex_alloc(2*N*sizeof(spx_word16_t));
   st->ft = (spx_word16_t*)speex_alloc(2*N*sizeof(spx_word16_t));
   
   st->ps = (spx_word32_t*)speex_alloc((N+M)*sizeof(spx_word32_t));
   st->noise = (spx_word32_t*)speex_alloc((N+M)*sizeof(spx_word32_t));
   st->echo_noise = (spx_word32_t*)speex_alloc((N+M)*sizeof(spx_word32_t));
   st->residual_echo = (spx_word32_t*)speex_alloc((N+M)*sizeof(spx_word32_t));
   st->reverb_estimate = (spx_word32_t*)speex_alloc((N+M)*sizeof(spx_word32_t));
   st->old_ps = (spx_word32_t*)speex_alloc((N+M)*sizeof(spx_word32_t));
   st->prior = (spx_word16_t*)speex_alloc((N+M)*sizeof(spx_word16_t));
   st->post = (spx_word16_t*)speex_alloc((N+M)*sizeof(spx_word16_t));
   st->gain = (spx_word16_t*)speex_alloc((N+M)*sizeof(spx_word16_t));
   st->gain2 = (spx_word16_t*)speex_alloc((N+M)*sizeof(spx_word16_t));
   st->gain_floor = (spx_word16_t*)speex_alloc((N+M)*sizeof(spx_word16_t));
   st->zeta = (spx_word16_t*)speex_alloc((N+M)*sizeof(spx_word16_t));
   
   st->S = (spx_word32_t*)speex_alloc(N*sizeof(spx_word32_t));
   st->Smin = (spx_word32_t*)speex_alloc(N*sizeof(spx_word32_t));
   st->Stmp = (spx_word32_t*)speex_alloc(N*sizeof(spx_word32_t));
   st->update_prob = (int*)speex_alloc(N*sizeof(int));
   
   st->inbuf = (spx_word16_t*)speex_alloc(N3*sizeof(spx_word16_t));
   st->outbuf = (spx_word16_t*)speex_alloc(N3*sizeof(spx_word16_t));

   conj_window(st->window, 2*N3);
   for (i=2*N3;i<2*st->ps_size;i++)
      st->window[i]=Q15_ONE;
   
   if (N4>0)
   {
      for (i=N3-1;i>=0;i--)
      {
         st->window[i+N3+N4]=st->window[i+N3];
         st->window[i+N3]=1;
      }
   }
   for (i=0;i<N+M;i++)
   {
      st->noise[i]=QCONST32(1.f,NOISE_SHIFT);
      st->reverb_estimate[i]=0;
      st->old_ps[i]=1;
      st->gain[i]=Q15_ONE;
      st->post[i]=SHL16(1, SNR_SHIFT);
      st->prior[i]=SHL16(1, SNR_SHIFT);
   }

   for (i=0;i<N;i++)
      st->update_prob[i] = 1;
   for (i=0;i<N3;i++)
   {
      st->inbuf[i]=0;
      st->outbuf[i]=0;
   }

   st->agc_enabled = 0;
   st->agc_level = 8000;
   st->loudness_weight = (float*)speex_alloc(N*sizeof(float));
   for (i=0;i<N;i++)
   {
      float ff=((float)i)*.5*sampling_rate/((float)N);
      st->loudness_weight[i] = .35f-.35f*ff/16000.f+.73f*exp(-.5f*(ff-3800)*(ff-3800)/9e5f);
      if (st->loudness_weight[i]<.01f)
         st->loudness_weight[i]=.01f;
      st->loudness_weight[i] *= st->loudness_weight[i];
   }
   st->loudness = 1e-15;
   st->agc_gain = 1;
   st->max_gain = 30;
   st->max_increase_step = exp(0.11513f * 12.*st->frame_size / st->sampling_rate);
   st->max_decrease_step = exp(-0.11513f * 40.*st->frame_size / st->sampling_rate);
   st->prev_loudness = 1;
   st->init_max = 1;
   st->was_speech = 0;

   st->fft_lookup = spx_fft_init(2*N);

   st->nb_adapt=0;
   st->min_count=0;
   return st;
}

void speex_preprocess_state_destroy(SpeexPreprocessState *st)
{
   speex_free(st->frame);
   speex_free(st->ft);
   speex_free(st->ps);
   speex_free(st->gain2);
   speex_free(st->gain_floor);
   speex_free(st->window);
   speex_free(st->noise);
   speex_free(st->reverb_estimate);
   speex_free(st->old_ps);
   speex_free(st->gain);
   speex_free(st->prior);
   speex_free(st->post);
   speex_free(st->loudness_weight);
   speex_free(st->echo_noise);
   speex_free(st->residual_echo);
   speex_free(st->S);
   speex_free(st->Smin);
   speex_free(st->Stmp);
   speex_free(st->update_prob);
   speex_free(st->zeta);
   speex_free(st->inbuf);
   speex_free(st->outbuf);
   spx_fft_destroy(st->fft_lookup);
   filterbank_destroy(st->bank);
   speex_free(st);
}

static void preprocess_analysis(SpeexPreprocessState *st, spx_int16_t *x)
{
   int i;
   int N = st->ps_size;
   int N3 = 2*N - st->frame_size;
   int N4 = st->frame_size - N3;
   spx_word32_t *ps=st->ps;

   /* Build input frame */
   for (i=0;i<N3;i++)
      st->frame[i]=st->inbuf[i];
   for (i=0;i<st->frame_size;i++)
      st->frame[N3+i]=x[i];
   
   /* Update inbuf */
   for (i=0;i<N3;i++)
      st->inbuf[i]=x[N4+i];

   /* Windowing */
   for (i=0;i<2*N;i++)
      st->frame[i] = MULT16_16_Q15(st->frame[i], st->window[i]);
   
   /* Perform FFT */
   spx_fft(st->fft_lookup, st->frame, st->ft);
         
   /* Power spectrum */
   ps[0]=MULT16_16(st->ft[0],st->ft[0]);
   for (i=1;i<N;i++)
      ps[i]=MULT16_16(st->ft[2*i-1],st->ft[2*i-1]) + MULT16_16(st->ft[2*i],st->ft[2*i]);

   filterbank_compute_bank32(st->bank, ps, ps+N);
}

static void update_noise_prob(SpeexPreprocessState *st)
{
   int i;
   int min_range;
   int N = st->ps_size;

   for (i=1;i<N-1;i++)
      st->S[i] =  MULT16_32_Q15(QCONST16(.8f,15),st->S[i]) + MULT16_32_Q15(QCONST16(.05f,15),st->ps[i-1]) 
                      + MULT16_32_Q15(QCONST16(.1f,15),st->ps[i]) + MULT16_32_Q15(QCONST16(.05f,15),st->ps[i+1]);
   st->S[0] =  MULT16_32_Q15(QCONST16(.8f,15),st->S[0]) + MULT16_32_Q15(QCONST16(.2f,15),st->ps[0]);
   st->S[N-1] =  MULT16_32_Q15(QCONST16(.8f,15),st->S[N-1]) + MULT16_32_Q15(QCONST16(.2f,15),st->ps[N-1]);
   
   if (st->nb_adapt==1)
   {
      for (i=0;i<N;i++)
         st->Smin[i] = st->Stmp[i] = 0;
   }

   if (st->nb_adapt < 100)
      min_range = 15;
   else if (st->nb_adapt < 1000)
      min_range = 50;
   else if (st->nb_adapt < 10000)
      min_range = 150;
   else
      min_range = 300;
   if (st->min_count > min_range)
   {
      st->min_count = 0;
      for (i=0;i<N;i++)
      {
         st->Smin[i] = MIN32(st->Stmp[i], st->S[i]);
         st->Stmp[i] = st->S[i];
      }
   } else {
      for (i=0;i<N;i++)
      {
         st->Smin[i] = MIN32(st->Smin[i], st->S[i]);
         st->Stmp[i] = MIN32(st->Stmp[i], st->S[i]);      
      }
   }
   for (i=0;i<N;i++)
   {
      if (MULT16_32_Q15(QCONST16(.4f,15),st->S[i]) > st->Smin[i])
         st->update_prob[i] = 1;
      else
         st->update_prob[i] = 0;
   }
}


int speex_preprocess_run(SpeexPreprocessState *st, spx_int16_t *x)
{
   int i;
   int M;
   int N = st->ps_size;
   int N3 = 2*N - st->frame_size;
   int N4 = st->frame_size - N3;
   spx_word32_t *ps=st->ps;
   spx_word32_t Zframe;
   spx_word16_t Pframe;
   spx_word16_t beta, beta_1;
   spx_word16_t effective_echo_suppress;
   
   st->nb_adapt++;
   if (st->nb_adapt>20000)
      st->nb_adapt = 20000;
   st->min_count++;
   
   beta = MAX16(QCONST16(.03,15),DIV32_16(Q15_ONE,st->nb_adapt));
   beta_1 = Q15_ONE-beta;
   M = st->nbands;
   
   /* Deal with residual echo if provided */
   if (st->echo_state)
   {
      speex_echo_get_residual(st->echo_state, st->residual_echo, N);
      /* If there are NaNs or ridiculous values, reset to zero */
      if (!(st->residual_echo[0] >=0 && st->residual_echo[0]<N*1e9f))
      {
         for (i=0;i<N;i++)
            st->residual_echo[i] = 0;
      }
      for (i=0;i<N;i++)
         st->echo_noise[i] = MAX32(MULT16_32_Q15(QCONST16(.6f,15),st->echo_noise[i]), st->residual_echo[i]);
      filterbank_compute_bank32(st->bank, st->echo_noise, st->echo_noise+N);
   } else {
      for (i=0;i<N+M;i++)
         st->echo_noise[i] = 0;
   }
   preprocess_analysis(st, x);

   update_noise_prob(st);
   
   /* Update the noise estimate for the frequencies where it can be */
   for (i=0;i<N;i++)
   {
      if (!st->update_prob[i] || st->ps[i] < PSHR32(st->noise[i], NOISE_SHIFT))
         st->noise[i] = MAX32(EXTEND32(0),MULT16_32_Q15(beta_1,st->noise[i]) + MULT16_32_Q15(beta,SHL32(st->ps[i],NOISE_SHIFT)));
   }
   filterbank_compute_bank32(st->bank, st->noise, st->noise+N);

   /* Special case for first frame */
   if (st->nb_adapt==1)
      for (i=0;i<N+M;i++)
         st->old_ps[i] = ps[i];

   /* Compute a posteriori SNR */
   for (i=0;i<N+M;i++)
   {
      spx_word16_t gamma;
      
      /* Total noise estimate including residual echo and reverberation */
      spx_word32_t tot_noise = ADD32(ADD32(ADD32(EXTEND32(1), PSHR32(st->noise[i],NOISE_SHIFT)) , st->echo_noise[i]) , st->reverb_estimate[i]);
      
      /* A posteriori SNR = ps/noise - 1*/
      st->post[i] = SUB16(DIV32_16_Q8(ps[i],tot_noise), QCONST16(1.f,SNR_SHIFT));
      st->post[i]=MIN16(st->post[i], QCONST16(100.f,SNR_SHIFT));
      
      /* Computing update gamma = .1 + .9*(old/(old+noise))^2 */
      gamma = QCONST16(.1f,15)+MULT16_16_Q15(QCONST16(.89f,15),SQR16_Q15(DIV32_16_Q15(st->old_ps[i],ADD32(st->old_ps[i],tot_noise))));
      
      /* A priori SNR update = gamma*max(0,post) + (1-gamma)*old/noise */
      st->prior[i] = EXTRACT16(PSHR32(ADD32(MULT16_16(gamma,MAX16(0,st->post[i])), MULT16_16(Q15_ONE-gamma,DIV32_16_Q8(st->old_ps[i],tot_noise))), 15));
      st->prior[i]=MIN16(st->prior[i], QCONST16(100.f,SNR_SHIFT));
   }

   /* Recursive average of the a priori SNR */
   st->zeta[0] = PSHR32(ADD32(MULT16_16(QCONST16(.7f,15),st->zeta[0]), MULT16_16(QCONST16(.3f,15),st->prior[0])),15);
   for (i=1;i<N-1;i++)
      st->zeta[i] = PSHR32(ADD32(ADD32(ADD32(MULT16_16(QCONST16(.7f,15),st->zeta[i]), MULT16_16(QCONST16(.15f,15),st->prior[i])),
                           MULT16_16(QCONST16(.075f,15),st->prior[i-1])), MULT16_16(QCONST16(.075f,15),st->prior[i+1])),15);
   for (i=N-1;i<N+M;i++)
      st->zeta[i] = PSHR32(ADD32(MULT16_16(QCONST16(.7f,15),st->zeta[i]), MULT16_16(QCONST16(.3f,15),st->prior[i])),15);

   /* Speech probability of presence for the entire frame */
   Zframe = 0;
   for (i=N;i<N+M;i++)
      Zframe = ADD32(Zframe, EXTEND32(st->zeta[i]));
   Pframe = QCONST16(.1f,15)+MULT16_16_Q15(QCONST16(.899f,15),qcurve(DIV32_16(Zframe,st->nbands)));
   
   effective_echo_suppress = EXTRACT16(PSHR32(ADD32(MULT16_16(SUB16(Q15_ONE,Pframe), st->echo_suppress), MULT16_16(Pframe, st->echo_suppress_active)),15));
   
   compute_gain_floor(st->noise_suppress, effective_echo_suppress, st->noise+N, st->echo_noise+N, st->gain_floor+N, M);
         
   /* Compute Ephraim & Malah gain speech probability of presence for each critical band */
   for (i=N;i<N+M;i++)
   {
      spx_word32_t theta;
      spx_word32_t MM;
      spx_word16_t prior_ratio;
      spx_word16_t P1;
      spx_word16_t q;
      
      prior_ratio = PDIV32_16(SHL32(EXTEND32(st->prior[i]), 15), ADD16(st->prior[i], SHL32(1,SNR_SHIFT)));
      theta = MULT16_32_P15(prior_ratio, QCONST32(1.f,EXPIN_SHIFT)+SHL32(EXTEND32(st->post[i]),EXPIN_SHIFT-SNR_SHIFT));

      MM = hypergeom_gain(theta);
      st->gain[i] = EXTRACT16(MIN32(Q15_ONE, MULT16_32_Q15(prior_ratio, MM)));
      st->old_ps[i] = MULT16_32_P15(QCONST16(.2f,15),st->old_ps[i]) + MULT16_32_P15(MULT16_16_P15(QCONST16(.8f,15),SQR16_Q15(st->gain[i])),ps[i]);

      P1 = QCONST16(.199f,15)+MULT16_16_Q15(QCONST16(.8f,15),qcurve (st->zeta[i]));
      q = Q15_ONE-MULT16_16_Q15(Pframe,P1);
      st->gain2[i]=1/(1.f + (q/(1.f-q))*(1+st->prior[i])*exp(-theta));
   }
   
   /* Convert the EM gains and speech prob to linear frequency */
   filterbank_compute_psd16(st->bank,st->gain2+N, st->gain2);
   filterbank_compute_psd16(st->bank,st->gain+N, st->gain);
   filterbank_compute_psd16(st->bank,st->gain_floor+N, st->gain_floor);

   /* Compute gain according to the Ephraim-Malah algorithm -- linear frequency */
   for (i=0;i<N;i++)
   {
      spx_word32_t MM;
      spx_word32_t theta;
      spx_word16_t prior_ratio;
      spx_word16_t tmp;
      spx_word16_t p;
      spx_word16_t g;
      
      prior_ratio = PDIV32_16(SHL32(EXTEND32(st->prior[i]), 15), ADD16(st->prior[i], SHL32(1,SNR_SHIFT)));
      theta = MULT16_32_P15(prior_ratio, QCONST32(1.f,EXPIN_SHIFT)+SHL32(EXTEND32(st->post[i]),EXPIN_SHIFT-SNR_SHIFT));

      MM = hypergeom_gain(theta);
      g = EXTRACT16(MIN32(Q15_ONE, MULT16_32_Q15(prior_ratio, MM)));
      p = st->gain2[i];
               
      if (MULT16_16_Q15(QCONST16(.333f,15),g) > st->gain[i])
         g = MULT16_16(3,st->gain[i]);
      st->gain[i] = g;
      
      st->old_ps[i] = MULT16_32_P15(QCONST16(.2f,15),st->old_ps[i]) + MULT16_32_P15(MULT16_16_P15(QCONST16(.8f,15),SQR16_Q15(st->gain[i])),ps[i]);
      
      if (st->gain[i] < st->gain_floor[i])
         st->gain[i] = st->gain_floor[i];

      tmp = MULT16_16_P15(p,spx_sqrt(SHL32(EXTEND32(st->gain[i]),15))) + MULT16_16_P15(SUB16(Q15_ONE,p),spx_sqrt(SHL32(EXTEND32(st->gain_floor[i]),15)));
      st->gain2[i]=SQR16_Q15(tmp);
   }
   
   /* If noise suppression is off, don't apply the gain */
   if (!st->denoise_enabled)
   {
      for (i=0;i<N+M;i++)
         st->gain2[i]=Q15_ONE;
   }
      
   /* Apply computed gain */
   for (i=1;i<N;i++)
   {
      st->ft[2*i-1] = MULT16_16_P15(st->gain2[i],st->ft[2*i-1]);
      st->ft[2*i] = MULT16_16_P15(st->gain2[i],st->ft[2*i]);
   }
   st->ft[0] = MULT16_16_P15(st->gain2[0],st->ft[0]);
   st->ft[2*N-1] = MULT16_16_P15(st->gain2[N-1],st->ft[2*N-1]);

   /* Inverse FFT with 1/N scaling */
   spx_ifft(st->fft_lookup, st->ft, st->frame);
   
   /* Synthesis window (for WOLA) */
   for (i=0;i<2*N;i++)
      st->frame[i] = MULT16_16_Q15(st->frame[i], st->window[i]);

   /* Perform overlap and add */
   for (i=0;i<N3;i++)
      x[i] = st->outbuf[i] + st->frame[i];
   for (i=0;i<N4;i++)
      x[N3+i] = st->frame[N3+i];
   
   /* Update outbuf */
   for (i=0;i<N3;i++)
      st->outbuf[i] = st->frame[st->frame_size+i];

   st->speech_prob = Pframe;
   if (st->vad_enabled)
   {
      if (st->speech_prob > st->speech_prob_start || (st->was_speech && st->speech_prob > st->speech_prob_continue))
      {
         st->was_speech=1;
         return 1;
      } else
      {
         st->was_speech=0;
         return 0;
      }
   } else {
      return 1;
   }
}

int speex_preprocess_ctl(SpeexPreprocessState *st, int request, void *ptr)
{
   switch(request)
   {
   case SPEEX_PREPROCESS_SET_DENOISE:
      st->denoise_enabled = (*(spx_int32_t*)ptr);
      break;
   case SPEEX_PREPROCESS_GET_DENOISE:
      (*(spx_int32_t*)ptr) = st->denoise_enabled;
      break;
   case SPEEX_PREPROCESS_SET_AGC:
      st->agc_enabled = (*(spx_int32_t*)ptr);
      break;
   case SPEEX_PREPROCESS_GET_AGC:
      (*(spx_int32_t*)ptr) = st->agc_enabled;
      break;
   case SPEEX_PREPROCESS_SET_AGC_LEVEL:
      st->agc_level = (*(float*)ptr);
      if (st->agc_level<1)
         st->agc_level=1;
      if (st->agc_level>32768)
         st->agc_level=32768;
      break;
   case SPEEX_PREPROCESS_GET_AGC_LEVEL:
      (*(float*)ptr) = st->agc_level;
      break;
   case SPEEX_PREPROCESS_SET_VAD:
      st->vad_enabled = (*(spx_int32_t*)ptr);
      break;
   case SPEEX_PREPROCESS_GET_VAD:
      (*(spx_int32_t*)ptr) = st->vad_enabled;
      break;
   case SPEEX_PREPROCESS_SET_DEREVERB:
      st->dereverb_enabled = (*(spx_int32_t*)ptr);
      break;
   case SPEEX_PREPROCESS_GET_DEREVERB:
      (*(spx_int32_t*)ptr) = st->dereverb_enabled;
      break;
   case SPEEX_PREPROCESS_SET_NOISE_SUPPRESS:
      st->noise_suppress = -abs(*(spx_int32_t*)ptr);
      break;
   case SPEEX_PREPROCESS_GET_NOISE_SUPPRESS:
      (*(spx_int32_t*)ptr) = st->noise_suppress;
      break;
   case SPEEX_PREPROCESS_SET_ECHO_SUPPRESS:
      st->echo_suppress = -abs(*(spx_int32_t*)ptr);
      break;
   case SPEEX_PREPROCESS_GET_ECHO_SUPPRESS:
      (*(spx_int32_t*)ptr) = st->echo_suppress;
      break;
   case SPEEX_PREPROCESS_SET_ECHO_SUPPRESS_ACTIVE:
      st->echo_suppress_active = -abs(*(spx_int32_t*)ptr);
      break;
   case SPEEX_PREPROCESS_GET_ECHO_SUPPRESS_ACTIVE:
      (*(spx_int32_t*)ptr) = st->echo_suppress_active;
      break;
   case SPEEX_PREPROCESS_SET_ECHO_STATE:
      st->echo_state = (SpeexEchoState*)ptr;
      break;
   case SPEEX_PREPROCESS_GET_ECHO_STATE:
      (*(SpeexEchoState**)ptr) = (SpeexEchoState*)st->echo_state;
      break;
   default:
      return -1;
   }
   return 0;
}
