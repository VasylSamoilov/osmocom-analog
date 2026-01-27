/* Copyright (C) Jean-Marc Valin */
/**
   @file speex_echo.h
   @brief Echo cancellation based on SpeexDSP MDF algorithm
*/
/*
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

#ifndef SPEEX_ECHO_H
#define SPEEX_ECHO_H

#include "speexdsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Obtain frame size used by the AEC */
#define SPEEX_ECHO_GET_FRAME_SIZE 3

/** Set sampling rate */
#define SPEEX_ECHO_SET_SAMPLING_RATE 24

/** Get sampling rate */
#define SPEEX_ECHO_GET_SAMPLING_RATE 25

/** Get size of impulse response (int32) */
#define SPEEX_ECHO_GET_IMPULSE_RESPONSE_SIZE 27

/** Get impulse response (int32[]) */
#define SPEEX_ECHO_GET_IMPULSE_RESPONSE 29

/** Get adapted state (int, 1=adapted, 0=not adapted) */
#define SPEEX_ECHO_GET_ADAPTED 30

/** Get leak estimate (float, 0-1, lower=better) */
#define SPEEX_ECHO_GET_LEAK_ESTIMATE 31

/** Set adaptation rate multiplier (float, default=1.0, higher=faster adaptation) */
#define SPEEX_ECHO_SET_ADAPT_RATE 32

/** Get adaptation rate multiplier */
#define SPEEX_ECHO_GET_ADAPT_RATE 33

/** Internal echo canceller state */
struct SpeexEchoState_;

/** Echo canceller state handle */
typedef struct SpeexEchoState_ SpeexEchoState;

/**
 * Echo canceller statistics for monitoring performance
 */
typedef struct speex_echo_stats {
    int adapted;            /* 1 if filter is adapted, 0 otherwise */
    int saturated;          /* Saturation counter (>0 means saturation occurred) */
    int screwed_up;         /* Divergence counter (high = filter is misbehaving) */
    float leak_estimate;    /* Estimated residual echo (0-1, lower is better) */
    float sum_adapt;        /* Total adaptation progress */
    
    /* Power levels (RMS-like, for diagnostics) */
    float Sxx;              /* Far-end (TX/play) signal power */
    float Syy;              /* Estimated echo power */
    float See;              /* Residual error power */
    float Sdd;              /* Near-end (RX/mic) input power */
    float Sff;              /* Foreground filter error power */
    
    /* Calculated metrics */
    float erle_db;          /* Echo Return Loss Enhancement in dB */
    float erle_inst_db;     /* Instantaneous ERLE in dB */
    
    /* Filter analysis */
    int peak_block;         /* Block index with highest filter energy (0=most recent) */
    float peak_block_ms;    /* Estimated echo delay in ms based on peak block */
} speex_echo_stats_t;

/**
 * speex_echo_get_stats - Get echo canceller statistics
 * @st: Echo canceller state
 * @stats: Pointer to stats structure to fill
 *
 * Fills the stats structure with current performance metrics.
 */
void speex_echo_get_stats(SpeexEchoState *st, speex_echo_stats_t *stats);

/**
 * speex_echo_state_init - Create echo canceller state
 * @frame_size: Number of samples to process at one time (10-20 ms recommended)
 * @filter_length: Number of samples of echo to cancel (100-500 ms)
 * @return: New echo canceller state
 *
 * The MDF algorithm uses block-based processing. The frame_size determines
 * the processing granularity and latency. Typical values:
 * - frame_size = 64 (8ms @ 8kHz)
 * - frame_size = 128 (16ms @ 8kHz)
 * - filter_length = 800-1200 (100-150ms @ 8kHz) for typical echo paths
 */
SpeexEchoState *speex_echo_state_init(int frame_size, int filter_length);

/**
 * speex_echo_state_init_mc - Create multi-channel echo canceller state
 * @frame_size: Number of samples per frame per channel
 * @filter_length: Filter length in samples
 * @nb_mic: Number of microphone channels
 * @nb_speakers: Number of speaker channels
 * @return: New echo canceller state
 */
SpeexEchoState *speex_echo_state_init_mc(int frame_size, int filter_length, int nb_mic, int nb_speakers);

/**
 * speex_echo_state_destroy - Destroy echo canceller state
 * @st: Echo canceller state to destroy
 */
void speex_echo_state_destroy(SpeexEchoState *st);

/**
 * speex_echo_cancellation - Perform echo cancellation on a frame
 * @st: Echo canceller state
 * @rec: Recorded signal (microphone input with echo)
 * @play: Played signal (speaker output, far-end reference)
 * @out: Output signal with echo removed
 *
 * This is the main processing function. Call once per frame.
 * All buffers must be frame_size samples.
 */
void speex_echo_cancellation(SpeexEchoState *st, const spx_int16_t *rec, const spx_int16_t *play, spx_int16_t *out);

/**
 * speex_echo_cancel - Deprecated alias for speex_echo_cancellation
 */
void speex_echo_cancel(SpeexEchoState *st, const spx_int16_t *rec, const spx_int16_t *play, spx_int16_t *out, spx_int32_t *Yout);

/**
 * speex_echo_capture - Process captured audio using internal playback buffer
 * @st: Echo canceller state
 * @rec: Recorded signal
 * @out: Output signal with echo removed
 *
 * Use this with speex_echo_playback() for async capture/playback.
 */
void speex_echo_capture(SpeexEchoState *st, const spx_int16_t *rec, spx_int16_t *out);

/**
 * speex_echo_playback - Queue playback audio to internal buffer
 * @st: Echo canceller state
 * @play: Playback signal
 *
 * Use this with speex_echo_capture() for async capture/playback.
 */
void speex_echo_playback(SpeexEchoState *st, const spx_int16_t *play);

/**
 * speex_echo_state_reset - Reset echo canceller to initial state
 * @st: Echo canceller state
 *
 * Call at start of new call or when echo path changes significantly.
 */
void speex_echo_state_reset(SpeexEchoState *st);

/**
 * speex_echo_ctl - Control echo canceller parameters
 * @st: Echo canceller state
 * @request: Request type (SPEEX_ECHO_* constant)
 * @ptr: Pointer to data for request
 * @return: 0 on success, -1 on error
 */
int speex_echo_ctl(SpeexEchoState *st, int request, void *ptr);

#ifdef __cplusplus
}
#endif

#endif /* SPEEX_ECHO_H */

