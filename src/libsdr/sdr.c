/* SDR processing
 *
 * (C) 2017 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

enum paging_signal;

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <limits.h>
#define __USE_GNU
#include <pthread.h>
#include <unistd.h>
#include "../libsample/sample.h"
#include "../libfm/fm.h"
#include "../libam/am.h"
#include <osmocom/core/timer.h>
#include "../libmobile/sender.h"
#include "sdr_config.h"
#include "sdr.h"
#include "sdr_status.h"
#ifdef HAVE_UHD
#include "uhd.h"
#endif
#ifdef HAVE_SOAPY
#include "soapy.h"
#include <SoapySDR/Constants.h>
#endif
#ifdef HAVE_RPITX
#include "rpitx.h"
#endif
#include "../liblogging/logging.h"
#include "../libchannelizer/channelizer.h"
#include "../libchannelizer/channelizer_tx.h"
#include "../libpolyphase/polyphase.h"

/* enable to debug buffer handling */
//#define DEBUG_BUFFER

/* enable to test without oversampling filter */
//#define DISABLE_FILTER

/* usable bandwidth of IQ rate, because no filter is perfect */
#define USABLE_BANDWIDTH	0.75

/* limit the IQ level to prevent IIR filter from exceeding range of -1 .. 1 */
#define LIMIT_IQ_LEVEL		0.95

int sdr_rx_overflow = 0;

static int tx_driver;  /* 0=none, 1=uhd, 2=soapy, 3=rpitx */
static int rx_driver;  /* 0=none, 1=uhd, 2=soapy */

/* Forward declaration for timing function used by threads */
static double get_time(void);

typedef struct sdr_thread {
	int use;
	volatile int running, exit;	/* flags to control exit of threads */
	int buffer_size;
	volatile float *buffer;
	float *buffer2;
	volatile int in, out;		/* in and out pointers (atomic, so no locking required) */
	int max_fill;			/* measure maximum buffer fill */
	double max_fill_timer;		/* timer to display/reset maximum fill */
	iir_filter_t lp[2];		/* filter for upsample/downsample IQ data */
} sdr_thread_t;

typedef struct sdr_chan {
	double		tx_frequency;	/* frequency used */
	double		rx_frequency;	/* frequency used */
	int		am;		/* use AM instead of FM */
	fm_mod_t	fm_mod;		/* modulator instance */
	fm_demod_t	fm_demod;	/* demodulator instance */
	am_mod_t	am_mod;		/* modulator instance */
	am_demod_t	am_demod;	/* demodulator instance */
	dispmeasparam_t	*dmp_rf_level;
	dispmeasparam_t	*dmp_freq_offset;
	dispmeasparam_t	*dmp_deviation;
	int		use_channelizer;
	channelizer_t	channelizer;
	sample_t	*ch_I, *ch_Q;	/* channelizer output buffers (double/sample_t) */
	float		*ch_iq_float;	/* interleaved float buffer for FM demod */
	/* TX channelizer (synthesis) */
	int		use_tx_channelizer;
	channelizer_tx_t tx_channelizer;
} sdr_chan_t;

typedef struct sdr {
	int		threads;	/* use threads */
	int		oversample;	/* oversample IQ rate (unified mode) */
	int		tx_oversample;	/* TX oversample (split mode) - integer part */
	int		rx_oversample;	/* RX oversample (split mode) - integer part */
	/* Polyphase resampler for non-integer rate conversion */
	int		tx_use_polyphase;	/* 1 if TX needs polyphase resampling */
	int		rx_use_polyphase;	/* 1 if RX needs polyphase resampling */
	polyphase_t	tx_polyphase;		/* TX polyphase resampler state */
	polyphase_t	rx_polyphase;		/* RX polyphase resampler state */
	sample_t	*tx_poly_i;		/* TX polyphase I buffer */
	sample_t	*tx_poly_q;		/* TX polyphase Q buffer */
	sample_t	*rx_poly_i;		/* RX polyphase I buffer */
	sample_t	*rx_poly_q;		/* RX polyphase Q buffer */
	float		*rx_poly_sdr_buff;	/* RX polyphase SDR interleaved buffer (non-threaded) */
	int		rx_poly_max_in;		/* Max input samples for RX polyphase */
	sdr_thread_t	thread_read,
			thread_write;
	sdr_chan_t	*chan;		/* settings for all channels */
	int		paging_channel;	/* if set, points to paging channel */
	sdr_chan_t	paging_chan;	/* settings for extra paging channel */
	int		channels;	/* number of frequencies */
	double		amplitude;	/* amplitude of each carrier */
	int		samplerate;	/* sample rate of audio data */
	/* TX channelizer buffers */
	sample_t	*tx_chan_I;	/* de-interleaved I buffer for TX channelizer */
	sample_t	*tx_chan_Q;	/* de-interleaved Q buffer for TX channelizer */
	int		buffer_size;	/* buffer in audio samples */
	double		interval;	/* how often to process the loop */
	wave_rec_t	wave_rx_rec;
	wave_rec_t	wave_tx_rec;
	wave_play_t	wave_rx_play;
	wave_play_t	wave_tx_play;
	float		*modbuff;	/* buffer for transmodulation */
	sample_t	*modbuff_I;
	sample_t	*modbuff_Q;
	sample_t	*modbuff_carrier;
	sample_t	*wavespl0;	/* sample buffer for wave generation */
	sample_t	*wavespl1;
	sample_t	*chan_in_buff;  /* buffer for channelizer input (decimated block read) */
	/* Radio mode channelizer (for channels=0 with use_channelizer) */
	int		radio_channelizer_init;	/* 1 if radio channelizer is initialized */
	channelizer_t	radio_channelizer;	/* channelizer for radio mode */
	sample_t	*radio_ch_I;		/* radio channelizer I output buffer */
	sample_t	*radio_ch_Q;		/* radio channelizer Q output buffer */
} sdr_t;

static void show_spectrum(const char *direction, double halfbandwidth, double center, double *frequency, double paging_frequency, int num)
{
	char text[80];
	int i, x;

	memset(text, ' ', 79);
	text[79] = '\0';

	// FIXME: better solution
	if (num > 9)
		num = 9;

	for (i = 0; i < num; i++) {
		x = (frequency[i] - center) / halfbandwidth * 39.0 + 39.5;
		if (x >= 0 && x < 79)
			text[x] = '1' + i;
	}
	if (paging_frequency) {
		x = (paging_frequency - center) / halfbandwidth * 39.0 + 39.5;
		if (x >= 0 && x < 79)
			text[x] = 'P';
	}

	LOGP(DSDR, LOGL_INFO, "%s Spectrum:\n%s\n---------------------------------------+---------------------------------------\n", direction, text);
	for (i = 0; i < num; i++)
		LOGP(DSDR, LOGL_INFO, "Frequency %c = %.4f MHz\n", '1' + i, frequency[i] / 1e6);
	if (paging_frequency)
		LOGP(DSDR, LOGL_INFO, "Frequency P = %.4f MHz (Paging Frequency)\n", paging_frequency / 1e6);
}

static void *sdr_open_internal(int direction, const char *device, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index, int use_channelizer, int fast_math)
{
	sdr_t *sdr;
	int threads = 1, oversample = 1; /* always use threads */
	double bandwidth;
	double tx_center_frequency = 0.0, rx_center_frequency = 0.0;
	int rc;
	int c;

	(void)direction;
	(void)device;

	LOGP(DSDR, LOGL_DEBUG, "Open SDR device\n");

	/* Auto-calculate optimal rate if requested */
	if (samplerate == 0 && use_channelizer) {
		double required_bw = 2.0 * (max_deviation + max_modulation);
		samplerate = sdr_calculate_optimal_rate(sdr_config->samplerate, required_bw);
		LOGP(DSDR, LOGL_INFO, "Auto-calculated optimal channel rate: %d Hz (from Master %d Hz)\n", 
		     samplerate, sdr_config->samplerate);
	}

	if (sdr_config->samplerate != samplerate) {
		if (samplerate > sdr_config->samplerate) {
			LOGP(DSDR, LOGL_ERROR, "SDR sample rate must be greater than audio sample rate! (SDR %d < Audio %d)\n", sdr_config->samplerate, samplerate);
			return NULL;
		}
		if (sdr_config->samplerate % samplerate) {
			/* Non-integer ratio - will use polyphase resampling (handled in split mode logic below) */
			LOGP(DSDR, LOGL_NOTICE, "SDR sample rate %d is not a multiple of audio sample rate %d - will use polyphase resampling\n", 
			     sdr_config->samplerate, samplerate);
			oversample = 1;  /* Polyphase handles the resampling */
		} else {
			oversample = sdr_config->samplerate / samplerate;
		}
		threads = 1;
	}

	bandwidth = 2.0 * (max_deviation + max_modulation);
	if (bandwidth)
		LOGP(DSDR, LOGL_INFO, "Require bandwidth of each channel is 2 * (%.1f deviation + %.1f modulation) = %.1f KHz\n", max_deviation / 1e3, max_modulation / 1e3, bandwidth / 1e3);

	if (use_channelizer && oversample > 1) {
		LOGP(DSDR, LOGL_INFO, "Using Channelizer with %dx decimation.\n", oversample);
	} else if (use_channelizer) {
		LOGP(DSDR, LOGL_NOTICE, "Channelizer requested but oversample is 1. Disabling channelizer.\n");
		use_channelizer = 0;
	}

	sdr = calloc(1, sizeof(*sdr));
	if (!sdr) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}
	sdr->channels = channels;
	sdr->amplitude = 1.0 / (double)channels;
	sdr->samplerate = samplerate;
	sdr->buffer_size = buffer_size;
	sdr->interval = interval;
	sdr->threads = threads; /* always required, because write may block */
	sdr->oversample = oversample;
	
	/* In split mode, calculate separate oversample factors for TX and RX */
	if (sdr_config->split_mode) {
		if (sdr_config->tx_samplerate && samplerate) {
			if (sdr_config->tx_samplerate % samplerate) {
				/* Non-integer ratio - use polyphase resampler */
				double ratio = (double)sdr_config->tx_samplerate / (double)samplerate;
				LOGP(DSDR, LOGL_NOTICE, "TX sample rate %d is not an integer multiple of DSP rate %d (ratio=%.6f)\n",
				     sdr_config->tx_samplerate, samplerate, ratio);
				LOGP(DSDR, LOGL_NOTICE, "Enabling polyphase resampler for TX path (DSP %d Hz -> SDR %d Hz)\n",
				     samplerate, sdr_config->tx_samplerate);
				sdr->tx_use_polyphase = 1;
				sdr->tx_oversample = 1; /* Not used when polyphase is active */
				
				/* Initialize TX polyphase resampler (upsample: DSP -> SDR).
				 * The rational polyphase resampler finds exact P/Q ratio via GCD.
				 * cutoff and phase_steps params are ignored (auto-computed). */
				int rc = polyphase_init_taps(&sdr->tx_polyphase,
				                             (double)samplerate,
				                             (double)sdr_config->tx_samplerate,
				                             0, 0, 16);
				if (rc < 0) {
					LOGP(DSDR, LOGL_ERROR, "Failed to initialize TX polyphase resampler!\n");
					goto error;
				}
				LOGP(DSDR, LOGL_INFO, "TX rational polyphase: %d Hz -> %d Hz (P=%d, Q=%d, branch_len=%d, total_taps=%d)\n",
				     samplerate, sdr_config->tx_samplerate,
				     sdr->tx_polyphase.up, sdr->tx_polyphase.down,
				     sdr->tx_polyphase.branch_len, sdr->tx_polyphase.total_taps);
				
				/* Allocate polyphase I/Q buffers - need space for both input and output
				 * Input: buffer_size samples, Output: max_out samples
				 * We use the same buffer with offset for output */
				int max_out = polyphase_output_num(&sdr->tx_polyphase, buffer_size) + 16;
				int total_size = buffer_size + max_out;  /* Input + output in same buffer */
				sdr->tx_poly_i = calloc(total_size, sizeof(sample_t));
				sdr->tx_poly_q = calloc(total_size, sizeof(sample_t));
				if (!sdr->tx_poly_i || !sdr->tx_poly_q) {
					LOGP(DSDR, LOGL_ERROR, "NO MEM for TX polyphase buffers!\n");
					goto error;
				}
				LOGP(DSDR, LOGL_INFO, "TX polyphase buffers: input=%d output=%d total=%d samples each\n",
				     buffer_size, max_out, total_size);
			} else {
				sdr->tx_oversample = sdr_config->tx_samplerate / samplerate;
				LOGP(DSDR, LOGL_INFO, "TX using integer resampling: %d Hz -> %d Hz (oversample=%dx)\n",
				     samplerate, sdr_config->tx_samplerate, sdr->tx_oversample);
			}
		} else {
			sdr->tx_oversample = oversample;
		}
		if (sdr_config->rx_samplerate && samplerate) {
			if (sdr_config->rx_samplerate % samplerate) {
				/* Non-integer ratio - use polyphase resampler */
				double ratio = (double)sdr_config->rx_samplerate / (double)samplerate;
				LOGP(DSDR, LOGL_NOTICE, "RX sample rate %d is not an integer multiple of DSP rate %d (ratio=%.6f)\n",
				     sdr_config->rx_samplerate, samplerate, ratio);
				LOGP(DSDR, LOGL_NOTICE, "Enabling polyphase resampler for RX path (SDR %d Hz -> DSP %d Hz)\n",
				     sdr_config->rx_samplerate, samplerate);
				sdr->rx_use_polyphase = 1;
				sdr->rx_oversample = 1; /* Not used when polyphase is active */
				
				/* Initialize RX polyphase resampler (downsample: SDR -> DSP).
				 * The rational polyphase resampler finds exact P/Q ratio via GCD. */
				int rc = polyphase_init_taps(&sdr->rx_polyphase,
				                             (double)sdr_config->rx_samplerate,
				                             (double)samplerate,
				                             0, 0, 16);
				if (rc < 0) {
					LOGP(DSDR, LOGL_ERROR, "Failed to initialize RX polyphase resampler!\n");
					goto error;
				}
				LOGP(DSDR, LOGL_INFO, "RX rational polyphase: %d Hz -> %d Hz (P=%d, Q=%d, branch_len=%d, total_taps=%d)\n",
				     sdr_config->rx_samplerate, samplerate,
				     sdr->rx_polyphase.up, sdr->rx_polyphase.down,
				     sdr->rx_polyphase.branch_len, sdr->rx_polyphase.total_taps);
				
				/* Allocate polyphase I/Q buffers - need space for both input and output
				 * Input: max_in samples (SDR rate), Output: buffer_size samples (DSP rate) */
				int max_in = polyphase_input_num(&sdr->rx_polyphase, buffer_size) + 16;
				int total_size = max_in + buffer_size;  /* Input + output in same buffer */
				sdr->rx_poly_i = calloc(total_size, sizeof(sample_t));
				sdr->rx_poly_q = calloc(total_size, sizeof(sample_t));
				sdr->rx_poly_sdr_buff = calloc(max_in * 2, sizeof(float));  /* Interleaved I/Q */
				sdr->rx_poly_max_in = max_in;
				if (!sdr->rx_poly_i || !sdr->rx_poly_q || !sdr->rx_poly_sdr_buff) {
					LOGP(DSDR, LOGL_ERROR, "NO MEM for RX polyphase buffers!\n");
					goto error;
				}
				LOGP(DSDR, LOGL_INFO, "RX polyphase buffers: input=%d output=%d total=%d samples each\n",
				     max_in, buffer_size, total_size);
			} else {
				sdr->rx_oversample = sdr_config->rx_samplerate / samplerate;
				LOGP(DSDR, LOGL_INFO, "RX using integer resampling: %d Hz -> %d Hz (oversample=%dx)\n",
				     sdr_config->rx_samplerate, samplerate, sdr->rx_oversample);
			}
		} else {
			sdr->rx_oversample = oversample;
		}
		LOGP(DSDR, LOGL_NOTICE, "Split mode rates: DSP=%d TX_SDR=%d RX_SDR=%d TX_%s=%s RX_%s=%s\n",
		     samplerate, sdr_config->tx_samplerate, sdr_config->rx_samplerate,
		     sdr->tx_use_polyphase ? "polyphase" : "os",
		     sdr->tx_use_polyphase ? "yes" : (sdr->tx_oversample > 1 ? "integer" : "1"),
		     sdr->rx_use_polyphase ? "polyphase" : "os",
		     sdr->rx_use_polyphase ? "yes" : (sdr->rx_oversample > 1 ? "integer" : "1"));
	} else {
		sdr->tx_oversample = oversample;
		sdr->rx_oversample = oversample;
	}

	if (threads) {
		/* RX buffer allocation - only if not TX-only mode */
		if (!sdr_config->tx_only) {
			LOGP(DSDR, LOGL_NOTICE, "Allocating RX thread buffers\n");
			memset(&sdr->thread_read, 0, sizeof(sdr->thread_read));
			/* Calculate RX input buffer size:
			 * - For integer resampling: buffer_size * rx_oversample
			 * - For polyphase: estimate based on ratio + margin */
			int rx_in_size;
			if (sdr->rx_use_polyphase) {
				/* Polyphase input size: need enough for one processing block at SDR rate */
				rx_in_size = (int)((double)sdr->buffer_size * 
				             (double)sdr_config->rx_samplerate / (double)samplerate) + 32;
			} else {
				rx_in_size = sdr->buffer_size * sdr->rx_oversample;
			}
			sdr->thread_read.buffer_size = rx_in_size * 2 + 2;
			sdr->thread_read.buffer = calloc(sdr->thread_read.buffer_size, sizeof(*sdr->thread_read.buffer));
			if (!sdr->thread_read.buffer) {
				LOGP(DSDR, LOGL_ERROR, "No mem!\n");
				goto error;
			}
			sdr->thread_read.buffer2 = calloc(sdr->thread_read.buffer_size, sizeof(*sdr->thread_read.buffer2));
			if (!sdr->thread_read.buffer2) {
				LOGP(DSDR, LOGL_ERROR, "No mem!\n");
				goto error;
			}
			sdr->thread_read.in = sdr->thread_read.out = 0;
			/* Use RX sample rate for read thread filter in split mode (only for integer resampling) */
			if (sdr->rx_oversample > 1 && !sdr->rx_use_polyphase) {
				int rx_rate = sdr_config->split_mode ? sdr_config->rx_samplerate : sdr_config->samplerate;
				iir_lowpass_init(&sdr->thread_read.lp[0], samplerate / 2.0, rx_rate, 2);
				iir_lowpass_init(&sdr->thread_read.lp[1], samplerate / 2.0, rx_rate, 2);
			}
		} else {
			LOGP(DSDR, LOGL_INFO, "Skipping RX buffer allocation (TX-only mode)\n");
		}
		/* TX buffer allocation - only if not RX-only mode */
		if (!sdr_config->rx_only) {
			LOGP(DSDR, LOGL_NOTICE, "Allocating TX thread buffers\n");
			memset(&sdr->thread_write, 0, sizeof(sdr->thread_write));
			sdr->thread_write.buffer_size = sdr->buffer_size * 2 + 2;
			sdr->thread_write.buffer = calloc(sdr->thread_write.buffer_size, sizeof(*sdr->thread_write.buffer));
			if (!sdr->thread_write.buffer) {
				LOGP(DSDR, LOGL_ERROR, "No mem!\n");
				goto error;
			}
			/* Calculate TX output buffer size:
			 * - For integer resampling: buffer_size * tx_oversample
			 * - For polyphase: estimate based on ratio + margin */
			int tx_out_size;
			if (sdr->tx_use_polyphase) {
				/* Polyphase output size estimate with margin */
				tx_out_size = (int)((double)sdr->buffer_size * 
				              (double)sdr_config->tx_samplerate / (double)samplerate) + 32;
			} else {
				tx_out_size = sdr->buffer_size * sdr->tx_oversample;
			}
			sdr->thread_write.buffer2 = calloc(tx_out_size * 2, sizeof(*sdr->thread_write.buffer2));
			if (!sdr->thread_write.buffer2) {
				LOGP(DSDR, LOGL_ERROR, "No mem!\n");
				goto error;
			}
			sdr->thread_write.in = sdr->thread_write.out = 0;
			/* Use TX sample rate for write thread filter in split mode (only for integer resampling) */
			if (sdr->tx_oversample > 1 && !sdr->tx_use_polyphase) {
				int tx_rate = sdr_config->split_mode ? sdr_config->tx_samplerate : sdr_config->samplerate;
				iir_lowpass_init(&sdr->thread_write.lp[0], samplerate / 2.0, tx_rate, 2);
				iir_lowpass_init(&sdr->thread_write.lp[1], samplerate / 2.0, tx_rate, 2);
			}
		} else {
			LOGP(DSDR, LOGL_INFO, "Skipping TX buffer allocation (RX-only mode)\n");
		}
	}

	/* alloc fm modulation buffers */
	sdr->modbuff = calloc(sdr->buffer_size * 2, sizeof(*sdr->modbuff));
	if (!sdr->modbuff) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}
	sdr->modbuff_I = calloc(sdr->buffer_size, sizeof(*sdr->modbuff_I));
	if (!sdr->modbuff_I) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}
	sdr->modbuff_Q = calloc(sdr->buffer_size, sizeof(*sdr->modbuff_Q));
	if (!sdr->modbuff_Q) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}
	sdr->modbuff_carrier = calloc(sdr->buffer_size, sizeof(*sdr->modbuff_carrier));
	if (!sdr->modbuff_carrier) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}
	sdr->wavespl0 = calloc(sdr->buffer_size, sizeof(*sdr->wavespl0));
	if (!sdr->wavespl0) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}
	sdr->wavespl1 = calloc(sdr->buffer_size, sizeof(*sdr->wavespl1));
	if (!sdr->wavespl1) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}

	/* special case where we use a paging frequency */
	if (paging_frequency) {
		/* add extra paging channel */
		sdr->paging_channel = channels;
	}

	/* create list of channel states */
	if (channels) {
		sdr->chan = calloc(channels + (sdr->paging_channel != 0), sizeof(*sdr->chan));
		if (!sdr->chan) {
			LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
			goto error;
		}
	}

	if (use_channelizer) {
		/* Use rx_oversample for RX channelizer buffer in split mode */
		int rx_os_alloc = (sdr_config && sdr_config->split_mode) ? sdr->rx_oversample : sdr->oversample;
		sdr->chan_in_buff = calloc(sdr->buffer_size * rx_os_alloc * 2, sizeof(sample_t));
		if (!sdr->chan_in_buff) {
			LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
			goto error;
		}
		/* Allocate TX channelizer buffers */
		sdr->tx_chan_I = calloc(sdr->buffer_size, sizeof(sample_t));
		sdr->tx_chan_Q = calloc(sdr->buffer_size, sizeof(sample_t));
		if (!sdr->tx_chan_I || !sdr->tx_chan_Q) {
			LOGP(DSDR, LOGL_ERROR, "NO MEM for TX channelizer buffers!\n");
			goto error;
		}
		
		/* Initialize radio channelizer for channels=0 mode (decimation only) */
		if (!channels && rx_os_alloc > 1) {
			int rc = channelizer_init(&sdr->radio_channelizer, 
			                          sdr_config->samplerate, samplerate, 0, fast_math);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "Failed to init radio channelizer\n");
				goto error;
			}
			sdr->radio_ch_I = calloc(sdr->buffer_size, sizeof(sample_t));
			sdr->radio_ch_Q = calloc(sdr->buffer_size, sizeof(sample_t));
			if (!sdr->radio_ch_I || !sdr->radio_ch_Q) {
				LOGP(DSDR, LOGL_ERROR, "NO MEM for radio channelizer buffers!\n");
				goto error;
			}
			sdr->radio_channelizer_init = 1;
			LOGP(DSDR, LOGL_INFO, "Radio channelizer: %d Hz -> %d Hz (decimation %dx)\n",
			     sdr_config->samplerate, samplerate, sdr->radio_channelizer.decimation);
		}
	}

	/* swap links, if required */
	if (sdr_config->swap_links) {
		double *temp;
		LOGP(DSDR, LOGL_NOTICE, "Sapping RX and TX frequencies!\n");
		temp = rx_frequency;
		rx_frequency = tx_frequency;
		tx_frequency = temp;
	}

	/* Suppress unused direction for TX-only / RX-only modes */
	if (sdr_config->tx_only) {
		LOGP(DSDR, LOGL_NOTICE, "TX-only mode: suppressing RX frequency\n");
		rx_frequency = NULL;
	}
	if (sdr_config->rx_only) {
		LOGP(DSDR, LOGL_NOTICE, "RX-only mode: suppressing TX frequency\n");
		tx_frequency = NULL;
	}

	if (tx_frequency && !channels)
		tx_center_frequency = tx_frequency[0];
	
	/* TX IQ wave recording - works for all TX modes, including channels=0 (osmoradio) */
	if (tx_frequency && sdr_config->write_iq_tx_wave) {
		rc = wave_create_record(&sdr->wave_tx_rec, sdr_config->write_iq_tx_wave, samplerate, 2, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create TX WAVE recording instance!\n");
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Recording TX IQ to: %s\n", sdr_config->write_iq_tx_wave);
	}
	if (tx_frequency && sdr_config->read_iq_tx_wave) {
		int two = 2;
		rc = wave_create_playback(&sdr->wave_tx_play, sdr_config->read_iq_tx_wave, &samplerate, &two, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create TX WAVE playback instance!\n");
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Replacing TX IQ from: %s\n", sdr_config->read_iq_tx_wave);
	}
	
	if (tx_frequency && channels) {
		/* calculate required bandwidth (IQ rate) */

		double tx_low_frequency = 0.0, tx_high_frequency = 0.0;
		for (c = 0; c < channels; c++) {
			sdr->chan[c].tx_frequency = tx_frequency[c];
			if (c == 0 || sdr->chan[c].tx_frequency < tx_low_frequency)
				tx_low_frequency = sdr->chan[c].tx_frequency;
			if (c == 0 || sdr->chan[c].tx_frequency > tx_high_frequency)
				tx_high_frequency = sdr->chan[c].tx_frequency;
		}
		if (sdr->paging_channel) {
			sdr->chan[sdr->paging_channel].tx_frequency = paging_frequency;
			if (sdr->chan[sdr->paging_channel].tx_frequency < tx_low_frequency)
				tx_low_frequency = sdr->chan[sdr->paging_channel].tx_frequency;
			if (sdr->chan[sdr->paging_channel].tx_frequency > tx_high_frequency)
				tx_high_frequency = sdr->chan[sdr->paging_channel].tx_frequency;
		}
		tx_center_frequency = (tx_high_frequency + tx_low_frequency) / 2.0;

		/* prevent channel bandwidth from overlapping with the center frequency */
		if (channels == 1 && !sdr->paging_channel) {
			/* simple: just move off the center by two times half of the bandwidth */
			tx_center_frequency -= 2.0 * bandwidth / 2.0;
			/* Note: tx_low_frequency is kept at old center.
			   Calculation of 'low_side' will become 0.
			   This is correct, since there is no bandwidth
			   below new center frequency.
			 */
			LOGP(DSDR, LOGL_INFO, "We shift center frequency %.0f KHz down (half bandwidth), to prevent channel from overlapping with DC level.\n", bandwidth / 2.0 / 1e3);
		} else {
			/* find two channels that are aside the center */
			double low_dist = 0, high_dist = 0, dist;
			int low_c = -1, high_c = -1;
			for (c = 0; c < channels; c++) {
				dist = fabs(tx_center_frequency - sdr->chan[c].tx_frequency);
				if (round(sdr->chan[c].tx_frequency) >= round(tx_center_frequency)) {
					if (high_c < 0 || dist < high_dist) {
						high_dist = dist;
						high_c = c;
					}
				} else {
					if (low_c < 0 || dist < low_dist) {
						low_dist = dist;
						low_c = c;
					}
				}
			}
			if (sdr->paging_channel) {
				dist = fabs(tx_center_frequency - sdr->chan[sdr->paging_channel].tx_frequency);
				if (round(sdr->chan[sdr->paging_channel].tx_frequency) >= round(tx_center_frequency)) {
					if (high_c < 0 || dist < high_dist) {
						high_dist = dist;
						high_c = sdr->paging_channel;
					}
				} else {
					if (low_c < 0 || dist < low_dist) {
						low_dist = dist;
						low_c = sdr->paging_channel;
					}
				}
			}
			/* new center = center of the two frequencies aside old center */
			if (low_c >= 0 && high_c >= 0) {
				tx_center_frequency =
					((sdr->chan[low_c].tx_frequency) +
					 (sdr->chan[high_c].tx_frequency)) / 2.0;
				LOGP(DSDR, LOGL_INFO, "We move center frequency between the two channels in the middle, to prevent them from overlapping with DC level.\n");
			}
		}

		/* show spectrum */
		int tx_rate = sdr_config->split_mode ? sdr_config->tx_samplerate : sdr_config->samplerate;
		show_spectrum("TX", (double)tx_rate / 2.0, tx_center_frequency, tx_frequency, paging_frequency, channels);

		/* range of TX */
		double low_side, high_side, range;
		low_side = (tx_center_frequency - tx_low_frequency) + bandwidth / 2.0;
		high_side = (tx_high_frequency - tx_center_frequency) + bandwidth / 2.0;
		range = ((low_side > high_side) ? low_side : high_side) * 2.0;
		LOGP(DSDR, LOGL_INFO, "Total bandwidth (two sidebands) for all TX Frequencies: %.0f Hz\n", range);
		if (range > tx_rate * USABLE_BANDWIDTH) {
			LOGP(DSDR, LOGL_NOTICE, "*******************************************************************************\n");
			LOGP(DSDR, LOGL_NOTICE, "The required bandwidth of %.0f Hz exceeds %.0f%% of the sample rate.\n", range, USABLE_BANDWIDTH * 100.0);
			LOGP(DSDR, LOGL_NOTICE, "Please increase samplerate!\n");
			LOGP(DSDR, LOGL_NOTICE, "*******************************************************************************\n");
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Using center frequency: TX %.6f MHz\n", tx_center_frequency / 1e6);
		/* set offsets to center frequency */
		for (c = 0; c < channels; c++) {
			double tx_offset;
			tx_offset = sdr->chan[c].tx_frequency - tx_center_frequency;
			LOGP(DSDR, LOGL_NOTICE, "FM MOD ch%d: tx_freq=%.6f center=%.6f offset=%.1fkHz samplerate=%d\n",
			     c, sdr->chan[c].tx_frequency / 1e6, tx_center_frequency / 1e6, tx_offset / 1e3, samplerate);
			sdr->chan[c].am = am[c];
			if (am[c]) {
				double gain, bias;
				gain = modulation_index / 2.0;
				bias = 1.0 - gain;
				rc = am_mod_init(&sdr->chan[c].am_mod, samplerate, tx_offset, sdr->amplitude * gain, sdr->amplitude * bias);
			} else
				rc = fm_mod_init(&sdr->chan[c].fm_mod, samplerate, tx_offset, sdr->amplitude);
			if (rc < 0)
				goto error;
			/* Initialize TX channelizer if enabled */
			if (use_channelizer && oversample > 1) {
				sdr->chan[c].use_tx_channelizer = 1;
				rc = channelizer_tx_init(&sdr->chan[c].tx_channelizer,
				                         sdr_config->samplerate,
				                         bandwidth, tx_offset, fast_math);
				if (rc < 0) {
					LOGP(DSDR, LOGL_ERROR, "Failed to init TX channelizer for channel %d\n", c);
					goto error;
				}
				LOGP(DSDR, LOGL_INFO, "TX Channelizer ch%d: %dx interpolation, offset %.1f kHz\n",
				     c, sdr->chan[c].tx_channelizer.interpolation, tx_offset / 1e3);
			}
		}
		if (sdr->paging_channel) {
			double tx_offset;
			tx_offset = sdr->chan[sdr->paging_channel].tx_frequency - tx_center_frequency;
			LOGP(DSDR, LOGL_DEBUG, "Paging Frequency: TX offset: %.6f MHz\n", tx_offset / 1e6);
			rc = fm_mod_init(&sdr->chan[sdr->paging_channel].fm_mod, samplerate, tx_offset, sdr->amplitude);
			if (rc < 0)
				goto error;
		}
		/* show gain */
		LOGP(DSDR, LOGL_INFO, "Using gain: TX %.1f dB\n", sdr_config->tx_gain);
	}

	if (rx_frequency && !channels)
		rx_center_frequency = rx_frequency[0];
	if (rx_frequency && channels) {
		/* calculate required bandwidth (IQ rate) */
		double rx_low_frequency = 0.0, rx_high_frequency = 0.0;
		for (c = 0; c < channels; c++) {
			sdr->chan[c].rx_frequency = rx_frequency[c];
			if (c == 0 || sdr->chan[c].rx_frequency < rx_low_frequency)
				rx_low_frequency = sdr->chan[c].rx_frequency;
			if (c == 0 || sdr->chan[c].rx_frequency > rx_high_frequency)
				rx_high_frequency = sdr->chan[c].rx_frequency;
		}
		rx_center_frequency = (rx_high_frequency + rx_low_frequency) / 2.0;

		/* prevent channel bandwidth from overlapping with the center frequency */
		if (channels == 1) {
			/* simple: just move off the center by two times half of the bandwidth */
			rx_center_frequency -= 2.0 * bandwidth / 2.0;
			/* Note: rx_low_frequency is kept at old center.
			   Calculation of 'low_side' will become 0.
			   This is correct, since there is no bandwidth
			   below new center frequency.
			 */
			LOGP(DSDR, LOGL_INFO, "We shift center frequency %.0f KHz down (half bandwidth), to prevent channel from overlapping with DC level.\n", bandwidth / 2.0 / 1e3);
		} else {
			/* find two channels that are aside the center */
			double low_dist, high_dist, dist;
			int low_c = -1, high_c = -1;
			for (c = 0; c < channels; c++) {
				dist = fabs(rx_center_frequency - sdr->chan[c].rx_frequency);
				if (round(sdr->chan[c].rx_frequency) >= round(rx_center_frequency)) {
					if (high_c < 0 || dist < high_dist) {
						high_dist = dist;
						high_c = c;
					}
				} else {
					if (low_c < 0 || dist < low_dist) {
						low_dist = dist;
						low_c = c;
					}
				}
			}
			/* new center = center of the two frequencies aside old center */
			if (low_c >= 0 && high_c >= 0) {
				rx_center_frequency =
					((sdr->chan[low_c].rx_frequency) +
					 (sdr->chan[high_c].rx_frequency)) / 2.0;
				LOGP(DSDR, LOGL_INFO, "We move center frequency between the two channels in the middle, to prevent them from overlapping with DC level.\n");
			}
		}

		/* show spectrum */
		int rx_rate = sdr_config->split_mode ? sdr_config->rx_samplerate : sdr_config->samplerate;
		show_spectrum("RX", (double)rx_rate / 2.0, rx_center_frequency, rx_frequency, 0.0, channels);

		/* range of RX */
		double low_side, high_side, range;
		low_side = (rx_center_frequency - rx_low_frequency) + bandwidth / 2.0;
		high_side = (rx_high_frequency - rx_center_frequency) + bandwidth / 2.0;
		range = ((low_side > high_side) ? low_side : high_side) * 2.0;
		LOGP(DSDR, LOGL_INFO, "Total bandwidth (two sidebands) for all RX Frequencies: %.0f Hz\n", range);
		if (range > rx_rate * USABLE_BANDWIDTH) {
			LOGP(DSDR, LOGL_NOTICE, "*******************************************************************************\n");
			LOGP(DSDR, LOGL_NOTICE, "The required bandwidth of %.0f Hz exceeds %.0f%% of the sample rate.\n", range, USABLE_BANDWIDTH * 100.0);
			LOGP(DSDR, LOGL_NOTICE, "Please increase samplerate!\n");
			LOGP(DSDR, LOGL_NOTICE, "*******************************************************************************\n");
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Using center frequency: RX %.6f MHz\n", rx_center_frequency / 1e6);
		/* set offsets to center frequency */
		for (c = 0; c < channels; c++) {
			double rx_offset;
			rx_offset = sdr->chan[c].rx_frequency - rx_center_frequency;
			LOGP(DSDR, LOGL_DEBUG, "Frequency #%d: RX offset: %.6f MHz\n", c, rx_offset / 1e6);
			sdr->chan[c].am = am[c];
			sdr->chan[c].use_channelizer = use_channelizer;
			if (sdr->chan[c].use_channelizer) {
				channelizer_init(&sdr->chan[c].channelizer, sdr_config->samplerate, samplerate, rx_offset, fast_math);
				sdr->chan[c].ch_I = calloc(buffer_size, sizeof(sample_t));
				sdr->chan[c].ch_Q = calloc(buffer_size, sizeof(sample_t));
				sdr->chan[c].ch_iq_float = calloc(buffer_size * 2, sizeof(float));
				if (!sdr->chan[c].ch_I || !sdr->chan[c].ch_Q || !sdr->chan[c].ch_iq_float) {
					LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
					goto error;
				}
			}
			if (am[c])
				rc = am_demod_init(&sdr->chan[c].am_demod, samplerate, use_channelizer ? 0.0 : rx_offset, bandwidth / 2.0, 1.0 / modulation_index); /* bandwidth is only one side band */
			else
				rc = fm_demod_init(&sdr->chan[c].fm_demod, samplerate, use_channelizer ? 0.0 : rx_offset, bandwidth); /* bandwidth are deviation and both sidebands */
			if (rc < 0)
				goto error;
		}
		/* show gain */
		LOGP(DSDR, LOGL_INFO, "Using gain: RX %.1f dB\n", sdr_config->rx_gain);
		/* open wave */
		if (sdr_config->write_iq_rx_wave) {
			rc = wave_create_record(&sdr->wave_rx_rec, sdr_config->write_iq_rx_wave, samplerate, 2, 1.0);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "Failed to create WAVE recoding instance!\n");
				goto error;
			}
		}
		if (sdr_config->read_iq_rx_wave) {
			int two = 2;
			rc = wave_create_playback(&sdr->wave_rx_play, sdr_config->read_iq_rx_wave, &samplerate, &two, 1.0);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "Failed to create WAVE playback instance!\n");
				goto error;
			}
		}
		/* init measurements display */
		for (c = 0; c < channels; c++) {
			sender_t *sender = get_sender_by_empfangsfrequenz(sdr->chan[c].rx_frequency);
			if (!sender)
				continue;
			sdr->chan[c].dmp_rf_level = display_measurements_add(&sender->dispmeas, "RF Level", "%.1f dB", DISPLAY_MEAS_AVG, DISPLAY_MEAS_LEFT, -96.0, 0.0, -INFINITY);
			if (!am[c]) {
				sdr->chan[c].dmp_freq_offset = display_measurements_add(&sender->dispmeas, "Freq. Offset", "%+.2f KHz", DISPLAY_MEAS_AVG, DISPLAY_MEAS_CENTER, -max_modulation / 1000.0 * 2.0, max_modulation / 1000.0 * 2.0, 0.0);
				sdr->chan[c].dmp_deviation = display_measurements_add(&sender->dispmeas, "Deviation", "%.2f KHz", DISPLAY_MEAS_PEAK2PEAK, DISPLAY_MEAS_LEFT, 0.0, max_deviation / 1000.0 * 1.5, max_deviation / 1000.0);
			}
		}
	}

	display_iq_init(samplerate);
	display_spectrum_init(samplerate, rx_center_frequency);

	LOGP(DSDR, LOGL_INFO, "Using local oscillator offset: %.0f Hz\n", sdr_config->lo_offset);

	/* Apply upconverter offset only when center frequency is set (i.e., that direction is used) */
	double actual_tx_center = (tx_center_frequency != 0.0) ? tx_center_frequency + sdr_config->tx_upconverter : 0.0;
	double actual_rx_center = (rx_center_frequency != 0.0) ? rx_center_frequency + sdr_config->rx_upconverter : 0.0;

	if (sdr_config->tx_upconverter != 0.0 && tx_center_frequency != 0.0) {
		LOGP(DSDR, LOGL_INFO, "Upconverter TX: %.6f MHz + %.6f MHz = %.6f MHz (SDR tuning)\n",
		     tx_center_frequency / 1e6, sdr_config->tx_upconverter / 1e6, actual_tx_center / 1e6);
	}
	if (sdr_config->rx_upconverter != 0.0 && rx_center_frequency != 0.0) {
		LOGP(DSDR, LOGL_INFO, "Upconverter RX: %.6f MHz + %.6f MHz = %.6f MHz (SDR tuning)\n",
		     rx_center_frequency / 1e6, sdr_config->rx_upconverter / 1e6, actual_rx_center / 1e6);
	}

	/* Reset per-direction driver tracking */
	tx_driver = 0;
	rx_driver = 0;

	if (sdr_config->split_mode) {
		/* Split mode: open TX and RX devices independently, possibly with different drivers */
#ifdef HAVE_UHD
		if (sdr_config->tx_uhd) {
			rc = uhd_open(sdr_config->tx_channel, sdr_config->tx_device_args, sdr_config->tx_stream_args, sdr_config->tx_tune_args, sdr_config->tx_antenna, NULL, sdr_config->tx_clock_source, actual_tx_center, 0.0, sdr_config->tx_lo_offset, (double)sdr_config->tx_samplerate, sdr_config->tx_gain, 0.0, sdr_config->tx_bandwidth, sdr_config->timestamps);
			if (rc)
				goto error;
			tx_driver = 1;
		}
#endif
#ifdef HAVE_SOAPY
		if (sdr_config->tx_soapy) {
			rc = soapy_open(sdr_config->tx_channel, sdr_config->tx_device_args, sdr_config->tx_stream_args, sdr_config->tx_tune_args, sdr_config->tx_antenna, NULL, sdr_config->tx_clock_source, actual_tx_center, 0.0, sdr_config->tx_lo_offset, (double)sdr_config->tx_samplerate, sdr_config->tx_gain, 0.0, sdr_config->tx_bandwidth, sdr_config->timestamps);
			if (rc)
				goto error;
			tx_driver = 2;
		}
#endif
#ifdef HAVE_RPITX
		if (sdr_config->tx_rpitx) {
			rc = rpitx_open(actual_tx_center, (double)sdr_config->tx_samplerate, sdr_config->tx_gain);
			if (rc)
				goto error;
			tx_driver = 3;
		}
#endif
#ifdef HAVE_UHD
		if (sdr_config->rx_uhd) {
			rc = uhd_open(sdr_config->rx_channel, sdr_config->rx_device_args, sdr_config->rx_stream_args, sdr_config->rx_tune_args, NULL, sdr_config->rx_antenna, sdr_config->rx_clock_source, 0.0, actual_rx_center, sdr_config->rx_lo_offset, (double)sdr_config->rx_samplerate, 0.0, sdr_config->rx_gain, sdr_config->rx_bandwidth, sdr_config->timestamps);
			if (rc)
				goto error;
			rx_driver = 1;
		}
#endif
#ifdef HAVE_SOAPY
		if (sdr_config->rx_soapy) {
			rc = soapy_open(sdr_config->rx_channel, sdr_config->rx_device_args, sdr_config->rx_stream_args, sdr_config->rx_tune_args, NULL, sdr_config->rx_antenna, sdr_config->rx_clock_source, 0.0, actual_rx_center, sdr_config->rx_lo_offset, (double)sdr_config->rx_samplerate, 0.0, sdr_config->rx_gain, sdr_config->rx_bandwidth, sdr_config->timestamps);
			if (rc)
				goto error;
			rx_driver = 2;
		}
#endif
	} else {
		/* Single-device mode (including TX-only / RX-only): existing dispatch */
#ifdef HAVE_UHD
		if (sdr_config->uhd) {
			rc = uhd_open(sdr_config->channel, sdr_config->device_args, sdr_config->stream_args, sdr_config->tune_args, sdr_config->tx_antenna, sdr_config->rx_antenna, sdr_config->clock_source, actual_tx_center, actual_rx_center, sdr_config->lo_offset, sdr_config->samplerate, sdr_config->tx_gain, sdr_config->rx_gain, sdr_config->bandwidth, sdr_config->timestamps);
			if (rc)
				goto error;
			if (actual_tx_center != 0.0)
				tx_driver = 1;
			if (actual_rx_center != 0.0)
				rx_driver = 1;
		}
#endif
#ifdef HAVE_SOAPY
		if (sdr_config->soapy) {
			rc = soapy_open(sdr_config->channel, sdr_config->device_args, sdr_config->stream_args, sdr_config->tune_args, sdr_config->tx_antenna, sdr_config->rx_antenna, sdr_config->clock_source, actual_tx_center, actual_rx_center, sdr_config->lo_offset, sdr_config->samplerate, sdr_config->tx_gain, sdr_config->rx_gain, sdr_config->bandwidth, sdr_config->timestamps);
			if (rc)
				goto error;
			if (actual_tx_center != 0.0)
				tx_driver = 2;
			if (actual_rx_center != 0.0)
				rx_driver = 2;
		}
#endif
#ifdef HAVE_RPITX
		if (sdr_config->rpitx) {
			rc = rpitx_open(actual_tx_center, (double)sdr_config->samplerate, sdr_config->tx_gain);
			if (rc)
				goto error;
			tx_driver = 3;
		}
#endif
	}

	/* Initialize SDR status monitoring */
	sdr_status_init(channels, 1.0);

	/* FFT-based spectral measurement is configured from main.c after sdr_open,
	 * using the same data path as the scan engine (decimated DSP-rate IQ). */

	return sdr;

error:
	sdr_close(sdr);
	return NULL;
}

void *sdr_open(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index)
{
	return sdr_open_internal(direction, audiodev, tx_frequency, rx_frequency, am, channels, paging_frequency, samplerate, buffer_size, interval, max_deviation, max_modulation, modulation_index, 0, 0);
}

void *sdr_open_channelizer(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index, int use_channelizer, int fast_math)
{
	return sdr_open_internal(direction, audiodev, tx_frequency, rx_frequency, am, channels, paging_frequency, samplerate, buffer_size, interval, max_deviation, max_modulation, modulation_index, use_channelizer, fast_math);
}

double bias_I, bias_Q; /* calculated bias */
int bias_count = -1; /* number of calculations */

void calibrate_bias(void)
{
	bias_count = 0;
	bias_I = 0.0;
	bias_Q = 0.0;
}


/* Set RX frequency on the active SDR backend */
int sdr_set_rx_frequency(double freq)
{
#ifdef HAVE_UHD
	if (rx_driver == 1)
		return uhd_set_rx_frequency(freq);
#endif
#ifdef HAVE_SOAPY
	if (rx_driver == 2)
		return soapy_set_rx_frequency(freq);
#endif
	LOGP(DSDR, LOGL_ERROR, "Cannot set RX frequency: no SDR backend active\n");
	return -ENODEV;
}

/* Set TX frequency on the active SDR backend */
int sdr_set_tx_frequency(double freq)
{
#ifdef HAVE_UHD
	if (tx_driver == 1)
		return uhd_set_tx_frequency(freq);
#endif
#ifdef HAVE_SOAPY
	if (tx_driver == 2)
		return soapy_set_tx_frequency(freq);
#endif
	LOGP(DSDR, LOGL_ERROR, "Cannot set TX frequency: no SDR backend active\n");
	return -ENODEV;
}

/* Get the negotiated sample rate */
int sdr_get_samplerate(void *inst)
{
	sdr_t *sdr = (sdr_t *)inst;
	if (!sdr)
		return 0;
	return sdr->samplerate;
}

/* Calculate optimal channel rate based on master rate and bandwidth */
int sdr_calculate_optimal_rate(int master_rate, double bandwidth)
{
	int rate = master_rate;
	/* Minimum rate requirement: signal processing sample rate must be at least
	 * one third greater than the signal's double bandwidth.
	 * This means: rate >= bandwidth * 2 / 0.75 = bandwidth * 2.666... */
	double min_rate = bandwidth * 2.0 / 0.75;
	
	/* We need at least the minimum required rate.
	 * Decimate by powers of 2 until we hit the limit. */
	while ((rate % 2) == 0) {
		int next_rate = rate / 2;
		if (next_rate < min_rate)
			break; /* Too low, keep current rate */
		rate = next_rate;
	}
	
	/* Ensure we meet the minimum requirement */
	if (rate < min_rate) {
		/* Round up to next power of 2 that meets the requirement */
		int pow2 = 1;
		while (pow2 < min_rate && pow2 < master_rate)
			pow2 *= 2;
		rate = pow2;
	}
	
	return rate;
}

static void sdr_bias(float *buffer, int count)
{
	int i;

	if (bias_count < sdr_config->samplerate) {
		for (i = 0; i < count; i++) {
			bias_I += *buffer++;
			bias_Q += *buffer++;
		}
		bias_count += count;
		if (bias_count >= sdr_config->samplerate) {
			bias_I /= bias_count;
			bias_Q /= bias_count;
			LOGP(DSDR, LOGL_INFO, "DC bias calibration finished.\n");
		}
	} else {
		for (i = 0; i < count; i++) {
			*buffer++ -= bias_I;
			*buffer++ -= bias_Q;
		}
	}
}

static void *sdr_write_child(void *arg)
{
	sdr_t *sdr = (sdr_t *)arg;
	int num;
	int fill, out;
	int s, ss, o;

	/* Diagnostics for split mode */
	static int diag_count = 0;
	static int empty_count = 0;
	static int total_samples = 0;
	static double last_diag_time = 0.0;
	static int max_fill = 0;
	static int min_fill = INT_MAX;
	int split_mode = sdr_config && sdr_config->split_mode;

	/* Use tx_oversample for TX operations in split mode (only for integer resampling) */
	int tx_os = split_mode ? sdr->tx_oversample : sdr->oversample;
	
	/* Check if using polyphase resampler for TX */
	int use_tx_polyphase = sdr->tx_use_polyphase;
	static int tx_polyphase_logged = 0;

	/* Check if TX channelizer is enabled (only check channel 0 for now) */
	int use_tx_chan = 0;
	if (sdr->chan && sdr->chan[0].use_tx_channelizer)
		use_tx_chan = 1;

	while (sdr->thread_write.running) {
		/* write to SDR */
		fill = (sdr->thread_write.in - sdr->thread_write.out + sdr->thread_write.buffer_size) % sdr->thread_write.buffer_size;
		num = fill / 2;
		
		/* Track buffer statistics for split mode diagnostics */
		if (split_mode) {
			diag_count++;
			if (num == 0) {
				empty_count++;
			} else {
				total_samples += num;
				if (fill > max_fill) max_fill = fill;
				if (fill < min_fill) min_fill = fill;
			}
			
			/* Log diagnostics every second */
			double now = get_time();
			if (last_diag_time == 0.0) last_diag_time = now;
			if (now - last_diag_time >= 1.0) {
				LOGP(DSDR, LOGL_DEBUG, "WRITE_THREAD: iterations=%d empty=%d (%.1f%%) samples=%d avg_fill=%d min=%d max=%d buf_size=%d\n",
				     diag_count, empty_count, 
				     diag_count > 0 ? (100.0 * empty_count / diag_count) : 0.0,
				     total_samples,
				     diag_count - empty_count > 0 ? total_samples / (diag_count - empty_count) : 0,
				     min_fill == INT_MAX ? 0 : min_fill,
				     max_fill,
				     sdr->thread_write.buffer_size);
				diag_count = 0;
				empty_count = 0;
				total_samples = 0;
				max_fill = 0;
				min_fill = INT_MAX;
				last_diag_time = now;
			}
		}
		
		if (num) {
#ifdef DEBUG_BUFFER
			printf("Thread found %d samples in write buffer and forwards them to SDR.\n", num);
#endif
			out = sdr->thread_write.out;
			int out_samples;  /* Number of samples to send to SDR */

			if (use_tx_chan && sdr->tx_chan_I && sdr->tx_chan_Q) {
				/* TX Channelizer path: proper polyphase interpolation */
				
				/* De-interleave I/Q from buffer and apply scaling */
				for (s = 0; s < num; s++) {
					sdr->tx_chan_I[s] = sdr->thread_write.buffer[out] * LIMIT_IQ_LEVEL;
					sdr->tx_chan_Q[s] = sdr->thread_write.buffer[out + 1] * LIMIT_IQ_LEVEL;
					out = (out + 2) % sdr->thread_write.buffer_size;
				}
				sdr->thread_write.out = out;
				
				/* Clear output buffer before channelizer (it ADDS to output) */
				memset(sdr->thread_write.buffer2, 0, sizeof(float) * num * tx_os * 2);
				
				/* Process through TX channelizer (interpolate + filter) */
				channelizer_tx_process(&sdr->chan[0].tx_channelizer,
				                       sdr->tx_chan_I, sdr->tx_chan_Q, num,
				                       sdr->thread_write.buffer2);
				out_samples = num * tx_os;
			} else if (use_tx_polyphase && sdr->tx_poly_i && sdr->tx_poly_q) {
				/* Polyphase resampler path: arbitrary ratio upsampling */
				static int tx_poly_debug_count = 0;
				if (!tx_polyphase_logged) {
					LOGP(DSDR, LOGL_NOTICE, "TX POLYPHASE UPSAMPLE: DSP %d Hz -> SDR %d Hz (ratio=%.6f)\n",
					     sdr->samplerate, sdr_config->tx_samplerate,
					     (double)sdr_config->tx_samplerate / (double)sdr->samplerate);
					tx_polyphase_logged = 1;
				}
				
				/* De-interleave I/Q from buffer into pre-allocated polyphase input buffers */
				sample_t *in_i = sdr->tx_poly_i;
				sample_t *in_q = sdr->tx_poly_q;
				double in_max = 0;
				for (s = 0; s < num; s++) {
					in_i[s] = sdr->thread_write.buffer[out] * LIMIT_IQ_LEVEL;
					in_q[s] = sdr->thread_write.buffer[out + 1] * LIMIT_IQ_LEVEL;
					if (fabs(in_i[s]) > in_max) in_max = fabs(in_i[s]);
					if (fabs(in_q[s]) > in_max) in_max = fabs(in_q[s]);
					out = (out + 2) % sdr->thread_write.buffer_size;
				}
				sdr->thread_write.out = out;
				
				/* Resample I and Q through polyphase filter
				 * Output goes directly into buffer2 (de-interleaved first, then interleaved) */
				int max_out = polyphase_output_num(&sdr->tx_polyphase, num) + 4;
				
				/* Use second half of tx_poly buffers for output (they're sized for max_out) */
				sample_t *out_i = in_i + sdr->buffer_size;  /* Offset past input area */
				sample_t *out_q = in_q + sdr->buffer_size;
				
				out_samples = polyphase_resample_iq(&sdr->tx_polyphase,
				                                    in_i, in_q, num,
				                                    out_i, out_q, max_out);
				
				/* Debug: check output */
				double out_max = 0;
				for (s = 0; s < out_samples; s++) {
					if (fabs(out_i[s]) > out_max) out_max = fabs(out_i[s]);
					if (fabs(out_q[s]) > out_max) out_max = fabs(out_q[s]);
				}
				tx_poly_debug_count++;
				if (tx_poly_debug_count % 50 == 1) {
					LOGP(DSDR, LOGL_NOTICE, "TX POLY: in=%d out=%d max_out=%d | in_max=%.4f out_max=%.4f\n",
					     num, out_samples, max_out, in_max, out_max);
				}
				
				/* Interleave I/Q into output buffer */
				for (s = 0; s < out_samples; s++) {
					sdr->thread_write.buffer2[s * 2] = out_i[s];
					sdr->thread_write.buffer2[s * 2 + 1] = out_q[s];
				}
			} else {
				/* Legacy path: zero-order hold interpolation + IIR filter */
				static int tx_upsample_logged = 0;
				if (!tx_upsample_logged) {
					LOGP(DSDR, LOGL_NOTICE, "TX INTEGER UPSAMPLE: in=%d tx_os=%d out=%d\n", num, tx_os, num * tx_os);
					tx_upsample_logged = 1;
				}
				for (s = 0, ss = 0; s < num; s++) {
					for (o = 0; o < tx_os; o++) {
						sdr->thread_write.buffer2[ss++] = sdr->thread_write.buffer[out] * LIMIT_IQ_LEVEL;
						sdr->thread_write.buffer2[ss++] = sdr->thread_write.buffer[out + 1] * LIMIT_IQ_LEVEL;
					}
					out = (out + 2) % sdr->thread_write.buffer_size;
				}
				sdr->thread_write.out = out;
#ifndef DISABLE_FILTER
				/* filter spectrum */
				if (tx_os > 1) {
					iir_process_baseband(&sdr->thread_write.lp[0], sdr->thread_write.buffer2, num * tx_os);
					iir_process_baseband(&sdr->thread_write.lp[1], sdr->thread_write.buffer2 + 1, num * tx_os);
				}
#endif
				out_samples = num * tx_os;
			}

#ifdef HAVE_UHD
			if (tx_driver == 1)
				uhd_send(sdr->thread_write.buffer2, out_samples);
#endif
#ifdef HAVE_SOAPY
			if (tx_driver == 2)
				soapy_send(sdr->thread_write.buffer2, out_samples);
#endif
#ifdef HAVE_RPITX
			if (tx_driver == 3)
				rpitx_send(sdr->thread_write.buffer2, out_samples);
#endif
		}

		/* delay some time */
		usleep(sdr->interval * 1000.0);
	}

	LOGP(DSDR, LOGL_DEBUG, "Thread received exit!\n");
	sdr->thread_write.exit = 1;
	return NULL;
}

static void *sdr_read_child(void *arg)
{
	sdr_t *sdr = (sdr_t *)arg;
	int num, count = 0;
	int space, in;
	int s, ss;
	int split_mode = sdr_config && sdr_config->split_mode;

	/* Use rx_oversample for RX operations in split mode */
	int rx_os = split_mode ? sdr->rx_oversample : sdr->oversample;

	/* DEBUG: track thread activity */
	static int thr_dbg_cnt = 0;
	static int thr_total_written = 0;
	static int thr_zero_space = 0;
	static int thr_zero_count = 0;
	static double thr_timer = 0;

	while (sdr->thread_read.running) {
		/* read from SDR */
		space = (sdr->thread_read.out - sdr->thread_read.in + sdr->thread_read.buffer_size - 2 + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size;
		num = space / 2;
		if (num) {
#ifdef HAVE_UHD
			if (rx_driver == 1)
				count = uhd_receive(sdr->thread_read.buffer2, num);
#endif
#ifdef HAVE_SOAPY
			if (rx_driver == 2)
				count = soapy_receive(sdr->thread_read.buffer2, num);
#endif
			if (bias_count >= 0)
				sdr_bias(sdr->thread_read.buffer2, count);
			if (count > 0) {
				thr_total_written += count;
				
				/* Diagnostics: track RX thread ring buffer writes */
				if (sdr->rx_use_polyphase) {
					static int rx_thr_calls = 0;
					static int rx_thr_total_written = 0;
					static double rx_thr_timer = 0;
					rx_thr_calls++;
					rx_thr_total_written += count;
					if (rx_thr_timer == 0.0)
						rx_thr_timer = get_time();
					double now = get_time();
					if (now - rx_thr_timer >= 1.0) {
						int fill_after = (sdr->thread_read.in - sdr->thread_read.out + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size;
						LOGP(DSDR, LOGL_NOTICE, "RX THREAD 1s: calls=%d written=%d samples | in=%d out=%d fill=%d/%d bufsz=%d\n",
						     rx_thr_calls, rx_thr_total_written,
						     sdr->thread_read.in, sdr->thread_read.out,
						     fill_after / 2, sdr->thread_read.buffer_size / 2,
						     sdr->thread_read.buffer_size);
						rx_thr_calls = 0;
						rx_thr_total_written = 0;
						rx_thr_timer = now;
					}
				}
#ifdef DEBUG_BUFFER
				printf("Thread read %d samples from SDR and writes them to read buffer.\n", count);
#endif
#ifndef DISABLE_FILTER
				/* filter spectrum */
				/* If channelizer is used (RX or TX), we MUST skip this filter because
				 * it is a lowpass at center frequency. It would kill offset channels.
				 * The channelizer handles filtering and decimation internally, which is
				 * more efficient and allows extracting narrow channels (NFM, AM) from
				 * a wideband spectrum. */
				int skip_filter = 0;
				if (sdr->chan && (sdr->chan[0].use_channelizer || sdr->chan[0].use_tx_channelizer))
					skip_filter = 1;

				if (!skip_filter && rx_os > 1) {
					iir_process_baseband(&sdr->thread_read.lp[0], sdr->thread_read.buffer2, count);
					iir_process_baseband(&sdr->thread_read.lp[1], sdr->thread_read.buffer2 + 1, count);
				}
#endif
				in = sdr->thread_read.in;
				for (s = 0, ss = 0; s < count; s++) {
					sdr->thread_read.buffer[in++] = sdr->thread_read.buffer2[ss++];
					sdr->thread_read.buffer[in++] = sdr->thread_read.buffer2[ss++];
					in %= sdr->thread_read.buffer_size;
				}
				__sync_synchronize();  /* Memory barrier to ensure writes are visible */
				sdr->thread_read.in = in;
			} else {
				thr_zero_count++;
			}
		} else {
			thr_zero_space++;
		}

		thr_dbg_cnt++;
		if (thr_timer == 0.0) thr_timer = get_time();
		double now = get_time();
		if (now - thr_timer >= 1.0) {
			int fill = (sdr->thread_read.in - sdr->thread_read.out + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size;
			LOGP(DSDR, LOGL_NOTICE, "RX_THREAD: calls=%d written=%d zero_space=%d zero_count=%d | fill=%d/%d\n",
			     thr_dbg_cnt, thr_total_written, thr_zero_space, thr_zero_count,
			     fill/2, sdr->thread_read.buffer_size/2);
			thr_dbg_cnt = 0;
			thr_total_written = 0;
			thr_zero_space = 0;
			thr_zero_count = 0;
			thr_timer = now;
		}

		/* If receive functions block, we always receive something, so don't sleep. */
		if (!num || count <= 0)
			usleep(sdr->interval * 1000.0);
	}

	LOGP(DSDR, LOGL_DEBUG, "Thread received exit!\n");
	sdr->thread_read.exit = 1;
	return NULL;
}

/* start streaming */
int sdr_start(void *inst)
{
	sdr_t *sdr = (sdr_t *)inst;
	int rc = -EINVAL;

#ifdef HAVE_UHD
	if (tx_driver == 1 || rx_driver == 1)
		rc = uhd_start();
#endif
#ifdef HAVE_SOAPY
	if (tx_driver == 2 || rx_driver == 2)
		rc = soapy_start();
#endif
#ifdef HAVE_RPITX
	if (tx_driver == 3)
		rc = rpitx_start();
#endif
	if (rc < 0)
		return rc;

	if (sdr->threads) {
		int rc;
		pthread_t tid;
		char tname[64];

		LOGP(DSDR, LOGL_DEBUG, "Create threads!\n");
		if (tx_driver != 0) {
			sdr->thread_write.running = 1;
			sdr->thread_write.exit = 0;
			rc = pthread_create(&tid, NULL, sdr_write_child, inst);
			if (rc < 0) {
				sdr->thread_write.running = 0;
				LOGP(DSDR, LOGL_ERROR, "Failed to create thread!\n");
				return rc;
			}
			pthread_getname_np(tid, tname, sizeof(tname));
			strncat(tname, "-sdr_tx", sizeof(tname) - 7 - 1);
			tname[sizeof(tname) - 1] = '\0';
			pthread_setname_np(tid, tname);
		}
		if (rx_driver != 0) {
			sdr->thread_read.running = 1;
			sdr->thread_read.exit = 0;
			rc = pthread_create(&tid, NULL, sdr_read_child, inst);
			if (rc < 0) {
				sdr->thread_read.running = 0;
				LOGP(DSDR, LOGL_ERROR, "Failed to create thread!\n");
				return rc;
			}
			pthread_getname_np(tid, tname, sizeof(tname));
			strncat(tname, "-sdr_rx", sizeof(tname) - 7 - 1);
			tname[sizeof(tname) - 1] = '\0';
			pthread_setname_np(tid, tname);
		}
	}

	return 0;
}

void sdr_close(void *inst)
{
	sdr_t *sdr = (sdr_t *)inst;

	LOGP(DSDR, LOGL_DEBUG, "Close SDR device\n");

	if (sdr->threads) {
		if (sdr->thread_write.running) {
			LOGP(DSDR, LOGL_DEBUG, "Thread sending exit!\n");
			sdr->thread_write.running = 0;
			while (sdr->thread_write.exit == 0)
				usleep(1000);
		}
		if (sdr->thread_read.running) {
			LOGP(DSDR, LOGL_DEBUG, "Thread sending exit!\n");
			sdr->thread_read.running = 0;
			while (sdr->thread_read.exit == 0)
				usleep(1000);
		}
	}

	if (sdr->thread_read.buffer)
		free((void *)sdr->thread_read.buffer);
	if (sdr->thread_read.buffer2)
		free((void *)sdr->thread_read.buffer2);
	if (sdr->thread_write.buffer)
		free((void *)sdr->thread_write.buffer);
	if (sdr->thread_write.buffer2)
		free((void *)sdr->thread_write.buffer2);

#ifdef HAVE_UHD
	if (tx_driver == 1 || rx_driver == 1)
		uhd_close();
#endif

#ifdef HAVE_SOAPY
	if (tx_driver == 2 || rx_driver == 2)
		soapy_close();
#endif

#ifdef HAVE_RPITX
	if (tx_driver == 3)
		rpitx_close();
#endif

	tx_driver = 0;
	rx_driver = 0;

	if (sdr) {
		free(sdr->modbuff);
		free(sdr->modbuff_I);
		free(sdr->modbuff_Q);
		free(sdr->modbuff_carrier);
		free(sdr->wavespl0);
		free(sdr->wavespl1);
		wave_destroy_record(&sdr->wave_rx_rec);
		wave_destroy_record(&sdr->wave_tx_rec);
		wave_destroy_playback(&sdr->wave_rx_play);
		wave_destroy_playback(&sdr->wave_tx_play);
		if (sdr->chan) {
			int c;

			for (c = 0; c < sdr->channels; c++) {
				fm_mod_exit(&sdr->chan[c].fm_mod);
				fm_demod_exit(&sdr->chan[c].fm_demod);
				am_mod_exit(&sdr->chan[c].am_mod);
				am_demod_exit(&sdr->chan[c].am_demod);
				if (sdr->chan[c].use_channelizer)
					channelizer_exit(&sdr->chan[c].channelizer);
				if (sdr->chan[c].ch_I) free(sdr->chan[c].ch_I);
				if (sdr->chan[c].ch_Q) free(sdr->chan[c].ch_Q);
				if (sdr->chan[c].ch_iq_float) free(sdr->chan[c].ch_iq_float);
			}
			if (sdr->paging_channel)
				fm_mod_exit(&sdr->chan[sdr->paging_channel].fm_mod);
			free(sdr->chan);
		}
		if (sdr->chan_in_buff)
			free(sdr->chan_in_buff);
		if (sdr->tx_chan_I)
			free(sdr->tx_chan_I);
		if (sdr->tx_chan_Q)
			free(sdr->tx_chan_Q);
		/* Free radio channelizer resources */
		if (sdr->radio_channelizer_init) {
			channelizer_exit(&sdr->radio_channelizer);
			if (sdr->radio_ch_I) free(sdr->radio_ch_I);
			if (sdr->radio_ch_Q) free(sdr->radio_ch_Q);
		}
		/* Free polyphase resampler resources */
		if (sdr->tx_use_polyphase) {
			polyphase_free(&sdr->tx_polyphase);
			if (sdr->tx_poly_i) free(sdr->tx_poly_i);
			if (sdr->tx_poly_q) free(sdr->tx_poly_q);
		}
		if (sdr->rx_use_polyphase) {
			polyphase_free(&sdr->rx_polyphase);
			if (sdr->rx_poly_i) free(sdr->rx_poly_i);
			if (sdr->rx_poly_q) free(sdr->rx_poly_q);
			if (sdr->rx_poly_sdr_buff) free(sdr->rx_poly_sdr_buff);
		}
		free(sdr);
		sdr = NULL;
	}

	display_spectrum_exit();
}

static double get_time(void)
{
	static struct timespec tv;

	clock_gettime(CLOCK_REALTIME, &tv);

	return (double)tv.tv_sec + (double)tv.tv_nsec / 1000000000.0;
}

int sdr_write(void *inst, sample_t **samples, uint8_t **power, int num, enum paging_signal __attribute__((unused)) *paging_signal, int *on, int channels)
{
	sdr_t *sdr = (sdr_t *)inst;
	float *buff = NULL;
	int c, s, ss;
	int sent = 0;

	/* Safety check: don't write if TX is not configured (rx_only mode) */
	if (sdr_config && sdr_config->rx_only) {
		LOGP(DSDR, LOGL_ERROR, "sdr_write() called in RX-only mode - this is a bug!\n");
		return num;  /* Pretend we sent everything */
	}

	if (num > sdr->buffer_size) {
		fprintf(stderr, "exceeding maximum size given by sdr->buffer_size, please fix!\n");
		abort();
	}
	if (channels != sdr->channels && channels != 0) {
		LOGP(DSDR, LOGL_ERROR, "Invalid number of channels, please fix!\n");
		abort();
	}

	/* process all channels */
	if (channels) {
		buff = sdr->modbuff;
		memset(buff, 0, sizeof(*buff) * num * 2);
		for (c = 0; c < channels; c++) {
			/* switch to paging channel, if requested */
			if (on[c] && sdr->paging_channel)
				fm_modulate_complex(&sdr->chan[sdr->paging_channel].fm_mod, samples[c], power[c], num, buff);
			else if (sdr->chan[c].am) {
				am_modulate_complex(&sdr->chan[c].am_mod, samples[c], power[c], num, buff);
			} else
				fm_modulate_complex(&sdr->chan[c].fm_mod, samples[c], power[c], num, buff);
		}

		/* TX level diagnostics via status objects */
		{
			double chan_tx_freq[SDR_STATUS_MAX_CHANNELS];
			for (c = 0; c < channels && c < SDR_STATUS_MAX_CHANNELS; c++)
				chan_tx_freq[c] = sdr->chan[c].tx_frequency;
			sdr_status_tx_accumulate(buff, num, power, channels, sdr->amplitude, chan_tx_freq);
		}
	} else {
		buff = (float *)samples;
	}

	if (sdr->wave_tx_rec.fp) {
		sample_t *spl_list[2] = { sdr->wavespl0, sdr->wavespl1 };
		for (s = 0, ss = 0; s < num; s++) {
			spl_list[0][s] = buff[ss++];
			spl_list[1][s] = buff[ss++];
		}
		wave_write(&sdr->wave_tx_rec, spl_list, num);
	}
	if (sdr->wave_tx_play.fp) {
		sample_t *spl_list[2] = { sdr->wavespl0, sdr->wavespl1 };
		wave_read(&sdr->wave_tx_play, spl_list, num);
		for (s = 0, ss = 0; s < num; s++) {
			buff[ss++] = spl_list[0][s];
			buff[ss++] = spl_list[1][s];
		}
	}

	if (sdr->threads) {
		/* store data towards SDR in ring buffer */
		int fill, space, in;

		fill = (sdr->thread_write.in - sdr->thread_write.out + sdr->thread_write.buffer_size) % sdr->thread_write.buffer_size;
		space = (sdr->thread_write.out - sdr->thread_write.in + sdr->thread_write.buffer_size - 2 + sdr->thread_write.buffer_size) % sdr->thread_write.buffer_size;

		/* debug fill level */
		if (fill > sdr->thread_write.max_fill)
			sdr->thread_write.max_fill = fill;
		if (sdr->thread_write.max_fill_timer == 0.0)
			sdr->thread_write.max_fill_timer = get_time();
		if (get_time() - sdr->thread_write.max_fill_timer > 1.0) {
			double delay;
			delay = (double)sdr->thread_write.max_fill / 2.0 / (double)sdr->samplerate;
			(void)delay;
			sdr->thread_write.max_fill = 0;
			sdr->thread_write.max_fill_timer += 1.0;
			// LOGP(DSDR, LOGL_DEBUG, "write delay = %.3f ms\n", delay * 1000.0);
		}

		if (space < num * 2) {
			LOGP(DSDR, LOGL_ERROR, "Write SDR buffer overflow!\n");
			num = space / 2;
		}
#ifdef DEBUG_BUFFER
		printf("Writing %d samples to write buffer.\n", num);
#endif
		in = sdr->thread_write.in;
		for (s = 0, ss = 0; s < num; s++) {
			sdr->thread_write.buffer[in++] = buff[ss++];
			sdr->thread_write.buffer[in++] = buff[ss++];
			in %= sdr->thread_write.buffer_size;
		}
		sdr->thread_write.in = in;
		sent = num;
	} else {
#ifdef HAVE_UHD
		if (tx_driver == 1)
			sent = uhd_send(buff, num);
#endif
#ifdef HAVE_SOAPY
		if (tx_driver == 2)
			sent = soapy_send(buff, num);
#endif
#ifdef HAVE_RPITX
		if (tx_driver == 3)
			sent = rpitx_send(buff, num);
#endif
		if (sent < 0)
			return sent;
	}
	
	return sent;
}

int sdr_read(void *inst, sample_t **samples, int num, int channels, double *rf_level_db)
{
	sdr_t *sdr = (sdr_t *)inst;
	float *buff = NULL;
	int count = 0;
	int c, s, ss;
	int split_mode = sdr_config && sdr_config->split_mode;

	/* Safety check: don't read if RX is not configured (tx_only mode) */
	if (sdr_config && sdr_config->tx_only) {
		LOGP(DSDR, LOGL_ERROR, "sdr_read() called in TX-only mode - this is a bug!\n");
		return 0;  /* No samples available */
	}

	/* Use rx_oversample for RX operations in split mode (only for integer resampling) */
	int rx_os = split_mode ? sdr->rx_oversample : sdr->oversample;
	
	/* Check if using polyphase resampler for RX */
	int use_rx_polyphase = sdr->rx_use_polyphase;
	static int rx_polyphase_logged = 0;

	if (num > sdr->buffer_size) {
		fprintf(stderr, "exceeding maximum size given by sdr->buffer_size, please fix!\n");
		abort();
	}

	/* Check if channelizer is enabled:
	 * - For channel-based mode: check sdr->chan[0].use_channelizer
	 * - For radio mode (channels=0): check if chan_in_buff was allocated
	 *   This enables decimation-only mode where SDR decimates but doesn't demodulate */
	int use_channelizer = (channels && sdr->chan && sdr->chan[0].use_channelizer) ||
	                      (!channels && sdr->chan_in_buff != NULL && rx_os > 1);
	int input_num = num;

	/* Calculate how many SDR samples we need to read:
	 * - Channelizer: num * rx_os (integer decimation)
	 * - Polyphase: num * ratio (arbitrary decimation)
	 * - Legacy integer: num * rx_os (but we skip samples)
	 */
	if (use_channelizer) {
		input_num = num * rx_os;
		if (sdr->chan_in_buff)
			buff = (float *)sdr->chan_in_buff;
		else {
			fprintf(stderr, "Channelizer buffer not allocated! Bug?\n");
			abort();
		}
	} else if (use_rx_polyphase) {
		/* Polyphase: need to read enough SDR samples to produce 'num' output samples */
		input_num = polyphase_input_num(&sdr->rx_polyphase, num) + 4;
		if (channels) {
			buff = sdr->modbuff;
		} else {
			buff = (float *)samples;
		}
	} else {
		if (channels) {
			buff = sdr->modbuff;
		} else {
			buff = (float *)samples;
		}
	}

	if (sdr->threads) {
		/* load data from SDR out of ring buffer */
		int fill, out;
		volatile int current_in;

		__sync_synchronize();  /* Memory barrier to ensure we see latest writes */
		current_in = sdr->thread_read.in;
		fill = (current_in - sdr->thread_read.out + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size;

		int available_sdr_samples = fill / 2;  /* SDR-rate samples available in buffer */

		if (use_channelizer) {
			int available_dsp_samples = available_sdr_samples / rx_os;
			if (available_dsp_samples < num)
				num = available_dsp_samples;
			input_num = num * rx_os;
			/* DEBUG: show buffer state for channelizer */
			static int buf_dbg = 0;
			if (++buf_dbg >= 100) {
				LOGP(DSDR, LOGL_DEBUG, "CHAN BUF: fill=%d avail_sdr=%d avail_dsp=%d num=%d input_num=%d\n",
				     fill, available_sdr_samples, available_dsp_samples, num, input_num);
				buf_dbg = 0;
			}
		} else if (use_rx_polyphase) {
			/* Don't clamp num — the resampler tracks exact in_count/out_count
			 * state and will produce the correct number of outputs.
			 * Just read what we need, or all available if less. */
			int needed_sdr = polyphase_input_num(&sdr->rx_polyphase, num);
			input_num = (needed_sdr < available_sdr_samples) ? needed_sdr : available_sdr_samples;
			/* num stays at the full requested amount — passed as output_max
			 * to the resampler, which will produce fewer if input is short */
		} else {
			int available_dsp_samples = available_sdr_samples / rx_os;
			if (available_dsp_samples < num)
				num = available_dsp_samples;
		}

#ifdef DEBUG_BUFFER
		printf("Reading %d samples from read buffer.\n", num);
#endif
		out = sdr->thread_read.out;
		
		/* Reading loop - determine how many samples to read from ring buffer */
		int read_count;
		if (use_channelizer) {
			read_count = input_num;  /* Channelizer: read all oversampled samples */
		} else if (use_rx_polyphase) {
			read_count = input_num;  /* Polyphase: read all available SDR samples */
		} else {
			read_count = num;  /* Legacy: read only what we need (skip samples) */
		}
		
		/* Temporary buffer for polyphase input */
		sample_t *poly_in_i = NULL, *poly_in_q = NULL;
		if (use_rx_polyphase && sdr->rx_poly_i && sdr->rx_poly_q) {
			poly_in_i = sdr->rx_poly_i;
			poly_in_q = sdr->rx_poly_q;
		}
		
		for (s = 0, ss = 0; s < read_count; s++) {
			if (use_rx_polyphase && poly_in_i && poly_in_q) {
				/* Store de-interleaved for polyphase processing */
				poly_in_i[s] = sdr->thread_read.buffer[out];
				poly_in_q[s] = sdr->thread_read.buffer[out + 1];
				out = (out + 2) % sdr->thread_read.buffer_size;
			} else if (use_channelizer) {
				/* Channelizer: contiguous read into buff */
				buff[ss++] = sdr->thread_read.buffer[out];
				buff[ss++] = sdr->thread_read.buffer[out + 1];
				out = (out + 2) % sdr->thread_read.buffer_size;
			} else {
				/* Legacy: read with skipping */
				buff[ss++] = sdr->thread_read.buffer[out];
				buff[ss++] = sdr->thread_read.buffer[out + 1];
				out = (out + 2 * rx_os) % sdr->thread_read.buffer_size;
			}
		}
		/* Only update out if we actually read something */
		if (read_count > 0) {
			sdr->thread_read.out = out;
		}
		
		/* Apply polyphase resampling if needed */
		if (use_rx_polyphase && poly_in_i && poly_in_q) {
			/* DEBUG: measure IQ levels BEFORE polyphase resampling */
			{
				static int pre_dbg_count = 0;
				static double pre_pwr_sum = 0.0;
				static double pre_peak = 0.0;
				static int pre_samples = 0;
				for (s = 0; s < read_count; s++) {
					double pwr = (double)poly_in_i[s] * poly_in_i[s]
					           + (double)poly_in_q[s] * poly_in_q[s];
					pre_pwr_sum += pwr;
					double mag = sqrt(pwr);
					if (mag > pre_peak) pre_peak = mag;
					pre_samples++;
				}
				pre_dbg_count++;
				if (pre_dbg_count >= 333) {
					double rms = sqrt(pre_pwr_sum / pre_samples);
					LOGP(DSDR, LOGL_NOTICE,
					     "IQ_BEFORE_POLY: samples=%d rms=%.6f(%.1fdBFS) peak=%.6f(%.1fdBFS) pwr=%.1fdBFS\n",
					     pre_samples, rms, 20.0*log10(rms+1e-20),
					     pre_peak, 20.0*log10(pre_peak+1e-20),
					     10.0*log10(pre_pwr_sum/pre_samples+1e-20));
					pre_pwr_sum = 0.0;
					pre_peak = 0.0;
					pre_samples = 0;
					pre_dbg_count = 0;
				}
			}

			if (!rx_polyphase_logged) {
				LOGP(DSDR, LOGL_NOTICE, "RX POLYPHASE DOWNSAMPLE: SDR %d Hz -> DSP %d Hz (ratio=%.6f)\n",
				     sdr_config->rx_samplerate, sdr->samplerate,
				     (double)sdr_config->rx_samplerate / (double)sdr->samplerate);
				rx_polyphase_logged = 1;
			}
			
			/* Resample I and Q through polyphase filter
			 * Use second half of rx_poly buffers for output */
			int max_in = polyphase_input_num(&sdr->rx_polyphase, sdr->buffer_size) + 16;
			sample_t *out_i = sdr->rx_poly_i + max_in;
			sample_t *out_q = sdr->rx_poly_q + max_in;
			
			int out_count = polyphase_resample_iq(&sdr->rx_polyphase,
			                                      poly_in_i, poly_in_q, read_count,
			                                      out_i, out_q, num);
			
			/* Diagnostics: accumulate per-second stats */
			{
				static int rd_calls = 0, rd_zero = 0;
				static int rd_total_sdr_in = 0, rd_total_dsp_out = 0;
				static int rd_total_fill = 0;
				static double rd_timer = 0;
				rd_calls++;
				if (read_count == 0) rd_zero++;
				rd_total_sdr_in += read_count;
				rd_total_dsp_out += out_count;
				rd_total_fill += available_sdr_samples;
				if (rd_timer == 0.0)
					rd_timer = get_time();
				double now = get_time();
				if (now - rd_timer >= 1.0) {
					LOGP(DSDR, LOGL_NOTICE, "RX POLY 1s: calls=%d zero=%d | sdr_in=%d dsp_out=%d (expect %d) | avg_fill=%.0f\n",
					     rd_calls, rd_zero,
					     rd_total_sdr_in, rd_total_dsp_out, sdr->samplerate,
					     (double)rd_total_fill / rd_calls);
					rd_calls = 0;
					rd_zero = 0;
					rd_total_sdr_in = 0;
					rd_total_dsp_out = 0;
					rd_total_fill = 0;
					rd_timer = now;
				}
			}
			
			/* Interleave I/Q into output buffer */
			for (s = 0; s < out_count; s++) {
				buff[s * 2] = out_i[s];
				buff[s * 2 + 1] = out_q[s];
			}
			count = out_count;

			/* DEBUG: measure IQ levels AFTER polyphase resampling */
			{
				static int post_dbg_count = 0;
				static double post_pwr_sum = 0.0;
				static double post_peak = 0.0;
				static int post_samples = 0;
				for (s = 0; s < out_count; s++) {
					double pwr = (double)buff[s*2] * buff[s*2]
					           + (double)buff[s*2+1] * buff[s*2+1];
					post_pwr_sum += pwr;
					double mag = sqrt(pwr);
					if (mag > post_peak) post_peak = mag;
					post_samples++;
				}
				post_dbg_count++;
				if (post_dbg_count >= 333) {
					double rms = sqrt(post_pwr_sum / post_samples);
					LOGP(DSDR, LOGL_NOTICE,
					     "IQ_AFTER_POLY: samples=%d rms=%.6f(%.1fdBFS) peak=%.6f(%.1fdBFS) pwr=%.1fdBFS\n",
					     post_samples, rms, 20.0*log10(rms+1e-20),
					     post_peak, 20.0*log10(post_peak+1e-20),
					     10.0*log10(post_pwr_sum/post_samples+1e-20));
					post_pwr_sum = 0.0;
					post_peak = 0.0;
					post_samples = 0;
					post_dbg_count = 0;
				}
			}
		} else {
			count = num; /* Output Sample Count */
		}
	} else {
		/* Non-threaded path: read directly from SDR */
		int direct_read_count = input_num;
		
		/* For polyphase, we need a temporary buffer for SDR samples */
		float *sdr_buff = buff;
		if (use_rx_polyphase) {
			/* Use pre-allocated buffer for SDR rate samples */
			sdr_buff = sdr->rx_poly_sdr_buff;
		}
		
#ifdef HAVE_UHD
		if (rx_driver == 1)
			count = uhd_receive(sdr_buff, direct_read_count);
#endif
#ifdef HAVE_SOAPY
		if (rx_driver == 2)
			count = soapy_receive(sdr_buff, direct_read_count);
#endif
		if (bias_count >= 0)
			sdr_bias(sdr_buff, count);
		
		if (count <= 0)
			return count;
		
		/* Apply resampling based on mode */
		if (use_channelizer) {
			/* Channelizer handles decimation internally */
			count /= rx_os;
		} else if (use_rx_polyphase) {
			/* Polyphase downsample */
			if (!rx_polyphase_logged) {
				LOGP(DSDR, LOGL_NOTICE, "RX POLYPHASE DOWNSAMPLE (direct): SDR %d Hz -> DSP %d Hz\n",
				     sdr_config->rx_samplerate, sdr->samplerate);
				rx_polyphase_logged = 1;
			}
			
			/* Use pre-allocated buffers for de-interleaved I/Q
			 * rx_poly_i/q layout: [input: max_in samples][output: buffer_size samples] */
			sample_t *in_i = sdr->rx_poly_i;
			sample_t *in_q = sdr->rx_poly_q;
			sample_t *out_i = sdr->rx_poly_i + sdr->rx_poly_max_in;
			sample_t *out_q = sdr->rx_poly_q + sdr->rx_poly_max_in;
			
			/* De-interleave SDR samples */
			for (s = 0; s < count; s++) {
				in_i[s] = sdr_buff[s * 2];
				in_q[s] = sdr_buff[s * 2 + 1];
			}
			
			/* Resample */
			int out_count = polyphase_resample_iq(&sdr->rx_polyphase,
			                                      in_i, in_q, count,
			                                      out_i, out_q, num);
			
			/* Interleave into output buffer */
			for (s = 0; s < out_count; s++) {
				buff[s * 2] = out_i[s];
				buff[s * 2 + 1] = out_q[s];
			}
			count = out_count;
		}
		/* else: legacy integer path - samples already in buff at correct rate */
	}

	if (sdr_rx_overflow) {
		LOGP(DSDR, LOGL_ERROR, "SDR RX overflow!\n");
		sdr_rx_overflow = 0;
	}

	/* IQ level monitoring via status objects (uses decimated data for RMS) */
	{
		int iq_total = use_channelizer ? count * rx_os : count;
		sdr_status_rx_accumulate(buff, iq_total);
		sdr_status_snapshot(get_time());
		/* Update spectral measurements at higher rate (10Hz) for signal meter */
		sdr_spectral_snapshot(get_time());
	}

	if (sdr->wave_rx_rec.fp) {
		// If channelizer, buff is high-speed. We probably want to record decimated output?
		// Existing logic records 'buff'. If using channelizer, 'buff' is chan_in_buff (wideband).
		// Maybe useful to record wideband? Or skip?
		// Let's keep recording whatever 'buff' has (wideband in channelizer case).
		// But wave_write expects 'count' samples. 'buff' has count * oversample.
		// Issue: recording wideband with 'count' length logic is wrong.
		// Fixing this requires more logic. I'll disable wave rec for channelizer or record proper length.
		// For now, I'll stick to original flow but be aware of rate mismatch.
		
		int rec_count = use_channelizer ? count * rx_os : count;
		sample_t *spl_list[2] = { sdr->wavespl0, sdr->wavespl1 };
		// wavespl0 size is buffer_size. rec_count might be buffer_size * oversample. Overflow!
		// We cannot record wideband easily.
		// I will SKIP wave recording in channelizer mode for now to prevent crash.
		if (!use_channelizer) {
			for (s = 0, ss = 0; s < rec_count; s++) {
				spl_list[0][s] = buff[ss++];
				spl_list[1][s] = buff[ss++];
			}
			wave_write(&sdr->wave_rx_rec, spl_list, rec_count);
		}
	}
	if (sdr->wave_rx_play.fp) {
		// Same issue as recording. Skip playback for channelizer mode.
		if (!use_channelizer) {
			sample_t *spl_list[2] = { sdr->wavespl0, sdr->wavespl1 };
			wave_read(&sdr->wave_rx_play, spl_list, count);
			for (s = 0, ss = 0; s < count; s++) {
				buff[ss++] = spl_list[0][s];
				buff[ss++] = spl_list[1][s];
			}
		}
	}

	if (use_channelizer) {
		// Channelizer Processing
		int final_count = 0;
		
		/* Sample rate tracking for channelizer */
		static long long ch_in_total = 0, ch_out_total = 0;
		static double ch_rate_timer = 0;
		
		if (channels == 0) {
			/* Radio mode: decimate IQ data using the radio's channelizer.
			 * The radio does its own FM demodulation, it just needs decimated IQ.
			 * Use the halfband channelizer for proper anti-alias filtering. */
			int ch_in = count * rx_os;
			
			/* DEBUG: IQ levels before channelizer */
			static double ch_in_peak = 0, ch_in_pwr = 0;
			static int ch_in_samples = 0;
			static int ch_dbg_cnt = 0;
			for (s = 0; s < ch_in; s++) {
				double pwr = buff[s*2] * buff[s*2] + buff[s*2+1] * buff[s*2+1];
				ch_in_pwr += pwr;
				double mag = sqrt(pwr);
				if (mag > ch_in_peak) ch_in_peak = mag;
				ch_in_samples++;
			}
			
			/* Use the radio channelizer (initialized in sdr_open when channels=0 && use_channelizer) */
			if (sdr->radio_channelizer_init) {
				int ch_out = channelizer_process(&sdr->radio_channelizer, buff, ch_in, 
				                                  sdr->radio_ch_I, sdr->radio_ch_Q);
				
				/* DEBUG: IQ levels after channelizer */
				static double ch_out_peak = 0, ch_out_pwr = 0;
				static int ch_out_samples = 0;
				for (s = 0; s < ch_out; s++) {
					double pwr = sdr->radio_ch_I[s] * sdr->radio_ch_I[s] + 
					             sdr->radio_ch_Q[s] * sdr->radio_ch_Q[s];
					ch_out_pwr += pwr;
					double mag = sqrt(pwr);
					if (mag > ch_out_peak) ch_out_peak = mag;
					ch_out_samples++;
				}
				
				ch_dbg_cnt++;
				if (ch_dbg_cnt >= 100) {
					double in_rms = sqrt(ch_in_pwr / ch_in_samples);
					double out_rms = sqrt(ch_out_pwr / ch_out_samples);
					LOGP(DSDR, LOGL_NOTICE, "CHAN IQ: in_peak=%.4f in_rms=%.4f | out_peak=%.4f out_rms=%.4f | ch_in=%d ch_out=%d\n",
					     ch_in_peak, in_rms, ch_out_peak, out_rms, ch_in, ch_out);
					ch_in_peak = ch_in_pwr = ch_in_samples = 0;
					ch_out_peak = ch_out_pwr = ch_out_samples = 0;
					ch_dbg_cnt = 0;
				}
				
				/* Interleave I/Q back to output buffer */
				float *out = (float *)samples;
				for (s = 0; s < ch_out; s++) {
					out[s * 2] = (float)sdr->radio_ch_I[s];
					out[s * 2 + 1] = (float)sdr->radio_ch_Q[s];
				}
				
				ch_in_total += ch_in;
				ch_out_total += ch_out;
				final_count = ch_out;
			} else {
				/* Fallback: simple decimation (not ideal but works) */
				int ch_out = count;
				float *out = (float *)samples;
				for (s = 0; s < ch_out; s++) {
					out[s * 2] = buff[s * rx_os * 2];
					out[s * 2 + 1] = buff[s * rx_os * 2 + 1];
				}
				ch_in_total += ch_in;
				ch_out_total += ch_out;
				final_count = ch_out;
			}
			
			/* Log channelizer rate stats every second */
			if (ch_rate_timer == 0.0) {
				ch_rate_timer = get_time();
			} else {
				double now = get_time();
				if (now - ch_rate_timer >= 1.0) {
					double in_rate = ch_in_total / (now - ch_rate_timer);
					double out_rate = ch_out_total / (now - ch_rate_timer);
					LOGP(DSDR, LOGL_NOTICE, "CHANNELIZER RATE (radio): in=%.0f Hz out=%.0f Hz ratio=%.4f (expect %d)\n",
					     in_rate, out_rate, in_rate / out_rate, rx_os);
					ch_in_total = 0;
					ch_out_total = 0;
					ch_rate_timer = now;
				}
			}
			
			return final_count;
		}
		
		for (c = 0; c < channels; c++) {
			// Process High-Rate 'buff' -> Decimated 'ch_I'/'ch_Q'
			// buff is sdr->chan_in_buff (float*).
			// count is Output samples. Input samples = count * oversample.
			// channelizer_process takes Input Count.
			int ch_in = count * rx_os;
			int ch_out = channelizer_process(&sdr->chan[c].channelizer, (float*)buff, ch_in, sdr->chan[c].ch_I, sdr->chan[c].ch_Q);
			
			ch_in_total += ch_in;
			ch_out_total += ch_out;
			
			// Use actual channelizer output count
			if (ch_out != count) {
				static int ch_mismatch_logged = 0;
				if (ch_mismatch_logged < 10) {
					LOGP(DSDR, LOGL_DEBUG, "Channelizer output mismatch: expected %d, got %d (in=%d, rx_os=%d)\n", 
					     count, ch_out, ch_in, rx_os);
					ch_mismatch_logged++;
				}
			}
			int ch_count = (ch_out < count) ? ch_out : count;
			final_count = ch_count;  /* Track for return */
			
			// Interleave doubles to floats for FM demod
			// FM Demod expects interleaved float baseband.
			for (s = 0; s < ch_count; s++) {
				sdr->chan[c].ch_iq_float[2 * s] = (float)sdr->chan[c].ch_I[s];
				sdr->chan[c].ch_iq_float[2 * s + 1] = (float)sdr->chan[c].ch_Q[s];
			}
			
			// Demodulate Decimated, Interleaved Buffer
			// This logic seamlessly supports both AM (Narrow/Wide) and FM (Narrow/Wide)
			// by directing the channelized component to the appropriate demodulator.
			if (sdr->chan[c].am)
				am_demodulate_complex(&sdr->chan[c].am_demod, samples[c], ch_count, sdr->chan[c].ch_iq_float, sdr->modbuff_I, sdr->modbuff_Q, sdr->modbuff_carrier);
			else
				fm_demodulate_complex(&sdr->chan[c].fm_demod, samples[c], ch_count, sdr->chan[c].ch_iq_float, sdr->modbuff_I, sdr->modbuff_Q);
				
			// Measurements (copied from legacy)
			sender_t *sender = get_sender_by_empfangsfrequenz(sdr->chan[c].rx_frequency);
			if (!sender || !ch_count) continue;
			
			double min, max, avg = 0.0;
			for (s = 0; s < ch_count; s++) {
				avg += sdr->modbuff_I[s] * sdr->modbuff_I[s] + sdr->modbuff_Q[s] * sdr->modbuff_Q[s];
			}
			avg = sqrt(avg /(double)ch_count);
			avg = log10(avg) * 20;
			display_measurements_update(sdr->chan[c].dmp_rf_level, avg, 0.0);
			if (rf_level_db) rf_level_db[c] = avg;
			
			if (!sdr->chan[c].am) {
				min = max = avg = 0.0;
				for (s = 0; s < ch_count; s++) {
					avg += samples[c][s];
					if (s == 0 || samples[c][s] > max) max = samples[c][s];
					if (s == 0 || samples[c][s] < min) min = samples[c][s];
				}
				avg /= (double)ch_count;
				display_measurements_update(sdr->chan[c].dmp_freq_offset, avg, 0.0);
				display_measurements_update(sdr->chan[c].dmp_deviation, max - min, 0.0);

				/* Update per-channel RX status */
				sdr_status_rx_chan_update(c, rf_level_db ? rf_level_db[c] : avg,
							 avg, max - min, sdr->chan[c].rx_frequency);
			}
		}
		
		/* Log channelizer rate stats every second */
		if (ch_rate_timer == 0.0) {
			ch_rate_timer = get_time();
		} else {
			double now = get_time();
			if (now - ch_rate_timer >= 1.0) {
				double in_rate = ch_in_total / (now - ch_rate_timer);
				double out_rate = ch_out_total / (now - ch_rate_timer);
				LOGP(DSDR, LOGL_NOTICE, "CHANNELIZER RATE: in=%.0f Hz out=%.0f Hz ratio=%.4f (expect %d)\n",
				     in_rate, out_rate, in_rate / out_rate, rx_os);
				ch_in_total = 0;
				ch_out_total = 0;
				ch_rate_timer = now;
			}
		}
		
		return final_count;  /* Return actual samples processed */
	}

	display_iq(buff, count);
	display_spectrum(buff, count);

	if (channels) {
		for (c = 0; c < channels; c++) {
			if (rf_level_db)
				rf_level_db[c] = NAN;
			if (sdr->chan[c].am)
				am_demodulate_complex(&sdr->chan[c].am_demod, samples[c], count, buff, sdr->modbuff_I, sdr->modbuff_Q, sdr->modbuff_carrier);
			else
				fm_demodulate_complex(&sdr->chan[c].fm_demod, samples[c], count, buff, sdr->modbuff_I, sdr->modbuff_Q);
			sender_t *sender = get_sender_by_empfangsfrequenz(sdr->chan[c].rx_frequency);
			if (!sender || !count)
				continue;
			double min, max, avg;
			avg = 0.0;
			for (s = 0; s < count; s++) {
				/* average the square length of vector */
				avg += sdr->modbuff_I[s] * sdr->modbuff_I[s] + sdr->modbuff_Q[s] * sdr->modbuff_Q[s];
			}
			avg = sqrt(avg /(double)count); /* RMS */
			avg = log10(avg) * 20;
			display_measurements_update(sdr->chan[c].dmp_rf_level, avg, 0.0);
			if (rf_level_db)
				rf_level_db[c] = avg;
			if (!sdr->chan[c].am) {
				min = 0.0;
				max = 0.0;
				avg = 0.0;
				for (s = 0; s < count; s++) {
					avg += samples[c][s];
					if (s == 0 || samples[c][s] > max)
						max = samples[c][s];
					if (s == 0 || samples[c][s] < min)
						min = samples[c][s];
				}
				avg /= (double)count;
				display_measurements_update(sdr->chan[c].dmp_freq_offset, avg / 1000.0, 0.0);
				/* use half min and max, because we want the deviation above/below (+-) center frequency. */
				display_measurements_update(sdr->chan[c].dmp_deviation, min / 2.0 / 1000.0, max / 2.0 / 1000.0);

				/* Update per-channel RX status */
				sdr_status_rx_chan_update(c, rf_level_db ? rf_level_db[c] : avg,
							 avg, max - min, sdr->chan[c].rx_frequency);
				
				/* DEBUG: FM-DEMOD level diagnostics (disabled - uncomment to enable)
				 * Logs: FM-DEMOD ch%d: avg/peak Hz deviation and normalized dB levels
				 * Measures FM demodulator output per channel every second */
#if 0
				static int fm_diag_count[16] = {0};
				static double fm_diag_sum[16] = {0};
				static double fm_diag_max[16] = {0};
				
				if (c < 16) {
					for (s = 0; s < count; s++) {
						double abs_dev = fabs(samples[c][s]);
						fm_diag_sum[c] += abs_dev;
						if (abs_dev > fm_diag_max[c]) fm_diag_max[c] = abs_dev;
					}
					fm_diag_count[c] += count;
					
					if (fm_diag_count[c] >= sdr->samplerate) {
						double fm_avg_hz = fm_diag_sum[c] / fm_diag_count[c];
						double fm_max_hz = fm_diag_max[c];
						LOGP(DSDR, LOGL_DEBUG, "FM-DEMOD ch%d: avg=%.0fHz pk=%.0fHz (norm: avg=%.1fdB pk=%.1fdB)\n",
							c, fm_avg_hz, fm_max_hz,
							20.0 * log10(fm_avg_hz / 2900.0 + 0.001),
							20.0 * log10(fm_max_hz / 2900.0 + 0.001));
						fm_diag_count[c] = 0;
						fm_diag_sum[c] = 0;
						fm_diag_max[c] = 0;
					}
				}
#endif
			}
		}
	}

	return count;
}

/* how much do we need to send (in audio sample duration) to get the target delay (buffer size) */
int sdr_get_tosend(void *inst, int buffer_size)
{
	sdr_t *sdr = (sdr_t *)inst;
	int count = 0;

	/* Safety check: no TX in rx_only mode */
	if (sdr_config && sdr_config->rx_only) {
		LOGP(DSDR, LOGL_ERROR, "sdr_get_tosend() called in RX-only mode - this is a bug!\n");
		return 0;
	}

	int tx_os = sdr_config->split_mode ? sdr->tx_oversample : sdr->oversample;

#ifdef HAVE_UHD
	if (tx_driver == 1)
		count = uhd_get_tosend(buffer_size * tx_os);
#endif
#ifdef HAVE_SOAPY
	if (tx_driver == 2)
		count = soapy_get_tosend(buffer_size * tx_os);
#endif
#ifdef HAVE_RPITX
	if (tx_driver == 3)
		count = rpitx_get_tosend(buffer_size * tx_os);
#endif
	if (count < 0)
		return count;
	/* rounding down, so we never overfill */
	count /= tx_os;

	if (sdr->threads) {
		/* subtract what we have in write buffer, because this is not jet sent to the SDR */
		int fill;

		fill = (sdr->thread_write.in - sdr->thread_write.out + sdr->thread_write.buffer_size) % sdr->thread_write.buffer_size;
		count -= fill / 2;
		if (count < 0)
			count = 0;
	}

	return count;
}



/* Query SDR capabilities from configured device(s) */
int sdr_query_caps(sdr_caps_t *caps)
{
	int rc = 0;

	if (!caps)
		return -1;

	memset(caps, 0, sizeof(*caps));

	if (!sdr_config) {
		LOGP(DSDR, LOGL_ERROR, "SDR not configured, cannot query caps\n");
		return -1;
	}

	/* Copy upconverter offsets from config */
	caps->rx_upconverter = sdr_config->rx_upconverter;
	caps->tx_upconverter = sdr_config->tx_upconverter;

	/* Determine mode flags */
	caps->is_split = sdr_config->split_mode;
	caps->has_rx = !sdr_config->tx_only;
	caps->has_tx = !sdr_config->rx_only;

	/* Query RX device capabilities */
	if (caps->has_rx) {
		const char *rx_args = sdr_config->split_mode ?
			sdr_config->rx_device_args : sdr_config->device_args;
		int rx_channel = sdr_config->split_mode && sdr_config->rx_channel_given ?
			sdr_config->rx_channel : sdr_config->channel;
		int rx_is_uhd = sdr_config->split_mode ? sdr_config->rx_uhd : sdr_config->uhd;
		int rx_is_soapy = sdr_config->split_mode ? sdr_config->rx_soapy : sdr_config->soapy;

#ifdef HAVE_UHD
		if (rx_is_uhd) {
			if (uhd_query_freq_range(rx_args, 0, rx_channel,
						 &caps->rx_freq_min, &caps->rx_freq_max) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query RX freq range from UHD\n");
			}
			if (uhd_query_gain_info(rx_args, 0, rx_channel,
						&caps->rx_gain_min, &caps->rx_gain_max,
						caps->rx_gain_names, sizeof(caps->rx_gain_names)) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query RX gain info from UHD\n");
			}
		}
#endif
#ifdef HAVE_SOAPY
		if (rx_is_soapy) {
			if (soapy_query_freq_range(rx_args, SOAPY_SDR_RX, rx_channel,
						   &caps->rx_freq_min, &caps->rx_freq_max) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query RX freq range from SoapySDR\n");
			}
			if (soapy_query_gain_info(rx_args, SOAPY_SDR_RX, rx_channel,
						  &caps->rx_gain_min, &caps->rx_gain_max,
						  caps->rx_gain_names, sizeof(caps->rx_gain_names)) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query RX gain info from SoapySDR\n");
			}
		}
#endif
	}

	/* Query TX device capabilities */
	if (caps->has_tx) {
		const char *tx_args = sdr_config->split_mode ?
			sdr_config->tx_device_args : sdr_config->device_args;
		int tx_channel = sdr_config->split_mode && sdr_config->tx_channel_given ?
			sdr_config->tx_channel : sdr_config->channel;
		int tx_is_uhd = sdr_config->split_mode ? sdr_config->tx_uhd : sdr_config->uhd;
		int tx_is_soapy = sdr_config->split_mode ? sdr_config->tx_soapy : sdr_config->soapy;

#ifdef HAVE_UHD
		if (tx_is_uhd) {
			if (uhd_query_freq_range(tx_args, 1, tx_channel,
						 &caps->tx_freq_min, &caps->tx_freq_max) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query TX freq range from UHD\n");
			}
			if (uhd_query_gain_info(tx_args, 1, tx_channel,
						&caps->tx_gain_min, &caps->tx_gain_max,
						caps->tx_gain_names, sizeof(caps->tx_gain_names)) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query TX gain info from UHD\n");
			}
		}
#endif
#ifdef HAVE_SOAPY
		if (tx_is_soapy) {
			if (soapy_query_freq_range(tx_args, SOAPY_SDR_TX, tx_channel,
						   &caps->tx_freq_min, &caps->tx_freq_max) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query TX freq range from SoapySDR\n");
			}
			if (soapy_query_gain_info(tx_args, SOAPY_SDR_TX, tx_channel,
						  &caps->tx_gain_min, &caps->tx_gain_max,
						  caps->tx_gain_names, sizeof(caps->tx_gain_names)) < 0) {
				LOGP(DSDR, LOGL_NOTICE, "Could not query TX gain info from SoapySDR\n");
			}
		}
#endif
	}

	LOGP(DSDR, LOGL_INFO, "SDR Capabilities:\n");
	LOGP(DSDR, LOGL_INFO, "  RX: %.0f - %.0f MHz, gain %.1f - %.1f dB, upconv %.0f Hz\n",
	     caps->rx_freq_min / 1e6, caps->rx_freq_max / 1e6,
	     caps->rx_gain_min, caps->rx_gain_max, caps->rx_upconverter);
	LOGP(DSDR, LOGL_INFO, "  TX: %.0f - %.0f MHz, gain %.1f - %.1f dB, upconv %.0f Hz\n",
	     caps->tx_freq_min / 1e6, caps->tx_freq_max / 1e6,
	     caps->tx_gain_min, caps->tx_gain_max, caps->tx_upconverter);
	if (caps->rx_gain_names[0])
		LOGP(DSDR, LOGL_INFO, "  RX gains: %s\n", caps->rx_gain_names);
	if (caps->tx_gain_names[0])
		LOGP(DSDR, LOGL_INFO, "  TX gains: %s\n", caps->tx_gain_names);

	return rc;
}
