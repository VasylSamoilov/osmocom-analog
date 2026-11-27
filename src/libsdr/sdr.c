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
#include "../libchannelizer/pfb.h"
#include "sdr_engine.h"
#include "sdr_lo.h"
#include "sdr_backend.h"

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
static double tx_band_shift, rx_band_shift;	/* the band (channels = 0) above the device's centre: retune keeps it */
static int rx_driver;  /* 0=none, 1=uhd, 2=soapy */

/* rpitx sends the phase of the IQ signal on one carrier (see rpitx.c):
 * no DC offset to avoid, at most two carriers, the second one weaker. */
static int sdr_tx_is_rpitx(void)
{
#ifdef HAVE_RPITX
	return sdr_config->rpitx || sdr_config->tx_rpitx;
#else
	return 0;
#endif
}

/* Forward declaration for timing function used by threads */
static double get_time(void);

typedef struct sdr_thread {
	int use;
	volatile int running, exit;	/* flags to control exit of threads */
	int buffer_size;
	volatile float *buffer;		/* ring of SDR rate IQ (interleaved float) */
	float *buffer2;			/* chunk between the ring and the device */
	volatile int in, out;		/* in and out pointers (atomic, so no locking required) */
	int max_fill;			/* measure maximum buffer fill */
	double max_fill_timer;		/* timer to display/reset maximum fill */
} sdr_thread_t;

/* Each channel is converted between the SDR rate and the DSP rate by its own
 * channelizer (RX) and synthesizer (TX), libchannelizer: NCO at the SDR rate,
 * halfband stages, rational resampler. The modulator and demodulator work at
 * 0 Hz at the DSP rate. See docs/CHANNELIZER.md. */
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
	int		rx_init, tx_init;
	channelizer_t	rx_ch;		/* SDR rate -> DSP rate, channel at 0 Hz */
	channelizer_t	tx_ch;		/* DSP rate -> SDR rate, 0 Hz to the channel */
	double		tx_offset, rx_offset;	/* from the SDR centre (tx: engine mode) */
	double		rx_trim;	/* RX frequency trim of the demodulator (Hz), set by the decoder */
	double		rx_bandwidth_req;	/* RX channel filter asked by the decoder (Hz), 0 = not asked (Carson) */
	double		rx_bandwidth;	/* RX channel filter set (Hz), 0 = never changed */
} sdr_chan_t;

typedef struct sdr {
	int		threads;	/* use threads */
	int		tx_rate;	/* SDR sample rate of TX */
	int		rx_rate;	/* SDR sample rate of RX */
	sdr_thread_t	thread_read,
			thread_write;
	sdr_chan_t	*chan;		/* settings for all channels (+ paging channel) */
	int		paging_channel;	/* if set, points to paging channel */
	int		channels;	/* number of frequencies */
	double		amplitude;	/* amplitude of each carrier */
	int		samplerate;	/* sample rate of audio data (DSP rate) */
	int		buffer_size;	/* buffer in audio samples */
	double		interval;	/* how often to process the loop */
	wave_rec_t	wave_rx_rec;	/* IQ recording at the SDR rate */
	wave_rec_t	wave_tx_rec;
	wave_play_t	wave_rx_play;
	wave_play_t	wave_tx_play;
	float		*modbuff;	/* DSP rate IQ of one channel (interleaved float) */
	sample_t	*modbuff_I;
	sample_t	*modbuff_Q;
	sample_t	*modbuff_carrier;
	sample_t	*ch_i, *ch_q;	/* DSP rate IQ of one channel (de-interleaved) */
	uint8_t		*zero_power;	/* all 0: a channel's carrier off (paging channel switched) */
	sample_t	*zero_spl;
	int		rx_max_in;	/* most SDR samples for buffer_size DSP samples */
	float		*rx_lin;	/* SDR rate IQ taken from the ring for one sdr_read() */
	int		tx_max_out;	/* most SDR samples from buffer_size DSP samples */
	float		*tx_sum;	/* SDR rate IQ of all channels for one sdr_write() */
	sample_t	*wavespl0;	/* sample buffer for wave recording / playback (SDR rate) */
	sample_t	*wavespl1;
	/* the LO offset to the driver and the band's shift off the DC, per
	 * direction (sdr_lo.c) */
	sdr_lo_plan_t	lo_tx, lo_rx;
	/* channels = 0 (osmoradio, tv, dcf77): the whole usable band, off the
	 * DC by lo_*.band_shift */
	int		band_rx_init, band_tx_init;
	channelizer_t	band_rx, band_tx;
	int		band_rx_pass;	/* band_rx has nothing to do (SDR rate = DSP rate, no shift): the ring as it is */
	/* engine mode: filter banks shared by all channels, run by the device
	 * threads; the program's channels are one group (docs/SDR-BACKEND.md) */
	int		use_engine;
	sdr_engine_t	engine;
	sdr_group_t	*group;
	sample_t	**eng_i, **eng_q;	/* DSP rate samples of every channel (TX: channels + paging) */
	int		eng_n;			/* number of them */
	double		bandwidth;		/* of each channel (Carson) */
	int		eng_chunk;		/* SDR samples per sdr_engine_tx_produce() */
	float		*rx_gap_zero;		/* zeros for lost RX samples (rx_max_in) */
	int		eng_advance;		/* TX: SDR samples the device is kept ahead (time stamps) */
	int		eng_ring_target;	/* TX: DSP samples kept in the group's ring */
	double		eng_report;		/* time of the last missing / dropped report */
	long		eng_missing, eng_dropped;	/* reported so far */
	/* the program side of the group: the engine's own (in-process) or a
	 * backend's view (client: no device, no device threads) */
	sdr_port_t	*port;
	int		use_client;
	int		band_client;	/* a band program (channels = 0) on the backend: one channel, its IQ */
	sdr_client_t	client;
	double		client_check;		/* time the backend was last checked */
} sdr_t;

/* use the filter banks: on request, or (auto) for two or more channels when
 * they fit the rates of every direction used */
static int sdr_engine_wanted(int tx_rate, int rx_rate, int dsp, int nchan)
{
	int fits = (!tx_rate || sdr_engine_bins(tx_rate, dsp, nchan)) && (!rx_rate || sdr_engine_bins(rx_rate, dsp, nchan));

	if (sdr_config->filter_bank == 0)
		return 0;
	if (sdr_config->filter_bank == 1) {
		if (!fits)
			LOGP(DSDR, LOGL_NOTICE, "No filter bank fits SDR rate %d / %d and DSP rate %d (sub-bands must be wider than the DSP rate), using a channelizer per channel.\n", tx_rate, rx_rate, dsp);
		return fits;
	}
	return (nchan >= 2) && fits;
}

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

/* Level of an FM carrier. Equal levels for SDRs. rpitx with two carriers:
 * the first one at 1 / (1 + r), the second one (the second channel or the
 * paging channel) at r / (1 + r). They must not be equal, else the sum of
 * both passes through zero, where its phase jumps. */
static double sdr_tx_amplitude(sdr_t *sdr, int c, int channels)
{
	double r;

	if (!sdr_tx_is_rpitx() || channels + (sdr->paging_channel ? 1 : 0) != 2)
		return sdr->amplitude;
	r = sdr_config->rpitx_ratio;
	return (c == 0) ? 1.0 / (1.0 + r) : r / (1.0 + r);
}

/* An LO offset the driver applies inside the stream puts the LO (its DC,
 * leakage) at centre + offset, which the channel placement (off the
 * centre) does not keep the channels from: say so for a channel on it
 * (SoapySDR applies it or not at open, soapy.c says which). */
static void lo_spur_notice(const char *dir, const sdr_lo_plan_t *p, double centre, int rate, const double *f, int n, double bandwidth)
{
	double spur = centre + p->lo_offset;
	int c;

	if (p->lo_offset == 0.0 || p->lo_outside || fabs(p->lo_offset) >= 0.5 * rate)
		return;
	for (c = 0; c < n; c++)
		if (fabs(f[c] - spur) < bandwidth / 2.0)
			LOGP(DSDR, LOGL_NOTICE, "%s channel %d (%.6f MHz) covers the LO at %.6f MHz (centre + the LO offset %.0f Hz): choose another --sdr-lo-offset, or none for the automatic one.\n",
			     dir, c, f[c] / 1e6, spur / 1e6, p->lo_offset);
}

/* The analog filter is centred on the LO (centre + offset), the band is
 * not: with the LO outside the stream it must reach |offset| + rate / 2
 * on the far side, i.e. a bandwidth of rate + 2 |offset| (B210 cable
 * loopback, 1.92 MS/s, automatic offset: with the bandwidth = rate the band
 * fell by 21 dB towards its far edge, 3 MHz still 2.5 dB, 4 MHz flat within
 * 1.8 dB). A bandwidth given is used as given. */
static double lo_bandwidth(const char *dir, double bw, int given, const sdr_lo_plan_t *p, int rate)
{
	double need = rate + 2.0 * fabs(p->lo_offset);

	if (given || !p->lo_outside || bw >= need)
		return bw;
	LOGP(DSDR, LOGL_INFO, "%s: analog bandwidth %.0f Hz (the rate + 2 x the LO offset: the filter is centred on the LO), not %.0f Hz.\n", dir, need, bw);
	return need;
}

static double unified_bandwidth(sdr_t *sdr, int tx, int rx)
{
	double t = tx ? lo_bandwidth("TX", sdr_config->bandwidth, sdr_config->bandwidth_given, &sdr->lo_tx, sdr->tx_rate) : 0.0;
	double r = rx ? lo_bandwidth("RX", sdr_config->bandwidth, sdr_config->bandwidth_given, &sdr->lo_rx, sdr->rx_rate) : 0.0;

	return (t > r) ? t : r;
}

/* Most samples at the SDR rate 'to' for n samples at the DSP rate 'from'.
 * A channelizer gives at most n * ceil(P/Q) * 2^k (channelizer_tx_max(), RX
 * needs no more), with ceil(P/Q) <= P/Q + 1 and 2^k <= to / from: at most
 * 2 n to / from. */
static int rate_max(int n, int from, int to)
{
	return (int)ceil(2.0 * n * to / from) + 64;
}

/* The device of an SDR backend (sdr_server_mode() before sdr_open() with no
 * channels, the frequencies are the centres): the engine runs in the
 * device threads without channels of its own, the backend registers the
 * programs' groups on it (sdr_server_engine()). */
static int server_nmax;		/* 0: not a backend */

void sdr_server_mode(int nmax)
{
	server_nmax = nmax;
}

sdr_engine_t *sdr_server_engine(void *inst)
{
	sdr_t *sdr = (sdr_t *)inst;

	return (sdr && sdr->use_engine && !sdr->use_client) ? &sdr->engine : NULL;
}

/* --sdr-backend: no device; the channels (by frequency) are registered on
 * the backend that owns the radio, it places them and runs the filter
 * banks; this program exchanges DSP rate samples with it through the
 * shared view, the same as engine mode in-process. The TX level of every
 * channel is the backend's (1 / its most channels, docs/SDR-BACKEND.md). */
static int sdr_open_backend(sdr_t *sdr, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double bandwidth, double max_deviation, double max_modulation, double modulation_index)
{
	double tx_f[SDR_ENGINE_MAX_CHAN], rx_f[SDR_ENGINE_MAX_CHAN];
	int ntx, nrx, nbuf, c, rc, band = !channels;
	char err[256];

	/* a program that takes the band (channels = 0, osmoradio: its own
	 * modulation, IQ at its rate around its frequency) gets one channel at
	 * its frequency; above the backend's DSP rate it is on the chain */
	ntx = tx_frequency ? (band ? 1 : channels + (sdr->paging_channel ? 1 : 0)) : 0;
	nrx = rx_frequency ? (band ? 1 : channels) : 0;
	nbuf = (ntx > nrx) ? ntx : nrx;
	if (nbuf > SDR_ENGINE_MAX_CHAN) {
		LOGP(DSDR, LOGL_ERROR, "%d channels, the SDR backend takes at most %d.\n", nbuf, SDR_ENGINE_MAX_CHAN);
		return -EINVAL;
	}
	for (c = 0; c < ntx; c++)
		tx_f[c] = (band || c < channels) ? tx_frequency[c] : paging_frequency;
	for (c = 0; c < nrx; c++)
		rx_f[c] = rx_frequency[c];
	/* rings as in engine mode: 4 buffers; TX kept at one buffer (the
	 * device's advance is the backend's) */
	rc = sdr_client_open(&sdr->client, sdr_config->backend, "osmocom-analog", samplerate, nrx, rx_f, ntx, tx_f,
			     (int)ceil(4.0 * buffer_size * 1000.0 / samplerate) + 10, buffer_size, err, sizeof(err));
	if (rc < 0) {
		LOGP(DSDR, LOGL_ERROR, "%s\n", err);
		return rc;
	}
	sdr->use_client = 1;
	sdr->use_engine = 1;
	sdr->port = &sdr->client.port;
	sdr->eng_ring_target = sdr->port->tx_target;
	sdr->amplitude = sdr->client.tx_scale;
	LOGP(DSDR, LOGL_INFO, "SDR backend '%s': %d RX / %d TX channels registered (RX centre %.6f MHz, TX centre %.6f MHz), TX level 1/%.0f\n",
	     sdr_config->backend, nrx, ntx, sdr->client.rx_centre / 1e6, sdr->client.tx_centre / 1e6, 1.0 / sdr->client.tx_scale);
	if (band) {
		/* the program's IQ goes through as it is (TX scaled to the
		 * channel's level), no modulators here */
		sdr->band_client = 1;
		if (ntx)
			LOGP(DSDR, LOGL_INFO, "TX: %.6f MHz, offset %+.1f kHz, DSP rate %d, on the backend\n", tx_f[0] / 1e6, (tx_f[0] - sdr->client.tx_centre) / 1e3, samplerate);
		if (nrx)
			LOGP(DSDR, LOGL_INFO, "RX: %.6f MHz, offset %+.1f kHz, DSP rate %d, on the backend\n", rx_f[0] / 1e6, (rx_f[0] - sdr->client.rx_centre) / 1e3, samplerate);
		goto buffers;
	}

	/* modulators and demodulators at 0 Hz at the DSP rate */
	for (c = 0; c < ntx; c++) {
		sdr->chan[c].tx_frequency = tx_f[c];
		if (c < channels && am[c]) {
			double gain = modulation_index / 2.0, bias = 1.0 - gain;

			sdr->chan[c].am = 1;
			rc = am_mod_init(&sdr->chan[c].am_mod, samplerate, 0.0, sdr->amplitude * gain, sdr->amplitude * bias);
		} else
			rc = fm_mod_init(&sdr->chan[c].fm_mod, samplerate, 0.0, sdr->amplitude);
		if (rc < 0)
			return rc;
		sdr->chan[c].tx_offset = tx_f[c] - sdr->client.tx_centre;
		sdr->chan[c].tx_init = 1;
		if (fabs(sdr->chan[c].tx_offset) < bandwidth / 2.0)
			LOGP(DSDR, LOGL_NOTICE, "TX %s %d (%.6f MHz) covers the backend's TX centre (LO leakage): choose the backend's centre off the channels.\n", (c < channels) ? "channel" : "paging channel", c, tx_f[c] / 1e6);
		LOGP(DSDR, LOGL_INFO, "TX %s %d: %.6f MHz, offset %+.1f kHz, on the backend\n", (c < channels) ? "channel" : "paging channel", c, tx_f[c] / 1e6, sdr->chan[c].tx_offset / 1e3);
	}
	for (c = 0; c < nrx; c++) {
		sender_t *sender;

		sdr->chan[c].rx_frequency = rx_f[c];
		sdr->chan[c].am = am[c];
		if (am[c])
			rc = am_demod_init(&sdr->chan[c].am_demod, samplerate, 0.0, bandwidth / 2.0, 1.0 / modulation_index);
		else
			rc = fm_demod_init(&sdr->chan[c].fm_demod, samplerate, 0.0, bandwidth);
		if (rc < 0)
			return rc;
		sdr->chan[c].rx_offset = rx_f[c] - sdr->client.rx_centre;
		sdr->chan[c].rx_init = 1;
		LOGP(DSDR, LOGL_INFO, "RX channel %d: %.6f MHz, offset %+.1f kHz, on the backend\n", c, rx_f[c] / 1e6, sdr->chan[c].rx_offset / 1e3);
		/* the backend's centre is its operator's choice: on it sit the
		 * receiver's DC and LO leakage */
		if (fabs(sdr->chan[c].rx_offset) < bandwidth / 2.0)
			LOGP(DSDR, LOGL_NOTICE, "RX channel %d (%.6f MHz) covers the backend's RX centre (DC, LO leakage): choose the backend's centre off the channels.\n", c, rx_f[c] / 1e6);
		/* measurements display, as with a device */
		sender = get_sender_by_empfangsfrequenz(rx_f[c]);
		if (!sender)
			continue;
		sdr->chan[c].dmp_rf_level = display_measurements_add(&sender->dispmeas, "RF Level", "%.1f dB", DISPLAY_MEAS_AVG, DISPLAY_MEAS_LEFT, -96.0, 0.0, -INFINITY);
		if (!am[c]) {
			sdr->chan[c].dmp_freq_offset = display_measurements_add(&sender->dispmeas, "Freq. Offset", "%+.2f KHz", DISPLAY_MEAS_AVG, DISPLAY_MEAS_CENTER, -max_modulation / 1000.0 * 2.0, max_modulation / 1000.0 * 2.0, 0.0);
			sdr->chan[c].dmp_deviation = display_measurements_add(&sender->dispmeas, "Deviation", "%.2f KHz", DISPLAY_MEAS_PEAK2PEAK, DISPLAY_MEAS_LEFT, 0.0, max_deviation / 1000.0 * 1.5, max_deviation / 1000.0);
		}
	}
buffers:
	sdr->eng_i = calloc(nbuf, sizeof(*sdr->eng_i));
	sdr->eng_q = calloc(nbuf, sizeof(*sdr->eng_q));
	if (!sdr->eng_i || !sdr->eng_q)
		return -ENOMEM;
	sdr->eng_n = nbuf;
	for (c = 0; c < nbuf; c++) {
		sdr->eng_i[c] = calloc(buffer_size, sizeof(sample_t));
		sdr->eng_q[c] = calloc(buffer_size, sizeof(sample_t));
		if (!sdr->eng_i[c] || !sdr->eng_q[c])
			return -ENOMEM;
	}
	sdr_status_init(channels, 1.0);
	return 0;
}

static void *sdr_open_internal(int direction, const char *device, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index)
{
	sdr_t *sdr;
	double bandwidth;
	double tx_center_frequency = 0.0, rx_center_frequency = 0.0;
	int rc;
	int c;

	(void)direction;
	(void)device;

	LOGP(DSDR, LOGL_DEBUG, "Open SDR device\n");

	bandwidth = 2.0 * (max_deviation + max_modulation);
	if (bandwidth)
		LOGP(DSDR, LOGL_INFO, "Require bandwidth of each channel is 2 * (%.1f deviation + %.1f modulation) = %.1f KHz\n", max_deviation / 1e3, max_modulation / 1e3, bandwidth / 1e3);

	sdr = calloc(1, sizeof(*sdr));
	if (!sdr) {
		LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
		goto error;
	}
	sdr->channels = channels;
	sdr->amplitude = 1.0 / (double)channels;
	sdr->samplerate = samplerate;
	sdr->bandwidth = bandwidth;
	sdr->buffer_size = buffer_size;
	sdr->interval = interval;
	sdr->threads = 1; /* always required, because write may block */

	/* SDR rate of each direction (split mode: each device its own) */
	sdr->tx_rate = (sdr_config->split_mode && sdr_config->tx_samplerate) ? sdr_config->tx_samplerate : sdr_config->samplerate;
	sdr->rx_rate = (sdr_config->split_mode && sdr_config->rx_samplerate) ? sdr_config->rx_samplerate : sdr_config->samplerate;
	/* on a backend the SDR rate is its own: here only DSP rate samples */
	if (sdr_config->backend)
		sdr->tx_rate = sdr->rx_rate = samplerate;
	if (sdr->tx_rate < samplerate || sdr->rx_rate < samplerate) {
		LOGP(DSDR, LOGL_ERROR, "The SDR sample rate (TX %d, RX %d) must be at least the DSP sample rate %d (-s).\n", sdr->tx_rate, sdr->rx_rate, samplerate);
		goto error;
	}
	if (sdr->tx_rate % samplerate)
		LOGP(DSDR, LOGL_NOTICE, "TX: SDR rate %d is not an integer multiple of the DSP rate %d: rational resampling (an integer multiple is cheaper).\n", sdr->tx_rate, samplerate);
	if (sdr->rx_rate % samplerate && sdr->rx_rate != sdr->tx_rate)
		LOGP(DSDR, LOGL_NOTICE, "RX: SDR rate %d is not an integer multiple of the DSP rate %d: rational resampling (an integer multiple is cheaper).\n", sdr->rx_rate, samplerate);
	sdr->rx_max_in = rate_max(buffer_size, samplerate, sdr->rx_rate);
	sdr->tx_max_out = rate_max(buffer_size, samplerate, sdr->tx_rate);

	/* the LO offset and the band's shift off the DC, per direction
	 * (sdr_lo.c): UHD applies an LO offset (its DSP moves the signal
	 * back); SoapySDR only knows at open whether it can, so the band is
	 * kept off the DC as without; rpitx has no LO */
	if (!sdr_config->backend) {
		int split = sdr_config->split_mode;
		int tx_given = split ? (sdr_config->tx_lo_offset_given || sdr_config->lo_offset_given) : sdr_config->lo_offset_given;
		int rx_given = split ? (sdr_config->rx_lo_offset_given || sdr_config->lo_offset_given) : sdr_config->lo_offset_given;
		double tx_off = (split && sdr_config->tx_lo_offset_given) ? sdr_config->tx_lo_offset : sdr_config->lo_offset;
		double rx_off = (split && sdr_config->rx_lo_offset_given) ? sdr_config->rx_lo_offset : sdr_config->lo_offset;
		int tx_uhd = split ? sdr_config->tx_uhd : sdr_config->uhd;
		int rx_uhd = split ? sdr_config->rx_uhd : sdr_config->uhd;
		int band = (!channels && !server_nmax) ? samplerate : 0;

		sdr_lo_plan(&sdr->lo_tx, sdr->tx_rate, tx_given, tx_off, tx_uhd, sdr_tx_is_rpitx(), band);
		sdr_lo_plan(&sdr->lo_rx, sdr->rx_rate, rx_given, rx_off, rx_uhd, 0, band);
		if (tx_frequency)
			LOGP(DSDR, LOGL_INFO, "TX: LO offset %.0f Hz%s (%s)%s\n", sdr->lo_tx.lo_offset,
			     sdr_tx_is_rpitx() ? "" : (sdr->lo_tx.lo_outside ? ": the LO outside the band sent" : ": the DC in the band sent, kept off the channels"),
			     sdr_tx_is_rpitx() ? "rpitx: no LO" : (tx_given ? "given" : "automatic"),
			     sdr->lo_tx.band_no_room ? "; no room to move the band off the DC, it stays at the centre" : "");
		if (rx_frequency)
			LOGP(DSDR, LOGL_INFO, "RX: LO offset %.0f Hz%s (%s)%s\n", sdr->lo_rx.lo_offset,
			     sdr->lo_rx.lo_outside ? ": the LO outside the band received" : ": the DC in the band received, kept off the channels",
			     rx_given ? "given" : "automatic",
			     sdr->lo_rx.band_no_room ? "; no room to move the band off the DC, it stays at the centre" : "");
		if (sdr->lo_tx.band_no_room || sdr->lo_rx.band_no_room)
			LOGP(DSDR, LOGL_NOTICE, "The band (%d S/s) is too wide for the SDR rate to move it off the DC: the DC is in the middle of it. A higher SDR rate, or an LO offset the device applies, avoids that.\n", samplerate);
	}

	/* RX ring of SDR rate IQ, 4 x buffer_size of headroom for the main loop jitter */
	if (!sdr_config->tx_only) {
		LOGP(DSDR, LOGL_NOTICE, "Allocating RX thread buffers\n");
		memset(&sdr->thread_read, 0, sizeof(sdr->thread_read));
		sdr->thread_read.buffer_size = sdr->rx_max_in * 2 * 4 + 2;
		sdr->thread_read.buffer = calloc(sdr->thread_read.buffer_size, sizeof(*sdr->thread_read.buffer));
		sdr->thread_read.buffer2 = calloc(sdr->thread_read.buffer_size, sizeof(*sdr->thread_read.buffer2));
		sdr->rx_lin = calloc(sdr->rx_max_in * 2, sizeof(*sdr->rx_lin));
		sdr->rx_gap_zero = calloc(sdr->rx_max_in * 2, sizeof(*sdr->rx_gap_zero));
		if (!sdr->thread_read.buffer || !sdr->thread_read.buffer2 || !sdr->rx_lin || !sdr->rx_gap_zero) {
			LOGP(DSDR, LOGL_ERROR, "No mem!\n");
			goto error;
		}
		sdr->thread_read.in = sdr->thread_read.out = 0;
	} else
		LOGP(DSDR, LOGL_INFO, "Skipping RX buffer allocation (TX-only mode)\n");
	/* TX ring of SDR rate IQ. It holds 3 x buffer_size: sdr_get_tosend()
	 * allows fill + tosend up to 2 x buffer_size (catch-up in time stamp
	 * mode); the third is margin, device paced mode keeps the fill at
	 * buffer_size, so the latency is unchanged. */
	if (!sdr_config->rx_only) {
		LOGP(DSDR, LOGL_NOTICE, "Allocating TX thread buffers\n");
		memset(&sdr->thread_write, 0, sizeof(sdr->thread_write));
		sdr->thread_write.buffer_size = sdr->tx_max_out * 2 * 3 + 2;
		sdr->thread_write.buffer = calloc(sdr->thread_write.buffer_size, sizeof(*sdr->thread_write.buffer));
		sdr->thread_write.buffer2 = calloc(sdr->tx_max_out * 2, sizeof(*sdr->thread_write.buffer2));
		sdr->tx_sum = calloc(sdr->tx_max_out * 2, sizeof(*sdr->tx_sum));
		if (!sdr->thread_write.buffer || !sdr->thread_write.buffer2 || !sdr->tx_sum) {
			LOGP(DSDR, LOGL_ERROR, "No mem!\n");
			goto error;
		}
		sdr->thread_write.in = sdr->thread_write.out = 0;
	} else
		LOGP(DSDR, LOGL_INFO, "Skipping TX buffer allocation (RX-only mode)\n");

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
	/* wave recording / playback of the IQ at the SDR rate */
	{
		int wmax = (sdr->rx_max_in > sdr->tx_max_out) ? sdr->rx_max_in : sdr->tx_max_out;

		sdr->wavespl0 = calloc(wmax, sizeof(*sdr->wavespl0));
		sdr->wavespl1 = calloc(wmax, sizeof(*sdr->wavespl1));
	}
	sdr->ch_i = calloc(sdr->buffer_size, sizeof(*sdr->ch_i));
	sdr->ch_q = calloc(sdr->buffer_size, sizeof(*sdr->ch_q));
	sdr->zero_power = calloc(sdr->buffer_size, sizeof(*sdr->zero_power));
	sdr->zero_spl = calloc(sdr->buffer_size, sizeof(*sdr->zero_spl));
	if (!sdr->wavespl0 || !sdr->wavespl1 || !sdr->ch_i || !sdr->ch_q || !sdr->zero_power || !sdr->zero_spl) {
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

	/* channels on a backend: no device */
	if (sdr_config->backend) {
		if (sdr_open_backend(sdr, tx_frequency, rx_frequency, am, channels, paging_frequency, samplerate, buffer_size, bandwidth, max_deviation, max_modulation, modulation_index) < 0)
			goto error;
		return sdr;
	}

	/* a backend's device: the engine without channels of its own; the
	 * DSP rate given is the highest the programs may use */
	if (server_nmax && !channels) {
		int tx_r = tx_frequency ? sdr->tx_rate : 0, rx_r = rx_frequency ? sdr->rx_rate : 0;
		int max_block = (sdr->rx_max_in > sdr->tx_max_out) ? sdr->rx_max_in : sdr->tx_max_out;

		if (tx_frequency)
			tx_center_frequency = tx_frequency[0];
		if (rx_frequency)
			rx_center_frequency = rx_frequency[0];
		if (sdr_tx_is_rpitx()) {
			LOGP(DSDR, LOGL_ERROR, "SDR backend: rpitx makes its own carrier, it cannot carry the programs' channels.\n");
			goto error;
		}
		if ((tx_r && !sdr_engine_bins(tx_r, samplerate, server_nmax)) || (rx_r && !sdr_engine_bins(rx_r, samplerate, server_nmax))) {
			LOGP(DSDR, LOGL_ERROR, "SDR backend: no filter bank fits SDR rate %d / %d and DSP rate %d (sub-bands must be wider than the DSP rate).\n", tx_r, rx_r, samplerate);
			goto error;
		}
		rc = sdr_engine_init(&sdr->engine, rx_r, tx_r, samplerate, server_nmax, max_block);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Filter bank engine: %s\n", sdr_engine_error());
			goto error;
		}
		sdr->use_engine = 1;
		/* TX: the device a few blocks ahead, as in engine mode */
		if (tx_r) {
			double adv = 0.005, buf = (double)buffer_size / samplerate;

			if (4.0 * 2040 / sdr->tx_rate > adv)
				adv = 4.0 * 2040 / sdr->tx_rate;
			if (adv > buf / 2.0)
				adv = buf / 2.0;
			sdr->eng_advance = (int)(adv * sdr->tx_rate);
		}
		LOGP(DSDR, LOGL_INFO, "SDR backend: filter banks for up to %d TX channels at DSP rates up to %d, centre TX %.6f MHz, RX %.6f MHz\n",
		     server_nmax, samplerate, tx_center_frequency / 1e6, rx_center_frequency / 1e6);
	} else
	if (tx_frequency && !channels) {
		/* channels = 0: the caller's IQ is the whole band around its
		 * frequency, band_shift above the centre (off the DC) */
		tx_center_frequency = tx_frequency[0] - sdr->lo_tx.band_shift;
		if (sdr->lo_tx.band_shift != 0.0)
			LOGP(DSDR, LOGL_INFO, "TX: the band %+.1f kHz off the centre %.6f MHz (off the DC)\n", sdr->lo_tx.band_shift / 1e3, tx_center_frequency / 1e6);
		rc = channelizer_tx_init(&sdr->band_tx, sdr->tx_rate, samplerate, sdr->lo_tx.band_shift, 2.0 * CHANNELIZER_PASS * samplerate, sdr->buffer_size);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "TX: %s\n", channelizer_error());
			goto error;
		}
		sdr->band_tx_init = 1;
	}

	/* TX IQ wave recording / playback at the SDR rate (the signal sent to the device) */
	if (tx_frequency && sdr_config->write_iq_tx_wave) {
		rc = wave_create_record(&sdr->wave_tx_rec, sdr_config->write_iq_tx_wave, sdr->tx_rate, 2, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create TX WAVE recording instance!\n");
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Recording TX IQ (%d S/s) to: %s\n", sdr->tx_rate, sdr_config->write_iq_tx_wave);
	}
	if (tx_frequency && sdr_config->read_iq_tx_wave) {
		int two = 2, rate = sdr->tx_rate;
		rc = wave_create_playback(&sdr->wave_tx_play, sdr_config->read_iq_tx_wave, &rate, &two, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create TX WAVE playback instance!\n");
			goto error;
		}
		if (rate != sdr->tx_rate) {
			LOGP(DSDR, LOGL_ERROR, "TX IQ wave '%s' has %d S/s, the SDR TX rate is %d S/s.\n", sdr_config->read_iq_tx_wave, rate, sdr->tx_rate);
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Replacing TX IQ from: %s\n", sdr_config->read_iq_tx_wave);
	}

	/* engine mode: the filter banks run in the device threads; the channels
	 * are registered as one group after their offsets are known (below) */
	if (channels) {
		int ntx = channels + (sdr->paging_channel ? 1 : 0);
		int tx_r = tx_frequency ? sdr->tx_rate : 0, rx_r = rx_frequency ? sdr->rx_rate : 0;

		if (!sdr_tx_is_rpitx() && sdr_engine_wanted(tx_r, rx_r, samplerate, ntx)) {
			int max_block = (sdr->rx_max_in > sdr->tx_max_out) ? sdr->rx_max_in : sdr->tx_max_out;

			rc = sdr_engine_init(&sdr->engine, rx_r, tx_r, samplerate, ntx, max_block);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "Filter bank engine: %s\n", sdr_engine_error());
				goto error;
			}
			sdr->use_engine = 1;
			if (tx_r)
				LOGP(DSDR, LOGL_INFO, "TX filter bank: %d sub-bands of %.1f kHz at %d S/s, %d taps per branch\n",
				     sdr->engine.bank_tx.m, (double)tx_r / sdr->engine.bank_tx.m / 1e3, pfb_bin_rate(&sdr->engine.bank_tx), sdr->engine.bank_tx.t);
			if (rx_r)
				LOGP(DSDR, LOGL_INFO, "RX filter bank: %d sub-bands of %.1f kHz at %d S/s, %d taps per branch\n",
				     sdr->engine.bank_rx.m, (double)rx_r / sdr->engine.bank_rx.m / 1e3, pfb_bin_rate(&sdr->engine.bank_rx), sdr->engine.bank_rx.t);
		}
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

		if (sdr_tx_is_rpitx()) {
			int carriers = channels + (sdr->paging_channel ? 1 : 0);

			if (carriers > 2) {
				LOGP(DSDR, LOGL_ERROR, "rpitx can send one or two channels, not %d: it has one carrier that only moves in frequency.\n", carriers);
				goto error;
			}
			for (c = 0; c < channels; c++) {
				if (am[c])
					LOGP(DSDR, LOGL_NOTICE, "rpitx cannot modulate the amplitude: AM channel %d only gets the carrier keyed on and off.\n", c);
			}
			if (carriers == 2) {
				double spacing = tx_high_frequency - tx_low_frequency;
				int tx_rate = sdr->tx_rate;

				LOGP(DSDR, LOGL_NOTICE, "rpitx: two channels on one carrier (experimental, see docs/RPITX.md). The second channel at %.2f of the first one's level (--sdr-rpitx-ratio). The first channel should stay on (control channel), the second one may key on and off. One spacing on the far side of the first channel carries mixing products, keep it unused.\n", sdr_config->rpitx_ratio);
				if (spacing > tx_rate / 3.0)
					LOGP(DSDR, LOGL_NOTICE, "rpitx: the spacing of %.0f kHz is more than a third of the sample rate, mixing products fold back near the channels. Increase the sample rate.\n", spacing / 1e3);
			}
		}

		/* prevent channel bandwidth from overlapping with the center frequency,
		 * rpitx makes the carrier directly, it has no DC offset */
		if (channels == 1 && !sdr->paging_channel && sdr_tx_is_rpitx()) {
			LOGP(DSDR, LOGL_INFO, "rpitx: the carrier is the channel frequency.\n");
		} else if (channels == 1 && !sdr->paging_channel) {
			/* simple: just move off the center by two times half of the bandwidth */
			tx_center_frequency -= 2.0 * bandwidth / 2.0;
			/* Note: tx_low_frequency is kept at old center.
			   Calculation of 'low_side' will become 0.
			   This is correct, since there is no bandwidth
			   below new center frequency.
			 */
			LOGP(DSDR, LOGL_INFO, "We shift center frequency %.0f KHz down (one channel bandwidth, the channel's edge half of it off the center), to prevent channel from overlapping with DC level.\n", bandwidth / 1e3);
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
		int tx_rate = sdr->tx_rate;
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
		lo_spur_notice("TX", &sdr->lo_tx, tx_center_frequency, sdr->tx_rate, tx_frequency, channels, bandwidth);
		/* modulators at 0 Hz at the DSP rate, each channel's synthesizer
		 * moves it to its offset at the SDR rate (the paging channel has
		 * its own, so states stay continuous when a channel switches) */
		for (c = 0; c < channels + (sdr->paging_channel ? 1 : 0); c++) {
			double tx_offset = sdr->chan[c].tx_frequency - tx_center_frequency;

			if (c < channels) {
				sdr->chan[c].am = am[c];
				if (am[c]) {
					double gain, bias;
					gain = modulation_index / 2.0;
					bias = 1.0 - gain;
					rc = am_mod_init(&sdr->chan[c].am_mod, samplerate, 0.0, sdr->amplitude * gain, sdr->amplitude * bias);
				} else
					rc = fm_mod_init(&sdr->chan[c].fm_mod, samplerate, 0.0, sdr_tx_amplitude(sdr, c, channels));
			} else
				rc = fm_mod_init(&sdr->chan[c].fm_mod, samplerate, 0.0, sdr_tx_amplitude(sdr, c, channels));
			if (rc < 0)
				goto error;
			if (sdr->use_engine) {
				/* registered with the engine below */
				sdr->chan[c].tx_offset = tx_offset;
				LOGP(DSDR, LOGL_INFO, "TX %s %d: %.6f MHz, offset %+.1f kHz, filter bank\n",
				     (c < channels) ? "channel" : "paging channel", c, sdr->chan[c].tx_frequency / 1e6, tx_offset / 1e3);
				continue;
			}
			rc = channelizer_tx_init(&sdr->chan[c].tx_ch, sdr->tx_rate, samplerate, tx_offset, bandwidth, sdr->buffer_size);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "TX %s %.6f MHz: %s\n", (c < channels) ? "channel" : "paging channel", sdr->chan[c].tx_frequency / 1e6, channelizer_error());
				goto error;
			}
			sdr->chan[c].tx_init = 1;
			LOGP(DSDR, LOGL_INFO, "TX %s %d: %.6f MHz, offset %+.1f kHz, %d halfband stage(s) to %d S/s%s\n",
			     (c < channels) ? "channel" : "paging channel", c, sdr->chan[c].tx_frequency / 1e6, tx_offset / 1e3,
			     sdr->chan[c].tx_ch.hb_stages, sdr->chan[c].tx_ch.mid_rate, sdr->chan[c].tx_ch.use_resamp ? ", resampler" : "");
		}
		/* show gain */
		LOGP(DSDR, LOGL_INFO, "Using gain: TX %.1f dB\n", sdr_config->tx_gain);
	}

	if (rx_frequency && !channels && !server_nmax) {
		rx_center_frequency = rx_frequency[0] - sdr->lo_rx.band_shift;
		if (sdr->lo_rx.band_shift != 0.0)
			LOGP(DSDR, LOGL_INFO, "RX: the band %+.1f kHz off the centre %.6f MHz (off the DC)\n", sdr->lo_rx.band_shift / 1e3, rx_center_frequency / 1e6);
	}
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
			LOGP(DSDR, LOGL_INFO, "We shift center frequency %.0f KHz down (one channel bandwidth, the channel's edge half of it off the center), to prevent channel from overlapping with DC level.\n", bandwidth / 1e3);
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
		int rx_rate = sdr->rx_rate;
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
		lo_spur_notice("RX", &sdr->lo_rx, rx_center_frequency, sdr->rx_rate, rx_frequency, channels, bandwidth);
		/* each channel's channelizer takes it from its offset at the SDR rate
		 * to 0 Hz at the DSP rate, the demodulators work at 0 Hz */
		for (c = 0; c < channels; c++) {
			double rx_offset = sdr->chan[c].rx_frequency - rx_center_frequency;

			sdr->chan[c].am = am[c];
			if (am[c])
				rc = am_demod_init(&sdr->chan[c].am_demod, samplerate, 0.0, bandwidth / 2.0, 1.0 / modulation_index); /* bandwidth is only one side band */
			else
				rc = fm_demod_init(&sdr->chan[c].fm_demod, samplerate, 0.0, bandwidth); /* bandwidth are deviation and both sidebands */
			if (rc < 0)
				goto error;
			sdr->chan[c].rx_offset = rx_offset;
			if (sdr->use_engine) {
				/* registered with the engine below */
				LOGP(DSDR, LOGL_INFO, "RX channel %d: %.6f MHz, offset %+.1f kHz, filter bank\n",
				     c, sdr->chan[c].rx_frequency / 1e6, rx_offset / 1e3);
				continue;
			}
			rc = channelizer_rx_init(&sdr->chan[c].rx_ch, sdr->rx_rate, samplerate, rx_offset, bandwidth, sdr->rx_max_in);
			if (rc < 0) {
				LOGP(DSDR, LOGL_ERROR, "RX channel %.6f MHz: %s\n", sdr->chan[c].rx_frequency / 1e6, channelizer_error());
				goto error;
			}
			sdr->chan[c].rx_init = 1;
			LOGP(DSDR, LOGL_INFO, "RX channel %d: %.6f MHz, offset %+.1f kHz, %d halfband stage(s) to %d S/s%s\n",
			     c, sdr->chan[c].rx_frequency / 1e6, rx_offset / 1e3,
			     sdr->chan[c].rx_ch.hb_stages, sdr->chan[c].rx_ch.mid_rate, sdr->chan[c].rx_ch.use_resamp ? ", resampler" : "");
		}
		/* show gain */
		LOGP(DSDR, LOGL_INFO, "Using gain: RX %.1f dB\n", sdr_config->rx_gain);
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

	/* engine mode: the program's channels as one group (a backend's
	 * device has none, its programs register theirs) */
	if (sdr->use_engine && channels) {
		int ntx = tx_frequency ? channels + (sdr->paging_channel ? 1 : 0) : 0;
		int nrx = rx_frequency ? channels : 0, nbuf = (ntx > nrx) ? ntx : nrx;
		double tx_off[SDR_ENGINE_MAX_CHAN], rx_off[SDR_ENGINE_MAX_CHAN];

		if (nbuf > SDR_ENGINE_MAX_CHAN) {
			LOGP(DSDR, LOGL_ERROR, "%d channels, the filter bank engine takes at most %d.\n", nbuf, SDR_ENGINE_MAX_CHAN);
			goto error;
		}
		for (c = 0; c < ntx; c++)
			tx_off[c] = sdr->chan[c].tx_offset;
		for (c = 0; c < nrx; c++)
			rx_off[c] = sdr->chan[c].rx_offset;
		/* rings: 4 buffers of DSP samples; TX kept at one buffer (the
		 * latency as before) */
		sdr->group = sdr_engine_group_add(&sdr->engine, samplerate, nrx, rx_off, ntx, tx_off,
						  (int)ceil(4.0 * buffer_size * 1000.0 / samplerate) + 10, buffer_size);
		if (!sdr->group) {
			LOGP(DSDR, LOGL_ERROR, "Filter bank engine: %s\n", sdr_engine_error());
			goto error;
		}
		sdr->port = &sdr->group->port;
		for (c = 0; c < ntx; c++)
			sdr->chan[c].tx_init = 1;
		for (c = 0; c < nrx; c++)
			sdr->chan[c].rx_init = 1;
		sdr->eng_i = calloc(nbuf, sizeof(*sdr->eng_i));
		sdr->eng_q = calloc(nbuf, sizeof(*sdr->eng_q));
		if (!sdr->eng_i || !sdr->eng_q) {
			LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
			goto error;
		}
		sdr->eng_n = nbuf;
		for (c = 0; c < nbuf; c++) {
			sdr->eng_i[c] = calloc(buffer_size, sizeof(sample_t));
			sdr->eng_q[c] = calloc(buffer_size, sizeof(sample_t));
			if (!sdr->eng_i[c] || !sdr->eng_q[c]) {
				LOGP(DSDR, LOGL_ERROR, "NO MEM!\n");
				goto error;
			}
		}
		/* TX latency as before (one buffer): the device a few blocks ahead
		 * (at least 5 ms, 4 UHD packets), the group's ring the rest */
		if (ntx) {
			double adv = 0.005, buf = (double)buffer_size / samplerate;

			if (4.0 * 2040 / sdr->tx_rate > adv)
				adv = 4.0 * 2040 / sdr->tx_rate;
			if (adv > buf / 2.0)
				adv = buf / 2.0;
			sdr->eng_advance = (int)(adv * sdr->tx_rate);
			sdr->eng_ring_target = buffer_size - (int)(adv * samplerate);
			LOGP(DSDR, LOGL_INFO, "TX latency %.1f ms: %.1f ms in the device, %.1f ms in the filter bank ring\n",
			     buf * 1e3, adv * 1e3, (buf - adv) * 1e3);
		}
		for (c = 0; c < ntx; c++)
			LOGP(DSDR, LOGL_INFO, "TX %s %d: filter bank sub-band %d\n", (c < channels) ? "channel" : "paging channel", c, sdr->group->tx[c].bin);
		for (c = 0; c < nrx; c++)
			LOGP(DSDR, LOGL_INFO, "RX channel %d: filter bank sub-band %d\n", c, sdr->group->rx[c].bin);
	}

	if (rx_frequency && !channels && !server_nmax) {
		/* channels = 0: the whole usable band around the centre to the caller */
		rc = channelizer_rx_init(&sdr->band_rx, sdr->rx_rate, samplerate, sdr->lo_rx.band_shift, 2.0 * CHANNELIZER_PASS * samplerate, sdr->rx_max_in);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "RX: %s\n", channelizer_error());
			goto error;
		}
		sdr->band_rx_init = 1;
		/* no stage, no NCO step: its output is its input, bit for bit */
		sdr->band_rx_pass = (sdr->band_rx.hb_stages == 0 && !sdr->band_rx.use_resamp && sdr->band_rx.offset == 0.0);
	}

	/* RX IQ wave recording / playback at the SDR rate (the signal from the device) */
	if (rx_frequency && sdr_config->write_iq_rx_wave) {
		rc = wave_create_record(&sdr->wave_rx_rec, sdr_config->write_iq_rx_wave, sdr->rx_rate, 2, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create RX WAVE recording instance!\n");
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Recording RX IQ (%d S/s) to: %s\n", sdr->rx_rate, sdr_config->write_iq_rx_wave);
	}
	if (rx_frequency && sdr_config->read_iq_rx_wave) {
		int two = 2, rate = sdr->rx_rate;
		rc = wave_create_playback(&sdr->wave_rx_play, sdr_config->read_iq_rx_wave, &rate, &two, 1.0);
		if (rc < 0) {
			LOGP(DSDR, LOGL_ERROR, "Failed to create RX WAVE playback instance (the file must have the SDR RX rate %d S/s)!\n", sdr->rx_rate);
			goto error;
		}
		LOGP(DSDR, LOGL_INFO, "Replacing RX IQ from: %s\n", sdr_config->read_iq_rx_wave);
	}

	/* IQ and spectrum display of the SDR rate IQ */
	display_iq_init(sdr->rx_rate);
	display_spectrum_init(sdr->rx_rate, rx_center_frequency);

	/* (the LO offsets: sdr->lo_tx / lo_rx, logged above) */
	rx_band_shift = sdr->lo_rx.band_shift;
	tx_band_shift = sdr->lo_tx.band_shift;

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
			rc = uhd_open(sdr_config->tx_channel, sdr_config->tx_device_args, sdr_config->tx_stream_args, sdr_config->tx_tune_args, sdr_config->tx_antenna, NULL, sdr_config->tx_clock_source, actual_tx_center, 0.0, sdr->lo_tx.lo_offset, (double)sdr_config->tx_samplerate, sdr_config->tx_gain, 0.0, lo_bandwidth("TX", sdr_config->tx_bandwidth, sdr_config->tx_bandwidth_given || sdr_config->bandwidth_given, &sdr->lo_tx, sdr->tx_rate), sdr_config->timestamps);
			if (rc)
				goto error;
			tx_driver = 1;
		}
#endif
#ifdef HAVE_SOAPY
		if (sdr_config->tx_soapy) {
			rc = soapy_open(sdr_config->tx_channel, sdr_config->tx_device_args, sdr_config->tx_stream_args, sdr_config->tx_tune_args, sdr_config->tx_antenna, NULL, sdr_config->tx_clock_source, actual_tx_center, 0.0, sdr->lo_tx.lo_offset, (double)sdr_config->tx_samplerate, sdr_config->tx_gain, 0.0, lo_bandwidth("TX", sdr_config->tx_bandwidth, sdr_config->tx_bandwidth_given || sdr_config->bandwidth_given, &sdr->lo_tx, sdr->tx_rate), sdr_config->timestamps);
			if (rc)
				goto error;
			tx_driver = 2;
		}
#endif
#ifdef HAVE_RPITX
		if (sdr_config->tx_rpitx) {
			double rpitx_freq = actual_tx_center;
			if (sdr_config->tx_ppm != 0.0) {
				rpitx_freq = actual_tx_center * (1.0 + sdr_config->tx_ppm / 1e6);
				LOGP(DSDR, LOGL_INFO, "Applying TX PPM correction %.3f for rpitx: %.0f Hz -> %.0f Hz\n", sdr_config->tx_ppm, actual_tx_center, rpitx_freq);
			}
			rc = rpitx_open(rpitx_freq, (double)sdr_config->tx_samplerate, sdr_config->tx_gain);
			if (rc)
				goto error;
			tx_driver = 3;
		}
#endif
#ifdef HAVE_UHD
		if (sdr_config->rx_uhd) {
			rc = uhd_open(sdr_config->rx_channel, sdr_config->rx_device_args, sdr_config->rx_stream_args, sdr_config->rx_tune_args, NULL, sdr_config->rx_antenna, sdr_config->rx_clock_source, 0.0, actual_rx_center, sdr->lo_rx.lo_offset, (double)sdr_config->rx_samplerate, 0.0, sdr_config->rx_gain, lo_bandwidth("RX", sdr_config->rx_bandwidth, sdr_config->rx_bandwidth_given || sdr_config->bandwidth_given, &sdr->lo_rx, sdr->rx_rate), sdr_config->timestamps);
			if (rc)
				goto error;
			rx_driver = 1;
		}
#endif
#ifdef HAVE_SOAPY
		if (sdr_config->rx_soapy) {
			rc = soapy_open(sdr_config->rx_channel, sdr_config->rx_device_args, sdr_config->rx_stream_args, sdr_config->rx_tune_args, NULL, sdr_config->rx_antenna, sdr_config->rx_clock_source, 0.0, actual_rx_center, sdr->lo_rx.lo_offset, (double)sdr_config->rx_samplerate, 0.0, sdr_config->rx_gain, lo_bandwidth("RX", sdr_config->rx_bandwidth, sdr_config->rx_bandwidth_given || sdr_config->bandwidth_given, &sdr->lo_rx, sdr->rx_rate), sdr_config->timestamps);
			if (rc)
				goto error;
			rx_driver = 2;
		}
#endif
	} else {
		/* Single-device mode (including TX-only / RX-only): existing dispatch */
#ifdef HAVE_UHD
		if (sdr_config->uhd) {
			rc = uhd_open(sdr_config->channel, sdr_config->device_args, sdr_config->stream_args, sdr_config->tune_args, sdr_config->tx_antenna, sdr_config->rx_antenna, sdr_config->clock_source, actual_tx_center, actual_rx_center, (actual_rx_center != 0.0) ? sdr->lo_rx.lo_offset : sdr->lo_tx.lo_offset, sdr_config->samplerate, sdr_config->tx_gain, sdr_config->rx_gain, unified_bandwidth(sdr, actual_tx_center != 0.0, actual_rx_center != 0.0), sdr_config->timestamps);
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
			rc = soapy_open(sdr_config->channel, sdr_config->device_args, sdr_config->stream_args, sdr_config->tune_args, sdr_config->tx_antenna, sdr_config->rx_antenna, sdr_config->clock_source, actual_tx_center, actual_rx_center, (actual_rx_center != 0.0) ? sdr->lo_rx.lo_offset : sdr->lo_tx.lo_offset, sdr_config->samplerate, sdr_config->tx_gain, sdr_config->rx_gain, unified_bandwidth(sdr, actual_tx_center != 0.0, actual_rx_center != 0.0), sdr_config->timestamps);
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
			double rpitx_freq = actual_tx_center;
			if (sdr_config->tx_ppm != 0.0) {
				rpitx_freq = actual_tx_center * (1.0 + sdr_config->tx_ppm / 1e6);
				LOGP(DSDR, LOGL_INFO, "Applying TX PPM correction %.3f for rpitx: %.0f Hz -> %.0f Hz\n", sdr_config->tx_ppm, actual_tx_center, rpitx_freq);
			}
			rc = rpitx_open(rpitx_freq, (double)sdr_config->samplerate, sdr_config->tx_gain);
			if (rc)
				goto error;
			tx_driver = 3;
		}
#endif
	}

	/* Initialize SDR status monitoring */
	sdr_status_init(channels, 1.0);
	sdr_status_iq_counters(sdr_config->iq_status);

	/* FFT-based spectral measurement is configured from main.c after sdr_open,
	 * using the same data path as the scan engine (decimated DSP-rate IQ). */

	return sdr;

error:
	sdr_close(sdr);
	return NULL;
}

void *sdr_open(int direction, const char *audiodev, double *tx_frequency, double *rx_frequency, int *am, int channels, double paging_frequency, int samplerate, int buffer_size, double interval, double max_deviation, double max_modulation, double modulation_index)
{
	return sdr_open_internal(direction, audiodev, tx_frequency, rx_frequency, am, channels, paging_frequency, samplerate, buffer_size, interval, max_deviation, max_modulation, modulation_index);
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
static double rx_retune_done;	/* when the last RX retune returned (get_time()) */

int sdr_set_rx_frequency(double freq)
{
	int rc = -ENODEV;

	/* the band (channels = 0) is off the DC: the device that much lower */
	freq -= rx_band_shift;
#ifdef HAVE_UHD
	if (rx_driver == 1)
		rc = uhd_set_rx_frequency(freq);
	else
#endif
#ifdef HAVE_SOAPY
	if (rx_driver == 2)
		rc = soapy_set_rx_frequency(freq);
	else
#endif
		LOGP(DSDR, LOGL_ERROR, "Cannot set RX frequency: no SDR backend active\n");
	rx_retune_done = get_time();
	return rc;
}

/* Measured on the B210 (UHD 4.8, 1 MS/s): a step within a band returns in
 * 3 ms, another band in 100-120 ms; for as long as the call takes, the
 * stream carries exact zeros (no gap reported: they come from the device or
 * UHD, not from the gap fill here); the first samples at the new frequency
 * are in the ring 1-3 ms after the return, their level settles within
 * 2.5 ms. */
long sdr_rx_settle(void *inst, double settle_sec)
{
	sdr_t *sdr = (sdr_t *)inst;
	long dropped = 0;
	int avail, last = 0;

	if (!sdr || !sdr->thread_read.buffer)
		return 0;
	while (1) {
		/* the ring's samples so far: dropped (the read side owns out) */
		__sync_synchronize();
		avail = ((sdr->thread_read.in - sdr->thread_read.out + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size) / 2;
		sdr->thread_read.out = (sdr->thread_read.out + 2 * avail) % sdr->thread_read.buffer_size;
		dropped += avail;
		if (last)
			break;
		/* once more after the time: what came until then */
		if (get_time() >= rx_retune_done + settle_sec)
			last = 1;
		else
			usleep(1000);
	}
	/* the band channelizer's kept output is from before, too */
	if (sdr->band_rx_init)
		sdr->band_rx.fifo_len = 0;
	return dropped;
}

/* Set TX frequency on the active SDR backend */
int sdr_set_tx_frequency(double freq)
{
	freq -= tx_band_shift;
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

static int sdr_tx_device_paced(void);

/* Number of RX samples the device dropped before the last received packet
 * (measured from device time stamps, 0 if unknown). Clears the value. */
static long sdr_rx_take_gap(void)
{
#ifdef HAVE_UHD
	if (rx_driver == 1)
		return uhd_rx_take_gap();
#endif
#ifdef HAVE_SOAPY
	if (rx_driver == 2)
		return soapy_rx_take_gap();
#endif
	return 0;
}

static void *sdr_write_child(void *arg)
{
	sdr_t *sdr = (sdr_t *)arg;
	int num;
	int fill, out;
	int s, ss;

	/* Diagnostics for split mode */
	int diag_count = 0;
	int empty_count = 0;
	int total_samples = 0;
	double last_diag_time = 0.0;
	int max_fill_diag = 0;
	int min_fill_diag = INT_MAX;
	int split_mode = sdr_config && sdr_config->split_mode;

	/* Device TX event counters and send() drain rate */
	unsigned long ev_underflow = 0, ev_seq_error = 0, ev_time_error = 0;
	unsigned long ev_underflow_logged = 0, ev_seq_error_logged = 0, ev_time_error_logged = 0;
	double pace_start = 0.0, pace_last = 0.0, pace_last_poll = 0.0;
	long pace_samples = 0;

	/* The ring holds SDR rate IQ (sdr_write() synthesizes the channels).
	 * Chunk size: UHD: the hardware MTU, so send() blocks as the
	 * pacemaker; others: about 1 ms. */
	int tx_chunk = 0;
#ifdef HAVE_UHD
	if (tx_driver == 1) {
		int mtu = uhd_get_tx_mtu();
		if (mtu > 0) {
			tx_chunk = mtu;
			LOGP(DSDR, LOGL_NOTICE, "WRITE_THREAD: UHD MTU=%d samples, tx_chunk=%d samples\n", mtu, tx_chunk);
		}
	}
#endif
	if (tx_chunk == 0) {
		tx_chunk = sdr->tx_rate / 1000;
		if (tx_chunk < 100)
			tx_chunk = 100;
		LOGP(DSDR, LOGL_NOTICE, "WRITE_THREAD: using tx_chunk=%d samples (1 ms)\n", tx_chunk);
	}
	if (tx_chunk > sdr->tx_max_out)
		tx_chunk = sdr->tx_max_out;
	/* engine mode: this thread makes the SDR rate IQ, in whole TX bank
	 * frames; paced by the device's blocking send, or by what the driver
	 * says is needed to stay one buffer ahead (time stamps, as
	 * sdr_get_tosend() did for the ring) */
	int eng_chunk = 0, eng_step = 1, eng_target = 0;
	if (sdr->use_engine) {
		eng_step = sdr_engine_tx_step(&sdr->engine);
		eng_chunk = tx_chunk / eng_step * eng_step;
		if (eng_chunk < eng_step)
			eng_chunk = eng_step;
		/* the device's advance: small (the group's ring holds the rest of
		 * the buffer, sdr_get_tosend()), so the latency is one buffer as
		 * before */
		eng_target = sdr->eng_advance;
		LOGP(DSDR, LOGL_NOTICE, "WRITE_THREAD: filter bank engine, up to %d samples per block, device %.1f ms ahead\n",
		     eng_chunk, 1000.0 * eng_target / sdr->tx_rate);
	}
	int out_samples;

	while (sdr->thread_write.running) {
		if (sdr->use_engine) {
			num = eng_chunk;
			if (!sdr_tx_device_paced()) {
				int want = 0;
#ifdef HAVE_UHD
				if (tx_driver == 1)
					want = uhd_get_tosend(eng_target);
#endif
#ifdef HAVE_SOAPY
				if (tx_driver == 2)
					want = soapy_get_tosend(eng_target);
#endif
				if (want < eng_step) {
					usleep(200);
					continue;
				}
				if (want < num)
					num = want / eng_step * eng_step;
			}
			sdr_engine_tx_produce(&sdr->engine, sdr->thread_write.buffer2, num);
			/* keep the sum within the range of the device */
			for (s = 0; s < 2 * num; s++)
				sdr->thread_write.buffer2[s] *= LIMIT_IQ_LEVEL;
			if (sdr->wave_tx_rec.fp || sdr->wave_tx_play.fp) {
				sample_t *spl_list[2] = { sdr->wavespl0, sdr->wavespl1 };

				if (sdr->wave_tx_rec.fp) {
					for (s = 0; s < num; s++) {
						spl_list[0][s] = sdr->thread_write.buffer2[2 * s];
						spl_list[1][s] = sdr->thread_write.buffer2[2 * s + 1];
					}
					wave_write(&sdr->wave_tx_rec, spl_list, num);
				}
				if (sdr->wave_tx_play.fp) {
					wave_read(&sdr->wave_tx_play, spl_list, num);
					for (s = 0; s < num; s++) {
						sdr->thread_write.buffer2[2 * s] = spl_list[0][s];
						sdr->thread_write.buffer2[2 * s + 1] = spl_list[1][s];
					}
				}
			}
			sdr_status_tx_iq_accumulate(sdr->thread_write.buffer2, num);
			out_samples = num;
			goto send_it;
		}
		/* Check how much data is available in ring buffer */
		fill = (sdr->thread_write.in - sdr->thread_write.out + sdr->thread_write.buffer_size) % sdr->thread_write.buffer_size;
		num = fill / 2;
		
		/* Track buffer statistics for split mode diagnostics */
		if (split_mode) {
			diag_count++;
			if (num == 0) {
				empty_count++;
			} else {
				if (fill > max_fill_diag) max_fill_diag = fill;
				if (fill < min_fill_diag) min_fill_diag = fill;
			}
			
			/* Log diagnostics every second */
			double now = get_time();
			if (last_diag_time == 0.0) last_diag_time = now;
			if (now - last_diag_time >= 1.0) {
				LOGP(DSDR, LOGL_DEBUG, "WRITE_THREAD: iterations=%d empty=%d (%.1f%%) samples=%d avg_fill=%d min=%d max=%d buf_size=%d chunk=%d\n",
				     diag_count, empty_count, 
				     diag_count > 0 ? (100.0 * empty_count / diag_count) : 0.0,
				     total_samples,
				     diag_count - empty_count > 0 ? total_samples / (diag_count - empty_count) : 0,
				     min_fill_diag == INT_MAX ? 0 : min_fill_diag,
				     max_fill_diag,
				     sdr->thread_write.buffer_size,
				     tx_chunk);
				diag_count = 0;
				empty_count = 0;
				total_samples = 0;
				max_fill_diag = 0;
				min_fill_diag = INT_MAX;
				last_diag_time = now;
			}
		}

		/* Send fixed-size chunks to keep the SDR FIFO continuously fed.
		 * Cap to tx_chunk so send() blocks as the pacemaker, spreading
		 * data evenly over time instead of sending bursts. */
		if (num >= tx_chunk)
			num = tx_chunk;
		else if (num == 0) {
			/* Ring buffer empty — short spin-wait to react quickly
			 * when the main loop produces more data. */
			usleep(50);
			continue;
		}
		/* else: num < tx_chunk but > 0 — send what we have to avoid starvation */

		if (split_mode)
			total_samples += num;

#ifdef DEBUG_BUFFER
		printf("Thread found %d samples in write buffer and forwards them to SDR.\n", num);
#endif
		out = sdr->thread_write.out;
		out_samples = num;	/* SDR rate samples to send */

		for (s = 0, ss = 0; s < num; s++) {
			sdr->thread_write.buffer2[ss++] = sdr->thread_write.buffer[out];
			sdr->thread_write.buffer2[ss++] = sdr->thread_write.buffer[out + 1];
			out = (out + 2) % sdr->thread_write.buffer_size;
		}
		sdr->thread_write.out = out;

send_it:
		/* send() blocks until the hardware is ready, acting as the
		 * pacemaker that keeps data flowing at the exact SDR rate. */
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
		pace_samples += num;

		/* Device TX events and pacing check, every 100 ms */
		{
			double now = get_time();

			if (pace_start == 0.0) {
				pace_start = now;
				pace_last = now;
				pace_last_poll = now;
			}
			if (now - pace_last_poll >= 0.1) {
				unsigned long u = 0, sq = 0, te = 0;

				pace_last_poll = now;
#ifdef HAVE_UHD
				if (tx_driver == 1)
					uhd_tx_poll_events(&u, &sq, &te);
#endif
#ifdef HAVE_SOAPY
				if (tx_driver == 2)
					soapy_tx_poll_events(&u, &sq, &te);
#endif
				ev_underflow += u;
				ev_seq_error += sq;
				ev_time_error += te;
			}
			if (now - pace_last >= 1.0) {
				unsigned long du = ev_underflow - ev_underflow_logged;
				unsigned long dsq = ev_seq_error - ev_seq_error_logged;
				unsigned long dte = ev_time_error - ev_time_error_logged;
				double rate = pace_samples / (now - pace_last);

				if (du || dsq || dte)
					LOGP(DSDR, LOGL_ERROR, "SDR TX device events in last %.1fs: underflow %lu, seq error %lu, time error %lu (total %lu/%lu/%lu)\n",
					     now - pace_last, du, dsq, dte, ev_underflow, ev_seq_error, ev_time_error);
				ev_underflow_logged = ev_underflow;
				ev_seq_error_logged = ev_seq_error;
				ev_time_error_logged = ev_time_error;

				/* After the device buffers are filled, a device paced
				 * send() drains at the sample rate. Much faster means
				 * send() does not block and TX is not paced at all. */
				if (sdr_tx_device_paced() && now - pace_start > 5.0 && rate > 1.5 * sdr->tx_rate)
					LOGP(DSDR, LOGL_ERROR, "SDR TX: send() drains %.0f samples/s, expected %d: driver send() is not blocking, TX is unpaced\n",
					     rate, sdr->tx_rate);
				pace_samples = 0;
				pace_last = now;
			}
		}
	}

	if (ev_underflow || ev_seq_error || ev_time_error)
		LOGP(DSDR, LOGL_NOTICE, "SDR TX device events total: underflow %lu, seq error %lu, time error %lu\n",
		     ev_underflow, ev_seq_error, ev_time_error);
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

	/* The ring takes the SDR rate IQ as received; sdr_read() channelizes it. */
	sample_t *wav_i = calloc(sdr->rx_max_in, sizeof(*wav_i));
	sample_t *wav_q = calloc(sdr->rx_max_in, sizeof(*wav_q));

	if (!wav_i || !wav_q) {
		LOGP(DSDR, LOGL_ERROR, "No mem!\n");
		free(wav_i);
		free(wav_q);
		sdr->thread_read.exit = 1;
		return NULL;
	}

	/* DEBUG: track thread activity */
	static int thr_dbg_cnt = 0;
	static int thr_total_written = 0;
	static int thr_zero_space = 0;
	static int thr_zero_count = 0;
	static double thr_timer = 0;

	while (sdr->thread_read.running) {
		int got = 0;	/* samples the device delivered (count may be cut to the ring's room) */

		/* read from SDR */
		space = (sdr->thread_read.out - sdr->thread_read.in + sdr->thread_read.buffer_size - 2 + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size;
		num = space / 2;
		/* engine mode: the channels are made here, never hold the device
		 * back for the ring (it only feeds the status and the displays) */
		if (sdr->use_engine)
			num = sdr->rx_max_in;
		/* the read chunk buffer holds at most rx_max_in samples */
		if (num > sdr->rx_max_in)
			num = sdr->rx_max_in;
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
			got = count;
			/* IQ wave at the SDR rate: playback replaces what the device
			 * delivered, recording keeps what goes into the ring */
			if (count > 0 && (sdr->wave_rx_play.fp || sdr->wave_rx_rec.fp)) {
				sample_t *spl_list[2] = { wav_i, wav_q };

				if (sdr->wave_rx_play.fp) {
					wave_read(&sdr->wave_rx_play, spl_list, count);
					for (s = 0; s < count; s++) {
						sdr->thread_read.buffer2[2 * s] = wav_i[s];
						sdr->thread_read.buffer2[2 * s + 1] = wav_q[s];
					}
				}
				if (sdr->wave_rx_rec.fp) {
					for (s = 0; s < count; s++) {
						wav_i[s] = sdr->thread_read.buffer2[2 * s];
						wav_q[s] = sdr->thread_read.buffer2[2 * s + 1];
					}
					wave_write(&sdr->wave_rx_rec, spl_list, count);
				}
			}
			if (count > 0) {
				thr_total_written += count;
#ifdef DEBUG_BUFFER
				printf("Thread read %d samples from SDR and writes them to read buffer.\n", count);
#endif
				in = sdr->thread_read.in;

				if (sdr->use_engine) {
					/* the lost samples as zeros (the timeline stays
					 * continuous), then the packet, into the channels */
					long gap = sdr_rx_take_gap();

					while (gap > 0) {
						int n = (gap > sdr->rx_max_in) ? sdr->rx_max_in : (int)gap;

						memset(sdr->rx_gap_zero, 0, sizeof(float) * 2 * n);
						sdr_engine_rx_feed(&sdr->engine, sdr->rx_gap_zero, n);
						gap -= n;
					}
					sdr_engine_rx_feed(&sdr->engine, sdr->thread_read.buffer2, count);
					/* the raw ring: what fits (status, displays) */
					if (count > space / 2)
						count = space / 2;
				} else
				/* Samples lost before this packet (device overflow), measured
				 * from device time stamps: insert the same number of zero
				 * samples, so the sample timeline seen by the DSP stays
				 * continuous (a fade of the right length, no time jump for
				 * PLLs and symbol timing). Limited by the free ring space. */
				{
					long gap = sdr_rx_take_gap();

					if (gap > 0) {
						long room = num - count;	/* free ring space after this packet */
						long fill_n = (gap < room) ? gap : room;
						float z[1024];

						if (fill_n < gap)
							LOGP(DSDR, LOGL_ERROR, "RX gap of %ld samples, ring has room for %ld: %ld samples skipped\n",
							     gap, fill_n, gap - fill_n);
						while (fill_n > 0) {
							int n = (fill_n > 512) ? 512 : (int)fill_n;

							memset(z, 0, sizeof(float) * n * 2);
							for (s = 0, ss = 0; s < n; s++) {
								sdr->thread_read.buffer[in++] = z[ss++];
								sdr->thread_read.buffer[in++] = z[ss++];
								in %= sdr->thread_read.buffer_size;
							}
							fill_n -= n;
						}
					}
				}

				/* into the ring: up to its end, the rest from its start */
				{
					int first = (sdr->thread_read.buffer_size - in) / 2;

					if (first > count)
						first = count;
					/* the volatile only orders the index updates (with
					 * __sync_synchronize); the samples are plain copies */
					memcpy((float *)sdr->thread_read.buffer + in, sdr->thread_read.buffer2, first * 2 * sizeof(float));
					memcpy((float *)sdr->thread_read.buffer, sdr->thread_read.buffer2 + first * 2, (count - first) * 2 * sizeof(float));
					in = (in + count * 2) % sdr->thread_read.buffer_size;
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
			LOGP(DSDR, LOGL_DEBUG, "RX_THREAD: calls=%d written=%d zero_space=%d zero_count=%d | fill=%d/%d\n",
			     thr_dbg_cnt, thr_total_written, thr_zero_space, thr_zero_count,
			     fill/2, sdr->thread_read.buffer_size/2);
			thr_dbg_cnt = 0;
			thr_total_written = 0;
			thr_zero_space = 0;
			thr_zero_count = 0;
			thr_timer = now;
		}

		/* If receive functions block, we always receive something, so don't sleep.
		 * When ring buffer is full (num==0), use a very short sleep to
		 * avoid burning CPU while waiting for the consumer, but keep it
		 * short enough that UHD's internal FIFO doesn't overflow. */
		if (!num)
			usleep(100);
		else if (got <= 0)
			/* only when the device had nothing: in engine mode a full
			 * raw ring (status, displays; a backend does not read it)
			 * cuts count to 0, sleeping then makes this thread fall
			 * behind the device (RX time late, TX time stamps late) */
			usleep(sdr->interval * 1000.0);
	}

	free(wav_i);
	free(wav_q);
	LOGP(DSDR, LOGL_DEBUG, "Thread received exit!\n");
	sdr->thread_read.exit = 1;
	return NULL;
}

/* start streaming */
int sdr_start(void *inst)
{
	sdr_t *sdr = (sdr_t *)inst;
	int rc = -EINVAL;

	/* on a backend: its device threads run already */
	if (sdr->use_client)
		return 0;

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
		free(sdr->ch_i);
		free(sdr->ch_q);
		free(sdr->zero_power);
		free(sdr->zero_spl);
		free(sdr->rx_lin);
		free(sdr->rx_gap_zero);
		free(sdr->tx_sum);
		wave_destroy_record(&sdr->wave_rx_rec);
		wave_destroy_record(&sdr->wave_tx_rec);
		wave_destroy_playback(&sdr->wave_rx_play);
		wave_destroy_playback(&sdr->wave_tx_play);
		if (sdr->chan) {
			int c;

			for (c = 0; c < sdr->channels + (sdr->paging_channel ? 1 : 0); c++) {
				fm_mod_exit(&sdr->chan[c].fm_mod);
				if (c < sdr->channels) {
					fm_demod_exit(&sdr->chan[c].fm_demod);
					am_mod_exit(&sdr->chan[c].am_mod);
					am_demod_exit(&sdr->chan[c].am_demod);
				}
				if (sdr->chan[c].rx_init && !sdr->use_engine)
					channelizer_exit(&sdr->chan[c].rx_ch);
				if (sdr->chan[c].tx_init && !sdr->use_engine)
					channelizer_exit(&sdr->chan[c].tx_ch);
			}
			free(sdr->chan);
		}
		if (sdr->band_rx_init)
			channelizer_exit(&sdr->band_rx);
		if (sdr->band_tx_init)
			channelizer_exit(&sdr->band_tx);
		/* the device threads are stopped: the engine and its group go;
		 * on a backend the registration ends with the connection */
		if (sdr->use_client)
			sdr_client_close(&sdr->client);
		else if (sdr->use_engine)
			sdr_engine_exit(&sdr->engine);
		if (sdr->eng_i) {
			int b;

			for (b = 0; b < sdr->eng_n; b++) {
				free(sdr->eng_i[b]);
				free(sdr->eng_q[b]);
			}
		}
		free(sdr->eng_i);
		free(sdr->eng_q);
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

	/* all channels, each modulated at 0 Hz at the DSP rate and synthesized
	 * to its frequency at the SDR rate, added up in tx_sum */
	buff = sdr->tx_sum;
	if (channels && sdr->use_engine) {
		/* engine mode: the device TX thread makes the SDR rate IQ */
	} else
		memset(buff, 0, sizeof(*buff) * 2 * channelizer_tx_max(channels ? &sdr->chan[0].tx_ch : &sdr->band_tx, num));
	if (channels) {
		int n, total = -1;

		for (c = 0; c < channels + (sdr->paging_channel ? 1 : 0); c++) {
			sdr_chan_t *ch = &sdr->chan[c];

			memset(sdr->modbuff, 0, sizeof(*sdr->modbuff) * num * 2);
			if (c == sdr->paging_channel && sdr->paging_channel) {
				/* the paging channel carries the channel switched to it; else
				 * it runs without carrier, so its states stay continuous */
				int pc;

				for (pc = 0; pc < channels; pc++)
					if (on[pc])
						break;
				if (pc < channels)
					fm_modulate_complex(&ch->fm_mod, samples[pc], power[pc], num, sdr->modbuff);
				else
					fm_modulate_complex(&ch->fm_mod, sdr->zero_spl, sdr->zero_power, num, sdr->modbuff);
			} else if (on[c] && sdr->paging_channel) {
				/* switched to the paging channel: this one without carrier */
				fm_modulate_complex(&ch->fm_mod, sdr->zero_spl, sdr->zero_power, num, sdr->modbuff);
			} else if (ch->am)
				am_modulate_complex(&ch->am_mod, samples[c], power[c], num, sdr->modbuff);
			else
				fm_modulate_complex(&ch->fm_mod, samples[c], power[c], num, sdr->modbuff);
			for (s = 0; s < num; s++) {
				sdr->ch_i[s] = sdr->modbuff[2 * s];
				sdr->ch_q[s] = sdr->modbuff[2 * s + 1];
			}
			if (sdr->use_engine) {
				/* the channel's DSP samples, a row of all channels goes
				 * into the group's TX ring below */
				memcpy(sdr->eng_i[c], sdr->ch_i, num * sizeof(*sdr->ch_i));
				memcpy(sdr->eng_q[c], sdr->ch_q, num * sizeof(*sdr->ch_q));
				continue;
			}
			n = channelizer_tx_process(&ch->tx_ch, sdr->ch_i, sdr->ch_q, num, buff);
			if (total >= 0 && n != total)
				LOGP(DSDR, LOGL_ERROR, "TX synthesizers out of step (%d / %d samples), please fix!\n", n, total);
			total = n;
		}
		if (sdr->use_engine) {
			double chan_tx_freq[SDR_STATUS_MAX_CHANNELS];
			int written = sdr_port_write(sdr->port, sdr->eng_i, sdr->eng_q, num);

			if (written < num)
				LOGP(DSDR, LOGL_ERROR, "Write SDR buffer overflow (%d of %d DSP samples fit)!\n", written, num);
			/* the channels' state; the IQ levels count in the TX thread */
			for (c = 0; c < channels && c < SDR_STATUS_MAX_CHANNELS; c++)
				chan_tx_freq[c] = sdr->chan[c].tx_frequency;
			sdr_status_tx_accumulate(NULL, 0, power, num, channels, sdr->amplitude, chan_tx_freq);
			return num;
		}
		sent = total;
	} else {
		/* channels = 0: the caller's IQ (interleaved float) is the whole band */
		float *in = (float *)samples;

		if (sdr->band_client) {
			/* on the backend: the channel's rows at its level (1 / N max
			 * of full scale, as every channel there) */
			int written;

			for (s = 0; s < num; s++) {
				sdr->eng_i[0][s] = in[2 * s] * sdr->amplitude;
				sdr->eng_q[0][s] = in[2 * s + 1] * sdr->amplitude;
			}
			written = sdr_port_write(sdr->port, sdr->eng_i, sdr->eng_q, num);
			if (written < num)
				LOGP(DSDR, LOGL_ERROR, "Write SDR buffer overflow (%d of %d DSP samples fit)!\n", written, num);
			return num;
		}
		for (s = 0; s < num; s++) {
			sdr->ch_i[s] = in[2 * s];
			sdr->ch_q[s] = in[2 * s + 1];
		}
		sent = channelizer_tx_process(&sdr->band_tx, sdr->ch_i, sdr->ch_q, num, buff);
	}
	/* keep the sum within the range of the device (all carriers at full level) */
	for (s = 0; s < sent * 2; s++)
		buff[s] *= LIMIT_IQ_LEVEL;

	/* TX level diagnostics via status objects */
	if (channels) {
		double chan_tx_freq[SDR_STATUS_MAX_CHANNELS];
		for (c = 0; c < channels && c < SDR_STATUS_MAX_CHANNELS; c++)
			chan_tx_freq[c] = sdr->chan[c].tx_frequency;
		sdr_status_tx_accumulate(buff, sent, power, num, channels, sdr->amplitude, chan_tx_freq);
	}

	if (sdr->wave_tx_rec.fp) {
		sample_t *spl_list[2] = { sdr->wavespl0, sdr->wavespl1 };
		for (s = 0, ss = 0; s < sent; s++) {
			spl_list[0][s] = buff[ss++];
			spl_list[1][s] = buff[ss++];
		}
		wave_write(&sdr->wave_tx_rec, spl_list, sent);
	}
	if (sdr->wave_tx_play.fp) {
		sample_t *spl_list[2] = { sdr->wavespl0, sdr->wavespl1 };
		wave_read(&sdr->wave_tx_play, spl_list, sent);
		for (s = 0, ss = 0; s < sent; s++) {
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
			sdr->thread_write.max_fill = 0;
			sdr->thread_write.max_fill_timer += 1.0;
		}

		/* the ring holds SDR rate samples */
		if (space < sent * 2) {
			LOGP(DSDR, LOGL_ERROR, "Write SDR buffer overflow!\n");
			sent = space / 2;
		}
#ifdef DEBUG_BUFFER
		printf("Writing %d samples to write buffer.\n", sent);
#endif
		in = sdr->thread_write.in;
		{
			int first = (sdr->thread_write.buffer_size - in) / 2;	/* IQ pairs up to the end of the ring */

			if (first > sent)
				first = sent;
			/* the ring's volatile only orders the index updates (with
			 * __sync_synchronize); the samples are plain copies */
			memcpy((float *)sdr->thread_write.buffer + in, buff, first * 2 * sizeof(*buff));
			memcpy((float *)sdr->thread_write.buffer, buff + first * 2, (sent - first) * 2 * sizeof(*buff));
			in = (in + sent * 2) % sdr->thread_write.buffer_size;
		}
		__sync_synchronize();
		sdr->thread_write.in = in;
	}

	/* the caller counts in DSP rate samples */
	return num;
}

int sdr_read(void *inst, sample_t **samples, int num, int channels, double *rf_level_db)
{
	sdr_t *sdr = (sdr_t *)inst;
	float *lin = sdr->rx_lin;
	int count = 0, avail, need, take, used = 0, out;
	int c, s;

	/* on a backend: once a second, is it still there? Gone: the program
	 * ends (-EPERM: the sender loop quits) */
	if (sdr->use_client) {
		double t = get_time();

		if (t - sdr->client_check >= 1.0) {
			sdr->client_check = t;
			if (!sdr_client_alive(&sdr->client)) {
				LOGP(DSDR, LOGL_ERROR, "SDR backend '%s' is gone, ending.\n", sdr_config->backend);
				return -EPERM;
			}
		}
	}

	/* No RX in tx_only mode (e.g. rpitx). The sender loop of libmobile reads
	 * on every turn, so this is expected: nothing received. */
	if (sdr_config && sdr_config->tx_only)
		return 0;

	if (num > sdr->buffer_size) {
		fprintf(stderr, "exceeding maximum size given by sdr->buffer_size, please fix!\n");
		abort();
	}

	/* SDR rate IQ from the ring: exactly what num DSP samples need (the
	 * same for every channel: same rates, same stages), or what is there */
	__sync_synchronize();
	avail = ((sdr->thread_read.in - sdr->thread_read.out + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size) / 2;
	if (channels && sdr->use_engine) {
		/* the channels come from the engine; the raw ring only feeds the
		 * status and the displays: take what is there */
		need = avail;
	} else
		need = channelizer_rx_need(channels ? &sdr->chan[0].rx_ch : &sdr->band_rx, num);
	if (need > sdr->rx_max_in)
		need = sdr->rx_max_in;
	take = (avail < need) ? avail : need;
	out = sdr->thread_read.out;
	/* the band passes unchanged: straight from the ring to the caller, the
	 * status and the displays below see the same samples there */
	if (!channels && !sdr->band_client && sdr->band_rx_pass)
		lin = (float *)samples;
	{
		int first = (sdr->thread_read.buffer_size - out) / 2;	/* IQ pairs up to the end of the ring */

		if (first > take)
			first = take;
		memcpy(lin, (const float *)sdr->thread_read.buffer + out, first * 2 * sizeof(*lin));
		memcpy(lin + first * 2, (const float *)sdr->thread_read.buffer, (take - first) * 2 * sizeof(*lin));
	}
	used = take;

	if (channels) {
		int rows = 0;

		/* engine mode: the rows the device RX thread made, all channels */
		if (sdr->use_engine) {
			double now = get_time();

			rows = sdr_port_read(sdr->port, sdr->eng_i, sdr->eng_q, num);
			/* the engine's faults, once a second when they change */
			if (now - sdr->eng_report >= 1.0) {
				long missing = sdr->port->view->tx_missing, dropped = sdr->port->view->rx_dropped;

				if (missing != sdr->eng_missing || dropped != sdr->eng_dropped)
					LOGP(DSDR, LOGL_ERROR, "Filter bank engine: TX rows missing %ld, RX rows dropped %ld in the last %.1f s (total %ld / %ld)\n",
					     missing - sdr->eng_missing, dropped - sdr->eng_dropped, now - sdr->eng_report, missing, dropped);
				sdr->eng_missing = missing;
				sdr->eng_dropped = dropped;
				sdr->eng_report = now;
			}
		}
		for (c = 0; c < channels; c++) {
			sdr_chan_t *ch = &sdr->chan[c];
			int n;

			if (sdr->use_engine) {
				n = rows;
				memcpy(sdr->ch_i, sdr->eng_i[c], n * sizeof(*sdr->ch_i));
				memcpy(sdr->ch_q, sdr->eng_q[c], n * sizeof(*sdr->ch_q));
			} else
				n = channelizer_rx_process(&ch->rx_ch, lin, take, num, sdr->ch_i, sdr->ch_q);
			if (c == 0)
				count = n;
			else if (n != count)
				LOGP(DSDR, LOGL_ERROR, "RX channelizers out of step (%d / %d samples), please fix!\n", n, count);
			for (s = 0; s < n; s++) {
				sdr->modbuff[2 * s] = sdr->ch_i[s];
				sdr->modbuff[2 * s + 1] = sdr->ch_q[s];
			}
			if (rf_level_db)
				rf_level_db[c] = NAN;
			if (ch->am)
				am_demodulate_complex(&ch->am_demod, samples[c], n, sdr->modbuff, sdr->modbuff_I, sdr->modbuff_Q, sdr->modbuff_carrier);
			else
				fm_demodulate_complex(&ch->fm_demod, samples[c], n, sdr->modbuff, sdr->modbuff_I, sdr->modbuff_Q);
			sender_t *sender = get_sender_by_empfangsfrequenz(ch->rx_frequency);
			if (!sender || !n)
				continue;
			double min, max, avg;
			avg = 0.0;
			for (s = 0; s < n; s++) {
				/* average the square length of vector */
				avg += sdr->modbuff_I[s] * sdr->modbuff_I[s] + sdr->modbuff_Q[s] * sdr->modbuff_Q[s];
			}
			avg = sqrt(avg / (double)n); /* RMS */
			avg = log10(avg) * 20;
			display_measurements_update(ch->dmp_rf_level, avg, 0.0);
			if (rf_level_db)
				rf_level_db[c] = avg;
			if (!ch->am) {
				min = 0.0;
				max = 0.0;
				avg = 0.0;
				for (s = 0; s < n; s++) {
					avg += samples[c][s];
					if (s == 0 || samples[c][s] > max)
						max = samples[c][s];
					if (s == 0 || samples[c][s] < min)
						min = samples[c][s];
				}
				avg /= (double)n;
				display_measurements_update(ch->dmp_freq_offset, avg / 1000.0, 0.0);
				/* use half min and max, because we want the deviation above/below (+-) center frequency. */
				display_measurements_update(ch->dmp_deviation, min / 2.0 / 1000.0, max / 2.0 / 1000.0);
				sdr_status_rx_chan_update(c, rf_level_db ? rf_level_db[c] : avg, avg, max - min, ch->rx_frequency);
			}
		}
	} else {
		/* channels = 0: the whole band, interleaved float to the caller */
		float *o = (float *)samples;

		if (sdr->band_client) {
			/* on the backend: the channel's rows */
			count = sdr->port->n_rx ? sdr_port_read(sdr->port, sdr->eng_i, sdr->eng_q, num) : 0;
			for (s = 0; s < count; s++) {
				o[2 * s] = sdr->eng_i[0][s];
				o[2 * s + 1] = sdr->eng_q[0][s];
			}
		} else if (sdr->band_rx_pass) {
			/* in the caller's buffer already */
			count = take;
		} else {
			count = channelizer_rx_process(&sdr->band_rx, lin, take, num, sdr->ch_i, sdr->ch_q);
			for (s = 0; s < count; s++) {
				o[2 * s] = sdr->ch_i[s];
				o[2 * s + 1] = sdr->ch_q[s];
			}
		}
	}

	/* the ring advances by what the channelizers took */
	sdr->thread_read.out = (sdr->thread_read.out + 2 * used) % sdr->thread_read.buffer_size;

	if (sdr_rx_overflow) {
		LOGP(DSDR, LOGL_ERROR, "SDR RX overflow!\n");
		sdr_rx_overflow = 0;
	}

	/* IQ level, status, IQ and spectrum display of the SDR rate IQ */
	sdr_status_rx_accumulate(lin, used);
	sdr_status_snapshot(get_time());
	sdr_spectral_snapshot(get_time());
	display_iq(lin, used);
	display_spectrum(lin, used);

	return count;
}

/* Backend: drain the raw SDR rate ring that the RX thread fills beside the
 * engine (no program reads it there): the RX status (clipping, DC, levels)
 * and the overflow report. No displays (the backend has no terminal UI).
 * Call it from the backend's main loop, more often than the ring fills
 * (4 x buffer_size); a full ring only cuts what the status sees. */
void sdr_server_poll(void *inst)
{
	sdr_t *sdr = (sdr_t *)inst;
	int avail, take, first, out;

	if (!sdr || !sdr->rx_lin || !sdr->thread_read.buffer)
		return;
	while (1) {
		__sync_synchronize();
		avail = ((sdr->thread_read.in - sdr->thread_read.out + sdr->thread_read.buffer_size) % sdr->thread_read.buffer_size) / 2;
		if (!avail)
			break;
		take = (avail > sdr->rx_max_in) ? sdr->rx_max_in : avail;
		out = sdr->thread_read.out;
		first = (sdr->thread_read.buffer_size - out) / 2;
		if (first > take)
			first = take;
		memcpy(sdr->rx_lin, (const float *)sdr->thread_read.buffer + out, first * 2 * sizeof(*sdr->rx_lin));
		memcpy(sdr->rx_lin + first * 2, (const float *)sdr->thread_read.buffer, (take - first) * 2 * sizeof(*sdr->rx_lin));
		__sync_synchronize();	/* copied before the RX thread may reuse it */
		sdr->thread_read.out = (out + 2 * take) % sdr->thread_read.buffer_size;
		sdr_status_rx_accumulate(sdr->rx_lin, take);
	}
	if (sdr_rx_overflow) {
		LOGP(DSDR, LOGL_ERROR, "SDR RX overflow!\n");
		sdr_rx_overflow = 0;
	}
	/* ADC clipping hurts every program's channels: tell the operator when
	 * it starts, every 10 s while it lasts, and when it ends */
	{
		static int clipping = 0;
		static double clip_report = 0.0;
		double now = get_time();
		const sdr_rx_status_t *rx = &sdr_status_get()->rx;

		if (sdr_status_snapshot(now) && rx->valid && rx->sample_count > 0) {
			if (rx->clip_soft > 0 && (!clipping || now - clip_report >= 10.0)) {
				LOGP(DSDR, LOGL_NOTICE, "RX clipping: %.2f %% of the samples near full scale (%.2f %% at the ADC rail), rms %.1f dBFS, DC %.3f / %.3f: lower --sdr-rx-gain.\n",
				     100.0 * rx->clip_soft / rx->sample_count, 100.0 * rx->clip_hard / rx->sample_count,
				     (rx->rms > 0.0) ? 20.0 * log10(rx->rms) : -120.0, rx->dc_offset_i, rx->dc_offset_q);
				clip_report = now;
			}
			if (!rx->clip_soft && clipping)
				LOGP(DSDR, LOGL_NOTICE, "RX clipping ended.\n");
			clipping = (rx->clip_soft > 0);
		}
	}
}

/* TX driver has no device time reference, so its send() backpressure is the
 * only clock that matches the DAC rate. */
/* The SDR cannot go on (a TX transport broken): the exit code the program
 * should end with, 0: it works */
int sdr_fatal(void)
{
#ifdef HAVE_UHD
	if (tx_driver == 1 && uhd_tx_dead())
		return UHD_EXIT_TX_DEAD;
#endif
	return 0;
}

static sdr_clock_status_t clock_status;
static double clock_polled;

void sdr_clock_poll(void)
{
#ifdef HAVE_UHD
	struct timespec ts;
	double now;

	if (tx_driver != 1)
		return;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	now = ts.tv_sec + ts.tv_nsec / 1e9;
	if (clock_status.valid && now - clock_polled < 1.0)
		return;
	clock_polled = now;
	uhd_clock_poll(&clock_status);
#endif
}

int sdr_clock_status(sdr_clock_status_t *status)
{
	if (!clock_status.valid)
		return -1;
	*status = clock_status;
	return 0;
}

static int sdr_tx_device_paced(void)
{
#ifdef HAVE_UHD
	if (tx_driver == 1)
		return uhd_tx_device_paced();
#endif
#ifdef HAVE_SOAPY
	if (tx_driver == 2)
		return soapy_tx_device_paced();
#endif
#ifdef HAVE_RPITX
	/* the blocking write into the DMA FIFO is the only clock */
	if (tx_driver == 3)
		return rpitx_tx_device_paced();
#endif
	return 0;
}

/* how much do we need to send (in audio sample duration) to get the target delay (buffer size) */
static void apply_rx_bandwidth(sdr_t *sdr, int channel);

/* RX frequency trim of one channel (Hz, added to the channel's frequency):
 * the FM demodulator's oscillator moves the channel to 0 Hz before its
 * lowpass (fm_demod_set_offset()), so the channel filter follows the signal.
 * For a decoder that measures the carrier offset (AFC). Applies to the
 * samples demodulated from the next sdr_read() on. Limited so the channel
 * stays in the usable DSP band (0.4 of the DSP rate each side); returns the
 * trim set. Main thread (as sdr_read()). */
double sdr_set_rx_trim(void *inst, int channel, double hz)
{
	sdr_t *sdr = (sdr_t *)inst;
	double max;

	if (!sdr || channel < 0 || channel >= sdr->channels || !sdr->chan[channel].rx_init || sdr->chan[channel].am)
		return 0.0;
	max = 0.4 * sdr->samplerate - sdr->bandwidth / 2.0;
	if (max < 0.0)
		max = 0.0;
	if (hz > max)
		hz = max;
	if (hz < -max)
		hz = -max;
	if (hz != sdr->chan[channel].rx_trim) {
		fm_demod_set_offset(&sdr->chan[channel].fm_demod, hz);
		sdr->chan[channel].rx_trim = hz;
		/* a wide filter moves with the channel: keep it off the centre */
		if (sdr->chan[channel].rx_bandwidth_req > 0.0)
			apply_rx_bandwidth(sdr, channel);
	}
	return hz;
}

/* the RX channel filter asked for, limited: wider than Carson only as far as
 * the filter stays off the SDR centre (its DC spike / LO leakage, the
 * channel placement keeps Carson off it) by RX_DC_GUARD, and within the
 * usable DSP band */
#define RX_DC_GUARD	1000.0
static void apply_rx_bandwidth(sdr_t *sdr, int channel)
{
	sdr_chan_t *ch = &sdr->chan[channel];
	double hz = ch->rx_bandwidth_req, room;

	if (hz <= 0.0)
		hz = sdr->bandwidth;
	if (hz > sdr->bandwidth) {
		room = 2.0 * (fabs(ch->rx_offset + ch->rx_trim) - RX_DC_GUARD);
		if (hz > room)
			hz = room;
		if (hz < sdr->bandwidth)
			hz = sdr->bandwidth;
	}
	if (hz > 0.8 * sdr->samplerate)
		hz = 0.8 * sdr->samplerate;
	if (hz != ch->rx_bandwidth) {
		fm_demod_set_bandwidth(&ch->fm_demod, hz);
		ch->rx_bandwidth = hz;
	}
}

/* RX channel filter bandwidth of one channel (Hz, both sidebands), for a
 * decoder that hunts with a wide filter and receives with a narrow one
 * (AFC); 0 = the channel's own (Carson) bandwidth. Limited (see
 * apply_rx_bandwidth()). Applies from the next sdr_read() on. Main thread. */
void sdr_set_rx_bandwidth(void *inst, int channel, double hz)
{
	sdr_t *sdr = (sdr_t *)inst;

	if (!sdr || channel < 0 || channel >= sdr->channels || !sdr->chan[channel].rx_init || sdr->chan[channel].am)
		return;
	sdr->chan[channel].rx_bandwidth_req = hz;
	apply_rx_bandwidth(sdr, channel);
}

int sdr_get_tosend(void *inst, int buffer_size)
{
	sdr_t *sdr = (sdr_t *)inst;
	int count = 0;
	double ratio;	/* DSP samples per SDR sample */

	/* RX-only: nothing to send. The sender loop asks every round, this is not an error (it logged
	 * an error here every millisecond, 12 MB of log per minute). */
	if (sdr_config && sdr_config->rx_only)
		return 0;
	ratio = (double)sdr->samplerate / sdr->tx_rate;

	/* engine mode: the group's TX ring (DSP samples) kept at buffer_size;
	 * the device TX thread takes from it at the device clock */
	if (sdr->use_engine && sdr->port && sdr->port->n_tx) {
		/* the device's advance is part of the buffer */
		count = buffer_size - (sdr->buffer_size - sdr->eng_ring_target) - sdr_port_tx_fill(sdr->port);
		return (count > 0) ? count : 0;
	}

	/* Device paced: keep the write ring filled to buffer_size. The write
	 * thread drains it through the blocking send(), i.e. at the device
	 * sample clock, so production follows that clock and cannot drift. */
	if (sdr->threads && sdr_tx_device_paced()) {
		int fill = (sdr->thread_write.in - sdr->thread_write.out + sdr->thread_write.buffer_size) % sdr->thread_write.buffer_size;
		static int logged = 0;

		if (!logged) {
			LOGP(DSDR, LOGL_NOTICE, "TX pacing: device sample clock (send backpressure), ring target %d samples (%.1f ms)\n",
			     buffer_size, 1000.0 * buffer_size / sdr->samplerate);
			logged = 1;
		}
		/* the ring holds SDR rate samples */
		count = buffer_size - (int)ceil(fill / 2 * ratio);
		return (count > 0) ? count : 0;
	}

#ifdef HAVE_UHD
	if (tx_driver == 1)
		count = uhd_get_tosend((int)(buffer_size / ratio));
#endif
#ifdef HAVE_SOAPY
	if (tx_driver == 2)
		count = soapy_get_tosend((int)(buffer_size / ratio));
#endif
#ifdef HAVE_RPITX
	if (tx_driver == 3)
		count = rpitx_get_tosend((int)(buffer_size / ratio));
#endif
	if (count < 0)
		return count;
	/* subtract what is in the ring, not sent to the SDR yet */
	{
		int fill = (sdr->thread_write.in - sdr->thread_write.out + sdr->thread_write.buffer_size) % sdr->thread_write.buffer_size;

		count -= fill / 2;
	}
	/* SDR rate -> DSP rate, rounding down, so we never overfill */
	count = (int)floor(count * ratio);
	if (count < 0)
		count = 0;
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


