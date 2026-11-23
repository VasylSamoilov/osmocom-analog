/* Tests of libchannelizer: halfband, rational resampler, channelizer (RX) and
 * synthesizer (TX), measured with tones. See docs/CHANNELIZER.md. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include "../libsample/sample.h"
#include "../libchannelizer/halfband.h"
#include "../libchannelizer/resamp.h"
#include "../libchannelizer/channelizer.h"
#include "../libchannelizer/fftc.h"
#include "../libchannelizer/pfb.h"

static int failures = 0;

static void check(int ok, const char *what, double value, const char *unit)
{
	printf("  %-62s %9.2f %-3s %s\n", what, value, unit, ok ? "ok" : "FAIL");
	if (!ok)
		failures++;
}

/* level (dB re 1) of the component at f (cycles per sample) in x, Hann window */
static double level(const double complex *x, int n, double f)
{
	double complex acc = 0;
	double wsum = 0;
	int k;

	for (k = 0; k < n; k++) {
		double w = 0.5 - 0.5 * cos(2 * M_PI * k / (n - 1));

		acc += w * x[k] * cexp(-2 * M_PI * I * f * k);
		wsum += w;
	}
	return 20 * log10(cabs(acc) / wsum + 1e-300);
}

/* strongest component within |f| <= fmax (cycles per sample), on a grid of
 * 1/2048 anchored at 0 */
static double strongest(const double complex *x, int n, double fmax, double *f_at)
{
	double best = -1e9, f;
	int k, kmax = (int)(fmax * 2048);

	*f_at = 0;
	for (k = -kmax; k <= kmax; k++) {
		double l;

		f = k / 2048.0;
		l = level(x, n, f);

		if (l > best) {
			best = l;
			*f_at = f;
		}
	}
	return best;
}

/* Each halfband as designed for a stage: passband 0 .. p (fraction of the
 * higher rate), stopband 0.5 - p .. 0.5. Measured with tones through the
 * block functions, in blocks of odd length (state across calls). */
static void test_halfband(void)
{
	static const double passes[] = { 0.005, 0.02, 0.08, 0.15, 0.2 };
	static double complex y[16384];
	static sample_t bi[16384], bq[16384], oi[32768], oq[32768];
	halfband_t hb;
	unsigned o;

	printf("Halfband 2x decimation and interpolation (designed per stage, %.0f dB):\n", CHANNELIZER_ATTEN_DB);
	for (o = 0; o < sizeof(passes) / sizeof(*passes); o++) {
		double p = passes[o], worst = 1e9, flat = 0, f;
		char what[100];
		int k;

		if (halfband_design(&hb, p, CHANNELIZER_ATTEN_DB) < 0) {
			printf("  passband %.3f: no design\n", p);
			failures++;
			continue;
		}
		int order = hb.order;
		halfband_exit(&hb);

		/* decimation: tones in the passband (level) and in the stopband (alias) */
		for (k = 0; k <= 40; k++) {
			int n = 0, m = 0, step = 333, s;

			f = (k <= 20) ? p * k / 20 : 0.5 - p + p * (k - 21) / 19;
			halfband_design(&hb, p, CHANNELIZER_ATTEN_DB);
			while (n < 4096) {
				for (s = 0; s < step; s++, m++) {
					double complex x = 0.5 * cexp(2 * M_PI * I * f * m);

					bi[s] = creal(x);
					bq[s] = cimag(x);
				}
				int got = halfband_decimate_block(&hb, bi, bq, step);

				for (s = 0; s < got && n < 4096; s++)
					y[n++] = (bi[s] + I * bq[s]) / 0.5;
			}
			halfband_exit(&hb);
			double fo = fmod(2 * f + 0.5, 1.0) - 0.5, l = level(y + 200, 3800, fo);

			if (k <= 20 && fabs(l) > flat)
				flat = fabs(l);
			if (k > 20 && -l < worst)
				worst = -l;
		}
		snprintf(what, sizeof(what), "pass %.3f (order %d) decimation: passband error", p, order);
		check(flat < 0.01, what, flat, "dB");
		snprintf(what, sizeof(what), "pass %.3f (order %d) decimation: alias rejection", p, order);
		check(worst >= CHANNELIZER_ATTEN_DB - 1.0, what, worst, "dB");

		/* interpolation: input tone at f (of the lower rate, |f| <= 2p) gives
		 * the wanted line at f/2 and the image at f/2 - 0.5 */
		worst = 1e9;
		flat = 0;
		for (k = -10; k <= 10; k++) {
			int m = 0, s, done = 0, step = 333;

			f = 2 * p * k / 10;
			halfband_design(&hb, p, CHANNELIZER_ATTEN_DB);
			while (done < 8192) {
				for (s = 0; s < step; s++, m++) {
					double complex x = cexp(2 * M_PI * I * f * m);

					bi[s] = creal(x);
					bq[s] = cimag(x);
				}
				halfband_interpolate_block(&hb, bi, bq, step, oi, oq);
				for (s = 0; s < 2 * step && done < 8192; s++)
					y[done++] = oi[s] + I * oq[s];
			}
			halfband_exit(&hb);
			double want = level(y + 400, 7600, f / 2), image = level(y + 400, 7600, f / 2 - 0.5);

			if (fabs(want) > flat)
				flat = fabs(want);
			if (want - image < worst)
				worst = want - image;
		}
		snprintf(what, sizeof(what), "pass %.3f (order %d) interpolation: gain error", p, order);
		check(flat < 0.01, what, flat, "dB");
		snprintf(what, sizeof(what), "pass %.3f (order %d) interpolation: image rejection", p, order);
		check(worst >= CHANNELIZER_ATTEN_DB - 1.0, what, worst, "dB");
	}
}

/* RX: tone at f_in (Hz, relative to the SDR centre) -> output: strongest
 * component within fmax (cycles per DSP sample) and its frequency (Hz); if
 * at_l, the level at the frequency where the tone belongs (f_in - off) */
static double rx_tone(int sdr, int dsp, double off, double bw, double f_in, double fmax, double *f_at, double *at_l)
{
	channelizer_t ch;
	int nin = (int)((double)sdr / dsp * 4096), n, k, s, step;
	float *iq = malloc(sizeof(*iq) * 2 * nin);
	sample_t *oi = malloc(sizeof(*oi) * nin), *oq = malloc(sizeof(*oq) * nin);
	double complex *y = malloc(sizeof(*y) * nin);
	double l;

	for (k = 0; k < nin; k++) {
		double complex z = 0.5 * cexp(2 * M_PI * I * f_in / sdr * k);

		iq[2 * k] = creal(z);
		iq[2 * k + 1] = cimag(z);
	}
	/* in blocks as sdr_read() gives them: the input needed for 'step'
	 * output samples, so block borders fall at every phase */
	step = 97;
	channelizer_rx_init(&ch, sdr, dsp, off, bw, 2 * step * sdr / dsp + 64);
	for (n = 0, s = 0; ; ) {
		int need = channelizer_rx_need(&ch, step), got;

		if (s + need > nin)
			break;
		got = channelizer_rx_process(&ch, iq + 2 * s, need, step, oi + n, oq + n);
		if (got != step) {
			printf("  channelizer_rx_need() input gave %d samples instead of %d\n", got, step);
			failures++;
		}
		n += got;
		s += need;
	}
	for (k = 0; k < n; k++)
		y[k] = (oi[k] + I * oq[k]) / 0.5;
	l = strongest(y + 600, n - 600, fmax, f_at);
	*f_at *= dsp;
	if (at_l)
		*at_l = level(y + 600, n - 600, (f_in - off) / dsp);
	channelizer_exit(&ch);
	free(iq);
	free(oi);
	free(oq);
	free(y);
	return l;
}

static void test_rx(int sdr, int dsp, double off, double bw)
{
	double worst = -999, worst_f = 0, f, f_at, l, err = 0;
	char what[100];

	printf("Channelizer RX %d -> %d S/s, channel at %+.0f Hz, bandwidth %.0f Hz:\n", sdr, dsp, off, bw);
	/* the channel: level and frequency */
	for (f = -bw / 2; f <= bw / 2; f += bw / 6) {
		double at;

		rx_tone(sdr, dsp, off, bw, off + f, bw / 2 / dsp + 0.01, &f_at, &at);
		if (fabs(at) > err)
			err = fabs(at);
		if (fabs(f_at - f) > (double)dsp / 2048 * 1.5) {
			printf("  tone %+.0f Hz came out at %+.0f Hz\n", f, f_at);
			failures++;
		}
	}
	check(err < 0.01, "level error in the channel (7 tones across the bandwidth)", err, "dB");
	/* everything else in the SDR band: nothing in the whole DSP band */
	for (f = -sdr / 2.0 + 997; f < sdr / 2.0; f += sdr / 41.0) {
		if (fabs(f - off) < 0.6 * dsp)
			continue;
		l = rx_tone(sdr, dsp, off, bw, f, 0.5, &f_at, NULL);
		if (l > worst) {
			worst = l;
			worst_f = f;
		}
	}
	snprintf(what, sizeof(what), "rejection of other frequencies, whole DSP band (worst %+.0f Hz)", worst_f);
	check(worst <= -100.0, what, worst, "dB");
}

static void test_tx(int sdr, int dsp, double off, double bw)
{
	channelizer_t ch;
	int n = 4096, k, total;
	sample_t *i = malloc(sizeof(*i) * n), *q = malloc(sizeof(*q) * n);
	float *iq;
	double complex *y;
	double f = 3000.0, want, worst = -999, g, worst_g = 0;
	char what[100];

	printf("Synthesizer TX %d <- %d S/s, channel at %+.0f Hz:\n", sdr, dsp, off);
	channelizer_tx_init(&ch, sdr, dsp, off, bw, n);
	iq = calloc(2 * channelizer_tx_max(&ch, n), sizeof(*iq));
	for (k = 0; k < n; k++) {
		double complex z = 0.5 * cexp(2 * M_PI * I * f / dsp * k);

		i[k] = creal(z);
		q[k] = cimag(z);
	}
	total = channelizer_tx_process(&ch, i, q, n, iq);
	/* over many calls the SDR samples follow the exact ratio (each call is
	 * within the filter start and the rounding of a non-integer ratio) */
	{
		channelizer_t c2;
		float *tmp = calloc(2 * channelizer_tx_max(&ch, n), sizeof(*tmp));
		long sum = 0;
		int b;

		/* the count moves in steps of 2^k SDR samples (one sample before the
		 * halfbands): its deviation from the exact ratio stays within one
		 * step, it never grows */
		double dev, worst = 0;

		channelizer_tx_init(&c2, sdr, dsp, off, bw, n);
		for (b = 0; b < 200; b++) {
			sum += channelizer_tx_process(&c2, i, q, n, tmp);
			dev = fabs((b + 1.0) * n * sdr / dsp - sum);
			if (dev > worst)
				worst = dev;
		}
		check(worst <= (1 << c2.hb_stages), "SDR sample count over 200 calls: worst deviation from the ratio", worst, "");
		channelizer_exit(&c2);
		free(tmp);
	}
	int skip = total / 8;
	y = malloc(sizeof(*y) * total);
	for (k = 0; k < total - skip; k++)
		y[k] = (iq[2 * (k + skip)] + I * iq[2 * (k + skip) + 1]) / 0.5;
	want = level(y, total - skip, (off + f) / sdr);
	check(fabs(want) < 0.01, "level of the wanted line", want, "dB");
	for (g = -sdr / 2.0 + 500; g < sdr / 2.0; g += sdr / 997.0) {
		if (fabs(g - (off + f)) < 2000)
			continue;
		double l = level(y, total - skip, g / sdr);

		if (l > worst) {
			worst = l;
			worst_g = g;
		}
	}
	snprintf(what, sizeof(what), "strongest other line, whole SDR band (at %+.0f Hz)", worst_g);
	check(worst <= -100.0, what, worst, "dB");
	channelizer_exit(&ch);
	free(i);
	free(q);
	free(iq);
	free(y);
}

/*
 * FFT and filter banks
 */

static void test_fftc(void)
{
	static const int sizes[] = { 8, 16, 32, 64, 128, 256, 1024 };
	unsigned s;
	double worst = 0;

	printf("FFT (fftc) against the direct DFT:\n");
	for (s = 0; s < sizeof(sizes) / sizeof(*sizes); s++) {
		int n = sizes[s], k, j, dir;

		for (dir = -1; dir <= 1; dir += 2) {
			fftc_t f;
			double *re = malloc(n * sizeof(double)), *im = malloc(n * sizeof(double));
			double complex *x = malloc(n * sizeof(*x));

			fftc_init(&f, n, dir);
			for (k = 0; k < n; k++) {
				x[k] = (rand() / (double)RAND_MAX - 0.5) + I * (rand() / (double)RAND_MAX - 0.5);
				re[k] = creal(x[k]);
				im[k] = cimag(x[k]);
			}
			fftc_process(&f, re, im);
			for (k = 0; k < n; k++) {
				double complex y = 0;

				for (j = 0; j < n; j++)
					y += x[j] * cexp(dir * 2 * M_PI * I * (double)((long)j * k % n) / n);
				if (cabs(y - (re[k] + I * im[k])) > worst)
					worst = cabs(y - (re[k] + I * im[k]));
			}
			fftc_exit(&f);
			free(re);
			free(im);
			free(x);
		}
	}
	check(worst < 1e-12, "sizes 8 .. 1024, both directions: worst error (x 1e15)", worst * 1e15, "");
}

/* the RX bank against its formula; random block borders */
static void test_pfb_rx_bank(int fs, int m, double clean)
{
	pfb_t pfb;
	int n, s, f, c, frames, got = 0, pos = 0, kk[3] = { 0, 5, m - 3 };
	float *iq;
	sample_t *oi[3], *oq[3], *pi[3], *pq[3];
	double worst = 0, worst2 = 0;
	char what[100];

	printf("RX filter bank %d S/s, %d bins, clean +-%.0f Hz:\n", fs, m, clean);
	if (pfb_rx_init(&pfb, fs, m, clean) < 0) {
		printf("  %s\n", pfb_error());
		failures++;
		return;
	}
	n = pfb.len * 4;
	iq = malloc(sizeof(float) * 2 * n);
	for (c = 0; c < 3; c++) {
		oi[c] = malloc(sizeof(sample_t) * (n / pfb.d + 1));
		oq[c] = malloc(sizeof(sample_t) * (n / pfb.d + 1));
		pi[c] = malloc(sizeof(sample_t) * (n / pfb.d + 1));
		pq[c] = malloc(sizeof(sample_t) * (n / pfb.d + 1));
	}
	for (s = 0; s < 2 * n; s++)
		iq[s] = rand() / (double)RAND_MAX - 0.5;
	frames = pfb_rx_process(&pfb, iq, n, 3, kk, oi, oq);
	/* the reversed rows back to h: h[pM + r] = hr[pM + M-1-r] */
	for (f = 0; f < frames; f++) {
		for (c = 0; c < 3; c++) {
			double complex acc = 0;
			int l, nn = f * pfb.d + pfb.d - 1;

			for (l = 0; l < pfb.len && nn - l >= 0; l++) {
				int p = l / m, r = l % m;
				double h = pfb.h[p * m + m - 1 - r];
				double complex x = iq[2 * (nn - l)] + I * iq[2 * (nn - l) + 1];

				acc += h * x * cexp(-2 * M_PI * I * kk[c] * (double)(nn - l) / m);
			}
			if (cabs(acc - (oi[c][f] + I * oq[c][f])) > worst)
				worst = cabs(acc - (oi[c][f] + I * oq[c][f]));
		}
	}
	snprintf(what, sizeof(what), "bank against the formula (%d frames, %d taps per branch): error (x 1e12)", frames, pfb.t);
	check(worst < 1e-12, what, worst * 1e12, "");
	pfb_exit(&pfb);
	pfb_rx_init(&pfb, fs, m, clean);
	while (pos < n) {
		int b = 1 + rand() % (3 * pfb.d);
		sample_t *ti[3], *tq[3];

		if (b > n - pos)
			b = n - pos;
		for (c = 0; c < 3; c++) {
			ti[c] = pi[c] + got;
			tq[c] = pq[c] + got;
		}
		got += pfb_rx_process(&pfb, iq + 2 * pos, b, 3, kk, ti, tq);
		pos += b;
	}
	for (f = 0; f < got && f < frames; f++)
		for (c = 0; c < 3; c++)
			worst2 = fmax(worst2, cabs((pi[c][f] - oi[c][f]) + I * (pq[c][f] - oq[c][f])));
	check(got == frames && worst2 == 0.0, "random blocks of 1 .. 3 M/2 samples: same frames, difference", worst2, "");
	pfb_exit(&pfb);
	free(iq);
	for (c = 0; c < 3; c++) {
		free(oi[c]);
		free(oq[c]);
		free(pi[c]);
		free(pq[c]);
	}
}

/* RX through bank and channel: tone at f_in -> DSP samples (blocks of
 * 'step' as sdr_read asks for them, each exactly 'step'); y gets them */
static int pfb_rx_tone(int fs, int m, int dsp, double off, double f_in, int nout, double complex *y)
{
	pfb_t pfb;
	pfb_chan_t ch;
	int step = 97, n = 0, s, k, bin, frames_max;
	long t = 0;
	sample_t *bi, *bq, *oi, *oq;
	float *iq;

	pfb_rx_init(&pfb, fs, m, 0.5 * dsp);
	frames_max = 2 * step * pfb_bin_rate(&pfb) / dsp + 16;
	if (pfb_chan_rx_init(&ch, &pfb, off, dsp, frames_max) < 0) {
		printf("  %s\n", pfb_error());
		failures++;
		pfb_exit(&pfb);
		return 0;
	}
	bin = ch.bin;
	bi = malloc(sizeof(*bi) * frames_max);
	bq = malloc(sizeof(*bq) * frames_max);
	oi = malloc(sizeof(*oi) * step);
	oq = malloc(sizeof(*oq) * step);
	iq = malloc(sizeof(float) * 2 * (pfb_rx_need(&pfb, frames_max) + 1));
	while (n + step <= nout) {
		int frames = pfb_chan_rx_need(&ch, step), need = pfb_rx_need(&pfb, frames), got, out;

		for (s = 0; s < need; s++, t++) {
			double complex z = 0.5 * cexp(2 * M_PI * I * f_in / fs * t);

			iq[2 * s] = creal(z);
			iq[2 * s + 1] = cimag(z);
		}
		got = pfb_rx_process(&pfb, iq, need, 1, &bin, &bi, &bq);
		out = pfb_chan_rx_process(&ch, bi, bq, got, step, oi, oq);
		if (got != frames || out != step) {
			printf("  pfb_chan_rx_need(): %d frames (wanted %d), %d samples (wanted %d)\n", got, frames, out, step);
			failures++;
			break;
		}
		for (k = 0; k < out; k++)
			y[n + k] = (oi[k] + I * oq[k]) / 0.5;
		n += out;
	}
	pfb_chan_exit(&ch);
	pfb_exit(&pfb);
	free(bi);
	free(bq);
	free(oi);
	free(oq);
	free(iq);
	return n;
}

static void test_pfb_rx(int fs, int m, int dsp, double off)
{
	int nout = dsp / 10, n, k;
	double complex *y = malloc(sizeof(*y) * nout);
	double err = 0, worst = -999, worst_f = 0, f, g;
	double sp = (double)fs / m, fb = 2.0 * sp;
	double crit[] = { off + 25000, off - 25000, off + 30000, off - 30000, off + 0.5 * dsp + 1000, off - 0.5 * dsp - 1000,
			  off + dsp, off - dsp, off + 1.5 * dsp, off - 1.5 * dsp, off + sp, off - sp, off + fb - 0.3 * dsp, off - fb + 0.3 * dsp,
			  off + fb, off - fb, off + 2 * dsp, off - 2 * dsp };
	char what[120];

	printf("RX bank + channel %d -> %d S/s, %d bins, channel at %+.0f Hz:\n", fs, dsp, m, off);
	for (k = -3; k <= 3; k++) {
		f = 0.4 * dsp * k / 3;
		n = pfb_rx_tone(fs, m, dsp, off, off + f, nout, y);
		if (n < nout / 2)
			return;
		double l = level(y + n / 10, n - n / 10, f / dsp), l2 = level(y + n / 10, n - n / 10, (f + 100.0) / dsp);

		if (fabs(l) > err)
			err = fabs(l);
		if (l2 > l - 20.0) {
			printf("  tone %+.0f Hz not at its frequency\n", f);
			failures++;
		}
	}
	check(err < 0.01, "level error in the channel (7 tones to 0.4 of the DSP rate)", err, "dB");
	for (k = 0; k < 61 + (int)(sizeof(crit) / sizeof(*crit)); k++) {
		f = (k < 61) ? -fs / 2.0 + 777 + k * (fs / 61.0) : crit[k - 61];
		if (fabs(f - off) < 0.5 * dsp + 1000)
			continue;
		n = pfb_rx_tone(fs, m, dsp, off, f, nout, y);
		for (g = -0.5 * dsp; g <= 0.5 * dsp; g += dsp / 64.0) {
			double l = level(y + n / 10, n - n / 10, g / dsp);

			if (l > worst) {
				worst = l;
				worst_f = f - off;
			}
		}
	}
	snprintf(what, sizeof(what), "61 tones + neighbours, band edge, alias points: worst in the DSP band (%+.0f)", worst_f);
	check(worst <= -100.0, what, worst, "dB");
	free(y);
}

/* TX: channels at off[] with inputs in[] (nd DSP samples, blocks of 97)
 * through channels and bank; returns the SDR samples in *out */
static int pfb_tx_run(int fs, int m, int dsp, int nch, const double *off, double complex **in, int nd, float **out)
{
	pfb_t pfb;
	pfb_chan_t *ch = calloc(nch, sizeof(*ch));
	const sample_t **ci = malloc(sizeof(*ci) * nch), **cq = malloc(sizeof(*cq) * nch);
	sample_t *xi = malloc(sizeof(sample_t) * 97), *xq = malloc(sizeof(sample_t) * 97);
	int *bin = malloc(sizeof(int) * nch), c, s, k, total = 0, cap;
	float *iq;

	pfb_tx_init(&pfb, fs, m, 0.5 * dsp);
	cap = (int)((double)nd * fs / dsp) + 4 * fs / 100 + 100000;
	iq = calloc(2 * (size_t)cap, sizeof(float));
	for (c = 0; c < nch; c++) {
		if (pfb_chan_tx_init(&ch[c], &pfb, off[c], dsp, 97) < 0) {
			printf("  %s\n", pfb_error());
			failures++;
		}
		bin[c] = ch[c].bin;
	}
	for (s = 0; s + 97 <= nd; s += 97) {
		int frames = -1, fr;

		for (c = 0; c < nch; c++) {
			for (k = 0; k < 97; k++) {
				xi[k] = creal(in[c][s + k]);
				xq[k] = cimag(in[c][s + k]);
			}
			fr = pfb_chan_tx_process(&ch[c], xi, xq, 97);
			if (frames >= 0 && fr != frames) {
				printf("  TX channels out of step (%d / %d frames)\n", fr, frames);
				failures++;
			}
			frames = fr;
			ci[c] = ch[c].out_i;
			cq[c] = ch[c].out_q;
		}
		if (total + frames * pfb.d > cap)
			break;
		total += pfb_tx_process(&pfb, nch, bin, ci, cq, frames, iq + 2 * total);
	}
	for (c = 0; c < nch; c++)
		pfb_chan_exit(&ch[c]);
	pfb_exit(&pfb);
	free(ch);
	free(ci);
	free(cq);
	free(xi);
	free(xq);
	free(bin);
	*out = iq;
	return total;
}

static double level_iq(const float *iq, int n, double f)
{
	double complex acc = 0;
	double wsum = 0;
	int k;

	for (k = 0; k < n; k++) {
		double w = 0.5 - 0.5 * cos(2 * M_PI * k / (n - 1));

		acc += w * (iq[2 * k] + I * iq[2 * k + 1]) * cexp(-2 * M_PI * I * f * k);
		wsum += w;
	}
	return 20 * log10(cabs(acc) / wsum + 1e-300);
}

static void test_pfb_tx(int fs, int m, int dsp, double off)
{
	int nd = dsp / 20, s, len, skip, k;
	double sp = (double)fs / m, fb = 2.0 * sp, worst, at = 0, f;
	double complex *in0 = malloc(sizeof(*in0) * nd), *in1 = malloc(sizeof(*in1) * nd), *ins[2] = { in0, in1 };
	float *iq;
	char what[120];

	printf("TX channel + bank %d <- %d S/s, %d bins, channel at %+.0f Hz:\n", fs, dsp, m, off);
	/* two tones */
	{
		double offs[1] = { off };
		double crit[] = { off + 3000 + fb, off + 3000 - fb, off - 5000 + fb, off - 5000 - fb, off + 3000 - dsp, off - 5000 + dsp,
				  off + 3000 - 2 * dsp, off - 5000 + 2 * dsp, off + sp, off - sp, off + 0.5 * dsp + 500, off - 0.5 * dsp - 500,
				  -(off + 3000), -(off - 5000) };

		for (s = 0; s < nd; s++)
			in0[s] = 0.25 * cexp(2 * M_PI * I * 3000.0 / dsp * s) + 0.125 * cexp(-2 * M_PI * I * 5000.0 / dsp * s);
		len = pfb_tx_run(fs, m, dsp, 1, offs, ins, nd, &iq);
		skip = len / 8;
		double l1 = level_iq(iq + 2 * skip, len - skip, (off + 3000.0) / fs) - 20 * log10(0.25);
		double l2 = level_iq(iq + 2 * skip, len - skip, (off - 5000.0) / fs) - 20 * log10(0.125);
		check(fabs(l1) < 0.01 && fabs(l2) < 0.01, "levels of two tones (+3 kHz, -5 kHz), worst", fmax(fabs(l1), fabs(l2)), "dB");
		worst = -999;
		for (k = 0; k < 211 + (int)(sizeof(crit) / sizeof(*crit)); k++) {
			f = (k < 211) ? -fs / 2.0 + 313 + k * (fs / 211.0) : crit[k - 211];
			if (fabs(f - off) < 0.5 * dsp)
				continue;
			double l = level_iq(iq + 2 * skip, len - skip, f / fs) - 20 * log10(0.25);
			if (l > worst) {
				worst = l;
				at = f - off;
			}
		}
		snprintf(what, sizeof(what), "strongest other line, whole SDR band (211 + images, bins; %+.0f)", at);
		check(worst <= -100.0, what, worst, "dBc");
		free(iq);
	}
	/* a tone at 0.42 of the DSP rate: its images must not be sent */
	{
		double offs[1] = { off }, ft = 0.42 * dsp, crit[] = { off + ft - dsp, off + ft - 2 * dsp, off + ft + dsp, off - ft };

		for (s = 0; s < nd; s++)
			in0[s] = 0.5 * cexp(2 * M_PI * I * ft / dsp * s);
		len = pfb_tx_run(fs, m, dsp, 1, offs, ins, nd, &iq);
		skip = len / 8;
		worst = -999;
		for (k = 0; k < 4; k++) {
			double l = level_iq(iq + 2 * skip, len - skip, crit[k] / fs) - 20 * log10(0.5);
			if (l > worst)
				worst = l;
		}
		check(worst <= -100.0, "tone at 0.42 of the DSP rate: its images (-dsp, -2 dsp, +dsp, mirror)", worst, "dBc");
		free(iq);
	}
	/* two FM channels 25 kHz apart, one bin */
	{
		double offs[2] = { off, off + 25000.0 }, peak = -999, g;
		double crit[] = { off + fb, off - fb, off + 25000 + fb, off + 25000 - fb, off - dsp, off + 25000 + dsp, off + sp, off - sp };

		for (s = 0; s < nd; s++) {
			in0[s] = 0.25 * cexp(I * (4.7 * sin(2 * M_PI * 1000.0 / dsp * s)));
			in1[s] = 0.25 * cexp(I * (3.0 * sin(2 * M_PI * 1500.0 / dsp * s)));
		}
		len = pfb_tx_run(fs, m, dsp, 2, offs, ins, nd, &iq);
		skip = len / 8;
		for (g = -20000; g <= 20000; g += 1000) {
			double l = level_iq(iq + 2 * skip, len - skip, (off + g) / fs);
			if (l > peak)
				peak = l;
		}
		worst = -999;
		for (k = 0; k < 211 + (int)(sizeof(crit) / sizeof(*crit)); k++) {
			f = (k < 211) ? -fs / 2.0 + 313 + k * (fs / 211.0) : crit[k - 211];
			if (fabs(f - off) < 0.5 * dsp || fabs(f - off - 25000) < 0.5 * dsp)
				continue;
			double l = level_iq(iq + 2 * skip, len - skip, f / fs) - peak;
			if (l > worst) {
				worst = l;
				at = f - off;
			}
		}
		snprintf(what, sizeof(what), "two FM channels 25 kHz apart: strongest outside both (%+.0f)", at);
		check(worst <= -100.0, what, worst, "dB");
		free(iq);
	}
	free(in0);
	free(in1);
}

/* TX pull: random frame counts, each pull exactly that many; the frames
 * laid end to end equal the push API's on the same input */
static void test_pfb_tx_pull(int fs, int m, int dsp, double off)
{
	pfb_t pfb;
	pfb_chan_t a, b;
	int nd = dsp / 5, s, k, total_push = 0, total_pull = 0, pos = 0, bad = 0, max_in = 4000;
	sample_t *in_i = malloc(sizeof(sample_t) * nd), *in_q = malloc(sizeof(sample_t) * nd);
	sample_t *push_i, *push_q;
	double worst = 0;
	char what[120];

	printf("TX pull %d <- %d S/s, %d bins:\n", fs, dsp, m);
	pfb_tx_init(&pfb, fs, m, 0.5 * dsp);
	pfb_chan_tx_init(&a, &pfb, off, dsp, max_in);
	pfb_chan_tx_init(&b, &pfb, off, dsp, max_in);
	for (s = 0; s < nd; s++) {
		in_i[s] = 0.3 * cos(2 * M_PI * 1234.0 / dsp * s) + 0.1 * sin(0.37 * s);
		in_q[s] = 0.3 * sin(2 * M_PI * 1234.0 / dsp * s);
	}
	push_i = malloc(sizeof(sample_t) * pfb_chan_tx_max(&a, nd));
	push_q = malloc(sizeof(sample_t) * pfb_chan_tx_max(&a, nd));
	/* push in blocks of 1000 */
	for (s = 0; s + 1000 <= nd; s += 1000) {
		int f = pfb_chan_tx_process(&a, in_i + s, in_q + s, 1000);

		memcpy(push_i + total_push, a.out_i, f * sizeof(sample_t));
		memcpy(push_q + total_push, a.out_q, f * sizeof(sample_t));
		total_push += f;
	}
	/* pull random frame counts */
	srand(11);
	while (1) {
		int frames = 1 + rand() % 300, n = pfb_chan_tx_need(&b, frames);

		if (pos + n > s || total_pull + frames > total_push)
			break;
		pfb_chan_tx_pull(&b, in_i + pos, in_q + pos, n, frames);
		pos += n;
		if (b.fr_given != frames)
			bad++;
		for (k = 0; k < frames; k++)
			worst = fmax(worst, fabs(b.out_i[k] - push_i[total_pull + k]) + fabs(b.out_q[k] - push_q[total_pull + k]));
		total_pull += frames;
	}
	/* the NCO phasor is renormalised at the end of each call: push and pull
	 * calls end at other samples, so the last bits differ */
	snprintf(what, sizeof(what), "%d frames pulled in random counts: wrong counts %d, difference to push (x 1e12)", total_pull, bad);
	check(!bad && worst < 1e-12 && total_pull > total_push / 2, what, worst * 1e12, "");
	pfb_chan_exit(&a);
	pfb_chan_exit(&b);
	pfb_exit(&pfb);
	free(in_i);
	free(in_q);
	free(push_i);
	free(push_q);
}

static void test_refusals(void)
{
	channelizer_t ch;

	printf("Refusals:\n");
	check(channelizer_rx_init(&ch, 48000, 96000, 0, 36000, 1000) < 0, "SDR rate below the DSP rate is refused", 0, "");
	check(channelizer_rx_init(&ch, 960000, 48000, 0, 40000, 1000) < 0, "channel wider than 0.8 of the DSP rate is refused", 0, "");
	check(channelizer_rx_init(&ch, 960000, 96000, 470000, 36000, 1000) < 0, "channel outside the SDR band is refused", 0, "");
	{
		pfb_t pfb;
		pfb_chan_t pc;

		check(pfb_rx_init(&pfb, 10000000, 48, 24000) < 0, "bank: bins not a power of two is refused", 0, "");
		check(pfb_rx_init(&pfb, 10000001, 64, 24000) < 0, "bank: SDR rate not a multiple of bins/2 is refused", 0, "");
		check(pfb_rx_init(&pfb, 10000000, 256, 24000) < 0, "bank: clean band too wide for the spacing is refused", 0, "");
		pfb_rx_init(&pfb, 10000000, 64, 24000);
		check(pfb_chan_rx_init(&pc, &pfb, 0, 96000, 100) < 0, "bank channel: DSP rate above what the bank keeps clean is refused", 0, "");
		check(pfb_chan_rx_init(&pc, &pfb, 4990000, 48000, 100) < 0, "bank channel: outside the SDR band is refused", 0, "");
		pfb_exit(&pfb);
	}
}

int main(void)
{
	printf("=== libchannelizer tests ===\n\n");
	test_halfband();
	test_refusals();
	/* integer ratios */
	test_rx(960000, 96000, -315000, 36000);
	test_rx(384000, 96000, 30000, 36000);
	test_tx(960000, 96000, -315000, 36000);
	test_tx(3840000, 96000, 945000, 36000);
	/* non-integer ratios */
	test_rx(1000000, 96000, 250000, 36000);
	test_tx(1000000, 96000, 250000, 36000);
	test_tx(2400000, 96000, -700000, 36000);
	/* high SDR rate (short early stages): 10 MS/s to the NMT DSP rate */
	test_rx(10000000, 200000, 2475000, 36000);
	test_tx(10000000, 200000, -3500000, 36000);
	test_rx(768000, 192000, -25000, 36000);
	test_tx(768000, 192000, 25000, 36000);
	/* FFT and filter banks */
	test_fftc();
	test_pfb_rx_bank(10000000, 64, 24000);
	test_pfb_rx_bank(1920000, 16, 24000);
	test_pfb_rx(10000000, 64, 48000, -3337500 + 0.31 * 156250);
	test_pfb_rx(1920000, 16, 48000, 412345);
	test_pfb_tx(10000000, 64, 48000, -3337500 + 0.31 * 156250);
	test_pfb_tx(1920000, 16, 48000, -412345);
	test_pfb_tx_pull(10000000, 64, 48000, -3337500 + 0.31 * 156250);
	test_pfb_tx_pull(1920000, 8, 48000, 61234);	/* bin rate 480k: 10 x DSP */
	test_pfb_tx_pull(3840000, 16, 96000, 945000);	/* bin rate = 2 dsp: no wide resampler */

	printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
