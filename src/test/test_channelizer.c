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

static void test_refusals(void)
{
	channelizer_t ch;

	printf("Refusals:\n");
	check(channelizer_rx_init(&ch, 48000, 96000, 0, 36000, 1000) < 0, "SDR rate below the DSP rate is refused", 0, "");
	check(channelizer_rx_init(&ch, 960000, 48000, 0, 40000, 1000) < 0, "channel wider than 0.8 of the DSP rate is refused", 0, "");
	check(channelizer_rx_init(&ch, 960000, 96000, 470000, 36000, 1000) < 0, "channel outside the SDR band is refused", 0, "");
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

	printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
