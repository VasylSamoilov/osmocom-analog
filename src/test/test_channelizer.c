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

static void test_halfband(void)
{
	static const int orders[] = { HALFBAND_ORDER_48, HALFBAND_ORDER_64 };
	static const double need[] = { 80.0, 100.0 };
	static double complex y[8192];
	halfband_t hb;
	unsigned o;

	printf("Halfband 2x decimation and interpolation:\n");
	for (o = 0; o < 2; o++) {
		double worst = 0, flat = 0, f;
		char what[80];

		/* decimation: passband 0 .. 0.2, aliases from 0.3 .. 0.5 */
		for (f = 0.0; f <= 0.45; f += 0.025) {
			int n = 0, m;

			halfband_init(&hb, orders[o]);
			for (m = 0; n < 4096; m++) {
				double complex x = 0.5 * cexp(2 * M_PI * I * f * m);
				sample_t i = creal(x), q = cimag(x);

				if (halfband_decimate(&hb, &i, &q))
					y[n++] = (i + I * q) / 0.5;
			}
			halfband_exit(&hb);
			double fo = fmod(2 * f + 0.5, 1.0) - 0.5, l = level(y + 200, 3800, fo);

			if (f <= 0.2 + 1e-9 && fabs(l) > flat)
				flat = fabs(l);
			if (f >= 0.3 - 1e-9 && -l < worst + 1e9 && (worst == 0 || -l < worst))
				worst = -l;
		}
		snprintf(what, sizeof(what), "order %d decimation: passband ripple (0 .. 0.2)", orders[o]);
		check(flat < 0.01, what, flat, "dB");
		snprintf(what, sizeof(what), "order %d decimation: alias rejection (0.3 .. 0.45)", orders[o]);
		check(worst >= need[o], what, worst, "dB");

		/* interpolation: image of a tone at f lands at f/2 - 0.5 */
		worst = 0;
		flat = 0;
		for (f = 0.0; f <= 0.4; f += 0.05) {
			sample_t oi[2], oq[2];
			int m;

			halfband_init(&hb, orders[o]);
			for (m = 0; m < 4096; m++) {
				double complex x = cexp(2 * M_PI * I * f * m);

				halfband_interpolate(&hb, creal(x), cimag(x), oi, oq);
				y[2 * m] = oi[0] + I * oq[0];
				y[2 * m + 1] = oi[1] + I * oq[1];
			}
			halfband_exit(&hb);
			double want = level(y + 200, 7800, f / 2), image = level(y + 200, 7800, f / 2 - 0.5);

			if (fabs(want) > flat)
				flat = fabs(want);
			if (worst == 0 || want - image < worst)
				worst = want - image;
		}
		snprintf(what, sizeof(what), "order %d interpolation: gain error (0 .. 0.4 of the input)", orders[o]);
		check(flat < 0.01, what, flat, "dB");
		snprintf(what, sizeof(what), "order %d interpolation: image rejection", orders[o]);
		check(worst >= need[o] - 20.0 * (o == 0), what, worst, "dB");
	}
}

/* RX: tone at f_in (Hz, relative to the SDR centre) -> output: strongest
 * component within fmax (cycles per DSP sample) and its frequency (Hz); if
 * at_l, the level at the frequency where the tone belongs (f_in - off) */
static double rx_tone(int sdr, int dsp, double off, double bw, double f_in, double fmax, double *f_at, double *at_l)
{
	channelizer_t ch;
	int nin = (int)((double)sdr / dsp * 4096), n, used, k;
	float *iq = malloc(sizeof(*iq) * 2 * nin);
	sample_t *oi = malloc(sizeof(*oi) * nin), *oq = malloc(sizeof(*oq) * nin);
	double complex *y = malloc(sizeof(*y) * nin);
	double l;

	channelizer_rx_init(&ch, sdr, dsp, off, bw);
	for (k = 0; k < nin; k++) {
		double complex z = 0.5 * cexp(2 * M_PI * I * f_in / sdr * k);

		iq[2 * k] = creal(z);
		iq[2 * k + 1] = cimag(z);
	}
	n = channelizer_rx_process(&ch, iq, nin, nin, oi, oq, &used);
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
	channelizer_tx_init(&ch, sdr, dsp, off, bw);
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

		channelizer_tx_init(&c2, sdr, dsp, off, bw);
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
	check(channelizer_rx_init(&ch, 48000, 96000, 0, 36000) < 0, "SDR rate below the DSP rate is refused", 0, "");
	check(channelizer_rx_init(&ch, 960000, 48000, 0, 40000) < 0, "channel wider than 0.8 of the DSP rate is refused", 0, "");
	check(channelizer_rx_init(&ch, 960000, 96000, 470000, 36000) < 0, "channel outside the SDR band is refused", 0, "");
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

	printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
