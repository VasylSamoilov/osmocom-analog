/* AMPS receiver of the base station: bursts of a phone into the DSP of a control channel.
 *
 * A registration (words A, B and C) as a phone sends it on the RECC is made here (dotting, sync,
 * coded DCC, each word 5 times, Manchester with the ramp of dsp.c) and given to sender_receive()
 * of a control channel at 96 kHz. What the frame layer decoded is read from the channel
 * (rx_recc_min1, ...); amps_rx_recc(), the call processing, is replaced by a counter (link with
 * --wrap=amps_rx_recc).
 *
 * (C) 2026 - osmocom-analog AMPS receiver tests.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libmobile/call.h"
#include "../libmobile/main_mobile.h"
#include "../amps/amps.h"
#include "../amps/frame.h"
#include "../amps/dsp.h"

/* what amps_main.c gives the AMPS library */
const int tacs = 0;
const int jtacs = 0;
const struct number_lengths number_lengths[] = {
	{ 10, "AMPS number (NPA-XXX-XXXX)" },
	{ 0, NULL }
};
const char *number_prefixes[] = {
	"1xxxxxxxxxx",
	"+1xxxxxxxxxx",
	NULL
};

static int failures;

#define CHECK(cond, msg) do { \
	if (cond) { \
		printf("  OK   %s\n", msg); \
	} else { \
		printf("  FAIL %s\n", msg); \
		failures++; \
	} \
} while (0)

/* the call processing: counted, the message kept */
static int recc_count;
static uint32_t recc_min1, recc_esn;
static uint16_t recc_min2;
static uint8_t recc_order, recc_ordq;

void __wrap_amps_rx_recc(amps_t *amps, uint8_t t, uint8_t scm, uint8_t mpci, uint32_t esn, uint32_t min1, uint16_t min2, uint8_t msg_type, uint8_t ordq, uint8_t order, const char *dialing, uint8_t mspc, uint8_t mscap, uint8_t sdcc1, uint8_t sdcc2);
void __wrap_amps_rx_recc(amps_t *amps, uint8_t t, uint8_t scm, uint8_t mpci, uint32_t esn, uint32_t min1, uint16_t min2, uint8_t msg_type, uint8_t ordq, uint8_t order, const char *dialing, uint8_t mspc, uint8_t mscap, uint8_t sdcc1, uint8_t sdcc2)
{
	(void)amps; (void)t; (void)scm; (void)mpci; (void)msg_type; (void)dialing; (void)mspc; (void)mscap; (void)sdcc1; (void)sdcc2;
	recc_count++;
	recc_min1 = min1;
	recc_min2 = min2;
	recc_esn = esn;
	recc_order = order;
	recc_ordq = ordq;
}

/* BCH (48,36) / (40,28) parity, as frame.c */
static uint16_t bch(uint64_t value, int length)
{
	static const char gp[12] = { 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1 };
	char redun[12] = { 0 };
	int i, j, feedback;
	uint16_t p = 0;

	for (i = 0; i < length; i++) {
		feedback = ((value >> (length - 1 - i)) & 1) ^ redun[0];
		for (j = 0; j < 11; j++)
			redun[j] = redun[j + 1] ^ (feedback && gp[j]);
		redun[11] = feedback && gp[11];
	}
	for (i = 0; i < 12; i++)
		p = (p << 1) | redun[i];
	return p;
}

static uint64_t reverse_word(uint64_t message)
{
	return (message << 12) | bch(message, 36);
}

/* a registration of MIN 0x654321/0x155, ESN 0x89abcdef: word A, B, C */
#define TEST_MIN1	0x654321
#define TEST_MIN2	0x155
#define TEST_ESN	0x89abcdef
static void registration(uint64_t *w)
{
	/* A: F=1 NAWC=2 T=1 S=1 E=1 ER=0 SCM=0xa MIN1 */
	w[0] = reverse_word((1ull << 35) | (2ull << 32) | (1ull << 31) | (1ull << 30) | (1ull << 29) | (0xaull << 24) | TEST_MIN1);
	/* B: F=0 NAWC=1 MSG_TYPE=0 ORDQ=3 ORDER=13 (autonomous registration) LT EP SCM MPCI SDCC1 SDCC2 = 0, MIN2 */
	w[1] = reverse_word((1ull << 32) | (3ull << 24) | (13ull << 19) | TEST_MIN2);
	/* C: F=0 NAWC=0 ESN */
	w[2] = reverse_word(TEST_ESN);
}

/* the bits of a RECC message: 30 dotting, sync, coded DCC 0, each word 5 times */
static int recc_bits(char *bits, const uint64_t *w, int words)
{
	int n = 0, i, r, b;

	for (i = 0; i < 30; i++)
		bits[n++] = (i & 1) ? 0 : 1;
	for (i = 10; i >= 0; i--)
		bits[n++] = (0x712 >> i) & 1;
	for (i = 0; i < 7; i++)
		bits[n++] = 0;		/* coded DCC of DCC 0 */
	for (i = 0; i < words; i++)
		for (r = 0; r < 5; r++)
			for (b = 47; b >= 0; b--)
				bits[n++] = (w[i] >> b) & 1;
	return n;
}

/* Manchester as dsp.c fsk_encode (a 1 low to high, a 0 high to low, a half cosine per change),
 * the bit clock of the phone off by ppm, in units of the speech deviation */
static int modulate(sample_t *out, const char *bits, int count, double samplerate, double ppm)
{
	const double deviation = 8000.0 / 2900.0;
	const double half = samplerate / 10000.0 / 2.0 / (1.0 + ppm * 1e-6);	/* samples per half bit */
	int n, k;

	for (n = 0; ; n++) {
		double t = (double)n / half, from, to, u;

		k = (int)t;
		if (k >= 2 * count)
			break;
		u = t - k;
		/* the frequency at the end of half bit k: the second half of a 1 is high */
		to = ((bits[k / 2] != 0) == (k & 1)) ? deviation : -deviation;
		if (k == 0)
			from = -deviation;	/* as after a 0 */
		else
			from = ((bits[(k - 1) / 2] != 0) == ((k - 1) & 1)) ? deviation : -deviation;
		out[n] = (from + to) / 2.0 + (from - to) / 2.0 * cos(M_PI * u);
	}
	return n;
}

static amps_t *control_channel(void)
{
	amps_t *amps = calloc(1, sizeof(*amps));

	amps->sender.samplerate = 96000;
	amps->sender.kanal = "334";
	display_measurements_init(&amps->sender.dispmeas, 96000, amps->sender.kanal);
	amps->chan_type = CHAN_TYPE_CC_PC;
	amps->si.word2.s = 1;
	if (dsp_init_sender(amps, 0) < 0) {
		printf("dsp_init_sender failed\n");
		exit(1);
	}
	amps_set_dsp_mode(amps, DSP_MODE_FRAME_RX_FRAME_TX, 247);
	return amps;
}

/* gaussian noise */
static double gauss(void)
{
	double u1 = (random() + 1.0) / 2147483649.0, u2 = random() / 2147483648.0;

	return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* the bits through the DSP of a new control channel, with silence before and after, noise of
 * the given rms (relative to the FSK deviation) on all */
static void receive_noise(const char *bits, int count, double ppm, double noise)
{
	static sample_t samples[96000];
	amps_t *amps = control_channel();
	int n = 0, i;

	for (i = 0; i < 960 + (int)(random() % 97); i++)
		samples[n++] = 0;
	n += modulate(samples + n, bits, count, 96000.0, ppm);
	for (i = 0; i < 960; i++)
		samples[n++] = 0;
	for (i = 0; i < n && noise > 0.0; i++)
		samples[i] += noise * 8000.0 / 2900.0 * gauss();
	recc_count = 0;
	recc_min1 = recc_min2 = recc_esn = 0;
	for (i = 0; i < n; i += 960)
		sender_receive(&amps->sender, samples + i, (n - i < 960) ? n - i : 960, NAN);
	dsp_cleanup_sender(amps);
	free(amps);
}

static void receive(const char *bits, int count, double ppm)
{
	receive_noise(bits, count, ppm, 0.0);
}

static int registration_received(void)
{
	return recc_count == 1 && recc_min1 == TEST_MIN1 && recc_min2 == TEST_MIN2 && recc_esn == TEST_ESN
	    && recc_order == 13 && recc_ordq == 3;
}

static void test_clean(void)
{
	uint64_t w[3];
	char bits[1024];
	int n;

	registration(w);
	n = recc_bits(bits, w, 3);
	receive(bits, n, 0.0);
	CHECK(registration_received(), "a registration (words A, B, C) is received");
}

/* test_amps_rx measure: registrations received out of 200 in noise and with clock offsets */
static void measure(void)
{
	static const double noises[] = { 0.3, 0.4, 0.5, 0.6 };
	static const double ppms[] = { 30.0, 500.0, 1000.0 };
	uint64_t w[3];
	char bits[1024];
	int n, i, j, k, ok;

	registration(w);
	n = recc_bits(bits, w, 3);
	srandom(1);
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 4; i++) {
			ok = 0;
			for (k = 0; k < 200; k++) {
				receive_noise(bits, n, ppms[j], noises[i]);
				ok += registration_received();
			}
			printf("MEASURE ppm %5.0f noise %.1f of the deviation: %3d of 200 received\n", ppms[j], noises[i], ok);
		}
	}
}

int main(int argc, char *argv[])
{
	dsp_init();
	init_frame();

	if (argc > 1 && !strcmp(argv[1], "measure")) {
		measure();
		return 0;
	}

	printf("AMPS base station receiver (RECC through the DSP at 96 kHz)\n");
	test_clean();

	if (failures) {
		printf("%d check(s) failed\n", failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
