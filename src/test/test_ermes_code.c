/* Test of the ERMES coding layer against the values printed in ETS 300 133-4 ed.2 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "../libsample/sample.h"
#include "../libfilter/iir_filter.h"
#include "../ermes/code.h"
#include "../ermes/frame.h"
#include "../ermes/rx.h"
#include "../ermes/dsp.h"
#include "../libpagercharset/pager_charset.h"

static int failed;

#define CHECK(cond, ...) do { \
	if (!(cond)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); failed++; } \
} while (0)

static uint32_t bits(const char *s)
{
	uint32_t v = 0;

	for (; *s; s++) {
		if (*s == '0' || *s == '1')
			v = (v << 1) | (*s - '0');
	}
	return v;
}

/* §6 p.24-25: the two worked examples */
static void test_bch_vectors(void)
{
	CHECK(ermes_bch_encode(1) == bits("000 000 000 000 000 001 101 011 001 101"),
	      "m=1 -> %08x", ermes_bch_encode(1));
	CHECK(ermes_bch_encode(0x3ffff) == bits("111 111 111 111 111 111 110 001 001 011"),
	      "m=all ones -> %08x", ermes_bch_encode(0x3ffff));
	/* p.24 generator matrix: row 1 (m17) check part 111100001000, row 5 (m13) 110110010110 */
	CHECK((ermes_bch_encode(1u << 17) & 0xfff) == bits("111100001000"), "G row 1");
	CHECK((ermes_bch_encode(1u << 13) & 0xfff) == bits("110110010110"), "G row 5");
	printf("bch vectors done\n");
}

/* d = 6: every 1- and 2-bit error corrected, every 3-bit error detected, never miscorrected */
static void test_bch_correction(void)
{
	static const uint32_t infos[] = { 0, 1, 0x3ffff, 0x2aaaa, 0x15555, 0x12345, 0x3c0f0 };
	unsigned k;
	int i, j, l, rc;
	long n3 = 0;

	for (k = 0; k < sizeof(infos) / sizeof(infos[0]); k++) {
		uint32_t cw = ermes_bch_encode(infos[k]), r;

		r = cw;
		CHECK(ermes_bch_decode(&r) == 0 && r == cw, "clean word %05x", infos[k]);
		for (i = 0; i < 30; i++) {
			r = cw ^ (1u << i);
			rc = ermes_bch_decode(&r);
			CHECK(rc == 1 && r == cw, "1 error bit %d info %05x rc %d", i, infos[k], rc);
			for (j = i + 1; j < 30; j++) {
				r = cw ^ (1u << i) ^ (1u << j);
				rc = ermes_bch_decode(&r);
				CHECK(rc == 2 && r == cw, "2 errors %d,%d info %05x rc %d", i, j, infos[k], rc);
				for (l = j + 1; l < 30; l++) {
					r = cw ^ (1u << i) ^ (1u << j) ^ (1u << l);
					rc = ermes_bch_decode(&r);
					CHECK(rc == -1, "3 errors %d,%d,%d info %05x not detected (rc %d)", i, j, l, infos[k], rc);
					n3++;
				}
			}
		}
	}
	printf("bch correction done (%ld triple errors detected)\n", n3);
}

/* The fixed words are patterns, not codewords; report their distance to the code */
static void test_fixed_words(void)
{
	static const struct { const char *name; uint32_t w; } words[] = {
		{ "PR",  ERMES_PREAMBLE }, { "SW1", ERMES_SW1 }, { "APT", ERMES_APT }, { "MD", ERMES_MD },
	};
	unsigned i, j;

	CHECK(ERMES_PREAMBLE == bits("00 10 00 10 00 10 00 10 00 10 00 10 00 10 00"), "PR");
	CHECK(ERMES_SW1 == bits("10 00 10 10 00 10 00 00 10 10 00 00 10 10 10"), "SW1");
	CHECK(ERMES_APT == bits("10 01 00 11 10 00 01 10 00 10 00 11 10 01 00"), "APT");
	CHECK(ERMES_MD == bits("11 01 01 01 11 10 01 11 11 10 11 10 11 10 11"), "MD");
	for (i = 0; i < 4; i++) {
		uint32_t r = words[i].w;
		int rc = ermes_bch_decode(&r);

		printf("  %-3s %08x: BCH decode rc %d", words[i].name, words[i].w, rc);
		for (j = 0; j < 4; j++) {
			if (j != i)
				printf("  d(%s)=%d", words[j].name, ermes_distance(words[i].w, words[j].w));
		}
		printf("\n");
		CHECK(ermes_bch_encode(words[i].w >> 12) != words[i].w, "%s is a codeword", words[i].name);
	}
	printf("fixed words done\n");
}

/* §7 p.25: column by column, bit 29 of codewords 1..9 first */
static void test_interleave(void)
{
	uint32_t cw[9], back[9];
	uint8_t b[270];
	int i;

	for (i = 0; i < 9; i++)
		cw[i] = ermes_bch_encode(0x1000 * i + 0x37 * i + 5);
	ermes_interleave(cw, b);
	for (i = 0; i < 9; i++)
		CHECK(b[i] == ((cw[i] >> 29) & 1), "column 29 row %d", i);
	CHECK(b[9] == ((cw[0] >> 28) & 1), "c1,28 is the 10th bit");
	CHECK(b[269] == (cw[8] & 1), "c9,0 is the last bit");
	ermes_deinterleave(b, back);
	for (i = 0; i < 9; i++)
		CHECK(back[i] == cw[i], "round trip row %d", i);
	printf("interleave done\n");
}

/* §4.1 p.10, §4.5 p.14, Table 1 p.13 */
static void test_batch_numbers(void)
{
	static const struct { int i, j, k_first, k_last; } typical[] = {
		/* "Typical values" listed under Table 1, p.13 */
		{ 0, 5, 16, 20 }, { 4, 1, 16, 20 }, { 5, 9, 15, -1 }, { 139, 1, 1, 5 }, { 22, 1, -1, 18 },
	};
	unsigned n;
	int s, total = 0;

	for (s = 0; s < 16; s++) {
		CHECK(ermes_batch_offset(s) == total, "offset slot %d", s);
		total += ermes_batch_length(s);
	}
	CHECK(total == ERMES_SUBSEQUENCE_CW, "subsequence %d codewords", total);
	CHECK(total * 30 == 12 * ERMES_BIT_RATE, "12 s at 6250 bit/s");
	for (n = 0; n < sizeof(typical) / sizeof(typical[0]); n++) {
		CHECK(ermes_apt_count(typical[n].i) == typical[n].j, "J for I=%d: %d", typical[n].i, ermes_apt_count(typical[n].i));
		if (typical[n].k_first >= 0)
			CHECK(ermes_max_codeblocks(0, typical[n].i) == typical[n].k_first, "K first, I=%d: %d",
			      typical[n].i, ermes_max_codeblocks(0, typical[n].i));
		if (typical[n].k_last >= 0)
			CHECK(ermes_max_codeblocks(15, typical[n].i) == typical[n].k_last, "K last, I=%d: %d",
			      typical[n].i, ermes_max_codeblocks(15, typical[n].i));
	}
	printf("batch numbers done\n");
}

/* Fig.3 p.11, first column of every row, and §8.2 p.25 */
static void test_channels(void)
{
	static const char *fig3_first = "A?P?O?N?M?L?K?J"; /* channels 1,3,..15 at even index */
	static const struct { int ch; char first; } even[] = {
		{ 16, 'I' }, { 14, 'H' }, { 12, 'G' }, { 10, 'F' }, { 8, 'E' }, { 6, 'D' }, { 4, 'C' }, { 2, 'B' },
	};
	unsigned i;
	int ch;

	for (ch = 1; ch <= 15; ch += 2)
		CHECK('A' + ermes_channel_first_batch(ch) == fig3_first[ch - 1], "channel %d", ch);
	for (i = 0; i < 8; i++)
		CHECK('A' + ermes_channel_first_batch(even[i].ch) == even[i].first, "channel %d", even[i].ch);
	/* row 03 continues P A B C ... (the active batch for type A is the second) */
	CHECK(ermes_channel_batch(3, 1) == 0, "channel 3 slot 1 = A");
	CHECK(ermes_channel_batch(2, 15) == 0, "channel 2 slot 15 = A");
	CHECK(ermes_channel_frequency(1) == 169425000.0, "channel 1");
	CHECK(ermes_channel_frequency(16) == 169800000.0, "channel 16");
	printf("channels done\n");
}

static uint32_t word_at(const uint8_t *b, int index)
{
	uint32_t w = 0;
	int i;

	for (i = 0; i < 30; i++)
		w = (w << 1) | b[index * 30 + i];
	return w;
}

/* Build a batch for the NEC PA3E6-1A factory default (local address 0x318E40) and read it back */
/* §8.3.3 Fig.16: the premodulation filter (10th order Bessel, 3 dB at 3.9 kHz) at the sample rate
 * of the transmitter, against the amplitude masks (a), (b) and the group delay mask (c) */
struct mask_point { double khz, db; };

/* the mask in dB at f, linear between its corners; outside: -1000 / +1000 (no limit) */
static double mask_db(const struct mask_point *m, int n, double khz, double none)
{
	int i;

	if (khz < m[0].khz || khz > m[n - 1].khz)
		return none;
	for (i = 1; i < n; i++) {
		if (khz <= m[i].khz)
			return m[i - 1].db + (m[i].db - m[i - 1].db) * (khz - m[i - 1].khz) / (m[i].khz - m[i - 1].khz);
	}
	return none;
}

/* response of the biquad cascade at f: magnitude (dB) and phase (rad) */
static void cascade_response(const iir_filter_t *f, int n, double hz, int rate, double *db, double *phase)
{
	double w = 2.0 * M_PI * hz / rate, mag = 1.0, ph = 0.0;
	int k;

	for (k = 0; k < n; k++) {
		/* H(z) = (a0 + a1 z^-1 + a2 z^-2) / (1 + b1 z^-1 + b2 z^-2) */
		double nr = f[k].a0 + f[k].a1 * cos(w) + f[k].a2 * cos(2 * w);
		double ni = -f[k].a1 * sin(w) - f[k].a2 * sin(2 * w);
		double dr = 1.0 + f[k].b1 * cos(w) + f[k].b2 * cos(2 * w);
		double di = -f[k].b1 * sin(w) - f[k].b2 * sin(2 * w);

		mag *= sqrt((nr * nr + ni * ni) / (dr * dr + di * di));
		ph += atan2(ni, nr) - atan2(di, dr);
	}
	*db = 20.0 * log10(mag);
	*phase = ph;
}

static void test_premodulation_filter(void)
{
	/* Fig.16(a)/(b): the response must stay below the upper and above the lower mask */
	static const struct mask_point upper[] = {
		{ 0.0, 0.0 }, { 1.47, 0.0 }, { 3.1, -1.0 }, { 4.5, -3.0 }, { 7.5, -10.0 }, { 15.2, -40.0 }, { 19.0, -60.0 },
	};
	static const struct mask_point lower[] = {
		{ 0.0, 0.0 }, { 1.47, -0.5 }, { 2.6, -1.5 }, { 3.7, -3.0 }, { 6.8, -10.0 }, { 12.0, -40.0 }, { 16.0, -70.0 },
	};
	/* the transmitter runs it at 4 times its sample rate of 48 kHz (ermes.c) */
	const int rate = 4 * 48000, order = 10, n = order / 2;
	iir_filter_t f[5];
	double hz, db, ph, last_ph = 0.0, gd, gd0 = 0.0, worst_amp = 1e9, worst_gd = 0.0, worst_gd_hz = 0.0;
	int first = 1;

	iir_bessel_lowpass_init(f, 3900.0, rate, order);
	cascade_response(f, n, 3900.0, rate, &db, &ph);
	CHECK(fabs(db + 3.0103) < 0.05, "3 dB at 3.9 kHz: %.3f dB", db);
	for (hz = 10.0; hz <= 19000.0; hz += 10.0) {
		double up, lo;

		cascade_response(f, n, hz, rate, &db, &ph);
		up = mask_db(upper, sizeof(upper) / sizeof(upper[0]), hz / 1000.0, 1000.0);
		lo = mask_db(lower, sizeof(lower) / sizeof(lower[0]), hz / 1000.0, -1000.0);
		CHECK(db <= up + 1e-9 && db >= lo - 1e-9, "amplitude %.2f dB at %.0f Hz outside the mask (%.2f .. %.2f)", db, hz, lo, up);
		if (up - db < worst_amp)
			worst_amp = up - db;
		if (db - lo < worst_amp)
			worst_amp = db - lo;
		/* group delay -d(phase)/d(omega), from the phase step (unwrapped) */
		if (!first) {
			double d = ph - last_ph;

			while (d > M_PI) d -= 2 * M_PI;
			while (d < -M_PI) d += 2 * M_PI;
			gd = -d / (2.0 * M_PI * 10.0);
			if (hz <= 20.0)
				gd0 = gd;
			/* Fig.16(c): +-32 us to 3.2 kHz, +-80 us to 5.5 kHz */
			if (hz <= 5500.0) {
				double lim = (hz <= 3200.0) ? 32e-6 : 80e-6;

				CHECK(fabs(gd - gd0) <= lim, "group delay %.1f us from DC at %.0f Hz (limit %.0f us)",
				      (gd - gd0) * 1e6, hz, lim * 1e6);
				if (fabs(gd - gd0) > worst_gd) {
					worst_gd = fabs(gd - gd0);
					worst_gd_hz = hz;
				}
			}
		}
		first = 0;
		last_ph = ph;
	}
	printf("premodulation filter done (amplitude at least %.2f dB inside the masks; group delay %.0f us at DC, deviation at most %.1f us at %.0f Hz)\n",
	       worst_amp, gd0 * 1e6, worst_gd * 1e6, worst_gd_hz);
}

/* §9.2 Fig.18 p.30 and its text examples */
static void test_fsi(void)
{
	static const int fsn12[] = { 12, 22, 27, 29, 30 };
	int fsi, fsn, n, count;

	/* "when the FSI on a channel equals 27 only messages to receivers with FSNs of 12, 13, 14 or
	 * 15 will be carried" */
	CHECK(ermes_fsi_subset(27, &count) == 12 && count == 4, "FSI 27 = FSN 12..15");
	/* "a receiver with FSN = 12 should look for its messages only on channels broadcasting FSI
	 * values of 12, 22, 27, 29 or 30" */
	for (n = 0, fsi = 0; fsi <= ERMES_FSI_MAX; fsi++) {
		if (!ermes_fsi_has_fsn(fsi, 12))
			continue;
		CHECK(n < 5 && fsn12[n] == fsi, "FSN 12 on FSI %d", fsi);
		n++;
	}
	CHECK(n == 5, "FSN 12 on %d FSIs", n);
	/* single channel 30 = all; two channels 28 and 29 split the FSNs */
	for (fsn = 0; fsn < ERMES_FSN_COUNT; fsn++) {
		CHECK(ermes_fsi_has_fsn(30, fsn), "FSI 30 has FSN %d", fsn);
		CHECK(ermes_fsi_has_fsn(28, fsn) + ermes_fsi_has_fsn(29, fsn) == 1, "FSN %d on one of 28/29", fsn);
	}
	CHECK(ermes_fsi_subset(28, &count) == 0 && count == 8, "FSI 28 = FSN 0..7");
	CHECK(!ermes_fsi_overlap(28, 29) && ermes_fsi_overlap(30, 5) && ermes_fsi_overlap(27, 13)
	      && !ermes_fsi_overlap(27, 11), "overlaps");
	CHECK(ermes_fsi_subset(31, &count) < 0, "FSI 31 invalid");
	printf("frequency subsets done\n");
}

static void test_frame(void)
{
	static uint8_t b[190 * 30];
	ermes_network_t net = { .zone = 2, .country = 1, .operator = 2, .pa = 0, .bai = 0, .fsi = 30 };
	ermes_batch_time_t t = { .cycle = 37, .ssn = 2, .slot = 0, .batch = 0 };
	ermes_ssi_t ssi = { .ssit = 0, .ssif = ermes_ssif_zone_hour_date(2, 14, 26) };
	ermes_page_t pages[2] = {
		{ .local_address = 0x318E40, .number = 1, .category = ERMES_CAT_TONE, .alert = 0 },
		{ .local_address = 0x000011, .number = 3, .category = ERMES_CAT_NUMERIC, .text = "1234" },
	};
	uint32_t data[8], cw[9], w;
	int rc, nd, i, n;

	/* numeric packing: 0001 0010 0011 0100, then partial space "10" (MSB of 1011) */
	nd = ermes_data_words(&pages[1], data, 8);
	CHECK(nd == 1 && data[0] == bits("0001 0010 0011 0100 10"), "numeric 1234 -> %d %05x", nd, data[0]);
	/* alpha: 'A' 1000001, EOM 0010001, then partial EOM 0010 */
	{
		ermes_page_t a = { .local_address = 0, .category = ERMES_CAT_ALPHA, .text = "A" };
		nd = ermes_data_words(&a, data, 8);
		CHECK(nd == 1 && data[0] == bits("1000001 0010001 0010"), "alpha A -> %d %05x", nd, data[0]);
	}

	rc = ermes_build_batch(&net, &t, &ssi, pages, 1, NULL, b);
	CHECK(rc == 1, "build rc %d", rc);
	CHECK(word_at(b, 0) == ERMES_PREAMBLE && word_at(b, 1) == ERMES_SW1, "sync partition");
	w = word_at(b, 2);
	CHECK(ermes_bch_decode(&w) == 0 && (w >> 12) == ((1u << 11) | (2u << 8)), "SI1 %05x", w >> 12);
	w = word_at(b, 3);
	CHECK(ermes_bch_decode(&w) == 0 && (w >> 12) == ((30u << 13) | (37u << 7) | (2u << 4) | 0), "SI2 %05x", w >> 12);
	w = word_at(b, 4);
	CHECK(ermes_bch_decode(&w) == 0 && (w >> 12) == ((0u << 14) | (2u << 11) | (14u << 6) | (26u << 1)), "SSI %05x", w >> 12);
	w = word_at(b, 5);
	CHECK(ermes_bch_decode(&w) == 0 && (w >> 12) == 0x318E4, "IA %05x", w >> 12);
	/* I = 1 -> J = 9 - 5 = 4 APTs, K = (149 - 5) / 9 = 16 codeblocks */
	for (i = 6; i < 10; i++)
		CHECK(word_at(b, i) == ERMES_APT, "APT at %d", i);
	CHECK(ermes_max_codeblocks(0, 1) == 16, "K");
	ermes_deinterleave(b + 10 * 30, cw);
	CHECK(cw[0] == ERMES_MD, "leading delimiter");
	w = cw[1];
	CHECK(ermes_bch_decode(&w) == 0 && (w >> 12) == 0x318E4, "header 1 %05x", w >> 12);
	w = cw[2];
	/* batch 0000, number 00001, EB 0, AII 0, VIF: RSVD 0, category 00 tone, UMI 0, alert 000 */
	CHECK(ermes_bch_decode(&w) == 0 && (w >> 12) == bits("0000 00001 0 0 0 00 0 000"), "header 2 %05x", w >> 12);
	for (i = 3; i < 9; i++)
		CHECK(cw[i] == ERMES_MD, "delimiter after message, word %d", i);
	for (n = 1; n < 16; n++) {
		ermes_deinterleave(b + (10 + 9 * n) * 30, cw);
		for (i = 0; i < 9; i++)
			CHECK(cw[i] == ERMES_MD, "idle codeblock %d word %d", n, i);
	}

	/* a page for another batch type is refused, the last slot has 190 words */
	CHECK(ermes_build_batch(&net, &t, &ssi, &pages[1], 1, NULL, b) == -1, "wrong batch refused");
	t.slot = 15; t.batch = 1;
	rc = ermes_build_batch(&net, &t, &ssi, &pages[1], 1, NULL, b);
	CHECK(rc == 1, "slot 15 rc %d", rc);
	t.slot = 3; t.batch = 0;
	CHECK(ermes_build_batch(&net, &t, &ssi, NULL, 0, NULL, b) == 0, "empty batch");
	for (i = 5; i < 10; i++)
		CHECK(word_at(b, i) == ERMES_APT, "empty: APT at %d", i);
	printf("frame done\n");
}

/* Message partition codewords of a built batch (I initial addresses), deinterleaved */
static int partition_words(const uint8_t *b, int slot, int ia, uint32_t *out)
{
	int k, n = 0, first = 5 + ia + ermes_apt_count(ia);

	for (k = 0; k < ermes_max_codeblocks(slot, ia); k++, n += 9)
		ermes_deinterleave(b + (first + 9 * k) * 30, out + n);
	return n;
}

/* §10.5 p.34: a message longer than its batch runs on in the next batches, immediately after their
 * address partition, with no delimiter between the parts; other pages wait, a message after it
 * starts behind its closing delimiter */
static void test_continuation(void)
{
	static uint8_t b[190 * 30];
	static char text[700];
	static uint32_t got[2000];
	ermes_network_t net = { .zone = 2, .country = 1, .operator = 2, .pa = 0, .bai = 0, .fsi = 30 };
	ermes_batch_time_t t = { .cycle = 10, .ssn = 0, .slot = 4, .batch = 0 };
	ermes_ssi_t ssi = { .ssit = 0, .ssif = 0 };
	ermes_page_t page = { .local_address = 0x318E40, .number = 7, .category = ERMES_CAT_ALPHA };
	ermes_page_t next = { .local_address = 0x000011, .number = 1, .category = ERMES_CAT_NUMERIC, .text = "42" };
	ermes_carry_t carry = { .count = 0 };
	uint32_t w, data[400], hdr[2], all[500];
	int i, n = 0, rc, nd, nall = 0, batches = 0, md_inside = 0;

	for (i = 0; i < 600; i++)
		text[i] = 'A' + i % 26;
	text[600] = '\0';
	page.text = text;
	nd = ermes_data_words(&page, data, 400);
	ermes_header_words(&page, hdr);
	all[nall++] = ermes_bch_encode(hdr[0]);
	all[nall++] = ermes_bch_encode(hdr[1]);
	for (i = 0; i < nd; i++)
		all[nall++] = ermes_bch_encode(data[i]);
	all[nall++] = ERMES_MD;

	/* batch A (slot 4) starts it: leading MD, then as much as fits */
	rc = ermes_build_batch(&net, &t, &ssi, &page, 1, &carry, b);
	CHECK(rc == 1, "first batch rc %d", rc);
	CHECK(carry.count > 0, "message runs on (carry %d)", carry.count);
	n = partition_words(b, t.slot, 1, got);
	CHECK(got[0] == ERMES_MD, "leading delimiter");
	memmove(got, got + 1, (n - 1) * sizeof(uint32_t)); n--;
	batches = 1;
	/* the next batches (B, C, ... in slots 5, 6, ...) carry the rest right after their address
	 * partition; batch B also has a page of its own, which must come after the closing delimiter */
	while (carry.count && batches < 10) {
		uint32_t part[200];
		int np, before = carry.count;

		t.slot++; t.batch++;
		rc = ermes_build_batch(&net, &t, &ssi, (t.batch == 1) ? &next : NULL, (t.batch == 1) ? 1 : 0, &carry, b);
		np = partition_words(b, t.slot, rc > 0 ? rc : 0, part);
		CHECK(part[0] != ERMES_MD, "batch %c: no delimiter before the continued part", 'A' + t.batch);
		for (i = 0; i < before - carry.count; i++) {
			if (part[i] == ERMES_MD && i < before - carry.count - 1)
				md_inside++;
			got[n++] = part[i];
		}
		if (!carry.count && rc == 1) {
			/* the page of this batch follows the closing delimiter */
			i = before;
			CHECK(part[i - 1] == ERMES_MD && part[i] != ERMES_MD, "next page right after the closing delimiter");
		}
		(void)np;
		batches++;
	}
	CHECK(!carry.count, "message ended within %d batches", batches);
	CHECK(md_inside == 0, "no delimiter inside the message");
	CHECK(n == nall, "%d codewords received, %d sent", n, nall);
	for (i = 0; i < n && i < nall; i++)
		CHECK(got[i] == all[i], "codeword %d differs", i);
	w = got[0];
	CHECK(ermes_bch_decode(&w) == 0 && (w >> 12) == 0x318E4, "header after reassembly");
	printf("continuation done (%d characters = %d codewords over %d batches)\n", 600, nall, batches);
}

/*
 * Receiver (rx.c): the encoder's own batches in, the pages out
 */

#define RX_MAX	64
static ermes_rx_msg_t rx_got[RX_MAX];
static int rx_ngot, rx_nbatch, rx_verbose;

static void rx_batch_cb(void __attribute__((unused)) *priv, const ermes_rx_batch_t *b)
{
	rx_nbatch++;
	if (rx_verbose)
		printf("  batch %c cycle %d ssn %d: country %d op %d ETI %d FSI %d, I %d J %d/%d K %d, %d corrected, %d in error, %d unreadable, fill %d%s%s\n",
		       'A' + b->batch, b->cycle, b->ssn, b->country, b->operator, b->eti, b->fsi, b->ia_count, b->apt,
		       b->apt_expected, b->codeblocks, b->corrected, b->bad, b->unreadable, b->fill, b->si_ok ? "" : " SI BAD",
		       b->flywheel ? " (no sync, on timing)" : "");
}

static void rx_msg_cb(void __attribute__((unused)) *priv, const ermes_rx_batch_t __attribute__((unused)) *b, const ermes_rx_msg_t *m)
{
	if (rx_verbose)
		printf("  %s\n", ermes_rx_msg_str(m));
	if (rx_ngot < RX_MAX)
		rx_got[rx_ngot++] = *m;
}

static const ermes_rx_msg_t *rx_find(uint32_t la, int kind)
{
	int i;

	for (i = 0; i < rx_ngot; i++) {
		if (rx_got[i].local_address == la && (int)rx_got[i].kind == kind)
			return &rx_got[i];
	}
	return NULL;
}

/* flip n bits of codeword w of the message partition of a batch (after interleaving: bit b of a
 * codeword in codeblock k is at column-major position) */
static void flip_partition(uint8_t *b, int first, int w, int nbits)
{
	int k = w / 9, row = w % 9, col;

	for (col = 0; col < nbits; col++)
		b[(first + 9 * k) * 30 + (5 + 7 * col) * 9 + row] ^= 1;
}

static void test_rx(void)
{
	static uint8_t b[190 * 30], stream[40 * 190 * 30];
	static char longtext[601];
	ermes_network_t net = { .zone = 2, .country = 1, .operator = 2, .pa = 0, .bai = 0, .fsi = 30 };
	ermes_ssi_t ssi = { .ssit = 0, .ssif = 0 };
	ermes_carry_t carry = { .count = 0 };
	ermes_batch_time_t t;
	ermes_rx_t rx;
	const ermes_rx_msg_t *m;
	int slot, i, n, len, rc, pos = 0, numeric_batch = -1;
	uint32_t seed = 1;
	/* pages that need their own batch type (initial address), and two that go in any batch */
	ermes_page_t pages[] = {
		{ .local_address = 0x318E40, .number = 1, .category = ERMES_CAT_ALPHA, .text = "Hello NEC" },
		{ .local_address = 0x000011, .number = 3, .category = ERMES_CAT_NUMERIC, .text = "0123456789" },
		{ .local_address = 0x000022, .number = 2, .category = ERMES_CAT_TONE, .alert = 5, .urgent = 1 },
		{ .local_address = 0x000033, .number = 7, .category = ERMES_CAT_ALPHA, .text = longtext },
		{ .local_address = 0x000044, .number = 1, .category = ERMES_CAT_ALPHA, .text = "visitor", .external = 1, .opid = (2 << 10) | (1 << 3) | 3 },
		{ .local_address = 0x000055, .number = 9, .last_number = 1 },
		{ .local_address = 0x000066, .number = 4, .category = ERMES_CAT_ALPHA, .group_pointer = 4, .ia_only = 0 },
		{ .local_address = 0x000077, .number = 6, .category = ERMES_CAT_NUMERIC, .text = "555123" },
	};
	ermes_page_t cta = { .local_address = (ERMES_CTA_IA << 4) | 3, .number = 0, .category = ERMES_CAT_ALPHA, .text = "group text" };
	int npages = sizeof(pages) / sizeof(pages[0]);

	for (i = 0; i < 600; i++)
		longtext[i] = 'a' + i % 26;
	longtext[600] = '\0';

	/* noise first: no lock on it */
	for (i = 0; i < 20000; i++) {
		seed = seed * 1103515245 + 12345;
		stream[pos++] = (seed >> 16) & 1;
	}
	/* two subsequences on channel 9: the pages in the batches of their type, the group text in
	 * the batch after the pointer's */
	for (n = 0; n < 32; n++) {
		ermes_page_t sel[4];
		int nsel = 0;

		slot = n % 16;
		t.cycle = 20;
		t.ssn = n / 16;
		t.slot = slot;
		t.batch = ermes_channel_batch(9, slot);
		if (n < 16) {
			for (i = 0; i < npages; i++) {
				if (ermes_address_batch(pages[i].local_address) == t.batch)
					sel[nsel++] = pages[i];
			}
		}
		if (n == 16 + 2)
			sel[nsel++] = cta;
		rc = ermes_build_batch(&net, &t, &ssi, sel, nsel, &carry, b);
		CHECK(rc == nsel, "batch %c built %d of %d", 'A' + t.batch, rc, nsel);
		len = ermes_batch_length(slot);
		/* bit errors: 2 in the system information (corrected) */
		if (n == 1) {
			b[3 * 30 + 3] ^= 1;
			b[3 * 30 + 17] ^= 1;
		}
		/* 3 in the numeric page's second data codeword (§7.5: no digit from there on) */
		for (i = 0; i < nsel; i++) {
			uint32_t cw[190], w1, w2;
			int first = 5 + nsel + ermes_apt_count(nsel), k, nw;

			if (sel[i].local_address != 0x000077)
				continue;
			nw = partition_words(b, slot, nsel, cw);
			for (k = 0; k + 3 < nw; k++) {
				w1 = cw[k];
				w2 = cw[k + 1];
				if (ermes_bch_decode(&w1) == 0 && ermes_bch_decode(&w2) == 0
				 && (w1 >> 12) == (0x000077 >> 4) && (w2 >> 26) == 7)
					break;
			}
			CHECK(k + 3 < nw, "numeric page found in its batch");
			flip_partition(b, first, k + 3, 3);
			numeric_batch = n;
		}
		memcpy(stream + pos, b, len * 30);
		pos += len * 30;
	}
	(void)numeric_batch;

	rx_ngot = rx_nbatch = 0;
	ermes_rx_init(&rx, 9, &pager_charset_ermes_generic_generic_standard, NULL, rx_batch_cb, rx_msg_cb);
	for (i = 0; i < pos; i++)
		ermes_rx_bit(&rx, stream[i]);
	CHECK(rx_nbatch == 32, "%d batches decoded, 32 sent", rx_nbatch);

	m = rx_find(0x318E40, ERMES_RX_MESSAGE);
	CHECK(m && m->number == 1 && !strcmp(m->text, "Hello NEC") && m->eom && !m->bad_fill && !m->chars_bad && m->chars == 9,
	      "alpha to the NEC: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x000011, ERMES_RX_MESSAGE);
	CHECK(m && m->category == ERMES_CAT_NUMERIC && !strcmp(m->text, "0123456789"), "numeric: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x000022, ERMES_RX_MESSAGE);
	CHECK(m && m->category == ERMES_CAT_TONE && m->alert == 5 && m->urgent, "tone: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x000033, ERMES_RX_MESSAGE);
	CHECK(m && !strcmp(m->text, longtext) && m->batches >= 2, "long text over batches: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x000044, ERMES_RX_MESSAGE);
	CHECK(m && m->external && m->opid == pages[4].opid && !strcmp(m->text, "visitor"), "visitor: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x000055, ERMES_RX_LAST_NUMBER);
	CHECK(m && m->number == 9, "lastnum: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x000066, ERMES_RX_POINTER);
	CHECK(m && m->ain == 3 && m->number == 4, "pointer: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(cta.local_address, ERMES_RX_GROUP);
	CHECK(m && !strcmp(m->text, "group text") && m->members == 1 && m->member[0] == 0x000066 && m->member_number[0] == 4,
	      "group text with its member: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x000077, ERMES_RX_MESSAGE);
	/* a monitor: the digits from the bad codeword '?', the rest kept (5551 in the good one) */
	CHECK(m && !strncmp(m->text, "5551?", 5) && m->chars_bad >= 2 && m->char_state[0] == 0 && m->char_state[4] == 2,
	      "numeric with a codeword in error: %s", m ? ermes_rx_msg_str(m) : "missing");
	CHECK(rx_ngot == npages + 1, "%d messages, %d sent", rx_ngot, npages + 1);
	printf("receiver done (%d batches, %d messages)\n", rx_nbatch, rx_ngot);
}

/*
 * Receiver conformance cases (ETS 300 133-5 §7, TBR 7 §8): batches built from raw codewords, as
 * the test figures draw them (FILL, PADs, a message in a later batch), which the encoder does not
 * make
 */

static ermes_network_t conf_net = { .zone = 2, .country = 1, .operator = 2, .pa = 0, .bai = 0, .fsi = 30 };
#define CONF_NEC	0x318E40u
#define CONF_FILL	ermes_bch_encode(0x2AAAA)	/* a codeword, not MD, no EOM (-5 p.17 "FILL") */
#define CONF_PAD	(ERMES_CTA_IA)			/* -5 p.17: an IA higher than the receiver's */

/* a batch on channel 9 in slot 'slot': its IAs (18 bits each), the APTs to the codeblock
 * boundary, the message partition's first codewords, MD to the end */
static int conf_batch(uint8_t *b, int slot, const uint32_t *ia, int nia, const uint32_t *msg, int nmsg)
{
	ermes_batch_time_t t = { .cycle = 30, .ssn = 0, .slot = slot, .batch = ermes_channel_batch(9, slot) };
	ermes_ssi_t ssi = { .ssit = 1, .ssif = 0 };
	uint32_t part[190];
	int n = 0, i, j, k, len = ermes_batch_length(slot), cb;

#define PUT(w) do { for (j = 29; j >= 0; j--) b[n++] = ((w) >> j) & 1; } while (0)
	PUT(ERMES_PREAMBLE);
	PUT(ERMES_SW1);
	PUT(ermes_bch_encode(ermes_si_word1(&conf_net, 0)));
	PUT(ermes_bch_encode(ermes_si_word2(&conf_net, &t)));
	PUT(ermes_bch_encode(ermes_ssi_word(&ssi)));
	for (i = 0; i < nia; i++)
		PUT(ermes_bch_encode(ia[i]));
	for (i = 0; i < ermes_apt_count(nia); i++)
		PUT(ERMES_APT);
#undef PUT
	cb = ermes_max_codeblocks(slot, nia);
	for (k = 0; k < cb * 9; k++)
		part[k] = (k < nmsg) ? msg[k] : ERMES_MD;
	for (k = 0; k < cb; k++) {
		ermes_interleave(part + 9 * k, b + n);
		n += 270;
	}
	return (n == len * 30) ? n : -1;
}

/* header codewords: Fig.10, VIF given */
static int conf_header(uint32_t *w, uint32_t la, int number, int eb, int aii, int vif)
{
	uint64_t h = ((uint64_t)la << 14) | ((uint64_t)number << 9) | ((uint64_t)eb << 8) | ((uint64_t)aii << 7) | vif;

	w[0] = ermes_bch_encode(h >> 18);
	w[1] = ermes_bch_encode(h & 0x3FFFF);
	return 2;
}

/* alphanumeric data codewords of a text, standard fill */
static int conf_alpha(uint32_t *w, const char *text)
{
	ermes_page_t p = { .category = ERMES_CAT_ALPHA, .text = text };
	uint32_t d[64];
	int n = ermes_data_words(&p, d, 64), i;

	for (i = 0; i < n; i++)
		w[i] = ermes_bch_encode(d[i]);
	return n;
}

static uint8_t conf_stream[40 * 190 * 30];
static int conf_pos;

static void conf_add(int slot, const uint32_t *ia, int nia, const uint32_t *msg, int nmsg)
{
	int n = conf_batch(conf_stream + conf_pos, slot, ia, nia, msg, nmsg);

	CHECK(n > 0, "conformance batch slot %d built", slot);
	conf_pos += (n > 0) ? n : 0;
}

static void conf_run(int lose_after_bits)
{
	ermes_rx_t rx;
	int i;

	rx_ngot = rx_nbatch = 0;
	ermes_rx_init(&rx, 9, &pager_charset_ermes_generic_generic_standard, NULL, rx_batch_cb, rx_msg_cb);
	for (i = 0; i < conf_pos; i++)
		ermes_rx_bit(&rx, (lose_after_bits && i >= lose_after_bits) ? ((i * 7) >> 3) & 1 : conf_stream[i]);
	/* noise after: the lock goes */
	for (i = 0; i < 3 * 190 * 30; i++)
		ermes_rx_bit(&rx, (i * 13 >> 2) & 1);
}

static void test_rx_conformance(void)
{
	uint32_t ia_nec = CONF_NEC >> 4, pads[4] = { CONF_PAD, CONF_PAD, CONF_PAD, CONF_PAD }, ia[4], msg[190];
	const ermes_rx_msg_t *m;
	int n, i, slot;

	/* -5 7.1.3 Fig.5: batch 1 "3*PAD IA APT FILL MD" with the MD last, batch 2 "PAD 4*APT MESSAGE MD" */
	conf_pos = 0;
	ia[0] = pads[0]; ia[1] = pads[1]; ia[2] = pads[2]; ia[3] = ia_nec;
	n = ermes_max_codeblocks(0, 4) * 9;
	for (i = 0; i < n - 1; i++)
		msg[i] = CONF_FILL;
	msg[n - 1] = ERMES_MD;
	conf_add(0, ia, 4, msg, n);
	n = conf_header(msg, CONF_NEC, 5, 0, 0, (ERMES_CAT_ALPHA << 4));
	n += conf_alpha(msg + n, "Fig 5 next batch");
	msg[n++] = ERMES_MD;
	conf_add(1, pads, 1, msg, n);
	conf_run(0);
	m = rx_find(CONF_NEC, ERMES_RX_MESSAGE);
	CHECK(m && !strcmp(m->text, "Fig 5 next batch") && rx_ngot == 1, "-5 7.1.3 message in the next batch: %s (%d)",
	      m ? ermes_rx_msg_str(m) : "missing", rx_ngot);

	/* -5 7.1.4/7.1.5: IA in one batch, the message 15 batches later (received) / 16 later (not) */
	for (int later = 15; later <= 16; later++) {
		conf_pos = 0;
		conf_add(0, &ia_nec, 1, NULL, 0);
		for (slot = 1; slot < later; slot++)
			conf_add(slot % 16, NULL, 0, NULL, 0);
		n = conf_header(msg, CONF_NEC, 6, 0, 0, (ERMES_CAT_ALPHA << 4));
		n += conf_alpha(msg + n, "late");
		msg[n++] = ERMES_MD;
		/* "4*PAD APT FILL MD MESSAGE MD" */
		{
			uint32_t m2[64];
			int k = 0;

			m2[k++] = CONF_FILL;
			m2[k++] = ERMES_MD;
			memcpy(m2 + k, msg, n * sizeof(uint32_t));
			conf_add(later % 16, pads, 4, m2, k + n);
		}
		conf_run(0);
		m = rx_find(CONF_NEC, ERMES_RX_MESSAGE);
		if (later == 15)
			CHECK(m && !strcmp(m->text, "late"), "-5 7.1.4 message 15 batches after the IA received");
		else
			CHECK(!m && rx_ngot == 0, "-5 7.1.5 message 16 batches after the IA not received (%d)", rx_ngot);
	}

	/* -5 7.8 Fig.16 with real FILL: pointer with the IA, 14 batches of FILL, the group text in the
	 * 15th; and 7.9: in the 17th the member's search is over */
	for (int later = 15; later <= 17; later += 2) {
		uint32_t cta = (ERMES_CTA_IA << 4) | 4;

		conf_pos = 0;
		ia[0] = pads[0]; ia[1] = pads[1]; ia[2] = pads[2]; ia[3] = ia_nec;
		msg[0] = ERMES_MD;
		n = 1 + conf_header(msg + 1, CONF_NEC, 7, 0, 1, (ERMES_AIT_CTAP << 4) | 4);
		msg[n++] = ERMES_MD;
		for (i = 0; i < 20; i++)
			msg[n++] = CONF_FILL;
		conf_add(0, ia, 4, msg, n);
		for (slot = 1; slot < later; slot++) {
			for (i = 0; i < 40; i++)
				msg[i] = CONF_FILL;
			conf_add(slot % 16, NULL, 0, msg, 40);
		}
		msg[0] = CONF_FILL;
		msg[1] = ERMES_MD;
		n = 2 + conf_header(msg + 2, cta, 0, 0, 0, (ERMES_CAT_ALPHA << 4));
		n += conf_alpha(msg + n, "group text");
		msg[n++] = ERMES_MD;
		conf_add(later % 16, pads, 4, msg, n);
		conf_run(0);
		m = rx_find(cta, ERMES_RX_GROUP);
		if (later == 15)
			CHECK(m && !strcmp(m->text, "group text") && m->members == 1 && m->member[0] == CONF_NEC
			      && rx_ngot == 2, "-5 7.8 group call with FILL: %s (%d messages)", m ? ermes_rx_msg_str(m) : "missing", rx_ngot);
		else
			CHECK(m && m->members == 0, "-5 7.9 group text after the search: no member (%d)", m ? m->members : -1);
	}

	/* -5 7.6 Fig.15: "MESSAGE EOM MD 3*CHARACTERS": the characters after the MD are no message */
	conf_pos = 0;
	msg[0] = ERMES_MD;
	n = 1 + conf_header(msg + 1, CONF_NEC, 8, 0, 0, (ERMES_CAT_ALPHA << 4));
	n += conf_alpha(msg + n, "ABCDEFGH");
	msg[n++] = ERMES_MD;
	n += conf_alpha(msg + n, "XYZ");
	conf_add(0, &ia_nec, 1, msg, n);
	conf_run(0);
	m = rx_find(CONF_NEC, ERMES_RX_MESSAGE);
	CHECK(m && !strcmp(m->text, "ABCDEFGH") && rx_ngot == 1 && !m->bad_fill, "-5 7.6 alpha termination: %d messages", rx_ngot);

	/* not the fill after the EOM: shown apart, with a warning */
	conf_pos = 0;
	{
		ermes_page_t p = { .category = ERMES_CAT_ALPHA, .text = "AB" };
		uint32_t d[8];
		int nd = ermes_data_words(&p, d, 8);

		msg[0] = ERMES_MD;
		n = 1 + conf_header(msg + 1, CONF_NEC, 9, 0, 0, (ERMES_CAT_ALPHA << 4));
		/* A B EOM fill: bits 21..27 (the fill's first 7, in the second word at 3..9) become 'X' */
		d[1] = (d[1] & ~(0x7Fu << 8)) | (0x58u << 8);
		for (i = 0; i < nd; i++)
			msg[n++] = ermes_bch_encode(d[i]);
		msg[n++] = ERMES_MD;
	}
	conf_add(0, &ia_nec, 1, msg, n);
	conf_run(0);
	m = rx_find(CONF_NEC, ERMES_RX_MESSAGE);
	CHECK(m && !strcmp(m->text, "AB") && m->bad_fill && strchr(m->after_eom, 'X'), "not the fill after the EOM: %s",
	      m ? ermes_rx_msg_str(m) : "missing");

	/* -4 p.20, p.34: long message, AIF = category 2 + length 16 (the p.34 example: 10000 bits),
	 * AIN = urgent + alert; and EB = 1 with an AIF right after the 13 bit OPID (p.22) */
	conf_pos = 0;
	msg[0] = ERMES_MD;
	n = 1 + conf_header(msg + 1, CONF_NEC, 10, 0, 1, (0 << 4) | 0xC);
	msg[n++] = ermes_bch_encode((ERMES_CAT_ALPHA << 16) | 10000);
	n += conf_alpha(msg + n, "part one");
	msg[n++] = ERMES_MD;
	{
		int opid = (2 << 10) | (1 << 3) | 3;
		uint64_t bits = ((uint64_t)opid << 11) | (1u << 10) | (5u << 7) | (0u << 2) | ERMES_CAT_ALPHA; /* 24 bits */

		n += conf_header(msg + n, 0x318E50, 3, 1, 1, (ERMES_AIT_MISC << 4) | 2);
		msg[n++] = ermes_bch_encode(bits >> 6);
		msg[n++] = ermes_bch_encode((bits & 63) << 12);
		n += conf_alpha(msg + n, "visitor");
		msg[n++] = ERMES_MD;
	}
	ia[0] = 0x318E5; ia[1] = ia_nec;
	conf_add(0, ia, 2, msg, n);
	conf_run(0);
	m = rx_find(CONF_NEC, ERMES_RX_SUBMESSAGE);
	CHECK(m && m->link_counter == 10000 && m->category == ERMES_CAT_ALPHA && m->urgent && m->alert == 4
	      && !strcmp(m->text, "part one"), "long message AIF: %s", m ? ermes_rx_msg_str(m) : "missing");
	m = rx_find(0x318E50, ERMES_RX_MESSAGE);
	CHECK(m && m->external && m->opid == ((2 << 10) | (1 << 3) | 3) && m->urgent && m->alert == 5
	      && !strcmp(m->text, "visitor"), "EB with an AIF after the OPID: %s", m ? ermes_rx_msg_str(m) : "missing");

	/* numbers: a repeat (-5 7.7) and a gap (8.2) */
	conf_pos = 0;
	for (i = 0; i < 3; i++) {
		static const int num[3] = { 11, 11, 14 };

		msg[0] = ERMES_MD;
		n = 1 + conf_header(msg + 1, CONF_NEC, num[i], 0, 0, (ERMES_CAT_ALPHA << 4));
		n += conf_alpha(msg + n, "same");
		msg[n++] = ERMES_MD;
		conf_add(0, &ia_nec, 1, msg, n);
		conf_add(1, NULL, 0, NULL, 0);
	}
	conf_run(0);
	CHECK(rx_ngot == 3 && rx_got[1].repeat == 1 && rx_got[2].missing_from == 12 && rx_got[2].missing_to == 13,
	      "repeat and gap: %d messages, repeat %d, missing %d..%d", rx_ngot, rx_ngot > 1 ? rx_got[1].repeat : -1,
	      rx_ngot > 2 ? rx_got[2].missing_from : -1, rx_ngot > 2 ? rx_got[2].missing_to : -1);

	/* the lock lost while a message runs on: reported with <..> */
	conf_pos = 0;
	{
		char longtext[500];

		memset(longtext, 'L', 499);
		longtext[499] = '\0';
		msg[0] = ERMES_MD;
		n = 1 + conf_header(msg + 1, CONF_NEC, 15, 0, 0, (ERMES_CAT_ALPHA << 4));
		{
			ermes_page_t p = { .category = ERMES_CAT_ALPHA, .text = longtext };
			uint32_t d[400];
			int nd = ermes_data_words(&p, d, 400);

			for (i = 0; i < nd && n < ermes_max_codeblocks(0, 1) * 9; i++)
				msg[n++] = ermes_bch_encode(d[i]);
		}
		conf_add(0, &ia_nec, 1, msg, n);
		conf_run(conf_pos);	/* nothing after this batch */
	}
	m = rx_find(CONF_NEC, ERMES_RX_MESSAGE);
	CHECK(m && m->missing_part && strstr(m->text, "<..>") && !strncmp(m->text, "LLLL", 4),
	      "rest not received: %s", m ? ermes_rx_msg_str(m) : "missing");

	printf("receiver conformance done\n");
}

/* ERMES_RX_FILE=x.bits (ASCII 0/1) or x.sym (bytes 0..3 = lowest..highest level) [ERMES_RX_CHANNEL]:
 * decode a capture, print everything */
static void test_rx_file(const char *file)
{
	static const int level_dibit[4] = { 0, 1, 3, 2 };
	ermes_rx_t rx;
	FILE *fp = fopen(file, "rb");
	int c, sym = strstr(file, ".sym") != NULL;

	if (!fp) {
		printf("cannot open %s\n", file);
		failed++;
		return;
	}
	rx_verbose = 1;
	rx_ngot = rx_nbatch = 0;
	ermes_rx_init(&rx, getenv("ERMES_RX_CHANNEL") ? atoi(getenv("ERMES_RX_CHANNEL")) : 0,
		      &pager_charset_ermes_generic_generic_standard, NULL, rx_batch_cb, rx_msg_cb);
	while ((c = fgetc(fp)) != EOF) {
		if (sym)
			ermes_rx_dibit(&rx, level_dibit[c & 3]);
		else if (c == '0' || c == '1')
			ermes_rx_bit(&rx, c - '0');
	}
	fclose(fp);
	printf("%s: %d batches, %d messages (texts: %d complete, %d damaged, %d incomplete; %d over several batches)%s\n",
	       file, rx.batches, rx.messages, rx.complete, rx.damaged, rx.incomplete, rx.runs_on,
	       rx.inverted_seen ? " (a sync was seen with inverted polarity)" : "");
}

/* ERMES_RX_WAV=x.wav (PCM 16 bit, first channel, full scale = outer deviation) [ERMES_RX_CHANNEL]
 * [ERMES_RX_POLARITY=-1]: the DSP and the decoder on a wave file */
static void test_rx_wav(const char *file)
{
	ermes_rx_t rx;
	ermes_dsp_rx_t dsp;
	FILE *fp = fopen(file, "rb");
	unsigned char hdr[44];
	int16_t pcm[4096];
	double x[4096];
	int channels, rate, n, i;

	if (!fp || fread(hdr, 1, 44, fp) != 44) {
		printf("cannot read %s\n", file);
		failed++;
		return;
	}
	channels = hdr[22] | (hdr[23] << 8);
	rate = hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24);
	rx_verbose = 1;
	rx_ngot = rx_nbatch = 0;
	ermes_rx_init(&rx, getenv("ERMES_RX_CHANNEL") ? atoi(getenv("ERMES_RX_CHANNEL")) : 0,
		      &pager_charset_ermes_generic_generic_standard, NULL, rx_batch_cb, rx_msg_cb);
	ermes_dsp_rx_init(&dsp, &rx, rate, getenv("ERMES_RX_POLARITY") ? atof(getenv("ERMES_RX_POLARITY")) : 1.0);
	while ((n = fread(pcm, 2 * channels, 4096 / channels, fp)) > 0) {
		for (i = 0; i < n; i++)
			x[i] = pcm[i * channels] / 32767.0;
		ermes_dsp_rx_samples(&dsp, x, n);
	}
	fclose(fp);
	printf("%s: %d Hz, %llu symbols, %d batches, %d messages (texts: %d complete, %d damaged, %d incomplete; %d over several batches); dc %.3f gain %.3f%s\n",
	       file, rate, (unsigned long long)dsp.symbols, rx.batches, rx.messages, rx.complete, rx.damaged, rx.incomplete,
	       rx.runs_on, dsp.dc, dsp.gain,
	       rx.inverted_seen ? " (a sync was seen with inverted polarity)" : "");
}

int main(void)
{
	if (getenv("ERMES_RX_WAV")) {
		test_rx_wav(getenv("ERMES_RX_WAV"));
		return failed ? 1 : 0;
	}
	if (getenv("ERMES_RX_FILE")) {
		test_rx_file(getenv("ERMES_RX_FILE"));
		return failed ? 1 : 0;
	}
	test_bch_vectors();
	test_bch_correction();
	test_fixed_words();
	test_interleave();
	test_batch_numbers();
	test_channels();
	test_fsi();
	test_premodulation_filter();
	test_frame();
	test_continuation();
	test_rx();
	test_rx_conformance();
	if (failed) {
		printf("%d check(s) FAILED\n", failed);
		return 1;
	}
	printf("all ERMES code checks passed\n");
	return 0;
}
