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

int main(void)
{
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
	if (failed) {
		printf("%d check(s) FAILED\n", failed);
		return 1;
	}
	printf("all ERMES code checks passed\n");
	return 0;
}
