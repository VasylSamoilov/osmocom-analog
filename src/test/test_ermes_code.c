/* Test of the ERMES coding layer against the values printed in ETS 300 133-4 ed.2 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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

	rc = ermes_build_batch(&net, &t, &ssi, pages, 1, b);
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
	CHECK(ermes_build_batch(&net, &t, &ssi, &pages[1], 1, b) == -1, "wrong batch refused");
	t.slot = 15; t.batch = 1;
	rc = ermes_build_batch(&net, &t, &ssi, &pages[1], 1, b);
	CHECK(rc == 1, "slot 15 rc %d", rc);
	t.slot = 3; t.batch = 0;
	CHECK(ermes_build_batch(&net, &t, &ssi, NULL, 0, b) == 0, "empty batch");
	for (i = 5; i < 10; i++)
		CHECK(word_at(b, i) == ERMES_APT, "empty: APT at %d", i);
	printf("frame done\n");
}

int main(void)
{
	test_bch_vectors();
	test_bch_correction();
	test_fixed_words();
	test_interleave();
	test_batch_numbers();
	test_channels();
	test_frame();
	if (failed) {
		printf("%d check(s) FAILED\n", failed);
		return 1;
	}
	printf("all ERMES code checks passed\n");
	return 0;
}
