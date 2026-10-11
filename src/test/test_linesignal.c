/* Line-signal majority-acceptance conformance test (NMT Doc 450-1 para 4.7.2.3).
 *
 * A line signal repeats the signal number L(n) across the low hex characters
 * of a frame.  The standard accepts the signal when a MINORITY of characters
 * are corrupted: "at least two of three" (frames 5a/5b/5c, MTX->MS) or "at
 * least three" (the 5-character frames 13a/13b, MS->MTX).  The former
 * implementation required EVERY character identical and so rejected
 * noisy-but-valid frames the standard accepts; this test pins the spec rule.
 *
 * Self-contained on purpose (same style as test_amps_orders): it mirrors the
 * algorithm of nmt_line_signal_majority() in src/nmt/frame.c byte-for-byte so
 * it needs no NMT library link, and asserts against the standard's rule
 * directly.  If the implementation's vote logic changes, update this copy.
 *
 * (C) 2026 — osmocom-analog NMT conformance tests.
 */
#include <stdio.h>
#include <stdint.h>

static int failures;

static void assert(int condition, const char *why)
{
	printf("%s = %s\n", why, (condition) ? "TRUE" : "FALSE");
	if (!condition)
		failures++;
}

/* Mirror of nmt_line_signal_majority() in src/nmt/frame.c. */
static int line_signal_majority(uint64_t line_signal, int num_chars, int min_equal)
{
	int count[16] = { 0 };
	int i, best = -1, best_count = 0, v;

	for (i = 0; i < num_chars; i++) {
		v = (line_signal >> (i * 4)) & 0xf;
		if (++count[v] > best_count) {
			best_count = count[v];
			best = v;
		}
	}

	if (best_count >= min_equal)
		return best;

	return -1;
}

/* five characters (MS->MTX, frames 13a/13b), most significant first */
static uint64_t ls5(int a, int b, int c, int d, int e)
{
	return ((uint64_t)(a & 0xf) << 16) | ((b & 0xf) << 12)
	     | ((c & 0xf) << 8) | ((d & 0xf) << 4) | (e & 0xf);
}

/* three characters (MTX->MS, frames 5a/5b/5c) */
static uint64_t ls3(int a, int b, int c)
{
	return ((a & 0xf) << 8) | ((b & 0xf) << 4) | (c & 0xf);
}

int main(void)
{
	printf("Testing line-signal majority acceptance (NMT Doc 450-1 para 4.7.2.3)\n\n");

	/* MS->MTX, 5 characters, accept if >= 3 equal (para 4.7.2.3b) */

	assert(line_signal_majority(ls5(1, 1, 1, 1, 1), 5, 3) == 1,
		"5x L(1) clean -> accepted as 1");

	assert(line_signal_majority(ls5(1, 1, 1, 1, 7), 5, 3) == 1,
		"L(1) with 1 corrupt char -> accepted (4-of-5)");

	assert(line_signal_majority(ls5(14, 14, 14, 2, 7), 5, 3) == 14,
		"L(14) with 2 differing corrupt chars -> accepted (3-of-5)");

	assert(line_signal_majority(ls5(1, 1, 1, 9, 9), 5, 3) == 1,
		"3x L(1) vs 2x L(9) -> majority 1 accepted");

	assert(line_signal_majority(ls5(5, 5, 5, 0, 15), 5, 3) == 5,
		"L(5) exactly 3-of-5 at the boundary -> accepted");

	assert(line_signal_majority(ls5(1, 1, 9, 9, 7), 5, 3) < 0,
		"2x L(1), 2x L(9), 1x L(7): no majority -> rejected");

	assert(line_signal_majority(ls5(1, 2, 3, 4, 5), 5, 3) < 0,
		"all 5 characters different -> rejected");

	/* The former all-equal check REJECTED this; the standard ACCEPTS it.
	 * This is the behaviour the majority rule restores. */
	assert(line_signal_majority(ls5(1, 1, 1, 1, 2), 5, 3) == 1,
		"regression guard: 4-of-5 accepted (old all-equal code rejected it)");

	/* MTX->MS, 3 characters, accept if >= 2 equal (para 4.7.2.3a) */

	assert(line_signal_majority(ls3(9, 9, 9), 3, 2) == 9,
		"3x L(9) clean -> accepted as 9");

	assert(line_signal_majority(ls3(9, 9, 3), 3, 2) == 9,
		"L(9) with 1 corrupt char -> accepted (2-of-3)");

	assert(line_signal_majority(ls3(9, 3, 7), 3, 2) < 0,
		"all 3 characters different -> rejected (2-of-3 rule)");

	printf("\n%s\n", failures ? "FAILURES PRESENT ;-(" : "ALL PASS ;->");
	return failures ? 1 : 0;
}
