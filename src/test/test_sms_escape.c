/* <0xNN> escape round-trip test for the operator SMS text interface.
 *
 * The operator sends/receives SMS over a text FIFO, so any byte that is not
 * printable ASCII -- including 0x00 (the GSM/NMT '@' code point) and the 8-bit
 * NMT national letters 0x80..0xff -- is carried as the literal escape <0xNN>.
 * This mirrors the parse in deliver_sms() and the render in sms_submit() (both
 * in nmt.c) and checks that encode(decode(x)) == x for an arbitrary byte string.
 *
 * Self-contained (same style as test_amps_orders / test_linesignal): if the
 * transforms in nmt.c change, update these copies.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

static int failures;

static void assert(int cond, const char *why)
{
	printf("%s = %s\n", why, cond ? "TRUE" : "FALSE");
	if (!cond)
		failures++;
}

/* mirror of the <0xNN> parser in deliver_sms(): text -> bytes + length */
static int parse_escapes(const char *message, uint8_t *out, int outsz)
{
	const char *s = message;
	int n = 0;
	while (*s && n < outsz) {
		if (s[0] == '<' && s[1] == '0' && (s[2] == 'x' || s[2] == 'X')
		 && isxdigit((unsigned char)s[3]) && isxdigit((unsigned char)s[4]) && s[5] == '>') {
			out[n++] = (uint8_t)strtol(s + 1, NULL, 16);
			s += 6;
		} else {
			out[n++] = (uint8_t)*s++;
		}
	}
	return n;
}

/* mirror of the render in sms_submit(): bytes + length -> text */
static void render_escapes(const uint8_t *msg, int len, char *text, int textsz)
{
	int i, p = 0;
	for (i = 0; i < len && p < textsz - 7; i++) {
		uint8_t c = msg[i];
		if (c >= 0x20 && c < 0x7f && c != '<')
			text[p++] = (char)c;
		else
			p += snprintf(text + p, textsz - p, "<0x%02x>", c);
	}
	text[p] = '\0';
}

int main(void)
{
	printf("Testing <0xNN> escape round-trip for the SMS text interface\n\n");

	/* send parse: a message with 0x00, a national letter and plain text */
	const char *in = "<0x00>Hi<0xc0>!";
	uint8_t bytes[256];
	int n = parse_escapes(in, bytes, sizeof(bytes));

	assert(n == 5, "parsed length is 5 bytes");
	assert(bytes[0] == 0x00, "first byte is 0x00 (embedded NUL carried)");
	assert(bytes[1] == 'H' && bytes[2] == 'i', "plain ASCII carried verbatim");
	assert(bytes[3] == 0xc0, "national letter 0xc0 carried");
	assert(bytes[4] == '!', "trailing '!' carried");

	/* receive render of the same bytes */
	char text[256];
	render_escapes(bytes, n, text, sizeof(text));
	assert(!strcmp(text, "<0x00>Hi<0xc0>!"), "render reproduces the <0xNN> notation");

	/* full round-trip for every byte value 0..255 */
	uint8_t all[256];
	int i;
	for (i = 0; i < 256; i++)
		all[i] = (uint8_t)i;
	char rendered[2048];
	render_escapes(all, 256, rendered, sizeof(rendered));
	uint8_t back[256];
	int m = parse_escapes(rendered, back, sizeof(back));
	assert(m == 256, "round-trip preserves all 256 byte positions");
	assert(!memcmp(all, back, 256), "round-trip preserves every byte value 0x00..0xff");

	printf("\n%s\n", failures ? "FAILURES PRESENT ;-(" : "ALL PASS ;->");
	return failures ? 1 : 0;
}
