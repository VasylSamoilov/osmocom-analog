/* test of the LO offset decision (sdr_lo.c): the automatic offset beyond
 * half the SDR rate, the band (channels = 0) off the DC where the driver
 * does not certainly move the LO out of the stream, room checks, rpitx. */

#include <stdio.h>
#include <math.h>
#include "../libsdr/sdr_lo.h"

static int failures;

static void check(int ok, const char *what)
{
	printf("  %-100s %s\n", what, ok ? "ok" : "FAILED");
	if (!ok)
		failures++;
}

static void one(const char *what, int rate, int given, double offset, int applies, int no_lo, int band,
		double want_lo, int want_outside, double want_shift, int want_no_room)
{
	sdr_lo_plan_t p;
	char text[256];

	sdr_lo_plan(&p, rate, given, offset, applies, no_lo, band);
	snprintf(text, sizeof(text), "%s: LO %.0f Hz%s, band shift %+.0f Hz%s", what, p.lo_offset, p.lo_outside ? " (outside)" : "",
		 p.band_shift, p.band_no_room ? " (no room)" : "");
	check(fabs(p.lo_offset - want_lo) < 0.5 && p.lo_outside == want_outside && fabs(p.band_shift - want_shift) < 0.5 && p.band_no_room == want_no_room, text);
}

int main(void)
{
	printf("=== LO offset decision ===\n");
	/* the programs' settings of the B210 runs: 1.92 MS/s, UHD (was -1 MHz) */
	one("NMT / AMPS on UHD 1.92 MS/s (channels)", 1920000, 0, 0, 1, 0, 0, -1036800, 1, 0, 0);
	one("osmoradio on UHD 1.92 MS/s, band 240 k", 1920000, 0, 0, 1, 0, 240000, -1036800, 1, 0, 0);
	one("the backend on UHD 1.92 MS/s (no band)", 1920000, 0, 0, 1, 0, 0, -1036800, 1, 0, 0);
	/* the old -1 MHz given: as before (outside at 1.92 MS/s) */
	one("-1 MHz given on UHD 1.92 MS/s", 1920000, 1, -1e6, 1, 0, 240000, -1e6, 1, 0, 0);
	/* at 5 MS/s the old -1 MHz was inside the stream; automatic is outside */
	one("UHD 5 MS/s (LimeSDR preset rate)", 5000000, 0, 0, 1, 0, 0, -2700000, 1, 0, 0);
	one("-1 MHz given on UHD 5 MS/s, band 480 k: LO inside, band above it", 5000000, 1, -1e6, 1, 0, 480000, -1e6, 0, -1e6 + 240000 + 200000, 0);
	/* SoapySDR: the driver decides at open; the band off the DC anyway */
	one("osmoradio on SoapySDR 1.92 MS/s, band 240 k", 1920000, 0, 0, 0, 0, 240000, -1036800, 0, 120000 + 76800, 0);
	one("osmoradio on SoapySDR 2.4 MS/s (RTL), band 480 k", 2400000, 0, 0, 0, 0, 480000, -1296000, 0, 240000 + 96000, 0);
	one("SoapySDR 0.96 MS/s, band 480 k: no room", 960000, 0, 0, 0, 0, 480000, -518400, 0, 0, 1);
	one("SoapySDR, band = SDR rate: no room", 240000, 0, 0, 0, 0, 240000, -129600, 0, 0, 1);
	one("SoapySDR channels: no band shift (placement)", 1920000, 0, 0, 0, 0, 0, -1036800, 0, 0, 0);
	/* 0 given: none; a band then off the DC on UHD too */
	one("0 given on UHD, band 240 k", 1920000, 1, 0, 1, 0, 240000, 0, 0, 120000 + 76800, 0);
	/* rpitx: no LO */
	one("rpitx, band", 1920000, 0, 0, 0, 1, 240000, 0, 0, 0, 0);
	one("rpitx, -1 MHz given", 1920000, 1, -1e6, 0, 1, 0, 0, 0, 0, 0);
	printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
