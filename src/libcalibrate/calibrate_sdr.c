/* SDR interface for calibration
 *
 * Provides simplified SDR access for raw IQ capture during calibration.
 * Uses the existing soapy/uhd infrastructure.
 *
 * (C) 2026 Osmocom-analog contributors
 * GPLv3
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "calibrate_sdr.h"
#include "../libsdr/sdr_config.h"

#ifdef HAVE_SOAPY
#include "../libsdr/soapy.h"
#endif

#ifdef HAVE_UHD
#include "../libsdr/uhd.h"
#endif

int calibrate_sdr_open(calibrate_sdr_t *sdr, const char *device_args,
                        double center_freq, double sample_rate,
                        double rx_gain, double bandwidth)
{
	int rc = -1;
	
	memset(sdr, 0, sizeof(*sdr));
	sdr->center_freq = center_freq;
	sdr->sample_rate = sample_rate;
	
#ifdef HAVE_SOAPY
	/* Try SoapySDR first */
	rc = soapy_open(0,                  /* channel */
	                device_args,         /* device args */
	                "",                  /* stream args */
	                "",                  /* tune args */
	                "",                  /* tx antenna (not used) */
	                "",                  /* rx antenna */
	                "",                  /* clock source */
	                0.0,                 /* tx freq (not used) */
	                center_freq,         /* rx freq */
	                0.0,                 /* lo offset */
	                sample_rate,         /* sample rate */
	                0.0,                 /* tx gain (not used) */
	                rx_gain,             /* rx gain */
	                bandwidth,           /* bandwidth */
	                0);                  /* timestamps */
	if (rc == 0) {
		sdr->use_soapy = 1;
		sdr->is_open = 1;
		
		/* Start streaming */
		rc = soapy_start();
		if (rc < 0) {
			soapy_close();
			sdr->is_open = 0;
			return rc;
		}
		return 0;
	}
#endif

#ifdef HAVE_UHD
	/* Try UHD */
	rc = uhd_open(0,                   /* channel */
	              device_args,          /* device args */
	              "",                   /* stream args */
	              "",                   /* tune args */
	              "",                   /* tx antenna */
	              "",                   /* rx antenna */
	              "",                   /* clock source */
	              0.0,                  /* tx freq */
	              center_freq,          /* rx freq */
	              0.0,                  /* lo offset */
	              sample_rate,          /* sample rate */
	              0.0,                  /* tx gain */
	              rx_gain,              /* rx gain */
	              bandwidth,            /* bandwidth */
	              0);                   /* timestamps */
	if (rc == 0) {
		sdr->use_uhd = 1;
		sdr->is_open = 1;
		
		rc = uhd_start();
		if (rc < 0) {
			uhd_close();
			sdr->is_open = 0;
			return rc;
		}
		return 0;
	}
#endif

	(void)device_args;
	(void)center_freq;
	(void)sample_rate;
	(void)rx_gain;
	(void)bandwidth;
	
	fprintf(stderr, "calibrate_sdr: no SDR backend available\n");
	return -ENODEV;
}

int calibrate_sdr_receive(calibrate_sdr_t *sdr, float *iq_buffer, int max_samples)
{
	int total = 0;
	int retries = 0;
	const int max_retries = 100;  /* 100ms max wait */
	
	if (!sdr->is_open)
		return -ENODEV;
	
	/* Keep receiving until we have enough samples or timeout */
	while (total < max_samples && retries < max_retries) {
		int got = 0;
		
#ifdef HAVE_SOAPY
		if (sdr->use_soapy)
			got = soapy_receive(iq_buffer + total * 2, max_samples - total);
#endif

#ifdef HAVE_UHD
		if (sdr->use_uhd)
			got = uhd_receive(iq_buffer + total * 2, max_samples - total);
#endif

		if (got > 0) {
			total += got;
			retries = 0;  /* Reset retry counter on success */
		} else {
			/* Wait 1ms for more samples */
			usleep(1000);
			retries++;
		}
	}
	
	return total;
}

int calibrate_sdr_set_freq(calibrate_sdr_t *sdr, double center_freq)
{
	if (!sdr->is_open)
		return -ENODEV;
	
	sdr->center_freq = center_freq;
	
	/* Note: For frequency changes during scanning, we would need to 
	 * close and reopen. This is a limitation of the current soapy/uhd
	 * wrappers. For now, we'll return success but not actually change.
	 * 
	 * TODO: Add soapy_set_frequency() / uhd_set_frequency() functions
	 * to the low-level wrappers.
	 */
	
	fprintf(stderr, "calibrate_sdr: frequency change not yet implemented\n");
	return -ENOSYS;
}

void calibrate_sdr_close(calibrate_sdr_t *sdr)
{
	if (!sdr->is_open)
		return;

#ifdef HAVE_SOAPY
	if (sdr->use_soapy)
		soapy_close();
#endif

#ifdef HAVE_UHD
	if (sdr->use_uhd)
		uhd_close();
#endif

	memset(sdr, 0, sizeof(*sdr));
}
