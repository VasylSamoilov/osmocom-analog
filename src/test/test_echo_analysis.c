/* Test echo analysis
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../libecho/echo_analysis.h"
#include "../libsample/sample.h"
#include "../liblogging/logging.h"

int num_kanal;

int main(void)
{
	echo_analysis_state_t *state;
	int duration_sec = 10;
	int tones_per_combo = 3;

	logging_init();

	printf("Testing echo analysis initialization...\n");

	/* Test 1: Initialize with default parameters */
	printf("\nTest 1: Initialize with duration=%d sec, tones_per_combo=%d\n", 
	       duration_sec, tones_per_combo);
	state = echo_analysis_init(duration_sec, tones_per_combo);
	if (!state) {
		printf("FAILED: echo_analysis_init returned NULL\n");
		return 1;
	}
	printf("PASSED: State initialized successfully\n");

	/* Test 2: Check if analysis is not complete immediately */
	printf("\nTest 2: Check if analysis is not complete immediately\n");
	if (echo_analysis_is_complete(state)) {
		printf("FAILED: Analysis should not be complete immediately\n");
		echo_analysis_cleanup(state);
		return 1;
	}
	printf("PASSED: Analysis is not complete immediately\n");

	/* Test 3: Cleanup */
	printf("\nTest 3: Cleanup\n");
	echo_analysis_cleanup(state);
	printf("PASSED: Cleanup successful\n");

	/* Test 4: Initialize with invalid parameters (should use defaults) */
	printf("\nTest 4: Initialize with invalid parameters\n");
	state = echo_analysis_init(-1, 10);  /* Invalid duration and tones */
	if (!state) {
		printf("FAILED: echo_analysis_init returned NULL with invalid params\n");
		return 1;
	}
	printf("PASSED: State initialized with defaults for invalid params\n");
	echo_analysis_cleanup(state);

	/* Test 5: Test process_tx and process_rx don't crash */
	printf("\nTest 5: Test process_tx and process_rx don't crash\n");
	state = echo_analysis_init(10, 3);
	if (!state) {
		printf("FAILED: echo_analysis_init returned NULL\n");
		return 1;
	}
	
	sample_t samples[160];  /* 20ms at 8kHz */
	memset(samples, 0, sizeof(samples));
	
	echo_analysis_process_tx(state, samples, 160);
	echo_analysis_process_rx(state, samples, 160);
	printf("PASSED: process_tx and process_rx executed without crash\n");
	
	echo_analysis_cleanup(state);

	printf("\n=== All tests passed ===\n");
	return 0;
}
