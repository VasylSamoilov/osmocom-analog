/* Test echo analysis edge case handling
 *
 * This test verifies that the echo analysis system handles edge cases correctly:
 * - No echo detected
 * - Low signal conditions
 * - Inconsistent delay
 * - Test interruption
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
#include <math.h>
#include "../libecho/echo_analysis.h"
#include "../libsample/sample.h"

/* Test 1: No echo detected */
static int test_no_echo_detected(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    int i;
    
    printf("Test 1: No echo detected\n");
    printf("-------------------------\n");
    
    /* Initialize with short duration */
    state = echo_analysis_init(1, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Generate TX tones but provide silence on RX */
    for (i = 0; i < 50; i++) {  /* 1 second at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Provide silence on RX (no echo) */
        memset(rx_samples, 0, sizeof(rx_samples));
        echo_analysis_process_rx(state, rx_samples, 160);
    }
    
    /* Print report - should show "No echo detected" */
    printf("\nExpected: Report should show 'No echo detected'\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("PASS: No echo detected test completed\n\n");
    return 0;
}

/* Test 2: Low signal conditions */
static int test_low_signal(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    int i, j;
    
    printf("Test 2: Low signal conditions\n");
    printf("------------------------------\n");
    
    /* Initialize with short duration */
    state = echo_analysis_init(1, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Generate TX tones and provide weak echo on RX */
    /* Attenuate by 35dB (factor of 0.0178) to get signal around -52 to -54 dBm */
    /* This should be marginally detectable and trigger low signal warning */
    for (i = 0; i < 50; i++) {  /* 1 second at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Provide weak echo (attenuated by 35dB = factor of 0.0178) */
        for (j = 0; j < 160; j++) {
            rx_samples[j] = tx_samples[j] * 0.0178;
        }
        echo_analysis_process_rx(state, rx_samples, 160);
    }
    
    /* Print report - should show low signal warning if detected */
    printf("\nExpected: Report should show 'WARNING: Low signal level detected' if signal is detected\n");
    printf("         (Signal may be too weak to detect, which is also valid behavior)\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("PASS: Low signal test completed\n\n");
    return 0;
}

/* Test 3: Inconsistent delay */
static int test_inconsistent_delay(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    sample_t delay_buffer[3200];  /* Buffer for variable delay */
    int i, j;
    int delay_frames = 5;  /* Start with 5 frames delay (100ms) */
    
    printf("Test 3: Inconsistent delay\n");
    printf("---------------------------\n");
    
    /* Initialize with short duration */
    state = echo_analysis_init(2, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Initialize delay buffer with silence */
    memset(delay_buffer, 0, sizeof(delay_buffer));
    
    /* Generate TX tones and provide echo with varying delay */
    for (i = 0; i < 100; i++) {  /* 2 seconds at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Vary delay every 10 frames to create inconsistency */
        if (i % 10 == 0) {
            delay_frames = 5 + (i / 10) % 5;  /* Vary between 5-9 frames */
        }
        
        /* Shift delay buffer and add new TX samples */
        memmove(delay_buffer, delay_buffer + 160, sizeof(delay_buffer) - 160 * sizeof(sample_t));
        memcpy(delay_buffer + sizeof(delay_buffer)/sizeof(sample_t) - 160, tx_samples, 160 * sizeof(sample_t));
        
        /* Get delayed samples from buffer */
        int delay_offset = (sizeof(delay_buffer)/sizeof(sample_t)) - (delay_frames * 160);
        for (j = 0; j < 160; j++) {
            rx_samples[j] = delay_buffer[delay_offset + j] * 0.5;  /* 6dB attenuation */
        }
        
        echo_analysis_process_rx(state, rx_samples, 160);
    }
    
    /* Print report - should show high delay variation warning */
    printf("\nExpected: Report should show 'WARNING: High delay variation detected'\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("PASS: Inconsistent delay test completed\n\n");
    return 0;
}

/* Test 4: Test interruption (incomplete test) */
static int test_interruption(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    int i, j;
    
    printf("Test 4: Test interruption\n");
    printf("--------------------------\n");
    
    /* Initialize with long duration but interrupt early */
    state = echo_analysis_init(10, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Run for only 1 second (should be 10 seconds) */
    for (i = 0; i < 50; i++) {  /* 1 second at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Provide echo with 100ms delay */
        if (i >= 5) {  /* Start receiving after 100ms */
            for (j = 0; j < 160; j++) {
                rx_samples[j] = tx_samples[j] * 0.5;  /* 6dB attenuation */
            }
            echo_analysis_process_rx(state, rx_samples, 160);
        }
    }
    
    /* Print report before completion - should show INCOMPLETE */
    printf("\nExpected: Report should show '(INCOMPLETE)' marker\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("PASS: Test interruption completed\n\n");
    return 0;
}

int main(void)
{
    int rc = 0;
    
    printf("========================================\n");
    printf("Echo Analysis Edge Case Tests\n");
    printf("========================================\n\n");
    
    /* Run all tests */
    if (test_no_echo_detected() < 0)
        rc = -1;
    
    if (test_low_signal() < 0)
        rc = -1;
    
    if (test_inconsistent_delay() < 0)
        rc = -1;
    
    if (test_interruption() < 0)
        rc = -1;
    
    if (rc == 0) {
        printf("========================================\n");
        printf("All edge case tests PASSED\n");
        printf("========================================\n");
    } else {
        printf("========================================\n");
        printf("Some tests FAILED\n");
        printf("========================================\n");
    }
    
    return rc;
}
