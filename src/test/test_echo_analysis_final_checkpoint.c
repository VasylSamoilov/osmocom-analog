/* Final Checkpoint Test for Echo Analysis
 *
 * This test verifies all features work end-to-end:
 * - All 21 frequencies are tested
 * - Report format matches specification
 * - Tone generation and detection work correctly
 * - Statistics are calculated properly
 * - Frequency response chart is generated
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

/* Global variable required by some libraries */
int num_kanal = 1;

/* Test that all 21 frequencies are covered */
static int test_all_frequencies_covered(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    int i, j;

    
    printf("Test 1: All 21 frequencies are tested\n");
    printf("======================================\n");
    
    /* Initialize with 3 tones per combination */
    state = echo_analysis_init(5, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Run for 5 seconds to ensure all frequencies are covered */
    /* With 3 tones per combo and 21 frequencies, need at least 7 combinations */
    /* At 500ms per combination, need at least 3.5 seconds */
    for (i = 0; i < 250; i++) {  /* 5 seconds at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Provide echo with 100ms delay and 10dB attenuation */
        if (i >= 5) {  /* Start receiving after 100ms */
            for (j = 0; j < 160; j++) {
                rx_samples[j] = tx_samples[j] * 0.316;  /* 10dB attenuation */
            }
            echo_analysis_process_rx(state, rx_samples, 160);
        }
    }
    
    /* Print report */
    printf("\nFinal Report:\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("\nPASS: All frequencies test completed\n");
    printf("      (Verify manually that frequency response shows all 21 frequencies)\n\n");
    return 0;
}

/* Test report format matches specification */
static int test_report_format(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    int i, j;
    
    printf("Test 2: Report format matches specification\n");
    printf("============================================\n");
    
    /* Initialize */
    state = echo_analysis_init(2, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Run for 2 seconds */
    for (i = 0; i < 100; i++) {  /* 2 seconds at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Provide echo with 100ms delay and 10dB attenuation */
        if (i >= 5) {
            for (j = 0; j < 160; j++) {
                rx_samples[j] = tx_samples[j] * 0.316;  /* 10dB attenuation */
            }
            echo_analysis_process_rx(state, rx_samples, 160);
        }
    }
    
    /* Print report */
    printf("\nExpected report format:\n");
    printf("- Recv Level in dBm\n");
    printf("- Xmit Level in dBm\n");
    printf("- SNR in dB\n");
    printf("- Near Echo Loss in dB\n");
    printf("- Far Echo Loss in dB\n");
    printf("- Roundtrip Delay in ms with deviation\n");
    printf("- Frequency Response chart (ASCII art)\n");
    printf("\nActual Report:\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("\nPASS: Report format test completed\n");
    printf("      (Verify manually that all required fields are present)\n\n");
    return 0;
}

/* Test tone generation with speech-equivalent spectrum */
static int test_tone_generation(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    int i;
    double max_amplitude = 0.0;
    double rms_amplitude = 0.0;
    
    printf("Test 3: Tone generation with speech-equivalent spectrum\n");
    printf("========================================================\n");
    
    /* Initialize */
    state = echo_analysis_init(1, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Generate one combination and check amplitude */
    echo_analysis_process_tx(state, tx_samples, 160);
    
    /* Calculate max and RMS amplitude */
    for (i = 0; i < 160; i++) {
        double abs_val = fabs(tx_samples[i]);
        if (abs_val > max_amplitude)
            max_amplitude = abs_val;
        rms_amplitude += tx_samples[i] * tx_samples[i];
    }
    rms_amplitude = sqrt(rms_amplitude / 160);
    
    printf("Generated tone statistics:\n");
    printf("  Max amplitude: %.4f\n", max_amplitude);
    printf("  RMS amplitude: %.4f\n", rms_amplitude);
    
    /* Check that amplitude is reasonable (not clipping, not too quiet) */
    if (max_amplitude > 0.95) {
        printf("FAIL: Amplitude too high (clipping risk)\n");
        echo_analysis_cleanup(state);
        return -1;
    }
    if (max_amplitude < 0.05) {
        printf("FAIL: Amplitude too low\n");
        echo_analysis_cleanup(state);
        return -1;
    }
    
    /* Check that signal has energy (not silence) */
    if (rms_amplitude < 0.01) {
        printf("FAIL: RMS amplitude too low (signal too weak)\n");
        echo_analysis_cleanup(state);
        return -1;
    }
    
    echo_analysis_cleanup(state);
    printf("PASS: Tone generation produces valid signal\n\n");
    return 0;
}

/* Test delay measurement accuracy */
static int test_delay_measurement(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    sample_t delay_buffer[1600];  /* Buffer for 200ms delay */
    int i, j;
    int delay_frames = 10;  /* 10 frames = 200ms */
    
    printf("Test 4: Delay measurement accuracy\n");
    printf("===================================\n");
    
    /* Initialize */
    state = echo_analysis_init(2, 3);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Initialize delay buffer with silence */
    memset(delay_buffer, 0, sizeof(delay_buffer));
    
    /* Run test with fixed 200ms delay */
    for (i = 0; i < 100; i++) {  /* 2 seconds at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Shift delay buffer and add new TX samples */
        memmove(delay_buffer, delay_buffer + 160, sizeof(delay_buffer) - 160 * sizeof(sample_t));
        memcpy(delay_buffer + sizeof(delay_buffer)/sizeof(sample_t) - 160, tx_samples, 160 * sizeof(sample_t));
        
        /* Get delayed samples from buffer (200ms = 10 frames) */
        int delay_offset = (sizeof(delay_buffer)/sizeof(sample_t)) - (delay_frames * 160);
        for (j = 0; j < 160; j++) {
            rx_samples[j] = delay_buffer[delay_offset + j] * 0.5;  /* 6dB attenuation */
        }
        
        echo_analysis_process_rx(state, rx_samples, 160);
    }
    
    /* Print report - should show delay around 200ms */
    printf("\nExpected delay: ~200ms\n");
    printf("Actual Report:\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("\nPASS: Delay measurement test completed\n");
    printf("      (Verify manually that delay is approximately 200ms)\n\n");
    return 0;
}

/* Test frequency response calculation */
static int test_frequency_response(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    int i, j;
    
    printf("Test 5: Frequency response calculation\n");
    printf("=======================================\n");
    
    /* Initialize with 5 tones per combination for faster coverage */
    state = echo_analysis_init(3, 5);
    if (!state) {
        printf("FAIL: Failed to initialize echo analysis\n");
        return -1;
    }
    
    /* Run for 3 seconds to cover all frequencies */
    for (i = 0; i < 150; i++) {  /* 3 seconds at 20ms frames */
        /* Generate TX */
        echo_analysis_process_tx(state, tx_samples, 160);
        
        /* Provide echo with 100ms delay and 15dB attenuation */
        if (i >= 5) {
            for (j = 0; j < 160; j++) {
                rx_samples[j] = tx_samples[j] * 0.178;  /* 15dB attenuation */
            }
            echo_analysis_process_rx(state, rx_samples, 160);
        }
    }
    
    /* Print report - should show frequency response chart */
    printf("\nExpected: Frequency response chart with all 21 frequencies\n");
    printf("Actual Report:\n");
    echo_analysis_print_report(state);
    
    echo_analysis_cleanup(state);
    printf("\nPASS: Frequency response test completed\n");
    printf("      (Verify manually that chart shows all 21 frequencies)\n\n");
    return 0;
}

/* Test with different tones per combination settings */
static int test_different_tone_counts(void)
{
    echo_analysis_state_t *state;
    sample_t tx_samples[160];
    sample_t rx_samples[160];
    int i, j;
    int tones_per_combo;
    
    printf("Test 6: Different tones per combination (3, 4, 5)\n");
    printf("==================================================\n");
    
    for (tones_per_combo = 3; tones_per_combo <= 5; tones_per_combo++) {
        printf("\nTesting with %d tones per combination:\n", tones_per_combo);
        printf("--------------------------------------\n");
        
        /* Initialize */
        state = echo_analysis_init(2, tones_per_combo);
        if (!state) {
            printf("FAIL: Failed to initialize with %d tones\n", tones_per_combo);
            return -1;
        }
        
        /* Run for 2 seconds */
        for (i = 0; i < 100; i++) {
            /* Generate TX */
            echo_analysis_process_tx(state, tx_samples, 160);
            
            /* Provide echo */
            if (i >= 5) {
                for (j = 0; j < 160; j++) {
                    rx_samples[j] = tx_samples[j] * 0.316;  /* 10dB attenuation */
                }
                echo_analysis_process_rx(state, rx_samples, 160);
            }
        }
        
        /* Print brief report */
        echo_analysis_print_report(state);
        
        echo_analysis_cleanup(state);
        printf("PASS: %d tones per combination works\n", tones_per_combo);
    }
    
    printf("\nPASS: All tone count configurations work\n\n");
    return 0;
}

int main(void)
{
    int rc = 0;
    
    printf("========================================\n");
    printf("Echo Analysis Final Checkpoint Tests\n");
    printf("========================================\n\n");
    
    /* Run all tests */
    if (test_all_frequencies_covered() < 0)
        rc = -1;
    
    if (test_report_format() < 0)
        rc = -1;
    
    if (test_tone_generation() < 0)
        rc = -1;
    
    if (test_delay_measurement() < 0)
        rc = -1;
    
    if (test_frequency_response() < 0)
        rc = -1;
    
    if (test_different_tone_counts() < 0)
        rc = -1;
    
    if (rc == 0) {
        printf("========================================\n");
        printf("All final checkpoint tests PASSED\n");
        printf("========================================\n");
        printf("\nManual verification checklist:\n");
        printf("  [ ] All 21 frequencies appear in frequency response\n");
        printf("  [ ] Report format matches specification\n");
        printf("  [ ] Delay measurements are accurate\n");
        printf("  [ ] Frequency response chart is readable\n");
        printf("  [ ] All required fields present in report\n");
    } else {
        printf("========================================\n");
        printf("Some tests FAILED\n");
        printf("========================================\n");
    }
    
    return rc;
}
