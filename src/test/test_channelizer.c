/* Test channelizer library
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
#include "../libchannelizer/channelizer.h"

#define TEST_INPUT_RATE   1000000  /* 1 MHz input */
#define TEST_OUTPUT_RATE  125000   /* 125 kHz output (8x decimation) */
#define TEST_BLOCK_SIZE   1024

static int test_halfband_init(void)
{
	halfband_t hb;
	int rc;

	printf("Testing halfband_init...\n");

	/* Test valid orders */
	rc = halfband_init(&hb, HALFBAND_ORDER_32, HALFBAND_CENTER, 0);
	if (rc < 0) {
		printf("  FAIL: order 32 init failed\n");
		return -1;
	}
	halfband_exit(&hb);

	rc = halfband_init(&hb, HALFBAND_ORDER_48, HALFBAND_LOWER, 0);
	if (rc < 0) {
		printf("  FAIL: order 48 init failed\n");
		return -1;
	}
	halfband_exit(&hb);

	rc = halfband_init(&hb, HALFBAND_ORDER_64, HALFBAND_UPPER, 1);
	if (rc < 0) {
		printf("  FAIL: order 64 init failed\n");
		return -1;
	}
	halfband_exit(&hb);

	/* Test invalid order */
	rc = halfband_init(&hb, 47, HALFBAND_CENTER, 0);
	if (rc == 0) {
		printf("  FAIL: invalid order should fail\n");
		halfband_exit(&hb);
		return -1;
	}

	printf("  PASS\n");
	return 0;
}

static int test_halfband_decimation(void)
{
	halfband_t hb;
	sample_t i, q;
	int rc, count = 0, output_count = 0;

	printf("Testing halfband 2x decimation...\n");

	rc = halfband_init(&hb, HALFBAND_ORDER_48, HALFBAND_CENTER, 0);
	if (rc < 0) {
		printf("  FAIL: init failed\n");
		return -1;
	}

	/* Process 100 samples, expect ~50 outputs */
	for (count = 0; count < 100; count++) {
		i = sin(2.0 * M_PI * count / 10.0);  /* Test tone */
		q = cos(2.0 * M_PI * count / 10.0);
		if (halfband_process(&hb, &i, &q))
			output_count++;
	}

	halfband_exit(&hb);

	if (output_count != 50) {
		printf("  FAIL: expected 50 outputs, got %d\n", output_count);
		return -1;
	}

	printf("  PASS: %d inputs -> %d outputs\n", count, output_count);
	return 0;
}

static int test_channelizer_init(void)
{
	channelizer_t ch;
	int rc;

	printf("Testing channelizer_init...\n");

	rc = channelizer_init(&ch, TEST_INPUT_RATE, TEST_OUTPUT_RATE, 0, 0);
	if (rc < 0) {
		printf("  FAIL: init failed\n");
		return -1;
	}

	printf("  Input rate: %d Hz\n", ch.input_rate);
	printf("  Output rate: %d Hz\n", ch.output_rate);
	printf("  Decimation: %dx\n", ch.decimation);
	printf("  Stages: %d\n", ch.num_stages);
	printf("  Center offset: %d Hz\n", ch.center_offset);

	/* Channelizer should decimate to at least the requested output rate */
	if (ch.output_rate > TEST_OUTPUT_RATE * 2) {
		printf("  FAIL: output rate %d too high for request %d\n", 
		       ch.output_rate, TEST_OUTPUT_RATE);
		channelizer_exit(&ch);
		return -1;
	}

	if (ch.num_stages < 1) {
		printf("  FAIL: expected at least 1 stage\n");
		channelizer_exit(&ch);
		return -1;
	}

	channelizer_exit(&ch);

	printf("  PASS\n");
	return 0;
}

static int test_channelizer_process(void)
{
	channelizer_t ch;
	float *iq_in;
	sample_t *i_out, *q_out;
	int rc, i, out_count;

	printf("Testing channelizer_process...\n");

	rc = channelizer_init(&ch, TEST_INPUT_RATE, TEST_OUTPUT_RATE, 0, 0);
	if (rc < 0) {
		printf("  FAIL: init failed\n");
		return -1;
	}

	/* Allocate buffers */
	iq_in = malloc(TEST_BLOCK_SIZE * 2 * sizeof(float));
	i_out = malloc(TEST_BLOCK_SIZE * sizeof(sample_t));
	q_out = malloc(TEST_BLOCK_SIZE * sizeof(sample_t));

	if (!iq_in || !i_out || !q_out) {
		printf("  FAIL: allocation failed\n");
		channelizer_exit(&ch);
		return -1;
	}

	/* Generate test signal - tone at center frequency */
	for (i = 0; i < TEST_BLOCK_SIZE; i++) {
		double phase = 2.0 * M_PI * i * 50000.0 / TEST_INPUT_RATE;
		iq_in[i * 2] = cos(phase);      /* I */
		iq_in[i * 2 + 1] = sin(phase);  /* Q */
	}

	/* Process */
	out_count = channelizer_process(&ch, iq_in, TEST_BLOCK_SIZE, i_out, q_out);

	printf("  Input samples: %d\n", TEST_BLOCK_SIZE);
	printf("  Output samples: %d\n", out_count);
	printf("  Expected: %d\n", TEST_BLOCK_SIZE / ch.decimation);

	if (out_count != TEST_BLOCK_SIZE / ch.decimation) {
		printf("  FAIL: wrong output count\n");
		free(iq_in);
		free(i_out);
		free(q_out);
		channelizer_exit(&ch);
		return -1;
	}

	free(iq_in);
	free(i_out);
	free(q_out);
	channelizer_exit(&ch);

	printf("  PASS\n");
	return 0;
}

static int test_fast_math_mode(void)
{
	channelizer_t ch_normal, ch_fast;
	int rc;

	printf("Testing fast_math mode...\n");

	rc = channelizer_init(&ch_normal, TEST_INPUT_RATE, TEST_OUTPUT_RATE, 0, 0);
	if (rc < 0) {
		printf("  FAIL: normal mode init failed\n");
		return -1;
	}

	rc = channelizer_init(&ch_fast, TEST_INPUT_RATE, TEST_OUTPUT_RATE, 0, 1);
	if (rc < 0) {
		printf("  FAIL: fast_math mode init failed\n");
		channelizer_exit(&ch_normal);
		return -1;
	}

	if (ch_normal.fast_math != 0) {
		printf("  FAIL: normal mode should have fast_math=0\n");
		channelizer_exit(&ch_normal);
		channelizer_exit(&ch_fast);
		return -1;
	}

	if (ch_fast.fast_math != 1) {
		printf("  FAIL: fast mode should have fast_math=1\n");
		channelizer_exit(&ch_normal);
		channelizer_exit(&ch_fast);
		return -1;
	}

	channelizer_exit(&ch_normal);
	channelizer_exit(&ch_fast);

	printf("  PASS\n");
	return 0;
}

int main(int argc, char *argv[])
{
	int failed = 0;

	(void)argc;
	(void)argv;

	printf("=== Channelizer Library Tests ===\n\n");

	if (test_halfband_init() < 0)
		failed++;

	if (test_halfband_decimation() < 0)
		failed++;

	if (test_channelizer_init() < 0)
		failed++;

	if (test_channelizer_process() < 0)
		failed++;

	if (test_fast_math_mode() < 0)
		failed++;

	printf("\n=== Results: %s ===\n", failed ? "FAILED" : "ALL PASSED");

	return failed ? 1 : 0;
}
