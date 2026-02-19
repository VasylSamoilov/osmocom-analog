/*
 * RDS Hex/Binary File Decoder Test Tool
 *
 * Feeds .hexrds and .rds files into the RDS decoder to validate decoding.
 * Supports formats used by RDS Spy, redsea, and RdsSurveyor2.
 *
 * Usage: osmordstest [-v level] [--rds-debug] [--rds-verbose] file1.hexrds file2.rds ...
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
#include <stdint.h>
#include <ctype.h>
#include "rds.h"
#include "../liblogging/logging.h"

static int rds_debug = 0;
static int rds_verbose = 0;
static int force_rbds = 0;
static int paging_enabled = 0;

/* Parse a single hex group line from .hexrds format.
 * Handles:
 *   - Plain:     "F203 001A 232F 5155"
 *   - Timestamp: "1704 20F1 2054 6865 @2018/04/24 03:32:22.607"
 *   - Stream:    "#S1 2001 7800 0500 0A00 @2002/03/29 09:48:31.29"
 *   - Missing:   "---- 0018 5F62 4D55" (---- = missing block)
 *
 * Returns 1 on success, 0 if line should be skipped (header, empty, etc.)
 */
static int parse_hexrds_line(const char *line, uint16_t blocks[4], uint8_t status[4])
{
	const char *p = line;

	/* Skip leading whitespace */
	while (*p && isspace((unsigned char)*p)) p++;

	/* Skip empty lines */
	if (*p == '\0' || *p == '\n' || *p == '\r')
		return 0;

	/* Skip XML-style header lines (RDS Spy format) */
	if (*p == '<')
		return 0;

	/* Skip #S stream markers -- advance past them to the hex data */
	if (*p == '#' && (p[1] == 'S' || p[1] == 's') && isdigit((unsigned char)p[2])) {
		p += 3;
		while (*p && isspace((unsigned char)*p)) p++;
	}

	/* Now expect 4 hex blocks separated by spaces */
	for (int i = 0; i < 4; i++) {
		while (*p && isspace((unsigned char)*p)) p++;

		if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '@')
			return 0; /* Not enough blocks */

		/* Check for missing block marker */
		if (p[0] == '-' && p[1] == '-' && p[2] == '-' && p[3] == '-') {
			blocks[i] = 0;
			status[i] = RDS_STATUS_NONE;
			p += 4;
		} else {
			/* Parse hex value */
			char *end;
			unsigned long val = strtoul(p, &end, 16);
			if (end == p)
				return 0; /* Parse failure */
			blocks[i] = (uint16_t)(val & 0xFFFF);
			status[i] = RDS_STATUS_VALID;
			p = end;
		}
	}

	return 1;
}

/* Process a .hexrds file */
static int process_hexrds(const char *filename, rds_decoder_t *rds)
{
	FILE *f = fopen(filename, "r");
	if (!f) {
		fprintf(stderr, "Cannot open %s: ", filename);
		perror("");
		return -1;
	}

	char line[512];
	uint16_t blocks[4];
	uint8_t status[4];
	int groups = 0;
	int skipped = 0;

	while (fgets(line, sizeof(line), f)) {
		if (parse_hexrds_line(line, blocks, status)) {
			rds_decoder_feed_group(rds, blocks, status);
			groups++;
		} else {
			skipped++;
		}
	}

	fclose(f);
	printf("%s: %d groups fed, %d lines skipped\n", filename, groups, skipped);
	return groups;
}

/* Process a binary .rds file (raw RDS bitstream, MSB first).
 * Each byte contains 8 sequential bits of the RDS bitstream.
 * The decoder's sync state machine finds block boundaries via CRC syndromes. */
static int process_rds_binary(const char *filename, rds_decoder_t *rds)
{
	FILE *f = fopen(filename, "rb");
	if (!f) {
		fprintf(stderr, "Cannot open %s: ", filename);
		perror("");
		return -1;
	}

	uint8_t buf[4096];
	size_t n;
	long total_bits = 0;

	while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
		for (size_t i = 0; i < n; i++) {
			uint8_t byte = buf[i];
			for (int j = 7; j >= 0; j--) {
				rds_decoder_feed_bit(rds, (byte >> j) & 1);
				total_bits++;
			}
		}
	}

	fclose(f);
	printf("%s: %ld bits fed (bitstream), ~%ld groups possible\n",
	       filename, total_bits, total_bits / 104);
	return (int)(total_bits / 104);  /* approximate group count */
}

/* Detect file type by extension */
static int is_binary_rds(const char *filename)
{
	const char *ext = strrchr(filename, '.');
	if (!ext) return 0;
	return (strcasecmp(ext, ".rds") == 0);
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [options] file1.hexrds [file2.rds ...]\n\n", prog);
	printf("RDS file decoder — validates decoder against captured RDS data.\n\n");
	printf("Options:\n");
	printf("  -v <level>    Set logging verbosity (0=debug .. 7=fatal, default=5/notice)\n");
	printf("  --rds-debug   Enable RDS decoder debug (raw hex codes, field extraction)\n");
	printf("  --rds-verbose Enable RDS decoder verbose (human-readable decoded fields)\n");
	printf("  --rbds        Force RBDS decoding (callsign lookup, US PTY names)\n");
	printf("  --paging      Enable Radio Paging decoding (Group 7A, 1A RPC, 13A)\n");
	printf("  --help        Show this help\n\n");
	printf("Supported formats:\n");
	printf("  .hexrds     Hex text (RDS Spy, redsea, RdsSurveyor2)\n");
	printf("  .rds        Binary bitstream (raw RDS bits, MSB first)\n\n");
	printf("Examples:\n");
	printf("  %s --rds-verbose docs/rds/20180424_KCRC.hexrds\n", prog);
	printf("  %s -v 0 --rds-debug --rds-verbose docs/rds/20091111_France_Info.rds\n", prog);
}

int main(int argc, char *argv[])
{
	int file_count = 0;
	const char *files[256];
	int total_groups = 0;

	/* Parse arguments */
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0) {
			if (i + 1 < argc) {
				if (parse_logging_opt(argv[++i]) < 0) {
					fprintf(stderr, "Invalid log level: %s\n", argv[i]);
					return 1;
				}
			} else {
				fprintf(stderr, "-v requires a level argument\n");
				return 1;
			}
		} else if (strcmp(argv[i], "--rds-debug") == 0) {
			rds_debug = 1;
		} else if (strcmp(argv[i], "--rds-verbose") == 0) {
			rds_verbose = 1;
		} else if (strcmp(argv[i], "--rbds") == 0) {
			force_rbds = 1;
		} else if (strcmp(argv[i], "--paging") == 0) {
			paging_enabled = 1;
		} else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
			print_usage(argv[0]);
			return 0;
		} else if (argv[i][0] == '-') {
			fprintf(stderr, "Unknown option: %s\n", argv[i]);
			print_usage(argv[0]);
			return 1;
		} else {
			if (file_count < 256)
				files[file_count++] = argv[i];
		}
	}

	if (file_count == 0) {
		print_usage(argv[0]);
		return 1;
	}

	/* Initialize logging framework.
	 * Default loglevel is LOGL_INFO. Use -v to override (e.g. -v 0 for debug).
	 * --rds-debug/--rds-verbose are separate flags that control what the
	 * RDS decoder emits -- they still need the logging level to be low enough
	 * for the messages to pass through the framework. */
	logging_init();

	/* Process each file with a fresh decoder */
	for (int i = 0; i < file_count; i++) {
		rds_decoder_t rds;
		int rc;

		printf("\n=== Processing: %s ===\n", files[i]);

		/* Init decoder: samplerate=0 (unused for feed_group).
		 * Set emphasis to match mode flag -- no actual radio, so avoid
		 * spurious emphasis/mode mismatch notices. */
		rc = rds_decoder_init(&rds, 250000.0, rds_debug, rds_verbose, force_rbds ? 75.0 : 50.0, force_rbds);
		if (rc) {
			fprintf(stderr, "Failed to init decoder for %s\n", files[i]);
			continue;
		}
		rds.paging_enabled = paging_enabled;

		int groups;
		if (is_binary_rds(files[i]))
			groups = process_rds_binary(files[i], &rds);
		else
			groups = process_hexrds(files[i], &rds);

		if (groups > 0) {
			total_groups += groups;
			rds_decoder_status(&rds);
		}

		rds_decoder_exit(&rds);
	}

	printf("\nTotal: %d groups processed across %d file(s)\n", total_groups, file_count);

	return 0;
}
