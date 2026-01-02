/*
 * RDS (Radio Data System) encoder and decoder
 *
 * (C) 2025-2026 by osmocom-analog authors
 * All Rights Reserved
 *
 * Implements IEC 62106 / NRSC-4-B RDS encoding and decoding
 * - 57 kHz BPSK subcarrier (3 x 19 kHz pilot)
 * - 1187.5 bps data rate
 * - Differential encoding with CRC-10 per block
 * - Basic groups: 0A (PS), 2A (RT), 4A (CT)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "rds.h"
#include "rds_tables.h"
#include "../liblogging/logging.h"
#include "../libfm/fm.h"


/* Forward declaration for encoder loopback verification */

static void rds_decode_group(rds_decoder_t *rds);


/* ============================================================
 * RRC Biphase Waveform Generation (IEC 62106 S2.3)
 * ============================================================
 * Generates the shaped biphase waveform at runtime using:
 *   - Root Raised Cosine (RRC) filter with alpha = 1.0
 *   - Biphase impulse pair (+1, -1 at half-symbol spacing)
 *   - Convolution and center extraction
 *
 * Based on Pydemod/CommPy RRC formula:
 *   https://github.com/ChristopheJacquet/Pydemod
 * ============================================================ */

/*
 * Root Raised Cosine filter impulse response generator
 * Based on CommPy formula (GPL v3)
 *
 * N: Filter length in samples
 * alpha: Roll-off factor (1.0 for RDS per IEC 62106)
 * Ts: Symbol period in seconds
 * Fs: Sample rate in Hz
 * h: Output buffer for filter coefficients (must be N elements)
 */
static void rrc_filter_generate(double *h, int N, double alpha, double Ts, double Fs)
{
	double T_delta = 1.0 / Fs;
	int x;

	for (x = 0; x < N; x++) {
		double t = ((double)x - (double)N / 2.0) * T_delta;

		if (fabs(t) < 1e-12) {
			/* t == 0 */
			h[x] = 1.0 - alpha + (4.0 * alpha / M_PI);
		}
		else if (alpha > 0 && fabs(fabs(t) - Ts / (4.0 * alpha)) < 1e-12) {
			/*
			 * Singularity at t = +/-Ts/(4*alpha)
			 *
			 * The RRC formula has a removable singularity here.
			 * We use L'Hopital's limit value directly.
			 *
			 * Note: Pydemod uses Fs+1 to avoid hitting this exactly,
			 * but we handle it properly with this explicit check.
			 */
			h[x] = (alpha / sqrt(2.0)) * (
				(1.0 + 2.0 / M_PI) * sin(M_PI / (4.0 * alpha)) +
				(1.0 - 2.0 / M_PI) * cos(M_PI / (4.0 * alpha))
			);
		}
		else {
			/* General case */
			double num = sin(M_PI * t * (1.0 - alpha) / Ts) +
			             4.0 * alpha * (t / Ts) * cos(M_PI * t * (1.0 + alpha) / Ts);
			double denom_inner = 1.0 - pow(4.0 * alpha * t / Ts, 2);
			double den = (M_PI * t / Ts) * denom_inner;

			if (fabs(denom_inner) < 1e-12) {
				/* Near singularity */
				h[x] = (alpha / sqrt(2.0)) * (
					(1.0 + 2.0 / M_PI) * sin(M_PI / (4.0 * alpha)) +
					(1.0 - 2.0 / M_PI) * cos(M_PI / (4.0 * alpha))
				);
			} else {
				h[x] = num / den;
			}
		}
	}
}

/*
 * Generate shaped biphase waveform for RDS encoding
 *
 * Process (matching Pydemod generate_waveforms.py):
 * 1. Generate 768-tap RRC filter (alpha=1.0, Ts=half-symbol period)
 * 2. Create biphase impulse: +1 at sample 96, -1 at sample 192
 * 3. Convolve filter with impulse
 * 4. Extract center 576 samples
 * 5. Scale by 1/2.5 for amplitude headroom
 *
 * Returns: 0 on success, -1 on memory allocation failure
 */
static int rds_generate_biphase_waveform(float **waveform_out)
{
	const int filter_taps = 768;       /* 96 * 8 */
	const int l = 96;                  /* Half-symbol in samples at 228kHz */
	const int impulse_len = 3 * l;     /* 288 samples */
	const int output_len = RDS_FILTER_LENGTH;  /* 576 */
	const double Ts = 1.0 / (2.0 * RDS_BITRATE);  /* Half-symbol period */
	/*
	 * Note: Pydemod uses (sample_rate + 1) to avoid hitting the
	 * singularity at t = +/-Ts/(4*alpha). We don't need this workaround
	 * because rrc_filter_generate() handles singularities explicitly.
	 */
	const double Fs = RDS_FILTER_SAMPLERATE;
	const double scale = 1.0 / 2.5;
	double *rrc, *impulse, *conv;
	float *waveform;
	int conv_len, start, i, j;

	/* Allocate arrays */
	rrc = malloc(filter_taps * sizeof(double));
	impulse = calloc(impulse_len, sizeof(double));
	conv_len = filter_taps + impulse_len - 1;  /* 1055 */
	conv = calloc(conv_len, sizeof(double));
	waveform = malloc(output_len * sizeof(float));

	if (!rrc || !impulse || !conv || !waveform) {
		free(rrc);
		free(impulse);
		free(conv);
		free(waveform);
		return -1;
	}

	/* Generate RRC filter */
	rrc_filter_generate(rrc, filter_taps, 1.0, Ts, Fs);

	/* Create biphase impulse: +1 at l, -1 at 2*l */
	impulse[l] = 1.0;
	impulse[2 * l] = -1.0;

	/* Convolve: impulse * rrc */
	for (i = 0; i < impulse_len; i++) {
		if (impulse[i] != 0.0) {
			for (j = 0; j < filter_taps; j++) {
				conv[i + j] += impulse[i] * rrc[j];
			}
		}
	}

	/* Extract center 576 samples (matching Python: shapedSamples[528-288:528+288]) */
	start = 528 - 288;  /* 240 */
	for (i = 0; i < output_len && (start + i) < conv_len; i++) {
		waveform[i] = (float)(conv[start + i] * scale);
	}

	/* Cleanup intermediate buffers */
	free(rrc);
	free(impulse);
	free(conv);

	*waveform_out = waveform;
	return 0;
}



/* rds_calc_syndrome and rds_calc_crc are now in rdsframe.c as:
 * - rds_syndrome_calc()
 * - rds_crc_calc()
 */

/* Build a complete 26-bit block with CRC and Offset Word
 * Wrapper around rds_block_encode() for compatibility */
static uint32_t rds_build_block(uint16_t data, uint16_t offset_word)
{
	/* Use shared implementation from rdsframe.c (no debug output) */
	return rds_block_encode(data, offset_word, 0);
}


/* ============================================================
 * AF Method A Helper Functions (IEC 62106 S3.2.1.6.3)
 * ============================================================ */

/**
 * Parse a human-readable AF frequency string into an rds_af_method_a_t struct.
 *
 * Format: "91.0, 102.5, 104.8, LF225, MF1008"
 *   - VHF frequencies in MHz (87.6-107.9)
 *   - LFxxx for LF band in kHz (153-279)
 *   - MFxxx for MF band in kHz (531-1602 RDS, 540-1700 RBDS)
 *
 * LF/MF frequencies count as 2 slots each (transmitted as 250+code pair).
 * Maximum 25 total slots allowed.
 *
 * @param input  Comma-separated frequency string
 * @param out    Output structure (cleared on entry)
 * @return       0 on success, -1 on error (slot count exceeded, invalid freq)
 */
int rds_af_method_a_parse(const char *input, rds_af_method_a_t *out)
{
	if (!input || !out)
		return -1;
	
	memset(out, 0, sizeof(*out));
	
	char buf[256];
	strncpy(buf, input, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	
	char *saveptr = NULL;
	char *token = strtok_r(buf, ", \t", &saveptr);
	
	while (token) {
		/* Skip empty tokens */
		while (*token == ' ' || *token == '\t')
			token++;
		
		if (*token == '\0') {
			token = strtok_r(NULL, ", \t", &saveptr);
			continue;
		}
		
		/* Check for LF/MF prefix */
		if (strncasecmp(token, "LF", 2) == 0) {
			/* LF frequency (153-279 kHz) */
			int freq_khz = atoi(token + 2);
			if (freq_khz < 153 || freq_khz > 279) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Invalid LF frequency %d kHz (valid: 153-279)\n", freq_khz);
				return -1;
			}
			
			/* LF/MF uses 2 codes, but counts as 1 frequency */
			if (out->slot_count + 1 > 25) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Frequency limit exceeded (max 25)\n");
				return -1;
			}
			
			if (out->lf_mf_count >= 12) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Max 12 LF/MF frequencies\n");
				return -1;
			}
			
			out->lf_mf_freq[out->lf_mf_count] = freq_khz;
			out->lf_mf_type[out->lf_mf_count] = RDS_AF_FREQ_LF;
			out->lf_mf_count++;
			out->slot_count++;  /* 1 frequency (uses 2 codes) */
			
		} else if (strncasecmp(token, "MF", 2) == 0) {
			/* MF frequency (531-1602 kHz RDS, 540-1700 kHz RBDS) */
			int freq_khz = atoi(token + 2);
			if (freq_khz < 531 || freq_khz > 1700) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Invalid MF frequency %d kHz (valid: 531-1700)\n", freq_khz);
				return -1;
			}
			
			/* LF/MF uses 2 codes, but counts as 1 frequency */
			if (out->slot_count + 1 > 25) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Frequency limit exceeded (max 25)\n");
				return -1;
			}
			
			if (out->lf_mf_count >= 12) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Max 12 LF/MF frequencies\n");
				return -1;
			}
			
			out->lf_mf_freq[out->lf_mf_count] = freq_khz;
			out->lf_mf_type[out->lf_mf_count] = RDS_AF_FREQ_MF;
			out->lf_mf_count++;
			out->slot_count++;  /* 1 frequency (uses 2 codes) */
			
		} else {
			/* VHF frequency (87.6-107.9 MHz) */
			double freq_mhz = atof(token);
			if (freq_mhz < 87.5 || freq_mhz > 108.0) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Invalid VHF frequency %.1f MHz (valid: 87.6-107.9)\n", freq_mhz);
				return -1;
			}
			
			/* VHF uses 1 slot */
			if (out->slot_count + 1 > 25) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Slot limit exceeded (max 25)\n");
				return -1;
			}
			
			if (out->vhf_count >= 25) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method A: Max 25 VHF frequencies\n");
				return -1;
			}
			
			out->vhf_freq[out->vhf_count] = (uint16_t)(freq_mhz * 10 + 0.5);  /* Store as 0.1 MHz */
			out->vhf_count++;
			out->slot_count++;
		}
		
		token = strtok_r(NULL, ", \t", &saveptr);
	}
	
	LOGP(DRADIO, LOGL_DEBUG, "AF Method A: Parsed %d VHF + %d LF/MF = %d slots\n",
	     out->vhf_count, out->lf_mf_count, out->slot_count);
	
	return 0;
}

/**
 * Build AF code sequence for Method A transmission.
 * 
 * Generates the raw 8-bit AF codes in transmission order:
 *   [count_code, AF1, AF2, AF3, ..., AFn, (filler)]
 * 
 * For LF/MF: inserts [250, LF/MF_code] pair.
 * Adds filler (205) if total codes is odd (pairs must be complete).
 * 
 * Example (6 VHF AFs):
 *   Output: [230, AF1, AF2, AF3, AF4, AF5, AF6, 205]
 *   Transmitted as: [230,AF1] [AF2,AF3] [AF4,AF5] [AF6,205]
 * 
 * @param af         Input AF structure
 * @param codes      Output buffer for AF codes
 * @param max_codes  Maximum codes buffer can hold
 * @return           Number of codes generated, or -1 on error
 */
int rds_af_method_a_build_codes(const rds_af_method_a_t *af, uint8_t *codes, int max_codes)
{
	if (!af || !codes || max_codes < 2)
		return -1;
	
	int idx = 0;
	
	/* First code: count (224-249) */
	if (af->slot_count == 0)
		return 0;  /* No AFs to transmit */
	
	codes[idx++] = RDS_AF_NO_AF + af->slot_count;  /* 224 + slot_count */
	
	/* Add VHF frequencies */
	for (int i = 0; i < af->vhf_count && idx < max_codes; i++) {
		/* Code = freq_0.1MHz - 875 (87.5 MHz = code 0, but 0 is reserved, so 87.6 = code 1) */
		uint16_t freq_01mhz = af->vhf_freq[i];
		if (freq_01mhz >= 876 && freq_01mhz <= 1079) {
			codes[idx++] = (uint8_t)(freq_01mhz - 875);
		}
	}
	
	/* Add LF/MF frequencies (each uses 2 codes: 250 + freq_code) */
	for (int i = 0; i < af->lf_mf_count && idx + 1 < max_codes; i++) {
		codes[idx++] = RDS_AF_LF_MF_FOLLOWS;  /* 250 */
		
		if (af->lf_mf_type[i] == RDS_AF_FREQ_LF) {
			/* LF: code 1-15 for 153-279 kHz (9 kHz spacing) */
			codes[idx++] = (uint8_t)((af->lf_mf_freq[i] - 144) / 9);
		} else {
			/* MF: code 16-135 for 531-1602 kHz (9 kHz RDS) */
			/* TODO: RBDS uses 10 kHz spacing from 540 kHz */
			codes[idx++] = (uint8_t)(16 + (af->lf_mf_freq[i] - 531) / 9);
		}
	}
	
	/* Add filler if odd count (pairs must be complete for Block C) */
	if ((idx % 2) == 1 && idx < max_codes) {
		codes[idx++] = RDS_AF_FILLER;  /* 205 */
	}
	
	return idx;
}

/* ============================================================
 * AF METHOD B Helper Functions (IEC 62106 S3.2.1.6.4)
 * ============================================================ */

/**
 * Parse AF Method B string into structure.
 * 
 * Format: "T89.3, 99.5, 88.8, R102.6, R89.0"
 *   T prefix = tuning frequency (required, must be first)
 *   R prefix = regional variant (transmitted with F1 > F2)
 *   No prefix = same programme (transmitted with F1 < F2)
 * 
 * @return 0 on success, -1 on error
 */
int rds_af_method_b_parse(const char *input, rds_af_method_b_list_t *out)
{
	if (!input || !out)
		return -1;
	
	memset(out, 0, sizeof(*out));
	
	char buf[256];
	strncpy(buf, input, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	
	char *saveptr = NULL;
	char *token = strtok_r(buf, ", \t", &saveptr);
	int first = 1;
	
	while (token) {
		/* Skip whitespace */
		while (*token == ' ' || *token == '\t')
			token++;
		
		if (*token == '\0') {
			token = strtok_r(NULL, ", \t", &saveptr);
			continue;
		}
		
		int is_tuning = 0;
		int is_regional = 0;
		
		/* Check for T (tuning) or R (regional) prefix */
		if (*token == 'T' || *token == 't') {
			is_tuning = 1;
			token++;
		} else if (*token == 'R' || *token == 'r') {
			is_regional = 1;
			token++;
		}
		
		/* Parse frequency */
		double freq_mhz = atof(token);
		if (freq_mhz < 87.5 || freq_mhz > 108.0) {
			LOGP(DRADIO, LOGL_ERROR, "AF Method B: Invalid frequency %.1f MHz (valid: 87.6-107.9)\n", freq_mhz);
			return -1;
		}
		
		uint16_t freq_01mhz = (uint16_t)(freq_mhz * 10 + 0.5);
		
		if (first) {
			/* First element must be tuning frequency */
			if (!is_tuning) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method B: First element must be tuning frequency (T prefix)\n");
				return -1;
			}
			out->tuning_freq = freq_01mhz;
			first = 0;
		} else {
			/* Subsequent elements are AFs */
			if (out->af_count >= RDS_AF_METHOD_B_MAX_AFS) {
				LOGP(DRADIO, LOGL_ERROR, "AF Method B: Max 12 AFs per list\n");
				return -1;
			}
			out->af_freq[out->af_count] = freq_01mhz;
			out->af_is_regional[out->af_count] = is_regional;
			out->af_count++;
		}
		
		token = strtok_r(NULL, ", \t", &saveptr);
	}
	
	if (out->tuning_freq == 0) {
		LOGP(DRADIO, LOGL_ERROR, "AF Method B: No tuning frequency specified\n");
		return -1;
	}
	
	LOGP(DRADIO, LOGL_DEBUG, "AF Method B: Parsed tuning=%.1f, %d AFs\n",
	     out->tuning_freq / 10.0, out->af_count);
	
	return 0;
}

/**
 * Build AF code sequence for Method B transmission.
 * 
 * Generates: [count_code, tuning_freq], then pairs [F1, F2]
 * 
 * Pair ordering follows specification:
 *   Same programme: F1 < F2 (ascending)
 *   Regional:       F1 > F2 (descending)
 * 
 * Tuning frequency can appear in either F1 or F2 position.
 * 
 * @return Number of codes generated, or -1 on error
 */
int rds_af_method_b_build_codes(const rds_af_method_b_list_t *list, uint8_t *codes, int max_codes)
{
	if (!list || !codes || max_codes < 4)
		return -1;
	
	if (list->af_count == 0 || list->tuning_freq == 0)
		return 0;
	
	int idx = 0;
	uint8_t tuning_code = (uint8_t)(list->tuning_freq - 875);
	
	/* Header: [count_code, tuning_freq]
	 * Count = 1 (tuning) + 2 × af_count (each pair has 2 frequencies)
	 * Method B count is ALWAYS ODD (1 + even number) */
	uint8_t count = 1 + 2 * list->af_count;
	if (count > 25) count = 25;  /* Max per spec */
	codes[idx++] = RDS_AF_NO_AF + count;  /* 224 + count */
	codes[idx++] = tuning_code;
	
	/* AF pairs: [F1, F2] with ordering based on regional flag
	 * Same programme: F1 < F2 (ascending order)
	 * Regional:       F1 > F2 (descending order) */
	for (int i = 0; i < list->af_count && idx + 1 < max_codes; i++) {
		uint8_t af_code = (uint8_t)(list->af_freq[i] - 875);
		
		if (list->af_is_regional[i]) {
			/* Regional: F1 > F2 (descending) */
			if (tuning_code > af_code) {
				codes[idx++] = tuning_code;
				codes[idx++] = af_code;
			} else {
				codes[idx++] = af_code;
				codes[idx++] = tuning_code;
			}
		} else {
			/* Same programme: F1 < F2 (ascending) */
			if (tuning_code < af_code) {
				codes[idx++] = tuning_code;
				codes[idx++] = af_code;
			} else {
				codes[idx++] = af_code;
				codes[idx++] = tuning_code;
			}
		}
	}
	
	return idx;
}

/* ============================================================
 * AF METHOD B Decoder Helper Functions (IEC 62106 S3.2.1.6.4)
 * ============================================================ */


/**
 * Compare two Method B history entries for equality.
 * @return 1 if equal, 0 if different
 */
static int rds_af_method_b_lists_equal(rds_af_method_b_history_t *a, 
                                        rds_af_method_b_dec_t *b)
{
	if (a->pi != b->pi || a->tuning_freq != b->tuning_freq)
		return 0;
	if (a->received_count != b->received_count)
		return 0;
	
	for (int i = 0; i < a->received_count; i++) {
		if (a->af_freq[i] != b->af_freq[i] ||
		    a->af_is_regional[i] != b->af_is_regional[i])
			return 0;
	}
	return 1;
}

/**
 * Add current Method B list to history.
 * If identical entry exists, move to top. Otherwise insert new.
 */
static void rds_af_method_b_add_to_history(rds_decoder_t *rds)
{
	rds_af_method_b_dec_t *dec = &rds->af_method_b_dec;
	
	/* Check if identical entry exists */
	for (int i = 0; i < dec->history_count; i++) {
		if (rds_af_method_b_lists_equal(&dec->history[i], dec)) {
			/* Move to top */
			rds_af_method_b_history_t tmp = dec->history[i];
			memmove(&dec->history[1], &dec->history[0], 
			        i * sizeof(rds_af_method_b_history_t));
			dec->history[0] = tmp;
			dec->history[0].timestamp = time(NULL);
			return;
		}
	}
	
	/* Insert new entry at top, shift others down */
	if (dec->history_count < RDS_AF_METHOD_B_HISTORY_MAX)
		dec->history_count++;
	memmove(&dec->history[1], &dec->history[0],
	        (dec->history_count - 1) * sizeof(rds_af_method_b_history_t));
	
	/* Copy current state to history[0] */
	dec->history[0].pi = dec->pi;
	dec->history[0].tuning_freq = dec->tuning_freq;
	dec->history[0].expected_count = dec->expected_count;
	dec->history[0].received_count = dec->received_count;
	memcpy(dec->history[0].af_freq, dec->af_freq, sizeof(dec->af_freq));
	memcpy(dec->history[0].af_is_regional, dec->af_is_regional, 
	       sizeof(dec->af_is_regional));
	memcpy(dec->history[0].af_status, dec->af_status, sizeof(dec->af_status));
	dec->history[0].timestamp = time(NULL);
	dec->history[0].complete = 1;
}

/**
 * Analyze completed AF collector and decode as Method A or B.
 * Called when expected AF count has been received.
 */
static void rds_af_decode_complete(rds_decoder_t *rds)
{
	rds_af_collector_t *col = &rds->af_collector;
	
	/* Check if count < 3 OR count is even: immediate Method A
	 * Method B count is always odd (1 + 2*N pairs) and >= 3 */
	if (col->expected_count < 3 || (col->expected_count % 2) == 0) {
		/* Decode as Method A */
		rds_af_method_a_dec_t *afd = &rds->af_method_a_dec;
		afd->complete = 1;
		
		/* Check if all received with good/corrected status - save as last_good */
		int all_good = 1;
		for (int i = 0; i < afd->received_count; i++) {
			if (afd->status[i] > RDS_STATUS_CORRECTED) {
				all_good = 0;
				break;
			}
		}
		if (all_good && afd->received_count > 0) {
			afd->last_good_pi = rds->pi;
			afd->last_good_tuning = 0;  /* Tuned frequency not tracked for Method A */
			afd->last_good_count = afd->received_count;
			memcpy(afd->last_good_freq, afd->freq, sizeof(afd->freq));
			memcpy(afd->last_good_type, afd->type, sizeof(afd->type));
			afd->last_good_time = time(NULL);
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF Method A complete, %d freqs saved as last good\n",
				     afd->received_count);
		} else if (rds->verbose) {
			LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF list complete (Method A), %d frequencies\n",
			     afd->received_count);
		}
		col->header_received = 0;
		return;
	}
	
	/* Check for all good/corrected status */
	for (int i = 0; i < col->received_pairs * 2; i++) {
		if (col->status[i] > RDS_STATUS_CORRECTED) {
			/* Bad status - cannot reliably determine method */
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF list has errors, using Method A\n");
			rds->af_method_a_dec.complete = 1;
			col->header_received = 0;
			return;
		}
	}
	
	/* Check Method B pattern: tuning_freq in all pairs after header */
	uint8_t tuning_code = col->codes[1];
	int is_method_b = 1;
	
	for (int i = 2; i < col->received_pairs * 2; i += 2) {
		uint8_t f1 = col->codes[i];
		uint8_t f2 = col->codes[i + 1];
		
		/* Skip filler codes */
		if (f1 == RDS_AF_FILLER || f2 == RDS_AF_FILLER)
			continue;
		
		/* Tuning freq must be in position 1 OR position 2 */
		if (f1 != tuning_code && f2 != tuning_code) {
			is_method_b = 0;
			break;
		}
	}
	
	if (is_method_b) {
		/* Decode as Method B */
		rds_af_method_b_dec_t *dec = &rds->af_method_b_dec;
		
		dec->pi = col->pi;
		dec->tuning_freq = RDS_AF_FM_BASE + tuning_code;
		dec->expected_count = col->expected_count;
		dec->received_count = 0;
		
		/* Extract AF pairs with regional detection */
		for (int i = 2; i < col->received_pairs * 2 && dec->received_count < RDS_AF_METHOD_B_MAX_AFS; i += 2) {
			uint8_t f1 = col->codes[i];
			uint8_t f2 = col->codes[i + 1];
			
			if (f1 == RDS_AF_FILLER || f2 == RDS_AF_FILLER)
				continue;
			
			int idx = dec->received_count;
			
			/* Identify AF (the one that's NOT tuning_freq) */
			uint8_t af_code = (f1 == tuning_code) ? f2 : f1;
			dec->af_freq[idx] = RDS_AF_FM_BASE + af_code;
			
			/* Regional detection: F1 > F2 = regional, F1 < F2 = same */
			dec->af_is_regional[idx] = (f1 > f2);
			
			dec->af_status[idx] = (col->status[i] == RDS_STATUS_VALID && 
			                       col->status[i+1] == RDS_STATUS_VALID) ?
			                      RDS_STATUS_VALID : RDS_STATUS_CORRECTED;
			dec->received_count++;
		}
		
		/* Add to history */
		rds_af_method_b_add_to_history(rds);
		
		if (rds->verbose)
			LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF Method B decoded, tuning=%.1f, %d AFs\n",
			     dec->tuning_freq / 10.0, dec->received_count);
	} else {
		/* Method A - already decoded during collection */
		rds_af_method_a_dec_t *afd = &rds->af_method_a_dec;
		afd->complete = 1;
		
		/* Check if all received with good/corrected status - save as last_good */
		int all_good = 1;
		for (int i = 0; i < afd->received_count; i++) {
			if (afd->status[i] > RDS_STATUS_CORRECTED) {
				all_good = 0;
				break;
			}
		}
		if (all_good && afd->received_count > 0) {
			afd->last_good_pi = rds->pi;
			afd->last_good_tuning = 0;
			afd->last_good_count = afd->received_count;
			memcpy(afd->last_good_freq, afd->freq, sizeof(afd->freq));
			memcpy(afd->last_good_type, afd->type, sizeof(afd->type));
			afd->last_good_time = time(NULL);
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF Method A complete, %d freqs saved as last good\n",
				     afd->received_count);
		} else if (rds->verbose) {
			LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF list complete (Method A), %d frequencies\n",
			     afd->received_count);
		}
	}
	
	/* Reset collector */
	col->header_received = 0;
}


/* ============================================================
 * Build Group 0A: Basic Tuning and Switching Information
 * IEC 62106 S6.1.5.1 - MANDATORY group for RDS transmission
 *
 * Group 0A provides essential tuning information including the
 * Programme Service name (PS) and Alternative Frequencies (AF).
 *
 * Block Structure:
 *   Block A [16 bits]: PI code
 *   Block B [16 bits]:
 *     - Bits 15-12: Group type (0000)
 *     - Bit 11:     Version (0 = A)
 *     - Bit 10:     TP (Traffic Programme)
 *     - Bits 9-5:   PTY (Programme Type)
 *     - Bit 4:      TA (Traffic Announcement)
 *     - Bit 3:      M/S (Music=1 / Speech=0)
 *     - Bit 2:      DI (Decoder Info, segment-dependent)
 *     - Bits 1-0:   PS segment address (0-3)
 *   Block C [16 bits]: AF1 (8 bits) + AF2 (8 bits)
 *   Block D [16 bits]: PS chars (2 per group)
 *
 * Complete PS name (8 chars) requires 4 groups (segments 0-3).
 * ============================================================ */

static void rds_build_group_0a(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b3;
	int seg = rds->ps_segment;
	
	/* DI flag depends on segment address */
	uint8_t di_flags[4] = {
		rds->di_dynamic_pty,
		rds->di_compressed,
		rds->di_artificial_head,
		rds->di_stereo
	};
	uint8_t di = di_flags[seg & 3] ? 1 : 0;
	
	/* Block A: PI code */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group type 0A + PTY + TP + TA + M/S + DI + segment */
	b2 = (RDS_GROUP_0A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     (rds->ta << RDS_0A_TA_BIT) |
	     (rds->ms << RDS_0A_MS_BIT) |
	     (di << RDS_0A_DI_BIT) |
	     seg;
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: Alternative Frequencies (Method A or Method B)
	 * Method A: Transmit pairs from pre-computed af_codes[] buffer.
	 * Method B: Generate pairs dynamically from af_method_b lists. */
	if (rds->use_method_b && rds->af_method_b.list_count > 0) {
		/* Method B: paired frequencies from lists */
		rds_af_method_b_list_t *list = &rds->af_method_b.lists[rds->af_method_b_list_idx];
		int pair_idx = rds->af_method_b_pair_idx;
		
		uint8_t tuning_code = (uint8_t)(list->tuning_freq - 875);
		
		if (pair_idx == 0) {
			/* Header: [count_code, tuning_freq]
			 * Count = 1 (tuning) + 2 × af_count (each pair has 2 frequencies)
			 * Example: 5 AFs = 1 + 10 = 11 (always odd for Method B) */
			uint8_t count = 1 + 2 * list->af_count;
			b3 = ((RDS_AF_NO_AF + count) << 8) | tuning_code;
			
			if (rds->debug)
				LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: Method B header: count=%d tuning=%.1f\n",
				     count, list->tuning_freq / 10.0);
		} else {
			/* AF pair: order by same/regional flag */
			int af_idx = pair_idx - 1;
			if (af_idx < list->af_count) {
				uint8_t af_code = (uint8_t)(list->af_freq[af_idx] - 875);
				
				if (list->af_is_regional[af_idx]) {
					/* Regional: F1 > F2 (descending) */
					if (tuning_code > af_code) {
						b3 = (tuning_code << 8) | af_code;
					} else {
						b3 = (af_code << 8) | tuning_code;
					}
				} else {
					/* Same programme: F1 < F2 (ascending) */
					if (tuning_code < af_code) {
						b3 = (tuning_code << 8) | af_code;
					} else {
						b3 = (af_code << 8) | tuning_code;
					}
				}
				
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: Method B pair[%d]: [%02X, %02X] %s\n",
					     af_idx, (b3 >> 8) & 0xFF, b3 & 0xFF,
					     list->af_is_regional[af_idx] ? "regional" : "same");
			} else {
				b3 = RDS_AF_NO_AF_PAIR;
			}
		}
		
		/* Advance: header, then pairs, then next list */
		rds->af_method_b_pair_idx++;
		if (rds->af_method_b_pair_idx > list->af_count) {
			rds->af_method_b_pair_idx = 0;
			rds->af_method_b_list_idx = (rds->af_method_b_list_idx + 1) % rds->af_method_b.list_count;
		}
	} else if (rds->af_code_count >= 2) {
		/* Method A: from pre-computed af_codes[] buffer */
		/* Each segment = 2 codes (one pair) */
		int pair_idx = rds->af_method_a_segment * 2;
		
		if (pair_idx < rds->af_code_count) {
			uint8_t af1 = rds->af_codes[pair_idx];
			uint8_t af2 = (pair_idx + 1 < rds->af_code_count) 
			              ? rds->af_codes[pair_idx + 1] 
			              : RDS_AF_FILLER;
			b3 = (af1 << 8) | af2;
			
			if (rds->debug)
				LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: AF pair %d: [%02X, %02X]\n",
				     rds->af_method_a_segment, af1, af2);
		} else {
			/* All pairs sent - use NO_AF_PAIR for remaining groups */
			b3 = RDS_AF_NO_AF_PAIR;
		}
		
		/* Advance to next pair, wrap to start after all pairs sent */
		rds->af_method_a_segment++;
		int num_pairs = (rds->af_code_count + 1) / 2;
		if (rds->af_method_a_segment >= num_pairs)
			rds->af_method_a_segment = 0;
	} else {
		/* No AF data */
		b3 = RDS_AF_NO_AF_PAIR;
	}

	blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
	
	/* Block D: PS name (2 characters per group) */
	uint16_t ps_chars = ((uint8_t)rds->ps[seg*2] << 8) | (uint8_t)rds->ps[seg*2+1];
	blocks[3] = rds_build_block(ps_chars, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	rds->ps_segment = (seg + 1) & RDS_PS_SEG_MASK;
}

/* ============================================================
 * Build Group 0B: Basic Tuning (PI repeat, no AF)
 * IEC 62106 S6.1.5.1 - Version B for faster station identification
 *
 * Group 0B is used when:
 *   - No Alternative Frequencies exist (single-transmitter station)
 *   - Better mobile reception needed (PI repeat in Block C)
 *
 * Trade-off vs 0A:
 *   - 0A: Has AF data for seamless frequency switching
 *   - 0B: PI repeat improves station identification in weak signals
 *
 * Block Structure:
 *   Block A [16 bits]: PI code
 *   Block B [16 bits]: Same as 0A (TP, PTY, TA, M/S, DI, segment)
 *   Block C [16 bits]: PI code repeat (offset C')
 *   Block D [16 bits]: PS chars (2 per group)
 * ============================================================ */
static void rds_build_group_0b(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2;
	int seg = rds->ps_segment;
	
	/* DI flag depends on segment address (same as 0A) */
	uint8_t di_flags[4] = {
		rds->di_dynamic_pty,
		rds->di_compressed,
		rds->di_artificial_head,
		rds->di_stereo
	};
	uint8_t di = di_flags[seg] ? 1 : 0;
	
	/* Block A: PI code */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group type 0B + PTY + TP + TA + M/S + DI + segment */
	b2 = (RDS_GROUP_0B << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     (rds->ta << RDS_0A_TA_BIT) |
	     (rds->ms << RDS_0A_MS_BIT) |
	     (di << RDS_0A_DI_BIT) |
	     seg;
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: PI code repeat (Type B groups use C' offset) */
	blocks[2] = rds_build_block(rds->pi, RDS_OFFSET_Cp);
	
	/* Block D: PS name (2 characters per group) */
	uint16_t ps_chars = ((uint8_t)rds->ps[seg*2] << 8) | (uint8_t)rds->ps[seg*2+1];
	blocks[3] = rds_build_block(ps_chars, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	rds->ps_segment = (seg + 1) & RDS_PS_SEG_MASK;
}


/* Build Group 2A: RadioText */
static void rds_build_group_2a(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2;
	int seg = rds->rt_segment;
	int pos = seg * 4;
	
	/* IEC 62106: Switching from 2B to 2A without toggling A/B flag causes
	 * problematic receiver behavior. Auto-toggle when version changes. */
	if (rds->rt_last_version == 1) {
		rds->rt_ab = !rds->rt_ab;
		rds->rt_segment = 0;  /* Restart from segment 0 */
		seg = 0;
		pos = 0;
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS TX: Switched 2B→2A, toggled A/B to %c\n",
			     rds->rt_ab ? 'B' : 'A');
	}
	rds->rt_last_version = 0;
	
	/* Block A: PI code */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group type + PTY + TP + A/B flag + segment */
	b2 = (RDS_GROUP_2A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     (rds->rt_ab << RDS_2A_AB_BIT) |
	     seg;
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: RT chars 0-1 */
	uint16_t rt_chars1 = ((uint8_t)rds->rt[pos] << 8) | (uint8_t)rds->rt[pos+1];
	blocks[2] = rds_build_block(rt_chars1, RDS_OFFSET_C);
	
	/* Block D: RT chars 2-3 */
	uint16_t rt_chars2 = ((uint8_t)rds->rt[pos+2] << 8) | (uint8_t)rds->rt[pos+3];
	blocks[3] = rds_build_block(rt_chars2, RDS_OFFSET_D);
	
	/* Debug: Log transmitted segment using proper RDS char display */
	if (rds->debug)
		LOGP(DRADIO, LOGL_DEBUG, "RDS TX 2A: seg=%d pos=%d-%d AB=%c chars='%s%s%s%s'\n",
		     seg, pos, pos+3, rds->rt_ab ? 'B' : 'A',
		     rds_char_to_display((uint8_t)rds->rt[pos]),
		     rds_char_to_display((uint8_t)rds->rt[pos+1]),
		     rds_char_to_display((uint8_t)rds->rt[pos+2]),
		     rds_char_to_display((uint8_t)rds->rt[pos+3]));
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	/* Advance segment, wrapping at end of message (CR terminator or max 16 segments)
	 * EN 50067: "A new text must start with segment address 0000 and there must
	 * be no gaps up to the highest used segment address of the current message." */
	int next_seg = seg + 1;
	
	/* Check if current segment contained CR or we've reached max segments */
	if (next_seg >= 16 ||
	    rds->rt[pos] == '\r' || rds->rt[pos+1] == '\r' ||
	    rds->rt[pos+2] == '\r' || rds->rt[pos+3] == '\r') {
		rds->rt_segment = 0;  /* Wrap to beginning */
	} else {
		rds->rt_segment = next_seg;
	}
}

/* Build Group 2B: RadioText (32 chars, PI repeat in Block C)
 * IEC 62106 S6.1.5.3 - Version B for improved mobile reception
 * Block C contains PI repeat; Block D contains 2 RT chars per segment
 * 
 * EN 50067: Group 2B uses 2 chars per segment, max 32 chars total.
 * Segment 0-15 = chars 0-31 (positions 0-31 in rt[] buffer).
 * Characters beyond position 31 are NOT transmitted in 2B. */
static void rds_build_group_2b(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2;
	int seg = rds->rt_segment;
	int pos = seg * 2;  /* 2 chars per segment for 2B */
	
	/* IEC 62106: Switching from 2A to 2B without toggling A/B flag causes
	 * problematic receiver behavior. Auto-toggle when version changes. */
	if (rds->rt_last_version == 0) {
		rds->rt_ab = !rds->rt_ab;
		rds->rt_segment = 0;  /* Restart from segment 0 */
		seg = 0;
		pos = 0;
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS TX: Switched 2A→2B, toggled A/B to %c\n",
			     rds->rt_ab ? 'B' : 'A');
	}
	rds->rt_last_version = 1;
	
	/* Safety: 2B only accesses positions 0-31 */
	if (pos > 30) {
		pos = 0;
		seg = 0;
		rds->rt_segment = 0;
	}
	
	/* Block A: PI code */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group type 2B + PTY + TP + A/B flag + segment */
	b2 = (RDS_GROUP_2B << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     (rds->rt_ab << RDS_2A_AB_BIT) |
	     seg;
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: PI repeat (offset C' for B-version groups) */
	blocks[2] = rds_build_block(rds->pi, RDS_OFFSET_Cp);
	
	/* Block D: RT chars (2 per segment, positions 0-31 only for 2B) */
	uint16_t rt_chars = ((uint8_t)rds->rt[pos] << 8) | (uint8_t)rds->rt[pos+1];
	blocks[3] = rds_build_block(rt_chars, RDS_OFFSET_D);
	
	/* Debug: Log transmitted segment using proper RDS char display */
	if (rds->debug)
		LOGP(DRADIO, LOGL_DEBUG, "RDS TX 2B: seg=%d pos=%d-%d AB=%c chars='%s%s'\n",
		     seg, pos, pos+1, rds->rt_ab ? 'B' : 'A',
		     rds_char_to_display((uint8_t)rds->rt[pos]),
		     rds_char_to_display((uint8_t)rds->rt[pos+1]));
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	/* Advance segment, wrapping at end of message (CR terminator or max 16 segments)
	 * EN 50067: Group 2B uses 2 chars/segment, max 32 chars total.
	 * "Each message should be ended by the code 0D (Hex) - carriage return" */
	int next_seg = seg + 1;
	
	/* Check if current segment contained CR or we've reached max 16 segments (32 chars) */
	if (next_seg >= 16 ||
	    rds->rt[pos] == '\r' || rds->rt[pos+1] == '\r') {
		rds->rt_segment = 0;  /* Wrap to beginning */
	} else {
		rds->rt_segment = next_seg;
	}
}

/* Build Group 1A: Extended Country Code, Language, and Programme Item Number (PIN)
 *
 * NOTE ON PIN (Programme Item Number) - IEC 62106 S6.1.5.2:
 *   Group 1A PIN identifies a specific programme occurrence in time for THIS SERVICE.
 *   It is the authoritative source for timer-controlled actions like:
 *     - Wake-up timers
 *     - Automatic recording
 *     - Programme reminders
 *
 *   This is DISTINCT from Group 14A PIN (variant 14), which transmits PIN for a
 *   LINKED service via EON (Enhanced Other Networks). The two PINs:
 *     - May have different values (usually do)
 *     - Apply to different PI codes (Group 1A = this PI, Group 14A = linked PI)
 *     - Are complementary, not redundant
 *
 *   Most receivers only implement Group 1A PIN and ignore Group 14A PIN entirely.
 */
static void rds_build_group_1a(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b3, b4;
	
	/* Build dynamic SLC variant sequence from enabled fields
	 * IEC 62106 Table 9 - cycle only through configured (non-zero) variants:
	 *   0 = ECC (Extended Country Code) - always included
	 *   1 = TMC ID - if tmc_id != 0
	 *   3 = Language Identification Code - if language != 0
	 *   6 = Broadcaster data - if slc_broadcaster != 0
	 *   7 = EWS channel ID - if ews_channel != 0
	 */
	int slc_variants[8];  /* Max 8 variants in IEC 62106 Table 9 */
	int slc_variant_count = 0;
	
	/* Always include ECC (variant 0) - even if ecc=0x00 ("not defined"),
	 * this variant carries the LA (Linkage Actuator) flag which is always useful */
	slc_variants[slc_variant_count++] = RDS_1A_VARIANT_ECC;
	
	/* Include optional variants only if configured */
	if (rds->tmc_id != 0)
		slc_variants[slc_variant_count++] = RDS_1A_VARIANT_TMC_ID;
	if (rds->language != 0)
		slc_variants[slc_variant_count++] = RDS_1A_VARIANT_LANGUAGE;
	if (rds->slc_broadcaster != 0)
		slc_variants[slc_variant_count++] = RDS_1A_VARIANT_BCAST;
	if (rds->ews_channel != 0)
		slc_variants[slc_variant_count++] = RDS_1A_VARIANT_EWS;
	
	int variant = slc_variants[rds->slc_variant % slc_variant_count];
	
	/* Block A: PI */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 1A + TP + PTY 
	 * Bits 4-0: Radio Paging codes (set to 0, deprecated) */
	b2 = (RDS_GROUP_1A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: LA + Variant + Payload (IEC 62106 Table 9)
	 * Bits 15:    LA (Linkage Actuator)
	 * Bits 14-12: Variant code (0-7)
	 * Bits 11-0:  Variant-specific payload */
	switch (variant) {
	case RDS_1A_VARIANT_ECC:
		/* Variant 0: Extended Country Code
		 * Bits 11-8: Paging (always 0, deprecated)
		 * Bits 7-0:  ECC value */
		b3 = ((rds->linkage_actuator ? 1 : 0) << RDS_1A_LA_BIT) |
		     (RDS_1A_VARIANT_ECC << RDS_1A_VARIANT_SHIFT) |
		     (rds->ecc & 0xFF);
		break;
		
	case RDS_1A_VARIANT_TMC_ID:
		/* Variant 1: TMC identification
		 * Bits 11-0: TMC ID for traffic message channel */
		b3 = ((rds->linkage_actuator ? 1 : 0) << RDS_1A_LA_BIT) |
		     (RDS_1A_VARIANT_TMC_ID << RDS_1A_VARIANT_SHIFT) |
		     (rds->tmc_id & RDS_1A_PAYLOAD_MASK);
		break;
		
	case RDS_1A_VARIANT_LANGUAGE:
		/* Variant 3: Language Identification Code (LIC)
		 * Bits 7-0: Language code (0-127, IEC 62106 Annex J) */
		b3 = ((rds->linkage_actuator ? 1 : 0) << RDS_1A_LA_BIT) |
		     (RDS_1A_VARIANT_LANGUAGE << RDS_1A_VARIANT_SHIFT) |
		     (rds->language & 0xFF);
		break;
		
	case RDS_1A_VARIANT_BCAST:
		/* Variant 6: Broadcaster use
		 * Bits 11-0: Broadcaster-defined data */
		b3 = ((rds->linkage_actuator ? 1 : 0) << RDS_1A_LA_BIT) |
		     (RDS_1A_VARIANT_BCAST << RDS_1A_VARIANT_SHIFT) |
		     (rds->slc_broadcaster & RDS_1A_PAYLOAD_MASK);
		break;
		
	case RDS_1A_VARIANT_EWS:
		/* Variant 7: Emergency Warning System channel ID
		 * Bits 11-0: EWS channel number */
		b3 = ((rds->linkage_actuator ? 1 : 0) << RDS_1A_LA_BIT) |
		     (RDS_1A_VARIANT_EWS << RDS_1A_VARIANT_SHIFT) |
		     (rds->ews_channel & RDS_1A_PAYLOAD_MASK);
		break;
		
	default:
		/* Fallback to ECC if unknown variant somehow selected */
		b3 = ((rds->linkage_actuator ? 1 : 0) << RDS_1A_LA_BIT) |
		     (RDS_1A_VARIANT_ECC << RDS_1A_VARIANT_SHIFT) |
		     (rds->ecc & 0xFF);
		break;
	}
	blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
	
	/* Block D: PIN (Programme Item Number) - for THIS service (current PI)
	 * IEC 62106:2015 S6.1.5.2
	 * Format: Day (5 bits) | Hour (5 bits) | Minute (6 bits)
	 * Day = day of month 1-31 (0 = PIN not used), receiver uses CT for month */
	b4 = ((rds->pin_day & (RDS_PIN_DAY_MASK >> RDS_PIN_DAY_SHIFT)) << RDS_PIN_DAY_SHIFT) |
	     ((rds->pin_hour & (RDS_PIN_HOUR_MASK >> RDS_PIN_HOUR_SHIFT)) << RDS_PIN_HOUR_SHIFT) |
	     (rds->pin_minute & RDS_PIN_MINUTE_MASK);
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	/* Advance to next variant for next call */
	rds->slc_variant = (rds->slc_variant + 1) % slc_variant_count;
}


/* Build Group 1B: PIN only (PI repeat in Block C, no SLC data)
 * IEC 62106 S6.1.5.2 - Version B for faster PIN transmission
 *
 * Use when:
 *   - Only PIN needed (no ECC/Language codes required)
 *   - Better mobile reception needed (PI repeat in Block C)
 *
 * Trade-off vs 1A:
 *   - 1A: ECC + Language + PIN (full slow labelling data)
 *   - 1B: PIN only, but with PI repeat for mobile reception
 *
 * Block structure:
 *   Block A: PI code
 *   Block B: Group 1B + TP + PTY + Spare bits (0)
 *   Block C: PI repeat (offset C')
 *   Block D: PIN (Day/Hour/Minute)
 */
static void rds_build_group_1b(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b4;
	
	/* Block A: PI code */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 1B + TP + PTY
	 * Bits 4-0: Spare (always 0 for 1B, Radio Paging only applies to 1A) */
	b2 = (RDS_GROUP_1B << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: PI repeat (offset C' for B-version groups) */
	blocks[2] = rds_build_block(rds->pi, RDS_OFFSET_Cp);
	
	/* Block D: PIN (Programme Item Number) - for THIS service
	 * Same format as Group 1A Block D */
	b4 = ((rds->pin_day & (RDS_PIN_DAY_MASK >> RDS_PIN_DAY_SHIFT)) << RDS_PIN_DAY_SHIFT) |
	     ((rds->pin_hour & (RDS_PIN_HOUR_MASK >> RDS_PIN_HOUR_SHIFT)) << RDS_PIN_HOUR_SHIFT) |
	     (rds->pin_minute & RDS_PIN_MINUTE_MASK);
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
}

/* ============================================================
 * RT+ (RadioText Plus) Encoder
 * IEC 62106-6: Content-type tagging for RadioText
 * ============================================================ */

/* Build RT+ ODA group (typically Group 11A)
 * Encodes RT+ tags into ODA carrier group
 */
static void rds_build_rtplus_group(rds_encoder_t *rds, rds_rtplus_encoder_t *rtplus,
                                   uint16_t *b2, uint16_t *b3, uint16_t *b4)
{
	uint16_t b2_base = (rds->tp << RDS_B2_TP_BIT) | (rds->pty << RDS_B2_PTY_SHIFT);
	
	/* Block B: toggle, item_running, tag1 content_type[5:3] */
	*b2 = b2_base;
	if (rtplus->tag_count > 0) {
		*b2 |= (rtplus->toggle << RDS_RTPLUS_TOGGLE_BIT);
		*b2 |= (rtplus->item_running << RDS_RTPLUS_ITEM_RUNNING_BIT);
		*b2 |= ((rtplus->tags[0].content_type >> RDS_RTPLUS_TAG1_CT_HIGH_SHIFT) & RDS_RTPLUS_TAG1_CT_HIGH_MASK);
	}
	
	/* Block C: tag1 content_type[2:0], start, length-1, tag2 content_type[5:4] */
	*b3 = 0;
	if (rtplus->tag_count > 0) {
		rds_rtplus_tag_t *tag1 = &rtplus->tags[0];
		*b3 |= ((tag1->content_type & 0x07) << RDS_RTPLUS_TAG1_CT_LOW_SHIFT);  /* bits 15-13 */
		*b3 |= ((tag1->start & 0x3F) << RDS_RTPLUS_TAG1_START_SHIFT);          /* bits 12-7 */
		*b3 |= (((tag1->length - 1) & 0x3F) << RDS_RTPLUS_TAG1_LEN_SHIFT);   /* bits 6-1 (length-1) */
		
		if (rtplus->tag_count > 1) {
			/* Tag2 content_type[5:4] goes in B3[1:0] (2 bits) */
			*b3 |= ((rtplus->tags[1].content_type >> RDS_RTPLUS_TAG2_CT_HIGH_SHIFT) & RDS_RTPLUS_TAG2_CT_HIGH_MASK);
		}
	}
	
	/* Block D: tag2 content_type[4:0], start, length-1, spare */
	*b4 = 0;
	if (rtplus->tag_count > 1) {
		rds_rtplus_tag_t *tag2 = &rtplus->tags[1];
		/* Tag2 content_type: extract bits 4:0 from the 6-bit value */
		*b4 |= ((tag2->content_type & 0x1F) << RDS_RTPLUS_TAG2_CT_LOW_SHIFT);  /* bits 15-11 */
		*b4 |= ((tag2->start & 0x3F) << RDS_RTPLUS_TAG2_START_SHIFT);         /* bits 10-5 */
		*b4 |= (((tag2->length - 1) & 0x1F) << RDS_RTPLUS_TAG2_LEN_SHIFT);  /* bits 4-0 (length-1, max 31) */
	}
	
	if (rds->debug) {
		LOGP(DRADIO, LOGL_DEBUG, "RDS TX RT+: toggle=%d running=%d tags=%d\n",
		     rtplus->toggle, rtplus->item_running, rtplus->tag_count);
		if (rtplus->tag_count > 0) {
			LOGP(DRADIO, LOGL_DEBUG, "  Tag1: type=%d start=%d len=%d\n",
			     rtplus->tags[0].content_type, rtplus->tags[0].start, rtplus->tags[0].length);
		}
		if (rtplus->tag_count > 1) {
			LOGP(DRADIO, LOGL_DEBUG, "  Tag2: type=%d start=%d len=%d\n",
			     rtplus->tags[1].content_type, rtplus->tags[1].start, rtplus->tags[1].length);
		}
	}
}

/* ============================================================
 * eRT (Enhanced RadioText) Encoder
 * RDS2 / IEC 62106-2: 128-byte RadioText with UTF-8/UCS-2
 * ============================================================ */

/* Build eRT ODA group (configurable ODA group)
 * Similar to Group 2A but for 128-byte eRT text
 */
static void rds_build_ert_group(rds_encoder_t *rds, uint16_t *b2, uint16_t *b3, uint16_t *b4)
{
	rds_ert_encoder_t *ert = &rds->ert;
	int seg = ert->segment;
	int pos = seg * RDS_ERT_BYTES_PER_SEGMENT;
	
	if (pos >= RDS_ERT_LENGTH) {
		seg = 0;
		pos = 0;
		ert->segment = 0;
	}
	
	/* Block B: segment address (0-31) */
	*b2 = (rds->tp << RDS_B2_TP_BIT) | (rds->pty << RDS_B2_PTY_SHIFT);
	*b2 |= (seg & RDS_ERT_SEGMENT_MASK);
	
	/* Block C: eRT bytes [pos] and [pos+1] */
	*b3 = ((uint16_t)ert->ert[pos] << 8) | (uint16_t)ert->ert[pos + 1];
	
	/* Block D: eRT bytes [pos+2] and [pos+3] */
	*b4 = ((uint16_t)ert->ert[pos + 2] << 8) | (uint16_t)ert->ert[pos + 3];
	
	if (rds->debug) {
		char char_display[5] = {0};
		for (int i = 0; i < 4 && pos + i < RDS_ERT_LENGTH; i++) {
			const char *disp = rds_char_to_display(ert->ert[pos + i]);
			if (disp && strlen(disp) == 1) {
				char_display[i] = disp[0];
			} else {
				char_display[i] = '.';
			}
		}
		LOGP(DRADIO, LOGL_DEBUG, "RDS TX eRT: seg=%d pos=%d-%d encoding=%s "
		     "chars='%s'\n",
		     seg, pos, pos + 3,
		     ert->encoding == RDS_ERT_ENCODING_UTF8 ? "UTF-8" : "UCS-2",
		     char_display);
	}
	
	/* Advance segment, wrapping at 32 */
	ert->segment = (seg + 1) % RDS_ERT_SEGMENTS;
}

/* Build Group 3A: ODA Identification (IEC 62106 S6.1.5.5)
 *
 * Group 3A announces which group type carries a specific ODA application.
 * Receivers use this to build a map of active ODAs.
 *
 * Block B bits 4-0: Application Group Type Code
 *   - Indicates which group (0A-15B) carries this ODA's data
 *   - Format: [group_number(4 bits)][version(1 bit)] → 0=0A, 1=0B, ..., 31=15B
 *
 * Block C: ODA-specific message (application-dependent, 16 bits)
 *   - RT+: CB flag, SCB, template number
 *   - eRT: encoding, direction, character table
 *   - TMC: system info
 *
 * Block D: Application Identification (AID) - 16-bit registered code
 *   - 0x4BD7 = RT+, 0x6552 = eRT, 0xCD46/CD47 = TMC, etc.
 *
 * Cycling: When multiple ODAs are configured, this function cycles through
 * them, transmitting one 3A group per call with the next ODA in sequence.
 */
static void rds_build_group_3a(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b3, b4;
	
	/* Get current ODA configuration to transmit */
	if (rds->oda_count == 0) {
		/* No ODAs configured - should not be called, but handle gracefully */
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS TX 3A: No ODA configured, skipping\n");
		return;
	}
	
	/* Cycle through configured ODAs */
	int idx = rds->oda_cycle_idx % rds->oda_count;
	rds_oda_config_t *oda = &rds->oda[idx];
	
	/* Block A: PI code */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group type (3A) + PTY + TP + App Group Type Code (bits 4-0) */
	b2 = (RDS_GROUP_3A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     (oda->carrier_group & RDS_3A_APP_GROUP_MASK);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: ODA-specific message */
	b3 = oda->message;
	blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
	
	/* Block D: Application ID (AID) */
	b4 = oda->aid;
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Debug logging */
	if (rds->debug) {
		const char *oda_name = "Unknown";
		switch (oda->aid) {
		case RDS_ODA_AID_RT_PLUS:     oda_name = "RT+"; break;
		case RDS_ODA_AID_ERT:         oda_name = "eRT"; break;
		case RDS_ODA_AID_ERT_PLUS:    oda_name = "eRT+"; break;
		case RDS_ODA_AID_DAB_XREF:    oda_name = "DAB"; break;
		case RDS_ODA_AID_TMC_ALERT:
		case RDS_ODA_AID_TMC_ALERT2:  oda_name = "TMC"; break;
		case RDS_ODA_AID_STATION_LOGO: oda_name = "Logo"; break;
		}
		LOGP(DRADIO, LOGL_DEBUG, "RDS TX 3A: ODA[%d/%d] AID=0x%04X (%s) "
		     "carrier=%d%c msg=0x%04X\n",
		     idx + 1, rds->oda_count, oda->aid, oda_name,
		     oda->carrier_group >> 1,
		     (oda->carrier_group & 1) ? 'B' : 'A',
		     oda->message);
	}
	
	/* Pack into bytes */
	rds_group_pack(blocks, group, 0);
	
	/* Advance to next ODA for cycling */
	rds->oda_cycle_idx = (idx + 1) % rds->oda_count;
}

/* ============================================================
 * Modified Julian Date (MJD) Conversion Utilities
 * IEC 62106 Annex G
 * ============================================================
 * MJD is a continuous day count starting from midnight November 17, 1858.
 * RDS uses MJD because it provides a compact 17-bit representation of dates
 * from 1900 to ~2199, avoiding complex calendar calculations in receivers.
 *
 * The magic constants come from astronomical conventions:
 *   14956 = MJD of March 1, 1900 (epoch reference)
 *   365.25 = average days per year (accounts for leap years)
 *   30.6001 = average days per month (slight offset avoids rounding errors)
 * ============================================================ */

/*
 * Convert Gregorian date to Modified Julian Date
 *
 * Formula:
 *   L = 1 if M <= 2, else 0  (January/February adjustment)
 *   MJD = 14956 + D + floor((Y - L - 1900) * 365.25) + floor((M + 1 + L*12) * 30.6001)
 */
static int mjd_from_date(int year, int month, int day)
{
	int L = (month <= 2) ? 1 : 0;
	return 14956 + day + (int)((year - L - 1900) * 365.25) + (int)((month + 1 + L * 12) * 30.6001);
}

/*
 * Convert Modified Julian Date to Gregorian date
 *
 * This is the inverse of mjd_from_date(). We compute:
 *   y' = provisional year offset from 1900
 *   m' = provisional month (March=3 to February=14)
 *   k  = correction for Jan/Feb (they're months 13-14 of prev year)
 *
 * Formulas:
 *   y' = floor((MJD - 15078.2) / 365.25)
 *   m' = floor((MJD - 14956.1 - floor(y' * 365.25)) / 30.6001)
 *   D  = MJD - 14956 - floor(y' * 365.25) - floor(m' * 30.6001)
 *   k  = 1 if m' = 14 or 15, else 0
 *   Y  = 1900 + y' + k
 *   M  = m' - 1 - k * 12
 */
static void mjd_to_date(int mjd, int *year, int *month, int *day)
{
	int yp = (int)((mjd - 15078.2) / 365.25);
	int mp = (int)((mjd - 14956.1 - (int)(yp * 365.25)) / 30.6001);
	int k = (mp == 14 || mp == 15) ? 1 : 0;
	
	*day = mjd - 14956 - (int)(yp * 365.25) - (int)(mp * 30.6001);
	*year = 1900 + yp + k;
	*month = mp - 1 - k * 12;
}

/* Build Group 4A: Clock-Time */
static void rds_build_group_4a(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b3, b4;
	time_t now = time(NULL) + rds->ct_time_offset;
	struct tm *t = gmtime(&now);
	
	/* Calculate MJD using helper function */
	int mjd = mjd_from_date(t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
	
	int hour = t->tm_hour;
	int min = t->tm_min;
	int offset_half_hours = rds->local_offset;
	
#if defined(__GLIBC__) || defined(__linux__)
	/* Debug Log */
	{
		float off_h = offset_half_hours / 2.0;
		int l_h = t->tm_hour + (int)off_h;
		int l_m = t->tm_min + (int)((off_h - (int)off_h) * 60);
		while (l_m >= 60) { l_m -= 60; l_h++; }
		while (l_h >= 24) l_h -= 24;
		while (l_h < 0) l_h += 24;
		LOGP(DRADIO, LOGL_DEBUG, "RDS 4A: CT Encoding - UTC: %04d-%02d-%02d %02d:%02d, Offset: %+.1f hours, Local: %02d:%02d\n",
		     t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, off_h, l_h, l_m);
	}
#endif
	
	/* Block A: PI */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 4A + MJD bits 16-15 */
	b2 = (RDS_GROUP_4A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     ((mjd >> 15) & 0x03);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: MJD bits 14-0 + Hour bit 4 */
	b3 = ((mjd & 0x7FFF) << 1) | ((hour >> 4) & 1);
	blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
	
	/* Block D: Hour 3-0, Min, Offset */
	b4 = ((hour & 0x0F) << 12) | (min << 6) | (offset_half_hours & 0x1F);
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
}


/* Build Group 10A: PTYN */
static void rds_build_group_10a(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b3, b4;
	int seg = rds->ptyn_segment;
	int pos = seg * 4;
	
	/* Block A: PI */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 10A + A/B flag + segment */
	b2 = (RDS_GROUP_10A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     (rds->ptyn_ab << RDS_10A_AB_FLAG_BIT) |
	     (seg << RDS_10A_SEGMENT_BIT);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: PTYN chars 0-1 */
	b3 = ((uint8_t)rds->ptyn[pos] << 8) | (uint8_t)rds->ptyn[pos+1];
	blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
	
	/* Block D: PTYN chars 2-3 */
	b4 = ((uint8_t)rds->ptyn[pos+2] << 8) | (uint8_t)rds->ptyn[pos+3];
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	rds->ptyn_segment = (seg + 1) % 2;
}


/* Build Group 14A: Enhanced Other Networks (EON)
 * Transmits information about Other Networks one variant at a time.
 * Cycles through configured ONs and their data variants.
 */
static void rds_build_group_14a(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b3, b4;
	
	if (rds->eon_tx_count == 0)
		return;
	
	/* Get current ON entry */
	rds_eon_entry_t *eon = &rds->eon_tx[rds->eon_tx_index];
	int variant = rds->eon_tx_variant;
	
	/* Block A: PI */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 14A + TP(ON) + variant code */
	b2 = (RDS_GROUP_14A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     ((eon->tp ? 1 : 0) << RDS_14A_TP_ON_BIT) |
	     (variant & RDS_14A_USAGE_MASK);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: Variant-dependent data */
	switch (variant) {
	case RDS_14A_VARIANT_PS_0:
	case RDS_14A_VARIANT_PS_1:
	case RDS_14A_VARIANT_PS_2:
	case RDS_14A_VARIANT_PS_3:
		/* PS chars */
		b3 = ((uint8_t)eon->ps[variant*2] << 8) | (uint8_t)eon->ps[variant*2 + 1];
		break;
	case RDS_14A_VARIANT_AF:
		/* AF Method A (if we have AFs) */
		if (eon->af_count >= 2) {
			uint8_t af1 = (eon->af[0] > 875) ? (eon->af[0] - 875) : 0;
			uint8_t af2 = (eon->af[1] > 875) ? (eon->af[1] - 875) : 0;
			b3 = (af1 << 8) | af2;
		} else {
			b3 = RDS_AF_NO_AF_PAIR;  /* No AF */
		}
		break;
	case RDS_14A_VARIANT_MAP_5:
	case RDS_14A_VARIANT_MAP_6:
	case RDS_14A_VARIANT_MAP_7:
	case RDS_14A_VARIANT_MAP_8:
	case RDS_14A_VARIANT_MAP_9:
		/* Mapped AF (if available) */
		{
			int map_idx = variant - RDS_14A_VARIANT_MAP_5;
			if (map_idx < eon->mapped_af_count) {
				b3 = (eon->mapped_af[map_idx].tuned_af << 8) | 
				     eon->mapped_af[map_idx].on_af;
			} else {
				b3 = RDS_AF_NO_AF_PAIR;
			}
		}
		break;
	case RDS_14A_VARIANT_LINK:
		/* Linkage Information */
		b3 = ((eon->linkage_la ? 1 : 0) << RDS_14A_LINK_LA_BIT) |
		     (eon->linkage_lsn & RDS_14A_LINK_LSN_MASK);
		break;
	case RDS_14A_VARIANT_INFO:
		/* PTY and TA for ON */
		b3 = ((eon->pty & 0x1F) << RDS_14A_INFO_PTY_SHIFT) |
		     ((eon->ta ? 1 : 0) << RDS_14A_INFO_TA_BIT);
		break;
	case RDS_14A_VARIANT_PIN:
		/* PIN for Other Network (ON) - IEC 62106 S6.1.5.14
		 *
		 * IMPORTANT: This PIN applies to the LINKED service (Block D: ON-PI),
		 * NOT to the current service. It is used for:
		 *   - Scheduling programmes on a different PI
		 *   - Network-wide timer events
		 *   - Cross-station programme reminders
		 *
		 * Receiver support: Only advanced receivers decode Group 14A PIN.
		 * Most consumer receivers ignore this entirely and only trust Group 1A PIN.
		 *
		 * Even if the time value matches the current station's PIN, they
		 * refer to DIFFERENT services and should not be treated as duplicates.
		 */
		b3 = ((eon->pin_day & (RDS_PIN_DAY_MASK >> RDS_PIN_DAY_SHIFT)) << RDS_PIN_DAY_SHIFT) |
		     ((eon->pin_hour & (RDS_PIN_HOUR_MASK >> RDS_PIN_HOUR_SHIFT)) << RDS_PIN_HOUR_SHIFT) |
		     (eon->pin_minute & RDS_PIN_MINUTE_MASK);
		break;
	case RDS_14A_VARIANT_BCAST:
		/* Broadcaster data (reserved) */
		b3 = eon->broadcaster_data;
		break;
	default:
		b3 = 0;
		break;
	}
	blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
	
	/* Block D: ON-PI */
	b4 = eon->pi;
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Pack into bytes */
	rds_group_pack(blocks, group, 0);
	
	/* Advance to next variant/station - cycle through ALL variants for testing:
	 * 0-3: PS name (4 groups)
	 * 4:   AF list
	 * 5:   Mapped AF (first pair)
	 * 12:  Linkage information
	 * 13:  PTY/TA info
	 * 14:  PIN
	 * 15:  Broadcaster data
	 * Then move to next station */
	static const int variant_sequence[] = {
		RDS_14A_VARIANT_PS_0, RDS_14A_VARIANT_PS_1,
		RDS_14A_VARIANT_PS_2, RDS_14A_VARIANT_PS_3,
		RDS_14A_VARIANT_AF, RDS_14A_VARIANT_MAP_5,
		RDS_14A_VARIANT_LINK, RDS_14A_VARIANT_INFO,
		RDS_14A_VARIANT_PIN, RDS_14A_VARIANT_BCAST
	};
	static const int sequence_len = sizeof(variant_sequence) / sizeof(variant_sequence[0]);
	
	/* Find current position in sequence */
	int seq_pos;
	for (seq_pos = 0; seq_pos < sequence_len; seq_pos++) {
		if (variant_sequence[seq_pos] == variant)
			break;
	}
	
	/* Advance to next in sequence */
	seq_pos++;
	if (seq_pos >= sequence_len) {
		/* Completed full cycle, move to next station */
		rds->eon_tx_variant = variant_sequence[0];
		rds->eon_tx_index = (rds->eon_tx_index + 1) % rds->eon_tx_count;
	} else {
		rds->eon_tx_variant = variant_sequence[seq_pos];
	}
}


/* Build Group 14B: EON TA Flag Update
 * Simplified version for fast TA switching notification.
 */
static void rds_build_group_14b(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4] = {0};
	uint16_t b2, b4;
	
	if (rds->eon_tx_count == 0)
		return;
	
	/* Get current ON entry */
	rds_eon_entry_t *eon = &rds->eon_tx[rds->eon_tx_index];
	
	/* Block A: PI */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 14B + TP(ON) + TA(ON) */
	b2 = (RDS_GROUP_14B << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT) |
	     ((eon->tp ? 1 : 0) << RDS_14A_TP_ON_BIT) |
	     ((eon->ta ? 1 : 0) << 3);  /* TA in bit 3 for 14B */
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: PI repeat (tuning aid) */
	blocks[2] = rds_build_block(rds->pi, RDS_OFFSET_Cp);
	
	/* Block D: ON-PI */
	b4 = eon->pi;
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Pack into bytes */
	rds_group_pack(blocks, group, 0);
	
	/* Move to next station */
	rds->eon_tx_index = (rds->eon_tx_index + 1) % rds->eon_tx_count;
}


/* ============================================================
 * RDS GROUP SCHEDULER (Fixed Cyclic Sequence)
 * ============================================================
 * 
 * Implements a configurable fixed group sequence as recommended by
 * professional encoder manufacturers (2wcom, Pira.cz, etc.).
 *
 * The sequence is rebuilt whenever configuration changes (e.g. new RT).
 * It ensures:
 *   - High repetition of Group 0 (PS/AF/TA)
 *   - Adequate repetition of optional groups (RT, 1A, etc.)
 *   - Consistent timing structure
 *
 * Group 4A (CT) remains a high-priority interrupt at minute boundaries.
 * ============================================================ */

void rds_scheduler_update(rds_encoder_t *rds)
{
	rds->group_sched_len = 0;
	rds->group_sched_index = 0;
	
	int use_0b = rds->use_0b;
	/* RT is active if string is not empty/space (check first char) */
	int has_rt = (rds->rt[0] != ' ' && rds->rt[0] != '\0' && rds->rt[0] != '\r');
	int has_slc = (rds->ecc != 0 || rds->language != 0 || rds->pin_day > 0);
	int has_ptyn = (rds->ptyn[0] != ' ' && rds->ptyn[0] != '\0');
	int has_eon = (rds->eon_enabled && rds->eon_tx_count > 0);
	int has_oda = (rds->oda_count > 0);
	
	/* Use Group 2B if configured, else 2A */
	enum rds_group_type g2 = rds->use_2b ? RDS_GROUP_2B : RDS_GROUP_2A;
	enum rds_group_type g0 = use_0b ? RDS_GROUP_0B : RDS_GROUP_0A;
	enum rds_group_type g1 = rds->use_1b ? RDS_GROUP_1B : RDS_GROUP_1A;
	
	/* Helper to add group to sequence */
	#define ADD_GRP(g) do { \
		if(rds->group_sched_len < RDS_SCHEDULER_MAX_LEN) \
			rds->group_sched_buffer[rds->group_sched_len++] = (g); \
	} while(0)
	
	/* 
	 * BUILD SEQUENCE
	 * Pattern based on Pira.cz and 2wcom recommendations for "Full Features"
	 * Aim: ~50% Group 0, ~25% RT, ~25% Others
	 */

	/* Block 1: 0, 0 (High priority PS/AF) */
	ADD_GRP(g0);
	ADD_GRP(g0);
	
	/* Block 2: RT or 0 */
	if (has_rt) {
		ADD_GRP(g2);
		ADD_GRP(g2);
	} else {
		ADD_GRP(g0);
	}
	
	/* Block 3: 0, 0 */
	ADD_GRP(g0);
	ADD_GRP(g0);

	/* Block 4: SLC (1A) or RT or 0 */
	if (has_slc) {
		ADD_GRP(g1);
	} else if (has_rt) {
		ADD_GRP(g2);
	} else {
		ADD_GRP(g0);
	}
	
	/* Block 5: 0 */
	ADD_GRP(g0);
	
	/* Block 6: PTYN (10A) or RT or 0 */
	if (has_ptyn) {
		ADD_GRP(RDS_GROUP_10A);
	} else if (has_rt) {
		ADD_GRP(g2);
	} else {
		ADD_GRP(g0);
	}
	
	/* Block 7: EON (14A) - optional append */
	if (has_eon) {
		ADD_GRP(RDS_GROUP_14A);
	}
	
	/* Block 8: ODA identification (3A) - cycles through configured ODAs */
	if (has_oda) {
		ADD_GRP(RDS_GROUP_3A);
	}
	
	/* Block 9: RT+ ODA group (if configured and has tags) */
	for (int i = 0; i < rds->oda_count; i++) {
		if (rds->oda[i].aid == RDS_ODA_AID_RT_PLUS && rds->oda[i].enabled) {
			if (rds->rtplus.tag_count > 0) {
				enum rds_group_type rtplus_group = (enum rds_group_type)rds->oda[i].carrier_group;
				ADD_GRP(rtplus_group);
			}
			break;
		}
	}
	
	/* Block 10: eRT+ ODA group (if configured and has tags) */
	for (int i = 0; i < rds->oda_count; i++) {
		if (rds->oda[i].aid == RDS_ODA_AID_ERT_PLUS && rds->oda[i].enabled) {
			if (rds->ert_plus.tag_count > 0) {
				enum rds_group_type ert_plus_group = (enum rds_group_type)rds->oda[i].carrier_group;
				ADD_GRP(ert_plus_group);
			}
			break;
		}
	}
	
	/* Block 11: eRT ODA group (if configured and has text) */
	for (int i = 0; i < rds->oda_count; i++) {
		if (rds->oda[i].aid == RDS_ODA_AID_ERT && rds->oda[i].enabled) {
			if (rds->ert.length > 0) {
				enum rds_group_type ert_group = (enum rds_group_type)rds->oda[i].carrier_group;
				ADD_GRP(ert_group);
			}
			break;
		}
	}

	/* Log the new sequence */
	/* 
	char seq_str[256] = {0};
	for(int i=0; i<rds->group_sequence_len; i++) {
		const char *n = rds_group_name(rds->group_sequence[i]); // Requires lookup
		snprintf(seq_str+strlen(seq_str), sizeof(seq_str)-strlen(seq_str), "%s ", n ? n : "??");
	}
	LOGP(DRADIO, LOGL_INFO, "RDS Scheduler: Sequence updated (%d items): %s\n", rds->group_sequence_len, seq_str);
	*/
}

static void rds_generate_group(rds_encoder_t *rds)
{
	/* ============================================================
	 * DEBUG TEST MODE: Minimal Group 0A Cycle
	 * ============================================================
	 * When enabled, transmits ONLY Group 0A in a simple 4-group cycle
	 * (segments 0,1,2,3) with fixed known values. No CT, no RT, no PTYN.
	 * 
	 * Expected output (PI=1234, PS="TEST1234", no AF):
	 *   1234 0008 E0E0 5445   | PS[0]='TE'
	 *   1234 0009 E0E0 5354   | PS[1]='ST'
	 *   1234 000A E0E0 3132   | PS[2]='12'
	 *   1234 000F E0E0 3334   | PS[3]='34'
	 *   (repeat)
	 * ============================================================ */
	
	/* ============================================================
	 * PRIORITY 1: Time-Triggered Group 4A (Clock-Time)
	 * ============================================================
	 * IEC 62106 S6.1.5.4: CT transmitted at minute boundaries.
	 * Accuracy: within +/-100ms of UTC minute edge.
	 * Rate: 1 group per minute (not per second). */
	if (rds->ct_enabled) {
		time_t now = time(NULL) + rds->ct_time_offset;
		struct tm t_now_buf, t_last_buf;
		
		/* Use reentrant gmtime_r if available, or just copy immediately.
		 * Since we can't assume gmtime_r exists on all platforms (though likely on Linux),
		 * we'll use a safe pattern with standard gmtime */
		
		struct tm *ptr = gmtime(&now);
		if (ptr) memcpy(&t_now_buf, ptr, sizeof(struct tm));
		else memset(&t_now_buf, 0, sizeof(struct tm));
		
		ptr = gmtime(&rds->last_ct_minute);
		if (ptr) memcpy(&t_last_buf, ptr, sizeof(struct tm));
		else memset(&t_last_buf, 0, sizeof(struct tm));

		if (t_now_buf.tm_min != t_last_buf.tm_min) {
			rds_build_group_4a(rds, rds->group_buffer);
			rds->last_ct_minute = now;
			rds->group_sequence++; /* Increment total counter */
			rds->group_bit_pos = 0;
			// Debug log for CT
			// LOGP(DRADIO, LOGL_DEBUG, "RDS TX: Group 4A (CT Interrupt)\n");
			return;
		}
	}
	
	/* ============================================================
	 * PRIORITY 2: Warmup Mode (First ~4 seconds after start/preset)
	 * ============================================================
	 * Transmit ONLY Group 0 (PS + PI) to help receivers acquire quickly. */
	if (rds->warmup_countdown > 0) {
		rds->warmup_countdown--;
		if (rds->use_0b)
			rds_build_group_0b(rds, rds->group_buffer);
		else
			rds_build_group_0a(rds, rds->group_buffer);
		rds->group_sequence++;
		rds->group_bit_pos = 0;
		return;
	}
	
	/* ============================================================
	 * PRIORITY 3: Fixed Cyclic Sequence
	 * ============================================================ */
	
	/* Safety check: if sequence empty, fallback to 0A */
	if (rds->group_sched_len == 0) {
		rds_scheduler_update(rds);
		if (rds->group_sched_len == 0) { // Should not happen
			rds_build_group_0a(rds, rds->group_buffer); // Emergency fallback
			rds->group_bit_pos = 0;
			return;
		}
	}
	
	/* Get next group from sequence */
	enum rds_group_type type = rds->group_sched_buffer[rds->group_sched_index];

	switch (type) {
	case RDS_GROUP_0A:
		rds_build_group_0a(rds, rds->group_buffer);
		break;
	case RDS_GROUP_0B:
		rds_build_group_0b(rds, rds->group_buffer);
		break;
	case RDS_GROUP_1A:
		rds_build_group_1a(rds, rds->group_buffer);
		break;
	case RDS_GROUP_1B:
		rds_build_group_1b(rds, rds->group_buffer);
		break;
	case RDS_GROUP_2A:
		rds_build_group_2a(rds, rds->group_buffer);
		break;
	case RDS_GROUP_2B:
		rds_build_group_2b(rds, rds->group_buffer);
		break;
	case RDS_GROUP_3A:
		rds_build_group_3a(rds, rds->group_buffer);
		break;
	case RDS_GROUP_10A:
		rds_build_group_10a(rds, rds->group_buffer);
		break;
	case RDS_GROUP_14A:
		rds_build_group_14a(rds, rds->group_buffer);
		break;
	case RDS_GROUP_14B:
		rds_build_group_14b(rds, rds->group_buffer);
		break;
	/* ODA groups (11A-13B, etc.) - route to specific ODA builders */
	case RDS_GROUP_11A:
	case RDS_GROUP_11B:
	case RDS_GROUP_12A:
	case RDS_GROUP_12B:
	case RDS_GROUP_13A:
	case RDS_GROUP_13B:
		/* Route to ODA-specific builders based on configured ODAs */
		{
			int carrier_code = (int)type;
			uint32_t blocks[4] = {0};
			uint16_t b2, b3, b4;
			int found = 0;
			
			/* Check if this carrier has an active ODA */
			for (int i = 0; i < rds->oda_count; i++) {
				if (rds->oda[i].carrier_group == carrier_code && rds->oda[i].enabled) {
					uint16_t aid = rds->oda[i].aid;
					
					/* Block A: PI code */
					blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
					
					if (aid == RDS_ODA_AID_RT_PLUS) {
						/* RT+ group */
						rds_build_rtplus_group(rds, &rds->rtplus, &b2, &b3, &b4);
						b2 |= (type << RDS_B2_GROUP_SHIFT);
						blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
						blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
						blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
						found = 1;
						break;
					} else if (aid == RDS_ODA_AID_ERT_PLUS) {
						/* eRT+ group (same format as RT+ but operates on eRT text) */
						rds_build_rtplus_group(rds, &rds->ert_plus, &b2, &b3, &b4);
						b2 |= (type << RDS_B2_GROUP_SHIFT);
						blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
						blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
						blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
						found = 1;
						break;
					} else if (aid == RDS_ODA_AID_ERT) {
						/* eRT group */
						rds_build_ert_group(rds, &b2, &b3, &b4);
						b2 |= (type << RDS_B2_GROUP_SHIFT);
						blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
						blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
						blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
						found = 1;
						break;
					}
				}
			}
			
			if (found) {
				rds_group_pack(blocks, rds->group_buffer, 0);
			} else {
				/* No ODA configured for this group - fallback to 0A */
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS TX: ODA group %d%c not configured, using 0A\n",
					     carrier_code >> 1, (carrier_code & 1) ? 'B' : 'A');
				rds_build_group_0a(rds, rds->group_buffer);
			}
		}
		break;
	/* Add other cases as needed */
	default:
		/* Fallback for unhandled types in sequence */
		rds_build_group_0a(rds, rds->group_buffer);
		break;
	}

	
	/* Advance index */
	rds->group_sched_index++;
	if (rds->group_sched_index >= rds->group_sched_len) {
		rds->group_sched_index = 0;
	}
	
	// /* Debug: Log every group generated in RDS Spy format */
	// {
	// 	/* 
	// 	 * The group_buffer is 13 bytes of packed 104 bits (4 x 26-bit blocks).
	// 	 * To extract the 16-bit DATA portion of each block, we need to unpack
	// 	 * from the 26-bit block positions, NOT just read byte pairs.
	// 	 * 
	// 	 * Block structure (26 bits each):
	// 	 *   Block A: bits 103-78 -> data in bits 103-88 (16 bits)
	// 	 *   Block B: bits  77-52 -> data in bits  77-62 (16 bits)
	// 	 *   Block C: bits  51-26 -> data in bits  51-36 (16 bits)
	// 	 *   Block D: bits  25-0  -> data in bits  25-10 (16 bits)
	// 	 *
	// 	 * Repacking from byte buffer:
	// 	 */
	// 	uint32_t all_bits[4];  /* Full 26-bit blocks */
		
	// 	/* Unpack 104 bits from 13 bytes back to 4 x 26-bit blocks */
	// 	all_bits[0] = ((uint32_t)rds->group_buffer[0] << 18) |
	// 	              ((uint32_t)rds->group_buffer[1] << 10) |
	// 	              ((uint32_t)rds->group_buffer[2] << 2) |
	// 	              ((uint32_t)rds->group_buffer[3] >> 6);
		
	// 	all_bits[1] = (((uint32_t)rds->group_buffer[3] & 0x3F) << 20) |
	// 	              ((uint32_t)rds->group_buffer[4] << 12) |
	// 	              ((uint32_t)rds->group_buffer[5] << 4) |
	// 	              ((uint32_t)rds->group_buffer[6] >> 4);
		
	// 	all_bits[2] = (((uint32_t)rds->group_buffer[6] & 0x0F) << 22) |
	// 	              ((uint32_t)rds->group_buffer[7] << 14) |
	// 	              ((uint32_t)rds->group_buffer[8] << 6) |
	// 	              ((uint32_t)rds->group_buffer[9] >> 2);
		
	// 	all_bits[3] = (((uint32_t)rds->group_buffer[9] & 0x03) << 24) |
	// 	              ((uint32_t)rds->group_buffer[10] << 16) |
	// 	              ((uint32_t)rds->group_buffer[11] << 8) |
	// 	              ((uint32_t)rds->group_buffer[12]);
		
	// 	/* Extract 16-bit data from 26-bit blocks (upper 16 bits) */
	// 	uint16_t blk_a = (all_bits[0] >> 10) & 0xFFFF;
	// 	uint16_t blk_b = (all_bits[1] >> 10) & 0xFFFF;
	// 	uint16_t blk_c = (all_bits[2] >> 10) & 0xFFFF;
	// 	uint16_t blk_d = (all_bits[3] >> 10) & 0xFFFF;
		
	// 	/* Log in RDS Spy format: PI BLKB BLKC BLKD */
	// 	LOGP(DRADIO, LOGL_DEBUG, "RDS TX [%s] seq=%lu: %04X %04X %04X %04X",
	// 	     group_name, (unsigned long)rds->group_sequence, 
	// 	     blk_a, blk_b, blk_c, blk_d);
		
	// 	/* Add extra info for specific group types */
	// 	if (type == RDS_GROUP_0A || type == RDS_GROUP_0B) {
	// 		int seg = blk_b & 0x03;
	// 		char c1 = (blk_d >> 8) & 0xFF;
	// 		char c2 = blk_d & 0xFF;
	// 		LOGP(DRADIO, LOGL_DEBUG, " | PS[%d]='%c%c'", seg, 
	// 		     (c1 >= 32 && c1 < 127) ? c1 : '?',
	// 		     (c2 >= 32 && c2 < 127) ? c2 : '?');
	// 	} else if (type == RDS_GROUP_2A || type == RDS_GROUP_2B) {
	// 		int seg = blk_b & 0x0F;
	// 		LOGP(DRADIO, LOGL_DEBUG, " | RT[%d]", seg);
	// 	} else if (type == RDS_GROUP_10A) {
	// 		int seg = blk_b & 0x01;
	// 		LOGP(DRADIO, LOGL_DEBUG, " | PTYN[%d]", seg);
	// 	}
	// 	LOGP(DRADIO, LOGL_DEBUG, "\n");
	// }
	
	rds->group_sequence++;
	rds->group_bit_pos = 0;
}

/* Get next bit from current group */
static int rds_get_next_bit(rds_encoder_t *rds)
{
	/* Check if we need to generate a new group BEFORE calculating indices */
	if (rds->group_bit_pos >= 104) {
		rds_generate_group(rds);
	}
	
	int byte_pos = rds->group_bit_pos / 8;
	int bit_pos = 7 - (rds->group_bit_pos % 8);
	int bit;
	
	if (byte_pos >= 13) {
		// Should not happen if generate_group resets pos to 0
		return 0;
	}
	
	bit = (rds->group_buffer[byte_pos] >> bit_pos) & 1;
	rds->group_bit_pos++;
	
	return bit;
}

int rds_encoder_init(rds_encoder_t *rds, double samplerate, uint16_t pi,
		     const char *ps, const char *rt, uint8_t pty, const char *ptyn)
{
	memset(rds, 0, sizeof(*rds));
	
	rds->samplerate = samplerate;
	rds->pi = pi;
	rds->pty = pty & 0x1F;
	rds->tp = 0;
	rds->ta = 0;
	rds->ms = 1;  /* Music */
	
	/* Detect system timezone offset and set default local_offset */
	time_t t_init = time(NULL);
	struct tm *lt_init = localtime(&t_init);
#if defined(__GLIBC__) || defined(__linux__)
	rds->local_offset = lt_init->tm_gmtoff / 1800; /* Seconds / 1800 = half-hours */
#else
	rds->local_offset = 0; /* Fallback: Assume UTC if unknown */
#endif
	
	/* Set PS name (pad with spaces) */
	memset(rds->ps, ' ', 8);
	rds->ps[8] = '\0';
	if (ps) {
		int len = strlen(ps);
		if (len > 8) len = 8;
		memcpy(rds->ps, ps, len);
	}
	
	/* Set RadioText (pad with CR after message end)
	 * EN 50067: Unused positions filled with 0x0D to mark end of text */
	memset(rds->rt, '\r', 64);  /* Fill with CR terminators */
	rds->rt[64] = '\0';
	if (rt) {
		int len = strlen(rt);
		if (len > 64) len = 64;
		memcpy(rds->rt, rt, len);
		/* Positions after message remain as CR from memset above */
	}
	rds->rt_last_version = 0xFF;  /* No version used yet */
	
	/* Calculate phase steps */
	rds->phasestep = 2.0 * M_PI * RDS_SUBCARRIER / samplerate;
	rds->bit_phasestep = RDS_BITRATE / samplerate;
	
	/* Initialize differential encoder */
	rds->last_diff_bit = 0;
	/* Initialize bit history to valid bipolar values (not zeros!) to ensure
	 * proper pulse shaping from the first bit. Zeros cause malformed symbols
	 * at startup because the RRC filter convolves with these invalid values.
	 * Values must be +1 or -1 to match the bipolar NRZ encoding used later. */
	rds->bit_history[0] = 1;  /* Matches last_diff_bit=0 -> +1 */
	rds->bit_history[1] = 1;
	rds->bit_history[2] = 1;
	
	/* Initialize PTYN with default or override */
	memset(rds->ptyn, ' ', 8);
	rds->ptyn[8] = '\0';
	
	const char *final_ptyn = NULL;
	if (ptyn) {
		final_ptyn = ptyn;
	} else {
		/* Default to standard PTY name - detect RBDS from encoder's ECC */
		final_ptyn = rds_get_pty_name(rds->pty, RDS_IS_RBDS(rds->ecc));
	}

	if (final_ptyn) {
		int len = strlen(final_ptyn);
		if (len > 8) len = 8;
		memcpy(rds->ptyn, final_ptyn, len);
	}
	
	/* Default CT enabled */
	rds->ct_enabled = 1;
	
	/* Start warmup mode: Group 0 only for ~5 seconds (57 groups @ 11.4/sec) */
	rds->warmup_countdown = 57;

	/* Generate RRC biphase waveform at runtime */
	if (rds_generate_biphase_waveform(&rds->waveform_biphase) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Failed to generate RDS biphase waveform\n");
		return -1;
	}
	

	
	/* Initialize Group Scheduler */
	rds_scheduler_update(rds);

	/* Generate first group */
	rds_generate_group(rds);
	
	LOGP(DRADIO, LOGL_INFO, "RDS encoder initialized: PI=%04X PS=\"%s\" RT=\"%s\" PTY=%d\n",
	     pi, ps ? ps : "", rt ? rt : "", pty);
	LOGP(DRADIO, LOGL_INFO, "RDS Config: CT=%d, Offset=%+.1fh (%s), ECC=%02X, PIN=%02d/%02d:%02d, PTYN=\"%.8s\"\n",
	     rds->ct_enabled, rds->local_offset/2.0, (rds->local_offset == 0) ? "UTC" : "Local",
	     rds->ecc, rds->pin_day, rds->pin_hour, rds->pin_minute, rds->ptyn);
	LOGP(DRADIO, LOGL_INFO, "RDS Groups: 0A (PS), 2A (RT), 1A (ECC), 10A (PTYN), 4A (CT)\n");
	
	return 0;
}

void rds_encoder_process(rds_encoder_t *rds, sample_t *samples, int num,
			 double pilot_phase, double pilot_phasestep)
{
	int i;
	double phase = pilot_phase;
	static int debug_count = 0;
	double max_injection = 0.0;
	
	/* Bit consumption tracking for diagnostics */
	static unsigned long total_bits_consumed = 0;
	static unsigned long total_samples_processed = 0;
	int bits_this_call = 0;
	(void)total_bits_consumed;  /* Suppress unused warning */
	(void)total_samples_processed;
	
	for (i = 0; i < num; i++) {
		/* Advance bit timing */
		rds->bit_phase += rds->bit_phasestep;
		while (rds->bit_phase >= 1.0) {
			rds->bit_phase -= 1.0;
			bits_this_call++;
			
			/* Shift bit history */
			rds->bit_history[2] = rds->bit_history[1];
			rds->bit_history[1] = rds->bit_history[0];
			
			/* Get next bit, apply differential encoding, and store as signed impulse */
			int data_bit = rds_get_next_bit(rds);
			if (data_bit)
				rds->last_diff_bit = !rds->last_diff_bit;
			
			/* 
			 * Convert to bipolar NRZ for pulse shaping.
			 * IEC 62106 S2.2: DBPSK uses +/-1 symbol levels.
			 * Polarity choice is arbitrary (either works due to differential encoding).
			 */
			rds->bit_history[0] = rds->last_diff_bit ? -1 : 1;
		}

		
		/* 
		 * Convolve bit history with RRC/Biphase waveform.
		 * Waveform table spans 3 bit periods (0..576 samples).
		 * 
		 * Current bit (history[0]) is at phase 0..1 -> Table index 0..192
		 * Previous bit (history[1]) is at phase 1..2 -> Table index 192..384
		 * Pre-prev bit (history[2]) is at phase 2..3 -> Table index 384..576
		 */
		double sample_val = 0.0;
		int k;
		
		for (k = 0; k < 3; k++) {
			/* Phase for this bit's contribution */
			double p = rds->bit_phase + k; 
			
			/* Map to table index (0..576) */
			double table_idx = p * 192.0;
			
			/* Linear interpolation */
			int idx0 = (int)table_idx;
			int idx1 = idx0 + 1;
			if (idx0 >= 575) idx0 = 575;
			if (idx1 >= 575) idx1 = 575;
			
			double frac = table_idx - idx0;
			double val = rds->waveform_biphase[idx0] * (1.0 - frac) + rds->waveform_biphase[idx1] * frac;
			
			sample_val += rds->bit_history[k] * val;
		}
		
		/* Generate RDS signal */
		/* Signal is AM modulated on 57 kHz suppressed carrier (BPSK) */
		/* Ideally: signal * sin(57k_t) */
		
		/* 3x pilot phase */
		double rds_phase = phase * 3.0;
		
		/* Modulate */
		double subcarrier;
		if (fm_fast_math_enabled()) {
			double sc_sin, sc_cos;
			fm_fast_sincos(rds_phase * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
			subcarrier = sample_val * sc_sin;
		} else {
			subcarrier = sample_val * sin(rds_phase);
		}
		double injection = subcarrier * RDS_INJECTION * -2.0; /* Scale factor; polarity is arbitrary per IEC 62106 DBPSK */
		/* Note: waveform values are small (~0.5 max), so we need gain to reach 5% injection */
		
		/* Track max injection for debug */
		if (fabs(injection) > max_injection)
			max_injection = fabs(injection);
		
		/* Add to samples with injection level */
		samples[i] += injection;
		
		/* Advance pilot phase (same as stereo loop) */
		phase += pilot_phasestep;
		if (phase >= 2.0 * M_PI)
			phase -= 2.0 * M_PI;
	}
	
	/* Update totals */
	total_bits_consumed += bits_this_call;
	total_samples_processed += num;
	
	/* Debug: log bit consumption for diagnostics */

	
	/* Debug: log occasionally to confirm RDS is being added */
	if (++debug_count >= 100) {
		/* LOGP(DRADIO, LOGL_DEBUG, "RDS: injecting (max=%.4f, phasestep=%.6f, samples=%d)\n",
		     max_injection, pilot_phasestep, num); */
		debug_count = 0;
	}
	
	/* Save phase state */
	rds->phase = phase;
}

void rds_enc_set_radiotext(rds_encoder_t *rds, const char *rt)
{
	if (!rt) return;
	
	/* Validate input text and warn about unencodable characters */
	rds_validate_text(rt, "RadioText");
	
	/* Clear buffer with spaces */
	/* Clear buffer with CR terminators (unused positions = 0x0D)
	 * EN 50067: Positions after message end should be 0x0D */
	memset(rds->rt, '\r', 64);
	
	/* Convert UTF-8 to RDS encoding
	 * - ASCII maps directly
	 * - LF (0x0A) preserved for line breaks
	 * - CR (0x0D) terminates the message
	 * - Other control chars are skipped
	 * - Non-encodable Unicode chars become space */
	int warn = 0;
	int len = rds_encode_text(rt, (uint8_t *)rds->rt, 64, &warn);
	
	/* Positions from len onwards remain as CR from memset above */
	rds->rt[64] = '\0';
	
	/* Toggle A/B flag to trigger receivers to clear display
	 * EN 50067: "If the receiver detects a change in the flag...
	 * then the whole RadioText display should be cleared" */
	uint8_t old_ab = rds->rt_ab;
	rds->rt_ab = !rds->rt_ab;
	rds->rt_segment = 0;
	
	/* Log RT change with new A/B flag */
	LOGP(DRADIO, LOGL_INFO, "RDS: RadioText set (%d chars), A/B flag toggled %c->%c\n",
	     len, old_ab ? 'B' : 'A', rds->rt_ab ? 'B' : 'A');
	/* Convert RDS-encoded RT to Unicode for logging */
	char rt_display[257];  /* 64 chars * 4 bytes UTF-8 max + 1 */
	rds_text_to_display((uint8_t *)rds->rt, 64, rt_display, sizeof(rt_display));
	LOGP(DRADIO, LOGL_DEBUG, "RDS: RT content: \"%s\"\n", rt_display);
	
	/* Update scheduler sequence (RT presence may have changed) */
	rds_scheduler_update(rds);
}

void rds_enc_clear_radiotext(rds_encoder_t *rds)
{
	/* Set RT to 64 CR terminators (empty RT) */
	memset(rds->rt, '\r', 64);
	rds->rt[64] = '\0';
	rds->rt_segment = 0;
	
	/* Update scheduler (RT presence may have changed) */
	rds_scheduler_update(rds);
	
	LOGP(DRADIO, LOGL_INFO, "RDS: RadioText cleared\n");
}

void rds_enc_get_radiotext(const rds_encoder_t *rds, char *rt, size_t len)
{
	if (!rt || len == 0) return;
	
	size_t copy_len = (len - 1 < 64) ? len - 1 : 64;
	memcpy(rt, rds->rt, copy_len);
	rt[copy_len] = '\0';
}

void rds_enc_set_ta(rds_encoder_t *rds, int ta)
{
	rds->ta = ta ? 1 : 0;
}

/* ============================================================
 * Dynamic RDS Configuration API - Phase 1: Core Fields
 * ============================================================ */

void rds_enc_set_pi(rds_encoder_t *rds, uint16_t pi)
{
	uint16_t old_pi = rds->pi;
	rds->pi = pi;
	
	if (old_pi != pi) {
		LOGP(DRADIO, LOGL_INFO, "RDS: PI set to 0x%04X\n", pi);
	}
}

uint16_t rds_enc_get_pi(const rds_encoder_t *rds)
{
	return rds->pi;
}

void rds_enc_set_ps(rds_encoder_t *rds, const char *ps)
{
	if (!ps) return;
	
	/* Validate input text and warn about unencodable characters */
	rds_validate_text(ps, "PS");
	
	/* Clear buffer with spaces */
	memset(rds->ps, ' ', 8);
	rds->ps[8] = '\0';
	
	/* Convert UTF-8 to RDS encoding */
	int warn = 0;
	rds_encode_text(ps, (uint8_t *)rds->ps, 8, &warn);
	
	/* Reset PS segment for fresh transmission */
	rds->ps_segment = 0;
	
	/* Convert RDS-encoded PS to Unicode for logging */
	char ps_display[33];  /* 8 chars * 4 bytes UTF-8 max + 1 */
	rds_text_to_display((uint8_t *)rds->ps, 8, ps_display, sizeof(ps_display));
	LOGP(DRADIO, LOGL_INFO, "RDS: PS set to \"%s\"\n", ps_display);
}

void rds_enc_clear_ps(rds_encoder_t *rds)
{
	memset(rds->ps, ' ', 8);
	rds->ps[8] = '\0';
	rds->ps_segment = 0;
	
	LOGP(DRADIO, LOGL_INFO, "RDS: PS cleared\n");
}

void rds_enc_get_ps(const rds_encoder_t *rds, char *ps, size_t len)
{
	if (!ps || len == 0) return;
	
	size_t copy_len = (len - 1 < 8) ? len - 1 : 8;
	memcpy(ps, rds->ps, copy_len);
	ps[copy_len] = '\0';
}

/* ============================================================
 * RT+ (RadioText Plus) Encoder API Implementation
 * ============================================================ */

int rds_enc_rtplus_add_tag(rds_encoder_t *rds, uint8_t content_type, 
                           uint8_t start, uint8_t length)
{
	rds_rtplus_encoder_t *rtplus = &rds->rtplus;
	
	if (rtplus->tag_count >= RDS_RTPLUS_MAX_TAGS) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS RT+: Tag array full (max %d tags)\n",
		     RDS_RTPLUS_MAX_TAGS);
		return -1;
	}
	
	if (content_type > 64) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS RT+: Invalid content_type %d (max 64)\n",
		     content_type);
		return -1;
	}
	
	if (start > RDS_RTPLUS_MAX_START) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS RT+: Invalid start %d (max %d)\n",
		     start, RDS_RTPLUS_MAX_START);
		return -1;
	}
	
	uint8_t max_len = (rtplus->tag_count == 0) ? RDS_RTPLUS_MAX_LEN_TAG1 : RDS_RTPLUS_MAX_LEN_TAG2;
	if (length == 0 || length > max_len) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS RT+: Invalid length %d (max %d for tag %d)\n",
		     length, max_len, rtplus->tag_count + 1);
		return -1;
	}
	
	rds_rtplus_tag_t *tag = &rtplus->tags[rtplus->tag_count];
	tag->content_type = content_type;
	tag->start = start;
	tag->length = length;
	rtplus->tag_count++;
	
	const char *ct_name = rds_get_rtplus_content_type(content_type);
	LOGP(DRADIO, LOGL_INFO, "RDS RT+: Added tag%d type=%d (%s) start=%d len=%d\n",
	     rtplus->tag_count, content_type, ct_name ? ct_name : "Unknown", start, length);
	
	return 0;
}

void rds_enc_rtplus_clear_tags(rds_encoder_t *rds)
{
	rds->rtplus.tag_count = 0;
	LOGP(DRADIO, LOGL_INFO, "RDS RT+: Tags cleared\n");
}

void rds_enc_rtplus_set_toggle(rds_encoder_t *rds, int toggle)
{
	rds->rtplus.toggle = (toggle != 0) ? 1 : 0;
	if (rds->debug)
		LOGP(DRADIO, LOGL_DEBUG, "RDS RT+: Toggle set to %d\n", rds->rtplus.toggle);
}

void rds_enc_rtplus_set_item_running(rds_encoder_t *rds, int running)
{
	rds->rtplus.item_running = (running != 0) ? 1 : 0;
	if (rds->debug)
		LOGP(DRADIO, LOGL_DEBUG, "RDS RT+: Item running set to %d\n", rds->rtplus.item_running);
}

int rds_enc_rtplus_get_tag_count(const rds_encoder_t *rds)
{
	return rds->rtplus.tag_count;
}

int rds_enc_rtplus_get_tag(const rds_encoder_t *rds, int index,
                            uint8_t *content_type, uint8_t *start, uint8_t *length)
{
	if (index < 0 || index >= rds->rtplus.tag_count) {
		return -1;
	}
	
	if (content_type) *content_type = rds->rtplus.tags[index].content_type;
	if (start) *start = rds->rtplus.tags[index].start;
	if (length) *length = rds->rtplus.tags[index].length;
	
	return 0;
}

/* eRT+ API (same as RT+ but for eRT) */
int rds_enc_ert_plus_add_tag(rds_encoder_t *rds, uint8_t content_type,
                             uint8_t start, uint8_t length)
{
	rds_rtplus_encoder_t *ert_plus = &rds->ert_plus;
	
	if (ert_plus->tag_count >= RDS_RTPLUS_MAX_TAGS) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS eRT+: Tag array full (max %d tags)\n",
		     RDS_RTPLUS_MAX_TAGS);
		return -1;
	}
	
	if (content_type > 64) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS eRT+: Invalid content_type %d (max 64)\n",
		     content_type);
		return -1;
	}
	
	/* eRT supports up to 128 bytes, so start can be up to 127 */
	if (start > 127) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS eRT+: Invalid start %d (max 127)\n", start);
		return -1;
	}
	
	uint8_t max_len = (ert_plus->tag_count == 0) ? RDS_RTPLUS_MAX_LEN_TAG1 : RDS_RTPLUS_MAX_LEN_TAG2;
	if (length == 0 || length > max_len) {
		LOGP(DRADIO, LOGL_NOTICE, "RDS eRT+: Invalid length %d (max %d for tag %d)\n",
		     length, max_len, ert_plus->tag_count + 1);
		return -1;
	}
	
	rds_rtplus_tag_t *tag = &ert_plus->tags[ert_plus->tag_count];
	tag->content_type = content_type;
	tag->start = start;
	tag->length = length;
	ert_plus->tag_count++;
	
	const char *ct_name = rds_get_rtplus_content_type(content_type);
	LOGP(DRADIO, LOGL_INFO, "RDS eRT+: Added tag%d type=%d (%s) start=%d len=%d\n",
	     ert_plus->tag_count, content_type, ct_name ? ct_name : "Unknown", start, length);
	
	return 0;
}

void rds_enc_ert_plus_clear_tags(rds_encoder_t *rds)
{
	rds->ert_plus.tag_count = 0;
	LOGP(DRADIO, LOGL_INFO, "RDS eRT+: Tags cleared\n");
}

int rds_enc_ert_plus_get_tag_count(const rds_encoder_t *rds)
{
	return rds->ert_plus.tag_count;
}

int rds_enc_ert_plus_get_tag(const rds_encoder_t *rds, int index,
                              uint8_t *content_type, uint8_t *start, uint8_t *length)
{
	if (index < 0 || index >= rds->ert_plus.tag_count) {
		return -1;
	}
	
	if (content_type) *content_type = rds->ert_plus.tags[index].content_type;
	if (start) *start = rds->ert_plus.tags[index].start;
	if (length) *length = rds->ert_plus.tags[index].length;
	
	return 0;
}

/* ============================================================
 * eRT (Enhanced RadioText) Encoder API Implementation
 * ============================================================ */

void rds_enc_set_ert(rds_encoder_t *rds, const uint8_t *text, size_t len)
{
	rds_ert_encoder_t *ert = &rds->ert;
	
	if (!text) {
		rds_enc_clear_ert(rds);
		return;
	}
	
	/* Limit to 128 bytes */
	if (len > RDS_ERT_LENGTH) {
		len = RDS_ERT_LENGTH;
		LOGP(DRADIO, LOGL_NOTICE, "RDS eRT: Text truncated to %d bytes\n",
		     RDS_ERT_LENGTH);
	}
	
	memset(ert->ert, 0, RDS_ERT_LENGTH + 1);
	memcpy(ert->ert, text, len);
	ert->length = len;
	ert->segment = 0;  /* Reset segment for fresh transmission */
	
	/* Convert to display-safe UTF-8 for logging */
	char ert_display[513];  /* 128 bytes * 4 bytes UTF-8 max + 1 */
	int display_len = rds_text_to_display(ert->ert, len, ert_display, sizeof(ert_display));
	LOGP(DRADIO, LOGL_INFO, "RDS eRT: Set %zu bytes (encoding=%s): \"%.*s\"\n",
	     len, ert->encoding == RDS_ERT_ENCODING_UTF8 ? "UTF-8" : "UCS-2",
	     display_len, ert_display);
}

void rds_enc_clear_ert(rds_encoder_t *rds)
{
	rds_ert_encoder_t *ert = &rds->ert;
	memset(ert->ert, 0, RDS_ERT_LENGTH + 1);
	ert->length = 0;
	ert->segment = 0;
	LOGP(DRADIO, LOGL_INFO, "RDS eRT: Cleared\n");
}

void rds_enc_get_ert(const rds_encoder_t *rds, uint8_t *text, size_t *len, size_t max_len)
{
	if (!text || !len) return;
	
	const rds_ert_encoder_t *ert = &rds->ert;
	size_t copy_len = (max_len < ert->length) ? max_len : ert->length;
	memcpy(text, ert->ert, copy_len);
	*len = copy_len;
}

/* ============================================================
 * RT+ and eRT Decoder API Implementation
 * ============================================================ */

int rds_dec_rtplus_get_tag_count(const rds_decoder_t *rds)
{
	return rds->rtplus.tag_count;
}

int rds_dec_rtplus_get_tag(const rds_decoder_t *rds, int index,
                            uint8_t *content_type, uint8_t *start, uint8_t *length)
{
	if (index < 0 || index >= rds->rtplus.tag_count) {
		return -1;
	}
	
	if (content_type) *content_type = rds->rtplus.tags[index].content_type;
	if (start) *start = rds->rtplus.tags[index].start;
	if (length) *length = rds->rtplus.tags[index].length;
	
	return 0;
}

int rds_dec_rtplus_get_toggle(const rds_decoder_t *rds)
{
	return rds->rtplus.toggle;
}

int rds_dec_rtplus_get_item_running(const rds_decoder_t *rds)
{
	return rds->rtplus.item_running;
}

int rds_dec_ert_plus_get_tag_count(const rds_decoder_t *rds)
{
	return rds->ert_plus.tag_count;
}

int rds_dec_ert_plus_get_tag(const rds_decoder_t *rds, int index,
                              uint8_t *content_type, uint8_t *start, uint8_t *length)
{
	if (index < 0 || index >= rds->ert_plus.tag_count) {
		return -1;
	}
	
	if (content_type) *content_type = rds->ert_plus.tags[index].content_type;
	if (start) *start = rds->ert_plus.tags[index].start;
	if (length) *length = rds->ert_plus.tags[index].length;
	
	return 0;
}

void rds_dec_get_ert(const rds_decoder_t *rds, uint8_t *text, size_t *len, size_t max_len)
{
	if (!text || !len) return;
	
	const rds_ert_decoder_t *ert = &rds->ert_dec;
	size_t copy_len = (max_len < RDS_ERT_LENGTH) ? max_len : RDS_ERT_LENGTH;
	memcpy(text, ert->ert, copy_len);
	*len = copy_len;
}

int rds_dec_get_ert_encoding(const rds_decoder_t *rds)
{
	return rds->ert_dec.encoding;
}

int rds_dec_get_ert_direction(const rds_decoder_t *rds)
{
	return rds->ert_dec.direction;
}

void rds_enc_set_pty(rds_encoder_t *rds, uint8_t pty)
{
	if (!RDS_VALID_PTY(pty)) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid PTY %d (must be 0-31)\n", pty);
		return;
	}
	
	uint8_t old_pty = rds->pty;
	rds->pty = pty & 0x1F;
	
	if (old_pty != rds->pty) {
		LOGP(DRADIO, LOGL_INFO, "RDS: PTY set to %d\n", rds->pty);
	}
}

uint8_t rds_enc_get_pty(const rds_encoder_t *rds)
{
	return rds->pty;
}

void rds_enc_set_ptyn(rds_encoder_t *rds, const char *ptyn)
{
	if (!ptyn) return;
	
	/* Validate input text and warn about unencodable characters */
	rds_validate_text(ptyn, "PTYN");
	
	/* Clear buffer with spaces */
	memset(rds->ptyn, ' ', 8);
	rds->ptyn[8] = '\0';
	
	/* Convert UTF-8 to RDS encoding */
	int warn = 0;
	rds_encode_text(ptyn, (uint8_t *)rds->ptyn, 8, &warn);
	
	/* Reset PTYN segment for fresh transmission */
	rds->ptyn_segment = 0;
	
	/* Update scheduler (PTYN presence may have changed) */
	rds_scheduler_update(rds);
	
	/* Convert RDS-encoded PTYN to Unicode for logging */
	char ptyn_display[33];  /* 8 chars * 4 bytes UTF-8 max + 1 */
	rds_text_to_display((uint8_t *)rds->ptyn, 8, ptyn_display, sizeof(ptyn_display));
	LOGP(DRADIO, LOGL_INFO, "RDS: PTYN set to \"%s\"\n", ptyn_display);
}

void rds_enc_clear_ptyn(rds_encoder_t *rds)
{
	memset(rds->ptyn, ' ', 8);
	rds->ptyn[8] = '\0';
	rds->ptyn_segment = 0;
	
	/* Update scheduler (PTYN presence may have changed) */
	rds_scheduler_update(rds);
	
	LOGP(DRADIO, LOGL_INFO, "RDS: PTYN cleared\n");
}

void rds_enc_get_ptyn(const rds_encoder_t *rds, char *ptyn, size_t len)
{
	if (!ptyn || len == 0) return;
	
	size_t copy_len = (len - 1 < 8) ? len - 1 : 8;
	memcpy(ptyn, rds->ptyn, copy_len);
	ptyn[copy_len] = '\0';
}

void rds_enc_set_tp(rds_encoder_t *rds, int tp)
{
	int old_tp = rds->tp;
	rds->tp = tp ? 1 : 0;
	
	if (old_tp != rds->tp) {
		LOGP(DRADIO, LOGL_INFO, "RDS: TP set to %d\n", rds->tp);
	}
}

int rds_enc_get_tp(const rds_encoder_t *rds)
{
	return rds->tp;
}

void rds_enc_set_ms(rds_encoder_t *rds, int ms)
{
	int old_ms = rds->ms;
	rds->ms = ms ? 1 : 0;
	
	if (old_ms != rds->ms) {
		LOGP(DRADIO, LOGL_INFO, "RDS: MS set to %d (%s)\n", 
		     rds->ms, rds->ms ? "Music" : "Speech");
	}
}

int rds_enc_get_ms(const rds_encoder_t *rds)
{
	return rds->ms;
}

/* ============================================================
 * Dynamic RDS Configuration API - Phase 2: Extended Info
 * ============================================================ */

void rds_enc_set_ecc(rds_encoder_t *rds, uint8_t ecc)
{
	uint8_t old_ecc = rds->ecc;
	rds->ecc = ecc;
	
	if (old_ecc != ecc) {
		LOGP(DRADIO, LOGL_INFO, "RDS: ECC set to 0x%02X\n", ecc);
		/* Update scheduler (ECC presence affects Group 1A scheduling) */
		rds_scheduler_update(rds);
	}
}

void rds_enc_clear_ecc(rds_encoder_t *rds)
{
	if (rds->ecc != 0) {
		rds->ecc = 0;
		LOGP(DRADIO, LOGL_INFO, "RDS: ECC cleared\n");
		/* Update scheduler (ECC presence affects Group 1A scheduling) */
		rds_scheduler_update(rds);
	}
}

uint8_t rds_enc_get_ecc(const rds_encoder_t *rds)
{
	return rds->ecc;
}

void rds_enc_set_language(rds_encoder_t *rds, uint8_t lang)
{
	if (lang > 127) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid language code %d (must be 0-127)\n", lang);
		return;
	}
	
	uint8_t old_lang = rds->language;
	rds->language = lang;
	
	if (old_lang != lang) {
		LOGP(DRADIO, LOGL_INFO, "RDS: Language set to %d\n", lang);
		/* Update scheduler (language presence affects Group 1A scheduling) */
		rds_scheduler_update(rds);
	}
}

void rds_enc_clear_language(rds_encoder_t *rds)
{
	if (rds->language != 0) {
		rds->language = 0;
		LOGP(DRADIO, LOGL_INFO, "RDS: Language cleared\n");
		/* Update scheduler (language presence affects Group 1A scheduling) */
		rds_scheduler_update(rds);
	}
}

uint8_t rds_enc_get_language(const rds_encoder_t *rds)
{
	return rds->language;
}

void rds_enc_set_pin(rds_encoder_t *rds, uint8_t day, uint8_t hour, uint8_t minute)
{
	/* Validation */
	if (day > 31) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid PIN day %d (must be 0-31)\n", day);
		return;
	}
	if (hour > 24) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid PIN hour %d (must be 0-24)\n", hour);
		return;
	}
	if (minute > 59) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid PIN minute %d (must be 0-59)\n", minute);
		return;
	}
	
	uint8_t old_day = rds->pin_day;
	rds->pin_day = day;
	rds->pin_hour = hour;
	rds->pin_minute = minute;
	
	if (old_day != day || rds->pin_hour != hour || rds->pin_minute != minute) {
		LOGP(DRADIO, LOGL_INFO, "RDS: PIN set to day=%d, %02d:%02d\n", 
		     day, hour, minute);
		/* Update scheduler (PIN presence affects Group 1A scheduling) */
		rds_scheduler_update(rds);
	}
}

void rds_enc_clear_pin(rds_encoder_t *rds)
{
	if (rds->pin_day != 0 || rds->pin_hour != 0 || rds->pin_minute != 0) {
		rds->pin_day = 0;
		rds->pin_hour = 0;
		rds->pin_minute = 0;
		LOGP(DRADIO, LOGL_INFO, "RDS: PIN cleared\n");
		/* Update scheduler (PIN presence affects Group 1A scheduling) */
		rds_scheduler_update(rds);
	}
}

void rds_enc_get_pin(const rds_encoder_t *rds, uint8_t *day, uint8_t *hour, uint8_t *minute)
{
	if (day) *day = rds->pin_day;
	if (hour) *hour = rds->pin_hour;
	if (minute) *minute = rds->pin_minute;
}

void rds_enc_set_di(rds_encoder_t *rds, int stereo, int artificial, int compressed, int dynamic_pty)
{
	/* Validation: each flag must be 0 or 1 */
	if (stereo != 0 && stereo != 1) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid DI stereo flag %d (must be 0 or 1)\n", stereo);
		return;
	}
	if (artificial != 0 && artificial != 1) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid DI artificial flag %d (must be 0 or 1)\n", artificial);
		return;
	}
	if (compressed != 0 && compressed != 1) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid DI compressed flag %d (must be 0 or 1)\n", compressed);
		return;
	}
	if (dynamic_pty != 0 && dynamic_pty != 1) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid DI dynamic_pty flag %d (must be 0 or 1)\n", dynamic_pty);
		return;
	}
	
	int old_stereo = rds->di_stereo;
	int old_artificial = rds->di_artificial_head;
	int old_compressed = rds->di_compressed;
	int old_dynamic_pty = rds->di_dynamic_pty;
	
	rds->di_stereo = stereo ? 1 : 0;
	rds->di_artificial_head = artificial ? 1 : 0;
	rds->di_compressed = compressed ? 1 : 0;
	rds->di_dynamic_pty = dynamic_pty ? 1 : 0;
	
	if (old_stereo != rds->di_stereo || old_artificial != rds->di_artificial_head ||
	    old_compressed != rds->di_compressed || old_dynamic_pty != rds->di_dynamic_pty) {
		LOGP(DRADIO, LOGL_INFO, "RDS: DI set to stereo=%d, artificial=%d, compressed=%d, dynamic_pty=%d\n",
		     rds->di_stereo, rds->di_artificial_head, rds->di_compressed, rds->di_dynamic_pty);
	}
}

void rds_enc_get_di(const rds_encoder_t *rds, int *stereo, int *artificial, int *compressed, int *dynamic_pty)
{
	if (stereo) *stereo = rds->di_stereo;
	if (artificial) *artificial = rds->di_artificial_head;
	if (compressed) *compressed = rds->di_compressed;
	if (dynamic_pty) *dynamic_pty = rds->di_dynamic_pty;
}

/* ============================================================
 * Dynamic RDS Configuration API - Phase 3: Alternative Frequencies
 * ============================================================ */

int rds_enc_af_set_method_a(rds_encoder_t *rds, const char *af_string)
{
	if (!af_string || af_string[0] == '\0') {
		rds_enc_af_clear(rds);
		return 0;
	}
	
	/* Clear Method B if active */
	rds->use_method_b = 0;
	memset(&rds->af_method_b, 0, sizeof(rds->af_method_b));
	
	/* Parse Method A string */
	if (rds_af_method_a_parse(af_string, &rds->af_method_a) != 0) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Failed to parse AF Method A string: %s\n", af_string);
		return -1;
	}
	
	/* Build pre-computed code sequence for transmission */
	rds->af_code_count = rds_af_method_a_build_codes(
		&rds->af_method_a, rds->af_codes, sizeof(rds->af_codes));
	
	if (rds->af_code_count > 0) {
		LOGP(DRADIO, LOGL_INFO, "RDS: AF Method A set: %d codes (%d VHF + %d LF/MF = %d frequencies)\n",
		     rds->af_code_count, rds->af_method_a.vhf_count, rds->af_method_a.lf_mf_count,
		     rds->af_method_a.slot_count);
		rds->af_method_a_segment = 0;
		/* Update scheduler (AF presence affects Group 0 version) */
		rds->use_0b = 0;  /* Method A requires Group 0A */
		rds_scheduler_update(rds);
		return 0;
	}
	
	return -1;
}

int rds_enc_af_get_method_a_count(const rds_encoder_t *rds)
{
	if (rds->use_method_b || rds->af_code_count == 0)
		return 0;
	return rds->af_method_a.vhf_count;
}

int rds_enc_af_method_b_add(rds_encoder_t *rds, const char *af_string)
{
	if (!af_string || af_string[0] == '\0') {
		return -1;
	}
	
	/* Check if array is full */
	if (rds->af_method_b.list_count >= RDS_AF_METHOD_B_MAX_LISTS) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Cannot add AF Method B list - max %d lists reached\n",
		     RDS_AF_METHOD_B_MAX_LISTS);
		return -1;
	}
	
	/* Parse Method B string */
	rds_af_method_b_list_t new_list;
	if (rds_af_method_b_parse(af_string, &new_list) != 0) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Failed to parse AF Method B string: %s\n", af_string);
		return -1;
	}
	
	/* Add to array */
	int idx = rds->af_method_b.list_count;
	rds->af_method_b.lists[idx] = new_list;
	rds->af_method_b.list_count++;
	
	/* Enable Method B */
	rds->use_method_b = 1;
	rds->af_method_b_list_idx = 0;
	rds->af_method_b_pair_idx = 0;
	
	/* Clear Method A */
	memset(&rds->af_method_a, 0, sizeof(rds->af_method_a));
	rds->af_code_count = 0;
	
	LOGP(DRADIO, LOGL_INFO, "RDS: AF Method B list[%d] added: tuning=%.1f, %d AFs\n",
	     idx, new_list.tuning_freq / 10.0, new_list.af_count);
	
	/* Update scheduler (AF presence affects Group 0 version) */
	rds->use_0b = 0;  /* Method B requires Group 0A */
	rds_scheduler_update(rds);
	
	return 0;
}

int rds_enc_af_method_b_add_list(rds_encoder_t *rds, uint16_t tuning_freq, const uint16_t *af_freqs, const uint8_t *af_is_regional, uint8_t af_count)
{
	/* Validation */
	if (tuning_freq < 876 || tuning_freq > 1079) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid tuning frequency %d (must be 876-1079 for 87.6-107.9 MHz)\n", tuning_freq);
		return -1;
	}
	if (af_count == 0 || af_count > RDS_AF_METHOD_B_MAX_AFS) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid AF count %d (must be 1-%d)\n", af_count, RDS_AF_METHOD_B_MAX_AFS);
		return -1;
	}
	if (!af_freqs || !af_is_regional) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: NULL pointer for AF frequencies or regional flags\n");
		return -1;
	}
	
	/* Validate AF frequencies */
	for (int i = 0; i < af_count; i++) {
		if (af_freqs[i] < 876 || af_freqs[i] > 1079) {
			LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid AF frequency %d at index %d (must be 876-1079)\n", af_freqs[i], i);
			return -1;
		}
		if (af_is_regional[i] != 0 && af_is_regional[i] != 1) {
			LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid regional flag %d at index %d (must be 0 or 1)\n", af_is_regional[i], i);
			return -1;
		}
	}
	
	/* Check if array is full */
	if (rds->af_method_b.list_count >= RDS_AF_METHOD_B_MAX_LISTS) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Cannot add AF Method B list - max %d lists reached\n",
		     RDS_AF_METHOD_B_MAX_LISTS);
		return -1;
	}
	
	/* Add to array */
	int idx = rds->af_method_b.list_count;
	rds_af_method_b_list_t *list = &rds->af_method_b.lists[idx];
	list->tuning_freq = tuning_freq;
	list->af_count = af_count;
	memcpy(list->af_freq, af_freqs, af_count * sizeof(uint16_t));
	memcpy(list->af_is_regional, af_is_regional, af_count * sizeof(uint8_t));
	rds->af_method_b.list_count++;
	
	/* Enable Method B */
	rds->use_method_b = 1;
	rds->af_method_b_list_idx = 0;
	rds->af_method_b_pair_idx = 0;
	
	/* Clear Method A */
	memset(&rds->af_method_a, 0, sizeof(rds->af_method_a));
	rds->af_code_count = 0;
	
	LOGP(DRADIO, LOGL_INFO, "RDS: AF Method B list[%d] added: tuning=%.1f, %d AFs\n",
	     idx, tuning_freq / 10.0, af_count);
	
	/* Update scheduler (AF presence affects Group 0 version) */
	rds->use_0b = 0;  /* Method B requires Group 0A */
	rds_scheduler_update(rds);
	
	return 0;
}

int rds_enc_af_method_b_remove_list(rds_encoder_t *rds, uint16_t tuning_freq)
{
	for (int i = 0; i < rds->af_method_b.list_count; i++) {
		if (rds->af_method_b.lists[i].tuning_freq == tuning_freq) {
			/* Shift remaining lists */
			for (int j = i; j < rds->af_method_b.list_count - 1; j++) {
				rds->af_method_b.lists[j] = rds->af_method_b.lists[j + 1];
			}
			rds->af_method_b.list_count--;
			
			/* Reset indices if needed */
			if (rds->af_method_b_list_idx >= rds->af_method_b.list_count) {
				rds->af_method_b_list_idx = 0;
				rds->af_method_b_pair_idx = 0;
			}
			
			/* Disable Method B if no lists remain */
			if (rds->af_method_b.list_count == 0) {
				rds->use_method_b = 0;
				rds->use_0b = 1;  /* No AF → use Group 0B */
			}
			
			LOGP(DRADIO, LOGL_INFO, "RDS: AF Method B list removed: tuning=%.1f\n", tuning_freq / 10.0);
			rds_scheduler_update(rds);
			return 0;
		}
	}
	
	return -1;  /* Not found */
}

int rds_enc_af_method_b_remove_list_by_index(rds_encoder_t *rds, int index)
{
	if (index < 0 || index >= rds->af_method_b.list_count) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid AF Method B list index %d (must be 0-%d)\n",
		     index, rds->af_method_b.list_count - 1);
		return -1;
	}
	
	uint16_t tuning_freq = rds->af_method_b.lists[index].tuning_freq;
	
	/* Shift remaining lists */
	for (int j = index; j < rds->af_method_b.list_count - 1; j++) {
		rds->af_method_b.lists[j] = rds->af_method_b.lists[j + 1];
	}
	rds->af_method_b.list_count--;
	
	/* Reset indices if needed */
	if (rds->af_method_b_list_idx >= rds->af_method_b.list_count) {
		rds->af_method_b_list_idx = 0;
		rds->af_method_b_pair_idx = 0;
	} else if (rds->af_method_b_list_idx > index) {
		rds->af_method_b_list_idx--;
	}
	
	/* Disable Method B if no lists remain */
	if (rds->af_method_b.list_count == 0) {
		rds->use_method_b = 0;
		rds->use_0b = 1;  /* No AF → use Group 0B */
	}
	
	LOGP(DRADIO, LOGL_INFO, "RDS: AF Method B list[%d] removed: tuning=%.1f\n", index, tuning_freq / 10.0);
	rds_scheduler_update(rds);
	return 0;
}

int rds_enc_af_method_b_add_entry(rds_encoder_t *rds, uint16_t tuning_freq, uint16_t af_freq, int is_regional)
{
	/* Validation */
	if (af_freq < 876 || af_freq > 1079) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid AF frequency %d (must be 876-1079 for 87.6-107.9 MHz)\n", af_freq);
		return -1;
	}
	if (is_regional != 0 && is_regional != 1) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid regional flag %d (must be 0 or 1)\n", is_regional);
		return -1;
	}
	
	/* Find list with matching tuning frequency */
	for (int i = 0; i < rds->af_method_b.list_count; i++) {
		if (rds->af_method_b.lists[i].tuning_freq == tuning_freq) {
			rds_af_method_b_list_t *list = &rds->af_method_b.lists[i];
			
			/* Check if list is full */
			if (list->af_count >= RDS_AF_METHOD_B_MAX_AFS) {
				LOGP(DRADIO, LOGL_ERROR, "RDS: AF Method B list is full (max %d AFs)\n",
				     RDS_AF_METHOD_B_MAX_AFS);
				return -1;
			}
			
			/* Check for duplicate */
			for (int j = 0; j < list->af_count; j++) {
				if (list->af_freq[j] == af_freq) {
					LOGP(DRADIO, LOGL_NOTICE, "RDS: AF frequency %.1f already exists in list\n", af_freq / 10.0);
					return -1;
				}
			}
			
			/* Add entry */
			list->af_freq[list->af_count] = af_freq;
			list->af_is_regional[list->af_count] = is_regional ? 1 : 0;
			list->af_count++;
			
			LOGP(DRADIO, LOGL_INFO, "RDS: AF Method B entry added: tuning=%.1f, AF=%.1f, regional=%d\n",
			     tuning_freq / 10.0, af_freq / 10.0, is_regional);
			return 0;
		}
	}
	
	LOGP(DRADIO, LOGL_ERROR, "RDS: AF Method B list not found for tuning frequency %.1f\n", tuning_freq / 10.0);
	return -1;
}

int rds_enc_af_method_b_remove_entry(rds_encoder_t *rds, uint16_t tuning_freq, uint16_t af_freq)
{
	/* Find list with matching tuning frequency */
	for (int i = 0; i < rds->af_method_b.list_count; i++) {
		if (rds->af_method_b.lists[i].tuning_freq == tuning_freq) {
			rds_af_method_b_list_t *list = &rds->af_method_b.lists[i];
			
			/* Find entry with matching AF frequency */
			for (int j = 0; j < list->af_count; j++) {
				if (list->af_freq[j] == af_freq) {
					/* Shift remaining entries */
					for (int k = j; k < list->af_count - 1; k++) {
						list->af_freq[k] = list->af_freq[k + 1];
						list->af_is_regional[k] = list->af_is_regional[k + 1];
					}
					list->af_count--;
					
					LOGP(DRADIO, LOGL_INFO, "RDS: AF Method B entry removed: tuning=%.1f, AF=%.1f\n",
					     tuning_freq / 10.0, af_freq / 10.0);
					return 0;
				}
			}
			
			LOGP(DRADIO, LOGL_ERROR, "RDS: AF frequency %.1f not found in list\n", af_freq / 10.0);
			return -1;
		}
	}
	
	LOGP(DRADIO, LOGL_ERROR, "RDS: AF Method B list not found for tuning frequency %.1f\n", tuning_freq / 10.0);
	return -1;
}

int rds_enc_af_method_b_remove_entry_by_index(rds_encoder_t *rds, uint16_t tuning_freq, int af_index)
{
	/* Find list with matching tuning frequency */
	for (int i = 0; i < rds->af_method_b.list_count; i++) {
		if (rds->af_method_b.lists[i].tuning_freq == tuning_freq) {
			rds_af_method_b_list_t *list = &rds->af_method_b.lists[i];
			
			if (af_index < 0 || af_index >= list->af_count) {
				LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid AF index %d (must be 0-%d)\n",
				     af_index, list->af_count - 1);
				return -1;
			}
			
			uint16_t af_freq = list->af_freq[af_index];
			
			/* Shift remaining entries */
			for (int j = af_index; j < list->af_count - 1; j++) {
				list->af_freq[j] = list->af_freq[j + 1];
				list->af_is_regional[j] = list->af_is_regional[j + 1];
			}
			list->af_count--;
			
			LOGP(DRADIO, LOGL_INFO, "RDS: AF Method B entry[%d] removed: tuning=%.1f, AF=%.1f\n",
			     af_index, tuning_freq / 10.0, af_freq / 10.0);
			return 0;
		}
	}
	
	LOGP(DRADIO, LOGL_ERROR, "RDS: AF Method B list not found for tuning frequency %.1f\n", tuning_freq / 10.0);
	return -1;
}

void rds_enc_af_clear(rds_encoder_t *rds)
{
	/* Clear Method A */
	memset(&rds->af_method_a, 0, sizeof(rds->af_method_a));
	rds->af_code_count = 0;
	rds->af_method_a_segment = 0;
	
	/* Clear Method B */
	memset(&rds->af_method_b, 0, sizeof(rds->af_method_b));
	rds->af_method_b_list_idx = 0;
	rds->af_method_b_pair_idx = 0;
	rds->use_method_b = 0;
	
	/* Clear pre-computed codes */
	memset(rds->af_codes, 0, sizeof(rds->af_codes));
	
	/* Switch to Group 0B (no AF) */
	rds->use_0b = 1;
	
	LOGP(DRADIO, LOGL_INFO, "RDS: AF cleared (both Method A and B)\n");
	rds_scheduler_update(rds);
}

int rds_enc_af_get_method(const rds_encoder_t *rds)
{
	if (rds->use_method_b && rds->af_method_b.list_count > 0)
		return 2;  /* Method B */
	if (rds->af_code_count > 0)
		return 1;  /* Method A */
	return 0;  /* No AF */
}

int rds_enc_af_method_b_get_list_count(const rds_encoder_t *rds)
{
	return rds->af_method_b.list_count;
}

int rds_enc_af_method_b_get_list(const rds_encoder_t *rds, int index, uint16_t *tuning_freq, uint16_t *af_freqs, uint8_t *af_is_regional, uint8_t *af_count, size_t max_afs)
{
	if (index < 0 || index >= rds->af_method_b.list_count) {
		return -1;
	}
	
	const rds_af_method_b_list_t *list = &rds->af_method_b.lists[index];
	
	if (tuning_freq) *tuning_freq = list->tuning_freq;
	if (af_count) *af_count = list->af_count;
	
	if (af_freqs && af_is_regional && max_afs > 0) {
		size_t copy_count = (list->af_count < max_afs) ? list->af_count : max_afs;
		memcpy(af_freqs, list->af_freq, copy_count * sizeof(uint16_t));
		memcpy(af_is_regional, list->af_is_regional, copy_count * sizeof(uint8_t));
	}
	
	return 0;
}

/* ============================================================
 * Dynamic RDS Configuration API - Phase 5: EON
 * ============================================================ */

int rds_enc_eon_add(rds_encoder_t *rds, uint16_t pi, const char *ps, uint8_t pty, uint8_t tp)
{
	/* Validation */
	if (pi == 0) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid EON PI 0x0000\n");
		return -1;
	}
	if (!RDS_VALID_PTY(pty)) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid EON PTY %d (must be 0-31)\n", pty);
		return -1;
	}
	if (tp != 0 && tp != 1) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid EON TP %d (must be 0 or 1)\n", tp);
		return -1;
	}
	
	/* Check if array is full */
	if (rds->eon_tx_count >= RDS_EON_MAX_ENTRIES) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Cannot add EON entry - max %d entries reached\n",
		     RDS_EON_MAX_ENTRIES);
		return -1;
	}
	
	/* Check for duplicate PI */
	for (int i = 0; i < rds->eon_tx_count; i++) {
		if (rds->eon_tx[i].pi == pi) {
			LOGP(DRADIO, LOGL_NOTICE, "RDS: EON entry with PI 0x%04X already exists, updating\n", pi);
			/* Update existing entry */
			rds_eon_entry_t *eon = &rds->eon_tx[i];
			memset(eon->ps, ' ', 8);
			eon->ps[8] = '\0';
			if (ps && ps[0] != '\0') {
				int len = strlen(ps);
				if (len > 8) len = 8;
				memcpy(eon->ps, ps, len);
			}
			eon->pty = pty;
			eon->tp = tp;
			rds_scheduler_update(rds);
			return 0;
		}
	}
	
	/* Add new entry */
	rds_eon_entry_t *eon = &rds->eon_tx[rds->eon_tx_count];
	memset(eon, 0, sizeof(*eon));
	eon->pi = pi;
	memset(eon->ps, ' ', 8);
	eon->ps[8] = '\0';
	if (ps && ps[0] != '\0') {
		int len = strlen(ps);
		if (len > 8) len = 8;
		memcpy(eon->ps, ps, len);
	}
	eon->pty = pty;
	eon->tp = tp;
	rds->eon_tx_count++;
	rds->eon_enabled = 1;
	
	/* Convert RDS-encoded EON PS to Unicode for logging */
	char eon_ps_display[33];  /* 8 chars * 4 bytes UTF-8 max + 1 */
	rds_text_to_display((uint8_t *)eon->ps, 8, eon_ps_display, sizeof(eon_ps_display));
	LOGP(DRADIO, LOGL_INFO, "RDS: EON entry added: PI=0x%04X, PS=\"%s\", PTY=%d, TP=%d\n",
	     pi, eon_ps_display, pty, tp);
	
	rds_scheduler_update(rds);
	return 0;
}

int rds_enc_eon_set_ta(rds_encoder_t *rds, uint16_t pi, int ta)
{
	if (ta != 0 && ta != 1) {
		LOGP(DRADIO, LOGL_ERROR, "RDS: Invalid EON TA %d (must be 0 or 1)\n", ta);
		return -1;
	}
	
	/* Find EON entry with matching PI */
	for (int i = 0; i < rds->eon_tx_count; i++) {
		if (rds->eon_tx[i].pi == pi) {
			rds->eon_tx[i].ta = ta ? 1 : 0;
			LOGP(DRADIO, LOGL_INFO, "RDS: EON TA set: PI=0x%04X, TA=%d\n", pi, rds->eon_tx[i].ta);
			return 0;
		}
	}
	
	LOGP(DRADIO, LOGL_ERROR, "RDS: EON entry not found for PI 0x%04X\n", pi);
	return -1;
}

int rds_enc_eon_remove(rds_encoder_t *rds, uint16_t pi)
{
	/* Find EON entry with matching PI */
	for (int i = 0; i < rds->eon_tx_count; i++) {
		if (rds->eon_tx[i].pi == pi) {
			/* Shift remaining entries */
			for (int j = i; j < rds->eon_tx_count - 1; j++) {
				rds->eon_tx[j] = rds->eon_tx[j + 1];
			}
			rds->eon_tx_count--;
			
			/* Disable EON if no entries remain */
			if (rds->eon_tx_count == 0) {
				rds->eon_enabled = 0;
			}
			
			/* Reset indices if needed */
			if (rds->eon_tx_index >= rds->eon_tx_count) {
				rds->eon_tx_index = 0;
				rds->eon_tx_variant = 0;
			}
			
			LOGP(DRADIO, LOGL_INFO, "RDS: EON entry removed: PI=0x%04X\n", pi);
			rds_scheduler_update(rds);
			return 0;
		}
	}
	
	LOGP(DRADIO, LOGL_ERROR, "RDS: EON entry not found for PI 0x%04X\n", pi);
	return -1;
}

void rds_enc_eon_clear(rds_encoder_t *rds)
{
	if (rds->eon_tx_count > 0) {
		memset(rds->eon_tx, 0, sizeof(rds->eon_tx));
		rds->eon_tx_count = 0;
		rds->eon_enabled = 0;
		rds->eon_tx_index = 0;
		rds->eon_tx_variant = 0;
		
		LOGP(DRADIO, LOGL_INFO, "RDS: EON cleared\n");
		rds_scheduler_update(rds);
	}
}

int rds_enc_eon_get_count(const rds_encoder_t *rds)
{
	return rds->eon_tx_count;
}

int rds_enc_eon_get_entry(const rds_encoder_t *rds, int index, uint16_t *pi, char *ps, size_t ps_len, uint8_t *pty, uint8_t *tp, uint8_t *ta)
{
	if (index < 0 || index >= rds->eon_tx_count) {
		return -1;
	}
	
	const rds_eon_entry_t *eon = &rds->eon_tx[index];
	
	if (pi) *pi = eon->pi;
	if (ps && ps_len > 0) {
		size_t copy_len = (ps_len - 1 < 8) ? ps_len - 1 : 8;
		memcpy(ps, eon->ps, copy_len);
		ps[copy_len] = '\0';
	}
	if (pty) *pty = eon->pty;
	if (tp) *tp = eon->tp;
	if (ta) *ta = eon->ta;
	
	return 0;
}

/* ============================================================
 * ODA (Open Data Application) Configuration API
 * ============================================================ */

int rds_enc_oda_add(rds_encoder_t *rds, uint8_t carrier_group, uint16_t aid, uint16_t message)
{
	/* Check for duplicates (same AID already configured) */
	for (int i = 0; i < rds->oda_count; i++) {
		if (rds->oda[i].aid == aid) {
			/* Update existing entry */
			rds->oda[i].carrier_group = carrier_group;
			rds->oda[i].message = message;
			rds->oda[i].enabled = 1;
			rds_scheduler_update(rds);
			return 0;
		}
	}
	
	/* Add new entry */
	if (rds->oda_count >= RDS_ODA_MAX_CONFIGS) {
		LOGP(DRADIO, LOGL_ERROR, "RDS ODA: Cannot add - max %d ODAs reached\n",
		     RDS_ODA_MAX_CONFIGS);
		return -1;
	}
	
	rds->oda[rds->oda_count].carrier_group = carrier_group;
	rds->oda[rds->oda_count].aid = aid;
	rds->oda[rds->oda_count].message = message;
	rds->oda[rds->oda_count].enabled = 1;
	rds->oda_count++;
	
	if (rds->debug)
		LOGP(DRADIO, LOGL_DEBUG, "RDS ODA: Added AID=0x%04X on Group %d%c\n",
		     aid, carrier_group >> 1, (carrier_group & 1) ? 'B' : 'A');
	
	rds_scheduler_update(rds);
	return 0;
}

int rds_enc_oda_remove(rds_encoder_t *rds, uint16_t aid)
{
	for (int i = 0; i < rds->oda_count; i++) {
		if (rds->oda[i].aid == aid) {
			/* Shift remaining entries down */
			for (int j = i; j < rds->oda_count - 1; j++) {
				rds->oda[j] = rds->oda[j + 1];
			}
			rds->oda_count--;
			
			/* Reset cycling index if needed */
			if (rds->oda_cycle_idx >= rds->oda_count)
				rds->oda_cycle_idx = 0;
			
			if (rds->debug)
				LOGP(DRADIO, LOGL_DEBUG, "RDS ODA: Removed AID=0x%04X\n", aid);
			
			rds_scheduler_update(rds);
			return 0;
		}
	}
	return -1;  /* Not found */
}

void rds_enc_oda_clear(rds_encoder_t *rds)
{
	rds->oda_count = 0;
	rds->oda_cycle_idx = 0;
	memset(rds->oda, 0, sizeof(rds->oda));
	
	if (rds->debug)
		LOGP(DRADIO, LOGL_DEBUG, "RDS ODA: Cleared all ODAs\n");
	
	rds_scheduler_update(rds);
}

void rds_encoder_exit(rds_encoder_t *rds)
{
	if (rds->waveform_biphase) {
		free(rds->waveform_biphase);
		rds->waveform_biphase = NULL;
	}
}


/* ==================== RDS2 ENCODER (PLACEHOLDER) ====================
 *
 * RDS2 (IEC 62106-2:2021) extends RDS with 3 additional BPSK subcarriers:
 *
 *   Stream 1:  57.0 kHz   (original RDS, 3 x 19 kHz pilot, phase-locked)
 *   Stream 2:  66.5 kHz   (NOT a pilot harmonic, free-running)
 *   Stream 3:  71.25 kHz  (NOT a pilot harmonic, free-running)
 *   Stream 4:  76.0 kHz   (4 x 19 kHz pilot, can be phase-locked)
 *
 * IMPLEMENTATION NOTES:
 * ---------------------
 * 1. Each stream runs at 1187.5 bps (same as original RDS)
 * 2. Combined bitrate: 4 x 1187.5 = 4750 bps
 * 3. Streams 2 & 3 frequencies are NOT harmonics of pilot:
 *    - 66.5 kHz = 19 kHz x 3.5   (fractional)
 *    - 71.25 kHz = 19 kHz x 3.75 (fractional)
 *    Therefore, they cannot be derived from pilot phase multiplication.
 *    They require independent NCOs (Numerically Controlled Oscillators).
 * 
 * 4. Stream 4 at 76 kHz = 4 x 19 kHz CAN be pilot-locked.
 *
 * GROUP TYPE C:
 * -------------
 * RDS2 streams 2-4 use a new "Group Type C" structure optimized for:
 * - Extended RadioText (eRT): 128 bytes UTF-8
 * - Long PS names via Group 15A
 * - RDS2 File Transfer (RFT): up to 163 KB files
 * - IPv6 address transmission for hybrid FM/IP linking
 *
 * INJECTION LEVELS:
 * -----------------
 * Each additional RDS2 stream typically uses same injection as original:
 * - IEC recommends +/-2.0 kHz per stream (~2.7% of +/-75 kHz)
 * - Total RDS2 deviation: up to +/-8 kHz (all 4 streams)
 *
 * REAL-WORLD NOTES:
 * -----------------
 * - RDS2 receiver adoption is very limited (as of 2024)
 * - Most car radios don't decode streams 2-4
 * - Mainly useful for future-proofing or specific ODA applications
 *
 * TODO: Implement rds2_encoder_t with:
 *       - 3 additional BPSK modulators
 *       - Group Type C data structures
 *       - UTF-8 RadioText/PS encoding
 *       - RFT file transfer protocol
 * ===================================================================== */

#if 0  /* RDS2 encoder stub - enable when implementing */

typedef struct rds2_encoder {
	/* Stream 2-4 oscillator phases (66.5, 71.25, 76 kHz) */
	double		stream2_phase;
	double		stream2_phasestep;
	double		stream3_phase;
	double		stream3_phasestep;
	double		stream4_phase;
	double		stream4_phasestep;
	
	/* Extended RadioText (UTF-8, 128 bytes) */
	char		ert[129];
	int		ert_segment;
	
	/* Group Type C data buffers */
	uint8_t		group_c_buffer[13];
	int		group_c_bit_pos;
} rds2_encoder_t;

int rds2_encoder_init(rds2_encoder_t *rds2, double samplerate)
{
	memset(rds2, 0, sizeof(*rds2));
	
	/* Initialize oscillators for non-harmonic frequencies */
	rds2->stream2_phasestep = 2.0 * M_PI * RDS2_STREAM2_FREQ / samplerate;
	rds2->stream3_phasestep = 2.0 * M_PI * RDS2_STREAM3_FREQ / samplerate;
	rds2->stream4_phasestep = 2.0 * M_PI * RDS2_STREAM4_FREQ / samplerate;
	
	return 0;
}

void rds2_encoder_process(rds2_encoder_t *rds2, sample_t *samples, int num,
			  double pilot_phase, double pilot_phasestep)
{
	/* 
	 * For each sample:
	 * 1. Generate BPSK for stream 2 at 66.5 kHz (free-running)
	 * 2. Generate BPSK for stream 3 at 71.25 kHz (free-running)
	 * 3. Generate BPSK for stream 4 at 76 kHz (4 x pilot_phase)
	 * 4. Sum all three with appropriate injection levels
	 */
	(void)rds2;
	(void)samples;
	(void)num;
	(void)pilot_phase;
	(void)pilot_phasestep;
}

#endif /* RDS2 encoder stub */

/* ==================== RDS DECODER ==================== */

/* Forward Error Correction (FEC) per IEC 62106 Annex B */
/* Offset words (Table B.1) - indexed by block_idx (0=A, 1=B, 2=C, 3=D) */
static const uint16_t RDS_OFFSET_WORDS[4] = {
	0x0FC, /* A - block_idx 0 */
	0x198, /* B - block_idx 1 */
	0x168, /* C - block_idx 2 (also covers C' for type B groups) */
	0x1B4  /* D - block_idx 3 */
};

/* FEC Lookup Table Entry */
typedef struct {
	uint16_t syndrome;
	uint32_t error_vector;
} rds_fec_entry_t;

/* Pre-computed FEC lookup tables (one per offset) */
/* Each table contains syndromes for 1-bit and 2-bit burst errors */
#define FEC_TABLE_SIZE 52  /* 26 * 2 patterns (1-bit + 2-bit) */
static rds_fec_entry_t rds_fec_table[4][FEC_TABLE_SIZE];
static int rds_fec_table_initialized = 0;

/* Initialize FEC lookup tables */
static void rds_fec_init(void)
{
	if (rds_fec_table_initialized) return;
	
	for (int offset_idx = 0; offset_idx < 4; offset_idx++) {
		int entry = 0;
		uint16_t offset_word = RDS_OFFSET_WORDS[offset_idx];
		
		/* 1-bit errors */
		for (int shift = 0; shift < 26; shift++) {
			uint32_t error_vector = (1U << shift);
			uint32_t test_block = error_vector ^ offset_word;
			uint16_t syndrome = rds_syndrome_calc(test_block);
			
			rds_fec_table[offset_idx][entry].syndrome = syndrome;
			rds_fec_table[offset_idx][entry].error_vector = error_vector;
			entry++;
		}
		
		/* 2-bit burst errors (adjacent bits) */
		for (int shift = 0; shift < 26; shift++) {
			uint32_t error_vector = (3U << shift) & 0x3FFFFFF;
			uint32_t test_block = error_vector ^ offset_word;
			uint16_t syndrome = rds_syndrome_calc(test_block);
			
			rds_fec_table[offset_idx][entry].syndrome = syndrome;
			rds_fec_table[offset_idx][entry].error_vector = error_vector;
			entry++;
		}
	}
	
	rds_fec_table_initialized = 1;
	LOGP(DRADIO, LOGL_DEBUG, "RDS FEC: Lookup tables initialized\n");
}

/* Try to correct 1-2 bit burst errors in a block */
/* Returns 1 if correction succeeded, 0 otherwise */
static int rds_correct_errors(uint32_t block, int expected_offset, uint32_t *corrected_out)
{
	if (!rds_fec_table_initialized) rds_fec_init();
	
	uint16_t syndrome = rds_syndrome_calc(block);
	
	/* Search for matching syndrome in the table for expected offset */
	for (int i = 0; i < FEC_TABLE_SIZE; i++) {
		if (rds_fec_table[expected_offset][i].syndrome == syndrome) {
			/* Found! XOR with error vector to correct */
			*corrected_out = block ^ rds_fec_table[expected_offset][i].error_vector;
			return 1;
		}
	}
	
	return 0; /* No correction found */
}


/* Check syndrome for block offset detection */
static int rds_check_syndrome(uint32_t block, uint16_t *offset_out)
{
	uint16_t syndrome = rds_syndrome_calc(block);
	
	/* Check against known syndromes */
	if (syndrome == RDS_SYNDROME_A) { *offset_out = 0; return 1; }
	if (syndrome == RDS_SYNDROME_B) { *offset_out = 1; return 1; }
	if (syndrome == RDS_SYNDROME_C) { *offset_out = 2; return 1; }
	if (syndrome == RDS_SYNDROME_Cp) { *offset_out = 2; return 1; } /* C' */
	if (syndrome == RDS_SYNDROME_D) { *offset_out = 3; return 1; }
	
	return 0; /* No valid offset */
}

/* ============================================================
 * RT+ (RadioText Plus) Decoder
 * IEC 62106-6: Content-type tagging for RadioText
 * ============================================================ */

/* Decode RT+ ODA group
 * Extracts content-type tags for RadioText or eRT
 */
static void rds_decode_rtplus(rds_decoder_t *rds, const uint16_t *blocks,
                               const uint8_t *status, rds_rtplus_decoder_t *rtplus,
                               const rds_oda_app_t *oda)
{
	uint16_t b2 = blocks[1];
	uint16_t b3 = blocks[2];
	uint16_t b4 = blocks[3];
	
	/* Extract toggle and item_running from Block B */
	uint8_t toggle = (b2 & RDS_RTPLUS_TOGGLE_MASK) >> RDS_RTPLUS_TOGGLE_BIT;
	uint8_t item_running = (b2 & RDS_RTPLUS_ITEM_RUNNING_MASK) >> RDS_RTPLUS_ITEM_RUNNING_BIT;
	
	/* Update registration info from 3A message if available */
	if (oda) {
		rtplus->cb = (oda->message & RDS_ERT_3A_CB_MASK) >> RDS_ERT_3A_CB_BIT;
		rtplus->scb = (oda->message & RDS_ERT_3A_SCB_MASK) >> RDS_ERT_3A_SCB_SHIFT;
		rtplus->template_num = oda->message & RDS_ERT_3A_TEMPLATE_MASK;
		rtplus->carrier_group = oda->carrier_group;
		rtplus->registered = 1;
	}
	
	/* Check if toggle/running changed - clear tags if so */
	if (toggle != rtplus->toggle || item_running != rtplus->item_running) {
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS RT+: Toggle/running changed "
			     "(toggle=%d->%d running=%d->%d), clearing tags\n",
			     rtplus->toggle, toggle, rtplus->item_running, item_running);
		rtplus->tag_count = 0;
		rtplus->toggle = toggle;
		rtplus->item_running = item_running;
	}
	
	rtplus->tag_count = 0;
	
	/* Extract tag1 if Block C is valid */
	if (status[2] <= RDS_STATUS_CORRECTED) {
		rds_rtplus_tag_t *tag1 = &rtplus->tags[0];
		
		/* Tag1 content_type: 6 bits split across B2[2:0] and B3[15:13] */
		uint8_t ct_high = (b2 & RDS_RTPLUS_TAG1_CT_HIGH_MASK) >> RDS_RTPLUS_TAG1_CT_HIGH_BITS;
		uint8_t ct_low = (b3 & RDS_RTPLUS_TAG1_CT_LOW_MASK) >> RDS_RTPLUS_TAG1_CT_LOW_SHIFT;
		tag1->content_type = (ct_high << RDS_RTPLUS_TAG1_CT_HIGH_SHIFT) | ct_low;
		
		/* Tag1 start: 6 bits from B3[12:7] */
		tag1->start = (b3 & RDS_RTPLUS_TAG1_START_MASK) >> RDS_RTPLUS_TAG1_START_SHIFT;
		
		/* Tag1 length: 6 bits from B3[6:1], stored as length-1 */
		tag1->length = ((b3 & RDS_RTPLUS_TAG1_LEN_MASK) >> RDS_RTPLUS_TAG1_LEN_SHIFT) + 1;
		
		rtplus->tag_count = 1;
		
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS RT+: Tag1 type=%d start=%d len=%d\n",
			     tag1->content_type, tag1->start, tag1->length);
		
		/* Extract tag2 if Block D is valid */
		if (status[3] <= RDS_STATUS_CORRECTED) {
			rds_rtplus_tag_t *tag2 = &rtplus->tags[1];
			
			/* Tag2 content_type: 6 bits extracted from combined B3+B4 starting at bit 11
			 * redsea: getBits<6>(B3, B4, 11) extracts 6 bits from combined (B3 << 16) | B4
			 * Combined value: bits 31-16 = B3, bits 15-0 = B4
			 * Bit 11 of combined = bit 11 of B3 (since B3 is upper 16 bits)
			 * So we extract: B3[11:15] (5 bits) + B4[0] (1 bit) = 6 bits total
			 * 
			 * However, the bit layout shows:
			 * - B3[0] = tag2 content_type[5] (1 bit from constants)
			 * - B4[15:12] = tag2 content_type[4:1] (4 bits from constants)
			 * - We need 6 bits total, so B4[11] might be bit 0, but that's in start field
			 * 
			 * Using redsea's exact method: extract 6 bits starting at offset 11
			 * of the combined 32-bit value (B3 << 16) | B4 */
			uint32_t combined = ((uint32_t)b3 << 16) | b4;
			tag2->content_type = (combined >> 11) & 0x3F;  /* 6 bits starting at bit 11 */
			
			/* Tag2 start: 6 bits from B4[11:6] */
			tag2->start = (b4 & RDS_RTPLUS_TAG2_START_MASK) >> RDS_RTPLUS_TAG2_START_SHIFT;
			
			/* Tag2 length: 5 bits from B4[5:1], stored as length-1 */
			tag2->length = ((b4 & RDS_RTPLUS_TAG2_LEN_MASK) >> RDS_RTPLUS_TAG2_LEN_SHIFT) + 1;
			
			rtplus->tag_count = 2;
			
			if (rds->debug)
				LOGP(DRADIO, LOGL_DEBUG, "RDS RT+: Tag2 type=%d start=%d len=%d\n",
				     tag2->content_type, tag2->start, tag2->length);
		}
	}
	
	rtplus->timestamp = time(NULL);
	
	if (rds->verbose && rtplus->tag_count > 0) {
		const char *oda_name = (oda && oda->aid == RDS_ODA_AID_ERT_PLUS) ? "eRT+" : "RT+";
		LOGP(DRADIO, LOGL_INFO, "RDS %s: toggle=%d running=%d tags=%d\n",
		     oda_name, rtplus->toggle, rtplus->item_running, rtplus->tag_count);
		for (int i = 0; i < rtplus->tag_count; i++) {
			const char *ct_name = rds_get_rtplus_content_type(rtplus->tags[i].content_type);
			LOGP(DRADIO, LOGL_INFO, "  Tag%d: type=%d (%s) start=%d len=%d\n",
			     i + 1, rtplus->tags[i].content_type, ct_name ? ct_name : "Unknown",
			     rtplus->tags[i].start, rtplus->tags[i].length);
		}
	}
}

/* ============================================================
 * eRT (Enhanced RadioText) Decoder
 * RDS2 / IEC 62106-2: 128-byte RadioText with UTF-8/UCS-2
 * ============================================================ */

/* Decode eRT ODA group
 * Decodes 128-byte eRT text segments (32 segments × 4 bytes)
 */
static void rds_decode_ert(rds_decoder_t *rds, const uint16_t *blocks,
                            const uint8_t *status, const rds_oda_app_t *oda)
{
	rds_ert_decoder_t *ert = &rds->ert_dec;
	uint16_t b2 = blocks[1];
	uint16_t b3 = blocks[2];
	uint16_t b4 = blocks[3];
	
	/* Update registration info from 3A message if available */
	if (oda) {
		ert->encoding = (oda->message & RDS_ERT_3A_ENCODING_MASK) >> RDS_ERT_3A_ENCODING_BIT;
		ert->direction = (oda->message & RDS_ERT_3A_DIRECTION_MASK) >> RDS_ERT_3A_DIRECTION_BIT;
		ert->use_chartable_e3 = ((oda->message & RDS_ERT_3A_CHARTABLE_MASK) >> RDS_ERT_3A_CHARTABLE_SHIFT) == 0;
		ert->carrier_group = oda->carrier_group;
		ert->registered = 1;
	}
	
	/* Extract segment address (0-31) from Block B */
	int seg = b2 & RDS_ERT_SEGMENT_MASK;
	int pos = seg * RDS_ERT_BYTES_PER_SEGMENT;
	
	if (pos >= RDS_ERT_LENGTH) {
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS eRT: Invalid segment %d (max %d)\n",
			     seg, RDS_ERT_SEGMENTS - 1);
		return;
	}
	
	/* Update eRT bytes if blocks are valid */
	if (status[2] <= RDS_STATUS_CORRECTED) {
		ert->ert[pos] = (b3 >> 8) & 0xFF;
		ert->ert[pos + 1] = b3 & 0xFF;
		ert->ert_status[pos] = status[2];
		ert->ert_status[pos + 1] = status[2];
		ert->segments_received |= (1U << seg);
	}
	
	if (status[3] <= RDS_STATUS_CORRECTED && pos + 2 < RDS_ERT_LENGTH) {
		ert->ert[pos + 2] = (b4 >> 8) & 0xFF;
		ert->ert[pos + 3] = b4 & 0xFF;
		ert->ert_status[pos + 2] = status[3];
		ert->ert_status[pos + 3] = status[3];
	}
	
	/* Add NUL terminator */
	ert->ert[RDS_ERT_LENGTH] = '\0';
	ert->timestamp = time(NULL);
	
	if (rds->debug)
		LOGP(DRADIO, LOGL_DEBUG, "RDS eRT: seg=%d pos=%d-%d encoding=%s direction=%s\n",
		     seg, pos, pos + 3,
		     ert->encoding == RDS_ERT_ENCODING_UTF8 ? "UTF-8" : "UCS-2",
		     ert->direction == RDS_ERT_DIR_RTL ? "RTL" : "LTR");
	
	/* Check if all segments received */
	int segments_count = 0;
	for (int i = 0; i < RDS_ERT_SEGMENTS; i++) {
		if (ert->segments_received & (1U << i))
			segments_count++;
	}
	
	if (rds->verbose) {
		char ert_display[513];  /* 128 bytes * 4 bytes UTF-8 max + 1 */
		int display_len = 0;
		
		/* Convert eRT to display-safe UTF-8 */
		if (ert->encoding == RDS_ERT_ENCODING_UTF8) {
			/* UTF-8: use as-is, but convert control chars */
			display_len = rds_text_to_display(ert->ert, RDS_ERT_LENGTH, ert_display, sizeof(ert_display));
		} else {
			/* UCS-2: convert 16-bit Unicode to UTF-8 (simplified - just show bytes for now) */
			/* TODO: Implement proper UCS-2 to UTF-8 conversion */
			display_len = rds_text_to_display(ert->ert, RDS_ERT_LENGTH, ert_display, sizeof(ert_display));
		}
		
		LOGP(DRADIO, LOGL_INFO, "RDS eRT: seg=%d/%d encoding=%s direction=%s "
		     "text=\"%.*s\"\n",
		     segments_count, RDS_ERT_SEGMENTS,
		     ert->encoding == RDS_ERT_ENCODING_UTF8 ? "UTF-8" : "UCS-2",
		     ert->direction == RDS_ERT_DIR_RTL ? "RTL" : "LTR",
		     display_len, ert_display);
	}
}

/* Decode a complete group */
static void rds_decode_group(rds_decoder_t *rds)
{
	uint16_t pi = rds->blocks[0];
	uint16_t b2 = rds->blocks[1];
	uint16_t b3 = rds->blocks[2];
	uint16_t b4 = rds->blocks[3];
	
	/* Use table-driven decoder for field extraction and logging (NMT-style)
	 * This populates a frame structure using field definition tables */
	rds_frame_t frame;
	rds_group_decode(rds->blocks, rds->block_status, &frame, rds->debug, 0);
	
	/* Extract common fields from frame (table-driven extraction) */
	int group_type = (frame.group_type >> 1) & 0x0F;
	int version = frame.group_type & 1;
	int tp = frame.tp;
	int pty = frame.pty;
	
	/* PI comes from Block A - only update if reliably decoded
	 * Keep last known good PI for AF processing */
	if ((rds->group_mask & (1<<0)) && rds->block_status[0] <= 1) {
		rds->pi = frame.pi;
		rds->pi_status = rds->block_status[0];
	}
	
	/* PTY and TP come from Block B */
	if (rds->group_mask & (1<<1)) {
		rds->tp = tp;
		rds->tp_status = rds->block_status[1];
		rds->pty = pty;
		rds->pty_status = rds->block_status[1];
	}
	
	/* REQUIRE Valid Block B for Group/Segment info */
	if (!(rds->group_mask & (1<<1))) return;
	
	if (group_type == 0) {
		/* Group 0A/0B - see RDS_0A_* macros in rds.h */
		int ta = (b2 & RDS_0A_TA_MASK) >> RDS_0A_TA_BIT;
		int ms = (b2 & RDS_0A_MS_MASK) >> RDS_0A_MS_BIT;
		int di = (b2 & RDS_0A_DI_MASK) >> RDS_0A_DI_BIT;
		int seg = b2 & RDS_0A_SEG_MASK;
		rds->ta = ta;
		rds->ta_status = rds->block_status[1];
		rds->ms = ms;
		rds->ms_status = rds->block_status[1];
		
		/* DI flags are transmitted per segment.
		 * IEC 62106 Table 9: seg 0→d3, seg 1→d2, seg 2→d1, seg 3→d0
		 * IEC 62106 Table 10: d0=stereo, d1=artificial head, d2=compressed, d3=dynamic PTY
		 * Combined: seg 0→dynamic PTY, seg 1→compressed, seg 2→artificial head, seg 3→stereo */
		switch (seg) {
		case 0: rds->di_dynamic_pty = di; break;
		case 1: rds->di_compressed = di; break;
		case 2: rds->di_artificial_head = di; break;
		case 3: rds->di_stereo = di; break;
		}
		rds->di_status = rds->block_status[1];
		
		/* ============================================================
		 * AF Method A Decode (NEW - dedicated structure with PI tracking)
		 * ============================================================
		 * PI-change detection: clear AF list when station changes.
		 * Only update slots when Block C is valid/corrected.
		 * ============================================================ */
		if (version == 0 && (rds->group_mask & (1<<2))) {
			rds_af_method_a_dec_t *afd = &rds->af_method_a_dec;
			
			/* PI-change detection: clear list if PI changed */
			if (rds->pi != afd->pi) {
				if (afd->pi != 0 && rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 0A: PI changed %04X -> %04X, clearing AF list\n",
					     afd->pi, rds->pi);
				memset(afd, 0, sizeof(*afd));
				afd->pi = rds->pi;
			}
			
			/* Only process if Block C was decoded OK or corrected */
			uint8_t block_status = rds->block_status[2];
			if (block_status <= RDS_STATUS_CORRECTED && rds->pi != 0) {
				uint8_t codes[2];
				codes[0] = (b3 >> 8) & 0xFF;
				codes[1] = b3 & 0xFF;
				
				int start_idx = 0;
				
				/* Check for Count Code (Start of new list) */
				if (codes[0] >= 224 && codes[0] <= 249) {
					afd->expected_count = codes[0] - 224;
					afd->received_count = 0;
					afd->complete = 0;
					afd->lf_mf_follows = 0;
					memset(afd->freq, 0, sizeof(afd->freq));
					memset(afd->type, 0, sizeof(afd->type));
					memset(afd->status, RDS_STATUS_NONE, sizeof(afd->status));
					
					/* Also init collector for Method B detection */
					rds_af_collector_t *col = &rds->af_collector;
					col->pi = rds->pi;
					col->expected_count = codes[0] - 224;
					col->received_pairs = 1;
					col->tuning_freq = RDS_AF_FM_BASE + codes[1];
					col->header_received = 1;
					col->codes[0] = codes[0];
					col->codes[1] = codes[1];
					col->status[0] = block_status;
					col->status[1] = block_status;
					
					/* Log count code - special case for 224 (no AF) */
				if (afd->expected_count == 0) {
					if (rds->debug)
						LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: AF code 224 (No AF exists)\n");
					if (rds->verbose)
						LOGP(DRADIO, LOGL_INFO, "RDS 0A: No AF (count 0)\n");
				} else if (rds->debug) {
					LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: AF list start, expecting %d frequencies\n",
					     afd->expected_count);
				}
					
					/* Count code consumes first byte */
					start_idx = 1;
				} else if (rds->af_collector.header_received) {
					/* Store pair in collector for Method B analysis */
					rds_af_collector_t *col = &rds->af_collector;
					if (col->received_pairs < 26) {
						int idx = col->received_pairs * 2;
						col->codes[idx] = codes[0];
						col->codes[idx + 1] = codes[1];
						col->status[idx] = block_status;
						col->status[idx + 1] = block_status;
						col->received_pairs++;
					}
				}
				
				/* Process 1 or 2 items based on start_idx */
				int is_us = RDS_IS_RBDS(rds->ecc);
				
				for (int i = start_idx; i < 2; i++) {
					uint8_t code = codes[i];
					
					if (afd->lf_mf_follows) {
						/* Previous byte was 250 -> this is LF/MF code */
						uint16_t freq_khz = 0;
						rds_af_freq_type_t ftype = RDS_AF_FREQ_MF;
						
						if (code >= 1 && code <= 15) {
							/* LF: 153-279 kHz */
							freq_khz = 144 + code * 9;
							ftype = RDS_AF_FREQ_LF;
						} else if (code >= 16 && code <= 135) {
							/* MF: 531-1602 kHz (RDS) or 540-1700 kHz (RBDS) */
							if (is_us)
								freq_khz = 540 + (code - 16) * 10;
							else
								freq_khz = 531 + (code - 16) * 9;
							ftype = RDS_AF_FREQ_MF;
						}
						
						if (freq_khz && afd->received_count < RDS_AF_MAX_METHOD_A && 
						    afd->received_count < afd->expected_count) {
							afd->freq[afd->received_count] = freq_khz;
							afd->type[afd->received_count] = ftype;
							afd->status[afd->received_count] = block_status;
							afd->received_count++;
							
							if (rds->verbose)
								LOGP(DRADIO, LOGL_INFO, "RDS 0A: %s AF %d kHz\n",
								     ftype == RDS_AF_FREQ_LF ? "LF" : "MF", freq_khz);
						}
						afd->lf_mf_follows = 0;
						
					} else if (code == RDS_AF_LF_MF_FOLLOWS) {
						/* Marker (250) -> next code is LF/MF */
						afd->lf_mf_follows = 1;
						
					} else if (RDS_VALID_AF_CODE(code)) {
						/* VHF Frequency (1-204) */
						if (afd->received_count < RDS_AF_MAX_METHOD_A && 
						    afd->received_count < afd->expected_count) {
							afd->freq[afd->received_count] = RDS_AF_FM_BASE + code;
							afd->type[afd->received_count] = RDS_AF_FREQ_VHF;
							afd->status[afd->received_count] = block_status;
							afd->received_count++;
						}
					} else {
						/* Log reserved/invalid AF codes per IEC 62106 Table 11 */
						if (code == 0) {
							if (rds->verbose)
								LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF code 0 (not to be used)\n");
						} else if (code >= RDS_AF_RESERVED1_MIN && code <= RDS_AF_RESERVED1_MAX) {
							if (rds->verbose)
								LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF code %d (reserved 206-223)\n", code);
						} else if (code >= RDS_AF_RESERVED2_MIN) {
							if (rds->verbose)
								LOGP(DRADIO, LOGL_INFO, "RDS 0A: AF code %d (reserved 251-255)\n", code);
						}
					}
				}
				
				/* Check completion */
				if (afd->received_count >= afd->expected_count && afd->expected_count > 0) {
					/* Analyze collected data for Method B pattern */
					rds_af_decode_complete(rds);
				}
			}
		}
		

		
		/* DEBUG: Compact codes */
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 0%c: PI=%04X PTY=%d TP=%d TA=%d M/S=%c DI[%d]=%d seg=%d\n",
			     version ? 'B' : 'A', pi, pty, tp, ta, ms ? 'M' : 'S', seg, di, seg);
		
		/* Only update PS if block D is valid (in group_mask) */
		if (rds->group_mask & (1<<3)) {
			rds->ps[seg*2] = (b4 >> 8) & 0xFF;
			rds->ps[seg*2+1] = b4 & 0xFF;
			rds->ps_status[seg*2] = rds->block_status[3];
			rds->ps_status[seg*2+1] = rds->block_status[3];
			rds->ps_segments |= (1 << seg);
			
			/* Verbose: Log Group 0 basic tuning info and PS segment
			 * DI segment mapping: seg 0->d3, seg 1->d2, seg 2->d1, seg 3->d0 */
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 0%c: PTY=%d (%s), TP=%d, TA=%d (%s), "
				     "M/S=%s, %s=%s, PS[%d]=\"%c%c\"\n",
				     version ? 'B' : 'A', pty, rds_get_pty_name(pty, RDS_IS_RBDS(rds->ecc)),
				     tp, ta, rds_get_tp_ta_description(tp, ta),
				     rds_get_ms_name(ms),
				     rds_get_di_name(3 - seg), rds_get_di_value_name(3 - seg, di),
				     seg,
				     (rds->ps[seg*2] >= 0x20 && rds->ps[seg*2] <= 0x7E) ? rds->ps[seg*2] : '.',
				     (rds->ps[seg*2+1] >= 0x20 && rds->ps[seg*2+1] <= 0x7E) ? rds->ps[seg*2+1] : '.');
		}
		
		if (rds->ps_segments == 0x0F) {
			rds->ps[8] = '\0';
			
			/* Log complete DI summary once we have all segments
			 * DI flag names from rds_tables.h: d0=dynamic_pty, d1=compressed, 
			 * d2=artificial_head, d3=stereo (mapped to segments 3,2,1,0) */
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 0%c: Decoder Info: %s=%s, %s=%s, %s=%s, %s=%s\n",
				     version ? 'B' : 'A',
				     rds_get_di_name(3), rds_get_di_value_name(3, rds->di_stereo),
				     rds_get_di_name(2), rds_get_di_value_name(2, rds->di_artificial_head),
				     rds_get_di_name(1), rds_get_di_value_name(1, rds->di_compressed),
				     rds_get_di_name(0), rds_get_di_value_name(0, rds->di_dynamic_pty));
		}
	}
	else if (group_type == 1) {
		/* Group 1A/1B: Programme Item Number and slow labeling codes
		 * See RDS_1A_* and RDS_PIN_* macros in rds.h for bit field definitions
		 * Block B bits 4-0: Radio Paging (1A, deprecated) or Spare (1B)
		 */
		int b2_payload = b2 & RDS_B2_PAYLOAD_MASK;  /* Block B bits 4-0 */
		
		/* Group 1A: Radio Paging codes (deprecated, usually 0)
		 * Group 1B: Spare bits (should be 0) */
		if (version == 0) {
			/* 1A: Radio Paging (deprecated) */
			if (b2_payload != 0) {
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: Radio Paging (B2[4:0])=%d (deprecated)\n",
					     b2_payload);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: Radio Paging codes=%d (deprecated, bits 4-0)\n",
					     b2_payload);
			}
		} else {
			/* 1B: Spare bits (should be 0) */
			if (b2_payload != 0) {
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1B: Spare bits (B2[4:0])=%d (expected 0)\n",
					     b2_payload);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1B: Spare bits=%d (expected 0, bits 4-0)\n",
					     b2_payload);
			}
		}
		
		/* Group 1B: Block C contains PI repeat (for fast station ID) */
		if (version == 1 && (rds->group_mask & (1<<2))) {
			uint16_t pi_repeat = b3;
			/* Validate PI repeat against Block A PI */
			if (rds->group_mask & (1<<0)) {
				if (pi_repeat != pi && rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1B: PI mismatch A=%04X C=%04X\n",
					     pi, pi_repeat);
			}
			/* Use PI repeat if Block A was missing/bad */
			if (!(rds->group_mask & (1<<0)) || rds->block_status[0] > RDS_STATUS_CORRECTED) {
				if (rds->block_status[2] <= RDS_STATUS_CORRECTED) {
					rds->pi = pi_repeat;
					rds->pi_status = rds->block_status[2];
					if (rds->debug)
						LOGP(DRADIO, LOGL_DEBUG, "RDS 1B: Using PI from Block C: %04X\n",
						     pi_repeat);
				}
			}
		}
	/* Decode PIN from Block D (both 1A and 1B) */
		if (rds->group_mask & (1<<3)) {
			rds->pin = b4;
			rds->pin_status = rds->block_status[3];
			rds->pin_day = (b4 & RDS_PIN_DAY_MASK) >> RDS_PIN_DAY_SHIFT;
			rds->pin_hour = (b4 & RDS_PIN_HOUR_MASK) >> RDS_PIN_HOUR_SHIFT;
			rds->pin_minute = b4 & RDS_PIN_MINUTE_MASK;
			
			/* Validate PIN fields (RDS_VALID_* macros) */
			int pin_valid = RDS_VALID_PIN_DAY(rds->pin_day) &&
			                RDS_VALID_HOUR(rds->pin_hour) &&
			                RDS_VALID_MINUTE(rds->pin_minute);
			
			if (rds->debug)
				LOGP(DRADIO, LOGL_DEBUG, "RDS 1%c: PIN=0x%04X day=%d hour=%d min=%d%s\n",
				     version ? 'B' : 'A', rds->pin, rds->pin_day, rds->pin_hour, rds->pin_minute,
				     pin_valid ? "" : " [INVALID]");
			
			if (rds->verbose && pin_valid) {
				if (rds->pin_day != 0) {
					LOGP(DRADIO, LOGL_INFO, "RDS 1%c: Programme Item Number - "
					     "Scheduled broadcast: Day %d of month, %02d:%02d\n",
					     version ? 'B' : 'A', rds->pin_day, rds->pin_hour, rds->pin_minute);
				} else {
					LOGP(DRADIO, LOGL_INFO, "RDS 1%c: Programme Item Number - "
					     "No scheduled broadcast time (day=0)\n",
					     version ? 'B' : 'A');
				}
			}
		}
		
		/* Group 1A: Slow Labeling Codes in Block C */
		if (version == 0 && (rds->group_mask & (1<<2))) {
			rds->linkage_actuator = (b3 & RDS_1A_LA_MASK) >> RDS_1A_LA_BIT;
			int variant = (b3 & RDS_1A_VARIANT_MASK) >> RDS_1A_VARIANT_SHIFT;
			int payload = b3 & RDS_1A_PAYLOAD_MASK;
			uint8_t cc = (pi >> 12) & 0x0F;  /* Country code from PI */
			
		switch (variant) {
			case RDS_1A_VARIANT_ECC:
				/* Variant 0: Extended Country Code + Paging (IEC 62106 Table 9)
				 * Bits 11-8: Paging codes (deprecated, usually 0)
				 * Bits 7-0:  Extended Country Code */
				rds->ecc = payload & 0xFF;
				rds->ecc_status = rds->block_status[2];
				{
					int paging = (payload >> 8) & 0x0F;
					/* Normal operation: paging=0 (deprecated), only log ECC */
					if (paging != 0) {
						/* Non-zero paging - log in both debug and verbose */
						if (rds->debug)
							LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: ECC=0x%02X Paging=%d (deprecated) CC=%d LA=%d\n",
							     rds->ecc, paging, cc, rds->linkage_actuator);
						if (rds->verbose)
							LOGP(DRADIO, LOGL_INFO, "RDS 1A: ECC=0x%02X (%s), "
							     "Paging=%d (deprecated), LA=%d. "
							     "Country=\"%s\", Linkage=%s\n",
							     rds->ecc, rds_get_ecc_name(rds->ecc),
							     paging, rds->linkage_actuator,
							     rds_get_country_code(cc, rds->ecc),
							     rds->linkage_actuator ? "Yes" : "No");
					} else {
						/* Normal case: paging=0, just log ECC */
						if (rds->debug)
							LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: ECC=0x%02X CC=%d LA=%d\n",
							     rds->ecc, cc, rds->linkage_actuator);
						if (rds->verbose)
							LOGP(DRADIO, LOGL_INFO, "RDS 1A: ECC=0x%02X (%s), LA=%d. "
							     "Country=\"%s\", Linkage=%s\n",
							     rds->ecc, rds_get_ecc_name(rds->ecc),
							     rds->linkage_actuator,
							     rds_get_country_code(cc, rds->ecc),
							     rds->linkage_actuator ? "Yes" : "No");
					}
				}
				break;
			case RDS_1A_VARIANT_LANGUAGE:
				/* Language code */
				rds->language_code = payload & 0xFF;
				rds->language_status = rds->block_status[2];
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: Language=%d LA=%d\n",
					     rds->language_code, rds->linkage_actuator);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: Language Code=%d, "
					     "Linkage Actuator=%d. "
					     "Summary: Language=\"%s\", Programme Linkage=%s\n",
					     rds->language_code, rds->linkage_actuator,
					     rds_get_language_name(rds->language_code),
					     rds->linkage_actuator ? "Yes" : "No");
				break;
				case RDS_1A_VARIANT_TMC_ID:
				/* TMC identification (IEC 62106 Table 9)
				 * Bits 11-0: TMC ID for traffic message channel
				 * Used with Group 8A for TMC data */
				rds->tmc_id = payload & RDS_1A_PAYLOAD_MASK;
				rds->tmc_id_status = rds->block_status[2];
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: TMC ID=0x%03X LA=%d\n",
					     rds->tmc_id, rds->linkage_actuator);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: TMC ID=0x%03X (%d), "
					     "Linkage Actuator=%d\n",
					     rds->tmc_id, rds->tmc_id, rds->linkage_actuator);
				break;
			case RDS_1A_VARIANT_PAGER:
				/* Paging ID (deprecated, IEC 62106 Table 9) */
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: Paging ID (deprecated), LA=%d, "
					     "data=0x%03X\n", rds->linkage_actuator,
					     payload & RDS_1A_PAYLOAD_MASK);
				break;
			case RDS_1A_VARIANT_BCAST:
				/* Broadcaster use (IEC 62106 Table 9)
				 * Bits 11-0: Broadcaster-defined data */
				rds->slc_broadcaster = payload & RDS_1A_PAYLOAD_MASK;
				rds->slc_broadcaster_status = rds->block_status[2];
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: Broadcaster=0x%03X LA=%d\n",
					     rds->slc_broadcaster, rds->linkage_actuator);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: Broadcaster data=0x%03X, "
					     "Linkage Actuator=%d\n",
					     rds->slc_broadcaster, rds->linkage_actuator);
				break;
			case RDS_1A_VARIANT_EWS:
				/* EWS channel ID (IEC 62106 Table 9)
				 * Bits 11-0: Emergency Warning System channel number */
				rds->ews_channel = payload & RDS_1A_PAYLOAD_MASK;
				rds->ews_channel_status = rds->block_status[2];
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: EWS channel=%d LA=%d\n",
					     rds->ews_channel, rds->linkage_actuator);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: EWS Channel ID=%d, "
					     "Linkage Actuator=%d\n",
					     rds->ews_channel, rds->linkage_actuator);
				break;
			default:
				/* Variants 4-5: Not assigned (IEC 62106 Table 9) */
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: Unassigned variant %d, LA=%d, "
					     "data=0x%03X\n", variant, rds->linkage_actuator,
					     payload & RDS_1A_PAYLOAD_MASK);
				break;
			}

		}
	}
	else if (group_type == 2) {
		/* Group 2A/2B: RadioText (IEC 62106 S6.1.5.3)
		 * 
		 * EN 50067: "A mixture of type 2A and type 2B groups must not be used
		 * when transmitting any one given message."
		 * 
		 * We maintain separate buffers for 2A (64 chars) and 2B (32 chars).
		 * The A/B flag is tracked independently for each version.
		 * The display buffer (rt[]) is updated with whichever was last received.
		 */
		int ab_flag = (b2 & RDS_2A_AB_MASK) >> RDS_2A_AB_BIT;
		int seg = b2 & RDS_2A_SEG_MASK;
		
		/* DEBUG: Compact codes */
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 2%c: AB=%d seg=%d\n",
			     version ? 'B' : 'A', ab_flag, seg);
		
		if (version == 0) {
			/* Group 2A: 64 chars (4 chars per segment, 16 segments) */
			
			/* Check A/B flag change for 2A buffer */
			if (ab_flag != rds->rt_2a_ab) {
				rds->rt_2a_ab = ab_flag;
				rds->rt_2a_segments = 0;
				memset(rds->rt_2a, ' ', 64);
				rds->rt_2a[64] = '\0';
				memset(rds->rt_2a_status, RDS_STATUS_NONE, 64);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 2A: A/B flag toggled to %c "
					     "(clearing 2A buffer)\n", ab_flag ? 'B' : 'A');
			}
			
			int pos = seg * 4;
			if (pos <= 60) {
				rds->rt_2a[pos] = (b3 >> 8) & 0xFF;
				rds->rt_2a[pos+1] = b3 & 0xFF;
				rds->rt_2a[pos+2] = (b4 >> 8) & 0xFF;
				rds->rt_2a[pos+3] = b4 & 0xFF;
				rds->rt_2a_status[pos] = rds->block_status[2];
				rds->rt_2a_status[pos+1] = rds->block_status[2];
				rds->rt_2a_status[pos+2] = rds->block_status[3];
				rds->rt_2a_status[pos+3] = rds->block_status[3];
				rds->rt_2a_segments |= (1 << seg);
				
				if (rds->verbose) {
					LOGP(DRADIO, LOGL_INFO, "RDS RX 2A: seg=%d pos=%d-%d AB=%c "
					     "chars='%s%s%s%s'\n", seg, pos, pos+3, ab_flag ? 'B' : 'A',
					     rds_char_to_display((uint8_t)rds->rt_2a[pos]),
					     rds_char_to_display((uint8_t)rds->rt_2a[pos+1]),
					     rds_char_to_display((uint8_t)rds->rt_2a[pos+2]),
					     rds_char_to_display((uint8_t)rds->rt_2a[pos+3]));
				}
			}
			
			/* Update display buffer with 2A content
			 * IEC 62106: "If no change in flag, segments should be written into
			 * the existing displayed message." We follow spec literally.
			 * Version and A/B flag tracked for status display. */
			if (rds->rt_display_version == 1 && rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS: Now receiving 2A (was 2B)\n");
			memcpy(rds->rt, rds->rt_2a, 65);
			memcpy(rds->rt_status, rds->rt_2a_status, 64);
			rds->rt_ab = rds->rt_2a_ab;
			rds->rt_version = 0;
			rds->rt_display_version = 0;
			rds->rt_segments = rds->rt_2a_segments;
			
		} else {
			/* Group 2B: 32 chars (2 chars per segment, 16 segments) */
			
			/* Check A/B flag change for 2B buffer */
			if (ab_flag != rds->rt_2b_ab) {
				rds->rt_2b_ab = ab_flag;
				rds->rt_2b_segments = 0;
				memset(rds->rt_2b, ' ', 32);
				rds->rt_2b[32] = '\0';
				memset(rds->rt_2b_status, RDS_STATUS_NONE, 32);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 2B: A/B flag toggled to %c "
					     "(clearing 2B buffer)\n", ab_flag ? 'B' : 'A');
			}
			
			/* Validate PI repeat in Block C (b3) */
			if (b3 != rds->pi) {
				if (rds->verbose)
					LOGP(DRADIO, LOGL_NOTICE, "RDS 2B: PI mismatch in Block C (got %04X, expected %04X)\n",
					     b3, rds->pi);
				/* We could reject the group here, but for now we just log it */
			}
			
			int pos = seg * 2;
			if (pos <= 30) {
				rds->rt_2b[pos] = (b4 >> 8) & 0xFF;
				rds->rt_2b[pos+1] = b4 & 0xFF;
				rds->rt_2b_status[pos] = rds->block_status[3];
				rds->rt_2b_status[pos+1] = rds->block_status[3];
				rds->rt_2b_segments |= (1 << seg);
				
				if (rds->verbose) {
					LOGP(DRADIO, LOGL_INFO, "RDS RX 2B: seg=%d pos=%d-%d AB=%c "
					     "chars='%s%s'\n", seg, pos, pos+1, ab_flag ? 'B' : 'A',
					     rds_char_to_display((uint8_t)rds->rt_2b[pos]),
					     rds_char_to_display((uint8_t)rds->rt_2b[pos+1]));
				}
			}
			
			/* Update display buffer with 2B content (padded to 64 for display)
			 * IEC 62106: "If no change in flag, segments should be written into
			 * the existing displayed message." We follow spec literally.
			 * Version and A/B flag tracked for status display. */
			if (rds->rt_display_version == 0 && rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS: Now receiving 2B (was 2A)\n");
			memset(rds->rt, ' ', 64);
			memcpy(rds->rt, rds->rt_2b, 32);
			rds->rt[64] = '\0';
			memset(rds->rt_status, RDS_STATUS_NONE, 64);
			memcpy(rds->rt_status, rds->rt_2b_status, 32);
			rds->rt_ab = rds->rt_2b_ab;
			rds->rt_version = 1;
			rds->rt_display_version = 1;
			rds->rt_segments = rds->rt_2b_segments;
		}
	}
	else if (group_type == 4 && version == 0) {
		/* Group 4A: Clock-Time and Date (IEC 62106 S6.1.5.4)
		 * See RDS_4A_* macros in rds.h for bit field definitions
		 * Requires all 4 blocks for complete CT decode */
		
		if ((rds->group_mask & 0x0F) == 0x0F) {
			/*
			 * Extract MJD (17 bits spanning Block B and Block C)
			 *
			 * Modified Julian Date is a compact day number used in RDS
			 * to represent calendar dates without complex encoding.
			 * MJD 15079 = January 1, 1900; current dates are ~60000+.
			 */
			uint32_t mjd = ((b2 & RDS_4A_MJD_B2_MASK) << 15) |
			               ((b3 & RDS_4A_MJD_B3_MASK) >> RDS_4A_MJD_B3_SHIFT);
			
			/* Extract hour (5 bits spanning B3 and B4) */
			int hour = ((b3 & 0x0001) << 4) |
			           ((b4 & RDS_4A_HOUR_B4_MASK) >> RDS_4A_HOUR_B4_SHIFT);
			
			/* Extract minute (6 bits in B4) */
			int minute = (b4 & RDS_4A_MINUTE_MASK) >> RDS_4A_MINUTE_SHIFT;
			
			/* Extract timezone offset */
			int tz_sign = (b4 & RDS_4A_TZ_SIGN_MASK) ? -1 : 1;
			int tz_offset = (b4 & RDS_4A_TZ_OFFSET_MASK) * tz_sign;
			
			/* Validate before storing (use RDS_VALID_* macros) */
			if (RDS_VALID_MJD(mjd) && RDS_VALID_HOUR(hour) && RDS_VALID_MINUTE(minute)) {
				rds->ct_mjd = mjd;
				rds->ct_hour = hour;
				rds->ct_minute = minute;
				rds->ct_offset = tz_offset;
				rds->ct_valid = 1;
				/* CT status = worst of all 4 blocks */
				rds->ct_status = rds->block_status[0];
				if (rds->block_status[1] > rds->ct_status) rds->ct_status = rds->block_status[1];
				if (rds->block_status[2] > rds->ct_status) rds->ct_status = rds->block_status[2];
				if (rds->block_status[3] > rds->ct_status) rds->ct_status = rds->block_status[3];
				
				/* Convert MJD to Gregorian date using helper function */
				int year, month, day;
				mjd_to_date(mjd, &year, &month, &day);
				
				/* Apply timezone offset to get local time */
				int local_hour = hour + (tz_offset / 2);
				int local_minute = minute + (tz_offset % 2) * 30;
				if (local_minute >= 60) { local_minute -= 60; local_hour++; }
				if (local_minute < 0) { local_minute += 60; local_hour--; }
				if (local_hour >= 24) { local_hour -= 24; day++; }
				if (local_hour < 0) { local_hour += 24; day--; }
				
				/* DEBUG: Compact codes */
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 4A: MJD=%u hour=%d min=%d tz=%+d\n",
					     mjd, hour, minute, tz_offset);
				
				/* Verbose: Log decoded Clock-Time with UTC, offset, and local time */
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 4A: Clock-Time received. "
					     "UTC=%04d-%02d-%02d %02d:%02d, "
					     "Timezone offset=%+.1f hours. "
					     "Local time=%02d:%02d\n",
					     year, month, day, hour, minute,
					     tz_offset / 2.0,
					     local_hour, local_minute);
				
				/* Calculate delta from system time */
				struct tm tm_rx = {0};
				tm_rx.tm_year = year - 1900;
				tm_rx.tm_mon = month - 1;  /* mjd_to_date returns 1-12, tm_mon expects 0-11 */
				tm_rx.tm_mday = day;
				tm_rx.tm_hour = hour;
				tm_rx.tm_min = minute;
				tm_rx.tm_sec = 0;
				tm_rx.tm_isdst = -1;
				/* We need UTC timestamp. timegm is non-standard but common on Linux */
#if defined(__GLIBC__) || defined(__linux__)
				time_t rx_utc = timegm(&tm_rx);
				time_t sys_now = time(NULL);
				double delta = difftime(rx_utc, sys_now);
				
				LOGP(DRADIO, LOGL_INFO, "RDS 4A: Time Delta: RX (UTC) is %.0f seconds %s system time\n",
				     fabs(delta), (delta >= 0) ? "ahead of" : "behind");
				     
				/* User Requested Format: UTC, Offset, Local, Delta */
				LOGP(DRADIO, LOGL_INFO, "RDS 4A Report: UTC=%04d-%02d-%02dT%02d:%02dZ  "
				     "Offset=%+.1fh  Local=%02d:%02d  Delta=%+.0fs\n",
				     year, month, day, hour, minute, /* month is already 1-12 from mjd_to_date */
				     tz_offset / 2.0,
				     local_hour, local_minute, delta);
#endif
			} else {
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 4A: Invalid CT data "
					     "(MJD=%u hour=%d min=%d)\n", mjd, hour, minute);
			}
		}
	}
	else if (group_type == 10 && version == 0) {
		/* Group 10A: Programme Type Name (IEC 62106 S6.1.5.8)
		 * PTYN (8 chars) transmitted in Blocks C and D */
		
		int ab = (b2 >> RDS_10A_AB_FLAG_BIT) & 1;
		int seg = (b2 >> RDS_10A_SEGMENT_BIT) & 1;
		
		/* Block B defines context */
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 10A: AB=%d seg=%d\n", ab, seg);

		/* Handle A/B flag change (PTYN changed) */
		if (ab != rds->ptyn_ab) {
			rds->ptyn_ab = ab;
			rds->ptyn_segments = 0;
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 10A: PTYN Change Flag toggled "
				     "(new PTY Name starting)\n");
		}
		
		int pos = seg * 4;
		
		/* Decode Block C (Chars 1-2 or 5-6) */
		if (rds->group_mask & (1<<2)) {
			rds->ptyn[pos]   = (b3 >> 8) & 0xFF;
			rds->ptyn[pos+1] = b3 & 0xFF;
			rds->ptyn_status[pos]   = rds->block_status[2];
			rds->ptyn_status[pos+1] = rds->block_status[2];
		}
		
		/* Decode Block D (Chars 3-4 or 7-8) */
		if (rds->group_mask & (1<<3)) {
			rds->ptyn[pos+2] = (b4 >> 8) & 0xFF;
			rds->ptyn[pos+3] = b4 & 0xFF;
			rds->ptyn_status[pos+2] = rds->block_status[3];
			rds->ptyn_status[pos+3] = rds->block_status[3];
		}
		
		rds->ptyn_segments |= (1 << seg);
		
		/* Full PTYN received or updated */
		if (rds->verbose && (rds->group_mask & 0x0C)) {
			/* Sanitize for display */
			char ptyn_disp[9];
			int i;
			memcpy(ptyn_disp, rds->ptyn, 8);
			for (i=0; i<8; i++) 
				if (ptyn_disp[i] < 0x20 || ptyn_disp[i] > 0x7E) ptyn_disp[i] = ' ';
			ptyn_disp[8] = '\0';
			
			LOGP(DRADIO, LOGL_INFO, "RDS 10A: PTYN segment %d/2 received. "
			     "Current PTYN=\"%s\"\n", seg + 1, ptyn_disp);
		}
	}
	else if (group_type == 14) {
		/* Group 14A/14B: Enhanced Other Networks (EON) - IEC 62106 S6.1.5.14 */
		/* Block D contains ON-PI (PI of the Other Network) */
		uint16_t on_pi = b4;
		int tp_on = (b2 >> RDS_14A_TP_ON_BIT) & 1;  /* TP flag for ON */
		
		/* Ignore if Block D is missing */
		if (!(rds->group_mask & (1<<3)))
			goto group14_done;
		
		/* Find or create EON entry for this PI */
		rds_eon_entry_t *eon = NULL;
		int eon_idx = -1;
		
		/* Search existing entries */
		for (int i = 0; i < rds->eon_count; i++) {
			if (rds->eon[i].pi == on_pi) {
				eon = &rds->eon[i];
				eon_idx = i;
				break;
			}
		}
		
		/* Create new entry if not found and space available */
		if (!eon && rds->eon_count < RDS_EON_MAX_ENTRIES) {
			eon_idx = rds->eon_count++;
			eon = &rds->eon[eon_idx];
			memset(eon, 0, sizeof(*eon));
			eon->pi = on_pi;
			memset(eon->ps, ' ', 8);
			eon->ps[8] = '\0';
		}
		
		if (!eon) {
			if (rds->debug)
				LOGP(DRADIO, LOGL_DEBUG, "RDS 14A: EON database full, ignoring PI=%04X\n", on_pi);
			goto group14_done;
		}
		
		/* Update TP for this network */
		eon->tp = tp_on;
		eon->last_update = rds->groups_received;
		
		/* Update legacy single-slot fields */
		rds->on_pi = on_pi;
		rds->on_tp = tp_on;

		if (version == 0) { /* 14A */
			int usage = b2 & RDS_14A_USAGE_MASK;
			
			if (rds->debug)
				LOGP(DRADIO, LOGL_DEBUG, "RDS 14A: ON-PI=%04X Usage=%d TP=%d\n", 
				     on_pi, usage, tp_on);

			if (usage <= 3) {
				/* Variants 0-3: ON-PS Characters (IEC 62106 Table 17) */
				int pos = usage * 2;
				if (rds->group_mask & (1<<2)) { /* Block C valid */
					eon->ps[pos]   = (b3 >> 8) & 0xFF;
					eon->ps[pos+1] = b3 & 0xFF;
					eon->ps_segments |= (1 << usage);
					
					/* Update legacy fields */
					rds->on_ps[pos]   = eon->ps[pos];
					rds->on_ps[pos+1] = eon->ps[pos+1];
					rds->on_ps_segments = eon->ps_segments;
					
					if (rds->verbose) {
						char on_ps_disp[9];
						memcpy(on_ps_disp, eon->ps, 8);
						for (int i = 0; i < 8; i++)
							if (on_ps_disp[i] < 0x20 || on_ps_disp[i] > 0x7E) 
								on_ps_disp[i] = ' ';
						on_ps_disp[8] = '\0';
						LOGP(DRADIO, LOGL_INFO, "RDS 14A: ON-PS segment %d/4. "
						     "Other Network (PI=%04X) Name=\"%s\"\n", 
						     usage+1, on_pi, on_ps_disp);
					}
				}
			}
			else if (usage == 4) {
				/* Variant 4: AF Method A for ON (IEC 62106 S6.2.1.6) */
				if (rds->group_mask & (1<<2)) {
					uint8_t af1 = (b3 >> 8) & 0xFF;
					uint8_t af2 = b3 & 0xFF;
					
					/* Add valid AF codes to ON's AF list */
					if (RDS_VALID_AF_CODE(af1) && eon->af_count < RDS_EON_MAX_AF) {
						eon->af[eon->af_count++] = 875 + af1;
					}
					if (RDS_VALID_AF_CODE(af2) && eon->af_count < RDS_EON_MAX_AF) {
						eon->af[eon->af_count++] = 875 + af2;
					}
					
					if (rds->verbose)
						LOGP(DRADIO, LOGL_INFO, "RDS 14A: ON (PI=%04X) AFs: %d, %d "
						     "(count=%d)\n", on_pi, af1, af2, eon->af_count);
				}
			}
			else if (usage >= 5 && usage <= 9) {
				/* Variants 5-9: Mapped AF (IEC 62106 S6.2.1.6.2) */
				if (rds->group_mask & (1<<2)) {
					uint8_t tuned_af = (b3 >> 8) & 0xFF;
					uint8_t on_af = b3 & 0xFF;
					
					if (RDS_VALID_AF_CODE(tuned_af) && RDS_VALID_AF_CODE(on_af)) {
						/* Store mapped AF if space available */
						if (eon->mapped_af_count < RDS_EON_MAX_MAPPED_AF) {
							/* Check if this mapping already exists */
							int found = 0;
							for (int i = 0; i < eon->mapped_af_count; i++) {
								if (eon->mapped_af[i].tuned_af == tuned_af) {
									eon->mapped_af[i].on_af = on_af;
									found = 1;
									break;
								}
							}
							if (!found) {
								eon->mapped_af[eon->mapped_af_count].tuned_af = tuned_af;
								eon->mapped_af[eon->mapped_af_count].on_af = on_af;
								eon->mapped_af_count++;
							}
						}
						
						if (rds->verbose)
							LOGP(DRADIO, LOGL_INFO, "RDS 14A: ON (PI=%04X) Mapped AF: "
							     "%.1f MHz -> %.1f MHz\n", on_pi, 
							     (875 + tuned_af) / 10.0, (875 + on_af) / 10.0);
					}
				}
			}
			else if (usage == RDS_14A_VARIANT_LINK) {
				/* Variant 12: Linkage Information (IEC 62106 S6.1.5.14)
				 * Block C structure: LA(1) + EG/ILS(2) + SG(1) + LSN(12)
				 * LA = Linkage Actuator, LSN = Linkage Set Number
				 * The upper 4 bits of LSN [11:8] contain the LIC */
				if (rds->group_mask & (1<<2)) {
					int la = (b3 >> RDS_14A_LINK_LA_BIT) & 1;
					uint16_t lsn = b3 & RDS_14A_LINK_LSN_MASK;
					uint8_t lic = (lsn >> 8) & 0x0F;  /* LIC in bits 11-8 */
					
					eon->linkage_la = la;
					eon->linkage_lsn = lsn;
					
					if (rds->verbose)
						LOGP(DRADIO, LOGL_INFO, "RDS 14A: ON (PI=%04X) Linkage: "
						     "LA=%d LSN=0x%03X LIC=%d (%s)\n", 
						     on_pi, la, lsn, lic, rds_get_linkage_name(lic));
				}
			}
			else if (usage == 13) {
				/* Variant 13: ON-PTY and ON-TA (IEC 62106 S6.1.5.14) */
				if (rds->group_mask & (1<<2)) {
					int on_pty = (b3 >> RDS_14A_INFO_PTY_SHIFT) & 0x1F;
					int on_ta  = b3 & RDS_14A_INFO_TA_MASK;
					
					eon->pty = on_pty;
					eon->ta = on_ta;
					
					/* Update legacy fields */
					rds->on_pty = on_pty;
					rds->on_ta = on_ta;
					
					if (rds->verbose)
						LOGP(DRADIO, LOGL_INFO, "RDS 14A: ON (PI=%04X) Info: "
						     "PTY=%d (%s), TA=%s\n",
						     on_pi, on_pty, rds_get_pty_name(on_pty, RDS_IS_RBDS(rds->ecc)),
						     on_ta ? "Yes" : "No");
				}
			}
			else if (usage == 14) {
				/* Variant 14: ON-PIN (IEC 62106 S6.1.5.14) */
				if (rds->group_mask & (1<<2)) {
					eon->pin = b3;
					eon->pin_day = (b3 >> 11) & 0x1F;
					eon->pin_hour = (b3 >> 6) & 0x1F;
					eon->pin_minute = b3 & 0x3F;
					
					/* Update legacy field */
					rds->on_pin = b3;
					
					if (rds->verbose)
						LOGP(DRADIO, LOGL_INFO, "RDS 14A: ON (PI=%04X) PIN: "
						     "Day %d, %02d:%02d\n", 
						     on_pi, eon->pin_day, eon->pin_hour, eon->pin_minute);
				}
			}
			else if (usage == 15) {
				/* Variant 15: Broadcaster Data (reserved) */
				if (rds->group_mask & (1<<2)) {
					eon->broadcaster_data = b3;
					
					if (rds->verbose)
						LOGP(DRADIO, LOGL_INFO, "RDS 14A: ON (PI=%04X) Broadcaster: "
						     "0x%04X\n", on_pi, b3);
				}
			}
		} else { /* 14B */
			/* Group 14B: TA Switch (IEC 62106 S6.1.5.14) */
			int ta_on = (b2 >> 3) & 1;  /* Block B bit 3 is TA for ON */
			
			eon->ta = ta_on;
			rds->on_ta = ta_on;
			
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 14B: ON (PI=%04X) TA Switch: TA=%s\n",
				     on_pi, ta_on ? "Yes" : "No");
		}
		
group14_done:
		; /* Empty statement for label */
	}
	else if (group_type == 3 && version == 0) {
		/* Group 3A: Open Data Application Identification (IEC 62106 S6.1.5.5)
		 * Block B bits 4-0: Application Group Type code
		 * Block C: Application-specific message (ODA-dependent)
		 * Block D: Application Identification (AID) - 16-bit registered code
		 */
		int app_group = b2 & RDS_3A_APP_GROUP_MASK;
		uint16_t oda_message = b3;
		uint16_t aid = b4;
		
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 3A: AppGroup=%d%c AID=%04X msg=%04X\n",
			     app_group >> 1, (app_group & 1) ? 'B' : 'A', aid, oda_message);
		
		/* Store ODA registration in oda_apps[] indexed by carrier group type
		 * This allows the decoder to later identify ODA data on other groups */
		if ((rds->group_mask & (1<<3))) {  /* Block D (AID) valid */
			rds_oda_app_t *oda = &rds->oda_apps[app_group];
			
			/* Check if this is a new or updated registration */
			int is_new = !oda->registered || oda->aid != aid;
			
			oda->carrier_group = app_group;
			oda->aid = aid;
			oda->message = oda_message;
			oda->registered = 1;
			oda->timestamp = time(NULL);
			
			if (is_new && rds->oda_app_count < 32)
				rds->oda_app_count++;
			
			if (rds->verbose) {
				const char *oda_name = "Unknown";
				/* Common ODA AIDs (EBU ODA Registry) */
				switch (aid) {
				case RDS_ODA_AID_DAB_XREF:    oda_name = "DAB Cross-Reference"; break;
				case RDS_ODA_AID_RT_PLUS:    oda_name = "RadioText Plus (RT+)"; break;
				case RDS_ODA_AID_ERT_PLUS:   oda_name = "RT+ for eRT"; break;
				case RDS_ODA_AID_ERT:        oda_name = "Enhanced RadioText (eRT)"; break;
				case RDS_ODA_AID_TMC_ALERT:
				case RDS_ODA_AID_TMC_ALERT2: oda_name = "TMC Alert-C"; break;
				case RDS_ODA_AID_STATION_LOGO: oda_name = "Station Logo"; break;
				}
				LOGP(DRADIO, LOGL_INFO, "RDS 3A: ODA %s - "
				     "AID=0x%04X (%s), carrier=Group %d%c, msg=0x%04X\n",
				     is_new ? "Registered" : "Updated",
				     aid, oda_name,
				     app_group >> 1, (app_group & 1) ? 'B' : 'A',
				     oda_message);
			}
		}
	}
	else if (group_type == 3 && version == 1) {
		/* Group 3B: ODA-only (IEC 62106 S6.1.5.5)
		 * No standard meaning - entirely ODA application dependent.
		 * Block C: PI repeat (standard for all version B groups)
		 * Block D: ODA-specific payload
		 */
		int oda_payload_b = b2 & RDS_3B_PAYLOAD_MASK;
		uint16_t pi_repeat = b3;  /* Block C */
		uint16_t oda_data = b4;   /* Block D */
		
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 3B: ODA-only payload=%02X PI=%04X data=%04X\n",
			     oda_payload_b, pi_repeat, oda_data);
		
		/* Check if an ODA is registered for group 3B (code 7 = 3<<1 | 1) */
		if (rds->oda_apps[7].registered) {
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 3B: ODA data for AID=0x%04X, "
				     "payload=%02X data=%04X\n",
				     rds->oda_apps[7].aid, oda_payload_b, oda_data);
		} else {
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 3B: ODA data (no 3A registration yet)\n");
		}
	}
	else if (group_type == 15 && version == 1) {
		/* Group 15B: Fast Basic Tuning and Switching (IEC 62106 S6.1.5.16)
		 * Mirrors Group 0B Block B payload for fast TA/TP detection.
		 * Block B bits 4-0: TA, M/S, DI, segment (same as 0A/0B)
		 * Block C: PI repeat
		 * Block D: Repeat of Block B payload structure
		 */
		int ta = (b2 & RDS_15B_TA_MASK) >> RDS_15B_TA_BIT;
		int ms = (b2 & RDS_15B_MS_MASK) >> RDS_15B_MS_BIT;
		int di = (b2 & RDS_15B_DI_MASK) >> RDS_15B_DI_BIT;
		int seg = b2 & RDS_15B_SEG_MASK;
		
		/* Update TA (fast switching use case) */
		rds->ta = ta;
		rds->ta_status = rds->block_status[1];
		rds->ms = ms;
		rds->ms_status = rds->block_status[1];
		
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 15B: TA=%d M/S=%d DI=%d seg=%d\n",
			     ta, ms, di, seg);
		
		if (rds->verbose)
			LOGP(DRADIO, LOGL_INFO, "RDS 15B: Fast Switching - "
			     "TA=%s, M/S=%s, DI[%d]=%d\n",
			     ta ? "Yes" : "No",
			     ms ? "Music" : "Speech",
			     3 - seg, di);
	}
	
	/* Debug: detailed group dump */

	// {
	// 	int addr = b2 & 0x1F;  /* Lower 5 bits = address/segment */
	// 	uint8_t c_hi = (b3 >> 8) & 0xFF;
	// 	uint8_t c_lo = b3 & 0xFF;
	// 	uint8_t d_hi = (b4 >> 8) & 0xFF;
	// 	uint8_t d_lo = b4 & 0xFF;
		
	// 	/* Make printable versions */
	// 	char c_str[3], d_str[3];
	// 	c_str[0] = (c_hi >= 0x20 && c_hi <= 0x7E) ? c_hi : '.';
	// 	c_str[1] = (c_lo >= 0x20 && c_lo <= 0x7E) ? c_lo : '.';
	// 	c_str[2] = '\0';
	// 	d_str[0] = (d_hi >= 0x20 && d_hi <= 0x7E) ? d_hi : '.';
	// 	d_str[1] = (d_lo >= 0x20 && d_lo <= 0x7E) ? d_lo : '.';
	// 	d_str[2] = '\0';
		

	// 	LOGP(DRADIO, LOGL_DEBUG, 
	// 		"%04X %04X %04X %04X   %d%c  %d %d%d%d%d%d %d%d%d%d%d  "
	// 		"%d%d%d%d%d%d%d%d %d%d%d%d%d%d%d%d  %d%d%d%d%d%d%d%d %d%d%d%d%d%d%d%d  "
	// 		"%3d %3d  %3d %3d  '%s' '%s'\n",
	// 		pi, b2, b3, b4,
	// 		group_type, version ? 'B' : 'A',
	// 		tp,
	// 		(pty >> 4) & 1, (pty >> 3) & 1, (pty >> 2) & 1, (pty >> 1) & 1, pty & 1,
	// 		(addr >> 4) & 1, (addr >> 3) & 1, (addr >> 2) & 1, (addr >> 1) & 1, addr & 1,
	// 		(c_hi >> 7) & 1, (c_hi >> 6) & 1, (c_hi >> 5) & 1, (c_hi >> 4) & 1,
	// 		(c_hi >> 3) & 1, (c_hi >> 2) & 1, (c_hi >> 1) & 1, c_hi & 1,
	// 		(c_lo >> 7) & 1, (c_lo >> 6) & 1, (c_lo >> 5) & 1, (c_lo >> 4) & 1,
	// 		(c_lo >> 3) & 1, (c_lo >> 2) & 1, (c_lo >> 1) & 1, c_lo & 1,
	// 		(d_hi >> 7) & 1, (d_hi >> 6) & 1, (d_hi >> 5) & 1, (d_hi >> 4) & 1,
	// 		(d_hi >> 3) & 1, (d_hi >> 2) & 1, (d_hi >> 1) & 1, d_hi & 1,
	// 		(d_lo >> 7) & 1, (d_lo >> 6) & 1, (d_lo >> 5) & 1, (d_lo >> 4) & 1,
	// 		(d_lo >> 3) & 1, (d_lo >> 2) & 1, (d_lo >> 1) & 1, d_lo & 1,
	// 		c_hi, c_lo, d_hi, d_lo,
	// 		c_str, d_str);
	// }

	/* Check for ODA groups that need routing to specific decoders */
	int group_code = (group_type << 1) | version;
	if (rds->oda_apps[group_code].registered) {
		rds_oda_app_t *oda = &rds->oda_apps[group_code];
		uint16_t aid = oda->aid;
		
		switch (aid) {
		case RDS_ODA_AID_RT_PLUS:
			rds_decode_rtplus(rds, rds->blocks, rds->block_status, &rds->rtplus, oda);
			break;
		case RDS_ODA_AID_ERT_PLUS:
			rds_decode_rtplus(rds, rds->blocks, rds->block_status, &rds->ert_plus, oda);
			break;
		case RDS_ODA_AID_ERT:
			rds_decode_ert(rds, rds->blocks, rds->block_status, oda);
			break;
		}
	}
	
	rds->groups_received++;
}

/* Simple 1-pole IIR Low Pass Filter step (RC equivalent) */
static inline double iir_lpf_step(rds_iir_filter_t *f, double input)
{
	f->y[0] += f->a[0] * (input - f->y[0]);
	return f->y[0];
}

/* Differential Biphase Decoding (IEC 62106 S2.2)
 *
 * DBPSK demodulation with automatic phase alignment.
 *
 * The RDS signal uses Differential BPSK where information is encoded in
 * phase CHANGES rather than absolute phase. This provides robustness
 * against 180deg phase ambiguity - receivers can lock to either phase.
 *
 * At the symbol rate (1187.5 Hz), we process pairs of integrated samples:
 * - Each biphase symbol spans 2 clock half-periods
 * - We maintain two candidate reading frames (even/odd alignment)
 * - Error counting determines which frame alignment is correct
 *
 * Differential decoding: output_bit = (current_symbol XOR previous_symbol)
 * This recovers the original data regardless of absolute carrier phase.
 */
static int rds_biphase_decode(rds_decoder_t *rds, double acc)
{
	int bit_decoded = 0;
	
	/* Determine symbol polarity (+1 or -1) */
	int s_acc = (acc >= 0) ? 1 : -1;
	int s_prev = (rds->prev_integral >= 0) ? 1 : -1;
	
	/* 
	 * Phase alignment error counting:
	 * Track sign changes between consecutive samples for each candidate frame.
	 * The correct frame alignment will have fewer unexpected transitions.
	 */
	if (s_acc != s_prev) {
		rds->total_errors[rds->biphase_counter % 2]++;
	}

	if ((rds->biphase_counter % 2) == rds->reading_frame) {
		/* Symbol decision: use dominant polarity of the pair */
		int sum_sign = (acc + rds->prev_integral >= 0) ? 1 : -1;
		
		/* Differential decode: XOR with previous symbol recovers data bit */
		int raw_bit = (sum_sign != rds->last_diff_bit) ? 1 : 0;
		rds->last_diff_bit = sum_sign;
		
		rds->curr_bit = raw_bit;
		bit_decoded = 1;
	}
	
	/* Auto-phase alignment (every ~0.67 sec) */
	if (rds->biphase_counter == 0) {
		/* Prefer frame with FEWER errors */
		if (rds->total_errors[1 - rds->reading_frame] < rds->total_errors[rds->reading_frame]) {
			rds->reading_frame = 1 - rds->reading_frame;
			// LOGP(DRADIO, LOGL_DEBUG, "RDS: Realigning biphase frame\n");
		}
		rds->total_errors[0] = 0;
		rds->total_errors[1] = 0;
	}
	
	rds->prev_integral = acc;
	rds->biphase_counter = (rds->biphase_counter + 1) % 800;
	
	return bit_decoded;
}

int rds_decoder_init(rds_decoder_t *rds, double samplerate, int debug, int verbose, double time_constant_us)
{
	memset(rds, 0, sizeof(*rds));
	rds->samplerate = samplerate;
	rds->debug = debug;
	rds->verbose = verbose;

	/* Heuristic: Only 50µs emphasis triggers RBDS (North America) assumption.
	 * All other values (including 75µs) default to RDS, which is more common globally.
	 * This sets a default ECC in the RBDS range (0xA0) for correct PTY name display
	 * (e.g. "Top 40") before the actual ECC is received from Group 1A. */
	if (RDS_IS_RBDS_EMPHASIS(time_constant_us)) {
		rds->ecc = 0xA0; /* Start with RBDS assumption */
	}
	
	/* Initialize Subcarrier PLL (57 kHz) */
	rds->freq_subcarrier = 57000.0;
	
	/* Filter Coefficients */
	/* 2400 Hz LPF */
	double alpha_2400 = (2.0 * M_PI * 2400.0) / samplerate;
	rds->filter_2400_i.a[0] = alpha_2400;
	rds->filter_2400_q.a[0] = alpha_2400;
	
	/* PLL Loop Filter (~54 Hz BW scaled) */
	double alpha_pll = 0.054 * (250000.0 / samplerate);
	/* Clamp alpha to sane range */
	if (alpha_pll > 1.0) alpha_pll = 1.0;
	if (alpha_pll < 0.001) alpha_pll = 0.001;
	rds->filter_pll.a[0] = alpha_pll;
	
	rds->status_interval = samplerate * 30.0;  /* Status dump every 30 seconds */
	
	/* BER Init */
	for (int i=0; i<BER_WINDOW_SIZE; i++) rds->ber_history[i] = 1.0f;
	rds->ber_accumulator = (double)BER_WINDOW_SIZE;
	rds->ber_percent = 100.0;
	
	/* Initialize status to NONE */
	rds->pi_status = RDS_STATUS_NONE;
	rds->pty_status = RDS_STATUS_NONE;
	rds->tp_status = RDS_STATUS_NONE;
	rds->ta_status = RDS_STATUS_NONE;
	memset(rds->ps_status, RDS_STATUS_NONE, sizeof(rds->ps_status));
	memset(rds->rt_status, RDS_STATUS_NONE, sizeof(rds->rt_status));
	rds->ecc_status = RDS_STATUS_NONE;
	rds->language_status = RDS_STATUS_NONE;
	rds->pin_status = RDS_STATUS_NONE;
	rds->ct_status = RDS_STATUS_NONE;
	memset(rds->ptyn_status, RDS_STATUS_NONE, sizeof(rds->ptyn_status));
	rds->rt_display_version = 0xFF;  /* No version in display yet */
	memset(rds->ptyn, ' ', 8);
	rds->ptyn[8] = '\0';
	
	/* Initialize EON fields */
	memset(rds->on_ps, ' ', 8);
	rds->on_ps[8] = '\0';
	
	/* TODO: Enhanced Other Networks (EON) Implementation
	 * Current implementation tracks only the "current" Other Network (single-slot).
	 * A full implementation requires a database/list of networks because:
	 * 1. EON data is interleaved: The receiver may get a chunk of ON-PS for Station A,
	 *    then a Traffic Flag for Station B, then the next chunk for Station A.
	 * 2. ON-PS is 8 characters sent in 4 chunks (pairs). Reassembly requires
	 *    persistence per ON-PI.
	 * 3. Mapped Frequencies (AF) for multiple networks need storage to be useful.
	 */
	
	if (verbose)
		LOGP(DRADIO, LOGL_INFO, "RDS decoder initialized (IEC 62106 DBPSK, fs=%.0f, debug=%d, verbose=%d)\n", samplerate, debug, verbose);
	return 0;
}

void rds_decoder_process(rds_decoder_t *rds, sample_t *samples, int num,
                         double pilot_phase, double pilot_phasestep)
{
	(void)pilot_phase;
	(void)pilot_phasestep;
	int i;
	const double PLL_BETA = 50.0 * (250000.0 / rds->samplerate) * 0.01; /* Tuned gain */
	const double FC_TOLERANCE = 12.0;
	const double SUBCARRIER_BITRATE_RATIO = 48.0;
	
	int decimate = (int)(rds->samplerate / 7125.0);
	if (decimate < 1) decimate = 1;
	
	/* Phase step per sample for 57k (free running part) */
	/* Actually we update phase by freq_subcarrier */
	double pll_step_base = 2.0 * M_PI / rds->samplerate;
	
	for (i = 0; i < num; i++) {
		rds->status_timer++;
		
		/* 1. Subcarrier Oscillator & Downmix */
		rds->phase_subcarrier += pll_step_base * rds->freq_subcarrier;
		/* 
		   NOTE: We do NOT wrap phase_subcarrier here (e.g. fmod 2PI).
		   We let it grow monotonically so that the derived clock phase
		   (phase / 48) remains continuous.
		*/
		
		double sc_sin, sc_cos;
		if (fm_fast_math_enabled()) {
			/* Convert phase to 0-65535 range for table lookup */
			double wrapped = fmod(rds->phase_subcarrier, 2.0 * M_PI);
			if (wrapped < 0) wrapped += 2.0 * M_PI;
			fm_fast_sincos(wrapped * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
		} else {
			sincos(rds->phase_subcarrier, &sc_sin, &sc_cos);
		}
		double bb_i = samples[i] * sc_cos;
		double bb_q = samples[i] * sc_sin;
		
		/* 2. Filter I/Q (2400 Hz) */
		bb_i = iir_lpf_step(&rds->filter_2400_i, bb_i);
		bb_q = iir_lpf_step(&rds->filter_2400_q, bb_q);
		
		/* 3. PLL Costas Error */
		double err = bb_i * bb_q; /* Phase error */
		double d_phi_sc = iir_lpf_step(&rds->filter_pll, err);
		
		/* Update PLL */
		rds->phase_subcarrier -= PLL_BETA * d_phi_sc;
		rds->freq_subcarrier  -= 0.5 * PLL_BETA * d_phi_sc;
		
		/* Debug PLL (every 1000 samples) */
		
		// if ((int)rds->status_timer % 1000 == 0) {
		// 	LOGP(DRADIO, LOGL_DEBUG, "RDS PLL: freq=%.2f err=%.6f bb_i=%.4f bb_q=%.4f\n", rds->freq_subcarrier, err, bb_i, bb_q);
		// }
		

		/* 4. Decimation & Symbol Processing */
		static int sample_count = 0;
		if (++sample_count >= decimate) {
			sample_count = 0;
			
			/* Clamp Freq */
			if (rds->freq_subcarrier > 57000.0 + FC_TOLERANCE) rds->freq_subcarrier = 57000.0 + FC_TOLERANCE;
			if (rds->freq_subcarrier < 57000.0 - FC_TOLERANCE) rds->freq_subcarrier = 57000.0 - FC_TOLERANCE;
			
			/* 5. Clock Recovery (Derived) */
			/* Directly divide the monotonic subcarrier phase */
			double clock_phi = (rds->phase_subcarrier / SUBCARRIER_BITRATE_RATIO) + rds->clock_offset;
			clock_phi = fmod(clock_phi, 2.0 * M_PI);
			if (clock_phi < 0) clock_phi += 2.0 * M_PI;
			
			int lo_clock = (clock_phi < M_PI) ? 1 : -1;
			
			/* 6. Alignment (Zero-Crossing) */
			int curr_sign = (bb_i >= 0) ? 1 : -1;
			int prev_sign = (rds->prev_bb_sample >= 0) ? 1 : -1;
			
			if (curr_sign != prev_sign) {
				/* Crossed zero. Check phase. */
				double d_cphi = fmod(clock_phi, M_PI); /* dist from 0 or PI */
				if (d_cphi >= M_PI/2.0) d_cphi -= M_PI; /* -PI/2 .. +PI/2 */
				
				/* Nudge offset to align edge to zero crossing */
				rds->clock_offset -= 0.005 * d_cphi;
				
				/* Debug Clock Alignment */
				////// LOGP(DRADIO, LOGL_DEBUG, "RDS CLK: ZC adjust d_cphi=%.4f new_offset=%.4f\n", d_cphi, rds->clock_offset);
			}
			rds->prev_bb_sample = bb_i;
			
			/* 7. Integrate */
			rds->integrator += bb_i * lo_clock;
			
			/* 8. Dump on clock edge */
			if (lo_clock != rds->prev_clock_bit) {
				/* Debug Integration result before decoding */
				// LOGP(DRADIO, LOGL_DEBUG, "RDS INT: integrator=%.4f lo_clock=%d\n", rds->integrator, lo_clock);

				if (rds_biphase_decode(rds, rds->integrator)) {
					/* Got Bit */
					rds->shift_reg = ((rds->shift_reg << 1) | rds->curr_bit) & 0x3FFFFFF;
					rds->bit_count++;
					rds->bit_count_in_block++;
					
					/* Debug Bit Stream */
					// LOGP(DRADIO, LOGL_DEBUG, "RDS BIT: %d (reg=%07X)\n", rds->curr_bit, rds->shift_reg);
					
					/* Sync State Machine */
					rds->bit_time++;
					
					if (!rds->synced) {
						/* Search Mode: Check every bit for valid syndrome */
						uint16_t block_index;
						if (rds_check_syndrome(rds->shift_reg, &block_index)) {
							/* Valid syndrome found. Track it for multi-hit confirmation. */
							int offset = rds->bit_time % 26;
							int pseudo_block = ((rds->bit_time / 26) + 4 - block_index) % 4;
							
							if (rds->debug)
								LOGP(DRADIO, LOGL_DEBUG, "RDS SYNC: Hit at offset=%d pseudo=%d block=%c\n", 
									offset, pseudo_block, 'A'+block_index);
							
							/* Age out old hits */
							if (rds->sync_hit_time[offset][pseudo_block] < rds->bit_time - SYNC_CONFIRM_BITS) {
								rds->sync_hits[offset][pseudo_block] = 0;
							}
							
							/* Record this hit */
							rds->sync_hits[offset][pseudo_block]++;
							rds->sync_hit_time[offset][pseudo_block] = rds->bit_time;
							
							/* Check if we have enough hits to acquire sync */
							if (rds->sync_hits[offset][pseudo_block] > SYNC_THRESHOLD) {
								rds->synced = 1;
								rds->block_idx = (block_index + 1) & 0x03;
								rds->errors = 0;
								rds->blocks_in_group = 0;
								rds->group_error_count = 0;
								rds->blocks_received++;
								rds->group_mask = (1 << block_index);
								rds->nb_ok = 1;
								rds->nb_unsync = 0;
								rds->bit_count_in_block = 0;
								
								uint16_t data = (rds->shift_reg >> 10) & 0xFFFF;
								rds->blocks[block_index] = data;
								rds->blocks_ok++;
								
								LOGP(DRADIO, LOGL_NOTICE, "RDS SYNC: Acquired at offset %c (hits=%d)\n", 
									'A'+block_index, rds->sync_hits[offset][pseudo_block]);
								
								/* Clear sync hit array */
								memset(rds->sync_hits, 0, sizeof(rds->sync_hits));
								memset(rds->sync_hit_time, 0, sizeof(rds->sync_hit_time));
							}
						}
					} else {
						/* Synced Mode: Flywheel - only check at expected boundaries */
						if (rds->bit_count_in_block >= 26) {
							rds->bit_count_in_block = 0; /* Reset for next block */
							rds->blocks_received++;
							
							uint16_t offset;
							rds_check_syndrome(rds->shift_reg, &offset); /* Ignore return value for now */
							
							int valid_syndrome = rds_check_syndrome(rds->shift_reg, &offset);
							
							if (valid_syndrome && (int)offset == rds->block_idx) {
								uint16_t data = (rds->shift_reg >> 10) & 0xFFFF;
								// LOGP(DRADIO, LOGL_DEBUG, "RDS SYNC: Valid Block offset=%c data=%04X\n", 'A'+offset, data);
								
								/* Valid Match */
								rds->blocks[rds->block_idx] = data;
								rds->group_mask |= (1 << rds->block_idx);
								rds->block_status[rds->block_idx] = RDS_STATUS_VALID;
								rds->blocks_ok++;
								rds->nb_ok++;
								rds->errors = 0;
							} else {
								/* Mismatch or Invalid - Try FEC */
								uint32_t corrected_block;
								if (rds_correct_errors(rds->shift_reg, rds->block_idx, &corrected_block)) {
									/* FEC succeeded! */
									uint16_t data = (corrected_block >> 10) & 0xFFFF;
									// LOGP(DRADIO, LOGL_DEBUG, "RDS FEC: Corrected block %c data=%04X\n", 'A'+rds->block_idx, data);
									
									rds->blocks[rds->block_idx] = data;
									rds->group_mask |= (1 << rds->block_idx);
									rds->block_status[rds->block_idx] = RDS_STATUS_CORRECTED;
									rds->blocks_ok++;
									rds->nb_ok++;
									rds->errors = 0;
								} else {
									/* FEC failed */
									if (valid_syndrome) {
										if (rds->debug)
											LOGP(DRADIO, LOGL_DEBUG, "RDS SYNC: Unexpected Offset! Expected %d Got %d. Ignoring.\n", rds->block_idx, offset);
									} else {
										// LOGP(DRADIO, LOGL_DEBUG, "RDS SYNC: Invalid Syndrome (reg=%07X)\n", rds->shift_reg);
									}
									rds->errors++;
									rds->blocks_bad++;
									rds->group_error_count++;
								}
							}
							
							/* Always advance expected block index */
							rds->block_idx = (rds->block_idx + 1) & 0x03;
							
							/* Group Processing at end of cycle */
							if (rds->block_idx == 0) {
								rds_decode_group(rds);
								rds->group_mask = 0;
								
								/* Sync loss detection */
								if (rds->nb_ok > 0) {
									rds->nb_unsync = 0;
								} else {
									rds->nb_unsync++;
								}
								
								if (rds->nb_unsync >= SYNC_LOSS_GROUPS) {
									LOGP(DRADIO, LOGL_NOTICE, "RDS SYNC: Lost Sync (%d groups with no valid blocks)\n", rds->nb_unsync);
									rds->synced = 0;
								}
								
								rds->nb_ok = 0;
							}
							
							/* BER Update (only on block boundaries) */
							rds->blocks_in_group++;
							if (rds->blocks_in_group >= 4) {
								float g_ber = (float)rds->group_error_count / 4.0;
								rds->ber_accumulator -= rds->ber_history[rds->ber_history_idx];
								rds->ber_history[rds->ber_history_idx] = g_ber;
								rds->ber_accumulator += g_ber;
								rds->ber_history_idx = (rds->ber_history_idx + 1) % BER_WINDOW_SIZE;
								rds->ber_percent = (rds->ber_accumulator / BER_WINDOW_SIZE) * 100.0;
								rds->blocks_in_group = 0;
								rds->group_error_count = 0;
							}
						}
					}
				}
				rds->integrator = 0;
			}
			rds->prev_clock_bit = lo_clock;
		}
	}
	
	/* Output status */
	if (rds->status_timer >= rds->status_interval) {
		rds->status_timer = 0;
		rds_decoder_status(rds);
	}
}
int rds_dec_get_pi(rds_decoder_t *rds, uint16_t *pi)
{
	if (rds->groups_received > 0) {
		*pi = rds->pi;
		return 1;
	}
	return 0;
}

int rds_dec_get_ps(rds_decoder_t *rds, char *ps)
{
	if (rds->ps_segments == 0x0F) {
		memcpy(ps, rds->ps, 9);
		return 1;
	}
	return 0;
}

int rds_dec_get_rt(rds_decoder_t *rds, char *rt)
{
	if (rds->rt_segments) {
		memcpy(rt, rds->rt, 65);
		return 1;
	}
	return 0;
}

void rds_decoder_status(rds_decoder_t *rds)
{
	char ps_display[64];   /* Larger buffer for UTF-8 + control char display */
	char rt_display[512];  /* Larger buffer for UTF-8 + control char display */
	int i;
	
	if (rds->blocks_received < 10) return;
	
	/* Helper: status character */
	#define STATUS_CHAR(s) ((s) == RDS_STATUS_VALID ? '+' : \
	                        (s) == RDS_STATUS_CORRECTED ? '?' : \
	                        (s) == RDS_STATUS_ERROR ? 'X' : '_')
	
	/* Convert PS to display-safe UTF-8 (control chars shown as <XX>) */
	rds_text_to_display((uint8_t *)rds->ps, 8, ps_display, sizeof(ps_display));
	
	/* Convert RT to display-safe UTF-8 (control chars shown as <XX>) */
	rds_text_to_display((uint8_t *)rds->rt, 64, rt_display, sizeof(rt_display));
	
	/* Header */
	LOGP(DRADIO, LOGL_NOTICE, "=== RDS Status Dump ===\n");
	
	/* PI and PS with status beneath */
	{
		char ps_status_str[9];
		char pi_str[32];
		const char *call = RDS_IS_RBDS(rds->ecc) ? rds_get_callsign(rds->pi) : NULL;

		if (call)
			snprintf(pi_str, sizeof(pi_str), "%04X (%s)", rds->pi, call);
		else
			snprintf(pi_str, sizeof(pi_str), "%04X", rds->pi);

		for (i=0; i<8; i++) {
			ps_status_str[i] = STATUS_CHAR(rds->ps_status[i]);
		}
		ps_status_str[8] = '\0';
		LOGP(DRADIO, LOGL_NOTICE, "PI=%-11s %c  PS=\"%s\"  Sync=%s  BER=%.1f%%\n",
		     pi_str, STATUS_CHAR(rds->pi_status), ps_display,
		     rds->synced ? "Y" : "N",
		     rds->ber_percent);
		/*               PI=XXXX X  PS=" = 15 chars padding */
		LOGP(DRADIO, LOGL_NOTICE, "               %s\n", ps_status_str);
	}
	
	/* Flags with status */
	{
		char pty_name_buf[32];
		snprintf(pty_name_buf, sizeof(pty_name_buf), "%s", rds_get_pty_name(rds->pty, RDS_IS_RBDS(rds->ecc)));
		LOGP(DRADIO, LOGL_NOTICE, "PTY=%d %c (%s)  TP=%s %c  TA=%s %c\n",
		     rds->pty, STATUS_CHAR(rds->pty_status), pty_name_buf,
		     rds->tp ? "Y" : "N", STATUS_CHAR(rds->tp_status),
		     rds->ta ? "Y" : "N", STATUS_CHAR(rds->ta_status));
	}
	
	/* Music/Speech and Decoder Identification flags */
	{
		LOGP(DRADIO, LOGL_NOTICE, "M/S=%s %c  DI: Stereo=%s ArtHead=%s Compress=%s DynPTY=%s %c\n",
		     rds->ms ? "Music" : "Speech", STATUS_CHAR(rds->ms_status),
		     rds->di_stereo ? "Y" : "N",
		     rds->di_artificial_head ? "Y" : "N",
		     rds->di_compressed ? "Y" : "N",
		     rds->di_dynamic_pty ? "Y" : "N",
		     STATUS_CHAR(rds->di_status));
	}
	
	/* Alternative Frequencies (Method A) */
	if (rds->af_method_a_dec.received_count > 0) {
		rds_af_method_a_dec_t *afd = &rds->af_method_a_dec;
		char af_buf[256] = "";
		int pos = 0;
		
		for (int j = 0; j < afd->received_count && pos < 240; j++) {
			if (afd->type[j] == RDS_AF_FREQ_VHF) {
				pos += snprintf(af_buf + pos, sizeof(af_buf) - pos, "%.1f ",
				                afd->freq[j] / 10.0);
			} else {
				pos += snprintf(af_buf + pos, sizeof(af_buf) - pos, "%d(%s) ",
				                afd->freq[j],
				                afd->type[j] == RDS_AF_FREQ_LF ? "LF" : "MF");
			}
		}
		LOGP(DRADIO, LOGL_NOTICE, "Alt. Frequencies    : %s%s\n", af_buf,
		     afd->complete ? "[complete]" : "[partial]");
	}
	
	/* AF Method A Last Good (if different from current or current has errors) */
	if (rds->af_method_a_dec.last_good_count > 0) {
		rds_af_method_a_dec_t *afd = &rds->af_method_a_dec;
		char buf[256] = "";
		int pos = 0;
		
		pos += snprintf(buf + pos, sizeof(buf) - pos, "  [1] PI=%04X: ",
		                afd->last_good_pi);
		
		for (int j = 0; j < afd->last_good_count && pos < 240; j++) {
			if (afd->last_good_type[j] == RDS_AF_FREQ_VHF) {
				pos += snprintf(buf + pos, sizeof(buf) - pos, "%.1f ",
				                afd->last_good_freq[j] / 10.0);
			} else {
				pos += snprintf(buf + pos, sizeof(buf) - pos, "%d(%s) ",
				                afd->last_good_freq[j],
				                afd->last_good_type[j] == RDS_AF_FREQ_LF ? "LF" : "MF");
			}
		}
		LOGP(DRADIO, LOGL_NOTICE, "AF Method A Last OK :\n");
		LOGP(DRADIO, LOGL_NOTICE, "%s\n", buf);
	}

	/* Alternative Frequencies (Method B History) */
	if (rds->af_method_b_dec.history_count > 0) {
		LOGP(DRADIO, LOGL_NOTICE, "AF Method B History :\n");
		for (int i = 0; i < rds->af_method_b_dec.history_count; i++) {
			rds_af_method_b_history_t *h = &rds->af_method_b_dec.history[i];
			char buf[256] = "";
			int pos = 0;
			
			pos += snprintf(buf + pos, sizeof(buf) - pos, "  [%d] PI=%04X Tune=%.1f: ",
			                i + 1, h->pi, h->tuning_freq / 10.0);
			
			for (int j = 0; j < h->received_count && pos < 240; j++) {
				pos += snprintf(buf + pos, sizeof(buf) - pos, "%s%.1f ",
				                h->af_is_regional[j] ? "R" : "",
				                h->af_freq[j] / 10.0);
			}
			LOGP(DRADIO, LOGL_NOTICE, "%s\n", buf);
		}
	}

	/* Block statistics */
	LOGP(DRADIO, LOGL_NOTICE, "Groups=%d  Blocks: OK=%ld  Bad=%ld  Total=%ld\n",
	     rds->groups_received, rds->blocks_ok, rds->blocks_bad, rds->blocks_received);

	
	/* PTYN (if any bits received) */
	if (rds->ptyn_segments) {
		char ptyn_display[64];
		char ptyn_status_str[9];
		
		rds_text_to_display((uint8_t *)rds->ptyn, 8, ptyn_display, sizeof(ptyn_display));
		for (i=0; i<8; i++) {
			ptyn_status_str[i] = STATUS_CHAR(rds->ptyn_status[i]);
		}
		ptyn_status_str[8] = '\0';
		
		LOGP(DRADIO, LOGL_NOTICE, "PTYN=\"%s\"\n", ptyn_display);
		/* Note: Status shows per-byte quality, may not align if control chars expanded */
		LOGP(DRADIO, LOGL_NOTICE, "      %s\n", ptyn_status_str);
	}
	
	/* RadioText (if any) with group version and A/B flag */
	if (rt_display[0] != '\0') {
		/* Note: rt_display contains UTF-8 with <XX> for control chars.
		 * Status string is per source byte, not per display character.
		 * For terminals, this won't align perfectly if control chars present. */
		LOGP(DRADIO, LOGL_NOTICE, "RT[%s AB=%c]=\"%s\"\n",
		     rds->rt_display_version == 0 ? "2A" : 
		     rds->rt_display_version == 1 ? "2B" : "??",
		     rds->rt_ab ? 'B' : 'A',
		     rt_display);
	}
	
	/* Extended info (if available) with status */
	if (rds->ecc != 0 || rds->language_code != 0) {
		uint8_t cc = (rds->pi >> 12) & 0x0F;
		LOGP(DRADIO, LOGL_NOTICE, "Country=%s %c (ECC=0x%02X CC=%d)  Language=%s %c (%d)\n",
		     rds_get_country_code(cc, rds->ecc), STATUS_CHAR(rds->ecc_status),
		     rds->ecc, cc,
		     rds_get_language_name(rds->language_code), STATUS_CHAR(rds->language_status),
		     rds->language_code);
	}
	
	/* Clock-Time (if received) with status */
	if (rds->ct_valid) {
		int yp = (int)((rds->ct_mjd - 15078.2) / 365.25);
		int mp = (int)((rds->ct_mjd - 14956.1 - (int)(yp * 365.25)) / 30.6001);
		int day = rds->ct_mjd - 14956 - (int)(yp * 365.25) - (int)(mp * 30.6001);
		int k = (mp == 14 || mp == 15) ? 1 : 0;
		int year = 1900 + yp + k;
		int month = mp - 1 - k * 12;
		
		LOGP(DRADIO, LOGL_NOTICE, "CT=%04d-%02d-%02d %02d:%02d UTC %c (offset=%+.1fh)\n",
		     year, month, day, rds->ct_hour, rds->ct_minute, 
		     STATUS_CHAR(rds->ct_status), rds->ct_offset / 2.0);
	}
	
	/* PIN (if set) with status */
	if (rds->pin_day > 0) {
		LOGP(DRADIO, LOGL_NOTICE, "PIN: Scheduled Day %d, %02d:%02d %c\n",
		     rds->pin_day, rds->pin_hour, rds->pin_minute, STATUS_CHAR(rds->pin_status));
	}
	
	LOGP(DRADIO, LOGL_NOTICE, "=======================\n");
	#undef STATUS_CHAR
}

void rds_decoder_exit(rds_decoder_t *rds)
{
	(void)rds;
}
