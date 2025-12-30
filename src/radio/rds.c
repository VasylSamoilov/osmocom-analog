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

/* Unified AF Table Management
 * Find existing entry by PI+Type+TX, or allocate new one.
 * Implements 5-step priority replacement for Method B.
 * Returns pointer to entry, or NULL on error.
 */
static rds_af_entry_t *rds_af_get_entry(rds_decoder_t *rds, rds_af_table_t *table, 
                                        uint16_t pi, rds_af_type_t type, uint16_t tx_freq)
{
	int i;
	
	/* Candidate indexes */
	int idx_free = -1;
	int idx_invalid = -1;
	int idx_lru = -1;
	
	/* Timestamps for finding oldest (lowest value) */
	uint32_t t_invalid = UINT32_MAX;
	uint32_t t_lru = UINT32_MAX;
	
	/* Track oldest valid match for appending */
	int target_idx = -1;
	uint32_t oldest_match_time = UINT32_MAX;
	
	/* 1. Check for Free slot */
	if (table->count < RDS_AF_TABLE_SIZE) {
		idx_free = table->count;
	}
	
	/* Iterate to find candidates */
	for (i = 0; i < table->count; i++) {
		rds_af_entry_t *e = &table->entries[i];
		
		/* Match check: PI + Type + TX */
		int is_match = (e->pi == pi && e->type == type && e->tx_freq == tx_freq);
		
		/* Invalid check (Method B only) */
		int has_invalid = 0;
		if (e->type == RDS_AF_TYPE_METHOD_B) {
			for (int j = 0; j < e->count; j++) {
				if (e->list[j].priority == RDS_AF_PRIORITY_INVALID) {
					has_invalid = 1;
					break;
				}
			}
		}
		
		/* Found a match - ONLY use for appending if it is VALID (or not Method B)
		 * For Method B, if it has invalid entries, ignore here so it falls to priority selection. */
		if (is_match && !has_invalid) {
			if (e->last_update < oldest_match_time) {
				oldest_match_time = e->last_update;
				target_idx = i;
			}
		}
		
		/* 3. Invalid (Any Method B) (Oldest) */
		if (has_invalid) {
			if (e->last_update < t_invalid) {
				t_invalid = e->last_update;
				idx_invalid = i;
			}
		}
		
		/* 5. LRU (Any) (Oldest) */
		if (e->last_update < t_lru) {
			t_lru = e->last_update;
			idx_lru = i;
		}
	}
	
	/* If found existing VALID match, return it (Append mode - preserve data) */
	if (target_idx >= 0) {
		table->entries[target_idx].last_update = rds->af_update_counter++;
		return &table->entries[target_idx];
	}
	
	/* Select candidate based on priority order */
	int selected_idx = -1;
	
	if (idx_free != -1) {
		/* 1. Free */
		selected_idx = table->count++; /* Increment count */
	} else {
		/* No free slots - replacement logic */
		if (type == RDS_AF_TYPE_METHOD_B) {
			/* Method B Priority: Inv -> LRU */
			if (idx_invalid != -1) selected_idx = idx_invalid;
			else if (idx_lru != -1) selected_idx = idx_lru;
		} else {
			/* Method A / LF/MF Simple: LRU */
			if (idx_lru != -1) selected_idx = idx_lru;
		}
	}
	
	if (selected_idx >= 0) {
		rds_af_entry_t *e = &table->entries[selected_idx];
		
		/* Replacement/Repair Mode: ALWAYS wipe/init */
		int old_tx = e->tx_freq;
		if (rds->debug && (e->pi != pi || e->type != type || e->tx_freq != tx_freq))
			LOGP(DRADIO, LOGL_DEBUG, "RDS: Replacing AF entry %d (old_TX=%.1f, new_TX=%.1f)\n", 
			     selected_idx, old_tx/10.0, tx_freq/10.0);
		
		memset(e, 0, sizeof(rds_af_entry_t));
		e->pi = pi;
		e->type = type;
		e->tx_freq = tx_freq;
		
		/* Update timestamp */
		e->last_update = rds->af_update_counter++;
		return e;
	}
	
	return NULL;
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
	uint32_t blocks[4];
	uint16_t b2, b3;
	int seg = rds->ps_segment;
	
	/* DI flag depends on segment address */
	uint8_t di_flags[4] = {
		rds->di_dynamic_pty,
		rds->di_compressed,
		rds->di_artificial_head,
		rds->di_stereo
	};
	uint8_t di = di_flags[seg] ? 1 : 0;
	
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
	
	/* Block C: Alternative Frequencies (AF) */
	rds_af_entry_t *e = NULL;
	
	/* Find AF entry based on configuration */
	if (rds->af_type_cfg == RDS_AF_TYPE_METHOD_A) {
		for(int i=0; i<rds->af_table.count; i++) {
			if (rds->af_table.entries[i].type == RDS_AF_TYPE_METHOD_A && 
			    rds->af_table.entries[i].pi == rds->pi) {
				e = &rds->af_table.entries[i];
				break;
			}
		}
	} else if (rds->af_type_cfg == RDS_AF_TYPE_LF_MF) {
		for(int i=0; i<rds->af_table.count; i++) {
			if (rds->af_table.entries[i].type == RDS_AF_TYPE_LF_MF && 
			    rds->af_table.entries[i].pi == rds->pi) {
				e = &rds->af_table.entries[i];
				break;
			}
		}
	} else if (rds->af_type_cfg == RDS_AF_TYPE_METHOD_B) {
		/* Determine which Method B list to transmit */
		/* Try cached index first */
		if (rds->af_list_index >= 0 && rds->af_list_index < rds->af_table.count &&
		    rds->af_table.entries[rds->af_list_index].type == RDS_AF_TYPE_METHOD_B &&
		    rds->af_table.entries[rds->af_list_index].pi == rds->pi) {
			e = &rds->af_table.entries[rds->af_list_index];
		} else {
			/* Fallback: Search from start */
			for(int i=0; i<rds->af_table.count; i++) {
				if (rds->af_table.entries[i].type == RDS_AF_TYPE_METHOD_B && 
				    rds->af_table.entries[i].pi == rds->pi) {
					e = &rds->af_table.entries[i];
					rds->af_list_index = i; /* Cache it */
					break;
				}
			}
		}
	}
	
	if (e && e->count > 0) {
		if (e->type == RDS_AF_TYPE_METHOD_A) {
			/* Method A: Header (Count) + List logic */
			if (rds->af_segment == 0) {
				/* Header: Count + First AF */
				uint8_t af_count_code = RDS_AF_NO_AF + e->count;
				uint8_t af1_code = (e->list[0].freq >= RDS_AF_FM_BASE) ? (e->list[0].freq - RDS_AF_FM_BASE) : 0;
				b3 = (af_count_code << 8) | af1_code;
			} else {
				/* Pairs */
				int idx = rds->af_segment * 2 - 1;
				uint16_t freq1 = (idx < e->count) ? e->list[idx].freq : 0;
				uint16_t freq2 = (idx + 1 < e->count) ? e->list[idx+1].freq : 0;
				
				uint8_t af1 = (freq1 >= RDS_AF_FM_BASE) ? (freq1 - RDS_AF_FM_BASE) : RDS_AF_FILLER;
				uint8_t af2 = (freq2 >= RDS_AF_FM_BASE) ? (freq2 - RDS_AF_FM_BASE) : RDS_AF_FILLER;
				b3 = (af1 << 8) | af2;
			}
			
			/* Advance segment */
			rds->af_segment++;
			/* Reset segment after all pairs sent */
			if (rds->af_segment > (e->count / 2) + 1) rds->af_segment = 0;
			
		} else if (e->type == RDS_AF_TYPE_LF_MF) {
			/* LF/MF: Stream of [250, code] */
			int idx = rds->af_segment % e->count;
			uint8_t code = 0;
			
			if (e->list[idx].flags & RDS_AF_FLAG_LF) {
				code = (e->list[idx].freq - 144) / 9;
			} else { /* MF */
				int is_us = RDS_IS_RBDS(rds->ecc);
				code = 16 + (e->list[idx].freq - (is_us?540:531)) / (is_us?10:9);
			}
			b3 = (RDS_AF_LF_MF_FOLLOWS << 8) | code;
			
			rds->af_segment = (rds->af_segment + 1) % e->count;
			
		} else {
			/* Method B: Pairs [Tx, AF] */
			/* Segment 0: Header [Count, Tx] */
			if (rds->af_segment == 0) {
				uint8_t count_code = RDS_AF_NO_AF + e->count;
				uint8_t tx_code = (e->tx_freq >= RDS_AF_FM_BASE) ? (e->tx_freq - RDS_AF_FM_BASE) : 0;
				b3 = (count_code << 8) | tx_code;
			} else {
				/* Segments 1..N: Pairs [Tx, AFn] */
				int idx = rds->af_segment - 1;
				if (idx < e->count) {
					uint8_t tx_code = (e->tx_freq >= RDS_AF_FM_BASE) ? (e->tx_freq - RDS_AF_FM_BASE) : 0;
					uint8_t af_code = (e->list[idx].freq >= RDS_AF_FM_BASE) ? (e->list[idx].freq - RDS_AF_FM_BASE) : 0;
					b3 = (tx_code << 8) | af_code;
				} else {
					b3 = RDS_AF_NO_AF_PAIR;
				}
			}
			
			rds->af_segment++;
			
			/* Check if we finished this Method B list */
			if (rds->af_segment > e->count) {
				rds->af_segment = 0;
				
				/* Advance to NEXT Method B entry in the table */
				/* We need to find the next entry with the same PI */
				int current_idx = -1;
				
				/* 1. Find current index */
				for(int i=0; i<rds->af_table.count; i++) {
					if (&rds->af_table.entries[i] == e) {
						current_idx = i;
						break;
					}
				}
				
				/* 2. Find next valid Method B entry (wrapping around) */
				if (current_idx >= 0) {

					
					/* Search forward from next index */
					for(int i=1; i<=rds->af_table.count; i++) {
						int next_idx = (current_idx + i) % rds->af_table.count;
						if (rds->af_table.entries[next_idx].type == RDS_AF_TYPE_METHOD_B && 
							rds->af_table.entries[next_idx].pi == rds->pi) {
							/* Found next list!
							   We need to persist this selection for the next call.
							   However, rds_build_group_0a determines 'e' by searching from 0.
							   To fix this, we need state. 
							   Let's use rds->af_list_index to track which Method B list we are serving.
							*/
							rds->af_list_index = next_idx;
							break;
						}
					}
				}
			}
		}
	} else {
		/* No AF */
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
	uint32_t blocks[4];
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
	uint32_t blocks[4];
	uint16_t b2;
	int seg = rds->rt_segment;
	int pos = seg * 4;
	
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
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	rds->rt_segment = (seg + 1) & RDS_RT_SEG_MASK;
}

/* Build Group 2B: RadioText (32 chars, PI repeat in Block C)
 * IEC 62106 S6.1.5.3 - Version B for improved mobile reception
 * Block C contains PI repeat; Block D contains 2 RT chars per segment */
static void rds_build_group_2b(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4];
	uint16_t b2;
	int seg = rds->rt_segment;
	int pos = seg * 2;  /* 2 chars per segment for 2B */
	
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
	
	/* Block D: RT chars (2 per segment, max 32 chars) */
	uint16_t rt_chars = ((uint8_t)rds->rt[pos] << 8) | (uint8_t)rds->rt[pos+1];
	blocks[3] = rds_build_block(rt_chars, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
	
	rds->rt_segment = (seg + 1) & RDS_RT_SEG_MASK;
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
	uint32_t blocks[4];
	uint16_t b2, b3, b4;
	
	/* SLC Variant Sequence: cycle through active variants
	 * IEC 62106 Table 9:
	 *   0 = ECC (Extended Country Code)
	 *   3 = Language Identification Code (LIC)
	 * Other variants (TMC, EWS) can be added as needed */
	static const int slc_variants[] = {
		RDS_1A_VARIANT_ECC,
		RDS_1A_VARIANT_LANGUAGE
	};
	static const int slc_variant_count = sizeof(slc_variants) / sizeof(slc_variants[0]);
	
	int variant = slc_variants[rds->slc_variant % slc_variant_count];
	
	/* Block A: PI */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 1A + TP + PTY 
	 * Bits 4-0: Radio Paging codes (set to 0) */
	b2 = (RDS_GROUP_1A << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: LA + Variant + Payload (IEC 62106 Table 9) */
	switch (variant) {
	case RDS_1A_VARIANT_ECC:
		/* Variant 0: Extended Country Code
		 * Bits 15: LA (Linkage Actuator)
		 * Bits 14-12: Variant (0)
		 * Bits 11-8: Paging (0)
		 * Bits 7-0: ECC */
		b3 = (RDS_1A_VARIANT_ECC << RDS_1A_VARIANT_SHIFT) |
		     (rds->ecc & 0xFF);
		break;
		
	case RDS_1A_VARIANT_LANGUAGE:
		/* Variant 3: Language Identification Code (LIC)
		 * TEF6686 displays this as "LIC" on screen
		 * Bits 14-12: Variant (3)
		 * Bits 7-0: Language code (0-127) */
		b3 = (RDS_1A_VARIANT_LANGUAGE << RDS_1A_VARIANT_SHIFT) |
		     (rds->language & 0xFF);
		break;
		
	default:
		/* Fallback to ECC */
		b3 = (RDS_1A_VARIANT_ECC << RDS_1A_VARIANT_SHIFT) |
		     (rds->ecc & 0xFF);
		break;
	}
	blocks[2] = rds_build_block(b3, RDS_OFFSET_C);
	
	/* Block D: PIN (Programme Item Number) - for THIS service (current PI)
	 * IEC 62106:2015 S6.1.5.2
	 * Format: Day (5 bits) | Hour (5 bits) | Minute (6 bits)
	 * Day = day of month 1-31 (0 = PIN not used), receiver uses CT for month */
	b4 = ((rds->pin_day & 0x1F) << RDS_PIN_DAY_SHIFT) |
	     ((rds->pin_hour & 0x1F) << RDS_PIN_HOUR_SHIFT) |
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
 *   Block B: Group 1B + TP + PTY + Radio Paging (0)
 *   Block C: PI repeat (offset C')
 *   Block D: PIN (Day/Hour/Minute)
 */
static void rds_build_group_1b(rds_encoder_t *rds, uint8_t *group)
{
	uint32_t blocks[4];
	uint16_t b2, b4;
	
	/* Block A: PI code */
	blocks[0] = rds_build_block(rds->pi, RDS_OFFSET_A);
	
	/* Block B: Group 1B + TP + PTY
	 * Bits 4-0: Radio Paging codes (set to 0) */
	b2 = (RDS_GROUP_1B << RDS_B2_GROUP_SHIFT) |
	     (rds->tp << RDS_B2_TP_BIT) |
	     (rds->pty << RDS_B2_PTY_SHIFT);
	blocks[1] = rds_build_block(b2, RDS_OFFSET_B);
	
	/* Block C: PI repeat (offset C' for B-version groups) */
	blocks[2] = rds_build_block(rds->pi, RDS_OFFSET_Cp);
	
	/* Block D: PIN (Programme Item Number) - for THIS service
	 * Same format as Group 1A Block D */
	b4 = ((rds->pin_day & 0x1F) << RDS_PIN_DAY_SHIFT) |
	     ((rds->pin_hour & 0x1F) << RDS_PIN_HOUR_SHIFT) |
	     (rds->pin_minute & RDS_PIN_MINUTE_MASK);
	blocks[3] = rds_build_block(b4, RDS_OFFSET_D);
	
	/* Pack into bytes using shared function */
	rds_group_pack(blocks, group, 0);
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
	uint32_t blocks[4];
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
	uint32_t blocks[4];
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
	uint32_t blocks[4];
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
		b3 = ((eon->pin_day & 0x1F) << RDS_PIN_DAY_SHIFT) |
		     ((eon->pin_hour & 0x1F) << RDS_PIN_HOUR_SHIFT) |
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
	uint32_t blocks[4];
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
 * RDS GROUP SCHEDULER (IEC 62106)
 * ============================================================
 * 
 * MANDATORY VS OPTIONAL GROUPS:
 *   Only Group 0 (0A or 0B) is MANDATORY for RDS transmission.
 *   All other groups are OPTIONAL and should only transmit if
 *   their data is configured. Unused slots go to Group 0.
 *
 * A VS B VERSION SELECTION (per group type):
 *   - Version A: Block C carries group-specific data (more capacity)
 *   - Version B: Block C carries PI repeat (faster station ID)
 *   
 *   Group 0: 0A if AF list, 0B if no AF (use_0b flag)
 *   Group 1: 1A if ECC/Language, 1B if PIN only (use_1b flag)
 *   Group 2: 2A for 64-char RT, 2B for 32-char faster (use_2b flag)
 *
 *   IMPORTANT: Never mix A and B for the SAME group type.
 *              Different group types CAN use different versions.
 *
 * SCHEDULING PRIORITY:
 *   1. CT (Group 4A) - Time-triggered at minute boundary
 *   2. Group 0 (PS) - Mandatory, fills most slots
 *   3. Group 2 (RT) - If RadioText configured
 *   4. Group 1 (ECC/PIN) - If ECC/Language/PIN configured  
 *   5. Group 10 (PTYN) - If PTYN configured
 *   6. Group 14 (EON) - If EON enabled with entries
 *
 * TIMING (at 1187.5 bps, ~11.4 groups/sec):
 *   - 5-group cycle: ~0.44 sec
 *   - 20 groups: ~1.75 sec (Group 1 interval)
 *   - 40 groups: ~3.5 sec (Group 10 interval)
 *   - 50 groups: ~4.4 sec (Group 14 interval)
 * ============================================================ */
static void rds_generate_group(rds_encoder_t *rds)
{
	/* --------------------------------------------------------
	 * 1. TIME-TRIGGERED: Group 4A (Clock-Time)
	 * --------------------------------------------------------
	 * CT is transmitted at minute boundaries, highest priority.
	 * IEC 62106 recommends CT accuracy within +/-100ms of UTC. */
	if (rds->ct_enabled) {
		time_t now = time(NULL) + rds->ct_time_offset;
		struct tm *t_now = gmtime(&now);
		int current_minute = t_now->tm_min;
		
		struct tm *t_last = gmtime(&rds->last_ct_minute);
		if (current_minute != t_last->tm_min) {
			rds_build_group_4a(rds, rds->group_buffer);
			rds->last_ct_minute = now;
			rds->group_sequence++;
			rds->group_bit_pos = 0;
			return;
		}
	}
	
	/* --------------------------------------------------------
	 * 2. DATA-DRIVEN SCHEDULING
	 * --------------------------------------------------------
	 * Only transmit optional groups if their data is configured.
	 * Group 0 fills all remaining slots (mandatory). */
	
	/* Check what optional data is configured */
	int has_rt = (rds->rt[0] != ' ' && rds->rt[0] != '\r' && rds->rt[0] != '\0');
	int has_slc = (rds->ecc != 0 || rds->language != 0 || rds->pin_day != 0);
	int has_ptyn = (rds->ptyn[0] != ' ' && rds->ptyn[0] != '\0');
	int has_eon = (rds->eon_enabled && rds->eon_tx_count > 0);
	
	uint64_t seq = rds->group_sequence++;
	int cycle_pos = seq % 5;  /* 5-group cycle */
	
	/* --------------------------------------------------------
	 * Slot 4: Group 2 (RadioText) - IF configured
	 * --------------------------------------------------------
	 * 2A: 64 chars, 4 chars/group, 16 groups to complete
	 * 2B: 32 chars, 2 chars/group, 16 groups to complete (faster PI)
	 * -------------------------------------------------------- */
	if (cycle_pos == 4 && has_rt) {
		if (rds->use_2b)
			rds_build_group_2b(rds, rds->group_buffer);
		else
			rds_build_group_2a(rds, rds->group_buffer);
	}
	/* --------------------------------------------------------
	 * Slots 0-3: Mix of Group 0 and other optional groups
	 * -------------------------------------------------------- */
	else {
		/* Every 20 groups: Group 1 (ECC/Language/PIN) - IF configured
		 * 1A: Full SLC data (ECC, Language) + PIN
		 * 1B: PIN only with PI repeat (faster station ID) */
		if ((seq % 20) == 0 && has_slc) {
			if (rds->use_1b)
				rds_build_group_1b(rds, rds->group_buffer);
			else
				rds_build_group_1a(rds, rds->group_buffer);
		}
		/* Every 40 groups: Group 10A (PTYN) - IF configured */
		else if ((seq % 40) == 10 && has_ptyn) {
			rds_build_group_10a(rds, rds->group_buffer);
		}
		/* Every 50 groups: Group 14A (EON) - IF configured */
		else if ((seq % 50) == 25 && has_eon) {
			rds_build_group_14a(rds, rds->group_buffer);
		}
		/* --------------------------------------------------------
		 * Default: Group 0 (MANDATORY - fills all unused slots)
		 * --------------------------------------------------------
		 * 0A: PS name + AF list (use when AF available)
		 * 0B: PS name + PI repeat (use when no AF, better mobile)
		 * -------------------------------------------------------- */
		else {
			if (rds->use_0b)
				rds_build_group_0b(rds, rds->group_buffer);
			else
				rds_build_group_0a(rds, rds->group_buffer);
		}
	}
	
	rds->group_bit_pos = 0;
}

/* Get next bit from current group */
static int rds_get_next_bit(rds_encoder_t *rds)
{
	int byte_pos = rds->group_bit_pos / 8;
	int bit_pos = 7 - (rds->group_bit_pos % 8);
	int bit;
	
	if (rds->group_bit_pos >= 104) {
		rds_generate_group(rds);
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
	
	/* Set RadioText (pad with spaces) */
	memset(rds->rt, ' ', 64);
	rds->rt[64] = '\0';
	if (rt) {
		int len = strlen(rt);
		if (len > 64) len = 64;
		memcpy(rds->rt, rt, len);
		/* Add CR terminator if space */
		if (len < 64)
			rds->rt[len] = '\r';
	}
	
	/* Calculate phase steps */
	rds->phasestep = 2.0 * M_PI * RDS_SUBCARRIER / samplerate;
	rds->bit_phasestep = RDS_BITRATE / samplerate;
	
	/* Initialize differential encoder */
	rds->last_diff_bit = 0;
	memset(rds->bit_history, 0, sizeof(rds->bit_history));
	
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

	/* Generate RRC biphase waveform at runtime */
	if (rds_generate_biphase_waveform(&rds->waveform_biphase) < 0) {
		LOGP(DRADIO, LOGL_ERROR, "Failed to generate RDS biphase waveform\n");
		return -1;
	}
	

	
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
	
	for (i = 0; i < num; i++) {
		/* Advance bit timing */
		rds->bit_phase += rds->bit_phasestep;
		while (rds->bit_phase >= 1.0) {
			rds->bit_phase -= 1.0;
			
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
		double subcarrier = sample_val * sin(rds_phase);
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
	
	/* Debug: log occasionally to confirm RDS is being added */
	if (++debug_count >= 100) {
		/* LOGP(DRADIO, LOGL_DEBUG, "RDS: injecting (max=%.4f, phasestep=%.6f, samples=%d)\n",
		     max_injection, pilot_phasestep, num); */
		debug_count = 0;
	}
	
	/* Save phase state */
	rds->phase = phase;
}

void rds_set_radiotext(rds_encoder_t *rds, const char *rt)
{
	if (!rt) return;
	memset(rds->rt, ' ', 64);
	int len = strlen(rt);
	if (len > 64) len = 64;
	memcpy(rds->rt, rt, len);
	if (len < 64) rds->rt[len] = '\r';
	rds->rt[64] = '\0';
	
	/* Toggle A/B flag to trigger receivers to clear display */
	rds->rt_ab = !rds->rt_ab;
	rds->rt_segment = 0;
}

void rds_set_ta(rds_encoder_t *rds, int ta)
{
	rds->ta = ta ? 1 : 0;
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
		
		
		/* Group 0A only: Decode Alternative Frequencies from Block C
		 *
		 * AF encoding (EN 50067 S3.2.1.6, IEC 62106 Table 11):
		 *   224-249: AF count code (N = code - 224, 1-25 AFs follow)
		 *   1-204:   FM frequency (87.6-107.9 MHz, code = (freq-87.5)*10)
		 *   205:     Filler code (ignore)
		 *   250:     LF/MF frequency follows (next byte is LF/MF table)
		 *   0,251-255: Reserved (ignore)
		 *
		 * Method A (simple list): All AFs are equivalent alternatives
		 *   [count, AF1] [AF2, AF3] [AF4, AF5] ...
		 *
		 * Method B (paired, for >25 AFs/regional): Tuning freq in each pair
		 *   [count, tuned] [freq1, freq2] [freq1, freq2] ...
		 *   - Ascending (freq1 < freq2): Same content
		 *   - Descending (freq1 > freq2): Regional variant
		 *
		 * NOTE: Method B is detected heuristically by receivers (no control code).
		 * This decoder stores all AFs as a simple list without method distinction.
		 */
		if (version == 0 && (rds->group_mask & (1<<2))) {
			/* AF processing requires valid PI - without it, we can't
			 * associate the AF list with any station */
			if (rds->pi == 0) {
				/* No valid PI received yet - ignore AF data */
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: Ignoring AF - no valid PI\n");
			} else {
			uint8_t af1 = (b3 >> 8) & 0xFF;
			uint8_t af2 = b3 & 0xFF;
			int block_c_reliable = (rds->block_status[2] <= 1);
			
			if (af1 >= 224 && af1 <= 249) {
				/* AF count code: start of new list */
				if (block_c_reliable) {
					/* Get/Create Method A and LF/MF entries to reset them */
					rds_af_entry_t *ea = rds_af_get_entry(rds, &rds->af_table, rds->pi, RDS_AF_TYPE_METHOD_A, 0);
					rds_af_entry_t *el = rds_af_get_entry(rds, &rds->af_table, rds->pi, RDS_AF_TYPE_LF_MF, 0);
					
					if (ea) ea->count = 0;
					if (el) el->count = 0;
					
					/* Store first frequency - candidate for Method B tuning freq */
					if (RDS_VALID_AF_CODE(af2) && ea) {
						ea->list[ea->count].freq = RDS_AF_FM_BASE + af2;
						ea->list[ea->count].flags = 0;
						ea->count++;
					}
				}
			} else if (af1 == RDS_AF_LF_MF_FOLLOWS) {
				/* LF/MF frequency indicator (code 250) */
				int is_us = RDS_IS_RBDS(rds->ecc);
				int spacing = is_us ? 10 : 9;
				int mf_base = is_us ? 380 : 387;
				rds_af_entry_t *el = rds_af_get_entry(rds, &rds->af_table, rds->pi, RDS_AF_TYPE_LF_MF, 0);
				
				if (el && block_c_reliable && rds->pi != 0) {
					uint16_t freq = 0;
					uint8_t flags = 0;
					
					if (af2 >= 1 && af2 <= 15) {
						freq = 144 + af2 * 9;
						flags = RDS_AF_FLAG_LF;
					} else if (af2 >= 16 && af2 <= 135) {
						freq = mf_base + af2 * spacing;
						flags = RDS_AF_FLAG_MF;
					}
					
					if (freq && el->count < RDS_AF_MAX_ITEMS) {
						el->list[el->count].freq = freq;
						el->list[el->count].flags = flags;
						el->count++;
						
						if (rds->verbose)
							LOGP(DRADIO, LOGL_INFO, "RDS 0A: LF/MF AF received: %d kHz (%s band)\n",
							     freq, (flags & RDS_AF_FLAG_LF) ? "LF" : "MF");
					}
				}
			} else if (block_c_reliable && af1 != 0 && af1 != RDS_AF_FILLER && af1 != RDS_AF_NO_AF) {
				/* Regular FM AF pair - detect Method B or add to Method A */
				uint16_t freq1 = RDS_VALID_AF_CODE(af1) ? RDS_AF_FM_BASE + af1 : 0;
				uint16_t freq2 = RDS_VALID_AF_CODE(af2) ? RDS_AF_FM_BASE + af2 : 0;
				
				/* Check Method B: compare against first freq of Method A list (tuning freq) */
				rds_af_entry_t *ea = rds_af_get_entry(rds, &rds->af_table, rds->pi, RDS_AF_TYPE_METHOD_A, 0);
				uint16_t tx_freq = (ea && ea->count > 0) ? ea->list[0].freq : 0;
				
				int is_method_b = (tx_freq && (freq1 == tx_freq || freq2 == tx_freq));
				
				if (is_method_b) {
					/* Method B: route to sub-entry for this transmitter */
					rds->af_method_b = 1; /* Keep flag for debug/status */
					rds_af_entry_t *eb = rds_af_get_entry(rds, &rds->af_table, rds->pi, RDS_AF_TYPE_METHOD_B, tx_freq);
					
					if (eb) {
						uint16_t alt = (freq1 == tx_freq) ? freq2 : freq1;
						if (alt) {
							int idx;
							
							/* Find slot: new or replace based on priority (Internal List Logic) */
							if (eb->count < RDS_AF_MAX_ITEMS) {
								/* Free slot available */
								idx = eb->count++;
							} else {
								/* Smart replacement: Invalid then Oldest */
								int replace_idx = 0;
								uint32_t oldest_invalid_time = UINT32_MAX;
								uint32_t oldest_time = UINT32_MAX;
								int oldest_idx = 0;
								
								for (int j = 0; j < eb->count; j++) {
									if (eb->list[j].entry_time < oldest_time) {
										oldest_time = eb->list[j].entry_time;
										oldest_idx = j;
									}
									if (eb->list[j].priority == RDS_AF_PRIORITY_INVALID &&
									    eb->list[j].entry_time < oldest_invalid_time) {
										oldest_invalid_time = eb->list[j].entry_time;
										replace_idx = j;
									}
								}
								
								if (oldest_invalid_time == UINT32_MAX)
									replace_idx = oldest_idx;
								
								idx = replace_idx;
								if (rds->debug)
									LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: Method B replacing item %d (%.1f MHz)\n",
									     idx, eb->list[idx].freq / 10.0);
							}
							
							/* Update Item */
							eb->list[idx].freq = alt;
							eb->list[idx].entry_time = rds->af_update_counter++;
							
							/* Determine Priority (Regional/Order) */
							if (idx == 0) {
								eb->list[idx].flags = 0; /* Clear Regional bit */
								eb->list[idx].priority = RDS_AF_PRIORITY_SAME;
							} else {
								uint16_t prev_alt = eb->list[idx-1].freq;
								int is_ascending = (alt > prev_alt);
								/* Note: We use existing buffer order. ideally idx order might be scrambled.
								 * Assuming append/linear behavior for detection. */
								int prev_was_reg = (eb->list[idx-1].flags & RDS_AF_FLAG_REGIONAL);
								
								if (is_ascending) {
									eb->list[idx].flags &= ~RDS_AF_FLAG_REGIONAL;
									if (prev_was_reg && idx > 1) eb->list[idx].priority = RDS_AF_PRIORITY_INVALID;
									else eb->list[idx].priority = RDS_AF_PRIORITY_SAME;
								} else {
									/* Descending or Equal -> Regional */
									eb->list[idx].flags |= RDS_AF_FLAG_REGIONAL;
									if (!prev_was_reg && idx > 1) eb->list[idx].priority = RDS_AF_PRIORITY_INVALID;
									else eb->list[idx].priority = RDS_AF_PRIORITY_REGIONAL;
								}
							}
							
							if (rds->verbose) {
								int is_reg = (eb->list[idx].flags & RDS_AF_FLAG_REGIONAL);
								int is_inv = (eb->list[idx].priority == RDS_AF_PRIORITY_INVALID);
								LOGP(DRADIO, LOGL_INFO, "RDS 0A: Method B update: TX=%.1f AF=%.1f %s%s\n",
								     tx_freq/10.0, alt/10.0, 
								     is_reg ? "(R)" : "",
								     is_inv ? " [INVALID ORDER]" : "");
							}
						}
					}
				} else {
					/* Method A: Append to list */
					if (ea && ea->count < RDS_AF_MAX_ITEMS) {
						if (freq1) ea->list[ea->count++].freq = freq1;
						if (freq2 && ea->count < RDS_AF_MAX_ITEMS) ea->list[ea->count++].freq = freq2;
					}
				}
			}
			
			if (rds->debug && af1 >= 1 && af1 <= 204)
				LOGP(DRADIO, LOGL_DEBUG, "RDS 0A: AF codes: %d, %d (Method %c)\n",
				     af1, af2, (rds->af_method_b && rds_af_get_entry(rds, &rds->af_table, rds->pi, RDS_AF_TYPE_METHOD_B, 0)) ? 'B' : 'A');
			}  /* end else (pi valid) */
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
		 */
		
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
				/* Extended Country Code */
				rds->ecc = payload & 0xFF;
				rds->ecc_status = rds->block_status[2];
				if (rds->debug)
					LOGP(DRADIO, LOGL_DEBUG, "RDS 1A: ECC=0x%02X CC=%d LA=%d\n",
					     rds->ecc, cc, rds->linkage_actuator);
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 1A: Extended Country Code=%s, "
					     "Country Code=%d, Linkage Actuator=%d. "
					     "Summary: Country=\"%s\", Programme Linkage=%s\n",
					     rds_get_ecc_name(rds->ecc), cc, rds->linkage_actuator,
					     rds_get_country_code(cc, rds->ecc),
					     rds->linkage_actuator ? "Yes" : "No");
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
			/* Variants TMC_ID, PAGER, BCAST, EWS not decoded yet */
			}
		}
	}
	else if (group_type == 2) {
		/* Group 2A/2B: RadioText - see RDS_2A_* macros in rds.h */
		int ab_flag = (b2 & RDS_2A_AB_MASK) >> RDS_2A_AB_BIT;
		int seg = b2 & RDS_2A_SEG_MASK;
		
		/* DEBUG: Compact codes */
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 2%c: AB=%d seg=%d\n",
			     version ? 'B' : 'A', ab_flag, seg);
		
		if (ab_flag != rds->rt_ab) {
			rds->rt_ab = ab_flag;
			rds->rt_segments = 0;
			memset(rds->rt, ' ', 64);
			if (rds->verbose)
				LOGP(DRADIO, LOGL_INFO, "RDS 2%c: Text Change Flag toggled to %c "
				     "(new RadioText message starting)\n",
				     version ? 'B' : 'A', ab_flag ? 'B' : 'A');
		}
		
		if (version == 0) {
			int pos = seg * 4;
			if (pos <= 60) {
				rds->rt[pos] = (b3 >> 8) & 0xFF;
				rds->rt[pos+1] = b3 & 0xFF;
				rds->rt[pos+2] = (b4 >> 8) & 0xFF;
				rds->rt[pos+3] = b4 & 0xFF;
				rds->rt_status[pos] = rds->block_status[2];
				rds->rt_status[pos+1] = rds->block_status[2];
				rds->rt_status[pos+2] = rds->block_status[3];
				rds->rt_status[pos+3] = rds->block_status[3];
				/* Verbose: Log RadioText 2A segment reception progress */
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 2A: RadioText segment %d/16, "
					     "chars %d-%d received\n", seg + 1, pos, pos + 3);
			}
		} else {
			int pos = seg * 2;
			if (pos <= 62) {
				rds->rt[pos] = (b4 >> 8) & 0xFF;
				rds->rt[pos+1] = b4 & 0xFF;
				rds->rt_status[pos] = rds->block_status[3];
				rds->rt_status[pos+1] = rds->block_status[3];
				/* Verbose: Log RadioText 2B segment reception progress */
				if (rds->verbose)
					LOGP(DRADIO, LOGL_INFO, "RDS 2B: RadioText segment %d/16, "
					     "chars %d-%d received\n", seg + 1, pos, pos + 1);
			}
		}
		rds->rt_segments |= (1 << seg);
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
		uint16_t aid = b4;
		
		if (rds->debug)
			LOGP(DRADIO, LOGL_DEBUG, "RDS 3A: AppGroup=%d%c AID=%04X\n",
			     app_group >> 1, (app_group & 1) ? 'B' : 'A', aid);
		
		if (rds->verbose && (rds->group_mask & (1<<3))) {
			const char *oda_name = "Unknown";
			/* Common ODA AIDs (EBU ODA Registry) */
			switch (aid) {
			case 0x0093: oda_name = "DAB Cross-Reference"; break;
			case 0x4BD7: oda_name = "RadioText Plus (RT+)"; break;
			case 0x6552: oda_name = "Enhanced RadioText (eRT)"; break;
			case 0xCD46: oda_name = "TMC Alert-C"; break;
			case 0xFF7F: oda_name = "Station Logo"; break;
			case 0xFF70: oda_name = "Internet Connection"; break;
			}
			LOGP(DRADIO, LOGL_INFO, "RDS 3A: ODA Identification - "
			     "AID=0x%04X (%s), carried in Group %d%c\n",
			     aid, oda_name,
			     app_group >> 1, (app_group & 1) ? 'B' : 'A');
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
int rds_get_pi(rds_decoder_t *rds, uint16_t *pi)
{
	if (rds->groups_received > 0) {
		*pi = rds->pi;
		return 1;
	}
	return 0;
}

int rds_get_ps(rds_decoder_t *rds, char *ps)
{
	if (rds->ps_segments == 0x0F) {
		memcpy(ps, rds->ps, 9);
		return 1;
	}
	return 0;
}

int rds_get_rt(rds_decoder_t *rds, char *rt)
{
	if (rds->rt_segments) {
		memcpy(rt, rds->rt, 65);
		return 1;
	}
	return 0;
}

void rds_decoder_status(rds_decoder_t *rds)
{
	char ps_sanitized[9];
	char rt_sanitized[65];
	int i;
	
	if (rds->blocks_received < 10) return;
	
	/* Helper: status character */
	#define STATUS_CHAR(s) ((s) == RDS_STATUS_VALID ? '+' : \
	                        (s) == RDS_STATUS_CORRECTED ? '?' : \
	                        (s) == RDS_STATUS_ERROR ? 'X' : '_')
	
	/* Sanitize PS */
	memcpy(ps_sanitized, rds->ps, 9);
	for (i=0; i<8; i++) {
		if (ps_sanitized[i] < 0x20 || ps_sanitized[i] > 0x7E) ps_sanitized[i] = ' ';
	}
	ps_sanitized[8] = '\0';
	
	/* Sanitize RT */
	memcpy(rt_sanitized, rds->rt, 65);
	for (i=0; i<64; i++) {
		if (rt_sanitized[i] < 0x20 || rt_sanitized[i] > 0x7E) rt_sanitized[i] = ' ';
	}
	rt_sanitized[64] = '\0';
	/* Trim trailing spaces */
	for (i=63; i>=0 && rt_sanitized[i]==' '; i--) rt_sanitized[i] = '\0';
	
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
		     pi_str, STATUS_CHAR(rds->pi_status), ps_sanitized,
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
	
	/* Alternative Frequencies (Unified) */
	if (rds->af_table.count > 0) {
		LOGP(DRADIO, LOGL_NOTICE, "Alternative Frequencies:\n");
		for (i = 0; i < rds->af_table.count; i++) {
			rds_af_entry_t *e = &rds->af_table.entries[i];
			if (e->count == 0) continue;
			
			/* Basic check: only show AFs matching current PI to avoid confusion, 
			 * or valid PIs if debugging multiple services */
			if (e->pi != rds->pi && rds->pi != 0) continue;
			
			char af_buf[256] = "";
			int pos = 0;
			
			/* Header */
			char header[64];
			if (e->type == RDS_AF_TYPE_METHOD_A)
				snprintf(header, sizeof(header), "  Method A");
			else if (e->type == RDS_AF_TYPE_METHOD_B)
				snprintf(header, sizeof(header), "  Method B (TX=%.1f)", e->tx_freq / 10.0);
			else if (e->type == RDS_AF_TYPE_LF_MF)
				snprintf(header, sizeof(header), "  LF/MF");
			else
				snprintf(header, sizeof(header), "  Unknown");

			/* List */
			for (int j = 0; j < e->count && pos < 240; j++) {
				if (e->type == RDS_AF_TYPE_LF_MF) {
					pos += snprintf(af_buf + pos, sizeof(af_buf) - pos, "%d(%s) ",
					                e->list[j].freq,
					                (e->list[j].flags & 0x02) ? "LF" : "MF");
				} else {
					pos += snprintf(af_buf + pos, sizeof(af_buf) - pos, "%.1f%s ",
					                e->list[j].freq / 10.0,
					                (e->list[j].flags & 0x01) ? "(R)" : "");
				}
			}
			LOGP(DRADIO, LOGL_NOTICE, "%-20s: %s\n", header, af_buf);
		}
	}
	
	/* Block statistics */
	LOGP(DRADIO, LOGL_NOTICE, "Groups=%d  Blocks: OK=%ld  Bad=%ld  Total=%ld\n",
	     rds->groups_received, rds->blocks_ok, rds->blocks_bad, rds->blocks_received);

	
	/* PTYN (if any bits received) */
	if (rds->ptyn_segments) {
		char ptyn_sanitized[9];
		char ptyn_status_str[9];
		
		memcpy(ptyn_sanitized, rds->ptyn, 9);
		for (i=0; i<8; i++) {
			if (ptyn_sanitized[i] < 0x20 || ptyn_sanitized[i] > 0x7E) ptyn_sanitized[i] = ' ';
			ptyn_status_str[i] = STATUS_CHAR(rds->ptyn_status[i]);
		}
		ptyn_sanitized[8] = '\0';
		ptyn_status_str[8] = '\0';
		
		LOGP(DRADIO, LOGL_NOTICE, "PTYN=\"%s\"\n", ptyn_sanitized);
		LOGP(DRADIO, LOGL_NOTICE, "      %s\n", ptyn_status_str);
	}
	
	/* RadioText (if any) with status beneath */
	if (rt_sanitized[0] != '\0') {
		char rt_status_str[65];
		int rt_len = strlen(rt_sanitized);
		for (i=0; i<rt_len; i++) {
			rt_status_str[i] = STATUS_CHAR(rds->rt_status[i]);
		}
		rt_status_str[rt_len] = '\0';
		LOGP(DRADIO, LOGL_NOTICE, "RT=\"%s\"\n", rt_sanitized);
		/*              RT=" = 4 chars padding */
		LOGP(DRADIO, LOGL_NOTICE, "    %s\n", rt_status_str);
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
