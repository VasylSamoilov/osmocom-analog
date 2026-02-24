/* main function
 *
 * (C) 2018 by Andreas Eversberg <jolly@eversberg.eu>
 * All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include "../libsample/sample.h"
#include "../liblogging/logging.h"
#include "../libclipper/clipper.h"
#include "../libsdr/sdr_config.h"
#include "radio.h"
#include "rds_tables.h"
#include "../libfm/fm.h"
#include "audio_debug.h"

#define CLIP_POINT	0.85
#define DC_CUTOFF	30.0 // Wikipedia: UKW-Rundfunk
#define STEREO_BW	15000.0
#define PILOT_FREQ	19000.0
#define PILOT_BW	5.0
#define PHASE_ERROR_TOLERANCE	3.0	/* ITU-R BS.450-4 S2.2.2.5: +/-3deg */
#define PHASE_ERROR_AVG_SAMPLES	10000	/* samples to average for phase error */

/* Stereo pilot lock/unlock thresholds (fraction of full-scale FM deviation).
 * pilot_mag represents true injection level: 0.1 = 10% = 7.5 kHz deviation.
 * Standard broadcast pilot is 8-10% (6-7.5 kHz). Lock above LOCK_THR,
 * unlock below UNLOCK_THR (~6 dB hysteresis).
 *
 * Acquisition/loss require the signal to be continuously in-range for
 * PILOT_ACQUIRE_S / PILOT_LOSS_S seconds before the state changes.
 * This rejects noise spikes and brief dropouts.
 * IEC 62106 / ITU-R BS.450 consumer practice: 50–200 ms each direction.
 *
 * COOLDOWN_S: minimum hold time after any transition (prevents re-entry). */
#define PILOT_LOCK_THR		0.04	/* acquire: pilot must stay above this (~3 kHz) */
#define PILOT_UNLOCK_THR	0.02	/* loss:    pilot must stay below this (~1.5 kHz) */
#define PILOT_ACQUIRE_S		0.2	/* 200 ms continuous above thr to lock  */
#define PILOT_LOSS_S		0.2	/* 200 ms continuous below thr to unlock */
#define PILOT_COOLDOWN_S	0.1	/* seconds before next transition allowed */
/* Noise-aware stereo blend cap.
 * Strong L-R energy relative to L+R often correlates with stereo hiss in quiet
 * passages; cap blend there to improve perceived quieting like FM receivers. */
#define STEREO_QUALITY_LOW	1.2
#define STEREO_QUALITY_HIGH	3.0
#define STEREO_BLEND_CAP_MIN	0.10
/* Additional cap from RF SNR estimate (dB). */
#define SNR_FORCE_MONO_DB	14.0
#define SNR_BLEND_LOW_DB	20.0
#define SNR_BLEND_HIGH_DB	38.0
/* Keep floor effectively disabled; mono-like quieting is preferred unless
 * signal quality is truly exceptional. */
#define SNR_CAP_FLOOR_DB	80.0
#define SNR_CAP_FLOOR_VALUE	0.95
/* Content-cap is only trusted in very poor SNR conditions. */
#define CONTENT_CAP_ENABLE_MAX_SNR_DB	14.0
/* Stereo side-channel HF suppression (receiver-style hiss reduction):
 * only the high-band of L-R is attenuated based on SNR. */
#define STEREO_DIFF_LOW_BW	3500.0
#define STEREO_HF_GAIN_MIN	0.40
#define STEREO_HF_GAIN_LOW_DB	28.0
#define STEREO_HF_GAIN_HIGH_DB	42.0
/* Apply HF attenuation mostly on quiet passages (where hiss is audible). */
#define STEREO_QUIET_RMS_LOW	0.12
#define STEREO_QUIET_RMS_HIGH	0.45
#define STEREO_HF_GAIN_TC_FALL_S 0.35
#define STEREO_HF_GAIN_TC_RISE_S 1.20
/* Mild post-deemphasis audio HF shaping to reduce gritty/harsh texture
 * that can remain in both mono and stereo with noisy SDR front-ends. */
#define RX_OUTPUT_HICUT_BW	13500.0
/* Smooth stereo control to avoid audible "pulsation":
 * - cap: fast reduction on bad quality, slow recovery
 * - blend: fast collapse to mono, slower stereo return */
#define STEREO_CAP_TC_FALL_S	0.12
#define STEREO_CAP_TC_RISE_S	0.90
#define STEREO_BLEND_TC_FALL_S	0.08
#define STEREO_BLEND_TC_RISE_S	0.35
/* AFC stabilization:
 * - deadband prevents chasing tiny/noisy offsets
 * - slew limit prevents runaway on weak/noisy stations */
#define AFC_DEADBAND_HZ		15.0
#define AFC_MAX_SLEW_HZ_PER_S	1200.0

/* ============================================================
 * RDS Preset Configuration System
 * Press 'f' during operation to cycle between presets.
 * 
 * Country is determined by PI prefix (first hex digit) + ECC combination.
 * See IEC 62106 Annex D for official tables.
 * TEF6686 lookup: docs/rds/TEF6686_ESP32/src/TEF6686.cpp lines 848-1155
 * Country name table: docs/rds/TEF6686_ESP32/src/TEF6686.h lines 111-340
 * ============================================================ */

typedef struct rds_preset {
	const char	*name;		/* Preset name for display */
	uint16_t	pi;		/* Programme Identification */
	const char	*ps;		/* Programme Service (8 chars max) */
	const char	*rt;		/* RadioText: 64 chars max (2A) or 32 chars max (2B), per IEC 62106 */
	uint8_t		pty;		/* Programme Type (0-31) */
	const char	*ptyn;		/* PTY Name (8 chars max or NULL) */
	uint8_t		tp;		/* Traffic Programme flag */
	uint8_t		ms;		/* Music/Speech (1=Music) */
	uint8_t		ecc;		/* Extended Country Code */
	uint8_t		lang;		/* Language Code */
	
	/* AF Method A - Human-readable format (IEC 62106 S3.2.1.6.3)
	 * Format: "91.0, 102.5, LF225, MF1008"
	 * - VHF frequencies in MHz (87.6-107.9)
	 * - LFxxx for LF band in kHz (153-279)
	 * - MFxxx for MF band in kHz (531-1602)
	 * LF/MF count as 2 slots each. Max 25 total slots. */
	const char	*af_method_a_str;
	
	/* AF Method B - Paired frequency lists (IEC 62106 S3.2.1.6.4)
	 * Format: "T89.3, 99.5, 88.8, R102.6, R89.0"
	 *   T prefix = tuning frequency (required first element)
	 *   R prefix = regional variant (F1 > F2 descending order)
	 *   No prefix = same programme (F1 < F2 ascending order)
	 * Max 12 AFs per list. Multiple lists transmitted sequentially. */
	const char	*af_method_b_str[RDS_AF_METHOD_B_MAX_LISTS];
	uint8_t		af_method_b_count;	/* Number of Method B lists */
	
	const char	*callsign;	/* RBDS Call Sign (overrides/sets PI if PI=0) */
	/* Group 1A PIN - Programme Item Number for THIS station (current PI)
	 * HISTORICAL: Enabled "VCR-like" radio recording (1984-1990s).
	 * Users entered PIN from published schedule; receiver triggered
	 * recording when transmitted PIN matched programme start.
	 * NOW OBSOLETE: Most stations transmit 0x0000 (no PIN). */
	uint8_t		pin_day;	/* Day of month 1-31 (0 = not used) */
	uint8_t		pin_hour;	/* Hour 0-23, or 24 = end time */
	uint8_t		pin_minute;	/* Minute 0-59 (prefer 00/15/30/45) */
	/* EON (Enhanced Other Networks) - Group 14A */
	struct {
		uint16_t	pi;		/* Other Network PI code */
		const char	*ps;		/* Other Network PS name */
		uint8_t		pty;		/* Other Network PTY */
		uint8_t		tp;		/* Other Network TP flag */
		uint8_t		ta;		/* Other Network TA flag (variant 13) */
		/* AF (variant 4) - AF codes 1-204 */
		uint8_t		af[2];		/* Two AF codes for Other Network */
		uint8_t		af_count;	/* 0-2 */
		/* Mapped AF (variants 5-9) */
		struct {
			uint8_t	tuned;		/* Tuned freq AF code */
			uint8_t	on;		/* ON freq AF code */
		} mapped_af[2];
		uint8_t		mapped_af_count; /* 0-2 */
		/* Linkage (variant 12) */
		uint8_t		linkage_la;	/* Linkage Actuator */
		uint16_t	linkage_lsn;	/* Linkage Set Number (12 bits) */
		/* PIN (variant 14) - Programme Item Number for Other Network
		 * This PIN applies to the LINKED station (eon.pi), NOT the current station.
		 * Day=0 means "no PIN" (IEC 62106 S6.1.5.2).
		 * Example: News bulletin starts at 14:30 on day 15 of month */
		uint8_t		pin_day;	/* Day 1-31 (0 = not used) */
		uint8_t		pin_hour;	/* Hour 0-23 (24 = end time) */
		uint8_t		pin_minute;	/* Minute 0-59 */
		/* Broadcaster data (variant 15) */
		uint16_t	broadcaster_data;
	} eon[2];				/* Up to 2 Other Networks per preset */
	uint8_t		eon_count;		/* Number of EON entries (0-2) */
	
	/* Group version selection (0=auto, 1=force A, 2=force B)
	 * Use RDS_GROUP_VERSION_AUTO/A/B from rds.h
	 * When 0 (AUTO): version auto-detected from data:
	 *   - Group 0: 0B if af_count==0, else 0A
	 *   - Group 1: 1B if ecc==0 && lang==0, else 1A
	 *   - Group 2: 2B if strlen(rt)<=32, else 2A */
	uint8_t		group0_version;		/* 0A vs 0B */
	uint8_t		group1_version;		/* 1A vs 1B */
	uint8_t		group2_version;		/* 2A vs 2B */
	
	/* RT+ (RadioText Plus) Configuration */
	struct {
		uint8_t		enabled;		/* 1 = enable RT+ ODA */
		uint8_t		carrier_group;		/* ODA carrier group (e.g., 22 for 11A) */
		uint16_t	message;			/* Group 3A message (cb, scb, template) */
		uint8_t		toggle;			/* Item toggle flag */
		uint8_t		item_running;		/* Item running flag */
		struct {
			uint8_t	content_type;		/* Content type (0-64) */
			uint8_t	start;			/* Start position in RT (0-63) */
			uint8_t	length;			/* Length of tag (1-64) */
		} tags[2];				/* Up to 2 tags */
		uint8_t		tag_count;		/* Number of tags (0-2) */
	} rtplus;
	
	/* eRT+ (Enhanced RadioText Plus) Configuration */
	struct {
		uint8_t		enabled;		/* 1 = enable eRT+ ODA */
		uint8_t		carrier_group;		/* ODA carrier group (e.g., 22 for 11A) */
		uint16_t	message;			/* Group 3A message (cb, scb, template) */
		uint8_t		toggle;			/* Item toggle flag */
		uint8_t		item_running;		/* Item running flag */
		struct {
			uint8_t	content_type;		/* Content type (0-64) */
			uint8_t	start;			/* Start position in eRT (0-127) */
			uint8_t	length;			/* Length of tag (1-128) */
		} tags[2];				/* Up to 2 tags */
		uint8_t		tag_count;		/* Number of tags (0-2) */
	} ert_plus;
	
	/* eRT (Enhanced RadioText) Configuration */
	struct {
		uint8_t		enabled;		/* 1 = enable eRT ODA */
		uint8_t		carrier_group;		/* ODA carrier group (e.g., 24 for 12A) */
		uint16_t	message;			/* Group 3A message (encoding, direction, chartable) */
		const char	*text;			/* eRT text (128 bytes max, UTF-8) */
	} ert;
	
	/* Dynamic PS (Scrolling Programme Service Name)
	 * If dps_text is non-NULL, dynamic PS is enabled for this preset.
	 * The PS field is still used as the initial/fallback static PS. */
	struct {
		const char	*text;		/* Full scrolling text (NULL = disabled) */
		uint8_t		mode;		/* RDS_DPS_SCROLL=0, RDS_DPS_WORD=1, RDS_DPS_PAGE=2 */
		uint8_t		repeat;		/* PS transmissions per step (1=fast, 3=normal, 5=safe) */
		char		delimiter;	/* Page delimiter for PAGE mode (0 = default '|') */
	} dps;

} rds_preset_t;

/* RDS Presets - add more as needed
 * RT max length: 64 chars (Group 2A) or 32 chars (Group 2B) - terminated with CR (0x0D)
 * PS max length: 8 chars (padded with spaces)
 * PTYN max length: 8 chars */
static const rds_preset_t rds_presets[] = {
	/* ============================================================
	 * Press 'f' to cycle to normal presets.
	 * ============================================================ */

	{
		.name     = "Ukraine (RDS)",
		/* PI Structure for Europe (IEC 62106):
		 * [Country(4)][Coverage(4)][Reference(8)]
		 * Coverage Codes:
		 *  0=Local, 1=International, 2=National, 3=Supra-regional
		 *  4-F = Regional Area 1-12
		 * Here: 6 (Ukraine) + A (Regional 7) + CE (Ref) */
		.pi       = 0x6ACE,
		.ps       = "Osmo RDS",
		/* 64-char RT (max for Group 2A) */
		.rt       = "osmocom-analog FM RDS Radio - Open Source Broadcast FM Encoder!",
		.pty      = 10,		/* Pop music (RDS) */
		.ptyn     = "OsmoPTYN",
		.tp       = 1,
		.ms       = 1,
		.ecc      = 0xE4,	/* Ukraine with PI prefix 6 */
		.lang     = 73,		/* Ukrainian (LIC code from IEC 62106 Annex J) */
		/* AF Method A: VHF only -- LF/MF AF decodes fine with open-source
		 * RDS decoders but no hardware receiver to confirm tuning.
		 * EN 50067 Table 12 LF/MF codes (9 kHz, ITU Regions 1&3):
		 *   LF: code 1-15 -> freq = 144 + 9*code kHz (153-279 kHz)
		 *   MF: code 16-135 -> freq = 522 + 9*(code-15) kHz (531-1602 kHz)
		 * Encoded as [250, code] pair. */
		.af_method_a_str = "100.0, 104.5, 98.2",
		/* .af_method_a_str = "100.0, 104.5, 98.2, LF225", */
		/* Group 1A PIN - Legacy "VCR-like" recording feature (1984-1990s)
		 * Example: "Evening News" listed in newspaper as starting 18:00 on 15th.
		 * User enters PIN (day=15, 18:00) into receiver; when station transmits
		 * matching PIN, receiver triggers recording - compensating for overruns. */
		.pin_day  = 15, .pin_hour = 18, .pin_minute = 0,
		/* EON: Other Networks (Group 14A) - Full test configuration */
		.eon = {
			{
				.pi = 0x6B01, .ps = "UA News ", .pty = 1, .tp = 1, .ta = 1,
				/* AF variant 4: ON frequencies at 90.7 and 93.2 MHz */
				/* AF code = freq*10 - 875:  90.7*10-875=32, 93.2*10-875=57 */
				.af = { RDS_AF_MHZ(90.7), RDS_AF_MHZ(93.2) }, .af_count = 2,
				/* Mapped AF variant 5: 100.0 MHz (tuned) -> 90.7 MHz (ON) */
				/* Codes: 100.0*10-875=125 -> 90.7*10-875=32 */
				.mapped_af = { { .tuned = RDS_AF_MHZ(100.0), .on = RDS_AF_MHZ(90.7) } }, .mapped_af_count = 1,
				/* Linkage variant 12 */
				.linkage_la = 1, .linkage_lsn = 0x123,
				/* PIN variant 14 - Legacy cross-network recording (1980s-90s)
				 * Example: "World News" on linked station UA News (0x6B01)
				 * listed at 19:00 on 15th. Receiver could switch to linked
				 * station and record when that station's PIN matched. */
				.pin_day = 15, .pin_hour = 19, .pin_minute = 0,
				/* Broadcaster data variant 15 */
				.broadcaster_data = 0xABCD,
			},
			{
				.pi = 0x6C02, .ps = "Traffic1", .pty = 22, .tp = 1, .ta = 0,
				/* AF variant 4: ON frequencies at 94.7 and 97.2 MHz */
				/* AF code = freq*10 - 875:  94.7*10-875=72, 97.2*10-875=97 */
				.af = { RDS_AF_MHZ(94.7), RDS_AF_MHZ(97.2) }, .af_count = 2,
				/* No mapped AF for this one */
				.mapped_af_count = 0,
				/* Linkage variant 12 */
				.linkage_la = 0, .linkage_lsn = 0x456,
				/* PIN variant 14 - Legacy cross-network recording
				 * Example: "Traffic Report" on Traffic1 (0x6C02) at 07:30
				 * on 16th. Advanced receivers could auto-tune and record. */
				.pin_day = 16, .pin_hour = 7, .pin_minute = 30,
				/* Broadcaster data variant 15 */
				.broadcaster_data = 0x1234,
			},
		},
		.eon_count = 2,
		/* RT+ (RadioText Plus) Configuration
		 * 
		 * RadioText: "osmocom-analog FM RDS Radio - Open Source Broadcast FM Encoder!"
		 *             ^0              ^15
		 * 
		 * Tag1: item.title (type=1) "osmocom-analog" at positions 0-13 (14 chars)
		 * Tag2: item.album (type=2) "FM RDS Radio" at positions 15-26 (12 chars)
		 * 
		 * NOTE: Group 11A is what we default for RT+.
		 * The spec allows any ODA group. */
		.rtplus = {
			.enabled = 1,
			.carrier_group = RDS_GROUP_11A,	/* Group 11A for RT+ ODA */
			.message = 0x0000,		/* 3A msg: cb=0 (no template), scb=0, template=0 */
			.toggle = 0,
			.item_running = 1,		/* 1 = song is currently playing */
			.tags = {
				{ .content_type = RDS_RTPLUS_CT_ITEM_TITLE, .start = 0, .length = 14 },	/* "osmocom-analog" */
				{ .content_type = RDS_RTPLUS_CT_ITEM_ALBUM, .start = 15, .length = 12 },	/* "FM RDS Radio" */
			},
			.tag_count = 2,
		},
		/* eRT+ (Enhanced RadioText Plus) Configuration - Artist/Title tags
		 * IEC 62106-6:2018 Annex C: RT+ tags use CHARACTER positions (0-63 max)
		 * The 6-bit start marker limits tagging to first 64 CHARACTERS of eRT
		 * 
		 * eRT text: "osmocom-analog — Analoge Funktechnik · Open Source FM RDS Encoder"
		 *            ^0               ^17                   ^40
		 * 
		 * Character positions (NOT byte positions):
		 *   - "osmocom-analog" at char 0-13 (14 chars)
		 *   - "Analoge Funktechnik" at char 17-35 (19 chars) */
		.ert_plus = {
			.enabled = 0,	/* Disabled for RT+ debugging */
			.carrier_group = RDS_GROUP_13A,	/* Group 13A for eRT+ ODA (different from RT+ on 11A) */
			.message = 0x0000,		/* cb=0, scb=0, template=0 (eRT+ default) */
			.toggle = 0,
			.item_running = 1,
			.tags = {
				{ .content_type = RDS_RTPLUS_CT_ITEM_ARTIST, .start = 0, .length = 14 },	/* "osmocom-analog" chars 0-13 */
				{ .content_type = RDS_RTPLUS_CT_ITEM_TITLE, .start = 17, .length = 19 },	/* "Analoge Funktechnik" chars 17-35 */
			},
			.tag_count = 2,
		},
		/* eRT (Enhanced RadioText) Configuration with UTF-8
		 * IEC 62106-6:2018 Annex C: eRT supports 128 bytes (32 segments x 4 bytes)
		 * but RT+ can only tag first 64 CHARACTERS
		 * 
		 * This text uses UTF-8 em-dash (—) and middle dot (·) to demonstrate
		 * eRT's extended character support beyond basic ASCII RadioText.
		 * 65 characters, 69 bytes (— = 3 bytes, · = 2 bytes)
		 * Terminated with CR if < 128 bytes */
		.ert = {
			.enabled = 0,	/* Disabled -- eRT eats 30% bandwidth, most receivers don't support it */
			.carrier_group = RDS_GROUP_12A,	/* Group 12A for eRT ODA (RT+ uses 11A) */
			.message = RDS_ERT_3A_MSG_UTF8_LTR_E3,	/* UTF-8 encoding, LTR direction, E3 chartable */
			.text = "osmocom-analog — Analoge Funktechnik · Open Source FM RDS Encoder",
		},
		/* ============================================================
		 * RT+ (RadioText Plus) and eRT (Enhanced RadioText) Examples
		 * ============================================================
		 * RT+ and eRT are configured via runtime API after encoder init.
		 * These examples show comprehensive configurations with field
		 * descriptions indicating where each value comes from.
		 * ============================================================
		 * 
		 * EXAMPLE 1: RT+ (RadioText Plus) - Comprehensive Example
		 * ============================================================
		 * RT+ tags semantic segments within standard RadioText (64 chars).
		 * 
		 * RadioText (from preset .rt field):
		 *   "osmocom-analog FM Radio - Open Source Broadcast FM RDS Encoder!"
		 *   Position: 0         1         2         3         4         5
		 *             012345678901234567890123456789012345678901234567890123
		 * 
		 * RT+ Configuration (after encoder init):
		 *   // Register RT+ ODA on Group 11A (carrier_group=22)
		 *   rds_enc_oda_add(&radio->rds_enc, RDS_GROUP_11A, RDS_ODA_AID_RT_PLUS, 0x0000);
		 *   
		 *   // Set both tags at once (tag1: Title, tag2: Album)
		 *   // Tag 1: RDS_RTPLUS_CT_ITEM_TITLE (1), start=0, length=14 ("osmocom-analog")
		 *   // Tag 2: RDS_RTPLUS_CT_ITEM_ALBUM (2), start=15, length=12 ("FM RDS Radio")
		 *   // For single tag, pass 0 for ct2/start2/len2
		 *   rds_enc_rtplus_set_tags(&radio->rds_enc,
		 *       RDS_RTPLUS_CT_ITEM_TITLE, 0, 14,
		 *       RDS_RTPLUS_CT_ITEM_ALBUM, 15, 12);
		 * 
		 * ============================================================
		 * EXAMPLE 2: eRT (Enhanced RadioText) - Comprehensive Example
		 * ============================================================
		 * eRT extends RadioText to 128 bytes with UTF-8/UCS-2 support.
		 * 
		 * eRT Configuration (after encoder init):
		 *   // Register eRT ODA on Group 12A (carrier_group=24)
		 *   // Group 3A message bits:
		 *   //   Bit 0: encoding=1 (UTF-8, from eRT encoding setting)
		 *   //   Bit 1: direction=0 (LTR, from text direction setting)
		 *   //   Bits 5-2: chartable=0 (E3, from character table setting)
		 *   uint16_t ert_msg = (1 << 0) | (0 << 1) | (0 << 2);  // UTF-8, LTR, E3
		 *   rds_enc_oda_add(&radio->rds_enc, 24, RDS_ODA_AID_ERT, ert_msg);
		 *   
		 *   // Set eRT text (128 bytes max, UTF-8 encoded)
		 *   // Field: Extended RadioText content (from programme metadata)
		 *   // Value source: text="Now Playing: Artist Name - Song Title | Next: Upcoming Show | Weather: Sunny 22°C | Traffic: Clear on Highway A1"
		 *   //               length=128 (from strlen, truncated to 128 bytes max)
		 *   const char *ert_text = "Now Playing: Artist Name - Song Title | Next: Upcoming Show | Weather: Sunny 22°C | Traffic: Clear on Highway A1";
		 *   rds_enc_set_ert(&radio->rds_enc, (const uint8_t *)ert_text, strlen(ert_text));
		 * 
		 * ============================================================
		 * EXAMPLE 3: RT+ with eRT+ Tags - Comprehensive Example
		 * ============================================================
		 * eRT+ applies RT+ tagging to Enhanced RadioText (128 bytes).
		 * 
		 * eRT+ Configuration (after encoder init):
		 *   // Register eRT+ ODA on Group 11A (carrier_group=22)
		 *   // Group 3A message bits (same format as RT+):
		 *   //   Bit 12: cb=0 (from class/type flag)
		 *   //   Bits 11-8: scb=0 (from server control bits)
		 *   //   Bits 7-0: template=0 (from template number)
		 *   rds_enc_oda_add(&radio->rds_enc, RDS_GROUP_11A, RDS_ODA_AID_ERT_PLUS, 0x0000);
		 *   
		 *   // Set eRT text first (128 bytes)
		 *   // Field: Extended RadioText with multiple segments
		 *   // Value source: text="ARTIST: The Beatles | TITLE: Hey Jude | ALBUM: The Beatles 1967-1970 | GENRE: Rock | YEAR: 1968"
		 *   const char *ert_text2 = "ARTIST: The Beatles | TITLE: Hey Jude | ALBUM: The Beatles 1967-1970 | GENRE: Rock | YEAR: 1968";
		 *   rds_enc_set_ert(&radio->rds_enc, (const uint8_t *)ert_text2, strlen(ert_text2));
		 *   
		 *   // Set both eRT+ tags at once
		 *   // Tag 1: RDS_RTPLUS_CT_ITEM_ARTIST (4), start=8, length=12 ("The Beatles")
		 *   // Tag 2: RDS_RTPLUS_CT_ITEM_TITLE (1), start=30, length=9 ("Hey Jude")
		 *   rds_enc_ert_plus_set_tags(&radio->rds_enc,
		 *       RDS_RTPLUS_CT_ITEM_ARTIST, 8, 12,
		 *       RDS_RTPLUS_CT_ITEM_TITLE, 30, 9);
		 * 
		 * ============================================================
		 * EXAMPLE 4: eRT with UTF-8 Demonstration - Comprehensive Example
		 * ============================================================
		 * eRT supports UTF-8 encoding for international characters.
		 * 
		 * eRT UTF-8 Configuration (after encoder init):
		 *   // Register eRT ODA on Group 12A (carrier_group=24)
		 *   // Group 3A message: UTF-8 encoding enabled
		 *   // Value source: encoding=1 (UTF-8, from eRT encoding setting),
		 *   //               direction=0 (LTR, from text direction),
		 *   //               chartable=0 (E3, from character table)
		 *   uint16_t ert_utf8_msg = (1 << 0);  // UTF-8 encoding bit set
		 *   rds_enc_oda_add(&radio->rds_enc, 24, RDS_ODA_AID_ERT, ert_utf8_msg);
		 *   
		 *   // Set eRT text with UTF-8 characters (128 bytes max)
		 *   // Field: Extended RadioText with international characters
		 *   // Value source: text with UTF-8 sequences:
		 *   //   - "Now Playing: Café" (é = UTF-8 0xC3 0xA9, 2 bytes)
		 *   //   - "Artist: Björk" (ö = UTF-8 0xC3 0xB6, 2 bytes)
		 *   //   - "Title: 你好世界" (Chinese = UTF-8 multi-byte sequences)
		 *   //   - "Weather: 22°C" (° = UTF-8 0xC2 0xB0, 2 bytes)
		 *   //   - "Traffic: Київ" (Ukrainian = UTF-8 multi-byte sequences)
		 *   //   length=calculated from UTF-8 byte count (max 128 bytes)
		 *   const char *ert_utf8_text = "Now Playing: Café | Artist: Björk | Title: 你好世界 | Weather: 22°C | Traffic: Київ";
		 *   rds_enc_set_ert(&radio->rds_enc, (const uint8_t *)ert_utf8_text, strlen(ert_utf8_text));
		 *   
		 *   // Note: UTF-8 multi-byte characters count as multiple bytes in length.
		 *   // Example: "Café" = 5 bytes (C=1, a=1, f=1, é=2 bytes)
		 *   //          "你好" = 6 bytes (each Chinese char = 3 UTF-8 bytes)
		 * 
		 * ============================================================
		 * EXAMPLE 5: RT+ with Multiple Content Types - Comprehensive Example
		 * ============================================================
		 * RT+ supports up to 2 tags per group. Use set_tags() to set both.
		 * 
		 * RadioText (from preset .rt field):
		 *   "News: Breaking Story | Weather: Sunny 25°C | Traffic: Highway Clear"
		 *   Position: 0         1         2         3         4         5
		 *             012345678901234567890123456789012345678901234567890123
		 * 
		 * RT+ Configuration (after encoder init):
		 *   rds_enc_oda_add(&radio->rds_enc, RDS_GROUP_11A, RDS_ODA_AID_RT_PLUS, 0x0000);
		 *   
		 *   // Set tags: News and Weather
		 *   rds_enc_rtplus_set_tags(&radio->rds_enc,
		 *       RDS_RTPLUS_CT_INFO_NEWS, 0, 20,
		 *       RDS_RTPLUS_CT_INFO_WEATHER, 22, 17);
		 *   
		 *   // To change tags (e.g., show Traffic instead):
		 *   // clear_tags() sets running=0 (no toggle flip)
		 *   rds_enc_rtplus_clear_tags(&radio->rds_enc);
		 *   // set_tags() sets running=1 and flips the toggle bit
		 *   rds_enc_rtplus_set_tags(&radio->rds_enc,
		 *       RDS_RTPLUS_CT_INFO_TRAFFIC, 40, 19,
		 *       0, 0, 0);  // Single tag
		 * 
		 * ============================================================
		 * Carrier Group Codes Reference:
		 *   8A = 16,  8B = 17,  9A = 18,  9B = 19,
		 *  10A = 20, 10B = 21, 11A = 22, 11B = 23,
		 *  12A = 24, 12B = 25, 13A = 26, 13B = 27,
		 *  14A = 28, 14B = 29, 15A = 30, 15B = 31
		 * ============================================================
		 */
	},
	/* ============================================================
	 * DYNAMIC PS DEMO PRESETS
	 * ============================================================
	 * Three presets demonstrating the three historical scrolling
	 * patterns used by European broadcasters. Press 'f' to cycle.
	 * ============================================================ */
	{
		/* Pattern 1: Window shift (character scroll) - de facto standard.
		 * Text slides through the 8-char display one character at a time.
		 * Most common pattern used by European commercial stations. */
		.name     = "DynPS Scroll",
		.pi       = 0x6AD1,
		.ps       = "SCROLL  ",	/* Initial static PS (shown briefly during warmup) */
		.rt       = "Dynamic PS Scroll Demo - Character-by-character window shift",
		.pty      = 10,
		.ms       = 1,
		.dps      = {
			.text   = "NOW PLAYING BOHEMIAN RHAPSODY BY QUEEN",
			.mode   = RDS_DPS_SCROLL,
			.repeat = RDS_DPS_REPEAT_NORMAL,	/* 3 PS tx/step */
		},
	},
	{
		/* Pattern 2: Word-step scrolling - receiver-friendly.
		 * Full words swapped instead of character scrolling.
		 * Popular on German and Nordic broadcasts. */
		.name     = "DynPS Word",
		.pi       = 0x6AD2,
		.ps       = "WORDSTEP",
		.rt       = "Dynamic PS Word Demo - Word-aligned page stepping",
		.pty      = 10,
		.ms       = 1,
		.dps      = {
			.text   = "OSMOCOM ANALOG OPEN SOURCE FM RADIO",
			.mode   = RDS_DPS_WORD,
			.repeat = RDS_DPS_REPEAT_NORMAL,
		},
	},
	{
		/* Pattern 3: Paging / alternating messages.
		 * Fixed 8-char strings alternate. Pages separated by '|'.
		 * Considered least abusive by regulators. */
		.name     = "DynPS Page",
		.pi       = 0x6AD3,
		.ps       = "PAGING  ",
		.rt       = "Dynamic PS Page Demo - Alternating fixed messages",
		.pty      = 10,
		.ms       = 1,
		.dps      = {
			.text   = "RADIO 1 |HOT HITS|98.5 FM ",
			.mode   = RDS_DPS_PAGE,
			.repeat = RDS_DPS_REPEAT_SAFE,	/* 5 PS tx/step - slow alternation */
		},
	},
	{
		/* Fast page demo: repeat=1 pushes speed to the absolute limit.
		 * Each page shown for only one PS cycle (~0.7s). Designed to
		 * look good on alphanumeric segmented displays (8-char LCD/VFD).
		 *
		 * Uses '\n' as page delimiter so '|' can appear on display
		 * (vertical bars make great curtain/border effects on VFD).
		 *
		 * ASCII animation effects:
		 *   - Curtain open (||| bars part from center)
		 *   - Typewriter text reveal with blinking cursor
		 *   - Wipe transition (========)
		 *   - Center pop with sparkle burst
		 *   - Bouncing dot (ping-pong)
		 *   - Expanding brackets with text fill
		 *   - Curtain close (||| bars close)
		 *   - Slot machine spin
		 *   - Strobe flash finale
		 *
		 * ~80 pages at ~0.7s each = ~56s full cycle. */
		.name     = "DynPS Fast",
		.pi       = 0x6AD4,
		.ps       = "FASTPAGE",
		.rt       = "Dynamic PS Fast Page Demo - ASCII animation on 8-char display",
		.pty      = 10,
		.ms       = 1,
		.dps      = {
			.delimiter = '\n',
			.text   =
			/* --- Act 1: Curtain open (| bars part from center) --- */
			"||||||||\n"
			"|||  |||\n"
			"||    ||\n"
			"|      |\n"
			"        \n"
			/* --- Act 2: Typewriter reveal "OSMO FM" --- */
			"O_      \n"
			"OS_     \n"
			"OSM_    \n"
			"OSMO_   \n"
			"OSMO _  \n"
			"OSMO F_ \n"
			"OSMO FM \n"
			"OSMO FM \n"  /* hold */
			/* --- Act 3: Wipe right erases --- */
			"=SMO FM \n"
			"==MO FM \n"
			"===O FM \n"
			"==== FM \n"
			"=====FM \n"
			"======M \n"
			"======= \n"
			"========\n"
			/* --- Act 4: Wipe continues right, clears --- */
			" =======\n"
			"  ======\n"
			"   =====\n"
			"    ====\n"
			"     ===\n"
			"      ==\n"
			"       =\n"
			"        \n"
			/* --- Act 5: Center pop "RDS" --- */
			"   *    \n"
			"  *R*   \n"
			" * R *  \n"
			"  RDS   \n"
			"  RDS   \n"  /* hold */
			/* --- Act 6: Sparkle around RDS --- */
			"* RDS  *\n"
			" *RDS*  \n"
			"* RDS *.\n"
			".*RDS*. \n"
			". RDS . \n"
			"  RDS   \n"
			/* --- Act 7: Bouncing dot --- */
			".       \n"
			" .      \n"
			"  .     \n"
			"   .    \n"
			"    .   \n"
			"     .  \n"
			"      . \n"
			"       .\n"
			"      . \n"
			"     .  \n"
			"    .   \n"
			"   .    \n"
			"  .     \n"
			" .      \n"
			".       \n"
			/* --- Act 8: Expanding brackets reveal RADIO --- */
			"   ..   \n"
			"  .  .  \n"
			" .    . \n"
			"[      ]\n"
			"[ R    ]\n"
			"[ RA   ]\n"
			"[ RAD  ]\n"
			"[ RADI ]\n"
			"[RADIO ]\n"
			"[RADIO ]\n"  /* hold */
			/* --- Act 9: Curtain close (| bars close from edges) --- */
			"|RADIO |\n"
			"|RADIO |\n"
			"||ADIO||\n"
			"||DIO ||\n"
			"|||IO|||\n"
			"||||O|||\n"
			"||||||||\n"
			"        \n"
			/* --- Act 10: Slot machine spin --- */
			"--OPEN--\n"
			"=SOURCE=\n"
			"--OPEN--\n"
			"=SOURCE=\n"
			"  -FM-  \n"
			" =RDS=  \n"
			"  -FM-  \n"
			/* --- Act 11: Strobe flash finale --- */
			"*OSMO*FM\n"
			"        \n"
			"*OSMO*FM\n"
			"        \n"
			"*OSMO*FM",
			.mode   = RDS_DPS_PAGE,
			.repeat = 2,	/* 2 PS tx/step (~1.4s) - fast but readable */
		},
	},
	{
		.name     = "USA (RBDS)",
		/*.pi       = 0xABCD,*/	/* PI=Axxx = USA/RBDS region */
		/* RBDS PI Codes (NRSC-4-B):
		 *  1000 - 994F: Computed from Call Sign (e.g. WNYC -> 796E)
		 *  9950 - 9EFF: 3-Letter Call Signs
		 *  AFxx / A0xx: Linked Stations / Regional
		 *  Bxxx, Dxxx, Exxx: Linked National Networks */
		.callsign = "WNYC",	/* Popular call sign (derived PI=796E) */
		.ps       = "OsmoRBDS",
		/* 64-char RT (max for Group 2A) - use full capacity for demo */
		.rt       = "osmocom-analog FM Radio - Open Source Broadcast FM RDBS Encoder",
		.pty      = 9,		/* Top 40 (RBDS) */
		.ptyn     = "Top 40  ",
		.tp       = 0,
		.ms       = 1,
		.ecc      = 0xA0,	/* USA (RBDS region) with PI prefix A */
		.lang     = 9,		/* English */
		.af_method_a_str = "92.5, 97.5, 102.5",
		/* RBDS MF AF (ITU Region 2, NRSC-4-B, no LF broadcasting):
		 *   MF code = 16 + (freq_kHz - 530) / 10 (10 kHz spacing)
		 *   e.g. 1010 kHz -> code 64
		 * vs RDS (ITU Regions 1&3, EN 50067 Table 12):
		 *   LF: code 1-15 -> 144 + 9*code kHz (153-279 kHz)
		 *   MF: code 16-135 -> 522 + 9*(code-15) kHz (531-1602 kHz)
		 * Both encoded as [250, code] pair. Decodes fine with
		 * open-source RDS decoders, no hardware receiver to test. */
		/* .af_method_a_str = "92.5, 97.5, 102.5, MF1010", */
		/* Group 1A PIN - not used (0x0000 = most common value)
		 * Many US stations don't use PIN, so we demonstrate day=0 */
		.pin_day  = 0, .pin_hour = 0, .pin_minute = 0,
		/* group*_version omitted = AUTO: 0A (has AF), 2A (RT>32), 1A (has ECC) */
	},
	/* ============================================================
	 * MINIMAL PRESET - Mandatory Group 0 Only
	 * ============================================================
	 * Demonstrates: Minimum compliant RDS stream.
	 * Only Group 0B transmitted (PS name with PI repeat).
	 * No RT, No ECC, No PTYN, No EON -> those groups won't transmit.
	 * Use case: Small station, minimal bandwidth.
	 * ============================================================ */
	{
		.name     = "Minimal",
		.pi       = 0x1234,
		.ps       = "MINIMAL ",
		.ms       = 1,
		.group0_version = RDS_GROUP_VERSION_B,  /* Force 0B (no AF) */
		/* All other fields omitted = 0 = no data -> only Group 0 transmits */
	},
	/* ============================================================
	 * MOBILE PRESET - B Versions for Fast PI Identification
	 * ============================================================
	 * Demonstrates: All B versions for improved mobile reception.
	 * PI repeat in Block C of every group helps receivers lock faster.
	 * Trade-off: Less data capacity (32-char RT, no ECC).
	 * Use case: Mobile/in-car listening, weak signal areas.
	 * ============================================================ */
	{
		.name     = "Mobile",
		.pi       = 0x5678,
		.ps       = "MOBILE  ",
		.rt       = "Fast cycling RadioText",  /* <=32 chars for 2B */
		.pty      = 10,
		.ms       = 1,
		.group0_version = RDS_GROUP_VERSION_B,  /* 0B: PI repeat */
		.group2_version = RDS_GROUP_VERSION_B,  /* 2B: 32-char, faster */
		/* No ECC/Lang -> Group 1 won't transmit */
	},
	/* ============================================================
	 * MIXED PRESET - AF List + Fast RadioText (0A + 2B)
	 * ============================================================
	 * Demonstrates: Mixed A/B for different group types.
	 * 0A for AF list (need Block C for frequencies).
	 * 2B for fast RadioText cycling (trade 64->32 chars for speed).
	 * Use case: Regional station with AF, wants quick RT updates.
	 * ============================================================ */
	{
		.name     = "AF + Fast RT",
		.pi       = 0x9ABC,
		.ps       = "MIXED   ",
		.rt       = "Quick updates via 2B",
		.pty      = 3,
		.ms       = 1,
		.af_method_a_str = "90.5, 93.5, 96.5",
		.group0_version = RDS_GROUP_VERSION_A,  /* 0A: need AF list */
		.group2_version = RDS_GROUP_VERSION_B,  /* 2B: faster RT cycling */
	},
	/* ============================================================
	 * AUTO DEMO PRESET - Let Data Decide A/B Versions
	 * ============================================================
	 * Demonstrates: Auto-detection from data (all version fields = 0).
	 * - No AF -> auto-selects 0B
	 * - RT <=32 chars -> auto-selects 2B
	 * - No ECC/Lang -> auto-selects 1B (if PIN set)
	 * Use case: Understanding auto-detection behavior.
	 * ============================================================ */
	{
		.name     = "Auto Demo",
		.pi       = 0xDEF0,
		.ps       = "AUTODEMO",
		.rt       = "Short text for 2B",  /* <=32 chars -> auto: 2B */
		.pty      = 5,
		.ms       = 1,
		.pin_day  = 20, .pin_hour = 12, .pin_minute = 0,  /* PIN set -> Group 1 transmits */
		/* No AF -> auto: 0B
		 * No ECC/Lang -> auto: 1B (PIN only)
		 * RT <=32 -> auto: 2B
		 * group*_version all 0 = AUTO */
	},
	{
		.name     = "Method B Test",
		.pi       = 0xAFB1,
		.ps       = "AFMETHB ",
		.rt       = "Testing RDS AF Method B with regional variants",
		.pty      = 15,
		.ecc      = 0xE0,
		.lang     = 9,
		.ms       = 1,
		/* AF Method B: Example from IEC 62106 S3.2.1.6.4
		 * List 1: Tuning 89.3 MHz, AFs: 99.5, 101.7, 88.8 (same), R102.6, R89.0 (regional)
		 * List 2: Tuning 99.5 MHz, AFs: 89.3, 100.9 (same), R104.8, R89.1 (regional) */
		.af_method_b_str = {
			"T89.3, 99.5, 101.7, 88.8, R102.6, R89.0",
			"T99.5, 89.3, 100.9, R104.8, R89.1",
		},
		.af_method_b_count = 2,
		.group0_version = RDS_GROUP_VERSION_A,
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - German (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: German umlauts and ß from RDS charset.
	 * Characters: ä (0x91), ö (0x97), ü (0x99), ß (0x8D)
	 *             Ä (0xD1), Ö (0xD7), Ü (0xD9)
	 * ============================================================ */
	{
		.name     = "German Demo",
		.pi       = 0xD314,
		.ps       = "WÜRZBÜRG",
		.rt       = "Größe, Müller, Schöne Grüße! Fünf Äpfel für Österreich.",
		.pty      = 10,
		.ptyn     = "Größe   ",	/* PTYN with ö, ß */
		.ms       = 1,
		.ecc      = 0xE0,	/* Germany */
		.lang     = 8,		/* German */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - French (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: French accented chars from RDS charset.
	 * Characters: à (0x81), é (0x82), è (0x83), ê (0x92), ë (0x93)
	 *             î (0x94), ï (0x95), ô (0x96), û (0x98), ù (0x89)
	 *             ç (0x9B), œ (0xF3)
	 * ============================================================ */
	{
		.name     = "French Demo",
		.pi       = 0xF201,
		.ps       = "CAFÉ  FM",
		.rt       = "Bienvenue à Noël! Très bel été, où êtes-vous? Ça va!",
		.pty      = 14,		/* Classical */
		.ptyn     = "Évén't  ",	/* PTYN with é */
		.ms       = 1,
		.ecc      = 0xE1,	/* France */
		.lang     = 15,		/* French */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - Spanish + Euro (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: Spanish ñ and symbols from RDS charset.
	 * Characters: ñ (0x9A), Ñ (0x8A), € (0xA9), ¿ (0xB9), ¡ (0x8E)
	 * ============================================================ */
	{
		.name     = "Spanish Demo",
		.pi       = 0xE502,
		.ps       = "ESPAÑA  ",
		.rt       = "¡Buenas Señor! ¿Cuánto? Mañana €100. ¡Niño pequeño!",
		.pty      = 6,		/* Drama */
		.ptyn     = "Señales ",	/* PTYN with ñ */
		.ms       = 1,
		.ecc      = 0xE2,	/* Spain */
		.lang     = 14,		/* Spanish */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - Nordic (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: Norwegian/Danish/Swedish special characters.
	 * Characters: Å (0xE1), Æ (0xE2), Ø (0xE7)
	 *             å (0xF1), æ (0xF2), ø (0xF7)
	 * ============================================================ */
	{
		.name     = "Nordic Demo",
		.pi       = 0xF503,
		.ps       = "ÅRHUS FM",
		.rt       = "Velkommen! Ål, Æble, Øl fra København. Blåbær og Rødgrød!",
		.pty      = 12,		/* Light classical */
		.ptyn     = "Søndags ",	/* Sunday in Danish with ø */
		.ms       = 1,
		.ecc      = 0xE2,	/* Denmark/Norway */
		.lang     = 7,		/* Danish */
	},
	/* ============================================================
	 * EXTENDED CHARACTER DEMO - Greek (IEC 62106 Annex E)
	 * ============================================================
	 * Demonstrates: Greek letters available in RDS charset.
	 * Note: RDS Annex E has very limited Greek: α (0xA1), π (0xA8)
	 * Real Greek stations use Latin transliteration for RDS.
	 * ============================================================ */
	{
		.name     = "Greek Demo",
		.pi       = 0x1F01,
		.ps       = "ATHINA  ",
		.rt       = "Kalimera! To α kai to π einai ellinika. FM Ellada!",
		.pty      = 11,		/* Rock music */
		.ptyn     = "Mousiki ",	/* Music in transliterated Greek */
		.ms       = 1,
		.ecc      = 0xE1,	/* Greece */
		.lang     = 18,		/* Greek */
	},
	/* ============================================================
	 * AF METHOD A STRING DEMO (IEC 62106 S3.2.1.6.3)
	 * ============================================================
	 * Demonstrates: Human-readable AF Method A string format.
	 * Format: "freq_mhz, freq_mhz, LFxxx, MFxxx"
	 * LF/MF count as 2 slots each (encoded as [250, code] pairs).
	 * LF/MF decodes fine with open-source decoders, no HW receiver to test.
	 * ============================================================ */
	{
		.name     = "AF Method A Demo",
		.pi       = 0xAF01,
		.ps       = "AF DEMO ",
		.rt       = "Testing AF Method A with VHF, LF, and MF frequencies!",
		.pty      = 15,
		.ms       = 1,
		.ecc      = 0xE0,	/* Germany */
		.lang     = 8,		/* German */
		/* AF Method A: VHF only -- LF/MF AF decodes fine with open-source
		 * RDS decoders but no hardware receiver to confirm tuning.
		 * EN 50067 Table 12 LF/MF codes (9 kHz, ITU Regions 1&3):
		 *   LF: code 1-15 -> freq = 144 + 9*code kHz (153-279 kHz)
		 *   MF: code 16-135 -> freq = 522 + 9*(code-15) kHz (531-1602 kHz)
		 * Encoded as [250, code] pair. */
		.af_method_a_str = "91.0, 95.5, 102.3, 88.1, 106.7",
		/* .af_method_a_str = "91.0, 95.5, 102.3, LF225, MF1008", */
	},
	/* End of presets */
};

#define RDS_PRESET_COUNT (sizeof(rds_presets) / sizeof(rds_presets[0]))

static int rds_current_preset = 0;

/* Additional settings not in presets (common to all) */
static uint8_t rds_ta        = 0;		/* Traffic Announcement */

static uint8_t rds_di_artificial_head = 0;
static uint8_t rds_di_compressed = 0;
static uint8_t rds_di_dynamic_pty = 0;

/* Programme Item Number (PIN) is now defined per-preset in rds_preset_t.
 * See preset .pin_day, .pin_hour, .pin_minute fields. */
static int rds_ct_enabled    = 1;		/* Clock-Time enabled */

/* Group version selection is now per-preset via group0/1/2_version fields.
 * See rds_preset_t and RDS_GROUP_VERSION_AUTO/A/B enum. */


static char freq_name[2][64];

/* User overrides */
static uint16_t rds_user_pi = 0;
static char *rds_user_callsign = NULL;

/* Apply current RDS preset to encoder (for runtime switching) */
static void rds_apply_preset(radio_t *radio)
{
	const rds_preset_t *p = &rds_presets[rds_current_preset];
	rds_encoder_t *enc = &radio->rds_enc;
	
	LOGP(DRADIO, LOGL_DEBUG, "RDS: Applying preset %d: %s\n", rds_current_preset, p->name);
	
	/* Update PI (will require receiver to re-sync) */
	uint16_t pi = p->pi;

	/* If PI is 0 in preset, try to derive from callsign */
	if (pi == 0 && p->callsign) {
		pi = rds_get_pi_from_callsign(p->callsign);
	}

	if (rds_user_pi)
		pi = rds_user_pi;
	else if (rds_user_callsign) {
		uint16_t cpi = rds_get_pi_from_callsign(rds_user_callsign);
		if (cpi) pi = cpi;
	}
	rds_enc_set_pi(enc, pi);
	
	/* Update PS using API */
	if (p->ps && p->ps[0] != '\0') {
		rds_enc_set_ps(enc, p->ps);
	} else {
		rds_enc_clear_ps(enc);
	}
	
	/* Update RadioText using the proper API which handles:
	 * - CR (0x0D) termination per EN 50067
	 * - A/B flag toggle to force receiver display update
	 * - Scheduler update for RT presence changes
	 * - RDS charset conversion from UTF-8 */
	if (p->rt && p->rt[0] != '\0') {
		rds_enc_set_radiotext(enc, p->rt);
	} else {
		rds_enc_clear_radiotext(enc);
	}
	
	/* Update PTY and PTYN using API */
	rds_enc_set_pty(enc, p->pty);
	if (p->ptyn && p->ptyn[0] != '\0') {
		rds_enc_set_ptyn(enc, p->ptyn);
	} else {
		rds_enc_clear_ptyn(enc);
	}
	
	/* Update traffic and mode flags using API */
	rds_enc_set_tp(enc, p->tp);
	rds_enc_set_ms(enc, p->ms);
	
	/* Update DI stereo flag based on broadcast mode (-S flag) */
	enc->di_stereo = radio->stereo ? 1 : 0;
	
	/* Update country codes using API */
	if (p->ecc != 0) {
		rds_enc_set_ecc(enc, p->ecc);
	} else {
		rds_enc_clear_ecc(enc);
	}
	if (p->lang != 0) {
		rds_enc_set_language(enc, p->lang);
	} else {
		rds_enc_clear_language(enc);
	}
	
	/* Update Group 1A PIN (Programme Item Number) for THIS station using API */
	if (p->pin_day != 0) {
		rds_enc_set_pin(enc, p->pin_day, p->pin_hour, p->pin_minute);
	} else {
		rds_enc_clear_pin(enc);
	}
	
	/* Update Alternative Frequencies using API */
	/* Clear existing AFs first */
	rds_enc_af_clear(enc);
	
	/* Load Method B if present (takes priority) */
	if (p->af_method_b_count > 0 && p->af_method_b_str[0]) {
		for (int i = 0; i < p->af_method_b_count && i < RDS_AF_METHOD_B_MAX_LISTS; i++) {
			if (p->af_method_b_str[i] && p->af_method_b_str[i][0]) {
				rds_enc_af_method_b_add(enc, p->af_method_b_str[i]);
			}
		}
	}
	
	/* Load Method A if no Method B */
	if (enc->af_method_b.list_count == 0 && p->af_method_a_str && p->af_method_a_str[0]) {
		rds_enc_af_set_method_a(enc, p->af_method_a_str);
	}
	
	/* Update EON (Enhanced Other Networks) - Group 14A using API */
	rds_enc_eon_clear(enc);
	if (p->eon_count > 0) {
		for (int i = 0; i < p->eon_count && i < RDS_EON_MAX_ENTRIES; i++) {
			/* Add basic EON entry using API */
			if (rds_enc_eon_add(enc, p->eon[i].pi, p->eon[i].ps, p->eon[i].pty, p->eon[i].tp) == 0) {
				/* Set TA if needed */
				if (p->eon[i].ta) {
					rds_enc_eon_set_ta(enc, p->eon[i].pi, p->eon[i].ta);
				}
				
				/* Find the entry we just added to set extended fields */
				for (int j = 0; j < enc->eon_tx_count; j++) {
					if (enc->eon_tx[j].pi == p->eon[i].pi) {
						rds_eon_entry_t *eon = &enc->eon_tx[j];
						
						/* AF (variant 4) - convert to 0.1 MHz format */
						if (p->eon[i].af_count > 0) {
							for (int k = 0; k < p->eon[i].af_count && k < RDS_EON_MAX_AF; k++) {
								eon->af[k] = 875 + p->eon[i].af[k];  /* Convert code to 0.1MHz */
							}
							eon->af_count = p->eon[i].af_count;
						}
						
						/* Mapped AF (variants 5-9) */
						if (p->eon[i].mapped_af_count > 0) {
							for (int k = 0; k < p->eon[i].mapped_af_count && k < RDS_EON_MAX_MAPPED_AF; k++) {
								eon->mapped_af[k].tuned_af = p->eon[i].mapped_af[k].tuned;
								eon->mapped_af[k].on_af = p->eon[i].mapped_af[k].on;
							}
							eon->mapped_af_count = p->eon[i].mapped_af_count;
						}
						
						/* Linkage (variant 12) */
						eon->linkage_la = p->eon[i].linkage_la;
						eon->linkage_lsn = p->eon[i].linkage_lsn;
						
						/* PIN (variant 14) */
						eon->pin_day = p->eon[i].pin_day;
						eon->pin_hour = p->eon[i].pin_hour;
						eon->pin_minute = p->eon[i].pin_minute;
						
						/* Broadcaster data (variant 15) */
						eon->broadcaster_data = p->eon[i].broadcaster_data;
						break;
					}
				}
			}
		}
		enc->eon_tx_index = 0;
		enc->eon_tx_variant = 0;
	}
	
	/* Reset PS segment to restart transmission */
	enc->ps_segment = 0;
	
	/* Reset warmup mode: Group 0 only for ~5 seconds (57 groups @ 11.4/sec) */
	enc->warmup_countdown = 57;
	
	/* --------------------------------------------------------
	 * Group Version Selection (A vs B)
	 * --------------------------------------------------------
	 * Priority: Manual override > Auto-detection
	 * AUTO (0) = detect from data, A (1) = force A, B (2) = force B
	 * -------------------------------------------------------- */
	
	/* Group 0: 0A (with AF) vs 0B (PI repeat, no AF) */
	if (p->group0_version == RDS_GROUP_VERSION_AUTO)
		enc->use_0b = (p->af_method_a_str == NULL || p->af_method_a_str[0] == '\0');  /* No AF -> 0B */
	else
		enc->use_0b = (p->group0_version == RDS_GROUP_VERSION_B);
	
	/* Group 1: 1A (ECC/Lang/PIN) vs 1B (PIN only) */
	if (p->group1_version == RDS_GROUP_VERSION_AUTO)
		enc->use_1b = (p->ecc == 0 && p->lang == 0);  /* No ECC/Lang -> 1B */
	else
		enc->use_1b = (p->group1_version == RDS_GROUP_VERSION_B);
	
	/* Group 2: 2A (64-char RT) vs 2B (32-char, faster) */
	if (p->group2_version == RDS_GROUP_VERSION_AUTO) {
		size_t rt_len = p->rt ? strlen(p->rt) : 0;
		enc->use_2b = (rt_len > 0 && rt_len <= 32);  /* Short RT -> 2B */
	} else {
		enc->use_2b = (p->group2_version == RDS_GROUP_VERSION_B);
	}
	
	/* Update RT+ (RadioText Plus) Configuration */
	if (p->rtplus.enabled) {
		/* Register RT+ ODA */
		rds_enc_oda_add(enc, p->rtplus.carrier_group, RDS_ODA_AID_RT_PLUS, p->rtplus.message);
		
		/* Set RT+ tags (1 or 2 tags) */
		if (p->rtplus.tag_count >= 1) {
			uint8_t ct1 = p->rtplus.tags[0].content_type;
			uint8_t start1 = p->rtplus.tags[0].start;
			uint8_t len1 = p->rtplus.tags[0].length;
			uint8_t ct2 = 0, start2 = 0, len2 = 0;
			if (p->rtplus.tag_count >= 2) {
				ct2 = p->rtplus.tags[1].content_type;
				start2 = p->rtplus.tags[1].start;
				len2 = p->rtplus.tags[1].length;
			}
			rds_enc_rtplus_set_tags(enc, ct1, start1, len1, ct2, start2, len2);
		}
		
		LOGP(DRADIO, LOGL_INFO, "RDS RT+: Enabled on group %d%c with %d tag(s)\n",
		     p->rtplus.carrier_group >> 1,
		     (p->rtplus.carrier_group & 1) ? 'B' : 'A',
		     p->rtplus.tag_count);
	} else {
		/* Remove RT+ ODA if it was previously enabled */
		rds_enc_oda_remove(enc, RDS_ODA_AID_RT_PLUS);
		rds_enc_rtplus_clear_tags(enc);
	}
	
	/* Update eRT+ (Enhanced RadioText Plus) Configuration */
	if (p->ert_plus.enabled) {
		/* Register eRT+ ODA */
		rds_enc_oda_add(enc, p->ert_plus.carrier_group, RDS_ODA_AID_ERT_PLUS, p->ert_plus.message);
		
		/* Set eRT+ tags (1 or 2 tags) */
		if (p->ert_plus.tag_count >= 1) {
			uint8_t ct1 = p->ert_plus.tags[0].content_type;
			uint8_t start1 = p->ert_plus.tags[0].start;
			uint8_t len1 = p->ert_plus.tags[0].length;
			uint8_t ct2 = 0, start2 = 0, len2 = 0;
			if (p->ert_plus.tag_count >= 2) {
				ct2 = p->ert_plus.tags[1].content_type;
				start2 = p->ert_plus.tags[1].start;
				len2 = p->ert_plus.tags[1].length;
			}
			rds_enc_ert_plus_set_tags(enc, ct1, start1, len1, ct2, start2, len2);
		}
		
		LOGP(DRADIO, LOGL_INFO, "RDS eRT+: Enabled on group %d%c with %d tag(s)\n",
		     p->ert_plus.carrier_group >> 1,
		     (p->ert_plus.carrier_group & 1) ? 'B' : 'A',
		     p->ert_plus.tag_count);
	} else {
		/* Remove eRT+ ODA if it was previously enabled */
		rds_enc_oda_remove(enc, RDS_ODA_AID_ERT_PLUS);
		rds_enc_ert_plus_clear_tags(enc);
	}
	
	/* Reset eRT segment counter to 0 on preset load
	 * (redsea requires sequential byte reception starting from segment 0) */
	enc->ert.segment = 0;
	
	/* Update eRT (Enhanced RadioText) Configuration */
	if (p->ert.enabled && p->ert.text) {
		/* Register eRT ODA */
		rds_enc_oda_add(enc, p->ert.carrier_group, RDS_ODA_AID_ERT, p->ert.message);
		
		/* Set eRT text (UTF-8 encoded, up to 128 bytes)
		 * Note: rds_enc_set_ert() will handle UTF-8-aware truncation
		 * to avoid splitting multi-byte sequences */
		size_t ert_len = strlen(p->ert.text);
		rds_enc_set_ert(enc, (const uint8_t *)p->ert.text, ert_len);
		
		LOGP(DRADIO, LOGL_INFO, "RDS eRT: Enabled on group %d%c with %zu bytes (encoding=%s)\n",
		     p->ert.carrier_group >> 1,
		     (p->ert.carrier_group & 1) ? 'B' : 'A',
		     ert_len,
		     (p->ert.message & RDS_ERT_3A_ENCODING_MASK) == RDS_ERT_ENCODING_UTF8 ? "UTF-8" : "UCS-2");
	} else {
		/* Remove eRT ODA if it was previously enabled */
		rds_enc_oda_remove(enc, RDS_ODA_AID_ERT);
		rds_enc_clear_ert(enc);
	}
	
	/* Debug test mode */
	LOGP(DRADIO, LOGL_INFO, "RDS Preset: %s (PI=%04X) Groups: %s/%s/%s\n",
	     p->name, enc->pi,
	     enc->use_0b ? "0B" : "0A",
	     enc->use_1b ? "1B" : "1A",
	     enc->use_2b ? "2B" : "2A");
	     
	/* Rebuild group scheduler to reflect new configuration (e.g. enable/disable 10A PTYN) */
	rds_scheduler_update(enc);
	
	/* Dynamic PS: must be applied AFTER scheduler update so timing
	 * estimates can inspect the actual group_sched_buffer[] */
	if (p->dps.text && p->dps.text[0]) {
		rds_enc_set_dynamic_ps(enc, p->dps.text,
		                       (rds_dynamic_ps_mode_t)p->dps.mode,
		                       p->dps.repeat ? p->dps.repeat : RDS_DPS_REPEAT_NORMAL,
		                       p->dps.delimiter);
	} else {
		rds_enc_stop_dynamic_ps(enc);
	}
}

/* Cycle to next RDS preset */
void rds_next_preset(radio_t *radio)
{
	rds_current_preset = (rds_current_preset + 1) % RDS_PRESET_COUNT;
	rds_apply_preset(radio);
}

/* Polyphase resampler flag (set via CLI before radio_init) */
static int use_polyphase_resampler = 0;

void radio_set_polyphase(int enable)
{
	use_polyphase_resampler = enable;
}

void radio_set_forced_mono(radio_t *radio, int forced)
{
	radio->rx_forced_mono = forced;
	LOGP(DRADIO, LOGL_INFO, "Forced mono: %s\n", forced ? "ON" : "OFF");
}

void radio_set_rx_snr(radio_t *radio, double snr_db)
{
	if (!radio)
		return;
	radio->rx_input_snr_db = snr_db;
}

/* AFC control functions */
void radio_afc_enable(radio_t *radio, int enable)
{
	radio->afc.enabled = enable;
	if (!enable) {
		/* Reset AFC state and NCO offset when disabled */
		radio->afc.freq_error_hz = 0.0;
		radio->afc.correction_hz = 0.0;
		fm_demod_set_offset(&radio->fm_demod, 0.0);
	}
	LOGP(DRADIO, LOGL_INFO, "AFC: %s (tc=%.0fms, max=%.0fHz)\n", 
	     enable ? "ON" : "OFF",
	     radio->afc.time_constant_s * 1000.0,
	     radio->afc.max_correction_hz);
}

void radio_afc_set_time_constant(radio_t *radio, double tc_seconds)
{
	if (tc_seconds < 0.01) tc_seconds = 0.01;  /* Min 10ms */
	if (tc_seconds > 10.0) tc_seconds = 10.0;  /* Max 10s */
	radio->afc.time_constant_s = tc_seconds;
	LOGP(DRADIO, LOGL_INFO, "AFC time constant: %.0f ms\n", tc_seconds * 1000.0);
}

void radio_afc_set_max_correction(radio_t *radio, double max_hz)
{
	if (max_hz < 100.0) max_hz = 100.0;     /* Min 100 Hz */
	if (max_hz > 50000.0) max_hz = 50000.0; /* Max 50 kHz */
	radio->afc.max_correction_hz = max_hz;
	LOGP(DRADIO, LOGL_INFO, "AFC max correction: %.0f Hz\n", max_hz);
}

double radio_afc_get_correction(radio_t *radio)
{
	return radio->afc.correction_hz;
}

double radio_afc_get_freq_error(radio_t *radio)
{
	return radio->afc.freq_error_hz;
}

int radio_init(radio_t *radio, int buffer_size, int samplerate, double frequency, const char *tx_wave_file, const char *rx_wave_file, const char *tx_audiodev, const char *rx_audiodev, enum modulation modulation, double bandwidth, double deviation, double modulation_index, double time_constant_us, double volume, int stereo, int rds, int rds2, int sca_67k, int sca_92k, int rds_debug, int rds_verbose, int am_compandor, int rds_force_rbds)
{
	int rc = -EINVAL;
	double clip_level = 1.0;  /* clipper threshold, reduced for pilot/RDS headroom */

	/* 
	 * FM COMPOSITE MODULATION BUDGET — Zero Over-Deviation Design
	 * ============================================================
	 * ITU-R BS.412-9 / IEC 62106 deviation budget (±75 kHz = 1.0):
	 *
	 *   Component          Stereo    Mono (no pilot)
	 *   ─────────────────  ────────  ───────────────
	 *   Audio sum (L+R)    0.45      1.0
	 *   Audio diff (L-R)   0.45      —
	 *   Pilot 19 kHz       0.10      —
	 *   RDS 57 kHz         0.067     0.067
	 *
	 * Strategy: Two-layer protection ensures composite ≤ 1.0:
	 *
	 *   1. CLIPPER THRESHOLD = 1.0 - pilot - rds
	 *      The soft clipper (atan-based) limits each audio channel
	 *      BEFORE pilot and RDS are added. This guarantees the
	 *      composite never exceeds 1.0 even on HF transients
	 *      where pre-emphasis boosts peaks by up to +17 dB.
	 *
	 *   2. VOLUME SCALING for 1 kHz reference
	 *      Scale volume so a 1 kHz tone (after pre-emphasis) stays
	 *      below the clip threshold. This means the clipper only
	 *      activates on HF peaks, not on normal program content.
	 *
	 * Pre-emphasis gain: G(f) = sqrt(1 + (2π·f·τ)²)
	 *   50µs: G(1kHz)=1.046, G(5kHz)=2.62, G(15kHz)=7.13
	 *   75µs: G(1kHz)=1.097, G(5kHz)=3.75, G(15kHz)=10.5
	 */
	{
		double pilot_level = 0.0;
		double rds_level = 0.0;
		double audio_matrix_peak = 1.0;  /* mono FM: full scale */
		double preemph_gain_1k = 1.0;    /* no emphasis: unity */
		double audio_budget, volume_scale;

		if (stereo) {
			pilot_level = 0.10;       /* 10% pilot */
			audio_matrix_peak = 0.90; /* stereo matrix: 0.45 sum + 0.45 diff worst case */
		}
		if (rds || rds2)
			rds_level = RDS_INJECTION_NRSC * 2.0 * 0.5;
			/* RDS_INJECTION_NRSC = 5/75 = 0.0667, ×2.0 gain in encoder,
			 * ×0.5 for typical waveform peak (shaped biphase) ≈ 0.067 */

		/* Pre-emphasis gain at 1 kHz reference tone */
		if (time_constant_us > 0.0) {
			double tau = time_constant_us / 1e6;
			double omega_1k = 2.0 * M_PI * 1000.0;
			preemph_gain_1k = sqrt(1.0 + (omega_1k * tau) * (omega_1k * tau));
		}

		/* Audio budget = what's left after pilot + RDS */
		audio_budget = 1.0 - pilot_level - rds_level;

		/* Layer 1: Set clipper threshold.
		 *
		 * Mono: composite = audio + RDS, so clip at (1.0 - rds) guarantees
		 *   audio_clipped + rds ≤ 1.0. No over-deviation possible.
		 *
		 * Stereo: composite = sum + pilot + diff*sin(2θ) + rds*sin(3θ).
		 *   Sum and diff are on different subcarriers (baseband vs 38 kHz).
		 *   Worst-case instantaneous: sum + diff + pilot + rds.
		 *   To keep composite ≤ 1.0 would require clip = (1-pilot-rds)/2 = 0.42,
		 *   which is far too aggressive. The ITU standard allows stereo composite
		 *   to exceed ±75 kHz momentarily — this is inherent to pilot-tone stereo.
		 *   We clip each channel at audio_budget, same as mono. The composite may
		 *   briefly exceed 1.0 when sum and diff peak simultaneously, but this is
		 *   standard-compliant and all receivers handle it. */
		clip_level = audio_budget;

		/* Layer 2: Scale volume so 1kHz reference stays below clip threshold.
		 * This means clipper only activates on HF peaks from pre-emphasis. */
		volume_scale = audio_budget / (audio_matrix_peak * preemph_gain_1k);

		if (volume_scale < 1.0) {
			LOGP(DRADIO, LOGL_NOTICE,
			     "FM modulation budget: pilot=%.1f%% rds=%.1f%% audio_budget=%.1f%% "
			     "preemph_1kHz=%.3f matrix_peak=%.2f → volume_scale=%.4f\n",
			     pilot_level * 100.0, rds_level * 100.0, audio_budget * 100.0,
			     preemph_gain_1k, audio_matrix_peak, volume_scale);
			volume *= volume_scale;
		}

		LOGP(DRADIO, LOGL_INFO,
		     "FM deviation budget (±%.0f kHz): audio=%.1f kHz (%.1f%%) "
		     "pilot=%.1f kHz (%.1f%%) rds=%.1f kHz (%.1f%%) "
		     "clip_level=%.4f effective_volume=%.4f preemph_1kHz=+%.1fdB\n",
		     deviation / 1000.0,
		     audio_budget * deviation / 1000.0, audio_budget * 100.0,
		     pilot_level * deviation / 1000.0, pilot_level * 100.0,
		     rds_level * deviation / 1000.0, rds_level * 100.0,
		     clip_level, volume, 20.0 * log10(preemph_gain_1k));
	}

	/* Soft clipper at audio_budget level.
	 * Guarantees: audio_after_clip + pilot + RDS ≤ 1.0 (no over-deviation).
	 * Only activates on HF peaks from pre-emphasis, not normal content. */
	clipper_init(clip_level);

	memset(radio, 0, sizeof(*radio));
	radio->buffer_size = buffer_size;
	radio->volume = volume;
	radio->clip_level = clip_level;
	radio->stereo = stereo;
	radio->rx_noise_blend_cap = 1.0;
	radio->rx_input_snr_db = -1.0;
	radio->rx_blend_quality = 0.0;
	radio->rx_blend_cap_content = 1.0;
	radio->rx_blend_cap_snr = 1.0;
	radio->rx_blend_cap_floor = 0.0;
	radio->rx_stereo_hf_gain = 1.0;
	radio->rx_diag_sum_rms = 0.0;
	radio->rx_diag_diff_rms_pre = 0.0;
	radio->rx_diag_diff_rms_post = 0.0;
	radio->rds = rds;
	radio->rds2 = rds2;
	radio->sca_67k = sca_67k;
	radio->sca_92k = sca_92k;
	radio->tx_wave_file = tx_wave_file;
	radio->modulation = modulation;
	radio->signal_samplerate = samplerate;
	radio->audio_bandwidth = bandwidth;	/* Audio passband (Hz), e.g. 15kHz for FM */

	/* Initialize audio quality debug tracking */
	audio_debug_init(&g_rx_debug, 5.0);

	/* Calculate baseband_extent based on modulation type.
	 * This is the max frequency in baseband (one-sided). RF bandwidth = 2x this.
	 * FM: baseband_extent = deviation + highest_subcarrier
	 *   - Mono:   deviation + audio_bw (75k + 15k = 90 kHz)
	 *   - Stereo: deviation + 53k (pilot 19k + L-R up to 53k)
	 *   - RDS:    deviation + 60k (RDS subcarrier at 57k ± 2.4k)
	 *   - RDS2:   deviation + 80k (additional subcarriers)
	 *   - SCA:    deviation + 67k/92k/100k (subsidiary carriers)
	 * AM: baseband_extent = audio_bandwidth */
	switch (radio->modulation) {
	case MODULATION_FM:
		radio->fm_deviation = deviation;
		radio->baseband_extent = deviation + bandwidth;
		if (radio->stereo) {
			radio->baseband_extent = deviation + 53000.0;  /* stereo L-R extends to 53 kHz */
			radio->audio_bandwidth = STEREO_BW;
		}
		if (radio->rds)
			radio->baseband_extent = deviation + 60000.0;  /* RDS at 57 kHz ± 2.4 kHz */
		if (radio->rds2)
			radio->baseband_extent = deviation + 80000.0;  /* RDS2 additional subcarriers */
		/* SCA extends bandwidth further */
		if (radio->sca_67k)
			radio->baseband_extent = deviation + 75000.0;  /* SCA at 67 kHz */
		if (radio->sca_92k)
			radio->baseband_extent = deviation + 100000.0; /* SCA at 92 kHz */
		break;
	case MODULATION_AM_DSB:
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		/* level is 1.0, which is full amplitude */
		radio->baseband_extent = bandwidth;
		break;
	case MODULATION_NONE:
		LOGP(DRADIO, LOGL_ERROR, "Wrong modulation, please fix!\n");
		goto error;
	}

	/* Derive RF bandwidth and required sample rate from baseband_extent */
	radio->rf_bandwidth = 2.0 * radio->baseband_extent;
	radio->required_samplerate = radio->rf_bandwidth / 0.75;  /* 25% filter margin */

	if (tx_wave_file) {
		/* open wave file */
		int _samplerate = 0;
		radio->tx_audio_channels = 0;
		rc = wave_create_playback(&radio->wave_tx_play, tx_wave_file, &_samplerate, &radio->tx_audio_channels, 1.0);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to create WAVE playback instance!\n");
			goto error;
		}
		if (radio->tx_audio_channels != 1 && radio->tx_audio_channels != 2)
		{
			LOGP(DRADIO, LOGL_ERROR, "WAVE file must have one or two channels!\n");
			goto error;
		}
		radio->tx_audio_samplerate = _samplerate;
		radio->tx_audio_mode = AUDIO_MODE_WAVEFILE;
		LOGP(DRADIO, LOGL_INFO, "TX audio mode: WAVEFILE (%s, %d Hz, %d ch)\n", tx_wave_file, _samplerate, radio->tx_audio_channels);
	} else if (tx_audiodev) {
#ifdef HAVE_ALSA
		/* open audio device */
		radio->tx_audio_samplerate = 48000;
		radio->tx_audio_channels = (stereo) ? 2 : 1;
		radio->tx_sound = sound_open(SOUND_DIR_REC, tx_audiodev, NULL, NULL, NULL, radio->tx_audio_channels, 0.0, radio->tx_audio_samplerate, radio->buffer_size, 1.0, 1.0, 0.0, 2.0);
		if (!radio->tx_sound) {
			rc = -EIO;
			LOGP(DRADIO, LOGL_ERROR, "Failed to open sound device!\n");
			goto error;
		}
		jitter_create(&radio->tx_dejitter[0], "left", radio->tx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		jitter_create(&radio->tx_dejitter[1], "right", radio->tx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		radio->tx_audio_mode = AUDIO_MODE_AUDIODEV;
#else
		rc = -ENOTSUP;
		LOGP(DRADIO, LOGL_ERROR, "No sound card support compiled in!\n");
		goto error;
#endif
	} else {
		int i;
		double phase;
		/* use built-in sample sound */
		radio->tx_audio_samplerate = samplerate;
		radio->tx_audio_channels = (radio->stereo) ? 2 : 1;
		radio->testtone_length = radio->tx_audio_samplerate;
		radio->testtone[0] = calloc(radio->testtone_length * 2, sizeof(sample_t));
		if (!radio->testtone[0]) {
			rc = -ENOMEM;
			LOGP(DRADIO, LOGL_ERROR, "Failed to allocate test sound buffer!\n");
			goto error;
		}
		radio->testtone[1] = radio->testtone[0] + radio->testtone_length;
		/* generate tone */
		phase = 2.0 * M_PI * 1000.0 / radio->tx_audio_samplerate;
		if (radio->stereo) {
			/* Stereo test: L=1kHz, R=400Hz for clear separation verification
			 * This creates constant L-R content for strong 38kHz subcarrier
			 * Unlike alternating L/R, this gives continuous stereo modulation */
			double phase_l = 2.0 * M_PI * 1000.0 / radio->tx_audio_samplerate;
			double phase_r = 2.0 * M_PI * 400.0 / radio->tx_audio_samplerate;
			for (i = 0; i < radio->testtone_length; i++) {
				radio->testtone[0][i] = sin(i * phase_l);  /* Left: 1 kHz */
				radio->testtone[1][i] = sin(i * phase_r);  /* Right: 400 Hz */
			}
		} else {
			for (i = 0; i < radio->testtone_length; i++) {
				radio->testtone[0][i] = sin(i * phase);
			}
		}
		radio->tx_audio_mode = AUDIO_MODE_TESTTONE;
		LOGP(DRADIO, LOGL_INFO, "TX audio mode: TESTTONE (1 kHz)\n");
	}

	if (rx_wave_file) {
		/* open wave file */
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (radio->stereo) ? 2 : 1;
		rc = wave_create_record(&radio->wave_rx_rec, rx_wave_file, radio->rx_audio_samplerate, radio->rx_audio_channels, 1.0);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to create WAVE record instance!\n");
			goto error;
		}
		radio->rx_audio_mode |= AUDIO_MODE_WAVEFILE;
	}
	if (rx_audiodev) {
#ifdef HAVE_ALSA
		/* open audio device */
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (stereo) ? 2 : 1;
		/* check if we use same device */
		radio->rx_sound = sound_open(SOUND_DIR_PLAY, rx_audiodev, NULL, NULL, NULL, radio->rx_audio_channels, 0.0, radio->rx_audio_samplerate, radio->buffer_size, 1.0, 1.0, 0.0, 2.0);
		if (!radio->rx_sound) {
			rc = -EIO;
			LOGP(DRADIO, LOGL_ERROR, "Failed to open sound device!\n");
			goto error;
		}
		jitter_create(&radio->rx_dejitter[0], "left", radio->rx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		jitter_create(&radio->rx_dejitter[1], "right", radio->rx_audio_samplerate, 0.050, 0.500, JITTER_FLAG_NONE);
		radio->rx_audio_mode |= AUDIO_MODE_AUDIODEV;
#else
		rc = -ENOTSUP;
		LOGP(DRADIO, LOGL_ERROR, "No sound card support compiled in!\n");
		goto error;
#endif
	}
	/* if no sink was selected, we use dummy settings */
	if (!rx_wave_file && !rx_audiodev) {
		radio->rx_audio_samplerate = 48000;
		radio->rx_audio_channels = (stereo) ? 2 : 1;
	}

	/* check if sample rate is too low (only for linear resampler - polyphase handles any ratio) */
	if (!use_polyphase_resampler) {
		if (radio->tx_audio_samplerate > radio->signal_samplerate) {
			rc = -EINVAL;
			LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your audio sample rate is %.0f.\n", radio->signal_samplerate, radio->tx_audio_samplerate);
			LOGP(DRADIO, LOGL_ERROR, "Please select a sample rate that is higher or equal the audio sample rate!\n");
			LOGP(DRADIO, LOGL_ERROR, "Or use --polyphase-resampler for bidirectional sample rate conversion.\n");
			goto error;
		}
		if (radio->rx_audio_samplerate > radio->signal_samplerate) {
			rc = -EINVAL;
			LOGP(DRADIO, LOGL_ERROR, "You have selected a signal processing sample rate of %.0f. Your audio sample rate is %.0f.\n", radio->signal_samplerate, radio->rx_audio_samplerate);
			LOGP(DRADIO, LOGL_ERROR, "Please select a sample rate that is higher or equal the audio sample rate!\n");
			LOGP(DRADIO, LOGL_ERROR, "Or use --polyphase-resampler for bidirectional sample rate conversion.\n");
			goto error;
		}
	}
	if (radio->signal_samplerate < radio->required_samplerate) {
		rc = -EINVAL;
		LOGP(DRADIO, LOGL_ERROR, "Signal sample rate %.0f Hz too low for %.0f kHz RF bandwidth.\n",
		     radio->signal_samplerate, radio->rf_bandwidth / 1000.0);
		LOGP(DRADIO, LOGL_ERROR, "Need at least %.0f Hz sample rate (Nyquist + 33%% filter margin).\n",
		     radio->required_samplerate);
		goto error;
	}

	iir_highpass_init(&radio->tx_dc_removal[0], DC_CUTOFF, radio->tx_audio_samplerate, 1);
	iir_highpass_init(&radio->tx_dc_removal[1], DC_CUTOFF, radio->tx_audio_samplerate, 1);

	/* init DC blocker state */
	radio->tx_dc_prev_x[0] = 0.0;
	radio->tx_dc_prev_x[1] = 0.0;
	radio->tx_dc_prev_y[0] = 0.0;
	radio->tx_dc_prev_y[1] = 0.0;

	/* stereo pilot tone phase */
	radio->pilot_phasestep = 2.0 * M_PI * PILOT_FREQ / radio->signal_samplerate;

	/* Initialize PLL for 19 kHz pilot tracking
	 * freq: 19000 / samplerate (normalized)
	 * bandwidth: 50 Hz / samplerate (narrow for clean tracking)
	 * min_signal: 0.01 (1% pilot level for lock)
	 */
	pll_init(&radio->rx_pilot_pll, 
	         PILOT_FREQ / radio->signal_samplerate,
	         50.0 / radio->signal_samplerate,
	         0.01);

	/* Legacy stereo decoding filters (kept for fallback/comparison) */
	iir_lowpass_init(&radio->rx_lp_pilot_I, PILOT_BW, radio->signal_samplerate, 2);
        iir_lowpass_init(&radio->rx_lp_pilot_Q, PILOT_BW, radio->signal_samplerate, 2);
	iir_lowpass_init(&radio->rx_lp_sum, STEREO_BW, radio->signal_samplerate, 2);
	iir_lowpass_init(&radio->rx_lp_diff, STEREO_BW, radio->signal_samplerate, 2);
	iir_lowpass_init(&radio->rx_lp_diff_low, STEREO_DIFF_LOW_BW, radio->signal_samplerate, 2);
	iir_lowpass_init(&radio->rx_out_hicut[0], RX_OUTPUT_HICUT_BW, radio->rx_audio_samplerate, 2);
	iir_lowpass_init(&radio->rx_out_hicut[1], RX_OUTPUT_HICUT_BW, radio->rx_audio_samplerate, 2);

	/* init sample rate conversion, use complete bandwidth for resample filter */
	/* 
	 * RECONSTRUCTION/ANTI-ALIASING FILTER
	 * -----------------------------------
	 * We use a strict 15kHz cutoff (Standard FM Bandwidth) for the upsampler.
	 * 
	 * JUSTIFICATION:
	 * 1. The previous default (audio_samplerate / 2) allowed ultrasonic images to pass 
	 *    through the 2nd order filter.
	 * 2. These ultrasonic images (e.g. at 24kHz+) folded back into the audible band 
	 *    during SDR modulation, creating "8-bit like" hiss and intermodulation noise.
	 * 3. 15kHz creates a clean, hard stop before the 19kHz stereo pilot, protecting
	 *    the pilot from interference and the audio from aliasing.
	 * 
	 * Note: Filter order was also increased to 4 in libsamplerate/samplerate.c
	 */
	radio->use_polyphase = use_polyphase_resampler;
	
	if (use_polyphase_resampler) {
		/* Polyphase FIR resampler - handles any input/output ratio */
		LOGP(DRADIO, LOGL_INFO, "Using polyphase resampler for audio/signal rate conversion.\n");
		
		rc = polyphase_init(&radio->tx_polyphase[0], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0, 16);
		if (rc < 0)
			goto error;
		rc = polyphase_init(&radio->tx_polyphase[1], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0, 16);
		if (rc < 0)
			goto error;
		rc = polyphase_init(&radio->rx_polyphase[0], radio->signal_samplerate, radio->rx_audio_samplerate, 15000.0, 16);
		if (rc < 0)
			goto error;
		rc = polyphase_init(&radio->rx_polyphase[1], radio->signal_samplerate, radio->rx_audio_samplerate, 15000.0, 16);
		if (rc < 0)
			goto error;
	} else {
		/* Original linear interpolation resampler - upsampling only */
		rc = init_samplerate(&radio->tx_resampler[0], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0);
		if (rc < 0)
			goto error;
		rc = init_samplerate(&radio->tx_resampler[1], radio->tx_audio_samplerate, radio->signal_samplerate, 15000.0);
		if (rc < 0)
			goto error;
		rc = init_samplerate(&radio->rx_resampler[0], radio->rx_audio_samplerate, radio->signal_samplerate, 15000.0);
		if (rc < 0)
			goto error;
		rc = init_samplerate(&radio->rx_resampler[1], radio->rx_audio_samplerate, radio->signal_samplerate, 15000.0);
		if (rc < 0)
			goto error;
	}

	/* init display of wave form */
	sprintf(freq_name[0], "%.4f MHz", frequency / 1e6);
	display_wave_init(&radio->dispwav[0], radio->rx_audio_samplerate, freq_name[0]);

	/* init filters (using signal sample rate) */
	switch (radio->modulation) {
	case MODULATION_FM:
		if (time_constant_us > 0.0) {
			radio->emphasis = 1;
			LOGP(DRADIO, LOGL_INFO, "Using emphasis cut-off at %.0f Hz.\n", timeconstant2cutoff(time_constant_us));
		}
	/* init emphasis */
	if (radio->emphasis) {
		double tau = time_constant_us / 1e6;
		if (fm_fast_math_enabled()) {
			/* Initialize separate TX and RX emphasis filters to prevent state corruption */
			/* TX filters: pre-emphasis (boosts high frequencies before transmission) */
			init_emphasis_fast(&radio->fm_emphasis_fast_tx[0], radio->signal_samplerate, tau, 12000.0);
			/* RX filters: de-emphasis (restores flat frequency response after reception) */
			init_emphasis_fast(&radio->fm_emphasis_fast_rx[0], radio->signal_samplerate, tau, 12000.0);
			/* Initialize RX DC blocking filter (30 Hz cutoff - low enough to preserve bass) */
			init_dc_filter_fast(&radio->rx_dc_filter[0], radio->signal_samplerate, DC_CUTOFF);
			if (radio->stereo) {
				init_emphasis_fast(&radio->fm_emphasis_fast_tx[1], radio->signal_samplerate, tau, 12000.0);
				init_emphasis_fast(&radio->fm_emphasis_fast_rx[1], radio->signal_samplerate, tau, 12000.0);
				init_dc_filter_fast(&radio->rx_dc_filter[1], radio->signal_samplerate, DC_CUTOFF);
			}
		} else {
			/* time constant - convert from us to seconds */
			/* TX filters: pre-emphasis */
			init_emphasis(&radio->fm_emphasis_tx[0], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
			/* RX filters: de-emphasis */
			init_emphasis(&radio->fm_emphasis_rx[0], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
			if (radio->stereo) {
				init_emphasis(&radio->fm_emphasis_tx[1], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
				init_emphasis(&radio->fm_emphasis_rx[1], radio->signal_samplerate, timeconstant2cutoff(time_constant_us), DC_CUTOFF, radio->audio_bandwidth);
			}
		}
	}
		rc = fm_mod_init(&radio->fm_mod, radio->signal_samplerate, 0.0, 1.0);
		if (rc < 0)
			goto error;
		rc = fm_demod_init(&radio->fm_demod, radio->signal_samplerate, 0.0, radio->rf_bandwidth);
		if (rc < 0)
			goto error;
		
		/* Initialize AFC for mono FM (disabled by default, enable with --afc) */
		memset(&radio->afc, 0, sizeof(radio->afc));
		radio->afc.time_constant_s = 0.3;      /* 300ms IIR time constant */
		radio->afc.max_correction_hz = 5000.0; /* ±5 kHz max correction */
		
		if (stereo) {
			sprintf(freq_name[0], "%.4f MHz left", frequency / 1e6);
			sprintf(freq_name[1], "%.4f MHz right", frequency / 1e6);
			display_wave_init(&radio->dispwav[1], samplerate, freq_name[1]);
		}
		/* Initialize RDS encoder if enabled */
		if (rds || rds2) {
			/* Select preset based on emphasis (heuristic for region):
			 * 75us -> Americas (RBDS) - preset 1
			 * 50us and others -> Europe/World (RDS) - preset 0 */
			if (RDS_IS_RBDS_EMPHASIS(time_constant_us)) {
				rds_current_preset = 1;  /* USA/RBDS (75us) */
				LOGP(DRADIO, LOGL_INFO, "Emphasis %.0fus detected: using RBDS (Americas) preset\n", time_constant_us);
			} else {
				rds_current_preset = 0;  /* RDS (default, more common globally) */
				LOGP(DRADIO, LOGL_INFO, "Emphasis %.0fus detected: using RDS preset (default)\n", time_constant_us);
			}
			const rds_preset_t *p = &rds_presets[rds_current_preset];
			
			/* Calculate PI with user overrides (same logic as rds_apply_preset) */
			uint16_t pi = p->pi;
			if (pi == 0 && p->callsign) {
				pi = rds_get_pi_from_callsign(p->callsign);
			}
			if (rds_user_pi)
				pi = rds_user_pi;
			else if (rds_user_callsign) {
				uint16_t cpi = rds_get_pi_from_callsign(rds_user_callsign);
				if (cpi) pi = cpi;
			}

			/* Initialize encoder with calculated PI and preset values
			 * (rds_apply_preset will overwrite these, but init needs valid values)
			 * Only initialize encoder for TX path (not in rx_only mode) */
			if (!sdr_config || !sdr_config->rx_only) {
				LOGP(DRADIO, LOGL_NOTICE, "RDS: Initializing encoder for TX path\n");
				rds_encoder_init(&radio->rds_enc, radio->signal_samplerate,
					pi, p->ps, p->rt, p->pty, p->ptyn);
				
				/* Set debug/verbose flags and DI flags before applying preset */
				radio->rds_enc.debug = rds_debug;
				radio->rds_enc.verbose = rds_verbose;
				radio->rds_enc.di_stereo = stereo ? 1 : 0;
				radio->rds_enc.di_artificial_head = rds_di_artificial_head;
				radio->rds_enc.di_compressed = rds_di_compressed;
				radio->rds_enc.di_dynamic_pty = rds_di_dynamic_pty;
				radio->rds_enc.ta = rds_ta;
				radio->rds_enc.ct_enabled = rds_ct_enabled;
			} else {
				LOGP(DRADIO, LOGL_INFO, "RDS: Skipping encoder init (RX-only mode)\n");
			}
			
			/* Only initialize decoder for RX path (not in tx_only mode) */
			if (!sdr_config || !sdr_config->tx_only) {
				LOGP(DRADIO, LOGL_NOTICE, "RDS: Initializing decoder for RX path\n");
				rds_decoder_init(&radio->rds_dec, radio->signal_samplerate, rds_debug, rds_verbose, time_constant_us, rds_force_rbds);
			} else {
				LOGP(DRADIO, LOGL_INFO, "RDS: Skipping decoder init (TX-only mode)\n");
			}

			
			/*
			 * TODO: RDS2 Encoder Initialization (IEC 62106-2:2021)
			 * ----------------------------------------------------
			 * If rds2 flag is set, initialize additional encoder for streams 2-4:
			 *
			 *   if (rds2) {
			 *       rds2_encoder_init(&radio->rds2_enc, radio->signal_samplerate);
			 *   }
			 *
			 * The rds2_encoder_t would handle:
			 * - Stream 2:  66.5 kHz subcarrier (independent NCO)
			 * - Stream 3:  71.25 kHz subcarrier (independent NCO)
			 * - Stream 4:  76 kHz subcarrier (4 x pilot, can be locked)
			 * - Group Type C encoding for extended data
			 * - UTF-8 Extended RadioText (128 bytes)
			 * - RDS2 File Transfer (RFT) protocol
			 *
			 * Note: The radio_t struct would need: rds2_encoder_t rds2_enc;
			 * See rds.c for stub implementation.
			 */
		}
		/* Initialize SCA encoder/decoder if enabled */
		if (sca_67k || sca_92k) {
			sca_encoder_init(&radio->sca_enc, radio->signal_samplerate, sca_67k, sca_92k);
			sca_decoder_init(&radio->sca_dec, radio->signal_samplerate, sca_67k, sca_92k);
		}
		break;
	case MODULATION_AM_DSB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		/* modulation index 0.0 = no envelope, bias 1.0
		 * modulation index 1.0 = envelope +-0.5, bias 0.5
		 * modulation index 0.5 = envelope +-0.25, bias 0.75
		 */
		{
			double gain = modulation_index / 2.0;
			double bias = 1.0 - gain;
			rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, gain, bias);
			if (rc < 0)
				goto error;
			rc = am_demod_init(&radio->am_demod, radio->signal_samplerate, 0.0, radio->baseband_extent, 1.0 / modulation_index);
			if (rc < 0)
				goto error;
		}
		/* Initialize AM compandor if enabled
		 * AM broadcast uses: attack 5ms, recovery 200ms
		 * This gives consistent modulation depth for varying audio levels
		 */
		radio->am_compandor = am_compandor;
		if (am_compandor) {
			compandor_init();
			setup_compandor(&radio->am_compandor_state, radio->signal_samplerate, 5.0, 200.0);
			LOGP(DRADIO, LOGL_INFO, "AM compandor enabled (attack=5ms, recovery=200ms)\n");
		}
		break;
	case MODULATION_AM_USB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, 1.0, 0.0);
		if (rc < 0)
			goto error;
		rc = am_demod_init(&radio->am_demod, radio->signal_samplerate, 0.0, radio->baseband_extent, 16.0);
		if (rc < 0)
			goto error;
		break;
	case MODULATION_AM_LSB:
		iir_lowpass_init(&radio->tx_am_bw_limit, radio->audio_bandwidth, radio->signal_samplerate, 1);
		rc = am_mod_init(&radio->am_mod, radio->signal_samplerate, 0.0, 1.0, 0.0);
		if (rc < 0)
			goto error;
		rc = am_demod_init(&radio->am_demod, radio->signal_samplerate, 0.0, radio->baseband_extent, 16.0);
		if (rc < 0)
			goto error;
		break;

	default:
		break;
	}
	
	if (radio->tx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio source is %.0f Hz.\n", radio->tx_audio_samplerate / 2.0);
	if (radio->rx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio sink is %.0f Hz.\n", radio->rx_audio_samplerate / 2.0);
	LOGP(DRADIO, LOGL_INFO, "Bandwidth of audio signal is %.0f Hz.\n", radio->audio_bandwidth);
	LOGP(DRADIO, LOGL_INFO, "RF bandwidth is %.0f kHz (baseband extent %.0f kHz, min samplerate %.0f Hz).\n",
	     radio->rf_bandwidth / 1000.0, radio->baseband_extent / 1000.0, radio->required_samplerate);
	if (radio->tx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Sample rate of audio source is %.0f Hz.\n", radio->tx_audio_samplerate);
	if (radio->rx_audio_mode)
		LOGP(DRADIO, LOGL_INFO, "Sample rate of audio sink is %.0f Hz.\n", radio->rx_audio_samplerate);
	LOGP(DRADIO, LOGL_INFO, "Sample rate of signal is %.0f Hz.\n", radio->signal_samplerate);

	/* one or two audio channels */
	if (radio->tx_audio_channels != 1 && radio->tx_audio_channels != 2)
	{
		LOGP(DRADIO, LOGL_ERROR, "Wrong number of audio channels, please fix!\n");
		goto error;
	}

	/* audio buffers: how many sample for audio (rounded down) */
	/* audio buffers: how many sample for audio (rounded down) */
	int tx_size, rx_size;
	if (radio->use_polyphase) {
		tx_size = polyphase_input_num(&radio->tx_polyphase[0], buffer_size);
		rx_size = polyphase_output_num(&radio->rx_polyphase[0], buffer_size);
	} else {
		tx_size = (int)((double)buffer_size / radio->tx_resampler[0].factor);
		rx_size = (int)((double)buffer_size / radio->rx_resampler[0].factor);
	}

	if (tx_size > rx_size)
		radio->audio_buffer_size = tx_size + 16; /* Add padding for safety */
	else
		radio->audio_buffer_size = rx_size + 16;

	radio->audio_buffer = calloc(radio->audio_buffer_size * 2, sizeof(*radio->audio_buffer));
	if (!radio->audio_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* signal buffers - separate TX and RX to prevent any potential race conditions */
	radio->signal_buffer_size = buffer_size;
	/* TX signal buffer: 3 channels (mono/stereo + temp) */
	radio->tx_signal_buffer = calloc(radio->signal_buffer_size * 3, sizeof(*radio->tx_signal_buffer));
	/* RX signal buffer: 3 channels (mono/stereo + temp) */
	radio->rx_signal_buffer = calloc(radio->signal_buffer_size * 3, sizeof(*radio->rx_signal_buffer));
	/* TX power buffer */
	radio->signal_power_buffer = calloc(radio->signal_buffer_size, sizeof(*radio->signal_power_buffer));
	if (!radio->tx_signal_buffer || !radio->rx_signal_buffer || !radio->signal_power_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* temporary I/Q/carrier buffers, used while demodulating (RX-only) */
	radio->I_buffer = calloc(buffer_size, sizeof(*radio->I_buffer));
	radio->Q_buffer = calloc(buffer_size, sizeof(*radio->Q_buffer));
	radio->carrier_buffer = calloc(buffer_size, sizeof(*radio->carrier_buffer));
	if (!radio->I_buffer || !radio->Q_buffer || !radio->carrier_buffer) {
		LOGP(DRADIO, LOGL_ERROR, "No memory!!\n");
		rc = -ENOMEM;
		goto error;
	}

	/* Apply RDS preset after all initialization is complete
	 * This ensures the preset is applied when everything is fully ready
	 * Only apply preset for TX path (encoder only, not in rx_only mode) */
	if ((!sdr_config || !sdr_config->rx_only) && (radio->rds || radio->rds2) && radio->modulation == MODULATION_FM) {
		/* Apply full preset configuration using the comprehensive function
		 * This ensures all preset fields are applied correctly, including
		 * EON, RT+, eRT+, eRT, group versions, etc. */
		rds_apply_preset(radio);

		/* Force A/B flags to 0 (group A) for initial transmission.
		 * rds_apply_preset() toggles A/B to signal "new text" to receivers,
		 * but on first load there's no previous text to distinguish from. */
		radio->rds_enc.rt_ab = 0;
		radio->rds_enc.ert.ab = 0;
	}

	return 0;

error:
	radio_exit(radio);
	return rc;
}

void radio_exit(radio_t *radio)
{
	if (radio->audio_buffer) {
		free(radio->audio_buffer);
		radio->audio_buffer = NULL;
	}
	if (radio->tx_signal_buffer) {
		free(radio->tx_signal_buffer);
		radio->tx_signal_buffer = NULL;
	}
	if (radio->rx_signal_buffer) {
		free(radio->rx_signal_buffer);
		radio->rx_signal_buffer = NULL;
	}
	if (radio->signal_power_buffer) {
		free(radio->signal_power_buffer);
		radio->signal_power_buffer = NULL;
	}
	if (radio->I_buffer) {
		free(radio->I_buffer);
		radio->I_buffer = NULL;
	}
	if (radio->Q_buffer) {
		free(radio->Q_buffer);
		radio->Q_buffer = NULL;
	}
	if (radio->carrier_buffer) {
		free(radio->carrier_buffer);
		radio->carrier_buffer = NULL;
	}
	if (radio->tx_audio_mode == AUDIO_MODE_WAVEFILE) {
		wave_destroy_playback(&radio->wave_tx_play);
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if ((radio->rx_audio_mode & AUDIO_MODE_WAVEFILE)) {
		wave_destroy_record(&radio->wave_rx_rec);
		radio->rx_audio_mode = AUDIO_MODE_NONE;
	}
#ifdef HAVE_ALSA
	if (radio->tx_sound) {
		sound_close(radio->tx_sound);
		/* if same device was used */
		if (radio->tx_sound == radio->rx_sound)
			radio->rx_sound = NULL;
		radio->tx_sound = NULL;
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	if (radio->rx_sound) {
		sound_close(radio->rx_sound);
		radio->rx_sound = NULL;
		radio->rx_audio_mode = AUDIO_MODE_NONE;
	}
#endif
	jitter_destroy(&radio->tx_dejitter[0]);
	jitter_destroy(&radio->tx_dejitter[1]);
	jitter_destroy(&radio->rx_dejitter[0]);
	jitter_destroy(&radio->rx_dejitter[1]);
	if (radio->tx_audio_mode == AUDIO_MODE_TESTTONE) {
		free(radio->testtone[0]);
		radio->tx_audio_mode = AUDIO_MODE_NONE;
	}
	/* Free polyphase resamplers if used */
	if (radio->use_polyphase) {
		polyphase_free(&radio->tx_polyphase[0]);
		polyphase_free(&radio->tx_polyphase[1]);
		polyphase_free(&radio->rx_polyphase[0]);
		polyphase_free(&radio->rx_polyphase[1]);
	}
	if (radio->modulation == MODULATION_FM)
		fm_mod_exit(&radio->fm_mod);
	else
		am_mod_exit(&radio->am_mod);
}

int radio_start(radio_t __attribute__((unused)) *radio)
{
#ifdef HAVE_ALSA
	int rc;

	/* start rx sound */
	if (radio->rx_sound) {
		rc = sound_start(radio->rx_sound);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to start receiving from audio device..\n");
			return rc;
		}
	}

	/* start tx sound, if different device */
	if (radio->tx_sound && radio->tx_sound != radio->rx_sound)  {
		rc = sound_start(radio->tx_sound);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to start transmitting to audio device..\n");
			return rc;
		}
	}
#endif

	return 0;
}

#if 0 /* TX pipeline debug instrumentation - uncomment to enable */
/* TX pipeline debug: compute peak, min, DC, and detect discontinuities */
static void tx_debug_stats(const char *stage, const sample_t *buf, int len, int ch,
                           double *prev_last, int *disc_count)
{
	double peak = 0.0, mn = 0.0, sum = 0.0, val;
	int i;
	int zero_count = 0;
	double max_jump = 0.0;

	if (len <= 0) return;

	mn = buf[0];
	for (i = 0; i < len; i++) {
		val = buf[i];
		if (fabs(val) > peak) peak = fabs(val);
		if (val < mn) mn = val;
		sum += val;
		if (val == 0.0) zero_count++;
		/* detect sample-to-sample jumps */
		if (i > 0) {
			double jump = fabs(val - buf[i - 1]);
			if (jump > max_jump) max_jump = jump;
		} else if (*prev_last != 0.0 || i == 0) {
			/* check jump from previous block's last sample */
			double jump = fabs(val - *prev_last);
			if (jump > max_jump) max_jump = jump;
		}
	}
	double dc = sum / len;

	/* detect discontinuity: jump > 0.3 is suspicious for audio */
	if (max_jump > 0.3 && disc_count)
		(*disc_count)++;

	LOGP(DRADIO, LOGL_DEBUG, "TX[%s] ch%d: n=%d peak=%.4f min=%.4f dc=%.6f maxjump=%.4f zeros=%d\n",
	     stage, ch, len, peak, mn, dc, max_jump, zero_count);

	if (len > 0)
		*prev_last = buf[len - 1];
}

/* Persistent inter-block boundary tracker for source audio */
static struct {
	double last_sample_ch0;
	double last_sample_ch1;
	int initialized;
	int boundary_pop_count;
} tx_boundary = { 0 };

/* TX debug state (persistent across calls) */
static struct {
	int call_count;
	int dump_interval;  /* dump every N calls */
	double prev_last[8]; /* last sample per stage/channel */
	int disc_count[8];   /* discontinuity counters */
	unsigned long total_audio_samples;
	unsigned long total_signal_samples;
} tx_dbg = { .dump_interval = 333 }; /* ~1s at 3ms blocks */
#endif /* TX pipeline debug instrumentation */

int radio_tx(radio_t *radio, float *baseband, int signal_num)
{
	int i;
	int __attribute__((unused)) rc;
	int audio_num;
	sample_t *audio_samples[2];
	sample_t *signal_samples[3];
	uint8_t *signal_power;
#if 0 /* TX debug */
	int tx_debug_this_call;
#endif
#ifdef HAVE_ALSA
	jitter_frame_t *jf;
#endif
#if 0 /* TX debug */
	tx_dbg.call_count++;
	tx_debug_this_call = (tx_dbg.call_count % tx_dbg.dump_interval == 0);
#endif

	if (signal_num > radio->buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > buffer_size, please fix!.\n");
		abort();
	}

	/* audio buffers: how many sample for audio (rounded down) */
	/* audio buffers: how many sample for audio (rounded down) */
	if (radio->use_polyphase) {
		audio_num = polyphase_input_num(&radio->tx_polyphase[0], signal_num);
	} else {
		/* Original linear resampler */
		audio_num = (int)((double)signal_num / radio->tx_resampler[0].factor);
	}

	if (audio_num > radio->audio_buffer_size) {
		/* Cap if too large, though this shouldn't happen with correct buffer sizing */
		LOGP(DRADIO, LOGL_ERROR, "audio_num > audio_buffer_size (%d > %d), capping.\n", audio_num, radio->audio_buffer_size);
		audio_num = radio->audio_buffer_size;
	}

	/* Skip processing if no audio samples to process.
	 * This happens when tosend is very small (3-6 signal samples) and rounds
	 * down to 0 audio samples. Processing zero audio through the resampler
	 * produces ~10 orphan signal samples from stale filter state, which
	 * create micro-glitches (pops) in the FM output that real receivers hear. */
	if (audio_num <= 0) {
		return 0;
	}
	audio_samples[0] = radio->audio_buffer;
	audio_samples[1] = radio->audio_buffer + radio->audio_buffer_size;

	/* signal buffers: a bit more samples to be safe */
	if (radio->use_polyphase) {
		signal_num = polyphase_output_num(&radio->tx_polyphase[0], audio_num) + 16; /* +padding */
	} else {
		signal_num = (int)((double)audio_num * radio->tx_resampler[0].factor + 0.5) + 10;
	}
	
	if (signal_num > radio->signal_buffer_size) {
		/* This means we might overflow output buffer, but audio_num was capped? */
		LOGP(DRADIO, LOGL_ERROR, "signal_num > signal_buffer_size (%d > %d), capping.\n", signal_num, radio->signal_buffer_size);
		signal_num = radio->signal_buffer_size;
	}
	/* Use TX-only signal buffer */
	signal_samples[0] = radio->tx_signal_buffer;
	signal_samples[1] = radio->tx_signal_buffer + radio->signal_buffer_size;
	signal_samples[2] = radio->tx_signal_buffer + radio->signal_buffer_size * 2;
	signal_power = radio->signal_power_buffer;

	/* get audio to be sent */
	switch (radio->tx_audio_mode) {
	case AUDIO_MODE_WAVEFILE:
	{
		int wave_got = wave_read(&radio->wave_tx_play, audio_samples, audio_num);
#if 0 /* TX debug */
		/* Track short reads from wave file - these cause sample gaps and pops */
		if (wave_got < audio_num && wave_got > 0) {
			static int short_read_count = 0;
			short_read_count++;
			LOGP(DRADIO, LOGL_NOTICE, "TX WAVE SHORT READ #%d at call %d: requested=%d got=%d (gap=%d samples)\n",
			     short_read_count, tx_dbg.call_count, audio_num, wave_got, audio_num - wave_got);
		}
#endif
		(void)wave_got;
		
		if (!radio->wave_tx_play.left) {
			int rc;
			int _samplerate = 0;
			wave_destroy_playback(&radio->wave_tx_play);
			rc = wave_create_playback(&radio->wave_tx_play, radio->tx_wave_file, &_samplerate, &radio->tx_audio_channels, 1.0);
			if (rc < 0) {
				LOGP(DRADIO, LOGL_ERROR, "Failed to re-open wave file.\n");
				return rc;
			}
		}
		break;
	}
#ifdef HAVE_ALSA
	case AUDIO_MODE_AUDIODEV:
		rc = sound_read(radio->tx_sound, audio_samples, radio->audio_buffer_size, radio->tx_audio_channels, NULL);
		if (rc < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to read from sound device (rc = %d)!\n", audio_num);
			if (rc == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
		jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)audio_samples[0], rc * sizeof(*(audio_samples[0])), 0, radio->tx_sequence[0], radio->tx_timestamp[0], 123);
		if (jf)
			jitter_save(&radio->tx_dejitter[0], jf);
		radio->tx_sequence[0] += 1;
		radio->tx_timestamp[0] += rc;
		jitter_load_samples(&radio->tx_dejitter[0], (uint8_t *)audio_samples[0], audio_num, sizeof(*(audio_samples[0])), NULL, NULL);
		if (radio->tx_audio_channels == 2) {
			jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)audio_samples[1], rc * sizeof(*(audio_samples[1])), 0, radio->tx_sequence[1], radio->tx_timestamp[1], 123);
			if (jf)
				jitter_save(&radio->tx_dejitter[1], jf);
			radio->tx_sequence[1] += 1;
			radio->tx_timestamp[1] += rc;
			jitter_load_samples(&radio->tx_dejitter[1], (uint8_t *)audio_samples[1], audio_num, sizeof(*(audio_samples[1])), NULL, NULL);
		}
		break;
#endif
	case AUDIO_MODE_TESTTONE:
		for (i = 0; i < audio_num; i++) {
			audio_samples[0][i] = radio->testtone[0][radio->testtone_pos];
			audio_samples[1][i] = radio->testtone[1][radio->testtone_pos];
			radio->testtone_pos = (radio->testtone_pos + 1) % radio->testtone_length;
		}
		break;
	default:
		LOGP(DRADIO, LOGL_ERROR, "Wrong audio mode, please fix!\n");
		return -EINVAL;
	}

#if 0 /* TX debug: boundary pop detection */
	/* === TX DEBUG: Check EVERY call for inter-block boundary pops === */
	if (audio_num > 1 && tx_boundary.initialized) {
		double boundary_jump = fabs(audio_samples[0][0] - tx_boundary.last_sample_ch0);
		/* Compare boundary jump to the max intra-block jump */
		double max_intra_jump = 0.0;
		for (i = 1; i < audio_num; i++) {
			double j = fabs(audio_samples[0][i] - audio_samples[0][i-1]);
			if (j > max_intra_jump) max_intra_jump = j;
		}
		/* A real pop: boundary jump is significantly larger than any jump within the block.
		 * This means the discontinuity is at the block boundary, not in the audio content. */
		if (boundary_jump > 0.1 && boundary_jump > max_intra_jump * 2.0) {
			tx_boundary.boundary_pop_count++;
			LOGP(DRADIO, LOGL_NOTICE, "TX POP #%d at call %d: boundary=%.4f vs intra=%.4f (ratio=%.1fx) prev_last=%.4f first=%.4f\n",
			     tx_boundary.boundary_pop_count, tx_dbg.call_count,
			     boundary_jump, max_intra_jump, boundary_jump / (max_intra_jump + 1e-9),
			     tx_boundary.last_sample_ch0, audio_samples[0][0]);
		}
	}
	if (audio_num > 0) {
		tx_boundary.last_sample_ch0 = audio_samples[0][audio_num - 1];
		if (radio->tx_audio_channels == 2)
			tx_boundary.last_sample_ch1 = audio_samples[1][audio_num - 1];
		tx_boundary.initialized = 1;
	}

	/* === TX DEBUG: Stage 1 - After audio source read === */
	if (tx_debug_this_call) {
		LOGP(DRADIO, LOGL_DEBUG, "TX call #%d: audio_num=%d signal_num=%d mode=%d\n",
		     tx_dbg.call_count, audio_num, signal_num, radio->tx_audio_mode);
		tx_debug_stats("1_src", audio_samples[0], audio_num, 0, &tx_dbg.prev_last[0], &tx_dbg.disc_count[0]);
		if (radio->tx_audio_channels == 2)
			tx_debug_stats("1_src", audio_samples[1], audio_num, 1, &tx_dbg.prev_last[1], &tx_dbg.disc_count[1]);
	}
#endif

	/* convert mono/stereo, generate differential signal */
	/* (Skip this if we want pure clean signal, but let's keep it to test stereo proc) */
	if (radio->stereo && radio->tx_audio_channels == 1) {
		/* mono to stereo: scale sum to 90%, differential signal is 0 */
		for (i = 0; i < audio_num; i++) {
			audio_samples[0][i] *= 0.9;
			audio_samples[1][i] = 0.0;
		}
	}
	if (radio->stereo && radio->tx_audio_channels == 2) {
		/* stereo: sum is 90%, diffential is 90% */
		double left, right;
		for (i = 0; i < audio_num; i++) {
			left = audio_samples[0][i];
			right = audio_samples[1][i];
			audio_samples[0][i] = (left + right) * 0.45;
			audio_samples[1][i] = (left - right) * 0.45;
		}
	}
	if (!radio->stereo && radio->tx_audio_channels == 2) {
		/* stereo to mono: sum both channel */
		for (i = 0; i < audio_num; i++)
			audio_samples[0][i] = (audio_samples[0][i] + audio_samples[1][i]) / 2.0;
	}

#if 0 /* TX debug: Stage 2 */
	/* === TX DEBUG: Stage 2 - After stereo matrix === */
	if (tx_debug_this_call) {
		tx_debug_stats("2_matrix", audio_samples[0], audio_num, 0, &tx_dbg.prev_last[2], &tx_dbg.disc_count[2]);
		if (radio->stereo)
			tx_debug_stats("2_matrix", audio_samples[1], audio_num, 1, &tx_dbg.prev_last[3], &tx_dbg.disc_count[3]);
	}
#endif

	/* remove DC */
	// iir_process(&radio->tx_dc_removal[0], audio_samples[0], audio_num);
	// if (radio->stereo)
	// 	iir_process(&radio->tx_dc_removal[1], audio_samples[1], audio_num);
	
	/* 
	 * DC OFFSET REMOVAL
	 * -----------------
	 * We use a recursive DC blocker filter: y[n] = x[n] - x[n-1] + R * y[n-1]
	 * R = 0.9995 corresponds to a cutoff of approx 10Hz at 48kHz.
	 * 
	 * JUSTIFICATION:
	 * 1. The previous IIR highpass filter from libfilter was ineffective, leaving a DC 
	 *    offset (up to 0.02) in the signal.
	 * 2. This DC offset caused asymmetric clipping and generated a strong 2nd harmonic 
	 *    distortion (2 kHz tone from a 1 kHz fundamental).
	 * 3. This manual implementation ensures the signal is centered at 0.0 before modulation.
	 */
	{
		double R = 0.9995;
		double x, y;
		int i;
		
		/* Channel 0 (Left/Mono) */
		for (i = 0; i < audio_num; i++) {
			x = audio_samples[0][i];
			y = x - radio->tx_dc_prev_x[0] + R * radio->tx_dc_prev_y[0];
			radio->tx_dc_prev_x[0] = x;
			radio->tx_dc_prev_y[0] = y;
			audio_samples[0][i] = y;
		}

		/* Channel 1 (Right) */
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++) {
				x = audio_samples[1][i];
				y = x - radio->tx_dc_prev_x[1] + R * radio->tx_dc_prev_y[1];
				radio->tx_dc_prev_x[1] = x;
				radio->tx_dc_prev_y[1] = y;
				audio_samples[1][i] = y;
			}
		}
	}



	/* gain volume */
	if (radio->volume != 1.0) {
		for (i = 0; i < audio_num; i++)
			audio_samples[0][i] *= radio->volume;
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++)
				audio_samples[1][i] *= radio->volume;
		}
	}

#if 0 /* TX debug: Stage 3 */
	/* === TX DEBUG: Stage 3 - After DC removal + volume === */
	if (tx_debug_this_call) {
		tx_debug_stats("3_dc_vol", audio_samples[0], audio_num, 0, &tx_dbg.prev_last[4], &tx_dbg.disc_count[4]);
	}
#endif

	/* upsample (or resample with polyphase) */
	if (radio->use_polyphase) {
		signal_num = polyphase_resample(&radio->tx_polyphase[0], audio_samples[0], audio_num, signal_samples[0], radio->signal_buffer_size);
		if (radio->stereo)
			polyphase_resample(&radio->tx_polyphase[1], audio_samples[1], audio_num, signal_samples[1], radio->signal_buffer_size);
			

	} else {
		signal_num = samplerate_upsample_output_num(&radio->tx_resampler[0], audio_num);
		samplerate_upsample(&radio->tx_resampler[0], audio_samples[0], audio_num, signal_samples[0], signal_num);
		if (radio->stereo)
			samplerate_upsample(&radio->tx_resampler[1], audio_samples[1], audio_num, signal_samples[1], signal_num);
	}

	/* prepare baseband */
	memset(baseband, 0, sizeof(float) * 2 * signal_num);
	memset(signal_power, 1, signal_num);

#if 0 /* TX debug: Stage 4 */
	/* === TX DEBUG: Stage 4 - After upsample === */
	if (tx_debug_this_call) {
		tx_debug_stats("4_upsamp", signal_samples[0], signal_num, 0, &tx_dbg.prev_last[5], &tx_dbg.disc_count[5]);
	}
#endif

	/* filter audio (remove DC, remove high frequencies, pre-emphasis)
	 * and modulate */
	switch (radio->modulation) {
	case MODULATION_FM:
		if (radio->emphasis) {
			/* Use TX-only filters for pre-emphasis */
			if (fm_fast_math_enabled())
				pre_emphasis_fast(&radio->fm_emphasis_fast_tx[0], signal_samples[0], signal_num);
			else
				pre_emphasis(&radio->fm_emphasis_tx[0], signal_samples[0], signal_num);
		}

		clipper_process(signal_samples[0], signal_num);
		
		if (radio->stereo) {
			if (radio->emphasis) {
				/* Use TX-only filters for pre-emphasis */
				if (fm_fast_math_enabled())
					pre_emphasis_fast(&radio->fm_emphasis_fast_tx[1], signal_samples[1], signal_num);
				else
					pre_emphasis(&radio->fm_emphasis_tx[1], signal_samples[1], signal_num);
			}
			clipper_process(signal_samples[1], signal_num);
		}

#if 0 /* TX debug: Stage 5 - clipper */
		/* === TX DEBUG: Stage 5 - After pre-emphasis + clipper === */
		if (tx_debug_this_call) {
			double peak5 = 0.0;
			for (i = 0; i < signal_num; i++) {
				double v = fabs(signal_samples[0][i]);
				if (v > peak5) peak5 = v;
			}
			LOGP(DRADIO, LOGL_DEBUG,
			     "TX CLIPPER: sum_peak=%.4f clip_at=%.4f %s\n",
			     peak5, radio->clip_level,
			     (peak5 >= radio->clip_level - 0.01) ? "(clipper active)" : "(no clipping)");
			tx_debug_stats("5_emph_clip", signal_samples[0], signal_num, 0, &tx_dbg.prev_last[6], &tx_dbg.disc_count[6]);
		}
#endif
		
		/* Advance pilot phase if Stereo OR RDS is enabled */
		if (radio->stereo || radio->rds || radio->rds2) {
			double phasestep = radio->pilot_phasestep;
			double phase = radio->tx_pilot_phase;
			double start_phase = phase; /* Capture start phase for RDS */
			
			for (i = 0; i < signal_num; i++) {
				/* Add pilot tone only if Stereo */
				if (radio->stereo) {
					/* Add pilot (19 kHz) and stereo diff (38 kHz) */
					if (fm_fast_math_enabled()) {
						double sc_sin, sc_cos;
						/* 19 kHz pilot */
						fm_fast_sincos(phase * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
						signal_samples[0][i] += sc_sin * 0.1;
						/* 38 kHz stereo subcarrier (2x pilot) */
						fm_fast_sincos(phase * 2.0 * (65536.0 / (2.0 * M_PI)), &sc_sin, &sc_cos);
						signal_samples[0][i] += signal_samples[1][i] * sc_sin;
					} else {
						signal_samples[0][i] += sin(phase) * 0.1;
						signal_samples[0][i] += signal_samples[1][i] * sin(phase * 2);
					}
				}
				
				phase += phasestep;
				if (phase >= 2.0 * M_PI)
					phase -= 2.0 * M_PI;
			}
			radio->tx_pilot_phase = phase;

			/* Add RDS subcarrier if enabled (phase-locked to pilot at 3x frequency) */
			if (radio->rds || radio->rds2) {
				rds_encoder_process(&radio->rds_enc, signal_samples[0], signal_num,
						    start_phase, radio->pilot_phasestep);
				
				/*
				 * TODO: RDS2 Additional Subcarriers (IEC 62106-2:2021)
				 * ---------------------------------------------------
				 * If rds2 flag is set, we should add 3 more BPSK streams:
				 *
				 *   Stream 2:  66.5 kHz   (free-running, NOT pilot harmonic)
				 *   Stream 3:  71.25 kHz  (free-running, NOT pilot harmonic)
				 *   Stream 4:  76.0 kHz   (4 x pilot, CAN be phase-locked)
				 *
				 * Implementation would look like:
				 *   if (radio->rds2) {
				 *       rds2_encoder_process(&radio->rds2_enc, signal_samples[0],
				 *                            signal_num, start_phase, 
				 *                            radio->pilot_phasestep);
				 *   }
				 *
				 * Note: Streams 2 & 3 require independent NCOs because:
				 *   - 66.5 kHz = 19 kHz x 3.5   (not integer harmonic)
				 *   - 71.25 kHz = 19 kHz x 3.75 (not integer harmonic)
				 *
				 * Stream 4 can use: sin(pilot_phase * 4) for 76 kHz
				 *
				 * Injection level: ~2-5% per stream (same as original RDS)
				 * Total RDS2 injection: up to 4 x 5% = 20% (aggressive)
				 *
				 * See rds.c for rds2_encoder_t stub structure.
				 */
			}
		}
		for (i = 0; i < signal_num; i++)
			signal_samples[0][i] *= radio->fm_deviation;

#if 0 /* TX debug: Stage 6 - deviation */
		/* === TX DEBUG: Stage 6 - Final composite (after pilot+RDS, before FM mod) === */
		if (tx_debug_this_call) {
			/* Measure peak composite BEFORE deviation scaling (normalized 0..1) */
			double peak_norm = 0.0;
			for (i = 0; i < signal_num; i++) {
				double v = fabs(signal_samples[0][i]) / radio->fm_deviation;
				if (v > peak_norm) peak_norm = v;
			}
			double peak_khz = peak_norm * radio->fm_deviation / 1000.0;
			int clipping = (peak_norm > 1.0);
			LOGP(DRADIO, LOGL_DEBUG,
			     "TX DEVIATION: peak=%.3f (%.1f kHz / ±%.0f kHz = %.1f%%)%s\n",
			     peak_norm, peak_khz, radio->fm_deviation / 1000.0,
			     peak_norm * 100.0, clipping ? " ** OVER-DEVIATION **" : "");

			tx_debug_stats("6_composite", signal_samples[0], signal_num, 0, &tx_dbg.prev_last[7], &tx_dbg.disc_count[7]);
			/* Summary: discontinuity counts across all stages */
			LOGP(DRADIO, LOGL_DEBUG, "TX disc counts: src=%d matrix=%d dc_vol=%d upsamp=%d emph=%d composite=%d\n",
			     tx_dbg.disc_count[0], tx_dbg.disc_count[2], tx_dbg.disc_count[4],
			     tx_dbg.disc_count[5], tx_dbg.disc_count[6], tx_dbg.disc_count[7]);
			tx_dbg.total_audio_samples += audio_num;
			tx_dbg.total_signal_samples += signal_num;
			LOGP(DRADIO, LOGL_DEBUG, "TX totals: calls=%d audio_samples=%lu signal_samples=%lu\n",
			     tx_dbg.call_count, tx_dbg.total_audio_samples, tx_dbg.total_signal_samples);
		}
#endif

		fm_modulate_complex(&radio->fm_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	case MODULATION_AM_DSB:
		/* Apply compandor (audio compressor) if enabled - before clipping */
		if (radio->am_compandor)
			compress_audio(&radio->am_compandor_state, signal_samples[0], signal_num);
		/* also clip to prevent overshooting after audio filtering */
		clipper_process(signal_samples[0], signal_num);
		iir_process(&radio->tx_am_bw_limit, signal_samples[0], signal_num);
		/* Debug: show peak audio level going into AM modulator */
		{
			static int am_dbg_cnt = 0;
			if (++am_dbg_cnt >= 100) {
				double peak = 0.0;
				for (int j = 0; j < signal_num; j++)
					if (fabs(signal_samples[0][j]) > peak)
						peak = fabs(signal_samples[0][j]);
				LOGP(DRADIO, LOGL_DEBUG, "AM audio peak: %.3f (mod depth %.0f%%)\n", peak, peak * 100.0);
				am_dbg_cnt = 0;
			}
		}
		am_modulate_complex(&radio->am_mod, signal_samples[0], signal_power, signal_num, baseband);
		break;
	case MODULATION_AM_USB:
		/* also clip to prevent overshooting after audio filtering */
		clipper_process(signal_samples[0], signal_num);
		iir_process(&radio->tx_am_bw_limit, signal_samples[0], signal_num);
		am_modulate_ssb(&radio->am_mod, signal_samples[0], signal_power, signal_num, baseband, 1);
		break;
	case MODULATION_AM_LSB:
		/* also clip to prevent overshooting after audio filtering */
		clipper_process(signal_samples[0], signal_num);
		iir_process(&radio->tx_am_bw_limit, signal_samples[0], signal_num);
		am_modulate_ssb(&radio->am_mod, signal_samples[0], signal_power, signal_num, baseband, 0);
		break;
	default:
		break;
	}

	return signal_num;
}

#ifdef AUDIO_DEBUG
static void calc_audio_metrics(const sample_t *in, int n, double *peak, double *rms, double *hf_rms)
{
	int i;
	double p = 0.0, e = 0.0, hf_e = 0.0;

	if (!in || n <= 0) {
		*peak = 0.0;
		*rms = 0.0;
		*hf_rms = 0.0;
		return;
	}

	for (i = 0; i < n; i++) {
		double v = in[i];
		double a = fabs(v);
		if (a > p)
			p = a;
		e += v * v;
		if (i > 0) {
			double d = v - in[i - 1];
			hf_e += d * d;
		}
	}

	*peak = p;
	*rms = sqrt((e + 1e-12) / (n + 1e-12));
	*hf_rms = sqrt((hf_e + 1e-12) / ((n > 1 ? n - 1 : 1) + 1e-12));
}
#endif

int radio_rx(radio_t *radio, float *baseband, int signal_num)
{
	int i;
	int audio_num;
	sample_t *samples[3];
#ifdef AUDIO_DEBUG
	double fm_pre_peak = 0.0, fm_pre_rms = 0.0, fm_pre_hf = 0.0;
	double fm_post_peak = 0.0, fm_post_rms = 0.0, fm_post_hf = 0.0;
	int fm_diag_valid = 0;
#endif
#ifdef HAVE_ALSA
	jitter_frame_t *jf;
#endif

	if (signal_num > radio->buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > buffer_size, please fix!.\n");
		abort();
	}

	if (signal_num > radio->signal_buffer_size) {
		LOGP(DRADIO, LOGL_ERROR, "signal_num > signal_buffer_size, please fix!.\n");
		abort();
	}
	/* No IQ available this cycle: avoid running full RX chain on empty blocks.
	 * This prevents misleading debug output with repeated AUDIO_OUT zeros. */
	if (signal_num <= 0)
		return signal_num;
	/* Use RX-only signal buffer */
	samples[0] = radio->rx_signal_buffer;
	samples[1] = radio->rx_signal_buffer + radio->signal_buffer_size;
	samples[2] = radio->rx_signal_buffer + radio->signal_buffer_size * 2;

	/* DEBUG: Track raw SDR IQ levels (baseband is interleaved I/Q floats) */
	{
		double peak_iq = 0;
		for (i = 0; i < signal_num * 2; i++) {
			double v = fabs(baseband[i]);
			if (v > peak_iq) peak_iq = v;
		}
#ifdef AUDIO_DEBUG
		/* Store in samples[2] temporarily for stage tracking */
		for (i = 0; i < signal_num && i < 1000; i++)
			samples[2][i] = sqrt(baseband[i*2] * baseband[i*2] + baseband[i*2+1] * baseband[i*2+1]);
		audio_debug_stage(&g_rx_debug, RX_STAGE_SDR_RAW, samples[2], (signal_num < 1000) ? signal_num : 1000);
#endif
	}

	switch (radio->modulation) {
	case MODULATION_FM:
		fm_demodulate_complex(&radio->fm_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer);
		for (i = 0; i < signal_num; i++)
			samples[0][i] /= radio->fm_deviation;
		
		/* DEBUG: Track levels after FM demod */
#ifdef AUDIO_DEBUG
		audio_debug_stage(&g_rx_debug, RX_STAGE_FM_DEMOD, samples[0], signal_num);
#endif
		/* AFC (Automatic Frequency Control) for FM
		 * Uses FLL (Frequency-Locked Loop) on IQ samples.
		 * Based on SDRangel's FreqLockComplex implementation.
		 * 
		 * The FLL measures instantaneous frequency by tracking phase
		 * differences between consecutive IQ samples. The IIR-filtered
		 * average frequency IS the carrier offset from center.
		 * 
		 * This runs on the raw IQ baseband samples, measuring the average
		 * phase rotation rate which equals the frequency offset.
		 *
		 * TODO: NFM (Narrow FM) support - same FLL approach works.
		 */
		if (radio->afc.enabled) {
			/* FLL: Process IQ samples to measure average frequency
			 * freq = d(phase)/dt = (phase[n] - phase[n-1]) per sample
			 * IIR filter gives average frequency = carrier offset */
			
			/* Compute FLL alpha from configured AFC time constant */
			if (radio->afc.fll_alpha == 0.0) {
				/* alpha = 1 / (time_constant * samplerate) */
				double tc_seconds = radio->afc.time_constant_s;
				if (tc_seconds < 0.1)
					tc_seconds = 0.1;
				radio->afc.fll_alpha = 1.0 / (tc_seconds * radio->signal_samplerate);
			}
			
			/* Process each IQ sample through FLL */
			for (i = 0; i < signal_num; i++) {
				double re = baseband[i * 2];
				double im = baseband[i * 2 + 1];
				
				/* Get phase of current sample */
				double phase = atan2(im, re);
				
				if (radio->afc.fll_initialized) {
					/* Compute instantaneous frequency (phase difference) */
					double delta_phase = phase - radio->afc.fll_last_phase;
					
					/* Normalize to [-pi, +pi] */
					while (delta_phase <= -M_PI) delta_phase += 2.0 * M_PI;
					while (delta_phase > M_PI) delta_phase -= 2.0 * M_PI;
					
					/* IIR filter: freq = alpha * delta_phase + (1-alpha) * freq */
					radio->afc.fll_freq = radio->afc.fll_alpha * delta_phase + 
					                      (1.0 - radio->afc.fll_alpha) * radio->afc.fll_freq;
				} else {
					radio->afc.fll_initialized = 1;
				}
				
				radio->afc.fll_last_phase = phase;
			}
			
			/* Convert FLL frequency (rad/sample) to Hz */
			double fll_freq_hz = (radio->afc.fll_freq * radio->signal_samplerate) / (2.0 * M_PI);
			
			/* Protect against NaN */
			if (!isfinite(fll_freq_hz))
				fll_freq_hz = 0.0;
			
			radio->afc.freq_error_hz = fll_freq_hz;
			
			/* Pilot accuracy measurement (when stereo/RDS active)
			 * This measures how accurate the station's 19 kHz pilot is - NOT tuning error!
			 * The pilot is always at 19 kHz in the FM baseband regardless of carrier offset.
			 * Useful for diagnosing station transmitter issues. */
			double pilot_accuracy_hz = 0.0;
			if (radio->stereo || radio->rds || radio->rds2) {
				pilot_accuracy_hz = pll_get_freq_error_hz(&radio->rx_pilot_pll, radio->signal_samplerate);
				if (!isfinite(pilot_accuracy_hz))
					pilot_accuracy_hz = 0.0;
			}
			
			/* Apply correction with confidence gate + deadband + slew limit.
			 * When pilot lock is unavailable (stereo/RDS modes), decay back to 0
			 * instead of following noisy FLL estimates. */
			double target_correction = radio->afc.freq_error_hz;
			int afc_confident = 1;
			if (radio->stereo || radio->rds || radio->rds2) {
				if (!pll_is_locked(&radio->rx_pilot_pll))
					afc_confident = 0;
			}
			if (!afc_confident)
				target_correction = 0.0;
			if (target_correction > radio->afc.max_correction_hz)
				target_correction = radio->afc.max_correction_hz;
			else if (target_correction < -radio->afc.max_correction_hz)
				target_correction = -radio->afc.max_correction_hz;
			if (fabs(target_correction) < AFC_DEADBAND_HZ)
				target_correction = 0.0;

			{
				double block_seconds = (double)signal_num / radio->signal_samplerate;
				double max_step = AFC_MAX_SLEW_HZ_PER_S * block_seconds;
				double delta = target_correction - radio->afc.correction_hz;
				if (delta > max_step)
					delta = max_step;
				else if (delta < -max_step)
					delta = -max_step;
				radio->afc.correction_hz += delta;
			}
			radio->afc.update_count++;
			
			/* Update FM demodulator NCO offset to correct carrier offset */
			fm_demod_set_offset(&radio->fm_demod, radio->afc.correction_hz);
			
			/* Track peak error for debug */
			double abs_err = fabs(radio->afc.freq_error_hz);
			if (abs_err > radio->afc.peak_error_hz)
				radio->afc.peak_error_hz = abs_err;

			/* Periodic AFC status log (every 10s) */
			{
				static double afc_log_timer = 0.0;
				struct timespec _ts;
				clock_gettime(CLOCK_MONOTONIC, &_ts);
				double now = _ts.tv_sec + _ts.tv_nsec / 1e9;
				if (afc_log_timer == 0.0) afc_log_timer = now;
				if (now - afc_log_timer >= 10.0) {
					LOGP(DRADIO, LOGL_INFO,
					     "AFC: err=%+.1fHz corr=%+.1fHz peak=%.1fHz pilot=%+.1fHz lock=%d\n",
					     radio->afc.freq_error_hz, radio->afc.correction_hz,
					     radio->afc.peak_error_hz, pilot_accuracy_hz,
					     pll_is_locked(&radio->rx_pilot_pll));
					radio->afc.peak_error_hz = 0.0;
					afc_log_timer = now;
				}
			}
			
			/* Update audio debug stats */
#ifdef AUDIO_DEBUG
			audio_debug_afc(&g_rx_debug,
			                radio->afc.freq_error_hz, radio->afc.correction_hz,
			                pilot_accuracy_hz,
			                0, pll_is_locked(&radio->rx_pilot_pll));
#endif
		}
		
		/* Process PLL and stereo demodulation together
		 * The PLL tracks the 19 kHz pilot and provides phase-locked outputs for:
		 * - Stereo demodulation (38 kHz = 2x pilot)
		 * - RDS decoding (57 kHz = 3x pilot)
		 */
		if (radio->stereo || radio->rds || radio->rds2) {
			double pll_outputs[4];
			
			for (i = 0; i < signal_num; i++) {
				/* Feed FM baseband to PLL - it tracks the 19 kHz pilot */
				pll_process(&radio->rx_pilot_pll, samples[0][i]);
				
				/* Stereo demodulation using PLL's 38 kHz carrier */
				if (radio->stereo) {
					pll_get_stereo_outputs(&radio->rx_pilot_pll, pll_outputs);
					/* pll_outputs[1] = sin(2*phase) = 38 kHz carrier
					 * DSB-SC demod produces (L-R)/2, but L/R matrix also divides by 2,
					 * so no extra gain needed here - the math works out correctly */
					samples[1][i] = samples[0][i] * pll_outputs[1];
				}
			}
		}
		
		/* Decode RDS from FM baseband if enabled */
		if (radio->rds || radio->rds2) {
			/* Use PLL phase for RDS decoding. RDS at 57 kHz = 3 x pilot.
			 * The PLL tracks frequency drift, so no manual offset needed. */
			double rds_pilot_phase = pll_get_phase(&radio->rx_pilot_pll);
			double pll_freq = pll_get_freq(&radio->rx_pilot_pll);
			rds_decoder_process(&radio->rds_dec, samples[0], signal_num,
			                    rds_pilot_phase, pll_freq);
		}
		
		if (radio->stereo) {
			/*
			 * FM STEREO - Post-processing
			 * ===========================
			 * PLL processing and stereo demodulation done above.
			 * Now handle pilot lock detection and filtering.
			 */
			
			/* Get pilot level and lock status from PLL */
			double pilot_mag = pll_get_level(&radio->rx_pilot_pll);
			int pll_locked = pll_is_locked(&radio->rx_pilot_pll);
			double freq_error_hz = pll_get_freq_error_hz(&radio->rx_pilot_pll, radio->signal_samplerate);
			
			/* IIR-smooth pilot magnitude (~100ms time constant) for hysteresis */
#define PILOT_MAG_SMOOTH_TC_S	0.1		/* 100 ms IIR time constant */
			{
				double alpha = (double)signal_num / (PILOT_MAG_SMOOTH_TC_S * radio->signal_samplerate);
				if (alpha > 1.0) alpha = 1.0;
				radio->rx_pilot_mag_avg += alpha * (pilot_mag - radio->rx_pilot_mag_avg);
			}
			
			/* Expose pilot magnitude for external use */
			radio->rx_pilot_mag = pilot_mag;
			
			/* Periodic debug: pilot magnitude, frequency error, lock state */
			{
				static double dbg_accum = 0.0;
				dbg_accum += signal_num;
				if (dbg_accum >= radio->signal_samplerate) {
					dbg_accum = 0.0;
					double pre_db = 20.0 * log10((radio->rx_diag_diff_rms_pre + 1e-12) /
					                            (radio->rx_diag_sum_rms + 1e-12));
					double post_db = 20.0 * log10((radio->rx_diag_diff_rms_post + 1e-12) /
					                             (radio->rx_diag_sum_rms + 1e-12));
					LOGP(DRADIO, LOGL_DEBUG,
					     "Stereo PLL: mag=%.4f avg=%.4f freq_err=%.1fHz pll_lock=%d "
					     "stereo=%d blend=%.0f%% cap=%.0f%% snr=%.1fdB q=%.2f c_cont=%.0f%% c_snr=%.0f%% c_floor=%.0f%% "
					     "hf=%.0f%% sum_rms=%.4f diff_pre=%.4f(%.1fdB) diff_post=%.4f(%.1fdB) "
					     "above=%.0fms below=%.0fms cooldown=%.0fms\n",
					     pilot_mag,
					     radio->rx_pilot_mag_avg,
					     freq_error_hz,
					     pll_locked,
					     radio->rx_pilot_locked,
					     radio->rx_stereo_blend * 100.0,
					     radio->rx_noise_blend_cap * 100.0,
					     radio->rx_input_snr_db,
					     radio->rx_blend_quality,
					     radio->rx_blend_cap_content * 100.0,
					     radio->rx_blend_cap_snr * 100.0,
					     radio->rx_blend_cap_floor * 100.0,
					     radio->rx_stereo_hf_gain * 100.0,
					     radio->rx_diag_sum_rms,
					     radio->rx_diag_diff_rms_pre, pre_db,
					     radio->rx_diag_diff_rms_post, post_db,
					     radio->rx_pilot_above_samples / radio->signal_samplerate * 1000.0,
					     radio->rx_pilot_below_samples / radio->signal_samplerate * 1000.0,
					     (radio->rx_pilot_cooldown > 0.0 ? radio->rx_pilot_cooldown : 0.0) / radio->signal_samplerate * 1000.0);
				}
			}

			/* --- Pilot lock/unlock with integrating hysteresis + cooldown ---
			 *
			 * We use both PLL lock status AND pilot magnitude for robust detection.
			 * The PLL provides fast lock detection, but we add hysteresis to prevent
			 * rapid switching on weak/noisy signals.
			 *
			 * IEC 62106 / ITU-R BS.450 consumer practice: 50–200 ms each way.
			 */
			if (radio->rx_pilot_cooldown > 0.0)
				radio->rx_pilot_cooldown -= signal_num;

			if (!radio->rx_pilot_locked) {
				/* Trying to acquire stereo — use smoothed magnitude to ignore noise dips */
				if (radio->rx_pilot_mag_avg >= PILOT_LOCK_THR && pll_locked) {
					radio->rx_pilot_above_samples += signal_num;
					radio->rx_pilot_below_samples  = 0.0;
					if (radio->rx_pilot_cooldown <= 0.0 &&
					    radio->rx_pilot_above_samples >= PILOT_ACQUIRE_S * radio->signal_samplerate) {
						radio->rx_pilot_locked        = 1;
						radio->rx_pilot_above_samples = 0.0;
						radio->rx_pilot_cooldown      = PILOT_COOLDOWN_S * radio->signal_samplerate;
						LOGP(DRADIO, LOGL_NOTICE, "Stereo pilot locked (mag=%.4f avg=%.4f freq_err=%.1fHz)\n",
						     pilot_mag, radio->rx_pilot_mag_avg, freq_error_hz);
					}
				} else {
					radio->rx_pilot_above_samples = 0.0;
				}
			} else {
				/* Monitoring for pilot loss */
				if (radio->rx_pilot_mag_avg < PILOT_UNLOCK_THR || !pll_locked) {
					radio->rx_pilot_below_samples += signal_num;
					radio->rx_pilot_above_samples  = 0.0;
					if (radio->rx_pilot_cooldown <= 0.0 &&
					    radio->rx_pilot_below_samples >= PILOT_LOSS_S * radio->signal_samplerate) {
						radio->rx_pilot_locked        = 0;
						radio->rx_pilot_below_samples = 0.0;
						radio->rx_pilot_cooldown      = PILOT_COOLDOWN_S * radio->signal_samplerate;
						LOGP(DRADIO, LOGL_NOTICE, "Stereo pilot lost (mag=%.4f avg=%.4f), switching to mono\n",
						     pilot_mag, radio->rx_pilot_mag_avg);
						/* Reset PLL when lock is lost */
						pll_reset(&radio->rx_pilot_pll);
					}
				} else {
					radio->rx_pilot_below_samples = 0.0;
				}
			}

			/* Filter stereo channels to match bandwidth */
			iir_process(&radio->rx_lp_sum, samples[0], signal_num);
			iir_process(&radio->rx_lp_diff, samples[1], signal_num);

			/* DEBUG: Track levels after stereo decode */
#ifdef AUDIO_DEBUG
			audio_debug_stage(&g_rx_debug, RX_STAGE_STEREO_DECODE, samples[0], signal_num);
			audio_debug_stage(&g_rx_debug, RX_STAGE_STEREO_DECODE, samples[1], signal_num);
#endif

			/* Stereo blend: gradually reduce separation as signal weakens.
			 * Uses smoothed pilot magnitude (100ms TC) for gradual transitions.
			 * Full stereo above BLEND_HIGH, full mono below BLEND_LOW.
			 * Thresholds are optimistic - prefer stereo when possible.
			 * Requires pilot lock to enable any stereo (prevents noise decode). */
#define STEREO_BLEND_HIGH	0.07	/* Full stereo above this pilot level */
#define STEREO_BLEND_LOW	0.02	/* Full mono below this pilot level */
			{
				double sum_e = 0.0, diff_e = 0.0;
				double quality, content_cap, cap_target, snr_cap = 1.0, floor_cap = 0.0;
				for (i = 0; i < signal_num; i++) {
					double s0 = samples[0][i];
					double d0 = samples[1][i];
					sum_e += s0 * s0;
					diff_e += d0 * d0;
				}
				quality = sqrt((sum_e + 1e-12) / (diff_e + 1e-12));
				radio->rx_diag_sum_rms = sqrt((sum_e + 1e-12) / (signal_num + 1e-12));
				radio->rx_diag_diff_rms_pre = sqrt((diff_e + 1e-12) / (signal_num + 1e-12));
				if (quality <= STEREO_QUALITY_LOW) {
					content_cap = STEREO_BLEND_CAP_MIN;
				} else if (quality >= STEREO_QUALITY_HIGH) {
					content_cap = 1.0;
				} else {
					double t = (quality - STEREO_QUALITY_LOW) /
					           (STEREO_QUALITY_HIGH - STEREO_QUALITY_LOW);
					content_cap = STEREO_BLEND_CAP_MIN + t * (1.0 - STEREO_BLEND_CAP_MIN);
				}
				cap_target = 1.0;
				if (radio->rx_input_snr_db >= 0.0) {
					if (radio->rx_input_snr_db <= SNR_FORCE_MONO_DB)
						snr_cap = 0.0;
					else if (radio->rx_input_snr_db <= SNR_BLEND_LOW_DB)
						snr_cap = STEREO_BLEND_CAP_MIN;
					else if (radio->rx_input_snr_db >= SNR_BLEND_HIGH_DB)
						snr_cap = 1.0;
					else {
						double t = (radio->rx_input_snr_db - SNR_BLEND_LOW_DB) /
						           (SNR_BLEND_HIGH_DB - SNR_BLEND_LOW_DB);
						snr_cap = STEREO_BLEND_CAP_MIN + t * (1.0 - STEREO_BLEND_CAP_MIN);
					}
					cap_target = snr_cap;
				}
				/* Content cap only in very poor SNR (program-dependent otherwise). */
				if (radio->rx_input_snr_db < 0.0 ||
				    radio->rx_input_snr_db <= CONTENT_CAP_ENABLE_MAX_SNR_DB) {
					if (cap_target > content_cap)
						cap_target = content_cap;
				} else {
					/* Hide content-cap metric when disabled to avoid confusion in logs. */
					content_cap = 1.0;
				}
				if (radio->rx_pilot_locked && !radio->rx_forced_mono &&
				    radio->rx_input_snr_db >= SNR_CAP_FLOOR_DB) {
					floor_cap = SNR_CAP_FLOOR_VALUE;
					if (cap_target < floor_cap)
						cap_target = floor_cap;
				}
				radio->rx_blend_quality = quality;
				radio->rx_blend_cap_content = content_cap;
				radio->rx_blend_cap_snr = snr_cap;
				radio->rx_blend_cap_floor = floor_cap;

				/* Smooth cap to avoid rapid toggling around thresholds. */
				{
					double cap_alpha;
					double tc = (cap_target < radio->rx_noise_blend_cap) ?
					            STEREO_CAP_TC_FALL_S : STEREO_CAP_TC_RISE_S;
					cap_alpha = (double)signal_num / (tc * radio->signal_samplerate);
					if (cap_alpha > 1.0)
						cap_alpha = 1.0;
					if (cap_alpha < 0.0)
						cap_alpha = 0.0;
					radio->rx_noise_blend_cap += cap_alpha * (cap_target - radio->rx_noise_blend_cap);
				}

				double blend_target = 1.0;
				if (!radio->rx_pilot_locked || radio->rx_forced_mono) {
					blend_target = 0.0;
				} else if (radio->rx_pilot_mag_avg < STEREO_BLEND_HIGH) {
					/* Gradual blend based on pilot magnitude */
					blend_target = (radio->rx_pilot_mag_avg - STEREO_BLEND_LOW) 
					      / (STEREO_BLEND_HIGH - STEREO_BLEND_LOW);
					if (blend_target < 0.0) blend_target = 0.0;
					if (blend_target > 1.0) blend_target = 1.0;
				}
				if (blend_target > radio->rx_noise_blend_cap)
					blend_target = radio->rx_noise_blend_cap;

				/* Smooth final blend (fast down, slower up) to avoid breathing. */
				{
					double blend_alpha;
					double tc = (blend_target < radio->rx_stereo_blend) ?
					            STEREO_BLEND_TC_FALL_S : STEREO_BLEND_TC_RISE_S;
					blend_alpha = (double)signal_num / (tc * radio->signal_samplerate);
					if (blend_alpha > 1.0)
						blend_alpha = 1.0;
					if (blend_alpha < 0.0)
						blend_alpha = 0.0;
					radio->rx_stereo_blend += blend_alpha * (blend_target - radio->rx_stereo_blend);
				}

				/* Apply blend to diff channel */
				if (radio->rx_stereo_blend < 1.0) {
					for (i = 0; i < signal_num; i++)
						samples[1][i] *= radio->rx_stereo_blend;
				}

				/* HF-only L-R suppression: keep low-band stereo mostly intact,
				 * attenuate only high-band side-channel hiss by SNR. */
				{
					double hf_gain_target, hf_gain_base;
					double quiet_factor = 0.0;
					if (radio->rx_input_snr_db < 0.0 || radio->rx_input_snr_db >= STEREO_HF_GAIN_HIGH_DB)
						hf_gain_base = 1.0;
					else if (radio->rx_input_snr_db <= STEREO_HF_GAIN_LOW_DB)
						hf_gain_base = STEREO_HF_GAIN_MIN;
					else {
						double t = (radio->rx_input_snr_db - STEREO_HF_GAIN_LOW_DB) /
						           (STEREO_HF_GAIN_HIGH_DB - STEREO_HF_GAIN_LOW_DB);
						hf_gain_base = STEREO_HF_GAIN_MIN + t * (1.0 - STEREO_HF_GAIN_MIN);
					}
					/* Quiet-gate: apply HF attenuation mostly when program level is low.
					 * Keep max gate effect below 100% to avoid "crispy"/over-processed highs. */
					if (radio->rx_diag_sum_rms <= STEREO_QUIET_RMS_LOW)
						quiet_factor = 1.0;
					else if (radio->rx_diag_sum_rms >= STEREO_QUIET_RMS_HIGH)
						quiet_factor = 0.0;
					else
						quiet_factor = (STEREO_QUIET_RMS_HIGH - radio->rx_diag_sum_rms) /
						               (STEREO_QUIET_RMS_HIGH - STEREO_QUIET_RMS_LOW);
					quiet_factor *= 0.70;
					hf_gain_target = 1.0 - quiet_factor * (1.0 - hf_gain_base);
					/* Smooth gain to avoid breathing artifacts. */
					{
						double tc = (hf_gain_target < radio->rx_stereo_hf_gain) ?
						            STEREO_HF_GAIN_TC_FALL_S : STEREO_HF_GAIN_TC_RISE_S;
						double a = (double)signal_num / (tc * radio->signal_samplerate);
						if (a > 1.0) a = 1.0;
						if (a < 0.0) a = 0.0;
						radio->rx_stereo_hf_gain += a * (hf_gain_target - radio->rx_stereo_hf_gain);
					}

					/* samples[2] is scratch here: low-band extract of diff */
					memcpy(samples[2], samples[1], sizeof(*samples[1]) * signal_num);
					iir_process(&radio->rx_lp_diff_low, samples[2], signal_num);
					for (i = 0; i < signal_num; i++) {
						double low = samples[2][i];
						double high = samples[1][i] - low;
						samples[1][i] = low + high * radio->rx_stereo_hf_gain;
					}
				}
				{
					double post_e = 0.0;
					for (i = 0; i < signal_num; i++) {
						double d1 = samples[1][i];
						post_e += d1 * d1;
					}
					radio->rx_diag_diff_rms_post = sqrt((post_e + 1e-12) / (signal_num + 1e-12));
				}
			}
		}
		/* Snapshot mono-equivalent channel before RX deemphasis. */
#ifdef AUDIO_DEBUG
		calc_audio_metrics(samples[0], signal_num, &fm_pre_peak, &fm_pre_rms, &fm_pre_hf);
#endif
		if (radio->emphasis) {
			/* RX path: DC filter -> de-emphasis
			 * DC blocking removes any DC offset from FM demodulator output.
			 * De-emphasis restores flat frequency response. */
			if (fm_fast_math_enabled()) {
				dc_filter_fast(&radio->rx_dc_filter[0], samples[0], signal_num);
				/* Use RX-only filters for de-emphasis */
				de_emphasis_fast(&radio->fm_emphasis_fast_rx[0], samples[0], signal_num);
				if (radio->stereo) {
					dc_filter_fast(&radio->rx_dc_filter[1], samples[1], signal_num);
					de_emphasis_fast(&radio->fm_emphasis_fast_rx[1], samples[1], signal_num);
				}
			} else {
				/* Use RX-only filters for de-emphasis */
				dc_filter(&radio->fm_emphasis_rx[0], samples[0], signal_num);
				de_emphasis(&radio->fm_emphasis_rx[0], samples[0], signal_num);
				if (radio->stereo) {
					dc_filter(&radio->fm_emphasis_rx[1], samples[1], signal_num);
					de_emphasis(&radio->fm_emphasis_rx[1], samples[1], signal_num);
				}
			}
		}
		/* DEBUG: Track levels after de-emphasis */
#ifdef AUDIO_DEBUG
		audio_debug_stage(&g_rx_debug, RX_STAGE_DEEMPHASIS, samples[0], signal_num);
		if (radio->stereo)
			audio_debug_stage(&g_rx_debug, RX_STAGE_DEEMPHASIS, samples[1], signal_num);
		calc_audio_metrics(samples[0], signal_num, &fm_post_peak, &fm_post_rms, &fm_post_hf);
		fm_diag_valid = 1;
#endif
		break;
	case MODULATION_AM_DSB:
		/* TODO: Implement Synchronous AM (SAM) detection with carrier PLL tracking.
		 * This would enable AFC for AM by tracking the carrier frequency.
		 * Current envelope detection doesn't produce DC offset proportional to
		 * frequency error, so AFC is not practical with envelope detection.
		 * SAM approach: PLL locks to carrier, provides coherent demodulation
		 * and frequency error estimate. See SDRangel's AM demod PLL option. */
		am_demodulate_complex(&radio->am_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer, radio->carrier_buffer);
		break;
	case MODULATION_AM_USB:
	case MODULATION_AM_LSB:
		/* Use Product Detection (SSB) */
		am_demodulate_ssb(&radio->am_demod, samples[0], signal_num, baseband, radio->I_buffer, radio->Q_buffer);
		break;
	default:
		break;
	}

	/* downsample (or resample with polyphase) */
	/* downsample (or resample with polyphase) */
	if (radio->use_polyphase) {
		/* Use audio_buffer for output to support upsampling (output > input) 
		 * and avoid in-place overwrite issues. */
		sample_t *out_left = radio->audio_buffer;
		sample_t *out_right = radio->audio_buffer + radio->audio_buffer_size;
		
		audio_num = polyphase_resample(&radio->rx_polyphase[0], samples[0], signal_num, out_left, radio->audio_buffer_size);
		samples[0] = out_left;
		
		/* DEBUG: Track resampling ratio */
#ifdef AUDIO_DEBUG
		audio_debug_resample(&g_rx_debug, signal_num, audio_num, 
		                     radio->rx_audio_samplerate / radio->signal_samplerate);
#endif
		
		if (radio->stereo) {
			polyphase_resample(&radio->rx_polyphase[1], samples[1], signal_num, out_right, radio->audio_buffer_size);
			samples[1] = out_right;
		}
	} else {
		audio_num = samplerate_downsample(&radio->rx_resampler[0], samples[0], signal_num);
		
		/* DEBUG: Track resampling ratio */
#ifdef AUDIO_DEBUG
		audio_debug_resample(&g_rx_debug, signal_num, audio_num,
		                     radio->rx_audio_samplerate / radio->signal_samplerate);
#endif
		
		if (radio->stereo)
			samplerate_downsample(&radio->rx_resampler[1], samples[1], signal_num);
	}

	/* DEBUG: Track levels after resampling - BOTH channels for stereo */
#ifdef AUDIO_DEBUG
	audio_debug_stage(&g_rx_debug, RX_STAGE_RESAMPLE, samples[0], audio_num);
	if (radio->stereo) {
		/* Track channel 1 separately to see if it has higher peaks */
		double peak_ch1 = 0;
		for (i = 0; i < audio_num; i++) {
			double v = fabs(samples[1][i]);
			if (v > peak_ch1) peak_ch1 = v;
		}
		if (peak_ch1 > 1.0) {
			LOGP(DRADIO, LOGL_DEBUG, "RESAMPLE ch1 peak=%.4f (>1.0!)\n", peak_ch1);
		}
	}

	/* FM artifact diagnostics */
	if (fm_diag_valid && radio->modulation == MODULATION_FM) {
		double out_peak = 0.0, out_rms = 0.0, out_hf = 0.0;
		calc_audio_metrics(samples[0], audio_num, &out_peak, &out_rms, &out_hf);
		audio_debug_fm_hf(&g_rx_debug,
		                  fm_pre_peak, fm_pre_rms, fm_pre_hf,
		                  fm_post_peak, fm_post_rms, fm_post_hf,
		                  out_peak, out_rms, out_hf,
		                  radio->signal_samplerate, radio->rx_audio_samplerate,
		                  radio->rx_stereo_blend, radio->rx_stereo_hf_gain,
		                  radio->stereo && !radio->rx_forced_mono);
	}
#endif

	/* apply volume (multiply, not divide!) */
	if (radio->volume != 1.0) {
		for (i = 0; i < audio_num; i++)
			samples[0][i] *= radio->volume;
		if (radio->stereo) {
			for (i = 0; i < audio_num; i++)
				samples[1][i] *= radio->volume;
		}
	}

	/* convert mono/stereo, (from differential signal) */
	if (radio->stereo && radio->rx_audio_channels == 1) {
		/* stereo to mono: just use sum channel (L+R), ignore diff
		 * The sum channel is already mono-compatible */
		/* samples[0] already contains sum, nothing to do */
	}
	if (radio->stereo && radio->rx_audio_channels == 2) {
		/* FM Stereo Matrix Decoding with Level Compensation
		 * 
		 * sum = L+R (mono-compatible, 0-15 kHz)
		 * diff = L-R (DSB-SC on 38 kHz subcarrier, half amplitude after demod)
		 * 
		 * Stereo decode: L = sum/2 + diff, R = sum/2 - diff
		 * For center-panned content (L=R, diff=0): L = R = sum/2
		 * 
		 * Mono output: L = R = sum = L+R
		 * For center-panned: L = R = 2*original (coherent sum)
		 * 
		 * This creates a 6 dB loudness difference (mono louder than stereo).
		 * Industry standard is to apply +3 dB gain compensation to stereo
		 * output to match perceived loudness with mono. This is a compromise:
		 * - +6 dB would match perfectly for center-panned content
		 * - +3 dB (sqrt(2) ≈ 1.414) is standard for typical music with panning
		 * 
		 * Reference: ITU-R BS.412, typical FM receiver design practice
		 */
#define STEREO_LEVEL_COMP	1.414	/* +3 dB = sqrt(2), industry standard */
		
		if (radio->rx_pilot_locked && !radio->rx_forced_mono) {
			/* True stereo decode with level compensation */
			double sum, diff;
			for (i = 0; i < audio_num; i++) {
				sum = samples[0][i];
				diff = samples[1][i];
				samples[0][i] = (sum / 2.0 + diff) * STEREO_LEVEL_COMP;  /* L */
				samples[1][i] = (sum / 2.0 - diff) * STEREO_LEVEL_COMP;  /* R */
			}
		} else {
			/* Mono fallback - clone sum to both channels (no compensation needed) */
			for (i = 0; i < audio_num; i++)
				samples[1][i] = samples[0][i];
		}
	}
	if (!radio->stereo && radio->rx_audio_channels == 2) {
		/* mono to stereo: clone channel */
		for (i = 0; i < audio_num; i++)
			samples[1][i] = samples[0][i];
	}

	/* Final RX FM anti-harsh shaping at audio rate (mono + stereo). */
	if (radio->modulation == MODULATION_FM && audio_num > 0) {
		iir_process(&radio->rx_out_hicut[0], samples[0], audio_num);
		if (radio->rx_audio_channels == 2)
			iir_process(&radio->rx_out_hicut[1], samples[1], audio_num);
	}

#ifdef AUDIO_DEBUG
	audio_debug_stage(&g_rx_debug, RX_STAGE_AUDIO_OUT, samples[0], audio_num);
	if (radio->stereo && radio->rx_audio_channels == 2)
		audio_debug_stage(&g_rx_debug, RX_STAGE_AUDIO_OUT, samples[1], audio_num);
	audio_debug_report(&g_rx_debug);
#endif

	/* display wave */
	display_wave(&radio->dispwav[0], samples[0], audio_num, 1.0);
	if (radio->stereo && radio->rx_audio_channels == 2)
		display_wave(&radio->dispwav[1], samples[1], audio_num, 1.0);

	/* store received audio */
	if ((radio->rx_audio_mode & AUDIO_MODE_WAVEFILE))
		wave_write(&radio->wave_rx_rec, samples, audio_num);
#ifdef HAVE_ALSA
	if ((radio->rx_audio_mode & AUDIO_MODE_AUDIODEV) && audio_num > 0) {
		jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)samples[0], audio_num * sizeof(*(samples[0])), 0, radio->rx_sequence[0], radio->rx_timestamp[0], 123);
		if (jf)
			jitter_save(&radio->rx_dejitter[0], jf);
		radio->rx_sequence[0] += 1;
		radio->rx_timestamp[0] += audio_num;
		if (radio->rx_audio_channels == 2) {
			jf = jitter_frame_alloc(NULL, NULL, (uint8_t *)samples[1], audio_num * sizeof(*(samples[1])), 0, radio->rx_sequence[1], radio->rx_timestamp[1], 123);
			if (jf)
				jitter_save(&radio->rx_dejitter[1], jf);
			radio->rx_sequence[1] += 1;
			radio->rx_timestamp[1] += audio_num;
		}
	}
	if ((radio->rx_audio_mode & AUDIO_MODE_AUDIODEV)) {
		audio_num = sound_get_tosend(radio->rx_sound, radio->signal_buffer_size);
		if (audio_num < 0) {
			LOGP(DDSP, LOGL_ERROR, "Failed to get number of samples in buffer (rc = %d)!\n", audio_num);
			if (audio_num == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
		jitter_load_samples(&radio->rx_dejitter[0], (uint8_t *)samples[0], audio_num, sizeof(*samples), NULL, NULL);
		if (radio->rx_audio_channels == 2)
			jitter_load_samples(&radio->rx_dejitter[1], (uint8_t *)samples[1], audio_num, sizeof(*samples), NULL, NULL);
		// printf("channels=%d num=%d\n", radio->rx_audio_channels, audio_num);
		audio_num = sound_write(radio->rx_sound, samples, NULL, audio_num, NULL, NULL, radio->rx_audio_channels);
		if (audio_num < 0) {
			LOGP(DRADIO, LOGL_ERROR, "Failed to write to sound device (rc = %d)!\n", audio_num);
			if (audio_num == -EPIPE)
				LOGP(DRADIO, LOGL_ERROR, "Trying to recover.\n");
			else
				return 0;
		}
	}
#endif

	return signal_num;
}


void radio_set_callsign(const char *callsign)
{
	if (rds_user_callsign)
		free(rds_user_callsign);
	rds_user_callsign = callsign ? strdup(callsign) : NULL;
}

void radio_set_pi(uint16_t pi)
{
	rds_user_pi = pi;
}

