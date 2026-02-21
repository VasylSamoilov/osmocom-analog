
#include "../libmobile/sender.h"
#include "../libfm/fm.h"
#include "../libam/am.h"
#include "../libpolyphase/polyphase.h"
#include "../libcompandor/compandor.h"
#include "../libpll/pll.h"
#include "rds.h"
#include "sca.h"
#include "audio_debug.h"

enum modulation {
	MODULATION_NONE = 0,
	MODULATION_FM,
	MODULATION_AM_DSB,
	MODULATION_AM_USB,
	MODULATION_AM_LSB,
};

enum audio_mode {
	AUDIO_MODE_NONE = 0,
	AUDIO_MODE_WAVEFILE = 1,
	AUDIO_MODE_AUDIODEV = 2,
	AUDIO_MODE_TESTTONE = 4,
};

/* AFC (Automatic Frequency Control) state for FM
 * 
 * Uses FLL (Frequency-Locked Loop) on IQ samples BEFORE FM demodulation.
 * The FLL measures instantaneous frequency by tracking phase differences
 * between consecutive IQ samples, then IIR-filters to get average frequency.
 * This average frequency IS the carrier offset from center.
 * 
 * For broadcast FM with ±75 kHz deviation, a long time constant (5 seconds)
 * is needed to average out the audio modulation and get the true carrier offset.
 * 
 * The 19 kHz stereo pilot PLL cannot be used for AFC because the pilot is
 * always at 19 kHz in the FM baseband regardless of carrier offset - it only
 * measures the station's pilot accuracy, not tuning error.
 */
typedef struct afc_state {
	int enabled;              /* AFC on/off */
	/* FLL state (operates on IQ samples before FM demod) */
	double fll_last_phase;    /* Previous sample phase for frequency calculation */
	double fll_freq;          /* IIR-filtered instantaneous frequency (rad/sample) */
	double fll_alpha;         /* IIR filter coefficient (computed from time constant) */
	int fll_initialized;      /* 1 after first sample processed */
	/* Computed values */
	double freq_error_hz;     /* Current carrier offset from FLL (Hz) */
	double correction_hz;     /* Applied NCO correction (Hz) */
	double time_constant_s;   /* IIR time constant (default 5s for broadcast FM) */
	double max_correction_hz; /* Limit (default 5000 Hz) */
	/* Debug/statistics */
	double peak_error_hz;     /* Peak frequency error seen */
	uint64_t update_count;    /* Number of AFC updates */
} afc_state_t;

typedef struct radio {
	/* modes */
	int		buffer_size;		/* maximum number of samples */
	enum modulation	modulation;		/* modulation type */
	enum audio_mode	tx_audio_mode;		/* mode for audio source */
	enum audio_mode	rx_audio_mode;		/* mode for audio sink */
	double		volume;			/* volume change (gain/dampen) */
	double		clip_level;		/* soft clipper threshold (audio budget after pilot/RDS) */
	int		stereo;			/* use stere FM */
	int		rds, rds2;		/* use RDS */
	int		emphasis;		/* use pre-/de-emphasis with FM */
	/* audio stage */
	double		tx_audio_samplerate;	/* sample rate of audio source */
	double		rx_audio_samplerate;	/* sample rate of audio sink */
	int		tx_audio_channels;	/* number of channels of audio source */
	int		rx_audio_channels;	/* number of channels of audio sink */
	double		audio_bandwidth;	/* audio bandwidth */
	const char 	*tx_wave_file;		/* wave file name of source */
	const char 	*rx_wave_file;		/* wave file name of sink */
	wave_play_t	wave_tx_play;		/* wave playback process */
	wave_rec_t	wave_rx_rec;		/* wave record process */
	void		*tx_sound;		/* sound card process */
	void		*rx_sound;		/* sound card process */
	jitter_t	tx_dejitter[2];		/* jitter buffer when reading from sound card */
	uint16_t	tx_sequence[2];		/* sequence & ts for jitter buffer */
	uint32_t	tx_timestamp[2];
	jitter_t	rx_dejitter[2];		/* jitter buffer when writing to sound card */
	uint16_t	rx_sequence[2];		/* sequence & ts for jitter buffer */
	uint32_t	rx_timestamp[2];
	sample_t	*testtone[2];		/* test tone sample */
	int		testtone_length;
	int		testtone_pos;
	dispwav_t	dispwav[2];		/* display wave form */
	/* signal stage */
	double		signal_samplerate;
	double		baseband_extent;	/* max baseband freq (one-sided), e.g. 135 kHz for FM+RDS */
	double		rf_bandwidth;		/* full RF bandwidth = 2 * baseband_extent */
	double		required_samplerate;	/* min sample rate = rf_bandwidth / 0.75 (filter margin) */
	samplerate_t	tx_resampler[2];	/* resampling from audio rate to signal rate (two channels) */
	samplerate_t	rx_resampler[2];	/* resampling from signal rate to audi rate (two channels) */
	polyphase_t	tx_polyphase[2];	/* polyphase resampler TX (optional) */
	polyphase_t	rx_polyphase[2];	/* polyphase resampler RX (optional) */
	int		use_polyphase;		/* use polyphase instead of linear resampler */
	emphasis_fast_t	fm_emphasis_fast_tx[2];	/* FM pre emphasis TX (optimized 1st-order) */
	emphasis_fast_t	fm_emphasis_fast_rx[2];	/* FM de emphasis RX (optimized 1st-order) */
	emphasis_t	fm_emphasis_tx[2];		/* FM pre emphasis TX */
	emphasis_t	fm_emphasis_rx[2];		/* FM de emphasis RX */
	dc_filter_fast_t rx_dc_filter[2];	/* RX DC blocking filter */
	double		fm_deviation;		/* deviation of fm signal */
	fm_mod_t	fm_mod;			/* FM modulation */
	fm_demod_t	fm_demod;		/* FM modulation */
	double		pilot_phasestep;	/* phase change of pilot tone for each sample */
	double		tx_pilot_phase;		/* current phase of tx sine */
	double		rx_pilot_phase;		/* current phase of rx mixer (legacy, kept for TX) */
	pll_t		rx_pilot_pll;		/* PLL for 19 kHz pilot tracking */
	iir_filter_t	tx_dc_removal[2];	/* AM/FM DC level removal */
	sample_t	tx_dc_prev_x[2], tx_dc_prev_y[2]; /* Manual DC filter state */
	iir_filter_t	tx_am_bw_limit;		/* AM bandwidth limiter */
	iir_filter_t	rx_lp_pilot_I;		/* low pass filter for pilot tone extraction (legacy) */
	iir_filter_t	rx_lp_pilot_Q;		/* low pass filter for pilot tone extraction (legacy) */
	iir_filter_t	rx_lp_sum;		/* filter sum signal of stereo */
	iir_filter_t	rx_lp_diff;		/* filter differential signal of stereo */
	iir_filter_t	rx_lp_diff_low;		/* low-band extract for L-R HF suppression */
	iir_filter_t	rx_out_hicut[2];	/* gentle post-deemphasis anti-harsh filter */
	double		rx_pll_freq_offset;	/* tracked phase offset (rad) for stereo demod (legacy) */
	double		rx_pilot_mag;		/* pilot tone magnitude (0=no signal, ~0.1 on lock) */
	double		rx_pilot_mag_avg;	/* IIR-smoothed pilot magnitude for threshold decisions */
	double		rx_stereo_blend;	/* stereo blend factor: 1.0=full stereo, 0.0=mono */
	double		rx_noise_blend_cap;	/* noise-aware max stereo blend (quieting aid) */
	double		rx_input_snr_db;	/* external RF SNR estimate from signal meter */
	double		rx_blend_quality;	/* debug: sqrt(sum_energy/diff_energy) */
	double		rx_blend_cap_content;	/* debug: cap from content metric */
	double		rx_blend_cap_snr;	/* debug: cap from SNR metric */
	double		rx_blend_cap_floor;	/* debug: enforced floor at high SNR */
	double		rx_stereo_hf_gain;	/* debug: applied gain to high-band L-R component */
	double		rx_diag_sum_rms;	/* debug: L+R RMS before matrix */
	double		rx_diag_diff_rms_pre;	/* debug: L-R RMS before blend */
	double		rx_diag_diff_rms_post;	/* debug: L-R RMS after blend */
	int		rx_pilot_locked;	/* 1 = pilot locked, 0 = mono fallback */
	int		rx_forced_mono;		/* 1 = force mono (B1 command), 0 = auto */
	double		rx_pilot_cooldown;	/* samples until next lock state change allowed */
	double		rx_pilot_above_samples;	/* samples continuously above LOCK_THR   */
	double		rx_pilot_below_samples;	/* samples continuously below UNLOCK_THR */
	/* RDS encoder */
	rds_encoder_t	rds_enc;		/* RDS encoder state */
	rds_decoder_t	rds_dec;		/* RDS decoder state */
	/* SCA encoder/decoder */
	sca_encoder_t	sca_enc;		/* SCA encoder state */
	sca_decoder_t	sca_dec;		/* SCA decoder state */
	int		sca_67k;		/* 67 kHz SCA enabled */
	int		sca_92k;		/* 92 kHz SCA enabled */
	am_mod_t	am_mod;			/* AM modulation */
	am_demod_t	am_demod;		/* AM modulation */
	/* AM compandor (audio compressor for better modulation depth) */
	int		am_compandor;		/* enable AM compandor */
	compandor_t	am_compandor_state;	/* compandor state for AM */
	/* buffers */
	sample_t	*audio_buffer;
	int		audio_buffer_size;
	sample_t	*tx_signal_buffer;		/* TX-only signal buffer */
	sample_t	*rx_signal_buffer;		/* RX-only signal buffer */
	uint8_t		*signal_power_buffer;		/* TX-only power buffer */
	int		signal_buffer_size;
	sample_t	*I_buffer;			/* RX-only I/Q buffers for demodulation */
	sample_t	*Q_buffer;
	sample_t	*carrier_buffer;		/* RX-only carrier buffer for AM */
	/* AFC state for mono FM */
	afc_state_t	afc;
} radio_t;

int radio_init(radio_t *radio, int buffer_size, int samplerate, double frequency, const char *tx_wave_file, const char *rx_wave_file, const char *tx_audiodev, const char *rx_audiodev, enum modulation modulation, double bandwidth, double deviation, double modulation_index, double time_constant, double volume, int stereo, int rds, int rds2, int sca_67k, int sca_92k, int rds_debug, int rds_verbose, int am_compandor, int rds_force_rbds);
void radio_exit(radio_t *radio);
int radio_start(radio_t *radio);
int radio_tx(radio_t *radio, float *baseband, int num);
int radio_rx(radio_t *radio, float *baseband, int num);

/* RDS preset switching (press 'f' to cycle) */
void rds_next_preset(radio_t *radio);

/* Force mono mode (B1 command from XDR-GTK) */
void radio_set_forced_mono(radio_t *radio, int forced);
void radio_set_rx_snr(radio_t *radio, double snr_db);

/* AFC control */
void radio_afc_enable(radio_t *radio, int enable);
void radio_afc_set_time_constant(radio_t *radio, double tc_seconds);
void radio_afc_set_max_correction(radio_t *radio, double max_hz);
double radio_afc_get_correction(radio_t *radio);
double radio_afc_get_freq_error(radio_t *radio);

void radio_set_callsign(const char *callsign);
void radio_set_pi(uint16_t pi);
void radio_set_polyphase(int enable);
