
#include "../libmobile/sender.h"
#include "../libfm/fm.h"
#include "../libam/am.h"
#include "rds.h"
#include "sca.h"

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

typedef struct radio {
	/* modes */
	int		buffer_size;		/* maximum number of samples */
	enum modulation	modulation;		/* modulation type */
	enum audio_mode	tx_audio_mode;		/* mode for audio source */
	enum audio_mode	rx_audio_mode;		/* mode for audio sink */
	double		volume;			/* volume change (gain/dampen) */
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
	double		signal_bandwidth;
	samplerate_t	tx_resampler[2];	/* resampling from audio rate to signal rate (two channels) */
	samplerate_t	rx_resampler[2];	/* resampling from signal rate to audi rate (two channels) */
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
	double		rx_pilot_phase;		/* current phase of rx mixer */
	iir_filter_t	tx_dc_removal[2];	/* AM/FM DC level removal */
	sample_t	tx_dc_prev_x[2], tx_dc_prev_y[2]; /* Manual DC filter state */
	iir_filter_t	tx_am_bw_limit;		/* AM bandwidth limiter */
	iir_filter_t	rx_lp_pilot_I;		/* low pass filter for pilot tone extraction */
	iir_filter_t	rx_lp_pilot_Q;		/* low pass filter for pilot tone extraction */
	iir_filter_t	rx_lp_sum;		/* filter sum signal of stereo */
	iir_filter_t	rx_lp_diff;		/* filter differential signal of stereo */
	double		rx_pll_freq_offset;	/* tracked phase offset (rad) for stereo demod */
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
} radio_t;

int radio_init(radio_t *radio, int buffer_size, int samplerate, double frequency, const char *tx_wave_file, const char *rx_wave_file, const char *tx_audiodev, const char *rx_audiodev, enum modulation modulation, double bandwidth, double deviation, double modulation_index, double time_constant, double volume, int stereo, int rds, int rds2, int sca_67k, int sca_92k, int rds_debug, int rds_verbose);
void radio_exit(radio_t *radio);
int radio_start(radio_t *radio);
int radio_tx(radio_t *radio, float *baseband, int num);
int radio_rx(radio_t *radio, float *baseband, int num);

/* RDS preset switching (press 'f' to cycle) */
void rds_next_preset(radio_t *radio);


void radio_set_callsign(const char *callsign);
void radio_set_pi(uint16_t pi);
