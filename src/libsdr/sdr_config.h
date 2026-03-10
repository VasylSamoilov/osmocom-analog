#ifndef SDR_CONFIG_H
#define SDR_CONFIG_H

typedef struct sdr_config {
	int		uhd,			/* select UHD API */
			soapy;			/* select Soapy SDR API */
#ifdef HAVE_RPITX
	int		rpitx;			/* select rpitx API (Raspberry Pi GPIO TX) */
#endif
	int		channel;		/* channel number */
	const char	*device_args,		/* arguments */
			*stream_args,
			*tune_args;
	int		samplerate;		/* ADC/DAC sample rate */
	double		lo_offset;		/* LO frequency offset */
	double		bandwidth;		/* IF bandwidth */
	double		tx_gain,		/* gain */
			rx_gain;
	const char	*tx_antenna,		/* list/override antennas */
			*rx_antenna;
	const char	*clock_source;		/* list/override clock source */
	const char	*write_iq_tx_wave;	/* wave recording and playback */
	const char	*write_iq_rx_wave;
	const char	*read_iq_tx_wave;
	const char	*read_iq_rx_wave;
	int		swap_links;		/* swap DL and UL frequency */
	int		timestamps;		/* use time stamps when transmitting */
	/* Split device mode (mutually exclusive with unified mode above) */
	int		split_mode;		/* flag: using separate TX/RX devices */
	const char	*tx_device_args;	/* TX device arguments */
	int		tx_samplerate;		/* TX sample rate */
	double		tx_bandwidth;		/* TX IF bandwidth */
	double		tx_lo_offset;		/* TX LO offset */
	const char	*rx_device_args;	/* RX device arguments */
	int		rx_samplerate;		/* RX sample rate */
	double		rx_bandwidth;		/* RX IF bandwidth */
	double		rx_lo_offset;		/* RX LO offset */
	/* Frequency correction (crystal oscillator PPM error) */
	double		ppm;			/* unified PPM correction (both TX and RX) */
	double		tx_ppm;			/* split mode: TX device PPM correction */
	double		rx_ppm;			/* split mode: RX device PPM correction */
	/* Upconverter support (e.g., Ham-It-Up for kHz band reception) */
	double		tx_upconverter;		/* TX upconverter offset (Hz) */
	double		rx_upconverter;		/* RX upconverter offset (Hz) */
	/* TX-only / RX-only mode flags */
	int		tx_only;		/* --sdr-tx-only flag */
	int		rx_only;		/* --sdr-rx-only flag */
	/* Per-device split overrides (split mode only) */
	int		tx_channel;		/* --sdr-tx-channel */
	int		rx_channel;		/* --sdr-rx-channel */
	int		tx_channel_given;	/* flag: --sdr-tx-channel was specified */
	int		rx_channel_given;	/* flag: --sdr-rx-channel was specified */
	const char	*tx_stream_args;	/* --sdr-tx-stream-args */
	const char	*rx_stream_args;	/* --sdr-rx-stream-args */
	const char	*tx_tune_args;		/* --sdr-tx-tune-args */
	const char	*rx_tune_args;		/* --sdr-rx-tune-args */
	const char	*tx_clock_source;	/* --sdr-tx-clock-source */
	const char	*rx_clock_source;	/* --sdr-rx-clock-source */
	/* Per-device driver selection (split mode only) */
	int		tx_uhd;			/* --sdr-tx-uhd */
	int		tx_soapy;		/* --sdr-tx-soapy */
#ifdef HAVE_RPITX
	int		tx_rpitx;		/* --sdr-tx-rpitx */
#endif
	int		rx_uhd;			/* --sdr-rx-uhd */
	int		rx_soapy;		/* --sdr-rx-soapy */
	/* Bandwidth hint for auto rate selection */
	double		bandwidth_hint;		/* Total required bandwidth (Hz) */
	int		bandwidth_hint_set;	/* Flag: bandwidth hint was provided */
	int		samplerate_given;	/* Flag: user explicitly set -s */
	int		sdr_samplerate_given;	/* Flag: user explicitly set --sdr-samplerate */
	int		auto_selection_done;	/* Flag: auto rate selection already completed */
	/* Auto-selected IF bandwidth from device query (0 = use samplerate) */
	double		tx_auto_bandwidth;	/* TX device auto-selected IF bandwidth */
	double		rx_auto_bandwidth;	/* RX device auto-selected IF bandwidth */
} sdr_config_t;

/* Sample rate information from device query */
typedef struct sdr_rate_info {
	double		*rates;			/* Array of discrete rates (NULL if continuous) */
	int		num_rates;		/* Number of discrete rates */
	double		min_rate;		/* Minimum rate (for continuous range) */
	double		max_rate;		/* Maximum rate (for continuous range) */
	int		is_continuous;		/* 1 if continuous range, 0 if discrete */
} sdr_rate_info_t;

typedef enum {
	SDR_MODE_SINGLE,	/* Single device, full duplex (default) */
	SDR_MODE_SPLIT,		/* Separate TX and RX devices */
	SDR_MODE_TX_ONLY,	/* TX device only */
	SDR_MODE_RX_ONLY	/* RX device only */
} sdr_mode_t;

extern sdr_config_t *sdr_config;

void sdr_config_init(double lo_offset);
void sdr_config_print_help(void);
void sdr_config_print_hotkeys(void);
void sdr_config_add_options(void);
int sdr_config_handle_options(int short_option, int argi, char **argv);
int sdr_configure(int samplerate);
sdr_mode_t sdr_get_mode(void);
int sdr_check_separate_device_support(int tx_only, int rx_only, int split_mode);

/* Bandwidth hint for auto rate selection */
void sdr_config_set_bandwidth(double max_deviation, double max_modulation, int num_channels, double channel_spacing);

/* Early rate selection - call BEFORE creating senders
 * Sets bandwidth hint and selects optimal SDR rate.
 * Updates dsp_samplerate if auto-selection finds a better rate.
 * required_bandwidth: Pre-calculated required bandwidth in Hz
 * Returns: 0 if no SDR, 1 if SDR configured, <0 on error
 */
int sdr_select_rate(double required_bandwidth, int *dsp_samplerate);

/* Rate info helpers */
void sdr_rate_info_free(sdr_rate_info_t *info);
int sdr_select_optimal_rate(double min_bandwidth, double min_if_bw, int dsp_rate, const sdr_rate_info_t *info, int *out_rate);

#endif /* SDR_CONFIG_H */
