
/* Global RX Kanji/Shift-JIS decode flag (defined in main.c, set by --rx-kanji) */
extern int rx_kanji_enabled;

int dsp_init_sender(flex_t *flex, int samplerate, double deviation, double polarity, int enable_lpf);
void dsp_set_speed(flex_t *flex, int bitrate, int modulation_type);
void dsp_set_polarity(flex_t *flex, double polarity);
void dsp_cleanup_sender(flex_t *flex);
