
int dsp_init_sender(flex_t *flex, int samplerate, double deviation, double polarity, int enable_lpf);
void dsp_set_speed(flex_t *flex, int baud_rate, int modulation_type);
void dsp_set_polarity(flex_t *flex, double polarity);
void dsp_cleanup_sender(flex_t *flex);
