
int dsp_init_sender(pocsag_t *pocsag, int samplerate, int baudrate, double deviation, double polarity, int auto_baud, int auto_polarity);
void dsp_init_ramp(pocsag_t *pocsag);
void dsp_cleanup_sender(pocsag_t *pocsag);

