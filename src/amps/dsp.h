
void dsp_init(void);
int dsp_init_sender(amps_t *amps, int tolerant);
void dsp_cleanup_sender(amps_t *amps);
void amps_set_dsp_mode(amps_t *amps, enum dsp_mode mode, int frame_length);

/* Get bandwidth parameters for SDR rate selection (works for AMPS/TACS/JTACS) */
void amps_get_bandwidth(int tacs, double *max_deviation, double *max_modulation);
