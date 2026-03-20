#ifndef DSP_H
#define DSP_H

int  dsp_init_sender(mobitex_t *mobitex, int samplerate, int baudrate,
                     double deviation, double polarity);
void dsp_cleanup_sender(mobitex_t *mobitex);
int  fsk_encode_bits(mobitex_t *mobitex, const uint8_t *bits, int nbits);

#endif /* DSP_H */
