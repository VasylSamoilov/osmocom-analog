#ifndef FRAME_H
#define FRAME_H

/* ---- Bit Scrambler ---- */
void     mobitex_scrambler_reset(uint16_t *sr);
int      mobitex_scrambler_bit(uint16_t *sr, int scrambling);

/* ---- FEC (12,8) Hamming ---- */
int      mobitex_fec_encode(int data_byte);
int      mobitex_fec_decode(int *codeword);

/* ---- CRC-16 ---- */
void     mobitex_crc_reset(uint16_t *sr, uint16_t *cr);
void     mobitex_crc_bit(uint16_t *sr, uint16_t *cr, int bit);
uint16_t mobitex_crc_finalize(uint16_t cr);

/* ---- Interleaving ---- */
void     mobitex_interleave(const uint16_t codewords[20], uint8_t bits[240]);
void     mobitex_deinterleave(const uint8_t bits[240], uint16_t codewords[20]);

/* ---- Data Block Processing ---- */
int      mobitex_encode_block(mobitex_t *mobitex, const uint8_t data[18],
                              uint8_t bits_out[240]);
int      mobitex_decode_block(mobitex_t *mobitex, const uint8_t bits_in[240],
                              uint8_t data_out[18], int *fec_errors);

/* ---- Frame Header ---- */
void     mobitex_encode_header(const mobitex_frame_header_t *hdr,
                               uint8_t bits_out[MOBITEX_HEADER_BITS]);
int      mobitex_decode_header(const uint8_t bits_in[MOBITEX_HEADER_BITS],
                               mobitex_frame_header_t *hdr);

/* ---- Link Control ---- */
void     mobitex_parse_link_control(const uint8_t block[20],
                                    mobitex_link_control_t *lc);
void     mobitex_build_link_control(const mobitex_link_control_t *lc,
                                    uint8_t block[20]);

/* ---- MPAK Header ---- */
void     mobitex_parse_mpak_header(const uint8_t block[20],
                                   mobitex_mpak_header_t *mpak);
void     mobitex_build_mpak_header(const mobitex_mpak_header_t *mpak,
                                   uint8_t block[20]);

/* ---- Frame Sync ---- */
extern const uint16_t mobitex_frsyncs[18];
extern const uint16_t mobitex_frevsyncs[18];
int      mobitex_match_frsync(uint16_t word, int *index);

/* ---- Frequency Mapping ---- */
double   mobitex_fbi_to_freq(int fbi, int channel_number);

#endif /* FRAME_H */
