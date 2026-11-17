
/* Codepage of the alphanumeric messages, and the Japanese Text Service:
 * HEX/binary messages of 16 or 8-bit Shift JIS characters (defined in
 * main.c, set by --charset) */
struct pager_charset;
extern const struct pager_charset *flex_charset;
extern int flex_kanji;

int dsp_init_sender(flex_t *flex, int samplerate, double deviation, double polarity, int enable_lpf);
void dsp_set_speed(flex_t *flex, int bitrate, int modulation_type);
void dsp_set_polarity(flex_t *flex, double polarity);
void dsp_cleanup_sender(flex_t *flex);
