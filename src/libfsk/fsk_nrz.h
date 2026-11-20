#ifndef _FSK_NRZ_H
#define _FSK_NRZ_H

#include <stdint.h>
#include "../libsample/sample.h"

/* FSK of NRZ levels: the baseband that drives the FM deviation directly, each
 * symbol a level, the change from one level to the next a cosine ramp
 * (the pagers FLEX, POCSAG and GSC, and Mobitex).
 *
 * 2 levels use the outer levels 0 and 3 (-1 and +1 of the deviation), 4
 * levels all four (-1, -1/3, +1/3, +1). A symbol holds the level it starts
 * from for its first quarter, ramps in the middle half and holds the new
 * level for its last quarter.
 *
 * The symbol clock is continuous: the phase left by a symbol (how far the
 * next sample already is into the next symbol) carries on to the next one,
 * across a change of symbol rate too, and so does the level on the air. A
 * transmission after silence starts from phase 0 and the lowest level: the
 * transmitter calls fsk_nrz_mod_silence() when it sends silence (or
 * something else, as voice), and the next symbol restarts. */

#define FSK_NRZ_LEVELS	4

typedef struct fsk_nrz_mod {
	int		samplerate;
	double		deviation;		/* the outer levels */
	double		polarity;		/* 1.0 normal, -1.0 inverted */
	double		symbol_samples;		/* samples per symbol */
	double		step;			/* phase step per sample, 256 / symbol_samples */
	double		phase;			/* position of the next sample in its symbol, 0..256 */
	int		level;			/* level on the air, 0..3 */
	sample_t	last;			/* the last sample made */
	int		silence;		/* silence was sent: the next symbol starts a transmission */
	sample_t	ramps[FSK_NRZ_LEVELS][FSK_NRZ_LEVELS][256]; /* [from][to][phase] */
} fsk_nrz_mod_t;

void fsk_nrz_mod_init(fsk_nrz_mod_t *mod, int samplerate, double symbol_rate, double deviation, double polarity);
void fsk_nrz_mod_set_rate(fsk_nrz_mod_t *mod, double symbol_rate);
void fsk_nrz_mod_set_polarity(fsk_nrz_mod_t *mod, double polarity);
void fsk_nrz_mod_restart(fsk_nrz_mod_t *mod);
void fsk_nrz_mod_silence(fsk_nrz_mod_t *mod);
int fsk_nrz_mod_symbol(fsk_nrz_mod_t *mod, int level, sample_t *spl);
int fsk_nrz_mod_bits(fsk_nrz_mod_t *mod, uint32_t word, int nbits, sample_t *spl);
int fsk_nrz_mod_guard(fsk_nrz_mod_t *mod, sample_t *spl, int max);
int fsk_nrz_mod_max_samples(double samplerate, double symbol_rate, int symbols);

#endif /* _FSK_NRZ_H */
