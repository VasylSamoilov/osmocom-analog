
#define COMPANDOR_LOOKAHEAD_MAX	128

typedef struct compandor {
	struct {
		double	step_up;
		double	step_down;
		double	peak;
		double	envelope;
		double	knee;	/* above this level of the pre-emphasized output its envelope is held, 0 = off */
		double	ke_a, ke_norm, ke_xl, ke_pk, ke_env;	/* pre-emphasis and envelope of the knee */
		double	kg_buf[COMPANDOR_LOOKAHEAD_MAX];	/* knee gains over the lookahead */
		int	lookahead;	/* samples the speech is delayed, so the gain is ready at an onset (0 = off) */
		int	la_pos;
		sample_t la_buf[COMPANDOR_LOOKAHEAD_MAX];
	} c;
	struct {
		double	step_up;
		double	step_down;
		double	peak;
		double	envelope;
		double	max_envelope;
	} e;
} compandor_t;

void compandor_init(void);
void setup_compandor(compandor_t *state, double samplerate, double attack_ms, double recovery_ms);
void compandor_set_expander_max(compandor_t *state, double max_envelope);
void compandor_set_compressor_lookahead(compandor_t *state, double samplerate, double lookahead_ms);
void compandor_set_compressor_knee(compandor_t *state, double samplerate, double knee, double emphasis_hz);
void compress_audio(compandor_t *state, sample_t *samples, int num);
void expand_audio(compandor_t *state, sample_t *samples, int num);

