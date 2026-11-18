
typedef struct compandor {
	struct {
		double	step_up;
		double	step_down;
		double	peak;
		double	envelope;
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
void compress_audio(compandor_t *state, sample_t *samples, int num);
void expand_audio(compandor_t *state, sample_t *samples, int num);

