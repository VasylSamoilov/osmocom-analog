/* Echo cancellation configuration */
typedef struct {
	int enabled;              /* Default: 0 (disabled) */
	int frame_size;           /* Default: 128 samples (debugging only) */
	int filter_length_ms;     /* Default: 0 (auto-detect with 500ms default) */
	double adapt_rate;        /* Default: 1.0 (debugging only) */
	int stats_enabled;        /* Default: 0 (debugging only) */
} echo_config_t;

extern echo_config_t echo_config;

extern const char *selected_config_file;
char *options_strdup(const char *s);
void option_add(int short_option, const char *long_option, int parameter_count);
int options_config_file(int argc, char *argv[], const char *config_file, int (*handle_options)(int short_option, int argi, char *argv[]));
int options_command_line(int argc, char *argv[], int (*handle_options)(int short_option, int argi, char *argv[]));
int option_is_first(void);
void options_free(void);

