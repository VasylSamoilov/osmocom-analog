#pragma once

#include <osmocom/core/logging.h>
#include "categories.h"

extern int loglevel;

/* Put before every LOGP line of this thread ("" = none), e.g. the station
 * a program with several of them works for now (osmoradio). */
extern __thread const char *logging_prefix;
#undef LOGP
#define LOGP(ss, level, fmt, args...) \
	LOGPSRC(ss, level, NULL, 0, "%s" fmt, logging_prefix, ## args)

#define LOGP_CHAN(cat, level, fmt, arg...) LOGP(cat, level, "(chan %s) " fmt, CHAN, ## arg)

void get_win_size(int *w, int *h);
void lock_logging(void);
void unlock_logging(void);
void enable_limit_scroll(bool enable);
void logging_limit_scroll_top(int lines);
void logging_limit_scroll_bottom(int lines);
const char *debug_amplitude(double level);
const char *debug_db(double level_db);
void logging_print_help(void);
int parse_logging_opt(const char *optarg);
void logging_init(void);

