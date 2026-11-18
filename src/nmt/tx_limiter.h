#ifndef _NMT_TX_LIMITER_H
#define _NMT_TX_LIMITER_H

/* Lookahead limiter of the speech towards the transmitter, see tx_limiter.c */
typedef struct tx_limiter {
	int		delay;		/* lookahead in samples */
	int		pre_emphasis;	/* the limit applies after the pre-emphasis */
	emphasis_t	estate;		/* pre-emphasis of the speech without gain, to find the peaks */
	double		limit;		/* highest amplitude after the pre-emphasis (and before the TX gain) */
	double		release;	/* factor per sample the gain may rise */
	sample_t	*x;		/* ring of the delayed input, delay + 1 */
	double		*r;		/* ring of the gain each input sample needs, delay + 1 */
	double		*m;		/* ring of the minimum of r over the last delay + 1 samples, delay + 1 */
	long		*dq;		/* monotonic deque of positions for the running minimum, delay + 2 */
	int		dq_head, dq_tail;
	long		pos;		/* input samples so far */
	double		m_sum;		/* sum of m over the last delay + 1 samples */
	double		gain;		/* gain of the last output sample */
	double		min_gain;	/* lowest gain since the last call of tx_limiter_min_gain() */
} tx_limiter_t;

int tx_limiter_init(tx_limiter_t *l, const emphasis_t *estate, int pre_emphasis, int samplerate, double limit, double lookahead_ms, double release_db_per_ms);
void tx_limiter_exit(tx_limiter_t *l);
void tx_limiter_reset(tx_limiter_t *l);
void tx_limiter_process(tx_limiter_t *l, sample_t *samples, int length);
double tx_limiter_min_gain(tx_limiter_t *l);

#endif
