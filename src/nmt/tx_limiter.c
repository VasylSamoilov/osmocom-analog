/* Lookahead speech limiter of the NMT transmitter
 *
 * NMT Doc 900-4 4.8.1 and 4.10: the maximum permissible frequency deviation is 4,7 kHz; 4.9: the supervisory
 * signal does not pass through the limiter. The speech is limited before the supervisory tone is added, so
 * that its peak after the pre-emphasis (done by the sender afterwards, when on) stays below the given limit.
 *
 * The pre-emphasis is close to a differentiator at high sample rates: a step of the gain gives a spike, and
 * a block cannot start at a lower gain than the previous one ended with. So the speech is delayed by the
 * lookahead, and the gain is lowered smoothly before a peak arrives:
 *  - r: the gain each sample needs: limit / |pre-emphasis of the speech| (at most 1);
 *  - m: the minimum of r over the last lookahead + 1 samples (running minimum);
 *  - the gain of the output sample (lookahead samples ago) is the mean of m over the last lookahead + 1
 *    samples. Every m in that window includes the peak, so the gain at the peak is at most its r.
 * The gain rises at most by the release rate. Since the gain changes smoothly over the lookahead, the
 * pre-emphasis of the scaled speech is the scaled pre-emphasis, within the margin given with the limit.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include "../libsample/sample.h"
#include "../libemphasis/emphasis.h"
#include "tx_limiter.h"

int tx_limiter_init(tx_limiter_t *l, const emphasis_t *estate, int pre_emphasis, int samplerate, double limit, double lookahead_ms, double release_db_per_ms)
{
	int n;

	memset(l, 0, sizeof(*l));
	l->delay = (int)((double)samplerate * lookahead_ms / 1000.0 + 0.5);
	if (l->delay < 1)
		l->delay = 1;
	n = l->delay + 1;
	l->pre_emphasis = pre_emphasis;
	if (estate)
		memcpy(&l->estate, estate, sizeof(l->estate));
	l->limit = limit;
	l->release = pow(10.0, release_db_per_ms / 20.0 * 1000.0 / (double)samplerate);
	l->x = calloc(n, sizeof(*l->x));
	l->r = calloc(n, sizeof(*l->r));
	l->m = calloc(n, sizeof(*l->m));
	l->dq = calloc(n + 1, sizeof(*l->dq));
	if (!l->x || !l->r || !l->m || !l->dq) {
		tx_limiter_exit(l);
		return -ENOMEM;
	}
	tx_limiter_reset(l);
	return 0;
}

void tx_limiter_exit(tx_limiter_t *l)
{
	free(l->x);
	free(l->r);
	free(l->m);
	free(l->dq);
	l->x = NULL;
	l->r = NULL;
	l->m = NULL;
	l->dq = NULL;
}

void tx_limiter_reset(tx_limiter_t *l)
{
	int n = l->delay + 1, i;

	if (!l->x)
		return;
	memset(l->x, 0, sizeof(*l->x) * n);
	for (i = 0; i < n; i++) {
		l->r[i] = 1.0;
		l->m[i] = 1.0;
	}
	l->dq_head = l->dq_tail = 0;
	l->pos = 0;
	l->m_sum = (double)n;
	l->gain = 1.0;
	l->min_gain = 1.0;
	/* the state of the pre-emphasis follows the speech from now on */
	memset(l->estate.p.lp.z1, 0, sizeof(l->estate.p.lp.z1));
	memset(l->estate.p.lp.z2, 0, sizeof(l->estate.p.lp.z2));
	l->estate.p.x_last = 0.0;
}

void tx_limiter_process(tx_limiter_t *l, sample_t *samples, int length)
{
	int n = l->delay + 1, dqn = l->delay + 2, i, c;
	sample_t ebuf[256];

	if (!l->x)
		return;
	for (i = 0; i < length; i++) {
		sample_t in = samples[i], e, out;
		long p = l->pos;
		int slot = (int)(p % n);
		double r, m, g;

		/* the pre-emphasis of the speech without gain, in chunks */
		c = i % 256;
		if (c == 0) {
			int num = (length - i < 256) ? length - i : 256;

			memcpy(ebuf, samples + i, sizeof(*ebuf) * num);
			if (l->pre_emphasis)
				pre_emphasis(&l->estate, ebuf, num);
		}
		e = ebuf[c];

		/* the gain this sample needs, after the pre-emphasis */
		r = (fabs(e) > l->limit) ? l->limit / fabs(e) : 1.0;

		/* the new sample enters, the one of position p - delay leaves (with n = delay + 1 slots, it sits in the
		 * slot after the new one) */
		l->x[slot] = in;
		l->r[slot] = r;
		out = l->x[(slot + 1) % n];

		/* running minimum of r over the positions p - delay .. p (monotonic deque of positions, its own ring
		 * of delay + 2, since it holds the new position before the expired one leaves) */
		while (l->dq_tail != l->dq_head) {
			int last = (l->dq_tail + dqn - 1) % dqn;

			if (l->r[l->dq[last] % n] < r)
				break;
			l->dq_tail = last;
		}
		l->dq[l->dq_tail] = p;
		l->dq_tail = (l->dq_tail + 1) % dqn;
		while (l->dq[l->dq_head] < p - l->delay)
			l->dq_head = (l->dq_head + 1) % dqn;
		m = l->r[l->dq[l->dq_head] % n];

		/* mean of m over the last delay + 1 samples */
		l->m_sum += m - l->m[slot];
		l->m[slot] = m;
		g = l->m_sum / (double)n;

		/* the gain rises at most by the release rate */
		if (g > l->gain * l->release)
			g = l->gain * l->release;
		if (g > 1.0)
			g = 1.0;
		l->gain = g;
		if (g < l->min_gain)
			l->min_gain = g;
		samples[i] = out * g;
		l->pos++;
	}
}

/* lowest gain since the last call, and start again */
double tx_limiter_min_gain(tx_limiter_t *l)
{
	double g = l->min_gain;

	l->min_gain = 1.0;
	return g;
}
