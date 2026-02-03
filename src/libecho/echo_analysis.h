/* Echo Delay Analysis for Analog Telephony
 *
 * (C) 2026 by Vasyl Samoilov <vasyl.samoilov@gmail.com>
 * All Rights Reserved
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

#ifndef _ECHO_ANALYSIS_H
#define _ECHO_ANALYSIS_H

#include <stdint.h>
#include "../libsample/sample.h"

/* Forward declaration of opaque state structure */
typedef struct echo_analysis_state echo_analysis_state_t;

/* Initialize echo analysis
 * duration_sec: Test duration in seconds (default 10)
 * tones_per_combo: Tones per combination (3-5, default 3)
 * Returns: Pointer to state, or NULL on error
 */
echo_analysis_state_t *echo_analysis_init(int duration_sec, int tones_per_combo);

/* Process TX audio - generates test tones
 * state: Echo analysis state
 * samples: Output buffer for generated tones
 * count: Number of samples to generate
 */
void echo_analysis_process_tx(echo_analysis_state_t *state, sample_t *samples, int count);

/* Process RX audio - detects echoed tones
 * state: Echo analysis state
 * samples: Input buffer with received audio
 * count: Number of samples
 */
void echo_analysis_process_rx(echo_analysis_state_t *state, sample_t *samples, int count);

/* Check if analysis is complete
 * state: Echo analysis state
 * Returns: 1 if complete, 0 if still running
 */
int echo_analysis_is_complete(echo_analysis_state_t *state);

/* Generate and print final report
 * state: Echo analysis state
 */
void echo_analysis_print_report(echo_analysis_state_t *state);

/* Cleanup echo analysis
 * state: Echo analysis state to free
 */
void echo_analysis_cleanup(echo_analysis_state_t *state);

#endif /* _ECHO_ANALYSIS_H */
