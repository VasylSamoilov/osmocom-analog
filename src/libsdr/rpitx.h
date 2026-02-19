/* rpitx backend - Raspberry Pi GPIO RF transmitter via librpitx
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

#ifndef RPITX_H
#define RPITX_H

int rpitx_open(double tx_frequency, double rate, double tx_gain);
int rpitx_start(void);
int rpitx_send(float *buff, int num);
void rpitx_close(void);
int rpitx_get_tosend(int buffer_size);

#endif /* RPITX_H */
