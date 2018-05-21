/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University
 *of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU  General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/** Class header for jacobian #defines to make equations seem a little more
 *readable
 *
 *
 *  \date May 18, 2016
 *  \author Andy Lewis
 */

#ifndef R2_JACOBIAN_DEFS_H_
#define R2_JACOBIAN_DEFS_H_

#include <cmath>

#define L 2     // shouldn't be 2
#define La12 1  // shouldn't be 1
#define La23 1  // shouldn't be 1

// DH pararms
#define A3 6  // shouldn't be 6
#define TH3_GREEN -M_PI / 2
#define TH3_GOLD M_PI / 2
float dh_alpha_green[6] = {M_PI, La12, La23, 0, M_PI / 2, M_PI / 2};
float dh_alpha_gold[6] = {0, La12, M_PI - La23, 0, M_PI / 2, M_PI / 2};
float dh_aa[6] = {0, 0, 0, A3, 0, 1};  // dh_a[5] is lw

// function-set variables
// lw - wrist length
// d4 - shaft lenth
//
// d3 - insertion joint value
// dh_alpha		j_pos

#define Ca12 cos(dh_alpha[0])
#define Ca23 cos(dh_alpha[1])

#define Sa12 sin(dh_alpha[0])
#define Sa23 sin(dh_alpha[1])

#define C2 cos(j_pos[1])
#define C4 cos(j_pos[3])
#define C5 cos(j_pos[4])
#define C6 cos(j_pos[5])

#define S2 sin(j_pos[1])
#define S4 sin(j_pos[3])
#define S5 sin(j_pos[4])
#define S6 sin(j_pos[5])

#define D3 j_pos[2]

#endif /* R2_JACOBIAN_DEFS_H_ */
