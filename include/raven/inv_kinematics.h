/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington
 *BioRobotics Laboratory
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
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/*************************************************
 *
 * File: inv_kinematics.h
 *
 * History:
 *  Created 26 July 005 by Mitch
 *
 ************************************************/
#ifndef INV_KINEMATICS_H
#define INV_KINEMATICS_H

#include "struct.h"
#include "defines.h"
#include "log.h"

#define _A5 0.0087  // 8.7mm
// Link Angles/Lengths taken from defines.h
const double a12 = 1.30899694;  /*Link1 - 75deg in RAD*/
const double a23 = 0.907571211; /*Link2 - 52deg in RAD - was set to 60*/

const double base_tilt = 0 DEG2RAD;  // Accomodates for 25deg offset from zero angle in shoulder

// Constant DH parameters
const double go_dh_al[6] = {0, -a12, M_PI - a23, 0, M_PI / 2, -M_PI / 2};
const double go_dh_a[6] = {0, 0, 0, 0, 0, 0};
const double gr_dh_al[6] = {M_PI, a12, a23, M_PI, M_PI / 2, M_PI / 2};
const double gr_dh_a[6] = {0, 0, 0, 0, 0, 0};

void invKin(struct device *device0, struct param_pass *currParam);
int invMechKin(struct mechanism *mech);

#endif
