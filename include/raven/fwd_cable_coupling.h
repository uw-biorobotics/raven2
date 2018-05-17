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
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/*************************************************
 *
 * File: fwd_cable_coupling.h
 *
 * History:
 *  Created 3 August 2005 by Mitch
 *
 ************************************************/

//#include <linux/kernel.h>
//#include <linux/module.h>
//#include <rtai.h>

#include "struct.h"
#include "utils.h"
#include "defines.h"

#define GB_RATIO (GEAR_BOX_GP42_TR / GEAR_BOX_GP32_TR * (CAPSTAN_RADIUS_GP32 / CAPSTAN_RADIUS_GP42))

// 0 – shoulder
// 1- elbow
// 2- insertion and tool DOFs
// cable coupling from joint 0 to joint 1
#define CABLE_COUPLING_01 (float)(0.14545)  // averaged from Ji's calculations
// cable coupling from joint 0 to joint 2
#define CABLE_COUPLING_02 (float)(0.0077973826)  // averaged from Ji's calcs
// cable coupling from joint 1 to joint 2
#define CABLE_COUPLING_12 (float)(0.008077387)  // averaged from Ji's calcs

void fwdCableCoupling(device *device0, int runlevel);
void fwdMechCableCoupling(mechanism *mech);

void fwdTorqueCoupling(device *device0, int runlevel);
void fwdMechTorqueCoupling(mechanism *mech);

int fwdJointEncoders(device *device0);
