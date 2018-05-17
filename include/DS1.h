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

/***********************************************
 **
 ** File: DS1.h
 ** Authors: Hawkeye King, Arash Aminpour
 **
 ** This file implements a data structure to pass
 ** control parameters to the device-control module
 ** from user-space.
 **
 ***********************************************/

#ifndef DS1_H
#define DS1_H

#ifndef DS0_H
#include "DS0.h"
#endif

#define STOP 0  // runlevel 0 is STOP state

//* \todo Delete stuff from OLD R_I code!
struct param_pass {
  u_08 runlevel;                                         // device runlevel
  u_08 sublevel;                                         // device runlevel
  int enc_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];        // desired encoder position
  int dac_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];        // desired dac level
  float jpos_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];     // desired joint coordinates
  float jvel_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];     // desired joint velocity
  float kp[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];         // position gain
  float kd[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];         // derivative gain
  position xd[MAX_MECH_PER_DEV];                         // desired end-point position
  orientation rd[MAX_MECH_PER_DEV];                      // desired end-point orientation
  int torque_vals[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];  // desired force/torque
  float grav_mag;                                        // gravity magnitude
  position grav_dir;                                     // gravity direction
  char cmdStr[200];
  int surgeon_mode;
  int robotControlMode;
  int last_sequence;
};

#endif
