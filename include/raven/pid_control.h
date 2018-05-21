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

/**
 * pid_control.h - control law
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * $Id: pd_control.h,v 1.1 2007/03/22 20:17:59 hawkeye1 Exp $
 */

#ifndef PD_CONTROL_H
#define PD_CONTROL_H

// Local include files
#include "struct.h" /*Includes DS0, DS1, DOF_type*/
#include "dof.h"

// PD Controller type defines
#define JOINT_PD_CTRL 1
#define MOTOR_PD_CTRL 2
#define MOTOR_VEL_CTRL 2

// Function Prototypes
void mpos_PD_control(DOF *joint, int reset_I = 0);
float jvel_PI_control(DOF *, int);

#endif  // PD_CONTROL_H
