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
 * \file init.h
 * \author Hawkeye
 * \version 2012
 * \brief Raven 2 Control Software
 *
 */

// Include files
#include <ros/ros.h>

#include "struct.h" /*Includes DS0, DS1, DOF_type*/
#include "r2_kinematics.h"
#include "fwd_cable_coupling.h"
#include "motor.h"
#include "USB_init.h"

#define JOINT_ENABLED 1

//
#define ENC_MAX_NOT_SET -10000
#define ENC_MIN_NOT_SET 10000

void initRobotData(device *device0, int runlevel, param_pass *currParams);

/// Structure initialization
void initDOFs(device *device0);

/// Get ravengains from ROS parameter server.
int init_ravengains(ros::NodeHandle n, device *device0);

/// set the starting xyz coordinate (pos_d = pos)
void setStartXYZ(device *device0);
