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

/*********
 *
 * File: local_io.h
 *
 *  Functions for interfacing with FIFOs and local system.
 */

#ifndef LOCAL_IO_H
#define LOCAL_IO_H

#include <ros/ros.h>

#include "struct.h"
#include "defines.h"
#include "USB_init.h"
#include "itp_teleoperation.h"

int initLocalioData();

// update controller state w/ toolkit input
void teleopIntoDS1(u_struct *);

// fifo handler to recv command data
int receiveUserspace(void *u, int size);

// Check: have any command updates happened?
int checkLocalUpdates();

// Return current parameter-update set
param_pass *getRcvdParams(param_pass *);

void updateMasterRelativeOrigin(device *device0);

int init_ravenstate_publishing(ros::NodeHandle &n);
void publish_ravenstate_ros(robot_device *, param_pass *);
void setSurgeonMode(int pedalstate);

#endif
