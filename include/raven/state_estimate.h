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

/***************************
 *
 * FILE: stateEstimate.h
 * Created May 2010 by H. Hawkeye King
 *
 *    I apply filters or whatever to get an estimate of the state
 * (position and velocity) of the system.
 *
 */

#include "struct.h"
#include "defines.h"
#include "dof.h"

void stateEstimate(robot_device *device0);
void getStateLPF(DOF *joint, int tool_type);
void resetFilter(DOF *_joint);
