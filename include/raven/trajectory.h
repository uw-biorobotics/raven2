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

/**
*    File: trajectory.h
*    Created by Hawkeye 10/2011
*
*    Generate joint and cartesian trajectories.
*    Internal datastructures track trajectory state, and update DOFs as needed upon calling.
*/

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "struct.h"

// Setup and teardown of trajectory generation
// int start_trajectory(struct DOF*);
int start_trajectory(struct DOF*, float = 0, float = 0);
int start_trajectory_mag(struct DOF*, float = 0, float = 0);
int stop_trajectory(struct DOF*);

// Velocity Trajectories
int update_sinusoid_velocity_trajectory(struct DOF*);
int update_linear_sinusoid_velocity_trajectory(struct DOF*);

// Position trajectories
int update_sinusoid_position_trajectory(struct DOF*);
int update_linear_sinusoid_position_trajectory(struct DOF*);
int update_position_trajectory(struct DOF*);

#endif
