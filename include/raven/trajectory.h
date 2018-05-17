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
*    File: trajectory.h
*    Created by Hawkeye 10/2011
*
*    Generate joint and cartesian trajectories.
*    Internal datastructures track trajectory state, and update DOFs as needed
*upon calling.
*/

#include "struct.h"

// Setup and teardown of trajectory generation
// int start_trajectory(DOF*);
int start_trajectory(DOF *, float = 0, float = 0);
int start_trajectory_mag(DOF *, float = 0, float = 0);
int stop_trajectory(DOF *);

// Velocity Trajectories
int update_sinusoid_velocity_trajectory(DOF *);
int update_linear_sinusoid_velocity_trajectory(DOF *);

// Position trajectories
int update_sinusoid_position_trajectory(DOF *);
int update_linear_sinusoid_position_trajectory(DOF *);
int update_position_trajectory(DOF *);
