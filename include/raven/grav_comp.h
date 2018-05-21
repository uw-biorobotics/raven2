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

/*
 *  FILE: GravComp.c
 *
 *  Re-written March 2013 by Andy Lewis and Hawkeye King
 *    Equations re-derived for UW Kinematics formulations for Raven II
 *    See forthcoming technical report for details.
 */

#ifndef GRAV_COMP_H
#define GRAV_COMP_H

#include <cmath>
#include <tf/transform_datatypes.h>

#include "struct.h"
#include "defines.h"

/*
 * Calculate gravity load on joints 1,2,3 on both arms
 */
void getGravityTorque(device &d0, param_pass &params);

#endif
