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
* 	\file: rt_raven.h
*
*	\brief Runs all raven control functions.
*
*   \desc Code split out from rt_process_preempt.cpp, in order to provide more
*flexibility.
*
* 	\author Hawkeye
*
*	\date 10/2011
*
*   \ingroup Control
*/

#include "DS0.h"
#include "DS1.h"

/** prototype for controlRaven()
 */
int controlRaven(robot_device *, param_pass *);
