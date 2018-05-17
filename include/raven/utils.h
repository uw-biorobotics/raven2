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
* 	\file utils.h
*
* 	\brief some common utility functions
*
* 	\author Kenneth Fodero
*
* 	\date 2005
*/

#ifndef __UTILS_H__
#define __UTILS_H__

#include <ctime>
#include "DS0.h"

#ifndef NULL
#define NULL 0
#endif

// Short maximum and minimum
#define SHORT_MAX 32767
#define SHORT_MIN -32768

// Return values
#define SHORT_OVERFLOW 1
#define SHORT_UNDERFLOW -1

int loop_over_joints(robot_device *, mechanism *&, DOF *&, int &, int &);
int loop_over_joints(mechanism *_mech, DOF *&_joint, int &jnum);

int toShort(int value, short int *target);

int is_toolDOF(DOF *);
int is_toolDOF(int);
int tools_ready(mechanism *mech);
int robot_ready(robot_device *device0);

/**
*	\fn static inline void tsnorm(timespec *ts)
*
*	\brief the struct timespec consists of nanoseconds
* 		and seconds. This rolls over the ns to seconds.
*
*	\param ts 	timespec struct containing nanosec times to convert to
*sec
*/
#define NSEC_PER_SEC 1000000000  // nanoseconds per sec
static inline void tsnorm(timespec *ts) {
  while (ts->tv_nsec >= NSEC_PER_SEC) {
    ts->tv_nsec -= NSEC_PER_SEC;
    ts->tv_sec++;
  }
}

timespec tsSubtract(timespec time1, timespec time2);

// Reset posd so that it is coincident with pos.
void set_posd_to_pos(robot_device *device0);

#define isbefore(a, b) ((a.tv_sec < b.tv_sec) || (a.tv_sec == b.tv_sec && a.tv_nsec < b.tv_nsec))

#endif
