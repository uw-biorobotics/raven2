/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
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
 * utils.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#include <time.h>

#ifndef NULL
#define NULL 0
#endif

//Short maximum and minimum
#define SHORT_MAX 32767
#define SHORT_MIN -32768

//Return values
#define SHORT_OVERFLOW    1
#define SHORT_UNDERFLOW  -1

int loop_over_joints(struct robot_device*, struct mechanism*&, struct DOF*&, int&, int&);
int loop_over_joints(struct mechanism* _mech, struct DOF*& _joint, int& jnum);

int toShort(int value, short int *target);
void strtoken(char *str, char *result, char delim);
void strcopy(const char *src, char *dest);

int is_toolDOF(struct DOF*);
int is_toolDOF(int);
int tools_ready(struct mechanism *mech);
int robot_ready(struct robot_device* device0);

const int _Qx=0, _Qy=1, _Qz=2, _Qw=3;
void getQuaternion(float* Q, float mat[3][3]);

/**
 * the struct timespec consists of nanoseconds
 * and seconds. This rolls over the ns to seconds.
 */
#define NSEC_PER_SEC    1000000000          // nanoseconds per sec
static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC)
    {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

struct  timespec  tsSubtract ( struct  timespec  time1,
                                           struct  timespec  time2);
// Reset posd so that it is coincident with pos.
void set_posd_to_pos(struct robot_device* device0);

#define isbefore(a, b) ( (a.tv_sec < b.tv_sec) || (a.tv_sec == b.tv_sec && a.tv_nsec < b.tv_nsec ))


#endif
