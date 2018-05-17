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
 * 	\file utils.cpp
 *
 * 	\brief some common utility functions
 *
 * 	\author Kenneth Fodero
 *
 *	\date 2005
 */

#include <cmath>
#include "utils.h"
#include "DS0.h"
#include "defines.h"

extern int NUM_MECH;

/**
*	\fn int toShort(int value, short int *target)
*
* 	\brief function that takes an integer and places it in the target short
*int, reporting under/overflow
*
* 	\param value int value to convert
* 	\param target short int pointer that points to the converted short value
*
* 	\return 1 if overflow
*		   -1 if underflow
*			0 if no problem
*/
int toShort(int value, short int *target) {
  // Overflow
  if (value > SHORT_MAX) {
    *target = SHORT_MAX;
    return SHORT_OVERFLOW;
  }
  // Underflow
  else if (value < SHORT_MIN) {
    *target = SHORT_MIN;
    return SHORT_UNDERFLOW;
  }
  // No problems
  else {
    *target = value;
    return 0;
  }
}

/**
*	\fn int loop_over_joints(robot_device* device0, mechanism*& _mech, DOF*&
*_joint, int& mechnum, int& jnum)
*
*	\brief Iterate over all joints of all mechanisms
*
*	\desc To start iteration, call function with _joint == _mech == NULL
*  		Function iterates by incrementing jnum and mnum from zero and
*returning the appropriate mech and joint.
*  		Iteration terminates when function is called with jnum ==
*MAX_DOF-1 and mnum = NUM_MECH-1.
*  		In the terminating case, mnum and jnum are not modified.
*_joint and _mech are also unmodified in that case.
*
*  	\pre  mechnum and jnum point to _previous_ joint.
*                   At start of iteration _mech == _joint == NULL
*  	\post jnum and mnum are incremented or reset as necessary
*                   _mech points to device0.mech[mechnum]
*                   _joint points to device0.mech[mechnum].joint[jnum+1]
*
*	\param device0 a pointer points to the robot_device struct
* 	\param _mech   a pointer points to the mechanism struct, and it is
*updated at the end of the function
* 	\param _joint  a pointer points to the DOF struct, and it is updated at
*the end of the function
* 	\param mechnum integer value represents the mechanism id, and it is
*updated at the end of the function
* 	\param jnum    integer value represents the joint id, and it is updated
*at the end of the function
*
*	\return 0 if reaches last joint index
*			1 otherwise
*/
int loop_over_joints(robot_device *device0, mechanism *&_mech, DOF *&_joint, int &mechnum,
                     int &jnum) {
  // Initialize iterators
  if (_mech == NULL || _joint == NULL) {
    mechnum = 0;
    jnum = 0;
  }

  // Terminating condition
  else if (mechnum >= (NUM_MECH - 1) && jnum >= (MAX_DOF_PER_MECH - 1))
    return 0;

  // Joint rollover
  else if (jnum >= (MAX_DOF_PER_MECH - 1)) {
    mechnum++;
    jnum = 0;
  }

  // Joint increment
  else {
    jnum++;
    if (jnum == NO_CONNECTION) jnum++;
  }

  // Set return structs
  _mech = &(device0->mech[mechnum]);
  _joint = &(device0->mech[mechnum].joint[jnum]);

  return 1;
}

/**
*	\fn loop_over_joints(mechanism* _mech, DOF*& _joint, int& jnum)
*
*	\brief Iterate over all joints of one mechanism.
*
*	\desc To start iteration, call function with _joint == NULL
*  		Function iterates by incrementing jnum from zero and returning
*the appropriate joint.
*  		Iteration terminates when function is called with jnum ==
*MAX_DOF-1.
*  		In the terminating case, jnum is not modified.  _joint is also
*unmodified in that case.
*
*  	\pre  jnum point to _previous_ joint.
*                   At start of iteration _joint == NULL
*  	\post jnum is incremented or reset as necessary
*                   _joint points to mech->joint[jnum+1]
*
* 	\param _mech   a pointer points to the mechanism struct, and it is
*updated at the end of the function
* 	\param _joint  a pointer points to the DOF struct, and it is updated at
*the end of the function
* 	\param jnum    integer value represents the joint id, and it is updated
*at the end of the function
*
*	\return 0 if reaches to last joint index
*			1 otherwise
*/
int loop_over_joints(mechanism *_mech, DOF *&_joint, int &jnum) {
  // Initialize iterators
  if (_joint == NULL) {
    jnum = 0;
  }

  // Terminating condition
  else if (jnum >= (MAX_DOF_PER_MECH - 1))
    return 0;

  // Joint increment
  else {
    jnum++;
    if (jnum == NO_CONNECTION) jnum++;
  }

  // Set return structs
  _joint = &(_mech->joint[jnum]);

  return 1;
}

/**
*	\fn int is_toolDOF(DOF *_joint)
*
*	\brief check if the current joint is a toolDOF
*
*	\param _joint a DOF struct
*
*	\return 1 if the joint is a toolDOF
*			0 otherwise
*/
int is_toolDOF(DOF *_joint) { return is_toolDOF(_joint->type); }

/**
*	\fn int is_toolDOF(int jointType)
*
*	\brief check if the current joint is a toolDOF
*
*	\param _joint an integer value of joint type
*
*	\return 1 if the joint is a toolDOF
*			0 otherwise
*/
int is_toolDOF(int jointType) {
  if (jointType == TOOL_ROT_GOLD || jointType == TOOL_ROT_GREEN) return 1;
  if (jointType == WRIST_GOLD || jointType == WRIST_GREEN) return 1;
  if (jointType == GRASP1_GOLD || jointType == GRASP1_GREEN) return 1;
  if (jointType == GRASP2_GOLD || jointType == GRASP2_GREEN) return 1;

  return 0;
}

/**
*	\fn int tools_ready(mechanism *mech)
*
* 	\brief check if all tool joints of the current mechanism are in the
*ready state
*
* 	\param 	mech    pointer to the mechanism struct
*
* 	\return 1 if and only if all toolDOFs are ready
*			0 otherwise
*/
int tools_ready(mechanism *mech) {
  if (mech->joint[TOOL_ROT].state != jstate_ready) return 0;
  if (mech->joint[WRIST].state != jstate_ready) return 0;
  if (mech->joint[GRASP1].state != jstate_ready) return 0;
  if (mech->joint[GRASP2].state != jstate_ready) return 0;

  return 1;
}

/**
*	\fn int robot_ready(robot_device* device0)
*
*	\brief check if robot is ready during homing procedure
*
*	\param mech  pointer to the robot_device struct
*
*	\return 1 if and only if all DOFS are ready
*			0 otherwise
*/
int robot_ready(robot_device *device0) {
  mechanism *_mech = NULL;
  DOF *_joint = NULL;
  int i, j;

  while (loop_over_joints(device0, _mech, _joint, i, j)) {
    if (_joint->state != jstate_ready) return 0;
  }
  return 1;
}

/**
*	\fn  timespec tsSubtract (timespec time1, timespec time2)
*
*	\brief function to get time interval
*
*	\param time1 - struct  timespec
* 	\param time2 - struct  timespec
*
*	\return time1-time2  or  (0,0) if time2>time1
*
*	\return timespec struct with time interval result.tv_nsec
*/
timespec tsSubtract(timespec time1, timespec time2) {
  timespec result;

  /* Subtract the second time from the first. */
  if ((time1.tv_sec < time2.tv_sec) ||
      ((time1.tv_sec == time2.tv_sec) && (time1.tv_nsec <= time2.tv_nsec))) /* TIME1 <= TIME2? */
  {
    result.tv_sec = result.tv_nsec = 0;
  } else /* TIME1 > TIME2 */
  {
    result.tv_sec = time1.tv_sec - time2.tv_sec;
    if (time1.tv_nsec < time2.tv_nsec) {
      result.tv_nsec = time1.tv_nsec + 1000000000L - time2.tv_nsec;
      result.tv_sec--; /* Borrow a second. */
    } else {
      result.tv_nsec = time1.tv_nsec - time2.tv_nsec;
    }
  }
  return (result);
}

/**
*	\fn void set_posd_to_pos(robot_device* device0)
*
*	\brief set the desired position to the robots current position
*
*	\param device0 a pointer points to the robot_device struct
*
*	\return void
*/
void set_posd_to_pos(robot_device *device0) {
  for (int m = 0; m < NUM_MECH; m++) {
    device0->mech[m].pos_d.x = device0->mech[m].pos.x;
    device0->mech[m].pos_d.y = device0->mech[m].pos.y;
    device0->mech[m].pos_d.z = device0->mech[m].pos.z;
    device0->mech[m].ori_d.yaw = device0->mech[m].ori.yaw;
    device0->mech[m].ori_d.pitch = device0->mech[m].ori.pitch;
    device0->mech[m].ori_d.roll = device0->mech[m].ori.roll;
    device0->mech[m].ori_d.grasp = device0->mech[m].ori.grasp;

    for (int k = 0; k < 3; k++)
      for (int j = 0; j < 3; j++) device0->mech[m].ori_d.R[k][j] = device0->mech[m].ori.R[k][j];
  }
}
