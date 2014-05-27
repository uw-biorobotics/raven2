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
 * \file utils.cpp
 * \brief some common utility functions
 *      toShort - convert an int to a short int
 *
 * \author Kenneth Fodero, Biorobotics Lab 
 * \date 2005
 *
 */

#include <math.h>
#include "utils.h"
#include "DS0.h"
#include "defines.h"

extern int NUM_MECH;
/**\fn int toShort(int value, short int *target)
 * \brief function that takes an integer and places it in the target short int, reporting under/overflow
 * \param value int value to convert
 * \param target short int pointer that points to the converted short value
 * \return 1 if overflow; -1 if underflow; 0 if no problem
 */
int toShort(int value, short int *target)
{

    //Overflow
    if (value > SHORT_MAX)
    {
        *target = SHORT_MAX;
        return SHORT_OVERFLOW;
    }
    //Underflow
    else if (value < SHORT_MIN)
    {
        *target = SHORT_MIN;
        return SHORT_UNDERFLOW;
    }
    //No problems
    else
    {
        *target = value;
        return 0;
    }
}


/**\fn int loop_over_joints(struct robot_device* device0, struct mechanism*& _mech, struct DOF*& _joint, int& mechnum, int& jnum)
 * \brief Iterate over all joints of all mechanisms
 *  To start iteration, call function with _joint == _mech == NULL
 *  Function iterates by incrementing jnum and mnum from zero and returning the appropriate mech and joint.
 *  Iteration terminates when function is called with jnum == MAX_DOF-1 and mnum = NUM_MECH-1.
 *  In the terminating case, mnum and jnum are not modified.  _joint and _mech are also unmodified in that case.
 *
 *  Precondition:  mechnum and jnum point to _previous_ joint.
 *                   At start of iteration _mech == _joint == NULL
 *
 *  Postcondition: jnum and mnum are incremented or reset as necessary
 *                   _mech points to device0.mech[mechnum]
 *                   _joint points to device0.mech[mechnum].joint[jnum+1]
 * \param device0 a pointer points to the robot_device struct
 * \param _mech   a pointer points to the mechanism struct, and it is updated at the end of the function
 * \param _joint  a pointer points to the DOF struct, and it is updated at the end of the function
 * \param mechnum integer value represents the mechanism id, and it is updated at the end of the function
 * \param jnum    integer value represents the joint id, and it is updated at the end of the function
 * \return 0 if reaches to last joint index; 1 elsewise
 */
int loop_over_joints(struct robot_device* device0, struct mechanism*& _mech, struct DOF*& _joint, int& mechnum, int& jnum)
{
    // Initialize iterators
    if (_mech == NULL || _joint == NULL)
    {
        mechnum = 0;
        jnum = 0;
    }

    // Terminating condition
    else if ( mechnum >= (NUM_MECH-1) && jnum >= (MAX_DOF_PER_MECH-1) )
        return 0;

    // Joint rollover
    else if ( jnum >= (MAX_DOF_PER_MECH-1) )
    {
        mechnum++;
        jnum=0;
    }

    // Joint increment
    else
    {
        jnum++;
        if (jnum == NO_CONNECTION) jnum++;
    }

    // Set return structs
    _mech = &(device0->mech[mechnum]);
    _joint =&(device0->mech[mechnum].joint[jnum]);

    return 1;
}

/**\fn loop_over_joints(struct mechanism* _mech, struct DOF*& _joint, int& jnum)
 * \brief Iterate over all joints of one mechanism.
 *  To start iteration, call function with _joint == NULL
 *  Function iterates by incrementing jnum from zero and returning the appropriate joint.
 *  Iteration terminates when function is called with jnum == MAX_DOF-1.
 *  In the terminating case, jnum is not modified.  _joint is also unmodified in that case.
 *
 *  Precondition:  jnum point to _previous_ joint.
 *                   At start of iteration _joint == NULL
 *
 *  Postcondition: jnum is incremented or reset as necessary
 *                   _joint points to mech->joint[jnum+1]
 * \param _mech   a pointer points to the mechanism struct, and it is updated at the end of the function
 * \param _joint  a pointer points to the DOF struct, and it is updated at the end of the function
 * \param jnum    integer value represents the joint id, and it is updated at the end of the function
 * \return 0 if reaches to last joint index; 1 elsewise
 */
int loop_over_joints(struct mechanism* _mech, struct DOF*& _joint, int& jnum)
{
    // Initialize iterators
    if (_joint == NULL)
    {
        jnum = 0;
    }

    // Terminating condition
    else if ( jnum >= (MAX_DOF_PER_MECH-1) )
        return 0;

    // Joint increment
    else
    {
        jnum++;
        if (jnum == NO_CONNECTION) jnum++;
    }

    // Set return structs
    _joint =&(_mech->joint[jnum]);

    return 1;
}


/**\fn int is_toolDOF(struct DOF *_joint)
 * \brief check if the current joint is a toolDOF
 * \param _joint a DOF struct 
 * \return 1 if the joint is a toolDOF;elsewise 0
 */
int is_toolDOF(struct DOF *_joint){
    return is_toolDOF(_joint->type);
}

/**\fn int is_toolDOF(int jointType)
 * \brief check if the current joint is a toolDOF
 * \param _joint an integer value of joint type
 * \return 1 if the joint is a toolDOF;elsewise 0
 */
int is_toolDOF(int jointType)
{
    if ( jointType == TOOL_ROT_GOLD || jointType == TOOL_ROT_GREEN)
        return 1;
    if ( jointType == WRIST_GOLD    || jointType == WRIST_GREEN)
        return 1;
    if ( jointType == GRASP1_GOLD   || jointType == GRASP1_GREEN)
        return 1;
    if ( jointType == GRASP2_GOLD   || jointType == GRASP2_GREEN)
        return 1;

    return 0;
}


/**\fn int tools_ready(struct mechanism *mech)
 *
 * \brief check if all tool joints of the current mechanism are in the ready state
 *
 * \param 	mech 		a pointer points to the mechanism struct
 *
 * \return 	1 			if and only if all toolDOFs are ready, otherwise 0
 */
int tools_ready(struct mechanism *mech)
{
    if ( mech->joint[TOOL_ROT].state != jstate_ready )
        return 0;
    if ( mech->joint[WRIST].state    != jstate_ready )
        return 0;
    if ( mech->joint[GRASP1].state   != jstate_ready )
        return 0;
    if ( mech->joint[GRASP2].state   != jstate_ready )
        return 0;

    return 1;
}



/**\fn int robot_ready(struct robot_device* device0)
 * \brief check if robot is ready during homing procedure
 * \param mech a pointer points to the robot_device struct
 * \return 1 if and only if all DOFS are ready, elsewise 0
 */
int robot_ready(struct robot_device* device0)
{
    struct mechanism* _mech = NULL;
    struct DOF* _joint = NULL;
    int i, j;

    while ( loop_over_joints(device0, _mech, _joint, i, j) )
    {
        if (_joint->state != jstate_ready)
            return 0;
    }
    return 1;
}


/**\fn void strtoken(char *str, char *result, char delim)
 * \brief function to tokenize a string
 * \param str - the string
 * \param result - the resulting string
 * \param delim - the delimeter
 *
 * \return void
 */
void strtoken(char *str, char *result, char delim)
{
    static char data[200] = {0};
    static int index = 0;
    int i = 0;

    //Copy over string
    if (str != NULL)
    {
        strcopy(str, data);
        index = 0;
    }

    //Loop through for delimeter
    while (data[index] != '\0')
    {
        //Found delimeter
        if (data[index] == delim)
        {
            result[i] = '\0';
            index++;
            return;
        }

        result[i++] = data[index];
        index++;
    }

    result[i] = NULL;
    return;
}


/**\fn void strcopy(const char *src, char *dest)
 * \brief function to copy a string from source to destionation
 * \param src - the source string
 * \param dest - the resulting string
 * \return void
 */
void strcopy(const char *src, char *dest)
{
    int i = 0;

    while (src[i] != '\0')
    {
        dest[i] = src[i];
        i++;
    }

    dest[i] = NULL;
}



/**\fn struct  timespec  tsSubtract ( struct  timespec  time1, struct  timespec  time2)
 * \brief function to get time interval
 * \param time1 - struct  timespec
 * \param time2 - struct  timespec
 * \return time1-time2  or  (0,0) if time2>time1
 */
struct  timespec  tsSubtract ( struct  timespec  time1,
                                           struct  timespec  time2)
{
    struct  timespec  result ;

    /* Subtract the second time from the first. */
    if ((time1.tv_sec < time2.tv_sec) ||
            ((time1.tv_sec == time2.tv_sec) &&
             (time1.tv_nsec <= time2.tv_nsec)))   /* TIME1 <= TIME2? */
    {
        result.tv_sec = result.tv_nsec = 0 ;
    }
    else                                      /* TIME1 > TIME2 */
    {
        result.tv_sec = time1.tv_sec - time2.tv_sec ;
        if (time1.tv_nsec < time2.tv_nsec)
        {
            result.tv_nsec = time1.tv_nsec + 1000000000L - time2.tv_nsec ;
            result.tv_sec-- ;                    /* Borrow a second. */
        }
        else
        {
            result.tv_nsec = time1.tv_nsec - time2.tv_nsec ;
        }
    }
    return (result);
}



/**\fn void getQuaternion(float* Q, float mat[3][3])
 * \brief function to convert a rotation matrix to quanternion
 * \param Q - a float pointer
 * \param mat - a float 3x3 multidimension array
 * \return void
 */
void getQuaternion(float* Q, float mat[3][3])
{
    Q[_Qw] = sqrt( fmax( 0, 1 + mat[0][0] + mat[1][1] + mat[2][2] ) ) / 2;
    Q[_Qx] = sqrt( fmax( 0, 1 + mat[0][0] - mat[1][1] - mat[2][2] ) ) / 2;
    Q[_Qy] = sqrt( fmax( 0, 1 - mat[0][0] + mat[1][1] - mat[2][2] ) ) / 2;
    Q[_Qz] = sqrt( fmax( 0, 1 - mat[0][0] - mat[1][1] + mat[2][2] ) ) / 2;

    Q[_Qx] = copysignf( Q[1], mat[2][1] - mat[1][2] );
    Q[_Qy] = copysignf( Q[2], mat[0][2] - mat[2][0] );
    Q[_Qz] = copysignf( Q[3], mat[1][0] - mat[0][1] );
}


/**\fn void set_posd_to_pos(struct robot_device* device0)
 * \brief set the desired position to the robots current position
 * \param device0 a pointer points to the robot_device struct
 * \return void
 */
void set_posd_to_pos(struct robot_device* device0)
{
    for (int m = 0; m < NUM_MECH; m++) {
        device0->mech[m].pos_d.x     = device0->mech[m].pos.x;
        device0->mech[m].pos_d.y     = device0->mech[m].pos.y;
        device0->mech[m].pos_d.z     = device0->mech[m].pos.z;
        device0->mech[m].ori_d.yaw   = device0->mech[m].ori.yaw;
        device0->mech[m].ori_d.pitch = device0->mech[m].ori.pitch;
        device0->mech[m].ori_d.roll  = device0->mech[m].ori.roll;
        device0->mech[m].ori_d.grasp = device0->mech[m].ori.grasp;

        for (int k = 0; k < 3; k++)
          for (int j = 0; j < 3; j++)
              device0->mech[m].ori_d.R[k][j] = device0->mech[m].ori.R[k][j];


    }
}
