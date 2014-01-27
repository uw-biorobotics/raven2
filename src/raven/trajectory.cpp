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
*    File: trajectory.h
*    Created by Hawkeye 10/2011
*
*    Generate joint and cartesian trajectories.
*    Internal datastructures track trajectory state, and update DOFs as needed upon calling.
*/

#include <ros/ros.h>

#include "trajectory.h"
#include "log.h"
#include "utils.h"
#include "defines.h"

extern unsigned long int gTime;

// Store trajectory parameters
struct _trajectory
{
    ros::Time startTime;
    float end_pos;
    float magnitude;
    float period;
    float startPos;
    float startVel;
};
struct _trajectory trajectory[MAX_MECH*MAX_DOF_PER_MECH];


/**
*  start_velocity_trajectory()
*    initialize trajectory parameters
*/
int start_trajectory(struct DOF* _joint, float _endPos, float _period)
{
    trajectory[_joint->type].startTime = trajectory[_joint->type].startTime.now();
    trajectory[_joint->type].startPos = _joint->jpos;
    trajectory[_joint->type].startVel = _joint->jvel;
    _joint->jpos_d = _joint->jpos;
    _joint->jvel_d = _joint->jvel;

    trajectory[_joint->type].magnitude = _endPos - _joint->jpos;
    trajectory[_joint->type].period = _period;
//    log_msg("starting trajectory on joint %d to magnitude: %0.3f (%0.3f - %0.3f), period:%0.3f",
//        _joint->type,
//        trajectory[_joint->type].magnitude,
//        _endPos, _joint->jpos,
//        trajectory[_joint->type].period);
    return 0;
}
/**
*  start_velocity_trajectory()
*    initialize trajectory parameters
*/
int start_trajectory_mag(struct DOF* _joint, float _mag, float _period)
{
    trajectory[_joint->type].startTime = trajectory[_joint->type].startTime.now();
    trajectory[_joint->type].startPos = _joint->jpos;
    trajectory[_joint->type].startVel = _joint->jvel;
    _joint->jpos_d = _joint->jpos;
    _joint->jvel_d = _joint->jvel;

    trajectory[_joint->type].magnitude = _mag;
    trajectory[_joint->type].period = _period;
    return 0;
}

/**
*   stop_velocity_trajectory()
*      Set jvel zero
*      Zero torque
*/
int stop_trajectory(struct DOF* _joint)
{
    trajectory[_joint->type].startTime = trajectory[_joint->type].startTime.now();
    trajectory[_joint->type].startPos = _joint->jpos;
    trajectory[_joint->type].startVel = 0;
    _joint->jpos_d = _joint->jpos;
    _joint->jvel_d = 0;
    _joint->tau_d = 0;
    _joint->current_cmd = 0;

    return 0;
}

/**
*  update_sinusoid_trajectory()
*        find next trajectory waypoint
*        Sinusoid trajcetory
*/
int update_sinusoid_velocity_trajectory(struct DOF* _joint)
{
    const float maxspeed = 15 DEG2RAD;
    const float f_period = 2000;         // 2 sec

    ros::Duration t = ros::Time::now() - trajectory[_joint->type].startTime;

   if (_joint->type      == SHOULDER_GOLD)
        _joint->jvel_d = -1 * maxspeed * sin( 2*M_PI * (1/f_period) * t.toSec());

//    else if (_joint->type == ELBOW_GOLD)
//        _joint->jvel_d =  maxspeed * sin( 2*M_PI * (1/f_period) * t);
//
//    else if (_joint->type == Z_INS_GOLD)
//        _joint->jvel_d =  0.2 * sin( 2*M_PI * (1/f_period) * t);

    else
        _joint->jvel_d = 0;

    return 0;
}

/**
*
*  update_linear_sinusoid_trajectory()
*     find next trajectory waypoint.
*     Sinusoid ramp up and linear velocity after peak.
*/
int update_linear_sinusoid_velocity_trajectory(struct DOF* _joint)
{
    const float maxspeed[8] = {-4 DEG2RAD, 4 DEG2RAD, 0.02, 15 DEG2RAD};
    const float f_period = 2;         // 2 sec

    ros::Duration t = ros::Time::now() - trajectory[_joint->type].startTime;

    // Sinusoid portion complete.  Return without changing velocity.
    if (t.toSec() >= f_period/2)
        return 1;

    if (_joint->type      == SHOULDER_GOLD)
        _joint->jvel_d = maxspeed[0] * (1-cos( 2*M_PI * (1/f_period) * t.toSec()));

    else if (_joint->type == ELBOW_GOLD)
        _joint->jvel_d = maxspeed[1] * (1-cos( 2*M_PI * (1/f_period) * t.toSec()));

    else if (_joint->type == Z_INS_GOLD)
        _joint->jvel_d = maxspeed[2] * (1-cos( 2*M_PI * (1/f_period) * t.toSec()));

    else
        _joint->jvel_d = 0;

    return 0;
}

/**
*  update_sinusoid_position_trajectory()
*     find next trajectory waypoint.
*     Sinusoidal position trajectory
*/
int update_sinusoid_position_trajectory(struct DOF* _joint)
{
    struct _trajectory* traj = &(trajectory[_joint->type]);
    float f_magnitude = traj->magnitude;
    float f_period    = traj->period;

    ros::Duration t = ros::Time::now() - traj->startTime;

    // Rising sinusoid
    if ( t.toSec() < f_period/4 )
        _joint->jpos_d = -f_magnitude * 0.5 * (1-cos( 4 * M_PI * t.toSec() / f_period )) + traj->startPos;
    else
        _joint->jpos_d = -f_magnitude * sin( 2*M_PI * t.toSec() / f_period) + traj->startPos;

    return 0;
}

/**
*  update_sinusoid_position_trajectory()
*     find next trajectory waypoint.
*     Sinusoidal position trajectory
*/
int update_linear_sinusoid_position_trajectory(struct DOF* _joint)
{
//    const float f_magnitude[8] = {-10 DEG2RAD, 10 DEG2RAD, 0.01, 0, 60 DEG2RAD, 60 DEG2RAD, 60 DEG2RAD, 60 DEG2RAD};
//    const float f_period[8] = {7000, 3200, 7000, 0000, 5000, 5000, 5000, 5000};
    struct _trajectory* traj = &(trajectory[_joint->type]);

    ros::Duration t = ros::Time::now() - traj->startTime;

    if ( t.toSec() < traj->period/2 )
//        _joint->jpos_d += ONE_MS * f_magnitude[index] * (1-cos( 2*M_PI * (1/f_period[index]) * t.toSec()));
        _joint->jpos_d += ONE_MS * traj->magnitude * (1-cos( 2*M_PI * (1/traj->period) * t.toSec()));
    else
        _joint->jpos_d += ONE_MS * traj->magnitude;

    return 0;
}


/**
*  update_sinusoid_position_trajectory()
*     find next trajectory waypoint.
*     Sinusoidal position trajectory
*/
int update_position_trajectory(struct DOF* _joint)
{
    struct _trajectory* traj = &(trajectory[_joint->type]);
    float magnitude = traj->magnitude;
    float period  = traj->period;

    ros::Duration t = ros::Time::now()- traj->startTime;

    if ( t.toSec() < period ){
        _joint->jpos_d = 0.5*magnitude * (1-cos( 2*M_PI * (1/(2*period)) * t.toSec())) + traj->startPos;
        return 1;
    }

    return 0;
}


