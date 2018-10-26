/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2018  Andrew Lewis, Yun-Hsuan Su, Blake Hannaford, 
 * and the University of Washington BioRobotics Laboratory
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
 * crtk_motion_api.h
 *
 * \brief Class file for CRTK state object, which holds all of the state
 *  flags for the "Robot Operating State" aspect of the CRTK API
 *
 *  \date Oct 25, 2018
 *  \author Andrew Lewis, Yun-Hsuan Su

 */

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>

#ifndef CRTK_MOTION_API_H_
#define CRTK_MOTION_API_H_

enum CRTK_input {CRTK_servo, CRTK_interp, CRTK_move, CRTK_out};

class CRTK_motion_api 
{
  public:
    CRTK_motion_api();
    ~CRTK_motion_api(){};
    void crtk_servo_cr_cb(geometry_msgs::TransformStamped);
    char reset_cp_updated();
    char get_cp_updated();
    tf::Transform get_goal_cp();

  private:
    // current robot pose
    tf::Transform pos;
    tf::Transform vel;
    float jpos[7];
    float jvel[7];

    tf::Transform setpoint_cp[3];
    tf::Transform goal_cp[3];
    char cp_updated;

    char set_cp_updated();
};


#endif