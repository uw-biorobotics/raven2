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
 * crtk_motion_planner.h
 *
 * \brief Class file for CRTK state object, which holds all of the state
 *  flags for the "Robot Operating State" aspect of the CRTK API
 *
 *  \date Oct 26, 2018
 *  \author Andrew Lewis, Yun-Hsuan Su

 */
#ifndef CRTK_MOTION_PLANNER_H_
#define CRTK_MOTION_PLANNER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include "crtk_motion_api.h"
#include "crtk_state.h"



class CRTK_motion_planner 
{
  public:
    CRTK_motion_planner();
    ~CRTK_motion_planner(){};

    CRTK_motion_api crtk_motion_api[2];
    CRTK_motion_api crtk_motion_api_prev[2];
    
    CRTK_motion_api crtk_motion_api_grasp[2];
    CRTK_motion_api crtk_motion_api_grasp_prev[2];

    int crtk_motion_state_machine(CRTK_robot_state);
    int mid_level_controller();
    int low_level_controller();
  private:

};


#endif