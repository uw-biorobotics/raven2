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
 * crtk_motion_planner.cpp
 *
 * \brief Class file for CRTK API state and status flags
 *
 *
 * \date Oct 26, 2018
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */
#include "crtk_motion_planner.h"

CRTK_motion_planner::CRTK_motion_planner(){

}

int CRTK_motion_planner::crtk_motion_state_machine(CRTK_robot_state current_state){

  static int count=0;
  float desiredl[3];
  float desiredr[3];
  if(count%500 == 0){
      desiredl[0] = crtk_motion_api[0].get_goal_cp().getOrigin().x();
      desiredr[0] = crtk_motion_api[1].get_goal_cp().getOrigin().x();
      desiredl[1] = crtk_motion_api[0].get_goal_cp().getOrigin().y();
      desiredr[1] = crtk_motion_api[1].get_goal_cp().getOrigin().y();
      desiredl[2] = crtk_motion_api[0].get_goal_cp().getOrigin().z();
      desiredr[2] = crtk_motion_api[1].get_goal_cp().getOrigin().z();
    // ROS_INFO("Robot arm pos desired = %f\t%f\t%f \t\t %f\t%f\t%f", 
      // desiredl[0],desiredl[1],desiredl[2], desiredr[0],desiredr[1],desiredr[2]);
  }
  count ++;

  if(current_state == CRTK_ENABLED){

  }
  else if (current_state == CRTK_PAUSED) {
    // do something...? plan?
  }
  else {
    // do nothing
  }

}