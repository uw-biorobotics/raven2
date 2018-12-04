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
  crtk_motion_api[0] = CRTK_motion_api(0);
  crtk_motion_api[1] = CRTK_motion_api(1);
  crtk_motion_api_prev[0] = CRTK_motion_api(0);
  crtk_motion_api_prev[1] = CRTK_motion_api(1);

  crtk_motion_api_grasp[0] = CRTK_motion_api(0);
  crtk_motion_api_grasp[1] = CRTK_motion_api(1);
  crtk_motion_api_grasp_prev[0] = CRTK_motion_api(0);
  crtk_motion_api_grasp_prev[1] = CRTK_motion_api(1);
}

/**
 * @brief      Top level crtk motion function calls appropriate planners
 *
 * @param[in]  current_state  The current CRTK state of the robot
 *
 * @return     1 for success
 *             - 0 for no activity
 */
int CRTK_motion_planner::crtk_motion_state_machine(CRTK_robot_state current_state){

  if(current_state == CRTK_ENABLED){
    mid_level_controller();
    low_level_controller();
  }
  else if (current_state == CRTK_PAUSED) {
    // do something...? plan?
  }
  else {
    // do nothing EVER
  }

}

/**
 * @brief      determines appropriate servo commands to pass to the device
 *
 * @return     1 for success
 */
int CRTK_motion_planner::low_level_controller(){  
  int out = 0;
  for(int i=0;i<2;i++){
    // check for setpoint updates
    if(crtk_motion_api[i].check_setpoint_updates()){
      // set setpoint_out
      out = crtk_motion_api[i].set_setpoint_out() ;     
    }

    // check for setpoint updates
    if(crtk_motion_api_grasp[i].check_setpoint_updates()){
      // set setpoint_out
      out = crtk_motion_api_grasp[i].set_setpoint_out() ;     
    }

  }
  return out;
}

/**
 * @brief      Coordinates Move and Interp commands and calls appropriate
 *             planners
 *
 * @return     1 for success
 */
int CRTK_motion_planner::mid_level_controller(){
  int out = 0;
  for(int i=0;i<2;i++){
    // check for goal updates
    if(crtk_motion_api[i].check_goal_updates()){
      // set goal_out
      out = crtk_motion_api[i].set_goal_out();     
    }

  }
  return out;
}