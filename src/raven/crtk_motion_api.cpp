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
 * crtk_motion_api.cpp
 *
 * \brief Class file for CRTK API state and status flags
 *
 *
 * \date Oct 18, 2018
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */
 
#include "crtk_motion_api.h"
#include "defines.h"
#include "local_io.h"
#include "update_device_state.h"

#include "rt_process_preempt.h"

CRTK_motion_api::CRTK_motion_api(){
  reset_cp_updated();
}

void CRTK_motion_api::crtk_servo_cr_cb(geometry_msgs::TransformStamped msg){

  // check length of incoming translation
  tf::Transform in_incr;
  tf::transformMsgToTF(msg.transform, in_incr);
  goal_cp[CRTK_servo].setOrigin(goal_cp[CRTK_servo].getOrigin()+in_incr.getOrigin()); // assume scale and rotation are correct

  set_cp_updated();
}

char CRTK_motion_api::set_cp_updated(){
  cp_updated = 1;
  return cp_updated;
}

char CRTK_motion_api::reset_cp_updated(){
  cp_updated = 0;
  return cp_updated;
}

char CRTK_motion_api::get_cp_updated(){
  return cp_updated;
}


tf::Transform CRTK_motion_api::get_goal_cp(){
  //TODO: figure this out later
  return goal_cp[CRTK_servo];
}

char CRTK_motion_api::check_updates(){
  return cp_updated; 
}

tf::Transform CRTK_motion_api::get_pos(){
  return pos;
}

// tf::Transform CRTK_motion_api::get_vel(){
//   return vel;
// }

void CRTK_motion_api::set_jpos(float* in){
  for(int i=0; i<7; i++){
    jpos[i] = in[i];
  }
}

// void CRTK_motion_api::set_jvel(float* in){

//   for(int i=0; i<7; i++){
//     jvel[i] = in[i];
//   }
// }

void CRTK_motion_api::set_setpoint_cp(tf::Transform* in){
  for(int i=0; i<3; i++){
    setpoint_cp[i]=tf::Transform(in[i]);
  }
}

void CRTK_motion_api::set_goal_cp(tf::Transform* in){
  for(int i=0; i<3; i++){
    goal_cp[i]=tf::Transform(in[i]);
  }
}


void CRTK_motion_api::copy_data(CRTK_motion_api* source){ // TODO: keep updating this
    pos = source->get_pos();
    //vel = source->get_vel();

    this->set_jpos(source->jpos);
    //this->set_jvel(source->jvel);

    this->set_setpoint_cp(source->setpoint_cp);
    this->set_goal_cp(source->goal_cp);
    cp_updated = source->get_cp_updated();

}