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

  // TODO: set goal_cp[3]
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

char CRTK_motion_api::set_pos(tf::Transform in){
  
  static int counter = 0;

  in.setRotation(in.getRotation().normalize());
  pos = in;
  counter ++;
  return 1;
}

// void CRTK_motion_api::set_jvel(float* in){

//   for(int i=0; i<7; i++){
//     jvel[i] = in[i];
//   }
// }

void CRTK_motion_api::set_setpoint_cp(tf::Transform in){
  in.setRotation(in.getRotation().normalize());
  setpoint_cp=tf::Transform(in);
}


void CRTK_motion_api::set_goal_cp(tf::Transform in ,CRTK_motion_type type){

  if(type > 4 || type < 0){
    ROS_ERROR("Invalid CRTK motion type.");
    return;
  }

  in.setRotation(in.getRotation().normalize());
  goal_cp[type]=tf::Transform(in);

}


void CRTK_motion_api::transfer_data(CRTK_motion_api* network_source){ // TODO: keep updating this
    char success = 0;
    success = success + preempt_to_network(network_source);
    success = success + network_to_preempt(network_source);

    if(success != 2){
      ROS_ERROR("CRTK data transfer failure.");
    }
    
    cp_updated = network_source->get_cp_updated();
    network_source->reset_cp_updated();
}

char CRTK_motion_api::preempt_to_network(CRTK_motion_api* network_source){
  // will be sending these:
  // pos, setpoint, jpos, goal(out), jvel, measured

  network_source->set_pos(this->get_pos());
  //vel = network_source->get_vel();

  network_source->set_jpos(this->jpos);
  //this->set_jvel(network_source->jvel);

  network_source->set_setpoint_cp(this->setpoint_cp);
  network_source->set_goal_cp(this->goal_cp[3], CRTK_out);

  
  return 1; // if all goes well
}

char CRTK_motion_api::network_to_preempt(CRTK_motion_api* network_source){
  // will be sending these:
  // goals(0-2)
   for(int i=0; i<3; i++){
    this->set_goal_cp(network_source->goal_cp[i], (CRTK_motion_type)i);
  } 

  return 1; // if all goes well
}