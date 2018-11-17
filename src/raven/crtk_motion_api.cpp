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


tf::Transform T_crtk_in_r0_gold = tf::Transform(tf::Matrix3x3( 0,   -1,  0,
                                                               0,    0, -1,
                                                               1,    0,  0), tf::Vector3(0,0,0));
tf::Transform T_crtk_in_r0_green = tf::Transform(tf::Matrix3x3(0,    1,  0,
                                                               0,    0,  1,
                                                               1,    0,  0), tf::Vector3(0,0,0));



CRTK_motion_api::CRTK_motion_api(){
  reset_goal_in();
  reset_setpoint_in();
  reset_goal_out();
  reset_setpoint_out();
}

CRTK_motion_api::CRTK_motion_api(char arm){
  reset_goal_in();
  reset_setpoint_in();
  reset_goal_out();
  reset_setpoint_out();
  if(arm == 0)
    set_base_frame(T_crtk_in_r0_gold);
  else if(arm == 1)
    set_base_frame(T_crtk_in_r0_green);
  else
    ROS_ERROR("Invalid arm type API constructor.");
}



char is_tf_type(CRTK_motion_type type){
  if(type == CRTK_cr || type == CRTK_cp || type == CRTK_cv || type == CRTK_cf){
    return 1;
  }
  else{
    return 0;
  }
}

char is_js_type(CRTK_motion_type type){
  if(type == CRTK_jr || type == CRTK_jp || type == CRTK_jv || type == CRTK_jf){
    return 1;
  }
  else{
    return 0;
  }
}

char is_type_valid(CRTK_motion_level level, CRTK_motion_type type){
  if(level == CRTK_move){
    if(type == CRTK_jf || type == CRTK_cf || type == CRTK_jv || type == CRTK_cv){
      ROS_ERROR("Invalid CRTK motion type. (level:%i, type:%i)",(int)level, (int)type);
      return 0;
    }
  }
  else if((level == CRTK_interp && type == CRTK_cf) || (level == CRTK_servo && type == CRTK_cf)){
    ROS_ERROR("Unsupported CRTK motion type. (level:%i, type:%i)",(int)level, (int)type);
    return 0;
  }
  return 1;
}


void CRTK_motion_api::crtk_servo_cr_cb(geometry_msgs::TransformStamped msg){

  // check length of incoming translation
  tf::Transform in_incr;
  tf::transformMsgToTF(msg.transform, in_incr);
  set_setpoint_in(CRTK_servo, CRTK_cr, in_incr);

  // tf::Transform out;
  // out = this->get_base_frame().inverse() * in_incr;  // TODO!!!
}


char CRTK_motion_api::check_updates(){
  // return 1 if any of the output flags aren't NULL
  
  return 0; 
}

tf::Transform CRTK_motion_api::get_base_frame(){
  return r0_transform;
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


float empty_out[7];


void CRTK_motion_api::set_base_frame(tf::Transform in){
  r0_transform = tf::Transform(in);
  return;
}

void CRTK_motion_api::set_default_base_frame(char arm){
  if(arm == 0)
    set_base_frame(T_crtk_in_r0_gold);
  else if(arm == 1)
    set_base_frame(T_crtk_in_r0_green);
  else
    ROS_ERROR("Invalid arm type default.");
}

void CRTK_motion_api::transfer_data(CRTK_motion_api* network_source){ // TODO: keep updating this
    char success = 0;
    success = success + preempt_to_network(network_source);
    success = success + network_to_preempt(network_source);

    if(success != 2){
      ROS_ERROR("CRTK data transfer failure.");
    }
}

char CRTK_motion_api::preempt_to_network(CRTK_motion_api* network_source){
  // copy goal_out and setpoint_out and measured...

  network_source->set_pos(this->get_pos());
  //vel = network_source->get_vel();

  network_source->set_jpos(this->jpos);
  //this->set_jvel(network_source->jvel);

  if(is_tf_type(this->setpoint_out_type)){
    network_source->set_setpoint_out_tf(this->setpoint_out_level, this->setpoint_out_type, this->setpoint_out_tf);
  }
  else if(is_js_type(this->setpoint_out_type)){
    network_source->set_setpoint_out_js(&this->setpoint_out_js);
  }


  if(is_tf_type(this->goal_out_type)){
    network_source->set_goal_out_tf(this->goal_out_level, this->goal_out_type, this->goal_out_tf);
  }
  else if(is_js_type(this->goal_out_type)){
    network_source->set_goal_out_js(&this->goal_out_js);
  }  

  return 1; // if all goes well
}

char CRTK_motion_api::network_to_preempt(CRTK_motion_api* network_source){
  // copy goal_in and setpoint_in

  for(int i=CRTK_interp; i<=CRTK_move; i++){
    this->set_goal_in((CRTK_motion_level)i, (network_source->get_goal_in((CRTK_motion_level)i)));
  } 
  this->set_setpoint_in(CRTK_servo, (network_source->get_setpoint_in(CRTK_servo)));

  network_source->reset_goal_in();
  network_source->reset_setpoint_in();
  return 1; // if all goes well
}


void CRTK_motion_api::reset_goal_in(){

  for(int i=0; i<2; i++){ // for interpolation, move
    goal_in[i].updated = 0;
    for(int j=0; j<7; i++)
    {
      goal_in[i].jr[j] = 0;
      goal_in[i].jp[j] = 0;
      goal_in[i].jv[j] = 0;
      goal_in[i].jf[j] = 0;
 
    }
    goal_in[i].cr = tf::Transform();
    goal_in[i].cp = tf::Transform();
    goal_in[i].cv = tf::Transform();
    goal_in[i].cf = tf::Transform();
  }  
}

void CRTK_motion_api::reset_setpoint_in(){

  for(int i=0; i<3; i++){ // for interpolation, move, servo
    setpoint_in[i].updated = 0;  
    for(int j=0; j<7; j++){
      setpoint_in[i].jr[j] = 0;
      setpoint_in[i].jp[j] = 0;
      setpoint_in[i].jv[j] = 0;
      setpoint_in[i].jf[j] = 0;
    }
    setpoint_in[i].cr = tf::Transform();
    setpoint_in[i].cp = tf::Transform();
    setpoint_in[i].cv = tf::Transform();
    setpoint_in[i].cf = tf::Transform();

  }
}

void CRTK_motion_api::reset_goal_out(){

  goal_out_js = sensor_msgs::JointState();
  goal_out_tf = tf::Transform();
  goal_out_level = CRTK_NULL_level;
  goal_out_type = CRTK_NULL_type;
}

void CRTK_motion_api::reset_setpoint_out(){

  setpoint_out_js = sensor_msgs::JointState();
  setpoint_out_tf = tf::Transform();
  setpoint_out_level = CRTK_NULL_level;
  setpoint_out_type = CRTK_NULL_type;
}


char CRTK_motion_api::set_setpoint_out_tf(CRTK_motion_level level, CRTK_motion_type type, tf::Transform in){
  if(is_tf_type(type)){
    setpoint_out_tf = tf::Transform(in);
    setpoint_out_level = level;
    setpoint_out_type = type;
    reset_setpoint_in();
    return 1;
  } 
  else{
    ROS_ERROR("Invalid CRTK_motion type for goal_out_tf.");
    return 0;
  }
}

char CRTK_motion_api::set_setpoint_out_js(CRTK_motion_level level, CRTK_motion_type type, float* in){
  if(is_js_type(type)){
    for(int i=0; i<7; i++){
      if(type == CRTK_jr)
        setpoint_out_js.position[i] = in[i];
      else if(type == CRTK_jp)
        setpoint_out_js.position[i] = in[i];
      else if(type == CRTK_jv)
        setpoint_out_js.velocity[i] = in[i];
      else if(type == CRTK_jf)
        setpoint_out_js.effort[i] = in[i];
    }
    setpoint_out_level = level;
    setpoint_out_type = type;
    reset_setpoint_in();
    return 1;
  }
  else{
    ROS_ERROR("Invalid CRTK_motion type for setpoint_out_js.");
    return 0;
  }
}

char CRTK_motion_api::set_goal_out_tf(CRTK_motion_level level, CRTK_motion_type type, tf::Transform in){
  if(is_tf_type(type)){
    goal_out_tf = tf::Transform(in);
    goal_out_level = level;
    goal_out_type = type;
    reset_goal_in();
    return 1;
  } 
  else{
    ROS_ERROR("Invalid CRTK_motion type for goal_out_tf.");
    return 0;
  }
}

char CRTK_motion_api::set_goal_out_js(CRTK_motion_level level, CRTK_motion_type type, float* in){
  if(is_js_type(type)){
    for(int i=0; i<7; i++){
      if(type == CRTK_jr)
        goal_out_js.position[i] = in[i];
      else if(type == CRTK_jp)
        goal_out_js.position[i] = in[i];
      else if(type == CRTK_jv)
        goal_out_js.velocity[i] = in[i];
      else if(type == CRTK_jf)
        goal_out_js.effort[i] = in[i];
    }
    goal_out_type = type;
    goal_out_level = level;
    reset_goal_in();
    return 1;
  }
  else{
    ROS_ERROR("Invalid CRTK_motion type for goal_out_js.");
    return 0;
  }
}

void CRTK_motion_api::set_setpoint_in(CRTK_motion_level level, CRTK_goal in){
  if(level == CRTK_servo || level == CRTK_interp || level == CRTK_move){
    setpoint_in[level] = in;
  }
  else{
    ROS_ERROR("Invalid CRTK_motion_level for set_setpoint_in().");
    return;
  } 
}

CRTK_goal CRTK_motion_api::get_setpoint_in(CRTK_motion_level level){
  if(level == CRTK_servo || level == CRTK_interp || level == CRTK_move){
    return setpoint_in[level];
  }
  else{
    ROS_ERROR("Invalid CRTK_motion_level for get_setpoint_in().");
  }
  CRTK_goal empty_out;
  return empty_out;
}

void CRTK_motion_api::set_setpoint_out_js(sensor_msgs::JointState* in){
  setpoint_out_js = *in;
}

void CRTK_motion_api::set_goal_out_js(sensor_msgs::JointState* in){
  goal_out_js = *in;
}

tf::Transform CRTK_motion_api::get_setpoint_out_tf(){
  return setpoint_out_tf;
}
sensor_msgs::JointState CRTK_motion_api::get_setpoint_out_js(){
  return setpoint_out_js;
}
tf::Transform CRTK_motion_api::get_goal_out_tf(){
  return goal_out_tf;
}
sensor_msgs::JointState CRTK_motion_api::get_goal_out_js(){
  return goal_out_js;
}

void CRTK_motion_api::set_goal_in(CRTK_motion_level level, CRTK_goal in){
  goal_in[level] = in;
}

void CRTK_motion_api::set_goal_in(CRTK_motion_level level, CRTK_motion_type type, tf::Transform in){
  if(level == CRTK_servo) {
    ROS_ERROR("CRTK_servo is not a goal type.");
    return;
  }
  if(is_tf_type(type) && is_type_valid(level,type)){
    if(type == CRTK_cr)       goal_in[level].cr = in;
    else if(type == CRTK_cp)  goal_in[level].cp = in;
    else if(type == CRTK_cv)  goal_in[level].cv = in;   
    goal_in[level].updated = 1;
  }
}
void CRTK_motion_api::set_goal_in(CRTK_motion_level level, CRTK_motion_type type, float* in){
  if(level == CRTK_servo) {
    ROS_ERROR("CRTK_servo is not a goal type.");
    return;
  }
  if(is_js_type(type) && is_type_valid(level,type)){
    for(int i=0; i<7; i++){
      if(type == CRTK_jr)       goal_in[level].jr[i] = in[i];
      else if(type == CRTK_jp)  goal_in[level].jp[i] = in[i];
      else if(type == CRTK_jv)  goal_in[level].jv[i] = in[i];  
      else                      goal_in[level].jf[i] = in[i];  
    }
    goal_in[level].updated = 1;
  }
}
void CRTK_motion_api::set_setpoint_in(CRTK_motion_level level, CRTK_motion_type type, tf::Transform in){
  if(is_tf_type(type) && is_type_valid(level,type)){
    if(type == CRTK_cr)       setpoint_in[level].cr = in;
    else if(type == CRTK_cp)  setpoint_in[level].cp = in;
    else if(type == CRTK_cv)  setpoint_in[level].cv = in;   
    setpoint_in[level].updated = 1;
  }
}
void CRTK_motion_api::set_setpoint_in(CRTK_motion_level level, CRTK_motion_type type, float* in){
  if(is_js_type(type) && is_type_valid(level,type)){
    for(int i=0; i<7; i++){
      if(type == CRTK_jr)       setpoint_in[level].jr[i] = in[i];
      else if(type == CRTK_jp)  setpoint_in[level].jp[i] = in[i];
      else if(type == CRTK_jv)  setpoint_in[level].jv[i] = in[i];  
      else                      setpoint_in[level].jf[i] = in[i]; 
    }
    setpoint_in[level].updated = 1;
  }
}

CRTK_goal CRTK_motion_api::get_goal_in(CRTK_motion_level level){
  if(level == CRTK_interp || level == CRTK_move)
    return goal_in[level];
  else{
    ROS_ERROR("Invalid CRTK_motion_level for get_goal_in().");
  }
  CRTK_goal empty_out;
  return empty_out;
}

tf::Transform CRTK_motion_api::get_goal_in_tf(CRTK_motion_level level, CRTK_motion_type type){
  if(level == CRTK_servo) {
    ROS_ERROR("CRTK_servo is not a goal type.");
  }
  if(is_tf_type(type) && is_type_valid(level,type)){
    if(type == CRTK_cr)       return goal_in[level].cr;
    else if(type == CRTK_cp)  return goal_in[level].cp;
    else if(type == CRTK_cv)  return goal_in[level].cv;  
  }
  return tf::Transform();
}
tf::Transform CRTK_motion_api::get_setpoint_in_tf(CRTK_motion_level level, CRTK_motion_type type){
  if(is_tf_type(type) && is_type_valid(level,type)){
    if(type == CRTK_cr)       return setpoint_in[level].cr;
    else if(type == CRTK_cp)  return setpoint_in[level].cp;
    else if(type == CRTK_cv)  return setpoint_in[level].cv;  
  }
  return tf::Transform();
}
float* CRTK_motion_api::get_goal_in_js(CRTK_motion_level level, CRTK_motion_type type){
  if(level == CRTK_servo) {
    ROS_ERROR("CRTK_servo is not a goal type.");
  }
  if(is_js_type(type) && is_type_valid(level,type)){
    if(type == CRTK_jr)       return goal_in[level].jr;
    else if(type == CRTK_jp)  return goal_in[level].jp;
    else if(type == CRTK_jv)  return goal_in[level].jv; 
    else                      return goal_in[level].jf; 
  }
  return empty_out;
}
float* CRTK_motion_api::get_setpoint_in_js(CRTK_motion_level level, CRTK_motion_type type){
  if(is_js_type(type) && is_type_valid(level,type)){
    if(type == CRTK_jr)       return setpoint_in[level].jr;
    else if(type == CRTK_jp)  return setpoint_in[level].jp;
    else if(type == CRTK_jv)  return setpoint_in[level].jv;  
    else                      return setpoint_in[level].jf;
  }
  return empty_out;
}


