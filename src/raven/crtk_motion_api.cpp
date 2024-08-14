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

  r0_transform = tf::Transform();
  new_base_frame_flag = 0;
  pos = tf::Transform(); // measured_cp

  goal_out_js = sensor_msgs::JointState();
  setpoint_out_js = sensor_msgs::JointState();

  for(int i= 0; i<7; i++) 
  {
    jpos[i] = 0;
    goal_out_js.position.push_back(0);
    goal_out_js.velocity.push_back(0);
    goal_out_js.effort.push_back(0);
    setpoint_out_js.position.push_back(0);
    setpoint_out_js.velocity.push_back(0);
    setpoint_out_js.effort.push_back(0);
  }

  goal_out_tf = tf::Transform ();
  setpoint_out_tf = tf::Transform ();
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
  else if(arm == 2) 
    set_base_frame(T_crtk_in_r0_gold);
  else if(arm == 3)
    set_base_frame(T_crtk_in_r0_green);
  else
    ROS_ERROR("Invalid arm type API constructor.");
  pos = tf::Transform(); // measured_cp

  new_base_frame_flag = 1;

  goal_out_js = sensor_msgs::JointState();
  setpoint_out_js = sensor_msgs::JointState();

  for(int i= 0; i<7; i++) 
  {
    jpos[i] = 0;
    goal_out_js.position.push_back(0);
    goal_out_js.velocity.push_back(0);
    goal_out_js.effort.push_back(0);
    setpoint_out_js.position.push_back(0);
    setpoint_out_js.velocity.push_back(0);
    setpoint_out_js.effort.push_back(0);
  }

  goal_out_tf = tf::Transform ();
  setpoint_out_tf = tf::Transform();
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


/**
 * @brief      callback function for relative cartesian servo commands
 *
 * @param[in]  msg   The message from ROS
 */
void CRTK_motion_api::crtk_servo_cr_cb(geometry_msgs::PoseStamped msg){

  static int count = 0; // check length of incoming translation
  // if(count%250 == 0){
  //   ROS_INFO("crtk_servo_cr_cb count %i",count);
  // }
  count ++;

  // get message and convert from pose to transform
  tf::Pose in_pose;
  tf::poseMsgToTF(msg.pose, in_pose);

  tf::Transform in_incr;
  pose_to_tf(in_pose, in_incr);

  if(get_setpoint_update_flag(CRTK_servo, CRTK_cr)){
    in_incr.setOrigin(  get_setpoint_in(CRTK_servo).cr.getOrigin()   + in_incr.getOrigin());
    if(!isnan(in_incr.getRotation().length()))
      in_incr.setRotation(get_setpoint_in(CRTK_servo).cr.getRotation() * in_incr.getRotation());
    // else
      //ROS_INFO("Getting servo_cr nan rotation command!");
  }
  set_setpoint_in(CRTK_servo, CRTK_cr, in_incr);
  // if(count%250 == 0)
  //   ROS_INFO("servo cr cmd of %f ", in_incr.getRotation().getAngle());

}


/**
 * @brief      callback function for cartesian velocity servo commands
 *
 * @param[in]  msg   The message from ROS
 */
void CRTK_motion_api::crtk_servo_cv_cb(geometry_msgs::TwistStamped msg){

  // get command and convert from pose to transform

  tf::Transform vel;
  tf::Quaternion in_rot;
  in_rot.setEuler(msg.twist.angular.y, msg.twist.angular.x, msg.twist.angular.z);

  vel.setOrigin(tf::Vector3(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z));
  vel.setRotation(in_rot);

  set_setpoint_in(CRTK_servo, CRTK_cv, vel);
}


/**
 * @brief      callback function for absolute cartesian servo commands
 * 
 * don't 
 * 
 * currently computes the increment from the previous command, sorry
 *
 * @param[in]  msg   The message from ROS
 */
void CRTK_motion_api::crtk_servo_cp_cb(geometry_msgs::PoseStamped msg){


  static int count = 0;
  count++;
  const float max_step = 0.015; // maximum step size per ctrl loop in m
  const float max_rot = 0.01 DEG2RAD; //maximum rotation step in degrees (to radians)
  float cmd_step, cmd_rot;
  char thresh_pos, thresh_rot;
  tf::Vector3 thresh_vec;

  //get command and convert from pose to transform
  tf::Pose in_pose;
  tf::poseMsgToTF(msg.pose, in_pose);

  tf::Transform in_tf, curr_tf;
  pose_to_tf(in_pose, in_tf);

  tf::Quaternion in_rot, curr_rot, diff_rot;
  tf::Vector3 in_pos, curr_pos;

  in_pos = in_tf.getOrigin();
  in_rot = in_tf.getRotation();

  curr_tf = get_pos();
  curr_pos = curr_tf.getOrigin();
  curr_rot = curr_tf.getRotation();

  //check if it's in workspace range
  if (!pos_in_workspace(in_pos)){
    ROS_ERROR("servo cp out of workspace - check yo self before you wreck yo robot");
    return;
  }

  //check if it's close enough to current position
  cmd_step = curr_pos.distance(in_pos); //always positive
  if(cmd_step >= max_step){
    if(cmd_step > 5 * max_step){
      ROS_ERROR("cmd_step = %f (too large)",cmd_step);
      return;
    } 
    else {
      // threshold direction
      thresh_pos = 1;
      thresh_vec = (in_pos - curr_pos).normalize(); //is this the correct direction?
      in_pos = curr_pos + max_step * thresh_vec;

      static int thresh_count = 0;
      thresh_count++;
      if (thresh_count % 500 == 0){
        ROS_INFO("Thresholded 500 of these here servo_cp positions for you, boss");
        thresh_count = 0;
      }

    }

  }

  // check if it's close enough to current position
  diff_rot = curr_rot * in_rot.inverse();

  cmd_rot = curr_rot.angleShortestPath(in_rot);
  if(cmd_rot != diff_rot.getAngle())
    thresh_vec = -diff_rot.getAxis();
  else
    thresh_vec = diff_rot.getAxis();

  
  if(false){//cmd_rot > max_rot){
    if(cmd_rot > 5 * max_rot){
      //ROS_ERROR("rot_step = %f (too large)",cmd_rot);
      return;
    } 
    else {
      // threshold direction
      thresh_rot = 1;
      //in_rot = tf::Quaternion(thresh_vec,max_rot);
      static int foo = 0;
      if (foo % 500 == 0){
        ROS_ERROR("We've had it up to here with these big ol' rotations!");
        foo = 0;
      }
    }
  } 
 

  // set goal
  tf::Transform out_tf = tf::Transform(in_rot,in_pos);

  // ROS_INFO("callback (shortest cmd_rot, cmd_rot) = (%f, %f)", in_rot.angleShortestPath(curr_rot),in_rot.angle(curr_rot)*2);

  set_setpoint_in(CRTK_servo, CRTK_cp, out_tf);

  // party
  return;
}

/**
 * @brief      callback function for relative joint servo commands
 *
 * @param[in]  msg   The message from ROS
 */
void CRTK_motion_api::crtk_servo_jr_cb(sensor_msgs::JointState in){

  static int count = 2;
  count++;

  if (in.position.size() >= 7){
    for(int i=0;i<7;i++){
      if(get_setpoint_update_flag(CRTK_servo, CRTK_jr))
        setpoint_in[CRTK_servo].jr[i] = in.position[i] + get_setpoint_in(CRTK_servo).jr[i];
      else
        setpoint_in[CRTK_servo].jr[i] = in.position[i];
    }
  }
  else if (in.position.size() == 1){
    for(int i=1;i<7;i++){
      if(get_setpoint_update_flag(CRTK_servo, CRTK_jr)){
        setpoint_in[CRTK_servo].jr[0] = in.position[0] + get_setpoint_in(CRTK_servo).jr[0];
        setpoint_in[CRTK_servo].jr[i] = 0 + get_setpoint_in(CRTK_servo).jr[i];
      }
      else{
        setpoint_in[CRTK_servo].jr[0] = in.position[0];
        setpoint_in[CRTK_servo].jr[i] = 0;
      }
    }
  }
  else{
    ROS_INFO("unusual # of inputs to servo_jr.");
    return;
  }

  setpoint_in[CRTK_servo].update_flags[CRTK_jr] = 1;
  setpoint_in[CRTK_servo].updated = 1;
}

/**
 * @brief      callback function for joint velocity servo commands
 *
 * @param[in]  msg   The message from ROS
 */
void CRTK_motion_api::crtk_servo_jv_cb(sensor_msgs::JointState in){

  static int count = 2;
  count++;

  if (in.velocity.size() >= 7){
    for(int i=0;i<7;i++){
      setpoint_in[CRTK_servo].jv[i] = in.velocity[i];
    }
  }
  else if (in.velocity.size() == 1){
    for(int i=1;i<7;i++){
      setpoint_in[CRTK_servo].jv[0] = in.velocity[0];
      setpoint_in[CRTK_servo].jv[i] = 0;
    }
  }
  else{
    ROS_INFO("unusual # of inputs to servo_jv.");
    return;
  }

  setpoint_in[CRTK_servo].update_flags[CRTK_jv] = 1;
  setpoint_in[CRTK_servo].updated = 1;
}


/**
 * @brief      callback function for absolute joint servo commands
 * 
 * treated as incremental for the moment
 *
 * @param[in]  msg   The message from ROS
 */
void CRTK_motion_api::crtk_servo_jp_cb(sensor_msgs::JointState in){

  static int count = 0;
  count++;

  if (in.position.size() == 7){
    for(int i=0;i<7;i++){
      setpoint_in[CRTK_servo].jp[i] = in.position[i];

    }
  }
  else if (in.position.size() == 1){
    setpoint_in[CRTK_servo].jp[0] = in.position[0];

    for(int i=1;i<7;i++){
      setpoint_in[CRTK_servo].jp[i] = 0;
    }
  }
  else{
    ROS_INFO("unusual # of inputs to servo_jp!?!?!");
  }


  setpoint_in[CRTK_servo].update_flags[CRTK_jp] = 1;
  setpoint_in[CRTK_servo].updated = 1;

  // if(count %500 == 0){
  //   ROS_INFO("heard 500 grasp things!!!!! omg %f", setpoint_in[CRTK_servo].jp[0]);
  // }
}


/**
 * @brief      Looks over all incoming motion goal commands and returns top priority
 *             level and type
 *
 * @param      level  pointer to the enumerated command level
 * @param      type   pointer to the enumerated motion type
 *
 * @return     1 if new command
 *             - 0 if no new command
 */
char CRTK_motion_api::check_goal_updates(){
  // return 1 if any of the output flags aren't NULL
  goal_out_level = CRTK_NULL_level;
  goal_out_type = CRTK_NULL_type;
  for(int i = CRTK_move; i <= CRTK_interp; i++){
    if(goal_in[i].updated){
      for(int j = CRTK_jr; j < CRTK_NULL_type; j++){
        if(goal_in[i].update_flags[j] == true){
          goal_out_level = (CRTK_motion_level)i;
          goal_out_type = (CRTK_motion_type)j;
          return 1;          
        }

      }   
    }
  }
  return 0; 

}


/**
 * @brief      Looks over all incoming motion setpoint commands and sets top
 *             priority level and type
 *
 * @return     1 if new command
 *             - 0 if no new command
 */
char CRTK_motion_api::check_setpoint_updates(){
  // return 1 if any of the output flags aren't NULL
  setpoint_out_level = CRTK_NULL_level;
  setpoint_out_type = CRTK_NULL_type;
  for(int i = CRTK_move; i <= CRTK_servo; i++){
    if(setpoint_in[i].updated){
      for(int j = CRTK_jr; j < CRTK_NULL_type; j++){
        if(setpoint_in[i].update_flags[j] == true){
          setpoint_out_level = (CRTK_motion_level)i;
          setpoint_out_type = (CRTK_motion_type)j;
          return 1;          
        }

      }   
    }
  }

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


float empty_out[7]= {0,0,0,0,0,0,0};


void CRTK_motion_api::set_base_frame(tf::Transform in){
  r0_transform = tf::Transform(in);
  return;
}

void CRTK_motion_api::set_default_base_frame(char arm){
  if(arm == 0)
    set_base_frame(T_crtk_in_r0_gold);
  else if(arm == 1)
    set_base_frame(T_crtk_in_r0_green);
  else if(arm == 2)
    set_base_frame(T_crtk_in_r0_gold);
  else if(arm == 3)
    set_base_frame(T_crtk_in_r0_green);
  else
    ROS_ERROR("Invalid arm type default.");
}


char CRTK_motion_api::transfer_data(CRTK_motion_api* network_source){ // TODO: keep updating this
    char success = 0;
    success += preempt_to_network(network_source);
    success += network_to_preempt(network_source);

    // if(success != 2){
    //   ROS_ERROR("CRTK data transfer failure.");
    // }

    return success;
}

/**
 * @brief      Transfers CRTK motion api data from network thread objects to device thread objects 
 * 
 * @details    Transfers outgoing current pos, jpos, vel, jvel, force, jforce
 *             (goal_out, setpoint_out)*(js, tf, level, type)
 *             Forces and velocities not implemented
 *
 * @param      network_source  The network source
 *
 * @return     success
 */
char CRTK_motion_api::preempt_to_network(CRTK_motion_api* network_source){

  // cartesian pos & velocity
  network_source->set_pos(get_pos());
  //network_source->set_vel(get_vel());

  // joint position and velocity
  network_source->set_jpos(jpos);
  //network_source->set_jvel(jvel);

  // force and joint force
  //network_source->set_force(force);
  //network_source->set_jforce(jforce);
  //
  
  //goal out (tf, js, level, type)
  if(is_tf_type(goal_out_type)){
    network_source->set_goal_out_tf(goal_out_level, goal_out_type, goal_out_tf);
  }
  else if(is_js_type(goal_out_type)){
    network_source->copy_goal_out_js(&goal_out_js);
  }  

  //setpoint out (tf, js, level, type)
  if(is_tf_type(setpoint_out_type)){
    network_source->set_setpoint_out_tf(setpoint_out_level, setpoint_out_type, setpoint_out_tf);
  }

  else if(is_js_type(setpoint_out_type)){
    network_source->copy_setpoint_out_js(&setpoint_out_js);
  }

  if(setpoint_out_type == CRTK_NULL_type)
    return 0;

  return 1; // if all goes well
}

/**
 * @brief      Transfers CRTK motion api data from device thread objects to network thread objects. 
 *
 * @details    transfers base transform, goal_in[2] and setpoint_in[servo] without performing any
 *             calculations on incoming commands.
 *
 * @param      network_source  The network source
 *
 * @return     success
 */
char CRTK_motion_api::network_to_preempt(CRTK_motion_api* network_source){
  // transfer base frame from user
  if(network_source->get_new_base_frame_flag()){
    set_base_frame(network_source->get_base_frame());
    network_source->set_new_base_frame_flag(0);
  }
  

  // copy goal_in(move and interp) and setpoint_in(just servo)
  for(int i=CRTK_move; i<=CRTK_interp; i++){
    set_goal_in((CRTK_motion_level)i, (network_source->get_goal_in((CRTK_motion_level)i)));
  } 
  set_setpoint_in(CRTK_servo, (network_source->get_setpoint_in(CRTK_servo)));



  //reset network data
  network_source->reset_goal_in();
  network_source->reset_setpoint_in();
  return 1; // if all goes well
}


void CRTK_motion_api::reset_goal_in(){

  for(int i=0; i<2; i++){ // for interpolation, move
    goal_in[i].updated = 0;
    for(int k=0; k<8;k++){
      goal_in[i].update_flags[k] = false;
    }
    for(int j=0; j<7; j++)
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
    for(int k=0; k<8;k++){
      setpoint_in[i].update_flags[k] = false;
    }
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

  for(int i= 0; i<7; i++) 
  {
    goal_out_js.position.push_back(0);
    goal_out_js.velocity.push_back(0);
    goal_out_js.effort.push_back(0);
  }
}

void CRTK_motion_api::reset_setpoint_out(){

  setpoint_out_js = sensor_msgs::JointState();
  setpoint_out_tf = tf::Transform();
  setpoint_out_level = CRTK_NULL_level;
  setpoint_out_type = CRTK_NULL_type;

  for(int i= 0; i<7; i++) 
  {
    setpoint_out_js.position.push_back(0);
    setpoint_out_js.velocity.push_back(0);
    setpoint_out_js.effort.push_back(0);
  }

}

/**
 * @brief      Sets the setpoint out based on the level and type flags stored in the object.
 *
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion_api::set_setpoint_out(){
  if(is_tf_type(setpoint_out_type)){
    set_setpoint_out_tf(setpoint_out_level,setpoint_out_type,get_setpoint_in_tf(setpoint_out_level,setpoint_out_type));
    return 1;
  }
  else if(is_js_type(setpoint_out_type)){
   
    float out[7];
    get_setpoint_in_js(setpoint_out_level,setpoint_out_type,out);
    set_setpoint_out_js(setpoint_out_level,setpoint_out_type,out);
    return 1;
  }
  ROS_ERROR("Set setpoint_out_error!");
  return 0;
}


char CRTK_motion_api::set_setpoint_out_tf(CRTK_motion_level level, CRTK_motion_type type, tf::Transform in){
//TODO remove level and type from params 
 if(is_tf_type(type)){
    setpoint_out_tf = in;
    setpoint_out_level = level;
    setpoint_out_type = type;
    reset_setpoint_in();
    // ROS_INFO("setpoint out: level=%i,type=%i (rotation: %f,%f,%f,%f)",setpoint_out_level,setpoint_out_type,setpoint_out_tf.getRotation().x(),setpoint_out_tf.getRotation().y(),setpoint_out_tf.getRotation().z(),setpoint_out_tf.getRotation().w());
    return 1;
  } 
  else{
    ROS_ERROR("Invalid CRTK_motion type for goal_out_tf.");
    return 0;
  }
}

char CRTK_motion_api::set_setpoint_out_js(CRTK_motion_level level, CRTK_motion_type type, float* in){
//TODO remove level and type from params 
   
  if(is_js_type(type)){
    for(int i=0; i<7; i++){
      if(type == CRTK_jr || type == CRTK_jp){
        setpoint_out_js.position[i] = in[i];
        setpoint_out_js.velocity[i] = 0;
        setpoint_out_js.effort[i] = 0;
      }
      else if(type == CRTK_jv){
        setpoint_out_js.velocity[i] = in[i];
        setpoint_out_js.position[i] = 0;
        setpoint_out_js.effort[i] = 0;
      }
      else if(type == CRTK_jf){
        setpoint_out_js.effort[i] = in[i];
        setpoint_out_js.velocity[i] = 0;
        setpoint_out_js.position[i] = 0;
      }
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



/**
 * @brief      Sets the goal out based on the level and type flags stored in the object.
 *
 *
 * @return     { description_of_the_return_value }
 */
char CRTK_motion_api::set_goal_out(){
  if(is_tf_type(goal_out_type)){
    set_goal_out_tf(goal_out_level,goal_out_type,get_goal_in_tf(goal_out_level,goal_out_type));
    return 1;
  }
  else if(is_js_type(goal_out_type)){
    set_goal_out_js(goal_out_level,goal_out_type,get_goal_in_js(goal_out_level,goal_out_type));
    return 1;
  }
  ROS_ERROR("Set goal_out_error!");
  return 0;
}

char CRTK_motion_api::set_goal_out_tf(CRTK_motion_level level, CRTK_motion_type type, tf::Transform in){
//TODO remove level and type from params 
  if(is_tf_type(type)){
    goal_out_tf = in;
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
//TODO remove level and type from params 
  if(is_js_type(type)){
    for(int i=0; i<7; i++){
      if(type == CRTK_jr)
        goal_out_js.position[i] = in[i];
      else if(type == CRTK_jp)
        goal_out_js.position[i] = in[i];
      else if(type == CRTK_jv){
        goal_out_js.velocity[i] = in[i];
      }
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


//TODO rename to copy
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


void CRTK_motion_api::copy_setpoint_out_js(sensor_msgs::JointState* in){
  setpoint_out_js = *in;
}

void CRTK_motion_api::copy_goal_out_js(sensor_msgs::JointState* in){ 
  goal_out_js = *in;
}


tf::Transform CRTK_motion_api::get_setpoint_out_tf(){
  return setpoint_out_tf;
}
sensor_msgs::JointState CRTK_motion_api::get_setpoint_out_js(){  
  return setpoint_out_js;
}


float CRTK_motion_api::get_setpoint_out_js_value(CRTK_motion_type type, int index)
{
  if(type == CRTK_jp || type == CRTK_jr)
    return setpoint_out_js.position[index];
  else if(type == CRTK_jv)
    return setpoint_out_js.velocity[index];
  else if(type == CRTK_jf)
    return setpoint_out_js.effort[index];
  else
  {
    ROS_INFO("get_setpoint_out_js_value invalid request type.");
    return -1;
  }
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
    if(type == CRTK_cr){
      goal_in[level].cr = in;
      goal_in[level].update_flags[CRTK_cr] = 1;
    }       
    else if(type == CRTK_cp){
      goal_in[level].cp = in;
      goal_in[level].update_flags[CRTK_cp] = 1;
    }  
    else if(type == CRTK_cv){
      goal_in[level].cv = in; 
      goal_in[level].update_flags[CRTK_cv] = 1;
    }    
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
      if(type == CRTK_jr){
        goal_in[level].jr[i] = in[i];
        goal_in[level].update_flags[CRTK_jr] = 1;
      }       
      else if(type == CRTK_jp){
        goal_in[level].jp[i] = in[i];
        goal_in[level].update_flags[CRTK_jp] = 1;
      }  
      else if(type == CRTK_jv){
        goal_in[level].jv[i] = in[i];  
        goal_in[level].update_flags[CRTK_jv] = 1;
      }  
      else {
        goal_in[level].jf[i] = in[i];  
        goal_in[level].update_flags[CRTK_jf] = 1;
      }                     
    }
    goal_in[level].updated = 1;
  }
}

void CRTK_motion_api::set_setpoint_in(CRTK_motion_level level, CRTK_motion_type type, tf::Transform in){
  if(is_tf_type(type) && is_type_valid(level,type)){
    if(type == CRTK_cr){
      setpoint_in[level].cr = in;
      setpoint_in[level].update_flags[CRTK_cr] = 1;
    }       
    else if(type == CRTK_cp){
      setpoint_in[level].cp = in;
      setpoint_in[level].update_flags[CRTK_cp] = 1;      
    }  
    else if(type == CRTK_cv){
      setpoint_in[level].cv = in;   
      setpoint_in[level].update_flags[CRTK_cv] = 1;        
    }  
    setpoint_in[level].updated = 1;
  }
  else ROS_ERROR("set_setpoint_in tf called in error");
}

void CRTK_motion_api::set_setpoint_in(CRTK_motion_level level, CRTK_motion_type type, float* in){
  if(is_js_type(type) && is_type_valid(level,type)){
    for(int i=0; i<7; i++){
      if(type == CRTK_jr) {
        setpoint_in[level].jr[i] = in[i];
        setpoint_in[level].update_flags[CRTK_jr] = 1; 
      }      
      else if(type == CRTK_jp){
        setpoint_in[level].jp[i] = in[i];
        setpoint_in[level].update_flags[CRTK_jp] = 1; 
      }  
      else if(type == CRTK_jv)  {
        setpoint_in[level].jv[i] = in[i];
        setpoint_in[level].update_flags[CRTK_jv] = 1;     
      }
      else {
        setpoint_in[level].jf[i] = in[i]; 
        setpoint_in[level].update_flags[CRTK_jf] = 1; 
      }                     
    }
    setpoint_in[level].updated = 1;
  }
  else ROS_ERROR("set_setpoint_in tf called in error");
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

void CRTK_motion_api::get_setpoint_in_js(CRTK_motion_level level, CRTK_motion_type type, float* out){

  if(is_js_type(type) && is_type_valid(level,type)){
    if(type == CRTK_jr){
      for(int i=0;i<7;i++)
        out[i] = setpoint_in[level].jr[i];
    }       
    else if(type == CRTK_jp){
      for(int i=0;i<7;i++)
        out[i] = setpoint_in[level].jp[i];
    }       
    else if(type == CRTK_jv){
      for(int i=0;i<7;i++)
        out[i] = setpoint_in[level].jv[i];
    }       
    else {
      for(int i=0;i<7;i++)
        out[i] = setpoint_in[level].jf[i];
    }       
  }
  return;
}


/**
 * @brief      Gets the goal out level.
 *
 * @return     The goal out level.
 */
CRTK_motion_level   CRTK_motion_api::get_goal_out_level(){
  return goal_out_level;
}

/**
 * @brief      Gets the goal out type.
 *
 * @return     The goal out type.
 */
CRTK_motion_type    CRTK_motion_api::get_goal_out_type(){
  return goal_out_type;
}

/**
 * @brief      Gets the setpoint out level.
 *
 * @return     The setpoint out level.
 */
CRTK_motion_level   CRTK_motion_api::get_setpoint_out_level(){
  return setpoint_out_level;
}

/**
 * @brief      Gets the setpoint out type.
 *
 * @return     The setpoint out type.
 */
CRTK_motion_type    CRTK_motion_api::get_setpoint_out_type(){
  return setpoint_out_type;
}


/**
 * @brief      Gets the new base frame flag.
 *
 * @return     The new base frame flag.
 */
char CRTK_motion_api::get_new_base_frame_flag(){
  return new_base_frame_flag;
}


/**
 * @brief      Sets the new base frame flag indicating that it should be sent to device
 *
 * @param[in]  in    new flag value (0 or 1)
 *
 * @return     returns the new flag value -1 for error
 */
char CRTK_motion_api::set_new_base_frame_flag(char in){
  if(in == 0 || in == 1){
    new_base_frame_flag = in;
    return in;
  }
  return -1;
}


/**
 * @brief      Gets the setpoint update flag for a given leveland type.
 *
 * @param[in]  level  The level
 * @param[in]  type   The type
 *
 * @return     The setpoint update flag.
 */
char CRTK_motion_api::get_setpoint_update_flag(CRTK_motion_level level, CRTK_motion_type type){
  char out = 0;
  if(setpoint_in[level].update_flags[type])
    out = 1;

  return out;
}


float CRTK_motion_api::get_setpoint_out_grasp_angle(){

  // static int count=0;
  // if(count %500 == 0)
  //   ROS_INFO("getting grasper setpoint_out angle %f",setpoint_out_js.position[0]);
  // count ++;
  return setpoint_out_js.position[0];
}



/**
 * @brief      Checks the input position against defined workspace limits
 *
 * @param[in]  pos   The position in meters
 *
 * @return     Is the position within the workspace
 */
char pos_in_workspace(tf::Vector3 pos){
  char out = 0;
  if(pos.x() > WKSP_X) return out;
  else if(pos.x() < WKSP_X_NEG) return out;
  else if(pos.y() > WKSP_Y) return out;
  else if(pos.y() < WKSP_Y_NEG) return out;
  else if(pos.z() > WKSP_Z) return out;
  else if(pos.z() < WKSP_Z_NEG) return out;
  else out = 1;
  return out;
}