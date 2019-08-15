/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University
 *of Washington BioRobotics Laboratory
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

#include "update_device_state.h"
#include "log.h"
#include "utils.h"

extern DOF_type DOF_types[];
extern int NUM_MECH;
extern volatile int isUpdated;
extern unsigned long gTime;

unsigned int newDofTorqueSetting = 0;  // for setting torque from console
int newDofTorqueMech = 0;     // for setting torque from console
int newDofTorqueMech_name = -5;     // for setting torque from console
unsigned int newDofTorqueDof = 0;      //
unsigned int newDofPosSetting = 0;     // for setting torque from console
int newDofPosMech = 0;        // for setting torque from console
int newDofPosMech_name = -5; 
unsigned int newDofPosDof = 0;         //
int newDofTorqueTorque = 0;            // torque value in mNm
float newDofPosPos = 0;                // pos delta in rad or m
t_controlmode newRobotControlMode = homing_mode;

/**
 * updateDeviceState - Function that update the device state based on parameters
 *passed from
 *       the user interface
 *
 * \param params_current    the current set of parameters
 * \param arams_update      the new set of parameters
 * \param device0           pointer to device informaiton
 *
 */
int updateDeviceState(param_pass *currParams, param_pass *rcvdParams, device *device0) {
  currParams->last_sequence = rcvdParams->last_sequence;
  for (int i = 0; i < NUM_MECH; i++) {
    currParams->xd[i].x = rcvdParams->xd[i].x;
    currParams->xd[i].y = rcvdParams->xd[i].y;
    currParams->xd[i].z = rcvdParams->xd[i].z;
    currParams->rd[i].yaw = rcvdParams->rd[i].yaw;
    currParams->rd[i].pitch = rcvdParams->rd[i].pitch * WRIST_SCALE_FACTOR;
    currParams->rd[i].roll = rcvdParams->rd[i].roll;
    currParams->rd[i].grasp = rcvdParams->rd[i].grasp;
  }

  // set desired mech position in pedal_down runlevel
  if (currParams->runlevel == RL_PEDAL_DN) {
    for (int i = 0; i < NUM_MECH; i++) {
      device0->mech[i].pos_d.x = rcvdParams->xd[i].x;
      device0->mech[i].pos_d.y = rcvdParams->xd[i].y;
      device0->mech[i].pos_d.z = rcvdParams->xd[i].z;
      device0->mech[i].ori_d.grasp = rcvdParams->rd[i].grasp;



      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) device0->mech[i].ori_d.R[j][k] = rcvdParams->rd[i].R[j][k];
    }
  }

  //copy current joint positions to rcvd params so they can be passed to data1 for local_io kin calcs
  for (int i = 0; i < NUM_MECH; i++) {
    for(int j = 0; j < MAX_DOF_PER_MECH; j++)
      rcvdParams->jpos[i*MAX_DOF_PER_MECH + j] = device0->mech[i].joint[j].jpos;
  }


  //also pass teleop transform so they can be passed to data1 for local_io transforming
  for(int i = 0; i < NUM_MECH; i++){
    for(int j = 0; j < 4; j++){
       rcvdParams->teleop_tf_quat[i*4 + j] = device0->mech[i].teleop_transform[j];
    }
  }

  // Switch control modes only in pedal up or init.
  if ((currParams->runlevel == RL_E_STOP) &&
      (currParams->robotControlMode != (int)newRobotControlMode)) {
    currParams->robotControlMode = (int)newRobotControlMode;
    log_msg("Control mode updated");
  }

  // Set new torque command from console user input
  if (newDofTorqueSetting) {
    newDofTorqueMech = mech_num_from_name(device0, newDofTorqueMech_name);
    if (newDofTorqueMech >= 0){
      // reset all other joints to zero
      for (unsigned int idx = 0; idx < MAX_MECH_PER_DEV * MAX_DOF_PER_MECH; idx++) {
        if (idx == MAX_DOF_PER_MECH * newDofTorqueMech + newDofTorqueDof)
          currParams->torque_vals[idx] = newDofTorqueTorque;
        else
          currParams->torque_vals[idx] = 0;
      }
      newDofTorqueSetting = 0;
      log_msg("DOF Torque updated\n");
    }
  }

  // Set new joint position command from console user input
  // robotControlMode for keyboard control of RAVEN is not yet set
  // TODO: reset to new keyboard mode when implemented
  if (newDofPosSetting && currParams->robotControlMode == -99) {
    // reset all other joints to zero
    newDofPosMech = mech_num_from_name(device0, newDofPosMech_name);
    if (newDofPosMech >= 0){
      for (unsigned int idx = 0; idx < MAX_MECH_PER_DEV * MAX_DOF_PER_MECH; idx++) {
        if (idx == MAX_DOF_PER_MECH * newDofPosMech + newDofPosDof)
          currParams->jpos_d[idx] += newDofPosPos;
        else
          currParams->jpos_d[idx] += 0;
      }
      newDofPosSetting = 0;
      log_msg("DOF Pos updated --- %f4\n",
              currParams->jpos_d[MAX_DOF_PER_MECH * newDofPosMech + newDofPosDof]);
    }
  }

  // Set new surgeon mode
  // if (device0->crtk_state.get_pedal_trigger() == -1){
  //   device0->surgeon_mode = 1; // set pedal down
  //   device0->crtk_state.reset_pedal_trigger();
  // }
  // else if(device0->crtk_state.get_pedal_trigger() == 1){
  //   device0->surgeon_mode = 0; // set pedal up
  //   device0->crtk_state.reset_pedal_trigger();
  // }
  // else 
  if (device0->surgeon_mode != rcvdParams->surgeon_mode) {
    device0->surgeon_mode = rcvdParams->surgeon_mode;  // store the surgeon_mode to DS0
  }

  return 0;
}

/**
*  setRobotControlMode()
*       Change controller mode, i.e. position control, velocity control, visual
* servoing, etc
*   \param t_controlmode    current control mode.
*/
void setRobotControlMode(t_controlmode in_controlMode) {
  log_msg("Robot control mode: %d", in_controlMode);
  newRobotControlMode = in_controlMode;
  isUpdated = TRUE;
}

/**
 * setDofTorque() Set a torque to output on a joint. Torque input is mNm
 *
 * @param      in_mech_name  Mechinism name of the joint
 * @param      in_dof        DOF number
 * @param      in_torque     Torque to set the DOF to (in mNm)
 */
void setDofTorque(unsigned int in_mech_name, unsigned int in_dof, int in_torque) {
  if (((int)in_mech_name < NUM_MECH) && ((int)in_dof < MAX_DOF_PER_MECH)) {
    newDofTorqueMech_name = (int)in_mech_name;
    newDofTorqueDof = in_dof;
    newDofTorqueTorque = in_torque;
    newDofTorqueSetting = 1;
  }
  isUpdated = TRUE;
}

/**
 * addDofPos() Set a pos to output on a joint.
 *
 * @param      in_mech_name  Mechinism name of the joint
 * @param      in_dof        DOF number
 * @param[in]  in_pos        In position
 * @param      in_torque  value to add to joint angle - radians or meters
 */
void addDofPos(unsigned int in_mech_name, unsigned int in_dof, float in_pos) {
  if (((int)in_mech_name < NUM_MECH) && ((int)in_dof < MAX_DOF_PER_MECH)) {
    newDofPosMech_name = (int)in_mech_name;
    newDofPosDof = in_dof;
    newDofPosPos = in_pos;
    newDofPosSetting = 1;
  }
  isUpdated = TRUE;
}


// Takes the current position and other data from the RAVEN structures and places them
// in the CRTK objects
//
// @param      dev   The robot device
//
// @return     success
//
int update_motion_apis(device* dev){
    tf::Vector3 curr_pos;
    tf::Matrix3x3 curr_rot;
    tf::Transform curr_tf, new_tf;
    tf::Transform base_transform;

    float r[9];
    // TODO: copy current to previous
    for(int i=0; i<2; i++){
      for(int j=0; j<9; j++){
        r[j] = dev->mech[i].ori.R[j/3][j%3];
      }
      curr_rot= tf::Matrix3x3(r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8]);
      curr_pos = tf::Vector3(dev->mech[i].pos.x/MICRON_PER_M,dev->mech[i].pos.y/MICRON_PER_M,dev->mech[i].pos.z/MICRON_PER_M);
      curr_tf = tf::Transform(curr_rot,curr_pos);

      base_transform = dev->crtk_motion_planner.crtk_motion_api[i].get_base_frame();//.inverse();
      new_tf =  base_transform * curr_tf; 
      
      dev->crtk_motion_planner.crtk_motion_api[i].set_pos(new_tf);
    }
    // counter ++;
    return 1;
}

/**
 * @brief      { function_description }
 *
 * @param      <unnamed>  { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
int update_device_crtk_motion(device* dev){
  static int busy_cycle = 0;
  static char busy_flag = 0;
  CRTK_motion_type type;
  bool js_messages = false;
  int out = 0;
  for(int i=0;i<2;i++){
    type = dev->crtk_motion_planner.crtk_motion_api[i].get_setpoint_out_type();    

    if(is_tf_type(type)){
      
      out = update_device_crtk_motion_tf(dev,i);
    }   
    else if(is_js_type(type)){
      // static int crud;
      // if(crud%500 == 1) ROS_INFO("Got 500 js messages :0");
      // crud++;
      js_messages = true;
      out = update_device_crtk_motion_js(dev,i);
    }

    type = dev->crtk_motion_planner.crtk_motion_api_grasp[i].get_setpoint_out_type();    
    if(is_js_type(type) && !js_messages)
      update_device_crtk_grasp(dev,i);

    dev->crtk_motion_planner.crtk_motion_api[i].reset_setpoint_out(); 
    dev->crtk_motion_planner.crtk_motion_api_grasp[i].reset_setpoint_out(); 
  
    //wait for js radio silence for 10 loops before switching back to cart mode

    if (!js_messages) dev->mech[i].joint_control = false;
  }

  if(out || busy_flag) 
  {
    dev->crtk_state.set_busy(1);
    busy_flag = 1;
    busy_cycle ++;
    if(!out && busy_cycle > 1000)
      busy_flag = 0;
  }
  else 
  {
    dev->crtk_state.set_busy(0);
    busy_cycle = 0;
  }


  return 1;
 
}

/**
 * @brief      updates the desired motions of the device from the CRTK interface
 *
 * @param      dev   The device
 * @param[in]  arm   The arm that's being updated
 *
 * @return     success
 */
int update_device_crtk_motion_tf(device* dev, int arm){

  float max_dist_per_ms = 0.1;   // m/ms
  float max_radian_per_ms = 0.5;  // rad/ms
  // float max_dist_per_s = 0.1 * SECOND;   // m/s
  // float max_radian_per_s = 0.5 * SECOND;  // rad/s
  // static int count = 0;
  int out = 0;
  CRTK_motion_type  type = dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_type(); 
  tf::Quaternion curr_rot = tf::Transform(tf::Matrix3x3(dev->mech[arm].ori.R[0][0], dev->mech[arm].ori.R[0][1], dev->mech[arm].ori.R[0][2], 
                                                        dev->mech[arm].ori.R[1][0], dev->mech[arm].ori.R[1][1], dev->mech[arm].ori.R[1][2], 
                                                        dev->mech[arm].ori.R[2][0], dev->mech[arm].ori.R[2][1], dev->mech[arm].ori.R[2][2])).getRotation();
  switch(type){
    case CRTK_cr:{
      tf::Transform incr_tf = dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_tf();

      // (1) for translation
      tf::Vector3 incr = incr_tf.getOrigin();
    
      // rotate to RAVEN frame and scale from meters to microns
      incr = dev->crtk_motion_planner.crtk_motion_api[arm].get_base_frame().inverse() * incr;
      incr = incr * MICRON_PER_M;
      

      if(incr.length() <= max_dist_per_ms * MICRON_PER_M){
        dev->mech[arm].pos_d.x += (int)(incr.x());
        dev->mech[arm].pos_d.y += (int)(incr.y());
        dev->mech[arm].pos_d.z += (int)(incr.z());
        out = 1;
      }
      else{
        ROS_INFO("Relative Cartesian translation scaled to max speed. (length= %f m per ms)",incr.length()/MICRON_PER_M);

        incr = incr.normalized() * max_dist_per_ms * MICRON_PER_M;

        dev->mech[arm].pos_d.x += (int)(incr.x());
        dev->mech[arm].pos_d.y += (int)(incr.y());
        dev->mech[arm].pos_d.z += (int)(incr.z());
        out = 1;
      }

      // (2) for rotation!
      static int count = 0;
      count ++;

      tf::Matrix3x3 t1 = dev->crtk_motion_planner.crtk_motion_api[arm].get_base_frame().inverse().getBasis();
      tf::Matrix3x3 t2 = incr_tf.getBasis();
      tf::Matrix3x3 q_temp = t1*t2*t1.inverse();
      tf::Matrix3x3 mx_temp = tf::Transform(tf::Matrix3x3(dev->mech[arm].ori_d.R[0][0], dev->mech[arm].ori_d.R[0][1], dev->mech[arm].ori_d.R[0][2], 
                                                        dev->mech[arm].ori_d.R[1][0], dev->mech[arm].ori_d.R[1][1], dev->mech[arm].ori_d.R[1][2], 
                                                        dev->mech[arm].ori_d.R[2][0], dev->mech[arm].ori_d.R[2][1], dev->mech[arm].ori_d.R[2][2])).getBasis();


      if (tf::Transform(t2).getRotation() != tf::Quaternion::getIdentity()) {
        q_temp = q_temp * mx_temp;
        tf::Matrix3x3 rot_mx_temp(q_temp);


        for (int j = 0; j < 3; j++)
          for (int k = 0; k < 3; k++) 
            dev->mech[arm].ori_d.R[j][k] = rot_mx_temp[j][k];
      }

      else{
        out = 0;
      }
      return out;
      break;
    }

    case CRTK_cp:{
      tf::Vector3 new_pos = dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_tf().getOrigin();
      tf::Vector3 curr_pos = tf::Vector3(dev->mech[arm].pos.x,dev->mech[arm].pos.y,dev->mech[arm].pos.z);

      tf::Quaternion new_rot = dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_tf().getRotation();
      
      // rotate to RAVEN frame and scale from meters to microns
      new_pos = dev->crtk_motion_planner.crtk_motion_api[arm].get_base_frame().inverse() * new_pos;
      new_pos = new_pos * MICRON_PER_M;

      if(fabs((new_pos-curr_pos).length()) <= max_dist_per_ms * MICRON_PER_M){
        dev->mech[arm].pos_d.x = (int)(new_pos.x());
        dev->mech[arm].pos_d.y = (int)(new_pos.y());
        dev->mech[arm].pos_d.z = (int)(new_pos.z());
        out = 1;
      }
      else if (fabs((new_pos-curr_pos).length()) <= 5 * max_dist_per_ms * MICRON_PER_M){
        // tf::Vector3 vec_threshed = (new_pos-curr_pos).normalized() * max_dist_per_ms * MICRON_PER_M;
        dev->mech[arm].pos_d.x = (int)(new_pos.x());
        dev->mech[arm].pos_d.y = (int)(new_pos.y());
        dev->mech[arm].pos_d.z = (int)(new_pos.z());
        // dev->mech[arm].pos_d.x = (int)(curr_pos.x() + vec_threshed.x());
        // dev->mech[arm].pos_d.y = (int)(curr_pos.y() + vec_threshed.y());
        // dev->mech[arm].pos_d.z = (int)(curr_pos.z() + vec_threshed.z());
        out = 1;
      }
      else{
        ROS_ERROR("Absolute Cartesian step size too large. (step length = %f micron per ms)",fabs((new_pos-curr_pos).length()));
        out = 0;
      }

      //rotation
      //transform CRTK rotation to RAVEN frame
      tf::Quaternion t1 = dev->crtk_motion_planner.crtk_motion_api[arm].get_base_frame().inverse().getRotation();
      new_rot = t1*new_rot;

      //check rotation step, update ori_d if fine
      if(new_rot.length() > 0 && !isnan(new_rot.length())){
        if(new_rot.angleShortestPath(curr_rot) <= max_radian_per_ms){
          tf::Matrix3x3 incr_rot_mx_mx = tf::Transform(new_rot).getBasis();
          for(int i=0; i<3 ; i++){
            for(int j=0;j<3;j++){
              dev->mech[arm].ori_d.R[i][j] = incr_rot_mx_mx[i][j];
            }
          }
        }
        else{
          ROS_ERROR("Absolute Cartesian rotation too large. (angle= %f deg per ms)",new_rot.angleShortestPath(curr_rot) RAD2DEG);
          out = -1;
        }
      }
      else{
        out = 0;
      }
      return out;
      break;
    }
    case CRTK_cv:{
      tf::Transform vel_tf = dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_tf();

      // (1) for translation
      tf::Vector3 incr = vel_tf.getOrigin() * ONE_MS;
    
      // rotate to RAVEN frame and scale from meters to microns
      incr = dev->crtk_motion_planner.crtk_motion_api[arm].get_base_frame().inverse() * incr;
      incr = incr * MICRON_PER_M;
      

      if(incr.length() <= max_dist_per_ms * MICRON_PER_M){
        dev->mech[arm].pos_d.x += (int)(incr.x());
        dev->mech[arm].pos_d.y += (int)(incr.y());
        dev->mech[arm].pos_d.z += (int)(incr.z());
        out = 1;
      }
      else{
        ROS_INFO("Cartesian velocity translation scaled to max speed. (length= %f m per ms)",incr.length()/MICRON_PER_M);

        incr = incr.normalized() * max_dist_per_ms * MICRON_PER_M;

        dev->mech[arm].pos_d.x += (int)(incr.x());
        dev->mech[arm].pos_d.y += (int)(incr.y());
        dev->mech[arm].pos_d.z += (int)(incr.z());
        out = 1;
      }

      // (2) for rotation!
      tf::Quaternion vel_rot = vel_tf.getRotation();
      tf::Vector3 incr_rot_axis = vel_rot.getAxis().normalize();
      float incr_rot_angle = vel_rot.getAngle() * ONE_MS;
      tf::Quaternion incr_rot = tf::Quaternion(incr_rot_axis,incr_rot_angle);

      tf::Matrix3x3 t1 = dev->crtk_motion_planner.crtk_motion_api[arm].get_base_frame().inverse().getBasis();
      tf::Matrix3x3 t2 = tf::Matrix3x3(incr_rot);
      tf::Matrix3x3 q_temp = t1*t2*t1.inverse();
      tf::Matrix3x3 mx_temp = tf::Transform(tf::Matrix3x3(dev->mech[arm].ori_d.R[0][0], dev->mech[arm].ori_d.R[0][1], dev->mech[arm].ori_d.R[0][2], 
                                                        dev->mech[arm].ori_d.R[1][0], dev->mech[arm].ori_d.R[1][1], dev->mech[arm].ori_d.R[1][2], 
                                                        dev->mech[arm].ori_d.R[2][0], dev->mech[arm].ori_d.R[2][1], dev->mech[arm].ori_d.R[2][2])).getBasis();


      if (tf::Transform(t2).getRotation() != tf::Quaternion::getIdentity()) {
        q_temp = q_temp * mx_temp;
        tf::Matrix3x3 rot_mx_temp(q_temp);


        for (int j = 0; j < 3; j++)
          for (int k = 0; k < 3; k++) 
            dev->mech[arm].ori_d.R[j][k] = rot_mx_temp[j][k];

        tf::Quaternion test = tf::Transform(rot_mx_temp).getRotation();
      }

      else{
        out = 0;
      }
      return out;
      break;
      break;
    }
    default:{
      ROS_ERROR("Unsupported motion tf type.");
      return -1;
    }
  }

  return out;
}



int update_device_crtk_motion_js(device* dev, int arm){

  float max_radian_per_ms = 0.0025;    // rad/ms
  float max_meters_per_ms = 0.00005;  // meters/ms
  float max_radian_per_s = max_radian_per_ms * SECOND;  // rad/m
  float max_meters_per_s = max_meters_per_ms * SECOND;  // meters/m
  static int count=0;
  static int limit_count=0;
  int index_offset =0;
  int out = 0;
  CRTK_motion_type  type = dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_type(); 
  sensor_msgs::JointState setpoint = dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_js();

  dev->mech[arm].joint_control = true;

/*
  for(int i=0; i<7;i++)
    ROS_INFO("arm %d: setpoint[%d]=(%f)",arm, i, 
      dev->crtk_motion_planner.crtk_motion_api[arm].get_setpoint_out_js_value(type,i));*/
  float limit;
  switch(type){
    case CRTK_jr:
        for(int i=0; i<7;i++){
          index_offset = (i>=3) ? 1 : 0;
          limit = (i==2) ? max_meters_per_ms : max_radian_per_ms;
          if(fabs(setpoint.position[i]) <= limit){ 
            
            dev->mech[arm].joint[i+index_offset].jpos_d += setpoint.position[i];
            out = 1;
          }
          else 
          {
            float sign = (setpoint.position[i] > 0) ? 1 : -1;
            dev->mech[arm].joint[i+index_offset].jpos_d += sign*limit;

            limit_count ++;
            out = 1;
            if(limit_count%500 == 0){
              ROS_ERROR("THAT WAS TOO FAR! joint %i jpos_d - %f, in %f ",
              i+index_offset, dev->mech[arm].joint[i+index_offset].jpos_d , setpoint.position[i]);
              out = -1;
            }
            
          }
        }
      break;

    case CRTK_jp:
        for(int i=0; i<7;i++){

          limit = (i==2) ? max_meters_per_ms : max_radian_per_ms;

          index_offset = (i>=3) ? 1 : 0;
          // if(count%250 == 0 && ( i == 3 || i == 4 ||i == 5 ||i == 6  ))
          // ROS_INFO("i+off %i, actual[i+off] - %f desired %f in_desired - %f", 
          // i+index_offset, dev->mech[arm].joint[i+index_offset].jpos, 
          // dev->mech[arm].joint[i+index_offset].jpos_d, setpoint.position[i]);


          float step_diff  = setpoint.position[i] - dev->mech[arm].joint[i+index_offset].jpos_d;
          if(fabs(step_diff) <= limit){
            dev->mech[arm].joint[i+index_offset].jpos_d = setpoint.position[i];
            out = 1;
          }
          else
          {

            float sign = (step_diff > 0) ? 1 : -1;
            dev->mech[arm].joint[i+index_offset].jpos_d = dev->mech[arm].joint[i+index_offset].jpos_d + sign*limit;
            out = 1;
            limit_count ++;

            if(limit_count%500 == 0)
            {
              ROS_ERROR("THAT WAS TOO FAR! joint %i actual, jpos_d - %f, in %f diff %f ",
               i+index_offset, dev->mech[arm].joint[i+index_offset].jpos_d , setpoint.position[i], 
               fabs(dev->mech[arm].joint[i+index_offset].jpos_d - setpoint.position[i]));
              out = -1;
            }
            
          }
        }
      break;

    case CRTK_jv:
        double incr;
        for(int i=0; i<7;i++){
          index_offset = (i>=3) ? 1 : 0;
          limit = (i==2) ? max_meters_per_s : max_radian_per_s;
          if(fabs(setpoint.velocity[i]) <= limit){ 
            incr = setpoint.velocity[i] * ONE_MS;
            dev->mech[arm].joint[i+index_offset].jpos_d += incr;
            out = 1;
          }
          else 
          {
            float sign = (setpoint.velocity[i] > 0) ? 1 : -1;
            dev->mech[arm].joint[i+index_offset].jpos_d += sign * limit * ONE_MS;

            limit_count ++;
            out = 1;
            if(limit_count%500 == 0){
              ROS_ERROR("THAT WAS TOO FAR! joint %i jpos_d - %f, in %f ",
              i+index_offset, dev->mech[arm].joint[i+index_offset].jpos_d , setpoint.velocity[i] * ONE_MS);
              out = -1;
            }
            
          }
        }
      break;

    case CRTK_jf:
        // TODO:
      break;

    default:
    {
      ROS_INFO("raven js setpoint out type = %i",(int)type);
      dev->mech[arm].joint_control = false;
      break;
    }

  }
  
  count ++;
  return out;
}



int update_device_crtk_grasp(device* dev, int arm){

  CRTK_motion_type type = dev->crtk_motion_planner.crtk_motion_api_grasp[arm].get_setpoint_out_type(); 
  sensor_msgs::JointState setpoint = dev->crtk_motion_planner.crtk_motion_api_grasp[arm].get_setpoint_out_js();
  float angle;

  static int count = 0;

  switch(type){

    case CRTK_jr:
    {
      // HELP! using "angle = setpoint.position[0]" doesn't work!!!!! 
      angle = dev->crtk_motion_planner.crtk_motion_api_grasp[arm].get_setpoint_out_grasp_angle();
      dev->mech[arm].ori_d.grasp = dev->mech[arm].ori_d.grasp + angle * 1000; //TODO: safety limit???      
      // if(count %500 == 0){
      //    ROS_INFO("doing 500 grasp things!!!!! omg %i = %i + %i", dev->mech[arm].ori_d.grasp, dev->mech[arm].ori.grasp, (int)angle * 1000);
      // }
      break;
    }
    case CRTK_jp:
    {
      angle = dev->crtk_motion_planner.crtk_motion_api_grasp[arm].get_setpoint_out_grasp_angle();
      dev->mech[arm].ori_d.grasp = angle; //TODO: safety limit???
      break;
    }
    case CRTK_jv:// TODO later:D
    {
      // angle = setpoint.velocity[0];
      // dev->mech[arm].ori_d.grasp = angle;
      break;
    }
    case CRTK_jf:// TODO later:D
    {
      // angle = setpoint.effort[0];
      // dev->mech[arm].ori_d.grasp = angle;
      break;
    }
    default:
    {
      ROS_INFO("grasper setpoint out type = %i",(int)type);
      break;
    }

  }

  count ++;
  return 1;
}