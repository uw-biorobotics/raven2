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

/***************************************/ /**

 \brief This is where the USB, ITP, and ROS interfaces live

 Local_io keeps its own copy of DS1 for incorporating new
 network-layer and toolkit updates.

 The local DS1 copy is protected by a mutex.

 This transient copy should then be read to another copy for
 active use.

 ROS publishing is at the bottom half of this file.

  ***************************************/

/// TODO: Modify the guts of local comm and network layer
#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // For realtime posix support. see
                     // http://www.gnu.org/s/libc/manual/html_node/Feature-Test-Macros.html
#endif 

#include <cstring>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_datatypes.h>

#include "log.h"
#include "local_io.h"
#include "utils.h"
#include "mapping.h"
#include "itp_teleoperation.h"
#include "r2_kinematics.h"
#include "reconfigure.h"
#include "r2_jacobian.h"
#include "crtk_motion_api.h"

extern int NUM_MECH;
extern USBStruct USBBoards;
extern unsigned long int gTime;

const static double d2r = M_PI / 180;  // degrees to radians
const static double r2d = 180 / M_PI;  // radians to degrees

static param_pass data1;  // local data structure that needs mutex protection
tf::Quaternion Q_ori[2];  //local representation of robot orientation, sync'd 
                          //with update master function to avoid divergence
pthread_mutexattr_t data1MutexAttr;
pthread_mutex_t data1Mutex;

volatile int isUpdated;  // TODO: HK volatile int instead of atomic_t ///Should we use
                         // atomic builtins?
                         // http://gcc.gnu.org/onlinedocs/gcc-4.1.2/gcc/Atomic-Builtins.html

extern offsets offsets_l;
extern offsets offsets_r;

int cameraTransform(tf::Quaternion &q_temp, position &p, int armidx);

CRTK_motion_api crtk_motion_api_gold(0); //class to hold robot status flags for CRTK API
CRTK_motion_api crtk_motion_api_green(1); //class to hold robot status flags for CRTK API
CRTK_motion_api crtk_motion_api_gold_grasp(0); //class to hold robot status flags for CRTK API
CRTK_motion_api crtk_motion_api_green_grasp(1); //class to hold robot status flags for CRTK API


CRTK_motion_api crtk_motion_api_blue(0); //class to hold robot status flags for CRTK API
CRTK_motion_api crtk_motion_api_orange(1); //class to hold robot status flags for CRTK API
CRTK_motion_api crtk_motion_api_blue_grasp(0); //class to hold robot status flags for CRTK API
CRTK_motion_api crtk_motion_api_orange_grasp(1); //class to hold robot status flags for CRTK API

/**
 * \brief Initialize data arrays to zero and create mutex
 *
 * The mutex is a method to protect the data from being overwritten while it's
 *being used
 * \ingroup DataStructures
 */

int initLocalioData(device *device0) {
  int i;
  pthread_mutexattr_init(&data1MutexAttr);
  pthread_mutexattr_setprotocol(&data1MutexAttr, PTHREAD_PRIO_INHERIT);
  pthread_mutex_init(&data1Mutex, &data1MutexAttr);

  pthread_mutex_lock(&data1Mutex);
  for (i = 0; i < NUM_MECH; i++) {
    data1.xd[i].x = 0;
    data1.xd[i].y = 0;
    data1.xd[i].z = 0;
    data1.rd[i].yaw = 0;
    data1.rd[i].pitch = 0;
    data1.rd[i].roll = 0;
    data1.rd[i].grasp = 0;
    Q_ori[i] = Q_ori[i].getIdentity();
  }
  data1.surgeon_mode = 0;
  data1.last_sequence = 111;

  log_msg("NUM_MECH -->  %i", NUM_MECH);

  pthread_mutex_unlock(&data1Mutex);

  crtk_motion_api_gold.set_default_base_frame(0);
  crtk_motion_api_green.set_default_base_frame(1);
  crtk_motion_api_blue.set_default_base_frame(2);
  crtk_motion_api_orange.set_default_base_frame(3);
  return 0;
}

/**
 * \brief Initiates update of data1 local paramater structure from userspace
 *
 * \param u pointer to new data
 * \param size
 *
 * \ingroup DataStructures
 * \todo check checksum, figure out what to do if the checksum fails
 */

int receiveUserspace(void *u, int size) {
  if (size == sizeof(u_struct)) {
    isUpdated = TRUE;
    teleopIntoDS1((u_struct *)u);
  }
  return 0;
}

/**
 * \brief Puts the master data into protected structure
 *
 * Takes the data from the master structure and places it into the parameter
 *passing structure.
 *
 * \question why is setting the sequence number like this a hack?
 * \todo Apply transform to incoming data </capslock>
 * \param us_t a pointer to the user input structure
 *
 *  \ingroup DataStructures
 */
void teleopIntoDS1(u_struct *us_t) {
  position p, p_temp;
  int i, armidx, arm_name, loops;
  pthread_mutex_lock(&data1Mutex);
  tf::Quaternion q_temp;
  tf::Matrix3x3 rot_mx_temp;
  tfScalar roll_temp, pitch_temp, yaw_temp;

  int debug_camera = 0;
  int extra_arm_mode = 0;

  static char print_once = 0;

  if(data1.robotControlMode == teleop_34 && print_once){
    print_once = 0;
    extra_arm_mode = 1;
    ROS_INFO("We're in camera teleop mode, now!");
  }

  // TODO:: APPLY TRANSFORM TO INCOMING DATA

  // this function wants to loop through boards instead of mechanisms
  // TODO:: loop over mechanisms instead?
  loops = USBBoards.activeAtStart;

  for (i = 0; i < loops; i++) {
    if (USBBoards.boards[i] == GOLD_ARM_SERIAL && !extra_arm_mode) {
      arm_name = gold;
      armidx = 0;
    } else if (USBBoards.boards[i] == GREEN_ARM_SERIAL && !extra_arm_mode) {
      arm_name = green;
      armidx = 1;
    } else if (USBBoards.boards[i] == BLUE_ARM_SERIAL && extra_arm_mode) {
      arm_name = blue;
      armidx = 2;
    } else {
      continue;  // don't do any teleop data for joint encoders 
                 // or extra arms, yet
    }

    
    if(!extra_arm_mode){
      // apply mapping to teleop data
      p.x = us_t->delx[armidx];
      p.y = us_t->dely[armidx];
      p.z = us_t->delz[armidx];

     // set local quaternion from teleop quaternion data
      q_temp.setX(us_t->Qx[armidx]);
      q_temp.setY(us_t->Qy[armidx]);
      q_temp.setZ(us_t->Qz[armidx]);
      q_temp.setW(us_t->Qw[armidx]);
    } else if (extra_arm_mode && arm_name == blue) {
      // apply mapping to teleop data from left controller

      p.x = us_t->delx[0];
      p.y = us_t->dely[0];
      p.z = us_t->delz[0];

     // set local quaternion from teleop quaternion data
      q_temp.setX(us_t->Qx[0]);
      q_temp.setY(us_t->Qy[0]);
      q_temp.setZ(us_t->Qz[0]);
      q_temp.setW(us_t->Qw[0]);
    }
    
   

    

    //tool is a camera tool, only take z comp
    //TODO make this a separate function
    if(data1.param_tool_type[i] == qut_camera){ 
      cameraTransform(q_temp, p, armidx);
    }else
      fromITP(&p, q_temp, arm_name);


    data1.xd[i].x += p.x;
    data1.xd[i].y += p.y;
    data1.xd[i].z += p.z;

    // Add quaternion increment
    Q_ori[armidx] = q_temp * Q_ori[armidx];
    rot_mx_temp.setRotation(Q_ori[armidx]);

    // Set rotation command
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) data1.rd[i].R[j][k] = rot_mx_temp[j][k];


    // set grasp command
    const int graspmax = (M_PI / 2 * 1000);
    int graspmin = (-10.0 * 1000.0 DEG2RAD);
    int grasp_gain = 1;

    // start with determining the gain on the grasping input
    // e.g. the omni only sends {-1, 0, 1}, so an int gain of >2 
    // increases grasping speed and makes it more user-friendly
    // scissors can be helped with a bit of oomph   
    if(GRASP_GAIN) grasp_gain = GRASP_GAIN;


    if(isScissor(data1.param_tool_type[armidx])){
      grasp_gain *= 4;
      graspmin = (-40.0 * 1000.0 DEG2RAD);
    } 

    //add grasp value to data1
    // if(isCamera(data1.param_tool_type[armidx])) 
    if(data1.param_tool_type[i] == r_grasper) 
      data1.rd[i].grasp -= grasp_gain * us_t->grasp[armidx];
    else  
      data1.rd[i].grasp -= grasp_gain * us_t->grasp[armidx];

    //limit grasping to safe max and min limits  
    if (data1.rd[i].grasp > graspmax)
      data1.rd[i].grasp = graspmax;
    else if (data1.rd[i].grasp < graspmin)
      data1.rd[i].grasp = graspmin;

  }

  /// \question HK: why is this a hack?
  // HACK HACK HACK
  // HACK HACK HACK
  // HACK HACK HACK
  // HACK HACK HACK
  data1.last_sequence = us_t->sequence;

  data1.surgeon_mode = us_t->surgeon_mode;
  pthread_mutex_unlock(&data1Mutex);
}


/**
* \brief Calculates camera-centric commands in 0-frame from teleop
*
* 
*
* \return 0 if success
*/
int cameraTransform(tf::Quaternion& q_temp, position& p, int armidx){
  int success = 0;

  // tf_0_4 --> roll frame represented in frame 0 (at RCM)
  tf::Transform tf_0_4, tf_4_cz, tf_m_c, tf_0_m;
  tf::Quaternion q_master;
  tf::Vector3 k_c, k_cz, k_0, k_m; // equivalent angle axis
  tf::Vector3 v_m, v_0, v_rot_axis; // master position incr and ouput incr 
  tfScalar out_angle, mech_tool_frame[4];
  tfScalar roll_temp, pitch_temp, yaw_temp;

  int debug_camera = 0;

  // *** 0 set known transforms *** 
  tf_4_cz.setIdentity(); // this transformation can be used to 
                         // compensate for camera angle offsets

  // camera frame represented in master frame
  tf_m_c = tf::Transform(tf::Matrix3x3(0,  0,  1,
                                       0,  -1, 0,
                                       1,  0,  0)); 

  // *** 1 represent master input in tf datatypes *** 
  q_master = q_temp;
  // equiv angle axis of master rot incr
  k_m = q_master.getAngle() * q_master.getAxis(); 
  // vector type of master position incr
  v_m.setValue(p.x, p.y, p.z); 



  // *** 2 get the transform 0_4 passed from kinematics to data1 *** 
  for(int i = 0; i < 4; i++){
    mech_tool_frame[i] = data1.teleop_tf_quat[armidx * 4 + i];
  }
  tf_0_4.setRotation(tf::Quaternion(mech_tool_frame[0], mech_tool_frame[1], 
    mech_tool_frame[2], mech_tool_frame[3]));


  // *** 3 project master rotation onto z axis *** 
  // camera only allows roll about insertion axis
  k_c = tf_m_c.getBasis().inverse() * k_m;
  k_cz = k_c * tf::Vector3(0,0,1);

  // *** 4 kind k_0 and v_0
  // k_0 = tf_0_4 * t_4_cz * k_cz;
  // v_0 = tf_0_4 * t_4_cz * tf_m_c.inverse() * v_m;
  k_0 = tf_0_4.getBasis() * k_cz;
  v_0 = tf_0_4.getBasis() * tf_m_c.getBasis().inverse() * v_m;



  // *** 5 translate data back to traditional forms ***
  v_rot_axis = k_0.normalized();
  out_angle = k_0.length();
  char zero_flag = 0;
  if(out_angle == 0.0){
    zero_flag = 1;
    q_temp.setRPY(0,0,0);
  }else
    q_temp.setRotation(v_rot_axis, out_angle);
  p.x = v_0.getX(); p.y = v_0.getY(); p.z = v_0.getZ();

  static int check = 0;
  check++;
  if((check % 1000 == 0) && debug_camera){
    tf::Transform(q_master).getBasis().getEulerYPR(yaw_temp, pitch_temp, roll_temp);
    log_msg("q_master   r p y   %f \t %f \t %f", roll_temp, pitch_temp, yaw_temp);
    log_msg("K_c                %f \t %f \t %f", k_c.getX(), k_c.getY(), k_c.getZ());        
    log_msg("K_c proj           %f \t %f \t %f", k_cz.getX(), k_cz.getY(), k_cz.getZ());
    log_msg("K_0                %f \t %f \t %f", k_0.getX(), k_0.getY(), k_0.getZ());
    log_msg("V_0                %f \t %f \t %f", v_0.getX(), v_0.getY(), v_0.getZ());
    log_msg("rot_axis           %f \t %f \t %f", v_rot_axis.getX(), v_rot_axis.getY(), v_rot_axis.getZ());
    log_msg("out_angle                %f", out_angle);
    if(zero_flag) log_msg("Zero!");
    else log_msg("");        
    log_msg("---");  
  }
  return success;    
}

/**
 * \brief Checks if there has been a recent update from master
 *
 * Checks if there has been a recent update from master.
 * If it has been too long since last update it sets state to pedal-up.
 *
 * \return true if updates have been received from master or toolkit since last
 *module update
 * \return false otherwise
 *
 * \ingroup Networking
 *
 */
int checkLocalUpdates() {
  static unsigned long int lastUpdated;

  if (isUpdated || lastUpdated == 0) {
    lastUpdated = gTime;
  } 
  #ifdef ALLOW_TIMEOUT 
  else if (((gTime - lastUpdated) > MASTER_CONN_TIMEOUT) && (data1.surgeon_mode)) {
    // if timeout period is expired, set surgeon_mode "DISENGAGED" if currently
    // "ENGAGED"
    log_msg("Master connection timeout.  surgeon_mode -> up.\n");
    data1.surgeon_mode = SURGEON_DISENGAGED;
    //       data1.surgeon_mode = 1;

    lastUpdated = gTime;
    isUpdated = TRUE;
  }
  #endif

  return isUpdated;
}

/** \brief Give the latest updated DS1 to the caller.
 *
 *   \pre d1 is a pointer to allocated memory
 *   \post memory location of d1 contains latest DS1 Data from network/toolkit.
 *
 *   \param d1 pointer to the protected data structure
 *   \return a copy of the data as a param_pass structure
 *
 *   \todo HK Check performance of trylock / default priority inversion scheme
 *  \ingroup DataStructures
 */
param_pass *getRcvdParams(param_pass *d1) {
  // \TODO Check performance of trylock / default priority inversion scheme
  if (pthread_mutex_trylock(&data1Mutex) != 0)  // Use trylock since this 
          // function is called form rt-thread. return
          // immediately with old values if unable to lock
    return d1;
  // pthread_mutex_lock(&data1Mutex); //Priority inversion enabled. Should force
  // completion of other parts and enter into this section.
  
  //pass current position into data 1 before copying data 1 into rcvd params
  //not really necessary now that we pass the teleop transform
  for(int i =0; i < NUM_MECH * MAX_DOF_PER_MECH; i++){
    data1.jpos[i] = d1->jpos[i];
  }

  //also pass teleop transform 
  for(int i = 0; i < NUM_MECH; i++){
    for(int j = 0; j < 4; j++){
      data1.teleop_tf_quat[i*4 + j] = d1->teleop_tf_quat[i*4 + j];
    }
  }
  memcpy(d1, &data1, sizeof(param_pass));
  isUpdated = 0;

  pthread_mutex_unlock(&data1Mutex);
  return d1;
}

/**
 * \brief Resets the desired position to the robot's current position
 *
 * Resetting the desired position when the robot is idle will prevent it from
 *accumulating deltas.
 * This function is particularly useful to prevent the grasp position from
 *changing too much
 * while the robot is moving
 *
 * Reset writable copy of DS1
 *  \ingroup Networking
 */
void updateMasterRelativeOrigin(device *device0) {
  int armidx;
  orientation *_ori;
  tf::Matrix3x3 tmpmx;

  // update data1 (network position desired) to device0.position_desired (device
  // position desired)
  //   This eliminates accumulation of deltas from network while robot is idle.
  pthread_mutex_lock(&data1Mutex);
  for (int i = 0; i < NUM_MECH; i++) {
    data1.xd[i].x = device0->mech[i].pos_d.x;
    data1.xd[i].y = device0->mech[i].pos_d.y;
    data1.xd[i].z = device0->mech[i].pos_d.z;
    _ori = &(device0->mech[i].ori_d);

    // CHECK GRASP SKIPPING CONDITION
    // Grasp angle should not be updated unless the angle change is "large"
    if (fabs(data1.rd[i].grasp - _ori->grasp) / 1000 > 45 * d2r) data1.rd[i].grasp = _ori->grasp;

    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) data1.rd[i].R[j][k] = _ori->R[j][k];

    // Set the local quaternion orientation rep.
    if (device0->mech[i].name == green) 
      armidx = 1;
    else if (device0->mech[i].name == gold)
      armidx = 0;
    else continue; //don't do anything for joint encoders or extra arms yet 
    tmpmx.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2], _ori->R[1][0], _ori->R[1][1],
                   _ori->R[1][2], _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
    tmpmx.getRotation(Q_ori[armidx]);


    data1.param_tool_type[i] = device0->mech[i].mech_tool.t_end;
  }
  pthread_mutex_unlock(&data1Mutex);
  isUpdated = TRUE;

  return;
}

void setSurgeonMode(int pedalstate) {
  pthread_mutex_lock(&data1Mutex);
  data1.surgeon_mode = pedalstate;
  pthread_mutex_unlock(&data1Mutex);
  isUpdated = TRUE;
  log_msg("surgeon mode: %d", data1.surgeon_mode);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///
/// PUBLISH ROS DATA
///
#include <raven_2_crtk/raven_state.h>
#include <raven_2_crtk/raven_automove.h>

#include "crtk_io.h"

void publish_joints(robot_device *);
void autoincrCallback(raven_2_crtk::raven_automove);


using namespace raven_2_crtk;


/**
 *  \brief Initiates all ROS publishers and subscribers
 *
 *  Currently advertises ravenstate, joint states, and 2 visualization markers.
 *  Subscribes to automove
 *
 *  \param n the address of a nodeHandle
 * \ingroup ROS
 *  \todo rename this functionto reflect it's current use as a general ROS topic
 *initializer
 */
int init_ravenstate_publishing(robot_device *dev, ros::NodeHandle &n) {
  // Global publisher and subscribers for raven data
  pub_ravenstate = n.advertise<raven_state>(
      "ravenstate", 1);  //, ros::TransportHints().unreliable().tcpNoDelay() );
  joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  sub_automove = n.subscribe<raven_automove>("raven_automove", 1, autoincrCallback
                                             ,ros::TransportHints().unreliable()
                                                                   .reliable());

  //CRTK publishers and subscribers
  sub_crtkCommand = n.subscribe<crtk_msgs::StringStamped>("state_command", 1, &CRTK_state::crtk_cmd_cb
                                             ,&dev->crtk_state);
  sub_crtkCommand_arm1 = n.subscribe<crtk_msgs::StringStamped>("/arm1/state_command", 1, &CRTK_state::crtk_cmd_cb
                                             ,&dev->crtk_state);
  sub_crtkCommand_arm2 = n.subscribe<crtk_msgs::StringStamped>("/arm2/state_command", 1, &CRTK_state::crtk_cmd_cb
                                             ,&dev->crtk_state);
  sub_crtkCommand_arm3 = n.subscribe<crtk_msgs::StringStamped>("/arm3/state_command", 1, &CRTK_state::crtk_cmd_cb
                                             ,&dev->crtk_state);
  sub_crtkCommand_arm4 = n.subscribe<crtk_msgs::StringStamped>("/arm4/state_command", 1, &CRTK_state::crtk_cmd_cb
                                             ,&dev->crtk_state);

  sub_servo_cr_gold = n.subscribe<geometry_msgs::PoseStamped>("/arm1/servo_cr", 1, &CRTK_motion_api::crtk_servo_cr_cb
                                             ,&crtk_motion_api_gold);
  sub_servo_cr_green = n.subscribe<geometry_msgs::PoseStamped>("/arm2/servo_cr", 1, &CRTK_motion_api::crtk_servo_cr_cb
                                             ,&crtk_motion_api_green);
  sub_servo_cr_blue = n.subscribe<geometry_msgs::PoseStamped>("/arm3/servo_cr", 1, &CRTK_motion_api::crtk_servo_cr_cb
                                             ,&crtk_motion_api_blue);
  sub_servo_cr_orange = n.subscribe<geometry_msgs::PoseStamped>("/arm4/servo_cr", 1, &CRTK_motion_api::crtk_servo_cr_cb
                                             ,&crtk_motion_api_orange);


  sub_servo_cp_gold = n.subscribe<geometry_msgs::PoseStamped>("/arm1/servo_cp", 1, &CRTK_motion_api::crtk_servo_cp_cb
                                             ,&crtk_motion_api_gold);
  sub_servo_cp_green = n.subscribe<geometry_msgs::PoseStamped>("/arm2/servo_cp", 1, &CRTK_motion_api::crtk_servo_cp_cb
                                             ,&crtk_motion_api_green);

  sub_servo_cv_gold = n.subscribe<geometry_msgs::TwistStamped>("/arm1/servo_cv", 1,  &CRTK_motion_api::crtk_servo_cv_cb
                                             ,&crtk_motion_api_gold);
  sub_servo_cv_green = n.subscribe<geometry_msgs::TwistStamped>("/arm2/servo_cv", 1, &CRTK_motion_api::crtk_servo_cv_cb
                                             ,&crtk_motion_api_green);
  sub_servo_cv_blue = n.subscribe<geometry_msgs::TwistStamped>("/arm3/servo_cv", 1,  &CRTK_motion_api::crtk_servo_cv_cb
                                             ,&crtk_motion_api_blue);
  sub_servo_cv_orange = n.subscribe<geometry_msgs::TwistStamped>("/arm4/servo_cv", 1, &CRTK_motion_api::crtk_servo_cv_cb
                                             ,&crtk_motion_api_orange);



  sub_servo_jr_gold = n.subscribe<sensor_msgs::JointState>("/arm1/servo_jr", 1, 
     &CRTK_motion_api::crtk_servo_jr_cb, &crtk_motion_api_gold);
  sub_servo_jr_green = n.subscribe<sensor_msgs::JointState>("/arm2/servo_jr", 1, 
     &CRTK_motion_api::crtk_servo_jr_cb, &crtk_motion_api_green);

  sub_servo_jp_gold = n.subscribe<sensor_msgs::JointState>("/arm1/servo_jp", 1, 
     &CRTK_motion_api::crtk_servo_jp_cb, &crtk_motion_api_gold);
  sub_servo_jp_green = n.subscribe<sensor_msgs::JointState>("/arm2/servo_jp", 1, 
     &CRTK_motion_api::crtk_servo_jp_cb, &crtk_motion_api_green);


  sub_servo_jv_gold = n.subscribe<sensor_msgs::JointState>("/arm1/servo_jv", 1, 
     &CRTK_motion_api::crtk_servo_jv_cb, &crtk_motion_api_gold);
  sub_servo_jv_green = n.subscribe<sensor_msgs::JointState>("/arm2/servo_jv", 1, 
     &CRTK_motion_api::crtk_servo_jv_cb, &crtk_motion_api_green);
  sub_servo_jv_blue = n.subscribe<sensor_msgs::JointState>("/arm3/servo_jv", 1, 
     &CRTK_motion_api::crtk_servo_jv_cb, &crtk_motion_api_blue);
  sub_servo_jv_orange = n.subscribe<sensor_msgs::JointState>("/arm4/servo_jv", 1, 
     &CRTK_motion_api::crtk_servo_jv_cb, &crtk_motion_api_orange);


  sub_servo_jv_orange_grasper = n.subscribe<sensor_msgs::JointState>("grasp4/servo_jv", 1, &CRTK_motion_api::crtk_servo_jv_cb
                                               ,&crtk_motion_api_orange_grasp);
  sub_servo_jv_blue_grasper = n.subscribe<sensor_msgs::JointState>("grasp3/servo_jv", 1, &CRTK_motion_api::crtk_servo_jv_cb
                                               ,&crtk_motion_api_blue_grasp);
  sub_servo_jv_green_grasper = n.subscribe<sensor_msgs::JointState>("grasp2/servo_jv", 1, &CRTK_motion_api::crtk_servo_jv_cb
                                           ,&crtk_motion_api_green_grasp);
  sub_servo_jv_gold_grasper = n.subscribe<sensor_msgs::JointState>("grasp1/servo_jv", 1, &CRTK_motion_api::crtk_servo_jv_cb
                                          ,&crtk_motion_api_gold_grasp);

  // sub_servo_jp_green_grasper = n.subscribe<sensor_msgs::JointState>("grasp2/servo_jp", 1, &CRTK_motion_api::crtk_servo_jp_cb
  //                                            ,&crtk_motion_api_green_grasp);

  // sub_servo_jf_green_grasper = n.subscribe<sensor_msgs::JointState>("grasp2/servo_jf", 1, &CRTK_motion_api::crtk_servo_jf_cb
  //                                            ,&crtk_motion_api_green_grasp);
  sub_servo_jr_green_grasper = n.subscribe<sensor_msgs::JointState>("grasp2/servo_jr", 1, &CRTK_motion_api::crtk_servo_jr_cb
                                             ,&crtk_motion_api_green_grasp);
  // sub_servo_jp_gold_grasper = n.subscribe<sensor_msgs::JointState>("grasp1/servo_jp", 1, &CRTK_motion_api::crtk_servo_jp_cb
  //                                            ,&crtk_motion_api_gold_grasp);

  // sub_servo_jf_gold_grasper = n.subscribe<sensor_msgs::JointState>("grasp1/servo_jf", 1, &CRTK_motion_api::crtk_servo_jf_cb
  //                                            ,&crtk_motion_api_gold_grasp);
  sub_servo_jr_gold_grasper = n.subscribe<sensor_msgs::JointState>("grasp1/servo_jr", 1, &CRTK_motion_api::crtk_servo_jr_cb
                                             ,&crtk_motion_api_gold_grasp);
  sub_servo_jp_gold_grasper = n.subscribe<sensor_msgs::JointState>("grasp1/servo_jp", 1, &CRTK_motion_api::crtk_servo_jp_cb
                                             ,&crtk_motion_api_gold_grasp);


  pub_crtk_state              = n.advertise<crtk_msgs::OperatingState>("operating_state", 1);
  pub_crtk_state_arm1         = n.advertise<crtk_msgs::OperatingState>("/arm1/operating_state", 1);
  pub_crtk_state_arm2         = n.advertise<crtk_msgs::OperatingState>("/arm2/operating_state", 1);
  pub_crtk_measured_js_gold   = n.advertise<sensor_msgs::JointState>("arm1/measured_js", 1);
  pub_crtk_measured_js_green  = n.advertise<sensor_msgs::JointState>("arm2/measured_js", 1);
  pub_crtk_measured_cp_gold   = n.advertise<geometry_msgs::PoseStamped>("arm1/measured_cp", 1);
  pub_crtk_measured_cp_green  = n.advertise<geometry_msgs::PoseStamped>("arm2/measured_cp", 1);
  pub_crtk_measured_cv_gold   = n.advertise<geometry_msgs::TwistStamped>("arm1/measured_cv", 1);
  pub_crtk_measured_cv_green  = n.advertise<geometry_msgs::TwistStamped>("arm2/measured_cv", 1);

  pub_crtk_setpoint_js_gold   = n.advertise<sensor_msgs::JointState>("arm1/setpoint_js", 1);
  pub_crtk_setpoint_js_green  = n.advertise<sensor_msgs::JointState>("arm2/setpoint_js", 1);
  pub_crtk_setpoint_cp_gold   = n.advertise<geometry_msgs::PoseStamped>("arm1/setpoint_cp", 1);
  pub_crtk_setpoint_cp_green  = n.advertise<geometry_msgs::PoseStamped>("arm2/setpoint_cp", 1);
  pub_crtk_setpoint_cv_gold   = n.advertise<geometry_msgs::TwistStamped>("arm1/setpoint_cv", 1);
  pub_crtk_setpoint_cv_green  = n.advertise<geometry_msgs::TwistStamped>("arm2/setpoint_cv", 1);

  pub_crtk_measured_js_gold_grasper  = n.advertise<sensor_msgs::JointState>("grasp1/measured_js", 1);
  pub_crtk_measured_js_green_grasper = n.advertise<sensor_msgs::JointState>("grasp2/measured_js", 1);
  pub_crtk_setpoint_js_gold_grasper  = n.advertise<sensor_msgs::JointState>("grasp1/setpoint_js", 1);
  pub_crtk_setpoint_js_green_grasper = n.advertise<sensor_msgs::JointState>("grasp2/setpoint_js", 1);

  return 0;
}


/**
 * @brief      Publishes all crtk stuff
 *
 * @param      device0  The device 0
 */
void publish_crtk(robot_device *device0){
    publish_crtk_state(device0);
    publish_crtk_state_arms(device0);
    publish_crtk_measured_js(device0);
    publish_crtk_setpoint_js(device0);
    publish_crtk_measured_cp(device0);
    publish_crtk_setpoint_cp(device0);
    publish_crtk_measured_gr_js(device0);
}


/**
 *\brief Callback for the automove topic - Updates the data1 structure
 *
 * Callback for the automove topic. Updates the data1 structure with the
 *information from the
 * ROS topic. Properly locks the data1 mutex. Accepts cartesian or quaternion
 *increments.
 *
 * \param msg the
 * \ingroup ROS
 *
 */
void autoincrCallback(raven_2_crtk::raven_automove msg) {
  tf::Transform in_incr[2];
  tf::transformMsgToTF(msg.tf_incr[0], in_incr[0]);
  tf::transformMsgToTF(msg.tf_incr[1], in_incr[1]);

  pthread_mutex_lock(&data1Mutex);

  // this function wants to loop through boards instead of mechanisms
  // TODO:: loop over mechanisms instead?
  int loops = USBBoards.activeAtStart;
  int armidx;

  for (int i = 0; i < loops; i++) {
    if (USBBoards.boards[i] == GOLD_ARM_SERIAL) {
      armidx = 0;
    } else if (USBBoards.boards[i] == GREEN_ARM_SERIAL) {
      armidx = 1;
    } else if ((USBBoards.boards[i] == JOINT_ENC_SERIAL) 
              || (USBBoards.boards[i] == BLUE_ARM_SERIAL)
              || (USBBoards.boards[i] == ORANGE_ARM_SERIAL)
              || (USBBoards.boards[i] == JOINT_ENC_SERIAL_2)) {
      continue;  // don't do any teleop data for joint encoders
                 // or extra arms yet
    }

    // add position increment
    tf::Vector3 tmpvec = in_incr[armidx].getOrigin();
    data1.xd[armidx].x += int(tmpvec[0]);
    data1.xd[armidx].y += int(tmpvec[1]);
    data1.xd[armidx].z += int(tmpvec[2]);

    // add rotation increment
    tf::Quaternion q_temp(in_incr[armidx].getRotation());
    if (q_temp != tf::Quaternion::getIdentity()) {
      Q_ori[armidx] = q_temp * Q_ori[armidx];
      tf::Matrix3x3 rot_mx_temp(Q_ori[armidx]);
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) data1.rd[armidx].R[j][k] = rot_mx_temp[j][k];
    }
  }

  pthread_mutex_unlock(&data1Mutex);
  isUpdated = TRUE;
}


void update_device_motion_api(CRTK_motion_planner* planner){

  // TODO: investigate locking the global variables
  // planner->crtk_motion_api_prev[0].transfer_data(&planner->crtk_motion_api[0]);
  // planner->crtk_motion_api_prev[1].transfer_data(&planner->crtk_motion_api[1]);

  planner->crtk_motion_api[0].transfer_data(&crtk_motion_api_gold);
  planner->crtk_motion_api[1].transfer_data(&crtk_motion_api_green);

  planner->crtk_motion_api_grasp[0].transfer_data(&crtk_motion_api_gold_grasp);
  planner->crtk_motion_api_grasp[1].transfer_data(&crtk_motion_api_green_grasp);

  return;
}

/**
 * @brief      Publishes the raven_state message from the robot and currParams
 *             structures
 *
 * @param      dev         robot device structure with the current state of the
 *                         robot
 * @param      currParams  the parameters being passed from the interfaces
 * @ingroup    ROS
 */
void publish_ravenstate_ros(robot_device *dev, param_pass *currParams) {
  static int count = 0;
  static raven_state msg_ravenstate;  // satic variables to minimize memory allocation calls
  static ros::Time t1;
  static ros::Time t2;
  static ros::Duration d;

  msg_ravenstate.last_seq = currParams->last_sequence;

  if (count == 0) {
    t1 = t1.now();
  }
  count++;
  t2 = t2.now();
  d = t2 - t1;

  //    if (d.toSec()<0.01)
  //        return;

  msg_ravenstate.dt = d;
  t1 = t2;

  publish_joints(dev);

  // Copy the robot state to the output datastructure.
  int numdof = 8;
  int j;
  for (int i = 0; i < NUM_MECH; i++) {
    j = dev->mech[i].type == GREEN_ARM ? 1 : 0;
    msg_ravenstate.type[j] = dev->mech[j].type;
    msg_ravenstate.pos[j * 3] = dev->mech[j].pos.x;
    msg_ravenstate.pos[j * 3 + 1] = dev->mech[j].pos.y;
    msg_ravenstate.pos[j * 3 + 2] = dev->mech[j].pos.z;
    msg_ravenstate.pos_d[j * 3] = dev->mech[j].pos_d.x;
    msg_ravenstate.pos_d[j * 3 + 1] = dev->mech[j].pos_d.y;
    msg_ravenstate.pos_d[j * 3 + 2] = dev->mech[j].pos_d.z;
    msg_ravenstate.grasp_d[j] = (float)dev->mech[j].ori_d.grasp / 1000;

    for (int orii = 0; orii < 3; orii++) {
      for (int orij = 0; orij < 3; orij++) {
        msg_ravenstate.ori[j * 9 + orii * 3 + orij] = dev->mech[j].ori.R[orii][orij];
        msg_ravenstate.ori_d[j * 9 + orii * 3 + orij] = dev->mech[j].ori_d.R[orii][orij];
      }
    }

    // TODO: WHY IS THIS IN DEGREES!? Like --- why?
    for (int m = 0; m < numdof; m++) {
      int jtype = dev->mech[j].joint[m].type;
      msg_ravenstate.encVals[jtype] = dev->mech[j].joint[m].enc_val;
      msg_ravenstate.tau[jtype] = dev->mech[j].joint[m].tau_d;
      msg_ravenstate.mpos[jtype] = dev->mech[j].joint[m].mpos RAD2DEG;
      msg_ravenstate.jpos[jtype] = dev->mech[j].joint[m].jpos RAD2DEG;
      msg_ravenstate.mvel[jtype] = dev->mech[j].joint[m].mvel RAD2DEG;
      msg_ravenstate.jvel[jtype] = dev->mech[j].joint[m].jvel RAD2DEG;
      msg_ravenstate.jpos_d[jtype] = dev->mech[j].joint[m].jpos_d RAD2DEG;
      msg_ravenstate.mpos_d[jtype] = dev->mech[j].joint[m].mpos_d RAD2DEG;
      msg_ravenstate.encoffsets[jtype] = dev->mech[j].joint[m].enc_offset;
      msg_ravenstate.dac_val[jtype] = dev->mech[j].joint[m].current_cmd;
    }

    // grab jacobian velocities and forces
    float vel[6];
    float f[6];
    if (dev->mech[i].name == green) 
      j = 1;
    else if (dev->mech[i].name == gold)
      j = 0;
    else continue; //don't do anything for joint encoders or extra arms yet 
    // j = dev->mech[i].name == green ? 1 : 0;
    dev->mech[j].r2_jac.get_vel(vel);
    dev->mech[j].r2_jac.get_vel(f);
    for (int k = 0; k < 6; k++) {
      msg_ravenstate.jac_vel[j * 6 + k] = vel[k];
      msg_ravenstate.jac_f[j * 6 + k] = f[k];
    }
  }
  //    msg_ravenstate.f_secs = d.toSec();
  msg_ravenstate.hdr.stamp = msg_ravenstate.hdr.stamp.now();

  msg_ravenstate.runlevel = currParams->runlevel;
  msg_ravenstate.sublevel = robot_ready(dev); // homed

  // Publish the raven data to ROS
  pub_ravenstate.publish(msg_ravenstate);
}

/**
 * @brief      Publishes the CRTK state message from the robot
 *
 *             The robot is in the CRTK disabled state if it is (e-stopped & not
 *             homed & no fault). This is the initial state of the robot.
 *
 * @param      dev   robot device structure with the current state of the robot
 * @param      currParams  the parameters being passed from the interfaces
 * @ingroup    ROS
 * @ingroup    CRTK
 */
void publish_crtk_state(robot_device *dev) {
  static crtk_msgs::OperatingState msg_state;

  msg_state.state = dev->crtk_state.get_state_string();
  msg_state.is_homed = dev->crtk_state.get_homed();
  msg_state.is_busy = dev->crtk_state.get_busy();

  msg_state.header.stamp = msg_state.header.stamp.now();

  pub_crtk_state.publish(msg_state);
}

/**
 * @brief      Publishes the CRTK state message from the robot arm 1
 *
 *             The robot is in the CRTK disabled state if it is (e-stopped & not
 *             homed & no fault). This is the initial state of the robot.
 *
 * @param      dev   robot device structure with the current state of the robot
 * @param      currParams  the parameters being passed from the interfaces
 * @ingroup    ROS
 * @ingroup    CRTK
 */
void publish_crtk_state_arms(robot_device *dev) {
  static crtk_msgs::OperatingState msg_state;

  msg_state.state = dev->crtk_state.get_state_string();
  msg_state.is_homed = dev->crtk_state.get_homed();
  msg_state.is_busy = dev->crtk_state.get_busy();

  msg_state.header.stamp = msg_state.header.stamp.now();

  pub_crtk_state_arm1.publish(msg_state);
  pub_crtk_state_arm2.publish(msg_state);
}




void publish_crtk_measured_gr_js(robot_device *dev){
// pub_crtk_measured_js_gold_grasper
  sensor_msgs::JointState msg1, msg2;
// Header header
// string[] name
// float64[] position
// float64[] velocity
// float64[] effort
  msg1.header.stamp = msg1.header.stamp.now();
  msg2.header.stamp = msg2.header.stamp.now();

  float g_angle1 = dev->mech[0].joint[GRASP2].jpos + dev->mech[0].joint[GRASP1].jpos;
  float g_angle2 = dev->mech[1].joint[GRASP2].jpos + dev->mech[1].joint[GRASP1].jpos;

  float vel1 = 0, vel2 = 0;
  float eff1 = 0, eff2 = 0;

  std::string name1[3] = {"gold_grasp","gold_jaw1","gold_jaw2"};
  std::string name2[3] = {"green_grasp","green_jaw1","green_jaw2"};

  msg1.position.push_back(g_angle1);
  msg1.position.push_back(dev->mech[0].joint[GRASP1].jpos);
  msg1.position.push_back(dev->mech[0].joint[GRASP2].jpos);
  msg1.velocity.push_back(vel1);
  msg1.effort.push_back(eff1);
  for(int i=0; i<3; i++)
    msg1.name.push_back(name1[i]);

  msg2.position.push_back(g_angle2);
  msg2.position.push_back(dev->mech[1].joint[GRASP1].jpos);
  msg2.position.push_back(dev->mech[1].joint[GRASP2].jpos);
  msg2.velocity.push_back(vel2);
  msg2.effort.push_back(eff2);
  for(int i=0; i<3; i++)
    msg2.name.push_back(name2[i]);


  pub_crtk_measured_js_gold_grasper.publish(msg1);
  pub_crtk_measured_js_green_grasper.publish(msg2);
}

/**
 * @brief      retrieves joint states from dev and publishes
 *
 * @param      dev   The robot device to get joint info from
 */
void publish_crtk_measured_js(robot_device *dev) {
  sensor_msgs::JointState msg1, msg2;
// Header header
// string[] name
// float64[] position
// float64[] velocity
// float64[] effort
// 
  static int count;
  static int once = 1;
  msg1.header.stamp = msg1.header.stamp.now();
  msg2.header.stamp = msg2.header.stamp.now();

  std::string gold_names[7] = {"gold_shoulder","gold_elbow","gold_ins","gold_roll","gold_wrist","gold_jaw1","gold_jaw2"};
  std::string green_names[7] = {"green_shoulder","green_elbow","green_ins","green_roll","green_wrist","green_jaw1","green_jaw2"};

  mechanism *_mech = NULL;
  DOF *_joint = NULL;
  int i, j;
  while (loop_over_7_joints(dev, _mech, _joint, i, j)) {
    if (_mech->name == gold) {

        msg1.position.push_back(_joint->jpos);
        msg1.velocity.push_back(_joint->jvel);
        msg1.effort.push_back(_joint->tau);
        msg1.name.push_back(gold_names[j]);
    }
    else if (_mech->name == green) {

        msg2.position.push_back(_joint->jpos);
        msg2.velocity.push_back(_joint->jvel);
        msg2.effort.push_back(_joint->tau);
        msg2.name.push_back(green_names[j]);

    }
  }
  pub_crtk_measured_js_gold.publish(msg1);
  pub_crtk_measured_js_green.publish(msg2);

  count++;
}

/**
 * @brief      publishes the robot's current perceived cartesian position
 *
 * @param      dev   the robot device
 */
void publish_crtk_measured_cp(robot_device *dev) {
  static geometry_msgs::PoseStamped msg1, msg2;
// Header header
// string child_frame_id # the frame id of the child frame
// Transform transform
  msg1.header.stamp = msg1.header.stamp.now();
  msg2.header.stamp = msg2.header.stamp.now();

  // static int counter = 0;
  // tf::Quaternion curr_quat = dev->crtk_motion_planner.crtk_motion_api[1].get_pos().getRotation();
  // if(counter % 100 == 0) ROS_INFO("pub_norm: %f", sqrt(pow(curr_quat.x(),2)+pow(curr_quat.y(),2)+pow(curr_quat.z(),2)+pow(curr_quat.w(),2)));
  // counter ++;

  tf::Transform pos1 = dev->crtk_motion_planner.crtk_motion_api[0].get_pos();
  tf::Transform pos2 = dev->crtk_motion_planner.crtk_motion_api[1].get_pos();

  tf::Quaternion norm1 = pos1.getRotation().normalized();
  tf::Quaternion norm2 = pos2.getRotation().normalized();

  pos1.setRotation(norm1); 
  pos2.setRotation(norm2);

  tf::Pose pose1;
  tf::Pose pose2;

  tf_to_pose(pos1, pose1);
  tf_to_pose(pos2, pose2);

  tf::poseTFToMsg(pose1,msg1.pose);
  tf::poseTFToMsg(pose2,msg2.pose);
  pub_crtk_measured_cp_gold.publish(msg1);
  pub_crtk_measured_cp_green.publish(msg2);
}

void publish_crtk_measured_cv(robot_device *dev) {
  static geometry_msgs::TwistStamped msg1, msg2;
// Header header
// Twist twist
  msg1.header.stamp = msg1.header.stamp.now();
  msg2.header.stamp = msg2.header.stamp.now();
  
  pub_crtk_measured_cv_gold.publish(msg1);
  pub_crtk_measured_cv_green.publish(msg2);
}

void publish_crtk_setpoint_js(robot_device *dev) {
  sensor_msgs::JointState msg1, msg2;
// Header header

// string[] name
// float64[] position
// float64[] velocity
// float64[] effort

  msg1 = crtk_motion_api_gold.get_setpoint_out_js();
  msg2 = crtk_motion_api_green.get_setpoint_out_js();

  msg1.header.stamp = msg1.header.stamp.now();
  msg2.header.stamp = msg2.header.stamp.now();

  std::string gold_names[7] = {"gold_shoulder","gold_elbow","gold_ins","gold_roll","gold_wrist","gold_jaw1","gold_jaw2"};
  std::string green_names[7] = {"green_shoulder","green_elbow","green_ins","green_roll","green_wrist","green_jaw1","green_jaw2"};

  for(int i=0; i<7; i++){
    msg1.name.push_back(gold_names[i]);
    msg2.name.push_back(green_names[i]);   
  }

  pub_crtk_setpoint_js_gold.publish(msg1);
  pub_crtk_setpoint_js_green.publish(msg2);
}

void publish_crtk_setpoint_cp(robot_device *dev) {
  static geometry_msgs::PoseStamped msg1, msg2;
// Header header
// string child_frame_id # the frame id of the child frame
// Transform transform
  tf::Transform trans1, trans2;
  msg1.header.stamp = msg1.header.stamp.now();
  msg2.header.stamp = msg2.header.stamp.now();

  tf::Vector3 trans_pos1 = tf::Vector3(dev->mech[0].pos_d.x,dev->mech[0].pos_d.y,dev->mech[0].pos_d.z);
  tf::Vector3 trans_pos2 = tf::Vector3(dev->mech[1].pos_d.x,dev->mech[1].pos_d.y,dev->mech[1].pos_d.z);

  trans1.setOrigin(trans_pos1 / (1 M_2_MICRON));
  trans2.setOrigin(trans_pos2 / (1 M_2_MICRON));

  float ori1[9], ori2[9];
  for (int orii = 0; orii < 3; orii++) {
    for (int orij = 0; orij < 3; orij++) {
      ori1[orii*3 + orij] = dev->mech[0].ori_d.R[orii][orij];
      ori2[orii*3 + orij] = dev->mech[1].ori_d.R[orii][orij];
    }
  }
  trans1.setBasis(tf::Matrix3x3(ori1[0],ori1[1],ori1[2],ori1[3],ori1[4],ori1[5],ori1[6],ori1[7],ori1[8]));
  trans2.setBasis(tf::Matrix3x3(ori2[0],ori2[1],ori2[2],ori2[3],ori2[4],ori2[5],ori2[6],ori2[7],ori2[8]));

  tf::Quaternion norm1 = trans1.getRotation().normalized();
  tf::Quaternion norm2 = trans2.getRotation().normalized();

  trans1.setRotation(norm1); 
  trans2.setRotation(norm2);

  tf::Pose pose1;
  tf::Pose pose2;

  tf_to_pose(trans1, pose1);
  tf_to_pose(trans2, pose2);

  tf::poseTFToMsg(pose1,msg1.pose);
  tf::poseTFToMsg(pose2,msg2.pose);
  
  pub_crtk_setpoint_cp_gold.publish(msg1);
  pub_crtk_setpoint_cp_green.publish(msg2);
}

void publish_crtk_setpoint_cv(robot_device *dev) {
  static geometry_msgs::TwistStamped msg1, msg2;
// Header header
// Twist twist
  msg1.header.stamp = msg1.header.stamp.now();
  msg2.header.stamp = msg2.header.stamp.now();

  pub_crtk_setpoint_cv_gold.publish(msg1);
  pub_crtk_setpoint_cv_green.publish(msg2);
}

/**
 *  \brief Publishes the joint angles for the visualization
 *
 *  \param device0 the robot and its state
 *
 *  \ingroup ROS
 *
 */
void publish_joints(robot_device *device0) {
  static int count = 0;
  static ros::Time t1;
  static ros::Time t2;
  static ros::Duration d;

  if (count == 0) {
    t1 = t1.now();
  }
  count++;
  t2 = t2.now();
  d = t2 - t1;

  if (d.toSec() < 0.030) return;
  t1 = t2;

  sensor_msgs::JointState joint_state;
  // update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(28);
  joint_state.position.resize(28);
  //    joint_state.name.resize(14);
  //    joint_state.position.resize(14);
  int left, right;
  if (device0->mech[0].name == gold) {
    left = 0;
    right = 1;
  } else {
    left = 1;
    right = 0;
  }
  //======================LEFT ARM===========================
  joint_state.name[0] = "shoulder_L";
  joint_state.position[0] = device0->mech[left].joint[0].jpos + offsets_l.shoulder_off;
  joint_state.name[1] = "elbow_L";
  joint_state.position[1] = device0->mech[left].joint[1].jpos + offsets_l.elbow_off;
  joint_state.name[2] = "insertion_L";
  joint_state.position[2] = device0->mech[left].joint[2].jpos + d4 + offsets_l.insertion_off;
  joint_state.name[3] = "tool_roll_L";
  joint_state.position[3] = device0->mech[left].joint[4].jpos - 45 * d2r + offsets_l.roll_off;
  joint_state.name[4] = "wrist_joint_L";
  joint_state.position[4] = device0->mech[left].joint[5].jpos + offsets_l.wrist_off;
  joint_state.name[5] = "grasper_joint_1_L";
  joint_state.position[5] = device0->mech[left].joint[6].jpos + offsets_l.grasp1_off;
  joint_state.name[6] = "grasper_joint_2_L";
  joint_state.position[6] = device0->mech[left].joint[7].jpos * -1 + offsets_l.grasp2_off;

  //======================RIGHT ARM===========================
  joint_state.name[7] = "shoulder_R";
  joint_state.position[7] = device0->mech[right].joint[0].jpos + offsets_r.shoulder_off;
  joint_state.name[8] = "elbow_R";
  joint_state.position[8] = device0->mech[right].joint[1].jpos + offsets_r.elbow_off;
  joint_state.name[9] = "insertion_R";
  joint_state.position[9] = device0->mech[right].joint[2].jpos + d4 + offsets_r.insertion_off;
  joint_state.name[10] = "tool_roll_R";
  joint_state.position[10] = device0->mech[right].joint[4].jpos + 45 * d2r + offsets_r.roll_off;
  joint_state.name[11] = "wrist_joint_R";
  joint_state.position[11] = device0->mech[right].joint[5].jpos * -1 + offsets_r.wrist_off;
  joint_state.name[12] = "grasper_joint_1_R";
  joint_state.position[12] = device0->mech[right].joint[6].jpos + offsets_r.grasp1_off;
  joint_state.name[13] = "grasper_joint_2_R";
  joint_state.position[13] = device0->mech[right].joint[7].jpos * -1 + offsets_r.grasp2_off;

  //======================LEFT ARM===========================

  joint_state.name[14] = "shoulder_L2";
  joint_state.position[14] = device0->mech[left].joint[0].jpos_d + offsets_l.shoulder_off;
  joint_state.name[15] = "elbow_L2";
  joint_state.position[15] = device0->mech[left].joint[1].jpos_d + offsets_l.elbow_off;
  joint_state.name[16] = "insertion_L2";
  joint_state.position[16] = device0->mech[left].joint[2].jpos_d + d4 + offsets_l.insertion_off;
  joint_state.name[17] = "tool_roll_L2";
  joint_state.position[17] = device0->mech[left].joint[4].jpos_d - 45 * d2r + offsets_l.roll_off;
  joint_state.name[18] = "wrist_joint_L2";
  joint_state.position[18] = device0->mech[left].joint[5].jpos_d + offsets_l.wrist_off;
  joint_state.name[19] = "grasper_joint_1_L2";
  joint_state.position[19] = device0->mech[left].joint[6].jpos_d + offsets_l.grasp1_off;
  joint_state.name[20] = "grasper_joint_2_L2";
  joint_state.position[20] = device0->mech[left].joint[7].jpos_d * -1 + offsets_l.grasp2_off;

  //======================RIGHT ARM===========================
  joint_state.name[21] = "shoulder_R2";
  joint_state.position[21] = device0->mech[right].joint[0].jpos_d + offsets_r.shoulder_off;
  joint_state.name[22] = "elbow_R2";
  joint_state.position[22] = device0->mech[right].joint[1].jpos_d + offsets_r.elbow_off;
  joint_state.name[23] = "insertion_R2";
  joint_state.position[23] = device0->mech[right].joint[2].jpos_d + d4 + offsets_r.insertion_off;
  joint_state.name[24] = "tool_roll_R2";
  joint_state.position[24] = device0->mech[right].joint[4].jpos_d + 45 * d2r + offsets_r.roll_off;
  joint_state.name[25] = "wrist_joint_R2";
  joint_state.position[25] = device0->mech[right].joint[5].jpos_d * -1 + offsets_r.wrist_off;
  joint_state.name[26] = "grasper_joint_1_R2";
  joint_state.position[26] = device0->mech[right].joint[6].jpos_d + offsets_r.grasp1_off;
  joint_state.name[27] = "grasper_joint_2_R2";
  joint_state.position[27] = device0->mech[right].joint[7].jpos_d * -1 + offsets_r.grasp2_off;

  // Publish the joint states
  joint_publisher.publish(joint_state);
}
