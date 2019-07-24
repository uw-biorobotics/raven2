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

/*********************************************
 **
 **
 **  DS0.h
 **
 **	DS0.h will describe the device/mechanism/DOF
 ** data structures that will
 ** 1) hold all data associated with a device and
 ** 2) be "sampled" and passed to user-space for display and logging.
 **
 **    Devices contain mechanisms, mechanisms contain DOFs
 **
 **
 *********************************************/

#ifndef DS0_H
#define DS0_H

#include "tools.h"
#include "r2_jacobian.h"
#include "crtk_state.h"
#include "crtk_motion_api.h"
#include "crtk_motion_planner.h"


//#define NUM_MECH 2
#define MAX_MECH 4
#define MAX_DOF_PER_MECH 8
#define MAX_MECH_PER_DEV 4

#define STATE_OFF 0
#define STATE_UNINIT 1
#define STATE_READY 2
#define STATE_I_OVERLOAD 3

// for our coding convenience on ix86 platforms:
typedef int s_24;
typedef short int s_16;
typedef unsigned char u_08;
typedef unsigned short int u_16;
typedef unsigned int u_24;
typedef unsigned int u_32;
typedef unsigned long long int u_64;

/********************************************************
 *
 *  Structs for Cartesian values (formerly cartvals)
 *
 */
struct position {
  int x;  // X coordinate
  int y;  // Y coordinate
  int z;  // Z coordiante
};

// NOTE: R_II uses R[][].  R_I uses RPY.
// TODO: Change to tf::Quaterion
// TODO: Better yet, get rid of these structs and just use a tf::Transform
struct orientation {
  float R[3][3];  // 3x3 Rotation Matrix (Dimensionless)
  int yaw;        // orientation expressed in XYZ Fixed frame notation
  int pitch;
  int roll;
  int grasp;  // integer.  Keep in miliradians
};

enum jointState {
  jstate_not_ready = 0,
  jstate_pos_unknown = 1,
  jstate_homing1 = 2,
  jstate_homing2 = 3,
  jstate_ready = 4,
  jstate_wait = 5,
  jstate_hard_stop = 6,
  jstate_last_type
};


enum mechName {
  gold,
  green,
  blue,
  orange
};

/*************************************************************************
 *
 *  Degree of Freedom Struct
 *      One of these for each mechanical Degree of Freedom
 *
 */
struct DOF {
  u_16 type;
  jointState state;    // is this DoF enabled?
  s_24 enc_val;        // encoder value
  s_24 joint_enc_val;  // Joint encoder value
  s_16 current_cmd;    // DAC command to achieve tau at actuator
  int homing_dac;  // DAC value that indicates joint hard stop
  float jpos;          // actual DOF coordinate (rad)
  float j_enc_pos;     // actual DOF coordinate of Joint encoder (rad)
  float mpos;
  //  float jpos_old;       // previous DOF coordinate (rad)
  //  float mpos_old;
  float jvel;  // actual DOF velocity(q-dot)
  float mvel;
  float tau;     // actual DOF force/torque at joint
  float tau_d;   // desired DOF force/torque at motor capstan after gearbox
  float tau_g;   // Estimated gravity force/torque on joint.
  float jpos_d;  // desired DOF coordinate (rad)
  float mpos_d;
  float jpos_d_old;  // previous desired DOF coordinate (rad)
  float mpos_d_old;
  float jvel_d;  // desired DOF velocity (q-dot-desired)
  float mvel_d;
  int enc_offset;        // Encoder offset to "zero"
  int joint_enc_offset;  // Joint Encoder offset to "zero"
  float perror_int;      // integrated position error for joint space position control
};


/********************************************************
 *
 *  mechanism Struct
 *
 */
struct mechanism {
  u_16 type;
  u_08 name;
  u_16 serial;
  tool mech_tool;
  position pos;
  position pos_d;
  position base_pos;          // base position in world frame
  orientation ori;
  orientation ori_d;
  orientation base_ori;       // base orientation in world frame
  DOF joint[MAX_DOF_PER_MECH];
  u_08 inputs;                // input pins
  u_08 outputs;               // output pins
  r2_jacobian r2_jac;         // class needed to avoid build error about forward declarations
  float teleop_transform[4];  // quaternion rotation between 0 and desired output frame
  bool joint_control;
};

/********************************************************
 *
 *   device Struct
 *
 */
struct robot_device {
  u_08 num_mechs;
  u_16 type;
  u_32 timestamp;    // time of last update
  u_08 runlevel;     // nothing/init/joints/kinematics/e-stop
  u_08 sublevel;     // which experimental mode are we running
  int surgeon_mode;  // Clutching/indexing state - 1==engaged; 0==disengaged
  mechanism mech[MAX_MECH_PER_DEV];
  float grav_mag;     // gravity magnitude
  position grav_dir;  // gravity direction
  CRTK_state crtk_state; //class to hold robot status flags for CRTK API
  CRTK_motion_planner crtk_motion_planner;
  char robot_homed;   // have all joints been homed
  char robot_fault;   //has an e-stop been triggered?
};

typedef robot_device device;

#endif
