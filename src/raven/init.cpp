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

/**	\file 	init.cpp
*
*	\brief 	contains functions for initializing the robot
* 	       	intializes the DOF structure AND runs initialization routine
*
* 	\fn These are the 4 functions in init.cpp file.
*           Functions marked with "*" are called explicitly from other files.
* 	       *(1) initRobotData	 	:uses (2)(4)
*       	(2) intDOFs
* 	       *(3) init_ravengains
*		(4) setStartXYZ			:uses fwd_cable_coupling.cpp
*(1), r2_kinematics.cpp (2), local_io.cpp (6)
*
*  	\date 	7/29/2005
*
*  	\author Hawkeye King
*/

#include <ros/console.h>

#include "init.h"
#include "USB_init.h"
#include "local_io.h"

#ifdef DV_ADAPTER
const e_tool_type use_tool = dv_adapter;
#else

#ifdef RAVEN_TOOLS
const e_tool_type use_tool = TOOL_GRASPER_10MM;  //
#else

#ifdef KIST
const e_tool_type use_tool = ricks_tools_type;

#endif
#endif
#endif

extern tool gold_arm_tool;
extern tool green_arm_tool;

extern int initialized;

extern DOF_type DOF_types[];

extern char usb_board_count;
extern USBStruct USBBoards;
extern int NUM_MECH;
extern int soft_estopped;

/**\fn void initRobotData (device *device0, int runlevel, param_pass
  *currParams)
  \brief This function initializes the robot data
  \struct device
  \struct param_pass
  \param device0 pointer to device struct
  \param runlevel
  \param currParams pointer to param struct containing current params
  \return
  \ingroup  DataStructures
*/
void initRobotData(device *device0, int runlevel, param_pass *currParams) {
  // init_wait_loop is a klugy way to wait a few times through the loop for our
  // kinematics to propogate.
  static int init_wait_loop = 0;

  // initialize gravity direction data
  if (!initialized) {
    currParams->grav_dir.x = 0;
    currParams->grav_dir.y = 0;
    currParams->grav_dir.z = 980;
    currParams->grav_mag = 9.8;

    device0->grav_dir.x = 0;
    device0->grav_dir.y = 0;
    device0->grav_dir.z = 980;
    device0->grav_mag = 9.8;
  }

  // In ESTOP reset initialization
  if (runlevel == RL_E_STOP) initialized = FALSE;

  if (soft_estopped) device0->mech[0].joint[0].state = jstate_pos_unknown;

  // Do nothing if we are not in the init runlevel
  if (runlevel != RL_INIT) return;

  switch (currParams->sublevel) {
    case 0: {
      currParams->sublevel = 1;  // Goto sublevel 1 to allow initial jpos_d setup by inv_kin.
      break;
    }
    case 1:             // Initialization off all joint variables
      if (initialized)  // If already initialized do nothing
      {
        break;
      }

      initDOFs(device0);
      setStartXYZ(device0);  // Set pos_d = current position

      currParams->sublevel = 2;  // Goto sublevel 1 to allow initial jpos_d setup by inv_kin.
      log_msg("    -> sublevel %d", currParams->sublevel);
      break;

    case 2:
      // Automatically jump to next sublevel after small delay
      init_wait_loop++;
      setStartXYZ(device0);  // set cartesian pos_d = current position

      if (init_wait_loop > 10) {
        // Go to auto init sublevel
        currParams->sublevel = SL_AUTO_INIT;
        init_wait_loop = 0;
      }
      break;

    case SL_AUTO_INIT:
      initialized = TRUE;  // Set initialized flag
  }

  return;
}

/**\fn void initDOFs(device *device0)
 \brief This function intializes all structures which are not DOF specific
 \struct device
 \param device0 pointer to device struct
 \return
 \ingroup  DataStructures
 \todo Add runtime config file (or use ROS parameter server) to elminate
 square/diamond #ifdefs. And support new tool types

*/
void initDOFs(device *device0) {
  static int dofs_inited = 0;
  if (dofs_inited) return;
  /// Set transmission ratios
  //    Yes, the numbering is weird (TOOL_ROT and Z_INS are physically 4th & 3rd
  //    respectively
  //    See defines.h for explanation
  DOF_types[SHOULDER_GOLD].TR = SHOULDER_TR_GOLD_ARM;
  DOF_types[ELBOW_GOLD].TR = ELBOW_TR_GOLD_ARM;
  DOF_types[Z_INS_GOLD].TR = Z_INS_TR_GOLD_ARM;
  DOF_types[TOOL_ROT_GOLD].TR = TOOL_ROT_TR_GOLD_ARM;
  DOF_types[WRIST_GOLD].TR = WRIST_TR_GOLD_ARM;
  DOF_types[GRASP1_GOLD].TR = GRASP1_TR_GOLD_ARM;
  DOF_types[GRASP2_GOLD].TR = GRASP2_TR_GOLD_ARM;

  DOF_types[SHOULDER_GREEN].TR = SHOULDER_TR_GREEN_ARM;
  DOF_types[ELBOW_GREEN].TR = ELBOW_TR_GREEN_ARM;
  DOF_types[Z_INS_GREEN].TR = Z_INS_TR_GREEN_ARM;
  DOF_types[TOOL_ROT_GREEN].TR = TOOL_ROT_TR_GREEN_ARM;
  DOF_types[WRIST_GREEN].TR = WRIST_TR_GREEN_ARM;
  DOF_types[GRASP1_GREEN].TR = GRASP1_TR_GOLD_ARM;
  DOF_types[GRASP2_GREEN].TR = GRASP2_TR_GREEN_ARM;

  /// Initialize current limits
  if (SAFETY_LEVEL == BEGINNER_MODE) {
    DOF_types[SHOULDER_GOLD].DAC_max = BEGINNER_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_GREEN].DAC_max = BEGINNER_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_GOLD].DAC_max = BEGINNER_ELBOW_MAX_DAC;
    DOF_types[ELBOW_GREEN].DAC_max = BEGINNER_ELBOW_MAX_DAC;
    DOF_types[Z_INS_GOLD].DAC_max = BEGINNER_Z_INS_MAX_DAC;
    DOF_types[Z_INS_GREEN].DAC_max = BEGINNER_Z_INS_MAX_DAC;
  } else if (SAFETY_LEVEL == MODERATE_MODE) {
    DOF_types[SHOULDER_GOLD].DAC_max = MODERATE_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_GREEN].DAC_max = MODERATE_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_GOLD].DAC_max = MODERATE_ELBOW_MAX_DAC;
    DOF_types[ELBOW_GREEN].DAC_max = MODERATE_ELBOW_MAX_DAC;
    DOF_types[Z_INS_GOLD].DAC_max = MODERATE_Z_INS_MAX_DAC;
    DOF_types[Z_INS_GREEN].DAC_max = MODERATE_Z_INS_MAX_DAC;
  } else  // SAFETY_LEVEL == ADVANCED_MODE
  {
    DOF_types[SHOULDER_GOLD].DAC_max = ADVANCED_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_GREEN].DAC_max = ADVANCED_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_GOLD].DAC_max = ADVANCED_ELBOW_MAX_DAC;
    DOF_types[ELBOW_GREEN].DAC_max = ADVANCED_ELBOW_MAX_DAC;
    DOF_types[Z_INS_GOLD].DAC_max = ADVANCED_Z_INS_MAX_DAC;
    DOF_types[Z_INS_GREEN].DAC_max = ADVANCED_Z_INS_MAX_DAC;
  }

  /*
  //This is the original code
  DOF_types[SHOULDER_GOLD].DAC_max  = SHOULDER_MAX_DAC;
  DOF_types[SHOULDER_GREEN].DAC_max = SHOULDER_MAX_DAC;
  DOF_types[ELBOW_GOLD].DAC_max     = ELBOW_MAX_DAC;
  DOF_types[ELBOW_GREEN].DAC_max    = ELBOW_MAX_DAC;
  DOF_types[Z_INS_GOLD].DAC_max     = Z_INS_MAX_DAC;
  DOF_types[Z_INS_GREEN].DAC_max    = Z_INS_MAX_DAC;
  */
  DOF_types[TOOL_ROT_GOLD].DAC_max = TOOL_ROT_MAX_DAC;
  DOF_types[TOOL_ROT_GREEN].DAC_max = TOOL_ROT_MAX_DAC;
  DOF_types[WRIST_GOLD].DAC_max = WRIST_MAX_DAC;
  DOF_types[WRIST_GREEN].DAC_max = WRIST_MAX_DAC;
  DOF_types[GRASP1_GOLD].DAC_max = GRASP1_MAX_DAC;
  DOF_types[GRASP1_GREEN].DAC_max = GRASP1_MAX_DAC;
  DOF_types[GRASP2_GOLD].DAC_max = GRASP2_MAX_DAC;
  DOF_types[GRASP2_GREEN].DAC_max = GRASP2_MAX_DAC;

  // DAC offsets to compensate for amp-specific output - defined in motor.h
  DOF_types[SHOULDER_GOLD].DAC_zero_offset = DAC_ZERO_0_0;
  DOF_types[ELBOW_GOLD].DAC_zero_offset = DAC_ZERO_0_1;
  DOF_types[Z_INS_GOLD].DAC_zero_offset = DAC_ZERO_0_2;
  DOF_types[NO_CONNECTION_GOLD].DAC_zero_offset = DAC_ZERO_0_3;
  DOF_types[TOOL_ROT_GOLD].DAC_zero_offset = DAC_ZERO_0_4;
  DOF_types[WRIST_GOLD].DAC_zero_offset = DAC_ZERO_0_5;
  DOF_types[GRASP1_GOLD].DAC_zero_offset = DAC_ZERO_0_6;
  DOF_types[GRASP2_GOLD].DAC_zero_offset = DAC_ZERO_0_7;

  DOF_types[SHOULDER_GREEN].DAC_zero_offset = DAC_ZERO_1_0;
  DOF_types[ELBOW_GREEN].DAC_zero_offset = DAC_ZERO_1_1;
  DOF_types[Z_INS_GREEN].DAC_zero_offset = DAC_ZERO_1_2;
  DOF_types[NO_CONNECTION_GREEN].DAC_zero_offset = DAC_ZERO_1_3;
  DOF_types[TOOL_ROT_GREEN].DAC_zero_offset = DAC_ZERO_1_4;
  DOF_types[WRIST_GREEN].DAC_zero_offset = DAC_ZERO_1_5;
  DOF_types[GRASP1_GREEN].DAC_zero_offset = DAC_ZERO_1_6;
  DOF_types[GRASP2_GREEN].DAC_zero_offset = DAC_ZERO_1_7;

  /// Initialize values of joint and DOF structures
  for (int i = 0; i < NUM_MECH; i++) {
    device0->mech[i].tool_type = use_tool;

    /// Initialize joint types
    if (device0->mech[i].type == GOLD_ARM) {
      // Set DOF type to unique index
      device0->mech[i].joint[SHOULDER].type = SHOULDER_GOLD;
      device0->mech[i].joint[ELBOW].type = ELBOW_GOLD;
      device0->mech[i].joint[Z_INS].type = Z_INS_GOLD;
      device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_GOLD;
      device0->mech[i].joint[WRIST].type = WRIST_GOLD;
      device0->mech[i].joint[GRASP1].type = GRASP1_GOLD;
      device0->mech[i].joint[GRASP2].type = GRASP2_GOLD;
      device0->mech[i].joint[NO_CONNECTION].type = NO_CONNECTION_GOLD;

      device0->mech[i].mech_tool = gold_arm_tool;
    } else if (device0->mech[i].type == GREEN_ARM) {
      device0->mech[i].joint[SHOULDER].type = SHOULDER_GREEN;
      device0->mech[i].joint[ELBOW].type = ELBOW_GREEN;
      device0->mech[i].joint[Z_INS].type = Z_INS_GREEN;
      device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_GREEN;
      device0->mech[i].joint[WRIST].type = WRIST_GREEN;
      device0->mech[i].joint[GRASP1].type = GRASP1_GREEN;
      device0->mech[i].joint[GRASP2].type = GRASP2_GREEN;
      device0->mech[i].joint[NO_CONNECTION].type = NO_CONNECTION_GREEN;

      device0->mech[i].mech_tool = green_arm_tool;
    }

    for (int j = 0; j < MAX_DOF_PER_MECH; j++) {
      DOF *_joint = &(device0->mech[i].joint[j]);
      int dofindex = _joint->type;
      DOF_type *_dof = &(DOF_types[dofindex]);

      // Initialize joint and motor position variables
      _joint->jpos = 0;
      _joint->mpos = 0;
      _joint->jpos_d = 0;
      _joint->jpos_d_old = 0;
      _joint->mpos_d = 0;
      _joint->mpos_d_old = 0;

      // Initialize the velocity storage elements
      _joint->jvel_d = 0;
      _joint->jvel = 0;
      _joint->mvel_d = 0;
      _joint->mvel = 0;

      // Initialize the position and velocity history for filter
      for (int k = 0; k < HISTORY_SIZE; k++) {
        _dof->old_mpos[k] = 0;
        _dof->old_mpos_d[k] = 0;
        _dof->old_mvel[k] = 0;
        _dof->old_mvel_d[k] = 0;
      }
      _dof->filterRdy = 0;

      // Set inital current command to zero
      _joint->current_cmd = 0;

      // on R_II torque convention is opposite for green/gold arms
      float torque_sign = 1;
#ifdef RAVEN_II

      // Positive torque -> positive joint angle change
      // Make sure encoders line up the same way.
      if (device0->mech[i].type == GOLD_ARM)
        torque_sign = -1;
      else if (device0->mech[i].type == GREEN_ARM)
        torque_sign = 1;
      else
        err_msg("Unknown mech type in init!");
#endif
      // Set i-max and current-torque conversion constants
      if ((j == SHOULDER) || (j == ELBOW) || (j == Z_INS)) {
        _dof->tau_per_amp =
            torque_sign * (float)(T_PER_AMP_BIG_MOTOR * GEAR_BOX_TR_BIG_MOTOR);  // Amps to torque
        _dof->DAC_per_amp = (float)(K_DAC_PER_AMP_HIGH_CURRENT);  // DAC counts to AMPS
        _dof->i_max = (float)(I_MAX_BIG_MOTOR);
        _dof->i_cont = (float)(I_CONT_BIG_MOTOR);
      } else  // set tool stuff
      {
        _dof->DAC_per_amp = (float)(K_DAC_PER_AMP_LOW_CURRENT);  // DAC counts to AMPS
        _dof->i_max = (float)(I_MAX_SMALL_MOTOR);
        _dof->i_cont = (float)(I_CONT_SMALL_MOTOR);

        //#ifdef RAVEN_II_SQUARE
        //                _dof->tau_per_amp = torque_sign *
        //                (float)(T_PER_AMP_SMALL_MOTOR *
        //                GEAR_BOX_TR_SMALL_MOTOR); // Amps to torque \todo why
        //                is this line not used?
        //#else
        //                _dof->tau_per_amp = -1 * (float)(T_PER_AMP_SMALL_MOTOR
        //                * GEAR_BOX_TR_SMALL_MOTOR); // Amps to torque
        //#endif

        // set tau_per_amp based on tool type
        //                switch (device0->mech[i].tool_type){
        //					case dv_adapter:
        //						_dof->tau_per_amp = 1 *
        //(float)(T_PER_AMP_SMALL_MOTOR  * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to
        // torque
        //						break;
        //					case RII_square_type:
        //						_dof->tau_per_amp = torque_sign *
        //(float)(T_PER_AMP_SMALL_MOTOR  * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to
        // torque \todo why is this line not used?
        //						break;
        //					default:
        //						_dof->tau_per_amp = -1 *
        //(float)(T_PER_AMP_SMALL_MOTOR  * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to
        // torque
        //						break;
        //                }

        switch (device0->mech[i].mech_tool.t_style) {
          case dv:
            _dof->tau_per_amp =
                1 * (float)(T_PER_AMP_SMALL_MOTOR * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to torque
            break;
          case square_raven:
            _dof->tau_per_amp = torque_sign * (float)(T_PER_AMP_SMALL_MOTOR *
                                                      GEAR_BOX_TR_SMALL_MOTOR);  // Amps to torque
                                                                                 // \todo why is
                                                                                 // this line not
                                                                                 // used?
            break;
          default:
            _dof->tau_per_amp =
                -1 * (float)(T_PER_AMP_SMALL_MOTOR * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to torque
            break;
        }

#ifdef OPPOSE_GRIP
        if (j == GRASP1) {
          _dof->tau_per_amp *= -1;  // swap the torque sign for the first grasper
        }
#endif
      }

      // Set encoder offset
      _joint->enc_offset = _joint->enc_val;

      // Set DOF state ready to go.
      _joint->state = jstate_not_ready;
    }

    /// Set the encoder offset from Kinematic Zero
    //  Note: enc_offset initialized first above
    //	Note: all evaluate to 0.0;
    if (device0->mech[i].type == GOLD_ARM) {
      device0->mech[i].joint[SHOULDER].enc_offset +=
          SHOULDER_GOLD_KIN_OFFSET * ENC_CNT_PER_DEG *
          DOF_types[SHOULDER_GOLD].TR;  // Degrees * enc/degree *
      device0->mech[i].joint[ELBOW].enc_offset +=
          ELBOW_GOLD_KIN_OFFSET * ENC_CNT_PER_DEG * DOF_types[ELBOW_GOLD].TR;
      device0->mech[i].joint[Z_INS].enc_offset +=
          Z_INS_GOLD_KIN_OFFSET * DOF_types[Z_INS_GOLD].TR *
          ENC_CNT_PER_RAD;  // use enc/rad because conversion from meters to
                            // revolutions is in radians

      device0->mech[i].joint[SHOULDER].joint_enc_offset +=
          SHOULDER_GOLD_KIN_OFFSET * JOINT_ENC_CNT_PER_DEG;  // Degrees * enc/degree *
      device0->mech[i].joint[ELBOW].joint_enc_offset +=
          ELBOW_GOLD_KIN_OFFSET * JOINT_ENC_CNT_PER_DEG;
      device0->mech[i].joint[Z_INS].joint_enc_offset +=
          Z_INS_GOLD_KIN_OFFSET *
          LINEAR_JOINT_ENC_PER_M;  // use enc/rad because conversion from meters
                                   // to revolutions is in radians
    } else if (device0->mech[i].type == GREEN_ARM) {
      device0->mech[i].joint[SHOULDER].enc_offset +=
          SHOULDER_GREEN_KIN_OFFSET * ENC_CNT_PER_DEG * DOF_types[SHOULDER_GREEN].TR;
      device0->mech[i].joint[ELBOW].enc_offset +=
          ELBOW_GREEN_KIN_OFFSET * ENC_CNT_PER_DEG * DOF_types[ELBOW_GREEN].TR;
      device0->mech[i].joint[Z_INS].enc_offset +=
          Z_INS_GREEN_KIN_OFFSET * DOF_types[Z_INS_GREEN].TR *
          ENC_CNT_PER_RAD;  // use enc/rad because conversion from meters to
                            // revolutions is in radians

      device0->mech[i].joint[SHOULDER].joint_enc_offset +=
          SHOULDER_GREEN_KIN_OFFSET * JOINT_ENC_CNT_PER_DEG;  // Degrees * enc/degree *
      device0->mech[i].joint[ELBOW].joint_enc_offset +=
          ELBOW_GREEN_KIN_OFFSET * JOINT_ENC_CNT_PER_DEG;
      device0->mech[i].joint[Z_INS].joint_enc_offset +=
          Z_INS_GREEN_KIN_OFFSET *
          LINEAR_JOINT_ENC_PER_M;  // use enc/rad because conversion from meters
                                   // to revolutions is in radians
    }

    /// Initialize some more mechanism stuff
    device0->mech[i].pos_d.x = 0;
    device0->mech[i].pos_d.y = 0;
    device0->mech[i].pos_d.z = 0;

    device0->mech[i].ori_d.yaw = 0;
    device0->mech[i].ori_d.pitch = 0;
    device0->mech[i].ori_d.roll = 0;

    // set tool specific values
    // TODO: add home angles????

    int offset = (device0->mech[i].type == GREEN_ARM) ? 8 : 0;

    DOF_types[SHOULDER + offset].max_position = SHOULDER_MAX_ANGLE;
    DOF_types[SHOULDER + offset].max_limit = SHOULDER_MAX_LIMIT;
    DOF_types[SHOULDER + offset].min_limit = SHOULDER_MIN_LIMIT;
    DOF_types[SHOULDER + offset].home_position = SHOULDER_HOME_ANGLE;

    DOF_types[ELBOW + offset].max_position = ELBOW_MAX_ANGLE;
    DOF_types[ELBOW + offset].max_limit = ELBOW_MAX_LIMIT;
    DOF_types[ELBOW + offset].min_limit = ELBOW_MIN_LIMIT;
    DOF_types[ELBOW + offset].home_position = ELBOW_HOME_ANGLE;

    DOF_types[Z_INS + offset].max_position = Z_INS_MAX_ANGLE;
    DOF_types[Z_INS + offset].max_limit = Z_INS_MAX_LIMIT;
    DOF_types[Z_INS + offset].min_limit = Z_INS_MIN_LIMIT;
    DOF_types[Z_INS + offset].home_position = Z_INS_HOME_ANGLE;

    DOF_types[TOOL_ROT + offset].max_position = device0->mech[i].mech_tool.rot_max_angle;
    DOF_types[TOOL_ROT + offset].max_limit = device0->mech[i].mech_tool.rot_max_limit;
    DOF_types[TOOL_ROT + offset].min_limit = device0->mech[i].mech_tool.rot_min_limit;
    DOF_types[TOOL_ROT + offset].home_position = device0->mech[i].mech_tool.rot_home_angle;

    DOF_types[WRIST + offset].max_position = device0->mech[i].mech_tool.wrist_max_angle;
    DOF_types[WRIST + offset].max_limit = device0->mech[i].mech_tool.wrist_max_limit;
    DOF_types[WRIST + offset].min_limit = device0->mech[i].mech_tool.wrist_min_limit;
    DOF_types[WRIST + offset].home_position = device0->mech[i].mech_tool.wrist_home_angle;

    DOF_types[GRASP1 + offset].max_position = device0->mech[i].mech_tool.grasp1_max_angle;
    DOF_types[GRASP1 + offset].max_limit = device0->mech[i].mech_tool.grasp1_max_limit;
    DOF_types[GRASP1 + offset].min_limit = device0->mech[i].mech_tool.grasp1_min_limit;
    DOF_types[GRASP1 + offset].home_position = device0->mech[i].mech_tool.grasp1_home_angle;

    DOF_types[GRASP2 + offset].max_position = device0->mech[i].mech_tool.grasp2_max_angle;
    DOF_types[GRASP2 + offset].max_limit = device0->mech[i].mech_tool.grasp2_max_limit;
    DOF_types[GRASP2 + offset].min_limit = device0->mech[i].mech_tool.grasp2_min_limit;
    DOF_types[GRASP2 + offset].home_position = device0->mech[i].mech_tool.grasp2_home_angle;
  }

  dofs_inited = 1;
}

/**\fn int init_ravengains(ros::NodeHandle n, device *device0)
  \brief Get ravengains from ROS parameter server.
  \struct device
  \param device0 pointer to device struct
  \warning The order of gains in the parameter is very important.
 *      Make sure that numerical order of parameters matches the numerical order
 of the dof types
  \post dof_types[].kp and dof_types[].kd have been set from ROS parameters
  \return
  \ingroup DataStructures
*/
int init_ravengains(ros::NodeHandle n, device *device0) {
  XmlRpc::XmlRpcValue kp_green, kp_gold, kd_green, kd_gold, ki_green, ki_gold;
  bool res = 0;
  log_msg("Getting gains params...");

  // initialize all gains to zero
  for (int i = 0; i < MAX_MECH * MAX_DOF_PER_MECH; i++) {
    DOF_types[i].KP = DOF_types[i].KD = DOF_types[i].KI = 0.0;
  }

  /// Get gains from parameter server, use default value of 0.0 if parameters
  /// ain't found
  if (n.hasParam("/gains_gold_kp") && n.hasParam("/gains_gold_kd") &&
      n.hasParam("/gains_gold_ki") && n.hasParam("/gains_green_kp") &&
      n.hasParam("/gains_green_kd") && n.hasParam("/gains_green_ki")) {
    res = n.getParam("/gains_green_kp", kp_green);
    res &= n.getParam("/gains_green_kd", kd_green);
    res &= n.getParam("/gains_green_ki", ki_green);
    res &= n.getParam("/gains_gold_kp", kp_gold);
    res &= n.getParam("/gains_gold_kd", kd_gold);
    res &= n.getParam("/gains_gold_ki", ki_gold);
  }

  // Did we get the gains??
  if (!res || (kp_green.size() != MAX_DOF_PER_MECH) || (kd_green.size() != MAX_DOF_PER_MECH) ||
      (ki_green.size() != MAX_DOF_PER_MECH) || (kp_gold.size() != MAX_DOF_PER_MECH) ||
      (kd_gold.size() != MAX_DOF_PER_MECH) || (ki_gold.size() != MAX_DOF_PER_MECH)) {
    ROS_ERROR("Gains parameters failed.  Setting zero gains");
  } else {
    bool initgold = 0, initgreen = 0;
    for (int i = 0; i < NUM_MECH; i++) {
      for (int j = 0; j < MAX_DOF_PER_MECH; j++) {
        int dofindex;

        // Set gains for gold and green arms
        if (device0->mech[i].type == GOLD_ARM) {
          initgold = true;
          dofindex = j;
          DOF_types[dofindex].KP =
              (double)kp_gold[j];  // Cast XMLRPC value to a double and set gain
          DOF_types[dofindex].KD = (double)kd_gold[j];  //   ""
          DOF_types[dofindex].KI = (double)ki_gold[j];  //   ""
        } else if (device0->mech[i].type == GREEN_ARM) {
          initgreen = true;
          dofindex = 1 * MAX_DOF_PER_MECH + j;
          DOF_types[dofindex].KP = (double)kp_green[j];  //   ""
          DOF_types[dofindex].KD = (double)kd_green[j];  //   ""
          DOF_types[dofindex].KI = (double)ki_green[j];  //   ""
        } else {
          ROS_ERROR("What device is this?? %d\n", device0->mech[i].type);
        }
      }
    }
    if (!initgold) {
      ROS_ERROR("Failed to set gains for gold arm (ser:%d not %d).  Set to zero",
                device0->mech[0].type, GOLD_ARM);
    }
    if (!initgreen) {
      ROS_ERROR("Failed to set gains for green arm (ser:%d not %d).  Set to zero",
                device0->mech[1].type, GREEN_ARM);
    }
    log_msg("  PD gains set to");
    log_msg(
        "    gold: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
        DOF_types[0].KP, DOF_types[0].KD, DOF_types[0].KI, DOF_types[1].KP, DOF_types[1].KD,
        DOF_types[1].KI, DOF_types[2].KP, DOF_types[2].KD, DOF_types[2].KI, DOF_types[3].KP,
        DOF_types[3].KD, DOF_types[3].KI, DOF_types[4].KP, DOF_types[4].KD, DOF_types[4].KI,
        DOF_types[5].KP, DOF_types[5].KD, DOF_types[5].KI, DOF_types[6].KP, DOF_types[6].KD,
        DOF_types[6].KI, DOF_types[7].KP, DOF_types[7].KD, DOF_types[7].KI);
    log_msg(
        "    green: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
        DOF_types[8].KP, DOF_types[8].KD, DOF_types[8].KI, DOF_types[9].KP, DOF_types[9].KD,
        DOF_types[9].KI, DOF_types[10].KP, DOF_types[10].KD, DOF_types[10].KI, DOF_types[11].KP,
        DOF_types[11].KD, DOF_types[11].KI, DOF_types[12].KP, DOF_types[12].KD, DOF_types[12].KI,
        DOF_types[13].KP, DOF_types[13].KD, DOF_types[13].KI, DOF_types[14].KP, DOF_types[14].KD,
        DOF_types[14].KI, DOF_types[15].KP, DOF_types[15].KD, DOF_types[15].KI);
  }

  return 0;
}

/**\fn void setStartXYZ(device *device0)
  \brief set the starting desired xyz coordinate (pos_d = pos)

  Set the initial desired position equal to the actual position so that
  system starts without error.

  \struct device
  \param device0 pointer to device
  \return
  \ingroup Control
  (could also be [ ]ingroup DataStructures )
*/
void setStartXYZ(device *device0) {
  int i;
  static int j;
  j++;

  // Get the forward kinematics of this position
  fwdCableCoupling(device0, RL_INIT);
  r2_fwd_kin(device0, RL_INIT);

  // Set XYZ offsets
  for (i = 0; i < NUM_MECH; i++) {
    device0->mech[i].pos_d.x = device0->mech[i].pos.x;
    device0->mech[i].pos_d.y = device0->mech[i].pos.y;
    device0->mech[i].pos_d.z = device0->mech[i].pos.z;

    device0->mech[i].ori_d.yaw = device0->mech[i].ori.yaw;
    device0->mech[i].ori_d.pitch = device0->mech[i].ori.pitch;
    device0->mech[i].ori_d.roll = device0->mech[i].ori.roll;
    device0->mech[i].ori_d.grasp = device0->mech[i].ori.grasp;

    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) device0->mech[i].ori_d.R[j][k] = device0->mech[i].ori.R[j][k];
  }

  // Update the origin, to which master-side deltas are added.
  updateMasterRelativeOrigin(device0);
}

/** \todo what is the function of the following commented code; shall we get rid
 * of them??
 * 			(AL - This code automatically bumps the encoders, so that the
 * user doesn't need to.
 * 				I don't think it worked...)
 */

/*

    int i=0, j=0;
    mechanism* _mech=NULL;
    DOF* _joint=NULL;
    float amps_on_wait = 0.9; // 900ms
    float bump_encoder_wait = amps_on_wait + 0.05; // 50ms
    static int startflag=0;

    static ros::Time t1 = t1.now();
    static ros::Time t2 = t2.now();
    static ros::Duration d;

   case 0:     // Automatically "bump" encoders
    {
        // run once to initialize initialization *sigh*
        if (!startflag)
        {
            log_msg("  Starting Initialization");
            startflag=1;
            t1=t1.now();
        }

        t2 = t2.now();
        d = t2-t1;

        // wait for motor amplifiers to turn on
        if (d.toSec() < amps_on_wait)
        {
            _mech=NULL;
            _joint=NULL;
            while(loop_over_joints(device0, _mech, _joint, i, j))
                _joint->current_cmd = 0;
        }

        // apply displacement current for short time
        else if (d.toSec() < bump_encoder_wait)
        {
            log_msg("bump");
            // apply first one direction, then the other.
            int sign = d.toSec() < ((bump_encoder_wait - amps_on_wait)/2) ?
   1:-1;

            _mech=NULL;
            _joint=NULL;
            while(loop_over_joints(device0, _mech, _joint, i, j))
            {
                if (is_toolDOF(_joint->type))
                    _joint->current_cmd = sign * TOOL_ROT_MAX_DAC;
                else
                    _joint->current_cmd = sign * -1499;
            }
        }
        // finished bumping encoders.  Continue initialization
        else
        {
            log_msg("    Encoders bumped.");
            _mech=NULL;
            _joint=NULL;
            while(loop_over_joints(device0, _mech, _joint, i, j))
                _joint->current_cmd = 0;

            usb_reset_encoders(GOLD_ARM_SERIAL);
            usb_reset_encoders(GREEN_ARM_SERIAL);
            init_wait_loop=0;
            currParams->sublevel = 1;
            startflag=0;
        }
        break;
    }
 */
