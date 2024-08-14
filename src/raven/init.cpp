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


/**************** tool selection ********************/
// tool gold_arm_tool (bipolar_forceps, GOLD_ARM);
tool gold_arm_tool(large_needle, GOLD_ARM);
 // tool gold_arm_tool(r_grasper, GOLD_ARM);
// tool gold_arm_tool(micro_forceps, GOLD_ARM);
//tool gold_arm_tool(ricks_tool, gold);
//tool gold_arm_tool(qut_camera, GOLD_ARM);


// tool green_arm_tool(mopocu_scissor, GREEN_ARM);
tool green_arm_tool(large_needle,  GREEN_ARM);
// tool green_arm_tool(mopocu_scissor, GREEN_ARM);
// tool green_arm_tool(potts_scissor, GREEN_ARM);
//tool green_arm_tool(r_grasper, GREEN_ARM);
//tool green_arm_tool(bipolar_forceps, GREEN_ARM);
//tool green_arm_tool(qut_camera, green);
// tool green_arm_tool(r_grasper, GREEN_ARM);
//tool green_arm_tool(ricks_tool, GREEN_ARM);



tool blue_arm_tool(r_grasper, blue);

tool orange_arm_tool(ricks_tool, orange);

/********** positioning joints Homing DAC ***********/
const int gold_joints_homing_max_dac[4] = {
                               2100,  // shoulder
                               2100,  // elbow
                               1600,  // z-ins
                               0};

const int green_joints_homing_max_dac[4] = {
                               2100,  // shoulder
                               2100,  // elbow
                               1600,  // z-ins
                               0};                               
const int blue_joints_homing_max_dac[4] = {
                               2100,  // shoulder
                               2100,  // elbow
                               1600,  // z-ins
                               0};

const int orange_joints_homing_max_dac[4] = {
                               2100,  // shoulder
                               2100,  // elbow
                               1600,  // z-ins
                               0};     

/**************** tool DOF Homing DAC ***************/
const int dv_tool_homing_max_dac[4] = {
                               2000,   // tool_rot 
                               2400,   // wrist
                               1600,   // grasp1
                               1600};  // grasp2

const int raven_tool_homing_max_dac[4] = {
                               1900,   // tool_rot
                               2100,   // wrist
                               2250,   // grasp1 decreased from 1900
                               2250};  // grasp2 decreased from 1900




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

    for(int i = 0; i < NUM_MECH; i++){
      if (device0->mech[i].name == green)
        currParams->param_tool_type[i] = green_arm_tool.t_end;
      else if (device0->mech[i].type == gold)
        currParams->param_tool_type[i] = gold_arm_tool.t_end;
      else if (device0->mech[i].type == blue)
        currParams->param_tool_type[i] = blue_arm_tool.t_end;
      else if (device0->mech[i].type == orange)
        currParams->param_tool_type[i] = orange_arm_tool.t_end;
    }
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

  device0->mech[0].joint_control = false;
  device0->mech[1].joint_control = false;
  device0->mech[2].joint_control = false;
  device0->mech[3].joint_control = false;
  // device0->crtk_motion_planner.crtk_motion_api[0].set_default_base_frame(0);
  // device0->crtk_motion_planner.crtk_motion_api[1].set_default_base_frame(1);
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
  DOF_types[GRASP1_GREEN].TR = GRASP1_TR_GREEN_ARM;
  DOF_types[GRASP2_GREEN].TR = GRASP2_TR_GREEN_ARM;

  DOF_types[SHOULDER_BLUE].TR = SHOULDER_TR_BLUE_ARM;
  DOF_types[ELBOW_BLUE].TR = ELBOW_TR_BLUE_ARM;
  DOF_types[Z_INS_BLUE].TR = Z_INS_TR_BLUE_ARM;
  DOF_types[TOOL_ROT_BLUE].TR = TOOL_ROT_TR_BLUE_ARM;
  DOF_types[WRIST_BLUE].TR = WRIST_TR_BLUE_ARM;
  DOF_types[GRASP1_BLUE].TR = GRASP1_TR_BLUE_ARM;
  DOF_types[GRASP2_BLUE].TR = GRASP2_TR_BLUE_ARM;

  DOF_types[SHOULDER_ORANGE].TR = SHOULDER_TR_ORANGE_ARM;
  DOF_types[ELBOW_ORANGE].TR = ELBOW_TR_ORANGE_ARM;
  DOF_types[Z_INS_ORANGE].TR = Z_INS_TR_ORANGE_ARM;
  DOF_types[TOOL_ROT_ORANGE].TR = TOOL_ROT_TR_ORANGE_ARM;
  DOF_types[WRIST_ORANGE].TR = WRIST_TR_ORANGE_ARM;
  DOF_types[GRASP1_ORANGE].TR = GRASP1_TR_ORANGE_ARM;
  DOF_types[GRASP2_ORANGE].TR = GRASP2_TR_ORANGE_ARM;

  /// Initialize current limits
  if (SAFETY_LEVEL == BEGINNER_MODE) {
    DOF_types[SHOULDER_GOLD].DAC_max = BEGINNER_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_GREEN].DAC_max = BEGINNER_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_GOLD].DAC_max = BEGINNER_ELBOW_MAX_DAC;
    DOF_types[ELBOW_GREEN].DAC_max = BEGINNER_ELBOW_MAX_DAC;
    DOF_types[Z_INS_GOLD].DAC_max = BEGINNER_Z_INS_MAX_DAC;
    DOF_types[Z_INS_GREEN].DAC_max = BEGINNER_Z_INS_MAX_DAC;

    DOF_types[SHOULDER_BLUE].DAC_max = BEGINNER_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_ORANGE].DAC_max = BEGINNER_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_BLUE].DAC_max = BEGINNER_ELBOW_MAX_DAC;
    DOF_types[ELBOW_ORANGE].DAC_max = BEGINNER_ELBOW_MAX_DAC;
    DOF_types[Z_INS_BLUE].DAC_max = BEGINNER_Z_INS_MAX_DAC;
    DOF_types[Z_INS_ORANGE].DAC_max = BEGINNER_Z_INS_MAX_DAC;
  } else if (SAFETY_LEVEL == MODERATE_MODE) {
    DOF_types[SHOULDER_GOLD].DAC_max = MODERATE_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_GREEN].DAC_max = MODERATE_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_GOLD].DAC_max = MODERATE_ELBOW_MAX_DAC;
    DOF_types[ELBOW_GREEN].DAC_max = MODERATE_ELBOW_MAX_DAC;
    DOF_types[Z_INS_GOLD].DAC_max = MODERATE_Z_INS_MAX_DAC;
    DOF_types[Z_INS_GREEN].DAC_max = MODERATE_Z_INS_MAX_DAC;

    DOF_types[SHOULDER_BLUE].DAC_max = MODERATE_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_ORANGE].DAC_max = MODERATE_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_BLUE].DAC_max = MODERATE_ELBOW_MAX_DAC;
    DOF_types[ELBOW_ORANGE].DAC_max = MODERATE_ELBOW_MAX_DAC;
    DOF_types[Z_INS_BLUE].DAC_max = MODERATE_Z_INS_MAX_DAC;
    DOF_types[Z_INS_ORANGE].DAC_max = MODERATE_Z_INS_MAX_DAC;
  } else  // SAFETY_LEVEL == ADVANCED_MODE
  {
    DOF_types[SHOULDER_GOLD].DAC_max = ADVANCED_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_GREEN].DAC_max = ADVANCED_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_GOLD].DAC_max = ADVANCED_ELBOW_MAX_DAC;
    DOF_types[ELBOW_GREEN].DAC_max = ADVANCED_ELBOW_MAX_DAC;
    DOF_types[Z_INS_GOLD].DAC_max = ADVANCED_Z_INS_MAX_DAC;
    DOF_types[Z_INS_GREEN].DAC_max = ADVANCED_Z_INS_MAX_DAC;

    DOF_types[SHOULDER_BLUE].DAC_max = ADVANCED_SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_ORANGE].DAC_max = ADVANCED_SHOULDER_MAX_DAC;
    DOF_types[ELBOW_BLUE].DAC_max = ADVANCED_ELBOW_MAX_DAC;
    DOF_types[ELBOW_ORANGE].DAC_max = ADVANCED_ELBOW_MAX_DAC;
    DOF_types[Z_INS_BLUE].DAC_max = ADVANCED_Z_INS_MAX_DAC;
    DOF_types[Z_INS_ORANGE].DAC_max = ADVANCED_Z_INS_MAX_DAC;
  }


  DOF_types[TOOL_ROT_GOLD].DAC_max = TOOL_ROT_MAX_DAC;
  DOF_types[TOOL_ROT_GREEN].DAC_max = TOOL_ROT_MAX_DAC;
  DOF_types[WRIST_GOLD].DAC_max = WRIST_MAX_DAC;
  DOF_types[WRIST_GREEN].DAC_max = WRIST_MAX_DAC;
  DOF_types[GRASP1_GOLD].DAC_max = GRASP1_MAX_DAC;
  DOF_types[GRASP1_GREEN].DAC_max = GRASP1_MAX_DAC;
  DOF_types[GRASP2_GOLD].DAC_max = GRASP2_MAX_DAC;
  DOF_types[GRASP2_GREEN].DAC_max = GRASP2_MAX_DAC;

  DOF_types[TOOL_ROT_BLUE].DAC_max = TOOL_ROT_MAX_DAC;
  DOF_types[TOOL_ROT_ORANGE].DAC_max = TOOL_ROT_MAX_DAC;
  DOF_types[WRIST_BLUE].DAC_max = WRIST_MAX_DAC;
  DOF_types[WRIST_ORANGE].DAC_max = WRIST_MAX_DAC;
  DOF_types[GRASP1_BLUE].DAC_max = GRASP1_MAX_DAC;
  DOF_types[GRASP1_ORANGE].DAC_max = GRASP1_MAX_DAC;
  DOF_types[GRASP2_BLUE].DAC_max = GRASP2_MAX_DAC;
  DOF_types[GRASP2_ORANGE].DAC_max = GRASP2_MAX_DAC;

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

  DOF_types[SHOULDER_BLUE].DAC_zero_offset = DAC_ZERO_2_0;
  DOF_types[ELBOW_BLUE].DAC_zero_offset = DAC_ZERO_2_1;
  DOF_types[Z_INS_BLUE].DAC_zero_offset = DAC_ZERO_2_2;
  DOF_types[NO_CONNECTION_BLUE].DAC_zero_offset = DAC_ZERO_2_3;
  DOF_types[TOOL_ROT_BLUE].DAC_zero_offset = DAC_ZERO_2_4;
  DOF_types[WRIST_BLUE].DAC_zero_offset = DAC_ZERO_2_5;
  DOF_types[GRASP1_BLUE].DAC_zero_offset = DAC_ZERO_2_6;
  DOF_types[GRASP2_BLUE].DAC_zero_offset = DAC_ZERO_2_7;

  DOF_types[SHOULDER_ORANGE].DAC_zero_offset = DAC_ZERO_3_0;
  DOF_types[ELBOW_ORANGE].DAC_zero_offset = DAC_ZERO_3_1;
  DOF_types[Z_INS_ORANGE].DAC_zero_offset = DAC_ZERO_3_2;
  DOF_types[NO_CONNECTION_ORANGE].DAC_zero_offset = DAC_ZERO_3_3;
  DOF_types[TOOL_ROT_ORANGE].DAC_zero_offset = DAC_ZERO_3_4;
  DOF_types[WRIST_ORANGE].DAC_zero_offset = DAC_ZERO_3_5;
  DOF_types[GRASP1_ORANGE].DAC_zero_offset = DAC_ZERO_3_6;
  DOF_types[GRASP2_ORANGE].DAC_zero_offset = DAC_ZERO_3_7;

  /// Initialize values of joint and DOF structures
  for (int i = 0; i < NUM_MECH; i++) {
    

    /// Initialize joint types
    if (device0->mech[i].name == gold) {
      // Set DOF type to unique index
      device0->mech[i].joint[SHOULDER].type = SHOULDER_GOLD;
      device0->mech[i].joint[ELBOW].type = ELBOW_GOLD;
      device0->mech[i].joint[Z_INS].type = Z_INS_GOLD;
      device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_GOLD;
      device0->mech[i].joint[WRIST].type = WRIST_GOLD;
      device0->mech[i].joint[GRASP1].type = GRASP1_GOLD;
      device0->mech[i].joint[GRASP2].type = GRASP2_GOLD;
      device0->mech[i].joint[NO_CONNECTION].type = NO_CONNECTION_GOLD;

      
    } else if (device0->mech[i].name == green) {
      device0->mech[i].joint[SHOULDER].type = SHOULDER_GREEN;
      device0->mech[i].joint[ELBOW].type = ELBOW_GREEN;
      device0->mech[i].joint[Z_INS].type = Z_INS_GREEN;
      device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_GREEN;
      device0->mech[i].joint[WRIST].type = WRIST_GREEN;
      device0->mech[i].joint[GRASP1].type = GRASP1_GREEN;
      device0->mech[i].joint[GRASP2].type = GRASP2_GREEN;
      device0->mech[i].joint[NO_CONNECTION].type = NO_CONNECTION_GREEN; 
    
    
    } else if (device0->mech[i].name == blue) {
      device0->mech[i].joint[SHOULDER].type = SHOULDER_BLUE;
      device0->mech[i].joint[ELBOW].type = ELBOW_BLUE;
      device0->mech[i].joint[Z_INS].type = Z_INS_BLUE;
      device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_BLUE;
      device0->mech[i].joint[WRIST].type = WRIST_BLUE;
      device0->mech[i].joint[GRASP1].type = GRASP1_BLUE;
      device0->mech[i].joint[GRASP2].type = GRASP2_BLUE;
      device0->mech[i].joint[NO_CONNECTION].type = NO_CONNECTION_BLUE;
  
    
    } else if (device0->mech[i].name == orange) {
      device0->mech[i].joint[SHOULDER].type = SHOULDER_ORANGE;
      device0->mech[i].joint[ELBOW].type = ELBOW_ORANGE;
      device0->mech[i].joint[Z_INS].type = Z_INS_ORANGE;
      device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_ORANGE;
      device0->mech[i].joint[WRIST].type = WRIST_ORANGE;
      device0->mech[i].joint[GRASP1].type = GRASP1_ORANGE;
      device0->mech[i].joint[GRASP2].type = GRASP2_ORANGE;
      device0->mech[i].joint[NO_CONNECTION].type = NO_CONNECTION_ORANGE;

    }

    if (device0->mech[i].name == gold)
      device0->mech[i].mech_tool = gold_arm_tool;
    else if (device0->mech[i].name == green)
      device0->mech[i].mech_tool = green_arm_tool;    
    else if (device0->mech[i].name == blue)
      device0->mech[i].mech_tool = blue_arm_tool;    
    else if (device0->mech[i].name == orange)
      device0->mech[i].mech_tool = orange_arm_tool;

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


      // Positive torque -> positive joint angle change
      // Make sure encoders line up the same way.
      if (device0->mech[i].type == GOLD_ARM)
        torque_sign = -1;
      else if (device0->mech[i].type == GREEN_ARM)
        torque_sign = 1;
      else
        err_msg("Unknown mech type in init!");

      // Set i-max and current-torque conversion constants
      if ((j == SHOULDER) || (j == ELBOW) || (j == Z_INS)) {
        _dof->tau_per_amp =
            torque_sign * (float)(T_PER_AMP_BIG_MOTOR * GEAR_BOX_TR_BIG_MOTOR);  // Amps to torque
        _dof->DAC_per_amp = (float)(K_DAC_PER_AMP_HIGH_CURRENT);  // DAC counts to AMPS
        _dof->i_max = (float)(I_MAX_BIG_MOTOR);
        _dof->i_cont = (float)(I_CONT_BIG_MOTOR);
        if (device0->mech[i].name == gold){
          _joint->homing_dac = gold_joints_homing_max_dac[j];
        }
        else if (device0->mech[i].name == green){ 
          _joint->homing_dac = green_joints_homing_max_dac[j];
        }
        else if (device0->mech[i].name == blue){ 
          _joint->homing_dac = blue_joints_homing_max_dac[j];
        }
        else if (device0->mech[i].name == orange){ 
          _joint->homing_dac = orange_joints_homing_max_dac[j];
        }
      } else  // set tool stuff
      {
        _dof->DAC_per_amp = (float)(K_DAC_PER_AMP_LOW_CURRENT);  // DAC counts to AMPS
        _dof->i_max = (float)(I_MAX_SMALL_MOTOR);
        _dof->i_cont = (float)(I_CONT_SMALL_MOTOR);

      

        switch (device0->mech[i].mech_tool.adapter_style) {
          case dv:
            _dof->tau_per_amp =
                1 * (float)(T_PER_AMP_SMALL_MOTOR * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to torque
            _joint->homing_dac = dv_tool_homing_max_dac[j-4];    
            break;
          case square_raven:
            _dof->tau_per_amp = torque_sign * (float)(T_PER_AMP_SMALL_MOTOR *
                                                      GEAR_BOX_TR_SMALL_MOTOR);  
            _joint->homing_dac = raven_sq_tool_homing_max_dac[j-4];
            break;
          default: //raven
            _dof->tau_per_amp =
                -1 * (float)(T_PER_AMP_SMALL_MOTOR * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to torque
            _joint->homing_dac = raven_tool_homing_max_dac[j-4];       
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

    int offset = 0; 
    if (device0->mech[i].name == green) 
      offset = 1 * MAX_DOF_PER_MECH; 
    else if (device0->mech[i].name == blue) 
      offset = 2 * MAX_DOF_PER_MECH;
    else if (device0->mech[i].name == orange) 
      offset = 3 * MAX_DOF_PER_MECH;

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

  //XmlRpc::XmlRpcValue my_list;
  //nh.getParam("my_list", my_list);

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
    bool initgold = 0, initgreen = 0, initblue = 0, initorange = 0;
    for (int i = 0; i < NUM_MECH; i++) {
      for (int j = 0; j < MAX_DOF_PER_MECH; j++) {
        int dofindex;

        // Set gains for gold and green arms
        if (device0->mech[i].name == gold) {
          initgold = true;
          dofindex = j;
          DOF_types[dofindex].KP = (double)kp_gold[j];  // Cast XMLRPC value to a double and set gain
          DOF_types[dofindex].KD = (double)kd_gold[j];  //   ""
          DOF_types[dofindex].KI = (double)ki_gold[j];  //   ""

        } else if (device0->mech[i].name == green) {
          initgreen = true;
          dofindex = 1 * MAX_DOF_PER_MECH + j;
          DOF_types[dofindex].KP = (double)kp_green[j];  //   ""
          DOF_types[dofindex].KD = (double)kd_green[j];  //   ""
          DOF_types[dofindex].KI = (double)ki_green[j];  //   ""

        } else if (device0->mech[i].name == blue) {
          initblue = true;
          dofindex = 2 * MAX_DOF_PER_MECH + j;
          DOF_types[dofindex].KP = (double)kp_gold[j];  // Cast XMLRPC value to a double and set gain
          DOF_types[dofindex].KD = (double)kd_gold[j];  //   ""
          DOF_types[dofindex].KI = (double)ki_gold[j];  //   ""
          if (device0->mech[i].name == blue) log_msg("blue arm gains set to GOLD gains");
        } else if (device0->mech[i].name == orange) {
          initorange = true;
          dofindex = 3 * MAX_DOF_PER_MECH + j;
          DOF_types[dofindex].KP = (double)kp_green[j];  //   ""
          DOF_types[dofindex].KD = (double)kd_green[j];  //   ""
          DOF_types[dofindex].KI = (double)ki_green[j];  //   ""
          if (device0->mech[i].name == orange) log_msg("orange arm gains set to GREEN gains");
        } else {
          ROS_ERROR("What device is this?? %d\n", device0->mech[i].type);
        }
      }
    }
    if (!initgold) {
      ROS_ERROR("Failed to set gains for gold arm (ser:%d not %d).  Set to zero",
                device0->mech[0].serial, GOLD_ARM_SERIAL);
    }
    if (!initgreen) {
      ROS_ERROR("Failed to set gains for green arm (ser:%d not %d).  Set to zero",
                device0->mech[1].serial, GREEN_ARM_SERIAL);
    }
    log_msg("  PD gains set to");
    int off = 0;
    log_msg(
        "    gold: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
        DOF_types[off + 0].KP, DOF_types[off + 0].KD, DOF_types[off + 0].KI, DOF_types[off + 1].KP, DOF_types[off + 1].KD,
        DOF_types[off + 1].KI, DOF_types[off + 2].KP, DOF_types[off + 2].KD, DOF_types[off + 2].KI, DOF_types[off + 3].KP,
        DOF_types[off + 3].KD, DOF_types[off + 3].KI, DOF_types[off + 4].KP, DOF_types[off + 4].KD, DOF_types[off + 4].KI,
        DOF_types[off + 5].KP, DOF_types[off + 5].KD, DOF_types[off + 5].KI, DOF_types[off + 6].KP, DOF_types[off + 6].KD,
        DOF_types[off + 6].KI, DOF_types[off + 7].KP, DOF_types[off + 7].KD, DOF_types[off + 7].KI);
    
    off = 8;
    log_msg(
        "    gold: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
        DOF_types[off + 0].KP, DOF_types[off + 0].KD, DOF_types[off + 0].KI, DOF_types[off + 1].KP, DOF_types[off + 1].KD,
        DOF_types[off + 1].KI, DOF_types[off + 2].KP, DOF_types[off + 2].KD, DOF_types[off + 2].KI, DOF_types[off + 3].KP,
        DOF_types[off + 3].KD, DOF_types[off + 3].KI, DOF_types[off + 4].KP, DOF_types[off + 4].KD, DOF_types[off + 4].KI,
        DOF_types[off + 5].KP, DOF_types[off + 5].KD, DOF_types[off + 5].KI, DOF_types[off + 6].KP, DOF_types[off + 6].KD,
        DOF_types[off + 6].KI, DOF_types[off + 7].KP, DOF_types[off + 7].KD, DOF_types[off + 7].KI);

    off = 16;
    log_msg(
        "    blue: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
        DOF_types[off + 0].KP, DOF_types[off + 0].KD, DOF_types[off + 0].KI, DOF_types[off + 1].KP, DOF_types[off + 1].KD,
        DOF_types[off + 1].KI, DOF_types[off + 2].KP, DOF_types[off + 2].KD, DOF_types[off + 2].KI, DOF_types[off + 3].KP,
        DOF_types[off + 3].KD, DOF_types[off + 3].KI, DOF_types[off + 4].KP, DOF_types[off + 4].KD, DOF_types[off + 4].KI,
        DOF_types[off + 5].KP, DOF_types[off + 5].KD, DOF_types[off + 5].KI, DOF_types[off + 6].KP, DOF_types[off + 6].KD,
        DOF_types[off + 6].KI, DOF_types[off + 7].KP, DOF_types[off + 7].KD, DOF_types[off + 7].KI);

    off = 24; 
        log_msg(
        "    orange: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, "
        "%.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
        DOF_types[off + 0].KP, DOF_types[off + 0].KD, DOF_types[off + 0].KI, DOF_types[off + 1].KP, DOF_types[off + 1].KD,
        DOF_types[off + 1].KI, DOF_types[off + 2].KP, DOF_types[off + 2].KD, DOF_types[off + 2].KI, DOF_types[off + 3].KP,
        DOF_types[off + 3].KD, DOF_types[off + 3].KI, DOF_types[off + 4].KP, DOF_types[off + 4].KD, DOF_types[off + 4].KI,
        DOF_types[off + 5].KP, DOF_types[off + 5].KD, DOF_types[off + 5].KI, DOF_types[off + 6].KP, DOF_types[off + 6].KD,
        DOF_types[off + 6].KI, DOF_types[off + 7].KP, DOF_types[off + 7].KD, DOF_types[off + 7].KI);
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
