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

/**
 *   \file homing.cpp
 *
 *	\brief Based on concept by UCSC, a procedure is implemented for joint
 *position
 *			 discovery from incremental encoders.
 *
 *	\desc raven_homing called 1000 times per sec during INIT mode. Moves
 *joints
 * 			to their limits (hard stop indicated by increased current)
 *and then
 *			to their predefined "home" position.
 *
 *	\fn These are the 5 functions in homing.cpp file.
 *           Functions marked with "*" are called explicitly from other files.
 * 	       *(1) raven_homing	 	:uses (2)(3)(4)(5), utils.cpp
 *(2)(3)(4)(6), inv_cable_coupling.cpp (1),
 *                                                     trajectory.cpp (3),
 *t_to_DAC_val.cpp (1), pid_control.cpp (1)(2)
 *       	(2) set_joints_known_pos	:uses (2)(3)(4)(5), utils.cpp
 *(3)(4), state_estimate.cpp (2)(3),
 *                                                     inv_cable_coupling.cpp
 *(1), fwd_cable_coupling.cpp (2)
 * 		(3) homing(joint)		:uses trajectory.cpp
 *(1)(2)(7)(8)
 * 		(4) homing(joint,tool)
 * 		(5) check_homing_condition
 *
 *	\author Hawkeye King
 *
 *       \date 3-Nov-2011
 *
 *	\ingroup Control
 */

#include <cstdlib>

#include "trajectory.h"
#include "pid_control.h"
#include "defines.h"
#include "inv_cable_coupling.h"
#include "fwd_cable_coupling.h"
#include "t_to_DAC_val.h"
#include "homing.h"
#include "state_estimate.h"
#include "log.h"

#include <iostream>

int set_joints_known_pos(mechanism *_mech, int tool_only);

extern unsigned long int gTime;
extern DOF_type DOF_types[];

/**
 *   \fn int raven_homing(device *device0, param_pass *currParams, int
 *begin_homing)
 *
 *	\brief Move to hard stops in controlled way, "zero" the joint value, and
 *then move to "home" position
 *
 *	\desc This function is called 1000 times per second during INIT mode
 *		This function operates in two phases:
 *    -# Discover joint position by running to hard stop. Using PD control (I
 *term zero'd) we move the
 *           joint at a smooth rate until current increases which indicates
 *hitting hard mechanical stop.
 *    -# Move joints to "home" position.  In this phase the robot moves from the
 *joint limits to a
 *			designated pose in the center of the workspace.
 *
 *  	\param device0           Which  (top level) device to home (usually
 *only one device per system)
 *   \param currParams        Current parameters (for Run Level)
 *   \param begin_homing      Flag to start the homing process
 *
 *   \ingroup Control
 *
 *	\return 0
 *
 *   \todo   Homing limits should be Amps not DAC units (see homing()).
 */
int raven_homing(device *device0, param_pass *currParams, int begin_homing) {
  static int homing_inited = 0;
  static unsigned long int delay, delay2;
  DOF *_joint = NULL;
  mechanism *_mech = NULL;
  int i = 0, j = 0;

#ifdef RICKS_TOOLS  // Refers to Enders Game Prop manager!!
  //  these were wooden dummy tools for the movie.
  _mech = NULL;
  _joint = NULL;

  while (loop_over_joints(device0, _mech, _joint, i, j)) {
    if (is_toolDOF(_joint)) {
      _joint->state = jstate_ready;
    }
  }
#endif

  // Only run in init mode
  if (!(currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT)) {
    homing_inited = 0;
    delay = gTime;
    return 0;  // return if we are in the WRONG run level (PLC state)
  }

  // Wait a short time for amps to turn on
  if (gTime - delay < 1000) {
    return 0;
  }
  // Initialize the homing sequence.
  if (begin_homing || !homing_inited)  // only do this the first time through
  {
    // Zero out joint torques, and control inputs. Set joint.state=not_ready.
    _mech = NULL;
    _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i, j))  // foreach (joint)
    {
      _joint->tau_d = 0;
      _joint->mpos_d = _joint->mpos;
      _joint->jpos_d = _joint->jpos;
      _joint->jvel_d = 0;
      _joint->state = jstate_not_ready;

      if (is_toolDOF(_joint)) jvel_PI_control(_joint, 1);  // reset PI control integral term
      homing_inited = 1;
    }
    log_msg("Homing sequence initialized");
  }

  // Specify motion commands
  _mech = NULL;
  _joint = NULL;
  while (loop_over_joints(device0, _mech, _joint, i, j)) {
    // Initialize tools first.
    if (is_toolDOF(_joint)) {
      homing(_joint, device0->mech[i].mech_tool);

    } else if (tools_ready(&(device0->mech[i]))) {
      homing(_joint);
    }
  }

  // Inverse Cable Coupling
  invCableCoupling(device0, currParams->runlevel);

  // Do PD control on all joints
  _mech = NULL;
  _joint = NULL;
  while (loop_over_joints(device0, _mech, _joint, i, j)) {
    mpos_PD_control(_joint);
  }

  // Calculate output DAC values
  TorqueToDAC(device0);

  // Check homing conditions and set joint angles appropriately.
  _mech = NULL;
  _joint = NULL;
  while (loop_over_joints(device0, _mech, _joint, i, j)) {
    DOF *_joint = &(_mech->joint[j]);  ///\todo is this line necessary?

    // Check to see if we've reached the joint limit.
    if (check_homing_condition(_joint)) {
      log_msg("Found limit on joint %d cmd: %d \t", _joint->type, _joint->current_cmd,
              DOF_types[_joint->type].DAC_max);
      _joint->state = jstate_hard_stop;
      _joint->current_cmd = 0;
      stop_trajectory(_joint);
      log_msg("joint %d checked ", j);
    }

    // For each mechanism, check to see if the mech is finished homing.
    if (j == (MAX_DOF_PER_MECH - 1)) {
      /// if we're homing tools, wait for tools to be finished
      if ((!tools_ready(_mech) && _mech->joint[TOOL_ROT].state == jstate_hard_stop &&
           _mech->joint[WRIST].state == jstate_hard_stop &&
           _mech->joint[GRASP1].state == jstate_hard_stop)  // \TODO check for grasp2 - bug?
          ||
          (tools_ready(_mech) && _mech->joint[SHOULDER].state == jstate_hard_stop &&
           _mech->joint[ELBOW].state == jstate_hard_stop &&
           _mech->joint[Z_INS].state == jstate_hard_stop)) {
        if (delay2 == 0) delay2 = gTime;

        if (gTime > delay2 + 200)  // wait 200 ticks for cables to settle down
        {
          set_joints_known_pos(_mech, !tools_ready(_mech));  // perform second phase
          delay2 = 0;
        }
      }
    }
  }

  return 0;
}

/**
 *   \fn int set_joints_known_pos(mechanism* _mech, int tool_only)
 *
 *	\brief  Set joint angles to known values after hard stops are reached.
 *
 *	\desc Set all the mechanism joints to known reference angles.
 *       Propagate the joint angle to motor position and encoder offset.
 *
 *   \param _mech       which mechanism (gold/green)
 *   \param tool_only   flag which initializes only the tool/wrist joints
 *
 *	\ingroup Control
 *
 *	\return 0
 *
 * 	\todo  Rationalize the sign changes on GREEN_ARM vs GOLD_ARM (see IFDEF
 *below).
 * 	\todo  This MAYBE needs to be changed to support device specific
 *parameter changes read from a config file or ROS service.
 */
int set_joints_known_pos(mechanism *_mech, int tool_only) {
  DOF *_joint = NULL;
  int j = 0;

  int scissor =
      ((_mech->mech_tool.t_end == mopocu_scissor) || (_mech->mech_tool.t_end == potts_scissor)) ? 1
                                                                                                : 0;

  log_msg("setting joints known");

  //    int offset = 0;
  //    if (_mech->type == GREEN_ARM) offset = 8;
  /// Set joint position reference for just tools, or all DOFS
  _joint = NULL;
  while (loop_over_joints(_mech, _joint, j)) {
    // when tool joints finish, set positioning joints to neutral
    if (tool_only && !is_toolDOF(_joint->type)) {
      // Set jpos_d to the joint limit value.
      _joint->jpos_d = DOF_types[_joint->type].home_position;  // keep non tool joints from moving
    }

    // when positioning joints finish, set tool joints to nothing special
    else if (!tool_only && is_toolDOF(_joint->type)) {
      _joint->jpos_d = _joint->jpos;
    }
    // when tool or positioning joints finish, set them to max_angle
    else {
      // Set jpos_d to the joint limit value.
      if (scissor && ((_joint->type == GRASP2_GREEN) || (_joint->type == GRASP2_GOLD))) {
        _joint->jpos_d = _mech->mech_tool.grasp2_min_angle;
        log_msg("setting grasp 2 to     arm %d,     joint %d to    value %f",
                _mech->mech_tool.mech_type, _joint->type, _joint->jpos_d);
      } else
        _joint->jpos_d = DOF_types[_joint->type].max_position;

      // Initialize a trajectory to operating angle
      _joint->state = jstate_homing1;
    }
  }
  /// Inverse cable coupling: jpos_d  ---> mpos_d
  // use_actual flag triggered for insertion axis
  invMechCableCoupling(_mech, 1);

  _joint = NULL;
  while (loop_over_joints(_mech, _joint, j)) {
    // Reset the state-estimate filter
    _joint->mpos = _joint->mpos_d;
    resetFilter(_joint);

    // Convert the motor position to an encoder offset.
    // mpos = k * (enc_val - enc_offset)  --->  enc_offset = enc_val - mpos/k
    float f_enc_val = _joint->enc_val;
    int j_flip = 1;

    // Encoder values on Gold arm are reversed.  See also state_machine.cpp
    switch (_mech->mech_tool.t_style) {
      case dv:
        if (_mech->type == GOLD_ARM && !is_toolDOF(_joint)) {
          f_enc_val *= -1.0;
        } else if (_mech->type == GREEN_ARM) {
          j_flip *= -1.0;
        }
        break;
      case square_raven:
        if ((_mech->type == GOLD_ARM && !is_toolDOF(_joint)) ||
            (_mech->type == GREEN_ARM &&
             is_toolDOF(_joint))  // Green arm tools are also reversed with square pattern
            )
          f_enc_val *= -1.0;
        break;
      default:
        if (_mech->type == GOLD_ARM || is_toolDOF(_joint)) {
          f_enc_val *= -1.0;
        }
        if (_mech->type == GREEN_ARM) {
          j_flip *= -1.0;
        }
        break;
    }
#ifdef OPPOSE_GRIP
    if (j == GRASP1) {
      f_enc_val *= -1;  // switch encoder value for opposed grasp
    }
#endif

    if (j == Z_INS) {
      j_flip *= -1;
    }

    /// Set the joint offset in encoder space.
    float cc = ENC_CNTS_PER_REV / (2 * M_PI);
    _joint->enc_offset = f_enc_val - (_joint->mpos_d * cc);

    // set the joint encoder offset in encoder space
    float j_enc_val = j_flip * _joint->joint_enc_val;

    if ((j == SHOULDER) || (j == ELBOW)) {  // if joint is shoulder or elbow
      cc = ROTARY_JOINT_ENC_PER_REV / (2 * PI);
      // log_msg("joint		%d", j);
      // log_msg("cc value			= %f", cc);
    } else if (j == Z_INS) {
      cc = LINEAR_JOINT_ENC_PER_M;
      // log_msg("joint		%d", j);
      // log_msg("cc value			= %f", cc);
    }

    if (!is_toolDOF(_joint->type)) {
      // log_msg("j_enc value			= %f", j_enc_val);
      // log_msg("j_pos_d				= %f", _joint->jpos_d);
      // log_msg("j_pos_d*cc				= %f", _joint->jpos_d *
      // cc);

      _joint->joint_enc_offset = j_enc_val - (_joint->jpos_d * cc);
    }

    getStateLPF(_joint, _mech->tool_type);
    // getStateLPF(_joint, _mech->mech_tool.t_style);
  }

  fwdMechCableCoupling(_mech);
  return 0;
}

/**
 *	\fn void homing(DOF* _joint)
 *
 *	\brief Set trajectory behavior for each joint during the homing process.
 *
 *   \param _joint The joint being controlled.
 *
 * 	\ingroup Control
 *
 *	\return void
 *
 *   \todo  Homing limits should be Amps not DAC units
 *   \todo  Change square vs. diamond to a config-file based runtime system
 *instead of #ifdef
 */
void homing(DOF *_joint) {
  // duration for homing of each joint
  const float f_period[MAX_MECH * MAX_DOF_PER_MECH] = {1, 1, 1, 9999999, 1, 1, 1, 1,
                                                       1, 1, 1, 9999999, 1, 1, 1, 1};
// degrees for homing of each joint
#ifdef RAVEN_II_SQUARE
  // roll is backwards because of the 'click' in the mechanism
  const float f_magnitude[MAX_MECH * MAX_DOF_PER_MECH] = {
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, -80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, -80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD};
#else
#ifdef DV_ADAPTER
  const float f_magnitude[MAX_MECH * MAX_DOF_PER_MECH] = {
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD};
#else  // default
  const float f_magnitude[MAX_MECH * MAX_DOF_PER_MECH] = {
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD};
#endif
#endif

  switch (_joint->state) {
    case jstate_wait:
      break;

    case jstate_not_ready:
      // Initialize velocity trajectory
      // log_msg("Starting homing on joint %d", _joint->type);
      _joint->state = jstate_pos_unknown;
      start_trajectory_mag(_joint, f_magnitude[_joint->type], f_period[_joint->type]);
      break;

    case jstate_pos_unknown:
      // Set desired joint trajectory
      update_linear_sinusoid_position_trajectory(_joint);
      break;

    case jstate_hard_stop:
      // Wait for all joints. No trajectory here.

      break;

    case jstate_homing1:
      start_trajectory(_joint, DOF_types[_joint->type].home_position, 2.5);
      _joint->state = jstate_homing2;
      break;

    case jstate_homing2:
      // Move to start position
      // Update position trajectory
      if (!update_position_trajectory(_joint)) {
        _joint->state = jstate_ready;
        log_msg("Joint %d ready", _joint->type);
      }
      break;

    default:
      // not doing joint homing.
      break;

  }  // switch

  return;
}

/**
 *	\fn void homing(DOF* _joint, tool a_tool)
 *
 *	\brief Set trajectory behavior for each tool joint during the homing
 *process.
 *
 *   \param _joint The joint being controlled.
 *	\param a_tool The tool being controlled.
 *
 * 	\ingroup Control
 *
 *	\return void
 *   \TODO refactor for tool class
 */
void homing(DOF *_joint, tool a_tool) {
  // duration for homing of each joint
  const float f_period[MAX_MECH * MAX_DOF_PER_MECH] = {1, 1, 1, 9999999, 1, 1, 1, 1,
                                                       1, 1, 1, 9999999, 1, 1, 1, 1};
  //    // degrees for homing of each joint
  //#ifdef RAVEN_II_SQUARE
  //    //roll is backwards because of the 'click' in the mechanism
  //    const float f_magnitude[MAX_MECH*MAX_DOF_PER_MECH] = {-10 DEG2RAD, 10
  //    DEG2RAD, 0.02, 9999999, -80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
  //                                                          -10 DEG2RAD, 10
  //                                                          DEG2RAD, 0.02,
  //                                                          9999999, -80
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD};
  //#else
  //#ifdef DV_ADAPTER
  //    const float f_magnitude[MAX_MECH*MAX_DOF_PER_MECH] = {-10 DEG2RAD, 10
  //    DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
  //                                                          -10 DEG2RAD, 10
  //                                                          DEG2RAD, 0.02,
  //                                                          9999999, 80
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD};
  //#else //default
  //    const float f_magnitude[MAX_MECH*MAX_DOF_PER_MECH] = {-10 DEG2RAD, 10
  //    DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
  //                                                          -10 DEG2RAD, 10
  //                                                          DEG2RAD, 0.02,
  //                                                          9999999, 80
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD, 40
  //                                                          DEG2RAD};
  //#endif
  //#endif

  // check if scissors
  int scissor = ((a_tool.t_end == mopocu_scissor) || (a_tool.t_end == potts_scissor)) ? 1 : 0;

  float f_magnitude[MAX_MECH * MAX_DOF_PER_MECH] = {
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
      -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD};

  if (a_tool.t_style == square_raven) {
    if (a_tool.mech_type == GOLD_ARM)
      f_magnitude[TOOL_ROT_GOLD] = -80 DEG2RAD;
    else if (a_tool.mech_type == GREEN_ARM)
      f_magnitude[TOOL_ROT_GREEN] = -80 DEG2RAD;
  }
  if (scissor) {
    if (a_tool.mech_type == GOLD_ARM)
      f_magnitude[GRASP2_GOLD] = -40 DEG2RAD;
    else if (a_tool.mech_type == GREEN_ARM)
      f_magnitude[GRASP2_GREEN] = -40 DEG2RAD;
  }

  switch (_joint->state) {
    case jstate_wait:
      break;

    case jstate_not_ready:
      // Initialize velocity trajectory
      // log_msg("Starting homing on joint %d", _joint->type);
      _joint->state = jstate_pos_unknown;
      start_trajectory_mag(_joint, f_magnitude[_joint->type], f_period[_joint->type]);
      break;

    case jstate_pos_unknown:
      // Set desired joint trajectory
      update_linear_sinusoid_position_trajectory(_joint);
      break;

    case jstate_hard_stop:
      // Wait for all joints. No trajectory here.

      break;

    case jstate_homing1:
      start_trajectory(_joint, DOF_types[_joint->type].home_position, 2.5);
      _joint->state = jstate_homing2;
      break;

    case jstate_homing2:
      // Move to start position
      // Update position trajectory
      if (!update_position_trajectory(_joint)) {
        _joint->state = jstate_ready;
        log_msg("Joint %d ready", _joint->type);
      }
      break;

    default:
      // not doing joint homing.
      break;

  }  // switch

  return;
}

//homing limits moved to homing.h


/**
 *  \fn int check_homing_condition(DOF *_joint)
 *
 * 	\brief Monitor joint currents to end the homing cycle at hard stop.
 *
 *  \desc Checks to see if a joint current is above a certain max value which
 *indicates that the
 *			joint has reached it's mechanical limit.
 *
 *  \param _joint    A joint struct
 *
 * 	\ingroup Control
 *
 *	\return 1 if DAC output is greater than the maximum allowable
 *			0 otherwise
 *
 *  \todo Homing limits should be Amps not DAC units (see homing()).
 */
int check_homing_condition(DOF *_joint) {
  if (_joint->state != jstate_pos_unknown) return 0;

  // check if the DAC output is greater than the maximum allowable.
  // Note, current_cmd is an integer, so using abs (not fabs) is okay.
  if (abs(_joint->current_cmd) >= homing_max_dac[_joint->type % 8]) {
    return 1;
  }

  return 0;
}
