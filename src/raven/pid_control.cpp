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
 * pid_control.c - control law
 *     pdControl - PD controller
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * Modified 8/28/06 by Hawkeye
 *   Rewrote jointLimits function
 * Modified 8/26/11 by Hawkeye
 *   Modified for Raven_II
 *
 *   \todo update motor velocity data from zero
 */

/**
 * \brief PD and PI controllers for control
 *
 * \ingroup Control
 *
 * \todo play with the feed-forward friction term in the PD controller
 */

#include <cstdlib>
#include "pid_control.h"
#include "log.h"
#include "utils.h"
#include "t_to_DAC_val.h"
#include "homing.h"

extern DOF_type DOF_types[];
extern unsigned long int gTime;

/**
 * \brief calculates PD control (or PI) for
 */

void mpos_PD_control(DOF *joint, int reset_I) {
  float err = 0.0;
  float errVel = 0.0;
  static float errInt[MAX_MECH * MAX_DOF_PER_MECH] = {0};
  float pTerm = 0.0, vTerm = 0.0, iTerm = 0.0;
  float friction_feedforward = 0.0;

  float kp = DOF_types[joint->type].KP;
  float kd = DOF_types[joint->type].KD;
  float ki = DOF_types[joint->type].KI;

  /* PD CONTROL LAW */

  // Calculate error
  err = joint->mpos_d - joint->mpos;
  errVel = joint->mvel_d - joint->mvel;

  // Calculate position term
  pTerm = err * kp;

  // Calculate velocity term
  vTerm = errVel * kd;

  // Calculate integral
  if (reset_I)
    errInt[joint->type] = 0;
  else
    errInt[joint->type] += err * ONE_MS;

  // Calculate integral term
  iTerm = errInt[joint->type] * ki;

  // Calculate feedforward friction term
  //    errSign = err < 0 ? -1 : 1;
  //    if (fabs(err) >= eps) {
  //        friction_feedforward = errSign * friction_comp_torque[joint->type];
  //    }
  //    else {
  //        friction_feedforward = err * friction_comp_torque[joint->type] /
  //        eps;
  //    }

  // Finally place torque
  joint->tau_d = pTerm + vTerm + iTerm + friction_feedforward;
}

/**
*    jointVelControl()
*       Move joints at constant rate.
*/
float jvel_PI_control(DOF *_joint, int resetI) {
  // Set gains.  Gains have been "empirically" tuned.
  // TODO: move this to a permanent place.
  float ki;
  float kv[MAX_MECH * MAX_DOF_PER_MECH] = {0};
  kv[SHOULDER_GOLD] = kv[SHOULDER_GREEN] = (0.528 / (15 DEG2RAD));
  kv[ELBOW_GOLD] = kv[ELBOW_GREEN] = (0.528 / (15 DEG2RAD));
  kv[Z_INS_GOLD] = kv[Z_INS_GREEN] = (0.400 / 0.1);
  kv[TOOL_ROT_GOLD] = kv[TOOL_ROT_GREEN] = (0.005 / (15 DEG2RAD));
  kv[WRIST_GOLD] = kv[WRIST_GREEN] = 0;
  kv[GRASP1_GOLD] = kv[GRASP1_GREEN] = 0;
  kv[GRASP2_GOLD] = kv[GRASP2_GREEN] = 0;

  static float jVelIntErr[MAX_MECH * MAX_DOF_PER_MECH];  // Integral of velocity error

  // Reset integral term
  if (resetI) {
    jVelIntErr[_joint->type] = 0;
    return 0;
  }
  float jVelErr;

  // Calculate joint velocity error
  if (is_toolDOF(_joint))
    jVelErr = _joint->mvel_d - _joint->mvel;
  else
    jVelErr = _joint->jvel_d - _joint->jvel;

  // Integrate error over 1ms
  jVelIntErr[_joint->type] += jVelErr * ONE_MS;
  ki = kv[_joint->type] * 0.1 * 0;

  // Calculate PI velocity control
  _joint->tau_d = (kv[_joint->type] * jVelErr + ki * jVelIntErr[_joint->type]);

  return jVelIntErr[_joint->type];
}
