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

extern DOF_type DOF_types[];
extern int NUM_MECH;
extern volatile int isUpdated;
extern unsigned long gTime;

unsigned int newDofTorqueSetting = 0;  // for setting torque from console
unsigned int newDofTorqueMech = 0;     // for setting torque from console
unsigned int newDofTorqueDof = 0;      //
unsigned int newDofPosSetting = 0;     // for setting torque from console
unsigned int newDofPosMech = 0;        // for setting torque from console
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

  // Switch control modes only in pedal up or init.
  if ((currParams->runlevel == RL_E_STOP) &&
      (currParams->robotControlMode != (int)newRobotControlMode)) {
    currParams->robotControlMode = (int)newRobotControlMode;
    log_msg("Control mode updated");
  }

  // Set new torque command from console user input
  if (newDofTorqueSetting) {
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

  // Set new joint position command from console user input
  // robotControlMode for keyboard control of RAVEN is not yest set
  // TODO: reset to new keyboard mode when implemented
  if (newDofPosSetting && currParams->robotControlMode == -99) {
    // reset all other joints to zero
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

  // Set new surgeon mode
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
*  setDofTorque()
*    Set a torque to output on a joint.
*     Torque input is mNm
*   \param in_mech      Mechinism number of the joint
*   \param in_dof       DOF number
*   \param in_torque    Torque to set the DOF to (in mNm)
*/
void setDofTorque(unsigned int in_mech, unsigned int in_dof, int in_torque) {
  if (((int)in_mech < NUM_MECH) && ((int)in_dof < MAX_DOF_PER_MECH)) {
    newDofTorqueMech = in_mech;
    newDofTorqueDof = in_dof;
    newDofTorqueTorque = in_torque;
    newDofTorqueSetting = 1;
  }
  isUpdated = TRUE;
}

/**
*  addDofPos()
*    Set a pos to output on a joint.
*
*   \param in_mech      Mechinism number of the joint
*   \param in_dof       DOF number
*   \param in_torque    value to add to joint angle - radians or meters
*/
void addDofPos(unsigned int in_mech, unsigned int in_dof, float in_pos) {
  if (((int)in_mech < NUM_MECH) && ((int)in_dof < MAX_DOF_PER_MECH)) {
    newDofPosMech = in_mech;
    newDofPosDof = in_dof;
    newDofPosPos = in_pos;
    newDofPosSetting = 1;
  }
  isUpdated = TRUE;
}
