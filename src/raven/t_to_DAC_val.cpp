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
 * brief Provide functionality for determining the correct DAC value for a given
 *torque
 *
 * \author Kenneth Fodero
 * \author Hawkeye King
 *
 * \ingroup Control
 */

/*
 * t_to_DAC_val.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * Modified by Hawkeye King
 */

#include "t_to_DAC_val.h"
#include "motor.h"
#include "utils.h"
#include "log.h"

extern DOF_type DOF_types[];
extern int NUM_MECH;

extern unsigned int soft_estopped;

/**
 * \brief Converts desired torque on each joint to desired DAC level
 *
 *	This function loops over all of the joints in the device to cal the
 *conversion calculation for the desired torque values.
 *	There are checks for the mechanism connection status and software
 *e-stops.
 *
 * \pre tau_d has been set for each joint
 * \post current_cmd is set for each joint
 * \param device0 pointer to device structure
 *
 */
int TorqueToDAC(device *device0) {
  int i, j;

  // for each arm
  for (i = 0; i < NUM_MECH; i++)
    for (j = 0; j < MAX_DOF_PER_MECH; j++) {
      if (device0->mech[i].joint[j].type == NO_CONNECTION_GOLD ||
          device0->mech[i].joint[j].type == NO_CONNECTION_GREEN) {
        continue;
      }

      device0->mech[i].joint[j].current_cmd =
          tToDACVal(&(device0->mech[i].joint[j]));  // Convert torque to DAC value

      if (soft_estopped) device0->mech[i].joint[j].current_cmd = 0;
    }
  return 0;
}

/**
 * \brief Takes a torque value and DOF and returns the appropriate
 *   encoder value.  This function could be reduced to one line, but that would
 *be un-readable.
 *
 * inputs - torque - the desired torque
 *          dof - the degree of freedom we are using
 *
 * \param joint pointer to DOF structure
 * \output DAC value
 */
short int tToDACVal(DOF *joint) {
  int DACVal, offset;
  short int result;
  float TFamplifier, TFmotor;

  int j_index = joint->type;

#ifdef DAC_TEST  // treat the desired torque as the desired DAC output
  TFmotor = 1;
  TFamplifier = 1;
  offset = 0;

// offset = DOF_types[j_index].DAC_zero_offset;
#else
  TFmotor = 1 / DOF_types[j_index].tau_per_amp;  // Determine the motor TF  = 1/(tau per amp)
  TFamplifier = DOF_types[j_index].DAC_per_amp;  // Determine the amplifier TF = (DAC_per_amp)
  offset = DOF_types[j_index].DAC_zero_offset;
#endif
  // compute DAC value: DAC=[tau*(amp/torque)*(DACs/amp)+zero_offset(DACs)]
  DACVal = (int)(joint->tau_d * TFmotor * TFamplifier + offset);

  // Perform range checking and convert to short int
  // Note: toShort saturates at max value for short int.
  toShort(DACVal, &result);

  return result;
}

/**
 * /brief sets DACs to 0V
 *
 * input: buffer_out
 * \param device0 pointer to device structure
 */
void clearDACs(device *device0) {
  int i, j;

  // Set all encoder values to no movement
  for (i = 0; i < NUM_MECH; i++)
    for (j = 0; j < MAX_DOF_PER_MECH; j++) device0->mech[i].joint[j].current_cmd = 0;
}

/**
 * \brief testing function for validating the control boards
 */

int TorqueToDACTest(device *device0) {
  static int count;
  int i, j;
  static unsigned int output = 0x4000;
  if (output < 0x8000)
    output = 0xa000;
  else
    output = 0x6000;
  // for each arm
  count++;

  for (i = 0; i < NUM_MECH; i++) {
    for (j = 0; j < MAX_DOF_PER_MECH; j++) {
      device0->mech[i].joint[j].current_cmd = output;
    }
  }

  return 0;
}
