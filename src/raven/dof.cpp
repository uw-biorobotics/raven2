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
 * dof.c - functions that fill in the DOF structure
 *     processEncVal - process an encoder value from a USB packet
 *     encToJPos - go from an encoder value to a Joint position
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

/**
*   \file dof.cpp
*
*        \brief This is a file to translate motor encoder values to robot
*position or joint angles.
*
*        \fn These are the 4 functions in dof.cpp file.
*            Functions marked with "*" are called explicitly from other files.
*             *(1) processEncVal
*              (2) encToMPos               :uses (3)
*              (3) encToMPos2              :uses (4)
*              (4) normalizedEncCnt
*              (5) encToJPos               :This function is declared in dof.h
*file but never defined here.
*
*       \author Hawkeye King
*
*       \date ??
*
*       \ingroup Control
*/

#include "dof.h"

extern DOF_type DOF_types[];

/**
 * processEncVal - reads an encoder value from a USB packet buffer
 *   and returns the integer result
 *
 * \param buffer[] - the USB packet buffer
 * \param channel - the encoder channel to load
 *
 * \return the resulting encoder value
 */
int processEncVal(unsigned char buffer[], int channel) {
  int result;

  // encoder value is in bytes 3,4,and 5 of the usb in-packet
  // see atmel_code/main.c: in_packet() for more info
  result =
      (buffer[3 * channel + 5] << 16) | (buffer[3 * channel + 4] << 8) | (buffer[3 * channel + 3]);

  // Handle negative values by padding result with ones
  if (result >> 23) {
    result = result | 0xFF000000;
  }
#ifdef RAVEN_I
  return result;
#else
  return (-1 * result);
#endif
}

/**
 * encToMPos - converts an encoder count to motor position. This function sets
 *the mpos parameter of the joint structure
 *
 * \param joint pointer to degree of freedom to work on
 *
 */
void encToMPos(DOF *joint) {
  // MPos is just the motor angle
  joint->mpos = encToMPos2(joint);
}

/**
* Similar returns an angle corresponding to encoder value contained in joint
* parameter. Function normalizes values so that they are returned measured from
* start position
*  \param joint Pointer to structure containing joint info
*  \return angle of the encoder
*/
float encToMPos2(DOF *joint) {
  float motorAngle;
  int normEnc;

  // Adjust encoder value - based on start position
  normEnc = normalizeEncCnt(joint);

  // Determine motor angle
  motorAngle = (2 * PI) * (1 / ((float)ENC_CNTS_PER_REV)) * normEnc;

  // MPos is just the motor angle
  return motorAngle;
}

/**
 * normalizeEncCnt - adjusts encVal based on middle pos.
 *
 * \param encVal the encoder value
 * \param dof the degree of freedom
 *
 * \return normalized encoder value
 */
int normalizeEncCnt(DOF *joint) { return (joint->enc_val - joint->enc_offset); }
