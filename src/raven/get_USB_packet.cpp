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

/**\file get_USB_packet.cpp
 *
 * 	\brief 	contains functions for initializing the robot
 * 		intializes the DOF structure AND runs initialization routine
 *
 * 	\fn These are the 4 functions in get_USB_packet.cpp file.
 *          Functions marked with "*" are called explicitly from other files.
 * 	       *(1) initiateUSBGet		:uses USB_init.cpp (6)
 * 	       *(2) getUSBPackets		:uses (3)
 * 		(3) getUSBPacket		:uses (4), USB_init.cpp (7)
 * 		(4) processEncoderPacket	:uses dof.cpp (1)
 *
 * 	\author Kenneth Fodero
 *
 * 	\date 2005
 */

#include "get_USB_packet.h"

extern unsigned long int gTime;
extern USBStruct USBBoards;
extern int NUM_MECH;

/**\fn void initiateUSBGet(device *device0)
  \brief Initiate data request from USB Board. Must be called before read
  \struct device
  \param device0 pointer to device struct
 */

void initiateUSBGet(device *device0) {
  int i;
  int err = 0;

  // Loop through all USB Boards
  for (i = 0; i < USBBoards.activeAtStart; i++) {
    err = startUSBRead(USBBoards.boards[i]);
    if (err < 0) {
      log_msg("Error (%d) initiating USB read %d on loop %d!", err, USBBoards.boards[i], gTime);
    }
  }
}

/**\fn int getUSBPackets(device *device0)
  \brief Takes data from USB packet(s) and uses it to fill the
 *   DS0 data structure
  \struct device
  \param device0 pointer to device struct
  \return zero on success and negative on failure
 */

int getUSBPackets(device *device0) {
  int ret = 0;
  int mech_index = 0;

  // Loop through all USB Boards
  for (int i = 0; i < USBBoards.activeAtStart; i++) {
    if (mech_index > NUM_MECH) {
      log_msg("USB/Mech index error");
      return -1;
    }

    int err = getUSBPacket(USBBoards.boards[i], device0, mech_index);

    // only increment mech index if a mechanism board was processed
    if (USBBoards.boards[i] != JOINT_ENC_SERIAL) mech_index++;

    if (err == -EBUSY || ret == -EBUSY)
      ret = -EBUSY;

    else if (err < 0) {
      ret = err;
    }
  }

  return ret;
}

/**\fn int getUSBPacket(int id, mechanism *mech)
  \brief Takes data from a USB packet and uses it to fill the
 *   DS0 data structure
  \struct mechanism the data structure to fill
  \param device	pointer to device struct
  \param id 	the USB board to read from
  \param index	the mechanism index associated with the board
                                unless it's a joint encoder USB, which is
 associated
                                with several mechanisms
  \return zero on success and negative on failure
 */

int getUSBPacket(int id, device *dev, int index) {
  int result, type;
  unsigned char buffer[MAX_IN_LENGTH];

  char joint_enc = (id == JOINT_ENC_SERIAL) ? 1 : 0;

  // Read USB Packet
  result = usb_read(id, buffer, IN_LENGTH);

  // -- Check for read errors --
  if (result < 0) {
    return result;
  }

  // No data or something. Boo.
  else if ((result == 0) || (result != IN_LENGTH))
    return -EIO;

  // -- Good packet so process it --
  type = buffer[0];

  // Load in the data from the USB packet
  switch (type) {
    // Handle and Encoder USB packet
    case ENC:
      if (!joint_enc)
        processEncoderPacket(&(dev->mech[index]), buffer);
      else if (joint_enc)
        processJointEncoderPacket(dev, buffer);
      break;
  }

  return 0;
}

/**\fn void processEncoderPacket(mechanism* mech, unsigned char buffer[])
  \struct mechanism the data structure to fill
  \param mech pointer to mechanism struct
  \param buffer
 */
void processEncoderPacket(mechanism *mech, unsigned char buffer[]) {
  int i, numChannels;
  int encVal;

  // Determine channels of data received
  numChannels = buffer[1];

// Get the input pin status
#ifdef RAVEN_I
  mech->inputs = ~buffer[2];
#else
  mech->inputs = buffer[2];
#endif

  // Loop through and read data for each channel
  for (i = 0; i < numChannels; i++) {
    // Load encoder values
    encVal = processEncVal(buffer, i);
    mech->joint[i].enc_val = encVal;
  }

  return;
}

/**\fn void processJointEncoderPacket(device* dev, unsigned char buffer[])
  \param device		device data structure, joint encoder data will be
  placed in
                                        each mechanism of device
  \param buffer		the USB buffer to parse
 */
void processJointEncoderPacket(device *dev, unsigned char buffer[]) {
  int i, numChannels;
  int encVal;
  mechanism *mech_gold;
  mechanism *mech_green;

  // Determine channels of data received
  numChannels = buffer[1];

  // assume that joint encoder boards don't have PLC inputs
  for (i = 0; i < NUM_MECH; i++) {
    if (dev->mech[i].type == GOLD_ARM) {
      mech_gold = &(dev->mech[i]);
    } else if (dev->mech[i].type == GREEN_ARM) {
      mech_green = &(dev->mech[i]);
    }
  }

  // Loop through and read data for each channel
  // place the values in the appropriate mechanism
  for (i = 0; i < numChannels / 2; i++) {
    // Load encoder values
    encVal = processEncVal(buffer, i);
    mech_gold->joint[i].joint_enc_val = encVal;
  }

  for (i = 0; i < numChannels / 2; i++) {
    // Load green encoder values (4-7) into mech joints 0-3
    encVal = processEncVal(buffer, i + numChannels / 2);
    mech_green->joint[i].joint_enc_val = encVal;
  }

  return;
}
