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
 * \author Kenneth Fodero
 * \date 2005
 * \ingroup Network
 */

#include "put_USB_packet.h"
#include "USB_init.h"
#include "update_atmel_io.h"

extern unsigned long int gTime;
extern USBStruct USBBoards;

/**\fn void putUSBPackets(device *device0)
  \brief Takes data from robot to send to USB board(s)
  \struct device
  \param device0 pointer to device struct
  \ingroup Network
 */

void putUSBPackets(device *device0) {
  int mech_index = 0;

  // Loop through all USB Boards
  for (int i = 0; i < USBBoards.activeAtStart; i++) {
    // don't put anything on the joint encoder board
    if (USBBoards.boards[i] != JOINT_ENC_SERIAL) {
      if (putUSBPacket(USBBoards.boards[i], &(device0->mech[mech_index])) == -USB_WRITE_ERROR) {
        log_msg("Error writing to USB Board %d!\n", USBBoards.boards[i]);
      }

      // only increment mechanisms if it was a mech board that was processed
      mech_index++;
    } else if (putJointEncUSBPacket(USBBoards.boards[i]) == -USB_WRITE_ERROR) {
      log_msg("Error writing to joint enc USB Board %d!\n", USBBoards.boards[i]);
    }
  }
}

/**\fn int putUSBPacket(int id, mechanism *mech)
  \brief Takes data from mech  and uses it to fill a USB
   packet on specified board

  \param mech pointer to mechanism struct
  \param id the usb board id number (serial#)
  \return success of the operation
  \ingroup Network
 */

int putUSBPacket(int id, mechanism *mech) {
  // encoder boards don't have any output yet
  if (id == JOINT_ENC_SERIAL) {
    return 0;
  }

  int i = 0;
  unsigned char buffer_out[MAX_OUT_LENGTH];

  buffer_out[0] = DAC;               // Type of USB packet
  buffer_out[1] = MAX_DOF_PER_MECH;  // Number of DAC channels

  for (i = 0; i < MAX_DOF_PER_MECH; i++) {
    // Factor in offset since we are in midrange operation
    mech->joint[i].current_cmd += DAC_OFFSET;

    buffer_out[2 * i + 2] = (char)(mech->joint[i].current_cmd);
    buffer_out[2 * i + 3] = (char)(mech->joint[i].current_cmd >> 8);

    // Remove offset
    mech->joint[i].current_cmd -= DAC_OFFSET;
  }

  // Set PortF outputs
  buffer_out[OUT_LENGTH - 1] = mech->outputs;

  // Write the packet to the USB Driver
  if (usb_write(id, &buffer_out, OUT_LENGTH) != OUT_LENGTH) {
    return -USB_WRITE_ERROR;
  }

  return 0;
}

/**\fn int putJointEncUSBPacket(int id)
  \brief 		sends empty packet to specified Joint Enc board

  \param id 	the usb board id number (serial#)
  \return 		success of the operation
  \ingroup Network
 */
int putJointEncUSBPacket(int id) {
  unsigned char buffer_out[MAX_OUT_LENGTH];

  buffer_out[0] = DAC;               // Type of USB packet
  buffer_out[1] = MAX_DOF_PER_MECH;  // Number of DAC channels

  for (int i = 0; i < MAX_DOF_PER_MECH; i++) {
    // send 0 values (at DAC_OFFSET)
    buffer_out[2 * i + 2] = (char)DAC_OFFSET;
    buffer_out[2 * i + 3] = (char)(DAC_OFFSET >> 8);
  }

  // Set PortF outputs to zero
  buffer_out[OUT_LENGTH - 1] = (char)0;

  // Write the packet to the USB Driver
  if (usb_write(id, &buffer_out, OUT_LENGTH) != OUT_LENGTH) {
    return -USB_WRITE_ERROR;
  }

  return 0;
}
