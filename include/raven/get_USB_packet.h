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
 * put_USB_packet.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

// RTAI + Linux include files
//#include <linux/module.h> //used for jiffies
//#include <rtai.h>

// Include files
#include "struct.h" /*Includes DS0, DS1, DOF_type*/
#include "dof.h"
#include "USB_init.h"

/* USB packet lengths */
#define IN_LENGTH 27 /* 27 with input pins */

// Function prototypes
void initiateUSBGet(device *device0);
int getUSBPackets(device *device0);

void processEncoderPacket(mechanism *mech, unsigned char buffer[]);

int getUSBPacket(int id, device *dev, int index);
void processJointEncoderPacket(device *dev, unsigned char buffer[]);
