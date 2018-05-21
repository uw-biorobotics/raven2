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
 * USB_init.h
 *
 */

#ifndef __USB_INIT_H__
#define __USB_INIT_H__

// Include files
#include <cstdio>
#include <sys/io.h>
#include <fcntl.h>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>
#include <sys/ioctl.h>
#include <vector>

#include "defines.h"
#include "struct.h"
#include "log.h"

// RTAI + LINUX include files
//#include <linux/kernel.h>
//#include <linux/module.h>
//#include <linux/delay.h>
//#include <rtai.h>

#define MAX_BOARD_COUNT 10  /// Maximum number of usb boards

/* USB packet lengths */
#define OUT_LENGTH (3 + MAX_DOF_PER_MECH * 2) /* (3+8*2) w/ output pins */

struct USBStruct {
  std::vector<int> boards;  /// Vector of serial numbers
  int activeAtStart;        /// Number of active boards
};

// Defines

#define MAX_ERROR_COUNT 50
#define USB_WRITE_ERROR 1
#define USB_READ_ERROR 2
#define USB_BUSY_ERROR 3

#define MAX_LOOPS 10
#define USB_INIT_ERROR -1
#define USB_RESET 1

// Function Prototypes
int USBInit(device *device0);
void USBShutdown();

void USBShutdown();

int startUSBRead(int id);
int usb_read(int id, void *buffer, size_t len);
int usb_write(int id, void *buffer, size_t len);

int usb_reset_encoders(int boardid);

#endif
