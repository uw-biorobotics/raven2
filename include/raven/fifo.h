/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
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
 * fifo.h - FIFO handling routines
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

//Rtai Include Files
//#include <rtai.h>
//#include <rtai_sched.h>
//#include <rtai_fifos.h>

//Include Files
#include "struct.h" //Includes DS0, DS1, DOF_type
#include "defines.h"

#include "stddef.h" //For size_t

#define NO_PACKET_FOUND   0
#define CMD_PACKET_RCVD   1
#define BAD_PACKET_FND    2

int putFIFOData(int FIFO, struct device *device0);
int putFIFODataSized(int FIFO, void* data, size_t size);
int getFIFOData(int FIFO, struct param_pass *rcv_params);



