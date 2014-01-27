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

/*
 * get_USB_packet.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "get_USB_packet.h"
#include "parallel.h"

extern unsigned long int gTime;
extern USBStruct USBBoards;


/**
 * initiateUSBGet() - Initiate data request from USB Board. Must be called before read
 *   DS0 data structure.
 */
void initiateUSBGet(struct device *device0)
{
  int i;
  int err=0;
  
  //Loop through all USB Boards
  for (i = 0; i < USBBoards.activeAtStart; i++)
    {
      err = startUSBRead( USBBoards.boards[1-i] );
      if ( err < 0)
        {
	  //	  log_msg("Error (%d) initiating USB read %d on loop %d!", err, USBBoards.boards[i], gTime);
        }
    }
}

/**
 * getUSBPackets() - Takes data from USB packet(s) and uses it to fill the
 *   DS0 data structure.
 *
 * inputs - device - the data structure to fill
 * outputs - returns zero on success
 *           returns negative on failure
 *
 */
int getUSBPackets(struct device *device0)
{
    int ret = 0;

    //Loop through all USB Boards
    for (int i = 0; i < USBBoards.activeAtStart; i++)
    {
        int err = getUSBPacket( 
			   USBBoards.boards[i], 
			   &(device0->mech[i] ) 
			    );
	if (err == -EBUSY || ret == -EBUSY)
	  ret = -EBUSY;

        else if ( err < 0 )
        {
	  ret = err;
        }
    }

    return ret;
}

/**
 * getUSBPacket() - Takes data from a USB packet and uses it to fill the
 *   DS0 data structure.
 *
 * inputs - mechanism - the data structure to fill
 *          id - the USB board to read from
 *
 * output - returns zero
 *          returns negative on failure
 *
 */
int getUSBPacket(int id, struct mechanism *mech)
{
    int result, type;
    unsigned char buffer[MAX_IN_LENGTH];

    //Read USB Packet
    result = usb_read(id,buffer,IN_LENGTH);

    // -- Check for read errors --
    if (result < 0){
      return result;
    }

    // No data or something. Boo.
    else if ((result == 0) || (result != IN_LENGTH))
      return -EIO;

    // -- Good packet so process it --
    type = buffer[0];

    //Load in the data from the USB packet
    switch (type)
      {
        //Handle and Encoder USB packet
      case ENC:
        processEncoderPacket(mech, buffer);
        break;
      }

    return 0;
}

void processEncoderPacket(struct mechanism* mech, unsigned char buffer[])
{
    int i, numChannels;
    int encVal;

    //Determine channels of data received
    numChannels = buffer[1];

    //Get the input pin status
#ifdef RAVEN_I
    mech->inputs = ~buffer[2];
#else
    mech->inputs = buffer[2];
#endif

    //Loop through and read data for each channel
    for (i = 0; i < numChannels; i++)
    {
        //Load encoder values
        encVal = processEncVal(buffer, i);
        mech->joint[i].enc_val = encVal;
    }
}
