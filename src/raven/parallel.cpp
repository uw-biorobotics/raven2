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
* \file parallel.cpp
*
* \author Kenneth Fodero
* \date 2005
*
*/

#include "parallel.h"
#include "log.h"
#include <sys/io.h>

/**\fn void parallelUpdate(int runlevel, int endOfLoop)
*  \brief
*  \param runlevel
*  \param endOfLoop
*  \return void
*  \ingroup IO
*/

void parallelUpdate(int runlevel, int endOfLoop)
{
  unsigned char data = 0x00;

  //Update RL pins
  data |= ((char)runlevel & 0x03);


  //Update Duty Cycle pins
  if (!endOfLoop)
    data |= DUTY_CYCLE_BIT;

	log_msg("parallel B");

  //Write the data
  outb(data, PARPORT);
	log_msg("parallel C");

}
/* increment counter and write to parallelport */
/**
 * \brief increment counter and write it to parallelport
 * \ingroup IO
 */
void parport_out(void)
{
  #ifdef PARPORT_DEBUG
  static unsigned char state=0;
  outb(state++,PARPORT);
  #endif
}
/**
 * \brief put data out in the parallel port
 * \param out_byte  the actual bits to output
 * \ingroup IO
 */
void parport_out(unsigned char out_byte)
{
  #ifdef PARPORT_DEBUG
  outb(out_byte,PARPORT);
  #endif
}


