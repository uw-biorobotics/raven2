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

/**\file update_atmel_io.cpp
 * \brief
 * \author Kenneth Fodero
 * \author Hawkeye King
 * \date 2005
 * \ingroup Network
 */

#include "update_atmel_io.h"
#include "log.h"

extern int initialized;
extern int soft_estopped;
extern int NUM_MECH;
extern unsigned long int gTime;

/**\fn void updateAtmelOutputs(device *device0, int runlevel)
 * \brief updates the software output: 8 bits of each arm (identical for both
 * arms) are reserved(5 bits are used) for this output.
 *        This output will later be written to the USB as hardware input.
 * \param device0 - pointer to device struct
 * \param runlevel - current runlevel
 * \return void
 * \ingroup Hardware
 */
void updateAtmelOutputs(device *device0, int runlevel) {
  static int counter;
  unsigned char i, outputs = 0x00;

  // Update Foot Pedal
  if ((runlevel > 1) && (device0->surgeon_mode)) outputs |= PIN_FP;

  // Update Ready
  if (initialized) outputs |= PIN_READY;

  // Update Linux State
  outputs |= (runlevel & (PIN_LS0 | PIN_LS1));

  // Update WD Timer - if not software triggered
  if (!soft_estopped) {
    if (counter <= (WD_PERIOD / 2)) {
      outputs |= PIN_WD;
    } else if (counter >= WD_PERIOD) {
      counter = 0;
    }
  }

  // Write Changes
  for (i = 0; i < NUM_MECH; i++) device0->mech[i].outputs = outputs;

  counter++;
}

/**\fn void updateAtmelInputs(device device0, int runlevel)
 * \brief reads the hardware output received from the USB package and retrieves
 * the runlevel in PLC.
 *        This function is mainly for debugging purpose.
 * \param device0 - device struct
 * \param runlevel - current runlevel
 * \return void
 * \ingroup Hardware
 * \ingroup Debug
 * \todo I don't think this function does anything - nothing is returned and I
 * don't think the device is updated
 */
void updateAtmelInputs(device device0, int runlevel) {
  // unsigned char inputs;
  int PLCState;

  // Update PLC State
  PLCState = (device0.mech[0].inputs & (PIN_PS0 | PIN_PS1)) >> 6;

  //  printk("Mech0.inputs: %#x\n",device0.mech[0].inputs);
  //  static int j;
  // if (j++ % 1000 == 0)
  //  printk("PLC State is %d\n",PLCState);
  // if (PLCState != runlevel)
  // printk("RunLevels on Linux Box and PLC do not match!!!\n");
}
