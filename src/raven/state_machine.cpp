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
 * \file state_machine.c
 * \author Kenneth Fodero
 * \version 2005
 *
 * \brief this file gets the desired runlevel from the rcvdParams or PLC
 */

#include "state_machine.h"
#include "log.h"

extern int initialized;    // Defined in rt_process_preempt
extern int NUM_MECH;       // Defined in rt_process_preempt
extern int soft_estopped;  // Defined in rt_process_preempt
extern int globalTime;
#include <sys/times.h>
tms dummy_times;

/**
 * \brief This function puts data in a state machine
 * \param device0 robot_device struct defined in DS0.h
 * \param currParam param_pass struct defined in DS1.h
 * \param rcvdParams param_pass struct
 *
 * In SOFTWARE_RUNLEVEL mode, get desired runlevel from the rcvdParams.
 * In PLC_RUNLEVELS mode, get desired runlevel from the PLC via atmel inputs.
 * If the two PLC's give different runlevels, select  the lowest of the two.
 *
 * \todo diagram of state machine and its effects on other functions
 *
 */
void stateMachine(device *device0, param_pass *currParams, param_pass *rcvdParams) {
  static int rlDelayCounter = 0;  // This is a software workaround to a PLC
                                  // switching transient.  Wait two cycles for
                                  // the delay.

  u_08 rlDesired;
  u_08 *rl = &(currParams->runlevel);
  int i;
  u_08 tmp;
  rlDesired = 9;  // arbitrary large number

  // Checks runlevel of all mechanisms. Lowest runlevel is chosen.
  for (i = 0; i < NUM_MECH; i++) {
    tmp = (device0->mech[i].inputs & (PIN_PS0 | PIN_PS1)) >> 6;
    if (tmp < rlDesired) {
      rlDesired = tmp;
    }
  }

  // already in desired runlevel.  Exit.
  if (*rl == rlDesired) {
    return;
  } else if (rlDelayCounter < 3) {
    rlDelayCounter++;
    return;
  }

  rlDelayCounter = 0;
  *rl = rlDesired;          // Update Run Level
  device0->runlevel = *rl;  // Log runlevels in DS0.
  log_msg("Entered runlevel %d", *rl);

  if (*rl == RL_E_STOP) {
    if (soft_estopped) {
      err_msg("Software e-stop.\n");
      soft_estopped = FALSE;
    }

    err_msg("*** ENTERED E-STOP STATE ***\n");

    initialized = FALSE;
    currParams->sublevel = 0;
  }
}
