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

/**\file globals.cpp
 *
 *    \brief  define Global variables
 *
 *    \fn     There is no function this file.
 *
 *    \author Kenneth Fodero
 *            Biorobotics Lab
 *
 *    \date 2005
 */

#include "struct.h"  // DS0, DS1, DOF_types defines
#include "USB_init.h"

// unsigned long int gTime = 0;

DOF_type DOF_types[MAX_MECH * MAX_DOF_PER_MECH];
USBStruct USBBoards;

// tool gold_arm_tool (bipolar_forceps, GOLD_ARM);
// tool gold_arm_tool(large_needle, GOLD_ARM);
tool gold_arm_tool(r_grasper, GOLD_ARM);
// tool gold_arm_tool(micro_forceps, GOLD_ARM);

//#ifdef SCISSOR_RIGHT
// tool green_arm_tool(mopocu_scissor, GREEN_ARM);
//#else
// tool green_arm_tool(large_needle,  GREEN_ARM);
//#endif

// tool green_arm_tool(mopocu_scissor, GREEN_ARM);
// tool green_arm_tool(potts_scissor, GREEN_ARM);
tool green_arm_tool(r_grasper, GREEN_ARM);
// tool green_arm_tool(bipolar_forceps, GREEN_ARM);
