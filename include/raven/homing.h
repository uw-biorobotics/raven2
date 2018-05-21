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
*   \file homing.h
*
*	\brief Based on concept by UCSC, I implement a procedure for joint
*position discovery from relative encoders.
*
*	\author	Hawkeye King
*
*   \date 3-Nov-2011
*
*   \ingroup Control
*/
#include "DS0.h"

/** prototype for homing()
 */
void homing(DOF *);

/** prototype for homing()
 */
void homing(DOF *, tool);

/** prototype for check_homing_condition()
 */
int check_homing_condition(DOF *);


#ifdef RAVEN_II_SQUARE
// RII_Square has higher limits on tool b/c there's more friction
const int homing_max_dac[8] = {2500,  // shoulder
                               2500,  // elbow
                               1200,  // z-ins
                               0,
                               2800,   // tool_rot // was 1400, lowered to reduce
                                       // calibration error //I think this is
                                       // labeled improperly - AL
                               2200,   // wrist
                               2300,   // grasp1
                               2300};  // grasp2
#else
#ifdef DV_ADAPTER
const int homing_max_dac[8] = {2500,  // shoulder
                               2500,  // elbow
                               1400,  // z-ins
                               0,
                               2000,   // tool_rot // was 1400, lowered to reduce
                                       // calibration error //I think this is
                                       // labeled improperly - AL
                               2400,   // wrist
                               2000,   // grasp1
                               2000};  // grasp2
#else
const int homing_max_dac[8] = {2100,  // shoulder
                               2100,  // elbow
                               1600,  // 1900,  //z_ins
                               0,
                               1900,   // tool_rot  //rasised from 1400 alewis 3/4/14
                               2100,   // wrist
                               2250,   // grasp1 decreased from 1900
                               2250};  // grasp2 decreased from 1900
#endif
#endif