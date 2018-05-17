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
 * t_to_DAC_val.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

//#include <rtai.h>

// Local include files
#include "struct.h" /*Includes DS0, DS1, DOF_type*/

// These are not used, but kept in for reference
//#define DAC_MAX_V   10 /*  10V max on DAC */
//#define DAC_MIN_V  -10 /* -10V min on DAC */
//#define TF_DAC  ((float)2*DAC_MAX_V/(float)DAC_STEPS) //
//(DAC_MAX_V-DAC_MIN_V)/DAC_STEPS = Volts / Dac_increment

#define DAC_STEPS 65536 /* 2^16 steps avail */

// Wrappers for c++
#ifdef __cplusplus
extern "C" {
#endif

short int tToDACVal(DOF *joint);
void clearDACs(device *device0);

int TorqueToDAC(device *device0);
int TorqueToDACTest(device *device0);  // Square wave for timing test

#ifdef __cplusplus
}
#endif
