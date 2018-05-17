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

/*********************************************
 **
 **
 **  DOF_type.h
 **
 **	DOF types contains a data structure containing parameters
 **     for a DOF that remain static from power on to power off of
 **     the surgical robot.
 **
 *********************************************/

/********************************************
 *
 * DOF_type - Struct for storing data that remains constant
 *  from power on to power off of the surgical robot.
 *
 */

#ifndef __DOF_type__
#define __DOF_type__

#define MAX_WINDOW_SIZE 1000 /* ms */
#define DAC_STORE_SIZE 10    /* s */
#define HISTORY_SIZE 10

struct Window {
  int length;
  int isFull;
  int first, last;
  float data[MAX_WINDOW_SIZE];
};

struct DOF_type {
  // joint physical limit in degrees
  float max_position;

  // joint software limits in degrees
  float max_limit;
  float min_limit;

  // starting position in degrees
  float home_position;

  // encoder counts per revolution
  int enc_cnts;

  int DAC_max;

  // DOF current variables
  float i_max;
  float i_cont;

  // Motor Transmission Ratio
  float TR;

  // Torque per amp - motor dependent
  float tau_per_amp;

  // DAC counts per amp - different for high / low current amps.
  float DAC_per_amp;

  float DAC_zero_offset;

  // Controller Gains
  float KP;
  float KD;
  float KI;

  // Old position data
  int filterRdy;
  float old_mpos[HISTORY_SIZE];
  float old_filtered_mpos[HISTORY_SIZE];
  float old_mpos_d[HISTORY_SIZE];

  // Old velocity data
  float old_mvel[HISTORY_SIZE];
  float old_mvel_d[HISTORY_SIZE];

  // Length of time motor has been overdriven
  int overdrive_time;
};

#endif
