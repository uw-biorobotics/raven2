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
*
*
*  teleoperation.h
*
*    I define datastructures representing the
*  information passed between master and slave
*  in teleoperation.
*
*  Based on the wave variables naming schema:
*
*  u_struct passes from master to slave
*  v_struct passes from slave to master
*
*********************************************/

#ifndef TELEOPERATION_H
#define TELEOPERATION_H
#define SURGEON_ENGAGED 1
#define SURGEON_DISENGAGED 0

/*
u_struct : structure passed from master to slave.
This struct defines an incremental movment packet type.

sequence     Packet's sequence number
pactyp       protocol version in use
version      Protocol version number  (***SRI)

delx[2]	     position increment
dely[2]
delz[2]
delyaw[2]    Orientation increment
delpitch[2]
delroll[2]
buttonstate[2]
grasp[2]        +32767 = 100% closing torque, -32768 = 100% opening
surgeon_mode    SURGEON_ENGAGED or SURGEON_DISENGAGED  (formerly Pedal_Down or
Pedal_UP)
checksum
*/

struct u_struct {
  unsigned int sequence;
  unsigned int pactyp;
  unsigned int version;

  int delx[2];
  int dely[2];
  int delz[2];
  double Qx[2];
  double Qy[2];
  double Qz[2];
  double Qw[2];
  int buttonstate[2];
  int grasp[2];
  int surgeon_mode;
  int checksum;
} __attribute__((__packed__));

/*
v_struct: Return DS from slave to master.
sequence
pactyp        protocol version in use
version       Protocol version number  (***SRI)
fx            X force
fy            Y force
fz            Z force
runlevel      Slave operating state
jointflags    bit flags for each joint limit (up to 16 joints).
checksum
*/
struct v_struct {
  unsigned int sequence;
  unsigned int last_sequence;
  unsigned int pactyp;
  unsigned int version;
  int fx[2];
  int fy[2];
  int fz[2];
  int runlevel;
  unsigned int jointflags;
  int checksum;
} __attribute__((__packed__));

#endif  // teleoperation_h
