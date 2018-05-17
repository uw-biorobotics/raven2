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

/**\file mapping.cpp
 * \brief contains functions involving mapping from master to slave
 * \author Mitch Lum
 * \author BioRobotics Lab
 * \date July 25, 2006
 */

#include "mapping.h"
#include "log.h"
#include <iostream>

const int USE_ITP = 1;

const static double d2r = M_PI / 180;
float xRot_rad = -25 * d2r;

/** \fn void fromITP(position *delpos, tf::Quaternion &delrot, int armserial)
 * \brief Transform a position increment and an orientation increment from ITP
 * coordinate frame into local robot zero coordinate frame.
 *        Do this using inv(R)*C*R : R= transform, C= increment
 * \param delpos - a pointer points to a position struct
 * \param delrot - a reference of a btQuanternion class
 * \param armserial - an integer number of of mechanisam id
 * \question why post multiply with R inverse?
*/
void fromITP(position *delpos, tf::Quaternion &delrot, int armserial) {
#ifdef ORIENTATION_V

  const tf::Transform ITP2Gold(tf::Matrix3x3(0, 0, -1, 0, 1, 0, 1, 0, 0), tf::Vector3(0, 0, 0));
  const tf::Transform ITP2Green(tf::Matrix3x3(0, 0, -1, 0, 1, 0, 1, 0, 0), tf::Vector3(0, 0, 0));

  const tf::Transform GoldZ25(
      tf::Matrix3x3(cos(-xRot_rad), -sin(-xRot_rad), 0, sin(-xRot_rad), cos(-xRot_rad), 0, 0, 0, 1),
      tf::Vector3(0, 0, 0));
  const tf::Transform GreenZ25(
      tf::Matrix3x3(cos(xRot_rad), -sin(xRot_rad), 0, sin(xRot_rad), cos(xRot_rad), 0, 0, 0, 1),
      tf::Vector3(0, 0, 0));

// const tf::Transform GoldZ25  ( tf::Matrix3x3 (0.906,0.423,0,  -0.423,0.906,0,
// 0,0,1), tf::Vector3 (0,0,0) );
// const tf::Transform GreenZ25( tf::Matrix3x3 (0.906,-0.423,0,  0.423,0.906,0,
// 0,0,1), tf::Vector3 (0,0,0) );

#else

  const tf::Transform ITP2Gold(tf::Matrix3x3(0, 0, -1, -1, 0, 0, 0, 1, 0), tf::Vector3(0, 0, 0));
  const tf::Transform ITP2Green(tf::Matrix3x3(0, 0, -1, 1, 0, 0, 0, -1, 0), tf::Vector3(0, 0, 0));

#endif

  tf::Transform incr(delrot, tf::Vector3(delpos->x, delpos->y, delpos->z));

  if (armserial == GOLD_ARM_SERIAL) {
    incr = ITP2Gold * incr * ITP2Gold.inverse();
  } else {
    incr = ITP2Green * incr * ITP2Green.inverse();
  }

#ifdef ORIENTATION_V
  if (armserial == GOLD_ARM_SERIAL) {
    incr = GoldZ25 * incr * GoldZ25.inverse();
  } else {
    incr = GreenZ25 * incr * GreenZ25.inverse();
  }
#endif
  delrot = incr.getRotation();
  delpos->x = (int)(incr.getOrigin()[0]);
  delpos->y = (int)(incr.getOrigin()[1]);
  delpos->z = (int)(incr.getOrigin()[2]);
}
