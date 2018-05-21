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
 *mapping.h - contains constants involving mapping from master to slave
 *Mitch Lum, July 25, 2006
 *BioRobotics Lab
 *
*/

//#include <rtai.h>
// Hack to get rid of annoying compiler warnings w/ math.h
/*#ifdef __attribute_used__
#undef __attribute_used__
#endif
#ifdef __attribute_pure__
#undef __attribute_pure__
#endif*/

#include <cmath>
#include <tf/transform_datatypes.h>

#include "struct.h"
#include "defines.h"

// Rotation about Ymaster into Slave Frame
// green arm using +1.5707 was 45 deg off in actual, so we hacked this number
#define Y_ROT_GREEN_ARM -1.5707
#define Y_ROT_GOLD_ARM 1.5707

void masterToSlave(position *, int);
void fromITP(position *, tf::Quaternion &, int);
