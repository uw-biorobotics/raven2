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

#include <dynamic_reconfigure/server.h>
#include <raven_2_crtk/Raven2Config.h>
#include <ros/ros.h>
#include "reconfigure.h"

offsets offsets_l;
offsets offsets_r;

// Dynamic reconfigure callback
void reconfigure_callback(raven_2_crtk::Raven2Config &config, uint32_t level) {
  ROS_INFO("Reconfigure request : %f  ", config.shoulder_l);
  offsets_l.shoulder_off = config.shoulder_l * M_PI / 180.0;
  offsets_l.elbow_off = config.elbow_l * M_PI / 180.0;
  offsets_l.insertion_off = config.insertion_l * 0.01;
  offsets_l.roll_off = config.roll_l * M_PI / 180.0;
  offsets_l.wrist_off = config.wrist_l * M_PI / 180.0;
  offsets_l.grasp1_off = config.grasp1_l * M_PI / 180.0;
  offsets_l.grasp2_off = config.grasp2_l * M_PI / 180.0;

  offsets_r.shoulder_off = config.shoulder_r * M_PI / 180.0;
  offsets_r.elbow_off = config.elbow_r * M_PI / 180.0;
  offsets_r.insertion_off = config.insertion_r * 0.01;
  offsets_r.roll_off = config.roll_r * M_PI / 180.0;
  offsets_r.wrist_off = config.wrist_r * M_PI / 180.0;
  offsets_r.grasp1_off = config.grasp1_r * M_PI / 180.0;
  offsets_r.grasp2_off = config.grasp2_r * M_PI / 180.0;

  // do nothing for now
}
