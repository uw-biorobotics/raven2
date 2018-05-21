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
 * tools.cpp
 *
 * \brief Class file for Tool object
 *
 * This is a new feature added to the RAVEN code in the indigo branch. This
 * allows us to use different tools on each arm and allows for differences
 * between tools and scissors. The interface type needs to be the same between
 * each arm (RAVEN or dV adapter) and must still be declared as a #define in
 *defines.h
 *
 * \date Oct 17, 2014
 * \author Andrew Lewis
 * \author Danying Hu
 * \author David Caballero
 *
 */

#include "tools.h"
#include "defines.h"

/** tool object constructor
*
*/
tool::tool(end_effector_type t_end_new, int a_mech) {
  set_tool(t_end_new, a_mech);
  set_tool_data();
}

/** creates new tool and sets style according to end effector type
 *  \param t_end_new   type of end effector
 *  \param a_mech      which arm is the tool mounted on
 *  \return void
 */
void tool::set_tool(end_effector_type t_end_new, int a_mech) {
  t_end = t_end_new;
  mech_type = a_mech;
  if (t_end == r_grasper)
    t_style = raven;
  else if (t_end == r_sq_grasper)
    t_style = square_raven;
  else
    t_style = dv;
}

/** assigns tool attribute values by calling other functions
 * \return void
 */
void tool::set_tool_data() {
  set_limits();
  set_home_angles();
  set_wrist_coupling();
  set_DH_params();
  set_max_opening_angle();
}

/** sets default tool limits based on the end effector type
 * \return void
 */
void tool::set_limits() {
  switch (t_end) {
    case r_grasper:
      rot_max_angle = 330 DEG2RAD;
      rot_min_angle = -330 DEG2RAD;  // might not be correct
      rot_max_limit = 182 DEG2RAD;
      rot_min_limit = -182 DEG2RAD;

      wrist_max_angle = 100 DEG2RAD;
      wrist_min_angle = -100 DEG2RAD;
      wrist_max_limit = 70 DEG2RAD;
      wrist_min_limit = -70 DEG2RAD;

      grasp1_max_angle = 120 DEG2RAD;
      grasp1_min_angle = -120 DEG2RAD;
      grasp1_max_limit = 105 DEG2RAD;
      grasp1_min_limit = -105 DEG2RAD;

      grasp2_max_angle = 120 DEG2RAD;
      grasp2_min_angle = -120 DEG2RAD;
      grasp2_max_limit = 105 DEG2RAD;
      grasp2_min_limit = -105 DEG2RAD;
      break;

    case r_sq_grasper:
      rot_max_angle = 330 DEG2RAD;
      rot_min_angle = -330 DEG2RAD;  // might not be correct
      rot_max_limit = 182 DEG2RAD;
      rot_min_limit = -182 DEG2RAD;

      wrist_max_angle = 115 DEG2RAD;
      wrist_min_angle = -115 DEG2RAD;
      wrist_max_limit = 75 DEG2RAD;
      wrist_min_limit = -75 DEG2RAD;

      grasp1_max_angle = 135 DEG2RAD;
      grasp1_min_angle = -135 DEG2RAD;
      grasp1_max_limit = 85.0 DEG2RAD;
      grasp1_min_limit = -85.0 DEG2RAD;

      grasp2_max_angle = 135 DEG2RAD;
      grasp2_min_angle = -135 DEG2RAD;
      grasp2_max_limit = 85.0 DEG2RAD;
      grasp2_min_limit = -85.0 DEG2RAD;
      break;

    case large_needle:
      rot_max_angle = 275 DEG2RAD;   // from 260
      rot_min_angle = -275 DEG2RAD;  // might not be correct
      rot_max_limit = 220 DEG2RAD;
      rot_min_limit = -220 DEG2RAD;

      wrist_max_angle = 95 DEG2RAD;
      wrist_min_angle = -95 DEG2RAD;
      wrist_max_limit = 75 DEG2RAD;
      wrist_min_limit = -75 DEG2RAD;

      grasp1_max_angle = 115 DEG2RAD;
      grasp1_min_angle = -115 DEG2RAD;
      grasp1_max_limit = 85 DEG2RAD;
      grasp1_min_limit = -85 DEG2RAD;

      grasp2_max_angle = 115 DEG2RAD;
      grasp2_min_angle = -115 DEG2RAD;
      grasp2_max_limit = 85 DEG2RAD;
      grasp2_min_limit = -85 DEG2RAD;
      break;

    case micro_forceps:
      rot_max_angle = 275 DEG2RAD;   // from 260
      rot_min_angle = -275 DEG2RAD;  // might not be correct
      rot_max_limit = 182 DEG2RAD;
      rot_min_limit = -182 DEG2RAD;

      wrist_max_angle = 85 DEG2RAD;
      wrist_min_angle = -85 DEG2RAD;
      wrist_max_limit = 75 DEG2RAD;
      wrist_min_limit = -75 DEG2RAD;

      grasp1_max_angle = 150 DEG2RAD;
      grasp1_min_angle = -150 DEG2RAD;
      grasp1_max_limit = 85 DEG2RAD;
      grasp1_min_limit = -85 DEG2RAD;

      grasp2_max_angle = 150 DEG2RAD;
      grasp2_min_angle = -150 DEG2RAD;
      grasp2_max_limit = 85 DEG2RAD;
      grasp2_min_limit = -85 DEG2RAD;
      break;

    case bipolar_forceps:
      rot_max_angle = 260 DEG2RAD;   // from 260
      rot_min_angle = -260 DEG2RAD;  // might not be correct
      rot_max_limit = 182 DEG2RAD;
      rot_min_limit = -182 DEG2RAD;

      wrist_max_angle = 70 DEG2RAD;
      wrist_min_angle = -70 DEG2RAD;
      wrist_max_limit = 65 DEG2RAD;
      wrist_min_limit = -65 DEG2RAD;

      grasp1_max_angle = 140 DEG2RAD;
      grasp1_min_angle = -140 DEG2RAD;
      grasp1_max_limit = 85 DEG2RAD;
      grasp1_min_limit = -85 DEG2RAD;

      grasp2_max_angle = 140 DEG2RAD;
      grasp2_min_angle = -140 DEG2RAD;
      grasp2_max_limit = 85 DEG2RAD;
      grasp2_min_limit = -85 DEG2RAD;
      break;

    case cardiere_forceps:
      rot_max_angle = 260 DEG2RAD;   // from 260
      rot_min_angle = -260 DEG2RAD;  // might not be correct
      rot_max_limit = 182 DEG2RAD;
      rot_min_limit = -182 DEG2RAD;

      wrist_max_angle = 90 DEG2RAD;
      wrist_min_angle = -90 DEG2RAD;
      wrist_max_limit = 85 DEG2RAD;
      wrist_min_limit = -85 DEG2RAD;

      grasp1_max_angle = 100 DEG2RAD;
      grasp1_min_angle = -100 DEG2RAD;
      grasp1_max_limit = 85 DEG2RAD;
      grasp1_min_limit = -85 DEG2RAD;

      grasp2_max_angle = 100 DEG2RAD;
      grasp2_min_angle = -100 DEG2RAD;
      grasp2_max_limit = 85 DEG2RAD;
      grasp2_min_limit = -85 DEG2RAD;
      break;

    case mopocu_scissor:
      rot_max_angle = 260 DEG2RAD;
      rot_min_angle = -260 DEG2RAD;  // might not be correct
      rot_max_limit = 250 DEG2RAD;
      rot_min_limit = -250 DEG2RAD;

      wrist_max_angle = 90 DEG2RAD;
      wrist_min_angle = -90 DEG2RAD;
      wrist_max_limit = 75 DEG2RAD;
      wrist_min_limit = -75 DEG2RAD;

      grasp1_max_angle = 100 DEG2RAD;
      grasp1_min_angle = -100 DEG2RAD;
      grasp1_max_limit = 85 DEG2RAD;
      grasp1_min_limit = -85 DEG2RAD;

      grasp2_max_angle = 105 DEG2RAD;
      grasp2_min_angle = -105 DEG2RAD;
      grasp2_max_limit = 95 DEG2RAD;
      grasp2_min_limit = -95 DEG2RAD;
      break;

    case potts_scissor:
      rot_max_angle = 260 DEG2RAD;
      rot_min_angle = -260 DEG2RAD;  // might not be correct
      rot_max_limit = 220 DEG2RAD;
      rot_min_limit = -220 DEG2RAD;

      wrist_max_angle = 85 DEG2RAD;
      wrist_min_angle = -85 DEG2RAD;
      wrist_max_limit = 75 DEG2RAD;
      wrist_min_limit = -75 DEG2RAD;

      grasp1_max_angle = 130 DEG2RAD;
      grasp1_min_angle = -130 DEG2RAD;
      grasp1_max_limit = 110 DEG2RAD;
      grasp1_min_limit = -110 DEG2RAD;

      grasp2_max_angle = 130 DEG2RAD;
      grasp2_min_angle = -130 DEG2RAD;
      grasp2_max_limit = 110 DEG2RAD;
      grasp2_min_limit = -110 DEG2RAD;
      break;

    case monopolar_cautery:
      break;

    default:
      break;
  }
}

/** sets joint angles to return to after initializing based on end effector type
 * \return void
 */
void tool::set_home_angles() {
  switch (t_end) {
    case r_grasper:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 45 DEG2RAD;
      grasp2_home_angle = 45 DEG2RAD;
      break;

    case r_sq_grasper:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 45 DEG2RAD;
      grasp2_home_angle = 45 DEG2RAD;
      break;

    case large_needle:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 45 DEG2RAD;
      grasp2_home_angle = 45 DEG2RAD;
      break;

    case micro_forceps:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 45 DEG2RAD;
      grasp2_home_angle = 45 DEG2RAD;
      break;

    case bipolar_forceps:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 45 DEG2RAD;
      grasp2_home_angle = 45 DEG2RAD;
      break;

    case cardiere_forceps:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 45 DEG2RAD;
      grasp2_home_angle = 45 DEG2RAD;
      break;

    case mopocu_scissor:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 10 DEG2RAD;
      grasp2_home_angle = 10 DEG2RAD;
      break;

    case potts_scissor:
      rot_home_angle = 0;
      wrist_home_angle = 0;
      grasp1_home_angle = 5 DEG2RAD;
      grasp2_home_angle = 5 DEG2RAD;
      break;

    case monopolar_cautery:
      break;

    default:
      break;
  }
}

/** sets wrist/grasper coupling coefficient based on end effector type
 * \return void
 */
void tool::set_wrist_coupling() {
  switch (t_end) {
    case r_grasper:
      wrist_coupling = 0;
      break;

    case r_sq_grasper:
      wrist_coupling = 0;
      break;

    case large_needle:
      wrist_coupling = 0.5;
      break;

    case micro_forceps:
      wrist_coupling = 0.5;
      break;

    case bipolar_forceps:
      wrist_coupling = 0.64;
      break;

    case cardiere_forceps:
      wrist_coupling = 0.7;
      break;

    case mopocu_scissor:
      wrist_coupling = 0.5;
      break;

    case potts_scissor:
      wrist_coupling = 0.5;
      break;

    case monopolar_cautery:
      break;

    default:
      wrist_coupling = 0.0;
      break;
  }
}

/** sets shaft and wrist DH parameters based on end effector type
 * \return void
 */
void tool::set_DH_params() {
  switch (t_end) {
    case r_grasper:
      shaft_length = 0.482;
      wrist_length = 0.013;
      break;

    case r_sq_grasper:
      shaft_length = 0.482;
      wrist_length = 0.013;
      break;

    case large_needle:
      shaft_length = 0.482;
      wrist_length = 0.009;
      break;

    case micro_forceps:
      shaft_length = 0.482;
      wrist_length = 0.009;
      break;

    case bipolar_forceps:
      shaft_length = 0.482;
      wrist_length = 0.011;
      break;

    case cardiere_forceps:
      shaft_length = 0.482;
      wrist_length = 0.009;
      break;

    case mopocu_scissor:
      shaft_length = 0.482;
      wrist_length = 0.009;
      break;

    case potts_scissor:
      shaft_length = 0.482;
      wrist_length = 0.009;
      break;

    case monopolar_cautery:
      break;

    default:
      shaft_length = 0.5;
      wrist_length = 0.009;
      break;
  }
}

/** sets safety limit for opening angle based on end effector type
 * \return void
 */
void tool::set_max_opening_angle() {
  switch (t_end) {
    case r_grasper:
      max_opening_angle = 120 DEG2RAD;
      break;

    case r_sq_grasper:
      max_opening_angle = 120 DEG2RAD;
      break;

    case large_needle:
      max_opening_angle = 120 DEG2RAD;
      break;

    case micro_forceps:
      max_opening_angle = 150 DEG2RAD;
      break;

    case bipolar_forceps:
      max_opening_angle = 150 DEG2RAD;
      break;

    case cardiere_forceps:
      max_opening_angle = 150 DEG2RAD;
      break;

    case mopocu_scissor:
      max_opening_angle = 40 DEG2RAD;
      break;

    case potts_scissor:
      max_opening_angle = 20 DEG2RAD;
      break;

    case monopolar_cautery:
      break;

    default:
      max_opening_angle = 20 DEG2RAD;
      break;
  }
}
