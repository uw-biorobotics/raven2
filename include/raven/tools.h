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
 * tools.h
 *
 * \brief Class file for tool object
 *
 *  \date Oct 17, 2014
 *  \author Andrew Lewis
 *  \author Danying Hu, David Caballero
 */

#ifndef TOOLS_H_
#define TOOLS_H_

/** Style of tool interface to robot, enumerated
 *
 */

enum style {
  dv,          /**< Da Vinci interface value 1 */
  raven,       /**< RAVEN interface value 2 */
  square_raven /**< Bionics lab square style interface value 3 */
};

/** name of end effector type
 *
 */
enum end_effector_type {
  r_grasper,        /**< RAVEN style grasper value 1 */
  r_sq_grasper,     /**< RAVEN Bionics lab square style grasper value 2 */
  large_needle,     /**< dV large needle driver value 3 */
  micro_forceps,    /**< dV micro forceps value 4 */
  bipolar_forceps,  /**< dV bipolar forceps value 5 */
  cardiere_forceps, /**< dV cardiere forceps value 6 */
  mopocu_scissor,   /**< dV monopolar curved scissors value 7 */
  potts_scissor,    /**< dV potts scissor value 8 */
  monopolar_cautery /**< dV monopolar cuatery tool value 9 */
};

/** a tool that can be used on either RAVEN arm
 *
 */

class tool {
 public:
  end_effector_type t_end; /**< enumerated end effector name */

  style t_style; /**< enumerated robot interface style */

  int mech_type; /**< arm type that the tool is mounted to */

  float wrist_coupling; /**< coupling coefficient between wrist and graspers */

  // angle is the maximum physical range of DOF
  // limit is the safe limit for joint saturation
  float rot_max_angle; /**< Physical maximum angle from the midpoint of tool
                          roll */
  float rot_min_angle; /**< Physical minimum angle from the midpoint of tool
                          roll  */
  float rot_max_limit; /**< Positive safety limit angle from midpoint of tool
                          roll  */
  float rot_min_limit; /**< Negative safety limit angle from midpoint of tool
                          roll  */

  float wrist_max_angle; /**< Physical maximum angle from the midpoint of wrist
                            motion*/
  float wrist_min_angle; /**< Physical minimum angle from the midpoint of wrist
                            motion*/
  float wrist_max_limit; /**< Positive safety limit angle from midpoint of wrist
                            motion */
  float wrist_min_limit; /**< Negative safety limit angle from midpoint of wrist
                            motion */

  float grasp1_max_angle; /**< Physical maximum angle from the midpoint of first
                             jaw motion*/
  float grasp1_min_angle; /**< Physical minimum angle from the midpoint of first
                             jaw motion*/
  float grasp1_max_limit; /**< Positive safety limit angle from midpoint of
                             first jaw motion */
  float grasp1_min_limit; /**< Negative safety limit angle from midpoint of
                             first jaw motion */

  float grasp2_max_angle; /**< Physical maximum angle from the midpoint of
                             second jaw motion*/
  float grasp2_min_angle; /**< Physical minimum angle from the midpoint of
                             second jaw motion*/
  float grasp2_max_limit; /**< Positive safety limit angle from midpoint of
                             second jaw motion */
  float grasp2_min_limit; /**< Negative safety limit angle from midpoint of
                             seond jaw motion */

  float max_opening_angle; /**< Safety limit angle of distance between jaws*/

  // angles to return to after homing
  float rot_home_angle;    /**< tool rotation angle to return to after homing */
  float wrist_home_angle;  /**< wrist angle to return to after homing */
  float grasp1_home_angle; /**< grasper one angle to return to after homing */
  float grasp2_home_angle; /**< grasper two angle to return to after homing */

  // DH (DanyingHu) parameters
  float shaft_length; /**< DH length parameter from middle of tool interface to
                         base of wrist */
  float wrist_length; /**< DH length parameter from shaft/wrist base to midpoint
                         of gasper length */

  // methods

  tool(){};
  tool(end_effector_type, int);

  ~tool(){};

  void set_tool(end_effector_type, int);

  void set_tool_data();

  void set_limits();

  void set_home_angles();

  void set_wrist_coupling();

  void set_DH_params();

  void set_max_opening_angle();
};

#endif /* TOOLS_H_ */
