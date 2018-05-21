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

/*
 * r2_kinematics.h
 *
 *  Created on: Jun 18, 2012
 *      Author: biorobotics
 */

#ifndef R2_KINEMATICS_H_
#define R2_KINEMATICS_H_

#include <tf/transform_datatypes.h>
#include "DS0.h"
#include "defines.h"

enum l_r { dh_left = 0, dh_right = 1, dh_l_r_last = 2 };
enum ik_valid_sol {
  ik_valid = 0,
  ik_invalid = 1,
  ik_valid_sol_last = 2,
};
/** \ ik_solution
 *  \brief  Holds a solution to the Raven inverse kinematics
 *
 */
struct ik_solution {
  int invalid;  ///< set to ik_invalid if a solution is not allowed for any
  /// reason
  l_r arm;     ///< Which arm (Left or Right)
  double th1;  ///< Theta 1
  double th2;  ///< Theta 2
  double d3;   ///< prismatic joint
  double th4;  ///< Theta 4 (tool roll)
  double th5;  ///< Theta 5
  double th6;  ///< Theta 6 (jaw)
};

const ik_solution ik_zerosol = {ik_valid, dh_left, 0, 0, 0, 0, 0, 0};

// Robot constants
const double La12 = 75 * M_PI / 180;
const double La23 = 52 * M_PI / 180;
const double La3 = 0;
const double V = 0;
const double d4 = -0.47;  // m
// const double d4 = -0.482; // 0.482 for daVinci tools  // m test value with
// connector
// const double Lw = 0.009;   // m
const double Lw = 0.013;  // m // .013 for raven II tools, 0.009 for daVinci tools
const double GM1 = sin(La12), GM2 = cos(La12), GM3 = sin(La23), GM4 = cos(La23);

void print_tf(tf::Transform);
void print_btVector(tf::Vector3 vv);
tf::Transform getFKTransform(int a, int b);

void showInverseKinematicsSolutions(device *d0, int runlevel);

int r2_fwd_kin(device *d0, int runlevel);
int getATransform(mechanism &in_mch, tf::Transform &out_xform, int frameA, int frameB);

/** fwd_kin()
 *   Runs the Raven II forward kinematics to determine end effector position.
 *   Inputs:  6 element array of joint angles ( float j[] = {shoulder, elbow,
 * ins, roll, wrist, grasp} )
 *            Arm type, left / right ( kin.armtype arm = left/right)
 *   Outputs: cartesian transform as 4x4 transformation matrix ( bullit
 * transform.  WHAT'S THE SYNTAX FOR THAT???)
 *   Return: 0 on success, -1 on failure
 */
int __attribute__((optimize("0")))
fwd_kin(double in_j[6], l_r in_armtype, tf::Transform &out_xform);

int r2_inv_kin(device *d0, int runlevel);

/** inv_kin()
 *   Runs the Raven II INVERSE kinematics to determine end effector position.
 *   Inputs:  cartesian transform as 4x4 transformation matrix ( bullit
 * transform.  WHAT'S THE SYNTAX FOR THAT???)
 *            Arm type, left / right ( kin.armtype arm = left/right)
 *   Outputs: 6 element array of joint angles ( float j[] = {shoulder, elbow,
 * ins, roll, wrist, grasp} )
 *   Return: 0 on success, -1 on failure
 */
int inv_kin(tf::Transform in_xf, l_r in_arm, ik_solution iksol[8]);

void joint2theta(double *out_iktheta, double *in_J, l_r);
void theta2joint(ik_solution in_iktheta, double *out_J);

#endif /* R2_KINEMATICS_H_ */
