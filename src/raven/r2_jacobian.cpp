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
 * You should have received a copy of the GNU  General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/** Methods for r2_jacobian class
 *
 *
 *  \date May 17, 2016
 *  \author Andy Lewis
 *
 *  \ingroup control
 *  \ingroup Kinematics
 */

#include "DS0.h"
#include <cstdio>
#include <iostream>
#include "r2_jacobian_defs.h"
#include "defines.h"
#include "struct.h"

extern int NUM_MECH;
extern DOF_type DOF_types[];

/** r2_jacobian constructor with pre-set velocities and forces
 *
 *	\param vel[6]    the velocities for constructing the r2_jacobian object
 *	\param f[6]      the forces for constructing the r2_jacobian object
 *
 */
r2_jacobian::r2_jacobian(Eigen::VectorXf vel, Eigen::VectorXf f) {
  set_vel(vel);
  set_force(f);
  return;
}

/** set the velocities of an r2_jacobian
 *
 *	\param Eigen::VectorXf vel   the new velocities for the jacobian
 *
 *	\return void
 */
void r2_jacobian::set_vel(Eigen::VectorXf vel) {
  velocity = vel;
  return;
}

/** get the velocities of an r2_jacobian
 *
 *	\param vel[6]   array to put velocities values into
 *
 *	\return void
 */
void r2_jacobian::get_vel(float vel[6]) {
  for (int i = 0; i < 6; i++) {
    vel[i] = velocity(i);
  }
  return;
}

/** set the forces of an r2_jacobian
 *
 *	\param Eigen::VectorXf f   the new force vector for the jacobian
 *
 *	\return void
 */
void r2_jacobian::set_force(Eigen::VectorXf f) {
  force = f;
  return;
}

/** get the forces of an r2_jacobian
 *
 *	\param f[6]   array to put force values into
 *
 *	\return void
 */
void r2_jacobian::get_force(float f[]) {
  for (int k = 0; k < 6; k++) {
    f[k] = force(k);
  }
  return;
}

/** This function is the bones of the r2_jacobian class
 *
 * \desc 	This function is the bones of the r2_jacobian class -
 * 			it orchestrates the calculation of the Jacobian, and
 * 			end effector velocities and torques.
 *
 * \param  j_pos[6]      the joint states of the RAVEN
 * \param  j_vel[6]      the joint velocities of the RAVEN
 * \param  j_torque[6]   the joint torques of the RAVEN
 *
 * \return int   success = 1
 */
int r2_jacobian::update_r2_jacobian(float j_pos[6], float j_vel[6], float j_torque[6], tool a_tool,
                                    int arm_type) {
  int success = 0;

  // recalculate matrix based on j pos
  success = calc_jacobian(j_pos, a_tool,
                          arm_type);  // success = 1 if jacobian is calculated successfully

  // calculate jacobian velocities
  success &= calc_velocities(j_vel);  // success if velocities also calculated

  // calculate jacobian forces
  success &= calc_forces(j_torque);  // success if velocities also calculated
                                     /*
                                             static int check = 0;
                                             if (check %3000 == 0){
                                                     printf("updated Jacobian! \n");
                                                     std::cout<<j_matrix<<std::endl;
                                                     printf("updated Velocity! \n");
                                                     std::cout<<velocity<<std::endl;
                                                     printf("updated Force! \n");
                                                     std::cout<<force<<std::endl;
                                                     check = 0;
                                             }
                                             check++;
                                     */
  return success;
}

/** re-calculates the jacobian matrix values using the input joint positions
 *
 * \param   j_vel  joint velocities of robot
 *
 * \return int   success = 1
 *
 */
int r2_jacobian::calc_velocities(float j_vel[6]) {
  int success = 0;

  Eigen::VectorXf j_vel_vec(6);

  for (int i = 0; i < 6; i++) {
    j_vel_vec(i) = j_vel[i];
  }

  velocity = j_matrix * j_vel_vec;

  success = 1;

  return success;
}

/** re-calculates the jacobian matrix values using the input joint positions
 *
 * \param   j_torques  joint torques of robot
 *
 * \return int   success = 1
 */
int r2_jacobian::calc_forces(float j_torques[6]) {
  int success = 0;

  Eigen::VectorXf j_torques_vec(6);

  for (int i = 0; i < 6; i++) {
    j_torques_vec(i) = j_torques[i];
  }

  force = j_matrix.transpose().inverse() * j_torques_vec;  // j_matrix * j_torques_vec;

  return success;
}

/** called from rt_raven in order to start the calculation process with a robot
 *device
 *
 * \desc Acts on the jacobian objects in each mechanism of the given device.
 *Doesn't use
 * 		 runlevel yet.
 *
 * \param d0         device to calculate jacobians for
 * \param runlevel	 runlevel of device, not used yet
 *
 * \return int success = 1
 *
 */
int r2_device_jacobian(robot_device *d0, int runlevel) {
  int success = 1;
  float j_pos[6];
  float j_vel[6];
  float j_torque[6];
  // float capstan_torque; //torque at the capstan (after gearbox)
  float grav_t;
  float applied_t;
  int arm_type;
  int offset = 0;
  tool m_tool;

  for (int m = 0; m < NUM_MECH; m++) {
    // populate arrays for updating jacobian
    for (int i = 0; i < 6; i++) {
      offset = (i >= 3) ? 1 : 0;  // skip getting tau for 4, since it is unpopulated

      j_pos[i] = d0->mech[m].joint[i + offset].jpos;
      j_vel[i] = d0->mech[m].joint[i + offset].jvel;

      // we really care about the joint torque applied beyond the gravity torque
      if (i < 3) {
        // capstan torque for the first 3 joints is already calculated
        grav_t = d0->mech[m].joint[i + offset].tau_g;
      } else
        // we haven't calculated gravity torques on the tool joints yet -
        // probably unnecessary?
        grav_t = 0;

      // grab the applied capstan torque
      applied_t = d0->mech[m].joint[i + offset].tau;

      //			printf("forces check! \n");
      //			if((i==0) and (check % 1000 ==
      // 0))std::cout<<grav_t<<",  "<<applied_t<<std::endl;

      // finally, subtract gravity joint torque from applied joint torque to
      // find the non-gravity joint torques
      // and populate the output array

      double gearbox = (i < 3) ? GEAR_BOX_GP42_TR
                               : GEAR_BOX_GP32_TR;  // use the big gearbox for the first 3 joints
      j_torque[i] = (applied_t - grav_t) * DOF_types[i + offset].TR /
                    gearbox;  // t_joint = t_capstan * (torque transfer ratio /
                              // Gear box ratio)
    }

    arm_type = d0->mech[m].type;
    m_tool = d0->mech[m].mech_tool;
    // calculate jacobian values for this mech
    success &= d0->mech[m].r2_jac.update_r2_jacobian(j_pos, j_vel, j_torque, m_tool, arm_type);

    //		static int check = 0;
    //		if (check %2000 == 0){
    //
    //			printf("velocity check! \n");
    //			std::cout<<j_vel[0]<<",  "<<j_vel[1]<<",  "<<j_vel[2]<<",
    //"<<j_vel[3]<<",  "<<j_vel[4]<<",  "<<j_vel[5]<<std::endl;
    //
    //
    //			printf("torque check! \n");
    //			std::cout<<j_torque[0]<<",  "<<j_torque[1]<<",  "<<j_torque[2]<<",
    //"<<j_torque[3]<<",  "<<j_torque[4]<<",  "<<j_torque[5]<<std::endl;
    //			check = 0;
    //		}
    //		check++;
  }

  return success;
}

/** re-calculates the jacobian matrix values using the input joint positions
 *
 * \param   j_pos  joint positions of robot
 *
 * \return int   success = 1
 *
 */
int r2_jacobian::calc_jacobian(float j_pos[6], tool a_tool, int arm_type) {
  int success = 0;

  int lw;
  int d4;
  float dh_alpha[6];  // the remainder of DH params defined in defs.h file
  int d3 = D3;

  // set dh_alpha based on mech type
  if (arm_type == GREEN_ARM_SERIAL) {
    for (int i = 0; i < 6; i++) {
      dh_alpha[i] = dh_alpha_gold[i];
    }
  } else if (arm_type == GOLD_ARM_SERIAL) {
    for (int i = 0; i < 6; i++) {
      dh_alpha[i] = dh_alpha_green[i];
    }
  } else {
    printf(
        "What great magic is this!? A new arm type! I'm not even mad - I'm "
        "impressed.");
  }

  // set tool-specific variables
  d4 = a_tool.shaft_length;
  lw = a_tool.wrist_length;
  // lw

  dh_aa[5] = lw;

  // ********************************************* row 1
  // **********************************************
  j_matrix(0, 0) = L * Ca12 * Ca23 * C5 * lw * S6 + Ca12 * C4 * D3 * Sa23 * S6 +
                   Ca12 * C4 * d4 * Sa23 * S6 + Ca12 * C4 * lw * Sa23 * S5 * S6 -
                   Ca12 * C5 * C6 * d3 * Sa23 * S4 - Ca12 * C5 * C6 * d4 * Sa23 * S4 -
                   C2 * Ca23 * C4 * D3 * Sa12 * S6 - C2 * Ca23 * C4 * d4 * Sa12 * S6 -
                   C2 * Ca23 * C4 * lw * Sa12 * S5 * S6 + C2 * Ca23 * C5 * d3 * Sa12 * S4 +
                   C2 * Ca23 * C5 * C6 * d4 * Sa12 * S4 + C2 * C5 * lw * Sa12 * Sa23 * S6 -
                   C4 * C5 * C6 * d3 * Sa12 * S2 - C4 * C5 * C6 * d4 * Sa12 * Sa12 -
                   d3 * Sa12 * S2 * S4 * S6 - d4 * Sa12 * S2 * S4 * S6 -
                   lw * Sa12 * S2 * S4 * S5 * S6;

  j_matrix(0, 1) = Ca23 * C5 * lw * S6 + C4 * d3 * Sa23 * S6 + C4 * d4 * Sa23 * S6 +
                   C4 * lw * Sa23 * S5 * S6 - C5 * C6 * d3 * Sa23 * S4 - C5 * C6 * d4 * Sa23 * S4;

  j_matrix(0, 2) = C6 * S5;

  j_matrix(0, 3) = -C5 * lw * S6;

  j_matrix(0, 4) = 0;

  j_matrix(0, 5) = 0;  // done
  //************************************************ row 2
  //********************************************
  j_matrix(1, 0) = L * Ca12 * Ca23 * C5 * C6 * lw + Ca12 * C4 * C6 * d3 * Sa23 +
                   Ca12 * C4 * C6 * d4 * Sa23 + Ca12 * C4 * C6 * lw * Sa23 * S5 +
                   Ca12 * C5 * d3 * Sa23 * S4 * S6 + Ca12 * C5 * d4 * Sa23 * S4 * S6 -
                   C2 * Ca23 * C4 * C6 * d3 * Sa12 - C2 * Ca23 * C4 * C6 * d4 * Sa12 -
                   C2 * Ca23 * C4 * C6 * lw * Sa12 * S5 - C2 * Ca23 * C5 * d3 * Sa12 * S4 * S6 -
                   C2 * Ca23 * C5 * d4 * Sa12 * S4 * S6 + C2 * C5 * C6 * lw * Sa12 * Sa23 +
                   C4 * C5 * d3 * Sa12 * S2 * S6 - C4 * C5 * C6 * d4 * Sa12 * S2 * S6 -
                   C6 * d3 * Sa12 * S2 * S4 - C6 * d4 * Sa12 * S2 * S4 -
                   C6 * lw * Sa12 * S2 * S4 * S5;

  j_matrix(1, 1) = Ca23 * C5 * C6 * lw + C4 * C6 * d3 * Sa23 + C4 * C6 * d4 * Sa23 +
                   C4 * C6 * lw * Sa23 * S5 + C5 * d3 * Sa23 * S4 * S6 + C5 * d4 * Sa23 * S4 * S6;

  j_matrix(1, 2) = -S5 * S6;

  j_matrix(1, 3) = -C5 * C6 * lw;

  j_matrix(1, 4) = 1;

  j_matrix(1, 5) = 0;  // done
  //************************************************ row 3
  //********************************************
  j_matrix(2, 0) = L - Ca12 * d3 * Sa23 * S4 * S5 - Ca12 * d4 * Sa23 * S4 * S5 -
                   Ca12 * lw * Sa23 * S4 + C2 * Ca23 * d3 * Sa12 * S4 * S5 +
                   C2 * Ca23 * d4 * Sa12 * S4 * S5 + C2 * Ca23 * lw * Sa12 * S4 -
                   C4 * d3 * Sa12 * S2 * S5 - C4 * d4 * Sa12 * S2 * S5 - C4 * lw * Sa12 * S2;

  j_matrix(2, 1) = -d3 * Sa23 * S4 * S5 - d4 * Sa23 * S4 * S5 - lw * Sa23 * S4;

  j_matrix(2, 2) = -C5;

  j_matrix(2, 3) = 0;

  j_matrix(2, 4) = -lw;

  j_matrix(2, 5) = 0;

  //************************************************ row 4
  //********************************************
  j_matrix(3, 0) = L - Ca12 * Ca23 * C6 * S5 + Ca12 * C4 * C5 * C6 * Sa23 + Ca12 * Sa23 * S4 * S6 -
                   C2 * Ca23 * C4 * C5 * C6 * Sa12 - C2 * Ca23 * Sa12 * S4 * S6 -
                   C2 * C6 * Sa12 * Sa23 * S5 + C4 * Sa12 * S2 * S6 - C5 * C6 * Sa12 * S2 * S4;

  j_matrix(3, 1) = -Ca23 * C6 * S5 + C4 * C5 * C6 * Sa23 + Sa23 * S4 * S6;

  j_matrix(3, 2) = 0;

  j_matrix(3, 3) = C6 * S5;

  j_matrix(3, 4) = S6;

  j_matrix(3, 5) = 0;

  //************************************************ row 5
  //********************************************
  j_matrix(4, 0) = L * Ca12 * Ca23 * S5 * S6 - Ca12 * C4 * C5 * Sa23 * S6 + Ca12 * C6 * Sa23 * S4 +
                   C2 * Ca23 * C4 * C5 * Sa12 * S6 - C2 * Ca23 * C6 * Sa12 * S4 +
                   C2 * Sa12 * Sa23 * S5 * S6 + C4 * C6 * Sa12 * S2 + C5 * Sa12 * S2 * S4 * S6;
  j_matrix(4, 1) = Ca23 * S5 * S6 - C4 * C5 * Sa23 * S6 + C6 * Sa23 * S4;
  j_matrix(4, 2) = 0;
  j_matrix(4, 3) = -S5 * S6;
  j_matrix(4, 4) = C6;
  j_matrix(4, 5) = 0;
  //************************************************ row 6
  //********************************************
  j_matrix(5, 0) = L * Ca12 * Ca23 * C5 + Ca12 * C4 * Sa23 * S5 - C2 * Ca23 * C4 * Sa12 * S5 +
                   C2 * C5 * Sa12 * Sa23 - Sa12 * S2 * S4 * S5;
  j_matrix(5, 1) = Ca23 * C5 + C4 * Sa23 * S5;
  j_matrix(5, 2) = 0;
  j_matrix(5, 3) = -C5;
  j_matrix(5, 4) = 0;
  j_matrix(5, 5) = 1;

  success = 1;

  return success;
}
