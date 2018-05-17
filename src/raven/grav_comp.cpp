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

/** \file  grav_comp.cpp
 *
 *  \brief Functions for calculating gravity torques
 *
 *  \desc  These functions will calculate gravity loads on each of the first 3
 *DOFs with predefined mass properties.
 *         However, arbitrary gravity vectors can be specified in the currParams
 *data structure, which are then
 *         used in these calculations.
 *
 *  \fn    These are the 3 functions in grav_comp.cpp file.
 *         Functions marked with "*" are called explicitly from other files.
 *             (1) getCurrentG
 * 	      *(2) getGravityTorque			:uses (1)(3)
 *             (3) getMotorTorqueFromJointTorque
 *
 *  \log   Re-written March 2013 by Andy Lewis and Hawkeye King
 *         Equations re-derived for UW Kinematics formulations for Raven II
 *         See Andy's MS thesis or ICRA '14 paper for technical details.
 *
 *  \author Hawkeye King
 *          Andrew Lewis
 *
 *  \date February 2013
 *
 *  \ingroup control
 */

#include "grav_comp.h"
#include "r2_kinematics.h"
#include "log.h"

extern int NUM_MECH;
extern DOF_type DOF_types[];
extern unsigned long int gTime;

// Define COM's (units: meters)
// Left arm values
const static tf::Vector3 COM1_1_GL(0.0065, 0.09775, -0.13771);   // meters
const static tf::Vector3 COM2_2_GL(-0.002, -0.18274, -0.23053);  // meters
const static tf::Vector3 COM3_3_GL(0, 0, 0);                     // meters

//// Right arm values::
const static tf::Vector3 COM1_1_GR(-0.0065, -0.09775, 0.13771);  // meters
const static tf::Vector3 COM2_2_GR(-0.002, -0.18274, 0.23053);   // meters
const static tf::Vector3 COM3_3_GR(0, 0, 0);                     // meters

// Define masses
// const static double M1 = 0.2395 * 2.7; // 0.6465 kg --> 1.42 lb		From
// Ji's code
// const static double M2 = 0.3568 * 2.7; // 0.96366 kg --> 2.12 lb
// const static double M3 = 0.1500 * 2.7; // 0.405 kg --> 0.89 lb

const static double M1 = 0.494;  // kg --> ? lb
const static double M2 = 0.750;  // kg --> ? lb
const static double M3 = 0.231;  // kg --> ? lb
// masses updated using fresh links from raven 2.1 build 6/13

tf::Vector3 getCurrentG(device *d0, int m);
void getMotorTorqueFromJointTorque(int, double, double, double, double &, double &, double &);

/**
 * getCurrentG()
 * \brief Return the current gravity vector from whatever power knows it.
 *
 * \param device   the robot device that needs gravity compensation
 * \param m		   the current mechanism (arm) being compensated
 *
 * \return gravity vector in m/s^2
 */
tf::Vector3 getCurrentG(device *d0, int m) {
  mechanism *_mech;
  _mech = &(d0->mech[m]);
  float xG0, yG0, zG0;
  if (_mech->type == GOLD_ARM_SERIAL) {
    // take new data and rotate to frame 0 GOLD and scale to m/s^2
    xG0 = -1 * ((float)d0->grav_dir.z) / 100;
    yG0 = ((float)d0->grav_dir.x) / 100;
    zG0 = -1 * ((float)d0->grav_dir.y) / 100;
  } else {
    // take new data and rotate to frame 0 GREEN and scale to m/s^2
    xG0 = -1 * ((float)d0->grav_dir.z) / 100;
    yG0 = -1 * ((float)d0->grav_dir.x) / 100;
    zG0 = ((float)d0->grav_dir.y) / 100;
  }

  // return as a bt vector
  return tf::Vector3(xG0, yG0, zG0);
}

/**
 * getGravityTorque()
 * \brief Calculate and set the gravity torque for each of the first three
 joints on both arms
 *
 * Using the current gravity vector in the input parameter struct and predefined
 COM information,
 * this function calculates the desired gravity compensation torque and sets the
 corresponding value in the
 * device struct.
 *
 * \param &d0		the robot device
 * \param &params	the current robot parameters struct
 *
 * \return void
 *
 * 	 Calculate Torque: T_i = sum( j=i..3 , (M_j * G_i) x ^iCOM_j )
                GT1 = (M1*G1) x ^1COM_1 + (M2*G1) x ^1COM_2 + (M3*G1) x ^1COM_3
                GT2 = (M2*G2) x ^2COM_2 + (M3*G2) x ^2COM_3
                GT3 = (M3 * G3)
 *
 *
 *    Notation:
 *    Txy    - Transform from frame x to frame y
 *    COMx_y - Center of mass of link x in link-frame y
 *    Gx     - Gravity vector represented in link-frame x
 *    GTx    - 3-vector of gravitational torque at joint x (z-component
 represents torque around joint)
 *    Mx     - mass of link x
 */
void getGravityTorque(device &d0, param_pass &params) {
  mechanism *_mech;
  tf::Vector3 G0;
  // static tf::Vector3 G0Static = tf::Vector3(-9.8, 0, 0); //unused
  tf::Vector3 COM1_1, COM2_2, COM3_3;

  /// Do FK for each mechanism
  for (int m = 0; m < NUM_MECH; m++) {
    _mech = &(d0.mech[m]);
    // G0 = G0Static;
    G0 = getCurrentG(&d0, m);  // uncomment this line to enable dynamic gravity vectors

    if (_mech->type == GOLD_ARM_SERIAL) {
      COM1_1 = COM1_1_GL;
      COM2_2 = COM2_2_GL;
      COM3_3 = COM3_3_GL;

    } else {
      COM1_1 = COM1_1_GR;
      COM2_2 = COM2_2_GR;
      COM3_3 = COM3_3_GR;
    }

    ///// Get the transforms: ^0_1T, ^1_2T, ^2_3T
    tf::Transform T01, T12, T23;
    tf::Matrix3x3 R01, R12, R23;
    tf::Matrix3x3 iR01, iR12, iR23;

    getATransform(*_mech, T01, 0, 1);
    getATransform(*_mech, T12, 1, 2);
    getATransform(*_mech, T23, 2, 3);

    R01 = T01.getBasis();
    R12 = T12.getBasis();
    R23 = T23.getBasis();

    ///// Calculate COM in lower link frames (closer to base)
    // Get COM3
    tf::Vector3 COM3_2 = T23 * COM3_3;
    tf::Vector3 COM3_1 = T12 * COM3_2;
    // tf::Vector3 COM3_0 = T01 * COM3_1; //unused?

    // Get COM2
    tf::Vector3 COM2_1 = T12 * COM2_2;
    // tf::Vector3 COM2_0 = T01 * COM2_1; //unused?

    // Get COM1
    // tf::Vector3 COM1_0 = T01 * COM1_1; //unused?

    ///// Get gravity vector in each link frame
    // Map G into Frame1
    iR01 = R01.inverse();
    tf::Vector3 G1 = iR01 * G0;

    // Map G into Frame2
    iR12 = R12.inverse();
    tf::Vector3 G2 = iR12 * G1;

    // Map G into Frame3
    iR23 = R23.inverse();
    tf::Vector3 G3 = iR23 * G2;

    ///// Calculate Torque: T_i = sum( j=i..3 , (M_j * G_i) x ^iCOM_j )
    // T1 = (M1*G1) x ^1COM_1 + (M2*G1) x ^1COM_2 + (M3*G1) x ^1COM_3
    // T2 = (M2*G2) x ^2COM_2 + (M3*G2) x ^2COM_3

    tf::Vector3 GT1 = COM1_1.cross(M1 * G1) + COM2_1.cross(M2 * G1) + COM3_1.cross(M3 * G1);

    tf::Vector3 GT2 = COM2_2.cross(M2 * G2) + COM3_2.cross(M3 * G2);

    tf::Vector3 GT3 = M3 * G3;

    // Set joint g-torque from -Z-axis projection:
    double GZ1 = tf::Vector3(0, 0, -1).dot(GT1);
    double GZ2 = tf::Vector3(0, 0, -1).dot(GT2);
    double GZ3 = tf::Vector3(0, 0, -1).dot(GT3);

    // Get motor torque from joint torque
    double MT1, MT2, MT3;
    getMotorTorqueFromJointTorque(_mech->type, GZ1, GZ2, GZ3, MT1, MT2, MT3);

    // Set motor g-torque
    _mech->joint[SHOULDER].tau_g = MT1;
    _mech->joint[ELBOW].tau_g = MT2;
    _mech->joint[Z_INS].tau_g = MT3;
  }

  return;
}

/**
 * \brief Calculates the motor torque required to output a specified joint
 *torque
 *
 * \param	arm 		the type of mechanism that the output is
 *calculate for
 * \param 	in_GZ1		the desired joint torque at DOF1
 * \param  	in_GZ2 		the desired joint torque at DOF2
 * \param  	in_GZ3 		the desired joint torque at DOF3
 * \param 	&out_MT1	an output pointer for calculated motor torque 1
 * \param 	&out_MT2	an output pointer for calculated motor torque 2
 * \param 	&out_MT3	an output pointer for calculated motor torque 3
 *
 * \return void
 */

// TODO: this function will need to be updated when cable coupling between first
// three axes is implemented as non-diagonal.
// TODO: this should be in a different file, maybe motors.cpp?
void getMotorTorqueFromJointTorque(int arm, double in_GZ1, double in_GZ2, double in_GZ3,
                                   double &out_MT1, double &out_MT2, double &out_MT3) {
  // claculate motor torques from joint torques
  // TODO:: add in additional cable-coupling terms
  out_MT1 = in_GZ1 / SHOULDER_TR_GOLD_ARM * GEAR_BOX_GP42_TR;
  out_MT2 = in_GZ2 / ELBOW_TR_GOLD_ARM * GEAR_BOX_GP42_TR;
  out_MT3 = in_GZ3 / Z_INS_TR_GOLD_ARM * GEAR_BOX_GP42_TR;

  return;
}
