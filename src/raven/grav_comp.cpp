/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
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
 *  FILE: GravComp.c
 *
 *  Re-written March 2013 by Andy Lewis and Hawkeye King	
 *    Equations re-derived for UW Kinematics formulations for Raven II
 *    See forthcoming technical report for details.
 *
 *
 *
 */

#include "grav_comp.h"
#include "r2_kinematics.h"
#include "log.h"

extern int NUM_MECH;
extern struct DOF_type DOF_types[];
extern unsigned long int gTime;

// Define COM's (units: meters)
// Left arm values ???
const static btVector3 COM1_1_GL( 0.0065,   0.09775, -0.13771   );
const static btVector3 COM2_2_GL( -0.002, -0.18274, -0.23053   );
const static btVector3 COM3_3_GL( 0,0,0);

//// Right arm values::
const static btVector3 COM1_1_GR( -0.0065,   -0.09775, 0.13771   );
const static btVector3 COM2_2_GR( -0.002, -0.18274, 0.23053   );
const static btVector3 COM3_3_GR( 0,0,0);

// Define masses
//const static double M1 = 0.2395 * 2.7; // 0.6465 kg --> 1.42 lb		From Ji's code
//const static double M2 = 0.3568 * 2.7; // 0.96366 kg --> 2.12 lb
//const static double M3 = 0.1500 * 2.7; // 0.405 kg --> 0.89 lb

const static double M1 = 0.572; // kg --> 1.26 lb
const static double M2 = 0.672; // kg --> 1.48 lb
const static double M3 = 0.238; // kg --> 0.52 lb
// total is less than 4, but not 40...

// These are empirically defined corrections to gravity compensation torques.
// They should all be one, but they're not.
//const static double Kg1 = 10.0;
//const static double Kg2 = 10.0;
//const static double Kg3 = 10.0;

const static double Kg1 = 1;
const static double Kg2 = 1;
const static double Kg3 = 1;

void getMotorTorqueFromJointTorque(int, double, double, double, double&, double&, double&);

/*
 * getCurrentG()
 *    Return the current gravity vector from whatever power knows of it.
 */
btVector3 getCurrentG()
{
	return btVector3(-9.8,0,0);
}

/*
 * getGravityTorque()
 *    Calculate and set the gravity torque for each of the first three joints on both arms
 *
 *    Notation:
 *    Txy    - Transform from frame 0 to frame 1
 *    COMx_y - Center of mass of link x in link-frame y
 *    Gx     - Gravity vector represented in link-frame x
 *    GTx    - 3-vector of gravitational torque at joint x (z-component represents torque around joint)
 *    Mx     - mass of link x
 */
void getGravityTorque(struct device &d0)
{
	struct mechanism *_mech;
	btVector3 G0 = getCurrentG();
	btVector3 COM1_1, COM2_2, COM3_3;

	/// Do FK for each mechanism
	for (int m=0; m<NUM_MECH; m++)
	{
		_mech = &(d0.mech[m]);

		if (_mech->type == GOLD_ARM_SERIAL)
		{
			COM1_1 =COM1_1_GL;
			COM2_2 =COM2_2_GL;
			COM3_3 =COM3_3_GL;
		}
		else
		{
			COM1_1 =COM1_1_GR;
			COM2_2 =COM2_2_GR;
			COM3_3 =COM3_3_GR;
		}

		///// Get the transforms: ^0_1T, ^1_2T, ^2_3T
		btTransform T01, T12, T23;
		btMatrix3x3 R01, R12, R23;

		getATransform (*_mech, T01, 0, 1);
		getATransform (*_mech, T12, 1, 2);
		getATransform (*_mech, T23, 2, 3);

		R01 = T01.getBasis();
		R12 = T12.getBasis();
		R23 = T23.getBasis();

		///// Calculate COM in lower ink frames (closer to base)
		// Get COM3
//		btVector3 COM3_2 = T23 * COM3_3;
//		btVector3 COM3_1 = T12 * COM3_2;
//		btVector3 COM3_0 = T01 * COM3_1;

		btVector3 COM3_2 = R23 * COM3_3;
		btVector3 COM3_1 = R12 * COM3_2;
		btVector3 COM3_0 = R01 * COM3_1;

		// Get COM2
//		btVector3 COM2_1 = T12 * COM2_2;
//		btVector3 COM2_0 = T01 * COM2_1;

		btVector3 COM2_1 = R12 * COM2_2;
		btVector3 COM2_0 = R01 * COM2_1;

		// Get COM1
//		btVector3 COM1_0 = T01 * COM1_1;

		btVector3 COM1_0 = R01 * COM1_1;

		///// Get gravity vector in each link frame
		// Map G into Frame1
		btVector3 G1 = R01.inverse() * G0;
		// Map G into Frame2
		btVector3 G2 = R12.inverse() * G1;
		// Map G into Frame3
		btVector3 G3 = R23.inverse() * G2;

		///// Calculate Torque: T_i = sum( j=i..3 , (M_j * G_i) x ^iCOM_j )
		// T1 = (M1*G1) x ^1COM_1 + (M2*G1) x ^1COM_2 + (M3*G1) x ^1COM_3
		// T2 = (M2*G2) x ^2COM_2 + (M3*G2) x ^2COM_3

		btVector3 GT1  = COM1_1.cross(M1*G1) + COM2_1.cross(M2*G1) + COM3_1.cross(M3*G1);

		btVector3 GT11 = COM1_1.cross(M1*G1);
		btVector3 GT12 = COM2_1.cross(M2*G1);
		btVector3 GT13 = COM3_1.cross(M3*G1);

		btVector3 GT2  = COM2_2.cross(M2*G2) + COM3_2.cross(M3*G2);
		btVector3 GT22 = COM2_2.cross(M2*G2);
		btVector3 GT23 = COM3_2.cross(M3*G2);

		// Set joint g-torque from -Z-axis projection:
		double GZ1 = btVector3(0,0,-1).dot(GT1);
		double GZ2 = btVector3(0,0,-1).dot(GT2);
		double GZ3 = -1 * M3 * G3[2];

		// Get motor torque from joint torque
		double MT1, MT2, MT3;
		getMotorTorqueFromJointTorque(_mech->type, GZ1, GZ2, GZ3, MT1, MT2, MT3);

		// Set motor g-torque
		_mech->joint[SHOULDER].tau_g = MT1 * Kg1;
		_mech->joint[ELBOW   ].tau_g = MT2 * Kg2;
		_mech->joint[Z_INS   ].tau_g = MT3 * Kg3;
	}

	return;
}

// TODO: this function will need to be updated when cable coupling between first three axes is implemented as non-diagonal.
void getMotorTorqueFromJointTorque(int arm, double in_GZ1, double in_GZ2, double in_GZ3, double &out_MT1, double &out_MT2, double &out_MT3)
{

	// Get transmission ratios
	double tr1, tr2, tr3;
	if (arm == GOLD_ARM)
	{
		tr1 = DOF_types[SHOULDER_GOLD ].TR;
		tr2 = DOF_types[ELBOW_GOLD    ].TR;
		tr3 = DOF_types[Z_INS_GOLD    ].TR;

	}
	else
	{
		tr1 = DOF_types[SHOULDER_GREEN ].TR;
		tr2 = DOF_types[ELBOW_GREEN    ].TR;
		tr3 = DOF_types[Z_INS_GREEN    ].TR;
	}

	// claculate motor torques from joint torques
	// TODO:: add in additional cable-coupling terms
	out_MT1 = (1/tr1) * in_GZ1;
	out_MT2 = (1/tr2) * in_GZ2;
	out_MT3 = (1/tr3) * in_GZ3;

	return;
}
