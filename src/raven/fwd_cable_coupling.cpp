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

/**
*  \file fwd_cable_coupling.cpp
*  \brief Calculate the forward cable coupling from a motor space Pose,
*         (m1, m2,m3) express the desired joint pose (th1, th2, d3)
*  \author Hawkeye
*/

#include "fwd_cable_coupling.h"
#include "log.h"
#include "tool.h"

extern struct DOF_type DOF_types[];
extern int NUM_MECH;

/**
* \fn void fwdCableCoupling(struct device *device0, int runlevel)
* \brief Calls fwdMechCableCoupling for each mechanism in device
* \param device0 - pointer to device struct
* \param runlevel - current runlevel
* \return void
* \todo Remove runlevel from args.
*/

void fwdCableCoupling(struct device *device0, int runlevel)
{
	//Run fwd cable coupling for each mechanism.
	// This should be run in all runlevels.
	for (int i = 0; i < NUM_MECH; i++)
		fwdMechCableCoupling(&(device0->mech[i]));
}

/**
* \fn void fwdMechCableCoupling(struct mechanism *mech)
* \param mech
* \return void
*/

void fwdMechCableCoupling(struct mechanism *mech)
{
	float th1, th2, th3, th5, th6, th7;
	float th1_dot, th2_dot;
	float d4;
	float d4_dot;
	float m1, m2, m3, m4, m5, m6, m7;
	float m1_dot, m2_dot, m4_dot;
	float tr1=0, tr2=0, tr3=0, tr4=0, tr5=0, tr6=0, tr7=0;

	m1 = mech->joint[SHOULDER].mpos;
	m2 = mech->joint[ELBOW].mpos;
	m3 = mech->joint[TOOL_ROT].mpos;
	m4 = mech->joint[Z_INS].mpos;
	m5 = mech->joint[WRIST].mpos;
	m6 = mech->joint[GRASP1].mpos;
	m7 = mech->joint[GRASP2].mpos;

	m1_dot = mech->joint[SHOULDER].mvel;
	m2_dot = mech->joint[ELBOW].mvel;
	m4_dot = mech->joint[Z_INS].mvel;

	if (mech->type == GOLD_ARM){
		tr1=DOF_types[SHOULDER_GOLD].TR;
		tr2=DOF_types[ELBOW_GOLD].TR;
		tr3=DOF_types[TOOL_ROT_GOLD].TR;
		tr4=DOF_types[Z_INS_GOLD].TR;
		tr5=DOF_types[WRIST_GOLD].TR;
		tr6=DOF_types[GRASP1_GOLD].TR;
		tr7=DOF_types[GRASP2_GOLD].TR;

	} else if (mech->type == GREEN_ARM){
		tr1=DOF_types[SHOULDER_GREEN].TR;
		tr2=DOF_types[ELBOW_GREEN].TR;
		tr3=DOF_types[TOOL_ROT_GREEN].TR;
		tr4=DOF_types[Z_INS_GREEN].TR;
		tr5=DOF_types[WRIST_GREEN].TR;
		tr6=DOF_types[GRASP1_GREEN].TR;
		tr7=DOF_types[GRASP2_GREEN].TR;
	}
	else {
		log_msg("ERROR: incorrect device type in fwdMechCableCoupling");
		return;
	}

	// Forward Cable Coupling equations
	//   Originally based on 11/7/2005, Mitch notebook pg. 169
	//   Updated from UCSC code.  Code simplified by HK 8/11
	th1 = (1.0/tr1) * m1;

	// DELETEME
	//	th2 = (1.0/tr2) * m2; // additional CC terms added 3/13
	//	d4  = (1.0/tr4) * m4;

	th2 = (1.0/tr2) * m2 - CABLE_COUPLING_01 * th1;
	d4  = (1.0/tr4) * m4 - CABLE_COUPLING_02 * th1 - CABLE_COUPLING_12 * th2;


	// TODO:: Update with new cable coupling terms
	th1_dot = (1.0/tr1) * m1_dot;
	th2_dot = (1.0/tr2) * m2_dot;
	d4_dot  = (1.0/tr4) * m4_dot;


	// Tool degrees of freedom ===========================================
    int sgn = 0;
    switch(mech->mech_tool.t_style){
	  case raven:
		sgn = (mech->type == GOLD_ARM) ? 1 : -1;
		break;
	  case dv:
		sgn = (mech->type != GOLD_ARM) ? 1 : -1;
		break;
	  case square_raven:
		sgn = -1;
		break;
	  default:
		log_msg("undefined tool style!!! inv_cable_coupling.cpp");
		break;
    }

    float tool_coupling = mech->mech_tool.wrist_coupling;
    th3 = (1.0/tr3) * (m3 - sgn*m4/GB_RATIO);
    th5 = (1.0/tr5) * (m5 - sgn*m4/GB_RATIO);
    th6 = (1.0/tr6) * (m6 - sgn*m4/GB_RATIO) - th5*tool_coupling;
    th7 = (1.0/tr7) * (m7 - sgn*m4/GB_RATIO) + th5*tool_coupling;


	// Now have solved for th1, th2, d3, th4, th5, th6
	mech->joint[SHOULDER].jpos 		= th1;// - mech->joint[SHOULDER].jpos_off;
	mech->joint[ELBOW].jpos 		= th2;// - mech->joint[ELBOW].jpos_off;
	mech->joint[TOOL_ROT].jpos 		= th3;// - mech->joint[TOOL_ROT].jpos_off;
	mech->joint[Z_INS].jpos 		= d4;//  - mech->joint[Z_INS].jpos_off;
	mech->joint[WRIST].jpos 		= th5;// - mech->joint[WRIST].jpos_off;
	mech->joint[GRASP1].jpos 		= th6;// - mech->joint[GRASP1].jpos_off;
	mech->joint[GRASP2].jpos 		= th7;// - mech->joint[GRASP2].jpos_off;

	mech->joint[SHOULDER].jvel 		= th1_dot;// - mech->joint[SHOULDER].jpos_off;
	mech->joint[ELBOW].jvel 		= th2_dot;// - mech->joint[ELBOW].jpos_off;
	mech->joint[Z_INS].jvel 		= d4_dot;//  - mech->joint[Z_INS].jpos_off;

	return;
}




/**
* \fn void fwdTorqueCoupling(struct device *device0, int runlevel)
* \brief Calls fwdMechTorqueCoupling for each mechanism in device
* \param device0 - pointer to device struct
* \param runlevel - current runlevel
* \return void
* \todo Remove runlevel from args.
*/

void fwdTorqueCoupling(struct device *device0, int runlevel)
{
	//Run fwd cable coupling for each mechanism.
	// This should be run in all runlevels.
	for (int i = 0; i < NUM_MECH; i++)
		fwdMechTorqueCoupling(&(device0->mech[i]));
}

/**
* \fn void fwdMechTorqueCoupling(struct mechanism *mech)
* \param mech
* \return void
*/

void fwdMechTorqueCoupling(struct mechanism *mech)
{
	float th1=0, th2=0, th3=0, th5=0, th6=0, th7=0;
	float th1_dot, th2_dot;
	float d4;
	float d4_dot;
	float m1, m2, m3, m4, m5, m6, m7;
	float m1_dot, m2_dot, m4_dot;
	float tr1=0, tr2=0, tr3=0, tr4=0, tr5=0, tr6=0, tr7=0;

	m1 = mech->joint[SHOULDER].mpos;
	m2 = mech->joint[ELBOW].mpos;
	m3 = mech->joint[TOOL_ROT].mpos;
	m4 = mech->joint[Z_INS].mpos;
	m5 = mech->joint[WRIST].mpos;
	m6 = mech->joint[GRASP1].mpos;
	m7 = mech->joint[GRASP2].mpos;

	m1_dot = mech->joint[SHOULDER].mvel;
	m2_dot = mech->joint[ELBOW].mvel;
	m4_dot = mech->joint[Z_INS].mvel;

	if (mech->type == GOLD_ARM){
		tr1=DOF_types[SHOULDER_GOLD].TR;
		tr2=DOF_types[ELBOW_GOLD].TR;
		tr3=DOF_types[TOOL_ROT_GOLD].TR;
		tr4=DOF_types[Z_INS_GOLD].TR;
		tr5=DOF_types[WRIST_GOLD].TR;
		tr6=DOF_types[GRASP1_GOLD].TR;
		tr7=DOF_types[GRASP2_GOLD].TR;

	} else if (mech->type == GREEN_ARM){
		tr1=DOF_types[SHOULDER_GREEN].TR;
		tr2=DOF_types[ELBOW_GREEN].TR;
		tr3=DOF_types[TOOL_ROT_GREEN].TR;
		tr4=DOF_types[Z_INS_GREEN].TR;
		tr5=DOF_types[WRIST_GREEN].TR;
		tr6=DOF_types[GRASP1_GREEN].TR;
		tr7=DOF_types[GRASP2_GREEN].TR;
	}
	else {
		log_msg("ERROR: incorrect device type in fwdMechCableCoupling");
		return;
	}

	// Forward Cable Coupling equations
	//   Originally based on 11/7/2005, Mitch notebook pg. 169
	//   Updated from UCSC code.  Code simplified by HK 8/11
	th1 = (1.0/tr1) * m1;

	// DELETEME
	//	th2 = (1.0/tr2) * m2; // additional CC terms added 3/13
	//	d4  = (1.0/tr4) * m4;

	th2 = (1.0/tr2) * m2 - CABLE_COUPLING_01 * th1;
	d4  = (1.0/tr4) * m4 - CABLE_COUPLING_02 * th1 -CABLE_COUPLING_12 * th2;


	// TODO:: Update with new cable coupling terms
	th1_dot = (1.0/tr1) * m1_dot;
	th2_dot = (1.0/tr2) * m2_dot;
	d4_dot  = (1.0/tr4) * m4_dot;


	// Tool degrees of freedom ===========================================
	if (mech->tool_type == TOOL_GRASPER_10MM)
	{
		int sgn = (mech->type == GOLD_ARM) ? 1 : -1;
		th3 = (1.0/tr3) * (m3 - sgn*m4/GB_RATIO);
		th5 = (1.0/tr5) * (m5 - sgn*m4/GB_RATIO);
		th6 = (1.0/tr6) * (m6 - sgn*m4/GB_RATIO);
		th7 = (1.0/tr7) * (m7 - sgn*m4/GB_RATIO);
	}

	// Now have solved for th1, th2, d3, th4, th5, th6
	mech->joint[SHOULDER].jpos 	= th1;// - mech->joint[SHOULDER].jpos_off;
	mech->joint[ELBOW].jpos 		= th2;// - mech->joint[ELBOW].jpos_off;
	mech->joint[TOOL_ROT].jpos 	= th3;// - mech->joint[TOOL_ROT].jpos_off;
	mech->joint[Z_INS].jpos 		= d4;//  - mech->joint[Z_INS].jpos_off;
	mech->joint[WRIST].jpos 		= th5;// - mech->joint[WRIST].jpos_off;
	mech->joint[GRASP1].jpos 		= th6;// - mech->joint[GRASP1].jpos_off;
	mech->joint[GRASP2].jpos 		= th7;// - mech->joint[GRASP2].jpos_off;

	mech->joint[SHOULDER].jvel 	= th1_dot;// - mech->joint[SHOULDER].jpos_off;
	mech->joint[ELBOW].jvel 		= th2_dot;// - mech->joint[ELBOW].jpos_off;
	mech->joint[Z_INS].jvel 		= d4_dot;//  - mech->joint[Z_INS].jpos_off;

	return;
}


