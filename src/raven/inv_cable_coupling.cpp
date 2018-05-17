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
*  \file inv_cable_coupling.cpp
*
*  \brief Calculate the inverse cable coupling from a Joint Space Pose,
* 	(th1, th2, d3) express the desired motor pose (m1, m2, m3)
*
*  \fn:These are the 2 functions in inv_cable_coupling.cpp file.
*      Functions marked with "*" are called explicitly from other files.
* 	   *(1) invCableCoupling	 	:uses (2)
*      	    (2) invMechCableCoupling
*
*  \ingroup Control
*           Tool
*
*  \todo standardize numbering system 0 or 1 origin for motor designation
*  \todo standardize 3 or 4 insertion axis
*  \todo analyze (DANYING) coupling and convert to matrix maths
*
*/

#include "inv_cable_coupling.h"
#include "log.h"

extern DOF_type DOF_types[];
extern int NUM_MECH;
extern unsigned long int gTime;
/**
* invCableCoupling - wrapper function that checks for correct runlevel
* and calls invMechCableCoupling for each mechanism in device

* Function should be called in all runlevels to ensure that mpos_d = jpos_d.
*
* \param device0 pointer to device struct
* \param runlevel current runlevel
*
*/
void invCableCoupling(device *device0, int runlevel) {
  int i;

  // Run inverse cable coupling for each mechanism
  for (i = 0; i < NUM_MECH; i++) invMechCableCoupling(&(device0->mech[i]));
}

/**
* \brief Calculates desired motor positions from desired joint positions
*
* \todo update to matrix calculations for the coupling (here and fwd)
*/

void invMechCableCoupling(mechanism *mech, int no_use_actual) {
  if (mech->type != GOLD_ARM && mech->type != GREEN_ARM) {
    log_msg("bad mech type!");
    return;
  }

  float th1, th2, th3, th5, th6, th7;
  float d4;
  float m1, m2, m3, m4, m5, m6, m7;
  float tr1 = 0, tr2 = 0, tr3 = 0, tr4 = 0, tr5 = 0, tr6 = 0, tr7 = 0;

  th1 = mech->joint[SHOULDER].jpos_d;
  th2 = mech->joint[ELBOW].jpos_d;
  th3 = mech->joint[TOOL_ROT].jpos_d;
  d4 = mech->joint[Z_INS].jpos_d;
  th5 = mech->joint[WRIST].jpos_d;
  th6 = mech->joint[GRASP1].jpos_d;
  th7 = mech->joint[GRASP2].jpos_d;

  if (mech->type == GOLD_ARM) {
    tr1 = DOF_types[SHOULDER_GOLD].TR;
    tr2 = DOF_types[ELBOW_GOLD].TR;
    tr3 = DOF_types[TOOL_ROT_GOLD].TR;
    tr4 = DOF_types[Z_INS_GOLD].TR;
    tr5 = DOF_types[WRIST_GOLD].TR;
    tr6 = DOF_types[GRASP1_GOLD].TR;
    tr7 = DOF_types[GRASP2_GOLD].TR;

  } else if (mech->type == GREEN_ARM) {
    tr1 = DOF_types[SHOULDER_GREEN].TR;
    tr2 = DOF_types[ELBOW_GREEN].TR;
    tr3 = DOF_types[TOOL_ROT_GREEN].TR;
    tr4 = DOF_types[Z_INS_GREEN].TR;
    tr5 = DOF_types[WRIST_GREEN].TR;
    tr6 = DOF_types[GRASP1_GREEN].TR;
    tr7 = DOF_types[GRASP2_GREEN].TR;
  }

  // --------------Coupling and Transmission
  // Matrix---------------------------------------------------
  // trs are transmission ration for each joint defined in define.h
  // GB_RATIO = (GEAR_BOX_GP42_TR/GEAR_BOX_GP32_TR * (
  // CAPSTAN_RADIUS_GP32/CAPSTAN_RADIUS_GP42))
  // js and d are joint angles/placement; ms are motor angles
  // Numeration: 1-shoulder; 2-eblow; 3-insertion; 4-tool rotation; 5-wrist;
  // 6-upper grasper; 7- lower grasper
  // -------------------------------------------------------------------------------------------------
  // Tool Type: Raven Diamond Tool
  // Gold Arm - Left Arm:
  // M_l = C_l * J_l ----[m1,m2,m3,m4,m5,m6,m7]'= C_l*[j1,j2,d3,j4,j5,j6,j7]
  // C_l = [tr1 				 0 				 0 	       0   0
  // 0
  // 0;
  //        tr2*CABLE_COUPLING_01          tr2 				 0 	       0
  //        0   0  0;
  //        tr3*CABLE_COUPLING_02          tr3*CABLE_COUPLING_12           tr3
  //        0   0   0  0;
  //        tr3*CABLE_COUPLING_02/GB_RATIO tr3*CABLE_COUPLING_12G/B_RATIO
  //        tr3/GB_RATIO  tr4 0   0  0;
  //        tr3*CABLE_COUPLING_02/GB_RATIO tr3*CABLE_COUPLING_12G/B_RATIO
  //        tr3/GB_RATIO  0   tr5 0  0;
  //        tr3*CABLE_COUPLING_02/GB_RATIO tr3*CABLE_COUPLING_12G/B_RATIO
  //        tr3/GB_RATIO  0   0  tr6 0;
  //        tr3*CABLE_COUPLING_02/GB_RATIO tr3*CABLE_COUPLING_12G/B_RATIO
  //        tr3/GB_RATIO  0   0  0  tr7;]
  //
  // Green Arm - Right Arm:
  // M_r = C_r * J_r ----[m1,m2,m3,m4,m5,m6,m7]'= C_l*[j1,j2,d3,j4,j5,j6,j7]
  // C_r = [tr1 				   0 				   0 	          0   0
  // 0
  // 0;
  //        tr2*CABLE_COUPLING_01            tr2 				   0
  //        0   0   0  0;
  //        tr3*CABLE_COUPLING_02            tr3*CABLE_COUPLING_12           tr3
  //        0   0   0  0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  tr4 0   0  0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   tr5 0  0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   0  tr6 0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   0  0  tr7;]
  // -------------------------------------------------------------------------------------------------
  // Tool Type: Raven Square Tool
  // C_l=C_r
  //      = [tr1 				   0 				   0
  //      0   0   0  0;
  //        tr2*CABLE_COUPLING_01            tr2 				   0
  //        0   0   0  0;
  //        tr3*CABLE_COUPLING_02            tr3*CABLE_COUPLING_12           tr3
  //        0   0   0  0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  tr4 0   0  0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   tr5 0  0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   0  tr6 0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   0  0  tr7;]
  // -------------------------------------------------------------------------------------------------
  // Tool Type: DaVinci Square Tool
  // C_l=C_r
  //      = [tr1 				   0 				   0
  //      0   0       0   0;
  //        tr2*CABLE_COUPLING_01            tr2 				   0
  //        0   0       0   0;
  //        tr3*CABLE_COUPLING_02            tr3*CABLE_COUPLING_12           tr3
  //        0   0       0   0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  tr4 0       0   0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   tr      0   0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   tr6/2   tr6 0;
  //        -tr3*CABLE_COUPLING_02/GB_RATIO -tr3*CABLE_COUPLING_12G/B_RATIO
  //        -tr3/GB_RATIO  0   -tr7/2  0   tr7;]

  m1 = tr1 * th1;
  m2 = tr2 * (th2 + CABLE_COUPLING_01 * th1);
  m4 = tr4 * (d4 + CABLE_COUPLING_02 * th1 + CABLE_COUPLING_12 * th2);

  // Use the current joint position for cable coupling
  float m4_actual = mech->joint[Z_INS].mpos;

  if (no_use_actual) m4_actual = m4;

  int sgn = 0;
  switch (mech->mech_tool.t_style) {
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

  int sgn_6 = sgn;
#ifdef OPPOSE_GRIP
  sgn_6 *= -1;

#endif

  float tool_coupling = mech->mech_tool.wrist_coupling;
  m3 = (tr3 * th3) + sgn * m4_actual / GB_RATIO;
  m5 = (tr5 * th5) + sgn * m4_actual / GB_RATIO;
  m6 = (tr6 * (th6 + th5 * tool_coupling)) + sgn_6 * m4_actual / GB_RATIO;  // was th6 +th5*...
  m7 = (tr7 * (th7 - th5 * tool_coupling)) + sgn * m4_actual / GB_RATIO;

  /*Now have solved for desired motor positions mpos_d*/
  mech->joint[SHOULDER].mpos_d = m1;
  mech->joint[ELBOW].mpos_d = m2;
  mech->joint[TOOL_ROT].mpos_d = m3;
  mech->joint[Z_INS].mpos_d = m4;
  mech->joint[WRIST].mpos_d = m5;
  mech->joint[GRASP1].mpos_d = m6;
  mech->joint[GRASP2].mpos_d = m7;

  return;
}
