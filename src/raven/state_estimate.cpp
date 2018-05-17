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

/***************************
 *
 * FILE: stateEstimate.c
 * Created May 2010 by H. Hawkeye King
 *
 *    I apply filters or whatever to get an estimate of the state
 * (position and velocity) of the system.
 *
 */

#include "state_estimate.h"
#include "log.h"

extern DOF_type DOF_types[];
extern int NUM_MECH;

void getStateLPF(DOF *joint);

/*
 * stateEstimate()
 */

void stateEstimate(robot_device *device0) {
  DOF *_joint;
  int i, j;

  // Loop through all joints
  for (i = 0; i < NUM_MECH; i++) {
    for (j = 0; j < MAX_DOF_PER_MECH; j++) {
      _joint = &(device0->mech[i].joint[j]);

      //            if (_joint->type == TOOL_ROT_GOLD || _joint->type ==
      //            TOOL_ROT_GOLD)
      //                encToMPos(_joint);
      //            else
      getStateLPF(_joint, device0->mech[i].tool_type);
    }
  }
}

/*
 * filterPosition()
 *
 *  Apply an LPF to the motor position to eliminate
 * high frequency content in the control loop.  The HF
 * will drive the cable transmission unstable.
 *
 */
void getStateLPF(DOF *joint, int tool_type) {
  // 50 HZ 3rd order butterworth
  //    float B[] = {0.0029,  0.0087,  0.0087,  0.0029};
  //    float A[] = {1.0000,  2.3741, -1.9294,  0.5321};
  // 75 Hz 3rd order butterworth
  //    float B[] = {0.00859,  0.0258,  0.0258,  0.00859};
  //    float A[] = {1.0000,   2.0651, -1.52,  0.3861};
  //  20 Hz 3rd order butterworth
  //    float B[] = {0.0002196,  0.0006588,  0.0006588,  0.0002196};
  //    float A[] = {1.0000,   2.7488, -2.5282,  0.7776};
  //  120 Hz 3rd order butterworth
  float B[] = {0.02864, 0.08591, 0.08591, 0.02864};
  float A[] = {1.0000, 1.5189, -0.9600, 0.2120};
  //    float B[] = {1.0, 0,0,0};
  //    float A[] = {0,0,0,0};

  float *oldPos = DOF_types[joint->type].old_mpos;
  float *oldFiltPos = DOF_types[joint->type].old_filtered_mpos;
  float filtPos = 0;
  float f_enc_val = joint->enc_val;

//#ifdef RAVEN_II
//    if ( (joint->type == SHOULDER_GOLD) ||
//         (joint->type == ELBOW_GOLD) ||
//         (joint->type == Z_INS_GOLD)
//         ||
//#ifndef RAVEN_II_SQUARE
//         (joint->type == TOOL_ROT_GOLD) ||
//         (joint->type == WRIST_GOLD) ||
//         (joint->type == GRASP1_GOLD) ||
//         (joint->type == GRASP2_GOLD)
//#endif
//         ||
//         (joint->type == TOOL_ROT_GREEN) ||
//         (joint->type == WRIST_GREEN) ||
//         (joint->type == GRASP1_GREEN) ||
//         (joint->type == GRASP2_GREEN)
//         )
//         f_enc_val *= -1;
//#endif

#ifdef RAVEN_II
  switch (tool_type) {
    case RII_square_type:
      if ((joint->type == SHOULDER_GOLD) || (joint->type == ELBOW_GOLD) ||
          (joint->type == Z_INS_GOLD) || (joint->type == TOOL_ROT_GREEN) ||
          (joint->type == WRIST_GREEN) || (joint->type == GRASP1_GREEN) ||
          (joint->type == GRASP2_GREEN))
        f_enc_val *= -1;
      break;

    case dv_adapter:
      if ((joint->type == SHOULDER_GOLD) || (joint->type == ELBOW_GOLD) ||
          (joint->type == Z_INS_GOLD))
        f_enc_val *= -1;
      break;

    default:
      if ((joint->type == SHOULDER_GOLD) || (joint->type == ELBOW_GOLD) ||
          (joint->type == Z_INS_GOLD) || (joint->type == TOOL_ROT_GOLD) ||
          (joint->type == WRIST_GOLD) || (joint->type == GRASP1_GOLD) ||
          (joint->type == GRASP2_GOLD) || (joint->type == TOOL_ROT_GREEN) ||
          (joint->type == WRIST_GREEN) || (joint->type == GRASP1_GREEN) ||
          (joint->type == GRASP2_GREEN))
        f_enc_val *= -1;
      break;
  }

#ifdef OPPOSE_GRIP
  if ((joint->type == GRASP1_GOLD) || (joint->type == GRASP1_GREEN)) f_enc_val *= -1;
#endif

//    static int i = 0;
//    static int j = 0;
//    j++;
//    if (j % 100){
//
//
//    i++;
//    if (i % (17) == 0){
//        log_msg("joint %i enc_val :  %f    f_enc :  %f", joint->type,
//        joint->enc_val, f_enc_val);
//        log_msg("tool type in state estimate -->  %i", tool_type);
//        //i = 0;
//    }
//    j=0;
//    }
#endif

  // Calculate motor angle from encoder value
  float motorPos =
      (2.0 * PI) * (1.0 / ((float)ENC_CNTS_PER_REV)) * (f_enc_val - (float)joint->enc_offset);

  // Initialize filter to steady state
  if (!DOF_types[joint->type].filterRdy) {
    oldPos[2] = motorPos;
    oldPos[1] = motorPos;
    oldPos[0] = motorPos;
    oldFiltPos[2] = motorPos;
    oldFiltPos[1] = motorPos;
    oldFiltPos[0] = motorPos;
    //    rt_printk("FILTER BEING READIED %d\n", gTime);
    DOF_types[joint->type].filterRdy = TRUE;
  }

  // Compute filtered motor angle
  filtPos = B[0] * motorPos + B[1] * oldPos[0] + B[2] * oldPos[1] + B[3] * oldPos[2] +
            A[1] * oldFiltPos[0] + A[2] * oldFiltPos[1] + A[3] * oldFiltPos[2];

// Compute velocity from first difference
// This is safe b/c noise is removed by LPF

// removed filter functionality - CIGIT 7/30/15
// the filter was shown to cause fluttering in the tool joints after homing
#ifdef NO_LPF
  joint->mvel = (motorPos - oldPos[0]) / STEP_PERIOD;
  joint->mpos = motorPos;

  static int print_once = 0;
  if (print_once < 1) {
    log_msg("!!!!!!!!!!!!!    LPF FILTER IS OFF    !!!!!!111!!1!1!!!", 0);
    print_once++;
  }
#else  // use the filter
  joint->mvel = (filtPos - oldFiltPos[0]) / STEP_PERIOD;
  joint->mpos = filtPos;
#endif

  // Update old values for filter
  oldPos[2] = oldPos[1];
  oldPos[1] = oldPos[0];
  oldPos[0] = motorPos;
  oldFiltPos[2] = oldFiltPos[1];
  oldFiltPos[1] = oldFiltPos[0];
  oldFiltPos[0] = filtPos;

  return;
}

void getStateLPF(DOF *joint, style t_style) {
  // 50 HZ 3rd order butterworth
  //    float B[] = {0.0029,  0.0087,  0.0087,  0.0029};
  //    float A[] = {1.0000,  2.3741, -1.9294,  0.5321};
  // 75 Hz 3rd order butterworth
  //    float B[] = {0.00859,  0.0258,  0.0258,  0.00859};
  //    float A[] = {1.0000,   2.0651, -1.52,  0.3861};
  //  20 Hz 3rd order butterworth
  //    float B[] = {0.0002196,  0.0006588,  0.0006588,  0.0002196};
  //    float A[] = {1.0000,   2.7488, -2.5282,  0.7776};
  //  120 Hz 3rd order butterworth
  float B[] = {0.02864, 0.08591, 0.08591, 0.02864};
  float A[] = {1.0000, 1.5189, -0.9600, 0.2120};
  //    float B[] = {1.0, 0,0,0};
  //    float A[] = {0,0,0,0};

  float *oldPos = DOF_types[joint->type].old_mpos;
  float *oldFiltPos = DOF_types[joint->type].old_filtered_mpos;
  float filtPos = 0;
  float f_enc_val = joint->enc_val;

//#ifdef RAVEN_II
//    if ( (joint->type == SHOULDER_GOLD) ||
//         (joint->type == ELBOW_GOLD) ||
//         (joint->type == Z_INS_GOLD)
//         ||
//#ifndef RAVEN_II_SQUARE
//         (joint->type == TOOL_ROT_GOLD) ||
//         (joint->type == WRIST_GOLD) ||
//         (joint->type == GRASP1_GOLD) ||
//         (joint->type == GRASP2_GOLD)
//#endif
//         ||
//         (joint->type == TOOL_ROT_GREEN) ||
//         (joint->type == WRIST_GREEN) ||
//         (joint->type == GRASP1_GREEN) ||
//         (joint->type == GRASP2_GREEN)
//         )
//         f_enc_val *= -1;
//#endif

#ifdef RAVEN_II
  switch (t_style) {
    case square_raven:
      if ((joint->type == SHOULDER_GOLD) || (joint->type == ELBOW_GOLD) ||
          (joint->type == Z_INS_GOLD) || (joint->type == TOOL_ROT_GREEN) ||
          (joint->type == WRIST_GREEN) || (joint->type == GRASP1_GREEN) ||
          (joint->type == GRASP2_GREEN))
        f_enc_val *= -1;
      break;

    case dv:
      if ((joint->type == SHOULDER_GOLD) || (joint->type == ELBOW_GOLD) ||
          (joint->type == Z_INS_GOLD))
        f_enc_val *= -1;
      break;

    default:
      if ((joint->type == SHOULDER_GOLD) || (joint->type == ELBOW_GOLD) ||
          (joint->type == Z_INS_GOLD) || (joint->type == TOOL_ROT_GOLD) ||
          (joint->type == WRIST_GOLD) || (joint->type == GRASP1_GOLD) ||
          (joint->type == GRASP2_GOLD) || (joint->type == TOOL_ROT_GREEN) ||
          (joint->type == WRIST_GREEN) || (joint->type == GRASP1_GREEN) ||
          (joint->type == GRASP2_GREEN))
        f_enc_val *= -1;
      break;
  }

#ifdef OPPOSE_GRIP
  if ((joint->type == GRASP1_GOLD) || (joint->type == GRASP1_GREEN)) f_enc_val *= -1;
#endif

  static int i = 0;
  static int j = 0;
  j++;
  if (j % 100) {
    i++;
    if (i % (17) == 0) {
      //        log_msg("joint %i enc_val :  %f    f_enc :  %f", joint->type,
      //        joint->enc_val, f_enc_val);
      log_msg("tool type in state estimate -->  %i", t_style);
      // i = 0;
    }
    j = 0;
  }
#endif

  // Calculate motor angle from encoder value
  float motorPos =
      (2.0 * PI) * (1.0 / ((float)ENC_CNTS_PER_REV)) * (f_enc_val - (float)joint->enc_offset);

  // Initialize filter to steady state
  if (!DOF_types[joint->type].filterRdy) {
    oldPos[2] = motorPos;
    oldPos[1] = motorPos;
    oldPos[0] = motorPos;
    oldFiltPos[2] = motorPos;
    oldFiltPos[1] = motorPos;
    oldFiltPos[0] = motorPos;
    //    rt_printk("FILTER BEING READIED %d\n", gTime);
    DOF_types[joint->type].filterRdy = TRUE;
  }

  // Compute filtered motor angle
  filtPos = B[0] * motorPos + B[1] * oldPos[0] + B[2] * oldPos[1] + B[3] * oldPos[2] +
            A[1] * oldFiltPos[0] + A[2] * oldFiltPos[1] + A[3] * oldFiltPos[2];

  // Compute velocity from first difference
  // This is safe b/c noise is removed by LPF
  joint->mvel = (filtPos - oldFiltPos[0]) / STEP_PERIOD;
  joint->mpos = filtPos;

  // Update old values for filter
  oldPos[2] = oldPos[1];
  oldPos[1] = oldPos[0];
  oldPos[0] = motorPos;
  oldFiltPos[2] = oldFiltPos[1];
  oldFiltPos[1] = oldFiltPos[0];
  oldFiltPos[0] = filtPos;

  return;
}

void resetFilter(DOF *_joint) {
  // reset filter
  for (int i = 0; i <= 2; i++) {
    DOF_types[_joint->type].old_mpos[i] = _joint->mpos_d;
    DOF_types[_joint->type].old_filtered_mpos[i] = _joint->mpos_d;
  }
}

/*
  if (joint->type == SHOULDER_B) {
    if (gTime % 100 == 0)
      printk("Filter state(%d):%d\t\t%d\t%d\t%d\t%d \t\t %d\t%d\t%d\n",
             jiffies,
             (int)(filtPos*100),
             (int)(motorPos*100),
             (int)(oldPos[0]*100) ,
             (int)(oldPos[1]*100) ,
             (int)(oldPos[2]*100) ,
             (int)(oldFiltPos[0]*100) ,
             (int)(oldFiltPos[1]*100) ,
             (int)(oldFiltPos[2]*100));
             }
*/
