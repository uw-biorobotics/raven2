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

/*********
 *
 * File: crtk_io.h
 *
 *  
 */

#ifndef __CRTK_PUBS_AND_SUBS_H__
#define __CRTK_PUBS_AND_SUBS_H__

#include <tf/transform_datatypes.h>
#include <crtk_msgs/StringStamped.h>
#include <crtk_msgs/OperatingState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>


// Global publisher and subscribers for raven data
ros::Publisher pub_ravenstate;
ros::Subscriber sub_automove;
ros::Subscriber sub_crtkCommand;
ros::Subscriber sub_crtkCommand_arm1;
ros::Subscriber sub_crtkCommand_arm2;
ros::Subscriber sub_crtkCommand_arm3;
ros::Subscriber sub_crtkCommand_arm4;
ros::Publisher joint_publisher;

//CRTK publishers and subscribers
ros::Publisher pub_crtk_state;
ros::Publisher pub_crtk_state_arm1;
ros::Publisher pub_crtk_state_arm2;
ros::Publisher pub_crtk_state_arm3;
ros::Publisher pub_crtk_state_arm4;

ros::Publisher pub_crtk_measured_js_gold;
ros::Publisher pub_crtk_measured_js_green;
ros::Publisher pub_crtk_measured_cp_gold;
ros::Publisher pub_crtk_measured_cp_green;
ros::Publisher pub_crtk_measured_cv_gold;
ros::Publisher pub_crtk_measured_cv_green;

ros::Publisher pub_crtk_setpoint_js_gold;
ros::Publisher pub_crtk_setpoint_js_green;
ros::Publisher pub_crtk_setpoint_cp_gold;
ros::Publisher pub_crtk_setpoint_cp_green;
ros::Publisher pub_crtk_setpoint_cv_gold;
ros::Publisher pub_crtk_setpoint_cv_green;

ros::Publisher pub_crtk_measured_js_blue;
ros::Publisher pub_crtk_measured_js_orange;
ros::Publisher pub_crtk_measured_cp_blue;
ros::Publisher pub_crtk_measured_cp_orange;
ros::Publisher pub_crtk_measured_cv_blue;
ros::Publisher pub_crtk_measured_cv_orange;

ros::Publisher pub_crtk_setpoint_js_blue;
ros::Publisher pub_crtk_setpoint_js_orange;
ros::Publisher pub_crtk_setpoint_cp_blue;
ros::Publisher pub_crtk_setpoint_cp_orange;
ros::Publisher pub_crtk_setpoint_cv_blue;
ros::Publisher pub_crtk_setpoint_cv_orange;

ros::Publisher pub_crtk_measured_js_gold_grasper;
ros::Publisher pub_crtk_measured_js_green_grasper;
ros::Publisher pub_crtk_setpoint_js_gold_grasper;
ros::Publisher pub_crtk_setpoint_js_green_grasper;

ros::Publisher pub_crtk_measured_js_blue_grasper;
ros::Publisher pub_crtk_measured_js_orange_grasper;
ros::Publisher pub_crtk_setpoint_js_blue_grasper;
ros::Publisher pub_crtk_setpoint_js_orange_grasper;

ros::Subscriber sub_servo_jp_gold;
ros::Subscriber sub_servo_jf_gold;
ros::Subscriber sub_servo_jr_gold;
ros::Subscriber sub_servo_jv_gold;
ros::Subscriber sub_servo_cp_gold;
ros::Subscriber sub_servo_cv_gold;
// ros::Subscriber sub_servo_cf_gold;
ros::Subscriber sub_servo_cr_gold;

ros::Subscriber sub_servo_jp_green;
ros::Subscriber sub_servo_jf_green;
ros::Subscriber sub_servo_jr_green;
ros::Subscriber sub_servo_jv_green;
ros::Subscriber sub_servo_cp_green;
ros::Subscriber sub_servo_cv_green;
// ros::Subscriber sub_servo_cf_green;
ros::Subscriber sub_servo_cr_green;

ros::Subscriber sub_servo_jp_green_grasper;
ros::Subscriber sub_servo_jf_green_grasper;
ros::Subscriber sub_servo_jr_green_grasper;
ros::Subscriber sub_servo_jv_green_grasper;
ros::Subscriber sub_servo_jp_gold_grasper;
ros::Subscriber sub_servo_jf_gold_grasper;
ros::Subscriber sub_servo_jr_gold_grasper;
ros::Subscriber sub_servo_jv_gold_grasper;

ros::Subscriber sub_servo_jp_blue;
ros::Subscriber sub_servo_jf_blue;
ros::Subscriber sub_servo_jr_blue;
ros::Subscriber sub_servo_jv_blue;
ros::Subscriber sub_servo_cp_blue;
ros::Subscriber sub_servo_cv_blue;
// ros::Subscriber sub_servo_cf_blue;
ros::Subscriber sub_servo_cr_blue;

ros::Subscriber sub_servo_jp_orange;
ros::Subscriber sub_servo_jf_orange;
ros::Subscriber sub_servo_jr_orange;
ros::Subscriber sub_servo_jv_orange;
ros::Subscriber sub_servo_cp_orange;
ros::Subscriber sub_servo_cv_orange;
// ros::Subscriber sub_servo_cf_orange;
ros::Subscriber sub_servo_cr_orange;

ros::Subscriber sub_servo_jp_orange_grasper;
ros::Subscriber sub_servo_jf_orange_grasper;
ros::Subscriber sub_servo_jr_orange_grasper;
ros::Subscriber sub_servo_jv_orange_grasper;
ros::Subscriber sub_servo_jp_blue_grasper;
ros::Subscriber sub_servo_jf_blue_grasper;
ros::Subscriber sub_servo_jr_blue_grasper;
ros::Subscriber sub_servo_jv_blue_grasper;

#endif