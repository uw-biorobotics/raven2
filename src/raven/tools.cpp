/*
 * tools.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: junjie
 */

#include "tools.h"
#include "defines.h"

tool::tool(end_effector_type t_end_new, int a_mech) {
	set_tool(t_end_new, a_mech);
	set_tool_data();
}


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


void tool::set_tool_data() {
	set_limits();
	set_home_angles();
	set_wrist_coupling();
	set_DH_params();
	set_max_opening_angle();
}

void tool::set_limits() {
	switch (t_end) {
	case r_grasper:
		rot_max_angle = 330 DEG2RAD;
		rot_min_angle = -330 DEG2RAD; //might not be correct
		rot_max_limit = 182 DEG2RAD;
		rot_min_limit = -182 DEG2RAD;

		wrist_max_angle = 115 DEG2RAD;
		wrist_min_angle = -115 DEG2RAD;
		wrist_max_limit = 75 DEG2RAD;
		wrist_min_limit = -75 DEG2RAD;

		grasp1_max_angle = 120 DEG2RAD;
		grasp1_min_angle = -120 DEG2RAD;
		grasp1_max_limit = 85 DEG2RAD;
		grasp1_min_limit = -85 DEG2RAD;

		grasp2_max_angle = 120 DEG2RAD;
		grasp2_min_angle = -120 DEG2RAD;
		grasp2_max_limit = 85 DEG2RAD;
		grasp2_min_limit = -85 DEG2RAD;
		break;

	case r_sq_grasper:
		rot_max_angle = 330 DEG2RAD;
		rot_min_angle = -330 DEG2RAD; //might not be correct
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
		rot_max_angle = 275 DEG2RAD; //from 260
		rot_min_angle = -275 DEG2RAD; //might not be correct
		rot_max_limit = 182 DEG2RAD;
		rot_min_limit = -182 DEG2RAD;

		wrist_max_angle = 95 DEG2RAD;
		wrist_min_angle = -95 DEG2RAD;
		wrist_max_limit = 75 DEG2RAD;
		wrist_min_limit = -75 DEG2RAD;

		grasp1_max_angle = 120 DEG2RAD;
		grasp1_min_angle = -120 DEG2RAD;
		grasp1_max_limit = 85 DEG2RAD;
		grasp1_min_limit = -85 DEG2RAD;

		grasp2_max_angle = 120 DEG2RAD;
		grasp2_min_angle = -120 DEG2RAD;
		grasp2_max_limit = 85 DEG2RAD;
		grasp2_min_limit = -85 DEG2RAD;
		break;

	case micro_forceps:
		rot_max_angle = 275 DEG2RAD; //from 260
		rot_min_angle = -275 DEG2RAD; //might not be correct
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
		rot_max_angle = 260 DEG2RAD; //from 260
		rot_min_angle = -260 DEG2RAD; //might not be correct
		rot_max_limit = 182 DEG2RAD;
		rot_min_limit = -182 DEG2RAD;

		wrist_max_angle = 70 DEG2RAD;
		wrist_min_angle = -70 DEG2RAD;
		wrist_max_limit = 65 DEG2RAD;
		wrist_min_limit = -65 DEG2RAD;

		grasp1_max_angle = 120 DEG2RAD;
		grasp1_min_angle = -120 DEG2RAD;
		grasp1_max_limit = 85 DEG2RAD;
		grasp1_min_limit = -85 DEG2RAD;

		grasp2_max_angle = 120 DEG2RAD;
		grasp2_min_angle = -120 DEG2RAD;
		grasp2_max_limit = 85 DEG2RAD;
		grasp2_min_limit = -85 DEG2RAD;
		break;

	case cardiere_forceps:
		rot_max_angle = 260 DEG2RAD; //from 260
		rot_min_angle = -260 DEG2RAD; //might not be correct
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
		rot_min_angle = -260 DEG2RAD; //might not be correct
		rot_max_limit = 182 DEG2RAD;
		rot_min_limit = -182 DEG2RAD;

		wrist_max_angle = 90 DEG2RAD;
		wrist_min_angle = -90 DEG2RAD;
		wrist_max_limit = 75 DEG2RAD;
		wrist_min_limit = -75 DEG2RAD;

		grasp1_max_angle = 100 DEG2RAD;
		grasp1_min_angle = -100 DEG2RAD;
		grasp1_max_limit = 85 DEG2RAD;
		grasp1_min_limit = -85 DEG2RAD;

		grasp2_max_angle = 100 DEG2RAD;
		grasp2_min_angle = -100 DEG2RAD;
		grasp2_max_limit = 85 DEG2RAD;
		grasp2_min_limit = -85 DEG2RAD;
		break;

	case potts_scissor:
		rot_max_angle = 260 DEG2RAD;
		rot_min_angle = -260 DEG2RAD; //might not be correct
		rot_max_limit = 182 DEG2RAD;
		rot_min_limit = -182 DEG2RAD;

		wrist_max_angle = 90 DEG2RAD;
		wrist_min_angle = -90 DEG2RAD;
		wrist_max_limit = 75 DEG2RAD;
		wrist_min_limit = -75 DEG2RAD;

		grasp1_max_angle = 120 DEG2RAD;
		grasp1_min_angle = -120 DEG2RAD;
		grasp1_max_limit = 90 DEG2RAD;
		grasp1_min_limit = -90 DEG2RAD;

		grasp2_max_angle = 130 DEG2RAD;
		grasp2_min_angle = -130 DEG2RAD;
		grasp2_max_limit = 90 DEG2RAD;
		grasp2_min_limit = -90 DEG2RAD;
		break;

	case monopolar_cautery:
		break;

	default:
		break;
	}
}

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
		break;
	}

}

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
		break;
	}
}

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
		break;
	}
}

