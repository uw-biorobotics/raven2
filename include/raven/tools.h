/*
 * tools.h
 *
 *  Created on: Oct 17, 2014
 *      Author: junjie
 */

#ifndef TOOLS_H_
#define TOOLS_H_

enum style{
	dv,
	raven,
	square_raven};

enum end_effector_type  {
	r_grasper,
	r_sq_grasper,
	large_needle,
	micro_forceps,
	bipolar_forceps,
	cardiere_forceps,
	mopocu_scissor,
	potts_scissor,
	monopolar_cautery};


class tool{
public:

	end_effector_type t_end;

	style t_style; //the style is set based on end_effector

	int mech_type;

	float wrist_coupling;

	//angle is the maximum physical range of DOF
	//limit is the safe limit for joint saturation
	float rot_max_angle;
	float rot_min_angle;
	float rot_max_limit;
	float rot_min_limit;

	float wrist_max_angle;
	float wrist_min_angle;
	float wrist_max_limit;
	float wrist_min_limit;

	float grasp1_max_angle;
	float grasp1_min_angle;
	float grasp1_max_limit;
	float grasp1_min_limit;

	float grasp2_max_angle;
	float grasp2_min_angle;
	float grasp2_max_limit;
	float grasp2_min_limit;

	float max_opening_angle; //max angle between graspers

	//angles to return to after homing
	float rot_home_angle;
	float wrist_home_angle;
	float grasp1_home_angle;
	float grasp2_home_angle;

	//DanyingHu parameters
	float shaft_length;
	float wrist_length;

//methods

    tool(){};
    tool(end_effector_type, int);

	~tool(){};

//	void set_tool(end_effector_type, int);
	void set_tool(end_effector_type, int);

	void set_tool_data();

	void set_limits();

	void set_home_angles();

	void set_wrist_coupling();

	void set_DH_params();

	void set_max_opening_angle();

};


#endif /* TOOLS_H_ */
