/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2018  Andrew Lewis, Yun-Hsuan Su, Blake Hannaford, 
 * and the University of Washington BioRobotics Laboratory
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
 * crtk_state.cpp
 *
 * \brief Class file for Tool object
 *
 * This is a new feature added to the RAVEN code in the indigo branch. This
 * allows us to use different tools on each arm and allows for differences
 * between tools and scissors. The interface type needs to be the same between
 * each arm (RAVEN or dV adapter) and must still be declared as a #define in
 *defines.h
 *
 * \date Oct 17, 2014
 * \author Andrew Lewis
 * \author Danying Hu
 * \author David Caballero
 *
 */

#include "crtk_state.h"
#include "defines.h"

/** CRTK_state object constructor
*
*/
CRTK_state::CRTK_state() {
  is_disabled 	= 1;
  is_enabled 	= 0;
  is_paused 	= 0;
  is_fault 		= 0;
  is_homing 	= 0;
  is_moving 	= 0;
  is_ready 		= 0;
  is_homed		= 0;
}

/** updates CRTK API state and statuses (statii?) based on RAVEN state and statuses
*
*	\param robot_state		RAVEN runlevel
*	\param robot_homed		have all joints homed?
*	\param robot_fault		was a HW or SW e-stop triggered?
*	
*	\output negative if error
*/

char CRTK_state::state_machine_update(char robot_state, char robot_homed, char robot_fault, int surgeon_mode){
  int success = 0;
  if(robot_state == RL_E_STOP){
  	success += set_moving(0);
  	success += set_ready(0);
  	success += set_homing(0); //no ability to use partially-homed joints

  	if(robot_homed){
  		success += set_paused_state();
  	}else if (robot_fault == 0){
  		success += set_disabled_state();
  	}else // fault
  		success += set_fault_state();
  }

  else if(robot_state == RL_INIT){
  	success += set_enabled_state();
  	success += set_homing(1);
  	success += set_moving(1);
  	success += set_ready(0);
  }

  else if(!surgeon_mode){
  	success += set_paused_state();
  	success += set_moving(0);
  	success += set_homing(0);
  	success += set_ready(0);
  }

  else if(surgeon_mode){
  	success += set_enabled_state();

  	success += set_homing(0);
  	//set_moving(1); // \TODO is the robot actively moving? is it owned by some node
  	success += set_ready(1);
  }

  if(robot_homed) success += set_homed(1);

  return success;
}


char CRTK_state::set_disabled_state(){
	is_disabled = 1;
	is_enabled 	= 0;
	is_paused 	= 0;
	is_fault 	= 0;

	return 0;

}

char CRTK_state::set_enabled_state(){
	is_disabled = 0;
	is_enabled 	= 1;
	is_paused 	= 0;
	is_fault 	= 0;

	return 0;
}

char CRTK_state::set_paused_state(){
	is_disabled = 0;
	is_enabled 	= 0;
	is_paused 	= 1;
	is_fault 	= 0;

	return 0;

}

char CRTK_state::set_fault_state(){
	is_disabled = 0;
	is_enabled 	= 0;
	is_paused 	= 0;
	is_fault 	= 1;

	return 0;

}

char CRTK_state::set_homing(char new_state){
	if(good_state_value(new_state))
		is_homing = new_state;
	else return -1;

	return 0;
}

char CRTK_state::set_moving(char new_state){
	if(good_state_value(new_state))
		is_moving = new_state;
	else return -1;

	return 0;

}

char CRTK_state::set_ready(char new_state){
	if(good_state_value(new_state))
		is_ready = new_state;
	else return -1;

	return 0;

}

char CRTK_state::set_homed(char new_state){
	if(good_state_value(new_state))
		is_homed = new_state;
	else return -1;

	return 0;
}

bool good_state_value(char state){
	if(state == 1 || state == 0 || state == '?')
		return true;
	else
		return false;
}


char CRTK_state::get_disabled(){
	return is_disabled;
}

char CRTK_state::get_enabled(){
	return is_enabled;
}

char CRTK_state::get_paused(){
	return is_paused;
}

char CRTK_state::get_fault(){
	return is_fault;
}
