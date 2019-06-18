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
 * \brief Class file for CRTK API state and status flags
 *
 *
 * \date Oct 18, 2018
 * \author Andrew Lewis
 * \author Melody Yun-Hsuan Su
 *
 */

#include "crtk_state.h"
#include "defines.h"
#include "local_io.h"
#include "update_device_state.h"

#include "rt_process_preempt.h"
extern int soft_estopped;

/** CRTK_state object constructor
*
*/
CRTK_state::CRTK_state() {
  is_disabled 	= 1;
  is_enabled 	= 0;
  is_paused 	= 0;
  is_fault 		= 0;
  is_homing 	= 0;
  is_busy 	= 0;
  is_ready 		= 0;
  is_homed		= 0;

  pedal_trigger = 0;
  estop_trigger = 0;
  unhome_trigger = 0;
  home_trigger = 0;
  raven_runlevel = RL_E_STOP;
  transition_err = 0;
}

/** updates CRTK API state and statuses (statii?) based on RAVEN state and statuses
*
*	\param robot_state		RAVEN runlevel
*	\param robot_homed		have all joints homed?
*	\param robot_fault		was a HW or SW e-stop triggered?
*	
*	\output negative if error
*/

char CRTK_state::state_machine_update(char robot_state, bool robot_homed, bool robot_fault, int surgeon_mode, char current_estop_level){
  
  raven_runlevel = robot_state;

  int success = 0;
  if(robot_state == RL_E_STOP){
  	success += set_busy(0);
  	// success += set_ready(0);
    if(robot_fault == 1){
      success += set_fault_state();
    }else if(robot_homed == 1 && current_estop_level == 1){
      success += set_paused_state();
    }else{
      success += set_disabled_state();
    }
  	
    if(get_home_trigger()){
      t_controlmode _cmode = homing_mode;
      setRobotControlMode(_cmode);
      reset_home_trigger();
      ROS_INFO("Starting CRTK homing command.") ;
      ROS_INFO("Press and release E-stop. Then press the silver button.");
      soft_estopped = FALSE;
    }
  }

  else if(robot_state == RL_INIT){
  	success += set_enabled_state();
  	success += set_busy(1);
  	// success += set_ready(0);
  }

  else if(!surgeon_mode){
  	success += set_paused_state();
  	success += set_busy(0);
  	// success += set_ready(0);
  }

  else if(surgeon_mode){
  	success += set_enabled_state();

  	//set_busy(1); // \TODO is the robot actively busy? is it owned by some node
  	// success += set_ready(1);
  }


  if(robot_homed) success += set_homed(1);
  else success += set_homed(0);
  success += set_homing();

  return success;
}

char CRTK_state::set_estop_trigger(int level){
  // CRTK_estop_levels:
  // {CRTK_NOT_ESTOP, CRTK_ESTOP_PAUSE, CRTK_ESTOP_DISABLE}

  estop_trigger = level;
  return estop_trigger;
}

char CRTK_state::set_unhome_trigger(){
  unhome_trigger = 1;
  is_homed = 0;
  ROS_INFO("Unhoming the robot. Needs to be rehomed.");
  return unhome_trigger;
}

char CRTK_state::set_home_trigger(){
  home_trigger = 1;
  is_homed = 0;
  return home_trigger;
}


char CRTK_state::set_pd_up_trigger(){
  pedal_trigger = 1;
  return pedal_trigger;
}


char CRTK_state::set_pd_dn_trigger(){
  pedal_trigger = -1;
  return pedal_trigger;
}

char CRTK_state::reset_unhome_trigger(){
  unhome_trigger = 0;
  return unhome_trigger;
}

char CRTK_state::reset_home_trigger(){
  home_trigger = 0;
  return home_trigger;
}

char CRTK_state::reset_pedal_trigger(){
  pedal_trigger = 0;
  return pedal_trigger;
}

char CRTK_state::reset_estop_trigger(){
  estop_trigger = CRTK_NOT_ESTOP;
  return estop_trigger;
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

char CRTK_state::set_homing(){
	is_homing = is_busy && !is_homed;

	return 0;
}

char CRTK_state::set_busy(bool new_state){
	is_busy = new_state;

	return 0;

}

// char CRTK_state::set_ready(char new_state){
// 	if(good_state_value(new_state))
// 		is_ready = new_state;
// 	else return -1;

// 	return 0;

// }

char CRTK_state::set_homed(bool new_state){
	is_homed = new_state;

	return 0;
}

// bool good_state_value(bool state){
// 	if(state == 1 || state == 0 || state == '?')
// 		return true;
// 	else
// 		return false;
// }


CRTK_robot_state CRTK_state::get_state(){
  if(is_disabled){
    return CRTK_DISABLED;
  }
  else if(is_enabled){
    return CRTK_ENABLED;
  }
  else if(is_paused){
    return CRTK_PAUSED;
  }
  else{
    return CRTK_FAULT;
  }
}

char CRTK_state::get_estop_trigger(){
  return estop_trigger;
}

char CRTK_state::get_pedal_trigger(){
  return pedal_trigger;
}

char CRTK_state::get_unhome_trigger(){
  return unhome_trigger;
}

char CRTK_state::get_home_trigger(){
  return home_trigger;
}

bool CRTK_state::get_disabled(){
	return is_disabled;
}

bool CRTK_state::get_enabled(){
	return is_enabled;
}

bool CRTK_state::get_paused(){
	return is_paused;
}

bool CRTK_state::get_fault(){
	return is_fault;
}


bool CRTK_state::get_homing(){
  return is_homing;
}
bool CRTK_state::get_busy(){
  return is_busy;
}
bool CRTK_state::get_ready(){
  return is_ready;
}
bool CRTK_state::get_homed(){
  return is_homed;
}
/**
 *\brief Callback for the automove topic - Updates the data1 structure
 *
 * \param msg the
 * \ingroup ROS
 *
 */
void CRTK_state::crtk_cmd_cb(crtk_msgs::StringStamped msg){

  std::string command = msg.string;
  ROS_INFO("Received %s", command.c_str());

  if(command == "enable") {
    enable();
  }
  else if (command == "disable") {
    disable();
  }
  else if (command == "pause"){
    pause();
  }
  else if (command == "resume") {
    resume();
  }
  else if (command == "unhome") {
    unhome();
  }
  else if (command == "home") {
    home();
  }
  else
  	return;
}




char CRTK_state::enable()
{
  if(raven_runlevel == RL_E_STOP){
    if(!is_homed){
      ROS_INFO("Please press silver button!"); // init state

  	}
  	else{
  	  //set_pd_dn_trigger(); //p_dn
      setSurgeonMode(1);
  	}
    // the unhome trigger flag is flipped off only when enable command is received.
    reset_unhome_trigger();
  }
  else{
  	transition_err = 1;
  	return -1;
  }

  transition_err = 0;
  return 0;
}

char CRTK_state::disable()
{
  if(is_enabled){
    set_estop_trigger(CRTK_ESTOP_DISABLE); //estop
  }
  else if(raven_runlevel == RL_PEDAL_UP){
  	set_estop_trigger(CRTK_ESTOP_DISABLE); //estop
  }
  else if(raven_runlevel == RL_E_STOP){
  	set_estop_trigger(CRTK_ESTOP_DISABLE*4); //estop(init) --> estop harderrrrrr (disable)
  }
  else{
  	transition_err = 1;
  	return -1;  	
  }
  
  transition_err = 0;
  return 0;
}

char CRTK_state::pause(){

  if(raven_runlevel == RL_INIT){
  	set_estop_trigger(CRTK_ESTOP_PAUSE); 
  }
  else if(raven_runlevel == RL_PEDAL_DN){
  	//set_pd_up_trigger(); //p_up
    setSurgeonMode(0);
  }
  else{
  	transition_err = 1;
  	return -1;  	
  }
   
  transition_err = 0;
  return 0;
}

char CRTK_state::resume(){
  if(raven_runlevel == RL_PEDAL_UP){
	 //set_pd_dn_trigger(); //p_dn
   setSurgeonMode(1);
  }
  else if(raven_runlevel == RL_E_STOP){
	 ROS_INFO("Please press silver button!"); // init state
  }
  else{
  	transition_err = 1;
  	return -1;  	
  }
  
  transition_err = 0;
  return 0;
}


char CRTK_state::unhome()
{
  set_homed(0);
  set_estop_trigger(CRTK_ESTOP_DISABLE);
  set_unhome_trigger();
  setSurgeonMode(0);
  transition_err = 0;
  return 0;
}


char CRTK_state::home()
{
  if(raven_runlevel == RL_INIT || RL_PEDAL_DN || RL_PEDAL_UP || RL_E_STOP){
    unhome_trigger = 1;
    is_homed = 0;
    ROS_INFO("Unhoming the robot. Preparing for rehomed.");
    set_estop_trigger(CRTK_ESTOP_DISABLE);
    setSurgeonMode(0);

    set_home_trigger();
    //reset_unhome_trigger();
  }
  else{
    transition_err = 1;
    return -1;    
  }
  
  transition_err = 0;
  return 0;
}

std::string CRTK_state::get_state_string(){
  std::string out;

  if(is_disabled)       out = "DISABLED";
  else if (is_enabled)  out = "ENABLED";
  else if (is_paused)   out = "PAUSED";
  else if (is_fault)    out = "FAULT";
  else out = "FAULT"; //no connection?

  return out;
}