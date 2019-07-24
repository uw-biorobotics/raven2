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
 * crtk_state.h
 *
 * \brief Class file for CRTK state object, which holds all of the state
 *  flags for the "Robot Operating State" aspect of the CRTK API
 *
 *  \date Oct 17, 2018
 *  \author Andrew Lewis, Yun-Hsuan Su

 */
#ifndef CRTK_STATE_H_
#define CRTK_STATE_H_

#include <ros/ros.h>
#include <crtk_msgs/StringStamped.h>
#include <std_msgs/String.h>



enum CRTK_robot_state {CRTK_ENABLED, CRTK_DISABLED, CRTK_PAUSED, CRTK_FAULT, CRTK_VOID};
enum CRTK_robot_command {CRTK_ENABLE, CRTK_DISABLE, CRTK_PAUSE, CRTK_RESUME, CRTK_UNHOME, CRTK_HOME};
enum CRTK_estop_level {CRTK_NOT_ESTOP, CRTK_ESTOP_PAUSE, CRTK_ESTOP_DISABLE};


class CRTK_state 
{
 public:
  
  // methods
  CRTK_state();

  ~CRTK_state(){};


  char set_homing();
  char set_busy(bool new_state);
  // char set_ready(char new_state);
  char set_homed(bool new_state);

  CRTK_robot_state get_state();
  bool get_disabled();
  bool get_enabled();
  bool get_paused();
  bool get_fault();
  bool get_homing();
  bool get_busy();
  bool get_ready();
  bool get_homed();
  char get_estop_trigger();
  char get_pedal_trigger();
  char get_unhome_trigger();
  char get_home_trigger();
  char reset_estop_trigger();
  char reset_pedal_trigger();
  char reset_unhome_trigger();

  char state_machine_update(char, bool, bool, int, char);
  void crtk_cmd_cb(crtk_msgs::StringStamped);

  std::string get_state_string();
  
private:
  char pedal_trigger;   // 0: neutral, 1: set pedal up, -1: set pedal dn
  char estop_trigger;   // 0: neutral, >0: set e-stop (1: in CRTK pause state, >1: in disabled state)
  char unhome_trigger;  // 0: neutral, 1: make unhome
  char home_trigger;

  bool is_disabled;
  bool is_enabled;
  bool is_paused;
  bool is_fault;
  bool is_homing;
  bool is_busy;
  bool is_ready;
  bool is_homed;
  std::string command;
  char raven_runlevel;
  char transition_err;

  char set_disabled_state();
  char set_enabled_state();
  char set_paused_state();
  char set_fault_state();

  char set_estop_trigger(int);
  char set_pd_up_trigger();
  char set_pd_dn_trigger();
  char set_unhome_trigger();
  char set_home_trigger();
  char reset_home_trigger();

  char enable();
  char disable();
  char pause();
  char resume();
  char unhome();
  char home();

};

bool good_state_value(char);

#endif /* CRTK_STATE_H_ */
