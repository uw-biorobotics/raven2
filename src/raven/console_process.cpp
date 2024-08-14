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

/**
 *  	\file console_process.cpp
 *
 *	\brief Lets the user set different control modes and outputs data to the
 *console periodically.
 *   	User can toggle to either specify joint torque, set control mode, or
 *toggle console messages.
 *
 *  	\author Hawkeye King
 *
 * 	\fn These are the 3 functions in console_process.cpp file.
 *           Functions marked with "*" are called explicitly from other files.
 * 	       *(1) console_process	 	:uses (2)(3)
 *       	(2) getkey
 * 		(3) outputRobotState
 *
 *  	\date ??
 *
 *  	\ingroup IO
 */

#include <cstdio>
#include <iomanip>
#include <termios.h>  // needed for terminal settings in getkey()
#include <queue>

#include "console_process.h"
#include "local_io.h"
#include "rt_process_preempt.h"
#include "rt_raven.h"

#ifdef UBUNTU_20
  #include <unistd.h> // needed for getkey on Ubuntu 20
#else
#endif


using namespace std;

// from rt_process.cpp
extern device device0;  // robot_device  defined in DS0.h

extern unsigned long int gTime;  // Defined in rt_process_preempt.cpp
extern int soft_estopped;        // Defined in rt_process_preempt.cpp
extern DOF_type DOF_types[];     // Defined in globals.cpp
extern std::queue<char *> msgqueue;

void outputRobotState();
int getkey();

/**
 *	\fn void *console_process(void *)
 *
 * 	\brief this thread is dedicated to console io
 *
 * 	\desc user sends commands via console where output messages are also
 *displayed
 *
 * 	\param a pointer to void
 *
 *	\ingroup IO
 *
 *	\return void
 */
void *console_process(void *) {
  ros::Time t1, t2;
  ros::Duration d;
  t1 = t1.now();
  t2 = t2.now();

  // Low priority non realtime thread
  sched_param param;  // priority settings
  param.sched_priority = 0;
  if (sched_setscheduler(0, SCHED_OTHER, &param) == -1) {
    perror("sched_setscheduler failed for console process");
    exit(-1);
  }

  int output_robot = false;
  int theKey, print_msg = 1;
  char inputbuffer[100];
  sleep(1);
  // Run shell interaction
  while (ros::ok()) {
    // Output UI hints
    if (print_msg) {
      log_msg("[[\t'C'    : toggle console messages ]]");
      log_msg("[[\t'T'    : specify joint torque    ]]");
      log_msg("[[\t'M'    : set control mode        ]]");
      log_msg("[[\t'U/D'  : Pedal Up/Down           ]]");
      log_msg("[[\t'^C'   : Quit                    ]]");
      print_msg = 0;
    }
    // Get UI command
    theKey = getkey();
    switch (theKey) {
      case 'z': {
        output_robot = 0;
        setDofTorque(0, 0, 0);  /// only mech 0 dof 0 -> 0 torque??
        log_msg("Torque zero'd");
        print_msg = 1;
        break;
      }
      case 'd':
      case 'D': {
        log_msg("pedal down");
        setSurgeonMode(1);
        updateMasterRelativeOrigin(&device0);
        break;
      }
      case 'u':
      case 'U': {
        log_msg("pedal up");
        setSurgeonMode(0);
        updateMasterRelativeOrigin(&device0);
        break;
      }
      case 'e':
      case 'E':
      case '0': {
        output_robot = 0;
        soft_estopped = TRUE;
        print_msg = 1;
        log_msg("Soft estopped");
        break;
      }
      case '+':
      case '=': {
        soft_estopped = FALSE;
        print_msg = 1;
        device0.crtk_state.reset_estop_trigger();
        log_msg("Soft estop off");
        break;
      }
      case 'c':
      case 'C': {
        log_msg("Console output on:%d", output_robot);
        output_robot = !output_robot;
        print_msg = 1;
        break;
      }
      case 't':
      case 'T': {
        print_msg = 1;
        // Get user-input mechanism #
        printf("\n\nEnter a mechanism number: 0-Gold, 1-Green, 2-Blue, 3-Orange:\t");
        cin.getline(inputbuffer, 100);
        unsigned int _mech = atoi(inputbuffer);
        if (_mech > MAX_MECH) break;

        // Get user-input joint #
        printf(
            "\nEnter a joint number: 0-shoulder, 1-elbow, 2-zins, "
            "4-tool_roll, 5-wrist, 6/7- grasp 1/2:\t");
        cin.getline(inputbuffer, 100);
        unsigned int _joint = atoi(inputbuffer);
        if (_joint >= MAX_DOF_PER_MECH) break;

// Get user-input DAC value #
#ifndef DAC_TEST
        printf("\nEnter a torque value in miliNewton-meters:\t");
#else
        printf("\nEnter a torque value in DAC UNITS:\t");
#endif
        cin.getline(inputbuffer, 100);
        int _torqueval = atoi(inputbuffer);

        log_msg("Commanded mech.joint (tau):%d.%d (%d))\n", _mech, _joint, _torqueval);
        setDofTorque(_mech, _joint, _torqueval);
        break;
      }
      case 'm':
      case 'M': {
        // Get user-input DAC value #
        printf(
            "\n\nEnter new control mode: 0=NULL, 1=NULL, 2=joint_velocity, "
            "3=apply_torque, 4=homing, 5=motor_pd, 6=cartesian_space_motion, "
            "7=multi_dof_sinusoid 8=teleop_third_arm\t");
        cin.getline(inputbuffer, 100);
        t_controlmode _cmode = (t_controlmode)(atoi(inputbuffer));
        log_msg("received control mode:%d\n\n", _cmode);
        setRobotControlMode(_cmode);
        print_msg = 1;
        break;
      }
    }

    // Output the robot state once/sec
    if (output_robot && (t1.now() - t1).toSec() > 1) {
      outputRobotState();
      t1 = t1.now();
    }

    usleep(33 * 1e3);  // Sleep for 1/30 seconds

    // Output log messages
    while (msgqueue.size() > 0) {
      char *buf = msgqueue.front();
      // std::cout << "buf\n" << buf << std::endl;
      ROS_INFO("%s", buf);
      msgqueue.pop();
      free(buf);
    }
  }

  return (NULL);
}

/**
 *	\fn int getkey()
 *
 *	\brief gets keyboard character for switch case's of console_process()
 *
 * 	\return returns keyboard character
 *
 *	\ingroup IO
 *
 *	\return character int
 */
int getkey() {
  int character;
  termios orig_term_attr;
  termios new_term_attr;

  /* set the terminal to raw mode */
  tcgetattr(fileno(stdin), &orig_term_attr);
  memcpy(&new_term_attr, &orig_term_attr, sizeof(termios));
  new_term_attr.c_lflag &= ~(ECHO | ICANON);
  new_term_attr.c_cc[VTIME] = 0;
  new_term_attr.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

  /* read a character from the stdin stream without blocking */
  /*   returns EOF (-1) if no character is available */
#ifdef UBUNTU_20
  read(STDIN_FILENO, &character, 1);
  if (ferror(stdin))
    log_msg("i/O error when reading from keyboard");
#else
  character = fgetc(stdin);
#endif


  /* restore the original terminal attributes */
  tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

  return character;
}

/**
 *	\fn void outputRobotState()
 *
 *	\brief prints out all the robot's states on the console window
 *
 *	\ingroup IO
 *
 *	\return void
 */
void outputRobotState() {
  cout << "Runlevel: " << static_cast<unsigned short int>(device0.runlevel) << "\n";
  int offset;

  for (int j = 0; j < device0.num_mechs; j++) {
    if (device0.mech[j].name == gold){
      cout << "Gold arm:\t" << "type: " << device0.mech[j].type;
      offset = 0;
    }
    else if (device0.mech[j].name == green){
      cout << "Green arm:\t " << "type: " << device0.mech[j].type;
      offset = 1 * MAX_DOF_PER_MECH;
    }
    else if (device0.mech[j].name == blue){
      cout << "Blue arm:\t" <<  "type: " << device0.mech[j].type;
      offset = 2 * MAX_DOF_PER_MECH;
    }
    else if (device0.mech[j].name == orange){
      cout << "Orange arm:\t" << "type: " << device0.mech[j].type;
      offset = 3 * MAX_DOF_PER_MECH;
    }
    else
      cout << "Unknown arm:\t";

    cout << ",  Board " << j << ",  serial " << device0.mech[j].serial << ":\n";
    cout << "P: (x,y,z) : (" << device0.mech[j].pos.x / (1000.0 * 1000.0) << "\t"
         << device0.mech[j].pos.y / (1000.0 * 1000.0);
    cout << "\t" << device0.mech[j].pos.z / (1000.0 * 1000.0);
    cout << ") :\t";
    cout << "PD: (x,y,z) : (" << device0.mech[j].pos_d.x / (1000.0 * 1000.0) << "\t"
         << device0.mech[j].pos_d.y / (1000.0 * 1000.0);
    cout << "\t" << device0.mech[j].pos_d.z / (1000.0 * 1000.0);
    cout << ") :\t";
    cout << " Grasp/d:" << (double)device0.mech[j].ori.grasp / 1000.0 << "/"
         << (double)device0.mech[j].ori_d.grasp / 1000.0 << "\n";

    cout << "type:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++) cout << device0.mech[j].joint[i].type << "\t";
    cout << "\n";

    cout << "enc_val:\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++) cout << device0.mech[j].joint[i].enc_val << "\t";
    cout << "\n";

    cout << "enc_off:\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++) cout << device0.mech[j].joint[i].enc_offset << "\t";
    cout << "\n";

    cout << "mpos:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      cout << fixed << setprecision(2) << device0.mech[j].joint[i].mpos << "\t";
    cout << "\n";

    cout << "mpos_d:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      cout << fixed << setprecision(2) << device0.mech[j].joint[i].mpos_d << "\t";
    cout << "\n";

    cout << "mvel:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      cout << fixed << setprecision(0) << device0.mech[j].joint[i].mvel << "\t";
    cout << "\n";

    cout << "mvel_d:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      cout << fixed << setprecision(0) << device0.mech[j].joint[i].mvel_d << "\t";
    cout << "\n";

    cout << "jpos:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      if (i != 2)
        cout << fixed << setprecision(3) << device0.mech[j].joint[i].jpos << "\t";
      else
        cout << fixed << setprecision(3) << device0.mech[j].joint[i].jpos << "\t";
    cout << "\n";

    cout << "jpos_d:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      if (i != 2)
        cout << fixed << setprecision(3) << device0.mech[j].joint[i].jpos_d << "\t";
      else
        cout << fixed << setprecision(3) << device0.mech[j].joint[i].jpos_d << "\t";
    cout << "\n";

    cout << "jvel:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      if (i != 2)
        cout << fixed << setprecision(3) << device0.mech[j].joint[i].jvel << "\t";
      else
        cout << fixed << setprecision(3) << device0.mech[j].joint[i].jvel << "\t";
    cout << "\n";

    cout << "jvel_d:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      if (i != 2)
        cout << fixed << setprecision(2) << device0.mech[j].joint[i].jvel_d << "\t";
      else
        cout << fixed << setprecision(2) << device0.mech[j].joint[i].jvel_d << "\t";
    cout << "\n";

    cout << "tau_d:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      cout << fixed << setprecision(3) << device0.mech[j].joint[i].tau_d << "\t";
    cout << "\n";

    cout << "tau:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      cout << fixed << setprecision(3) << device0.mech[j].joint[i].tau << "\t";
    cout << "\n";

    // cout << "tau_g:\t\t";
    // for (int i = 0; i < MAX_DOF_PER_MECH; i++)
    //   cout << fixed << setprecision(3) << device0.mech[j].joint[i].tau_g << "\t";
    // cout << "\n";

    cout << "DAC:\t\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++)
      cout << fixed << setprecision(3) << (device0.mech[j].joint[i].current_cmd -
                                           DOF_types[offset + i].DAC_zero_offset)
           << "\t";
    cout << "\n";

    // cout << "KP gains:\t";
    // for (int i = 0; i < MAX_DOF_PER_MECH; i++)
    //   cout << fixed << setprecision(3) << DOF_types[offset + i].KP << "\t";
    // cout << "\n";

    // cout << "KD gains:\t";
    // for (int i = 0; i < MAX_DOF_PER_MECH; i++)
    //   cout << fixed << setprecision(3) << DOF_types[offset + i].KD << "\t";
    // cout << "\n";
    /*
      cout<<"jac force:\t";
      float forces[6];
      device0.mech[j].r2_jac.get_force(forces);
      for (int i=0; i < 6;i++)
      cout<<fixed<<"\t"<<setprecision(3)<<forces[i];
      cout<<"\n";

      cout<<"velocity:\t";
      float velocity[6];
      device0.mech[j].r2_jac.get_vel(velocity);
      for (int i=0; i < 6;i++)
      cout<<fixed<<"\t"<<setprecision(3)<<velocity[i];
      cout<<"\n";
     */

    if(JOINT_ENCODERS){
      cout << "j_enc_val:\t";
      for (int i = 0; i < MAX_DOF_PER_MECH / 2; i++)
        cout << device0.mech[j].joint[i].joint_enc_val << "\t";
      cout << "\n";

      cout << "j_enc_off:\t";
      for (int i = 0; i < MAX_DOF_PER_MECH / 2; i++)
        cout << device0.mech[j].joint[i].joint_enc_offset << "\t";
      cout << "\n";

      cout << "j_enc_pos:\t";
      for (int i = 0; i < MAX_DOF_PER_MECH / 2; i++)
        cout << device0.mech[j].joint[i].j_enc_pos << "\t";
      cout << "\n";
    }
    //
    //        cout<<"enc_offset:\t";
    //        for (int i=0;i<MAX_DOF_PER_MECH;i++)
    //            cout<<device0.mech[j].joint[i].enc_offset<<"\t";
    //        cout<<"\n";


#ifdef PRINT_J_STATE
    cout << "j_state:\t";
    for (int i = 0; i < MAX_DOF_PER_MECH; i++) cout << device0.mech[j].joint[i].state << "\t";
    cout << "\n";

#endif

    cout << "\n";
  }

}
