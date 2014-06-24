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
 * PREEMPT_RT Raven control implementation
 * RTAI version info
 *------------------------------------------------*
 *   Added data_router module
 *   Data_router module put in ifdef
 * RTAI version written by Ken Fodero, Hawkeye King
 * BioRobotics Lab, University of Washington
 * ken@ee.washington.edu
 *
 */

 /**
 *  \file rt_process_preempt.cpp
 *  \author Hawkeye King and Ken Fodero
 *  \brief PREEMPT_RT Raven control implementation
 *     \ingroup Control
 *
 *  Configures and starts the RAVEN control RT process.
 */

#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h> // Needed for mlockall()
#include <unistd.h> // needed for sysconf(int name);
#include <malloc.h>
#include <sys/time.h> // needed for getrusage
#include <sys/resource.h> // needed for getrusage
#include <sched.h>
#include <stropts.h>
#include <time.h>
#include <pthread.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h> //Needed for umask

#include <ros/ros.h>     // Use ROS

#include "rt_process_preempt.h"
#include "console_process.h"
#include "rt_raven.h"
#include "r2_kinematics.h"
#include "network_layer.h"
#include "parallel.h"
#include "reconfigure.h"

using namespace std;

// Defines
#define POOLSIZE (200*1024*1024) // 200 MB   Size of mlocked memory pool

#define NS  1
#define US  (1000 * NS)
#define MS  (1000 * US)
#define SEC (1000 * MS)

//Global Variables
unsigned long int gTime;
int initialized=0;     // State initialized flag
int soft_estopped=0;   // Soft estop flag- indicate desired software estop.

int    deviceType = SURGICAL_ROBOT;//PULLEY_BOARD;
struct device device0 ={0};  //Declaration Moved outside rt loop for access from console thread
int    mech_gravcomp_done[2]={0};

int NUM_MECH=0;   // Define NUM_MECH as a C variable, not a c++ variable

pthread_t rt_thread;
pthread_t net_thread;
pthread_t console_thread;
pthread_t reconfigure_thread;

//Global Variables from globals.c
extern struct DOF_type DOF_types[];

// flag to kill loops and stuff
int r2_kill = 0;

/**
* Traps the Ctrl-C Signal
* \param sig The signal number sent.
*     \ingroup Control
*/
void sigTrap(int sig){
  log_msg("r2_control terminating on signal %d\n", sig);
  r2_kill = 1;
  if (ros::ok()) ros::shutdown();
}

/**
 *  From PREEMPT_RT Dynamic memory allocation tips page.
 *  This function creates a pool of memory in ram for use with any malloc or new calls so that they do not cause page faults.
 *  https://rt.wiki.kernel.org/index.php/Dynamic_memory_allocation_example
 *     \ingroup Control
 */
int initialize_rt_memory_pool()
{
  int i, page_size;
  char* buffer;

  // Now lock all current and future pages from preventing of being paged
  if (mlockall(MCL_CURRENT | MCL_FUTURE ))
    {
      perror("mlockall failed:");
      return -1;
    }
  mallopt (M_TRIM_THRESHOLD, -1);  // Turn off malloc trimming.
  mallopt (M_MMAP_MAX, 0);         // Turn off mmap usage.

  page_size = sysconf(_SC_PAGESIZE);
  buffer = (char *)malloc(POOLSIZE);

  // Touch each page in this piece of memory to get it mapped into RAM for performance improvement
  // Once the pagefault is handled a page will be locked in memory and never given back to the system.
  for (i=0; i < POOLSIZE; i+=page_size)
    {
      buffer[i] = 0;
    }
  free(buffer);        // buffer is now released but mem is locked to process

  return 0;
}

/**
 * This is the real time thread.
 *
 *     \ingroup Control
 */
static void *rt_process(void* )
{
  struct param_pass currParams =
    {
      0
    };          // robot command struct
  struct param_pass rcvdParams =
    {
      0
    };
  struct timespec t, tnow, t2, tbz;                           // Tracks the timer value
  int interval= 1 * MS;                        // task period in nanoseconds

  //CPU locking doesn't help timing.  Oh well.
  //Lock thread to first available CPU
  // cpu_set_t set;
  // CPU_ZERO(&set);
  // CPU_SET(0,&set);
  // if (sched_setaffinity(0,sizeof(set),&set) < 0)
  //   {
  //     perror("sched_setaffinity() failed");
  //     exit(-1);
  //   }

  // set thread priority and stuff
  struct sched_param param;                    // process / thread priority settings
  param.sched_priority = 96;
  log_msg("Using realtime, priority: %d", param.sched_priority);
  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (ret != 0)
    {
      perror("pthread_setscheduler failed");
      exit(-1);
    }

  currParams.runlevel = STOP;
  currParams.sublevel = 0;

  log_msg("Starting RT Process..");

  // Initializations (run here and again in init.cpp)
  initDOFs(&device0);

  // initialize global loop count
  gTime=0;

  // Setup periodic timer
  clock_gettime(CLOCK_REALTIME,&t);   // get current time
  t.tv_sec += 1;                      // start after short delay
  tsnorm(&t);
  clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);

  log_msg("*** Ready to teleoperate ***");




  // --- Main robot control loop ---
  // TODO: Break when board becomes disconnected.
  while (ros::ok() && !r2_kill)
    {

      // Initiate USB Read
      initiateUSBGet(&device0);

      // Set next timer-shot (must be in future)
      clock_gettime(CLOCK_REALTIME,&tnow);
      int sleeploops = 0;
      while (isbefore(t,tnow))
        {
	  t.tv_nsec+=interval;
	  tsnorm(&t);
	  sleeploops++;
        }
      if (sleeploops!=1)
	std::cout<< "slplup"<< sleeploops <<std::endl;

      parport_out(0x00);
      /// SLEEP until next timer shot
      clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);
      parport_out(0x03);
      gTime++;

      // Get USB data that's been initiated already
      // Get and Process USB Packets

      // HACK HACK HACK
      // loop until data ready
      // better to ensure realtime access to driver
      int loops = 0;
      int ret;

      clock_gettime(CLOCK_REALTIME,&tbz);
      clock_gettime(CLOCK_REALTIME,&tnow);
      while ( (ret=getUSBPackets(&device0)) == -EBUSY && loops < 10)
        {
	  tbz.tv_nsec+=10*US; //Update timer count for next clock interrupt
	  tsnorm(&tbz);
	  clock_nanosleep(0, TIMER_ABSTIME, &tbz, NULL);
	  loops++;
        }
      clock_gettime(CLOCK_REALTIME,&t2);
      t2 = tsSubtract(t2, tnow);
      if (loops!=0)
	std::cout<< "bzlup"<<loops<<"0us time:" << (double)t2.tv_sec + (double)t2.tv_nsec/SEC <<std::endl;

      //Run Safety State Machine
      stateMachine(&device0, &currParams, &rcvdParams);

      //Update Atmel Input Pins
      // TODO: deleteme
      updateAtmelInputs(device0, currParams.runlevel);

      //Get state updates from master
      if ( checkLocalUpdates() == TRUE)
	updateDeviceState(&currParams, getRcvdParams(&rcvdParams), &device0);
      else
	rcvdParams.runlevel = currParams.runlevel;

      //Clear DAC Values (set current_cmd to zero on all joints)
      clearDACs(&device0);

      //////////////// SURGICAL ROBOT CODE //////////////////////////
      if (deviceType == SURGICAL_ROBOT)
        {
	  // Calculate Raven control
	  controlRaven(&device0, &currParams);
        }
      //////////////// END SURGICAL ROBOT CODE ///////////////////////////

      // Check for overcurrent and impose safe torque limits
      if (overdriveDetect(&device0))
        {
	  soft_estopped = TRUE;
	  showInverseKinematicsSolutions(&device0, currParams.runlevel);
	  outputRobotState();
        }
      //Update Atmel Output Pins
      updateAtmelOutputs(&device0, currParams.runlevel);

      //Fill USB Packet and send it out
      putUSBPackets(&device0); //disable usb for par port test

      //Publish current raven state
      publish_ravenstate_ros(&device0,&currParams);   // from local_io

      //Done for this cycle
    }


  log_msg("Raven Control is shutdown");
  return 0;
}














/**
* Initializes USB boards.
*/
int init_module(void)
{
  log_msg("Initializing USB I/O...");

  //Initiailze USB Board
  if (USBInit(&device0) == FALSE)
    {
      err_msg("\nERROR: Could not init USB. Boards on?");
      return STARTUP_ERROR;
    }

  // Initialize Local_io datastructs.
  log_msg("Initializing Local I/O...");
  initLocalioData();

  return 0;
}

/**
* Initializes the raven ROS node
* \param argc Number of string arguments
* \param argv Arguments as character arrays
*/
int init_ros(int argc, char **argv)
{
  /**
   * Initialize ros and rosrt
   */
  ros::init(argc, argv, "r2_control", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //    rosrt::init();
  init_ravenstate_publishing(n);
  init_ravengains(n, &device0);

  return 0;
}

/**
* Main entry point for the raven RT control system.
* \param argc Number of string arguments
* \param argv Arguments as character arrays
*     \ingroup Control
*/
int main(int argc, char **argv)
{
  // set ctrl-C handler (override ROS b/c it's slow to cancel)
  signal( SIGINT,&sigTrap);

  // set parallelport permissions
  ioperm(PARPORT,1,1);

  // init stuff (usb, local-io, rt-memory, etc.);
  if ( init_module() )
    {
      cerr << "ERROR! Failed to init module.  Exiting.\n";
      exit(1);
    }
  if ( init_ros(argc, argv) )
    {
      cerr << "ERROR! Failed to init ROS.  Exiting.\n";
      exit(1);
    }
  if ( initialize_rt_memory_pool() )
    {
      cerr << "ERROR! Failed to init memory_pool.  Exiting.\n";
      exit(1);
    }

  // init reconfigure
  dynamic_reconfigure::Server<raven_2::MyStuffConfig> srv;
  dynamic_reconfigure::Server<raven_2::MyStuffConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1, _2);
  srv.setCallback(f);


  pthread_create(&net_thread, NULL, network_process, NULL); //Start the network thread
  pthread_create(&console_thread, NULL, console_process, NULL);
  pthread_create(&rt_thread, NULL, rt_process, NULL);

  ros::spin();

  USBShutdown();
  //Suspend main until all threads terminate
  pthread_join(rt_thread,NULL);
  pthread_join(console_thread, NULL);
  pthread_join(net_thread, NULL);


  log_msg("\n\n\nI'm shutting down now... \n\n\n");
  usleep(1e6); //Sleep for 1 second

  exit(0);
}

