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
 * \file: overdrive_detect.c 
 * \author Kenneth Fodero
 * \version 2005
 * \brief  Functions related to checking for motor over heating
 *
 * \ingroup Control
 *
 * 5/06 Modified by Hawkeye King
 */

#include "overdrive_detect.h"

extern struct DOF_type DOF_types[];//Defined in globals.cpp
extern int NUM_MECH; //Defined in rt_process_preempt.cpp
extern int soft_estopped;//Defined in rt_process_preempt.cpp
extern unsigned long int gTime;//Defined in rt_process_preempt.cpp

/**
 * \brief detect over current
 * \param device0 pointer to robot_device struct defined in DS0.h
 *
 * \output TRUE if the instant DAC value is too high
 * \output FALSE otherwise
 *
 * This function loops through all active joints to detect currrent situations
 * that could cause overheating, it checks joint current_cmd against MAX_INST_DAC that is
 * defined in defines.h
 */
int overdriveDetect(struct device *device0)
{
    int i, j;
    struct DOF* _joint;
    int ret = FALSE;


    for (i = 0; i < NUM_MECH; i++)
        for (j = 0; j < (MAX_DOF_PER_MECH-1); j++)
        {
            _joint = &(device0->mech[i].joint[j]);
            int _dac_max = DOF_types[_joint->type].DAC_max;

            // Kill current if greater than MAX_INST_DAC.  Probably indicates a problem.
            if (abs(_joint->current_cmd) > MAX_INST_DAC)
            {
                log_msg("Instant i command too high. Joint type: %d DAC:%d \t tau:%0.3f\n", _joint->type, _joint->current_cmd, _joint->tau_d);
                _joint->current_cmd = 0;
                ret = TRUE;
            }

            else if ( _joint->current_cmd > _dac_max )
            {
                //Clip current to max_torque
                if (gTime %100 == 0) //don't saturate the console
                    err_msg("Joint type %d is current clipped high (%d) at DAC:%d\n", _joint->type, _dac_max, _joint->current_cmd);
                _joint->current_cmd = _dac_max;
            }

            else if ( _joint->current_cmd < _dac_max*-1 )
            {
                //Clip current to -1*max_torque
                if (gTime %100 == 0)
                    err_msg("Joint type %d is current clipped low (%d) at DAC:%d\n", _joint->type, _dac_max*-1,  _joint->current_cmd);
                _joint->current_cmd = _dac_max*-1;
            }
        }

    return ret;
}
