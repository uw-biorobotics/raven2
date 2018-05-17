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
 * update_atmel_io.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "struct.h"
#include "defines.h"

// Bit Defines
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

// Output Pins

#ifdef RAVEN_I
#define PIN_LS0 BIT0
#define PIN_LS1 BIT1
#define PIN_FP BIT2
#define PIN_READY BIT3
#define PIN_WD BIT4

#else
#define PIN_LS0 BIT0
#define PIN_LS1 BIT1
#define PIN_FP BIT2
#define PIN_READY BIT3
#define PIN_WD BIT4
#endif

// Input Pins
#define PIN_PS0 BIT6  // state bit 0  (LSB)
#define PIN_PS1 BIT7  // state bit 1  (MSB)

// Function Prototypes
void updateAtmelOutputs(device *device0, int runlevel);
void updateAtmelInputs(device device0, int runlevel);
