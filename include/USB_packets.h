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

//#define NUM_DAC_CH 1 //number of DAC channels being used

#define USB_BUSY_WAIT 1000

// Driver Enabled Flag
#define CYPRESS_ENABLED 1

// Packet sizes
/* USB packet lengths */
#define MAX_IN_LENGTH 512
#define MAX_OUT_LENGTH 512

/*
 * Packet Information
 */
#define PACKET_ID 0x00
#define PACKET_NUM_CH 0x01

/*
 * Data Packet Types
 */
#define E_STOP 0x00  // OUT: Emergency Stop

/* Part: LSI LS7266R1 dual 24-bit quadrature counters */
#define RST_ENC 0x01  // OUT: Reset Encoder
#define REQ_ENC 0x02  // OUT: Request Encoder packet
#define ENC 0x03      // IN:  Data packet: encoder counts
#define ENC_VEL 0x04  // IN:  Data packet: encoder counts and velocity

/* Part: TI DAC7731E 16-Bit DACs */
#define RST_DAC 0x05  // OUT: Reset Dac
#define DAC 0x06      // OUT: Data packet: Dac value

/*Part: both counter and DAC */
#define RST_ENC_DAC 0x07  // OUT: Reset Encs and Dac

/* Acks sent back to host */
#define ACK_ESTOP 0x08        // IN: Ack E_STOP
#define ACK_RST_ENC 0x09      // IN: Ack RST_ENC
#define ACK_RST_DAC 0x0A      // IN: Ack RST_DAC
#define ACK_RST_ENC_DAC 0x0B  // IN: Ack RST_DAC
