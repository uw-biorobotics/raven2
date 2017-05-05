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
 * hmatrix.h
 *
 *  Created on: Jan 22, 2010
 *      Author: glozmand
 */

#ifndef HMATRIX_H_
#define HMATRIX_H_

#include "struct.h" /*Includes DS0, DS1, DOF_type, defines, etc*/
#include "utils.h"

#define HMATRIX_SIZE 4
#define SQR(x)  ( (x) * (x) )

#define ABS(x)  ( (x) > (0) ? (x) : (-(x)) )


void mulMatrix(float A[HMATRIX_SIZE][HMATRIX_SIZE], float B[HMATRIX_SIZE][HMATRIX_SIZE], float C[HMATRIX_SIZE][HMATRIX_SIZE]);

void mulMatrix3x3i(int A[3][3], int B[3][3], int C[3][3]);
void mulMatrix3x3f(float A[3][3], float B[3][3], float C[3][3]);
void transpose3x3f(float R[3][3], float R_transpose[3][3]);

void invOrthMatrix(float tip[HMATRIX_SIZE][HMATRIX_SIZE], float tip_inv[HMATRIX_SIZE][HMATRIX_SIZE]);
void printMatrix4x4(double rot_matrix[4][4], char *str);
void printMatrix3x3(double rot_matrix[3][3], char *str);
void printMatrix3x3i(int rot_matrix[3][3], char *str);

void orientation_from_hmatrix(float tip[HMATRIX_SIZE][HMATRIX_SIZE], struct orientation *ori);
void hmatrix_from_orientation(struct orientation *ori, double tip[HMATRIX_SIZE][HMATRIX_SIZE]);

void create_rotation_matrix(float n[3], float phi, float mr[3][3]);
void create_rotation_from_rmatrix(float R[3][3], float rvec[3], float *angle);

#endif /* HMATRIX_H_ */
