/*
 * hmatrix.c
 *
 *  Created on: Jan 22, 2010
 *      Author: glozmand
 */

#include <math.h>
#include <stdio.h>
#include "hmatrix.h"

//Mul matrices 4x4 A*B=C
void mulMatrix(float A[HMATRIX_SIZE][HMATRIX_SIZE], float B[HMATRIX_SIZE][HMATRIX_SIZE], float C[HMATRIX_SIZE][HMATRIX_SIZE])
{
	int r, i, j;
	for (r = 0; r < HMATRIX_SIZE ; r++)
		for (i = 0; i < HMATRIX_SIZE; i++)
		{
			C[i][r] = 0;
			for (j = 0; j < HMATRIX_SIZE; j++)
				C[i][r] += A[i][j] * B[j][r];
		}
}

void mulMatrix3x3i(int A[3][3], int B[3][3], int C[3][3])
{
	int r, i, j;
	for (r = 0; r < 3 ; r++)
		for (i = 0; i < 3; i++)
		{
			C[i][r] = 0;
			for (j = 0; j < 3; j++)
				C[i][r] += A[i][j] * B[j][r];
		}
}

void mulMatrix3x3f(float A[3][3], float B[3][3], float C[3][3])
{
	int r, i, j;
	for (r = 0; r < 3 ; r++)
		for (i = 0; i < 3; i++)
		{
			C[i][r] = 0;
			for (j = 0; j < 3; j++)
				C[i][r] += A[i][j] * B[j][r];
		}
}

//Inverse of orthonormal matrix
void invOrthMatrix(float tip[HMATRIX_SIZE][HMATRIX_SIZE], float tip_inv[HMATRIX_SIZE][HMATRIX_SIZE])
{
	int i, j;
	float sum;

	for (i = 0; i < HMATRIX_SIZE ; i++)
	{
		tip_inv[HMATRIX_SIZE-1][i] = tip[HMATRIX_SIZE-1][i];
	}

	for (i = 0; i < HMATRIX_SIZE-1;i++)
	{
		for (j=0; j<(HMATRIX_SIZE-1);j++)
		{
			tip_inv[i][j] = tip[j][i];
		}
	}

	for (i = 0; i < HMATRIX_SIZE-1; i++)
	{
		sum = 0;
		for (j=0; j<HMATRIX_SIZE-1;j++)
		{
			sum += tip_inv[i][j]*tip[j][HMATRIX_SIZE-1];
		}
		tip_inv[i][HMATRIX_SIZE-1] = -sum;
	}
}

void transpose3x3f(float R[3][3], float R_transpose[3][3])
{
	int i, j;

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			R_transpose[i][j] = R[j][i];
		}
	}
}

void printMatrix4x4(msg_level level, double rot_matrix[4][4], char *str)
{
	RTDEBUG(level, "%s", str);
	RTDEBUG(level, "%7.4f %7.4f %7.4f %7.4f", rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2], rot_matrix[0][3]);
	RTDEBUG(level, "%7.4f %7.4f %7.4f %7.4f", rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2], rot_matrix[1][3]);
	RTDEBUG(level, "%7.4f %7.4f %7.4f %7.4f", rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2], rot_matrix[2][3]);
	RTDEBUG(level, "%7.4f %7.4f %7.4f %7.4f", rot_matrix[3][0], rot_matrix[3][1], rot_matrix[3][2], rot_matrix[3][3]);
}

void printMatrix3x3(double rot_matrix[3][3], char *str)
{
	printf("%s\n", str);
	printf("%7.4f %7.4f %7.4f\n", rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2]);
	printf("%7.4f %7.4f %7.4f\n", rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2]);
	printf("%7.4f %7.4f %7.4f\n", rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2]);
}

void printMatrix3x3i(int rot_matrix[3][3], char *str)
{
	printf("%s\n", str);
	printf("%9d %9d %9d\n", rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2]);
	printf("%9d %9d %9d\n", rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2]);
	printf("%9d %9d %9d\n", rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2]);
}

void orientation_from_hmatrix(float tip[HMATRIX_SIZE][HMATRIX_SIZE], struct orientation *ori)
{
	// from here: http://planning.cs.uiuc.edu/node103.html

	ori->yaw = atan2(tip[1][0] , tip[0][0]) * MICRORADS_PER_RAD;
	ori->pitch = atan2( -tip[2][0] , sqrt( SQR(tip[2][1]) + SQR(tip[2][2])) ) * MICRORADS_PER_RAD;
	ori->roll = atan2(tip[2][1] , tip[2][2]) * MICRORADS_PER_RAD;

}

void hmatrix_from_orientation(struct orientation *ori, double tip[HMATRIX_SIZE][HMATRIX_SIZE])
{
	double alpha = ori->yaw / MICRORADS_PER_RAD;
	double beta = ori->pitch / MICRORADS_PER_RAD;
	double gamma = ori->roll / MICRORADS_PER_RAD;

	tip[0][0] = cos(alpha) * cos(beta);
	tip[1][0] = sin(alpha) * cos(beta);
	tip[2][0] = -sin(beta);
	tip[3][0] = 0.0;

	tip[0][1] = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha)*cos(gamma);
	tip[1][1] = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha)*cos(gamma);
	tip[2][1] = cos(beta) * sin(gamma);
	tip[3][1] = 0.0;

	tip[0][2] = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha)*sin(gamma);
	tip[1][2] = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha)*sin(gamma);
	tip[2][2] = cos(beta) * cos(gamma);
	tip[3][2] = 0.0;

	tip[0][3] = 0.0;
	tip[1][3] = 0.0;
	tip[2][3] = 0.0;
	tip[3][3] = 0.0;

	return;
}

void create_rotation_matrix(float n[3], float phi, float mr[3][3])
{
	double sp = sin(phi);
	double s5p = SQR(sin(.5*phi));

	// normalize n
	double n_length = sqrt(SQR(n[0]) + SQR(n[1]) + SQR(n[2]));
	n[0] /= n_length;
	n[1] /= n_length;
	n[2] /= n_length;

	mr[0][0] = 1.-2.*(SQR(n[1])+SQR(n[2])) * s5p;
	mr[0][1] = -n[2]*sp+2.*n[0]*n[1] * s5p;
	mr[0][2] = n[1]*sp+2.*n[2]*n[0] * s5p;

	mr[1][0] = n[2]*sp+2.*n[0]*n[1] * s5p;
	mr[1][1] = 1.-2.*(SQR(n[2])+SQR(n[0])) * s5p;
	mr[1][2] = -n[0]*sp+2.*n[1]*n[2] * s5p;

	mr[2][0] = -n[1]*sp+2.*n[2]*n[0] * s5p;
	mr[2][1] = n[0]*sp+2.*n[1]*n[2] * s5p;
	mr[2][2] = 1.-2.*(SQR(n[0])+SQR(n[1])) * s5p;

}

void create_rotation_from_rmatrix(float R[3][3], float rvec[3], float *angle)
{
	float sin_phi, cos_phi, diag_1;
	float norm;

	diag_1 = R[0][0] + R[1][1] + R[2][2] - 1.0;
	cos_phi = diag_1/2.0;
	sin_phi = 0.5 * sqrt(4.0 - diag_1*diag_1);

	//cr-yoav: check for division by zero: when sin_phi == 0.0
	rvec[0] = (R[2][1] - R[1][2]) / (2.0 * sin_phi);
	rvec[1] = (R[0][2] - R[2][0]) / (2.0 * sin_phi);
	rvec[2] = (R[1][0] - R[0][1]) / (2.0 * sin_phi);

// TODO: should we check cos_phi != 0 ?
	*angle = atan2(sin_phi, cos_phi);

	//norm = sqrt(SQR(rvec[0]) + SQR(rvec[1]) + SQR(rvec[2]));

	// TODO: do we need to normalize rvec ?
	//RTDEBUG(MSG_INFO, "c_rotation_vector: %5.2f %5.2f %5.2f, angle = %f", rvec[0], rvec[1], rvec[2], *angle);
}
