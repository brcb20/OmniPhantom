#include <stdio.h>
#include <math.h>
#include "coordinates.h"

#define PI 3.14159265

/***************************************************************
 * new_coord: Returns x,y,z in a new coordinate sys located at *
 *            base of the second joint - x is now towards the  *
 *			  user, y to the right if facing the Omni and z    *
 *            is upwards.                                      *
 **************************************************************/
double * new_coord(double *thetas)
{
	double L1 = 133.35, L2 = L1; //mm
	double theta1 = -thetas[0], theta2 = thetas[1];
	double sigma = -(PI/2 + theta2 - thetas[2]);
	double c1 = cos (theta1), c2 = cos (theta2), c3 = cos (sigma);
	double s1 = sin (theta1), s2 = sin (theta2), s3 = sin (sigma);
	static double t[3];

	t[0] = (c1*c2*c3 - c1*s2*s3)*L2 + c1*c2*L1;
	t[1] = (s1*c2*c3 - s1*s2*s3)*L2 + s1*c2*L1;
	t[2] = (s2*c3 + c2*s3)*L2 + s2*L1;

	return t;
}

/***********************************************************************
 * new_coord_withGimbal: Returns x,y,z in same coordinate sys as Omni  *
 *						 but for the true end-effector, i.e the tip of *
 *                       the stylus. Stylus measurements taken experi- *
 *						 mentally.                                     *
 ***********************************************************************/
double * new_coord_withGimbal(double *thetas, double *gimbals)
{
	/* Lengths */
	double L1 = 133.35, L2 = L1, L3 = 42.0; //mm
	/* Angles */
	double theta1 = -thetas[0], theta2 = thetas[1], theta3 = thetas[2];
	double sigma = (theta3 - theta2);
	double theta4 = -gimbals[0], theta5 = gimbals[1], theta6 = gimbals[2];
	/* Cos & Sin of angles */
	double c1 = cos (theta1), c2 = cos (theta2), c3 = cos (sigma), c4 = cos (theta4), c5 = sin (theta5), c23 = cos(theta2 + sigma);
	double s1 = sin (theta1), s2 = sin (theta2), s3 = sin (sigma), s4 = sin (theta4), s5 = -cos (theta5), s23 = sin(theta2 + sigma), s23n = sin(theta2 - sigma);
	
	static double tg[3];
	double r[8];

	r[0] = c1*c23*c4 + s1*s4;
	r[1] = -c1*s23;
	r[2] = -c1*c23*s4 + s1*c4;
	r[3] = s1*c23*c4 - c1*s4;
	r[4] = -s1*s23n;
	r[5] = -s1*c23*s4 - c1*c4;
	r[6] = s23*c4;
	r[7] = c23;

	double px = c1*s23*L2 + c1*c2*L1;
	double py = s1*s23*L2 + L1*s1*c2;
	double pz = -c23*L2 + s2*L1;

	double q3 = r[0]*s5 - r[1]*c5;
	double q6 = r[3]*s5 - r[4]*c5;
	double q9 = r[6]*s5 - r[7]*c5;

	/* Position x, y, z respectively */
	tg[0] = q3*L3 + px;
	tg[1] = q6*L3 + py;
	tg[2] = q9*L3 + pz;

	return tg;
}

double * helper(double *omniPos, double *thetas){

	/* Position of Omni in new coordinate system */
	double *currentPos;
	/* The offset for x, y, z directions */
	static double tempCalibration[3];
	/* Constants */
	int i, j;
	char str[] = {"x" "y" "z"};

	currentPos = new_coord(thetas);
	
	/* Aligning axes & calculating offsets */
	printf("Calibration of axis:\n");
	for (i = 0; i < 3; i++)
	{
		j = (i == 2 ? i-2:i+1);
		tempCalibration[i] = currentPos[j] - omniPos[i];
		printf("\t\t  %c: %f\n", str[i], tempCalibration[i]);
	}

	return tempCalibration;
}

double * gimbal_position(double *omniPos, double *thetas, double *gimbalAngles)
{
	int j;
	/* Stylus end position in Omni coordinate system */
	static double gimbalOmniPos[3];

	/* This calibration is calculated only once during runtime */
	static const double *calibration = helper(omniPos, thetas);

	/* Stylus end position in new coordinate system */
	double *gimbalPos = new_coord_withGimbal(thetas, gimbalAngles);

	for (int i = 0; i < 3; i++)
	{
		j = (i == 2 ? i-2:i+1);
		gimbalOmniPos[i] = gimbalPos[j] - calibration[i];
	}

	return gimbalOmniPos;
}
