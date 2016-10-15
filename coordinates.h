#ifndef COORD_H
#define COORD_H

/******************************************************************
 * gimbal_position: Returns the position x, y, z of the end of    *
 *                  the stylus in the Omni coordinate system.     *
 ******************************************************************/
double * gimbal_position(double *omniPos, double *thetas, double *gimbalAngles);

 #endif