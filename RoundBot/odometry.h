/*************************************************************************//**
 * @file 	odometry.h
 * @brief	xx
 * @author	Nicolas BOUTIN
 * @date	25 juil. 2012
 * @module	xx
 *****************************************************************************/

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

/*****************************************************************************
 * DEFINITION
 *****************************************************************************/
#define WHEEL_BASE 27.75 //mm
#define LEFT_CLICKS_PER_CM 14.40
#define RIGHT_CLICKS_PER_CM 14.30

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/

#include "define.h"
/*****************************************************************************
 * CLASS
 *****************************************************************************/
/**
 * @class 	xx
 * @brief	xx
 * @author	boutboutnico
 * @date	25 juil. 2012
 */
class Odometry
{
public:

	Odometry();
	void compute(int32_t l_enc, int32_t r_enc);
	ST_pos getPos();
	ST_vel getVel();

	float l_cumul;

private:

	float theta_rad; /* bot heading */
	float x_pos; /* bot X position in cm */
	float y_pos; /* bot Y position in cm */

	float r_vel;	// cm/sec
	float l_vel;	// cm/sec

	int32_t l_last_enc;
	int32_t r_last_enc;

};

#endif /* ODOMETRY_H_ */
/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
