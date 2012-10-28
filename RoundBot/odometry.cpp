//	 Start of the Copyright Notice
//	****************************************************************************
// 	* Copyright Itron SAS.                                                     *
//	* This computer program includes confidential proprietary information and  *
//	* is a trade secret of Itron. All use, disclosures and/or reproduction 	   *
//	* is prohibited unless authorized in writing.          					   *
//	* All Rights Reserved                                                      *
//	****************************************************************************
//	 End of the Copyright Notice

/*************************************************************************//**
 * @file 	odometry.cpp
 * @brief	xx
 * @author	boutboutnico
 * @date	25 juil. 2012
 * @company	Itron
 * @site	Chasseneuil
 * @product	xx
 * @module	xx
 *****************************************************************************/

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/
#include <Arduino.h>
#include "odometry.h"

/*****************************************************************************
 * GLOBALE VARIABLE
 *****************************************************************************/

/*****************************************************************************
 * PUBLIC IMPLEMENTATION
 *****************************************************************************/
Odometry::Odometry()
{
	theta_rad = 0.0;
	x_pos = 0.0;
	y_pos = 0.0;

	l_last_enc = 0;
	r_last_enc = 0;

	l_vel = 0;
	r_vel = 0;

	l_cumul = 0;
}

void Odometry::compute(int32 l_enc, int32 r_enc)
{
	int32 l_samp = 0;
	int32 r_samp = 0;
	int32 l_delta_enc = 0;
	int32 r_delta_enc = 0;

	float left_cm = 0.0;
	float right_cm = 0.0;
	float dist_cm = 0.0;

	static uint32 last_time = 0;

	/* sample the left and right encoder counts as close together */
	/* in time as possible */
	l_samp = l_enc;
	r_samp = r_enc;

	/* determine how many ticks since our last sampling? */
	l_delta_enc = l_samp - l_last_enc;
	r_delta_enc = r_samp - r_last_enc;

	/* and update last sampling for next time */
	l_last_enc = l_samp;
	r_last_enc = r_samp;

	/* convert longs to floats and ticks to millimeter */
	left_cm = (float) (l_delta_enc / LEFT_CLICKS_PER_CM);
	right_cm = (float) (r_delta_enc / RIGHT_CLICKS_PER_CM);

	l_cumul += left_cm;

	/* calculate distance we have traveled since last sampling */
	dist_cm = (left_cm + right_cm) / 2.0;

	/* accumulate total rotation around our center */
	theta_rad += (left_cm - right_cm) / WHEEL_BASE;

	/* and clip the rotation to plus or minus 360 degrees */
	theta_rad -= (float) ((int) (theta_rad / TWO_PI)) * TWO_PI;

	/* now calculate and accumulate our position in millimeter */
	y_pos += (dist_cm * cos(theta_rad));
	x_pos += (dist_cm * sin(theta_rad));

	/*How long since we last calculated*/
	uint32 now = millis();
	float delta_time = (float) (now - last_time);

	// Calcul velocity for left and right
	l_vel = (left_cm * 1000) / delta_time; // cm/sec
	r_vel = (right_cm * 1000) / delta_time; // cm/sec

	// Remember for last time
	last_time = now;

//	Serial.println(l_samp);
//	Serial.println(r_samp);
//	Serial.println(l_delta_enc);
//	Serial.println(r_delta_enc);
//	Serial.println(left_cm, 2);
//	Serial.println(right_cm, 2);
//	Serial.println(dist_cm, 2);
//	Serial.println(theta, 2);
//	Serial.println(x_pos, 2);
//	Serial.println(y_pos, 2);
//	Serial.println();
}

ST_pos Odometry::getPos()
{
	ST_pos st_pos = { theta_rad, x_pos, y_pos };
	return st_pos;
}

ST_vel Odometry::getVel()
{
	ST_vel st_vel = { l_vel, r_vel };
	return st_vel;
}

/*****************************************************************************
 * PRIVATE IMPLEMENTATION
 *****************************************************************************/

/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
