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
 * @file 	target.cpp
 * @brief	xx
 * @author	boutboutnico
 * @date	26 juil. 2012
 * @company	Itron
 * @site	Chasseneuil
 * @product	xx
 * @module	xx
 *****************************************************************************/

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/
#include <Arduino.h>
#include "target.h"
/*****************************************************************************
 * NAMESPACE
 *****************************************************************************/

/*****************************************************************************
 * GLOBALE VARIABLE
 *****************************************************************************/

/*****************************************************************************
 * PUBLIC IMPLEMENTATION
 *****************************************************************************/
Target::Target()
{
	target_distance = 0.0;
	heading_error_rad = 0.0;
}

ST_traj Target::computeToXY(ST_pos& st_pos, float x_target, float y_target)
{
	float x = x_target - st_pos.x;
	float y = y_target - st_pos.y;
	float target_bearing_rad = 0.0;

	target_distance = sqrt((x * x) + (y * y));

	/* no divide-by-zero allowed! */
	if(x > 0.00001) target_bearing_rad = HALF_PI - atan(y / x);
	else if(x < -0.00001) target_bearing_rad = -HALF_PI - atan(y / x);

	heading_error_rad = target_bearing_rad - (st_pos.theta_rad);// * RAD_TO_DEG);

	if(heading_error_rad > PI) heading_error_rad -= TWO_PI;
	else if(heading_error_rad < -PI) heading_error_rad += TWO_PI;

	ST_traj st_traj = {target_distance, heading_error_rad};
	return st_traj;
}
/*****************************************************************************
 * PRIVATE IMPLEMENTATION
 *****************************************************************************/

/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
