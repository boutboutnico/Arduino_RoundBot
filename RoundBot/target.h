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
 * @file 	target.h
 * @brief	xx
 * @author	boutboutnico
 * @date	26 juil. 2012
 * @company	Itron
 * @site	Chasseneuil
 * @product	xx
 * @module	xx
 *****************************************************************************/

#ifndef TARGET_H_
#define TARGET_H_

/*****************************************************************************
 * DEFINITION
 *****************************************************************************/

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
 * @date	26 juil. 2012
 */
class Target
{
public:
	Target();
	ST_traj computeToXY(ST_pos& st_pos, float x_target, float y_target);

private:
	float target_distance;
	float heading_error_rad;

};

#endif /* TARGET_H_ */
/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
