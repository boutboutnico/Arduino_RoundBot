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
 * @file 	define.h
 * @brief	xx
 * @author	boutboutnico
 * @date	25 juil. 2012
 * @company
 * @site
 * @product	xx
 * @module	xx
 *****************************************************************************/

#ifndef DEFINE_H_
#define DEFINE_H_

/*****************************************************************************
 * DEFINITION
 *****************************************************************************/

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/

//--------------------------------------------------
//	Arduino Uno R3
//--------------------------------------------------
typedef char int8;
typedef int int16;
typedef long int int32;

typedef unsigned char uint8;
typedef unsigned int uint16;
typedef unsigned long int uint32;

typedef struct
{
	float theta_rad;
	float x;
	float y;
} ST_pos;

typedef struct
{
	float target_distance;
	float heading_error_rad;
} ST_traj;

typedef struct
{
	float l_vel;
	float r_vel;
}ST_vel;

#endif /* DEFINE_H_ */
/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
