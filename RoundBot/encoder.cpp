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
 * @file 	encoder.cpp
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
#include "encoder.h"

/*****************************************************************************
 * NAMESPACE
 *****************************************************************************/

/*****************************************************************************
 * GLOBALE VARIABLE
 *****************************************************************************/

/*****************************************************************************
 * PUBLIC IMPLEMENTATION
 *****************************************************************************/

Encoder::Encoder(int8 pin_A, int8 int_pin_A, int8 pin_B, pfunc interrupt, bool isCCW)
		:
		  pin_A(pin_A),
		  int_pin_A(int_pin_A),
		  pin_B(pin_B),
		  interrupt(interrupt),
		  b_CCW(isCCW)
{
	cpt_encoder = 0;
}

void Encoder::begin()
{
	pinMode(pin_A, INPUT);
	digitalWrite(pin_A, HIGH); // turn on pullup resistor

	pinMode(pin_B, INPUT);
	digitalWrite(pin_B, HIGH); // turn on pullup resistor

	attachInterrupt(int_pin_A, interrupt, CHANGE);
}

void Encoder::update()
{
	// found a low-to-high on channel A
	if(digitalRead(pin_A) == HIGH)
	{
		// check channel B to see which way encoder is turning
		if(digitalRead(pin_B) == LOW)
		{
			cpt_encoder++;         // CW
		}
		else
		{
			cpt_encoder--;         // CCW
		}
	}
	// found a high-to-low on channel A
	else
	{
		// check channel B to see which way encoder is turning
		if(digitalRead(pin_B) == LOW)
		{
			cpt_encoder--;          // CCW
		}
		else
		{
			cpt_encoder++;          // CW
		}
	}
}

int32 Encoder::getCount()
{
	if(b_CCW == true)	return cpt_encoder;
	else return -cpt_encoder;
}

/*****************************************************************************
 * PRIVATE IMPLEMENTATION
 *****************************************************************************/

/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
