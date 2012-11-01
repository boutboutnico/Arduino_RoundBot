/*************************************************************************//**
 * @file 	roundbot.ino
 * @brief	xx
 * @author	Nicolas BOUTIN
 * @date	25 juil. 2012
 * @module	xx
 *****************************************************************************/

// gets rid of annoying "depreciated conversion from string constant warning
#pragma GCC diagnostic ignored "-Wwrite-strings"

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/
#include <Arduino.h>
#include <Wire.h>

// Libraries
#include "lcd_I2C.h"
#include "PID_v1.h"
#include "motor.h"

// Project include
#include "define.h"
#include "encoder.h"
#include "odometry.h"
#include "target.h"

/*****************************************************************************
 * CONSTANTS
 *****************************************************************************/
static const float f_kp = 3.75;
static const float f_ki = 0.75;
static const float f_kd = 0.025;

/*****************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************/

//--------------------------------------------------
// LCD
//--------------------------------------------------
LCD_I2C lcd(0x00, 4, 20);

//--------------------------------------------------
// Motors & Encoders
//--------------------------------------------------

// dir_pin / cmd_pin / dir / cmd
Motor r_mot(4, 5, R_FORWARD, 0);
Motor l_mot(7, 6, L_FORWARD, 0);

// pin_A / int_pin_A / pin_B / int_func / is_CCW
Encoder r_enc(3, 1, 9, doRightEncoder, true);
Encoder l_enc(2, 0, 8, doLeftEncoder, false);

//--------------------------------------------------
// PID
//--------------------------------------------------
// min = 5 (less no tested)
// max = 60 (pwm = 255)
float l_consigne = 50; //cm/sec
float r_consigne = 50; //cm/sec

float l_cmd = 0;
float r_cmd = 0;
ST_vel st_vel;

PID l_pid(l_consigne, st_vel.l_vel, l_cmd, f_kp, f_ki, f_kd);
PID r_pid(r_consigne, st_vel.r_vel, r_cmd, f_kp, f_ki, f_kd);

//--------------------------------------------------
// Others
//--------------------------------------------------
Odometry odo;
Target target;

ST_pos st_pos;

/*****************************************************************************
 * SETUP
 *****************************************************************************/
void setup()
{
	//--------------------------------------------------
	// Right motor
	//--------------------------------------------------
//	pinMode(st_r_mot.dir_pin, OUTPUT);
//	pinMode(st_r_mot.cmd_pin, OUTPUT);
//	digitalWrite(st_r_mot.dir_pin, R_FORWARD);
//	analogWrite(st_r_mot.cmd_pin, 0);

	r_mot.begin();
	r_enc.begin();

	//--------------------------------------------------
	// Left motor
	//--------------------------------------------------
//	pinMode(st_l_mot.dir_pin, OUTPUT);
//	pinMode(st_l_mot.cmd_pin, OUTPUT);
//	digitalWrite(st_l_mot.dir_pin, L_FORWARD);
//	analogWrite(st_l_mot.cmd_pin, 0);

	l_mot.begin();
	l_enc.begin();

	//--------------------------------------------------
	// LCD
	//--------------------------------------------------
	lcd.begin();
	lcd.cursor(false);
	lcd.clear();
	lcd.backlight(true);

	//--------------------------------------------------
	// Serial
	//--------------------------------------------------
	Serial.begin(9600);

	//--------------------------------------------------
	// Debug
	//--------------------------------------------------
//	Serial.println("time\tcons\tvel\tcmd");
}

/*****************************************************************************
 * LOOP
 *****************************************************************************/
void loop()
{
	odo.compute(l_enc.getCount(), r_enc.getCount());
	st_vel = odo.getVel();

	l_pid.Compute();
	r_pid.Compute();

	l_mot.command(l_cmd);
	r_mot.command(r_cmd);

	//--------------------------------------------------
	//	Print Serial
	//--------------------------------------------------
	static uint32 serialTime = 0;
	if(millis() > serialTime)
	{
		Serial.print(serialTime);
		Serial.print("\t");
		Serial.print(l_consigne);
		Serial.print("\t");
		Serial.print(st_vel.l_vel);
		Serial.print("\t");
		Serial.println(l_cmd);

		serialTime += 50;
	}

	//--------------------------------------------------
	//	Print LCD
	//--------------------------------------------------
	static uint32 lcdTime = 0;
	if(millis() > lcdTime)
	{
		lcdTime += 200;

		lcd.printf(0, 0, "l_vel:%d", (uint32_t) st_vel.l_vel);
		lcd.printf(0, 1, "l_cmd:%d", (uint32_t) l_cmd);
		lcd.printf(0, 2, "l_con:%d", (uint32_t) l_consigne);

		lcd.printf(11, 0, "r_vel:%d", (uint32_t) st_vel.r_vel);
		lcd.printf(11, 1, "r_cmd:%d", (uint32_t) r_cmd);
		lcd.printf(11, 2, "r_con:%d", (uint32_t) r_consigne);

		lcd.printf(0, 3, "Kp:%d", (uint32_t) l_pid.GetKp());
		lcd.printf(6, 3, "Ki:%d", (uint32_t) l_pid.GetKi());
		lcd.printf(12, 3, "Kd:%d", (uint32_t) l_pid.GetKd());
	}

	delay(50); //50ms
}

//--------------------------------------------------
//	Interrupt
//--------------------------------------------------

void doRightEncoder()
{
	r_enc.update();
}

void doLeftEncoder()
{
	l_enc.update();
}

//--------------------------------------------------
//	test
//--------------------------------------------------

//--------------------------------------------------
//	debug
//--------------------------------------------------

/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
