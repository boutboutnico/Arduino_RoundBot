// gets rid of annoying "depreciated conversion from string constant warning
#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <Arduino.h>
#include <Wire.h>

// Libraries
#include "lcd_I2C.h"
#include "PID_v1.h"

// Project include
#include "define.h"
#include "encoder.h"
#include "odometry.h"
#include "target.h"

LCD_I2C lcd(0x00, 4, 20);
Encoder r_enc(3, 1, 9, doRightEncoder, true);
Encoder l_enc(2, 0, 8, doLeftEncoder, false);
Odometry odo;
Target target;

ST_Motor st_r_mot = { 4, 5, R_FORWARD, 0 };
ST_Motor st_l_mot = { 7, 6, L_FORWARD, 0 };

ST_vel st_vel;
ST_pos st_pos;

float l_cmd = 0;
float r_cmd = 0;
uint32 time = 0;
uint32 time_limit = 40;

// min = 5 (less no tested)
// max = 60 (pwm = 255)
float l_consigne = 0; //cm/sec
float r_consigne = 0; //cm/sec

// Result from Ziegler & Nichols method
//float ku = 4.1;
//float tu = 0.1;

float kp = 3;
float ki = 1;
float kd = 0;

PID l_pid(l_consigne, st_vel.l_vel, l_cmd, kp, ki, kd);
PID r_pid(r_consigne, st_vel.r_vel, r_cmd, 2, 3, 2);

//--------------------------------------------------
//	Setup
//--------------------------------------------------
/**
 *
 */
void setup()
{
	//--------------------------------------------------
	// Right motor
	//--------------------------------------------------
	pinMode(st_r_mot.dir_pin, OUTPUT);
	pinMode(st_r_mot.cmd_pin, OUTPUT);
	digitalWrite(st_r_mot.dir_pin, R_FORWARD);
	analogWrite(st_r_mot.cmd_pin, 0);

	r_enc.begin();

	//--------------------------------------------------
	// Left motor
	//--------------------------------------------------
	pinMode(st_l_mot.dir_pin, OUTPUT);
	pinMode(st_l_mot.cmd_pin, OUTPUT);
	digitalWrite(st_l_mot.dir_pin, L_FORWARD);
	analogWrite(st_l_mot.cmd_pin, 0);

	l_enc.begin();

	//--------------------------------------------------
	// PID
	//--------------------------------------------------
	l_pid.SetMode(AUTOMATIC);
	l_pid.SetOutputLimits(0, 255);

	r_pid.SetMode(AUTOMATIC);
	r_pid.SetOutputLimits(0, 255);

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
}

//--------------------------------------------------
//	Loop
//--------------------------------------------------
void loop()
{
	odo.compute(l_enc.getCount(), r_enc.getCount());
	st_vel = odo.getVel();

	l_pid.Compute();
	r_pid.Compute();

	analogWrite(st_l_mot.cmd_pin, (int) l_cmd);
	analogWrite(st_r_mot.cmd_pin, (int) r_cmd);

	static uint32 motorTime = 5000;
	if(millis() > motorTime)
	{
		motorTime += 5000;

		static uint8_t step = 0;
		switch(step++){
		case 0:
			l_consigne = 50;
			break;
		case 1:
			l_consigne = 0;
			break;
		case 3:
			l_consigne = 10;
			break;
		case 4:
			l_consigne = 0;
//			step = 0;
			break;
		}
	}

	//send-receive with processing if it's time
	static uint32 serialTime = 0;
	if(millis() > serialTime)
	{
		SerialReceive();
		SerialSend();
		serialTime += 500;
	}

	//--------------------------------------------------
	//	Print LCD & serial
	//--------------------------------------------------
	static uint32 lcdTime = 0;
	if(millis() > lcdTime)
	{
		lcdTime += 500;

		lcd.print(0, 0, "l_vel:      ");
		lcd.print(6, 0, st_vel.l_vel);
		lcd.print(0, 1, "l_cmd:      ");
		lcd.print(6, 1, l_cmd);
		lcd.print(0, 2, "l_con:      ");
		lcd.print(6, 2, l_consigne);

		lcd.print(11, 0, "r_vel:      ");
		lcd.print(17, 0, st_vel.r_vel);
		lcd.print(11, 1, "r_cmd:      ");
		lcd.print(17, 1, r_cmd);

		lcd.print(0, 3, "Kp:    ");
		lcd.print(3, 3, l_pid.GetKp());
		lcd.print(6, 3, "Ki:    ");
		lcd.print(9, 3, l_pid.GetKi());
		lcd.print(12, 3, "Kd:    ");
		lcd.print(15, 3, l_pid.GetKd());
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
union
{                // This Data structure lets
	byte asBytes[24];    // us take the byte array
	float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array

void SerialReceive()
{

	// read the bytes sent from Processing
	int index = 0;
	byte Auto_Man = -1;
	byte Direct_Reverse = -1;
	while(Serial.available() && index < 26)
	{
		if(index == 0) Auto_Man = Serial.read();
		else if(index == 1) Direct_Reverse = Serial.read();
		else foo.asBytes[index - 2] = Serial.read();
		index++;
	}

	// if the information we got was in the correct format,
	// read it into the system
	if(index == 26 && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
	{
		l_consigne = double(foo.asFloat[0]);
		//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
		//   value of "Input"  in most cases (as
		//   in this one) this is not needed.
		if(Auto_Man == 0)              // * only change the output if we are in
		{                      //   manual mode.  otherwise we'll get an
			l_cmd = double(foo.asFloat[2]); //   output blip, then the controller will
		}                                     //   overwrite.

		double p, i, d;              // * read in and set the controller tunings
		p = double(foo.asFloat[3]);           //
		i = double(foo.asFloat[4]);           //
		d = double(foo.asFloat[5]);           //
		l_pid.SetTunings(p, i, d);            //

		if(Auto_Man == 0) l_pid.SetMode(MANUAL);          // * set the controller mode
		else l_pid.SetMode(AUTOMATIC);             //

//		if(Direct_Reverse == 0) myPID.SetControllerDirection(DIRECT); // * set the controller Direction
//		else myPID.SetControllerDirection(REVERSE);          //
	}
	Serial.flush();            // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
	Serial.print("PID ");
	Serial.print(l_consigne);
	Serial.print(" ");
	Serial.print(st_vel.l_vel);
	Serial.print(" ");
	Serial.print(l_cmd);
	Serial.print(" ");

	Serial.print(l_pid.GetKp());
	Serial.print(" ");
	Serial.print(l_pid.GetKi());
	Serial.print(" ");
	Serial.print(l_pid.GetKd());
	Serial.print(" ");

	if(l_pid.GetMode() == AUTOMATIC) Serial.print("Automatic");
	else Serial.print("Manual");
	Serial.print(" ");

	if(l_pid.GetDirection() == DIRECT) Serial.println("Direct");
	else Serial.println("Reverse");
}
/*****************************************************************************
 * END OF FILE
 *****************************************************************************/
