#include "NXTServo-lib-UW.c"

void CRServoCW(int CRCWdur)				//Rotate Continuos Servo Clockwise for specified duration in ms
{
	setServoSpeed(S1, 1, 25, 13, 39);		//Configured to account for servo deadband (13 - 39)
	wait1Msec(CRCWdur);				//Turn CW for user specified duration
	setServoSpeed(S1, 1, 25);			//Unconfigured setServoSpeed function to avoid creep at deadband boundary, manually set speed to center of deadband to ensure 0 creep
}

void CRServoCCW(int CRCCWdur)				//Rotate Continuos Servo Counter-Clockwise for specified duration in ms
{
	setServoSpeed(S1, 1, -25, 13, 39);		//Configured to account for servo deadband (13 - 39)
	wait1Msec(CRCCWdur);				//Turn CCW for user specified duration
	setServoSpeed(S1, 1, 25);			//Unconfigured setServoSpeed function to avoid creep at deadband boundary, manually set speed to center of deadband to ensure 0 creep
}

void DrillDown(int Dist, bool Dir)			//Moves Drill by user specified distance (in mm), in user specified direction (true for down, false for up)
{
	nMotorEncoder[motorB] = 0;
	if (Dir){
		while (nMotorEncoder[motorB] < (Dist/(PI*23))*360){motor[motorB] = 80;}
	}
	else {
		while (nMotorEncoder[motorB] > -(Dist/(PI*23))*360){motor[motorB] = -80;}
	}
	motor[motorB] = 0;
}

task main()
{
	SensorType[S1] = sensorI2CCustom9V;		//Declare Tetrix Servo Controller Port
	SensorType[S2] = sensorColorNxtFULL;		//Colour Sensor
	SensorType[S3] = sensorLightActive;		//Intensity Sensor (for line following)



}
