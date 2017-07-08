#include "NXTServo-lib-UW.c"


task main()
{
	SensorType[S1] = sensorI2CCustom9V;

	//NOTE: Motor directions are determined by looking down the shaft, TOWARDS the motor


	while(1){														//Continuous loop

	setServoSpeed(S1, 1, -25, 13, 39);	//Run the motor @ speed 25 past the deadband, Clockwise
	while (nNxtButtonPressed != 3){}		//Press & release orange button to switch speed
	while (nNxtButtonPressed != -1){}

	setServoSpeed(S1, 1, 25, 13, 39);		//Run the motor @ speed 25 past the deadband, Counter-Clockwise
	while (nNxtButtonPressed != 3){}		//Press & release orange button to switch speed
	while (nNxtButtonPressed != -1){}

	setServoSpeed(S1, 1, 25);						//Stop motor (uses unconfigured setspeed func. to avoid creeping on the deadband boundary, manually sets speed in the middle of deadband)
	while (nNxtButtonPressed != 3){}		//Press & release orange button to switch speed
	while (nNxtButtonPressed != -1){}

	}
}
