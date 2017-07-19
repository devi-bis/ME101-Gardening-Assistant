#include "NXTServo-lib-UW.c"

void CRServoOFF()
{
	setServoSpeed(S1, 1, 25);										//Unconfigured setServoSpeed function to avoid creep at deadband boundary, manually set speed to center of deadband to ensure 0 creep
}

void CRServo(int CRDir)												//Rotate Continuos Servo (1 for CW, -1 for CCW)
{
	setServoSpeed(S1, 1, 25*CRDir, 13, 39);			//Configured to account for servo deadband (13 - 39)
}

void DrillVert(float Dist, int Dir)			//Moves Drill by user specified distance (in mm), in user specified direction (1 for down, -1 for up)
{
	nMotorEncoder[motorB] = 0;
	const float winchDiam = 23.0;					//Diameter of motorB "winch" in mm

	while (abs(nMotorEncoder[motorB]) < (Dist/4.0)*220.0){
		motor[motorB] = 10*Dir;
	}

	motor[motorB] = 0;
}

void LineFollow(bool Terminate)					//Follows black line until colour senor detects colour other than Bl/W, adjusting motor speeds to keep robot on track
{																				//Program designed such that black line is on the LEFT side of intensity sensor, white on the RIGHT. Sensor placed dead CENTER during setup
	float AdjustFactor = 0.0;
	const float left = 35.0;							//Uses const vars for ease of calibration - ability to tweak adjust factor without editing the whole function
	const float leftDiv = 10.0;
	const float right = 40.0;
	const float rightDiv = 10.0;


	while (Terminate && SensorValue[S2] != 5){		//If terminate function has been activated, robot follows line ignoring colours, until it sees red, where it stops

		if (SensorValue[S3] >= left && SensorValue[S3] <= right){
			motor[motorA] = 50;
			motor[motorC] = 50;
		}

		else if (SensorValue[S3] < left){
			AdjustFactor = (left - SensorValue[S3])/leftDiv;
			motor[motorA] = 50;
			motor[motorC] = 50 - AdjustFactor*50;
		}

		else if (SensorValue[S3] > right){
			AdjustFactor = (SensorValue[S3] - right)/rightDiv;
			motor[motorC] = 50;
			motor[motorA] = 50 - AdjustFactor*50;
		}
	}

	while (SensorValue[S2] == 1 || SensorValue[S2] == 6){				//If terminate function has not been activated, follow line until colour sensor detects a colour

		if (SensorValue[S3] >= left && SensorValue[S3] <= right){
			motor[motorA] = 50;
			motor[motorC] = 50;
		}

		else if (SensorValue[S3] < left){
			AdjustFactor = (left - SensorValue[S3])/leftDiv;
			motor[motorA] = 50;
			motor[motorC] = 50 - AdjustFactor*50;
		}

		else if (SensorValue[S3] > right){
			AdjustFactor = (SensorValue[S3] - right)/rightDiv;
			motor[motorC] = 50;
			motor[motorA] = 50 - AdjustFactor*50;
		}
	}

	motor[motorA] = 0;
	motor[motorC] = 0;
}

void Terminate()												//Initiates program termination, returns robot to start location
{
	motor[motorA] = -20;									//Manually bypasses seeing black line on turn
	motor[motorC] = 20;
	wait1Msec(1000);

	while (SensorValue[S3] > 38){}				//Turns around until intensity sensor reading is low (into the black line)
	motor[motorA] = 0;
	motor[motorC] = 0;


	LineFollow(true);
}

void Drill(float drillDist)
{
	CRServo(1);																	//Drill CCW

	DrillVert(drillDist, -1);										//Pulls drill down on winch

	CRServo(-1);																//Drill CW

	DrillVert(drillDist, 1);										//Returns drill on winch

	CRServoOFF();																//Drill OFF

	wait1Msec(1000);

}

bool Dig()
{
	int config = SensorValue[S2];
	const float turnDeg = 520.0;					//Degrees one motor encoder needs to turn for a 90 degree turn - determined empirically
	const float clearance  = 0.0;					//Drill bit clearance from the ground, in mm
	const float troughDist = 100.0;				//Distance from dirt trough after turning, in mm
	const float wheelDiam = 30.0;
	float drillDist = 0.0;

	if (config == 5){
		Terminate();
		return false;
	}

	nMotorEncoder[motorA] = 0;						//Turn
	motor[motorA] = 25;
	motor[motorC] = -25;
	while (nMotorEncoder[motorA] < turnDeg){}

	motor[motorA] = 0;
	motor[motorC] = 0;

	nMotorEncoder[motorA] = 0;						//Move Forward
	motor[motorA] = 50;
	motor[motorC] = 50;
	while (nMotorEncoder[motorA] < (troughDist/(PI*wheelDiam))*360.0){}

	motor[motorA] = 0;
	motor[motorC] = 0;

//	drillDist = (config + 1.0)*9.0 + clearance;

	Drill(config);

	nMotorEncoder[motorA] = 0;						//Move Back
	motor[motorA] = -50;
	motor[motorC] = -50;
	while (nMotorEncoder[motorA] > -1.0*(troughDist/(PI*wheelDiam))*360.0){}

	motor[motorA] = 0;
	motor[motorC] = 0;

	nMotorEncoder[motorA] = 0;						//Turn Back
	motor[motorA] = -20;
	motor[motorC] = 20;
	wait1Msec(2500);
	while (SensorValue[S3] > 37){}				//Lock itself back onto the black line

	motor[motorA] = 50;										//Manually bypass seeing same dig colour again
	motor[motorC] = 50;
	wait1Msec(500);
	motor[motorA] = 0;
	motor[motorC] = 0;

	return true;

}

task main()
{
	SensorType[S1] = sensorI2CCustom9V;				//Declare Tetrix Servo Controller Port
	SensorType[S2] = sensorColorNxtFULL;			//Declare	Sensor Ports
	SensorType[S3] = sensorLightActive;				//Intensity Sensor (for line following)

	bool runProgram = true;
	int holesDug = 0;

	displayString(0, "Press and release");		//Start program on orange button press and release
	displayString(1, "orange button to");
	displayString(2, "start program.");
	while(nNxtButtonPressed != 3){}
	while(nNxtButtonPressed != -1){}
	eraseDisplay();
	time1[0] = 0;															//Used to track elapsed time

	motor[motorA] = 50;												//Ignore terminate colour at start point
	motor[motorC] = 50;
	wait1Msec(250);

	while (runProgram){												//Run robot functions while terminate colour is not seen

		eraseDisplay();
		displayString(1, "Holes Dug: %d", holesDug);

		LineFollow(false);

		runProgram = Dig();

		holesDug += runProgram;

	}

	float time = time1[0]/1000.0;							//Convert elapsed time to seconds and display
	displayString(2, "Elapsed Time:");
	displayString(3, "%0.2f s", time);

	displayString(4, "Digging Complete!");		//End program on orange button press and release
	displayString(5, "Press and release");
	displayString(6, "orange button to");
	displayString(7, "end program.");
	while(nNxtButtonPressed != 3){}
	while(nNxtButtonPressed != -1){}
}
