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

	while (abs(nMotorEncoder[motorB]) < (Dist/(PI*winchDiam))*360.0){
		motor[motorB] = 80*Dir;
	}

	motor[motorB] = 0;
}

void LineFollow(bool Terminate)					//Follows black line until colour senor detects colour other than Bl/W, adjusting motor speeds to keep robot on track
{																				//Program designed such that black line is on the LEFT side of intensity sensor, white on the RIGHT. Sensor placed dead CENTER during setup
	float AdjustFactor = 0.0;
	const float left = 36.0;							//Uses const vars for ease of calibration - ability to tweak adjust factor without editing the whole function
	const float leftDiv = 30.0;
	const float right = 48.0;
	const float rightDiv = 56.0;

	while (Terminate && SensorValue[S2] != 5){		//If terminate function has been activated, robot follows line backwards, ignoring colours, until it sees red, where it stops

		if (SensorValue[S3] >= left && SensorValue[S3] <= right){
			motor[motorA] = -50;
			motor[motorC] = -50;
		}

		else if (SensorValue[S3] < left){
			AdjustFactor = (left - SensorValue[S3])/leftDiv;
			motor[motorA] = -50;
			motor[motorC] = -50 + AdjustFactor*50;
		}

		else if (SensorValue[S3] > right){
			AdjustFactor = (SensorValue[S3] - right)/rightDiv;
			motor[motorC] = -50;
			motor[motorA] = -50 + AdjustFactor*50;
		}
	}

	while (SensorValue[S2] == 1 || SensorValue[S2] == 6){

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
	motor[motorA] = -50;									//Manually bypass seeing terminate colour again
	motor[motorC] = -50;
	wait1Msec(500);

	LineFollow(true);
}

bool Dig()
{
	int config = SensorValue[S2];
	const float turnDeg = 1440.0;					//Degrees one motor encoder needs to turn for a 90 degree turn - determined empirically
	const float clearance  = 40.0;				//Drill bit clearance from the ground, in mm
	const float troughDist = 100.0;				//Distance from dirt trough after turning, in mm
	const float wheelDiam = 30.0;
	float drillDist = 0.0;

	if (config == 5){
		Terminate();
		return false;
	}

	nMotorEncoder[motorA] = 0;						//Turn
	motor[motorA] = 50;
	motor[motorC] = -50;
	while (nMotorEncoder[motorA] < turnDeg){}

	motor[motorA] = 0;
	motor[motorC] = 0;

	nMotorEncoder[motorA] = 0;						//Move Forward
	motor[motorA] = 50;
	motor[motorB] = 50;
	while (nMotorEncoder[motorA] < (troughDist/(PI*wheelDiam))*360.0){}

	motor[motorA] = 0;
	motor[motorC] = 0;

	CRServo(-1);																	//Drill CCW

	drillDist = (config + 1)*10 + clearance;			//Digs 3cm deep hole for Blue, 4cm for Green, 5cm for Yellow
	DrillVert(drillDist, 1);

	CRServo(1);																		//Drill CW

	DrillVert(drillDist, -1);											//Returns drill on winch

	CRServoOFF();																	//Drill OFF

	nMotorEncoder[motorA] = 0;						//Move Back
	motor[motorA] = -50;
	motor[motorB] = -50;
	while (nMotorEncoder[motorA] > -1.0*(troughDist/(PI*wheelDiam))*360.0){}

	motor[motorA] = 0;
	motor[motorC] = 0;

	nMotorEncoder[motorA] = 0;						//Turn Back
	motor[motorA] = -50;
	motor[motorC] = 50;
	while (nMotorEncoder[motorA] > -1.0*turnDeg){}

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

	motor[motorA] = 50;												//Ignore terminate colour at start point
	motor[motorC] = 50;
	wait1Msec(500);

	while (runProgram){												//Run robot functions while terminate colour is not seen

		eraseDisplay();
		displayString(1, "Holes Dug: %d", holesDug);

		LineFollow(false);

		runProgram = Dig();

		holesDug += runProgram;

	}

	displayString(3, "Digging Complete!");		//End program on orange button press and release
	displayString(4, "Press and release");
	displayString(5, "orange button to");
	displayString(6, "end program.");
	while(nNxtButtonPressed != 3){}
	while(nNxtButtonPressed != -1){}
}
