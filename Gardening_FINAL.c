#include "NXTServo-lib-UW.c"									//Tetrix NXT Servo Controller Library

//Program specifies name of developer over each function

//Devidutta Biswabharati
void CRServoOFF()															//Turn Off Tetrix Continuos Servo
{
	setServoSpeed(S1, 1, 25);										//Unconfigured setServoSpeed function to avoid creep at deadband boundary, manually set speed to center of deadband to ensure 0 creep
}

//Devidutta Biswabharati
void CRServo(int CRDir)												//Rotate Tetrix Continuos Servo (1 for CW, -1 for CCW)
{
	setServoSpeed(S1, 1, 25*CRDir, 13, 39);			//Configured to account for servo deadband (13 - 39)
}

//Non Trivial Function 1: Devidutta Biswabharati
void DrillVert(float Dist, int Dir)			//Moves Drill based on colour-coded distance, in user specified direction (1 for down, -1 for up)
{
	nMotorEncoder[motorB] = 0;

	while (abs(nMotorEncoder[motorB]) < (Dist/4.0)*280.0){	//Empirically determined multiplier to optimize drill depths
		motor[motorB] = 10*Dir;
	}

	motor[motorB] = 0;
}

//Non Trivial Function 2: Jusroop Sangha
void LineFollow()												//Follows black line until colour sensor detects colour other than Bl/W, adjusting motor speeds to keep robot on track
{																				//Program designed such that black line is on the LEFT side of intensity sensor, white on the RIGHT.
	float AdjustFactor = 0.0;
	const float left = 38.0;							//Uses const vars for ease of calibration - ability to tweak adjust factor without editing the whole function
	const float leftDiv = 8.0;
	const float right = 43.0;
	const float rightDiv = 8.0;

	while (SensorValue[S2] == 1 || SensorValue[S2] == 6){					//Follow line until colour sensor detects a colour

		if (SensorValue[S3] >= left && SensorValue[S3] <= right){		//Drive straight when intensity sensor is in nominal range
			motor[motorA] = 50;
			motor[motorC] = 50;
		}

		else if (SensorValue[S3] < left){														//Adjust right when intensity sensor reading is low
			AdjustFactor = (left - SensorValue[S3])/leftDiv;
			motor[motorA] = 50;
			motor[motorC] = 50 - AdjustFactor*50;
		}

		else if (SensorValue[S3] > right){													//Adjust left when intensity sensor reading is high
			AdjustFactor = (SensorValue[S3] - right)/rightDiv;
			motor[motorC] = 50;
			motor[motorA] = 50 - AdjustFactor*50;
		}
	}

	motor[motorA] = 0;
	motor[motorC] = 0;
}

//Non Trivial Function 3: Siddharth Kumar
void Terminate()												//Initiates program termination, returns robot to start location
{
	motor[motorA] = -20;									//Blind turn to avoid seeing black line initially
	motor[motorC] = 20;
	wait1Msec(1000);

	while (SensorValue[S3] > 40){}				//Turns around until intensity sensor reading is low (into the black line)
	motor[motorA] = 0;
	motor[motorC] = 0;


	LineFollow();
}

//Non Trivial Function 4: Siddharth Kumar
void Drill(float drillDist)
{
	CRServo(-1);													//Drill CCW

	DrillVert(drillDist, -1);							//Pulls drill down on winch

	wait1Msec(3000);

	CRServo(1);														//Drill CW

	DrillVert(drillDist, 1);							//Returns drill on winch

	CRServoOFF();													//Drill OFF

	wait1Msec(1000);

}

//Non Trivial Function 5: Zulphkar Yalchin
bool Dig()
{
	int config = SensorValue[S2];					//Save colour sensor reading
	const float turnDeg = 520.0;					//Degrees one motor encoder needs to turn for a 90 degree turn - determined empirically
	const float troughDist = 100.0;				//Distance from dirt trough after turning, in mm
	const float wheelDiam = 30.0;

	if (config == 5){											//If colour sensor reads red, initiate program termination (turn robot and return to start location)
		Terminate();
		return false;												//Signal main function to end
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

	Drill(config);												//Perform drilling activities using colour sensor reading configuration

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
	while (SensorValue[S3] > 38){}				//Lock itself back onto the black line

	motor[motorA] = 50;										//Manually bypass seeing same dig colour again
	motor[motorC] = 50;
	wait1Msec(500);
	motor[motorA] = 0;
	motor[motorC] = 0;

	return true;													//Signal main function to continue looping

}

//Non Trivial Function 6: Devidutta Biswabharati
void IOManager(bool StartDisplay, bool EndDisplay, bool ButtonPress)	//Program to handle general Input/Output tasks - to avoid repetition and clean up main program
{
	if (StartDisplay){
		displayString(0, "Press and release");	//Start program on orange button press and release
		displayString(1, "orange button to");
		displayString(2, "start program.");
	}

	if (EndDisplay){
		displayString(4, "Digging Complete!");	//End program on orange button press and release
		displayString(5, "Press and release");
		displayString(6, "orange button to");
		displayString(7, "end program.");
	}

	if (ButtonPress){
		while(nNxtButtonPressed != 3){}					//Waits for orange button press and release
		while(nNxtButtonPressed != -1){}
	}

}

//Main Function: Devidutta Biswabharati
task main()
{
	SensorType[S1] = sensorI2CCustom9V;				//Tetrix Servo Controller
	SensorType[S2] = sensorColorNxtFULL;			//Colour Sensor
	SensorType[S3] = sensorLightActive;				//Intensity Sensor (for line following)

	bool runProgram = true;
	int holesDug = 0;

	IOManager (1, 0, 1);											//Program start display + wait for orange button press and release
	eraseDisplay();

	time1[0] = 0;															//Used to track elapsed time

	motor[motorA] = 50;												//Ignore terminate colour at start point
	motor[motorC] = 50;
	wait1Msec(250);

	while (runProgram){												//Run robot functions while terminate colour is not seen

		eraseDisplay();
		displayString(1, "Holes Dug: %d", holesDug);

		LineFollow();

		runProgram = Dig();

		holesDug += runProgram;									//Track # of holes dug

	}

	float time = time1[0]/1000.0;							//Convert elapsed time to seconds and display
	displayString(2, "Elapsed Time:");
	displayString(3, "%0.2f s", time);

	IOManager(0, 1, 1);												//Program end display + wait for orange button press and release
}
