// LabelMachTraj.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"

#include "Control.h"
#include "AxisControl.h"

#include <Windows.h>

#define NUM_AXES 2
const char *axName[ NUM_AXES ] = { "DEF_AXIS_4", "DEF_AXIS_5" };
SAC_AXIS axId[NUM_AXES];

using namespace std;

void shutTest()
{
	printf("shut!/n");
}

int _tmain(int argc, _TCHAR* argv[])
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = NyceInit(NYCE_SIM);
	//nyceStatus = NyceInit(NYCE_ETH);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES, axId, axName);

	LABEL_MECH_INIT_PARS initPars;
	initPars.axId[0] = axId[0];
	initPars.axId[1] = axId[1];
	initPars.cameraPos[0] = 100000.0;
	initPars.cameraPos[1] = 130450.0;
	initPars.nozzleDeflection = 0.0;
	initPars.nozzleInterval = 5000.0;
	initPars.pShut = shutTest;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : LabelMechInit(initPars);

	LABEL_MECH_MOTION_PARS motionPars;
	motionPars.endPos[0] = 230000.0;
	motionPars.endPos[1] = 230000.0;
	motionPars.cameraVel = 1000000.0;
	motionPars.maxVel[0] = 1000000.0;
	motionPars.maxVel[1] = 1000000.0;
	motionPars.maxAcc[0] = 10000000.0;
	motionPars.maxAcc[1] = 10000000.0;
	motionPars.maxJerk[0] = 100000000.0;
	motionPars.maxJerk[1] = 100000000.0;
	motionPars.shutterDelay = 0.1;
	motionPars.splineTime = 0.001;
	
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : LabelMechMoveOptTrajectory(motionPars);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : LabelMechTerm();

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : TermAxis(NUM_AXES, axId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();

	system("pause");
	return 0;
}

