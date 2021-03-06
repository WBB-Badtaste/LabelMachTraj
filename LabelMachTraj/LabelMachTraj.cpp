// LabelMachTraj.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"

#include "Interface.h"
#include "AxisControl.h"

#include <Windows.h>

#define SIM_MOD

#define NUM_AXES 2
const char *axName[ NUM_AXES ] = { "DEF_AXIS_4", "DEF_AXIS_5" };
SAC_AXIS axId[NUM_AXES];

using namespace std;

void shutTest()
{
	printf("shut!\n");
}

int _tmain(int argc, _TCHAR* argv[])
{
	NYCE_STATUS nyceStatus(NYCE_OK);

#ifdef SIM_MOD
	nyceStatus = NyceInit(NYCE_SIM);
#else:
	nyceStatus = NyceInit(NYCE_ETH);
#endif

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES, axId, axName);

	LABEL_MECH_INIT_PARS initPars;
	initPars.axId[0] = axId[0];
	initPars.axId[1] = axId[1];
	initPars.nozzleRelPos1[0] = -10.0;
	initPars.nozzleRelPos1[1] = -0.1;
	initPars.nozzleRelPos2[0] = 10.0;
	initPars.nozzleRelPos2[1] = 0.1;
	initPars.splineTime = 0.005;
	initPars.pShut = shutTest;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : LabelMechInit(initPars);

	LABEL_MECH_MOTION_PARS motionPars;
	motionPars.endPos[0] = 240.0;
	motionPars.endPos[1] = 620.0;
	motionPars.cameraPos[0] = 245.0;
	motionPars.cameraPos[1] = 220.0;
	motionPars.cameraVel = 300.0;
	motionPars.maxVel  = 300.0;
	motionPars.maxAcc  = 30000.0;
	motionPars.maxJerk = 3000000.0;
	motionPars.shutterDelay = 0.1;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : LabelMechMoveOptTrajectory(motionPars);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : LabelMechTerm();

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : TermAxis(NUM_AXES, axId);

	printf(LabelMechGetStatusString(nyceStatus));

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();

	system("pause");
	return 0;
}

