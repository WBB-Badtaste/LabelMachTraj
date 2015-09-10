#pragma once

#include "LabelMachDefs.h"
#include "Trajectory.h"
#include "Streamer.h"
#include "Structs.h"

LABEL_MECH_PARS mechPars;

NYCE_STATUS LabelMechInit(const LABEL_MECH_INIT_PARS &initPars)
{
	if (mechPars.haveInited)
		return LABEL_MACHINE_ERR_REINIT;

	mechPars.baseSplineTime = 0.01;
	mechPars.shutOffest = 0.01;
	mechPars.perSendSplineNum = NYCE_MAX_NR_OF_SPLINE_SEGMENTS_WRITE;
	mechPars.lessSplineTime = 0.0003;

	mechPars.axId[0] = initPars.axId[0];
	mechPars.axId[1] = initPars.axId[1];

	mechPars.cameraPos[0] = initPars.cameraPos[0];
	mechPars.cameraPos[1] = initPars.cameraPos[1];

	mechPars.nozzleInterval = initPars.nozzleInterval;
	if (mechPars.nozzleDeflection > 90 || mechPars.nozzleDeflection < 0)
		return LABEL_MACHINE_ERR_NOZZLE_DEFLECTION_OUT_OF_RANGE;
	mechPars.nozzleDeflection = initPars.nozzleDeflection / 180 * M_PI;

	mechPars.pShut = initPars.pShut;

	NYCE_STATUS nyceStatus(MacDefineSyncGroup(mechPars.axId, 2, &mechPars.groupId));

	if (NyceSuccess(nyceStatus))
		nyceStatus = SacAddVariableToSet(mechPars.axId[0], SAC_VAR_SETPOINT_POS, &mechPars.varId[0]);

	if (NyceSuccess(nyceStatus))
		nyceStatus = SacAddVariableToSet(mechPars.axId[1], SAC_VAR_SETPOINT_POS, &mechPars.varId[1]);

	if (NyceSuccess(nyceStatus))
		mechPars.haveInited = TRUE;

	return nyceStatus;
}

NYCE_STATUS LabelMechTerm()
{
	if (!mechPars.haveInited)
		return LABEL_MACHINE_ERR_HAVE_NOT_INITED;

	NYCE_STATUS nyceStatus(MacDeleteSyncGroup(mechPars.groupId));

	if (NyceSuccess(nyceStatus))
		nyceStatus = NyceDeleteVariableFromSet(mechPars.varId[0]);

	if (NyceSuccess(nyceStatus))
		nyceStatus = NyceDeleteVariableFromSet(mechPars.varId[1]);

	if (NyceSuccess(nyceStatus))
		mechPars.haveInited = FALSE;

	return nyceStatus;
}

NYCE_STATUS LabelMechMoveOptTrajectory(const LABEL_MECH_MOTION_PARS &motionPars)
{
	if (!mechPars.haveInited)
		return LABEL_MACHINE_ERR_HAVE_NOT_INITED;

	double currentPos[2];
	NYCE_STATUS nyceStatus(NyceReadVariableSet(mechPars.varId[0], mechPars.varId[1],currentPos));

	if (NyceSuccess(nyceStatus))
	{
		const double angle(currentPos[0] < mechPars.cameraPos[0] ? mechPars.nozzleDeflection : -M_PI - mechPars.nozzleDeflection);
		const double cos_angle(cos(angle));
		const double sin_angle(sin(angle));

		double shutterTime = motionPars.shutterDelay + mechPars.shutOffest;
		double distanceOffset(motionPars.cameraVel * shutterTime);
		double tcpOffset(0.5 * mechPars.nozzleInterval);

		const double maxVel(sqrt(motionPars.maxVel[0] * motionPars.maxVel[0] + motionPars.maxVel[1] * motionPars.maxVel[1]));
		const double maxAcc(sqrt(motionPars.maxAcc[0] * motionPars.maxAcc[0] + motionPars.maxAcc[1] * motionPars.maxAcc[1]));
		const double maxJerk(sqrt(motionPars.maxJerk[0] * motionPars.maxJerk[0] + motionPars.maxJerk[1] * motionPars.maxJerk[1]));

		double shutDistance(distanceOffset + tcpOffset);
		mechPars.firstShutPos[0] = mechPars.cameraPos[0] - shutDistance * cos_angle;
		mechPars.firstShutPos[1] = mechPars.cameraPos[1] - shutDistance * sin_angle;

		shutDistance = distanceOffset - tcpOffset;
		mechPars.SecondShutPos[0] = mechPars.cameraPos[0] - shutDistance * cos_angle;
		mechPars.SecondShutPos[1] = mechPars.cameraPos[1] - shutDistance * sin_angle;

		mechPars.finishShutPos[0] = mechPars.cameraPos[0] + tcpOffset * cos_angle;
		mechPars.finishShutPos[1] = mechPars.cameraPos[1] + tcpOffset * sin_angle;

// 		TRAJ_SEG_START_PARS startPars;
// 		startPars.startPos[0] = currentPos[0];
// 		startPars.startPos[1] = currentPos[1];
// 		startPars.splineTime = motionPars.splineTime;
// 
// 		nyceStatus = TarjSegmentStart(startPars, mechPars);
// 
// 		TRAJ_SEG_CURVE_PARS curvePars1;
// 		curvePars1.startPos[0] = currentPos[0];
// 		curvePars1.startPos[1] = currentPos[1];
// 		curvePars1.startVel[0] = 0.0;
// 		curvePars1.startVel[1] = 0.0;
// 		curvePars1.endPos[0] = mechPars.firstShutPos[0];
// 		curvePars1.endPos[1] = mechPars.firstShutPos[1];
// 		curvePars1.endVel[0] = motionPars.cameraVel * cos_angle;
// 		curvePars1.endVel[1] = motionPars.cameraVel * sin_angle;
// 		curvePars1.maxVel[0] = motionPars.maxVel[0];
// 		curvePars1.maxVel[1] = motionPars.maxVel[1];
// 		curvePars1.maxAcc[0] = motionPars.maxAcc[0];
// 		curvePars1.maxAcc[1] = motionPars.maxAcc[1];
// 		curvePars1.maxJerk[0] = motionPars.maxJerk[0];
// 		curvePars1.maxJerk[1] = motionPars.maxJerk[1];
// 		
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicCurve(curvePars1, mechPars, TRUE);
// 
// 		TRAJ_SEG_LINE_PARS linePars1;
// 		linePars1.startPos[0] = curvePars1.endPos[0];
// 		linePars1.startPos[1] = curvePars1.endPos[1];
// 		linePars1.endPos[0] = mechPars.SecondShutPos[0];
// 		linePars1.endPos[1] = mechPars.SecondShutPos[1];
// 		linePars1.startVel = motionPars.cameraVel;
// 		linePars1.endVel = motionPars.cameraVel;
// 		linePars1.maxVel = maxVel;
// 		linePars1.maxAcc = maxAcc;
// 		linePars1.maxJerk = maxJerk;
// 		
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentLineCurve(linePars1, mechPars, TRUE);
// 
// 		TRAJ_SEG_LINE_PARS linePars2;
// 		linePars2.startPos[0] = linePars1.endPos[0];
// 		linePars2.startPos[1] = linePars1.endPos[1];
// 		linePars2.endPos[0] = mechPars.finishShutPos[0];
// 		linePars2.endPos[1] = mechPars.finishShutPos[1];
// 		linePars2.startVel = motionPars.cameraVel;
// 		linePars2.endVel = motionPars.cameraVel;
// 		linePars2.maxVel = maxVel;
// 		linePars2.maxAcc = maxAcc;
// 		linePars2.maxJerk = maxJerk;
// 
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentLineCurve(linePars2, mechPars);
// 
// 		TRAJ_SEG_CURVE_PARS curvePars2;
// 		curvePars2.startPos[0] = linePars2.endPos[0];
// 		curvePars2.startPos[1] = linePars2.endPos[1];
// 		curvePars2.startVel[0] = motionPars.cameraVel * cos_angle;
// 		curvePars2.startVel[1] = motionPars.cameraVel * sin_angle;
// 		curvePars2.endPos[0] = motionPars.endPos[0];
// 		curvePars2.endPos[1] = motionPars.endPos[1];
// 		curvePars2.endVel[0] = 0.0;
// 		curvePars2.endVel[0] = 0.0;
// 		curvePars2.maxVel[0] = motionPars.maxVel[0];
// 		curvePars2.maxVel[1] = motionPars.maxVel[1];
// 		curvePars2.maxAcc[0] = motionPars.maxAcc[0];
// 		curvePars2.maxAcc[1] = motionPars.maxAcc[1];
// 		curvePars2.maxJerk[0] = motionPars.maxJerk[0];
// 		curvePars2.maxJerk[1] = motionPars.maxJerk[1];
// 
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicCurve(curvePars2, mechPars);
// 
// 		//stream
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : stream(mechPars);                            
// 
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : streamSync(mechPars, INFINITE);
	}						 

	return nyceStatus;
}