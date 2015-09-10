#pragma once

#include "nyceapi.h"
#include "sacapi.h"
#include "Structs.h"

#define _USE_MATH_DEFINES
#include <math.h>


NYCE_STATUS TarjSegmentStart(const TRAJ_SEG_START_PARS &startPars, LABEL_MECH_PARS &mechPars)
{
	mechPars.baseSplineTime = startPars.splineTime;

	if (mechPars.cubPars[0])
		delete[] mechPars.cubPars[0];
	if (mechPars.cubPars[1])
		delete[] mechPars.cubPars[1];
	
	mechPars.cubPars[0] = new SAC_CUB_PARS[256];
	mechPars.cubPars[1] = new SAC_CUB_PARS[256];

	if (mechPars.segDistance)
		delete[] mechPars.segDistance;
	if (mechPars.segVelocity)
		delete[] mechPars.segVelocity;		

	mechPars.segDistance = new double[256];
	mechPars.segVelocity = new double[256];

	mechPars.maxNrOfCubPars = 256;
	
	mechPars.cubPars[0][0].position = startPars.startPos[0];
	mechPars.cubPars[1][0].position = startPars.startPos[1];
	mechPars.usedNrOfCubPars = 1;

	return NYCE_OK; 
}

//管理缓冲区
NYCE_STATUS BufferManager(const uint32_t &splineNum, LABEL_MECH_PARS &mechPars)
{
	if (mechPars.usedNrOfCubPars + splineNum > mechPars.maxNrOfCubPars)
	{
		SAC_CUB_PARS *cubsX = new SAC_CUB_PARS[mechPars.usedNrOfCubPars + splineNum + 512]();
		SAC_CUB_PARS *cubsY = new SAC_CUB_PARS[mechPars.usedNrOfCubPars + splineNum + 512]();

		memcpy(cubsX, mechPars.cubPars[0], mechPars.usedNrOfCubPars * sizeof(SAC_CUB_PARS));
		memcpy(cubsY, mechPars.cubPars[1], mechPars.usedNrOfCubPars * sizeof(SAC_CUB_PARS));

		delete[] mechPars.cubPars[0];
		delete[] mechPars.cubPars[1];

		mechPars.cubPars[0] = cubsX;
		mechPars.cubPars[1] = cubsY;

		double *distance = new double[mechPars.usedNrOfCubPars + splineNum + 512]();
		double *velocity = new double[mechPars.usedNrOfCubPars + splineNum + 512]();

		memcpy(distance, mechPars.segDistance, mechPars.usedNrOfCubPars * sizeof(double));
		memcpy(velocity, mechPars.segVelocity, mechPars.usedNrOfCubPars * sizeof(double));

		delete[] mechPars.segDistance;
		delete[] mechPars.segVelocity;

		mechPars.segDistance = distance;
		mechPars.segVelocity = velocity;
	}

	return NYCE_OK;
}

//计算三次方加速度运动参数,不适合考虑方向的情况
NYCE_STATUS CalacCubicMotionBasePars(TRAJ_SEG_PARS *pSegPars)
{
	//计算
	pSegPars->time[0] = pSegPars->distance / (pSegPars->startVel + pSegPars->maxVel) * 0.125;
	pSegPars->time[1] = pSegPars->distance / (pSegPars->endVel + pSegPars->maxVel) * 0.125;

	const double pow_time1_2(pSegPars->time[0] * pSegPars->time[0]);
	const double pow_time1_3(pow_time1_2 * pSegPars->time[0]);
	const double pow_time1_4(pow_time1_3 * pSegPars->time[0]);
	const double pow_time2_2(pSegPars->time[1] * pSegPars->time[1]);
	const double pow_time2_3(pow_time2_2 * pSegPars->time[1]);
	const double pow_time2_4(pow_time2_3 * pSegPars->time[1]);

	pSegPars->crackle[0] = (pSegPars->maxVel - pSegPars->startVel) / pow_time1_4 * 0.125;
	pSegPars->crackle[1] = (pSegPars->endVel - pSegPars->maxVel) / pow_time2_4 * 0.125;

	const double maxJerk0(2 * pSegPars->crackle[0] * pow_time1_2);
	const double maxJerk1(2 * pSegPars->crackle[1] * pow_time2_2);

	const double maxAcc0(2 * pSegPars->crackle[0] * pow_time1_3);
	const double maxAcc1(2 * pSegPars->crackle[1] * pow_time2_3);

	const double maxVel(pSegPars->startVel + 8 * pSegPars->crackle[0] * pow_time1_4);

	//判断极值
	if (maxJerk0 >  pSegPars->maxJerk ||
		maxJerk0 < -pSegPars->maxJerk ||
		maxJerk1 >  pSegPars->maxJerk ||
		maxJerk1 < -pSegPars->maxJerk )
		return LABEL_MACHINE_ERR_TRAJ_MAX_JERK_EXCEEDED;

	if (maxAcc0 >  pSegPars->maxAcc ||
		maxAcc0 < -pSegPars->maxAcc ||
		maxAcc1 >  pSegPars->maxAcc ||
		maxAcc1 < -pSegPars->maxAcc )
		return LABEL_MACHINE_ERR_TRAJ_MAX_ACCELERATION_EXCEEDED;

	if (maxVel >  pSegPars->maxVel ||
		maxVel < -pSegPars->maxVel )
		return LABEL_MACHINE_ERR_TRAJ_MAX_VELOCITY_EXCEEDED;
	
	return NYCE_OK;
}

//计算所有关键点对应的路程
NYCE_STATUS CalacSegDistance(const uint32_t &splineNum, const TRAJ_SEG_PARS* const pSegPars, LABEL_MECH_PARS &mechPars)
{
	double currentTime(0.0);
	double crackle(0.0), snap(0.0), jerk(0.0), acc(0.0), vel(0.0), pos(0.0);

	double pow_splintTime_2(pow(mechPars.baseSplineTime, 2));
	double pow_splintTime_3(pow(mechPars.baseSplineTime, 3));
	double pow_splintTime_4(pow(mechPars.baseSplineTime, 4));
	double pow_splintTime_5(pow(mechPars.baseSplineTime, 5));

	//计算12个奇点的运动参数
	SINGULAR_POINT_PARS singularPointPars[12];
	double time(0.0);
	for (uint32_t i = 0; i < 12; ++i)
	{
		switch(i)
		{
		case 4:
			time += pSegPars->time[0];
		case 0:
		case 2:
			time += pSegPars->time[0];
			singularPointPars[i].crackle =  pSegPars->crackle[0];
			break;
		case 1:
			time += pSegPars->time[0];
		case 3:
		case 5:
			time += pSegPars->time[0];
			singularPointPars[i].crackle = -pSegPars->crackle[0];
			break;
		case 10:
			time += pSegPars->time[1];
		case 6:
		case 8:
			time += pSegPars->time[1];
			singularPointPars[i].crackle =  pSegPars->crackle[1];
			break;
		case 7:
			time += pSegPars->time[1];
		case 9:
		case 11:
			time += pSegPars->time[1];
			singularPointPars[i].crackle = -pSegPars->crackle[2];
			break;
		default:
			break;
		}

		singularPointPars[i].time = time;

		if (i)
		{
			const double segTime(singularPointPars[i].time - singularPointPars[i - 1].time);
			singularPointPars[i].snap = singularPointPars[i - 1].snap + singularPointPars[i - 1].crackle * segTime;
		}
		else
		{
			singularPointPars[i].snap = 0.0;
			singularPointPars[i].jerk = 0.0;
			singularPointPars[i].acceleration = 0.0;
			singularPointPars[i].velocity = 0.0;
			singularPointPars[i].distance = 0.0;
		}
		
	}

// 	uint32_t index(0);
// 	for (; index < splineNum - 1; ++index)
// 	{
// 		currentTime = (index + 1) * mechPars.baseSplineTime;
// 		if (currentTime < timePoint)
// 		{
// 			switch (uint32_t(currentTime / pSegPars->time[0]))
// 			{
// 			case 0:
// 			case 3:
// 			case 5:
// 			case 6:
// 				crackle =  pSegPars->crackle[0];
// 				break;
// 			case 1:
// 			case 2:
// 			case 4:
// 			case 7:
// 				crackle = -pSegPars->crackle[0];
// 				break;
// 			default:
// 				break;
// 			}
// 		}
// 		else
// 		{
// 			switch (uint32_t((currentTime - timePoint) / pSegPars->time[1]))
// 			{
// 			case 0:
// 			case 3:
// 			case 5:
// 			case 6:
// 				crackle =  pSegPars->crackle[1];
// 				break;
// 			case 1:
// 			case 2:
// 			case 4:
// 			case 7:
// 				crackle = -pSegPars->crackle[1];
// 				break;
// 			default:
// 				break;
// 			}
// 		}
// 
// 		pos  += vel		 * mechPars.baseSplineTime + 0.5 * acc		* pow_splintTime_2 +  jerk	  * pow_splintTime_3 / 6 + snap    * pow_splintTime_4 / 24 + crackle * pow_splintTime_5 / 120;
// 		vel  += acc		 * mechPars.baseSplineTime + 0.5 * jerk		* pow_splintTime_2 +  snap	  * pow_splintTime_3 / 6 + crackle * pow_splintTime_4 / 24;
// 		acc  += jerk	 * mechPars.baseSplineTime + 0.5 * snap		* pow_splintTime_2 +  crackle * pow_splintTime_3 / 6;		
// 		jerk += snap	 * mechPars.baseSplineTime + 0.5 * crackle  * pow_splintTime_2;		
// 		snap += crackle  * mechPars.baseSplineTime;	
// 	}
	return NYCE_OK;
}

//直线轨迹
NYCE_STATUS TrajSegmentCubicLine(const TRAJ_SEG_LINE_PARS &linePars, LABEL_MECH_PARS &mechPars, const uint32_t shutIndex = 0)
{
	if ( linePars.maxVel < linePars.endVel	 || 
		-linePars.maxVel > linePars.endVel	 ||
		 linePars.maxVel < linePars.startVel ||
		-linePars.maxVel > linePars.startVel )
		return LABEL_MACHINE_ERR_MOTION_PARS;

	const double disDiff((linePars.endPos[0] - linePars.startPos[0]) * (linePars.endPos[0] - linePars.startPos[0]) + (linePars.endPos[1] - linePars.startPos[1]) * (linePars.endPos[1] - linePars.startPos[1]));
	const double angle(atan2(linePars.endPos[1] - linePars.startPos[1], linePars.endPos[0] - linePars.startPos[0]));
	const double time1(disDiff / (linePars.startVel + linePars.maxVel) * 0.125);
	const double time2(disDiff / (linePars.endVel + linePars.maxVel) * 0.125);

	const double pow_time1_2(pow(time1, 2));
	const double pow_time2_2(pow(time2, 2));
	const double pow_time1_3(pow(time1, 3));
	const double pow_time2_3(pow(time2, 3));
	const double pow_time1_4(pow(time1, 4));
	const double pow_time2_4(pow(time2, 4));

	const double crackle1((linePars.maxVel - linePars.startVel) / pow_time1_4 * 0.125);
	const double crackle2((linePars.endVel - linePars.maxVel) / pow_time2_4 * 0.125);

	double maxJerk0(2 * crackle1 * pow_time1_2);
	double maxJerk1(2 * crackle2 * pow_time2_2);

	double maxAcc0(2 * crackle1 * pow_time1_3);
	double maxAcc1(2 * crackle2 * pow_time2_3);

	double maxVel(linePars.startVel + 8 * crackle1 * pow_time1_4);

	if (maxJerk0 >  linePars.maxJerk ||
		maxJerk0 < -linePars.maxJerk ||
		maxJerk1 >  linePars.maxJerk ||
		maxJerk1 < -linePars.maxJerk )
		return LABEL_MACHINE_ERR_TRAJ_MAX_JERK_EXCEEDED;

	if (maxAcc0 >  linePars.maxAcc ||
		maxAcc0 < -linePars.maxAcc ||
		maxAcc1 >  linePars.maxAcc ||
		maxAcc1 < -linePars.maxAcc )
		return LABEL_MACHINE_ERR_TRAJ_MAX_ACCELERATION_EXCEEDED;

	if (maxVel >  linePars.maxVel ||
		maxVel < -linePars.maxVel )
		return LABEL_MACHINE_ERR_TRAJ_MAX_VELOCITY_EXCEEDED;

	//估算段数
	const double dSplineNum((time1 * 8 + time2 * 8) / mechPars.baseSplineTime); 
	uint32_t splineNum((uint32_t)dSplineNum);

	if (dSplineNum - (double)splineNum > mechPars.lessSplineTime)
		splineNum++;

	//Buffer manage
	if (mechPars.usedNrOfCubPars + splineNum > mechPars.maxNrOfCubPars)
	{
		SAC_CUB_PARS *cubsX = new SAC_CUB_PARS[mechPars.usedNrOfCubPars + splineNum + 512]();
		SAC_CUB_PARS *cubsY = new SAC_CUB_PARS[mechPars.usedNrOfCubPars + splineNum + 512]();

		memcpy(cubsX, mechPars.cubPars[0], mechPars.usedNrOfCubPars * sizeof(SAC_CUB_PARS));
		memcpy(cubsY, mechPars.cubPars[1], mechPars.usedNrOfCubPars * sizeof(SAC_CUB_PARS));

		delete[] mechPars.cubPars[0];
		delete[] mechPars.cubPars[1];

		mechPars.cubPars[0] = cubsX;
		mechPars.cubPars[1] = cubsY;
	}

	//Calc
	const double sinAngle(sin(angle));
	const double cosAngle(cos(angle));

	double currentTime(0.0);
	uint32_t singal(0);
	double timePoint(time1 * 8);
	double cra(0.0), snap(0.0), jerk(0.0), acc(0.0), vel(linePars.startVel), pos(0.0);

	double pow_splintTime_2(pow(mechPars.baseSplineTime, 2));
	double pow_splintTime_3(pow(mechPars.baseSplineTime, 3));
	double pow_splintTime_4(pow(mechPars.baseSplineTime, 4));
	double pow_splintTime_5(pow(mechPars.baseSplineTime, 5));

	uint32_t index(0);
	for (; index < splineNum - 1; ++index)
	{
		currentTime = (index + 1) * mechPars.baseSplineTime;
		if (currentTime < timePoint)
		{
			switch (uint32_t(currentTime / time1))
			{
			case 0:
			case 3:
			case 5:
			case 6:
				cra =  crackle1;
				break;
			case 1:
			case 2:
			case 4:
			case 7:
				cra = -crackle1;
				break;
			default:
				break;
			}
		}
		else
		{
			switch (uint32_t((currentTime - timePoint) / time2))
			{
			case 0:
			case 3:
			case 5:
			case 6:
				cra =  crackle2;
				break;
			case 1:
			case 2:
			case 4:
			case 7:
				cra = -crackle2;
				break;
			default:
				break;
			}
		}

		pos  +=  vel  * mechPars.baseSplineTime + 0.5 * acc  * pow_splintTime_2 +  jerk * pow_splintTime_3 / 6 + snap  * pow_splintTime_4 / 24 + cra * pow_splintTime_5 / 120;
		vel  +=  acc  * mechPars.baseSplineTime + 0.5 * jerk * pow_splintTime_2 +  snap * pow_splintTime_3 / 6 +  cra  * pow_splintTime_4 / 24;
		acc  += jerk  * mechPars.baseSplineTime + 0.5 * snap * pow_splintTime_2 +  cra  * pow_splintTime_3 / 6;		
		jerk += snap  * mechPars.baseSplineTime + 0.5 * cra  * pow_splintTime_2;		
		snap +=  cra  * mechPars.baseSplineTime;	

		mechPars.cubPars[0][mechPars.usedNrOfCubPars].position = linePars.startPos[0] + pos * cosAngle;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].velocity = vel * cosAngle;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].time = mechPars.baseSplineTime;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].generateEvent = mechPars.usedNrOfCubPars % mechPars.perSendSplineNum == 0 ? TRUE : FALSE;

		mechPars.cubPars[1][mechPars.usedNrOfCubPars].position = linePars.startPos[1] + pos * sinAngle;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].velocity = vel * sinAngle;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].time = mechPars.baseSplineTime;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].generateEvent = FALSE;
	}

	mechPars.cubPars[0][mechPars.usedNrOfCubPars].position = linePars.endPos[0];
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].velocity = linePars.endVel * cosAngle;
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].time = time1 * 8 + time2 * 8 - mechPars.usedNrOfCubPars * mechPars.baseSplineTime;
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].generateEvent = mechPars.usedNrOfCubPars % mechPars.perSendSplineNum == 0 ? TRUE : FALSE;
	
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].position = linePars.endPos[1];
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].velocity = linePars.endVel * sinAngle;
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].time = time1 * 8 + time2 * 8 - mechPars.usedNrOfCubPars * mechPars.baseSplineTime;
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].generateEvent = FALSE;

	++mechPars.usedNrOfCubPars;

	if (shutIndex)
		mechPars.shutId[shutIndex -1] = mechPars.usedNrOfCubPars - 1;

	return NYCE_OK;
} 

//圆弧轨迹
NYCE_STATUS TrajSegmentCubicArc(TRAJ_SEG_LINE_PARS &linePars, LABEL_MECH_PARS &mechPars, const uint32_t shutIndex = 0)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	if ( linePars.maxVel < linePars.endVel	 || 
		-linePars.maxVel > linePars.endVel	 ||
		linePars.maxVel < linePars.startVel  ||
		-linePars.maxVel > linePars.startVel )
		return LABEL_MACHINE_ERR_MOTION_PARS;

	linePars.distance = (linePars.endPos[0] - linePars.startPos[0]) * (linePars.endPos[0] - linePars.startPos[0]) + (linePars.endPos[1] - linePars.startPos[1]) * (linePars.endPos[1] - linePars.startPos[1]);
	
	//计算运动参数
	nyceStatus = CalacCubicMotionBasePars(&linePars);

	//估算段数
	const double dSplineNum((linePars.time[0] * 8 + linePars.time[1] * 8) / mechPars.baseSplineTime); 
	uint32_t splineNum((uint32_t)dSplineNum);

	if (dSplineNum - (double)splineNum > mechPars.lessSplineTime)
		splineNum++;

	//管理缓冲区
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : BufferManager(splineNum, mechPars);

	//Calc
	const double angle(atan2(linePars.endPos[1] - linePars.startPos[1], linePars.endPos[0] - linePars.startPos[0]));
	const double sinAngle(sin(angle));
	const double cosAngle(cos(angle));

// 	
// 
// 		mechPars.cubPars[0][mechPars.usedNrOfCubPars].position = linePars.startPos[0] + pos * cosAngle;
// 		mechPars.cubPars[0][mechPars.usedNrOfCubPars].velocity = vel * cosAngle;
// 		mechPars.cubPars[0][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
// 		mechPars.cubPars[0][mechPars.usedNrOfCubPars].time = mechPars.baseSplineTime;
// 		mechPars.cubPars[0][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
// 		mechPars.cubPars[0][mechPars.usedNrOfCubPars].generateEvent = mechPars.usedNrOfCubPars % mechPars.perSendSplineNum == 0 ? TRUE : FALSE;
// 
// 		mechPars.cubPars[1][mechPars.usedNrOfCubPars].position = linePars.startPos[1] + pos * sinAngle;
// 		mechPars.cubPars[1][mechPars.usedNrOfCubPars].velocity = vel * sinAngle;
// 		mechPars.cubPars[1][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
// 		mechPars.cubPars[1][mechPars.usedNrOfCubPars].time = mechPars.baseSplineTime;
// 		mechPars.cubPars[1][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
// 		mechPars.cubPars[1][mechPars.usedNrOfCubPars].generateEvent = FALSE;
// 	}
// 
// 	mechPars.cubPars[0][mechPars.usedNrOfCubPars].position = linePars.endPos[0];
// 	mechPars.cubPars[0][mechPars.usedNrOfCubPars].velocity = linePars.endVel * cosAngle;
// 	mechPars.cubPars[0][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
// 	mechPars.cubPars[0][mechPars.usedNrOfCubPars].time = time1 * 8 + time2 * 8 - mechPars.usedNrOfCubPars * mechPars.baseSplineTime;
// 	mechPars.cubPars[0][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
// 	mechPars.cubPars[0][mechPars.usedNrOfCubPars].generateEvent = mechPars.usedNrOfCubPars % mechPars.perSendSplineNum == 0 ? TRUE : FALSE;
// 
// 	mechPars.cubPars[1][mechPars.usedNrOfCubPars].position = linePars.endPos[1];
// 	mechPars.cubPars[1][mechPars.usedNrOfCubPars].velocity = linePars.endVel * sinAngle;
// 	mechPars.cubPars[1][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
// 	mechPars.cubPars[1][mechPars.usedNrOfCubPars].time = time1 * 8 + time2 * 8 - mechPars.usedNrOfCubPars * mechPars.baseSplineTime;
// 	mechPars.cubPars[1][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
// 	mechPars.cubPars[1][mechPars.usedNrOfCubPars].generateEvent = FALSE;
// 
// 	++mechPars.usedNrOfCubPars;
// 
// 	if (shutIndex)
// 		mechPars.shutId[shutIndex -1] = mechPars.usedNrOfCubPars - 1;

	return NYCE_OK;
} 
                   



//拓展曲线轨迹，未完善
NYCE_STATUS TrajSegmentCubicCurve(const TRAJ_SEG_CURVE_PARS &curvePars, LABEL_MECH_PARS &mechPars, const uint32_t shutIndex = 0)
{
	if ( curvePars.maxVel[0] < curvePars.endVel[0]	 || 
		-curvePars.maxVel[0] > curvePars.endVel[0]	 ||
		curvePars.maxVel[1] < curvePars.endVel[1]	 || 
		-curvePars.maxVel[1] > curvePars.endVel[1]	 ||
		curvePars.maxVel[0] < curvePars.startVel[0] ||
		-curvePars.maxVel[0] > curvePars.startVel[0] ||
		curvePars.maxVel[1] < curvePars.startVel[1] ||
		-curvePars.maxVel[1] > curvePars.startVel[1] )
		return LABEL_MACHINE_ERR_MOTION_PARS;

	const double disDiffX(curvePars.endPos[0] - curvePars.startPos[0]);
	const double disDiffY(curvePars.endPos[1] - curvePars.startPos[1]);
	double timeX[2], timeY[2];
	double crackleX[2], crackleY[2];

	//Estimate
	timeX[0] = disDiffX / (curvePars.startVel[0] + curvePars.maxVel[0]) * 0.125;
	timeX[1] = disDiffX / (curvePars.endVel[0] + curvePars.maxVel[0]) * 0.125;
	timeY[0] = disDiffY / (curvePars.startVel[1] + curvePars.maxVel[1]) * 0.125;
	timeY[1] = disDiffY / (curvePars.endVel[1] + curvePars.maxVel[1]) * 0.125;

	double pow_timeX0_4(pow(timeX[0], 4));
	double pow_timeX1_4(pow(timeX[1], 4));
	double pow_timeY0_4(pow(timeY[0], 4));
	double pow_timeY1_4(pow(timeY[1], 4));

	crackleX[0] = (curvePars.maxVel[0] - curvePars.startVel[0]) / pow_timeX0_4 * 0.125;
	crackleX[1] = (curvePars.endVel[0] - curvePars.maxVel[0])   / pow_timeX1_4 * 0.125;
	crackleY[0] = (curvePars.maxVel[1] - curvePars.startVel[1]) / pow_timeY0_4 * 0.125;
	crackleY[1] = (curvePars.endVel[1] - curvePars.maxVel[1])   / pow_timeY1_4 * 0.125;

	//Correct
	if (timeX[0] + timeX[1] < timeY[0] + timeY[1])
	{
		timeX[0] = disDiffX * 0.25 - (curvePars.endVel[0] + curvePars.maxVel[0]) * (timeX[0] + timeX[1]) / (curvePars.startVel[0] + curvePars.endVel[0]);
		timeX[1] = timeY[0] + timeY[1] - timeX[0];

		crackleX[0] = (curvePars.maxVel[0] - curvePars.startVel[0]) / pow_timeX0_4 * 0.125;
		crackleX[1] = (curvePars.endVel[0] - curvePars.maxVel[0])   / pow_timeX1_4 * 0.125;
	}
	if (timeX[0] + timeX[1] > timeY[0] + timeY[1])
	{
		timeY[0] = disDiffY * 0.25 - (curvePars.endVel[1] + curvePars.maxVel[1]) * (timeX[0] + timeX[1]) / (curvePars.startVel[1] + curvePars.endVel[1]);
		timeY[1] = timeX[0] + timeX[1] - timeY[0];

		crackleY[0] = (curvePars.maxVel[1] - curvePars.startVel[1]) / pow_timeY0_4 * 0.125;
		crackleY[1] = (curvePars.endVel[1] - curvePars.maxVel[1])   / pow_timeY1_4 * 0.125;
	}

	//Judgment
	double maxJerkX0(2 * crackleX[0] * pow(timeX[0], 2));
	double maxJerkX1(2 * crackleX[1] * pow(timeX[1], 2));
	double maxJerkY0(2 * crackleY[0] * pow(timeY[0], 2));
	double maxJerkY1(2 * crackleY[1] * pow(timeY[1], 2));

	double maxAccX0(2 * crackleX[0] * pow(timeX[0], 3));
	double maxAccX1(2 * crackleX[1] * pow(timeX[1], 3));
	double maxAccY0(2 * crackleY[0] * pow(timeY[0], 3));
	double maxAccY1(2 * crackleY[1] * pow(timeY[1], 3));

	double maxVelX(curvePars.startVel[0] + 8 * crackleX[0] * pow_timeX0_4);
	double maxVelY(curvePars.startVel[1] + 8 * crackleY[0] * pow_timeY0_4);

	if (maxJerkX0 >  curvePars.maxJerk[0] ||
		maxJerkX0 < -curvePars.maxJerk[0] ||
		maxJerkX1 >  curvePars.maxJerk[0] ||
		maxJerkX1 < -curvePars.maxJerk[0] )
		return LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_JERK_EXCEEDED;

	if (maxJerkY0 >  curvePars.maxJerk[1] ||
		maxJerkY0 < -curvePars.maxJerk[1] ||
		maxJerkY1 >  curvePars.maxJerk[1] ||
		maxJerkY1 < -curvePars.maxJerk[1] )
		return LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_JERK_EXCEEDED;

	if (maxAccX0 >  curvePars.maxAcc[0] ||
		maxAccX0 < -curvePars.maxAcc[0] ||
		maxAccX1 >  curvePars.maxAcc[0] ||
		maxAccX1 < -curvePars.maxAcc[0] )
		return LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_ACCELERATION_EXCEEDED;

	if (maxAccY0 >  curvePars.maxAcc[1] ||
		maxAccY0 < -curvePars.maxAcc[1] ||
		maxAccY1 >  curvePars.maxAcc[1] ||
		maxAccY1 < -curvePars.maxAcc[1] )
		return LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_ACCELERATION_EXCEEDED;

	if (maxVelX >  curvePars.maxVel[0] ||
		maxVelX < -curvePars.maxVel[0] )
		return LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_VELOCITY_EXCEEDED;

	if (maxVelY >  curvePars.maxVel[1] ||
		maxVelY < -curvePars.maxVel[1] )
		return LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_VELOCITY_EXCEEDED;

	//估算段数
	const double dSplineNum((timeY[0] * 8 + timeY[1] * 8) / mechPars.baseSplineTime); 
	uint32_t splineNum((uint32_t)dSplineNum);

	if (dSplineNum - (double)splineNum > mechPars.lessSplineTime)
		splineNum++;

	//Buffer manage
	if (mechPars.usedNrOfCubPars + splineNum > mechPars.maxNrOfCubPars)
	{
		SAC_CUB_PARS *cubsX = new SAC_CUB_PARS[mechPars.usedNrOfCubPars + splineNum + 512]();
		SAC_CUB_PARS *cubsY = new SAC_CUB_PARS[mechPars.usedNrOfCubPars + splineNum + 512]();

		memcpy(cubsX, mechPars.cubPars[0], mechPars.usedNrOfCubPars * sizeof(SAC_CUB_PARS));
		memcpy(cubsY, mechPars.cubPars[1], mechPars.usedNrOfCubPars * sizeof(SAC_CUB_PARS));

		delete[] mechPars.cubPars[0];
		delete[] mechPars.cubPars[1];

		mechPars.cubPars[0] = cubsX;
		mechPars.cubPars[1] = cubsY;
	}

	//Calc
	double currentTime(0.0);
	uint32_t singal(0);
	double timePointX(timeX[0] * 8);
	double timePointY(timeY[0] * 8);
	double craX(0.0), snapX(0.0), jerkX(0.0), accX(0.0), velX(curvePars.startVel[0]), posX(curvePars.startPos[0]);
	double craY(0.0), snapY(0.0), jerkY(0.0), accY(0.0), velY(curvePars.startVel[0]), posY(curvePars.startPos[0]);

	double pow_splintTime_2(pow(mechPars.baseSplineTime, 2));
	double pow_splintTime_3(pow(mechPars.baseSplineTime, 3));
	double pow_splintTime_4(pow(mechPars.baseSplineTime, 4));
	double pow_splintTime_5(pow(mechPars.baseSplineTime, 5));

	uint32_t index(0);
	for (; index < splineNum - 1; ++index)
	{
		currentTime = (index + 1) * mechPars.baseSplineTime;
		if (currentTime < timePointX)
		{
			switch (uint32_t(currentTime / timeX[0]))
			{
			case 0:
			case 3:
			case 5:
			case 6:
				craX =  crackleX[0];
				break;
			case 1:
			case 2:
			case 4:
			case 7:
				craX = -crackleX[0];
				break;
			default:
				break;
			}
		}
		else
		{
			switch (uint32_t((currentTime - timePointX) / timeX[1]))
			{
			case 0:
			case 3:
			case 5:
			case 6:
				craX =  crackleX[1];
				break;
			case 1:
			case 2:
			case 4:
			case 7:
				craX = -crackleX[1];
				break;
			default:
				break;
			}
		}

		posX  += velX  * mechPars.baseSplineTime + 0.5 * accX  * pow_splintTime_2 + jerkX * pow_splintTime_3 / 6 + snapX * pow_splintTime_4 / 24 + craX * pow_splintTime_5 / 120;
		velX  += accX  * mechPars.baseSplineTime + 0.5 * jerkX * pow_splintTime_2 + snapX * pow_splintTime_3 / 6 + craX  * pow_splintTime_4 / 24;
		accX  += jerkX * mechPars.baseSplineTime + 0.5 * snapX * pow_splintTime_2 + craX  * pow_splintTime_3 / 6;		
		jerkX += snapX * mechPars.baseSplineTime + 0.5 * craX  * pow_splintTime_2;		
		snapX += craX  * mechPars.baseSplineTime;	

		mechPars.cubPars[0][mechPars.usedNrOfCubPars].position = posX;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].velocity = velX;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].time = mechPars.baseSplineTime;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
		mechPars.cubPars[0][mechPars.usedNrOfCubPars].generateEvent = mechPars.usedNrOfCubPars % mechPars.perSendSplineNum == 0 ? TRUE : FALSE;

		if (currentTime < timePointY)
		{
			switch (uint32_t(currentTime / timeY[0]))
			{
			case 0:
			case 3:
			case 5:
			case 6:
				craY =  crackleY[0];
				break;
			case 1:
			case 2:
			case 4:
			case 7:
				craY = -crackleY[0];
				break;
			default:
				break;
			}
		}
		else
		{
			switch (uint32_t((currentTime - timePointY) / timeY[1]))
			{
			case 0:
			case 3:
			case 5:
			case 6:
				craY =  crackleY[1];
				break;
			case 1:
			case 2:
			case 4:
			case 7:
				craY = -crackleY[1];
				break;
			default:
				break;
			}
		}

		posY  += velY  * mechPars.baseSplineTime + 0.5 * accY  * pow_splintTime_2 + jerkY * pow_splintTime_3 / 6 + snapY * pow_splintTime_4 / 24 + craY * pow_splintTime_5 / 120;
		velY  += accY  * mechPars.baseSplineTime + 0.5 * jerkY * pow_splintTime_2 + snapY * pow_splintTime_3 / 6 +  craY * pow_splintTime_4 / 24;
		accY  += jerkY * mechPars.baseSplineTime + 0.5 * snapY * pow_splintTime_2 +  craY * pow_splintTime_3 / 6;		
		jerkY += snapY * mechPars.baseSplineTime + 0.5 * craY  * pow_splintTime_2;		
		snapY += craY  * mechPars.baseSplineTime;	

		mechPars.cubPars[1][mechPars.usedNrOfCubPars].position = posY;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].velocity = velY;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].time = mechPars.baseSplineTime;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
		mechPars.cubPars[1][mechPars.usedNrOfCubPars].generateEvent = FALSE;

		++mechPars.usedNrOfCubPars;
	}

	mechPars.cubPars[0][mechPars.usedNrOfCubPars].position = curvePars.endPos[0];
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].velocity = curvePars.endVel[0];
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].time = timeX[0] * 8 + timeX[1] * 8 - mechPars.usedNrOfCubPars * mechPars.baseSplineTime;
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
	mechPars.cubPars[0][mechPars.usedNrOfCubPars].generateEvent = mechPars.usedNrOfCubPars % mechPars.perSendSplineNum == 0 ? TRUE : FALSE;

	mechPars.cubPars[1][mechPars.usedNrOfCubPars].position = curvePars.endPos[1];
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].velocity = curvePars.endVel[1];
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].splineId = mechPars.usedNrOfCubPars;
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].time = timeY[0] * 8 + timeY[1] * 8 - mechPars.usedNrOfCubPars * mechPars.baseSplineTime;
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].positionReference = SAC_ABSOLUTE;
	mechPars.cubPars[1][mechPars.usedNrOfCubPars].generateEvent = FALSE;

	++mechPars.usedNrOfCubPars;

	if (shutIndex)
		mechPars.shutId[shutIndex -1] = mechPars.usedNrOfCubPars - 1;

	return NYCE_OK;
}