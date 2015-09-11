#pragma once

#include <Winsock2.h>
#include <process.h>

#include "nyceapi.h"
#include "sacapi.h"
#include "Structs.h"

HANDLE hThread;
unsigned threadID;

void EventThrd( NYCE_ID nyceId, NYCE_EVENT eventId, NYCE_EVENT_DATA *pEventData, void *pUserData )
{
	LABEL_MECH_PARS *pMechPars = (LABEL_MECH_PARS*)pUserData;

	if (nyceId.type == SAC_ID_TYPE && nyceId.id == pMechPars->axId[0])
	{
		if (pEventData->uEventUlong == pMechPars->shutId[0] || pEventData->uEventUlong == pMechPars->shutId[1])
			pMechPars->pShut();

		if (pEventData->uEventUlong % pMechPars->perSendSplineNum == 0)
			ResumeThread(hThread);
	}
}

NYCE_STATUS __stdcall SendSplineThrd(void * pParam)
{
	LABEL_MECH_PARS *pMechPars = (LABEL_MECH_PARS*)pParam;

	NYCE_STATUS nyceStatus(NYCE_OK);

	uint32_t loopTimes(NYCE_MAX_NR_OF_BUFFERED_SPLINE_SEGMENTS / pMechPars->perSendSplineNum);

	uint32_t restSplineNum(pMechPars->usedNrOfCubPars);
	while(--loopTimes > 0 && restSplineNum > 0)//--20150911 Martin
											   //--loopTimes，先减去1
											   //使MCU缓冲区余量大于1次小于两次发送量
											   //在执行第一个spline数据时马上有回调
	{
		if (restSplineNum > pMechPars->perSendSplineNum)
		{
			nyceStatus = SacWriteCubicIntBuffer(pMechPars->axId[0], pMechPars->perSendSplineNum, &pMechPars->cubPars[0][pMechPars->haveSentSplineNum]);

			nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacWriteCubicIntBuffer(pMechPars->axId[1], pMechPars->perSendSplineNum, &pMechPars->cubPars[1][pMechPars->haveSentSplineNum]);

			pMechPars->haveSentSplineNum += pMechPars->perSendSplineNum;
		}
		else
		{
			nyceStatus = SacWriteCubicIntBuffer(pMechPars->axId[0], restSplineNum, &pMechPars->cubPars[0][pMechPars->haveSentSplineNum]);

			nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacWriteCubicIntBuffer(pMechPars->axId[1], restSplineNum, &pMechPars->cubPars[1][pMechPars->haveSentSplineNum]);

			pMechPars->haveSentSplineNum = pMechPars->usedNrOfCubPars;
		}

		restSplineNum = pMechPars->usedNrOfCubPars - pMechPars->haveSentSplineNum;

		if (NyceError(nyceStatus))
			break;
	}

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStartInterpolation(pMechPars->axId[0]);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStartInterpolation(pMechPars->axId[1]);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : MacStartSyncGroup(pMechPars->groupId, MAC_SYNC_MOTION);

	while(restSplineNum > 0)
	{
		SuspendThread(hThread);

		if (pMechPars->streamTimeOut)
			break;

		if (restSplineNum > pMechPars->perSendSplineNum)
		{
			nyceStatus = SacWriteCubicIntBuffer(pMechPars->axId[0], pMechPars->perSendSplineNum, &pMechPars->cubPars[0][pMechPars->haveSentSplineNum]);

			nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacWriteCubicIntBuffer(pMechPars->axId[1], pMechPars->perSendSplineNum, &pMechPars->cubPars[1][pMechPars->haveSentSplineNum]);

			pMechPars->haveSentSplineNum += pMechPars->perSendSplineNum;
		}
		else
		{
			nyceStatus = SacWriteCubicIntBuffer(pMechPars->axId[0], restSplineNum, &pMechPars->cubPars[0][pMechPars->haveSentSplineNum]);

			nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacWriteCubicIntBuffer(pMechPars->axId[1], restSplineNum, &pMechPars->cubPars[1][pMechPars->haveSentSplineNum]);

			pMechPars->haveSentSplineNum = pMechPars->usedNrOfCubPars;

			break;
		}

		restSplineNum = pMechPars->usedNrOfCubPars - pMechPars->haveSentSplineNum;

		if (NyceError(nyceStatus))
			break;
	}

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStopInterpolation(pMechPars->axId[0]);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStopInterpolation(pMechPars->axId[1]);

	return nyceStatus;
}

NYCE_STATUS stream(LABEL_MECH_PARS &mechPars)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = SacClearInterpolantBuffer(mechPars.axId[0]);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacClearInterpolantBuffer(mechPars.axId[1]);

	mechPars.haveSentSplineNum = 0;
	mechPars.streamTimeOut = FALSE;

	double currentPos[2];
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceReadVariableSet(mechPars.varId[0], mechPars.varId[1], currentPos);
	if (currentPos[0] != mechPars.cubPars[0][0].position || currentPos[1] != mechPars.cubPars[1][0].position)
		return LABEL_MACHINE_ERR_START_POSITION_CHANGED;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDefineEventEnrolment(mechPars.axId[0], SAC_EV_INTERPOLANT_STARTED, EventThrd, &mechPars);
	
	hThread = (HANDLE)_beginthreadex(NULL, 0, SendSplineThrd, (LPVOID)&mechPars, 0, &threadID);
	
	return nyceStatus;
}

NYCE_STATUS streamSync(LABEL_MECH_PARS &mechPars, double timeout = INFINITE)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	if(WaitForSingleObject(hThread, (DWORD)timeout) != WAIT_OBJECT_0)
	{
		mechPars.streamTimeOut = TRUE;
		ResumeThread(hThread);

		nyceStatus = LABEL_MACHINE_ERR_STREAM_TIMEOUT;
	}
	else
	{
		DWORD exitCode;
		GetExitCodeThread(hThread, &exitCode);
		nyceStatus = (NYCE_STATUS)exitCode;

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize(mechPars.axId[0], SAC_REQ_MOTION_STOPPED, timeout);

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize(mechPars.axId[1], SAC_REQ_MOTION_STOPPED, 0);
	}

	return nyceStatus;
}