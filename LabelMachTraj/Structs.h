#pragma once

//�ڲ�ʹ��
typedef struct labelMechPars
{
	labelMechPars():
		haveInited(FALSE),
		streamTimeOut(FALSE),
		groupId(0),
		nozzleInterval(0.0),
		nozzleDeflection(0.0),
		perSendSplineNum(0),
		haveSentSplineNum(0),
		baseSplineTime(0.0),
		lessSplineTime(0.0),
		maxNrOfCubPars(0),
		usedNrOfCubPars(0),
		shutOffest(0.0),
		pShut(nullptr)
	{
		axId[0] = 0;
		axId[1] = 0;
		varId[0] = 0;
		varId[1] = 0;
		cameraPos[0] = 0;
		cameraPos[1] = 0;
		firstShutPos[0] = 0;
		firstShutPos[1] = 0;
		SecondShutPos[0] = 0;
		SecondShutPos[1] = 0;
		finishShutPos[0] = 0;
		finishShutPos[1] = 0;
		cubPars[0] = nullptr;
		cubPars[1] = nullptr;
		shutId[0] = 0;
		shutId[1] = 0;
	}

	BOOL haveInited;
	BOOL streamTimeOut;

	uint32_t groupId;
	SAC_AXIS axId[2];
	SAC_VAR_ID varId[2];

	double cameraPos[2];
	double nozzleInterval;
	double nozzleDeflection;

	double firstShutPos[2];
	double SecondShutPos[2];
	double finishShutPos[2];

	SAC_CUB_PARS *cubPars[2];
	double *segDistance;
	double *segVelocity;
	uint32_t perSendSplineNum;
	uint32_t haveSentSplineNum;
	double baseSplineTime;
	double lessSplineTime;
	uint32_t maxNrOfCubPars;
	uint32_t usedNrOfCubPars;
	uint32_t shutId[2];//���յ�splineID
	double shutOffest;

	void (*pShut)(void);
}LABEL_MECH_PARS;

typedef struct trajSegStartPars 
{
	trajSegStartPars():splineTime(0.0)
	{
		startPos[0] = startPos [1] = 0.0;
	}
	double startPos[2];
	double splineTime;
}TRAJ_SEG_START_PARS;

typedef struct trajSegPars
{
	trajSegPars():
		distance(0.0),
		startVel(0.0),
		endVel(0.0),
		maxVel(0.0),
		maxAcc(0.0),
		maxJerk(0.0)
	{
		time[0] = 0.0;
		time[1] = 0.0;
		crackle[0] = 0.0;
		crackle[1] = 0.0;
	}
	double distance;//����
	double startVel;//����
	double endVel;//����
	double maxVel;//����
	double maxAcc;//����
	double maxJerk;//����
	double time[2];//����
	double crackle[2];
}TRAJ_SEG_PARS;

typedef struct trajSegLinePars : trajSegPars
{
	//�������ݶ�������
	trajSegLinePars() : trajSegPars()
	{
		startPos[0] = 0.0;
		startPos[1] = 0.0;
		endPos[0] = 0.0;
		endPos[1] = 0.0;
	}
	double startPos[2];
	double endPos[2];
}TRAJ_SEG_LINE_PARS;

typedef struct trajSegArcPars : trajSegPars
{
	//�������ݶ�������
	trajSegArcPars() : angle(0.0), trajSegPars()
	{
		startPos[0] = 0.0;
		startPos[1] = 0.0;
		endPos[0] = 0.0;
		endPos[1] = 0.0;
	}
	double startPos[2];
	double endPos[2];
	double angle;
}TRAJ_SEG_ARC_PARS;

typedef struct trajSegCurvePars//δʹ��
{
	trajSegCurvePars()
	{
		startPos[0] = 0.0;
		startPos[1] = 0.0;
		endPos[0] = 0.0;
		endPos[1] = 0.0;
		startVel[0] = 0.0;
		startVel[1] = 0.0;
		endVel[0] = 0.0;
		endVel[1] = 0.0;
		maxVel[0] = 0.0;
		maxVel[1] = 0.0;
		maxAcc[0] = 0.0;
		maxAcc[1] = 0.0;
		maxJerk[0] = 0.0;
		maxJerk[1] = 0.0;
	}
	double startPos[2];
	double endPos[2];
	double startVel[2];
	double endVel[2];
	double maxVel[2];//����
	double maxAcc[2];//����
	double maxJerk[2];//����
}TRAJ_SEG_CURVE_PARS;

typedef struct singularPointPars
{
	singularPointPars() : time(0.0), crackle(0.0), snap(0.0), jerk(0.0), acceleration(0.0), velocity(0.0), distance(0.0){}
	double time;
	double crackle;
	double snap;
	double jerk;
	double acceleration;
	double velocity;
	double distance;
}SINGULAR_POINT_PARS;




//API
typedef struct labelMechInitPars
{
	labelMechInitPars():nozzleInterval(0.0),nozzleDeflection(0.0),pShut(nullptr)
	{
		axId[0] = 0;
		axId[1] = 0;
		cameraPos[0] = 0.0;
		cameraPos[1] = 0.0;
	}
	SAC_AXIS axId[2];//XY�ᰴ˳�򴢴�
	double cameraPos[2];
	double nozzleInterval;//������
	double nozzleDeflection;//����������X��ƫ�ǣ�-90~90��
	void (*pShut)(void);
}LABEL_MECH_INIT_PARS;

typedef struct labelMechMotionPars
{
	labelMechMotionPars():cameraVel(0.0),shutterDelay(0.0),splineTime(0.0)
	{
		endPos[0] = 0.0;
		endPos[1] = 0.0;
		maxVel[0] = 0.0;
		maxVel[1] = 0.0;
		maxAcc[0] = 0.0;
		maxAcc[1] = 0.0;
		maxJerk[0] = 0.0;
		maxJerk[1] = 0.0;
	}
	double endPos[2];//Ŀ��λ��
	double cameraVel;//ͨ�����λ�õ��ٶ�
	double shutterDelay;//������ʱ
	double maxVel[2];
	double maxAcc[2];
	double maxJerk[2];
	double splineTime;
}LABEL_MECH_MOTION_PARS;