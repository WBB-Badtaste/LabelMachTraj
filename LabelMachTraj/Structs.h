#pragma once

/*
--20150914 Martin
	labelMechPars�ṹ��ᴩ������ܣ��������ڲ����ݵĴ���㡣
	��Ϊ�켣�滮���˶�ѧ������ʱû�У������������󲿷ֵ����ݻ��ཻ�棬���һ��ͨ�ýṹ���ܿ��ٽ�������ҷ����պ���չ��
	������Ȼ�������ⲻ��һ����õĽ���������Ķ�������ȷ��涼�������⡣
	����ROCKS���߼�����ƣ���ͬ����labelMechPars�����û����š�
*/
typedef struct labelMechPars
{
	labelMechPars():
		haveInited(FALSE),
		streamTimeOut(FALSE),
		groupId(0),
		perSendSplineNum(0),
		haveSentSplineNum(0),
		baseSplineTime(0.0),
		lessSplineTime(0.0),
		maxNrOfCubPars(0),
		usedNrOfCubPars(0),
		pShut(nullptr)
	{
		axId[0] = 0;
		axId[1] = 0;
		varId[0] = 0;
		varId[1] = 0;
		nozzleRelPos1[0] = 0.0;
		nozzleRelPos1[1] = 0.0;
		nozzleRelPos2[0] = 0.0;
		nozzleRelPos2[1] = 0.0;
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

	double nozzleRelPos1[2];
	double nozzleRelPos2[2];

	SAC_CUB_PARS *cubPars[2];
	double *segDistance;
	double *segVelocity;
	uint32_t perSendSplineNum;
	uint32_t haveSentSplineNum;

	double baseSplineTime;
	double lessSplineTime;//segment̫�࣬lessSplineTime̫С�����ܴ��ڷ������������������
	double lastSplineTime;

	uint32_t maxNrOfCubPars;
	uint32_t usedNrOfCubPars;

	uint32_t shutId[2];//���յ�splineID

	void (*pShut)(void);
}LABEL_MECH_PARS;

typedef struct trajSegStartPars 
{
	trajSegStartPars()
	{
		startPos[0] = startPos [1] = 0.0;
	}
	double startPos[2];
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
	double distance;//*����
	double startVel;//����
	double endVel;//����
	double maxVel;//����
	double maxAcc;//����
	double maxJerk;//����
	double time[2];//*����
	double crackle[2];//*
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
	//��ת���⣬�������ݶ�������
	trajSegArcPars() : angle(0.0), trajSegPars()
	{
		startPos[0] = 0.0;
		startPos[1] = 0.0;
		endPos[0] = 0.0;
		endPos[1] = 0.0;
		center[0] = 0.0;
		center[1] = 0.0;
	}
	double startPos[2];
	double endPos[2];
	double center[2];
	double angle;//��:CCW ��:CW
}TRAJ_SEG_ARC_PARS;

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

//δʹ��
typedef struct trajSegCurvePars
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


//API
typedef struct labelMechInitPars
{
	labelMechInitPars():splineTime(0.0), pShut(nullptr)
	{
		axId[0] = 0;
		axId[1] = 0;
		nozzleRelPos1[0] = 0.0;
		nozzleRelPos1[1] = 0.0;
		nozzleRelPos2[0] = 0.0;
		nozzleRelPos2[1] = 0.0;
	}
	SAC_AXIS axId[2];//XY�ᰴ˳�򴢴�
	double nozzleRelPos1[2];//����1���ͷ��������λ��
	double nozzleRelPos2[2];//����2���ͷ��������λ��
	void (*pShut)(void);//���ջص�����
	double splineTime;//�������߲�������ʱ�䣬����0.005
}LABEL_MECH_INIT_PARS;

typedef struct labelMechMotionPars
{
	labelMechMotionPars():cameraVel(0.0),shutterDelay(0.0),maxVel(0.0),maxAcc(0.0),maxJerk(0.0)
	{
		endPos[0] = 0.0;
		endPos[1] = 0.0;
		cameraPos[0] = 0.0;
		cameraPos[1] = 0.0;
		rotaeRadius[0] = 0.0;
		rotaeRadius[1] = 0.0;
		rotaeAngle[0] = 0.0;
		rotaeAngle[1] = 0.0;
	}
	double endPos[2];//Ŀ��λ��
	double cameraPos[2];//���λ��
	double rotaeRadius[2];//����ת��뾶,���ݹ����ٶȶ�̬����
	double rotaeAngle[2];//����ת�򻡶Ƚǣ�����,��С����
	double shutterDelay;//������ʱ
	double cameraVel;//����ͨ�����λ�õ��ٶ�
	double maxVel;//��������ٶ�
	double maxAcc;//���������ٶ�
	double maxJerk;//�������Ӽ��ٶ�
}LABEL_MECH_MOTION_PARS;