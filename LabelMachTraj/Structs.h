#pragma once

/*
--20150914 Martin
	labelMechPars结构体贯穿整个框架，是所有内部数据的储存点。
	因为轨迹规划、运动学处理（暂时没有）、发送器三大部分的数据互相交叉，设计一个通用结构体能快速解决问题且方便日后拓展。
	但是显然而见，这不是一个最好的解决方案，阅读、管理等方面都存在问题。
	仿照ROCKS的逻辑而设计，不同的是labelMechPars不对用户开放。
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
	double lessSplineTime;//segment太多，lessSplineTime太小，可能存在发送数据来不及的情况
	double lastSplineTime;

	uint32_t maxNrOfCubPars;
	uint32_t usedNrOfCubPars;

	uint32_t shutId[2];//拍照的splineID

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
	double distance;//*正数
	double startVel;//正数
	double endVel;//正数
	double maxVel;//正数
	double maxAcc;//正数
	double maxJerk;//正数
	double time[2];//*正数
	double crackle[2];//*
}TRAJ_SEG_PARS;

typedef struct trajSegLinePars : trajSegPars
{
	//所有数据都是正数
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
	//除转角外，所有数据都是正数
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
	double angle;//正:CCW 负:CW
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

//未使用
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
	double maxVel[2];//正数
	double maxAcc[2];//正数
	double maxJerk[2];//正数
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
	SAC_AXIS axId[2];//XY轴按顺序储存
	double nozzleRelPos1[2];//吸嘴1相对头（？）的位置
	double nozzleRelPos2[2];//吸嘴2相对头（？）的位置
	void (*pShut)(void);//拍照回调函数
	double splineTime;//样条曲线采样点间隔时间，建议0.005
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
	double endPos[2];//目标位置
	double cameraPos[2];//相机位置
	double rotaeRadius[2];//两次转向半径,根据估算速度动态调节
	double rotaeAngle[2];//两次转向弧度角，正数,大小适中
	double shutterDelay;//快门延时
	double cameraVel;//吸嘴通过相机位置的速度
	double maxVel;//吸嘴最大速度
	double maxAcc;//吸嘴最大加速度
	double maxJerk;//吸嘴最大加加速度
}LABEL_MECH_MOTION_PARS;