
#pragma once

#include "LabelMachDefs.h"
#include "Trajectory.h"
#include "Streamer.h"
#include "Structs.h"

#include <iostream>
#include <fstream>

//#define _OUTPUT_DATAS_

LABEL_MECH_PARS mechPars;

using namespace std;

/*
--20150914 Martin
	初始化函数
	主要作用是：
		1.将轴分组，使各轴同步运动；
		2.将各轴的SETPOINT_POS加入velSet，使读取位置时各轴的位置数据在同一个sample，提高位置的计算精度。
		3.将不随意变动的数据（包括拍照函数指针）提前保存到mechPars，减少运动函数的传入参数量
*/
NYCE_STATUS LabelMechInit(const LABEL_MECH_INIT_PARS &initPars)
{
	if (mechPars.haveInited)
		return LABEL_MACHINE_ERR_REINIT;

	mechPars.baseSplineTime = initPars.splineTime;

	mechPars.axId[0] = initPars.axId[0];
	mechPars.axId[1] = initPars.axId[1];

	mechPars.nozzleRelPos1[0] = initPars.nozzleRelPos1[0];
	mechPars.nozzleRelPos1[1] = initPars.nozzleRelPos1[1];
	mechPars.nozzleRelPos2[0] = initPars.nozzleRelPos2[0];
	mechPars.nozzleRelPos2[1] = initPars.nozzleRelPos2[1];

	mechPars.pShut = initPars.pShut;

	//以下变量不建议用户随意修改
	mechPars.perSendSplineNum = NYCE_MAX_NR_OF_SPLINE_SEGMENTS_WRITE;
	mechPars.lessSplineTime = 0.0002;
	mechPars.radiusFactor[0] = 0.0005;
	mechPars.radiusFactor[1] = 0.0005;

	if (mechPars.baseSplineTime < mechPars.lessSplineTime)
		return LABEL_MACHINE_ERR_INIT_PARS_SPLINETIME;

	NYCE_STATUS nyceStatus(MacDefineSyncGroup(mechPars.axId, 2, &mechPars.groupId));

	if (NyceSuccess(nyceStatus))
		nyceStatus = SacAddVariableToSet(mechPars.axId[0], SAC_VAR_SETPOINT_POS, &mechPars.varId[0]);

	if (NyceSuccess(nyceStatus))
		nyceStatus = SacAddVariableToSet(mechPars.axId[1], SAC_VAR_SETPOINT_POS, &mechPars.varId[1]);

	if (NyceSuccess(nyceStatus))
		mechPars.haveInited = TRUE;

	return nyceStatus;
}

//终止函数
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

//运动函数
NYCE_STATUS LabelMechMoveOptTrajectory(const LABEL_MECH_MOTION_PARS &motionPars)
{
	if (!mechPars.haveInited)
		return LABEL_MACHINE_ERR_HAVE_NOT_INITED;

	double currentPos[2];
	NYCE_STATUS nyceStatus(NyceReadVariableSet(mechPars.varId[0], mechPars.varId[1],currentPos));

	TRAJ_SEG_START_PARS startPars;
	startPars.startPos[0] = currentPos[0];
	startPars.startPos[1] = currentPos[1];
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : TarjSegmentStart(startPars, mechPars);

	if (NyceSuccess(nyceStatus))
	{
		const double shutDistance(motionPars.cameraVel * motionPars.shutterDelay);//快门缓冲距离
		const double nozzleOffset1(sqrt(mechPars.nozzleRelPos1[0] * mechPars.nozzleRelPos1[0] + mechPars.nozzleRelPos1[1] * mechPars.nozzleRelPos1[1]));//吸嘴1偏移距离
		const double nozzleOffset2(sqrt(mechPars.nozzleRelPos2[0] * mechPars.nozzleRelPos2[0] + mechPars.nozzleRelPos2[1] * mechPars.nozzleRelPos2[1]));//吸嘴2偏移距离

		double cameraAngle(0.0);//进入拍照区的向量角,指向相机位置
		double firstShutPos[2];//首次快门位置
		double secondShutPos[2];//二次快门位置
		double finishShutPos[2];//完成拍照位置
		double arcAngle1[2];//拍照前圆弧始末位值的相对偏角
		double arcAngle2[2];//拍照后圆弧始末位值的相对偏角
		
		if (currentPos[0] < motionPars.cameraPos[0])
		{	//从左边进入照相区

			//计算通过照相点的直线轨迹矢量与X轴的偏角，实际上就是两个吸嘴连线矢量与X轴的偏角
			cameraAngle = atan2(mechPars.nozzleRelPos2[1] - mechPars.nozzleRelPos1[1], mechPars.nozzleRelPos2[0] - mechPars.nozzleRelPos1[0]);
			//第一个开启快门位置点就是第一个吸嘴达到缓冲距离的位置
			firstShutPos[0]  = motionPars.cameraPos[0] - (shutDistance + nozzleOffset2) * cos(cameraAngle);
			firstShutPos[1]  = motionPars.cameraPos[1] - (shutDistance + nozzleOffset2) * sin(cameraAngle);
			//第二个开启快门位置点就是第二个吸嘴达到缓冲距离的位置
			secondShutPos[0] = motionPars.cameraPos[0] - (shutDistance - nozzleOffset1) * cos(cameraAngle);
			secondShutPos[1] = motionPars.cameraPos[1] - (shutDistance - nozzleOffset1) * sin(cameraAngle);
			//照相完成位置点是第二个吸嘴到达相机位置
			finishShutPos[0] = motionPars.cameraPos[0] + nozzleOffset1 * cos(cameraAngle);
			finishShutPos[1] = motionPars.cameraPos[1] + nozzleOffset1 * sin(cameraAngle);
			//第一段圆弧轨迹过后进入过相机的直线轨迹，因此第一段圆弧的末速度沿过相机直线轨迹矢量方向，那么第一段圆弧圆心到末尾点的向量角就知道了
			arcAngle1[1] = cameraAngle + M_PI_2;
			//第二段圆弧圆心到初始位置的向量角同理可得
			arcAngle2[0] = cameraAngle - M_PI_2;
		}
		else
		{	//从右边进入照相区
			cameraAngle = atan2(mechPars.nozzleRelPos1[1] - mechPars.nozzleRelPos2[1], mechPars.nozzleRelPos1[0] - mechPars.nozzleRelPos2[0]);
			firstShutPos[0]  = motionPars.cameraPos[0] - (shutDistance + nozzleOffset1) * cos(cameraAngle);
			firstShutPos[1]  = motionPars.cameraPos[1] - (shutDistance + nozzleOffset1) * sin(cameraAngle);
			secondShutPos[0] = motionPars.cameraPos[0] - (shutDistance - nozzleOffset2) * cos(cameraAngle);
			secondShutPos[1] = motionPars.cameraPos[1] - (shutDistance - nozzleOffset2) * sin(cameraAngle);
			finishShutPos[0] = motionPars.cameraPos[0] + nozzleOffset2 * cos(cameraAngle);
			finishShutPos[1] = motionPars.cameraPos[1] + nozzleOffset2 * sin(cameraAngle);
			arcAngle1[1] = cameraAngle - M_PI_2;
			arcAngle2[0] = cameraAngle + M_PI_2;
		}

		//计算两段圆弧的半径
		const double radius1(motionPars.cameraVel * motionPars.cameraVel * mechPars.radiusFactor[0]);
		const double radius2(motionPars.cameraVel * motionPars.cameraVel * mechPars.radiusFactor[1]);

		//计算两段圆弧的中心位置
		double arcCenter1[2];//拍照前的转向中心
		double arcCenter2[2];//拍照后的转向中心
		arcCenter1[0] =  firstShutPos[0] - radius1 * cos(arcAngle1[1]);
		arcCenter1[1] =  firstShutPos[1] - radius1 * sin(arcAngle1[1]);
		arcCenter2[0] = finishShutPos[0] - radius2 * cos(arcAngle2[0]);
		arcCenter2[1] = finishShutPos[1] - radius2 * sin(arcAngle2[0]);

		//计算当前点到圆弧1中心的距离
		const double distance1(sqrt((currentPos[0] - arcCenter1[0]) * (currentPos[0] - arcCenter1[0]) + (currentPos[1] - arcCenter1[1]) * (currentPos[1] - arcCenter1[1])));
		//计算目标点到圆弧2中心的距离
		const double distance2(sqrt((motionPars.endPos[0] - arcCenter2[0]) * (motionPars.endPos[0] - arcCenter2[0]) + (motionPars.endPos[1] - arcCenter2[1]) * (motionPars.endPos[1] - arcCenter2[1])));

		//判断圆弧设置得是否合理
		if (radius1 > distance1)
			return LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_ONE_FACTOR;
		if (radius2 > distance2)
			return LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_TWO_FACTOR;

		//计算当前点相对圆弧1初始位置的偏角绝对值
		const double current_center1Angle(acos(radius1 / distance1));
		//计算目标点相对圆弧2末位置的偏角绝对值
		const double end_center2Angle(acos(radius2 / distance2));

		//计算圆弧1中心到当前点的向量角
		double currentAngle(atan2(currentPos[1] - arcCenter1[1], currentPos[0] - arcCenter1[0]));
		if (currentAngle < 0)
			currentAngle = M_PI * 2 + currentAngle;
		//计算圆弧2中心到目标点的向量角
		double endAngle(atan2(motionPars.endPos[1] - arcCenter2[1], motionPars.endPos[0] - arcCenter2[0]));
		if (endAngle < 0)
			endAngle = M_PI * 2 + endAngle;

		//计算圆弧1中心到其初始位置的向量角
		arcAngle1[0] = arcAngle1[1] - currentAngle >= 0 ? currentAngle + current_center1Angle : currentAngle - current_center1Angle;
		//计算圆弧2中心到其末位置的向量角
		arcAngle2[1] = arcAngle2[0] - endAngle >= 0 ? endAngle + end_center2Angle : endAngle - end_center2Angle;

		double arcStartPos1[2];//拍照前转向开始位置
		double arcEndPos2[2];//拍照后转向结束位置
		arcStartPos1[0] = arcCenter1[0] + radius1 * cos(arcAngle1[0]);
		arcStartPos1[1] = arcCenter1[1] + radius1 * sin(arcAngle1[0]);
		arcEndPos2[0]	= arcCenter2[0] + radius2 * cos(arcAngle2[1]);
		arcEndPos2[1]	= arcCenter2[1] + radius2 * sin(arcAngle2[1]);

		TRAJ_SEG_LINE_PARS linePars;
		TRAJ_SEG_ARC_PARS arcPars;

		arcPars.maxVel  = linePars.maxVel  = motionPars.maxVel;
		arcPars.maxAcc  = linePars.maxAcc  = motionPars.maxAcc;
		arcPars.maxJerk = linePars.maxJerk = motionPars.maxJerk;

		linePars.startPos[0] = currentPos[0];
		linePars.startPos[1] = currentPos[1];
		linePars.endPos[0]	 = arcStartPos1[0];
		linePars.endPos[1]	 = arcStartPos1[1];
		linePars.startVel	 = 0.0;
		linePars.endVel	= motionPars.cameraVel;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicLine(linePars, mechPars);

		arcPars.startPos[0] = arcStartPos1[0];
		arcPars.startPos[1] = arcStartPos1[1];
		arcPars.endPos[0]	= firstShutPos[0];
		arcPars.endPos[1]	= firstShutPos[1];
		arcPars.center[0]   = arcCenter1[0];
		arcPars.center[1]   = arcCenter1[1];
		arcPars.angle		= arcAngle1[1] - arcAngle1[0];
		arcPars.startVel	= motionPars.cameraVel;
		arcPars.endVel		= motionPars.cameraVel;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicArc(arcPars, mechPars, 1);

		linePars.startPos[0] = firstShutPos[0];
		linePars.startPos[1] = firstShutPos[1];
		linePars.endPos[0]	 = secondShutPos[0];
		linePars.endPos[1]	 = secondShutPos[1];
		linePars.startVel	 = motionPars.cameraVel;
		linePars.endVel		 = motionPars.cameraVel;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicLine(linePars, mechPars, 2);

		linePars.startPos[0] = secondShutPos[0];
		linePars.startPos[1] = secondShutPos[1];
		linePars.endPos[0]	 = finishShutPos[0];
		linePars.endPos[1]	 = finishShutPos[1];
		linePars.startVel	 = motionPars.cameraVel;
		linePars.endVel		 = motionPars.cameraVel;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicLine(linePars, mechPars);

		arcPars.startPos[0] = finishShutPos[0];
		arcPars.startPos[1] = finishShutPos[1];
		arcPars.endPos[0]	= arcEndPos2[0];
		arcPars.endPos[1]	= arcEndPos2[1];
		arcPars.center[0]   = arcCenter2[0];
		arcPars.center[1]   = arcCenter2[1];
		arcPars.angle		= arcAngle2[1] - arcAngle2[0];
		arcPars.startVel	= motionPars.cameraVel;
		arcPars.endVel		= motionPars.cameraVel;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicArc(arcPars, mechPars);

		linePars.startPos[0] = arcEndPos2[0];
		linePars.startPos[1] = arcEndPos2[1];
		linePars.endPos[0]	 = motionPars.endPos[0];
		linePars.endPos[1]	 = motionPars.endPos[1];
		linePars.startVel	 = motionPars.cameraVel;
		linePars.endVel		 = 0.0;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : TrajSegmentCubicLine(linePars, mechPars);

		//测试输出
#ifdef _OUTPUT_DATAS_
		ofstream file1("..//SegmentDatas.txt");	
		file1<<"|Index|Distance|Velocity|"<<endl<<"|:-:|:-:|:-:|"<<endl;
		for (uint32_t i = 0; i < mechPars.usedNrOfCubPars; ++i)
		{
			file1<<"|"<<i<<"|"<<mechPars.segDistance[i]<<"|"<<mechPars.segVelocity[i]<<"|"<<endl;
		}
		file1.close();

		ofstream file2("..//SplineDatas.txt");	
		file2<<"|Index|posX|velX|posY|velY|timeX|timeY|IDX|IDY"<<endl<<"|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|"<<endl;
		for (uint32_t i = 0; i < mechPars.usedNrOfCubPars; ++i)
		{
			file2<<"|"<<i<<"|"
				<<mechPars.cubPars[0][i].position<<"|"<<mechPars.cubPars[0][i].velocity<<"|"
				<<mechPars.cubPars[1][i].position<<"|"<<mechPars.cubPars[1][i].velocity<<"|"
				<<mechPars.cubPars[0][i].time<<"|"<<mechPars.cubPars[1][i].time<<"|"
				<<mechPars.cubPars[0][i].splineId<<"|"<<mechPars.cubPars[1][i].splineId<<"|"<<endl;
		}
		file2.close();
#endif
		//测试输出

		//stream
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : stream(mechPars);                            
 
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : streamSync(mechPars, SAC_INDEFINITE);
	}						 

	return nyceStatus;
}

//获取状态代码
const char* LabelMechGetStatusString(NYCE_STATUS nyceStatus)
{
	switch (nyceStatus)
	{
	case LABEL_MACHINE_ERR_INIT_FAILED							  : 
		return "LABEL_MACHINE_ERR_INIT_FAILED";
	case LABEL_MACHINE_ERR_HAVE_NOT_INITED						  : 
		return "LABEL_MACHINE_ERR_HAVE_NOT_INITED";
	case LABEL_MACHINE_ERR_TERM_FAILED							  : 
		return "LABEL_MACHINE_ERR_TERM_FAILED";
	case LABEL_MACHINE_ERR_REINIT								  :
		return "LABEL_MACHINE_ERR_REINIT";
	case LABEL_MACHINE_ERR_NOZZLE_DEFLECTION_OUT_OF_RANGE		  : 
		return "LABEL_MACHINE_ERR_NOZZLE_DEFLECTION_OUT_OF_RANGE";
	case LABEL_MACHINE_ERR_MOTION_PARS							  : 
		return "LABEL_MACHINE_ERR_MOTION_PARS";
	case LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_JERK_EXCEEDED		  : 
		return "LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_JERK_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_ACCELERATION_EXCEEDED : 
		return "LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_ACCELERATION_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_VELOCITY_EXCEEDED	  : 
		return "LABEL_MACHINE_ERR_TRAJ_CURVE_X_MAX_VELOCITY_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_JERK_EXCEEDED		  : 
		return "LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_JERK_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_ACCELERATION_EXCEEDED : 
		return "LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_ACCELERATION_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_VELOCITY_EXCEEDED	  : 
		return "LABEL_MACHINE_ERR_TRAJ_CURVE_Y_MAX_VELOCITY_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_MAX_JERK_EXCEEDED				  : 
		return "LABEL_MACHINE_ERR_TRAJ_MAX_JERK_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_MAX_ACCELERATION_EXCEEDED		  : 
		return "LABEL_MACHINE_ERR_TRAJ_MAX_ACCELERATION_EXCEEDED";
	case LABEL_MACHINE_ERR_TRAJ_MAX_VELOCITY_EXCEEDED			  :
		return "LABEL_MACHINE_ERR_TRAJ_MAX_VELOCITY_EXCEEDED";
	case LABEL_MACHINE_ERR_START_POSITION_CHANGED				  : 
		return "LABEL_MACHINE_ERR_START_POSITION_CHANGED";
	case LABEL_MACHINE_ERR_STREAM_TIMEOUT						  : 
		return "LABEL_MACHINE_ERR_STREAM_TIMEOUT";
	case LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_ONE_FACTOR			  :
		return "LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_ONE_FACTOR";
	case LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_TWO_FACTOR			  : 
		return "LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_TWO_FACTOR";
	case LABEL_MACHINE_ERR_INIT_PARS_SPLINETIME					  : 
		return "LABEL_MACHINE_ERR_INIT_PARS_SPLINETIME";
	default:
		return NyceGetStatusString(nyceStatus);
	}
}