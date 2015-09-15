
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
	��ʼ������
	��Ҫ�����ǣ�
		1.������飬ʹ����ͬ���˶���
		2.�������SETPOINT_POS����velSet��ʹ��ȡλ��ʱ�����λ��������ͬһ��sample�����λ�õļ��㾫�ȡ�
		3.��������䶯�����ݣ��������պ���ָ�룩��ǰ���浽mechPars�������˶������Ĵ��������
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

	//���±����������û������޸�
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

//��ֹ����
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

//�˶�����
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
		const double shutDistance(motionPars.cameraVel * motionPars.shutterDelay);//���Ż������
		const double nozzleOffset1(sqrt(mechPars.nozzleRelPos1[0] * mechPars.nozzleRelPos1[0] + mechPars.nozzleRelPos1[1] * mechPars.nozzleRelPos1[1]));//����1ƫ�ƾ���
		const double nozzleOffset2(sqrt(mechPars.nozzleRelPos2[0] * mechPars.nozzleRelPos2[0] + mechPars.nozzleRelPos2[1] * mechPars.nozzleRelPos2[1]));//����2ƫ�ƾ���

		double cameraAngle(0.0);//������������������,ָ�����λ��
		double firstShutPos[2];//�״ο���λ��
		double secondShutPos[2];//���ο���λ��
		double finishShutPos[2];//�������λ��
		double arcAngle1[2];//����ǰԲ��ʼĩλֵ�����ƫ��
		double arcAngle2[2];//���պ�Բ��ʼĩλֵ�����ƫ��
		
		if (currentPos[0] < motionPars.cameraPos[0])
		{	//����߽���������

			//����ͨ��������ֱ�߹켣ʸ����X���ƫ�ǣ�ʵ���Ͼ���������������ʸ����X���ƫ��
			cameraAngle = atan2(mechPars.nozzleRelPos2[1] - mechPars.nozzleRelPos1[1], mechPars.nozzleRelPos2[0] - mechPars.nozzleRelPos1[0]);
			//��һ����������λ�õ���ǵ�һ������ﵽ��������λ��
			firstShutPos[0]  = motionPars.cameraPos[0] - (shutDistance + nozzleOffset2) * cos(cameraAngle);
			firstShutPos[1]  = motionPars.cameraPos[1] - (shutDistance + nozzleOffset2) * sin(cameraAngle);
			//�ڶ�����������λ�õ���ǵڶ�������ﵽ��������λ��
			secondShutPos[0] = motionPars.cameraPos[0] - (shutDistance - nozzleOffset1) * cos(cameraAngle);
			secondShutPos[1] = motionPars.cameraPos[1] - (shutDistance - nozzleOffset1) * sin(cameraAngle);
			//�������λ�õ��ǵڶ������쵽�����λ��
			finishShutPos[0] = motionPars.cameraPos[0] + nozzleOffset1 * cos(cameraAngle);
			finishShutPos[1] = motionPars.cameraPos[1] + nozzleOffset1 * sin(cameraAngle);
			//��һ��Բ���켣�������������ֱ�߹켣����˵�һ��Բ����ĩ�ٶ��ع����ֱ�߹켣ʸ��������ô��һ��Բ��Բ�ĵ�ĩβ��������Ǿ�֪����
			arcAngle1[1] = cameraAngle + M_PI_2;
			//�ڶ���Բ��Բ�ĵ���ʼλ�õ�������ͬ��ɵ�
			arcAngle2[0] = cameraAngle - M_PI_2;
		}
		else
		{	//���ұ߽���������
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

		//��������Բ���İ뾶
		const double radius1(motionPars.cameraVel * motionPars.cameraVel * mechPars.radiusFactor[0]);
		const double radius2(motionPars.cameraVel * motionPars.cameraVel * mechPars.radiusFactor[1]);

		//��������Բ��������λ��
		double arcCenter1[2];//����ǰ��ת������
		double arcCenter2[2];//���պ��ת������
		arcCenter1[0] =  firstShutPos[0] - radius1 * cos(arcAngle1[1]);
		arcCenter1[1] =  firstShutPos[1] - radius1 * sin(arcAngle1[1]);
		arcCenter2[0] = finishShutPos[0] - radius2 * cos(arcAngle2[0]);
		arcCenter2[1] = finishShutPos[1] - radius2 * sin(arcAngle2[0]);

		//���㵱ǰ�㵽Բ��1���ĵľ���
		const double distance1(sqrt((currentPos[0] - arcCenter1[0]) * (currentPos[0] - arcCenter1[0]) + (currentPos[1] - arcCenter1[1]) * (currentPos[1] - arcCenter1[1])));
		//����Ŀ��㵽Բ��2���ĵľ���
		const double distance2(sqrt((motionPars.endPos[0] - arcCenter2[0]) * (motionPars.endPos[0] - arcCenter2[0]) + (motionPars.endPos[1] - arcCenter2[1]) * (motionPars.endPos[1] - arcCenter2[1])));

		//�ж�Բ�����õ��Ƿ����
		if (radius1 > distance1)
			return LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_ONE_FACTOR;
		if (radius2 > distance2)
			return LABEL_MACHINE_ERR_TRAJ_PARS_RADIUS_TWO_FACTOR;

		//���㵱ǰ�����Բ��1��ʼλ�õ�ƫ�Ǿ���ֵ
		const double current_center1Angle(acos(radius1 / distance1));
		//����Ŀ������Բ��2ĩλ�õ�ƫ�Ǿ���ֵ
		const double end_center2Angle(acos(radius2 / distance2));

		//����Բ��1���ĵ���ǰ���������
		double currentAngle(atan2(currentPos[1] - arcCenter1[1], currentPos[0] - arcCenter1[0]));
		if (currentAngle < 0)
			currentAngle = M_PI * 2 + currentAngle;
		//����Բ��2���ĵ�Ŀ����������
		double endAngle(atan2(motionPars.endPos[1] - arcCenter2[1], motionPars.endPos[0] - arcCenter2[0]));
		if (endAngle < 0)
			endAngle = M_PI * 2 + endAngle;

		//����Բ��1���ĵ����ʼλ�õ�������
		arcAngle1[0] = arcAngle1[1] - currentAngle >= 0 ? currentAngle + current_center1Angle : currentAngle - current_center1Angle;
		//����Բ��2���ĵ���ĩλ�õ�������
		arcAngle2[1] = arcAngle2[0] - endAngle >= 0 ? endAngle + end_center2Angle : endAngle - end_center2Angle;

		double arcStartPos1[2];//����ǰת��ʼλ��
		double arcEndPos2[2];//���պ�ת�����λ��
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

		//�������
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
		//�������

		//stream
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : stream(mechPars);                            
 
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : streamSync(mechPars, SAC_INDEFINITE);
	}						 

	return nyceStatus;
}

//��ȡ״̬����
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