#include "AutoPilot.h"
#include "PathData.h"
#include "stdlib.h"
#include "CommonAlg.h"
#include "PathFollower.h"

#define		ANTEANA_DX	(-0.061)
#define		ANTEANA_DY	(0)
#define		ANTEANA_DZ	(1.92925)

#define		IMU_OVERTURN_ROLL	(30.0/180.0*3.1415926)
#define		IMU_OVERTURN_PITCH	(30.0/180.0*3.1415926)

PilotState gPilotState;

void InitAutoPilot(void)
{
	gPilotState=PILOT_STATE_INIT;
}

void RunPilot(void)
{
	CmdType cmd=CMD_NONE;
	if(receiveLoRaCmd(&cmd))//����appָ��
	{
		ackApp(cmd,g_Heart_Beat);//�ظ�app��ѯ
	}
	
	receiveINMData();//������ϵ���ģ������
	
	switch (gPilotState)
	{
		case PILOT_STATE_INIT:
			gPilotState=PilotInit(cmd);
			break;
		case PILOT_STATE_IDLE:
			gPilotState=PilotIdle(cmd);
			break;
		case PILOT_STATE_READY:
			gPilotState=PilotReady(cmd);
			break;
		case PILOT_STATE_AUTO:
			gPilotState=PilotAuto(cmd);
			break;
		case PILOT_STATE_MANUAL_WORK:
			gPilotState=PilotManualWork(cmd);
			break;
		case PILOT_STATE_MANUAL_TRANS:
			gPilotState=PilotManualTrans(cmd);
			break;
		case PILOT_STATE_SUPPLY:
			gPilotState=PilotSupply(cmd);
			break;
		case PILOT_STATE_STOP:
			gPilotState=PilotStop(cmd);
			break;
		default:
			while(1);
			break;
	}
}

/*
������е���ԭ���жϣ������б�Ҫ�Ļָ�����
*/
PilotState PilotInit(CmdType cmd)
{
	if(0)//��ʵ�ֵ��紦��
		return PILOT_STATE_SUPPLY;
	
	initPathPointsData();//�����ļ���ʼ��Ϊ��
	SetEngineMode(ENGINE_MODE_STOP);//��������ʼ��Ϊ��������
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_AUTO);//������ģʽ����Ϊ�Զ�����
	return PILOT_STATE_IDLE;//�������״̬
}


PilotState PilotIdle(CmdType cmd)
{
	if(cmd!=CMD_NONE)//APP���ӳɹ���
	{
		SetEngineMode(ENGINE_MODE_START);//��������ʼ��Ϊ��������
		HAL_Delay(1);
		SetDriverMode(DRIVER_MODE_MANUAL);//������ģʽ����Ϊ�ֶ�����
		return PILOT_STATE_READY;
	}
	
	HAL_Delay(10);
	return PILOT_STATE_IDLE;
}

PilotState PilotReady(CmdType cmd)
{
	if((cmd==CMD_BLE_START)&&(!isBleDoing()))
	{
		startReceiveBleFile();
	
	}
	else if((cmd==CMD_BLE_END)&&(isBleDoing()))
	{
		stopReceiveBleFile();
		
		if(1)//todo:У���������������·���ļ��Ƿ���ȷ����
		{
			intoPilotAuto();
			return PILOT_STATE_AUTO;
		}
		else
		{
			initPathPointsData();//���³�ʼ��·���ļ�Ϊ��
			return PILOT_STATE_READY;
		}
	}
	else if(isBleDoing())
	{
		HAL_Delay(10);
	}
	
	return PILOT_STATE_READY;
}



PilotState PilotAuto(CmdType cmd)
{
	if(cmd==CMD_MANUAL)
	{
		intoPilotManualWork();//ת���ֶ���ҵģʽ
		return PILOT_STATE_MANUAL_WORK;
	}
	else if(cmd==CMD_TRANSITION)
	{
		intoPilotManualTrans();//ת���ֶ�ת��ģʽ
		return PILOT_STATE_MANUAL_TRANS;
	}
	else if(cmd==CMD_SUPPLY||(0))//todo:������ͣ������ж�
	{
		intoPilotSupply();
		return PILOT_STATE_SUPPLY;
	}
	
	static uint16_t rtk_lost_delay=0;
	if(getINMData().rtk_state!=RTK_FIX)//�ж�rtk�Ƿ���
	{
		SendSpeed(0,0);
		HAL_Delay(10);
		if(rtk_lost_delay<500)
		{
			rtk_lost_delay++;
		}
		else if(rtk_lost_delay==500)//��ʱ5s����δ�̶�����رշ�����
		{
			SetEngineMode(ENGINE_MODE_STOP);
			rtk_lost_delay++;
		} 
	}
	else
	{
		rtk_lost_delay=0;
	}
	
	static uint16_t imu_over_turn_delay=0;
	if((abs(getINMData().roll)>IMU_OVERTURN_ROLL)||(abs(getINMData().pitch)>IMU_OVERTURN_PITCH))
	{
		if(++imu_over_turn_delay>100)//�����˽��ӣ�
		{
			imu_over_turn_delay=0;
			
			//todo ��APP��������ź�
			
			intoPilotManualTrans();
			return PILOT_STATE_MANUAL_TRANS;
		}
	}
	else if(imu_over_turn_delay>0)
		imu_over_turn_delay--;
	
	
	//�Զ���ʻ
	float poseX,poseY,poseYaw;
	cvtINMData2Pose(getINMData(),&poseX,&poseY,&poseYaw);
	
	PathPoint pathPoint;
	if(!getCurPathPoint(&pathPoint))//������ɣ�����
	{
		SendSpeed(0,0);
		//todo ��APP��������ź�
		intoPilotStop();
		return PILOT_STATE_STOP;
	}
	
	float Vc=0.0f,Wc=0.0f;
	CRS crs=runController(0.5,pathPoint,poseX,poseY,poseYaw,&Vc,&Wc);//todo�ٶȱ仯���
	
	if(crs==CRS_REACHED)
	{
		moveCurPpt2Next();
		return PILOT_STATE_AUTO;
	}
	else if(crs==CRS_YERR)//�켣����ʧ��
	{
		//todo ��APP��������ź�
		intoPilotManualTrans();
		return PILOT_STATE_MANUAL_TRANS;
	}
	else
	{
		SendSpeed(Vc,Wc);
		return PILOT_STATE_AUTO;
	}
}



PilotState PilotManualWork(CmdType cmd)
{
	
	return PILOT_STATE_IDLE;
}
PilotState PilotManualTrans(CmdType cmd)
{
	return PILOT_STATE_IDLE;
}
PilotState PilotSupply(CmdType cmd){return PILOT_STATE_IDLE;}
PilotState PilotStop(CmdType cmd)
{
	if(1)//todo �жϷ�������ͣ��
	{
		intoPilotReady();
		return PILOT_STATE_READY;
	}
	
	return PILOT_STATE_STOP;
}


void intoPilotReady(void)
{
	SetDriverMode(DRIVER_MODE_MANUAL);
	HAL_Delay(1);
	SetEngineMode(ENGINE_MODE_START);
}
void intoPilotAuto(void)
{
	SetEngineMode(ENGINE_MODE_START);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_AUTO);
}
void intoPilotManualWork(void)
{
	SetEngineMode(ENGINE_MODE_START);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
	initPathPointsData();//ɾ����ҵ�ļ�
}
void intoPilotManualTrans(void)
{
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
	initPathPointsData();//ɾ����ҵ�ļ�
}
void intoPilotSupply(void)
{
	//todo:����ؼ����ݣ��Ա�����ص��硣����
	
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
}
void intoPilotStop(void)
{
	SetEngineMode(ENGINE_MODE_STOP);//�رշ�����
	initPathPointsData();//ɾ����ҵ�ļ�
}


void cvtINMData2Pose(INM_Data inm_data,float*pose_x,float*pose_y,float*pose_yaw)
{
	cvtGpsPt2Xy(inm_data.longitude,inm_data.latitude,pose_x,pose_y);
	
	float x0=-ANTEANA_DX,y0=-ANTEANA_DY,z0=-ANTEANA_DZ;
	
	RotateTheta4Point(&x0,&y0,inm_data.yaw);
	
	RotateTheta4Point(&z0,&x0,inm_data.pitch);
	
	RotateTheta4Point(&y0,&z0,inm_data.roll);
	
	(*pose_x)+=x0;
	(*pose_y)+=y0;
	
	(*pose_yaw)=inm_data.yaw;
}




