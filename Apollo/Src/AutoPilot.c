#include "AutoPilot.h"
#include "PathData.h"
#include "stdlib.h"
#include "CommonAlg.h"
#include "PathFollower.h"
#include "stm32f4xx_hal.h"
#include "stmflash.h"
#include "math.h"

#define		ANTEANA_DX	(-0.061)
#define		ANTEANA_DY	(0)
#define		ANTEANA_DZ	(1.92925)

#define		IMU_OVERTURN_ROLL	(30.0/180.0*ALG_PI)
#define		IMU_OVERTURN_PITCH	(30.0/180.0*ALG_PI)

#define		TANK_LEVEL_THRESHOLD	10

PilotState gPilotState;

void InitAutoPilot(void)
{
	gPilotState=PILOT_STATE_INIT;
}

void RunPilot(void)
{
	//��APPͨѶ��
	CmdType cmd=CMD_NONE;
	static uint32_t app_cmd_delay=0;
	if(receiveAPPCmd(&cmd))//����appָ��
	{
		ackApp(cmd,gHeartBeat);//�ظ�app��ѯ
		app_cmd_delay=HAL_GetTick();
	}
	else if(gPilotState!=PILOT_STATE_INIT&&gPilotState!=PILOT_STATE_IDLE)
	{
		uint32_t abort_time=HAL_GetTick()-app_cmd_delay;
		if(abort_time>3000&&abort_time<10000)//ͨѶ�ж�3s~10s
		{
			SendSpeed(0,0);
			HAL_Delay(10);
			return;
		}
		else if(abort_time>=10000)
		{
			gPilotState=PILOT_STATE_INIT;
		}
	}
	
	//��������ϵ���ģ�����ݡ�
	if(receiveINMData())
	{
		float poseX,poseY,poseYaw;
		cvtINMData2Pose(getINMData(),&poseX,&poseY,&poseYaw);
		setHBPose(poseX,poseY,poseYaw);
		setHBRtkState(getINMData().rtk_state);
	}
	
	//��CANͨѶ��
	uint8_t tankLevel=0;
	static uint16_t tank_level_delay=0;
	if(GetTankLevel(&tankLevel))//���ˮ��ˮλ��������رշ�����
	{
		setHBTankLevel(tankLevel);
		if(tankLevel<TANK_LEVEL_THRESHOLD&&tank_level_delay==0)
		{
			SetEngineMode(ENGINE_MODE_STOP);
			tank_level_delay=1;
		}		
		else if(tankLevel>TANK_LEVEL_THRESHOLD)
		{
			tank_level_delay=0;
		}
	}
	
	uint16_t voltage=0;
	if(GetBatteryVolt(&voltage))//��ص�ص�ѹ
	{
		setHBBatteryPercentage(voltage);
	}
	
	uint16_t servorAlarm=0;
	if(GetServorAlarm(&servorAlarm))//����ŷ���������������緢����������뼱ͣ״̬���ȴ������ϵ�
	{
		setHBServorAlarm(1);
		gPilotState=PILOT_STATE_EMERGENCY;
	}
	
	setHBEngineState(1);//todo:��ʵ�ַ��������
	setHBPathId(0);//���Զ���ʻ״̬�»��ٴθ��µ�ǰ·��������
	
	
	//������״̬����
	switch (gPilotState)
	{
		case PILOT_STATE_INIT:
			gPilotState=PilotInit(cmd);
			break;
		case PILOT_STATE_IDLE:
			gPilotState=PilotIdle(cmd);
			break;
		case PILOT_STATE_TRANSITION:
			gPilotState=PilotTransition(cmd);
			break;
		case PILOT_STATE_AUTO:
			gPilotState=PilotAuto(cmd);
			break;
		case PILOT_STATE_MANUAL_WORK:
			gPilotState=PilotManualWork(cmd);
			break;
		case PILOT_STATE_SUPPLY:
			gPilotState=PilotSupply(cmd);
			break;
		case PILOT_STATE_BLE_TRANSFER:
			gPilotState=PilotBleTransfer(cmd);
			break;
		case PILOT_STATE_EMERGENCY:
			HAL_Delay(10);
			break;
		default:
			while(1);
			break;
	}
	
	setHBFileExist(isPathDataFileExist());
	setHBPilotState(gPilotState);
	HAL_Delay(5);
}

/*
������е���ԭ���жϣ������б�Ҫ�Ļָ�����
*/
PilotState PilotInit(CmdType cmd)
{	
	initPathPointsData();//�����ļ���ʼ��Ϊ��
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_AUTO);
	return PILOT_STATE_IDLE;//�������״̬
}


PilotState PilotIdle(CmdType cmd)
{
	if(cmd!=CMD_NONE)//APP���ӳɹ���
	{
		if(GetSupplyState())//���紦��
		{
			intoPilotSupply();
			return PILOT_STATE_SUPPLY;
		}
		
		intoPilotTransition();//����ת��ģʽ
		return PILOT_STATE_TRANSITION;
	}
	
	HAL_Delay(10);
	return PILOT_STATE_IDLE;
}

PilotState PilotTransition(CmdType cmd)
{
	if(cmd==CMD_MANUAL)//APP�ֶ���ҵָ��
	{
		intoPilotManualWork();
		return PILOT_STATE_MANUAL_WORK;
	}
	
	if(cmd==CMD_AUTO)
	{
		return PILOT_STATE_BLE_TRANSFER;
	}
	
	HAL_Delay(10);
	return PILOT_STATE_TRANSITION;
	
	

	
	return PILOT_STATE_TRANSITION;
}

PilotState PilotBleTransfer(CmdType cmd)
{
	if(isPathDataFileExist())//·���ļ�����
	{
		intoPilotAuto();
		return PILOT_STATE_AUTO;
	}
	
	if((cmd==CMD_BLE_START)&&(!isBleDoing()))
	{
		startReceiveBleFile();
	}
	else if((cmd==CMD_BLE_END)&&(isBleDoing()))
	{
		stopReceiveBleFile();
		
		if(1)//todo:У���������������·���ļ��Ƿ���ȷ����
		{
			setPathDataFileExist();
			
			//��Ѱƥ�����
			float poseX,poseY,poseYaw;
			cvtINMData2Pose(getINMData(),&poseX,&poseY,&poseYaw);
			uint32_t ppt_num=getPathPointNum();
			uint32_t min_id=0;
			float min_dist=1000000;
			for(uint32_t i=0;i<ppt_num;i++)
			{
				PathPoint ppt=getPathPoint(i);
				float dist=sqrt(pow(ppt.startPt[0]-poseX,2)+pow(ppt.startPt[1]-poseY,2));
				if(dist<min_dist)
				{
					min_dist=dist;
					min_id=i;
				}
			}
			setCurPathPointId(min_id);
			
			intoPilotAuto();
			return PILOT_STATE_AUTO;
		}
		else
		{
			initPathPointsData();//���³�ʼ��·���ļ�Ϊ��
			intoPilotTransition();
			return PILOT_STATE_TRANSITION;
		}
	}
	else if(isBleDoing())
	{
		HAL_Delay(10);
	}
	
	return PILOT_STATE_BLE_TRANSFER;
}



PilotState PilotAuto(CmdType cmd)
{
	if(cmd==CMD_SUPPLY||(0))//todo:������ͣ������ж�
	{
		intoPilotSupply();
		return PILOT_STATE_SUPPLY;
	}
	else if(cmd==CMD_MANUAL)
	{
		intoPilotManualWork();//ת���ֶ���ҵģʽ
		return PILOT_STATE_MANUAL_WORK;
	}
	else if(cmd==CMD_TRANSITION)
	{
		intoPilotTransition();//ת���ֶ�ת��ģʽ
		return PILOT_STATE_TRANSITION;
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
		
		return PILOT_STATE_AUTO;
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
			SendSpeed(0,0);
			imu_over_turn_delay=0;
			
			//todo: ��APP��������ź�
			
			intoPilotTransition();
			return PILOT_STATE_TRANSITION;
		}
	}
	else if(imu_over_turn_delay>0)
	{
		imu_over_turn_delay--;
	}
	
	//�Զ���ʻ
	float poseX,poseY,poseYaw;
	cvtINMData2Pose(getINMData(),&poseX,&poseY,&poseYaw);
	
	setHBPathId(getCurPathPointId());
	
	PathPoint pathPoint;
	if(getCurPathPoint(&pathPoint)==0)//������ɣ�����
	{
		SendSpeed(0,0);
		//todo ��APP��������ź�
		intoPilotTransition();
		return PILOT_STATE_TRANSITION;
	}
	
	float Vc=0.0f,Wc=0.0f;
	CRS crs=runController(0.5,pathPoint,poseX,poseY,poseYaw,&Vc,&Wc);//todo�ٶȱ仯���
	
	if(crs==CRS_REACHED)
	{
		moveCurPpt2Next();
		return PILOT_STATE_AUTO;
	}
	else if(crs==CRS_YERR||crs==CRS_XERR||crs==CRS_PHIERR)//�켣����ʧ��
	{
		//todo ��APP��������ź�
		intoPilotTransition();
		return PILOT_STATE_TRANSITION;
	}
	else
	{
		SendSpeed(Vc,Wc);
		HAL_Delay(5);
		return PILOT_STATE_AUTO;
	}
}

PilotState PilotManualWork(CmdType cmd)
{
	if(cmd==CMD_SUPPLY||(0))//todo:������ͣ������ж�
	{
		intoPilotSupply();
		return PILOT_STATE_SUPPLY;
	}
	else if(cmd==CMD_AUTO)
	{
		return PILOT_STATE_BLE_TRANSFER;
	}
	else if(cmd==CMD_TRANSITION)
	{
		intoPilotTransition();//ת���ֶ�ת��ģʽ
		return PILOT_STATE_TRANSITION;
	}
	
	
	HAL_Delay(10);
	return PILOT_STATE_MANUAL_WORK;
}

uint16_t gEngineStopDelay=0;
PilotState PilotSupply(CmdType cmd)
{ 	
	if(cmd==CMD_AUTO)
	{
		SetSupplyState(0);
		return PILOT_STATE_BLE_TRANSFER;
	}
	else if(cmd==CMD_MANUAL)
	{
		SetSupplyState(0);
		intoPilotManualWork();//ת���ֶ���ҵģʽ
		return PILOT_STATE_MANUAL_WORK;
	}
	else if(cmd==CMD_TRANSITION)
	{
		SetSupplyState(0);
		intoPilotTransition();//ת���ֶ�ת��ģʽ
		return PILOT_STATE_TRANSITION;
	}
	
	if(0)//todo:��⵽������ͣ������������������
	{
		SetEngineMode(ENGINE_MODE_START);
	}
	
	//todo:��ʱ�����������⹦��
	
	if(gEngineStopDelay<1000)
	{
		gEngineStopDelay++;
	}
	else if(gEngineStopDelay==1000)
	{
		SetEngineMode(ENGINE_MODE_START);
		gEngineStopDelay++;
	}
	
	HAL_Delay(10);
	return PILOT_STATE_SUPPLY;
}

void intoPilotTransition(void)
{
	initPathPointsData();//���·���ļ�
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
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
	//����ؼ����ݣ��Ա�����ص��硣����
	SetSupplyState(1);
	
	gEngineStopDelay=0;//���㷢����Ϩ���ʱ��
	
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
}
void intoPilotBleTransfer(void)
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



