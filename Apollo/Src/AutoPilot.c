#include "AutoPilot.h"
#include "PathData.h"
#include "stdlib.h"
#include "CommonAlg.h"
#include "PathFollower.h"
#include "stm32f4xx_hal.h"
#include "stmflash.h"
#include "math.h"
#include "lcd_show.h"

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
	//��debug��
	static int tt=0;
	static uint32_t cycle_time_start=0;
	static char debug_text[50];
	if(tt++%50==0)
	{
		uint32_t cycle_time=HAL_GetTick()-cycle_time_start;
		cycle_time_start=HAL_GetTick();
		int curpathid=getCurPathId();
		sprintf(debug_text,"cycle_t=%d ms,curPathId=%d ,PilotErr=%d \n",cycle_time,curpathid,GetPilotErr());
		lcdshow(debug_text);
		lcdshowpilotstate(gPilotState);
		
		int pptnum=getPathPointNum();
		float xx=0,yy=0;
		if(pptnum>0)
		{
			xx=getPathPoint(pptnum-1).startPt[0];
			yy=getPathPoint(pptnum-1).startPt[1];
		}
		lcdshowBLEdata(isBleDoing(),getPathPointNum(),xx,yy);
	}
	
	//��APPͨѶ��
	CmdType cmd=CMD_NONE;
	static uint32_t app_cmd_delay=0;
	static uint32_t app_cmd_wait_time=0;
	static uint8_t jumpstatemachine=0;
	if(receiveAPPCmd(&cmd))//����appָ��
	{
		ackApp(cmd,gHeartBeat);//�ظ�app��ѯ
		app_cmd_delay=HAL_GetTick();
		jumpstatemachine=0;
		
		if(cmd==CMD_WAIT)
			app_cmd_wait_time=10000;
		else
			app_cmd_wait_time=0;
		
		//debug
		lcdshowcmd(cmd);
	}
	else if(gPilotState==PILOT_STATE_INIT||gPilotState==PILOT_STATE_IDLE||gPilotState==PILOT_STATE_BLE_TRANSFER)
	{
		app_cmd_delay=HAL_GetTick();
	}
	else
	{
		uint32_t abort_time=HAL_GetTick()-app_cmd_delay;
		if(abort_time>3000+app_cmd_wait_time&&abort_time<10000+app_cmd_wait_time)//ͨѶ�ж�3s~10s
		{
			SetSpeed(0,0);
			HAL_Delay(10);
			jumpstatemachine=1;
			//return;
		}
		else if(abort_time>=10000+app_cmd_wait_time)
		{
			gPilotState=PILOT_STATE_INIT;
		}
	}
	
	//��������ϵ���ģ�����ݡ�
	if(receiveINMData())
	{
		if(isPathDataFileExist())
		{
			float poseX,poseY,poseYaw;
			INM_Data inm_data=getINMData();
			inm_data.roll=getIMUData().roll;
			inm_data.pitch=getIMUData().pitch;
			cvtINMData2Pose(inm_data,&poseX,&poseY,&poseYaw);
			
			if(getINMData().rtk_state==RTK_FIX)
				SetPose(poseX,poseY,poseYaw);
			
			double longitude,latitude;
			cvtXyPt2Gps(poseX,poseY,&longitude,&latitude);
			setHBPose(longitude,latitude,poseYaw);
		}
		else
		{
			setHBPose((double)getINMData().longitude/INM_LON_LAT_SCALE,(double)getINMData().latitude/INM_LON_LAT_SCALE,getINMData().yaw);
		}
		
		setHBRtkState(getINMData().rtk_state);
		
		lcdshowinmdata(getINMData());
	}
	else
	{
		ReckonPose();//ִ�к�λ�Ʋ�
	}
	
	//������IMU���ݡ�
	if(receiveIMUData())
	{
		lcdshowimudata(getIMUData());
	}
	
	//��CANͨѶ��
	HoldCanReceive();
	
	uint8_t tankLevel=0;
	static uint8_t tank_level_delay=0;
	if(GetTankLevel(&tankLevel))//���ˮ��ˮλ��������رշ�����
	{
		setHBTankLevel(tankLevel);
		lcdshowtanklevel(tankLevel);
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
		lcdshowbatteryvolt(voltage);
	}
	
	uint16_t servorAlarm=0;
	if(GetServorAlarm(&servorAlarm))//����ŷ���������������緢����������뼱ͣ״̬���ȴ������ϵ�
	{
		setHBServorAlarm(1);
		SetPilotErr(PILOT_ERR_MOTOR);
		gPilotState=PILOT_STATE_EMERGENCY;
	}
	
	TypeEngineMode engineMode;
	if(GetEngineMode(&engineMode))
	{
		lcdshowenginemode(engineMode);
	}
	
	TypeDriverMode driverMode;
	if(GetDriverMode(&driverMode))
	{
		lcdshowdrivermode(driverMode);
	}
	
	setHBEngineState(1);//todo:��ʵ�ַ��������
	
	//������״̬����
	if(jumpstatemachine==0)
	{
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
	}
	
	setHBFileExist(isPathDataFileExist());
	setHBPilotState(gPilotState);
	HAL_Delay(10);
}

/*
������е���ԭ���жϣ������б�Ҫ�Ļָ�����
*/
PilotState PilotInit(CmdType cmd)
{	
	HAL_Delay(20);
	initPathPointsData();//�����ļ���ʼ��Ϊ��
	stopReceiveBleFile();//��ʼ������ͨ��
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_AUTO);
	//SetDriverMode(DRIVER_MODE_MANUAL);
	
	gINMData.rtk_state=RTK_FAIL;
	
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
	//SetDriverMode(DRIVER_MODE_MANUAL);
	
	SetSpeed(0,0);
	HAL_Delay(10);
	return PILOT_STATE_IDLE;
}

PilotState PilotTransition(CmdType cmd)
{	
	if(cmd==CMD_SUPPLY)//APP�ֶ���ҵָ��
	{
		intoPilotSupply();
		return PILOT_STATE_SUPPLY;
	}
	
	HAL_Delay(10);
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
			updateBaseLocationByFile();//���»�վ����
			
			//��Ѱƥ�����
			float poseX,poseY,poseYaw;
			INM_Data inm_data=getINMData();
			inm_data.roll=getIMUData().roll;
			inm_data.pitch=getIMUData().pitch;
			cvtINMData2Pose(inm_data,&poseX,&poseY,&poseYaw);
			uint32_t ppt_num=getPathPointNum();
			uint32_t min_id=1;
			float min_dist=1000000;
			for(uint32_t i=1;i<ppt_num;i++)
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
	else if(cmd==CMD_BLE_ABORT)
	{
		stopReceiveBleFile();
		intoPilotTransition();
		return PILOT_STATE_TRANSITION;
	}
	else if(isBleDoing())
	{
		HAL_Delay(5);
	}
	HAL_Delay(10);
	
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
	
	if(HAL_GetTick()-GetPoseUpdateTime()>500)//�ж�rtk�Ƿ���
	{
		SetSpeed(0,0);
		if(HAL_GetTick()-GetPoseUpdateTime()>5000)//��ʱ5s����δ�̶�����رշ�����
		{
			SetEngineMode(ENGINE_MODE_STOP);
		}
		HAL_Delay(10);
		return PILOT_STATE_AUTO;
	}
	
	static uint32_t imu_over_turn_delay=0;
	if((abs(getIMUData().roll)>IMU_OVERTURN_ROLL)||(abs(getIMUData().pitch)>IMU_OVERTURN_PITCH))
	{
		if(imu_over_turn_delay==0)
			imu_over_turn_delay=HAL_GetTick();
		
		if(HAL_GetTick()-imu_over_turn_delay>1000)//�����˽��ӣ�
		{
			imu_over_turn_delay=0;
			
			//todo: ��APP��������ź�
			SetPilotErr(PILOT_ERR_FALL);
			
			intoPilotTransition();
			return PILOT_STATE_TRANSITION;
		}
	}
	else
	{
		imu_over_turn_delay=0;
	}
	
	//�Զ���ʻ
	PathPoint pathPoint;
	if(getCurPathPoint(&pathPoint)==0)//������ɣ�����
	{
		SetSpeed(0,0);
		//todo ��APP��������ź�
		SetPilotErr(PILOT_ERR_SUCCESS);
		
		intoPilotTransition();
		return PILOT_STATE_TRANSITION;
	}
	
	float Vc=0.0f,Wc=0.0f;
	CRS crs=runController(0.3,pathPoint,GetPose().poseX,GetPose().poseY,GetPose().poseYaw,&Vc,&Wc);//todo�ٶȱ仯���
	
	if(crs==CRS_REACHED)
	{
		moveCurPpt2Next();
		return PILOT_STATE_AUTO;
	}
	else if(crs==CRS_YERR||crs==CRS_XERR||crs==CRS_PHIERR)//�켣����ʧ��
	{
		//todo ��APP��������ź�
		SetPilotErr(PILOT_ERR_Y);
		intoPilotTransition();
		return PILOT_STATE_TRANSITION;
	}
	else
	{
		SetSpeed(Vc,Wc);
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

uint32_t gSupplyEngineStopTime=0;
uint8_t gSupplyEngineStartFlag=0;
PilotState PilotSupply(CmdType cmd)
{ 	
	if(cmd==CMD_AUTO)
	{
		SetSupplyState(0);//to check
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
	
	if(0)//todo:��⵽������ͣ����������������
	{
		SetEngineMode(ENGINE_MODE_START);
	}
	
	//todo:��ʱ�����������⹦��
	if(HAL_GetTick()-gSupplyEngineStopTime>10000&&gSupplyEngineStartFlag==0)
	{
		SetEngineMode(ENGINE_MODE_START);
		gSupplyEngineStartFlag=1;
	}
	
	HAL_Delay(10);
	return PILOT_STATE_SUPPLY;
}

void intoPilotTransition(void)
{
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
	initPathPointsData();//ɾ����ҵ�ļ�
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

void intoPilotSupply(void)
{
	//����ؼ����ݣ��Ա�����ص��硣����
	SetSupplyState(1);
	
	gSupplyEngineStopTime=HAL_GetTick();//��ʼ��������Ϩ���ʱ��
	gSupplyEngineStartFlag=0;
	
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
}



Speed gSpeed;
Speed GetSpeed()
{
	return gSpeed;
}

void SetSpeed(float v,float w)
{
	//�ٶ�ת��������
	double vl=0.0f,vr=0.0f;
	vr=v+(double)CAR_HALF_WIDTH*w;
	vl=v-(double)CAR_HALF_WIDTH*w;
	
	int16_t vl_cmd=0,vr_cmd=0;
	vl_cmd=-(int16_t)(vl*60.0/2.0/ALG_PI/CAR_WHEEL_RADIUS*16.0);
	vr_cmd=(int16_t)(vr*60.0/2.0/ALG_PI/CAR_WHEEL_RADIUS*16.0);
	
	//�ٶ�����
	float scale=1.0;
	if(abs(vl_cmd)>CAR_MOTOR_MAX_SPEED&&abs(vl_cmd)>abs(vr_cmd))
	{
		scale=(float)CAR_MOTOR_MAX_SPEED/(float)abs(vl_cmd);
		
	}
	else if(abs(vr_cmd)>CAR_MOTOR_MAX_SPEED&&abs(vr_cmd)>abs(vl_cmd))
	{
		scale=(float)CAR_MOTOR_MAX_SPEED/(float)abs(vr_cmd);
	}
	vl_cmd=scale*vl_cmd;
	vr_cmd=scale*vr_cmd;
	
	SendSpeed(vl_cmd,vr_cmd);
	
	//�ٶ�����ת��
	vl=(double)(-vl_cmd)/16.0*CAR_WHEEL_RADIUS*ALG_PI*2.0/60.0;
	vr=(double)vr_cmd/16.0*CAR_WHEEL_RADIUS*ALG_PI*2.0/60.0;
	gSpeed.v=(vl+vr)/2.0;
	gSpeed.w=(vr-vl)/(2.0*CAR_HALF_WIDTH);
}


Pose3D gPose;
Pose3D GetPose()
{
	return gPose;
}

uint8_t gIsPoseUpdate=0;
uint32_t gPoseUpdateTime=0;
uint32_t gPoseReckonTime=0;

void SetPose(float x,float y,float yaw)
{
	gPose.poseX=x;
	gPose.poseY=y;
	gPose.poseYaw=yaw;
	gIsPoseUpdate=1;
	gPoseUpdateTime=HAL_GetTick();
	gPoseReckonTime=gPoseUpdateTime;
}

uint32_t GetPoseUpdateTime()
{
	return gPoseUpdateTime;
}

void ReckonPose()
{
	uint32_t cur_time=HAL_GetTick();
	double dtime=(cur_time-gPoseReckonTime)*0.001;
	gPoseReckonTime=cur_time;
	
	gPose.poseX+=gSpeed.v*dtime*cos(gPose.poseYaw);
	gPose.poseY+=gSpeed.v*dtime*sin(gPose.poseYaw);
	gPose.poseYaw+=gSpeed.w*dtime;
}





void cvtINMData2Pose(INM_Data inm_data,float*pose_x,float*pose_y,float*pose_yaw)
{
	double longitude=(double)inm_data.longitude/INM_LON_LAT_SCALE;
	double latitude=(double)inm_data.latitude/INM_LON_LAT_SCALE;
	
	cvtGpsPt2Xy(longitude,latitude,pose_x,pose_y);
	
	float x0=-ANTEANA_DX,y0=-ANTEANA_DY,z0=-ANTEANA_DZ;
	
	RotateTheta4Point(&y0,&z0,inm_data.roll);
	
	RotateTheta4Point(&z0,&x0,inm_data.pitch);
	
	RotateTheta4Point(&x0,&y0,inm_data.yaw);
	
	(*pose_x)+=x0;
	(*pose_y)+=y0;
	
	(*pose_yaw)=inm_data.yaw;
}

PilotErr gPilotErr;
void SetPilotErr(PilotErr err)
{
	gPilotErr=err;
}
PilotErr GetPilotErr()
{
	return gPilotErr;
}




