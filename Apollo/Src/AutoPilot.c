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
	//【debug】
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
	
	//【APP通讯】
	CmdType cmd=CMD_NONE;
	static uint32_t app_cmd_delay=0;
	static uint32_t app_cmd_wait_time=0;
	static uint8_t jumpstatemachine=0;
	if(receiveAPPCmd(&cmd))//接收app指令
	{
		ackApp(cmd,gHeartBeat);//回复app轮询
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
		if(abort_time>3000+app_cmd_wait_time&&abort_time<10000+app_cmd_wait_time)//通讯中断3s~10s
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
	
	//【接收组合导航模块数据】
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
		ReckonPose();//执行航位推测
	}
	
	//【接收IMU数据】
	if(receiveIMUData())
	{
		lcdshowimudata(getIMUData());
	}
	
	//【CAN通讯】
	HoldCanReceive();
	
	uint8_t tankLevel=0;
	static uint8_t tank_level_delay=0;
	if(GetTankLevel(&tankLevel))//监测水箱水位，视情况关闭发动机
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
	if(GetBatteryVolt(&voltage))//监控电池电压
	{
		setHBBatteryPercentage(voltage);
		lcdshowbatteryvolt(voltage);
	}
	
	uint16_t servorAlarm=0;
	if(GetServorAlarm(&servorAlarm))//监控伺服电机驱动警报，如发生警报则进入急停状态，等待重新上电
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
	
	setHBEngineState(1);//todo:待实现发动机检测
	
	//【运行状态机】
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
这里进行掉电原因判断，并进行必要的恢复工作
*/
PilotState PilotInit(CmdType cmd)
{	
	HAL_Delay(20);
	initPathPointsData();//任务文件初始化为空
	stopReceiveBleFile();//初始化蓝牙通信
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_AUTO);
	//SetDriverMode(DRIVER_MODE_MANUAL);
	
	gINMData.rtk_state=RTK_FAIL;
	
	return PILOT_STATE_IDLE;//进入空闲状态
}


PilotState PilotIdle(CmdType cmd)
{
	if(cmd!=CMD_NONE)//APP连接成功？
	{
		if(GetSupplyState())//掉电处理
		{
			intoPilotSupply();
			return PILOT_STATE_SUPPLY;
		}
		
		intoPilotTransition();//进入转场模式
		return PILOT_STATE_TRANSITION;
	}
	//SetDriverMode(DRIVER_MODE_MANUAL);
	
	SetSpeed(0,0);
	HAL_Delay(10);
	return PILOT_STATE_IDLE;
}

PilotState PilotTransition(CmdType cmd)
{	
	if(cmd==CMD_SUPPLY)//APP手动作业指令
	{
		intoPilotSupply();
		return PILOT_STATE_SUPPLY;
	}
	
	HAL_Delay(10);
	return PILOT_STATE_TRANSITION;
}

PilotState PilotBleTransfer(CmdType cmd)
{
	if(isPathDataFileExist())//路径文件存在
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
		
		if(1)//todo:校验蓝牙传输过来的路径文件是否正确无误
		{
			setPathDataFileExist();
			updateBaseLocationByFile();//更新基站坐标
			
			//搜寻匹配起点
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
			initPathPointsData();//重新初始化路径文件为空
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
	if(cmd==CMD_SUPPLY||(0))//todo:发动机停机检测判断
	{
		intoPilotSupply();
		return PILOT_STATE_SUPPLY;
	}
	else if(cmd==CMD_MANUAL)
	{
		intoPilotManualWork();//转入手动作业模式
		return PILOT_STATE_MANUAL_WORK;
	}
	else if(cmd==CMD_TRANSITION)
	{
		intoPilotTransition();//转入手动转场模式
		return PILOT_STATE_TRANSITION;
	}
	
	if(HAL_GetTick()-GetPoseUpdateTime()>500)//判断rtk是否丢星
	{
		SetSpeed(0,0);
		if(HAL_GetTick()-GetPoseUpdateTime()>5000)//延时5s后，仍未固定，则关闭发动机
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
		
		if(HAL_GetTick()-imu_over_turn_delay>1000)//机器人进坑？
		{
			imu_over_turn_delay=0;
			
			//todo: 向APP发送求救信号
			SetPilotErr(PILOT_ERR_FALL);
			
			intoPilotTransition();
			return PILOT_STATE_TRANSITION;
		}
	}
	else
	{
		imu_over_turn_delay=0;
	}
	
	//自动驾驶
	PathPoint pathPoint;
	if(getCurPathPoint(&pathPoint)==0)//任务完成！！！
	{
		SetSpeed(0,0);
		//todo 向APP发送完成信号
		SetPilotErr(PILOT_ERR_SUCCESS);
		
		intoPilotTransition();
		return PILOT_STATE_TRANSITION;
	}
	
	float Vc=0.0f,Wc=0.0f;
	CRS crs=runController(0.3,pathPoint,GetPose().poseX,GetPose().poseY,GetPose().poseYaw,&Vc,&Wc);//todo速度变化设计
	
	if(crs==CRS_REACHED)
	{
		moveCurPpt2Next();
		return PILOT_STATE_AUTO;
	}
	else if(crs==CRS_YERR||crs==CRS_XERR||crs==CRS_PHIERR)//轨迹跟踪失败
	{
		//todo 向APP发送求救信号
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
	if(cmd==CMD_SUPPLY||(0))//todo:发动机停机检测判断
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
		intoPilotTransition();//转入手动转场模式
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
		intoPilotManualWork();//转入手动作业模式
		return PILOT_STATE_MANUAL_WORK;
	}
	else if(cmd==CMD_TRANSITION)
	{
		SetSupplyState(0);
		intoPilotTransition();//转入手动转场模式
		return PILOT_STATE_TRANSITION;
	}
	
	if(0)//todo:检测到发动机停机后，允许发动机启动
	{
		SetEngineMode(ENGINE_MODE_START);
	}
	
	//todo:临时替代发动机检测功能
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
	initPathPointsData();//删除作业文件
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
	//保存关键数据，以备换电池掉电。。。
	SetSupplyState(1);
	
	gSupplyEngineStopTime=HAL_GetTick();//初始化发动机熄火计时器
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
	//速度转换。。。
	double vl=0.0f,vr=0.0f;
	vr=v+(double)CAR_HALF_WIDTH*w;
	vl=v-(double)CAR_HALF_WIDTH*w;
	
	int16_t vl_cmd=0,vr_cmd=0;
	vl_cmd=-(int16_t)(vl*60.0/2.0/ALG_PI/CAR_WHEEL_RADIUS*16.0);
	vr_cmd=(int16_t)(vr*60.0/2.0/ALG_PI/CAR_WHEEL_RADIUS*16.0);
	
	//速度限制
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
	
	//速度逆向转换
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




