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
	if(tt++%100==0)
	{
		uint32_t cycle_time=HAL_GetTick()-cycle_time_start;
		cycle_time_start=HAL_GetTick();
		sprintf(debug_text,"cycle_t=%d ms  \n",cycle_time);
		lcdshow(debug_text);
		lcdshowpilotstate(gPilotState);
	}
	
//	lcdshowpilotstate(100);
//	lcdshowcmd(100);
//	lcdshowenginemode(100);
//	lcdshowdrivermode(100);
//	lcdshowtanklevel(200);
//	lcdshowbatteryvolt(1000);
//	
//	INM_Data inmdata;
//	inmdata.longitude=DEG2RAD(113.897329);
//	inmdata.latitude=DEG2RAD(22.960622);
//	inmdata.yaw=0.1;
//	inmdata.roll=0.2;
//	inmdata.pitch=0.3;
//	lcdshowinmdata(inmdata);
		
	
	//【APP通讯】
	CmdType cmd=CMD_NONE;
	static uint32_t app_cmd_delay=0;
	if(receiveAPPCmd(&cmd))//接收app指令
	{
		ackApp(cmd,gHeartBeat);//回复app轮询
		app_cmd_delay=HAL_GetTick();
		
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
		if(abort_time>3000&&abort_time<10000)//通讯中断3s~10s
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
	
	//【接收组合导航模块数据】
	if(receiveINMData())
	{
		float poseX,poseY,poseYaw;
		cvtINMData2Pose(getINMData(),&poseX,&poseY,&poseYaw);
		setHBPose(poseX,poseY,poseYaw);
		setHBRtkState(getINMData().rtk_state);
		
		lcdshowinmdata(getINMData());
	}
	
	//【CAN通讯】
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
	setHBPathId(0);//在自动驾驶状态下会再次更新当前路径点坐标
	
	
	//【运行状态机】
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
这里进行掉电原因判断，并进行必要的恢复工作
*/
PilotState PilotInit(CmdType cmd)
{	
	initPathPointsData();//任务文件初始化为空
	stopReceiveBleFile();//初始化蓝牙通信
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_AUTO);
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
			
			//搜寻匹配起点
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
	 
	
	static uint32_t rtk_lost_delay=0;
	if(getINMData().rtk_state!=RTK_FIX)//判断rtk是否丢星
	{
		SendSpeed(0,0);
		HAL_Delay(10);
		if(rtk_lost_delay==0)
		{
			rtk_lost_delay=HAL_GetTick();
		}
		else if(HAL_GetTick()-rtk_lost_delay>5000)//延时5s后，仍未固定，则关闭发动机
		{
			SetEngineMode(ENGINE_MODE_STOP);
			rtk_lost_delay=0;
		} 
		return PILOT_STATE_AUTO;
	}
	else
	{
		rtk_lost_delay=0;
	}
	
	static uint32_t imu_over_turn_delay=0;
	if((abs(getINMData().roll)>IMU_OVERTURN_ROLL)||(abs(getINMData().pitch)>IMU_OVERTURN_PITCH))
	{
		if(imu_over_turn_delay==0)
			imu_over_turn_delay=HAL_GetTick();
		
		if(HAL_GetTick()-imu_over_turn_delay>1000)//机器人进坑？
		{
			imu_over_turn_delay=0;
			
			//todo: 向APP发送求救信号
			
			intoPilotTransition();
			return PILOT_STATE_TRANSITION;
		}
	}
	else
	{
		imu_over_turn_delay=0;
	}
	
	//自动驾驶
	float poseX,poseY,poseYaw;
	cvtINMData2Pose(getINMData(),&poseX,&poseY,&poseYaw);
	
	setHBPathId(getCurPathPointId());
	
	PathPoint pathPoint;
	if(getCurPathPoint(&pathPoint)==0)//任务完成！！！
	{
		SendSpeed(0,0);
		//todo 向APP发送完成信号
		intoPilotTransition();
		return PILOT_STATE_TRANSITION;
	}
	
	float Vc=0.0f,Wc=0.0f;
	CRS crs=runController(0.5,pathPoint,poseX,poseY,poseYaw,&Vc,&Wc);//todo速度变化设计
	
	if(crs==CRS_REACHED)
	{
		moveCurPpt2Next();
		return PILOT_STATE_AUTO;
	}
	else if(crs==CRS_YERR||crs==CRS_XERR||crs==CRS_PHIERR)//轨迹跟踪失败
	{
		//todo 向APP发送求救信号
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
		SetSupplyState(0);
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




