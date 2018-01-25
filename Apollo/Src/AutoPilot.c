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
	if(receiveLoRaCmd(&cmd))//接收app指令
	{
		ackApp(cmd,g_Heart_Beat);//回复app轮询
	}
	
	receiveINMData();//接收组合导航模块数据
	
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
这里进行掉电原因判断，并进行必要的恢复工作
*/
PilotState PilotInit(CmdType cmd)
{
	if(0)//待实现掉电处理
		return PILOT_STATE_SUPPLY;
	
	initPathPointsData();//任务文件初始化为空
	SetEngineMode(ENGINE_MODE_STOP);//发动机初始化为不能启动
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_AUTO);//驱动板模式设置为自动控制
	return PILOT_STATE_IDLE;//进入空闲状态
}


PilotState PilotIdle(CmdType cmd)
{
	if(cmd!=CMD_NONE)//APP连接成功？
	{
		SetEngineMode(ENGINE_MODE_START);//发动机初始化为可以启动
		HAL_Delay(1);
		SetDriverMode(DRIVER_MODE_MANUAL);//驱动板模式设置为手动控制
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
		
		if(1)//todo:校验蓝牙传输过来的路径文件是否正确无误
		{
			intoPilotAuto();
			return PILOT_STATE_AUTO;
		}
		else
		{
			initPathPointsData();//重新初始化路径文件为空
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
		intoPilotManualWork();//转入手动作业模式
		return PILOT_STATE_MANUAL_WORK;
	}
	else if(cmd==CMD_TRANSITION)
	{
		intoPilotManualTrans();//转入手动转场模式
		return PILOT_STATE_MANUAL_TRANS;
	}
	else if(cmd==CMD_SUPPLY||(0))//todo:发动机停机检测判断
	{
		intoPilotSupply();
		return PILOT_STATE_SUPPLY;
	}
	
	static uint16_t rtk_lost_delay=0;
	if(getINMData().rtk_state!=RTK_FIX)//判断rtk是否丢星
	{
		SendSpeed(0,0);
		HAL_Delay(10);
		if(rtk_lost_delay<500)
		{
			rtk_lost_delay++;
		}
		else if(rtk_lost_delay==500)//延时5s后，仍未固定，则关闭发动机
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
		if(++imu_over_turn_delay>100)//机器人进坑？
		{
			imu_over_turn_delay=0;
			
			//todo 向APP发送求救信号
			
			intoPilotManualTrans();
			return PILOT_STATE_MANUAL_TRANS;
		}
	}
	else if(imu_over_turn_delay>0)
		imu_over_turn_delay--;
	
	
	//自动驾驶
	float poseX,poseY,poseYaw;
	cvtINMData2Pose(getINMData(),&poseX,&poseY,&poseYaw);
	
	PathPoint pathPoint;
	if(!getCurPathPoint(&pathPoint))//任务完成！！！
	{
		SendSpeed(0,0);
		//todo 向APP发送完成信号
		intoPilotStop();
		return PILOT_STATE_STOP;
	}
	
	float Vc=0.0f,Wc=0.0f;
	CRS crs=runController(0.5,pathPoint,poseX,poseY,poseYaw,&Vc,&Wc);//todo速度变化设计
	
	if(crs==CRS_REACHED)
	{
		moveCurPpt2Next();
		return PILOT_STATE_AUTO;
	}
	else if(crs==CRS_YERR)//轨迹跟踪失败
	{
		//todo 向APP发送求救信号
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
	if(1)//todo 判断发动机已停机
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
	initPathPointsData();//删除作业文件
}
void intoPilotManualTrans(void)
{
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
	initPathPointsData();//删除作业文件
}
void intoPilotSupply(void)
{
	//todo:保存关键数据，以备换电池掉电。。。
	
	SetEngineMode(ENGINE_MODE_STOP);
	HAL_Delay(1);
	SetDriverMode(DRIVER_MODE_MANUAL);
}
void intoPilotStop(void)
{
	SetEngineMode(ENGINE_MODE_STOP);//关闭发动机
	initPathPointsData();//删除作业文件
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




