#ifndef _AUTOPILOT_H
#define _AUTOPILOT_H

#include "Communication.h"
typedef enum
{
	PILOT_STATE_INIT,//初始化
	PILOT_STATE_IDLE,//空闲
	PILOT_STATE_TRANSITION,//转场
	PILOT_STATE_AUTO,//自动驾驶
	PILOT_STATE_MANUAL_WORK,//手动作业6
	PILOT_STATE_SUPPLY,//补给状态
	PILOT_STATE_BLE_TRANSFER,//蓝牙传输状态
	PILOT_STATE_EMERGENCY//急停状态
}PilotState;


extern PilotState gPilotState;

typedef enum
{
	PILOT_ERR_NONE,
	PILOT_ERR_Y,
	PILOT_ERR_MOTOR,
	PILOT_ERR_SUCCESS,
	PILOT_ERR_ENGINE_OFF,
	PILOT_ERR_FALL
}PilotErr;
extern PilotErr gPilotErr;
void SetPilotErr(PilotErr err);
PilotErr GetPilotErr();

void InitAutoPilot(void);

void RunPilot(void);

PilotState PilotInit(CmdType cmd);
PilotState PilotIdle(CmdType cmd);
PilotState PilotTransition(CmdType cmd);
PilotState PilotAuto(CmdType cmd);
PilotState PilotManualWork(CmdType cmd);
PilotState PilotSupply(CmdType cmd);
PilotState PilotBleTransfer(CmdType cmd);

void intoPilotTransition(void);
void intoPilotAuto(void);
void intoPilotManualWork(void);
void intoPilotSupply(void);

//位姿数据
typedef struct
{
	float poseX;
	float poseY;
	float poseYaw;
}Pose3D;
Pose3D GetPose();
void SetPose(float x,float y,float yaw);
void ReckonPose();
uint32_t GetPoseUpdateTime();
//void InitReckonPose();

//速度数据
typedef struct
{
	float v;
	float w;
}Speed;
Speed GetSpeed();
void SetSpeed(float v,float w);

//航位推测


void cvtINMData2Pose(INM_Data inm_data,float*pose_x,float*pose_y,float*pose_yaw);

#endif

