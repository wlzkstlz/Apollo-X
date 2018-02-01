#ifndef _AUTOPILOT_H
#define _AUTOPILOT_H

#include "Communication.h"
typedef enum
{
	PILOT_STATE_INIT,//初始化
	PILOT_STATE_IDLE,//空闲
	PILOT_STATE_TRANSITION,//转场
	PILOT_STATE_AUTO,//自动驾驶
	PILOT_STATE_MANUAL_WORK,//手动作业
	PILOT_STATE_SUPPLY,//补给状态
	PILOT_STATE_BLE_TRANSFER,//蓝牙传输状态
	PILOT_STATE_EMERGENCY//急停状态
}PilotState;

extern PilotState gPilotState;

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

void cvtINMData2Pose(INM_Data inm_data,float*pose_x,float*pose_y,float*pose_yaw);

#endif

