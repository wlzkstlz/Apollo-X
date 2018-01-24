#ifndef _AUTOPILOT_H
#define _AUTOPILOT_H

#include "Communication.h"
typedef enum
{
	PILOT_STATE_INIT,
	PILOT_STATE_IDLE,
	PILOT_STATE_READY,
	PILOT_STATE_AUTO,
	PILOT_STATE_MANUAL_WORK,
	PILOT_STATE_MANUAL_TRANS,
	PILOT_STATE_SUPPLY,
	PILOT_STATE_STOP
}PilotState;

extern PilotState gPilotState;

void InitAutoPilot(void);

void RunPilot(void);

PilotState PilotInit(CmdType cmd);
PilotState PilotIdle(CmdType cmd);
PilotState PilotReady(CmdType cmd);
PilotState PilotAuto(CmdType cmd);
PilotState PilotManualWork(CmdType cmd);
PilotState PilotManualTrans(CmdType cmd);
PilotState PilotSupply(CmdType cmd);
PilotState PilotStop(CmdType cmd);


void intoPilotReady(void);
void intoPilotAuto(void);
void intoPilotManualWork(void);
void intoPilotManualTrans(void);
void intoPilotSupply(void);
void intoPilotStop(void);

void cvtINMData2Pose(INM_Data inm_data,float*pose_x,float*pose_y,float*pose_yaw);

#endif

