#ifndef _AUTOPILOT_H
#define _AUTOPILOT_H

#include "Communication.h"
typedef enum
{
	PILOT_STATE_INIT,//��ʼ��
	PILOT_STATE_IDLE,//����
	PILOT_STATE_TRANSITION,//ת��
	PILOT_STATE_AUTO,//�Զ���ʻ
	PILOT_STATE_MANUAL_WORK,//�ֶ���ҵ6
	PILOT_STATE_SUPPLY,//����״̬
	PILOT_STATE_BLE_TRANSFER,//��������״̬
	PILOT_STATE_EMERGENCY//��ͣ״̬
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

//λ������
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

//�ٶ�����
typedef struct
{
	float v;
	float w;
}Speed;
Speed GetSpeed();
void SetSpeed(float v,float w);

//��λ�Ʋ�


void cvtINMData2Pose(INM_Data inm_data,float*pose_x,float*pose_y,float*pose_yaw);

#endif

