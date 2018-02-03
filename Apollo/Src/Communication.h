#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "stm32f4xx_hal.h"

//��APPͨ�š�
#define ROBOT_ID	0x0001  //todo��������ID����Զ����䣿
#define START_OF_CMD	0xaa

typedef enum 
{
	CMD_HEARTBEAT=0,//����
	CMD_AUTO,//�Զ���ʻ
	CMD_MANUAL,//�ֶ���ʻ
	CMD_TRANSITION,//ת��
	CMD_SUPPLY,//����
	CMD_STOP,//��ͣ
	CMD_BLE_START,//��ʼ��������
	CMD_BLE_END,//������������
	CMD_BLE_ABORT,//�ж���������
	CMD_NONE,//
	CMD_WAIT//�������������������У��ȴ�����
}CmdType;

typedef struct 
{
	int32_t poseLongitude;//����С����ʽ��������*INM_LON_LAT_SCALE
	int32_t poseLatitude;//����С����ʽ��������*INM_LON_LAT_SCALE
	int16_t posePhi;
	
	uint8_t tankLevel;
	uint8_t batteryPercentage;
	
	uint8_t curState;
	uint8_t curBitsState;
	
}HEART_BEAT_DATA;
extern HEART_BEAT_DATA gHeartBeat;

void setHBPose(double longitude,double latitude,float posePhi);
void setHBTankLevel(uint8_t level);
void setHBBatteryPercentage(uint16_t volt);
void setHBPilotState(uint8_t state);
void setHBEngineState(uint8_t engine_state);
void setHBFileExist(uint8_t file_exist);
void setHBRtkState(uint8_t rtk_state);
void setHBServorAlarm(uint8_t servor_alarm);

#define APP_ACK_LEN	19	//sof1+id2+cmd1+pose10+tank1+volt1+state1+bitstate1+crc1=19
#define APP_ACK_SOF	0x55;
extern uint8_t gAppAck[APP_ACK_LEN];

// LoRa USART1
#define LORA_BUF_LEN	3
extern uint8_t LORA_REGISTER[LORA_BUF_LEN];
extern volatile uint8_t LORA_REG_VALID;

void updateLoRaData(void);
uint8_t receiveAPPCmd(CmdType *cmd);

void assemAppAck(CmdType cmd,HEART_BEAT_DATA heartBeat);
void ackApp(CmdType cmd,HEART_BEAT_DATA heartBeat);//�ظ�APP��ѯ



//����ϵ���ģ�鴫�����ݡ�
#define	INM_LON_LAT_SCALE	100000000.0 //��γ��ת���ɶ���С���ı���ϵ��
typedef enum
{
	RTK_FIX=0,
	RTK_FLOAT,
	RTK_SINGLE,
	RTK_FAIL
}TypeRtkState;
typedef struct
{
	TypeRtkState rtk_state;
	int32_t longitude;//����С����ʽ��������*INM_LON_LAT_SCALE
	int32_t latitude;//����С����ʽ��������*INM_LON_LAT_SCALE
	float altitude;
	float roll;
	float pitch;
	float yaw;
	uint16_t gps_weeks;
	uint32_t gps_ms;
}INM_Data;
extern INM_Data gINMData;

#define RS232_BUF_LEN		34	//(1+1+6*4+2+4+2)
extern uint8_t RS232_REGISTER[RS232_BUF_LEN];
extern volatile uint8_t RS232_REG_VALID;

void updateRS232Data(void);
uint8_t receiveINMData(void);
INM_Data getINMData(void);

//��CANͨ�š�
#define CAR_HALF_WIDTH	0.326f
#define CAR_WHEEL_RADIUS	0.1485f
#define	CAR_MOTOR_MAX_SPEED	1000
typedef enum
{
	DRIVER_MODE_AUTO,
	DRIVER_MODE_MANUAL,
	DRIVER_MODE_EMERGENCY
}TypeDriverMode;
typedef enum
{
	ENGINE_MODE_STOP,
	ENGINE_MODE_START
}TypeEngineMode;
void initCan1(void);

void SendSpeed(float v,float w);
void SetDriverMode(TypeDriverMode mode);
void SetEngineMode(TypeEngineMode mode);

uint8_t GetDriverMode(TypeDriverMode *mode);
uint8_t GetEngineMode(TypeEngineMode *mode);
uint8_t GetTankLevel(uint8_t *level);
uint8_t GetBatteryVolt(uint16_t *voltage);
uint8_t GetServorAlarm(uint16_t *alarm);


//�������ļ����䡿
#define BLE_BUF_LEN	20
void updateBleData(void);
void startReceiveBleFile(void );
void stopReceiveBleFile(void);
uint8_t isBleDoing(void);


//������ͨ����ء�
HAL_StatusTypeDef byhhReceiveUsart(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
#endif
