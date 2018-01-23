#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "stm32f4xx_hal.h"

//【APP通信】
#define ROBOT_ID	0x0001
#define START_OF_CMD	0xaa

typedef enum 
{
	CMD_HEARTBEAT=0,
	CMD_AUTO,
	CMD_MANUAL,
	CMD_TRANSITION,
	CMD_SUPPLY,
	CMD_STOP,
	CMD_BLE_START,
	CMD_BLE_END
}CmdType;

typedef struct 
{
	int16_t poseX;
	int16_t poseY;
	int16_t posePhi;
	
	uint8_t remainedLiq;
	uint8_t batteryVolt;
	
	uint8_t curState;
	uint16_t curPathId;
	uint8_t curBitsState;
	
}HEART_BEAT_DATA;
extern HEART_BEAT_DATA g_Heart_Beat;

#define APP_ACK_LEN	20
extern uint8_t gAppAck[APP_ACK_LEN];

// LoRa USART1
#define LORA_BUF_LEN	3
extern uint8_t LORA_REGISTER[LORA_BUF_LEN];
extern volatile uint8_t LORA_REG_VALID;

void updateLoRaData(void);
uint8_t receiveLoRaCmd(CmdType *cmd);

void assemAppAck(CmdType cmd,HEART_BEAT_DATA heartBeat);
void ackApp(CmdType cmd,HEART_BEAT_DATA heartBeat);//回复APP轮询



//【组合导航模块传输数据】
typedef struct
{
	float longitude;
	float latitude;
	float altitude;
	float yaw;
	uint16_t gps_weeks;
	uint32_t gps_ms;
}INM_Data;
extern INM_Data gINMData;

#define RS232_BUF_LEN		25
extern uint8_t RS232_REGISTER[RS232_BUF_LEN];
extern volatile uint8_t RS232_REG_VALID;

void updateRS232Data(void);
uint8_t receiveINMData(void);



//【CAN通信】
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
void ChangeDriverMode(TypeDriverMode mode);
void SetEngineMode(TypeEngineMode mode);

uint8_t GetDriverMode(TypeDriverMode *mode);
uint8_t GetEngineMode(TypeEngineMode *mode);
uint8_t GetTankLevel(uint8_t *level);
uint8_t GetBatteryVolt(uint16_t *voltage);


//【蓝牙文件传输】
#define BLE_BUF_LEN	20
void updateBleData(void);
void startReceiveBleFile(void );
void stopReceiveBleFile(void);


//【串口通信相关】
HAL_StatusTypeDef byhhReceiveUsart(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
#endif
