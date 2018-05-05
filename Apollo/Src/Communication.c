#include "Communication.h"
#include "can.h"
#include "usart.h"
#include "string.h"
#include "PathData.h"
#include "stdlib.h"
#include "CommonAlg.h"


// 【APP通信】
uint8_t lora_buf[LORA_BUF_LEN+10];
uint8_t lora_buf_tmp[LORA_BUF_LEN+10];	
volatile uint8_t g_lora_data_new=0;

uint8_t LORA_REGISTER[LORA_BUF_LEN];
volatile uint8_t LORA_REG_VALID=0;

HEART_BEAT_DATA gHeartBeat={0.0f,0.0f,0.0f,0,0,0,0};
uint8_t gAppAck[APP_ACK_LEN];

void setHBPose(double longitude,double latitude,float posePhi)
{
//	while(longitude>ALG_2_PI){longitude-=ALG_2_PI;}
//	while(longitude<-ALG_2_PI){longitude+=ALG_2_PI;}
//	while(latitude>ALG_2_PI){latitude-=ALG_2_PI;}
//	while(latitude<-ALG_2_PI){latitude+=ALG_2_PI;}
	while(posePhi>ALG_2_PI){posePhi-=ALG_2_PI;}
	while(posePhi<-ALG_2_PI){posePhi+=ALG_2_PI;}
	
	gHeartBeat.poseLongitude=(int32_t)(longitude*INM_LON_LAT_SCALE);
	gHeartBeat.poseLatitude=(int32_t)(latitude*INM_LON_LAT_SCALE);//单位cm

	gHeartBeat.posePhi=(int16_t)(posePhi/0.001f);//单位毫弧度
}

void setHBTankLevel(uint8_t level)
{
	gHeartBeat.tankLevel=level;
}

void setHBBatteryPercentage(uint16_t volt)
{
	float percentage=100.0f*(((float)volt/100.0f-(3.7f*12.0f))/(4.2f*12.0f-3.7f*12.0f));
	if(percentage>0)
		gHeartBeat.batteryPercentage=(uint8_t)percentage;
	else 
		gHeartBeat.batteryPercentage=0;
}

void setHBPilotState(uint8_t state)
{
	gHeartBeat.curState=state;
}

void setHBEngineState(uint8_t engine_state)
{
	if(engine_state)
		gHeartBeat.curBitsState|=(uint8_t)(0x01);
	else
		gHeartBeat.curBitsState&=(uint8_t)(~(0x01));
}


void setHBFileExist(uint8_t file_exist)
{
	if(file_exist)
		gHeartBeat.curBitsState|=(uint8_t)(0x01<<1);
	else
		gHeartBeat.curBitsState&=(uint8_t)(~(0x01<<1));
}

void setHBRtkState(uint8_t rtk_state)
{
	if(rtk_state>3)
		rtk_state=3;
	
	gHeartBeat.curBitsState&=(uint8_t)(~(0x03<<2));
	gHeartBeat.curBitsState|=(uint8_t)((uint8_t)rtk_state<<2);
}
void setHBServorAlarm(uint8_t servor_alarm)
{
	if(servor_alarm)
		gHeartBeat.curBitsState|=(uint8_t)(0x01<<4);
	else
		gHeartBeat.curBitsState&=(uint8_t)(~(0x01<<4));
}

void assemAppAck(CmdType cmd,HEART_BEAT_DATA heartBeat)
{
	uint8_t*ptr=gAppAck;
	ptr[0]=0xc3;
	ptr[1]=0x50;
	ptr[2]=0x00;//0x28;
	ptr+=3;
	
	ptr[0]=APP_ACK_SOF;
	ptr++;
	
	uint16_t id=ROBOT_ID;
	memcpy(ptr,(uint8_t*)&id,2);
	ptr+=2;
	
	ptr[0]=cmd;
	ptr++;
	
	int32_t poseLongitude=heartBeat.poseLongitude;
	memcpy(ptr,(uint8_t*)&poseLongitude,4);
	ptr+=4;
	
	int32_t poseLatitude=heartBeat.poseLatitude;
	memcpy(ptr,(uint8_t*)&poseLatitude,4);
	ptr+=4;
	
	int16_t posePhi=heartBeat.posePhi;
	memcpy(ptr,(uint8_t*)&posePhi,2);
	ptr+=2;
	
	ptr[0]=heartBeat.tankLevel;
	ptr[1]=heartBeat.batteryPercentage;
	ptr[2]=heartBeat.curState;
	ptr+=3;
	
	ptr[0]=heartBeat.curBitsState;
	ptr++;
	
	uint8_t crc=0;//todo:循环冗余校验待实现
	ptr[0]=crc;
}

void ackApp(CmdType cmd,HEART_BEAT_DATA heartBeat)
{
	assemAppAck(cmd,heartBeat);
	HAL_UART_Transmit(&huart1,(uint8_t*)&gAppAck,sizeof(gAppAck),100);//回复心跳包
}

void updateLoRaData(void)
{
	if (g_lora_data_new)
	{
		memcpy(lora_buf_tmp,lora_buf,sizeof(lora_buf_tmp));
		if(lora_buf_tmp[0]==START_OF_CMD)
		{					
			//printf("LoRa:%s\n",lora_buf_tmp);
			memcpy(LORA_REGISTER,lora_buf_tmp,sizeof(LORA_REGISTER));
			LORA_REG_VALID=1;
			byhhReceiveUsart(&huart1, lora_buf,LORA_BUF_LEN);
		}
		else
		{
			//printf("LoRa err!!!:%s\n",lora_buf_tmp);
			HAL_Delay(10);
			byhhReceiveUsart(&huart1, lora_buf,LORA_BUF_LEN);
		}
		g_lora_data_new=0;
	}
	else if(huart1.RxState==HAL_UART_STATE_READY)
	{
		//printf("restart usart1!!!\n");
		HAL_Delay(10);
		byhhReceiveUsart(&huart1, lora_buf,LORA_BUF_LEN);
		g_lora_data_new=0;
	}
}


uint8_t receiveAPPCmd(CmdType *cmd)
{
	updateLoRaData();
	if(LORA_REG_VALID==0)
		return 0;
	else
	{
		LORA_REG_VALID=0;
		(*cmd)=(CmdType)LORA_REGISTER[1];
		return 1;
	}
}







//【组合导航模块数据传输】
//RS232 USART2
uint8_t RS232_REGISTER[RS232_BUF_LEN];
volatile uint8_t RS232_REG_VALID=0;

uint8_t rs232_buf[RS232_BUF_LEN+10];
volatile uint8_t g_rs232_data_new=0;

INM_Data gINMData;

void updateRS232Data(void)
{
	if (g_rs232_data_new)
	{
		g_rs232_data_new=0;
		memcpy(RS232_REGISTER,rs232_buf,sizeof(RS232_REGISTER));
		
		if(RS232_REGISTER[0]==0x55)
		{		
			//HAL_UART_AbortReceive_IT(&huart2);
			HAL_StatusTypeDef ret= byhhReceiveUsart(&huart2, rs232_buf,RS232_BUF_LEN);	
//			printf("ret:%d \n\n",ret);
//			printf("RS232:\n");
			RS232_REG_VALID=1;	
		}
		else
		{
			//printf("RS232 err!:%s\n",RS232_REGISTER);
			HAL_Delay(10);
			byhhReceiveUsart(&huart2, rs232_buf,RS232_BUF_LEN);
		}
	}
	else if(huart2.RxState==HAL_UART_STATE_READY)
	{
		//printf("restart usart2!!!\n");
		HAL_Delay(10);
		g_rs232_data_new=0;
		byhhReceiveUsart(&huart2, rs232_buf,RS232_BUF_LEN);
	}
}




uint8_t receiveINMData(void)
{
	updateRS232Data();
	
	if(RS232_REG_VALID==0)
		return 0;
	else
	{
		RS232_REG_VALID=0;
		
		uint8_t* ptr=RS232_REGISTER;
		ptr++;
		
		gINMData.rtk_state=(TypeRtkState)ptr[0];
		ptr++;
		
		memcpy((uint8_t*)&gINMData.longitude,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.latitude,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.altitude,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.roll,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.pitch,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.yaw,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.gps_weeks,ptr,2);
		ptr+=2;
		
		memcpy((uint8_t*)&gINMData.gps_ms,ptr,4);
		ptr+=4;
		
		
		return 1;
	}
}

INM_Data getINMData(void)
{
	return gINMData;
}


//【IMU数据接收】 USART4
uint8_t IMU_REGISTER[IMU_BUF_LEN];
volatile uint8_t IMU_REG_VALID=0;

uint8_t imu_buf[IMU_BUF_LEN+10];
volatile uint8_t g_imu_data_new=0;

IMU_Data gIMUData;

void updateIMUData(void)
{
	if (g_imu_data_new)
	{
		g_imu_data_new=0;
		memcpy(IMU_REGISTER,imu_buf,sizeof(IMU_REGISTER));
		
		if(IMU_REGISTER[0]==0x55&&IMU_REGISTER[1]==0x53)
		{		
			HAL_StatusTypeDef ret= byhhReceiveUsart(&huart4, imu_buf,IMU_BUF_LEN);	
			IMU_REG_VALID=1;	
		}
		else
		{
			HAL_Delay(5);
			byhhReceiveUsart(&huart4, imu_buf,IMU_BUF_LEN);
		}
	}
	else if(huart4.RxState==HAL_UART_STATE_READY)
	{
		HAL_Delay(5);
		g_imu_data_new=0;
		byhhReceiveUsart(&huart4, imu_buf,IMU_BUF_LEN);
	}
}




uint8_t receiveIMUData(void)
{
	updateIMUData();
	
	if(IMU_REG_VALID==0)
		return 0;
	else
	{
		IMU_REG_VALID=0;
		
		uint8_t* ptr=IMU_REGISTER;
		ptr+=2;
		
		int16_t cvt_data=0;
		memcpy(&cvt_data,ptr,2);
		gIMUData.roll=(float)((double)cvt_data/32768.0*ALG_PI);
		ptr+=2;
		
		memcpy(&cvt_data,ptr,2);
		gIMUData.pitch=(float)((double)cvt_data/32768.0*ALG_PI);
		ptr+=2;
		
		memcpy(&cvt_data,ptr,2);
		gIMUData.yaw=(float)((double)cvt_data/32768.0*ALG_PI);
		ptr+=2;
		
		return 1;
	}
}

IMU_Data getIMUData(void)
{
	return gIMUData;
}






//【CAN通信】
//CAN_FilterConfTypeDef		sFilterConfig;
//static CanTxMsgTypeDef	TxMessage;
static CanRxMsgTypeDef	RxMessage;


static CanTxMsgTypeDef	gSpeedTxMsg;
static CanTxMsgTypeDef	gDriverModeTxMsg;
static CanTxMsgTypeDef	gEngineModeTxMsg;

CAN_FilterConfTypeDef gCanReceiveFilter1;
CAN_FilterConfTypeDef gCanReceiveFilter2;

void initCan1(void)
{
	//1、初始化速度控制发送邮件
	gSpeedTxMsg.StdId=0x30;
	gSpeedTxMsg.RTR=CAN_RTR_DATA;
	gSpeedTxMsg.IDE=CAN_ID_STD;
	gSpeedTxMsg.DLC=4;
	for(int i=0;i<4;i++)
	{gSpeedTxMsg.Data[i]=0;}
	
	//2、初始化状态切换控制发送邮件
	gDriverModeTxMsg.StdId=0x10;
	gDriverModeTxMsg.RTR=CAN_RTR_DATA;
	gDriverModeTxMsg.IDE=CAN_ID_STD;
	gDriverModeTxMsg.DLC=1;
	gDriverModeTxMsg.Data[0]=100;
	
	//3、初始化发动机控制发送邮件
	gEngineModeTxMsg.StdId=0x20;
	gEngineModeTxMsg.RTR=CAN_RTR_DATA;
	gEngineModeTxMsg.IDE=CAN_ID_STD;
	gEngineModeTxMsg.DLC=1;
	gEngineModeTxMsg.Data[0]=100;
	
	
//  hcan1.pTxMsg = &TxMessage;
	hcan1.pRxMsg = &RxMessage;

	gCanReceiveFilter1.FilterNumber=1;
	gCanReceiveFilter1.FilterMode=CAN_FILTERMODE_IDLIST;
	gCanReceiveFilter1.FilterScale=CAN_FILTERSCALE_16BIT;
	gCanReceiveFilter1.FilterIdHigh=0x40<<5;
	gCanReceiveFilter1.FilterIdLow=0x50<<5;
	gCanReceiveFilter1.FilterMaskIdHigh=0x60<<5;
	gCanReceiveFilter1.FilterMaskIdLow=0x70<<5;
	gCanReceiveFilter1.FilterFIFOAssignment=CAN_FilterFIFO0;
	gCanReceiveFilter1.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &gCanReceiveFilter1);
	
	gCanReceiveFilter2.FilterNumber=2;
	gCanReceiveFilter2.FilterMode=CAN_FILTERMODE_IDLIST;
	gCanReceiveFilter2.FilterScale=CAN_FILTERSCALE_16BIT;
	gCanReceiveFilter2.FilterIdHigh=0x80<<5;
	gCanReceiveFilter2.FilterIdLow=0xC0<<5;
	gCanReceiveFilter2.FilterMaskIdHigh=0xA0<<5;
	gCanReceiveFilter2.FilterMaskIdLow=0xB0<<5;
	gCanReceiveFilter2.FilterFIFOAssignment=CAN_FilterFIFO0;
	gCanReceiveFilter2.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &gCanReceiveFilter2);
	
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
}



void SendSpeed(int16_t vl,int16_t vr)
{
	
	memcpy((uint8_t *)&gSpeedTxMsg.Data[0],&vl,2);
	memcpy((uint8_t *)&gSpeedTxMsg.Data[2],&vr,2);
	
	hcan1.pTxMsg=&gSpeedTxMsg;
	HAL_CAN_Transmit(&hcan1,10);	
}


void SetDriverMode(TypeDriverMode mode)
{
	gDriverModeTxMsg.Data[0]=mode;
	
	hcan1.pTxMsg=&gDriverModeTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
}


void SetEngineMode(TypeEngineMode mode)
{
	gEngineModeTxMsg.Data[0]=mode;
	
	hcan1.pTxMsg=&gEngineModeTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
}


volatile TypeDriverMode gDriverMode=DRIVER_MODE_AUTO;
volatile uint8_t gDriverModeNew=0;
uint8_t GetDriverMode(TypeDriverMode *mode)
{
	*mode= gDriverMode;
	if(gDriverModeNew)
	{
		gDriverModeNew=0;
		return 1;
	}
	else
		return 0;
}


volatile TypeEngineMode gEngineMode=ENGINE_MODE_START;
volatile uint8_t gEngineModeNew=0;
uint8_t GetEngineMode(TypeEngineMode *mode)
{
	*mode= gEngineMode;
	if(gEngineModeNew)
	{
		gEngineModeNew=0;
		return 1;
	}
	else
		return 0;
}

volatile uint8_t gTankLevel=0;
volatile uint8_t gTankLevelNew=0;
uint8_t GetTankLevel(uint8_t *level)
{
	*level= gTankLevel;
	if(gTankLevelNew)
	{
		gTankLevelNew=0;
		return 1;
	}
	else
		return 0;
}

volatile uint16_t gBaterryVolt=0;
volatile uint8_t gBaterryVoltNew=0;
uint8_t GetBatteryVolt(uint16_t *voltage)
{
	*voltage= gBaterryVolt;
	if(gBaterryVoltNew)
	{
		gBaterryVoltNew=0;
		return 1;
	}
	else
		return 0;
}

volatile uint16_t gServorAlarm=0;
volatile uint8_t gServerAlarmNew=0;
uint8_t GetServorAlarm(uint16_t *alarm)
{
	*alarm= gServorAlarm;
	if(gServerAlarmNew)
	{
		gServerAlarmNew=0;
		return 1;
	}
	else
		return 0;
}

uint8_t gNeedRestartCanReceive=0;
void SetNeedRestartCanReceive(uint8_t need)
{
	gNeedRestartCanReceive=need;
}
uint8_t GetNeedRestartCanReceive()
{
	return gNeedRestartCanReceive;
}

void HoldCanReceive()
{
	if(GetNeedRestartCanReceive())
	{
		HAL_StatusTypeDef ret=HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
		if(ret!=HAL_OK)
			SetNeedRestartCanReceive(1);
		else
			SetNeedRestartCanReceive(0);
	}
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan->pRxMsg->StdId==0x40)
	{
		gDriverMode=(TypeDriverMode)hcan->pRxMsg->Data[0];
		gDriverModeNew=1;
	}
	else if(hcan->pRxMsg->StdId==0x50)
	{
		gEngineMode=(TypeEngineMode)hcan->pRxMsg->Data[0];
		gEngineModeNew=1;
	}
	else if(hcan->pRxMsg->StdId==0x60)
	{
		memcpy((uint8_t*)&gServorAlarm,hcan->pRxMsg->Data,2);
		gServerAlarmNew=1;
	}
	else if(hcan->pRxMsg->StdId==0x70)
	{
		gTankLevel=hcan->pRxMsg->Data[0];
		gTankLevelNew=1;
	}
	else if(hcan->pRxMsg->StdId==0x80)
	{
		memcpy((uint8_t*)&gBaterryVolt,hcan->pRxMsg->Data,2);
		gBaterryVoltNew=1;
	}
	
	HAL_StatusTypeDef ret=HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	if(ret!=HAL_OK)
		SetNeedRestartCanReceive(1);
	else
		SetNeedRestartCanReceive(0);
	
}

//【蓝牙文件传输】
uint8_t ble_buf[BLE_BUF_LEN+10];
uint8_t ble_byte=0;
uint8_t ble_bytes_count=0;
uint32_t ble_frame_start_time=0;

uint8_t gBleDoing=0;
void startReceiveBleFile(void )
{
	gBleDoing=1;
	//HAL_UART_Receive_IT(&huart3,ble_buf,BLE_BUF_LEN);
	byhhReceiveUsart(&huart3,&ble_byte,1);
}

void stopReceiveBleFile(void)
{
	HAL_UART_AbortReceive_IT(&huart3);
	gBleDoing=0;
	ble_bytes_count=0;
}
uint8_t isBleDoing(void)
{
	return gBleDoing;
}


//【串口回调函数】
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		if(ble_bytes_count==0)//接收到一帧数据中的第一个字节
		{
			ble_frame_start_time=HAL_GetTick();
			ble_buf[ble_bytes_count++]=ble_byte;
		}
		else if(HAL_GetTick()-ble_frame_start_time>=10)//超时，认为遇到了协议字节
		{
			ble_bytes_count=0;
			ble_frame_start_time=HAL_GetTick();
			ble_buf[ble_bytes_count++]=ble_byte;
		}
		else if(ble_bytes_count==BLE_BUF_LEN-1)//在限定时间内组成了一帧数据
		{
			ble_buf[ble_bytes_count]=ble_byte;
			PathPoint ppt=*(PathPoint*)ble_buf;
			addPathPoint(ppt);
			
			ble_bytes_count=0;//为接收下一帧数据做准备
		}
		else//正在接收一帧数据中...
		{
			ble_buf[ble_bytes_count++]=ble_byte;
		}
		
		startReceiveBleFile();
	}
	else if(huart->Instance==USART1)
	{
		g_lora_data_new=1;
	}
	else if(huart->Instance==USART2)
	{
		g_rs232_data_new=1;
	}
	else if(huart->Instance==UART4)
	{
		g_imu_data_new=1;
	}
}

HAL_StatusTypeDef byhhReceiveUsart(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	  /* Check that a Rx process is not already ongoing */ 
  if(huart->RxState == HAL_UART_STATE_READY)
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return HAL_ERROR;
    }
    
    /* Process Locked */
    __HAL_LOCK(huart);
    
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->RxXferCount = Size;
    
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    
    /* Process Unlocked */
    __HAL_UNLOCK(huart);
		
		//byhh modified 
		int8_t tmp=huart->Instance->DR;
        
    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Enable the UART Parity Error and Data Register not empty Interrupts */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY; 
  }
}





