#include "Communication.h"
#include "can.h"
#include "usart.h"
#include "string.h"
#include "PathData.h"
#include "stdlib.h"


// 【APP通信】
uint8_t lora_buf[LORA_BUF_LEN+10];
uint8_t lora_buf_tmp[LORA_BUF_LEN+10];	
volatile uint8_t g_lora_data_new=0;

uint8_t LORA_REGISTER[LORA_BUF_LEN];
volatile uint8_t LORA_REG_VALID=0;

HEART_BEAT_DATA g_Heart_Beat={1,2,3,4,5,6,7,8};
uint8_t gAppAck[APP_ACK_LEN];

void assemAppAck(CmdType cmd,HEART_BEAT_DATA heartBeat)
{
	uint8_t*ptr=gAppAck;
	uint16_t id=ROBOT_ID;
	memcpy(ptr,(uint8_t*)&id,2);
	ptr+=2;
	
	uint8_t cmd_tmp=cmd;
	memcpy(ptr,&cmd_tmp,1);
	ptr++;
	
	int16_t poseX=heartBeat.poseX;
	memcpy(ptr,(uint8_t*)&poseX,2);
	ptr+=2;
	
	int16_t poseY=heartBeat.poseY;
	memcpy(ptr,(uint8_t*)&poseY,2);
	ptr+=2;
	
	int16_t posePhi=heartBeat.posePhi;
	memcpy(ptr,(uint8_t*)&posePhi,2);
	ptr+=2;
	
	ptr[0]=heartBeat.remainedLiq;
	ptr[1]=heartBeat.batteryVolt;
	ptr[2]=heartBeat.curState;
	ptr+=3;
	
	uint16_t pathid=heartBeat.curPathId;
	memcpy(ptr,(uint8_t*)&pathid,2);
	ptr+=2;
	
	ptr[0]=heartBeat.curBitsState;
	ptr[1]=0;
	
	ptr+=2;
	uint32_t crc=0;
	memcpy(ptr,(uint8_t*)&crc,4);
	
}

void ackApp(CmdType cmd,HEART_BEAT_DATA heartBeat)
{
	assemAppAck(cmd,heartBeat);
	HAL_UART_Transmit(&huart1,(uint8_t*)&gAppAck,sizeof(gAppAck),100);//回复心跳包，注意结构体的字节对齐
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


uint8_t receiveLoRaCmd(CmdType *cmd)
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
			printf("ret:%d \n\n",ret);
			printf("RS232:\n");
			RS232_REG_VALID=1;	
		}
		else
		{
			printf("RS232 err!:%s\n",RS232_REGISTER);
			HAL_Delay(10);
			byhhReceiveUsart(&huart2, rs232_buf,RS232_BUF_LEN);
		}
	}
	else if(huart2.RxState==HAL_UART_STATE_READY)
	{
		printf("restart usart2!!!\n");
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



void SendSpeed(float v,float w)
{
	//速度转换。。。
	float vl=0.0f,vr=0.0f;
	vr=v+(float)CAR_HALF_WIDTH*w;
	vl=v-(float)CAR_HALF_WIDTH*w;
	
	int16_t vl_cmd=0,vr_cmd=0;
	vl_cmd=-(int16_t)(vl*60.0f/2.0f/3.1415926f/CAR_WHEEL_RADIUS*16.0f);
	vr_cmd=(int16_t)(vr*60.0f/2.0f/3.1415926f/CAR_WHEEL_RADIUS*16.0f);
	
	//速度限制
	float scale=1.0;
	if(abs(vl_cmd)>CAR_MOTOR_MAX_SPEED&&abs(vl_cmd)>abs(vr_cmd))
	{
		scale=CAR_MOTOR_MAX_SPEED/abs(vl_cmd);
		
	}
	else if(abs(vr_cmd)>CAR_MOTOR_MAX_SPEED&&abs(vr_cmd)>abs(vl_cmd))
	{
		scale=CAR_MOTOR_MAX_SPEED/abs(vr_cmd);
	}
	vl_cmd=scale*vl_cmd;
	vr_cmd=scale*vr_cmd;
	
	printf("vl:%d  vr:%d\n",vl_cmd,vr_cmd);
	
	
	memcpy((uint8_t *)&gSpeedTxMsg.Data[0],&vl_cmd,2);
	memcpy((uint8_t *)&gSpeedTxMsg.Data[2],&vr_cmd,2);
	
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
	
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
		
}

//【蓝牙文件传输】
uint8_t ble_buf[BLE_BUF_LEN+10];
uint8_t gBleDoing=0;
void startReceiveBleFile(void )
{
	gBleDoing=1;
	HAL_UART_Receive_IT(&huart3,ble_buf,BLE_BUF_LEN);
}

void stopReceiveBleFile(void)
{
	HAL_UART_AbortReceive_IT(&huart3);
	gBleDoing=0;
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
		PathPoint ppt=*(PathPoint*)ble_buf;
		addPathPoint(ppt);
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





