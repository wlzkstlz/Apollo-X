#include "Communication.h"
#include "can.h"
#include "usart.h"
#include "string.h"
#include "PathData.h"


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
uint8_t rs232_buf_tmp[RS232_BUF_LEN+10];
volatile uint8_t g_rs232_data_new=0;

INM_Data gINMData;

void updateRS232Data(void)
{
	if (g_rs232_data_new)
	{
		memcpy(rs232_buf_tmp,rs232_buf,sizeof(rs232_buf_tmp));
		if(rs232_buf_tmp[0]==0x55)
		{		
			byhhReceiveUsart(&huart2, rs232_buf,RS232_BUF_LEN);	
			
			printf("RS232:%s\n",rs232_buf_tmp);
			memcpy(RS232_REGISTER,rs232_buf_tmp,sizeof(RS232_REGISTER));
			RS232_REG_VALID=1;
			
		}
		else
		{
			printf("RS232 err!:%s\n",rs232_buf_tmp);
			HAL_Delay(10);
			byhhReceiveUsart(&huart2, rs232_buf,RS232_BUF_LEN);
		}
		g_rs232_data_new=0;
	}
	else if(huart2.RxState==HAL_UART_STATE_READY)
	{
		printf("restart usart2!!!\n");
		HAL_Delay(10);
		byhhReceiveUsart(&huart2, rs232_buf,RS232_BUF_LEN);
		g_rs232_data_new=0;
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
		
		memcpy((uint8_t*)&gINMData.longitude,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.latitude,ptr,4);
		ptr+=4;
		
		memcpy((uint8_t*)&gINMData.altitude,ptr,4);
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








//【CAN通信】
CAN_FilterConfTypeDef		sFilterConfig;
static CanTxMsgTypeDef	TxMessage;
static CanRxMsgTypeDef	RxMessage;


static CanTxMsgTypeDef	gSpeedTxMsg;
static CanTxMsgTypeDef	gDriverModeTxMsg;
static CanTxMsgTypeDef	gEngineModeTxMsg;

CAN_FilterConfTypeDef gFilterDriverMode;

void initCan1(void)
{
	//1、初始化速度控制发送邮件
	gSpeedTxMsg.StdId=0x10;
	gSpeedTxMsg.RTR=CAN_RTR_DATA;
	gSpeedTxMsg.IDE=CAN_ID_STD;
	gSpeedTxMsg.DLC=4;
	for(int i=0;i<4;i++)
	{gSpeedTxMsg.Data[i]=0;}
	
	//2、初始化状态切换控制发送邮件
	gDriverModeTxMsg.StdId=0x20;
	gDriverModeTxMsg.RTR=CAN_RTR_DATA;
	gDriverModeTxMsg.IDE=CAN_ID_STD;
	gDriverModeTxMsg.DLC=1;
	gDriverModeTxMsg.Data[0]=100;
	
	//3、初始化发动机控制发送邮件
	gEngineModeTxMsg.StdId=0x30;
	gEngineModeTxMsg.RTR=CAN_RTR_DATA;
	gEngineModeTxMsg.IDE=CAN_ID_STD;
	gEngineModeTxMsg.DLC=1;
	gEngineModeTxMsg.Data[0]=100;
	
	
  hcan1.pTxMsg = &TxMessage;
  hcan1.pRxMsg = &RxMessage;

  /*##-1- Configure CAN1 Transmission Massage #####################################*/
  hcan1.pTxMsg->StdId = 0x123;
  hcan1.pTxMsg->RTR = CAN_RTR_DATA;
  hcan1.pTxMsg->IDE = CAN_ID_STD;
  hcan1.pTxMsg->DLC = 8;
  hcan1.pTxMsg->Data[0] = 'C';
  hcan1.pTxMsg->Data[1] = 'A';
  hcan1.pTxMsg->Data[2] = 'N';
  hcan1.pTxMsg->Data[3] = ' ';
  hcan1.pTxMsg->Data[4] = 'T';
  hcan1.pTxMsg->Data[5] = 'e';
  hcan1.pTxMsg->Data[6] = 's';
  hcan1.pTxMsg->Data[7] = 't';

  /*##-2- Configure the CAN1 Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  //sFilterConfig.BankNumber = 14;//默认值就是14
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
	
	
	gFilterDriverMode.FilterNumber=1;
	gFilterDriverMode.FilterMode=CAN_FILTERMODE_IDLIST;
	gFilterDriverMode.FilterScale=CAN_FILTERSCALE_16BIT;
	gFilterDriverMode.FilterIdHigh=0X40;
	gFilterDriverMode.FilterIdLow=0X41;
	gFilterDriverMode.FilterMaskIdHigh=0X42;
	gFilterDriverMode.FilterMaskIdLow=0X43;
	gFilterDriverMode.FilterFIFOAssignment=CAN_FilterFIFO0;
	gFilterDriverMode.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &gFilterDriverMode);
	
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
}



void SendSpeed(float v,float w)
{
	int16_t vl=314,vr=628;
	
	//速度转换。。。
	
	memcpy((uint8_t *)&gSpeedTxMsg.Data[0],&vl,2);
	memcpy((uint8_t *)&gSpeedTxMsg.Data[2],&vr,2);
	
	hcan1.pTxMsg=&gSpeedTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	
}


void ChangeDriverMode(TypeDriverMode mode)
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


volatile TypeEngineMode gEngineMode=ENGINE_MODE_START;
TypeEngineMode GetEngineMode(void)
{
	return gEngineMode;
}



void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan->pRxMsg->StdId==0x40)
	{
		gEngineMode=(TypeEngineMode)hcan->pRxMsg->Data[0];
	}
	

	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
		
}

//【蓝牙文件传输】
uint8_t ble_buf[BLE_BUF_LEN+10];
void startReceiveBleFile(void )
{
	HAL_UART_Receive_IT(&huart3,ble_buf,BLE_BUF_LEN);
}

void stopReceiveBleFile(void)
{
	HAL_UART_AbortReceive_IT(&huart3);
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





