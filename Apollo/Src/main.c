/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "Communication.h"
#include "PathData.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//路径文件相关
extern PathPoint gPathPoints[PATH_PT_SIZE];
extern int gValidPathPtNum;//
extern int gCurPathId;

//APP通信相关  （LoRa USART1）
extern HEART_BEAT_DATA g_Heart_Beat;
extern uint8_t LORA_REGISTER[LORA_BUF_LEN];
extern volatile uint8_t LORA_REG_VALID;


//自动驾驶传感器相关  （RS232 USART2）
extern uint8_t RS232_REGISTER[RS232_BUF_LEN];
extern volatile uint8_t RS232_REG_VALID;
extern INM_Data gINMData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void lcdshow(char*s);
void lcdshowcmd(CmdType cmd);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#ifdef __GNUC__  
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf 
   set to 'Yes') calls __io_putchar() */  
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
#else  
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  
#endif /* __GNUC__ */  
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	char lcdtext[100];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

	printf("new program: apollo 6\n");

	//【1】初始化。。。
	initCan1();
	initPathPointsData();
	
	LCD_Init();                     //LCD初始化
	POINT_COLOR=RED; 
	//BACK_COLOR=GRAY;
	
	LCD_Clear(GREEN); 
	//LCD_ShowString(10,40,260,32*2,12,"Apollo STM32F4/F7"); 
	lcdshow("AgriX Project!!!\n");
	//【2】测试。。。
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(10);
		
		
		//【3】 处理APP通信
		CmdType cmd=CMD_HEARTBEAT;
		if(receiveLoRaCmd(&cmd))
		{
			switch(cmd)
			{
				case CMD_HEARTBEAT:
					//printf("cmd bh\n");
					break;
				case CMD_AUTO:
					startReceiveBleFile();
					//printf("cmd auto\n");
					break;
				case CMD_BLE_END:
					stopReceiveBleFile();
				
					printf("gValidPathPtNum=%d\n\n",gValidPathPtNum);
					for(int i=0;i<gValidPathPtNum;i++)
					{
						//printf("path%d x=%f\n",i,gPathPoints[i].startPt[0]);
						sprintf(lcdtext,"path%d x=%f\n",i,gPathPoints[i].startPt[0]);
						lcdshow(lcdtext);
						HAL_Delay(1);
					}
					printf("gValidPathPtNum=%d\n\n",gValidPathPtNum);
					
					break;
				case CMD_MANUAL:
					SendSpeed(0,0);
					ChangeDriverMode(DRIVER_MODE_EMERGENCY);
					break;
				case CMD_STOP:
					ChangeDriverMode(DRIVER_MODE_EMERGENCY);
				break;
				default:
					break;
			}
			
			lcdshowcmd(cmd);
			ackApp(cmd,g_Heart_Beat);//回复app轮询
		}
		
		
		
		//【接受组合导航模块定位数据】
		receiveINMData();
		float posex=0,posey=0,posez=0;
		cvtGpsPt2Xyz(&gINMData.longitude,&gINMData.latitude,&gINMData.altitude,&posex,&posey,&posez);
		
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/** 
  * @brief  Retargets the C library printf function to the USART. 
  * @param  None 
  * @retval None 
  */  
PUTCHAR_PROTOTYPE  
{  
  /* Place your implementation of fputc here */  
  /* e.g. write a character to the USART1 and Loop until the end of transmission */  
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);  
  return ch;  
} 

void lcdshow(char*s)
{
	LCD_ShowString(10,20,260,32*2,16,(u8*)s); 
}
void lcdshowcmd(CmdType cmd)
{
	char ss[50];
	switch(cmd)
	{
		case CMD_AUTO:
			sprintf(ss,"CMD_AUTO______\n");
			break;
		case CMD_MANUAL:
			sprintf(ss,"CMD_MANUAL____\n");
			break;
		case CMD_HEARTBEAT:
			sprintf(ss,"CMD_HEARTBEAT_\n");
			break;
		case CMD_BLE_END:
			sprintf(ss,"CMD_BLE_END___\n");
			break;
		case CMD_STOP:
			sprintf(ss,"CMD_STOP______\n");
			break;
		case CMD_SUPPLY:
			sprintf(ss,"CMD_SUPPLY____\n");
			break;
		case CMD_TRANSITION:
			sprintf(ss,"CMD_TRANSITION\n");
			break;
		default:
			sprintf(ss,"CMD_NO________\n");
			break;
	}

	LCD_ShowString(10,20,260,32,16,(u8*)ss);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
