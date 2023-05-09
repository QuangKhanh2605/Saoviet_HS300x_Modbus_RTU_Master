/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/*

Add "HAL_SYSTICK_IRQHandler();" To "Systick_Handler" In "stm32L1xx_it.c"

*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "HS300x.h"
#include "user_uart.h"
#include "user_modbus_rtu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADDRESS_STM32 0X66
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t Time_Sampling = 3000;

uint32_t GetTick_Ms=0;
uint8_t check_GetTick_Ms=0;

int16_t Tem=0;
int16_t Humi=0;

int16_t Drop_Tem=0;
int16_t Drop_Humi=0;

uint8_t aTemperature[2];
uint8_t aHumidity[2];

uint16_t temRx=0;
uint16_t humiRx=0;

uint8_t address=0;

UART_BUFFER sUart2;
uint8_t check_address_slave = 0;
uint16_t T[10]={0};
uint16_t H[10]={0};

int16_t AVG_T[10]={0};
int16_t AVG_H[10]={0};

uint32_t getTick_transmit_uart=0;
uint8_t address_Tx=26;
uint32_t count=0;
char Tem_Humi[50];

float tem_stm=0.0;
float adc=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_SYSTICK_Callback(void);
uint8_t Modbus_RTU_Master(void);
void FLASH_Earse(uint32_t addr_ready_earse);
void FLASH_WriteNews(uint32_t addr_start_write,char News[]);
void display_uart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float avg_tem=0.0;
float tem_stm32=0.0;
uint16_t adcRead=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


	sUart2.huart = &huart2;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc);
	HAL_UART_Receive_IT(sUart2.huart,&sUart2.buffer,1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	//HS300X_Init(&hi2c1, TM, HU);
//	for(int i=1;i<128;i++)
//	{
//		if(HAL_I2C_IsDeviceReady(&hi2c2, i<<1, 5,5) == HAL_OK)
//		{
//			address=i;
//		}
//	
//	while(1)
//	{
//		avg_tem=0;
//		for(int i=0; i<5;i++)
//		{
//			HAL_ADC_PollForConversion(&hadc, 1000);
//			adcRead = HAL_ADC_GetValue(&hadc);
//			//adcRead = adcRead*3;
//			avg_tem +=  (float)(((0.6268-3.3/4096*adcRead)/0.00161)-44);
//			HAL_Delay(20);
//		}
//		tem_stm32 =  avg_tem/5;
//	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(GetTick_Ms>HAL_GetTick()) GetTick_Ms=0;
		if(HAL_GetTick()-GetTick_Ms>Time_Sampling) 
		{
//			HS300X_Start_Measurement(&hi2c2, (int16_t*)&Tem, (int16_t*)&Humi);
//			sprintf(Tem_Humi,"T1:%d  H1:%d|T2:%d  H2:%d       ",Tem, Humi, temRx, humiRx);
//			HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
//			HAL_UART_Transmit(&huart3,(uint8_t *)"\r",(uint16_t)strlen("\r"),1000);
			display_uart();
			aTemperature[0] = Tem >> 8;
			aTemperature[1] = Tem ;
			aHumidity[0] = 0x00;
			aHumidity[1] = Humi ;
			GetTick_Ms = HAL_GetTick();
		}
		
		if(getTick_transmit_uart >HAL_GetTick()) getTick_transmit_uart=0;
		if(HAL_GetTick()- getTick_transmit_uart > 100) 
		{
			HS300X_Start_Measurement(&hi2c2, (int16_t*)&Tem, (int16_t*)&Humi);
			Tem  = Tem  + Drop_Tem;
			Humi = Humi + Drop_Humi;
			uint8_t Frame[8];
			sData sFrame;
			sFrame.Data_a8 = Frame;
			ModRTU_Master_Read_Frame(&sFrame, address_Tx, 0x03, 0x02, 4);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart2, sFrame.Data_a8, sFrame.Length_u16, 1000);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			address_Tx++;
			if(address_Tx > 35) address_Tx=26;
			getTick_transmit_uart = HAL_GetTick();
		}
		
		check_address_slave = Modbus_RTU_Master();
		switch(check_address_slave)
		{
			case 26:
				T[0]=temRx; H[0]=humiRx;
				break;
			case 27:
				T[1]=temRx; H[1]=humiRx;
				break;
			case 28:
				T[2]=temRx; H[2]=humiRx;
				break;
			case 29:
				T[3]=temRx; H[3]=humiRx;
				break;
			case 30:
				T[4]=temRx; H[4]=humiRx;
				break;
			case 31:
				T[5]=temRx; H[5]=humiRx;
				break;
			case 32:
				T[6]=temRx; H[6]=humiRx;
				break;
			case 33:
				T[7]=temRx; H[7]=humiRx;
				break;
			case 34:
				T[8]=temRx; H[8]=humiRx;
				break;
			case 35:
				T[9]=temRx; H[9]=humiRx;
				break;
			default:
				break;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_192CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t Modbus_RTU_Master(void)
{
	if(Check_CountBuffer_Complete_Uart(&sUart2)==1)
	{
		uint16_t addressRx=sUart2.sim_rx[0];
		uint16_t funCode=sUart2.sim_rx[1];
		if(addressRx <= 35 && addressRx>=26 )
		{
			if(funCode == 0x03)
			{
				uint16_t CRC_rx = sUart2.sim_rx[sUart2.countBuffer-1] << 8 | sUart2.sim_rx[sUart2.countBuffer-2];
				uint16_t CRC_check = ModRTU_CRC(&sUart2.sim_rx[0], sUart2.countBuffer-2);
				if(CRC_check == CRC_rx)
				{
					temRx= sUart2.sim_rx[3]<<8 | sUart2.sim_rx[4];
					humiRx= sUart2.sim_rx[9]<<8 | sUart2.sim_rx[10];
				}
			}
			Delete_Buffer(&sUart2);
			return addressRx;
		}
		Delete_Buffer(&sUart2);
	}
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	if(huart->Instance == huart2.Instance)
	{
		sUart2.sim_rx[(sUart2.countBuffer)++]= sUart2.buffer;
		HAL_UART_Receive_IT(&huart2,&sUart2.buffer,1);
	}

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}
void HAL_SYSTICK_Callback(void)
{
//	GetTick_Ms++;
//	if(GetTick_Ms > Time_Sampling)
//	{
//		check_GetTick_Ms=1;
//		GetTick_Ms=0;
//	}
}

void FLASH_WriteNews(uint32_t addr_start_write,char News[])
{
  HAL_FLASH_Unlock();
	uint8_t length_tmp=0;
	if(32 %4 ==0) length_tmp=32/4;
	else length_tmp=32/4+1;
	uint32_t tmp[length_tmp];
	for(uint8_t i=0;i<length_tmp;i++)
	{
		tmp[i]=News[i];
		for(uint8_t j=i*4;j<i*4+4;j++)
		{
			tmp[i]=tmp[i]<< 8 | News[j];
		}
	}
	for(uint8_t i=0;i<length_tmp;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr_start_write , tmp[i]);
		(addr_start_write)=(addr_start_write)+4;
	}
  HAL_FLASH_Lock();
}

void FLASH_Earse(uint32_t addr_ready_earse)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInit.PageAddress = addr_ready_earse;
	EraseInit.NbPages = (1024)/256;
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInit, &PageError);
  HAL_FLASH_Lock();
}

void display_uart(void)
{
	int16_t max_tem=0; 
	int16_t min_tem=0;
	
	int16_t max_humi=0;
	int16_t min_humi=0;
	
	count++;
	sprintf(Tem_Humi,"T1:%d | T2:%d | T3:%d | T4:%d | T5:%d",T[0],T[1],T[2],T[3],T[4]);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r",(uint16_t)strlen("\r"),1000);
	
	sprintf(Tem_Humi,"T6:%d | T7:%d | T8:%d | T9:%d | T10:%d",T[5],T[6],T[7],T[8],T[9]);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
	sprintf(Tem_Humi,"H1:%d | H2:%d | H3:%d | H4:%d | H5:%d ",H[0],H[1],H[2],H[3],H[4]);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
				
	sprintf(Tem_Humi,"H6:%d | H7:%d | H8:%d | H9:%d | H10:%d ",H[5],H[6],H[7],H[8],H[9]);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
	uint16_t average_tem=0;
	uint16_t average_humi=0;
	uint8_t count_tem=0;
	uint8_t count_humi=0;
	for(uint8_t i=0;i<10;i++)
	{
		if(T[i]>0 ) 
		{
			min_tem=T[i];
			count_tem++;
			average_tem = average_tem + T[i];
		}
		
		if(H[i]>0 ) 
		{
			min_humi=H[i];
			count_humi++;
			average_humi = average_humi + H[i];
		}
	}
	if(count_tem>0) average_tem = average_tem/count_tem;
	if(count_humi>0) average_humi = average_humi/count_humi;
	
	for(uint8_t i=0; i<10;i++)
	{
		if(count_tem > 0)
		{
			if(T[i]>0)
			{
				AVG_T[i] = average_tem - T[i];
				if(T[i] > max_tem) max_tem = T[i];
				if(T[i] < min_tem) min_tem = T[i];
			}
		}
		
		if(count_humi > 0)
		{
			if(H[i]>0 )
			{
				AVG_H[i] = average_humi - H[i];
				if(H[i] > max_humi) max_humi = H[i];
				if(H[i] < min_humi) min_humi = H[i];
			}
		}
	}
	
	sprintf(Tem_Humi,"Average_Tem:%d      Average_Humi:%d", average_tem, average_humi);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
//	sprintf(Tem_Humi,"T1:%d | T2:%d | T3:%d | T4:%d | T5:%d",AVG_T[0],AVG_T[1],AVG_T[2],AVG_T[3],AVG_T[4]);
//	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
//	HAL_UART_Transmit(&huart3,(uint8_t *)"\r",(uint16_t)strlen("\r"),1000);
//	
//	sprintf(Tem_Humi,"T6:%d | T7:%d | T8:%d | T9:%d | T10:%d",AVG_T[5],AVG_T[6],AVG_T[7],AVG_T[8],AVG_T[9]);
//	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
//	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
//	
//	sprintf(Tem_Humi,"H1:%d | H2:%d | H3:%d | H4:%d | H5:%d ",AVG_H[0],AVG_H[1],AVG_H[2],AVG_H[3],AVG_H[4]);
//	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
//	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
//				
//	sprintf(Tem_Humi,"H6:%d | H7:%d | H8:%d | H9:%d | H10:%d ",AVG_H[5],AVG_H[6],AVG_H[7],AVG_H[8],AVG_H[9]);
//	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
//	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
	sprintf(Tem_Humi,"Tmax-Tmin:%d      Hmax-Hmin:%d",max_tem - min_tem , max_humi- min_humi);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
	sprintf(Tem_Humi,"Master_Tem:%d      Humi_Tem:%d",Tem , Humi);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	
	sprintf(Tem_Humi,"-----------------------%d------------------------",count);
	HAL_UART_Transmit(&huart3, (uint8_t *)Tem_Humi, (uint16_t)strlen(Tem_Humi), 1000);
	HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
