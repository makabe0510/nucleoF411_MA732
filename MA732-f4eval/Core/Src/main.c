/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct servo_params sp;

void magenc_update();

int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

uint8_t calcevenparity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

void magenc_update(SPI_HandleTypeDef *hspi){
	uint8_t t_data[2];
	uint8_t val[2];
	uint16_t command;

	command = AS5048A_ANGLE | 0x4000;
	command |= ((uint16_t)calcevenparity(command)<<15);

	t_data[1] = command & 0xff;
	t_data[0] = (command >> 8) & 0xff;
	val[0] = 0;
	val[1] = 0;

	HAL_GPIO_WritePin(MAGENC_CS_PORT, MAGENC_CS_PIN, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(hspi, (uint8_t *)&t_data, 2, 0xFFFF);
	HAL_SPI_Transmit(hspi, (uint8_t *)&t_data, 2, 100);
	while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {}
	HAL_GPIO_WritePin(MAGENC_CS_PORT, MAGENC_CS_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(MAGENC_CS_PORT, MAGENC_CS_PIN, GPIO_PIN_RESET);
	//HAL_SPI_Receive(hspi, (uint8_t *)&val, 2, 0xFFFF);
	HAL_SPI_Receive(hspi, (uint8_t *)&val, 2, 100);
	while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {}
	HAL_GPIO_WritePin(MAGENC_CS_PORT, MAGENC_CS_PIN, GPIO_PIN_SET);

	if(!((val[0] == 0x00 && val[1] == 0x00) || (val[0] == 0xff && val[1] == 0xff))){
		sp.magenc_noten_count = 0;
		sp.magenc_position = (( ( val[1] & 0xFF ) << 8 ) | ( val[0] & 0xFF )) & ~0xC000;
		sp.magenc_position_raw [0] = val[0];
		sp.magenc_position_raw [1] = val[1];
	}else{
		sp.magenc_noten_count += 1;
	}
}

uint16_t readMagAlphaAngle(SPI_HandleTypeDef *hspi)
{
  uint32_t timeout=10;
  uint8_t txData[2];
  uint8_t rxData[2];
  txData[1]=0;
  txData[0]=0;
  uint16_t angleSensor;
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, txData, rxData, 2, timeout);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  angleSensor=rxData[0]<<8 | rxData[1];
  return angleSensor;
}

uint8_t readMagAlphaRegister(SPI_HandleTypeDef *hspi, uint8_t address)
{
  uint32_t timeout=10;
  uint32_t delay=1;//ms
  uint8_t txData1[2];
  uint8_t rxData1[2];
  uint8_t txData2[2];
  uint8_t rxData2[2];
  txData1[0]=(0x2<<5)|(0x1F&address);
  txData1[1]=0x00;
  txData2[0]=0x00;
  txData2[1]=0x00;
  uint8_t registerReadbackValue;
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, txData1, rxData1, 2, timeout);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, txData2, rxData2, 2, timeout);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  registerReadbackValue=rxData2[0];
  return registerReadbackValue;
}

uint8_t readMagAlphaRegister2(SPI_HandleTypeDef *hspi, uint8_t address)
{
  uint32_t timeout=10;
  uint32_t delay=1;//ms
  uint8_t txData1[2];
  uint8_t rxData1[2];
  //uint8_t txData2[2];
  //uint8_t rxData2[2];
  txData1[0]=(0x2<<5)|(0x1F&address);
  txData1[1]=0x00;
  //txData2[0]=0x00;
  //txData2[1]=0x00;
  uint16_t registerReadbackValue;
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, txData1, rxData1, 1, timeout);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  //HAL_Delay(delay);
  //HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  //HAL_SPI_TransmitReceive(hspi, txData2, rxData2, 2, timeout);
  //HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  //registerReadbackValue=rxData2[0];
  //registerReadbackValue = rxData1[0]<<8 | rxData1[1];
  registerReadbackValue = rxData1[0];
  return registerReadbackValue;
}

uint8_t writeMagAlphaRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value)
{
  uint32_t timeout=10;
  uint32_t delay=20;//ms
  uint8_t txData1[2];
  uint8_t rxData1[2];
  uint8_t txData2[2];
  uint8_t rxData2[2];
  txData1[0]=(0x4<<5)|(0x1F&address);
  txData1[1]=value;
  txData2[0]=0x00;
  txData2[1]=0x00;
  uint8_t registerReadbackValue;
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, txData1, rxData1, 2, timeout);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, txData2, rxData2, 2, timeout);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  registerReadbackValue=rxData2[0];
  return registerReadbackValue;
}

//uint16_t readMagAlphaAngleWithParityBitCheck(SPI_HandleTypeDef *hspi, _Bool* error)
uint16_t readMagAlphaAngleWithParityBitCheck(SPI_HandleTypeDef *hspi)
{
  uint32_t timeout=10;
  uint8_t highStateCount = 0;
  uint8_t parity;
  uint8_t txData[3];
  uint8_t rxData[3];
  txData[2]=0;
  txData[1]=0;
  txData[0]=0;
  uint16_t angleSensor;
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, txData, rxData, 3, timeout);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  angleSensor=rxData[0]<<8 | rxData[1];
  parity = ((rxData[2] & 0x80) >> 7);
  //Count the number of 1 in the angle binary value
  for (int i=0;i<16;++i)
  {
    if ((angleSensor & (1 << i)) != 0)
    {
        highStateCount++;
    }
  }
  //check if parity bit is correct
  /*
  if ((highStateCount % 2) == 0) //number of bits set to 1 in the angle is even
  {
    if (parity == 0)
    {
      *error = false;
    }
    else
    {
      *error = true;
    }
  }
  else //number of bits set to 1 in the angle is odd
  {
    if (parity == 1)
    {
      *error = false;
    }
    else
    {
      *error = true;
    }
  }
  */
  return angleSensor;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  sp.magenc_noten_count = 0;
  sp.magenc_position_MA732 = 0;
  sp.MA732_BCT = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(sp.magenc_noten_count < MAGENC_NOT_EN_COUNT){
		  magenc_update(&hspi2);
	  }

	  sp.magenc_position_MA732 = readMagAlphaAngle(&hspi3);
	  //HAL_Delay(10);
	  //sp.MA732_BCT = readMagAlphaRegister(&hspi3, 0x2);

	  printf("hoge:%d\r\n", 100);
	  printf("AS5048A:%d\r\n", sp.magenc_position);
	  printf("MA732:%d\r\n", sp.magenc_position_MA732);
	  printf("MA732_BCT:%d\r\n", sp.MA732_BCT);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	  HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA15 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
