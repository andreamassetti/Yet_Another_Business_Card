/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TICK_DELAY 0x40
#define COLOR_DEPTH 2
#define COLOR_COUNT (1 << COLOR_DEPTH)
#define COLOR_MAX (COLOR_COUNT - 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint32_t bitDuration[COLOR_DEPTH];
#define xres 19
#define yres 20
uint8_t frameBuffer[yres * xres];

volatile int frame = 0;

uint32_t matrix_level_1[] = {349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762};

uint32_t reg_GPIOA_CRL, reg_GPIOB_CRL, reg_GPIOC_CRL, reg_GPIOD_CRL;
uint32_t reg_GPIOA_CRH, reg_GPIOB_CRH, reg_GPIOC_CRH, reg_GPIOD_CRH;
uint16_t reg_GPIOA_ODR, reg_GPIOB_ODR, reg_GPIOC_ODR, reg_GPIOD_ODR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void convertRowToRegisters(uint8_t rowIndex, uint32_t rowArray);
void convertArrayToRegistersA(uint8_t rowIndex, uint32_t pinsA);
void convertArrayToRegistersB(uint8_t rowIndex, uint32_t pinsB);
void convertArrayToRegistersC(uint8_t rowIndex, uint32_t pinsC);
void convertArrayToRegistersD(uint8_t rowIndex, uint32_t pinsD);
void writeRegisters();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  initBitDurationLUT();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int delay = 0;
  while (1)
  {
//	  HAL_GPIO_TogglePin(CHRLY1_GPIO_Port, CHRLY1_Pin);
//	  HAL_GPIO_TogglePin(CHRLY2_GPIO_Port, CHRLY2_Pin);
//  	HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY2_GPIO_Port, CHRLY2_Pin);
//	  HAL_GPIO_TogglePin(CHRLY3_GPIO_Port, CHRLY3_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY3_GPIO_Port, CHRLY3_Pin);
//	  HAL_GPIO_TogglePin(CHRLY4_GPIO_Port, CHRLY4_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY4_GPIO_Port, CHRLY4_Pin);
//	  HAL_GPIO_TogglePin(CHRLY5_GPIO_Port, CHRLY5_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY5_GPIO_Port, CHRLY5_Pin);
//	  HAL_GPIO_TogglePin(CHRLY6_GPIO_Port, CHRLY6_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY6_GPIO_Port, CHRLY6_Pin);
//	  HAL_GPIO_TogglePin(CHRLY7_GPIO_Port, CHRLY7_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY7_GPIO_Port, CHRLY7_Pin);
//	  HAL_GPIO_TogglePin(CHRLY8_GPIO_Port, CHRLY8_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY8_GPIO_Port, CHRLY8_Pin);
//	  HAL_GPIO_TogglePin(CHRLY9_GPIO_Port, CHRLY9_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY9_GPIO_Port, CHRLY9_Pin);
//	  HAL_GPIO_TogglePin(CHRLY10_GPIO_Port, CHRLY10_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY10_GPIO_Port, CHRLY10_Pin);
//	  HAL_GPIO_TogglePin(CHRLY11_GPIO_Port, CHRLY11_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY11_GPIO_Port, CHRLY11_Pin);
//	  HAL_GPIO_TogglePin(CHRLY12_GPIO_Port, CHRLY12_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY12_GPIO_Port, CHRLY12_Pin);
//	  HAL_GPIO_TogglePin(CHRLY13_GPIO_Port, CHRLY13_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY13_GPIO_Port, CHRLY13_Pin);
//	  HAL_GPIO_TogglePin(CHRLY14_GPIO_Port, CHRLY14_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY14_GPIO_Port, CHRLY14_Pin);
//	  HAL_GPIO_TogglePin(CHRLY15_GPIO_Port, CHRLY15_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY15_GPIO_Port, CHRLY15_Pin);
//	  HAL_GPIO_TogglePin(CHRLY16_GPIO_Port, CHRLY16_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY16_GPIO_Port, CHRLY16_Pin);
//	  HAL_GPIO_TogglePin(CHRLY17_GPIO_Port, CHRLY17_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY17_GPIO_Port, CHRLY17_Pin);
//	  HAL_GPIO_TogglePin(CHRLY18_GPIO_Port, CHRLY18_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY18_GPIO_Port, CHRLY18_Pin);
//	  HAL_GPIO_TogglePin(CHRLY19_GPIO_Port, CHRLY19_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY19_GPIO_Port, CHRLY19_Pin);
//	  HAL_GPIO_TogglePin(CHRLY20_GPIO_Port, CHRLY20_Pin);
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(CHRLY20_GPIO_Port, CHRLY20_Pin);
//	  HAL_GPIO_TogglePin(CHRLY1_GPIO_Port, CHRLY1_Pin);
//	  HAL_Delay(delay);

//  	GPIOB->BSRR = GPIO_BSRR_BR11; GPIOB->BSRR = GPIO_BSRR_BS8;
//  	//
//	  GPIOB->BSRR = GPIO_BSRR_BR8; GPIOB->BSRR = GPIO_BSRR_BS9;
//	  //
//	  GPIOB->BSRR = GPIO_BSRR_BR9; GPIOC->BSRR = GPIO_BSRR_BS13;
//	  //
//	  GPIOC->BSRR = GPIO_BSRR_BR13; GPIOC->BSRR = GPIO_BSRR_BS14;
//	  //
//	  GPIOC->BSRR = GPIO_BSRR_BR14; GPIOC->BSRR = GPIO_BSRR_BS15;
//	  //
//	  GPIOC->BSRR = GPIO_BSRR_BR15; GPIOD->BSRR = GPIO_BSRR_BS0;
//	  //
//	  GPIOD->BSRR = GPIO_BSRR_BR0; GPIOD->BSRR = GPIO_BSRR_BS1;
//	  //
//	  GPIOD->BSRR = GPIO_BSRR_BR1; GPIOA->BSRR = GPIO_BSRR_BS0;
//	  //
//	  GPIOA->BSRR = GPIO_BSRR_BR0; GPIOA->BSRR = GPIO_BSRR_BS1;
//	  //
//	  GPIOA->BSRR = GPIO_BSRR_BR1; GPIOA->BSRR = GPIO_BSRR_BS2;
//	  //
//	  GPIOA->BSRR = GPIO_BSRR_BR2; GPIOA->BSRR = GPIO_BSRR_BS3;
//	  //
//	  GPIOA->BSRR = GPIO_BSRR_BR3; GPIOA->BSRR = GPIO_BSRR_BS4;
//	  //
//	  GPIOA->BSRR = GPIO_BSRR_BR4; GPIOA->BSRR = GPIO_BSRR_BS5;
//	  //
//	  GPIOA->BSRR = GPIO_BSRR_BR5; GPIOA->BSRR = GPIO_BSRR_BS6;
//	  //
//	  GPIOA->BSRR = GPIO_BSRR_BR6; GPIOA->BSRR = GPIO_BSRR_BS7;
//	  //
//		GPIOA->BSRR = GPIO_BSRR_BR7; GPIOB->BSRR = GPIO_BSRR_BS0;
//		//
//		GPIOB->BSRR = GPIO_BSRR_BR0; GPIOB->BSRR = GPIO_BSRR_BS1;
//		//
//		GPIOB->BSRR = GPIO_BSRR_BR1; GPIOB->BSRR = GPIO_BSRR_BS2;
//		//
//		GPIOB->BSRR = GPIO_BSRR_BR2; GPIOB->BSRR = GPIO_BSRR_BS10;
//		//
//		GPIOB->BSRR = GPIO_BSRR_BR10; GPIOB->BSRR = GPIO_BSRR_BS11;
//		//


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 250;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CHRLY3_Pin|CHRLY4_Pin|CHRLY5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CHRLY6_Pin|CHRLY7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CHRLY8_Pin|CHRLY9_Pin|CHRLY10_Pin|CHRLY11_Pin
                          |CHRLY12_Pin|CHRLY13_Pin|CHRLY14_Pin|CHRLY15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CHRLY16_Pin|CHRLY17_Pin|CHRLY18_Pin|CHRLY19_Pin
                          |CHRLY20_Pin|CHRLY2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CHRLY1_GPIO_Port, CHRLY1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CHRLY3_Pin CHRLY4_Pin CHRLY5_Pin */
  GPIO_InitStruct.Pin = CHRLY3_Pin|CHRLY4_Pin|CHRLY5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CHRLY6_Pin CHRLY7_Pin */
  GPIO_InitStruct.Pin = CHRLY6_Pin|CHRLY7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CHRLY8_Pin CHRLY9_Pin CHRLY10_Pin CHRLY11_Pin
                           CHRLY12_Pin CHRLY13_Pin CHRLY14_Pin CHRLY15_Pin */
  GPIO_InitStruct.Pin = CHRLY8_Pin|CHRLY9_Pin|CHRLY10_Pin|CHRLY11_Pin
                          |CHRLY12_Pin|CHRLY13_Pin|CHRLY14_Pin|CHRLY15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CHRLY16_Pin CHRLY17_Pin CHRLY18_Pin CHRLY19_Pin
                           CHRLY20_Pin CHRLY1_Pin CHRLY2_Pin */
  GPIO_InitStruct.Pin = CHRLY16_Pin|CHRLY17_Pin|CHRLY18_Pin|CHRLY19_Pin
                          |CHRLY20_Pin|CHRLY1_Pin|CHRLY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void initBitDurationLUT(void){
	for(uint32_t i = 0; i < COLOR_DEPTH; i++){
		int dur = (1 << i);
		bitDuration[i] = TICK_DELAY * (dur + 1);
	}
}

//void mainTickHandler(void)
//{
//    static int segment = 0;
//    static int bit = 0;
//
//    setLeds(segment, bit + 8 - COLOR_DEPTH);
//
//    segment++;
//    if(segment == 16)
//    {
//        segment = 0;
//        bit++;
//        if(bit == COLOR_DEPTH)
//        {
//            bit = 0;
//            frame++;
//        }
//        SysTick->CMP = bitDuration[bit];
//    }
//    SysTick->SR=0;
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//	  HAL_GPIO_TogglePin(CHRLY1_GPIO_Port, CHRLY1_Pin);
//	  HAL_GPIO_TogglePin(CHRLY2_GPIO_Port, CHRLY2_Pin);
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	static int segment = 0;
	static int bit = 0;

	// setLeds(segment, bit + 8 - COLOR_DEPTH);

//	HAL_GPIO_TogglePin(CHRLY1_GPIO_Port, CHRLY1_Pin);
//	HAL_GPIO_TogglePin(CHRLY2_GPIO_Port, CHRLY2_Pin);
	// TIM1->ARR += 10;

	uint8_t row = 0;
	setLeds(row, matrix_level_1[row]);

	segment++;
	if(segment == yres)
	{
		segment = 0;
		bit++;
		if(bit == COLOR_DEPTH)
		{
			bit = 0;
		}
		// SysTick->CMP = bitDuration[bit];
		// Set the Auto Reload Register Value
		TIM1->ARR = bitDuration[bit];
	}
	// SysTick->SR=0;
}

void setLeds(uint8_t rowIndex, uint32_t rowArray){

	switch (rowIndex) {
		case 0:
			convertRowToRegisters(rowIndex, rowArray);

			break;
		case 1:

			break;
	}


}

void convertRowToRegisters(uint8_t rowIndex, uint32_t rowArray){
	uint32_t maskA1, maskB1, maskC1, maskD1;
	uint8_t GPIOA_bits, GPIOB_bits, GPIOC_bits, GPIOD_bits;
	uint16_t resA0, resA1, resA2, resA3, resA4, resA5, resA6, resA7;
	uint16_t resB0, resB1, resB2, resB8, resB9, resB10, resB11;
	uint16_t resC13, resC14, resC15;
	uint16_t resD0, resD1;


	switch (rowIndex) {
		case 0:
			// code block
			// set pin PB8 HIGH
			maskA1 = 0b00000000000000000001111111100000;
			maskB1 = 0b00000000000001000000000000011111;
			maskC1 = 0b00000000000000111000000000000000;
			maskD1 = 0b00000000000000000110000000000000;

			// Exctract which pins are on (OUTPUT LOW) and which are off (INPUT) for each register
			GPIOA_bits = rowArray & maskA1;
			GPIOB_bits = rowArray & maskB1;
			GPIOC_bits = rowArray & maskC1;
			GPIOD_bits = rowArray & maskD1;

			// Get the individual state of the pins from the

			// REGISTER A
			uint32_t resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			uint32_t resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 9; // B9 -> devo andare alla 9 ma sono alla 18
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			uint32_t resC = 0;
			resC |= (rowArray & 0b00000000000000100000000000000000) >> 4; // devo andare alla 13 ma sono alla 17
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 2; // devo andare alla 14 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 0; // devo andare alla 15 ma sono alla 15
			// REGISTER D
			uint32_t resD = 0;
			resD |= (rowArray & 0b00000000000000000100000000000000) >> 14; // devo andare alla 0 ma sono alla 14
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(8, resB);
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// reg_GPIOB_CRL =
			// reg_GPIOB_CRH |= (0b0010 << (0*4));
			reg_GPIOB_ODR |= (1 << 8);

			writeRegisters();

			break;
		case 1:
			// code block

			break;
		case 2:
			// code block
			// set pin PC13 HIGH


			break;
		case 3:
			// code block
			// set pin PC14 HIGH

			break;
		case 4:
			// code block
			// set pin PC15 HIGH

			break;
		case 5:
			// code block
			// set pin PD0 HIGH

			break;
		case 6:
			// code block
			// set pin PD1 HIGH

			break;
		case 7:
			// code block
			// set pin PA0 HIGH

			break;
		case 8:
			// code block
			// set pin PA1 HIGH

			break;
		case 9:
			// code block
			// set pin PA2 HIGH

			break;
		case 10:
			// code block
			// set pin PA3 HIGH

			break;
		case 11:
			// code block
			// set pin PA4 HIGH

			break;
		case 12:
			// code block
			// set pin PA5 HIGH

			break;
		case 13:
			// code block
			// set pin PA6 HIGH

			break;
		case 14:
			// code block
			// set pin PA7 HIGH

			break;
		case 15:
			// code block
			// set pin PB0 HIGH

			break;
		case 16:
			// code block
			// set pin PB1 HIGH

			break;
		case 17:
			// code block
			// set pin PB2 HIGH

			break;
		case 18:
			// code block
			// set pin PB10 HIGH

			break;
		case 19:
			// code block
			// set pin PB11 HIGH

			break;
	default:
		// code block
	}
}

void convertArrayToRegistersA(uint8_t rowIndex, uint32_t pinsA){
	// If the bit is HIGH we turn the LED on, then the pin needs to be OUTPUT LOW
	// If the bit is LOW we turn the LED off, then the pin needs to be INPUT Floating

	reg_GPIOA_CRL = 0;
	reg_GPIOA_CRH = 0;
	reg_GPIOA_ODR = 0;

	for(int i=0; i<8; i++){
		if(pinsA & (1 << i) || i==rowIndex){
			reg_GPIOA_CRL |= (0b0010 << (i*4));
		} else {
			reg_GPIOA_CRL |= (0b0100 << (i*4));
		}
	}
	for(int i=8; i<16; i++){
		if(pinsA & (1 << i) || i==rowIndex){
			reg_GPIOA_CRH |= (0b0010 << ((i-8)*4));
		} else {
			reg_GPIOA_CRH |= (0b0100 << ((i-8)*4));
		}
	}

	return;
}

void convertArrayToRegistersB(uint8_t rowIndex, uint32_t pinsB){
	// If the bit is HIGH we turn the LED on, then the pin needs to be OUTPUT LOW
	// If the bit is LOW we turn the LED off, then the pin needs to be INPUT Floating

	reg_GPIOB_CRL = 0;
	reg_GPIOB_CRH = 0;
	reg_GPIOB_ODR = 0;

	for(int i=0; i<8; i++){
		if(pinsB & (1 << i) || i==rowIndex){
			reg_GPIOB_CRL |= (0b0010 << (i*4));
		} else {
			reg_GPIOB_CRL |= (0b0100 << (i*4));
		}
	}
	for(int i=8; i<16; i++){
		if(pinsB & (1 << i) || i==rowIndex){
			reg_GPIOB_CRH |= (0b0010 << ((i-8)*4));
		} else {
			reg_GPIOB_CRH |= (0b0100 << ((i-8)*4));
		}
	}

	return;
}

void convertArrayToRegistersC(uint8_t rowIndex, uint32_t pinsC){
	// If the bit is HIGH we turn the LED on, then the pin needs to be OUTPUT LOW
	// If the bit is LOW we turn the LED off, then the pin needs to be INPUT Floating

	reg_GPIOC_CRL = 0;
	reg_GPIOC_CRH = 0;
	reg_GPIOC_ODR = 0;

	for(int i=0; i<8; i++){
		if(pinsC & (1 << i) || i==rowIndex){
			reg_GPIOC_CRL |= (0b0010 << (i*4));
		} else {
			reg_GPIOC_CRL |= (0b0100 << (i*4));
		}
	}
	for(int i=8; i<16; i++){
		if(pinsC & (1 << i) || i==rowIndex){
			reg_GPIOC_CRH |= (0b0010 << ((i-8)*4));
		} else {
			reg_GPIOC_CRH |= (0b0100 << ((i-8)*4));
		}
	}

	return;
}

void convertArrayToRegistersD(uint8_t rowIndex, uint32_t pinsD){
	// If the bit is HIGH we turn the LED on, then the pin needs to be OUTPUT LOW
	// If the bit is LOW we turn the LED off, then the pin needs to be INPUT Floating

	reg_GPIOD_CRL = 0;
	reg_GPIOD_CRH = 0;
	reg_GPIOD_ODR = 0;

	for(int i=0; i<8; i++){
		if(pinsD & (1 << i) || i==rowIndex){
			reg_GPIOD_CRL |= (0b0010 << (i*4));
		} else {
			reg_GPIOD_CRL |= (0b0100 << (i*4));
		}
	}
	for(int i=8; i<16; i++){
		if(pinsD & (1 << i) || i==rowIndex){
			reg_GPIOD_CRH |= (0b0010 << ((i-8)*4));
		} else {
			reg_GPIOD_CRH |= (0b0100 << ((i-8)*4));
		}
	}

	return;
}

void writeRegisters(){

	GPIOA->CRL = reg_GPIOA_CRL;
	GPIOA->CRH = reg_GPIOA_CRH;
	GPIOA->ODR = reg_GPIOA_ODR;

	GPIOB->CRL = reg_GPIOB_CRL;
	GPIOB->CRH = reg_GPIOB_CRH;
	GPIOB->ODR = reg_GPIOB_ODR;

	GPIOC->CRL = reg_GPIOC_CRL;
	GPIOC->CRH = reg_GPIOC_CRH;
	GPIOC->ODR = reg_GPIOC_ODR;

	GPIOD->CRL = reg_GPIOD_CRL;
	GPIOD->CRH = reg_GPIOD_CRH;
	GPIOD->ODR = reg_GPIOD_ODR;

	return;
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
#ifdef USE_FULL_ASSERT
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
