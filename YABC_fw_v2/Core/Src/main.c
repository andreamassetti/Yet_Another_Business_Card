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

#include "animations.h" // <--- Add this

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 1. STRUCTURES FOR LOOKUP TABLE
typedef struct {
    uint32_t PortA_CRL;
    // uint32_t PortA_CRH; // REMOVED to protect SWD (PA13/PA14)
    uint32_t PortA_ODR;

    uint32_t PortB_CRL;
    uint32_t PortB_CRH;
    uint32_t PortB_ODR;

    uint32_t PortC_CRL;
    uint32_t PortC_CRH;
    uint32_t PortC_ODR;

    uint32_t PortD_CRL;
    uint32_t PortD_CRH;
    uint32_t PortD_ODR;
} RowRegisterState;

typedef struct {
    GPIO_TypeDef* port;
    uint8_t pinIndex;
} PinDef;

typedef struct {
    PinDef anode;
    PinDef cathodes[19];
} HardwareRowDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TICK_DELAY 0x3
#define COLOR_DEPTH 2
#define COLOR_COUNT (1 << COLOR_DEPTH)
#define COLOR_MAX (COLOR_COUNT - 1)
#define ROW_COUNT 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// uint32_t bitDuration[COLOR_DEPTH];
#define xres 19
#define yres 20
uint8_t frameBuffer[yres * xres];

volatile int frame = 0;

// volatile int current_frame_idx = 0;

// uint32_t matrix_level_0[] = {349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762, 349525, 174762};
// uint32_t matrix_level_1[] = {349525, 0, 349525, 0, 349525, 0, 349525, 0, 349525, 0, 349525, 0, 349525, 0, 349525, 0, 349525, 0, 349525, 0};
// uint32_t matrix_level_1[] = {262145, 131074, 65540, 32776, 16400, 8224, 4160, 2176, 1280, 512, 1280, 2176, 4160, 8224, 16400, 32776, 65540, 131074, 262145, 262145};


uint32_t matrix_greyscale[20][19] = {
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7},
	{0x0D, 0x1A, 0x27, 0x34, 0x41, 0x4E, 0x5B, 0x68, 0x075, 0x82, 0x8F, 0x9C, 0xA9, 0xB6, 0xC3, 0xD0, 0xDD, 0xEA, 0xF7}
};


/* C-style array: 19x20 Matrix */
uint32_t matrix_level_0[20] = {
		0x00000000, /* Row  0 */
		0x0005C000, /* Row  1 */
		0x00050300, /* Row  2 */
		0x0007C780, /* Row  3 */
		0x00014780, /* Row  4 */
		0x00074FC0, /* Row  5 */
		0x00000FE0, /* Row  6 */
		0x00001CE0, /* Row  7 */
		0x00003870, /* Row  8 */
		0x00003030, /* Row  9 */
		0x00007038, /* Row 10 */
		0x0000FFF8, /* Row 11 */
		0x0001C01C, /* Row 12 */
		0x0003800E, /* Row 13 */
		0x0003000E, /* Row 14 */
		0x000000E0, /* Row 15 */
		0x00003FA0, /* Row 16 */
		0x0003A9E0, /* Row 17 */
		0x00003FA0, /* Row 18 */
		0x000000E0, /* Row 19 */
};
uint32_t matrix_level_1[20];

uint32_t frame_0[20] = {
	0x00000000, /* Row  0 */
    0x0003FFFC, /* Row  1 */
    0x00020604, /* Row  2 */
    0x00020604, /* Row  3 */
    0x00020604, /* Row  4 */
    0x00020604, /* Row  5 */
    0x00020604, /* Row  6 */
    0x00020604, /* Row  7 */
    0x0003FFFC, /* Row  8 */
    0x0003FFFC, /* Row  9 */
    0x00020604, /* Row 10 */
    0x00020604, /* Row 11 */
    0x00020604, /* Row 12 */
    0x00020604, /* Row 13 */
    0x00020604, /* Row 14 */
    0x00020604, /* Row 15 */
    0x00020604, /* Row 16 */
    0x0003FFFC, /* Row 17 */
    0x0003FFFC, /* Row 18 */
    0x0003FFFC, /* Row 19 */
};

uint32_t frame_1[20] = {
	0x00000000, /* Row  0 */
    0x0003FFFC, /* Row  1 */
    0x0003FFFC, /* Row  2 */
    0x0003FFFC, /* Row  3 */
    0x0003C63C, /* Row  4 */
    0x0003A65C, /* Row  5 */
    0x0003969C, /* Row  6 */
    0x00038F1C, /* Row  7 */
    0x0003FFFC, /* Row  8 */
    0x0003FFFC, /* Row  9 */
    0x0003861C, /* Row 10 */
    0x00038F1C, /* Row 11 */
    0x0003969C, /* Row 12 */
    0x0003A65C, /* Row 13 */
    0x0003C63C, /* Row 14 */
    0x0003FFFC, /* Row 15 */
    0x0003FFFC, /* Row 16 */
    0x0003FFFC, /* Row 17 */
    0x0003FFFC, /* Row 18 */
    0x0003FFFC, /* Row 19 */
};

uint32_t rowData = 0;

// volatile uint8_t system_active = 1; // Flag to track if we are awake
// uint32_t last_activity_time = 0;    // Timer to track the 10 seconds

uint32_t reg_GPIOA_CRL, reg_GPIOB_CRL, reg_GPIOC_CRL, reg_GPIOD_CRL;
uint32_t reg_GPIOA_CRH, reg_GPIOB_CRH, reg_GPIOC_CRH, reg_GPIOD_CRH;
uint16_t reg_GPIOA_ODR, reg_GPIOB_ODR, reg_GPIOC_ODR, reg_GPIOD_ODR;

RowRegisterState frameBufferLUT[ROW_COUNT][COLOR_DEPTH];

uint32_t bitDuration[COLOR_DEPTH];
volatile int current_frame_idx = 0;
volatile uint8_t update_frame_flag = 1; // Flag to trigger calculation

volatile uint8_t system_active = 1;
uint32_t last_activity_time = 0;

// 3. HARDWARE PIN MAPPING
// The exact physical order of your Charlieplexing network
const PinDef ALL_PINS[20] = {
    {GPIOB, 11}, {GPIOB, 10}, {GPIOB, 2},  {GPIOB, 1},  {GPIOB, 0},
    {GPIOA, 7},  {GPIOA, 6},  {GPIOA, 5},  {GPIOA, 4},  {GPIOA, 3},
    {GPIOA, 2},  {GPIOA, 1},  {GPIOA, 0},  {GPIOD, 1},  {GPIOD, 0},
    {GPIOC, 15}, {GPIOC, 14}, {GPIOC, 13}, {GPIOB, 9},  {GPIOB, 8}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void convertRowToRegisters(uint8_t rowIndex, uint32_t rowArray);
void convertArrayToRegistersA(uint8_t rowIndex, uint32_t pinsA);
void convertArrayToRegistersB(uint8_t rowIndex, uint32_t pinsB);
void convertArrayToRegistersC(uint8_t rowIndex, uint32_t pinsC);
void convertArrayToRegistersD(uint8_t rowIndex, uint32_t pinsD);
void initBitDurationLUT(void);
void setLeds(uint8_t rowIndex, uint32_t rowArray);
void writeRegisters();
void GetHardwareRow(int rowIndex, HardwareRowDef *rowDef);
void PrecomputeFrame(void);


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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // --- Example: Populate the frameBuffer with a test pattern ---

	// // 1. Clear the entire buffer to Level 0 (Off)
	// for (int i = 0; i < (yres * xres); i++) {
	// 		frameBuffer[i] = 0;
	// }

	// // 2. Draw a diagonal line with increasing brightness (0, 1, 2, 3)
	// for (int i = 0; i < yres && i < xres; i++) {
	// 		// Set pixel at (row i, col i)
	// 		// Brightness loops: 0, 1, 2, 3, 0, 1, 2, 3...
	// 		frameBuffer[i * xres + i] = (i % COLOR_COUNT);
	// }

	// // 3. Draw a static horizontal line at row 5 with medium brightness (Level 2)
	// for (int x = 0; x < xres; x++) {
	// 		frameBuffer[5 * xres + x] = 2;
	// }

	// // 4. Draw a static vertical line at col 2 with dim brightness (Level 1)
	// for (int y = 0; y < yres; y++) {
	// 		frameBuffer[y * xres + 2] = 1;
	// }
	PrecomputeFrame();

  HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // int delay = 0;
  last_activity_time = HAL_GetTick(); // Reset timer on startup
  
  while(1){
		// // 1. Check if 10 seconds (10000 ms) have passed
		// if(system_active && (HAL_GetTick() - last_activity_time > 10000)){
		// 	system_active = 0; // Time to sleep
		// }

		// if(system_active){
		// 	// --- ACTIVE STATE ---
		// 	// Ensure Timer is running for LEDs
		// 	if (htim1.State != HAL_TIM_STATE_BUSY) {
		// 		HAL_TIM_Base_Start_IT(&htim1);
		// 		HAL_TIM_Base_Start_IT(&htim2);
		// 	}
			
		// 	// If button is pressed while running, reset the 10s timer
		// 	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
		// 		last_activity_time = HAL_GetTick();
		// 	}
		// } else {
		// 	// --- SLEEP PREPARATION ---
			
		// 	// 1. Stop the LED Timer (Crucial: prevents interrupts waking cpu)
		// 	HAL_TIM_Base_Stop_IT(&htim1);
		// 	HAL_TIM_Base_Stop_IT(&htim2);

		// 	// 2. TURN OFF ALL LEDs (Crucial: prevents battery drain)
		// 	// Based on your 'writeRegisters' logic, we can force ODR to 0.
		// 	// (We modify the shadow registers you created)
		// 	reg_GPIOA_ODR = 0;
		// 	reg_GPIOB_ODR = 0;
		// 	reg_GPIOC_ODR = 0;
		// 	reg_GPIOD_ODR = 0;
		// 	writeRegisters(); // Apply the "All Off" state

		// 	// 3. Enter STOP Mode
		// 	// The CPU will halt here until PB12 is pressed
		// 	HAL_SuspendTick();
		// 	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

		// 	// --- WAKE UP SEQUENCE ---
		// 	// The code resumes here immediately after the button press interrupt
			
		// 	SystemClock_Config();  // Reconfigure system clocks
		// 	HAL_ResumeTick();      // Resume SysTick


		// 	// 5. Reset logic to run again
		// 	system_active = 1;
		// 	last_activity_time = HAL_GetTick();
		// }
		// --- SLEEP LOGIC ---
    if(system_active && (HAL_GetTick() - last_activity_time > 10000)){
        system_active = 0;
        
        // Disable Timers
        HAL_TIM_Base_Stop_IT(&htim1);
        HAL_TIM_Base_Stop_IT(&htim2);
        
        // Turn off all LEDs (Reset ODRs)
        GPIOA->ODR = 0; 
        GPIOB->ODR = (1<<12); // Keep PB12 Pull-Up!
        GPIOC->ODR = 0; 
        GPIOD->ODR = 0;

        // Enter Stop Mode
        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        
        // --- WAKE UP ---
        SystemClock_Config(); // Restore 48MHz clock
        HAL_ResumeTick();
        
        system_active = 1;
        last_activity_time = HAL_GetTick();
        
        // Restart Timers
        HAL_TIM_Base_Start_IT(&htim1);
        HAL_TIM_Base_Start_IT(&htim2);
    }

    // --- RENDER LOGIC ---
    // Only calculate when the frame changes
    if(system_active && update_frame_flag) {
        PrecomputeFrame();
        update_frame_flag = 0; // Clear flag
    }
  	// turn_on_all_LEDs_FAST();

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Constructs a 32-bit row bitmask for a specific bit-plane.
  * @param  rowIndex: The row index (0-19) to get data for.
  * @param  bitPlane: The bit-plane (0 for LSB, 1 for MSB, etc.)
  * @retval uint32_t: A bitmask where bit 'x' is set if the pixel (row, x)
  * has that bit-plane's bit set in its brightness value.
  */
uint32_t getRowDataFromFrameBuffer(int rowIndex, int bitPlane)
{
	uint32_t rowData = 0;

	// Calculate the starting index in the 1D frameBuffer for this row
	int rowOffset = rowIndex * xres;

	// Iterate through each pixel (column) in this row
	for (int x = 0; x < xres; x++)
	{
		// 1. Get the 8-bit brightness value for this pixel
		//    (This value will be 0, 1, 2, or 3)
		uint8_t brightness = frameBuffer[rowOffset + x];

		// 2. Extract the specific bit for the current bit-plane
		//    (brightness >> 0) & 1  -> gets bit 0
		//    (brightness >> 1) & 1  -> gets bit 1
		uint8_t pixelBit = (brightness >> bitPlane) & 1;

		// 3. If that bit is 1, set the corresponding bit in our rowData mask
		//    (This creates the 32-bit mask setLeds() expects)
		if (pixelBit)
		{
			rowData |= (1 << x);
		}
	}

	return rowData;
}

//void initBitDurationLUT(void){
//	for(uint32_t i = 0; i < COLOR_DEPTH; i++){
//		int dur = (1 << i); // 1, 2, 4, 8...
//		bitDuration[i] = TICK_DELAY * dur; // Correct BCM duration
//	}
//}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
// 	static int row = 0;
// 	static int bit = 0;

// 	if(htim->Instance == TIM1){

// 		rowData = animation_data[current_frame_idx][row];

// 		setLeds(row, rowData);

// 		row++;
// 		if(row == yres) // Have we finished all rows?
// 		{
// 			row = 0;    // Reset to first row
// 			bit++;    // Move to the next bit-plane

// 			if(bit == COLOR_DEPTH) // Have we finished all bit-planes?
// 			{
// 				bit = 0; // Reset to first bit-plane
// 			}

// 			// Set the timer period for the *next* bit plane
// 			TIM1->ARR = bitDuration[bit];
// 		}
// 	}

// 	if(htim->Instance == TIM2){
// 		// This timer dictates how fast the animation plays (FPS)
// 		current_frame_idx++;

// 		if (current_frame_idx >= ANIMATION_FRAMES) {
// 			current_frame_idx = 0; // Loop back to start
// 		}
// 	}
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    static int row = 0;
    static int bit = 0;

    if(htim->Instance == TIM1) // LED Refresh
    {
        // Pointer to the pre-calculated register state
        RowRegisterState *state = &frameBufferLUT[row][bit];

        // 1. Dump Registers (Takes ~20 cycles)
        GPIOA->CRL = state->PortA_CRL;
        // GPIOA->CRH = state->PortA_CRH; // DISABLED FOR SWD SAFETY (PA13/14)
        GPIOA->ODR = state->PortA_ODR;

        GPIOB->CRL = state->PortB_CRL;
        GPIOB->CRH = state->PortB_CRH;
        GPIOB->ODR = state->PortB_ODR;

        GPIOC->CRL = state->PortC_CRL;
        GPIOC->CRH = state->PortC_CRH;
        GPIOC->ODR = state->PortC_ODR;

        GPIOD->CRL = state->PortD_CRL;
        GPIOD->CRH = state->PortD_CRH;
        GPIOD->ODR = state->PortD_ODR;

        // 2. Advance Counters
        row++;
        if(row >= ROW_COUNT)
        {
            row = 0;
            bit++;
            if(bit >= COLOR_DEPTH) bit = 0;
            
            TIM1->ARR = bitDuration[bit]; // Update BAM timing
        }
    }

    if(htim->Instance == TIM2) // Animation Tick
    {
        current_frame_idx++;
        if (current_frame_idx >= ANIMATION_FRAMES) {
            current_frame_idx = 0;
        }
        update_frame_flag = 1; // Tell Main loop to calculate next frame
    }
}

void PrecomputeFrame(void) {
    const uint32_t DEFAULT_CRL = 0x44444444; 
    const uint32_t DEFAULT_CRH = 0x44444444; 

    // SAFETY FOR PB12: Default CRH for Port B must set Pin 12 to Input Pull-Up/Down (0x8)
    const uint32_t PORTB_CRH_SAFE = 0x44484444; 
    const uint32_t PORTB_ODR_SAFE = (1 << 12); // Pull-Up selected

    HardwareRowDef rowHardware;

    for(int row = 0; row < ROW_COUNT; row++) {
        GetHardwareRow(row, &rowHardware);

        for(int bitPlane = 0; bitPlane < COLOR_DEPTH; bitPlane++) {
            
            RowRegisterState *state = &frameBufferLUT[row][bitPlane];

            // 1. Reset State to Safe Defaults
            state->PortA_CRL = DEFAULT_CRL; 
            state->PortA_ODR = 0; // CRH skipped for SWD

            state->PortB_CRL = DEFAULT_CRL; 
            state->PortB_CRH = PORTB_CRH_SAFE; // PB12 Safety
            state->PortB_ODR = PORTB_ODR_SAFE; // PB12 Safety

            state->PortC_CRL = DEFAULT_CRL; 
            state->PortC_CRH = DEFAULT_CRH; 
            state->PortC_ODR = 0;

            state->PortD_CRL = DEFAULT_CRL; 
            state->PortD_CRH = DEFAULT_CRH; 
            state->PortD_ODR = 0;

            // 2. Configure ANODE (Active High)
            PinDef anode = rowHardware.anode;
            
            // Determine Port Pointers
            uint32_t *crl = (anode.pinIndex < 8) ? ((anode.port == GPIOA) ? &state->PortA_CRL : (anode.port == GPIOB) ? &state->PortB_CRL : (anode.port == GPIOC) ? &state->PortC_CRL : &state->PortD_CRL)
                                                 : ((anode.port == GPIOA) ? NULL : (anode.port == GPIOB) ? &state->PortB_CRH : (anode.port == GPIOC) ? &state->PortC_CRH : &state->PortD_CRH);
            
            uint32_t *odr = (anode.port == GPIOA) ? &state->PortA_ODR : (anode.port == GPIOB) ? &state->PortB_ODR : (anode.port == GPIOC) ? &state->PortC_ODR : &state->PortD_ODR;

            if(crl) {
                uint8_t shift = (anode.pinIndex % 8) * 4;
                *crl &= ~(0xF << shift); 
                *crl |=  (0x2 << shift); // Output 2MHz
                *odr |=  (1 << anode.pinIndex); // HIGH
            }

            // 3. Configure CATHODES (Active Low)
            uint32_t rowData = animation_data[current_frame_idx][row]; 

            for(int col = 0; col < 19; col++) {
                // Bit extraction logic (Modify if you increase COLOR_DEPTH)
                // Currently checks if the specific column bit is High
                if((rowData >> col) & 0x01) { 
                    PinDef cathode = rowHardware.cathodes[col];
                    
                    uint32_t *c_crl = (cathode.pinIndex < 8) ? ((cathode.port == GPIOA) ? &state->PortA_CRL : (cathode.port == GPIOB) ? &state->PortB_CRL : (cathode.port == GPIOC) ? &state->PortC_CRL : &state->PortD_CRL)
                                                             : ((cathode.port == GPIOA) ? NULL : (cathode.port == GPIOB) ? &state->PortB_CRH : (cathode.port == GPIOC) ? &state->PortC_CRH : &state->PortD_CRH);
                    
                    uint32_t *c_odr = (cathode.port == GPIOA) ? &state->PortA_ODR : (cathode.port == GPIOB) ? &state->PortB_ODR : (cathode.port == GPIOC) ? &state->PortC_ODR : &state->PortD_ODR;

                    if(c_crl) {
                        uint8_t c_shift = (cathode.pinIndex % 8) * 4;
                        *c_crl &= ~(0xF << c_shift);
                        *c_crl |=  (0x2 << c_shift); // Output
                        *c_odr &= ~(1 << cathode.pinIndex); // LOW
                    }
                }
            }
        }
    }
}

// Helper to map Row Index -> Hardware Pins
void GetHardwareRow(int rowIndex, HardwareRowDef *rowDef) {
    int anodeIndex = 19 - rowIndex; 
    rowDef->anode = ALL_PINS[anodeIndex];

    int cathodeCount = 0;
    for (int i = 0; i < 20; i++) {
        if (i == anodeIndex) continue;
        rowDef->cathodes[cathodeCount++] = ALL_PINS[i];
    }
}

void initBitDurationLUT(void){
    // Example logarithmic brightness for BAM
    for(uint32_t i = 0; i < COLOR_DEPTH; i++){
        bitDuration[i] = 20 * (1 << i); 
    }
}

void setLeds(uint8_t rowIndex, uint32_t rowArray){

	uint32_t resA, resB, resC, resD;

	switch (rowIndex) {
		case 0:
			// PB8 is HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 9; // B9 -> devo andare alla 9 ma sono alla 18
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000100000000000000000) >> 4; // devo andare alla 13 ma sono alla 17
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 2; // devo andare alla 14 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 0; // devo andare alla 15 ma sono alla 15
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000100000000000000) >> 14; // devo andare alla 0 ma sono alla 14
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(8, resB);
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			reg_GPIOB_ODR |= (1 << 8);

			writeRegisters();

			break;
		case 1:
			// PB9 is HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000100000000000000000) >> 4; // devo andare alla 13 ma sono alla 17
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 2; // devo andare alla 14 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 0; // devo andare alla 15 ma sono alla 15
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000100000000000000) >> 14; // devo andare alla 0 ma sono alla 14
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(9, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PB9 HIGH
			reg_GPIOB_ODR |= (1 << 9);

			writeRegisters();

			break;
		case 2:
			// code block
			// set pin PC13 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			// resC |= (rowArray & 0b00000000000000100000000000000000) >> 4; // PC13 -> devo andare alla 13 ma sono alla 17
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 2; // PC14 -> devo andare alla 14 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 0; // PC15 -> devo andare alla 15 ma sono alla 15
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000100000000000000) >> 14; // devo andare alla 0 ma sono alla 14
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(13, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PC13 HIGH
			reg_GPIOC_ODR |= (1 << 13);

			writeRegisters();

			break;
		case 3:
			// code block
			// set pin PC14 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			// resC |= (rowArray & 0b00000000000000010000000000000000) >> 2; // PC14 -> devo andare alla 14 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 0; // PC15 -> devo andare alla 15 ma sono alla 15
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000100000000000000) >> 14; // devo andare alla 0 ma sono alla 14
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(-1, resB);
			convertArrayToRegistersC(14, resC);
			convertArrayToRegistersD(-1, resD);

			// set pin PC14 HIGH
			reg_GPIOC_ODR |= (1 << 14);

			writeRegisters();

			break;
		case 4:
			// code block
			// set pin PC15 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			// resC |= (rowArray & 0b00000000000000001000000000000000) >> 0; // PC15 -> devo andare alla 15 ma sono alla 15
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000100000000000000) >> 14; // devo andare alla 0 ma sono alla 14
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(15, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PC15 HIGH
			reg_GPIOC_ODR |= (1 << 15);

			writeRegisters();

			break;
		case 5:
			// code block
			// set pin PD0 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			// resD |= (rowArray & 0b00000000000000000100000000000000) >> 14; // devo andare alla 0 ma sono alla 14
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(0, resD);

			// set ping PD0 HIGH
			reg_GPIOD_ODR |= (1 << 0);

			writeRegisters();

			break;
		case 6:
			// code block
			// set pin PD1 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			// resD |= (rowArray & 0b00000000000000000010000000000000) >> 12; // devo andare alla 1 ma sono alla 13

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(1, resD);

			// set ping PD1 HIGH
			reg_GPIOD_ODR |= (1 << 1);

			writeRegisters();

			break;
		case 7:
			// code block
			// set pin PA0 HIGH

			// REGISTER A
			resA = 0;
			// resA |= (rowArray & 0b00000000000000000001000000000000) >> 12; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(0, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PC13 HIGH
			reg_GPIOA_ODR |= (1 << 0);

			writeRegisters();

			break;
		case 8:
			// code block
			// set pin PA1 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			// resA |= (rowArray & 0b00000000000000000000100000000000) >> 10; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(1, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA1 HIGH
			reg_GPIOA_ODR |= (1 << 1);

			writeRegisters();

			break;
		case 9:
			// code block
			// set pin PA2 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			// resA |= (rowArray & 0b00000000000000000000010000000000) >> 8; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(2, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA1 HIGH
			reg_GPIOA_ODR |= (1 << 2);

			writeRegisters();

			break;
		case 10:
			// code block
			// set pin PA3 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			// resA |= (rowArray & 0b00000000000000000000001000000000) >> 6; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(3, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA3 HIGH
			reg_GPIOA_ODR |= (1 << 3);

			writeRegisters();

			break;
		case 11:
			// code block
			// set pin PA4 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			// resA |= (rowArray & 0b00000000000000000000000100000000) >> 4; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(4, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA4 HIGH
			reg_GPIOA_ODR |= (1 << 4);

			writeRegisters();

			break;
		case 12:
			// code block
			// set pin PA5 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			// resA |= (rowArray & 0b00000000000000000000000010000000) >> 2; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(5, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA4 HIGH
			reg_GPIOA_ODR |= (1 << 5);

			writeRegisters();

			break;
		case 13:
			// code block
			// set pin PA6 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 1; // devo andare alla 5 ma sono alla 7
			// resA |= (rowArray & 0b00000000000000000000000001000000) >> 0; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(6, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA4 HIGH
			reg_GPIOA_ODR |= (1 << 6);

			writeRegisters();

			break;
		case 14:
			// code block
			// set pin PA7 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 1; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000000100000) << 1; // devo andare alla 6 ma sono alla 6
			// resA |= (rowArray & 0b00000000000000000000000000100000) << 2; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(7, resA);
			convertArrayToRegistersB(-1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA4 HIGH
			reg_GPIOA_ODR |= (1 << 7);

			writeRegisters();

			break;
		case 15:
			// code block
			// set pin PB0 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 1; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000000100000) << 1; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000010000) << 3; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			// resB |= (rowArray & 0b00000000000000000000000000010000) >> 4; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(0, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PA4 HIGH
			reg_GPIOB_ODR |= (1 << 0);

			writeRegisters();

			break;
		case 16:
			// code block
			// set pin PB1 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 1; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000000100000) << 1; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000010000) << 3; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 3; // B0 -> devo andare alla 0 ma sono alla 4
			// resB |= (rowArray & 0b00000000000000000000000000001000) >> 2; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(1, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PB1 HIGH
			reg_GPIOB_ODR |= (1 << 1);

			writeRegisters();

			break;
		case 17:
			// code block
			// set pin PB2 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 1; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000000100000) << 1; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000010000) << 3; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 3; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 1; // B1 -> devo andare alla 1 ma sono alla 3
			// resB |= (rowArray & 0b00000000000000000000000000000100) >> 0; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(2, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PB1 HIGH
			reg_GPIOB_ODR |= (1 << 2);

			writeRegisters();

			break;
		case 18:
			// code block
			// set pin PB10 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 1; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000000100000) << 1; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000010000) << 3; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 3; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 1; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000010) << 1; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			// resB |= (rowArray & 0b00000000000000000000000000000010) << 9; // B10 -> devo andare alla 10 ma sono alla 1
			resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(10, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PB1 HIGH
			reg_GPIOB_ODR |= (1 << 10);

			writeRegisters();

			break;
		case 19:
			// code block
			// set pin PB11 HIGH

			// REGISTER A
			resA = 0;
			resA |= (rowArray & 0b00000000000000000000100000000000) >> 11; // devo andare alla 0 ma sono alla 12
			resA |= (rowArray & 0b00000000000000000000010000000000) >> 9; // devo andare alla 1 ma sono alla 11
			resA |= (rowArray & 0b00000000000000000000001000000000) >> 7; // devo andare alla 2 ma sono alla 10
			resA |= (rowArray & 0b00000000000000000000000100000000) >> 5; // devo andare alla 3 ma sono alla 9
			resA |= (rowArray & 0b00000000000000000000000010000000) >> 3; // devo andare alla 4 ma sono alla 8
			resA |= (rowArray & 0b00000000000000000000000001000000) >> 1; // devo andare alla 5 ma sono alla 7
			resA |= (rowArray & 0b00000000000000000000000000100000) << 1; // devo andare alla 6 ma sono alla 6
			resA |= (rowArray & 0b00000000000000000000000000010000) << 3; // devo andare alla 7 ma sono alla 5
			// REGISTER B
			resB = 0;
			resB |= (rowArray & 0b00000000000000000000000000001000) >> 3; // B0 -> devo andare alla 0 ma sono alla 4
			resB |= (rowArray & 0b00000000000000000000000000000100) >> 1; // B1 -> devo andare alla 1 ma sono alla 3
			resB |= (rowArray & 0b00000000000000000000000000000010) << 1; // B2 -> devo andare alla 2 ma sono alla 2
			resB |= (rowArray & 0b00000000000001000000000000000000) >> 10; // B8 -> devo andare alla 8 ma sono alla 18
			resB |= (rowArray & 0b00000000000000100000000000000000) >> 8; // B9 -> devo andare alla 9 ma sono alla 17
			resB |= (rowArray & 0b00000000000000000000000000000001) << 10; // B10 -> devo andare alla 10 ma sono alla 1
			// resB |= (rowArray & 0b00000000000000000000000000000001) << 11; // B11 -> devo andare alla 11 ma sono alla 0
			// REGISTER C
			resC = 0;
			resC |= (rowArray & 0b00000000000000010000000000000000) >> 3; // PC13 -> devo andare alla 13 ma sono alla 16
			resC |= (rowArray & 0b00000000000000001000000000000000) >> 1; // PC14 -> devo andare alla 14 ma sono alla 15
			resC |= (rowArray & 0b00000000000000000100000000000000) << 1; // PC15 -> devo andare alla 15 ma sono alla 14
			// REGISTER D
			resD = 0;
			resD |= (rowArray & 0b00000000000000000010000000000000) >> 13; // devo andare alla 0 ma sono alla 13
			resD |= (rowArray & 0b00000000000000000001000000000000) >> 11; // devo andare alla 1 ma sono alla 12

			convertArrayToRegistersA(-1, resA);
			convertArrayToRegistersB(11, resB); // set also pin PB9 as OUTPUT
			convertArrayToRegistersC(-1, resC);
			convertArrayToRegistersD(-1, resD);

			// set ping PB1 HIGH
			reg_GPIOB_ODR |= (1 << 11);

			writeRegisters();

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

void turn_on_all_LEDs_FAST(){
//	GPIOB->BSRR = GPIO_BSRR_BR11; GPIOB->BSRR = GPIO_BSRR_BS8;
//	//
//	GPIOB->BSRR = GPIO_BSRR_BR8; GPIOB->BSRR = GPIO_BSRR_BS9;
//	//
//	GPIOB->BSRR = GPIO_BSRR_BR9; GPIOC->BSRR = GPIO_BSRR_BS13;
//	//
//	GPIOC->BSRR = GPIO_BSRR_BR13; GPIOC->BSRR = GPIO_BSRR_BS14;
//	//
//	GPIOC->BSRR = GPIO_BSRR_BR14; GPIOC->BSRR = GPIO_BSRR_BS15;
//	//
//	GPIOC->BSRR = GPIO_BSRR_BR15; GPIOD->BSRR = GPIO_BSRR_BS0;
//	//
//	GPIOD->BSRR = GPIO_BSRR_BR0; GPIOD->BSRR = GPIO_BSRR_BS1;
//	//
//	GPIOD->BSRR = GPIO_BSRR_BR1; GPIOA->BSRR = GPIO_BSRR_BS0;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR0; GPIOA->BSRR = GPIO_BSRR_BS1;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR1; GPIOA->BSRR = GPIO_BSRR_BS2;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR2; GPIOA->BSRR = GPIO_BSRR_BS3;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR3; GPIOA->BSRR = GPIO_BSRR_BS4;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR4; GPIOA->BSRR = GPIO_BSRR_BS5;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR5; GPIOA->BSRR = GPIO_BSRR_BS6;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR6; GPIOA->BSRR = GPIO_BSRR_BS7;
//	//
//	GPIOA->BSRR = GPIO_BSRR_BR7; GPIOB->BSRR = GPIO_BSRR_BS0;
//	//
//	GPIOB->BSRR = GPIO_BSRR_BR0; GPIOB->BSRR = GPIO_BSRR_BS1;
//	//
//	GPIOB->BSRR = GPIO_BSRR_BR1; GPIOB->BSRR = GPIO_BSRR_BS2;
//	//
//	GPIOB->BSRR = GPIO_BSRR_BR2; GPIOB->BSRR = GPIO_BSRR_BS10;
//	//
//	GPIOB->BSRR = GPIO_BSRR_BR10; GPIOB->BSRR = GPIO_BSRR_BS11;
//	//
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // if (GPIO_Pin == GPIO_PIN_12)
    // {
    // 	SystemClock_Config();  // Reconfigure system clocks
    // 	HAL_ResumeTick();      // Resume SysTick
    // }
		// 4. Restore Clock (HSI -> PLL)
	
}

// 1. Define the 20 pins in the exact physical order used by your circuit

//
//// 2. Helper to get the Anode and Cathodes for any row
//// (This logic replaces the switch statement entirely)
//void GetHardwareRow(int rowIndex, HardwareRowDef *rowDef) {
//    // Your logic runs backward: Row 0 uses Pin 19, Row 19 uses Pin 0
//    int anodeIndex = 19 - rowIndex;
//
//    rowDef->anode = ALL_PINS[anodeIndex];
//
//    // Map columns to the remaining 19 pins
//    // If the cathode index is >= anode index, we skip the anode pin
//    int cathodeCount = 0;
//    for (int i = 0; i < 20; i++) {
//        if (i == anodeIndex) continue; // Skip the anode
//        rowDef->cathodes[cathodeCount++] = ALL_PINS[i];
//    }
//}
//
//void PrecomputeFrame(void) {
//    // Default: Input Floating (0x4)
//    const uint32_t DEFAULT_CRL = 0x44444444;
//    const uint32_t DEFAULT_CRH = 0x44444444;
//
//    // SAFETY FOR PB12: Default CRH for Port B must set Pin 12 to Input Pull-Up/Down (0x8)
//    // Pin 12 is the lowest nibble of CRH [3:0] -> 0x....8
//    const uint32_t PORTB_CRH_SAFE = 0x44484444;
//
//    // SAFETY FOR PB12: ODR must be 1 to select Pull-UP (not Pull-Down)
//    const uint32_t PORTB_ODR_SAFE = (1 << 12);
//
//    HardwareRowDef rowHardware;
//
//    for(int row = 0; row < 20; row++) {
//        // Get the pins for this row dynamically
//        GetHardwareRow(row, &rowHardware);
//
//        for(int bitPlane = 0; bitPlane < COLOR_DEPTH; bitPlane++) {
//
//            RowRegisterState *state = &frameBufferLUT[row][bitPlane];
//
//            // 1. Initialize with SAFE defaults
//            state->PortA_CRL = DEFAULT_CRL;
//            // state->PortA_CRH = ... REMOVED to protect SWD (PA13/14)
//            state->PortA_ODR = 0;
//
//            state->PortB_CRL = DEFAULT_CRL;
//            state->PortB_CRH = PORTB_CRH_SAFE; // Protects PB12 Mode
//            state->PortB_ODR = PORTB_ODR_SAFE; // Protects PB12 Pull-Up
//
//            state->PortC_CRL = DEFAULT_CRL;
//            state->PortC_CRH = DEFAULT_CRH;
//            state->PortC_ODR = 0;
//
//            state->PortD_CRL = DEFAULT_CRL;
//            state->PortD_CRH = DEFAULT_CRH;
//            state->PortD_ODR = 0;
//
//            // 2. Configure ANODE (Active High)
//            PinDef anode = rowHardware.anode;
//
//            // Pointer selection logic...
//            uint32_t *crl = (anode.pinIndex < 8) ? ((anode.port == GPIOA) ? &state->PortA_CRL : (anode.port == GPIOB) ? &state->PortB_CRL : (anode.port == GPIOC) ? &state->PortC_CRL : &state->PortD_CRL)
//                                                 : ((anode.port == GPIOA) ? NULL : (anode.port == GPIOB) ? &state->PortB_CRH : (anode.port == GPIOC) ? &state->PortC_CRH : &state->PortD_CRH);
//            // Note: NULL check added for PortA_CRH safety
//
//            uint32_t *odr = (anode.port == GPIOA) ? &state->PortA_ODR : (anode.port == GPIOB) ? &state->PortB_ODR : (anode.port == GPIOC) ? &state->PortC_ODR : &state->PortD_ODR;
//
//            if(crl) {
//                uint8_t shift = (anode.pinIndex % 8) * 4;
//                *crl &= ~(0xF << shift);
//                *crl |=  (0x2 << shift); // Output 2MHz
//                *odr |=  (1 << anode.pinIndex); // HIGH
//            }
//
//            // 3. Configure CATHODES (Active Low)
//            uint32_t rowData = animation_data[current_frame_idx][row];
//
//            for(int col = 0; col < 19; col++) {
//                if((rowData >> col) & 0x01) { // If pixel is ON
//                    PinDef cathode = rowHardware.cathodes[col];
//
//                    uint32_t *c_crl = (cathode.pinIndex < 8) ? ((cathode.port == GPIOA) ? &state->PortA_CRL : (cathode.port == GPIOB) ? &state->PortB_CRL : (cathode.port == GPIOC) ? &state->PortC_CRL : &state->PortD_CRL)
//                                                             : ((cathode.port == GPIOA) ? NULL : (cathode.port == GPIOB) ? &state->PortB_CRH : (cathode.port == GPIOC) ? &state->PortC_CRH : &state->PortD_CRH);
//
//                    uint32_t *c_odr = (cathode.port == GPIOA) ? &state->PortA_ODR : (cathode.port == GPIOB) ? &state->PortB_ODR : (cathode.port == GPIOC) ? &state->PortC_ODR : &state->PortD_ODR;
//
//                    if(c_crl) {
//                        uint8_t c_shift = (cathode.pinIndex % 8) * 4;
//                        *c_crl &= ~(0xF << c_shift);
//                        *c_crl |=  (0x2 << c_shift); // Output
//                        *c_odr &= ~(1 << cathode.pinIndex); // LOW
//                    }
//                }
//            }
//        }
//    }
//}

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
