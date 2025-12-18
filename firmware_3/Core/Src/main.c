/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Optimized Main program body for Charlieplexing (4MHz Low Power)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

// Ensure this header defines 'video_frames[ANIMATION_FRAMES][20][19]'
// #include "animations.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Represents the register state for one specific row/bit-plane
typedef struct {
    uint32_t PortA_CRL;
    // uint32_t PortA_CRH; // EXCLUDED to protect SWD (PA13/PA14)
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

// Physical Pin Definition
typedef struct {
    GPIO_TypeDef* port;
    uint8_t pinIndex;
} PinDef;

// Hardware Map for a Row
typedef struct {
    PinDef anode;
    PinDef cathodes[19];
} HardwareRowDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COLOR_DEPTH 2    // 2-bit BCM (4 brightness levels: 0, 1, 2, 3)
#define ROW_COUNT 20
#define COL_COUNT 19

#define ANIMATION_FRAMES 1
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// --- GLOBAL BUFFERS ---
// The ISR reads from here. Main loop writes to here.
RowRegisterState frameBufferLUT[ROW_COUNT][COLOR_DEPTH];

uint32_t bitDuration[COLOR_DEPTH];
volatile int current_frame_idx = 0;
volatile uint8_t update_frame_flag = 1; // Flag to trigger calculation

volatile uint8_t system_active = 1;
uint32_t last_activity_time = 0;

// --- HARDWARE PIN MAPPING ---
// The exact physical order of your Charlieplexing network (Pins 0-19)
const PinDef ALL_PINS[20] = {
    {GPIOB, 11}, {GPIOB, 10}, {GPIOB, 2},  {GPIOB, 1},  {GPIOB, 0},
    {GPIOA, 7},  {GPIOA, 6},  {GPIOA, 5},  {GPIOA, 4},  {GPIOA, 3},
    {GPIOA, 2},  {GPIOA, 1},  {GPIOA, 0},  {GPIOD, 1},  {GPIOD, 0},
    {GPIOC, 15}, {GPIOC, 14}, {GPIOC, 13}, {GPIOB, 9},  {GPIOB, 8}
};

uint32_t video_frames[1][20][19] = {
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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
void initBitDurationLUT(void);
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
  HAL_Init();

  // 1. Configure System Clock to 4 MHz (Low Power)
  SystemClock_Config();

  initBitDurationLUT();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  // 2. Initial Frame Calculation
  PrecomputeFrame();

  // 3. Start Timers
  HAL_TIM_Base_Start_IT(&htim1); // LED Refresh (High Speed)
  HAL_TIM_Base_Start_IT(&htim2); // Animation Tick (Low Speed)

  last_activity_time = HAL_GetTick();

  while (1)
  {
    // --- SLEEP LOGIC (10 Seconds Timeout) ---
    if(system_active && (HAL_GetTick() - last_activity_time > 10000)){
        system_active = 0;
        
        // Disable Timers to stop interrupts
        HAL_TIM_Base_Stop_IT(&htim1);
        HAL_TIM_Base_Stop_IT(&htim2);
        
        // Turn off all LEDs (Reset ODRs to safe state)
        GPIOA->ODR = 0; 
        GPIOB->ODR = (1<<12); // Keep PB12 Pull-Up active!
        GPIOC->ODR = 0; 
        GPIOD->ODR = 0;

        // Enter Stop Mode
        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        
        // --- WAKE UP POINT ---
        SystemClock_Config(); // Restore 4 MHz clock
        HAL_ResumeTick();
        
        system_active = 1;
        last_activity_time = HAL_GetTick();
        
        // Restart Timers
        HAL_TIM_Base_Start_IT(&htim1);
        HAL_TIM_Base_Start_IT(&htim2);
    }

    // --- RENDER LOGIC ---
    // Only calculate when TIM2 says the frame has changed
    if(system_active && update_frame_flag) {
        PrecomputeFrame();
        update_frame_flag = 0;
    }
  }
}

/**
  * @brief  ISR for LED Refresh (TIM1)
  * @note   Extremely optimized register dump. No logic here.
  */
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
        // GPIOA->CRH = state->PortA_CRH; // DISABLED FOR SWD SAFETY
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

/**
  * @brief  Calculates the LUT for the current frame based on 8-bit input.
  * Runs in Main Loop.
  */
void PrecomputeFrame(void) {
    const uint32_t DEFAULT_CRL = 0x44444444; 
    const uint32_t DEFAULT_CRH = 0x44444444; 

    // SAFETY FOR PB12: Default CRH for Port B must set Pin 12 to Input Pull-Up/Down (0x8)
    const uint32_t PORTB_CRH_SAFE = 0x44484444; 
    const uint32_t PORTB_ODR_SAFE = (1 << 12); // Pull-Up selected

    HardwareRowDef rowHardware;

    // Pre-calculate the divisor for thresholding
    // For Depth 2 (4 levels): 256 / 4 = 64.
    uint8_t divisor = 256 / (1 << COLOR_DEPTH);

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
            for(int col = 0; col < COL_COUNT; col++) {

                // --- THRESHOLD LOGIC ---
                uint8_t rawBrightness = video_frames[current_frame_idx][row][col];

                // Map 0-255 to 0-3 (if depth 2)
                uint8_t pixelLevel = rawBrightness / divisor;

                // Extract BCM bit
                if( (pixelLevel >> bitPlane) & 0x01 ) {

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

// 4 MHz System Clock Config
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // 1. Use HSI (8 MHz) directly, no PLL needed for low speed
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // PLL Off
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  // 2. Set SysClk to HSI, divide AHB by 2 to get 4 MHz
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // 8 MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;     // 8 MHz / 2 = 4 MHz System Clock
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

void initBitDurationLUT(void){
    // Optimized for 4MHz Clock, Prescaler 49 (12.5us ticks)
    uint32_t multiplier = 15;

    for(uint32_t i = 0; i < COLOR_DEPTH; i++){
        bitDuration[i] = (1 << i) * multiplier;
    }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  // Prescaler 49: 4MHz / 50 = 80 KHz (12.5us per tick)
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure PB12 for Wakeup Button */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12) {
        last_activity_time = HAL_GetTick();
    }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
