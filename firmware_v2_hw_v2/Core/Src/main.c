/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Optimized Main program body for Charlieplexing (4MHz Low Power)
  * Updated for Hardware V2 Pin Assignments & 4-Button UI (19x18 Matrix)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "animations.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 1. STRUCTURES FOR LOOKUP TABLE
// Optimized: We only need to control Port B and Port C for the LEDs now.
typedef struct {
    uint32_t PortB_CRL;
    uint32_t PortB_CRH;
    uint32_t PortB_ODR;

    uint32_t PortC_CRH;
    uint32_t PortC_ODR;
} RowRegisterState;

typedef struct {
    GPIO_TypeDef* port;
    uint8_t pinIndex;
} PinDef;

typedef struct {
    PinDef anode;
    PinDef cathodes[18]; // 19 pins total -> 1 anode, 18 cathodes
} HardwareRowDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TICK_DELAY 0x3
// Animation playback rate, in frames per second. Set this to the FPS of the source
// video used to generate animation_data so playback matches the original speed.
// TIM2 is configured (in MX_TIM2_Init) to fire ANIM_FPS times per second and the
// frame index advances by one each tick.
#define ANIM_FPS 30
// End-of-animation heart: shown for this many frame ticks (~2 s at ANIM_FPS).
#define HEART_HOLD_TICKS (2u * ANIM_FPS)
// COLOR_DEPTH is defined in animations.h (data contract): 4 planes => 16 grey levels
#define COLOR_COUNT (1 << COLOR_DEPTH)
#define COLOR_MAX (COLOR_COUNT - 1)
// End-of-animation heart pulsation: breathes between half and full brightness.
#define HEART_PULSE_PERIOD (ANIM_FPS)         // ticks per pulse (~1 s => 2 pulses over 2 s)
#define HEART_MIN_LEVEL    ((COLOR_MAX + 1) / 2)  // ~half brightness (8 of 15)
#define HEART_MAX_LEVEL    (COLOR_MAX)         // full brightness (15)
#define ROW_COUNT 19
#define xres 18
#define yres 19
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile int frame = 0;
volatile int current_frame_idx = 0;
volatile uint8_t update_frame_flag = 1;

volatile uint8_t system_active = 1;
uint32_t last_activity_time = 0;

// Animation playthrough tracking (for "sleep after one playthrough").
uint8_t animation_complete = 0;    // set by the decoder once the last frame is decoded
uint8_t animation_done_shown = 0;  // set after the end-of-animation heart has been shown
uint16_t heart_ticks = 0;          // frame ticks the end-of-animation heart has been shown

// Double-buffered to prevent tearing: PrecomputeFrame fills the back buffer, then publishes
// it with a single atomic pointer write. The TIM1 ISR always reads a complete frame.
RowRegisterState frameBufferLUT[2][ROW_COUNT][COLOR_DEPTH];
RowRegisterState (* volatile displayBuf)[COLOR_DEPTH] = frameBufferLUT[0];
uint32_t bitDuration[COLOR_DEPTH];

// Current animation frame, decoded from the compressed stream (delta+RLE) into RAM.
// Plane-major (60 words): framebuf[plane][row]. PrecomputeFrame reads from here.
uint32_t framebuf[COLOR_DEPTH][Y_RES];

// --- VARIABILI SNAKE ---
#define MODE_ANIMATION 0
#define MODE_SNAKE 1
volatile uint8_t current_mode = MODE_ANIMATION;

#define MAX_SNAKE_LEN 100
// On death: blink the board for this long (500 ms period => ~3 blinks), then stay blank.
#define GAME_OVER_BLINK_MS 1500
// ...then enter standby this long after death, so the next press wakes back to the video.
#define GAME_OVER_SLEEP_MS 2000
// Snake greyscale levels (0..COLOR_MAX). Head brightest, body dim, apple/score mid-high.
#define SNAKE_HEAD_LEVEL  COLOR_MAX
#define SNAKE_BODY_LEVEL  5
#define APPLE_LEVEL       COLOR_MAX
#define SCORE_LEVEL       9
int8_t snake_x[MAX_SNAKE_LEN];
int8_t snake_y[MAX_SNAKE_LEN];
int snake_len = 3;
int8_t dir_x = 0;
int8_t dir_y = -1; // Direzione CORRENTE (cambiata solo al passo, in UpdateSnake)
// Queued direction from the buttons, applied at the next step. A single byte
// (0=UP,1=DOWN,2=LEFT,3=RIGHT) so the button ISR updates it atomically (no torn read).
volatile int8_t pend_dir = 0;
int8_t apple_x = 5;
int8_t apple_y = 5;
volatile uint8_t game_over = 0;
uint32_t game_over_time = 0;   // HAL_GetTick() when the player died (for the short blink window)
// Snake graphics are rendered into the shared bit-plane buffer `framebuf` (greyscale),
// not a separate 1-bit board, so the head/body/apple can have different brightness levels.

// 3. HARDWARE PIN MAPPING (19 Pins Total)
// PB0-PB15, PC13-PC15 based on the v2 text file mapping
const PinDef ALL_PINS[19] = {
    {GPIOB, 0},  {GPIOB, 1},  {GPIOB, 2},  {GPIOB, 3},
    {GPIOB, 4},  {GPIOB, 5},  {GPIOB, 6},  {GPIOB, 7},
    {GPIOB, 8},  {GPIOB, 9},  {GPIOB, 10}, {GPIOB, 11},
    {GPIOB, 12}, {GPIOB, 13}, {GPIOB, 14}, {GPIOB, 15},
    {GPIOC, 13}, {GPIOC, 14}, {GPIOC, 15}
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

// Delta+RLE animation decoder
void DecodeReset(void);
void DecodeNextFrame(void);

// End-of-animation heart (drawn at brightness 0..COLOR_MAX)
void RenderHeart(uint8_t level);

// Prototipi Snake
void InitSnake(void);
void SpawnApple(void);
void UpdateSnake(void);
void RenderSnake(void);

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

  DecodeReset();
  DecodeNextFrame();   // decode frame 0 so the first displayed frame is correct
  PrecomputeFrame();

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  last_activity_time = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // --- SLEEP POLICY ---
	  // Animation mode: sleep once, after a single full playthrough has been shown.
	  // Snake mode:     after a game over, sleep shortly after the blink (so the next button
	  //                 press wakes back to the video); otherwise sleep after 10 s idle.
	  uint8_t should_sleep;
	  if (current_mode == MODE_ANIMATION) {
	      should_sleep = animation_done_shown;
	  } else if (game_over) {
	      should_sleep = (HAL_GetTick() - game_over_time > GAME_OVER_SLEEP_MS);
	  } else {
	      should_sleep = (HAL_GetTick() - last_activity_time > 10000);
	  }
	  if(system_active && should_sleep){
		  system_active = 0;

		  // Disable Timers to stop interrupts
		  HAL_TIM_Base_Stop_IT(&htim1);
		  HAL_TIM_Base_Stop_IT(&htim2);

		  // Turn off all LEDs (Reset ODRs to safe low state)
		  GPIOB->ODR = 0;
		  GPIOC->ODR = 0;

      //		  // Enter Stop Mode
      //		  HAL_SuspendTick();
      //		  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
      //
      //		  // --- WAKE UP POINT ---
      //		  SystemClock_Config(); // Restore 4 MHz clock
      //		  HAL_ResumeTick();
      //
      //		  system_active = 1;
      //		  last_activity_time = HAL_GetTick();
      //
      //		  // Restart Timers
      //		  HAL_TIM_Base_Start_IT(&htim1);
      //		  HAL_TIM_Base_Start_IT(&htim2);

		  // 1. Clear the Wake-Up flag (Ensures we don't wake up instantly)
		  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

		  // 2. Enable the Wake-Up pin (PA0)
		  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

		  // 3. Enter Standby Mode
		  HAL_PWR_EnterSTANDBYMode();

		  // NOTE: The MCU will completely power down here.
		  // It will NEVER execute code past this line.
		  // Upon waking, it will reboot and start from the top of main()
	  }

	  // --- RENDER LOGIC ---
	  // Only calculate when TIM2 says the frame has changed
	  // if(system_active && update_frame_flag) {
		//   PrecomputeFrame();
		//   update_frame_flag = 0;
	  // }

    if(system_active && update_frame_flag) {
      if (current_mode == MODE_SNAKE) {
        static int snake_speed_divider = 0;
        // Game logic steps ~3x/sec; rendering runs every tick so the apple blinks smoothly.
        if (++snake_speed_divider >= 10) {
          UpdateSnake();
          snake_speed_divider = 0;
        }
        RenderSnake();
        PrecomputeFrame();
      } else if (animation_complete) {
        // Animation finished: show the heart for ~2 s while pulsing (breathing) between half
        // and full brightness, then flag for sleep. Re-rendered each tick (brightness changes).
        uint16_t phase = heart_ticks % HEART_PULSE_PERIOD;
        uint16_t half  = HEART_PULSE_PERIOD / 2u;
        uint16_t tri   = (phase < half) ? phase : (HEART_PULSE_PERIOD - phase);  // 0..half triangle
        uint8_t  level = HEART_MIN_LEVEL +
                         (uint8_t)(((HEART_MAX_LEVEL - HEART_MIN_LEVEL) * tri) / half);
        RenderHeart(level);
        PrecomputeFrame();
        if (++heart_ticks >= HEART_HOLD_TICKS) animation_done_shown = 1;
      } else {
        DecodeNextFrame();   // advance one frame (sets animation_complete after the last)
        PrecomputeFrame();
      }
      update_frame_flag = 0;
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
  // TIM2 clock = 4 MHz. Prescaler 99 -> 40 kHz tick. Period = 40000/ANIM_FPS - 1
  // gives an update (interrupt) ANIM_FPS times per second; one frame advances per tick.
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (40000U / ANIM_FPS) - 1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Lean TIM1 display update: runs from the bare-metal TIM1_UP_IRQHandler (NOT via HAL) so
// the per-interrupt overhead is a few microseconds instead of ~15-25 us. That low overhead
// is what lets the LSB bit-plane be short enough to fit 4 planes (16 levels) at ~60 Hz with
// good low-brightness linearity. Called once per (row, bit-plane) display slot.
void TIM1_DisplayUpdate(void)
{
    static int row = 0;
    static int bit = 0;

    RowRegisterState *state = &displayBuf[row][bit];

    // --- BLANKING E AGGIORNAMENTO SICURO ---
    // 1. Spegni tutti i LED mettendo le porte in Input Fluttuante (High-Z)
    // 0x44444444 è il reset value per Floating Input sull'STM32F1
    GPIOB->CRL = 0x44444444;
    GPIOB->CRH = 0x44444444;
    GPIOC->CRH = 0x44444444;

    // 2. Pre-carica i nuovi valori logici mentre i pin sono scollegati
    GPIOB->ODR = state->PortB_ODR;
    GPIOC->ODR = state->PortC_ODR;

    // 3. Attiva la nuova configurazione delle direzioni (Accende i nuovi LED)
    GPIOB->CRL = state->PortB_CRL;
    GPIOB->CRH = state->PortB_CRH;
    GPIOC->CRH = state->PortC_CRH;

    // 4. Advance Counters
    row++;
    if(row >= ROW_COUNT)
    {
        row = 0;
        bit++;
        if(bit >= COLOR_DEPTH) bit = 0;

        TIM1->ARR = bitDuration[bit];
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    // TIM1 is handled by the lean bare-metal ISR (TIM1_DisplayUpdate); only TIM2 is here.
    if(htim->Instance == TIM2)
    {
        // TIM2 fires ANIM_FPS times per second. The main loop decodes + advances one
        // animation frame per tick, so playback runs at the source video's frame rate.
        update_frame_flag = 1;
    }
}

void PrecomputeFrame(void) {
    const uint32_t DEFAULT_CRL = 0x44444444;
    const uint32_t DEFAULT_CRH = 0x44444444;

    static int backIdx = 1;                                    // buffer NOT being displayed
    RowRegisterState (*buf)[COLOR_DEPTH] = frameBufferLUT[backIdx];

    // Decide the game-over blink once per frame (not per pixel) so a row can't be half-blanked.
    // On death, blink for GAME_OVER_BLINK_MS (~3 flashes); after that, stay blank.
    uint8_t go_blank = 0;
    if (current_mode == MODE_SNAKE && game_over) {
        uint32_t go_elapsed = HAL_GetTick() - game_over_time;
        go_blank = (go_elapsed >= GAME_OVER_BLINK_MS) || (HAL_GetTick() % 500 < 250);
    }

    HardwareRowDef rowHardware;

    for(int row = 0; row < ROW_COUNT; row++) {
        GetHardwareRow(row, &rowHardware);

        for(int bitPlane = 0; bitPlane < COLOR_DEPTH; bitPlane++) {

            RowRegisterState *state = &buf[row][bitPlane];

            // 1. Reset State to Safe Defaults (Inputs floating)
            state->PortB_CRL = DEFAULT_CRL;
            state->PortB_CRH = DEFAULT_CRH;
            state->PortB_ODR = 0;

            state->PortC_CRH = DEFAULT_CRH;
            state->PortC_ODR = 0;

            // 2. Configure ANODE (Active High)
            PinDef anode = rowHardware.anode;

            // Determine Port Pointers for B & C
            uint32_t *crl = (anode.pinIndex < 8) ? ((anode.port == GPIOB) ? &state->PortB_CRL : NULL)
                                                 : ((anode.port == GPIOB) ? &state->PortB_CRH : &state->PortC_CRH);

            uint32_t *odr = (anode.port == GPIOB) ? &state->PortB_ODR : &state->PortC_ODR;

            if(crl) {
                uint8_t shift = (anode.pinIndex % 8) * 4;
                *crl &= ~(0xF << shift);
                *crl |=  (0x2 << shift); // Output 2MHz Push-Pull
                *odr |=  (1 << anode.pinIndex); // Set HIGH
            }

            // 3. Configure CATHODES (Active Low)

			// Greyscale (Binary Code Modulation): for THIS bit plane, light only the pixels
			// whose brightness has bit `bitPlane` set. The data is pre-decomposed into planes.
			uint32_t rowData;
            if (current_mode == MODE_ANIMATION) {
                // Applica lo specchio sull'asse Y per orientare correttamente l'animazione
                // (row=0 su scheda = in basso, row=0 su array = in alto)
                rowData = framebuf[bitPlane][(ROW_COUNT - 1) - row];
            } else {
                // Gioco: la grafica (con luminosità per testa/corpo/mela) è renderizzata
                // nei piani di bit di framebuf da RenderSnake(); stessa orientazione di prima.
                rowData = go_blank ? 0u : framebuf[bitPlane][row];
            }

            for(int col = 0; col < xres; col++) {
                if((rowData >> col) & 0x01) {
                    PinDef cathode = rowHardware.cathodes[col];

                    uint32_t *c_crl = (cathode.pinIndex < 8) ? ((cathode.port == GPIOB) ? &state->PortB_CRL : NULL)
                                                             : ((cathode.port == GPIOB) ? &state->PortB_CRH : &state->PortC_CRH);

                    uint32_t *c_odr = (cathode.port == GPIOB) ? &state->PortB_ODR : &state->PortC_ODR;

                    if(c_crl) {
                        uint8_t c_shift = (cathode.pinIndex % 8) * 4;
                        *c_crl &= ~(0xF << c_shift);
                        *c_crl |=  (0x2 << c_shift); // Output 2MHz
                        *c_odr &= ~(1 << cathode.pinIndex); // Set LOW
                    }
                }
            }
        }
    }

    // Publish the finished frame atomically (single aligned pointer write) and flip buffers.
    displayBuf = buf;
    backIdx ^= 1;
}

// Helper to map Row Index -> Hardware Pins (1 Anode, 18 Cathodes)
void GetHardwareRow(int rowIndex, HardwareRowDef *rowDef) {
    int anodeIndex = 18 - rowIndex; // Reverse order mapping as done previously
    rowDef->anode = ALL_PINS[anodeIndex];

    int cathodeCount = 0;
    for (int i = 0; i < 19; i++) {
        if (i == anodeIndex) continue;
        rowDef->cathodes[cathodeCount++] = ALL_PINS[i];
    }
}

// --- Delta+RLE animation decoder -------------------------------------------------
// Frames are stored as a temporal XOR delta (vs the previous frame) that is run-length
// encoded. Decoding is sequential: each call advances framebuf by one frame. The frame
// before frame 0 is all-zero, so on loop we just clear framebuf and rewind to byte 0.
//   control byte: bit7=0 -> zero run of (low7+1) unchanged words
//                 bit7=1 -> literal run of (low7+1) changed words, 3 bytes each (24-bit LE)
static uint32_t rle_pos = 0;        // read cursor into anim_rle[]
static uint16_t frames_decoded = 0;

void DecodeReset(void) {
    uint32_t *w = (uint32_t*)framebuf;
    for (int i = 0; i < COLOR_DEPTH * Y_RES; i++) w[i] = 0;
    rle_pos = 0;
    frames_decoded = 0;
    animation_complete = 0;
    animation_done_shown = 0;
    heart_ticks = 0;
}

// Draw a filled 6x6 heart in the bottom-right corner, into framebuf, at brightness `level`
// (0..COLOR_MAX). Heart rows are 6-bit patterns (bit 0 = left), placed flush to the corner:
//   columns -> bits 12..17 (rightmost 6 of the 18 columns)
//   rows    -> framebuf rows 13..18 (bottom 6 of 19; framebuf row 0 = top after the Y mirror)
// Brightness via BCM: set the pixel on plane p only if bit p of `level` is set.
void RenderHeart(uint8_t level) {
    static const uint16_t heart[6] = {
        0x12, // .#..#.
        0x3F, // ######
        0x3F, // ######
        0x3F, // ######
        0x1E, // .####.
        0x0C  // ..##..
    };
    uint32_t *w = (uint32_t*)framebuf;
    for (int i = 0; i < COLOR_DEPTH * Y_RES; i++) w[i] = 0;   // clear screen
    for (int r = 0; r < 6; r++) {
        uint32_t rowbits = ((uint32_t)heart[r]) << 12;         // shift to the right edge
        for (int p = 0; p < COLOR_DEPTH; p++) {
            if ((level >> p) & 1u) {
                framebuf[p][13 + r] |= rowbits;                // bottom rows; plane set per brightness bit
            }
        }
    }
}

void DecodeNextFrame(void) {
    if (frames_decoded >= ANIMATION_FRAMES) DecodeReset();   // loop the animation

    uint32_t *w = (uint32_t*)framebuf;   // 60 words, plane-major
    int idx = 0;
    while (idx < COLOR_DEPTH * Y_RES) {
        uint8_t ctrl = anim_rle[rle_pos++];
        int cnt = (ctrl & 0x7F) + 1;
        if (ctrl & 0x80) {               // literal run: cnt changed words
            while (cnt--) {
                uint32_t v = (uint32_t)anim_rle[rle_pos]
                           | ((uint32_t)anim_rle[rle_pos + 1] << 8)
                           | ((uint32_t)anim_rle[rle_pos + 2] << 16);
                rle_pos += 3;
                w[idx++] ^= v;           // apply XOR delta against previous frame
            }
        } else {                          // zero run: cnt unchanged words
            idx += cnt;
        }
    }
    frames_decoded++;
    if (frames_decoded >= ANIMATION_FRAMES) animation_complete = 1;  // last frame decoded
}

void initBitDurationLUT(void){
    // Binary Code Modulation: each plane is shown for a time proportional to its bit weight.
    // Timer tick = 12.5us (TIM1 @ 80kHz). The total display time per frame scales as
    // (2^COLOR_DEPTH - 1) * base * ROW_COUNT.  With base 5 and COLOR_DEPTH 4:
    //   planes last 62.5 / 125 / 250 / 500 us; frame = 19 * (5+10+20+40) ticks * 12.5us
    //   ~= 17.8 ms => ~56 Hz (no visible flicker). The lean TIM1 ISR keeps the 62.5us
    //   LSB linear (interrupt overhead is now a few us, not ~15-25us).
    // (For COLOR_DEPTH 3 a base of 10 gives the same ~60 Hz.)
    const uint32_t base = (COLOR_DEPTH >= 4) ? 5u : 10u;
    for(uint32_t i = 0; i < COLOR_DEPTH; i++){
        bitDuration[i] = base * (1 << i);
    }
}

// Update activity timer when ANY button (PA0-PA4) triggers an EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // // PA0-PA4 are the newly assigned buttons
    // if (GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_3 || GPIO_Pin == GPIO_PIN_4)
    // {
    //     last_activity_time = HAL_GetTick();
    // }

	// Solo sui trigger di pressione positiva (evita di triggerare al rilascio)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_SET) 
    {
        last_activity_time = HAL_GetTick();

        if (current_mode == MODE_ANIMATION) {
            // Qualsiasi pulsante preme avvia il gioco
            current_mode = MODE_SNAKE;
            InitSnake();
        } else {
            // Dopo un game over i pulsanti non fanno nulla: la scheda va in standby (~2 s)
            // e la pressione successiva la risveglia facendo ripartire il video.
            if (game_over) {
                return;
            }

            // NOTA: Adatta l'assegnazione PA1-PA4 a Su/Giù/Sx/Dx a seconda del tuo layout fisico.
            // Set the QUEUED direction only; UpdateSnake() commits it at the next step and
            // rejects 180-degree reversals, so quick multi-presses can't kill the snake.
            if (GPIO_Pin == GPIO_PIN_2) pend_dir = 0; // UP
            if (GPIO_Pin == GPIO_PIN_3) pend_dir = 1; // DOWN
            if (GPIO_Pin == GPIO_PIN_1) pend_dir = 2; // LEFT
            if (GPIO_Pin == GPIO_PIN_4) pend_dir = 3; // RIGHT
        }
    }
}

void SpawnApple(void) {
    static uint32_t prng_state = 0x12345678;
    // Iniettiamo entropia ad ogni chiamata in base al tempo esatto
    prng_state ^= HAL_GetTick(); 
    
    for (int tries = 0; tries < 200; tries++) {
        // Generatore Xorshift pseudo-casuale a 32 bit
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;

        // Estraiamo bit completamente indipendenti per X e Y, senza correlazioni
        apple_x = (prng_state & 0xFF) % (xres / 2);
        apple_y = ((prng_state >> 8) & 0xFF) % (ROW_COUNT / 2);

        // Evita di spawnare la mela sotto al punteggio (in alto a destra)
        if (apple_x >= 5 && apple_y >= 6) continue;

        // Evita di spawnare la mela sopra il serpente
        int on_snake = 0;
        for (int i = 0; i < snake_len; i++) {
            if (snake_x[i] == apple_x && snake_y[i] == apple_y) { on_snake = 1; break; }
        }
        if (on_snake) continue;

        return; // Posizione valida trovata
    }
    // Fallback (scacchiera quasi piena): accetta l'ultima posizione generata.
}

void InitSnake(void) {
    snake_len = 3;
    // Partenza al centro dello schermo logico (x=4, y=4)
    snake_x[0] = 4; snake_y[0] = 4;
    snake_x[1] = 4; snake_y[1] = 5;
    snake_x[2] = 4; snake_y[2] = 6;
    dir_x = 0; dir_y = -1; pend_dir = 0; // Muove verso l'alto
    SpawnApple();
    game_over = 0;
}

void UpdateSnake(void) {
    if (game_over) return;

    // Apply the queued direction once per step, but never a 180-degree reversal into the
    // neck. Validating here (against the committed dir) makes fast multi-presses between
    // steps safe -- they can't flip the snake back on itself.
    static const int8_t DIRX[4] = { 0,  0, -1,  1 };  // UP, DOWN, LEFT, RIGHT
    static const int8_t DIRY[4] = {-1,  1,  0,  0 };
    int8_t ndx = DIRX[pend_dir], ndy = DIRY[pend_dir];
    if (!(ndx == -dir_x && ndy == -dir_y)) {
        dir_x = ndx; dir_y = ndy;
    }

    // 1. Muovi il corpo
    for (int i = snake_len - 1; i > 0; i--) {
        snake_x[i] = snake_x[i-1];
        snake_y[i] = snake_y[i-1];
    }

    // 2. Muovi la testa
    snake_x[0] += dir_x;
    snake_y[0] += dir_y;

    // 3. Collisione con i muri
    if (snake_x[0] < 0 || snake_x[0] >= (xres / 2) || snake_y[0] < 0 || snake_y[0] >= (ROW_COUNT / 2)) {
        game_over = 1;
        game_over_time = HAL_GetTick();
        return;
    }

    // 4. Collisione con se stesso
    for (int i = 1; i < snake_len; i++) {
        if (snake_x[0] == snake_x[i] && snake_y[0] == snake_y[i]) {
            game_over = 1;
            game_over_time = HAL_GetTick();
            return;
        }
    }

    // 5. Mangia la mela
    if (snake_x[0] == apple_x && snake_y[0] == apple_y) {
        if (snake_len < MAX_SNAKE_LEN) {
            // Seed the new tail segment on top of the current tail, otherwise it would be
            // drawn at a stale/garbage position for one step (a stray lit cell). It
            // separates naturally on the next move.
            snake_x[snake_len] = snake_x[snake_len - 1];
            snake_y[snake_len] = snake_y[snake_len - 1];
            snake_len++;
        }
        SpawnApple();
    }

    // Rendering happens every tick in RenderSnake(); UpdateSnake() is game logic only.
}

// ---- Snake rendering (greyscale) -------------------------------------------------
// Draw helpers write into the shared bit-plane buffer `framebuf` at an exact brightness
// level (0..COLOR_MAX). Snake uses the UNMIRRORED orientation that PrecomputeFrame's game
// branch reads (framebuf[plane][row]).
static void PlotPixel(int x, int y, uint8_t level) {
    uint32_t m = 1u << x;
    for (int p = 0; p < COLOR_DEPTH; p++) {
        if ((level >> p) & 1u) framebuf[p][y] |=  m;
        else                   framebuf[p][y] &= ~m;
    }
}

// Logical game cell (lx,ly) -> 2x2 pixel block.
static void PlotCell(int lx, int ly, uint8_t level) {
    uint32_t m = 3u << (lx * 2);
    int r0 = ly * 2, r1 = ly * 2 + 1;
    for (int p = 0; p < COLOR_DEPTH; p++) {
        if ((level >> p) & 1u) { framebuf[p][r0] |=  m; framebuf[p][r1] |=  m; }
        else                   { framebuf[p][r0] &= ~m; framebuf[p][r1] &= ~m; }
    }
}

void RenderSnake(void) {
    static const uint8_t font3x5[10][5] = {
        {0x7, 0x5, 0x5, 0x5, 0x7}, {0x2, 0x3, 0x2, 0x2, 0x7},
        {0x7, 0x4, 0x7, 0x1, 0x7}, {0x7, 0x4, 0x7, 0x4, 0x7},
        {0x5, 0x5, 0x7, 0x4, 0x4}, {0x7, 0x1, 0x7, 0x4, 0x7},
        {0x7, 0x1, 0x7, 0x5, 0x7}, {0x7, 0x4, 0x4, 0x4, 0x4},
        {0x7, 0x5, 0x7, 0x5, 0x7}, {0x7, 0x5, 0x7, 0x4, 0x7}
    };

    // Clear the frame buffer.
    uint32_t *w = (uint32_t*)framebuf;
    for (int i = 0; i < COLOR_DEPTH * Y_RES; i++) w[i] = 0;

    // Apple: blink so it reads differently from the body.
    if ((HAL_GetTick() % 500) < 350) PlotCell(apple_x, apple_y, APPLE_LEVEL);

    // Body (dim) then head (bright) so the head wins any overlap.
    for (int i = 1; i < snake_len; i++) PlotCell(snake_x[i], snake_y[i], SNAKE_BODY_LEVEL);
    PlotCell(snake_x[0], snake_y[0], SNAKE_HEAD_LEVEL);

    // Score, top-right, single-pixel 3x5 font, drawn last (sits on top).
    int score = snake_len - 3;
    if (score < 0) score = 0;
    if (score > 99) score = 99;
    int tens = score / 10, ones = score % 10;
    for (int r = 0; r < 5; r++) {
        int by = 18 - r;                 // top rows (unmirrored)
        for (int c = 0; c < 3; c++) {
            if (score >= 10 && ((font3x5[tens][r] >> c) & 1)) PlotPixel(11 + c, by, SCORE_LEVEL);
            if ((font3x5[ones][r] >> c) & 1)                  PlotPixel(15 + c, by, SCORE_LEVEL);
        }
    }
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
