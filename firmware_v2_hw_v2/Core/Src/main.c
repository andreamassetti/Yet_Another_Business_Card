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
#define COLOR_DEPTH 2
#define COLOR_COUNT (1 << COLOR_DEPTH)
#define COLOR_MAX (COLOR_COUNT - 1)
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

RowRegisterState frameBufferLUT[ROW_COUNT][COLOR_DEPTH];
uint32_t bitDuration[COLOR_DEPTH];

// --- VARIABILI SNAKE ---
#define MODE_ANIMATION 0
#define MODE_SNAKE 1
volatile uint8_t current_mode = MODE_ANIMATION;

#define MAX_SNAKE_LEN 100
int8_t snake_x[MAX_SNAKE_LEN];
int8_t snake_y[MAX_SNAKE_LEN];
int snake_len = 3;
int8_t dir_x = 0;
int8_t dir_y = -1; // Inizia muovendosi verso l'alto
int8_t apple_x = 5;
int8_t apple_y = 5;
uint32_t snake_board[ROW_COUNT]; // Buffer 1D per la matrice 19x18
volatile uint8_t game_over = 0;

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

// Prototipi Snake
void InitSnake(void);
void SpawnApple(void);
void UpdateSnake(void);

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

	  // --- SLEEP LOGIC (10 Seconds Timeout) ---
	  if(system_active && (HAL_GetTick() - last_activity_time > 10000)){
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
        // Rallenta il frame rate del serpente: esegue l'update circa ogni 15 frame (3.3Hz)
        if (++snake_speed_divider >= 15) {
          UpdateSnake();
          snake_speed_divider = 0;
        }
      }
      PrecomputeFrame();
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    static int row = 0;
    static int bit = 0;

    if(htim->Instance == TIM1)
    {
        RowRegisterState *state = &frameBufferLUT[row][bit];

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

    if(htim->Instance == TIM2)
    {
        static int anim_speed_divider = 0;
        
        // I primi 150 frame scorrono molto più lenti (divider a 4), i restanti tornano a velocità normale (divider a 1)
        int current_divider = (current_frame_idx < 150) ? 4 : 1;
        
        if (++anim_speed_divider >= current_divider) {
            current_frame_idx++;
            if (current_frame_idx >= ANIMATION_FRAMES) {
                current_frame_idx = 0;
            }
            anim_speed_divider = 0;
        }
        update_frame_flag = 1; // Mantieni il tick per il resto del sistema
    }
}

void PrecomputeFrame(void) {
    const uint32_t DEFAULT_CRL = 0x44444444;
    const uint32_t DEFAULT_CRH = 0x44444444;

    HardwareRowDef rowHardware;

    for(int row = 0; row < ROW_COUNT; row++) {
        GetHardwareRow(row, &rowHardware);

        for(int bitPlane = 0; bitPlane < COLOR_DEPTH; bitPlane++) {

            RowRegisterState *state = &frameBufferLUT[row][bitPlane];

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

			// OLD implementation
            // uint32_t rowData = animation_data[current_frame_idx][row];

			// NEW implementation
			uint32_t rowData;
            if (current_mode == MODE_ANIMATION) {
                // Applica lo specchio sull'asse Y per orientare correttamente l'animazione
                // (row=0 su scheda = in basso, row=0 su array = in alto)
                rowData = animation_data[current_frame_idx][(ROW_COUNT - 1) - row];
            } else {
                // Modalità Gioco
                if (game_over && (HAL_GetTick() % 500 < 250)) {
                    rowData = 0; // Lampeggio dello schermo intero al Game Over
                } else {
                    rowData = snake_board[row];
                }
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

void initBitDurationLUT(void){
    for(uint32_t i = 0; i < COLOR_DEPTH; i++){
        bitDuration[i] = 20 * (1 << i);
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
            // Se in game over, un pulsante riavvia
            if (game_over) {
                InitSnake();
                return;
            }

            // NOTA: Adatta l'assegnazione PA1-PA4 a Su/Giù/Sx/Dx a seconda del tuo layout fisico. 
            // Questa è un'ipotesi basata su 4 tasti liberi (PA1-PA4).
            if (GPIO_Pin == GPIO_PIN_2 && dir_y != 1)  { dir_x = 0; dir_y = -1; } // UP
            if (GPIO_Pin == GPIO_PIN_3 && dir_y != -1) { dir_x = 0; dir_y = 1; }  // DOWN
            if (GPIO_Pin == GPIO_PIN_1 && dir_x != 1)  { dir_x = -1; dir_y = 0; } // LEFT
            if (GPIO_Pin == GPIO_PIN_4 && dir_x != -1) { dir_x = 1; dir_y = 0; }  // RIGHT
        }
    }
}

void SpawnApple(void) {
    static uint32_t prng_state = 0x12345678;
    // Iniettiamo entropia ad ogni chiamata in base al tempo esatto
    prng_state ^= HAL_GetTick(); 
    
    while (1) {
        // Generatore Xorshift pseudo-casuale a 32 bit
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;
        
        // Estraiamo bit completamente indipendenti per X e Y, senza correlazioni
        apple_x = (prng_state & 0xFF) % (xres / 2);
        apple_y = ((prng_state >> 8) & 0xFF) % (ROW_COUNT / 2);
        
        // Evita di spawnare la mela sotto al punteggio (in alto a destra)
        if (apple_x >= 5 && apple_y >= 6) {
            continue; // Ri-esegui il ciclo per un nuovo numero
        } else {
            break; // Posizione valida trovata
        }
    }
}

void InitSnake(void) {
    snake_len = 3;
    // Partenza al centro dello schermo logico (x=4, y=4)
    snake_x[0] = 4; snake_y[0] = 4;
    snake_x[1] = 4; snake_y[1] = 5;
    snake_x[2] = 4; snake_y[2] = 6;
    dir_x = 0; dir_y = -1; // Muove verso l'alto
    SpawnApple();
    game_over = 0;
}

void UpdateSnake(void) {
    static const uint8_t font3x5[10][5] = {
        {0x7, 0x5, 0x5, 0x5, 0x7}, // 0
        {0x2, 0x3, 0x2, 0x2, 0x7}, // 1
        {0x7, 0x4, 0x7, 0x1, 0x7}, // 2
        {0x7, 0x4, 0x7, 0x4, 0x7}, // 3
        {0x5, 0x5, 0x7, 0x4, 0x4}, // 4
        {0x7, 0x1, 0x7, 0x4, 0x7}, // 5
        {0x7, 0x1, 0x7, 0x5, 0x7}, // 6
        {0x7, 0x4, 0x4, 0x4, 0x4}, // 7
        {0x7, 0x5, 0x7, 0x5, 0x7}, // 8
        {0x7, 0x5, 0x7, 0x4, 0x7}  // 9
    };

    if (game_over) return;

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
        return;
    }

    // 4. Collisione con se stesso
    for (int i = 1; i < snake_len; i++) {
        if (snake_x[0] == snake_x[i] && snake_y[0] == snake_y[i]) {
            game_over = 1;
            return;
        }
    }

    // 5. Mangia la mela
    if (snake_x[0] == apple_x && snake_y[0] == apple_y) {
        if (snake_len < MAX_SNAKE_LEN) snake_len++;
        SpawnApple();
    }

    // 6. Rendi su snake_board
    for(int i = 0; i < ROW_COUNT; i++) snake_board[i] = 0;

    // Disegna la mela (lampeggiante per distinguerla dal serpente)
    if ((HAL_GetTick() % 250) > 125) {
        snake_board[apple_y * 2] |= (3 << (apple_x * 2));
        snake_board[apple_y * 2 + 1] |= (3 << (apple_x * 2));
    }

    // Disegna il serpente
    for (int i = 0; i < snake_len; i++) {
        snake_board[snake_y[i] * 2] |= (3 << (snake_x[i] * 2));
        snake_board[snake_y[i] * 2 + 1] |= (3 << (snake_x[i] * 2));
    }

    // Disegna il punteggio in alto a destra
    int score = snake_len - 3;
    if (score < 0) score = 0;
    if (score > 99) score = 99; // Cap at 99
    
    int tens = score / 10;
    int ones = score % 10;
    
    for (int r = 0; r < 6; r++) {
        int board_y = 18 - r; // Inverti l'asse Y: 18 è la riga superiore
        // Pulisci area sfondo punteggio (7 pixel di larghezza, x=11..17)
        snake_board[board_y] &= ~(0x7F << 11); 
        if (r < 5) {
            if (score >= 10) {
                snake_board[board_y] |= ((uint32_t)font3x5[tens][r]) << 11;
            }
            snake_board[board_y] |= ((uint32_t)font3x5[ones][r]) << 15;
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
