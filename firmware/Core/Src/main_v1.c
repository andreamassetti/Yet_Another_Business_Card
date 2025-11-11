/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * @note
  * This code has been upgraded to implement a non-blocking, interrupt-driven
  * display driver with 8-bit Binary Code Modulation (BCM) for brightness
  * control, ported from the CH32V003 example.
  *
  * It uses the STM32F103-compatible functions (CRL/CRH registers)
  * from the original file.
  *
  * The SysTick_Handler is now defined in this file and will override the
  * weak definition in stm32f1xx_it.c.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // For memset
#include "core_cm3.h" // For DWT
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Structure to hold pin information
typedef struct {
    GPIO_TypeDef* port; // GPIO Port (e.g., GPIOA, GPIOB)
    uint8_t       pin_num;  // Pin number (0-15)
} GpioPin;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ====================================================================
 * MATRIX DIMENSIONS & GAME DEFINES
 * ==================================================================== */
#define MATRIX_ROWS 20
#define MATRIX_COLS 19
#define NUM_PINS 20 // 10 pins drive 10*9 = 90 LEDs

#define PADDLE_WIDTH 9
#define PADDLE_Y (MATRIX_ROWS - 1) // Paddle is on the last row
#define BRICK_ROWS 5
// Game logic now runs in the main loop, delayed by a DWT delay
// The display runs completely independently in the SysTick interrupt

/* ====================================================================
 * 8-BIT BCM (BRIGHTNESS) DRIVER DEFINES (Ported from CH32)
 * ==================================================================== */
#define COLOR_DEPTH 8           // 8-bit brightness (0-255)
#define COLOR_COUNT (1 << COLOR_DEPTH)
#define COLOR_MAX (COLOR_COUNT - 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ====================================================================
 * FRAME BUFFER (Now stores 8-bit brightness)
 * ==================================================================== */
uint8_t frame_buffer[MATRIX_ROWS][MATRIX_COLS] = {0};

/* ====================================================================
 * 8-BIT BCM (BRIGHTNESS) DRIVER STATE
 * (Ported from CH32)
 * ==================================================================== */
uint32_t bitDuration[COLOR_DEPTH]; // Holds the "on-time" for each bit-plane
static volatile int g_current_bit = 0;   // Which bit-plane we are drawing (0-7)
static volatile int g_current_anode = 0; // Which anode pin we are driving (0-9)

/* ====================================================================
 * PIN DEFINITIONS (From User)
 * ==================================================================== */
// Array mapping your 10 pins to their ports and numbers
const GpioPin matrix_pins[NUM_PINS] = {
    {GPIOB, 8},  // Pin 0: PB8
    {GPIOB, 9},  // Pin 1: PB9
    {GPIOC, 13}, // Pin 2: PC13
    {GPIOC, 14}, // Pin 3: PC14
    {GPIOC, 15}, // Pin 4: PC15
    {GPIOD, 0},  // Pin 5: PD0
    {GPIOD, 1},  // Pin 6: PD1
    {GPIOA, 0},  // Pin 7: PA0
    {GPIOA, 1},  // Pin 8: PA1
    {GPIOA, 2},  // Pin 9: PA2
    {GPIOA, 3},  // Pin 10: PA3
    {GPIOA, 4},  // Pin 11: PA4
    {GPIOA, 5},  // Pin 12: PA5
    {GPIOA, 6},  // Pin 13: PA6
    {GPIOA, 7},  // Pin 14: PA7
    {GPIOB, 0},  // Pin 15: PB0
    {GPIOB, 1},  // Pin 16: PB1
    {GPIOB, 2},  // Pin 17: PB2
    {GPIOB, 10}, // Pin 18: PB10
    {GPIOB, 11}  // Pin 19: PB11
};

/* ====================================================================
 * GAME STATE
 * ==================================================================== */
static uint8_t bricks[BRICK_ROWS][MATRIX_COLS];
static int ball_x, ball_y;
static int ball_vx, ball_vy; // velocity
static int paddle_x;
static uint8_t game_over = 0;
static uint32_t score = 0;
// game_tick_counter is removed, game speed is now controlled by main loop delay
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/* ====================================================================
 * USER FUNCTION PROTOTYPES
 * ==================================================================== */
// Core hardware drivers
void DWT_Delay_Init(void);
void DWT_Delay_us(volatile uint32_t us);
void enable_gpio_clocks(void);
void set_pin_input(GPIO_TypeDef* port, uint8_t pin_num);
void set_pin_output(GPIO_TypeDef* port, uint8_t pin_num);
void set_pin_high(GPIO_TypeDef* port, uint8_t pin_num);
void set_pin_low(GPIO_TypeDef* port, uint8_t pin_num);
void set_all_pins_input(void);
void initBCM(void);

// Button input
void init_button(void);
uint8_t read_button(void);

// Game logic
void game_init(void);
void update_game_state(uint32_t tick);
void draw_game_to_framebuffer(void);
void draw_game_over_screen(uint32_t tick);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ====================================================================
 * 1. TIMING FUNCTIONS (From User)
 * ==================================================================== */

void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

void DWT_Delay_us(volatile uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    // Note: HAL_RCC_GetHCLKFreq() will return 4,000,000 with your config
    uint32_t cycles_per_us = (HAL_RCC_GetHCLKFreq() / 1000000);
    uint32_t count = us * cycles_per_us;
    while ((DWT->CYCCNT - start) < count);
}

/* ====================================================================
 * 2. LOW-LEVEL GPIO REGISTER FUNCTIONS (From User - F103 Compatible)
 * ==================================================================== */

void enable_gpio_clocks(void) {
    // This is the correct register (APB2ENR) for STM32F103
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_IOPDEN;
}

void set_pin_input(GPIO_TypeDef* port, uint8_t pin_num) {
    if (pin_num < 8) { // Use CRL register
        uint8_t shift = pin_num * 4;
        port->CRL &= ~(0xF << shift);
        // Set to 0b0100: Input with floating
        // 0x0 is Analog, 0x4 is Floating Input, 0x8 is Input PU/PD
        // Let's use 0x4 (CNF=01, MODE=00) for Floating Input (Hi-Z)
        port->CRL |= (0x4 << shift);
    } else { // Use CRH register
        uint8_t shift = (pin_num - 8) * 4;
        port->CRH &= ~(0xF << shift);
        port->CRH |= (0x4 << shift);
    }
}

void set_pin_output(GPIO_TypeDef* port, uint8_t pin_num) {
    if (pin_num < 8) { // Use CRL register
        uint8_t shift = pin_num * 4;
        port->CRL &= ~(0xF << shift);
        // Set to 0b0011: GP Output Push-Pull, 50MHz
        // 0x3 (CNF=00, MODE=11)
        port->CRL |= (0x3 << shift);
    } else { // Use CRH register
        uint8_t shift = (pin_num - 8) * 4;
        port->CRH &= ~(0xF << shift);
        port->CRH |= (0x3 << shift);
    }
}

void set_pin_high(GPIO_TypeDef* port, uint8_t pin_num) {
    port->BSRR = (1 << pin_num);
}

void set_pin_low(GPIO_TypeDef* port, uint8_t pin_num) {
    // F103 has a dedicated BRR (Bit Reset Register)
    port->BRR = (1 << pin_num);
}

/**
 * @brief  Sets all 10 matrix pins to input (tri-state)
 */
void set_all_pins_input(void) {
    for (int i = 0; i < NUM_PINS; i++) {
        set_pin_input(matrix_pins[i].port, matrix_pins[i].pin_num);
    }
}

/* ====================================================================
 * 3. BCM (BRIGHTNESS) INIT
 * ==================================================================== */

/**
 * @brief  Initializes the bit-duration array for BCM.
 */
void initBCM(void) {
    // From your SystemClock_Config, HCLK is 4MHz
    // SysTick runs on HCLK by default (can be HCLK/8)
    // We will set SysTick to run on HCLK (4MHz)
    // Let's aim for a ~60Hz refresh rate

    // 4,000,000 cycles/sec / 60 Hz = 66,666 cycles per frame
    // 66,666 / 10 anodes = 6,666 cycles per anode scan
    // Total BCM "weight" = (2^0+1) + (2^1+1) ... + (2^7+1) = (255) + 8 = 263
    // 6,666 / 263 = 25.3 cycles per base unit
    const uint32_t TICK_DELAY_CYCLES = 25;

    for(uint32_t i = 0; i < COLOR_DEPTH; i++) {
        int dur = (1 << i);
        // The duration will be in raw CPU clock cycles
        bitDuration[i] = TICK_DELAY_CYCLES * (dur + 1);
    }
}

/* ====================================================================
 * 4. BUTTON HARDWARE FUNCTIONS (From User - F103 Compatible)
 * ==================================================================== */

/**
 * @brief Configures PB12 as an input with an internal pull-up resistor.
 */
void init_button(void)
{
    // enable_gpio_clocks() already enabled GPIOB

    // Configure PB12 (Pin 12)
    uint8_t pin_num = 12;
    uint8_t shift = (pin_num - 8) * 4; // Pin 12 is in CRH, shift is 16

    // Clear CNF and MODE bits for pin 12
    GPIOB->CRH &= ~(0xF << shift);

    // Set MODE = 00 (Input)
    // Set CNF = 10 (Input with pull-up / pull-down)
    GPIOB->CRH |= (0x8 << shift);

    // Enable the internal PULL-UP resistor for PB12
    // (Set the corresponding bit in ODR to 1)
    GPIOB->ODR |= (1 << pin_num);
}

/**
 * @brief Reads the button state.
 * @return 1 if pressed (pin is LOW), 0 if not pressed (pin is HIGH).
 */
uint8_t read_button(void)
{
    // Read the Input Data Register (IDR)
    // If the button is pressed, it pulls the pin LOW (0)
    if ((GPIOB->IDR & (1 << 12)) == 0)
    {
        return 1; // Pressed
    }
    else
    {
        return 0; // Not pressed
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  static uint32_t game_tick = 0;
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
  /* USER CODE BEGIN 2 */

  // 1. Enable clocks for all our GPIO ports
  enable_gpio_clocks();

  // 2. Initialize the high-precision delay timer for game logic
  DWT_Delay_Init();

  // 3. Initialize the button
  init_button();

  // 4. Initialize the BCM brightness durations
  initBCM();

  // 5. Initialize the game state (bricks, ball, paddle)
  game_init();

  // 6. Set all pins to input (Hi-Z) to start
  set_all_pins_input();

  // 7. ---- START THE INTERRUPT DRIVER ----
  // We override the default HAL_Init() SysTick config
  SysTick->LOAD = bitDuration[0] - 1; // Load duration for bit 0
  SysTick->VAL = 0;                   // Reset counter
  // Enable SysTick, core clock source (HCLK, not HCLK/8), and interrupt
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk   |
                  SysTick_CTRL_ENABLE_Msk;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 1. Update the game state.
    //    The display is updating itself in the background!
	update_game_state(game_tick++);

    // 2. We no longer call a blocking render_frame()!

    // 3. Add a small delay so the game logic
    //    doesn't run *too* fast.
    //    We MUST use DWT_Delay_us since HAL_Delay() is broken.
    DWT_Delay_us(5000); // 5ms delay, so game logic runs at ~200Hz

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* This function is from CubeMX and is fine.
   * We do our own GPIO init in main() for the matrix pins.
   */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ====================================================================
 * SysTick HANDLER (THE NEW DISPLAY ENGINE)
 * ==================================================================== */

/**
 * @brief This function handles System tick timer interrupt.
 * This is our non-blocking display driver.
 * NOTE: This function's definition will override the weak one in
 * stm32f1xx_it.c
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

    // --- 1. Turn OFF the previous anode pin ---
    // This is critical to prevent ghosting.
    set_pin_input(matrix_pins[g_current_anode].port, matrix_pins[g_current_anode].pin_num);

    // --- 2. Advance the scanner state ---
    g_current_anode++;
    if (g_current_anode == NUM_PINS) {
        g_current_anode = 0; // Wrap around anodes
        g_current_bit++;
        if (g_current_bit == COLOR_DEPTH) {
            g_current_bit = 0; // Wrap around bit-planes
        }
        // Set the timer's duration for the *next* bit-plane
        // We use SysTick->LOAD directly for fast, register-level control
        SysTick->LOAD = bitDuration[g_current_bit] - 1;
        SysTick->VAL = 0; // Reset counter
    }

    // --- 3. Set up the CATHODES for the new anode ---
    // We are now driving g_current_anode.
    // We check all 9 possible cathodes.
    uint8_t bit_mask = (1 << g_current_bit);

    for (int cathode_pin_index = 0; cathode_pin_index < NUM_PINS; cathode_pin_index++) {
        if (cathode_pin_index == g_current_anode) continue; // Skip self

        // Find the logical (x,y) pixel for this (anode, cathode) pair
        int row = g_current_anode;
        int col = (cathode_pin_index < g_current_anode) ? cathode_pin_index : (cathode_pin_index - 1);

        // Get the 8-bit brightness value from the frame buffer
        uint8_t brightness = frame_buffer[row][col];

        if (brightness & bit_mask) {
            // This bit is ON. Set pin as Output LOW.
            set_pin_output(matrix_pins[cathode_pin_index].port, matrix_pins[cathode_pin_index].pin_num);
            set_pin_low(matrix_pins[cathode_pin_index].port, matrix_pins[cathode_pin_index].pin_num);
        } else {
            // This bit is OFF. Tri-state the pin (Input).
            set_pin_input(matrix_pins[cathode_pin_index].port, matrix_pins[cathode_pin_index].pin_num);
        }
    }

    // --- 4. Turn ON the new anode pin ---
    // (All cathodes are set, now we provide power)
    set_pin_output(matrix_pins[g_current_anode].port, matrix_pins[g_current_anode].pin_num);
    set_pin_high(matrix_pins[g_current_anode].port, matrix_pins[g_current_anode].pin_num);

  /* USER CODE END SysTick_IRQn 0 */

  // DO NOT CALL HAL_IncTick()!
  // This interrupt is not running at 1ms.
  // HAL_IncTick();

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/* ====================================================================
 * GAME LOGIC FUNCTIONS (Upgraded for 8-bit brightness)
 * ==================================================================== */

/**
 * @brief Initializes/resets the game to the starting state.
 */
void game_init(void)
{
    // Set all bricks to be visible
    memset(bricks, 1, sizeof(bricks));

    // Place paddle in the middle
    paddle_x = (MATRIX_COLS / 2) - (PADDLE_WIDTH / 2);

    // Place ball just above the paddle
    ball_y = PADDLE_Y - 1;
    ball_x = MATRIX_COLS / 2;
    ball_vx = 1;  // Start moving right
    ball_vy = -1; // Start moving up

    // Reset state
    game_over = 0;
    score = 0;
}

/**
 * @brief Updates the game state for one logic tick.
 * This moves the paddle, ball, and checks for collisions.
 */
void update_game_state(uint32_t tick)
{
    // The main loop delay now controls game speed,
    // so we don't need the GAME_SPEED_DIVIDER logic.
    // We can just update every time this is called.

    // 1. Check for Game Over / Restart
    if (game_over)
    {
        if (read_button())
        {
            game_init();
        }
        draw_game_over_screen(tick); // Show flashing 'G'
        return;
    }

    // 2. Update Paddle Position
    // Button pressed = move right, Not pressed = move left
    if (read_button())
    {
        paddle_x++;
    }
    else
    {
        paddle_x--;
    }

    // 3. Constrain paddle to screen
    if (paddle_x < 0) paddle_x = 0;
    if (paddle_x > (MATRIX_COLS - PADDLE_WIDTH))
    {
        paddle_x = (MATRIX_COLS - PADDLE_WIDTH);
    }

    // 4. Update Ball Position
    ball_x += ball_vx;
    ball_y += ball_vy;

    // 5. Check Ball Collisions
    //  a. Left/Right Walls
    if (ball_x <= 0)
    {
        ball_x = 0;
        ball_vx = -ball_vx;
    }
    if (ball_x >= (MATRIX_COLS - 1))
    {
        ball_x = MATRIX_COLS - 1;
        ball_vx = -ball_vx;
    }

    //  b. Top Wall
    if (ball_y <= 0)
    {
        ball_y = 0;
        ball_vy = -ball_vy;
    }

    //  c. Floor (Game Over)
    if (ball_y >= (MATRIX_ROWS - 1))
    {
        game_over = 1;
        return;
    }

    //  d. Paddle
    if (ball_y == (PADDLE_Y - 1) &&   // Ball is at row above paddle
        ball_x >= paddle_x &&        // Ball is right of paddle's left edge
        ball_x < (paddle_x + PADDLE_WIDTH)) // Ball is left of paddle's right edge
    {
        ball_vy = -ball_vy; // Bounce
    }

    //  e. Bricks
    if (ball_y < BRICK_ROWS)
    {
        if (bricks[ball_y][ball_x] == 1)
        {
            bricks[ball_y][ball_x] = 0; // Break brick
            ball_vy = -ball_vy;         // Bounce
            score++;
        }
    }

    // 6. Draw the new game state to the frame_buffer
    draw_game_to_framebuffer();
}

/**
 * @brief Draws all game elements (bricks, ball, paddle) into the frame_buffer.
 * (Upgraded to write 8-bit brightness values)
 */
void draw_game_to_framebuffer(void)
{
    // 1. Clear the frame buffer
    memset(frame_buffer, 0, sizeof(frame_buffer));

    // 2. Draw bricks (dim)
    for (int r = 0; r < BRICK_ROWS; r++)
    {
        for (int c = 0; c < MATRIX_COLS; c++)
        {
            if (bricks[r][c])
            {
                frame_buffer[r][c] = 50; // Dim brightness for bricks
            }
        }
    }

    // 3. Draw paddle (medium)
    for (int i = 0; i < PADDLE_WIDTH; i++)
    {
        if((paddle_x + i) < MATRIX_COLS) // Safety check
        {
            frame_buffer[PADDLE_Y][paddle_x + i] = 150; // Medium brightness
        }
    }

    // 4. Draw ball (full bright)
    frame_buffer[ball_y][ball_x] = 255; // Full brightness
}

/**
 * @brief Draws a flashing "G" for "Game Over".
 * (Upgraded to write 8-bit brightness values)
 */
void draw_game_over_screen(uint32_t tick)
{
    memset(frame_buffer, 0, sizeof(frame_buffer));

    // Flash the 'G' every 40 logic ticks (5ms * 40 = 200ms)
    if ((tick / 40) % 2)
    {
        uint8_t bright = 200; // Brightness for 'G'
        int x = 2; // Centered X
        int y = 2; // Centered Y

        frame_buffer[y+0][x+1] = bright; frame_buffer[y+0][x+2] = bright;
        frame_buffer[y+1][x+0] = bright;
        frame_buffer[y+2][x+0] = bright; frame_buffer[y+2][x+3] = bright;
        frame_buffer[y+3][x+0] = bright; frame_buffer[y+3][x+3] = bright;
        frame_buffer[y+4][x+1] = bright; frame_buffer[y+4][x+2] = bright;
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
