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
#include <string.h> // For memset
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ====================================================================
 * MATRIX DIMENSIONS & GAME DEFINES
 * ==================================================================== */
#define MATRIX_ROWS 10
#define MATRIX_COLS 9

#define PADDLE_WIDTH 9
#define PADDLE_Y (MATRIX_ROWS - 1) // Paddle is on the last row
#define BRICK_ROWS 5
#define GAME_SPEED_DIVIDER 5 // Lower = faster game. Updates game logic every 5 render frames.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ====================================================================
 * FRAME BUFFER
 * ==================================================================== */
uint8_t frame_buffer[MATRIX_ROWS][MATRIX_COLS] = {0};

/* ====================================================================
 * RENDER STATE
 * ==================================================================== */
static int last_anode = -1;
static int last_cathode = -1;

/* ====================================================================
 * GAME STATE
 * ==================================================================== */
static uint8_t bricks[BRICK_ROWS][MATRIX_COLS];
static int ball_x, ball_y;
static int ball_vx, ball_vy; // velocity
static int paddle_x;
static uint8_t game_over = 0;
static uint32_t score = 0;
static uint32_t game_tick_counter = 0;

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
void set_all_pins_input(void);
void turn_off_led(int anode_index, int cathode_index);
void light_led_fast(int anode_index, int cathode_index);
void render_frame(void);

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

/*
 * All of your existing helper functions are here.
 * (DWT_Delay, pin definitions, and register
 * functions are all preserved).
 *
 * I've added init_button() and read_button()
 * for your new input.
*/

#include "main.h" // Should be present from CubeIDE
#include <stdint.h>
/* --- Add these includes for DWT --- */
#include "core_cm3.h" // For DWT, CoreDebug

/* ====================================================================
 * 1. PIN DEFINITIONS (From User)
 * ==================================================================== */

#define NUM_PINS 10

// Structure to hold pin information
typedef struct {
    GPIO_TypeDef* port; // GPIO Port (e.g., GPIOA, GPIOB)
    uint8_t       pin_num;  // Pin number (0-15)
} GpioPin;

// Array mapping your 20 pins to their ports and numbers
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
//    {GPIOA, 3},  // Pin 10: PA3
//    {GPIOA, 4},  // Pin 11: PA4
//    {GPIOA, 5},  // Pin 12: PA5
//    {GPIOA, 6},  // Pin 13: PA6
//    {GPIOA, 7},  // Pin 14: PA7
//    {GPIOB, 0},  // Pin 15: PB0
//    {GPIOB, 1},  // Pin 16: PB1
//    {GPIOB, 2},  // Pin 17: PB2
//    {GPIOB, 10}, // Pin 18: PB10
//    {GPIOB, 11}  // Pin 19: PB11
};


/* ====================================================================
 * 2. LOW-LEVEL GPIO REGISTER FUNCTIONS (From User)
 * ==================================================================== */

void enable_gpio_clocks(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_IOPDEN;
}

void set_pin_input(GPIO_TypeDef* port, uint8_t pin_num) {
    if (pin_num < 8) { // Use CRL register
        uint8_t shift = pin_num * 4;
        port->CRL &= ~(0xF << shift);
        port->CRL |= (0x0 << shift);  // MODE=00, CNF=00 (Analog Input)
    } else { // Use CRH register
        uint8_t shift = (pin_num - 8) * 4;
        port->CRH &= ~(0xF << shift);
        port->CRH |= (0x0 << shift);  // MODE=00, CNF=00 (Analog Input)
    }
}

void set_pin_output(GPIO_TypeDef* port, uint8_t pin_num) {
    if (pin_num < 8) { // Use CRL register
        uint8_t shift = pin_num * 4;
        port->CRL &= ~(0xF << shift);
        port->CRL |= (0x3 << shift);  // MODE=11 (50MHz), CNF=00 (GP Output PP)
    } else { // Use CRH register
        uint8_t shift = (pin_num - 8) * 4;
        port->CRH &= ~(0xF << shift);
        port->CRH |= (0x3 << shift);  // MODE=11 (50MHz), CNF=00 (GP Output PP)
    }
}

void set_pin_high(GPIO_TypeDef* port, uint8_t pin_num) {
    port->BSRR = (1 << pin_num);
}

void set_pin_low(GPIO_TypeDef* port, uint8_t pin_num) {
    port->BRR = (1 << pin_num);
}

/* ====================================================================
 * 3. TIMING FUNCTIONS (From User)
 * ==================================================================== */

void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

void DWT_Delay_us(volatile uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles_per_us = (HAL_RCC_GetHCLKFreq() / 1000000);
    uint32_t count = us * cycles_per_us;
    while ((DWT->CYCCNT - start) < count);
}


/* ====================================================================
 * 4. CHARLIEPLEXING LOGIC (From User)
 * ==================================================================== */

void set_all_pins_input(void) {
    for (int i = 0; i < NUM_PINS; i++) {
        set_pin_input(matrix_pins[i].port, matrix_pins[i].pin_num);
    }
}

void turn_off_led(int anode_index, int cathode_index) {
    if (anode_index < 0 || cathode_index < 0 || anode_index >= NUM_PINS || cathode_index >= NUM_PINS) return;
    set_pin_input(matrix_pins[anode_index].port, matrix_pins[anode_index].pin_num);
    set_pin_input(matrix_pins[cathode_index].port, matrix_pins[cathode_index].pin_num);
}


void light_led_fast(int anode_index, int cathode_index) {
    GPIO_TypeDef* anode_port = matrix_pins[anode_index].port;
    uint8_t       anode_pin  = matrix_pins[anode_index].pin_num;
    GPIO_TypeDef* cathode_port = matrix_pins[cathode_index].port;
    uint8_t       cathode_pin  = matrix_pins[cathode_index].pin_num;

    set_pin_output(anode_port, anode_pin);
    set_pin_high(anode_port, anode_pin);
    set_pin_output(cathode_port, cathode_pin);
    set_pin_low(cathode_port, cathode_pin);
}

/* ====================================================================
 * 5. TEST FUNCTIONS (From User, unused in game)
 * ==================================================================== */
void test_all_leds(void) {
    enable_gpio_clocks();
    while (1) {
        for (int i = 0; i < NUM_PINS; i++) {
            for (int j = 0; j < NUM_PINS; j++) {
                if (i == j) continue;
                set_all_pins_input();
                light_led_fast(i, j);
                HAL_Delay(5);
            }
        }
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

  // 2. Initialize the high-precision delay timer
  DWT_Delay_Init();

  // 3. Initialize the button on PB12
  init_button();

  // 4. Initialize the game state (bricks, ball, paddle)
  game_init();

  // 5. Set all pins to input (Hi-Z) to start
  set_all_pins_input();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// 1. Render the current frame_buffer to the LEDs
	render_frame();

	// 2. Calculate the next game state (and update the frame_buffer)
	update_game_state(game_tick++);
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
  /* This function is from CubeMX. We will add our
   * button init code separately in init_button()
   */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ====================================================================
 * BUTTON HARDWARE FUNCTIONS
 * ==================================================================== */

/**
 * @brief Configures PB12 as an input with an internal pull-up resistor.
 */
void init_button(void)
{
    // enable_gpio_clocks() already enabled GPIOB, but we can be safe
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

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


/* ====================================================================
 * GAME LOGIC FUNCTIONS
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
    // 1. Slow down the game logic
    game_tick_counter++;
    if (game_tick_counter < GAME_SPEED_DIVIDER)
    {
        return; // Not time for a logic update yet
    }
    game_tick_counter = 0; // Reset counter


    // 2. Check for Game Over / Restart
    if (game_over)
    {
        if (read_button())
        {
            game_init();
        }
        draw_game_over_screen(tick); // Show flashing 'G'
        return;
    }

    // 3. Update Paddle Position
    // Button pressed = move right, Not pressed = move left
    if (read_button())
    {
        paddle_x++;
    }
    else
    {
        paddle_x--;
    }

    // 4. Constrain paddle to screen
    if (paddle_x < 0) paddle_x = 0;
    if (paddle_x > (MATRIX_COLS - PADDLE_WIDTH))
    {
        paddle_x = (MATRIX_COLS - PADDLE_WIDTH);
    }

    // 5. Update Ball Position
    ball_x += ball_vx;
    ball_y += ball_vy;

    // 6. Check Ball Collisions
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

    // 7. Draw the new game state to the frame_buffer
    draw_game_to_framebuffer();
}

/**
 * @brief Draws all game elements (bricks, ball, paddle) into the frame_buffer.
 */
void draw_game_to_framebuffer(void)
{
    // 1. Clear the frame buffer
    memset(frame_buffer, 0, sizeof(frame_buffer));

    // 2. Draw bricks
    for (int r = 0; r < BRICK_ROWS; r++)
    {
        for (int c = 0; c < MATRIX_COLS; c++)
        {
            if (bricks[r][c])
            {
                frame_buffer[r][c] = 1;
            }
        }
    }

    // 3. Draw paddle
    for (int i = 0; i < PADDLE_WIDTH; i++)
    {
        if((paddle_x + i) < MATRIX_COLS) // Safety check
        {
            frame_buffer[PADDLE_Y][paddle_x + i] = 1;
        }
    }

    // 4. Draw ball
    frame_buffer[ball_y][ball_x] = 1;
}

/**
 * @brief Draws a flashing "G" for "Game Over".
 */
void draw_game_over_screen(uint32_t tick)
{
    memset(frame_buffer, 0, sizeof(frame_buffer));

    // Flash the 'G' every 20 logic ticks
    if ((tick / 20) % 2)
    {
        int x = 6;
        int y = 7;
        frame_buffer[y+0][x+1] = 1; frame_buffer[y+0][x+2] = 1; frame_buffer[y+0][x+3] = 1;
        frame_buffer[y+1][x+0] = 1;
        frame_buffer[y+2][x+0] = 1; frame_buffer[y+2][x+3] = 1; frame_buffer[y+2][x+4] = 1;
        frame_buffer[y+3][x+0] = 1; frame_buffer[y+3][x+4] = 1;
        frame_buffer[y+4][x+1] = 1; frame_buffer[y+4][x+2] = 1; frame_buffer[y+4][x+3] = 1; frame_buffer[y+4][x+4] = 1;
    }
}


/* ====================================================================
 * FRAME RENDER FUNCTION (From Previous Step)
 * ==================================================================== */

/**
 * @brief Renders the global 'frame_buffer' to the matrix using POV.
 * This function is designed to be called continuously in the main loop.
 */
void render_frame(void)
{
    // This is the "on" time for each LED in microseconds.
    // 39us gives ~67Hz refresh rate for all 380 LEDs.
    const uint32_t delay_per_led = 100;

    // Iterate over every pixel in our 20x19 logical frame
    for (int r = 0; r < MATRIX_ROWS; r++)
    {
        for (int c = 0; c < MATRIX_COLS; c++)
        {
            // 1. Turn OFF the *previous* LED
            turn_off_led(last_anode, last_cathode);

            // 2. Check if the current pixel should be ON
            if (frame_buffer[r][c])
            {
                // 3. If YES, calculate the physical pin mapping
                int anode_pin = r;
                int cathode_pin = (c < r) ? c : (c + 1);

                // 4. Light the new LED
                light_led_fast(anode_pin, cathode_pin);

                // 5. Remember which LED is on
                last_anode = anode_pin;
                last_cathode = cathode_pin;
            }
            else
            {
                // 6. If NO, no LED is on, so "remember" nothing
                last_anode = -1;
                last_cathode = -1;
            }

            // 7. Hold this state (either on or off) for one time slot
            DWT_Delay_us(delay_per_led);
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
